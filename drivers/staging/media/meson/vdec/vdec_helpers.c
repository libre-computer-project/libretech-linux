// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018 BayLibre, SAS
 * Author: Maxime Jourdan <mjourdan@baylibre.com>
 */

#include <linux/gcd.h>
#include <crypto/sha2.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-dma-contig.h>

#include "vdec_helpers.h"
#include "vdec_debugfs.h"
#include "dos_regs.h"
#include "hevc_regs.h"
#include "vdec_hevc.h"
#include "codec_hevc_common.h"

#define NUM_CANVAS_NV12 2
#define NUM_CANVAS_YUV420 3

/*
 * On G12A+, the VDEC has a per-canvas block-mode shadow register that must
 * be written after each meson_canvas_config().  Without this, the decoder
 * reads the wrong block layout and corrupts the Y plane.
 *
 * Bit 11: canvas_blk32_wr (write-enable)
 * Bit 10: canvas_blk32 (0 = linear, 1 = 32x32)
 * Bit  8: canvas_index_wr (write-enable)
 * Bits 7:0: canvas_index
 */
static void canvas_blk32_write(struct amvdec_core *core, u8 canvas_idx,
			       u32 blkmode)
{
	if (core->platform->revision >= VDEC_REVISION_G12A)
		amvdec_write_dos(core, VDEC_ASSIST_CANVAS_BLK32,
				 CANVAS_BLK32_WR | (blkmode << CANVAS_BLK32_BIT) |
				 CANVAS_INDEX_WR | canvas_idx);
}

/*
 * Canvas endianness.  SWAP64 (endian=7) is required on all platforms
 * for correct byte ordering in NV12/YUV420 output.
 * Tested: endian=0 on G12A+ corrupts UV planes (PSNR 7dB).
 *
 * For NV21 (VU byte order), the UV canvas uses endian=6 (SWAP64
 * without SWAP16).  SWAP64 (0x7) does a full 64-bit byte reversal
 * (swap16+swap32+swap64).  Removing SWAP16 (bit 0) leaves adjacent
 * byte pairs unswapped, effectively swapping U and V within each
 * interleaved UV pair.
 */
static u32 canvas_endian(struct amvdec_core *core)
{
	return MESON_CANVAS_ENDIAN_SWAP64;
}

static u32 canvas_endian_uv_swap(struct amvdec_core *core)
{
	return MESON_CANVAS_ENDIAN_SWAP64 ^ MESON_CANVAS_ENDIAN_SWAP16;
}

u32 amvdec_read_dos(struct amvdec_core *core, u32 reg)
{
	return readl_relaxed(core->dos_base + reg);
}
EXPORT_SYMBOL_GPL(amvdec_read_dos);

void amvdec_write_dos(struct amvdec_core *core, u32 reg, u32 val)
{
	writel_relaxed(val, core->dos_base + reg);
}
EXPORT_SYMBOL_GPL(amvdec_write_dos);

void amvdec_write_dos_bits(struct amvdec_core *core, u32 reg, u32 val)
{
	amvdec_write_dos(core, reg, amvdec_read_dos(core, reg) | val);
}
EXPORT_SYMBOL_GPL(amvdec_write_dos_bits);

void amvdec_clear_dos_bits(struct amvdec_core *core, u32 reg, u32 val)
{
	amvdec_write_dos(core, reg, amvdec_read_dos(core, reg) & ~val);
}
EXPORT_SYMBOL_GPL(amvdec_clear_dos_bits);

u32 amvdec_read_parser(struct amvdec_core *core, u32 reg)
{
	return readl_relaxed(core->esparser_base + reg);
}
EXPORT_SYMBOL_GPL(amvdec_read_parser);

void amvdec_write_parser(struct amvdec_core *core, u32 reg, u32 val)
{
	writel_relaxed(val, core->esparser_base + reg);
}
EXPORT_SYMBOL_GPL(amvdec_write_parser);

/*
 * FBC compressed body size per block.
 *
 * 8-bit: 3200 bytes per 64x32 block (mem_saving_mode, SAO_CTRL5 BIT(9)).
 * 10-bit: 4096 bytes per 64x32 block.
 *
 * The HEVC codec init path sets SAO_CTRL5 BIT(9) unconditionally, and
 * codec_hevc_setup_decode_head programs HEVC_CM_BODY_LENGTH with the
 * matching 3200/4096 constant per session.  The allocator must match
 * what the firmware writes or we corrupt the neighbor buffer.
 *
 * Saves ~20% FBC body bytes for 8-bit streams (the common case for
 * VP9 Profile 0 + HEVC Main); 10-bit callers (VP9 Profile 2 -- aborted
 * today per KI-058, HEVC Main10) pass is_10bit=true to get the
 * conservative sizing.
 */
u32 amvdec_am21c_body_size(u32 width, u32 height, bool is_10bit)
{
	u32 width_64 = ALIGN(width, 64) / 64;
	u32 height_32 = ALIGN(height, 32) / 32;
	u32 bytes_per_block = is_10bit ? SZ_4K : 3200;

	return bytes_per_block * width_64 * height_32;
}
EXPORT_SYMBOL_GPL(amvdec_am21c_body_size);

/* 32 bytes per 128x64 block */
u32 amvdec_am21c_head_size(u32 width, u32 height)
{
	u32 width_128 = ALIGN(width, 128) / 128;
	u32 height_64 = ALIGN(height, 64) / 64;

	return 32 * width_128 * height_64;
}
EXPORT_SYMBOL_GPL(amvdec_am21c_head_size);

u32 amvdec_am21c_size(u32 width, u32 height, bool is_10bit)
{
	return ALIGN(amvdec_am21c_body_size(width, height, is_10bit) +
		     amvdec_am21c_head_size(width, height), SZ_64K);
}
EXPORT_SYMBOL_GPL(amvdec_am21c_size);

static int canvas_alloc(struct amvdec_session *sess, u8 *canvas_id)
{
	int ret;

	if (sess->canvas_num >= MAX_CANVAS) {
		dev_err(sess->core->dev, "Reached max number of canvas\n");
		return -ENOMEM;
	}

	ret = meson_canvas_alloc(sess->core->canvas, canvas_id);
	if (ret)
		return ret;

	sess->canvas_alloc[sess->canvas_num++] = *canvas_id;
	return 0;
}

static int set_canvas_yuv420m(struct amvdec_session *sess,
			      struct vb2_buffer *vb, u32 width,
			      u32 height, u32 reg)
{
	struct amvdec_core *core = sess->core;
	u8 canvas_id[NUM_CANVAS_YUV420]; /* Y U V */
	dma_addr_t buf_paddr[NUM_CANVAS_YUV420]; /* Y U V */
	int ret, i;

	for (i = 0; i < NUM_CANVAS_YUV420; ++i) {
		ret = canvas_alloc(sess, &canvas_id[i]);
		if (ret)
			return ret;

		buf_paddr[i] =
		    vb2_dma_contig_plane_dma_addr(vb, i);
	}

	/* Y plane */
	meson_canvas_config(core->canvas, canvas_id[0], buf_paddr[0],
			    width, height, MESON_CANVAS_WRAP_NONE,
			    MESON_CANVAS_BLKMODE_LINEAR,
			    canvas_endian(core));
	canvas_blk32_write(core, canvas_id[0], MESON_CANVAS_BLKMODE_LINEAR);

	/* U plane */
	meson_canvas_config(core->canvas, canvas_id[1], buf_paddr[1],
			    width / 2, height / 2, MESON_CANVAS_WRAP_NONE,
			    MESON_CANVAS_BLKMODE_LINEAR,
			    canvas_endian(core));
	canvas_blk32_write(core, canvas_id[1], MESON_CANVAS_BLKMODE_LINEAR);

	/* V plane */
	meson_canvas_config(core->canvas, canvas_id[2], buf_paddr[2],
			    width / 2, height / 2, MESON_CANVAS_WRAP_NONE,
			    MESON_CANVAS_BLKMODE_LINEAR,
			    canvas_endian(core));
	canvas_blk32_write(core, canvas_id[2], MESON_CANVAS_BLKMODE_LINEAR);

	amvdec_write_dos(core, reg,
			 ((canvas_id[2]) << 16) |
			 ((canvas_id[1]) << 8)  |
			 (canvas_id[0]));

	return 0;
}

static int set_canvas_nv12(struct amvdec_session *sess,
			   struct vb2_buffer *vb, u32 width,
			   u32 height, u32 reg, bool swap_uv)
{
	struct amvdec_core *core = sess->core;
	u8 canvas_id[NUM_CANVAS_NV12]; /* Y U/V */
	dma_addr_t buf_paddr;
	u32 uv_endian = swap_uv ? canvas_endian_uv_swap(core) :
				   canvas_endian(core);
	int ret, i;

	buf_paddr = vb2_dma_contig_plane_dma_addr(vb, 0);

	for (i = 0; i < NUM_CANVAS_NV12; ++i) {
		ret = canvas_alloc(sess, &canvas_id[i]);
		if (ret)
			return ret;
	}

	/* Y plane */
	meson_canvas_config(core->canvas, canvas_id[0], buf_paddr,
			    width, height, MESON_CANVAS_WRAP_NONE,
			    MESON_CANVAS_BLKMODE_LINEAR,
			    canvas_endian(core));
	canvas_blk32_write(core, canvas_id[0], MESON_CANVAS_BLKMODE_LINEAR);

	/* U/V plane at offset width*height */
	meson_canvas_config(core->canvas, canvas_id[1],
			    buf_paddr + (width * height),
			    width, height / 2, MESON_CANVAS_WRAP_NONE,
			    MESON_CANVAS_BLKMODE_LINEAR,
			    uv_endian);
	canvas_blk32_write(core, canvas_id[1], MESON_CANVAS_BLKMODE_LINEAR);

	amvdec_write_dos(core, reg,
			 ((canvas_id[1]) << 16) |
			 ((canvas_id[1]) << 8)  |
			 (canvas_id[0]));

	return 0;
}

static int set_canvas_nv12m(struct amvdec_session *sess,
			    struct vb2_buffer *vb, u32 width,
			    u32 height, u32 reg, bool swap_uv)
{
	struct amvdec_core *core = sess->core;
	u8 canvas_id[NUM_CANVAS_NV12]; /* Y U/V */
	dma_addr_t buf_paddr[NUM_CANVAS_NV12]; /* Y U/V */
	u32 uv_endian = swap_uv ? canvas_endian_uv_swap(core) :
				   canvas_endian(core);
	int ret, i;

	for (i = 0; i < NUM_CANVAS_NV12; ++i) {
		ret = canvas_alloc(sess, &canvas_id[i]);
		if (ret)
			return ret;

		buf_paddr[i] =
		    vb2_dma_contig_plane_dma_addr(vb, i) +
		    vb->planes[i].data_offset;
	}

	/*
	 * Single-buffer NV12M import (Android CCodec / v4l2_codec2): the gralloc
	 * output pool allocates ONE contiguous NV12 buffer and imports it for BOTH
	 * MPLANE planes with data_offset=0 (board-measured), so plane 1 (U/V)
	 * resolves to the SAME DMA address as plane 0 (Y).  Programming the U/V
	 * canvas at the Y base overlaps the two planes, so the hardware would write
	 * chroma on top of luma.  Place U/V at the contiguous single-buffer offset
	 * (Y stride*height), the same layout the single-plane set_canvas_nv12 uses.
	 * Genuine 2-plane imports (MMAP, or a HAL that allocates separate planes /
	 * sets a proper data_offset) have distinct addresses and are unaffected.
	 * (Note: this fixes the aliased-plane layout only; it is NOT what caused the
	 * Android 0-frames -- that was the DPB overflow, see codec_h264.c
	 * H264_FW_MAX_DPB.)
	 */
	if (buf_paddr[1] == buf_paddr[0])
		buf_paddr[1] = buf_paddr[0] + (u64)width * height;

	/* Y plane */
	meson_canvas_config(core->canvas, canvas_id[0], buf_paddr[0],
			    width, height, MESON_CANVAS_WRAP_NONE,
			    MESON_CANVAS_BLKMODE_LINEAR,
			    canvas_endian(core));
	canvas_blk32_write(core, canvas_id[0], MESON_CANVAS_BLKMODE_LINEAR);

	/* U/V plane */
	meson_canvas_config(core->canvas, canvas_id[1], buf_paddr[1],
			    width, height / 2, MESON_CANVAS_WRAP_NONE,
			    MESON_CANVAS_BLKMODE_LINEAR,
			    uv_endian);
	canvas_blk32_write(core, canvas_id[1], MESON_CANVAS_BLKMODE_LINEAR);

	amvdec_write_dos(core, reg,
			 ((canvas_id[1]) << 16) |
			 ((canvas_id[1]) << 8)  |
			 (canvas_id[0]));

	return 0;
}

/*
 * Configure ONE firmware canvas register (reg) to a raw contiguous NV12 paddr.
 * Mirror of set_canvas_nv12 (single-plane Y + UV-at-Y_size layout) but takes a
 * raw dma_addr_t instead of a vb2 buffer.  Used by amvdec_set_scratch_canvases
 * for the MPEG-1/2 non-zero-copy bridge's bridge-owned scratch pool, where the
 * firmware decodes into bridge DMA memory (not V4L2 buffers) so > 8 V4L2 CAPTURE
 * buffers can be allocated against the firmware's fixed 8 canvases.
 */
static int set_canvas_nv12_paddr(struct amvdec_session *sess,
				 dma_addr_t buf_paddr, u32 width,
				 u32 height, u32 reg)
{
	struct amvdec_core *core = sess->core;
	u8 canvas_id[NUM_CANVAS_NV12]; /* Y U/V */
	int ret, i;

	for (i = 0; i < NUM_CANVAS_NV12; ++i) {
		ret = canvas_alloc(sess, &canvas_id[i]);
		if (ret)
			return ret;
	}

	/* Y plane */
	meson_canvas_config(core->canvas, canvas_id[0], buf_paddr,
			    width, height, MESON_CANVAS_WRAP_NONE,
			    MESON_CANVAS_BLKMODE_LINEAR,
			    canvas_endian(core));
	canvas_blk32_write(core, canvas_id[0], MESON_CANVAS_BLKMODE_LINEAR);

	/* U/V plane at offset width*height */
	meson_canvas_config(core->canvas, canvas_id[1],
			    buf_paddr + (width * height),
			    width, height / 2, MESON_CANVAS_WRAP_NONE,
			    MESON_CANVAS_BLKMODE_LINEAR,
			    canvas_endian(core));
	canvas_blk32_write(core, canvas_id[1], MESON_CANVAS_BLKMODE_LINEAR);

	amvdec_write_dos(core, reg,
			 ((canvas_id[1]) << 16) |
			 ((canvas_id[1]) << 8)  |
			 (canvas_id[0]));

	return 0;
}

/*
 * Program a set of firmware canvas registers from raw bridge-owned scratch
 * paddrs (NON-ZERO-COPY).  Each canvas register at regs[i] is configured to
 * point at paddrs[i] in the contiguous NV12 layout (Y stride = ALIGN(w,64), Y
 * height = ALIGN(h,32), UV at Y_size).  This is the geometry set_canvas_nv12
 * programs for a real V4L2 buffer, so the firmware's MC output lands identically
 * and the copy-out geometry (scratch -> V4L2) is well-defined.
 */
int amvdec_set_scratch_canvases(struct amvdec_session *sess,
				u32 regs[], dma_addr_t paddrs[], u32 count)
{
	u32 width = ALIGN(sess->width, 64);
	u32 height = ALIGN(sess->height, 32);
	u32 i;
	int ret;

	for (i = 0; i < count; i++) {
		ret = set_canvas_nv12_paddr(sess, paddrs[i], width, height,
					    regs[i]);
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(amvdec_set_scratch_canvases);

int amvdec_set_canvases(struct amvdec_session *sess,
			u32 reg_base[], u32 reg_num[])
{
	struct v4l2_m2m_buffer *buf;
	u32 pixfmt = sess->pixfmt_cap;
	u32 width = ALIGN(sess->width, 64);
	u32 height = ALIGN(sess->height, 32);
	u32 reg_cur;
	u32 reg_num_cur = 0;
	u32 reg_base_cur = 0;
	int i = 0;
	int ret;

	v4l2_m2m_for_each_dst_buf(sess->m2m_ctx, buf) {
		if (!reg_base[reg_base_cur])
			return -EINVAL;

		reg_cur = reg_base[reg_base_cur] + reg_num_cur * 4;

		switch (pixfmt) {
		case V4L2_PIX_FMT_NV12:
			ret = set_canvas_nv12(sess, &buf->vb.vb2_buf, width,
					      height, reg_cur, false);
			if (ret)
				return ret;
			break;
		case V4L2_PIX_FMT_NV21:
			ret = set_canvas_nv12(sess, &buf->vb.vb2_buf, width,
					      height, reg_cur, true);
			if (ret)
				return ret;
			break;
		case V4L2_PIX_FMT_NV12M:
			ret = set_canvas_nv12m(sess, &buf->vb.vb2_buf, width,
					       height, reg_cur, false);
			if (ret)
				return ret;
			break;
		case V4L2_PIX_FMT_NV21M:
			ret = set_canvas_nv12m(sess, &buf->vb.vb2_buf, width,
					       height, reg_cur, true);
			if (ret)
				return ret;
			break;
		case V4L2_PIX_FMT_YUV420M:
			ret = set_canvas_yuv420m(sess, &buf->vb.vb2_buf, width,
						 height, reg_cur);
			if (ret)
				return ret;
			break;
		default:
			dev_err(sess->core->dev, "Unsupported pixfmt %08X\n",
				pixfmt);
			return -EINVAL;
		}

		reg_num_cur++;
		if (reg_num_cur >= reg_num[reg_base_cur]) {
			reg_base_cur++;
			reg_num_cur = 0;
		}

		sess->fw_idx_to_vb2_idx[i++] = buf->vb.vb2_buf.index;
	}

	/*
	 * Stateless (P4.9): a Request-API client (gst v4l2slmpeg2dec) QBUFs its
	 * CAPTURE buffers incrementally (1/frame), so the loop above -- which only
	 * sees QUEUED buffers -- may have bound just buffer 0 at the deferred
	 * start. The remaining buffers then have no canvas + no fw_idx_to_vb2_idx
	 * entry, so pre_queue's lookup misses them (fw_found=0), MREG_BUFFERIN
	 * re-hands buffer 0 (the reference) and P-frames stall (P4.8). All
	 * CREATE_BUFS-allocated buffers exist before STREAMON, so bind a canvas +
	 * mapping for every allocated buffer not already covered. Stateful is
	 * untouched (all its DPB buffers are queued at start).
	 */
	if (amvdec_is_stateless(sess)) {
		struct vb2_queue *cap_q =
			v4l2_m2m_get_vq(sess->m2m_ctx,
					V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
		unsigned int idx, nbufs = cap_q ? vb2_get_num_buffers(cap_q) : 0;

		for (idx = 0; idx < nbufs; idx++) {
			struct vb2_buffer *vb = vb2_get_buffer(cap_q, idx);
			bool mapped = false;
			int k;

			if (!vb)
				continue;
			for (k = 0; k < i; k++)
				if (sess->fw_idx_to_vb2_idx[k] == vb->index) {
					mapped = true;
					break;
				}
			if (mapped)
				continue;
			if (i >= (int)ARRAY_SIZE(sess->fw_idx_to_vb2_idx))
				break;
			if (!reg_base[reg_base_cur])
				return -EINVAL;
			reg_cur = reg_base[reg_base_cur] + reg_num_cur * 4;

			switch (pixfmt) {
			case V4L2_PIX_FMT_NV12:
				ret = set_canvas_nv12(sess, vb, width, height,
						      reg_cur, false);
				break;
			case V4L2_PIX_FMT_NV21:
				ret = set_canvas_nv12(sess, vb, width, height,
						      reg_cur, true);
				break;
			case V4L2_PIX_FMT_NV12M:
				ret = set_canvas_nv12m(sess, vb, width, height,
						       reg_cur, false);
				break;
			case V4L2_PIX_FMT_NV21M:
				ret = set_canvas_nv12m(sess, vb, width, height,
						       reg_cur, true);
				break;
			case V4L2_PIX_FMT_YUV420M:
				ret = set_canvas_yuv420m(sess, vb, width, height,
							 reg_cur);
				break;
			default:
				return -EINVAL;
			}
			if (ret)
				return ret;

			reg_num_cur++;
			if (reg_num_cur >= reg_num[reg_base_cur]) {
				reg_base_cur++;
				reg_num_cur = 0;
			}

			sess->fw_idx_to_vb2_idx[i++] = vb->index;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(amvdec_set_canvases);

/*
 * Bind ONE CAPTURE buffer's canvas + firmware-index mapping on demand.
 *
 * amvdec_set_canvases() runs once at start and only sees the CAPTURE buffers
 * allocated by then. A stateless Request-API client may CREATE_BUFS more
 * CAPTURE buffers AFTER STREAMON (the libva-v4l2-request shim does: it grows
 * the pool 1 buffer at a time as frames are decoded), so those buffers have no
 * canvas and no fw_idx_to_vb2_idx entry -- pre_queue's lookup misses them
 * (fw_found=0), MREG_BUFFERIN re-hands buffer 0 and the decode fails. Call this
 * from a codec's pre_queue when the lookup misses: it assigns the next free
 * firmware slot, programs that slot's canvas register (reg_base + fw_idx*4),
 * records the mapping, and bumps num_dst_bufs. Returns the firmware index
 * (existing if already mapped, newly assigned otherwise), or <0 on error.
 *
 * reg_base is the codec's canvas-ID register block (AV_SCRATCH_0 for the
 * VDEC_1 MPEG-1/2 path), max_slots its capacity (8 for AV_SCRATCH_0..7); a
 * canvas-per-buffer layout (MPEG-1/2) is assumed.
 */
int amvdec_bind_dst_canvas(struct amvdec_session *sess, struct vb2_buffer *vb,
			   u32 reg_base, u32 max_slots)
{
	u32 pixfmt = sess->pixfmt_cap;
	u32 width = ALIGN(sess->width, 64);
	u32 height = ALIGN(sess->height, 32);
	u32 fw_idx, reg_cur;
	int ret;

	for (fw_idx = 0; fw_idx < sess->num_dst_bufs; fw_idx++)
		if (sess->fw_idx_to_vb2_idx[fw_idx] == vb->index)
			return fw_idx;

	if (sess->num_dst_bufs >= max_slots ||
	    sess->num_dst_bufs >= ARRAY_SIZE(sess->fw_idx_to_vb2_idx))
		return -ENOSPC;

	fw_idx = sess->num_dst_bufs;
	reg_cur = reg_base + fw_idx * 4;

	switch (pixfmt) {
	case V4L2_PIX_FMT_NV12:
		ret = set_canvas_nv12(sess, vb, width, height, reg_cur, false);
		break;
	case V4L2_PIX_FMT_NV21:
		ret = set_canvas_nv12(sess, vb, width, height, reg_cur, true);
		break;
	case V4L2_PIX_FMT_NV12M:
		ret = set_canvas_nv12m(sess, vb, width, height, reg_cur, false);
		break;
	case V4L2_PIX_FMT_NV21M:
		ret = set_canvas_nv12m(sess, vb, width, height, reg_cur, true);
		break;
	case V4L2_PIX_FMT_YUV420M:
		ret = set_canvas_yuv420m(sess, vb, width, height, reg_cur);
		break;
	default:
		dev_err(sess->core->dev, "Unsupported pixfmt %08X\n", pixfmt);
		return -EINVAL;
	}
	if (ret)
		return ret;

	sess->fw_idx_to_vb2_idx[fw_idx] = vb->index;
	sess->num_dst_bufs = fw_idx + 1;
	return fw_idx;
}
EXPORT_SYMBOL_GPL(amvdec_bind_dst_canvas);

int amvdec_add_ts(struct amvdec_session *sess, u64 ts,
		  struct v4l2_timecode tc, u32 offset, u32 vbuf_flags)
{
	struct amvdec_timestamp *new_ts;
	unsigned long flags;

	new_ts = kzalloc(sizeof(*new_ts), GFP_KERNEL);
	if (!new_ts)
		return -ENOMEM;

	new_ts->ts = ts;
	new_ts->tc = tc;
	new_ts->offset = offset;
	new_ts->flags = vbuf_flags;

	spin_lock_irqsave(&sess->ts_spinlock, flags);
	list_add_tail(&new_ts->list, &sess->timestamps);
	spin_unlock_irqrestore(&sess->ts_spinlock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(amvdec_add_ts);

void amvdec_remove_ts(struct amvdec_session *sess, u64 ts)
{
	struct amvdec_timestamp *tmp;
	unsigned long flags;

	spin_lock_irqsave(&sess->ts_spinlock, flags);
	list_for_each_entry(tmp, &sess->timestamps, list) {
		if (tmp->ts == ts) {
			list_del(&tmp->list);
			kfree(tmp);
			goto unlock;
		}
	}
	dev_warn(sess->core->dev_dec,
		 "Couldn't remove buffer with timestamp %llu from list\n", ts);

unlock:
	spin_unlock_irqrestore(&sess->ts_spinlock, flags);
}
EXPORT_SYMBOL_GPL(amvdec_remove_ts);

/*
 * amvdec_drop_stale_ts() - at an EOS drain, drop the timestamp records no
 * picture can still claim, and return how many remain.
 *
 * Every picture still to come out of a drained stream displays after the
 * last one delivered, so a record at or before sess->last_ts_done belongs to
 * an input that produced no picture of its own -- a codec-config buffer
 * (Android v4l2_codec2 sends SPS/PPS as bitstream 0, timestamp 0).  Left in
 * the FIFO, it is what a picture delivered without a bitstream offset is
 * stamped with.  Safe from atomic context.
 */
int amvdec_drop_stale_ts(struct amvdec_session *sess)
{
	struct amvdec_timestamp *tmp, *n;
	unsigned long flags;
	int left = 0;

	spin_lock_irqsave(&sess->ts_spinlock, flags);
	list_for_each_entry_safe(tmp, n, &sess->timestamps, list) {
		if (tmp->ts <= sess->last_ts_done) {
			list_del(&tmp->list);
			kfree(tmp);
		} else {
			left++;
		}
	}
	spin_unlock_irqrestore(&sess->ts_spinlock, flags);
	return left;
}
EXPORT_SYMBOL_GPL(amvdec_drop_stale_ts);

static void dst_buf_done(struct amvdec_session *sess,
			 struct vb2_v4l2_buffer *vbuf,
			 u32 field, u64 timestamp,
			 struct v4l2_timecode timecode, u32 flags)
{
	struct device *dev = sess->core->dev_dec;
	u32 output_size = amvdec_get_output_size(sess);

	switch (sess->pixfmt_cap) {
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
		vb2_set_plane_payload(&vbuf->vb2_buf, 0,
				      output_size + output_size / 2);
		break;
	case V4L2_PIX_FMT_NV12M:
	case V4L2_PIX_FMT_NV21M:
		vb2_set_plane_payload(&vbuf->vb2_buf, 0, output_size);
		vb2_set_plane_payload(&vbuf->vb2_buf, 1, output_size / 2);
		break;
	case V4L2_PIX_FMT_YUV420M:
		vb2_set_plane_payload(&vbuf->vb2_buf, 0, output_size);
		vb2_set_plane_payload(&vbuf->vb2_buf, 1, output_size / 4);
		vb2_set_plane_payload(&vbuf->vb2_buf, 2, output_size / 4);
		break;
	case V4L2_PIX_FMT_MESON_FBC08_SCATTER:
		vb2_set_plane_payload(&vbuf->vb2_buf, 0,
				      amvdec_am21c_size(sess->width,
							sess->height, false));
		break;
	case V4L2_PIX_FMT_MESON_FBC10_SCATTER:
		vb2_set_plane_payload(&vbuf->vb2_buf, 0,
				      amvdec_am21c_size(sess->width,
							sess->height, true));
		break;
	}

	vbuf->vb2_buf.timestamp = timestamp;
	vbuf->sequence = sess->sequence_cap++;
	vbuf->flags = flags;
	vbuf->timecode = timecode;

	if (sess->status == STATUS_NEEDS_RESUME) {
		/* Mark LAST for drained show frames during a source change */
		vbuf->flags |= V4L2_BUF_FLAG_LAST;
		sess->sequence_cap = 0;
	}

	dev_dbg(dev, "Buffer %u done, ts = %llu, flags = %08X\n",
		vbuf->vb2_buf.index, timestamp, flags);
	vbuf->field = field;
	v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_DONE);

	/* Universal per-frame decoded count (every codec passes through here)
	 * -- exposed via the irq_counters debugfs as a backpressure-free
	 * frame counter. */
	atomic_inc(&sess->core->debugfs.irq.frames_done);

	/* Buffer done probably means the vififo got freed */
	schedule_work(&sess->esparser_queue_work);
}

void amvdec_dst_buf_done(struct amvdec_session *sess,
			 struct vb2_v4l2_buffer *vbuf, u32 field)
{
	struct device *dev = sess->core->dev_dec;
	struct amvdec_timestamp *tmp;
	struct list_head *timestamps = &sess->timestamps;
	struct v4l2_timecode timecode;
	u64 timestamp;
	u32 vbuf_flags;
	unsigned long flags;

	spin_lock_irqsave(&sess->ts_spinlock, flags);
	if (list_empty(timestamps)) {
		dev_err(dev, "Buffer %u done but list is empty\n",
			vbuf->vb2_buf.index);

		v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
		spin_unlock_irqrestore(&sess->ts_spinlock, flags);
		return;
	}

	tmp = list_first_entry(timestamps, struct amvdec_timestamp, list);
	timestamp = tmp->ts;
	timecode = tmp->tc;
	vbuf_flags = tmp->flags;
	list_del(&tmp->list);
	kfree(tmp);
	spin_unlock_irqrestore(&sess->ts_spinlock, flags);

	dst_buf_done(sess, vbuf, field, timestamp, timecode, vbuf_flags);
	atomic_dec(&sess->esparser_queued_bufs);
	wake_up(&sess->esparser_queue_done_wq);
}
EXPORT_SYMBOL_GPL(amvdec_dst_buf_done);

void amvdec_dst_buf_done_ts(struct amvdec_session *sess,
			    struct vb2_v4l2_buffer *vbuf, u32 field,
			    u64 timestamp, struct v4l2_timecode timecode,
			    u32 flags)
{
	dst_buf_done(sess, vbuf, field, timestamp, timecode, flags);
}
EXPORT_SYMBOL_GPL(amvdec_dst_buf_done_ts);

void amvdec_dst_buf_done_offset(struct amvdec_session *sess,
				struct vb2_v4l2_buffer *vbuf,
				u32 offset, u32 field, bool allow_drop)
{
	struct device *dev = sess->core->dev_dec;
	struct amvdec_timestamp *match = NULL;
	struct amvdec_timestamp *tmp, *n;
	struct v4l2_timecode timecode = { 0 };
	u64 timestamp = 0;
	u32 vbuf_flags = 0;
	unsigned long flags;
	bool clean_match = false;

	spin_lock_irqsave(&sess->ts_spinlock, flags);

	/* Look for our vififo offset to get the corresponding timestamp. */
	list_for_each_entry_safe(tmp, n, &sess->timestamps, list) {
		if (tmp->offset > offset) {
			/*
			 * Delete any record that remained unused for 32 match
			 * checks
			 */
			if (tmp->used_count++ >= 32) {
				list_del(&tmp->list);
				kfree(tmp);
			}
			break;
		}

		match = tmp;
	}
	if (match)
		clean_match = true;

	if (!match && sess->wrap_count > 0) {
		/*
		 * VIFIFO wrap-around: esparser tracks offsets as logical
		 * monotonic values (raw_offset + wrap_count * vififo_size),
		 * but MREG_FRAME_OFFSET reports the raw ring-buffer position.
		 * Convert the raw offset to logical and retry the match.
		 */
		u32 logical = offset + sess->wrap_count * sess->vififo_size;

		list_for_each_entry_safe(tmp, n, &sess->timestamps, list) {
			if (tmp->offset > logical)
				break;
			match = tmp;
		}

		if (match) {
			clean_match = true;
			dev_dbg(dev,
				"Buffer %u: matched after wrap adjust (%08X -> %08X)\n",
				vbuf->vb2_buf.index, offset, logical);
		}
	}

	if (!match) {
		dev_dbg(dev, "Buffer %u done but can't match offset (%08X)\n",
			vbuf->vb2_buf.index, offset);
		/*
		 * Last resort: fall back to FIFO order (oldest entry).
		 * MREG_FRAME_OFFSET reports the VLD read pointer which can
		 * lag the esparser write pointer stored in the timestamp list,
		 * especially at low bitrates.  The FIFO fallback preserves
		 * approximate PTS ordering when offset matching fails.
		 *
		 * The monotonic clamp below prevents backwards-PTS drops
		 * even if the fallback cascade assigns entries out of order.
		 */
		if (!list_empty(&sess->timestamps)) {
			match = list_first_entry(&sess->timestamps,
						 struct amvdec_timestamp, list);
		} else {
			/*
			 * Timestamp list exhausted.  Synthesize a monotonically
			 * increasing timestamp so userspace does not discard
			 * this frame as out-of-order (PTS=0 drop).
			 */
			dev_dbg(dev,
				"Buffer %u: ts list empty at offset %08X, using synthetic ts (last=%llu)\n",
				vbuf->vb2_buf.index, offset, sess->last_ts_done);
			timestamp = sess->last_ts_done + 1;
		}
	}

	if (match) {
		timestamp = match->ts;
		timecode = match->tc;
		vbuf_flags = match->flags;
		list_del(&match->list);
		kfree(match);
	}
	spin_unlock_irqrestore(&sess->ts_spinlock, flags);

	/*
	 * Clamp to monotonically increasing PTS *only* for the heuristic
	 * FIFO-fallback / synthetic paths.  A clean offset match already
	 * carries the frame's true PTS; for B-frame (display-reordered)
	 * streams the firmware emits in decode order, so those true PTS are
	 * legitimately non-monotonic and userspace (GstVideoDecoder / ffmpeg)
	 * reorders them by PTS.  Clamping a clean match forced reordered
	 * frames to near-duplicate PTS (last_ts_done+1), which userspace then
	 * dropped as unmatched -- the B-frame frame-delivery deficit (~50/90
	 * vs the firmware's full 90 frames_done).  So clamp only when the PTS
	 * came from the fallback, where a backwards value is a heuristic
	 * artifact rather than real frame order.
	 */
	if (!clean_match && timestamp < sess->last_ts_done + 1)
		timestamp = sess->last_ts_done + 1;
	/*
	 * Track the high-water mark, not the last value: a clean match may be
	 * legitimately backwards (decode-order B-frame), and the fallback
	 * clamp baseline must stay at the max PTS seen so it never rewinds.
	 */
	if (timestamp > sess->last_ts_done)
		sess->last_ts_done = timestamp;

	dst_buf_done(sess, vbuf, field, timestamp, timecode, vbuf_flags);
	/*
	 * Decrement whether or not offset matched: the firmware produced a
	 * frame, consuming one queued source slot, regardless of whether we
	 * found a matching timestamp.  Without this, an offset mismatch on
	 * the last frame leaves esparser_queued_bufs stuck at 1, causing
	 * drain_work to time out after 5 seconds before signalling EOS.
	 */
	atomic_dec_if_positive(&sess->esparser_queued_bufs);
	wake_up(&sess->esparser_queue_done_wq);
}
EXPORT_SYMBOL_GPL(amvdec_dst_buf_done_offset);

void amvdec_dst_buf_done_idx(struct amvdec_session *sess,
			     u32 buf_idx, u32 offset, u32 field)
{
	struct vb2_v4l2_buffer *vbuf;

	vbuf = v4l2_m2m_dst_buf_remove_by_idx(sess->m2m_ctx,
					      sess->fw_idx_to_vb2_idx[buf_idx]);

	if (!vbuf) {
		/* Buffer is still in userspace -- defer delivery until
		 * it's re-queued.  The recycle thread will retry.
		 *
		 * This path is critical during H.264 drain: the EOS
		 * sequence flushes up to 16 reference frames from the
		 * DPB in a single ISR burst, but capture buffers are
		 * still in userspace being DQBUFed.  Without deferral,
		 * those frames are silently dropped (observed: 16/90
		 * frames delivered for 720p refs=16 bf=16 gop=300).
		 *
		 * Safe during teardown too: vdec_poweroff() halts the
		 * firmware (MPSR=0) before stopping the recycle thread,
		 * so no ISRs fire after halt.  Any frames deferred
		 * before halt are cleaned up by vdec_reset_pending_frames()
		 * in stop_streaming.
		 */
		struct amvdec_pending_frame *pf;
		unsigned long flags;

		pf = kmalloc(sizeof(*pf), GFP_ATOMIC);
		if (pf) {
			pf->buf_idx = buf_idx;
			pf->offset = offset;
			pf->field = field;
			/*
			 * irqsave: this path runs from the codec stall timer
			 * (softirq, e.g. the H.264 KI-091 harvest) as well as
			 * from threads, so a plain spin_lock here could spin on
			 * a lock its own CPU's interrupted thread holds (KI-170).
			 */
			spin_lock_irqsave(&sess->pending_frames_lock, flags);
			list_add_tail(&pf->list, &sess->pending_frames);
			spin_unlock_irqrestore(&sess->pending_frames_lock, flags);
			wake_up_interruptible(&sess->recycle_wq);
			dev_dbg(sess->core->dev_dec,
				"Buffer %u deferred (in userspace)\n",
				buf_idx);
		}
		return;
	}

	if (offset != -1)
		amvdec_dst_buf_done_offset(sess, vbuf, offset, field, true);
	else
		amvdec_dst_buf_done(sess, vbuf, field);
	sess->dst_delivered++;
}
EXPORT_SYMBOL_GPL(amvdec_dst_buf_done_idx);

void amvdec_flush_pending_frames(struct amvdec_session *sess)
{
	struct amvdec_pending_frame *pf, *n;
	struct vb2_v4l2_buffer *vbuf;
	unsigned long flags;
	LIST_HEAD(local);

	/*
	 * pending_frames_lock is also taken from the codec stall timer
	 * (softirq) via amvdec_dst_buf_done_idx(), so every taker must keep
	 * interrupts off while holding it (KI-170 self-deadlock).
	 */
	spin_lock_irqsave(&sess->pending_frames_lock, flags);
	list_splice_init(&sess->pending_frames, &local);
	spin_unlock_irqrestore(&sess->pending_frames_lock, flags);

	list_for_each_entry_safe(pf, n, &local, list) {
		vbuf = v4l2_m2m_dst_buf_remove_by_idx(sess->m2m_ctx,
			sess->fw_idx_to_vb2_idx[pf->buf_idx]);
		if (vbuf) {
			dev_dbg(sess->core->dev_dec,
				"FLUSH-DELIVER pending buf_idx %u vb2 %u\n",
				pf->buf_idx, sess->fw_idx_to_vb2_idx[pf->buf_idx]);
			if (pf->offset != -1)
				amvdec_dst_buf_done_offset(sess, vbuf,
					pf->offset, pf->field, true);
			else
				amvdec_dst_buf_done(sess, vbuf, pf->field);
			list_del(&pf->list);
			kfree(pf);
		} else {
			/* Still not available -- put back */
			spin_lock_irqsave(&sess->pending_frames_lock, flags);
			list_move_tail(&pf->list, &sess->pending_frames);
			spin_unlock_irqrestore(&sess->pending_frames_lock, flags);
		}
	}
}
EXPORT_SYMBOL_GPL(amvdec_flush_pending_frames);

void amvdec_set_par_from_dar(struct amvdec_session *sess,
			     u32 dar_num, u32 dar_den)
{
	u32 div;

	sess->pixelaspect.numerator = sess->height * dar_num;
	sess->pixelaspect.denominator = sess->width * dar_den;
	div = gcd(sess->pixelaspect.numerator, sess->pixelaspect.denominator);
	sess->pixelaspect.numerator /= div;
	sess->pixelaspect.denominator /= div;
}
EXPORT_SYMBOL_GPL(amvdec_set_par_from_dar);

void amvdec_src_change(struct amvdec_session *sess, u32 width,
		       u32 height, u32 dpb_size)
{
	static const struct v4l2_event ev = {
		.type = V4L2_EVENT_SOURCE_CHANGE,
		.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION };
	bool cap_fits;

	/* Reject bitstream dimensions exceeding platform limits.
	 * Without this, the driver allocates oversized capture buffers
	 * that exhaust CMA and crash the system.
	 */
	if (sess->fmt_out->max_width && width > sess->fmt_out->max_width) {
		dev_err(sess->core->dev,
			"Stream width %u exceeds max %u, aborting\n",
			width, sess->fmt_out->max_width);
		amvdec_abort(sess);
		return;
	}
	if (sess->fmt_out->max_height && height > sess->fmt_out->max_height) {
		dev_err(sess->core->dev,
			"Stream height %u exceeds max %u, aborting\n",
			height, sess->fmt_out->max_height);
		amvdec_abort(sess);
		return;
	}
	if (sess->fmt_out->min_width && width < sess->fmt_out->min_width) {
		dev_err(sess->core->dev,
			"Stream width %u below minimum %u, aborting\n",
			width, sess->fmt_out->min_width);
		amvdec_abort(sess);
		return;
	}

	/*
	 * CMA-budget product clamp for HEVC-engine codecs (HEVC, VP9).
	 * A stream with dimensions within per-dim caps but exceeding the
	 * probe-time CMA budget would call codec_hevc_setup_buffers with
	 * oversized requests, triggering __alloc_pages_noprof WARN + abort
	 * after the BUG print storm.  Abort up front with a clean error.
	 *
	 * PARTIAL: remove once IOMMU / scatter-gather removes the
	 * contiguous-CMA constraint (see cma-opt-sg in vp9/todo.md).
	 */
	/* Scatter-gather FBC body removes the contiguous-CMA budget as
	 * the binding constraint; let the allocation itself report failure
	 * (cleaner error path than a pre-emptive abort).
	 */
	if (sess->core->hevc_engine_cma_max_pixels && !codec_hevc_fbc_sg &&
	    (sess->fmt_out->pixfmt == V4L2_PIX_FMT_VP9 ||
	     sess->fmt_out->pixfmt == V4L2_PIX_FMT_HEVC) &&
	    (u64)width * height > sess->core->hevc_engine_cma_max_pixels) {
		dev_err(sess->core->dev,
			"Stream %ux%u (%llu MP) exceeds HEVC-engine CMA budget %llu MP; aborting\n",
			width, height, ((u64)width * height) >> 20,
			sess->core->hevc_engine_cma_max_pixels >> 20);
		amvdec_abort(sess);
		return;
	}

	v4l2_ctrl_s_ctrl(sess->ctrl_min_buf_capture, dpb_size);

	/*
	 * Every direct-resume path below skips V4L2_EVENT_SOURCE_CHANGE, so
	 * each one also requires the CAPTURE buffers already allocated to FIT
	 * the new resolution, not just to be numerous enough.  A client that
	 * streamed CAPTURE at a placeholder format (Android v4l2_codec2
	 * negotiates 256x144 first) otherwise has a 720p frame staged into a
	 * 256x144 buffer: a write past its last page and an Oops in
	 * codec_hevc_stage_output().  Too-small buffers fall through to
	 * STATUS_NEEDS_RESUME + the event, so the client reallocates first.
	 */
	cap_fits = sess->streamon_cap && amvdec_cap_bufs_fit(sess, width, height);
	if (sess->streamon_cap && !cap_fits)
		dev_info(sess->core->dev,
			 "Res. %ux%u: capture buffers too small, sending SOURCE_CHANGE (bufs=%u)\n",
			 width, height, sess->num_dst_bufs);

	/* Fast resume: capture is already streaming and resolution unchanged.
	 *
	 * If dpb_size exceeds the allocated buffer count, the firmware can
	 * still decode -- it will manage the DPB with fewer slots, evicting
	 * older references as needed.  Without this path, the driver sets
	 * STATUS_NEEDS_RESUME and sends SRC_CHANGE, but ffmpeg (correctly)
	 * ignores the event since resolution is unchanged, leaving the
	 * decoder permanently stalled with zero output frames.
	 *
	 * Log a warning when clamping so the mismatch is visible in dmesg.
	 */
	if (cap_fits &&
	    sess->width == width &&
	    sess->height == height) {
		if (dpb_size > sess->num_dst_bufs)
			dev_info(sess->core->dev,
				 "DPB %u exceeds capture bufs %u, resuming with available buffers\n",
				 dpb_size, sess->num_dst_bufs);
		dev_dbg(sess->core->dev,
			"Res. unchanged (%ux%u), fast resume (dpb=%u bufs=%u)\n",
			width, height, dpb_size, sess->num_dst_bufs);
		sess->fmt_out->codec_ops->resume(sess);
		return;
	}

	sess->changed_format = 0;
	sess->width = width;
	sess->height = height;

	vdec_dbg_log_event(sess->core, VDEC_EVT_SRC_CHANGE, 0,
			   (width << 16) | height);

	/*
	 * Spec: dev-decoder.rst "Dynamic resolution change" -- once the
	 * SOURCE_CHANGE event is raised, a stateful decoder must produce
	 * no CAPTURE buffers until the client has re-initialised the
	 * capture queue (STREAMOFF/REQBUFS/STREAMON).  The previous
	 * "cap already streaming -> direct resume" and
	 * "dpb too large -> clamp and resume" shortcuts decoded straight
	 * into the OLD pool while the client was mid-reinit, emptying it
	 * under the running decoder ("No dst buffer available" after one
	 * frame).  Wait for the client's CAPTURE restart instead:
	 * vdec_start_streaming() picks up STATUS_NEEDS_RESUME and calls
	 * resume() with the freshly (re)queued buffers.
	 */

	/*
	 * Set STATUS_NEEDS_RESUME before the log message and event.
	 * vdec_start_streaming() reads sess->status on STREAMON(CAPTURE)
	 * which can arrive within <1ms of the resolution change event.
	 * If status is still STATUS_INIT when STREAMON runs, the resume
	 * path is skipped and the decoder produces zero output frames.
	 */
	atomic_set(&sess->resumed, 0);
	WRITE_ONCE(sess->status, STATUS_NEEDS_RESUME);

	dev_info(sess->core->dev,
		 "Res. changed (%ux%u), DPB %u: cap=%d w=%u/%u h=%u/%u bufs=%u\n",
		 width, height, dpb_size, sess->streamon_cap,
		 sess->width, width, sess->height, height,
		 sess->num_dst_bufs);
	v4l2_event_queue_fh(&sess->fh, &ev);

	/*
	 * No direct resume here even when capture is already streaming:
	 * a CHANGED resolution obliges the client to re-initialise the
	 * capture queue (see the spec note above), and
	 * vdec_start_streaming() is the single rendezvous that calls
	 * resume() -- with the client's freshly queued buffers. Resuming
	 * from here raced that reinit and starved frame 1.
	 */
}
EXPORT_SYMBOL_GPL(amvdec_src_change);

void amvdec_abort(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;

	vdec_dbg_log_event(core, VDEC_EVT_ABORT, 0, 0);

	dev_info(core->dev, "Aborting decoding session!\n");

	/*
	 * Set aborted flag BEFORE halting firmware so the recycle thread
	 * and ISR handlers see it and skip further buffer access.
	 */
	WRITE_ONCE(sess->aborted, 1);
	smp_wmb();

	/*
	 * Halt the firmware immediately to prevent stray DMA writes
	 * and further ISRs.  For unsupported profiles (e.g. H.264
	 * High10) the firmware may perform DMA with mis-programmed
	 * parameters.  MPSR=0 halts the processor; mask=0 prevents
	 * new IRQs.  The aborted flag above ensures the recycle thread
	 * bails out if it was already awake.
	 */
	if (sess->fmt_out->vdec_ops == &vdec_hevc_ops) {
		amvdec_write_dos(core, HEVC_MPSR, 0);
		amvdec_write_dos(core, HEVC_ASSIST_MBOX1_MASK, 0);
	} else {
		amvdec_write_dos(core, MPSR, 0);
		amvdec_write_dos(core, CPSR, 0);
		amvdec_write_dos(core, ASSIST_MBOX1_MASK, 0);
	}

	/*
	 * Do NOT clear core->cur_sess here (KI-148).
	 *
	 * It used to be cleared so an in-flight threaded ISR would bail out --
	 * but cur_sess is the single-engine CLAIM, and dropping it here hands
	 * the engine to the next opener while this session is still powered and
	 * has not been torn down.  The next session then powers the engine on
	 * and the aborted session's (deferred) teardown powers it off
	 * underneath: that is the exact sequence in the 2026-07-30 03:12
	 * pre-reset log -- "Aborting decoding session!" followed immediately by
	 * the next session's "Res. changed (320x240)" and then a silent SoC
	 * reset.
	 *
	 * The ISR bail-out is preserved without giving away the claim: both
	 * vdec_isr() and vdec_threaded_isr() now return early on
	 * READ_ONCE(sess->aborted), which is set above (with the smp_wmb)
	 * before the firmware is halted.  The claim is released by
	 * vdec_stop_streaming()/vdec_close() once the engine is actually off.
	 */

	vb2_queue_error(&sess->m2m_ctx->cap_q_ctx.q);
	vb2_queue_error(&sess->m2m_ctx->out_q_ctx.q);
}
EXPORT_SYMBOL_GPL(amvdec_abort);

/*
 * Scratch buffer size: must fit one decoded frame at max VDEC_1 resolution.
 * VDEC_1 codecs support up to 1920x1088 NV12 = ~3.1MB.
 * Use 4MB to cover alignment and block-mode overhead.
 */
#define SCRATCH_SIZE SZ_4M

int amvdec_scratch_init(struct amvdec_core *core)
{
	int ret, i;

	core->scratch_vaddr = dma_alloc_coherent(core->dev, SCRATCH_SIZE,
						 &core->scratch_paddr,
						 GFP_KERNEL);
	if (!core->scratch_vaddr)
		return -ENOMEM;

	for (i = 0; i < 3; i++) {
		ret = meson_canvas_alloc(core->canvas,
					 &core->scratch_canvas[i]);
		if (ret)
			goto free_canvas;
	}

	/*
	 * Configure Y/U/V canvases pointing to the scratch buffer.
	 * Use large stride and height so any firmware resolution fits.
	 * BLKMODE_32x32 matches what VDEC_1 firmware expects.
	 */
	meson_canvas_config(core->canvas, core->scratch_canvas[0],
			    core->scratch_paddr, 1920, 1088,
			    MESON_CANVAS_WRAP_NONE,
			    MESON_CANVAS_BLKMODE_32x32,
			    MESON_CANVAS_ENDIAN_SWAP64);
	meson_canvas_config(core->canvas, core->scratch_canvas[1],
			    core->scratch_paddr, 960, 544,
			    MESON_CANVAS_WRAP_NONE,
			    MESON_CANVAS_BLKMODE_32x32,
			    MESON_CANVAS_ENDIAN_SWAP64);
	meson_canvas_config(core->canvas, core->scratch_canvas[2],
			    core->scratch_paddr, 960, 544,
			    MESON_CANVAS_WRAP_NONE,
			    MESON_CANVAS_BLKMODE_32x32,
			    MESON_CANVAS_ENDIAN_SWAP64);

	return 0;

free_canvas:
	while (i--)
		meson_canvas_free(core->canvas, core->scratch_canvas[i]);
	dma_free_coherent(core->dev, SCRATCH_SIZE,
			  core->scratch_vaddr, core->scratch_paddr);
	core->scratch_vaddr = NULL;
	return ret;
}
EXPORT_SYMBOL_GPL(amvdec_scratch_init);

void amvdec_scratch_free(struct amvdec_core *core)
{
	int i;

	if (!core->scratch_vaddr)
		return;

	for (i = 0; i < 3; i++)
		meson_canvas_free(core->canvas, core->scratch_canvas[i]);

	dma_free_coherent(core->dev, SCRATCH_SIZE,
			  core->scratch_vaddr, core->scratch_paddr);
	core->scratch_vaddr = NULL;
}
EXPORT_SYMBOL_GPL(amvdec_scratch_free);

void amvdec_fill_scratch_canvases(struct amvdec_core *core)
{
	u32 packed;
	int i;

	packed = ((u32)core->scratch_canvas[2] << 16) |
		 ((u32)core->scratch_canvas[1] << 8) |
		 (u32)core->scratch_canvas[0];

	/* Fill AV_SCRATCH_0..L (22 regs) — covers MPEG12, MPEG4, MJPEG */
	for (i = 0; i < 22; i++)
		amvdec_write_dos(core, AV_SCRATCH_0 + i * 4, packed);

	/* Fill ANC0_CANVAS_ADDR + 0..31 — covers H.264 (32 DPB slots) */
	for (i = 0; i < 32; i++)
		amvdec_write_dos(core, ANC0_CANVAS_ADDR + i * 4, packed);
}
EXPORT_SYMBOL_GPL(amvdec_fill_scratch_canvases);

#define MAX_REF_PIC_NUM 24

void amvdec_fill_scratch_hevc(struct amvdec_core *core)
{
	int i;

	/*
	 * GXL+ path: ANC2AXI_TBL_DATA takes address >> 5.
	 * Fill all reference slots with the scratch buffer address.
	 */
	amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_CONF_ADDR,
			 ANC2AXI_CONF_AUTO_INC);
	for (i = 0; i < MAX_REF_PIC_NUM; i++)
		amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_DATA,
				 core->scratch_paddr >> 5);
	amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_CONF_ADDR, 1);

	/* Zero out canvas data — GXL+ uses AXI table, not canvas */
	amvdec_write_dos(core, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, 1);
	for (i = 0; i < 32; i++)
		amvdec_write_dos(core, HEVCD_MPP_ANC_CANVAS_DATA_ADDR, 0);
}
EXPORT_SYMBOL_GPL(amvdec_fill_scratch_hevc);

/**
 * amvdec_fw_identify - Compute SHA256 of a firmware buffer and look it up.
 *
 * See vdec_helpers.h for the full return-value contract:
 *   valid pointer  -- matched entry, caller continues
 *   ERR_PTR(-ENOENT) -- table non-empty but hash not found; caller MUST refuse
 *   NULL           -- no table; caller continues (backward compat)
 */
const struct amvdec_fw_version *
amvdec_fw_identify(struct device *dev, const char *fwname,
		   const u8 *data, size_t size,
		   const struct amvdec_fw_version *table, size_t n)
{
	u8 digest[SHA256_DIGEST_SIZE];
	const struct amvdec_fw_version *match = NULL;
	const struct amvdec_fw_version *latest = NULL;
	size_t i;

	sha256(data, size, digest);

	if (!table || !n) {
		dev_info(dev,
			 "meson-vdec fw: %s size=%zu sha256=%02x%02x%02x%02x%02x%02x%02x%02x (no version table)\n",
			 fwname, size,
			 digest[0], digest[1], digest[2], digest[3],
			 digest[4], digest[5], digest[6], digest[7]);
		return NULL;
	}

	for (i = 0; i < n; i++) {
		if (table[i].size == size &&
		    memcmp(table[i].sha256, digest, SHA256_DIGEST_SIZE) == 0) {
			match = &table[i];
			break;
		}
		if (table[i].latest)
			latest = &table[i];
	}

	if (!match) {
		/*
		 * Static-vs-dynamic firmware split:
		 *
		 * BSP / upstream firmware (filename does NOT start with `libre_`,
		 * e.g. `gxl_h264.bin`, `g12a_hevc_mmu.bin`) ships from
		 * linux-firmware or vendor BSP releases.  SHAs are static --
		 * a hash miss means the operator loaded the wrong file or a
		 * fuzzed binary, so refuse to load and force them to verify
		 * the firmware catalog.  Original 2026-04-16 HEVC G12B incident
		 * (wrong firmware silently used, multi-strike investigation).
		 *
		 * libre stateless firmware (filename starts with `libre_`,
		 * e.g. `libre_sm1_h264.bin`, `libre_sm1_vdec1.bin`) is built
		 * fresh from the amlogic codec/decoder/stateless/firmware/
		 * source tree on every M4.x iteration.  SHA changes per build.
		 * Refusing to load every dev iteration would force a kernel
		 * rebuild + reload cycle for every firmware tweak (~15 min)
		 * and gate development on driver-side bookkeeping.  Instead
		 * warn-and-continue with NULL fw_match: caller proceeds with
		 * no quirks (libre dev firmware is the test subject; quirks
		 * are pinned only after a libre version is finalized).
		 *
		 * Per /codec-stateless skill rule 1a (revised 2026-04-23
		 * after this contradiction surfaced) the libre-permissive
		 * carve-out is the documented behavior.
		 */
		if (fwname && strstr(fwname, "libre_")) {
			dev_warn(dev,
				 "meson-vdec fw: %s size=%zu sha256=%02x%02x%02x%02x%02x%02x%02x%02x -- unrecognized libre firmware (dev mode); continuing without quirks; add to fw_versions[] when stable\n",
				 fwname, size,
				 digest[0], digest[1], digest[2], digest[3],
				 digest[4], digest[5], digest[6], digest[7]);
			return NULL;
		}

		dev_err(dev,
			"meson-vdec fw: %s size=%zu sha256=%02x%02x%02x%02x%02x%02x%02x%02x -- unrecognized firmware, please report\n",
			fwname, size,
			digest[0], digest[1], digest[2], digest[3],
			digest[4], digest[5], digest[6], digest[7]);
		dev_err(dev,
			"meson-vdec fw: %s refusing to load unrecognized firmware\n",
			fwname);
		return ERR_PTR(-ENOENT);
	}

	dev_info(dev,
		 "meson-vdec fw: %s label=\"%s\" provenance=\"%s\" size=%zu sha256=%02x%02x%02x%02x%02x%02x%02x%02x quirks=0x%x\n",
		 fwname, match->label, match->provenance, match->size,
		 digest[0], digest[1], digest[2], digest[3],
		 digest[4], digest[5], digest[6], digest[7],
		 match->quirks);

	if (!match->latest && latest) {
		dev_warn(dev,
			 "meson-vdec fw: %s WARNING: firmware is not latest (label=\"%s\", latest=\"%s\"); update linux-firmware for bug fixes\n",
			 fwname, match->label, latest->label);
	}

	return match;
}
EXPORT_SYMBOL_GPL(amvdec_fw_identify);
