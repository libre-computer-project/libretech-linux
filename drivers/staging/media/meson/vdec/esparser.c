// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018 BayLibre, SAS
 * Author: Maxime Jourdan <mjourdan@baylibre.com>
 *
 * The Elementary Stream Parser is a HW bitstream parser.
 * It reads bitstream buffers and feeds them to the VIFIFO
 */

#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/reset.h>
#include <linux/interrupt.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-mem2mem.h>

#include "dos_regs.h"
#include "esparser.h"
#include "vdec_1.h"
#include "vdec_helpers.h"
#include "vdec_debugfs.h"
#include "codec_hevc_common.h"

/* PARSER REGS (CBUS) */
#define PARSER_CONTROL 0x00
	#define ES_PACK_SIZE_BIT	8
	#define ES_WRITE		BIT(5)
	#define ES_SEARCH		BIT(1)
	#define ES_PARSER_START		BIT(0)
#define PARSER_FETCH_ADDR	0x4
#define PARSER_FETCH_CMD	0x8
#define PARSER_CONFIG 0x14
	#define PS_CFG_MAX_FETCH_CYCLE_BIT	0
	#define PS_CFG_STARTCODE_WID_24_BIT	10
	#define PS_CFG_MAX_ES_WR_CYCLE_BIT	12
	#define PS_CFG_PFIFO_EMPTY_CNT_BIT	16
#define PFIFO_WR_PTR 0x18
#define PFIFO_RD_PTR 0x1c
#define PARSER_SEARCH_PATTERN 0x24
	#define ES_START_CODE_PATTERN 0x00000100
#define PARSER_SEARCH_MASK 0x28
	#define ES_START_CODE_MASK	0xffffff00
	#define FETCH_ENDIAN_BIT	27
	/* SWAP64 endian for parser fetch DMA (same as IMEM DMA endian=7) */
	#define FETCH_ENDIAN_SWAP64	(7 << FETCH_ENDIAN_BIT)
#define PARSER_INT_ENABLE	0x2c
	#define PARSER_INT_HOST_EN_BIT	8
#define PARSER_INT_STATUS	0x30
	#define PARSER_INTSTAT_SC_FOUND	1
#define PARSER_ES_CONTROL	0x5c
	#define PARSER_ES_CTRL_VIDEO_MAN_RD	BIT(0)
#define PARSER_VIDEO_START_PTR	0x80
#define PARSER_VIDEO_END_PTR	0x84
#define PARSER_VIDEO_WP		0x88
#define PARSER_VIDEO_HOLE	0x90

#define SEARCH_PATTERN_LEN	512
#define VP9_HEADER_SIZE		16
/* Internal parser FIFO size (must be subtracted from VIFIFO free space) */
#define PARSER_INTERNAL_FIFO_SIZE	(6 * SZ_1K)
/* VIFIFO end pointer must be 8 bytes before the buffer end (HW constraint) */
#define VIFIFO_END_PTR_ALIGN		8

static DECLARE_WAIT_QUEUE_HEAD(wq);
static int search_done;
/*
 * One parser, one search at a time.  The fetch registers, search_done and wq
 * are shared by every writer -- a session's feed work, the DEC_CMD_STOP EOS
 * injection and the codecs' drain-time trailers -- and a second fetch issued
 * while one is in flight clears search_done and overwrites the first: that
 * write's wait then times out ("input parsing error") and its access unit is
 * dropped (KI-172: the last H.264 frame, ~4% of A311D sessions).
 */
static DEFINE_MUTEX(esparser_fetch_lock);

static irqreturn_t esparser_isr(int irq, void *dev)
{
	int int_status;
	struct amvdec_core *core = dev;

	int_status = amvdec_read_parser(core, PARSER_INT_STATUS);
	amvdec_write_parser(core, PARSER_INT_STATUS, int_status);

	if (int_status & PARSER_INTSTAT_SC_FOUND) {
		amvdec_write_parser(core, PFIFO_RD_PTR, 0);
		amvdec_write_parser(core, PFIFO_WR_PTR, 0);
		search_done = 1;
		wake_up_interruptible(&wq);
	}

	return IRQ_HANDLED;
}

/*
 * VP9 frame headers need to be appended by a 16-byte long
 * Amlogic custom header
 */
static int vp9_update_header(struct amvdec_core *core, struct vb2_buffer *buf)
{
	u8 *dp;
	u8 marker;
	int dsize;
	int num_frames, cur_frame;
	int cur_mag, mag, mag_ptr;
	int frame_size[8], tot_frame_size[8];
	int total_datasize = 0;
	int new_frame_size;
	unsigned char *old_header = NULL;

	dp = (uint8_t *)vb2_plane_vaddr(buf, 0);
	dsize = vb2_get_plane_payload(buf, 0);

	if (dsize == vb2_plane_size(buf, 0)) {
		dev_warn(core->dev, "%s: unable to update header\n", __func__);
		return 0;
	}

	marker = dp[dsize - 1];
	if ((marker & 0xe0) == 0xc0) {
		num_frames = (marker & 0x7) + 1;
		mag = ((marker >> 3) & 0x3) + 1;
		mag_ptr = dsize - mag * num_frames - 2;
		if (dp[mag_ptr] != marker)
			return 0;

		mag_ptr++;
		for (cur_frame = 0; cur_frame < num_frames; cur_frame++) {
			frame_size[cur_frame] = 0;
			for (cur_mag = 0; cur_mag < mag; cur_mag++) {
				frame_size[cur_frame] |=
					(dp[mag_ptr] << (cur_mag * 8));
				mag_ptr++;
			}
			if (cur_frame == 0)
				tot_frame_size[cur_frame] =
					frame_size[cur_frame];
			else
				tot_frame_size[cur_frame] =
					tot_frame_size[cur_frame - 1] +
					frame_size[cur_frame];
			total_datasize += frame_size[cur_frame];
		}
	} else {
		num_frames = 1;
		frame_size[0] = dsize;
		tot_frame_size[0] = dsize;
		total_datasize = dsize;
	}

	new_frame_size = total_datasize + num_frames * VP9_HEADER_SIZE;

	if (new_frame_size >= vb2_plane_size(buf, 0)) {
		dev_warn(core->dev, "%s: unable to update header\n", __func__);
		return 0;
	}

	for (cur_frame = num_frames - 1; cur_frame >= 0; cur_frame--) {
		int framesize = frame_size[cur_frame];
		int framesize_header = framesize + 4;
		int oldframeoff = tot_frame_size[cur_frame] - framesize;
		int outheaderoff =  oldframeoff + cur_frame * VP9_HEADER_SIZE;
		u8 *fdata = dp + outheaderoff;
		u8 *old_framedata = dp + oldframeoff;

		memmove(fdata + VP9_HEADER_SIZE, old_framedata, framesize);

		fdata[0] = (framesize_header >> 24) & 0xff;
		fdata[1] = (framesize_header >> 16) & 0xff;
		fdata[2] = (framesize_header >> 8) & 0xff;
		fdata[3] = (framesize_header >> 0) & 0xff;
		fdata[4] = ((framesize_header >> 24) & 0xff) ^ 0xff;
		fdata[5] = ((framesize_header >> 16) & 0xff) ^ 0xff;
		fdata[6] = ((framesize_header >> 8) & 0xff) ^ 0xff;
		fdata[7] = ((framesize_header >> 0) & 0xff) ^ 0xff;
		fdata[8] = 0;
		fdata[9] = 0;
		fdata[10] = 0;
		fdata[11] = 1;
		fdata[12] = 'A';
		fdata[13] = 'M';
		fdata[14] = 'L';
		fdata[15] = 'V';

		if (!old_header) {
			/* nothing */
		} else if (old_header > fdata + 16 + framesize) {
			dev_dbg(core->dev, "%s: data has gaps, setting to 0\n",
				__func__);
			memset(fdata + 16 + framesize, 0,
			       (old_header - fdata + 16 + framesize));
		} else if (old_header < fdata + 16 + framesize) {
			dev_err(core->dev, "%s: data overwritten\n", __func__);
		}
		old_header = fdata;
	}

	return new_frame_size;
}

/* Pad the packet to at least 4KiB bytes otherwise the VDEC unit won't trigger
 * ISRs.
 * Also append a start code 000001ff at the end to trigger
 * the ESPARSER interrupt.
 *
 * @payload_size is in/out: on entry the length of the packet the caller is
 * about to hand to the parser (which a codec path may already have changed
 * without touching the vb2 plane payload); on truncation it is lowered to the
 * size that leaves room for the search pattern.
 */
static int esparser_pad_start_code(struct amvdec_core *core,
				  struct vb2_buffer *vb,
				  u32 *payload_size_p)
{
	u32 buf_size = vb2_plane_size(vb, 0);
	u32 payload_size = *payload_size_p;
	u32 pad_size = 0;
	u32 max_payload;
	u8 *vaddr = vb2_plane_vaddr(vb, 0);

	/*
	 * Everything this function writes lands inside this one vb2 plane:
	 * the zero padding up to ESPARSER_MIN_PACKET_SIZE, then the
	 * SEARCH_PATTERN_LEN scratch area holding the 00 00 01 ff start code.
	 * A plane smaller than the sum of the two can therefore never hold a
	 * packet the hardware will accept, so refuse it here instead of
	 * writing past the mapping.
	 */
	if (buf_size < ESPARSER_MIN_PACKET_SIZE + SEARCH_PATTERN_LEN) {
		dev_err_ratelimited(core->dev,
			"%s: OUTPUT plane too small: %u < %u (min packet %u + search pattern %u)\n",
			__func__, buf_size,
			ESPARSER_MIN_PACKET_SIZE + SEARCH_PATTERN_LEN,
			ESPARSER_MIN_PACKET_SIZE, SEARCH_PATTERN_LEN);
		return -EINVAL;
	}

	/*
	 * Bound the payload FIRST, so the padding below can never push the
	 * packet back out of the buffer.  The old code padded first and then
	 * tried to "truncate to make room", but clamped the truncated size
	 * back UP to ESPARSER_MIN_PACKET_SIZE -- which re-created the very
	 * overflow the truncation existed to prevent whenever the plane was
	 * smaller than ESPARSER_MIN_PACKET_SIZE + SEARCH_PATTERN_LEN.
	 *
	 * Payload filling the buffer with no room for the search pattern
	 * happens when userspace (e.g., v4l2-ctl --stream-from) sets
	 * bytesused = vb2_plane_size instead of sizeimage.
	 *
	 * For stateful decoders receiving a continuous elementary stream, the
	 * truncated tail bytes are harmless -- they are trailing data past the
	 * last complete frame in this chunk.  The firmware will decode all
	 * complete frames within the truncated payload.
	 */
	max_payload = buf_size - SEARCH_PATTERN_LEN;
	if (payload_size > max_payload) {
		dev_dbg(core->dev,
			"%s: payload %u truncated to %u to fit search pattern\n",
			__func__, payload_size, max_payload);
		payload_size = max_payload;
		*payload_size_p = payload_size;
		vb2_set_plane_payload(vb, 0, payload_size);
	}

	if (payload_size < ESPARSER_MIN_PACKET_SIZE) {
		pad_size = ESPARSER_MIN_PACKET_SIZE - payload_size;
		memset(vaddr + payload_size, 0, pad_size);
	}

	/*
	 * buf_size >= ESPARSER_MIN_PACKET_SIZE + SEARCH_PATTERN_LEN and
	 * payload_size <= buf_size - SEARCH_PATTERN_LEN, so
	 * payload_size + pad_size <= buf_size - SEARCH_PATTERN_LEN and the
	 * write below always terminates at or before the end of the plane.
	 */
	memset(vaddr + payload_size + pad_size, 0, SEARCH_PATTERN_LEN);
	vaddr[payload_size + pad_size]     = 0x00;
	vaddr[payload_size + pad_size + 1] = 0x00;
	vaddr[payload_size + pad_size + 2] = 0x01;
	vaddr[payload_size + pad_size + 3] = 0xff;

	return pad_size;
}

static int
esparser_write_data(struct amvdec_core *core, dma_addr_t addr, u32 size,
		    bool cached_import)
{
	int ret;

	mutex_lock(&esparser_fetch_lock);
	amvdec_write_parser(core, PFIFO_RD_PTR, 0);
	amvdec_write_parser(core, PFIFO_WR_PTR, 0);
	amvdec_write_parser(core, PARSER_CONTROL,
			    ES_WRITE |
			    ES_PARSER_START |
			    ES_SEARCH |
			    (size << ES_PACK_SIZE_BIT));

	/*
	 * The ESPARSER fetches `size + SEARCH_PATTERN_LEN` bytes from `addr`
	 * by raw DMA (no IOMMU on this SoC).
	 *
	 * ONLY an imported, CPU-cached DMABUF (an Android v4l2_codec2 coded
	 * input from the linux,cma dma-heap) needs a for-device sync here:
	 * vb2_dc_prepare() skips the cache sync for imported DMABUFs, so
	 * without this the parser reads stale physical memory, finds no start
	 * code, and the SC search times out ("esparser: input parsing error").
	 *
	 * MMAP OUTPUT buffers (vb2_dma_contig => dma_alloc_coherent) and the
	 * EOS scratch buffer (also dma_alloc_coherent) are COHERENT, and
	 * issuing a STREAMING dma_sync on a coherent allocation is illegal and
	 * ACTIVELY CORRUPTING: on this non-dma-coherent decoder
	 * dma_sync_single_for_device() cleans the buffer's cacheable
	 * linear-map alias -- which can hold stale/dirty lines left from a
	 * prior use of those physical pages -- writing that garbage back over
	 * the freshly (non-cached) written coded bytes in DRAM. The damage
	 * scales with the synced range (= coded-frame size), so a large
	 * (>=720p) I-frame loses its slice headers and the firmware stalls
	 * after its first IRQ with 0 frames out, while a small (<=VGA) frame
	 * survives -- the meson HEVC ">=720p decode hang" regression (KI-140,
	 * introduced by the unconditional sync in 58c3e432ab9d). Gate the sync
	 * on the imported-cached-DMABUF case that actually needs it.
	 */
	if (cached_import)
		dma_sync_single_for_device(core->dev, addr,
					   size + SEARCH_PATTERN_LEN,
					   DMA_TO_DEVICE);

	search_done = 0;
	amvdec_write_parser(core, PARSER_FETCH_ADDR, addr);
	amvdec_write_parser(core, PARSER_FETCH_CMD,
			    FETCH_ENDIAN_SWAP64 |
			    (size + SEARCH_PATTERN_LEN));

	ret = wait_event_interruptible_timeout(wq, search_done, (HZ / 5));
	if (ret <= 0)	/* cancel the failed fetch before the next writer */
		amvdec_write_parser(core, PARSER_FETCH_CMD, 0);
	mutex_unlock(&esparser_fetch_lock);
	return ret;
}

static u32 esparser_vififo_get_free_space(struct amvdec_session *sess)
{
	u32 vififo_usage;
	struct amvdec_ops *vdec_ops = sess->fmt_out->vdec_ops;
	struct amvdec_core *core = sess->core;

	vififo_usage  = vdec_ops->vififo_level(sess);
	vififo_usage += amvdec_read_parser(core, PARSER_VIDEO_HOLE);
	vififo_usage += PARSER_INTERNAL_FIFO_SIZE;

	if (vififo_usage > sess->vififo_size) {
		dev_warn(sess->core->dev,
			 "VIFIFO usage (%u) > VIFIFO size (%u)\n",
			 vififo_usage, sess->vififo_size);
		return 0;
	}

	return sess->vififo_size - vififo_usage;
}

int esparser_queue_eos(struct amvdec_core *core, const u8 *data, u32 len)
{
	struct device *dev = core->dev;
	void *eos_vaddr;
	dma_addr_t eos_paddr;
	int ret;

	eos_vaddr = dma_alloc_coherent(dev, len + SEARCH_PATTERN_LEN,
				       &eos_paddr, GFP_KERNEL);
	if (!eos_vaddr)
		return -ENOMEM;

	memcpy(eos_vaddr, data, len);
	/* Coherent (dma_alloc_coherent) scratch -- must NOT be dma_sync'd. */
	ret = esparser_write_data(core, eos_paddr, len, false);
	dma_free_coherent(dev, len + SEARCH_PATTERN_LEN,
			  eos_vaddr, eos_paddr);

	return ret;
}
EXPORT_SYMBOL_GPL(esparser_queue_eos);

static u32 esparser_get_offset(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	u32 offset = amvdec_read_parser(core, PARSER_VIDEO_WP) -
		     sess->vififo_paddr;

	if (offset < sess->last_offset)
		sess->wrap_count++;

	sess->last_offset = offset;
	offset += (sess->wrap_count * sess->vififo_size);

	return offset;
}

int
esparser_queue(struct amvdec_session *sess, struct vb2_v4l2_buffer *vbuf)
{
	int ret;
	struct vb2_buffer *vb = &vbuf->vb2_buf;
	struct amvdec_core *core = sess->core;
	struct amvdec_codec_ops *codec_ops = sess->fmt_out->codec_ops;
	u32 payload_size = vb2_get_plane_payload(vb, 0);
	dma_addr_t phy = vb2_dma_contig_plane_dma_addr(vb, 0);
	u32 num_dst_bufs = 0;
	u32 offset;
	int pad_size;
	bool skip_ts = false;

	/*
	 * Throttle input based on available capture buffers.
	 * Without this, all input can be queued to the VIFIFO at once,
	 * causing the firmware to decode faster than userspace can
	 * recycle capture buffers.  When capture buffers run out, the
	 * decoder stalls or drops frames.
	 *
	 * Only applied after capture streaming starts (streamon_cap).
	 * Before SOURCE_CHANGE, no capture buffers exist so the throttle
	 * would block all initial input, preventing resolution detection.
	 *
	 * Applied to:
	 *  - VP9/HEVC: original throttle (these codecs provide
	 *    num_pending_bufs for accurate accounting)
	 *  - MPEG1/2: same starvation on G12B where the faster hardware
	 *    exhausts 8 canvas buffers before userspace can recycle
	 *    (observed: 30/75 MPEG-2 frames on G12B cold start)
	 *
	 * NOT applied to:
	 *  - H.264: its 24-buffer DPB is large enough to avoid
	 *    starvation, and throttling causes B-frame reference deadlocks
	 *  - MPEG4/H.263: firmware holds reference frames that deplete
	 *    the capture ready count to 0, deadlocking the throttle after
	 *    ~32 frames (KI-052)
	 *  - MJPEG: no reference frames, no starvation risk
	 *
	 * The margin of 3 accounts for buffers held as reference frames
	 * by the decoder that haven't been returned to the V4L2 queue.
	 */
	if (!amvdec_is_stateless(sess) &&
	    sess->streamon_cap &&
	    (sess->fmt_out->pixfmt == V4L2_PIX_FMT_VP9 ||
	     sess->fmt_out->pixfmt == V4L2_PIX_FMT_HEVC ||
	     (sess->fmt_out->max_buffers <= 8 &&
	      sess->fmt_out->pixfmt != V4L2_PIX_FMT_MJPEG &&
	      sess->fmt_out->pixfmt != V4L2_PIX_FMT_MPEG4 &&
	      sess->fmt_out->pixfmt != V4L2_PIX_FMT_H263))) {
		if (codec_ops->num_pending_bufs)
			num_dst_bufs = codec_ops->num_pending_bufs(sess);

		num_dst_bufs += v4l2_m2m_num_dst_bufs_ready(sess->m2m_ctx);
		if (num_dst_bufs > 3)
			num_dst_bufs -= 3;
		else
			num_dst_bufs = 0;

		if (esparser_vififo_get_free_space(sess) < payload_size ||
		    atomic_read(&sess->esparser_queued_bufs) >= num_dst_bufs)
			return -EAGAIN;
	} else if (esparser_vififo_get_free_space(sess) < payload_size) {
		return -EAGAIN;
	}

	v4l2_m2m_src_buf_remove_by_buf(sess->m2m_ctx, vbuf);

	offset = esparser_get_offset(sess);

	/*
	 * HEVC config-only-AU timestamp leak fix.
	 *
	 * Android CCodec feeds a parameter-set access unit (VPS/SPS/PPS/SEI,
	 * no VCL slice -- FLAG_CODEC_CONFIG, bitstreamId 0) as its own OUTPUT
	 * buffer.  It decodes no picture.  The HEVC codec claims per-picture
	 * timestamps in DECODE ORDER from a FIFO (codec_hevc_take_ts), so if
	 * this config buffer enqueues a ts the first real picture (the IDR)
	 * pops it and every picture's ts is shifted by one input -- the IDR's
	 * CAPTURE frame is delivered under the config buffer's timestamp
	 * (== bitstreamId 0), whose c2 Work was already completed+erased, so
	 * CCodec reports "Work with bitstreamId=0 not found" -> C2_CORRUPTED.
	 * H.264 is immune: it matches timestamps by bitstream offset
	 * (amvdec_dst_buf_done_offset), which skips a spurious config entry.
	 *
	 * Do not enqueue a decode-order ts for a config-only HEVC buffer, so
	 * real pictures pop their own input buffer's ts.  Scoped to HEVC (the
	 * only FIFO-take_ts codec fed separate CSD AUs); VP9's superframe
	 * headers always carry a frame, so it is unaffected.
	 */
	if (sess->fmt_out->pixfmt == V4L2_PIX_FMT_HEVC &&
	    codec_hevc_is_config_only(vb2_plane_vaddr(vb, 0), payload_size)) {
		skip_ts = hevc_skip_config_ts;
		dev_info(core->dev,
			 "esparser: HEVC config-only AU (no VCL) ts=%llu offset=%08X size=%u -- ts %s\n",
			 vb->timestamp, offset, payload_size,
			 skip_ts ? "NOT enqueued" : "ENQUEUED (skip disabled)");
	}

	if (!skip_ts) {
		ret = amvdec_add_ts(sess, vb->timestamp, vbuf->timecode, offset,
				    vbuf->flags);
		if (ret) {
			v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
			return ret;
		}

		/* KI-175: tell the HEVC codec which picture starts a new
		 * coded video sequence; the ts entry just added is this AU's,
		 * and the codec claims entries in decode order.
		 */
		if (sess->fmt_out->pixfmt == V4L2_PIX_FMT_HEVC &&
		    codec_hevc_au_starts_cvs(vb2_plane_vaddr(vb, 0),
					     payload_size)) {
			unsigned long flags;

			spin_lock_irqsave(&sess->ts_spinlock, flags);
			list_last_entry(&sess->timestamps,
					struct amvdec_timestamp,
					list)->cvs_start = true;
			spin_unlock_irqrestore(&sess->ts_spinlock, flags);
		}
	}

	dev_dbg(core->dev, "esparser: ts = %llu pld_size = %u offset = %08X flags = %08X\n",
		vb->timestamp, payload_size, offset, vbuf->flags);

	vbuf->flags = 0;
	vbuf->field = V4L2_FIELD_NONE;
	vbuf->sequence = sess->sequence_out++;

	if (sess->fmt_out->pixfmt == V4L2_PIX_FMT_VP9) {
		u8 *vp9_data = vb2_plane_vaddr(vb, 0);

		/* VP9 uncompressed header: 2-bit frame_marker (0b10),
		 * 1-bit profile_low_bit, 1-bit profile_high_bit.
		 * Profile = (high << 1) | low.
		 * Profiles 0 (8-bit 4:2:0) and 2 (10-bit 4:2:0) are
		 * supported.  Profiles 1 and 3 (4:2:2/4:4:4 chroma)
		 * cause the VP9 firmware parser to hang -- it never
		 * fires VP9_HEAD_PARSER_DONE, so the ISR never runs
		 * and the session blocks indefinitely.
		 */
		if (vp9_data && payload_size >= 1) {
			u8 b0 = vp9_data[0];
			int prof = (((b0 >> 4) & 1) << 1) | ((b0 >> 5) & 1);

			if (prof == 1 || prof == 3) {
				dev_err(core->dev,
					"VP9 Profile %d not supported (only Profile 0 and 2)\n",
					prof);
				amvdec_remove_ts(sess, vb->timestamp);
				v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
				amvdec_abort(sess);
				return 0;
			}
		}

		payload_size = vp9_update_header(core, vb);

		/* If unable to alter buffer to add headers */
		if (payload_size == 0) {
			amvdec_remove_ts(sess, vb->timestamp);
			v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);

			return 0;
		}
	}

	if (codec_ops->pre_queue) {
		ret = codec_ops->pre_queue(sess, vb);
		if (ret) {
			if (!skip_ts)
				amvdec_remove_ts(sess, vb->timestamp);
			v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
			return 0;
		}
		/* pre_queue may have modified the payload (e.g. prepended headers) */
		payload_size = vb2_get_plane_payload(vb, 0);
	}

	/*
	 * The truncated size comes back through the pointer, not through the
	 * vb2 plane payload: codec paths (vp9_update_header(), pre_queue) may
	 * have changed the packet length in payload_size without updating the
	 * plane payload, so re-reading vb2_get_plane_payload() here would hand
	 * the parser DMA a stale, shorter length.
	 */
	pad_size = esparser_pad_start_code(core, vb, &payload_size);
	if (pad_size < 0) {
		/*
		 * Buffer too full to append the search start code.
		 * Do NOT program the parser HW -- without the terminating
		 * start code, the DMA search engine would never fire the
		 * completion IRQ, leaving the parser wedged.  Drop the
		 * frame and let userspace continue with the next buffer.
		 */
		if (!skip_ts)
			amvdec_remove_ts(sess, vb->timestamp);
		v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
		return 0;
	}

	/*
	 * Only an imported DMABUF (V4L2_MEMORY_DMABUF) can be a CPU-cached
	 * allocation that vb2_dc_prepare() left unsynced; MMAP buffers are the
	 * driver's own coherent vb2_dma_contig allocation and must NOT be
	 * streaming-synced (see esparser_write_data(), KI-140).
	 */
	ret = esparser_write_data(core, phy, payload_size + pad_size,
				  vb->memory == V4L2_MEMORY_DMABUF);

	if (ret <= 0) {
		const u8 *p = vb2_plane_vaddr(vb, 0);
		/*
		 * Say why the start-code search ended (KI-172): ret 0 is the
		 * 200 ms timeout, <0 a signal; done=1 means the SC_FOUND IRQ
		 * arrived after the wait gave up.  The VLD FIFO registers are
		 * read only on a VDEC_1 session -- a VDEC_1 read with that
		 * engine powered off stops the bus (N26).
		 */
		bool vdec1 = sess->fmt_out->vdec_ops == &vdec_1_ops;

		dev_warn(core->dev,
			 "esparser: input parsing error ret=%d done=%d size=%u pad=%d off=%08x int=%08x ctrl=%08x pfifo=%08x/%08x vwp=%08x vififo=%08x/%08x/%08x head=%*ph\n",
			 ret, READ_ONCE(search_done), payload_size, pad_size, offset,
			 amvdec_read_parser(core, PARSER_INT_STATUS),
			 amvdec_read_parser(core, PARSER_CONTROL),
			 amvdec_read_parser(core, PFIFO_WR_PTR),
			 amvdec_read_parser(core, PFIFO_RD_PTR),
			 amvdec_read_parser(core, PARSER_VIDEO_WP),
			 vdec1 ? amvdec_read_dos(core, VLD_MEM_VIFIFO_LEVEL) : 0,
			 vdec1 ? amvdec_read_dos(core, VLD_MEM_VIFIFO_WP) : 0,
			 vdec1 ? amvdec_read_dos(core, VLD_MEM_VIFIFO_RP) : 0,
			 p ? 8 : 0, p);
		if (!skip_ts)
			amvdec_remove_ts(sess, vb->timestamp);
		v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);

		return 0;
	}

	/*
	 * Re-trigger VLD startcode search after data is written to the FIFO.
	 *
	 * The GXL firmware writes VLD_STATUS_CTRL=2 once during init (address
	 * 0x0008) to start VLD search.  On SM1 (12nm), VLD search completes
	 * on the empty FIFO before esparser pushes data, and the firmware's
	 * polling loop never re-triggers search.  On GXL (28nm), data arrives
	 * before VLD search finishes, so it works.
	 *
	 * Write VLD_STATUS_CTRL=2 after each buffer to re-arm the search.
	 * This is harmless on GXL (VLD is already searching or has data).
	 *
	 * Strike 6 (2026-04-28): added V4L2_PIX_FMT_H264_SLICE.  Per
	 * per-platform-h264-stateless-results.md "Strike 5", the libre
	 * H.264 stateless decode shows VLD_DECODE_STATUS=0 end-to-end
	 * even though VIFIFO contains the bitstream.  The libre firmware
	 * boot writes VLD_STATUS_CTRL=2 once at init like GXL BSP, but
	 * does not re-arm after data arrives (libre is resident-only by
	 * design and lacks the BSP overlay code that re-arms).  Extending
	 * this MPEG4/H263-style host-side re-arm to H.264 stateless
	 * provides the FSM kick that libre firmware lacks.
	 */
	/*
	 * MPEG2_SLICE: the SM1 VLD-search self-trigger DROPS at the 2-GOP boundary
	 * -- HW-instrumented + vicigol-co-validated 2026-06-23: at the intermittent
	 * stall VLD_STATUS_CTRL reads 0 (bit 0 clear) while the VIFIFO holds the
	 * next-GOP data (LEVEL=0xa3, RP frozen), so the firmware work-ready handler
	 * (gxl_mpeg12 0x0036) takes its 0x0039 bit0-clear branch to idle (0x0032)
	 * and never decodes (reproduced board-free by
	 * test_gxl_mpeg12_workready_gated_on_vld_status).  The MECHANISM is the VLD
	 * search not being re-armed at the GOP boundary.  BUT the blunt per-buffer
	 * host re-arm used for the discrete-buffer codecs above CRASHES the board
	 * when applied to the MPEG2 continuous session (HW 2026-06-23, kernel -01163:
	 * adding MPEG2_SLICE here rebooted the board on the FIRST clip, single-GOP
	 * included): the firmware OWNS the VLD across the continuous session (the
	 * stateful codec_mpeg12.c never re-arms per buffer), so a host
	 * VLD_STATUS_CTRL=2 write mid-session races the firmware's own VLD control.
	 * MPEG2_SLICE stays OUT of this blunt re-arm.  The correct fix is targeted:
	 * re-arm ONLY when the firmware is idle/not-scanning at the GOP boundary
	 * (gate on VLD_DECODE_STATUS != scanning, or feed an I-frame marker), OR fix
	 * the firmware's own self-re-arm (0x0486) timing on SM1.
	 */
	if (sess->fmt_out->pixfmt == V4L2_PIX_FMT_MPEG4 ||
	    sess->fmt_out->pixfmt == V4L2_PIX_FMT_H263 ||
	    sess->fmt_out->pixfmt == V4L2_PIX_FMT_H264_SLICE)
		amvdec_write_dos(core, VLD_STATUS_CTRL, 2);

	vdec_dbg_log_esparser(core, vb2_plane_vaddr(vb, 0), payload_size);

	atomic_inc(&sess->esparser_queued_bufs);
	v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_DONE);

	return 0;
}

void esparser_queue_all_src(struct work_struct *work)
{
	struct v4l2_m2m_buffer *buf, *n;
	struct amvdec_session *sess =
		container_of(work, struct amvdec_session, esparser_queue_work);

	/*
	 * N16b: esparser_queue_work is self-requeuing (codec_hevc's KI-061
	 * trailer reschedulers + the BUFEMPTY ISR).  vdec_close() runs
	 * cancel_work_sync(&esparser_queue_work) BEFORE the firmware is
	 * quiesced (that happens later, in v4l2_m2m_ctx_release ->
	 * vdec_stop_streaming), so a reschedule that fires after sess->aborted
	 * is set can escape the cancel and run once more -- after
	 * mutex_destroy(&sess->lock) + kfree(sess).  It then blocks forever on
	 * the destroyed/freed lock (no live holder), tripping hung_task_panic
	 * (== the residual "reset" on ipcm_A/DBLK_A through the shim).  Bail
	 * before touching the lock once the session is aborting: the src
	 * buffers are errored by the teardown's own drain, and the
	 * reschedulers below are likewise gated on !aborted so the requeue loop
	 * cannot outlive the cancel.  aborted is 0 during a normal EOS drain,
	 * so the N15 trailer recovery is unaffected.
	 */
	if (READ_ONCE(sess->aborted))
		return;

	mutex_lock(&sess->lock);
	v4l2_m2m_for_each_src_buf_safe(sess->m2m_ctx, buf, n) {
		/*
		 * Push this frame to the VIFIFO before checking should_stop.
		 * should_stop is set by DEC_CMD_STOP, which can race with the
		 * final QBUF: if the last source frame was queued just before
		 * STOP was processed, this work item may see should_stop=1 even
		 * though that final frame must still be pushed.  Checking before
		 * the push skips the frame, causing a permanent 1-frame EOS loss.
		 *
		 * Check should_stop AFTER the push instead: this ensures every
		 * frame that userspace queued before STOP is sent to the firmware.
		 */
		if (esparser_queue(sess, &buf->vb) < 0)
			break;

		/* A pending EOS needs every queued buffer fed first. */
		if (sess->should_stop && !sess->eos_pending)
			break;
	}

	/*
	 * DEC_CMD_STOP left its EOS sequence for us (KI-172): send it once the
	 * last queued access unit is in the VIFIFO, never ahead of it.
	 */
	if (sess->eos_pending && !v4l2_m2m_num_src_bufs_ready(sess->m2m_ctx)) {
		struct amvdec_codec_ops *codec_ops = sess->fmt_out->codec_ops;
		u32 len;
		const u8 *data = codec_ops->eos_sequence(&len);

		esparser_queue_eos(sess->core, data, len);
		sess->eos_pending = false;
		sess->last_irq_jiffies = get_jiffies_64();
	}
	mutex_unlock(&sess->lock);

	/*
	 * KI-115 tail: resume a firmware that the codec left PAUSED on a
	 * VIFIFO-underrun BUFEMPTY, now that source has been (re-)fed above the
	 * underrun mark.  No-op (one flag read) for codecs/sessions that did
	 * not pause.
	 */
	if (sess->fmt_out->codec_ops->resume_after_feed)
		sess->fmt_out->codec_ops->resume_after_feed(sess);
}

int esparser_power_up(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct amvdec_ops *vdec_ops = sess->fmt_out->vdec_ops;

	reset_control_reset(core->esparser_reset);
	amvdec_write_parser(core, PARSER_CONFIG,
			    (10 << PS_CFG_PFIFO_EMPTY_CNT_BIT) |
			    (1  << PS_CFG_MAX_ES_WR_CYCLE_BIT) |
			    (16 << PS_CFG_MAX_FETCH_CYCLE_BIT));

	amvdec_write_parser(core, PFIFO_RD_PTR, 0);
	amvdec_write_parser(core, PFIFO_WR_PTR, 0);

	amvdec_write_parser(core, PARSER_SEARCH_PATTERN,
			    ES_START_CODE_PATTERN);
	amvdec_write_parser(core, PARSER_SEARCH_MASK, ES_START_CODE_MASK);

	amvdec_write_parser(core, PARSER_CONFIG,
			    (10 << PS_CFG_PFIFO_EMPTY_CNT_BIT) |
			    (1  << PS_CFG_MAX_ES_WR_CYCLE_BIT) |
			    (16 << PS_CFG_MAX_FETCH_CYCLE_BIT) |
			    (2  << PS_CFG_STARTCODE_WID_24_BIT));

	amvdec_write_parser(core, PARSER_CONTROL,
			    (ES_SEARCH | ES_PARSER_START));

	amvdec_write_parser(core, PARSER_VIDEO_START_PTR, sess->vififo_paddr);
	amvdec_write_parser(core, PARSER_VIDEO_END_PTR,
			    sess->vififo_paddr + sess->vififo_size -
			    VIFIFO_END_PTR_ALIGN);
	amvdec_write_parser(core, PARSER_ES_CONTROL,
			    amvdec_read_parser(core, PARSER_ES_CONTROL) &
			    ~PARSER_ES_CTRL_VIDEO_MAN_RD);

	if (vdec_ops->conf_esparser)
		vdec_ops->conf_esparser(sess);

	/* Clear all pending parser interrupt status bits */
	amvdec_write_parser(core, PARSER_INT_STATUS, 0xffff);
	amvdec_write_parser(core, PARSER_INT_ENABLE,
			    BIT(PARSER_INT_HOST_EN_BIT));

	return 0;
}

int esparser_init(struct platform_device *pdev, struct amvdec_core *core)
{
	struct device *dev = &pdev->dev;
	int ret;
	int irq;

	irq = platform_get_irq_byname(pdev, "esparser");
	if (irq < 0)
		return irq;

	ret = devm_request_irq(dev, irq, esparser_isr, IRQF_SHARED,
			       "esparserirq", core);
	if (ret) {
		dev_err(dev, "Failed requesting ESPARSER IRQ\n");
		return ret;
	}

	core->esparser_reset =
		devm_reset_control_get_exclusive(dev, "esparser");
	if (IS_ERR(core->esparser_reset)) {
		dev_err(dev, "Failed to get esparser_reset\n");
		return PTR_ERR(core->esparser_reset);
	}

	/* HEVC front-end DMC pipeline reset (RESET7 bit 13).
	 * Optional -- only present in DT for G12A/G12B/SM1.
	 * BSP hevc_reset_core() toggles this for SM1; without it,
	 * HEVC decode on SM1 may soft-lock on first use after boot.
	 */
	core->hevcf_dmc_pipl_reset =
		devm_reset_control_get_optional_exclusive(dev, "hevcf_dmc_pipl");
	if (IS_ERR(core->hevcf_dmc_pipl_reset))
		core->hevcf_dmc_pipl_reset = NULL;

	return 0;
}
