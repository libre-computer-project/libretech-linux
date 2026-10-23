// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 Libre Computer
 * Author: Da Xue <da@libre.computer>
 *
 * === TEMPLATE, NOT A LIVE FILE ===
 *
 * This is a skeleton for the libre stateless H.264 decoder driver.
 * It lives in the amlogic/ design repo (not the kernel tree) until
 * M2 bringup copies it into
 * `drivers/staging/media/meson/vdec/codec_h264_libre.c` via a
 * worktree operation.
 *
 * Scope: SKELETON ONLY. Struct layout, codec_ops stubs, module
 * registration. No actual V4L2 Request API param handling yet --
 * that lands per-overlay during M4 bringup.
 *
 * Parallels: codec_mpeg12_libre.c (MPEG-1/2 libre driver) which
 * shipped on linux-6.12.y-lc via commit 14bf1286cef1 (2026-04-18).
 *
 * Firmware protocol: uses LMEM DMA for per-frame params (not direct
 * scratch register writes like MPEG-1/2). H.264 has ~40+ ctrl words
 * per slice; the LMEM window 0x100-0x1EF holds them per
 * h264-firmware-design.md §3.
 *
 * Command protocol (same as MPEG-1/2 libre):
 *   Driver packs params into DMA-coherent buffer, programs
 *   LMEM_DMA_ADR/COUNT/CTRL, then writes AV_SCRATCH_9 = 1.
 *   Firmware decodes one slice, writes AV_SCRATCH_9 = 2, fires
 *   MBOX1 IRQ. Firmware loops back to wait for next cmd.
 *
 * H.264 is SLICE-granular: one V4L2 Request = one slice, not one
 * frame. Multiple slices per frame mean multiple Requests. Driver
 * aggregates slices into a frame before dst_buf_done per
 * LAST_SLICE flag in LMEM[0x101] bit 11.
 *
 * Firmware: libre_<plat>_h264.bin (GXL / G12A+ binary families).
 * Built from libre_h264.S. Loaded per-session via request_firmware;
 * driver does the IMEM initial load (16 KiB) + stages the 36-KiB
 * mc region for runtime overlay swaps per §11 of the design spec.
 */

#include <media/v4l2-mem2mem.h>
#include <media/v4l2-ctrls.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>

#include "codec_h264_libre.h"
#include "dos_regs.h"
#include "vdec_helpers.h"

/* LMEM DMA base size: 4 KiB (2048 x 16-bit words) */
#define H264_LMEM_SIZE		SZ_4K

/*
 * Firmware mc region (DMA-coherent): 36 KiB total, same layout as
 * BSP vmh264.c:6215-6241 (ucode_copy) for compatibility. Only 3 of
 * the 5 slots are actively DMA'd by the libre firmware (HEADER,
 * DATA, SLICE); MMCO and LIST are zero-filled per §11.7 of the
 * design spec.
 */
#define H264_MC_TOTAL_SIZE	(SZ_32K + SZ_4K)	/* 36 KiB */
#define H264_MC_OFFSET_HEADER	0x0000
#define H264_MC_OFFSET_DATA	0x1000
#define H264_MC_OFFSET_MMCO	0x2000	/* zero-fill, libre v1 unused */
#define H264_MC_OFFSET_LIST	0x3000	/* zero-fill, libre v1 unused */
#define H264_MC_OFFSET_SLICE	0x4000
#define H264_MC_OFFSET_MAIN	0x5000	/* resident region source */

/*
 * Scratch register assignments (match libre_h264.S).
 *
 * No ABI handshake register (R1 dissolved 2026-04-22): the driver
 * already knows what firmware it loaded via request_firmware() +
 * amvdec_fw_identify() (KI-100 SHA256 match against fw_versions[]).
 * Firmware echoing a version number back would be tautological and
 * add a boot race. The version/quirks map lives in h264_fw_versions.
 */
#define LIBRE_REG_CMD		AV_SCRATCH_9	/* command/status */
#define LIBRE_REG_SLICE_INFO	AV_SCRATCH_E	/* per-slice locator (§8g addendum) */
#define LIBRE_REG_ERROR		AV_SCRATCH_F	/* error code (§8g taxonomy) */
#define LIBRE_REG_MC_BASE	AV_SCRATCH_G	/* mc region DMA base (firmware r25) */

/* AV_SCRATCH_9 protocol values (match libre_h264.S) */
#define LIBRE_CMD_IDLE		0
#define LIBRE_CMD_DECODE	1
#define LIBRE_CMD_DONE		2
#define LIBRE_CMD_ABORT		3

/*
 * Per-session state for the libre H.264 codec. Allocated at start,
 * freed at stop. Stored in sess->priv.
 */
struct codec_h264_libre {
	void	  *mc_vaddr;		/* DMA-coherent firmware mc region (36 KiB) */
	dma_addr_t mc_paddr;		/* Physical/DMA address for firmware r25 */

	void	  *lmem_vaddr;		/* DMA-coherent LMEM param buffer */
	dma_addr_t lmem_paddr;

	void	  *workspace_vaddr;	/* DMA-coherent workspace (CO_MV + scratch) */
	dma_addr_t workspace_paddr;	/* Packed into LMEM[0x1EC-0x1EE] per §3.2 */

	/* Per-frame aggregation across multi-slice requests. */
	u32	   slice_count;		/* slices received this frame */
	u32	   current_canvas;	/* output canvas for the frame-in-progress */
	bool	   frame_in_progress;	/* set between FIRST_SLICE and LAST_SLICE */
};

/*
 * H.264 workspace size: CO_MV (collocated motion vectors for B-frame
 * temporal direct mode) + slice-data scratch.
 *
 * CO_MV requirement: one MV pair per MB of prev picture. For Level 5.1
 * (4096x2304 = 36864 MBs) at 96 B/MB per BSP vmh264.c MB_MV_SIZE,
 * that's ~3.3 MB of MV storage per ref. Scratch is ~256 KiB.
 *
 * Pick SZ_1M as a starting allocation that handles <= 1080p. M4 must
 * compute this dynamically from max_mb_total or allocate per-stream.
 */
#define SIZE_WORKSPACE		SZ_1M

/*
 * Canvas helpers. H.264 uses up to 17 capture buffers (16 refs + 1
 * output). Canvas IDs are pre-filled in AV_SCRATCH_0..7 for first 8
 * buffers by amvdec_set_canvases(); the remaining 9 entries go to
 * sess-managed canvas slots indexed via fw_idx_to_vb2_idx.
 *
 * Parallels codec_mpeg12_libre.c's get_ref_canvas / get_dst_canvas
 * but extended for 17-entry DPB vs MPEG-1/2's 8.
 */

/*
 * Map a V4L2 vb2 capture buffer index to its firmware canvas ID
 * by scanning the fw_idx_to_vb2_idx table and reading back the
 * canvas from AV_SCRATCH_0..N.
 *
 * Returns 0xFF if no match (empty slot / unmapped buffer).
 */
static u32 get_dst_canvas(struct amvdec_session *sess, u32 vb2_idx)
{
	struct amvdec_core *core = sess->core;
	int i;

	for (i = 0; i < sess->num_dst_bufs; i++) {
		if (sess->fw_idx_to_vb2_idx[i] == vb2_idx)
			return amvdec_read_dos(core,
					       AV_SCRATCH_0 + i * 4);
	}

	return 0xff;
}

/*
 * Resolve a V4L2 Request reference_ts to a firmware canvas ID.
 * Mirrors codec_mpeg12_libre.c:get_ref_canvas but extended for the
 * full H.264 DPB (16 slots).
 *
 * Returns 0xFF if no capture buffer matches the timestamp -- the
 * firmware treats this as an empty DPB slot.
 */
static u32 resolve_ref_canvas(struct amvdec_session *sess, u64 reference_ts)
{
	struct vb2_queue *cap_q = &sess->m2m_ctx->cap_q_ctx.q;
	struct vb2_buffer *vb;

	vb = vb2_find_buffer(cap_q, reference_ts);
	if (!vb)
		return 0xff;

	return get_dst_canvas(sess, vb->index);
}

/*
 * Pack H.264 scaling matrices into LMEM[0x1A0..0x1E7].
 *
 * H.264 4:2:0 has up to 6 4x4 lists (intra/inter Y/Cb/Cr) + 2 8x8
 * lists (intra/inter Y). 4x4 entries are 8-bit; 8x8 entries are 8-bit.
 *
 * Per h264-firmware-design.md §3.2:
 *   0x1A0-0x1DF  scaling_4x4[0..3]  (first 4 lists)
 *   0x1E0-0x1E7  scaling_4x4[4..5]  (last 2 lists)
 *   0x1E8        scaling_8x8_present bitmask (bit 0,1 = list 0/1)
 *   8x8 lists    passed via workspace DMA region (not in LMEM)
 *
 * KNOWN_UNKNOWN: the spec is ambiguous about 4x4 packing density.
 * §3.2 implies the first 4 lists use 64 LMEM words for 4x16 entries
 * (1-per-word), while the last 2 use 8 words for 2x16 entries
 * (2-per-word). That asymmetry doesn't match any plausible firmware
 * read pattern; likely a spec bug. Verify against BSP firmware at
 * M4 bringup -- probably all 6 lists use 2-per-word = 48 total
 * LMEM words (6 * 8), freeing 40 words for 8x8 entries inline.
 *
 * Returns 0 on success, negative on error.
 */
static int pack_scaling_matrices(struct amvdec_session *sess, u32 *lmem)
{
	const struct v4l2_ctrl_h264_scaling_matrix *sm;
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_find(&sess->ctrl_handler,
			      V4L2_CID_STATELESS_H264_SCALING_MATRIX);
	if (!ctrl)
		return 0;		/* no scaling matrix -> defaults */

	sm = ctrl->p_cur.p;

	/* TODO M4: resolve §3.2 packing ambiguity via BSP firmware probe,
	 * then:
	 *   for (list = 0; list < 6; list++)
	 *       for (entry = 0; entry < 16; entry++)
	 *           lmem[0xA0 + list * N + entry / K] |=
	 *               sm->scaling_list_4x4[list][entry] << ((entry % K) * 8);
	 * where N = 16/K (2 if K=8-bit packed 2-per-word, etc).
	 */
	(void)sm;

	/* 0x1E8: scaling_8x8_present -- bit 0 = intra Y, bit 1 = inter Y. */
	/* lmem[0xE8] = scaling_8x8_present from PPS + presence flags */

	return 0;
}

/*
 * Flag bits at LMEM[0x101] (match h264-firmware-design.md §3.1).
 */
#define H264_FLAG_CABAC			BIT(0)	/* 0 = CAVLC */
#define H264_FLAG_WEIGHTED_PRED		BIT(1)
#define H264_FLAG_TRANSFORM_8X8		BIT(2)
#define H264_FLAG_SCALING_MATRIX_PRESENT BIT(3)
#define H264_FLAG_MVC_DEPENDENT		BIT(4)
#define H264_FLAG_INTERLACED		BIT(5)
#define H264_FLAG_MBAFF			BIT(6)
#define H264_FLAG_IDR			BIT(7)
#define H264_FLAG_SLICE_TYPE_SHIFT	8	/* bits [11:8] */
#define H264_FLAG_SLICE_TYPE_MASK	(0xfU << 8)
/*
 * Bits 12-13 allocated by driver for per-slice aggregation tracking
 * (§5.1 slice-granular state machine). Firmware doesn't use them
 * itself but includes them in slice_info echoed back at completion.
 * Spec §3.1 said bits 12-15 reserved; bits 14-15 still reserved.
 */
#define H264_FLAG_FIRST_SLICE		BIT(12)
#define H264_FLAG_LAST_SLICE		BIT(13)

/*
 * required_blocks bitmask for LMEM[0x108] per architecture §8f.
 * H.264 always needs VLD, IQIDCT, MC, DBLK, MCRCC, VIFIFO. PSCALE
 * off (not an MJPEG codec).
 *
 * M2 prerequisite: dos_regs.h currently only defines
 * `DOS_GCLK_EN0_VDEC1 = 0x3ff` as the blanket VDEC1 block mask.
 * Individual per-block bits (VLD, IQIDCT, etc.) are NOT defined.
 * Skeleton uses the blanket mask for simplicity. M2 should either:
 *   (a) add per-block bit defines to dos_regs.h + flip this to
 *       (VLD|IQIDCT|MC|DBLK|MCRCC|VIFIFO), leaving PSCALE off
 *   (b) accept 0x3ff (all blocks on) as a minor power cost for
 *       ~5% PSCALE gate overhead in H.264 sessions
 * Recommend (a); bit positions from libre_sm1_vdec1.S comments.
 */
#define H264_REQUIRED_BLOCKS	DOS_GCLK_EN0_VDEC1	/* 0x3ff (all VDEC1) */

/*
 * LMEM DMA registers for per-slice param push. dos_regs.h defines
 * LMEM_DMA_CTRL but not the ADR/COUNT siblings. M2 prerequisite:
 * add both to dos_regs.h as part of the libre H.264 patch series.
 * Addresses follow the adjacent DOS offset convention.
 */
#ifndef LMEM_DMA_ADR
#define LMEM_DMA_ADR		0x0d44		/* DOS byte */
#endif
#ifndef LMEM_DMA_COUNT
#define LMEM_DMA_COUNT		0x0d48
#endif

/*
 * Pack a 16-bit value at logical LMEM word index `idx` (relative to
 * the LMEM_DMA target base, which the driver sets to LMEM[0x100]).
 *
 * The hardware LMEM_DMA does an 8-byte (64-bit) endian-swap during
 * transfer.  Within each 8-byte chunk, the four 16-bit LMEM words
 * land in this order from the source u32 array `ws`:
 *   LMEM[chunk*4 + 0] = high16(ws[chunk*2 + 1])
 *   LMEM[chunk*4 + 1] = low16 (ws[chunk*2 + 1])
 *   LMEM[chunk*4 + 2] = high16(ws[chunk*2 + 0])
 *   LMEM[chunk*4 + 3] = low16 (ws[chunk*2 + 0])
 * Empirically characterized 2026-04-24 via M0.4 libre-fw-probe sweep
 * on SM1; full bit-field encoding in
 * decoder/firmware/amrisc-ip-characterization.md "4a. LMEM_DMA_CTRL".
 */
static inline void lmem_put16(u32 *ws, int idx, u16 val)
{
	int chunk = idx / 4;
	int pos = idx % 4;
	int ws_idx = chunk * 2 + (pos < 2 ? 1 : 0);
	bool hi = (pos == 0 || pos == 2);

	if (hi)
		ws[ws_idx] = (ws[ws_idx] & 0xffffU) | ((u32)val << 16);
	else
		ws[ws_idx] = (ws[ws_idx] & 0xffff0000U) | val;
}

/*
 * Pack V4L2 H.264 controls into LMEM[0x100..0x1EF] per
 * h264-firmware-design.md §3. Driver writes this buffer then
 * triggers LMEM DMA; firmware reads from its LMEM view.
 *
 * All LMEM words are 16-bit in hardware. Use lmem_put16(ws, idx, val)
 * to write `val` at LMEM[0x100 + idx] -- the helper handles the
 * 64-bit endian swap that LMEM_DMA performs in transit.
 *
 * NOTE: the legacy direct `lmem[idx] = val` writes (treating lmem as
 * a 32-bit-per-word array) DO NOT land where the firmware expects
 * after the endian swap.  Code below uses lmem_put16 for everything
 * the firmware reads at LIBRE_CMD_DECODE.
 */
static int pack_lmem_params(struct amvdec_session *sess,
			    struct vb2_buffer *dst_vb, u32 *lmem)
{
	struct codec_h264_libre *h264 = sess->priv;
	const struct v4l2_ctrl_h264_sps *sps;
	const struct v4l2_ctrl_h264_pps *pps;
	const struct v4l2_ctrl_h264_decode_params *decode_params;
	const struct v4l2_ctrl_h264_slice_params *slice_params;
	u32 flags = 0;
	int i;

	sps = v4l2_ctrl_find(&sess->ctrl_handler,
			     V4L2_CID_STATELESS_H264_SPS)->p_cur.p;
	pps = v4l2_ctrl_find(&sess->ctrl_handler,
			     V4L2_CID_STATELESS_H264_PPS)->p_cur.p;
	decode_params = v4l2_ctrl_find(&sess->ctrl_handler,
				       V4L2_CID_STATELESS_H264_DECODE_PARAMS)->p_cur.p;
	slice_params = v4l2_ctrl_find(&sess->ctrl_handler,
				      V4L2_CID_STATELESS_H264_SLICE_PARAMS)->p_cur.p;

	memset(lmem, 0, H264_LMEM_SIZE);

	/* =============================================================
	 * §3.1 Codec-agnostic header (0x100..0x117)
	 * Values written via lmem_put16 -- handles 64-bit endian swap
	 * transparently. See helper above for layout.
	 * ============================================================= */
	lmem_put16(lmem, 0x00, 2);			/* 0x100: codec_id = H.264 */

	if (pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE)
		flags |= H264_FLAG_CABAC;
	if (pps->flags & V4L2_H264_PPS_FLAG_WEIGHTED_PRED)
		flags |= H264_FLAG_WEIGHTED_PRED;
	if (pps->flags & V4L2_H264_PPS_FLAG_TRANSFORM_8X8_MODE)
		flags |= H264_FLAG_TRANSFORM_8X8;
	if (pps->flags & V4L2_H264_PPS_FLAG_SCALING_MATRIX_PRESENT)
		flags |= H264_FLAG_SCALING_MATRIX_PRESENT;
	if (!(sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY))
		flags |= H264_FLAG_INTERLACED;
	if (sps->flags & V4L2_H264_SPS_FLAG_MB_ADAPTIVE_FRAME_FIELD)
		flags |= H264_FLAG_MBAFF;
	if (decode_params->flags & V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC)
		flags |= H264_FLAG_IDR;
	flags |= (slice_params->slice_type & 0xf) << H264_FLAG_SLICE_TYPE_SHIFT;
	/* TODO MVC: flags |= H264_FLAG_MVC_DEPENDENT when MVC dep view active */

	/*
	 * FIRST_SLICE / LAST_SLICE flags for per-frame aggregation.
	 *
	 * FIRST_SLICE is straightforward: first_mb_in_slice == 0.
	 *
	 * LAST_SLICE is HARD. V4L2 stateless H.264 UAPI does NOT expose
	 * a "num MBs in this slice" field (verified against
	 * include/uapi/linux/v4l2-controls.h:1594 -- struct
	 * v4l2_ctrl_h264_slice_params has header_bit_size and
	 * first_mb_in_slice but nothing like num_macroblocks). So the
	 * driver cannot compute last_mb = first_mb + num_mbs at pack
	 * time -- the num_mbs information only exists in the slice
	 * bitstream, which the firmware parses.
	 *
	 * Three viable M4 strategies:
	 *   (a) Defer completion by one slice: LAST_SLICE of frame N is
	 *       detected when FIRST_SLICE of frame N+1 arrives. Driver
	 *       completes frame N's capture buffer at that point. Last
	 *       frame of stream needs EOS to flush.
	 *   (b) Track slice count in sess state + use SLICE_PARAMS
	 *       ref_pic_list to detect frame boundary heuristically.
	 *   (c) Have firmware report "mbs_processed >= mb_total" as a
	 *       per-slice completion flag in slice_info (currently
	 *       slice_info[31:16] = "MB count processed"; §5.3 already
	 *       defines it).
	 *
	 * Strategy (a) is what rkvdec and sunxi cedrus do. Strategy (c)
	 * requires firmware cooperation but is cleanest if we can add
	 * a bit to slice_info format.
	 *
	 * SKELETON assumption: single-slice streams (first_mb_in_slice
	 * == 0 AND no prior slice this frame). In that case
	 * FIRST_SLICE == LAST_SLICE == true. Set both. M4 picks one of
	 * (a)/(b)/(c) and replaces this block.
	 */
	if (slice_params->first_mb_in_slice == 0) {
		flags |= H264_FLAG_FIRST_SLICE;
		/* Single-slice assumption: every "first slice" is also
		 * the last slice. Multi-slice streams will need M4 to
		 * adopt strategy (a)/(b)/(c) above. */
		flags |= H264_FLAG_LAST_SLICE;
	}

	lmem_put16(lmem, 0x01, flags & 0xffffU);	/* 0x101: flags (low 16) */

	lmem_put16(lmem, 0x02, sps->pic_width_in_mbs_minus1 + 1);	/* 0x102: mb_width */
	lmem_put16(lmem, 0x03,
		(sps->pic_height_in_map_units_minus1 + 1) *
		((sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY) ? 1 : 2));
							/* 0x103: mb_height */

	/* 0x104-0x106: output + fwd + bwd canvas.
	 *
	 * Output comes from the current dst buffer (dst_vb passed in --
	 * caller got it via v4l2_m2m_next_dst_buf; same pattern as
	 * codec_mpeg12_libre:pre_queue). fwd/bwd from the first active
	 * L0/L1 entry for P/B slices. Intra slices leave fwd/bwd=0.
	 */
	lmem_put16(lmem, 0x04, get_dst_canvas(sess, dst_vb->index));
	lmem_put16(lmem, 0x05,
		(slice_params->slice_type == V4L2_H264_SLICE_TYPE_I ||
		 slice_params->slice_type == V4L2_H264_SLICE_TYPE_SI) ? 0
		: resolve_ref_canvas(sess, decode_params->dpb[0].reference_ts));
	lmem_put16(lmem, 0x06,
		(slice_params->slice_type == V4L2_H264_SLICE_TYPE_B ||
		 slice_params->slice_type == V4L2_H264_SLICE_TYPE_SI) ? 0
		: resolve_ref_canvas(sess, decode_params->dpb[1].reference_ts));
	/* 0x107 reserved per §8f -- already 0 from memset */
	lmem_put16(lmem, 0x08, H264_REQUIRED_BLOCKS);
	/* 0x109..0x10C reserved */

	/* 0x10D..0x10E: source buffer DDR bounds.  Filled in pre_queue()
	 * from the source vb dma addr + payload size.  Leave 0 here. */

	/* 0x10F: header_bit_size.  V4L2 stateless H.264 spec says
	 * userspace pre-parses the slice header and reports its size in
	 * bits via slice_params->header_bit_size.  The firmware uses
	 * this to position the VLD bit pointer past the header before
	 * kick_decode (M4.1.7-blocker-1) -- the OUTPUT buffer carries
	 * the FULL NAL (header + payload), and VLD would otherwise treat
	 * header bits as MB syntax.
	 *
	 * Semantics revision 2026-04-25: design doc §3.1 originally said
	 * "slice_bit_size (slice data size in bits)" but the M4.1.2
	 * design code-comment said "header_bit_size".  M4.1.7a wired
	 * `src_size * 8` (total bits) which doesn't match either
	 * interpretation usefully.  Picked header_bit_size because that's
	 * what the bit-skip needs; total bits isn't needed (HW signals
	 * end via VLD_STATUS_CTRL bit 6).  Updated h264-firmware-design.md
	 * §3.1 + §5.2 in the same commit. */
	lmem_put16(lmem, 0x0F, slice_params->header_bit_size & 0xffffU);

	/* 0x110..0x117: picture dimensions + profile/level. */
	{
		u16 w_mbs = sps->pic_width_in_mbs_minus1 + 1;
		u16 h_mbs = (sps->pic_height_in_map_units_minus1 + 1) *
			    ((sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY) ? 1 : 2);

		lmem_put16(lmem, 0x10, w_mbs * 16);
		lmem_put16(lmem, 0x11, h_mbs * 16);
	}
	lmem_put16(lmem, 0x12, sps->bit_depth_luma_minus8 + 8);
	lmem_put16(lmem, 0x13, sps->chroma_format_idc);
	lmem_put16(lmem, 0x14, sps->profile_idc);
	lmem_put16(lmem, 0x15, sps->level_idc);

	/* =============================================================
	 * §3.2 H.264-specific extended window (0x118..0x1EF)
	 * ============================================================= */

	/* 0x118..0x119: pic_order_cnt_lsb (split hi/lo for 32-bit values).
	 *
	 * UAPI fix (2026-04-22 audit): frame_num, idr_pic_id, nal_ref_idc,
	 * delta_pic_order_cnt0, pic_order_cnt_bit_size are in
	 * decode_params, NOT slice_params. Verified against
	 * include/uapi/linux/v4l2-controls.h.
	 */
	lmem[0x18] = decode_params->pic_order_cnt_bit_size ?
		     (u16)decode_params->delta_pic_order_cnt0 : 0;
	lmem[0x19] = 0;					/* TODO: top-field POC vs bottom when interlaced */

	lmem[0x1A] = decode_params->frame_num;
	lmem[0x1B] = decode_params->idr_pic_id;
	lmem[0x1C] = sps->pic_order_cnt_type;
	lmem[0x1D] = slice_params->num_ref_idx_l0_active_minus1 + 1;
	lmem[0x1E] = slice_params->num_ref_idx_l1_active_minus1 + 1;
	lmem[0x1F] = decode_params->nal_ref_idc;

	lmem[0x20] = slice_params->slice_qp_delta;
	lmem[0x21] = slice_params->slice_alpha_c0_offset_div2;
	lmem[0x22] = slice_params->slice_beta_offset_div2;
	lmem[0x23] = slice_params->disable_deblocking_filter_idc;
	lmem[0x24] = slice_params->cabac_init_idc;

	/* first_mb_in_slice: 24-bit value split high/low */
	lmem[0x25] = (slice_params->first_mb_in_slice >> 16) & 0xff;
	lmem[0x26] = slice_params->first_mb_in_slice & 0xffff;
	/* 0x127 reserved (upper nibble of first_mb_in_slice_lo unused) */

	/* 0x128..0x12F reserved for firmware scratch */

	/* =============================================================
	 * Reference lists (0x130..0x18F)
	 * TODO: walk decode_params->dpb[i], match reference_ts to a
	 * capture buffer's canvas ID via get_dst_canvas(). Handle
	 * long-term vs short-term refs; empty slots = 0xFF.
	 * ============================================================= */
	for (i = 0; i < V4L2_H264_NUM_DPB_ENTRIES; i++) {
		u32 slot = 0xffU;			/* empty marker */
		const struct v4l2_h264_dpb_entry *dpb = &decode_params->dpb[i];

		if (dpb->flags & V4L2_H264_DPB_ENTRY_FLAG_ACTIVE)
			slot = resolve_ref_canvas(sess, dpb->reference_ts);

		/* TODO M4: build ref_list_l0 and ref_list_l1 from SLICE_PARAMS
		 * ref_pic_list_modification (slice_params has ref_pic_list0[]
		 * and ref_pic_list1[] per V4L2 UAPI). For now mirror the raw
		 * DPB in both lists -- correct for the simple case where the
		 * slice uses the default-constructed list. */
		lmem[0x30 + i] = slot;			/* 0x130..0x13F: ref_list_l0 */
		lmem[0x40 + i] = slot;			/* 0x140..0x14F: ref_list_l1 */
		lmem[0x50 + i] = dpb->top_field_order_cnt & 0xffff;	/* 0x150..0x15F */
		lmem[0x60 + i] = dpb->bottom_field_order_cnt & 0xffff;	/* 0x160..0x16F */
		/* 0x170..0x18F: field flags. 0=top, 1=bot, 2=frame. For
		 * progressive streams the frame flag is correct; interlaced
		 * needs per-ref top/bot decoding from dpb->flags. */
		lmem[0x70 + i] = (dpb->flags & V4L2_H264_DPB_ENTRY_FLAG_FIELD) ? 0 : 2;
		lmem[0x80 + i] = (dpb->flags & V4L2_H264_DPB_ENTRY_FLAG_FIELD) ? 1 : 2;
	}

	/* 0x190..0x19F: long_term_flags bitmask (1 bit per slot) */
	{
		u32 lt = 0;

		for (i = 0; i < V4L2_H264_NUM_DPB_ENTRIES; i++) {
			if (decode_params->dpb[i].flags &
			    V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM)
				lt |= BIT(i);
		}
		lmem[0x90] = lt & 0xffff;
		lmem[0x91] = lt >> 16;
	}

	/* =============================================================
	 * 0x1A0..0x1E7: scaling_4x4 matrices (6 lists x 16 entries).
	 * See pack_scaling_matrices() above for packing details +
	 * KNOWN_UNKNOWN around §3.2 packing density.
	 * ============================================================= */
	if (flags & H264_FLAG_SCALING_MATRIX_PRESENT)
		pack_scaling_matrices(sess, lmem);

	/* 0x1E8: scaling_8x8_present bitmask */
	lmem[0xE8] = 0;	/* TODO: PPS transform_8x8_mode bits */

	/* 0x1E9..0x1EB: pred_weights DMA address (3 words = 48-bit) */
	/* TODO M4: alloc weighted_pred region via dma_alloc_coherent
	 * when flags & H264_FLAG_WEIGHTED_PRED. For now zero -- firmware
	 * falls back to PPS/slice default weights when flag bit 1 = 0. */

	/* 0x1EC..0x1EE: workspace DMA address (CO_MV + slice scratch).
	 * 3 LMEM words (16-bit each) = 48-bit address; current ARM64
	 * DMA is 32-bit so we pack {low16, high16, 0}. If a future
	 * platform uses >32-bit DMA, the third word carries bits 32-47. */
	lmem[0xEC] = (u32)h264->workspace_paddr & 0xffff;
	lmem[0xED] = ((u32)h264->workspace_paddr >> 16) & 0xffff;
	lmem[0xEE] = 0;

	return 0;
}

/*
 * Start of session: allocate DMA buffers, load firmware, initialize
 * HW, advertise the firmware mc base via AV_SCRATCH_G.
 */
static int codec_h264_libre_start(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_h264_libre *h264;
	int ret;

	h264 = kzalloc(sizeof(*h264), GFP_KERNEL);
	if (!h264)
		return -ENOMEM;

	/* Firmware mc region for overlay swap staging. Driver writes the
	 * firmware blob contents into here during firmware load; firmware
	 * DMAs slots 0x0000 (HEADER) / 0x1000 (DATA) / 0x4000 (SLICE) back
	 * into IMEM on demand. */
	h264->mc_vaddr = dma_alloc_coherent(core->dev, H264_MC_TOTAL_SIZE,
					    &h264->mc_paddr, GFP_KERNEL);
	if (!h264->mc_vaddr) {
		ret = -ENOMEM;
		goto free_h264;
	}

	/* LMEM param buffer for per-slice param push. */
	h264->lmem_vaddr = dma_alloc_coherent(core->dev, H264_LMEM_SIZE,
					      &h264->lmem_paddr, GFP_KERNEL);
	if (!h264->lmem_vaddr) {
		ret = -ENOMEM;
		goto free_mc;
	}

	/* Workspace: CO_MV storage + slice scratch. Driver tells firmware
	 * its base address via LMEM[0x1EC-0x1EE]; firmware uses it for
	 * collocated MV storage (B-frame temporal direct) and internal
	 * slice-data scratch. */
	h264->workspace_vaddr = dma_alloc_coherent(core->dev, SIZE_WORKSPACE,
						   &h264->workspace_paddr,
						   GFP_KERNEL);
	if (!h264->workspace_vaddr) {
		ret = -ENOMEM;
		goto free_lmem;
	}

	/* TODO M4: parse firmware blob header, validate magic + version,
	 * memcpy per-role slots (HEADER/DATA/MMCO/LIST/SLICE/MAIN) from
	 * blob into h264->mc_vaddr, run initial 16-KiB IMEM DMA
	 * (aml_loadmc_vdec-equivalent per amvdec.c:386-446). */

	/*
	 * Full IQIDCT/MC/VLD reset (mirror codec_mpeg12_libre.c:203-225).
	 * Save POWER_CTL_VLD first; the reset pulse clears it but driver
	 * writes H.264-specific POWER_CTL_VLD below (cabac_step_2 etc).
	 */
	{
		u32 save_vld = amvdec_read_dos(core, POWER_CTL_VLD);

		amvdec_write_dos(core, DOS_SW_RESET0,
				 DOS_SW_RESET0_MDEC | DOS_SW_RESET0_VLD);
		amvdec_write_dos(core, DOS_SW_RESET0, 0);
		amvdec_read_dos(core, DOS_SW_RESET0);
		amvdec_read_dos(core, DOS_SW_RESET0);
		amvdec_read_dos(core, DOS_SW_RESET0);
		amvdec_write_dos(core, DOS_SW_RESET0,
				 DOS_SW_RESET0_MDEC | DOS_SW_RESET0_VLD);
		amvdec_write_dos(core, DOS_SW_RESET0, 0);
		amvdec_write_dos(core, DOS_SW_RESET0, DOS_SW_RESET0_MC);
		amvdec_write_dos(core, DOS_SW_RESET0, 0);
		amvdec_read_dos(core, DOS_SW_RESET0);
		amvdec_read_dos(core, DOS_SW_RESET0);
		amvdec_read_dos(core, DOS_SW_RESET0);
		amvdec_write_dos(core, MDEC_SW_RESET, MDEC_SW_RESET_MDEC_CORE);
		amvdec_write_dos(core, MDEC_SW_RESET, 0);

		amvdec_write_dos(core, POWER_CTL_VLD, save_vld);
	}

	/*
	 * POWER_CTL_VLD for H.264. CABAC decode is HARDWARE via the
	 * `cabac_step_2` sub-block (confirmed 2026-04-22 BSP audit) --
	 * enabled by CLEARING bit 10 (0 = enable, 1 = disable).
	 * Companion bits: 6=h264_000003_en (start-code emulation
	 * prevention strip), 9=viff_drop_flag_en.
	 *
	 * Mirrors BSP vmh264.c:9892-9894 which writes
	 * `(0 << 10) | (1 << 9) | (1 << 6)`. MVC variant disables CABAC
	 * (bit 10 = 1); libre follows mainline H.264 not MVC.
	 */
	amvdec_write_dos(core, POWER_CTL_VLD,
			 amvdec_read_dos(core, POWER_CTL_VLD) |
			 (0 << 10) |	/* cabac_step_2 enable */
			 (1 << 9)  |	/* viff_drop_flag_en */
			 (1 << 6));	/* h264_000003_en */

	/* mc region base: firmware reads r25 = AV_SCRATCH_G at first
	 * overlay swap. Must be set BEFORE MPSR=1 (see §11.3 of design). */
	amvdec_write_dos(core, LIBRE_REG_MC_BASE, h264->mc_paddr);
	amvdec_write_dos(core, LIBRE_REG_CMD, LIBRE_CMD_IDLE);
	amvdec_write_dos(core, LIBRE_REG_ERROR, 0);

	/* Disable PSCALE (MJPEG-specific; H.264 doesn't use it). §8i
	 * shared-latch discipline: any prior MJPEG session leaves
	 * PSCALE_CTRL dirty; vdec_1_start clears it but we defensively
	 * re-clear for the session. */
	amvdec_write_dos(core, PSCALE_CTRL, 0);

	/* Set up canvases for output. H.264 needs up to 17 ref + 1 output
	 * per H.264 spec. TODO M4: wire amvdec_set_canvases for 17 dst. */
	ret = amvdec_set_canvases(sess, (u32[]){ AV_SCRATCH_0, 0 },
				  (u32[]){ 17, 0 });
	if (ret) {
		dev_err(core->dev, "h264 libre: canvas setup failed: %d\n", ret);
		goto free_workspace;
	}

	/* No ABI handshake readback -- driver already verified the
	 * firmware blob via amvdec_fw_identify() at request_firmware
	 * time. The fw_versions[] table is the source of truth for
	 * which firmware is loaded; quirk flags propagate via
	 * sess->fw_match (KI-100 infrastructure). */

	sess->keyframe_found = 0;
	sess->priv = h264;

	return 0;

free_workspace:
	dma_free_coherent(core->dev, SIZE_WORKSPACE,
			  h264->workspace_vaddr, h264->workspace_paddr);
free_lmem:
	dma_free_coherent(core->dev, H264_LMEM_SIZE,
			  h264->lmem_vaddr, h264->lmem_paddr);
free_mc:
	dma_free_coherent(core->dev, H264_MC_TOTAL_SIZE,
			  h264->mc_vaddr, h264->mc_paddr);
free_h264:
	kfree(h264);
	return ret;
}

static int codec_h264_libre_stop(struct amvdec_session *sess)
{
	struct codec_h264_libre *h264 = sess->priv;
	struct amvdec_core *core = sess->core;

	/*
	 * codec_ops->stop is called from BOTH vdec.c:613 (codec
	 * unwind) AND vdec_1.c:649 (engine unwind) -- the latter
	 * gates on sess->priv being non-NULL as a defensive
	 * second-call guard. Without NULLing sess->priv after
	 * kfree(), the second call dereferences freed memory and
	 * dma_free_coherent's the same buffers again, producing a
	 * CMA double-free WARN that panics the board on
	 * hung_task_panic boards. Pattern from codec_h264_mvc.c:1466.
	 */
	if (h264->workspace_vaddr) {
		dma_free_coherent(core->dev, SIZE_WORKSPACE,
				  h264->workspace_vaddr, h264->workspace_paddr);
		h264->workspace_vaddr = NULL;
	}
	if (h264->lmem_vaddr) {
		dma_free_coherent(core->dev, H264_LMEM_SIZE,
				  h264->lmem_vaddr, h264->lmem_paddr);
		h264->lmem_vaddr = NULL;
	}
	if (h264->mc_vaddr) {
		dma_free_coherent(core->dev, H264_MC_TOTAL_SIZE,
				  h264->mc_vaddr, h264->mc_paddr);
		h264->mc_vaddr = NULL;
	}

	kfree(h264);
	sess->priv = NULL;
	return 0;
}

/*
 * pre_queue: called per Request before firmware decode. Pack ctrl
 * values into LMEM, trigger LMEM DMA, write AV_SCRATCH_9 = DECODE.
 *
 * @vb: OUTPUT (source = bitstream) buffer for this Request. The
 *      CAPTURE (destination) buffer is retrieved via
 *      v4l2_m2m_next_dst_buf(sess->m2m_ctx) -- same pattern as
 *      codec_mpeg12_libre:pre_queue. 2026-04-22 discovery: the
 *      framework hook for dst buffer lookup has always existed;
 *      earlier §10.4 claim that it was missing was wrong.
 */
static int codec_h264_libre_pre_queue(struct amvdec_session *sess,
				      struct vb2_buffer *vb)
{
	struct codec_h264_libre *h264 = sess->priv;
	struct amvdec_core *core = sess->core;
	struct vb2_v4l2_buffer *dst_buf;
	u32 *lmem = h264->lmem_vaddr;
	int ret;

	dst_buf = v4l2_m2m_next_dst_buf(sess->m2m_ctx);
	if (!dst_buf) {
		dev_err(core->dev, "h264 libre: no dst buffer\n");
		return -EINVAL;
	}

	ret = pack_lmem_params(sess, &dst_buf->vb2_buf, lmem);
	if (ret)
		return ret;

	/*
	 * M4.1.7a: wire vififo_start/end from the source buffer into
	 * LMEM[0x10D..0x10E].  pre_queue's `vb` argument is the SRC
	 * buffer (per esparser.c:esparser_queue flow); pack_lmem_params
	 * left these slots zero with a TODO because it took dst_vb only.
	 * Now they get the real DDR bounds of the bitstream payload.
	 *
	 * LMEM[0x10F] (header_bit_size) is set by pack_lmem_params from
	 * slice_params->header_bit_size (semantics revised 2026-04-25
	 * per HIGH-blocker-2).
	 *
	 * The firmware kick_decode gate (libre_h264.S) refuses to fire
	 * POWER_CTL_VLD bit 13 unless vififo_start != 0 AND
	 * vififo_start != vififo_end.  Stub-test V4L2 buffers carry 0
	 * bytes of payload -> end == start -> kick still skipped.
	 * Real bitstream feeds end > start -> kick fires.  See
	 * h264-firmware-design.md §3.1 for the LMEM contract.
	 */
	{
		dma_addr_t src_phy = vb2_dma_contig_plane_dma_addr(vb, 0);
		u32 src_size = vb2_get_plane_payload(vb, 0);
		u32 src_end = src_phy + src_size;

		/*
		 * Pack 32-bit DDR addresses across TWO LMEM words each.
		 * AMRISC `lda` reads a single 16-bit LMEM word, so the
		 * firmware combines low+high halves via shli + or to
		 * recover the full 32-bit pointer.  Slot assignment
		 * (M4.2 vififo-32bit revision; the 0x10A/0x10B slots are
		 * in the "0x109..0x10C reserved" range of the M4.1 design):
		 *   LMEM[0x10D] = vififo_start low 16
		 *   LMEM[0x10A] = vififo_start high 16
		 *   LMEM[0x10E] = vififo_end   low 16
		 *   LMEM[0x10B] = vififo_end   high 16
		 * Firmware-side recovery: see libre_h264.S program_vld
		 * "Read vififo_start / end from LMEM contract".
		 */
		lmem_put16(lmem, 0x0D, src_phy & 0xffffU);
		lmem_put16(lmem, 0x0A, (src_phy >> 16) & 0xffffU);
		lmem_put16(lmem, 0x0E, src_end & 0xffffU);
		lmem_put16(lmem, 0x0B, (src_end >> 16) & 0xffffU);
	}

	/*
	 * LMEM DMA: copy host-packed params into firmware's LMEM view.
	 *
	 * LMEM_DMA_CTRL encoding (empirically characterized 2026-04-24
	 * via M0.4 libre-fw-probe sweep on SM1; see
	 * decoder/firmware/amrisc-ip-characterization.md "4a. LMEM_DMA_CTRL
	 * bit-field encoding"):
	 *   bit 15    = START / busy (poll for clear)
	 *   bit 14    = direction: 1 = mem -> LMEM, 0 = LMEM -> mem
	 *   bits 11:0 = LMEM start word offset (LMEM is 2048 16-bit words)
	 *
	 * We write to LMEM[0x100+] which is the parameter region per §3.
	 * COUNT is in 16-bit LMEM words (not 32-bit). The DMA does an
	 * 8-byte (64-bit) endian-swap during transfer; pack_lmem_params
	 * has been arranged so codec_id lands at LMEM[0x100] correctly
	 * (lmem_put16 helper handles the layout).
	 *
	 * Timeout: BSP has no timeout on this poll. We add a 1-ms
	 * bounded wait to prevent an infinite hang if the hardware
	 * goes unresponsive; pre_queue is a hot path and a hang here
	 * would lock the decoder thread permanently.
	 */
	/*
	 * COUNT is in 16-bit LMEM words. Start offset (CTRL bits 11..0) is
	 * 0x100, so the DMA writes LMEM[0x100..0x100+count-1].  Cap count
	 * so the end stays within LMEM (2048 16-bit words total) AND
	 * within the workspace allocation. The libre H.264 firmware only
	 * reads LMEM[0x100..0x117] for the codec-agnostic header and
	 * LMEM[0x118..0x1EF] for the H.264-specific extended window
	 * (max 0xF0 = 240 16-bit words = 480 bytes). Use 240 to match.
	 *
	 * Previous H264_LMEM_SIZE/2 = 2048 attempted to write LMEM[0x100..0x8FF]
	 * which goes past the LMEM end at 0x7FF, causing the DMA hardware to
	 * never set bit 15 to 0 (observed timeout 2026-04-25 on SM1).
	 */
	amvdec_write_dos(core, LMEM_DMA_ADR, h264->lmem_paddr);
	amvdec_write_dos(core, LMEM_DMA_COUNT, 240);                  /* 16-bit words; LMEM[0x100..0x1EF] */
	amvdec_write_dos(core, LMEM_DMA_CTRL, 0xC100);                /* mem->LMEM[0x100] */

	{
		u32 ctrl;
		int spin = 10000;		/* ~1 ms at typical DOS clock */

		do {
			ctrl = amvdec_read_dos(core, LMEM_DMA_CTRL);
			if (!(ctrl & BIT(15)))
				break;
			udelay(1);
		} while (--spin);

		if (ctrl & BIT(15)) {
			dev_err(core->dev,
				"h264 libre: LMEM DMA timeout (CTRL=0x%08x)\n",
				ctrl);
			return -EIO;
		}
	}

	/* Signal firmware to start decoding */
	amvdec_write_dos(core, LIBRE_REG_CMD, LIBRE_CMD_DECODE);

	return 0;
}

static irqreturn_t codec_h264_libre_isr(struct amvdec_session *sess)
{
	return IRQ_WAKE_THREAD;
}

static irqreturn_t codec_h264_libre_threaded_isr(struct amvdec_session *sess)
{
	struct codec_h264_libre *h264 = sess->priv;
	struct amvdec_core *core = sess->core;
	u32 cmd, error, slice_info, flags_echo;
	u32 slice_err;
	bool last_slice;

	amvdec_write_dos(core, ASSIST_MBOX1_CLR_REG, 1);

	cmd = amvdec_read_dos(core, LIBRE_REG_CMD);

	/*
	 * Firmware cmd value taxonomy (per libre_h264.S):
	 *   LIBRE_CMD_DONE (2)  -- normal completion; continue to slice-info read
	 *   0xFF                -- error exit (cmd_error / codec_unsupported in
	 *                         firmware). AV_SCRATCH_F holds the specific code:
	 *                         0xFF = unknown cmd, 0xFE = unsupported codec_id,
	 *                         0x01-0x0F = recoverable, 0x10-0x1F = session-fatal
	 *                         (per architecture §8g error-code taxonomy).
	 *   anything else       -- spurious wakeup (e.g., IRQ fired while cmd
	 *                         register in transition). Log + drop.
	 *
	 * Without the error path (cmd == 0xFF), a firmware-reported error leaves
	 * the request hanging forever -- threaded_isr returns IRQ_HANDLED without
	 * calling v4l2_m2m_buf_done or job_finish, and userspace waits on
	 * POLLIN/DQBUF indefinitely. With v4l2_ctrl_request_complete now called
	 * in device_run (mainline 4fbe3798c629), POLLPRI on req_fd fires
	 * regardless of outcome, but the buffers still need explicit ERROR
	 * completion so userspace sees V4L2_BUF_FLAG_ERROR on DQBUF.
	 */
	if (cmd == 0xFF) {
		error = amvdec_read_dos(core, LIBRE_REG_ERROR);
		dev_err(core->dev,
			"h264 libre: firmware cmd-error (AV_SCRATCH_F=0x%02x)\n",
			error);
		/*
		 * amvdec_abort halts the firmware (MPSR=0), calls
		 * vb2_queue_error on BOTH queues (marks all pending buffers
		 * ERROR + wakes pollers), clears core->cur_sess so any
		 * in-flight threaded_isr on other sessions bails. Matches
		 * the existing error patterns at lines 819-823 (firmware
		 * error with cmd=DONE) and 833-839 (slice_err). Session is
		 * terminated; userspace must STREAMOFF and reopen to
		 * recover.
		 */
		amvdec_abort(sess);
		return IRQ_HANDLED;
	}

	if (cmd != LIBRE_CMD_DONE) {
		dev_warn(core->dev,
			 "h264 libre: spurious threaded_isr -- cmd=0x%02x (expected DONE=2)\n",
			 cmd);
		return IRQ_HANDLED;
	}

	/*
	 * AV_SCRATCH_F (LIBRE_REG_ERROR) layout per libre_h264.S signal_done
	 * (lines 579-604).  The firmware ALWAYS writes diagnostic snapshot
	 * bits even on success:
	 *   [7:0]   status code (0=OK, 0x10-0x1f session-fatal, 0x01-0x0f recoverable)
	 *   [11:8]  VLD_DECODE_STATUS bits[3:0] (FSM state at signal_done)
	 *   [15:12] DBLK_STATUS bits[3:0]
	 *   [23:16] MDEC_PIC_DC_STATUS [7:0]
	 *   [31:24] reserved
	 *
	 * Strike 7 (2026-04-28): only treat the low byte as the actual
	 * error -- pre-strike-7 the driver checked the whole u32 and
	 * spuriously errored on success runs where the diagnostic snapshot
	 * was non-zero (e.g. 0x500 = status=0 + VLD FSM bits[3:0]=5
	 * scanning).  Log the full register at info level for post-mortem.
	 */
	error = amvdec_read_dos(core, LIBRE_REG_ERROR);
	if (error & 0xFF) {
		dev_err(core->dev, "h264 libre: firmware error 0x%x (status=0x%02x)\n",
			error, error & 0xFF);
		amvdec_abort(sess);
		return IRQ_HANDLED;
	}
	if (error)
		dev_info(core->dev, "h264 libre: signal_done diag=0x%x (vld_fsm=0x%x dblk=0x%x mdec=0x%x)\n",
			 error,
			 (error >> 8) & 0xF,
			 (error >> 12) & 0xF,
			 (error >> 16) & 0xFF);

	/*
	 * SLICE_INFO format per design §5.3:
	 *   [7:0]   slice_idx within current frame
	 *   [15:8]  slice error (0 = OK, see §6 for codes)
	 *   [31:16] MB count processed (for partial slice diagnostics)
	 */
	slice_info = amvdec_read_dos(core, LIBRE_REG_SLICE_INFO);
	slice_err = (slice_info >> 8) & 0xff;

	if (slice_err) {
		dev_err(core->dev,
			"h264 libre: slice %u error 0x%02x mbs=%u\n",
			slice_info & 0xff, slice_err,
			(slice_info >> 16) & 0xffff);
		amvdec_abort(sess);
		return IRQ_HANDLED;
	}

	h264->slice_count++;

	/*
	 * Determine LAST_SLICE by reading back the flags we packed at
	 * LMEM[0x101]. Firmware doesn't interpret these bits; we echo
	 * our own intent. Avoids tracking LAST_SLICE separately in
	 * struct state (which would race if pre_queue fires before the
	 * prior slice's threaded_isr completes).
	 */
	flags_echo = ((u32 *)h264->lmem_vaddr)[0x01];
	last_slice = !!(flags_echo & H264_FLAG_LAST_SLICE);

	dev_dbg(core->dev,
		"h264 libre: slice %u of frame (info=0x%08x last=%d)\n",
		h264->slice_count - 1, slice_info, last_slice);

	if (!last_slice) {
		/*
		 * Intermediate slice: framework's v4l2_ctrl_request_complete
		 * hook releases the Request; we just ack MBOX and wait for
		 * the next slice's pre_queue. Do NOT call m2m_job_finish
		 * (that would tear down the Request state mid-frame).
		 */
		return IRQ_HANDLED;
	}

	/*
	 * Last slice of a frame: complete the request by marking both src
	 * and dst buffers DONE, then signal m2m that the job is complete.
	 *
	 * Why explicit v4l2_m2m_buf_done here (not amvdec_dst_buf_done_idx
	 * like codec_mpeg12_libre): mpeg12 uses the framework's
	 * fw_idx_to_vb2_idx mapping to translate firmware-side canvas IDs
	 * back to vb2 buffer indices -- because mpeg12 firmware writes the
	 * output canvas ID into FRAME_INFO and that may not match the
	 * driver's last-queued dst (multi-frame queue depth). For libre
	 * H.264 stateless we DO know which dst buffer the firmware wrote
	 * to: it's whichever was queued by the current Request (one per
	 * device_run cycle). v4l2_m2m_dst_buf_remove pops the head of
	 * the dst queue, which IS that buffer.
	 *
	 * Marking dst DONE fires vdec.c:vdec_vb2_buf_request_complete ->
	 * v4l2_ctrl_request_complete -> userspace poll(POLLPRI) returns.
	 * Without this call, the request stays pending forever.
	 */
	{
		struct vb2_v4l2_buffer *src_vbuf, *dst_vbuf;

		src_vbuf = v4l2_m2m_src_buf_remove(sess->m2m_ctx);
		dst_vbuf = v4l2_m2m_dst_buf_remove(sess->m2m_ctx);

		if (src_vbuf && dst_vbuf) {
			/* Copy timestamp + flags from src to dst (V4L2 m2m
			 * convention; framework helper handles it).  TODO M4:
			 * set actual frame size in plane payload once decode
			 * produces real output. For skeleton, payload stays 0
			 * (no decoded data). */
			v4l2_m2m_buf_copy_metadata(src_vbuf, dst_vbuf, true);
		}
		if (src_vbuf)
			v4l2_m2m_buf_done(src_vbuf, VB2_BUF_STATE_DONE);
		if (dst_vbuf)
			v4l2_m2m_buf_done(dst_vbuf, VB2_BUF_STATE_DONE);
	}

	h264->slice_count = 0;
	h264->frame_in_progress = false;

	v4l2_m2m_job_finish(sess->m2m_dev, sess->m2m_ctx);
	return IRQ_HANDLED;
}

/* H.264 EOS sequence per Annex B. */
static const u8 eos_sequence[] = {
	0x00, 0x00, 0x01, 0x0b,		/* end of sequence NALU */
};

static const u8 *codec_h264_libre_eos_sequence(u32 *len)
{
	*len = ARRAY_SIZE(eos_sequence);
	return eos_sequence;
}

struct amvdec_codec_ops codec_h264_libre_ops = {
	.start = codec_h264_libre_start,
	.stop = codec_h264_libre_stop,
	.pre_queue = codec_h264_libre_pre_queue,
	.isr = codec_h264_libre_isr,
	.threaded_isr = codec_h264_libre_threaded_isr,
	.eos_sequence = codec_h264_libre_eos_sequence,
};

static int __init codec_h264_libre_mod_init(void)
{
	return amvdec_register_codec(V4L2_PIX_FMT_H264_SLICE,
				     &codec_h264_libre_ops);
}

static void __exit codec_h264_libre_mod_exit(void)
{
	amvdec_unregister_codec(V4L2_PIX_FMT_H264_SLICE,
				&codec_h264_libre_ops);
}

module_init(codec_h264_libre_mod_init);
module_exit(codec_h264_libre_mod_exit);
MODULE_DESCRIPTION("Meson VDEC H.264 libre stateless codec");
MODULE_LICENSE("GPL");
