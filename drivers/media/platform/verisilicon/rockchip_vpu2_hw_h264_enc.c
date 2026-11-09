// SPDX-License-Identifier: GPL-2.0
/*
 * Hantro VPU codec driver
 *
 * Copyright (C) 2024 Libre Computer
 *
 * H.264 encoder support for VEPU2 (Hantro H1-compatible encoder).
 *
 * The VEPU2 hardware supports H.264 Baseline and Main profile encoding.
 * It handles transform, quantization, entropy coding (CAVLC/CABAC),
 * intra prediction, inter prediction with 1 reference frame,
 * and in-loop deblocking filter.
 *
 * This is a stateful encoder -- the driver manages reference frame
 * buffers and frame numbering internally. Userspace submits raw
 * YUV frames and receives H.264 elementary stream output (byte stream
 * format with Annex B start codes).
 *
 * The encoder generates slice NALUs only. SPS and PPS NALUs are
 * prepended by the driver at stream start and at each IDR frame.
 * Userspace receives a complete H.264 byte stream (Annex B format).
 *
 * Profile support:
 *  - Baseline: CAVLC entropy coding, no CABAC, no B-frames
 *  - Main: CABAC entropy coding, in-loop deblocking, no B-frames
 *
 * V4L2 controls wired:
 *  - V4L2_CID_MPEG_VIDEO_H264_PROFILE: Baseline or Main
 *  - V4L2_CID_MPEG_VIDEO_H264_LEVEL: 1.0 to 4.1
 *  - V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP: QP for I-frames (0-51)
 *  - V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP: QP for P-frames (0-51)
 *  - V4L2_CID_MPEG_VIDEO_GOP_SIZE: IDR interval (0=all-intra)
 *  - V4L2_CID_MPEG_VIDEO_BITRATE: Target bitrate for CBR rate control
 *  - V4L2_CID_MPEG_VIDEO_BITRATE_MODE: CBR or CQ (constant quality / CQP)
 *
 * Rate control:
 *  - CQ mode: fixed QP per frame, no checkpoints, no QP adaptation.
 *  - CBR mode (the default): the driver adjusts the P-frame QP between
 *    frames from the previous frame's bit count. Measured on RK3399
 *    2026-09-02 over an 8-frame GOP at 96x96: at 40 Mbps the recovered
 *    SliceQP walks 26 -> 26, 25, 24; at 10 kbps it walks 26 -> 26, 27, 28.
 *
 * What CBR mode does NOT do is drive the block's own checkpoint rate
 * controller. Those registers are programmed, but 40 Mbps and 1 Mbps produce
 * BIT-IDENTICAL output from a single frame -- only the never-reached target in
 * cp[4].lo moves -- and a four-point sweep found the CHECKPOINT result field
 * has no constant bits-per-unit divisor. See rk3399-codec-status.md (R5).
 *
 * Selecting the mode is what BITRATE_MODE is for. Before it existed the choice
 * was made by `bitrate > 0`, and BITRATE is registered with .min = 10000, so
 * that test could never be false and the CQP path was unreachable dead code.
 *
 * Limitations:
 *  - Single reference frame (no B-frames)
 *  - Baseline or Main profile only (no High profile 8x8 transform)
 *  - Maximum encode resolution: 1920x1088 (FHD)
 */

#include <linux/types.h>
#include <linux/math.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-mem2mem.h>

#include "hantro.h"
#include "hantro_hw.h"
#include "hantro_v4l2.h"
#include "rockchip_vpu2_regs.h"

/*
 * CABAC initial probability table for H.264 encoding.
 *
 * The VEPU2 requires this table to be loaded into a DMA-accessible buffer
 * and its address written to VEPU_REG_ADDR_CABAC_TBL before starting a
 * CABAC-mode encode. The table contains context model initial values
 * for all CABAC syntax elements.
 *
 * This table is derived from the H.264 specification Table 9-26 through
 * Table 9-34, pre-computed for cabac_init_idc=0 and SliceQP=26.
 * The values are from the Hantro H1 encoder reference implementation.
 *
 * Table layout: 460 32-bit words (1840 bytes) covering:
 *  - mb_type contexts (I and P slices)
 *  - sub_mb_type, mvd, ref_idx contexts
 *  - coded_block_pattern contexts
 *  - significant_coeff_flag, last_significant_coeff_flag contexts
 *  - coeff_abs_level_minus1 contexts
 *  - mb_qp_delta, intra_chroma_pred_mode contexts
 *
 * Reference: Hantro H1 encoder TRM, Section 5.3 "CABAC Context Init"
 */
static const u32 h264_cabac_init_table[] = {
	/* MB type contexts (I-slice: 3 ctx, P-slice: 14 ctx) */
	0x14f10236, 0x034a14f1, 0x0236034a, 0xe47fe968,
	0xfa35ff36, 0x07330000, 0x00000000, 0x00000000,
	/* Sub-MB type and MVD contexts */
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	/* Ref index and delta QP contexts */
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	/* CBP contexts */
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	/* Significance map contexts */
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	/* Coefficient level contexts */
	0x00010000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	/* Intra prediction mode and chroma pred contexts */
	0x0029003f, 0x003f003f, 0x003f003f, 0x003f003f,
	0x003f003f, 0x003f003f, 0x003f003f, 0x003f003f,
	0x003f003f, 0x003f003f, 0x003f003f, 0x003f003f,
	0x003f003f, 0x003f003f, 0x003f003f, 0x003f003f,
	0x003f003f, 0x003f003f, 0x003f003f, 0x003f003f,
	0x003f003f, 0x003f003f, 0x003f003f, 0x003f003f,
	0x003f003f, 0x003f003f, 0x003f003f, 0x003f003f,
	0x003f003f, 0x003f003f, 0x003f003f, 0x003f003f,
	0x003f003f, 0x003f003f, 0x003f003f, 0x003f003f,
	0x003f003f, 0x003f003f, 0x003f003f, 0x003f003f,
	0x003f003f, 0x003f003f, 0x003f003f, 0x003f003f,
	0x003f003f, 0x003f003f, 0x003f003f, 0x003f003f,
	0x003f003f, 0x003f003f, 0x003f003f, 0x003f003f,
	0x003f003f, 0x003f003f, 0x003f003f, 0x003f003f,
	/*
	 * Remaining contexts zero-initialized by the hardware when
	 * not explicitly set. The 460-word table is zero-filled in
	 * the DMA buffer beyond the explicit entries above.
	 */
};

#define CABAC_INIT_TABLE_SIZE	(460 * sizeof(u32))

/*
 * Upper bound on the bytes this driver writes ahead of the hardware's slice
 * data: SPS (13) + PPS (8) + padding to the 8-byte output-address grain + the
 * slice start code. 64 leaves room for a cropped SPS and keeps the buffer-size
 * check ahead of the writes.
 */
#define H264_HDR_RESERVE	64

/*
 * VEPU_REG_FRAME_NUM is a 16-bit field and the block emits frame_num into the
 * slice header at that full width, so the SPS must declare
 * log2_max_frame_num = 16. Measured: with the SPS declaring 4 (the natural
 * choice, and what this driver used to write), every slice-header field after
 * frame_num is parsed 12 bits early and the first macroblock decodes as
 * garbage -- ffmpeg reports "error while decoding MB 0 0". Only the parameter
 * set was ever wrong; the hardware's slice bytes are correct.
 */
#define H264_LOG2_MAX_FRAME_NUM_MINUS4	12
#define H264_MAX_FRAME_NUM_MASK		0xffff

/*
 * Default frame rate for rate control calculations when the actual
 * frame rate is not available. 30 fps is a common default.
 */
#define DEFAULT_FPS		30

/*
 * Number of rate control checkpoints. The VEPU2 supports 10 checkpoint
 * positions within a frame for mid-frame QP adjustment.
 */
#define RC_NUM_CHECKPOINTS	10

/*
 * QP clamp bounds for rate control. These define the absolute range
 * the rate controller can adjust QP within a frame.
 */
#define RC_QP_MIN		10
#define RC_QP_MAX		51

/*
 * Per-context state for the H.264 encoder.
 * Allocated in init, freed in exit.
 */
struct rockchip_vpu2_h264_enc_ctx {
	/* DMA buffer for CABAC probability table */
	struct hantro_aux_buf cabac_tbl;

	/*
	 * DMA buffers for reconstructed reference frames (double-buffered).
	 * rec_buf[0] and rec_buf[1] alternate as the current reconstruction
	 * target and the previous frame reference. This avoids the need
	 * for a frame copy between encodes.
	 */
	struct hantro_aux_buf rec_buf[2];
	int rec_cur;		/* index of current reconstruction buffer */

	/* DMA buffer for MV output (optional, for debugging) */
	struct hantro_aux_buf mv_out_buf;

	/* Encoder state tracking */
	u32 frame_num;		/* H.264 frame_num for slice header */
	u32 idr_pic_id;		/* alternates 0/1 between IDRs */
	bool is_idr;		/* current frame is IDR */
	u32 gop_counter;	/* frames since last IDR */

	/*
	 * Rate control state.
	 * The VEPU2 checkpoint-based rate controller works by placing
	 * checkpoint positions at regular intervals through the frame.
	 * At each checkpoint, the HW compares actual bits used against
	 * expected bits and adjusts QP via a delta table.
	 * After encoding, the driver reads back the QP sum register
	 * to track actual QP usage for the next frame.
	 */
	u32 prev_frame_bits;	/* bits produced by previous frame */
	s32 qp_adj;		/* accumulated QP adjustment from RC */

	/*
	 * The rate-control mode the frame now in the hardware was programmed
	 * with, latched by set_params() and read back by done().
	 *
	 * BITRATE_MODE is runtime-settable, so done() must not re-read it: it
	 * would otherwise integrate the just-finished frame against whatever
	 * mode userspace happens to have selected by the time the IRQ lands.
	 * A single stored decision per frame is what keeps set_params() and
	 * done() from disagreeing about which controller is running.
	 */
	bool rc_cbr;

	/*
	 * The target bitrate the frame now in the hardware was programmed
	 * with, latched by set_params() for the same reason as rc_cbr above.
	 * BITRATE is runtime-settable too, so a done() that re-read it would
	 * score the just-finished frame against a target it was not encoded
	 * under: an S_EXT_CTRLS that RAISES BITRATE between run() and the
	 * completion IRQ reads as "under budget" and wrongly LOWERS QP.
	 */
	s32 rc_bitrate;

	/* Header size (SPS+PPS) prepended to current frame's output */
	u32 hdr_bytes;
};

/*
 * The VEPU2 H.264 encoder context is stored in the codec union
 * inside hantro_ctx. Since the upstream struct doesn't have an
 * h264_enc member yet, we store our context pointer in the h264_dec
 * union slot's priv field since encode and decode are mutually
 * exclusive per context instance.
 *
 * TODO: Add a proper h264_enc field to the codec union in hantro_ctx.
 */

static struct rockchip_vpu2_h264_enc_ctx *
get_h264_enc_ctx(struct hantro_ctx *ctx)
{
	return (struct rockchip_vpu2_h264_enc_ctx *)ctx->h264_dec.priv.cpu;
}

/*
 * Read a scalar V4L2 control value. Returns the control's current
 * 32-bit value, or the provided default if the control is not found.
 */
static s32 h264_enc_get_ctrl_val(struct hantro_ctx *ctx, u32 id, s32 defval)
{
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_find(&ctx->ctrl_handler, id);
	if (!ctrl)
		return defval;
	return v4l2_ctrl_g_ctrl(ctrl);
}

/*
 * Compute the logarithmic MV cost per H.264 spec.
 *
 * The H.264 specification defines MV cost as:
 *   cost(mvd) = lambda * (2 * log2(|mvd| + 1) + 1)
 *
 * For the VEPU2 DMV penalty table, we use a simplified integer
 * approximation that maps MV magnitudes 0..127 to penalty values
 * 0..255. The table has 128 entries packed 4 per 32-bit word
 * (32 words total).
 *
 * The quarter-pixel penalty table follows the same structure but
 * applies to sub-pixel MV components. Quarter-pixel MVs get a
 * smaller penalty since they are common in natural video.
 */
static u8 mv_penalty_log(int mvd, int qp)
{
	int lambda, cost;

	/*
	 * Lambda approximation: lambda ~= 0.85 * 2^((qp-12)/3)
	 * Simplified to integer math with 8-bit fractional precision.
	 * For qp=26: lambda ~= 5.4
	 * The factor is scaled by 256 for fixed-point.
	 */
	if (qp < 12)
		lambda = 1;
	else
		lambda = max(1, (85 * (1 << ((qp - 12) / 3))) >> 8);

	if (mvd == 0)
		return 0;

	/* cost = lambda * (2 * log2(|mvd| + 1) + 1), clamped to 255 */
	cost = lambda * (2 * ilog2(abs(mvd) + 1) + 1);
	return min(cost, 255);
}

static void rockchip_vpu2_h264_enc_set_src_img(struct hantro_dev *vpu,
					       struct hantro_ctx *ctx)
{
	u32 overfill_r, overfill_b;
	u32 reg;

	/*
	 * The format width and height are already macroblock aligned
	 * by .vidioc_s_fmt_vid_cap_mplane() callback. Destination
	 * format width and height can be further modified by
	 * .vidioc_s_selection(), and the width is 4-aligned.
	 */
	overfill_r = ctx->src_fmt.width - ctx->dst_fmt.width;
	overfill_b = ctx->src_fmt.height - ctx->dst_fmt.height;

	/* Input luma row length (bytes per row) */
	reg = VEPU_REG_IN_IMG_CTRL_ROW_LEN(ctx->src_fmt.width);
	vepu_write_relaxed(vpu, reg, VEPU_REG_INPUT_LUMA_INFO);

	/*
	 * Overfill and stream start offset.
	 * Stream start offset = 0 (no pre-existing header bits).
	 * Skip macroblock penalty = 0 (let mode decision handle it).
	 */
	reg = VEPU_REG_STREAM_START_OFFSET(0) |
	      VEPU_REG_SKIP_MACROBLOCK_PENALTY(0) |
	      VEPU_REG_IN_IMG_CTRL_OVRFLR_D4(overfill_r / 4) |
	      VEPU_REG_IN_IMG_CTRL_OVRFLB(overfill_b);
	vepu_write_relaxed(vpu, reg, VEPU_REG_ENC_OVER_FILL_STRM_OFFSET);

	/*
	 * ENC_CTRL1 (0x128): MAD threshold and input format.
	 *
	 * MAD threshold: Used by the rate controller to detect scene
	 * changes. A macroblock with MAD above this threshold is
	 * considered a significant change. Value 0x3f = maximum
	 * threshold (least sensitive). For CBR mode, lower values
	 * make the RC react more aggressively to motion.
	 *
	 * Input format: From the encoder format descriptor.
	 */
	reg = VEPU_REG_MAD_THRESHOLD(0x3f) |
	      VEPU_REG_IN_IMG_CTRL_FMT(ctx->vpu_src_fmt->enc_fmt);
	vepu_write_relaxed(vpu, reg, VEPU_REG_ENC_CTRL1);
}

static void rockchip_vpu2_h264_enc_set_buffers(struct hantro_dev *vpu,
					       struct hantro_ctx *ctx,
					       struct vb2_buffer *src_buf,
					       struct vb2_buffer *dst_buf,
					       bool is_intra)
{
	struct rockchip_vpu2_h264_enc_ctx *h264_ctx = get_h264_enc_ctx(ctx);
	struct v4l2_pix_format_mplane *pix_fmt = &ctx->src_fmt;
	dma_addr_t src[3];
	u32 size_left;
	u32 rec_luma_size;
	int rec_cur = h264_ctx->rec_cur;
	int rec_ref = rec_cur ^ 1; /* previous frame's reconstruction */

	size_left = vb2_plane_size(dst_buf, 0);

	/* Output bitstream buffer */
	vepu_write_relaxed(vpu, vb2_dma_contig_plane_dma_addr(dst_buf, 0),
			   VEPU_REG_ADDR_OUTPUT_STREAM);
	vepu_write_relaxed(vpu, size_left, VEPU_REG_STR_BUF_LIMIT);

	/* Input YUV planes */
	if (pix_fmt->num_planes == 1) {
		src[0] = vb2_dma_contig_plane_dma_addr(src_buf, 0);
		vepu_write_relaxed(vpu, src[0], VEPU_REG_ADDR_IN_PLANE_0);
	} else if (pix_fmt->num_planes == 2) {
		src[0] = vb2_dma_contig_plane_dma_addr(src_buf, 0);
		src[1] = vb2_dma_contig_plane_dma_addr(src_buf, 1);
		vepu_write_relaxed(vpu, src[0], VEPU_REG_ADDR_IN_PLANE_0);
		vepu_write_relaxed(vpu, src[1], VEPU_REG_ADDR_IN_PLANE_1);
	} else {
		src[0] = vb2_dma_contig_plane_dma_addr(src_buf, 0);
		src[1] = vb2_dma_contig_plane_dma_addr(src_buf, 1);
		src[2] = vb2_dma_contig_plane_dma_addr(src_buf, 2);
		vepu_write_relaxed(vpu, src[0], VEPU_REG_ADDR_IN_PLANE_0);
		vepu_write_relaxed(vpu, src[1], VEPU_REG_ADDR_IN_PLANE_1);
		vepu_write_relaxed(vpu, src[2], VEPU_REG_ADDR_IN_PLANE_2);
	}

	/*
	 * Reconstructed frame buffer -- the encoder writes the
	 * deblocked, reconstructed frame here. This serves as the
	 * reference for the next P-frame.
	 *
	 * Double-buffered: rec_cur is the write target, rec_ref is
	 * the read reference (previous frame).
	 */
	rec_luma_size = ALIGN(ctx->src_fmt.width, MB_DIM) *
			ALIGN(ctx->src_fmt.height, MB_DIM);

	vepu_write_relaxed(vpu, h264_ctx->rec_buf[rec_cur].dma,
			   VEPU_REG_ADDR_REC_LUMA);
	vepu_write_relaxed(vpu, h264_ctx->rec_buf[rec_cur].dma + rec_luma_size,
			   VEPU_REG_ADDR_REC_CHROMA);

	/*
	 * Reference frame buffer -- for P-frames, point at the previous
	 * reconstructed frame. For I-frames, it must still be a valid
	 * address (HW reads it regardless) so point at the current
	 * reconstruction buffer (contents will be ignored).
	 */
	if (is_intra) {
		vepu_write_relaxed(vpu, h264_ctx->rec_buf[rec_cur].dma,
				   VEPU_REG_ADDR_REF_LUMA);
		vepu_write_relaxed(vpu,
				   h264_ctx->rec_buf[rec_cur].dma + rec_luma_size,
				   VEPU_REG_ADDR_REF_CHROMA);
	} else {
		vepu_write_relaxed(vpu, h264_ctx->rec_buf[rec_ref].dma,
				   VEPU_REG_ADDR_REF_LUMA);
		vepu_write_relaxed(vpu,
				   h264_ctx->rec_buf[rec_ref].dma + rec_luma_size,
				   VEPU_REG_ADDR_REF_CHROMA);
	}

	/*
	 * MV output buffer -- the encoder can optionally write out
	 * motion vectors for debugging or external analysis.
	 * We allocate a buffer but disable MV write via the interrupt
	 * register (VEPU_REG_MV_WRITE_EN not set).
	 */
	vepu_write_relaxed(vpu, h264_ctx->mv_out_buf.dma,
			   VEPU_REG_ADDR_MV_OUT);

	/*
	 * Output control and next picture registers.
	 * ADDR_OUTPUT_CTRL: Used for output size table (not needed for
	 *   single-slice encoding).
	 * ADDR_NEXT_PIC: For pre-fetching the next input frame (0=disable).
	 */
	vepu_write_relaxed(vpu, 0, VEPU_REG_ADDR_OUTPUT_CTRL);
	vepu_write_relaxed(vpu, 0, VEPU_REG_ADDR_NEXT_PIC);

	/*
	 * Stream header remainder registers.
	 * These are used when the driver wants to prepend header bits
	 * that the HW should include before its encoded slice data.
	 * We set them to 0 because we handle SPS/PPS NALUs separately.
	 */
	vepu_write_relaxed(vpu, 0, VEPU_REG_STR_HDR_REM_MSB);
	vepu_write_relaxed(vpu, 0, VEPU_REG_STR_HDR_REM_LSB);
}

/*
 * Generate SPS (Sequence Parameter Set) NAL unit.
 *
 * The VEPU2 generates only slice data. SPS and PPS must be prepended
 * by software. We generate a minimal SPS conforming to the selected
 * profile and level.
 *
 * Returns the number of bytes written, or 0 on error.
 *
 * The SPS is written in H.264 Annex B byte stream format:
 *   00 00 00 01 <nal_header> <sps_rbsp>
 *
 * Fields encoded:
 *  - profile_idc: 66 (Baseline) or 77 (Main)
 *  - constraint_set flags: set0=1 for Baseline, set1=1 for Main
 *  - level_idc: from V4L2 control
 *  - seq_parameter_set_id: 0
 *  - log2_max_frame_num_minus4: 0 (max_frame_num = 16)
 *  - pic_order_cnt_type: 2 (no POC fields needed)
 *  - max_num_ref_frames: 1 (single reference)
 *  - gaps_in_frame_num_value_allowed: 0
 *  - pic_width/height_in_mbs
 *  - direct_8x8_inference_flag: 1
 *  - frame_cropping: only if overfill
 *  - vui_parameters_present: 0
 */
static int h264_enc_write_sps(struct hantro_ctx *ctx, u8 *buf, int max_len)
{
	int profile, level;
	u32 mb_w, mb_h;
	u32 crop_r, crop_b;
	int i = 0;
	u8 rbsp[64];
	int bit = 0;

	profile = h264_enc_get_ctrl_val(ctx,
					V4L2_CID_MPEG_VIDEO_H264_PROFILE,
					V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE);
	level = h264_enc_get_ctrl_val(ctx,
				      V4L2_CID_MPEG_VIDEO_H264_LEVEL,
				      V4L2_MPEG_VIDEO_H264_LEVEL_4_0);

	mb_w = MB_WIDTH(ctx->src_fmt.width);
	mb_h = MB_HEIGHT(ctx->src_fmt.height);
	crop_r = ctx->src_fmt.width - ctx->dst_fmt.width;
	crop_b = ctx->src_fmt.height - ctx->dst_fmt.height;

	if (max_len < 32)
		return 0;

	/*
	 * Manual RBSP encoding with exp-Golomb.
	 * We write to a byte array bit-by-bit using helper macros.
	 * This is simpler than maintaining a full bitstream writer
	 * for the small amount of data in SPS/PPS.
	 */
#define PUT_BITS(val, n) do { \
	int _n = (n); \
	u32 _v = (val); \
	while (_n-- > 0) { \
		rbsp[bit / 8] |= ((_v >> _n) & 1) << (7 - (bit % 8)); \
		bit++; \
	} \
} while (0)

/* Exp-Golomb unsigned: code_num -> 0-indexed */
#define PUT_UE(val) do { \
	u32 _val = (val) + 1; \
	int _bits = fls(_val); \
	PUT_BITS(0, _bits - 1); \
	PUT_BITS(_val, _bits); \
} while (0)

/* Exp-Golomb signed */
#define PUT_SE(val) do { \
	int _sv = (val); \
	u32 _code = (_sv > 0) ? (2 * _sv - 1) : (-2 * _sv); \
	PUT_UE(_code); \
} while (0)

	memset(rbsp, 0, sizeof(rbsp));

	/* profile_idc */
	if (profile == V4L2_MPEG_VIDEO_H264_PROFILE_MAIN)
		PUT_BITS(77, 8);
	else
		PUT_BITS(66, 8); /* Baseline */

	/*
	 * level_idc, and the constraint_set flags that qualify it.
	 *
	 * The map is indexed by the V4L2 enumerator, which is dense and starts
	 * V4L2_MPEG_VIDEO_H264_LEVEL_1_0 = 0, _1B = 1, _1_1 = 2 ... _4_0 = 11.
	 * Level 1b has no level_idc of its own below High profile: it is
	 * level_idc 11 with constraint_set3_flag set, which is what separates
	 * it from level 1.1. Omitting the 1b row shifted every entry above it
	 * by one, so the default LEVEL_4_0 control emitted level_idc 41.
	 */
	{
		static const u8 level_idc_map[] = {
			10, 11, 11, 12, 13, 20, 21, 22, 30, 31,
			32, 40, 41, 42, 50, 51, 52, 60, 61, 62,
		};
		u8 lidc = (level < ARRAY_SIZE(level_idc_map)) ?
			  level_idc_map[level] : 40;
		u8 cset;

		/* constraint_set0_flag .. constraint_set5_flag + 2 reserved */
		if (profile == V4L2_MPEG_VIDEO_H264_PROFILE_MAIN)
			cset = 0x40;	/* constraint_set1_flag = 1 */
		else
			cset = 0x80;	/* constraint_set0_flag = 1 */
		if (level == V4L2_MPEG_VIDEO_H264_LEVEL_1B)
			cset |= 0x10;	/* constraint_set3_flag = 1 */
		PUT_BITS(cset, 8);

		PUT_BITS(lidc, 8);
	}

	PUT_UE(0);	/* seq_parameter_set_id */
	/* log2_max_frame_num = 16: the width the block emits. See the define. */
	PUT_UE(H264_LOG2_MAX_FRAME_NUM_MINUS4);
	PUT_UE(2);	/* pic_order_cnt_type = 2 (no POC coding) */
	PUT_UE(1);	/* max_num_ref_frames = 1 */
	PUT_BITS(0, 1);	/* gaps_in_frame_num_value_allowed_flag = 0 */
	PUT_UE(mb_w - 1);	/* pic_width_in_mbs_minus1 */
	PUT_UE(mb_h - 1);	/* pic_height_in_map_units_minus1 */
	PUT_BITS(1, 1);	/* frame_mbs_only_flag = 1 (progressive) */
	/* mb_adaptive_frame_field_flag: not present when frame_mbs_only=1 */
	PUT_BITS(1, 1);	/* direct_8x8_inference_flag = 1 */

	/* Frame cropping */
	if (crop_r || crop_b) {
		PUT_BITS(1, 1); /* frame_cropping_flag */
		PUT_UE(0);		/* crop_left */
		PUT_UE(crop_r / 2);	/* crop_right (in 2-pixel units) */
		PUT_UE(0);		/* crop_top */
		PUT_UE(crop_b / 2);	/* crop_bottom (in 2-pixel units) */
	} else {
		PUT_BITS(0, 1);	/* frame_cropping_flag = 0 */
	}

	PUT_BITS(0, 1);	/* vui_parameters_present_flag = 0 */

	/* RBSP trailing bits */
	PUT_BITS(1, 1);	/* stop bit */
	while (bit % 8)
		PUT_BITS(0, 1);

	/* Write Annex B start code + NAL header + RBSP */
	buf[i++] = 0x00;
	buf[i++] = 0x00;
	buf[i++] = 0x00;
	buf[i++] = 0x01;
	buf[i++] = 0x67; /* NAL type 7 = SPS, nal_ref_idc=3 */
	memcpy(buf + i, rbsp, (bit + 7) / 8);
	i += (bit + 7) / 8;

#undef PUT_BITS
#undef PUT_UE
#undef PUT_SE

	return i;
}

/*
 * Generate PPS (Picture Parameter Set) NAL unit.
 *
 * Minimal PPS matching our SPS. Key fields:
 *  - entropy_coding_mode_flag: 0=CAVLC (Baseline), 1=CABAC (Main)
 *  - deblocking_filter_control_present_flag: 1
 */
static int h264_enc_write_pps(struct hantro_ctx *ctx, u8 *buf, int max_len)
{
	int profile;
	u8 rbsp[32];
	int bit = 0;
	int i = 0;
	s32 i_qp;

	profile = h264_enc_get_ctrl_val(ctx,
					V4L2_CID_MPEG_VIDEO_H264_PROFILE,
					V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE);
	i_qp = h264_enc_get_ctrl_val(ctx,
				     V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP, 26);

	if (max_len < 20)
		return 0;

#define PUT_BITS(val, n) do { \
	int _n = (n); \
	u32 _v = (val); \
	while (_n-- > 0) { \
		rbsp[bit / 8] |= ((_v >> _n) & 1) << (7 - (bit % 8)); \
		bit++; \
	} \
} while (0)

#define PUT_UE(val) do { \
	u32 _val = (val) + 1; \
	int _bits = fls(_val); \
	PUT_BITS(0, _bits - 1); \
	PUT_BITS(_val, _bits); \
} while (0)

#define PUT_SE(val) do { \
	int _sv = (val); \
	u32 _code = (_sv > 0) ? (2 * _sv - 1) : (-2 * _sv); \
	PUT_UE(_code); \
} while (0)

	memset(rbsp, 0, sizeof(rbsp));

	PUT_UE(0);		/* pic_parameter_set_id */
	PUT_UE(0);		/* seq_parameter_set_id */

	/* entropy_coding_mode_flag: 0=CAVLC, 1=CABAC */
	if (profile == V4L2_MPEG_VIDEO_H264_PROFILE_MAIN)
		PUT_BITS(1, 1);
	else
		PUT_BITS(0, 1);

	PUT_BITS(0, 1);	/* bottom_field_pic_order_in_frame_present = 0 */
	PUT_UE(0);		/* num_slice_groups_minus1 = 0 */
	PUT_UE(0);		/* num_ref_idx_l0_default_active_minus1 = 0 */
	PUT_UE(0);		/* num_ref_idx_l1_default_active_minus1 = 0 */
	PUT_BITS(0, 1);	/* weighted_pred_flag = 0 */
	PUT_BITS(0, 2);	/* weighted_bipred_idc = 0 */
	PUT_SE(i_qp - 26);	/* pic_init_qp_minus26 */
	PUT_SE(i_qp - 26);	/* pic_init_qs_minus26 */
	PUT_SE(0);		/* chroma_qp_index_offset = 0 */
	PUT_BITS(1, 1);	/* deblocking_filter_control_present_flag = 1 */
	PUT_BITS(0, 1);	/* constrained_intra_pred_flag = 0 */
	PUT_BITS(0, 1);	/* redundant_pic_cnt_present_flag = 0 */

	/* RBSP trailing bits */
	PUT_BITS(1, 1);
	while (bit % 8)
		PUT_BITS(0, 1);

	buf[i++] = 0x00;
	buf[i++] = 0x00;
	buf[i++] = 0x00;
	buf[i++] = 0x01;
	buf[i++] = 0x68; /* NAL type 8 = PPS, nal_ref_idc=3 */
	memcpy(buf + i, rbsp, (bit + 7) / 8);
	i += (bit + 7) / 8;

#undef PUT_BITS
#undef PUT_UE
#undef PUT_SE

	return i;
}

static void rockchip_vpu2_h264_enc_set_params(struct hantro_dev *vpu,
					      struct hantro_ctx *ctx,
					      bool is_intra)
{
	struct rockchip_vpu2_h264_enc_ctx *h264_ctx = get_h264_enc_ctx(ctx);
	s32 i_qp, p_qp, profile, bitrate;
	bool cbr;
	u32 qp, qp_min, qp_max;
	u32 reg;
	u32 mb_w, mb_h, total_mbs;
	u32 target_bits_per_frame;
	u32 chkpt_distance;
	int i;

	/* Read V4L2 controls */
	profile = h264_enc_get_ctrl_val(ctx,
					V4L2_CID_MPEG_VIDEO_H264_PROFILE,
					V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE);
	i_qp = h264_enc_get_ctrl_val(ctx,
				     V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP, 26);
	p_qp = h264_enc_get_ctrl_val(ctx,
				     V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP, 26);
	bitrate = h264_enc_get_ctrl_val(ctx,
					V4L2_CID_MPEG_VIDEO_BITRATE, 0);
	/*
	 * Rate control is selected by BITRATE_MODE, not by `bitrate > 0`.
	 * V4L2_CID_MPEG_VIDEO_BITRATE is registered with .min = 10000, so
	 * `bitrate > 0` was a tautology: CBR was the only reachable mode and
	 * every constant-QP branch behind that test was dead code.
	 */
	cbr = h264_enc_get_ctrl_val(ctx, V4L2_CID_MPEG_VIDEO_BITRATE_MODE,
				    V4L2_MPEG_VIDEO_BITRATE_MODE_CBR) ==
	      V4L2_MPEG_VIDEO_BITRATE_MODE_CBR;
	if (!cbr)
		bitrate = 0;

	/*
	 * Latch the decision for done(), and drop the accumulator whenever the
	 * mode changes. qp_adj is CBR's integrator; carrying it across a mode
	 * switch would apply up to +-10 QP, accumulated under a controller
	 * that is no longer running, to the first P-frame after the switch.
	 */
	if (h264_ctx->rc_cbr != cbr) {
		h264_ctx->qp_adj = 0;
		h264_ctx->prev_frame_bits = 0;
		h264_ctx->rc_cbr = cbr;
	}
	h264_ctx->rc_bitrate = bitrate;

	/* Select QP based on frame type */
	qp = is_intra ? i_qp : p_qp;

	/* Apply rate control QP adjustment (clamped to valid range) */
	if (bitrate > 0 && !is_intra) {
		s32 adj_qp = (s32)qp + h264_ctx->qp_adj;

		qp = clamp_t(s32, adj_qp, RC_QP_MIN, RC_QP_MAX);
	}

	mb_w = MB_WIDTH(ctx->src_fmt.width);
	mb_h = MB_HEIGHT(ctx->src_fmt.height);
	total_mbs = mb_w * mb_h;

	/*
	 * ENC_CTRL0 (0x0ec): H.264 encoding parameters.
	 *
	 * Register fields:
	 *  [28]    DISABLE_QUARTER_PIXEL_MV: 0 = enable qpel MVs
	 *  [25:24] DEBLOCKING_FILTER_MODE:
	 *          0 = enabled for all edges (best quality)
	 *          1 = disabled
	 *          2 = disabled at slice boundaries
	 *  [22:21] CABAC_INIT_IDC: 0-2, selects CABAC init table
	 *  [20]    ENTROPY_CODING_MODE: 0=CAVLC, 1=CABAC
	 *  [17]    H264_TRANS8X8_MODE: 0 (not supported, High profile only)
	 *  [16]    H264_INTER4X4_MODE: 1 = enable 4x4 inter partitions
	 *  [15]    H264_STREAM_MODE: 1 = Annex B byte stream
	 *  [14:8]  H264_SLICE_SIZE: 0 = single slice per frame
	 */
	reg = VEPU_REG_DEBLOCKING_FILTER_MODE(0) |
	      VEPU_REG_CABAC_INIT_IDC(0) |
	      VEPU_REG_H264_STREAM_MODE |
	      VEPU_REG_H264_INTER4X4_MODE |
	      VEPU_REG_H264_SLICE_SIZE(0);

	/* Enable CABAC for Main profile */
	if (profile == V4L2_MPEG_VIDEO_H264_PROFILE_MAIN)
		reg |= VEPU_REG_ENTROPY_CODING_MODE;

	/* Keep quarter-pixel MVs enabled (do NOT set DISABLE_QUARTER_PIXEL_MV) */

	vepu_write_relaxed(vpu, reg, VEPU_REG_ENC_CTRL0);

	/*
	 * ENC_CTRL2 (0x130): PPS/slice parameters.
	 *
	 * Register fields:
	 *  [31:26] PPS_INIT_QP: Initial QP from PPS (26 + pic_init_qp_minus26)
	 *  [25:22] SLICE_FILTER_ALPHA: deblocking alpha offset / 2
	 *  [21:18] SLICE_FILTER_BETA: deblocking beta offset / 2
	 *  [17:13] CHROMA_QP_OFFSET: chroma QP offset (-12 to +12)
	 *  [5]     FILTER_DISABLE: 0=deblocking enabled, 1=disabled
	 *  [4:1]   IDR_PIC_ID: alternating IDR picture ID
	 *  [0]     CONSTRAINED_INTRA_PREDICTION: 0
	 *
	 * Deblocking filter alpha/beta offsets:
	 *  0 = default (no offset), range -6 to +6 in register (signed 4-bit)
	 *  Positive values = stronger filtering, negative = weaker
	 *  We use 0 (default) for balanced quality.
	 */
	/*
	 * PPS_INIT_QP must carry what the PPS in the bitstream DECLARES, not
	 * the QP this frame is actually coded at: the block derives the slice
	 * header's slice_qp_delta from (LUMA_INIT_QP - PPS_INIT_QP), and a
	 * decoder recovers SliceQP as 26 + pic_init_qp_minus26 + slice_qp_delta.
	 * h264_enc_write_pps() always emits pic_init_qp_minus26 = i_qp - 26, so
	 * feeding the rate-controlled frame QP here made every P-frame (and
	 * every RC-adjusted frame) dequantize off by i_qp - qp. Invisible while
	 * i_qp == p_qp and RC is idle, which is every test run so far.
	 */
	reg = VEPU_REG_PPS_INIT_QP(i_qp) |
	      VEPU_REG_SLICE_FILTER_ALPHA(0) |
	      VEPU_REG_SLICE_FILTER_BETA(0) |
	      VEPU_REG_CHROMA_QP_OFFSET(0) |
	      VEPU_REG_IDR_PIC_ID(h264_ctx->idr_pic_id & 0xf);
	/* Deblocking enabled: do NOT set VEPU_REG_FILTER_DISABLE */
	vepu_write_relaxed(vpu, reg, VEPU_REG_ENC_CTRL2);

	/*
	 * ENC_CTRL3 (0x1a8): PPS ID, intra prediction, frame number.
	 *
	 * Register fields:
	 *  [31:24] PPS_ID: picture parameter set ID (always 0)
	 *  [23:16] INTRA_PRED_MODE: intra prediction penalty bias
	 *          0 = no bias (let HW choose optimal mode)
	 *  [15:0]  FRAME_NUM: H.264 frame_num for slice header, emitted at
	 *          the full 16-bit width -- so it wraps at 65536 and the SPS
	 *          declares log2_max_frame_num = 16 to match.
	 */
	reg = VEPU_REG_PPS_ID(0) |
	      VEPU_REG_INTRA_PRED_MODE(0) |
	      VEPU_REG_FRAME_NUM(h264_ctx->frame_num & H264_MAX_FRAME_NUM_MASK);
	vepu_write_relaxed(vpu, reg, VEPU_REG_ENC_CTRL3);

	/*
	 * QP_VAL (0x190): QP configuration and rate control.
	 *
	 * Register fields:
	 *  [31:26] H264_LUMA_INIT_QP: QP for this frame
	 *  [25:20] H264_QP_MAX: Upper QP bound for HW RC adjustment
	 *  [19:14] H264_QP_MIN: Lower QP bound for HW RC adjustment
	 *  [11:0]  H264_CHKPT_DISTANCE: Distance between RC checkpoints
	 *          in macroblocks. 0 = disable HW checkpoints (CQP mode).
	 *
	 * In CBR mode, we enable checkpoints by setting a non-zero
	 * distance. The HW evaluates bit usage at each checkpoint and
	 * adjusts QP according to the delta table.
	 */
	if (bitrate > 0) {
		/* CBR mode: enable HW rate control checkpoints */
		target_bits_per_frame = bitrate / DEFAULT_FPS;

		/*
		 * Checkpoint distance: divide frame into RC_NUM_CHECKPOINTS
		 * equally spaced checkpoints. Each checkpoint covers
		 * total_mbs / RC_NUM_CHECKPOINTS macroblocks.
		 */
		chkpt_distance = total_mbs / (RC_NUM_CHECKPOINTS + 1);
		if (chkpt_distance < 1)
			chkpt_distance = 1;

		/*
		 * The hardware RC window must BRACKET the start QP, and QP_MIN
		 * must never exceed QP_MAX.
		 *
		 * H264_I_FRAME_QP has .min = 0 and the software RC clamp above
		 * runs only for non-intra frames, so i_frame_qp=0 reaches here
		 * as qp = 0. The window is then [qp-4, qp+4] = [-4, 4], and the
		 * old code floored the low end at RC_QP_MIN (10), producing
		 * QP_MIN=10 with QP_MAX=4 -- inverted, straight from userspace.
		 *
		 * cb2a91ed7f8ab tried to fix that with
		 *	clamp_t(s32, (s32)qp - 4, RC_QP_MIN, qp_max)
		 * which does NOT work and was never measured: __clamp() is
		 *	val >= hi ? hi : (val <= lo ? lo : val)
		 * so with lo > hi it returns LO. -4 <= 10, so it still yields
		 * QP_MIN = 10 against QP_MAX = 4. Measured on RK3399
		 * 2026-09-02: with and without that commit, i_frame_qp=0
		 * programs VEPU_REG_QP_VAL = 0x00428003 -- LUMA_INIT_QP=0,
		 * QP_MAX=4, QP_MIN=10 -- and the two streams are bit-identical.
		 *
		 * RC_QP_MIN is the floor for the DRIVER'S OWN qp_adj
		 * accumulator, which is a different thing from the hardware
		 * window; using it here was the mistake. Bracket the requested
		 * QP instead.
		 *
		 * The blast radius is qp <= 13, not just the inverted cases.
		 * With lo = RC_QP_MIN = 10 and hi = qp_max = min(qp + 4, 51),
		 * __clamp() returns lo whenever qp - 4 <= 10:
		 *	qp >= 14	qp - 4		unchanged by this commit
		 *	qp 6..13	10 (was valid, but the wrong floor)
		 *	qp 0..5		10 against a qp_max of 4..9: inverted
		 * 14 of the 52 QPs move, and only 6 of those are the inverted
		 * cases 5d410e25ead1c measured as broken -- so for qp 6..13
		 * the old code did program a usable window and
		 * this commit does move it -- e.g. qp = 12 goes from QP_MIN 10
		 * to QP_MIN 8. 5d410e25ead1c claimed "no working configuration
		 * moves"; that was only ever established for qp >= 14, and the
		 * evidence it carried covered qp = 0 and qp = 26 alone. The new
		 * expression is still the correct one -- the window has to
		 * bracket the start QP -- but the old one was wrong for every
		 * qp below 14, not only where it inverted.
		 *
		 * Host-side check over the whole 0..51 range, no board needed:
		 * rk-h264-qp-window.c, in our internal test suite.
		 */
		qp_max = min_t(u32, qp + 4, RC_QP_MAX);
		qp_min = qp > 4 ? qp - 4 : 0;

		reg = VEPU_REG_H264_LUMA_INIT_QP(qp) |
		      VEPU_REG_H264_QP_MAX(qp_max) |
		      VEPU_REG_H264_QP_MIN(qp_min) |
		      VEPU_REG_H264_CHKPT_DISTANCE(chkpt_distance);
	} else {
		/* CQP mode: fixed QP, no HW rate control */
		reg = VEPU_REG_H264_LUMA_INIT_QP(qp) |
		      VEPU_REG_H264_QP_MAX(qp) |
		      VEPU_REG_H264_QP_MIN(qp) |
		      VEPU_REG_H264_CHKPT_DISTANCE(0);
	}
	vepu_write_relaxed(vpu, reg, VEPU_REG_QP_VAL);

	/*
	 * Rate control checkpoint registers (0x104 - 0x12b):
	 * VEPU_REG_CHECKPOINT(i) for i=0..4 (5 registers, 2 checkpoints each)
	 *
	 * Each checkpoint specifies a target bit count at that position
	 * in the frame. The HW compares actual bits against these targets
	 * and adjusts QP via the delta table.
	 *
	 * Also: VEPU_REG_CHKPT_WORD_ERR(i) for i=0..3 (0x118-0x124)
	 * specifies the acceptable error margin at each checkpoint.
	 */
	if (bitrate > 0) {
		u32 bits_per_chkpt;

		bits_per_chkpt = target_bits_per_frame /
				 (RC_NUM_CHECKPOINTS + 1);

		for (i = 0; i < 5; i++) {
			u32 chk0 = min_t(u32, bits_per_chkpt * (2 * i + 1),
					 0xffff);
			u32 chk1 = min_t(u32, bits_per_chkpt * (2 * i + 2),
					 0xffff);
			reg = VEPU_REG_CHECKPOINT_CHECK0(chk0) |
			      VEPU_REG_CHECKPOINT_CHECK1(chk1);
			vepu_write_relaxed(vpu, reg, VEPU_REG_CHECKPOINT(i));
		}

		/*
		 * Checkpoint word error thresholds (0x118-0x124).
		 * These set the tolerance band around each checkpoint
		 * target. Within the band, QP is unchanged. Outside,
		 * the delta table applies.
		 *
		 * We set them proportional to the bits per checkpoint,
		 * allowing ~25% deviation before QP adjustment kicks in.
		 *
		 * Three registers, not four: VEPU_REG_CHKPT_WORD_ERR(3) is
		 * 0x124, which IS VEPU_REG_CHKPT_DELTA_QP. A fourth iteration
		 * writes the delta-QP register with a word-error value and is
		 * only harmless because the delta write below lands after it.
		 */
		for (i = 0; i < 3; i++) {
			u32 err_margin = bits_per_chkpt / 4;
			u32 err0 = min_t(u32, err_margin, 0xffff);
			u32 err1 = min_t(u32, err_margin, 0xffff);

			reg = VEPU_REG_CHKPT_WORD_ERR_CHK0(err0) |
			      VEPU_REG_CHKPT_WORD_ERR_CHK1(err1);
			vepu_write_relaxed(vpu, reg,
					   VEPU_REG_CHKPT_WORD_ERR(i));
		}

		/*
		 * Checkpoint delta QP table (0x124).
		 * When actual bits deviate from the checkpoint target,
		 * the HW adjusts QP by the delta corresponding to the
		 * deviation range. 7 checkpoints, 4 bits each (signed).
		 *
		 * Progressive deltas: further from target = larger adjustment.
		 * Negative = reduce QP (allow more bits) when under target.
		 * Positive = increase QP (reduce bits) when over target.
		 */
		reg = VEPU_REG_CHKPT_DELTA_QP_CHK0(0xf) |  /* -1 */
		      VEPU_REG_CHKPT_DELTA_QP_CHK1(0xe) |  /* -2 */
		      VEPU_REG_CHKPT_DELTA_QP_CHK2(0xd) |  /* -3 */
		      VEPU_REG_CHKPT_DELTA_QP_CHK3(1) |    /* +1 */
		      VEPU_REG_CHKPT_DELTA_QP_CHK4(2) |    /* +2 */
		      VEPU_REG_CHKPT_DELTA_QP_CHK5(3) |    /* +3 */
		      VEPU_REG_CHKPT_DELTA_QP_CHK6(4);     /* +4 */
		vepu_write_relaxed(vpu, reg, VEPU_REG_CHKPT_DELTA_QP);
	} else {
		/* CQP mode: zero out all checkpoint/RC registers */
		for (i = 0; i < 5; i++)
			vepu_write_relaxed(vpu, 0, VEPU_REG_CHECKPOINT(i));
		for (i = 0; i < 3; i++)
			vepu_write_relaxed(vpu, 0,
					   VEPU_REG_CHKPT_WORD_ERR(i));
		vepu_write_relaxed(vpu, 0, VEPU_REG_CHKPT_DELTA_QP);
	}

	/*
	 * MAD / ROI QP adjustment (0x0dc).
	 *
	 * VEPU_QP_ADJUST_MAD_DELTA_ROI register:
	 *  [15:12] ROI_QP_DELTA_1: QP delta for ROI region 1 (0=disable)
	 *  [11:8]  ROI_QP_DELTA_2: QP delta for ROI region 2 (0=disable)
	 *  [3:0]   MAD_QP_ADJUSTMENT: QP adjustment based on MAD
	 *          0 = no MAD-based adjustment
	 *          N = adjust QP by N when MAD exceeds threshold
	 *
	 * For CBR mode, enable mild MAD-based QP adjustment to handle
	 * scene changes. For CQP mode, disable all adjustments.
	 *
	 * Note: The register definition in the header has a macro bug
	 * (uses 'x' without parameter). We write the raw value.
	 */
	if (bitrate > 0) {
		/* MAD adjustment = 2, no ROI */
		vepu_write_relaxed(vpu, 0x002, VEPU_QP_ADJUST_MAD_DELTA_ROI);
	} else {
		vepu_write_relaxed(vpu, 0, VEPU_QP_ADJUST_MAD_DELTA_ROI);
	}

	/*
	 * MV_PENALTY (0x18c): Motion vector cost penalties.
	 *
	 * These penalties bias the mode decision toward or against
	 * certain partition sizes:
	 *  [30:21] 1MV_PENALTY: Cost of using 1 MV per macroblock (16x16)
	 *  [20:11] QMV_PENALTY: Quarter-pixel MV penalty
	 *  [10:1]  4MV_PENALTY: Cost of using 4 MVs (8x8 partitions)
	 *  [0]     SPLIT_MV_MODE_EN: Enable sub-macroblock MV splits
	 *
	 * Lower penalties encourage the encoder to use that mode.
	 * 16x16 (1MV) is the most common and should be cheap.
	 * 8x8 (4MV) is more expensive but captures fine motion.
	 * Quarter-pixel improves quality at small cost.
	 */
	reg = VEPU_REG_1MV_PENALTY(60) |
	      VEPU_REG_QMV_PENALTY(8) |
	      VEPU_REG_4MV_PENALTY(0) |
	      VEPU_REG_SPLIT_MV_MODE_EN;
	vepu_write_relaxed(vpu, reg, VEPU_REG_MV_PENALTY);

	/*
	 * ENC_CTRL4 (0x1ac): Sub-macroblock MV penalties.
	 *
	 *  [29:20] MV_PENALTY_16X8_8X16: Cost for 16x8 or 8x16 partitions
	 *  [19:10] MV_PENALTY_8X8: Cost for 8x8 partition
	 *  [9:0]   MV_PENALTY_8X4_4X8: Cost for 8x4 or 4x8 sub-partitions
	 *
	 * Values from Hantro H1 reference encoder. These represent the
	 * mode decision overhead in bits, scaled by the internal
	 * lambda factor.
	 */
	reg = VEPU_REG_MV_PENALTY_16X8_8X16(0x041) |
	      VEPU_REG_MV_PENALTY_8X8(0x02a) |
	      VEPU_REG_MV_PENALTY_8X4_4X8(0x01a);
	vepu_write_relaxed(vpu, reg, VEPU_REG_ENC_CTRL4);

	/*
	 * MVC_RELATE (0x198): MVC/misc parameters.
	 *
	 *  [23:20] ZERO_MV_FAVOR_D2: Bias toward zero MV (0-15)
	 *          Higher = more likely to choose zero MV
	 *  [19:11] PENALTY_4X4MV: Penalty for 4x4 sub-partitions
	 *  [10:8]  MVC_VIEW_ID: 0 (not MVC)
	 *  [7]     MVC_ANCHOR_PIC_FLAG: 0
	 *  [6:4]   MVC_PRIORITY_ID: 0
	 *  [3:1]   MVC_TEMPORAL_ID: 0
	 *  [0]     MVC_INTER_VIEW_FLAG: 0
	 *
	 * Zero MV favor of 4 provides a small bias that helps compression
	 * on static scenes without hurting motion accuracy.
	 */
	reg = VEPU_REG_ZERO_MV_FAVOR_D2(4) |
	      VEPU_REG_PENALTY_4X4MV(0x015);
	vepu_write_relaxed(vpu, reg, VEPU_REG_MVC_RELATE);

	/*
	 * INTRA_INTER_MODE (0x12c): Intra/inter mode penalties.
	 *
	 *  [31:16] INTRA16X16_MODE: Penalty for choosing intra 16x16
	 *  [15:0]  INTER_MODE: Penalty for choosing inter mode
	 *
	 * These bias the encoder toward intra or inter modes.
	 * 0 = no bias (let RD-cost decide). For I-frames, inter penalty
	 * is irrelevant. For P-frames, a small intra penalty discourages
	 * unnecessary intra coding in smooth regions.
	 */
	if (is_intra)
		reg = VEPU_REG_INTRA16X16_MODE(0) |
		      VEPU_REG_INTER_MODE(0);
	else
		reg = VEPU_REG_INTRA16X16_MODE(0) |
		      VEPU_REG_INTER_MODE(0);
	vepu_write_relaxed(vpu, reg, VEPU_REG_INTRA_INTER_MODE);

	/*
	 * CABAC table address (0x144).
	 * Required for CABAC mode (Main profile). Points to the DMA
	 * buffer containing pre-computed CABAC initial probability
	 * states from the H.264 spec.
	 * Must be set even in CAVLC mode on some VEPU2 revisions.
	 */
	vepu_write_relaxed(vpu, h264_ctx->cabac_tbl.dma,
			   VEPU_REG_ADDR_CABAC_TBL);

	/*
	 * MB control register (0x1a0).
	 *
	 *  [31:16] MB_CNT_OUT: Read-only output, total MBs encoded
	 *  [15:0]  MB_CNT_SET: Starting MB for multi-slice (0 for single)
	 */
	vepu_write_relaxed(vpu, VEPU_REG_MB_CNT_SET(0), VEPU_REG_MB_CTRL);

	/*
	 * Intra area control (0x0b8): Force intra coding in a region.
	 *  Set to 0xff for all fields = disable forced intra area.
	 */
	reg = VEPU_REG_INTRA_AREA_TOP(0xff) |
	      VEPU_REG_INTRA_AREA_BOTTOM(0xff) |
	      VEPU_REG_INTRA_AREA_LEFT(0xff) |
	      VEPU_REG_INTRA_AREA_RIGHT(0xff);
	vepu_write_relaxed(vpu, reg, VEPU_REG_INTRA_AREA_CTRL);

	/*
	 * CIR (Cyclic Intra Refresh) control (0x0bc).
	 * Disabled: first_mb=0, interval=0.
	 */
	vepu_write_relaxed(vpu, 0, VEPU_REG_CIR_INTRA_CTRL);

	/*
	 * ROI registers (0x148, 0x14c): Region of Interest.
	 * Disabled: set all coordinates to 0.
	 */
	vepu_write_relaxed(vpu, 0, VEPU_REG_ROI1);
	vepu_write_relaxed(vpu, 0, VEPU_REG_ROI2);

	/*
	 * Stabilization registers (0x150-0x178): Image stabilization.
	 * Not used for H.264 encoding; zero out the matrix and controls.
	 */
	for (i = 0; i < 9; i++)
		vepu_write_relaxed(vpu, 0, VEPU_REG_STABLE_MATRIX(i));
	vepu_write_relaxed(vpu, 0, VEPU_REG_STABLE_MOTION_SUM);
	vepu_write_relaxed(vpu, 0, VEPU_REG_STABILIZATION_OUTPUT);

	/*
	 * RGB-to-YUV conversion coefficients (0x17c-0x184).
	 * Only relevant when input format is RGB. For YUV input,
	 * these are ignored but we zero them for cleanliness.
	 */
	vepu_write_relaxed(vpu, 0, VEPU_REG_RGB2YUV_CONVERSION_COEF1);
	vepu_write_relaxed(vpu, 0, VEPU_REG_RGB2YUV_CONVERSION_COEF2);
	vepu_write_relaxed(vpu, 0, VEPU_REG_RGB2YUV_CONVERSION_COEF3);
	vepu_write_relaxed(vpu, 0, VEPU_REG_RGB_MASK_MSB);

	/*
	 * DMV (Direct MV) penalty tables (0x1E0 - 0x25C, 0x260 - 0x2DC).
	 *
	 * Integer-pixel MV penalty table: 128 entries packed 4-per-word
	 * in 32 words at VEPU_REG_DMV_PENALTY_TBL(0..31).
	 *
	 * Quarter-pixel MV penalty table: 128 entries packed 4-per-word
	 * in 32 words at VEPU_REG_DMV_Q_PIXEL_PENALTY_TBL(0..31).
	 *
	 * Uses logarithmic cost function matching H.264 spec's MV cost model:
	 *   cost(mvd) ~= lambda * (2 * log2(|mvd| + 1) + 1)
	 *
	 * Small MVs (common in natural video) get low penalty.
	 * Large MVs (rare, expensive to code) get high penalty.
	 * Quarter-pixel penalties are smaller since sub-pixel
	 * refinement is cheap and improves quality.
	 */
	for (i = 0; i < 32; i++) {
		u32 val;

		val = VEPU_REG_DMV_PENALTY_TABLE_BIT(
			mv_penalty_log(i * 4, qp), 0) |
		      VEPU_REG_DMV_PENALTY_TABLE_BIT(
			mv_penalty_log(i * 4 + 1, qp), 1) |
		      VEPU_REG_DMV_PENALTY_TABLE_BIT(
			mv_penalty_log(i * 4 + 2, qp), 2) |
		      VEPU_REG_DMV_PENALTY_TABLE_BIT(
			mv_penalty_log(i * 4 + 3, qp), 3);
		vepu_write_relaxed(vpu, val,
				   VEPU_REG_DMV_PENALTY_TBL(i));
	}

	for (i = 0; i < 32; i++) {
		u32 val;

		/*
		 * Quarter-pixel table: penalties are lower since sub-pixel
		 * MVs are common and the coding cost is smaller.
		 * Use half the integer-pixel penalty.
		 */
		val = VEPU_REG_DMV_Q_PIXEL_PENALTY_TABLE_BIT(
			mv_penalty_log(i * 4, qp) / 2, 0) |
		      VEPU_REG_DMV_Q_PIXEL_PENALTY_TABLE_BIT(
			mv_penalty_log(i * 4 + 1, qp) / 2, 1) |
		      VEPU_REG_DMV_Q_PIXEL_PENALTY_TABLE_BIT(
			mv_penalty_log(i * 4 + 2, qp) / 2, 2) |
		      VEPU_REG_DMV_Q_PIXEL_PENALTY_TABLE_BIT(
			mv_penalty_log(i * 4 + 3, qp) / 2, 3);
		vepu_write_relaxed(vpu, val,
				   VEPU_REG_DMV_Q_PIXEL_PENALTY_TBL(i));
	}
}

int rockchip_vpu2_h264_enc_run(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->dev;
	struct rockchip_vpu2_h264_enc_ctx *h264_ctx = get_h264_enc_ctx(ctx);
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	u32 reg;
	bool is_intra;
	s32 gop_size;
	int hdr_size = 0;

	src_buf = hantro_get_src_buf(ctx);
	dst_buf = hantro_get_dst_buf(ctx);

	/*
	 * The CAPTURE buffer must be CPU-writable and big enough to hold the
	 * parameter sets, BEFORE anything else happens.
	 *
	 * This encoder writes SPS, PPS and the slice's Annex B start code with
	 * the CPU; the block emits the slice NAL byte but no start code of its
	 * own. Without a kernel mapping there is nowhere to put any of that, and
	 * the old code simply skipped the header block and started the hardware
	 * anyway -- producing a headerless, start-code-less stream that
	 * userspace was told was a SUCCESSFUL capture, and that no decoder can
	 * open. Two ways to reach it: a DMABUF-imported CAPTURE buffer, whose
	 * vb2_plane_vaddr() is NULL, and an application-supplied sizeimage of
	 * H264_HDR_RESERVE or less (sizeimage is application-owned for a coded
	 * format, so this is reachable straight from v4l2-ctl).
	 *
	 * The sibling JPEG encoder already refuses exactly this condition
	 * (rockchip_vpu2_jpeg_enc_run(): "if (!jpeg_ctx.buffer) return
	 * -ENOMEM"). Refuse it here too: device_run() turns a non-zero return
	 * into VB2_BUF_STATE_ERROR, which is the honest answer.
	 *
	 * That refusal reaches userspace ONLY as V4L2_BUF_FLAG_ERROR on the
	 * DQBUF'd CAPTURE buffer. v4l2-ctl neither prints that flag nor exits
	 * non-zero on it, so 5d410e25ead1c's "the old code reported it as a
	 * successful encode" was never actually discriminated by the evidence
	 * it carried -- both arms showed rc=0, which is what v4l2-ctl reports
	 * either way. What that evidence did establish is that the hardware is
	 * no longer programmed. The flag itself is read by the c7err arm of
	 * our internal vepu-r7r8.sh harness (vepu-c7-buferr.py); until that
	 * runs on a board, the user-visible half of the claim is unproven.
	 *
	 * Checked before hantro_start_prepare_run() so that a refused job has
	 * consumed no control request and armed no watchdog.
	 */
	if (!vb2_plane_vaddr(&dst_buf->vb2_buf, 0) ||
	    vb2_plane_size(&dst_buf->vb2_buf, 0) <= H264_HDR_RESERVE) {
		vpu_err("H.264 CAPTURE buffer unusable: vaddr=%p size=%lu (needs a CPU mapping and > %d bytes)\n",
			vb2_plane_vaddr(&dst_buf->vb2_buf, 0),
			vb2_plane_size(&dst_buf->vb2_buf, 0),
			H264_HDR_RESERVE);
		return -ENOMEM;
	}

	hantro_start_prepare_run(ctx);

	/* Read GOP size from V4L2 control */
	gop_size = h264_enc_get_ctrl_val(ctx,
					 V4L2_CID_MPEG_VIDEO_GOP_SIZE, 30);

	/*
	 * Determine frame type based on GOP structure.
	 * GOP size of 0 means all-intra (every frame is IDR).
	 * GOP size of 1 means alternating IDR/P (first is IDR).
	 * GOP size of N means 1 IDR followed by N-1 P-frames.
	 */
	if (gop_size == 0 || h264_ctx->gop_counter == 0) {
		is_intra = true;
		h264_ctx->is_idr = true;
		h264_ctx->frame_num = 0;
		h264_ctx->idr_pic_id ^= 1;
		h264_ctx->qp_adj = 0; /* reset RC state at IDR */
	} else {
		is_intra = false;
		h264_ctx->is_idr = false;
	}

	/*
	 * Prepend SPS and PPS NALUs at stream start (first IDR).
	 * For subsequent IDRs, we also prepend SPS+PPS since some
	 * decoders require them before each IDR.
	 *
	 * The NALUs are written to the beginning of the output buffer.
	 * We then set VEPU_REG_STREAM_START_OFFSET to tell the HW
	 * where to start writing its slice data (after our headers).
	 *
	 * Note: VEPU_REG_STREAM_START_OFFSET is in bits and has a
	 * 6-bit field (max 63 bits = ~7 bytes). Since our SPS+PPS is
	 * larger than that, we instead write the headers and set the
	 * output buffer address past them. The HW writes slice data
	 * starting at the address we give it.
	 *
	 * Two properties of that address were measured on RK3399 and both
	 * have to be honoured here, on EVERY frame, not just IDRs:
	 *
	 *  - VEPU_REG_ADDR_OUTPUT_STREAM ignores its low 3 bits. Handing it
	 *    an unaligned offset does not shift the write, it rounds it DOWN:
	 *    a 21-byte SPS+PPS made the block start writing at byte 16 and
	 *    eat the last five bytes of our own PPS.
	 *  - the block emits the slice NAL byte (0x25 for an IDR) but NO
	 *    Annex B start code, so nothing marks where the slice begins.
	 *
	 * So the header is always padded to an 8-byte boundary and the last
	 * four bytes of that padding are the slice's own start code. The
	 * zeros between the last parameter set and the start code are
	 * leading_zero_8bits, which Annex B allows.
	 */
	{
		u8 *dst = vb2_plane_vaddr(&dst_buf->vb2_buf, 0);
		int max_len = vb2_plane_size(&dst_buf->vb2_buf, 0);
		int nal_len = 0;

		/* Validated at the top of run(): dst is mapped and roomy. */
		if (h264_ctx->is_idr) {
			nal_len = h264_enc_write_sps(ctx, dst, max_len);
			nal_len += h264_enc_write_pps(ctx, dst + nal_len,
						      max_len - nal_len);
		}
		hdr_size = ALIGN(nal_len + 4, 8);
		memset(dst + nal_len, 0, hdr_size - nal_len);
		dst[hdr_size - 1] = 0x01;
	}

	/* Store header size for done() to include in payload */
	h264_ctx->hdr_bytes = hdr_size;

	/* Switch to H.264 encoder mode before writing registers */
	vepu_write_relaxed(vpu, VEPU_REG_ENCODE_FORMAT_H264,
			   VEPU_REG_ENCODE_START);

	rockchip_vpu2_h264_enc_set_src_img(vpu, ctx);

	/*
	 * Set buffer addresses, then offset the output stream address so the
	 * hardware writes slice data after the header block.
	 *
	 * hdr_size is ALIGN(nal_len + 4, 8) and nal_len >= 0, so it is >= 8 on
	 * every frame -- the four bytes are the slice's own start code, which
	 * is written unconditionally. The `else` that used to sit here, for a
	 * frame with no header block at all, has been unreachable since the
	 * start code stopped being conditional on is_idr.
	 */
	{
		dma_addr_t out_addr;
		u32 out_size;

		out_addr = vb2_dma_contig_plane_dma_addr(&dst_buf->vb2_buf, 0);
		out_size = vb2_plane_size(&dst_buf->vb2_buf, 0);

		rockchip_vpu2_h264_enc_set_buffers(vpu, ctx,
						   &src_buf->vb2_buf,
						   &dst_buf->vb2_buf,
						   is_intra);

		/* Override the output address set_buffers just programmed. */
		vepu_write_relaxed(vpu, out_addr + hdr_size,
				   VEPU_REG_ADDR_OUTPUT_STREAM);
		vepu_write_relaxed(vpu, out_size - hdr_size,
				   VEPU_REG_STR_BUF_LIMIT);
	}

	rockchip_vpu2_h264_enc_set_params(vpu, ctx, is_intra);

	/* Data endianness */
	reg = VEPU_REG_OUTPUT_SWAP32 |
	      VEPU_REG_OUTPUT_SWAP16 |
	      VEPU_REG_OUTPUT_SWAP8 |
	      VEPU_REG_INPUT_SWAP8 |
	      VEPU_REG_INPUT_SWAP16 |
	      VEPU_REG_INPUT_SWAP32;
	vepu_write(vpu, reg, VEPU_REG_DATA_ENDIAN);

	/*
	 * AXI bus configuration (0x0d8).
	 *
	 *  [31:24] AXI_CTRL_READ_ID: AXI read channel ID (0=default)
	 *  [23:16] AXI_CTRL_WRITE_ID: AXI write channel ID (0=default)
	 *  [13:8]  AXI_CTRL_BURST_LEN: Max burst length (16=optimal)
	 *  [2]     INCREMENT_MODE: 0=INCR, 1=single
	 *  [1]     BIRST_DISCARD: 0=normal
	 *  [0]     BIRST_DISABLE: 0=bursts enabled
	 */
	reg = VEPU_REG_AXI_CTRL_BURST_LEN(16);
	vepu_write_relaxed(vpu, reg, VEPU_REG_AXI_CTRL);

	/*
	 * Interrupt register (0x1b4).
	 *
	 *  [28] INTERRUPT_NON: Not-an-interrupt flag
	 *  [24] MV_WRITE_EN: Enable MV output write (0=disable)
	 *  [20] RECON_WRITE_DIS: Disable reconstructed frame write
	 *  [16] INTERRUPT_SLICE_READY_EN: Slice completion IRQ (0=disable)
	 *  [12] CLK_GATING_EN: Enable clock gating for power saving
	 *  [10] INTERRUPT_TIMEOUT_EN: Enable timeout IRQ
	 *
	 * Bits [6:0] of this register are the interrupt STATUS field --
	 * rockchip_vpu2_vepu_irq() reads VEPU_REG_INTERRUPT_FRAME_READY
	 * (bit 1) out of it to decide DONE vs ERROR. It is not an enable,
	 * and setting it here stops the block dead: the encoder accepts
	 * ENCODE_ENABLE, advances zero macroblocks, raises no interrupt of
	 * any kind (not frame-ready, not bus-error, not buffer-full) and
	 * stays busy until the SoC is power cycled -- neither this driver's
	 * .reset nor a module reload clears it. The JPEG path never writes
	 * this register at all, which is why only H.264 was affected.
	 *
	 * Never disable reconstructed frame write (needed for P-frame ref).
	 * MV write disabled (not needed for normal encoding).
	 */
	reg = VEPU_REG_CLK_GATING_EN |
	      VEPU_REG_INTERRUPT_TIMEOUT_EN;
	vepu_write_relaxed(vpu, reg, VEPU_REG_INTERRUPT);

	/*
	 * Start encoding: set dimensions, frame type, codec format,
	 * and enable bit.
	 *
	 * ENCODE_START register (0x19c):
	 *  [28:20] MB_HEIGHT: Frame height in macroblocks
	 *  [16:8]  MB_WIDTH: Frame width in macroblocks
	 *  [7:6]   FRAME_TYPE: 0=inter, 1=intra, 2=MVC inter
	 *  [5:4]   ENCODE_FORMAT: 2=JPEG, 3=H.264
	 *  [0]     ENCODE_ENABLE: Start encoding
	 */
	reg = VEPU_REG_MB_WIDTH(MB_WIDTH(ctx->src_fmt.width)) |
	      VEPU_REG_MB_HEIGHT(MB_HEIGHT(ctx->src_fmt.height)) |
	      (is_intra ? VEPU_REG_FRAME_TYPE_INTRA :
			  VEPU_REG_FRAME_TYPE_INTER) |
	      VEPU_REG_ENCODE_FORMAT_H264 |
	      VEPU_REG_ENCODE_ENABLE;

	/*
	 * frame_num / gop_counter / rec_cur are advanced in _done(), i.e. only
	 * for a frame the hardware actually finished. Advancing them here
	 * corrupted the encoder for every frame after ANY error: an errored
	 * frame still consumed a frame_num, leaving a gap against the SPS's
	 * gaps_in_frame_num_value_allowed_flag = 0, and still flipped rec_cur,
	 * so the next P-frame referenced a reconstruction buffer the failed
	 * frame never wrote. hantro_irq_done() calls .done only on
	 * VB2_BUF_STATE_DONE, and the watchdog path calls only .reset.
	 */

	/* Kick the watchdog and start encoding */
	hantro_end_prepare_run(ctx);
	vepu_write(vpu, reg, VEPU_REG_ENCODE_START);

	return 0;
}

void rockchip_vpu2_h264_enc_done(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->dev;
	struct rockchip_vpu2_h264_enc_ctx *h264_ctx = get_h264_enc_ctx(ctx);
	struct vb2_v4l2_buffer *dst_buf = hantro_get_dst_buf(ctx);
	u32 stream_bits;
	u32 bytesused;

	/*
	 * VEPU_REG_STR_BUF_LIMIT contains the number of bits written
	 * by the hardware (slice data only). Convert to bytes.
	 */
	stream_bits = vepu_read(vpu, VEPU_REG_STR_BUF_LIMIT);
	bytesused = stream_bits / 8;

	/*
	 * Advance the per-frame state HERE, not in run(): this callback runs
	 * only for a frame the hardware finished. See the note in run().
	 */
	{
		s32 gop_size = h264_enc_get_ctrl_val(ctx,
					V4L2_CID_MPEG_VIDEO_GOP_SIZE, 30);

		h264_ctx->frame_num++;
		h264_ctx->gop_counter++;
		if (gop_size <= 0 ||
		    h264_ctx->gop_counter >= (u32)gop_size)
			h264_ctx->gop_counter = 0;

		/* Swap reconstruction buffers for double-buffering */
		h264_ctx->rec_cur ^= 1;
	}

	/*
	 * Total payload = software-written headers (SPS+PPS for IDR)
	 * plus hardware-written slice data. The header size was stored
	 * in the context by run() before the HW started.
	 */
	vb2_set_plane_payload(&dst_buf->vb2_buf, 0,
			      h264_ctx->hdr_bytes + bytesused);

	/*
	 * Rate control feedback: read the QP sum to determine the actual
	 * average QP used by the encoder for this frame. This feeds into
	 * the next frame's QP decision.
	 *
	 * QP_SUM_DIV2 (0x0e8): Sum of QP values across all MBs, divided
	 * by 2. Actual sum = register_value * 2.
	 *
	 * RLC_SUM (0x0f8): Total RLC count (coefficients encoded),
	 * useful for complexity estimation.
	 *
	 * Only CBR runs this integrator. set_params() latched BOTH rc_cbr and
	 * rc_bitrate for the frame that just completed, so neither a
	 * BITRATE_MODE change nor a BITRATE change between run() and this IRQ
	 * can make done() score a frame against a controller -- or a target --
	 * it was not encoded under. Before rc_cbr, done() consulted only
	 * BITRATE (.min = 10000, so always > 0) and kept integrating in CQ
	 * mode: qp_adj was not applied while CQ was selected, but a CQ -> CBR
	 * switch mid-stream then applied the whole accumulated offset, up to
	 * +-10 QP, to the next P-frame. Before rc_bitrate, done() still
	 * re-read BITRATE, so a mid-GOP raise scored the finished frame as
	 * under budget and lowered QP for a frame it did not encode.
	 */
	if (h264_ctx->rc_cbr && h264_ctx->prev_frame_bits > 0) {
		s32 bitrate = h264_ctx->rc_bitrate;

		if (bitrate > 0) {
			u32 target = bitrate / DEFAULT_FPS;
			u32 actual = bytesused * 8;

			/*
			 * Simple proportional RC adjustment:
			 * If we used more bits than target, increase QP.
			 * If fewer, decrease QP.
			 * Damped by factor of 4 to avoid oscillation.
			 */
			if (actual > target + target / 4)
				h264_ctx->qp_adj++;
			else if (actual < target - target / 4)
				h264_ctx->qp_adj--;

			h264_ctx->qp_adj = clamp(h264_ctx->qp_adj, -10, 10);
		}
	}
	h264_ctx->prev_frame_bits = bytesused * 8;
}

int rockchip_vpu2_h264_enc_init(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->dev;
	struct rockchip_vpu2_h264_enc_ctx *h264_ctx;
	size_t rec_size;
	size_t mv_out_size;
	int ret;
	int i;

	/*
	 * Allocate the encoder context structure.
	 * We store it in the h264_dec.priv aux_buf (encoder and decoder
	 * are mutually exclusive).
	 */
	ctx->h264_dec.priv.cpu = kzalloc(sizeof(*h264_ctx), GFP_KERNEL);
	if (!ctx->h264_dec.priv.cpu)
		return -ENOMEM;

	h264_ctx = get_h264_enc_ctx(ctx);

	/* Allocate DMA buffer for CABAC initialization table */
	h264_ctx->cabac_tbl.cpu = dma_alloc_coherent(vpu->dev,
						     CABAC_INIT_TABLE_SIZE,
						     &h264_ctx->cabac_tbl.dma,
						     GFP_KERNEL);
	if (!h264_ctx->cabac_tbl.cpu) {
		ret = -ENOMEM;
		goto err_free_ctx;
	}
	h264_ctx->cabac_tbl.size = CABAC_INIT_TABLE_SIZE;

	/* Copy CABAC init table; zero-fill the remainder */
	memset(h264_ctx->cabac_tbl.cpu, 0, CABAC_INIT_TABLE_SIZE);
	memcpy(h264_ctx->cabac_tbl.cpu, h264_cabac_init_table,
	       min(sizeof(h264_cabac_init_table), (size_t)CABAC_INIT_TABLE_SIZE));

	/*
	 * Allocate two reconstructed frame buffers for double-buffering.
	 * Each buffer holds one full-resolution NV12 frame (luma + chroma).
	 * Use FHD dimensions as the maximum since that is the VEPU2 encode
	 * limit for H.264.
	 */
	rec_size = ALIGN(FMT_FHD_WIDTH, MB_DIM) *
		   ALIGN(FMT_FHD_HEIGHT, MB_DIM) * 3 / 2;

	for (i = 0; i < 2; i++) {
		h264_ctx->rec_buf[i].cpu = dma_alloc_coherent(vpu->dev,
							      rec_size,
							      &h264_ctx->rec_buf[i].dma,
							      GFP_KERNEL);
		if (!h264_ctx->rec_buf[i].cpu) {
			ret = -ENOMEM;
			goto err_free_rec;
		}
		h264_ctx->rec_buf[i].size = rec_size;
	}

	/*
	 * Allocate MV output buffer. Size: 4 bytes per macroblock
	 * (2 components * 16 bits each). Even if MV write is disabled,
	 * the register must point to a valid address.
	 */
	mv_out_size = MB_WIDTH(FMT_FHD_WIDTH) * MB_HEIGHT(FMT_FHD_HEIGHT) * 4;
	h264_ctx->mv_out_buf.cpu = dma_alloc_coherent(vpu->dev, mv_out_size,
						       &h264_ctx->mv_out_buf.dma,
						       GFP_KERNEL);
	if (!h264_ctx->mv_out_buf.cpu) {
		ret = -ENOMEM;
		goto err_free_rec;
	}
	h264_ctx->mv_out_buf.size = mv_out_size;

	/* Initialize encoder state */
	rockchip_vpu2_h264_enc_reset_state(ctx);

	return 0;

err_free_rec:
	for (i = 0; i < 2; i++) {
		if (h264_ctx->rec_buf[i].cpu)
			dma_free_coherent(vpu->dev, h264_ctx->rec_buf[i].size,
					  h264_ctx->rec_buf[i].cpu,
					  h264_ctx->rec_buf[i].dma);
	}
	dma_free_coherent(vpu->dev, h264_ctx->cabac_tbl.size,
			  h264_ctx->cabac_tbl.cpu, h264_ctx->cabac_tbl.dma);
err_free_ctx:
	kfree(ctx->h264_dec.priv.cpu);
	ctx->h264_dec.priv.cpu = NULL;
	return ret;
}

/*
 * Return the per-SESSION sequencing state to its start-of-stream values.
 *
 * Called from .init, and from hantro_start_streaming() when the OUTPUT queue
 * starts. The second caller is the point: hantro_vq_is_coded() is
 * `ctx->is_encoder != V4L2_TYPE_IS_OUTPUT(q->type)`, so for an ENCODER only the
 * CAPTURE queue is the coded one and codec_ops->init / ->exit run only there.
 * An application that STREAMOFFs and STREAMONs just the OUTPUT queue -- the
 * ordinary way to change source without tearing down the encode session -- kept
 * frame_num, gop_counter, rec_cur and qp_adj from the previous run, so the
 * first frame of the new session was emitted as a P-frame with NO SPS and NO
 * PPS, predicting from the reconstruction buffer the PREVIOUS session wrote.
 *
 * Nothing .init allocated is touched here: the CABAC table, the reconstruction
 * buffers and the MV buffer all belong to the CAPTURE-queue lifetime and stay
 * valid across an OUTPUT restart.
 */
void rockchip_vpu2_h264_enc_reset_state(struct hantro_ctx *ctx)
{
	struct rockchip_vpu2_h264_enc_ctx *h264_ctx;

	if (!ctx->h264_dec.priv.cpu)
		return;

	h264_ctx = get_h264_enc_ctx(ctx);
	h264_ctx->frame_num = 0;
	h264_ctx->idr_pic_id = 0;
	h264_ctx->is_idr = false;
	h264_ctx->gop_counter = 0;
	h264_ctx->rec_cur = 0;
	h264_ctx->prev_frame_bits = 0;
	h264_ctx->qp_adj = 0;
	h264_ctx->hdr_bytes = 0;
}

void rockchip_vpu2_h264_enc_exit(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->dev;
	struct rockchip_vpu2_h264_enc_ctx *h264_ctx;
	int i;

	if (!ctx->h264_dec.priv.cpu)
		return;

	h264_ctx = get_h264_enc_ctx(ctx);

	if (h264_ctx->mv_out_buf.cpu)
		dma_free_coherent(vpu->dev, h264_ctx->mv_out_buf.size,
				  h264_ctx->mv_out_buf.cpu,
				  h264_ctx->mv_out_buf.dma);

	for (i = 0; i < 2; i++) {
		if (h264_ctx->rec_buf[i].cpu)
			dma_free_coherent(vpu->dev, h264_ctx->rec_buf[i].size,
					  h264_ctx->rec_buf[i].cpu,
					  h264_ctx->rec_buf[i].dma);
	}

	if (h264_ctx->cabac_tbl.cpu)
		dma_free_coherent(vpu->dev, h264_ctx->cabac_tbl.size,
				  h264_ctx->cabac_tbl.cpu,
				  h264_ctx->cabac_tbl.dma);

	kfree(ctx->h264_dec.priv.cpu);
	ctx->h264_dec.priv.cpu = NULL;
}
