// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Libre Computer
 * Author: Da Xue <da@libre.computer>
 *
 * Stateless MPEG-1/2 decoder bridge using the stock gxl_mpeg12.bin firmware.
 *
 * This bridge runs the gxl_mpeg12 firmware as a PERSISTENT CONTINUOUS session
 * (mirror of the stateful codec_mpeg12.c), with a synthesized-ES front end that
 * reconstructs a minimal valid MPEG-2 elementary stream from the V4L2 stateless
 * Request controls and feeds it ONE picture per device_run into the SAME running
 * VIFIFO (continuous bitstream, no restart).
 *
 * ===========================================================================
 * NON-ZERO-COPY CAPTURE PATH (2026-06-24)
 * ===========================================================================
 * The firmware has EXACTLY 8 output canvases (LMEM[0x9] is an 8-bit busy mask;
 * the firmware SELF-SELECTS the lowest-free canvas at decode start, disasm PC
 * 0x03db, and a write of MREG_BUFFERIN = idx+1 RELEASES canvas idx -- clears its
 * busy bit, disasm PC 0x050c; analysis.md 2026-06-24).  In the original
 * ZERO-COPY design each V4L2 CAPTURE buffer's DMA memory WAS a firmware canvas,
 * which capped the V4L2 CAPTURE pool at 8.  A VA-API/ffmpeg consumer whose pool
 * is 9 surfaces (ffmpeg-vaapi MPEG-2) then ENOMEM'd on the 9th CAPTURE buffer.
 *
 * This bridge is now NON-ZERO-COPY:
 *   - It owns a SCRATCH pool of 8 DMA-coherent buffers in the firmware's
 *     canvas/NV12 layout.  The firmware's 8 canvases (AV_SCRATCH_0..7) are bound
 *     to THESE scratch paddrs (codec_mpeg12_libre_setup_scratch).  The firmware
 *     decodes into the scratch pool and self-manages its 2-deep DPB there.
 *   - The V4L2 CAPTURE buffers (N, the consumer's pool -- now allowed to be > 8)
 *     are COPY TARGETS only.  They are NOT bound as canvases.
 *   - On the BUFFEROUT/MBOX1 picture-done post for scratch canvas C, the threaded
 *     ISR (a) takes a free V4L2 CAPTURE buffer off the m2m rdy_queue, (b) COPIES
 *     scratch C -> that V4L2 buffer (both planes, honoring scratch-vs-V4L2
 *     stride), (c) v4l2_m2m_buf_done's the V4L2 buffer to the consumer, (d)
 *     RELEASES scratch C back to the firmware (MREG_BUFFERIN = C+1) under
 *     bufferin_lock.
 *   - Scratch-canvas recycle is therefore BRIDGE-controlled and INDEPENDENT of
 *     how many V4L2 buffers the consumer holds: the firmware reports a canvas as
 *     displayed (evicted from its 2-deep DPB) on BUFFEROUT, the bridge copies it
 *     out, then frees it immediately.  This eliminates the slow-consumer
 *     scratch-starvation residual of the zero-copy supply-only path.
 *
 * Why a CONTINUOUS session, fed per-picture (analysis.md 2026-06-21):
 *   - The STATEFUL codec_mpeg12.c decodes a full GOP (30/30, ~68 dB) on this
 *     same firmware by feeding its bitstream into ONE continuous VIFIFO -- start
 *     / poweron runs ONCE, NO per-frame seqend, the seqend appears only at drain.
 *   - The VIFIFO COMMIT is the parser DMA in esparser_write_data
 *     (esparser.c, run on EVERY esparser_queue); the firmware self-triggers
 *     decode at >=128 VIFIFO bytes.  Each picture is terminated by the NEXT
 *     picture's start code already resident in the VIFIFO (firmware
 *     picture_start handler PC 0x0310) -- NOT by a per-frame 0xB7 seqend.  The
 *     consumer keeps >=2 requests in flight, so picture N's header is resident to
 *     terminate picture N-1 even though each device_run feeds only ONE picture.
 *   - Running start ONCE + appending each picture into the running VIFIFO keeps
 *     the DPB ring alive across the GOP.
 *
 * Feed cadence (the path lives in vdec.c vdec_mpeg2_feed_contiguous): each
 * device_run consumes ONE OUTPUT request, applies its per-request controls, then
 * esparser_queue (-> this pre_queue) appends its [synth headers + slice] into the
 * running VIFIFO, then finishes the m2m job so the scheduler dispatches the next
 * request.  Per fed picture, pre_queue synthesizes:
 *   - frame 0  : sequence_header + sequence_extension + picture_header +
 *                picture_coding_extension, prepended to the slice.
 *   - frame N>0: ONLY picture_header + picture_coding_extension (NO sequence
 *                header, NO 0xB7 seqend), prepended to the slice.
 * Expect a 1-frame delivery lag (picture N delivers when picture N+1's start
 * code arrives); the drain seqend flushes the final picture.
 *
 * Drain seqend: the 0xB7 sequence_end_code is emitted ONLY at drain (the
 * .drain hook, appended by esparser_queue_eos), flushing the final lagged
 * picture.  It is never appended per fed picture.
 *
 * Firmware: stock gxl_mpeg12.bin (no firmware change -- the rework is entirely
 * driver-side).  The bridge speaks gxl's MREG protocol on the scratch range.
 */

#include <media/v4l2-mem2mem.h>
#include <media/v4l2-ctrls.h>
#include <media/videobuf2-v4l2.h>

#include "codec_mpeg12_libre.h"
#include "dos_regs.h"
#include "vdec_helpers.h"
#include "esparser.h"		/* esparser_queue_eos (drain EOS flush) */

#define SIZE_WORKSPACE		SZ_128K
#define WORKSPACE_OFFSET	(5 * SZ_1K)

/*
 * gxl MREG protocol on the scratch range (mirror of the stateful
 * codec_mpeg12.c:22-37): the firmware reads the canvas table from
 * AV_SCRATCH_0..7 into LMEM at init, self-selects the lowest-free canvas per
 * picture, and posts the decoded index via MREG_BUFFEROUT.  MREG_BUFFERIN =
 * idx+1 RELEASES canvas idx (clears its LMEM[0x9] busy bit, disasm PC 0x050c).
 */
#define MREG_PIC_INFO		AV_SCRATCH_5
#define MREG_BUFFERIN		AV_SCRATCH_8	/* driver -> fw: release canvas (idx + 1) */
#define MREG_BUFFEROUT		AV_SCRATCH_9	/* fw -> driver: decoded canvas (idx + 1) */
#define MREG_CMD		AV_SCRATCH_A
#define MREG_CO_MV_START	AV_SCRATCH_B	/* B-frame MV workspace base */
#define MREG_ERROR_COUNT	AV_SCRATCH_C
#define MREG_FRAME_OFFSET	AV_SCRATCH_D
#define MREG_WAIT_BUFFER	AV_SCRATCH_E
#define MREG_FATAL_ERROR	AV_SCRATCH_F

/* BUFFEROUT bits [23:17] all set = non-frame firmware notification (skip). */
#define MPEG12_BUFFEROUT_NOTIFY		GENMASK(23, 17)
/* BUFFEROUT decoded-buffer index lives in the low nibble (idx + 1). */
#define MPEG12_BUFFEROUT_IDX_MASK	0xf

/*
 * Firmware output-canvas slots.  BUFFEROUT reports (idx + 1) masked to the low
 * nibble and the driver indexes canvases with `& 7`, so there are 8 slots.
 * These are now BRIDGE-OWNED SCRATCH buffers (non-zero-copy), NOT V4L2 buffers.
 */
#define MPEG12_LIBRE_NUM_CANVAS		8

/* PIC_INFO field/progressive bits (mirror codec_mpeg12.c:39-40). */
#define PICINFO_PROG		BIT(15)
#define PICINFO_TOP_FIRST	BIT(13)

/*
 * Bogus-offset clamp (mirror codec_mpeg12.c:64): after a hardware reset the VLD
 * can report a stale high offset for the first decoded frame.  Any offset with
 * the high bit set is treated as garbage and clamped to 0.
 */
#define MPEG12_BOGUS_OFFSET_THRESHOLD	0x80000000

/*
 * Synthesized-ES front end.  The driver reconstructs a minimal valid MPEG-2 ES
 * from the V4L2 stateless controls and prepends it to the slice data so the
 * UNMODIFIED stock gxl_mpeg12 firmware parses it natively (the parse path is
 * what arms INT2 on SM1).  Kernel port of the P1-validated reference
 * mpeg12_es_synth.c from our internal test suite (ffmpeg round-trip
 * bit-exact).
 * Field order is verbatim ISO/IEC 13818-2 6.2.2.1/6.2.2.3/6.2.3/6.2.3.1.
 */

/* MSB-first bit writer over a caller-provided, zero-initialised buffer. */
struct mpeg12_bw {
	u8     *buf;
	size_t  cap;
	size_t  bit;
	int     overflow;
};

static void mpeg12_bw_put(struct mpeg12_bw *bw, unsigned int nbits, u32 val)
{
	int i;

	for (i = (int)nbits - 1; i >= 0; i--) {
		size_t byte = bw->bit >> 3;
		unsigned int off = 7u - (bw->bit & 7u);

		if (byte >= bw->cap) {
			bw->overflow = 1;
			return;
		}
		if ((val >> i) & 1u)
			bw->buf[byte] |= (u8)(1u << off);
		bw->bit++;
	}
}

static void mpeg12_bw_align(struct mpeg12_bw *bw)
{
	while (bw->bit & 7u) {
		if ((bw->bit >> 3) >= bw->cap) {
			bw->overflow = 1;
			return;
		}
		bw->bit++;	/* bits already 0 */
	}
}

static void mpeg12_bw_start_code(struct mpeg12_bw *bw, u8 code)
{
	mpeg12_bw_align(bw);
	mpeg12_bw_put(bw, 32, 0x00000100u | (u32)code);	/* 00 00 01 <code> */
}

/*
 * sequence_header() (6.2.2.1) + (MPEG-2 only) sequence_extension() (6.2.2.3).
 */
static void mpeg12_bw_seq(struct mpeg12_bw *bw,
			  const struct v4l2_ctrl_mpeg2_sequence *seq,
			  const struct v4l2_ctrl_mpeg2_quantisation *quant,
			  bool is_mpeg1)
{
	int i;

	/* --- sequence_header() (6.2.2.1) --- */
	mpeg12_bw_start_code(bw, 0xB3);
	mpeg12_bw_put(bw, 12, seq->horizontal_size & 0xfff);
	mpeg12_bw_put(bw, 12, seq->vertical_size & 0xfff);
	mpeg12_bw_put(bw, 4, 0x1);		/* aspect_ratio_information (display-only) */
	mpeg12_bw_put(bw, 4, 0x3);		/* frame_rate_code (display-only) */
	mpeg12_bw_put(bw, 18, 0x3ffff);		/* bit_rate_value (display-only) */
	mpeg12_bw_put(bw, 1, 1);		/* marker_bit */
	mpeg12_bw_put(bw, 10, seq->vbv_buffer_size & 0x3ff);
	mpeg12_bw_put(bw, 1, 0);		/* constrained_parameters_flag */
	if (quant) {
		mpeg12_bw_put(bw, 1, 1);	/* load_intra_quantiser_matrix */
		for (i = 0; i < 64; i++)
			mpeg12_bw_put(bw, 8, quant->intra_quantiser_matrix[i]);
		mpeg12_bw_put(bw, 1, 1);	/* load_non_intra_quantiser_matrix */
		for (i = 0; i < 64; i++)
			mpeg12_bw_put(bw, 8, quant->non_intra_quantiser_matrix[i]);
	} else {
		mpeg12_bw_put(bw, 1, 0);
		mpeg12_bw_put(bw, 1, 0);
	}

	if (is_mpeg1)
		return;				/* MPEG-1: no sequence_extension */

	/* --- sequence_extension() (6.2.2.3) --- */
	mpeg12_bw_start_code(bw, 0xB5);
	mpeg12_bw_put(bw, 4, 0x1);		/* extension_start_code_identifier: Sequence */
	mpeg12_bw_put(bw, 8, seq->profile_and_level_indication & 0xff);
	mpeg12_bw_put(bw, 1,
		      (seq->flags & V4L2_MPEG2_SEQ_FLAG_PROGRESSIVE) ? 1 : 0);
	mpeg12_bw_put(bw, 2, seq->chroma_format & 0x3);
	mpeg12_bw_put(bw, 2, (seq->horizontal_size >> 12) & 0x3);
	mpeg12_bw_put(bw, 2, (seq->vertical_size >> 12) & 0x3);
	mpeg12_bw_put(bw, 12, 0);		/* bit_rate_extension */
	mpeg12_bw_put(bw, 1, 1);		/* marker_bit */
	mpeg12_bw_put(bw, 8, (seq->vbv_buffer_size >> 10) & 0xff);
	mpeg12_bw_put(bw, 1, 0);		/* low_delay */
	mpeg12_bw_put(bw, 2, 0);		/* frame_rate_extension_n */
	mpeg12_bw_put(bw, 5, 0);		/* frame_rate_extension_d */
}

/*
 * picture_header() (6.2.3) + (MPEG-2 only) picture_coding_extension() (6.2.3.1).
 */
static void mpeg12_bw_pic(struct mpeg12_bw *bw,
			  const struct v4l2_ctrl_mpeg2_picture *pic,
			  u16 temporal_reference, bool is_mpeg1)
{
	unsigned int ct = pic->picture_coding_type & 0x7;
	unsigned int fwd = is_mpeg1 ? (pic->f_code[0][0] & 0x7) : 0x7;
	unsigned int bwd = is_mpeg1 ? (pic->f_code[1][0] & 0x7) : 0x7;
	int prog;

	/* --- picture_header() (6.2.3) --- */
	mpeg12_bw_start_code(bw, 0x00);
	mpeg12_bw_put(bw, 10, temporal_reference & 0x3ff);
	mpeg12_bw_put(bw, 3, ct);		/* picture_coding_type */
	mpeg12_bw_put(bw, 16, 0xffff);		/* vbv_delay */
	if (ct == 2 || ct == 3) {		/* P or B */
		mpeg12_bw_put(bw, 1, 0);	/* full_pel_forward_vector (half-pel) */
		mpeg12_bw_put(bw, 3, fwd);	/* forward_f_code (MPEG-1: real; MPEG-2: 0b111) */
	}
	if (ct == 3) {				/* B */
		mpeg12_bw_put(bw, 1, 0);	/* full_pel_backward_vector */
		mpeg12_bw_put(bw, 3, bwd);	/* backward_f_code */
	}
	mpeg12_bw_put(bw, 1, 0);		/* extra_bit_picture = 0 */

	if (is_mpeg1)
		return;				/* MPEG-1: no picture_coding_extension */

	/* --- picture_coding_extension() (6.2.3.1) --- */
	mpeg12_bw_start_code(bw, 0xB5);
	mpeg12_bw_put(bw, 4, 0x8);		/* extension_start_code_identifier: Picture Coding */
	mpeg12_bw_put(bw, 4, pic->f_code[0][0] & 0xf);
	mpeg12_bw_put(bw, 4, pic->f_code[0][1] & 0xf);
	mpeg12_bw_put(bw, 4, pic->f_code[1][0] & 0xf);
	mpeg12_bw_put(bw, 4, pic->f_code[1][1] & 0xf);
	mpeg12_bw_put(bw, 2, pic->intra_dc_precision & 0x3);
	mpeg12_bw_put(bw, 2, pic->picture_structure & 0x3);
	prog = (pic->flags & V4L2_MPEG2_PIC_FLAG_PROGRESSIVE) ? 1 : 0;
	mpeg12_bw_put(bw, 1, (pic->flags & V4L2_MPEG2_PIC_FLAG_TOP_FIELD_FIRST) ? 1 : 0);
	mpeg12_bw_put(bw, 1, (pic->flags & V4L2_MPEG2_PIC_FLAG_FRAME_PRED_DCT) ? 1 : 0);
	mpeg12_bw_put(bw, 1, (pic->flags & V4L2_MPEG2_PIC_FLAG_CONCEALMENT_MV) ? 1 : 0);
	mpeg12_bw_put(bw, 1, (pic->flags & V4L2_MPEG2_PIC_FLAG_Q_SCALE_TYPE) ? 1 : 0);
	mpeg12_bw_put(bw, 1, (pic->flags & V4L2_MPEG2_PIC_FLAG_INTRA_VLC) ? 1 : 0);
	mpeg12_bw_put(bw, 1, (pic->flags & V4L2_MPEG2_PIC_FLAG_ALT_SCAN) ? 1 : 0);
	mpeg12_bw_put(bw, 1, (pic->flags & V4L2_MPEG2_PIC_FLAG_REPEAT_FIRST) ? 1 : 0);
	mpeg12_bw_put(bw, 1, prog);		/* chroma_420_type = progressive_frame */
	mpeg12_bw_put(bw, 1, prog);		/* progressive_frame */
	mpeg12_bw_put(bw, 1, 0);		/* composite_display_flag */
}

/*
 * Build the per-device_run MPEG-2 headers into out[0..out_cap).  Returns the
 * byte length written (byte-aligned) or -1 on overflow.
 */
static int mpeg12_build_headers(u8 *out, size_t out_cap, bool with_seq,
				const struct v4l2_ctrl_mpeg2_sequence *seq,
				const struct v4l2_ctrl_mpeg2_picture *pic,
				const struct v4l2_ctrl_mpeg2_quantisation *quant,
				u16 temporal_reference)
{
	struct mpeg12_bw bw = { .buf = out, .cap = out_cap, .bit = 0, .overflow = 0 };
	bool is_mpeg1 = (seq->profile_and_level_indication == 0);

	memset(out, 0, out_cap);

	if (with_seq)
		mpeg12_bw_seq(&bw, seq, quant, is_mpeg1);

	mpeg12_bw_pic(&bw, pic, temporal_reference, is_mpeg1);

	mpeg12_bw_align(&bw);	/* byte-align so the following slice start code is aligned */

	if (bw.overflow)
		return -1;
	return (int)(bw.bit >> 3);
}

/*
 * One bridge-owned scratch canvas buffer.  The firmware decodes into this DMA
 * memory; the threaded ISR copies it out to a V4L2 CAPTURE buffer on display.
 */
struct mpeg12_scratch_buf {
	void	  *vaddr;
	dma_addr_t paddr;
};

struct codec_mpeg12_libre {
	void	  *workspace_vaddr;
	dma_addr_t workspace_paddr;
	u16	   temporal_ref;

	/*
	 * Continuous-session frame counter.  frames_fed == 0 selects the
	 * first-frame feed (sequence header + picture); >0 selects the
	 * picture-only feed.  The firmware is NOT re-initialised per frame;
	 * this only gates which ES headers the synthesizer emits.
	 */
	u32	   frames_fed;
	/* Pictures delivered to userspace (incremented in the threaded ISR). */
	u32	   frames_delivered;

	/*
	 * NON-ZERO-COPY scratch canvas pool.  Eight DMA-coherent buffers in the
	 * firmware's NV12 canvas layout, bound to AV_SCRATCH_0..7 at start.  The
	 * firmware self-selects the lowest-free among these (disasm 0x03db) and
	 * the bridge releases one on BUFFEROUT (MREG_BUFFERIN, disasm 0x050c).
	 */
	struct mpeg12_scratch_buf scratch[MPEG12_LIBRE_NUM_CANVAS];
	u32	   scratch_count;
	size_t	   scratch_buf_size;	/* per-buffer total (Y + UV) */
	u32	   scratch_stride;	/* ALIGN(width, 64) -- bytes/line, Y and UV */
	u32	   scratch_y_height;	/* ALIGN(height, 32) */
	/*
	 * Canvases the firmware has posted via BUFFEROUT (displayed/evicted from
	 * its 2-deep DPB) but the bridge has not yet released.  Released under
	 * bufferin_lock by the threaded ISR once the copy-out is done.  Tracked
	 * only for diagnostics + double-post guarding.
	 */
	u32	   displayed_mask;

	/*
	 * DELIVERY ring.  The HARDIRQ (codec_mpeg12_libre_isr) reads + CLEARS
	 * MREG_BUFFEROUT, does the copy_out, RELEASES the scratch canvas
	 * (MREG_BUFFERIN) -- all at zero scheduling latency -- then stashes the
	 * filled V4L2 buffer here; the threaded ISR drains it and does the
	 * consumer hand-off (amvdec_dst_buf_done_offset), which is not
	 * latency-critical.  Doing the clear+copy+release in the hardirq is
	 * load-bearing: the firmware busy-gates the next picture at PC 0x038f /
	 * busy-spins the seqend slot-2 at PC 0x0300-0x0301 until the host clears
	 * BUFFEROUT, and busy-waits the canvas via MREG_BUFFERIN -- deferring the
	 * release to the threaded ISR added 600-900us of schedule jitter that
	 * re-crossed the recycle-lag frontier even after the copy was cached.
	 * SPSC under bo_lock; head/tail are free-running u8 (count =
	 * (u8)(head-tail), index = %8).
	 */
	struct {
		struct vb2_v4l2_buffer *dst_buf;  /* filled buffer to deliver (or NULL) */
		u32 pic_info;
		u32 offset;
		u32 canvas;
		int copy_ret;		/* copy_out result (0 = ok) */
	}	   bo_ring[8];
	u8	   bo_head;
	u8	   bo_tail;
	spinlock_t bo_lock;
	/*
	 * Serializes the two MREG_BUFFERIN (canvas-release) writers.  NOT bo_lock
	 * (which is taken in the hardirq -- deadlock-unsafe to share for an
	 * irqsave critical section that also writes DOS regs).  Taken
	 * spin_lock_irqsave.  Today only the threaded ISR writes MREG_BUFFERIN,
	 * but the lock keeps the release atomic vs any future writer (e.g. a
	 * .recycle hook) per the analysis.md 2026-06-24 two-writer design.
	 */
	spinlock_t bufferin_lock;
};

/*
 * Allocate the 8 scratch canvas buffers and bind the firmware's 8 canvases
 * (AV_SCRATCH_0..7) to the scratch paddrs in the firmware's NV12 layout.  Called
 * once at start.  The geometry MUST match what set_canvas_nv12 would program for
 * a real V4L2 buffer (Y stride = ALIGN(w,64), Y height = ALIGN(h,32), UV plane
 * at offset Y_stride*Y_height) so the firmware's MC output lands correctly and
 * the copy-out geometry is well-defined.
 */
static int codec_mpeg12_libre_setup_scratch(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_mpeg12_libre *mpeg12 = sess->priv;
	u32 stride = ALIGN(sess->width, 64);
	u32 y_height = ALIGN(sess->height, 32);
	size_t y_size = (size_t)stride * y_height;
	size_t buf_size = y_size + y_size / 2;		/* NV12: Y + UV(=Y/2) */
	u32 regs[MPEG12_LIBRE_NUM_CANVAS] = { 0 };
	u32 reg_num = MPEG12_LIBRE_NUM_CANVAS;
	dma_addr_t paddrs[MPEG12_LIBRE_NUM_CANVAS];
	int i, ret;

	mpeg12->scratch_stride = stride;
	mpeg12->scratch_y_height = y_height;
	mpeg12->scratch_buf_size = buf_size;

	for (i = 0; i < MPEG12_LIBRE_NUM_CANVAS; i++) {
		/*
		 * COHERENT (uncached) scratch.  A CACHED (dma_alloc_noncoherent)
		 * mapping + dma_sync_single_for_cpu(invalidate) in the hardirq
		 * copy_out produced NON-DETERMINISTIC luma corruption: the firmware
		 * posts MREG_BUFFEROUT before its MC/deblock write-response DMA has
		 * fully drained the picture to DRAM, and the cache-invalidate does
		 * NOT wait for those writes -- so the fast cached read can pick up
		 * partially-written DRAM, tearing a non-deterministic set of
		 * macroblock-row bands (17-25 dB), chroma intact (it drains first).
		 * No register exposes per-row write completion in the hardirq
		 * (MC_STATUS0 never reads idle at BUFFEROUT, MCW_DBLK_WRRSP_CNT
		 * never stabilizes), and a wall-time delay only helps
		 * non-monotonically, so a hardirq write-completion fence cannot make
		 * the cached copy reliable.  A coherent mapping has the CPU read
		 * current DRAM directly with no cache-vs-DMA race -> reliably clean
		 * luma (58-70 dB, all frames incl. the first GOP).  The copy is
		 * slower (uncached read BW) but correctness wins.
		 */
		mpeg12->scratch[i].vaddr =
			dma_alloc_coherent(core->dev, buf_size,
					   &mpeg12->scratch[i].paddr,
					   GFP_KERNEL);
		if (!mpeg12->scratch[i].vaddr) {
			ret = -ENOMEM;
			goto free_scratch;
		}
		paddrs[i] = mpeg12->scratch[i].paddr;
		regs[i] = AV_SCRATCH_0 + i * 4;
	}
	mpeg12->scratch_count = MPEG12_LIBRE_NUM_CANVAS;

	/*
	 * Program the 8 canvas-ID registers (AV_SCRATCH_0..7) to point at the
	 * scratch paddrs.  amvdec_set_scratch_canvases mirrors set_canvas_nv12's
	 * Y/UV layout but takes raw paddrs instead of a vb2 buffer.
	 */
	ret = amvdec_set_scratch_canvases(sess, regs, paddrs, reg_num);
	if (ret) {
		dev_err(core->dev,
			"mpeg12 libre: scratch canvas setup failed: %d\n", ret);
		goto free_scratch;
	}

	return 0;

free_scratch:
	for (i = 0; i < MPEG12_LIBRE_NUM_CANVAS; i++) {
		if (mpeg12->scratch[i].vaddr) {
			dma_free_coherent(core->dev, buf_size,
					  mpeg12->scratch[i].vaddr,
					  mpeg12->scratch[i].paddr);
			mpeg12->scratch[i].vaddr = NULL;
		}
	}
	mpeg12->scratch_count = 0;
	return ret;
}

static void codec_mpeg12_libre_free_scratch(struct amvdec_session *sess)
{
	struct codec_mpeg12_libre *mpeg12 = sess->priv;
	struct amvdec_core *core = sess->core;
	int i;

	for (i = 0; i < MPEG12_LIBRE_NUM_CANVAS; i++) {
		if (mpeg12->scratch[i].vaddr) {
			dma_free_coherent(core->dev, mpeg12->scratch_buf_size,
					  mpeg12->scratch[i].vaddr,
					  mpeg12->scratch[i].paddr);
			mpeg12->scratch[i].vaddr = NULL;
		}
	}
	mpeg12->scratch_count = 0;
}

/*
 * Per-fed-picture header synthesis, driven by the CONTIGUOUS feed
 * (vdec_mpeg2_feed_contiguous -> esparser_queue -> pre_queue).  The src buffer's
 * per-request controls were applied by the feed loop immediately before this
 * call, so the controls read here belong to THIS picture.
 *
 * NON-ZERO-COPY: this no longer binds or supplies a V4L2 CAPTURE buffer.  The
 * firmware self-selects the lowest-free SCRATCH canvas; the bridge supplies no
 * decode-target index.  This function only prepends the synthesized MPEG-2
 * headers to the slice so the firmware parses + decodes the picture into the
 * scratch pool.
 *
 * Frame 0 carries sequence_header + sequence_extension once; every subsequent
 * picture carries only its picture_header + picture_coding_ext.  NO 0xB7
 * sequence_end_code is appended per picture -- the next picture's resident start
 * code terminates this one; the drain seqend flushes the final lagged picture.
 */
static int codec_mpeg12_libre_pre_queue(struct amvdec_session *sess,
					struct vb2_buffer *vb)
{
	struct amvdec_core *core = sess->core;
	struct codec_mpeg12_libre *mpeg12 = sess->priv;
	struct v4l2_ctrl *ctrl;
	const struct v4l2_ctrl_mpeg2_picture *pic;
	const struct v4l2_ctrl_mpeg2_sequence *seq = NULL;
	const struct v4l2_ctrl_mpeg2_quantisation *quant = NULL;
	bool with_seq = (mpeg12->frames_fed == 0);
	u8 hdr[256];
	u8 *p;
	u32 payload;
	int hlen;

	ctrl = v4l2_ctrl_find(&sess->ctrl_handler,
			      V4L2_CID_STATELESS_MPEG2_PICTURE);
	if (!ctrl) {
		dev_err(core->dev, "mpeg12 libre: no picture control\n");
		return -EINVAL;
	}
	pic = ctrl->p_cur.p;

	ctrl = v4l2_ctrl_find(&sess->ctrl_handler,
			      V4L2_CID_STATELESS_MPEG2_SEQUENCE);
	if (ctrl)
		seq = ctrl->p_cur.p;
	if (!seq) {
		dev_err(core->dev, "mpeg12 libre: no sequence control\n");
		return -EINVAL;
	}

	ctrl = v4l2_ctrl_find(&sess->ctrl_handler,
			      V4L2_CID_STATELESS_MPEG2_QUANTISATION);
	if (ctrl)
		quant = ctrl->p_cur.p;

	hlen = mpeg12_build_headers(hdr, sizeof(hdr), with_seq,
				    seq, pic, quant, mpeg12->temporal_ref++);
	if (hlen < 0) {
		dev_err(core->dev, "mpeg12 libre: header synth overflow\n");
		return -ENOSPC;
	}

	p = vb2_plane_vaddr(vb, 0);
	payload = vb2_get_plane_payload(vb, 0);
	if (!p || payload + (u32)hlen > vb2_plane_size(vb, 0)) {
		dev_err(core->dev,
			"mpeg12 libre: no headroom to prepend headers\n");
		return -ENOSPC;
	}
	memmove(p + hlen, p, payload);
	memcpy(p, hdr, (size_t)hlen);
	vb2_set_plane_payload(vb, 0, payload + (u32)hlen);

	dev_dbg(core->dev,
		"mpeg12 libre: FEED picture (fed=%u, with_seq=%d)\n",
		mpeg12->frames_fed, with_seq);

	mpeg12->frames_fed++;

	return 0;
}

static int codec_mpeg12_libre_start(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_mpeg12_libre *mpeg12;
	int ret;

	mpeg12 = kzalloc(sizeof(*mpeg12), GFP_KERNEL);
	if (!mpeg12)
		return -ENOMEM;

	spin_lock_init(&mpeg12->bo_lock);	/* BUFFEROUT hardirq<->thread ring */
	spin_lock_init(&mpeg12->bufferin_lock);	/* canvas-release serialization */

	sess->priv = mpeg12;

	mpeg12->workspace_vaddr = dma_alloc_coherent(core->dev, SIZE_WORKSPACE,
						     &mpeg12->workspace_paddr,
						     GFP_KERNEL);
	if (!mpeg12->workspace_vaddr) {
		ret = -ENOMEM;
		goto free_mpeg12;
	}

	/*
	 * Full IQIDCT/MC/VLD reset from BSP -- runs ONCE per session.
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

	amvdec_write_dos(core, POWER_CTL_VLD, POWER_CTL_VLD_ENABLE);

	/*
	 * gxl MREG init (mirror codec_mpeg12.c:167-179). AV_SCRATCH_B is the
	 * B-frame MV workspace base, BUFFERIN/BUFFEROUT the buffer handshake,
	 * CMD = (w<<16)|h.
	 */
	amvdec_write_dos(core, MREG_CO_MV_START,
			 mpeg12->workspace_paddr + WORKSPACE_OFFSET);
	amvdec_write_dos(core, MPEG1_2_REG, 0);
	amvdec_write_dos(core, PSCALE_CTRL, 0);
	amvdec_write_dos(core, PIC_HEAD_INFO, PIC_HEAD_INFO_INIT);
	amvdec_write_dos(core, M4_CONTROL_REG, 0);
	amvdec_write_dos(core, MREG_BUFFERIN, 0);
	amvdec_write_dos(core, MREG_BUFFEROUT, 0);
	amvdec_write_dos(core, MREG_CMD, (sess->width << 16) | sess->height);
	amvdec_write_dos(core, MREG_ERROR_COUNT, 0);
	amvdec_write_dos(core, MREG_FATAL_ERROR, 0);
	amvdec_write_dos(core, MREG_WAIT_BUFFER, 0);

	/*
	 * NON-ZERO-COPY: bind the firmware's 8 canvases (AV_SCRATCH_0..7) to the
	 * BRIDGE-OWNED scratch pool, NOT to the V4L2 CAPTURE buffers.  The
	 * firmware reads all 8 canvases into LMEM at init and decodes into them;
	 * the threaded ISR copies each displayed scratch canvas out to a V4L2
	 * buffer.  Decoupling the firmware's 8-canvas pool from the V4L2 pool is
	 * what lets the consumer allocate > 8 CAPTURE buffers.
	 */
	ret = codec_mpeg12_libre_setup_scratch(sess);
	if (ret)
		goto free_workspace;

	sess->keyframe_found = 1;

	return 0;

free_workspace:
	dma_free_coherent(core->dev, SIZE_WORKSPACE,
			  mpeg12->workspace_vaddr, mpeg12->workspace_paddr);
free_mpeg12:
	kfree(mpeg12);
	sess->priv = NULL;
	return ret;
}

static int codec_mpeg12_libre_stop(struct amvdec_session *sess)
{
	struct codec_mpeg12_libre *mpeg12 = sess->priv;
	struct amvdec_core *core = sess->core;

	/*
	 * codec_ops->stop is called from BOTH vdec.c (codec unwind) and
	 * vdec_1.c (engine unwind, gated on sess->priv non-NULL). NULL
	 * sess->priv after kfree so the second call doesn't double-free.
	 */
	if (!mpeg12)
		return 0;

	codec_mpeg12_libre_free_scratch(sess);

	if (mpeg12->workspace_vaddr) {
		dma_free_coherent(core->dev, SIZE_WORKSPACE,
				  mpeg12->workspace_vaddr,
				  mpeg12->workspace_paddr);
		mpeg12->workspace_vaddr = NULL;
	}

	kfree(mpeg12);
	sess->priv = NULL;
	return 0;
}

static int codec_mpeg12_libre_copy_out(struct amvdec_session *sess, u32 canvas,
				       struct vb2_v4l2_buffer *vbuf);

static irqreturn_t codec_mpeg12_libre_isr(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_mpeg12_libre *mpeg12 = sess->priv;
	u32 reg, pic_info, offset, canvas;
	struct vb2_v4l2_buffer *dst_buf = NULL;
	int copy_ret = -ENOBUFS;

	/*
	 * Clear MREG_BUFFEROUT IMMEDIATELY to release the firmware's BUFFEROUT-
	 * wait (busy-gate PC 0x038f, seqend slot-2 spin PC 0x0300-0x0301).  NOTIFY
	 * (bits 23:17) is a non-frame event (still cleared, not a picture).
	 */
	reg = amvdec_read_dos(core, MREG_BUFFEROUT);
	if (!reg)
		return IRQ_WAKE_THREAD;
	pic_info = amvdec_read_dos(core, MREG_PIC_INFO);
	offset = amvdec_read_dos(core, MREG_FRAME_OFFSET);
	amvdec_write_dos(core, MREG_BUFFEROUT, 0);

	if ((reg & MPEG12_BUFFEROUT_NOTIFY) == MPEG12_BUFFEROUT_NOTIFY)
		return IRQ_WAKE_THREAD;

	canvas = ((reg & MPEG12_BUFFEROUT_IDX_MASK) - 1) & 7;
	if (canvas >= mpeg12->scratch_count)
		return IRQ_WAKE_THREAD;	/* bogus canvas: cannot release, drop */

	/*
	 * Copy_out + RELEASE the scratch canvas HERE in the hardirq, at zero
	 * scheduling latency.  copy_out reads the (cached) scratch -- the
	 * MREG_BUFFERIN release MUST follow the copy (the firmware may reuse the
	 * canvas the instant it is released).  Delivery (buf_done) is deferred to
	 * the threaded ISR via the ring.
	 */
	dst_buf = v4l2_m2m_dst_buf_remove(sess->m2m_ctx);
	if (dst_buf)
		copy_ret = codec_mpeg12_libre_copy_out(sess, canvas, dst_buf);

	/* RELEASE: firmware clears LMEM[0x9] bit (canvas) at PC 0x050c. */
	amvdec_write_dos(core, MREG_BUFFERIN, canvas + 1);

	spin_lock(&mpeg12->bo_lock);
	if ((u8)(mpeg12->bo_head - mpeg12->bo_tail) <
	    ARRAY_SIZE(mpeg12->bo_ring)) {
		u8 h = mpeg12->bo_head % ARRAY_SIZE(mpeg12->bo_ring);

		mpeg12->bo_ring[h].dst_buf = dst_buf;
		mpeg12->bo_ring[h].pic_info = pic_info;
		mpeg12->bo_ring[h].offset = offset;
		mpeg12->bo_ring[h].canvas = canvas;
		mpeg12->bo_ring[h].copy_ret = copy_ret;
		mpeg12->bo_head++;
	} else if (dst_buf) {
		/* ring full (should not happen, 8-deep vs 8 canvases): error it */
		v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_ERROR);
	}
	spin_unlock(&mpeg12->bo_lock);

	return IRQ_WAKE_THREAD;
}

/*
 * Copy one decoded SCRATCH canvas (NV12) into a V4L2 CAPTURE buffer.  Honors the
 * scratch canvas stride vs the V4L2 plane stride: if they match, the planes are
 * contiguous and copy in one shot; otherwise copy line-by-line.  Handles both
 * single-plane NV12/NV21 (Y then UV in one plane) and multi-plane NV12M/NV21M.
 * Returns 0 on success, <0 if the V4L2 buffer is too small (should not happen --
 * vdec_queue_setup sizes the V4L2 plane >= the canvas).
 */
static int codec_mpeg12_libre_copy_out(struct amvdec_session *sess,
				       u32 canvas,
				       struct vb2_v4l2_buffer *vbuf)
{
	struct amvdec_core *core = sess->core;
	struct codec_mpeg12_libre *mpeg12 = sess->priv;
	const u8 *src = mpeg12->scratch[canvas].vaddr;
	u32 s_stride = mpeg12->scratch_stride;		/* Y + UV both use s_stride */

	/*
	 * COHERENT scratch: the CPU reads current DRAM directly, so no cache
	 * sync is needed (and none can fence the engine's in-flight writes).
	 */
	u32 y_lines = mpeg12->scratch_y_height;
	u32 uv_lines = y_lines / 2;
	const u8 *src_y = src;
	const u8 *src_uv = src + (size_t)s_stride * y_lines;
	/*
	 * Visible image dimensions: copy only the visible width per line (the
	 * scratch canvas may be wider than sess->width due to 64-byte
	 * alignment, but the V4L2 plane is sized to the same aligned stride, so
	 * copying the full aligned stride is safe and simplest).  We copy the
	 * full aligned stride to keep both planes byte-aligned with what the
	 * firmware wrote.
	 */
	u32 copy_w = s_stride;

	if (sess->pixfmt_cap == V4L2_PIX_FMT_NV12M ||
	    sess->pixfmt_cap == V4L2_PIX_FMT_NV21M) {
		u8 *dst_y = vb2_plane_vaddr(&vbuf->vb2_buf, 0);
		u8 *dst_uv = vb2_plane_vaddr(&vbuf->vb2_buf, 1);
		u32 dy = s_stride;	/* V4L2 plane stride == scratch stride */
		u32 i;

		if (!dst_y || !dst_uv)
			return -EFAULT;
		if (vb2_plane_size(&vbuf->vb2_buf, 0) < (size_t)dy * y_lines ||
		    vb2_plane_size(&vbuf->vb2_buf, 1) < (size_t)dy * uv_lines)
			return -ENOSPC;

		/*
		 * Fast path: when the V4L2 plane stride matches the scratch
		 * stride AND we copy the full stride, each plane is contiguous in
		 * both src and dst -- one memcpy per plane instead of a per-line
		 * loop.  The scratch is dma_alloc_coherent (uncached on ARM64), so
		 * the per-line loop's hundreds of small uncached reads dominate the
		 * BUFFEROUT->MREG_BUFFERIN recycle lag (~1.3ms measured); a single
		 * wide memcpy lets the CPU burst the uncached reads.
		 */
		if (copy_w == dy && copy_w == s_stride) {
			memcpy(dst_y, src_y, (size_t)s_stride * y_lines);
			memcpy(dst_uv, src_uv, (size_t)s_stride * uv_lines);
		} else {
			for (i = 0; i < y_lines; i++)
				memcpy(dst_y + (size_t)i * dy,
				       src_y + (size_t)i * s_stride, copy_w);
			for (i = 0; i < uv_lines; i++)
				memcpy(dst_uv + (size_t)i * dy,
				       src_uv + (size_t)i * s_stride, copy_w);
		}
	} else {
		/* Single-plane NV12/NV21: Y then UV in plane 0. */
		u8 *dst = vb2_plane_vaddr(&vbuf->vb2_buf, 0);
		u32 dy = s_stride;	/* V4L2 plane stride == scratch stride */
		size_t y_bytes = (size_t)dy * y_lines;
		size_t uv_bytes = (size_t)dy * uv_lines;
		u32 i;

		if (!dst)
			return -EFAULT;
		if (vb2_plane_size(&vbuf->vb2_buf, 0) < y_bytes + uv_bytes)
			return -ENOSPC;

		/* Contiguous fast path (see NV12M branch above). */
		if (copy_w == dy && copy_w == s_stride) {
			memcpy(dst, src_y, y_bytes);
			memcpy(dst + y_bytes, src_uv, uv_bytes);
		} else {
			for (i = 0; i < y_lines; i++)
				memcpy(dst + (size_t)i * dy,
				       src_y + (size_t)i * s_stride, copy_w);
			for (i = 0; i < uv_lines; i++)
				memcpy(dst + y_bytes + (size_t)i * dy,
				       src_uv + (size_t)i * s_stride, copy_w);
		}
	}

	(void)core;
	return 0;
}

static irqreturn_t codec_mpeg12_libre_threaded_isr(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_mpeg12_libre *mpeg12 = sess->priv;
	u32 reg;

	amvdec_write_dos(core, ASSIST_MBOX1_CLR_REG, 1);

	reg = amvdec_read_dos(core, MREG_FATAL_ERROR);
	if (reg) {
		dev_err(core->dev, "mpeg12 libre: firmware fatal error 0x%x\n",
			reg);
		amvdec_abort(sess);
		return IRQ_HANDLED;
	}

	/*
	 * Drain the delivery ring the hardirq filled.  The copy_out + scratch
	 * canvas RELEASE were already done in the hardirq (zero-latency recycle);
	 * here we only do the consumer hand-off (buf_done), which is not
	 * latency-critical.  The seqend posts its two held refs back-to-back, so
	 * more than one entry can accumulate before this thread runs.
	 */
	for (;;) {
		u32 pic_info, offset, canvas, field = V4L2_FIELD_NONE;
		unsigned long flags;
		struct vb2_v4l2_buffer *dst_buf;
		int copy_ret;

		spin_lock_irqsave(&mpeg12->bo_lock, flags);
		if (mpeg12->bo_tail == mpeg12->bo_head) {
			spin_unlock_irqrestore(&mpeg12->bo_lock, flags);
			break;
		}
		{
			u8 t = mpeg12->bo_tail % ARRAY_SIZE(mpeg12->bo_ring);

			dst_buf = mpeg12->bo_ring[t].dst_buf;
			pic_info = mpeg12->bo_ring[t].pic_info;
			offset = mpeg12->bo_ring[t].offset;
			canvas = mpeg12->bo_ring[t].canvas;
			copy_ret = mpeg12->bo_ring[t].copy_ret;
			mpeg12->bo_tail++;
		}
		spin_unlock_irqrestore(&mpeg12->bo_lock, flags);

		if (dst_buf && !copy_ret) {
			if (!(pic_info & PICINFO_PROG))
				field = (pic_info & PICINFO_TOP_FIRST) ?
					V4L2_FIELD_INTERLACED_TB :
					V4L2_FIELD_INTERLACED_BT;
			if (offset >= MPEG12_BOGUS_OFFSET_THRESHOLD)
				offset = 0;

			amvdec_dst_buf_done_offset(sess, dst_buf,
						   offset, field, true);
			mpeg12->frames_delivered++;
			sess->stateless_frames_done++;
		} else if (dst_buf) {
			dev_warn(core->dev,
				 "mpeg12 libre: copy-out canvas %u failed (%d)\n",
				 canvas, copy_ret);
			v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_ERROR);
		} else {
			/*
			 * No V4L2 buffer was free in the hardirq; the canvas was
			 * still released (the firmware needs it back) and the
			 * picture dropped.  Account it so drain does not hang.
			 */
			atomic_dec_if_positive(&sess->esparser_queued_bufs);
			wake_up(&sess->esparser_queue_done_wq);
			sess->stateless_frames_done++;
		}
	}

	return IRQ_HANDLED;
}

/*
 * Drain-only sequence_end_code.  Emitted ONLY when the .drain hook appends it on
 * V4L2_DEC_CMD_STOP to flush the final lagged picture.  It is NEVER appended per
 * fed picture.  Mirror codec_mpeg12.c.
 */
static const u8 eos_sequence[SZ_1K] = { 0x00, 0x00, 0x01, 0xB7 };

/*
 * End-of-stream drain.  At EOS the firmware still holds its last 1-2 reference
 * pictures (the 1-deep reorder lag).  Those have no terminating next-picture
 * start code, so feed the 0xB7 sequence_end_code: its handler (PC 0x02e8) posts
 * the held refs.  We send one seqend and wait, IRQ-paced, for the threaded ISR
 * to copy out + deliver + release the tail.
 */
static void codec_mpeg12_libre_drain(struct amvdec_session *sess)
{
	struct codec_mpeg12_libre *mpeg12 = sess->priv;
	struct amvdec_core *core = sess->core;
	int attempts;

	/*
	 * DEC_CMD_STOP can fire while the session is torn down (sess->priv freed
	 * / never set) -- e.g. a STOP issued by a stalled consumer.  Without this
	 * guard the mpeg12->frames_fed read NULL-derefs and reboots the board
	 * (codec_mpeg12_libre_drain+0x1c).  A drain on a dead session is a no-op.
	 */
	if (!mpeg12)
		return;

	if (mpeg12->frames_fed == mpeg12->frames_delivered)
		return;

	esparser_queue_eos(core, eos_sequence, ARRAY_SIZE(eos_sequence));

	/* Wait for the ISR to flush the held tail.  200 * 2.5ms = 0.5s ceiling. */
	for (attempts = 0;
	     mpeg12->frames_fed > mpeg12->frames_delivered && attempts < 200;
	     attempts++)
		usleep_range(2000, 3000);
}

/*
 * NON-ZERO-COPY feed window.  Gate the contiguous MPEG2 feed on the SCRATCH
 * canvas pool the firmware actually decodes into (the bridge releases one on
 * every BUFFEROUT in the threaded ISR, even when it drops a picture), NOT on
 * free V4L2 CAPTURE buffers.  Here the V4L2 pool is DECOUPLED from the firmware
 * canvases (the ISR copies scratch -> V4L2), so gating the feed on free V4L2
 * buffers collapses the window when the consumer lags and deadlocks the
 * firmware at ~6 frames: feed stops -> the continuous-feed firmware never gets
 * pic N+1's start code to terminate pic N -> OUTPUT requests never complete ->
 * the consumer cannot DQBUF-OUTPUT/REINIT to recycle -> the window stays
 * collapsed.  scratch_count (8) is a constant pool the bridge always keeps
 * available, so in_flight is bounded only by MPEG2_CONT_MAX_INFLIGHT and the
 * circular dependency cannot form.  See analysis.md 2026-06-24.
 */
static u32 codec_mpeg12_libre_feed_window(struct amvdec_session *sess)
{
	struct codec_mpeg12_libre *mpeg12 = sess->priv;

	return mpeg12->scratch_count;
}

struct amvdec_codec_ops codec_mpeg12_libre_ops = {
	.start = codec_mpeg12_libre_start,
	.stop = codec_mpeg12_libre_stop,
	.pre_queue = codec_mpeg12_libre_pre_queue,
	.isr = codec_mpeg12_libre_isr,
	.threaded_isr = codec_mpeg12_libre_threaded_isr,
	.feed_window = codec_mpeg12_libre_feed_window,
	/*
	 * NON-ZERO-COPY: NO .setup_buffers (the firmware decodes into the
	 * bridge-owned scratch pool, not the V4L2 buffers, so there is nothing
	 * to bind per device_run).  NO .can_recycle/.recycle: scratch-canvas
	 * release is BRIDGE-controlled in the threaded ISR on BUFFEROUT
	 * (MREG_BUFFERIN = canvas+1), INDEPENDENT of the consumer's re-QBUF, so
	 * the recycle thread is not needed.  The active .drain owns the EOS
	 * flush.
	 */
	.drain = codec_mpeg12_libre_drain,
};

static int __init codec_mpeg12_libre_mod_init(void)
{
	return amvdec_register_codec(V4L2_PIX_FMT_MPEG2_SLICE,
				     &codec_mpeg12_libre_ops);
}

static void __exit codec_mpeg12_libre_mod_exit(void)
{
	amvdec_unregister_codec(V4L2_PIX_FMT_MPEG2_SLICE,
				&codec_mpeg12_libre_ops);
}

module_init(codec_mpeg12_libre_mod_init);
module_exit(codec_mpeg12_libre_mod_exit);
MODULE_DESCRIPTION("Meson VDEC MPEG-1/2 libre stateless codec");
MODULE_LICENSE("GPL");
