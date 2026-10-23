// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018 BayLibre, SAS
 * Author: Maxime Jourdan <mjourdan@baylibre.com>
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "codec_mpeg12.h"
#include "dos_regs.h"
#include "esparser.h"
#include "vdec_helpers.h"

#define SIZE_WORKSPACE		SZ_128K
/* Offset substracted by the firmware from the workspace paddr */
#define WORKSPACE_OFFSET	(5 * SZ_1K)

/* map firmware registers to known MPEG1/2 functions */
#define MREG_SEQ_INFO		AV_SCRATCH_4
	#define MPEG2_SEQ_DAR_MASK	GENMASK(3, 0)
	#define MPEG2_DAR_4_3		2
	#define MPEG2_DAR_16_9		3
	#define MPEG2_DAR_221_100	4
#define MREG_PIC_INFO		AV_SCRATCH_5
#define MREG_PIC_WIDTH		AV_SCRATCH_6
#define MREG_PIC_HEIGHT		AV_SCRATCH_7
#define MREG_BUFFERIN		AV_SCRATCH_8
#define MREG_BUFFEROUT		AV_SCRATCH_9
#define MREG_CMD		AV_SCRATCH_A
#define MREG_CO_MV_START	AV_SCRATCH_B
#define MREG_ERROR_COUNT	AV_SCRATCH_C
#define MREG_FRAME_OFFSET	AV_SCRATCH_D
#define MREG_WAIT_BUFFER	AV_SCRATCH_E
#define MREG_FATAL_ERROR	AV_SCRATCH_F

#define PICINFO_PROG		BIT(15)
#define PICINFO_TOP_FIRST	BIT(13)

/*
 * Non-frame firmware notification: BUFFEROUT bits [23:17] all set.
 * The firmware uses this pattern to signal status/error events
 * (e.g., sequence header parsed, error conditions) rather than
 * decoded frame delivery.
 */
#define MPEG12_BUFFEROUT_NOTIFY	GENMASK(23, 17)

/*
 * BUFFEROUT buffer index extraction:
 * The firmware writes (buffer_index + 1) in bits [3:0].
 * Subtract 1 and mask to 3 bits for the 0-7 buffer range.
 */
#define MPEG12_BUFFEROUT_IDX_MASK	0xf

/*
 * Bogus-offset threshold for first-frame timestamp.  On the first
 * decoded frame the firmware can report a stale high offset (e.g.
 * 0xffffff20) from prior session state.  Any offset with the high
 * bit set is treated as garbage and clamped to 0 so amvdec_dst_buf_done
 * falls back to the session's esparser queue timestamp instead.
 */
#define MPEG12_BOGUS_OFFSET_THRESHOLD	0x80000000

/* PIC_HEAD_INFO_INIT is defined in dos_regs.h */

struct codec_mpeg12 {
	/* Buffer for the MPEG1/2 Workspace */
	void	  *workspace_vaddr;
	dma_addr_t workspace_paddr;

	/*
	 * Serializes the read-and-clear of MREG_BUFFEROUT between the
	 * threaded ISR and codec_mpeg12_drain's poll loops.  On GXL the
	 * drain does NOT mask the ISR (the ISR is the correct in-order
	 * deliverer there; see codec_mpeg12_drain), so both contexts can
	 * race to consume the same firmware frame post -- delivering it
	 * twice (the sporadic +1 drain frame).  Claiming a post = reading
	 * BUFFEROUT and clearing it to 0 atomically under this lock, so a
	 * given post is consumed exactly once.  Uncontended (a no-op)
	 * everywhere except GXL-during-drain: during normal decode only
	 * the ISR runs, and on G12A+/SM1 the drain masks the ISR so only
	 * the poll runs.
	 */
	spinlock_t bufferout_lock;
};

static const u8 eos_sequence[SZ_1K] = { 0x00, 0x00, 0x01, 0xB7 };

static const u8 *codec_mpeg12_eos_sequence(u32 *len)
{
	*len = ARRAY_SIZE(eos_sequence);
	return eos_sequence;
}

static int codec_mpeg12_can_recycle(struct amvdec_core *core)
{
	return !amvdec_read_dos(core, MREG_BUFFERIN);
}

static void codec_mpeg12_recycle(struct amvdec_core *core, u32 buf_idx)
{
	amvdec_write_dos(core, MREG_BUFFERIN, buf_idx + 1);
}

static int codec_mpeg12_start(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_mpeg12 *mpeg12;
	int ret;

	mpeg12 = kzalloc(sizeof(*mpeg12), GFP_KERNEL);
	if (!mpeg12)
		return -ENOMEM;

	spin_lock_init(&mpeg12->bufferout_lock);

	/* Allocate some memory for the MPEG1/2 decoder's state */
	mpeg12->workspace_vaddr = dma_alloc_coherent(core->dev, SIZE_WORKSPACE,
						     &mpeg12->workspace_paddr,
						     GFP_KERNEL);
	if (!mpeg12->workspace_vaddr) {
		dev_err(core->dev, "Failed to request MPEG 1/2 Workspace\n");
		ret = -ENOMEM;
		goto free_mpeg12;
	}

	ret = amvdec_set_canvases(sess, (u32[]){ AV_SCRATCH_0, 0 },
					(u32[]){ 8, 0 });
	if (ret)
		goto free_workspace;

	/*
	 * Full BSP IQIDCT/MC/VLD reset sequence to flush stale SRAM
	 * data.  The vdec_1_start() reset (DOS_SW_RESET0=0xfffffffc)
	 * does not include MDEC_SW_RESET, so IQIDCT SRAM can retain
	 * data from prior decode sessions on all platforms.
	 *
	 * On G12A+, SRAM is never hardware-zeroed, causing ~34dB PSNR
	 * (vs ~62dB correct) due to corrupted DCT coefficients.
	 * On GXL, SRAM powers up zeroed at boot but can accumulate
	 * stale data across decode sessions, causing intermittent
	 * MPEG1 PSNR drops (~27-30dB) on ~9% of decodes.
	 *
	 * The stateless codec (codec_mpeg12_stateless.c) already
	 * performs this reset unconditionally on all platforms.
	 */
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

	amvdec_write_dos(core, POWER_CTL_VLD, POWER_CTL_VLD_ENABLE);
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
	 * On G12A+, reset IQIDCT_CONTROL to clear stale state from prior
	 * decode sessions. The IQIDCT block retains bits set by other
	 * codecs (e.g. H.264 sets bit 16 for dcac_dma_read_cache_disable).
	 * Without clearing on SM1, PSNR alternates between ~26dB and ~34dB
	 * on successive decodes.
	 *
	 * On GXL, do NOT write this register -- the hardware reset value
	 * is correct and writing 0 causes PSNR regression (65dB -> 26dB).
	 * The BSP mpeg12 driver does not touch IQIDCT_CONTROL.
	 */
	if (core->platform->revision >= VDEC_REVISION_G12A)
		amvdec_write_dos(core, IQIDCT_CONTROL, 0);

	/*
	 * On SM1, MDEC_PIC_DC_CTRL bit 31 is required for proper VDEC1
	 * decoder operation.  The vdec_1 start path clears it before
	 * codec start, so codecs that need it must set it explicitly.
	 * Without this, SM1/G12B produce lower-quality output (PSNR drop
	 * from ~62dB to ~34dB).
	 */
	if (core->platform->revision >= VDEC_REVISION_G12A)
		amvdec_write_dos_bits(core, MDEC_PIC_DC_CTRL,
				      MDEC_PIC_DC_CTRL_OLD_MODE);

	sess->keyframe_found = 1;
	sess->priv = mpeg12;

	return 0;

free_workspace:
	dma_free_coherent(core->dev, SIZE_WORKSPACE, mpeg12->workspace_vaddr,
			  mpeg12->workspace_paddr);
free_mpeg12:
	kfree(mpeg12);

	return ret;
}

static int codec_mpeg12_stop(struct amvdec_session *sess)
{
	struct codec_mpeg12 *mpeg12 = sess->priv;
	struct amvdec_core *core = sess->core;

	if (mpeg12->workspace_vaddr) {
		dma_free_coherent(core->dev, SIZE_WORKSPACE,
				  mpeg12->workspace_vaddr,
				  mpeg12->workspace_paddr);
		mpeg12->workspace_vaddr = NULL;
	}

	return 0;
}

static void codec_mpeg12_update_dar(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	u32 seq = amvdec_read_dos(core, MREG_SEQ_INFO);
	u32 ar = seq & MPEG2_SEQ_DAR_MASK;

	switch (ar) {
	case MPEG2_DAR_4_3:
		amvdec_set_par_from_dar(sess, 4, 3);
		break;
	case MPEG2_DAR_16_9:
		amvdec_set_par_from_dar(sess, 16, 9);
		break;
	case MPEG2_DAR_221_100:
		amvdec_set_par_from_dar(sess, 221, 100);
		break;
	default:
		sess->pixelaspect.numerator = 1;
		sess->pixelaspect.denominator = 1;
		break;
	}
}

static irqreturn_t codec_mpeg12_threaded_isr(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_mpeg12 *mpeg12 = sess->priv;
	u32 reg;
	u32 pic_info;
	u32 is_progressive;
	u32 buffer_index;
	u32 field = V4L2_FIELD_NONE;
	u32 offset;

	amvdec_write_dos(core, ASSIST_MBOX1_CLR_REG, 1);
	reg = amvdec_read_dos(core, MREG_FATAL_ERROR);
	if (reg == 1) {
		dev_err(core->dev, "MPEG1/2 fatal error\n");
		amvdec_abort(sess);
		return IRQ_HANDLED;
	}

	/*
	 * Claim the firmware frame post atomically: read BUFFEROUT (and
	 * its PIC_INFO/FRAME_OFFSET companions) then clear BUFFEROUT to 0
	 * under the lock, so the drain poll cannot consume the same post.
	 * Process (deliver) outside the lock.
	 */
	spin_lock(&mpeg12->bufferout_lock);
	reg = amvdec_read_dos(core, MREG_BUFFEROUT);
	if (reg) {
		pic_info = amvdec_read_dos(core, MREG_PIC_INFO);
		offset = amvdec_read_dos(core, MREG_FRAME_OFFSET);
		amvdec_write_dos(core, MREG_BUFFEROUT, 0);
	}
	spin_unlock(&mpeg12->bufferout_lock);

	if (!reg)
		return IRQ_HANDLED;

	/* Non-frame firmware notification -- status/error event */
	if ((reg & MPEG12_BUFFEROUT_NOTIFY) == MPEG12_BUFFEROUT_NOTIFY)
		return IRQ_HANDLED;

	is_progressive = pic_info & PICINFO_PROG;

	if (!is_progressive)
		field = (pic_info & PICINFO_TOP_FIRST) ?
			V4L2_FIELD_INTERLACED_TB :
			V4L2_FIELD_INTERLACED_BT;

	codec_mpeg12_update_dar(sess);
	buffer_index = ((reg & MPEG12_BUFFEROUT_IDX_MASK) - 1) & 7;

	/*
	 * Fix up obviously bogus stream offsets.  After a hardware reset
	 * the VLD can retain residual state that causes the firmware to
	 * report a stale offset (e.g. 0xffffff20) for the first decoded
	 * frame.  The frame DATA is valid (the firmware decoded into the
	 * correct canvas buffer), but the offset would cause timestamp
	 * matching to consume the wrong timestamp.  Clamp to 0 so it
	 * matches the first queued timestamp instead.
	 */
	if (offset >= MPEG12_BOGUS_OFFSET_THRESHOLD) {
		dev_dbg(core->dev,
			"MPEG12: fixing bogus offset %08x -> 0\n",
			offset);
		offset = 0;
	}

	amvdec_dst_buf_done_idx(sess, buffer_index, offset, field);

	return IRQ_HANDLED;
}

static irqreturn_t codec_mpeg12_isr(struct amvdec_session *sess)
{
	return IRQ_WAKE_THREAD;
}

/**
 * codec_mpeg12_drain() - Actively drain buffered frames from the firmware.
 *
 * The MPEG1/2 firmware buffers frames internally for B-frame reordering.
 * At end-of-stream, a sequence_end_code (0x000001B7) triggers the firmware
 * to flush these buffered frames.  The firmware writes each frame's
 * buffer_index+1 to MREG_BUFFEROUT and busy-waits until the driver clears
 * it to 0.
 *
 * The passive vdec_wait_inactive() approach (200ms timeout) misses frames
 * because:
 *  - The firmware may still be flushing when the timeout expires
 *  - Without active buffer recycling, the firmware stalls waiting for
 *    BUFFEROUT to be cleared
 *
 * This function sends the EOS sequence, then actively polls BUFFEROUT
 * to collect all remaining frames.
 */
static void codec_mpeg12_drain(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_mpeg12 *mpeg12 = sess->priv;
	u32 reg, buffer_index, pic_info, offset;
	u32 field;
	u32 eos_len;
	const u8 *eos_data;
	bool deliver;
	int attempts = 0;

	/*
	 * Keep the MBOX1 interrupt ENABLED during drain on every platform
	 * so the IRQ-paced threaded ISR delivers the firmware's EOS-flush
	 * frames in display order -- the BUFFEROUT bufferout_lock makes the
	 * ISR and the poll loop mutually exclusive, so they cannot
	 * double-consume a post (the GXL +1 race) and the poll never has to
	 * free-run-deliver the tail.
	 *
	 * History: G12A+ used to DISABLE MBOX1 here (and the poll became the
	 * sole consumer) to avoid a different failure -- with should_stop=1
	 * the ISR DROPS frames when all capture buffers are in userspace,
	 * losing 30-70% of frames on SM1.  But the free-running poll delivers
	 * the EOS-flush tail OUT OF DISPLAY ORDER (frames mis-slotted +
	 * duplicated -- proven by MSE frame-map on GXL-masked, G12B and SM1).
	 * The correct combination is: leave the ISR enabled (paced, in-order
	 * delivery) + clear should_stop so the ISR DEFERS instead of dropping
	 * + serialize BUFFEROUT via the lock.  This unifies all platforms on
	 * the in-order ISR-delivery path that GXL always used.
	 *
	 * should_stop is cleared only on G12A+ (matching the historical
	 * defer-vs-drop fix); GXL is left at should_stop=1, its verified-good
	 * state.  The firmware keeps running (MPSR!=0), posting BUFFEROUT and
	 * spinning until the driver clears it (ISR or poll, lock-serialized).
	 */
	if (core->platform->revision >= VDEC_REVISION_G12A)
		sess->should_stop = 0;

	/*
	 * Pre-EOS active drain: poll BUFFEROUT to pick up any frames
	 * the firmware decoded but the ISR hasn't serviced yet.  After
	 * delivering each frame, recycle its canvas immediately so the
	 * firmware can continue decoding.  This breaks a canvas
	 * starvation deadlock where the firmware cannot decode because
	 * all 8 canvases are held in the VB2 done queue.
	 *
	 * 1000 iterations * 2ms = 2s timeout.  The 2ms poll interval
	 * (vs 10ms) gives 5x better responsiveness to firmware BUFFEROUT
	 * writes; we notice a new frame ~2ms after the firmware posts it
	 * instead of up to 10ms.  Saves 4ms average per drain frame.
	 */
	while (atomic_read(&sess->esparser_queued_bufs) > 0 &&
	       attempts++ < 1000) {
		/* Claim the post under the lock (see bufferout_lock). */
		deliver = false;
		spin_lock(&mpeg12->bufferout_lock);
		reg = amvdec_read_dos(core, MREG_BUFFEROUT);
		if (reg) {
			deliver = (reg & MPEG12_BUFFEROUT_NOTIFY) !=
				  MPEG12_BUFFEROUT_NOTIFY;
			if (deliver) {
				pic_info = amvdec_read_dos(core, MREG_PIC_INFO);
				offset = amvdec_read_dos(core, MREG_FRAME_OFFSET);
			}
			amvdec_write_dos(core, MREG_BUFFEROUT, 0);
		}
		spin_unlock(&mpeg12->bufferout_lock);

		if (deliver) {
			field = V4L2_FIELD_NONE;
			if (!(pic_info & PICINFO_PROG))
				field = (pic_info & PICINFO_TOP_FIRST) ?
					V4L2_FIELD_INTERLACED_TB :
					V4L2_FIELD_INTERLACED_BT;

			buffer_index = ((reg & MPEG12_BUFFEROUT_IDX_MASK) - 1) & 7;
			if (offset >= MPEG12_BOGUS_OFFSET_THRESHOLD)
				offset = 0;
			amvdec_dst_buf_done_idx(sess, buffer_index,
						offset, field);
			if (codec_mpeg12_can_recycle(core))
				codec_mpeg12_recycle(core, buffer_index);
			continue;
		}
		usleep_range(2000, 3000);
	}

	if (atomic_read(&sess->esparser_queued_bufs) <= 0)
		return;

	/*
	 * Send sequence_end_code to trigger firmware flush of any
	 * internally buffered frames (B-frame reordering delay).
	 */
	eos_data = codec_mpeg12_eos_sequence(&eos_len);
	esparser_queue_eos(core, eos_data, eos_len);
	usleep_range(5000, 6000);

	/*
	 * Poll BUFFEROUT for EOS-flushed frames.  Use
	 * esparser_queued_bufs as the primary exit condition since
	 * the threaded ISR races with this loop.
	 *
	 * Max 1000 iterations * 2ms = 2s timeout.
	 */
	attempts = 0;
	while (attempts++ < 1000) {
		if (atomic_read(&sess->esparser_queued_bufs) <= 0)
			break;

		/* Claim the post under the lock (see bufferout_lock). */
		deliver = false;
		spin_lock(&mpeg12->bufferout_lock);
		reg = amvdec_read_dos(core, MREG_BUFFEROUT);
		if (reg) {
			deliver = (reg & MPEG12_BUFFEROUT_NOTIFY) !=
				  MPEG12_BUFFEROUT_NOTIFY;
			if (deliver) {
				pic_info = amvdec_read_dos(core, MREG_PIC_INFO);
				offset = amvdec_read_dos(core, MREG_FRAME_OFFSET);
			}
			amvdec_write_dos(core, MREG_BUFFEROUT, 0);
		}
		spin_unlock(&mpeg12->bufferout_lock);

		if (!reg) {
			usleep_range(2000, 3000);
			continue;
		}
		if (!deliver)
			continue;

		field = V4L2_FIELD_NONE;
		if (!(pic_info & PICINFO_PROG))
			field = (pic_info & PICINFO_TOP_FIRST) ?
				V4L2_FIELD_INTERLACED_TB :
				V4L2_FIELD_INTERLACED_BT;

		buffer_index = ((reg & MPEG12_BUFFEROUT_IDX_MASK) - 1) & 7;
		if (offset >= MPEG12_BOGUS_OFFSET_THRESHOLD)
			offset = 0;
		amvdec_dst_buf_done_idx(sess, buffer_index, offset, field);

		if (codec_mpeg12_can_recycle(core))
			codec_mpeg12_recycle(core, buffer_index);
	}

	/*
	 * The gxl_mpeg12.bin firmware has a known limitation: its
	 * sequence_end handler (0x02e8) skips one of two buffered
	 * frames when the backward reference index is invalid, which
	 * occurs with I/P-only content.  The firmware consumes the
	 * source bitstream but never signals BUFFEROUT for the last
	 * reference frame.
	 *
	 * When this happens, esparser_queued_bufs is stuck at 1 with
	 * BUFFEROUT=0 and WAIT_BUFFER=0 (firmware idle).  Force the
	 * counter to zero so the post-drain wait in vdec_drain_work
	 * does not stall for 5s.  The affected frame's pixels are
	 * correct (decoded into a canvas) but are lost to userspace.
	 */
	if (atomic_read(&sess->esparser_queued_bufs) > 0) {
		dev_dbg(core->dev,
			"MPEG12 drain: forcing queued count from %d to 0 (firmware EOS limitation)\n",
			atomic_read(&sess->esparser_queued_bufs));
		atomic_set(&sess->esparser_queued_bufs, 0);
	}

	/* Restore should_stop (MBOX1 was never disabled -- see drain entry). */
	if (core->platform->revision >= VDEC_REVISION_G12A)
		sess->should_stop = 1;
}

struct amvdec_codec_ops codec_mpeg12_ops = {
	.start = codec_mpeg12_start,
	.stop = codec_mpeg12_stop,
	.isr = codec_mpeg12_isr,
	.threaded_isr = codec_mpeg12_threaded_isr,
	.can_recycle = codec_mpeg12_can_recycle,
	.recycle = codec_mpeg12_recycle,
	.drain = codec_mpeg12_drain,
};

static int __init codec_mpeg12_mod_init(void)
{
	int ret;

	ret = amvdec_register_codec(V4L2_PIX_FMT_MPEG1, &codec_mpeg12_ops);
	if (ret)
		return ret;

	ret = amvdec_register_codec(V4L2_PIX_FMT_MPEG2, &codec_mpeg12_ops);
	if (ret) {
		amvdec_unregister_codec(V4L2_PIX_FMT_MPEG1, &codec_mpeg12_ops);
		return ret;
	}

	return 0;
}

static void __exit codec_mpeg12_mod_exit(void)
{
	amvdec_unregister_codec(V4L2_PIX_FMT_MPEG2, &codec_mpeg12_ops);
	amvdec_unregister_codec(V4L2_PIX_FMT_MPEG1, &codec_mpeg12_ops);
}

module_init(codec_mpeg12_mod_init);
module_exit(codec_mpeg12_mod_exit);
MODULE_DESCRIPTION("Meson VDEC MPEG-1/2 codec");
MODULE_LICENSE("GPL");
