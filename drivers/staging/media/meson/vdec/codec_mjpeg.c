// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024
 *
 * MJPEG decoder, using VDEC_1 engine.
 * Based on BSP vmjpeg.c and upstream codec_mpeg12.c pattern.
 */

#include <linux/module.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "codec_mjpeg.h"
#include "dos_regs.h"
#include "esparser.h"
#include "vdec_helpers.h"

/* Firmware register mapping */
#define MREG_DECODE_PARAM	AV_SCRATCH_2
#define MREG_TO_AMRISC		AV_SCRATCH_8
#define MREG_FROM_AMRISC	AV_SCRATCH_9
#define MREG_FRAME_OFFSET	AV_SCRATCH_A
#define MREG_RP_WORKAROUND	AV_SCRATCH_L

/* Size of the zero-filled buffer for the firmware RP workaround.
 * The firmware reads this address from AV_SCRATCH_L and uses it to
 * prevent the VLD read pointer from advancing into uninitialized memory
 * at end-of-stream.  Must be at least 4KB per BSP vmjpeg_multi.c.
 */
#define RP_WORKAROUND_SIZE	SZ_4K

struct codec_mjpeg_priv {
	void *rp_buf;
	dma_addr_t rp_paddr;
};

#define PICINFO_BUF_IDX_MASK	GENMASK(2, 0)

/*
 * MJPEG DECODE_PARAM register format:
 * Bits [15:4] = picture height (height << 4)
 * Bit  [15]   = 0x8000 flag: signals firmware to use PSCALE output path
 */
#define MJPEG_DECODE_PARAM_PSCALE	BIT(15)

/*
 * MJPEG firmware AV_SCRATCH_0: max decode buffers.
 * BSP vmjpeg.c sets this to 12 for multi-instance mode.
 */
#define MJPEG_MAX_DECODE_BUFS		12

/*
 * MJPEG firmware AV_SCRATCH_1: firmware configuration word.
 * 0x031a = BSP default from vmjpeg.c.  Bit fields:
 *   [9]   = enable picture finish interrupt
 *   [8]   = enable error interrupt
 *   [4:3] = output format (01 = YUV420)
 *   [1]   = enable multi-instance mode
 */
#define MJPEG_FW_CONFIG			0x031a

/*
 * MCPU_INTR_MSK: enable all firmware interrupt sources.
 * BSP sets 0xffff to unmask all 16 interrupt bits.
 */
#define MCPU_INTR_MSK_ALL		0xffff

/*
 * VDEC_ASSIST_AMR1_INT8: VLD interrupt routing to AMR1 line 8.
 * BSP vmjpeg.c sets this to 8 to route VLD completion to MBOX1.
 */
#define MJPEG_AMR1_INT8_VLD		8

/*
 * PSCALE initial info: 8 taps, start phase = 0.
 * Written to PSCALE_BMEM_DAT as a pair: {0x0008, 0x60000000}.
 *   0x0008      = 8-tap filter
 *   0x60000000  = initial phase (BSP vpp_pscaler.c default)
 */
#define PSCALE_INIT_TAPS		0x0008
#define PSCALE_INIT_PHASE		0x60000000

/*
 * PSCALE 1:1 phase step in 16.16 fixed-point: 0x10000 = 1.0.
 * Used for both horizontal and vertical Y/C scaling.
 */
#define PSCALE_PHASE_STEP_1_1		0x10000

/*
 * PSCALE BMEM addresses for scaler parameters.
 * Each parameter occupies 2 BMEM words (address * 2).
 *   36 = Y horizontal phase step
 *   37 = Y horizontal initial info
 *   38 = Y vertical phase step
 *   39 = Y vertical initial info
 *   40 = C horizontal phase step
 *   41 = C horizontal initial info
 *   42 = C vertical phase step
 *   43 = C vertical initial info
 */
#define PSCALE_BMEM_Y_HPHASE		36
#define PSCALE_BMEM_Y_HINIT		37
#define PSCALE_BMEM_Y_VPHASE		38
#define PSCALE_BMEM_Y_VINIT		39
#define PSCALE_BMEM_C_HPHASE		40
#define PSCALE_BMEM_C_HINIT		41
#define PSCALE_BMEM_C_VPHASE		42
#define PSCALE_BMEM_C_VINIT		43

/*
 * The MJPEG firmware requires the PSCALE block to be initialized with filter
 * coefficients for proper YUV conversion. Without this, decoded frames cannot
 * be written to the output buffers.
 */
static void codec_mjpeg_init_scaler(struct amvdec_core *core)
{
	static const u32 filt_coef[] = {
		0x20402000, 0x20402000, 0x1f3f2101, 0x1f3f2101,
		0x1e3e2202, 0x1e3e2202, 0x1d3d2303, 0x1d3d2303,
		0x1c3c2404, 0x1c3c2404, 0x1b3b2505, 0x1b3b2505,
		0x1a3a2606, 0x1a3a2606, 0x19392707, 0x19392707,
		0x18382808, 0x18382808, 0x17372909, 0x17372909,
		0x16362a0a, 0x16362a0a, 0x15352b0b, 0x15352b0b,
		0x14342c0c, 0x14342c0c, 0x13332d0d, 0x13332d0d,
		0x12322e0e, 0x12322e0e, 0x11312f0f, 0x11312f0f,
		0x10303010
	};
	int i;

	/* Enable PSCALE and CBUS BMEM access */
	amvdec_write_dos(core, PSCALE_CTRL, PSCALE_CTRL_EN_BMEM);

	/* Write filter coefficients starting at BMEM address 0 */
	amvdec_write_dos(core, PSCALE_BMEM_ADDR, 0);
	for (i = 0; i < ARRAY_SIZE(filt_coef); i++) {
		amvdec_write_dos(core, PSCALE_BMEM_DAT, 0);
		amvdec_write_dos(core, PSCALE_BMEM_DAT, filt_coef[i]);
	}

	/* Y horizontal initial info */
	amvdec_write_dos(core, PSCALE_BMEM_ADDR, PSCALE_BMEM_Y_HINIT * 2);
	amvdec_write_dos(core, PSCALE_BMEM_DAT, PSCALE_INIT_TAPS);
	amvdec_write_dos(core, PSCALE_BMEM_DAT, PSCALE_INIT_PHASE);

	/* C horizontal initial info */
	amvdec_write_dos(core, PSCALE_BMEM_ADDR, PSCALE_BMEM_C_HINIT * 2);
	amvdec_write_dos(core, PSCALE_BMEM_DAT, PSCALE_INIT_TAPS);
	amvdec_write_dos(core, PSCALE_BMEM_DAT, PSCALE_INIT_PHASE);

	/* Y vertical initial info */
	amvdec_write_dos(core, PSCALE_BMEM_ADDR, PSCALE_BMEM_Y_VINIT * 2);
	amvdec_write_dos(core, PSCALE_BMEM_DAT, PSCALE_INIT_TAPS);
	amvdec_write_dos(core, PSCALE_BMEM_DAT, PSCALE_INIT_PHASE);

	/* C vertical initial info */
	amvdec_write_dos(core, PSCALE_BMEM_ADDR, PSCALE_BMEM_C_VINIT * 2);
	amvdec_write_dos(core, PSCALE_BMEM_DAT, PSCALE_INIT_TAPS);
	amvdec_write_dos(core, PSCALE_BMEM_DAT, PSCALE_INIT_PHASE);

	/* Y horizontal phase step (1:1) */
	amvdec_write_dos(core, PSCALE_BMEM_ADDR,
			 PSCALE_BMEM_Y_HPHASE * 2 + 1);
	amvdec_write_dos(core, PSCALE_BMEM_DAT, PSCALE_PHASE_STEP_1_1);

	/* C horizontal phase step (1:1) */
	amvdec_write_dos(core, PSCALE_BMEM_ADDR,
			 PSCALE_BMEM_C_HPHASE * 2 + 1);
	amvdec_write_dos(core, PSCALE_BMEM_DAT, PSCALE_PHASE_STEP_1_1);

	/* Y vertical phase step (1:1) */
	amvdec_write_dos(core, PSCALE_BMEM_ADDR,
			 PSCALE_BMEM_Y_VPHASE * 2 + 1);
	amvdec_write_dos(core, PSCALE_BMEM_DAT, PSCALE_PHASE_STEP_1_1);

	/* C vertical phase step (1:1) */
	amvdec_write_dos(core, PSCALE_BMEM_ADDR,
			 PSCALE_BMEM_C_VPHASE * 2 + 1);
	amvdec_write_dos(core, PSCALE_BMEM_DAT, PSCALE_PHASE_STEP_1_1);

	/* Reset PSCALE */
	amvdec_write_dos(core, DOS_SW_RESET0, DOS_SW_RESET0_PSCALE);
	amvdec_write_dos(core, DOS_SW_RESET0, 0);

	amvdec_write_dos(core, PSCALE_RST, PSCALE_RST_ALL);
	amvdec_write_dos(core, PSCALE_RST, 0);
}

/*
 * JPEG marker definitions for SOF header parsing.
 * SOF0 (baseline) and SOF1 (extended sequential) contain the chroma
 * subsampling factors in the component specification section.
 */
#define JPEG_MARKER_SOI		0xffd8
#define JPEG_MARKER_SOF0	0xffc0
#define JPEG_MARKER_SOF1	0xffc1

/*
 * Gate the input JPEG by chroma subsampling.  The firmware decodes 4:2:0
 * (Y 2x2) and minimal 4:2:2 (Y 2x1) when Cb/Cr are 1x1; it traps on
 * 4:4:4/4:4:0 and has no chroma-Vi=2 path.  Reject unsupported forms early
 * (a silent 5s firmware stall on the trap path can hard-lock GXL).
 *
 * JPEG SOF structure (after 2-byte marker + 2-byte length):
 *   byte 0: precision (8 or 12)
 *   byte 1-2: height (big-endian)
 *   byte 3-4: width (big-endian)
 *   byte 5: number of components (1=grayscale, 3=YCbCr)
 *   For each component (3 bytes):
 *     byte 0: component ID
 *     byte 1: sampling factors (high nibble = H, low = V)
 *     byte 2: quantization table index
 *
 * So Y sampling is at sof_off+7, Cb sampling at sof_off+10.
 * 4:2:0: Y=0x22, Cb=Cr=0x11.  Minimal 4:2:2: Y=0x21, Cb=Cr=0x11.
 */
static int codec_mjpeg_pre_queue(struct amvdec_session *sess,
				 struct vb2_buffer *vb)
{
	u8 *data = vb2_plane_vaddr(vb, 0);
	u32 size = vb2_get_plane_payload(vb, 0);
	u32 i;

	if (!data || size < 4)
		return 0; /* let firmware handle it */

	/* Scan for SOF0 or SOF1 marker */
	for (i = 0; i + 1 < size; i++) {
		u16 marker;
		u32 sof_off, ncomp;
		u8 y_hv, cb_v;

		if (data[i] != 0xff)
			continue;
		marker = (data[i] << 8) | data[i + 1];
		if (marker != JPEG_MARKER_SOF0 && marker != JPEG_MARKER_SOF1)
			continue;

		/* Found SOF.  Parse component sampling factors. */
		sof_off = i + 4; /* skip marker(2) + length(2) */
		if (sof_off + 11 > size)
			break;

		ncomp = data[sof_off + 5];
		if (ncomp < 3) {
			dev_err(sess->core->dev_dec,
				"MJPEG: grayscale (%u components) not supported\n",
				ncomp);
			return -EINVAL;
		}

		/*
		 * Sampling factors: Y at sof_off+7, Cb at sof_off+10
		 * (high nibble = H, low = V).  The firmware decode dispatch
		 * branches on G&3 = (Hmax-1)[0] | ((Vmax-1)[0]<<1):
		 *   Y=0x22 (Hmax2,Vmax2) -> 0x0310  (4:2:0, 4Y+1Cb+1Cr)
		 *   Y=0x21 (Hmax2,Vmax1) -> 0x02f4  (minimal 4:2:2, 2Y+1Cb+1Cr)
		 * Both decode correctly when Cb/Cr are 1x1 (Cb_V==1): minimal
		 * 4:2:2 verified Y68/U49/V49 dB on stock fw (GXL, 2026-06-04).
		 *
		 * Reject everything else:
		 *  - Y=0x11/0x12 (4:4:4 / 4:4:0) hit the firmware's 0x02f2
		 *    infinite-loop trap (GXL hard-lock risk, observed
		 *    2026-04-26 on aml-s805x-ac fae185feddc4).
		 *  - Y=0x22 with Cb_V==2 is the ffmpeg "yuv422p" form
		 *    (Y 2x2 + chroma 1x2, 8-block MCU 4Y+2Cb+2Cr).  The
		 *    firmware has no chroma-Vi=2 path; 0x0310 reads only 6
		 *    blocks -> entropy desync from MCU 0 -> garbage (5 dB).
		 *  These two are the beyond-BSP firmware items (full-subsampling
		 *  -plan.md Parts B/C); reject at queue until the fw path lands.
		 */
		y_hv = data[sof_off + 7];
		cb_v = data[sof_off + 10] & 0xf;
		if (!((y_hv == 0x22 || y_hv == 0x21) && cb_v == 1)) {
			dev_err(sess->core->dev_dec,
				"MJPEG: unsupported subsampling (Y=%u:%u Cb_V=%u) rejected -- supported: 4:2:0 and minimal 4:2:2 (Y2x1)\n",
				y_hv >> 4, y_hv & 0xf, cb_v);
			return -EINVAL;
		}

		return 0;
	}

	return 0; /* no SOF found, let firmware handle it */
}

static int codec_mjpeg_can_recycle(struct amvdec_core *core)
{
	return !amvdec_read_dos(core, MREG_TO_AMRISC);
}

static void codec_mjpeg_recycle(struct amvdec_core *core, u32 buf_idx)
{
	amvdec_write_dos(core, MREG_TO_AMRISC, buf_idx + 1);
}

static int codec_mjpeg_start(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_mjpeg_priv *priv;
	int ret;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	sess->priv = priv;

	/* Reset IQIDCT and MC */
	amvdec_write_dos(core, DOS_SW_RESET0, DOS_SW_RESET0_MDEC);
	amvdec_write_dos(core, DOS_SW_RESET0, 0);

	/*
	 * On G12A+, clear IQIDCT_CONTROL to remove stale state from prior
	 * codec sessions (e.g. H.264 sets bit 16). Without this, PSNR
	 * degrades on successive decodes. Do not write on GXL -- the
	 * hardware reset value is correct there.
	 */
	if (core->platform->revision >= VDEC_REVISION_G12A)
		amvdec_write_dos(core, IQIDCT_CONTROL, 0);

	ret = amvdec_set_canvases(sess, (u32[]){ AV_SCRATCH_4, 0 },
					(u32[]){ 4, 0 });
	if (ret)
		goto err_free_priv;

	/* Firmware parameters: max decode buffers and settings */
	amvdec_write_dos(core, AV_SCRATCH_0, MJPEG_MAX_DECODE_BUFS);
	amvdec_write_dos(core, AV_SCRATCH_1, MJPEG_FW_CONFIG);

	codec_mjpeg_init_scaler(core);

	/* Enable all firmware interrupt sources */
	amvdec_write_dos(core, MCPU_INTR_MSK, MCPU_INTR_MSK_ALL);

	amvdec_write_dos(core, MREG_DECODE_PARAM,
			 (sess->height << 4) | MJPEG_DECODE_PARAM_PSCALE);
	amvdec_write_dos(core, MREG_TO_AMRISC, 0);
	/*
	 * Clear MREG_FROM_AMRISC before IRQ enable.  With early_irq_enable,
	 * MBOX1 is armed before MPSR=1, so any stale pending interrupt from
	 * a prior session fires immediately.  A zero value here ensures the
	 * ISR treats it as a no-op and returns without delivering a frame.
	 */
	amvdec_write_dos(core, MREG_FROM_AMRISC, 0);

	/*
	 * RP workaround: allocate a zero-filled buffer and register it with
	 * the firmware via AV_SCRATCH_L.  The firmware uses this buffer to
	 * prevent the VLD read pointer from advancing into uninitialized
	 * memory at end-of-stream, which would cause the last frame to be
	 * consumed during VLD read-ahead without being decoded.
	 *
	 * Without this, the VIFIFO is effectively empty (level ~97 bytes)
	 * when the firmware is triggered for the last frame, causing a
	 * consistent 1-frame loss at EOS on all platforms.
	 *
	 * Per BSP vmjpeg_multi.c lines 703-717 and 1081-1083.
	 */
	priv->rp_buf = dma_alloc_coherent(core->dev, RP_WORKAROUND_SIZE,
					  &priv->rp_paddr, GFP_KERNEL);
	if (priv->rp_buf)
		amvdec_write_dos(core, MREG_RP_WORKAROUND, priv->rp_paddr);

	/* Set VLD interrupt routing to AMR1 line 8 */
	amvdec_write_dos(core, VDEC_ASSIST_AMR1_INT8, MJPEG_AMR1_INT8_VLD);

	/*
	 * On G12A+, MDEC_PIC_DC_CTRL bit 31 is required for proper VDEC1
	 * decoder operation.  The vdec_1 start path clears it before
	 * codec start, so codecs that need it must set it explicitly.
	 * Without this, SM1 produces no output frames.
	 */
	if (core->platform->revision >= VDEC_REVISION_G12A)
		amvdec_write_dos_bits(core, MDEC_PIC_DC_CTRL,
				      MDEC_PIC_DC_CTRL_OLD_MODE);

	sess->keyframe_found = 1;

	return 0;

err_free_priv:
	kfree(priv);
	sess->priv = NULL;
	return ret;
}

/*
 * The MJPEG firmware is a multi-instance decoder that decodes one frame at a
 * time.  After init, it enters a busy-wait loop at address 0x0040 polling
 * MREG_TO_AMRISC (AV_SCRATCH_8) bits 0-2.  When non-zero, the firmware
 * treats the value as (buffer_index + 1) and decodes one JPEG frame from
 * the VIFIFO into that buffer.  After decode, it fires an IRQ and loops
 * back to wait for the next non-zero value.
 *
 * This post_start hook writes the initial trigger to kick off the first
 * frame decode.  Subsequent frames are triggered by codec_mjpeg_recycle()
 * which is called by the recycle thread after each decoded frame is
 * returned to userspace.
 */
static void codec_mjpeg_post_start(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;

	/* Trigger first frame decode: buffer index 0 -> write 1 */
	amvdec_write_dos(core, MREG_TO_AMRISC, 1);
}

static int codec_mjpeg_stop(struct amvdec_session *sess)
{
	struct codec_mjpeg_priv *priv = sess->priv;

	if (priv) {
		if (priv->rp_buf)
			dma_free_coherent(sess->core->dev, RP_WORKAROUND_SIZE,
					  priv->rp_buf, priv->rp_paddr);
		kfree(priv);
		sess->priv = NULL;
	}
	return 0;
}

static irqreturn_t codec_mjpeg_threaded_isr(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	u32 reg;
	u32 buffer_index;

	amvdec_write_dos(core, ASSIST_MBOX1_CLR_REG, 1);

	reg = amvdec_read_dos(core, MREG_FROM_AMRISC);
	if (!(reg & PICINFO_BUF_IDX_MASK))
		return IRQ_HANDLED;

	buffer_index = ((reg & PICINFO_BUF_IDX_MASK) - 1) & 3;

	/* Skip offset-based timestamp matching -- firmware offset doesn't
	 * match esparser write-pointer offsets reliably.
	 */
	amvdec_dst_buf_done_idx(sess, buffer_index, (u32)-1, V4L2_FIELD_NONE);

	amvdec_write_dos(core, MREG_FROM_AMRISC, 0);
	return IRQ_HANDLED;
}

static irqreturn_t codec_mjpeg_isr(struct amvdec_session *sess)
{
	return IRQ_WAKE_THREAD;
}

static void codec_mjpeg_drain(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	u32 reg, to_amrisc;
	u32 buffer_index;

	/* Check if firmware has a decoded frame we haven't delivered.
	 * MREG_FROM_AMRISC holds the last decoded buffer index + 1.
	 * If non-zero, the ISR hasn't fired for it yet (e.g., the
	 * firmware decoded it but the IRQ was masked during drain).
	 */
	reg = amvdec_read_dos(core, MREG_FROM_AMRISC);
	to_amrisc = amvdec_read_dos(core, MREG_TO_AMRISC);
	dev_dbg(core->dev_dec,
		"MJPEG drain: FROM=%u TO=%u esparser_queued=%d vififo_level=%u wp=0x%08x rp=0x%08x\n",
		reg, to_amrisc, atomic_read(&sess->esparser_queued_bufs),
		amvdec_read_dos(core, VLD_MEM_VIFIFO_LEVEL),
		amvdec_read_dos(core, VLD_MEM_VIFIFO_WP),
		amvdec_read_dos(core, VLD_MEM_VIFIFO_RP));
	if (reg & PICINFO_BUF_IDX_MASK) {
		buffer_index = ((reg & PICINFO_BUF_IDX_MASK) - 1) & 3;
		dev_dbg(core->dev_dec,
			"MJPEG drain: delivering pending frame (buf %u)\n",
			buffer_index);
		amvdec_dst_buf_done_idx(sess, buffer_index, (u32)-1,
					V4L2_FIELD_NONE);
		amvdec_write_dos(core, MREG_FROM_AMRISC, 0);
	}

	/*
	 * Push VIFIFO padding to prevent VLD read-ahead starvation.
	 *
	 * The VLD hardware prefetches data from the VIFIFO into its internal
	 * buffer, advancing RP past the last JPEG frame's data.  When the
	 * firmware is triggered for the last frame, the VIFIFO appears empty
	 * (~97 bytes) even though the VLD consumed the data into its internal
	 * buffer without decoding it.
	 *
	 * Pushing 16KB of zeros extends the VIFIFO past the read-ahead window,
	 * ensuring the firmware can decode the last frame before RP reaches WP.
	 * The zeros contain no SOI markers, so no spurious output is produced.
	 *
	 * Combined with the AV_SCRATCH_L RP workaround buffer (set in start()),
	 * this ensures reliable last-frame delivery on all platforms.
	 */
	if (atomic_read(&sess->esparser_queued_bufs) > 0) {
		u8 *padding;

		padding = kzalloc(SZ_16K, GFP_KERNEL);
		if (padding) {
			esparser_queue_eos(core, padding, SZ_16K);
			kfree(padding);
		}
	}

	/*
	 * Force-zero safety net.  MJPEG is a single-frame codec: no DPB, no
	 * reorder buffer, one independent JPEG frame per decode trigger.  The
	 * pending frame (if any) was already delivered above, so nothing is in
	 * flight that could still be lost.  If an ISR was missed (the firmware
	 * decoded a frame but the MBOX1 IRQ was masked/dropped during drain),
	 * esparser_queued_bufs can stay stuck > 0 and vdec_drain_work() then
	 * stalls for the full 5s post-drain timeout before giving up.  Closing
	 * the counter here cannot lose frames for a single-frame codec and
	 * removes that 5s stall.
	 */
	atomic_set(&sess->esparser_queued_bufs, 0);
}

struct amvdec_codec_ops codec_mjpeg_ops = {
	.start = codec_mjpeg_start,
	.post_start = codec_mjpeg_post_start,
	.stop = codec_mjpeg_stop,
	.drain = codec_mjpeg_drain,
	.isr = codec_mjpeg_isr,
	.threaded_isr = codec_mjpeg_threaded_isr,
	.pre_queue = codec_mjpeg_pre_queue,
	.can_recycle = codec_mjpeg_can_recycle,
	.recycle = codec_mjpeg_recycle,
	/*
	 * Enable MBOX1 before firmware start to capture the first-frame IRQ.
	 * MJPEG firmware does not fire spurious init IRQs (busy-waits for
	 * MREG_TO_AMRISC != 0 before decoding).  See vdec_1.c for details.
	 */
	.early_irq_enable = true,
};

static int __init codec_mjpeg_mod_init(void)
{
	return amvdec_register_codec(V4L2_PIX_FMT_MJPEG, &codec_mjpeg_ops);
}

static void __exit codec_mjpeg_mod_exit(void)
{
	amvdec_unregister_codec(V4L2_PIX_FMT_MJPEG, &codec_mjpeg_ops);
}

module_init(codec_mjpeg_mod_init);
module_exit(codec_mjpeg_mod_exit);
MODULE_DESCRIPTION("Meson VDEC MJPEG codec");
MODULE_LICENSE("GPL");
