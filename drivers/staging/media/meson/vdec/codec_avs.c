// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026
 *
 * AVS (Audio Video Standard, GB/T 20090.2) decoder, using VDEC_1 engine.
 * Based on BSP avs_multi/avs_multi.c and upstream codec_mpeg4.c pattern.
 *
 * AVS is the Chinese national video coding standard (Jizhun profile).
 * Hardware support: GXL, GXM, G12A, G12B, SM1 (all via VDEC_1 engine).
 * Max resolution: 1080p on all platforms.
 * Output format: NV12 (VDEC_1 canvas output).
 *
 * Firmware protocol:
 *   - AVS_BUFFERIN (AV_SCRATCH_8): host writes recycle bitmask
 *   - AVS_BUFFEROUT (AV_SCRATCH_9): firmware writes decoded buffer index
 *   - DECODE_STATUS (AV_SCRATCH_H): firmware writes decode status code
 *   - DECODE_MODE (AV_SCRATCH_6): host writes decode mode (single/multi)
 *   - MEM_OFFSET_REG (AV_SCRATCH_F): host writes workspace offset
 *
 * Status codes (DECODE_STATUS & 0xff):
 *   0x01 = PIC_DONE (frame decoded)
 *   0x02 = DECODE_BUF_EMPTY
 *   0x03 = SEARCH_BUF_EMPTY
 *   0x04 = SKIP_PIC_DONE
 */

#include <linux/module.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "codec_avs.h"
#include "dos_regs.h"
#include "esparser.h"
#include "hevc_regs.h"
#include "vdec_helpers.h"

#define SIZE_WORKSPACE		SZ_1M

/*
 * The AVS firmware addresses its workspace via hardcoded internal addresses
 * starting at RV_AI_BUFF_START_ADDR.  MEM_OFFSET_REG tells the firmware the
 * delta between internal addresses and the actual physical address.
 * BSP reference: avs_multi.c uses RV_AI_BUFF_START_ADDR = 0x01a00000.
 * Note: this is different from MPEG4's DCAC_BUFF_START_IP (0x02b00000).
 */
#define RV_AI_BUFF_START_ADDR	0x01a00000

/* Firmware register mapping (BSP avs_multi.c) */
#define AVS_PIC_RATIO		AV_SCRATCH_0
#define AVS_PIC_INFO		AV_SCRATCH_1
#define AVS_FRAME_RATE		AV_SCRATCH_3
#define AVS_SOS_COUNT		AV_SCRATCH_7
#define AVS_BUFFERIN		AV_SCRATCH_8
#define AVS_BUFFEROUT		AV_SCRATCH_9
#define AVS_REPEAT_COUNT	AV_SCRATCH_A
#define AVS_TIME_STAMP		AV_SCRATCH_B
#define AVS_OFFSET_REG		AV_SCRATCH_C
#define MEM_OFFSET_REG		AV_SCRATCH_F
#define AVS_ERROR_RECOVERY_MODE	AV_SCRATCH_G
#define DECODE_STATUS		AV_SCRATCH_H
#define DECODE_LMEM_BUF_ADR	AV_SCRATCH_I
#define DECODE_STOP_POS		AV_SCRATCH_J
#define DECODE_CFG		AV_SCRATCH_K

/* Decode status values from firmware (DECODE_STATUS & 0xff) */
#define DECODE_STATUS_PIC_DONE		0x01
#define DECODE_STATUS_DECODE_BUF_EMPTY	0x02
#define DECODE_STATUS_SEARCH_BUF_EMPTY	0x03
#define DECODE_STATUS_SKIP_PIC_DONE	0x04

/* Decode mode values (written to AV_SCRATCH_6 aka DECODE_MODE) */
#define DECODE_MODE_SINGLE		0x0
#define DECODE_MODE_MULTI_STREAMBASE	0x2

/*
 * BUFFEROUT encoding for the AVS firmware (BSP avs_multi.c:3901):
 *   buffer_index = ((reg & 0x7) + (((reg >> 8) & 0x3) << 3) - 1) & 0x1f
 *   picture_type = (reg >> 3) & 7
 *
 * Bits [2:0]  = buffer index low (1-based! subtract 1 for 0-based)
 * Bits [5:3]  = picture type (0=I, 1=P, 2=B)
 * Bits [9:8]  = buffer index high (for >8 buffers)
 */
#define AVS_BUFFEROUT_PIC_TYPE_SHIFT	3
#define AVS_BUFFEROUT_PIC_TYPE_MASK	0x7

#define AVS_PIC_TYPE_I		0
#define AVS_PIC_TYPE_P		1
#define AVS_PIC_TYPE_B		2

#define NUM_BUFS		8

/*
 * MDEC_PIC_DC_THRESH value from BSP avs_multi.c.
 * Same value as MPEG4 -- shared VDEC_1 MDEC block.
 */
#define MDEC_PIC_DC_THRESH_AVS		0x404038aa

struct codec_avs {
	void	  *workspace_vaddr;
	dma_addr_t workspace_paddr;
	bool	   ready;	/* true after source change + capture setup */

	/*
	 * The first ISR's BUFFEROUT is consumed by the source change
	 * path and not delivered.  Save it here so resume() can feed
	 * it through the normal ISR frame delivery logic.
	 */
	u32	   src_change_bufferout;

	/*
	 * Reference frame sliding window.
	 *
	 * AVS uses I/P/B frame types like MPEG-4.  The firmware writes
	 * reference frame canvas addresses to ANC0/ANC1_CANVAS_ADDR.
	 * The driver tracks the two most recent I/P frames for display
	 * reordering (same pattern as codec_mpeg4.c).
	 */
	int	   ref_buf_idx[2];
	int	   ref_count;
};

/*
 * On G12A+, VDEC_1 and HEVC share a combined power domain.
 * The HEVCD_IPP and HEVCD_MPP blocks must be initialized even for VDEC_1
 * codecs.  Without this, the hardware stalls silently.
 *
 * BSP reference: avs_multi.c hw_ctx_restore.
 */
static void codec_avs_init_hevc_combine(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct v4l2_m2m_buffer *buf;
	u32 buf_y_paddr;
	u32 buf_uv_paddr;
	u32 width = ALIGN(sess->width, 64);
	u32 output_size = width * ALIGN(sess->height, 32);
	int idx = 0;

	if (core->platform->revision < VDEC_REVISION_G12A)
		return;

	/* Reset and enable HEVCD_IPP block */
	amvdec_write_dos(core, HEVCD_IPP_TOP_CNTL, BIT(0));
	amvdec_write_dos(core, HEVCD_IPP_TOP_CNTL, BIT(1));

	/* Enable MCR and decompression */
	amvdec_write_dos(core, HEVCD_MPP_VDEC_MCR_CTL, BIT(4) | BIT(0));
	amvdec_write_dos(core, HEVCD_MPP_DECOMP_CTL1, BIT(31));

	/*
	 * HEVCD_MCR_FIXSIZE_CFG: BSP sets based on resolution.
	 * SD (<=720): BIT(15) | 768;  HD: BIT(15) | 1920.
	 * Use 1920 (HD) since AVS supports up to 1080p.
	 * BSP register 0x34a8, byte offset 0xd2a0.
	 */
	amvdec_write_dos(core, 0xd2a0, BIT(15) | 1920);

	/*
	 * MDEC_PIC_DC_CTRL bit 18: required for HEVC combine mode.
	 * BSP: SET_VREG_MASK(MDEC_PIC_DC_CTRL, 1 << 18)
	 */
	amvdec_write_dos_bits(core, MDEC_PIC_DC_CTRL, BIT(18));

	/* Disable HEVC core (VDEC_1 path only) */
	amvdec_write_dos(core, HEVC_CORE_ENABLE, 0);

	/* Populate ANC2AXI table with buffer physical addresses */
	amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_CONF_ADDR,
			 ANC2AXI_CONF_AUTO_INC);

	v4l2_m2m_for_each_dst_buf(sess->m2m_ctx, buf) {
		struct vb2_buffer *vb = &buf->vb.vb2_buf;

		if (vb->num_planes > 1) {
			buf_y_paddr = vb2_dma_contig_plane_dma_addr(vb, 0);
			buf_uv_paddr = vb2_dma_contig_plane_dma_addr(vb, 1);
		} else {
			buf_y_paddr = vb2_dma_contig_plane_dma_addr(vb, 0);
			buf_uv_paddr = buf_y_paddr + output_size;
		}

		amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_DATA,
				 buf_y_paddr >> 5);
		amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_DATA,
				 buf_uv_paddr >> 5);
		idx++;
	}

	for (; idx < 16; idx++) {
		amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_DATA,
				 core->scratch_paddr >> 5);
		amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_DATA,
				 core->scratch_paddr >> 5);
	}

	amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_CONF_ADDR, 1);

	amvdec_write_dos(core, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, 1);
	for (idx = 0; idx < 32; idx++)
		amvdec_write_dos(core, HEVCD_MPP_ANC_CANVAS_DATA_ADDR, 0);

	amvdec_write_dos(core, HEVCD_MCRCC_CTL1, 0x2);
	amvdec_write_dos(core, HEVCD_MCRCC_CTL1, 0xff0);

	/*
	 * HEVCD_IPP_AXIIF_CONFIG: BSP clears bit 12.
	 * Also set endian and linear mode (BIT(3) | 7) like codec_mpeg4.c.
	 */
	amvdec_write_dos(core, HEVCD_IPP_AXIIF_CONFIG, BIT(3) | 7);
	amvdec_clear_dos_bits(core, HEVCD_IPP_AXIIF_CONFIG, BIT(12));
}

static int codec_avs_start(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_avs *avs;
	int ret;

	avs = kzalloc(sizeof(*avs), GFP_KERNEL);
	if (!avs)
		return -ENOMEM;

	avs->workspace_vaddr = dma_alloc_coherent(core->dev, SIZE_WORKSPACE,
						   &avs->workspace_paddr,
						   GFP_KERNEL);
	if (!avs->workspace_vaddr) {
		dev_err(core->dev, "Failed to request AVS Workspace\n");
		ret = -ENOMEM;
		goto free_avs;
	}

	memset(avs->workspace_vaddr, 0, SIZE_WORKSPACE);

	/*
	 * NOTE: vdec_1_start() has already done:
	 *   - DOS_SW_RESET0 (MDEC + VLD reset)
	 *   - PSCALE_CTRL = 0
	 *   - VIFIFO power up and configuration
	 *   - Firmware load (IMEM DMA)
	 *   - Scratch canvas fill
	 * We only need codec-specific register setup here.
	 * MPSR=1 and MBOX1 enable happen AFTER this function returns.
	 */

	/*
	 * Workspace offset: firmware maps internal address RV_AI_BUFF_START_ADDR
	 * to the physical address of the workspace allocation.
	 * BSP: hw->buf_offset = buf_start - RV_AI_BUFF_START_ADDR
	 */
	amvdec_write_dos(core, MEM_OFFSET_REG,
			 avs->workspace_paddr - RV_AI_BUFF_START_ADDR);

	/* Clear scratch registers */
	amvdec_write_dos(core, AVS_SOS_COUNT, 0);
	amvdec_write_dos(core, AVS_BUFFERIN, 0);
	amvdec_write_dos(core, AVS_BUFFEROUT, 0);
	amvdec_write_dos(core, AVS_REPEAT_COUNT, 0);
	amvdec_write_dos(core, AVS_TIME_STAMP, 0);
	amvdec_write_dos(core, AVS_OFFSET_REG, 0);
	amvdec_write_dos(core, AV_SCRATCH_D, 0);	/* debug register */

	/*
	 * AVS_ERROR_RECOVERY_MODE: BSP sets to 1 (no error recovery) by
	 * default.  Value 0 enables error recovery which can mask bitstream
	 * issues during bringup.
	 */
	amvdec_write_dos(core, AVS_ERROR_RECOVERY_MODE, 1);

	/* Clear decode status and stop position */
	amvdec_write_dos(core, DECODE_STATUS, 0);
	amvdec_write_dos(core, DECODE_STOP_POS, 0);

	/*
	 * VLD_STATUS_CTRL is cleared in post_start() after MPSR=1.
	 *
	 * Single-instance decode mode.
	 * DECODE_MODE_SINGLE (0) tells firmware to decode continuously
	 * without per-frame stop/restart.
	 */
	amvdec_write_dos(core, AV_SCRATCH_6, DECODE_MODE_SINGLE);

	/*
	 * NOTE: MDEC_PIC_DC_CTRL NV12/NV21 bits and PSCALE_CTRL are set by
	 * vdec_1_start() after this function returns.  Don't set them here.
	 */

	ret = amvdec_set_canvases(sess,
				  (u32[]){ AV_SCRATCH_0, AV_SCRATCH_G, 0 },
				  (u32[]){ 4, 4, 0 });
	if (ret)
		goto free_workspace;

	sess->priv = avs;
	codec_avs_init_hevc_combine(sess);

	avs->ref_buf_idx[0] = -1;
	avs->ref_buf_idx[1] = -1;
	avs->ref_count = 0;
	/*
	 * Start with ready=false so the first ISR emits SOURCE_CHANGE.
	 * This follows the standard V4L2 stateful decoder protocol:
	 * 1. Firmware decodes the first frame and fires MBOX1 ISR
	 * 2. ISR reads AVS_PIC_INFO for resolution, calls amvdec_src_change
	 * 3. Userspace receives SOURCE_CHANGE, allocates CAPTURE buffers
	 * 4. codec_avs_resume() sets ready=true
	 * 5. Subsequent ISRs deliver frames via amvdec_dst_buf_done_idx
	 */
	avs->ready = false;
	sess->keyframe_found = 1;

	return 0;

free_workspace:
	dma_free_coherent(core->dev, SIZE_WORKSPACE, avs->workspace_vaddr,
			  avs->workspace_paddr);
free_avs:
	kfree(avs);
	return ret;
}

static int codec_avs_stop(struct amvdec_session *sess)
{
	struct codec_avs *avs = sess->priv;
	struct amvdec_core *core = sess->core;

	if (avs->workspace_vaddr) {
		dma_free_coherent(core->dev, SIZE_WORKSPACE,
				  avs->workspace_vaddr,
				  avs->workspace_paddr);
		avs->workspace_vaddr = NULL;
	}

	return 0;
}

static int codec_avs_can_recycle(struct amvdec_core *core)
{
	return !amvdec_read_dos(core, AVS_BUFFERIN);
}

static void codec_avs_recycle(struct amvdec_core *core, u32 buf_idx)
{
	/*
	 * AVS firmware uses a bitmask-based recycle protocol (same as MPEG4).
	 * Write ~(1 << buf_idx) to signal that buffer buf_idx is available.
	 */
	amvdec_write_dos(core, AVS_BUFFERIN, ~(1 << buf_idx));
}

static irqreturn_t codec_avs_isr(struct amvdec_session *sess)
{
	return IRQ_WAKE_THREAD;
}

static irqreturn_t codec_avs_threaded_isr(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_avs *avs = sess->priv;
	u32 reg;
	int buffer_index;
	int pic_type;
	int display_idx;

	amvdec_write_dos(core, ASSIST_MBOX1_CLR_REG, 1);

	/*
	 * The single-instance AVS firmware signals frame completion by
	 * writing BUFFEROUT and firing MCPU_INTR_REQ.  Unlike the multi-
	 * instance firmware, it does NOT use DECODE_STATUS (AV_SCRATCH_H).
	 * Register dump confirmed: DECODE_STATUS=0 while BUFFEROUT=0x41.
	 *
	 * Read BUFFEROUT directly.  A non-zero value means a frame is ready.
	 */
	reg = amvdec_read_dos(core, AVS_BUFFEROUT);
	if (!reg) {
		u32 status = amvdec_read_dos(core, DECODE_STATUS);

		dev_dbg(core->dev,
			"AVS ISR: BUFFEROUT=0 DECODE_STATUS=0x%x\n", status);
		return IRQ_HANDLED;
	}

	dev_dbg(core->dev, "AVS ISR: BUFFEROUT=0x%x\n", reg);

	/*
	 * On the first IRQ with BUFFEROUT set, emit a source change event
	 * so userspace allocates capture buffers.  The firmware has parsed
	 * the sequence header and decoded the first frame -- we can read
	 * AVS_PIC_INFO for the resolution.
	 *
	 * Same pattern as codec_mpeg4.c: first ISR triggers source change,
	 * subsequent ISRs deliver frames.
	 */
	if (!avs->ready) {
		u32 pic_info = amvdec_read_dos(core, AVS_PIC_INFO);
		/*
		 * BSP avs_multi.c:783-784:
		 *   width  = READ_VREG(AVS_PIC_INFO) & 0x3fff
		 *   height = (READ_VREG(AVS_PIC_INFO) >> 14) & 0x3fff
		 *   interlace = (READ_VREG(AVS_PIC_INFO) >> 28) & 0x1
		 */
		u32 width = pic_info & 0x3fff;
		u32 height = (pic_info >> 14) & 0x3fff;

		/*
		 * The single-instance firmware may only set width in
		 * PIC_INFO (observed: PIC_INFO=0x2d0 for 720x576 content).
		 * Fall back to MC_PIC_W_H which the firmware programs
		 * from the sequence header during VLD parsing.
		 * MC_PIC_W_H: width in [31:16], height in [15:0] (pixels).
		 */
		if (!width || !height) {
			u32 mc_wh = amvdec_read_dos(core, 0x2420);

			dev_info(core->dev,
				 "AVS: PIC_INFO=0x%08x MC_PIC_W_H=0x%08x\n",
				 pic_info, mc_wh);
			if (!width && (mc_wh >> 16))
				width = mc_wh >> 16;
			if (!height && (mc_wh & 0xffff))
				height = mc_wh & 0xffff;
		}

		/* Last resort: use session defaults from S_FMT */
		if (!width)
			width = sess->width;
		if (!height)
			height = sess->height;

		if (width && height)
			amvdec_src_change(sess, width, height, NUM_BUFS);

		/*
		 * Save the BUFFEROUT value so resume() can replay this
		 * frame through the normal delivery path.  The firmware
		 * decoded a frame before capture buffers existed.
		 */
		avs->src_change_bufferout = reg;
		return IRQ_HANDLED;
	}

	/*
	 * Extract buffer index and picture type from BUFFEROUT.
	 * BSP formula: ((reg & 0x7) + (((reg >> 8) & 0x3) << 3) - 1) & 0x1f
	 * The firmware uses 1-based buffer indices (subtract 1 for 0-based).
	 * Bits [9:8] extend the index for >8 buffers.
	 */
	buffer_index = ((reg & 0x7) + (((reg >> 8) & 0x3) << 3) - 1) & 0x1f;
	pic_type = (reg >> AVS_BUFFEROUT_PIC_TYPE_SHIFT) &
		   AVS_BUFFEROUT_PIC_TYPE_MASK;

	if (buffer_index >= NUM_BUFS) {
		dev_dbg(core->dev, "AVS: invalid buffer index %d (reg=0x%x)\n",
			buffer_index, reg);
		amvdec_write_dos(core, AVS_BUFFEROUT, 0);
		return IRQ_HANDLED;
	}

	/*
	 * Reference frame sliding window (same as codec_mpeg4.c).
	 *
	 * I/P frames are reference frames.  We store the just-decoded
	 * frame and display the previous oldest reference.
	 * B frames are displayed immediately.
	 */
	if (pic_type == AVS_PIC_TYPE_I || pic_type == AVS_PIC_TYPE_P) {
		display_idx = avs->ref_buf_idx[0];
		avs->ref_buf_idx[0] = avs->ref_buf_idx[1];
		avs->ref_buf_idx[1] = buffer_index;
		if (avs->ref_count < 2)
			avs->ref_count++;

		if (display_idx >= 0)
			amvdec_dst_buf_done_idx(sess, display_idx,
						(u32)-1, V4L2_FIELD_NONE);
	} else {
		amvdec_dst_buf_done_idx(sess, buffer_index,
					(u32)-1, V4L2_FIELD_NONE);
	}

	amvdec_write_dos(core, AVS_BUFFEROUT, 0);
	return IRQ_HANDLED;
}

static void codec_avs_drain(struct amvdec_session *sess)
{
	struct codec_avs *avs = sess->priv;
	int i;

	/*
	 * At EOS, flush the reference frame sliding window.
	 * Deliver any remaining reference frames in order.
	 */
	for (i = 0; i < 2; i++) {
		if (avs->ref_buf_idx[i] >= 0) {
			amvdec_dst_buf_done_idx(sess, avs->ref_buf_idx[i],
						(u32)-1, V4L2_FIELD_NONE);
			avs->ref_buf_idx[i] = -1;
		}
	}
	avs->ref_count = 0;
}

static void codec_avs_resume(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_avs *avs = sess->priv;

	amvdec_set_canvases(sess,
			    (u32[]){ AV_SCRATCH_0, AV_SCRATCH_G, 0 },
			    (u32[]){ 4, 4, 0 });
	codec_avs_init_hevc_combine(sess);

	avs->ref_buf_idx[0] = -1;
	avs->ref_buf_idx[1] = -1;
	avs->ref_count = 0;
	avs->ready = true;

	/*
	 * Replay the first decoded frame that triggered SOURCE_CHANGE.
	 * That frame was decoded before capture buffers existed, so the
	 * ISR consumed BUFFEROUT for resolution detection without
	 * delivering the frame.  Now that capture buffers are ready,
	 * feed it through the normal reference frame sliding window.
	 */
	if (avs->src_change_bufferout) {
		u32 reg = avs->src_change_bufferout;
		int buffer_index = ((reg & 0x7) +
				    (((reg >> 8) & 0x3) << 3) - 1) & 0x1f;
		int pic_type = (reg >> AVS_BUFFEROUT_PIC_TYPE_SHIFT) &
			       AVS_BUFFEROUT_PIC_TYPE_MASK;

		if (buffer_index < NUM_BUFS) {
			if (pic_type == AVS_PIC_TYPE_I ||
			    pic_type == AVS_PIC_TYPE_P) {
				avs->ref_buf_idx[1] = buffer_index;
				avs->ref_count = 1;
			} else {
				amvdec_dst_buf_done_idx(sess, buffer_index,
							(u32)-1,
							V4L2_FIELD_NONE);
			}
		}
		avs->src_change_bufferout = 0;
	}

	/*
	 * Clear BUFFEROUT so the firmware can proceed to decode the
	 * next frame.  The first ISR left BUFFEROUT set (it was consumed
	 * by the source change path without clearing), and the firmware
	 * won't decode another frame until the host clears it.
	 */
	amvdec_write_dos(core, AVS_BUFFEROUT, 0);
}

/*
 * post_start: called after MPSR=1 and MBOX1 enable.
 *
 * Clear VLD_STATUS_CTRL so the firmware's main loop sees VLD idle and
 * proceeds to decode.  Must happen AFTER MPSR=1 because the firmware's
 * init sequence (address 0x0008) writes VLD_STATUS_CTRL=2, and the
 * esparser may set status bits during data write.
 */
static void codec_avs_post_start(struct amvdec_session *sess)
{
	/*
	 * VLD_STATUS_CTRL busy check is patched out in firmware (NOP at 0x07dc).
	 * No post_start register writes needed currently.
	 * Keep hook for future use (e.g., triggering first frame decode).
	 */
	struct amvdec_core *core = sess->core;

	amvdec_write_dos(core, VLD_STATUS_CTRL, 0);
}

struct amvdec_codec_ops codec_avs_ops = {
	.start = codec_avs_start,
	.post_start = codec_avs_post_start,
	.stop = codec_avs_stop,
	.drain = codec_avs_drain,
	.resume = codec_avs_resume,
	.isr = codec_avs_isr,
	.threaded_isr = codec_avs_threaded_isr,
	.can_recycle = codec_avs_can_recycle,
	.recycle = codec_avs_recycle,
	/*
	 * Do NOT use early_irq_enable: the AVS ISR fires before capture
	 * buffers are ready, causing a NULL pointer crash in the recycle
	 * thread (pending_frames list).  Use the normal post-MPSR enable
	 * sequence.  The post_start hook clears VLD_STATUS_CTRL to unblock
	 * the firmware's main loop.
	 */
};

static int __init codec_avs_mod_init(void)
{
	return amvdec_register_codec(V4L2_PIX_FMT_AVS, &codec_avs_ops);
}

static void __exit codec_avs_mod_exit(void)
{
	amvdec_unregister_codec(V4L2_PIX_FMT_AVS, &codec_avs_ops);
}

module_init(codec_avs_mod_init);
module_exit(codec_avs_mod_exit);
MODULE_DESCRIPTION("Meson VDEC AVS codec");
MODULE_LICENSE("GPL");
