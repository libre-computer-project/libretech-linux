// SPDX-License-Identifier: GPL-2.0+
/*
 * Amlogic Meson HCODEC H.264 Encoder - V4L2 M2M Interface
 *
 * Integrates into the meson-vdec platform driver, sharing the DOS
 * MMIO region and AO regmap. Creates a separate /dev/videoX node
 * for encoding.
 *
 * Copyright (C) 2024 LibreTech
 */

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/log2.h>
#include <linux/sizes.h>
#include <linux/platform_device.h>
#include <media/media-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "vdec.h"
#include "vdec_helpers.h"
#include "hcodec.h"
#include "hcodec_regs.h"
#include "jpegenc.h"

/* Vendor-specific compound control IDs for JPEG quantization table upload */
#define V4L2_CID_MESON_JPEG_LUMA_QUANT    (V4L2_CID_USER_BASE + 0x1000)
#define V4L2_CID_MESON_JPEG_CHROMA_QUANT  (V4L2_CID_USER_BASE + 0x1001)

/* Noise reduction mode: 0=off, 1=SNR only, 2=TNR only, 3=3DNR (SNR+TNR) */
static int nr_mode = 3; /* 3DNR: +3-5 dB PSNR improvement (measured 2026-04-06) */
module_param(nr_mode, int, 0644);
MODULE_PARM_DESC(nr_mode, "Encoder noise reduction: 0=off 1=SNR 2=TNR 3=3DNR (default=3)");

/* ME/IE weight tuning: higher = prefer that MB type; 0 = use defaults */
static int me_weight;
module_param(me_weight, int, 0644);
MODULE_PARM_DESC(me_weight, "ME weight offset (default 0x520, 0=default)");

static int i4mb_weight;
module_param(i4mb_weight, int, 0644);
MODULE_PARM_DESC(i4mb_weight, "I4MB weight offset (default 0x655, 0=default)");

static int i16mb_weight;
module_param(i16mb_weight, int, 0644);
MODULE_PARM_DESC(i16mb_weight, "I16MB weight offset (default 0x560, 0=default)");

/*
 * Default quant tables: each byte = HCODEC_DEFAULT_QP (26 = 0x1a).
 * The BSP fills these with memset(tbl, quant, sizeof(tbl)).
 * Must match HCODEC_DEFAULT_QP so the QDCT quantization parameters
 * agree with QPPICTURE and the VLC quant registers.
 */
static const u32 default_quant_tbl[8] = {
	0x1a1a1a1a, 0x1a1a1a1a, 0x1a1a1a1a, 0x1a1a1a1a,
	0x1a1a1a1a, 0x1a1a1a1a, 0x1a1a1a1a, 0x1a1a1a1a,
};

static struct amvenc_ctx *file_to_ctx(struct file *file)
{
	return container_of(file_to_v4l2_fh(file), struct amvenc_ctx, fh);
}

static void amvenc_set_default_params(struct amvenc_ctx *ctx)
{
	ctx->width = 640;
	ctx->height = 480;
	ctx->gop_size = 12;
	ctx->qp = HCODEC_DEFAULT_QP;
	ctx->h264_level = V4L2_MPEG_VIDEO_H264_LEVEL_4_0;
	ctx->force_keyframe = false;
	ctx->frame_number = 0;
	ctx->idr_pic_id = 0;
	ctx->fps = 30;
	ctx->slice_max_mb = 0;
	ctx->pic_order_cnt_lsb = 0;
	ctx->frames_since_idr = 0;
	ctx->sequence_out = 0;
	ctx->sequence_cap = 0;
	ctx->jpeg_quality = JPEGENC_DEFAULT_QUALITY;
	ctx->jpeg_chroma = V4L2_JPEG_CHROMA_SUBSAMPLING_420;

	memcpy(ctx->quant_tbl_i4, default_quant_tbl, sizeof(default_quant_tbl));
	memcpy(ctx->quant_tbl_i16, default_quant_tbl, sizeof(default_quant_tbl));
	memcpy(ctx->quant_tbl_me, default_quant_tbl, sizeof(default_quant_tbl));

	ctx->src_fmt.width = ctx->width;
	ctx->src_fmt.height = ctx->height;
	ctx->src_fmt.pixelformat = V4L2_PIX_FMT_NV12M;
	ctx->src_fmt.num_planes = 2;
	ctx->src_fmt.plane_fmt[0].sizeimage =
		ALIGN(ctx->width, 32) * ctx->height;
	ctx->src_fmt.plane_fmt[0].bytesperline = ALIGN(ctx->width, 32);
	ctx->src_fmt.plane_fmt[1].sizeimage =
		ALIGN(ctx->width, 32) * ctx->height / 2;
	ctx->src_fmt.plane_fmt[1].bytesperline = ALIGN(ctx->width, 32);
	ctx->src_fmt.colorspace = V4L2_COLORSPACE_SRGB;
	ctx->src_fmt.xfer_func = V4L2_XFER_FUNC_SRGB;
	ctx->src_fmt.ycbcr_enc = V4L2_YCBCR_ENC_601;
	ctx->src_fmt.quantization = V4L2_QUANTIZATION_LIM_RANGE;
	ctx->src_fmt.field = V4L2_FIELD_NONE;

	ctx->dst_fmt.width = ctx->width;
	ctx->dst_fmt.height = ctx->height;
	ctx->dst_fmt.pixelformat = V4L2_PIX_FMT_H264;
	ctx->dst_fmt.num_planes = 1;
	/* Layout and sizeimage will be set after hcodec_compute_buf_layout() below */
	ctx->dst_fmt.plane_fmt[0].sizeimage = SZ_1M;
	ctx->dst_fmt.plane_fmt[0].bytesperline = 0;
	ctx->dst_fmt.colorspace = V4L2_COLORSPACE_SRGB;
	ctx->dst_fmt.xfer_func = V4L2_XFER_FUNC_SRGB;
	ctx->dst_fmt.ycbcr_enc = V4L2_YCBCR_ENC_601;
	ctx->dst_fmt.quantization = V4L2_QUANTIZATION_LIM_RANGE;
	ctx->dst_fmt.field = V4L2_FIELD_NONE;

	/* Initialize buffer layout for default resolution (default = H.264) */
	hcodec_compute_buf_layout(ctx->width, ctx->height, false,
				  &ctx->buf_layout);
	/* no allocation yet -- sub_alloc_size[] zeroed by kzalloc */
	/* Update dst_fmt sizeimage for default layout */
	ctx->dst_fmt.plane_fmt[0].sizeimage = ctx->buf_layout.bs_size;
}

/* V4L2 control operations */
static int amvenc_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct amvenc_ctx *ctx =
		container_of(ctrl->handler, struct amvenc_ctx, ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_MPEG_VIDEO_GOP_SIZE:
	case V4L2_CID_MPEG_VIDEO_H264_I_PERIOD:
		/*
		 * The generic GOP_SIZE (what ffmpeg's -g and most V4L2
		 * userspace set) and the H.264-specific I_PERIOD both select
		 * the firmware I-frame period. Map both to gop_size so standard
		 * tooling -- which drives GOP_SIZE, not I_PERIOD -- works;
		 * otherwise "ffmpeg -g 1" is silently ignored and produces a
		 * default-GOP stream instead of all-intra.
		 */
		ctx->gop_size = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_MIN_QP:
		ctx->qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME:
		ctx->force_keyframe = true;
		break;
	case V4L2_CID_MPEG_VIDEO_BITRATE_MODE:
		ctx->bitrate_mode = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_BITRATE:
		ctx->bitrate = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE:
		ctx->rc_enable = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_MAX_QP:
		ctx->max_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_PROFILE:
		/* Read-only, accept silently */
		break;
	case V4L2_CID_MPEG_VIDEO_H264_LEVEL:
		ctx->h264_level = ctrl->val;
		break;
	case V4L2_CID_JPEG_COMPRESSION_QUALITY:
		ctx->jpeg_quality = ctrl->val;
		break;
	case V4L2_CID_JPEG_CHROMA_SUBSAMPLING:
		ctx->jpeg_chroma = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_MB:
		ctx->slice_max_mb = ctrl->val;
		break;
	case V4L2_CID_MESON_JPEG_LUMA_QUANT:
		memcpy(ctx->jpeg_luma_quant_user, ctrl->p_new.p_u8, 64);
		break;
	case V4L2_CID_MESON_JPEG_CHROMA_QUANT:
		memcpy(ctx->jpeg_chroma_quant_user, ctrl->p_new.p_u8, 64);
		break;
	}
	return 0;
}

static const struct v4l2_ctrl_ops amvenc_ctrl_ops = {
	.s_ctrl = amvenc_s_ctrl,
};

static const struct v4l2_ctrl_config meson_jpeg_luma_quant_cfg = {
	.ops  = &amvenc_ctrl_ops,
	.id   = V4L2_CID_MESON_JPEG_LUMA_QUANT,
	.name = "JPEG Luma Quantization Table",
	.type = V4L2_CTRL_TYPE_U8,
	.min  = 1,
	.max  = 255,
	.step = 1,
	.def  = 1,
	.dims = { 64 },
};

static const struct v4l2_ctrl_config meson_jpeg_chroma_quant_cfg = {
	.ops  = &amvenc_ctrl_ops,
	.id   = V4L2_CID_MESON_JPEG_CHROMA_QUANT,
	.name = "JPEG Chroma Quantization Table",
	.type = V4L2_CTRL_TYPE_U8,
	.min  = 1,
	.max  = 255,
	.step = 1,
	.def  = 1,
	.dims = { 64 },
};

/* Queue init (forward declaration) */
static int amvenc_queue_init(void *priv, struct vb2_queue *src_vq,
			     struct vb2_queue *dst_vq);

/* V4L2 file operations */
static int amvenc_open(struct file *file)
{
	struct amvdec_core *core = video_drvdata(file);
	struct amvenc_ctx *ctx;
	int ret;

	/*
	 * HCODEC is single-instance hardware -- reject concurrent opens.
	 * Two simultaneous encode sessions corrupt shared hardware registers
	 * and cause hard kernel lockups (KI-028).
	 *
	 * amvenc_release() clears enc_in_use only at the very end, after the
	 * v4l2_m2m / canvas / firmware / DMA teardown completes.  A rapid
	 * encoder re-open can therefore race the prior session's still-running
	 * release, and concurrent-decode CPU load on slow SoCs (S805X) widens
	 * that window -> spurious -EBUSY (BUG-CONC-MODEC-GXL).  Wait briefly for
	 * the transient release to finish; a genuine second concurrent encoder
	 * persists past the timeout and still gets -EBUSY, preserving KI-028.
	 */
	if (atomic_cmpxchg(&core->enc_in_use, 0, 1) != 0) {
		if (wait_event_interruptible_timeout(core->enc_wq,
				atomic_cmpxchg(&core->enc_in_use, 0, 1) == 0,
				msecs_to_jiffies(200)) <= 0)
			return -EBUSY;
	}

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->core = core;
	v4l2_fh_init(&ctx->fh, core->vdev_enc);

	ret = v4l2_ctrl_handler_init(&ctx->ctrl_handler, 17);
	if (ret)
		goto err_free;

	/* Generic GOP size (ffmpeg -g); aliased to I_PERIOD in s_ctrl. */
	v4l2_ctrl_new_std(&ctx->ctrl_handler, &amvenc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_GOP_SIZE,
			  1, 300, 1, 12);
	v4l2_ctrl_new_std(&ctx->ctrl_handler, &amvenc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_H264_I_PERIOD,
			  1, 300, 1, 12);
	v4l2_ctrl_new_std(&ctx->ctrl_handler, &amvenc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_H264_MIN_QP,
			  0, 51, 1, HCODEC_DEFAULT_QP);
	v4l2_ctrl_new_std(&ctx->ctrl_handler, &amvenc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME,
			  0, 0, 0, 0);
	v4l2_ctrl_new_std_menu(&ctx->ctrl_handler, &amvenc_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_BITRATE_MODE,
			       V4L2_MPEG_VIDEO_BITRATE_MODE_CBR,
			       0, V4L2_MPEG_VIDEO_BITRATE_MODE_VBR);
	v4l2_ctrl_new_std(&ctx->ctrl_handler, &amvenc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_BITRATE,
			  0, 20000000, 1, 0);
	v4l2_ctrl_new_std(&ctx->ctrl_handler, &amvenc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE,
			  0, 1, 1, 0);
	v4l2_ctrl_new_std(&ctx->ctrl_handler, &amvenc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_H264_MAX_QP,
			  0, 51, 1, 51);
	v4l2_ctrl_new_std_menu(&ctx->ctrl_handler, &amvenc_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_H264_PROFILE,
			       V4L2_MPEG_VIDEO_H264_PROFILE_MAIN,
			       ~((1 << V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE) |
				 (1 << V4L2_MPEG_VIDEO_H264_PROFILE_MAIN)),
			       V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE);
	v4l2_ctrl_new_std_menu(&ctx->ctrl_handler, &amvenc_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_H264_LEVEL,
			       V4L2_MPEG_VIDEO_H264_LEVEL_5_1,
			       0,
			       V4L2_MPEG_VIDEO_H264_LEVEL_4_0);
	v4l2_ctrl_new_std(&ctx->ctrl_handler, NULL,
			  V4L2_CID_MIN_BUFFERS_FOR_OUTPUT,
			  1, 1, 1, 1);
	v4l2_ctrl_new_std(&ctx->ctrl_handler, &amvenc_ctrl_ops,
			  V4L2_CID_JPEG_COMPRESSION_QUALITY,
			  1, 100, 1, JPEGENC_DEFAULT_QUALITY);
	v4l2_ctrl_new_std_menu(&ctx->ctrl_handler, &amvenc_ctrl_ops,
			       V4L2_CID_JPEG_CHROMA_SUBSAMPLING,
			       V4L2_JPEG_CHROMA_SUBSAMPLING_420,
			       /* mask: only 444, 422, 420 are selectable */
			       ~((1 << V4L2_JPEG_CHROMA_SUBSAMPLING_444) |
				 (1 << V4L2_JPEG_CHROMA_SUBSAMPLING_422) |
				 (1 << V4L2_JPEG_CHROMA_SUBSAMPLING_420)),
			       V4L2_JPEG_CHROMA_SUBSAMPLING_420);
	/* Multi-slice: max MBs per slice (0 = single slice = full frame) */
	v4l2_ctrl_new_std(&ctx->ctrl_handler, &amvenc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_MB,
			  0, 524288, 1, 0); /* 524288 = 1024*512 = max MBs at 16368x8192 */

	/* Custom JPEG quantization tables (all-ones default = use standard scaled tables) */
	v4l2_ctrl_new_custom(&ctx->ctrl_handler, &meson_jpeg_luma_quant_cfg, NULL);
	v4l2_ctrl_new_custom(&ctx->ctrl_handler, &meson_jpeg_chroma_quant_cfg, NULL);

	if (ctx->ctrl_handler.error) {
		ret = ctx->ctrl_handler.error;
		goto err_ctrl;
	}

	ctx->fh.ctrl_handler = &ctx->ctrl_handler;

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(core->m2m_enc, ctx,
					     amvenc_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto err_ctrl;
	}

	amvenc_set_default_params(ctx);

	/*
	 * N104: publish the session and take the core reference in one step,
	 * exactly as vdec_open() does.  The unbound check closes the one
	 * window v4l2_open()'s videodev_lock check leaves -- a caller already
	 * past it when vdec_remove() ran -- and the reference is what keeps
	 * struct amvdec_core and m2m_enc alive for this fd's ->release.
	 */
	mutex_lock(&core->sessions_lock);
	if (core->unbound) {
		mutex_unlock(&core->sessions_lock);
		ret = -ENODEV;
		goto err_m2m_ctx;
	}
	vdec_core_get(core);
	core->enc_ctx = ctx;
	mutex_unlock(&core->sessions_lock);

	v4l2_fh_add(&ctx->fh, file);

	return 0;

err_m2m_ctx:
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
err_ctrl:
	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
err_free:
	kfree(ctx);
	atomic_set(&core->enc_in_use, 0);
	return ret;
}

/*
 * Free every allocated HCODEC working-set sub-buffer (KI-116 CMA-frag
 * fix: the working set is now N independent dma_alloc_coherent buffers).
 */
static void amvenc_free_sub_buffers(struct amvdec_core *core,
				    struct amvenc_ctx *ctx)
{
	unsigned int i;

	for (i = 0; i < HCODEC_SUB_COUNT; i++) {
		if (!ctx->sub_vaddr[i])
			continue;
		dma_free_coherent(core->dev, ctx->sub_alloc_size[i],
				  ctx->sub_vaddr[i], ctx->sub_paddr[i]);
		ctx->sub_vaddr[i] = NULL;
		ctx->sub_paddr[i] = 0;
		ctx->sub_alloc_size[i] = 0;
	}
}

static int amvenc_release(struct file *file)
{
	struct amvenc_ctx *ctx = file_to_ctx(file);
	struct amvdec_core *core = ctx->core;

	v4l2_fh_del(&ctx->fh, file);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_ctrl_handler_free(&ctx->ctrl_handler);

	if (ctx->canvas_num)
		hcodec_free_canvases(core, ctx);
	hcodec_free_cached_firmware(ctx);
	amvenc_free_sub_buffers(core, ctx);

	/* N104: unpublish before freeing, so amvenc_quiesce() cannot see it. */
	mutex_lock(&core->sessions_lock);
	if (core->enc_ctx == ctx)
		core->enc_ctx = NULL;
	mutex_unlock(&core->sessions_lock);

	kfree(ctx);
	atomic_set(&core->enc_in_use, 0);
	wake_up_interruptible(&core->enc_wq);	/* let a racing open() proceed */
	/* Last: this may be the reference that frees core. */
	vdec_core_put(core);
	return 0;
}

static const struct v4l2_file_operations amvenc_fops = {
	.owner = THIS_MODULE,
	.open = amvenc_open,
	.release = amvenc_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

/* V4L2 ioctl operations */
static int amvenc_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	strscpy(cap->driver, "meson-vdec", sizeof(cap->driver));
	strscpy(cap->card, "Meson HCODEC Encoder", sizeof(cap->card));
	strscpy(cap->bus_info, "platform:meson-vdec", sizeof(cap->bus_info));
	return 0;
}

static const u32 amvenc_src_pixfmts[] = {
	V4L2_PIX_FMT_NV12M,
	V4L2_PIX_FMT_NV12,
	V4L2_PIX_FMT_NV21M,
	V4L2_PIX_FMT_NV21,
	V4L2_PIX_FMT_YUV420M,
	V4L2_PIX_FMT_YUYV,
	V4L2_PIX_FMT_RGB24,
	V4L2_PIX_FMT_ABGR32,
};

static int amvenc_enum_fmt_out(struct file *file, void *priv,
			       struct v4l2_fmtdesc *f)
{
	if (f->index >= ARRAY_SIZE(amvenc_src_pixfmts))
		return -EINVAL;
	f->pixelformat = amvenc_src_pixfmts[f->index];
	return 0;
}

static const u32 amvenc_cap_pixfmts[] = {
	V4L2_PIX_FMT_H264,
	V4L2_PIX_FMT_JPEG,
};

static int amvenc_enum_fmt_cap(struct file *file, void *priv,
			       struct v4l2_fmtdesc *f)
{
	if (f->index >= ARRAY_SIZE(amvenc_cap_pixfmts))
		return -EINVAL;
	f->pixelformat = amvenc_cap_pixfmts[f->index];
	return 0;
}

static int amvenc_g_fmt_out(struct file *file, void *priv,
			    struct v4l2_format *f)
{
	struct amvenc_ctx *ctx = file_to_ctx(file);

	f->fmt.pix_mp = ctx->src_fmt;
	return 0;
}

static int amvenc_g_fmt_cap(struct file *file, void *priv,
			    struct v4l2_format *f)
{
	struct amvenc_ctx *ctx = file_to_ctx(file);

	f->fmt.pix_mp = ctx->dst_fmt;
	return 0;
}

/*
 * KI-124: shrink (*max_w, *max_h) so their product fits the probe-time
 * encoder CMA budget (core->enc_cma_max_pixels).  Preserve aspect by
 * scaling both dimensions by sqrt(budget/product); round down to 16 px
 * (H.264 MB / JPEG MCU alignment) so the clamped max is still a legal
 * encode size.  Operates in-place.  Zero budget = uncapped.  Same
 * mechanism as vdec_cma_clamp_hevc_engine()/_mjpeg() on the decode side.
 */
static void amvenc_cma_clamp(struct amvdec_core *core, u32 *max_w, u32 *max_h)
{
	u64 budget = core->enc_cma_max_pixels;
	u64 product;
	u64 scale_1000;

	if (!budget)
		return;

	product = (u64)*max_w * *max_h;
	if (product <= budget)
		return;

	scale_1000 = int_sqrt(product * 1000000 / budget);
	if (scale_1000 == 0)
		scale_1000 = 1;
	*max_w = round_down((u32)((u64)*max_w * 1000 / scale_1000), 16);
	*max_h = round_down((u32)((u64)*max_h * 1000 / scale_1000), 16);
	if (*max_w < HCODEC_MIN_WIDTH)
		*max_w = HCODEC_MIN_WIDTH;
	if (*max_h < HCODEC_MIN_HEIGHT)
		*max_h = HCODEC_MIN_HEIGHT;
}

static int amvenc_try_fmt_out(struct file *file, void *priv,
			      struct v4l2_format *f)
{
	struct amvenc_ctx *ctx = file_to_ctx(file);
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	u32 w, h, bpl;
	u32 max_w = HCODEC_MAX_WIDTH, max_h = HCODEC_MAX_HEIGHT;
	bool found = false;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(amvenc_src_pixfmts); i++) {
		if (pix->pixelformat == amvenc_src_pixfmts[i]) {
			found = true;
			break;
		}
	}
	if (!found)
		pix->pixelformat = V4L2_PIX_FMT_NV12M;

	/* KI-124: bound the upper limit to the CMA-feasible cap so the
	 * resulting OUTPUT-plane sizeimage (and internal DCT alloc) never
	 * exceeds what the board's CMA pool can give contiguously. */
	amvenc_cma_clamp(ctx->core, &max_w, &max_h);

	w = clamp(pix->width, (u32)HCODEC_MIN_WIDTH, max_w);
	h = clamp(pix->height, (u32)HCODEC_MIN_HEIGHT, max_h);
	w = ALIGN(w, 16);
	/*
	 * Height: keep the user-requested value (do NOT round to 16).
	 * The H.264 MB row size is 16 -- internally the encoder operates on
	 * `mb_h = (height + 15) / 16` rows and the SPS reports
	 * `pic_height_in_map_units_minus1 = mb_h - 1` with a
	 * `frame_cropping_flag` + `frame_crop_bottom_offset` covering the
	 * (canvas - display) pixel gap.  Preserving the user's 1080 here
	 * (instead of returning 1088) is the load-bearing piece for that
	 * SPS-side cropping to come out correct -- if the driver loses the
	 * 1080 -> 1088 distinction at try_fmt_out time, the SPS emission
	 * code at hcodec_build_sps_pps:1562 computes `crop_bottom = mb_h *
	 * 16 - ctx->height = 0` and emits a bare 1088-coded SPS that the
	 * VDEC HW H.264 decoder firmware then rejects at SOURCE_CHANGE
	 * (INTEROP-FOLLOWUP-HCODEC-VDEC-1080P-SPS, 2026-05-27).
	 *
	 * sizeimage is reported at the VISIBLE height (`bpl * h`), self-consistent
	 * with the format -- NOT canvas-aligned (`bpl * ALIGN(h,16)`). The input
	 * canvas is configured at src_height = ctx->height (e.g. 1080) and the
	 * HCODEC HW edge-replicates the partial bottom MB row, so the v4l2 input
	 * buffer only needs the visible height; the encoder still codes the
	 * MB-aligned 1088 internally and frame_cropping-back to 1080.
	 *
	 * Over-reporting sizeimage as the 1088 canvas size broke stock gst at a
	 * non-16-aligned height: gst derives the v4l2 buffer layout from the
	 * VISIBLE height (1080), so a 1088-sized NV12M buffer it modelled as 1080
	 * placed the chroma plane at the wrong offset, and an MMAP producer
	 * (videotestsrc/webcam -> v4l2h264enc/v4l2jpegenc) wrote out of bounds ->
	 * SIGSEGV. With sizeimage == bpl*h the format is consistent and raw/MMAP
	 * 1080p H.264 + JPEG encode on stock gst, no userspace patch.
	 */
	pix->width = w;
	pix->height = h;

	bpl = ALIGN(w, 32);

	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_NV12M:
	case V4L2_PIX_FMT_NV21M:
		pix->num_planes = 2;
		pix->plane_fmt[0].bytesperline = bpl;
		pix->plane_fmt[0].sizeimage = bpl * h;
		pix->plane_fmt[1].bytesperline = bpl;
		pix->plane_fmt[1].sizeimage = bpl * h / 2;
		break;
	case V4L2_PIX_FMT_YUV420M:
		/*
		 * MFDIN requires 64-byte aligned Y stride for planar
		 * YUV420 (iformat=4).  BSP uses ALIGN(w, 64) for this
		 * format while NV12/NV21 use ALIGN(w, 32).  Using 32
		 * causes chroma garbling for widths that are 32-aligned
		 * but not 64-aligned.
		 */
		bpl = ALIGN(w, 64);
		pix->num_planes = 3;
		pix->plane_fmt[0].bytesperline = bpl;
		pix->plane_fmt[0].sizeimage = bpl * h;
		pix->plane_fmt[1].bytesperline = bpl / 2;
		pix->plane_fmt[1].sizeimage = bpl / 2 * h / 2;
		pix->plane_fmt[2].bytesperline = bpl / 2;
		pix->plane_fmt[2].sizeimage = bpl / 2 * h / 2;
		break;
	case V4L2_PIX_FMT_YUYV:
		pix->num_planes = 1;
		pix->plane_fmt[0].bytesperline = bpl * 2;
		pix->plane_fmt[0].sizeimage = bpl * 2 * h;
		break;
	case V4L2_PIX_FMT_RGB24:
		/* RGB888: 3 bytes/pixel, MFDIN iformat=1 with R2Y */
		pix->num_planes = 1;
		pix->plane_fmt[0].bytesperline = bpl * 3;
		pix->plane_fmt[0].sizeimage = bpl * 3 * h;
		break;
	case V4L2_PIX_FMT_ABGR32:
		pix->num_planes = 1;
		pix->plane_fmt[0].bytesperline = bpl * 4;
		pix->plane_fmt[0].sizeimage = bpl * 4 * h;
		break;
	default: /* NV12, NV21 */
		pix->num_planes = 1;
		pix->plane_fmt[0].bytesperline = bpl;
		pix->plane_fmt[0].sizeimage = bpl * h * 3 / 2;
		break;
	}

	pix->field = V4L2_FIELD_NONE;
	if (!pix->colorspace)
		pix->colorspace = V4L2_COLORSPACE_SRGB;
	if (!pix->xfer_func)
		pix->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(pix->colorspace);
	if (!pix->ycbcr_enc)
		pix->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(pix->colorspace);
	if (!pix->quantization)
		pix->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(
			false, pix->colorspace, pix->ycbcr_enc);

	return 0;
}

static int amvenc_try_fmt_cap(struct file *file, void *priv,
			      struct v4l2_format *f)
{
	struct amvenc_ctx *ctx = file_to_ctx(file);
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;

	if (pix->pixelformat != V4L2_PIX_FMT_JPEG)
		pix->pixelformat = V4L2_PIX_FMT_H264;

	pix->width = ctx->src_fmt.width;
	pix->height = ctx->src_fmt.height;
	pix->num_planes = 1;
	pix->plane_fmt[0].bytesperline = 0;
	/*
	 * KI-126: clamp the capture (bitstream) plane to a realistic bound.
	 * bs_size is the resolution-scaled VLC ring the firmware writes the
	 * compressed bitstream into (zero-copy into the CAPTURE buffer); a
	 * compressed H.264/JPEG frame cannot exceed the raw frame.  Some V4L2
	 * encoders (notably GStreamer's v4l2 video encoder) request a fixed
	 * ~64 MB coded-buffer default -- honoring it via max() forced a single
	 * ~64 MB contiguous vb2 CMA alloc that WARNs in __alloc_pages on a
	 * fragmented small-CMA pool (the alloc falls back but taints, KI-126).
	 * Floor at bs_size, cap at the raw-frame size.
	 */
	{
		u32 raw_max = (u32)((u64)ALIGN(pix->width, 64) *
				    ALIGN(pix->height, 32) * 3 / 2);

		pix->plane_fmt[0].sizeimage =
			clamp_t(u32, pix->plane_fmt[0].sizeimage,
				ctx->buf_layout.bs_size,
				max(ctx->buf_layout.bs_size, raw_max));
	}
	pix->field = V4L2_FIELD_NONE;
	pix->colorspace = ctx->src_fmt.colorspace;
	pix->xfer_func = ctx->src_fmt.xfer_func;
	pix->ycbcr_enc = ctx->src_fmt.ycbcr_enc;
	if (pix->pixelformat == V4L2_PIX_FMT_JPEG)
		pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	else
		pix->quantization = ctx->src_fmt.quantization;

	return 0;
}

static int amvenc_s_fmt_out(struct file *file, void *priv,
			    struct v4l2_format *f)
{
	struct amvenc_ctx *ctx = file_to_ctx(file);
	struct hcodec_buf_layout new_layout;
	int ret;

	ret = amvenc_try_fmt_out(file, priv, f);
	if (ret)
		return ret;

	ctx->src_fmt = f->fmt.pix_mp;
	ctx->width = f->fmt.pix_mp.width;
	ctx->height = f->fmt.pix_mp.height;

	/*
	 * Compute new buffer layout for this resolution.  Use the current
	 * CAPTURE format to pick the JPEG (intra-only) vs H.264 layout;
	 * start_streaming recomputes authoritatively once the CAPTURE format
	 * is final, but doing it here keeps sub_size[] / sizeimage hints right.
	 */
	hcodec_compute_buf_layout(ctx->width, ctx->height,
				  ctx->dst_fmt.pixelformat == V4L2_PIX_FMT_JPEG,
				  &new_layout);

	/*
	 * Free any sub-buffer whose required size grew beyond its current
	 * allocation (or is no longer needed) so start_streaming re-allocates
	 * it at the new size.  Per-sub-buffer (KI-116 split allocation).
	 */
	{
		struct amvdec_core *core = ctx->core;
		unsigned int i;

		for (i = 0; i < HCODEC_SUB_COUNT; i++) {
			if (ctx->sub_vaddr[i] &&
			    new_layout.sub_size[i] > ctx->sub_alloc_size[i]) {
				dma_free_coherent(core->dev,
						  ctx->sub_alloc_size[i],
						  ctx->sub_vaddr[i],
						  ctx->sub_paddr[i]);
				ctx->sub_vaddr[i] = NULL;
				ctx->sub_paddr[i] = 0;
				ctx->sub_alloc_size[i] = 0;
			}
		}
	}
	ctx->buf_layout = new_layout;

	ctx->dst_fmt.width = ctx->width;
	ctx->dst_fmt.height = ctx->height;
	ctx->dst_fmt.colorspace = ctx->src_fmt.colorspace;
	ctx->dst_fmt.xfer_func = ctx->src_fmt.xfer_func;
	ctx->dst_fmt.ycbcr_enc = ctx->src_fmt.ycbcr_enc;
	if (ctx->dst_fmt.pixelformat == V4L2_PIX_FMT_JPEG)
		ctx->dst_fmt.quantization = V4L2_QUANTIZATION_FULL_RANGE;
	else
		ctx->dst_fmt.quantization = ctx->src_fmt.quantization;


	return 0;
}

static int amvenc_s_fmt_cap(struct file *file, void *priv,
			    struct v4l2_format *f)
{
	struct amvenc_ctx *ctx = file_to_ctx(file);
	int ret;

	ret = amvenc_try_fmt_cap(file, priv, f);
	if (ret)
		return ret;

	ctx->dst_fmt = f->fmt.pix_mp;
	return 0;
}

static int amvenc_enum_framesizes(struct file *file, void *priv,
				  struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index != 0)
		return -EINVAL;

	if (fsize->pixel_format != V4L2_PIX_FMT_NV12M &&
	    fsize->pixel_format != V4L2_PIX_FMT_NV12 &&
	    fsize->pixel_format != V4L2_PIX_FMT_NV21M &&
	    fsize->pixel_format != V4L2_PIX_FMT_NV21 &&
	    fsize->pixel_format != V4L2_PIX_FMT_YUV420M &&
	    fsize->pixel_format != V4L2_PIX_FMT_YUYV &&
	    fsize->pixel_format != V4L2_PIX_FMT_RGB24 &&
	    fsize->pixel_format != V4L2_PIX_FMT_ABGR32 &&
	    fsize->pixel_format != V4L2_PIX_FMT_H264 &&
	    fsize->pixel_format != V4L2_PIX_FMT_JPEG)
		return -EINVAL;

	{
		struct amvenc_ctx *ctx = file_to_ctx(file);
		u32 max_w = HCODEC_MAX_WIDTH, max_h = HCODEC_MAX_HEIGHT;

		/* KI-124: advertise the CMA-feasible cap, not the silicon
		 * max, so userspace negotiates a resolution REQBUFS can
		 * actually back with contiguous CMA on small-CMA boards. */
		amvenc_cma_clamp(ctx->core, &max_w, &max_h);

		fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
		fsize->stepwise.min_width = HCODEC_MIN_WIDTH;
		fsize->stepwise.max_width = max_w;
		fsize->stepwise.step_width = 16;
		fsize->stepwise.min_height = HCODEC_MIN_HEIGHT;
		fsize->stepwise.max_height = max_h;
		/*
		 * Height step is 8, not 16. Macroblocks (H.264) and MCUs (JPEG
		 * 4:2:0) are 16x16, so the CODED frame must be 16-aligned -- but
		 * the encoder pads/crops internally: try_fmt_out keeps the raw
		 * height, hcodec_setup_mfdin programs it, and
		 * hcodec_build_sps_pps emits frame_cropping (H.264).
		 * step_height=16 was the only thing rejecting an 8-aligned input
		 * (1080 = 135*8) during caps negotiation, blocking
		 * native-resolution 1080p encode. Width stays 16-aligned (the
		 * firmware's per-row MB counter is column-indexed, no crop
		 * equivalent).
		 */
		fsize->stepwise.step_height = 8;
	}

	return 0;
}

static int amvenc_g_selection(struct file *file, void *priv,
			      struct v4l2_selection *s)
{
	struct amvenc_ctx *ctx = file_to_ctx(file);

	if (s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT &&
	    s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return -EINVAL;

	switch (s->target) {
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		/* Full source frame */
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = ctx->src_fmt.width;
		s->r.height = ctx->src_fmt.height;
		break;
	case V4L2_SEL_TGT_CROP:
		/* Active/visible area (may differ from padded MB-aligned) */
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = ctx->width;
		s->r.height = ctx->height;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int amvenc_s_selection(struct file *file, void *priv,
			      struct v4l2_selection *s)
{
	struct amvenc_ctx *ctx = file_to_ctx(file);

	if (s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT &&
	    s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return -EINVAL;

	if (s->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	/* Constrain to source frame bounds, MB-aligned width */
	s->r.left = 0;
	s->r.top = 0;
	s->r.width = clamp(s->r.width, (u32)HCODEC_MIN_WIDTH,
			   ctx->src_fmt.width);
	s->r.height = clamp(s->r.height, (u32)HCODEC_MIN_HEIGHT,
			    ctx->src_fmt.height);
	s->r.width = ALIGN(s->r.width, 16);

	ctx->width = s->r.width;
	ctx->height = s->r.height;
	ctx->dst_fmt.width = ctx->width;
	ctx->dst_fmt.height = ctx->height;

	return 0;
}

static int amvenc_subscribe_event(struct v4l2_fh *fh,
				  const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 0, NULL);
	default:
		return v4l2_ctrl_subscribe_event(fh, sub);
	}
}

static int amvenc_s_parm(struct file *file, void *fh,
			 struct v4l2_streamparm *parm)
{
	struct amvenc_ctx *ctx = file_to_ctx(file);
	struct v4l2_fract *tf;

	if (parm->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return -EINVAL;

	tf = &parm->parm.output.timeperframe;
	if (tf->numerator && tf->denominator)
		ctx->fps = clamp_t(u32, tf->denominator / tf->numerator,
				   1, 120);
	else
		ctx->fps = 30;

	/* Echo back the effective value */
	memset(&parm->parm.output, 0, sizeof(parm->parm.output));
	parm->parm.output.capability = V4L2_CAP_TIMEPERFRAME;
	parm->parm.output.timeperframe.numerator = 1;
	parm->parm.output.timeperframe.denominator = ctx->fps;

	return 0;
}

static int amvenc_g_parm(struct file *file, void *fh,
			 struct v4l2_streamparm *parm)
{
	struct amvenc_ctx *ctx = file_to_ctx(file);

	if (parm->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return -EINVAL;

	memset(&parm->parm.output, 0, sizeof(parm->parm.output));
	parm->parm.output.capability = V4L2_CAP_TIMEPERFRAME;
	parm->parm.output.timeperframe.numerator = 1;
	parm->parm.output.timeperframe.denominator = ctx->fps;

	return 0;
}

static const struct v4l2_ioctl_ops amvenc_ioctl_ops = {
	.vidioc_querycap = amvenc_querycap,

	.vidioc_enum_fmt_vid_out = amvenc_enum_fmt_out,
	.vidioc_enum_fmt_vid_cap = amvenc_enum_fmt_cap,

	.vidioc_g_fmt_vid_out_mplane = amvenc_g_fmt_out,
	.vidioc_g_fmt_vid_cap_mplane = amvenc_g_fmt_cap,
	.vidioc_try_fmt_vid_out_mplane = amvenc_try_fmt_out,
	.vidioc_try_fmt_vid_cap_mplane = amvenc_try_fmt_cap,
	.vidioc_s_fmt_vid_out_mplane = amvenc_s_fmt_out,
	.vidioc_s_fmt_vid_cap_mplane = amvenc_s_fmt_cap,

	.vidioc_enum_framesizes = amvenc_enum_framesizes,

	.vidioc_s_parm = amvenc_s_parm,
	.vidioc_g_parm = amvenc_g_parm,

	.vidioc_g_selection = amvenc_g_selection,
	.vidioc_s_selection = amvenc_s_selection,

	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,

	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,

	.vidioc_encoder_cmd = v4l2_m2m_ioctl_encoder_cmd,
	.vidioc_try_encoder_cmd = v4l2_m2m_ioctl_try_encoder_cmd,

	.vidioc_subscribe_event = amvenc_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

/* VB2 queue operations */
static int amvenc_queue_setup(struct vb2_queue *q, unsigned int *num_buffers,
			      unsigned int *num_planes, unsigned int sizes[],
			      struct device *alloc_devs[])
{
	struct amvenc_ctx *ctx = vb2_get_drv_priv(q);
	struct v4l2_pix_format_mplane *fmt;
	unsigned int i;

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		fmt = &ctx->src_fmt;
	else
		fmt = &ctx->dst_fmt;

	if (*num_planes) {
		if (*num_planes != fmt->num_planes)
			return -EINVAL;
		for (i = 0; i < fmt->num_planes; i++)
			if (sizes[i] < fmt->plane_fmt[i].sizeimage)
				return -EINVAL;
		return 0;
	}

	*num_planes = fmt->num_planes;
	for (i = 0; i < fmt->num_planes; i++)
		sizes[i] = fmt->plane_fmt[i].sizeimage;

	return 0;
}

static int amvenc_buf_prepare(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct amvenc_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct v4l2_pix_format_mplane *fmt;
	unsigned int i;

	if (vb->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		fmt = &ctx->src_fmt;
	else
		fmt = &ctx->dst_fmt;

	vbuf->field = V4L2_FIELD_NONE;

	for (i = 0; i < fmt->num_planes; i++) {
		if (vb2_plane_size(vb, i) < fmt->plane_fmt[i].sizeimage)
			return -EINVAL;
		if (vb->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
			vb2_set_plane_payload(vb, i,
					      fmt->plane_fmt[i].sizeimage);
	}
	return 0;
}

static void amvenc_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct amvenc_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}

static int amvenc_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct amvenc_ctx *ctx = vb2_get_drv_priv(q);
	struct amvdec_core *core = ctx->core;
	int ret;

	/*
	 * N118: refuse a STREAMON that arrives after vdec_remove() has shut the
	 * door.  amvenc_open()'s unbound check is not enough -- open() and
	 * STREAMON are separate syscalls, so a session that opened while the
	 * device was live can still enter here mid-unbind, and the path below
	 * reaches DOS MMIO through hcodec_power_get() -> vdec_1_power_get()
	 * before any job exists (Oops 96000047, GXL, N107).
	 *
	 * This is the cheap first line and it is NOT the whole fix: the flag
	 * could flip one instruction later.  The refusal that actually closes
	 * the race is the one those two power_get() functions make under
	 * hw_lock, against a vdec_remove() power-off block that now takes the
	 * same lock.  This test just keeps the common case out of them.
	 */
	if (READ_ONCE(core->unbound)) {
		ret = -ENODEV;
		goto bufs_done;
	}

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		/*
		 * H.264 firmware has a 128 MB/row width cliff (7-bit
		 * column counter). Check here, not in try_fmt_out,
		 * because the CAPTURE format (H.264 vs JPEG) may not
		 * be set when TRY_FMT OUTPUT is called.
		 */
		if (ctx->dst_fmt.pixelformat != V4L2_PIX_FMT_JPEG &&
		    ctx->width > HCODEC_H264_MAX_WIDTH) {
			dev_err(core->dev,
				"HCODEC: H.264 width %u exceeds firmware max %u\n",
				ctx->width, HCODEC_H264_MAX_WIDTH);
			ret = -EINVAL;
			goto bufs_done;
		}

		/*
		 * Recompute the layout now that the CAPTURE format is final:
		 * JPEG (intra-only) omits the DCT + two DEC reference frames,
		 * shrinking the largest contiguous allocation from ~16 MB
		 * (H.264 @1080p) to ~1 MB (KI-116 CMA-fragmentation fix).
		 */
		hcodec_compute_buf_layout(ctx->width, ctx->height,
			ctx->dst_fmt.pixelformat == V4L2_PIX_FMT_JPEG,
			&ctx->buf_layout);

		/*
		 * Allocate each working-set sub-buffer with its OWN
		 * dma_alloc_coherent.  The engine registers take independent
		 * base addresses, so contiguity is only required per sub-buffer
		 * (~8 MB DCT worst case for H.264; ~1 MB for JPEG) rather than
		 * across the whole working set -- this survives a fragmented
		 * CMA pool that the old monolithic ~16 MB alloc could not.
		 */
		{
			unsigned int i;

			for (i = 0; i < HCODEC_SUB_COUNT; i++) {
				u32 sz = ctx->buf_layout.sub_size[i];

				if (!sz || ctx->sub_vaddr[i])
					continue;
				ctx->sub_vaddr[i] = dma_alloc_coherent(
					core->dev, sz, &ctx->sub_paddr[i],
					GFP_KERNEL);
				if (!ctx->sub_vaddr[i]) {
					dev_err(core->dev,
						"HCODEC: sub-buffer %u alloc (%u bytes) failed\n",
						i, sz);
					amvenc_free_sub_buffers(core, ctx);
					ret = -ENOMEM;
					goto bufs_done;
				}
				ctx->sub_alloc_size[i] = sz;
			}
		}

		ret = hcodec_power_get(core);
		if (ret)
			goto bufs_done;

		mutex_lock(&core->hw_lock);
		if (ctx->dst_fmt.pixelformat == V4L2_PIX_FMT_JPEG)
			ret = jpegenc_load_firmware(core);
		else
			ret = hcodec_load_firmware(core);
		if (ret) {
			mutex_unlock(&core->hw_lock);
			hcodec_power_put(core);
			goto bufs_done;
		}

		/* Cache firmware for fast per-frame reload (H.264 only) */
		if (ctx->dst_fmt.pixelformat == V4L2_PIX_FMT_H264 &&
		    !ctx->fw_vaddr) {
			ret = hcodec_cache_firmware(core, ctx);
			if (ret)
				dev_warn(core->dev,
					 "FW cache failed, per-frame reload disabled\n");
		}
		mutex_unlock(&core->hw_lock);

		if (!ctx->canvas_num) {
			ret = hcodec_alloc_canvases(core, ctx);
			if (ret)
				goto bufs_done;
		}

		mutex_lock(&core->hw_lock);
		ctx->frame_number = 0;
		ctx->idr_pic_id = 0;
		ctx->pic_order_cnt_lsb = 0;
		ctx->frames_since_idr = 0;
		ctx->sequence_out = 0;

		/* Apply noise reduction setting */
		ctx->nr_mode = clamp(nr_mode, 0, 3);

		/* Apply ME/IE weight tuning */
		ctx->me_weight = me_weight ? me_weight : ME_WEIGHT_DEFAULT;
		ctx->i4mb_weight = i4mb_weight ? i4mb_weight : I4MB_WEIGHT_DEFAULT;
		ctx->i16mb_weight = i16mb_weight ? i16mb_weight : I16MB_WEIGHT_DEFAULT;

		/*
		 * Always compute CBR block dimensions and fill the CBR
		 * table.  The firmware reads this table for per-block QP
		 * even in constant-QP mode.  hcodec_rc_init handles both
		 * RC-enabled (dynamic QP) and RC-disabled (constant QP
		 * in all table entries) cases.
		 */
		hcodec_compute_block_dims(ctx);
		hcodec_rc_init(ctx, ctx->fps);

		/*
		 * Generate firmware SPS/PPS for H.264, but only when
		 * rate control is OFF.  When RC is enabled, QP changes
		 * per-frame, so the PPS pic_init_qp_minus26 must be
		 * regenerated for each IDR.  The software SPS/PPS path
		 * in amvenc_h264_complete handles this (reads ctx->qp
		 * at IDR time).
		 *
		 * Additional case (added 2026-05-27 INTEROP-FOLLOWUP-HCODEC-
		 * VDEC-1080P-SPS): the firmware-generated SPS omits the
		 * `frame_cropping_flag` + `frame_crop_bottom_offset` fields
		 * even when the encoder operates on a canvas larger than the
		 * userspace-requested height (e.g., 1080 -> canvas_h = 1088
		 * per the KI-026 alignment fix).  The Amlogic VDEC HW H.264
		 * decoder rejects such an SPS at SOURCE_CHANGE detection (no
		 * event fires; downstream pipelines time out).  When the
		 * canvas height differs from ctx->height we must therefore
		 * fall through to the SW SPS path (`hcodec_build_sps_pps`
		 * at hcodec.c:1544+) which emits proper cropping for the
		 * 16-aligned canvas.
		 */
		if (ctx->dst_fmt.pixelformat == V4L2_PIX_FMT_H264 &&
		    !ctx->rc_enable &&
		    ALIGN(ctx->height, 16) == ctx->height) {
			ret = hcodec_fw_generate_sps_pps(core, ctx);
			if (ret)
				dev_warn(core->dev,
					 "FW SPS/PPS failed, using software fallback\n");
			else
				dev_dbg(core->dev,
					"HCODEC: FW SPS/PPS cached (%u bytes, qp=%u)\n",
					ctx->sps_pps_len, ctx->qp);
		} else if (ctx->dst_fmt.pixelformat == V4L2_PIX_FMT_H264 &&
			   !ctx->rc_enable) {
			dev_info(core->dev,
				 "HCODEC: height %u not 16-aligned (canvas %u); SW SPS path required for proper bottom-crop\n",
				 ctx->height, ALIGN(ctx->height, 16));
		} else if (ctx->dst_fmt.pixelformat == V4L2_PIX_FMT_H264) {
			dev_dbg(core->dev,
				"HCODEC: RC enabled, skipping FW SPS/PPS (rc=%d, sps_pps_len=%u)\n",
				ctx->rc_enable, ctx->sps_pps_len);
		}
		mutex_unlock(&core->hw_lock);
	} else {
		ctx->sequence_cap = 0;
	}

	return 0;

bufs_done:
	{
		struct vb2_v4l2_buffer *buf;

		while ((buf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx)))
			v4l2_m2m_buf_done(buf, VB2_BUF_STATE_QUEUED);
		while ((buf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx)))
			v4l2_m2m_buf_done(buf, VB2_BUF_STATE_QUEUED);
	}
	return ret;
}

/*
 * Stop the HCODEC engine and drop this session's power reference.
 *
 * Signal any in-progress device_run to abort its poll loop, then wait for it
 * to finish before powering off.  Without this, STREAMOFF during encode causes
 * device_run to read registers after hcodec_poweroff disables the DOS bus
 * clock, triggering a synchronous external abort (kernel panic).  KI-025.
 *
 * Factored out of amvenc_stop_streaming() by N104 so the unbind quiesce runs
 * exactly this sequence rather than a second, differently-ordered copy of it.
 */
static void amvenc_halt_engine(struct amvdec_core *core)
{
	atomic_set(&core->enc_aborting, 1);
	wait_for_completion_timeout(&core->enc_run_done,
				    msecs_to_jiffies(2000));

	mutex_lock(&core->hw_lock);
	hcodec_stop(core);
	mutex_unlock(&core->hw_lock);
	hcodec_power_put(core);

	atomic_set(&core->enc_aborting, 0);
}

/*
 * N104: force-quiesce the live encode session during vdec_remove().
 *
 * The decode side got this in f970fe5b4cefc; the encoder had no teardown at
 * all.  It runs BEFORE remove()'s power-off block and AFTER core->unbound is
 * set, so amvenc_stop_streaming() -- which still arrives later, from the fd's
 * ->release via v4l2_m2m_ctx_release() -> vb2_queue_release() -- takes its
 * no-MMIO branch and the engine is halted exactly once.
 *
 * vb2_is_streaming() is the predicate rather than a new flag: vb2 calls
 * stop_streaming() only after a successful start_streaming(), which is where
 * hcodec_power_get() happens, so an open-but-not-streaming ctx must NOT be
 * power_put() here.
 */
void amvenc_quiesce(struct amvdec_core *core)
{
	struct amvenc_ctx *ctx;
	struct vb2_queue *out_q;

	mutex_lock(&core->sessions_lock);
	ctx = core->enc_ctx;
	mutex_unlock(&core->sessions_lock);
	if (!ctx)
		return;

	out_q = v4l2_m2m_get_vq(ctx->fh.m2m_ctx,
				V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	if (!out_q || !vb2_is_streaming(out_q))
		return;

	amvenc_halt_engine(core);
}

static void amvenc_stop_streaming(struct vb2_queue *q)
{
	struct amvenc_ctx *ctx = vb2_get_drv_priv(q);
	struct amvdec_core *core = ctx->core;
	struct vb2_v4l2_buffer *buf;

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		/*
		 * N104: after an unbind, amvenc_quiesce() has already done
		 * every hardware step below -- and the iomem this reaches
		 * through is gone with the devres unmap, so repeating it is
		 * the encoder's version of the panic N103 fixed on the decode
		 * side.  The buffer return below still runs.
		 */
		if (!READ_ONCE(core->unbound))
			amvenc_halt_engine(core);

		while ((buf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx)))
			v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);
	} else {
		while ((buf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx)))
			v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);
	}
}

static const struct vb2_ops amvenc_vb2_ops = {
	.queue_setup = amvenc_queue_setup,
	.buf_prepare = amvenc_buf_prepare,
	.buf_queue = amvenc_buf_queue,
	.start_streaming = amvenc_start_streaming,
	.stop_streaming = amvenc_stop_streaming,
};

static int amvenc_queue_init(void *priv, struct vb2_queue *src_vq,
			     struct vb2_queue *dst_vq)
{
	struct amvenc_ctx *ctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct vb2_v4l2_buffer);
	src_vq->ops = &amvenc_vb2_ops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->core->enc_lock;
	src_vq->dev = ctx->core->dev;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct vb2_v4l2_buffer);
	dst_vq->ops = &amvenc_vb2_ops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->core->enc_lock;
	dst_vq->dev = ctx->core->dev;

	return vb2_queue_init(dst_vq);
}

/*
 * H.264 frame completion: copy bitstream output, update state.
 * Called from both interrupt (threaded ISR) and poll paths.
 */
static void amvenc_h264_complete(struct amvdec_core *core,
				 struct amvenc_ctx *ctx, bool is_idr)
{
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	u32 total_bytes;

	src_buf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
	dst_buf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);

	if (!src_buf || !dst_buf) {
		if (src_buf)
			v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_ERROR);
		if (dst_buf)
			v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_ERROR);
		goto finish;
	}

	total_bytes = amvdec_read_dos(core, HCODEC_VLC_TOTAL_BYTES);

	if (total_bytes > 0 &&
	    ctx->h264_hdr_len + total_bytes <=
	    vb2_plane_size(&dst_buf->vb2_buf, 0)) {
		void *dst = vb2_plane_vaddr(&dst_buf->vb2_buf, 0);
		u32 hdr_len = ctx->h264_hdr_len;

		if (dst) {
			/*
			 * Zero-copy: VLC data is already in the CAPTURE buffer
			 * at h264_vlc_off (ALIGN(hdr_len, 8)).  Close the
			 * alignment gap, then write the SPS/PPS header.
			 */
			if (ctx->h264_vlc_off != hdr_len)
				memmove(dst + hdr_len,
					dst + ctx->h264_vlc_off,
					total_bytes);
			if (hdr_len > 0) {
				dev_dbg(core->dev,
					"HCODEC IDR: SPS/PPS (%u bytes, rc=%d qp=%u)\n",
					hdr_len, ctx->rc_enable, ctx->qp);
				memcpy(dst, ctx->sps_pps, hdr_len);
				/* Dump SPS/PPS hex for debugging pic_init_qp */
				if (hdr_len <= 128) {
					dev_dbg(core->dev,
						"HCODEC SPS/PPS[0..%u]: %*ph\n",
						min_t(u32, hdr_len, 48) - 1,
						min_t(u32, hdr_len, 48), (u8 *)dst);
				}
			}
			/* Dump first 16 bytes of slice NALU for analysis */
			if (total_bytes >= 16) {
				u8 *s = dst + hdr_len;

				dev_dbg(core->dev,
					"HCODEC slice[0..15]: %*ph (total=%u idr=%d qp=%u)\n",
					16, s, total_bytes,
					is_idr, ctx->qp);
			}
		}

		vb2_set_plane_payload(&dst_buf->vb2_buf, 0,
				     hdr_len + total_bytes);
	} else {
		vb2_set_plane_payload(&dst_buf->vb2_buf, 0, 0);
	}

	if (is_idr)
		dst_buf->flags |= V4L2_BUF_FLAG_KEYFRAME;
	else
		dst_buf->flags |= V4L2_BUF_FLAG_PFRAME;

	ctx->frames_since_idr++;
	ctx->frame_number++;
	ctx->pic_order_cnt_lsb += 2;
	if (ctx->pic_order_cnt_lsb >= (1 << 4))
		ctx->pic_order_cnt_lsb = 0;

	swap(ctx->dblk_canvas, ctx->ref_canvas);

	/* Update rate control with actual frame size */
	if (ctx->rc_enable && ctx->target_bits)
		hcodec_rc_update(ctx, total_bytes * 8);

	src_buf->sequence = ctx->sequence_out++;
	dst_buf->sequence = ctx->sequence_cap++;

	v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_DONE);
	v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_DONE);
finish:
	v4l2_m2m_job_finish(core->m2m_enc, ctx->fh.m2m_ctx);
}

/* M2M device_run: triggered when both src and dst buffers are available */
static void amvenc_device_run(void *priv)
{
	struct amvenc_ctx *ctx = priv;
	struct amvdec_core *core = ctx->core;
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	struct vb2_buffer *src_vb;
	dma_addr_t y_addr, uv_addr, v_addr;
	u32 canvas_w;
	bool idr;

	reinit_completion(&core->enc_run_done);

	/*
	 * N106: the device is unbinding -- return the pair and finish the job
	 * WITHOUT touching a register.
	 *
	 * amvenc_quiesce() has stopped the engine and dropped its HCODEC power
	 * reference, and vdec_remove() cuts the DOS bus clock ~20 us later
	 * (measured), so every MMIO below is on a dead block.  This dispatch
	 * exists because v4l2_m2m_job_finish() from the aborted run schedules
	 * job_work asynchronously: measured on GXL, a follow-on device_run
	 * entered 3646 us after the halt returned and oopsed the board.
	 * amvenc_job_ready() stops the queueing side; this is the already-
	 * queued side, and the two together are the fix.
	 *
	 * The buffers must still be returned and the job still finished --
	 * v4l2_m2m requires every device_run to end in a job_finish, and
	 * stop_streaming() cannot reclaim a job the scheduler thinks is live.
	 */
	if (READ_ONCE(core->unbound)) {
		src_buf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		dst_buf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		if (src_buf)
			v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_ERROR);
		if (dst_buf)
			v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_ERROR);
		v4l2_m2m_job_finish(core->m2m_enc, ctx->fh.m2m_ctx);
		complete(&core->enc_run_done);
		return;
	}

	src_buf = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst_buf = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	if (!src_buf || !dst_buf) {
		dev_err(core->dev, "HCODEC: device_run with NULL buffer (src=%p dst=%p)\n",
			src_buf, dst_buf);
		v4l2_m2m_job_finish(core->m2m_enc, ctx->fh.m2m_ctx);
		complete(&core->enc_run_done);
		return;
	}
	src_vb = &src_buf->vb2_buf;

	idr = (ctx->frames_since_idr == 0) ||
	      (ctx->frames_since_idr >= ctx->gop_size) ||
	      ctx->force_keyframe;

	if (idr) {
		ctx->frames_since_idr = 0;
		ctx->frame_number = 0;
		ctx->pic_order_cnt_lsb = 0;
		ctx->idr_pic_id++;
		ctx->force_keyframe = false;
	}
	ctx->current_idr = idr;

	canvas_w = ALIGN(ctx->width, 32);
	v_addr = 0;

	/*
	 * Multi-planar input: each plane starts data_offset bytes into its
	 * buffer.  A client that backs every plane with ONE dmabuf (Android
	 * v4l2_codec2's V4L2Encoder over a gralloc NV12 buffer) passes the
	 * chroma position only there -- plane 1's DMA address is plane 0's --
	 * so ignoring it fed luma to the chroma input (round trip U/V ~6 dB).
	 */
	switch (ctx->src_fmt.pixelformat) {
	case V4L2_PIX_FMT_NV12M:
	case V4L2_PIX_FMT_NV21M:
		y_addr = vb2_dma_contig_plane_dma_addr(src_vb, 0) +
			 src_vb->planes[0].data_offset;
		uv_addr = vb2_dma_contig_plane_dma_addr(src_vb, 1) +
			  src_vb->planes[1].data_offset;
		break;
	case V4L2_PIX_FMT_YUV420M:
		/* MFDIN requires 64-byte aligned stride for planar YUV420 */
		canvas_w = ALIGN(ctx->width, 64);
		y_addr = vb2_dma_contig_plane_dma_addr(src_vb, 0) +
			 src_vb->planes[0].data_offset;
		uv_addr = vb2_dma_contig_plane_dma_addr(src_vb, 1) +
			  src_vb->planes[1].data_offset;
		v_addr = vb2_dma_contig_plane_dma_addr(src_vb, 2) +
			 src_vb->planes[2].data_offset;
		break;
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_RGB24:
	case V4L2_PIX_FMT_ABGR32:
		y_addr = vb2_dma_contig_plane_dma_addr(src_vb, 0);
		uv_addr = 0; /* single-plane packed format */
		break;
	default: /* NV12, NV21 single-planar */
		y_addr = vb2_dma_contig_plane_dma_addr(src_vb, 0);
		uv_addr = y_addr + canvas_w * ctx->height;
		break;
	}

	dev_dbg(core->dev,
		"HCODEC encode: pixfmt=%c%c%c%c %ux%u canvas_w=%u y=%pad uv=%pad v=%pad\n",
		ctx->src_fmt.pixelformat & 0xff,
		(ctx->src_fmt.pixelformat >> 8) & 0xff,
		(ctx->src_fmt.pixelformat >> 16) & 0xff,
		(ctx->src_fmt.pixelformat >> 24) & 0xff,
		ctx->width, ctx->height, canvas_w,
		&y_addr, &uv_addr, &v_addr);

	dst_buf->vb2_buf.timestamp = src_buf->vb2_buf.timestamp;

	mutex_lock(&core->hw_lock);
	reinit_completion(&core->enc_done);

	if (ctx->dst_fmt.pixelformat == V4L2_PIX_FMT_JPEG) {
		/*
		 * Zero-copy JPEG: build header to get exact length, then
		 * program VLC to write scan data at ALIGN(hdr_len, 8) in
		 * the CAPTURE buffer.  VLC requires 8-byte aligned start.
		 * After encode, memmove closes the 0-7 byte alignment gap.
		 */
		dma_addr_t dst_dma;
		u32 capture_size, vlc_off, vlc_space;

		/*
		 * Effective chroma is determined by source pixel format:
		 *   ABGR32 (RGBA) -> YUV444  (RGB->YUV444, full chroma)
		 *   YUYV          -> YUV422  (native 4:2:2 packed format)
		 *   NV12/other    -> YUV420  (default)
		 *
		 * The V4L2_CID_JPEG_CHROMA_SUBSAMPLING control is exposed
		 * for GStreamer compat but GStreamer 1.26.x cannot set menu
		 * controls via extra-controls. The format-to-chroma mapping
		 * is therefore implicit from the source format, not the
		 * control value. This ensures correct JPEG output regardless
		 * of whether the application sets the control.
		 */
		u32 effective_chroma;

		if (ctx->src_fmt.pixelformat == V4L2_PIX_FMT_ABGR32)
			effective_chroma = V4L2_JPEG_CHROMA_SUBSAMPLING_444;
		else if (ctx->src_fmt.pixelformat == V4L2_PIX_FMT_YUYV)
			effective_chroma = V4L2_JPEG_CHROMA_SUBSAMPLING_422;
		else
			effective_chroma = V4L2_JPEG_CHROMA_SUBSAMPLING_420;
		/* Resolve quality vs. custom tables once; used by both header and HW */
		jpegenc_compute_tables(ctx);
		ctx->jpeg_hdr_len = jpegenc_build_header(ctx->jpeg_hdr,
							 ctx->width,
							 ctx->height,
							 ctx->jpeg_eff_luma_qt,
							 ctx->jpeg_eff_chroma_qt,
							 effective_chroma);
		dst_dma = vb2_dma_contig_plane_dma_addr(&dst_buf->vb2_buf, 0);
		capture_size = vb2_plane_size(&dst_buf->vb2_buf, 0);
		vlc_off = ALIGN(ctx->jpeg_hdr_len, 8);
		vlc_space = capture_size - vlc_off - 2; /* -2 for EOI */
		ctx->jpeg_vlc_off = vlc_off;

		jpegenc_encode(core, ctx, y_addr, uv_addr, canvas_w,
			       dst_dma + vlc_off, vlc_space);
	} else {
		/*
		 * Zero-copy H.264: compute header length up front so we can
		 * point VLC directly at the CAPTURE buffer, leaving room for
		 * the SPS/PPS header at the start.  VLC requires 8-byte
		 * alignment, so VLC starts at ALIGN(hdr_len, 8).  At
		 * completion, amvenc_h264_complete() closes the alignment gap
		 * with memmove and writes the header at offset 0.
		 */
		dma_addr_t dst_dma;
		u32 capture_size, vlc_off, vlc_space;
		u32 hdr_len;

		dst_dma = vb2_dma_contig_plane_dma_addr(&dst_buf->vb2_buf, 0);
		capture_size = vb2_plane_size(&dst_buf->vb2_buf, 0);

		/* Pre-compute header length for VLC placement */
		if (idr) {
			if (ctx->sps_pps_len > 0) {
				hdr_len = ctx->sps_pps_len;
			} else {
				/*
				 * SW SPS/PPS path: build it now and cache in
				 * ctx->sps_pps so completion can copy it.
				 */
				ctx->sps_pps_len = hcodec_build_sps_pps(
					core, ctx, ctx->sps_pps,
					sizeof(ctx->sps_pps));
				hdr_len = ctx->sps_pps_len;
			}
		} else {
			hdr_len = 0;
		}

		vlc_off = ALIGN(hdr_len, 8);
		vlc_space = capture_size - vlc_off;
		ctx->h264_hdr_len = hdr_len;
		ctx->h264_vlc_off = vlc_off;

		/* Apply rate-controlled QP if RC is active */
		if (ctx->rc_enable && ctx->target_bits)
			hcodec_rc_get_qp(ctx, idr);
		hcodec_encode(core, ctx, idr, y_addr, uv_addr, v_addr, canvas_w,
			      dst_dma + vlc_off, vlc_space);
	}

	mutex_unlock(&core->hw_lock);

	/*
	 * Wait for encode completion.  The hardirq ISR (amvenc_isr)
	 * signals enc_done when firmware writes ENCODER_IDR_DONE or
	 * ENCODER_NON_IDR_DONE to ENCODER_STATUS.
	 *
	 * Strategy: IRQ-driven wait first, with a polling fallback for
	 * robustness.  MBOX2 interrupts fire reliably on all platforms
	 * (GXL CAVLC, G12B/SM1 CABAC -- verified 2026-04-11 via
	 * /proc/interrupts: exactly 1 IRQ per encoded frame on GXL
	 * S805X 52626db91ddc, 200 frames, 0 missed).
	 *
	 * The threaded ISR is a no-op for H.264 -- all completion
	 * processing happens here, so there is no double-complete race.
	 * For JPEG, the threaded ISR handles completion, but we still
	 * keep a poll fallback for safety.
	 */
	{
		u32 poll_status;
		int poll_i;
		bool poll_ok = false;
		bool is_jpeg = (ctx->dst_fmt.pixelformat == V4L2_PIX_FMT_JPEG);
		unsigned long timeout_jiffies;

		/*
		 * Phase 1: IRQ-driven wait.
		 * MBOX2 fires reliably on all platforms (GXL, G12B, SM1).
		 *
		 * JPEG 4:4:4 (pic_format=1) processes a 40x30 MCU grid at
		 * 320x240 (2x more positions than 4:2:2) and each MCU requires
		 * the MFDIN RGB->YUV444 conversion path, which runs ~16x slower
		 * than the 4:2:0 planar path: measured ~2.35s at 320x240 vs
		 * ~150ms for 4:2:0. Use 5000ms for JPEG to cover 4:4:4 up to
		 * 320x240 with headroom. H.264 frames finish in <100ms.
		 *
		 * TODO: investigate why MFDIN oformat=1 (YUV444) is 16x slower
		 * than oformat=0 (YUV420) for the same RGBA input. It may be a
		 * missing MFDIN configuration bit (e.g. stride or pixel size).
		 */
		{
			/*
			 * Scale timeout with clock rate. At 800 MHz the
			 * base timeouts (500ms H.264, 5000ms JPEG) have
			 * ample headroom. At lower thermal states the
			 * AMRISC firmware runs proportionally slower, so
			 * scale the timeout inversely:
			 *   timeout = base * (800 MHz / actual_clk)
			 *
			 * This prevents encode timeouts during thermal
			 * throttle (200 MHz = 4x base timeout).
			 */
			unsigned long clk_hz = clk_get_rate(core->hcodec_clk);
			unsigned long base_ms = is_jpeg ? 5000 : 500;
			unsigned long scaled_ms;

			if (clk_hz > 0 && clk_hz < 800000000)
				scaled_ms = base_ms * 800000000 / clk_hz;
			else
				scaled_ms = base_ms;
			timeout_jiffies = msecs_to_jiffies(scaled_ms);
		}
		if (!atomic_read(&core->enc_aborting)) {
			long ret;

			ret = wait_for_completion_interruptible_timeout(
				&core->enc_done, timeout_jiffies);
			if (ret > 0) {
				/* IRQ fired -- check status register */
				poll_status = amvdec_read_dos(core,
							     ENCODER_STATUS);
				if (is_jpeg) {
					if (poll_status == JPEGENC_DONE)
						poll_ok = true;
				} else {
					if (poll_status == ENCODER_IDR_DONE ||
					    poll_status == ENCODER_NON_IDR_DONE)
						poll_ok = true;
				}
				if (poll_ok)
					core->enc_status = poll_status;
			}
		}

		/*
		 * Phase 2: polling fallback (100us intervals, up to 100ms).
		 * Safety net for any edge case where the IRQ was missed
		 * (e.g., IRQ disabled, handler contention, or JPEG path).
		 */
		if (!poll_ok) {
			for (poll_i = 0; poll_i < 1000; poll_i++) {
				if (atomic_read(&core->enc_aborting))
					break;
				poll_status = amvdec_read_dos(core,
							     ENCODER_STATUS);
				if (is_jpeg) {
					if (poll_status == JPEGENC_DONE) {
						poll_ok = true;
						break;
					}
				} else {
					if (poll_status == ENCODER_IDR_DONE ||
					    poll_status == ENCODER_NON_IDR_DONE) {
						poll_ok = true;
						break;
					}
				}
				usleep_range(100, 200);
			}
			if (poll_ok)
				core->enc_status = poll_status;
		}

		if (!poll_ok) {
			/*
			 * If aborting (STREAMOFF during encode), do not
			 * access hardware registers -- the DOS bus clock
			 * may already be disabled.  stop_streaming handles
			 * hcodec_stop and buffer cleanup.
			 */
			if (atomic_read(&core->enc_aborting)) {
				dev_dbg(core->dev,
					"HCODEC: encode aborted by STREAMOFF\n");
			} else {
				dev_err(core->dev,
					"HCODEC: encode timeout status=0x%x total_bytes=0x%x\n",
					amvdec_read_dos(core, ENCODER_STATUS),
					amvdec_read_dos(core,
							HCODEC_VLC_TOTAL_BYTES));
				mutex_lock(&core->hw_lock);
				hcodec_stop(core);
				mutex_unlock(&core->hw_lock);
			}

			src_buf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
			dst_buf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
			if (src_buf)
				v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_ERROR);
			if (dst_buf)
				v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_ERROR);
			v4l2_m2m_job_finish(core->m2m_enc, ctx->fh.m2m_ctx);
			complete(&core->enc_run_done);
			return;
		}

		/* Clear any pending interrupt that fired during poll */
		amvdec_write_dos(core, HCODEC_ASSIST_MBOX2_CLR_REG, 1);

		if (is_jpeg) {
			u32 total_bytes;

			src_buf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
			dst_buf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);

			if (!src_buf || !dst_buf) {
				if (src_buf)
					v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_ERROR);
				if (dst_buf)
					v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_ERROR);
				v4l2_m2m_job_finish(core->m2m_enc, ctx->fh.m2m_ctx);
				complete(&core->enc_run_done);
				return;
			}

			total_bytes = amvdec_read_dos(core, HCODEC_VLC_TOTAL_BYTES);

			if (total_bytes > 0 &&
			    ctx->jpeg_hdr_len + total_bytes + 2 <=
			    vb2_plane_size(&dst_buf->vb2_buf, 0)) {
				void *dst = vb2_plane_vaddr(&dst_buf->vb2_buf, 0);

				if (dst) {
					/*
					 * Zero-copy: scan data is already in
					 * CAPTURE buffer at jpeg_vlc_off.
					 * Close the 0-7 byte alignment gap
					 * with memmove, then write the header.
					 */
					if (ctx->jpeg_vlc_off != ctx->jpeg_hdr_len)
						memmove(dst + ctx->jpeg_hdr_len,
							dst + ctx->jpeg_vlc_off,
							total_bytes);
					memcpy(dst, ctx->jpeg_hdr,
					       ctx->jpeg_hdr_len);

					/* Append EOI */
					if (ctx->jpeg_hdr_len + total_bytes + 2 <=
					    vb2_plane_size(&dst_buf->vb2_buf, 0)) {
						u8 *eoi = dst + ctx->jpeg_hdr_len + total_bytes;

						eoi[0] = 0xff;
						eoi[1] = 0xd9;
						vb2_set_plane_payload(&dst_buf->vb2_buf, 0,
								     ctx->jpeg_hdr_len + total_bytes + 2);
					} else {
						vb2_set_plane_payload(&dst_buf->vb2_buf, 0,
								     ctx->jpeg_hdr_len + total_bytes);
					}
				} else {
					vb2_set_plane_payload(&dst_buf->vb2_buf, 0, 0);
				}
			} else {
				vb2_set_plane_payload(&dst_buf->vb2_buf, 0, 0);
			}

			dst_buf->flags |= V4L2_BUF_FLAG_KEYFRAME;
			src_buf->sequence = ctx->sequence_out++;
			dst_buf->sequence = ctx->sequence_cap++;

			v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_DONE);
			v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_DONE);
			v4l2_m2m_job_finish(core->m2m_enc, ctx->fh.m2m_ctx);
		} else {
			amvenc_h264_complete(core, ctx, idr);
		}

		complete(&core->enc_run_done);
	}
}

/* IRQ handler - hardirq context (Mailbox 2) */
irqreturn_t amvenc_isr(int irq, void *data)
{
	struct amvdec_core *core = data;
	u32 status;

	amvdec_write_dos(core, HCODEC_ASSIST_MBOX2_CLR_REG, 1);

	status = amvdec_read_dos(core, ENCODER_STATUS);

	if (status == ENCODER_IDR_DONE || status == ENCODER_NON_IDR_DONE ||
	    status == ENCODER_SEQUENCE_DONE || status == ENCODER_PICTURE_DONE ||
	    status == JPEGENC_DONE) {
		core->enc_status = status;
		complete(&core->enc_done);
		return IRQ_WAKE_THREAD;
	}

	return IRQ_HANDLED;
}

/* IRQ threaded handler - process context */
irqreturn_t amvenc_threaded_isr(int irq, void *data)
{
	/*
	 * Completion for BOTH H.264 and JPEG is handled by the poll path
	 * in amvenc_device_run() after wait_for_completion_interruptible_timeout
	 * returns. The hardirq handler's complete(&core->enc_done) wakes
	 * device_run, which reads the status register and processes the
	 * buffer + job_finish.
	 *
	 * Previously, this threaded ISR ALSO processed JPEG completion
	 * independently, causing a double-completion race with the poll
	 * path: both contexts would call v4l2_m2m_src_buf_remove /
	 * _dst_buf_remove / buf_done / job_finish. The first path saw the
	 * buffers and completed them; the second saw NULL and fell through
	 * to the error path, leaving vb2 state inconsistent. This manifested
	 * as "stop_streaming leaving buffer in active state" warnings and
	 * a pipeline hang after 1-2 frames at 640x480+ (HCODEC JPEG
	 * multi-frame hang).
	 *
	 * Keep the threaded ISR as a no-op. The hardirq complete() is
	 * sufficient to wake the poll path.
	 */
	return IRQ_HANDLED;
}

/*
 * Refuse to QUEUE a new job once the device is unbinding (N106).
 *
 * __v4l2_m2m_try_queue() consults this before putting the context on the
 * job_queue, so with it in place no dispatch can be created after
 * vdec_remove() sets core->unbound.  It is not sufficient on its own -- a
 * context already ON the queue when unbound is set has passed this test
 * already, which is what the guard at the top of amvenc_device_run() covers --
 * but it is what stops the aborted run's own v4l2_m2m_job_finish() from
 * re-queueing the context for every remaining buffer pair.
 */
static int amvenc_job_ready(void *priv)
{
	struct amvenc_ctx *ctx = priv;

	return !READ_ONCE(ctx->core->unbound);
}

static const struct v4l2_m2m_ops amvenc_m2m_ops = {
	.device_run = amvenc_device_run,
	.job_ready = amvenc_job_ready,
};

int amvenc_init(struct amvdec_core *core)
{
	struct video_device *vdev;
	int ret;

	mutex_init(&core->enc_lock);
	mutex_init(&core->hw_lock);
	init_waitqueue_head(&core->enc_wq);
	init_completion(&core->enc_done);
	init_completion(&core->enc_run_done);
	complete(&core->enc_run_done); /* initially "not running" */
	atomic_set(&core->enc_aborting, 0);

	core->m2m_enc = v4l2_m2m_init(&amvenc_m2m_ops);
	if (IS_ERR(core->m2m_enc)) {
		dev_err(core->dev, "Failed to init encoder M2M device\n");
		return PTR_ERR(core->m2m_enc);
	}

	vdev = video_device_alloc();
	if (!vdev) {
		ret = -ENOMEM;
		goto err_m2m;
	}

	core->vdev_enc = vdev;
	strscpy(vdev->name, "meson-video-encoder", sizeof(vdev->name));
	vdev->release = video_device_release;
	vdev->fops = &amvenc_fops;
	vdev->ioctl_ops = &amvenc_ioctl_ops;
	vdev->vfl_dir = VFL_DIR_M2M;
	vdev->v4l2_dev = &core->v4l2_dev;
	vdev->lock = &core->enc_lock;
	vdev->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	video_set_drvdata(vdev, core);

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		dev_err(core->dev, "Failed to register encoder video device\n");
		goto err_vdev;
	}

	ret = v4l2_m2m_register_media_controller(core->m2m_enc, vdev,
						 MEDIA_ENT_F_PROC_VIDEO_ENCODER);
	if (ret) {
		dev_err(core->dev, "Failed to register encoder media controller\n");
		goto err_unreg;
	}

	dev_info(core->dev, "HCODEC encoder registered as %s\n",
		 video_device_node_name(vdev));

	return 0;

err_unreg:
	video_unregister_device(vdev);
	core->vdev_enc = NULL;
err_vdev:
	video_device_release(vdev);
err_m2m:
	v4l2_m2m_release(core->m2m_enc);
	core->m2m_enc = NULL;
	return ret;
}

void amvenc_cleanup(struct amvdec_core *core)
{
	if (!core->vdev_enc)
		return;

	/*
	 * N104: the node is ALREADY unregistered, by vdec_remove()'s
	 * close-the-door block, and m2m_enc is deliberately NOT released here
	 * -- a still-open encoder fd dereferences it from amvenc_release()
	 * via v4l2_m2m_ctx_release().  vdec_core_release() frees it when the
	 * last reference goes.  This function is called only from
	 * vdec_remove(); amvenc_init()'s own failure path unwinds itself.
	 */
	core->vdev_enc = NULL;
}
