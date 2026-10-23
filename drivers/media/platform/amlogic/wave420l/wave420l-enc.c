// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Chips&Media Wave420L HEVC encoder - V4L2 M2M interface
 *
 * Copyright (C) 2025 Libre Computer
 */

#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "wave420l.h"

#define WAVE420L_ENC_NAME	"wave420l-enc"

/* Vendor-specific V4L2 controls (no standard CIDs for encoder NR) */
#define V4L2_CID_WAVE420L_BASE			(V4L2_CID_USER_BASE + 0x1100)
#define V4L2_CID_WAVE420L_NR_Y_ENABLE		(V4L2_CID_WAVE420L_BASE + 0)
#define V4L2_CID_WAVE420L_NR_CB_ENABLE		(V4L2_CID_WAVE420L_BASE + 1)
#define V4L2_CID_WAVE420L_NR_CR_ENABLE		(V4L2_CID_WAVE420L_BASE + 2)
#define V4L2_CID_WAVE420L_NR_NOISE_EST_ENABLE	(V4L2_CID_WAVE420L_BASE + 3)
#define V4L2_CID_WAVE420L_NR_NOISE_SIGMA_Y	(V4L2_CID_WAVE420L_BASE + 4)
#define V4L2_CID_WAVE420L_NR_NOISE_SIGMA_CB	(V4L2_CID_WAVE420L_BASE + 5)
#define V4L2_CID_WAVE420L_NR_NOISE_SIGMA_CR	(V4L2_CID_WAVE420L_BASE + 6)
#define V4L2_CID_WAVE420L_CHROMA_CB_QP_OFFSET	(V4L2_CID_WAVE420L_BASE + 7)
#define V4L2_CID_WAVE420L_CHROMA_CR_QP_OFFSET	(V4L2_CID_WAVE420L_BASE + 8)

/* Default bitstream buffer size per CAPTURE buffer */
#define WAVE420L_BS_BUF_SIZE	SZ_2M

/* Default encoding parameters */
#define WAVE420L_DEFAULT_BITRATE	4000000
#define WAVE420L_DEFAULT_GOP		30
#define WAVE420L_DEFAULT_QP		20
#define WAVE420L_DEFAULT_MIN_QP		8
#define WAVE420L_DEFAULT_MAX_QP		51
#define WAVE420L_DEFAULT_FRAMERATE	30

/* ------------------------------------------------------------------ */
/* Format definitions                                                  */
/* ------------------------------------------------------------------ */

struct wave420l_fmt {
	u32 fourcc;
	int num_planes;
	int bpp;		/* bits per pixel (for single-planar) */
	u32 enc_fmt;		/* W4_ENC_SRC_FORMAT_* */
};

static const struct wave420l_fmt wave420l_src_fmts[] = {
	{
		.fourcc = V4L2_PIX_FMT_NV12,
		.num_planes = 1,
		.bpp = 12,
		.enc_fmt = W4_ENC_SRC_FORMAT_NV12,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV21,
		.num_planes = 1,
		.bpp = 12,
		.enc_fmt = W4_ENC_SRC_FORMAT_NV21,
	},
	{
		.fourcc = V4L2_PIX_FMT_YUV420,
		.num_planes = 1,
		.bpp = 12,
		.enc_fmt = W4_ENC_SRC_FORMAT_I420,
	},
	/*
	 * Multi-plane (M) variants as TRUE multi-plane formats (KI-073).
	 * Wave420L hardware programs picSrcAddrY/U/V independently, so the
	 * M-variants honor the V4L2 multi-plane contract: NV12M / NV21M
	 * carry Y + interleaved UV in two DMA-separate buffers (num_planes=2),
	 * and YUV420M carries Y + U + V in three (num_planes=3).
	 *
	 * The earlier num_planes=1 single-buffer alias (added so ffmpeg 7.x's
	 * AV_PIX_FMT_NV21 -> V4L2_PIX_FMT_NV21M-only mapping would negotiate)
	 * lied about the multi-plane contract; GStreamer validates and
	 * rejected the lie at preroll with "Device wants 1 planes" -- broke
	 * v4l2h265enc end-to-end.  Reporting num_planes truthfully lets
	 * GStreamer negotiate AND ffmpeg's `v4l2_save_to_context` (which
	 * does not validate the returned plane count) keeps working in
	 * single-plane mode against the non-M entries.
	 */
	{
		.fourcc = V4L2_PIX_FMT_NV12M,
		.num_planes = 2,
		.bpp = 12,
		.enc_fmt = W4_ENC_SRC_FORMAT_NV12,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV21M,
		.num_planes = 2,
		.bpp = 12,
		.enc_fmt = W4_ENC_SRC_FORMAT_NV21,
	},
	{
		.fourcc = V4L2_PIX_FMT_YUV420M,
		.num_planes = 3,
		.bpp = 12,
		.enc_fmt = W4_ENC_SRC_FORMAT_I420,
	},
	/*
	 * P010 (NV12 with 16-bit/sample, 10-bit LSB): Main 10 profile input.
	 * Each pixel occupies 2 bytes (10 bits used, left-justified in 16-bit
	 * word).  bytesperline = ALIGN(width, 32) * 2; same interleaved UV
	 * layout as NV12 (cbcrInterleave=1, nv21=0, srcFrameFormat=2) but
	 * srcPixelFormat=5 (left-justified 16-bit, W4_WTL_LEFT_JUSTIFIED<<2 |
	 * W4_WTL_PIXEL_16BIT = (1<<2)|1).
	 * bpp=20 encodes the per-pixel cost: 16*1 + 16*0.5 = 20 bits/pixel avg
	 * (luma stride * height * 5/4 gives correct buffer size below).
	 */
	{
		.fourcc = V4L2_PIX_FMT_P010,
		.num_planes = 1,
		.bpp = 20,
		.enc_fmt = W4_ENC_SRC_FORMAT_NV12,	/* same frame layout as NV12 */
	},
};

static const struct wave420l_fmt wave420l_dst_fmts[] = {
	{
		.fourcc = V4L2_PIX_FMT_HEVC,
		.num_planes = 1,
	},
};

static const struct wave420l_fmt *wave420l_find_fmt(const struct wave420l_fmt *fmts,
						    int nfmts, u32 fourcc)
{
	int i;

	for (i = 0; i < nfmts; i++)
		if (fmts[i].fourcc == fourcc)
			return &fmts[i];

	return &fmts[0];
}

/* ------------------------------------------------------------------ */
/* V4L2 colorspace to HEVC VUI mapping                                 */
/* ------------------------------------------------------------------ */

/*
 * Map V4L2 colorspace/xfer_func/ycbcr_enc to HEVC VUI values.
 * These use the ITU-T H.273 / ISO 23091-2 coding-independent code points
 * (identical to HEVC Annex E Table E.3/E.4/E.5).
 *
 * This auto-mapping only runs if the user hasn't set explicit controls.
 * Explicit control values always take priority.
 */
static void wave420l_map_colorspace_to_vui(struct wave420l_instance *inst)
{
	struct v4l2_pix_format_mplane *fmt = &inst->src_fmt;

	/* Only auto-map if no explicit VUI controls were set */
	if (inst->colour_primaries || inst->transfer_characteristics ||
	    inst->matrix_coefficients)
		return;

	switch (fmt->colorspace) {
	case V4L2_COLORSPACE_DEFAULT:
		/*
		 * V4L2_COLORSPACE_DEFAULT (0) is what most userspace sends
		 * when it doesn't explicitly set colorspace.  Default to
		 * BT.709 per V4L2 convention for HDTV resolutions.
		 */
	case V4L2_COLORSPACE_REC709:
		inst->colour_primaries = 1;	   /* BT.709 */
		inst->transfer_characteristics = 1; /* BT.709 */
		inst->matrix_coefficients = 1;	   /* BT.709 */
		break;
	case V4L2_COLORSPACE_BT2020:
		inst->colour_primaries = 9;	   /* BT.2020 */
		inst->matrix_coefficients = 9;	   /* BT.2020 non-const lum */
		/* Transfer function from xfer_func */
		switch (fmt->xfer_func) {
		case V4L2_XFER_FUNC_SMPTE2084:
			inst->transfer_characteristics = 16; /* PQ */
			break;
		default:
			inst->transfer_characteristics = 14; /* BT.2020-10 */
			break;
		}
		break;
	case V4L2_COLORSPACE_SMPTE170M:
		inst->colour_primaries = 6;	   /* BT.601-625 */
		inst->transfer_characteristics = 6; /* BT.601 */
		inst->matrix_coefficients = 6;	   /* BT.601 */
		break;
	case V4L2_COLORSPACE_SMPTE240M:
		inst->colour_primaries = 7;	   /* SMPTE 240M */
		inst->transfer_characteristics = 7; /* SMPTE 240M */
		inst->matrix_coefficients = 7;	   /* SMPTE 240M */
		break;
	case V4L2_COLORSPACE_SRGB:
		inst->colour_primaries = 1;	   /* BT.709 */
		inst->transfer_characteristics = 13; /* sRGB */
		inst->matrix_coefficients = 1;	   /* BT.709 */
		break;
	default:
		/* Unspecified -- don't set VUI */
		break;
	}

	/* Override transfer function if explicitly set on format */
	if (fmt->xfer_func && inst->colour_primaries) {
		switch (fmt->xfer_func) {
		case V4L2_XFER_FUNC_SMPTE2084:
			inst->transfer_characteristics = 16;
			break;
		default:
			break;
		}
	}

	/* Video format: component (0) for progressive */
	inst->video_format = 5;  /* unspecified */
}

/* ------------------------------------------------------------------ */
/* Format helpers                                                      */
/* ------------------------------------------------------------------ */

static void wave420l_set_default_src_fmt(struct v4l2_pix_format_mplane *fmt)
{
	memset(fmt, 0, sizeof(*fmt));
	fmt->pixelformat = V4L2_PIX_FMT_NV12;
	fmt->width = 1920;
	fmt->height = 1080;
	fmt->num_planes = 1;
	fmt->plane_fmt[0].bytesperline = ALIGN(fmt->width, 32);
	fmt->plane_fmt[0].sizeimage = fmt->plane_fmt[0].bytesperline *
				      fmt->height * 3 / 2;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_REC709;
}

static void wave420l_set_default_dst_fmt(struct v4l2_pix_format_mplane *fmt)
{
	memset(fmt, 0, sizeof(*fmt));
	fmt->pixelformat = V4L2_PIX_FMT_HEVC;
	fmt->width = 1920;
	fmt->height = 1080;
	fmt->num_planes = 1;
	fmt->plane_fmt[0].bytesperline = 0;
	fmt->plane_fmt[0].sizeimage = WAVE420L_BS_BUF_SIZE;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_REC709;
}

/* ------------------------------------------------------------------ */
/* v4l2_fh helper — convert file handle to instance                    */
/* ------------------------------------------------------------------ */

static inline struct wave420l_instance *v4l2_fh_to_inst(struct file *file)
{
	return container_of(file_to_v4l2_fh(file), struct wave420l_instance, fh);
}

/* ------------------------------------------------------------------ */
/* V4L2 ioctl operations                                               */
/* ------------------------------------------------------------------ */

static int wave420l_querycap(struct file *file, void *priv,
			     struct v4l2_capability *cap)
{
	strscpy(cap->driver, WAVE420L_ENC_NAME, sizeof(cap->driver));
	strscpy(cap->card, "Wave420L HEVC Encoder", sizeof(cap->card));

	return 0;
}

static int wave420l_enum_fmt_out(struct file *file, void *priv,
				 struct v4l2_fmtdesc *f)
{
	if (f->index >= ARRAY_SIZE(wave420l_src_fmts))
		return -EINVAL;

	f->pixelformat = wave420l_src_fmts[f->index].fourcc;
	return 0;
}

static int wave420l_enum_fmt_cap(struct file *file, void *priv,
				 struct v4l2_fmtdesc *f)
{
	if (f->index >= ARRAY_SIZE(wave420l_dst_fmts))
		return -EINVAL;

	f->pixelformat = wave420l_dst_fmts[f->index].fourcc;
	return 0;
}

static int wave420l_g_fmt_out(struct file *file, void *priv,
			      struct v4l2_format *f)
{
	struct wave420l_instance *inst = v4l2_fh_to_inst(file);

	f->fmt.pix_mp = inst->src_fmt;
	return 0;
}

static int wave420l_g_fmt_cap(struct file *file, void *priv,
			      struct v4l2_format *f)
{
	struct wave420l_instance *inst = v4l2_fh_to_inst(file);

	f->fmt.pix_mp = inst->dst_fmt;
	return 0;
}

static int wave420l_try_fmt_out(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	const struct wave420l_fmt *fmt;

	fmt = wave420l_find_fmt(wave420l_src_fmts, ARRAY_SIZE(wave420l_src_fmts),
				pix->pixelformat);
	pix->pixelformat = fmt->fourcc;

	pix->width = clamp(pix->width, (u32)WAVE420L_MIN_WIDTH,
			   (u32)WAVE420L_MAX_WIDTH);
	pix->height = clamp(pix->height, (u32)WAVE420L_MIN_HEIGHT,
			    (u32)WAVE420L_MAX_HEIGHT);
	/* Wave420L requires 8-pixel alignment */
	pix->width = ALIGN(pix->width, 8);
	pix->height = ALIGN(pix->height, 8);

	pix->num_planes = fmt->num_planes;
	/*
	 * P010 stores each luma/chroma sample in a 16-bit word, so luma
	 * stride is ALIGN(width, 32) * 2 bytes.  All other formats use
	 * ALIGN(width, 32) bytes per luma row.
	 */
	if (fmt->fourcc == V4L2_PIX_FMT_P010)
		pix->plane_fmt[0].bytesperline = ALIGN(pix->width, 32) * 2;
	else
		pix->plane_fmt[0].bytesperline = ALIGN(pix->width, 32);

	if (fmt->num_planes == 1) {
		/* Single-buffer layout: plane[0] holds Y + chroma contiguous,
		 * sizeimage = stride * height * bpp / 8 covers the full image.
		 */
		pix->plane_fmt[0].sizeimage = pix->plane_fmt[0].bytesperline *
					      pix->height * fmt->bpp / 8;
	} else if (fmt->num_planes == 2) {
		/* NV12M / NV21M: Y plane + interleaved UV plane.
		 * plane[0] = stride * height (Y), plane[1] = stride * height / 2 (UV).
		 */
		pix->plane_fmt[0].sizeimage =
			pix->plane_fmt[0].bytesperline * pix->height;
		pix->plane_fmt[1].bytesperline = pix->plane_fmt[0].bytesperline;
		pix->plane_fmt[1].sizeimage =
			pix->plane_fmt[1].bytesperline * pix->height / 2;
	} else { /* num_planes == 3 */
		/* YUV420M: separate Y, U, V planes.  Y at full stride,
		 * U/V at half stride and half height.
		 */
		pix->plane_fmt[0].sizeimage =
			pix->plane_fmt[0].bytesperline * pix->height;
		pix->plane_fmt[1].bytesperline =
			pix->plane_fmt[0].bytesperline / 2;
		pix->plane_fmt[1].sizeimage =
			pix->plane_fmt[1].bytesperline * pix->height / 2;
		pix->plane_fmt[2].bytesperline =
			pix->plane_fmt[1].bytesperline;
		pix->plane_fmt[2].sizeimage = pix->plane_fmt[1].sizeimage;
	}

	pix->field = V4L2_FIELD_NONE;

	return 0;
}

static int wave420l_try_fmt_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct wave420l_instance *inst = v4l2_fh_to_inst(file);
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;

	pix->pixelformat = V4L2_PIX_FMT_HEVC;
	/* CAPTURE size matches OUTPUT */
	pix->width = inst->src_fmt.width;
	pix->height = inst->src_fmt.height;
	pix->num_planes = 1;
	pix->plane_fmt[0].bytesperline = 0;
	if (pix->plane_fmt[0].sizeimage < WAVE420L_BS_BUF_SIZE)
		pix->plane_fmt[0].sizeimage = WAVE420L_BS_BUF_SIZE;
	pix->field = V4L2_FIELD_NONE;

	return 0;
}

static int wave420l_s_fmt_out(struct file *file, void *priv,
			      struct v4l2_format *f)
{
	struct wave420l_instance *inst = v4l2_fh_to_inst(file);
	int ret;

	ret = wave420l_try_fmt_out(file, priv, f);
	if (ret)
		return ret;

	inst->src_fmt = f->fmt.pix_mp;

	/* Propagate size and colorspace to CAPTURE */
	inst->dst_fmt.width = inst->src_fmt.width;
	inst->dst_fmt.height = inst->src_fmt.height;
	inst->dst_fmt.colorspace = inst->src_fmt.colorspace;
	inst->dst_fmt.xfer_func = inst->src_fmt.xfer_func;
	inst->dst_fmt.ycbcr_enc = inst->src_fmt.ycbcr_enc;
	inst->dst_fmt.quantization = inst->src_fmt.quantization;

	return 0;
}

static int wave420l_s_fmt_cap(struct file *file, void *priv,
			      struct v4l2_format *f)
{
	struct wave420l_instance *inst = v4l2_fh_to_inst(file);
	int ret;

	ret = wave420l_try_fmt_cap(file, priv, f);
	if (ret)
		return ret;

	inst->dst_fmt = f->fmt.pix_mp;
	return 0;
}

static bool wave420l_is_valid_fmt(u32 fourcc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(wave420l_src_fmts); i++)
		if (wave420l_src_fmts[i].fourcc == fourcc)
			return true;
	for (i = 0; i < ARRAY_SIZE(wave420l_dst_fmts); i++)
		if (wave420l_dst_fmts[i].fourcc == fourcc)
			return true;
	return false;
}

static int wave420l_enum_framesizes(struct file *file, void *priv,
				    struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index)
		return -EINVAL;

	if (!wave420l_is_valid_fmt(fsize->pixel_format))
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.min_width = WAVE420L_MIN_WIDTH;
	fsize->stepwise.max_width = WAVE420L_MAX_WIDTH;
	fsize->stepwise.step_width = 8;
	fsize->stepwise.min_height = WAVE420L_MIN_HEIGHT;
	fsize->stepwise.max_height = WAVE420L_MAX_HEIGHT;
	fsize->stepwise.step_height = 8;

	return 0;
}

static int wave420l_encoder_cmd(struct file *file, void *priv,
				struct v4l2_encoder_cmd *cmd)
{
	struct wave420l_instance *inst = v4l2_fh_to_inst(file);
	int ret;

	ret = v4l2_m2m_ioctl_try_encoder_cmd(file, priv, cmd);
	if (ret)
		return ret;

	ret = v4l2_m2m_ioctl_encoder_cmd(file, &inst->fh, cmd);
	if (ret)
		return ret;

	/*
	 * On STOP command, signal end-of-stream to the encoder firmware
	 * by issuing ENC_PIC with srcPicIdx=0xFFFFFFFF.  This flushes
	 * any buffered B-frames from the firmware reorder pipeline.
	 *
	 * The flush runs after v4l2_m2m_ioctl_encoder_cmd() so that the
	 * M2M context is already in drain state when we deliver flushed
	 * frames and signal EOS.  For IPP-only GOP (bframe_delay==0) the
	 * firmware returns immediately with no output, and the normal
	 * device_run drain path handles EOS.
	 */
	if (cmd->cmd == V4L2_ENC_CMD_STOP &&
	    inst->state >= WAVE420L_STATE_INIT_SEQ) {
		int flush_ret = wave420l_hw_flush_instance(inst);

		if (flush_ret)
			dev_dbg(inst->dev->dev,
				"flush_instance: %d (continuing drain)\n",
				flush_ret);
	}

	return 0;
}

static int wave420l_try_encoder_cmd(struct file *file, void *priv,
				    struct v4l2_encoder_cmd *cmd)
{
	return v4l2_m2m_ioctl_try_encoder_cmd(file, priv, cmd);
}

static int wave420l_subscribe_event(struct v4l2_fh *fh,
				    const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 0, NULL);
	case V4L2_EVENT_CTRL:
		return v4l2_ctrl_subscribe_event(fh, sub);
	default:
		return -EINVAL;
	}
}

static const struct v4l2_ioctl_ops wave420l_ioctl_ops = {
	.vidioc_querycap		= wave420l_querycap,

	.vidioc_enum_fmt_vid_out	= wave420l_enum_fmt_out,
	.vidioc_enum_fmt_vid_cap	= wave420l_enum_fmt_cap,

	.vidioc_g_fmt_vid_out_mplane	= wave420l_g_fmt_out,
	.vidioc_g_fmt_vid_cap_mplane	= wave420l_g_fmt_cap,

	.vidioc_try_fmt_vid_out_mplane	= wave420l_try_fmt_out,
	.vidioc_try_fmt_vid_cap_mplane	= wave420l_try_fmt_cap,

	.vidioc_s_fmt_vid_out_mplane	= wave420l_s_fmt_out,
	.vidioc_s_fmt_vid_cap_mplane	= wave420l_s_fmt_cap,

	.vidioc_enum_framesizes		= wave420l_enum_framesizes,

	.vidioc_reqbufs			= v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf		= v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf			= v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf			= v4l2_m2m_ioctl_dqbuf,
	.vidioc_expbuf			= v4l2_m2m_ioctl_expbuf,
	.vidioc_create_bufs		= v4l2_m2m_ioctl_create_bufs,
	.vidioc_prepare_buf		= v4l2_m2m_ioctl_prepare_buf,

	.vidioc_streamon		= v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff		= v4l2_m2m_ioctl_streamoff,

	.vidioc_encoder_cmd		= wave420l_encoder_cmd,
	.vidioc_try_encoder_cmd		= wave420l_try_encoder_cmd,

	.vidioc_subscribe_event		= wave420l_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,
};

/* ------------------------------------------------------------------ */
/* VB2 queue operations                                                */
/* ------------------------------------------------------------------ */

static int wave420l_queue_setup(struct vb2_queue *q, unsigned int *num_buffers,
				unsigned int *num_planes,
				unsigned int sizes[], struct device *alloc_devs[])
{
	struct wave420l_instance *inst = vb2_get_drv_priv(q);
	struct v4l2_pix_format_mplane *fmt;

	if (V4L2_TYPE_IS_OUTPUT(q->type))
		fmt = &inst->src_fmt;
	else
		fmt = &inst->dst_fmt;

	if (*num_planes) {
		unsigned int p;

		if (*num_planes != fmt->num_planes)
			return -EINVAL;
		for (p = 0; p < fmt->num_planes; p++) {
			if (sizes[p] < fmt->plane_fmt[p].sizeimage)
				return -EINVAL;
		}
		return 0;
	}

	*num_planes = fmt->num_planes;
	{
		unsigned int p;

		for (p = 0; p < fmt->num_planes; p++)
			sizes[p] = fmt->plane_fmt[p].sizeimage;
	}

	return 0;
}

static int wave420l_buf_prepare(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct wave420l_instance *inst = vb2_get_drv_priv(vb->vb2_queue);
	struct v4l2_pix_format_mplane *fmt;

	if (V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type))
		fmt = &inst->src_fmt;
	else
		fmt = &inst->dst_fmt;

	if (vb2_plane_size(vb, 0) < fmt->plane_fmt[0].sizeimage)
		return -EINVAL;

	vbuf->field = V4L2_FIELD_NONE;

	return 0;
}

static const struct v4l2_event wave420l_eos_event = {
	.type = V4L2_EVENT_EOS,
};

static void wave420l_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct wave420l_instance *inst = vb2_get_drv_priv(vb->vb2_queue);

	if (V4L2_TYPE_IS_CAPTURE(vb->vb2_queue->type) &&
	    vb2_is_streaming(vb->vb2_queue) &&
	    v4l2_m2m_dst_buf_is_last(inst->fh.m2m_ctx)) {
		unsigned int i;

		for (i = 0; i < vb->num_planes; i++)
			vb2_set_plane_payload(vb, i, 0);

		vbuf->field = V4L2_FIELD_NONE;
		vbuf->sequence = inst->sequence;

		v4l2_m2m_last_buffer_done(inst->fh.m2m_ctx, vbuf);
		v4l2_event_queue_fh(&inst->fh, &wave420l_eos_event);
		return;
	}

	v4l2_m2m_buf_queue(inst->fh.m2m_ctx, vbuf);
}

static int wave420l_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct wave420l_instance *inst = vb2_get_drv_priv(q);
	struct wave420l_dev *dev = inst->dev;
	int ret;

	if (!V4L2_TYPE_IS_OUTPUT(q->type))
		return 0;

	/*
	 * VPU acquire: power on + firmware load + init on first instance,
	 * reference-counted for multi-instance support.
	 */
	ret = wave420l_hw_vpu_acquire(dev);
	if (ret) {
		dev_err(dev->dev, "vpu_acquire failed: %d\n", ret);
		goto err_return_bufs;
	}

	/* Create encoder instance on VPU */
	ret = wave420l_hw_create_instance(inst);
	if (ret) {
		dev_err(dev->dev, "create_instance failed: %d\n", ret);
		goto err_vpu_release;
	}

	/* Initialize encoding sequence parameters */
	ret = wave420l_hw_init_seq(inst);
	if (ret) {
		dev_err(dev->dev, "init_seq failed: %d\n", ret);
		goto err_destroy_inst;
	}

	/*
	 * Set VUI parameters.  If the user set explicit colour controls,
	 * those take priority.  Otherwise, auto-map from the V4L2 colorspace
	 * set on the OUTPUT format.
	 */
	wave420l_map_colorspace_to_vui(inst);
	ret = wave420l_hw_set_vui(inst);
	if (ret) {
		/*
		 * VUI failure is non-fatal: the stream will simply lack
		 * colour signalling in the SPS.  Log and continue.
		 */
		dev_warn(dev->dev, "set_vui failed: %d (continuing)\n", ret);
	}

	/* Allocate and register reference frame buffers */
	ret = wave420l_hw_set_fb(inst);
	if (ret) {
		dev_err(dev->dev, "set_fb failed: %d\n", ret);
		goto err_destroy_inst;
	}

	/*
	 * KI-092: cache VPS+SPS+PPS so the first CAPTURE buffer can prepend
	 * them to be self-contained.  Non-fatal: a stream without parameter
	 * sets is the pre-fix status quo.
	 */
	ret = wave420l_hw_emit_header(inst);
	if (ret) {
		dev_warn(dev->dev,
			 "emit_header failed: %d (output will lack VPS/SPS/PPS)\n",
			 ret);
		inst->header_nal_len = 0;
		ret = 0;
	}

	/*
	 * Allocate CTU map DMA buffer for ROI / CTU QP map.
	 * CTU size = 64x64 pixels; one byte per CTU.
	 * Buffer is zeroed (equal weight for all CTUs) until userspace
	 * writes per-CTU values via mmap or DMABUF.
	 * Non-fatal: if allocation fails, roi_enable and ctu_qp_enable
	 * are cleared and encoding proceeds without per-CTU control.
	 */
	{
		size_t w_ctu = DIV_ROUND_UP(inst->src_fmt.width, 64);
		size_t h_ctu = DIV_ROUND_UP(inst->src_fmt.height, 64);
		size_t map_size = w_ctu * h_ctu;

		if (!inst->ctu_map.virt) {
			inst->ctu_map.virt = dma_alloc_coherent(dev->dev,
								map_size,
								&inst->ctu_map.dma,
								GFP_KERNEL);
			if (inst->ctu_map.virt) {
				inst->ctu_map.size = map_size;
				memset(inst->ctu_map.virt, 0, map_size);
				dev_dbg(dev->dev,
					"CTU map: %zux%zu CTUs, %zu bytes, dma=0x%llx\n",
					w_ctu, h_ctu, map_size,
					(u64)inst->ctu_map.dma);
			} else {
				dev_warn(dev->dev,
					 "CTU map alloc failed (%zu bytes); ROI/CTU-QP disabled\n",
					 map_size);
				inst->roi_enable = false;
				inst->ctu_qp_enable = false;
			}
		}
	}

	inst->state = WAVE420L_STATE_INIT_SEQ;
	inst->sequence = 0;

	/*
	 * Do NOT clear params_changed here -- v4l2-ctl --set-ctrl between
	 * open() and STREAMON sets params_changed AFTER this point.
	 * Instead, device_run skips SET_PARAM on the first frame (sequence==0)
	 * since init_seq already applied all parameters (KI-092).
	 */

	return 0;

err_destroy_inst:
	wave420l_hw_destroy_instance(inst);

err_vpu_release:
	wave420l_hw_vpu_release(dev);
	goto err_return_bufs;

err_return_bufs:
	{
		struct vb2_v4l2_buffer *buf;

		while ((buf = v4l2_m2m_src_buf_remove(inst->fh.m2m_ctx)))
			v4l2_m2m_buf_done(buf, VB2_BUF_STATE_QUEUED);
	}
	return ret;
}

static void wave420l_stop_streaming(struct vb2_queue *q)
{
	struct wave420l_instance *inst = vb2_get_drv_priv(q);
	struct wave420l_dev *dev = inst->dev;
	struct vb2_v4l2_buffer *buf;

	if (V4L2_TYPE_IS_OUTPUT(q->type)) {
		if (inst->state >= WAVE420L_STATE_INIT_SEQ) {
			wave420l_hw_destroy_instance(inst);
			wave420l_hw_vpu_release(dev);
		}
		inst->state = WAVE420L_STATE_OPEN;

		/* Free CTU map buffer on stream stop */
		if (inst->ctu_map.virt) {
			dma_free_coherent(dev->dev, inst->ctu_map.size,
					  inst->ctu_map.virt, inst->ctu_map.dma);
			inst->ctu_map.virt = NULL;
			inst->ctu_map.dma = 0;
			inst->ctu_map.size = 0;
		}

		while ((buf = v4l2_m2m_src_buf_remove(inst->fh.m2m_ctx)))
			v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);
	} else {
		while ((buf = v4l2_m2m_dst_buf_remove(inst->fh.m2m_ctx)))
			v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);
	}
}

static const struct vb2_ops wave420l_vb2_ops = {
	.queue_setup		= wave420l_queue_setup,
	.buf_prepare		= wave420l_buf_prepare,
	.buf_queue		= wave420l_buf_queue,
	.start_streaming	= wave420l_start_streaming,
	.stop_streaming		= wave420l_stop_streaming,
};

/* ------------------------------------------------------------------ */
/* M2M device operations                                               */
/* ------------------------------------------------------------------ */

static bool wave420l_uses_bframes(struct wave420l_instance *inst)
{
	/*
	 * Use bframe_delay set by wave420l_hw_init_seq() which covers all
	 * B-frame GOP presets (IBPBP=1, IBBBP=3).  Before init_seq runs,
	 * fall back to checking gop_preset directly.
	 */
	if (inst->bframe_delay > 0)
		return true;
	/* gop_preset 4 = IBPBP, 5 = IBBBP -- both use B-frames */
	return inst->gop_preset == 4 || inst->gop_preset == 5;
}

static void wave420l_device_run(void *priv)
{
	struct wave420l_instance *inst = priv;
	struct wave420l_dev *dev = inst->dev;
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	const struct wave420l_fmt *src_fmt;
	dma_addr_t src_y, src_u, src_v;
	dma_addr_t bs_addr;
	u32 bs_size, src_stride;
	int ret;

	/*
	 * Apply dynamic parameter changes before encoding.
	 * Skip on the first frame (sequence == 0): init_seq already applied
	 * all parameters.  v4l2-ctl --set-ctrl between open() and STREAMON
	 * re-sets params_changed, but those controls were already consumed
	 * by init_seq.  Issuing SET_PARAM on the first frame would use
	 * OPT_CHANGE_PARAM which resets the firmware's implicit VPS/SPS/PPS
	 * state (KI-092).
	 */
	if (inst->params_changed && inst->sequence > 0) {
		ret = wave420l_hw_set_param_change(inst);
		if (ret)
			dev_warn(dev->dev, "SET_PARAM change failed: %d (continuing)\n", ret);
	} else if (inst->params_changed && inst->sequence == 0) {
		inst->params_changed = 0;
	}

	src_buf = v4l2_m2m_next_src_buf(inst->fh.m2m_ctx);
	dst_buf = v4l2_m2m_next_dst_buf(inst->fh.m2m_ctx);

	if (!src_buf || !dst_buf) {
		dev_err(dev->dev, "missing src or dst buffer\n");
		return;
	}

	src_fmt = wave420l_find_fmt(wave420l_src_fmts,
				    ARRAY_SIZE(wave420l_src_fmts),
				    inst->src_fmt.pixelformat);

	src_stride = inst->src_fmt.plane_fmt[0].bytesperline;
	src_y = vb2_dma_contig_plane_dma_addr(&src_buf->vb2_buf, 0);

	/*
	 * Resolve U/V plane DMA addresses.  Multi-plane (M) variants take
	 * each plane's DMA address directly from the vb2 buffer; single-
	 * plane variants derive U/V by offset from the Y address within
	 * the same contiguous buffer (KI-073).
	 */
	if (src_fmt->num_planes >= 2) {
		src_u = vb2_dma_contig_plane_dma_addr(&src_buf->vb2_buf, 1);
		if (src_fmt->num_planes == 3)
			src_v = vb2_dma_contig_plane_dma_addr(&src_buf->vb2_buf, 2);
		else
			src_v = src_u; /* NV12M / NV21M: interleaved UV in plane 1 */
	} else if (src_fmt->fourcc == V4L2_PIX_FMT_NV12 ||
		   src_fmt->fourcc == V4L2_PIX_FMT_NV21 ||
		   src_fmt->fourcc == V4L2_PIX_FMT_P010) {
		src_u = src_y + src_stride * inst->src_fmt.height;
		src_v = src_u; /* interleaved UV */
	} else {
		/* Single-buffer I420: separate U and V planes within one buf */
		u32 y_size = src_stride * inst->src_fmt.height;
		u32 uv_stride = src_stride / 2;
		u32 uv_size = uv_stride * inst->src_fmt.height / 2;

		src_u = src_y + y_size;
		src_v = src_u + uv_size;
	}

	bs_addr = vb2_dma_contig_plane_dma_addr(&dst_buf->vb2_buf, 0);
	bs_size = vb2_plane_size(&dst_buf->vb2_buf, 0);

	inst->state = WAVE420L_STATE_PIC_RUN;

	/*
	 * PERF-BENCH async device_run: stash the buffer pointers so the IRQ
	 * thread can find them without holding the M2M queue lock.  The
	 * buffers remain in the M2M queue (v4l2_m2m_next_*) until the IRQ
	 * thread removes them with v4l2_m2m_{src,dst}_buf_remove().
	 */
	inst->pending_src_buf = src_buf;
	inst->pending_dst_buf = dst_buf;

	ret = wave420l_hw_start_encode(inst, src_y, src_u, src_v,
				       src_stride, bs_addr, bs_size,
				       inst->sequence);
	if (ret) {
		inst->pending_src_buf = NULL;
		inst->pending_dst_buf = NULL;
		src_buf = v4l2_m2m_src_buf_remove(inst->fh.m2m_ctx);
		dst_buf = v4l2_m2m_dst_buf_remove(inst->fh.m2m_ctx);
		v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_ERROR);
		v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_ERROR);
		v4l2_m2m_job_finish(dev->m2m_dev, inst->fh.m2m_ctx);
		return;
	}

	/*
	 * ENC_PIC command issued; return immediately.  The IRQ thread will
	 * call wave420l_enc_irq_complete() to read the result, complete the
	 * V4L2 buffers, and call v4l2_m2m_job_finish().
	 */
}

/*
 * wave420l_enc_irq_complete - finalize an async encode from the IRQ thread.
 *
 * Called by wave420l_irq_thread() (wave420l-hw.c) when async_enc_pending is
 * set.  Reads encode results, completes src/dst V4L2 buffers, and releases
 * the M2M scheduler so the next frame can be dispatched.
 */
void wave420l_enc_irq_complete(struct wave420l_instance *inst)
{
	struct wave420l_dev *dev = inst->dev;
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	u32 enc_bs_size, pic_type, used_src_idx;
	int ret;

	src_buf = v4l2_m2m_src_buf_remove(inst->fh.m2m_ctx);
	dst_buf = v4l2_m2m_dst_buf_remove(inst->fh.m2m_ctx);

	/*
	 * Sanity: these must match what device_run stashed.  If they differ,
	 * the queue order changed unexpectedly -- log and error-complete.
	 */
	if (WARN_ON(src_buf != inst->pending_src_buf ||
		    dst_buf != inst->pending_dst_buf)) {
		v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_ERROR);
		v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_ERROR);
		v4l2_m2m_job_finish(dev->m2m_dev, inst->fh.m2m_ctx);
		inst->pending_src_buf = NULL;
		inst->pending_dst_buf = NULL;
		return;
	}
	inst->pending_src_buf = NULL;
	inst->pending_dst_buf = NULL;

	ret = wave420l_hw_get_encode_result(inst, &enc_bs_size,
					    &pic_type, &used_src_idx);

	if (ret) {
		v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_ERROR);
		v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_ERROR);
	} else if (enc_bs_size == 0 && wave420l_uses_bframes(inst)) {
		/*
		 * B-frame deferred output: the encoder is buffering
		 * this frame for reordering and has no output yet.
		 * Return the source buffer as done (it has been
		 * consumed by the encoder's reference pool) but
		 * return the capture buffer with zero payload to
		 * signal no output for this frame.
		 */
		dev_dbg(dev->dev, "B-frame deferred: no output for src %u\n",
			inst->sequence);
		vb2_set_plane_payload(&dst_buf->vb2_buf, 0, 0);
		dst_buf->sequence = inst->sequence;
		src_buf->sequence = inst->sequence;
		dst_buf->flags |= V4L2_BUF_FLAG_BFRAME;

		v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_DONE);
		v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_DONE);
	} else {
		/*
		 * Copy encoded bitstream from firmware's DMA buffer to the
		 * V4L2 capture buffer (mmap'd by userspace).  On the first
		 * frame, prepend the cached VPS+SPS+PPS NAL units from
		 * inst->header_nal so the output bitstream is self-
		 * contained (KI-092).
		 */
		{
			u32 hdr_len = (inst->sequence == 0) ?
				      inst->header_nal_len : 0;
			u32 total = hdr_len + enc_bs_size;
			void *dst = vb2_plane_vaddr(&dst_buf->vb2_buf, 0);

			if (enc_bs_size > 0 && dst && inst->bs_buf.virt &&
			    total <= vb2_plane_size(&dst_buf->vb2_buf, 0)) {
				if (hdr_len)
					memcpy(dst, inst->header_nal, hdr_len);
				memcpy(dst + hdr_len, inst->bs_buf.virt,
				       enc_bs_size);
				enc_bs_size = total;
			}
		}
		vb2_set_plane_payload(&dst_buf->vb2_buf, 0, enc_bs_size);
		dst_buf->sequence = inst->sequence;
		src_buf->sequence = inst->sequence;

		/* Copy timestamp, timecode, field, tstamp_src flags */
		v4l2_m2m_buf_copy_metadata(src_buf, dst_buf, false);

		if (pic_type == 0) /* IDR/I frame */
			dst_buf->flags |= V4L2_BUF_FLAG_KEYFRAME;
		else if (pic_type == 2) /* B frame */
			dst_buf->flags |= V4L2_BUF_FLAG_BFRAME;
		else
			dst_buf->flags |= V4L2_BUF_FLAG_PFRAME;

		/*
		 * Signal drain completion: mark the capture buffer
		 * with LAST flag if this was the final source buffer.
		 */
		dst_buf->flags &= ~V4L2_BUF_FLAG_LAST;
		if (v4l2_m2m_is_last_draining_src_buf(inst->fh.m2m_ctx,
						      src_buf)) {
			dst_buf->flags |= V4L2_BUF_FLAG_LAST;
			v4l2_event_queue_fh(&inst->fh,
					    &wave420l_eos_event);
			v4l2_m2m_mark_stopped(inst->fh.m2m_ctx);
		}

		v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_DONE);
		v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_DONE);
	}

	inst->sequence++;
	inst->state = WAVE420L_STATE_INIT_SEQ;

	v4l2_m2m_job_finish(dev->m2m_dev, inst->fh.m2m_ctx);
}

/*
 * Called by v4l2_m2m_cancel_job with TRANS_RUNNING set, from TWO
 * distinct entry points with DIFFERENT lock state:
 *
 *   1. wave420l_release -> v4l2_m2m_ctx_release: the close() fop is
 *      NOT wrapped by vdev->lock, so dev_lock is NOT held here.
 *   2. VIDIOC_STREAMOFF -> v4l2_m2m_streamoff: video_ioctl2 takes
 *      vdev->lock (== dev->dev_lock, see wave420l_queue_init and the
 *      vdev->lock assignment in probe) BEFORE dispatching the ioctl,
 *      so dev_lock IS ALREADY HELD by the calling task here.
 *
 * The IRQ thread normally clears TRANS_RUNNING via
 * wave420l_enc_irq_complete -> v4l2_m2m_job_finish; we wait briefly
 * for that to happen.  If the firmware fails to fire IRQ within the
 * timeout (a LIVE, never-FLUSHed encode aborted mid-flight, a hung
 * encode, or unbind-during-encode), force-complete the job here so
 * v4l2_m2m_cancel_job's TRANS_RUNNING wait can wake.  Without this,
 * an in-flight async ENC_PIC whose IRQ never lands leaves the
 * m2m_ctx wedged forever -- the root cause of KI-018's
 * unbind-during-encode oops and KI-020's 768 KB-per-leaked-session
 * CMA leak.
 *
 * Lock choice -- MUST NOT be dev_lock.  active_inst and
 * async_enc_pending are owned by hw_lock everywhere else (the async
 * ENC_PIC issuer in wave420l_hw_start_encode and the sync paths in
 * wave420l-hw.c all guard them under hw_lock, never dev_lock), so
 * hw_lock is the correct lock for this critical section regardless.
 * Critically, taking dev_lock here SELF-DEADLOCKS the STREAMOFF path
 * (entry point 2): the task already holds dev_lock, so the
 * non-recursive mutex_lock spins in __mutex_lock_slowpath forever
 * (D-state; hung_task fires every ~122 s; only a reboot clears it).
 * This is exactly the trap the dev_lock-aliasing note at
 * wave420l_queue_init warns about, and is the same class of failure
 * (dev_lock reused on a firmware/teardown path) that mainline
 * 873a93f6f73b had to undo for the FLUSH IRQ thread.  hw_lock is
 * never held across a wait (the async issuer drops it before
 * returning; flush_one drops it before its irq_done wait), so taking
 * it here cannot deadlock the IRQ thread or any encode in flight.
 */
static void wave420l_job_abort(void *priv)
{
	struct wave420l_instance *inst = priv;
	struct wave420l_dev *dev = inst->dev;

	if (wait_for_completion_timeout(&inst->irq_done,
					msecs_to_jiffies(50)))
		return;

	dev_warn(dev->dev,
		 "job_abort: firmware IRQ timeout, forcing job_finish\n");

	mutex_lock(&dev->hw_lock);
	if (dev->active_inst == inst)
		dev->active_inst = NULL;
	if (xchg(&inst->async_enc_pending, false)) {
		/* Drop the pm_runtime ref hw_start_encode took; the IRQ
		 * never fired, so wave420l_hw_get_encode_result -- which
		 * normally calls pm_runtime_put_autosuspend -- will not run.
		 */
		pm_runtime_put_autosuspend(dev->dev);
		/* ...and the command slot it took (the IRQ thread would
		 * otherwise release it; the xchg makes exactly one of us).
		 */
		wave420l_hw_cmd_slot_put(dev);
	}
	mutex_unlock(&dev->hw_lock);

	/*
	 * Error-complete the in-flight buffers.  The async device_run path
	 * leaves them in the M2M queue (v4l2_m2m_next_*) for the IRQ thread
	 * to remove; since the IRQ never fired, we remove them HERE before
	 * v4l2_m2m_buf_done -- exactly as wave420l_enc_irq_complete does with
	 * v4l2_m2m_{src,dst}_buf_remove.  Skipping the remove leaves the
	 * buffers queued, and the subsequent vb2 stop_streaming (which
	 * v4l2_m2m_streamoff runs right after cancel_job) then removes +
	 * completes them a SECOND time -- a double vb2_buffer_done that trips
	 * WARN_ON at videobuf2-core.c:1187.  Latent until this fix made the
	 * force-finish path actually reachable (it previously self-deadlocked
	 * on dev_lock before getting here).
	 */
	if (inst->pending_src_buf) {
		v4l2_m2m_src_buf_remove(inst->fh.m2m_ctx);
		v4l2_m2m_buf_done(inst->pending_src_buf, VB2_BUF_STATE_ERROR);
		inst->pending_src_buf = NULL;
	}
	if (inst->pending_dst_buf) {
		v4l2_m2m_dst_buf_remove(inst->fh.m2m_ctx);
		v4l2_m2m_buf_done(inst->pending_dst_buf, VB2_BUF_STATE_ERROR);
		inst->pending_dst_buf = NULL;
	}

	v4l2_m2m_job_finish(dev->m2m_dev, inst->fh.m2m_ctx);
}

static const struct v4l2_m2m_ops wave420l_m2m_ops = {
	.device_run = wave420l_device_run,
	.job_abort = wave420l_job_abort,
};

/* ------------------------------------------------------------------ */
/* V4L2 controls                                                       */
/* ------------------------------------------------------------------ */

static int wave420l_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct wave420l_instance *inst =
		container_of(ctrl->handler, struct wave420l_instance, ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_MPEG_VIDEO_BITRATE_MODE:
		inst->bitrate_mode = ctrl->val;
		/*
		 * CQ mode: rcEnable=0, firmware uses I-frame QP directly.
		 * CBR/VBR: rcEnable=1, bitrate set via BITRATE control.
		 */
		if (ctrl->val == V4L2_MPEG_VIDEO_BITRATE_MODE_CQ) {
			inst->rc_enable = false;
		} else {
			inst->rc_enable = true;
		}
		inst->params_changed |= W4_ENC_RC_PARAM_CHANGE |
					W4_ENC_RC_TRANS_RATE_CHANGE |
					W4_ENC_RC_TARGET_RATE_CHANGE;
		break;
	case V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE:
		/*
		 * ffmpeg hevc_v4l2m2m sets FRAME_RC_ENABLE=1 for CBR/VBR modes
		 * and =0 for CQP.  This control directly gates rc_enable.
		 * BITRATE_MODE can also control rc_enable (CQ=false, CBR/VBR=true);
		 * whichever control is applied last wins.
		 */
		inst->rc_enable = ctrl->val;
		inst->params_changed |= W4_ENC_RC_PARAM_CHANGE |
					W4_ENC_RC_TRANS_RATE_CHANGE |
					W4_ENC_RC_TARGET_RATE_CHANGE;
		break;
	case V4L2_CID_MPEG_VIDEO_BITRATE:
		inst->bitrate = ctrl->val;
		/*
		 * Dynamic bitrate change: update RC params plus the
		 * transition and target rate registers.  rc_enable is
		 * governed by BITRATE_MODE or FRAME_RC_ENABLE, not this control.
		 */
		inst->params_changed |= W4_ENC_RC_PARAM_CHANGE |
					W4_ENC_RC_TRANS_RATE_CHANGE |
					W4_ENC_RC_TARGET_RATE_CHANGE;
		break;
	case V4L2_CID_MPEG_VIDEO_GOP_SIZE:
		inst->gop_size = ctrl->val;
		inst->params_changed |= W4_ENC_INTRA_PARAM_CHANGE;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_MIN_QP:
		inst->min_qp = ctrl->val;
		inst->params_changed |= W4_ENC_RC_MIN_MAX_QP_CHANGE;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_MAX_QP:
		inst->max_qp = ctrl->val;
		inst->params_changed |= W4_ENC_RC_MIN_MAX_QP_CHANGE;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_I_FRAME_QP:
		inst->init_qp = ctrl->val;
		inst->params_changed |= W4_ENC_INTRA_PARAM_CHANGE;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_PROFILE:
		inst->profile = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_LEVEL:
		inst->level = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_TIER:
		inst->tier = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_MAX_NUM_MERGE_MV_MINUS1:
		inst->max_num_merge = ctrl->val + 1;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_TMV_PREDICTION:
		inst->tmv_prediction = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_INTRA_REFRESH_PERIOD:
		inst->intra_refresh_num = ctrl->val;
		break;
	case WAVE420L_CID_INTRA_REFRESH_MODE:
		inst->intra_refresh_mode = ctrl->val;
		break;
	case WAVE420L_CID_LONG_TERM_REF_ENABLE:
		inst->long_term_ref_enable = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_PREPEND_SPSPPS_TO_IDR:
		inst->prepend_spspps = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME:
		/* Handled per-frame */
		break;
	case V4L2_CID_MPEG_VIDEO_B_FRAMES:
		/*
		 * Map B-frame count to GOP preset index:
		 *   0 B-frames -> preset 2 (IPP)
		 *   1 B-frame  -> preset 4 (IBPBP)
		 */
		if (ctrl->val > 0)
			inst->gop_preset = 4;
		else
			inst->gop_preset = 2;
		/* Keep b_frames for backward compat */
		inst->b_frames = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_LOOP_FILTER_MODE:
		inst->lf_mode = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_LF_BETA_OFFSET_DIV2:
		inst->lf_beta_offset = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_LF_TC_OFFSET_DIV2:
		inst->lf_tc_offset = ctrl->val;
		break;
	case V4L2_CID_ROTATE:
		/*
		 * Hardware rotation via ROT_PARAM[2:1] (rot_angle).
		 * Takes effect at INIT_SEQ (RegisterFramebuffer).
		 * Valid values: 0, 90, 180, 270.
		 */
		inst->rotation = ctrl->val;
		break;
	case V4L2_CID_HFLIP:
		/*
		 * Horizontal mirror via ROT_PARAM[4] (hor_mir).
		 * Takes effect at INIT_SEQ.
		 */
		inst->hflip = ctrl->val;
		break;
	case V4L2_CID_VFLIP:
		/*
		 * Vertical mirror via ROT_PARAM[3] (ver_mir).
		 * Takes effect at INIT_SEQ.
		 */
		inst->vflip = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_VBV_SIZE:
		inst->vbv_size = ctrl->val;
		inst->params_changed |= W4_ENC_RC_PARAM_CHANGE;
		break;
	case V4L2_CID_MPEG_VIDEO_MB_RC_ENABLE:
		inst->cu_rc_enable = ctrl->val;
		inst->params_changed |= W4_ENC_RC_PARAM_CHANGE;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_REFRESH_TYPE:
		inst->refresh_type = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_REFRESH_PERIOD:
		inst->refresh_period = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_P_FRAME_QP:
		inst->p_frame_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_B_FRAME_QP:
		inst->b_frame_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_GENERAL_PB:
		inst->general_pb = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_MAX_PARTITION_DEPTH:
		inst->cu_size_mode = ctrl->val;
		break;
	case V4L2_CID_WAVE420L_NR_Y_ENABLE:
		inst->nr_y_enable = ctrl->val;
		break;
	case V4L2_CID_WAVE420L_NR_CB_ENABLE:
		inst->nr_cb_enable = ctrl->val;
		break;
	case V4L2_CID_WAVE420L_NR_CR_ENABLE:
		inst->nr_cr_enable = ctrl->val;
		break;
	case V4L2_CID_WAVE420L_NR_NOISE_EST_ENABLE:
		inst->nr_noise_est_enable = ctrl->val;
		break;
	case V4L2_CID_WAVE420L_NR_NOISE_SIGMA_Y:
		inst->nr_noise_sigma_y = ctrl->val;
		break;
	case V4L2_CID_WAVE420L_NR_NOISE_SIGMA_CB:
		inst->nr_noise_sigma_cb = ctrl->val;
		break;
	case V4L2_CID_WAVE420L_NR_NOISE_SIGMA_CR:
		inst->nr_noise_sigma_cr = ctrl->val;
		break;
	case V4L2_CID_WAVE420L_CHROMA_CB_QP_OFFSET:
		inst->chroma_cb_qp_offset = ctrl->val;
		break;
	case V4L2_CID_WAVE420L_CHROMA_CR_QP_OFFSET:
		inst->chroma_cr_qp_offset = ctrl->val;
		break;
	case WAVE420L_CID_SCALING_LIST:
		inst->scaling_list_enable = ctrl->val;
		break;
	case WAVE420L_CID_COLOUR_PRIMARIES:
		inst->colour_primaries = ctrl->val;
		break;
	case WAVE420L_CID_TRANSFER_CHARS:
		inst->transfer_characteristics = ctrl->val;
		break;
	case WAVE420L_CID_MATRIX_COEFFS:
		inst->matrix_coefficients = ctrl->val;
		break;
	case WAVE420L_CID_VIDEO_FULL_RANGE:
		inst->video_full_range = ctrl->val;
		break;
	case WAVE420L_CID_SEI_DATA:
		/* SEI data is read directly from ctrl->p_new.p_u8 at encode time */
		break;
	case WAVE420L_CID_TEMPORAL_LAYER_QP_0:
		inst->temporal_layer_qp[0] = (s8)ctrl->val;
		break;
	case WAVE420L_CID_TEMPORAL_LAYER_QP_1:
		inst->temporal_layer_qp[1] = (s8)ctrl->val;
		break;
	case WAVE420L_CID_TEMPORAL_LAYER_QP_2:
		inst->temporal_layer_qp[2] = (s8)ctrl->val;
		break;
	case WAVE420L_CID_FORCE_PIC_TYPE:
		inst->force_pic_type = ctrl->val;
		break;
	case WAVE420L_CID_LOSSLESS:
		inst->lossless = ctrl->val;
		/*
		 * Lossless encoding requires RC disabled (rcEnable must be 0).
		 * When lossless is enabled, force rc_enable=false to satisfy
		 * the firmware constraint.
		 */
		if (inst->lossless)
			inst->rc_enable = false;
		break;
	case V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE:
		inst->slice_mode = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_MB:
		/* Control name says MB but for HEVC we treat the value as CTU count */
		inst->slice_arg = ctrl->val;
		break;
	case WAVE420L_CID_ROI_ENABLE:
		/*
		 * ROI enable: per-CTU importance map (64x64 pixel CTUs).
		 * Hardware requires rcEnable=1 (CBR/VBR mode).
		 * Mutually exclusive with ctu_qp_enable.
		 */
		inst->roi_enable = ctrl->val;
		if (inst->roi_enable)
			inst->ctu_qp_enable = false;
		break;
	case WAVE420L_CID_ROI_DELTA_QP:
		/*
		 * Delta QP applied to all CTUs via ROI map.
		 * Signed 8-bit value, hardware field W4_CMD_ENC_CTU_OPT_PARAM[8:1].
		 */
		inst->roi_delta_qp = (s8)ctrl->val;
		break;
	case WAVE420L_CID_CTU_QP_ENABLE:
		/*
		 * CTU QP map enable: per-CTU absolute QP values.
		 * Hardware requires rcEnable=0 (CQP mode).
		 * Mutually exclusive with roi_enable.
		 */
		inst->ctu_qp_enable = ctrl->val;
		if (inst->ctu_qp_enable)
			inst->roi_enable = false;
		break;
	case WAVE420L_CID_CUSTOM_GOP_ENABLE:
		inst->custom_gop_enable = ctrl->val;
		break;
	case WAVE420L_CID_CUSTOM_GOP_PARAM:
		if (ctrl->p_new.p_u8) {
			if (ctrl->elems * ctrl->elem_size >=
			    sizeof(struct wave420l_custom_gop_param))
				memcpy(&inst->custom_gop, ctrl->p_new.p_u8,
				       sizeof(struct wave420l_custom_gop_param));
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_ctrl_ops wave420l_ctrl_ops = {
	.s_ctrl = wave420l_s_ctrl,
};

static const struct v4l2_ctrl_config wave420l_scaling_list_cfg = {
	.ops = &wave420l_ctrl_ops,
	.id = WAVE420L_CID_SCALING_LIST,
	.name = "HEVC Scaling List",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.max = 1,
	.step = 1,
	.def = 0,
};



/*
 * VUI colour parameter controls.
 * Values follow ITU-T H.273 / ISO 23091-2 code points.
 * Common values:
 *   colour_primaries: 1=BT.709, 6=BT.601, 9=BT.2020
 *   transfer_characteristics: 1=BT.709, 6=BT.601, 13=sRGB, 16=PQ, 18=HLG
 *   matrix_coefficients: 1=BT.709, 6=BT.601, 9=BT.2020
 *   0 = auto (derive from V4L2 colorspace)
 */
static const struct v4l2_ctrl_config wave420l_colour_primaries_cfg = {
	.ops = &wave420l_ctrl_ops,
	.id = WAVE420L_CID_COLOUR_PRIMARIES,
	.name = "HEVC Colour Primaries",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 255,
	.step = 1,
	.def = 0,
};

static const struct v4l2_ctrl_config wave420l_transfer_chars_cfg = {
	.ops = &wave420l_ctrl_ops,
	.id = WAVE420L_CID_TRANSFER_CHARS,
	.name = "HEVC Transfer Characteristics",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 255,
	.step = 1,
	.def = 0,
};

static const struct v4l2_ctrl_config wave420l_matrix_coeffs_cfg = {
	.ops = &wave420l_ctrl_ops,
	.id = WAVE420L_CID_MATRIX_COEFFS,
	.name = "HEVC Matrix Coefficients",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 255,
	.step = 1,
	.def = 0,
};

static const struct v4l2_ctrl_config wave420l_video_full_range_cfg = {
	.ops = &wave420l_ctrl_ops,
	.id = WAVE420L_CID_VIDEO_FULL_RANGE,
	.name = "HEVC Video Full Range",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.max = 1,
	.step = 1,
	.def = 0,
};

static const struct v4l2_ctrl_config wave420l_sei_data_cfg = {
	.ops = &wave420l_ctrl_ops,
	.id = WAVE420L_CID_SEI_DATA,
	.name = "HEVC SEI Data",
	.type = V4L2_CTRL_TYPE_U8,
	.min = 0,
	.max = 255,
	.step = 1,
	.def = 0,
	.dims = { WAVE420L_SEI_BUF_SIZE },
};

static const struct v4l2_ctrl_config wave420l_lossless_cfg = {
	.ops = &wave420l_ctrl_ops,
	.id = WAVE420L_CID_LOSSLESS,
	.name = "HEVC Lossless Encoding",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.max = 1,
	.step = 1,
	.def = 0,
};

static int wave420l_init_ctrls(struct wave420l_instance *inst)
{
	struct v4l2_ctrl_handler *handler = &inst->ctrl_handler;

	v4l2_ctrl_handler_init(handler, 48);

	v4l2_ctrl_new_std(handler, NULL,
			  V4L2_CID_MIN_BUFFERS_FOR_OUTPUT,
			  1, 1, 1, 1);

	/*
	 * BITRATE_MODE: CBR/VBR/CQ selection.  Default is CQ (no rate control)
	 * to match the instance default (rc_enable=false, init_qp=20).
	 * ffmpeg hevc_v4l2m2m sets this via -rc_mode CQ/CBR/VBR.
	 */
	v4l2_ctrl_new_std_menu(handler, &wave420l_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_BITRATE_MODE,
			       V4L2_MPEG_VIDEO_BITRATE_MODE_CQ,
			       0,
			       V4L2_MPEG_VIDEO_BITRATE_MODE_CQ);

	/*
	 * BITRATE default 0 matches the instance default (CQP mode).
	 * When BITRATE_MODE is set to CBR or VBR, userspace must also set
	 * a non-zero bitrate or the firmware will use RC with 0 bps target.
	 */
	v4l2_ctrl_new_std(handler, &wave420l_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_BITRATE,
			  0, 60000000, 1,
			  0);

	/*
	 * FRAME_RC_ENABLE: standard "enable rate control" control.
	 * ffmpeg hevc_v4l2m2m sets this to 1 for CBR/VBR modes.
	 * Setting it to 0 disables RC (CQP mode).
	 * BITRATE_MODE=CQ takes precedence -- FRAME_RC_ENABLE=1 with
	 * BITRATE_MODE=CQ still results in CQP mode.
	 */
	v4l2_ctrl_new_std(handler, &wave420l_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE,
			  0, 1, 1, 0);

	v4l2_ctrl_new_std(handler, &wave420l_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_GOP_SIZE,
			  1, 300, 1,
			  WAVE420L_DEFAULT_GOP);

	v4l2_ctrl_new_std(handler, &wave420l_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_HEVC_MIN_QP,
			  0, 51, 1,
			  WAVE420L_DEFAULT_MIN_QP);

	v4l2_ctrl_new_std(handler, &wave420l_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_HEVC_MAX_QP,
			  0, 51, 1,
			  WAVE420L_DEFAULT_MAX_QP);

	v4l2_ctrl_new_std(handler, &wave420l_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_HEVC_I_FRAME_QP,
			  0, 51, 1,
			  WAVE420L_DEFAULT_QP);

	/*
	 * HEVC profile menu: MAIN (0), MAIN_STILL_PICTURE (1), MAIN_10 (2).
	 * Firmware maps Main=1, MainSP=3, Main10=2 (see wave420l_hw_init_seq).
	 * Hardware confirmed to support Main 10 (SDK product.c:1425).
	 */
	v4l2_ctrl_new_std_menu(handler, &wave420l_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_HEVC_PROFILE,
			       V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10,
			       0,
			       V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN);

	v4l2_ctrl_new_std_menu(handler, &wave420l_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_HEVC_LEVEL,
			       V4L2_MPEG_VIDEO_HEVC_LEVEL_5_1,
			       0,
			       V4L2_MPEG_VIDEO_HEVC_LEVEL_4);

	v4l2_ctrl_new_std_menu(handler, &wave420l_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_HEVC_TIER,
			       V4L2_MPEG_VIDEO_HEVC_TIER_HIGH,
			       0,
			       V4L2_MPEG_VIDEO_HEVC_TIER_MAIN);

	v4l2_ctrl_new_std(handler, &wave420l_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME,
			  0, 0, 0, 0);

	/* Loop filter controls */
	v4l2_ctrl_new_std_menu(handler, &wave420l_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_HEVC_LOOP_FILTER_MODE,
			       V4L2_MPEG_VIDEO_HEVC_LOOP_FILTER_MODE_DISABLED_AT_SLICE_BOUNDARY,
			       0,
			       V4L2_MPEG_VIDEO_HEVC_LOOP_FILTER_MODE_ENABLED);

	v4l2_ctrl_new_std(handler, &wave420l_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_HEVC_LF_BETA_OFFSET_DIV2,
			  -6, 6, 1, 0);

	v4l2_ctrl_new_std(handler, &wave420l_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_HEVC_LF_TC_OFFSET_DIV2,
			  -6, 6, 1, 0);

	/*
	 * VBV buffer size (VBR/CBR only).
	 * Maps to initialDelay[31:20] in W4_CMD_ENC_RC_PARAM.
	 * Firmware interprets this as the CPB initial delay in ms.
	 * SDK default: 3000 ms.  Range 0-3000.  0 = no HRD constraint.
	 */
	v4l2_ctrl_new_std(handler, &wave420l_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_VBV_SIZE,
			  0, 3000, 1, 3000);

	/*
	 * CU-level rate control: enables per-CTU QP adjustment within the
	 * RC framework.  Maps to cuLevelRCEnable[1] in W4_CMD_ENC_RC_PARAM.
	 * Only meaningful when BITRATE_MODE is CBR or VBR.
	 */
	v4l2_ctrl_new_std(handler, &wave420l_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_MB_RC_ENABLE,
			  0, 1, 1, 0);

	/* B-frame control */
	v4l2_ctrl_new_std(handler, &wave420l_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_B_FRAMES,
			  0, 3, 1, 0);

	/* Intra refresh controls */
	v4l2_ctrl_new_std_menu(handler, &wave420l_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_HEVC_REFRESH_TYPE,
			       V4L2_MPEG_VIDEO_HEVC_REFRESH_IDR,
			       0,
			       V4L2_MPEG_VIDEO_HEVC_REFRESH_CRA);

	v4l2_ctrl_new_std(handler, &wave420l_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_HEVC_REFRESH_PERIOD,
			  0, 2047, 1, 0);

	/* P/B frame QP override (0 = disabled, use RC or I-frame QP) */
	v4l2_ctrl_new_std(handler, &wave420l_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_HEVC_P_FRAME_QP,
			  0, 51, 1, 0);

	v4l2_ctrl_new_std(handler, &wave420l_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_HEVC_B_FRAME_QP,
			  0, 51, 1, 0);

	/* General P/B reference structure */
	v4l2_ctrl_new_std(handler, &wave420l_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_HEVC_GENERAL_PB,
			  0, 1, 1, 0);

	/* Max partition depth: 0=32x32 only, 1=+16x16, 2=+8x8 (all) */
	v4l2_ctrl_new_std(handler, &wave420l_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_HEVC_MAX_PARTITION_DEPTH,
			  0, 2, 1, 2);

	/* Noise reduction (pre-encoding filter, vendor-specific controls) */
	{
		static const struct v4l2_ctrl_config nr_ctrls[] = {
			{
				.ops = &wave420l_ctrl_ops,
				.id = V4L2_CID_WAVE420L_NR_Y_ENABLE,
				.name = "Noise Reduction Y Enable",
				.type = V4L2_CTRL_TYPE_BOOLEAN,
				.min = 0, .max = 1, .step = 1, .def = 0,
			},
			{
				.ops = &wave420l_ctrl_ops,
				.id = V4L2_CID_WAVE420L_NR_CB_ENABLE,
				.name = "Noise Reduction Cb Enable",
				.type = V4L2_CTRL_TYPE_BOOLEAN,
				.min = 0, .max = 1, .step = 1, .def = 0,
			},
			{
				.ops = &wave420l_ctrl_ops,
				.id = V4L2_CID_WAVE420L_NR_CR_ENABLE,
				.name = "Noise Reduction Cr Enable",
				.type = V4L2_CTRL_TYPE_BOOLEAN,
				.min = 0, .max = 1, .step = 1, .def = 0,
			},
			{
				.ops = &wave420l_ctrl_ops,
				.id = V4L2_CID_WAVE420L_NR_NOISE_EST_ENABLE,
				.name = "Noise Reduction Auto Estimate",
				.type = V4L2_CTRL_TYPE_BOOLEAN,
				.min = 0, .max = 1, .step = 1, .def = 1,
			},
			{
				.ops = &wave420l_ctrl_ops,
				.id = V4L2_CID_WAVE420L_NR_NOISE_SIGMA_Y,
				.name = "Noise Reduction Sigma Y",
				.type = V4L2_CTRL_TYPE_INTEGER,
				.min = 0, .max = 255, .step = 1, .def = 0,
			},
			{
				.ops = &wave420l_ctrl_ops,
				.id = V4L2_CID_WAVE420L_NR_NOISE_SIGMA_CB,
				.name = "Noise Reduction Sigma Cb",
				.type = V4L2_CTRL_TYPE_INTEGER,
				.min = 0, .max = 255, .step = 1, .def = 0,
			},
			{
				.ops = &wave420l_ctrl_ops,
				.id = V4L2_CID_WAVE420L_NR_NOISE_SIGMA_CR,
				.name = "Noise Reduction Sigma Cr",
				.type = V4L2_CTRL_TYPE_INTEGER,
				.min = 0, .max = 255, .step = 1, .def = 0,
			},
		};
		int i;

		for (i = 0; i < ARRAY_SIZE(nr_ctrls); i++)
			v4l2_ctrl_new_custom(handler, &nr_ctrls[i], NULL);
	}

	/* Chroma QP offsets (PPS-level, HEVC spec range: -12..+12) */
	{
		static const struct v4l2_ctrl_config chroma_qp_ctrls[] = {
			{
				.ops = &wave420l_ctrl_ops,
				.id = V4L2_CID_WAVE420L_CHROMA_CB_QP_OFFSET,
				.name = "HEVC Chroma Cb QP Offset",
				.type = V4L2_CTRL_TYPE_INTEGER,
				.min = -12, .max = 12, .step = 1, .def = 0,
			},
			{
				.ops = &wave420l_ctrl_ops,
				.id = V4L2_CID_WAVE420L_CHROMA_CR_QP_OFFSET,
				.name = "HEVC Chroma Cr QP Offset",
				.type = V4L2_CTRL_TYPE_INTEGER,
				.min = -12, .max = 12, .step = 1, .def = 0,
			},
		};
		int i;

		for (i = 0; i < ARRAY_SIZE(chroma_qp_ctrls); i++)
			v4l2_ctrl_new_custom(handler, &chroma_qp_ctrls[i],
					     NULL);
	}

	/* Max merge MV candidates minus 1 (0-4, default 2 = 3 candidates) */
	v4l2_ctrl_new_std(handler, &wave420l_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_HEVC_MAX_NUM_MERGE_MV_MINUS1,
			  0, 4, 1, 2);

	/* Temporal motion vector prediction enable */
	v4l2_ctrl_new_std(handler, &wave420l_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_HEVC_TMV_PREDICTION,
			  0, 1, 1, 1);

	/* Intra refresh: CTU count per refresh step */
	v4l2_ctrl_new_std(handler, &wave420l_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_INTRA_REFRESH_PERIOD,
			  0, 65535, 1, 0);

	/* Intra refresh mode: 0=none, 1=row, 2=column, 3=step-size */
	{
		struct v4l2_ctrl_config cfg = {
			.ops = &wave420l_ctrl_ops,
			.id = WAVE420L_CID_INTRA_REFRESH_MODE,
			.name = "Intra Refresh Mode",
			.type = V4L2_CTRL_TYPE_INTEGER,
			.min = 0,
			.max = 3,
			.step = 1,
			.def = 0,
		};

		v4l2_ctrl_new_custom(handler, &cfg, NULL);
	}

	/* Long-term reference picture enable */
	{
		struct v4l2_ctrl_config cfg = {
			.ops = &wave420l_ctrl_ops,
			.id = WAVE420L_CID_LONG_TERM_REF_ENABLE,
			.name = "Long Term Reference Enable",
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.min = 0,
			.max = 1,
			.step = 1,
			.def = 0,
		};

		v4l2_ctrl_new_custom(handler, &cfg, NULL);
	}

	/* HEVC Scaling List enable */
	v4l2_ctrl_new_custom(handler, &wave420l_scaling_list_cfg, NULL);

	v4l2_ctrl_new_std(handler, &wave420l_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_PREPEND_SPSPPS_TO_IDR,
			  0, 1, 1, 1);

	v4l2_ctrl_new_std(handler, &wave420l_ctrl_ops,
			  V4L2_CID_ROTATE,
			  0, 270, 90, 0);

	v4l2_ctrl_new_std(handler, &wave420l_ctrl_ops,
			  V4L2_CID_HFLIP,
			  0, 1, 1, 0);

	v4l2_ctrl_new_std(handler, &wave420l_ctrl_ops,
			  V4L2_CID_VFLIP,
			  0, 1, 1, 0);

	/* Temporal layer QP offsets (UND-TEMPORAL-QP):
	 * Three separate controls for layers 0/1/2.  Signed 6-bit range: -32..+31.
	 * Non-zero value on any layer sets enTemporalLayerQp in GOP_PARAM.
	 */
	{
		static const struct v4l2_ctrl_config temporal_qp_ctrls[] = {
			{
				.ops = &wave420l_ctrl_ops,
				.id = WAVE420L_CID_TEMPORAL_LAYER_QP_0,
				.name = "Temporal Layer 0 QP",
				.type = V4L2_CTRL_TYPE_INTEGER,
				.min = -32, .max = 31, .step = 1, .def = 0,
			},
			{
				.ops = &wave420l_ctrl_ops,
				.id = WAVE420L_CID_TEMPORAL_LAYER_QP_1,
				.name = "Temporal Layer 1 QP",
				.type = V4L2_CTRL_TYPE_INTEGER,
				.min = -32, .max = 31, .step = 1, .def = 0,
			},
			{
				.ops = &wave420l_ctrl_ops,
				.id = WAVE420L_CID_TEMPORAL_LAYER_QP_2,
				.name = "Temporal Layer 2 QP",
				.type = V4L2_CTRL_TYPE_INTEGER,
				.min = -32, .max = 31, .step = 1, .def = 0,
			},
		};
		int i;

		for (i = 0; i < ARRAY_SIZE(temporal_qp_ctrls); i++)
			v4l2_ctrl_new_custom(handler, &temporal_qp_ctrls[i], NULL);
	}

	/*
	 * Force picture type (UND-FORCE-PIC):
	 * Per-frame override applied at ENC_PIC_PARAM[22:20].
	 *   0 = auto (firmware decides)
	 *   1 = I/IDR (forceIntra: forcePicTypeEnable + forcePicType=0)
	 *   2 = P     (forcePicTypeEnable + forcePicType=1)
	 *   3 = B     (forcePicTypeEnable + forcePicType=2)
	 * V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME overrides to IDR and takes
	 * precedence if set on the same frame.
	 * Per SDK wave4.c:1184.
	 */
	{
		struct v4l2_ctrl_config cfg = {
			.ops = &wave420l_ctrl_ops,
			.id = WAVE420L_CID_FORCE_PIC_TYPE,
			.name = "Force Picture Type",
			.type = V4L2_CTRL_TYPE_INTEGER,
			.min = 0,
			.max = 3,
			.step = 1,
			.def = 0,
		};

		v4l2_ctrl_new_custom(handler, &cfg, NULL);
	}

	/* Lossless encoding (PIC_PARAM[20]); mutually exclusive with RC */
	v4l2_ctrl_new_custom(handler, &wave420l_lossless_cfg, NULL);

	/* Slice encoding (SET_PARAM independent slice mode/CTU count) */
	v4l2_ctrl_new_std_menu(handler, &wave420l_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE,
			       V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_MAX_MB,
			       0,
			       V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE);

	/* CTU count per slice (used when MULTI_SLICE_MODE=MAX_MB, 1..65535) */
	v4l2_ctrl_new_std(handler, &wave420l_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_MB,
			  1, 65535, 1, 1);

	/* ROI encoding: per-CTU importance map (requires RC mode) */
	{
		static const struct v4l2_ctrl_config roi_ctrls[] = {
			{
				.ops = &wave420l_ctrl_ops,
				.id = WAVE420L_CID_ROI_ENABLE,
				.name = "HEVC ROI Enable",
				.type = V4L2_CTRL_TYPE_BOOLEAN,
				.min = 0, .max = 1, .step = 1, .def = 0,
			},
			{
				.ops = &wave420l_ctrl_ops,
				.id = WAVE420L_CID_ROI_DELTA_QP,
				.name = "HEVC ROI Delta QP",
				.type = V4L2_CTRL_TYPE_INTEGER,
				.min = -31, .max = 31, .step = 1, .def = 0,
			},
			{
				.ops = &wave420l_ctrl_ops,
				.id = WAVE420L_CID_CTU_QP_ENABLE,
				.name = "HEVC CTU QP Map Enable",
				.type = V4L2_CTRL_TYPE_BOOLEAN,
				.min = 0, .max = 1, .step = 1, .def = 0,
			},
		};
		int i;

		for (i = 0; i < ARRAY_SIZE(roi_ctrls); i++)
			v4l2_ctrl_new_custom(handler, &roi_ctrls[i], NULL);
	}

	/*
	 * Custom GOP (UND-CUSTOM-GOP):
	 *
	 * WAVE420L_CID_CUSTOM_GOP_ENABLE: boolean, enables gopPresetIdx=0
	 * and a second SET_PARAM (OPT_CUSTOM_GOP) after INIT_SEQ.
	 *
	 * WAVE420L_CID_CUSTOM_GOP_PARAM: BLOB of sizeof(wave420l_custom_gop_param).
	 * Provides per-picture GOP structure.  If gop_size=0, driver uses a
	 * built-in IPPP default (identical to gopPresetIdx=2).
	 *
	 * Per SDK wave4.c:912-970 and vpuapi.h CustomGopPicParam.
	 */
	{
		struct v4l2_ctrl_config cfg = {
			.ops = &wave420l_ctrl_ops,
			.id = WAVE420L_CID_CUSTOM_GOP_ENABLE,
			.name = "Custom GOP Enable",
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.min = 0,
			.max = 1,
			.step = 1,
			.def = 0,
		};

		v4l2_ctrl_new_custom(handler, &cfg, NULL);
	}

	{
		struct v4l2_ctrl_config cfg = {
			.ops = &wave420l_ctrl_ops,
			.id = WAVE420L_CID_CUSTOM_GOP_PARAM,
			.name = "Custom GOP Parameters",
			.type = V4L2_CTRL_TYPE_U8,
			.min = 0,
			.max = 0xFF,
			.step = 1,
			.def = 0,
			.dims = { sizeof(struct wave420l_custom_gop_param) },
		};

		v4l2_ctrl_new_custom(handler, &cfg, NULL);
	}

	if (handler->error)
		return handler->error;

	return 0;
}

/* ------------------------------------------------------------------ */
/* Queue initialization helper                                         */
/* ------------------------------------------------------------------ */

static int wave420l_queue_init(void *priv, struct vb2_queue *src_vq,
			       struct vb2_queue *dst_vq)
{
	struct wave420l_instance *inst = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = inst;
	src_vq->ops = &wave420l_vb2_ops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->buf_struct_size = sizeof(struct vb2_v4l2_buffer);
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	/*
	 * NOTE: q->lock is aliased to dev->dev_lock here AND on the CAPTURE
	 * queue below AND on vdev->lock at registration time.  Any code path
	 * that holds dev_lock through stop_streaming / wave420l_hw_flush_*
	 * therefore blocks every other dev_lock acquisition for the
	 * duration of the wait.  Do NOT add a new dev_lock acquisition to
	 * wave420l_irq_thread or any other firmware-IRQ path -- that
	 * recreates the FLUSH-ENC_PIC deadlock that mainline `873a93f6f73b`
	 * had to undo.  If a dedicated instances-list serializer is needed,
	 * introduce a separate mutex; do not reuse dev_lock.
	 */
	src_vq->lock = &inst->dev->dev_lock;
	src_vq->dev = inst->dev->dev;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = inst;
	dst_vq->ops = &wave420l_vb2_ops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->buf_struct_size = sizeof(struct vb2_v4l2_buffer);
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &inst->dev->dev_lock;
	dst_vq->dev = inst->dev->dev;

	return vb2_queue_init(dst_vq);
}

/* ------------------------------------------------------------------ */
/* File operations                                                     */
/* ------------------------------------------------------------------ */

static int wave420l_open(struct file *file)
{
	struct wave420l_dev *dev = video_drvdata(file);
	struct wave420l_instance *inst;
	int ret;

	inst = kzalloc(sizeof(*inst), GFP_KERNEL);
	if (!inst)
		return -ENOMEM;

	inst->dev = dev;
	init_completion(&inst->irq_done);

	inst->id = ida_alloc(&dev->inst_ida, GFP_KERNEL);
	if (inst->id < 0) {
		ret = inst->id;
		goto err_free;
	}

	/* Set default formats */
	wave420l_set_default_src_fmt(&inst->src_fmt);
	wave420l_set_default_dst_fmt(&inst->dst_fmt);

	/* Set default encoding parameters */
	inst->bitrate = 0;  /* CQP mode by default -- no rate control */
	inst->gop_size = WAVE420L_DEFAULT_GOP;
	inst->init_qp = WAVE420L_DEFAULT_QP;
	inst->min_qp = WAVE420L_DEFAULT_MIN_QP;
	inst->max_qp = WAVE420L_DEFAULT_MAX_QP;
	inst->profile = V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN;
	inst->level = V4L2_MPEG_VIDEO_HEVC_LEVEL_4;
	inst->tier = V4L2_MPEG_VIDEO_HEVC_TIER_MAIN;
	inst->frame_rate = WAVE420L_DEFAULT_FRAMERATE;
	inst->rc_enable = false;  /* CQP mode: QP controls quality directly */
	inst->lf_mode = V4L2_MPEG_VIDEO_HEVC_LOOP_FILTER_MODE_ENABLED;
	inst->lf_beta_offset = 0;
	inst->lf_tc_offset = 0;
	inst->b_frames = 0;
	inst->refresh_type = V4L2_MPEG_VIDEO_HEVC_REFRESH_CRA;
	inst->refresh_period = 0;  /* 0 = use gop_size as intra period */
	inst->max_num_merge = 3;  /* default: 3 merge candidates */
	inst->tmv_prediction = true;
	inst->intra_refresh_mode = 0;
	inst->intra_refresh_num = 0;
	inst->long_term_ref_enable = false;
	inst->scaling_list_enable = false;
	inst->prepend_spspps = true;
	inst->force_key_frame = false;
	inst->gop_preset = 2;  /* IPP (no B-frames) */
	inst->params_changed = 0;

	/* NR weights: SDK defaults (intra=7, inter=4), sigma starts at 0 (disabled) */
	inst->nr_intra_weight_y = 7;
	inst->nr_intra_weight_cb = 7;
	inst->nr_intra_weight_cr = 7;
	inst->nr_inter_weight_y = 4;
	inst->nr_inter_weight_cb = 4;
	inst->nr_inter_weight_cr = 4;

	/* Rotation/mirror: disabled by default */
	inst->rotation = 0;
	inst->hflip = false;
	inst->vflip = false;

	/* Initialize controls */
	ret = wave420l_init_ctrls(inst);
	if (ret)
		goto err_ida;

	/*
	 * Push control defaults into the driver instance.  Without this,
	 * s_ctrl is never called for controls that keep their default value
	 * (e.g. BITRATE_MODE=CQ), so inst->rc_enable stays at whatever
	 * wave420l_open() set -- which may disagree with the control default.
	 * v4l2_ctrl_handler_setup() calls s_ctrl for every non-volatile
	 * control, ensuring inst->* mirrors the registered defaults.
	 */
	ret = v4l2_ctrl_handler_setup(&inst->ctrl_handler);
	if (ret) {
		dev_err(dev->dev, "ctrl_handler_setup failed: %d\n", ret);
		goto err_ctrl;
	}

	/* Initialize V4L2 file handle */
	v4l2_fh_init(&inst->fh, dev->vdev);
	inst->fh.ctrl_handler = &inst->ctrl_handler;

	/* Create M2M context */
	inst->fh.m2m_ctx = v4l2_m2m_ctx_init(dev->m2m_dev, inst,
					      wave420l_queue_init);
	if (IS_ERR(inst->fh.m2m_ctx)) {
		ret = PTR_ERR(inst->fh.m2m_ctx);
		goto err_fh;
	}

	v4l2_fh_add(&inst->fh, file);

	mutex_lock(&dev->dev_lock);
	list_add_tail(&inst->list, &dev->instances);
	mutex_unlock(&dev->dev_lock);

	inst->state = WAVE420L_STATE_OPEN;

	return 0;

err_fh:
	v4l2_fh_exit(&inst->fh);
err_ctrl:
	v4l2_ctrl_handler_free(&inst->ctrl_handler);
err_ida:
	ida_free(&dev->inst_ida, inst->id);
err_free:
	kfree(inst);
	return ret;
}

static int wave420l_release(struct file *file)
{
	struct wave420l_instance *inst = v4l2_fh_to_inst(file);
	struct wave420l_dev *dev = inst->dev;

	/*
	 * Drain any in-flight m2m job BEFORE removing the instance from
	 * dev->instances.  v4l2_m2m_ctx_release calls v4l2_m2m_cancel_job
	 * which invokes our .job_abort and then waits for TRANS_RUNNING
	 * to clear.  Clearing happens in the IRQ thread's
	 * wave420l_enc_irq_complete path (for dev->active_inst) or in
	 * job_abort; the instance must stay valid until that wait returns,
	 * or the wait hangs forever -- which is exactly the leak/oops path
	 * shared by KI-018 (unbind oops) and KI-020 (768 KB CMA leak).
	 * v4l2_m2m_ctx_release also drives vb2 stop_streaming, which is
	 * what calls wave420l_hw_destroy_instance to free work_buf,
	 * temp_buf, bs_buf, mv_buf, fbc_*_tbl_buf, sub_sam_buf, fb_bufs.
	 */
	v4l2_m2m_ctx_release(inst->fh.m2m_ctx);

	mutex_lock(&dev->dev_lock);
	list_del(&inst->list);
	mutex_unlock(&dev->dev_lock);

	v4l2_fh_del(&inst->fh, file);
	v4l2_fh_exit(&inst->fh);
	v4l2_ctrl_handler_free(&inst->ctrl_handler);

	/* Free CTU map DMA buffer (should already be freed in stop_streaming) */
	if (inst->ctu_map.virt)
		dma_free_coherent(dev->dev, inst->ctu_map.size,
				  inst->ctu_map.virt, inst->ctu_map.dma);

	/* Free SEI DMA buffer */
	if (inst->sei_buf.virt)
		dma_free_coherent(dev->dev, inst->sei_buf.size,
				  inst->sei_buf.virt, inst->sei_buf.dma);

	/* Free encoder auxiliary buffers */
	if (inst->sub_sam_buf.virt)
		dma_free_coherent(dev->dev, inst->sub_sam_buf.size,
				  inst->sub_sam_buf.virt, inst->sub_sam_buf.dma);
	if (inst->fbc_y_tbl_buf.virt)
		dma_free_coherent(dev->dev, inst->fbc_y_tbl_buf.size,
				  inst->fbc_y_tbl_buf.virt, inst->fbc_y_tbl_buf.dma);
	if (inst->fbc_c_tbl_buf.virt)
		dma_free_coherent(dev->dev, inst->fbc_c_tbl_buf.size,
				  inst->fbc_c_tbl_buf.virt, inst->fbc_c_tbl_buf.dma);
	if (inst->mv_buf.virt)
		dma_free_coherent(dev->dev, inst->mv_buf.size,
				  inst->mv_buf.virt, inst->mv_buf.dma);
	if (inst->bs_buf.virt)
		dma_free_coherent(dev->dev, inst->bs_buf.size,
				  inst->bs_buf.virt, inst->bs_buf.dma);
	/*
	 * Defensive frees for work_buf (512 KB) and temp_buf (256 KB).
	 * destroy_instance frees them on the normal STREAMOFF path; this
	 * pair mirrors the same pattern every other inst buffer above
	 * follows.  Without these, the 768 KB-per-leaked-session CMA
	 * leak documented in KI-020 reproduces whenever the normal
	 * destroy_instance path is skipped (e.g., open-without-streamon
	 * close, or any IRQ-race that prevents stop_streaming from
	 * running).
	 */
	if (inst->work_buf.virt)
		dma_free_coherent(dev->dev, inst->work_buf.size,
				  inst->work_buf.virt, inst->work_buf.dma);
	if (inst->temp_buf.virt)
		dma_free_coherent(dev->dev, inst->temp_buf.size,
				  inst->temp_buf.virt, inst->temp_buf.dma);
	if (inst->fb_bufs) {
		int j;

		for (j = 0; j < inst->fb_count; j++) {
			if (inst->fb_bufs[j].virt)
				dma_free_coherent(dev->dev,
						  inst->fb_bufs[j].size,
						  inst->fb_bufs[j].virt,
						  inst->fb_bufs[j].dma);
		}
		kfree(inst->fb_bufs);
	}

	ida_free(&dev->inst_ida, inst->id);
	kfree(inst);

	return 0;
}

static const struct v4l2_file_operations wave420l_fops = {
	.owner		= THIS_MODULE,
	.open		= wave420l_open,
	.release	= wave420l_release,
	.poll		= v4l2_m2m_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= v4l2_m2m_fop_mmap,
};

/* ------------------------------------------------------------------ */
/* Registration                                                        */
/* ------------------------------------------------------------------ */

int wave420l_enc_register(struct wave420l_dev *dev)
{
	struct video_device *vdev;
	int ret;

	dev->m2m_dev = v4l2_m2m_init(&wave420l_m2m_ops);
	if (IS_ERR(dev->m2m_dev)) {
		dev_err(dev->dev, "failed to init M2M device\n");
		return PTR_ERR(dev->m2m_dev);
	}

	vdev = video_device_alloc();
	if (!vdev) {
		ret = -ENOMEM;
		goto err_m2m;
	}

	strscpy(vdev->name, WAVE420L_ENC_NAME, sizeof(vdev->name));
	vdev->fops = &wave420l_fops;
	vdev->ioctl_ops = &wave420l_ioctl_ops;
	vdev->release = video_device_release;
	vdev->v4l2_dev = &dev->v4l2_dev;
	vdev->vfl_dir = VFL_DIR_M2M;
	vdev->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	vdev->lock = &dev->dev_lock;

	video_set_drvdata(vdev, dev);

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		dev_err(dev->dev, "failed to register video device: %d\n", ret);
		video_device_release(vdev);
		goto err_m2m;
	}

	dev->vdev = vdev;

	return 0;

err_m2m:
	v4l2_m2m_release(dev->m2m_dev);
	return ret;
}

void wave420l_enc_unregister(struct wave420l_dev *dev)
{
	if (dev->vdev) {
		video_unregister_device(dev->vdev);
		dev->vdev = NULL;
	}

	if (dev->m2m_dev) {
		v4l2_m2m_release(dev->m2m_dev);
		dev->m2m_dev = NULL;
	}
}
