/*
* Copyright (c) 2016 MediaTek Inc.
* Author: PC Chen <pc.chen@mediatek.com>
*         Tiffany Lin <tiffany.lin@mediatek.com>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/

#include <media/v4l2-event.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>
#include <soc/mediatek/smi.h>
#include <linux/delay.h>
#include <linux/of.h>

#include "mtk_vcodec_drv.h"
#include "mtk_vcodec_enc.h"
#include "mtk_vcodec_intr.h"
#include "mtk_vcodec_util.h"
#include "venc_drv_if.h"

#define MTK_VENC_MIN_W	160U
#define MTK_VENC_MIN_H	128U
#define MTK_VENC_MAX_W	1280U
#define MTK_VENC_MAX_H	720U
#define MTK_VENC_FHD_MAX_W	1920U
#define MTK_VENC_FHD_MAX_H	1088U
#define DFT_CFG_WIDTH	MTK_VENC_MIN_W
#define DFT_CFG_HEIGHT	MTK_VENC_MIN_H
#define MTK_MAX_CTRLS_HINT	20
#define OUT_FMT_IDX		0
#define CAP_FMT_IDX		8
#define CAP_FMT_IDX_MT8167	4

#define MTK_DEFAULT_FRAMERATE_NUM 1001
#define MTK_DEFAULT_FRAMERATE_DENOM 30000

#define MTK_VENC_CAPABILITY_NOT_AVAILABLE 0xFFFFFFFF

static void mtk_venc_worker(struct work_struct *work);

static struct mtk_video_fmt mtk_video_formats[] = {
	{
		.fourcc = V4L2_PIX_FMT_NV12M,
		.type = MTK_FMT_FRAME,
		.num_planes = 2,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV21M,
		.type = MTK_FMT_FRAME,
		.num_planes = 2,
	},
	{
		.fourcc = V4L2_PIX_FMT_YUV420M,
		.type = MTK_FMT_FRAME,
		.num_planes = 3,
	},
	{
		.fourcc = V4L2_PIX_FMT_YVU420M,
		.type = MTK_FMT_FRAME,
		.num_planes = 3,
	},
	{
		.fourcc = V4L2_PIX_FMT_YUV420,
		.type = MTK_FMT_FRAME,
		.num_planes = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_YVU420,
		.type = MTK_FMT_FRAME,
		.num_planes = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV12,
		.type = MTK_FMT_FRAME,
		.num_planes = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV21,
		.type = MTK_FMT_FRAME,
		.num_planes = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_H264,
		.type = MTK_FMT_ENC,
		.num_planes = 1,
	},
};

#define NUM_FORMATS ARRAY_SIZE(mtk_video_formats)

static struct mtk_video_fmt mtk_video_formats_mt8167[] = {
	{
		.fourcc = V4L2_PIX_FMT_YUV420,
		.type = MTK_FMT_FRAME,
		.num_planes = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_YVU420,
		.type = MTK_FMT_FRAME,
		.num_planes = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV12,
		.type = MTK_FMT_FRAME,
		.num_planes = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV21,
		.type = MTK_FMT_FRAME,
		.num_planes = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_H264,
		.type = MTK_FMT_ENC,
		.num_planes = 1,
	},
};

#define NUM_FORMATS_MT8167 ARRAY_SIZE(mtk_video_formats_mt8167)

static const struct v4l2_frmsize_stepwise mtk_venc_framesizes = {
	MTK_VENC_MIN_W, MTK_VENC_MAX_W, 8,
	MTK_VENC_MIN_H, MTK_VENC_MAX_H, 8,
};

static const struct v4l2_frmsize_stepwise mtk_venc_fhd_framesizes = {
	MTK_VENC_MIN_W, MTK_VENC_FHD_MAX_W, 8,
	MTK_VENC_MIN_H, MTK_VENC_FHD_MAX_H, 8,
};

static int vidioc_venc_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mtk_vcodec_ctx *ctx = ctrl_to_ctx(ctrl);
	struct mtk_enc_params *p = &ctx->enc_params;
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_MPEG_VIDEO_BITRATE:
		mtk_v4l2_debug(2, "V4L2_CID_MPEG_VIDEO_BITRATE val = %d",
			       ctrl->val);
		p->bitrate = ctrl->val;
		ctx->param_change |= MTK_ENCODE_PARAM_BITRATE;
		break;
	case V4L2_CID_MPEG_VIDEO_HEADER_MODE:
		mtk_v4l2_debug(2, "V4L2_CID_MPEG_VIDEO_HEADER_MODE val = %d",
			       ctrl->val);
		p->seq_hdr_mode = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_PROFILE:
		if (ctrl->val == V4L2_MPEG_VIDEO_H264_PROFILE_EXTENDED ||
			ctrl->val == V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE) {
			mtk_v4l2_err("unsupported V4L2_CID_MPEG_VIDEO_H264_PROFILE val = %d",
			       ctrl->val);
			ret = -EINVAL;
		}
		else {
			mtk_v4l2_debug(2, "V4L2_CID_MPEG_VIDEO_H264_PROFILE val = %d",
			       ctrl->val);
			p->h264_profile = ctrl->val;
		}
		break;
	case V4L2_CID_MPEG_VIDEO_H264_LEVEL:
		if (ctrl->val == V4L2_MPEG_VIDEO_H264_LEVEL_1B) {
			mtk_v4l2_err("unsupported V4L2_CID_MPEG_VIDEO_H264_LEVEL val = %d",
			       ctrl->val);
			ret = -EINVAL;
		}
		else {
			mtk_v4l2_debug(2, "V4L2_CID_MPEG_VIDEO_H264_LEVEL val = %d",
			       ctrl->val);
			p->h264_level = ctrl->val;
		}
		break;
	case V4L2_CID_MPEG_VIDEO_H264_I_PERIOD:
		mtk_v4l2_debug(2, "V4L2_CID_MPEG_VIDEO_H264_I_PERIOD val = %d",
			       ctrl->val);
		p->intra_period = ctrl->val;
		ctx->param_change |= MTK_ENCODE_PARAM_INTRA_PERIOD;
		break;
	case V4L2_CID_MPEG_VIDEO_GOP_SIZE:
		mtk_v4l2_debug(2, "V4L2_CID_MPEG_VIDEO_GOP_SIZE val = %d",
			       ctrl->val);
		p->gop_size = ctrl->val;
		ctx->param_change |= MTK_ENCODE_PARAM_GOP_SIZE;
		break;
	case V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME:
		mtk_v4l2_debug(2, "V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME");
		p->force_intra = 1;
		ctx->param_change |= MTK_ENCODE_PARAM_FORCE_INTRA;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops mtk_vcodec_enc_ctrl_ops = {
	.s_ctrl = vidioc_venc_s_ctrl,
};

static int vidioc_enum_fmt(struct mtk_vcodec_ctx *ctx, struct v4l2_fmtdesc *f, bool output_queue)
{
	struct mtk_video_fmt *fmt;
	int i, j = 0;
	mtk_v4l2_debug(2,"vidioc_enum_fmt,output_queue is %d\n",output_queue);

	for (i = 0; i < ctx->num_formats; ++i) {
		if (output_queue && ctx->formats[i].type != MTK_FMT_FRAME)
			continue;
		if (!output_queue && ctx->formats[i].type != MTK_FMT_ENC)
			continue;
		if (j == f->index) {
			fmt = &ctx->formats[i];
			f->pixelformat = fmt->fourcc;
			mtk_v4l2_debug(2,"vidioc_enum_fmt,get index is %d\n",i);
			memset(f->reserved, 0, sizeof(f->reserved));
			return 0;
		}
		++j;
	}

	return -EINVAL;
}

static int vidioc_enum_framesizes(struct file *file, void *priv,
				  struct v4l2_frmsizeenum *fsize)
{
	struct mtk_vcodec_ctx *ctx = fh_to_ctx(file->private_data);
	struct mtk_video_fmt *fmt;
	int k = 0;

	if (!file->private_data)
		return -EINVAL;

	if (fsize->index != 0)
		return -EINVAL;

	for (k = 0; k < ctx->num_formats; k++) {
		fmt = &ctx->formats[k];
		if (fmt->fourcc == fsize->pixel_format)
			break;
	}
	if (k == ctx->num_formats)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	if (ctx->dev->enc_capability == MTK_VENC_CAPABILITY_NOT_AVAILABLE) {
		fsize->stepwise = mtk_venc_framesizes;
	} else {
		fsize->stepwise = mtk_venc_fhd_framesizes;
	}

	return 0;
}

static int vidioc_enum_fmt_vid_cap(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	struct mtk_vcodec_ctx *ctx = fh_to_ctx(file->private_data);

	if (!file->private_data)
		return -EINVAL;

	return vidioc_enum_fmt(ctx, f, false);
}

static int vidioc_enum_fmt_vid_out(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	struct mtk_vcodec_ctx *ctx = fh_to_ctx(file->private_data);

	/* PORT/guard: probe tools (e.g. ffmpeg scanning /dev/video*) can call
	 * this with a NULL fh before the ctx is wired up; guard on priv (a !ctx
	 * test is elided by the compiler) to avoid Oops on ctx->num_formats. */
	if (!file->private_data)
		return -EINVAL;

	mtk_v4l2_debug(2,"vidioc_enum_fmt_vid_out_mplane\n");

	return vidioc_enum_fmt(ctx, f, true);
}

static int vidioc_venc_querycap(struct file *file, void *priv,
				struct v4l2_capability *cap)
{
	strscpy(cap->driver, MTK_VCODEC_ENC_NAME, sizeof(cap->driver));
	strscpy(cap->bus_info, MTK_PLATFORM_STR, sizeof(cap->bus_info));
	strscpy(cap->card, MTK_PLATFORM_STR, sizeof(cap->card));

	return 0;
}

static int vidioc_venc_s_parm(struct file *file, void *priv,
			      struct v4l2_streamparm *a)
{
	struct mtk_vcodec_ctx *ctx = fh_to_ctx(file->private_data);
	struct v4l2_fract *timeperframe = &a->parm.output.timeperframe;
	mtk_v4l2_debug(2,"vidioc_venc_g_parm a->type is %d\n",a->type);

	if (a->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return -EINVAL;

	if (timeperframe->numerator == 0 || timeperframe->denominator == 0) {
		timeperframe->numerator = MTK_DEFAULT_FRAMERATE_NUM;
		timeperframe->denominator = MTK_DEFAULT_FRAMERATE_DENOM;
	}

	ctx->enc_params.framerate_num = timeperframe->denominator;
	ctx->enc_params.framerate_denom = timeperframe->numerator;
	ctx->param_change |= MTK_ENCODE_PARAM_FRAMERATE;

	a->parm.output.capability = V4L2_CAP_TIMEPERFRAME;

	return 0;
}

static int vidioc_venc_g_parm(struct file *file, void *priv,
			      struct v4l2_streamparm *a)
{
	struct mtk_vcodec_ctx *ctx = fh_to_ctx(file->private_data);
	mtk_v4l2_debug(2,"vidioc_venc_g_parm a->type is %d\n",a->type);

	if (a->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return -EINVAL;

	a->parm.output.capability = V4L2_CAP_TIMEPERFRAME;
	a->parm.output.timeperframe.denominator =
			ctx->enc_params.framerate_num;
	a->parm.output.timeperframe.numerator =
			ctx->enc_params.framerate_denom;

	return 0;
}

static struct mtk_q_data *mtk_venc_get_q_data(struct mtk_vcodec_ctx *ctx,
					      enum v4l2_buf_type type)
{
	if (V4L2_TYPE_IS_OUTPUT(type))
		return &ctx->q_data[MTK_Q_DATA_SRC];

	return &ctx->q_data[MTK_Q_DATA_DST];
}

static struct mtk_video_fmt *mtk_venc_find_format(struct mtk_vcodec_ctx *ctx,
						  struct v4l2_format *f)
{
	struct mtk_video_fmt *fmt;
	unsigned int k;

	for (k = 0; k < ctx->num_formats; k++) {
		fmt = &ctx->formats[k];
		if (fmt->fourcc == f->fmt.pix.pixelformat)
			return fmt;
	}

	return NULL;
}

/* V4L2 specification suggests the driver corrects the format struct if any of
 * the dimensions is unsupported
 */
static int vidioc_try_fmt(struct mtk_vcodec_ctx *ctx, struct v4l2_format *f, struct mtk_video_fmt *fmt)
{
	struct v4l2_pix_format_mplane *pix_fmt_mp = &f->fmt.pix_mp;
	int i;

	pix_fmt_mp->field = V4L2_FIELD_NONE;
	mtk_v4l2_debug(2,"vidioc_try_fmt f->type is %d,fmt->num_planes is %d\n",f->type,fmt->num_planes);

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		pix_fmt_mp->num_planes = 1;
		pix_fmt_mp->plane_fmt[0].bytesperline = 0;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		int org_w, org_h;
		unsigned int max_width, max_height;

		if (ctx->dev->enc_capability == MTK_VENC_CAPABILITY_NOT_AVAILABLE) {
			max_width = MTK_VENC_MAX_W;
			max_height = MTK_VENC_MAX_H;
		} else {
			max_width = MTK_VENC_FHD_MAX_W;
			max_height = MTK_VENC_FHD_MAX_H;
		}

		pix_fmt_mp->height = clamp(pix_fmt_mp->height,
					MTK_VENC_MIN_H,
					max_height);
		pix_fmt_mp->width = clamp(pix_fmt_mp->width,
					MTK_VENC_MIN_W,
					max_width);

		/* find next closer width align 16, heign align 32, size align
		 * 64 rectangle
		 */
		org_w = pix_fmt_mp->width;
		org_h = pix_fmt_mp->height;
		v4l_bound_align_image(&pix_fmt_mp->width,
					MTK_VENC_MIN_W,
					max_width, 4,
					&pix_fmt_mp->height,
					MTK_VENC_MIN_H,
					max_height, 4, 6);

		if (pix_fmt_mp->width < org_w &&
			(pix_fmt_mp->width + 16) <= max_width)
			pix_fmt_mp->width += 16;
		if (pix_fmt_mp->height < org_h &&
			(pix_fmt_mp->height + 32) <= max_height)
			pix_fmt_mp->height += 32;

		pix_fmt_mp->num_planes = fmt->num_planes;

		if (pix_fmt_mp->num_planes == 1U) {
			pix_fmt_mp->plane_fmt[0].sizeimage =
				(pix_fmt_mp->width * pix_fmt_mp->height) +
				(pix_fmt_mp->width * pix_fmt_mp->height) / 2 +
				((ALIGN(pix_fmt_mp->width, 16) * 2) * 16);
			pix_fmt_mp->plane_fmt[0].bytesperline =
				pix_fmt_mp->width;
		} else if (pix_fmt_mp->num_planes == 2U) {
			pix_fmt_mp->plane_fmt[0].sizeimage =
				pix_fmt_mp->width * pix_fmt_mp->height +
				((ALIGN(pix_fmt_mp->width, 16) * 2) * 16);
			pix_fmt_mp->plane_fmt[0].bytesperline =
				pix_fmt_mp->width;
			pix_fmt_mp->plane_fmt[1].sizeimage =
				(pix_fmt_mp->width * pix_fmt_mp->height) / 2 +
				(ALIGN(pix_fmt_mp->width, 16) * 16);
			pix_fmt_mp->plane_fmt[1].bytesperline =
				pix_fmt_mp->width;
		} else if (pix_fmt_mp->num_planes == 3U) {
			pix_fmt_mp->plane_fmt[0].sizeimage =
				pix_fmt_mp->width * pix_fmt_mp->height +
				((ALIGN(pix_fmt_mp->width, 16) * 2) * 16);
			pix_fmt_mp->plane_fmt[0].bytesperline =
				pix_fmt_mp->width;
			pix_fmt_mp->plane_fmt[1].sizeimage =
				(pix_fmt_mp->width * pix_fmt_mp->height) / 4 +
				((ALIGN(pix_fmt_mp->width, 16) / 2) * 16);
			pix_fmt_mp->plane_fmt[1].bytesperline =
				pix_fmt_mp->width / 2;
			pix_fmt_mp->plane_fmt[2].sizeimage =
				(pix_fmt_mp->width * pix_fmt_mp->height) / 4 +
				((ALIGN(pix_fmt_mp->width, 16) / 2) * 16);
			pix_fmt_mp->plane_fmt[2].bytesperline =
				pix_fmt_mp->width / 2;
		} else
			mtk_v4l2_err("Unsupport num planes = %d\n",
				pix_fmt_mp->num_planes);

		mtk_v4l2_debug(0,
			"w/h (%d, %d) -> (%d,%d), sizeimage[%d,%d,%d]",
			org_w, org_h,
			pix_fmt_mp->width, pix_fmt_mp->height,
			pix_fmt_mp->plane_fmt[0].sizeimage,
			pix_fmt_mp->plane_fmt[1].sizeimage,
			pix_fmt_mp->plane_fmt[2].sizeimage);
	}

	for (i = 0; i < pix_fmt_mp->num_planes; i++)
		memset(&(pix_fmt_mp->plane_fmt[i].reserved[0]), 0x0,
			   sizeof(pix_fmt_mp->plane_fmt[0].reserved));

	pix_fmt_mp->flags = 0;
	memset(&pix_fmt_mp->reserved, 0x0,
		sizeof(pix_fmt_mp->reserved));

	return 0;
}

static void mtk_venc_set_param(struct mtk_vcodec_ctx *ctx,
				struct venc_enc_param *param)
{
	struct mtk_q_data *q_data_src = &ctx->q_data[MTK_Q_DATA_SRC];
	struct mtk_enc_params *enc_params = &ctx->enc_params;

	switch (q_data_src->fmt->fourcc) {
	case V4L2_PIX_FMT_YUV420M:
	case V4L2_PIX_FMT_YUV420:
		param->input_yuv_fmt = VENC_YUV_FORMAT_I420;
		break;
	case V4L2_PIX_FMT_YVU420M:
	case V4L2_PIX_FMT_YVU420:
		param->input_yuv_fmt = VENC_YUV_FORMAT_YV12;
		break;
	case V4L2_PIX_FMT_NV12M:
	case V4L2_PIX_FMT_NV12:
		param->input_yuv_fmt = VENC_YUV_FORMAT_NV12;
		break;
	case V4L2_PIX_FMT_NV21M:
	case V4L2_PIX_FMT_NV21:
		param->input_yuv_fmt = VENC_YUV_FORMAT_NV21;
		break;
	default:
		mtk_v4l2_err("Unsupport fourcc =%d", q_data_src->fmt->fourcc);
		break;
	}
	param->h264_profile = enc_params->h264_profile;
	param->h264_level = enc_params->h264_level;

	/* Config visible resolution */
	param->width = q_data_src->visible_width;
	param->height = q_data_src->visible_height;
	/* Config coded resolution */
	param->buf_width = q_data_src->coded_width;
	param->buf_height = q_data_src->coded_height;
	param->frm_rate = enc_params->framerate_num /
			enc_params->framerate_denom;
	param->intra_period = enc_params->intra_period;
	param->gop_size = enc_params->gop_size;
	param->bitrate = enc_params->bitrate;

	mtk_v4l2_debug(0,
		"fmt 0x%x, P/L %d/%d, w/h %d/%d, buf %d/%d, fps/bps %d/%d, gop %d, i_period %d",
		param->input_yuv_fmt, param->h264_profile,
		param->h264_level, param->width, param->height,
		param->buf_width, param->buf_height,
		param->frm_rate, param->bitrate,
		param->gop_size, param->intra_period);
}

static int vidioc_venc_s_fmt_cap(struct file *file, void *priv,
			     struct v4l2_format *f)
{
	struct mtk_vcodec_ctx *ctx = fh_to_ctx(file->private_data);
	struct vb2_queue *vq;
	struct mtk_q_data *q_data;
	int i, ret;
	struct mtk_video_fmt *fmt;

	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);
	if (!vq) {
		mtk_v4l2_err("fail to get vq");
		return -EINVAL;
	}

	if (vb2_is_busy(vq)) {
		mtk_v4l2_err("queue busy");
		return -EBUSY;
	}

	q_data = mtk_venc_get_q_data(ctx, f->type);
	if (!q_data) {
		mtk_v4l2_err("fail to get q data");
		return -EINVAL;
	}

	fmt = mtk_venc_find_format(ctx, f);
	if (!fmt) {
		f->fmt.pix.pixelformat = ctx->cap_fmt_default->fourcc;
		fmt = mtk_venc_find_format(ctx, f);
	}
	mtk_v4l2_debug(2,"vidioc_venc_s_fmt_cap fmt->num_planes is %d \n",fmt->num_planes);
	mtk_v4l2_debug(2,"vidioc_venc_s_fmt_cap fmt->fourcc is %d ,V4L2_PIX_FMT_H264 is %d\n",fmt->fourcc,V4L2_PIX_FMT_H264);

	q_data->fmt = fmt;
	ret = vidioc_try_fmt(ctx, f, q_data->fmt);
	if (ret)
		return ret;

	q_data->coded_width = f->fmt.pix_mp.width;
	q_data->coded_height = f->fmt.pix_mp.height;
	q_data->field = f->fmt.pix_mp.field;

	for (i = 0; i < f->fmt.pix_mp.num_planes; i++) {
		struct v4l2_plane_pix_format	*plane_fmt;

		plane_fmt = &f->fmt.pix_mp.plane_fmt[i];
		q_data->bytesperline[i]	= plane_fmt->bytesperline;
		q_data->sizeimage[i] = plane_fmt->sizeimage;
	}

	if (ctx->state == MTK_STATE_FREE) {
		ret = venc_if_init(ctx, q_data->fmt->fourcc);
		if (ret) {
			mtk_v4l2_err("venc_if_init failed=%d, codec type=%x",
					ret, q_data->fmt->fourcc);
			return -EBUSY;
		}
		ctx->state = MTK_STATE_INIT;
	}

	return 0;
}

static int vidioc_venc_s_fmt_out(struct file *file, void *priv,
			     struct v4l2_format *f)
{
	struct mtk_vcodec_ctx *ctx = fh_to_ctx(file->private_data);
	struct vb2_queue *vq;
	struct mtk_q_data *q_data;
	int ret, i;
	struct mtk_video_fmt *fmt;

	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);
	if (!vq) {
		mtk_v4l2_err("fail to get vq");
		return -EINVAL;
	}

	if (vb2_is_busy(vq)) {
		mtk_v4l2_err("queue busy");
		return -EBUSY;
	}

	q_data = mtk_venc_get_q_data(ctx, f->type);
	if (!q_data) {
		mtk_v4l2_err("fail to get q data");
		return -EINVAL;
	}

	fmt = mtk_venc_find_format(ctx, f);
	if (!fmt) {
		f->fmt.pix.pixelformat = ctx->out_fmt_default->fourcc;
		fmt = mtk_venc_find_format(ctx, f);
	}
	mtk_v4l2_debug(2,"vidioc_venc_s_fmt_out fmt->num_planes is %d \n",fmt->num_planes);
	mtk_v4l2_debug(2,"vidioc_venc_s_fmt_out fmt->fourcc is %d\n",fmt->fourcc);
	mtk_v4l2_debug(2,"V4L2_PIX_FMT_YUV420M is %d\n",V4L2_PIX_FMT_YUV420M);
	mtk_v4l2_debug(2,"V4L2_PIX_FMT_YVU420M is %d\n",V4L2_PIX_FMT_YVU420M);

	/* image width / height might be modified with in vidioc_try_fmt() for buffer alignment */
	q_data->visible_width = f->fmt.pix_mp.width;
	q_data->visible_height = f->fmt.pix_mp.height;

	ret = vidioc_try_fmt(ctx, f, fmt);
	if (ret)
		return ret;

	q_data->fmt = fmt;
	q_data->coded_width = f->fmt.pix_mp.width;
	q_data->coded_height = f->fmt.pix_mp.height;

	q_data->field = f->fmt.pix_mp.field;
	ctx->colorspace = f->fmt.pix_mp.colorspace;
	ctx->ycbcr_enc = f->fmt.pix_mp.ycbcr_enc;
	ctx->quantization = f->fmt.pix_mp.quantization;
	ctx->xfer_func = f->fmt.pix_mp.xfer_func;

	for (i = 0; i < f->fmt.pix_mp.num_planes; i++) {
		struct v4l2_plane_pix_format *plane_fmt;

		plane_fmt = &f->fmt.pix_mp.plane_fmt[i];
		q_data->bytesperline[i] = plane_fmt->bytesperline;
		q_data->sizeimage[i] = plane_fmt->sizeimage;
	}

	return 0;
}

static int vidioc_venc_g_fmt(struct file *file, void *priv,
			     struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	struct mtk_vcodec_ctx *ctx = fh_to_ctx(file->private_data);
	struct vb2_queue *vq;
	struct mtk_q_data *q_data;
	int i;

	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	q_data = mtk_venc_get_q_data(ctx, f->type);

	pix->width = q_data->coded_width;
	pix->height = q_data->coded_height;
	pix->pixelformat = q_data->fmt->fourcc;
	pix->field = q_data->field;
	pix->num_planes = q_data->fmt->num_planes;
	for (i = 0; i < pix->num_planes; i++) {
		pix->plane_fmt[i].bytesperline = q_data->bytesperline[i];
		pix->plane_fmt[i].sizeimage = q_data->sizeimage[i];
		memset(&(pix->plane_fmt[i].reserved[0]), 0x0,
		       sizeof(pix->plane_fmt[i].reserved));
	}

	pix->flags = 0;
	pix->colorspace = ctx->colorspace;
	pix->ycbcr_enc = ctx->ycbcr_enc;
	pix->quantization = ctx->quantization;
	pix->xfer_func = ctx->xfer_func;

	return 0;
}

static int vidioc_try_fmt_vid_cap_mplane(struct file *file, void *priv,
					 struct v4l2_format *f)
{
	struct mtk_video_fmt *fmt;
	struct mtk_vcodec_ctx *ctx = fh_to_ctx(file->private_data);

	fmt = mtk_venc_find_format(ctx, f);
	if (!fmt) {
		f->fmt.pix.pixelformat = ctx->cap_fmt_default->fourcc;
		fmt = mtk_venc_find_format(ctx, f);
	}
	f->fmt.pix_mp.colorspace = ctx->colorspace;
	f->fmt.pix_mp.ycbcr_enc = ctx->ycbcr_enc;
	f->fmt.pix_mp.quantization = ctx->quantization;
	f->fmt.pix_mp.xfer_func = ctx->xfer_func;

	return vidioc_try_fmt(ctx, f, fmt);
}

static int vidioc_try_fmt_vid_out_mplane(struct file *file, void *priv,
					 struct v4l2_format *f)
{
	struct mtk_video_fmt *fmt;
	struct mtk_vcodec_ctx *ctx = fh_to_ctx(file->private_data);

	fmt = mtk_venc_find_format(ctx, f);
	if (!fmt) {
		f->fmt.pix.pixelformat = ctx->out_fmt_default->fourcc;
		fmt = mtk_venc_find_format(ctx, f);
	}
	if (!f->fmt.pix_mp.colorspace) {
		f->fmt.pix_mp.colorspace = V4L2_COLORSPACE_REC709;
		f->fmt.pix_mp.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
		f->fmt.pix_mp.quantization = V4L2_QUANTIZATION_DEFAULT;
		f->fmt.pix_mp.xfer_func = V4L2_XFER_FUNC_DEFAULT;
	}

	return vidioc_try_fmt(ctx, f, fmt);
}

static int vidioc_venc_g_selection(struct file *file, void *priv,
				     struct v4l2_selection *s)
{
	struct mtk_vcodec_ctx *ctx = fh_to_ctx(file->private_data);
	struct mtk_q_data *q_data;

	if (!V4L2_TYPE_IS_OUTPUT(s->type))
		return -EINVAL;

	q_data = mtk_venc_get_q_data(ctx, s->type);
	if (!q_data)
		return -EINVAL;

	switch (s->target) {
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		s->r.top = 0;
		s->r.left = 0;
		s->r.width = q_data->coded_width;
		s->r.height = q_data->coded_height;
		break;
	case V4L2_SEL_TGT_CROP:
		s->r.top = 0;
		s->r.left = 0;
		s->r.width = q_data->visible_width;
		s->r.height = q_data->visible_height;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int vidioc_venc_s_selection(struct file *file, void *priv,
				     struct v4l2_selection *s)
{
	struct mtk_vcodec_ctx *ctx = fh_to_ctx(file->private_data);
	struct mtk_q_data *q_data;


	if (!V4L2_TYPE_IS_OUTPUT(s->type))
		return -EINVAL;

	q_data = mtk_venc_get_q_data(ctx, s->type);
	if (!q_data)
		return -EINVAL;

	switch (s->target) {
	case V4L2_SEL_TGT_CROP:
		/* Only support crop from (0,0) */
		s->r.top = 0;
		s->r.left = 0;
		s->r.width = min(s->r.width, q_data->coded_width);
		s->r.height = min(s->r.height, q_data->coded_height);
		q_data->visible_width = s->r.width;
		q_data->visible_height = s->r.height;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int vidioc_venc_qbuf(struct file *file, void *priv,
			    struct v4l2_buffer *buf)
{
	struct mtk_vcodec_ctx *ctx = fh_to_ctx(file->private_data);
	mtk_v4l2_debug(2,"vidioc_venc_qbuf \n");
	if (ctx->state == MTK_STATE_ABORT) {
		mtk_v4l2_err("[%d] Call on QBUF after unrecoverable error",
				ctx->id);
		return -EIO;
	}

	return v4l2_m2m_qbuf(file, ctx->m2m_ctx, buf);
}

static int vidioc_venc_dqbuf(struct file *file, void *priv,
			     struct v4l2_buffer *buf)
{
	struct mtk_vcodec_ctx *ctx = fh_to_ctx(file->private_data);

	if (ctx->state == MTK_STATE_ABORT) {
		mtk_v4l2_err("[%d] Call on QBUF after unrecoverable error",
				ctx->id);
		return -EIO;
	}

	return v4l2_m2m_dqbuf(file, ctx->m2m_ctx, buf);
}

static int vidioc_encoder_cmd(struct file *file, void *priv,
			      struct v4l2_encoder_cmd *cmd)
{
	struct mtk_vcodec_ctx *ctx = fh_to_ctx(file->private_data);
	struct vb2_queue *src_vq, *dst_vq;
	int ret;

	ret = v4l2_m2m_ioctl_try_encoder_cmd(file, priv, cmd);
	if (ret)
		return ret;

	if (!vb2_is_streaming(&ctx->fh.m2m_ctx->cap_q_ctx.q) ||
	    !vb2_is_streaming(&ctx->fh.m2m_ctx->out_q_ctx.q))
		return 0;

	mtk_v4l2_debug(1, "encoder cmd=%u", cmd->cmd);

	switch (cmd->cmd) {
	case V4L2_ENC_CMD_STOP:
		src_vq = v4l2_m2m_get_vq(ctx->m2m_ctx,
					V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
		dst_vq = v4l2_m2m_get_vq(ctx->m2m_ctx,
					V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
		if (!vb2_is_streaming(src_vq)) {
			mtk_v4l2_debug(1, "Output stream is off. No need to flush.");
			return 0;
		}
		if (!vb2_is_streaming(dst_vq)) {
			mtk_v4l2_debug(1, "Capture stream is off. No need to flush.");
			return 0;
		}
		v4l2_m2m_buf_queue(ctx->m2m_ctx, &ctx->empty_flush_buf.vb);
		v4l2_m2m_try_schedule(ctx->m2m_ctx);
		break;

	case V4L2_ENC_CMD_START:
		if (ctx->is_draining)
			return -EBUSY;
		ctx->is_stopped = false;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int vidioc_subscribe_event(struct v4l2_fh *fh,
				  const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 0, NULL);
	default:
		return v4l2_ctrl_subscribe_event(fh, sub);
	}
}

const struct v4l2_ioctl_ops mtk_venc_ioctl_ops = {
	.vidioc_streamon		= v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff		= v4l2_m2m_ioctl_streamoff,

	.vidioc_reqbufs			= v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf		= v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf			= vidioc_venc_qbuf,
	.vidioc_dqbuf			= vidioc_venc_dqbuf,

	.vidioc_querycap		= vidioc_venc_querycap,
	.vidioc_enum_fmt_vid_cap	= vidioc_enum_fmt_vid_cap,
	.vidioc_enum_fmt_vid_out	= vidioc_enum_fmt_vid_out,
	.vidioc_enum_framesizes		= vidioc_enum_framesizes,

	.vidioc_try_fmt_vid_cap_mplane	= vidioc_try_fmt_vid_cap_mplane,
	.vidioc_try_fmt_vid_out_mplane	= vidioc_try_fmt_vid_out_mplane,
	.vidioc_expbuf			= v4l2_m2m_ioctl_expbuf,
	.vidioc_subscribe_event		= vidioc_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,

	.vidioc_s_parm			= vidioc_venc_s_parm,
	.vidioc_g_parm			= vidioc_venc_g_parm,
	.vidioc_s_fmt_vid_cap_mplane	= vidioc_venc_s_fmt_cap,
	.vidioc_s_fmt_vid_out_mplane	= vidioc_venc_s_fmt_out,

	.vidioc_g_fmt_vid_cap_mplane	= vidioc_venc_g_fmt,
	.vidioc_g_fmt_vid_out_mplane	= vidioc_venc_g_fmt,

	.vidioc_create_bufs		= v4l2_m2m_ioctl_create_bufs,
	.vidioc_prepare_buf		= v4l2_m2m_ioctl_prepare_buf,

	.vidioc_g_selection		= vidioc_venc_g_selection,
	.vidioc_s_selection		= vidioc_venc_s_selection,

	.vidioc_encoder_cmd		= vidioc_encoder_cmd,
	.vidioc_try_encoder_cmd		= v4l2_m2m_ioctl_try_encoder_cmd,
};

static int vb2ops_venc_queue_setup(struct vb2_queue *vq,
				   unsigned int *nbuffers,
				   unsigned int *nplanes,
				   unsigned int sizes[],
				   struct device *alloc_ctxs[])
{
	struct mtk_vcodec_ctx *ctx = vb2_get_drv_priv(vq);
	struct mtk_q_data *q_data;
	unsigned int i;
	mtk_v4l2_debug(2,"vb2ops_venc_queue_setup nbuffers is %d,nplanes is %d\n",*nbuffers,*nplanes);

	q_data = mtk_venc_get_q_data(ctx, vq->type);

	if (q_data == NULL)
		return -EINVAL;

	if (*nplanes) {
		for (i = 0; i < *nplanes; i++) {
			if (sizes[i] < q_data->sizeimage[i])
				return -EINVAL;
			alloc_ctxs[i] = ctx->dev->pm.dev;
		}
	} else {
		*nplanes = q_data->fmt->num_planes;
		for (i = 0; i < *nplanes; i++) {
			sizes[i] = q_data->sizeimage[i];
			alloc_ctxs[i] = ctx->dev->pm.dev;
		}
	}

	mtk_v4l2_debug(0, "[%d] nplanes %d sizeimage %d %d %d",
			       ctx->id,
			       *nplanes,
			       q_data->sizeimage[0],
			       q_data->sizeimage[1],
			       q_data->sizeimage[2]);

	return 0;
}


static int vb2ops_venc_buf_prepare(struct vb2_buffer *vb)
{
	struct mtk_vcodec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct mtk_q_data *q_data;
	int i;

	q_data = mtk_venc_get_q_data(ctx, vb->vb2_queue->type);

	vbuf->field = V4L2_FIELD_NONE;

	for (i = 0; i < q_data->fmt->num_planes; i++) {
		if (vb2_plane_size(vb, i) < q_data->sizeimage[i]) {
			mtk_v4l2_err("data will not fit into plane %d (%lu < %d)",
				i, vb2_plane_size(vb, i),
				q_data->sizeimage[i]);
			return -EINVAL;
		}
	}

	return 0;
}

static void vb2ops_venc_buf_queue(struct vb2_buffer *vb)
{
	static const struct v4l2_event eos_event = {
		.type = V4L2_EVENT_EOS
	};
	struct mtk_vcodec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vb2_v4l2 =
			container_of(vb, struct vb2_v4l2_buffer, vb2_buf);

	struct mtk_video_enc_buf *mtk_buf =
			container_of(vb2_v4l2, struct mtk_video_enc_buf, vb);
	mtk_v4l2_debug(2,"vb2ops_venc_buf_queue vb->vb2_queue->type is %d\n",vb->vb2_queue->type );

	if (!V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type) &&
	    vb2_is_streaming(&ctx->fh.m2m_ctx->cap_q_ctx.q) &&
	    ctx->next_is_last) {
		unsigned int i;

		for (i = 0; i < vb->num_planes; i++)
			vb->planes[i].bytesused = 0;
		vb2_v4l2->flags = V4L2_BUF_FLAG_LAST;
		vb2_v4l2->field = V4L2_FIELD_NONE;
		vb2_v4l2->sequence = ctx->sequence_cap++;
		vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
		ctx->is_draining = false;
		ctx->is_stopped = true;
		ctx->next_is_last = false;
		v4l2_event_queue_fh(&ctx->fh, &eos_event);
		return;
	}

	if ((vb->vb2_queue->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) &&
	    (ctx->param_change != MTK_ENCODE_PARAM_NONE)) {
		mtk_v4l2_debug(1, "[%d] Before id=%d encode parameter change %x",
			       ctx->id,
			       mtk_buf->vb.vb2_buf.index,
			       ctx->param_change);
		mtk_buf->param_change = ctx->param_change;
		mtk_buf->enc_params = ctx->enc_params;
		ctx->param_change = MTK_ENCODE_PARAM_NONE;
	}

	v4l2_m2m_buf_queue(ctx->m2m_ctx, to_vb2_v4l2_buffer(vb));
}

static int vb2ops_venc_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct mtk_vcodec_ctx *ctx = vb2_get_drv_priv(q);
	struct venc_enc_param param;
	int ret;
	int i;

	/* Once state turn into MTK_STATE_ABORT, we need stop_streaming
	  * to clear it
	  */
	if (ctx->state == MTK_STATE_ABORT) {
		ret = -EIO;
		goto err_set_param;
	}

	/* If S_FMT hasn't been called, setup VENC IF here */
	if (ctx->state == MTK_STATE_FREE) {
		struct mtk_q_data *q_data = &ctx->q_data[MTK_Q_DATA_DST];

		ret = venc_if_init(ctx, q_data->fmt->fourcc);
		if (ret) {
			mtk_v4l2_err("venc_if_init failed=%d, codec type=%x",
					ret, q_data->fmt->fourcc);
			return -EBUSY;
		}
		ctx->state = MTK_STATE_INIT;
	}

	/* Do the initialization when both start_streaming have been called */
	if (V4L2_TYPE_IS_OUTPUT(q->type)) {
		ctx->sequence_out = 0;
		if (!vb2_start_streaming_called(&ctx->m2m_ctx->cap_q_ctx.q))
			return 0;
	} else {
		ctx->sequence_cap = 0;
		if (!vb2_start_streaming_called(&ctx->m2m_ctx->out_q_ctx.q))
			return 0;
	}
	mtk_v4l2_debug(2,"vb2ops_venc_start_streaming \n");

	mtk_venc_set_param(ctx, &param);
	ret = venc_if_set_param(ctx, VENC_SET_PARAM_ENC, &param);
	if (ret) {
		mtk_v4l2_err("venc_if_set_param failed=%d", ret);
		ctx->state = MTK_STATE_ABORT;
		goto err_set_param;
	}
	ctx->param_change = MTK_ENCODE_PARAM_NONE;

	if ((ctx->q_data[MTK_Q_DATA_DST].fmt->fourcc == V4L2_PIX_FMT_H264) &&
	    (ctx->enc_params.seq_hdr_mode !=
				V4L2_MPEG_VIDEO_HEADER_MODE_SEPARATE)) {
		ret = venc_if_set_param(ctx,
					VENC_SET_PARAM_PREPEND_HEADER,
					NULL);
		if (ret) {
			mtk_v4l2_err("venc_if_set_param failed=%d", ret);
			ctx->state = MTK_STATE_ABORT;
			goto err_set_param;
		}
		ctx->state = MTK_STATE_HEADER;
	}

	if (ctx->oal_vcodec == 1)
		ctx->pend_src_buf = NULL;

	return 0;

err_set_param:
	/*
	 * PORT 5.10->6.18: vb2_queue.num_buffers removed and q->bufs is now a
	 * sparse pointer array; iterate via vb2_get_num_buffers()/
	 * vb2_get_buffer() and skip empty slots.
	 */
	for (i = 0; i < vb2_get_num_buffers(q); ++i) {
		struct vb2_buffer *vb = vb2_get_buffer(q, i);

		if (vb && vb->state == VB2_BUF_STATE_ACTIVE) {
			mtk_v4l2_debug(0, "[%d] id=%d, type=%d, %d -> VB2_BUF_STATE_QUEUED",
					ctx->id, i, q->type,
					(int)vb->state);
			v4l2_m2m_buf_done(to_vb2_v4l2_buffer(vb),
					VB2_BUF_STATE_QUEUED);
		}
	}

	return ret;
}

static void vb2ops_venc_stop_streaming(struct vb2_queue *q)
{
	static const struct v4l2_event eos_event = {
		.type = V4L2_EVENT_EOS
	};
	struct mtk_vcodec_ctx *ctx = vb2_get_drv_priv(q);
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	int ret;
	int i;

	if (V4L2_TYPE_IS_OUTPUT(q->type)) {
		if (ctx->is_draining) {
			struct vb2_v4l2_buffer *next_dst_buf;

			next_dst_buf = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx);
			if (!next_dst_buf)
				ctx->next_is_last = true;
			else {
				next_dst_buf->flags |= V4L2_BUF_FLAG_LAST;
				v4l2_m2m_buf_done(next_dst_buf, VB2_BUF_STATE_DONE);
				v4l2_event_queue_fh(&ctx->fh, &eos_event);
			}
		}
	} else {
		ctx->is_draining = false;
		ctx->is_stopped = false;
		ctx->next_is_last = false;
	}

	mtk_v4l2_debug(2, "[%d]-> type=%d", ctx->id, q->type);

	if (q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		while ((dst_buf = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx))) {
			dst_buf->vb2_buf.planes[0].bytesused = 0;
			v4l2_m2m_buf_done(dst_buf,
					VB2_BUF_STATE_ERROR);
		}
	} else {
		while ((src_buf = v4l2_m2m_src_buf_remove(ctx->m2m_ctx))) {
			if (src_buf != &ctx->empty_flush_buf.vb) {
				v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_ERROR);
			}
		}

		/* PORT 5.10->6.18: sparse q->bufs via vb2_get_buffer() */
		for (i = 0; i < vb2_get_num_buffers(q); ++i) {
			struct vb2_buffer *vb = vb2_get_buffer(q, i);

			if (vb && vb->state == VB2_BUF_STATE_ACTIVE)
				vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
		}
	}

	if ((q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE &&
	     vb2_is_streaming(&ctx->m2m_ctx->out_q_ctx.q)) ||
	    (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE &&
	     vb2_is_streaming(&ctx->m2m_ctx->cap_q_ctx.q))) {
		mtk_v4l2_debug(1, "[%d]-> q type %d out=%d cap=%d",
			       ctx->id, q->type,
			       vb2_is_streaming(&ctx->m2m_ctx->out_q_ctx.q),
			       vb2_is_streaming(&ctx->m2m_ctx->cap_q_ctx.q));
		return;
	}

	/* Release the encoder if both streams are stopped. */
	ret = venc_if_deinit(ctx);
	if (ret)
		mtk_v4l2_err("venc_if_deinit failed=%d", ret);

	ctx->state = MTK_STATE_FREE;
}

static int vb2ops_venc_buf_out_validate(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	vbuf->field = V4L2_FIELD_NONE;
	return 0;
}

static const struct vb2_ops mtk_venc_vb2_ops = {
	.queue_setup		= vb2ops_venc_queue_setup,
	.buf_out_validate	= vb2ops_venc_buf_out_validate,
	.buf_prepare		= vb2ops_venc_buf_prepare,
	.buf_queue		= vb2ops_venc_buf_queue,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
	.start_streaming	= vb2ops_venc_start_streaming,
	.stop_streaming		= vb2ops_venc_stop_streaming,
};

static int mtk_venc_encode_header(void *priv)
{
	struct mtk_vcodec_ctx *ctx = priv;
	int ret;
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	struct mtk_vcodec_mem bs_buf;
	struct venc_done_result enc_result;

	dst_buf = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx);
	if (!dst_buf) {
		mtk_v4l2_debug(1, "No dst buffer");
		return -EINVAL;
	}

	bs_buf.va = vb2_plane_vaddr(&dst_buf->vb2_buf, 0);
	bs_buf.dma_addr = vb2_dma_contig_plane_dma_addr(&dst_buf->vb2_buf, 0);
	bs_buf.size = (size_t)dst_buf->planes[0].length;

	mtk_v4l2_debug(1,
			"[%d] buf id=%d va=0x%p dma_addr=0x%llx size=%zu",
			ctx->id,
			dst_buf->vb2_buf.index, bs_buf.va,
			(u64)bs_buf.dma_addr,
			bs_buf.size);

	ret = venc_if_encode(ctx,
			VENC_START_OPT_ENCODE_SEQUENCE_HEADER,
			NULL, &bs_buf, &enc_result);

	if (ret) {
		dst_buf->vb2_buf.planes[0].bytesused = 0;
		ctx->state = MTK_STATE_ABORT;
		v4l2_m2m_buf_done(dst_buf,
				  VB2_BUF_STATE_ERROR);
		mtk_v4l2_err("venc_if_encode failed=%d", ret);
		return -EINVAL;
	}
	src_buf = v4l2_m2m_next_src_buf(ctx->m2m_ctx);
	if (src_buf) {
		src_buf->sequence = ctx->sequence_out;
		dst_buf->sequence = ctx->sequence_cap++;
		dst_buf->vb2_buf.timestamp = src_buf->vb2_buf.timestamp;
		if (src_buf->flags & V4L2_BUF_FLAG_TIMECODE)
			dst_buf->timecode = src_buf->timecode;
		dst_buf->field = src_buf->field;
		dst_buf->flags |= src_buf->flags &
			(V4L2_BUF_FLAG_TIMECODE |
			 V4L2_BUF_FLAG_KEYFRAME |
			 V4L2_BUF_FLAG_PFRAME |
			 V4L2_BUF_FLAG_BFRAME |
			 V4L2_BUF_FLAG_TSTAMP_SRC_MASK);
	} else {
		mtk_v4l2_err("No timestamp for the header buffer.");
	}

	if (src_buf == &ctx->empty_flush_buf.vb) {
		/* This shouldn't happen. Just in case. */
		mtk_v4l2_err("Invalid flush buffer.");
		v4l2_m2m_src_buf_remove(ctx->m2m_ctx);
		return -EINVAL;
	}

	ctx->state = MTK_STATE_HEADER;
	dst_buf->vb2_buf.planes[0].bytesused = enc_result.bs_size;
	v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_DONE);

	return 0;
}

static int mtk_venc_param_change(struct mtk_vcodec_ctx *ctx)
{
	struct venc_enc_param enc_prm;
	struct vb2_v4l2_buffer *vb2_v4l2 = v4l2_m2m_next_src_buf(ctx->m2m_ctx);
	struct mtk_video_enc_buf *mtk_buf =
			container_of(vb2_v4l2, struct mtk_video_enc_buf, vb);

	int ret = 0;

	memset(&enc_prm, 0, sizeof(enc_prm));
	if (mtk_buf->param_change == MTK_ENCODE_PARAM_NONE)
		return 0;

	if (mtk_buf->param_change & MTK_ENCODE_PARAM_BITRATE) {
		enc_prm.bitrate = mtk_buf->enc_params.bitrate;
		mtk_v4l2_debug(1, "[%d] id=%d, change param br=%d",
				ctx->id,
				mtk_buf->vb.vb2_buf.index,
				enc_prm.bitrate);
		ret |= venc_if_set_param(ctx,
					 VENC_SET_PARAM_ADJUST_BITRATE,
					 &enc_prm);
	}
	if (!ret && mtk_buf->param_change & MTK_ENCODE_PARAM_FRAMERATE) {
		enc_prm.frm_rate = mtk_buf->enc_params.framerate_num /
				   mtk_buf->enc_params.framerate_denom;
		mtk_v4l2_debug(1, "[%d] id=%d, change param fr=%d",
			       ctx->id,
			       mtk_buf->vb.vb2_buf.index,
			       enc_prm.frm_rate);
		ret |= venc_if_set_param(ctx,
					 VENC_SET_PARAM_ADJUST_FRAMERATE,
					 &enc_prm);
	}
	if (!ret && mtk_buf->param_change & MTK_ENCODE_PARAM_GOP_SIZE) {
		enc_prm.gop_size = mtk_buf->enc_params.gop_size;
		mtk_v4l2_debug(1, "change param intra period=%d",
			       enc_prm.gop_size);
		ret |= venc_if_set_param(ctx,
					 VENC_SET_PARAM_GOP_SIZE,
					 &enc_prm);
	}
	if (!ret && mtk_buf->param_change & MTK_ENCODE_PARAM_FORCE_INTRA) {
		mtk_v4l2_debug(1, "[%d] id=%d, change param force I=%d",
				ctx->id,
				mtk_buf->vb.vb2_buf.index,
				mtk_buf->enc_params.force_intra);
		if (mtk_buf->enc_params.force_intra)
			ret |= venc_if_set_param(ctx,
						 VENC_SET_PARAM_FORCE_INTRA,
						 NULL);
	}

	mtk_buf->param_change = MTK_ENCODE_PARAM_NONE;

	if (ret) {
		ctx->state = MTK_STATE_ABORT;
		mtk_v4l2_err("venc_if_set_param %d failed=%d",
				mtk_buf->param_change, ret);
		return -1;
	}

	return 0;
}

/*
 * v4l2_m2m_streamoff() holds dev_mutex and waits mtk_venc_worker()
 * to call v4l2_m2m_job_finish().
 * If mtk_venc_worker() tries to acquire dev_mutex, it will deadlock.
 * So this function must not try to acquire dev->dev_mutex.
 * This means v4l2 ioctls and mtk_venc_worker() can run at the same time.
 * mtk_venc_worker() should be carefully implemented to avoid bugs.
 */
static void mtk_venc_worker(struct work_struct *work)
{
	struct mtk_vcodec_ctx *ctx = container_of(work, struct mtk_vcodec_ctx,
				    encode_work);
	struct mtk_q_data *q_data_src = &ctx->q_data[MTK_Q_DATA_SRC];
	struct vb2_v4l2_buffer *src_buf, *dst_buf, *pend_src_vb2_v4l2, *peek_src_buf;
	struct venc_frm_buf frm_buf;
	struct mtk_vcodec_mem bs_buf;
	struct venc_done_result enc_result;
	int ret, i, length;
	bool is_eos = false;
	char *frame_data;
	static const struct v4l2_event eos_event = {
		.type = V4L2_EVENT_EOS
	};

	/* check dst_buf, dst_buf may be removed in device_run
	 * to stored encdoe header so we need check dst_buf and
	 * call job_finish here to prevent recursion
	 */
	dst_buf = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx);
	if (!dst_buf) {
		v4l2_m2m_job_finish(ctx->dev->m2m_dev_enc, ctx->m2m_ctx);
		return;
	}

	src_buf = v4l2_m2m_src_buf_remove(ctx->m2m_ctx);
	if (src_buf == &ctx->empty_flush_buf.vb) {
		/* This shouldn't happen. Just in case. */
		mtk_v4l2_debug(1, "Got empty flush input buffer.");
		v4l2_m2m_job_finish(ctx->dev->m2m_dev_enc, ctx->m2m_ctx);
		return;
	}

	memset(&frm_buf, 0, sizeof(frm_buf));
	mtk_v4l2_debug(2,"src_buf->vb2_buf.num_planes is %d",src_buf->vb2_buf.num_planes);
	for (i = 0; i < src_buf->vb2_buf.num_planes ; i++) {
		frm_buf.fb_addr[i].va = vb2_plane_vaddr(&src_buf->vb2_buf, i);
		frm_buf.fb_addr[i].dma_addr =
				vb2_dma_contig_plane_dma_addr(&src_buf->vb2_buf, i);
		frm_buf.fb_addr[i].size =
				(size_t)src_buf->planes[i].length;
	}
	frm_buf.num_planes = src_buf->vb2_buf.num_planes;

	bs_buf.va = vb2_plane_vaddr(&dst_buf->vb2_buf, 0);
	bs_buf.dma_addr = vb2_dma_contig_plane_dma_addr(&dst_buf->vb2_buf, 0);
	bs_buf.size = (size_t)dst_buf->vb2_buf.planes[0].length;

	frame_data = frm_buf.fb_addr[0].va;
	length = q_data_src->coded_width * q_data_src->coded_height;

	mtk_v4l2_debug(2,
			"Framebuf VA=%p PA=%llx Size=0x%zx;VA=%p PA=0x%llx Size=0x%zx;VA=%p PA=0x%llx Size=%zu",
			frm_buf.fb_addr[0].va,
			(u64)frm_buf.fb_addr[0].dma_addr,
			frm_buf.fb_addr[0].size,
			frm_buf.fb_addr[1].va,
			(u64)frm_buf.fb_addr[1].dma_addr,
			frm_buf.fb_addr[1].size,
			frm_buf.fb_addr[2].va,
			(u64)frm_buf.fb_addr[2].dma_addr,
			frm_buf.fb_addr[2].size);

	ret = venc_if_encode(ctx, VENC_START_OPT_ENCODE_FRAME,
			     &frm_buf, &bs_buf, &enc_result);

	if (enc_result.is_key_frm)
		dst_buf->flags |= V4L2_BUF_FLAG_KEYFRAME;

	peek_src_buf = v4l2_m2m_next_src_buf(ctx->m2m_ctx);
	mtk_v4l2_debug(1, "peek src buffer = %p", peek_src_buf);
	if (peek_src_buf && (peek_src_buf == &ctx->empty_flush_buf.vb)) {
		/* remvoe eos buffer */
		peek_src_buf = v4l2_m2m_src_buf_remove(ctx->m2m_ctx);
		is_eos = true;
		mtk_v4l2_debug(1, "GOT EOS EVENT!!");
	}

	if (!ctx->oal_vcodec && is_eos) {
		dst_buf->flags |= V4L2_BUF_FLAG_LAST;
		v4l2_event_queue_fh(&ctx->fh, &eos_event);
		mtk_v4l2_debug(1, "append V4L2_BUF_FLAG_LAST flag");
	}

	if (ret) {
		dst_buf->vb2_buf.planes[0].bytesused = 0;
		v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_ERROR);
		v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_ERROR);
		mtk_v4l2_err("venc_if_encode failed=%d", ret);
	} else {
		if (ctx->oal_vcodec == 1) {
		    if (ctx->pend_src_buf) {
				pend_src_vb2_v4l2 = to_vb2_v4l2_buffer(ctx->pend_src_buf);
				dst_buf->vb2_buf.timestamp = ctx->pend_src_buf->timestamp;
				if (pend_src_vb2_v4l2->flags & V4L2_BUF_FLAG_TIMECODE)
					dst_buf->timecode = pend_src_vb2_v4l2->timecode;
				dst_buf->field = pend_src_vb2_v4l2->field;
				dst_buf->flags |= pend_src_vb2_v4l2->flags &
					(V4L2_BUF_FLAG_TIMECODE |
					 V4L2_BUF_FLAG_KEYFRAME |
					 V4L2_BUF_FLAG_PFRAME |
					 V4L2_BUF_FLAG_BFRAME |
					 V4L2_BUF_FLAG_TSTAMP_SRC_MASK);

				pend_src_vb2_v4l2->sequence = ctx->sequence_out++;
				dst_buf->sequence = ctx->sequence_cap++;
				dst_buf->vb2_buf.planes[0].bytesused = enc_result.bs_size;
				v4l2_m2m_buf_done(pend_src_vb2_v4l2,
					  VB2_BUF_STATE_DONE);
				v4l2_m2m_buf_done(dst_buf,
					  VB2_BUF_STATE_DONE);

				if (is_eos) {
					dst_buf = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx);
					if (!dst_buf) {
						mtk_v4l2_err("missing dst buf got last venc_if_encode failed=%d", ret);
						return;
					}

					bs_buf.va = vb2_plane_vaddr(&dst_buf->vb2_buf, 0);
					bs_buf.dma_addr = vb2_dma_contig_plane_dma_addr(&dst_buf->vb2_buf, 0);
					bs_buf.size = (size_t)dst_buf->vb2_buf.planes[0].length;

					ret = venc_if_encode(ctx, VENC_START_OPT_ENCODE_FRAME_FINAL,
							NULL, &bs_buf, &enc_result);

					if (enc_result.is_key_frm)
						dst_buf->flags |= V4L2_BUF_FLAG_KEYFRAME;

					dst_buf->flags |= V4L2_BUF_FLAG_LAST;
					v4l2_event_queue_fh(&ctx->fh, &eos_event);

					dst_buf->vb2_buf.timestamp = src_buf->vb2_buf.timestamp;
					if (src_buf->flags & V4L2_BUF_FLAG_TIMECODE)
						dst_buf->timecode = src_buf->timecode;
					dst_buf->field = src_buf->field;
					dst_buf->flags |= src_buf->flags &
						(V4L2_BUF_FLAG_TIMECODE |
						 V4L2_BUF_FLAG_KEYFRAME |
						 V4L2_BUF_FLAG_PFRAME |
						 V4L2_BUF_FLAG_BFRAME |
						 V4L2_BUF_FLAG_TSTAMP_SRC_MASK);

					src_buf->sequence = ctx->sequence_out++;
					dst_buf->sequence = ctx->sequence_cap++;

					if (enc_result.is_key_frm)
						dst_buf->flags |= V4L2_BUF_FLAG_KEYFRAME;

					if (ret) {
						dst_buf->vb2_buf.planes[0].bytesused = 0;
						v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_ERROR);
						v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_ERROR);
						mtk_v4l2_err("last venc_if_encode failed=%d", ret);
					} else {
						dst_buf->vb2_buf.planes[0].bytesused = enc_result.bs_size;
						v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_DONE);
						v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_DONE);
					}

					ctx->pend_src_buf = NULL;
				} else
					/* pending current src buf for hybrid encoder */
					ctx->pend_src_buf = &src_buf->vb2_buf;
		    } else {
				/* for hybrid encoder, first src buffer will not be
				 * encoded in the first encode cycle, so put the
				 * first dst buffer to empty queue
				 */
		        ctx->pend_src_buf = &src_buf->vb2_buf;
				v4l2_m2m_buf_queue(ctx->m2m_ctx, dst_buf);
		    }
		} else {
			dst_buf->vb2_buf.timestamp = src_buf->vb2_buf.timestamp;
			if (src_buf->flags & V4L2_BUF_FLAG_TIMECODE)
				dst_buf->timecode = src_buf->timecode;
			dst_buf->field = src_buf->field;
			dst_buf->flags |= src_buf->flags &
				(V4L2_BUF_FLAG_TIMECODE |
				 V4L2_BUF_FLAG_KEYFRAME |
				 V4L2_BUF_FLAG_PFRAME |
				 V4L2_BUF_FLAG_BFRAME |
				 V4L2_BUF_FLAG_TSTAMP_SRC_MASK);

			src_buf->sequence = ctx->sequence_out++;
			dst_buf->sequence = ctx->sequence_cap++;
			dst_buf->vb2_buf.planes[0].bytesused = enc_result.bs_size;
			v4l2_m2m_buf_done(src_buf,
					VB2_BUF_STATE_DONE);
			v4l2_m2m_buf_done(dst_buf,
					VB2_BUF_STATE_DONE);
		}

		mtk_v4l2_debug(2, "venc_if_encode bs size=%d",
				 enc_result.bs_size);
	}

	v4l2_m2m_job_finish(ctx->dev->m2m_dev_enc, ctx->m2m_ctx);

	mtk_v4l2_debug(1, "<=== src_buf[%d] dst_buf[%d] venc_if_encode ret=%d Size=%u===>",
			src_buf->vb2_buf.index, dst_buf->vb2_buf.index, ret,
			enc_result.bs_size);
}

static void m2mops_venc_device_run(void *priv)
{
	struct mtk_vcodec_ctx *ctx = priv;

	mtk_venc_param_change(ctx);

	if ((ctx->q_data[MTK_Q_DATA_DST].fmt->fourcc == V4L2_PIX_FMT_H264) &&
	    (ctx->state != MTK_STATE_HEADER)) {
		/* encode h264 sps/pps header */
		mtk_venc_encode_header(ctx);
		queue_work(ctx->dev->encode_workqueue, &ctx->encode_work);
		return;
	}

	queue_work(ctx->dev->encode_workqueue, &ctx->encode_work);
}

static int m2mops_venc_job_ready(void *m2m_priv)
{
	struct mtk_vcodec_ctx *ctx = m2m_priv;

	if (ctx->is_stopped)
		return 0;

	if (ctx->state == MTK_STATE_ABORT || ctx->state == MTK_STATE_FREE) {
		mtk_v4l2_debug(4, "[%d]Not ready: state=0x%x.",
			       ctx->id, ctx->state);
		return 0;
	}

	return 1;
}

static void m2mops_venc_job_abort(void *priv)
{
	struct mtk_vcodec_ctx *ctx = priv;

	ctx->state = MTK_STATE_ABORT;
}

const struct v4l2_m2m_ops mtk_venc_m2m_ops = {
	.device_run	= m2mops_venc_device_run,
	.job_ready	= m2mops_venc_job_ready,
	.job_abort	= m2mops_venc_job_abort,
};

void mtk_vcodec_enc_set_default_params(struct mtk_vcodec_ctx *ctx)
{
	struct mtk_q_data *q_data;

	/* MT8167 only support 1-plane OUTPUT formats */
	if (of_find_compatible_node(NULL, NULL, "mediatek,mt8167-vcodec-enc")) {
		ctx->formats = mtk_video_formats_mt8167;
		ctx->num_formats = NUM_FORMATS_MT8167;
		ctx->out_fmt_default = &mtk_video_formats_mt8167[OUT_FMT_IDX];
		ctx->cap_fmt_default = &mtk_video_formats_mt8167[CAP_FMT_IDX_MT8167];
	} else {
		ctx->formats = mtk_video_formats;
		ctx->num_formats = NUM_FORMATS;
		ctx->out_fmt_default = &mtk_video_formats[OUT_FMT_IDX];
		ctx->cap_fmt_default = &mtk_video_formats[CAP_FMT_IDX];
	}

	ctx->m2m_ctx->q_lock = &ctx->dev->dev_mutex;
	ctx->fh.m2m_ctx = ctx->m2m_ctx;
	ctx->fh.ctrl_handler = &ctx->ctrl_hdl;
	INIT_WORK(&ctx->encode_work, mtk_venc_worker);

	ctx->colorspace = V4L2_COLORSPACE_REC709;
	ctx->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	ctx->quantization = V4L2_QUANTIZATION_DEFAULT;
	ctx->xfer_func = V4L2_XFER_FUNC_DEFAULT;

	q_data = &ctx->q_data[MTK_Q_DATA_SRC];
	memset(q_data, 0, sizeof(struct mtk_q_data));
	q_data->visible_width = DFT_CFG_WIDTH;
	q_data->visible_height = DFT_CFG_HEIGHT;
	q_data->coded_width = DFT_CFG_WIDTH;
	q_data->coded_height = DFT_CFG_HEIGHT;
	q_data->field = V4L2_FIELD_NONE;

	q_data->fmt = ctx->out_fmt_default;

	v4l_bound_align_image(&q_data->coded_width,
				MTK_VENC_MIN_W,
				MTK_VENC_MAX_W, 4,
				&q_data->coded_height,
				MTK_VENC_MIN_H,
				MTK_VENC_MAX_H, 4, 6);

	if (q_data->coded_width < DFT_CFG_WIDTH &&
		(q_data->coded_width + 16) <= MTK_VENC_MAX_W)
		q_data->coded_width += 16;
	if (q_data->coded_height < DFT_CFG_HEIGHT &&
		(q_data->coded_height + 32) <= MTK_VENC_MAX_H)
		q_data->coded_height += 32;

	if (ctx->out_fmt_default->num_planes == 1) {
		q_data->sizeimage[0] =
			(q_data->coded_width * q_data->coded_height) +
			(q_data->coded_width * q_data->coded_height) / 2 +
			((ALIGN(q_data->coded_width, 16) * 2) * 16);
		q_data->bytesperline[0] = q_data->coded_width;
	} else if (ctx->out_fmt_default->num_planes == 2) {
		q_data->sizeimage[0] =
			q_data->coded_width * q_data->coded_height+
			((ALIGN(q_data->coded_width, 16) * 2) * 16);
		q_data->bytesperline[0] = q_data->coded_width;
		q_data->sizeimage[1] =
			(q_data->coded_width * q_data->coded_height) / 2 +
			(ALIGN(q_data->coded_width, 16) * 16);
		q_data->bytesperline[1] = q_data->coded_width;
	} else if (ctx->out_fmt_default->num_planes == 3) {
		q_data->sizeimage[0] =
			q_data->coded_width * q_data->coded_height+
			((ALIGN(q_data->coded_width, 16) * 2) * 16);
		q_data->bytesperline[0] = q_data->coded_width;
		q_data->sizeimage[1] =
			(q_data->coded_width * q_data->coded_height) / 4 +
			(ALIGN(q_data->coded_width, 16) * 16);
		q_data->bytesperline[1] = q_data->coded_width / 2;
		q_data->sizeimage[2] =
			(q_data->coded_width * q_data->coded_height) / 4 +
			(ALIGN(q_data->coded_width, 16) * 16);
		q_data->bytesperline[2] = q_data->coded_width / 2;
	}

	q_data = &ctx->q_data[MTK_Q_DATA_DST];
	memset(q_data, 0, sizeof(struct mtk_q_data));
	q_data->coded_width = DFT_CFG_WIDTH;
	q_data->coded_height = DFT_CFG_HEIGHT;
	q_data->fmt = ctx->cap_fmt_default;
	q_data->field = V4L2_FIELD_NONE;
	ctx->q_data[MTK_Q_DATA_DST].sizeimage[0] =
		DFT_CFG_WIDTH * DFT_CFG_HEIGHT;
	ctx->q_data[MTK_Q_DATA_DST].bytesperline[0] = 0;

	ctx->enc_params.framerate_num = MTK_DEFAULT_FRAMERATE_NUM;
	ctx->enc_params.framerate_denom = MTK_DEFAULT_FRAMERATE_DENOM;

}

int mtk_vcodec_enc_ctrls_setup(struct mtk_vcodec_ctx *ctx)
{
	const struct v4l2_ctrl_ops *ops = &mtk_vcodec_enc_ctrl_ops;
	struct v4l2_ctrl_handler *handler = &ctx->ctrl_hdl;

	v4l2_ctrl_handler_init(handler, MTK_MAX_CTRLS_HINT);

	v4l2_ctrl_new_std(handler, ops, V4L2_CID_MIN_BUFFERS_FOR_OUTPUT, 1, 1, 1, 1);

	v4l2_ctrl_new_std(handler, ops, V4L2_CID_MPEG_VIDEO_BITRATE,
			1, 20000000, 1, 20000000);
	v4l2_ctrl_new_std(handler, ops, V4L2_CID_MPEG_VIDEO_H264_I_PERIOD,
			0, 65535, 1, 0);
	v4l2_ctrl_new_std(handler, ops, V4L2_CID_MPEG_VIDEO_GOP_SIZE,
			0, 65535, 1, 0);
	v4l2_ctrl_new_std(handler, ops, V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME,
			0, 0, 0, 0);
	v4l2_ctrl_new_std_menu(handler, ops,
			V4L2_CID_MPEG_VIDEO_HEADER_MODE,
			V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME,
			0, V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME);
	/*
	 * H264. profile Constrained-baseline / Extended encoding is not supported.
	 */
	v4l2_ctrl_new_std_menu(handler, ops, V4L2_CID_MPEG_VIDEO_H264_PROFILE,
			V4L2_MPEG_VIDEO_H264_PROFILE_HIGH,
			((1 << V4L2_MPEG_VIDEO_H264_PROFILE_EXTENDED) |
			 (1 << V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE)),
			V4L2_MPEG_VIDEO_H264_PROFILE_HIGH);
	/*
	 * H264. level 1B is not supported.
	 */
	v4l2_ctrl_new_std_menu(handler, ops, V4L2_CID_MPEG_VIDEO_H264_LEVEL,
			V4L2_MPEG_VIDEO_H264_LEVEL_4_2,
			(1 << V4L2_MPEG_VIDEO_H264_LEVEL_1B),
			V4L2_MPEG_VIDEO_H264_LEVEL_4_0);
	if (handler->error) {
		mtk_v4l2_err("Init control handler fail %d",
				handler->error);
		return handler->error;
	}

	v4l2_ctrl_handler_setup(&ctx->ctrl_hdl);

	return 0;
}

int mtk_vcodec_enc_queue_init(void *priv, struct vb2_queue *src_vq,
			      struct vb2_queue *dst_vq)
{
	struct mtk_vcodec_ctx *ctx = priv;
	int ret;

	/* Note: VB2_USERPTR works with dma-contig because mt8173
	 * support iommu
	 * https://patchwork.kernel.org/patch/8335461/
	 * https://patchwork.kernel.org/patch/7596181/
	 */
	src_vq->type		= V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes	= VB2_DMABUF | VB2_MMAP | VB2_USERPTR;
	src_vq->drv_priv	= ctx;
	src_vq->buf_struct_size = sizeof(struct mtk_video_enc_buf);
	src_vq->ops		= &mtk_venc_vb2_ops;
	src_vq->mem_ops		= &vb2_dma_contig_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock		= &ctx->dev->dev_mutex;
	src_vq->dev		= &ctx->dev->plat_dev->dev;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type		= V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes	= VB2_DMABUF | VB2_MMAP | VB2_USERPTR;
	dst_vq->drv_priv	= ctx;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->ops		= &mtk_venc_vb2_ops;
	dst_vq->mem_ops		= &vb2_dma_contig_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock		= &ctx->dev->dev_mutex;
	dst_vq->dev		= &ctx->dev->plat_dev->dev;

	return vb2_queue_init(dst_vq);
}

int mtk_venc_unlock(struct mtk_vcodec_ctx *ctx)
{
	mtk_v4l2_debug(4, "ctx %p [%d]", ctx, ctx->id);
	up(&ctx->dev->enc_sem);
	return 0;
}

int mtk_venc_lock(struct mtk_vcodec_ctx *ctx)
{
	unsigned int suspend_block_cnt = 0;

	mtk_v4l2_debug(4, "ctx %p [%d]", ctx, ctx->id);
	return down_interruptible(&ctx->dev->enc_sem);
}

void mtk_vcodec_enc_release(struct mtk_vcodec_ctx *ctx)
{
	int ret = venc_if_deinit(ctx);

	if (ret)
		mtk_v4l2_err("venc_if_deinit failed=%d", ret);

	ctx->state = MTK_STATE_FREE;
}
