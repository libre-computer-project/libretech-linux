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
#include <linux/delay.h>

#include "mtk_vcodec_drv.h"
#include "mtk_vcodec_dec.h"
#include "mtk_vcodec_intr.h"
#include "mtk_vcodec_util.h"
#include "vdec_drv_if.h"
#include "mtk_vcodec_dec_pm.h"

/*
 * PORT 5.10->6.18: mainline only defines the compressed MT21 variant
 * (V4L2_PIX_FMT_MT21C, 'MT21'). The vendor BSP also used the
 * uncompressed V4L2_PIX_FMT_MT21 ('MT21' with a different fourcc),
 * which has no mainline define. Provide it locally to preserve the
 * vendor format list.
 */
#ifndef V4L2_PIX_FMT_MT21
#define V4L2_PIX_FMT_MT21	v4l2_fourcc('M', 'M', '2', '1')
#endif

#define OUT_FMT_IDX	2
#define CAP_FMT_IDX	0

#define MTK_VDEC_MIN_W	64U
#define MTK_VDEC_MIN_H	64U
#define DFT_CFG_WIDTH	MTK_VDEC_MIN_W
#define DFT_CFG_HEIGHT	MTK_VDEC_MIN_H

static struct mtk_video_fmt mtk_video_formats[] = {
	/* PORT: MT8365 VDEC outputs the UNCOMPRESSED MTK tiled NV12 (MM21,
	 * V4L2_PIX_FMT_MT21 here) for non-UFO streams. List it FIRST so userspace
	 * (ffmpeg's v4l2_m2m picks the first supported capture fmt) doesn't pick the
	 * compressed MT21C and misread MM21 data as compressed ("invalid frame").
	 * Validated: gstreamer v4l2h264dec decodes 10/10 frames (422400 B MM21). */
	{
		.fourcc = V4L2_PIX_FMT_MT21,
		.type = MTK_FMT_FRAME,
		.num_planes = 2,
	},
	{
		.fourcc = V4L2_PIX_FMT_MT21C,
		.type = MTK_FMT_FRAME,
		.num_planes = 2,
	},
	{
		.fourcc = V4L2_PIX_FMT_H264,
		.type = MTK_FMT_DEC,
		.num_planes = 1,
		.flags = V4L2_FMT_FLAG_DYN_RESOLUTION,
	},
	{
		.fourcc = V4L2_PIX_FMT_HEVC,
		.type = MTK_FMT_DEC,
		.num_planes = 1,
		.flags = V4L2_FMT_FLAG_DYN_RESOLUTION,
	},
	{
		.fourcc = V4L2_PIX_FMT_VP8,
		.type = MTK_FMT_DEC,
		.num_planes = 1,
		.flags = V4L2_FMT_FLAG_DYN_RESOLUTION,
	},
	{
		.fourcc = V4L2_PIX_FMT_VP9,
		.type = MTK_FMT_DEC,
		.num_planes = 1,
		.flags = V4L2_FMT_FLAG_DYN_RESOLUTION,
	},
	{
		.fourcc = V4L2_PIX_FMT_MPEG1,
		.type = MTK_FMT_DEC,
		.num_planes = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_MPEG2,
		.type = MTK_FMT_DEC,
		.num_planes = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_MPEG4,
		.type = MTK_FMT_DEC,
		.num_planes = 1,
	},
};

static const struct mtk_codec_framesizes mtk_vdec_framesizes[] = {
	{
		.fourcc	= V4L2_PIX_FMT_H264,
		.stepwise = {  MTK_VDEC_MIN_W, MTK_VDEC_MAX_W, 1,
				MTK_VDEC_MIN_H, MTK_VDEC_MAX_H, 1 },
	},
	{
		.fourcc	= V4L2_PIX_FMT_HEVC,
		.stepwise = {  MTK_VDEC_MIN_W, VCODEC_DEC_4K_CODED_WIDTH, 1,
				MTK_VDEC_MIN_H, VCODEC_DEC_4K_CODED_HEIGHT, 1 },
	},
	{
		.fourcc	= V4L2_PIX_FMT_VP8,
		.stepwise = {  MTK_VDEC_MIN_W, MTK_VDEC_MAX_W, 1,
				MTK_VDEC_MIN_H, MTK_VDEC_MAX_H, 1 },
	},
	{
		.fourcc	= V4L2_PIX_FMT_VP9,
		.stepwise = {  1, MTK_VDEC_MAX_W, 1, 1, MTK_VDEC_MAX_H, 1 },
	},
	{
		.fourcc	= V4L2_PIX_FMT_MPEG1,
		.stepwise = {  MTK_VDEC_MIN_W, MTK_VDEC_MAX_W, 8,
				MTK_VDEC_MIN_H, MTK_VDEC_MAX_H, 8 },
	},
	{
		.fourcc	= V4L2_PIX_FMT_MPEG2,
		.stepwise = {  MTK_VDEC_MIN_W, MTK_VDEC_MAX_W, 8,
				MTK_VDEC_MIN_H, MTK_VDEC_MAX_H, 8 },
	},
	{
		.fourcc	= V4L2_PIX_FMT_MPEG4,
		.stepwise = {  MTK_VDEC_MIN_W, MTK_VDEC_MAX_W, 1,
				MTK_VDEC_MIN_H, MTK_VDEC_MAX_H, 1 },
	},
};

#define NUM_SUPPORTED_FRAMESIZE ARRAY_SIZE(mtk_vdec_framesizes)
#define NUM_FORMATS ARRAY_SIZE(mtk_video_formats)

static struct mtk_video_fmt *mtk_vdec_find_format(struct v4l2_format *f)
{
	struct mtk_video_fmt *fmt;
	unsigned int k;

	for (k = 0; k < NUM_FORMATS; k++) {
		fmt = &mtk_video_formats[k];
		if (fmt->fourcc == f->fmt.pix_mp.pixelformat)
			return fmt;
	}

	return NULL;
}

static struct mtk_video_fmt *mtk_find_fmt_by_pixel(unsigned int pixelformat)
{
	struct mtk_video_fmt *fmt;
	unsigned int k;

	for (k = 0; k < NUM_FORMATS; k++) {
		fmt = &mtk_video_formats[k];
		if (fmt->fourcc == pixelformat)
			return fmt;
	}

	return NULL;
}

static struct mtk_q_data *mtk_vdec_get_q_data(struct mtk_vcodec_ctx *ctx,
					      enum v4l2_buf_type type)
{
	if (V4L2_TYPE_IS_OUTPUT(type))
		return &ctx->q_data[MTK_Q_DATA_SRC];

	return &ctx->q_data[MTK_Q_DATA_DST];
}

/*
 * This function tries to clean all display buffers, the buffers will return
 * in display order.
 * Note the buffers returned from codec driver may still be in driver's
 * reference list.
 */
static struct vb2_buffer *get_display_buffer(struct mtk_vcodec_ctx *ctx)
{
	struct vdec_fb *disp_frame_buffer = NULL;
	struct mtk_video_dec_buf *dstbuf;
	unsigned int i = 0;
	unsigned int num_planes = 0;

	mtk_v4l2_debug(4, "[%d]", ctx->id);
	if (vdec_if_get_param(ctx,
			GET_PARAM_DISP_FRAME_BUFFER,
			&disp_frame_buffer)) {
		mtk_v4l2_err("[%d]Cannot get param : GET_PARAM_DISP_FRAME_BUFFER",
			ctx->id);
		return NULL;
	}

	if (disp_frame_buffer == NULL) {
		mtk_v4l2_debug(4, "No display frame buffer");
		return NULL;
	}

	dstbuf = container_of(disp_frame_buffer, struct mtk_video_dec_buf,
				frame_buffer);
	num_planes = dstbuf->vb.vb2_buf.num_planes;
	mutex_lock(&ctx->lock);
	if (dstbuf->used) {
		for (i = 0; i < num_planes; i++) {
			vb2_set_plane_payload(&dstbuf->vb.vb2_buf, i,
						ctx->picinfo.fb_sz[i]);
		}

		mtk_v4l2_debug(2,
				"[%d]status=%x queue id=%d to done_list %d",
				ctx->id, disp_frame_buffer->status,
				dstbuf->vb.vb2_buf.index,
				dstbuf->queued_in_vb2);

		dstbuf->vb.sequence = ctx->sequence_cap++;
		v4l2_m2m_buf_done(&dstbuf->vb, VB2_BUF_STATE_DONE);
		ctx->decoded_frame_cnt++;
	}
	mutex_unlock(&ctx->lock);
	return &dstbuf->vb.vb2_buf;
}

/*
 * This function tries to clean all capture buffers that are not used as
 * reference buffers by codec driver any more
 * In this case, we need re-queue buffer to vb2 buffer if user space
 * already returns this buffer to v4l2 or this buffer is just the output of
 * previous sps/pps/resolution change decode, or do nothing if user
 * space still owns this buffer
 */
static struct vb2_buffer *get_free_buffer(struct mtk_vcodec_ctx *ctx)
{
	struct mtk_video_dec_buf *dstbuf;
	struct vdec_fb *free_frame_buffer = NULL;

	if (vdec_if_get_param(ctx,
				GET_PARAM_FREE_FRAME_BUFFER,
				&free_frame_buffer)) {
		mtk_v4l2_err("[%d] Error!! Cannot get param", ctx->id);
		return NULL;
	}
	if (free_frame_buffer == NULL) {
		mtk_v4l2_debug(4, " No free frame buffer");
		return NULL;
	}

	mtk_v4l2_debug(4, "[%d] tmp_frame_addr = 0x%p",
			ctx->id, free_frame_buffer);

	dstbuf = container_of(free_frame_buffer, struct mtk_video_dec_buf,
				frame_buffer);

	mutex_lock(&ctx->lock);
	if (dstbuf->used) {
		if ((dstbuf->queued_in_vb2) &&
		    (dstbuf->queued_in_v4l2) &&
		    (free_frame_buffer->status == FB_ST_FREE)) {
			/*
			 * After decode sps/pps or non-display buffer, we don't
			 * need to return capture buffer to user space, but
			 * just re-queue this capture buffer to vb2 queue.
			 * This reduce overheads that dq/q unused capture
			 * buffer. In this case, queued_in_vb2 = true.
			 */
			mtk_v4l2_debug(2,
				"[%d]status=%x queue id=%d to rdy_queue %d",
				ctx->id, free_frame_buffer->status,
				dstbuf->vb.vb2_buf.index,
				dstbuf->queued_in_vb2);
			v4l2_m2m_buf_queue(ctx->m2m_ctx, &dstbuf->vb);
		} else if ((dstbuf->queued_in_vb2 == false) &&
			   (dstbuf->queued_in_v4l2 == true)) {
			/*
			 * If buffer in v4l2 driver but not in vb2 queue yet,
			 * and we get this buffer from free_list, it means
			 * that codec driver do not use this buffer as
			 * reference buffer anymore. We should q buffer to vb2
			 * queue, so later work thread could get this buffer
			 * for decode. In this case, queued_in_vb2 = false
			 * means this buffer is not from previous decode
			 * output.
			 */
			mtk_v4l2_debug(2,
					"[%d]status=%x queue id=%d to rdy_queue",
					ctx->id, free_frame_buffer->status,
					dstbuf->vb.vb2_buf.index);
			v4l2_m2m_buf_queue(ctx->m2m_ctx, &dstbuf->vb);
			dstbuf->queued_in_vb2 = true;
		} else {
			/*
			 * Codec driver do not need to reference this capture
			 * buffer and this buffer is not in v4l2 driver.
			 * Then we don't need to do any thing, just add log when
			 * we need to debug buffer flow.
			 * When this buffer q from user space, it could
			 * directly q to vb2 buffer
			 */
			mtk_v4l2_debug(4, "[%d]status=%x err queue id=%d %d %d",
					ctx->id, free_frame_buffer->status,
					dstbuf->vb.vb2_buf.index,
					dstbuf->queued_in_vb2,
					dstbuf->queued_in_v4l2);
		}
		dstbuf->used = false;
	}
	mutex_unlock(&ctx->lock);
	return &dstbuf->vb.vb2_buf;
}

static void clean_display_buffer(struct mtk_vcodec_ctx *ctx)
{
	struct vb2_buffer *framptr;

	do {
		framptr = get_display_buffer(ctx);
	} while (framptr);
}

static void clean_free_buffer(struct mtk_vcodec_ctx *ctx)
{
	struct vb2_buffer *framptr;

	do {
		framptr = get_free_buffer(ctx);
	} while (framptr);
}

static void mtk_vdec_queue_res_chg_event(struct mtk_vcodec_ctx *ctx)
{
	static const struct v4l2_event ev_src_ch = {
		.type = V4L2_EVENT_SOURCE_CHANGE,
		.u.src_change.changes =
		V4L2_EVENT_SRC_CH_RESOLUTION,
	};

	mtk_v4l2_debug(1, "[%d]", ctx->id);
	v4l2_event_queue_fh(&ctx->fh, &ev_src_ch);
}

static void mtk_vdec_queue_stop_play_event(struct mtk_vcodec_ctx *ctx)
{
	static const struct v4l2_event ev_eos = {
		.type = V4L2_EVENT_EOS,
	};

	mtk_v4l2_debug(1, "[%d]", ctx->id);
	v4l2_event_queue_fh(&ctx->fh, &ev_eos);
}

static void mtk_vdec_flush_decoder(struct mtk_vcodec_ctx *ctx)
{
	unsigned int src_chg = 0;
	int ret = 0;

	ret = vdec_if_decode(ctx, NULL, NULL, &src_chg);
	if (ret)
		mtk_v4l2_err("DecodeFinal failed, ret=%d", ret);

	clean_display_buffer(ctx);
	clean_free_buffer(ctx);
}

static void mtk_vdec_pic_info_update(struct mtk_vcodec_ctx *ctx)
{
	unsigned int dpbsize = 0;
	int ret;

	if (vdec_if_get_param(ctx,
				GET_PARAM_PIC_INFO,
				&ctx->last_decoded_picinfo)) {
		mtk_v4l2_err("[%d]Error!! Cannot get param : GET_PARAM_PICTURE_INFO ERR",
				ctx->id);
		return;
	}

	if (ctx->last_decoded_picinfo.pic_w == 0 ||
		ctx->last_decoded_picinfo.pic_h == 0 ||
		ctx->last_decoded_picinfo.buf_w == 0 ||
		ctx->last_decoded_picinfo.buf_h == 0) {
		mtk_v4l2_err("Cannot get correct pic info");
		return;
	}

	if ((ctx->last_decoded_picinfo.pic_w == ctx->picinfo.pic_w) ||
	    (ctx->last_decoded_picinfo.pic_h == ctx->picinfo.pic_h))
		return;

	mtk_v4l2_debug(1,
			"[%d]-> new(%d,%d), old(%d,%d), bit(%d) real(%d,%d)",
			ctx->id, ctx->last_decoded_picinfo.pic_w,
			ctx->last_decoded_picinfo.pic_h,
			ctx->picinfo.pic_w, ctx->picinfo.pic_h,
			ctx->picinfo.bitdepth,
			ctx->last_decoded_picinfo.buf_w,
			ctx->last_decoded_picinfo.buf_h);

	ret = vdec_if_get_param(ctx, GET_PARAM_DPB_SIZE, &dpbsize);
	if (dpbsize == 0)
		mtk_v4l2_err("Incorrect dpb size, ret=%d", ret);

	ctx->dpb_size = dpbsize;
}

static void mtk_vdec_worker(struct work_struct *work)
{
	struct mtk_vcodec_ctx *ctx = container_of(work, struct mtk_vcodec_ctx,
				decode_work);
	struct mtk_vcodec_dev *dev = ctx->dev;
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	struct mtk_vcodec_mem buf;
	struct vdec_fb *pfb;
	unsigned int src_chg = 0;
	bool res_chg = false;
	bool mtk_vcodec_unsupport = false;
	int ret;
	unsigned int i = 0;
	unsigned int num_planes;
	struct mtk_video_dec_buf *dst_buf_info, *src_buf_info;

	if (ctx->state != MTK_STATE_HEADER) {
		v4l2_m2m_job_finish(dev->m2m_dev_dec, ctx->m2m_ctx);
		mtk_v4l2_debug(1, " %d", ctx->state);
		return;
	}

	src_buf = v4l2_m2m_next_src_buf(ctx->m2m_ctx);
	if (src_buf == NULL) {
		v4l2_m2m_job_finish(dev->m2m_dev_dec, ctx->m2m_ctx);
		mtk_v4l2_debug(1, "[%d] src_buf empty!!", ctx->id);
		return;
	}

	dst_buf = v4l2_m2m_next_dst_buf(ctx->m2m_ctx);
	if (dst_buf == NULL) {
		v4l2_m2m_job_finish(dev->m2m_dev_dec, ctx->m2m_ctx);
		mtk_v4l2_debug(1, "[%d] dst_buf empty!!", ctx->id);
		return;
	}

	src_buf_info = container_of(src_buf, struct mtk_video_dec_buf, vb);
	dst_buf_info = container_of(dst_buf, struct mtk_video_dec_buf, vb);

	pfb = &dst_buf_info->frame_buffer;
	num_planes = dst_buf->vb2_buf.num_planes;
	pfb->num_planes = num_planes;
	pfb->index = dst_buf->vb2_buf.index;

	for (i = 0; i < num_planes; i++) {
		pfb->fb_base[i].va = vb2_plane_vaddr(&dst_buf->vb2_buf, i);
		pfb->fb_base[i].dma_addr =
			vb2_dma_contig_plane_dma_addr(&dst_buf->vb2_buf, i);
		pfb->fb_base[i].size = ctx->picinfo.fb_sz[i];
		pfb->fb_base[i].length = dst_buf->planes[i].length;
		pfb->fb_base[i].dmabuf = dst_buf->planes[i].dbuf;
	}

	pfb->status = 0;
	mtk_v4l2_debug(4, "===>[%d] vdec_if_decode() ===>", ctx->id);

	mtk_v4l2_debug(4,
			"id=%d Framebuf  pfb=%p VA=%p Y_DMA=%pad C_DMA=%pad Size=%zx",
			dst_buf->vb2_buf.index, pfb,
			pfb->fb_base[0].va, &pfb->fb_base[0].dma_addr,
			&pfb->fb_base[1].dma_addr, pfb->fb_base[0].size);

	if (src_buf == &ctx->empty_flush_buf.vb) {
		mtk_v4l2_debug(1, "Got empty flush input buffer.");
		src_buf = v4l2_m2m_src_buf_remove(ctx->m2m_ctx);

		/* update dst buf status */
		dst_buf = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx);
		mutex_lock(&ctx->lock);
		dst_buf_info->used = false;
		mutex_unlock(&ctx->lock);

		vdec_if_decode(ctx, NULL, NULL, &src_chg);
		clean_display_buffer(ctx);
		vb2_set_plane_payload(&dst_buf->vb2_buf, 0, 0);
		if (ctx->q_data[MTK_Q_DATA_DST].fmt->num_planes == 2)
			vb2_set_plane_payload(&dst_buf->vb2_buf, 1, 0);
		dst_buf->flags |= V4L2_BUF_FLAG_LAST;
		v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_DONE);
		clean_free_buffer(ctx);
		v4l2_m2m_job_finish(dev->m2m_dev_dec, ctx->m2m_ctx);
		return;
	}

	buf.va = vb2_plane_vaddr(&src_buf->vb2_buf, 0);
	buf.dma_addr = vb2_dma_contig_plane_dma_addr(&src_buf->vb2_buf, 0);
	buf.size = (size_t)src_buf->planes[0].bytesused;
	buf.length = (size_t)src_buf->planes[0].length;
	buf.dmabuf = src_buf->planes[0].dbuf;

	if (buf.va == NULL) {
		v4l2_m2m_job_finish(dev->m2m_dev_dec, ctx->m2m_ctx);
		mtk_v4l2_err("[%d] id=%d src_addr is NULL!!",
				ctx->id, src_buf->vb2_buf.index);
		return;
	}
	mtk_v4l2_debug(4, "[%d] Bitstream VA=%p DMA=%pad Size=%zx vb=%p",
			ctx->id, buf.va, &buf.dma_addr, buf.size, src_buf);
	dst_buf_info->vb.vb2_buf.timestamp
			= src_buf_info->vb.vb2_buf.timestamp;
	if (src_buf_info->vb.flags & V4L2_BUF_FLAG_TIMECODE)
		dst_buf_info->vb.timecode = src_buf_info->vb.timecode;
	dst_buf_info->vb.field = src_buf_info->vb.field;
	dst_buf_info->vb.flags |= src_buf_info->vb.flags &
			(V4L2_BUF_FLAG_TIMECODE |
			 V4L2_BUF_FLAG_KEYFRAME |
			 V4L2_BUF_FLAG_PFRAME |
			 V4L2_BUF_FLAG_BFRAME |
			 V4L2_BUF_FLAG_TSTAMP_SRC_MASK);
	mutex_lock(&ctx->lock);
	dst_buf_info->used = true;
	mutex_unlock(&ctx->lock);
	src_buf_info->used = true;
	ret = vdec_if_decode(ctx, &buf, pfb, &src_chg);

	res_chg = ((src_chg & VDEC_RES_CHANGE) != 0U) ? true : false;
	mtk_vcodec_unsupport = ((src_chg & VDEC_HW_NOT_SUPPORT) != 0) ?
							true : false;

	if (ret < 0 || mtk_vcodec_unsupport) {
		mtk_v4l2_err(
			" <===[%d], src_buf[%d] last_frame = %d sz=0x%zx dst_buf[%d] vdec_if_decode() ret=%d src_chg=%d===>",
			ctx->id,
			src_buf->vb2_buf.index,
			src_buf_info->lastframe,
			buf.size,
			dst_buf->vb2_buf.index,
			ret, src_chg);
		if (mtk_vcodec_unsupport) {
			src_buf = v4l2_m2m_src_buf_remove(ctx->m2m_ctx);
			src_buf->sequence = ctx->sequence_out++;
			v4l2_m2m_buf_done(&src_buf_info->vb, VB2_BUF_STATE_DONE);
			/*
			 * If cncounter the src unsupport during play, egs:
			 * width/height, bitdepth, level, then teturn EOS event
			 * to  user to stop play it
			 */
			mtk_v4l2_err(" <=== [%d] vcodec not support the source!===>", ctx->id);
			ctx->state = MTK_STATE_FLUSH;
			mtk_vdec_queue_stop_play_event(ctx);
		} else {
			src_buf = v4l2_m2m_src_buf_remove(ctx->m2m_ctx);
			v4l2_m2m_buf_done(&src_buf_info->vb, VB2_BUF_STATE_ERROR);
		}
	} else if (ret == 0 && res_chg == false) {
		/*
		 * we only return src buffer with VB2_BUF_STATE_DONE
		 * when decode success without resolution change except rv30/rv40.
		 */
		src_buf = v4l2_m2m_src_buf_remove(ctx->m2m_ctx);
		src_buf->sequence = ctx->sequence_out++;
		v4l2_m2m_buf_done(&src_buf_info->vb, VB2_BUF_STATE_DONE);
	} else {
		mtk_v4l2_debug(1, "Need more capture buffer\n");
	}

	dst_buf = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx);
	clean_display_buffer(ctx);
	clean_free_buffer(ctx);

	if (ret == 0 && res_chg) {
			mtk_vdec_pic_info_update(ctx);
			/*
			 * On encountering a resolution change in the stream.
			 * The driver must first process and decode all
			 * remaining buffers from before the resolution change
			 * point, so call flush decode here
			 */
			mtk_vdec_flush_decoder(ctx);
			/*
			 * After all buffers containing decoded frames from
			 * before the resolution change point ready to be
			 * dequeued on the CAPTURE queue, the driver sends a
			 * V4L2_EVENT_SOURCE_CHANGE event for source change
			 * type V4L2_EVENT_SRC_CH_RESOLUTION
			 */
			mtk_vdec_queue_res_chg_event(ctx);
	}

	v4l2_m2m_job_finish(dev->m2m_dev_dec, ctx->m2m_ctx);
}

static int vidioc_decoder_cmd(struct file *file, void *priv,
				struct v4l2_decoder_cmd *cmd)
{
	struct mtk_vcodec_ctx *ctx = fh_to_ctx(file->private_data);
	struct vb2_queue *src_vq, *dst_vq;
	int ret;

	ret = v4l2_m2m_ioctl_try_decoder_cmd(file, priv, cmd);
	if (ret)
		return ret;

	if (!vb2_is_streaming(&ctx->fh.m2m_ctx->cap_q_ctx.q) ||
	    !vb2_is_streaming(&ctx->fh.m2m_ctx->out_q_ctx.q))
		return 0;

	mtk_v4l2_debug(1, "decoder cmd=%u", cmd->cmd);
	dst_vq = v4l2_m2m_get_vq(ctx->m2m_ctx,
					V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	switch (cmd->cmd) {
	case V4L2_DEC_CMD_STOP:
		src_vq = v4l2_m2m_get_vq(ctx->m2m_ctx,
				V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
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

	case V4L2_DEC_CMD_START:
		if (ctx->is_draining)
			return -EBUSY;
		if (ctx->is_stopped) {
			vb2_clear_last_buffer_dequeued(dst_vq);
		}
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

void mtk_vdec_unlock(struct mtk_vcodec_ctx *ctx)
{
	struct mtk_vcodec_dev *dev = ctx->dev;

	mtk_v4l2_debug(4, "ctx %p [%d] sem_cnt %d",
		ctx, ctx->id, dev->dec_sem.count);

	up(&dev->dec_sem);
}

void mtk_vdec_lock(struct mtk_vcodec_ctx *ctx)
{
	struct mtk_vcodec_dev *dev = ctx->dev;
	int ret = -1;

	mtk_v4l2_debug(4, "ctx %p [%d] sem_cnt %d",
		ctx, ctx->id, ctx->dev->dec_sem.count);

	while (ret != 0)
		ret = down_interruptible(&ctx->dev->dec_sem);
	mtk_v4l2_debug(4, "lock done\n");
}

void mtk_vcodec_dec_release(struct mtk_vcodec_ctx *ctx)
{
	vdec_if_deinit(ctx);
	ctx->state = MTK_STATE_FREE;
}

void mtk_vcodec_dec_set_default_params(struct mtk_vcodec_ctx *ctx)
{
	struct mtk_q_data *q_data;

	ctx->m2m_ctx->q_lock = &ctx->dev->dev_mutex;
	ctx->fh.m2m_ctx = ctx->m2m_ctx;
	ctx->fh.ctrl_handler = &ctx->ctrl_hdl;
	INIT_WORK(&ctx->decode_work, mtk_vdec_worker);
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
	q_data->fmt = &mtk_video_formats[OUT_FMT_IDX];
	q_data->field = V4L2_FIELD_NONE;

	q_data->sizeimage[0] = 1024 * 1024;
	q_data->bytesperline[0] = 0;

	q_data = &ctx->q_data[MTK_Q_DATA_DST];
	memset(q_data, 0, sizeof(struct mtk_q_data));
	q_data->visible_width = DFT_CFG_WIDTH;
	q_data->visible_height = DFT_CFG_HEIGHT;
	q_data->coded_width = DFT_CFG_WIDTH;
	q_data->coded_height = DFT_CFG_HEIGHT;
	q_data->fmt = &mtk_video_formats[CAP_FMT_IDX];
	q_data->field = V4L2_FIELD_NONE;

	v4l_bound_align_image(&q_data->coded_width,
				MTK_VDEC_MIN_W,
				MTK_VDEC_MAX_W, 4,
				&q_data->coded_height,
				MTK_VDEC_MIN_H,
				MTK_VDEC_MAX_H, 5, 6);

	q_data->sizeimage[0] = q_data->coded_width * q_data->coded_height;
	q_data->bytesperline[0] = q_data->coded_width;
	q_data->sizeimage[1] = q_data->sizeimage[0] / 2;
	q_data->bytesperline[1] = q_data->coded_width;
}

static u32 mtk_vdec_get_capture_fourcc(struct mtk_vcodec_ctx *ctx)
{
	u32 fourcc;
	unsigned int num_planes;
	struct mtk_q_data *src_q_data, *dst_q_data;
	struct vdec_pic_info *picinfo;

	fourcc = V4L2_PIX_FMT_MT21;
	picinfo = &ctx->picinfo;
	src_q_data = &ctx->q_data[MTK_Q_DATA_SRC];
	dst_q_data = &ctx->q_data[MTK_Q_DATA_DST];
	num_planes = dst_q_data->fmt->num_planes;

		fourcc = picinfo->ufo_mode == 0U ?
			V4L2_PIX_FMT_MT21 : V4L2_PIX_FMT_MT21C;

	return fourcc;
}

static int vidioc_vdec_qbuf(struct file *file, void *priv,
			    struct v4l2_buffer *buf)
{
	struct mtk_vcodec_ctx *ctx = fh_to_ctx(file->private_data);
	struct vb2_queue *vq;
	struct vb2_buffer *vb;
	struct mtk_video_dec_buf *mtkbuf;
	struct vb2_v4l2_buffer	*vb2_v4l2;

	if (ctx->state == MTK_STATE_ABORT) {
		mtk_v4l2_err("[%d] Call on QBUF after unrecoverable error",
				ctx->id);
		return -EIO;
	}

	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, buf->type);
	vb = vq->bufs[buf->index];
	vb2_v4l2 = container_of(vb, struct vb2_v4l2_buffer, vb2_buf);
	mtkbuf = container_of(vb2_v4l2, struct mtk_video_dec_buf, vb);

	if ((buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) &&
	    (buf->m.planes[0].bytesused == 0)) {
		mtkbuf->lastframe = true;
		mtk_v4l2_debug(1, "[%d] (%d) id=%d lastframe=%d (%d,%d, %d) vb=%p",
			 ctx->id, buf->type, buf->index,
			 mtkbuf->lastframe, buf->bytesused,
			 buf->m.planes[0].bytesused, buf->length,
			 vb);
	}

	return v4l2_m2m_qbuf(file, ctx->m2m_ctx, buf);
}

static int vidioc_vdec_dqbuf(struct file *file, void *priv,
			     struct v4l2_buffer *buf)
{
	struct mtk_vcodec_ctx *ctx = fh_to_ctx(file->private_data);

	if (ctx->state == MTK_STATE_ABORT) {
		mtk_v4l2_err("[%d] Call on DQBUF after unrecoverable error",
				ctx->id);
		return -EIO;
	}

	return v4l2_m2m_dqbuf(file, ctx->m2m_ctx, buf);
}

static int vidioc_vdec_querycap(struct file *file, void *priv,
				struct v4l2_capability *cap)
{
	strscpy(cap->driver, MTK_VCODEC_DEC_NAME, sizeof(cap->driver));
	strscpy(cap->bus_info, MTK_PLATFORM_STR, sizeof(cap->bus_info));
	strscpy(cap->card, MTK_PLATFORM_STR, sizeof(cap->card));

	return 0;
}

static int vidioc_vdec_subscribe_evt(struct v4l2_fh *fh,
				     const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 2, NULL);
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subscribe(fh, sub);
	default:
		return v4l2_ctrl_subscribe_event(fh, sub);
	}
}

static int vidioc_try_fmt(struct v4l2_format *f, struct mtk_video_fmt *fmt)
{
	struct v4l2_pix_format_mplane *pix_fmt_mp = &f->fmt.pix_mp;
	unsigned int i;

	pix_fmt_mp->field = V4L2_FIELD_NONE;

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		pix_fmt_mp->num_planes = 1;
		pix_fmt_mp->plane_fmt[0].bytesperline = 0;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		int tmp_w, tmp_h;

		pix_fmt_mp->height = clamp(pix_fmt_mp->height,
					MTK_VDEC_MIN_H,
					MTK_VDEC_MAX_H);
		pix_fmt_mp->width = clamp(pix_fmt_mp->width,
					MTK_VDEC_MIN_W,
					MTK_VDEC_MAX_W);

		/*
		 * Find next closer width align 64, heign align 64, size align
		 * 64 rectangle
		 * Note: This only get default value, the real HW needed value
		 *       only available when ctx in MTK_STATE_HEADER state
		 */
		tmp_w = pix_fmt_mp->width;
		tmp_h = pix_fmt_mp->height;
		v4l_bound_align_image(&pix_fmt_mp->width,
					MTK_VDEC_MIN_W,
					MTK_VDEC_MAX_W, 6,
					&pix_fmt_mp->height,
					MTK_VDEC_MIN_H,
					MTK_VDEC_MAX_H, 6, 9);

		if (pix_fmt_mp->width < tmp_w &&
			(pix_fmt_mp->width + 64) <= MTK_VDEC_MAX_W)
			pix_fmt_mp->width += 64;
		if (pix_fmt_mp->height < tmp_h &&
			(pix_fmt_mp->height + 64) <= MTK_VDEC_MAX_H)
			pix_fmt_mp->height += 64;

		mtk_v4l2_debug(0,
			"before resize width=%d, height=%d, after resize width=%d, height=%d, sizeimage=%d",
			tmp_w, tmp_h, pix_fmt_mp->width,
			pix_fmt_mp->height,
			pix_fmt_mp->width * pix_fmt_mp->height);

		pix_fmt_mp->num_planes = fmt->num_planes;

		for (i = 0; i < pix_fmt_mp->num_planes; i++) {
			pix_fmt_mp->plane_fmt[i].sizeimage =
					(pix_fmt_mp->width *
					pix_fmt_mp->height) >> i;
			pix_fmt_mp->plane_fmt[i].bytesperline =
					pix_fmt_mp->width;
		}
	}

	for (i = 0; i < pix_fmt_mp->num_planes; i++)
		memset(&(pix_fmt_mp->plane_fmt[i].reserved[0]), 0x0,
			   sizeof(pix_fmt_mp->plane_fmt[0].reserved));

	pix_fmt_mp->flags = 0;
	memset(&pix_fmt_mp->reserved, 0x0, sizeof(pix_fmt_mp->reserved));
	return 0;
}

static int vidioc_try_fmt_vid_cap_mplane(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct mtk_video_fmt *fmt;

	fmt = mtk_vdec_find_format(f);
	if (!fmt) {
		f->fmt.pix.pixelformat = mtk_video_formats[CAP_FMT_IDX].fourcc;
		fmt = mtk_vdec_find_format(f);
	}

	return vidioc_try_fmt(f, fmt);
}

static int vidioc_try_fmt_vid_out_mplane(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix_fmt_mp = &f->fmt.pix_mp;
	struct mtk_video_fmt *fmt;

	fmt = mtk_vdec_find_format(f);
	if (!fmt) {
		f->fmt.pix.pixelformat = mtk_video_formats[OUT_FMT_IDX].fourcc;
		fmt = mtk_vdec_find_format(f);
	}

	if (pix_fmt_mp->plane_fmt[0].sizeimage == 0) {
		mtk_v4l2_err("sizeimage of output format must be given");
		return -EINVAL;
	}

	return vidioc_try_fmt(f, fmt);
}

static int vidioc_vdec_g_selection(struct file *file, void *priv,
			struct v4l2_selection *s)
{
	struct mtk_vcodec_ctx *ctx = fh_to_ctx(file->private_data);
	struct mtk_q_data *q_data;

	/* PORT/guard: G_SELECTION can arrive with a NULL fh (priv) before a
	 * decode fh/instance is fully wired up; bail rather than Oops. Check
	 * priv (a plain param) not ctx -- the compiler elides a container_of
	 * NULL test. */
	if (!file->private_data || s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	q_data = &ctx->q_data[MTK_Q_DATA_DST];

	switch (s->target) {
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = ctx->picinfo.pic_w;
		s->r.height = ctx->picinfo.pic_h;
		break;
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = ctx->picinfo.buf_w;
		s->r.height = ctx->picinfo.buf_h;
		break;
	case V4L2_SEL_TGT_COMPOSE:
		/* PORT/guard: don't query the codec instance before a decode
		 * instance exists (state < HEADER => drv_handle NULL => Oops in
		 * vdec_if_get_param); short-circuit to the default crop instead. */
		if (ctx->state < MTK_STATE_HEADER ||
		    vdec_if_get_param(ctx, GET_PARAM_CROP_INFO, &(s->r))) {
			/* set to default value if header info not ready yet*/
			s->r.left = 0;
			s->r.top = 0;
			s->r.width = q_data->visible_width;
			s->r.height = q_data->visible_height;
		}
		break;
	default:
		return -EINVAL;
	}

	if (ctx->state < MTK_STATE_HEADER) {
		/* set to default value if header info not ready yet*/
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = q_data->visible_width;
		s->r.height = q_data->visible_height;
		return 0;
	}

	return 0;
}

static int vidioc_vdec_s_selection(struct file *file, void *priv,
				struct v4l2_selection *s)
{
	struct mtk_vcodec_ctx *ctx = fh_to_ctx(file->private_data);

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	switch (s->target) {
	case V4L2_SEL_TGT_COMPOSE:
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = ctx->picinfo.pic_w;
		s->r.height = ctx->picinfo.pic_h;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int vidioc_vdec_g_fmt(struct file *file, void *priv,
			     struct v4l2_format *f);

static int vidioc_vdec_s_fmt_out(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct mtk_vcodec_ctx *ctx = fh_to_ctx(file->private_data);
	struct v4l2_pix_format_mplane *pix_mp;
	struct mtk_q_data *q_data;
	int ret = 0;
	struct mtk_video_fmt *fmt;
	uint32_t size[2];

	mtk_v4l2_debug(4, "[%d]", ctx->id);

	q_data = mtk_vdec_get_q_data(ctx, f->type);
	if (!q_data)
		return -EINVAL;

	if (ctx->state >= MTK_STATE_HEADER) {
		ret = vidioc_vdec_g_fmt(file, priv, f);
		if (ret)
			return ret;

		ctx->colorspace = f->fmt.pix_mp.colorspace;
		ctx->ycbcr_enc = f->fmt.pix_mp.ycbcr_enc;
		ctx->quantization = f->fmt.pix_mp.quantization;
		ctx->xfer_func = f->fmt.pix_mp.xfer_func;

		size[0] = q_data->visible_width;
		size[1] = q_data->visible_height;
		vdec_if_set_param(ctx, SET_PARAM_FRAME_SIZE, (void *) size);

		return 0;
	}

	pix_mp = &f->fmt.pix_mp;
	if (vb2_is_busy(&ctx->m2m_ctx->out_q_ctx.q)) {
		mtk_v4l2_err("out_q_ctx buffers already requested");
		ret = -EBUSY;
	}

	fmt = mtk_vdec_find_format(f);
	if (fmt == NULL) {
		f->fmt.pix.pixelformat =
			mtk_video_formats[OUT_FMT_IDX].fourcc;
		fmt = mtk_vdec_find_format(f);
	}

	q_data->fmt = fmt;
	vidioc_try_fmt(f, q_data->fmt);


	q_data->sizeimage[0] = pix_mp->plane_fmt[0].sizeimage;
	q_data->coded_width = pix_mp->width;
	q_data->coded_height = pix_mp->height;

	size[0] = q_data->coded_width;
	size[1] = q_data->coded_height;

	ctx->colorspace = f->fmt.pix_mp.colorspace;
	ctx->ycbcr_enc = f->fmt.pix_mp.ycbcr_enc;
	ctx->quantization = f->fmt.pix_mp.quantization;
	ctx->xfer_func = f->fmt.pix_mp.xfer_func;

	if (ctx->state == MTK_STATE_FREE) {
		ret = vdec_if_init(ctx, q_data->fmt->fourcc);
		if (ret) {
			mtk_v4l2_err("[%d]: vdec_if_init() fail ret=%d",
					ctx->id, ret);
			return -EINVAL;
		}
		ctx->state = MTK_STATE_INIT;
	}

	if (ctx->state != MTK_STATE_FREE)
		vdec_if_set_param(ctx, SET_PARAM_FRAME_SIZE, (void *) size);

	return 0;
}

static int vidioc_vdec_s_fmt_cap(struct file *file, void *priv,
				 struct v4l2_format *f)
{
	struct mtk_vcodec_ctx *ctx = fh_to_ctx(file->private_data);
	struct v4l2_pix_format_mplane *pix_mp;
	struct mtk_q_data *q_data, *src_q_data;
	int ret = 0, i;
	struct mtk_video_fmt *fmt;

	mtk_v4l2_debug(4, "[%d]", ctx->id);

	q_data = mtk_vdec_get_q_data(ctx, f->type);
	if (!q_data)
		return -EINVAL;

	if (ctx->state >= MTK_STATE_HEADER) {
		ret = vidioc_vdec_g_fmt(file, priv, f);
		if (ret)
			return ret;

		vdec_if_set_param(ctx, SET_PARAM_FB_NUM_PLANES,
				(void *) &q_data->fmt->num_planes);

		return 0;
	}

	src_q_data = mtk_vdec_get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	if (!q_data)
		return -EINVAL;

	pix_mp = &f->fmt.pix_mp;

	if (vb2_is_busy(&ctx->m2m_ctx->cap_q_ctx.q)) {
		mtk_v4l2_err("cap_q_ctx buffers already requested");
		ret = -EBUSY;
	}

	fmt = mtk_vdec_find_format(f);
	if (fmt == NULL) {
		f->fmt.pix.pixelformat =
			mtk_video_formats[CAP_FMT_IDX].fourcc;
		fmt = mtk_vdec_find_format(f);
	}

	/* use OUTPUT's dimensions */
	pix_mp->height = src_q_data->coded_height;
	pix_mp->width = src_q_data->coded_width;

	q_data->fmt = fmt;
	vidioc_try_fmt(f, q_data->fmt);

	q_data->coded_width = pix_mp->width;
	q_data->coded_height = pix_mp->height;

	for (i = 0; i < pix_mp->num_planes; i++) {
		q_data->sizeimage[0] = pix_mp->plane_fmt[0].sizeimage;
		q_data->sizeimage[1] = pix_mp->plane_fmt[1].sizeimage;
	}

	if (ctx->state != MTK_STATE_FREE)
		vdec_if_set_param(ctx, SET_PARAM_FB_NUM_PLANES,
				(void *) &q_data->fmt->num_planes);

	return 0;
}

static int vidioc_enum_framesizes(struct file *file, void *priv,
				struct v4l2_frmsizeenum *fsize)
{
	int i = 0;
	struct mtk_vcodec_ctx *ctx = fh_to_ctx(file->private_data);

	if (fsize->index != 0)
		return -EINVAL;

	for (i = 0; i < NUM_SUPPORTED_FRAMESIZE; ++i) {
		if (fsize->pixel_format != mtk_vdec_framesizes[i].fourcc)
			continue;

		fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
		fsize->stepwise = mtk_vdec_framesizes[i].stepwise;
		if (!(ctx->dev->dec_capability &
				VCODEC_CAPABILITY_4K_DISABLED)) {
			mtk_v4l2_debug(4, "4K is enabled");
			fsize->stepwise.max_width =
					VCODEC_DEC_4K_CODED_WIDTH;
			fsize->stepwise.max_height =
					VCODEC_DEC_4K_CODED_HEIGHT;
		}
		mtk_v4l2_debug(1, "%x, %d %d %d %d %d %d",
				ctx->dev->dec_capability,
				fsize->stepwise.min_width,
				fsize->stepwise.max_width,
				fsize->stepwise.step_width,
				fsize->stepwise.min_height,
				fsize->stepwise.max_height,
				fsize->stepwise.step_height);
		return 0;
	}

	return -EINVAL;
}

static int vidioc_enum_fmt(struct v4l2_fmtdesc *f, bool output_queue)
{
	struct mtk_video_fmt *fmt;
	int i, j = 0;

	for (i = 0; i < NUM_FORMATS; i++) {
		if (output_queue && (mtk_video_formats[i].type != MTK_FMT_DEC))
			continue;
		if (!output_queue &&
			(mtk_video_formats[i].type != MTK_FMT_FRAME))
			continue;

		if (j == f->index)
			break;
		++j;
	}

	if (i == NUM_FORMATS)
		return -EINVAL;

	fmt = &mtk_video_formats[i];
	f->pixelformat = fmt->fourcc;
	f->flags = fmt->flags;

	return 0;
}

static int vidioc_vdec_enum_fmt_vid_cap_mplane(struct file *file, void *pirv,
					       struct v4l2_fmtdesc *f)
{
	return vidioc_enum_fmt(f, false);
}

static int vidioc_vdec_enum_fmt_vid_out_mplane(struct file *file, void *priv,
					       struct v4l2_fmtdesc *f)
{
	return vidioc_enum_fmt(f, true);
}

static int vidioc_vdec_g_fmt(struct file *file, void *priv,
			     struct v4l2_format *f)
{
	struct mtk_vcodec_ctx *ctx = fh_to_ctx(file->private_data);
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	struct vb2_queue *vq;
	struct mtk_q_data *q_data;
	unsigned int i = 0;


	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);
	if (!vq) {
		mtk_v4l2_err("no vb2 queue for type=%d", f->type);
		return -EINVAL;
	}

	q_data = mtk_vdec_get_q_data(ctx, f->type);
	if (!q_data)
		return -EINVAL;

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		pix_mp->width = q_data->visible_width;
		pix_mp->height = q_data->visible_height;
	} else {
		struct mtk_q_data *src_q_data =
			mtk_vdec_get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
		pix_mp->width = src_q_data->coded_width;
		pix_mp->height = src_q_data->coded_height;
	}

	pix_mp->field = V4L2_FIELD_NONE;
	pix_mp->colorspace = ctx->colorspace;
	pix_mp->ycbcr_enc = ctx->ycbcr_enc;
	pix_mp->quantization = ctx->quantization;
	pix_mp->xfer_func = ctx->xfer_func;
	pix_mp->num_planes = q_data->fmt->num_planes;
	pix_mp->pixelformat = q_data->fmt->fourcc;
	for (i = 0; i < pix_mp->num_planes; i++) {
		pix_mp->plane_fmt[i].bytesperline =
			q_data->bytesperline[i];
		pix_mp->plane_fmt[i].sizeimage =
			q_data->sizeimage[i];
	}

	return 0;
}

static int vb2ops_vdec_queue_setup(struct vb2_queue *vq,
				unsigned int *nbuffers,
				unsigned int *nplanes,
				unsigned int sizes[],
				struct device *alloc_ctxs[])
{
	struct mtk_vcodec_ctx *ctx = vb2_get_drv_priv(vq);
	struct mtk_q_data *q_data;
	unsigned int i;

	q_data = mtk_vdec_get_q_data(ctx, vq->type);

	if (q_data == NULL) {
		mtk_v4l2_err("vq->type=%d err\n", vq->type);
		return -EINVAL;
	}

	if (*nplanes) {
		for (i = 0; i < *nplanes; i++) {
			if (sizes[i] < q_data->sizeimage[i])
				return -EINVAL;
		}
	} else {
		if (vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
			*nplanes = q_data->fmt->num_planes;

			if (ctx->state >= MTK_STATE_HEADER)
				*nbuffers = max_t(unsigned int, *nbuffers, ctx->dpb_size);
		} else
			*nplanes = 1;

		for (i = 0; i < *nplanes; i++)
			sizes[i] = q_data->sizeimage[i];
	}

	mtk_v4l2_debug(1,
			"[%d]\t type = %d, get %d plane(s), %d buffer(s) of size 0x%x 0x%x ",
			ctx->id, vq->type, *nplanes, *nbuffers,
			sizes[0], sizes[1]);

	return 0;
}

static int vb2ops_vdec_buf_prepare(struct vb2_buffer *vb)
{
	struct mtk_vcodec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct mtk_q_data *q_data;
	unsigned int i;

	mtk_v4l2_debug(4, "[%d] (%d) id=%d",
			ctx->id, vb->vb2_queue->type, vb->index);

	q_data = mtk_vdec_get_q_data(ctx, vb->vb2_queue->type);

	vbuf->field = V4L2_FIELD_NONE;

	for (i = 0; i < q_data->fmt->num_planes; i++) {
		if (vb2_plane_size(vb, i) < q_data->sizeimage[i]) {
			mtk_v4l2_err("data will not fit into plane %d (%lu < %d)",
				i, vb2_plane_size(vb, i),
				q_data->sizeimage[i]);
		}
	}

	return 0;
}

static void vb2ops_vdec_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *src_buf;
	struct mtk_vcodec_mem src_mem;
	unsigned int src_chg = 0;
	bool res_chg = false;
	bool mtk_vcodec_unsupport = false;
	bool wait_seq_header = false;
	int ret = 0;
	unsigned int i = 0;
	unsigned int dpbsize = 1;
	struct mtk_vcodec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vb2_v4l2 = NULL;
	struct mtk_video_dec_buf *buf = NULL;
	struct mtk_q_data *src_q_data, *dst_q_data;
	static const struct v4l2_event eos_event = {
		.type = V4L2_EVENT_EOS
	};
	u32 fourcc;

	mtk_v4l2_debug(4, "[%d] (%d) id=%d, vb=%p",
			ctx->id, vb->vb2_queue->type,
			vb->index, vb);

	if (!V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type) &&
	    vb2_is_streaming(&ctx->fh.m2m_ctx->cap_q_ctx.q) &&
	    ctx->next_is_last) {
		unsigned int i;

		vb2_v4l2 = to_vb2_v4l2_buffer(vb);
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

	/*
	 * check if this buffer is ready to be used after decode
	 */
	if (vb->vb2_queue->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		vb2_v4l2 = to_vb2_v4l2_buffer(vb);
		buf = container_of(vb2_v4l2, struct mtk_video_dec_buf, vb);
		mutex_lock(&ctx->lock);
		if (buf->used == false) {
			v4l2_m2m_buf_queue(ctx->m2m_ctx, vb2_v4l2);
			buf->queued_in_vb2 = true;
			buf->queued_in_v4l2 = true;
		} else {
			buf->queued_in_vb2 = false;
			buf->queued_in_v4l2 = true;
		}
		mutex_unlock(&ctx->lock);
		return;
	}

	v4l2_m2m_buf_queue(ctx->m2m_ctx, to_vb2_v4l2_buffer(vb));

	if (ctx->state == MTK_STATE_FREE) {
		struct mtk_q_data *q_data;
		uint32_t size[2];
		int ret;

		q_data = mtk_vdec_get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
		ret = vdec_if_init(ctx, q_data->fmt->fourcc);
		if (ret) {
			mtk_v4l2_err("[%d]: vdec_if_init() fail ret=%d",
					ctx->id, ret);
			return;
		}

		size[0] = q_data->coded_width;
		size[1] = q_data->coded_height;
		vdec_if_set_param(ctx, SET_PARAM_FRAME_SIZE, (void *) size);

		q_data = mtk_vdec_get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
		vdec_if_set_param(ctx, SET_PARAM_FB_NUM_PLANES,
			(void *) &q_data->fmt->num_planes);

		ctx->state = MTK_STATE_INIT;
	}

	if (ctx->state != MTK_STATE_INIT) {
		mtk_v4l2_debug(4, "[%d] already init driver %d",
				ctx->id, ctx->state);
		return;
	}

	src_buf = v4l2_m2m_next_src_buf(ctx->m2m_ctx);
	if (!src_buf) {
		mtk_v4l2_err("No src buffer");
		return;
	}

	if (src_buf == &ctx->empty_flush_buf.vb) {
		/* This shouldn't happen. Just in case. */
		mtk_v4l2_err("Invalid flush buffer.");
		v4l2_m2m_src_buf_remove(ctx->m2m_ctx);
		return;
	}

	src_mem.va = vb2_plane_vaddr(&src_buf->vb2_buf, 0);
	src_mem.dma_addr = vb2_dma_contig_plane_dma_addr(&src_buf->vb2_buf, 0);
	src_mem.size = (size_t)src_buf->planes[0].bytesused;
	src_mem.length = (size_t)src_buf->planes[0].length;
	src_mem.dmabuf = src_buf->planes[0].dbuf;
    mtk_v4l2_debug(0, "[%d] %x %x %x %x %x %x %x %x %x %x %x \n",ctx->id,
        ((char*)src_mem.va)[0],
        ((char*)src_mem.va)[1],
        ((char*)src_mem.va)[2],
        ((char*)src_mem.va)[3],
        ((char*)src_mem.va)[4],
        ((char*)src_mem.va)[5],
        ((char*)src_mem.va)[6],
        ((char*)src_mem.va)[7],
        ((char*)src_mem.va)[8],
        ((char*)src_mem.va)[9],
        ((char*)src_mem.va)[10]);
	mtk_v4l2_debug(2,
			"[%d] buf id=%d va=%p dma=%pad size=%zx length=%zu dmabuf=%p",
			ctx->id, src_buf->vb2_buf.index,
			src_mem.va, &src_mem.dma_addr,
			src_mem.size, src_mem.length, src_mem.dmabuf);

	ret = vdec_if_decode(ctx, &src_mem, NULL, &src_chg);

	/* src_chg bit0 for res change flag, bit1 for realloc mv buf flag,
	 * bit2 for not support flag, other bits are reserved
	 */
	res_chg = ((src_chg & VDEC_RES_CHANGE) != 0U) ? true : false;
	mtk_vcodec_unsupport = ((src_chg & VDEC_HW_NOT_SUPPORT) != 0) ?
							true : false;
	wait_seq_header = ((src_chg & VDEC_NEED_SEQ_HEADER) != 0U) ?
						true : false;
	if (ret || !res_chg || mtk_vcodec_unsupport || wait_seq_header) {
		/*
		 * fb == NULL menas to parse SPS/PPS header or
		 * resolution info in src_mem. Decode can fail
		 * if there is no SPS header or picture info
		 * in bs
		 */

		src_buf = v4l2_m2m_src_buf_remove(ctx->m2m_ctx);
		src_buf->sequence = ctx->sequence_out++;
		v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_DONE);
		mtk_v4l2_debug(ret ? 0 : 1,
			       "[%d] vdec_if_decode() src_buf=%d, size=%zu, fail=%d, res_chg=%d, mtk_vcodec_unsupport=%d",
			       ctx->id, src_buf->vb2_buf.index,
			       src_mem.size, ret, res_chg, mtk_vcodec_unsupport);

		/* If not support the source, eg: w/h, bitdepth, level, we need to stop to play it */
		if (mtk_vcodec_unsupport) {
			mtk_v4l2_err("[%d]Error!! Codec driver not support the file!",
				ctx->id);
			mtk_vdec_queue_stop_play_event(ctx);
		}
		return;
	}

	if (res_chg) {
		mtk_v4l2_debug(3, "[%d] vdec_if_decode() res_chg: %d\n",
			ctx->id, res_chg);
	}

	if (vdec_if_get_param(ctx, GET_PARAM_PIC_INFO, &ctx->picinfo)) {
		mtk_v4l2_err("[%d]Error!! Cannot get param : GET_PARAM_PICTURE_INFO ERR",
				ctx->id);
		return;
	}

	ctx->last_decoded_picinfo = ctx->picinfo;

	src_q_data = &ctx->q_data[MTK_Q_DATA_SRC];
	src_q_data->visible_width = ctx->picinfo.pic_w;
	src_q_data->visible_height = ctx->picinfo.pic_h;
	src_q_data->coded_width = ctx->picinfo.buf_w;
	src_q_data->coded_height = ctx->picinfo.buf_h;

	dst_q_data = &ctx->q_data[MTK_Q_DATA_DST];
	fourcc = mtk_vdec_get_capture_fourcc(ctx);
	dst_q_data->fmt = mtk_find_fmt_by_pixel(fourcc);
	dst_q_data->visible_width = ctx->picinfo.pic_w;
	dst_q_data->visible_height = ctx->picinfo.pic_h;
	dst_q_data->coded_width = ctx->picinfo.buf_w;
	dst_q_data->coded_height = ctx->picinfo.buf_h;
	for (i = 0; i < dst_q_data->fmt->num_planes; i++) {
		dst_q_data->sizeimage[i] = ctx->picinfo.fb_sz[i];
		dst_q_data->bytesperline[i] = ctx->picinfo.buf_w;
	}

	mtk_v4l2_debug(2, "[%d] Init Vdec OK wxh=%dx%d pic wxh=%dx%d bitdepth:%d ufo:%d sz[0]=0x%x sz[1]=0x%x",
			ctx->id,
			ctx->picinfo.buf_w, ctx->picinfo.buf_h,
			ctx->picinfo.pic_w, ctx->picinfo.pic_h,
			ctx->picinfo.bitdepth, ctx->picinfo.ufo_mode,
			dst_q_data->sizeimage[0],
			dst_q_data->sizeimage[1]);

	mtk_v4l2_debug(2, "[%d] fmt %d, num_planes %d, fb_sz[0] %d, fb_sz[1] %d",
			ctx->id,
			dst_q_data->fmt->fourcc,
			dst_q_data->fmt->num_planes,
			ctx->picinfo.fb_sz[0],
			ctx->picinfo.fb_sz[1]);

	ret = vdec_if_get_param(ctx, GET_PARAM_DPB_SIZE, &dpbsize);
	if (dpbsize == 0)
		mtk_v4l2_err("[%d] GET_PARAM_DPB_SIZE fail=%d", ctx->id, ret);

	ctx->dpb_size = dpbsize;
	ctx->state = MTK_STATE_HEADER;
	mtk_v4l2_debug(1, "[%d] dpbsize=%d", ctx->id, ctx->dpb_size);

	mtk_vdec_queue_res_chg_event(ctx);
}

static void vb2ops_vdec_buf_finish(struct vb2_buffer *vb)
{
	struct mtk_vcodec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vb2_v4l2;
	struct mtk_video_dec_buf *buf;

	if (vb->vb2_queue->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return;

	vb2_v4l2 = container_of(vb, struct vb2_v4l2_buffer, vb2_buf);
	buf = container_of(vb2_v4l2, struct mtk_video_dec_buf, vb);
	mutex_lock(&ctx->lock);
	buf->queued_in_v4l2 = false;
	buf->queued_in_vb2 = false;
	mutex_unlock(&ctx->lock);
}

static int vb2ops_vdec_buf_init(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vb2_v4l2 = container_of(vb,
					struct vb2_v4l2_buffer, vb2_buf);
	struct mtk_video_dec_buf *buf = container_of(vb2_v4l2,
					struct mtk_video_dec_buf, vb);

	if (vb->vb2_queue->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		buf->used = false;
		buf->queued_in_v4l2 = false;
	} else {
		buf->lastframe = false;
	}

	return 0;
}

static int vb2ops_vdec_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct mtk_vcodec_ctx *ctx = vb2_get_drv_priv(q);

	if (ctx->state == MTK_STATE_FLUSH)
		ctx->state = MTK_STATE_HEADER;

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

	return 0;
}

static void vb2ops_vdec_stop_streaming(struct vb2_queue *q)
{
	struct vb2_v4l2_buffer *src_buf = NULL, *dst_buf = NULL;
	struct mtk_vcodec_ctx *ctx = vb2_get_drv_priv(q);
	static const struct v4l2_event eos_event = {
		.type = V4L2_EVENT_EOS
	};
	unsigned int i = 0;

	mtk_v4l2_debug(4, "[%d] (%d) state=(%x) ctx->decoded_frame_cnt=%d",
			ctx->id, q->type, ctx->state, ctx->decoded_frame_cnt);

	if (V4L2_TYPE_IS_OUTPUT(q->type)) {
		if (ctx->is_draining) {
			struct vb2_v4l2_buffer *next_dst_buf;

			next_dst_buf = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx);
			if (!next_dst_buf)
				ctx->next_is_last = true;
			else {
				next_dst_buf->flags |= V4L2_BUF_FLAG_LAST;
				next_dst_buf->sequence = ctx->sequence_cap++;
				v4l2_m2m_buf_done(next_dst_buf, VB2_BUF_STATE_DONE);
				v4l2_event_queue_fh(&ctx->fh, &eos_event);
			}
		}
	} else {
		ctx->is_draining = false;
		ctx->is_stopped = false;
		ctx->next_is_last = false;
	}

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		while ((src_buf = v4l2_m2m_src_buf_remove(ctx->m2m_ctx))) {
			if (src_buf != &ctx->empty_flush_buf.vb) {
				v4l2_m2m_buf_done(src_buf,
								VB2_BUF_STATE_ERROR);
			}
		}
		/* PORT 5.10->6.18: sparse q->bufs via vb2_get_buffer() */
		for (i = 0; i < vb2_get_num_buffers(q); ++i) {
			struct vb2_buffer *vb = vb2_get_buffer(q, i);

			if (vb && vb->state == VB2_BUF_STATE_ACTIVE)
				vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
		}
		return;
	}

	if (ctx->state >= MTK_STATE_HEADER) {

		/* Until STREAMOFF is called on the CAPTURE queue
		 * (acknowledging the event), the driver operates
		 * as if the resolution hasn't changed yet, i.e.
		 * VIDIOC_G_FMT< etc. return previous resolution.
		 * So we update picinfo here
		 */
		ctx->picinfo = ctx->last_decoded_picinfo;

		mtk_v4l2_debug(2,
				"[%d]-> new(%d,%d), old(%d,%d), real(%d,%d) bit:%d\n",
				ctx->id, ctx->last_decoded_picinfo.pic_w,
				ctx->last_decoded_picinfo.pic_h,
				ctx->picinfo.pic_w, ctx->picinfo.pic_h,
				ctx->last_decoded_picinfo.buf_w,
				ctx->last_decoded_picinfo.buf_h,
				ctx->picinfo.bitdepth);

		mtk_vdec_flush_decoder(ctx);
	}
	ctx->state = MTK_STATE_FLUSH;

	while ((dst_buf = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx))) {
		for (i = 0; i < dst_buf->vb2_buf.num_planes; i++)
			vb2_set_plane_payload(&dst_buf->vb2_buf, i, 0);
		v4l2_m2m_buf_done(dst_buf,
					VB2_BUF_STATE_ERROR);
	}

}

static void m2mops_vdec_device_run(void *priv)
{
	struct mtk_vcodec_ctx *ctx = priv;
	struct mtk_vcodec_dev *dev = ctx->dev;

	queue_work(dev->decode_workqueue, &ctx->decode_work);
}

static int m2mops_vdec_job_ready(void *m2m_priv)
{
	struct mtk_vcodec_ctx *ctx = m2m_priv;

	mtk_v4l2_debug(4, "[%d]", ctx->id);

	if (ctx->state == MTK_STATE_ABORT)
		return 0;

	if ((ctx->last_decoded_picinfo.pic_w != ctx->picinfo.pic_w) ||
	    (ctx->last_decoded_picinfo.pic_h != ctx->picinfo.pic_h))
		return 0;

	if (ctx->state != MTK_STATE_HEADER)
		return 0;

	return 1;
}

static void m2mops_vdec_job_abort(void *priv)
{
	struct mtk_vcodec_ctx *ctx = priv;

	ctx->state = MTK_STATE_ABORT;
}

static int mtk_vdec_g_v_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mtk_vcodec_ctx *ctx = ctrl_to_ctx(ctrl);
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_MIN_BUFFERS_FOR_CAPTURE:
		if (ctx->state >= MTK_STATE_HEADER) {
			ctrl->val = ctx->dpb_size;
		} else {
			mtk_v4l2_debug(0, "Seqinfo not ready");
			ctrl->val = 0;
		}
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}

static const struct v4l2_ctrl_ops mtk_vcodec_dec_ctrl_ops = {
	.g_volatile_ctrl = mtk_vdec_g_v_ctrl,
};

int mtk_vcodec_dec_ctrls_setup(struct mtk_vcodec_ctx *ctx)
{
	struct v4l2_ctrl *ctrl;

	v4l2_ctrl_handler_init(&ctx->ctrl_hdl, 1);

	/* g_volatile_ctrl */
	ctrl = v4l2_ctrl_new_std(&ctx->ctrl_hdl,
				&mtk_vcodec_dec_ctrl_ops,
				V4L2_CID_MIN_BUFFERS_FOR_CAPTURE,
				0, 32, 1, 1);
	ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE;

	if (ctx->ctrl_hdl.error) {
		mtk_v4l2_err("Adding control failed %d",
				ctx->ctrl_hdl.error);
		return ctx->ctrl_hdl.error;
	}

	v4l2_ctrl_handler_setup(&ctx->ctrl_hdl);
	return 0;
}

static int vb2ops_vdec_buf_out_validate(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	vbuf->field = V4L2_FIELD_NONE;
	return 0;
}

const struct v4l2_m2m_ops mtk_vdec_m2m_ops = {
	.device_run	= m2mops_vdec_device_run,
	.job_ready	= m2mops_vdec_job_ready,
	.job_abort	= m2mops_vdec_job_abort,
};

static const struct vb2_ops mtk_vdec_vb2_ops = {
	.queue_setup	= vb2ops_vdec_queue_setup,
	.buf_out_validate	= vb2ops_vdec_buf_out_validate,
	.buf_prepare	= vb2ops_vdec_buf_prepare,
	.buf_queue	= vb2ops_vdec_buf_queue,
	.wait_prepare	= vb2_ops_wait_prepare,
	.wait_finish	= vb2_ops_wait_finish,
	.buf_init	= vb2ops_vdec_buf_init,
	.buf_finish	= vb2ops_vdec_buf_finish,
	.start_streaming	= vb2ops_vdec_start_streaming,
	.stop_streaming	= vb2ops_vdec_stop_streaming,
};

const struct v4l2_ioctl_ops mtk_vdec_ioctl_ops = {
	.vidioc_streamon	= v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff	= v4l2_m2m_ioctl_streamoff,
	.vidioc_reqbufs		= v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf	= v4l2_m2m_ioctl_querybuf,
	.vidioc_expbuf		= v4l2_m2m_ioctl_expbuf,

	.vidioc_qbuf		= vidioc_vdec_qbuf,
	.vidioc_dqbuf		= vidioc_vdec_dqbuf,

	.vidioc_try_fmt_vid_cap_mplane	= vidioc_try_fmt_vid_cap_mplane,
	.vidioc_try_fmt_vid_out_mplane	= vidioc_try_fmt_vid_out_mplane,

	.vidioc_s_fmt_vid_cap_mplane	= vidioc_vdec_s_fmt_cap,
	.vidioc_s_fmt_vid_out_mplane	= vidioc_vdec_s_fmt_out,
	.vidioc_g_fmt_vid_cap_mplane	= vidioc_vdec_g_fmt,
	.vidioc_g_fmt_vid_out_mplane	= vidioc_vdec_g_fmt,

	.vidioc_create_bufs		= v4l2_m2m_ioctl_create_bufs,

	.vidioc_enum_fmt_vid_cap = vidioc_vdec_enum_fmt_vid_cap_mplane,
	.vidioc_enum_fmt_vid_out = vidioc_vdec_enum_fmt_vid_out_mplane,
	.vidioc_enum_framesizes	= vidioc_enum_framesizes,

	.vidioc_querycap		= vidioc_vdec_querycap,
	.vidioc_subscribe_event		= vidioc_vdec_subscribe_evt,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,
	.vidioc_g_selection             = vidioc_vdec_g_selection,
	.vidioc_s_selection             = vidioc_vdec_s_selection,

	.vidioc_decoder_cmd = vidioc_decoder_cmd,
	.vidioc_try_decoder_cmd = v4l2_m2m_ioctl_try_decoder_cmd,
};

int mtk_vcodec_dec_queue_init(void *priv, struct vb2_queue *src_vq,
			   struct vb2_queue *dst_vq)
{
	struct mtk_vcodec_ctx *ctx = priv;
	int ret = 0;

	mtk_v4l2_debug(4, "[%d]", ctx->id);

	src_vq->type		= V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes	= VB2_DMABUF | VB2_MMAP;
	src_vq->drv_priv	= ctx;
	src_vq->buf_struct_size = sizeof(struct mtk_video_dec_buf);
	src_vq->ops		= &mtk_vdec_vb2_ops;
	src_vq->mem_ops		= &vb2_dma_contig_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock		= &ctx->dev->dev_mutex;
	src_vq->dev             = &ctx->dev->plat_dev->dev;

	ret = vb2_queue_init(src_vq);
	if (ret) {
		mtk_v4l2_err("Failed to initialize videobuf2 queue(output)");
		return ret;
	}
	dst_vq->type		= V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes	= VB2_DMABUF | VB2_MMAP;
	dst_vq->drv_priv	= ctx;
	dst_vq->buf_struct_size = sizeof(struct mtk_video_dec_buf);
	dst_vq->ops		= &mtk_vdec_vb2_ops;
	dst_vq->mem_ops		= &vb2_dma_contig_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock		= &ctx->dev->dev_mutex;
	dst_vq->dev             = &ctx->dev->plat_dev->dev;

	ret = vb2_queue_init(dst_vq);
	if (ret) {
		vb2_queue_release(src_vq);
		mtk_v4l2_err("Failed to initialize videobuf2 queue(capture)");
	}

	return ret;
}
