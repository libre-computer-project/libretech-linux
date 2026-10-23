/*
 * Copyright (c) 2018 MediaTek Inc.
 * Author: Longfei Wang <longfei.wang@mediatek.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/slab.h>

#include "../mtk_vcodec_drv.h"
#include "../mtk_vcodec_util.h"
#include "../mtk_vcodec_intr.h"
#include "../mtk_vcodec_enc.h"
#include "../mtk_vcodec_enc_pm.h"
#include "../venc_drv_base.h"
#include "../venc_ipi_msg.h"
#include "../venc_vcu_if.h"
#include "mtk_vpu.h"


/**
 * enum venc_bs_mode - for bs_mode argument in venc_bs_mode
 */
enum venc_bs_mode {
	VENC_BS_MODE_SPS = 0,
	VENC_BS_MODE_PPS,
	VENC_BS_MODE_SEQ_HDR,
	VENC_BS_MODE_FRAME,
	VENC_BS_MODE_FRAME_FINAL,
	VENC_BS_MODE_MAX
};

/*
 * struct venc_vcu_config - Structure for encoder configuration
 *                               AP-W/R : AP is writer/reader on this item
 *                               VCU-W/R: VCU is write/reader on this item
 * @input_fourcc: input fourcc
 * @bitrate: target bitrate (in bps)
 * @pic_w: picture width. Picture size is visible stream resolution, in pixels,
 *         to be used for display purposes; must be smaller or equal to buffer
 *         size.
 * @pic_h: picture height
 * @buf_w: buffer width. Buffer size is stream resolution in pixels aligned to
 *         hardware requirements.
 * @buf_h: buffer height
 * @gop_size: group of picture size (idr frame)
 * @intra_period: intra frame period
 * @framerate: frame rate in fps
 * @profile: as specified in standard
 * @level: as specified in standard
 * @wfd: WFD mode 1:on, 0:off
 */
struct venc_vcu_config {
	u32 input_fourcc;
	u32 bitrate;
	u32 pic_w;
	u32 pic_h;
	u32 buf_w;
	u32 buf_h;
	u32 gop_size;
	u32 intra_period;
	u32 framerate;
	u32 profile;
	u32 level;
	u32 wfd;
	u32 scenario;
};


/*
 * struct venc_vsi - Structure for VCU driver control and info share
 *                        AP-W/R : AP is writer/reader on this item
 *                        VCU-W/R: VCU is write/reader on this item
 * This structure is allocated in VCU side and shared to AP side.
 * @config: h264 encoder configuration
 * @work_bufs: working buffer information in VCU side
 * The work_bufs here is for storing the 'size' info shared to AP side.
 * The similar item in struct venc_inst is for memory allocation
 * in AP side. The AP driver will copy the 'size' from here to the one in
 * struct mtk_vcodec_mem, then invoke mtk_vcodec_mem_alloc to allocate
 * the buffer. After that, bypass the 'dma_addr' to the 'iova' field here for
 * register setting in VCU side.
 */
struct venc_vsi {
	struct venc_vcu_config config;
	unsigned int sizeimage[MTK_VCODEC_MAX_PLANES];
};

/*
 * struct venc_inst - encoder AP driver instance
 * @hw_base: encoder hardware register base
 * @work_bufs: working buffer
 * @pps_buf: buffer to store the pps bitstream
 * @work_buf_allocated: working buffer allocated flag
 * @frm_cnt: encoded frame count
 * @prepend_hdr: when the v4l2 layer send VENC_SET_PARAM_PREPEND_HEADER cmd
 *  through venc_set_param interface, it will set this flag and prepend the
 *  sps/pps in venc_encode function.
 * @vcu_inst: VCU instance to exchange information between AP and VCU
 * @vsi: driver structure allocated by VCU side and shared to AP side for
 *	 control and info share
 * @ctx: context for v4l2 layer integration
 */
struct venc_inst {
	void __iomem *hw_base;
	struct mtk_vcodec_mem pps_buf;
	bool work_buf_allocated;
	unsigned int frm_cnt;
	unsigned int prepend_hdr;
	struct venc_vcu_inst vcu_inst;
	struct venc_vsi *vsi;
	struct mtk_vcodec_ctx *ctx;
};

static unsigned int venc_h264_get_profile(struct venc_inst *inst,
				     unsigned int profile)
{
	switch (profile) {
	case V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE:
		return 66;
	case V4L2_MPEG_VIDEO_H264_PROFILE_MAIN:
		return 77;
	case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH:
		return 100;
	case V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE:
		mtk_vcodec_err(inst, "unsupported CONSTRAINED_BASELINE");
		return 0;
	case V4L2_MPEG_VIDEO_H264_PROFILE_EXTENDED:
		mtk_vcodec_err(inst, "unsupported EXTENDED");
		return 0;
	default:
		mtk_vcodec_debug(inst, "unsupported profile %d", profile);
		return 100;
	}
}

static unsigned int venc_h264_get_level(struct venc_inst *inst,
				   unsigned int level)
{
	switch (level) {
	case V4L2_MPEG_VIDEO_H264_LEVEL_1B:
		mtk_vcodec_err(inst, "unsupported 1B");
		return 0;
	case V4L2_MPEG_VIDEO_H264_LEVEL_1_0:
		return 10;
	case V4L2_MPEG_VIDEO_H264_LEVEL_1_1:
		return 11;
	case V4L2_MPEG_VIDEO_H264_LEVEL_1_2:
		return 12;
	case V4L2_MPEG_VIDEO_H264_LEVEL_1_3:
		return 13;
	case V4L2_MPEG_VIDEO_H264_LEVEL_2_0:
		return 20;
	case V4L2_MPEG_VIDEO_H264_LEVEL_2_1:
		return 21;
	case V4L2_MPEG_VIDEO_H264_LEVEL_2_2:
		return 22;
	case V4L2_MPEG_VIDEO_H264_LEVEL_3_0:
		return 30;
	case V4L2_MPEG_VIDEO_H264_LEVEL_3_1:
		return 31;
	case V4L2_MPEG_VIDEO_H264_LEVEL_3_2:
		return 32;
	case V4L2_MPEG_VIDEO_H264_LEVEL_4_0:
		return 40;
	case V4L2_MPEG_VIDEO_H264_LEVEL_4_1:
		return 41;
	case V4L2_MPEG_VIDEO_H264_LEVEL_4_2:
		return 42;
	default:
		mtk_vcodec_debug(inst, "unsupported level %d", level);
		return 31;
	}
}

static int venc_encode_header(struct venc_inst *inst,
			      struct mtk_vcodec_mem *bs_buf,
			      unsigned int *bs_size)
{
	int ret = 0;

	mtk_vcodec_debug_enter(inst);

	ret = vcu_enc_encode(&inst->vcu_inst, VENC_BS_MODE_SEQ_HDR, NULL,
			     bs_buf, bs_size);
	if (ret)
		return ret;

	*bs_size = inst->vcu_inst.bs_size;
	mtk_vcodec_debug(inst, "bs size %d <-", *bs_size);

	return ret;
}

static int venc_encode_frame(struct venc_inst *inst,
			     struct venc_frm_buf *frm_buf,
			     struct mtk_vcodec_mem *bs_buf,
			     unsigned int *bs_size)
{
	int ret = 0;

	mtk_vcodec_debug_enter(inst);

	ret = vcu_enc_encode(&inst->vcu_inst, VENC_BS_MODE_FRAME, frm_buf,
			     bs_buf, bs_size);
	if (ret)
		return ret;

	*bs_size = inst->vcu_inst.bs_size;

	++inst->frm_cnt;
	mtk_vcodec_debug(inst, "frm %d bs_size %d key_frm %d <-",
			 inst->frm_cnt, *bs_size, inst->vcu_inst.is_key_frm);

	return ret;
}

static int venc_encode_frame_final(struct venc_inst *inst,
			      struct mtk_vcodec_mem *bs_buf,
			      unsigned int *bs_size)
{
	int ret = 0;

	mtk_vcodec_debug_enter(inst);

	ret = vcu_enc_encode(&inst->vcu_inst, VENC_BS_MODE_FRAME_FINAL, NULL,
			     bs_buf, bs_size);
	if (ret)
		return ret;

	*bs_size = inst->vcu_inst.bs_size;
	mtk_vcodec_debug(inst, "bs size %d <-", *bs_size);

	return ret;
}


static int venc_init(struct mtk_vcodec_ctx *ctx, unsigned long *handle)
{
	int ret = 0;
	struct venc_inst *inst;
	u32 fourcc = ctx->q_data[MTK_Q_DATA_DST].fmt->fourcc;

	inst = kzalloc(sizeof(*inst), GFP_KERNEL);
	if (!inst)
		return -ENOMEM;

	inst->ctx = ctx;
	inst->vcu_inst.ctx = ctx;
	inst->vcu_inst.dev = ctx->dev->vcu_plat_dev;

	switch (fourcc) {
		case V4L2_PIX_FMT_H264: {
			if (ctx->oal_vcodec == 1)
				inst->vcu_inst.id = IPI_VENC_HYBRID_H264;
			else
				inst->vcu_inst.id = IPI_VENC_H264;
			break;
		}

		case V4L2_PIX_FMT_VP8: {
			inst->vcu_inst.id = IPI_VENC_VP8;
			break;
		}

		default: {
			mtk_vcodec_err(inst, "venc_init fourcc not supported");
			break;
		}
	}

	inst->hw_base = mtk_vcodec_get_reg_addr(inst->ctx, VENC_SYS);

	mtk_vcodec_debug_enter(inst);

	ret = vcu_enc_init(&inst->vcu_inst);

	inst->vsi = (struct venc_vsi *)inst->vcu_inst.vsi;

	mtk_vcodec_debug_leave(inst);

	if (ret)
		kfree(inst);
	else
		(*handle) = (unsigned long)inst;

	return ret;
}

static int venc_encode(unsigned long handle,
			   enum venc_start_opt opt,
			   struct venc_frm_buf *frm_buf,
			   struct mtk_vcodec_mem *bs_buf,
			   struct venc_done_result *result)
{
	int ret = 0;
	struct venc_inst *inst = (struct venc_inst *)handle;
	struct mtk_vcodec_ctx *ctx = inst->ctx;

	mtk_vcodec_debug(inst, "opt %d ->", opt);

	if (ctx->oal_vcodec == 0)
		enable_irq(ctx->dev->enc_irq);

	switch (opt) {
	case VENC_START_OPT_ENCODE_SEQUENCE_HEADER: {
		unsigned int bs_size_hdr;

		ret = venc_encode_header(inst, bs_buf, &bs_size_hdr);
		if (ret)
			goto encode_err;

		result->bs_size = bs_size_hdr;
		result->is_key_frm = false;
		break;
	}

	case VENC_START_OPT_ENCODE_FRAME: {
		unsigned int bs_size_hdr;

		if (inst->prepend_hdr == 1) {
			ret = venc_encode_header(inst, bs_buf, &bs_size_hdr);
			if (ret)
				goto encode_err;
			inst->prepend_hdr = 0;
		}

		ret = venc_encode_frame(inst, frm_buf, bs_buf,
					&result->bs_size);
		if (ret)
			goto encode_err;
		result->is_key_frm = inst->vcu_inst.is_key_frm;
		break;
	}

	case VENC_START_OPT_ENCODE_FRAME_FINAL: {
		ret = venc_encode_frame_final(inst, bs_buf, &result->bs_size);
		if (ret)
			goto encode_err;
		result->is_key_frm = inst->vcu_inst.is_key_frm;
		break;
	}

	default:
		mtk_vcodec_err(inst, "venc_start_opt %d not supported", opt);
		ret = -EINVAL;
		break;
	}

encode_err:

	if (ctx->oal_vcodec == 0)
		disable_irq(ctx->dev->enc_irq);
	mtk_vcodec_debug(inst, "opt %d <-", opt);

	return ret;
}

static int venc_set_param(unsigned long handle,
			      enum venc_set_param_type type,
			      struct venc_enc_param *enc_prm)
{
	int i;
	int ret = 0;
	struct venc_inst *inst = (struct venc_inst *)handle;

	mtk_vcodec_debug(inst, "->type=%d", type);

	switch (type) {
	case VENC_SET_PARAM_ENC:
		inst->vsi->config.input_fourcc = enc_prm->input_yuv_fmt;
		inst->vsi->config.bitrate = enc_prm->bitrate;
		inst->vsi->config.pic_w = enc_prm->width;
		inst->vsi->config.pic_h = enc_prm->height;
		inst->vsi->config.buf_w = enc_prm->buf_width;
		inst->vsi->config.buf_h = enc_prm->buf_height;
		inst->vsi->config.gop_size = enc_prm->gop_size;
		inst->vsi->config.framerate = enc_prm->frm_rate;
		inst->vsi->config.intra_period = enc_prm->intra_period;

		if (inst->vcu_inst.id == IPI_VENC_H264 ||
			inst->vcu_inst.id == IPI_VENC_HYBRID_H264) {
			inst->vsi->config.profile =
				venc_h264_get_profile(inst, enc_prm->h264_profile);
			inst->vsi->config.level =
				venc_h264_get_level(inst, enc_prm->h264_level);
		}
		inst->vsi->config.wfd = 0;
		ret = vcu_enc_set_param(&inst->vcu_inst, type, enc_prm);
		if (ret)
			break;

		for (i = 0; i < MTK_VCODEC_MAX_PLANES; i++) {
			enc_prm->sizeimage[i] =
				inst->vsi->sizeimage[i];
			mtk_vcodec_debug(inst, "sizeimage[%d] size=0x%x", i,
					 enc_prm->sizeimage[i]);
		}
		break;
	case VENC_SET_PARAM_PREPEND_HEADER:
		inst->prepend_hdr = 1;
		ret = vcu_enc_set_param(&inst->vcu_inst, type, enc_prm);
		break;
	default:
		ret = vcu_enc_set_param(&inst->vcu_inst, type, enc_prm);
		break;
	}

	mtk_vcodec_debug_leave(inst);

	return ret;
}

static int venc_deinit(unsigned long handle)
{
	int ret = 0;
	struct venc_inst *inst = (struct venc_inst *)handle;

	mtk_vcodec_debug_enter(inst);

	ret = vcu_enc_deinit(&inst->vcu_inst);

	mtk_vcodec_debug_leave(inst);
	kfree(inst);

	return ret;
}

static const struct venc_common_if venc_if = {
	.init = venc_init,
	.encode = venc_encode,
	.set_param = venc_set_param,
	.deinit = venc_deinit,
};

const struct venc_common_if *get_enc_common_if(void);

const struct venc_common_if *get_enc_common_if(void)
{
	return &venc_if;
}
