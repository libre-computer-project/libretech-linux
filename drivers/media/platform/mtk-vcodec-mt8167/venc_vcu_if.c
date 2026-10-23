/*
 * Copyright (c) 2016 MediaTek Inc.
 * Author: PoChun Lin <pochun.lin@mediatek.com>
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
#include "mtk_vpu.h"
#include "venc_ipi_msg.h"
#include "venc_vcu_if.h"
#include "mtk_vcodec_intr.h"
#include "mtk_vcodec_enc_pm.h"
#include "mtk_vcodec_enc.h"


static void handle_enc_init_msg(struct venc_vcu_inst *vcu, void *data)
{
	struct venc_vcu_ipi_msg_init *msg = data;

	vcu->inst_addr = msg->vcu_inst_addr;
	vcu->vsi = vpu_mapping_dm_addr(vcu->dev, msg->vcu_inst_addr);
}

static void handle_enc_encode_msg(struct venc_vcu_inst *vcu, void *data)
{
	struct venc_vcu_ipi_msg_enc *msg = data;

	vcu->state = msg->state;
	vcu->bs_size = msg->bs_size;
	vcu->is_key_frm = msg->is_key_frm;
}

static void handle_enc_waitisr_msg(struct venc_vcu_inst *vcu,
	void *data, uint32_t timeout)
{
	struct venc_vcu_ipi_msg_waitisr *msg = data;
	struct mtk_vcodec_ctx *ctx = vcu->ctx;

	msg->irq_status = ctx->irq_status;
	msg->timeout = timeout;
}

static int vcu_enc_ipi_handler(void *data, unsigned int len, void *priv)
{
	struct venc_vcu_ipi_msg_common *msg = data;
	struct venc_vcu_inst *vcu =
		(struct venc_vcu_inst *)(unsigned long)msg->venc_inst;
	struct mtk_vcodec_ctx *ctx = vcu->ctx;
	int ret = 0;
	unsigned long flags;

	mtk_vcodec_debug(vcu, "msg_id %x inst %p status %d",
			 msg->msg_id, vcu, msg->status);

	switch (msg->msg_id) {
	case VCU_IPIMSG_ENC_INIT_DONE:
		handle_enc_init_msg(vcu, data);
		break;
	case VCU_IPIMSG_ENC_SET_PARAM_DONE:
		break;
	case VCU_IPIMSG_ENC_ENCODE_DONE:
		handle_enc_encode_msg(vcu, data);
		break;
	case VCU_IPIMSG_ENC_DEINIT_DONE:
		break;
	case VCU_IPIMSG_ENC_POWER_ON:
		mtk_venc_lock(ctx);
		spin_lock_irqsave(&ctx->dev->irqlock, flags);
		ctx->dev->curr_ctx = ctx;
		spin_unlock_irqrestore(&ctx->dev->irqlock, flags);
		enable_irq(ctx->dev->enc_irq);
		mtk_vcodec_enc_clock_on(&ctx->dev->pm);
		ret = 1;
		break;
	case VCU_IPIMSG_ENC_POWER_OFF:
		mtk_vcodec_enc_clock_off(&ctx->dev->pm);
		disable_irq(ctx->dev->enc_irq);
		spin_lock_irqsave(&ctx->dev->irqlock, flags);
		ctx->dev->curr_ctx = NULL;
		spin_unlock_irqrestore(&ctx->dev->irqlock, flags);
		mtk_venc_unlock(ctx);
		ret = 1;
		break;
	case VCU_IPIMSG_ENC_WAIT_ISR:
		if (-1 == mtk_vcodec_wait_for_done_ctx(ctx,
				MTK_INST_IRQ_RECEIVED,
				WAIT_INTR_TIMEOUT_MS)) {
			handle_enc_waitisr_msg(vcu, data, 1);
			mtk_vcodec_debug(vcu,
				"irq_status %x <-", ctx->irq_status);
		} else
			handle_enc_waitisr_msg(vcu, data, 0);
		ret = 1;
		break;
	default:
		mtk_vcodec_err(vcu, "unknown msg id %x", msg->msg_id);
		break;
	}

	vcu->signaled = 1;
	vcu->failure = (msg->status != VENC_IPI_MSG_STATUS_OK);

	mtk_vcodec_debug_leave(vcu);
	return ret;
}

static int vcu_enc_send_msg(struct venc_vcu_inst *vcu, void *msg,
			    int len)
{
	int status;

	mtk_vcodec_debug_enter(vcu);

	if (!vcu->dev) {
		mtk_vcodec_err(vcu, "inst dev is NULL");
		return -EINVAL;
	}

	status = vpu_ipi_send(vcu->dev, vcu->id, msg, len);
	if (status) {
		mtk_vcodec_err(vcu, "vcu_ipi_send msg_id %x len %d fail %d",
			       *(uint32_t *)msg, len, status);
		return -EINVAL;
	}
	if (vcu->failure)
		return -EINVAL;

	mtk_vcodec_debug_leave(vcu);

	return 0;
}

int vcu_enc_init(struct venc_vcu_inst *vcu)
{
	int status;
	struct venc_ap_ipi_msg_init out;

	mtk_vcodec_debug_enter(vcu);

	init_waitqueue_head(&vcu->wq_hd);
	vcu->signaled = 0;
	vcu->failure = 0;

	status = vpu_ipi_register(vcu->dev, vcu->id, vcu_enc_ipi_handler,
				  NULL, NULL);
	if (status) {
		mtk_vcodec_err(vcu, "vcu_ipi_register fail %d", status);
		return -EINVAL;
	}

	memset(&out, 0, sizeof(out));
	out.msg_id = AP_IPIMSG_ENC_INIT;
	out.venc_inst = (unsigned long)vcu;
	if (vcu_enc_send_msg(vcu, &out, sizeof(out))) {
		mtk_vcodec_err(vcu, "AP_IPIMSG_ENC_INIT fail");
		return -EINVAL;
	}

	mtk_vcodec_debug_leave(vcu);

	return 0;
}

int vcu_enc_set_param(struct venc_vcu_inst *vcu,
		      enum venc_set_param_type id,
		      struct venc_enc_param *enc_param)
{
	struct venc_ap_ipi_msg_set_param out;

	mtk_vcodec_debug(vcu, "id %d ->", id);

	memset(&out, 0, sizeof(out));
	out.msg_id = AP_IPIMSG_ENC_SET_PARAM;
	out.vcu_inst_addr = vcu->inst_addr;
	out.param_id = id;
	switch (id) {
	case VENC_SET_PARAM_ENC:
		out.data_item = 0;
		break;
	case VENC_SET_PARAM_FORCE_INTRA:
		out.data_item = 0;
		break;
	case VENC_SET_PARAM_ADJUST_BITRATE:
		out.data_item = 1;
		out.data[0] = enc_param->bitrate;
		break;
	case VENC_SET_PARAM_ADJUST_FRAMERATE:
		out.data_item = 1;
		out.data[0] = enc_param->frm_rate;
		break;
	case VENC_SET_PARAM_GOP_SIZE:
		out.data_item = 1;
		out.data[0] = enc_param->gop_size;
		break;
	case VENC_SET_PARAM_INTRA_PERIOD:
		out.data_item = 1;
		out.data[0] = enc_param->intra_period;
		break;
	case VENC_SET_PARAM_SKIP_FRAME:
		out.data_item = 0;
		break;
	case VENC_SET_PARAM_PREPEND_HEADER:
		out.data_item = 0;
		break;
	default:
		mtk_vcodec_err(vcu, "id %d not supported", id);
		return -EINVAL;
	}

	if (vcu_enc_send_msg(vcu, &out, sizeof(out))) {
		mtk_vcodec_err(vcu,
			       "AP_IPIMSG_ENC_SET_PARAM %d fail", id);
		return -EINVAL;
	}

	mtk_vcodec_debug(vcu, "id %d <-", id);

	return 0;
}

int vcu_enc_encode(struct venc_vcu_inst *vcu, unsigned int bs_mode,
		   struct venc_frm_buf *frm_buf,
		   struct mtk_vcodec_mem *bs_buf,
		   unsigned int *bs_size)
{
	struct venc_ap_ipi_msg_enc out;
	unsigned int i;

	mtk_vcodec_debug(vcu, "bs_mode %d ->", bs_mode);

	memset(&out, 0, sizeof(out));
	out.msg_id = AP_IPIMSG_ENC_ENCODE;
	out.vcu_inst_addr = vcu->inst_addr;
	out.bs_mode = bs_mode;
	if (frm_buf) {
		out.fb_num_planes = frm_buf->num_planes;
		for (i = 0; i < frm_buf->num_planes; i++) {
			if (frm_buf->fb_addr[i].dma_addr % 16 == 0U) {
				out.input_addr[i] =
					frm_buf->fb_addr[i].dma_addr;
				out.input_size[i] =
					frm_buf->fb_addr[i].size;
			} else {
				mtk_vcodec_err(vcu,
					"planes[%d] dma_addr need 16 align"
					, i);
				return -EINVAL;
			}
		}
	}

	if (bs_buf) {
		out.bs_addr = bs_buf->dma_addr;
		out.bs_size = bs_buf->size;
	}

	if (vcu_enc_send_msg(vcu, &out, sizeof(out))) {
		mtk_vcodec_err(vcu, "AP_IPIMSG_ENC_ENCODE %d fail",
			       bs_mode);
		return -EINVAL;
	}

	mtk_vcodec_debug(vcu, "bs_mode %d state %d size %d key_frm %d <-",
			 bs_mode, vcu->state, vcu->bs_size, vcu->is_key_frm);

	return 0;
}

int vcu_enc_deinit(struct venc_vcu_inst *vcu)
{
	struct venc_ap_ipi_msg_deinit out;

	mtk_vcodec_debug_enter(vcu);

	memset(&out, 0, sizeof(out));
	out.msg_id = AP_IPIMSG_ENC_DEINIT;
	out.vcu_inst_addr = vcu->inst_addr;
	if (vcu_enc_send_msg(vcu, &out, sizeof(out))) {
		mtk_vcodec_err(vcu, "AP_IPIMSG_ENC_DEINIT fail");
		return -EINVAL;
	}

	mtk_vcodec_debug_leave(vcu);

	return 0;
}
