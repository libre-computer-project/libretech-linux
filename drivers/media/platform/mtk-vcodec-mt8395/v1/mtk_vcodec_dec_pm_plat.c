// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2016 MediaTek Inc.
 * Author: Tiffany Lin <tiffany.lin@mediatek.com>
 */

#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <soc/mediatek/smi.h>
#include <linux/slab.h>
#include "mtk_vcodec_dec_pm.h"
#include "mtk_vcodec_dec_pm_plat.h"
#include "mtk_vcodec_util.h"
#include "mtk_vcu.h"

#if DEC_DVFS
#include <linux/pm_opp.h>
#include <linux/regulator/consumer.h>
#define STD_VDEC_FREQ 218
#define ONE_MHZ (1000*1000) /*1MHZ*/
#endif

#define MAX_VDEC_INSTANCE_NUM 256
#define VDEC_DRV_UFO_ON (1 << 0)
#define VDEC_DRV_UFO_AUO_ON (1 << 1)
#if DEC_EMI_BW
#include "mtk-interconnect.h"
#endif

void mtk_prepare_vdec_dvfs(struct mtk_vcodec_dev *dev)
{
#if DEC_DVFS
	int ret;
	struct dev_pm_opp *opp = 0;
	unsigned long freq = 0;
	int i = 0;

	ret = dev_pm_opp_of_add_table(&dev->plat_dev->dev);
	if (ret < 0) {
		mtk_v4l2_err("Failed to get opp table (%d)\n", ret);
		return;
	}

	dev->vdec_reg = devm_regulator_get(&dev->plat_dev->dev,
						"dvfsrc-vcore");
	if (IS_ERR(dev->vdec_reg)) {
		mtk_v4l2_err("%s, %d Failed to get regulator\n",
			__func__, __LINE__);
		return;
	}

	dev->vdec_freq_cnt = dev_pm_opp_get_opp_count(&dev->plat_dev->dev);
	freq = 0;
	while (!IS_ERR(opp =
		dev_pm_opp_find_freq_ceil(&dev->plat_dev->dev, &freq))) {
		dev->vdec_freqs[i] = freq;
		mtk_v4l2_debug(1, "i:%d, freq:%lu\n", i, freq);
		freq++;
		i++;
		dev_pm_opp_put(opp);
	}
#endif
}

void mtk_unprepare_vdec_dvfs(struct mtk_vcodec_dev *dev)
{
#if DEC_DVFS
#endif
}

void mtk_prepare_vdec_emi_bw(struct mtk_vcodec_dev *dev)
{
#if DEC_EMI_BW
	int i = 0;
	struct platform_device *pdev = 0;

	pdev = dev->plat_dev;
	for (i = 0; i < MTK_VDEC_PORT_NUM; i++)
		dev->vdec_qos_req[i] = 0;

	i = 0;
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l21_mc");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l21_ufo");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l21_pp");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l21_pred_rd");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l21_pred_wr");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l21_ppwrap");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l21_tile");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l21_vld");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l21_vld2");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l21_avc_mv");//9

	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l22_mc");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l22_ufo");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l22_pp");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l22_pred_rd");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l22_pred_wr");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l22_ppwrap");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l22_tile");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l22_vld");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l22_vld2");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l22_avc_mv");//19

	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l23_ufo_enc");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l23_rdma");//21

	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "lat0_vld");//22
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "lat0_vld2");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "lat0_avc_mc");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "lat0_pred_rd");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "lat0_tile");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "lat0_wdma");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "lat1_vld");//28
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "lat1_vld2");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "lat1_avc_mc");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "lat1_pred_rd");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "lat1_tile");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "lat1_wdma");//33
	for (i = 0; i < MTK_VDEC_PORT_NUM; i++) {
		mtk_v4l2_debug(3, "%s:%d, vdec_qos_req[%d]:%pK\n",
			__func__, __LINE__, i, dev->vdec_qos_req[i]);
	}
#endif
}

void mtk_unprepare_vdec_emi_bw(struct mtk_vcodec_dev *dev)
{
#if DEC_EMI_BW
#endif
}

#if DEC_DVFS
static unsigned int mtk_vdec_inst_num(struct mtk_vcodec_ctx *ctx)
{
	struct mtk_vcodec_ctx *ctx_interator;
	unsigned int inst_num = 0;

	mutex_lock(&ctx->dev->ctx_mutex);
	list_for_each_entry(ctx_interator, &ctx->dev->ctx_list, list) {
		inst_num++;
	}
	mutex_unlock(&ctx->dev->ctx_mutex);

	return inst_num;
}

static bool is_vc_case(struct mtk_vcodec_ctx *ctx)
{
	bool is_vc = false;

	if (mtk_vdec_inst_num(ctx) >= MAX_VDEC_INSTANCE_NUM)
		is_vc = true;

	return is_vc;
}

u64 mtk_vdec_dvfs_get_freq(struct mtk_vcodec_ctx *ctx)
{
	struct mtk_vcodec_ctx *ctx_interator;
	unsigned int area = 0;
	u64 freq_vdec = 218;
	unsigned int inst_num = 0;
	u32 fourcc = 0;


	mutex_lock(&ctx->dev->ctx_mutex);
	list_for_each_entry(ctx_interator, &ctx->dev->ctx_list, list) {
		area += ctx_interator->q_data[MTK_Q_DATA_DST].coded_width *
			ctx_interator->q_data[MTK_Q_DATA_DST].coded_height *
			ctx_interator->frame_rate;
		inst_num++;
	}
	mutex_unlock(&ctx->dev->ctx_mutex);

	if (area <= 1920 * 1088 * 60)
		freq_vdec = 218;
	else if (area <= 3840 * 2176 * 30)
		freq_vdec = 312;
	else if (area <= 3840 * 2176 * 60)
		freq_vdec = 458;
	else
		freq_vdec = 680;

	fourcc = ctx->q_data[MTK_Q_DATA_SRC].fmt->fourcc;
	if (((fourcc == V4L2_PIX_FMT_VP8) ||
		(fourcc == V4L2_PIX_FMT_MPEG2) ||
		(fourcc == V4L2_PIX_FMT_MPEG4) ||
		(fourcc == V4L2_PIX_FMT_H263)) &&
		(freq_vdec < 458))
		freq_vdec = 458;

	if (is_vc_case(ctx))//video conference, performance first
		freq_vdec = 680;

	return freq_vdec;
}

static unsigned long mtk_vdec_get_max_freq(struct mtk_vcodec_ctx *ctx)
{
	unsigned long max_freq = 0;
	int idx = 0;

	for (idx = 0; idx < MAX_VDEC_HW; idx++) {
		if (max_freq < ctx->dev->vdec_freq.freq[idx])
			max_freq = ctx->dev->vdec_freq.freq[idx];
	}

	return max_freq * ONE_MHZ;//convert to HZ
}
#endif

void mtk_vdec_dvfs_begin(struct mtk_vcodec_ctx *ctx, int hw_id)
{
#if DEC_DVFS
	struct dev_pm_opp *opp = NULL;
	struct platform_device *pdev = NULL;
	int low_volt = 0;
	int ret;

	pdev = ctx->dev->plat_dev;

	mutex_lock(&ctx->dev->dec_dvfs_mutex);
	ctx->dev->vdec_freq.freq[hw_id] = mtk_vdec_dvfs_get_freq(ctx);

	ctx->dev->vdec_freq.active_freq = mtk_vdec_get_max_freq(ctx);

	opp = dev_pm_opp_find_freq_ceil(&pdev->dev,
		&ctx->dev->vdec_freq.active_freq);
	if (!IS_ERR(opp))
		opp = dev_pm_opp_find_freq_floor(&pdev->dev,
			&ctx->dev->vdec_freq.active_freq);
	else {
		mtk_v4l2_debug(1, "%s:%d, get opp fail\n",
			__func__, __LINE__);
		mutex_unlock(&ctx->dev->dec_dvfs_mutex);
		return;
	}
	low_volt = dev_pm_opp_get_voltage(opp);
	dev_pm_opp_put(opp);
	mtk_v4l2_debug(2, "%s:%d, vdec_freq:%ld, low_volt:%d\n",
		__func__, __LINE__,
		ctx->dev->vdec_freq.active_freq,
		low_volt);
	if (ctx->dev->vdec_reg != 0) {
		ret = regulator_set_voltage(ctx->dev->vdec_reg,
			low_volt, INT_MAX);
		if (ret)
			mtk_v4l2_debug(0, "%s:%d, set voltage failed\n",
				__func__, __LINE__);
	}
	mutex_unlock(&ctx->dev->dec_dvfs_mutex);
#endif
}

void mtk_vdec_dvfs_end(struct mtk_vcodec_ctx *ctx, int hw_id)
{
#if DEC_DVFS
	struct dev_pm_opp *opp = NULL;
	struct platform_device *pdev = NULL;
	int low_volt = 0;
	int ret;

	if (is_vc_case(ctx))
		return;

	pdev = ctx->dev->plat_dev;

	mutex_lock(&ctx->dev->dec_dvfs_mutex);
	ctx->dev->vdec_freq.freq[hw_id] = 218;

	ctx->dev->vdec_freq.active_freq = mtk_vdec_get_max_freq(ctx);

	opp = dev_pm_opp_find_freq_ceil(&pdev->dev,
		&ctx->dev->vdec_freq.active_freq);
	if (!IS_ERR(opp))
		opp = dev_pm_opp_find_freq_floor(&pdev->dev,
			&ctx->dev->vdec_freq.active_freq);
	else {
		mtk_v4l2_debug(1, "%s:%d, get opp fail\n",
			__func__, __LINE__);
		mutex_unlock(&ctx->dev->dec_dvfs_mutex);
		return;
	}
	low_volt = dev_pm_opp_get_voltage(opp);
	dev_pm_opp_put(opp);
	mtk_v4l2_debug(2, "%s:%d, vdec_freq:%ld, low_volt:%d\n",
		__func__, __LINE__,
		ctx->dev->vdec_freq.active_freq,
		low_volt);
	if (ctx->dev->vdec_reg != 0) {
		ret = regulator_set_voltage(ctx->dev->vdec_reg,
			low_volt, INT_MAX);
		if (ret)
			mtk_v4l2_debug(0, "%s:%d, set voltage failed\n",
				__func__, __LINE__);
	}
	mutex_unlock(&ctx->dev->dec_dvfs_mutex);
#endif
}

void mtk_vdec_emi_bw_begin(struct mtk_vcodec_ctx *ctx, int hw_id)
{
#if DEC_EMI_BW
	long emi_bw = 0;
	long emi_bw_input = 0;
	long emi_bw_output = 0;
	struct mtk_vcodec_dev *pdev = 0;
	unsigned int    coded_width = 0;
	unsigned int    coded_height = 0;
	unsigned int    larb21_bw, larb22_bw;
	unsigned int    larb23_bw, larb24_bw;

	larb21_bw = larb22_bw = larb23_bw = larb24_bw = 0;
	pdev = ctx->dev;
	coded_width = ctx->q_data[MTK_Q_DATA_DST].coded_width;
	coded_height = ctx->q_data[MTK_Q_DATA_DST].coded_height;

	if (is_vc_case(ctx)) {
		mtk_icc_set_bw(pdev->vdec_qos_req[22],
			MBps_to_icc(12000), 0);
		return;
	}

	if (hw_id == MTK_VDEC_LAT) {
		switch (ctx->q_data[MTK_Q_DATA_SRC].fmt->fourcc) {
		case V4L2_PIX_FMT_H264:
		case V4L2_PIX_FMT_HEVC:
			emi_bw_input = 31L * pdev->vdec_freq.active_freq /
					1000000 / STD_VDEC_FREQ;
			break;
		case V4L2_PIX_FMT_VP9:
		case V4L2_PIX_FMT_AV1:
			emi_bw_input = 13L * pdev->vdec_freq.active_freq /
					1000000 / STD_VDEC_FREQ;
			break;
		default:
			emi_bw_input = 31L * pdev->vdec_freq.active_freq /
					1000000 / STD_VDEC_FREQ;
		}

		if (pdev->vdec_qos_req[22] != 0) {
			//lat
			larb24_bw = emi_bw_input * 5 + 10;
			mtk_icc_set_bw(pdev->vdec_qos_req[22],
					MBps_to_icc(larb24_bw), 0);
		}
	} else if (hw_id == MTK_VDEC_CORE) {
		emi_bw = 8L * 1920 * 1088 * 9 * 9 * 5 *
			pdev->vdec_freq.active_freq / 1000000 / 2 / 3;
		emi_bw_output = 1920L * 1088 * 9 * 30 * 9 * 5 *
				pdev->vdec_freq.active_freq / 1000000 /
				4 / 3 / 3 / STD_VDEC_FREQ / 1024 / 1024;

		switch (ctx->q_data[MTK_Q_DATA_SRC].fmt->fourcc) {
		case V4L2_PIX_FMT_H264:
			emi_bw_input = 62L * pdev->vdec_freq.active_freq /
					1000000 / STD_VDEC_FREQ;
			emi_bw = emi_bw * 24 /
					(2 * STD_VDEC_FREQ);
			break;
		case V4L2_PIX_FMT_HEVC:
			emi_bw_input = 62L * pdev->vdec_freq.active_freq /
					1000000 / STD_VDEC_FREQ;
			emi_bw = emi_bw * 24 /
					(2 * STD_VDEC_FREQ);
			break;
		case V4L2_PIX_FMT_VP8:
			emi_bw_input = 13L * pdev->vdec_freq.active_freq /
					1000000 / STD_VDEC_FREQ;
			emi_bw = emi_bw * 24 /
					(2 * STD_VDEC_FREQ);
			break;
		case V4L2_PIX_FMT_VP9:
			emi_bw_input = 26L * pdev->vdec_freq.active_freq /
					1000000 / STD_VDEC_FREQ;
			emi_bw = emi_bw * 24 /
					(2 * STD_VDEC_FREQ);
			break;
		case V4L2_PIX_FMT_AV1:
			emi_bw_input = 26L * pdev->vdec_freq.active_freq /
					10000000 / STD_VDEC_FREQ;
			emi_bw = emi_bw * 24 /
					(2 * STD_VDEC_FREQ);
			break;
		case V4L2_PIX_FMT_MPEG4:
		case V4L2_PIX_FMT_H263:
		case V4L2_PIX_FMT_MPEG1:
		case V4L2_PIX_FMT_MPEG2:
			emi_bw_input = 13L * pdev->vdec_freq.active_freq /
					10000000 / STD_VDEC_FREQ;
			emi_bw = emi_bw * 20 /
					(2 * STD_VDEC_FREQ);
			break;
		}
		emi_bw = emi_bw / (1024 * 1024) / 8;
		emi_bw = emi_bw - emi_bw_output - emi_bw_input;
		if (emi_bw < 0)
			emi_bw = 0;

		if (pdev->vdec_qos_req[0] != 0) {
			if (ctx->picinfo.layout_mode & VDEC_DRV_UFO_AUO_ON ||
				ctx->picinfo.layout_mode & VDEC_DRV_UFO_ON) {
				larb21_bw += emi_bw * 2;
				larb22_bw += emi_bw * 2;
				larb23_bw += emi_bw;
			} else {
				larb21_bw += emi_bw * 2;
				larb22_bw += emi_bw * 2;
			}
			larb21_bw += 1;
			larb22_bw += 1;
			if ((ctx->q_data[MTK_Q_DATA_SRC].fmt->fourcc ==
				V4L2_PIX_FMT_AV1) ||
				(ctx->q_data[MTK_Q_DATA_SRC].fmt->fourcc ==
				V4L2_PIX_FMT_VP9)) {
				larb21_bw += emi_bw;
				larb22_bw += emi_bw;
			} else {
				larb21_bw += 1;
				larb22_bw += 1;
			}
			larb21_bw += emi_bw_input * 2;
			larb22_bw += emi_bw_input * 2;
			if (pdev->vdec_qos_req[0] != 0) {
				mtk_icc_set_bw(pdev->vdec_qos_req[0],
						MBps_to_icc(larb21_bw), 0);
				if (pdev->vdec_qos_req[10] != 0) {
					mtk_icc_set_bw(pdev->vdec_qos_req[10],
							MBps_to_icc(larb22_bw + larb23_bw), 0);
				}
			}
		}
	} else {
		pr_debug("%s unknown hw_id %d\n", __func__, hw_id);
	}
#endif
}

void mtk_vdec_emi_bw_end(struct mtk_vcodec_ctx *ctx, int hw_id)
{
#if DEC_EMI_BW
	if (is_vc_case(ctx))
		return;
	if (hw_id == MTK_VDEC_LAT) {
		mtk_icc_set_bw(ctx->dev->vdec_qos_req[22],
			MBps_to_icc(0), 0);
	} else if (hw_id == MTK_VDEC_CORE) {
		mtk_icc_set_bw(ctx->dev->vdec_qos_req[0],
			MBps_to_icc(0), 0);
		mtk_icc_set_bw(ctx->dev->vdec_qos_req[10],
			MBps_to_icc(0), 0);
	}
#endif
}

void mtk_vdec_pmqos_prelock(struct mtk_vcodec_ctx *ctx, int hw_id)
{
#if DEC_DVFS
	mutex_lock(&ctx->dev->dec_dvfs_mutex);
	/* add_job(&ctx->id, &vdec_jobs); */
	mutex_unlock(&ctx->dev->dec_dvfs_mutex);
#endif
}

void mtk_vdec_pmqos_begin_frame(struct mtk_vcodec_ctx *ctx, int hw_id)
{
	mtk_vdec_dvfs_begin(ctx, hw_id);
	mtk_vdec_emi_bw_begin(ctx, hw_id);
}

void mtk_vdec_pmqos_end_frame(struct mtk_vcodec_ctx *ctx, int hw_id)
{
	mtk_vdec_dvfs_end(ctx, hw_id);
	mtk_vdec_emi_bw_end(ctx, hw_id);
}

