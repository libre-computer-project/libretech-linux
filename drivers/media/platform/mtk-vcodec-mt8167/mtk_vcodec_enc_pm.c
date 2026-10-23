/*
* Copyright (c) 2016 MediaTek Inc.
* Author: Tiffany Lin <tiffany.lin@mediatek.com>
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

#include <linux/clk.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <soc/mediatek/smi.h>

#include "mtk_vcodec_enc_pm.h"
#include "mtk_vcodec_util.h"
#include "mtk_vpu.h"


int mtk_vcodec_init_enc_pm(struct mtk_vcodec_dev *mtkdev)
{
	struct device_node *node;
	struct platform_device *pdev;
	struct device *dev;
	struct mtk_vcodec_pm *pm;
	int ret = 0;

	pdev = mtkdev->plat_dev;
	pm = &mtkdev->pm;
	memset(pm, 0, sizeof(struct mtk_vcodec_pm));
	pm->mtkdev = mtkdev;
	pm->dev = &pdev->dev;
	dev = &pdev->dev;

	node = of_parse_phandle(dev->of_node, "mediatek,larb", 0);
	if (!node) {
		mtk_v4l2_err("no mediatek,larb found");
		return -1;
	}
	pdev = of_find_device_by_node(node);
	if (!pdev) {
		mtk_v4l2_err("no mediatek,larb device found");
		return -1;
	}
	pm->larbvenc = &pdev->dev;

	pdev = mtkdev->plat_dev;
	pm->dev = &pdev->dev;

	pm->venc_sel = devm_clk_get(&pdev->dev, "img_venc");
	if (IS_ERR(pm->venc_sel)) {
		mtk_v4l2_err("devm_clk_get img_venc fail");
		ret = PTR_ERR(pm->venc_sel);
	}

	pm_runtime_enable(&pdev->dev);

	return ret;
}

void mtk_vcodec_release_enc_pm(struct mtk_vcodec_dev *mtkdev)
{
	pm_runtime_disable(mtkdev->pm.dev);
}


void mtk_vcodec_enc_clock_on(struct mtk_vcodec_pm *pm)
{
	int ret;

	ret = pm_runtime_get_sync(pm->dev);
	if (ret)
		mtk_v4l2_err("%s power domain fail %d", __func__, ret);

	/*
	 * PORT 5.10->6.18: mtk_smi_larb_get()/put() were removed. The SMI
	 * larb is now powered via runtime-PM device links; pump its runtime
	 * PM directly.
	 */
	ret = pm_runtime_resume_and_get(pm->larbvenc);
	if (ret)
		mtk_v4l2_err("larbvenc runtime resume fail %d", ret);

	ret = clk_prepare_enable(pm->venc_sel);
	if (ret)
		mtk_v4l2_err("clk_prepare_enable fail %d", ret);
}

void mtk_vcodec_enc_clock_off(struct mtk_vcodec_pm *pm)
{
	clk_disable_unprepare(pm->venc_sel);
	/* PORT 5.10->6.18: mtk_smi_larb_put() -> pm_runtime_put_sync() */
	pm_runtime_put_sync(pm->larbvenc);
	pm_runtime_put_sync(pm->dev);
}

