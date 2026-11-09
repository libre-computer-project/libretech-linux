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

#include "mtk_vcodec_dec_pm.h"
#include "mtk_vcodec_util.h"
#include "mtk_vpu.h"

int mtk_vcodec_init_dec_pm(struct mtk_vcodec_dev *mtkdev)
{
	struct device_node *node;
	struct platform_device *pdev;
	struct mtk_vcodec_pm *pm;
	int ret = 0;

	pdev = mtkdev->plat_dev;
	pm = &mtkdev->pm;
	pm->mtkdev = mtkdev;
	pm->chip_node = of_find_compatible_node(NULL, NULL, "mediatek,mt8173-vcodec-dec");
	node = of_parse_phandle(pdev->dev.of_node, "mediatek,larb", 0);
	if (!node) {
		mtk_v4l2_err("of_parse_phandle mediatek,larb fail!");
		return -1;
	}

	pdev = of_find_device_by_node(node);
	if (WARN_ON(!pdev)) {
		of_node_put(node);
		return -1;
	}
	pm->larbvdec = &pdev->dev;
	pdev = mtkdev->plat_dev;
	pm->dev = &pdev->dev;

	pm->vcodecpll = devm_clk_get(&pdev->dev, "vdec_sel");
	if (IS_ERR(pm->vcodecpll)) {
		mtk_v4l2_err("devm_clk_get vcodecpll fail");
		ret = PTR_ERR(pm->vcodecpll);
	}

	pm->univpll_d2 = devm_clk_get(&pdev->dev, "normal");
	if (IS_ERR(pm->univpll_d2)) {
		mtk_v4l2_err("devm_clk_get univpll_d2 fail");
		ret = PTR_ERR(pm->univpll_d2);
	}

	pm_runtime_enable(&pdev->dev);

	return ret;
}

void mtk_vcodec_release_dec_pm(struct mtk_vcodec_dev *dev)
{
	pm_runtime_disable(dev->pm.dev);
}

void mtk_vcodec_dec_pw_on(struct mtk_vcodec_pm *pm)
{
	int ret;

	ret = pm_runtime_get_sync(pm->dev);
	if (ret)
		mtk_v4l2_err("pm_runtime_get_sync fail %d", ret);
}

void mtk_vcodec_dec_pw_off(struct mtk_vcodec_pm *pm)
{
	int ret;

	ret = pm_runtime_put_sync(pm->dev);
	if (ret)
		mtk_v4l2_err("pm_runtime_put_sync fail %d", ret);
}

void mtk_vcodec_dec_clock_on(struct mtk_vcodec_pm *pm)
{
	int ret;

	if (pm->chip_node) {
		ret = clk_set_rate(pm->vcodecpll, 1482 * 1000000);
		if (ret)
			mtk_v4l2_err("clk_set_rate vcodecpll fail %d", ret);

		ret = clk_set_rate(pm->vencpll, 800 * 1000000);
		if (ret)
			mtk_v4l2_err("clk_set_rate vencpll fail %d", ret);
	}

	ret = clk_prepare_enable(pm->vcodecpll);
	if (ret)
		mtk_v4l2_err("clk_prepare_enable vcodecpll fail %d", ret);

	ret = clk_prepare_enable(pm->univpll_d2);
	if (ret)
		mtk_v4l2_err("clk_prepare_enable univpll_d2 fail %d", ret);

	/*
	 * PORT 5.10->6.18: mtk_smi_larb_get()/put() were removed. The SMI
	 * larb is now powered via runtime-PM device links (the larb is a
	 * pm_runtime supplier of this device). Pump its runtime PM directly.
	 */
	ret = pm_runtime_resume_and_get(pm->larbvdec);
	if (ret)
		mtk_v4l2_err("larbvdec runtime resume fail %d", ret);

}

void mtk_vcodec_dec_clock_off(struct mtk_vcodec_pm *pm)
{
	/* PORT 5.10->6.18: mtk_smi_larb_put() -> pm_runtime_put_sync() */
	pm_runtime_put_sync(pm->larbvdec);
	clk_disable_unprepare(pm->vdec_sel);
	clk_disable_unprepare(pm->univpll_d2);
}
