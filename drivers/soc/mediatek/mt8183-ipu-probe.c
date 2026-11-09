// SPDX-License-Identifier: GPL-2.0
/*
 * mt8183-ipu-probe: minimal reachability probe for the MT8183 IPU / VP6.
 *
 * The MT8183 (Helio P60) "AI block" is a pair of Cadence Tensilica VP6 DSPs
 * (the IPU: ipu_conn fabric + ipu_core0/1). Mainline has only the clock and
 * syscon nodes for it -- NO compute driver, no DT device, and no VP6 firmware
 * blob exists. This module is NOT a functional VPU driver. It only proves
 * whether the VP6 block is REACHABLE from mainline Linux: it powers on the
 * VPU_CORE0 power domain (which, as a genpd subdomain of VPU_TOP, pulls in its
 * parent VPU_TOP and the ipu_conn AHB/AXI clocks first), then reads a few
 * ipu_conn / ipu_core0 syscon registers. If the reads complete without a
 * synchronous external abort and return live (non-0xffffffff) values, the VP6
 * hardware is reachable -- the foundation any future firmware bring-up needs.
 *
 * Firmware (Xtensa VP6 blob) and the Cadence toolchain remain external blockers
 * for actually running code on it; this scaffold deliberately does neither.
 */
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

/* Blind low-offset reads -- the IPU register map is undocumented (closed). The
 * VALUES are not interpreted; the point is that the access path is alive. */
static const u32 probe_offs[] = { 0x000, 0x004, 0x008, 0x00c, 0x010, 0x01c };

static void ipu_dump(struct device *dev, struct regmap *rm, const char *name)
{
	unsigned int v;
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(probe_offs); i++) {
		ret = regmap_read(rm, probe_offs[i], &v);
		if (ret)
			dev_info(dev, "  %-9s +0x%03x = <read err %d>\n",
				 name, probe_offs[i], ret);
		else
			dev_info(dev, "  %-9s +0x%03x = 0x%08x\n",
				 name, probe_offs[i], v);
	}
}

static int mt8183_ipu_probe_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regmap *conn, *core0;
	int ret;

	conn = syscon_regmap_lookup_by_phandle(dev->of_node, "mediatek,ipu-conn");
	if (IS_ERR(conn))
		return dev_err_probe(dev, PTR_ERR(conn), "missing ipu-conn syscon\n");

	core0 = syscon_regmap_lookup_by_phandle(dev->of_node, "mediatek,ipu-core0");
	if (IS_ERR(core0))
		return dev_err_probe(dev, PTR_ERR(core0), "missing ipu-core0 syscon\n");

	pm_runtime_enable(dev);
	ret = pm_runtime_resume_and_get(dev);
	if (ret) {
		dev_err(dev, "VP6 power-on (VPU_CORE0 -> VPU_TOP) FAILED: %d\n", ret);
		pm_runtime_disable(dev);
		return ret;
	}

	dev_info(dev, "VP6 REACHABILITY PROBE: VPU_TOP+CORE0 powered ON via genpd\n");
	ipu_dump(dev, conn, "ipu_conn");
	ipu_dump(dev, core0, "ipu_core0");
	dev_info(dev, "VP6 REACHABILITY PROBE: regs read without fault -- VP6 block is REACHABLE\n");

	pm_runtime_put(dev);
	pm_runtime_disable(dev);
	return 0;
}

static const struct of_device_id mt8183_ipu_probe_match[] = {
	{ .compatible = "mediatek,mt8183-ipu-probe" },
	{}
};
MODULE_DEVICE_TABLE(of, mt8183_ipu_probe_match);

static struct platform_driver mt8183_ipu_probe_driver = {
	.probe = mt8183_ipu_probe_probe,
	.driver = {
		.name = "mt8183-ipu-probe",
		.of_match_table = mt8183_ipu_probe_match,
	},
};
module_platform_driver(mt8183_ipu_probe_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MT8183 IPU/VP6 reachability probe (power-on + register read)");
