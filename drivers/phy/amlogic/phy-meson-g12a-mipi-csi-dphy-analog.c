// SPDX-License-Identifier: GPL-2.0
/*
 * Meson G12A MIPI CSI Analog D-PHY
 *
 * Copyright (C) 2018 Amlogic, Inc. All rights reserved
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/phy/phy.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/platform_device.h>

/*
 * Runtime-tunable analog PHY registers.
 * Module params override DT which overrides computed defaults.
 * Value 0 = use DT or computed default.
 *
 * BSP CNTL0 values:
 *   0x0b440585 for >=1000 Mbps/lane (high-speed)
 *   0x0b440581 for  <1000 Mbps/lane (low-speed)
 * Difference is bit 2 (analog EQ/termination).
 */
static unsigned int cntl0;
module_param(cntl0, uint, 0644);
MODULE_PARM_DESC(cntl0, "HHI_CSI_PHY_CNTL0 override (0=auto based on data rate)");

static unsigned int cntl1;
module_param(cntl1, uint, 0644);
MODULE_PARM_DESC(cntl1, "HHI_CSI_PHY_CNTL1 override (0=default 0x803f0000)");

static unsigned int cntl2;
module_param(cntl2, uint, 0644);
MODULE_PARM_DESC(cntl2, "HHI_CSI_PHY_CNTL2 override (0=default 0xf002)");

static unsigned int cntl3;
module_param(cntl3, uint, 0644);
MODULE_PARM_DESC(cntl3, "HHI_CSI_PHY_CNTL3 override (0=default 0x38c002)");

#define HHI_CSI_PHY_CNTL0	0x34c
#define HHI_CSI_PHY_CNTL1	0x350
#define HHI_CSI_PHY_CNTL2	0x354
#define HHI_CSI_PHY_CNTL3	0x358

struct phy_g12a_mipi_csi_dphy_analog_priv {
	struct phy *phy;
	struct regmap *regmap;
	struct phy_configure_opts_mipi_dphy config;
	/* DT overrides (0 = use default) */
	u32 dt_cntl0;
	u32 dt_cntl1;
	u32 dt_cntl2;
	u32 dt_cntl3;
};

static int phy_g12a_mipi_csi_dphy_analog_configure(struct phy *phy,
						    union phy_configure_opts *opts)
{
	struct phy_g12a_mipi_csi_dphy_analog_priv *priv = phy_get_drvdata(phy);
	int ret;

	ret = phy_mipi_dphy_config_validate(&opts->mipi_dphy);
	if (ret)
		return ret;

	memcpy(&priv->config, opts, sizeof(priv->config));

	return 0;
}

/*
 * Resolve a CNTL register value from the priority chain:
 *   1. Module parameter (runtime override)
 *   2. DT property (board-specific tuning)
 *   3. Computed/default value
 */
static u32 resolve_cntl(unsigned int modparam, u32 dt_val, u32 default_val,
			const char *name, struct device *dev)
{
	if (modparam) {
		dev_info(dev, "%s: module param 0x%08x\n", name, modparam);
		return modparam;
	}
	if (dt_val) {
		dev_info(dev, "%s: DT override 0x%08x\n", name, dt_val);
		return dt_val;
	}
	return default_val;
}

static int phy_g12a_mipi_csi_dphy_analog_power_on(struct phy *phy)
{
	struct phy_g12a_mipi_csi_dphy_analog_priv *priv = phy_get_drvdata(phy);
	struct device *dev = &phy->dev;
	u32 val_cntl0, val_cntl1, val_cntl2, val_cntl3;
	u32 default_cntl0;

	/*
	 * BSP selects CNTL0 based on data rate:
	 *   ui_val = 1000 / (data_rate_mbps_per_lane)
	 *   ui_val <= 1 (>=1000 Mbps): 0x0b440585
	 *   ui_val >  1 ( <1000 Mbps): 0x0b440581
	 */
	if (priv->config.hs_clk_rate >= 1000000000ULL)
		default_cntl0 = 0x0b440585;
	else
		default_cntl0 = 0x0b440581;

	val_cntl0 = resolve_cntl(cntl0, priv->dt_cntl0, default_cntl0,
				 "CNTL0", dev);
	val_cntl1 = resolve_cntl(cntl1, priv->dt_cntl1, 0x803f0000,
				 "CNTL1", dev);
	val_cntl2 = resolve_cntl(cntl2, priv->dt_cntl2, 0xf002,
				 "CNTL2", dev);
	val_cntl3 = resolve_cntl(cntl3, priv->dt_cntl3, 0x38c002,
				 "CNTL3", dev);

	regmap_write(priv->regmap, HHI_CSI_PHY_CNTL0, val_cntl0);
	regmap_write(priv->regmap, HHI_CSI_PHY_CNTL1, val_cntl1);
	regmap_write(priv->regmap, HHI_CSI_PHY_CNTL2, val_cntl2);
	regmap_write(priv->regmap, HHI_CSI_PHY_CNTL3, val_cntl3);

	{
		u32 rb0, rb1, rb2, rb3;

		regmap_read(priv->regmap, HHI_CSI_PHY_CNTL0, &rb0);
		regmap_read(priv->regmap, HHI_CSI_PHY_CNTL1, &rb1);
		regmap_read(priv->regmap, HHI_CSI_PHY_CNTL2, &rb2);
		regmap_read(priv->regmap, HHI_CSI_PHY_CNTL3, &rb3);
		pr_info("csi-dphy-analog: CNTL0=0x%08x CNTL1=0x%08x CNTL2=0x%08x CNTL3=0x%08x (hs_clk=%lu)\n",
			rb0, rb1, rb2, rb3, priv->config.hs_clk_rate);
	}

	return 0;
}

static int phy_g12a_mipi_csi_dphy_analog_power_off(struct phy *phy)
{
	struct phy_g12a_mipi_csi_dphy_analog_priv *priv = phy_get_drvdata(phy);

	regmap_write(priv->regmap, HHI_CSI_PHY_CNTL0, 0);
	regmap_write(priv->regmap, HHI_CSI_PHY_CNTL1, 0);
	regmap_write(priv->regmap, HHI_CSI_PHY_CNTL2, 0);
	regmap_write(priv->regmap, HHI_CSI_PHY_CNTL3, 0);

	return 0;
}

static const struct phy_ops phy_g12a_mipi_csi_dphy_analog_ops = {
	.configure = phy_g12a_mipi_csi_dphy_analog_configure,
	.power_on = phy_g12a_mipi_csi_dphy_analog_power_on,
	.power_off = phy_g12a_mipi_csi_dphy_analog_power_off,
	.owner = THIS_MODULE,
};

static int phy_g12a_mipi_csi_dphy_analog_probe(struct platform_device *pdev)
{
	struct phy_provider *phy;
	struct device *dev = &pdev->dev;
	struct phy_g12a_mipi_csi_dphy_analog_priv *priv;
	struct device_node *np = dev->of_node, *parent_np;
	struct regmap *map;

	priv = devm_kmalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	/* Get the hhi system controller node */
	parent_np = of_get_parent(np);
	map = syscon_node_to_regmap(parent_np);
	of_node_put(parent_np);
	if (IS_ERR(map))
		return dev_err_probe(dev, PTR_ERR(map), "failed to get HHI regmap\n");

	priv->regmap = map;

	/*
	 * Optional DT overrides for analog PHY tuning.
	 * These allow board-specific tuning of the undocumented HHI registers
	 * without recompiling. Module params still take highest priority.
	 *
	 * Example DT:
	 *   csi_analog_dphy {
	 *       amlogic,cntl0 = <0x0b440585>;
	 *       amlogic,cntl1 = <0x803f0000>;
	 *   };
	 */
	of_property_read_u32(np, "amlogic,cntl0", &priv->dt_cntl0);
	of_property_read_u32(np, "amlogic,cntl1", &priv->dt_cntl1);
	of_property_read_u32(np, "amlogic,cntl2", &priv->dt_cntl2);
	of_property_read_u32(np, "amlogic,cntl3", &priv->dt_cntl3);

	if (priv->dt_cntl0 || priv->dt_cntl1 || priv->dt_cntl2 || priv->dt_cntl3)
		dev_info(dev, "DT analog overrides: CNTL0=0x%x CNTL1=0x%x CNTL2=0x%x CNTL3=0x%x\n",
			 priv->dt_cntl0, priv->dt_cntl1,
			 priv->dt_cntl2, priv->dt_cntl3);

	priv->phy = devm_phy_create(dev, np, &phy_g12a_mipi_csi_dphy_analog_ops);
	if (IS_ERR(priv->phy))
		return dev_err_probe(dev, PTR_ERR(priv->phy), "failed to create PHY\n");

	phy_set_drvdata(priv->phy, priv);
	dev_set_drvdata(dev, priv);

	phy = devm_of_phy_provider_register(dev, of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(phy);
}

static const struct of_device_id phy_g12a_mipi_csi_dphy_analog_of_match[] = {
	{
		.compatible = "amlogic,g12a-mipi-csi-dphy-analog",
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, phy_g12a_mipi_csi_dphy_analog_of_match);

static struct platform_driver phy_g12a_mipi_csi_dphy_analog_driver = {
	.probe = phy_g12a_mipi_csi_dphy_analog_probe,
	.driver = {
		.name = "phy-meson-g12a-mipi-csi-dphy-analog",
		.of_match_table = phy_g12a_mipi_csi_dphy_analog_of_match,
	},
};
module_platform_driver(phy_g12a_mipi_csi_dphy_analog_driver);

MODULE_AUTHOR("Da Xue <da@libre.computer>");
MODULE_DESCRIPTION("Meson G12A MIPI CSI Analog D-PHY driver");
MODULE_LICENSE("GPL v2");
