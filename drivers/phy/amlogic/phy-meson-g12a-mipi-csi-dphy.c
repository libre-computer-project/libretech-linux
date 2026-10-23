// SPDX-License-Identifier: GPL-2.0
/*
 * Meson G12A MIPI CSI D-PHY driver
 *
 * Copyright (C) 2018 Amlogic, Inc. All rights reserved
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>

/*
 * Module parameters -- manual overrides for debugging.
 * Value 0 = use auto/DT/computed value.
 */
static unsigned int ths_settle;
module_param(ths_settle, uint, 0644);
MODULE_PARM_DESC(ths_settle, "THS_SETTLE override (0=auto-calibrate)");

static unsigned int tclk_settle;
module_param(tclk_settle, uint, 0644);
MODULE_PARM_DESC(tclk_settle, "TCLK_SETTLE override (0=default 0x1f)");

static unsigned int ths_skip;
module_param(ths_skip, uint, 0644);
MODULE_PARM_DESC(ths_skip, "THS_SKIP override (0=default 0xa)");

static unsigned int ths_exit;
module_param(ths_exit, uint, 0644);
MODULE_PARM_DESC(ths_exit, "THS_EXIT override (0=default 0x1f)");

static unsigned int clk_lane_ctrl;
module_param(clk_lane_ctrl, uint, 0644);
MODULE_PARM_DESC(clk_lane_ctrl, "CLK_LANE_CTRL override (0=default 0x3d8)");

static unsigned int tclk_miss;
module_param(tclk_miss, uint, 0644);
MODULE_PARM_DESC(tclk_miss, "TCLK_MISS override (0=default 0x9)");

static unsigned int lane_mux0;
module_param(lane_mux0, uint, 0644);
MODULE_PARM_DESC(lane_mux0, "MUX_CTRL0 override (0=derive from data-lanes DT)");

static unsigned int lane_mux1;
module_param(lane_mux1, uint, 0644);
MODULE_PARM_DESC(lane_mux1, "MUX_CTRL1 override (0=same as MUX_CTRL0)");

/* CSI PHY registers */
#define MIPI_PHY_CTRL			0x00
#define MIPI_PHY_CLK_LANE_CTRL		0x04
#define MIPI_PHY_DATA_LANE_CTRL		0x08
#define MIPI_PHY_DATA_LANE_CTRL1	0x0c
#define MIPI_PHY_TCLK_MISS		0x10
#define MIPI_PHY_TCLK_SETTLE		0x14
#define MIPI_PHY_THS_EXIT		0x18
#define MIPI_PHY_THS_SKIP		0x1c
#define MIPI_PHY_THS_SETTLE		0x20
#define MIPI_PHY_TINIT			0x24
#define MIPI_PHY_TULPS_C		0x28
#define MIPI_PHY_TULPS_S		0x2c
#define MIPI_PHY_TMBIAS			0x30
#define MIPI_PHY_TLP_EN_W		0x34
#define MIPI_PHY_TLPOK			0x38
#define MIPI_PHY_TWD_INIT		0x3c
#define MIPI_PHY_TWD_HS			0x40
#define MIPI_PHY_STATE			0x44
#define MIPI_PHY_CLK_LANE_STS		0x50
#define MIPI_PHY_DATA_LANE0_STS		0x54
#define MIPI_PHY_DATA_LANE1_STS		0x58
#define MIPI_PHY_DATA_LANE2_STS		0x5c
#define MIPI_PHY_DATA_LANE3_STS		0x60
#define MIPI_PHY_INT_STS		0x6c
#define MIPI_PHY_MUX_CTRL0		0x184
#define MIPI_PHY_MUX_CTRL1		0x188

#define MIPI_PHY_CTRL_RST		BIT(31)
#define MIPI_PHY_LANES_MAX		4

struct phy_meson_g12a_mipi_csi_dphy_priv {
	struct device			*dev;
	struct regmap			*regmap;
	struct clk			*clk;
	struct clk			*clk_phy;
	struct phy			*analog;
	struct phy_configure_opts_mipi_dphy config;

	/* Lane mapping from DT data-lanes property (0-indexed physical lanes) */
	u8				data_lanes[MIPI_PHY_LANES_MAX];
	unsigned int			num_data_lanes;

	/* Computed MUX value from data_lanes[] */
	u32				mux_ctrl;

	/* DT override for THS_SETTLE (0 = auto-calibrate) */
	unsigned int			dt_ths_settle;
};

static const struct regmap_config phy_meson_g12a_mipi_csi_dphy_regmap_conf = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = MIPI_PHY_MUX_CTRL1 + 4,
};

/*
 * Compute MUX_CTRL0 from the data-lanes array.
 *
 * MUX_CTRL0 nibble[N] selects which physical lane feeds logical lane N.
 * For a 2-lane camera on physical lanes 2,3:
 *   nibble[0] = 2 (phys2 -> log0)
 *   nibble[1] = 3 (phys3 -> log1)
 *   nibble[2] = 0 (fill remaining)
 *   nibble[3] = 1 (fill remaining)
 *   => MUX_CTRL0 = 0x1032
 *
 * data_lanes[] contains 0-indexed physical lane numbers.
 */
static u32 compute_lane_mux(const u8 *data_lanes, unsigned int num_lanes)
{
	u32 mux = 0;
	bool used[MIPI_PHY_LANES_MAX] = {};
	unsigned int i, fill;

	/* Assign requested physical lanes to logical lanes 0..N-1 */
	for (i = 0; i < num_lanes && i < MIPI_PHY_LANES_MAX; i++) {
		u8 phys = data_lanes[i] & 0x3;

		mux |= (u32)phys << (i * 4);
		used[phys] = true;
	}

	/* Fill remaining logical lanes with unused physical lanes */
	fill = 0;
	for (i = num_lanes; i < MIPI_PHY_LANES_MAX; i++) {
		while (fill < MIPI_PHY_LANES_MAX && used[fill])
			fill++;
		if (fill < MIPI_PHY_LANES_MAX) {
			mux |= (u32)fill << (i * 4);
			used[fill] = true;
		}
	}

	return mux;
}

/*
 * THS_SETTLE auto-calibration.
 *
 * Sweeps THS_SETTLE values and picks the one that produces the cleanest
 * D-PHY data lane status. The optimal settle value allows the D-PHY to
 * properly detect HS entry on the data lanes.
 *
 * Scoring: for each settle value, read DATA_LANE0_STS and INT_STS.
 * A lane status that differs from the idle state (0x03) indicates the
 * D-PHY is responding to lane activity. Lower INT_STS error count is
 * better. The value with lane activity and fewest errors wins.
 *
 * Returns the best settle value, or the computed default if no clear
 * winner is found.
 */
static unsigned int calibrate_ths_settle(
	struct phy_meson_g12a_mipi_csi_dphy_priv *priv,
	unsigned int computed_settle)
{
	unsigned int best_val = computed_settle;
	unsigned int best_score = 0;
	unsigned int val, d0, ints;
	unsigned int lo, hi, step;

	/*
	 * Sweep range: center on computed value, extend +/- 16.
	 * The register accepts 0x01..0x7F.
	 */
	lo = (computed_settle > 16) ? computed_settle - 16 : 1;
	hi = (computed_settle + 16 < 0x7f) ? computed_settle + 16 : 0x7f;
	step = 1;

	dev_info(priv->dev, "ths_settle auto-calibrate: range 0x%x..0x%x (computed=0x%x)\n",
		 lo, hi, computed_settle);

	for (val = lo; val <= hi; val += step) {
		unsigned int score = 0;

		regmap_write(priv->regmap, MIPI_PHY_THS_SETTLE, val);

		/* Clear INT_STS and wait for errors to accumulate */
		regmap_write(priv->regmap, MIPI_PHY_INT_STS, 0);
		usleep_range(2000, 3000);

		regmap_read(priv->regmap, MIPI_PHY_DATA_LANE0_STS, &d0);
		regmap_read(priv->regmap, MIPI_PHY_INT_STS, &ints);

		/*
		 * Scoring heuristic -- prefer values where the D-PHY shows
		 * active lane state (indicating HS transitions detected):
		 * - D0 has HS-like state (0x36, 0x26, 0x47): +20
		 * - D0 != idle 0x03 and != stop 0x02: +10
		 * - INT_STS == 0 (no errors): +5
		 * - Fewer INT_STS bits set: +1 per zero bit in low 16
		 * - D0 == 0x02 (LP-11 between frames): +3
		 */
		if (d0 == 0x36 || d0 == 0x26 || d0 == 0x47)
			score += 20;
		else if (d0 != 0x03 && d0 != 0x02)
			score += 10;
		else if (d0 == 0x02)
			score += 3;

		if (ints == 0)
			score += 5;
		else
			score += (16 - hweight32(ints & 0xffff));

		if (score > best_score) {
			best_score = score;
			best_val = val;
		}

		dev_dbg(priv->dev,
			"  settle=0x%02x D0=0x%03x INT=0x%05x score=%u%s\n",
			val, d0, ints, score,
			score > best_score ? " (new best)" : "");
	}

	dev_info(priv->dev, "ths_settle auto-calibrate: best=0x%x (score=%u)\n",
		 best_val, best_score);

	return best_val;
}

static int phy_meson_g12a_mipi_csi_dphy_init(struct phy *phy)
{
	struct phy_meson_g12a_mipi_csi_dphy_priv *priv = phy_get_drvdata(phy);

	return phy_init(priv->analog);
}

static int phy_meson_g12a_mipi_csi_dphy_configure(struct phy *phy,
						   union phy_configure_opts *opts)
{
	struct phy_meson_g12a_mipi_csi_dphy_priv *priv = phy_get_drvdata(phy);
	int ret;

	ret = phy_mipi_dphy_config_validate(&opts->mipi_dphy);
	if (ret)
		return ret;

	ret = phy_configure(priv->analog, opts);
	if (ret)
		return ret;

	memcpy(&priv->config, opts, sizeof(priv->config));

	dev_info(priv->dev, "configure: hs_settle=%u lanes=%u hs_clk_rate=%lu\n",
		 priv->config.hs_settle, priv->config.lanes,
		 priv->config.hs_clk_rate);

	return 0;
}

static int phy_meson_g12a_mipi_csi_dphy_power_on(struct phy *phy)
{
	struct phy_meson_g12a_mipi_csi_dphy_priv *priv = phy_get_drvdata(phy);
	unsigned int settle_val, computed_settle;
	u32 mux_val;
	int ret;

	ret = phy_power_on(priv->analog);
	if (ret)
		return ret;

	/* Clock lane timing */
	regmap_write(priv->regmap, MIPI_PHY_CLK_LANE_CTRL,
		     clk_lane_ctrl ? clk_lane_ctrl : 0x3d8);
	regmap_write(priv->regmap, MIPI_PHY_TCLK_MISS,
		     tclk_miss ? tclk_miss : 0x9);
	regmap_write(priv->regmap, MIPI_PHY_TCLK_SETTLE,
		     tclk_settle ? tclk_settle : 0x1f);

	/* Data lane HS timing */
	regmap_write(priv->regmap, MIPI_PHY_THS_EXIT,
		     ths_exit ? ths_exit : 0x1f);
	regmap_write(priv->regmap, MIPI_PHY_THS_SKIP,
		     ths_skip ? ths_skip : 0xa);

	/* Init and bias timing */
	regmap_write(priv->regmap, MIPI_PHY_TINIT, 0x4e20);
	regmap_write(priv->regmap, MIPI_PHY_TMBIAS, 0x100);
	regmap_write(priv->regmap, MIPI_PHY_TULPS_C, 0x1000);
	regmap_write(priv->regmap, MIPI_PHY_TULPS_S, 0x100);
	regmap_write(priv->regmap, MIPI_PHY_TLP_EN_W, 0x0c);
	regmap_write(priv->regmap, MIPI_PHY_TLPOK, 0x100);
	regmap_write(priv->regmap, MIPI_PHY_TWD_INIT, 0x400000);
	regmap_write(priv->regmap, MIPI_PHY_TWD_HS, 0x400000);

	/* Data lane control */
	regmap_write(priv->regmap, MIPI_PHY_DATA_LANE_CTRL, 0x0);
	regmap_write(priv->regmap, MIPI_PHY_DATA_LANE_CTRL1,
		     0x3 | (0x1f << 2) | (0x3 << 7));

	/*
	 * Lane mux: priority order:
	 * 1. Module parameter (lane_mux0)
	 * 2. Computed from DT data-lanes property
	 * 3. Identity mapping 0x3210 (fallback)
	 */
	if (lane_mux0) {
		mux_val = lane_mux0;
		dev_info(priv->dev, "lane mux: module param override 0x%04x\n",
			 mux_val);
	} else if (priv->num_data_lanes) {
		mux_val = priv->mux_ctrl;
		dev_info(priv->dev,
			 "lane mux: from DT data-lanes 0x%04x (phys %u,%u -> log 0,1)\n",
			 mux_val, priv->data_lanes[0],
			 priv->num_data_lanes > 1 ? priv->data_lanes[1] : 0);
	} else {
		mux_val = 0x3210;
		dev_info(priv->dev, "lane mux: identity 0x3210 (no DT data-lanes)\n");
	}

	regmap_write(priv->regmap, MIPI_PHY_MUX_CTRL0, mux_val);
	regmap_write(priv->regmap, MIPI_PHY_MUX_CTRL1,
		     lane_mux1 ? lane_mux1 : mux_val);

	/* Enable PHY (clear reset) */
	regmap_write(priv->regmap, MIPI_PHY_CTRL, 0);

	/*
	 * THS_SETTLE: priority order:
	 * 1. Module parameter (ths_settle)
	 * 2. DT property (amlogic,ths-settle)
	 * 3. Auto-calibrate (sweep + score)
	 * 4. Computed from hs_settle (MIPI DPHY framework)
	 * 5. Safe default 0x1f
	 */
	computed_settle = 0x1f;
	if (priv->config.hs_settle) {
		computed_settle = (unsigned int)(priv->config.hs_settle / 5000);
		if (computed_settle < 1)
			computed_settle = 1;
		if (computed_settle > 0x7f)
			computed_settle = 0x7f;
	}

	if (ths_settle) {
		settle_val = ths_settle;
		dev_info(priv->dev, "ths_settle: module param 0x%x\n",
			 settle_val);
	} else if (priv->dt_ths_settle) {
		settle_val = priv->dt_ths_settle;
		dev_info(priv->dev, "ths_settle: DT override 0x%x\n",
			 settle_val);
	} else {
		/* Auto-calibrate around the computed value */
		settle_val = calibrate_ths_settle(priv, computed_settle);
	}

	regmap_write(priv->regmap, MIPI_PHY_THS_SETTLE, settle_val);

	dev_info(priv->dev,
		 "power_on: ths_settle=0x%x (computed=0x%x from %u ps) lanes=%u mux=0x%04x\n",
		 settle_val, computed_settle,
		 (unsigned int)priv->config.hs_settle,
		 priv->config.lanes, mux_val);

	/* Status dump */
	{
		unsigned int state, clk, d0, d1, d2, d3, ints;

		usleep_range(1000, 2000);
		regmap_read(priv->regmap, MIPI_PHY_STATE, &state);
		regmap_read(priv->regmap, MIPI_PHY_CLK_LANE_STS, &clk);
		regmap_read(priv->regmap, MIPI_PHY_DATA_LANE0_STS, &d0);
		regmap_read(priv->regmap, MIPI_PHY_DATA_LANE1_STS, &d1);
		regmap_read(priv->regmap, MIPI_PHY_DATA_LANE2_STS, &d2);
		regmap_read(priv->regmap, MIPI_PHY_DATA_LANE3_STS, &d3);
		regmap_read(priv->regmap, MIPI_PHY_INT_STS, &ints);
		dev_info(priv->dev,
			 "status: STATE=0x%x CLK=0x%x D0=0x%x D1=0x%x D2=0x%x D3=0x%x INT=0x%x\n",
			 state, clk, d0, d1, d2, d3, ints);
	}

	return 0;
}

static int phy_meson_g12a_mipi_csi_dphy_power_off(struct phy *phy)
{
	struct phy_meson_g12a_mipi_csi_dphy_priv *priv = phy_get_drvdata(phy);

	regmap_write(priv->regmap, MIPI_PHY_CTRL, MIPI_PHY_CTRL_RST);

	phy_power_off(priv->analog);

	return 0;
}

static int phy_meson_g12a_mipi_csi_dphy_exit(struct phy *phy)
{
	struct phy_meson_g12a_mipi_csi_dphy_priv *priv = phy_get_drvdata(phy);

	return phy_exit(priv->analog);
}

static const struct phy_ops phy_meson_g12a_mipi_csi_dphy_ops = {
	.configure	= phy_meson_g12a_mipi_csi_dphy_configure,
	.init		= phy_meson_g12a_mipi_csi_dphy_init,
	.exit		= phy_meson_g12a_mipi_csi_dphy_exit,
	.power_on	= phy_meson_g12a_mipi_csi_dphy_power_on,
	.power_off	= phy_meson_g12a_mipi_csi_dphy_power_off,
	.owner		= THIS_MODULE,
};

/*
 * Parse data-lanes from DT.
 *
 * The data-lanes property uses 1-indexed lane numbers per the standard
 * MIPI CSI-2 DT binding (e.g., <3 4> = physical lanes 2 and 3).
 * We convert to 0-indexed internally.
 *
 * The D-PHY node itself may have a data-lanes property, or we look
 * at the CSI-2 host's endpoint via the phy consumer.
 */
static void parse_data_lanes(struct phy_meson_g12a_mipi_csi_dphy_priv *priv)
{
	struct device_node *np = priv->dev->of_node;
	u32 lanes[MIPI_PHY_LANES_MAX];
	int num, i;

	/* Try reading data-lanes from the D-PHY node itself */
	num = of_property_read_variable_u32_array(np, "data-lanes",
						  lanes, 1,
						  MIPI_PHY_LANES_MAX);
	if (num <= 0) {
		dev_info(priv->dev,
			 "no data-lanes in DT, will use identity mux 0x3210\n");
		return;
	}

	priv->num_data_lanes = num;
	for (i = 0; i < num; i++) {
		/* Convert from 1-indexed DT to 0-indexed internal */
		if (lanes[i] < 1 || lanes[i] > MIPI_PHY_LANES_MAX) {
			dev_warn(priv->dev,
				 "invalid data-lane %u, must be 1..%u\n",
				 lanes[i], MIPI_PHY_LANES_MAX);
			priv->num_data_lanes = 0;
			return;
		}
		priv->data_lanes[i] = lanes[i] - 1;
	}

	priv->mux_ctrl = compute_lane_mux(priv->data_lanes,
					   priv->num_data_lanes);

	dev_info(priv->dev, "DT data-lanes: %u lanes, mux=0x%04x\n",
		 priv->num_data_lanes, priv->mux_ctrl);
	for (i = 0; i < num; i++)
		dev_info(priv->dev, "  logical %d <- physical %u\n",
			 i, priv->data_lanes[i]);
}

static int phy_meson_g12a_mipi_csi_dphy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy_provider *phy_provider;
	struct phy_meson_g12a_mipi_csi_dphy_priv *priv;
	struct phy *phy;
	void __iomem *base;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	platform_set_drvdata(pdev, priv);

	/*
	 * The PHY functional clock (HHI_MIPI_CSI_PHY_CLK_CNTL) must be
	 * enabled before any MMIO to the D-PHY registers. Without it,
	 * register access causes a bus error.
	 */
	priv->clk_phy = devm_clk_get(dev, "phy");
	if (IS_ERR(priv->clk_phy))
		return dev_err_probe(dev, PTR_ERR(priv->clk_phy),
				     "failed to get phy clock\n");

	ret = clk_set_rate(priv->clk_phy, 200000000);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set phy clock rate\n");

	ret = clk_prepare_enable(priv->clk_phy);
	if (ret)
		return ret;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base)) {
		ret = PTR_ERR(base);
		goto err_clk_phy;
	}

	priv->regmap = devm_regmap_init_mmio(dev, base,
				&phy_meson_g12a_mipi_csi_dphy_regmap_conf);
	if (IS_ERR(priv->regmap)) {
		ret = PTR_ERR(priv->regmap);
		goto err_clk_phy;
	}

	priv->clk = devm_clk_get(dev, "pclk");
	if (IS_ERR(priv->clk)) {
		ret = PTR_ERR(priv->clk);
		goto err_clk_phy;
	}

	priv->analog = devm_phy_get(dev, "analog");
	if (IS_ERR(priv->analog)) {
		ret = PTR_ERR(priv->analog);
		goto err_clk_phy;
	}

	ret = clk_prepare_enable(priv->clk);
	if (ret)
		goto err_clk_phy;

	/* Parse lane mapping and optional THS_SETTLE from DT */
	parse_data_lanes(priv);

	of_property_read_u32(dev->of_node, "amlogic,ths-settle",
			     &priv->dt_ths_settle);
	if (priv->dt_ths_settle)
		dev_info(dev, "DT ths-settle override: 0x%x\n",
			 priv->dt_ths_settle);

	phy = devm_phy_create(dev, NULL, &phy_meson_g12a_mipi_csi_dphy_ops);
	if (IS_ERR(phy)) {
		ret = dev_err_probe(dev, PTR_ERR(phy), "failed to create PHY\n");
		goto err_clk;
	}

	phy_set_drvdata(phy, priv);

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(phy_provider)) {
		ret = PTR_ERR(phy_provider);
		goto err_clk;
	}

	return 0;

err_clk:
	clk_disable_unprepare(priv->clk);
err_clk_phy:
	clk_disable_unprepare(priv->clk_phy);
	return ret;
}

static void phy_meson_g12a_mipi_csi_dphy_remove(struct platform_device *pdev)
{
	struct phy_meson_g12a_mipi_csi_dphy_priv *priv = platform_get_drvdata(pdev);

	clk_disable_unprepare(priv->clk);
	clk_disable_unprepare(priv->clk_phy);
}

static const struct of_device_id phy_meson_g12a_mipi_csi_dphy_of_match[] = {
	{ .compatible = "amlogic,g12a-mipi-csi-dphy", },
	{ },
};
MODULE_DEVICE_TABLE(of, phy_meson_g12a_mipi_csi_dphy_of_match);

static struct platform_driver phy_meson_g12a_mipi_csi_dphy_driver = {
	.probe	= phy_meson_g12a_mipi_csi_dphy_probe,
	.remove	= phy_meson_g12a_mipi_csi_dphy_remove,
	.driver	= {
		.name		= "phy-meson-g12a-mipi-csi-dphy",
		.of_match_table	= phy_meson_g12a_mipi_csi_dphy_of_match,
	},
};
module_platform_driver(phy_meson_g12a_mipi_csi_dphy_driver);

MODULE_AUTHOR("Da Xue <da@libre.computer>");
MODULE_DESCRIPTION("Meson G12A MIPI CSI D-PHY driver");
MODULE_LICENSE("GPL v2");
