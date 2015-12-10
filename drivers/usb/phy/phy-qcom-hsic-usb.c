/*
 * Copyright (c) 2015, Baylibre SAS
 * Based on phy-qcom-hsic-usb.c
 * Copyright (c) 2015, Linaro Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/extcon.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/usb/ulpi.h>
#include <linux/usb/hcd.h>
#include <linux/usb/msm_hsusb_hw.h>
#include <linux/regmap.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/mfd/syscon.h>

#define HSPHY_ULPI_VIEWPORT	0x0170

#define USB_PHY_VDD_DIG_VOL_MIN	1000000 /* uV */
#define USB_PHY_VDD_DIG_VOL_MAX	1320000 /* uV */
#define USB_PHY_SUSP_DIG_VOL	500000  /* uV */

#define ULPI_IO_TIMEOUT_USEC	(10 * 1000)

#define HSIC_DBG1_REG		0x38
#define HSIC_CFG_REG		0x30
#define HSIC_CFG1_REG		0x31
#define HSIC_IO_CAL_PER_REG	0x33

#define HSIC_CAL_PAD_CTL	0x20C8
#define HSIC_LV_MODE		0x04
#define HSIC_PAD_CALIBRATION	0xA8

#define MSM_USB_BASE (qphy->regs)

enum vdd_levels {
	VDD_LEVEL_NONE = 0,
	VDD_LEVEL_MIN,
	VDD_LEVEL_MAX,
};

struct phy_hsic {
	struct usb_phy			phy;
	void __iomem			*regs;
	struct clk			*core_clk;
	struct clk			*alt_core_clk;
	struct clk			*phy_clk;
	struct clk			*cal_clk;
	struct clk			*iface_clk;
	struct regulator		*vdd;

	struct reset_control		*link_reset;

	int vdd_levels[3];

	struct usb_bus			*host;

	struct regmap			*tlmm;
	u32				tlmm_cfg[4];

	int				hsic_gpios[2];
	bool				hsic_gpios_en;
};

static int ulpi_read(struct usb_phy *phy, u32 reg)
{
	struct phy_hsic *qphy = container_of(phy, struct phy_hsic, phy);
	int cnt = 0;

	/* initiate read operation */
	writel(ULPI_RUN | ULPI_READ | ULPI_ADDR(reg),
	       USB_ULPI_VIEWPORT);

	/* wait for completion */
	while (cnt < ULPI_IO_TIMEOUT_USEC) {
		if (!(readl(USB_ULPI_VIEWPORT) & ULPI_RUN))
			break;
		udelay(1);
		cnt++;
	}

	if (cnt >= ULPI_IO_TIMEOUT_USEC) {
		dev_err(phy->dev, "ulpi_read: timeout %08x\n",
			readl(USB_ULPI_VIEWPORT));
		return -ETIMEDOUT;
	}
	return ULPI_DATA_READ(readl(USB_ULPI_VIEWPORT));
}

static int ulpi_write(struct usb_phy *phy, u32 val, u32 reg)
{
	struct phy_hsic *qphy = container_of(phy, struct phy_hsic, phy);
	int cnt = 0;

	/* initiate write operation */
	writel(ULPI_RUN | ULPI_WRITE |
	       ULPI_ADDR(reg) | ULPI_DATA(val),
	       USB_ULPI_VIEWPORT);

	/* wait for completion */
	while (cnt < ULPI_IO_TIMEOUT_USEC) {
		if (!(readl(USB_ULPI_VIEWPORT) & ULPI_RUN))
			break;
		udelay(1);
		cnt++;
	}

	if (cnt >= ULPI_IO_TIMEOUT_USEC) {
		dev_err(phy->dev, "ulpi_write: timeout\n");
		return -ETIMEDOUT;
	}
	return 0;
}

static struct usb_phy_io_ops qcom_hsic_io_ops = {
	.read = ulpi_read,
	.write = ulpi_write,
};

static int phy_hsic_regulators_enable(struct phy_hsic *qphy)
{
	int ret;

	ret = regulator_set_voltage(qphy->vdd,
				qphy->vdd_levels[VDD_LEVEL_MIN],
				qphy->vdd_levels[VDD_LEVEL_MAX]);
	if (ret)
		return ret;

	ret = regulator_enable(qphy->vdd);
	if (ret)
		return ret;

	return 0;
}

static void phy_hsic_regulators_disable(struct phy_hsic *qphy)
{
	regulator_disable(qphy->vdd);
}

static int phy_hsic_clock_reset(struct phy_hsic *qphy)
{
	/* Reset sequence */
	if (!IS_ERR(qphy->link_reset))
		reset_control_assert(qphy->link_reset);

	clk_disable(qphy->core_clk);
	clk_disable(qphy->alt_core_clk);

	if (!IS_ERR(qphy->link_reset))
		reset_control_deassert(qphy->link_reset);

	usleep_range(10000, 12000);

	clk_enable(qphy->core_clk);
	clk_enable(qphy->alt_core_clk);

	return 0;
}

static int phy_hsic_reset(struct phy_hsic *qphy)
{
	phy_hsic_clock_reset(qphy);

	/* select ULPI phy and clear other status/control bits in PORTSC */
	writel_relaxed(0x80000000, USB_PORTSC);

	/* Be sure PORTSC is written */
	mb();

	if (qphy->tlmm &&
	    qphy->hsic_gpios[0] > 0 &&
	    qphy->hsic_gpios[1] > 0) {

		/* Enable LV_MODE in HSIC_CAL_PAD_CTL register */
		regmap_write(qphy->tlmm, HSIC_CAL_PAD_CTL, HSIC_LV_MODE);

		/* Be sure register is written */
		mb();

		/* set periodic calibration interval to ~2.048sec */
		ulpi_write(&qphy->phy, 0xFF, HSIC_IO_CAL_PER_REG);

		/* Enable periodic IO calibration in HSIC_CFG register */
		ulpi_write(&qphy->phy, 0xA8, HSIC_CFG_REG);

		/* Configure GPIO pins for HSIC functionality mode */
		if (!qphy->hsic_gpios_en) {
			gpio_request(qphy->hsic_gpios[0], "HSIC_GPIO0");
			gpio_request(qphy->hsic_gpios[1], "HSIC_GPIO1");
			qphy->hsic_gpios_en = true;
		}

		/* Set LV_MODE=0x1 and DCC=0x2 in HSIC_GPIO PAD_CTL register */
		regmap_write(qphy->tlmm, qphy->tlmm_cfg[0],
					 qphy->tlmm_cfg[1]);
		regmap_write(qphy->tlmm, qphy->tlmm_cfg[2],
					 qphy->tlmm_cfg[3]);

		/* Enable HSIC mode in HSIC_CFG register */
		ulpi_write(&qphy->phy, 0x01, HSIC_CFG1_REG);

	} else {
		/* Setup HSIC pads */
		if (qphy->tlmm) {
			regmap_write(qphy->tlmm, qphy->tlmm_cfg[0],
						 qphy->tlmm_cfg[1]);
			regmap_write(qphy->tlmm, qphy->tlmm_cfg[2],
						 qphy->tlmm_cfg[3]);
		}

		/* programmable length of connect signaling (33.2ns) */
		ulpi_write(&qphy->phy, 3, HSIC_DBG1_REG);

		/* set periodic calibration interval to ~2.048sec */
		ulpi_write(&qphy->phy, 0xFF, HSIC_IO_CAL_PER_REG);

		/* Enable HSIC mode in HSIC_CFG register */
		ulpi_write(&qphy->phy, 0xA9, HSIC_CFG_REG);
	}

	/* Disable auto resume */
	ulpi_write(&qphy->phy, ULPI_IFC_CTRL_AUTORESUME,
			ULPI_CLR(ULPI_IFC_CTRL));

	return 0;
};

static int phy_hsic_init(struct usb_phy *phy)
{
	struct phy_hsic *qphy = container_of(phy, struct phy_hsic, phy);

	/* bursts of unspecified length. */
	writel_relaxed(0, USB_AHBBURST);

	/* Use the AHB transactor */
	writel_relaxed(0x08, USB_AHBMODE);

	/* Disable streaming mode and select host mode */
	writel_relaxed(0x13, USB_USBMODE);

	return 0;
}

static int phy_hsic_set_host(struct usb_otg *otg, struct usb_bus *host)
{
	struct phy_hsic *qphy =
		container_of(otg->usb_phy, struct phy_hsic, phy);
	struct usb_hcd *hcd;

	dev_info(otg->usb_phy->dev, "%s()\n", __func__);

	if (!host) {
		if (!qphy->host)
			return -EINVAL;

		hcd = bus_to_hcd(host);

		usb_remove_hcd(hcd);

		qphy->host = NULL;

		dev_dbg(otg->usb_phy->dev, "host off\n");

	} else {
		if (qphy->host) {
			dev_err(otg->usb_phy->dev, "host already registered\n");
			return -EFAULT;
		}

		phy_hsic_init(otg->usb_phy);

		hcd = bus_to_hcd(host);

		usb_add_hcd(hcd, hcd->irq, IRQF_SHARED);

		device_wakeup_enable(hcd->self.controller);

		dev_dbg(otg->usb_phy->dev, "host on\n");

		qphy->host = host;
	}

	return 0;
}

static int phy_hsic_read_devicetree(struct phy_hsic *qphy)
{
	struct regulator_bulk_data regs[1];
	struct device *dev = qphy->phy.dev;
	u32 tmp[3];
	int ret;
	int len;

	qphy->core_clk = devm_clk_get(dev, "core");
	if (IS_ERR(qphy->core_clk))
		return PTR_ERR(qphy->core_clk);

	qphy->alt_core_clk = devm_clk_get(dev, "alt-core");
	if (IS_ERR(qphy->alt_core_clk))
		return PTR_ERR(qphy->alt_core_clk);

	qphy->phy_clk = devm_clk_get(dev, "phy");
	if (IS_ERR(qphy->phy_clk))
		return PTR_ERR(qphy->phy_clk);

	qphy->cal_clk = devm_clk_get(dev, "cal");
	if (IS_ERR(qphy->cal_clk))
		return PTR_ERR(qphy->cal_clk);

	qphy->iface_clk = devm_clk_get(dev, "iface");
	if (IS_ERR(qphy->iface_clk))
		return PTR_ERR(qphy->iface_clk);

	regs[0].supply = "vddcx";

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(regs), regs);
	if (ret)
		return ret;

	qphy->vdd  = regs[0].consumer;

	qphy->vdd_levels[VDD_LEVEL_NONE] = USB_PHY_SUSP_DIG_VOL;
	qphy->vdd_levels[VDD_LEVEL_MIN] = USB_PHY_VDD_DIG_VOL_MIN;
	qphy->vdd_levels[VDD_LEVEL_MAX] = USB_PHY_VDD_DIG_VOL_MAX;

	if (of_get_property(dev->of_node, "qcom,vdd-levels", &len) &&
	    len == sizeof(tmp)) {
		of_property_read_u32_array(dev->of_node, "qcom,vdd-levels",
					   tmp, len / sizeof(*tmp));
		qphy->vdd_levels[VDD_LEVEL_NONE] = tmp[VDD_LEVEL_NONE];
		qphy->vdd_levels[VDD_LEVEL_MIN] = tmp[VDD_LEVEL_MIN];
		qphy->vdd_levels[VDD_LEVEL_MAX] = tmp[VDD_LEVEL_MAX];
	}

	qphy->link_reset = devm_reset_control_get(dev, "link");
	if (IS_ERR(qphy->link_reset))
		return PTR_ERR(qphy->link_reset);

	qphy->tlmm = syscon_regmap_lookup_by_phandle(dev->of_node,
						     "qcom,tlmm");
	if (!IS_ERR(qphy->tlmm) &&
	    of_get_property(dev->of_node, "qcom,tlmm-cfg", &len) &&
	    len == (4 * sizeof(u32))) {
		of_property_read_u32_array(dev->of_node, "qcom,tlmm-cfg",
					   qphy->tlmm_cfg, 4);
		dev_info(dev, "got tlmm hsic pad cfg\n");

		if (of_gpio_named_count(dev->of_node,
					"qcom,hsic-gpios") == 2) {
			qphy->hsic_gpios[0] = of_get_named_gpio(dev->of_node,
							"qcom,hsic-gpios", 0);
			qphy->hsic_gpios[1] = of_get_named_gpio(dev->of_node,
							"qcom,hsic-gpios", 1);
		}
	} else
		qphy->tlmm = NULL;

	return 0;
}

static int phy_hsic_probe(struct platform_device *pdev)
{
	struct phy_hsic *qphy;
	struct resource *res;
	struct usb_phy *phy;
	int ret;

	qphy = devm_kzalloc(&pdev->dev, sizeof(*qphy), GFP_KERNEL);
	if (!qphy)
		return -ENOMEM;

	qphy->phy.otg = devm_kzalloc(&pdev->dev, sizeof(struct usb_otg),
				     GFP_KERNEL);
	if (!qphy->phy.otg)
		return -ENOMEM;

	platform_set_drvdata(pdev, qphy);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	qphy->regs = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!qphy->regs)
		return -ENOMEM;

	phy			= &qphy->phy;
	phy->dev		= &pdev->dev;
	phy->label		= dev_name(&pdev->dev);
	phy->io_ops		= &qcom_hsic_io_ops;
	phy->type		= USB_PHY_TYPE_USB2;
	phy->init		= phy_hsic_init;

	phy->otg->usb_phy	= phy;
	phy->otg->set_host	= phy_hsic_set_host;

	ret = phy_hsic_read_devicetree(qphy);
	if (ret < 0)
		return ret;

	ret = clk_prepare_enable(qphy->core_clk);
	if (ret < 0)
		return ret;

	ret = clk_prepare_enable(qphy->phy_clk);
	if (ret < 0)
		goto off_alt;

	ret = clk_prepare_enable(qphy->cal_clk);
	if (ret < 0)
		goto off_phy;

	ret = clk_prepare_enable(qphy->iface_clk);
	if (ret < 0)
		goto off_cal;

	ret = clk_prepare_enable(qphy->alt_core_clk);
	if (ret < 0)
		goto off_core;

	ret = phy_hsic_regulators_enable(qphy);
	if (ret)
		goto off_clks;

	ret = phy_hsic_reset(qphy);
	if (ret)
		goto off_clks;

	ret = usb_add_phy_dev(&qphy->phy);
	if (ret)
		goto off_power;

	return 0;

off_power:
	phy_hsic_regulators_disable(qphy);
off_clks:
	clk_disable_unprepare(qphy->iface_clk);
off_cal:
	clk_disable_unprepare(qphy->cal_clk);
off_phy:
	clk_disable_unprepare(qphy->phy_clk);
off_alt:
	clk_disable_unprepare(qphy->alt_core_clk);
off_core:
	clk_disable_unprepare(qphy->core_clk);
	return ret;
}

static int phy_hsic_remove(struct platform_device *pdev)
{
	struct phy_hsic *qphy = platform_get_drvdata(pdev);

	usb_remove_phy(&qphy->phy);

	clk_disable_unprepare(qphy->iface_clk);
	clk_disable_unprepare(qphy->cal_clk);
	clk_disable_unprepare(qphy->phy_clk);
	clk_disable_unprepare(qphy->alt_core_clk);
	clk_disable_unprepare(qphy->core_clk);
	phy_hsic_regulators_disable(qphy);
	return 0;
}

static const struct of_device_id phy_hsic_dt_match[] = {
	{ .compatible = "qcom,usb-hsic-phy" },
	{ }
};
MODULE_DEVICE_TABLE(of, phy_hsic_dt_match);

static struct platform_driver phy_hsic_driver = {
	.probe	= phy_hsic_probe,
	.remove = phy_hsic_remove,
	.driver = {
		.name = "phy-qcom-hsic-usb",
		.of_match_table = phy_hsic_dt_match,
	},
};
module_platform_driver(phy_hsic_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Qualcomm HSIC USB transceiver driver");
