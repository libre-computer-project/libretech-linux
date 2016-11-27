/*
 * Driver for Broadcom BCM2835 soc temperature sensor
 *
 * Copyright (C) 2016 Martin Sperl
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>

#define BCM2835_TS_TSENSCTL			0x00
#define BCM2835_TS_TSENSSTAT			0x04

#define BCM2835_TS_TSENSCTL_PRWDW		BIT(0)
#define BCM2835_TS_TSENSCTL_RSTB		BIT(1)
#define BCM2835_TS_TSENSCTL_CTRL_BITS		3
#define BCM2835_TS_TSENSCTL_CTRL_SHIFT		2
#define BCM2835_TS_TSENSCTL_CTRL_MASK		    \
	GENMASK(BCM2835_TS_TSENSCTL_CTRL_BITS +     \
		BCM2835_TS_TSENSCTL_CTRL_SHIFT - 1, \
		BCM2835_TS_TSENSCTL_CTRL_SHIFT)
#define BCM2835_TS_TSENSCTL_CTRL_DEFAULT	1
#define BCM2835_TS_TSENSCTL_EN_INT		BIT(5)
#define BCM2835_TS_TSENSCTL_DIRECT		BIT(6)
#define BCM2835_TS_TSENSCTL_CLR_INT		BIT(7)
#define BCM2835_TS_TSENSCTL_THOLD_SHIFT		8
#define BCM2835_TS_TSENSCTL_THOLD_BITS		10
#define BCM2835_TS_TSENSCTL_THOLD_MASK		     \
	GENMASK(BCM2835_TS_TSENSCTL_THOLD_BITS +     \
		BCM2835_TS_TSENSCTL_THOLD_SHIFT - 1, \
		BCM2835_TS_TSENSCTL_THOLD_SHIFT)
#define BCM2835_TS_TSENSCTL_RSTDELAY_SHIFT	18
#define BCM2835_TS_TSENSCTL_RSTDELAY_BITS	8
#define BCM2835_TS_TSENSCTL_REGULEN		BIT(26)

#define BCM2835_TS_TSENSSTAT_DATA_BITS		10
#define BCM2835_TS_TSENSSTAT_DATA_SHIFT		0
#define BCM2835_TS_TSENSSTAT_DATA_MASK		     \
	GENMASK(BCM2835_TS_TSENSSTAT_DATA_BITS +     \
		BCM2835_TS_TSENSSTAT_DATA_SHIFT - 1, \
		BCM2835_TS_TSENSSTAT_DATA_SHIFT)
#define BCM2835_TS_TSENSSTAT_VALID		BIT(10)
#define BCM2835_TS_TSENSSTAT_INTERRUPT		BIT(11)

struct bcm2835_thermal_info {
	int offset;
	int slope;
	int trip_temp;
};

struct bcm2835_thermal_data {
	const struct bcm2835_thermal_info *info;
	void __iomem *regs;
	struct clk *clk;
	struct dentry *debugfsdir;
};

static int bcm2835_thermal_adc2temp(
	const struct bcm2835_thermal_info *info, u32 adc)
{
	return info->offset + (adc * info->slope);
}

static int bcm2835_thermal_temp2adc(
	const struct bcm2835_thermal_info *info, int temp)
{
	temp -= info->offset;
	temp /= info->slope;

	if (temp < 0)
		temp = 0;
	if (temp >= BIT(BCM2835_TS_TSENSSTAT_DATA_BITS))
		temp = BIT(BCM2835_TS_TSENSSTAT_DATA_BITS) - 1;

	return temp;
}

static int bcm2835_thermal_get_trip_type(
	struct thermal_zone_device *tz, int trip,
	enum thermal_trip_type *type)
{
	*type = THERMAL_TRIP_CRITICAL;
	return 0;
}

static int bcm2835_thermal_get_trip_temp(
	struct thermal_zone_device *tz, int trip, int *temp)
{
	struct bcm2835_thermal_data *data = tz->devdata;
	u32 val = readl(data->regs + BCM2835_TS_TSENSCTL);

	/* get the THOLD bits */
	val &= BCM2835_TS_TSENSCTL_THOLD_MASK;
	val >>= BCM2835_TS_TSENSCTL_THOLD_SHIFT;

	/* if it is zero then use the info value */
	if (val)
		*temp = bcm2835_thermal_adc2temp(data->info, val);
	else
		*temp = data->info->trip_temp;

	return 0;
}

static int bcm2835_thermal_get_temp(struct thermal_zone_device *tz,
				    int *temp)
{
	struct bcm2835_thermal_data *data = tz->devdata;
	u32 val = readl(data->regs + BCM2835_TS_TSENSSTAT);

	if (!(val & BCM2835_TS_TSENSSTAT_VALID))
		return -EIO;

	val &= BCM2835_TS_TSENSSTAT_DATA_MASK;

	*temp = bcm2835_thermal_adc2temp(data->info, val);

	return 0;
}

static const struct debugfs_reg32 bcm2835_thermal_regs[] = {
	{
		.name = "ctl",
		.offset = 0
	},
	{
		.name = "stat",
		.offset = 4
	}
};

static void bcm2835_thermal_debugfs(struct platform_device *pdev)
{
	struct thermal_zone_device *tz = platform_get_drvdata(pdev);
	struct bcm2835_thermal_data *data = tz->devdata;
	struct debugfs_regset32 *regset;

	data->debugfsdir = debugfs_create_dir("bcm2835_thermal", NULL);
	if (!data->debugfsdir)
		return;

	regset = devm_kzalloc(&pdev->dev, sizeof(*regset), GFP_KERNEL);
	if (!regset)
		return;

	regset->regs = bcm2835_thermal_regs;
	regset->nregs = ARRAY_SIZE(bcm2835_thermal_regs);
	regset->base = data->regs;

	debugfs_create_regset32("regset", S_IRUGO,
				data->debugfsdir, regset);
}

static struct thermal_zone_device_ops bcm2835_thermal_ops  = {
	.get_temp = bcm2835_thermal_get_temp,
	.get_trip_temp = bcm2835_thermal_get_trip_temp,
	.get_trip_type = bcm2835_thermal_get_trip_type,
};

static const struct of_device_id bcm2835_thermal_of_match_table[];
static int bcm2835_thermal_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct thermal_zone_device *tz;
	struct bcm2835_thermal_data *data;
	struct resource *res;
	int err;
	u32 val;
	unsigned long rate;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	match = of_match_device(bcm2835_thermal_of_match_table,
				&pdev->dev);
	if (!match)
		return -EINVAL;
	data->info = match->data;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(data->regs)) {
		err = PTR_ERR(data->regs);
		dev_err(&pdev->dev, "Could not get registers: %d\n", err);
		return err;
	}

	data->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(data->clk)) {
		err = PTR_ERR(data->clk);
		if (err != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Could not get clk: %d\n", err);
		return err;
	}

	err = clk_prepare_enable(data->clk);
	if (err)
		return err;

	rate = clk_get_rate(data->clk);
	if ((rate < 1920000) || (rate > 5000000))
		dev_warn(&pdev->dev,
			 "Clock %pCn running at %pCr Hz is outside of the recommended range: 1.92 to 5MHz\n",
			 data->clk, data->clk);

	/*
	 * right now the FW does set up the HW-block, so we are not
	 * touching the configuration registers.
	 * But if the HW is not enabled, then set it up
	 * using "sane" values used by the firmware right now.
	 */
	val = readl(data->regs + BCM2835_TS_TSENSCTL);
	if (!(val & BCM2835_TS_TSENSCTL_RSTB)) {
		/* the basic required flags */
		val = (BCM2835_TS_TSENSCTL_CTRL_DEFAULT <<
		       BCM2835_TS_TSENSCTL_CTRL_SHIFT) |
		      BCM2835_TS_TSENSCTL_REGULEN;

		/*
		 * reset delay using the current firmware value of 14
		 * - units of time are unknown.
		 */
		val |= (14 << BCM2835_TS_TSENSCTL_RSTDELAY_SHIFT);

		/*  trip_adc value from info */
		val |= bcm2835_thermal_temp2adc(data->info,
						data->info->trip_temp) <<
			BCM2835_TS_TSENSCTL_THOLD_SHIFT;

		/* write the value back to the register as 2 steps */
		writel(val, data->regs + BCM2835_TS_TSENSCTL);
		val |= BCM2835_TS_TSENSCTL_RSTB;
		writel(val, data->regs + BCM2835_TS_TSENSCTL);
	}

	/* register thermal zone with 1 trip point an 1s polling */
	tz = thermal_zone_device_register("bcm2835_thermal",
					  1, 0, data,
					  &bcm2835_thermal_ops,
					  NULL,
					  0, 1000);
	if (IS_ERR(tz)) {
		clk_disable_unprepare(data->clk);
		err = PTR_ERR(tz);
		dev_err(&pdev->dev,
			"Failed to register the thermal device: %d\n",
			err);
		return err;
	}

	platform_set_drvdata(pdev, tz);

	bcm2835_thermal_debugfs(pdev);

	return 0;
}

static int bcm2835_thermal_remove(struct platform_device *pdev)
{
	struct thermal_zone_device *tz = platform_get_drvdata(pdev);
	struct bcm2835_thermal_data *data = tz->devdata;

	debugfs_remove_recursive(data->debugfsdir);
	thermal_zone_device_unregister(tz);
	clk_disable_unprepare(data->clk);

	return 0;
}

/*
 * Note: as per Raspberry Foundation FAQ
 * (https://www.raspberrypi.org/help/faqs/#performanceOperatingTemperature)
 * the recommended temperature range for the SOC -40C to +85C
 * so the trip limit is set to 80C.
 * this applies to all the BCM283X SOC
 */

static const struct of_device_id bcm2835_thermal_of_match_table[] = {
	{
		.compatible = "brcm,bcm2835-thermal",
		.data = &(struct bcm2835_thermal_info) {
			.offset = 407000,
			.slope = -538,
			.trip_temp = 80000
		}
	},
	{
		.compatible = "brcm,bcm2836-thermal",
		.data = &(struct bcm2835_thermal_info) {
			.offset = 407000,
			.slope = -538,
			.trip_temp = 80000
		}
	},
	{
		.compatible = "brcm,bcm2837-thermal",
		.data = &(struct bcm2835_thermal_info) {
			/* the bcm2837 needs adjustment of +5C */
			.offset = 407000 + 5000,
			.slope = -538,
			.trip_temp = 80000
		}
	},
	{},
};
MODULE_DEVICE_TABLE(of, bcm2835_thermal_of_match_table);

static struct platform_driver bcm2835_thermal_driver = {
	.probe = bcm2835_thermal_probe,
	.remove = bcm2835_thermal_remove,
	.driver = {
		.name = "bcm2835_thermal",
		.of_match_table = bcm2835_thermal_of_match_table,
	},
};
module_platform_driver(bcm2835_thermal_driver);

MODULE_AUTHOR("Martin Sperl");
MODULE_DESCRIPTION("Thermal driver for bcm2835 chip");
MODULE_LICENSE("GPL");
