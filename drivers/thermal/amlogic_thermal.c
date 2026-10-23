// SPDX-License-Identifier: GPL-2.0+
/*
 * Amlogic Thermal Sensor Driver
 *
 * Copyright (C) 2017 Huan Biao <huan.biao@amlogic.com>
 * Copyright (C) 2019 Guillaume La Roque <glaroque@baylibre.com>
 *
 * Register value to celsius temperature formulas:
 *	Read_Val	    m * U
 * U = ---------, uptat = ---------
 *	2^16		  1 + n * U
 *
 * Temperature = A * ( uptat + u_efuse / 2^16 )- B
 *
 *  A B m n : calibration parameters
 *  u_efuse : fused calibration value, it's a signed 16 bits value
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/thermal.h>

#include "thermal_hwmon.h"

#define TSENSOR_CFG_REG1			0x4
	#define TSENSOR_CFG_REG1_RSET_VBG	BIT(12)
	#define TSENSOR_CFG_REG1_RSET_ADC	BIT(11)
	#define TSENSOR_CFG_REG1_VCM_EN		BIT(10)
	#define TSENSOR_CFG_REG1_VBG_EN		BIT(9)
	#define TSENSOR_CFG_REG1_OUT_CTL	BIT(6)
	#define TSENSOR_CFG_REG1_FILTER_EN	BIT(5)
	#define TSENSOR_CFG_REG1_DEM_EN		BIT(3)
	#define TSENSOR_CFG_REG1_CH_SEL		GENMASK(1, 0)
	#define TSENSOR_CFG_REG1_ENABLE		\
		(TSENSOR_CFG_REG1_FILTER_EN |	\
		 TSENSOR_CFG_REG1_VCM_EN |	\
		 TSENSOR_CFG_REG1_VBG_EN |	\
		 TSENSOR_CFG_REG1_DEM_EN |	\
		 TSENSOR_CFG_REG1_CH_SEL)
	#define TSENSOR_CFG_REG1_IRQ_RISE0_EN	BIT(24)
	#define TSENSOR_CFG_REG1_IRQ_FALL0_EN	BIT(28)
	#define TSENSOR_CFG_REG1_IRQ_EN_MASK	GENMASK(31, 24)
	#define TSENSOR_CFG_REG1_IRQ_CLR_MASK	GENMASK(23, 16)

#define TSENSOR_CFG_REG2			0x8
	#define TSENSOR_CFG_REG2_REBOOT_CODE	GENMASK(15, 4)
	#define TSENSOR_CFG_REG2_REBOOT_TIME	GENMASK(23, 16)
	#define TSENSOR_CFG_REG2_REBOOT_ALL_EN	BIT(30)
	#define TSENSOR_CFG_REG2_HITEMP_EN	BIT(31)

#define TSENSOR_CFG_REG4			0x10
#define TSENSOR_CFG_REG5			0x14
#define TSENSOR_CFG_REG6			0x18
#define TSENSOR_CFG_REG7			0x1c

#define TSENSOR_STAT0			0x40
#define TSENSOR_STAT1			0x44

#define TSENSOR_STAT9			0x64

#define TSENSOR_READ_TEMP_MASK		GENMASK(15, 0)
#define TSENSOR_TEMP_MASK		GENMASK(11, 0)

#define TSENSOR_TRIM_SIGN_MASK		BIT(15)
#define TSENSOR_TRIM_TEMP_MASK		GENMASK(14, 0)
#define TSENSOR_TRIM_VERSION_MASK	GENMASK(31, 24)

#define TSENSOR_TRIM_VERSION(_version)	\
	FIELD_GET(TSENSOR_TRIM_VERSION_MASK, _version)

#define TSENSOR_TRIM_CALIB_VALID_MASK	(GENMASK(3, 2) | BIT(7))

#define TSENSOR_CALIB_OFFSET	1
#define TSENSOR_CALIB_SHIFT	4

#define TSENSOR_REBOOT_TEMP		115000	/* millicelsius */

/**
 * struct amlogic_thermal_soc_calib_data
 * @A: calibration parameters
 * @B: calibration parameters
 * @m: calibration parameters
 * @n: calibration parameters
 *
 * This structure is required for configuration of amlogic thermal driver.
 */
struct amlogic_thermal_soc_calib_data {
	int A;
	int B;
	int m;
	int n;
};

/**
 * struct amlogic_thermal_data
 * @u_efuse_off: register offset to read fused calibration value
 * @calibration_parameters: calibration parameters structure pointer
 * @regmap_config: regmap config for the device
 * This structure is required for configuration of amlogic thermal driver.
 */
struct amlogic_thermal_data {
	int u_efuse_off;
	const struct amlogic_thermal_soc_calib_data *calibration_parameters;
	const struct regmap_config *regmap_config;
};

struct amlogic_thermal {
	struct platform_device *pdev;
	const struct amlogic_thermal_data *data;
	struct regmap *regmap;
	struct regmap *sec_ao_map;
	struct clk *clk;
	struct thermal_zone_device *tzd;
	u32 trim_info;
	int irq;
};

/*
 * Calculate a temperature value from a temperature code.
 * The unit of the temperature is degree milliCelsius.
 */
static int amlogic_thermal_code_to_millicelsius(struct amlogic_thermal *pdata,
						int temp_code)
{
	const struct amlogic_thermal_soc_calib_data *param =
					pdata->data->calibration_parameters;
	int temp;
	s64 factor, uptat, uefuse;

	uefuse = pdata->trim_info & TSENSOR_TRIM_SIGN_MASK ?
			     ~(pdata->trim_info & TSENSOR_TRIM_TEMP_MASK) + 1 :
			     (pdata->trim_info & TSENSOR_TRIM_TEMP_MASK);

	factor = param->n * temp_code;
	factor = div_s64(factor, 100);

	uptat = temp_code * param->m;
	uptat = div_s64(uptat, 100);
	uptat = uptat * BIT(16);
	uptat = div_s64(uptat, BIT(16) + factor);

	temp = (uptat + uefuse) * param->A;
	temp = div_s64(temp, BIT(16));
	temp = (temp - param->B) * 100;

	return temp;
}

/*
 * Inverse of code_to_millicelsius: convert millicelsius to sensor code.
 *
 * From the forward formula:
 *   U = code / 2^16
 *   uptat = (m * U) / (1 + n * U)   (with m,n scaled by /100)
 *   T = A * (uptat + uefuse/2^16) - B   (result in centi-celsius, *100)
 *
 * Inverting:
 *   uptat = (T/100 + B) * 2^16 / A - uefuse
 *   U = uptat / (m - n * uptat)  (scaled)
 *   code = U (the 16-bit sensor reading)
 *
 * We return a 12-bit code (>> 4) suitable for threshold registers.
 */
static int amlogic_thermal_millicelsius_to_code(struct amlogic_thermal *pdata,
						int temp_mc)
{
	const struct amlogic_thermal_soc_calib_data *param =
					pdata->data->calibration_parameters;
	s64 uefuse, uptat, code;

	uefuse = pdata->trim_info & TSENSOR_TRIM_SIGN_MASK ?
			     ~(pdata->trim_info & TSENSOR_TRIM_TEMP_MASK) + 1 :
			     (pdata->trim_info & TSENSOR_TRIM_TEMP_MASK);

	/* T (millicelsius) = (A * (uptat + uefuse) / 2^16 - B) * 100 */
	/* So: uptat = ((T/100 + B) * 2^16 / A) - uefuse */
	uptat = (s64)(temp_mc / 100 + param->B) * BIT(16);
	uptat = div_s64(uptat, param->A);
	uptat = uptat - uefuse;

	/* Forward: uptat = m*code*2^16 / (100*2^16 + n*code) */
	/* Inverse: code = uptat * 100 * 2^16 / (m * 2^16 - n * uptat) */
	code = uptat * 100 * BIT(16);
	code = div_s64(code, (s64)param->m * BIT(16) - (s64)param->n * uptat);

	/* Return 12-bit threshold code */
	return clamp_val((int)(code >> 4) & TSENSOR_TEMP_MASK, 0,
			 TSENSOR_TEMP_MASK);
}

static irqreturn_t amlogic_thermal_irq(int irq, void *data)
{
	struct amlogic_thermal *pdata = data;
	unsigned int stat, reg1;

	regmap_read(pdata->regmap, TSENSOR_STAT1, &stat);
	if (!(stat & 0xff))
		return IRQ_NONE;

	/* Clear pending IRQs: write-1-to-clear in CFG_REG1[23:16] */
	regmap_read(pdata->regmap, TSENSOR_CFG_REG1, &reg1);
	reg1 |= (stat & 0xff) << 16;
	regmap_write(pdata->regmap, TSENSOR_CFG_REG1, reg1);
	reg1 &= ~TSENSOR_CFG_REG1_IRQ_CLR_MASK;
	regmap_write(pdata->regmap, TSENSOR_CFG_REG1, reg1);

	thermal_zone_device_update(pdata->tzd, THERMAL_TRIP_VIOLATED);

	return IRQ_HANDLED;
}

static int amlogic_thermal_set_trips(struct thermal_zone_device *tz,
				     int low, int high)
{
	struct amlogic_thermal *pdata = thermal_zone_device_priv(tz);
	unsigned int reg1;
	u32 irq_en = 0;
	int high_code, low_code;

	/* Threshold 0 is in bits [23:12] of CFG_REG4/CFG_REG6 */

	/* Program rising threshold 0 (high) into CFG_REG4[23:12] */
	if (high != INT_MAX) {
		high_code = amlogic_thermal_millicelsius_to_code(pdata, high);
		regmap_update_bits(pdata->regmap, TSENSOR_CFG_REG4,
				   TSENSOR_TEMP_MASK << 12, high_code << 12);
		irq_en |= TSENSOR_CFG_REG1_IRQ_RISE0_EN;
	}

	/* Program falling threshold 0 (low) into CFG_REG6[23:12] */
	if (low != -INT_MAX) {
		low_code = amlogic_thermal_millicelsius_to_code(pdata, low);
		regmap_update_bits(pdata->regmap, TSENSOR_CFG_REG6,
				   TSENSOR_TEMP_MASK << 12, low_code << 12);
		irq_en |= TSENSOR_CFG_REG1_IRQ_FALL0_EN;
	}

	/* Enable only the relevant IRQs, clear any pending */
	regmap_read(pdata->regmap, TSENSOR_CFG_REG1, &reg1);
	reg1 &= ~(TSENSOR_CFG_REG1_IRQ_EN_MASK | TSENSOR_CFG_REG1_IRQ_CLR_MASK);
	reg1 |= irq_en;
	regmap_write(pdata->regmap, TSENSOR_CFG_REG1, reg1);

	return 0;
}

static int amlogic_thermal_initialize(struct amlogic_thermal *pdata)
{
	int ret = 0;
	int ver;

	regmap_read(pdata->sec_ao_map, pdata->data->u_efuse_off,
		    &pdata->trim_info);

	ver = TSENSOR_TRIM_VERSION(pdata->trim_info);

	if ((ver & TSENSOR_TRIM_CALIB_VALID_MASK) == 0) {
		ret = -EINVAL;
		dev_err(&pdata->pdev->dev,
			"tsensor thermal calibration not supported: 0x%x!\n",
			ver);
	}

	return ret;
}

static void amlogic_thermal_enable_reboot(struct amlogic_thermal *data)
{
	int code;
	u32 val;

	code = amlogic_thermal_millicelsius_to_code(data, TSENSOR_REBOOT_TEMP);
	val = FIELD_PREP(TSENSOR_CFG_REG2_REBOOT_CODE, code) |
	      FIELD_PREP(TSENSOR_CFG_REG2_REBOOT_TIME, 0xff) |
	      TSENSOR_CFG_REG2_HITEMP_EN |
	      TSENSOR_CFG_REG2_REBOOT_ALL_EN;

	regmap_write(data->regmap, TSENSOR_CFG_REG2, val);

	dev_info(&data->pdev->dev,
		 "hardware reboot enabled at %d mC (code %d)\n",
		 TSENSOR_REBOOT_TEMP, code);
}

static int amlogic_thermal_enable(struct amlogic_thermal *data)
{
	int ret;

	ret = clk_prepare_enable(data->clk);
	if (ret)
		return ret;

	regmap_update_bits(data->regmap, TSENSOR_CFG_REG1,
			   TSENSOR_CFG_REG1_ENABLE, TSENSOR_CFG_REG1_ENABLE);

	amlogic_thermal_enable_reboot(data);

	return 0;
}

static void amlogic_thermal_disable(struct amlogic_thermal *data)
{
	/* Disable IRQs and sensor */
	regmap_update_bits(data->regmap, TSENSOR_CFG_REG1,
			   TSENSOR_CFG_REG1_IRQ_EN_MASK |
			   TSENSOR_CFG_REG1_ENABLE, 0);
	/* Disable hardware reboot */
	regmap_write(data->regmap, TSENSOR_CFG_REG2, 0);
	clk_disable_unprepare(data->clk);
}

static int amlogic_thermal_get_temp(struct thermal_zone_device *tz, int *temp)
{
	unsigned int tval;
	struct amlogic_thermal *pdata = thermal_zone_device_priv(tz);

	if (!pdata)
		return -EINVAL;

	regmap_read(pdata->regmap, TSENSOR_STAT0, &tval);
	*temp =
	   amlogic_thermal_code_to_millicelsius(pdata,
						tval & TSENSOR_READ_TEMP_MASK);

	return 0;
}

static const struct thermal_zone_device_ops amlogic_thermal_ops = {
	.get_temp	= amlogic_thermal_get_temp,
	.set_trips	= amlogic_thermal_set_trips,
};

static const struct regmap_config amlogic_thermal_regmap_config_g12a = {
	.reg_bits = 8,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = TSENSOR_STAT9,
};

static const struct amlogic_thermal_soc_calib_data amlogic_thermal_g12a = {
	.A = 9411,
	.B = 3159,
	.m = 424,
	.n = 324,
};

static const struct amlogic_thermal_data amlogic_thermal_g12a_cpu_param = {
	.u_efuse_off = 0x128,
	.calibration_parameters = &amlogic_thermal_g12a,
	.regmap_config = &amlogic_thermal_regmap_config_g12a,
};

static const struct amlogic_thermal_data amlogic_thermal_g12a_ddr_param = {
	.u_efuse_off = 0xf0,
	.calibration_parameters = &amlogic_thermal_g12a,
	.regmap_config = &amlogic_thermal_regmap_config_g12a,
};

static const struct amlogic_thermal_data amlogic_thermal_a1_cpu_param = {
	.u_efuse_off = 0x114,
	.calibration_parameters = &amlogic_thermal_g12a,
	.regmap_config = &amlogic_thermal_regmap_config_g12a,
};

static const struct of_device_id of_amlogic_thermal_match[] = {
	{
		.compatible = "amlogic,g12a-ddr-thermal",
		.data = &amlogic_thermal_g12a_ddr_param,
	},
	{
		.compatible = "amlogic,g12a-cpu-thermal",
		.data = &amlogic_thermal_g12a_cpu_param,
	},
	{
		.compatible = "amlogic,a1-cpu-thermal",
		.data = &amlogic_thermal_a1_cpu_param,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_amlogic_thermal_match);

static int amlogic_thermal_probe(struct platform_device *pdev)
{
	struct amlogic_thermal *pdata;
	struct device *dev = &pdev->dev;
	void __iomem *base;
	int ret;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdata->data = of_device_get_match_data(dev);
	pdata->pdev = pdev;
	platform_set_drvdata(pdev, pdata);

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	pdata->regmap = devm_regmap_init_mmio(dev, base,
					      pdata->data->regmap_config);
	if (IS_ERR(pdata->regmap))
		return PTR_ERR(pdata->regmap);

	pdata->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(pdata->clk))
		return dev_err_probe(dev, PTR_ERR(pdata->clk), "failed to get clock\n");

	pdata->sec_ao_map = syscon_regmap_lookup_by_phandle
		(pdev->dev.of_node, "amlogic,ao-secure");
	if (IS_ERR(pdata->sec_ao_map)) {
		dev_err(dev, "syscon regmap lookup failed.\n");
		return PTR_ERR(pdata->sec_ao_map);
	}

	pdata->tzd = devm_thermal_of_zone_register(&pdev->dev,
						   0,
						   pdata,
						   &amlogic_thermal_ops);
	if (IS_ERR(pdata->tzd)) {
		ret = PTR_ERR(pdata->tzd);
		dev_err(dev, "Failed to register tsensor: %d\n", ret);
		return ret;
	}

	devm_thermal_add_hwmon_sysfs(&pdev->dev, pdata->tzd);

	ret = amlogic_thermal_initialize(pdata);
	if (ret)
		return ret;

	ret = amlogic_thermal_enable(pdata);
	if (ret)
		return ret;

	pdata->irq = platform_get_irq_optional(pdev, 0);
	if (pdata->irq > 0) {
		ret = devm_request_threaded_irq(dev, pdata->irq, NULL,
						amlogic_thermal_irq,
						IRQF_ONESHOT,
						dev_name(dev), pdata);
		if (ret) {
			dev_warn(dev, "failed to request IRQ %d, using polling\n",
				 pdata->irq);
			pdata->irq = 0;
		}
	}

	return 0;
}

static void amlogic_thermal_remove(struct platform_device *pdev)
{
	struct amlogic_thermal *data = platform_get_drvdata(pdev);

	amlogic_thermal_disable(data);
}

static int amlogic_thermal_suspend(struct device *dev)
{
	struct amlogic_thermal *data = dev_get_drvdata(dev);

	amlogic_thermal_disable(data);

	return 0;
}

static int amlogic_thermal_resume(struct device *dev)
{
	struct amlogic_thermal *data = dev_get_drvdata(dev);

	return amlogic_thermal_enable(data);
}

static DEFINE_SIMPLE_DEV_PM_OPS(amlogic_thermal_pm_ops,
				amlogic_thermal_suspend,
				amlogic_thermal_resume);

static struct platform_driver amlogic_thermal_driver = {
	.driver = {
		.name		= "amlogic_thermal",
		.pm		= pm_ptr(&amlogic_thermal_pm_ops),
		.of_match_table = of_amlogic_thermal_match,
	},
	.probe = amlogic_thermal_probe,
	.remove = amlogic_thermal_remove,
};

module_platform_driver(amlogic_thermal_driver);

MODULE_AUTHOR("Guillaume La Roque <glaroque@baylibre.com>");
MODULE_DESCRIPTION("Amlogic thermal driver");
MODULE_LICENSE("GPL v2");
