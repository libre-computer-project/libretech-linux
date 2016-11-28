/*
 * CPU frequency scaling for Broadcom set top box SoCs
 *
 * Copyright (c) 2016 Broadcom
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/cpufreq.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

#define BRCMSTB_CPUFREQ_PREFIX	"brcmstb"
#define BRCMSTB_CPUFREQ_NAME	BRCMSTB_CPUFREQ_PREFIX "-cpufreq"

/* We search for these compatible strings. */
#define BRCMSTB_DT_CPU_CLK_CTRL	"brcm,brcmstb-cpu-clk-div"
#define BRCMSTB_DT_MEMC_DDR	"brcm,brcmstb-memc-ddr"
#define BRCM_AVS_CPU_DATA	"brcm,avs-cpu-data-mem"

/* We also need a few clocks in device tree. These are node names. */
#define BRCMSTB_CLK_MDIV_CH0	"cpu_mdiv_ch0"
#define BRCMSTB_CLK_NDIV_INT	"cpu_ndiv_int"
#define BRCMSTB_CLK_SW_SCB	"sw_scb"

#define BRCMSTB_TBL_SAFE_MODE	BIT(0)
#define BRCMSTB_REG_SAFE_MODE	BIT(4)

#define TRANSITION_LATENCY	(25 * 1000)	/* 25 us */

/* This is as low as we'll go in the frequency table. */
#define MIN_CPU_FREQ		(100 * 1000)	/* in kHz */

struct private_data {
	void __iomem *cpu_clk_ctrl_reg;
	struct device *dev;
};

/* Count the active memory controllers in the system. */
static int count_memory_controllers(void)
{
	struct device_node *np = NULL;
	int i = 0;

	do {
		np = of_find_compatible_node(np, NULL, BRCMSTB_DT_MEMC_DDR);
		if (of_device_is_available(np))
			i++;
		of_node_put(np);
	} while (np);

	return i;
}

static int get_frequencies(const struct cpufreq_policy *policy,
			   unsigned int *vco_freq, unsigned int *cpu_freq,
			   unsigned int *scb_freq)
{
	struct clk *cpu_ndiv_int, *sw_scb;

	cpu_ndiv_int = __clk_lookup(BRCMSTB_CLK_NDIV_INT);
	if (!cpu_ndiv_int)
		return -ENODEV;

	sw_scb = __clk_lookup(BRCMSTB_CLK_SW_SCB);
	if (!sw_scb)
		return -ENODEV;

	/* return frequencies in kHz */
	*vco_freq = clk_get_rate(cpu_ndiv_int) / 1000;
	*cpu_freq = clk_get_rate(policy->clk) / 1000;
	*scb_freq = clk_get_rate(sw_scb) / 1000;

	return 0;
}

/*
 * Safe mode: When set, the CPU's bus unit is being throttled. This is done to
 * avoid buffer overflows when the CPU-to-bus-clock ratio is low.
 *
 * The formula as to what constitutes a low CPU-to-bus-clock ratio takes into
 * account the number of memory controllers active in the system and the SCB
 * frequency. More memory controllers means safe mode is required starting at
 * higher frequencies.
 *
 * For 1 memory controller, cpu_freq/scb_freq must be greater than or equal to
 * 2 to not require safe mode.
 *
 * For 2 or 3 memory controllers, cpu_freq/scb_freq must be greater than or
 * equal 3 to not require safe mode.
 */

static int freq_requires_safe_mode(unsigned int cpu_freq, unsigned int scb_freq,
				   int num_memc)
{
	unsigned int safe_ratio;

	switch (num_memc) {
	case 1:
		safe_ratio = 2;
		break;
	case 2:
	case 3:
		safe_ratio = 3;
		break;
	default:
		return -EINVAL;
	}

	return ((cpu_freq / scb_freq) < safe_ratio);
}

static struct cpufreq_frequency_table *
brcmstb_get_freq_table(const struct cpufreq_policy *policy)
{
	unsigned int cpu_freq, vco_freq, scb_freq, mdiv, init_mdiv, f;
	struct cpufreq_frequency_table *table;
	struct private_data *priv;
	int num_memc, ret;
	unsigned int i = 0;

	ret = get_frequencies(policy, &vco_freq, &cpu_freq, &scb_freq);
	if (ret)
		return ERR_PTR(ret);

	priv = policy->driver_data;
	num_memc = count_memory_controllers();

	/* Calculate the initial mdiv value. We'll increment mdiv from here. */
	init_mdiv = vco_freq / cpu_freq;

	/* Count how many frequencies we'll offer. */
	f = cpu_freq;
	for (mdiv = init_mdiv; f >= MIN_CPU_FREQ; mdiv++, f = vco_freq / mdiv) {
		/* We only want to use "whole" MHz. */
		if ((f % 1000) == 0)
			i++;
	}

	table = devm_kzalloc(priv->dev, (i + 1) * sizeof(*table), GFP_KERNEL);
	if (!table)
		return ERR_PTR(-ENOMEM);

	/* Now, fill the table. */
	f = cpu_freq;
	i = 0;
	for (mdiv = init_mdiv; f >= MIN_CPU_FREQ; mdiv++, f = vco_freq / mdiv) {
		if ((f % 1000) == 0) {
			table[i].frequency = f;
			ret = freq_requires_safe_mode(f, scb_freq, num_memc);
			if (ret < 0)
				return ERR_PTR(ret);
			if (ret > 0)
				table[i].driver_data |= BRCMSTB_TBL_SAFE_MODE;
			i++;
		}
	}
	table[i].frequency = CPUFREQ_TABLE_END;

	return table;
}

static int brcmstb_target_index(struct cpufreq_policy *policy,
				unsigned int index)
{
	struct cpufreq_frequency_table *entry;
	struct private_data *priv;
	int ret, safe_mode_needed;
	u32 reg;

	priv = policy->driver_data;
	entry = &policy->freq_table[index];
	safe_mode_needed = entry->driver_data & BRCMSTB_TBL_SAFE_MODE;

	reg = readl(priv->cpu_clk_ctrl_reg);
	if (safe_mode_needed && !(reg & BRCMSTB_REG_SAFE_MODE)) {
		reg |= BRCMSTB_REG_SAFE_MODE;
		writel(reg, priv->cpu_clk_ctrl_reg);
	}
	ret = clk_set_rate(policy->clk, entry->frequency * 1000);
	if (!ret && !safe_mode_needed && (reg & BRCMSTB_REG_SAFE_MODE)) {
		reg &= ~BRCMSTB_REG_SAFE_MODE;
		writel(reg, priv->cpu_clk_ctrl_reg);
	}

	return ret;
}

/*
 * All initialization code that we only want to execute once goes here. Setup
 * code that can be re-tried on every core (if it failed before) can go into
 * brcmstb_cpufreq_init().
 */
static int brcmstb_prepare_init(struct platform_device *pdev)
{
	struct private_data *priv;
	struct resource *res;
	struct device *dev;

	/*
	 * If the BRCM STB AVS CPUfreq driver is supported, we bail, so that
	 * the more modern approach implementing DVFS in firmware can be used.
	 */
	if (IS_ENABLED(CONFIG_ARM_BRCM_AVS_CPUFREQ)) {
		struct device_node *np;

		np = of_find_compatible_node(NULL, NULL, BRCM_AVS_CPU_DATA);
		if (np) {
			of_node_put(np);
			return -ENXIO;
		}
	}

	dev = &pdev->dev;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->cpu_clk_ctrl_reg = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->cpu_clk_ctrl_reg)) {
		dev_err(dev, "couldn't map DT entry %s\n",
			BRCMSTB_DT_CPU_CLK_CTRL);
		return -ENODEV;
	}

	priv->dev = dev;
	platform_set_drvdata(pdev, priv);

	return 0;
}

static int brcmstb_cpufreq_init(struct cpufreq_policy *policy)
{
	struct cpufreq_frequency_table *freq_table;
	struct platform_device *pdev;
	struct private_data *priv;
	struct clk *cpu_mdiv_ch0;
	struct device *dev;
	int ret;

	cpu_mdiv_ch0 = __clk_lookup(BRCMSTB_CLK_MDIV_CH0);
	if (!cpu_mdiv_ch0)
		return -ENODEV;

	pdev = cpufreq_get_driver_data();
	priv = platform_get_drvdata(pdev);
	dev = &pdev->dev;

	policy->clk = cpu_mdiv_ch0;
	policy->driver_data = priv;

	freq_table = brcmstb_get_freq_table(policy);
	if (IS_ERR(freq_table)) {
		ret = PTR_ERR(freq_table);
		dev_err(dev, "Couldn't determine frequency table (%d).\n", ret);
		if (ret == -EINVAL)
			dev_emerg(dev,
				"Invalid number of memory controllers -- %d!\n",
				count_memory_controllers());
		return ret;
	}

	ret = cpufreq_generic_init(policy, freq_table, TRANSITION_LATENCY);
	if (!ret)
		dev_info(dev, "registered\n");

	return ret;
}

/* Shows the number of memory controllers. */
static ssize_t show_brcmstb_num_memc(struct cpufreq_policy *policy, char *buf)
{
	return sprintf(buf, "%u\n", count_memory_controllers());
}

/* Shows vco_freq, cpu_freq, and scb_freq in kHz. */
static ssize_t show_brcmstb_freqs(struct cpufreq_policy *policy, char *buf)
{
	unsigned int vco_freq, cpu_freq, scb_freq;
	int ret;

	ret = get_frequencies(policy, &vco_freq, &cpu_freq, &scb_freq);
	if (ret)
		return sprintf(buf, "<unknown>\n");

	return sprintf(buf, "%u %u %u\n", vco_freq, cpu_freq, scb_freq);
}

/* Shows the lowest frequency (in kHz) that can be used without "safe mode". */
static ssize_t show_brcmstb_safe_freq(struct cpufreq_policy *policy, char *buf)
{
	struct cpufreq_frequency_table *entry;
	unsigned int safe_freq = 0;

	cpufreq_for_each_valid_entry(entry, policy->freq_table) {
		if (!(entry->driver_data & BRCMSTB_TBL_SAFE_MODE))
			safe_freq = entry->frequency;
	}

	return sprintf(buf, "%u\n", safe_freq);
}

cpufreq_freq_attr_ro(brcmstb_num_memc);
cpufreq_freq_attr_ro(brcmstb_freqs);
cpufreq_freq_attr_ro(brcmstb_safe_freq);

static struct freq_attr *brcmstb_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	&brcmstb_num_memc,
	&brcmstb_freqs,
	&brcmstb_safe_freq,
	NULL
};

static struct cpufreq_driver brcmstb_driver = {
	.flags		= CPUFREQ_NEED_INITIAL_FREQ_CHECK,
	.verify		= cpufreq_generic_frequency_table_verify,
	.target_index	= brcmstb_target_index,
	.get		= cpufreq_generic_get,
	.init		= brcmstb_cpufreq_init,
	.attr		= brcmstb_cpufreq_attr,
	.name		= BRCMSTB_CPUFREQ_PREFIX,
};

static int brcmstb_cpufreq_probe(struct platform_device *pdev)
{
	int ret;

	ret = brcmstb_prepare_init(pdev);
	if (ret)
		return ret;

	brcmstb_driver.driver_data = pdev;

	return cpufreq_register_driver(&brcmstb_driver);
}

static int brcmstb_cpufreq_remove(struct platform_device *pdev)
{
	int ret;

	ret = cpufreq_unregister_driver(&brcmstb_driver);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct of_device_id brcmstb_cpufreq_match[] = {
	{ .compatible = BRCMSTB_DT_CPU_CLK_CTRL },
	{ }
};
MODULE_DEVICE_TABLE(platform, brcmstb_cpufreq_match);

static struct platform_driver brcmstb_cpufreq_platdrv = {
	.driver = {
		.name	= BRCMSTB_CPUFREQ_NAME,
		.of_match_table = brcmstb_cpufreq_match,
	},
	.probe		= brcmstb_cpufreq_probe,
	.remove		= brcmstb_cpufreq_remove,
};
module_platform_driver(brcmstb_cpufreq_platdrv);

MODULE_AUTHOR("Markus Mayer <mmayer@broadcom.com>");
MODULE_DESCRIPTION("CPUfreq driver for Broadcom STB SoCs");
MODULE_LICENSE("GPL");
