// SPDX-License-Identifier: GPL-2.0-only
/*
 * Amlogic G12B DMC per-port DDR arbitration priority
 *
 * The G12B DDR memory controller (DMC, 0xff638000) arbitrates its requester
 * ports through one CHAN_CTRL register per port: DMC_AM0..AM7_CHAN_CTRL at
 * words 0x60 + 4n for the AM ports (16..23) and DMC_AXI0..AXI11_CHAN_CTRL at
 * words 0x80 + 4n for the AXI ports (0..11). Bits [18:16] of each are a 3-bit
 * priority weight, 0 (the lowest) being what BL2 leaves on every port.
 *
 * This driver applies the (port, priority) pairs a board lists in its
 * "amlogic,port-priority" property once at probe -- a read-modify-write of the
 * 3-bit field, read back, one dev_info line per port -- and re-applies them on
 * system resume. With the property absent it writes nothing.
 *
 * Port numbering is the vendor's ddr_port_desc_g12b[] table, which is also the
 * bit numbering of the DDR PMU's config1 axi-line filter
 * (drivers/perf/amlogic/meson_g12_ddr_pmu.c): 0 ARM-ADDR-E, 1 MALI, 3 HDCP,
 * 4 HEVC FRONT, 5 TEST, 6 USB3.0, 7 DEVICE, 8 HEVC BACK, 9 H265ENC, 10 NNA,
 * 11 GDC, 16/17/18 VPU0/1/2 READ, 19/20 VPU0/1 WRITE, 21 VDEC, 22 HCODEC,
 * 23 GE2D. Ports 2 (PCIE) and 12 (MIPI ISP) are refused: the vendor's
 * ddr_priority_g12b[] points them at words 0x84 and 0xae, which the A311D
 * register list names DMC_AXI1_CHAN_CTRL and nothing at all, so the two
 * sources disagree. Port 13 (ARM-ADDR-F) has no priority register. The
 * device-side ports (32 and up) have no CHAN_CTRL.
 *
 * The field's effect was measured on the NNA port of an AML-A311D-CC: the
 * NPU completes under a 4-writer CPU DDR hog at the A73's full 2208 MHz from
 * priority 2 and saturates by 4 (ai/vsi/vipnano-qi/audit/2026-09-11-n1522-*,
 * n1526-*, n1530-*).
 */

#include <linux/bitfield.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>

#define DMC_CHAN_CTRL_PRIO		GENMASK(18, 16)
#define DMC_PORT_PRIO_MAX		7

/* Offsets relative to DMC_AM0_CHAN_CTRL (word 0x60), the node's reg base. */
#define DMC_AM_CHAN_CTRL(n)		((n) * 0x10)
#define DMC_AXI_CHAN_CTRL(n)		(0x80 + (n) * 0x10)

struct meson_dmc_port_priority {
	void __iomem *base;
	u32 *pairs;		/* port, priority, port, priority, ... */
	unsigned int npairs;
};

static int meson_dmc_port_reg(u32 port)
{
	if (port == 2 || port == 12 || port == 13)
		return -EINVAL;
	if (port <= 11)
		return DMC_AXI_CHAN_CTRL(port);
	if (port >= 16 && port <= 23)
		return DMC_AM_CHAN_CTRL(port - 16);
	return -EINVAL;
}

static int meson_dmc_port_priority_apply(struct device *dev,
					 struct meson_dmc_port_priority *priv)
{
	unsigned int i;

	for (i = 0; i < priv->npairs; i++) {
		u32 port = priv->pairs[2 * i];
		u32 prio = priv->pairs[2 * i + 1];
		int reg = meson_dmc_port_reg(port);
		u32 old, val;

		old = readl(priv->base + reg);
		val = (old & ~DMC_CHAN_CTRL_PRIO) |
		      FIELD_PREP(DMC_CHAN_CTRL_PRIO, prio);
		writel(val, priv->base + reg);
		val = readl(priv->base + reg);
		if (FIELD_GET(DMC_CHAN_CTRL_PRIO, val) != prio) {
			dev_err(dev, "port %u priority %u refused: 0x%08x -> 0x%08x\n",
				port, prio, old, val);
			return -EIO;
		}
		dev_info(dev, "port %u priority %u (0x%08x -> 0x%08x)\n",
			 port, prio, old, val);
	}

	return 0;
}

static int meson_dmc_port_priority_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct meson_dmc_port_priority *priv;
	unsigned int i;
	int n, ret;

	n = of_property_count_u32_elems(dev->of_node, "amlogic,port-priority");
	if (n == -EINVAL || n == 0)
		return 0;	/* property absent: touch nothing */
	if (n < 0)
		return dev_err_probe(dev, n, "amlogic,port-priority unreadable\n");
	if (n & 1)
		return dev_err_probe(dev, -EINVAL,
				     "amlogic,port-priority needs <port prio> pairs\n");

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->pairs = devm_kcalloc(dev, n, sizeof(*priv->pairs), GFP_KERNEL);
	if (!priv->pairs)
		return -ENOMEM;
	priv->npairs = n / 2;

	ret = of_property_read_u32_array(dev->of_node, "amlogic,port-priority",
					 priv->pairs, n);
	if (ret)
		return dev_err_probe(dev, ret, "amlogic,port-priority unreadable\n");

	for (i = 0; i < priv->npairs; i++) {
		u32 port = priv->pairs[2 * i];
		u32 prio = priv->pairs[2 * i + 1];

		if (meson_dmc_port_reg(port) < 0)
			return dev_err_probe(dev, -EINVAL,
					     "port %u has no priority register\n",
					     port);
		if (prio > DMC_PORT_PRIO_MAX)
			return dev_err_probe(dev, -EINVAL,
					     "port %u priority %u exceeds %u\n",
					     port, prio, DMC_PORT_PRIO_MAX);
	}

	platform_set_drvdata(pdev, priv);

	return meson_dmc_port_priority_apply(dev, priv);
}

static int meson_dmc_port_priority_resume(struct device *dev)
{
	struct meson_dmc_port_priority *priv = dev_get_drvdata(dev);

	if (!priv)
		return 0;

	return meson_dmc_port_priority_apply(dev, priv);
}

static DEFINE_NOIRQ_DEV_PM_OPS(meson_dmc_port_priority_pm_ops, NULL,
			       meson_dmc_port_priority_resume);

static const struct of_device_id meson_dmc_port_priority_of_match[] = {
	{ .compatible = "amlogic,g12b-dmc-port-priority" },
	{ }
};
MODULE_DEVICE_TABLE(of, meson_dmc_port_priority_of_match);

static struct platform_driver meson_dmc_port_priority_driver = {
	.probe = meson_dmc_port_priority_probe,
	.driver = {
		.name = "meson-dmc-port-priority",
		.of_match_table = meson_dmc_port_priority_of_match,
		.pm = pm_sleep_ptr(&meson_dmc_port_priority_pm_ops),
	},
};
builtin_platform_driver(meson_dmc_port_priority_driver);
