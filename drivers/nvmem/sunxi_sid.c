// SPDX-License-Identifier: GPL-2.0+
/*
 * Allwinner sunXi SoCs Security ID support.
 *
 * Copyright (c) 2013 Oliver Schinagl <oliver@schinagl.nl>
 * Copyright (C) 2014 Maxime Ripard <maxime.ripard@free-electrons.com>
 */

#include <linux/device.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/nvmem-provider.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/random.h>
#include <linux/crc32.h>
#include <linux/ctype.h>
#include <linux/sys_soc.h>
#include <linux/unaligned.h>

/* Registers and special values for doing register-based SID readout on H3 */
#define SUN8I_SID_PRCTL		0x40
#define SUN8I_SID_RDKEY		0x60

#define SUN8I_SID_OFFSET_MASK	0x1FF
#define SUN8I_SID_OFFSET_SHIFT	16
#define SUN8I_SID_OP_LOCK	(0xAC << 8)
#define SUN8I_SID_READ		BIT(1)

struct sunxi_sid_cfg {
	u32	value_offset;
	u32	size;
	bool	need_register_readout;
	/* serial word 1 is the crc32 of SID words 1-3 (U-Boot, newer SoCs) */
	bool	serial_crc;
};

struct sunxi_sid {
	void __iomem		*base;
	u32			value_offset;
};

static int sunxi_sid_read(void *context, unsigned int offset,
			  void *val, size_t bytes)
{
	struct sunxi_sid *sid = context;
	u32 word;

	/* .stride = 4 so offset is guaranteed to be aligned */
	__ioread32_copy(val, sid->base + sid->value_offset + offset, bytes / 4);

	val += round_down(bytes, 4);
	offset += round_down(bytes, 4);
	bytes = bytes % 4;

	if (!bytes)
		return 0;

	/* Handle any trailing bytes */
	word = readl_relaxed(sid->base + sid->value_offset + offset);
	memcpy(val, &word, bytes);

	return 0;
}

static int sun8i_sid_register_readout(const struct sunxi_sid *sid,
				      const unsigned int offset,
				      u32 *out)
{
	u32 reg_val;
	int ret;

	/* Set word, lock access, and set read command */
	reg_val = (offset & SUN8I_SID_OFFSET_MASK)
		  << SUN8I_SID_OFFSET_SHIFT;
	reg_val |= SUN8I_SID_OP_LOCK | SUN8I_SID_READ;
	writel(reg_val, sid->base + SUN8I_SID_PRCTL);

	ret = readl_poll_timeout(sid->base + SUN8I_SID_PRCTL, reg_val,
				 !(reg_val & SUN8I_SID_READ), 100, 250000);
	if (ret)
		return ret;

	if (out)
		*out = readl(sid->base + SUN8I_SID_RDKEY);

	writel(0, sid->base + SUN8I_SID_PRCTL);

	return 0;
}

/*
 * On Allwinner H3, the value on the 0x200 offset of the SID controller seems
 * to be not reliable at all.
 * Read by the registers instead.
 */
static int sun8i_sid_read_by_reg(void *context, unsigned int offset,
				 void *val, size_t bytes)
{
	struct sunxi_sid *sid = context;
	u32 word;
	int ret;

	/* .stride = 4 so offset is guaranteed to be aligned */
	while (bytes >= 4) {
		ret = sun8i_sid_register_readout(sid, offset, val);
		if (ret)
			return ret;

		val += 4;
		offset += 4;
		bytes -= 4;
	}

	if (!bytes)
		return 0;

	/* Handle any trailing bytes */
	ret = sun8i_sid_register_readout(sid, offset, &word);
	if (ret)
		return ret;

	memcpy(val, &word, bytes);

	return 0;
}

static void sunxi_sid_socinfo_unregister(void *soc_dev)
{
	soc_device_unregister(soc_dev);
}

/*
 * soc0 from the SID: soc_id from the compatible ("allwinner,sun8i-h3-sid" ->
 * "H3"), serial_number the board serial# U-Boot derives from the same words
 * (board/sunxi/board.c get_unique_sid, setup_environment): "%08x%08x" of
 * word 0 and word 3, where on newer SoCs word 3 is first replaced by the
 * crc32 of words 1-3 and has bit 23 set if its low 24 bits are zero.
 */
static void sunxi_sid_add_socinfo(struct device *dev,
				  const struct sunxi_sid_cfg *cfg,
				  const u8 *raw)
{
	struct soc_device_attribute *attr;
	struct soc_device *soc_dev;
	const char *compat, *p, *end;
	u32 w0, w3;
	char *soc_id;
	int i;

	w0 = get_unaligned_le32(raw);
	if (!w0)
		return;
	w3 = cfg->serial_crc ? ~crc32_le(~0, raw + 4, 12) :
			       get_unaligned_le32(raw + 12);
	if (!(w3 & 0xffffff))
		w3 |= 0x800000;

	attr = devm_kzalloc(dev, sizeof(*attr), GFP_KERNEL);
	if (!attr)
		return;
	attr->family = "Allwinner";
	if (!of_property_read_string(dev->of_node, "compatible", &compat) &&
	    (p = strchr(compat, '-')) && (end = strstr(p, "-sid"))) {
		soc_id = devm_kasprintf(dev, GFP_KERNEL, "%.*s",
					(int)(end - p - 1), p + 1);
		for (i = 0; soc_id && soc_id[i]; i++)
			soc_id[i] = toupper(soc_id[i]);
		attr->soc_id = soc_id;
	}
	attr->serial_number = devm_kasprintf(dev, GFP_KERNEL, "%08x%08x", w0, w3);
	if (!attr->serial_number)
		return;

	soc_dev = soc_device_register(attr);
	if (IS_ERR(soc_dev)) {
		dev_warn(dev, "soc device: %pe\n", soc_dev);
		return;
	}
	devm_add_action_or_reset(dev, sunxi_sid_socinfo_unregister, soc_dev);
}

static int sunxi_sid_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nvmem_config *nvmem_cfg;
	struct nvmem_device *nvmem;
	struct sunxi_sid *sid;
	int size;
	char *randomness;
	const struct sunxi_sid_cfg *cfg;

	sid = devm_kzalloc(dev, sizeof(*sid), GFP_KERNEL);
	if (!sid)
		return -ENOMEM;

	cfg = of_device_get_match_data(dev);
	if (!cfg)
		return -EINVAL;
	sid->value_offset = cfg->value_offset;

	sid->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(sid->base))
		return PTR_ERR(sid->base);

	size = cfg->size;

	nvmem_cfg = devm_kzalloc(dev, sizeof(*nvmem_cfg), GFP_KERNEL);
	if (!nvmem_cfg)
		return -ENOMEM;

	nvmem_cfg->dev = dev;
	nvmem_cfg->name = "sunxi-sid";
	nvmem_cfg->type = NVMEM_TYPE_OTP;
	nvmem_cfg->add_legacy_fixed_of_cells = true;
	nvmem_cfg->read_only = true;
	/* the dump is the chip's unique SID: it identifies the board */
	nvmem_cfg->root_only = true;
	nvmem_cfg->size = cfg->size;
	nvmem_cfg->word_size = 1;
	nvmem_cfg->stride = 4;
	nvmem_cfg->priv = sid;
	if (cfg->need_register_readout)
		nvmem_cfg->reg_read = sun8i_sid_read_by_reg;
	else
		nvmem_cfg->reg_read = sunxi_sid_read;

	nvmem = devm_nvmem_register(dev, nvmem_cfg);
	if (IS_ERR(nvmem))
		return PTR_ERR(nvmem);

	randomness = kzalloc(size, GFP_KERNEL);
	if (!randomness)
		return -ENOMEM;

	nvmem_cfg->reg_read(sid, 0, randomness, size);
	add_device_randomness(randomness, size);
	sunxi_sid_add_socinfo(dev, cfg, randomness);
	kfree(randomness);

	platform_set_drvdata(pdev, nvmem);

	return 0;
}

static const struct sunxi_sid_cfg sun4i_a10_cfg = {
	.size = 0x10,
};

static const struct sunxi_sid_cfg sun7i_a20_cfg = {
	.size = 0x200,
};

static const struct sunxi_sid_cfg sun8i_h3_cfg = {
	.value_offset = 0x200,
	.size = 0x100,
	.need_register_readout = true,
	.serial_crc = true,
};

static const struct sunxi_sid_cfg sun50i_a64_cfg = {
	.value_offset = 0x200,
	.size = 0x100,
	.serial_crc = true,
};

static const struct sunxi_sid_cfg sun50i_h6_cfg = {
	.value_offset = 0x200,
	.size = 0x200,
	.serial_crc = true,
};

static const struct of_device_id sunxi_sid_of_match[] = {
	{ .compatible = "allwinner,sun4i-a10-sid", .data = &sun4i_a10_cfg },
	{ .compatible = "allwinner,sun7i-a20-sid", .data = &sun7i_a20_cfg },
	{ .compatible = "allwinner,sun8i-a83t-sid", .data = &sun50i_a64_cfg },
	{ .compatible = "allwinner,sun8i-h3-sid", .data = &sun8i_h3_cfg },
	{ .compatible = "allwinner,sun20i-d1-sid", .data = &sun50i_a64_cfg },
	{ .compatible = "allwinner,sun50i-a64-sid", .data = &sun50i_a64_cfg },
	{ .compatible = "allwinner,sun50i-h5-sid", .data = &sun50i_a64_cfg },
	{ .compatible = "allwinner,sun50i-h6-sid", .data = &sun50i_h6_cfg },
	{/* sentinel */},
};
MODULE_DEVICE_TABLE(of, sunxi_sid_of_match);

static struct platform_driver sunxi_sid_driver = {
	.probe = sunxi_sid_probe,
	.driver = {
		.name = "eeprom-sunxi-sid",
		.of_match_table = sunxi_sid_of_match,
	},
};
module_platform_driver(sunxi_sid_driver);

MODULE_AUTHOR("Oliver Schinagl <oliver@schinagl.nl>");
MODULE_DESCRIPTION("Allwinner sunxi security id driver");
MODULE_LICENSE("GPL");
