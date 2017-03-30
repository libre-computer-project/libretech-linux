/*
 * Copyright (c) 2017 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 *
 * SPDX-License-Identifier: GPL-2.0+
 */

#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sys_soc.h>

#define SOCINFO_MAJOR_SHIFT	24
#define SOCINFO_MINOR_SHIFT	16
#define SOCINFO_PACK_SHIFT	8
#define SOCINFO_MISC_SHIFT	0
#define SOCINFO_MASK		0xff

static const struct meson_gx_soc_id {
	const char *name;
	unsigned int id;
} soc_ids[] = {
	{ "GXBB", 0x1f },
	{ "GXTVBB", 0x20 },
	{ "GXL", 0x21 },
	{ "GXM", 0x22 },
	{ "TXL", 0x23 },
};

static const struct meson_gx_package_id {
	const char *name;
	unsigned int major_id;
	unsigned int pack_id;
} soc_packages[] = {
	{ "S905", 0x1f, 0 },
	{ "S905M", 0x1f, 0x20 },
	{ "S905D", 0x21, 0 },
	{ "S905X", 0x21, 0x80 },
	{ "S905L", 0x21, 0xc0 },
	{ "S905M2", 0x21, 0xe0 },
	{ "S912", 0x22, 0 },
};

static inline unsigned int socinfo_to_major(u32 socinfo)
{
	return (socinfo >> SOCINFO_MAJOR_SHIFT) & SOCINFO_MASK;
}

static inline unsigned int socinfo_to_minor(u32 socinfo)
{
	return (socinfo >> SOCINFO_MINOR_SHIFT) & SOCINFO_MASK;
}

static inline unsigned int socinfo_to_pack(u32 socinfo)
{
	return (socinfo >> SOCINFO_PACK_SHIFT) & SOCINFO_MASK;
}

static inline unsigned int socinfo_to_misc(u32 socinfo)
{
	return (socinfo >> SOCINFO_MISC_SHIFT) & SOCINFO_MASK;
}

static const char *socinfo_to_package_id(u32 socinfo)
{
	unsigned int pack = socinfo_to_pack(socinfo) & 0xf0;
	unsigned int major = socinfo_to_major(socinfo);
	int i;

	for (i = 0 ; i < ARRAY_SIZE(soc_packages) ; ++i) {
		if (soc_packages[i].major_id == major &&
		    soc_packages[i].pack_id == pack)
			return soc_packages[i].name;
	}

	return "Unknown";
}

static const char *socinfo_to_soc_id(u32 socinfo)
{
	unsigned int id = socinfo_to_major(socinfo);
	int i;

	for (i = 0 ; i < ARRAY_SIZE(soc_ids) ; ++i) {
		if (soc_ids[i].id == id)
			return soc_ids[i].name;
	}

	return "Unknown";
}

int __init meson_gx_socinfo_init(void)
{
	struct soc_device_attribute *soc_dev_attr;
	void __iomem *meson_gx_socinfo_base;
	struct soc_device *soc_dev;
	struct device_node *root;
	struct device_node *np;
	struct device *dev;
	u32 socinfo;

	/* look up for socinfo node */
	np = of_find_compatible_node(NULL, NULL, "amlogic,meson-gx-socinfo");
	if (!np)
		return -ENODEV;

	meson_gx_socinfo_base = of_iomap(np, 0);
	of_node_put(np);

	if (!meson_gx_socinfo_base) {
		pr_err("%s: failed to map socinfo\n", np->name);
		return -ENOMEM;
	}

	socinfo = readl_relaxed(meson_gx_socinfo_base);
	iounmap(meson_gx_socinfo_base);

	soc_dev_attr = kzalloc(sizeof(*soc_dev_attr), GFP_KERNEL);
	if (!soc_dev_attr)
		return -ENODEV;

	soc_dev_attr->family = "Amlogic Meson";

	root = of_find_node_by_path("/");
	of_property_read_string(root, "model", &soc_dev_attr->machine);
	of_node_put(root);

	soc_dev_attr->revision = kasprintf(GFP_KERNEL, "%x:%x - %x:%x",
					   socinfo_to_major(socinfo),
					   socinfo_to_minor(socinfo),
					   socinfo_to_pack(socinfo),
					   socinfo_to_misc(socinfo));
	soc_dev_attr->soc_id = kasprintf(GFP_KERNEL, "%s (%s)",
					 socinfo_to_soc_id(socinfo),
					 socinfo_to_package_id(socinfo));

	soc_dev = soc_device_register(soc_dev_attr);
	if (IS_ERR(soc_dev)) {
		kfree(soc_dev_attr->revision);
		kfree_const(soc_dev_attr->soc_id);
		kfree(soc_dev_attr);
		return PTR_ERR(soc_dev);
	}
	dev = soc_device_to_device(soc_dev);

	dev_info(dev, "Amlogic Meson %s Revision %x:%x (%x:%x) Detected\n",
			soc_dev_attr->soc_id,
			socinfo_to_major(socinfo),
			socinfo_to_minor(socinfo),
			socinfo_to_pack(socinfo),
			socinfo_to_misc(socinfo));

	return 0;
}
core_initcall(meson_gx_socinfo_init);
