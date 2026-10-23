// SPDX-License-Identifier: GPL-2.0-only
// Copyright(c) 2017-2022 Intel Corporation

#include <linux/device.h>
#include <linux/module.h>
#include <linux/pci.h>

#include "gna_device.h"
#include "gna_hw.h"
#include "gna_pci.h"

static const struct gna_dev_info cnl_dev_info = {
	.hwid = GNA_DEV_HWID_CNL,
	GNA_GEN1_FEATURES
};

static const struct gna_dev_info glk_dev_info = {
	.hwid = GNA_DEV_HWID_GLK,
	GNA_GEN1_FEATURES
};

static const struct gna_dev_info ehl_dev_info = {
	.hwid = GNA_DEV_HWID_EHL,
	GNA_GEN1_FEATURES
};

static const struct gna_dev_info icl_dev_info = {
	.hwid = GNA_DEV_HWID_ICL,
	GNA_GEN1_FEATURES
};

static const struct gna_dev_info jsl_dev_info = {
	.hwid = GNA_DEV_HWID_JSL,
	GNA_GEN2_FEATURES
};

static const struct gna_dev_info tgl_dev_info = {
	.hwid = GNA_DEV_HWID_TGL,
	GNA_GEN2_FEATURES
};

static const struct gna_dev_info rkl_dev_info = {
	.hwid = GNA_DEV_HWID_RKL,
	GNA_GEN2_FEATURES
};

static const struct gna_dev_info adl_dev_info = {
	.hwid = GNA_DEV_HWID_ADL,
	GNA_GEN2_FEATURES
};

static const struct gna_dev_info rpl_dev_info = {
	.hwid = GNA_DEV_HWID_RPL,
	GNA_GEN2_FEATURES
};

static const struct gna_dev_info mtl_dev_info = {
	.hwid = GNA_DEV_HWID_MTL,
	GNA_GEN2_FEATURES
};

#define INTEL_GNA_DEVICE(hwid, info)				\
	{ PCI_VDEVICE(INTEL, hwid), (kernel_ulong_t)(info) }

static const struct pci_device_id gna_pci_ids[] = {
	INTEL_GNA_DEVICE(GNA_DEV_HWID_CNL, &cnl_dev_info),
	INTEL_GNA_DEVICE(GNA_DEV_HWID_EHL, &ehl_dev_info),
	INTEL_GNA_DEVICE(GNA_DEV_HWID_GLK, &glk_dev_info),
	INTEL_GNA_DEVICE(GNA_DEV_HWID_ICL, &icl_dev_info),
	INTEL_GNA_DEVICE(GNA_DEV_HWID_JSL, &jsl_dev_info),
	INTEL_GNA_DEVICE(GNA_DEV_HWID_TGL, &tgl_dev_info),
	INTEL_GNA_DEVICE(GNA_DEV_HWID_RKL, &rkl_dev_info),
	INTEL_GNA_DEVICE(GNA_DEV_HWID_ADL, &adl_dev_info),
	INTEL_GNA_DEVICE(GNA_DEV_HWID_RPL, &rpl_dev_info),
	INTEL_GNA_DEVICE(GNA_DEV_HWID_MTL, &mtl_dev_info),
	{ }
};

static void gna_irq_vectors_fini(void *data)
{
	struct pci_dev *pcidev = data;

	pci_free_irq_vectors(pcidev);
}

static int gna_irq_vectors_init(struct pci_dev *pcidev)
{
	int ret;

	ret = pci_alloc_irq_vectors(pcidev, 1, 1, PCI_IRQ_ALL_TYPES);
	if (ret < 0)
		return ret;

	ret = devm_add_action(&pcidev->dev, gna_irq_vectors_fini, pcidev);
	if (ret)
		gna_irq_vectors_fini(pcidev);

	return ret;
}

int gna_pci_probe(struct pci_dev *pcidev, const struct pci_device_id *pci_id)
{
	struct gna_dev_info *dev_info;
	void __iomem *iobase;
	int irq;
	int err;

	err = pcim_enable_device(pcidev);
	if (err)
		return err;

	err = pcim_iomap_regions(pcidev, BIT(0), pci_name(pcidev));
	if (err)
		return err;

	iobase = pcim_iomap_table(pcidev)[0];

	pci_set_master(pcidev);

	err = gna_irq_vectors_init(pcidev);
	if (err < 0)
		return err;

	irq = pci_irq_vector(pcidev, 0);
	if (irq < 0)
		return irq;

	dev_info = (struct gna_dev_info *)pci_id->driver_data;

	err = gna_probe(&pcidev->dev, dev_info, iobase, irq);
	if (err)
		return err;

	return 0;
}

static struct pci_driver gna_pci_driver = {
	.name = DRIVER_NAME,
	.id_table = gna_pci_ids,
	.probe = gna_pci_probe,
	.driver = {
		.pm = &gna_pm,
	},
};

module_pci_driver(gna_pci_driver);

MODULE_DEVICE_TABLE(pci, gna_pci_ids);
