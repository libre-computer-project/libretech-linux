// SPDX-License-Identifier: GPL-2.0-only
// Copyright(c) 2017-2022 Intel Corporation

#include <linux/bitfield.h>
#include <linux/iopoll.h>

#include <uapi/drm/gna_drm.h>

#include "gna_device.h"
#include "gna_hw.h"

int gna_parse_hw_status(struct gna_device *gna_priv, u32 hw_status)
{
	if (hw_status & GNA_ERROR) {
		dev_dbg(gna_dev(gna_priv), "GNA completed with errors: %#x\n", hw_status);
		return -EIO;
	}

	if (hw_status & GNA_STS_SCORE_COMPLETED) {
		dev_dbg(gna_dev(gna_priv), "GNA completed successfully: %#x\n", hw_status);
		return 0;
	}

	dev_dbg(gna_dev(gna_priv), "GNA not completed, status: %#x\n", hw_status);
	return -ENODATA;
}

void gna_print_error_status(struct gna_device *gna_priv, u32 hw_status)
{
	if (hw_status & GNA_STS_PARAM_OOR)
		dev_dbg(gna_dev(gna_priv), "GNA error: Param Out Range Error\n");

	if (hw_status & GNA_STS_VA_OOR)
		dev_dbg(gna_dev(gna_priv), "GNA error: VA Out of Range Error\n");

	if (hw_status & GNA_STS_PCI_MMU_ERR)
		dev_dbg(gna_dev(gna_priv), "GNA error: PCI MMU Error\n");

	if (hw_status & GNA_STS_PCI_DMA_ERR)
		dev_dbg(gna_dev(gna_priv), "GNA error: PCI MMU Error\n");

	if (hw_status & GNA_STS_PCI_UNEXCOMPL_ERR)
		dev_dbg(gna_dev(gna_priv), "GNA error: PCI Unexpected Completion Error\n");

	if (hw_status & GNA_STS_SATURATE)
		dev_dbg(gna_dev(gna_priv), "GNA error: Saturation Reached !\n");
}

bool gna_hw_perf_enabled(struct gna_device *gna_priv)
{
	u32 ctrl = gna_reg_read(gna_priv, GNA_MMIO_CTRL);

	return !!FIELD_GET(GNA_CTRL_COMP_STATS_EN, ctrl);
}

void gna_start_scoring(struct gna_device *gna_priv,
		       struct gna_compute_cfg *compute_cfg)
{
	u32 ctrl = gna_reg_read(gna_priv, GNA_MMIO_CTRL);

	ctrl |= GNA_CTRL_START_ACCEL | GNA_CTRL_COMP_INT_EN | GNA_CTRL_ERR_INT_EN;

	ctrl &= ~GNA_CTRL_COMP_STATS_EN;
	ctrl |= FIELD_PREP(GNA_CTRL_COMP_STATS_EN,
			compute_cfg->hw_perf_encoding & FIELD_MAX(GNA_CTRL_COMP_STATS_EN));

	ctrl &= ~GNA_CTRL_ACTIVE_LIST_EN;
	ctrl |= FIELD_PREP(GNA_CTRL_ACTIVE_LIST_EN,
			compute_cfg->active_list_on & FIELD_MAX(GNA_CTRL_ACTIVE_LIST_EN));

	ctrl &= ~GNA_CTRL_OP_MODE;
	ctrl |= FIELD_PREP(GNA_CTRL_OP_MODE,
			compute_cfg->gna_mode & FIELD_MAX(GNA_CTRL_OP_MODE));

	gna_reg_write(gna_priv, GNA_MMIO_CTRL, ctrl);
}

static void gna_clear_saturation(struct gna_device *gna_priv)
{
	u32 val;

	val = gna_reg_read(gna_priv, GNA_MMIO_STS);
	if (val & GNA_STS_SATURATE) {
		dev_dbg(gna_dev(gna_priv), "status (saturation): %#x\n", val);

		val = val & GNA_STS_SATURATE;
		gna_reg_write(gna_priv, GNA_MMIO_STS, val);
	}
}

int gna_abort_hw(struct gna_device *gna_priv)
{
	u32 val;

	/* saturation bit in the GNA status register needs
	 * to be explicitly cleared.
	 */
	gna_clear_saturation(gna_priv);

	val = gna_reg_read(gna_priv, GNA_MMIO_STS);
	dev_dbg(gna_dev(gna_priv), "status (before abort): %#x\n", val);

	val = gna_reg_read(gna_priv, GNA_MMIO_CTRL);
	val |= GNA_CTRL_ABORT_CLR_ACCEL;
	gna_reg_write(gna_priv, GNA_MMIO_CTRL, val);

	return readl_poll_timeout(gna_priv->iobase + GNA_MMIO_STS, val,
				!(val & 0x1),
				0, 1000);
}
