// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2016 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 * Copyright (C) 2015 Amlogic, Inc. All rights reserved.
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>

#include <drm/bridge/dw_hdmi.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_device.h>
#include <drm/drm_edid.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_print.h>

#include <linux/videodev2.h>

#include "meson_drv.h"
#include "meson_dw_hdmi.h"
#include "meson_registers.h"

#define DRIVER_NAME "meson-dw-hdmi"
#define DRIVER_DESC "Amlogic Meson HDMI-TX DRM driver"

/**
 * DOC: HDMI Output
 *
 * HDMI Output is composed of :
 *
 * - A Synopsys DesignWare HDMI Controller IP
 * - A TOP control block controlling the Clocks and PHY
 * - A custom HDMI PHY in order convert video to TMDS signal
 *
 * .. code::
 *
 *    ___________________________________
 *   |            HDMI TOP               |<= HPD
 *   |___________________________________|
 *   |                  |                |
 *   |  Synopsys HDMI   |   HDMI PHY     |=> TMDS
 *   |    Controller    |________________|
 *   |___________________________________|<=> DDC
 *
 *
 * The HDMI TOP block only supports HPD sensing.
 * The Synopsys HDMI Controller interrupt is routed
 * through the TOP Block interrupt.
 * Communication to the TOP Block and the Synopsys
 * HDMI Controller is done a pair of addr+read/write
 * registers.
 * The HDMI PHY is configured by registers in the
 * HHI register block.
 *
 * Pixel data arrives in 4:4:4 format from the VENC
 * block and the VPU HDMI mux selects either the ENCI
 * encoder for the 576i or 480i formats or the ENCP
 * encoder for all the other formats including
 * interlaced HD formats.
 * The VENC uses a DVI encoder on top of the ENCI
 * or ENCP encoders to generate DVI timings for the
 * HDMI controller.
 *
 * GXBB, GXL and GXM embeds the Synopsys DesignWare
 * HDMI TX IP version 2.01a with HDCP and I2C & S/PDIF
 * audio source interfaces.
 *
 * We handle the following features :
 *
 * - HPD Rise & Fall interrupt
 * - HDMI Controller Interrupt
 * - HDMI PHY Init for 480i to 1080p60
 * - VENC & HDMI Clock setup for 480i to 1080p60
 * - VENC Mode setup for 480i to 1080p60
 *
 * What is missing :
 *
 * - PHY, Clock and Mode setup for 2k && 4k modes
 * - SDDC Scrambling mode for HDMI 2.0a
 * - HDCP Setup
 * - CEC Management
 */

/* TOP Block Communication Channel */
#define HDMITX_TOP_ADDR_REG	0x0
#define HDMITX_TOP_DATA_REG	0x4
#define HDMITX_TOP_CTRL_REG	0x8
#define HDMITX_TOP_G12A_OFFSET	0x8000

/* Controller Communication Channel */
#define HDMITX_DWC_ADDR_REG	0x10
#define HDMITX_DWC_DATA_REG	0x14
#define HDMITX_DWC_CTRL_REG	0x18

/* HHI Registers */
#define HHI_MEM_PD_REG0		0x100 /* 0x40 */
#define HHI_HDMI_CLK_CNTL	0x1cc /* 0x73 */
#define HHI_HDMI_PHY_CNTL0	0x3a0 /* 0xe8 */
#define HHI_HDMI_PHY_CNTL1	0x3a4 /* 0xe9 */
#define  PHY_CNTL1_INIT		0x03900000
#define  PHY_INVERT		BIT(17)
#define HHI_HDMI_PHY_CNTL2	0x3a8 /* 0xea */
#define HHI_HDMI_PHY_CNTL3	0x3ac /* 0xeb */
#define HHI_HDMI_PHY_CNTL4	0x3b0 /* 0xec */
#define HHI_HDMI_PHY_CNTL5	0x3b4 /* 0xed */

static DEFINE_SPINLOCK(reg_lock);

enum meson_venc_source {
	MESON_VENC_SOURCE_NONE = 0,
	MESON_VENC_SOURCE_ENCI = 1,
	MESON_VENC_SOURCE_ENCP = 2,
};

struct meson_dw_hdmi;

struct meson_dw_hdmi_data {
	unsigned int	(*top_read)(struct meson_dw_hdmi *dw_hdmi,
				    unsigned int addr);
	void		(*top_write)(struct meson_dw_hdmi *dw_hdmi,
				     unsigned int addr, unsigned int data);
	unsigned int	(*dwc_read)(struct meson_dw_hdmi *dw_hdmi,
				    unsigned int addr);
	void		(*dwc_write)(struct meson_dw_hdmi *dw_hdmi,
				     unsigned int addr, unsigned int data);
	u32 cntl0_init;
	u32 cntl1_init;
};

struct meson_dw_hdmi {
	struct dw_hdmi_plat_data dw_plat_data;
	struct meson_drm *priv;
	struct device *dev;
	void __iomem *hdmitx;
	const struct meson_dw_hdmi_data *data;
	struct reset_control *hdmitx_apb;
	struct reset_control *hdmitx_ctrl;
	struct reset_control *hdmitx_phy;
	u32 irq_stat;
	struct dw_hdmi *hdmi;
	struct drm_bridge *bridge;
};

static inline int dw_hdmi_is_compatible(struct meson_dw_hdmi *dw_hdmi,
					const char *compat)
{
	return of_device_is_compatible(dw_hdmi->dev->of_node, compat);
}

/* PHY (via TOP bridge) and Controller dedicated register interface */

static unsigned int dw_hdmi_top_read(struct meson_dw_hdmi *dw_hdmi,
				     unsigned int addr)
{
	unsigned long flags;
	unsigned int data;

	spin_lock_irqsave(&reg_lock, flags);

	/* ADDR must be written twice */
	writel(addr & 0xffff, dw_hdmi->hdmitx + HDMITX_TOP_ADDR_REG);
	writel(addr & 0xffff, dw_hdmi->hdmitx + HDMITX_TOP_ADDR_REG);

	/* Read needs a second DATA read */
	data = readl(dw_hdmi->hdmitx + HDMITX_TOP_DATA_REG);
	data = readl(dw_hdmi->hdmitx + HDMITX_TOP_DATA_REG);

	spin_unlock_irqrestore(&reg_lock, flags);

	return data;
}

static unsigned int dw_hdmi_g12a_top_read(struct meson_dw_hdmi *dw_hdmi,
					  unsigned int addr)
{
	return readl(dw_hdmi->hdmitx + HDMITX_TOP_G12A_OFFSET + (addr << 2));
}

static inline void dw_hdmi_top_write(struct meson_dw_hdmi *dw_hdmi,
				     unsigned int addr, unsigned int data)
{
	unsigned long flags;

	spin_lock_irqsave(&reg_lock, flags);

	/* ADDR must be written twice */
	writel(addr & 0xffff, dw_hdmi->hdmitx + HDMITX_TOP_ADDR_REG);
	writel(addr & 0xffff, dw_hdmi->hdmitx + HDMITX_TOP_ADDR_REG);

	/* Write needs single DATA write */
	writel(data, dw_hdmi->hdmitx + HDMITX_TOP_DATA_REG);

	spin_unlock_irqrestore(&reg_lock, flags);
}

static inline void dw_hdmi_g12a_top_write(struct meson_dw_hdmi *dw_hdmi,
					  unsigned int addr, unsigned int data)
{
	writel(data, dw_hdmi->hdmitx + HDMITX_TOP_G12A_OFFSET + (addr << 2));
}

/* Helper to change specific bits in PHY registers */
static inline void dw_hdmi_top_write_bits(struct meson_dw_hdmi *dw_hdmi,
					  unsigned int addr,
					  unsigned int mask,
					  unsigned int val)
{
	unsigned int data = dw_hdmi->data->top_read(dw_hdmi, addr);

	data &= ~mask;
	data |= val;

	dw_hdmi->data->top_write(dw_hdmi, addr, data);
}

static unsigned int dw_hdmi_dwc_read(struct meson_dw_hdmi *dw_hdmi,
				     unsigned int addr)
{
	unsigned long flags;
	unsigned int data;

	spin_lock_irqsave(&reg_lock, flags);

	/* ADDR must be written twice */
	writel(addr & 0xffff, dw_hdmi->hdmitx + HDMITX_DWC_ADDR_REG);
	writel(addr & 0xffff, dw_hdmi->hdmitx + HDMITX_DWC_ADDR_REG);

	/* Read needs a second DATA read */
	data = readl(dw_hdmi->hdmitx + HDMITX_DWC_DATA_REG);
	data = readl(dw_hdmi->hdmitx + HDMITX_DWC_DATA_REG);

	spin_unlock_irqrestore(&reg_lock, flags);

	return data;
}

static unsigned int dw_hdmi_g12a_dwc_read(struct meson_dw_hdmi *dw_hdmi,
					  unsigned int addr)
{
	return readb(dw_hdmi->hdmitx + addr);
}

static inline void dw_hdmi_dwc_write(struct meson_dw_hdmi *dw_hdmi,
				     unsigned int addr, unsigned int data)
{
	unsigned long flags;

	spin_lock_irqsave(&reg_lock, flags);

	/* ADDR must be written twice */
	writel(addr & 0xffff, dw_hdmi->hdmitx + HDMITX_DWC_ADDR_REG);
	writel(addr & 0xffff, dw_hdmi->hdmitx + HDMITX_DWC_ADDR_REG);

	/* Write needs single DATA write */
	writel(data, dw_hdmi->hdmitx + HDMITX_DWC_DATA_REG);

	spin_unlock_irqrestore(&reg_lock, flags);
}

static inline void dw_hdmi_g12a_dwc_write(struct meson_dw_hdmi *dw_hdmi,
					  unsigned int addr, unsigned int data)
{
	writeb(data, dw_hdmi->hdmitx + addr);
}

/* Bridge */

/* Setup PHY bandwidth modes */
static void meson_hdmi_phy_setup_mode(struct meson_dw_hdmi *dw_hdmi,
				      unsigned int tmds_clock)
{
	struct meson_drm *priv = dw_hdmi->priv;

	/*
	 * The PHY bandwidth band must be picked from the real TMDS CHARACTER
	 * RATE (in kHz, computed once by the caller -- dw_hdmi_phy_init() --
	 * and passed in here so this function and the >340MHz TMDS-pattern /
	 * SCDC decision in dw_hdmi_phy_init() can never disagree), not the raw
	 * pixel clock: 4:2:0 halves the pixel rate and, together with 4:4:4,
	 * scales by depth/8 for deep color; 4:2:2 never scales (it carries any
	 * depth within the 8-bit bandwidth). e.g. 4K60 420-10 = 594/2 * 10/8 =
	 * 371.25MHz (the 3.7Gbps band), NOT the 297MHz (2.97Gbps band) that the
	 * halved pixel clock alone selects -- which leaves the link
	 * under-driven / marginal.
	 */
	if (dw_hdmi_is_compatible(dw_hdmi, "amlogic,meson-gxl-dw-hdmi") ||
	    dw_hdmi_is_compatible(dw_hdmi, "amlogic,meson-gxm-dw-hdmi")) {
		if (tmds_clock >= 371250) {
			/* 5.94Gbps, 3.7125Gbps */
			regmap_write(priv->hhi, HHI_HDMI_PHY_CNTL0, 0x333d3282);
			regmap_write(priv->hhi, HHI_HDMI_PHY_CNTL3, 0x2136315b);
		} else if (tmds_clock >= 297000) {
			/* 2.97Gbps */
			regmap_write(priv->hhi, HHI_HDMI_PHY_CNTL0, 0x33303382);
			regmap_write(priv->hhi, HHI_HDMI_PHY_CNTL3, 0x2036315b);
		} else if (tmds_clock >= 148500) {
			/* 1.485Gbps */
			regmap_write(priv->hhi, HHI_HDMI_PHY_CNTL0, 0x33303362);
			regmap_write(priv->hhi, HHI_HDMI_PHY_CNTL3, 0x2016315b);
		} else {
			/* 742.5Mbps, and below */
			regmap_write(priv->hhi, HHI_HDMI_PHY_CNTL0, 0x33604142);
			regmap_write(priv->hhi, HHI_HDMI_PHY_CNTL3, 0x0016315b);
		}
	} else if (dw_hdmi_is_compatible(dw_hdmi,
					 "amlogic,meson-gxbb-dw-hdmi")) {
		if (tmds_clock >= 371250) {
			/* 5.94Gbps, 3.7125Gbps */
			regmap_write(priv->hhi, HHI_HDMI_PHY_CNTL0, 0x33353245);
			regmap_write(priv->hhi, HHI_HDMI_PHY_CNTL3, 0x2100115b);
		} else if (tmds_clock >= 297000) {
			/* 2.97Gbps */
			regmap_write(priv->hhi, HHI_HDMI_PHY_CNTL0, 0x33634283);
			regmap_write(priv->hhi, HHI_HDMI_PHY_CNTL3, 0xb000115b);
		} else {
			/* 1.485Gbps, and below */
			regmap_write(priv->hhi, HHI_HDMI_PHY_CNTL0, 0x33632122);
			regmap_write(priv->hhi, HHI_HDMI_PHY_CNTL3, 0x2000115b);
		}
	} else if (dw_hdmi_is_compatible(dw_hdmi,
					 "amlogic,meson-g12a-dw-hdmi")) {
		if (tmds_clock >= 371250) {
			/* 5.94Gbps, 3.7125Gbps */
			regmap_write(priv->hhi, HHI_HDMI_PHY_CNTL0, 0x37eb65c4);
			regmap_write(priv->hhi, HHI_HDMI_PHY_CNTL3, 0x2ab0ff3b);
			regmap_write(priv->hhi, HHI_HDMI_PHY_CNTL5, 0x0000080b);
		} else if (tmds_clock >= 297000) {
			/* 2.97Gbps */
			regmap_write(priv->hhi, HHI_HDMI_PHY_CNTL0, 0x33eb6262);
			regmap_write(priv->hhi, HHI_HDMI_PHY_CNTL3, 0x2ab0ff3b);
			regmap_write(priv->hhi, HHI_HDMI_PHY_CNTL5, 0x00000003);
		} else {
			/* 1.485Gbps, and below */
			regmap_write(priv->hhi, HHI_HDMI_PHY_CNTL0, 0x33eb4242);
			regmap_write(priv->hhi, HHI_HDMI_PHY_CNTL3, 0x2ab0ff3b);
			regmap_write(priv->hhi, HHI_HDMI_PHY_CNTL5, 0x00000003);
		}
	}
}

static inline void meson_dw_hdmi_phy_reset(struct meson_dw_hdmi *dw_hdmi)
{
	struct meson_drm *priv = dw_hdmi->priv;

	/* Enable and software reset */
	regmap_update_bits(priv->hhi, HHI_HDMI_PHY_CNTL1, 0xf, 0xf);

	mdelay(2);

	/* Enable and unreset */
	regmap_update_bits(priv->hhi, HHI_HDMI_PHY_CNTL1, 0xf, 0xe);

	mdelay(2);
}

static int dw_hdmi_phy_init(struct dw_hdmi *hdmi, void *data,
			    const struct drm_display_info *display,
			    const struct drm_display_mode *mode)
{
	struct meson_dw_hdmi *dw_hdmi = (struct meson_dw_hdmi *)data;
	struct meson_drm *priv = dw_hdmi->priv;
	unsigned int wr_clk =
		readl_relaxed(priv->io_base + _REG(VPU_HDMI_SETTING));
	/*
	 * E5: this used to also OR in drm_mode_is_420_only(display, mode) and
	 * (!is_hdmi2_sink && drm_mode_is_420_also(display, mode)) -- PRE-
	 * negotiation predictions mirrored from dw-hdmi's own eligibility gate
	 * for OFFERING a 420 candidate. They go stale whenever DC-002's
	 * 444-preferred candidate ordering (dw-hdmi.c
	 * atomic_get_output_bus_fmts, ~2994-3001) actually picks 444/422 on a
	 * 420-also sink: with is_hdmi2_sink scdc-only here, a 420-also
	 * non-SCDC sink kept "!is_hdmi2_sink && 420_also" == true even though
	 * the negotiated link is 444 -- under-driving the PHY band and, at
	 * 4K60 444 (594MHz), selecting the LOW (<=340MHz) TMDS pattern for a
	 * link that needs the high pattern + 1/40 scrambling. Black screen.
	 *
	 * dw_hdmi_bus_fmt_is_420(hdmi) reads hdmi->hdmi_data.enc_out_bus_format
	 * directly -- the ACTUAL negotiated format -- and is already the sole
	 * source of truth this function trusts for mode_is_422 one line below
	 * (since E3). By the time phy_init runs, negotiation is complete (see
	 * the E14 comment on color_depth below), so ground truth alone is
	 * both sufficient and correct.
	 */
	bool mode_is_420 = dw_hdmi_bus_fmt_is_420(hdmi);
	bool mode_is_422 = dw_hdmi_bus_fmt_is_422(hdmi);
	unsigned int color_depth = dw_hdmi_bus_fmt_color_depth(hdmi);
	unsigned int tmds_clock;

	/*
	 * E14: color_depth is 0 for MEDIA_BUS_FMT_FIXED (dw-hdmi's default
	 * case) -- e.g. before bus-format negotiation has actually run.
	 * Unguarded, this zeroes tmds_clock below, forcing the lowest PHY
	 * band + div10 pattern for every mode. Unreachable today (meson is
	 * always the first/only bridge negotiating a real format by the
	 * time this runs), but a silent 0 here is unsafe -- clamp to the
	 * 8-bit default and log loudly instead.
	 */
	if (WARN_ON(!color_depth))
		color_depth = 8;

	/*
	 * Real TMDS character rate (kHz): 4:2:0 halves the pixel rate, and
	 * 4:4:4/4:2:0 deep color scales it by depth/8 -- but 4:2:2 does NOT get
	 * this bump (it carries any depth within the SAME bandwidth as 8-bit
	 * 4:4:4/4:2:2, matching dw-hdmi's OWN mtmdsclock computation, which
	 * excludes 4:2:2 from the depth scaling -- see hdmi_bus_fmt_is_yuv422()
	 * in dw-hdmi.c). Scaling 4:2:2 anyway (the bug this fixes) over-selects
	 * the PHY band and wrongly enables the >340MHz TMDS pattern, so meson's
	 * belief and dw-hdmi's own (correct) mtmdsclock -- and thus its SCDC
	 * 1/40 bit-clock-ratio decision -- disagree and the sink can't lock.
	 * The high-TMDS (>340MHz) clock pattern below + the PHY band + the SCDC
	 * ratio all key off THIS value, not the pixel clock -- e.g. 4K60 420-10
	 * is 371.25MHz (>340), which the pixel clock (halved to 297) alone
	 * misses, leaving the high pattern off + the PHY under-driven -> a
	 * marginal link the sink cannot lock.
	 */
	tmds_clock = mode->clock;
	if (mode_is_420)
		tmds_clock /= 2;
	if (!mode_is_422)
		tmds_clock = tmds_clock * color_depth / 8;

	DRM_DEBUG_DRIVER("\"%s\" tmds=%ukHz div%d\n", mode->name,
			 tmds_clock, tmds_clock > 340000 ? 40 : 10);

	/* TMDS pattern setup */
	if (tmds_clock > 340000) {
		dw_hdmi->data->top_write(dw_hdmi, HDMITX_TOP_TMDS_CLK_PTTN_01,
				  0);
		dw_hdmi->data->top_write(dw_hdmi, HDMITX_TOP_TMDS_CLK_PTTN_23,
				  0x03ff03ff);
	} else {
		dw_hdmi->data->top_write(dw_hdmi, HDMITX_TOP_TMDS_CLK_PTTN_01,
				  0x001f001f);
		dw_hdmi->data->top_write(dw_hdmi, HDMITX_TOP_TMDS_CLK_PTTN_23,
				  0x001f001f);
	}

	/* Load TMDS pattern */
	dw_hdmi->data->top_write(dw_hdmi, HDMITX_TOP_TMDS_CLK_PTTN_CNTL, 0x1);
	msleep(20);
	dw_hdmi->data->top_write(dw_hdmi, HDMITX_TOP_TMDS_CLK_PTTN_CNTL, 0x2);

	/* Setup PHY parameters */
	meson_hdmi_phy_setup_mode(dw_hdmi, tmds_clock);

	/* Disable clock, fifo, fifo_wr */
	regmap_update_bits(priv->hhi, HHI_HDMI_PHY_CNTL1, 0xf, 0);

	dw_hdmi_set_high_tmds_clock_ratio(hdmi, display);

	msleep(100);

	/* Reset PHY 3 times in a row */
	meson_dw_hdmi_phy_reset(dw_hdmi);
	meson_dw_hdmi_phy_reset(dw_hdmi);
	meson_dw_hdmi_phy_reset(dw_hdmi);

	/* Temporary Disable VENC video stream */
	if (priv->venc.hdmi_use_enci)
		writel_relaxed(0, priv->io_base + _REG(ENCI_VIDEO_EN));
	else
		writel_relaxed(0, priv->io_base + _REG(ENCP_VIDEO_EN));

	/*
	 * DC-001 fix (2026-06-08): re-sync the VPU->HDMI-TX async FIFO in the BSP
	 * enc_vpu_bridge_reset() order -- stop FIFO source + wr_clk, restore
	 * wr_clk, re-arm the source, then enable VENC LAST -- with a ~1ms settle
	 * after each VPU_HDMI_SETTING transition. The previous code re-enabled
	 * VENC BEFORE restoring wr_clk/source and used no delays, so the FIFO
	 * re-armed at a wrong word phase. Harmless at 8-bit (1 byte/pixel
	 * self-aligns) but at 10-bit the packed word straddles the boundary and
	 * the per-channel misalignment locks in -> scrambled deep-color output.
	 */

	/* Temporary Disable HDMI video stream to HDMI-TX (source + wr_clk off) */
	writel_bits_relaxed(0x3, 0,
			    priv->io_base + _REG(VPU_HDMI_SETTING));
	writel_bits_relaxed(0xf << 8, 0,
			    priv->io_base + _REG(VPU_HDMI_SETTING));
	usleep_range(1000, 1005);

	/* Push back HDMI clock settings (restore wr_clk) */
	writel_bits_relaxed(0xf << 8, wr_clk & (0xf << 8),
			    priv->io_base + _REG(VPU_HDMI_SETTING));
	usleep_range(1000, 1005);

	/* Enable and Select HDMI video source for HDMI-TX (re-arm FIFO source) */
	if (priv->venc.hdmi_use_enci)
		writel_bits_relaxed(0x3, MESON_VENC_SOURCE_ENCI,
				    priv->io_base + _REG(VPU_HDMI_SETTING));
	else
		writel_bits_relaxed(0x3, MESON_VENC_SOURCE_ENCP,
				    priv->io_base + _REG(VPU_HDMI_SETTING));
	usleep_range(1000, 1005);

	/* Re-Enable VENC video stream LAST, after the FIFO is fully re-armed */
	if (priv->venc.hdmi_use_enci)
		writel_relaxed(1, priv->io_base + _REG(ENCI_VIDEO_EN));
	else
		writel_relaxed(1, priv->io_base + _REG(ENCP_VIDEO_EN));

	return 0;
}

static void dw_hdmi_phy_disable(struct dw_hdmi *hdmi,
				void *data)
{
	struct meson_dw_hdmi *dw_hdmi = (struct meson_dw_hdmi *)data;
	struct meson_drm *priv = dw_hdmi->priv;

	DRM_DEBUG_DRIVER("\n");

	/* Fallback to init mode */
	regmap_write(priv->hhi, HHI_HDMI_PHY_CNTL1, dw_hdmi->data->cntl1_init);
	regmap_write(priv->hhi, HHI_HDMI_PHY_CNTL0, dw_hdmi->data->cntl0_init);
}

static enum drm_connector_status dw_hdmi_read_hpd(struct dw_hdmi *hdmi,
			     void *data)
{
	struct meson_dw_hdmi *dw_hdmi = (struct meson_dw_hdmi *)data;

	return !!dw_hdmi->data->top_read(dw_hdmi, HDMITX_TOP_STAT0) ?
		connector_status_connected : connector_status_disconnected;
}

static void dw_hdmi_setup_hpd(struct dw_hdmi *hdmi,
			      void *data)
{
	struct meson_dw_hdmi *dw_hdmi = (struct meson_dw_hdmi *)data;

	/* Setup HPD Filter */
	dw_hdmi->data->top_write(dw_hdmi, HDMITX_TOP_HPD_FILTER,
			  (0xa << 12) | 0xa0);

	/* Clear interrupts */
	dw_hdmi->data->top_write(dw_hdmi, HDMITX_TOP_INTR_STAT_CLR,
			  HDMITX_TOP_INTR_HPD_RISE | HDMITX_TOP_INTR_HPD_FALL);

	/* Unmask interrupts */
	dw_hdmi_top_write_bits(dw_hdmi, HDMITX_TOP_INTR_MASKN,
			HDMITX_TOP_INTR_HPD_RISE | HDMITX_TOP_INTR_HPD_FALL,
			HDMITX_TOP_INTR_HPD_RISE | HDMITX_TOP_INTR_HPD_FALL);
}

static const struct dw_hdmi_phy_ops meson_dw_hdmi_phy_ops = {
	.init = dw_hdmi_phy_init,
	.disable = dw_hdmi_phy_disable,
	.read_hpd = dw_hdmi_read_hpd,
	.setup_hpd = dw_hdmi_setup_hpd,
};

static irqreturn_t dw_hdmi_top_irq(int irq, void *dev_id)
{
	struct meson_dw_hdmi *dw_hdmi = dev_id;
	u32 stat;

	stat = dw_hdmi->data->top_read(dw_hdmi, HDMITX_TOP_INTR_STAT);
	dw_hdmi->data->top_write(dw_hdmi, HDMITX_TOP_INTR_STAT_CLR, stat);

	/* HPD Events, handle in the threaded interrupt handler */
	if (stat & (HDMITX_TOP_INTR_HPD_RISE | HDMITX_TOP_INTR_HPD_FALL)) {
		dw_hdmi->irq_stat = stat;
		return IRQ_WAKE_THREAD;
	}

	/* HDMI Controller Interrupt */
	if (stat & 1)
		return IRQ_NONE;

	/* TOFIX Handle HDCP Interrupts */

	return IRQ_HANDLED;
}

/* Threaded interrupt handler to manage HPD events */
static irqreturn_t dw_hdmi_top_thread_irq(int irq, void *dev_id)
{
	struct meson_dw_hdmi *dw_hdmi = dev_id;
	u32 stat = dw_hdmi->irq_stat;

	/* HPD Events */
	if (stat & (HDMITX_TOP_INTR_HPD_RISE | HDMITX_TOP_INTR_HPD_FALL)) {
		bool hpd_connected = false;

		if (stat & HDMITX_TOP_INTR_HPD_RISE)
			hpd_connected = true;

		dw_hdmi_setup_rx_sense(dw_hdmi->hdmi, hpd_connected,
				       hpd_connected);

		drm_helper_hpd_irq_event(dw_hdmi->bridge->dev);
		drm_bridge_hpd_notify(dw_hdmi->bridge,
				      hpd_connected ? connector_status_connected
						    : connector_status_disconnected);
	}

	return IRQ_HANDLED;
}

/* DW HDMI Regmap */

static int meson_dw_hdmi_reg_read(void *context, unsigned int reg,
				  unsigned int *result)
{
	struct meson_dw_hdmi *dw_hdmi = context;

	*result = dw_hdmi->data->dwc_read(dw_hdmi, reg);

	return 0;

}

static int meson_dw_hdmi_reg_write(void *context, unsigned int reg,
				   unsigned int val)
{
	struct meson_dw_hdmi *dw_hdmi = context;

	dw_hdmi->data->dwc_write(dw_hdmi, reg, val);

	return 0;
}

static const struct regmap_config meson_dw_hdmi_regmap_config = {
	.reg_bits = 32,
	.val_bits = 8,
	.reg_read = meson_dw_hdmi_reg_read,
	.reg_write = meson_dw_hdmi_reg_write,
	.max_register = 0x10000,
	.fast_io = true,
};

static const struct meson_dw_hdmi_data meson_dw_hdmi_gxbb_data = {
	.top_read = dw_hdmi_top_read,
	.top_write = dw_hdmi_top_write,
	.dwc_read = dw_hdmi_dwc_read,
	.dwc_write = dw_hdmi_dwc_write,
	.cntl0_init = 0x0,
	.cntl1_init = PHY_CNTL1_INIT | PHY_INVERT,
};

static const struct meson_dw_hdmi_data meson_dw_hdmi_gxl_data = {
	.top_read = dw_hdmi_top_read,
	.top_write = dw_hdmi_top_write,
	.dwc_read = dw_hdmi_dwc_read,
	.dwc_write = dw_hdmi_dwc_write,
	.cntl0_init = 0x0,
	.cntl1_init = PHY_CNTL1_INIT,
};

static const struct meson_dw_hdmi_data meson_dw_hdmi_g12a_data = {
	.top_read = dw_hdmi_g12a_top_read,
	.top_write = dw_hdmi_g12a_top_write,
	.dwc_read = dw_hdmi_g12a_dwc_read,
	.dwc_write = dw_hdmi_g12a_dwc_write,
	.cntl0_init = 0x000b4242, /* Bandgap */
	.cntl1_init = PHY_CNTL1_INIT,
};

static void meson_dw_hdmi_init(struct meson_dw_hdmi *meson_dw_hdmi)
{
	struct meson_drm *priv = meson_dw_hdmi->priv;

	/* Enable clocks */
	regmap_update_bits(priv->hhi, HHI_HDMI_CLK_CNTL, 0xffff, 0x100);

	/* Bring HDMITX MEM output of power down */
	regmap_update_bits(priv->hhi, HHI_MEM_PD_REG0, 0xff << 8, 0);

	/* Reset HDMITX APB & TX & PHY */
	reset_control_reset(meson_dw_hdmi->hdmitx_apb);
	reset_control_reset(meson_dw_hdmi->hdmitx_ctrl);
	reset_control_reset(meson_dw_hdmi->hdmitx_phy);

	/* Enable APB3 fail on error */
	if (!meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A)) {
		writel_bits_relaxed(BIT(15), BIT(15),
				    meson_dw_hdmi->hdmitx + HDMITX_TOP_CTRL_REG);
		writel_bits_relaxed(BIT(15), BIT(15),
				    meson_dw_hdmi->hdmitx + HDMITX_DWC_CTRL_REG);
	}

	/* Bring out of reset */
	meson_dw_hdmi->data->top_write(meson_dw_hdmi,
				       HDMITX_TOP_SW_RESET,  0);

	msleep(20);

	meson_dw_hdmi->data->top_write(meson_dw_hdmi,
				       HDMITX_TOP_CLK_CNTL, 0xff);

	/* Enable normal output to PHY */
	meson_dw_hdmi->data->top_write(meson_dw_hdmi, HDMITX_TOP_BIST_CNTL, BIT(12));

	/* Setup PHY */
	regmap_write(priv->hhi, HHI_HDMI_PHY_CNTL1, meson_dw_hdmi->data->cntl1_init);
	regmap_write(priv->hhi, HHI_HDMI_PHY_CNTL0, meson_dw_hdmi->data->cntl0_init);

	/* Enable HDMI-TX Interrupt */
	meson_dw_hdmi->data->top_write(meson_dw_hdmi, HDMITX_TOP_INTR_STAT_CLR,
				       HDMITX_TOP_INTR_CORE);

	meson_dw_hdmi->data->top_write(meson_dw_hdmi, HDMITX_TOP_INTR_MASKN,
				       HDMITX_TOP_INTR_CORE);

}

static void meson_disable_clk(void *data)
{
	clk_disable_unprepare(data);
}

static int meson_enable_clk(struct device *dev, char *name)
{
	struct clk *clk;
	int ret;

	clk = devm_clk_get(dev, name);
	if (IS_ERR(clk)) {
		dev_err(dev, "Unable to get %s pclk\n", name);
		return PTR_ERR(clk);
	}

	ret = clk_prepare_enable(clk);
	if (!ret)
		ret = devm_add_action_or_reset(dev, meson_disable_clk, clk);

	return ret;
}

static void meson_dw_hdmi_del_link(void *data)
{
	device_link_del(data);
}

static int meson_dw_hdmi_bind(struct device *dev, struct device *master,
				void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	const struct meson_dw_hdmi_data *match;
	struct meson_dw_hdmi *meson_dw_hdmi;
	struct drm_device *drm = data;
	struct meson_drm *priv = drm->dev_private;
	struct dw_hdmi_plat_data *dw_plat_data;
	struct device_link *link;
	int irq;
	int ret;

	DRM_DEBUG_DRIVER("\n");

	match = of_device_get_match_data(&pdev->dev);
	if (!match) {
		dev_err(&pdev->dev, "failed to get match data\n");
		return -ENODEV;
	}

	/*
	 * Enforce suspend/resume ordering by making the HDMI TX a supplier
	 * of the DRM master: it is suspended after and resumed before the
	 * master. The master's resume re-enables the dw-hdmi bridge, which
	 * needs the HDMI TX re-initialised by meson_dw_hdmi_pm_resume()
	 * first when the power domain was switched off across suspend.
	 */
	link = device_link_add(master, dev, DL_FLAG_STATELESS);
	if (!link) {
		dev_err(dev, "failed to create device link to %s\n",
			dev_name(master));
		return -EINVAL;
	}

	ret = devm_add_action_or_reset(dev, meson_dw_hdmi_del_link, link);
	if (ret)
		return ret;

	meson_dw_hdmi = devm_kzalloc(dev, sizeof(*meson_dw_hdmi),
				     GFP_KERNEL);
	if (!meson_dw_hdmi)
		return -ENOMEM;

	meson_dw_hdmi->priv = priv;
	meson_dw_hdmi->dev = dev;
	meson_dw_hdmi->data = match;
	dw_plat_data = &meson_dw_hdmi->dw_plat_data;

	ret = devm_regulator_get_enable_optional(dev, "hdmi");
	if (ret < 0 && ret != -ENODEV)
		return ret;

	meson_dw_hdmi->hdmitx_apb = devm_reset_control_get_exclusive(dev,
						"hdmitx_apb");
	if (IS_ERR(meson_dw_hdmi->hdmitx_apb)) {
		dev_err(dev, "Failed to get hdmitx_apb reset\n");
		return PTR_ERR(meson_dw_hdmi->hdmitx_apb);
	}

	meson_dw_hdmi->hdmitx_ctrl = devm_reset_control_get_exclusive(dev,
						"hdmitx");
	if (IS_ERR(meson_dw_hdmi->hdmitx_ctrl)) {
		dev_err(dev, "Failed to get hdmitx reset\n");
		return PTR_ERR(meson_dw_hdmi->hdmitx_ctrl);
	}

	meson_dw_hdmi->hdmitx_phy = devm_reset_control_get_exclusive(dev,
						"hdmitx_phy");
	if (IS_ERR(meson_dw_hdmi->hdmitx_phy)) {
		dev_err(dev, "Failed to get hdmitx_phy reset\n");
		return PTR_ERR(meson_dw_hdmi->hdmitx_phy);
	}

	meson_dw_hdmi->hdmitx = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(meson_dw_hdmi->hdmitx))
		return PTR_ERR(meson_dw_hdmi->hdmitx);

	ret = meson_enable_clk(dev, "isfr");
	if (ret)
		return ret;

	ret = meson_enable_clk(dev, "iahb");
	if (ret)
		return ret;

	ret = meson_enable_clk(dev, "venci");
	if (ret)
		return ret;

	dw_plat_data->regm = devm_regmap_init(dev, NULL, meson_dw_hdmi,
					      &meson_dw_hdmi_regmap_config);
	if (IS_ERR(dw_plat_data->regm))
		return PTR_ERR(dw_plat_data->regm);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_threaded_irq(dev, irq, dw_hdmi_top_irq,
					dw_hdmi_top_thread_irq, IRQF_SHARED,
					"dw_hdmi_top_irq", meson_dw_hdmi);
	if (ret) {
		dev_err(dev, "Failed to request hdmi top irq\n");
		return ret;
	}

	meson_dw_hdmi_init(meson_dw_hdmi);

	/* Bridge / Connector */

	dw_plat_data->priv_data = meson_dw_hdmi;
	dw_plat_data->phy_ops = &meson_dw_hdmi_phy_ops;
	dw_plat_data->phy_name = "meson_dw_hdmi_phy";
	dw_plat_data->phy_data = meson_dw_hdmi;
	dw_plat_data->input_bus_encoding = V4L2_YCBCR_ENC_709;
	dw_plat_data->ycbcr_420_allowed = true;
	dw_plat_data->disable_cec = true;
	dw_plat_data->amd_freesync_hdmi = true;	/* FR-001: advertise FreeSync VSIF */
	dw_plat_data->output_port = 1;
	/* Let the bridge fall back to YUV420 for modes whose 4:4:4 PHY clock would
	 * exceed this SoC's HDMI-TX limit (e.g. S805X 4K). 0 (no limit) elsewhere. */
	dw_plat_data->max_hdmi_phy_freq =
		priv->limits ? priv->limits->max_hdmi_phy_freq : 0;
	/* DC-001: meson VENC emits limited-range YUV -- prefer YUV444 sink for
	 * deep color (no CSC) and use the limited-range YUV->RGB matrix when an
	 * RGB sink is chosen. See dw_hdmi_plat_data.yuv_deep_color_first. */
	dw_plat_data->yuv_deep_color_first = true;

	if (dw_hdmi_is_compatible(meson_dw_hdmi, "amlogic,meson-gxl-dw-hdmi") ||
	    dw_hdmi_is_compatible(meson_dw_hdmi, "amlogic,meson-gxm-dw-hdmi") ||
	    dw_hdmi_is_compatible(meson_dw_hdmi, "amlogic,meson-g12a-dw-hdmi"))
		dw_plat_data->use_drm_infoframe = true;

	platform_set_drvdata(pdev, meson_dw_hdmi);

	meson_dw_hdmi->hdmi = dw_hdmi_probe(pdev, &meson_dw_hdmi->dw_plat_data);
	if (IS_ERR(meson_dw_hdmi->hdmi))
		return PTR_ERR(meson_dw_hdmi->hdmi);

	meson_dw_hdmi->bridge = of_drm_find_bridge(pdev->dev.of_node);

	DRM_DEBUG_DRIVER("HDMI controller initialized\n");

	return 0;
}

static void meson_dw_hdmi_unbind(struct device *dev, struct device *master,
				   void *data)
{
	struct meson_dw_hdmi *meson_dw_hdmi = dev_get_drvdata(dev);

	dw_hdmi_unbind(meson_dw_hdmi->hdmi);
}

static const struct component_ops meson_dw_hdmi_ops = {
	.bind	= meson_dw_hdmi_bind,
	.unbind	= meson_dw_hdmi_unbind,
};

static int __maybe_unused meson_dw_hdmi_pm_suspend(struct device *dev)
{
	struct meson_dw_hdmi *meson_dw_hdmi = dev_get_drvdata(dev);

	if (!meson_dw_hdmi)
		return 0;

	/* Reset TOP */
	meson_dw_hdmi->data->top_write(meson_dw_hdmi,
				       HDMITX_TOP_SW_RESET, 0);

	/*
	 * D4: serialize against the HPD/rxsense threaded IRQ, which also
	 * calls dw_hdmi_poweroff()/poweron() (via dw_hdmi_update_power())
	 * under hdmi->mutex and sleeps ~100ms inside dw_hdmi_setup().
	 */
	dw_hdmi_lock(meson_dw_hdmi->hdmi);
	if (dw_hdmi_is_bridge_on(meson_dw_hdmi->hdmi))
		dw_hdmi_poweroff(meson_dw_hdmi->hdmi);
	dw_hdmi_unlock(meson_dw_hdmi->hdmi);

	return 0;
}

static int __maybe_unused meson_dw_hdmi_pm_resume(struct device *dev)
{
	struct meson_dw_hdmi *meson_dw_hdmi = dev_get_drvdata(dev);

	if (!meson_dw_hdmi)
		return 0;

	meson_dw_hdmi_init(meson_dw_hdmi);

	/*
	 * D12: dw_hdmi_resume() (-> dw_hdmi_init_hw()) must run BEFORE
	 * poweron -- it masks every interrupt and resets DDC I2CM. Running
	 * it after poweron (the previous order here) re-masked interrupts
	 * and reset I2CM while the PHY was already up and FC packets armed.
	 */
	dw_hdmi_resume(meson_dw_hdmi->hdmi);

	dw_hdmi_lock(meson_dw_hdmi->hdmi);
	if (dw_hdmi_is_bridge_on(meson_dw_hdmi->hdmi))
		dw_hdmi_poweron(meson_dw_hdmi->hdmi);
	dw_hdmi_unlock(meson_dw_hdmi->hdmi);

	return 0;
}

static int meson_dw_hdmi_probe(struct platform_device *pdev)
{
	return component_add(&pdev->dev, &meson_dw_hdmi_ops);
}

static void meson_dw_hdmi_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &meson_dw_hdmi_ops);
}

static const struct dev_pm_ops meson_dw_hdmi_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(meson_dw_hdmi_pm_suspend,
				meson_dw_hdmi_pm_resume)
};

static const struct of_device_id meson_dw_hdmi_of_table[] = {
	{ .compatible = "amlogic,meson-gxbb-dw-hdmi",
	  .data = &meson_dw_hdmi_gxbb_data },
	{ .compatible = "amlogic,meson-gxl-dw-hdmi",
	  .data = &meson_dw_hdmi_gxl_data },
	{ .compatible = "amlogic,meson-gxm-dw-hdmi",
	  .data = &meson_dw_hdmi_gxl_data },
	{ .compatible = "amlogic,meson-g12a-dw-hdmi",
	  .data = &meson_dw_hdmi_g12a_data },
	{ }
};
MODULE_DEVICE_TABLE(of, meson_dw_hdmi_of_table);

static struct platform_driver meson_dw_hdmi_platform_driver = {
	.probe		= meson_dw_hdmi_probe,
	.remove		= meson_dw_hdmi_remove,
	.driver		= {
		.name		= DRIVER_NAME,
		.of_match_table	= meson_dw_hdmi_of_table,
		.pm = &meson_dw_hdmi_pm_ops,
	},
};
module_platform_driver(meson_dw_hdmi_platform_driver);

MODULE_AUTHOR("Neil Armstrong <narmstrong@baylibre.com>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
