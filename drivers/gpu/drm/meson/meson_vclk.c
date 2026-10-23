// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2016 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 * Copyright (C) 2015 Amlogic, Inc. All rights reserved.
 */

#include <linux/export.h>
#include <linux/math64.h>

#include <drm/drm_print.h>

#include "meson_drv.h"
#include "meson_vclk.h"

/**
 * DOC: Video Clocks
 *
 * VCLK is the "Pixel Clock" frequency generator from a dedicated PLL.
 * We handle the following encodings :
 *
 * - CVBS 27MHz generator via the VCLK2 to the VENCI and VDAC blocks
 * - HDMI Pixel Clocks generation
 *
 * What is missing :
 *
 * - Genenate Pixel clocks for 2K/4K 10bit formats
 *
 * Clock generator scheme :
 *
 * .. code::
 *
 *    __________   _________            _____
 *   |          | |         |          |     |--ENCI
 *   | HDMI PLL |-| PLL_DIV |--- VCLK--|     |--ENCL
 *   |__________| |_________| \        | MUX |--ENCP
 *                             --VCLK2-|     |--VDAC
 *                                     |_____|--HDMI-TX
 *
 * Final clocks can take input for either VCLK or VCLK2, but
 * VCLK is the preferred path for HDMI clocking and VCLK2 is the
 * preferred path for CVBS VDAC clocking.
 *
 * VCLK and VCLK2 have fixed divided clocks paths for /1, /2, /4, /6 or /12.
 *
 * The PLL_DIV can achieve an additional fractional dividing like
 * 1.5, 3.5, 3.75... to generate special 2K and 4K 10bit clocks.
 */

/* HHI Registers */
#define HHI_VID_PLL_CLK_DIV	0x1a0 /* 0x68 offset in data sheet */
#define VID_PLL_EN		BIT(19)
#define VID_PLL_BYPASS		BIT(18)
#define VID_PLL_PRESET		BIT(15)
#define HHI_VIID_CLK_DIV	0x128 /* 0x4a offset in data sheet */
#define VCLK2_DIV_MASK		0xff
#define VCLK2_DIV_EN		BIT(16)
#define VCLK2_DIV_RESET		BIT(17)
#define CTS_VDAC_SEL_MASK	(0xf << 28)
#define CTS_VDAC_SEL_SHIFT	28
#define HHI_VIID_CLK_CNTL	0x12c /* 0x4b offset in data sheet */
#define VCLK2_EN		BIT(19)
#define VCLK2_SEL_MASK		(0x7 << 16)
#define VCLK2_SEL_SHIFT		16
#define VCLK2_SOFT_RESET	BIT(15)
#define VCLK2_DIV1_EN		BIT(0)
#define HHI_VID_CLK_DIV		0x164 /* 0x59 offset in data sheet */
#define VCLK_DIV_MASK		0xff
#define VCLK_DIV_EN		BIT(16)
#define VCLK_DIV_RESET		BIT(17)
#define CTS_ENCP_SEL_MASK	(0xf << 24)
#define CTS_ENCP_SEL_SHIFT	24
#define CTS_ENCI_SEL_MASK	(0xf << 28)
#define CTS_ENCI_SEL_SHIFT	28
#define HHI_VID_CLK_CNTL	0x17c /* 0x5f offset in data sheet */
#define VCLK_EN			BIT(19)
#define VCLK_SEL_MASK		(0x7 << 16)
#define VCLK_SEL_SHIFT		16
#define VCLK_SOFT_RESET		BIT(15)
#define VCLK_DIV1_EN		BIT(0)
#define VCLK_DIV2_EN		BIT(1)
#define VCLK_DIV4_EN		BIT(2)
#define VCLK_DIV6_EN		BIT(3)
#define VCLK_DIV12_EN		BIT(4)
#define HHI_VID_CLK_CNTL2	0x194 /* 0x65 offset in data sheet */
#define CTS_ENCI_EN		BIT(0)
#define CTS_ENCP_EN		BIT(2)
#define CTS_VDAC_EN		BIT(4)
#define HDMI_TX_PIXEL_EN	BIT(5)
#define HHI_HDMI_CLK_CNTL	0x1cc /* 0x73 offset in data sheet */
#define HDMI_TX_PIXEL_SEL_MASK	(0xf << 16)
#define HDMI_TX_PIXEL_SEL_SHIFT	16
#define CTS_HDMI_SYS_SEL_MASK	(0x7 << 9)
#define CTS_HDMI_SYS_DIV_MASK	(0x7f)
#define CTS_HDMI_SYS_EN		BIT(8)

#define HHI_VDAC_CNTL0		0x2F4 /* 0xbd offset in data sheet */
#define HHI_VDAC_CNTL1		0x2F8 /* 0xbe offset in data sheet */

#define HHI_HDMI_PLL_CNTL	0x320 /* 0xc8 offset in data sheet */
#define HHI_HDMI_PLL_CNTL_EN	BIT(30)
#define HHI_HDMI_PLL_CNTL2	0x324 /* 0xc9 offset in data sheet */
#define HHI_HDMI_PLL_CNTL3	0x328 /* 0xca offset in data sheet */
#define HHI_HDMI_PLL_CNTL4	0x32C /* 0xcb offset in data sheet */
#define HHI_HDMI_PLL_CNTL5	0x330 /* 0xcc offset in data sheet */
#define HHI_HDMI_PLL_CNTL6	0x334 /* 0xcd offset in data sheet */
#define HHI_HDMI_PLL_CNTL7	0x338 /* 0xce offset in data sheet */

#define HDMI_PLL_RESET		BIT(28)
#define HDMI_PLL_RESET_G12A	BIT(29)
#define HDMI_PLL_LOCK		BIT(31)
#define HDMI_PLL_LOCK_G12A	(3 << 30)

/*
 * Bound the HDMI PLL lock poll. The lock bit normally asserts within a few
 * microseconds, so the poll exits immediately on a healthy lock; the timeout
 * only matters when the PLL cannot lock (an HDMI sink flapping HPD mid-modeset,
 * or a mode near the SoC PLL ceiling -- e.g. the documented S805X > 1.65 GHz
 * limit). Upstream passes timeout_us=0 here, which is an INFINITE poll: a PLL
 * that never locks then wedges the atomic-modeset thread forever. Bound it and
 * warn instead, so a bad sink cannot hang the modeset -- the next modeset
 * re-attempts. 10 ms is >100x the microsecond-scale lock time. The G12A path
 * resets+repolls in a loop, so it is bounded by a retry cap of equivalent
 * total duration (100 tries x 100 us poll).
 */
#define HDMI_PLL_LOCK_TIMEOUT_US	10000
#define HDMI_PLL_LOCK_G12A_MAX_TRIES	100

#define FREQ_1000_1001(_freq)	DIV_ROUND_CLOSEST_ULL((_freq) * 1000ULL, 1001ULL)

/* VID PLL Dividers */
enum {
	VID_PLL_DIV_1 = 0,
	VID_PLL_DIV_2,
	VID_PLL_DIV_2p5,
	VID_PLL_DIV_3,
	VID_PLL_DIV_3p5,
	VID_PLL_DIV_3p75,
	VID_PLL_DIV_4,
	VID_PLL_DIV_5,
	VID_PLL_DIV_6,
	VID_PLL_DIV_6p25,
	VID_PLL_DIV_7,
	VID_PLL_DIV_7p5,
	VID_PLL_DIV_12,
	VID_PLL_DIV_14,
	VID_PLL_DIV_15,
};

static void meson_vid_pll_set(struct meson_drm *priv, unsigned int div)
{
	unsigned int shift_val = 0;
	unsigned int shift_sel = 0;

	/* Disable vid_pll output clock */
	regmap_update_bits(priv->hhi, HHI_VID_PLL_CLK_DIV, VID_PLL_EN, 0);
	regmap_update_bits(priv->hhi, HHI_VID_PLL_CLK_DIV, VID_PLL_PRESET, 0);

	switch (div) {
	case VID_PLL_DIV_2:
		shift_val = 0x0aaa;
		shift_sel = 0;
		break;
	case VID_PLL_DIV_2p5:
		shift_val = 0x5294;
		shift_sel = 2;
		break;
	case VID_PLL_DIV_3:
		shift_val = 0x0db6;
		shift_sel = 0;
		break;
	case VID_PLL_DIV_3p5:
		shift_val = 0x36cc;
		shift_sel = 1;
		break;
	case VID_PLL_DIV_3p75:
		shift_val = 0x6666;
		shift_sel = 2;
		break;
	case VID_PLL_DIV_4:
		shift_val = 0x0ccc;
		shift_sel = 0;
		break;
	case VID_PLL_DIV_5:
		shift_val = 0x739c;
		shift_sel = 2;
		break;
	case VID_PLL_DIV_6:
		shift_val = 0x0e38;
		shift_sel = 0;
		break;
	case VID_PLL_DIV_6p25:
		shift_val = 0x0000;
		shift_sel = 3;
		break;
	case VID_PLL_DIV_7:
		shift_val = 0x3c78;
		shift_sel = 1;
		break;
	case VID_PLL_DIV_7p5:
		shift_val = 0x78f0;
		shift_sel = 2;
		break;
	case VID_PLL_DIV_12:
		shift_val = 0x0fc0;
		shift_sel = 0;
		break;
	case VID_PLL_DIV_14:
		shift_val = 0x3f80;
		shift_sel = 1;
		break;
	case VID_PLL_DIV_15:
		shift_val = 0x7f80;
		shift_sel = 2;
		break;
	}

	if (div == VID_PLL_DIV_1)
		/* Enable vid_pll bypass to HDMI pll */
		regmap_update_bits(priv->hhi, HHI_VID_PLL_CLK_DIV,
				   VID_PLL_BYPASS, VID_PLL_BYPASS);
	else {
		/* Disable Bypass */
		regmap_update_bits(priv->hhi, HHI_VID_PLL_CLK_DIV,
				   VID_PLL_BYPASS, 0);
		/* Clear sel */
		regmap_update_bits(priv->hhi, HHI_VID_PLL_CLK_DIV,
				   3 << 16, 0);
		regmap_update_bits(priv->hhi, HHI_VID_PLL_CLK_DIV,
				   VID_PLL_PRESET, 0);
		regmap_update_bits(priv->hhi, HHI_VID_PLL_CLK_DIV,
				   0x7fff, 0);

		/* Setup sel and val */
		regmap_update_bits(priv->hhi, HHI_VID_PLL_CLK_DIV,
				   3 << 16, shift_sel << 16);
		regmap_update_bits(priv->hhi, HHI_VID_PLL_CLK_DIV,
				   VID_PLL_PRESET, VID_PLL_PRESET);
		regmap_update_bits(priv->hhi, HHI_VID_PLL_CLK_DIV,
				   0x7fff, shift_val);

		regmap_update_bits(priv->hhi, HHI_VID_PLL_CLK_DIV,
				   VID_PLL_PRESET, 0);
	}

	/* Enable the vid_pll output clock */
	regmap_update_bits(priv->hhi, HHI_VID_PLL_CLK_DIV,
				VID_PLL_EN, VID_PLL_EN);
}

/*
 * Setup VCLK2 for 27MHz, and enable clocks for ENCI and VDAC
 *
 * TOFIX: Refactor into table to also handle HDMI frequency and paths
 */
static void meson_venci_cvbs_clock_config(struct meson_drm *priv)
{
	unsigned int val;

	/* Setup PLL to output 1.485GHz */
	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXBB)) {
		regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL, 0x5800023d);
		regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL2, 0x00404e00);
		regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL3, 0x0d5c5091);
		regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL4, 0x801da72c);
		regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL5, 0x71486980);
		regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL6, 0x00000e55);
		regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL, 0x4800023d);

		/* Poll for lock bit (bounded -- never hang on a PLL that
		 * cannot lock; see HDMI_PLL_LOCK_TIMEOUT_US above) */
		if (regmap_read_poll_timeout(priv->hhi, HHI_HDMI_PLL_CNTL, val,
					     (val & HDMI_PLL_LOCK), 10,
					     HDMI_PLL_LOCK_TIMEOUT_US))
			pr_warn("meson-vclk: CVBS PLL (GXBB) failed to lock\n");
	} else if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXM) ||
		   meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXL)) {
		regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL, 0x4000027b);
		regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL2, 0x800cb300);
		regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL3, 0xa6212844);
		regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL4, 0x0c4d000c);
		regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL5, 0x001fa729);
		regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL6, 0x01a31500);

		/* Reset PLL */
		regmap_update_bits(priv->hhi, HHI_HDMI_PLL_CNTL,
					HDMI_PLL_RESET, HDMI_PLL_RESET);
		regmap_update_bits(priv->hhi, HHI_HDMI_PLL_CNTL,
					HDMI_PLL_RESET, 0);

		/* Poll for lock bit (bounded -- never hang on a PLL that
		 * cannot lock; see HDMI_PLL_LOCK_TIMEOUT_US above) */
		if (regmap_read_poll_timeout(priv->hhi, HHI_HDMI_PLL_CNTL, val,
					     (val & HDMI_PLL_LOCK), 10,
					     HDMI_PLL_LOCK_TIMEOUT_US))
			pr_warn("meson-vclk: CVBS PLL (GXL/GXM) failed to lock\n");
	} else if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A)) {
		regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL, 0x1a0504f7);
		regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL2, 0x00010000);
		regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL3, 0x00000000);
		regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL4, 0x6a28dc00);
		regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL5, 0x65771290);
		regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL6, 0x39272000);
		regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL7, 0x56540000);
		regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL, 0x3a0504f7);
		regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL, 0x1a0504f7);

		/* Poll for lock bit (bounded -- never hang on a PLL that
		 * cannot lock; see HDMI_PLL_LOCK_TIMEOUT_US above) */
		if (regmap_read_poll_timeout(priv->hhi, HHI_HDMI_PLL_CNTL, val,
			((val & HDMI_PLL_LOCK_G12A) == HDMI_PLL_LOCK_G12A),
			10, HDMI_PLL_LOCK_TIMEOUT_US))
			pr_warn("meson-vclk: CVBS PLL (G12A) failed to lock\n");
	}

	/* Disable VCLK2 */
	regmap_update_bits(priv->hhi, HHI_VIID_CLK_CNTL, VCLK2_EN, 0);

	/* Setup vid_pll to /1 */
	meson_vid_pll_set(priv, VID_PLL_DIV_1);

	/* Setup the VCLK2 divider value to achieve 27MHz */
	regmap_update_bits(priv->hhi, HHI_VIID_CLK_DIV,
				VCLK2_DIV_MASK, (55 - 1));

	/* select vid_pll for vclk2 */
	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A))
		regmap_update_bits(priv->hhi, HHI_VIID_CLK_CNTL,
					VCLK2_SEL_MASK, (0 << VCLK2_SEL_SHIFT));
	else
		regmap_update_bits(priv->hhi, HHI_VIID_CLK_CNTL,
					VCLK2_SEL_MASK, (4 << VCLK2_SEL_SHIFT));

	/* enable vclk2 gate */
	regmap_update_bits(priv->hhi, HHI_VIID_CLK_CNTL, VCLK2_EN, VCLK2_EN);

	/* select vclk_div1 for enci */
	regmap_update_bits(priv->hhi, HHI_VID_CLK_DIV,
				CTS_ENCI_SEL_MASK, (8 << CTS_ENCI_SEL_SHIFT));
	/* select vclk_div1 for vdac */
	regmap_update_bits(priv->hhi, HHI_VIID_CLK_DIV,
				CTS_VDAC_SEL_MASK, (8 << CTS_VDAC_SEL_SHIFT));

	/* release vclk2_div_reset and enable vclk2_div */
	regmap_update_bits(priv->hhi, HHI_VIID_CLK_DIV,
				VCLK2_DIV_EN | VCLK2_DIV_RESET, VCLK2_DIV_EN);

	/* enable vclk2_div1 gate */
	regmap_update_bits(priv->hhi, HHI_VIID_CLK_CNTL,
				VCLK2_DIV1_EN, VCLK2_DIV1_EN);

	/* reset vclk2 */
	regmap_update_bits(priv->hhi, HHI_VIID_CLK_CNTL,
				VCLK2_SOFT_RESET, VCLK2_SOFT_RESET);
	regmap_update_bits(priv->hhi, HHI_VIID_CLK_CNTL,
				VCLK2_SOFT_RESET, 0);

	/* enable enci_clk */
	regmap_update_bits(priv->hhi, HHI_VID_CLK_CNTL2,
				CTS_ENCI_EN, CTS_ENCI_EN);
	/* enable vdac_clk */
	regmap_update_bits(priv->hhi, HHI_VID_CLK_CNTL2,
				CTS_VDAC_EN, CTS_VDAC_EN);
}

enum {
/* PLL	O1 O2 O3 VP DV     EN TX */
/* 4320 /4 /4 /1 /5 /1  => /2 /2 */
	MESON_VCLK_HDMI_ENCI_54000 = 0,
/* 4320 /4 /4 /1 /5 /1  => /1 /2 */
	MESON_VCLK_HDMI_DDR_54000,
/* 2970 /4 /1 /1 /5 /1  => /1 /2 */
	MESON_VCLK_HDMI_DDR_148500,
/* 2970 /2 /2 /2 /5 /1  => /1 /1 */
	MESON_VCLK_HDMI_74250,
/* 2970 /1 /2 /2 /5 /1  => /1 /1 */
	MESON_VCLK_HDMI_148500,
/* 2970 /1 /1 /1 /5 /2  => /1 /1 */
	MESON_VCLK_HDMI_297000,
/* 5940 /1 /1 /2 /5 /1  => /1 /1 */
	MESON_VCLK_HDMI_594000,
/* 2970 /1 /1 /1 /5 /1  => /1 /2 */
	MESON_VCLK_HDMI_594000_YUV420,
/* 4830 /2 /1 /2 /5 /1  => /1 /1 */
	MESON_VCLK_HDMI_241500,
/* 5940 /2 /2 /1 /5 /1 -- 4K30 YUV420: phy 1485 (TMDS halved), vclk 297 */
	MESON_VCLK_HDMI_297000_YUV420,
};

struct meson_vclk_params {
	unsigned long long pll_freq;
	unsigned long long phy_freq;
	unsigned long long vclk_freq;
	unsigned long long venc_freq;
	unsigned long long pixel_freq;
	unsigned int pll_od1;
	unsigned int pll_od2;
	unsigned int pll_od3;
	unsigned int vid_pll_div;
	unsigned int vclk_div;
} params[] = {
	[MESON_VCLK_HDMI_ENCI_54000] = {
		.pll_freq = 4320000000,
		.phy_freq = 270000000,
		.vclk_freq = 54000000,
		.venc_freq = 54000000,
		.pixel_freq = 54000000,
		.pll_od1 = 4,
		.pll_od2 = 4,
		.pll_od3 = 1,
		.vid_pll_div = VID_PLL_DIV_5,
		.vclk_div = 1,
	},
	[MESON_VCLK_HDMI_DDR_54000] = {
		.pll_freq = 4320000000,
		.phy_freq = 270000000,
		.vclk_freq = 54000000,
		.venc_freq = 54000000,
		.pixel_freq = 27000000,
		.pll_od1 = 4,
		.pll_od2 = 4,
		.pll_od3 = 1,
		.vid_pll_div = VID_PLL_DIV_5,
		.vclk_div = 1,
	},
	[MESON_VCLK_HDMI_DDR_148500] = {
		.pll_freq = 2970000000,
		.phy_freq = 742500000,
		.vclk_freq = 148500000,
		.venc_freq = 148500000,
		.pixel_freq = 74250000,
		.pll_od1 = 4,
		.pll_od2 = 1,
		.pll_od3 = 1,
		.vid_pll_div = VID_PLL_DIV_5,
		.vclk_div = 1,
	},
	[MESON_VCLK_HDMI_74250] = {
		.pll_freq = 2970000000,
		.phy_freq = 742500000,
		.vclk_freq = 74250000,
		.venc_freq = 74250000,
		.pixel_freq = 74250000,
		.pll_od1 = 2,
		.pll_od2 = 2,
		.pll_od3 = 2,
		.vid_pll_div = VID_PLL_DIV_5,
		.vclk_div = 1,
	},
	[MESON_VCLK_HDMI_148500] = {
		.pll_freq = 2970000000,
		.phy_freq = 1485000000,
		.vclk_freq = 148500000,
		.venc_freq = 148500000,
		.pixel_freq = 148500000,
		.pll_od1 = 1,
		.pll_od2 = 2,
		.pll_od3 = 2,
		.vid_pll_div = VID_PLL_DIV_5,
		.vclk_div = 1,
	},
	[MESON_VCLK_HDMI_297000] = {
		.pll_freq = 5940000000,
		.phy_freq = 2970000000,
		.venc_freq = 297000000,
		.vclk_freq = 297000000,
		.pixel_freq = 297000000,
		.pll_od1 = 2,
		.pll_od2 = 1,
		.pll_od3 = 1,
		.vid_pll_div = VID_PLL_DIV_5,
		.vclk_div = 2,
	},
	[MESON_VCLK_HDMI_594000] = {
		.pll_freq = 5940000000,
		.phy_freq = 5940000000,
		.venc_freq = 594000000,
		.vclk_freq = 594000000,
		.pixel_freq = 594000000,
		.pll_od1 = 1,
		.pll_od2 = 1,
		.pll_od3 = 2,
		.vid_pll_div = VID_PLL_DIV_5,
		.vclk_div = 1,
	},
	[MESON_VCLK_HDMI_594000_YUV420] = {
		.pll_freq = 5940000000,
		.phy_freq = 2970000000,
		.venc_freq = 594000000,
		.vclk_freq = 594000000,
		.pixel_freq = 297000000,
		.pll_od1 = 2,
		.pll_od2 = 1,
		.pll_od3 = 1,
		.vid_pll_div = VID_PLL_DIV_5,
		.vclk_div = 1,
	},
	/* E12(a): every field here was 1000x too small (uniform missing-000
	 * typo) -- this table entry could never match any real PLL/PHY/VENC
	 * frequency, so it was fully dead. Verified self-consistent against
	 * every sibling entry's derivation (phy_freq = pll_freq/(od1*od2);
	 * venc_freq = pll_freq/(od1*od2*od3*vid_pll_div); vclk_freq =
	 * pixel_freq = venc_freq for this od/div combo). */
	[MESON_VCLK_HDMI_241500] = {
		.pll_freq = 4830000000,
		.phy_freq = 2415000000,
		.venc_freq = 241500000,
		.vclk_freq = 241500000,
		.pixel_freq = 241500000,
		.pll_od1 = 2,
		.pll_od2 = 1,
		.pll_od3 = 2,
		.vid_pll_div = VID_PLL_DIV_5,
		.vclk_div = 1,
	},
	[MESON_VCLK_HDMI_297000_YUV420] = {
		.pll_freq = 5940000000,
		.phy_freq = 1485000000,
		.venc_freq = 297000000,
		.vclk_freq = 297000000,
		.pixel_freq = 148500000,
		.pll_od1 = 2,
		.pll_od2 = 2,
		.pll_od3 = 1,
		.vid_pll_div = VID_PLL_DIV_5,
		.vclk_div = 1,
	},
	{ /* sentinel */ },
};

static inline unsigned int pll_od_to_reg(unsigned int od)
{
	switch (od) {
	case 1:
		return 0;
	case 2:
		return 1;
	case 4:
		return 2;
	case 8:
		return 3;
	}

	/* Invalid */
	return 0;
}

/* E9: returns false if the PLL never locked, so the caller can abort the
 * clock-tree programming instead of writing the OD dividers (and every
 * downstream vid_pll/vclk/hdmi_tx/venc divider) off a PLL that isn't
 * actually running at the requested frequency. */
static bool meson_hdmi_pll_set_params(struct meson_drm *priv, unsigned int m,
				      unsigned int frac, unsigned int od1,
				      unsigned int od2, unsigned int od3)
{
	unsigned int val;
	bool locked = true;

	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXBB)) {
		regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL, 0x58000200 | m);
		if (frac)
			regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL2,
				     0x00004000 | frac);
		else
			regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL2,
				     0x00000000);
		regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL3, 0x0d5c5091);
		regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL4, 0x801da72c);
		regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL5, 0x71486980);
		regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL6, 0x00000e55);

		/* Enable and unreset */
		regmap_update_bits(priv->hhi, HHI_HDMI_PLL_CNTL,
				   0x7 << 28, HHI_HDMI_PLL_CNTL_EN);

		/* Poll for lock bit (bounded -- never hang on a PLL that
		 * cannot lock; see HDMI_PLL_LOCK_TIMEOUT_US above) */
		if (regmap_read_poll_timeout(priv->hhi, HHI_HDMI_PLL_CNTL,
					     val, (val & HDMI_PLL_LOCK), 10,
					     HDMI_PLL_LOCK_TIMEOUT_US)) {
			pr_warn("meson-vclk: HDMI PLL (GXBB) failed to lock\n");
			locked = false;
		}
	} else if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXM) ||
		   meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXL)) {
		regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL, 0x40000200 | m);
		regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL2, 0x800cb000 | frac);
		regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL3, 0x860f30c4);
		regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL4, 0x0c8e0000);
		regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL5, 0x001fa729);
		regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL6, 0x01a31500);

		/* Reset PLL */
		regmap_update_bits(priv->hhi, HHI_HDMI_PLL_CNTL,
				HDMI_PLL_RESET, HDMI_PLL_RESET);
		regmap_update_bits(priv->hhi, HHI_HDMI_PLL_CNTL,
				HDMI_PLL_RESET, 0);

		/* Poll for lock bit (bounded -- never hang on a PLL that
		 * cannot lock; see HDMI_PLL_LOCK_TIMEOUT_US above) */
		if (regmap_read_poll_timeout(priv->hhi, HHI_HDMI_PLL_CNTL, val,
				(val & HDMI_PLL_LOCK), 10,
				HDMI_PLL_LOCK_TIMEOUT_US)) {
			pr_warn("meson-vclk: HDMI PLL (GXL/GXM) failed to lock\n");
			locked = false;
		}
	} else if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A)) {
		unsigned int tries = 0;

		regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL, 0x0b3a0400 | m);

		/* Enable and reset */
		/* TODO: add specific macro for g12a here */
		regmap_update_bits(priv->hhi, HHI_HDMI_PLL_CNTL,
				   0x3 << 28, 0x3 << 28);

		regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL2, frac);
		regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL3, 0x00000000);

		/* G12A HDMI PLL Needs specific parameters for 5.4GHz */
		if (m >= 0xf7) {
			if (frac < 0x10000) {
				regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL4,
							0x6a685c00);
				regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL5,
							0x11551293);
			} else {
				regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL4,
							0xea68dc00);
				regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL5,
							0x65771290);
			}
			regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL6, 0x39272000);
			regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL7, 0x55540000);
		} else {
			regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL4, 0x0a691c00);
			regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL5, 0x33771290);
			regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL6, 0x39270000);
			regmap_write(priv->hhi, HHI_HDMI_PLL_CNTL7, 0x50540000);
		}

		do {
			/* Reset PLL */
			regmap_update_bits(priv->hhi, HHI_HDMI_PLL_CNTL,
					HDMI_PLL_RESET_G12A, HDMI_PLL_RESET_G12A);

			/* UN-Reset PLL */
			regmap_update_bits(priv->hhi, HHI_HDMI_PLL_CNTL,
					HDMI_PLL_RESET_G12A, 0);

			/* Poll for lock bits */
			if (!regmap_read_poll_timeout(priv->hhi,
						      HHI_HDMI_PLL_CNTL, val,
						      ((val & HDMI_PLL_LOCK_G12A)
						        == HDMI_PLL_LOCK_G12A),
						      10, 100))
				break;
		} while (++tries < HDMI_PLL_LOCK_G12A_MAX_TRIES);
		/* Bounded -- never hang on a PLL that cannot lock */
		if (tries >= HDMI_PLL_LOCK_G12A_MAX_TRIES) {
			pr_warn("meson-vclk: HDMI PLL (G12A) failed to lock\n");
			locked = false;
		}
	}

	/*
	 * E9: don't program the OD dividers below off a PLL that never
	 * locked -- leaves the clock tree half-programmed instead of
	 * failing visibly.
	 */
	if (!locked)
		return false;

	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXBB))
		regmap_update_bits(priv->hhi, HHI_HDMI_PLL_CNTL2,
				3 << 16, pll_od_to_reg(od1) << 16);
	else if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXM) ||
		 meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXL))
		regmap_update_bits(priv->hhi, HHI_HDMI_PLL_CNTL3,
				3 << 21, pll_od_to_reg(od1) << 21);
	else if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A))
		regmap_update_bits(priv->hhi, HHI_HDMI_PLL_CNTL,
				3 << 16, pll_od_to_reg(od1) << 16);

	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXBB))
		regmap_update_bits(priv->hhi, HHI_HDMI_PLL_CNTL2,
				3 << 22, pll_od_to_reg(od2) << 22);
	else if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXM) ||
		 meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXL))
		regmap_update_bits(priv->hhi, HHI_HDMI_PLL_CNTL3,
				3 << 23, pll_od_to_reg(od2) << 23);
	else if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A))
		regmap_update_bits(priv->hhi, HHI_HDMI_PLL_CNTL,
				3 << 18, pll_od_to_reg(od2) << 18);

	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXBB))
		regmap_update_bits(priv->hhi, HHI_HDMI_PLL_CNTL2,
				3 << 18, pll_od_to_reg(od3) << 18);
	else if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXM) ||
		 meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXL))
		regmap_update_bits(priv->hhi, HHI_HDMI_PLL_CNTL3,
				3 << 19, pll_od_to_reg(od3) << 19);
	else if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A))
		regmap_update_bits(priv->hhi, HHI_HDMI_PLL_CNTL,
				3 << 20, pll_od_to_reg(od3) << 20);

	return true;
}

#define XTAL_FREQ (24 * 1000 * 1000)

static unsigned int meson_hdmi_pll_get_m(struct meson_drm *priv,
					 unsigned long long pll_freq)
{
	/* The GXBB PLL has a /2 pre-multiplier */
	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXBB))
		pll_freq = DIV_ROUND_DOWN_ULL(pll_freq, 2);

	return DIV_ROUND_DOWN_ULL(pll_freq, XTAL_FREQ);
}

#define HDMI_FRAC_MAX_GXBB	4096
#define HDMI_FRAC_MAX_GXL	1024
#define HDMI_FRAC_MAX_G12A	131072

static unsigned int meson_hdmi_pll_get_frac(struct meson_drm *priv,
					    unsigned int m,
					    unsigned long long pll_freq)
{
	unsigned long long parent_freq = XTAL_FREQ;
	unsigned int frac_max = HDMI_FRAC_MAX_GXL;
	unsigned int frac_m;
	unsigned int frac;
	u32 remainder;

	/* The GXBB PLL has a /2 pre-multiplier and a larger FRAC width */
	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXBB)) {
		frac_max = HDMI_FRAC_MAX_GXBB;
		parent_freq *= 2;
	}

	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A))
		frac_max = HDMI_FRAC_MAX_G12A;

	/* We can have a perfect match !*/
	if (div_u64_rem(pll_freq, m, &remainder) == parent_freq &&
	    remainder == 0)
		return 0;

	frac = mul_u64_u64_div_u64(pll_freq, frac_max, parent_freq);
	frac_m = m * frac_max;
	if (frac_m > frac)
		return frac_max;
	frac -= frac_m;

	return min((u16)frac, (u16)(frac_max - 1));
}

static bool meson_hdmi_pll_validate_params(struct meson_drm *priv,
					   unsigned long long m,
					   unsigned int frac)
{
	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXBB)) {
		/* Empiric supported min/max dividers */
		if (m < 53 || m > 123)
			return false;
		if (frac >= HDMI_FRAC_MAX_GXBB)
			return false;
	} else if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXM) ||
		   meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXL)) {
		/* Empiric supported min/max dividers */
		if (m < 106 || m > 247)
			return false;
		if (frac >= HDMI_FRAC_MAX_GXL)
			return false;
	} else if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A)) {
		/* Empiric supported min/max dividers */
		if (m < 106 || m > 247)
			return false;
		if (frac >= HDMI_FRAC_MAX_G12A)
			return false;
	}

	return true;
}

static bool meson_hdmi_pll_find_params(struct meson_drm *priv,
				       unsigned long long freq,
				       unsigned int *m,
				       unsigned int *frac,
				       unsigned int *od)
{
	/* Cycle from /16 to /1. od=1 (VCO fed straight to the PHY) is only ever
	 * selected for phy in (2976, 5952] MHz -- where od=2 would need a VCO above
	 * the m-range max -- i.e. char rates ~298..595 MHz. Every existing mode
	 * (phy <= 2976) still resolves to od >= 2, unchanged. This is what lets the
	 * PLL synthesize the 3197.5 MHz that 3440x1440@60 needs.
	 *
	 * NB the ceiling is 5951.977 MHz, not 247*24 = 5928: m is only the INTEGER
	 * term, and get_frac() adds up to 1023/1024 of a step on GXL, so the real
	 * max VCO is 24MHz * (247 + 1023/1024). An earlier version of this comment
	 * used the integer-only 5928 figure, which made 5940 MHz (4K60 4:4:4) look
	 * like a 0.2% overshoot when it is in fact synthesizable -- m=247, frac=512,
	 * od=1, the same pair meson_vclk_set() hard-codes for case 5940000000. */
	for (*od = 16 ; *od >= 1 ; *od >>= 1) {
		*m = meson_hdmi_pll_get_m(priv, freq * *od);
		if (!*m)
			continue;
		*frac = meson_hdmi_pll_get_frac(priv, *m, freq * *od);

		DRM_DEBUG_DRIVER("PLL params for %lluHz: m=%x frac=%x od=%d\n",
				 freq, *m, *frac, *od);

		if (meson_hdmi_pll_validate_params(priv, *m, *frac))
			return true;
	}

	return false;
}

/* pll_freq is the frequency after the OD dividers */
enum drm_mode_status
meson_vclk_dmt_supported_freq(struct meson_drm *priv, unsigned long long freq)
{
	unsigned int od, m, frac;

	/* In DMT mode, path after PLL is always /10 */
	freq *= 10;

	/* Check against soc revision/package limits */
	if (priv->limits) {
		if (priv->limits->max_hdmi_phy_freq &&
		    freq > priv->limits->max_hdmi_phy_freq)
			return MODE_CLOCK_HIGH;
	}

	if (meson_hdmi_pll_find_params(priv, freq, &m, &frac, &od))
		return MODE_OK;

	return MODE_CLOCK_RANGE;
}
EXPORT_SYMBOL_GPL(meson_vclk_dmt_supported_freq);

/* pll_freq is the frequency after the OD dividers */
static bool meson_hdmi_pll_generic_set(struct meson_drm *priv,
				       unsigned long long pll_freq)
{
	unsigned int od, m, frac, od1, od2, od3;

	if (meson_hdmi_pll_find_params(priv, pll_freq, &m, &frac, &od)) {
		/* OD2 goes to the PHY, and needs to be *10, so keep OD3=1 */
		od3 = 1;
		if (od < 2) {
			/* od=1: VCO fed straight through (no post-divider). */
			od1 = 1;
			od2 = 1;
		} else if (od < 4) {
			od1 = 2;
			od2 = 1;
		} else {
			od2 = od / 4;
			od1 = od / od2;
		}

		DRM_DEBUG_DRIVER("PLL params for %lluHz: m=%x frac=%x od=%d/%d/%d\n",
				 pll_freq, m, frac, od1, od2, od3);

		return meson_hdmi_pll_set_params(priv, m, frac, od1, od2, od3);
	}

	DRM_ERROR("Fatal, unable to find parameters for PLL freq %lluHz\n",
		  pll_freq);
	return false;
}

static bool meson_vclk_freqs_are_matching_param(unsigned int idx,
						unsigned long long phy_freq,
						unsigned long long vclk_freq)
{
	DRM_DEBUG_DRIVER("i = %d vclk_freq = %lluHz alt = %lluHz\n",
			 idx, params[idx].vclk_freq,
			 FREQ_1000_1001(params[idx].vclk_freq));
	DRM_DEBUG_DRIVER("i = %d phy_freq = %lluHz alt = %lluHz\n",
			 idx, params[idx].phy_freq,
			 FREQ_1000_1001(params[idx].phy_freq));

	/* Match strict frequency */
	if (phy_freq == params[idx].phy_freq &&
	    vclk_freq == params[idx].vclk_freq)
		return true;

	/* Match 1000/1001 variant: vclk deviation has to be less than 1kHz
	 * (drm EDID is defined in 1kHz steps, so everything smaller must be
	 * rounding error) and the PHY freq deviation has to be less than
	 * 10kHz (as the TMDS clock is 10 times the pixel clock, so anything
	 * smaller must be rounding error as well).
	 */
	if (abs(vclk_freq - FREQ_1000_1001(params[idx].vclk_freq)) < 1000 &&
	    abs(phy_freq - FREQ_1000_1001(params[idx].phy_freq)) < 10000)
		return true;

	/* no match */
	return false;
}

/*
 * E6: is phy_freq (the HDMI-TX PHY serial bit rate, Hz) actually
 * synthesizable by the HDMI PLL's generic computed-PLL path (the od/m/frac
 * search in meson_hdmi_pll_find_params())? This is the SAME search
 * meson_hdmi_pll_generic_set() runs when atomic_enable() programs the clock
 * tree -- exposing it as its own entry point lets a caller validate a
 * deep-color (10-bit) phy_freq at CHECK time, before any hardware is
 * touched, instead of discovering a PLL-lock failure deep inside
 * atomic_enable() (a void function, called only after VENC and the HDMI-TX
 * sys clock bits are already programmed -- see meson_encoder_hdmi.c).
 *
 * Deliberately does NOT check priv->limits->max_hdmi_phy_freq (the SoC
 * package limit) -- that is a separate, already-covered gate (this function
 * only answers "can the PLL lock to this frequency", not "is this frequency
 * allowed on this SoC").
 */
bool meson_vclk_pll_freq_synthesizable(struct meson_drm *priv,
				       unsigned long long phy_freq)
{
	unsigned int od, m, frac;

	return meson_hdmi_pll_find_params(priv, phy_freq, &m, &frac, &od);
}
EXPORT_SYMBOL_GPL(meson_vclk_pll_freq_synthesizable);

enum drm_mode_status
meson_vclk_vic_supported_freq(struct meson_drm *priv,
			      unsigned long long phy_freq,
			      unsigned long long vclk_freq)
{
	int i;

	DRM_DEBUG_DRIVER("phy_freq = %lluHz vclk_freq = %lluHz\n",
			 phy_freq, vclk_freq);

	/* Check against soc revision/package limits */
	if (priv->limits) {
		if (priv->limits->max_hdmi_phy_freq &&
		    phy_freq > priv->limits->max_hdmi_phy_freq)
			return MODE_CLOCK_HIGH;
	}

	for (i = 0 ; params[i].pixel_freq ; ++i) {
		if (meson_vclk_freqs_are_matching_param(i, phy_freq, vclk_freq))
			return MODE_OK;
	}

	/*
	 * DC-001 (deep color): a 10-bit (DC_30bit) request runs the PHY at
	 * pixel_clock * 12.5 (phy_freq == vclk_freq * 25 / 2), which is not in
	 * params[] (all 8-bit). meson_vclk_setup synthesizes it via the generic
	 * computed-PLL path, so accept it here as long as the PLL can lock to it
	 * (and it is under the SoC phy limit, already checked above). 8-bit
	 * requests keep phy_freq == vclk_freq * 10 and never match this.
	 */
	if (phy_freq == DIV_ROUND_DOWN_ULL(vclk_freq * 25ULL, 2ULL)) {
		if (meson_vclk_pll_freq_synthesizable(priv, phy_freq))
			return MODE_OK;
	}

	return MODE_CLOCK_RANGE;
}
EXPORT_SYMBOL_GPL(meson_vclk_vic_supported_freq);

static void meson_vclk_set(struct meson_drm *priv,
			   unsigned long long pll_base_freq, unsigned int od1,
			   unsigned int od2, unsigned int od3,
			   unsigned int vid_pll_div, unsigned int vclk_div,
			   unsigned int hdmi_tx_div, unsigned int venc_div,
			   bool hdmi_use_enci, bool vic_alternate_clock)
{
	unsigned int m = 0, frac = 0;

	/* Set HDMI-TX sys clock */
	regmap_update_bits(priv->hhi, HHI_HDMI_CLK_CNTL,
			   CTS_HDMI_SYS_SEL_MASK, 0);
	regmap_update_bits(priv->hhi, HHI_HDMI_CLK_CNTL,
			   CTS_HDMI_SYS_DIV_MASK, 0);
	regmap_update_bits(priv->hhi, HHI_HDMI_CLK_CNTL,
			   CTS_HDMI_SYS_EN, CTS_HDMI_SYS_EN);

	/* Set HDMI PLL rate */
	if (!od1 && !od2 && !od3) {
		if (!meson_hdmi_pll_generic_set(priv, pll_base_freq)) {
			DRM_ERROR("meson-vclk: aborting clock-tree programming: PLL setup failed for %lluHz\n",
				  pll_base_freq);
			return;
		}
	} else if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXBB)) {
		switch (pll_base_freq) {
		case 2970000000:
			m = 0x3d;
			frac = vic_alternate_clock ? 0xd02 : 0xe00;
			break;
		case 4320000000:
			m = vic_alternate_clock ? 0x59 : 0x5a;
			frac = vic_alternate_clock ? 0xe8f : 0;
			break;
		case 5940000000:
			m = 0x7b;
			frac = vic_alternate_clock ? 0xa05 : 0xc00;
			break;
		default:
			DRM_ERROR("meson-vclk: no PLL params for GXBB pll_base_freq %lluHz -- aborting clock-tree programming\n",
				  pll_base_freq);
			return;
		}

		if (!meson_hdmi_pll_set_params(priv, m, frac, od1, od2, od3)) {
			DRM_ERROR("meson-vclk: aborting clock-tree programming: HDMI PLL (GXBB) failed to lock\n");
			return;
		}
	} else if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXM) ||
		   meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXL)) {
		switch (pll_base_freq) {
		case 2970000000:
			m = 0x7b;
			frac = vic_alternate_clock ? 0x281 : 0x300;
			break;
		case 4320000000:
			m = vic_alternate_clock ? 0xb3 : 0xb4;
			frac = vic_alternate_clock ? 0x347 : 0;
			break;
		case 5940000000:
			m = 0xf7;
			frac = vic_alternate_clock ? 0x102 : 0x200;
			break;
		default:
			DRM_ERROR("meson-vclk: no PLL params for GXM/GXL pll_base_freq %lluHz -- aborting clock-tree programming\n",
				  pll_base_freq);
			return;
		}

		if (!meson_hdmi_pll_set_params(priv, m, frac, od1, od2, od3)) {
			DRM_ERROR("meson-vclk: aborting clock-tree programming: HDMI PLL (GXM/GXL) failed to lock\n");
			return;
		}
	} else if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A)) {
		switch (pll_base_freq) {
		case 2970000000:
			m = 0x7b;
			frac = vic_alternate_clock ? 0x140b4 : 0x18000;
			break;
		case 4320000000:
			m = vic_alternate_clock ? 0xb3 : 0xb4;
			frac = vic_alternate_clock ? 0x1a3ee : 0;
			break;
		case 5940000000:
			m = 0xf7;
			frac = vic_alternate_clock ? 0x8148 : 0x10000;
			break;
		case 4830000000:		/* E12: was 4830000 (1000x too small) */
			m = 0xc9;
			frac = 0xd560;
			break;
		default:
			DRM_ERROR("meson-vclk: no PLL params for G12A pll_base_freq %lluHz -- aborting clock-tree programming\n",
				  pll_base_freq);
			return;
		}

		if (!meson_hdmi_pll_set_params(priv, m, frac, od1, od2, od3)) {
			DRM_ERROR("meson-vclk: aborting clock-tree programming: HDMI PLL (G12A) failed to lock\n");
			return;
		}
	}

	/* Setup vid_pll divider */
	meson_vid_pll_set(priv, vid_pll_div);

	/* Set VCLK div */
	regmap_update_bits(priv->hhi, HHI_VID_CLK_CNTL,
			   VCLK_SEL_MASK, 0);
	/* DC-4K30-XD diag: see meson_vclk_setup(); reports the divisor actually
	 * written, so a wrong XD can be tied to the caller above. */
	dev_info(priv->dev,
		 "vclk_set: pll=%lluHz vid_pll_div=%u vclk_div=%u (XD=%u) hdmi_tx_div=%u venc_div=%u\n",
		 pll_base_freq, vid_pll_div, vclk_div, vclk_div - 1,
		 hdmi_tx_div, venc_div);
	regmap_update_bits(priv->hhi, HHI_VID_CLK_DIV,
			   VCLK_DIV_MASK, vclk_div - 1);

	/* Set HDMI-TX source */
	switch (hdmi_tx_div) {
	case 1:
		/* enable vclk_div1 gate */
		regmap_update_bits(priv->hhi, HHI_VID_CLK_CNTL,
				   VCLK_DIV1_EN, VCLK_DIV1_EN);

		/* select vclk_div1 for HDMI-TX */
		regmap_update_bits(priv->hhi, HHI_HDMI_CLK_CNTL,
				   HDMI_TX_PIXEL_SEL_MASK, 0);
		break;
	case 2:
		/* enable vclk_div2 gate */
		regmap_update_bits(priv->hhi, HHI_VID_CLK_CNTL,
				   VCLK_DIV2_EN, VCLK_DIV2_EN);

		/* select vclk_div2 for HDMI-TX */
		regmap_update_bits(priv->hhi, HHI_HDMI_CLK_CNTL,
			HDMI_TX_PIXEL_SEL_MASK, 1 << HDMI_TX_PIXEL_SEL_SHIFT);
		break;
	case 4:
		/* enable vclk_div4 gate */
		regmap_update_bits(priv->hhi, HHI_VID_CLK_CNTL,
				   VCLK_DIV4_EN, VCLK_DIV4_EN);

		/* select vclk_div4 for HDMI-TX */
		regmap_update_bits(priv->hhi, HHI_HDMI_CLK_CNTL,
			HDMI_TX_PIXEL_SEL_MASK, 2 << HDMI_TX_PIXEL_SEL_SHIFT);
		break;
	case 6:
		/* enable vclk_div6 gate */
		regmap_update_bits(priv->hhi, HHI_VID_CLK_CNTL,
				   VCLK_DIV6_EN, VCLK_DIV6_EN);

		/* select vclk_div6 for HDMI-TX */
		regmap_update_bits(priv->hhi, HHI_HDMI_CLK_CNTL,
			HDMI_TX_PIXEL_SEL_MASK, 3 << HDMI_TX_PIXEL_SEL_SHIFT);
		break;
	case 12:
		/* enable vclk_div12 gate */
		regmap_update_bits(priv->hhi, HHI_VID_CLK_CNTL,
				   VCLK_DIV12_EN, VCLK_DIV12_EN);

		/* select vclk_div12 for HDMI-TX */
		regmap_update_bits(priv->hhi, HHI_HDMI_CLK_CNTL,
			HDMI_TX_PIXEL_SEL_MASK, 4 << HDMI_TX_PIXEL_SEL_SHIFT);
		break;
	}
	regmap_update_bits(priv->hhi, HHI_VID_CLK_CNTL2,
				   HDMI_TX_PIXEL_EN, HDMI_TX_PIXEL_EN);

	/* Set ENCI/ENCP Source */
	switch (venc_div) {
	case 1:
		/* enable vclk_div1 gate */
		regmap_update_bits(priv->hhi, HHI_VID_CLK_CNTL,
				   VCLK_DIV1_EN, VCLK_DIV1_EN);

		if (hdmi_use_enci)
			/* select vclk_div1 for enci */
			regmap_update_bits(priv->hhi, HHI_VID_CLK_DIV,
					   CTS_ENCI_SEL_MASK, 0);
		else
			/* select vclk_div1 for encp */
			regmap_update_bits(priv->hhi, HHI_VID_CLK_DIV,
					   CTS_ENCP_SEL_MASK, 0);
		break;
	case 2:
		/* enable vclk_div2 gate */
		regmap_update_bits(priv->hhi, HHI_VID_CLK_CNTL,
				   VCLK_DIV2_EN, VCLK_DIV2_EN);

		if (hdmi_use_enci)
			/* select vclk_div2 for enci */
			regmap_update_bits(priv->hhi, HHI_VID_CLK_DIV,
				CTS_ENCI_SEL_MASK, 1 << CTS_ENCI_SEL_SHIFT);
		else
			/* select vclk_div2 for encp */
			regmap_update_bits(priv->hhi, HHI_VID_CLK_DIV,
				CTS_ENCP_SEL_MASK, 1 << CTS_ENCP_SEL_SHIFT);
		break;
	case 4:
		/* enable vclk_div4 gate */
		regmap_update_bits(priv->hhi, HHI_VID_CLK_CNTL,
				   VCLK_DIV4_EN, VCLK_DIV4_EN);

		if (hdmi_use_enci)
			/* select vclk_div4 for enci */
			regmap_update_bits(priv->hhi, HHI_VID_CLK_DIV,
				CTS_ENCI_SEL_MASK, 2 << CTS_ENCI_SEL_SHIFT);
		else
			/* select vclk_div4 for encp */
			regmap_update_bits(priv->hhi, HHI_VID_CLK_DIV,
				CTS_ENCP_SEL_MASK, 2 << CTS_ENCP_SEL_SHIFT);
		break;
	case 6:
		/* enable vclk_div6 gate */
		regmap_update_bits(priv->hhi, HHI_VID_CLK_CNTL,
				   VCLK_DIV6_EN, VCLK_DIV6_EN);

		if (hdmi_use_enci)
			/* select vclk_div6 for enci */
			regmap_update_bits(priv->hhi, HHI_VID_CLK_DIV,
				CTS_ENCI_SEL_MASK, 3 << CTS_ENCI_SEL_SHIFT);
		else
			/* select vclk_div6 for encp */
			regmap_update_bits(priv->hhi, HHI_VID_CLK_DIV,
				CTS_ENCP_SEL_MASK, 3 << CTS_ENCP_SEL_SHIFT);
		break;
	case 12:
		/* enable vclk_div12 gate */
		regmap_update_bits(priv->hhi, HHI_VID_CLK_CNTL,
				   VCLK_DIV12_EN, VCLK_DIV12_EN);

		if (hdmi_use_enci)
			/* select vclk_div12 for enci */
			regmap_update_bits(priv->hhi, HHI_VID_CLK_DIV,
				CTS_ENCI_SEL_MASK, 4 << CTS_ENCI_SEL_SHIFT);
		else
			/* select vclk_div12 for encp */
			regmap_update_bits(priv->hhi, HHI_VID_CLK_DIV,
				CTS_ENCP_SEL_MASK, 4 << CTS_ENCP_SEL_SHIFT);
		break;
	}

	if (hdmi_use_enci)
		/* Enable ENCI clock gate */
		regmap_update_bits(priv->hhi, HHI_VID_CLK_CNTL2,
				   CTS_ENCI_EN, CTS_ENCI_EN);
	else
		/* Enable ENCP clock gate */
		regmap_update_bits(priv->hhi, HHI_VID_CLK_CNTL2,
				   CTS_ENCP_EN, CTS_ENCP_EN);

	regmap_update_bits(priv->hhi, HHI_VID_CLK_CNTL, VCLK_EN, VCLK_EN);
}

void meson_vclk_setup(struct meson_drm *priv, unsigned int target,
		      unsigned long long phy_freq, unsigned long long vclk_freq,
		      unsigned long long venc_freq, unsigned long long dac_freq,
		      bool hdmi_use_enci)
{
	bool vic_alternate_clock = false;
	unsigned long long freq;
	unsigned long long hdmi_tx_div;
	unsigned long long venc_div;

	/*
	 * DC-4K30-XD diag: 4K30 444-10 was measured programming the XD divider
	 * to /5 (cts_encp 118.8MHz instead of 297MHz), while every code path
	 * that can reach that PLL state computes /2. Name the caller and its
	 * arguments so the /5 can be attributed in one boot.
	 */
	dev_info(priv->dev,
		 "vclk_setup: target=%u phy=%lluHz vclk=%lluHz venc=%lluHz dac=%lluHz enci=%d\n",
		 target, phy_freq, vclk_freq, venc_freq, dac_freq,
		 hdmi_use_enci);

	if (target == MESON_VCLK_TARGET_CVBS) {
		meson_venci_cvbs_clock_config(priv);
		return;
	} else if (target == MESON_VCLK_TARGET_DMT) {
		/*
		 * The DMT clock path is fixed after the PLL:
		 * - automatic PLL freq + OD management
		 * - vid_pll_div = VID_PLL_DIV_5 (8-bit) / VID_PLL_DIV_6p25 (10-bit)
		 * - vclk_div = 2
		 * - hdmi_tx_div = 1
		 * - venc_div = 1
		 * - encp encoder
		 *
		 * DC-001 (deep color): a 10-bit (DC_30bit) request runs the PHY at
		 * pixel_clock * 12.5 (phy_freq == vclk_freq * 25 / 2) instead of the
		 * 8-bit * 10. The encoder/vclk must stay at the pixel rate, so the
		 * post-PLL chain has to divide phy_freq by 12.5 (vid_pll_div 6.25 *
		 * vclk_div 2), not 10. The HDMI target's DC-001 branch below does the
		 * same; a CEA mode reaches deep color through that path, but a non-CEA
		 * mode (drm_match_cea_mode() == 0, e.g. a 1080p/4K EDID detailed
		 * timing) reaches it through here. Without the 6.25 divider the encoder
		 * is clocked 1.25x too fast (185.25 instead of 148.5 MHz for 1080p),
		 * so every 10-bit pixel word is emitted at the wrong phase relative to
		 * the TMDS character clock -- the output scrambles (green-dominant).
		 * Confirmed on demo-aml-s905d3-cc-alpha: cts_encp read 185.25 MHz at
		 * 10-bit, 148.2 MHz at 8-bit, via /sys/kernel/debug/clk/clk_summary.
		 */
		unsigned int dmt_vid_pll_div = VID_PLL_DIV_5;

		if (phy_freq == DIV_ROUND_DOWN_ULL(vclk_freq * 25ULL, 2ULL))
			dmt_vid_pll_div = VID_PLL_DIV_6p25;

		meson_vclk_set(priv, phy_freq, 0, 0, 0,
			       dmt_vid_pll_div, 2, 1, 1, false, false);
		return;
	}

	hdmi_tx_div = DIV_ROUND_DOWN_ULL(vclk_freq, dac_freq);

	if (hdmi_tx_div == 0) {
		pr_err("Fatal Error, invalid HDMI-TX freq %lluHz\n",
		       dac_freq);
		return;
	}

	venc_div = DIV_ROUND_DOWN_ULL(vclk_freq, venc_freq);

	if (venc_div == 0) {
		pr_err("Fatal Error, invalid HDMI venc freq %lluHz\n",
		       venc_freq);
		return;
	}

	/*
	 * DC-001 (deep color): 10-bit (DC_30bit) output runs the PHY at
	 * pixel_clock * 12.5 (== vclk_freq * 25 / 2) instead of the 8-bit * 10.
	 * That phy_freq is not in params[] (the table is all 8-bit), so route it
	 * through the generic computed-PLL path, exactly as the DMT target does.
	 * Dividers (verified against the vendor BSP hdmitx20 hw_clk.c colordepth
	 * 30B table, e.g. 1080p60 10-bit row "3712500,1,2,2,VID_PLL_DIV_6p25,1,
	 * 1,1"): the HPLL is computed by meson_hdmi_pll_generic_set so its output
	 * after the OD chain == phy_freq; then vid_pll_div = 6.25 and vclk_div
	 * divide phy_freq back down to vclk_freq (1856.25 MHz / 6.25 / 2 =
	 * 148.5 MHz for 1080p). hdmi_tx_div / venc_div stay at the values derived
	 * from the caller's vclk/dac/venc. 8-bit output never reaches this branch,
	 * so the existing table path is byte-for-byte unchanged.
	 */
	/*
	 * DC-001-REPEAT: test against dac_freq (the HDMI-TX / pixel rate), NOT
	 * vclk_freq. For a venc-repeat mode (480p/576p/720p/1080i, see
	 * meson_venc_hdmi_venc_repeat) the caller doubles vclk_freq to clock the
	 * VENC, while phy_freq stays derived from the PIXEL rate -- so
	 * "phy == vclk * 12.5" can never hold and 10-bit silently fell through to
	 * the 8-bit params[] table. dw-hdmi had already been told to emit 10-bit,
	 * so the link ran an 8-bit clock into a 10-bit packetizer and never locked
	 * (measured on GXL 720p60: phy programmed 1485MHz = the doubled venc rate
	 * x10, instead of 74.25MHz x12.5 = 928.125MHz; capture black at 3.3dB).
	 * dac_freq stays at the pixel rate in both cases, and equals vclk_freq for
	 * every non-repeat mode, so this is a no-op there.
	 */
	if (phy_freq == DIV_ROUND_DOWN_ULL(dac_freq * 25ULL, 2ULL)) {
		/*
		 * The post-PLL chain must divide phy_freq down to vclk_freq, and
		 * vclk_freq is the VENC rate -- doubled on a repeat mode. With
		 * vid_pll_div fixed at 6.25 that makes vclk_div = phy/(6.25*vclk):
		 * 2 for a non-repeat mode (phy = pixel*12.5, vclk = pixel) and 1 for
		 * a repeat mode (phy = pixel*12.5, vclk = pixel*2). Hardcoding 2
		 * clocked the VENC at half rate on repeat modes.
		 */
		/*
		 * DC-4K30-XD: do NOT use DIV_ROUND_CLOSEST_ULL() here. It is
		 * built on do_div(), whose divisor is u32, so a 64-bit divisor
		 * is silently TRUNCATED to its low 32 bits. vclk_freq * 25
		 * exceeds 2^32 for any pixel rate above ~171.8 MHz, so every
		 * deep-color mode at 4K-class rates computed a wildly wrong
		 * divider while 1080p-class rates (148.5 MHz -> 3.7125e9, still
		 * under 2^32) happened to be correct -- which is exactly why
		 * 1080p60 444-10 worked and 4K30 444-10 did not.
		 *
		 * Measured on GXL S905X before this fix: 4K30 444-10 wanted
		 * vclk_div 2 but programmed 5 (divisor 7.425e9 wrapped to
		 * 3.130e9), so cts_encp ran at 118.8 MHz instead of 297 MHz and
		 * the link never locked. 4K60 420-10 wanted 1 and got 11
		 * (divisor 14.85e9 wrapped to 1.965e9).
		 *
		 * div64_u64() takes a full 64-bit divisor; round-to-nearest is
		 * open-coded since there is no _ULL round-closest variant that
		 * is 64-bit-divisor safe.
		 */
		unsigned long long vclk_num = phy_freq * 4ULL;
		unsigned long long vclk_den = vclk_freq * 25ULL;
		unsigned long long vclk_div =
			div64_u64(vclk_num + vclk_den / 2, vclk_den);

		if (!vclk_div)
			vclk_div = 1;

		DRM_DEBUG_DRIVER("deep-color 10bit: phy=%lluHz vclk=%lluHz dac=%lluHz vclk_div=%llu hdmi_tx_div=%llu venc_div=%llu\n",
				 phy_freq, vclk_freq, dac_freq, vclk_div,
				 hdmi_tx_div, venc_div);
		meson_vclk_set(priv, phy_freq, 0, 0, 0,
			       VID_PLL_DIV_6p25, vclk_div, hdmi_tx_div, venc_div,
			       hdmi_use_enci, false);
		return;
	}

	for (freq = 0 ; params[freq].pixel_freq ; ++freq) {
		if (meson_vclk_freqs_are_matching_param(freq, phy_freq,
							vclk_freq)) {
			if (vclk_freq != params[freq].vclk_freq)
				vic_alternate_clock = true;
			else
				vic_alternate_clock = false;

			if (freq == MESON_VCLK_HDMI_ENCI_54000 &&
			    !hdmi_use_enci)
				continue;

			if (freq == MESON_VCLK_HDMI_DDR_54000 &&
			    hdmi_use_enci)
				continue;

			if (freq == MESON_VCLK_HDMI_DDR_148500 &&
			    dac_freq == vclk_freq)
				continue;

			if (freq == MESON_VCLK_HDMI_148500 &&
			    dac_freq != vclk_freq)
				continue;
			break;
		}
	}

	if (!params[freq].pixel_freq) {
		pr_err("Fatal Error, invalid HDMI vclk freq %lluHz\n",
		       vclk_freq);
		return;
	}

	meson_vclk_set(priv, params[freq].pll_freq,
		       params[freq].pll_od1, params[freq].pll_od2,
		       params[freq].pll_od3, params[freq].vid_pll_div,
		       params[freq].vclk_div, hdmi_tx_div, venc_div,
		       hdmi_use_enci, vic_alternate_clock);
}
EXPORT_SYMBOL_GPL(meson_vclk_setup);
