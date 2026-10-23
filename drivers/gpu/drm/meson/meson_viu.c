// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2016 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 * Copyright (C) 2015 Amlogic, Inc. All rights reserved.
 * Copyright (C) 2014 Endless Mobile
 */

#include <linux/export.h>
#include <linux/bitfield.h>
#include <linux/iopoll.h>

#include <drm/drm_color_mgmt.h>
#include <drm/drm_fourcc.h>

#include "meson_drv.h"
#include "meson_viu.h"
#include "meson_registers.h"

/**
 * DOC: Video Input Unit
 *
 * VIU Handles the Pixel scanout and the basic Colorspace conversions
 * We handle the following features :
 *
 * - OSD1 RGB565/RGB888/xRGB8888 scanout
 * - RGB conversion to x/cb/cr
 * - Progressive or Interlace buffer scanout
 * - OSD1 Commit on Vsync
 * - HDR OSD matrix for GXL/GXM
 *
 * What is missing :
 *
 * - BGR888/xBGR8888/BGRx8888/BGRx8888 modes
 * - YUV4:2:2 Y0CbY1Cr scanout
 * - Conversion to YUV 4:4:4 from 4:2:2 input
 * - Colorkey Alpha matching
 * - Big endian scanout
 * - X/Y reverse scanout
 * - Global alpha setup
 * - OSD2 support, would need interlace switching on vsync
 * - OSD1 full scaling to support TV overscan
 */

/* OSD csc defines */

enum viu_matrix_sel_e {
	VIU_MATRIX_OSD_EOTF = 0,
	VIU_MATRIX_OSD,
};

enum viu_lut_sel_e {
	VIU_LUT_OSD_EOTF = 0,
	VIU_LUT_OSD_OETF,
};

#define COEFF_NORM(a) ((int)((((a) * 2048.0) + 1) / 2))
#define MATRIX_5X3_COEF_SIZE 24

#define EOTF_COEFF_NORM(a) ((int)((((a) * 4096.0) + 1) / 2))
#define EOTF_COEFF_SIZE 10
#define EOTF_COEFF_RIGHTSHIFT 1

static int RGB709_to_YUV709l_coeff[MATRIX_5X3_COEF_SIZE] = {
	0, 0, 0, /* pre offset */
	COEFF_NORM(0.181873),	COEFF_NORM(0.611831),	COEFF_NORM(0.061765),
	COEFF_NORM(-0.100251),	COEFF_NORM(-0.337249),	COEFF_NORM(0.437500),
	COEFF_NORM(0.437500),	COEFF_NORM(-0.397384),	COEFF_NORM(-0.040116),
	0, 0, 0, /* 10'/11'/12' */
	0, 0, 0, /* 20'/21'/22' */
	64, 512, 512, /* offset */
	0, 0, 0 /* mode, right_shift, clip_en */
};

/*  eotf matrix: bypass */
static int eotf_bypass_coeff[EOTF_COEFF_SIZE] = {
	EOTF_COEFF_NORM(1.0),	EOTF_COEFF_NORM(0.0),	EOTF_COEFF_NORM(0.0),
	EOTF_COEFF_NORM(0.0),	EOTF_COEFF_NORM(1.0),	EOTF_COEFF_NORM(0.0),
	EOTF_COEFF_NORM(0.0),	EOTF_COEFF_NORM(0.0),	EOTF_COEFF_NORM(1.0),
	EOTF_COEFF_RIGHTSHIFT /* right shift */
};

static void meson_viu_set_g12a_osd1_matrix(struct meson_drm *priv,
					   int *m, bool csc_on)
{
	/* VPP WRAP OSD1 matrix */
	writel(((m[0] & 0xfff) << 16) | (m[1] & 0xfff),
		priv->io_base + _REG(VPP_WRAP_OSD1_MATRIX_PRE_OFFSET0_1));
	writel(m[2] & 0xfff,
		priv->io_base + _REG(VPP_WRAP_OSD1_MATRIX_PRE_OFFSET2));
	writel(((m[3] & 0x1fff) << 16) | (m[4] & 0x1fff),
		priv->io_base + _REG(VPP_WRAP_OSD1_MATRIX_COEF00_01));
	writel(((m[5] & 0x1fff) << 16) | (m[6] & 0x1fff),
		priv->io_base + _REG(VPP_WRAP_OSD1_MATRIX_COEF02_10));
	writel(((m[7] & 0x1fff) << 16) | (m[8] & 0x1fff),
		priv->io_base + _REG(VPP_WRAP_OSD1_MATRIX_COEF11_12));
	writel(((m[9] & 0x1fff) << 16) | (m[10] & 0x1fff),
		priv->io_base + _REG(VPP_WRAP_OSD1_MATRIX_COEF20_21));
	writel((m[11] & 0x1fff),
		priv->io_base +	_REG(VPP_WRAP_OSD1_MATRIX_COEF22));

	writel(((m[18] & 0xfff) << 16) | (m[19] & 0xfff),
		priv->io_base + _REG(VPP_WRAP_OSD1_MATRIX_OFFSET0_1));
	writel(m[20] & 0xfff,
		priv->io_base + _REG(VPP_WRAP_OSD1_MATRIX_OFFSET2));

	writel_bits_relaxed(BIT(0), csc_on ? BIT(0) : 0,
		priv->io_base + _REG(VPP_WRAP_OSD1_MATRIX_EN_CTRL));
}

/*
 * The G12A VPP WRAP has a SEPARATE per-OSD RGB->YUV output matrix for OSD2
 * (VPP_WRAP_OSD2_MATRIX_*, 0x3d70..0x3d7d), parallel to OSD1's (0x3d60..0x3d6d).
 * Upstream only ever programs OSD1's matrix, leaving OSD2's EN_CTRL=0 (CSC
 * bypassed): OSD2 then feeds RGB into the YUV-expecting downstream pipe and its
 * colours come out permuted (red->green->blue rotation, white->pink) -- the same
 * "green/pink distortion" class the OSD1 matrix fixes for OSD1. Program OSD2's
 * matrix identically so OSD2 renders correct colour for every advertised RGB
 * fourcc. (Plane-correctness P2: confirmed by HDMI loopback capture on SM1 --
 * OSD2 four-quadrant R/G/B/W was scrambled with EN_CTRL=0 and correct after this,
 * matching OSD1 pixel-for-pixel.) Layout is identical to OSD1, +0x10 per register.
 */
static void meson_viu_set_g12a_osd2_matrix(struct meson_drm *priv,
					   int *m, bool csc_on)
{
	/* VPP WRAP OSD2 matrix */
	writel(((m[0] & 0xfff) << 16) | (m[1] & 0xfff),
		priv->io_base + _REG(VPP_WRAP_OSD2_MATRIX_PRE_OFFSET0_1));
	writel(m[2] & 0xfff,
		priv->io_base + _REG(VPP_WRAP_OSD2_MATRIX_PRE_OFFSET2));
	writel(((m[3] & 0x1fff) << 16) | (m[4] & 0x1fff),
		priv->io_base + _REG(VPP_WRAP_OSD2_MATRIX_COEF00_01));
	writel(((m[5] & 0x1fff) << 16) | (m[6] & 0x1fff),
		priv->io_base + _REG(VPP_WRAP_OSD2_MATRIX_COEF02_10));
	writel(((m[7] & 0x1fff) << 16) | (m[8] & 0x1fff),
		priv->io_base + _REG(VPP_WRAP_OSD2_MATRIX_COEF11_12));
	writel(((m[9] & 0x1fff) << 16) | (m[10] & 0x1fff),
		priv->io_base + _REG(VPP_WRAP_OSD2_MATRIX_COEF20_21));
	writel((m[11] & 0x1fff),
		priv->io_base +	_REG(VPP_WRAP_OSD2_MATRIX_COEF22));

	writel(((m[18] & 0xfff) << 16) | (m[19] & 0xfff),
		priv->io_base + _REG(VPP_WRAP_OSD2_MATRIX_OFFSET0_1));
	writel(m[20] & 0xfff,
		priv->io_base + _REG(VPP_WRAP_OSD2_MATRIX_OFFSET2));

	writel_bits_relaxed(BIT(0), csc_on ? BIT(0) : 0,
		priv->io_base + _REG(VPP_WRAP_OSD2_MATRIX_EN_CTRL));
}

/*
 * GXL/GXM analog of meson_viu_set_g12a_osd2_matrix. The pre-G12A VIU has a
 * dedicated OSD2 RGB->YUV output matrix (VIU_OSD2_MATRIX_*, 0x1ab0..0x1ab9),
 * parallel to OSD1's (VIU_OSD1_MATRIX_*, 0x1a90..). meson_viu_load_matrix only
 * ever programmed OSD1's, leaving OSD2's CTRL=0 (CSC bypassed): OSD2 then feeds
 * RGB into the YUV-expecting VPP postblend and its colours come out permuted
 * (red->green->blue rotation, white->pink) -- the same green/pink class the OSD1
 * matrix fixes for OSD1, and the same bug the G12A path fixes via VPP_WRAP. The
 * VIU OSD2 matrix is a plain 3x3 (no HDR 5x3 extension), so only the
 * pre-offset (m[0..2]), the nine coefficients (m[3..11]) and the post-offset
 * (m[18..20]) are used; the enable is VIU_OSD2_MATRIX_CTRL BIT(0), same shape as
 * OSD1's VIU_OSD1_MATRIX_CTRL. (Plane-correctness: confirmed by HDMI loopback on
 * GXL/S805X -- OSD2 four-quadrant R/G/B/W was scrambled with CTRL=0 and correct
 * after this, matching OSD1.)
 */
static void meson_viu_set_viu_osd2_matrix(struct meson_drm *priv,
					  int *m, bool csc_on)
{
	writel(((m[0] & 0xfff) << 16) | (m[1] & 0xfff),
		priv->io_base + _REG(VIU_OSD2_MATRIX_PRE_OFFSET0_1));
	writel(m[2] & 0xfff,
		priv->io_base + _REG(VIU_OSD2_MATRIX_PRE_OFFSET2));
	writel(((m[3] & 0x1fff) << 16) | (m[4] & 0x1fff),
		priv->io_base + _REG(VIU_OSD2_MATRIX_COEF00_01));
	writel(((m[5] & 0x1fff) << 16) | (m[6] & 0x1fff),
		priv->io_base + _REG(VIU_OSD2_MATRIX_COEF02_10));
	writel(((m[7] & 0x1fff) << 16) | (m[8] & 0x1fff),
		priv->io_base + _REG(VIU_OSD2_MATRIX_COEF11_12));
	writel(((m[9] & 0x1fff) << 16) | (m[10] & 0x1fff),
		priv->io_base + _REG(VIU_OSD2_MATRIX_COEF20_21));
	writel((m[11] & 0x1fff),
		priv->io_base + _REG(VIU_OSD2_MATRIX_COEF22));

	writel(((m[18] & 0xfff) << 16) | (m[19] & 0xfff),
		priv->io_base + _REG(VIU_OSD2_MATRIX_OFFSET0_1));
	writel(m[20] & 0xfff,
		priv->io_base + _REG(VIU_OSD2_MATRIX_OFFSET2));

	/* CTRL: BIT(0)=matrix enable, BIT(1) cleared (mirrors OSD1 CTRL) */
	writel_bits_relaxed(BIT(0), csc_on ? BIT(0) : 0,
		priv->io_base + _REG(VIU_OSD2_MATRIX_CTRL));
	writel_bits_relaxed(BIT(1), 0,
		priv->io_base + _REG(VIU_OSD2_MATRIX_CTRL));
}

static void meson_viu_set_osd_matrix(struct meson_drm *priv,
				     enum viu_matrix_sel_e m_select,
			      int *m, bool csc_on)
{
	if (m_select == VIU_MATRIX_OSD) {
		/* osd matrix, VIU_MATRIX_0 */
		writel(((m[0] & 0xfff) << 16) | (m[1] & 0xfff),
			priv->io_base + _REG(VIU_OSD1_MATRIX_PRE_OFFSET0_1));
		writel(m[2] & 0xfff,
			priv->io_base + _REG(VIU_OSD1_MATRIX_PRE_OFFSET2));
		writel(((m[3] & 0x1fff) << 16) | (m[4] & 0x1fff),
			priv->io_base + _REG(VIU_OSD1_MATRIX_COEF00_01));
		writel(((m[5] & 0x1fff) << 16) | (m[6] & 0x1fff),
			priv->io_base + _REG(VIU_OSD1_MATRIX_COEF02_10));
		writel(((m[7] & 0x1fff) << 16) | (m[8] & 0x1fff),
			priv->io_base + _REG(VIU_OSD1_MATRIX_COEF11_12));
		writel(((m[9] & 0x1fff) << 16) | (m[10] & 0x1fff),
			priv->io_base + _REG(VIU_OSD1_MATRIX_COEF20_21));

		if (m[21]) {
			writel(((m[11] & 0x1fff) << 16) | (m[12] & 0x1fff),
				priv->io_base +
					_REG(VIU_OSD1_MATRIX_COEF22_30));
			writel(((m[13] & 0x1fff) << 16) | (m[14] & 0x1fff),
				priv->io_base +
					_REG(VIU_OSD1_MATRIX_COEF31_32));
			writel(((m[15] & 0x1fff) << 16) | (m[16] & 0x1fff),
				priv->io_base +
					_REG(VIU_OSD1_MATRIX_COEF40_41));
			writel(m[17] & 0x1fff, priv->io_base +
				_REG(VIU_OSD1_MATRIX_COLMOD_COEF42));
		} else
			writel((m[11] & 0x1fff) << 16, priv->io_base +
				_REG(VIU_OSD1_MATRIX_COEF22_30));

		writel(((m[18] & 0xfff) << 16) | (m[19] & 0xfff),
			priv->io_base + _REG(VIU_OSD1_MATRIX_OFFSET0_1));
		writel(m[20] & 0xfff,
			priv->io_base + _REG(VIU_OSD1_MATRIX_OFFSET2));

		writel_bits_relaxed(3 << 30, m[21] << 30,
			priv->io_base + _REG(VIU_OSD1_MATRIX_COLMOD_COEF42));
		writel_bits_relaxed(7 << 16, m[22] << 16,
			priv->io_base + _REG(VIU_OSD1_MATRIX_COLMOD_COEF42));

		/* 23 reserved for clipping control */
		writel_bits_relaxed(BIT(0), csc_on ? BIT(0) : 0,
			priv->io_base + _REG(VIU_OSD1_MATRIX_CTRL));
		writel_bits_relaxed(BIT(1), 0,
			priv->io_base + _REG(VIU_OSD1_MATRIX_CTRL));
	} else if (m_select == VIU_MATRIX_OSD_EOTF) {
		int i;

		/* osd eotf matrix, VIU_MATRIX_OSD_EOTF */
		for (i = 0; i < 5; i++)
			writel(((m[i * 2] & 0x1fff) << 16) |
				(m[i * 2 + 1] & 0x1fff), priv->io_base +
				_REG(VIU_OSD1_EOTF_CTL + i + 1));

		writel_bits_relaxed(BIT(30), csc_on ? BIT(30) : 0,
			priv->io_base + _REG(VIU_OSD1_EOTF_CTL));
		writel_bits_relaxed(BIT(31), csc_on ? BIT(31) : 0,
			priv->io_base + _REG(VIU_OSD1_EOTF_CTL));
	}
}

#define OSD_EOTF_LUT_SIZE 33
#define OSD_OETF_LUT_SIZE 41

static void
meson_viu_set_osd_lut(struct meson_drm *priv, enum viu_lut_sel_e lut_sel,
		      unsigned int *r_map, unsigned int *g_map,
		      unsigned int *b_map, bool csc_on)
{
	unsigned int addr_port;
	unsigned int data_port;
	unsigned int ctrl_port;
	int i;

	if (lut_sel == VIU_LUT_OSD_EOTF) {
		addr_port = VIU_OSD1_EOTF_LUT_ADDR_PORT;
		data_port = VIU_OSD1_EOTF_LUT_DATA_PORT;
		ctrl_port = VIU_OSD1_EOTF_CTL;
	} else if (lut_sel == VIU_LUT_OSD_OETF) {
		addr_port = VIU_OSD1_OETF_LUT_ADDR_PORT;
		data_port = VIU_OSD1_OETF_LUT_DATA_PORT;
		ctrl_port = VIU_OSD1_OETF_CTL;
	} else
		return;

	if (lut_sel == VIU_LUT_OSD_OETF) {
		writel(0, priv->io_base + _REG(addr_port));

		for (i = 0; i < (OSD_OETF_LUT_SIZE / 2); i++)
			writel(r_map[i * 2] | (r_map[i * 2 + 1] << 16),
				priv->io_base + _REG(data_port));

		writel(r_map[OSD_OETF_LUT_SIZE - 1] | (g_map[0] << 16),
			priv->io_base + _REG(data_port));

		for (i = 0; i < (OSD_OETF_LUT_SIZE / 2); i++)
			writel(g_map[i * 2 + 1] | (g_map[i * 2 + 2] << 16),
				priv->io_base + _REG(data_port));

		for (i = 0; i < (OSD_OETF_LUT_SIZE / 2); i++)
			writel(b_map[i * 2] | (b_map[i * 2 + 1] << 16),
				priv->io_base + _REG(data_port));

		writel(b_map[OSD_OETF_LUT_SIZE - 1],
			priv->io_base + _REG(data_port));

		if (csc_on)
			writel_bits_relaxed(0x7 << 29, 7 << 29,
					    priv->io_base + _REG(ctrl_port));
		else
			writel_bits_relaxed(0x7 << 29, 0,
					    priv->io_base + _REG(ctrl_port));
	} else if (lut_sel == VIU_LUT_OSD_EOTF) {
		writel(0, priv->io_base + _REG(addr_port));

		for (i = 0; i < (OSD_EOTF_LUT_SIZE / 2); i++)
			writel(r_map[i * 2] | (r_map[i * 2 + 1] << 16),
				priv->io_base + _REG(data_port));

		writel(r_map[OSD_EOTF_LUT_SIZE - 1] | (g_map[0] << 16),
			priv->io_base + _REG(data_port));

		for (i = 0; i < (OSD_EOTF_LUT_SIZE / 2); i++)
			writel(g_map[i * 2 + 1] | (g_map[i * 2 + 2] << 16),
				priv->io_base + _REG(data_port));

		for (i = 0; i < (OSD_EOTF_LUT_SIZE / 2); i++)
			writel(b_map[i * 2] | (b_map[i * 2 + 1] << 16),
				priv->io_base + _REG(data_port));

		writel(b_map[OSD_EOTF_LUT_SIZE - 1],
			priv->io_base + _REG(data_port));

		if (csc_on)
			writel_bits_relaxed(7 << 27, 7 << 27,
					    priv->io_base + _REG(ctrl_port));
		else
			writel_bits_relaxed(7 << 27, 0,
					    priv->io_base + _REG(ctrl_port));

		writel_bits_relaxed(BIT(31), BIT(31),
				    priv->io_base + _REG(ctrl_port));
	}
}

/* eotf lut: linear */
static unsigned int eotf_33_linear_mapping[OSD_EOTF_LUT_SIZE] = {
	0x0000,	0x0200,	0x0400, 0x0600,
	0x0800, 0x0a00, 0x0c00, 0x0e00,
	0x1000, 0x1200, 0x1400, 0x1600,
	0x1800, 0x1a00, 0x1c00, 0x1e00,
	0x2000, 0x2200, 0x2400, 0x2600,
	0x2800, 0x2a00, 0x2c00, 0x2e00,
	0x3000, 0x3200, 0x3400, 0x3600,
	0x3800, 0x3a00, 0x3c00, 0x3e00,
	0x4000
};

/* osd oetf lut: linear */
static unsigned int oetf_41_linear_mapping[OSD_OETF_LUT_SIZE] = {
	0, 0, 0, 0,
	0, 32, 64, 96,
	128, 160, 196, 224,
	256, 288, 320, 352,
	384, 416, 448, 480,
	512, 544, 576, 608,
	640, 672, 704, 736,
	768, 800, 832, 864,
	896, 928, 960, 992,
	1023, 1023, 1023, 1023,
	1023
};

static void meson_viu_load_matrix(struct meson_drm *priv)
{
	/* eotf lut bypass */
	meson_viu_set_osd_lut(priv, VIU_LUT_OSD_EOTF,
			      eotf_33_linear_mapping, /* R */
			      eotf_33_linear_mapping, /* G */
			      eotf_33_linear_mapping, /* B */
			      false);

	/* eotf matrix bypass */
	meson_viu_set_osd_matrix(priv, VIU_MATRIX_OSD_EOTF,
				 eotf_bypass_coeff,
				 false);

	/* oetf lut bypass */
	meson_viu_set_osd_lut(priv, VIU_LUT_OSD_OETF,
			      oetf_41_linear_mapping, /* R */
			      oetf_41_linear_mapping, /* G */
			      oetf_41_linear_mapping, /* B */
			      false);

	/* osd matrix RGB709 to YUV709 limit */
	meson_viu_set_osd_matrix(priv, VIU_MATRIX_OSD,
				 RGB709_to_YUV709l_coeff,
				 true);

	/*
	 * Audit #7: this function is also called (via meson_viu_osd1_reset())
	 * on every OSD1 disable->enable transition, and VIU_SW_RESET_OSD1 does
	 * not touch OSD2's matrix state. OSD2's VIU_OSD2_MATRIX_* used to be
	 * re-driven here too -- an unconditional, ungated rewrite of a register
	 * block that may belong to an actively scanning-out OSD2, triggered
	 * purely by OSD1's own commit. It only needs programming once, in
	 * meson_viu_init()'s GXL/GXM branch -- deliberately NOT repeated here.
	 */
}

/* VIU OSD1 Reset as workaround for GXL+ Alpha OSD Bug */
void meson_viu_osd1_reset(struct meson_drm *priv)
{
	uint32_t osd1_fifo_ctrl_stat, osd1_ctrl_stat2;

	/* Save these 2 registers state */
	osd1_fifo_ctrl_stat = readl_relaxed(
				priv->io_base + _REG(VIU_OSD1_FIFO_CTRL_STAT));
	osd1_ctrl_stat2 = readl_relaxed(
				priv->io_base + _REG(VIU_OSD1_CTRL_STAT2));

	/* Reset OSD1 */
	writel_bits_relaxed(VIU_SW_RESET_OSD1, VIU_SW_RESET_OSD1,
			    priv->io_base + _REG(VIU_SW_RESET));
	writel_bits_relaxed(VIU_SW_RESET_OSD1, 0,
			    priv->io_base + _REG(VIU_SW_RESET));

	/* Rewrite these registers state lost in the reset */
	writel_relaxed(osd1_fifo_ctrl_stat,
		       priv->io_base + _REG(VIU_OSD1_FIFO_CTRL_STAT));
	writel_relaxed(osd1_ctrl_stat2,
		       priv->io_base + _REG(VIU_OSD1_CTRL_STAT2));

	/* Reload the conversion matrix */
	meson_viu_load_matrix(priv);

	/*
	 * Audit #6: meson_viu_load_matrix() above unconditionally rewrites the
	 * EOTF matrix (VIU_MATRIX_OSD_EOTF) to bypass, with no restore -- any
	 * CTM the user had set via the DRM CTM property (same register block,
	 * see meson_viu_set_ctm()/meson_viu_apply_ctm()) was silently destroyed
	 * on every OSD1 re-enable. Re-apply the cached CTM (or bypass, if none
	 * is active) from the cache meson_crtc_atomic_flush() populates.
	 */
	meson_viu_apply_ctm(priv);
}

#define OSD1_MALI_ORDER_ABGR				\
	(FIELD_PREP(VIU_OSD1_MALI_AFBCD_A_REORDER,	\
		    VIU_OSD1_MALI_REORDER_A) |		\
	 FIELD_PREP(VIU_OSD1_MALI_AFBCD_B_REORDER,	\
		    VIU_OSD1_MALI_REORDER_B) |		\
	 FIELD_PREP(VIU_OSD1_MALI_AFBCD_G_REORDER,	\
		    VIU_OSD1_MALI_REORDER_G) |		\
	 FIELD_PREP(VIU_OSD1_MALI_AFBCD_R_REORDER,	\
		    VIU_OSD1_MALI_REORDER_R))

#define OSD1_MALI_ORDER_ARGB				\
	(FIELD_PREP(VIU_OSD1_MALI_AFBCD_A_REORDER,	\
		    VIU_OSD1_MALI_REORDER_A) |		\
	 FIELD_PREP(VIU_OSD1_MALI_AFBCD_B_REORDER,	\
		    VIU_OSD1_MALI_REORDER_R) |		\
	 FIELD_PREP(VIU_OSD1_MALI_AFBCD_G_REORDER,	\
		    VIU_OSD1_MALI_REORDER_G) |		\
	 FIELD_PREP(VIU_OSD1_MALI_AFBCD_R_REORDER,	\
		    VIU_OSD1_MALI_REORDER_B))

void meson_viu_g12a_enable_osd1_afbc(struct meson_drm *priv)
{
	u32 afbc_order = OSD1_MALI_ORDER_ARGB;

	/* Enable Mali AFBC Unpack */
	writel_bits_relaxed(VIU_OSD1_MALI_UNPACK_EN,
			    VIU_OSD1_MALI_UNPACK_EN,
			    priv->io_base + _REG(VIU_OSD1_MALI_UNPACK_CTRL));

	switch (priv->afbcd.format) {
	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_ABGR8888:
		afbc_order = OSD1_MALI_ORDER_ABGR;
		break;
	}

	/* Setup RGBA Reordering */
	writel_bits_relaxed(VIU_OSD1_MALI_AFBCD_A_REORDER |
			    VIU_OSD1_MALI_AFBCD_B_REORDER |
			    VIU_OSD1_MALI_AFBCD_G_REORDER |
			    VIU_OSD1_MALI_AFBCD_R_REORDER,
			    afbc_order,
			    priv->io_base + _REG(VIU_OSD1_MALI_UNPACK_CTRL));

	/* Select AFBCD path for OSD1 */
	writel_bits_relaxed(OSD_PATH_OSD_AXI_SEL_OSD1_AFBCD,
			    OSD_PATH_OSD_AXI_SEL_OSD1_AFBCD,
			    priv->io_base + _REG(OSD_PATH_MISC_CTRL));
}

void meson_viu_g12a_disable_osd1_afbc(struct meson_drm *priv)
{
	/* Disable AFBCD path for OSD1 */
	writel_bits_relaxed(OSD_PATH_OSD_AXI_SEL_OSD1_AFBCD, 0,
			    priv->io_base + _REG(OSD_PATH_MISC_CTRL));

	/* Disable AFBCD unpack */
	writel_bits_relaxed(VIU_OSD1_MALI_UNPACK_EN, 0,
			    priv->io_base + _REG(VIU_OSD1_MALI_UNPACK_CTRL));
}

void meson_viu_gxm_enable_osd1_afbc(struct meson_drm *priv)
{
	writel_bits_relaxed(MALI_AFBC_MISC, FIELD_PREP(MALI_AFBC_MISC, 0x90),
			    priv->io_base + _REG(VIU_MISC_CTRL1));
}

void meson_viu_gxm_disable_osd1_afbc(struct meson_drm *priv)
{
	writel_bits_relaxed(MALI_AFBC_MISC, FIELD_PREP(MALI_AFBC_MISC, 0x00),
			    priv->io_base + _REG(VIU_MISC_CTRL1));
}

static int meson_ctm_s31_32_to_13bit(u64 val)
{
	bool negative = !!(val & BIT_ULL(63));
	u64 mag = val & ~BIT_ULL(63);
	int coeff;

	/*
	 * Convert S31.32 sign-magnitude to 13-bit signed (Q1.11).
	 * Right-shift by 21 (32 fractional - 11 fractional) with rounding.
	 */
	coeff = (int)((mag + BIT_ULL(20)) >> 21);

	/* Clamp to 12-bit magnitude (13-bit signed max) */
	if (coeff > 0xfff)
		coeff = 0xfff;

	return negative ? -coeff : coeff;
}

/*
 * Audit #8: this used to write VIU_MATRIX_OSD_EOTF directly, synchronous MMIO
 * from meson_crtc_atomic_flush() in the commit tail -- the one remaining
 * direct-from-atomic_flush register write in this driver, violating its own
 * deferred/vsync-IRQ discipline (every other register goes through a
 * priv->viu.*_commit flag consumed by meson_crtc_irq). It now only updates
 * the cache; meson_viu_apply_ctm() does the actual write, called from the
 * vsync IRQ and also reused to restore the CTM after meson_viu_osd1_reset()
 * (#6) and at resume (#10).
 */
void meson_viu_set_ctm(struct meson_drm *priv, struct drm_color_ctm *ctm)
{
	int i;

	if (!ctm) {
		priv->viu.cm_ctm_active = false;
		return;
	}

	for (i = 0; i < 9; i++)
		priv->viu.cm_ctm_coeffs[i] =
			meson_ctm_s31_32_to_13bit(ctm->matrix[i]);

	priv->viu.cm_ctm_coeffs[9] = EOTF_COEFF_RIGHTSHIFT;
	priv->viu.cm_ctm_active = true;
}

/* Apply the cached CTM (or bypass) to hardware. Cheap (no polling) -- safe
 * to call from the vsync IRQ (#8) or from meson_viu_osd1_reset() (#6),
 * which already runs under priv->drm->event_lock via its plane caller. */
void meson_viu_apply_ctm(struct meson_drm *priv)
{
	if (priv->viu.cm_ctm_active)
		meson_viu_set_osd_matrix(priv, VIU_MATRIX_OSD_EOTF,
					 priv->viu.cm_ctm_coeffs, true);
	else
		meson_viu_set_osd_matrix(priv, VIU_MATRIX_OSD_EOTF,
					 eotf_bypass_coeff, false);
}

/*
 * The active VPP gamma LUT on all supported meson SoCs (GXBB..SM1, i.e. pre-T7)
 * is the L_GAMMA block (L_GAMMA_CNTL/DATA/ADDR_PORT, 0x1400/0x1401/0x1402), NOT
 * the GAMMA_*_PORT block (0x1480-0x1482) this code used to write -- that block is
 * not in the active pipe, so the DRM GAMMA_LUT property was a silent no-op
 * (CM-001). The Amlogic BSP (amvecm: vpp_set_lcd_gamma_table /
 * vpp_enable_lcd_gamma_table) drives L_GAMMA for the whole pre-T7 family and only
 * switches to LCD_GAMMA_*_PORT0 (0x14b4) on T7+. The L_GAMMA LUT port needs a
 * ready handshake: poll ADR_RDY before programming the address, WR_RDY before
 * each data write (the GAMMA_*_PORT path skipped this).
 */
/*
 * Audit #9: each of the 3x256 LUT writes was preceded by a busy-spun (no
 * delay) poll of up to 10000 readl() calls, with NO error path -- worst
 * case ~7.7M MMIO reads (~0.8s) in the commit tail if WR_RDY never
 * asserts, and the code wrote the LUT entry anyway once the poll gave up.
 * Use the bounded, error-returning poll helper. The ATOMIC variant is
 * required because meson_viu_apply_gamma() below also runs from the vsync
 * hard-IRQ (#8); it degrades gracefully (busy-wait, no sleep) when also
 * called from the process-context resume path (#10).
 */
static int meson_viu_lgamma_wait(struct meson_drm *priv, u32 rdy)
{
	u32 val;

	return readl_relaxed_poll_timeout_atomic(
			priv->io_base + _REG(L_GAMMA_CNTL_PORT),
			val, val & rdy, 10, 10000);
}

static int meson_viu_lgamma_channel(struct meson_drm *priv, u32 sel,
				    const u16 *chan, unsigned int size)
{
	unsigned int i;
	int ret;

	ret = meson_viu_lgamma_wait(priv, L_GAMMA_CNTL_PORT_ADR_RDY);
	if (ret)
		return ret;

	writel(L_GAMMA_ADDR_PORT_AUTO_INC | sel,
	       priv->io_base + _REG(L_GAMMA_ADDR_PORT));

	for (i = 0; i < size; i++) {
		ret = meson_viu_lgamma_wait(priv, L_GAMMA_CNTL_PORT_WR_RDY);
		if (ret)
			return ret;

		writel(drm_color_lut_extract(chan[i], 10),
		       priv->io_base + _REG(L_GAMMA_DATA_PORT));
	}

	return 0;
}

/*
 * Audit #8: this used to write the L_GAMMA_* ports directly, synchronous
 * MMIO from meson_crtc_atomic_flush() in the commit tail. It now only
 * copies the LUT into the cache (no MMIO, no polling); meson_viu_apply_gamma()
 * does the actual hardware load, deferred to the vsync IRQ and also reused
 * to restore the LUT at resume (#10).
 */
void meson_viu_set_gamma_lut(struct meson_drm *priv,
			     struct drm_color_lut *lut, unsigned int size)
{
	unsigned int i;

	if (!lut || !size) {
		priv->viu.cm_gamma_active = false;
		return;
	}

	if (size > 256)
		size = 256;

	for (i = 0; i < size; i++) {
		priv->viu.cm_gamma_r[i] = lut[i].red;
		priv->viu.cm_gamma_g[i] = lut[i].green;
		priv->viu.cm_gamma_b[i] = lut[i].blue;
	}
	priv->viu.cm_gamma_size = size;
	priv->viu.cm_gamma_active = true;
}

/*
 * Apply the cached gamma LUT (or disable) to hardware. May run from the
 * vsync IRQ (#8) or from process context at resume (#10). Returns
 * -ETIMEDOUT (propagated from the poll, #9) if the L_GAMMA handshake never
 * becomes ready -- the LUT is left/forced disabled rather than partially
 * written; the caller should treat a non-zero return as a logged failure.
 */
int meson_viu_apply_gamma(struct meson_drm *priv)
{
	int ret;

	if (!priv->viu.cm_gamma_active) {
		writel_bits_relaxed(L_GAMMA_CNTL_PORT_EN, 0,
				    priv->io_base + _REG(L_GAMMA_CNTL_PORT));
		return 0;
	}

	ret = meson_viu_lgamma_channel(priv, L_GAMMA_ADDR_PORT_SEL_R,
				       priv->viu.cm_gamma_r,
				       priv->viu.cm_gamma_size);
	if (!ret)
		ret = meson_viu_lgamma_channel(priv, L_GAMMA_ADDR_PORT_SEL_G,
					       priv->viu.cm_gamma_g,
					       priv->viu.cm_gamma_size);
	if (!ret)
		ret = meson_viu_lgamma_channel(priv, L_GAMMA_ADDR_PORT_SEL_B,
					       priv->viu.cm_gamma_b,
					       priv->viu.cm_gamma_size);
	if (ret) {
		dev_err(priv->dev,
			"VIU gamma LUT: L_GAMMA port not ready, aborting LUT load (%d)\n",
			ret);
		writel_bits_relaxed(L_GAMMA_CNTL_PORT_EN, 0,
				    priv->io_base + _REG(L_GAMMA_CNTL_PORT));
		return ret;
	}

	/* Enable gamma correction */
	writel_bits_relaxed(L_GAMMA_CNTL_PORT_EN, L_GAMMA_CNTL_PORT_EN,
			    priv->io_base + _REG(L_GAMMA_CNTL_PORT));
	return 0;
}

void meson_viu_init(struct meson_drm *priv)
{
	uint32_t reg;

	/* Disable OSDs */
	writel_bits_relaxed(VIU_OSD1_OSD_BLK_ENABLE | VIU_OSD1_OSD_ENABLE, 0,
			    priv->io_base + _REG(VIU_OSD1_CTRL_STAT));
	writel_bits_relaxed(VIU_OSD1_OSD_BLK_ENABLE | VIU_OSD1_OSD_ENABLE, 0,
			    priv->io_base + _REG(VIU_OSD2_CTRL_STAT));

	/* On GXL/GXM, Use the 10bit HDR conversion matrix */
	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXM) ||
	    meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXL)) {
		meson_viu_load_matrix(priv);
		/*
		 * Audit #7: program OSD2's RGB709->YUV709l output matrix here,
		 * once, instead of from meson_viu_load_matrix() (which
		 * meson_viu_osd1_reset() also calls on every OSD1 re-enable --
		 * see the comment there). OSD2 needs the same output CSC as
		 * OSD1 or its colours come out permuted (red->green->blue,
		 * white->pink); GXL/GXM analog of the G12A
		 * meson_viu_set_g12a_osd2_matrix fix.
		 */
		meson_viu_set_viu_osd2_matrix(priv, RGB709_to_YUV709l_coeff,
					      true);
	} else if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A)) {
		meson_viu_set_g12a_osd1_matrix(priv, RGB709_to_YUV709l_coeff,
					       true);
		/* OSD2 needs the same RGB709->YUV709l output CSC as OSD1 --
		 * upstream never programs it, so OSD2 colours come out permuted
		 * (red->green->blue, white->pink). Plane-correctness P2. */
		meson_viu_set_g12a_osd2_matrix(priv, RGB709_to_YUV709l_coeff,
					       true);
		/* fix green/pink color distortion from vendor u-boot */
		writel_bits_relaxed(OSD1_HDR2_CTRL_REG_ONLY_MAT |
				OSD1_HDR2_CTRL_VDIN0_HDR2_TOP_EN, 0,
				priv->io_base + _REG(OSD1_HDR2_CTRL));
	}

	/* Initialize OSD1 fifo control register */
	reg = VIU_OSD_DDR_PRIORITY_URGENT |
		VIU_OSD_FIFO_DEPTH_VAL(32) | /* fifo_depth_val: 32*8=256 */
		VIU_OSD_WORDS_PER_BURST(4) | /* 4 words in 1 burst */
		VIU_OSD_FIFO_LIMITS(2);      /* fifo_lim: 2*16=32 */

	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A))
		reg |= (VIU_OSD_BURST_LENGTH_32 | VIU_OSD_HOLD_FIFO_LINES(31));
	else
		reg |= (VIU_OSD_BURST_LENGTH_96 | VIU_OSD_HOLD_FIFO_LINES(31));

	writel_relaxed(reg, priv->io_base + _REG(VIU_OSD1_FIFO_CTRL_STAT));
	writel_relaxed(reg, priv->io_base + _REG(VIU_OSD2_FIFO_CTRL_STAT));

	/* Set OSD alpha replace value */
	writel_bits_relaxed(0xff << OSD_REPLACE_SHIFT,
			    0xff << OSD_REPLACE_SHIFT,
			    priv->io_base + _REG(VIU_OSD1_CTRL_STAT2));
	writel_bits_relaxed(0xff << OSD_REPLACE_SHIFT,
			    0xff << OSD_REPLACE_SHIFT,
			    priv->io_base + _REG(VIU_OSD2_CTRL_STAT2));

	/* Disable VD1 AFBC */
	/* di_mif0_en=0 mif0_to_vpp_en=0 di_mad_en=0 and afbc vd1 set=0*/
	writel_bits_relaxed(VIU_CTRL0_VD1_AFBC_MASK, 0,
			    priv->io_base + _REG(VIU_MISC_CTRL0));
	writel_relaxed(0, priv->io_base + _REG(AFBC_ENABLE));

	writel_relaxed(0x00FF00C0,
			priv->io_base + _REG(VD1_IF0_LUMA_FIFO_SIZE));
	writel_relaxed(0x00FF00C0,
			priv->io_base + _REG(VD2_IF0_LUMA_FIFO_SIZE));

	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A)) {
		u32 val = (u32)VIU_OSD_BLEND_REORDER(0, 1) |
			  (u32)VIU_OSD_BLEND_REORDER(1, 0) |
			  (u32)VIU_OSD_BLEND_REORDER(2, 0) |
			  (u32)VIU_OSD_BLEND_REORDER(3, 0) |
			  (u32)VIU_OSD_BLEND_DIN_EN(1) |
			  (u32)VIU_OSD_BLEND1_DIN3_BYPASS_TO_DOUT1 |
			  (u32)VIU_OSD_BLEND1_DOUT_BYPASS_TO_BLEND2 |
			  (u32)VIU_OSD_BLEND_DIN0_BYPASS_TO_DOUT0 |
			  (u32)VIU_OSD_BLEND_BLEN2_PREMULT_EN(1) |
			  (u32)VIU_OSD_BLEND_HOLD_LINES(4);
		writel_relaxed(val, priv->io_base + _REG(VIU_OSD_BLEND_CTRL));

		writel_relaxed(OSD_BLEND_PATH_SEL_ENABLE,
			       priv->io_base + _REG(OSD1_BLEND_SRC_CTRL));
		writel_relaxed(OSD_BLEND_PATH_SEL_ENABLE,
			       priv->io_base + _REG(OSD2_BLEND_SRC_CTRL));
		writel_relaxed(0, priv->io_base + _REG(VD1_BLEND_SRC_CTRL));
		writel_relaxed(0, priv->io_base + _REG(VD2_BLEND_SRC_CTRL));
		writel_relaxed(0,
				priv->io_base + _REG(VIU_OSD_BLEND_DUMMY_DATA0));
		writel_relaxed(0,
				priv->io_base + _REG(VIU_OSD_BLEND_DUMMY_ALPHA));

		/* VPU-SWEEP-001: the VPP post-blend background (the full-canvas base
		 * layer shown wherever no plane covers a pixel) resets to YCbCr
		 * 0x00008080 -- Y=0, not studio-black Y=16 -- which the output CSC
		 * turns into a full-screen GREEN field. It is hidden while a
		 * full-screen plane (VD1) covers the canvas, but shows through in
		 * VD1-disabled compose states (e.g. VD2-only PiP over a transparent
		 * OSD1 stage). Upstream never programs it. Set it to studio-black +
		 * opaque, mirroring the BSP (video_hw.c post_blend_dummy_data_update /
		 * VPP_POST_BLEND_DUMMY_ALPHA init = 0x7fffffff). Hardware-verified on
		 * SM1: 0x00008080 -> RGB[~10,87,11] (green); 0x00108080 + opaque
		 * alpha -> RGB[~9,16,11] (black). */
		writel_relaxed(VPP_POST_BLEND_DUMMY_YUV_BLACK,
			       priv->io_base + _REG(VPP_POST_BLEND_BLEND_DUMMY_DATA));
		writel_relaxed(VPP_POST_BLEND_DUMMY_ALPHA_OPAQUE,
			       priv->io_base + _REG(VPP_POST_BLEND_DUMMY_ALPHA));

		writel_bits_relaxed(DOLBY_BYPASS_EN(0xc), DOLBY_BYPASS_EN(0xc),
				    priv->io_base + _REG(DOLBY_PATH_CTRL));

		meson_viu_g12a_disable_osd1_afbc(priv);
	}

	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXM))
		meson_viu_gxm_disable_osd1_afbc(priv);

	priv->viu.osd1_enabled = false;
	priv->viu.osd1_commit = false;
	priv->viu.osd1_interlace = false;

	priv->viu.osd2_enabled = false;
	priv->viu.osd2_commit = false;
}
