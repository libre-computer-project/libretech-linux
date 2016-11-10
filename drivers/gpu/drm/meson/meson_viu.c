/*
 * Copyright (C) 2016 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 * Copyright (C) 2015 Amlogic, Inc. All rights reserved.
 * Copyright (C) 2014 Endless Mobile
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 * 02111-1307, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <drm/drmP.h>
#include "meson_drv.h"
#include "meson_viu.h"
#include "meson_vpp.h"
#include "meson_venc.h"
#include "meson_canvas.h"
#include "meson_registers.h"

/*
 * VIU Handles the Pixel scanout and the basic Colorspace conversions
 * We handle the following features :
 * - OSD1 RGB565/RGB888/xRGB8888 scanout
 * - RGB conversion to x/cb/cr
 * - Progressive or Interlace buffer scanout
 * - OSD1 Commit on Vsync
 * - HDR OSD matrix for GXL/GXM
 *
 * What is missing :
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

/* OSDx_BLKx_CFG */
#define OSD_CANVAS_SEL		16

#define OSD_ENDIANNESS_LE	BIT(15)
#define OSD_ENDIANNESS_BE	(0)

#define OSD_BLK_MODE_422	(0x03 << 8)
#define OSD_BLK_MODE_16		(0x04 << 8)
#define OSD_BLK_MODE_32		(0x05 << 8)
#define OSD_BLK_MODE_24		(0x07 << 8)

#define OSD_OUTPUT_COLOR_RGB	BIT(7)
#define OSD_OUTPUT_COLOR_YUV	(0)

#define OSD_COLOR_MATRIX_32_RGBA	(0x00 << 2)
#define OSD_COLOR_MATRIX_32_ARGB	(0x01 << 2)
#define OSD_COLOR_MATRIX_32_ABGR	(0x02 << 2)
#define OSD_COLOR_MATRIX_32_BGRA	(0x03 << 2)

#define OSD_COLOR_MATRIX_24_RGB		(0x00 << 2)

#define OSD_COLOR_MATRIX_16_RGB655	(0x00 << 2)
#define OSD_COLOR_MATRIX_16_RGB565	(0x04 << 2)

#define OSD_INTERLACE_ENABLED	BIT(1)
#define OSD_INTERLACE_ODD	BIT(0)
#define OSD_INTERLACE_EVEN	(0)

/* OSDx_CTRL_STAT */
#define OSD_ENABLE		BIT(21)
#define OSD_BLK0_ENABLE		BIT(0)

#define OSD_GLOBAL_ALPHA_SHIFT	12

/* OSDx_CTRL_STAT2 */
#define OSD_REPLACE_EN		BIT(14)
#define OSD_REPLACE_SHIFT	6

/* Takes a fixed 16.16 number and converts it to integer. */
static inline int64_t fixed16_to_int(int64_t value)
{
	return value >> 16;
}

void meson_viu_update_osd1(struct meson_drm *priv, struct drm_plane *plane)
{
	struct drm_plane_state *state = plane->state;
	struct drm_framebuffer *fb = state->fb;
	struct drm_rect src = {
		.x1 = (state->src_x),
		.y1 = (state->src_y),
		.x2 = (state->src_x + state->src_w),
		.y2 = (state->src_y + state->src_h),
	};
	struct drm_rect dest = {
		.x1 = state->crtc_x,
		.y1 = state->crtc_y,
		.x2 = state->crtc_x + state->crtc_w,
		.y2 = state->crtc_y + state->crtc_h,
	};
	unsigned long flags;

	spin_lock_irqsave(&priv->drm->event_lock, flags);

	/* Enable OSD and BLK0, set max global alpha */
	priv->viu.osd1_ctrl_stat = OSD_ENABLE |
				   (0xFF << OSD_GLOBAL_ALPHA_SHIFT) |
				   OSD_BLK0_ENABLE;

	/* Set up BLK0 to point to the right canvas */
	priv->viu.osd1_blk0_cfg[0] = ((MESON_CANVAS_ID_OSD1 << OSD_CANVAS_SEL) |
				      OSD_ENDIANNESS_LE);

	/* On GXBB, Use the old non-HDR RGB2YUV converter */
	if (meson_vpu_is_compatible(priv, "amlogic,meson-gxbb-vpu"))
		priv->viu.osd1_blk0_cfg[0] |= OSD_OUTPUT_COLOR_RGB;

	switch (fb->pixel_format) {
	case DRM_FORMAT_XRGB8888:
		/* For XRGB, replace the pixel's alpha by 0xFF */
		writel_bits_relaxed(OSD_REPLACE_EN, OSD_REPLACE_EN,
				    priv->io_base + _REG(VIU_OSD1_CTRL_STAT2));
		priv->viu.osd1_blk0_cfg[0] |= OSD_BLK_MODE_32 |
					      OSD_COLOR_MATRIX_32_ARGB;
		break;
	case DRM_FORMAT_ARGB8888:
		/* For ARGB, use the pixel's alpha */
		writel_bits_relaxed(OSD_REPLACE_EN, 0,
				    priv->io_base + _REG(VIU_OSD1_CTRL_STAT2));
		priv->viu.osd1_blk0_cfg[0] |= OSD_BLK_MODE_32 |
					      OSD_COLOR_MATRIX_32_ARGB;
		break;
	case DRM_FORMAT_RGB888:
		priv->viu.osd1_blk0_cfg[0] |= OSD_BLK_MODE_24 |
					      OSD_COLOR_MATRIX_24_RGB;
		break;
	case DRM_FORMAT_RGB565:
		priv->viu.osd1_blk0_cfg[0] |= OSD_BLK_MODE_16 |
					      OSD_COLOR_MATRIX_16_RGB565;
		break;
	};

	if (state->crtc->mode.flags & DRM_MODE_FLAG_INTERLACE) {
		priv->viu.osd1_interlace = true;

		dest.y1 /= 2;
		dest.y2 /= 2;
	} else {
		priv->viu.osd1_interlace = true;
		meson_vpp_disable_interlace_vscaler_osd1(priv);
	}

	/*
	 * The format of these registers is (x2 << 16 | x1),
	 * where x2 is exclusive.
	 * e.g. +30x1920 would be (1919 << 16) | 30
	 */
	priv->viu.osd1_blk0_cfg[1] = ((fixed16_to_int(src.x2) - 1) << 16) |
					fixed16_to_int(src.x1);
	priv->viu.osd1_blk0_cfg[2] = ((fixed16_to_int(src.y2) - 1) << 16) |
					fixed16_to_int(src.y1);
	priv->viu.osd1_blk0_cfg[3] = ((dest.x2 - 1) << 16) | dest.x1;
	priv->viu.osd1_blk0_cfg[4] = ((dest.y2 - 1) << 16) | dest.y1;

	spin_unlock_irqrestore(&priv->drm->event_lock, flags);
}

void meson_viu_sync_osd1(struct meson_drm *priv)
{
	/* Update the OSD registers */
	if (priv->viu.osd1_enabled && priv->viu.osd1_commit) {
		writel_relaxed(priv->viu.osd1_ctrl_stat,
				priv->io_base + _REG(VIU_OSD1_CTRL_STAT));
		writel_relaxed(priv->viu.osd1_blk0_cfg[0],
				priv->io_base + _REG(VIU_OSD1_BLK0_CFG_W0));
		writel_relaxed(priv->viu.osd1_blk0_cfg[1],
				priv->io_base + _REG(VIU_OSD1_BLK0_CFG_W1));
		writel_relaxed(priv->viu.osd1_blk0_cfg[2],
				priv->io_base + _REG(VIU_OSD1_BLK0_CFG_W2));
		writel_relaxed(priv->viu.osd1_blk0_cfg[3],
				priv->io_base + _REG(VIU_OSD1_BLK0_CFG_W3));
		writel_relaxed(priv->viu.osd1_blk0_cfg[4],
				priv->io_base + _REG(VIU_OSD1_BLK0_CFG_W4));

		if (priv->viu.osd1_interlace) {
			struct drm_plane *plane = priv->primary_plane;
			struct drm_plane_state *state = plane->state;
			struct drm_rect dest = {
				.x1 = state->crtc_x,
				.y1 = state->crtc_y,
				.x2 = state->crtc_x + state->crtc_w,
				.y2 = state->crtc_y + state->crtc_h,
			};

			meson_vpp_setup_interlace_vscaler_osd1(priv, &dest);
		}

		meson_vpp_enable_osd1(priv);

		priv->viu.osd1_commit = false;
	}
}


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

void meson_viu_set_osd_matrix(struct meson_drm *priv,
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

void meson_viu_set_osd_lut(struct meson_drm *priv, enum viu_lut_sel_e lut_sel,
			   unsigned int *r_map, unsigned int *g_map,
			   unsigned int *b_map,
			   bool csc_on)
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

		for (i = 0; i < 20; i++)
			writel(r_map[i * 2] | (r_map[i * 2 + 1] << 16),
				priv->io_base + _REG(data_port));

		writel(r_map[OSD_OETF_LUT_SIZE - 1] | (g_map[0] << 16),
			priv->io_base + _REG(data_port));

		for (i = 0; i < 20; i++)
			writel(g_map[i * 2 + 1] | (g_map[i * 2 + 2] << 16),
				priv->io_base + _REG(data_port));

		for (i = 0; i < 20; i++)
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

		for (i = 0; i < 20; i++)
			writel(r_map[i * 2] | (r_map[i * 2 + 1] << 16),
				priv->io_base + _REG(data_port));

		writel(r_map[OSD_EOTF_LUT_SIZE - 1] | (g_map[0] << 16),
			priv->io_base + _REG(data_port));

		for (i = 0; i < 20; i++)
			writel(g_map[i * 2 + 1] | (g_map[i * 2 + 2] << 16),
				priv->io_base + _REG(data_port));

		for (i = 0; i < 20; i++)
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
}

void meson_viu_init(struct meson_drm *priv)
{
	uint32_t reg;

	/* Disable OSDs */
	writel_bits_relaxed(BIT(0) | BIT(21), 0,
			priv->io_base + _REG(VIU_OSD1_CTRL_STAT));
	writel_bits_relaxed(BIT(0) | BIT(21), 0,
			priv->io_base + _REG(VIU_OSD2_CTRL_STAT));

	/* On GXL/GXM, Use the 10bit HDR conversion matrix */
	if (meson_vpu_is_compatible(priv, "amlogic,meson-gxm-vpu") ||
	    meson_vpu_is_compatible(priv, "amlogic,meson-gxl-vpu"))
		meson_viu_load_matrix(priv);

	/* Initialize OSD1 fifo control register */
	reg = BIT(0) |	/* Urgent DDR request priority */
	      (4 << 5) | /* hold_fifo_lines */
	      (3 << 10) | /* burst length 64 */
	      (32 << 12) | /* fifo_depth_val: 32*8=256 */
	      (2 << 22) | /* 4 words in 1 burst */
	      (2 << 24);
	writel_relaxed(reg, priv->io_base + _REG(VIU_OSD1_FIFO_CTRL_STAT));
	writel_relaxed(reg, priv->io_base + _REG(VIU_OSD2_FIFO_CTRL_STAT));

	/* Set OSD alpha replace value */
	writel_bits_relaxed(0xff << OSD_REPLACE_SHIFT,
			    0xff << OSD_REPLACE_SHIFT,
			    priv->io_base + _REG(VIU_OSD1_CTRL_STAT2));
	writel_bits_relaxed(0xff << OSD_REPLACE_SHIFT,
			    0xff << OSD_REPLACE_SHIFT,
			    priv->io_base + _REG(VIU_OSD2_CTRL_STAT2));

	priv->viu.osd1_enabled = false;
	priv->viu.osd1_commit = false;
	priv->viu.osd1_interlace = false;
}
