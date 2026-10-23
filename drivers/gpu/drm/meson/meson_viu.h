/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2016 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 */

/* Video Input Unit */

#ifndef __MESON_VIU_H
#define __MESON_VIU_H

#include <linux/bitfield.h>

/* OSDx_BLKx_CFG */
#define OSD_MALI_SRC_EN		BIT(30)

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

#define OSD_MALI_COLOR_MODE_R8		(0 << 8)
#define OSD_MALI_COLOR_MODE_YUV422	(1 << 8)
#define OSD_MALI_COLOR_MODE_RGB565	(2 << 8)
#define OSD_MALI_COLOR_MODE_RGBA5551	(3 << 8)
#define OSD_MALI_COLOR_MODE_RGBA4444	(4 << 8)
#define OSD_MALI_COLOR_MODE_RGBA8888	(5 << 8)
#define OSD_MALI_COLOR_MODE_RGB888	(7 << 8)
#define OSD_MALI_COLOR_MODE_YUV422_10B	(8 << 8)
#define OSD_MALI_COLOR_MODE_RGBA1010102	(9 << 8)

#define OSD_INTERLACE_ENABLED	BIT(1)
#define OSD_INTERLACE_ODD	BIT(0)
#define OSD_INTERLACE_EVEN	(0)

/* OSDx_CTRL_STAT */
#define OSD_ENABLE		BIT(21)
#define OSD_MEM_LINEAR_ADDR	BIT(2)
#define OSD_BLK0_ENABLE		BIT(0)

#define OSD_GLOBAL_ALPHA_SHIFT	12

/* OSDx_CTRL_STAT2 */
#define OSD_DPATH_MALI_AFBCD	BIT(15)
#define OSD_REPLACE_EN		BIT(14)
#define OSD_REPLACE_SHIFT	6
#define OSD_PENDING_STAT_CLEAN	BIT(1)

/* OSD Scaler Control Bits (shared by OSD1 and OSD2 scalers) */
/* OSD_SCI_WH_M1 */
#define SCI_WH_M1_W(w)			FIELD_PREP(GENMASK(28, 16), w)
#define SCI_WH_M1_H(h)			FIELD_PREP(GENMASK(12, 0), h)

/* OSD_SCO_H_START_END / OSD_SCO_V_START_END */
#define SCO_HV_START(start)		FIELD_PREP(GENMASK(27, 16), start)
#define SCO_HV_END(end)			FIELD_PREP(GENMASK(11, 0), end)

/* OSD_SC_CTRL0 */
#define SC_CTRL0_PATH_EN		BIT(3)
#define SC_CTRL0_SEL_OSD1		BIT(2)

/* OSD_VSC_CTRL0 */
#define VSC_BANK_LEN(value)		FIELD_PREP(GENMASK(2, 0), value)
#define VSC_TOP_INI_RCV_NUM(value)	FIELD_PREP(GENMASK(6, 3), value)
#define VSC_TOP_RPT_L0_NUM(value)	FIELD_PREP(GENMASK(9, 8), value)
#define VSC_BOT_INI_RCV_NUM(value)	FIELD_PREP(GENMASK(14, 11), value)
#define VSC_BOT_RPT_L0_NUM(value)	FIELD_PREP(GENMASK(17, 16), value)
#define VSC_PROG_INTERLACE		BIT(23)
#define VSC_VERTICAL_SCALER_EN		BIT(24)

/* OSD_VSC_INI_PHASE */
#define VSC_INI_PHASE_BOT(bottom)	FIELD_PREP(GENMASK(31, 16), bottom)
#define VSC_INI_PHASE_TOP(top)		FIELD_PREP(GENMASK(15, 0), top)

/* OSD_HSC_CTRL0 */
#define HSC_BANK_LENGTH(value)		FIELD_PREP(GENMASK(2, 0), value)
#define HSC_INI_RCV_NUM0(value)		FIELD_PREP(GENMASK(6, 3), value)
#define HSC_RPT_P0_NUM0(value)		FIELD_PREP(GENMASK(9, 8), value)
#define HSC_HORIZ_SCALER_EN		BIT(22)

/* OSD_VSC_PHASE_STEP / OSD_HSC_PHASE_STEP */
#define SC_PHASE_STEP(value)		FIELD_PREP(GENMASK(27, 0), value)

struct drm_color_ctm;
struct drm_color_lut;

void meson_viu_set_ctm(struct meson_drm *priv, struct drm_color_ctm *ctm);
void meson_viu_set_gamma_lut(struct meson_drm *priv,
			     struct drm_color_lut *lut, unsigned int size);
/* Apply the cached CTM/gamma LUT to hardware (audit #6/#8/#9/#10). Callable
 * from the vsync IRQ, from meson_viu_osd1_reset(), or from process context
 * at probe/resume. meson_viu_apply_gamma() can fail (-ETIMEDOUT) if the
 * L_GAMMA handshake never becomes ready; the LUT is left disabled rather
 * than partially written. */
void meson_viu_apply_ctm(struct meson_drm *priv);
int meson_viu_apply_gamma(struct meson_drm *priv);
void meson_viu_osd1_reset(struct meson_drm *priv);
void meson_viu_g12a_enable_osd1_afbc(struct meson_drm *priv);
void meson_viu_g12a_disable_osd1_afbc(struct meson_drm *priv);
void meson_viu_gxm_enable_osd1_afbc(struct meson_drm *priv);
void meson_viu_gxm_disable_osd1_afbc(struct meson_drm *priv);
void meson_viu_init(struct meson_drm *priv);

#endif /* __MESON_VIU_H */
