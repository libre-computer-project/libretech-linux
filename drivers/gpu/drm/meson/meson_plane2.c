// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2016 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 *
 * OSD2 overlay plane with scaling (G12A+) and interlace support
 */

#include <linux/bitfield.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_blend.h>
#include <drm/drm_device.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>

#include "meson_plane2.h"
#include "meson_registers.h"
#include "meson_viu.h"

/* Scaling limits: 1/5x to 5x */
#define FRAC_16_16(mult, div)    (((mult) << 16) / (div))
#define OSD2_MIN_SCALE           FRAC_16_16(1, 5)
#define OSD2_MAX_SCALE           FRAC_16_16(5, 1)

struct meson_plane2 {
	struct drm_plane base;
	struct meson_drm *priv;
};
#define to_meson_plane2(x) container_of(x, struct meson_plane2, base)

/* Takes a fixed 16.16 number and converts it to integer. */
static inline int64_t fixed16_to_int(int64_t value)
{
	return value >> 16;
}

static int meson_plane2_atomic_check(struct drm_plane *plane,
				     struct drm_atomic_state *state)
{
	struct drm_plane_state *new_plane_state =
		drm_atomic_get_new_plane_state(state, plane);
	struct meson_plane2 *meson_plane2 = to_meson_plane2(plane);
	struct meson_drm *priv = meson_plane2->priv;
	struct drm_crtc_state *crtc_state;
	int min_scale, max_scale;

	if (!new_plane_state->crtc)
		return 0;

	crtc_state = drm_atomic_get_crtc_state(state, new_plane_state->crtc);
	if (IS_ERR(crtc_state))
		return PTR_ERR(crtc_state);

	/*
	 * G12A+ has a dedicated OSD2 scaler. Older SoCs share the VPP OSD
	 * scaler with OSD1, so OSD2 cannot scale independently.
	 */
	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A)) {
		min_scale = OSD2_MIN_SCALE;
		max_scale = OSD2_MAX_SCALE;
	} else {
		min_scale = DRM_PLANE_NO_SCALING;
		max_scale = DRM_PLANE_NO_SCALING;
	}

	/*
	 * #16: on pre-G12A, atomic_update() halves the destination window for
	 * an interlaced mode (dest.y1/=2, dest.y2/=2, dst_h/=2) unconditionally,
	 * but the scaler block that engages a compensating 2:1 vertical
	 * downscale + VSC_PROG_INTERLACE is gated to G12A+ only (OSD2 has no
	 * dedicated scaler pre-G12A -- it would have to arbitrate for OSD1's
	 * shared VPP OSD scaler, real hardware-sharing logic not implementable
	 * without a board to verify against). Refuse cleanly instead of a
	 * halved destination window with no compensating scale-down.
	 */
	if (!meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A) &&
	    (crtc_state->mode.flags & DRM_MODE_FLAG_INTERLACE))
		return -EINVAL;

	return drm_atomic_helper_check_plane_state(new_plane_state,
						   crtc_state,
						   min_scale, max_scale,
						   true, true);
}

static void meson_plane2_atomic_update(struct drm_plane *plane,
				       struct drm_atomic_state *state)
{
	struct meson_plane2 *meson_plane2 = to_meson_plane2(plane);
	struct drm_plane_state *new_state =
		drm_atomic_get_new_plane_state(state, plane);
	struct drm_rect dest = drm_plane_state_dest(new_state);
	struct meson_drm *priv = meson_plane2->priv;
	struct drm_framebuffer *fb = new_state->fb;
	struct drm_gem_dma_object *gem;
	unsigned long flags;
	int src_w, src_h, dst_w, dst_h;
	unsigned int hf_phase_step, vf_phase_step;
	unsigned int bot_ini_phase = 0;
	u64 vf_phase_step64;
	unsigned int hf_bank_len = 4;
	unsigned int vf_bank_len = 4;
	unsigned int hsc_ini_rcv_num, hsc_ini_rpt_p0_num;
	unsigned int vsc_ini_rcv_num, vsc_ini_rpt_p0_num;
	unsigned int vsc_bot_rcv_num = 0;
	unsigned int vsc_bot_rpt_p0_num = 0;

	spin_lock_irqsave(&priv->drm->event_lock, flags);

	/* Enable OSD2 and BLK0, set global alpha from plane state */
	priv->viu.osd2_ctrl_stat = OSD_ENABLE |
				   (((new_state->alpha + 1) >> 8) << OSD_GLOBAL_ALPHA_SHIFT) |
				   OSD_BLK0_ENABLE;

	priv->viu.osd2_ctrl_stat2 = readl(priv->io_base +
					   _REG(VIU_OSD2_CTRL_STAT2));

	/* Set up BLK0 to point to the right canvas */
	priv->viu.osd2_blk0_cfg[0] = priv->canvas_id_osd2 << OSD_CANVAS_SEL;
	priv->viu.osd2_blk0_cfg[0] |= OSD_ENDIANNESS_LE;

	/* On GXBB, Use the old non-HDR RGB2YUV converter */
	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXBB))
		priv->viu.osd2_blk0_cfg[0] |= OSD_OUTPUT_COLOR_RGB;

	switch (fb->format->format) {
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ARGB8888:
		priv->viu.osd2_blk0_cfg[0] |= OSD_BLK_MODE_32 |
					OSD_COLOR_MATRIX_32_ARGB;
		break;
	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_ABGR8888:
		priv->viu.osd2_blk0_cfg[0] |= OSD_BLK_MODE_32 |
					OSD_COLOR_MATRIX_32_ABGR;
		break;
	case DRM_FORMAT_RGBA8888:
	case DRM_FORMAT_RGBX8888:
		priv->viu.osd2_blk0_cfg[0] |= OSD_BLK_MODE_32 |
					OSD_COLOR_MATRIX_32_RGBA;
		break;
	case DRM_FORMAT_BGRA8888:
	case DRM_FORMAT_BGRX8888:
		priv->viu.osd2_blk0_cfg[0] |= OSD_BLK_MODE_32 |
					OSD_COLOR_MATRIX_32_BGRA;
		break;
	case DRM_FORMAT_RGB888:
		priv->viu.osd2_blk0_cfg[0] |= OSD_BLK_MODE_24 |
					OSD_COLOR_MATRIX_24_RGB;
		break;
	case DRM_FORMAT_RGB565:
		priv->viu.osd2_blk0_cfg[0] |= OSD_BLK_MODE_16 |
					OSD_COLOR_MATRIX_16_RGB565;
		break;
	}

	switch (fb->format->format) {
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_RGBX8888:
	case DRM_FORMAT_BGRX8888:
		/* For X formats, replace the pixel's alpha by 0xFF */
		priv->viu.osd2_ctrl_stat2 |= OSD_REPLACE_EN;
		break;
	case DRM_FORMAT_ARGB8888:
	case DRM_FORMAT_ABGR8888:
	case DRM_FORMAT_RGBA8888:
	case DRM_FORMAT_BGRA8888:
		/* For A formats, use the pixel's alpha */
		priv->viu.osd2_ctrl_stat2 &= ~OSD_REPLACE_EN;
		break;
	}

	if (new_state->crtc->mode.flags & DRM_MODE_FLAG_INTERLACE) {
		vsc_bot_rcv_num = 6;
		vsc_bot_rpt_p0_num = 2;
	}

	hsc_ini_rcv_num = hf_bank_len;
	vsc_ini_rcv_num = vf_bank_len;
	hsc_ini_rpt_p0_num = (hf_bank_len / 2) - 1;
	vsc_ini_rpt_p0_num = (vf_bank_len / 2) - 1;

	src_w = fixed16_to_int(new_state->src_w);
	src_h = fixed16_to_int(new_state->src_h);
	dst_w = new_state->crtc_w;
	dst_h = new_state->crtc_h;

	/*
	 * When the output is interlaced, the OSD must switch between
	 * each field using the INTERLACE_SEL_ODD of VIU_OSD2_BLK0_CFG_W0
	 * at each vsync. The vertical scaler handles this if configured
	 * for 2:1 scaling with interlace options enabled.
	 */
	if (new_state->crtc->mode.flags & DRM_MODE_FLAG_INTERLACE) {
		dest.y1 /= 2;
		dest.y2 /= 2;
		dst_h /= 2;
	}

	/*
	 * G12A+ has a dedicated OSD2 scaler at register block 0x3d00.
	 * Configure it when scaling or interlaced output is needed.
	 */
	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A)) {
		hf_phase_step = ((src_w << 18) / dst_w) << 6;

		/*
		 * src_h can be as large as 4096 (mode_config.max_height in
		 * meson_drv.c), so src_h << 20 overflows a 32-bit signed
		 * int for src_h >= 2048. Do the shift in 64-bit before
		 * narrowing back down for the register field.
		 */
		vf_phase_step64 = ((u64)src_h << 20) / dst_h;

		if (new_state->crtc->mode.flags & DRM_MODE_FLAG_INTERLACE)
			bot_ini_phase = ((vf_phase_step64 / 2) >> 4);
		else
			bot_ini_phase = 0;

		vf_phase_step = (unsigned int)(vf_phase_step64 << 4);

		if (src_h != dst_h || src_w != dst_w ||
		    (new_state->crtc->mode.flags & DRM_MODE_FLAG_INTERLACE)) {
			priv->viu.osd2_sc_i_wh_m1 =
				SCI_WH_M1_W(src_w - 1) |
				SCI_WH_M1_H(src_h - 1);
			priv->viu.osd2_sc_o_h_start_end =
				SCO_HV_START(dest.x1) |
				SCO_HV_END(dest.x2 - 1);
			priv->viu.osd2_sc_o_v_start_end =
				SCO_HV_START(dest.y1) |
				SCO_HV_END(dest.y2 - 1);
			priv->viu.osd2_sc_ctrl0 = SC_CTRL0_PATH_EN;
		} else {
			priv->viu.osd2_sc_i_wh_m1 = 0;
			priv->viu.osd2_sc_o_h_start_end = 0;
			priv->viu.osd2_sc_o_v_start_end = 0;
			priv->viu.osd2_sc_ctrl0 = 0;
		}

		if (src_h != dst_h ||
		    (new_state->crtc->mode.flags & DRM_MODE_FLAG_INTERLACE)) {
			priv->viu.osd2_sc_v_ctrl0 =
				VSC_BANK_LEN(vf_bank_len) |
				VSC_TOP_INI_RCV_NUM(vsc_ini_rcv_num) |
				VSC_TOP_RPT_L0_NUM(vsc_ini_rpt_p0_num) |
				VSC_VERTICAL_SCALER_EN;

			if (new_state->crtc->mode.flags & DRM_MODE_FLAG_INTERLACE)
				priv->viu.osd2_sc_v_ctrl0 |=
					VSC_BOT_INI_RCV_NUM(vsc_bot_rcv_num) |
					VSC_BOT_RPT_L0_NUM(vsc_bot_rpt_p0_num) |
					VSC_PROG_INTERLACE;

			priv->viu.osd2_sc_v_phase_step =
				SC_PHASE_STEP(vf_phase_step);
			priv->viu.osd2_sc_v_ini_phase =
				VSC_INI_PHASE_BOT(bot_ini_phase);
		} else {
			priv->viu.osd2_sc_v_ctrl0 = 0;
			priv->viu.osd2_sc_v_phase_step = 0;
			priv->viu.osd2_sc_v_ini_phase = 0;
		}

		if (src_w != dst_w) {
			priv->viu.osd2_sc_h_ctrl0 =
				HSC_BANK_LENGTH(hf_bank_len) |
				HSC_INI_RCV_NUM0(hsc_ini_rcv_num) |
				HSC_RPT_P0_NUM0(hsc_ini_rpt_p0_num) |
				HSC_HORIZ_SCALER_EN;
			priv->viu.osd2_sc_h_phase_step =
				SC_PHASE_STEP(hf_phase_step);
			priv->viu.osd2_sc_h_ini_phase = 0;
		} else {
			priv->viu.osd2_sc_h_ctrl0 = 0;
			priv->viu.osd2_sc_h_phase_step = 0;
			priv->viu.osd2_sc_h_ini_phase = 0;
		}
	}

	/*
	 * The format of these registers is (x2 << 16 | x1),
	 * where x2 is exclusive.
	 * e.g. +30x1920 would be (1919 << 16) | 30
	 */
	priv->viu.osd2_blk0_cfg[1] =
			((fixed16_to_int(new_state->src.x2) - 1) << 16) |
			fixed16_to_int(new_state->src.x1);
	priv->viu.osd2_blk0_cfg[2] =
			((fixed16_to_int(new_state->src.y2) - 1) << 16) |
			fixed16_to_int(new_state->src.y1);
	priv->viu.osd2_blk0_cfg[3] = ((dest.x2 - 1) << 16) | dest.x1;
	priv->viu.osd2_blk0_cfg[4] = ((dest.y2 - 1) << 16) | dest.y1;

	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A)) {
		priv->viu.osd_blend_din1_scope_h =
			((dest.x2 - 1) << 16) | dest.x1;
		priv->viu.osd_blend_din1_scope_v =
			((dest.y2 - 1) << 16) | dest.y1;
		/* VPP-level OSD2 blend output window. Format is (start << 16) | end
		 * (same as VPP_OSD1_BLD_*, opposite byte order to the VIU DIN scopes).
		 * Without this the reg keeps a bogus power-on default (start > end) and
		 * OSD2 renders full-screen garbage instead of the committed window. */
		priv->viu.osd2_bld_h_scope = (dest.x1 << 16) | (dest.x2 - 1);
		priv->viu.osd2_bld_v_scope = (dest.y1 << 16) | (dest.y2 - 1);
	}

	/* Update Canvas with buffer address */
	gem = drm_fb_dma_get_gem_obj(fb, 0);

	priv->viu.osd2_addr = gem->dma_addr;
	priv->viu.osd2_stride = fb->pitches[0];
	priv->viu.osd2_height = fb->height;
	priv->viu.osd2_width = fb->width;

	priv->viu.osd2_zpos = new_state->zpos;
	priv->viu.osd2_enabled = true;

	spin_unlock_irqrestore(&priv->drm->event_lock, flags);
}

static void meson_plane2_atomic_disable(struct drm_plane *plane,
					struct drm_atomic_state *state)
{
	struct meson_plane2 *meson_plane2 = to_meson_plane2(plane);
	struct meson_drm *priv = meson_plane2->priv;

	/* Disable OSD2 */
	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A))
		writel_bits_relaxed(3 << 8, 0,
				    priv->io_base + _REG(OSD2_BLEND_SRC_CTRL));
	else
		writel_bits_relaxed(VPP_OSD2_POSTBLEND, 0,
				    priv->io_base + _REG(VPP_MISC));

	priv->viu.osd2_enabled = false;
}

static const struct drm_plane_helper_funcs meson_plane2_helper_funcs = {
	.atomic_check	= meson_plane2_atomic_check,
	.atomic_disable	= meson_plane2_atomic_disable,
	.atomic_update	= meson_plane2_atomic_update,
};

static const struct drm_plane_funcs meson_plane2_funcs = {
	.update_plane		= drm_atomic_helper_update_plane,
	.disable_plane		= drm_atomic_helper_disable_plane,
	.destroy		= drm_plane_cleanup,
	.reset			= drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_plane_destroy_state,
};

static const uint32_t supported_drm_formats[] = {
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_RGBA8888,
	DRM_FORMAT_BGRA8888,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_RGBX8888,
	DRM_FORMAT_BGRX8888,
	DRM_FORMAT_RGB888,
	DRM_FORMAT_RGB565,
};

static const uint64_t format_modifiers_default[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID,
};

int meson_plane2_create(struct meson_drm *priv)
{
	struct meson_plane2 *meson_plane2;
	struct drm_plane *plane;
	int ret;

	meson_plane2 = devm_kzalloc(priv->drm->dev, sizeof(*meson_plane2),
				    GFP_KERNEL);
	if (!meson_plane2)
		return -ENOMEM;

	meson_plane2->priv = priv;
	plane = &meson_plane2->base;

	/* #29: return value used to go unchecked (matches the pre-existing
	 * pattern in meson_plane.c's meson_plane_create()). */
	ret = drm_universal_plane_init(priv->drm, plane, 0xFF,
					&meson_plane2_funcs,
					supported_drm_formats,
					ARRAY_SIZE(supported_drm_formats),
					format_modifiers_default,
					DRM_PLANE_TYPE_OVERLAY, "meson_overlay_plane");
	if (ret) {
		devm_kfree(priv->drm->dev, meson_plane2);
		return ret;
	}

	drm_plane_helper_add(plane, &meson_plane2_helper_funcs);

	drm_plane_create_alpha_property(plane);

	/*
	 * #14: COVERAGE (straight, non-premultiplied alpha) used to be
	 * advertised here but never read anywhere -- the OSD blend engine is
	 * hardwired to premultiplied alpha (VIU_OSD_BLEND_BLEN2_PREMULT_EN(1),
	 * unconditional in both meson_viu.c's meson_viu_init() and
	 * meson_crtc.c's meson_g12a_crtc_set_osd_order(), the only per-commit
	 * writer of VIU_OSD_BLEND_CTRL). A client selecting COVERAGE (the
	 * common compositor default) silently got premultiplied blending
	 * instead -> dark halos on partially-transparent OSD2 content.
	 * Honouring COVERAGE for real needs new per-commit hardware
	 * programming in meson_crtc.c's set_osd_order(). Until that lands,
	 * only advertise the mode this driver truly implements. PIXEL_NONE
	 * (alpha ignored, treated as opaque) is unaffected by the
	 * premult/coverage distinction -- alpha=1 makes premultiplied and
	 * straight math identical -- so it stays offered.
	 */
	drm_plane_create_blend_mode_property(plane,
					     BIT(DRM_MODE_BLEND_PIXEL_NONE) |
					     BIT(DRM_MODE_BLEND_PREMULTI));

	/* OSD2 z-order is mutable on G12A+ across the full [0,3] plane space:
	 * swappable with OSD1 (stage 1) and orderable against the video group
	 * (stage 2). Default 3 = top of the OSD band. GXL/GXM have a fixed blend
	 * stack -> keep zpos immutable there. */
	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A))
		drm_plane_create_zpos_property(plane, 3, 0, 3);
	else
		drm_plane_create_zpos_immutable_property(plane, 2);

	return 0;
}
