/*
 * Copyright (C) 2016 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
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
 *
 * Written by:
 *     Jasper St. Pierre <jstpierre@mecheye.net>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <drm/drmP.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_rect.h>

#include "meson_plane.h"
#include "meson_vpp.h"
#include "meson_viu.h"

struct meson_plane {
	struct drm_plane base;
	struct meson_drm *priv;
#if 0
	struct osd_plane_def *def;

	/* These are shadow registers that are updated
	 * at vblank time. The various atomic_commit
	 * functions set these and we copy them into the
	 * real set of mapped registers at runtime. */
	struct osd_plane_registers reg;

	enum meson_interlacing_strategy interlacing_strategy;

	bool fb_changed;
	bool visible;
#endif
};
#define to_meson_plane(x) container_of(x, struct meson_plane, base)

#if 0
/* Canvas configuration. */

enum meson_canvas_wrap {
	MESON_CANVAS_WRAP_NONE = 0x00,
	MESON_CANVAS_WRAP_X    = 0x01,
	MESON_CANVAS_WRAP_Y    = 0x02,
};

enum meson_canvas_blkmode {
	MESON_CANVAS_BLKMODE_LINEAR = 0x00,
	MESON_CANVAS_BLKMODE_32x32  = 0x01,
	MESON_CANVAS_BLKMODE_64x64  = 0x02,
};

/* Set up a canvas. */
static void canvas_setup(uint32_t canvas_index,
			 uint32_t addr,
			 uint32_t stride, uint32_t height,
			 enum meson_canvas_wrap wrap,
			 enum meson_canvas_blkmode blkmode)
{
	CANVAS_WRITE(DC_CAV_LUT_DATAL,
		     (((addr + 7) >> 3)) |
		     (((stride + 7) >> 3) << CANVAS_WIDTH_LBIT));
	CANVAS_WRITE(DC_CAV_LUT_DATAH,
		     ((((stride + 7) >> 3) >> CANVAS_WIDTH_LWID) << CANVAS_WIDTH_HBIT) |
		     (height << CANVAS_HEIGHT_BIT) |
		     (wrap << 22) |
		     (blkmode << CANVAS_BLKMODE_BIT));
	CANVAS_WRITE(DC_CAV_LUT_ADDR, CANVAS_LUT_WR_EN | canvas_index);

	/* Force a read-back to make sure everything is flushed. */
	CANVAS_READ(DC_CAV_LUT_DATAH);
}


enum osd_w0_bitflags {
	OSD_ENDIANNESS_BE = (0x00 << 15),
	OSD_ENDIANNESS_LE = (0x01 << 15),

	OSD_BLK_MODE_422 = (0x03 << 8),
	OSD_BLK_MODE_16  = (0x04 << 8),
	OSD_BLK_MODE_32  = (0x05 << 8),
	OSD_BLK_MODE_24  = (0x07 << 8),

	OSD_OUTPUT_COLOR_YUV = (0x00 << 7),
	OSD_OUTPUT_COLOR_RGB = (0x01 << 7),

	OSD_COLOR_MATRIX_32_RGBA = (0x00 << 2),
	OSD_COLOR_MATRIX_32_ARGB = (0x01 << 2),
	OSD_COLOR_MATRIX_32_ABGR = (0x02 << 2),
	OSD_COLOR_MATRIX_32_BGRA = (0x03 << 2),

	OSD_INTERLACE_ENABLED  = (0x01 << 1),
	OSD_INTERLACE_ODD      = (0x01 << 0),
	OSD_INTERLACE_EVEN     = (0x00 << 0),
};

/* Dumb metaprogramming, should replace with something better. */
#define OSD_REGISTERS				\
	M(CTRL_STAT)				\
	M(BLK0_CFG_W0)				\
	M(BLK0_CFG_W1)				\
	M(BLK0_CFG_W2)				\
	M(BLK0_CFG_W3)				\
	M(BLK0_CFG_W4)

struct osd_plane_def {
	bool uses_scaler;
	bool compensate_for_scaler;

	uint32_t canvas_index;

	uint32_t vpp_misc_postblend;

	struct {
#define M(n) uint32_t n;
OSD_REGISTERS
#undef M
	} reg;
};

struct osd_plane_registers {
#define M(n) uint32_t n;
OSD_REGISTERS
#undef M
};

enum meson_interlacing_strategy {
	/* We don't require interlacing -- scan as progressive. */
	MESON_INTERLACING_STRATEGY_NONE,

	/* We are interlacing out this plane using the OSD interlacer. */
	MESON_INTERLACING_STRATEGY_OSD,

	/* We are interlacing out this plane using the HW scaler, so
	 * scan this out as progressive. */
	MESON_INTERLACING_STRATEGY_SCALER,
};

/* For CVBS mode, add a fixed underscan border that is
 * 7.5% of each display dimension */
#define CVBS_UNDERSCAN_MANGLE(x) ((x) * 15 / 200)

static void get_underscan_border(struct drm_plane_state *state,
				 int *hborder_p, int *vborder_p)
{
	struct meson_crtc *meson_crtc = to_meson_crtc(state->crtc);
	int hborder = 0;
	int vborder = 0;

	if (meson_crtc->underscan_type == UNDERSCAN_ON) {
		hborder += meson_crtc->underscan_hborder;
		vborder += meson_crtc->underscan_vborder;
	}

	/* If we're on a CVBS mode, add in some constant underscan borders. */

	/* XXX: We're detecting CVBS through interlaced vs. not, but
	 * HDMI modes can be interlaced too! */
	if (state->crtc->mode.flags & DRM_MODE_FLAG_INTERLACE) {
		hborder += CVBS_UNDERSCAN_MANGLE(state->crtc_w);
		vborder += CVBS_UNDERSCAN_MANGLE(state->crtc_h);
	}

	*hborder_p = hborder;
	*vborder_p = vborder;
}

static bool get_scaler_rects(struct drm_crtc *crtc,
			     struct drm_rect *input,
			     struct drm_rect *output)
{
	struct drm_plane *plane = crtc->primary;
	struct drm_plane_state *state = plane->state;
	int hborder, vborder;

	input->x1 = 0;
	input->y1 = 0;
	input->x2 = state->crtc_w;
	input->y2 = state->crtc_h;

	*output = *input;
#if 0
	get_underscan_border(state, &hborder, &vborder);

	output->x1 += hborder;
	output->x2 -= hborder;
	output->y1 += vborder;
	output->y2 -= vborder;
#endif

	return (!drm_rect_equals(input, output));
}

/* Scales from the range a1..a2 to b1..b2 */
static inline int scale_into(int v, int a1, int a2, int b1, int b2)
{
	return ((v - a1) * (b2 - b1) / (a2 - a1)) + b1;
}

static inline void scale_rect_into(struct drm_rect *dest,
				   struct drm_rect *input,
				   struct drm_rect *output)
{
	int offs;

	offs = scale_into(dest->x1, input->x1, input->x2, output->x1, output->x2) - dest->x1;
	dest->x1 += offs;
	dest->x2 += offs;

	offs = scale_into(dest->y1, input->y1, input->y2, output->y1, output->y2) - dest->y1;
	dest->y1 += offs;
	dest->y2 += offs;
}
#endif

static int meson_plane_atomic_check(struct drm_plane *plane,
				    struct drm_plane_state *state)
{
	pr_info("%s:%s\n", __FILE__, __func__);
#if 0
	struct drm_rect src = {
		.x1 = state->src_x,
		.y1 = state->src_y,
		.x2 = state->src_x + state->src_w,
		.y2 = state->src_y + state->src_h,
	};
	struct drm_rect dest = {
		.x1 = state->crtc_x,
		.y1 = state->crtc_y,
		.x2 = state->crtc_x + state->crtc_w,
		.y2 = state->crtc_y + state->crtc_h,
	};

	if (state->fb) {
		int ret;

		ret = drm_rect_calc_hscale(&src, &dest, DRM_PLANE_HELPER_NO_SCALING, DRM_PLANE_HELPER_NO_SCALING);
		if (ret < 0)
			return ret;

		ret = drm_rect_calc_vscale(&src, &dest, DRM_PLANE_HELPER_NO_SCALING, DRM_PLANE_HELPER_NO_SCALING);
		if (ret < 0)
			return ret;
	}
#endif

	return 0;
}

#if 0
/* Takes a fixed 16.16 number and converts it to integer. */
static inline int64_t fixed16_to_int(int64_t value)
{
	return value >> 16;
}
#endif

static void meson_plane_atomic_update(struct drm_plane *plane,
				      struct drm_plane_state *old_state)
{
	struct meson_plane *meson_plane = to_meson_plane(plane);

	pr_info("%s:%s\n", __FILE__, __func__);
	/*
	 * Update Coordinates
	 * Update Formats
	 * Update Buffer
	 * Enable Plane
	 */

	meson_viu_update_osd1(meson_plane->priv, plane);
	meson_vpp_enable_osd1(meson_plane->priv);
#if 0
	struct meson_plane *meson_plane = to_meson_plane(plane);
	struct drm_plane_state *state = plane->state;
	struct meson_crtc *meson_crtc = to_meson_crtc(state->crtc);
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
	struct drm_rect clip = {};
	bool is_scaling;
	unsigned long flags;

	if (state->fb) {
		struct drm_rect input, output;

		is_scaling = get_scaler_rects(state->crtc, &input, &output);

		if (meson_plane->def->compensate_for_scaler && is_scaling)
			scale_rect_into(&dest, &input, &output);

		if (meson_plane->def->uses_scaler)
			clip = input;
		else
			clip = output;

		if (state->crtc->mode.flags & DRM_MODE_FLAG_INTERLACE) {
			clip.y1 /= 2;
			clip.y2 /= 2;

			dest.y1 /= 2;
			dest.y2 /= 2;
		}

		meson_plane->visible = drm_rect_clip_scaled(&src, &dest, &clip,
							    DRM_PLANE_HELPER_NO_SCALING,
							    DRM_PLANE_HELPER_NO_SCALING);
	} else {
		meson_plane->visible = false;
	}

	if (meson_plane->visible) {
		/* If we're interlacing, then figure out what strategy we're
		 * going to use. */
		if (state->crtc->mode.flags & DRM_MODE_FLAG_INTERLACE) {
			/* If this plane is going to use the vertical scaler, then scan it out as
			 * progressive, as the scaler will take care of interlacing for us. */
			if (meson_plane->def->uses_scaler && is_scaling)
				meson_plane->interlacing_strategy = MESON_INTERLACING_STRATEGY_SCALER;
			else
				meson_plane->interlacing_strategy = MESON_INTERLACING_STRATEGY_OSD;
		} else {
			meson_plane->interlacing_strategy = MESON_INTERLACING_STRATEGY_NONE;
		}

		if (state->fb != old_state->fb)
			meson_plane->fb_changed = true;

		spin_lock_irqsave(&meson_crtc->irq_lock, flags);
		/* Enable OSD and BLK0. */
		meson_plane->reg.CTRL_STAT = ((1 << 21) |    /* Enable OSD */
					      (0xFF << 12) | /* Alpha is 0xFF */
					      (1 << 0)       /* Enable BLK0 */);

		/* Set up BLK0 to point to the right canvas */
		meson_plane->reg.BLK0_CFG_W0 = ((meson_plane->def->canvas_index << 16) |
						OSD_ENDIANNESS_LE | OSD_BLK_MODE_32 | OSD_OUTPUT_COLOR_RGB | OSD_COLOR_MATRIX_32_ARGB);

		if (meson_plane->interlacing_strategy == MESON_INTERLACING_STRATEGY_OSD)
			meson_plane->reg.BLK0_CFG_W0 |= OSD_INTERLACE_ENABLED;

		/* The format of these registers is (x2 << 16 | x1), where x2 is exclusive.
		 * e.g. +30x1920 would be (1949 << 16) | 30. */
		meson_plane->reg.BLK0_CFG_W1 = ((fixed16_to_int(src.x2) - 1) << 16) | fixed16_to_int(src.x1);
		meson_plane->reg.BLK0_CFG_W2 = ((fixed16_to_int(src.y2) - 1) << 16) | fixed16_to_int(src.y1);
		meson_plane->reg.BLK0_CFG_W3 = ((dest.x2 - 1) << 16) | dest.x1;
		meson_plane->reg.BLK0_CFG_W4 = ((dest.y2 - 1) << 16) | dest.y1;
		spin_unlock_irqrestore(&meson_crtc->irq_lock, flags);
	}
#endif
}

static void meson_plane_atomic_disable(struct drm_plane *plane,
				       struct drm_plane_state *old_state)
{
	struct meson_plane *meson_plane = to_meson_plane(plane);

	meson_vpp_disable_osd1(meson_plane->priv);
}

static const struct drm_plane_helper_funcs meson_plane_helper_funcs = {
	.atomic_check	= meson_plane_atomic_check,
	.atomic_disable	= meson_plane_atomic_disable,
	.atomic_update	= meson_plane_atomic_update,
};

static const struct drm_plane_funcs meson_plane_funcs = {
	.update_plane		= drm_atomic_helper_update_plane,
	.disable_plane		= drm_atomic_helper_disable_plane,
	.destroy		= drm_plane_cleanup,
	.reset			= drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_plane_destroy_state,
};

static const uint32_t supported_drm_formats[] = {
	DRM_FORMAT_ARGB8888,
};

#if 0
/* Pick two canvases in the "user canvas" space that aren't
 * likely to compete. */
static struct osd_plane_def osd_plane_defs[] = {
	{
		.uses_scaler = true,
		.compensate_for_scaler = false,
		.canvas_index = 0x4e,
#if 0
		.vpp_misc_postblend = VPP_OSD1_POSTBLEND,
		{
#define M(n) .n = P_VIU_OSD1_##n ,
			OSD_REGISTERS
#undef M
		}
#endif
	},
	{
		.uses_scaler = false,
		.compensate_for_scaler = true,
		.canvas_index = 0x4f,
#if 0
		.vpp_misc_postblend = VPP_OSD2_POSTBLEND,
		{
#define M(n) .n = P_VIU_OSD2_##n ,
			OSD_REGISTERS
#undef M
		}
#endif
	},
};
#endif

#if 0
void meson_plane_update_interlaced_field(struct drm_crtc *crtc)
{
	pr_info("%s:%s\n", __FILE__, __func__);
	struct meson_plane *meson_plane = to_meson_plane(crtc->primary);

	if (meson_plane->interlacing_strategy != MESON_INTERLACING_STRATEGY_NONE) {
		int field = aml_read_reg32(P_ENCI_INFO_READ) & (1 << 29);
		meson_plane->reg.BLK0_CFG_W0 = ((meson_plane->reg.BLK0_CFG_W0 & ~0x01) |
						(field ? OSD_INTERLACE_ODD : OSD_INTERLACE_EVEN));
	}
}

void meson_plane_update_shadow_registers(struct drm_crtc *crtc)
{
	pr_info("%s:%s\n", __FILE__, __func__);
	struct meson_plane *meson_plane = to_meson_plane(crtc->primary);
	struct drm_plane_state *state = plane->state;

	if (meson_plane->visible) {
		if (meson_plane->fb_changed) {
			struct meson_drm_gem_object *bo;

			bo = meson_drm_get_gem_obj(state->fb, 0);

			/* Swap out the OSD canvas with the new addr. */
			canvas_setup(meson_plane->def->canvas_index,
				     bo->paddr,
				     state->fb->pitches[0],
				     state->fb->height,
				     MESON_CANVAS_WRAP_NONE,
				     MESON_CANVAS_BLKMODE_LINEAR);

			meson_plane->fb_changed = false;
		}

		aml_set_reg32_mask(P_VPP_MISC, meson_plane->def->vpp_misc_postblend);

		/* Copy the shadow registers into the real registers. */
#define M(n) aml_write_reg32(meson_plane->def->reg.n, meson_plane->reg.n);
OSD_REGISTERS
#undef M
	} else {
		aml_clr_reg32_mask(P_VPP_MISC, meson_plane->def->vpp_misc_postblend);
	}
}

void meson_plane_update_scaler(struct drm_crtc *crtc)
{
	pr_info("%s:%s\n", __FILE__, __func__);
	struct meson_plane *meson_plane = to_meson_plane(crtc->primary);
	struct drm_plane_state *state = crtc->primary->state;
	struct drm_rect input, output;
get_scaler_rects
	if (!meson_plane->visible)
		return;

	if (!state)
		return;

	if (get_scaler_rects(crtc, &input, &output)) {
		bool interlace = (meson_plane->interlacing_strategy == MESON_INTERLACING_STRATEGY_SCALER);

		if (interlace) {
			output.y1 /= 2;
			output.y2 /= 2;
		}

		/* Basic scaler config */
		aml_write_reg32(P_VPP_OSD_SC_CTRL0,
				(1 << 3) /* Enable scaler */ |
				(0 << 2) /* Select OSD1 */);
		aml_write_reg32(P_VPP_OSD_SCI_WH_M1,
				((drm_rect_width(&input) - 1) << 16) | (drm_rect_height(&input) - 1));
		aml_write_reg32(P_VPP_OSD_SCO_H_START_END, ((output.x1) << 16) | (output.x2));
		aml_write_reg32(P_VPP_OSD_SCO_V_START_END, ((output.y1) << 16) | (output.y2));

		/* HSC */
		if (input.x1 != output.x1 || input.x2 != output.x2) {
			int hf_phase_step = ((drm_rect_width(&input) << 18) / drm_rect_width(&output));
			aml_write_reg32(P_VPP_OSD_HSC_PHASE_STEP, hf_phase_step << 6);

			aml_write_reg32(P_VPP_OSD_HSC_CTRL0,
					(4 << 0) /* osd_hsc_bank_length */ |
					(4 << 3) /* osd_hsc_ini_rcv_num0 */ |
					(1 << 8) /* osd_hsc_rpt_p0_num0 */ |
					(1 << 22) /* Enable horizontal scaler */);
		} else {
			aml_write_reg32(P_VPP_OSD_HSC_CTRL0, 0);
		}

		/* VSC */
		if (input.y1 != output.y1 || input.y2 != output.y2) {
			int vf_phase_step = ((drm_rect_height(&input) << 20) / drm_rect_height(&output));

			aml_write_reg32(P_VPP_OSD_VSC_INI_PHASE, interlace ? (vf_phase_step >> 5) : 0);
			aml_write_reg32(P_VPP_OSD_VSC_PHASE_STEP, vf_phase_step << 4);

			aml_write_reg32(P_VPP_OSD_VSC_CTRL0,
					(4 << 0) /* osd_vsc_bank_length */ |
					(4 << 3) /* osd_vsc_top_ini_rcv_num0 */ |
					(1 << 8) /* osd_vsc_top_rpt_p0_num0 */ |
					(6 << 11) /* osd_vsc_bot_ini_rcv_num0 */ |
					(2 << 16) /* osd_vsc_bot_rpt_p0_num0 */ |
					((interlace ? 1 : 0) << 23) /* osd_prog_interlace */ |
					(1 << 24) /* Enable vertical scaler */);
		} else {
			aml_write_reg32(P_VPP_OSD_VSC_CTRL0, 0);
		}
	} else {
		aml_write_reg32(P_VPP_OSD_SC_CTRL0,
				(0 << 3) /* Disable scaler */);
	}
}
#endif

int meson_plane_create(struct meson_drm *priv)
{
	struct meson_plane *meson_plane;
	struct drm_plane *plane;

	pr_info("%s:%s\n", __FILE__, __func__);

	meson_plane = devm_kzalloc(priv->drm->dev, sizeof(*meson_plane),
				   GFP_KERNEL);
	if (!meson_plane)
		return -ENOMEM;

	meson_plane->priv = priv;
	plane = &meson_plane->base;

	//meson_plane->def = osd_plane_def;

	drm_universal_plane_init(priv->drm, plane, 0xFF,
				 &meson_plane_funcs,
				 supported_drm_formats,
				 ARRAY_SIZE(supported_drm_formats),
				 DRM_PLANE_TYPE_PRIMARY, "meson_primary_plane");

	drm_plane_helper_add(plane, &meson_plane_helper_funcs);

	priv->primary_plane = plane;

	return 0;
}
