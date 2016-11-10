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
#include "meson_canvas.h"

struct meson_plane {
	struct drm_plane base;
	struct meson_drm *priv;
};
#define to_meson_plane(x) container_of(x, struct meson_plane, base)

static int meson_plane_atomic_check(struct drm_plane *plane,
				    struct drm_plane_state *state)
{
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

		ret = drm_rect_calc_hscale(&src, &dest,
					   DRM_PLANE_HELPER_NO_SCALING,
					   DRM_PLANE_HELPER_NO_SCALING);
		if (ret < 0)
			return ret;

		ret = drm_rect_calc_vscale(&src, &dest,
					   DRM_PLANE_HELPER_NO_SCALING,
					   DRM_PLANE_HELPER_NO_SCALING);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static void meson_plane_atomic_update(struct drm_plane *plane,
				      struct drm_plane_state *old_state)
{
	struct meson_plane *meson_plane = to_meson_plane(plane);

	/*
	 * Update Coordinates
	 * Update Formats
	 * Update Buffer
	 * Enable Plane
	 */
	meson_viu_update_osd1(meson_plane->priv, plane);
	meson_canvas_update_osd1_buffer(meson_plane->priv, plane);
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
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_RGB888,
	DRM_FORMAT_RGB565,
};

int meson_plane_create(struct meson_drm *priv)
{
	struct meson_plane *meson_plane;
	struct drm_plane *plane;

	meson_plane = devm_kzalloc(priv->drm->dev, sizeof(*meson_plane),
				   GFP_KERNEL);
	if (!meson_plane)
		return -ENOMEM;

	meson_plane->priv = priv;
	plane = &meson_plane->base;

	drm_universal_plane_init(priv->drm, plane, 0xFF,
				 &meson_plane_funcs,
				 supported_drm_formats,
				 ARRAY_SIZE(supported_drm_formats),
				 DRM_PLANE_TYPE_PRIMARY, "meson_primary_plane");

	drm_plane_helper_add(plane, &meson_plane_helper_funcs);

	priv->primary_plane = plane;

	return 0;
}
