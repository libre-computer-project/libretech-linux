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
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 *
 * Written by:
 *     Jasper St. Pierre <jstpierre@mecheye.net>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/component.h>

#include <drm/drmP.h>
#include <drm/drm_edid.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_atomic_helper.h>

#include "meson_venc_cvbs.h"
#include "meson_venc.h"
#include "meson_registers.h"

/* HHI VDAC Registers */
#define HHI_VDAC_CNTL0		0x2F4 /* 0xbd offset in data sheet */
#define HHI_VDAC_CNTL1		0x2F8 /* 0xbe offset in data sheet */

struct meson_venc_cvbs {
	struct drm_encoder	encoder;
	struct meson_drm	*priv;
};
#define encoder_to_meson_venc_cvbs(x) \
	container_of(x, struct meson_venc_cvbs, encoder)

/* Supported Modes */

struct meson_cvbs_mode meson_cvbs_modes[MESON_CVBS_MODES_COUNT] = {
	{ /* PAL */
		.enci = &meson_cvbs_enci_pal,
		.mode = {
			DRM_MODE("720x576i", DRM_MODE_TYPE_DRIVER, 13500,
				 720, 732, 795, 864, 0, 576, 580, 586, 625, 0,
				 DRM_MODE_FLAG_INTERLACE),
			.vrefresh = 50,
			.picture_aspect_ratio = HDMI_PICTURE_ASPECT_4_3,
		},
	},
	{ /* NTSC */
		.enci = &meson_cvbs_enci_ntsc,
		.mode = {
			DRM_MODE("720x480i", DRM_MODE_TYPE_DRIVER, 13500,
				720, 739, 801, 858, 0, 480, 488, 494, 525, 0,
				DRM_MODE_FLAG_INTERLACE),
			.vrefresh = 60,
			.picture_aspect_ratio = HDMI_PICTURE_ASPECT_4_3,
		},
	},
};

/* Encoder */

static void meson_venc_cvbs_encoder_destroy(struct drm_encoder *encoder)
{
	drm_encoder_cleanup(encoder);
}

static const struct drm_encoder_funcs meson_venc_cvbs_encoder_funcs = {
	.destroy        = meson_venc_cvbs_encoder_destroy,
};

static int meson_venc_cvbs_encoder_atomic_check(struct drm_encoder *encoder,
					struct drm_crtc_state *crtc_state,
					struct drm_connector_state *conn_state)
{
	int i;

	for (i = 0; i < MESON_CVBS_MODES_COUNT; ++i) {
		struct meson_cvbs_mode *meson_mode = &meson_cvbs_modes[i];

		if (drm_mode_equal(&crtc_state->mode, &meson_mode->mode))
			return 0;
	}

	return -EINVAL;
}

static void meson_venc_cvbs_encoder_disable(struct drm_encoder *encoder)
{
	struct meson_venc_cvbs *meson_venc_cvbs =
					encoder_to_meson_venc_cvbs(encoder);
	struct meson_drm *priv = meson_venc_cvbs->priv;

	/* Disable CVBS VDAC */
	regmap_write(priv->hhi, HHI_VDAC_CNTL0, 0);
	regmap_write(priv->hhi, HHI_VDAC_CNTL1, 0);
}

static void meson_venc_cvbs_encoder_enable(struct drm_encoder *encoder)
{
	struct meson_venc_cvbs *meson_venc_cvbs =
					encoder_to_meson_venc_cvbs(encoder);
	struct meson_drm *priv = meson_venc_cvbs->priv;

	/* VDAC0 source is not from ATV */
	writel_bits_relaxed(BIT(5), 0, priv->io_base + _REG(VENC_VDAC_DACSEL0));

	if (of_machine_is_compatible("amlogic,meson-gxbb"))
		regmap_write(priv->hhi, HHI_VDAC_CNTL0, 1);
	else if (of_machine_is_compatible("amlogic,meson-gxm") ||
		   of_machine_is_compatible("amlogic,meson-gxl"))
		regmap_write(priv->hhi, HHI_VDAC_CNTL0, 0xf0001);

	regmap_write(priv->hhi, HHI_VDAC_CNTL1, 0);
}

static void meson_venc_cvbs_encoder_mode_set(struct drm_encoder *encoder,
				   struct drm_display_mode *mode,
				   struct drm_display_mode *adjusted_mode)
{
	struct meson_venc_cvbs *meson_venc_cvbs =
					encoder_to_meson_venc_cvbs(encoder);
	int i;

	for (i = 0; i < MESON_CVBS_MODES_COUNT; ++i) {
		struct meson_cvbs_mode *meson_mode = &meson_cvbs_modes[i];

		if (drm_mode_equal(mode, &meson_mode->mode)) {
			meson_venci_cvbs_mode_set(meson_venc_cvbs->priv,
						  meson_mode->enci);
			break;
		}
	}
}

static const struct drm_encoder_helper_funcs
				meson_venc_cvbs_encoder_helper_funcs = {
	.atomic_check	= meson_venc_cvbs_encoder_atomic_check,
	.disable	= meson_venc_cvbs_encoder_disable,
	.enable		= meson_venc_cvbs_encoder_enable,
	.mode_set	= meson_venc_cvbs_encoder_mode_set,
};

int meson_venc_cvbs_create(struct meson_drm *priv)
{
	struct drm_device *drm = priv->drm;
	struct meson_venc_cvbs *meson_venc_cvbs;
	struct drm_encoder *encoder;
	int ret;

	meson_venc_cvbs = devm_kzalloc(priv->dev, sizeof(*meson_venc_cvbs),
				       GFP_KERNEL);
	if (!meson_venc_cvbs)
		return -ENOMEM;

	meson_venc_cvbs->priv = priv;
	encoder = &meson_venc_cvbs->encoder;

	/* Encoder */

	drm_encoder_helper_add(encoder, &meson_venc_cvbs_encoder_helper_funcs);

	ret = drm_encoder_init(drm, encoder, &meson_venc_cvbs_encoder_funcs,
			       DRM_MODE_ENCODER_TVDAC, "meson_venc_cvbs");
	if (ret) {
		dev_err(priv->dev, "Failed to init CVBS encoder\n");
		return ret;
	}

	encoder->possible_crtcs = BIT(0);

	priv->cvbs_encoder = encoder;

	return 0;
}
