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
#include <linux/component.h>

#include <drm/drmP.h>
#include <drm/drm_edid.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_atomic_helper.h>

#include "meson_cvbs.h"
#include "meson_venc.h"

struct meson_cvbs_mode {
	struct meson_cvbs_enci_mode *enci;
	struct drm_display_mode mode;
};
struct meson_cvbs {
	struct drm_connector	connector;
	struct drm_encoder	encoder;
	struct meson_drm	*priv;
	struct meson_cvbs_mode	*mode;
};
#define connector_to_meson_cvbs(x) \
	container_of(x, struct meson_cvbs, connector)
#define encoder_to_meson_cvbs(x) \
	container_of(x, struct meson_cvbs, encoder)

/* Supported Modes */

struct meson_cvbs_mode meson_cvbs_modes[] = {
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

static void meson_cvbs_encoder_destroy(struct drm_encoder *encoder)
{
	drm_encoder_cleanup(encoder);
}

static const struct drm_encoder_funcs meson_cvbs_encoder_funcs = {
	.destroy        = meson_cvbs_encoder_destroy,
};

static int meson_cvbs_encoder_atomic_check(struct drm_encoder *encoder,
					struct drm_crtc_state *crtc_state,
					struct drm_connector_state *conn_state)
{
	return 0;
}

static void meson_cvbs_encoder_disable(struct drm_encoder *encoder)
{
	struct meson_cvbs *meson_cvbs = encoder_to_meson_cvbs(encoder);

	meson_venci_cvbs_disable(meson_cvbs->priv);
}

static void meson_cvbs_encoder_enable(struct drm_encoder *encoder)
{
	struct meson_cvbs *meson_cvbs = encoder_to_meson_cvbs(encoder);

	meson_venci_cvbs_enable(meson_cvbs->priv);
}

static void meson_cvbs_encoder_mode_set(struct drm_encoder *encoder,
				   struct drm_display_mode *mode,
				   struct drm_display_mode *adjusted_mode)
{
	struct meson_cvbs *meson_cvbs = encoder_to_meson_cvbs(encoder);
	int i;

	drm_mode_debug_printmodeline(mode);

	for (i = 0; i < ARRAY_SIZE(meson_cvbs_modes); ++i) {
		struct meson_cvbs_mode *meson_mode = &meson_cvbs_modes[i];

		if (drm_mode_equal(mode, &meson_mode->mode)) {
			meson_cvbs->mode = meson_mode;

			meson_venci_cvbs_mode_set(meson_cvbs->priv,
						  meson_mode->enci);
			break;
		}
	}
}

static const struct drm_encoder_helper_funcs meson_cvbs_encoder_helper_funcs = {
	.atomic_check	= meson_cvbs_encoder_atomic_check,
	.disable	= meson_cvbs_encoder_disable,
	.enable		= meson_cvbs_encoder_enable,
	.mode_set	= meson_cvbs_encoder_mode_set,
};

/* Connector */

static void meson_cvbs_connector_destroy(struct drm_connector *connector)
{
	drm_connector_cleanup(connector);
}

static enum drm_connector_status
meson_cvbs_connector_detect(struct drm_connector *connector, bool force)
{
	return connector_status_connected;
}

static int meson_cvbs_connector_get_modes(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;
	struct drm_display_mode *mode;
	int i;

	for (i = 0; i < ARRAY_SIZE(meson_cvbs_modes); ++i) {
		struct meson_cvbs_mode *meson_mode = &meson_cvbs_modes[i];

		mode = drm_mode_duplicate(dev, &meson_mode->mode);
		if (!mode) {
			DRM_ERROR("Failed to create a new display mode\n");
			return 0;
		}

		drm_mode_probed_add(connector, mode);
	}

	return i;
}

static int meson_cvbs_connector_mode_valid(struct drm_connector *connector,
					   struct drm_display_mode *mode)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(meson_cvbs_modes); ++i) {
		struct meson_cvbs_mode *meson_mode = &meson_cvbs_modes[i];

		if (drm_mode_equal(mode, &meson_mode->mode))
			return MODE_OK;
	}

	return MODE_BAD;
}

static const struct drm_connector_funcs meson_cvbs_connector_funcs = {
	.dpms			= drm_atomic_helper_connector_dpms,
	.detect			= meson_cvbs_connector_detect,
	.fill_modes		= drm_helper_probe_single_connector_modes,
	.destroy		= meson_cvbs_connector_destroy,
	.reset			= drm_atomic_helper_connector_reset,
	.atomic_duplicate_state	= drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_connector_destroy_state,
};

static const
struct drm_connector_helper_funcs meson_cvbs_connector_helper_funcs = {
	.get_modes	= meson_cvbs_connector_get_modes,
	.mode_valid	= meson_cvbs_connector_mode_valid,
};

static int meson_cvbs_bind(struct device *dev, struct device *master,
			   void *data)
{
	struct drm_device *drm = data;
	struct meson_drm *priv = drm->dev_private;
	struct meson_cvbs *meson_cvbs;
	struct drm_connector *connector;
	struct drm_encoder *encoder;
	int ret;

	meson_cvbs = devm_kzalloc(dev, sizeof(*meson_cvbs), GFP_KERNEL);
	if (!meson_cvbs)
		return -ENOMEM;

	meson_cvbs->priv = priv;
	connector = &meson_cvbs->connector;
	encoder = &meson_cvbs->encoder;

	/* Encoder */

	drm_encoder_helper_add(encoder, &meson_cvbs_encoder_helper_funcs);

	ret = drm_encoder_init(drm, encoder, &meson_cvbs_encoder_funcs,
			       DRM_MODE_ENCODER_TVDAC, "meson_cvbs");
	if (ret) {
		dev_err(dev, "Failed to init CVBS encoder\n");
		return ret;
	}

	encoder->possible_crtcs = BIT(0);

	/* Connector */

	drm_connector_helper_add(connector,
				 &meson_cvbs_connector_helper_funcs);

	drm_connector_init(drm, connector, &meson_cvbs_connector_funcs,
			   DRM_MODE_CONNECTOR_Composite);
	if (ret) {
		dev_err(dev, "Failed to init CVBS connector\n");
		return ret;
	}

	connector->interlace_allowed = 1;

	drm_mode_connector_attach_encoder(connector, encoder);

	return 0;
}

static void meson_cvbs_unbind(struct device *dev, struct device *master,
			      void *data)
{
	/* Nothing to do yet */
}

static const struct component_ops meson_cvbs_ops = {
	.bind	= meson_cvbs_bind,
	.unbind	= meson_cvbs_unbind,
};

static int meson_cvbs_probe(struct platform_device *pdev)
{
	return component_add(&pdev->dev, &meson_cvbs_ops);
}

static int meson_cvbs_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &meson_cvbs_ops);

	return 0;
}

static const struct of_device_id meson_cvbs_of_table[] = {
	{ .compatible = "amlogic,meson-gx-venc-cvbs" },
	{ .compatible = "amlogic,meson-gxl-venc-cvbs" },
	{ .compatible = "amlogic,meson-gxm-venc-cvbs" },
	{ .compatible = "amlogic,meson-gxbb-venc-cvbs" },
	{ }
};
MODULE_DEVICE_TABLE(of, meson_cvbs_of_table);

static struct platform_driver meson_cvbs_platform_driver = {
	.probe		= meson_cvbs_probe,
	.remove		= meson_cvbs_remove,
	.driver		= {
		.name		= "meson-cvbs",
		.of_match_table	= meson_cvbs_of_table,
	},
};
module_platform_driver(meson_cvbs_platform_driver);
