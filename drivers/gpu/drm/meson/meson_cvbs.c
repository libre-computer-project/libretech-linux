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

#include "meson_drv.h"
#include "meson_venc.h"
#include "meson_venc_cvbs.h"

struct meson_cvbs {
	struct drm_connector	connector;
	struct meson_drm	*priv;
	struct meson_cvbs_mode	*mode;
};
#define connector_to_meson_cvbs(x) \
	container_of(x, struct meson_cvbs, connector)

/* Connector */

static void meson_cvbs_connector_destroy(struct drm_connector *connector)
{
	drm_connector_cleanup(connector);
}

static enum drm_connector_status
meson_cvbs_connector_detect(struct drm_connector *connector, bool force)
{
	/* FIXME: Add load-detect of jack-detect if possible */
	return connector_status_connected;
}

static int meson_cvbs_connector_get_modes(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;
	struct drm_display_mode *mode;
	int i;

	for (i = 0; i < MESON_CVBS_MODES_COUNT; ++i) {
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
	/* Validate the modes added in get_modes */
	return MODE_OK;
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
	int ret;

	meson_cvbs = devm_kzalloc(dev, sizeof(*meson_cvbs), GFP_KERNEL);
	if (!meson_cvbs)
		return -ENOMEM;

	meson_cvbs->priv = priv;
	connector = &meson_cvbs->connector;

	/* Connector */

	drm_connector_helper_add(connector,
				 &meson_cvbs_connector_helper_funcs);

	ret = drm_connector_init(drm, connector, &meson_cvbs_connector_funcs,
				 DRM_MODE_CONNECTOR_Composite);
	if (ret) {
		dev_err(dev, "Failed to init CVBS connector\n");
		return ret;
	}

	connector->interlace_allowed = 1;

	drm_mode_connector_attach_encoder(connector, priv->cvbs_encoder);

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
	{ .compatible = "amlogic,meson-gx-cvbs" },
	{ .compatible = "amlogic,meson-gxl-cvbs" },
	{ .compatible = "amlogic,meson-gxm-cvbs" },
	{ .compatible = "amlogic,meson-gxbb-cvbs" },
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
