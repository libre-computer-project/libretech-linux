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
#include <drm/drmP.h>
#include <drm/drm_edid.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_atomic_helper.h>

#include "meson_cvbs.h"
#include "meson_venc.h"

struct meson_cvbs {
	struct drm_connector	connector;
	struct drm_encoder	encoder;
	struct meson_drm	*priv;
};
#define connector_to_meson_cvbs(x) \
	container_of(x, struct meson_cvbs, connector)
#define encoder_to_meson_cvbs(x) \
	container_of(x, struct meson_cvbs, encoder)

/* Encoder */

static void meson_cvbs_encoder_destroy(struct drm_encoder *encoder)
{
	pr_info("%s:%s\n", __FILE__, __func__);

	drm_encoder_cleanup(encoder);
}

static const struct drm_encoder_funcs meson_cvbs_encoder_funcs = {
	.destroy        = meson_cvbs_encoder_destroy,
};

static int meson_cvbs_encoder_atomic_check(struct drm_encoder *encoder,
					struct drm_crtc_state *crtc_state,
					struct drm_connector_state *conn_state)
{
	pr_info("%s:%s\n", __FILE__, __func__);

	return 0;
}

static void meson_cvbs_encoder_disable(struct drm_encoder *encoder)
{
	struct meson_cvbs *meson_cvbs = encoder_to_meson_cvbs(encoder);

	pr_info("%s:%s\n", __FILE__, __func__);

	meson_venci_disable(meson_cvbs->priv);
}

static void meson_cvbs_encoder_enable(struct drm_encoder *encoder)
{
	struct meson_cvbs *meson_cvbs = encoder_to_meson_cvbs(encoder);

	pr_info("%s:%s\n", __FILE__, __func__);

	meson_venci_enable(meson_cvbs->priv);
}

static void meson_cvbs_encoder_mode_set(struct drm_encoder *encoder,
				   struct drm_display_mode *mode,
				   struct drm_display_mode *adjusted_mode)
{
	struct meson_cvbs *meson_cvbs = encoder_to_meson_cvbs(encoder);

	pr_info("%s:%s\n", __FILE__, __func__);

	meson_venci_mode_set(meson_cvbs->priv, mode);
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
	pr_info("%s:%s\n", __FILE__, __func__);

	drm_connector_cleanup(connector);
}

static enum drm_connector_status
meson_cvbs_connector_detect(struct drm_connector *connector, bool force)
{
	pr_info("%s:%s\n", __FILE__, __func__);

	return connector_status_connected;
}

static int meson_cvbs_connector_get_modes(struct drm_connector *connector)
{
	struct meson_cvbs *meson_cvbs = connector_to_meson_cvbs(connector);
	struct drm_device *dev = connector->dev;
	struct drm_display_mode *mode;

	pr_info("%s:%s\n", __FILE__, __func__);

	/* PAL */
	mode = drm_cvt_mode(dev, 720, 576, 50, false, true, false);
	mode->type |= DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	strcpy(mode->name, "PAL");

	drm_mode_probed_add(connector, mode);

	return 1;
}

static int meson_cvbs_connector_mode_valid(struct drm_connector *connector,
					   struct drm_display_mode *mode)
{
	pr_info("%s:%s\n", __FILE__, __func__);

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

static const struct drm_connector_helper_funcs meson_cvbs_connector_helper_funcs = {
	.get_modes	= meson_cvbs_connector_get_modes,
	.mode_valid	= meson_cvbs_connector_mode_valid,
};

int meson_cvbs_create(struct meson_drm *priv)
{
	struct meson_cvbs *meson_cvbs;
	struct drm_connector *connector;
        struct drm_encoder *encoder;
	int ret;

	pr_info("%s:%s\n", __FILE__, __func__);

	meson_cvbs = devm_kzalloc(priv->drm->dev, sizeof(*meson_cvbs),
				  GFP_KERNEL);
	if (!meson_cvbs)
		return -ENOMEM;

	meson_cvbs->priv = priv;
	connector = &meson_cvbs->connector;
	encoder = &meson_cvbs->encoder;

	/* Encoder */

	drm_encoder_helper_add(encoder, &meson_cvbs_encoder_helper_funcs);

	ret = drm_encoder_init(priv->drm, encoder, &meson_cvbs_encoder_funcs,
			       DRM_MODE_ENCODER_TVDAC, "meson_cvbs");
	if (ret) {
		dev_err(priv->drm->dev, "Failed to init CVBS encoder\n");
		return ret;
	}

	encoder->possible_crtcs = BIT(0);

	/* Connector */

	drm_connector_helper_add(connector,
				 &meson_cvbs_connector_helper_funcs);

	drm_connector_init(priv->drm, connector, &meson_cvbs_connector_funcs,
			   DRM_MODE_CONNECTOR_Composite);
	if (ret) {
		dev_err(priv->drm->dev, "Failed to init CVBS connector\n");
		return ret;
	}

	connector->interlace_allowed = 1;

	drm_mode_connector_attach_encoder(connector, encoder);

	return 0;
}
