/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2016 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 * Copyright (C) 2014 Endless Mobile
 *
 * Written by:
 *     Jasper St. Pierre <jstpierre@mecheye.net>
 */

#ifndef __MESON_CRTC_H
#define __MESON_CRTC_H

#include "meson_drv.h"

int meson_crtc_create(struct meson_drm *priv);

void meson_crtc_irq(struct meson_drm *priv);

/*
 * Publish the connected sink's FreeSync refresh range (Hz) to the CRTC, which
 * clamps its per-frame VRR line count to it. A zero or degenerate range means
 * "no usable VRR range" and makes the CRTC refuse VRR_ENABLED outright.
 */
void meson_crtc_set_vrr_range(struct drm_crtc *crtc, u16 min_vfreq,
			      u16 max_vfreq);

#endif /* __MESON_CRTC_H */
