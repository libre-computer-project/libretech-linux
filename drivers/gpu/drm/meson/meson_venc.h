/* 
 * Copyright (C) 2016 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
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

/* 
 * Video Encoders
 * - ENCI : Composite Video Encoder
 * - ENCP : TMDS/HDMI Video Encoder
 */

#ifndef __MESON_VENC_H
#define __MESON_VENC_H

void meson_venci_mode_set(struct meson_drm *priv,
			  struct drm_display_mode *mode);
void meson_venci_enable(struct meson_drm *priv);
void meson_venci_disable(struct meson_drm *priv);

#endif /* __MESON_VENC_H */
