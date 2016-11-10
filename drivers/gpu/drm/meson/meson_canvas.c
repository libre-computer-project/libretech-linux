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

#include <linux/kernel.h>
#include <linux/module.h>
#include <drm/drmP.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include "meson_drv.h"
#include "meson_canvas.h"
#include "meson_registers.h"

/*
 * CANVAS is a memory zone where physical memory frames information
 * are stored for the VIU to scanout.
 */

/* DMC Registers */
#define DMC_CAV_LUT_DATAL	0x48 /* 0x12 offset in data sheet */
#define CANVAS_WIDTH_LBIT	29
#define CANVAS_WIDTH_LWID       3
#define DMC_CAV_LUT_DATAH	0x4c /* 0x13 offset in data sheet */
#define CANVAS_WIDTH_HBIT       0
#define CANVAS_HEIGHT_BIT       9
#define CANVAS_BLKMODE_BIT      24
#define DMC_CAV_LUT_ADDR	0x50 /* 0x14 offset in data sheet */
#define CANVAS_LUT_WR_EN        (0x2 << 8)
#define CANVAS_LUT_RD_EN        (0x1 << 8)

/* Canvas configuration. */
#define MESON_CANVAS_WRAP_NONE	0x00
#define	MESON_CANVAS_WRAP_X	0x01
#define	MESON_CANVAS_WRAP_Y	0x02

#define	MESON_CANVAS_BLKMODE_LINEAR	0x00
#define	MESON_CANVAS_BLKMODE_32x32	0x01
#define	MESON_CANVAS_BLKMODE_64x64	0x02

static void meson_canvas_setup(struct meson_drm *priv,
			       uint32_t canvas_index, uint32_t addr,
			       uint32_t stride, uint32_t height,
			       unsigned int wrap,
			       unsigned int blkmode)
{
	unsigned int val;

	regmap_write(priv->dmc, DMC_CAV_LUT_DATAL,
		(((addr + 7) >> 3)) |
		(((stride + 7) >> 3) << CANVAS_WIDTH_LBIT));

	regmap_write(priv->dmc, DMC_CAV_LUT_DATAH,
		((((stride + 7) >> 3) >> CANVAS_WIDTH_LWID) <<
						CANVAS_WIDTH_HBIT) |
		(height << CANVAS_HEIGHT_BIT) |
		(wrap << 22) |
		(blkmode << CANVAS_BLKMODE_BIT));

	regmap_write(priv->dmc, DMC_CAV_LUT_ADDR,
			CANVAS_LUT_WR_EN | canvas_index);

	/* Force a read-back to make sure everything is flushed. */
	regmap_read(priv->dmc, DMC_CAV_LUT_DATAH, &val);
}

void meson_canvas_update_osd1_buffer(struct meson_drm *priv,
				     struct drm_plane *plane)
{
	struct drm_plane_state *state = plane->state;
	struct drm_framebuffer *fb = state->fb;
	struct drm_gem_cma_object *gem;

	gem = drm_fb_cma_get_gem_obj(fb, 0);

	/* Swap out the OSD canvas with the new addr. */
	meson_canvas_setup(priv, MESON_CANVAS_ID_OSD1,
			   gem->paddr, fb->pitches[0],
			   fb->height, MESON_CANVAS_WRAP_NONE,
			   MESON_CANVAS_BLKMODE_LINEAR);
}
