/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2025 Libre Computer
 */

#ifndef __MESON_WRITEBACK_H
#define __MESON_WRITEBACK_H

struct meson_drm;

int meson_writeback_create(struct meson_drm *priv);

/*
 * #23 fix: called from meson_crtc_irq() (hard IRQ) right after it latches
 * priv->viu.osd1_addr into the live OSD1 canvas -- see the definition in
 * meson_writeback.c for the full deferred-capture rationale.
 */
void meson_writeback_vsync_latched(struct meson_drm *priv);

#endif
