/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2018 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 */

#ifndef __MESON_OVERLAY_H
#define __MESON_OVERLAY_H

#include "meson_drv.h"

int meson_overlay_create(struct meson_drm *priv);

/*
 * #13 fix: deferred flush of the post-blend PQ/HDR2 register blocks
 * (VADJ1/VADJ2/black-extension/white-balance + G12A+ HDR2). Must be called
 * unconditionally from the per-vsync commit-tail in meson_crtc.c -- see the
 * definition in meson_overlay.c for the full rationale.
 */
void meson_overlay_pq_flush(struct meson_drm *priv);

/*
 * FBC-output compress-header registry setter (fbc-output-display-impl-plan.md
 * S1/S2).  Called by meson-vdec (=m) -- which exports nothing to us and only
 * includes this prototype via an extern at its call site (it must NOT include
 * meson-DRM private headers) -- to hand the VD1 AFBC display the per-frame
 * am21c compress-header phys addr (mmu_header_paddr[slot], codec_hevc.c:1657-
 * 1658) keyed by the contiguous body base (the vb2 capture-buffer dma_addr).
 * Defined + EXPORT_SYMBOL_GPL'd in meson_overlay.c.  header_paddr==0 clears.
 */
void meson_overlay_fbc_set_header(dma_addr_t body_base, dma_addr_t header_paddr);

/* Getter for the compress-header registry, exported so meson_overlay2.c's
 * VD2 AFBC path can consult it too (hdmi-vpu-gap-review.md finding #2). See
 * the definition in meson_overlay.c for the full lookup semantics. */
dma_addr_t meson_overlay_fbc_get_header(dma_addr_t body_base);

#endif /* __MESON_OVERLAY_H */
