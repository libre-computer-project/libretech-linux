// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2016 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 * Copyright (C) 2015 Amlogic, Inc. All rights reserved.
 * Copyright (C) 2014 Endless Mobile
 *
 * Written by:
 *     Jasper St. Pierre <jstpierre@mecheye.net>
 */

#include <linux/bitfield.h>
#include <linux/math64.h>
#include <linux/of.h>
#include <linux/spinlock.h>
#include <linux/soc/amlogic/meson-canvas.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_color_mgmt.h>
#include <drm/drm_device.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#include "meson_crtc.h"
#include "meson_overlay.h"
#include "meson_plane.h"
#include "meson_registers.h"
#include "meson_venc.h"
#include "meson_viu.h"
#include "meson_rdma.h"
#include "meson_vpp.h"
#include "meson_osd_afbcd.h"
#include "meson_writeback.h"

#define MESON_G12A_VIU_OFFSET	0x5ec0

/*
 * VRR-002 tunables.
 *
 * GXL/GXM/G12/SM1 have NO VRR hardware block. The vendor implements VRR from
 * T7/T3 onwards via VENC_VRR_CTRL (0x1cd8) + VENC_VRR_ADJ_LMT (0x1cd9), where
 * the timing generator itself holds in vertical blanking and terminates the
 * frame on the real flip pulse (cfg_vsp_din). Nothing equivalent exists on the
 * SoCs this driver supports, so everything below is a SOFTWARE APPROXIMATION
 * built out of the one lever that does exist: ENCP_VIDEO_MAX_LNCNT (vtotal-1),
 * reloaded by the ENCP timing generator at each frame boundary.
 *
 * MESON_VRR_LNCNT_CEILING: the ENCP_VIDEO_MAX_LNCNT field width is NOT
 * documented in any datasheet we hold and has NOT been probed on hardware, so
 * we refuse to infer one. 4095 (12 bits) is a deliberately conservative literal
 * that still covers every real case we care about (the largest nominal vtotal in
 * the driver's mode tables is 2250 for 4K60/4K30, which stretches to 2812 at a
 * 48 Hz sink floor). A sink range that would demand more lines is clipped here
 * with a one-time warning rather than allowed to overflow into whatever bits sit
 * above the field.
 *
 * MESON_VRR_SLEW_SHIFT: per-frame move limit for the vtotal, expressed as
 * nominal >> SHIFT lines (nominal/4). The vendor clamps the same quantity via a
 * configurable mdelta_limit in hdmi_tx_vrr.c calc_vtotal_with_mdelta(); we have
 * no measured value for these sinks, so this is a chosen, UNTUNED constant. It
 * bounds both directions: it keeps a single frame from jumping the full
 * nominal..floor span (e.g. 1125 -> 2812 at 1080p120), which no sink PLL is
 * obliged to track, and it keeps a shortening write from landing below the line
 * counter the ENCP has already reached by the time the vsync IRQ runs.
 */
#define MESON_VRR_LNCNT_CEILING		4095
#define MESON_VRR_SLEW_SHIFT		2

/* CRTC definition */

struct meson_crtc {
	struct drm_crtc base;
	struct drm_pending_vblank_event *event;
	struct meson_drm *priv;
	void (*enable_osd1)(struct meson_drm *priv);
	void (*enable_osd2)(struct meson_drm *priv);
	void (*disable_osd2)(struct meson_drm *priv);
	void (*enable_vd1)(struct meson_drm *priv);
	void (*disable_vd1)(struct meson_drm *priv);
	void (*enable_vd2)(struct meson_drm *priv);
	void (*disable_vd2)(struct meson_drm *priv);
	void (*enable_osd1_afbc)(struct meson_drm *priv);
	void (*disable_osd1_afbc)(struct meson_drm *priv);
	unsigned int viu_offset;
	bool vsync_forced;
	bool vsync_disabled;
	bool vpp_crc_enable;
	/*
	 * Bug 4: previous-frame enabled state for OSD2/VD1/VD2, sampled once
	 * per vsync IRQ. Gates each plane's disable/teardown call (meson_crtc_irq)
	 * on the enabled->disabled EDGE rather than the disabled LEVEL -- without
	 * this, disable_osd2/disable_vd1/disable_vd2 reissue the same register
	 * writes on every single vsync for as long as the plane stays off,
	 * instead of once at the moment it goes off. Reset alongside the rest of
	 * the plane's CRTC-owned state on CRTC disable.
	 */
	bool osd2_enabled_prev;
	bool vd1_enabled_prev;
	bool vd2_enabled_prev;
	/*
	 * VRR-002: software variable-refresh control-loop state.
	 *
	 * Producers are atomic_flush / atomic_enable / atomic_disable (process
	 * context, serialised by the DRM commit path); the consumer is
	 * meson_crtc_irq (hard IRQ, one call per ENCP frame boundary). Every
	 * field below is written under vrr_lock so the IRQ never observes a
	 * half-published configuration. vrr_active and vrr_dirty are only ever
	 * WRITTEN from process context, which is serialised against itself, so the
	 * lock-free reads of them there are safe. vrr_restore and vrr_flip_pending
	 * are written from both sides and always under the lock; the IRQ's unlocked
	 * read of vrr_active/vrr_restore is only a fast-path gate, re-tested under
	 * the lock before anything is consumed.
	 */
	spinlock_t vrr_lock;
	bool vrr_active;		/* the loop is modulating the timing */
	bool vrr_dirty;			/* we have written MAX_LNCNT at least once */
	bool vrr_restore;		/* IRQ owes a nominal-timing restore */
	bool vrr_flip_pending;		/* a commit was flushed since last vsync */
	u32 vrr_nominal_lines;		/* the mode's own vtotal */
	u32 vrr_min_lines;		/* sink VRRmax -> fewest lines */
	u32 vrr_max_lines;		/* sink VRRmin -> most lines */
	u32 vrr_cur_lines;		/* vtotal currently programmed */
	u32 vrr_slew_lines;		/* per-frame move limit */
	/*
	 * VRR-001d (EDID-range clamp): the sink's EDID-advertised FreeSync
	 * refresh range in Hz, published by the HDMI encoder atomic_check via
	 * meson_crtc_set_vrr_range(). 0 == no usable VRR range known -> VRR is
	 * refused in atomic_check and atomic_flush will not modulate the timing.
	 */
	u16 vrr_min_vfreq;
	u16 vrr_max_vfreq;
};
#define to_meson_crtc(x) container_of(x, struct meson_crtc, base)

/* CRTC */

static int meson_crtc_enable_vblank(struct drm_crtc *crtc)
{
	struct meson_crtc *meson_crtc = to_meson_crtc(crtc);
	struct meson_drm *priv = meson_crtc->priv;

	meson_crtc->vsync_disabled = false;
	meson_venc_enable_vsync(priv);

	return 0;
}

static void meson_crtc_disable_vblank(struct drm_crtc *crtc)
{
	struct meson_crtc *meson_crtc = to_meson_crtc(crtc);
	struct meson_drm *priv = meson_crtc->priv;

	if (!meson_crtc->vsync_forced) {
		meson_crtc->vsync_disabled = true;
		meson_venc_disable_vsync(priv);
	}
}

static int meson_crtc_set_crc_source(struct drm_crtc *crtc, const char *source)
{
	struct meson_crtc *meson_crtc = to_meson_crtc(crtc);
	struct meson_drm *priv = meson_crtc->priv;

	if (!source) {
		if (meson_crtc->vpp_crc_enable) {
			meson_crtc->vpp_crc_enable = false;
			drm_crtc_vblank_put(crtc);
		}
		return 0;
	}

	if (strcmp(source, "auto") == 0) {
		/*
		 * VPP CRC hardware (VPP_RO_CRCSUM 0x1DB2) only works on
		 * SM1 and newer. G12A/G12B returns 0; GXL faults.
		 */
		if (!of_machine_is_compatible("amlogic,sm1"))
			return -ENODEV;
		if (!meson_crtc->vpp_crc_enable) {
			int ret = drm_crtc_vblank_get(crtc);

			if (ret)
				return ret;
			/* Trigger initial CRC computation */
			writel_relaxed(1, priv->io_base + _REG(VPP_CRC_CHK));
			meson_crtc->vpp_crc_enable = true;
		}
		return 0;
	}

	return -EINVAL;
}

static int meson_crtc_verify_crc_source(struct drm_crtc *crtc,
					const char *source, size_t *values_cnt)
{
	if (source && strcmp(source, "auto") != 0)
		return -EINVAL;

	if (source && !of_machine_is_compatible("amlogic,sm1"))
		return -ENODEV;

	*values_cnt = 1;
	return 0;
}

static const struct drm_crtc_funcs meson_crtc_funcs = {
	.atomic_destroy_state	= drm_atomic_helper_crtc_destroy_state,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.destroy		= drm_crtc_cleanup,
	.page_flip		= drm_atomic_helper_page_flip,
	.reset			= drm_atomic_helper_crtc_reset,
	.set_config             = drm_atomic_helper_set_config,
	.enable_vblank		= meson_crtc_enable_vblank,
	.disable_vblank		= meson_crtc_disable_vblank,
	.set_crc_source		= meson_crtc_set_crc_source,
	.verify_crc_source	= meson_crtc_verify_crc_source,
};

/*
 * VRR-002 (audit item 4): VRR here is implemented purely by modulating
 * ENCP_VIDEO_MAX_LNCNT, so it is only meaningful when the ENCP progressive
 * timing generator is the block actually driving the link.
 *
 * 480i/576i (DRM_MODE_FLAG_DBLCLK) and every interlaced mode are driven by ENCI
 * instead -- meson_venc_hdmi_mode_set() sets use_enci for DBLCLK, and
 * meson_encoder_hdmi_atomic_enable() then enables ENCI_VIDEO_EN and leaves
 * ENCP_VIDEO_EN at 0. CVBS and DSI never run the ENCP-HDMI path at all. In all
 * of those cases the MAX_LNCNT write lands in a register no live timing
 * generator reads, and the previous code happily reported VRR as active while
 * writing that dead register.
 *
 * The mode-flag half of the test is evaluated in atomic_check (it depends only
 * on the mode being committed). The priv->venc half describes what mode_set has
 * actually programmed, which on a combined modeset+VRR commit is not yet up to
 * date at check time, so it is only consulted from atomic_flush; the worst case
 * there is that VRR stays off for one extra commit.
 */
static bool meson_crtc_vrr_mode_flags_ok(const struct drm_display_mode *mode)
{
	return !(mode->flags & (DRM_MODE_FLAG_INTERLACE | DRM_MODE_FLAG_DBLCLK));
}

static bool meson_crtc_vrr_encp_active(struct meson_drm *priv)
{
	return priv->venc.current_mode == MESON_VENC_MODE_HDMI &&
	       !priv->venc.hdmi_use_enci;
}

/*
 * VRR-002 (audit item 3): put the loop back into a defined state.
 *
 * Neither disable path used to clear anything, so after a DPMS off->on the first
 * flush saw a stale timestamp, computed a gigantic dt and ran the first frame at
 * the sink's floor. Reset on both disable and enable; @mode publishes the nominal
 * vtotal that the IRQ needs for a restore (NULL on the disable side, where the
 * next enable will publish the new mode's value).
 */
static void meson_crtc_vrr_reset(struct meson_crtc *meson_crtc,
				 const struct drm_display_mode *mode)
{
	unsigned long flags;

	spin_lock_irqsave(&meson_crtc->vrr_lock, flags);
	meson_crtc->vrr_active = false;
	meson_crtc->vrr_restore = false;
	meson_crtc->vrr_flip_pending = false;
	meson_crtc->vrr_cur_lines = 0;
	meson_crtc->vrr_min_lines = 0;
	meson_crtc->vrr_max_lines = 0;
	meson_crtc->vrr_slew_lines = 0;
	meson_crtc->vrr_nominal_lines = mode ? mode->vtotal : 0;
	spin_unlock_irqrestore(&meson_crtc->vrr_lock, flags);
}

/*
 * Shared tail of both atomic_enable variants.
 *
 * A DPMS off->on is an active-only change: drm_atomic_helper commit_modeset_*
 * skips crtc_set_mode() when neither mode_changed nor connectors_changed is set,
 * so the bridge mode_set that normally programs ENCP_VIDEO_MAX_LNCNT is NOT
 * re-run and a stretched value left behind by a previous VRR session would
 * survive into the re-enabled ENCP. Restore the nominal timing here, before the
 * encoder re-enables ENCP_VIDEO_EN (crtc atomic_enable runs ahead of the bridge
 * enable in commit_modeset_enables).
 *
 * Gated on vrr_dirty: a CRTC that has never run VRR issues no register write at
 * all, so the no-VRR path stays byte-for-byte what it was.
 */
static void meson_crtc_vrr_on_enable(struct meson_crtc *meson_crtc,
				     const struct drm_display_mode *mode)
{
	struct meson_drm *priv = meson_crtc->priv;

	if (meson_crtc->vrr_dirty && mode->vtotal >= 2)
		writel_relaxed(mode->vtotal - 1,
			       priv->io_base + _REG(ENCP_VIDEO_MAX_LNCNT));

	meson_crtc_vrr_reset(meson_crtc, mode);
}

static void meson_g12a_crtc_atomic_enable(struct drm_crtc *crtc,
					  struct drm_atomic_state *state)
{
	struct meson_crtc *meson_crtc = to_meson_crtc(crtc);
	struct drm_crtc_state *crtc_state = crtc->state;
	struct meson_drm *priv = meson_crtc->priv;

	DRM_DEBUG_DRIVER("\n");

	if (!crtc_state) {
		DRM_ERROR("Invalid crtc_state\n");
		return;
	}

	/* VD1 Preblend vertical start/end */
	writel(FIELD_PREP(GENMASK(11, 0), 2303),
	       priv->io_base + _REG(VPP_PREBLEND_VD1_V_START_END));

	/* Setup Blender */
	writel(crtc_state->mode.hdisplay |
	       crtc_state->mode.vdisplay << 16,
	       priv->io_base + _REG(VPP_POSTBLEND_H_SIZE));

	writel_relaxed(0 << 16 |
			(crtc_state->mode.hdisplay - 1),
			priv->io_base + _REG(VPP_OSD1_BLD_H_SCOPE));
	writel_relaxed(0 << 16 |
			(crtc_state->mode.vdisplay - 1),
			priv->io_base + _REG(VPP_OSD1_BLD_V_SCOPE));
	writel_relaxed(crtc_state->mode.hdisplay << 16 |
			crtc_state->mode.vdisplay,
			priv->io_base + _REG(VPP_OUT_H_V_SIZE));

	meson_crtc_vrr_on_enable(meson_crtc, &crtc_state->adjusted_mode);

	drm_crtc_vblank_on(crtc);
}

static void meson_crtc_atomic_enable(struct drm_crtc *crtc,
				     struct drm_atomic_state *state)
{
	struct meson_crtc *meson_crtc = to_meson_crtc(crtc);
	struct drm_crtc_state *crtc_state = crtc->state;
	struct meson_drm *priv = meson_crtc->priv;

	DRM_DEBUG_DRIVER("\n");

	if (!crtc_state) {
		DRM_ERROR("Invalid crtc_state\n");
		return;
	}

	/* Enable VPP Postblend */
	writel(crtc_state->mode.hdisplay,
	       priv->io_base + _REG(VPP_POSTBLEND_H_SIZE));

	/* VD1 Preblend vertical start/end */
	writel(FIELD_PREP(GENMASK(11, 0), 2303),
			priv->io_base + _REG(VPP_PREBLEND_VD1_V_START_END));

	writel_bits_relaxed(VPP_POSTBLEND_ENABLE, VPP_POSTBLEND_ENABLE,
			    priv->io_base + _REG(VPP_MISC));

	meson_crtc_vrr_on_enable(meson_crtc, &crtc_state->adjusted_mode);

	drm_crtc_vblank_on(crtc);
}

static void meson_g12a_crtc_atomic_disable(struct drm_crtc *crtc,
					   struct drm_atomic_state *state)
{
	struct meson_crtc *meson_crtc = to_meson_crtc(crtc);
	struct meson_drm *priv = meson_crtc->priv;

	DRM_DEBUG_DRIVER("\n");

	drm_crtc_vblank_off(crtc);

	/* VRR-002 (audit item 3): drop the VRR loop state with the pipe. */
	meson_crtc_vrr_reset(meson_crtc, NULL);

	priv->viu.osd1_enabled = false;
	priv->viu.osd1_commit = false;

	priv->viu.osd2_enabled = false;
	priv->viu.osd2_commit = false;

	priv->viu.vd1_enabled = false;
	priv->viu.vd1_commit = false;

	priv->viu.vd2_enabled = false;
	priv->viu.vd2_commit = false;

	/* Bug 4: drop the disable-edge tracking with the rest of the pipe's
	 * per-plane state, so a re-enable starts from a known baseline. */
	meson_crtc->osd2_enabled_prev = false;
	meson_crtc->vd1_enabled_prev = false;
	meson_crtc->vd2_enabled_prev = false;

	if (crtc->state->event && !crtc->state->active) {
		spin_lock_irq(&crtc->dev->event_lock);
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		spin_unlock_irq(&crtc->dev->event_lock);

		crtc->state->event = NULL;
	}
}

static void meson_crtc_atomic_disable(struct drm_crtc *crtc,
				      struct drm_atomic_state *state)
{
	struct meson_crtc *meson_crtc = to_meson_crtc(crtc);
	struct meson_drm *priv = meson_crtc->priv;

	DRM_DEBUG_DRIVER("\n");

	drm_crtc_vblank_off(crtc);

	/* VRR-002 (audit item 3): drop the VRR loop state with the pipe. */
	meson_crtc_vrr_reset(meson_crtc, NULL);

	priv->viu.osd1_enabled = false;
	priv->viu.osd1_commit = false;

	priv->viu.osd2_enabled = false;
	priv->viu.osd2_commit = false;

	priv->viu.vd1_enabled = false;
	priv->viu.vd1_commit = false;

	/* Bug 4: drop the disable-edge tracking with the rest of the pipe's
	 * per-plane state, so a re-enable starts from a known baseline. */
	meson_crtc->osd2_enabled_prev = false;
	meson_crtc->vd1_enabled_prev = false;
	meson_crtc->vd2_enabled_prev = false;

	/* Disable VPP Postblend */
	writel_bits_relaxed(VPP_OSD1_POSTBLEND | VPP_VD1_POSTBLEND |
			    VPP_VD1_PREBLEND | VPP_POSTBLEND_ENABLE, 0,
			    priv->io_base + _REG(VPP_MISC));

	if (crtc->state->event && !crtc->state->active) {
		spin_lock_irq(&crtc->dev->event_lock);
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		spin_unlock_irq(&crtc->dev->event_lock);

		crtc->state->event = NULL;
	}
}

static void meson_crtc_atomic_begin(struct drm_crtc *crtc,
				    struct drm_atomic_state *state)
{
	struct meson_crtc *meson_crtc = to_meson_crtc(crtc);
	unsigned long flags;

	if (crtc->state->event) {
		WARN_ON(drm_crtc_vblank_get(crtc) != 0);

		spin_lock_irqsave(&crtc->dev->event_lock, flags);
		meson_crtc->event = crtc->state->event;
		spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
		crtc->state->event = NULL;
	}
}

static void meson_crtc_atomic_flush(struct drm_crtc *crtc,
				    struct drm_atomic_state *state)
{
	struct meson_crtc *meson_crtc = to_meson_crtc(crtc);
	struct drm_crtc_state *crtc_state =
		drm_atomic_get_new_crtc_state(state, crtc);
	struct meson_drm *priv = meson_crtc->priv;

	if (crtc_state->color_mgmt_changed) {
		unsigned long cm_flags;

		/*
		 * Audit #8: meson_viu_set_ctm()/set_gamma_lut() now only
		 * update priv->viu's cache (no MMIO) -- the actual hardware
		 * programming is deferred to the vsync IRQ (meson_crtc_irq(),
		 * via cm_commit below), matching every other register in
		 * this driver instead of writing synchronously from the
		 * commit tail. Audit #26: take event_lock around the cache
		 * update, the same lock meson_crtc_irq takes around its
		 * consuming read, since the cached CTM/LUT are multi-word
		 * state shared with that IRQ.
		 */
		spin_lock_irqsave(&crtc->dev->event_lock, cm_flags);

		if (crtc_state->ctm)
			meson_viu_set_ctm(priv, crtc_state->ctm->data);
		else
			meson_viu_set_ctm(priv, NULL);

		if (crtc_state->gamma_lut) {
			unsigned int size = crtc_state->gamma_lut->length /
					    sizeof(struct drm_color_lut);
			meson_viu_set_gamma_lut(priv,
						crtc_state->gamma_lut->data,
						size);
		} else {
			meson_viu_set_gamma_lut(priv, NULL, 0);
		}

		priv->viu.cm_commit = true;

		spin_unlock_irqrestore(&crtc->dev->event_lock, cm_flags);
	}

	/*
	 * VRR-002: variable refresh via ENCP_VIDEO_MAX_LNCNT (vtotal-1). The ENCP
	 * timing generator reloads max_lncnt at each frame boundary, so a larger
	 * value lengthens the next frame -> lower refresh.
	 *
	 * WHY THE OLD LOOP COULD NOT WORK. VRR-001 sampled ktime here and derived
	 * the next frame's vtotal from the inter-flush interval. atomic_flush runs
	 * in the commit tail, i.e. AFTER the commit has waited on the previous
	 * commit's flip_done -- and that vblank is the end of the very frame this
	 * code had just stretched. So dt ~= vtotal * htotal / clock and
	 * vtotal(N+1) ~= vtotal(N): every value was a fixed point and the loop
	 * ratcheted to max_lines and stayed there. Measured at 1080p120 (nominal
	 * 1125, sink 48-120 Hz -> max_lines 2812): requested flip intervals of
	 * 16667 / 33333 / 41667 us ALL settled at ENCP=2811, presenting as a static
	 * 48 Hz -- the floor -- so the sink locked it as a fixed low-refresh mode
	 * and never entered VRR at all.
	 *
	 * THE REPLACEMENT. No interval is measured anywhere. atomic_flush only
	 * publishes the loop's configuration and raises a one-bit "new frame was
	 * submitted" flag; the loop itself runs in the vsync IRQ (meson_crtc_irq),
	 * one step per ENCP frame boundary, and its only input is whether that flag
	 * was set during the frame that just ended:
	 *
	 *   frame arrived  -> step vtotal DOWN by the slew limit (refresh faster)
	 *   nothing arrived-> step vtotal UP   by the slew limit (hold longer)
	 *
	 * That is an integrating bang-bang controller whose feedback is NEGATIVE:
	 * too long a vtotal means content lands every frame and drives it back
	 * down; too short means frames are missed and drives it up. Its fixed point
	 * is the rate at which userspace actually submits, clamped to the sink's
	 * range -- content below the sink floor saturates at max_lines (which is
	 * what LFC does), content at or above the mode's own rate saturates at
	 * nominal. It limit-cycles by one slew step around the fixed point; that
	 * jitter is inherent to a boundary-only actuator and is exactly the kind of
	 * variation a VRR sink exists to absorb.
	 *
	 * Crucially the input is a BOOLEAN sampled on the hardware frame clock, not
	 * a duration measured across a vblank we control, so the self-reference is
	 * gone. It remains an APPROXIMATION: real VRR hardware (T7+ VENC_VRR_CTRL)
	 * terminates the frame mid-blanking on the actual flip pulse, whereas we can
	 * only choose a frame's length at its start, so a frame that arrives just
	 * after a boundary still waits out the frame in progress.
	 *
	 * EDID-range clamp (inherited from VRR-001d): the vtotal is bounded to the
	 * SINK's advertised FreeSync range (vrr_min_vfreq..vrr_max_vfreq in Hz,
	 * published by the HDMI encoder atomic_check from the parsed EDID). Lines
	 * for a target refresh R are clock_khz * 1000 / (htotal * R): lowest
	 * refresh (VRRmin) -> most lines (max_lines), highest refresh (VRRmax) ->
	 * fewest lines (min_lines). min_lines is floored at the mode nominal so VRR
	 * can only EXTEND a frame, never shorten it below the mode's own timing. An
	 * out-of-range excursion does not merely glitch: it drops the link and can
	 * wedge the HDMI TX until a controller re-init (see
	 * display/vrr/runtime-validation.md).
	 *
	 * vrr_enabled is only ever set when the sink is vrr_capable AND the request
	 * passed atomic_check (which rejects an unknown range), so vrr_min_vfreq is
	 * non-zero here; the guard below is defensive -> stay at nominal.
	 */
	if (crtc_state->vrr_enabled &&
	    meson_crtc_vrr_mode_flags_ok(&crtc_state->adjusted_mode) &&
	    meson_crtc_vrr_encp_active(priv)) {
		const struct drm_display_mode *m = &crtc_state->adjusted_mode;
		u32 nominal = m->vtotal;
		u32 min_lines = nominal;
		u32 max_lines = nominal;
		unsigned long flags;

		if (m->clock && m->htotal && meson_crtc->vrr_min_vfreq &&
		    meson_crtc->vrr_max_vfreq) {
			/* most lines = lowest refresh = sink VRRmin */
			max_lines = div_u64((u64)m->clock * 1000,
					    (u64)m->htotal *
					    meson_crtc->vrr_min_vfreq);
			/* fewest lines = highest refresh = sink VRRmax */
			min_lines = div_u64((u64)m->clock * 1000,
					    (u64)m->htotal *
					    meson_crtc->vrr_max_vfreq);
		}

		/* never shorten below the mode's own nominal timing */
		if (min_lines < nominal)
			min_lines = nominal;
		/*
		 * VRR-002 (audit item 5): the MAX_LNCNT field width is unverified,
		 * so clip to a conservative literal instead of inventing one.
		 */
		if (max_lines > MESON_VRR_LNCNT_CEILING) {
			drm_warn_once(priv->drm,
				      "VRR: sink floor %u Hz needs %u lines, clipping to %u\n",
				      meson_crtc->vrr_min_vfreq, max_lines,
				      MESON_VRR_LNCNT_CEILING);
			max_lines = MESON_VRR_LNCNT_CEILING;
		}
		if (min_lines > MESON_VRR_LNCNT_CEILING)
			min_lines = MESON_VRR_LNCNT_CEILING;
		if (max_lines < min_lines)
			max_lines = min_lines;

		spin_lock_irqsave(&meson_crtc->vrr_lock, flags);
		meson_crtc->vrr_nominal_lines = nominal;
		meson_crtc->vrr_min_lines = min_lines;
		meson_crtc->vrr_max_lines = max_lines;
		meson_crtc->vrr_slew_lines = max_t(u32, nominal >>
						   MESON_VRR_SLEW_SHIFT, 1);
		if (!meson_crtc->vrr_active) {
			/* new session: start from the mode's nominal timing */
			meson_crtc->vrr_cur_lines = nominal;
			meson_crtc->vrr_active = true;
			meson_crtc->vrr_dirty = true;
		}
		meson_crtc->vrr_restore = false;
		/*
		 * The loop's entire input: a frame was submitted. The IRQ
		 * consumes (and clears) it at the next frame boundary.
		 */
		meson_crtc->vrr_flip_pending = true;
		spin_unlock_irqrestore(&meson_crtc->vrr_lock, flags);
	} else if (meson_crtc->vrr_active) {
		unsigned long flags;

		/*
		 * VRR just turned off. Hand the nominal-timing restore to the
		 * vsync IRQ rather than doing MMIO from the commit tail (audit
		 * item 1): MAX_LNCNT is a timing-generator register and must be
		 * written at a frame boundary, like every other deferred
		 * register in this driver.
		 */
		spin_lock_irqsave(&meson_crtc->vrr_lock, flags);
		meson_crtc->vrr_active = false;
		meson_crtc->vrr_flip_pending = false;
		meson_crtc->vrr_cur_lines = 0;
		meson_crtc->vrr_nominal_lines = crtc_state->adjusted_mode.vtotal;
		meson_crtc->vrr_restore = true;
		spin_unlock_irqrestore(&meson_crtc->vrr_lock, flags);
	}

	priv->viu.osd1_commit = true;
	priv->viu.osd2_commit = true;
	priv->viu.vd1_commit = true;
	priv->viu.vd2_commit = true;
}

static int meson_crtc_atomic_check(struct drm_crtc *crtc,
				   struct drm_atomic_state *state)
{
	struct meson_crtc *meson_crtc = to_meson_crtc(crtc);
	struct drm_crtc_state *crtc_state =
		drm_atomic_get_new_crtc_state(state, crtc);

	/*
	 * Gate VRR_ENABLED here, in the CRTC atomic_check, so it is enforced on
	 * EVERY commit that touches the CRTC -- including a property-only
	 * VRR_ENABLED change that does not flag a modeset and so never reaches
	 * the HDMI encoder (bridge) atomic_check. A bridge-only gate lets a
	 * property-only VRR_ENABLED=1 through, where it reads back as 1 on a
	 * sink that cannot do VRR at all.
	 *
	 * vrr_min_vfreq is the connected sink's advertised FreeSync floor,
	 * published to the CRTC by the encoder atomic_check
	 * (meson_crtc_set_vrr_range); 0 == the sink advertised no usable
	 * FreeSync range, i.e. not VRR-capable. It persists from the last
	 * modeset, so it is the right thing to consult for a property-only
	 * commit. Refuse VRR on such a sink rather than accept it as a no-op.
	 */
	if (crtc_state->vrr_enabled && !meson_crtc->vrr_min_vfreq)
		return -EINVAL;

	/*
	 * VRR-002 (audit item 4): refuse VRR on a mode this implementation cannot
	 * drive at all. 480i/576i (DBLCLK) and interlaced modes run on ENCI, where
	 * ENCP_VIDEO_EN is 0 and the MAX_LNCNT write is dead -- accepting
	 * VRR_ENABLED there would report VRR active while doing nothing.
	 */
	if (crtc_state->vrr_enabled &&
	    !meson_crtc_vrr_mode_flags_ok(&crtc_state->adjusted_mode))
		return -EINVAL;

	return 0;
}

static const struct drm_crtc_helper_funcs meson_crtc_helper_funcs = {
	.atomic_check	= meson_crtc_atomic_check,
	.atomic_begin	= meson_crtc_atomic_begin,
	.atomic_flush	= meson_crtc_atomic_flush,
	.atomic_enable	= meson_crtc_atomic_enable,
	.atomic_disable	= meson_crtc_atomic_disable,
};

static const struct drm_crtc_helper_funcs meson_g12a_crtc_helper_funcs = {
	.atomic_check	= meson_crtc_atomic_check,
	.atomic_begin	= meson_crtc_atomic_begin,
	.atomic_flush	= meson_crtc_atomic_flush,
	.atomic_enable	= meson_g12a_crtc_atomic_enable,
	.atomic_disable	= meson_g12a_crtc_atomic_disable,
};

static void meson_crtc_enable_osd1(struct meson_drm *priv)
{
	writel_bits_relaxed(VPP_OSD1_POSTBLEND, VPP_OSD1_POSTBLEND,
			    priv->io_base + _REG(VPP_MISC));
}

static void meson_crtc_g12a_enable_osd1_afbc(struct meson_drm *priv)
{
	writel_relaxed(priv->viu.osd1_blk2_cfg4,
		       priv->io_base + _REG(VIU_OSD1_BLK2_CFG_W4));

	writel_bits_relaxed(OSD_MEM_LINEAR_ADDR, OSD_MEM_LINEAR_ADDR,
			    priv->io_base + _REG(VIU_OSD1_CTRL_STAT));

	writel_relaxed(priv->viu.osd1_blk1_cfg4,
		       priv->io_base + _REG(VIU_OSD1_BLK1_CFG_W4));

	meson_viu_g12a_enable_osd1_afbc(priv);

	writel_bits_relaxed(OSD_MEM_LINEAR_ADDR, OSD_MEM_LINEAR_ADDR,
			    priv->io_base + _REG(VIU_OSD1_CTRL_STAT));

	writel_bits_relaxed(OSD_MALI_SRC_EN, OSD_MALI_SRC_EN,
			    priv->io_base + _REG(VIU_OSD1_BLK0_CFG_W0));
}

static void meson_g12a_crtc_enable_osd1(struct meson_drm *priv)
{
	writel_relaxed(priv->viu.osd_blend_din0_scope_h,
		       priv->io_base +
		       _REG(VIU_OSD_BLEND_DIN0_SCOPE_H));
	writel_relaxed(priv->viu.osd_blend_din0_scope_v,
		       priv->io_base +
		       _REG(VIU_OSD_BLEND_DIN0_SCOPE_V));
	writel_relaxed(priv->viu.osb_blend0_size,
		       priv->io_base +
		       _REG(VIU_OSD_BLEND_BLEND0_SIZE));
	writel_relaxed(priv->viu.osb_blend1_size,
		       priv->io_base +
		       _REG(VIU_OSD_BLEND_BLEND1_SIZE));
	writel_bits_relaxed(3 << 8, 3 << 8,
			    priv->io_base + _REG(OSD1_BLEND_SRC_CTRL));
}

/*
 * Phase 2 stage 1: program the OSD-blender (VIU_OSD_BLEND_CTRL) z-order from the
 * OSD planes' zpos. Whenever BOTH OSDs are composited, route them through the
 * blender as DIN0 (bottom) + DIN1 (top) so each is clipped to its own DIN scope
 * -- OSD2's DIN1 scope is the small bounce box; without DIN1 enabled OSD2 has no
 * scoped path and its opaque pixels flood the whole frame. The zpos picks which
 * OSD is DIN1 (top). A single OSD keeps the config byte-identical to
 * meson_viu_init (DIN0 only + bypass), so that common case is provably unchanged
 * (no regression). G12A/G12B/SM1 only.
 */
static void meson_g12a_crtc_set_osd_order(struct meson_drm *priv)
{
	u32 val = VIU_OSD_BLEND1_DIN3_BYPASS_TO_DOUT1 |
		  VIU_OSD_BLEND1_DOUT_BYPASS_TO_BLEND2 |
		  VIU_OSD_BLEND_BLEN2_PREMULT_EN(1) |
		  VIU_OSD_BLEND_HOLD_LINES(4);

	if (priv->viu.osd1_enabled && priv->viu.osd2_enabled) {
		/* Both OSDs: blend DIN0 (bottom) + DIN1 (top), each clipped to its
		 * DIN scope. zpos selects which OSD is on top. */
		if (priv->viu.osd1_zpos > priv->viu.osd2_zpos)
			val |= VIU_OSD_BLEND_REORDER(0, 2) |	/* DIN0 (bottom) <- OSD2 */
			       VIU_OSD_BLEND_REORDER(1, 1) |	/* DIN1 (top)    <- OSD1 */
			       VIU_OSD_BLEND_DIN_EN(3);		/* enable DIN0 + DIN1 */
		else
			val |= VIU_OSD_BLEND_REORDER(0, 1) |	/* DIN0 (bottom) <- OSD1 */
			       VIU_OSD_BLEND_REORDER(1, 2) |	/* DIN1 (top)    <- OSD2 */
			       VIU_OSD_BLEND_DIN_EN(3);		/* enable DIN0 + DIN1 */
	} else {
		/* Single OSD: DIN0 <- OSD1, bypass to dout0 (== meson_viu_init). */
		val |= VIU_OSD_BLEND_REORDER(0, 1) |
		       VIU_OSD_BLEND_REORDER(1, 0) |
		       VIU_OSD_BLEND_DIN_EN(1) |
		       VIU_OSD_BLEND_DIN0_BYPASS_TO_DOUT0;
	}
	writel_relaxed(val, priv->io_base + _REG(VIU_OSD_BLEND_CTRL));
}

/*
 * Phase 2 stage 2: order the video group against the OSD from zpos, via the VPP
 * post-blend ports. Per the BSP (video_hw.c port_val model) the post-blend video
 * ports are the fixed-order array {VD1,VD2,VD3,OSD2}_BLEND_SRC_CTRL; a video
 * layer's POSTBLD_SRC ([11:8]) is written into the port matching its z-order.
 * The OSD-dout enters at its OWN port (OSD1_BLEND_SRC_CTRL, bit20 PATH_SEL),
 * which sits between the VD2 and VD3 ports -- so a video source in VD1/VD2 is
 * BELOW the OSD, and a source in VD3 (or OSD2) is ABOVE it.
 *
 * Default (video below OSD) keeps the per-layer enable_* writes byte-identical,
 * so the common case is provably unchanged. "Video above OSD" lifts VD1's source
 * up into the OSD2 port (top of the stack) and clears it from the VD1 port. The
 * write must come after disable_osd2 (set_vd_order runs at IRQ end), which is why
 * the OSD2 port is usable here. G12A/G12B/SM1; requires the OSD2 plane inactive.
 *
 * NOTE: captured proof on demo-aml-s905d3-cc (4K HDMI via a capture dongle)
 * eliminated two earlier guesses: the VD1<->OSD1 selector swap (OSD1 is the
 * OSD-dout port, not a video port) and the VD3 port (0x1def is not backed on
 * SM1 -- writes read back 0). OSD2 (0x1dfe) is the usable above-OSD slot.
 */
static void meson_g12a_crtc_set_vd_order(struct meson_drm *priv)
{
	bool vd1 = priv->viu.vd1_enabled, vd2 = priv->viu.vd2_enabled;
	bool osd1 = priv->viu.osd1_enabled, osd2 = priv->viu.osd2_enabled;
	u32 video_bot, osd_top;
	bool video_above_osd;

	/* Only meaningful when video and OSD are composited together. */
	if (!(vd1 || vd2) || !(osd1 || osd2))
		return;

	/* video band bottom = lowest enabled video zpos */
	video_bot = vd1 ? priv->viu.vd1_zpos : priv->viu.vd2_zpos;
	if (vd1 && vd2 && priv->viu.vd2_zpos < video_bot)
		video_bot = priv->viu.vd2_zpos;

	/* OSD band top = highest enabled OSD zpos */
	osd_top = osd1 ? priv->viu.osd1_zpos : priv->viu.osd2_zpos;
	if (osd1 && osd2 && priv->viu.osd2_zpos > osd_top)
		osd_top = priv->viu.osd2_zpos;

	/* "Video above OSD" is only realizable by routing VD1 into the OSD2
	 * port -- the top of the SM1 {VD1,VD2,OSD1,OSD2}_BLEND_SRC_CTRL stack,
	 * ABOVE the OSD-dout's OSD1 slot -- which requires a VD1 to route and
	 * the OSD2 port to be free (OSD2 plane inactive; VD3 at 0x1def is not
	 * backed on SM1, so OSD2 is the only usable above-OSD slot). */
	video_above_osd = video_bot > osd_top && vd1 && !osd2;

	if (video_above_osd) {
		/* Route VD1's post-blend source into the OSD2 port and drop it
		 * from its own (low) port -- keep its premult/preblend. This
		 * runs at the END of the vsync IRQ, after meson_g12a_crtc_disable_osd2
		 * would have cleared the OSD2 port, so it wins.
		 *
		 * The upstream driver never programs VPP_MISC1 -- the VD1
		 * post-blend layer alpha -- leaving it at its 0x0f000000 reset
		 * value. With VD1 routed into the premultiplied OSD2 port above
		 * the OSD-dout, that wrong alpha makes the post-blend ADD ~50%
		 * of the OSD through the video (additive bleed-through, out =
		 * video + osd*(1 - layer_alpha)) instead of occluding it. The
		 * BSP sets vd_layer[0].layer_alpha = VPP_VD1_ALPHA_MAX (0x100,
		 * fully opaque) via VPP_MISC1; do the same so video-above-OSD is
		 * opaque, not semi-transparent. Confirmed on SM1
		 * (demo-aml-s905d3-cc, 4K HDMI capture). */
		writel_relaxed(VPP_VD1_ALPHA_MAX, priv->io_base + _REG(VPP_MISC1));
		writel_relaxed(VD_BLEND_POSTBLD_SRC_VD1 | VD_BLEND_POSTBLD_PREMULT_EN,
			       priv->io_base + _REG(OSD2_BLEND_SRC_CTRL));
		writel_bits_relaxed(VD_BLEND_POSTBLD_SRC_MASK, 0,
				    priv->io_base + _REG(VD1_BLEND_SRC_CTRL));
	} else {
		/*
		 * Bug 2 fix: default order (video band below OSD band), OR
		 * "above" was requested but is not realizable (no VD1, or OSD2
		 * active and holding its own... well, using its port). VD1
		 * stays in / returns to its own (low) port -- enable_vd1
		 * re-arms that port on every enable regardless of this
		 * function's decision, so nothing to do there.
		 *
		 * Actively clear the OSD2 port here rather than assuming it is
		 * already clear. The old code used a bare early return for this
		 * branch, which meant a previous "video above OSD" ordering that
		 * had routed VD1 into the OSD2 port was NEVER undone on the
		 * reverse zpos transition: VD1 ended up sourced into BOTH the
		 * OSD2 port (stale) and its own dedicated port (freshly re-armed
		 * by enable_vd1) at once, blending additively and never leaving
		 * the "on top" visual state even after the ordering changed.
		 */
		writel_bits_relaxed(3 << 8, 0,
				    priv->io_base + _REG(OSD2_BLEND_SRC_CTRL));
	}
}

/* Legacy (pre-G12A) OSD2 enable. VPP_OSD2_POSTBLEND alone only lets OSD2
 * participate in the blend; VPP_PRE_FG_OSD2 | VPP_POST_FG_OSD2 select it as
 * the TOP layer over OSD1, matching the immutable zpos (OSD2 > OSD1) this
 * driver exposes on every platform. Without them OSD2 is composited but
 * hidden -- a full-screen opaque OSD1 completely occludes it. Confirmed on
 * hardware (aml-s905x-cc, GXL): a windowed OSD2 with OSD1 active rendered
 * pure OSD1 with zero trace of OSD2 before this fix, capture-verified
 * correct after. See meson_registers.h VPP_MISC for the BSP reference. */
static void meson_crtc_enable_osd2(struct meson_drm *priv)
{
	writel_bits_relaxed(VPP_OSD2_POSTBLEND | VPP_PRE_FG_OSD2 | VPP_POST_FG_OSD2,
			    VPP_OSD2_POSTBLEND | VPP_PRE_FG_OSD2 | VPP_POST_FG_OSD2,
			    priv->io_base + _REG(VPP_MISC));
}

/* Symmetric to meson_crtc_enable_osd2: clear the postblend-enable AND the
 * z-order FG bits so a disabled OSD2 stops contributing and doesn't leave
 * stale order state for the next OSD1-only frame. Previously there was no
 * legacy disable_osd2 at all (meson_crtc->disable_osd2 was left NULL for the
 * non-G12A else-branch, unlike G12A's meson_g12a_crtc_disable_osd2) -- so
 * OSD2's postblend-enable bit was never cleared on disable on this platform
 * family, only ever set once and left set. */
static void meson_crtc_disable_osd2(struct meson_drm *priv)
{
	writel_bits_relaxed(VPP_OSD2_POSTBLEND | VPP_PRE_FG_OSD2 | VPP_POST_FG_OSD2, 0,
			    priv->io_base + _REG(VPP_MISC));
}

static void meson_g12a_crtc_enable_osd2(struct meson_drm *priv)
{
	writel_relaxed(priv->viu.osd_blend_din1_scope_h,
		       priv->io_base + _REG(VIU_OSD_BLEND_DIN1_SCOPE_H));
	writel_relaxed(priv->viu.osd_blend_din1_scope_v,
		       priv->io_base + _REG(VIU_OSD_BLEND_DIN1_SCOPE_V));
	/* VPP-level OSD2 blend window -- never programmed upstream, so it kept a bogus
	 * power-on default (start > end, devmem: 0x02d00000) and OSD2 rendered
	 * full-screen garbage instead of the committed window. */
	writel_relaxed(priv->viu.osd2_bld_h_scope,
		       priv->io_base + _REG(VPP_OSD2_BLD_H_SCOPE));
	writel_relaxed(priv->viu.osd2_bld_v_scope,
		       priv->io_base + _REG(VPP_OSD2_BLD_V_SCOPE));
	/*
	 * Do NOT route anything into the post-blend OSD2 port here. OSD2 is now
	 * composited inside the VIU OSD blender (DIN1, see set_osd_order) and
	 * reaches the post-blend as part of the OSD-dout via the OSD1 port. The
	 * old "writel_bits(3<<8, OSD2_BLEND_SRC_CTRL)" fed POSTBLD_SRC_OSD1 (the
	 * OSD-dout, copy-pasted from enable_osd1) into the above-OSD video slot,
	 * duplicating the dout full-screen -- which, once OSD2's buffer became
	 * opaque, flooded the frame. The OSD2 port stays clear (disable_osd2
	 * leaves it clear; set_vd_order owns it only when the OSD2 plane is off).
	 */
}

static void meson_g12a_crtc_disable_osd2(struct meson_drm *priv)
{
	writel_bits_relaxed(3 << 8, 0,
			    priv->io_base + _REG(OSD2_BLEND_SRC_CTRL));
}

static void meson_crtc_enable_vd1(struct meson_drm *priv)
{
	writel_bits_relaxed(VPP_VD1_PREBLEND | VPP_VD1_POSTBLEND |
			    VPP_COLOR_MNG_ENABLE,
			    VPP_VD1_PREBLEND | VPP_VD1_POSTBLEND |
			    VPP_COLOR_MNG_ENABLE,
			    priv->io_base + _REG(VPP_MISC));

	writel_bits_relaxed(VIU_CTRL0_AFBC_TO_VD1,
			    priv->viu.vd1_afbc ? VIU_CTRL0_AFBC_TO_VD1 : 0,
			    priv->io_base + _REG(VIU_MISC_CTRL0));
}

/* Symmetric to meson_crtc_enable_vd1: clear VD1 from the post-blend so a hidden
 * VD1 stops contributing. Without it the disabled VD1 layer keeps scanning out
 * (green, no data) -- VD2 had meson_crtc_disable_vd2 but VD1 had no disable. */
static void meson_crtc_disable_vd1(struct meson_drm *priv)
{
	writel_bits_relaxed(VPP_VD1_PREBLEND | VPP_VD1_POSTBLEND, 0,
			    priv->io_base + _REG(VPP_MISC));
}

static void meson_g12a_crtc_enable_vd1(struct meson_drm *priv)
{
	writel_relaxed(VD_BLEND_PREBLD_SRC_VD1 |
		       VD_BLEND_PREBLD_PREMULT_EN |
		       VD_BLEND_POSTBLD_SRC_VD1 |
		       VD_BLEND_POSTBLD_PREMULT_EN,
		       priv->io_base + _REG(VD1_BLEND_SRC_CTRL));

	writel_relaxed(priv->viu.vd1_afbc ?
		       (VD1_AXI_SEL_AFBC | AFBC_VD1_SEL) : 0,
		       priv->io_base + _REG(VD1_AFBCD0_MISC_CTRL));
}

/* Symmetric to meson_g12a_crtc_enable_vd1: tear down the VD1 fetch + blend so a
 * hidden VD1 stops contributing, mirroring the BSP video_hw.c disable_vd1_blend()
 * (clears the VD1 MIF general reg + the AFBC decoder enable) plus the post-blend
 * routing the upstream meson_overlay_atomic_disable() already drops:
 *   - VD1_BLEND_SRC_CTRL = 0          : VD1 out of its own post-blend port.
 *   - VD1_IF0_GEN_REG (G12A MIF, word VD1_IF0_GEN_REG+0x17b0=0x3200) = 0 : clears
 *     VD_ENABLE (bit0) so the VD1 MIF stops fetching (BSP rdma_wr vd_if0_gen_reg=0).
 *   - VD1_AFBCD0_MISC_CTRL = 0 and AFBC_ENABLE (0x1ae0) = 0 : disable the VD1 AFBC
 *     decoder (BSP rdma_wr afbc_enable=0; upstream left it set).
 *
 * NOTE the OSD2 port (OSD2_BLEND_SRC_CTRL) is deliberately NOT touched here. An
 * earlier version cleared VD_BLEND_POSTBLD_SRC_MASK on it to drop a VD1-routed-
 * above source, but that mask overlaps the OSD2 plane's OWN source (3 << 8) and
 * clobbered an active OSD2 plane (0x300 -> 0) whenever VD1 was disabled with OSD2
 * on. set_vd_order early-returns when OSD2 is active, so VD1-above-OSD + active-OSD2
 * never co-occur -- there is nothing for this disable to clean off the OSD2 port,
 * and writing it only corrupts the OSD2 plane. VPU-SWEEP-001 / OSD2-clobber fix. */
static void meson_g12a_crtc_disable_vd1(struct meson_drm *priv)
{
	writel_relaxed(0, priv->io_base + _REG(VD1_BLEND_SRC_CTRL));
	writel_relaxed(0, priv->io_base + _REG(VD1_IF0_GEN_REG + 0x17b0));
	writel_relaxed(0, priv->io_base + _REG(VD1_AFBCD0_MISC_CTRL));
	writel_relaxed(0, priv->io_base + _REG(AFBC_ENABLE));
}

/* VD2: independent second video plane (1:1, no AFBC -- Phase 1). The VD2 LAYER
 * ALPHA in VPP_MISC[26:18] must be driven to opaque (VPP_VD1_ALPHA_MAX): it
 * defaults to 0 (fully transparent), so enabling only the pre/post-blend leaves
 * VD2 invisible (a black PiP) even though its fetch/canvas/scope are correct --
 * the same omission the G12A/SM1 path (meson_g12a_crtc_enable_vd2) already fixes.
 * VD1's VPP_MISC alpha defaults opaque, so only VD2 needs this write. */
static void meson_crtc_enable_vd2(struct meson_drm *priv)
{
	writel_bits_relaxed(VPP_VD2_PREBLEND | VPP_VD2_POSTBLEND | VPP_VD2_ALPHA_MASK,
			    VPP_VD2_PREBLEND | VPP_VD2_POSTBLEND |
			    (VPP_VD1_ALPHA_MAX << VPP_VD2_ALPHA_SHIFT),
			    priv->io_base + _REG(VPP_MISC));
}

static void meson_crtc_disable_vd2(struct meson_drm *priv)
{
	writel_bits_relaxed(VPP_VD2_PREBLEND | VPP_VD2_POSTBLEND | VPP_VD2_ALPHA_MASK, 0,
			    priv->io_base + _REG(VPP_MISC));
}

static void meson_g12a_crtc_enable_vd2(struct meson_drm *priv)
{
	/* SM1/G12 use the *legacy* VD2 blend (BSP vd2_independ_blend_ctrl=0): the
	 * VD2 post-blend port needs the path-select bit BIT(20), and the VD2 layer
	 * alpha lives in VPP_MISC [26:18] (0x100 = opaque) together with the
	 * VPP_VD2_POSTBLEND enable -- NOT the independent BIT(17)/port-[31:20] path.
	 * Upstream set none of these, so VD2 had no path-sel and zero layer alpha ->
	 * fully transparent -> the PiP plane was black even though its
	 * fetch/canvas/scope/routing were all correct. Confirmed on demo-aml-s905d3-cc:
	 * VD1+VD2 PiP both composite once these are set. */
	writel_relaxed(VD_BLEND_POSTBLD_SRC_VD2 | VD_BLEND_VD2_PATH_SEL,
		       priv->io_base + _REG(VD2_BLEND_SRC_CTRL));
	writel_bits_relaxed(VPP_VD2_POSTBLEND | VPP_VD2_ALPHA_MASK,
			    VPP_VD2_POSTBLEND |
			    (VPP_VD1_ALPHA_MAX << VPP_VD2_ALPHA_SHIFT),
			    priv->io_base + _REG(VPP_MISC));

	/* AFBC-002: select the AFBCD1 input for VD2 (axi-sel + data-sel afbc),
	 * else the VD2 MIF. BSP video_hw.c vdx_path_select(): VD2 routing lives
	 * entirely in VD2_AFBCD1_MISC_CTRL on G12/SM1 (no VIU_MISC_CTRL0 bit). */
	writel_relaxed(priv->viu.vd2_afbc ?
		       (VD2_AXI_SEL_AFBC | AFBC_VD2_SEL) : 0,
		       priv->io_base + _REG(VD2_AFBCD1_MISC_CTRL));
}

static void meson_g12a_crtc_disable_vd2(struct meson_drm *priv)
{
	writel_relaxed(0, priv->io_base + _REG(VD2_BLEND_SRC_CTRL));
	writel_bits_relaxed(VPP_VD2_POSTBLEND | VPP_VD2_ALPHA_MASK, 0,
			    priv->io_base + _REG(VPP_MISC));
}

void meson_crtc_irq(struct meson_drm *priv)
{
	struct meson_crtc *meson_crtc = to_meson_crtc(priv->crtc);
	unsigned long flags;

	/*
	 * VRR-002: one step of the software variable-refresh loop, and the only
	 * place ENCP_VIDEO_MAX_LNCNT is written at runtime (audit item 1 -- it used
	 * to be the single direct-from-atomic_flush MMIO write in this driver;
	 * everything else defers through an *_commit flag consumed here, and the
	 * vendor likewise programs its VRR timing from a vsync IRQ handler).
	 *
	 * Runs first in the handler so the value lands as early as possible in the
	 * frame that has just begun -- the ENCP reloads max_lncnt at the boundary,
	 * so this write sets the length of THAT frame.
	 *
	 * Input: vrr_flip_pending, i.e. "did atomic_flush publish a new frame during
	 * the frame that just ended". Boolean, sampled on the hardware frame clock.
	 * No interval is measured, so nothing here depends on a vblank whose timing
	 * this code itself chose (the VRR-001 defect). The per-frame move is bounded
	 * by vrr_slew_lines (audit item 2), so the vtotal cannot jump the whole
	 * nominal..sink-floor span in one frame.
	 *
	 * Note the loop only advances while the vsync IRQ is enabled. On a fully
	 * idle pipe DRM drops the last vblank reference and meson_crtc_disable_vblank
	 * stops the interrupt, so the timing simply freezes at the last programmed
	 * value (which, the last commit having set vrr_flip_pending, is at or near
	 * the mode's nominal). That is a safe resting point, not a stall to fix.
	 *
	 * vrr_active / vrr_restore are only ever WRITTEN from process context; the
	 * unlocked read below is a fast-path gate and both are re-tested under the
	 * lock before anything is consumed.
	 */
	if (meson_crtc->vrr_active || meson_crtc->vrr_restore) {
		u32 lines = 0;

		spin_lock_irqsave(&meson_crtc->vrr_lock, flags);
		if (meson_crtc->vrr_active) {
			u32 cur = meson_crtc->vrr_cur_lines;
			u32 step = meson_crtc->vrr_slew_lines;

			if (meson_crtc->vrr_flip_pending) {
				meson_crtc->vrr_flip_pending = false;
				cur = (cur > step) ? cur - step : 0;
			} else {
				cur += step;
			}
			cur = clamp_t(u32, cur, meson_crtc->vrr_min_lines,
				      meson_crtc->vrr_max_lines);
			meson_crtc->vrr_cur_lines = cur;
			lines = cur;
		} else {
			lines = meson_crtc->vrr_nominal_lines;
			meson_crtc->vrr_restore = false;
		}
		spin_unlock_irqrestore(&meson_crtc->vrr_lock, flags);

		if (lines >= 2)
			writel_relaxed(lines - 1,
				       priv->io_base +
				       _REG(ENCP_VIDEO_MAX_LNCNT));
	}

	/*
	 * Audit #26: every atomic_update() (meson_plane.c, meson_overlay.c,
	 * meson_plane2.c, meson_overlay2.c) publishes its shadow state into
	 * priv->viu under event_lock. This handler read that same shadow
	 * state below with NO lock -- a real lock-ordering/coverage gap, and
	 * the OSD2+VD2 additions roughly doubled the shadow surface at risk.
	 * event_lock is already IRQ-safe (taken with spin_lock_irqsave from
	 * process context in atomic_update, and again further down in this
	 * same handler for vblank event delivery), so holding it here for the
	 * whole read+flush region matches the writer side with no new
	 * lock-ordering hazard. Everything inside is non-blocking MMIO
	 * (writel_relaxed/readl_relaxed, meson_canvas_config()'s own internal
	 * spinlock, meson_osd_afbcd.c's ops -- none sleep) plus the bounded
	 * atomic poll in meson_viu_apply_gamma() (#9), so nothing here can
	 * block while the lock is held.
	 */
	spin_lock_irqsave(&priv->drm->event_lock, flags);

	/*
	 * Audit #8: apply the cached CTM/gamma LUT here, deferred from
	 * atomic_flush like every other register in this driver.
	 */
	if (priv->viu.cm_commit) {
		meson_viu_apply_ctm(priv);
		meson_viu_apply_gamma(priv);
		priv->viu.cm_commit = false;
	}

	/*
	 * #13 fix: unconditional per-vsync flush of the deferred PQ/HDR2
	 * work -- see meson_overlay_pq_flush() for the full deferral
	 * rationale. Documented hard-IRQ-safe (no sleeping) by its own
	 * author, so calling it under this spinlock is safe. Order relative
	 * to the OSD/VD register flush below doesn't matter -- PQ/HDR2/
	 * blackext/WB are independent register blocks.
	 */
	meson_overlay_pq_flush(priv);

	/* Update the OSD registers */
	if (priv->viu.osd1_enabled && priv->viu.osd1_commit) {
		writel_relaxed(priv->viu.osd1_ctrl_stat,
				priv->io_base + _REG(VIU_OSD1_CTRL_STAT));
		writel_relaxed(priv->viu.osd1_ctrl_stat2,
				priv->io_base + _REG(VIU_OSD1_CTRL_STAT2));
		writel_relaxed(priv->viu.osd1_blk0_cfg[0],
				priv->io_base + _REG(VIU_OSD1_BLK0_CFG_W0));
		writel_relaxed(priv->viu.osd1_blk0_cfg[1],
				priv->io_base + _REG(VIU_OSD1_BLK0_CFG_W1));
		writel_relaxed(priv->viu.osd1_blk0_cfg[2],
				priv->io_base + _REG(VIU_OSD1_BLK0_CFG_W2));
		writel_relaxed(priv->viu.osd1_blk0_cfg[3],
				priv->io_base + _REG(VIU_OSD1_BLK0_CFG_W3));
		writel_relaxed(priv->viu.osd1_blk0_cfg[4],
				priv->io_base + _REG(VIU_OSD1_BLK0_CFG_W4));

		if (priv->viu.osd1_afbcd) {
			if (meson_crtc->enable_osd1_afbc)
				meson_crtc->enable_osd1_afbc(priv);
		} else {
			if (meson_crtc->disable_osd1_afbc)
				meson_crtc->disable_osd1_afbc(priv);
			if (priv->afbcd.ops) {
				priv->afbcd.ops->reset(priv);
				priv->afbcd.ops->disable(priv);
			}
			meson_crtc->vsync_forced = false;
		}

		writel_relaxed(priv->viu.osd_sc_ctrl0,
				priv->io_base + _REG(VPP_OSD_SC_CTRL0));
		writel_relaxed(priv->viu.osd_sc_i_wh_m1,
				priv->io_base + _REG(VPP_OSD_SCI_WH_M1));
		writel_relaxed(priv->viu.osd_sc_o_h_start_end,
				priv->io_base + _REG(VPP_OSD_SCO_H_START_END));
		writel_relaxed(priv->viu.osd_sc_o_v_start_end,
				priv->io_base + _REG(VPP_OSD_SCO_V_START_END));
		writel_relaxed(priv->viu.osd_sc_v_ini_phase,
				priv->io_base + _REG(VPP_OSD_VSC_INI_PHASE));
		writel_relaxed(priv->viu.osd_sc_v_phase_step,
				priv->io_base + _REG(VPP_OSD_VSC_PHASE_STEP));
		writel_relaxed(priv->viu.osd_sc_h_ini_phase,
				priv->io_base + _REG(VPP_OSD_HSC_INI_PHASE));
		writel_relaxed(priv->viu.osd_sc_h_phase_step,
				priv->io_base + _REG(VPP_OSD_HSC_PHASE_STEP));
		writel_relaxed(priv->viu.osd_sc_h_ctrl0,
				priv->io_base + _REG(VPP_OSD_HSC_CTRL0));
		writel_relaxed(priv->viu.osd_sc_v_ctrl0,
				priv->io_base + _REG(VPP_OSD_VSC_CTRL0));

		if (!priv->viu.osd1_afbcd)
			meson_canvas_config(priv->canvas, priv->canvas_id_osd1,
					    priv->viu.osd1_addr,
					    priv->viu.osd1_stride,
					    priv->viu.osd1_height,
					    MESON_CANVAS_WRAP_NONE,
					    MESON_CANVAS_BLKMODE_LINEAR, 0);

		/* Enable OSD1 */
		if (meson_crtc->enable_osd1)
			meson_crtc->enable_osd1(priv);

		if (priv->viu.osd1_afbcd) {
			priv->afbcd.ops->reset(priv);
			priv->afbcd.ops->setup(priv);
			priv->afbcd.ops->enable(priv);
			meson_crtc->vsync_forced = true;
		}

		priv->viu.osd1_commit = false;
	}

	/*
	 * #23 fix: hand off any pending writeback capture jobs now that
	 * priv->viu.osd1_addr (whether just latched above, or unchanged
	 * from an earlier vsync) reflects the buffer actually driving OSD1
	 * scanout. Deliberately outside the if() above -- runs on every
	 * vsync, including ones where OSD1 had no pending change, so a
	 * writeback job queued between two OSD1 commits still completes.
	 * See meson_writeback_vsync_latched() for the full rationale.
	 */
	meson_writeback_vsync_latched(priv);

	/* Update the OSD2 registers */
	if (priv->viu.osd2_enabled && priv->viu.osd2_commit) {
		writel_relaxed(priv->viu.osd2_ctrl_stat,
			       priv->io_base + _REG(VIU_OSD2_CTRL_STAT));
		writel_relaxed(priv->viu.osd2_ctrl_stat2,
			       priv->io_base + _REG(VIU_OSD2_CTRL_STAT2));
		writel_relaxed(priv->viu.osd2_blk0_cfg[0],
			       priv->io_base + _REG(VIU_OSD2_BLK0_CFG_W0));
		writel_relaxed(priv->viu.osd2_blk0_cfg[1],
			       priv->io_base + _REG(VIU_OSD2_BLK0_CFG_W1));
		writel_relaxed(priv->viu.osd2_blk0_cfg[2],
			       priv->io_base + _REG(VIU_OSD2_BLK0_CFG_W2));
		writel_relaxed(priv->viu.osd2_blk0_cfg[3],
			       priv->io_base + _REG(VIU_OSD2_BLK0_CFG_W3));
		writel_relaxed(priv->viu.osd2_blk0_cfg[4],
			       priv->io_base + _REG(VIU_OSD2_BLK0_CFG_W4));

		meson_canvas_config(priv->canvas, priv->canvas_id_osd2,
				    priv->viu.osd2_addr,
				    priv->viu.osd2_stride,
				    priv->viu.osd2_height,
				    MESON_CANVAS_WRAP_NONE,
				    MESON_CANVAS_BLKMODE_LINEAR, 0);

		/* Write OSD2 dedicated scaler registers (G12A+) */
		if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A)) {
			writel_relaxed(priv->viu.osd2_sc_ctrl0,
				       priv->io_base + _REG(OSD2_SC_CTRL0));
			writel_relaxed(priv->viu.osd2_sc_i_wh_m1,
				       priv->io_base + _REG(OSD2_SCI_WH_M1));
			writel_relaxed(priv->viu.osd2_sc_o_h_start_end,
				       priv->io_base + _REG(OSD2_SCO_H_START_END));
			writel_relaxed(priv->viu.osd2_sc_o_v_start_end,
				       priv->io_base + _REG(OSD2_SCO_V_START_END));
			writel_relaxed(priv->viu.osd2_sc_v_ini_phase,
				       priv->io_base + _REG(OSD2_VSC_INI_PHASE));
			writel_relaxed(priv->viu.osd2_sc_v_phase_step,
				       priv->io_base + _REG(OSD2_VSC_PHASE_STEP));
			writel_relaxed(priv->viu.osd2_sc_h_ini_phase,
				       priv->io_base + _REG(OSD2_HSC_INI_PHASE));
			writel_relaxed(priv->viu.osd2_sc_h_phase_step,
				       priv->io_base + _REG(OSD2_HSC_PHASE_STEP));
			writel_relaxed(priv->viu.osd2_sc_h_ctrl0,
				       priv->io_base + _REG(OSD2_HSC_CTRL0));
			writel_relaxed(priv->viu.osd2_sc_v_ctrl0,
				       priv->io_base + _REG(OSD2_VSC_CTRL0));
		}

		if (meson_crtc->enable_osd2)
			meson_crtc->enable_osd2(priv);

		priv->viu.osd2_commit = false;
	} else if (!priv->viu.osd2_enabled && meson_crtc->disable_osd2) {
		/* Bug 4: only tear down on the enabled->disabled edge, not on
		 * every vsync the plane happens to still be off. */
		if (meson_crtc->osd2_enabled_prev) {
			meson_crtc->disable_osd2(priv);

			/* Bug 3: OSD2's dedicated scaler (G12A+, programmed only
			 * in the osd2_enabled branch above) is otherwise never
			 * cleared, so it stays live with the last active plane's
			 * stale window/ratio config. Disable the scale-enable bit. */
			if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A))
				writel_bits_relaxed(SC_CTRL0_PATH_EN, 0,
						    priv->io_base +
						    _REG(OSD2_SC_CTRL0));
		}
	}
	meson_crtc->osd2_enabled_prev = priv->viu.osd2_enabled;

	/*
	 * Bug 1: program the OSD1/OSD2 blend z-order from zpos regardless of
	 * which specific OSD plane committed this frame -- the routing depends
	 * on the CURRENT enabled state of BOTH OSD planes together, not just
	 * OSD1's. This used to run only inside the osd1_enabled+commit branch
	 * above, so an OSD1-off/OSD2-on scene (a valid compositor state: primary
	 * off, overlay on) never programmed the blender at all and rendered
	 * blank, since the blender was left in its single-input OSD1-only
	 * default. Mirrors the unconditional call to set_vd_order below.
	 */
	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A))
		meson_g12a_crtc_set_osd_order(priv);

	/* Update the VD1 registers */
	if (priv->viu.vd1_enabled && priv->viu.vd1_commit) {

		if (priv->viu.vd1_afbc) {
			writel_relaxed(priv->viu.vd1_afbc_head_addr,
				       priv->io_base +
				       _REG(AFBC_HEAD_BADDR));
			writel_relaxed(priv->viu.vd1_afbc_body_addr,
				       priv->io_base +
				       _REG(AFBC_BODY_BADDR));
			writel_relaxed(priv->viu.vd1_afbc_en,
				       priv->io_base +
				       _REG(AFBC_ENABLE));
			writel_relaxed(priv->viu.vd1_afbc_mode,
				       priv->io_base +
				       _REG(AFBC_MODE));
			writel_relaxed(priv->viu.vd1_afbc_size_in,
				       priv->io_base +
				       _REG(AFBC_SIZE_IN));
			writel_relaxed(priv->viu.vd1_afbc_dec_def_color,
				       priv->io_base +
				       _REG(AFBC_DEC_DEF_COLOR));
			writel_relaxed(priv->viu.vd1_afbc_conv_ctrl,
				       priv->io_base +
				       _REG(AFBC_CONV_CTRL));
			writel_relaxed(priv->viu.vd1_afbc_size_out,
				       priv->io_base +
				       _REG(AFBC_SIZE_OUT));
			writel_relaxed(priv->viu.vd1_afbc_vd_cfmt_ctrl,
				       priv->io_base +
				       _REG(AFBC_VD_CFMT_CTRL));
			writel_relaxed(priv->viu.vd1_afbc_vd_cfmt_w,
				       priv->io_base +
				       _REG(AFBC_VD_CFMT_W));
			writel_relaxed(priv->viu.vd1_afbc_mif_hor_scope,
				       priv->io_base +
				       _REG(AFBC_MIF_HOR_SCOPE));
			writel_relaxed(priv->viu.vd1_afbc_mif_ver_scope,
				       priv->io_base +
				       _REG(AFBC_MIF_VER_SCOPE));
			writel_relaxed(priv->viu.vd1_afbc_pixel_hor_scope,
				       priv->io_base+
				       _REG(AFBC_PIXEL_HOR_SCOPE));
			writel_relaxed(priv->viu.vd1_afbc_pixel_ver_scope,
				       priv->io_base +
				       _REG(AFBC_PIXEL_VER_SCOPE));
			writel_relaxed(priv->viu.vd1_afbc_vd_cfmt_h,
				       priv->io_base +
				       _REG(AFBC_VD_CFMT_H));
		} else {
			switch (priv->viu.vd1_planes) {
			case 3:
				meson_canvas_config(priv->canvas,
						    priv->canvas_id_vd1_2,
						    priv->viu.vd1_addr2,
						    priv->viu.vd1_stride2,
						    priv->viu.vd1_height2,
						    MESON_CANVAS_WRAP_NONE,
						    MESON_CANVAS_BLKMODE_LINEAR,
						    MESON_CANVAS_ENDIAN_SWAP64);
				fallthrough;
			case 2:
				meson_canvas_config(priv->canvas,
						    priv->canvas_id_vd1_1,
						    priv->viu.vd1_addr1,
						    priv->viu.vd1_stride1,
						    priv->viu.vd1_height1,
						    MESON_CANVAS_WRAP_NONE,
						    MESON_CANVAS_BLKMODE_LINEAR,
						    MESON_CANVAS_ENDIAN_SWAP64);
				fallthrough;
			case 1:
				meson_canvas_config(priv->canvas,
						    priv->canvas_id_vd1_0,
						    priv->viu.vd1_addr0,
						    priv->viu.vd1_stride0,
						    priv->viu.vd1_height0,
						    MESON_CANVAS_WRAP_NONE,
						    MESON_CANVAS_BLKMODE_LINEAR,
						    MESON_CANVAS_ENDIAN_SWAP64);
			}

			writel_relaxed(0, priv->io_base + _REG(AFBC_ENABLE));
		}

		/*
		 * Only program the VD1 registers here. VD2 is an independent
		 * plane with its own flush block (below) -- it is no longer
		 * mirrored from VD1.
		 */
		writel_relaxed(priv->viu.vd1_if0_gen_reg,
				priv->io_base + meson_crtc->viu_offset +
				_REG(VD1_IF0_GEN_REG));
		writel_relaxed(priv->viu.vd1_if0_gen_reg2,
				priv->io_base + meson_crtc->viu_offset +
				_REG(VD1_IF0_GEN_REG2));
		writel_relaxed(priv->viu.viu_vd1_fmt_ctrl,
				priv->io_base + meson_crtc->viu_offset +
				_REG(VIU_VD1_FMT_CTRL));
		writel_relaxed(priv->viu.viu_vd1_fmt_w,
				priv->io_base + meson_crtc->viu_offset +
				_REG(VIU_VD1_FMT_W));
		writel_relaxed(priv->viu.vd1_if0_canvas0,
				priv->io_base + meson_crtc->viu_offset +
				_REG(VD1_IF0_CANVAS0));
		writel_relaxed(priv->viu.vd1_if0_canvas0,
				priv->io_base + meson_crtc->viu_offset +
				_REG(VD1_IF0_CANVAS1));
		writel_relaxed(priv->viu.vd1_if0_luma_x0,
				priv->io_base + meson_crtc->viu_offset +
				_REG(VD1_IF0_LUMA_X0));
		writel_relaxed(priv->viu.vd1_if0_luma_x0,
				priv->io_base + meson_crtc->viu_offset +
				_REG(VD1_IF0_LUMA_X1));
		writel_relaxed(priv->viu.vd1_if0_luma_y0,
				priv->io_base + meson_crtc->viu_offset +
				_REG(VD1_IF0_LUMA_Y0));
		writel_relaxed(priv->viu.vd1_if0_luma_y0,
				priv->io_base + meson_crtc->viu_offset +
				_REG(VD1_IF0_LUMA_Y1));
		writel_relaxed(priv->viu.vd1_if0_chroma_x0,
				priv->io_base + meson_crtc->viu_offset +
				_REG(VD1_IF0_CHROMA_X0));
		writel_relaxed(priv->viu.vd1_if0_chroma_x0,
				priv->io_base + meson_crtc->viu_offset +
				_REG(VD1_IF0_CHROMA_X1));
		writel_relaxed(priv->viu.vd1_if0_chroma_y0,
				priv->io_base + meson_crtc->viu_offset +
				_REG(VD1_IF0_CHROMA_Y0));
		writel_relaxed(priv->viu.vd1_if0_chroma_y0,
				priv->io_base + meson_crtc->viu_offset +
				_REG(VD1_IF0_CHROMA_Y1));
		writel_relaxed(priv->viu.vd1_if0_repeat_loop,
				priv->io_base + meson_crtc->viu_offset +
				_REG(VD1_IF0_RPT_LOOP));
		writel_relaxed(priv->viu.vd1_if0_luma0_rpt_pat,
				priv->io_base + meson_crtc->viu_offset +
				_REG(VD1_IF0_LUMA0_RPT_PAT));
		writel_relaxed(priv->viu.vd1_if0_luma0_rpt_pat,
				priv->io_base + meson_crtc->viu_offset +
				_REG(VD1_IF0_LUMA1_RPT_PAT));
		writel_relaxed(priv->viu.vd1_if0_chroma0_rpt_pat,
				priv->io_base + meson_crtc->viu_offset +
				_REG(VD1_IF0_CHROMA0_RPT_PAT));
		writel_relaxed(priv->viu.vd1_if0_chroma0_rpt_pat,
				priv->io_base + meson_crtc->viu_offset +
				_REG(VD1_IF0_CHROMA1_RPT_PAT));
		writel_relaxed(0, priv->io_base + meson_crtc->viu_offset +
				_REG(VD1_IF0_LUMA_PSEL));
		writel_relaxed(0, priv->io_base + meson_crtc->viu_offset +
				_REG(VD1_IF0_CHROMA_PSEL));
		writel_relaxed(priv->viu.vd1_range_map_y,
				priv->io_base + meson_crtc->viu_offset +
				_REG(VD1_IF0_RANGE_MAP_Y));
		writel_relaxed(priv->viu.vd1_range_map_cb,
				priv->io_base + meson_crtc->viu_offset +
				_REG(VD1_IF0_RANGE_MAP_CB));
		writel_relaxed(priv->viu.vd1_range_map_cr,
				priv->io_base + meson_crtc->viu_offset +
				_REG(VD1_IF0_RANGE_MAP_CR));
		writel_relaxed(VPP_VSC_BANK_LENGTH(4) |
			       VPP_HSC_BANK_LENGTH(4) |
			       VPP_SC_VD_EN_ENABLE |
			       VPP_SC_TOP_EN_ENABLE |
			       VPP_SC_HSC_EN_ENABLE |
			       VPP_SC_VSC_EN_ENABLE,
				priv->io_base + _REG(VPP_SC_MISC));
		writel_relaxed(priv->viu.vpp_pic_in_height,
				priv->io_base + _REG(VPP_PIC_IN_HEIGHT));
		writel_relaxed(priv->viu.vpp_postblend_vd1_h_start_end,
			priv->io_base + _REG(VPP_POSTBLEND_VD1_H_START_END));
		writel_relaxed(priv->viu.vpp_postblend_vd1_v_start_end,
			priv->io_base + _REG(VPP_POSTBLEND_VD1_V_START_END));
		writel_relaxed(priv->viu.vpp_hsc_region12_startp,
				priv->io_base + _REG(VPP_HSC_REGION12_STARTP));
		writel_relaxed(priv->viu.vpp_hsc_region34_startp,
				priv->io_base + _REG(VPP_HSC_REGION34_STARTP));
		writel_relaxed(priv->viu.vpp_hsc_region4_endp,
				priv->io_base + _REG(VPP_HSC_REGION4_ENDP));
		writel_relaxed(priv->viu.vpp_hsc_start_phase_step,
				priv->io_base + _REG(VPP_HSC_START_PHASE_STEP));
		writel_relaxed(priv->viu.vpp_hsc_region1_phase_slope,
			priv->io_base + _REG(VPP_HSC_REGION1_PHASE_SLOPE));
		writel_relaxed(priv->viu.vpp_hsc_region3_phase_slope,
			priv->io_base + _REG(VPP_HSC_REGION3_PHASE_SLOPE));
		writel_relaxed(priv->viu.vpp_line_in_length,
				priv->io_base + _REG(VPP_LINE_IN_LENGTH));
		writel_relaxed(priv->viu.vpp_preblend_h_size,
				priv->io_base + _REG(VPP_PREBLEND_H_SIZE));
		writel_relaxed(priv->viu.vpp_vsc_region12_startp,
				priv->io_base + _REG(VPP_VSC_REGION12_STARTP));
		writel_relaxed(priv->viu.vpp_vsc_region34_startp,
				priv->io_base + _REG(VPP_VSC_REGION34_STARTP));
		writel_relaxed(priv->viu.vpp_vsc_region4_endp,
				priv->io_base + _REG(VPP_VSC_REGION4_ENDP));
		writel_relaxed(priv->viu.vpp_vsc_start_phase_step,
				priv->io_base + _REG(VPP_VSC_START_PHASE_STEP));
		writel_relaxed(priv->viu.vpp_vsc_ini_phase,
				priv->io_base + _REG(VPP_VSC_INI_PHASE));
		writel_relaxed(priv->viu.vpp_vsc_phase_ctrl,
				priv->io_base + _REG(VPP_VSC_PHASE_CTRL));
		writel_relaxed(priv->viu.vpp_hsc_phase_ctrl,
				priv->io_base + _REG(VPP_HSC_PHASE_CTRL));
		writel_relaxed(0x42, priv->io_base + _REG(VPP_SCALE_COEF_IDX));

		/* Enable VD1 */
		if (meson_crtc->enable_vd1)
			meson_crtc->enable_vd1(priv);

		priv->viu.vd1_commit = false;
	} else if (!priv->viu.vd1_enabled && meson_crtc->disable_vd1) {
		/* Bug 4: only tear down on the enabled->disabled edge, not on
		 * every vsync the plane happens to still be off. */
		if (meson_crtc->vd1_enabled_prev)
			meson_crtc->disable_vd1(priv);
	}
	meson_crtc->vd1_enabled_prev = priv->viu.vd1_enabled;

	/*
	 * Update the VD2 registers (independent second video plane).
	 * Runs after the VD1 block so that VPP_BLEND_VD2_* -- which the VD1
	 * block still mirror-writes from VD1's interlace state -- is overridden
	 * here with VD2's own destination window. VD2 is 1:1 (no scaler), so it
	 * does not touch the shared VPP scaler that VD1 owns.
	 */
	if (priv->viu.vd2_enabled && priv->viu.vd2_commit) {

		if (priv->viu.vd2_afbc) {
			/* AFBC-002: program the VD2 AFBC decoder (0x3180 block),
			 * mirroring the VD1 AFBC flush above. */
			writel_relaxed(priv->viu.vd2_afbc_head_addr,
				       priv->io_base + _REG(VD2_AFBC_HEAD_BADDR));
			writel_relaxed(priv->viu.vd2_afbc_body_addr,
				       priv->io_base + _REG(VD2_AFBC_BODY_BADDR));
			writel_relaxed(priv->viu.vd2_afbc_en,
				       priv->io_base + _REG(VD2_AFBC_ENABLE));
			writel_relaxed(priv->viu.vd2_afbc_mode,
				       priv->io_base + _REG(VD2_AFBC_MODE));
			writel_relaxed(priv->viu.vd2_afbc_size_in,
				       priv->io_base + _REG(VD2_AFBC_SIZE_IN));
			writel_relaxed(priv->viu.vd2_afbc_dec_def_color,
				       priv->io_base + _REG(VD2_AFBC_DEC_DEF_COLOR));
			writel_relaxed(priv->viu.vd2_afbc_conv_ctrl,
				       priv->io_base + _REG(VD2_AFBC_CONV_CTRL));
			writel_relaxed(priv->viu.vd2_afbc_size_out,
				       priv->io_base + _REG(VD2_AFBC_SIZE_OUT));
			writel_relaxed(priv->viu.vd2_afbc_vd_cfmt_ctrl,
				       priv->io_base + _REG(VD2_AFBC_VD_CFMT_CTRL));
			writel_relaxed(priv->viu.vd2_afbc_vd_cfmt_w,
				       priv->io_base + _REG(VD2_AFBC_VD_CFMT_W));
			writel_relaxed(priv->viu.vd2_afbc_mif_hor_scope,
				       priv->io_base + _REG(VD2_AFBC_MIF_HOR_SCOPE));
			writel_relaxed(priv->viu.vd2_afbc_mif_ver_scope,
				       priv->io_base + _REG(VD2_AFBC_MIF_VER_SCOPE));
			writel_relaxed(priv->viu.vd2_afbc_pixel_hor_scope,
				       priv->io_base + _REG(VD2_AFBC_PIXEL_HOR_SCOPE));
			writel_relaxed(priv->viu.vd2_afbc_pixel_ver_scope,
				       priv->io_base + _REG(VD2_AFBC_PIXEL_VER_SCOPE));
			writel_relaxed(priv->viu.vd2_afbc_vd_cfmt_h,
				       priv->io_base + _REG(VD2_AFBC_VD_CFMT_H));
		} else {
			switch (priv->viu.vd2_planes) {
			case 3:
				meson_canvas_config(priv->canvas,
						    priv->canvas_id_vd2_2,
						    priv->viu.vd2_addr2,
						    priv->viu.vd2_stride2,
						    priv->viu.vd2_height2,
						    MESON_CANVAS_WRAP_NONE,
						    MESON_CANVAS_BLKMODE_LINEAR,
						    MESON_CANVAS_ENDIAN_SWAP64);
				fallthrough;
			case 2:
				meson_canvas_config(priv->canvas,
						    priv->canvas_id_vd2_1,
						    priv->viu.vd2_addr1,
						    priv->viu.vd2_stride1,
						    priv->viu.vd2_height1,
						    MESON_CANVAS_WRAP_NONE,
						    MESON_CANVAS_BLKMODE_LINEAR,
						    MESON_CANVAS_ENDIAN_SWAP64);
				fallthrough;
			case 1:
				meson_canvas_config(priv->canvas,
						    priv->canvas_id_vd2_0,
						    priv->viu.vd2_addr0,
						    priv->viu.vd2_stride0,
						    priv->viu.vd2_height0,
						    MESON_CANVAS_WRAP_NONE,
						    MESON_CANVAS_BLKMODE_LINEAR,
						    MESON_CANVAS_ENDIAN_SWAP64);
			}

			writel_relaxed(0, priv->io_base + _REG(VD2_AFBC_ENABLE));
		}

		writel_relaxed(priv->viu.vd2_if0_gen_reg,
			priv->io_base + meson_crtc->viu_offset +
			_REG(VD2_IF0_GEN_REG));
		writel_relaxed(priv->viu.vd2_if0_gen_reg2,
			priv->io_base + meson_crtc->viu_offset +
			_REG(VD2_IF0_GEN_REG2));
		writel_relaxed(priv->viu.viu_vd2_fmt_ctrl,
			priv->io_base + meson_crtc->viu_offset +
			_REG(VIU_VD2_FMT_CTRL));
		writel_relaxed(priv->viu.viu_vd2_fmt_w,
			priv->io_base + meson_crtc->viu_offset +
			_REG(VIU_VD2_FMT_W));
		writel_relaxed(priv->viu.vd2_if0_canvas0,
			priv->io_base + meson_crtc->viu_offset +
			_REG(VD2_IF0_CANVAS0));
		writel_relaxed(priv->viu.vd2_if0_canvas0,
			priv->io_base + meson_crtc->viu_offset +
			_REG(VD2_IF0_CANVAS1));
		writel_relaxed(priv->viu.vd2_if0_luma_x0,
			priv->io_base + meson_crtc->viu_offset +
			_REG(VD2_IF0_LUMA_X0));
		writel_relaxed(priv->viu.vd2_if0_luma_x0,
			priv->io_base + meson_crtc->viu_offset +
			_REG(VD2_IF0_LUMA_X1));
		writel_relaxed(priv->viu.vd2_if0_luma_y0,
			priv->io_base + meson_crtc->viu_offset +
			_REG(VD2_IF0_LUMA_Y0));
		writel_relaxed(priv->viu.vd2_if0_luma_y0,
			priv->io_base + meson_crtc->viu_offset +
			_REG(VD2_IF0_LUMA_Y1));
		writel_relaxed(priv->viu.vd2_if0_chroma_x0,
			priv->io_base + meson_crtc->viu_offset +
			_REG(VD2_IF0_CHROMA_X0));
		writel_relaxed(priv->viu.vd2_if0_chroma_x0,
			priv->io_base + meson_crtc->viu_offset +
			_REG(VD2_IF0_CHROMA_X1));
		writel_relaxed(priv->viu.vd2_if0_chroma_y0,
			priv->io_base + meson_crtc->viu_offset +
			_REG(VD2_IF0_CHROMA_Y0));
		writel_relaxed(priv->viu.vd2_if0_chroma_y0,
			priv->io_base + meson_crtc->viu_offset +
			_REG(VD2_IF0_CHROMA_Y1));
		writel_relaxed(priv->viu.vd2_if0_repeat_loop,
			priv->io_base + meson_crtc->viu_offset +
			_REG(VD2_IF0_RPT_LOOP));
		writel_relaxed(priv->viu.vd2_if0_luma0_rpt_pat,
			priv->io_base + meson_crtc->viu_offset +
			_REG(VD2_IF0_LUMA0_RPT_PAT));
		writel_relaxed(priv->viu.vd2_if0_luma0_rpt_pat,
			priv->io_base + meson_crtc->viu_offset +
			_REG(VD2_IF0_LUMA1_RPT_PAT));
		writel_relaxed(priv->viu.vd2_if0_chroma0_rpt_pat,
			priv->io_base + meson_crtc->viu_offset +
			_REG(VD2_IF0_CHROMA0_RPT_PAT));
		writel_relaxed(priv->viu.vd2_if0_chroma0_rpt_pat,
			priv->io_base + meson_crtc->viu_offset +
			_REG(VD2_IF0_CHROMA1_RPT_PAT));
		writel_relaxed(0, priv->io_base + meson_crtc->viu_offset +
			_REG(VD2_IF0_LUMA_PSEL));
		writel_relaxed(0, priv->io_base + meson_crtc->viu_offset +
			_REG(VD2_IF0_CHROMA_PSEL));

		/* VD2 destination window into the preblend (1:1, no scaler) */
		writel_relaxed(priv->viu.vd2_dst_h_start_end,
			priv->io_base + _REG(VPP_BLEND_VD2_H_START_END));
		writel_relaxed(priv->viu.vd2_dst_v_start_end,
			priv->io_base + _REG(VPP_BLEND_VD2_V_START_END));

		/* Enable VD2 */
		if (meson_crtc->enable_vd2)
			meson_crtc->enable_vd2(priv);

		priv->viu.vd2_commit = false;
	} else if (!priv->viu.vd2_enabled && meson_crtc->disable_vd2) {
		/* Bug 4: only tear down on the enabled->disabled edge, not on
		 * every vsync the plane happens to still be off. */
		if (meson_crtc->vd2_enabled_prev)
			meson_crtc->disable_vd2(priv);
	}
	meson_crtc->vd2_enabled_prev = priv->viu.vd2_enabled;

	/* Phase 2 stage 2: apply video<->OSD group order from zpos (after the
	 * per-layer enable_* writes above have set the default selectors). */
	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A))
		meson_g12a_crtc_set_vd_order(priv);

	/* Audit #26: matches the spin_lock_irqsave() above the OSD update
	 * block. Must run before EVERY return from this point on, including
	 * the vsync_disabled early-out immediately below. */
	spin_unlock_irqrestore(&priv->drm->event_lock, flags);

	if (meson_crtc->vsync_disabled)
		return;

	/* VPP post-blend CRC: report previous frame's result and trigger next */
	if (meson_crtc->vpp_crc_enable) {
		u32 crc = readl_relaxed(priv->io_base + _REG(VPP_RO_CRCSUM));

		drm_crtc_add_crc_entry(priv->crtc, true,
				       drm_crtc_accurate_vblank_count(priv->crtc),
				       &crc);
		/* Trigger CRC computation for the next frame */
		writel_relaxed(1, priv->io_base + _REG(VPP_CRC_CHK));
	}

	drm_crtc_handle_vblank(priv->crtc);

	spin_lock_irqsave(&priv->drm->event_lock, flags);
	if (meson_crtc->event) {
		drm_crtc_send_vblank_event(priv->crtc, meson_crtc->event);
		drm_crtc_vblank_put(priv->crtc);
		meson_crtc->event = NULL;
	}
	spin_unlock_irqrestore(&priv->drm->event_lock, flags);
}

void meson_crtc_set_vrr_range(struct drm_crtc *crtc, u16 min_vfreq,
			      u16 max_vfreq)
{
	struct meson_crtc *meson_crtc = to_meson_crtc(crtc);

	/* A degenerate range (min >= max) is treated as no usable VRR range. */
	if (min_vfreq && max_vfreq && min_vfreq < max_vfreq) {
		meson_crtc->vrr_min_vfreq = min_vfreq;
		meson_crtc->vrr_max_vfreq = max_vfreq;
	} else {
		meson_crtc->vrr_min_vfreq = 0;
		meson_crtc->vrr_max_vfreq = 0;
	}
}

int meson_crtc_create(struct meson_drm *priv)
{
	struct meson_crtc *meson_crtc;
	struct drm_crtc *crtc;
	int ret;

	meson_crtc = devm_kzalloc(priv->drm->dev, sizeof(*meson_crtc),
				  GFP_KERNEL);
	if (!meson_crtc)
		return -ENOMEM;

	meson_crtc->priv = priv;
	spin_lock_init(&meson_crtc->vrr_lock);
	crtc = &meson_crtc->base;
	ret = drm_crtc_init_with_planes(priv->drm, crtc,
					priv->primary_plane, NULL,
					&meson_crtc_funcs, "meson_crtc");
	if (ret) {
		dev_err(priv->drm->dev, "Failed to init CRTC\n");
		return ret;
	}

	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A)) {
		meson_crtc->enable_osd1 = meson_g12a_crtc_enable_osd1;
		meson_crtc->enable_osd2 = meson_g12a_crtc_enable_osd2;
		meson_crtc->disable_osd2 = meson_g12a_crtc_disable_osd2;
		meson_crtc->enable_vd1 = meson_g12a_crtc_enable_vd1;
		meson_crtc->disable_vd1 = meson_g12a_crtc_disable_vd1;
		meson_crtc->enable_vd2 = meson_g12a_crtc_enable_vd2;
		meson_crtc->disable_vd2 = meson_g12a_crtc_disable_vd2;
		meson_crtc->viu_offset = MESON_G12A_VIU_OFFSET;
		meson_crtc->enable_osd1_afbc =
					meson_crtc_g12a_enable_osd1_afbc;
		meson_crtc->disable_osd1_afbc =
					meson_viu_g12a_disable_osd1_afbc;
		drm_crtc_helper_add(crtc, &meson_g12a_crtc_helper_funcs);
	} else {
		meson_crtc->enable_osd1 = meson_crtc_enable_osd1;
		meson_crtc->enable_osd2 = meson_crtc_enable_osd2;
		meson_crtc->disable_osd2 = meson_crtc_disable_osd2;
		meson_crtc->enable_vd1 = meson_crtc_enable_vd1;
		meson_crtc->disable_vd1 = meson_crtc_disable_vd1;
		meson_crtc->enable_vd2 = meson_crtc_enable_vd2;
		meson_crtc->disable_vd2 = meson_crtc_disable_vd2;
		if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXM)) {
			meson_crtc->enable_osd1_afbc =
					meson_viu_gxm_enable_osd1_afbc;
			meson_crtc->disable_osd1_afbc =
					meson_viu_gxm_disable_osd1_afbc;
		}
		drm_crtc_helper_add(crtc, &meson_crtc_helper_funcs);
	}

	/* degamma_size=0, has_ctm=true, gamma_size=256 (VPP gamma LUT) */
	drm_crtc_enable_color_mgmt(crtc, 0, true, 256);

	/*
	 * VRR-002: expose the standard VRR_ENABLED CRTC property so a VRR-aware
	 * userspace can request adaptive sync. While it is set, the vsync IRQ runs
	 * a software control loop over ENCP_VIDEO_MAX_LNCNT, clamped to the sink's
	 * EDID FreeSync range. VRR_ENABLED is refused in atomic_check on a sink
	 * that advertises no such range (connector vrr_capable = 0) and on a mode
	 * this implementation cannot drive (interlaced / 480i / 576i, which run on
	 * ENCI rather than ENCP).
	 */
	drm_object_attach_property(&crtc->base,
				   priv->drm->mode_config.prop_vrr_enabled, 0);

	priv->crtc = crtc;

	return 0;
}
