// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2016 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 * Copyright (C) 2015 Amlogic, Inc. All rights reserved.
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>

#include <media/cec-notifier.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_bridge_connector.h>
#include <drm/drm_device.h>
#include <drm/drm_edid.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>

#include <linux/media-bus-format.h>
#include <linux/videodev2.h>

#include "meson_crtc.h"
#include "meson_drv.h"
#include "meson_registers.h"
#include "meson_vclk.h"
#include "meson_venc.h"
#include "meson_encoder_hdmi.h"

struct meson_encoder_hdmi {
	struct drm_encoder encoder;
	struct drm_bridge bridge;
	struct drm_bridge *next_bridge;
	struct drm_connector *connector;
	struct meson_drm *priv;
	unsigned long output_bus_fmt;
	struct cec_notifier *cec_notifier;
	/*
	 * VRR-001d: drm_bridge_connector owns the connector helper vtable, and
	 * there is no bridge callback that fires after the connector's EDID has
	 * been re-read. We therefore wrap its .get_modes (see
	 * meson_encoder_hdmi_connector_get_modes) to publish vrr_capable right
	 * after display_info is refreshed. conn_helper is our copy of the
	 * bridge-connector vtable with .get_modes replaced; orig_conn_helper is
	 * the original, whose .get_modes the wrapper chains to.
	 */
	const struct drm_connector_helper_funcs *orig_conn_helper;
	struct drm_connector_helper_funcs conn_helper;
};

#define bridge_to_meson_encoder_hdmi(x) \
	container_of(x, struct meson_encoder_hdmi, bridge)

/*
 * Publish the sink's variable-refresh capability onto the connector.
 *
 * This has to run after the connector's EDID has been re-read and parsed into
 * display_info, and it has to run under &drm_mode_config.mutex, which is what
 * protects display_info. The connector probe (.get_modes) is the only context
 * that satisfies both -- drm_bridge_connector_get_modes_edid() calls
 * drm_edid_connector_update() itself, and drm_helper_probe_single_connector_
 * modes() holds mode_config.mutex across the call. It also runs on every EDID
 * change, including unplug (where the core zeroes has_freesync, so vrr_capable
 * correctly drops back to false).
 *
 * Notably this must NOT be done from .hpd_notify: that fires BEFORE the EDID is
 * re-read, and it runs under the bridge hpd_mutex, so re-reading the EDID from
 * there inverts against the probe path -- which is exactly what hung this board
 * at boot on the first attempt.
 *
 * drm_bridge_connector's helper vtable is static and private, so we chain
 * through the pointer we saved at probe rather than naming it.
 */
static int meson_encoder_hdmi_connector_get_modes(struct drm_connector *connector)
{
	/* cast away const: helper_private points at our own embedded vtable */
	struct drm_connector_helper_funcs *helper =
		(struct drm_connector_helper_funcs *)connector->helper_private;
	const struct drm_display_info *info = &connector->display_info;
	struct meson_encoder_hdmi *encoder_hdmi;
	int count = 0;

	if (WARN_ON(!helper))
		return 0;

	encoder_hdmi = container_of(helper, struct meson_encoder_hdmi,
				    conn_helper);

	if (encoder_hdmi->orig_conn_helper &&
	    encoder_hdmi->orig_conn_helper->get_modes)
		count = encoder_hdmi->orig_conn_helper->get_modes(connector);

	drm_connector_set_vrr_capable_property(connector, info->has_freesync);

	return count;
}

static int meson_encoder_hdmi_attach(struct drm_bridge *bridge,
				     struct drm_encoder *encoder,
				     enum drm_bridge_attach_flags flags)
{
	struct meson_encoder_hdmi *encoder_hdmi = bridge_to_meson_encoder_hdmi(bridge);

	return drm_bridge_attach(encoder, encoder_hdmi->next_bridge,
				 &encoder_hdmi->bridge, flags);
}

static void meson_encoder_hdmi_detach(struct drm_bridge *bridge)
{
	struct meson_encoder_hdmi *encoder_hdmi = bridge_to_meson_encoder_hdmi(bridge);

	cec_notifier_conn_unregister(encoder_hdmi->cec_notifier);
	encoder_hdmi->cec_notifier = NULL;
}

static void meson_encoder_hdmi_set_vclk(struct meson_encoder_hdmi *encoder_hdmi,
					const struct drm_display_mode *mode)
{
	struct meson_drm *priv = encoder_hdmi->priv;
	int vic = drm_match_cea_mode(mode);
	unsigned long long phy_freq;
	unsigned long long vclk_freq;
	unsigned long long venc_freq;
	unsigned long long hdmi_freq;

	vclk_freq = mode->clock * 1000ULL;

	/* For 420, pixel clock is half unlike venc clock */
	if (encoder_hdmi->output_bus_fmt == MEDIA_BUS_FMT_UYYVYY8_0_5X24 ||
	    encoder_hdmi->output_bus_fmt == MEDIA_BUS_FMT_UYYVYY10_0_5X30)
		vclk_freq /= 2;

	/*
	 * TMDS clock is pixel_clock * 10 for 8-bit. For 10-bit (DC_30bit)
	 * deep color the TMDS character rate is pixel_clock * 5/4, so the PHY
	 * runs at pixel_clock * 12.5 (== vclk_freq * 25 / 2). venc/vclk stay
	 * at the pixel rate -- only the PHY/TMDS path is * 1.25. The bandwidth
	 * gate in get_inp_bus_fmts guarantees 10-bit is only ever selected for
	 * a mode whose PHY freq is reachable, so no extra clamp is needed here.
	 */
	if (encoder_hdmi->output_bus_fmt == MEDIA_BUS_FMT_YUV10_1X30 ||
	    encoder_hdmi->output_bus_fmt == MEDIA_BUS_FMT_UYYVYY10_0_5X30)
		phy_freq = vclk_freq * 25 / 2;
	else
		phy_freq = vclk_freq * 10;

	if (!vic) {
		meson_vclk_setup(priv, MESON_VCLK_TARGET_DMT, phy_freq,
				 vclk_freq, vclk_freq, vclk_freq, false);
		return;
	}

	/* 480i/576i needs global pixel doubling */
	if (mode->flags & DRM_MODE_FLAG_DBLCLK)
		vclk_freq *= 2;

	venc_freq = vclk_freq;
	hdmi_freq = vclk_freq;

	/* VENC double pixels for 1080i, 720p and YUV420 modes */
	if (meson_venc_hdmi_venc_repeat(vic) ||
	    encoder_hdmi->output_bus_fmt == MEDIA_BUS_FMT_UYYVYY8_0_5X24 ||
	    encoder_hdmi->output_bus_fmt == MEDIA_BUS_FMT_UYYVYY10_0_5X30)
		venc_freq *= 2;

	vclk_freq = max(venc_freq, hdmi_freq);

	if (mode->flags & DRM_MODE_FLAG_DBLCLK)
		venc_freq /= 2;

	dev_dbg(priv->dev,
		"phy:%lluHz vclk=%lluHz venc=%lluHz hdmi=%lluHz enci=%d\n",
		phy_freq, vclk_freq, venc_freq, hdmi_freq,
		priv->venc.hdmi_use_enci);

	meson_vclk_setup(priv, MESON_VCLK_TARGET_HDMI, phy_freq, vclk_freq,
			 venc_freq, hdmi_freq, priv->venc.hdmi_use_enci);
}

static enum drm_mode_status meson_encoder_hdmi_mode_valid(struct drm_bridge *bridge,
					const struct drm_display_info *display_info,
					const struct drm_display_mode *mode)
{
	struct meson_encoder_hdmi *encoder_hdmi = bridge_to_meson_encoder_hdmi(bridge);
	struct meson_drm *priv = encoder_hdmi->priv;
	/*
	 * E5/D6: match dw-hdmi's own is_hdmi2_sink definition (dw-hdmi.c
	 * dw_hdmi_bridge_atomic_get_output_bus_fmts(): scdc.supported ||
	 * YCBCR420) instead of scdc-only. drm_edid.c's parse_cta_y420cmdb()
	 * always sets DRM_COLOR_FORMAT_YCBCR420 whenever any VIC's
	 * y420_cmdb_modes bit is set, so drm_mode_is_420_also() being true for
	 * THIS mode guarantees color_formats already carries YCBCR420 -- with
	 * this fixed definition is_hdmi2_sink is therefore unconditionally
	 * true whenever drm_mode_is_420_also() is, making the stale
	 * "!is_hdmi2_sink && 420_also" clause below structurally dead (can
	 * never contribute true). Matches dw-hdmi's current DC-002 behavior:
	 * a 420-also sink is tried at 444/422/RGB first; 420 is a last-resort
	 * fallback, never force-selected purely for being non-SCDC.
	 */
	bool is_hdmi2_sink = display_info->hdmi.scdc.supported ||
			     (display_info->color_formats & DRM_COLOR_FORMAT_YCBCR420);
	unsigned long long clock = mode->clock * 1000ULL;
	unsigned long long phy_freq;
	unsigned long long vclk_freq;
	unsigned long long venc_freq;
	unsigned long long hdmi_freq;
	int vic = drm_match_cea_mode(mode);
	enum drm_mode_status status;
	bool use_420;

	dev_dbg(priv->dev, "Modeline " DRM_MODE_FMT "\n", DRM_MODE_ARG(mode));

	/* If sink does not support 540MHz, reject the non-420 HDMI2 modes */
	if (display_info->max_tmds_clock &&
	    mode->clock > display_info->max_tmds_clock &&
	    !drm_mode_is_420_only(display_info, mode) &&
	    !drm_mode_is_420_also(display_info, mode))
		return MODE_BAD;

	/* Check against non-VIC supported modes */
	if (!vic) {
		status = meson_venc_hdmi_supported_mode(mode);
		if (status != MODE_OK)
			return status;

		return meson_vclk_dmt_supported_freq(priv, clock);
	/* Check against supported VIC modes */
	} else if (!meson_venc_hdmi_supported_vic(vic))
		return MODE_BAD;

	vclk_freq = clock;

	/* Use YUV420 (half the TMDS clock) when the sink requires it (420-only),
	 * when a non-HDMI2 sink offers it (420-also), OR when the source HDMI-TX
	 * PHY cannot serialize 444 at this clock but can in 420 -- e.g. the S805X
	 * (max_hdmi_phy_freq 1.65GHz) can do 4K30 only via 420 (444 needs a 2.97GHz
	 * phy, 420 needs 1.485GHz). The PHY-limited case needs only that the sink
	 * advertises YUV420 support (color_formats); the dw-hdmi bridge offers a
	 * base-420 fallback for such sinks and get_inp_bus_fmts (below) rejects the
	 * 444/422 candidates, so the negotiation lands on 420. */
	use_420 = drm_mode_is_420_only(display_info, mode) ||
		  (!is_hdmi2_sink && drm_mode_is_420_also(display_info, mode)) ||
		  (priv->limits && priv->limits->max_hdmi_phy_freq &&
		   (clock * 10) > priv->limits->max_hdmi_phy_freq &&
		   (display_info->color_formats & DRM_COLOR_FORMAT_YCBCR420));

	/* For 420, pixel clock is half unlike venc clock */
	if (use_420)
		vclk_freq /= 2;

	/* TMDS clock is pixel_clock * 10 */
	phy_freq = vclk_freq * 10;

	/* 480i/576i needs global pixel doubling */
	if (mode->flags & DRM_MODE_FLAG_DBLCLK)
		vclk_freq *= 2;

	venc_freq = vclk_freq;
	hdmi_freq = vclk_freq;

	/* VENC double pixels for 1080i, 720p and YUV420 modes */
	if (meson_venc_hdmi_venc_repeat(vic) || use_420)
		venc_freq *= 2;

	vclk_freq = max(venc_freq, hdmi_freq);

	if (mode->flags & DRM_MODE_FLAG_DBLCLK)
		venc_freq /= 2;

	/*
	 * Labels ordered to match the args -- these read phy first. They used to
	 * say "vclk:... phy=..." while passing (phy_freq, vclk_freq), so the log
	 * reported a 5.94GHz vclk and a 594MHz phy, i.e. exactly swapped. The
	 * sibling dev_dbg in meson_encoder_hdmi_set_vclk() prints phy first, so
	 * the two lines also disagreed with each other for the same modeset.
	 */
	dev_dbg(priv->dev,
		"%s: phy:%lluHz vclk=%lluHz venc=%lluHz hdmi=%lluHz\n",
		__func__, phy_freq, vclk_freq, venc_freq, hdmi_freq);

	return meson_vclk_vic_supported_freq(priv, phy_freq, vclk_freq);
}

static void meson_encoder_hdmi_atomic_enable(struct drm_bridge *bridge,
					     struct drm_atomic_state *state)
{
	struct meson_encoder_hdmi *encoder_hdmi = bridge_to_meson_encoder_hdmi(bridge);
	/*
	 * D7: the bridge state for THIS commit. On 6.12 this was derived from
	 * the &drm_bridge_state passed into .atomic_enable; that callback now
	 * receives the &drm_atomic_state directly (drm_bridge_funcs in
	 * include/drm/drm_bridge.h), so look the bridge state up from it.
	 */
	struct drm_bridge_state *new_bridge_state =
		drm_atomic_get_new_bridge_state(state, bridge);
	unsigned int ycrcb_map = VPU_HDMI_OUTPUT_CBYCR;
	struct meson_drm *priv = encoder_hdmi->priv;
	struct drm_connector_state *conn_state;
	const struct drm_display_mode *mode;
	struct drm_crtc_state *crtc_state;
	struct drm_connector *connector;
	bool yuv420_mode = false;
	bool yuv422_mode = false;
	int vic;

	connector = drm_atomic_get_new_connector_for_encoder(state, bridge->encoder);
	if (WARN_ON(!connector))
		return;

	conn_state = drm_atomic_get_new_connector_state(state, connector);
	if (WARN_ON(!conn_state))
		return;

	crtc_state = drm_atomic_get_new_crtc_state(state, conn_state->crtc);
	if (WARN_ON(!crtc_state))
		return;

	/*
	 * D7: latch output_bus_fmt from THIS commit's bridge state here
	 * (guaranteed a real apply), not in atomic_check() -- see
	 * meson_encoder_hdmi_atomic_check().
	 */
	if (new_bridge_state)
		encoder_hdmi->output_bus_fmt = new_bridge_state->output_bus_cfg.format;

	mode = &crtc_state->adjusted_mode;

	vic = drm_match_cea_mode(mode);

	dev_dbg(priv->dev, "\"%s\" vic %d\n", mode->name, vic);

	if (encoder_hdmi->output_bus_fmt == MEDIA_BUS_FMT_UYYVYY8_0_5X24 ||
	    encoder_hdmi->output_bus_fmt == MEDIA_BUS_FMT_UYYVYY10_0_5X30) {
		ycrcb_map = VPU_HDMI_OUTPUT_CRYCB;
		yuv420_mode = true;
	} else if (encoder_hdmi->output_bus_fmt == MEDIA_BUS_FMT_UYVY8_1X16 ||
		   encoder_hdmi->output_bus_fmt == MEDIA_BUS_FMT_UYVY10_1X20) {
		/*
		 * DC-001: YUV422 (8- AND 10-bit) needs the CRYCB component-order
		 * mux, NOT the default CBYCR. dw-hdmi's YCC422 input packer reads
		 * the VPU's 422 stream in CrYCb lane order; leaving the YUV444
		 * default CBYCR swaps Cb/Cr -> a green cast. Bench-verified on an
		 * LG QNED85: CBYCR=all-green, CRCBY=white-crushes-to-black,
		 * CRYCB=correct. (UYVY10_1X20 added here for the 10-bit path.)
		 */
		ycrcb_map = VPU_HDMI_OUTPUT_CRYCB;
		yuv422_mode = true;
	}

	/* VENC + VENC-DVI Mode setup */
	meson_venc_hdmi_mode_set(priv, vic, ycrcb_map, yuv420_mode, yuv422_mode, mode);

	/* VCLK Set clock */
	meson_encoder_hdmi_set_vclk(encoder_hdmi, mode);

	/* E8: VPU_HDMI_DITH_CNTL is documented GXM/G12A/SM1+ only
	 * (meson_registers.h) -- gate the RMW below per that map. */
	bool dith10_soc = meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXM) ||
			   meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A);

	if (encoder_hdmi->output_bus_fmt == MEDIA_BUS_FMT_YUV10_1X30 ||
	    encoder_hdmi->output_bus_fmt == MEDIA_BUS_FMT_UYVY10_1X20 ||
	    encoder_hdmi->output_bus_fmt == MEDIA_BUS_FMT_UYYVYY10_0_5X30) {
		/*
		 * DC-001: 10-bit (DC_30bit) deep color to HDMI-TX. The VPU
		 * pipeline already carries 10-bit pixels; the two writes here
		 * stop them being dithered down to 8 so native 10-bit reaches
		 * the HDMI-TX. FMT_CTRL chroma bits[1:0]: 444=0 (YUV10_1X30),
		 * 422=1 (UYVY10_1X20), 420=2 (UYYVYY10_0_5X30); the 422/420 paths
		 * add chroma_dnsmp=2 ((2<<2), average) -- same as their 8-bit
		 * paths but with the 10->8 dither disabled. The decisive bit is
		 * DITH_CNTL[4]=0. Per BSP hdmitx20 30B path; the GCP 30-bit color
		 * depth is set by dw-hdmi from the bus format, not here.
		 */
		u32 fmt_ctrl = 0;					/* 444 */

		if (encoder_hdmi->output_bus_fmt == MEDIA_BUS_FMT_UYVY10_1X20)
			fmt_ctrl = 1 | (2 << 2);			/* 422 + avg */
		else if (encoder_hdmi->output_bus_fmt == MEDIA_BUS_FMT_UYYVYY10_0_5X30)
			fmt_ctrl = 2 | (2 << 2);			/* 420 + avg */

		writel_relaxed(fmt_ctrl,
			       priv->io_base + _REG(VPU_HDMI_FMT_CTRL));
		if (dith10_soc)
			writel_bits_relaxed(BIT(4), 0,
					    priv->io_base + _REG(VPU_HDMI_DITH_CNTL));
	} else if (encoder_hdmi->output_bus_fmt == MEDIA_BUS_FMT_UYYVYY8_0_5X24) {
		/* Setup YUV420 to HDMI-TX, no 10bit diphering */
		writel_relaxed(2 | (2 << 2),
			       priv->io_base + _REG(VPU_HDMI_FMT_CTRL));
		/*
		 * E8: restore dith10_en for this 8-bit link -- a prior 10-bit
		 * modeset cleared it above and it was never restored, leaving
		 * 10->8 dither disabled (banding) until reboot.
		 */
		if (dith10_soc)
			writel_bits_relaxed(BIT(4), BIT(4),
					    priv->io_base + _REG(VPU_HDMI_DITH_CNTL));
	} else if (encoder_hdmi->output_bus_fmt == MEDIA_BUS_FMT_UYVY8_1X16) {
		/* Setup YUV422 to HDMI-TX, no 10bit diphering */
		writel_relaxed(1 | (2 << 2),
				priv->io_base + _REG(VPU_HDMI_FMT_CTRL));
		if (dith10_soc)
			writel_bits_relaxed(BIT(4), BIT(4),
					    priv->io_base + _REG(VPU_HDMI_DITH_CNTL));
	} else {
		/* Setup YUV444 to HDMI-TX, no 10bit diphering */
		writel_relaxed(0, priv->io_base + _REG(VPU_HDMI_FMT_CTRL));
		if (dith10_soc)
			writel_bits_relaxed(BIT(4), BIT(4),
					    priv->io_base + _REG(VPU_HDMI_DITH_CNTL));
	}

	dev_dbg(priv->dev, "%s\n", priv->venc.hdmi_use_enci ? "VENCI" : "VENCP");

	if (priv->venc.hdmi_use_enci)
		writel_relaxed(1, priv->io_base + _REG(ENCI_VIDEO_EN));
	else
		writel_relaxed(1, priv->io_base + _REG(ENCP_VIDEO_EN));
}

static void meson_encoder_hdmi_atomic_disable(struct drm_bridge *bridge,
					      struct drm_atomic_state *state)
{
	struct meson_encoder_hdmi *encoder_hdmi = bridge_to_meson_encoder_hdmi(bridge);
	struct meson_drm *priv = encoder_hdmi->priv;

	writel_bits_relaxed(0x3, 0,
			    priv->io_base + _REG(VPU_HDMI_SETTING));

	writel_relaxed(0, priv->io_base + _REG(ENCI_VIDEO_EN));
	writel_relaxed(0, priv->io_base + _REG(ENCP_VIDEO_EN));
}

/*
 * DC-001/D1/H4: does this mode's implied HDMI TMDS character rate for OUTPUT
 * FORMAT output_fmt fit under the sink's advertised TMDS ceiling? mode->clock
 * is the pixel clock in kHz; the ceiling is display_info->max_tmds_clock
 * (kHz), which DRM sets to the HDMI Forum VSDB Max_TMDS_Character_Rate for an
 * HDMI-2.0 sink (e.g. 600 MHz, drm_edid.c drm_parse_hdmi_forum_scds) and
 * otherwise to the base HDMI-VSDB Max_TMDS_Clock (typically 300 MHz on
 * HDMI-1.4, or unset on a bare DVI EDID). The earlier hardcoded 300 MHz
 * constant was that base-VSDB value read as the limit -- wrong on HDMI-2.0
 * sinks, where it walled every 4K deep-color mode.
 *
 * 4:2:0 (UYYVYY8/UYYVYY10) HALVES the pixel rate per TMDS character; 4:4:4
 * and 4:2:2 run at the full pixel rate. Only the two formats that actually
 * carry a deep-color (DC_30bit) link -- YUV10_1X30 (444-10) and
 * UYYVYY10_0_5X30 (420-10) -- add the flat * 5/4 deep-color multiplier;
 * UYVY10_1X20 (422-10) carries its extra bits WITHIN the 8-bit TMDS
 * bandwidth (pixel * 1.0, no clock bump -- HDMI 4:2:2 is always transmitted
 * at a fixed 8-bit-equivalent character rate regardless of per-channel bit
 * depth), so it is treated like an unscaled format here, same as the 8-bit
 * candidates.
 *
 * e.g. 4K60 420-10 = 594 * 5/4 / 2 = 371.25 MHz -- within a 600 MHz sink AND
 * deep-color-CD-signaled (the standard 4K-HDR path), whereas 4K60 444-10 =
 * 742.5 MHz stays correctly walled.
 *
 * D1/H4: this used to be called ONLY for the two deep-color formats, so
 * nothing enforced the ceiling for an 8-bit (or 422-10) candidate. A DC-002
 * reorder that prefers 4:4:4 over 4:2:0 for a 4:2:0-also sink removed the
 * only thing that had incidentally kept such a mode off an over-spec 4:4:4
 * link -- so a 4:2:0-also sink whose Forum VSDB (or lack of one) caps it at
 * 300 MHz could get driven at 4K60 444-8 (594 MHz), ~2x its declared
 * ceiling. It is now called from every candidate in the caller's per-format
 * loop, not just the two deep-color ones.
 *
 * When max_tmds_clock is unset (no VSDB info at all), deep-color candidates
 * stay conservatively refused -- unchanged from before this helper covered
 * every format, since offering 10-bit with no known headroom is unsafe.
 * Every other candidate is left ungated in that case, exactly as before this
 * check existed for them (only the separate PHY-clock check applied) -- so a
 * sink that never populates max_tmds_clock keeps working as it did before.
 */

/*
 * Shared bit-depth-aware TMDS CHARACTER rate (kHz) for a candidate output
 * format at a given mode's pixel clock. Factored out of
 * meson_encoder_hdmi_bus_fmt_fits_tmds() so the SoC PHY-frequency-ceiling
 * check in meson_encoder_hdmi_get_inp_bus_fmts() (below) can use the same
 * per-format multiplier instead of the flat pixel_clock * 10 it used to
 * apply to every candidate including the 10-bit deep-color ones -- which
 * under-computed the real PHY rate by 20-25% for those formats (e.g. 1080p60
 * 444-10: flat math gives 1485 MHz phy, the real rate is 1856.25 MHz).
 * PHY serial bit rate = this TMDS character rate * 10 (see
 * meson_encoder_hdmi_set_vclk()'s phy_freq = vclk_freq * 25 / 2 for the two
 * 10-bit formats, which is (vclk_freq * 5/4) * 10).
 */
static unsigned long long
meson_encoder_hdmi_bus_fmt_tmds_khz(const struct drm_display_mode *mode,
				    u32 output_fmt)
{
	unsigned long long tmds_khz = mode->clock;
	bool is_420 = output_fmt == MEDIA_BUS_FMT_UYYVYY8_0_5X24 ||
		      output_fmt == MEDIA_BUS_FMT_UYYVYY10_0_5X30;
	bool is_deep_color = output_fmt == MEDIA_BUS_FMT_YUV10_1X30 ||
			     output_fmt == MEDIA_BUS_FMT_UYYVYY10_0_5X30;

	/* 4:2:0 carries half the pixels per TMDS character. */
	if (is_420)
		tmds_khz /= 2;

	/* Deep color (DC_30bit): TMDS character rate is pixel_clock * 5/4. */
	if (is_deep_color)
		tmds_khz = tmds_khz * 5 / 4;

	return tmds_khz;
}

static bool
meson_encoder_hdmi_bus_fmt_fits_tmds(const struct drm_display_mode *mode,
				     const struct drm_display_info *display_info,
				     u32 output_fmt)
{
	bool is_deep_color = output_fmt == MEDIA_BUS_FMT_YUV10_1X30 ||
			     output_fmt == MEDIA_BUS_FMT_UYYVYY10_0_5X30;

	if (!display_info->max_tmds_clock)
		return !is_deep_color;

	return meson_encoder_hdmi_bus_fmt_tmds_khz(mode, output_fmt) <=
	       display_info->max_tmds_clock;
}

static const u32 meson_encoder_hdmi_out_bus_fmts[] = {
	/*
	 * DC-001: YUV10_1X30 (10-bit/DC_30bit, YUV444) is offered FIRST so dw-hdmi,
	 * whose atomic_get_output_bus_fmts proposes the highest-bpc format first
	 * and whose max_requested_bpc defaults to 10, can negotiate deep color.
	 * It is gated per-mode in meson_encoder_hdmi_get_inp_bus_fmts(): for a
	 * mode whose 10-bit TMDS exceeds the 300 MHz ceiling (e.g. 4K30 4:4:4 or
	 * the 4K boot fbdev modeset) meson reports zero input formats for this
	 * output, so the bus-format negotiation falls back to the 8-bit YUV
	 * path. 8-bit output is therefore byte-for-byte unchanged.
	 */
	MEDIA_BUS_FMT_YUV10_1X30,
	MEDIA_BUS_FMT_UYVY10_1X20,
	MEDIA_BUS_FMT_UYYVYY10_0_5X30,
	MEDIA_BUS_FMT_YUV8_1X24,
	MEDIA_BUS_FMT_UYVY8_1X16,
	MEDIA_BUS_FMT_UYYVYY8_0_5X24,
};

static u32 *
meson_encoder_hdmi_get_inp_bus_fmts(struct drm_bridge *bridge,
					struct drm_bridge_state *bridge_state,
					struct drm_crtc_state *crtc_state,
					struct drm_connector_state *conn_state,
					u32 output_fmt,
					unsigned int *num_input_fmts)
{
	struct meson_encoder_hdmi *encoder_hdmi = bridge_to_meson_encoder_hdmi(bridge);
	struct meson_drm *priv = encoder_hdmi->priv;
	const struct drm_display_info *display_info =
		conn_state->connector ?
		&conn_state->connector->display_info : NULL;
	/*
	 * H1: max_requested_bpc alone is not sink capability -- it is a
	 * connector PROPERTY that drm_connector_attach_max_bpc_property()
	 * defaults to its max (10, see the probe comment below), so it reads
	 * >=10 even against a sink whose EDID has no deep-colour bits set at
	 * all. Require display_info->bpc >= 10 too, mirroring dw-hdmi's own
	 * gate on offering a deep-color output format in
	 * dw_hdmi_bridge_atomic_get_output_bus_fmts() (dw-hdmi.c, e.g.
	 * "max_bpc >= 10 && info->bpc >= 10" at its 444/422/420 branches).
	 * Without this, a non-deep-color sink whose 10-bit TMDS rate happens
	 * to fit under max_tmds_clock (e.g. 1080p60: 185.6MHz fits a 300MHz
	 * ceiling) latched ten_bit_ok true, which made every 8-bit candidate
	 * below get refused -- and since dw-hdmi never offers a 10-bit
	 * candidate to a bpc==8 sink either, get_inp_bus_fmts returned NULL
	 * for every candidate and the whole modeset failed with -ENOTSUPP.
	 * Hardware-confirmed today with a synthetic EDID (deep-colour bits
	 * cleared): 1080p60 bpc=10 request -> commit ret=-524 (ENOTSUPP).
	 */
	/*
	 * E7: refuse 10-bit for a DBLCLK mode (480i/576i). phy_freq is
	 * derived from the PRE-doubling pixel rate in set_vclk(), while
	 * meson_vclk_setup()'s DC-001-REPEAT branch compares against
	 * dac_freq -- the POST-doubling rate -- so they can never match for a
	 * DBLCLK mode; the 8-bit params[] table also has no matching entry,
	 * and meson_vclk_setup() hits "Fatal Error, invalid HDMI vclk freq"
	 * and returns WITHOUT writing any clock-tree registers, while dw-hdmi
	 * has already committed to 10-bit GCP and atomic_enable() enables
	 * VENC anyway -- a half-configured link. No real sink requests 10-bit
	 * on an SD interlaced mode, and correctly handling the doubled rate
	 * needs hardware iteration; refuse cleanly instead -- falls through
	 * to the always-working 8-bit DBLCLK path (params[]
	 * MESON_VCLK_HDMI_ENCI_54000/DDR_54000).
	 */
	bool is_dblclk_mode = crtc_state->adjusted_mode.flags & DRM_MODE_FLAG_DBLCLK;
	bool sink_supports_10bit = display_info && display_info->bpc >= 10;
	bool req_10bit = sink_supports_10bit && !is_dblclk_mode &&
		conn_state->max_requested_bpc >= 10;
	bool ten_bit_ok = req_10bit &&
		meson_encoder_hdmi_bus_fmt_fits_tmds(&crtc_state->adjusted_mode,
						     display_info,
						     MEDIA_BUS_FMT_YUV10_1X30);
	u32 *input_fmts = NULL;
	int i;

	*num_input_fmts = 0;

	/*
	 * DC-001 deep-color negotiation. dw-hdmi's atomic_get_output_bus_fmts
	 * proposes the deep-color (10-bit) and 8-bit formats in preference
	 * order and drm_atomic_bridge_chain_select_bus_fmts() takes the first
	 * the chain accepts. Steer that here, per output format. req_10bit /
	 * ten_bit_ok already require sink_supports_10bit (display_info->bpc
	 * >= 10, see above) -- a sink with no deep-colour capability at all
	 * never reaches any of the 10-bit branches below and never hits the
	 * "every 8-bit format is REFUSED when ten_bit_ok" branch either:
	 *
	 *   - YUV444-10 (YUV10_1X30) needs the *1.25 deep-color TMDS, so it is
	 *     accepted ONLY when it fits the deep-color ceiling (ten_bit_ok).
	 *   - YUV422-10 (UYVY10_1X20) carries 10-bit WITHIN the 8-bit TMDS
	 *     bandwidth -- 4:2:2 gets no deep-color clock bump (pixel*1.0, not
	 *     *1.25) -- so it fits wherever 8-bit 4:4:4 fits. Gate it only on a
	 *     10-bit request and let the shared PHY-clock check below admit it.
	 *     This makes high-TMDS deep color (e.g. 4K60 422-10 at 594MHz, over
	 *     the 300MHz *1.25 ceiling yet within the 8-bit PHY range) reachable
	 *     via the force_bus_format knob. The AUTO path is unchanged: dw-hdmi
	 *     offers YUV8_1X24 BEFORE UYVY10_1X20 and meson accepts 444-8
	 *     whenever 444-10 does not fit (ten_bit_ok false), so 4K stays 8-bit
	 *     by default and is never silently subsampled to 4:2:2.
	 *   - every 8-bit format is REFUSED when ten_bit_ok (444-10 fits), so
	 *     the negotiation falls through to the 10-bit format.
	 *
	 * When 10-bit does not fit, ten_bit_ok is false and the 8-bit path runs
	 * unchanged -- e.g. the 4K30/4K60 boot fbdev stays 8-bit.
	 */
	if (output_fmt == MEDIA_BUS_FMT_YUV10_1X30) {
		if (!ten_bit_ok)
			return NULL;

		/*
		 * E6: ten_bit_ok only confirms the TMDS CHARACTER rate fits the
		 * sink's max_tmds_clock ceiling; the SoC package-limit check
		 * further below (priv->limits->max_hdmi_phy_freq) is also only a
		 * ceiling, and is skipped entirely on a SoC with no limits set.
		 * Neither ever asks whether the HDMI PLL can actually LOCK to
		 * the resulting *12.5 phy_freq -- that od/m/frac search
		 * (meson_vclk_pll_freq_synthesizable(), same search
		 * meson_hdmi_pll_generic_set() runs) previously ran for the
		 * first time inside atomic_enable() -> meson_vclk_setup(), a
		 * void call chain: a miss there is a DRM_ERROR plus a
		 * half-programmed clock tree (VENC and HDMI-TX sys clock
		 * already written), not a clean mode rejection. Run the same
		 * search here, at negotiation time, so an unsynthesizable
		 * 10-bit phy_freq falls back to the 8-bit candidate below
		 * instead of reaching atomic_enable() at all.
		 */
		if (!meson_vclk_pll_freq_synthesizable(priv,
			meson_encoder_hdmi_bus_fmt_tmds_khz(&crtc_state->adjusted_mode,
							    output_fmt) * 1000 * 10))
			return NULL;
	} else if (output_fmt == MEDIA_BUS_FMT_UYVY10_1X20) {
		if (!req_10bit)
			return NULL;
	} else if (output_fmt == MEDIA_BUS_FMT_UYYVYY10_0_5X30) {
		/* 4:2:0 10-bit: pixel*1.25/2 (4K60 = 371MHz) -- CD-signaled deep
		 * color that fits an HDMI-2.0 sink where 444-10 (742MHz) cannot.
		 * Reachable via force_bus_format; the auto path prefers 444-8 at
		 * 4K (offered before 420-10) so the default is unchanged. */
		if (!(req_10bit &&
		      meson_encoder_hdmi_bus_fmt_fits_tmds(&crtc_state->adjusted_mode,
							   display_info,
							   MEDIA_BUS_FMT_UYYVYY10_0_5X30)))
			return NULL;

		/* E6: same PLL-lock check as the 444-10 branch above -- see its
		 * comment. */
		if (!meson_vclk_pll_freq_synthesizable(priv,
			meson_encoder_hdmi_bus_fmt_tmds_khz(&crtc_state->adjusted_mode,
							    output_fmt) * 1000 * 10))
			return NULL;
	} else if (ten_bit_ok) {
		return NULL;
	}

	for (i = 0 ; i < ARRAY_SIZE(meson_encoder_hdmi_out_bus_fmts) ; ++i) {
		if (output_fmt == meson_encoder_hdmi_out_bus_fmts[i]) {
			/* D1/H4: reject a candidate whose implied TMDS character rate
			 * would exceed the SINK's advertised max_tmds_clock ceiling,
			 * for EVERY candidate -- not just the two deep-color ones
			 * gated above. See meson_encoder_hdmi_bus_fmt_fits_tmds()'s
			 * comment for why this is needed even for 8-bit candidates. */
			/* E13: crtc_state is provably non-NULL at this call
			 * site (this function is only ever invoked via
			 * drm_atomic_helper.c's mode_fixup(), which always
			 * supplies one) -- matches its unconditional use just
			 * above; the check was dead code. */
			if (display_info &&
			    !meson_encoder_hdmi_bus_fmt_fits_tmds(&crtc_state->adjusted_mode,
								  display_info,
								  output_fmt))
				return NULL;

			/* Reject an output format whose HDMI-TX PHY clock would exceed
			 * the SoC limit, so the bus-format negotiation falls back to a
			 * lower-bandwidth format. 444/422 run the TMDS at the full pixel
			 * rate; 420 halves it. e.g. S805X 4K30: 444/422 need a 2.97GHz
			 * phy (over the 1.65GHz limit), 420 needs 1.485GHz (fits).
			 *
			 * D1/H4-follow-up: this used to compute phy_freq from the flat
			 * pixel_clock * 10 formula for EVERY candidate, including the
			 * two 10-bit deep-color ones -- under-stating their real PHY
			 * rate by 20-25% (deep color adds a *5/4 TMDS-character-rate
			 * multiplier, see meson_encoder_hdmi_bus_fmt_tmds_khz() above).
			 * On a SoC with a narrow enough max_hdmi_phy_freq (e.g. S805X
			 * at 1.65GHz) that under-count can flip an accept/reject
			 * decision: 1080p60 444-10 computed 1485 MHz (accepted) when
			 * the true PHY rate is 1856.25 MHz (over the ceiling). Reuse
			 * the same bit-depth-aware helper the sink max_tmds_clock
			 * check above uses, so both checks agree on the candidate's
			 * real TMDS character rate; * 1000 converts kHz to Hz, * 10
			 * converts TMDS character rate to PHY serial bit rate. */
			/* E13: same dead crtc_state check as above. */
			if (priv->limits &&
			    priv->limits->max_hdmi_phy_freq) {
				unsigned long long phy_freq =
					meson_encoder_hdmi_bus_fmt_tmds_khz(&crtc_state->adjusted_mode,
									    output_fmt) * 1000 * 10;

				dev_dbg(priv->dev,
					"%s: output_fmt %#x phy_freq %lluHz\n",
					__func__, output_fmt, phy_freq);

				if (phy_freq > priv->limits->max_hdmi_phy_freq)
					return NULL;
			}

			*num_input_fmts = 1;
			input_fmts = kcalloc(*num_input_fmts,
					     sizeof(*input_fmts),
					     GFP_KERNEL);
			if (!input_fmts)
				return NULL;

			input_fmts[0] = output_fmt;

			break;
		}
	}

	return input_fmts;
}

static int meson_encoder_hdmi_atomic_check(struct drm_bridge *bridge,
					struct drm_bridge_state *bridge_state,
					struct drm_crtc_state *crtc_state,
					struct drm_connector_state *conn_state)
{
	struct meson_encoder_hdmi *encoder_hdmi = bridge_to_meson_encoder_hdmi(bridge);
	struct drm_connector_state *old_conn_state =
		drm_atomic_get_old_connector_state(conn_state->state, conn_state->connector);
	struct meson_drm *priv = encoder_hdmi->priv;
	const struct drm_display_info *info =
		conn_state->connector ? &conn_state->connector->display_info : NULL;

	/*
	 * D7: do NOT latch encoder_hdmi->output_bus_fmt here -- same hazard as
	 * dw-hdmi's enc_out_bus_format (dw_hdmi_bridge_atomic_check()): this is
	 * a persistent struct field and atomic_check() runs for TEST_ONLY
	 * probes too. Latched instead in meson_encoder_hdmi_atomic_enable(),
	 * from that commit's own bridge state.
	 */
	dev_dbg(priv->dev, "output_bus_fmt %lx\n", bridge_state->output_bus_cfg.format);

	/*
	 * Deep color (DC-001): the "max bpc" property advertises up to 10
	 * (DC_30bit) and drm_connector_attach_max_bpc_property() initializes it to
	 * its MAX. When the connector requests >=10 AND the 10-bit TMDS link fits
	 * the deep-color ceiling, meson_encoder_hdmi_get_inp_bus_fmts() selects
	 * RGB101010_1X30 and meson_vclk drives the *12.5 (10-bit) phy clock;
	 * otherwise it gracefully downgrades to 8-bit (e.g. 4K30, the boot fbdev).
	 */

	if (!drm_connector_atomic_hdr_metadata_equal(old_conn_state, conn_state))
		crtc_state->mode_changed = true;

	/*
	 * DC-001: a runtime "max bpc" change (a compositor toggling deep color
	 * 8<->10 at a fixed mode) re-runs the bus-format negotiation but does not
	 * flag a modeset on its own -- so the dw-hdmi VP packetizer and the vclk,
	 * both reprogrammed only on a full modeset, keep their previous color depth
	 * and the link never actually changes bpc. Force one, exactly like the HDR
	 * metadata and colorspace cases.
	 */
	if (old_conn_state->max_requested_bpc != conn_state->max_requested_bpc)
		crtc_state->mode_changed = true;

	/*
	 * Colorspace (BT.2020) and IT content-type only reach the wire when the
	 * AVI InfoFrame is rebuilt, which dw-hdmi does on a modeset. A
	 * property-only change does not flag a modeset on its own, so -- exactly
	 * like the HDR metadata case above -- force one when either property
	 * changes so hdmi_config_AVI re-runs with the new conn_state.
	 */
	if (old_conn_state->colorspace != conn_state->colorspace ||
	    old_conn_state->content_type != conn_state->content_type)
		crtc_state->mode_changed = true;

	/*
	 * FR-001: the AMD FreeSync advertisement is programmed by dw-hdmi from its
	 * bridge-enable path (setup_video_path), which only runs on a modeset. A
	 * property-only VRR_ENABLED change does not flag one, so without this the
	 * sink would never be told that variable refresh started -- and, worse,
	 * never told that it stopped, leaving a stale "FreeSync Active" packet
	 * standing while the encoder emits fixed timings again. Force a modeset on
	 * the toggle, exactly like the HDR-metadata / colorspace / max_bpc cases
	 * above, all of which have the same "only rebuilt on modeset" problem.
	 *
	 * VRR_ENABLED lives on the CRTC, not the connector, so the old value comes
	 * from the CRTC's old atomic state rather than old_conn_state.
	 */
	if (crtc_state->crtc) {
		const struct drm_crtc_state *old_crtc_state =
			drm_atomic_get_old_crtc_state(conn_state->state,
						      crtc_state->crtc);

		if (old_crtc_state &&
		    old_crtc_state->vrr_enabled != crtc_state->vrr_enabled)
			crtc_state->mode_changed = true;
	}

	/*
	 * VRR-001d: publish the connected sink's FreeSync range to the CRTC on
	 * EVERY commit that reaches this bridge, not only when VRR is being
	 * enabled. The CRTC's own atomic_check gates VRR_ENABLED on a non-zero
	 * range, and a property-only VRR_ENABLED=1 never reaches this bridge --
	 * so if the range were published only alongside an enable, the very
	 * first property-only enable on a perfectly VRR-capable sink would be
	 * rejected for want of a range nobody had published yet.
	 *
	 * display_info is refreshed by the connector probe (drm_edid_connector_
	 * update) and read here under the modeset locks; an unplug or a non-
	 * FreeSync sink zeroes has_freesync, which clears the range and makes
	 * the CRTC refuse VRR.
	 */
	if (info && info->has_freesync)
		meson_crtc_set_vrr_range(crtc_state->crtc,
					 info->freesync_range.min_vfreq,
					 info->freesync_range.max_vfreq);
	else
		meson_crtc_set_vrr_range(crtc_state->crtc, 0, 0);

	/*
	 * Gate VRR_ENABLED on real sink capability. Refusing VRR on a sink that
	 * advertised no usable FreeSync range is what prevents driving it out of
	 * range -- which does not merely glitch, it drops the link and can wedge
	 * the HDMI TX until a controller re-init (display/vrr/runtime-validation.md).
	 */
	if (crtc_state->vrr_enabled) {
		int vrefresh;

		if (!info || !info->has_freesync) {
			dev_dbg(priv->dev,
				"VRR requested but sink advertises no FreeSync range\n");
			return -EINVAL;
		}

		/*
		 * The mode's own refresh must sit inside the sink's VRR range;
		 * otherwise the nominal timing is already out of range and any
		 * modulation would only make it worse.
		 */
		vrefresh = drm_mode_vrefresh(&crtc_state->adjusted_mode);
		if (vrefresh &&
		    (vrefresh < info->freesync_range.min_vfreq ||
		     vrefresh > info->freesync_range.max_vfreq)) {
			dev_dbg(priv->dev,
				"mode %d Hz outside sink VRR range %u-%u Hz\n",
				vrefresh, info->freesync_range.min_vfreq,
				info->freesync_range.max_vfreq);
			return -EINVAL;
		}
	}

	return 0;
}

static void meson_encoder_hdmi_hpd_notify(struct drm_bridge *bridge,
					  struct drm_connector *connector,
					  enum drm_connector_status status)
{
	struct meson_encoder_hdmi *encoder_hdmi = bridge_to_meson_encoder_hdmi(bridge);

	if (!encoder_hdmi->cec_notifier)
		return;

	if (status == connector_status_connected) {
		const struct drm_edid *drm_edid;
		const struct edid *edid;

		drm_edid = drm_bridge_edid_read(encoder_hdmi->next_bridge,
						encoder_hdmi->connector);
		if (!drm_edid)
			return;

		/*
		 * FIXME: The CEC physical address should be set using
		 * cec_notifier_set_phys_addr(encoder_hdmi->cec_notifier,
		 * connector->display_info.source_physical_address) from a path
		 * that has read the EDID and called
		 * drm_edid_connector_update().
		 */
		edid = drm_edid_raw(drm_edid);

		cec_notifier_set_phys_addr_from_edid(encoder_hdmi->cec_notifier, edid);

		drm_edid_free(drm_edid);
	} else
		cec_notifier_phys_addr_invalidate(encoder_hdmi->cec_notifier);
}

static const struct drm_bridge_funcs meson_encoder_hdmi_bridge_funcs = {
	.attach = meson_encoder_hdmi_attach,
	.detach = meson_encoder_hdmi_detach,
	.mode_valid = meson_encoder_hdmi_mode_valid,
	.hpd_notify = meson_encoder_hdmi_hpd_notify,
	.atomic_enable = meson_encoder_hdmi_atomic_enable,
	.atomic_disable = meson_encoder_hdmi_atomic_disable,
	.atomic_get_input_bus_fmts = meson_encoder_hdmi_get_inp_bus_fmts,
	.atomic_check = meson_encoder_hdmi_atomic_check,
	.atomic_duplicate_state = drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_bridge_destroy_state,
	.atomic_reset = drm_atomic_helper_bridge_reset,
};

int meson_encoder_hdmi_probe(struct meson_drm *priv)
{
	struct meson_encoder_hdmi *meson_encoder_hdmi;
	struct platform_device *pdev;
	struct device_node *remote;
	int ret;

	meson_encoder_hdmi = devm_drm_bridge_alloc(priv->dev,
						   struct meson_encoder_hdmi,
						   bridge,
						   &meson_encoder_hdmi_bridge_funcs);
	if (IS_ERR(meson_encoder_hdmi))
		return PTR_ERR(meson_encoder_hdmi);

	/* HDMI Transceiver Bridge */
	remote = of_graph_get_remote_node(priv->dev->of_node, 1, 0);
	if (!remote) {
		dev_err(priv->dev, "HDMI transceiver device is disabled");
		return 0;
	}

	meson_encoder_hdmi->next_bridge = of_drm_find_bridge(remote);
	if (!meson_encoder_hdmi->next_bridge) {
		ret = dev_err_probe(priv->dev, -EPROBE_DEFER,
				    "Failed to find HDMI transceiver bridge\n");
		goto err_put_node;
	}

	/* HDMI Encoder Bridge */
	meson_encoder_hdmi->bridge.of_node = priv->dev->of_node;
	meson_encoder_hdmi->bridge.type = DRM_MODE_CONNECTOR_HDMIA;
	meson_encoder_hdmi->bridge.interlace_allowed = true;

	drm_bridge_add(&meson_encoder_hdmi->bridge);

	meson_encoder_hdmi->priv = priv;

	/* Encoder */
	ret = drm_simple_encoder_init(priv->drm, &meson_encoder_hdmi->encoder,
				      DRM_MODE_ENCODER_TMDS);
	if (ret) {
		dev_err_probe(priv->dev, ret, "Failed to init HDMI encoder\n");
		goto err_put_node;
	}

	meson_encoder_hdmi->encoder.possible_crtcs = BIT(0);

	/* Attach HDMI Encoder Bridge to Encoder */
	ret = drm_bridge_attach(&meson_encoder_hdmi->encoder, &meson_encoder_hdmi->bridge, NULL,
				DRM_BRIDGE_ATTACH_NO_CONNECTOR);
	if (ret) {
		dev_err_probe(priv->dev, ret, "Failed to attach bridge\n");
		goto err_put_node;
	}

	/* Initialize & attach Bridge Connector */
	meson_encoder_hdmi->connector = drm_bridge_connector_init(priv->drm,
							&meson_encoder_hdmi->encoder);
	if (IS_ERR(meson_encoder_hdmi->connector)) {
		ret = dev_err_probe(priv->dev,
				    PTR_ERR(meson_encoder_hdmi->connector),
				    "Unable to create HDMI bridge connector\n");
		goto err_put_node;
	}
	drm_connector_attach_encoder(meson_encoder_hdmi->connector,
				     &meson_encoder_hdmi->encoder);

	/*
	 * We should have now in place:
	 * encoder->[hdmi encoder bridge]->[dw-hdmi bridge]->[display connector bridge]->[display connector]
	 */

	/*
	 * drm_connector_attach_max_bpc_property() requires the
	 * connector to have a state.
	 */
	drm_atomic_helper_connector_reset(meson_encoder_hdmi->connector);

	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXL) ||
	    meson_vpu_is_compatible(priv, VPU_COMPATIBLE_GXM) ||
	    meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A)) {
		drm_connector_attach_hdr_output_metadata_property(meson_encoder_hdmi->connector);

		/*
		 * DC-BT2020: expose the DRM "Colorspace" property so a client can
		 * request a wide-gamut colorimetry (BT.2020) in the AVI infoframe.
		 * meson_encoder_hdmi_atomic_check() already forces a modeset when
		 * conn_state->colorspace changes, and dw-hdmi now maps a non-default
		 * colorspace into the infoframe via
		 * drm_hdmi_avi_infoframe_colorimetry() -- but the property was never
		 * attached, so conn_state->colorspace could only ever read
		 * DRM_MODE_COLORIMETRY_DEFAULT and the HDR10 path signalled PQ EOTF
		 * with no BT.2020 gamut (HDR-PQ, not full wide gamut).
		 *
		 * The property must be CREATED before it can be attached:
		 * drm_connector_attach_colorspace_property() reads
		 * connector->colorspace_property with no NULL check and hands it to
		 * drm_object_attach_property(), which dereferences it immediately
		 * (`property->dev`). Attaching without creating therefore NULL-derefs
		 * inside meson-drm's probe -- and since CONFIG_DRM_MESON=y that panics
		 * before userspace, i.e. an early boot hang with no journal at all
		 * (measured: kernel 6.12.80-01248-g2498a5d085ba never reached
		 * userspace). Passing 0 for supported_colorspaces selects the full
		 * hdmi_colorspaces set, which includes BT2020_YCC/BT2020_RGB.
		 */
		if (!drm_mode_create_hdmi_colorspace_property(meson_encoder_hdmi->connector, 0))
			drm_connector_attach_colorspace_property(meson_encoder_hdmi->connector);
	}

	/*
	 * DC-001: widen the "max bpc" property range to [8,10] so a client (or
	 * the force_bus_format debug knob path) can request 10-bit -- dw-hdmi's
	 * atomic_get_output_bus_fmts only offers the deep-color (YUV10_1X30 /
	 * RGB101010 / UYVY10) wire formats under "max_bpc >= 10", and the meson
	 * encoder's get_inp_bus_fmts ten_bit_ok gate needs max_requested_bpc >= 10.
	 * Mainline hardcodes [8,8], which caps every request at 8-bit and makes the
	 * whole deep-color output path unreachable. This tree has NO drm_client
	 * max_bpc reset (the 6.12 DC-001 drm_client_modeset.c hunk was not
	 * ported): the property starts at its max (10), so the fbdev modeset gets
	 * 10-bit whenever get_inp_bus_fmts' ten_bit_ok holds (sink EDID bpc >= 10
	 * and 444-10 fits the TMDS ceiling) and 8-bit otherwise (e.g. 4K), and an
	 * fbdev restore after a client changed max_bpc inherits the client's
	 * value unchanged (DC-001c). (Matches the lineage-A s805x-dc001 tip; the
	 * cited lineage-B SHA range omitted it.)
	 */
	drm_connector_attach_max_bpc_property(meson_encoder_hdmi->connector, 8, 10);

	/*
	 * VRR-001d: expose vrr_capable so userspace can tell whether this sink
	 * will accept a varying frame rate before it asks for one, and wrap the
	 * bridge connector's .get_modes so the value is refreshed from the EDID
	 * on every probe (see meson_encoder_hdmi_connector_get_modes).
	 */
	ret = drm_connector_attach_vrr_capable_property(meson_encoder_hdmi->connector);
	if (ret) {
		dev_err_probe(priv->dev, ret,
			      "Unable to attach vrr_capable property\n");
		goto err_put_node;
	}

	meson_encoder_hdmi->orig_conn_helper =
		meson_encoder_hdmi->connector->helper_private;
	if (meson_encoder_hdmi->orig_conn_helper) {
		meson_encoder_hdmi->conn_helper = *meson_encoder_hdmi->orig_conn_helper;
		meson_encoder_hdmi->conn_helper.get_modes =
			meson_encoder_hdmi_connector_get_modes;
		drm_connector_helper_add(meson_encoder_hdmi->connector,
					 &meson_encoder_hdmi->conn_helper);
	}

	/* Handle this here until handled by drm_bridge_connector_init() */
	meson_encoder_hdmi->connector->ycbcr_420_allowed = true;

	pdev = of_find_device_by_node(remote);
	of_node_put(remote);
	if (pdev) {
		struct cec_connector_info conn_info;
		struct cec_notifier *notifier;

		cec_fill_conn_info_from_drm(&conn_info, meson_encoder_hdmi->connector);

		notifier = cec_notifier_conn_register(&pdev->dev, NULL, &conn_info);
		if (!notifier) {
			put_device(&pdev->dev);
			return -ENOMEM;
		}

		meson_encoder_hdmi->cec_notifier = notifier;
	}

	priv->encoders[MESON_ENC_HDMI] = meson_encoder_hdmi;

	dev_dbg(priv->dev, "HDMI encoder initialized\n");

	return 0;

err_put_node:
	of_node_put(remote);
	return ret;
}

void meson_encoder_hdmi_remove(struct meson_drm *priv)
{
	struct meson_encoder_hdmi *meson_encoder_hdmi;

	if (priv->encoders[MESON_ENC_HDMI]) {
		meson_encoder_hdmi = priv->encoders[MESON_ENC_HDMI];
		drm_bridge_remove(&meson_encoder_hdmi->bridge);
	}
}
