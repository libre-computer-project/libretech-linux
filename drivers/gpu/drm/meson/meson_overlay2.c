// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 Libre Computer
 *
 * Second video overlay plane (VD2) for the Amlogic Meson VPU.
 *
 * The mainline meson DRM driver exposes a single video plane (VD1, in
 * meson_overlay.c) and leaves the VD2 hardware layer dormant: VD2's IF0
 * registers were mirror-written from VD1 in the vsync handler, but VD2 was
 * never enabled in the blend tree (VD2_BLEND_SRC_CTRL / VPP_VD2_* were only
 * ever cleared). This file brings VD2 up as an independent DRM_PLANE_TYPE_OVERLAY
 * plane so two video buffers can be composited at once (picture-in-picture).
 *
 * Phase 1 limitation: VD2 is 1:1 only (no scaling). The single VPP polyphase
 * scaler is owned by VD1; VD2 is positioned via its IF0 source crop and the
 * VPP_BLEND_VD2 destination window, which needs no scaler for an unscaled
 * source. atomic_check rejects any scale request. Independent VD2 scaling
 * (shared-scaler arbitration) is a follow-up.
 *
 * The actual register flush + VD2 blend enable/disable happens in
 * meson_crtc_irq() (meson_crtc.c); this file fills the priv->viu.vd2_* shadow
 * state, modelled on meson_overlay.c's non-AFBC VD1 path.
 */

#include <linux/bitfield.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_blend.h>
#include <drm/drm_device.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>

#include "meson_overlay.h"
#include "meson_overlay2.h"
#include "meson_registers.h"
#include "meson_viu.h"
#include "meson_vpp.h"

/* VD2_IF0_GEN_REG (same field layout as VD1_IF0_GEN_REG) */
#define VD_URGENT_CHROMA		BIT(28)
#define VD_URGENT_LUMA			BIT(27)
#define VD_HOLD_LINES(lines)		FIELD_PREP(GENMASK(24, 19), lines)
#define VD_DEMUX_MODE_RGB		BIT(16)
#define VD_BYTES_PER_PIXEL(val)		FIELD_PREP(GENMASK(15, 14), val)
#define VD_CHRO_RPT_LASTL_CTRL		BIT(6)
#define VD_LITTLE_ENDIAN		BIT(4)
#define VD_SEPARATE_EN			BIT(1)
#define VD_ENABLE			BIT(0)

/* VD2_IF0_CANVAS0 */
#define CANVAS_ADDR2(addr)		FIELD_PREP(GENMASK(23, 16), addr)
#define CANVAS_ADDR1(addr)		FIELD_PREP(GENMASK(15, 8), addr)
#define CANVAS_ADDR0(addr)		FIELD_PREP(GENMASK(7, 0), addr)

/* VD2_IF0_LUMA_X0 VD2_IF0_CHROMA_X0 */
#define VD_X_START(value)		FIELD_PREP(GENMASK(14, 0), value)
#define VD_X_END(value)			FIELD_PREP(GENMASK(30, 16), value)

/* VD2_IF0_LUMA_Y0 VD2_IF0_CHROMA_Y0 */
#define VD_Y_START(value)		FIELD_PREP(GENMASK(12, 0), value)
#define VD_Y_END(value)			FIELD_PREP(GENMASK(28, 16), value)

/* VD2_IF0_GEN_REG2 */
#define VD_COLOR_MAP(value)		FIELD_PREP(GENMASK(1, 0), value)

/* VIU_VD2_FMT_CTRL */
#define VD_HORZ_Y_C_RATIO(value)	FIELD_PREP(GENMASK(22, 21), value)
#define VD_HORZ_FMT_EN			BIT(20)
#define VD_VERT_RPT_LINE0		BIT(16)
#define VD_VERT_INITIAL_PHASE(value)	FIELD_PREP(GENMASK(11, 8), value)
#define VD_VERT_PHASE_STEP(value)	FIELD_PREP(GENMASK(7, 1), value)
#define VD_VERT_FMT_EN			BIT(0)

/* VIU_VD2_FMT_W */
#define VD_V_WIDTH(value)		FIELD_PREP(GENMASK(11, 0), value)
#define VD_H_WIDTH(value)		FIELD_PREP(GENMASK(27, 16), value)

/* VPP_BLEND_VD2_H_START_END (destination window into the preblend) */
#define VD_H_END(value)			FIELD_PREP(GENMASK(11, 0), value)
#define VD_H_START(value)		FIELD_PREP(GENMASK(27, 16), \
						   ((value) & GENMASK(13, 0)))

/* VPP_BLEND_VD2_V_START_END */
#define VD_V_END(value)			FIELD_PREP(GENMASK(11, 0), value)
#define VD_V_START(value)		FIELD_PREP(GENMASK(27, 16), value)

/* VD2 AFBC decoder field macros (AFBC-002), mirroring meson_overlay.c's VD1 set
 * -- this file already duplicates the VD_* macros from there in the same style. */
#define AFBC_DEC_ENABLE			BIT(8)
#define AFBC_COMPBITS_YUV(value)	FIELD_PREP(GENMASK(13, 8), value)
#define AFBC_COMPBITS_8BIT		0
#define AFBC_COMPBITS_10BIT		(2 | (2 << 2) | (2 << 4))
#define AFBC_BURST_LEN(value)		FIELD_PREP(GENMASK(15, 14), value)
#define AFBC_HOLD_LINE_NUM(value)	FIELD_PREP(GENMASK(22, 16), value)
#define AFBC_MIF_URGENT(value)		FIELD_PREP(GENMASK(25, 24), value)
#define AFBC_BLK_MEM_MODE		BIT(28)
#define AFBC_SCATTER_MODE		BIT(29)
#define AFBC_HSIZE_IN(value)		FIELD_PREP(GENMASK(28, 16), value)
#define AFBC_VSIZE_IN(value)		FIELD_PREP(GENMASK(12, 0), value)
#define AFBC_DEF_COLOR_Y(value)		FIELD_PREP(GENMASK(29, 20), value)
#define AFBC_DEF_COLOR_U(value)		FIELD_PREP(GENMASK(19, 10), value)
#define AFBC_DEF_COLOR_V(value)		FIELD_PREP(GENMASK(9, 0), value)
#define AFBC_CONV_LBUF_LEN(value)	FIELD_PREP(GENMASK(11, 0), value)
#define AFBC_HSIZE_OUT(value)		FIELD_PREP(GENMASK(28, 16), value)
#define AFBC_VSIZE_OUT(value)		FIELD_PREP(GENMASK(12, 0), value)
#define AFBC_HORZ_RPT_PIXEL0		BIT(23)
#define AFBC_HORZ_Y_C_RATIO(value)	FIELD_PREP(GENMASK(22, 21), value)
#define AFBC_HORZ_FMT_EN		BIT(20)
#define AFBC_VERT_RPT_LINE0		BIT(16)
#define AFBC_VERT_INITIAL_PHASE(value)	FIELD_PREP(GENMASK(11, 8), value)
#define AFBC_VERT_PHASE_STEP(value)	FIELD_PREP(GENMASK(7, 1), value)
#define AFBC_VERT_FMT_EN		BIT(0)
#define AFBC_VD_V_WIDTH(value)		FIELD_PREP(GENMASK(11, 0), value)
#define AFBC_VD_H_WIDTH(value)		FIELD_PREP(GENMASK(27, 16), value)
#define AFBC_MIF_BLK_BGN_H(value)	FIELD_PREP(GENMASK(25, 16), value)
#define AFBC_MIF_BLK_END_H(value)	FIELD_PREP(GENMASK(9, 0), value)
#define AFBC_MIF_BLK_BGN_V(value)	FIELD_PREP(GENMASK(27, 16), value)
#define AFBC_MIF_BLK_END_V(value)	FIELD_PREP(GENMASK(11, 0), value)
#define AFBC_DEC_PIXEL_BGN_H(value)	FIELD_PREP(GENMASK(28, 16), value)
#define AFBC_DEC_PIXEL_END_H(value)	FIELD_PREP(GENMASK(12, 0), value)
#define AFBC_DEC_PIXEL_BGN_V(value)	FIELD_PREP(GENMASK(28, 16), value)
#define AFBC_DEC_PIXEL_END_V(value)	FIELD_PREP(GENMASK(12, 0), value)
#define AFBC_VD_HEIGHT(value)		FIELD_PREP(GENMASK(12, 0), value)

struct meson_overlay2 {
	struct drm_plane base;
	struct meson_drm *priv;
};
#define to_meson_overlay2(x) container_of(x, struct meson_overlay2, base)

static int meson_overlay2_atomic_check(struct drm_plane *plane,
				       struct drm_atomic_state *state)
{
	struct drm_plane_state *new_plane_state =
		drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc_state *crtc_state;
	int ret;

	if (!new_plane_state->crtc)
		return 0;

	crtc_state = drm_atomic_get_crtc_state(state, new_plane_state->crtc);
	if (IS_ERR(crtc_state))
		return PTR_ERR(crtc_state);

	/*
	 * VD2 is 1:1 (no scaling) by design: the meson VPU has a single
	 * pre-blend polyphase scaler and the VD1 plane owns it, so VD2 cannot
	 * scale independently. Reject any scale request (min == max ==
	 * DRM_PLANE_NO_SCALING); positioning (crtc_x/y) is still allowed, which
	 * is sufficient for a 1:1 picture-in-picture of a pre-sized source.
	 */
	ret = drm_atomic_helper_check_plane_state(new_plane_state,
						   crtc_state,
						   DRM_PLANE_NO_SCALING,
						   DRM_PLANE_NO_SCALING,
						   true, true);
	if (ret)
		return ret;

	/*
	 * #19: VD2 has no interlace handling at all -- unlike VD1 (Phase 3
	 * per-field VFORMATTER phase override, meson_overlay.c) and OSD1/OSD2
	 * (destination-window halving + vertical-scaler VSC_PROG_INTERLACE),
	 * VD2's destination window (vd2_dst_h_start_end/vd2_dst_v_start_end,
	 * atomic_update below) stays in full-frame coordinates on an
	 * interlaced mode, with no field-phase selection. Correct support
	 * needs the same per-commit field state VD1 gets from
	 * meson_overlay_state (a VD2-specific plane-state extension this file
	 * does not have) plus hardware verification. Refuse cleanly instead
	 * of compositing wrong output.
	 */
	if (crtc_state->mode.flags & DRM_MODE_FLAG_INTERLACE)
		return -EINVAL;

	/*
	 * #17: VD2's 1:1 fetch window (VD_X_START/VD_Y_START, atomic_update
	 * below) now honours a non-zero source crop for linear buffers. AFBC
	 * input still hardcodes afbc_left=afbc_top=0 (full-frame): VD1's AFBC
	 * crop math is derived from meson_overlay_setup_scaler_params()'s
	 * scaler-coupled vd_start_lines/hd_start_lines, which does not carry
	 * over to VD2's 1:1 (no-scaler) case without a re-derivation this
	 * cannot be verified without hardware. Reject a non-zero crop on an
	 * AFBC framebuffer instead of silently ignoring it.
	 */
	if (new_plane_state->fb &&
	    (new_plane_state->fb->modifier & DRM_FORMAT_MOD_AMLOGIC_FBC(0, 0)) ==
			DRM_FORMAT_MOD_AMLOGIC_FBC(0, 0) &&
	    (new_plane_state->src_x || new_plane_state->src_y))
		return -EINVAL;

	return 0;
}

/* Takes a fixed 16.16 number and converts it to integer. */
static inline int64_t fixed16_to_int(int64_t value)
{
	return value >> 16;
}

static void meson_overlay2_atomic_update(struct drm_plane *plane,
					 struct drm_atomic_state *state)
{
	struct meson_overlay2 *meson_overlay2 = to_meson_overlay2(plane);
	struct drm_plane_state *new_state =
		drm_atomic_get_new_plane_state(state, plane);
	struct drm_framebuffer *fb = new_state->fb;
	struct meson_drm *priv = meson_overlay2->priv;
	struct drm_gem_dma_object *gem;
	unsigned int src_w, src_h, src_x, src_y, dst_x, dst_y, dst_w, dst_h;
	unsigned long flags;

	DRM_DEBUG_DRIVER("\n");

	/* 1:1 (enforced by atomic_check): src size == dst size */
	src_w = fixed16_to_int(new_state->src_w);
	src_h = fixed16_to_int(new_state->src_h);
	/* #17: crop offset into the framebuffer (linear formats only -- AFBC
	 * crop is rejected in atomic_check, see the comment there). */
	src_x = fixed16_to_int(new_state->src_x);
	src_y = fixed16_to_int(new_state->src_y);
	dst_x = new_state->crtc_x;
	dst_y = new_state->crtc_y;
	dst_w = new_state->crtc_w;
	dst_h = new_state->crtc_h;

	spin_lock_irqsave(&priv->drm->event_lock, flags);

	/* Phase 2 stage 2: stash zpos for the VPP post-blend port assignment */
	priv->viu.vd2_zpos = new_state->zpos;

	if ((fb->modifier & DRM_FORMAT_MOD_AMLOGIC_FBC(0, 0)) ==
			    DRM_FORMAT_MOD_AMLOGIC_FBC(0, 0)) {
		/*
		 * AFBC-002: VD2 AMLOGIC_FBC input -- mirror of the VD1 AFBC path
		 * in meson_overlay.c, with the scope math collapsed for the 1:1
		 * (no-scaler) VD2 case: afbc_left=afbc_top=0, full-frame block-
		 * aligned right/bottom. Feeds VD2 via the VD2_AFBC_* (0x3180)
		 * block + the VD2_AFBCD1_MISC_CTRL routing set in the crtc.
		 */
		u32 afbc_bottom = round_up(src_h, 4);
		u32 afbc_bottom_src = round_up(src_h + 1, 4);
		u32 afbc_right = round_up(src_w, 32);

		priv->viu.vd2_afbc = true;

		priv->viu.vd2_afbc_mode = AFBC_MIF_URGENT(3) |
					  AFBC_HOLD_LINE_NUM(8) |
					  AFBC_BURST_LEN(2);
		if (fb->modifier & DRM_FORMAT_MOD_AMLOGIC_FBC(0,
						AMLOGIC_FBC_OPTION_MEM_SAVING))
			priv->viu.vd2_afbc_mode |= AFBC_BLK_MEM_MODE;
		if ((fb->modifier & __fourcc_mod_amlogic_layout_mask) ==
				AMLOGIC_FBC_LAYOUT_SCATTER)
			priv->viu.vd2_afbc_mode |= AFBC_SCATTER_MODE;

		priv->viu.vd2_afbc_en = 0x1600 | AFBC_DEC_ENABLE;
		priv->viu.vd2_afbc_conv_ctrl = AFBC_CONV_LBUF_LEN(256);
		priv->viu.vd2_afbc_dec_def_color = AFBC_DEF_COLOR_Y(1023);
		priv->viu.vd2_afbc_vd_cfmt_ctrl = AFBC_HORZ_RPT_PIXEL0 |
						  AFBC_HORZ_Y_C_RATIO(1) |
						  AFBC_HORZ_FMT_EN |
						  AFBC_VERT_RPT_LINE0 |
						  AFBC_VERT_INITIAL_PHASE(12) |
						  AFBC_VERT_PHASE_STEP(8) |
						  AFBC_VERT_FMT_EN;

		switch (fb->format->format) {
		case DRM_FORMAT_YUV420_10BIT:
			priv->viu.vd2_afbc_mode |=
				AFBC_COMPBITS_YUV(AFBC_COMPBITS_10BIT);
			priv->viu.vd2_afbc_dec_def_color |=
				AFBC_DEF_COLOR_U(512) | AFBC_DEF_COLOR_V(512);
			break;
		case DRM_FORMAT_YUV420_8BIT:
			priv->viu.vd2_afbc_dec_def_color |=
				AFBC_DEF_COLOR_U(128) | AFBC_DEF_COLOR_V(128);
			break;
		}

		priv->viu.vd2_afbc_vd_cfmt_w =
				AFBC_VD_H_WIDTH(afbc_right) |
				AFBC_VD_V_WIDTH(afbc_right / 2);
		priv->viu.vd2_afbc_vd_cfmt_h = AFBC_VD_HEIGHT(afbc_bottom / 2);
		priv->viu.vd2_afbc_mif_hor_scope =
				AFBC_MIF_BLK_BGN_H(0) |
				AFBC_MIF_BLK_END_H((afbc_right / 32) - 1);
		priv->viu.vd2_afbc_mif_ver_scope =
				AFBC_MIF_BLK_BGN_V(0) |
				AFBC_MIF_BLK_END_V((afbc_bottom / 4) - 1);
		priv->viu.vd2_afbc_size_out =
				AFBC_HSIZE_OUT(afbc_right) |
				AFBC_VSIZE_OUT(afbc_bottom);
		priv->viu.vd2_afbc_pixel_hor_scope =
				AFBC_DEC_PIXEL_BGN_H(0) |
				AFBC_DEC_PIXEL_END_H(src_w - 1);
		priv->viu.vd2_afbc_pixel_ver_scope =
				AFBC_DEC_PIXEL_BGN_V(0) |
				AFBC_DEC_PIXEL_END_V(src_h - 1);
		priv->viu.vd2_afbc_size_in =
				AFBC_HSIZE_IN(afbc_right) |
				AFBC_VSIZE_IN(afbc_bottom_src);

		/* AFBC bypasses the canvas IF0 fetch path */
		priv->viu.vd2_if0_gen_reg = 0;
		priv->viu.vd2_if0_gen_reg2 = 0;
		priv->viu.vd2_if0_canvas0 = 0;
		priv->viu.viu_vd2_fmt_ctrl = 0;
		priv->viu.vd2_if0_repeat_loop = 0;
		priv->viu.vd2_if0_luma0_rpt_pat = 0;
		priv->viu.vd2_if0_chroma0_rpt_pat = 0;
		goto fetch;
	}

	priv->viu.vd2_afbc = false;

	priv->viu.vd2_if0_gen_reg = VD_URGENT_CHROMA |
				    VD_URGENT_LUMA |
				    VD_HOLD_LINES(9) |
				    VD_CHRO_RPT_LASTL_CTRL |
				    VD_ENABLE;

	priv->viu.vd2_if0_repeat_loop = 0;
	priv->viu.vd2_if0_luma0_rpt_pat = 0;
	priv->viu.vd2_if0_chroma0_rpt_pat = 0;

	/* Default values for RGB888/YUV444 */
	priv->viu.vd2_if0_gen_reg2 = 0;
	priv->viu.viu_vd2_fmt_ctrl = 0;

	switch (fb->format->format) {
	case DRM_FORMAT_RGB888:
		priv->viu.vd2_if0_gen_reg |= VD_DEMUX_MODE_RGB |
					     VD_BYTES_PER_PIXEL(2) |
					     VD_LITTLE_ENDIAN;
		priv->viu.vd2_if0_canvas0 =
					CANVAS_ADDR2(priv->canvas_id_vd2_0) |
					CANVAS_ADDR1(priv->canvas_id_vd2_0) |
					CANVAS_ADDR0(priv->canvas_id_vd2_0);
		break;
	case DRM_FORMAT_YUYV:
		priv->viu.vd2_if0_gen_reg |= VD_BYTES_PER_PIXEL(1);
		priv->viu.vd2_if0_canvas0 =
					CANVAS_ADDR2(priv->canvas_id_vd2_0) |
					CANVAS_ADDR1(priv->canvas_id_vd2_0) |
					CANVAS_ADDR0(priv->canvas_id_vd2_0);
		priv->viu.viu_vd2_fmt_ctrl = VD_HORZ_Y_C_RATIO(1) | /* /2 */
					     VD_HORZ_FMT_EN |
					     VD_VERT_RPT_LINE0 |
					     VD_VERT_INITIAL_PHASE(12) |
					     VD_VERT_PHASE_STEP(16) | /* /2 */
					     VD_VERT_FMT_EN;
		break;
	case DRM_FORMAT_NV12:
	case DRM_FORMAT_NV21:
		priv->viu.vd2_if0_gen_reg |= VD_SEPARATE_EN;
		priv->viu.vd2_if0_canvas0 =
					CANVAS_ADDR2(priv->canvas_id_vd2_1) |
					CANVAS_ADDR1(priv->canvas_id_vd2_1) |
					CANVAS_ADDR0(priv->canvas_id_vd2_0);
		if (fb->format->format == DRM_FORMAT_NV12)
			priv->viu.vd2_if0_gen_reg2 = VD_COLOR_MAP(1);
		else
			priv->viu.vd2_if0_gen_reg2 = VD_COLOR_MAP(2);
		priv->viu.viu_vd2_fmt_ctrl = VD_HORZ_Y_C_RATIO(1) | /* /2 */
					     VD_HORZ_FMT_EN |
					     VD_VERT_RPT_LINE0 |
					     VD_VERT_INITIAL_PHASE(12) |
					     VD_VERT_PHASE_STEP(8) | /* /4 */
					     VD_VERT_FMT_EN;
		break;
	case DRM_FORMAT_YUV420:
		/* 4:2:0 planar (3 plane). 4:2:2/4:4:4 planar are NOT supported
		 * yet: this plane's chroma-height is hardcoded to src_h/2 below
		 * (correct only for 4:2:0). */
		priv->viu.vd2_if0_gen_reg |= VD_SEPARATE_EN;
		priv->viu.vd2_if0_canvas0 =
					CANVAS_ADDR2(priv->canvas_id_vd2_2) |
					CANVAS_ADDR1(priv->canvas_id_vd2_1) |
					CANVAS_ADDR0(priv->canvas_id_vd2_0);
		priv->viu.viu_vd2_fmt_ctrl = VD_HORZ_Y_C_RATIO(1) | /* /2 */
					     VD_HORZ_FMT_EN |
					     VD_VERT_RPT_LINE0 |
					     VD_VERT_INITIAL_PHASE(12) |
					     VD_VERT_PHASE_STEP(8) | /* /4 */
					     VD_VERT_FMT_EN;
		break;
	}

	/* Source crop (1:1, so window size == dest size; #17: honour a
	 * non-zero src_x/src_y offset into the framebuffer instead of always
	 * reading from the top-left corner -- byte-identical to the old code
	 * when src_x==src_y==0, the common case). Chroma offset/extent halved
	 * to match the existing chroma-height halving (correct for 4:2:0). */
	priv->viu.vd2_if0_luma_x0 = VD_X_START(src_x) | VD_X_END(src_x + src_w - 1);
	priv->viu.vd2_if0_luma_y0 = VD_Y_START(src_y) | VD_Y_END(src_y + src_h - 1);
	priv->viu.vd2_if0_chroma_x0 =
			VD_X_START(src_x / 2) |
			VD_X_END((src_x + src_w) / 2 - 1);
	priv->viu.vd2_if0_chroma_y0 =
			VD_Y_START(src_y / 2) |
			VD_Y_END((src_y + src_h) / 2 - 1);

	priv->viu.viu_vd2_fmt_w = VD_H_WIDTH(src_w) | VD_V_WIDTH(src_w / 2);

fetch:
	/* Destination window into the preblend (no scaling: 1:1) */
	priv->viu.vd2_dst_h_start_end =
			VD_H_START(dst_x) | VD_H_END(dst_x + dst_w - 1);
	priv->viu.vd2_dst_v_start_end =
			VD_V_START(dst_y) | VD_V_END(dst_y + dst_h - 1);

	/* Update canvas with buffer address */
	priv->viu.vd2_planes = fb->format->num_planes;

	switch (priv->viu.vd2_planes) {
	case 3:
		gem = drm_fb_dma_get_gem_obj(fb, 2);
		priv->viu.vd2_addr2 = gem->dma_addr + fb->offsets[2];
		priv->viu.vd2_stride2 = fb->pitches[2];
		priv->viu.vd2_height2 =
			drm_format_info_plane_height(fb->format, fb->height, 2);
		fallthrough;
	case 2:
		gem = drm_fb_dma_get_gem_obj(fb, 1);
		priv->viu.vd2_addr1 = gem->dma_addr + fb->offsets[1];
		priv->viu.vd2_stride1 = fb->pitches[1];
		priv->viu.vd2_height1 =
			drm_format_info_plane_height(fb->format, fb->height, 1);
		fallthrough;
	case 1:
		gem = drm_fb_dma_get_gem_obj(fb, 0);
		priv->viu.vd2_addr0 = gem->dma_addr + fb->offsets[0];
		priv->viu.vd2_stride0 = fb->pitches[0];
		priv->viu.vd2_height0 =
			drm_format_info_plane_height(fb->format, fb->height, 0);
	}

	if (priv->viu.vd2_afbc) {
		dma_addr_t fbc_header;

		/*
		 * Head/body addr -- ported from meson_overlay.c's VD1 AFBC fix
		 * (AFBC-002). Scatter mode STILL needs the body base address:
		 * zeroing body_addr made the AFBC decoder fetch body
		 * superblocks from address 0 -> garbage (the exact bug VD1
		 * documents fixing, SM1 10-bit AMSA, 7.64 dB HDMI-capture
		 * loopback). Layout: [body @ offsets[0]][header @ +body_size].
		 * Scatter mode sizes the body from the frame WIDTH (the FB
		 * pitch is 0 for AFBC), matching VD1's scatter branch exactly;
		 * non-scatter sizes it from the actual stride, unchanged.
		 */
		if (priv->viu.vd2_afbc_mode & AFBC_SCATTER_MODE) {
			unsigned long block_size =
				(priv->viu.vd2_afbc_mode & AFBC_BLK_MEM_MODE) ?
					3072 : 4096;
			unsigned long body_size =
				(ALIGN(fb->width, 64) / 64) *
				(ALIGN(priv->viu.vd2_height0, 32) / 32) *
				block_size;

			priv->viu.vd2_afbc_body_addr = priv->viu.vd2_addr0 >> 4;
			priv->viu.vd2_afbc_head_addr =
				(priv->viu.vd2_addr0 + body_size) >> 4;
		} else {
			unsigned long block_size = 4096; /* 4k per superblock */
			unsigned long body_size;

			if (priv->viu.vd2_afbc_mode & AFBC_BLK_MEM_MODE)
				block_size = 3072; /* 8bit mem-saving */

			body_size = (ALIGN(priv->viu.vd2_stride0, 64) / 64) *
				    (ALIGN(priv->viu.vd2_height0, 32) / 32) *
				    block_size;

			priv->viu.vd2_afbc_body_addr = priv->viu.vd2_addr0 >> 4;
			priv->viu.vd2_afbc_head_addr =
				(priv->viu.vd2_addr0 + body_size) >> 4;
		}

		/*
		 * Finding #2 (hdmi-vpu-gap-review.md): mirror VD1's registered-
		 * header lookup. For a use_mmu decode (G12A+/SM1) the compress
		 * HEADER is NOT in the vb2 buffer -- meson-vdec wrote it to a
		 * separate per-slot buffer and registered the {body base ->
		 * header} association via meson_overlay_fbc_set_header(). If
		 * the registry has a header for this body base, point the AFBC
		 * HEAD at it and keep the body at the vb2 base; this supersedes
		 * the computed head above. If NOT registered (e.g. a non-decoder
		 * FBC FB, or a decode path that didn't register), fall back to
		 * the computed head -- do not crash.
		 */
		fbc_header = meson_overlay_fbc_get_header(priv->viu.vd2_addr0);
		if (fbc_header) {
			priv->viu.vd2_afbc_body_addr = priv->viu.vd2_addr0 >> 4;
			priv->viu.vd2_afbc_head_addr = fbc_header >> 4;
		} else {
			DRM_DEBUG("VD2 AFBC body 0x%x: no registered compress header, using computed head 0x%x\n",
				  priv->viu.vd2_addr0,
				  priv->viu.vd2_afbc_head_addr);
		}
	}

	priv->viu.vd2_enabled = true;

	spin_unlock_irqrestore(&priv->drm->event_lock, flags);

	DRM_DEBUG_DRIVER("\n");
}

static void meson_overlay2_atomic_disable(struct drm_plane *plane,
					  struct drm_atomic_state *state)
{
	struct meson_overlay2 *meson_overlay2 = to_meson_overlay2(plane);
	struct meson_drm *priv = meson_overlay2->priv;

	DRM_DEBUG_DRIVER("\n");

	priv->viu.vd2_enabled = false;

	/* Disable VD2 in the blend tree */
	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A)) {
		writel_relaxed(0, priv->io_base + _REG(VD2_BLEND_SRC_CTRL));
		writel_relaxed(0, priv->io_base + _REG(VD2_IF0_GEN_REG + 0x17b0));
	} else {
		writel_bits_relaxed(VPP_VD2_POSTBLEND | VPP_VD2_PREBLEND, 0,
				    priv->io_base + _REG(VPP_MISC));
	}
}

static const struct drm_plane_helper_funcs meson_overlay2_helper_funcs = {
	.atomic_check	= meson_overlay2_atomic_check,
	.atomic_disable	= meson_overlay2_atomic_disable,
	.atomic_update	= meson_overlay2_atomic_update,
};

/* AFBC-002: accept linear (for the non-FBC formats) + AMLOGIC_FBC (8/10-bit
 * YUV420 only), mirroring meson_overlay.c's VD1 modifier validation. */
static bool meson_overlay2_format_mod_supported(struct drm_plane *plane,
						u32 format, u64 modifier)
{
	if (modifier == DRM_FORMAT_MOD_LINEAR &&
	    format != DRM_FORMAT_YUV420_8BIT &&
	    format != DRM_FORMAT_YUV420_10BIT)
		return true;

	if ((modifier & DRM_FORMAT_MOD_AMLOGIC_FBC(0, 0)) ==
			DRM_FORMAT_MOD_AMLOGIC_FBC(0, 0)) {
		unsigned int layout = modifier &
			DRM_FORMAT_MOD_AMLOGIC_FBC(
				__fourcc_mod_amlogic_layout_mask, 0);
		unsigned int options =
			(modifier >> __fourcc_mod_amlogic_options_shift) &
			__fourcc_mod_amlogic_options_mask;

		/* AMLOGIC_FBC is G12A+ only (no video FBC decoder on pre-G12A, and the
		 * meson-vdec there outputs linear NV12). Gate it off so GXL/GXM stop
		 * advertising an unusable modifier. Mirror of the VD1 overlay gate. */
		if (!meson_vpu_is_compatible(to_meson_overlay2(plane)->priv,
					     VPU_COMPATIBLE_G12A))
			return false;

		if (format != DRM_FORMAT_YUV420_8BIT &&
		    format != DRM_FORMAT_YUV420_10BIT)
			return false;
		if (layout != AMLOGIC_FBC_LAYOUT_BASIC &&
		    layout != AMLOGIC_FBC_LAYOUT_SCATTER)
			return false;
		if (options && options != AMLOGIC_FBC_OPTION_MEM_SAVING)
			return false;

		return true;
	}

	return false;
}

static const struct drm_plane_funcs meson_overlay2_funcs = {
	.update_plane		= drm_atomic_helper_update_plane,
	.disable_plane		= drm_atomic_helper_disable_plane,
	.destroy		= drm_plane_cleanup,
	.reset			= drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_plane_destroy_state,
	.format_mod_supported	= meson_overlay2_format_mod_supported,
};

/*
 * Linear: 4:2:0 family + RGB888 (planar 4:2:2/4:4:4 deferred -- the 1:1 path
 * hardcodes chroma height to src_h/2, valid for 4:2:0). AFBC-only formats
 * (YUV420 8/10-bit) are accepted via the AMLOGIC_FBC modifiers below.
 *
 * #18: DRM_FORMAT_YUYV (packed 4:2:2) used to be advertised here despite this
 * file's own chroma-height math being 4:2:0-only (src_h/2 on BOTH axes;
 * 4:2:2 chroma is full height, not halved) -- and despite the vertical
 * formatter (VD_VERT_FMT_EN/VD_VERT_PHASE_STEP) YUYV's switch case programs
 * applying that same wrong /2 vertical chroma decimation. Fixing the
 * formatter geometry needs datasheet-level VD-formatter detail; drop the
 * format from the advertised list instead of shipping one that silently
 * loses vertical chroma resolution.
 */
static const uint32_t supported_drm_formats[] = {
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV21,
	DRM_FORMAT_YUV420,
	DRM_FORMAT_RGB888,
	DRM_FORMAT_YUV420_8BIT,  /* Amlogic FBC only */
	DRM_FORMAT_YUV420_10BIT, /* Amlogic FBC only */
};

/* AFBC-002: advertise the AMLOGIC_FBC layouts (mirror of the VD1 overlay) */
static const uint64_t format_modifiers[] = {
	DRM_FORMAT_MOD_AMLOGIC_FBC(AMLOGIC_FBC_LAYOUT_SCATTER,
				   AMLOGIC_FBC_OPTION_MEM_SAVING),
	DRM_FORMAT_MOD_AMLOGIC_FBC(AMLOGIC_FBC_LAYOUT_BASIC,
				   AMLOGIC_FBC_OPTION_MEM_SAVING),
	DRM_FORMAT_MOD_AMLOGIC_FBC(AMLOGIC_FBC_LAYOUT_SCATTER, 0),
	DRM_FORMAT_MOD_AMLOGIC_FBC(AMLOGIC_FBC_LAYOUT_BASIC, 0),
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID,
};

int meson_overlay2_create(struct meson_drm *priv)
{
	struct meson_overlay2 *meson_overlay2;
	struct drm_plane *plane;
	int ret;

	DRM_DEBUG_DRIVER("\n");

	meson_overlay2 = devm_kzalloc(priv->drm->dev, sizeof(*meson_overlay2),
				      GFP_KERNEL);
	if (!meson_overlay2)
		return -ENOMEM;

	meson_overlay2->priv = priv;
	plane = &meson_overlay2->base;

	/* #29: return value used to go unchecked. */
	ret = drm_universal_plane_init(priv->drm, plane, 0xFF,
					&meson_overlay2_funcs,
					supported_drm_formats,
					ARRAY_SIZE(supported_drm_formats),
					format_modifiers,
					DRM_PLANE_TYPE_OVERLAY, "meson_overlay2_plane");
	if (ret) {
		devm_kfree(priv->drm->dev, meson_overlay2);
		return ret;
	}

	drm_plane_helper_add(plane, &meson_overlay2_helper_funcs);

	/* VD2 video plane. On G12A+ the VPP post-blend ports are reassignable, so
	 * zpos is mutable [0,3] (default 1 = above VD1, below the OSD band 2/3 ->
	 * video group below OSD, matching the Phase-1 default composite).
	 *
	 * #28: pre-G12A (GXL/GXM) used to expose an immutable zpos of 3 (above
	 * OSD2=2/OSD1=1), contradicting the G12A+ "video below OSD" default and
	 * VD1's own pre-G12A immutable value of 0. The legacy VPP_MISC layout
	 * gives VD1/VD2 a dedicated PREBLEND stage (VPP_VD1_PREBLEND/
	 * VPP_VD2_PREBLEND, bits 14/15) combining them into one video signal
	 * upstream of the fixed POSTBLEND stack, while OSD1/OSD2 use
	 * POSTBLEND-only + FG-select bits -- the classic legacy Amlogic layout
	 * where preblended video sits below the OSD band. The vendor BSP
	 * (video_hw.c vpp_zorder_check()) also programs VD1/VD2 order
	 * dynamically via these same bits, confirming "immutable" here is this
	 * driver's own Phase-1 simplification (meson_crtc.c's pre-G12A
	 * enable_vd2()/enable_osd2() just toggle static bits, consuming neither
	 * vd2_zpos nor osd2_zpos) rather than a silicon constraint -- so the
	 * exposed VALUE should reflect the correct default composite. Tie VD2
	 * to VD1's value (0): both are preblended into one video layer below
	 * the OSD band. Reasoned from BSP + register layout, not hardware/
	 * capture-verified -- no genuine GXL-generation BSP snapshot was
	 * available, only the G12B/A311D-era video_hw.c. */
	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A))
		drm_plane_create_zpos_property(plane, 1, 0, 3);
	else
		drm_plane_create_zpos_immutable_property(plane, 0);

	priv->overlay2_plane = plane;

	DRM_DEBUG_DRIVER("\n");

	return 0;
}
