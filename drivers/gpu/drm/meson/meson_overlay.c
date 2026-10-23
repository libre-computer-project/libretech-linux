// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 * Copyright (C) 2015 Amlogic, Inc. All rights reserved.
 */

#include <linux/bitfield.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_blend.h>
#include <drm/drm_device.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_property.h>

#include "meson_overlay.h"
#include "meson_registers.h"
#include "meson_viu.h"
#include "meson_vpp.h"

/* VD1_IF0_GEN_REG */
#define VD_URGENT_CHROMA		BIT(28)
#define VD_URGENT_LUMA			BIT(27)
#define VD_HOLD_LINES(lines)		FIELD_PREP(GENMASK(24, 19), lines)
#define VD_DEMUX_MODE_RGB		BIT(16)
#define VD_BYTES_PER_PIXEL(val)		FIELD_PREP(GENMASK(15, 14), val)
#define VD_CHRO_RPT_LASTL_CTRL		BIT(6)
#define VD_LITTLE_ENDIAN		BIT(4)
#define VD_SEPARATE_EN			BIT(1)
#define VD_ENABLE			BIT(0)

/* VD1_IF0_CANVAS0 */
#define CANVAS_ADDR2(addr)		FIELD_PREP(GENMASK(23, 16), addr)
#define CANVAS_ADDR1(addr)		FIELD_PREP(GENMASK(15, 8), addr)
#define CANVAS_ADDR0(addr)		FIELD_PREP(GENMASK(7, 0), addr)

/* VD1_IF0_LUMA_X0 VD1_IF0_CHROMA_X0 */
#define VD_X_START(value)		FIELD_PREP(GENMASK(14, 0), value)
#define VD_X_END(value)			FIELD_PREP(GENMASK(30, 16), value)

/* VD1_IF0_LUMA_Y0 VD1_IF0_CHROMA_Y0 */
#define VD_Y_START(value)		FIELD_PREP(GENMASK(12, 0), value)
#define VD_Y_END(value)			FIELD_PREP(GENMASK(28, 16), value)

/* VD1_IF0_GEN_REG2 */
#define VD_COLOR_MAP(value)		FIELD_PREP(GENMASK(1, 0), value)

/* VIU_VD1_FMT_CTRL */
#define VD_HORZ_Y_C_RATIO(value)	FIELD_PREP(GENMASK(22, 21), value)
#define VD_HORZ_FMT_EN			BIT(20)
#define VD_VERT_RPT_LINE0		BIT(16)
#define VD_VERT_INITIAL_PHASE(value)	FIELD_PREP(GENMASK(11, 8), value)
#define VD_VERT_PHASE_STEP(value)	FIELD_PREP(GENMASK(7, 1), value)
#define VD_VERT_FMT_EN			BIT(0)

/* VPP_POSTBLEND_VD1_H_START_END */
#define VD_H_END(value)			FIELD_PREP(GENMASK(11, 0), value)
#define VD_H_START(value)		FIELD_PREP(GENMASK(27, 16), \
						   ((value) & GENMASK(13, 0)))

/* VPP_POSTBLEND_VD1_V_START_END */
#define VD_V_END(value)			FIELD_PREP(GENMASK(11, 0), value)
#define VD_V_START(value)		FIELD_PREP(GENMASK(27, 16), value)

/* VPP_BLEND_VD2_V_START_END */
#define VD2_V_END(value)		FIELD_PREP(GENMASK(11, 0), value)
#define VD2_V_START(value)		FIELD_PREP(GENMASK(27, 16), value)

/* VIU_VD1_FMT_W */
#define VD_V_WIDTH(value)		FIELD_PREP(GENMASK(11, 0), value)
#define VD_H_WIDTH(value)		FIELD_PREP(GENMASK(27, 16), value)

/* VPP_HSC_REGION12_STARTP VPP_HSC_REGION34_STARTP */
#define VD_REGION24_START(value)	FIELD_PREP(GENMASK(11, 0), value)
#define VD_REGION13_END(value)		FIELD_PREP(GENMASK(27, 16), value)

/* AFBC_ENABLE */
#define AFBC_DEC_ENABLE			BIT(8)
#define AFBC_FRM_START			BIT(0)

/* AFBC_MODE */
#define AFBC_HORZ_SKIP_UV(value)	FIELD_PREP(GENMASK(1, 0), value)
#define AFBC_VERT_SKIP_UV(value)	FIELD_PREP(GENMASK(3, 2), value)
#define AFBC_HORZ_SKIP_Y(value)		FIELD_PREP(GENMASK(5, 4), value)
#define AFBC_VERT_SKIP_Y(value)		FIELD_PREP(GENMASK(7, 6), value)
#define AFBC_COMPBITS_YUV(value)	FIELD_PREP(GENMASK(13, 8), value)
#define AFBC_COMPBITS_8BIT		0
#define AFBC_COMPBITS_10BIT		(2 | (2 << 2) | (2 << 4))
#define AFBC_BURST_LEN(value)		FIELD_PREP(GENMASK(15, 14), value)
#define AFBC_HOLD_LINE_NUM(value)	FIELD_PREP(GENMASK(22, 16), value)
#define AFBC_MIF_URGENT(value)		FIELD_PREP(GENMASK(25, 24), value)
#define AFBC_REV_MODE(value)		FIELD_PREP(GENMASK(27, 26), value)
#define AFBC_BLK_MEM_MODE		BIT(28)
#define AFBC_SCATTER_MODE		BIT(29)
#define AFBC_SOFT_RESET			BIT(31)

/* AFBC_SIZE_IN */
#define AFBC_HSIZE_IN(value)		FIELD_PREP(GENMASK(28, 16), value)
#define AFBC_VSIZE_IN(value)		FIELD_PREP(GENMASK(12, 0), value)

/* AFBC_DEC_DEF_COLOR */
#define AFBC_DEF_COLOR_Y(value)		FIELD_PREP(GENMASK(29, 20), value)
#define AFBC_DEF_COLOR_U(value)		FIELD_PREP(GENMASK(19, 10), value)
#define AFBC_DEF_COLOR_V(value)		FIELD_PREP(GENMASK(9, 0), value)

/* AFBC_CONV_CTRL */
#define AFBC_CONV_LBUF_LEN(value)	FIELD_PREP(GENMASK(11, 0), value)

/* AFBC_LBUF_DEPTH */
#define AFBC_DEC_LBUF_DEPTH(value)	FIELD_PREP(GENMASK(27, 16), value)
#define AFBC_MIF_LBUF_DEPTH(value)	FIELD_PREP(GENMASK(11, 0), value)

/* AFBC_OUT_XSCOPE/AFBC_SIZE_OUT */
#define AFBC_HSIZE_OUT(value)		FIELD_PREP(GENMASK(28, 16), value)
#define AFBC_VSIZE_OUT(value)		FIELD_PREP(GENMASK(12, 0), value)
#define AFBC_OUT_HORZ_BGN(value)	FIELD_PREP(GENMASK(28, 16), value)
#define AFBC_OUT_HORZ_END(value)	FIELD_PREP(GENMASK(12, 0), value)

/* AFBC_OUT_YSCOPE */
#define AFBC_OUT_VERT_BGN(value)	FIELD_PREP(GENMASK(28, 16), value)
#define AFBC_OUT_VERT_END(value)	FIELD_PREP(GENMASK(12, 0), value)

/* AFBC_VD_CFMT_CTRL */
#define AFBC_HORZ_RPT_PIXEL0		BIT(23)
#define AFBC_HORZ_Y_C_RATIO(value)	FIELD_PREP(GENMASK(22, 21), value)
#define AFBC_HORZ_FMT_EN		BIT(20)
#define AFBC_VERT_RPT_LINE0		BIT(16)
#define AFBC_VERT_INITIAL_PHASE(value)	FIELD_PREP(GENMASK(11, 8), value)
#define AFBC_VERT_PHASE_STEP(value)	FIELD_PREP(GENMASK(7, 1), value)
#define AFBC_VERT_FMT_EN		BIT(0)

/* AFBC_VD_CFMT_W */
#define AFBC_VD_V_WIDTH(value)		FIELD_PREP(GENMASK(11, 0), value)
#define AFBC_VD_H_WIDTH(value)		FIELD_PREP(GENMASK(27, 16), value)

/* AFBC_MIF_HOR_SCOPE */
#define AFBC_MIF_BLK_BGN_H(value)	FIELD_PREP(GENMASK(25, 16), value)
#define AFBC_MIF_BLK_END_H(value)	FIELD_PREP(GENMASK(9, 0), value)

/* AFBC_MIF_VER_SCOPE */
#define AFBC_MIF_BLK_BGN_V(value)	FIELD_PREP(GENMASK(27, 16), value)
#define AFBC_MIF_BLK_END_V(value)	FIELD_PREP(GENMASK(11, 0), value)

/* AFBC_PIXEL_HOR_SCOPE */
#define AFBC_DEC_PIXEL_BGN_H(value)	FIELD_PREP(GENMASK(28, 16), \
						   ((value) & GENMASK(12, 0)))
#define AFBC_DEC_PIXEL_END_H(value)	FIELD_PREP(GENMASK(12, 0), value)

/* AFBC_PIXEL_VER_SCOPE */
#define AFBC_DEC_PIXEL_BGN_V(value)	FIELD_PREP(GENMASK(28, 16), value)
#define AFBC_DEC_PIXEL_END_V(value)	FIELD_PREP(GENMASK(12, 0), value)

/* AFBC_VD_CFMT_H */
#define AFBC_VD_HEIGHT(value)		FIELD_PREP(GENMASK(12, 0), value)

/*
 * Phase 3: per-commit interlaced field input mode. Drives the VD1 VFORMATTER
 * vertical init-phase (VIU_VD1_FMT_CTRL[11:8]): progressive=0xc (default, the
 * chroma-siting value), top field=0xe, bottom field=0xa (BSP-confirmed). This
 * is field-phase selection (bob/weave), not motion-adaptive deinterlace (that
 * stays the DI block's job). Userspace alternates top/bottom per field flip.
 */
enum meson_overlay_field {
	MESON_FIELD_PROGRESSIVE = 0,
	MESON_FIELD_TOP = 1,
	MESON_FIELD_BOTTOM = 2,
};

struct meson_overlay {
	struct drm_plane base;
	struct meson_drm *priv;
	struct drm_property *field_prop;
};
#define to_meson_overlay(x) container_of(x, struct meson_overlay, base)

/* Plane state subclass carrying the custom "field" property value. */
struct meson_overlay_state {
	struct drm_plane_state base;
	u32 field;
};
#define to_meson_overlay_state(x) \
	container_of(x, struct meson_overlay_state, base)

#define FRAC_16_16(mult, div)    (((mult) << 16) / (div))

/*
 * FBC-output compress-header registry (fbc-output-display-impl-plan.md S1/S2).
 *
 * For an FBC-output (am21c) decode on use_mmu SoCs (G12A+/SM1), the decoder
 * writes the compress HEADER to a SEPARATE per-slot buffer mmu_header_paddr[slot]
 * (codec_hevc.c:1657-1658, HEVC_CM_HEADER_START_ADDR) -- it is NOT inside the
 * vb2 capture buffer.  The BODY *is* contiguous at offset 0 of the vb2 capture
 * buffer (vb2_dma_contig).  The VD1 AFBC scatter decoder therefore needs
 * AFBC_HEAD_BADDR = mmu_header_paddr[slot] and AFBC_BODY_BADDR = vb2 base.
 * The display side only has a handle to the vb2 buffer (the DRM FB), so
 * meson-vdec must hand us the per-buffer header phys addr.
 *
 * This registry is the channel.  It lives in meson-DRM (built-in =y) and is
 * written by meson-vdec (=m) via the EXPORT_SYMBOL_GPL'd setter below -- a =m
 * module may call a symbol exported by =y code, but not the reverse, so the
 * registry MUST live on the DRM side.  Keyed/looked-up by the body base (the
 * vb2 capture-buffer dma_addr, which equals gem->dma_addr + offsets[0] on the
 * display side).  No IOMMU anywhere on this fleet (GXL/G12A/G12B/SM1), so the
 * addresses are raw phys.  S0 PoC proved head=mmu_header_paddr -> 23.4 dB
 * (vs 7.7 dB for any vb2-buffer-based head); this supersedes the S0 PoC
 * debugfs node + fbc_head_override module param.
 */
#define MESON_FBC_HDR_REG_SIZE 32

struct meson_fbc_hdr_entry {
	dma_addr_t body_base;		/* vb2 capture-buffer base (AFBC body) */
	dma_addr_t header_paddr;	/* mmu_header_paddr[slot] (AFBC head) */
	u64 seq;			/* insertion order, for oldest-slot evict */
};

static struct meson_fbc_hdr_entry meson_fbc_hdr_reg[MESON_FBC_HDR_REG_SIZE];
static u64 meson_fbc_hdr_seq;
static DEFINE_SPINLOCK(meson_fbc_hdr_lock);

/**
 * meson_overlay_fbc_set_header - associate an FBC-output body base with its
 *	compress-header phys addr.
 * @body_base:   vb2 capture-buffer dma_addr (the contiguous am21c body base).
 * @header_paddr: mmu_header_paddr[slot] for that frame (the am21c compress
 *	header the VD1 AFBC scatter HEAD must read).  Pass 0 to clear the entry.
 *
 * Called by meson-vdec (codec_hevc_set_sao) per FBC-output frame, right after
 * the HW is programmed to write the header to mmu_header_paddr[slot].  Replaces
 * an existing entry for the same body_base; otherwise takes a free slot, or
 * evicts the oldest.  Strict no-op channel for non-FBC decode (meson-vdec only
 * calls this for use_mmu && codec_hevc_is_fbc_output).
 */
void meson_overlay_fbc_set_header(dma_addr_t body_base, dma_addr_t header_paddr)
{
	unsigned long flags;
	int i, free = -1, oldest = 0;
	u64 oldest_seq = U64_MAX;

	if (!body_base)
		return;

	spin_lock_irqsave(&meson_fbc_hdr_lock, flags);

	for (i = 0; i < MESON_FBC_HDR_REG_SIZE; i++) {
		if (meson_fbc_hdr_reg[i].body_base == body_base) {
			/* Update in place (header_paddr==0 clears the entry). */
			if (header_paddr) {
				meson_fbc_hdr_reg[i].header_paddr = header_paddr;
				meson_fbc_hdr_reg[i].seq = ++meson_fbc_hdr_seq;
			} else {
				meson_fbc_hdr_reg[i].body_base = 0;
				meson_fbc_hdr_reg[i].header_paddr = 0;
			}
			spin_unlock_irqrestore(&meson_fbc_hdr_lock, flags);
			return;
		}
		if (meson_fbc_hdr_reg[i].body_base == 0 && free < 0)
			free = i;
		if (meson_fbc_hdr_reg[i].seq < oldest_seq) {
			oldest_seq = meson_fbc_hdr_reg[i].seq;
			oldest = i;
		}
	}

	if (header_paddr) {
		/* Insert: prefer a free slot, else evict the oldest. */
		i = free >= 0 ? free : oldest;
		meson_fbc_hdr_reg[i].body_base = body_base;
		meson_fbc_hdr_reg[i].header_paddr = header_paddr;
		meson_fbc_hdr_reg[i].seq = ++meson_fbc_hdr_seq;
	}

	spin_unlock_irqrestore(&meson_fbc_hdr_lock, flags);
}
EXPORT_SYMBOL_GPL(meson_overlay_fbc_set_header);

/**
 * meson_overlay_fbc_get_header - look up the compress-header phys addr for
 *	an FBC-output body base.
 * @body_base: vb2 capture-buffer dma_addr (the contiguous am21c body base)
 *	to look up, as registered by meson_overlay_fbc_set_header().
 *
 * Returns the registered mmu_header_paddr[slot] for @body_base, or 0 if no
 * header is registered (caller then falls back to the legacy vb2-buffer-
 * based head computation -- see the VD1 AFBC path in
 * meson_overlay_atomic_update() for the canonical caller pattern). Exported
 * so meson_overlay2.c's VD2 AFBC path can consult the same registry
 * (hdmi-vpu-gap-review.md finding #2).
 */
dma_addr_t meson_overlay_fbc_get_header(dma_addr_t body_base)
{
	unsigned long flags;
	dma_addr_t header = 0;
	int i;

	spin_lock_irqsave(&meson_fbc_hdr_lock, flags);
	for (i = 0; i < MESON_FBC_HDR_REG_SIZE; i++) {
		if (meson_fbc_hdr_reg[i].body_base == body_base) {
			header = meson_fbc_hdr_reg[i].header_paddr;
			break;
		}
	}
	spin_unlock_irqrestore(&meson_fbc_hdr_lock, flags);

	return header;
}
EXPORT_SYMBOL_GPL(meson_overlay_fbc_get_header);

static int meson_overlay_atomic_check(struct drm_plane *plane,
				      struct drm_atomic_state *state)
{
	struct drm_plane_state *new_plane_state = drm_atomic_get_new_plane_state(state,
										 plane);
	struct drm_crtc_state *crtc_state;

	if (!new_plane_state->crtc)
		return 0;

	crtc_state = drm_atomic_get_crtc_state(state,
					       new_plane_state->crtc);
	if (IS_ERR(crtc_state))
		return PTR_ERR(crtc_state);

	return drm_atomic_helper_check_plane_state(new_plane_state,
						   crtc_state,
						   FRAC_16_16(1, 5),
						   FRAC_16_16(5, 1),
						   true, true);
}

/* Takes a fixed 16.16 number and converts it to integer. */
static inline int64_t fixed16_to_int(int64_t value)
{
	return value >> 16;
}

static const uint8_t skip_tab[6] = {
	0x24, 0x04, 0x68, 0x48, 0x28, 0x08,
};

static void meson_overlay_get_vertical_phase(unsigned int ratio_y, int *phase,
					     int *repeat, bool interlace)
{
	int offset_in = 0;
	int offset_out = 0;
	int repeat_skip = 0;

	if (!interlace && ratio_y > (1 << 18))
		offset_out = (1 * ratio_y) >> 10;

	while ((offset_in + (4 << 8)) <= offset_out) {
		repeat_skip++;
		offset_in += 4 << 8;
	}

	*phase = (offset_out - offset_in) >> 2;

	if (*phase > 0x100)
		repeat_skip++;

	*phase = *phase & 0xff;

	if (repeat_skip > 5)
		repeat_skip = 5;

	*repeat = skip_tab[repeat_skip];
}

static void meson_overlay_setup_scaler_params(struct meson_drm *priv,
					      struct drm_plane *plane,
					      bool interlace_mode)
{
	struct drm_crtc_state *crtc_state = priv->crtc->state;
	int video_top, video_left, video_width, video_height;
	struct drm_plane_state *state = plane->state;
	unsigned int vd_start_lines, vd_end_lines;
	unsigned int hd_start_lines, hd_end_lines;
	unsigned int crtc_height, crtc_width;
	unsigned int vsc_startp, vsc_endp;
	unsigned int hsc_startp, hsc_endp;
	unsigned int crop_top, crop_left;
	int vphase, vphase_repeat_skip;
	unsigned int ratio_x, ratio_y;
	int temp_height, temp_width;
	unsigned int w_in, h_in;
	int afbc_left, afbc_right;
	int afbc_top_src, afbc_bottom_src;
	int afbc_top, afbc_bottom;
	int temp, start, end;

	if (!crtc_state) {
		DRM_ERROR("Invalid crtc_state\n");
		return;
	}

	crtc_height = crtc_state->mode.vdisplay;
	crtc_width = crtc_state->mode.hdisplay;

	w_in = fixed16_to_int(state->src_w);
	h_in = fixed16_to_int(state->src_h);
	crop_top = fixed16_to_int(state->src_y);
	crop_left = fixed16_to_int(state->src_x);

	video_top = state->crtc_y;
	video_left = state->crtc_x;
	video_width = state->crtc_w;
	video_height = state->crtc_h;

	DRM_DEBUG("crtc_width %d crtc_height %d interlace %d\n",
		  crtc_width, crtc_height, interlace_mode);
	DRM_DEBUG("w_in %d h_in %d crop_top %d crop_left %d\n",
		  w_in, h_in, crop_top, crop_left);
	DRM_DEBUG("video top %d left %d width %d height %d\n",
		  video_top, video_left, video_width, video_height);

	ratio_x = (w_in << 18) / video_width;
	ratio_y = (h_in << 18) / video_height;

	if (ratio_x * video_width < (w_in << 18))
		ratio_x++;

	DRM_DEBUG("ratio x 0x%x y 0x%x\n", ratio_x, ratio_y);

	meson_overlay_get_vertical_phase(ratio_y, &vphase, &vphase_repeat_skip,
					 interlace_mode);

	DRM_DEBUG("vphase 0x%x skip %d\n", vphase, vphase_repeat_skip);

	/* Vertical */

	start = video_top + video_height / 2 - ((h_in << 17) / ratio_y);
	end = (h_in << 18) / ratio_y + start - 1;

	if (video_top < 0 && start < 0)
		vd_start_lines = (-(start) * ratio_y) >> 18;
	else if (start < video_top)
		vd_start_lines = ((video_top - start) * ratio_y) >> 18;
	else
		vd_start_lines = 0;

	if (video_top < 0)
		temp_height = min_t(unsigned int,
				    video_top + video_height - 1,
				    crtc_height - 1);
	else
		temp_height = min_t(unsigned int,
				    video_top + video_height - 1,
				    crtc_height - 1) - video_top + 1;

	temp = vd_start_lines + (temp_height * ratio_y >> 18);
	vd_end_lines = (temp <= (h_in - 1)) ? temp : (h_in - 1);

	vd_start_lines += crop_left;
	vd_end_lines += crop_left;

	/*
	 * TOFIX: Input frames are handled and scaled like progressive frames,
	 * proper handling of interlaced field input frames need to be figured
	 * out using the proper framebuffer flags set by userspace.
	 */
	if (interlace_mode) {
		start >>= 1;
		end >>= 1;
	}

	vsc_startp = max_t(int, start,
			   max_t(int, 0, video_top));
	vsc_endp = min_t(int, end,
			 min_t(int, crtc_height - 1,
			       video_top + video_height - 1));

	DRM_DEBUG("vsc startp %d endp %d start_lines %d end_lines %d\n",
		 vsc_startp, vsc_endp, vd_start_lines, vd_end_lines);

	afbc_top = round_down(vd_start_lines, 4);
	afbc_bottom = round_up(vd_end_lines + 1, 4);
	afbc_top_src = 0;
	afbc_bottom_src = round_up(h_in + 1, 4);

	DRM_DEBUG("afbc top %d (src %d) bottom %d (src %d)\n",
		  afbc_top, afbc_top_src, afbc_bottom, afbc_bottom_src);

	/* Horizontal */

	start = video_left + video_width / 2 - ((w_in << 17) / ratio_x);
	end = (w_in << 18) / ratio_x + start - 1;

	if (video_left < 0 && start < 0)
		hd_start_lines = (-(start) * ratio_x) >> 18;
	else if (start < video_left)
		hd_start_lines = ((video_left - start) * ratio_x) >> 18;
	else
		hd_start_lines = 0;

	if (video_left < 0)
		temp_width = min_t(unsigned int,
				   video_left + video_width - 1,
				   crtc_width - 1);
	else
		temp_width = min_t(unsigned int,
				   video_left + video_width - 1,
				   crtc_width - 1) - video_left + 1;

	temp = hd_start_lines + (temp_width * ratio_x >> 18);
	hd_end_lines = (temp <= (w_in - 1)) ? temp : (w_in - 1);

	priv->viu.vpp_line_in_length = hd_end_lines - hd_start_lines + 1;
	hsc_startp = max_t(int, start, max_t(int, 0, video_left));
	hsc_endp = min_t(int, end, min_t(int, crtc_width - 1,
					 video_left + video_width - 1));

	hd_start_lines += crop_top;
	hd_end_lines += crop_top;

	DRM_DEBUG("hsc startp %d endp %d start_lines %d end_lines %d\n",
		 hsc_startp, hsc_endp, hd_start_lines, hd_end_lines);

	if (hd_start_lines > 0 || (hd_end_lines < w_in)) {
		afbc_left = 0;
		afbc_right = round_up(w_in, 32);
	} else {
		afbc_left = round_down(hd_start_lines, 32);
		afbc_right = round_up(hd_end_lines + 1, 32);
	}

	DRM_DEBUG("afbc left %d right %d\n", afbc_left, afbc_right);

	priv->viu.vpp_vsc_start_phase_step = ratio_y << 6;

	priv->viu.vpp_vsc_ini_phase = vphase << 8;
	priv->viu.vpp_vsc_phase_ctrl = (1 << 13) | (4 << 8) |
				       vphase_repeat_skip;

	priv->viu.vd1_if0_luma_x0 = VD_X_START(hd_start_lines) |
				    VD_X_END(hd_end_lines);
	priv->viu.vd1_if0_chroma_x0 = VD_X_START(hd_start_lines >> 1) |
				      VD_X_END(hd_end_lines >> 1);

	priv->viu.viu_vd1_fmt_w =
			VD_H_WIDTH(hd_end_lines - hd_start_lines + 1) |
			VD_V_WIDTH(hd_end_lines/2 - hd_start_lines/2 + 1);

	priv->viu.vd1_afbc_vd_cfmt_w =
			AFBC_VD_H_WIDTH(afbc_right - afbc_left) |
			AFBC_VD_V_WIDTH(afbc_right / 2 - afbc_left / 2);

	priv->viu.vd1_afbc_vd_cfmt_h =
			AFBC_VD_HEIGHT((afbc_bottom - afbc_top) / 2);

	priv->viu.vd1_afbc_mif_hor_scope = AFBC_MIF_BLK_BGN_H(afbc_left / 32) |
				AFBC_MIF_BLK_END_H((afbc_right / 32) - 1);

	priv->viu.vd1_afbc_mif_ver_scope = AFBC_MIF_BLK_BGN_V(afbc_top / 4) |
				AFBC_MIF_BLK_END_H((afbc_bottom / 4) - 1);

	priv->viu.vd1_afbc_size_out =
			AFBC_HSIZE_OUT(afbc_right - afbc_left) |
			AFBC_VSIZE_OUT(afbc_bottom - afbc_top);

	priv->viu.vd1_afbc_pixel_hor_scope =
			AFBC_DEC_PIXEL_BGN_H(hd_start_lines - afbc_left) |
			AFBC_DEC_PIXEL_END_H(hd_end_lines - afbc_left);

	priv->viu.vd1_afbc_pixel_ver_scope =
			AFBC_DEC_PIXEL_BGN_V(vd_start_lines - afbc_top) |
			AFBC_DEC_PIXEL_END_V(vd_end_lines - afbc_top);

	priv->viu.vd1_afbc_size_in =
				AFBC_HSIZE_IN(afbc_right - afbc_left) |
				AFBC_VSIZE_IN(afbc_bottom_src - afbc_top_src);

	priv->viu.vd1_if0_luma_y0 = VD_Y_START(vd_start_lines) |
				    VD_Y_END(vd_end_lines);

	priv->viu.vd1_if0_chroma_y0 = VD_Y_START(vd_start_lines >> 1) |
				      VD_Y_END(vd_end_lines >> 1);

	priv->viu.vpp_pic_in_height = h_in;

	priv->viu.vpp_postblend_vd1_h_start_end = VD_H_START(hsc_startp) |
						  VD_H_END(hsc_endp);
	priv->viu.vpp_blend_vd2_h_start_end = VD_H_START(hd_start_lines) |
					      VD_H_END(hd_end_lines);
	priv->viu.vpp_hsc_region12_startp = VD_REGION13_END(0) |
					    VD_REGION24_START(hsc_startp);
	priv->viu.vpp_hsc_region34_startp =
				VD_REGION13_END(hsc_startp) |
				VD_REGION24_START(hsc_endp - hsc_startp);
	priv->viu.vpp_hsc_region4_endp = hsc_endp - hsc_startp;
	priv->viu.vpp_hsc_start_phase_step = ratio_x << 6;
	priv->viu.vpp_hsc_region1_phase_slope = 0;
	priv->viu.vpp_hsc_region3_phase_slope = 0;
	priv->viu.vpp_hsc_phase_ctrl = (1 << 21) | (4 << 16);

	priv->viu.vpp_line_in_length = hd_end_lines - hd_start_lines + 1;
	priv->viu.vpp_preblend_h_size = hd_end_lines - hd_start_lines + 1;

	priv->viu.vpp_postblend_vd1_v_start_end = VD_V_START(vsc_startp) |
						  VD_V_END(vsc_endp);
	priv->viu.vpp_blend_vd2_v_start_end =
				VD2_V_START((vd_end_lines + 1) >> 1) |
				VD2_V_END(vd_end_lines);

	priv->viu.vpp_vsc_region12_startp = 0;
	priv->viu.vpp_vsc_region34_startp =
				VD_REGION13_END(vsc_endp - vsc_startp) |
				VD_REGION24_START(vsc_endp - vsc_startp);
	priv->viu.vpp_vsc_region4_endp = vsc_endp - vsc_startp;
	priv->viu.vpp_vsc_start_phase_step = ratio_y << 6;
}

/*
 * HDR-002 Phase 2/3 bring-up: VD1 HDR2 in matrix-only mode (LUTs bypassed).
 *   0 = HDR2 off (upstream default)
 *   1 = identity MATRIXI/MATRIXO -> visually TRANSPARENT (Phase 2 de-risk; md5 a
 *       captured frame with param 0 vs 1, must match)
 *   2 = Cb/Cr-swap MATRIXO -> a deterministic, easily-verified colour swap
 *       (Phase 3: validates the matrix coefficient packing + enable path)
 *   3 = real BT.2020-YCbCr->RGB (MATRIXI) + RGB->BT.709-YCbCr (MATRIXO) CSC
 *       coefficients from the BSP (Phase 3: the matrices the full HDR path uses)
 * The full HDR pipeline (GMUT gamut matrix + EOTF/OOTF/OETF LUTs) is Phase 4 --
 * it needs the BSP LUT tables and the non-mtx_only dataflow.
 */
static int hdr2_bypass;
module_param(hdr2_bypass, int, 0644);
MODULE_PARM_DESC(hdr2_bypass,
		 "VD1 HDR2 matrix-only mode: 0=off 1=identity 2=CbCr-swap 3=BT.2020->709 CSC");

/* 3x3 matrices, row-major, 1.0 = 1024 (13-bit signed); from BSP amvecm set_hdr2_v0.c */
static const int hdr2_mtx_identity[9]    = {1024, 0, 0,  0, 1024, 0,  0, 0, 1024};
static const int hdr2_mtxo_cbcr_swap[9]  = {1024, 0, 0,  0, 0, 1024,  0, 1024, 0};
static const int hdr2_mtxi_yuv2020_rgb[9] = {1196, 0, 1724,  1196, -193, -668,  1196, 2200, 0};
static const int hdr2_mtxo_rgb_yuv709[9]  = {186, 627, 63,  -103, -345, 448,  448, -407, -41};
/* MATRIXO that halves channel 0 (Y, since MATRIXI=identity feeds YUV as "RGB"):
 * a predictable transform -- output luma -> 0.5*input luma, chroma unchanged --
 * for an unambiguous matrix-packing check (captured mean luma must halve). */
static const int hdr2_mtxo_luma_half[9] = {512, 0, 0,  0, 1024, 0,  0, 0, 1024};
static const int hdr2_off_zero[3]  = {0, 0, 0};
static const int hdr2_ipre_2020[3] = {-64, -512, -512}; /* MATRIXI pre-offset (limited-range in) */
static const int hdr2_opost_709[3] = {64, 512, 512};    /* MATRIXO post-offset (limited-range out) */

/* Program one HDR2 3x3 matrix: coef0_reg..+4, pre/post offset regs, enable reg. */
static void meson_hdr2_set_matrix(struct meson_drm *priv, unsigned int coef0_reg,
				  unsigned int preoff_reg, unsigned int postoff_reg,
				  unsigned int en_reg, const int c[9],
				  const int preoff[3], const int postoff[3])
{
	writel_relaxed(((c[0] & 0x1fff) << 16) | (c[1] & 0x1fff), priv->io_base + _REG(coef0_reg));
	writel_relaxed(((c[2] & 0x1fff) << 16) | (c[3] & 0x1fff), priv->io_base + _REG(coef0_reg + 1));
	writel_relaxed(((c[4] & 0x1fff) << 16) | (c[5] & 0x1fff), priv->io_base + _REG(coef0_reg + 2));
	writel_relaxed(((c[6] & 0x1fff) << 16) | (c[7] & 0x1fff), priv->io_base + _REG(coef0_reg + 3));
	writel_relaxed(c[8] & 0x1fff, priv->io_base + _REG(coef0_reg + 4));
	writel_relaxed(((preoff[0] & 0xfff) << 16) | (preoff[1] & 0xfff), priv->io_base + _REG(preoff_reg));
	writel_relaxed(preoff[2] & 0xfff, priv->io_base + _REG(preoff_reg + 1));
	writel_relaxed(((postoff[0] & 0xfff) << 16) | (postoff[1] & 0xfff), priv->io_base + _REG(postoff_reg));
	writel_relaxed(postoff[2] & 0xfff, priv->io_base + _REG(postoff_reg + 1));
	writel_relaxed(1, priv->io_base + _REG(en_reg));
}

/*
 * #13 fix: frame-dependent HDR2 input size, stashed by
 * meson_overlay_atomic_update() (src_w/h are only known at plane-commit
 * time) and consumed by meson_overlay_pq_flush() at the next vsync -- see
 * that function's comment for the full deferral story.
 */
static u32 meson_overlay_pq_hdr2_in_w, meson_overlay_pq_hdr2_in_h;

static void meson_overlay_hdr2_program(struct meson_drm *priv, u32 in_w, u32 in_h)
{
	const int *mtxi = hdr2_mtx_identity, *mtxo = hdr2_mtx_identity;
	const int *ipre = hdr2_off_zero, *opost = hdr2_off_zero;

	if (hdr2_bypass <= 0) {
		/* disable: master enable off, input size 0 (VD1 bypasses HDR2), gate clk */
		writel_relaxed(0, priv->io_base + _REG(VD1_HDR2_CTRL));
		writel_relaxed(0, priv->io_base + _REG(VPP_VD1_HDR_IN_SIZE));
		writel_relaxed(0, priv->io_base + _REG(VD1_HDR2_CLK_GATE));
		return;
	}

	switch (hdr2_bypass) {
	case 2:
		mtxo = hdr2_mtxo_cbcr_swap;
		break;
	case 3:
		mtxi = hdr2_mtxi_yuv2020_rgb; ipre = hdr2_ipre_2020;
		mtxo = hdr2_mtxo_rgb_yuv709;  opost = hdr2_opost_709;
		break;
	case 4:
		mtxo = hdr2_mtxo_luma_half;   /* Y -> 0.5*Y (packing check) */
		break;
	default: /* 1: identity */
		break;
	}

	/* force the HDR2 clock on before touching the core */
	writel_relaxed(HDR2_CLK_GATE_FORCE_ON, priv->io_base + _REG(VD1_HDR2_CLK_GATE));

	/* HDR2 input size = (height << 16) | width -- gates the block into the VD1
	 * datapath (a zero size leaves VD1 bypassing HDR2). */
	writel_relaxed((in_h << 16) | (in_w & 0xffff),
		       priv->io_base + _REG(VPP_VD1_HDR_IN_SIZE));

	meson_hdr2_set_matrix(priv, VD1_HDR2_MATRIXI_COEF00_01,
			      VD1_HDR2_MATRIXI_PRE_OFFSET0_1, VD1_HDR2_MATRIXI_OFFSET0_1,
			      VD1_HDR2_MATRIXI_EN_CTRL, mtxi, ipre, hdr2_off_zero);
	meson_hdr2_set_matrix(priv, VD1_HDR2_MATRIXO_COEF00_01,
			      VD1_HDR2_MATRIXO_PRE_OFFSET0_1, VD1_HDR2_MATRIXO_OFFSET0_1,
			      VD1_HDR2_MATRIXO_EN_CTRL, mtxo, hdr2_off_zero, opost);

	/* enable the core in matrix-only mode (LUTs bypassed); latch matrices valid */
	writel_relaxed(HDR2_CTRL_TOP_EN | HDR2_CTRL_ONLY_MAT | HDR2_CTRL_IN_MTX_EN |
		       HDR2_CTRL_IN_MTX_VALID | HDR2_CTRL_OUT_MTX_VALID,
		       priv->io_base + _REG(VD1_HDR2_CTRL));
}

/* PQ-001: VADJ1 post-blend brightness + contrast. The upstream driver only
 * enables the minus-black-level bit; the brightness/contrast block itself is
 * never programmed. Expose brightness + contrast and wire them into the G12A+
 * VADJ1 register bank (this driver only runs on G12A/G12B/SM1; the legacy 0x1d4x
 * VADJ1 bank is inert on these SoCs).
 *
 * VPP_VADJ1_Y_2 (0x3282): bits[18:8] = brightness (11-bit signed, no halving),
 * bits[7:0] = contrast (NEUTRAL 0x80, range 0..255). VPP_VADJ1_MA_MB_2 (0x3283) +
 * MC_MD_2 (0x3284): the saturation 2x2 chroma matrix (hue=0 here: ma=md=chroma
 * gain, mb=mc=0; neutral 256 = 1.0). Block enable lives in VPP_VADJ1_MISC (0x3280)
 * bit 0. Validated on SM1 by dongle capture: brightness shifts mean-Y (123.7 ->
 * 183.3 at +400), contrast shifts luma std-dev (44.8 -> 50.3 / 23.8), saturation
 * scales captured chroma (143.5 neutral -> 75.7 at 0.5x / 155.8 at 1.5x). The
 * register map + the saturation gain clamp were pinned down with live devmem
 * before wiring. Applied from the overlay atomic_update path (engages while VD1 is
 * active -- bring-up scaffolding; post-blend PQ ultimately belongs in the CRTC path).
 */
static int pq_brightness;
module_param(pq_brightness, int, 0644);
MODULE_PARM_DESC(pq_brightness,
		 "VADJ1 post-blend brightness offset, signed ~-1024..1023 (0 = neutral)");

static int pq_contrast = 0x80;
module_param(pq_contrast, int, 0644);
MODULE_PARM_DESC(pq_contrast,
		 "VADJ1 post-blend contrast, 0..255 (0x80 = neutral)");

static int pq_saturation;
module_param(pq_saturation, int, 0644);
MODULE_PARM_DESC(pq_saturation,
		 "VADJ1 post-blend saturation, signed -128..127 (0 = neutral; chroma gain capped at 1.5x to avoid the HW overflow-to-black above that)");

/* VADJ2: second post-blend video-adjust stage (GXL/GXM legacy bank). */
static int pq2_brightness;
module_param(pq2_brightness, int, 0644);
MODULE_PARM_DESC(pq2_brightness,
		 "VADJ2 post-blend brightness offset, signed ~-1024..1023 (0 = neutral)");

static int pq2_contrast = 0x80;
module_param(pq2_contrast, int, 0644);
MODULE_PARM_DESC(pq2_contrast,
		 "VADJ2 post-blend contrast, 0..255 (0x80 = neutral)");

static int pq2_saturation;
module_param(pq2_saturation, int, 0644);
MODULE_PARM_DESC(pq2_saturation,
		 "VADJ2 post-blend saturation, signed -128..127 (0 = neutral; same 1.5x chroma-gain cap as VADJ1)");

/* Black-level extension (dark-detail stretch), default-off. */
static int pq_blackext_start = -1;
module_param(pq_blackext_start, int, 0644);
MODULE_PARM_DESC(pq_blackext_start,
		 "Black-extension start threshold 0..255 (-1 = disabled)");

static int pq_blackext_slope;
module_param(pq_blackext_slope, int, 0644);
MODULE_PARM_DESC(pq_blackext_slope,
		 "Black-extension slope/gain 0..255");

/* VPP_GAINOFF white-balance: per-channel RGB gain (GXL/GXM legacy bank). */
static int pq_wb_rgain = 1024;
module_param(pq_wb_rgain, int, 0644);
MODULE_PARM_DESC(pq_wb_rgain,
		 "White-balance R gain 0..2047 (1024 = unity); GXL/GXM");

static int pq_wb_ggain = 1024;
module_param(pq_wb_ggain, int, 0644);
MODULE_PARM_DESC(pq_wb_ggain,
		 "White-balance G gain 0..2047 (1024 = unity); GXL/GXM");

static int pq_wb_bgain = 1024;
module_param(pq_wb_bgain, int, 0644);
MODULE_PARM_DESC(pq_wb_bgain,
		 "White-balance B gain 0..2047 (1024 = unity); GXL/GXM");

static void meson_overlay_vadj1_program(struct meson_drm *priv)
{
	bool g12a = meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A);
	int bri = clamp_t(int, pq_brightness, -1024, 1023);
	u32 contrast = clamp_t(int, pq_contrast, 0, 255);
	int sat = clamp_t(int, pq_saturation, -128, 127);
	/*
	 * Chroma gain ma = 256*(sat+128) >> 7 (256 = 1.0 at sat=0). Clamp to 384
	 * (~1.5x): the VADJ matrix-gain datapath overflows to black above that
	 * (RE 2026-06-07 devmem sweep on SM1). hue=0 -> md=ma, mb=mc=0.
	 */
	u32 ma = clamp_t(int, (256 * (sat + 128)) >> 7, 0, 384);
	bool neutral = (!bri && contrast == 0x80 && ma == 256);

	if (g12a) {
		/* G12A+ vadj1 _2 bank (0x328x), enable in VPP_VADJ1_MISC. */
		u32 misc = readl_relaxed(priv->io_base + _REG(VPP_VADJ1_MISC));

		/* Neutral (brightness 0, contrast at the 0x80 midpoint, saturation
		 * gain 1.0): write neutral values and drop our enable bit. RMW the
		 * MISC enable to preserve the register's other bits. */
		if (neutral) {
			writel_relaxed(0x80, priv->io_base + _REG(VPP_VADJ1_Y_2));
			writel_relaxed(256 << 16, priv->io_base + _REG(VPP_VADJ1_MA_MB_2));
			writel_relaxed(256, priv->io_base + _REG(VPP_VADJ1_MC_MD_2));
			writel_relaxed(misc & ~VPP_VADJ1_MISC_VADJ_EN,
				       priv->io_base + _REG(VPP_VADJ1_MISC));
			return;
		}

		/* bits[18:8] = brightness (11-bit, no halving), bits[7:0] = contrast */
		writel_relaxed(((bri & 0x7ff) << 8) | contrast,
			       priv->io_base + _REG(VPP_VADJ1_Y_2));
		/* saturation chroma matrix (hue=0): MA_MB = ma<<16 (mb=0), MC_MD = ma (md) */
		writel_relaxed((ma & 0x3ff) << 16, priv->io_base + _REG(VPP_VADJ1_MA_MB_2));
		writel_relaxed(ma & 0x3ff, priv->io_base + _REG(VPP_VADJ1_MC_MD_2));
		writel_relaxed(misc | VPP_VADJ1_MISC_VADJ_EN,
			       priv->io_base + _REG(VPP_VADJ1_MISC));
	} else {
		/*
		 * GXL/GXM legacy vadj1 bank (0x1d4x). Enable is VPP_VADJ_CTRL bit0
		 * (the pre-G12A path; VPP_VADJ1_MISC is inert here). The brightness
		 * field in VPP_VADJ1_Y is the narrower [17:8] 10-bit form, so halve
		 * the input (the >>1 the BSP applies on the pre-G12A path) to match
		 * the same physical range as the G12A _2 11-bit field. Contrast [7:0]
		 * and the MA_MB/MC_MD saturation matrix keep the same encoding.
		 * Preserve VPP_MINUS_BLACK_LVL_VADJ1_ENABLE (bit1) in the ctrl RMW.
		 */
		u32 ctrl = readl_relaxed(priv->io_base + _REG(VPP_VADJ_CTRL));

		if (neutral) {
			writel_relaxed(0x80, priv->io_base + _REG(VPP_VADJ1_Y));
			writel_relaxed(256 << 16, priv->io_base + _REG(VPP_VADJ1_MA_MB));
			writel_relaxed(256, priv->io_base + _REG(VPP_VADJ1_MC_MD));
			writel_relaxed(ctrl & ~VPP_VADJ1_EN,
				       priv->io_base + _REG(VPP_VADJ_CTRL));
			return;
		}

		/* bits[17:8] = brightness (10-bit, halved), bits[7:0] = contrast */
		writel_relaxed((((bri >> 1) & 0x3ff) << 8) | contrast,
			       priv->io_base + _REG(VPP_VADJ1_Y));
		writel_relaxed((ma & 0x3ff) << 16, priv->io_base + _REG(VPP_VADJ1_MA_MB));
		writel_relaxed(ma & 0x3ff, priv->io_base + _REG(VPP_VADJ1_MC_MD));
		writel_relaxed(ctrl | VPP_VADJ1_EN,
			       priv->io_base + _REG(VPP_VADJ_CTRL));
	}
}

/*
 * VADJ2: the second post-blend video-adjust stage. GXL/GXM legacy bank
 * (VPP_VADJ2_*, 0x1d44-46), enable VPP_VADJ_CTRL bit2 -- identical field
 * encoding to VADJ1 (brightness [17:8] halved, contrast [7:0], 2x2 chroma
 * MA_MB/MC_MD). G12A+ uses a separate _2 bank (VPP_VADJ2_MISC) not wired
 * here, so VADJ2 is GXL/GXM-only for now (returns early on G12A+). A second
 * independent stage lets userspace split adjustments (e.g. coarse VADJ1 +
 * fine VADJ2, or brightness on one stage and saturation on the other).
 */
static void meson_overlay_vadj2_program(struct meson_drm *priv)
{
	int bri = clamp_t(int, pq2_brightness, -1024, 1023);
	u32 contrast = clamp_t(int, pq2_contrast, 0, 255);
	int sat = clamp_t(int, pq2_saturation, -128, 127);
	u32 ma = clamp_t(int, (256 * (sat + 128)) >> 7, 0, 384);
	bool neutral = (!bri && contrast == 0x80 && ma == 256);
	u32 ctrl;

	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A))
		return;

	ctrl = readl_relaxed(priv->io_base + _REG(VPP_VADJ_CTRL));

	if (neutral) {
		writel_relaxed(0x80, priv->io_base + _REG(VPP_VADJ2_Y));
		writel_relaxed(256 << 16, priv->io_base + _REG(VPP_VADJ2_MA_MB));
		writel_relaxed(256, priv->io_base + _REG(VPP_VADJ2_MC_MD));
		writel_relaxed(ctrl & ~VPP_VADJ2_EN,
			       priv->io_base + _REG(VPP_VADJ_CTRL));
		return;
	}

	/* bits[17:8] = brightness (10-bit, halved like VADJ1); bits[7:0] = contrast */
	writel_relaxed((((bri >> 1) & 0x3ff) << 8) | contrast,
		       priv->io_base + _REG(VPP_VADJ2_Y));
	writel_relaxed((ma & 0x3ff) << 16, priv->io_base + _REG(VPP_VADJ2_MA_MB));
	writel_relaxed(ma & 0x3ff, priv->io_base + _REG(VPP_VADJ2_MC_MD));
	writel_relaxed(ctrl | VPP_VADJ2_EN,
		       priv->io_base + _REG(VPP_VADJ_CTRL));
}

/*
 * Black-level extension: stretches dark detail below a threshold for more
 * shadow contrast. VPP_BLACKEXT_CTRL packs start[31:24] + slope[23:16]; the
 * block enable is VPP_VE_ENABLE_CTRL bit3 (RMW to preserve the other VE-enable
 * bits, e.g. DNLP at bit2). Default-off (pq_blackext_start < 0) so it never
 * engages unless explicitly set. The VE block is the classic pre-G12A
 * enhancement path -- GXL/GXM-only (G12A+ has no equivalent
 * VPP_VE_ENABLE_CTRL/VPP_BLACKEXT_CTRL block wired here): return early there,
 * mirroring meson_overlay_vadj2_program()/meson_overlay_wb_program() below.
 */
static void meson_overlay_blackext_program(struct meson_drm *priv)
{
	u32 ve;

	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A))
		return;

	ve = readl_relaxed(priv->io_base + _REG(VPP_VE_ENABLE_CTRL));

	if (pq_blackext_start < 0) {
		writel_relaxed(ve & ~VPP_VE_BEXT_EN,
			       priv->io_base + _REG(VPP_VE_ENABLE_CTRL));
		return;
	}

	writel_relaxed((clamp_t(int, pq_blackext_start, 0, 255) << 24) |
		       (clamp_t(int, pq_blackext_slope, 0, 255) << 16),
		       priv->io_base + _REG(VPP_BLACKEXT_CTRL));
	writel_relaxed(ve | VPP_VE_BEXT_EN,
		       priv->io_base + _REG(VPP_VE_ENABLE_CTRL));
}

/*
 * VPP_GAINOFF white-balance: per-channel RGB gain on the post-blend RGB (the
 * BSP "txl-and-before" GAINOFF path; the WB sink on GXL since
 * video_rgb_ogo_xvy_mtx defaults off). CTRL0 packs en[31] + r_gain[26:16] +
 * g_gain[10:0]; CTRL1 b_gain[26:16] + r_post_offset[10:0]; CTRL2-4 hold the
 * g/b post-offset and r/g/b pre-offset, all left 0 -- we expose only the gains.
 * Gains are 11-bit, 1024 = unity; the block is enabled (bit31) only when a gain
 * is non-unity, otherwise bypassed. GXL/GXM-only (G12A+ uses a different WB
 * bank): return early there. Mainline never programs GAINOFF, so the bypass we
 * write when neutral is the existing identity state.
 */
static void meson_overlay_wb_program(struct meson_drm *priv)
{
	u32 rg = clamp_t(int, pq_wb_rgain, 0, 2047);
	u32 gg = clamp_t(int, pq_wb_ggain, 0, 2047);
	u32 bg = clamp_t(int, pq_wb_bgain, 0, 2047);
	bool en = (rg != 1024 || gg != 1024 || bg != 1024);

	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A))
		return;

	writel_relaxed(((en ? 1u : 0u) << 31) |
		       ((rg & 0x7ff) << 16) | (gg & 0x7ff),
		       priv->io_base + _REG(VPP_GAINOFF_CTRL0));
	writel_relaxed((bg & 0x7ff) << 16,
		       priv->io_base + _REG(VPP_GAINOFF_CTRL1));
	writel_relaxed(0, priv->io_base + _REG(VPP_GAINOFF_CTRL2));
	writel_relaxed(0, priv->io_base + _REG(VPP_GAINOFF_CTRL3));
	writel_relaxed(0, priv->io_base + _REG(VPP_GAINOFF_CTRL4));
}

/*
 * #13 fix: single deferred-flush point for every post-blend PQ/HDR2 block in
 * this file. Previously each ran synchronously from
 * meson_overlay_atomic_update() (direct MMIO in the commit tail) -- the same
 * "not RDMA/vsync-deferred" defect class as the colour-management finding in
 * meson_crtc.c (#8). Register writes here are cheap RMWs against small
 * fixed blocks (no per-frame buffer geometry), so calling this
 * unconditionally once per vsync is the same write volume the old
 * per-VD1-commit call site already produced -- just correctly timed.
 *
 * Called from meson_crtc.c's per-vsync commit-tail UNCONDITIONALLY --
 * deliberately NOT gated on vd1_enabled/vd1_commit, because VADJ1/VADJ2/
 * blackext/WB are post-blend and must keep working on an OSD-only desktop
 * (only the HDR2 call inside is VD1-specific, and it already self-gates via
 * hdr2_bypass<=0). This is what makes the pq_* and hdr2_bypass sysfs
 * writes take effect with no video plane active -- previously nothing in this file's
 * call graph ran at all in that case, since this file is only the VD1
 * plane's ops.
 *
 * Residual gap: a compositor that is fully idle (no commit of ANY kind, not
 * even OSD) still won't see a sysfs write applied until the next commit
 * happens to occur. Forcing an immediate out-of-band flush from the sysfs
 * .set path would need a stored `struct meson_drm *` singleton plus a way
 * to kick a vsync/commit with nothing pending -- flagged, not implemented.
 */
void meson_overlay_pq_flush(struct meson_drm *priv)
{
	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A))
		meson_overlay_hdr2_program(priv, meson_overlay_pq_hdr2_in_w,
					   meson_overlay_pq_hdr2_in_h);
	meson_overlay_vadj1_program(priv);
	meson_overlay_vadj2_program(priv);
	meson_overlay_blackext_program(priv);
	meson_overlay_wb_program(priv);
}

static void meson_overlay_atomic_update(struct drm_plane *plane,
					struct drm_atomic_state *state)
{
	struct meson_overlay *meson_overlay = to_meson_overlay(plane);
	struct drm_plane_state *new_state = drm_atomic_get_new_plane_state(state,
									   plane);
	struct drm_framebuffer *fb = new_state->fb;
	struct meson_drm *priv = meson_overlay->priv;
	struct drm_gem_dma_object *gem;
	unsigned long flags;
	bool interlace_mode;

	DRM_DEBUG_DRIVER("\n");

	interlace_mode = new_state->crtc->mode.flags & DRM_MODE_FLAG_INTERLACE;

	/* Phase 2 stage 2: stash zpos for the VPP post-blend port assignment */
	priv->viu.vd1_zpos = new_state->zpos;

	spin_lock_irqsave(&priv->drm->event_lock, flags);

	if ((fb->modifier & DRM_FORMAT_MOD_AMLOGIC_FBC(0, 0)) ==
			    DRM_FORMAT_MOD_AMLOGIC_FBC(0, 0)) {
		priv->viu.vd1_afbc = true;

		priv->viu.vd1_afbc_mode = AFBC_MIF_URGENT(3) |
					  AFBC_HOLD_LINE_NUM(8) |
					  AFBC_BURST_LEN(2);

		if (fb->modifier & DRM_FORMAT_MOD_AMLOGIC_FBC(0,
						AMLOGIC_FBC_OPTION_MEM_SAVING))
			priv->viu.vd1_afbc_mode |= AFBC_BLK_MEM_MODE;

		if ((fb->modifier & __fourcc_mod_amlogic_layout_mask) ==
				AMLOGIC_FBC_LAYOUT_SCATTER)
			priv->viu.vd1_afbc_mode |= AFBC_SCATTER_MODE;

		priv->viu.vd1_afbc_en = 0x1600 | AFBC_DEC_ENABLE;

		priv->viu.vd1_afbc_conv_ctrl = AFBC_CONV_LBUF_LEN(256);

		priv->viu.vd1_afbc_dec_def_color = AFBC_DEF_COLOR_Y(1023);

		/* 420: horizontal / 2, vertical / 4 */
		priv->viu.vd1_afbc_vd_cfmt_ctrl = AFBC_HORZ_RPT_PIXEL0 |
						  AFBC_HORZ_Y_C_RATIO(1) |
						  AFBC_HORZ_FMT_EN |
						  AFBC_VERT_RPT_LINE0 |
						  AFBC_VERT_INITIAL_PHASE(12) |
						  AFBC_VERT_PHASE_STEP(8) |
						  AFBC_VERT_FMT_EN;

		switch (fb->format->format) {
		/* AFBC Only formats */
		case DRM_FORMAT_YUV420_10BIT:
			priv->viu.vd1_afbc_mode |=
				AFBC_COMPBITS_YUV(AFBC_COMPBITS_10BIT);
			priv->viu.vd1_afbc_dec_def_color |=
					AFBC_DEF_COLOR_U(512) |
					AFBC_DEF_COLOR_V(512);
			break;
		case DRM_FORMAT_YUV420_8BIT:
			priv->viu.vd1_afbc_dec_def_color |=
					AFBC_DEF_COLOR_U(128) |
					AFBC_DEF_COLOR_V(128);
			break;
		}

		priv->viu.vd1_if0_gen_reg = 0;
		priv->viu.vd1_if0_canvas0 = 0;
		priv->viu.viu_vd1_fmt_ctrl = 0;
	} else {
		priv->viu.vd1_afbc = false;

		priv->viu.vd1_if0_gen_reg = VD_URGENT_CHROMA |
					    VD_URGENT_LUMA |
					    VD_HOLD_LINES(9) |
					    VD_CHRO_RPT_LASTL_CTRL |
					    VD_ENABLE;
	}

	/* Setup scaler params */
	meson_overlay_setup_scaler_params(priv, plane, interlace_mode);

	priv->viu.vd1_if0_repeat_loop = 0;
	priv->viu.vd1_if0_luma0_rpt_pat = interlace_mode ? 8 : 0;
	priv->viu.vd1_if0_chroma0_rpt_pat = interlace_mode ? 8 : 0;
	priv->viu.vd1_range_map_y = 0;
	priv->viu.vd1_range_map_cb = 0;
	priv->viu.vd1_range_map_cr = 0;

	/* Default values for RGB888/YUV444 */
	priv->viu.vd1_if0_gen_reg2 = 0;
	priv->viu.viu_vd1_fmt_ctrl = 0;

	/* None will match for AFBC Only formats */
	switch (fb->format->format) {
	case DRM_FORMAT_RGB888:
		priv->viu.vd1_if0_gen_reg |= VD_DEMUX_MODE_RGB |
					     VD_BYTES_PER_PIXEL(2) |
					     VD_LITTLE_ENDIAN;
		priv->viu.vd1_if0_canvas0 =
					CANVAS_ADDR2(priv->canvas_id_vd1_0) |
					CANVAS_ADDR1(priv->canvas_id_vd1_0) |
					CANVAS_ADDR0(priv->canvas_id_vd1_0);
		break;
	case DRM_FORMAT_YUYV:
		priv->viu.vd1_if0_gen_reg |= VD_BYTES_PER_PIXEL(1);
		priv->viu.vd1_if0_canvas0 =
					CANVAS_ADDR2(priv->canvas_id_vd1_0) |
					CANVAS_ADDR1(priv->canvas_id_vd1_0) |
					CANVAS_ADDR0(priv->canvas_id_vd1_0);
		priv->viu.viu_vd1_fmt_ctrl = VD_HORZ_Y_C_RATIO(1) | /* /2 */
					     VD_HORZ_FMT_EN |
					     VD_VERT_RPT_LINE0 |
					     VD_VERT_INITIAL_PHASE(12) |
					     VD_VERT_PHASE_STEP(16) | /* /2 */
					     VD_VERT_FMT_EN;
		break;
	case DRM_FORMAT_NV12:
	case DRM_FORMAT_NV21:
		priv->viu.vd1_if0_gen_reg |= VD_SEPARATE_EN;
		priv->viu.vd1_if0_canvas0 =
					CANVAS_ADDR2(priv->canvas_id_vd1_1) |
					CANVAS_ADDR1(priv->canvas_id_vd1_1) |
					CANVAS_ADDR0(priv->canvas_id_vd1_0);
		if (fb->format->format == DRM_FORMAT_NV12)
			priv->viu.vd1_if0_gen_reg2 = VD_COLOR_MAP(1);
		else
			priv->viu.vd1_if0_gen_reg2 = VD_COLOR_MAP(2);
		priv->viu.viu_vd1_fmt_ctrl = VD_HORZ_Y_C_RATIO(1) | /* /2 */
					     VD_HORZ_FMT_EN |
					     VD_VERT_RPT_LINE0 |
					     VD_VERT_INITIAL_PHASE(12) |
					     VD_VERT_PHASE_STEP(8) | /* /4 */
					     VD_VERT_FMT_EN;
		break;
	case DRM_FORMAT_YUV444:
	case DRM_FORMAT_YUV422:
	case DRM_FORMAT_YUV420:
	case DRM_FORMAT_YUV411:
	case DRM_FORMAT_YUV410:
		priv->viu.vd1_if0_gen_reg |= VD_SEPARATE_EN;
		priv->viu.vd1_if0_canvas0 =
					CANVAS_ADDR2(priv->canvas_id_vd1_2) |
					CANVAS_ADDR1(priv->canvas_id_vd1_1) |
					CANVAS_ADDR0(priv->canvas_id_vd1_0);
		switch (fb->format->format) {
		case DRM_FORMAT_YUV422:
			priv->viu.viu_vd1_fmt_ctrl =
					VD_HORZ_Y_C_RATIO(1) | /* /2 */
					VD_HORZ_FMT_EN |
					VD_VERT_RPT_LINE0 |
					VD_VERT_INITIAL_PHASE(12) |
					VD_VERT_PHASE_STEP(16) | /* /2 */
					VD_VERT_FMT_EN;
			break;
		case DRM_FORMAT_YUV420:
			priv->viu.viu_vd1_fmt_ctrl =
					VD_HORZ_Y_C_RATIO(1) | /* /2 */
					VD_HORZ_FMT_EN |
					VD_VERT_RPT_LINE0 |
					VD_VERT_INITIAL_PHASE(12) |
					VD_VERT_PHASE_STEP(8) | /* /4 */
					VD_VERT_FMT_EN;
			break;
		case DRM_FORMAT_YUV411:
			priv->viu.viu_vd1_fmt_ctrl =
					VD_HORZ_Y_C_RATIO(2) | /* /4 */
					VD_HORZ_FMT_EN |
					VD_VERT_RPT_LINE0 |
					VD_VERT_INITIAL_PHASE(12) |
					VD_VERT_PHASE_STEP(16) | /* /2 */
					VD_VERT_FMT_EN;
			break;
		case DRM_FORMAT_YUV410:
			priv->viu.viu_vd1_fmt_ctrl =
					VD_HORZ_Y_C_RATIO(2) | /* /4 */
					VD_HORZ_FMT_EN |
					VD_VERT_RPT_LINE0 |
					VD_VERT_INITIAL_PHASE(12) |
					VD_VERT_PHASE_STEP(8) | /* /4 */
					VD_VERT_FMT_EN;
			break;
		}
		break;
	}

	/*
	 * Phase 3: interlaced field input -- override the VFORMATTER vertical
	 * init-phase per field (top=0xe, bottom=0xa). Progressive keeps the
	 * chroma-siting default (0xc) the format switch just set, so the common
	 * case is byte-identical (no regression).
	 */
	if (to_meson_overlay_state(new_state)->field != MESON_FIELD_PROGRESSIVE &&
	    (priv->viu.viu_vd1_fmt_ctrl & VD_VERT_FMT_EN)) {
		u32 ph = (to_meson_overlay_state(new_state)->field ==
			  MESON_FIELD_TOP) ? 0xe : 0xa;

		priv->viu.viu_vd1_fmt_ctrl &= ~VD_VERT_INITIAL_PHASE(0xf);
		priv->viu.viu_vd1_fmt_ctrl |= VD_VERT_INITIAL_PHASE(ph);
	}

	/* Update Canvas with buffer address */
	priv->viu.vd1_planes = fb->format->num_planes;

	switch (priv->viu.vd1_planes) {
	case 3:
		gem = drm_fb_dma_get_gem_obj(fb, 2);
		priv->viu.vd1_addr2 = gem->dma_addr + fb->offsets[2];
		priv->viu.vd1_stride2 = fb->pitches[2];
		priv->viu.vd1_height2 =
			drm_format_info_plane_height(fb->format,
						fb->height, 2);
		DRM_DEBUG("plane 2 addr 0x%x stride %d height %d\n",
			 priv->viu.vd1_addr2,
			 priv->viu.vd1_stride2,
			 priv->viu.vd1_height2);
		fallthrough;
	case 2:
		gem = drm_fb_dma_get_gem_obj(fb, 1);
		priv->viu.vd1_addr1 = gem->dma_addr + fb->offsets[1];
		priv->viu.vd1_stride1 = fb->pitches[1];
		priv->viu.vd1_height1 =
			drm_format_info_plane_height(fb->format,
						fb->height, 1);
		DRM_DEBUG("plane 1 addr 0x%x stride %d height %d\n",
			 priv->viu.vd1_addr1,
			 priv->viu.vd1_stride1,
			 priv->viu.vd1_height1);
		fallthrough;
	case 1:
		gem = drm_fb_dma_get_gem_obj(fb, 0);
		priv->viu.vd1_addr0 = gem->dma_addr + fb->offsets[0];
		priv->viu.vd1_stride0 = fb->pitches[0];
		priv->viu.vd1_height0 =
			drm_format_info_plane_height(fb->format,
						     fb->height, 0);
		DRM_DEBUG("plane 0 addr 0x%x stride %d height %d\n",
			 priv->viu.vd1_addr0,
			 priv->viu.vd1_stride0,
			 priv->viu.vd1_height0);
	}

	if (priv->viu.vd1_afbc) {
		dma_addr_t fbc_header;

		if (priv->viu.vd1_afbc_mode & AFBC_SCATTER_MODE) {
			/*
			 * Scatter mode STILL needs the body base address. The
			 * BSP (video_hw.c set_layer_display_canvas) writes BOTH
			 * AFBC_HEAD_BADDR=compHeadAddr>>4 and
			 * AFBC_BODY_BADDR=compBodyAddr>>4 for every compressed
			 * frame -- no scatter exception. Zeroing body_addr made
			 * the AFBC decoder fetch body superblocks from address 0
			 * -> garbage on SM1 (10-bit AMSA, confirmed by HDMI
			 * loopback-dongle capture 2026-06-12, 7.64 dB).
			 *
			 * Layout: [body @ offsets[0]][header @ +body_size].
			 * body_size from the frame WIDTH (the FB pitch is 0 for
			 * AFBC); 4096 B/superblock, 3072 for 8-bit mem-saving.
			 */
			unsigned long block_size =
				(priv->viu.vd1_afbc_mode & AFBC_BLK_MEM_MODE) ?
					3072 : 4096;
			unsigned long body_size =
				(ALIGN(fb->width, 64) / 64) *
				(ALIGN(priv->viu.vd1_height0, 32) / 32) *
				block_size;

			priv->viu.vd1_afbc_body_addr = priv->viu.vd1_addr0 >> 4;
			priv->viu.vd1_afbc_head_addr =
				(priv->viu.vd1_addr0 + body_size) >> 4;
		} else {
			/* Default mode is 4k per superblock */
			unsigned long block_size = 4096;
			unsigned long body_size;

			/* 8bit mem saving mode is 3072bytes per superblock */
			if (priv->viu.vd1_afbc_mode & AFBC_BLK_MEM_MODE)
				block_size = 3072;

			body_size = (ALIGN(priv->viu.vd1_stride0, 64) / 64) *
				    (ALIGN(priv->viu.vd1_height0, 32) / 32) *
				    block_size;

			priv->viu.vd1_afbc_body_addr = priv->viu.vd1_addr0 >> 4;
			/* Header is after body content */
			priv->viu.vd1_afbc_head_addr = (priv->viu.vd1_addr0 +
							body_size) >> 4;
		}

		/*
		 * Decoder FBC-output (am21c) fix (fbc-output-display-impl-plan.md
		 * S1/S2, PROVEN by the S0 PoC: 7.7 dB garbage -> 23.4 dB picture).
		 * This branch only runs for AMLOGIC_FBC-modifier FBs, so it is
		 * already modifier-gated.  For a use_mmu decode (G12A+/SM1) the
		 * compress HEADER is NOT in the vb2 buffer -- the decoder wrote it
		 * to a SEPARATE per-slot buffer mmu_header_paddr[slot]
		 * (codec_hevc.c:1657-1658) and registered the {body base -> header}
		 * association via meson_overlay_fbc_set_header().  The BODY is
		 * contiguous at the vb2 base (vd1_addr0).  So if the registry has a
		 * header for this body base, point the AFBC scatter HEAD at it and
		 * keep the body at the vb2 base; this supersedes the (wrong) head =
		 * vb2_base + body_size computation above.  If NOT registered (e.g.
		 * a non-decoder FBC FB, or a decode path that didn't register),
		 * fall back to the computed head -- do not crash.
		 */
		fbc_header = meson_overlay_fbc_get_header(priv->viu.vd1_addr0);
		if (fbc_header) {
			priv->viu.vd1_afbc_body_addr = priv->viu.vd1_addr0 >> 4;
			priv->viu.vd1_afbc_head_addr = fbc_header >> 4;
		} else {
			DRM_DEBUG("AFBC body 0x%x: no registered compress header, using computed head 0x%x\n",
				  priv->viu.vd1_addr0,
				  priv->viu.vd1_afbc_head_addr);
		}
	}

	priv->viu.vd1_enabled = true;

	spin_unlock_irqrestore(&priv->drm->event_lock, flags);

	/*
	 * #13 fix: do NOT fire the PQ/HDR2 register writes here anymore. This
	 * runs from atomic_update (commit tail) against live MMIO -- a write
	 * here can land mid-raster (tearing) whenever a sysfs pq_* or
	 * hdr2_bypass write races an in-flight VD1 buffer flip, and it never
	 * ran at all on an OSD-only desktop (no VD1 commit ever reaches this
	 * function), so those sysfs params were permanently dead in that case.
	 * Stash only the frame-dependent HDR2 input size (src_w/h are only
	 * known at plane-commit time); the actual register programming is
	 * deferred to meson_overlay_pq_flush() below -- see its comment for
	 * what still needs to be wired up in meson_crtc.c.
	 */
	meson_overlay_pq_hdr2_in_w = new_state->src_w >> 16;
	meson_overlay_pq_hdr2_in_h = new_state->src_h >> 16;

	DRM_DEBUG_DRIVER("\n");
}

static void meson_overlay_atomic_disable(struct drm_plane *plane,
				       struct drm_atomic_state *state)
{
	struct meson_overlay *meson_overlay = to_meson_overlay(plane);
	struct meson_drm *priv = meson_overlay->priv;

	DRM_DEBUG_DRIVER("\n");

	priv->viu.vd1_enabled = false;

	/*
	 * Disable VD1 only. VD2 is an independent plane (meson_overlay2.c) and
	 * owns its own teardown -- do not clear VD2_BLEND_SRC_CTRL / VD2 IF0
	 * here, or disabling VD1 would also kill an active VD2.
	 */
	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A)) {
		writel_relaxed(0, priv->io_base + _REG(VD1_BLEND_SRC_CTRL));
		writel_relaxed(0, priv->io_base + _REG(VD1_IF0_GEN_REG + 0x17b0));
	} else
		writel_bits_relaxed(VPP_VD1_POSTBLEND | VPP_VD1_PREBLEND, 0,
				    priv->io_base + _REG(VPP_MISC));

}

static const struct drm_plane_helper_funcs meson_overlay_helper_funcs = {
	.atomic_check	= meson_overlay_atomic_check,
	.atomic_disable	= meson_overlay_atomic_disable,
	.atomic_update	= meson_overlay_atomic_update,
};

static bool meson_overlay_format_mod_supported(struct drm_plane *plane,
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

		/* The video AFBC decoder (VD1_AFBCD0) is G12A+; pre-G12A SoCs have no
		 * video FBC decoder and their meson-vdec outputs linear NV12, so gate
		 * the AMLOGIC_FBC modifiers off -- they were advertised but unusable on
		 * GXL/GXM (the IN_FORMATS list is filtered by this callback). */
		if (!meson_vpu_is_compatible(to_meson_overlay(plane)->priv,
					     VPU_COMPATIBLE_G12A))
			return false;

		if (format != DRM_FORMAT_YUV420_8BIT &&
		    format != DRM_FORMAT_YUV420_10BIT) {
			DRM_DEBUG_KMS("%llx invalid format 0x%08x\n",
				      modifier, format);
			return false;
		}

		if (layout != AMLOGIC_FBC_LAYOUT_BASIC &&
		    layout != AMLOGIC_FBC_LAYOUT_SCATTER) {
			DRM_DEBUG_KMS("%llx invalid layout %x\n",
				      modifier, layout);
			return false;
		}

		if (options &&
		    options != AMLOGIC_FBC_OPTION_MEM_SAVING) {
			DRM_DEBUG_KMS("%llx invalid layout %x\n",
				      modifier, layout);
			return false;
		}

		return true;
	}

	DRM_DEBUG_KMS("invalid modifier %llx for format 0x%08x\n",
		      modifier, format);

	return false;
}

static const struct drm_prop_enum_list meson_overlay_field_enum[] = {
	{ MESON_FIELD_PROGRESSIVE, "progressive" },
	{ MESON_FIELD_TOP,         "top-field" },
	{ MESON_FIELD_BOTTOM,      "bottom-field" },
};

/* Custom plane-state lifecycle: carry meson_overlay_state.field through atomic */
static void meson_overlay_reset(struct drm_plane *plane)
{
	struct meson_overlay_state *mos;

	if (plane->state) {
		__drm_atomic_helper_plane_destroy_state(plane->state);
		kfree(to_meson_overlay_state(plane->state));
		plane->state = NULL;
	}

	mos = kzalloc(sizeof(*mos), GFP_KERNEL);
	if (!mos)
		return;
	__drm_atomic_helper_plane_reset(plane, &mos->base);
	/* mos->field = MESON_FIELD_PROGRESSIVE (kzalloc) */
}

static struct drm_plane_state *
meson_overlay_duplicate_state(struct drm_plane *plane)
{
	struct meson_overlay_state *mos;

	if (WARN_ON(!plane->state))
		return NULL;

	mos = kzalloc(sizeof(*mos), GFP_KERNEL);
	if (!mos)
		return NULL;
	__drm_atomic_helper_plane_duplicate_state(plane, &mos->base);
	mos->field = to_meson_overlay_state(plane->state)->field;
	return &mos->base;
}

static void meson_overlay_destroy_state(struct drm_plane *plane,
					struct drm_plane_state *state)
{
	__drm_atomic_helper_plane_destroy_state(state);
	kfree(to_meson_overlay_state(state));
}

static int meson_overlay_set_property(struct drm_plane *plane,
				      struct drm_plane_state *state,
				      struct drm_property *property,
				      uint64_t val)
{
	struct meson_overlay *mo = to_meson_overlay(plane);

	if (property == mo->field_prop) {
		to_meson_overlay_state(state)->field = val;
		return 0;
	}
	return -EINVAL;
}

static int meson_overlay_get_property(struct drm_plane *plane,
				      const struct drm_plane_state *state,
				      struct drm_property *property,
				      uint64_t *val)
{
	struct meson_overlay *mo = to_meson_overlay(plane);

	if (property == mo->field_prop) {
		*val = to_meson_overlay_state(state)->field;
		return 0;
	}
	return -EINVAL;
}

static const struct drm_plane_funcs meson_overlay_funcs = {
	.update_plane		= drm_atomic_helper_update_plane,
	.disable_plane		= drm_atomic_helper_disable_plane,
	.destroy		= drm_plane_cleanup,
	.reset			= meson_overlay_reset,
	.atomic_duplicate_state = meson_overlay_duplicate_state,
	.atomic_destroy_state	= meson_overlay_destroy_state,
	.atomic_set_property	= meson_overlay_set_property,
	.atomic_get_property	= meson_overlay_get_property,
	.format_mod_supported   = meson_overlay_format_mod_supported,
};

static const uint32_t supported_drm_formats[] = {
	DRM_FORMAT_YUYV,
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV21,
	DRM_FORMAT_YUV444,
	DRM_FORMAT_YUV422,
	DRM_FORMAT_YUV420,
	DRM_FORMAT_YUV411,
	DRM_FORMAT_YUV410,
	DRM_FORMAT_RGB888,
	DRM_FORMAT_YUV420_8BIT, /* Amlogic FBC Only */
	DRM_FORMAT_YUV420_10BIT, /* Amlogic FBC Only */
};

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

int meson_overlay_create(struct meson_drm *priv)
{
	struct meson_overlay *meson_overlay;
	struct drm_plane *plane;

	DRM_DEBUG_DRIVER("\n");

	meson_overlay = devm_kzalloc(priv->drm->dev, sizeof(*meson_overlay),
				   GFP_KERNEL);
	if (!meson_overlay)
		return -ENOMEM;

	meson_overlay->priv = priv;
	plane = &meson_overlay->base;

	drm_universal_plane_init(priv->drm, plane, 0xFF,
				 &meson_overlay_funcs,
				 supported_drm_formats,
				 ARRAY_SIZE(supported_drm_formats),
				 format_modifiers,
				 DRM_PLANE_TYPE_OVERLAY, "meson_overlay_plane");

	drm_plane_helper_add(plane, &meson_overlay_helper_funcs);

	/* VD1 video plane. On G12A+ the VPP post-blend ports are reassignable,
	 * so zpos is mutable [0,3] (default 0 = bottom, video below OSD). GXL has
	 * a fixed post-blend stack -> keep zpos immutable. */
	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A))
		drm_plane_create_zpos_property(plane, 0, 0, 3);
	else
		drm_plane_create_zpos_immutable_property(plane, 0);

	/* Phase 3: interlaced field input mode (VFORMATTER vertical phase).
	 * Default "progressive" is a no-op; opt-in only. Generic VD1 register,
	 * so available on all SoCs. */
	meson_overlay->field_prop = drm_property_create_enum(priv->drm, 0,
				"field", meson_overlay_field_enum,
				ARRAY_SIZE(meson_overlay_field_enum));
	if (meson_overlay->field_prop)
		drm_object_attach_property(&plane->base,
					   meson_overlay->field_prop,
					   MESON_FIELD_PROGRESSIVE);

	priv->overlay_plane = plane;

	DRM_DEBUG_DRIVER("\n");

	return 0;
}
