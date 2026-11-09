/* SPDX-License-Identifier: GPL-2.0+ */

#ifndef __MESON_VDEC_CODEC_MJPEG_H_
#define __MESON_VDEC_CODEC_MJPEG_H_

#include "vdec.h"

extern struct amvdec_codec_ops codec_mjpeg_ops;

/*
 * MJPEG hardware/firmware resolution limits (gxl_mjpeg.bin, upstream
 * variant sha256 8a4997cd...).  These are the compile-time MAXIMA that
 * reflect what the firmware can decode -- NOT a soft policy cap.  The
 * actual usable max on a given board is CMA-constrained at probe time
 * (see core->mjpeg_cma_max_pixels) so high-CMA boards get the full
 * capability and low-CMA boards reject cleanly at S_FMT instead of
 * failing with ENOMEM at start_streaming (internal codec resolution-limits
 * note, "Resolution Limits").
 *
 * Width 8192: confirmed by boundary test on G12B (A311D, 1G CMA,
 * 2026-06-07) -- 8192x256 and the full 8192x2048 corner both decode
 * 5/5 frames (output = W*H*3/2, vdec IRQ ~1/frame).  Widths above 8192
 * are rejected at format negotiation by this same cap; 8192 is the
 * proven HW maximum and matches the VDEC engine's 8K limit.
 *
 * Height 2048: the firmware silently produces 0 frames above 2048 (a
 * sharp 2^11 PSCALE boundary), confirmed SM1 2026-04-01.  Do NOT raise.
 */
#define MJPEG_HW_MAX_WIDTH	8192
#define MJPEG_HW_MAX_HEIGHT	2048

#endif
