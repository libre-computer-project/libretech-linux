// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018 BayLibre, SAS
 * Author: Maxime Jourdan <mjourdan@baylibre.com>
 */

#include "vdec_platform.h"
#include "vdec.h"

#include "vdec_1.h"
#include "vdec_hevc.h"
#include "codec_avs.h"
#include "codec_hevc_fw.h"
#include "codec_vp9_fw.h"
#include "codec_avs2_fw.h"
#include "codec_h264_fw.h"
#include "codec_h264_mvc_fw.h"
#include "codec_mpeg12_fw.h"
#include "codec_mpeg4_fw.h"
#include "codec_mjpeg_fw.h"
#include "codec_mjpeg.h"
#include "codec_h263_fw.h"

static struct amvdec_format vdec_formats_gxbb[] = {
	{
		.pixfmt = V4L2_PIX_FMT_H264,
		.min_buffers = 2,
		.max_buffers = 32,
		.max_width = 1920,
		.max_height = 1080,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-h264 */
		.firmware_path = "meson/vdec/gxbb_h264.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV12, 0 },
		.fw_versions = h264_fw_versions,
		.fw_versions_count = H264_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED |
			 V4L2_FMT_FLAG_DYN_RESOLUTION,
	}, {
		.pixfmt = V4L2_PIX_FMT_MPEG1,
		.min_buffers = 8,
		.max_buffers = 8,
		.max_width = 1920,
		.max_height = 1080,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-mpeg12 */
		.firmware_path = "meson/vdec/gxl_mpeg12.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV12, 0 },
		.fw_versions = mpeg12_fw_versions,
		.fw_versions_count = MPEG12_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED,
	}, {
		.pixfmt = V4L2_PIX_FMT_MPEG2,
		.min_buffers = 8,
		.max_buffers = 8,
		.max_width = 1920,
		.max_height = 1080,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-mpeg12 */
		.firmware_path = "meson/vdec/gxl_mpeg12.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV12, 0 },
		.fw_versions = mpeg12_fw_versions,
		.fw_versions_count = MPEG12_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED,
	},
};

static struct amvdec_format vdec_formats_gxl[] = {
	{
		.pixfmt = V4L2_PIX_FMT_HEVC,
		.min_buffers = 16,
		.max_buffers = 24,
		.max_width = 4096,
		.max_height = 2304,
		.vdec_ops = &vdec_hevc_ops,
		.codec_ops = NULL, /* meson-vdec-hevc */
		.firmware_path = "meson/vdec/gxl_hevc_mmu.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV21M, V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_NV21, V4L2_PIX_FMT_YUV420M },
		.fw_versions = hevc_fw_versions,
		.fw_versions_count = HEVC_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED |
			 V4L2_FMT_FLAG_DYN_RESOLUTION,
	}, {
		.pixfmt = V4L2_PIX_FMT_VP9,
		.min_buffers = 16,
		.max_buffers = 24,
		.max_width = 4096,
		.max_height = 2304,
		.vdec_ops = &vdec_hevc_ops,
		.codec_ops = NULL, /* meson-vdec-vp9 */
		.firmware_path = "meson/vdec/gxl_vp9_mmu.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV21M, V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_NV21, V4L2_PIX_FMT_YUV420M },
		.fw_versions = vp9_fw_versions,
		.fw_versions_count = VP9_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED |
			 V4L2_FMT_FLAG_DYN_RESOLUTION,
	}, {
		.pixfmt = V4L2_PIX_FMT_H264,
		.min_buffers = 2,
		.max_buffers = 32,
		/* KI-171: H.264 max DPB; gxl_h264.bin stalls on fewer bufs */
		.prehdr_min_capture = 16,
		.min_width = 208,
		/*
		 * gxl_h264.bin firmware has a COMBINED macroblock budget of ~8400
		 * MBs and deadlocks at MBY_MBX=0 for any input above the budget,
		 * regardless of SoC.  Applies to BOTH S905X (GXL) and S805X (GXLX)
		 * because both load the same gxl_h264.bin.  The prior 4096x4096
		 * advertised value was stale -- inherited from the BSP path
		 * VIDEO_DEC_H264_4k2K, which uses a different multi-instance firmware
		 * never ported to mainline.  See:
		 *   the internal GXL H.264 audit directory
		 *     s805x resolution-firmware note (CORRECTION 2026-04-05)
		 * for the firmware-budget derivation.  max_height=2400 reflects the
		 * width-dependent cap at narrow widths; max_mb_total enforces the
		 * actual combined budget in codec_h264.c pre_queue() via SPS parse.
		 */
		.max_width = 1952,
		.max_height = 2400,
		.max_mb_total = 8400,	/* gxl_h264.bin combined macroblock budget */
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-h264 */
		.firmware_path = "meson/vdec/gxl_h264.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV12, 0 },
		.fw_versions = h264_fw_versions,
		.fw_versions_count = H264_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED |
			 V4L2_FMT_FLAG_DYN_RESOLUTION,
	}, {
		/*
		 * H.264 stateless (V4L2 Request API). Uses libre_gxl_h264.bin
		 * firmware with IMEM-overlay per h264-firmware-design.md §11
		 * (3 active overlays: HEADER/DATA/SLICE). CABAC is HW per
		 * 2026-04-22 audit (cabac_step_2 block in POWER_CTL_VLD).
		 *
		 * GXL loads libre_gxl_h264.bin (the 3 per-arch firmware deltas
		 * from §7 are the only difference vs G12B/SM1).
		 */
		.pixfmt = V4L2_PIX_FMT_H264_SLICE,
		.min_buffers = 4,
		.max_buffers = 17,		/* up to 16 refs + 1 output */
		.min_width = 258,		/* same as stateful H.264 GXL lower bound */
		.max_width = 4096,
		.max_height = 4096,
		.max_mb_total = 36864,		/* Level 5.1 MaxFS */
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL,		/* meson-vdec-h264-libre */
		.firmware_path = "meson/vdec/libre_gxl_h264.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV12, 0 },
		.fw_versions = h264_fw_versions,
		.fw_versions_count = H264_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED |
			 V4L2_FMT_FLAG_DYN_RESOLUTION,
	}, {
		/*
		 * H.264 MVC (Multi-View Coding) -- stereo 3D, outputs view 0 only.
		 * Uses dedicated MVC firmware with bidirectional mailbox protocol.
		 * BSP source: vh264_mvc.c, firmware: gxl_h264_mvc.bin.
		 */
		.pixfmt = V4L2_PIX_FMT_H264_MVC,
		.min_buffers = 4,
		.max_buffers = 34, /* MVC needs 30+4 margin */
		.max_width = 1920,
		.max_height = 1088,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-h264-mvc */
		.firmware_path = "meson/vdec/gxl_h264_mvc.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV12, 0 },
		.fw_versions = h264_mvc_fw_versions,
		.fw_versions_count = H264_MVC_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED |
			 V4L2_FMT_FLAG_DYN_RESOLUTION,
	}, {
		.pixfmt = V4L2_PIX_FMT_MPEG4,
		.min_buffers = 8,
		.max_buffers = 8,
		.max_width = 1920,
		.max_height = 1080,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-mpeg4 */
		.firmware_path = "meson/vdec/gxl_mpeg4_5.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV12, 0 },
		.fw_versions = mpeg4_fw_versions,
		.fw_versions_count = MPEG4_FW_VERSIONS_COUNT,
		/*
		 * Continuous single-instance firmware: no DYN_RESOLUTION, so
		 * poweron defers to CAPTURE STREAMON -- the firmware starts
		 * with real canvases and runs continuously (BSP/MPEG12 model).
		 * early_src_change emits SOURCE_CHANGE at OUTPUT STREAMON from
		 * the parsed resolution.  mpeg4/plan/src-change-restructure.md.
		 */
		.flags = V4L2_FMT_FLAG_COMPRESSED,
		.early_src_change = true,
	}, {
		.pixfmt = V4L2_PIX_FMT_H263,
		.min_buffers = 8,
		.max_buffers = 8,
		.max_width = 1920,
		.max_height = 1080,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-mpeg4 */
		.firmware_path = "meson/vdec/gxl_h263.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV12, 0 },
		.fw_versions = h263_fw_versions,
		.fw_versions_count = H263_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED |
			 V4L2_FMT_FLAG_DYN_RESOLUTION,
	}, {
		.pixfmt = V4L2_PIX_FMT_MJPEG,
		.min_buffers = 4,
		.max_buffers = 4,
		/*
		 * HW maxima (see codec_mjpeg.h): width 8192 confirmed by G12B
		 * boundary test (8192x2048 = 5/5 frames, 2026-06-07); height
		 * 2048 is a sharp firmware/PSCALE 2^11 boundary (>2048 silently
		 * produces 0 frames, SM1 2026-04-01).  Effective HW max
		 * 8192x2048 (16.7 MP).  Actual usable max is CMA-constrained at
		 * probe (core->mjpeg_cma_max_pixels) and enforced in
		 * enum_framesizes + try_fmt.
		 */
		.max_width = MJPEG_HW_MAX_WIDTH,
		.max_height = MJPEG_HW_MAX_HEIGHT,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-mjpeg */
		.firmware_path = "meson/vdec/gxl_mjpeg.bin",
		/*
		 * MJPEG PSCALE writes I420 (3 separate planes: Y, U, V).
		 * NV12/NV21 canvases pack U+V into one interleaved plane,
		 * causing V data to overwrite U -- destroying chroma.
		 * Only YUV420M (3-plane) matches the hardware output.
		 */
		.pixfmts_cap = { V4L2_PIX_FMT_YUV420M, 0 },
		.fw_versions = mjpeg_fw_versions,
		.fw_versions_count = MJPEG_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED,
	}, {
		.pixfmt = V4L2_PIX_FMT_MPEG1,
		.min_buffers = 8,
		.max_buffers = 8,
		.max_width = 1920,
		.max_height = 1080,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-mpeg12 */
		.firmware_path = "meson/vdec/gxl_mpeg12.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV12, 0 },
		.fw_versions = mpeg12_fw_versions,
		.fw_versions_count = MPEG12_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED,
	}, {
		.pixfmt = V4L2_PIX_FMT_MPEG2,
		.min_buffers = 8,
		.max_buffers = 8,
		.max_width = 1920,
		.max_height = 1080,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-mpeg12 */
		.firmware_path = "meson/vdec/gxl_mpeg12.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV12, 0 },
		.fw_versions = mpeg12_fw_versions,
		.fw_versions_count = MPEG12_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED,
	}, {
		/*
		 * AVS (Audio Video Standard, GB/T 20090.2) -- Chinese national
		 * video coding standard, Jizhun profile.  VDEC_1 engine, max 1080p.
		 * BSP source: avs_multi/avs_multi.c, firmware: gxl_avs.bin.
		 */
		.pixfmt = V4L2_PIX_FMT_AVS,
		.min_buffers = 8,
		.max_buffers = 8,
		.max_width = 1920,
		.max_height = 1080,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-avs */
		.firmware_path = "meson/vdec/gxl_avs.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV12, 0 },
		.flags = V4L2_FMT_FLAG_COMPRESSED |
			 V4L2_FMT_FLAG_DYN_RESOLUTION,
		.fw_versions = avs_fw_versions,
		.fw_versions_count = AVS_FW_VERSIONS_COUNT,
	},
};

static struct amvdec_format vdec_formats_gxlx[] = {
	{
		.pixfmt = V4L2_PIX_FMT_H264,
		.min_buffers = 2,
		.max_buffers = 32,
		/* KI-171: H.264 max DPB; gxl_h264.bin stalls on fewer bufs */
		.prehdr_min_capture = 16,
		.min_width = 208,
		.max_width = 1920,	/* S805X GXLX: firmware hangs at MBY_MBX=0 for any */
		.max_height = 1088,	/* resolution above 1080p regardless of DOS_SCRATCH_31 */
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-h264 */
		.firmware_path = "meson/vdec/gxl_h264.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV12, 0 },
		.fw_versions = h264_fw_versions,
		.fw_versions_count = H264_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED |
			 V4L2_FMT_FLAG_DYN_RESOLUTION,
	}, {
		.pixfmt = V4L2_PIX_FMT_MPEG1,
		.min_buffers = 8,
		.max_buffers = 8,
		.max_width = 1920,
		.max_height = 1080,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-mpeg12 */
		.firmware_path = "meson/vdec/gxl_mpeg12.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV12, 0 },
		.fw_versions = mpeg12_fw_versions,
		.fw_versions_count = MPEG12_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED,
	}, {
		.pixfmt = V4L2_PIX_FMT_MPEG2,
		.min_buffers = 8,
		.max_buffers = 8,
		.max_width = 1920,
		.max_height = 1080,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-mpeg12 */
		.firmware_path = "meson/vdec/gxl_mpeg12.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV12, 0 },
		.fw_versions = mpeg12_fw_versions,
		.fw_versions_count = MPEG12_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED,
	},
};

static struct amvdec_format vdec_formats_gxm[] = {
	{
		.pixfmt = V4L2_PIX_FMT_VP9,
		.min_buffers = 16,
		.max_buffers = 24,
		.max_width = 4096,
		.max_height = 2304,
		.vdec_ops = &vdec_hevc_ops,
		.codec_ops = NULL, /* meson-vdec-vp9 */
		.firmware_path = "meson/vdec/gxl_vp9_mmu.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV21M, V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_NV21, V4L2_PIX_FMT_YUV420M },
		.fw_versions = vp9_fw_versions,
		.fw_versions_count = VP9_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED |
			 V4L2_FMT_FLAG_DYN_RESOLUTION,
	}, {
		.pixfmt = V4L2_PIX_FMT_H264,
		.min_buffers = 2,
		.max_buffers = 32,
		.min_width = 208,
		.max_width = 4080,
		.max_height = 4080,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-h264 */
		.firmware_path = "meson/vdec/gxm_h264.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV12, 0 },
		.fw_versions = h264_fw_versions,
		.fw_versions_count = H264_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED |
			 V4L2_FMT_FLAG_DYN_RESOLUTION,
	}, {
		/* H.264 MVC on GXM (S912) -- uses gxm_h264_mvc.bin firmware */
		.pixfmt = V4L2_PIX_FMT_H264_MVC,
		.min_buffers = 4,
		.max_buffers = 34, /* MVC needs 30+4 margin */
		.max_width = 1920,
		.max_height = 1088,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-h264-mvc */
		.firmware_path = "meson/vdec/gxm_h264_mvc.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV12, 0 },
		.fw_versions = h264_mvc_fw_versions,
		.fw_versions_count = H264_MVC_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED |
			 V4L2_FMT_FLAG_DYN_RESOLUTION,
	}, {
		.pixfmt = V4L2_PIX_FMT_MPEG1,
		.min_buffers = 8,
		.max_buffers = 8,
		.max_width = 1920,
		.max_height = 1080,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-mpeg12 */
		.firmware_path = "meson/vdec/gxl_mpeg12.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV12, 0 },
		.fw_versions = mpeg12_fw_versions,
		.fw_versions_count = MPEG12_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED,
	}, {
		.pixfmt = V4L2_PIX_FMT_MPEG2,
		.min_buffers = 8,
		.max_buffers = 8,
		.max_width = 1920,
		.max_height = 1080,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-mpeg12 */
		.firmware_path = "meson/vdec/gxl_mpeg12.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV12, 0 },
		.fw_versions = mpeg12_fw_versions,
		.fw_versions_count = MPEG12_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED,
	}, {
		.pixfmt = V4L2_PIX_FMT_AVS,
		.min_buffers = 8,
		.max_buffers = 8,
		.max_width = 1920,
		.max_height = 1080,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-avs */
		.firmware_path = "meson/vdec/gxl_avs.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV12, 0 },
		.flags = V4L2_FMT_FLAG_COMPRESSED |
			 V4L2_FMT_FLAG_DYN_RESOLUTION,
		.fw_versions = avs_fw_versions,
		.fw_versions_count = AVS_FW_VERSIONS_COUNT,
	},
};

static struct amvdec_format vdec_formats_g12a[] = {
	{
		.pixfmt = V4L2_PIX_FMT_HEVC,
		.min_buffers = 16,
		.max_buffers = 24,
		.max_width = 4096,
		.max_height = 2304,
		.vdec_ops = &vdec_hevc_ops,
		.codec_ops = NULL, /* meson-vdec-hevc */
		.firmware_path = "meson/vdec/g12a_hevc_mmu.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV21M, V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_NV21, V4L2_PIX_FMT_YUV420M, V4L2_PIX_FMT_MESON_FBC08_SCATTER, V4L2_PIX_FMT_MESON_FBC10_SCATTER },
		.fw_versions = hevc_fw_versions,
		.fw_versions_count = HEVC_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED |
			 V4L2_FMT_FLAG_DYN_RESOLUTION,
	}, {
		.pixfmt = V4L2_PIX_FMT_VP9,
		/*
		 * VP9 spec requires at most 8 reference frames. The driver
		 * holds those 8 ref slots plus 1 cur_frame internally, leaving
		 * a thin margin for user-space dequeue. Set min == max so the
		 * v4l2 client allocates the full pool up front and m2m never
		 * runs dry mid-decode at 4K (TODO-VP9-011 root cause).
		 *
		 * Original 4K cap budget: 12 * (12 MB NV12 + 16 MB FBC body)
		 * + 48 MB fixed = ~388 MB.  cma-opt-sg (mainline 7672f0bb06f0)
		 * moves the 16 MB FBC body off contiguous CMA per page, so the
		 * remaining contig budget is ~12 * 12 MB NV12 + fixed (~150 MB
		 * at 4K, ~265 MB at 5K, ~385 MB at 6K).  G12A/G12B 1 GB CMA
		 * boards accommodate up to 6K with fbc_sg=1; raise the cap to
		 * the HW max (8K) to match SM1.  fbc_sg=0 sessions on smaller
		 * G12A/G12B CMA boards still fail allocation at 5K+ -- expected.
		 */
		.min_buffers = 12,
		.max_buffers = 12,
		.max_width = 8192,
		.max_height = 4320,
		.vdec_ops = &vdec_hevc_ops,
		.codec_ops = NULL, /* meson-vdec-vp9 */
		/* Use SM1 firmware on G12A/G12B too: the older g12a_vp9.bin
		 * caps decode at ~24-25 frames on 5K/6K streams (VP9-025,
		 * firmware-RE hypothesis: frame-buffer limit encoded in
		 * VP9_CONTROL init at PC 0x0061).  sm1_vp9_mmu.bin has the
		 * limit removed and produces 30/30 at every resolution up to
		 * the 8K HW ceiling, bit-for-bit compatible with the G12A/G12B
		 * HEVC engine (identical register map, same 28 KB size).
		 */
		.firmware_path = "meson/vdec/sm1_vp9_mmu.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV21M, V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_NV21, V4L2_PIX_FMT_YUV420M, V4L2_PIX_FMT_MESON_FBC08_SCATTER, V4L2_PIX_FMT_MESON_FBC10_SCATTER },
		.fw_versions = vp9_fw_versions,
		.fw_versions_count = VP9_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED |
			 V4L2_FMT_FLAG_DYN_RESOLUTION,
	}, {
		.pixfmt = V4L2_PIX_FMT_H264,
		.min_buffers = 2,
		/*
		 * g12a_h264.bin manages a DPB of at most 24 buffers (its internal
		 * buffer_spec[] mirrors the driver's MAX_REF_PIC_NUM=24).  Writing a
		 * DPB size of 25 into AV_SCRATCH_0[23:16] overflows it: the C-core
		 * reconstruction wedges on the first frame (CPC_E stuck ~0xc51, the
		 * M-core spins the C-core doorbell forever at 0x060d) and 0 frames are
		 * emitted.  gst/v4l2 clients that allocate <=24 CAPTURE buffers decode
		 * fine; Android CCodec/v4l2_codec2 requests 25 and hit the overflow
		 * (KI-091/125 "last-MB withhold" was this, mis-attributed to firmware).
		 * Cap at 24 -- verified on SM1 g12a_h264.bin (ed2014ed): 24 buffers ->
		 * 91/91 frames, 25 -> C-core wedge.
		 */
		.max_buffers = 24,
		/* KI-171: H.264 max DPB; g12a_h264.bin stalls on fewer bufs too */
		.prehdr_min_capture = 16,
		.max_width = 4096,	/* g12a_h264.bin: 8-bit mb_width overflow at 4096px recovered */
		.max_height = 4096,	/* via SPS pre-parse (86cec90044cc). Height has no register   */
				/* overflow (mb_height = mb_total/mb_width, not a stored field). */
				/* G12A/G12B both use this table (g12b DTS uses amlogic,g12a-vdec). */
				/* Actual limit is mb_total ~36864 MBs (firmware budget).          */
		.max_mb_total = 36864,	/* g12a_h264.bin firmware budget: 4096x2304 (256x144 MBs)     */
				/* or 2304x4096 (144x256 MBs). 4080x4080 fails (firmware returns  */
				/* mb_total=0 at 255x255 MBs). Probed 2026-04-04.                 */
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-h264 */
		.firmware_path = "meson/vdec/g12a_h264.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV21M, V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_NV21, V4L2_PIX_FMT_YUV420M },
		.fw_versions = h264_fw_versions,
		.fw_versions_count = H264_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED |
			 V4L2_FMT_FLAG_DYN_RESOLUTION,
	}, {
		/*
		 * H.264 MVC on G12A -- uses gxm_h264_mvc.bin (highest CPU ID <= G12A = 0x28).
		 * No g12a/g12b-specific MVC firmware; gxm is the closest match.
		 * BSP source: firmware_drv.c:fw_data_filter().
		 */
		.pixfmt = V4L2_PIX_FMT_H264_MVC,
		.min_buffers = 4,
		.max_buffers = 34, /* MVC needs 30+4 margin */
		.max_width = 1920,
		.max_height = 1088,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-h264-mvc */
		.firmware_path = "meson/vdec/gxm_h264_mvc.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV12, 0 },
		.fw_versions = h264_mvc_fw_versions,
		.fw_versions_count = H264_MVC_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED |
			 V4L2_FMT_FLAG_DYN_RESOLUTION,
	}, {
		.pixfmt = V4L2_PIX_FMT_MPEG4,
		.min_buffers = 8,
		.max_buffers = 8,
		.max_width = 1920,
		.max_height = 1080,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-mpeg4 */
		.firmware_path = "meson/vdec/gxl_mpeg4_5.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV21M, V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_NV21, V4L2_PIX_FMT_YUV420M },
		.fw_versions = mpeg4_fw_versions,
		.fw_versions_count = MPEG4_FW_VERSIONS_COUNT,
		/*
		 * Continuous single-instance firmware: no DYN_RESOLUTION, so
		 * poweron defers to CAPTURE STREAMON -- the firmware starts
		 * with real canvases and runs continuously (BSP/MPEG12 model).
		 * early_src_change emits SOURCE_CHANGE at OUTPUT STREAMON from
		 * the parsed resolution.  mpeg4/plan/src-change-restructure.md.
		 */
		.flags = V4L2_FMT_FLAG_COMPRESSED,
		.early_src_change = true,
	}, {
		.pixfmt = V4L2_PIX_FMT_H263,
		.min_buffers = 8,
		.max_buffers = 8,
		.max_width = 1920,
		.max_height = 1080,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-mpeg4 */
		.firmware_path = "meson/vdec/gxl_mpeg4_5.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV21M, V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_NV21, V4L2_PIX_FMT_YUV420M },
		.fw_versions = h263_fw_versions,
		.fw_versions_count = H263_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED |
			 V4L2_FMT_FLAG_DYN_RESOLUTION,
	}, {
		.pixfmt = V4L2_PIX_FMT_MJPEG,
		.min_buffers = 4,
		.max_buffers = 4,
		/*
		 * HW maxima (see codec_mjpeg.h): width 8192 confirmed by G12B
		 * boundary test (8192x2048 = 5/5 frames, 2026-06-07); height
		 * 2048 is a sharp firmware/PSCALE 2^11 boundary (>2048 silently
		 * produces 0 frames, SM1 2026-04-01).  Effective HW max
		 * 8192x2048 (16.7 MP).  Actual usable max is CMA-constrained at
		 * probe (core->mjpeg_cma_max_pixels) and enforced in
		 * enum_framesizes + try_fmt.
		 */
		.max_width = MJPEG_HW_MAX_WIDTH,
		.max_height = MJPEG_HW_MAX_HEIGHT,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-mjpeg */
		.firmware_path = "meson/vdec/gxl_mjpeg.bin",
		/*
		 * MJPEG PSCALE writes I420 (3 separate planes: Y, U, V).
		 * NV12/NV21 canvases pack U+V into one interleaved plane,
		 * causing V data to overwrite U -- destroying chroma.
		 * Only YUV420M (3-plane) matches the hardware output.
		 */
		.pixfmts_cap = { V4L2_PIX_FMT_YUV420M, 0 },
		.fw_versions = mjpeg_fw_versions,
		.fw_versions_count = MJPEG_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED,
	}, {
		.pixfmt = V4L2_PIX_FMT_MPEG1,
		.min_buffers = 8,
		.max_buffers = 8,
		.max_width = 1920,
		.max_height = 1080,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-mpeg12 */
		.firmware_path = "meson/vdec/gxl_mpeg12.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV21M, V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_NV21, V4L2_PIX_FMT_YUV420M },
		.fw_versions = mpeg12_fw_versions,
		.fw_versions_count = MPEG12_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED,
	}, {
		.pixfmt = V4L2_PIX_FMT_MPEG2,
		.min_buffers = 8,
		.max_buffers = 8,
		.max_width = 1920,
		.max_height = 1080,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-mpeg12 */
		.firmware_path = "meson/vdec/gxl_mpeg12.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV21M, V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_NV21, V4L2_PIX_FMT_YUV420M },
		.fw_versions = mpeg12_fw_versions,
		.fw_versions_count = MPEG12_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED,
	}, {
		.pixfmt = V4L2_PIX_FMT_AVS,
		.min_buffers = 8,
		.max_buffers = 8,
		.max_width = 1920,
		.max_height = 1080,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-avs */
		.firmware_path = "meson/vdec/gxl_avs.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV21M, V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_NV21, V4L2_PIX_FMT_YUV420M },
		.flags = V4L2_FMT_FLAG_COMPRESSED |
			 V4L2_FMT_FLAG_DYN_RESOLUTION,
		.fw_versions = avs_fw_versions,
		.fw_versions_count = AVS_FW_VERSIONS_COUNT,
	},
};

static struct amvdec_format vdec_formats_g12b[] = {
	{
		.pixfmt = V4L2_PIX_FMT_HEVC,
		.min_buffers = 16,
		.max_buffers = 24,
		.max_width = 4096,
		.max_height = 2304,
		.vdec_ops = &vdec_hevc_ops,
		.codec_ops = NULL, /* meson-vdec-hevc */
		.firmware_path = "meson/vdec/g12a_hevc_mmu.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV21M, V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_NV21, V4L2_PIX_FMT_YUV420M, V4L2_PIX_FMT_MESON_FBC08_SCATTER, V4L2_PIX_FMT_MESON_FBC10_SCATTER },
		.fw_versions = hevc_fw_versions,
		.fw_versions_count = HEVC_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED |
			 V4L2_FMT_FLAG_DYN_RESOLUTION,
	}, {
		.pixfmt = V4L2_PIX_FMT_VP9,
		/*
		 * VP9 spec requires at most 8 reference frames. The driver
		 * holds those 8 ref slots plus 1 cur_frame internally, leaving
		 * a thin margin for user-space dequeue. Set min == max so the
		 * v4l2 client allocates the full pool up front and m2m never
		 * runs dry mid-decode at 4K (TODO-VP9-011 root cause).
		 *
		 * Original 4K cap budget: 12 * (12 MB NV12 + 16 MB FBC body)
		 * + 48 MB fixed = ~388 MB.  cma-opt-sg (mainline 7672f0bb06f0)
		 * moves the 16 MB FBC body off contiguous CMA per page, so the
		 * remaining contig budget is ~12 * 12 MB NV12 + fixed (~150 MB
		 * at 4K, ~265 MB at 5K, ~385 MB at 6K).  G12A/G12B 1 GB CMA
		 * boards accommodate up to 6K with fbc_sg=1; raise the cap to
		 * the HW max (8K) to match SM1.  fbc_sg=0 sessions on smaller
		 * G12A/G12B CMA boards still fail allocation at 5K+ -- expected.
		 */
		.min_buffers = 12,
		.max_buffers = 12,
		.max_width = 8192,
		.max_height = 4320,
		.vdec_ops = &vdec_hevc_ops,
		.codec_ops = NULL, /* meson-vdec-vp9 */
		/* Use SM1 firmware on G12A/G12B too: the older g12a_vp9.bin
		 * caps decode at ~24-25 frames on 5K/6K streams (VP9-025,
		 * firmware-RE hypothesis: frame-buffer limit encoded in
		 * VP9_CONTROL init at PC 0x0061).  sm1_vp9_mmu.bin has the
		 * limit removed and produces 30/30 at every resolution up to
		 * the 8K HW ceiling, bit-for-bit compatible with the G12A/G12B
		 * HEVC engine (identical register map, same 28 KB size).
		 */
		.firmware_path = "meson/vdec/sm1_vp9_mmu.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV21M, V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_NV21, V4L2_PIX_FMT_YUV420M, V4L2_PIX_FMT_MESON_FBC08_SCATTER, V4L2_PIX_FMT_MESON_FBC10_SCATTER },
		.fw_versions = vp9_fw_versions,
		.fw_versions_count = VP9_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED |
			 V4L2_FMT_FLAG_DYN_RESOLUTION,
	}, {
		.pixfmt = V4L2_PIX_FMT_H264,
		.min_buffers = 2,
		/*
		 * g12a_h264.bin manages a DPB of at most 24 buffers (its internal
		 * buffer_spec[] mirrors the driver's MAX_REF_PIC_NUM=24).  Writing a
		 * DPB size of 25 into AV_SCRATCH_0[23:16] overflows it: the C-core
		 * reconstruction wedges on the first frame (CPC_E stuck ~0xc51, the
		 * M-core spins the C-core doorbell forever at 0x060d) and 0 frames are
		 * emitted.  gst/v4l2 clients that allocate <=24 CAPTURE buffers decode
		 * fine; Android CCodec/v4l2_codec2 requests 25 and hit the overflow
		 * (KI-091/125 "last-MB withhold" was this, mis-attributed to firmware).
		 * Cap at 24 -- verified on SM1 g12a_h264.bin (ed2014ed): 24 buffers ->
		 * 91/91 frames, 25 -> C-core wedge.
		 */
		.max_buffers = 24,
		/* KI-171: H.264 max DPB; g12a_h264.bin stalls on fewer bufs too */
		.prehdr_min_capture = 16,
		.max_width = 4096,	/* g12a_h264.bin: 8-bit mb_width overflow at 4096px recovered */
		.max_height = 4096,	/* via SPS pre-parse (86cec90044cc). Height has no register   */
				/* overflow (mb_height = mb_total/mb_width, not a stored field). */
				/* Actual limit is mb_total ~36864 MBs (firmware budget). At    */
				/* 4096px wide: max height = 36864/256 = 144 MBs = 2304px.      */
				/* At 2304px wide: max height = 36864/144 = 256 MBs = 4096px.   */
				/* 4080x4080 fails: firmware returns mb_total=0 at 255x255 MBs. */
		.max_mb_total = 36864,	/* g12a_h264.bin firmware budget. Checked in pre_queue via SPS. */
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-h264 */
		.firmware_path = "meson/vdec/g12a_h264.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV21M, V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_NV21, V4L2_PIX_FMT_YUV420M },
		.fw_versions = h264_fw_versions,
		.fw_versions_count = H264_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED |
			 V4L2_FMT_FLAG_DYN_RESOLUTION,
	}, {
		/*
		 * H.264 stateless (V4L2 Request API). Uses libre_g12b_h264.bin
		 * firmware with IMEM-overlay per h264-firmware-design.md §11
		 * (3 active overlays: HEADER/DATA/SLICE). CABAC is HW per
		 * 2026-04-22 audit (cabac_step_2 block in POWER_CTL_VLD).
		 *
		 * G12B and SM1 share g12a_h264_multi.bin firmware family, so
		 * libre_g12b_h264.bin is byte-identical to libre_sm1_h264.bin.
		 */
		.pixfmt = V4L2_PIX_FMT_H264_SLICE,
		.min_buffers = 4,
		.max_buffers = 17,		/* up to 16 refs + 1 output */
		.min_width = 258,
		.max_width = 4096,
		.max_height = 4096,
		.max_mb_total = 36864,		/* Level 5.1 MaxFS */
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL,		/* meson-vdec-h264-libre */
		.firmware_path = "meson/vdec/libre_g12b_h264.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV12, 0 },
		.fw_versions = h264_fw_versions,
		.fw_versions_count = H264_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED |
			 V4L2_FMT_FLAG_DYN_RESOLUTION,
	}, {
		/*
		 * H.264 MVC on G12B (A311D) -- uses gxm_h264_mvc.bin.
		 * No g12b-specific MVC firmware; gxm is highest CPU ID <= G12B (0x29).
		 * BSP source: firmware_drv.c:fw_data_filter().
		 */
		.pixfmt = V4L2_PIX_FMT_H264_MVC,
		.min_buffers = 4,
		.max_buffers = 34, /* MVC needs 30+4 margin */
		.max_width = 1920,
		.max_height = 1088,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-h264-mvc */
		.firmware_path = "meson/vdec/gxm_h264_mvc.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV12, 0 },
		.fw_versions = h264_mvc_fw_versions,
		.fw_versions_count = H264_MVC_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED |
			 V4L2_FMT_FLAG_DYN_RESOLUTION,
	}, {
		.pixfmt = V4L2_PIX_FMT_MPEG4,
		.min_buffers = 8,
		.max_buffers = 8,
		.max_width = 1920,
		.max_height = 1080,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-mpeg4 */
		.firmware_path = "meson/vdec/gxl_mpeg4_5.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV21M, V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_NV21, V4L2_PIX_FMT_YUV420M },
		.fw_versions = mpeg4_fw_versions,
		.fw_versions_count = MPEG4_FW_VERSIONS_COUNT,
		/*
		 * Continuous single-instance firmware: no DYN_RESOLUTION, so
		 * poweron defers to CAPTURE STREAMON -- the firmware starts
		 * with real canvases and runs continuously (BSP/MPEG12 model).
		 * early_src_change emits SOURCE_CHANGE at OUTPUT STREAMON from
		 * the parsed resolution.  mpeg4/plan/src-change-restructure.md.
		 */
		.flags = V4L2_FMT_FLAG_COMPRESSED,
		.early_src_change = true,
	}, {
		.pixfmt = V4L2_PIX_FMT_H263,
		.min_buffers = 8,
		.max_buffers = 8,
		.max_width = 1920,
		.max_height = 1080,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-mpeg4 */
		.firmware_path = "meson/vdec/gxl_mpeg4_5.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV21M, V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_NV21, V4L2_PIX_FMT_YUV420M },
		.fw_versions = h263_fw_versions,
		.fw_versions_count = H263_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED |
			 V4L2_FMT_FLAG_DYN_RESOLUTION,
	}, {
		.pixfmt = V4L2_PIX_FMT_MJPEG,
		.min_buffers = 4,
		.max_buffers = 4,
		/*
		 * HW maxima (see codec_mjpeg.h): width 8192 confirmed by G12B
		 * boundary test (8192x2048 = 5/5 frames, 2026-06-07); height
		 * 2048 is a sharp firmware/PSCALE 2^11 boundary (>2048 silently
		 * produces 0 frames, SM1 2026-04-01).  Effective HW max
		 * 8192x2048 (16.7 MP).  Actual usable max is CMA-constrained at
		 * probe (core->mjpeg_cma_max_pixels) and enforced in
		 * enum_framesizes + try_fmt.
		 */
		.max_width = MJPEG_HW_MAX_WIDTH,
		.max_height = MJPEG_HW_MAX_HEIGHT,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-mjpeg */
		.firmware_path = "meson/vdec/gxl_mjpeg.bin",
		/*
		 * MJPEG PSCALE writes I420 (3 separate planes: Y, U, V).
		 * NV12/NV21 canvases pack U+V into one interleaved plane,
		 * causing V data to overwrite U -- destroying chroma.
		 * Only YUV420M (3-plane) matches the hardware output.
		 */
		.pixfmts_cap = { V4L2_PIX_FMT_YUV420M, 0 },
		.fw_versions = mjpeg_fw_versions,
		.fw_versions_count = MJPEG_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED,
	}, {
		.pixfmt = V4L2_PIX_FMT_MPEG1,
		.min_buffers = 8,
		.max_buffers = 8,
		.max_width = 1920,
		.max_height = 1080,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-mpeg12 */
		.firmware_path = "meson/vdec/gxl_mpeg12.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV21M, V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_NV21, V4L2_PIX_FMT_YUV420M },
		.fw_versions = mpeg12_fw_versions,
		.fw_versions_count = MPEG12_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED,
	}, {
		.pixfmt = V4L2_PIX_FMT_MPEG2,
		.min_buffers = 8,
		.max_buffers = 8,
		.max_width = 1920,
		.max_height = 1080,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-mpeg12 */
		.firmware_path = "meson/vdec/gxl_mpeg12.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV21M, V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_NV21, V4L2_PIX_FMT_YUV420M },
		.fw_versions = mpeg12_fw_versions,
		.fw_versions_count = MPEG12_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED,
	}, {
		.pixfmt = V4L2_PIX_FMT_AVS,
		.min_buffers = 8,
		.max_buffers = 8,
		.max_width = 1920,
		.max_height = 1080,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-avs */
		.firmware_path = "meson/vdec/gxl_avs.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV21M, V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_NV21, V4L2_PIX_FMT_YUV420M },
		.flags = V4L2_FMT_FLAG_COMPRESSED |
			 V4L2_FMT_FLAG_DYN_RESOLUTION,
		.fw_versions = avs_fw_versions,
		.fw_versions_count = AVS_FW_VERSIONS_COUNT,
	},
};

static struct amvdec_format vdec_formats_sm1[] = {
	{
		.pixfmt = V4L2_PIX_FMT_HEVC,
		.min_buffers = 16,
		.max_buffers = 24,
		.max_width = 8192,
		.max_height = 4320,
		.vdec_ops = &vdec_hevc_ops,
		.codec_ops = NULL, /* meson-vdec-hevc */
		.firmware_path = "meson/vdec/sm1_hevc_mmu.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV21M, V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_NV21, V4L2_PIX_FMT_YUV420M, V4L2_PIX_FMT_MESON_FBC08_SCATTER, V4L2_PIX_FMT_MESON_FBC10_SCATTER },
		.fw_versions = hevc_fw_versions,
		.fw_versions_count = HEVC_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED |
			 V4L2_FMT_FLAG_DYN_RESOLUTION,
	}, {
		.pixfmt = V4L2_PIX_FMT_VP9,
		/*
		 * VP9 spec requires at most 8 reference frames. The driver
		 * holds those 8 ref slots plus 1 cur_frame internally, leaving
		 * a thin margin for user-space dequeue. Set min == max so the
		 * v4l2 client allocates the full pool up front and m2m never
		 * runs dry mid-decode at 4K (TODO-VP9-011 root cause).
		 *   12 x (12 MB NV12 + 16 MB FBC) + 48 MB fixed = ~388 MB.
		 * The old 16/24 default required ~614 MB at 4K, exceeding the
		 * 512 MB CMA reservation on SM1 boards.
		 *
		 * WIP 2026-04-20: try 10/10 (9 internal + 1 margin) to free
		 * ~2x per-buffer worth of CMA (~100 MB at 5K) and see if 6K
		 * now fits under 980 MB.  Gate: 4K/5K must still pass 30/30
		 * 2-run deterministic.  If m2m starvation regresses, revert.
		 */
		.min_buffers = 10,
		.max_buffers = 10,
		.max_width = 8192,
		.max_height = 4320,
		.vdec_ops = &vdec_hevc_ops,
		.codec_ops = NULL, /* meson-vdec-vp9 */
		.firmware_path = "meson/vdec/sm1_vp9_mmu.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV21M, V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_NV21, V4L2_PIX_FMT_YUV420M, V4L2_PIX_FMT_MESON_FBC08_SCATTER, V4L2_PIX_FMT_MESON_FBC10_SCATTER },
		.fw_versions = vp9_fw_versions,
		.fw_versions_count = VP9_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED |
			 V4L2_FMT_FLAG_DYN_RESOLUTION,
	}, {
		.pixfmt = V4L2_PIX_FMT_H264,
		.min_buffers = 2,
		.max_buffers = 32,
		/* KI-171: H.264 max DPB; g12a_h264.bin stalls on fewer bufs too */
		.prehdr_min_capture = 16,
		.min_width = 258,
		.max_width = 4096,	/* same as G12B: 8-bit mb_width overflow recovered via SPS pre-parse */
		.max_height = 4096,	/* height has no register overflow; actual limit is mb_total budget */
		.max_mb_total = 36864,	/* g12a_h264.bin firmware budget; same binary as G12B. Checked in  */
				/* pre_queue. 4096x2304 and 2304x4096 verified 2026-04-04.         */
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-h264 */
		.firmware_path = "meson/vdec/g12a_h264.bin",
		/*
		 * SM1 VDEC_1 canvas ignores endian=6 (UV byte swap) and does not
		 * support 3-plane output, same as GXL. Verified 2026-04-04:
		 * NV21 output is byte-identical to NV12; YUV420M stalls at 0 frames.
		 * Restrict to NV12M/NV12 only. (G12B verified working; G12A untested.)
		 */
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV12, 0 },
		.fw_versions = h264_fw_versions,
		.fw_versions_count = H264_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED |
			 V4L2_FMT_FLAG_DYN_RESOLUTION,
	}, {
		/*
		 * H.264 stateless (V4L2 Request API). Uses libre_sm1_h264.bin
		 * firmware (byte-identical to libre_g12b_h264.bin; per-platform
		 * path preserves convention) with IMEM-overlay per
		 * h264-firmware-design.md §11 (3 active overlays:
		 * HEADER/DATA/SLICE). CABAC is HW per 2026-04-22 audit
		 * (cabac_step_2 block in POWER_CTL_VLD).
		 */
		.pixfmt = V4L2_PIX_FMT_H264_SLICE,
		.min_buffers = 4,
		.max_buffers = 17,		/* up to 16 refs + 1 output */
		.min_width = 258,
		.max_width = 4096,
		.max_height = 4096,
		.max_mb_total = 36864,		/* Level 5.1 MaxFS */
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL,		/* meson-vdec-h264-libre */
		.firmware_path = "meson/vdec/libre_sm1_h264.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV12, 0 },
		.fw_versions = h264_fw_versions,
		.fw_versions_count = H264_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED |
			 V4L2_FMT_FLAG_DYN_RESOLUTION,
	}, {
		/*
		 * H.264 MVC on SM1 (S905D3) -- uses sm1_h264_mvc.bin.
		 * BSP source: firmware_drv.c:fw_data_filter() -- SM1 exact match.
		 * max_buffers=34: driver reports min_buffers_for_capture=30,
		 * so cap must be >= 30+4 margin to allow decode to proceed.
		 */
		.pixfmt = V4L2_PIX_FMT_H264_MVC,
		.min_buffers = 4,
		.max_buffers = 34,
		.max_width = 1920,
		.max_height = 1088,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-h264-mvc */
		.firmware_path = "meson/vdec/sm1_h264_mvc.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV12, 0 },
		.fw_versions = h264_mvc_fw_versions,
		.fw_versions_count = H264_MVC_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED |
			 V4L2_FMT_FLAG_DYN_RESOLUTION,
	}, {
		.pixfmt = V4L2_PIX_FMT_MPEG4,
		.min_buffers = 8,
		.max_buffers = 8,
		.max_width = 1920,
		.max_height = 1080,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-mpeg4 */
		.firmware_path = "meson/vdec/sm1_mpeg4_5.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV12, 0 }, /* SM1 canvas: NV12 only */
		.fw_versions = mpeg4_fw_versions,
		.fw_versions_count = MPEG4_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED |
			 V4L2_FMT_FLAG_DYN_RESOLUTION,
	}, {
		.pixfmt = V4L2_PIX_FMT_H263,
		.min_buffers = 8,
		.max_buffers = 8,
		.max_width = 1920,
		.max_height = 1080,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-mpeg4 */
		.firmware_path = "meson/vdec/sm1_mpeg4_5.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV12, 0 }, /* SM1 canvas: NV12 only */
		.fw_versions = h263_fw_versions,
		.fw_versions_count = H263_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED |
			 V4L2_FMT_FLAG_DYN_RESOLUTION,
	}, {
		.pixfmt = V4L2_PIX_FMT_MJPEG,
		.min_buffers = 4,
		.max_buffers = 4,
		/*
		 * HW maxima (see codec_mjpeg.h): width 8192 confirmed by G12B
		 * boundary test (8192x2048 = 5/5 frames, 2026-06-07); height
		 * 2048 is a sharp firmware/PSCALE 2^11 boundary (>2048 silently
		 * produces 0 frames, SM1 2026-04-01).  Effective HW max
		 * 8192x2048 (16.7 MP).  Actual usable max is CMA-constrained at
		 * probe (core->mjpeg_cma_max_pixels) and enforced in
		 * enum_framesizes + try_fmt.
		 */
		.max_width = MJPEG_HW_MAX_WIDTH,
		.max_height = MJPEG_HW_MAX_HEIGHT,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-mjpeg */
		.firmware_path = "meson/vdec/gxl_mjpeg.bin",
		/*
		 * MJPEG PSCALE writes I420 (3 separate planes: Y, U, V).
		 * NV12/NV21 canvases pack U+V into one interleaved plane,
		 * causing V data to overwrite U -- destroying chroma.
		 * Only YUV420M (3-plane) matches the hardware output.
		 */
		.pixfmts_cap = { V4L2_PIX_FMT_YUV420M, 0 },
		.fw_versions = mjpeg_fw_versions,
		.fw_versions_count = MJPEG_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED,
	}, {
		.pixfmt = V4L2_PIX_FMT_MPEG1,
		.min_buffers = 8,
		.max_buffers = 8,
		.max_width = 1920,
		.max_height = 1080,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-mpeg12 */
		.firmware_path = "meson/vdec/gxl_mpeg12.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV12, 0 }, /* SM1 canvas: NV12 only */
		.fw_versions = mpeg12_fw_versions,
		.fw_versions_count = MPEG12_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED,
	}, {
		.pixfmt = V4L2_PIX_FMT_MPEG2,
		.min_buffers = 8,
		.max_buffers = 8,
		.max_width = 1920,
		.max_height = 1080,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-mpeg12 */
		.firmware_path = "meson/vdec/gxl_mpeg12.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV12, 0 }, /* SM1 canvas: NV12 only */
		.fw_versions = mpeg12_fw_versions,
		.fw_versions_count = MPEG12_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED,
	}, {
		.pixfmt = V4L2_PIX_FMT_MPEG2_SLICE,
		/*
		 * NON-ZERO-COPY bridge (codec_mpeg12_libre.c): the firmware's 8
		 * canvases are bridge-owned SCRATCH buffers, decoupled from the
		 * V4L2 CAPTURE pool (each decoded scratch canvas is COPIED to a
		 * V4L2 buffer on display).  So the V4L2 CAPTURE count is NOT
		 * capped at the firmware's 8 canvases -- a VA-API consumer whose
		 * pool is 9 surfaces (ffmpeg-vaapi MPEG-2) now fits.
		 */
		.min_buffers = 4,
		.max_buffers = 32,
		.max_width = 1920,
		.max_height = 1080,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-mpeg12-libre */
		.firmware_path = "meson/vdec/gxl_mpeg12.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV12, 0 }, /* SM1 canvas: NV12 only */
		.fw_versions = mpeg12_fw_versions,
		.fw_versions_count = MPEG12_FW_VERSIONS_COUNT,
		.flags = V4L2_FMT_FLAG_COMPRESSED,
	}, {
		/*
		 * AVS on SM1 -- uses sm1_avs.bin (incompatible with GXL firmware).
		 * SM1 VDEC_1 canvas ignores endian=6 (UV byte swap), NV12 only.
		 */
		.pixfmt = V4L2_PIX_FMT_AVS,
		.min_buffers = 8,
		.max_buffers = 8,
		.max_width = 1920,
		.max_height = 1080,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = NULL, /* meson-vdec-avs */
		.firmware_path = "meson/vdec/sm1_avs.bin",
		.pixfmts_cap = { V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV12, 0 },
		.flags = V4L2_FMT_FLAG_COMPRESSED |
			 V4L2_FMT_FLAG_DYN_RESOLUTION,
		.fw_versions = avs_fw_versions,
		.fw_versions_count = AVS_FW_VERSIONS_COUNT,
	},
};

const struct vdec_platform vdec_platform_gxbb = {
	.formats = vdec_formats_gxbb,
	.num_formats = ARRAY_SIZE(vdec_formats_gxbb),
	.revision = VDEC_REVISION_GXBB,
};

const struct vdec_platform vdec_platform_gxl = {
	.formats = vdec_formats_gxl,
	.num_formats = ARRAY_SIZE(vdec_formats_gxl),
	.revision = VDEC_REVISION_GXL,
};

const struct vdec_platform vdec_platform_gxlx = {
	.formats = vdec_formats_gxlx,
	.num_formats = ARRAY_SIZE(vdec_formats_gxlx),
	.revision = VDEC_REVISION_GXLX,
};

const struct vdec_platform vdec_platform_gxm = {
	.formats = vdec_formats_gxm,
	.num_formats = ARRAY_SIZE(vdec_formats_gxm),
	.revision = VDEC_REVISION_GXM,
};

const struct vdec_platform vdec_platform_g12a = {
	.formats = vdec_formats_g12a,
	.num_formats = ARRAY_SIZE(vdec_formats_g12a),
	.revision = VDEC_REVISION_G12A,
};

const struct vdec_platform vdec_platform_g12b = {
	.formats = vdec_formats_g12b,
	.num_formats = ARRAY_SIZE(vdec_formats_g12b),
	.revision = VDEC_REVISION_G12B,
};

const struct vdec_platform vdec_platform_sm1 = {
	.formats = vdec_formats_sm1,
	.num_formats = ARRAY_SIZE(vdec_formats_sm1),
	.revision = VDEC_REVISION_SM1,
};

MODULE_FIRMWARE("meson/vdec/g12a_h264.bin");
MODULE_FIRMWARE("meson/vdec/g12a_hevc_mmu.bin");
/* g12a_vp9.bin removed: G12A/G12B VP9 now use sm1_vp9_mmu.bin (VP9-025). */
MODULE_FIRMWARE("meson/vdec/gxbb_h264.bin");
MODULE_FIRMWARE("meson/vdec/gxl_h263.bin");
MODULE_FIRMWARE("meson/vdec/gxl_h264.bin");
MODULE_FIRMWARE("meson/vdec/gxl_hevc_mmu.bin");
MODULE_FIRMWARE("meson/vdec/gxl_mjpeg.bin");
MODULE_FIRMWARE("meson/vdec/gxl_mpeg12.bin");
MODULE_FIRMWARE("meson/vdec/gxl_mpeg4_5.bin");
MODULE_FIRMWARE("meson/vdec/gxl_vp9_mmu.bin");
MODULE_FIRMWARE("meson/vdec/gxm_h264.bin");
MODULE_FIRMWARE("meson/vdec/sm1_hevc_mmu.bin");
MODULE_FIRMWARE("meson/vdec/sm1_mpeg4_5.bin");
MODULE_FIRMWARE("meson/vdec/sm1_vp9_mmu.bin");
/* H.264 MVC firmware -- one per SoC family (GXL/GXM/SM1); G12A/G12B use gxm variant */
MODULE_FIRMWARE("meson/vdec/gxl_h264_mvc.bin");
MODULE_FIRMWARE("meson/vdec/gxm_h264_mvc.bin");
MODULE_FIRMWARE("meson/vdec/sm1_h264_mvc.bin");
/* AVS firmware -- GXL/GXM/G12A/G12B share gxl variant; SM1 requires its own */
MODULE_FIRMWARE("meson/vdec/gxl_avs.bin");
MODULE_FIRMWARE("meson/vdec/sm1_avs.bin");
