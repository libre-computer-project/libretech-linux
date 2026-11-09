// SPDX-License-Identifier: GPL-2.0+
/*
 * HCODEC encoder firmware known-versions table.
 *
 * Covers all three HCODEC engine firmware variants loaded by hcodec.c
 * (H.264) and jpegenc.c (JPEG).  All entries are from the Amlogic BSP
 * firmware-amlogic dpkg.
 *
 * SHA256 values correspond to:
 *   entry 1: gxl_h264_enc.bin   (GXL H.264, 24576 bytes)
 *   entry 2: ga_h264_enc_cabac.bin (G12A+ H.264 CABAC, 24576 bytes)
 *   entry 3: gxl_jpeg_enc.bin   (all SoCs JPEG, 16384 bytes)
 */

#include <linux/build_bug.h>
#include "codec_hcodec_fw.h"
#include "vdec_helpers.h"

const struct amvdec_fw_version hcodec_fw_versions[] = {
	{
		/* gxl_h264_enc.bin: GXL H.264 encoder firmware. */
		.label = "BSP (gxl_h264_enc.bin, GXL)",
		.provenance = "Amlogic BSP, firmware-amlogic dpkg",
		.size = 24576,
		.sha256 = { 0xed, 0xc5, 0x96, 0xf8, 0x1d, 0x0f, 0xf3, 0x34,
			    0x59, 0x5e, 0xd8, 0x41, 0x72, 0xc5, 0xcc, 0xe9,
			    0x27, 0x69, 0xd0, 0x57, 0x29, 0x99, 0xf1, 0xf7,
			    0x35, 0xc0, 0xef, 0x62, 0x69, 0xa9, 0x09, 0xf6 },
		.quirks = 0,
		.latest = false,
	},
	{
		/* ga_h264_enc_cabac.bin: G12A+ H.264 CABAC encoder firmware.
		 * Used on G12A, G12B, and SM1 SoCs.
		 */
		.label = "BSP (ga_h264_enc_cabac.bin, G12A+)",
		.provenance = "Amlogic BSP, firmware-amlogic dpkg",
		.size = 24576,
		.sha256 = { 0xb3, 0xf1, 0xc3, 0xc8, 0xe3, 0xd2, 0x6b, 0x1b,
			    0x1a, 0x54, 0x9c, 0xe5, 0x16, 0x27, 0xad, 0x93,
			    0x13, 0xc7, 0xf5, 0x7d, 0x96, 0xfa, 0x23, 0xfd,
			    0xbc, 0x64, 0x84, 0xd9, 0x76, 0xd2, 0xde, 0xab },
		.quirks = 0,
		.latest = true,
	},
	{
		/* gxl_jpeg_enc.bin: JPEG encoder firmware, used on all SoCs. */
		.label = "BSP (gxl_jpeg_enc.bin, all)",
		.provenance = "Amlogic BSP, firmware-amlogic dpkg",
		.size = 16384,
		.sha256 = { 0x4c, 0xbb, 0x8a, 0x9c, 0x25, 0x03, 0x8d, 0x5d,
			    0x96, 0x8f, 0x2e, 0x79, 0xde, 0x54, 0x4a, 0x46,
			    0x9a, 0xec, 0x82, 0x0b, 0x47, 0x0d, 0x91, 0xf0,
			    0xd3, 0xcd, 0x24, 0xd1, 0xe2, 0x10, 0x22, 0x3f },
		.quirks = 0,
		.latest = true,
	},
};
const size_t hcodec_fw_versions_count = ARRAY_SIZE(hcodec_fw_versions);

/* Catch mismatches between codec_hcodec_fw.h define and actual table size */
static_assert(ARRAY_SIZE(hcodec_fw_versions) == HCODEC_FW_VERSIONS_COUNT,
	      "HCODEC_FW_VERSIONS_COUNT in codec_hcodec_fw.h doesn't match hcodec_fw_versions[] size");
