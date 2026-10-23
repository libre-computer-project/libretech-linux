// SPDX-License-Identifier: GPL-2.0+
/*
 * H.264 MVC firmware known-versions table.
 *
 * Each entry covers one known H.264 MVC firmware binary identified by
 * SHA256 + size.  The .quirks field uses the AMVDEC_FW_QUIRK_H264_MVC_*
 * bits defined in codec_h264_mvc_fw.h.
 *
 * SHA256 values re-verified on 2026-04-17 against deployed NFS rootfs
 * (board 8602c0c66f05, /lib/firmware/meson/vdec/).  GXL and GXM entries
 * had transcription errors (1-byte off) in the initial commit; corrected here.
 *
 * H.264 MVC uses the VDEC_1 engine.  Three SoC-specific binaries are deployed:
 *   gxl_h264_mvc.bin  -- GXL
 *   gxm_h264_mvc.bin  -- GXM; also used by G12A and G12B (highest CPU ID <= G12B)
 *   sm1_h264_mvc.bin  -- SM1 (exact match in BSP firmware_drv.c:fw_data_filter())
 */

#include <linux/build_bug.h>
#include "codec_h264_mvc_fw.h"
#include "vdec_helpers.h"

const struct amvdec_fw_version h264_mvc_fw_versions[] = {
	{
		/* gxl_h264_mvc.bin: deployed on GXL NFS rootfs. */
		.label = "BSP (gxl_h264_mvc.bin 28K)",
		.provenance = "Amlogic BSP, firmware-amlogic dpkg",
		.size = 28672,
		.sha256 = { 0xdd, 0x9e, 0x25, 0x22, 0xc8, 0x14, 0xde, 0xb1,
			    0x96, 0xc2, 0x28, 0x2c, 0xff, 0x35, 0x1c, 0xf7,
			    0x13, 0x16, 0x7d, 0xe1, 0x0e, 0xb7, 0x76, 0x13,
			    0xf6, 0xc2, 0x0c, 0x42, 0x1d, 0xd8, 0x2f, 0x19 },
		.quirks = 0,
		.latest = true,
	},
	{
		/* gxm_h264_mvc.bin: deployed on GXM/G12A/G12B NFS rootfs.
		 * G12A and G12B have no dedicated MVC firmware; gxm is the
		 * closest match (highest CPU ID <= G12A=0x28, G12B=0x29).
		 * BSP source: firmware_drv.c:fw_data_filter().
		 */
		.label = "BSP (gxm_h264_mvc.bin 28K)",
		.provenance = "Amlogic BSP, firmware-amlogic dpkg",
		.size = 28672,
		.sha256 = { 0xf4, 0x98, 0x18, 0xab, 0x28, 0x1c, 0x7a, 0xa0,
			    0x5e, 0x3f, 0x76, 0xef, 0x2a, 0x71, 0x15, 0x3d,
			    0xc4, 0x1c, 0xd4, 0xd5, 0xb6, 0x87, 0xf4, 0x66,
			    0xd2, 0x13, 0xea, 0xd6, 0xea, 0x6a, 0xce, 0x0f },
		.quirks = 0,
		.latest = true,
	},
	{
		/* sm1_h264_mvc.bin: deployed on SM1 NFS rootfs.
		 * BSP source: firmware_drv.c:fw_data_filter() -- SM1 exact match.
		 */
		.label = "BSP (sm1_h264_mvc.bin 28K)",
		.provenance = "Amlogic BSP, firmware-amlogic dpkg",
		.size = 28672,
		.sha256 = { 0x0e, 0xf2, 0xa0, 0x60, 0x0d, 0xbf, 0x7c, 0x6c,
			    0xdd, 0x5c, 0xec, 0x92, 0xa6, 0xf1, 0x37, 0xeb,
			    0x4d, 0x89, 0x98, 0x1d, 0xc7, 0xe9, 0x65, 0x44,
			    0x19, 0x5a, 0x3a, 0x74, 0x1e, 0xb3, 0x85, 0xed },
		.quirks = 0,
		.latest = true,
	},
};
const size_t h264_mvc_fw_versions_count = ARRAY_SIZE(h264_mvc_fw_versions);

/* Catch mismatches between codec_h264_mvc_fw.h define and actual table size */
static_assert(ARRAY_SIZE(h264_mvc_fw_versions) == H264_MVC_FW_VERSIONS_COUNT,
	      "H264_MVC_FW_VERSIONS_COUNT in codec_h264_mvc_fw.h doesn't match h264_mvc_fw_versions[] size");
