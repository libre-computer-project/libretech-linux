// SPDX-License-Identifier: GPL-2.0+
/*
 * H.263 firmware known-versions table.
 *
 * Each entry covers one known H.263 firmware binary identified by
 * SHA256 + size.  The .quirks field uses the AMVDEC_FW_QUIRK_H263_*
 * bits defined in codec_h263_fw.h.
 *
 * SHA256 values verified on 2026-04-16 against deployed NFS rootfs
 * (board 8abde1aa5c2c, /lib/firmware/meson/vdec/).
 *
 * H.263 uses the VDEC_1 engine.  Three firmware paths are used across
 * supported SoCs (H.263 shares MPEG-4 firmware on G12A/G12B/SM1):
 *   gxl_h263.bin      -- GXL (dedicated H.263 firmware, same bytes as sm1_h263.bin)
 *   gxl_mpeg4_5.bin   -- G12A/G12B (shared with MPEG-4 on those SoCs)
 *   sm1_mpeg4_5.bin   -- SM1 (shared with MPEG-4; KI-005: 0 frames on SM1 MPEG-4,
 *                        H.263 not yet validated on SM1)
 */

#include <linux/build_bug.h>
#include "codec_h263_fw.h"
#include "vdec_helpers.h"

const struct amvdec_fw_version h263_fw_versions[] = {
	{
		/* gxl_h263.bin: deployed on GXL NFS rootfs.
		 * Byte-identical to sm1_h263.bin (same SHA256).
		 */
		.label = "BSP (gxl_h263.bin 16K)",
		.provenance = "Amlogic BSP, firmware-amlogic dpkg",
		.size = 16384,
		.sha256 = { 0x23, 0x36, 0xaf, 0x11, 0x2b, 0x49, 0x8c, 0x97,
			    0x1b, 0xef, 0x9b, 0xbe, 0x46, 0xe0, 0x5f, 0xfb,
			    0x5b, 0xeb, 0xda, 0xaa, 0xaf, 0xcb, 0x70, 0x33,
			    0x81, 0x62, 0x26, 0x7d, 0x8a, 0x40, 0x66, 0xcd },
		.quirks = 0,
		.latest = true,
	},
	{
		/* gxl_mpeg4_5.bin: H.263 on G12A/G12B reuses the MPEG-4
		 * firmware.  BSP firmware_drv.c assigns the MPEG-4 binary
		 * to the H.263 decoder on these SoCs.
		 */
		.label = "BSP (gxl_mpeg4_5.bin 16K -- H.263 on G12A/G12B)",
		.provenance = "Amlogic BSP, firmware-amlogic dpkg",
		.size = 16384,
		.sha256 = { 0x10, 0x01, 0x13, 0x3d, 0x64, 0xb8, 0x92, 0x2e,
			    0x80, 0x95, 0x95, 0x2e, 0xf3, 0xbb, 0x62, 0xe1,
			    0x4f, 0x78, 0xa0, 0xcc, 0xd1, 0xaf, 0x9c, 0x65,
			    0x9b, 0xa2, 0x81, 0x24, 0xac, 0x1c, 0x9d, 0xc5 },
		.quirks = 0,
		.latest = true,
	},
	{
		/* sm1_mpeg4_5.bin: H.263 on SM1 reuses the SM1 MPEG-4
		 * firmware.  Same KI-005 stall note as MPEG-4 on SM1 applies
		 * (0 decoded frames seen for MPEG-4; H.263 not separately
		 * validated on SM1 as of 2026-04-16).
		 */
		.label = "BSP (sm1_mpeg4_5.bin 16K -- H.263 on SM1)",
		.provenance = "Amlogic BSP, firmware-amlogic dpkg",
		.size = 16384,
		.sha256 = { 0xa5, 0x1f, 0x2c, 0xf9, 0x73, 0x93, 0x45, 0x7f,
			    0x0d, 0x5e, 0x08, 0x6d, 0x1c, 0x1a, 0x07, 0xa3,
			    0x34, 0xee, 0x6b, 0xcc, 0x87, 0xa8, 0x59, 0x5a,
			    0xed, 0xe5, 0x19, 0x3b, 0xaf, 0x6e, 0xbd, 0x32 },
		.quirks = 0,
		.latest = true,
	},
};
const size_t h263_fw_versions_count = ARRAY_SIZE(h263_fw_versions);

/* Catch mismatches between codec_h263_fw.h define and actual table size */
static_assert(ARRAY_SIZE(h263_fw_versions) == H263_FW_VERSIONS_COUNT,
	      "H263_FW_VERSIONS_COUNT in codec_h263_fw.h doesn't match h263_fw_versions[] size");
