// SPDX-License-Identifier: GPL-2.0+
/*
 * MPEG-4 firmware known-versions table.
 *
 * Each entry covers one known MPEG-4 firmware binary identified by
 * SHA256 + size.  The .quirks field uses the AMVDEC_FW_QUIRK_MPEG4_*
 * bits defined in codec_mpeg4_fw.h.
 *
 * SHA256 values verified on 2026-04-16 against deployed NFS rootfs
 * (board 8abde1aa5c2c, /lib/firmware/meson/vdec/).
 *
 * MPEG-4 uses the VDEC_1 engine.  Two SoC-specific binaries are deployed:
 *   gxl_mpeg4_5.bin  -- GXL/G12A/G12B (works correctly)
 *   sm1_mpeg4_5.bin  -- SM1 (present but produces 0 frames, KI-005)
 */

#include <linux/build_bug.h>
#include "codec_mpeg4_fw.h"
#include "vdec_helpers.h"

const struct amvdec_fw_version mpeg4_fw_versions[] = {
	{
		/* gxl_mpeg4_5.bin: deployed on GXL/G12A/G12B NFS rootfs. */
		.label = "BSP (gxl_mpeg4_5.bin 16K)",
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
		/* sm1_mpeg4_5.bin: deployed on SM1 NFS rootfs.
		 * KI-005 (2026-04-16): firmware is recognized but produces
		 * 0 decoded frames on SM1 hardware.  AMVDEC_FW_QUIRK_MPEG4_SM1_STALL
		 * is an informational marker only -- not wired to any driver path.
		 */
		.label = "BSP (sm1_mpeg4_5.bin 16K)",
		.provenance = "Amlogic BSP, firmware-amlogic dpkg",
		.size = 16384,
		.sha256 = { 0xa5, 0x1f, 0x2c, 0xf9, 0x73, 0x93, 0x45, 0x7f,
			    0x0d, 0x5e, 0x08, 0x6d, 0x1c, 0x1a, 0x07, 0xa3,
			    0x34, 0xee, 0x6b, 0xcc, 0x87, 0xa8, 0x59, 0x5a,
			    0xed, 0xe5, 0x19, 0x3b, 0xaf, 0x6e, 0xbd, 0x32 },
		.quirks = AMVDEC_FW_QUIRK_MPEG4_SM1_STALL,
		.latest = true,
	},
};
const size_t mpeg4_fw_versions_count = ARRAY_SIZE(mpeg4_fw_versions);

/* Catch mismatches between codec_mpeg4_fw.h define and actual table size */
static_assert(ARRAY_SIZE(mpeg4_fw_versions) == MPEG4_FW_VERSIONS_COUNT,
	      "MPEG4_FW_VERSIONS_COUNT in codec_mpeg4_fw.h doesn't match mpeg4_fw_versions[] size");
