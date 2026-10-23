// SPDX-License-Identifier: GPL-2.0+
/*
 * MPEG-1/2 firmware known-versions table.
 *
 * Each entry covers one known MPEG-1/2 firmware binary identified by
 * SHA256 + size.  The .quirks field uses the AMVDEC_FW_QUIRK_MPEG12_*
 * bits defined in codec_mpeg12_fw.h.
 *
 * SHA256 values verified on 2026-04-16 against deployed NFS rootfs
 * (board 8abde1aa5c2c, /lib/firmware/meson/vdec/).
 *
 * MPEG-1/2 uses the VDEC_1 engine.  One binary covers all supported SoCs:
 *   gxl_mpeg12.bin  -- GXL/GXM/G12A/G12B/SM1/GXBB (single shared binary)
 */

#include <linux/build_bug.h>
#include "codec_mpeg12_fw.h"
#include "vdec_helpers.h"

const struct amvdec_fw_version mpeg12_fw_versions[] = {
	{
		/* gxl_mpeg12.bin: deployed on all supported SoCs.
		 * Single binary shared across GXL/GXM/G12A/G12B/SM1/GXBB.
		 */
		.label = "BSP (gxl_mpeg12.bin 16K)",
		.provenance = "Amlogic BSP, firmware-amlogic dpkg",
		.size = 16384,
		.sha256 = { 0x4a, 0x85, 0x13, 0x02, 0x78, 0x5a, 0x48, 0xe8,
			    0x4c, 0xb8, 0x46, 0xe5, 0xe9, 0x74, 0xa4, 0xf5,
			    0x1a, 0x4c, 0xa8, 0x7c, 0x52, 0xd9, 0x61, 0xaa,
			    0xb4, 0x30, 0x7b, 0x1c, 0x91, 0xda, 0xf2, 0xb2 },
		.quirks = 0,
		.latest = true,
	},
};
const size_t mpeg12_fw_versions_count = ARRAY_SIZE(mpeg12_fw_versions);

/* Catch mismatches between codec_mpeg12_fw.h define and actual table size */
static_assert(ARRAY_SIZE(mpeg12_fw_versions) == MPEG12_FW_VERSIONS_COUNT,
	      "MPEG12_FW_VERSIONS_COUNT in codec_mpeg12_fw.h doesn't match mpeg12_fw_versions[] size");
