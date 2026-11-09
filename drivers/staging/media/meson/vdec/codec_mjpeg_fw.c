// SPDX-License-Identifier: GPL-2.0+
/*
 * MJPEG firmware known-versions table.
 *
 * Each entry covers one known MJPEG firmware binary identified by
 * SHA256 + size.  The .quirks field uses the AMVDEC_FW_QUIRK_MJPEG_*
 * bits defined in codec_mjpeg_fw.h.
 *
 * SHA256 values verified on 2026-04-16 against deployed NFS rootfs
 * (board 8abde1aa5c2c, /lib/firmware/meson/vdec/).
 *
 * MJPEG uses the VDEC_1 engine.  One binary covers all supported SoCs:
 *   gxl_mjpeg.bin  -- GXL/G12A/G12B/SM1 (single shared binary)
 */

#include <linux/build_bug.h>
#include "codec_mjpeg_fw.h"
#include "vdec_helpers.h"

const struct amvdec_fw_version mjpeg_fw_versions[] = {
	{
		/* gxl_mjpeg.bin: deployed on GXL/G12A/G12B/SM1 NFS rootfs.
		 * Single binary shared across all supported SoCs.
		 */
		.label = "BSP (gxl_mjpeg.bin 16K)",
		.provenance = "Amlogic BSP, firmware-amlogic dpkg",
		.size = 16384,
		.sha256 = { 0x8a, 0x49, 0x97, 0xcd, 0x0c, 0x5c, 0x5c, 0xbe,
			    0x67, 0x8c, 0x75, 0xc4, 0x5a, 0xe6, 0x6a, 0x64,
			    0xd1, 0xf9, 0xc1, 0xa0, 0xba, 0x2d, 0x26, 0xd1,
			    0x37, 0xf2, 0x4d, 0x21, 0xf0, 0xab, 0x6d, 0x53 },
		.quirks = 0,
		.latest = true,
	},
};
const size_t mjpeg_fw_versions_count = ARRAY_SIZE(mjpeg_fw_versions);

/* Catch mismatches between codec_mjpeg_fw.h define and actual table size */
static_assert(ARRAY_SIZE(mjpeg_fw_versions) == MJPEG_FW_VERSIONS_COUNT,
	      "MJPEG_FW_VERSIONS_COUNT in codec_mjpeg_fw.h doesn't match mjpeg_fw_versions[] size");
