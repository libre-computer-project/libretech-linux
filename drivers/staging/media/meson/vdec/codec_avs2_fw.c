// SPDX-License-Identifier: GPL-2.0+
/*
 * AVS2 firmware known-versions table.
 *
 * Each entry covers one known AVS2 firmware binary identified by
 * SHA256 + size.  The .quirks field uses the AMVDEC_FW_QUIRK_AVS2_*
 * bits defined in codec_avs2_fw.h.
 *
 * SHA256 values verified on 2026-04-16 against deployed NFS rootfs
 * (board 8abde1aa5c2c, /lib/firmware/meson/vdec/).
 *
 * AVS2 uses the HEVC engine.  One binary is deployed:
 *   g12a_avs2_mmu.bin -- G12A/G12B/SM1 (same binary; no GXL support)
 */

#include <linux/build_bug.h>
#include "codec_avs2_fw.h"
#include "vdec_helpers.h"

const struct amvdec_fw_version avs2_fw_versions[] = {
	{
		/* g12a_avs2_mmu.bin: deployed on G12A/G12B/SM1 NFS rootfs. */
		.label = "BSP (g12a_avs2_mmu.bin 28K)",
		.provenance = "Amlogic BSP, firmware-amlogic dpkg",
		.size = 28672,
		.sha256 = { 0xee, 0x7b, 0xbc, 0x0d, 0x62, 0x29, 0x70, 0x10,
			    0x3c, 0x00, 0x71, 0x61, 0x20, 0xa2, 0x4b, 0x6f,
			    0x5f, 0x5f, 0x6b, 0x26, 0x29, 0x63, 0x67, 0x09,
			    0xbd, 0xfd, 0x1f, 0x3d, 0xbd, 0x79, 0x7f, 0x0e },
		.quirks = 0,
		.latest = true,
	},
};
const size_t avs2_fw_versions_count = ARRAY_SIZE(avs2_fw_versions);

/* Catch mismatches between codec_avs2_fw.h define and actual table size */
static_assert(ARRAY_SIZE(avs2_fw_versions) == AVS2_FW_VERSIONS_COUNT,
	      "AVS2_FW_VERSIONS_COUNT in codec_avs2_fw.h doesn't match avs2_fw_versions[] size");
