// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Wave420L firmware known-versions table.
 *
 * One entry for monet.bin, the Chips&Media Wave420L BIT processor firmware.
 * Provenance: Amlogic BSP, firmware-amlogic dpkg, Chips&Media Wave420L
 * BIT processor firmware.
 */

#include <linux/build_bug.h>
#include "codec_wave420l_fw.h"

const struct amvdec_fw_version wave420l_fw_versions[] = {
	{
		/* monet.bin: Wave420L HEVC encoder BIT processor firmware. */
		.label = "BSP (monet.bin, Wave420L HEVC)",
		.provenance = "Amlogic BSP, firmware-amlogic dpkg, Chips&Media Wave420L BIT processor firmware",
		.size = 147120,
		.sha256 = { 0xd1, 0xad, 0xb3, 0x8a, 0x81, 0xe6, 0x33, 0x12,
			    0x08, 0x31, 0x7a, 0x06, 0x9e, 0x06, 0x82, 0xf5,
			    0xb0, 0xd0, 0x51, 0x0f, 0xe5, 0xfb, 0xc6, 0x88,
			    0x25, 0x19, 0xe1, 0x85, 0xbf, 0x07, 0xae, 0xdd },
		.quirks = 0,
		.latest = true,
	},
};
const size_t wave420l_fw_versions_count = ARRAY_SIZE(wave420l_fw_versions);

/* Catch mismatches between codec_wave420l_fw.h define and actual table size */
static_assert(ARRAY_SIZE(wave420l_fw_versions) == WAVE420L_FW_VERSIONS_COUNT,
	      "WAVE420L_FW_VERSIONS_COUNT in codec_wave420l_fw.h doesn't match wave420l_fw_versions[] size");
