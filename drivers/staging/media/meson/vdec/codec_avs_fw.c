// SPDX-License-Identifier: GPL-2.0+
/*
 * AVS firmware known-versions table.
 *
 * Each entry covers one known AVS firmware binary identified by SHA256 + size.
 * The .quirks field uses the AMVDEC_FW_QUIRK_AVS_* bits defined in codec_avs.h.
 *
 * SHA256 values computed from Amlogic BSP V2603 V2.5 (firmware-amlogic dpkg):
 *   gxl_avs.bin      16384  2129b5ad3025662fea3dafc1e2cae05188be3bdba3c7bcca52032f1401ab4e72
 *   sm1_avs.bin      12288  a168af959bc778d749a7fca32a33d7c5fc4b34eed41c51b0c4d7ff8c01c6488f
 *   gxl_avs_multi.bin 20480 26d09014614757111425 39f9709ec105bea3c8a4c534e81c61c881757d9a5cc3
 *   sm1_avs_multi.bin 16384  cbea00eb22d30970322b97d422da70052d152c01f4488d7d28408659785672a6
 *   gxm_avs_multi.bin 16384  dac8b47b53cc9d4d6c0a96bbcb41ea1a3a87c5daa8e6c43a102c5430e4a3f6ef
 */

#include <linux/build_bug.h>
#include "codec_avs.h"
#include "vdec_helpers.h"

const struct amvdec_fw_version avs_fw_versions[] = {
	{
		/* gxl_avs.bin: single-instance AVS for GXL/GXM/G12A/G12B */
		.label = "BSP V2603 V2.5 (gxl_avs.bin)",
		.provenance = "Amlogic BSP, firmware-amlogic dpkg",
		.size = 16384,
		.sha256 = { 0x21, 0x29, 0xb5, 0xad, 0x30, 0x25, 0x66, 0x2f,
			    0xea, 0x3d, 0xaf, 0xc1, 0xe2, 0xca, 0xe0, 0x51,
			    0x88, 0xbe, 0x3b, 0xdb, 0xa3, 0xc7, 0xbc, 0xca,
			    0x52, 0x03, 0x2f, 0x14, 0x01, 0xab, 0x4e, 0x72 },
		.quirks = AMVDEC_FW_QUIRK_AVS_SINGLE_INSTANCE |
			  AMVDEC_FW_QUIRK_AVS_VLD_BUSY_PATCH,
		.latest = true,
	},
	{
		/* sm1_avs.bin: single-instance AVS for SM1 (incompatible with GXL) */
		.label = "BSP V2603 V2.5 (sm1_avs.bin)",
		.provenance = "Amlogic BSP, firmware-amlogic dpkg",
		.size = 12288,
		.sha256 = { 0xa1, 0x68, 0xaf, 0x95, 0x9b, 0xc7, 0x78, 0xd7,
			    0x49, 0xa7, 0xfc, 0xa3, 0x2a, 0x33, 0xd7, 0xc5,
			    0xfc, 0x4b, 0x34, 0xee, 0xd4, 0x1c, 0x51, 0xb0,
			    0xc4, 0xd7, 0xff, 0x8c, 0x01, 0xc6, 0x48, 0x8f },
		.quirks = AMVDEC_FW_QUIRK_AVS_SINGLE_INSTANCE,
		.latest = true,
	},
	{
		/* gxl_avs_multi.bin: multi-instance AVS for GXL/G12A/G12B */
		.label = "BSP V2603 V2.5 (gxl_avs_multi.bin)",
		.provenance = "Amlogic BSP, firmware-amlogic dpkg",
		.size = 20480,
		.sha256 = { 0x26, 0xd0, 0x90, 0x14, 0x61, 0x47, 0x57, 0x11,
			    0x14, 0x25, 0x39, 0xf9, 0x70, 0x9e, 0xc1, 0x05,
			    0xbe, 0xa3, 0xc8, 0xa4, 0xc5, 0x34, 0xe8, 0x1c,
			    0x61, 0xc8, 0x81, 0x75, 0x7d, 0x9a, 0x5c, 0xc3 },
		.quirks = AMVDEC_FW_QUIRK_AVS_MULTI_INSTANCE,
		.latest = true,
	},
	{
		/* sm1_avs_multi.bin: multi-instance AVS for SM1 */
		.label = "BSP V2603 V2.5 (sm1_avs_multi.bin)",
		.provenance = "Amlogic BSP, firmware-amlogic dpkg",
		.size = 16384,
		.sha256 = { 0xcb, 0xea, 0x00, 0xeb, 0x22, 0xd3, 0x09, 0x70,
			    0x32, 0x2b, 0x97, 0xd4, 0x22, 0xda, 0x70, 0x05,
			    0x2d, 0x15, 0x2c, 0x01, 0xf4, 0x48, 0x8d, 0x7d,
			    0x28, 0x40, 0x86, 0x59, 0x78, 0x72, 0x56, 0xa6 },
		.quirks = AMVDEC_FW_QUIRK_AVS_MULTI_INSTANCE,
		.latest = true,
	},
	{
		/* gxm_avs_multi.bin: multi-instance AVS for GXM */
		.label = "BSP V2603 V2.5 (gxm_avs_multi.bin)",
		.provenance = "Amlogic BSP, firmware-amlogic dpkg",
		.size = 16384,
		.sha256 = { 0xda, 0xc8, 0xb4, 0x7b, 0x53, 0xcc, 0x9d, 0x4d,
			    0x6c, 0x0a, 0x96, 0xbb, 0xcb, 0x41, 0xea, 0x1a,
			    0x3a, 0x87, 0xc5, 0xda, 0xa8, 0xe6, 0xc4, 0x3a,
			    0x10, 0x2c, 0x54, 0x30, 0xe4, 0xa3, 0xf6, 0xef },
		.quirks = AMVDEC_FW_QUIRK_AVS_MULTI_INSTANCE,
		.latest = true,
	},
};
const size_t avs_fw_versions_count = ARRAY_SIZE(avs_fw_versions);

/* Catch mismatches between codec_avs.h define and actual table size */
static_assert(ARRAY_SIZE(avs_fw_versions) == AVS_FW_VERSIONS_COUNT,
	      "AVS_FW_VERSIONS_COUNT in codec_avs.h doesn't match avs_fw_versions[] size");
