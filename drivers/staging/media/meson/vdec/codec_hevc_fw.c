// SPDX-License-Identifier: GPL-2.0+
/*
 * HEVC firmware known-versions table.
 *
 * Each entry covers one known HEVC firmware binary identified by
 * SHA256 + size.  The .quirks field uses the AMVDEC_FW_QUIRK_HEVC_*
 * bits defined in codec_hevc_fw.h.
 *
 * SHA256 values verified on 2026-04-16 against deployed NFS rootfs
 * and our internal vdec BSP reference tree.  The upstream 16KB
 * variants for G12A/SM1 and GXL are marked .latest = true.
 */

#include <linux/build_bug.h>
#include "codec_hevc_fw.h"
#include "vdec_helpers.h"

const struct amvdec_fw_version hevc_fw_versions[] = {
	{
		/* g12a_hevc_mmu.bin / sm1_hevc_mmu.bin (same bytes):
		 * upstream linux-firmware variant, 16KB.  Currently deployed
		 * on G12A/G12B/SM1 NFS rootfs.
		 */
		.label = "linux-firmware (g12a/sm1 hevc_mmu 16K)",
		.provenance = "linux-firmware upstream",
		.size = 16384,
		.sha256 = { 0xf9, 0x82, 0x49, 0xd9, 0x58, 0x6a, 0x66, 0xa5,
			    0xe7, 0x47, 0xb3, 0x86, 0xf9, 0xeb, 0xa7, 0x6a,
			    0x67, 0x0e, 0xfb, 0xbc, 0xf0, 0xd7, 0x44, 0x61,
			    0x23, 0x87, 0xfe, 0xbb, 0x3b, 0x2b, 0x0d, 0x2a },
		.quirks = AMVDEC_FW_QUIRK_HEVC_UPSTREAM_PROTO,
		.latest = true,
	},
	{
		/* g12a_hevc_mmu.bin / sm1_hevc_mmu.bin: BSP 36KB variant.
		 * Uses swap protocol that crashes G12A+ on upstream driver
		 * (TASK-003, 2026-04-10).  Recognized here so the driver
		 * does not refuse-to-load if a user switches to BSP firmware,
		 * but will still crash without matching driver-side swap
		 * support.
		 */
		.label = "BSP V2603 V2.5 (g12a/sm1 hevc_mmu 36K)",
		.provenance = "Amlogic BSP V2603 V2.5, firmware-amlogic dpkg",
		.size = 36864,
		.sha256 = { 0xda, 0x56, 0x8d, 0xad, 0x5c, 0x8f, 0x4b, 0xd3,
			    0xff, 0xa1, 0x80, 0xfb, 0x29, 0xfd, 0x1b, 0xd3,
			    0xd9, 0xea, 0x6e, 0x44, 0x68, 0xbe, 0xc6, 0xe9,
			    0x73, 0x0b, 0xa4, 0x50, 0xf0, 0x05, 0xb6, 0x93 },
		.quirks = AMVDEC_FW_QUIRK_HEVC_BSP_PROTO |
			  AMVDEC_FW_QUIRK_HEVC_BSP_SWAP_CRASH_G12A |
			  AMVDEC_FW_QUIRK_HEVC_BSP_36K,
		.latest = false,
	},
	{
		/* gxl_hevc_mmu.bin: upstream linux-firmware variant, 16KB.
		 * Currently deployed on GXL NFS rootfs.  GXL HEVC is
		 * production-ready on this firmware -- the 16KB G12A
		 * upstream variant is functionally incomplete (TASK-004),
		 * but the GXL 16KB variant has not exhibited the same
		 * wedge.  Until a static disasm confirms whether the GXL
		 * 16KB has the LMEM[0xf9] dispatch handler, conservatively
		 * tag UPSTREAM_PROTO; revisit if behavior diverges.
		 */
		.label = "linux-firmware (gxl hevc_mmu 16K)",
		.provenance = "linux-firmware upstream",
		.size = 16384,
		.sha256 = { 0x1b, 0x8b, 0x51, 0x1e, 0x50, 0xc4, 0xc9, 0x2e,
			    0xfb, 0x76, 0xa9, 0x99, 0x72, 0x2c, 0x38, 0x26,
			    0xc5, 0x4c, 0x01, 0x70, 0xfd, 0x04, 0xd9, 0xbc,
			    0x4c, 0x53, 0x39, 0x91, 0xeb, 0xe4, 0xf4, 0x81 },
		.quirks = AMVDEC_FW_QUIRK_HEVC_UPSTREAM_PROTO,
		.latest = true,
	},
	{
		/* gxl_hevc_mmu.bin: BSP 36KB variant. */
		.label = "BSP V2603 V2.5 (gxl hevc_mmu 36K)",
		.provenance = "Amlogic BSP V2603 V2.5, firmware-amlogic dpkg",
		.size = 36864,
		.sha256 = { 0x90, 0x38, 0x46, 0x32, 0x7d, 0xbd, 0xe4, 0xf1,
			    0x9a, 0x63, 0xce, 0x4b, 0x7b, 0xc4, 0x0a, 0xd7,
			    0x22, 0x8a, 0x15, 0xd3, 0x2b, 0x54, 0x15, 0x39,
			    0x2a, 0x97, 0x12, 0x5e, 0xb3, 0x93, 0xe5, 0x62 },
		.quirks = AMVDEC_FW_QUIRK_HEVC_BSP_PROTO |
			  AMVDEC_FW_QUIRK_HEVC_BSP_36K,
		.latest = false,
	},
	{
		/* gxl_hevc_mmu_swap.bin: 40KB BSP swap variant.  Not used by
		 * upstream driver (firmware_path=gxl_hevc_mmu.bin) but present
		 * on some rootfs images; recognize it so switching firmware
		 * files for investigation does not hit refuse-to-load.
		 */
		.label = "BSP (gxl hevc_mmu_swap 40K)",
		.provenance = "Amlogic BSP",
		.size = 40960,
		.sha256 = { 0x4b, 0x74, 0x87, 0x30, 0xb7, 0x5c, 0xdc, 0xc6,
			    0x5a, 0xfa, 0xc6, 0x03, 0x8d, 0x1b, 0x29, 0xaa,
			    0x30, 0x6c, 0x56, 0x56, 0x21, 0x6d, 0x81, 0x91,
			    0x52, 0xb0, 0x32, 0x5c, 0x86, 0x2b, 0x50, 0x3d },
		.quirks = AMVDEC_FW_QUIRK_HEVC_BSP_PROTO,
		.latest = false,
	},
	{
		/* g12a_hevc_mmu_swap.bin / sm1_hevc_mmu_swap.bin: BSP swap
		 * variant.  Not used by upstream driver but recognized for
		 * switching during investigation.
		 */
		.label = "BSP (g12a/sm1 hevc_mmu_swap 40K)",
		.provenance = "Amlogic BSP",
		.size = 40960,
		.sha256 = { 0x1c, 0x57, 0x1e, 0x35, 0xef, 0xad, 0xeb, 0x92,
			    0x9a, 0xce, 0x25, 0xce, 0x16, 0x05, 0x25, 0x73,
			    0xad, 0xaa, 0x78, 0x8a, 0xa6, 0xd1, 0x3b, 0xdd,
			    0xca, 0x3f, 0xd1, 0x01, 0xbb, 0x02, 0x72, 0x20 },
		.quirks = AMVDEC_FW_QUIRK_HEVC_BSP_PROTO,
		.latest = false,
	},
};
const size_t hevc_fw_versions_count = ARRAY_SIZE(hevc_fw_versions);

/* Catch mismatches between codec_hevc_fw.h define and actual table size */
static_assert(ARRAY_SIZE(hevc_fw_versions) == HEVC_FW_VERSIONS_COUNT,
	      "HEVC_FW_VERSIONS_COUNT in codec_hevc_fw.h doesn't match hevc_fw_versions[] size");
