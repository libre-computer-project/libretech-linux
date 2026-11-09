// SPDX-License-Identifier: GPL-2.0+
/*
 * VP9 firmware known-versions table.
 *
 * Each entry covers one known VP9 firmware binary identified by
 * SHA256 + size.  The .quirks field uses the AMVDEC_FW_QUIRK_VP9_*
 * bits defined in codec_vp9_fw.h.
 *
 * SHA256 values verified on 2026-04-16 against deployed NFS rootfs
 * (board 8abde1aa5c2c, /lib/firmware/meson/vdec/).
 *
 * VP9 uses the HEVC engine.  Four SoC-specific binaries are deployed:
 *   gxl_vp9_mmu.bin  -- GXL (deployed; gxl_vp9.bin is the non-MMU variant,
 *                        not used by the upstream driver)
 *   gxl_vp9.bin      -- GXL BSP non-MMU variant (not loaded by upstream)
 *   g12a_vp9.bin     -- G12A/G12B (same binary)
 *   sm1_vp9_mmu.bin  -- SM1
 */

#include <linux/build_bug.h>
#include "codec_vp9_fw.h"
#include "vdec_helpers.h"

const struct amvdec_fw_version vp9_fw_versions[] = {
	{
		/* gxl_vp9_mmu.bin: deployed on GXL NFS rootfs.
		 * Upstream driver requests gxl_vp9_mmu.bin for GXL.
		 */
		.label = "BSP (gxl_vp9_mmu.bin 28K)",
		.provenance = "Amlogic BSP, firmware-amlogic dpkg",
		.size = 28672,
		.sha256 = { 0xab, 0xbc, 0x04, 0xcb, 0xfa, 0xf5, 0x5d, 0x14,
			    0xbd, 0xb4, 0x54, 0x2a, 0xe2, 0x9d, 0x64, 0x11,
			    0xd4, 0x23, 0xb7, 0xb9, 0xf3, 0x19, 0x6d, 0x24,
			    0x3f, 0x1c, 0xc8, 0xac, 0x29, 0xa7, 0xaf, 0x8a },
		.quirks = 0,
		.latest = true,
	},
	{
		/* gxl_vp9.bin: BSP non-MMU variant for GXL.
		 * Upstream driver does not request this path, but we
		 * recognize it so investigation swaps do not hit refuse-to-load.
		 */
		.label = "BSP (gxl_vp9.bin non-MMU 28K)",
		.provenance = "Amlogic BSP, firmware-amlogic dpkg",
		.size = 28672,
		.sha256 = { 0xde, 0xaf, 0xb8, 0x38, 0x2e, 0x95, 0x10, 0xec,
			    0xfe, 0x0c, 0xd2, 0x55, 0xe8, 0x80, 0x84, 0x61,
			    0x8b, 0xcd, 0xef, 0xfb, 0x88, 0x9d, 0xf6, 0xc1,
			    0x29, 0x03, 0xc6, 0x15, 0x66, 0xde, 0xa7, 0xec },
		.quirks = 0,
		.latest = false,
	},
	{
		/* g12a_vp9.bin: deployed on G12A/G12B NFS rootfs. */
		.label = "BSP (g12a_vp9.bin 28K)",
		.provenance = "Amlogic BSP, firmware-amlogic dpkg",
		.size = 28672,
		.sha256 = { 0xe9, 0xa0, 0xd7, 0xc5, 0x84, 0x0d, 0x5a, 0x95,
			    0x52, 0x1d, 0x90, 0xa0, 0xd9, 0x18, 0x1d, 0xee,
			    0xe1, 0x63, 0xa2, 0x2b, 0x9e, 0x75, 0xa8, 0xbc,
			    0xd7, 0x7a, 0x36, 0x8e, 0x96, 0x29, 0x38, 0xfe },
		.quirks = 0,
		.latest = true,
	},
	{
		/* sm1_vp9_mmu.bin: deployed on SM1 NFS rootfs. */
		.label = "BSP (sm1_vp9_mmu.bin 28K)",
		.provenance = "Amlogic BSP, firmware-amlogic dpkg",
		.size = 28672,
		.sha256 = { 0xe4, 0x18, 0x0e, 0x3c, 0xf7, 0x96, 0x6f, 0xa2,
			    0xe9, 0x3f, 0x2d, 0xb0, 0xa6, 0x7a, 0x95, 0x7e,
			    0x47, 0x94, 0x8b, 0x8a, 0x6a, 0xb1, 0xf7, 0x67,
			    0xaa, 0x47, 0xb3, 0x10, 0xdb, 0x2a, 0x22, 0xd6 },
		.quirks = 0,
		.latest = true,
	},
	/*
	 * Upstream linux-firmware 16KB variants.  Separately-compiled VP9
	 * firmware with IDENTICAL driver-visible protocol to BSP (same
	 * AV_SCRATCH_0 status codes 0x00/0x0a/0x0e/0x20/0x23/0xf0/0xf1/
	 * 0xf2/0xff, same HEVC MPRED/SAO/DBLK/IPP register programming).
	 *
	 * Internal differences (firmware-private LMEM slot renumbering,
	 * shifted call-target PCs) are invisible to the driver so quirks=0.
	 * See vp9/task-vp9-fw-arch-review-findings-20260420.md for the
	 * full static-diff analysis.
	 *
	 * Fleet ships BSP 28K variants by default.  These entries exist so
	 * amvdec_fw_identify() logs a known label instead of "UNKNOWN" if
	 * someone installs the upstream linux-firmware pkg (Debian/Ubuntu).
	 * No GXL-MMU upstream entry because upstream gxl_vp9_mmu.bin is
	 * identical to BSP (28K, sha256 abbc04cb -- already in the BSP
	 * entry above).
	 */
	{
		.label = "upstream linux-firmware (gxl_vp9.bin 16K non-MMU)",
		.provenance = "linux-firmware pre-00419 backup, archived at slc:/srv/nfs/.../backup-00419/",
		.size = 16384,
		.sha256 = { 0xcf, 0x40, 0xaf, 0x8d, 0x5e, 0x8b, 0x02, 0xa2,
			    0x47, 0xc2, 0x8f, 0x99, 0xcd, 0x8a, 0xfe, 0x80,
			    0x94, 0xe4, 0xc4, 0x07, 0x88, 0x17, 0xe3, 0x7c,
			    0x4c, 0x98, 0xf5, 0x7c, 0x57, 0xe8, 0x79, 0x98 },
		.quirks = 0,
		.latest = false,
	},
	{
		.label = "upstream linux-firmware (g12a_vp9.bin 16K)",
		.provenance = "linux-firmware pre-00419 backup, archived at slc:/srv/nfs/.../backup-00419/",
		.size = 16384,
		.sha256 = { 0x17, 0x91, 0x62, 0xf2, 0xf0, 0x5a, 0xcd, 0x01,
			    0xcc, 0x28, 0xce, 0xfb, 0xef, 0x8d, 0xbc, 0x22,
			    0x81, 0xf2, 0x76, 0x9a, 0x0e, 0xdc, 0xff, 0x43,
			    0x41, 0x8d, 0xf9, 0x98, 0xd7, 0x1d, 0x0f, 0x86 },
		.quirks = 0,
		.latest = false,
	},
	{
		.label = "upstream linux-firmware (sm1_vp9_mmu.bin 16K)",
		.provenance = "linux-firmware pre-00419 backup, archived at slc:/srv/nfs/.../backup-00419/",
		.size = 16384,
		.sha256 = { 0xa9, 0x99, 0xea, 0xc6, 0x9e, 0xc6, 0xd8, 0x77,
			    0x85, 0x95, 0x1b, 0x11, 0xeb, 0x53, 0xbc, 0x2c,
			    0x8e, 0x5f, 0xc3, 0xe4, 0x64, 0xa3, 0x56, 0x41,
			    0x1c, 0xb2, 0xf6, 0x46, 0xc6, 0x8e, 0x2b, 0x1a },
		.quirks = 0,
		.latest = false,
	},
};
const size_t vp9_fw_versions_count = ARRAY_SIZE(vp9_fw_versions);

/* Catch mismatches between codec_vp9_fw.h define and actual table size */
static_assert(ARRAY_SIZE(vp9_fw_versions) == VP9_FW_VERSIONS_COUNT,
	      "VP9_FW_VERSIONS_COUNT in codec_vp9_fw.h doesn't match vp9_fw_versions[] size");
