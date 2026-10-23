// SPDX-License-Identifier: GPL-2.0+
/*
 * H.264 firmware known-versions table.
 *
 * Each entry covers one known H.264 (stateful) firmware binary identified
 * by SHA256 + size.  The .quirks field uses the AMVDEC_FW_QUIRK_H264_*
 * bits defined in codec_h264_fw.h.
 *
 * SHA256 values verified on 2026-04-16 against deployed NFS rootfs
 * (board 8abde1aa5c2c, /lib/firmware/meson/vdec/).
 *
 * H.264 uses the VDEC_1 engine.  Four SoC-specific binaries are deployed:
 *   gxbb_h264.bin  -- GXBB (older SoC)
 *   gxl_h264.bin   -- GXL (also used by GXLX)
 *   gxm_h264.bin   -- GXM
 *   g12a_h264.bin  -- G12A/G12B/SM1 (same binary)
 */

#include <linux/build_bug.h>
#include "codec_h264_fw.h"
#include "vdec_helpers.h"

const struct amvdec_fw_version h264_fw_versions[] = {
	{
		/* gxbb_h264.bin: deployed on GXBB NFS rootfs. */
		.label = "BSP (gxbb_h264.bin 36K)",
		.provenance = "Amlogic BSP, firmware-amlogic dpkg",
		.size = 36864,
		.sha256 = { 0xc4, 0x0b, 0x6e, 0x5b, 0xaf, 0x36, 0xda, 0x4b,
			    0x75, 0x67, 0x55, 0x3f, 0xbf, 0xd8, 0x7b, 0x32,
			    0xb2, 0x49, 0x5d, 0x49, 0xdb, 0x53, 0xe3, 0x9c,
			    0x16, 0x4d, 0xb9, 0xac, 0x78, 0xee, 0x82, 0x58 },
		.quirks = 0,
		.latest = true,
	},
	{
		/* gxl_h264.bin: deployed on GXL NFS rootfs. */
		.label = "BSP (gxl_h264.bin 36K)",
		.provenance = "Amlogic BSP, firmware-amlogic dpkg",
		.size = 36864,
		.sha256 = { 0xfb, 0xef, 0xe7, 0xec, 0x33, 0x3a, 0x05, 0xc6,
			    0xda, 0xd0, 0x3b, 0x21, 0x14, 0x5b, 0x0b, 0x27,
			    0x66, 0xbb, 0xc0, 0x33, 0xc8, 0xd1, 0x85, 0x5f,
			    0x91, 0xae, 0xea, 0x12, 0x3f, 0x48, 0x5b, 0xec },
		.quirks = 0,
		.latest = true,
	},
	{
		/* gxm_h264.bin: deployed on GXM NFS rootfs. */
		.label = "BSP (gxm_h264.bin 36K)",
		.provenance = "Amlogic BSP, firmware-amlogic dpkg",
		.size = 36864,
		.sha256 = { 0x3a, 0x89, 0xd7, 0x35, 0xca, 0x87, 0xd5, 0x07,
			    0x5e, 0x27, 0xc1, 0xbb, 0xe1, 0xfa, 0x65, 0x34,
			    0xe8, 0x75, 0xa1, 0x4f, 0xda, 0x77, 0x76, 0x83,
			    0x1f, 0xf3, 0xc1, 0x6b, 0x1a, 0x5b, 0x81, 0x04 },
		.quirks = 0,
		.latest = true,
	},
	{
		/* g12a_h264.bin: deployed on G12A/G12B/SM1 NFS rootfs.
		 * Same binary for all three SoCs.
		 */
		.label = "BSP (g12a_h264.bin 36K)",
		.provenance = "Amlogic BSP, firmware-amlogic dpkg",
		.size = 36864,
		.sha256 = { 0xed, 0x20, 0x14, 0xed, 0xb5, 0xb7, 0x59, 0x65,
			    0x49, 0x56, 0xb8, 0x29, 0x77, 0x01, 0x81, 0xb4,
			    0x6e, 0xfb, 0x91, 0xc6, 0x8a, 0x2e, 0x6c, 0x65,
			    0x10, 0xc3, 0x54, 0xff, 0xdc, 0xbe, 0x1a, 0x0a },
		.quirks = 0,
		.latest = true,
	},
	{
		/* libre_gxl_h264.bin: skeleton v0 (post-R1 dissolution).
		 * Codec body lands at M4; this entry exists so the skeleton
		 * passes amvdec_fw_identify and M3 stage-1 stub tests have a
		 * match to exercise the identify path.
		 */
		.label = "libre_h264 skeleton v0 GXL (2026-04-22)",
		.provenance = "libre_h264.S @ a3e5cec1",
		.size = 16384,
		.sha256 = { 0xc6, 0x7c, 0xf1, 0x50, 0x02, 0xec, 0x0a, 0xb6,
			    0x34, 0x53, 0xfc, 0x13, 0x1d, 0x38, 0x7f, 0x0c,
			    0xea, 0x5c, 0x4f, 0x2a, 0x3b, 0xcf, 0x5f, 0x5a,
			    0xd3, 0xc1, 0x76, 0x7d, 0x8b, 0xc5, 0x92, 0x68 },
		.quirks = AMVDEC_FW_QUIRK_H264_LIBRE |
			  AMVDEC_FW_QUIRK_H264_LIBRE_SKELETON,
		.latest = false,
	},
	{
		/* libre_g12b_h264.bin: skeleton v0 for G12B + SM1
		 * (byte-identical).
		 */
		.label = "libre_h264 skeleton v0 G12B+SM1 (2026-04-22)",
		.provenance = "libre_h264.S @ a3e5cec1",
		.size = 16384,
		.sha256 = { 0xce, 0xde, 0xba, 0x77, 0x52, 0x65, 0xd9, 0xb2,
			    0x27, 0x49, 0x39, 0x02, 0xdf, 0x19, 0x9c, 0xbd,
			    0xb1, 0xd8, 0x79, 0xf0, 0x36, 0x1c, 0x66, 0x13,
			    0x3f, 0x19, 0xe0, 0xf2, 0x2d, 0x11, 0x82, 0x5f },
		.quirks = AMVDEC_FW_QUIRK_H264_LIBRE |
			  AMVDEC_FW_QUIRK_H264_LIBRE_SKELETON,
		.latest = false,
	},
	{
		/* libre_gxl_h264.bin: M4.1.1 handler skeleton with §5.2
		 * control flow + stub helpers (post-stateful-bleed audit).
		 * h264_handler walks: init_picture -> program_vld ->
		 * program_iqidct -> program_mc -> program_dblk ->
		 * kick_decode -> mb_wait_stub -> drain_pipeline ->
		 * pack_slice_outcome -> signal_done.  Each helper is a
		 * stub (writes its L3 stage marker to AV_SCRATCH_C and
		 * rets); real register sequences land in M4.1.2+.
		 * amlogic commit 09fb629b.
		 */
		.label = "libre_h264 M4.1.1 skeleton GXL (2026-04-23)",
		.provenance = "libre_h264.S @ 09fb629b",
		.size = 16384,
		.sha256 = { 0xf8, 0xf3, 0xaf, 0x9a, 0x26, 0x77, 0x05, 0xee,
			    0x45, 0x7a, 0xf0, 0x85, 0x91, 0x38, 0x5c, 0xd5,
			    0x3a, 0xf3, 0x96, 0xb0, 0x23, 0xad, 0x9b, 0xed,
			    0xa9, 0x34, 0x6b, 0xa0, 0xb2, 0x35, 0x4b, 0x86 },
		.quirks = AMVDEC_FW_QUIRK_H264_LIBRE |
			  AMVDEC_FW_QUIRK_H264_LIBRE_SKELETON,
		.latest = false,
	},
	{
		/* libre_g12b_h264.bin: M4.1.1 skeleton for G12B + SM1
		 * (byte-identical).
		 */
		.label = "libre_h264 M4.1.1 skeleton G12B+SM1 (2026-04-23)",
		.provenance = "libre_h264.S @ 09fb629b",
		.size = 16384,
		.sha256 = { 0x22, 0x22, 0xfe, 0x80, 0xfe, 0xcc, 0x90, 0xd5,
			    0x89, 0xaa, 0x35, 0x91, 0x57, 0xc3, 0xfd, 0x61,
			    0xe6, 0xcd, 0xf0, 0xa2, 0xb9, 0xd5, 0x22, 0x01,
			    0x23, 0x72, 0xdb, 0x21, 0xf6, 0x17, 0xfe, 0xd5 },
		.quirks = AMVDEC_FW_QUIRK_H264_LIBRE |
			  AMVDEC_FW_QUIRK_H264_LIBRE_SKELETON,
		.latest = false,
	},
	{
		/* libre_gxl_h264.bin: M4.1.2 -- program_vld real impl
		 * (BSP h264-firmware-trace.md §6.2 + §5.3).  Reads
		 * LMEM[0x10D/0x10E] vififo_start/end and writes the four
		 * VIFIFO pointer registers (start/curr/end/wp); writes
		 * the BSP §6.2 per-frame init constants
		 * SLICE_START_BYTE_01=0 / SLICE_START_BYTE_23=0x01010000
		 * / MPEG1_2_REG=1 / VLD_ERROR_MASK=0x1011.  Does NOT
		 * write VIFIFO_CONTROL (driver/ESPARSER does that) and
		 * does NOT poll bit 8 (BSP polls only at slice
		 * boundaries, not in init).  Other helpers
		 * (init_picture, program_iqidct, program_mc,
		 * program_dblk, kick_decode, mb_wait, drain_pipeline,
		 * pack_slice_outcome) remain L3 stubs -- SKELETON
		 * quirk still set, real decode pending M4.1.7.
		 */
		.label = "libre_h264 M4.1.2 program_vld GXL (2026-04-24)",
		.provenance = "libre_h264.S @ 7b79745f",
		.size = 16384,
		.sha256 = { 0x8e, 0x84, 0xdd, 0x2b, 0xaf, 0x5e, 0xee, 0xf0,
			    0x5a, 0xb6, 0xf9, 0xb4, 0xcf, 0x6d, 0xc0, 0x55,
			    0xf8, 0x96, 0xdf, 0x8a, 0x9a, 0x86, 0xd1, 0xa1,
			    0xf3, 0x7f, 0x15, 0xd7, 0x94, 0x97, 0x98, 0x1b },
		.quirks = AMVDEC_FW_QUIRK_H264_LIBRE |
			  AMVDEC_FW_QUIRK_H264_LIBRE_SKELETON,
		.latest = false,
	},
	{
		/* libre_g12b_h264.bin: M4.1.2 program_vld for G12B + SM1
		 * (byte-identical).
		 */
		.label = "libre_h264 M4.1.2 program_vld G12B+SM1 (2026-04-24)",
		.provenance = "libre_h264.S @ 7b79745f",
		.size = 16384,
		.sha256 = { 0x43, 0xe4, 0x3f, 0x0e, 0xfd, 0xf3, 0x56, 0x44,
			    0x26, 0xf8, 0x69, 0x0e, 0x02, 0x5e, 0x9d, 0x25,
			    0x89, 0xd5, 0x08, 0x96, 0x75, 0xc6, 0xb6, 0xa5,
			    0x03, 0x26, 0xf5, 0xa7, 0xc5, 0x75, 0xb7, 0xe6 },
		.quirks = AMVDEC_FW_QUIRK_H264_LIBRE |
			  AMVDEC_FW_QUIRK_H264_LIBRE_SKELETON,
		.latest = false,
	},
	{
		/* libre_gxl_h264.bin: M4.1.3 -- init_picture + program_iqidct
		 * real impls.  init_picture body ported verbatim from
		 * BSP upstream_gxl_h264.asm PC 0x00a2-0x00be (MC_CTRL1
		 * pulse, DBLK_RST pulse, MDEC_PIC_DC_STATUS idle wait,
		 * VLD_C3D drain wait, MDEC_PIC_DC_CTRL threshold
		 * pack/unpack) + IQIDCT_CONTROL bit-9 enable from
		 * BSP PC 0x01e0-0x01e2 + PMV1_Y reset (368 / 0x70) from
		 * BSP PC 0x00f8-0x00fa.  Skips the BSP `call 0x0317`
		 * LMEM-state-reload (libre uses driver-side LMEM DMA)
		 * and the BSP private-workspace LMEM stashes
		 * (LMEM[0x9b/0x94/0xa0/0xa9/0x92]).  program_iqidct
		 * reads slice_qp from LMEM[0x120] and writes SLICE_QP +
		 * M4_TABLE_SELECT=0 (BSP PC 0x016e/0x016f).
		 *
		 * Also fixes vdec1_regs.h IQIDCT_CONTROL define from
		 * 0x0e04 (wrong) to 0x0e0e (BSP-confirmed) -- the wrong
		 * define would have silently mis-targeted the bit-9 set.
		 */
		.label = "libre_h264 M4.1.3 init_picture+iqidct GXL (2026-04-24)",
		.provenance = "libre_h264.S @ bb129999",
		.size = 16384,
		.sha256 = { 0x9a, 0x99, 0x70, 0x41, 0x3d, 0x75, 0x8d, 0x20,
			    0x73, 0x52, 0xcd, 0x94, 0x68, 0xb6, 0xb1, 0x07,
			    0xc9, 0xd4, 0xa4, 0xf3, 0xb1, 0x0e, 0xc3, 0xe6,
			    0xd4, 0xe2, 0x92, 0x67, 0xb5, 0xa4, 0x9d, 0xe3 },
		.quirks = AMVDEC_FW_QUIRK_H264_LIBRE |
			  AMVDEC_FW_QUIRK_H264_LIBRE_SKELETON,
		.latest = false,
	},
	{
		/* libre_g12b_h264.bin: M4.1.3 init_picture + program_iqidct
		 * for G12B + SM1 (byte-identical).
		 */
		.label = "libre_h264 M4.1.3 init_picture+iqidct G12B+SM1 (2026-04-24)",
		.provenance = "libre_h264.S @ bb129999",
		.size = 16384,
		.sha256 = { 0xde, 0x01, 0x16, 0xe7, 0x23, 0xf2, 0x49, 0x86,
			    0x4b, 0xf3, 0x10, 0x49, 0x58, 0x0f, 0x5d, 0xe6,
			    0x39, 0x21, 0x0f, 0x52, 0x56, 0xe3, 0xaa, 0xf2,
			    0x00, 0x70, 0x8d, 0xac, 0x7d, 0x3b, 0x8a, 0x85 },
		.quirks = AMVDEC_FW_QUIRK_H264_LIBRE |
			  AMVDEC_FW_QUIRK_H264_LIBRE_SKELETON,
		.latest = false,
	},
	{
		/* libre_gxl_h264.bin: M4.1.4 -- program_mc forwards 16 L0
		 * ref canvases LMEM[0x130..0x13F] -> ANC0..ANC15
		 * (CR 0x0990..0x099F).  16 unrolled lda+mtcr (mtcr requires
		 * constant CR addr; runtime loop with computed dest is not
		 * possible).
		 *
		 * Architectural difference from stateful BSP H.264: stateful
		 * driver populates ANC0..N once via amvdec_set_canvases at
		 * codec_h264.c:879; libre stateless driver pre-fills only
		 * AV_SCRATCH_0..N (capture buffer pool) and stashes per-
		 * slice resolved canvases in LMEM[0x130..]; firmware
		 * forwards LMEM->ANC each slice because V4L2 stateless
		 * ref-list can change every Request.
		 *
		 * Scope: L0 only (P-frames).  L1 (B-frames) + weighted_pred
		 * + output canvas destination defer to M4.1.5 / M4.1.6.
		 */
		.label = "libre_h264 M4.1.4 program_mc GXL (2026-04-24)",
		.provenance = "libre_h264.S @ 7a7cbbbf",
		.size = 16384,
		.sha256 = { 0xc6, 0xb7, 0xcd, 0xda, 0x5d, 0x4a, 0xb0, 0x74,
			    0x24, 0x7a, 0x23, 0x46, 0xef, 0x18, 0x48, 0x05,
			    0xf1, 0x50, 0x2f, 0xb9, 0x88, 0x62, 0x28, 0x6a,
			    0x23, 0x2a, 0x9c, 0x2a, 0x70, 0x9e, 0xe8, 0xf1 },
		.quirks = AMVDEC_FW_QUIRK_H264_LIBRE |
			  AMVDEC_FW_QUIRK_H264_LIBRE_SKELETON,
		.latest = false,
	},
	{
		/* libre_g12b_h264.bin: M4.1.4 program_mc for G12B + SM1
		 * (byte-identical).
		 */
		.label = "libre_h264 M4.1.4 program_mc G12B+SM1 (2026-04-24)",
		.provenance = "libre_h264.S @ 7a7cbbbf",
		.size = 16384,
		.sha256 = { 0x61, 0x0b, 0x3d, 0x54, 0xa9, 0x92, 0xee, 0x8d,
			    0x29, 0x4a, 0xf7, 0x6c, 0xf9, 0xa6, 0x5c, 0x5f,
			    0x73, 0x8a, 0x7d, 0x0a, 0x26, 0xce, 0x4e, 0x86,
			    0x22, 0x45, 0xfd, 0x81, 0x49, 0x88, 0x5e, 0x92 },
		.quirks = AMVDEC_FW_QUIRK_H264_LIBRE |
			  AMVDEC_FW_QUIRK_H264_LIBRE_SKELETON,
		.latest = false,
	},
	{
		/* libre_gxl_h264.bin: M4.1.5 -- program_dblk reads V4L2
		 * deblock params from LMEM[0x121..0x123] (proves contract)
		 * but does NOT write DBLK_CTRL yet.  BSP DBLK programming
		 * depends on BSP-private slice-parser state at multiple
		 * PCs (0x035f/0x0686/0x068d/0x06bc) that V4L2 stateless
		 * does not expose; speculative bit packing risks hanging
		 * the deblock pipeline.  Real DBLK_CTRL packing deferred
		 * to M4.1.7 first-light where we can correlate against
		 * software-reference frames.
		 */
		.label = "libre_h264 M4.1.5 program_dblk GXL (2026-04-24)",
		.provenance = "libre_h264.S @ 826c7345",
		.size = 16384,
		.sha256 = { 0xdf, 0xb1, 0x3e, 0x82, 0xdc, 0x31, 0xd5, 0xb6,
			    0xb8, 0x02, 0xdc, 0x5c, 0xb0, 0x7b, 0x69, 0xe0,
			    0x81, 0x24, 0x18, 0xa3, 0x94, 0x3c, 0x77, 0x21,
			    0xc6, 0x84, 0x2f, 0x3c, 0x8f, 0x46, 0x05, 0x9d },
		.quirks = AMVDEC_FW_QUIRK_H264_LIBRE |
			  AMVDEC_FW_QUIRK_H264_LIBRE_SKELETON,
		.latest = false,
	},
	{
		/* libre_g12b_h264.bin: M4.1.5 program_dblk for G12B + SM1
		 * (byte-identical).
		 */
		.label = "libre_h264 M4.1.5 program_dblk G12B+SM1 (2026-04-24)",
		.provenance = "libre_h264.S @ 826c7345",
		.size = 16384,
		.sha256 = { 0x62, 0x57, 0xc3, 0x34, 0xcc, 0x8c, 0x1c, 0x1f,
			    0x0e, 0x80, 0x8d, 0x91, 0xb4, 0x68, 0x67, 0x4f,
			    0xf1, 0x75, 0x30, 0x84, 0x18, 0xae, 0x26, 0xca,
			    0x0f, 0xa4, 0x54, 0x1f, 0x06, 0x8e, 0xd7, 0xdf },
		.quirks = AMVDEC_FW_QUIRK_H264_LIBRE |
			  AMVDEC_FW_QUIRK_H264_LIBRE_SKELETON,
		.latest = false,
	},
	{
		/* libre_gxl_h264.bin: M4.1.6 -- kick_decode + mb_wait +
		 * drain_pipeline real implementations.
		 * - kick_decode: POWER_CTL_VLD bit 13 set (BSP PC
		 *   0x0111-0x0114).  Gated on LMEM[0x10D]!=0 (vififo_start)
		 *   so stub-test (LMEM=0) skips kick and avoids wedging
		 *   VLD on garbage bounds.
		 * - mb_wait: bounded poll on VLD_STATUS_CTRL bit 6
		 *   (frame/slice end) with 4095-iter timeout (~160 us).
		 *   Libre-polled equivalent of BSP IRQ dispatch at PC
		 *   0x0069 `bandi r7, 64, +5`.
		 * - drain_pipeline: bounded poll on MC_OMDN_MB_XY any-
		 *   progress (BSP mc_cache_sync PC 0x009b-0x00a0).  Real
		 *   per-frame MB-target match deferred to M4.1.7.
		 */
		.label = "libre_h264 M4.1.6 kick+wait+drain GXL (2026-04-24)",
		.provenance = "libre_h264.S @ 69d618e3",
		.size = 16384,
		.sha256 = { 0x3d, 0x70, 0xb6, 0xd6, 0x4e, 0x5b, 0x8d, 0xee,
			    0x16, 0xcb, 0x0f, 0x62, 0xdf, 0xb8, 0xe0, 0xd6,
			    0x54, 0xe8, 0x3c, 0x78, 0x27, 0xad, 0x94, 0x17,
			    0x5f, 0xe7, 0xd8, 0x58, 0x06, 0xf2, 0x81, 0x6e },
		.quirks = AMVDEC_FW_QUIRK_H264_LIBRE |
			  AMVDEC_FW_QUIRK_H264_LIBRE_SKELETON,
		.latest = false,
	},
	{
		/* libre_g12b_h264.bin: M4.1.6 kick+wait+drain for
		 * G12B + SM1 (byte-identical).
		 */
		.label = "libre_h264 M4.1.6 kick+wait+drain G12B+SM1 (2026-04-24)",
		.provenance = "libre_h264.S @ 69d618e3",
		.size = 16384,
		.sha256 = { 0x5d, 0xb6, 0xbc, 0x87, 0x42, 0x49, 0xc0, 0xd1,
			    0x60, 0xa3, 0x01, 0x45, 0xf2, 0xee, 0xe8, 0x09,
			    0x1b, 0x30, 0x4d, 0x30, 0x4b, 0x75, 0x63, 0x07,
			    0xa0, 0x5b, 0x02, 0xbf, 0x02, 0xb1, 0x6e, 0xff },
		.quirks = AMVDEC_FW_QUIRK_H264_LIBRE |
			  AMVDEC_FW_QUIRK_H264_LIBRE_SKELETON,
		.latest = false,
	},
	{
		/* libre_gxl_h264.bin: M4.1.5b -- L1 ref forwarding +
		 * output canvas (closes the deferred scope from M4.1.4).
		 * - init_picture: write LMEM[0x104] output_canvas to
		 *   REC/DBKR/DBKW (BSP PC 0x20c4-0x20c6 pattern).
		 * - program_mc: extend L0 ANC0..15 with L1 ANC16..31 from
		 *   LMEM[0x140..0x14F] (16 more unrolled lda+mtcr).
		 * weighted_pred DMA still deferred (rare path, MC_WT_PRED_CTRL
		 * bit packing unconfirmed in BSP trace §5.6).
		 */
		.label = "libre_h264 M4.1.5b L1+output GXL (2026-04-24)",
		.provenance = "libre_h264.S @ 7bd8cb1d",
		.size = 16384,
		.sha256 = { 0x2b, 0x8a, 0x2e, 0xcc, 0xc5, 0x14, 0x96, 0xcd,
			    0x59, 0xe9, 0xda, 0x66, 0x7e, 0x2f, 0xcc, 0xbd,
			    0xa6, 0xb6, 0x16, 0x14, 0x5e, 0x75, 0x4f, 0xbf,
			    0xcd, 0xff, 0x71, 0x03, 0x1a, 0x7d, 0x1a, 0xe3 },
		.quirks = AMVDEC_FW_QUIRK_H264_LIBRE |
			  AMVDEC_FW_QUIRK_H264_LIBRE_SKELETON,
		.latest = false,
	},
	{
		/* libre_g12b_h264.bin: M4.1.5b L1+output for G12B + SM1
		 * (byte-identical).
		 */
		.label = "libre_h264 M4.1.5b L1+output G12B+SM1 (2026-04-24)",
		.provenance = "libre_h264.S @ 7bd8cb1d",
		.size = 16384,
		.sha256 = { 0x9b, 0xfe, 0xd0, 0xfd, 0xf2, 0x0d, 0xa8, 0xc3,
			    0xae, 0x82, 0x2f, 0x11, 0xf1, 0xa8, 0x05, 0xc7,
			    0x22, 0x9f, 0xc2, 0xbb, 0xe9, 0x20, 0x3c, 0xb9,
			    0xab, 0x77, 0x8d, 0xe1, 0x2f, 0x0b, 0x9a, 0xc4 },
		.quirks = AMVDEC_FW_QUIRK_H264_LIBRE |
			  AMVDEC_FW_QUIRK_H264_LIBRE_SKELETON,
		.latest = false,
	},
	{
		/* libre_gxl_h264.bin: M4.1.7a -- kick_decode gate
		 * tightened to skip on empty VIFIFO (start == end), not
		 * just null start.  Lets driver wire vififo_start/end
		 * from src vb DMA without breaking stub-test (V4L2 stub
		 * feeds 0-byte buffers -> end == start -> kick still
		 * skipped).  Real bitstream feed -> end > start -> kick
		 * fires.  Two extra firmware insns (lda + beq).
		 */
		.label = "libre_h264 M4.1.7a kick-gate GXL (2026-04-24)",
		.provenance = "libre_h264.S @ 10871899",
		.size = 16384,
		.sha256 = { 0xae, 0x6f, 0xb5, 0xb1, 0x0a, 0x93, 0x9c, 0x08,
			    0x58, 0x2e, 0xcb, 0xe1, 0x0b, 0x0b, 0xd7, 0xaf,
			    0x67, 0xf9, 0xca, 0x1a, 0x9f, 0x6f, 0x05, 0xe8,
			    0x3a, 0x39, 0xd4, 0x0b, 0xa8, 0xc6, 0x3b, 0x54 },
		.quirks = AMVDEC_FW_QUIRK_H264_LIBRE |
			  AMVDEC_FW_QUIRK_H264_LIBRE_SKELETON,
		.latest = false,
	},
	{
		/* libre_g12b_h264.bin: M4.1.7a kick-gate for G12B + SM1
		 * (byte-identical).
		 */
		.label = "libre_h264 M4.1.7a kick-gate G12B+SM1 (2026-04-24)",
		.provenance = "libre_h264.S @ 10871899",
		.size = 16384,
		.sha256 = { 0x60, 0xbc, 0x03, 0x8e, 0x43, 0x4e, 0x38, 0x9e,
			    0x93, 0x57, 0xd4, 0x36, 0xed, 0x69, 0xef, 0x46,
			    0xde, 0xd8, 0x9c, 0xd1, 0xb6, 0x43, 0x44, 0x8f,
			    0x0e, 0x20, 0x3c, 0x6d, 0x17, 0x01, 0x97, 0x57 },
		.quirks = AMVDEC_FW_QUIRK_H264_LIBRE |
			  AMVDEC_FW_QUIRK_H264_LIBRE_SKELETON,
		.latest = false,
	},
	{
		/* libre_gxl_h264.bin: M4.1.7-instr -- toward first-light
		 * decode visibility.  pack_slice_outcome packs HW MB count
		 * (MC_OMDN_MB_XY[15:8]) into AV_SCRATCH_E bits[31:16];
		 * signal_done packs VLD_DECODE_STATUS / DBLK_STATUS /
		 * MDEC_PIC_DC_STATUS into AV_SCRATCH_F upper bits.  Stub-
		 * test path: all status regs read 0 -> SCRATCH_E and
		 * SCRATCH_F both 0, indistinguishable from prior M4.1.7a.
		 * Real decode: post-mortem visible without rebuilding fw.
		 */
		.label = "libre_h264 M4.1.7-instr GXL (2026-04-25)",
		.provenance = "libre_h264.S @ 85cce11b",
		.size = 16384,
		.sha256 = { 0x97, 0x01, 0x1f, 0x3c, 0x8a, 0xdc, 0xa5, 0xa9,
			    0xdc, 0x55, 0x24, 0x04, 0xe2, 0x2f, 0xac, 0x74,
			    0x0d, 0x34, 0x02, 0x25, 0xba, 0x35, 0xf6, 0x99,
			    0x62, 0x02, 0x8a, 0x4c, 0x5b, 0x46, 0x5c, 0x03 },
		.quirks = AMVDEC_FW_QUIRK_H264_LIBRE |
			  AMVDEC_FW_QUIRK_H264_LIBRE_SKELETON,
		.latest = false,
	},
	{
		/* libre_g12b_h264.bin: M4.1.7-instr for G12B + SM1
		 * (byte-identical).
		 */
		.label = "libre_h264 M4.1.7-instr G12B+SM1 (2026-04-25)",
		.provenance = "libre_h264.S @ 85cce11b",
		.size = 16384,
		.sha256 = { 0x6e, 0xa0, 0x52, 0x4f, 0x6a, 0x1f, 0xdd, 0x80,
			    0x02, 0x42, 0x83, 0x02, 0xce, 0x8e, 0xc6, 0xb6,
			    0x36, 0x29, 0xf8, 0x44, 0xdf, 0xc7, 0x47, 0xcf,
			    0x9f, 0x08, 0x2c, 0x38, 0x20, 0x9f, 0xa7, 0x82 },
		.quirks = AMVDEC_FW_QUIRK_H264_LIBRE |
			  AMVDEC_FW_QUIRK_H264_LIBRE_SKELETON,
		.latest = false,
	},
	{
		/* libre_gxl_h264.bin: M4.1.7-blocker-1 -- VLD bit-pointer
		 * skip via VIFF_BIT_CNT direct write (mechanism (ii) per
		 * probe-1).  kick_decode now positions VLD past slice
		 * header before firing POWER_CTL_VLD bit 13.  Stub-test
		 * unchanged: empty-buffer gate still skips kick.  Real
		 * decode validates empirically (M4.1.7-final).
		 */
		.label = "libre_h264 M4.1.7-bit-skip GXL (2026-04-25)",
		.provenance = "libre_h264.S @ ef6ea6e9",
		.size = 16384,
		.sha256 = { 0xbf, 0xaa, 0xef, 0x74, 0xcc, 0x61, 0x93, 0x77,
			    0x8a, 0x67, 0x3d, 0x78, 0xe7, 0x34, 0x8e, 0x22,
			    0x82, 0x5b, 0x41, 0x0e, 0x7d, 0xfe, 0xde, 0xc1,
			    0x50, 0xcd, 0xc0, 0xcf, 0x76, 0x99, 0xad, 0xeb },
		.quirks = AMVDEC_FW_QUIRK_H264_LIBRE |
			  AMVDEC_FW_QUIRK_H264_LIBRE_SKELETON,
		.latest = false,
	},
	{
		/* libre_g12b_h264.bin: M4.1.7-bit-skip for G12B + SM1
		 * (byte-identical).
		 */
		.label = "libre_h264 M4.1.7-bit-skip G12B+SM1 (2026-04-25)",
		.provenance = "libre_h264.S @ ef6ea6e9",
		.size = 16384,
		.sha256 = { 0x0d, 0x93, 0x6a, 0x67, 0xd1, 0xd4, 0xc1, 0xc6,
			    0x3b, 0xf2, 0xa8, 0xec, 0xde, 0xb8, 0x34, 0x4d,
			    0xea, 0x92, 0x82, 0x69, 0x27, 0x18, 0x9a, 0x0e,
			    0x21, 0xcd, 0x15, 0x46, 0xfa, 0x18, 0x84, 0xbb },
		.quirks = AMVDEC_FW_QUIRK_H264_LIBRE |
			  AMVDEC_FW_QUIRK_H264_LIBRE_SKELETON,
		.latest = false,
	},
};
const size_t h264_fw_versions_count = ARRAY_SIZE(h264_fw_versions);

/* Catch mismatches between codec_h264_fw.h define and actual table size */
static_assert(ARRAY_SIZE(h264_fw_versions) == H264_FW_VERSIONS_COUNT,
	      "H264_FW_VERSIONS_COUNT in codec_h264_fw.h doesn't match h264_fw_versions[] size");
