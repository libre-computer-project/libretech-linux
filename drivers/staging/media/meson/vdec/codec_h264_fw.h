/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * H.264 firmware known-versions table -- declarations.
 */

#ifndef __CODEC_H264_FW_H_
#define __CODEC_H264_FW_H_

#include "vdec.h"

/* Quirk bits for struct amvdec_fw_version.quirks in H.264 firmware table. */
/* libre firmware family (vs BSP). Set on every libre entry. */
#define AMVDEC_FW_QUIRK_H264_LIBRE		BIT(0)
/*
 * skeleton (boot-up + overlay stub only; no codec body). Driver MUST
 * refuse M4 decode flow when this bit is set -- skeleton cannot decode
 * frames. M3 stage-1 stub tests tolerate it.
 */
#define AMVDEC_FW_QUIRK_H264_LIBRE_SKELETON	BIT(1)

/* Number of entries in h264_fw_versions[]
 * - 4 BSP entries (gxbb / gxl / gxm / g12a)
 * - 2 libre skeleton v0 entries (post-R1, 2026-04-22): GXL + G12B/SM1
 * - 2 libre M4.1.1 skeleton entries (2026-04-23): GXL + G12B/SM1
 * - 2 libre M4.1.2 program_vld entries (2026-04-24): GXL + G12B/SM1
 * - 2 libre M4.1.3 init_picture+iqidct entries (2026-04-24): GXL + G12B/SM1
 * - 2 libre M4.1.4 program_mc entries (2026-04-24): GXL + G12B/SM1
 * - 2 libre M4.1.5 program_dblk entries (2026-04-24): GXL + G12B/SM1
 * - 2 libre M4.1.6 kick+wait+drain entries (2026-04-24): GXL + G12B/SM1
 * - 2 libre M4.1.5b L1+output entries (2026-04-24): GXL + G12B/SM1
 * - 2 libre M4.1.7a kick-gate entries (2026-04-24): GXL + G12B/SM1
 * - 2 libre M4.1.7-instr entries (2026-04-25): GXL + G12B/SM1
 * - 2 libre M4.1.7-bit-skip entries (2026-04-25): GXL + G12B/SM1
 *
 * Each libre M4.1.x iteration adds 2 more entries (one per binary
 * family).  The libre-permissive carve-out in amvdec_fw_identify()
 * means missing entries warn-and-load (don't refuse), but the table
 * is still the source of truth for known-libre quirk dispatch.
 */
#define H264_FW_VERSIONS_COUNT 26

extern const struct amvdec_fw_version h264_fw_versions[];
extern const size_t h264_fw_versions_count;

#endif
