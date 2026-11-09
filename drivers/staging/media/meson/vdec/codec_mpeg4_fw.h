/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * MPEG-4 firmware known-versions table -- declarations.
 */

#ifndef __CODEC_MPEG4_FW_H_
#define __CODEC_MPEG4_FW_H_

#include "vdec.h"

/* Quirk bits for struct amvdec_fw_version.quirks in MPEG-4 firmware table.
 *
 * Bit 0: SM1 stall producing 0 frames (KI-005, 2026-04-16).
 *        sm1_mpeg4_5.bin is recognized but produces 0 decoded frames on SM1.
 *        Informational marker only -- not wired to any driver code path.
 */
#define AMVDEC_FW_QUIRK_MPEG4_SM1_STALL  BIT(0)

/* Number of entries in mpeg4_fw_versions[] */
#define MPEG4_FW_VERSIONS_COUNT 2

extern const struct amvdec_fw_version mpeg4_fw_versions[];
extern const size_t mpeg4_fw_versions_count;

#endif
