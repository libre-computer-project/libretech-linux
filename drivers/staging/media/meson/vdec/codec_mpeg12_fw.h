/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * MPEG-1/2 firmware known-versions table -- declarations.
 */

#ifndef __CODEC_MPEG12_FW_H_
#define __CODEC_MPEG12_FW_H_

#include "vdec.h"

/* Quirk bits for struct amvdec_fw_version.quirks in MPEG-1/2 firmware table. */
#define AMVDEC_FW_QUIRK_MPEG12_RESERVED_0  BIT(0)

/* Number of entries in mpeg12_fw_versions[] */
#define MPEG12_FW_VERSIONS_COUNT 1

extern const struct amvdec_fw_version mpeg12_fw_versions[];
extern const size_t mpeg12_fw_versions_count;

#endif
