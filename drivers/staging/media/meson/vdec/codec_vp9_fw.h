/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * VP9 firmware known-versions table -- declarations.
 */

#ifndef __CODEC_VP9_FW_H_
#define __CODEC_VP9_FW_H_

#include "vdec.h"

/* Quirk bits for struct amvdec_fw_version.quirks in VP9 firmware table. */
#define AMVDEC_FW_QUIRK_VP9_RESERVED_0  BIT(0)

/* Number of entries in vp9_fw_versions[] */
#define VP9_FW_VERSIONS_COUNT 7

extern const struct amvdec_fw_version vp9_fw_versions[];
extern const size_t vp9_fw_versions_count;

#endif
