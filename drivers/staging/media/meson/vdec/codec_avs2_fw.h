/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * AVS2 firmware known-versions table -- declarations.
 */

#ifndef __CODEC_AVS2_FW_H_
#define __CODEC_AVS2_FW_H_

#include "vdec.h"

/* Quirk bits for struct amvdec_fw_version.quirks in AVS2 firmware table. */
#define AMVDEC_FW_QUIRK_AVS2_RESERVED_0  BIT(0)

/* Number of entries in avs2_fw_versions[] */
#define AVS2_FW_VERSIONS_COUNT 1

extern const struct amvdec_fw_version avs2_fw_versions[];
extern const size_t avs2_fw_versions_count;

#endif
