/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * H.263 firmware known-versions table -- declarations.
 */

#ifndef __CODEC_H263_FW_H_
#define __CODEC_H263_FW_H_

#include "vdec.h"

/* Quirk bits for struct amvdec_fw_version.quirks in H.263 firmware table. */
#define AMVDEC_FW_QUIRK_H263_RESERVED_0  BIT(0)

/* Number of entries in h263_fw_versions[] */
#define H263_FW_VERSIONS_COUNT 3

extern const struct amvdec_fw_version h263_fw_versions[];
extern const size_t h263_fw_versions_count;

#endif
