/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * MJPEG firmware known-versions table -- declarations.
 */

#ifndef __CODEC_MJPEG_FW_H_
#define __CODEC_MJPEG_FW_H_

#include "vdec.h"

/* Quirk bits for struct amvdec_fw_version.quirks in MJPEG firmware table. */
#define AMVDEC_FW_QUIRK_MJPEG_RESERVED_0  BIT(0)

/* Number of entries in mjpeg_fw_versions[] */
#define MJPEG_FW_VERSIONS_COUNT 1

extern const struct amvdec_fw_version mjpeg_fw_versions[];
extern const size_t mjpeg_fw_versions_count;

#endif
