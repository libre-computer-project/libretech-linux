/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * H.264 MVC firmware known-versions table -- declarations.
 */

#ifndef __CODEC_H264_MVC_FW_H_
#define __CODEC_H264_MVC_FW_H_

#include "vdec.h"

/* Quirk bits for struct amvdec_fw_version.quirks in H.264 MVC firmware table. */
#define AMVDEC_FW_QUIRK_H264_MVC_RESERVED_0  BIT(0)

/* Number of entries in h264_mvc_fw_versions[] */
#define H264_MVC_FW_VERSIONS_COUNT 3

extern const struct amvdec_fw_version h264_mvc_fw_versions[];
extern const size_t h264_mvc_fw_versions_count;

#endif
