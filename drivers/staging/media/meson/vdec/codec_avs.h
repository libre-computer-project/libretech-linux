/* SPDX-License-Identifier: GPL-2.0+ */

#ifndef __MESON_VDEC_CODEC_AVS_H_
#define __MESON_VDEC_CODEC_AVS_H_

#include <linux/bits.h>
#include "vdec.h"

extern struct amvdec_codec_ops codec_avs_ops;

/*
 * Quirk bits for struct amvdec_fw_version.quirks in the AVS firmware
 * known-versions table.  Each bit is AVS-specific (other codecs use
 * the same u32 field with their own bit definitions).
 */
#define AMVDEC_FW_QUIRK_AVS_SINGLE_INSTANCE	BIT(0)
#define AMVDEC_FW_QUIRK_AVS_MULTI_INSTANCE	BIT(1)
#define AMVDEC_FW_QUIRK_AVS_VLD_BUSY_PATCH	BIT(2)

/* Number of entries in avs_fw_versions[]; must match codec_avs_fw.c */
#define AVS_FW_VERSIONS_COUNT	5

extern const struct amvdec_fw_version avs_fw_versions[];
extern const size_t avs_fw_versions_count;

#endif
