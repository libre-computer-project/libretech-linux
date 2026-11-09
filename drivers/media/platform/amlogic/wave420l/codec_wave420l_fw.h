/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Wave420L firmware known-versions table -- declarations.
 *
 * Covers monet.bin, the Chips&Media Wave420L BIT processor firmware
 * used for HEVC encoding on Amlogic SM1/G12B SoCs.
 *
 * amvdec_fw_version and amvdec_fw_identify() live in the meson-vdec
 * staging driver.  The wave420l Makefile adds the staging/vdec directory
 * to ccflags-y so these headers resolve correctly.
 */

#ifndef __CODEC_WAVE420L_FW_H_
#define __CODEC_WAVE420L_FW_H_

#include "vdec_helpers.h"

/*
 * Quirk bits for struct amvdec_fw_version.quirks in Wave420L firmware table.
 * Bit 0: reserved, no active quirks yet.
 */
#define AMVDEC_FW_QUIRK_WAVE420L_RESERVED_0  BIT(0)

/* Number of entries in wave420l_fw_versions[] */
#define WAVE420L_FW_VERSIONS_COUNT 1

extern const struct amvdec_fw_version wave420l_fw_versions[];
extern const size_t wave420l_fw_versions_count;

#endif /* __CODEC_WAVE420L_FW_H_ */
