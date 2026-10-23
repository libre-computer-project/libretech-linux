/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * HCODEC encoder firmware known-versions table -- declarations.
 *
 * Covers all three HCODEC engine firmware variants:
 *   - gxl_h264_enc.bin   (GXL H.264 encoder)
 *   - ga_h264_enc_cabac.bin (G12A+ H.264 encoder, CABAC)
 *   - gxl_jpeg_enc.bin   (all SoCs, JPEG encoder)
 */

#ifndef __CODEC_HCODEC_FW_H_
#define __CODEC_HCODEC_FW_H_

#include "vdec.h"

/*
 * Quirk bits for struct amvdec_fw_version.quirks in HCODEC firmware table.
 * Bit 0: reserved, no active quirks yet.
 */
#define AMVDEC_FW_QUIRK_HCODEC_RESERVED_0  BIT(0)

/* Number of entries in hcodec_fw_versions[] */
#define HCODEC_FW_VERSIONS_COUNT 3

extern const struct amvdec_fw_version hcodec_fw_versions[];
extern const size_t hcodec_fw_versions_count;

#endif /* __CODEC_HCODEC_FW_H_ */
