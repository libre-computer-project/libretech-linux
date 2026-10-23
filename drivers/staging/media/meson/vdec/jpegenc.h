/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Amlogic Meson HCODEC JPEG Encoder
 *
 * Copyright (C) 2024 LibreTech
 */

#ifndef __MESON_JPEGENC_H
#define __MESON_JPEGENC_H

#include "hcodec.h"

/* JPEG firmware size: 4096 * 4 = 16384 bytes */
#define JPEGENC_MC_SIZE		(4096 * 4)

/* JPEG firmware name (same binary for all SoCs) */
#define JPEGENC_FW_NAME		"meson/vdec/gxl_jpeg_enc.bin"

/* JPEGENC_HEADER_MAX is defined in hcodec.h (included above) */

/* Default JPEG quality (1-100) */
#define JPEGENC_DEFAULT_QUALITY	85

int jpegenc_load_firmware(struct amvdec_core *core);
void jpegenc_compute_tables(struct amvenc_ctx *ctx);
void jpegenc_encode(struct amvdec_core *core, struct amvenc_ctx *ctx,
		    dma_addr_t y_addr, dma_addr_t uv_addr, u32 canvas_w,
		    dma_addr_t vlc_dst, u32 vlc_dst_size);
u32 jpegenc_build_header(u8 *buf, u32 width, u32 height,
			 const u8 *luma_qt, const u8 *chroma_qt, u32 chroma);

#endif /* __MESON_JPEGENC_H */
