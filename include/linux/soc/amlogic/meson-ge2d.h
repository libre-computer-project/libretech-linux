/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2025 Libre Computer
 * Kernel API for Amlogic GE2D 2D graphics engine
 */
#ifndef __SOC_MESON_GE2D_H
#define __SOC_MESON_GE2D_H

#include <linux/types.h>

struct device;
struct meson_ge2d_dev;

/* Pixel formats for GE2D kernel API */
enum meson_ge2d_fmt {
	GE2D_FMT_XRGB8888,	/* [X][R][G][B] in memory (LE) */
	GE2D_FMT_ARGB8888,
	GE2D_FMT_RGB888,
	GE2D_FMT_YUYV,		/* Packed YUV422 */
	GE2D_FMT_NV12,		/* Semi-planar YUV420: Y + UV */
	GE2D_FMT_NV21,		/* Semi-planar YUV420: Y + VU */
};

/**
 * struct meson_ge2d_job - describes a GE2D blit/convert operation
 * @src_addr:	DMA address of source Y (or packed) plane
 * @src_addr_uv: DMA address of source UV plane (NV12/NV21 only; 0 otherwise)
 * @src_w:	source width in pixels
 * @src_h:	source height in pixels
 * @src_stride:	source stride in bytes (Y plane; UV plane uses same stride)
 * @src_fmt:	source pixel format
 * @dst_addr:	DMA address of destination Y (or packed) plane
 * @dst_addr_uv: DMA address of destination UV plane (NV12/NV21 only; 0 otherwise)
 * @dst_w:	destination width in pixels
 * @dst_h:	destination height in pixels
 * @dst_stride:	destination stride in bytes (Y plane; UV plane uses same stride)
 * @dst_fmt:	destination pixel format
 *
 * For NV12/NV21 formats, the caller must populate src_addr_uv / dst_addr_uv
 * with the CbCr plane DMA address. The UV plane height is src_h/2 (4:2:0)
 * and its stride equals src_stride (interleaved CbCr at full width).
 * For all other formats, src_addr_uv and dst_addr_uv are unused (set to 0).
 */
struct meson_ge2d_job {
	dma_addr_t src_addr;
	dma_addr_t src_addr_uv;
	u32 src_w, src_h, src_stride;
	enum meson_ge2d_fmt src_fmt;

	dma_addr_t dst_addr;
	dma_addr_t dst_addr_uv;
	u32 dst_w, dst_h, dst_stride;
	enum meson_ge2d_fmt dst_fmt;
};

struct meson_ge2d_ops {
	struct meson_ge2d_dev *(*get)(struct device *dev);
	int (*blit)(struct meson_ge2d_dev *dev, struct meson_ge2d_job *job);
};

struct meson_ge2d_dev *meson_ge2d_get(struct device *dev);
int meson_ge2d_blit(struct meson_ge2d_dev *ge2d, struct meson_ge2d_job *job);

int meson_ge2d_register(const struct meson_ge2d_ops *ops);
void meson_ge2d_unregister(const struct meson_ge2d_ops *ops);

#endif
