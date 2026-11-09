/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright(c) 2017-2022 Intel Corporation */

#ifndef __GNA_MEM_H__
#define __GNA_MEM_H__

#include <linux/types.h>

#include "gna_hw.h"

struct drm_gem_object;
struct drm_file;
struct gna_buffer_with_object;
struct gna_gem_object;
struct gna_device;

/* requests with up to this many buffers keep their mapping for the next request (gna_mmu_map_request) */
#define GNA_MMU_MAPPED_MAX 16

struct gna_xnn_descriptor {
	u32 labase;
	u16 lacount;
	u16 _rsvd;
};

struct gna_mmu {
	u32 vamaxaddr;
	u8 __res_204[12];
	u32 pagedir_n[GNA_PGDIRN_LEN];
};

struct gna_hw_descriptor {
	u8 __res_0000[256];
	struct gna_xnn_descriptor xnn_config;
	u8 __unused[248];
	struct gna_mmu mmu;
};

struct gna_mmu_object {
	struct gna_hw_descriptor *hwdesc;

	dma_addr_t hwdesc_dma;

	u32 **pagetables;
	dma_addr_t *pagetables_dma;

	u32 num_pagetables;

	u32 filled_pts;
	u32 filled_pages;

	/* the objects whose pages fill the page tables, in order, each holding a reference; 0 = nothing kept */
	struct drm_gem_object *mapped[GNA_MMU_MAPPED_MAX];
	u32 mapped_count;
};

int gna_mmu_init(struct gna_device *gna_priv);

int gna_mmu_map_request(struct gna_device *gna_priv, struct gna_buffer_with_object *buffers, u64 count);

void gna_mmu_forget(struct gna_device *gna_priv, struct drm_gem_object *obj);

void gna_mmu_release(struct gna_device *gna_priv);

void gna_gem_free_requests(struct gna_device *gna_priv, struct drm_file *file, u32 handle,
			   struct drm_gem_object *obj);

#endif // __GNA_MEM_H__
