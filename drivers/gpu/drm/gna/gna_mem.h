/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright(c) 2017-2022 Intel Corporation */

#ifndef __GNA_MEM_H__
#define __GNA_MEM_H__

#include <linux/types.h>

#include "gna_hw.h"

struct gna_gem_object;
struct work_struct;
struct gna_device;

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
};

int gna_mmu_init(struct gna_device *gna_priv);

void gna_mmu_add(struct gna_device *gna_priv, struct drm_gem_shmem_object *drmshmemo);

void gna_mmu_clear(struct gna_device *gna_priv);

bool gna_gem_object_put_pages_sgt(struct gna_gem_object *gna_obj);

void gna_gem_obj_release_work(struct work_struct *work);

#endif // __GNA_MEM_H__
