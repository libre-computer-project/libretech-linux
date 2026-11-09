/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2015-2018 Etnaviv Project
 */

#ifndef __ETNAVIV_MMU_H__
#define __ETNAVIV_MMU_H__

#define ETNAVIV_PROT_READ	(1 << 0)
#define ETNAVIV_PROT_WRITE	(1 << 1)

enum etnaviv_iommu_version {
	ETNAVIV_IOMMU_V1 = 0,
	ETNAVIV_IOMMU_V2,
};

struct etnaviv_gpu;
struct etnaviv_vram_mapping;
struct etnaviv_iommu_global;
struct etnaviv_iommu_context;
struct etnaviv_iommuv2_hw_tables;

struct etnaviv_iommu_ops {
	struct etnaviv_iommu_context *(*init)(struct etnaviv_iommu_global *);
	void (*free)(struct etnaviv_iommu_context *);
	int (*map)(struct etnaviv_iommu_context *context, unsigned long iova,
		   phys_addr_t paddr, size_t size, int prot);
	size_t (*unmap)(struct etnaviv_iommu_context *context, unsigned long iova,
			size_t size);
	size_t (*dump_size)(struct etnaviv_iommu_context *);
	void (*dump)(struct etnaviv_iommu_context *, void *);
	void (*restore)(struct etnaviv_gpu *, struct etnaviv_iommu_context *);
};

extern const struct etnaviv_iommu_ops etnaviv_iommuv1_ops;
extern const struct etnaviv_iommu_ops etnaviv_iommuv2_ops;

/*
 * qi:N1541 / qi:N1543 -- 2 = 64 KiB PTEs (default); 0 = 4 KiB; 1 (1 MiB) is
 * REFUSED and reads as 0 -- it corrupts the output in 4K MTLB mode
 * (qi:N1541 D2).  The 1 MiB map path in etnaviv_iommu_v2.c is kept, unreachable
 * from the knob, for the N1541-b row.  (etnaviv_mmu.c)
 */
extern unsigned int npu_mmu_large_pages;
static inline unsigned int etnaviv_mmu_large_shift(void)
{
	return npu_mmu_large_pages == 2 ? 16 : 0;
}

#define ETNAVIV_PTA_SIZE	SZ_4K
#define ETNAVIV_PTA_ENTRIES	(ETNAVIV_PTA_SIZE / sizeof(u64))

struct etnaviv_iommu_global {
	struct device *dev;
	enum etnaviv_iommu_version version;
	const struct etnaviv_iommu_ops *ops;
	unsigned int use;
	struct mutex lock;

	void *bad_page_cpu;
	dma_addr_t bad_page_dma;

	u32 memory_base;

	/*
	 * AXI SRAM flat-map.
	 *
	 * NPU chips with `axi_sram_size > 0` (VIPNano-QI on A311D today)
	 * get a coherent DDR chunk allocated at global init and
	 * flat-mapped into every MMUv2 context at GPU VA
	 * `ETNAVIV_AXI_SRAM_VA`. Mesa's etnaviv ML driver reads the VA
	 * via DRM_ETNAVIV_PARAM_GPU_AXI_SRAM_START and emits
	 * `VIVS_GL_OCB_REMAP_START/END` accordingly.
	 *
	 * Null when the GPU has no AXI SRAM (S905D3 VIPPico v3) or when
	 * the DDR allocation failed at init. Userspace then sees
	 * AXI_SRAM_SIZE = 0 / AXI_SRAM_START = 0 and skips OCB_REMAP
	 * programming.
	 */
	void *axi_sram_cpu;
	dma_addr_t axi_sram_dma;
	size_t axi_sram_size;
	/*
	 * If non-NULL, the AXI SRAM backing came from an mmio-sram gen_pool
	 * reachable via the "sram" DT phandle on the GPU node -- that is,
	 * from real on-chip SRAM. NULL means the backing was DDR via
	 * dma_alloc_coherent (CI / silicon with axi_sram_size > 0 but no
	 * DT-provided SRAM pool). fini uses this to pick the right free.
	 */
	struct gen_pool *axi_sram_pool;

	/*
	 * Flop-reset DDR flat-map. A coherent DDR page allocated at global
	 * init (A311D only) and flat-mapped into every MMUv2 context at GPU
	 * VA ETNAVIV_FLOP_RESET_VA. Holds the A311D per-submit flop-reset
	 * NN descriptor + coef + in/out, which the per-submit emission
	 * (etnaviv_buffer_v8_qi.c) references from ANY client context. Must
	 * be DDR, not the AXI-SRAM flat-map: the NN cannot fetch a descriptor
	 * from the on-chip OCB region (audit y6 -- AXI-SRAM descriptor never
	 * completes, MC stuck busy). Null on non-A311D / alloc failure.
	 */
	void *flop_cpu;
	dma_addr_t flop_dma;
	size_t flop_size;

	/*
	 * This union holds members needed by either MMUv1 or MMUv2, which
	 * can not exist at the same time.
	 */
	union {
		struct {
			struct etnaviv_iommu_context *shared_context;
		} v1;
		struct {
			/* P(age) T(able) A(rray) */
			u64 *pta_cpu;
			dma_addr_t pta_dma;
			DECLARE_BITMAP(pta_alloc, ETNAVIV_PTA_ENTRIES);
			/*
			 * VIPPico v3 (0x99) only: GPU-owned page tables the
			 * MMU actually walks (etnaviv_iommu_v2.c,
			 * etnaviv_iommuv2_sync_hw_mtlb).  NULL elsewhere.
			 */
			struct etnaviv_iommuv2_hw_tables *hw;
		} v2;
	};
};

struct etnaviv_iommu_context {
	struct kref refcount;
	struct etnaviv_iommu_global *global;

	/* memory manager for GPU address area */
	struct mutex lock;
	struct list_head mappings;
	struct drm_mm mm;
	unsigned int flush_seq;

	/* Not part of the context, but needs to have the same lifetime */
	struct etnaviv_vram_mapping cmdbuf_mapping;

	/*
	 * AXI SRAM flat-map reservation at ETNAVIV_AXI_SRAM_VA. Reserved
	 * in context_init (iff global->axi_sram_cpu is non-NULL) so
	 * softpinned GEM allocs can't collide with the flat-map.
	 */
	struct drm_mm_node axi_sram_node;

	/*
	 * Flop-reset DDR flat-map reservation at ETNAVIV_FLOP_RESET_VA.
	 * Reserved in context_init (iff global->flop_cpu is non-NULL) so the
	 * A311D per-submit flop-reset descriptor is reachable from EVERY
	 * client context. DDR-backed (the NN cannot fetch a descriptor from
	 * the on-chip AXI-SRAM/OCB region -- audit y6).
	 */
	struct drm_mm_node flop_node;
};

int etnaviv_iommu_global_init(struct etnaviv_gpu *gpu);
void etnaviv_iommu_global_fini(struct etnaviv_gpu *gpu);

struct etnaviv_gem_object;

int etnaviv_iommu_map_gem(struct etnaviv_iommu_context *context,
	struct etnaviv_gem_object *etnaviv_obj, u32 memory_base,
	struct etnaviv_vram_mapping *mapping, u64 va);
void etnaviv_iommu_unmap_gem(struct etnaviv_iommu_context *context,
	struct etnaviv_vram_mapping *mapping);
void etnaviv_iommu_reap_mapping(struct etnaviv_vram_mapping *mapping);

int etnaviv_iommu_get_suballoc_va(struct etnaviv_iommu_context *ctx,
				  struct etnaviv_vram_mapping *mapping,
				  u32 memory_base, dma_addr_t paddr,
				  size_t size);

/*
 * AXI SRAM flat-map GPU VA. See `struct etnaviv_iommu_global`
 * axi_sram_* comment for the rationale. Value chosen to stay clear of
 * `ETNAVIV_SOFTPIN_START_ADDRESS + ETNAVIV_SOFTPIN_CAP` for typical
 * workloads (MobileNet v1 sums to a few MB of softpinned coefficients;
 * 64 MB of softpin headroom before AXI SRAM is ample). NOT the BSP
 * value (0x01000000) -- see ANALYSIS.md "VA layout collision".
 */
#define ETNAVIV_AXI_SRAM_VA	0x04000000

int etnaviv_iommu_install_axi_sram(struct etnaviv_iommu_context *ctx);

/*
 * Flop-reset DDR flat-map GPU VA. One page above the AXI SRAM window
 * (0x04000000 + up to ~1 MB), clear of softpin. Carries the A311D
 * per-submit flop-reset descriptor; DDR-backed so the NN can fetch it
 * (audit y6).
 */
#define ETNAVIV_FLOP_RESET_VA	0x05000000

int etnaviv_iommu_install_flop_reset(struct etnaviv_iommu_context *ctx);
void etnaviv_iommu_put_suballoc_va(struct etnaviv_iommu_context *ctx,
				   struct etnaviv_vram_mapping *mapping);

size_t etnaviv_iommu_dump_size(struct etnaviv_iommu_context *ctx);
void etnaviv_iommu_dump(struct etnaviv_iommu_context *ctx, void *buf);

struct etnaviv_iommu_context *
etnaviv_iommu_context_init(struct etnaviv_iommu_global *global,
			   struct etnaviv_cmdbuf_suballoc *suballoc);
static inline struct etnaviv_iommu_context *
etnaviv_iommu_context_get(struct etnaviv_iommu_context *ctx)
{
	kref_get(&ctx->refcount);
	return ctx;
}
void etnaviv_iommu_context_put(struct etnaviv_iommu_context *ctx);
void etnaviv_iommu_restore(struct etnaviv_gpu *gpu,
			   struct etnaviv_iommu_context *ctx);

struct etnaviv_iommu_context *
etnaviv_iommuv1_context_alloc(struct etnaviv_iommu_global *global);
struct etnaviv_iommu_context *
etnaviv_iommuv2_context_alloc(struct etnaviv_iommu_global *global);

u32 etnaviv_iommuv2_get_mtlb_addr(struct etnaviv_iommu_context *context);
unsigned short etnaviv_iommuv2_get_pta_id(struct etnaviv_iommu_context *context);
u32 etnaviv_iommuv2_sync_hw_mtlb(struct etnaviv_iommu_context *context);
void etnaviv_iommuv2_free_hw_tables(struct etnaviv_iommu_global *global);

#endif /* __ETNAVIV_MMU_H__ */
