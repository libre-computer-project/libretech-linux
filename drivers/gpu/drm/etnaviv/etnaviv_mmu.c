// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2015-2018 Etnaviv Project
 */

#include <linux/bitmap.h>
#include <linux/dma-mapping.h>
#include <linux/genalloc.h>
#include <linux/moduleparam.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include "common.xml.h"
#include "etnaviv_cmdbuf.h"
#include "etnaviv_drv.h"
#include "etnaviv_gem.h"
#include "etnaviv_gpu.h"
#include "etnaviv_mmu.h"

/*
 * qi:N1541 -- MMU page granularity over the NPU's DDR buffer objects.
 *
 * qi:N1539 measured, on person's three DDR-image ops (100 / 103 / 137), our
 * stack moving 133-158 % of the descriptor's bytes on the NNA DDR port where
 * the vendor moves 103-109 %, sized per plane-row ACCESS (2.32 / 1.52 / 1.30
 * sixteen-byte grants each) on a 15,360 B plane stride -- every access lands
 * on a different 4 KiB page, so page-table traffic is per access and
 * qi:N1439's one-walk-per-page bound does not apply.  galcore maps this same
 * pool on this same MMU with 1 MiB pages (gc_hal_kernel_mmu.c), etnaviv with
 * 4 KiB only.  This is the A/B.
 *
 * npu_mmu_large_pages = 0           -- 4 KiB PTEs, upstream byte for byte.
 *                       1           -- REFUSED (treated as 0).  qi:N1541 D2:
 *                                      1 MiB PTEs in etnaviv's 4K MTLB mode
 *                                      resolve to valid memory that is not the
 *                                      right memory -- output wrong on 5 of 5,
 *                                      zero faults.  galcore ships in 1K mode.
 *                       2 (default) -- 64 KiB PTEs (encoded in galcore's MTLB
 *                                      table, never exercised by it at runtime).
 *                                      qi:N1543: person [96,168) stage 63.6 %
 *                                      of 4 KiB, per-inference wall 88.2 %, rfb
 *                                      98.8 %, yolov3 98.3 %, every digest
 *                                      intact; the default flipped there.
 *
 * The default flip reaches a graph ONLY when its DDR pool is first touched by
 * the submit that carries its softpin VA (see the phase note below): with the
 * runtime's zero-memset at arena creation still in place the pool's pages are
 * allocated at phase 0 and only whole-slot BOs whose VA phase is already a
 * 64 KiB multiple (person's LCD, not its coefficient pool) get large entries --
 * measured inert on the wall (qi:N1543 arm Z).  The userspace half is
 * etnaviv_ml's arena zeroing (ETML_NO_ZERO_ARENAS) or a 64 KiB-aligned softpin.
 *
 * What a large PTE needs and how it is obtained:
 *  - physically contiguous, naturally aligned backing per large page: the knob
 *    implies qi:N1440's contiguous allocator for every non-cached BO of at least
 *    one large page (etnaviv_gem.c), PHASE-MATCHED to the BO's softpin VA --
 *    the VA is 4 KiB-granular and top-down (person's 15 MiB COEFF_POOL sits at
 *    0xfd3eb000), so block boundaries must fall on the VA's large-page
 *    boundaries, not on the BO's offset 0.  That is only possible when the pages
 *    are allocated at the first MAPPING (where the VA is known) rather than at
 *    a CPU fault; the runtime's zero-memset of a fresh arena is the one earlier
 *    touch and ETML_NO_ZERO_ARENAS=1 removes it (fresh pages are zero anyway).
 *  - one page size per 4 MiB MTLB slot: a slot is typed large ONLY when THIS
 *    mapping covers the whole slot and every large page in it is backed
 *    contiguously; head / tail slots a mapping shares with other BOs stay 4 KiB.
 *    Anything not eligible falls back to 4 KiB, per page, never per BO.
 *
 * One dev_info per mapping of at least one large page reports the entry counts
 * written, whatever the knob reads, so a patched-but-off arm prints
 * `1M=0 ... 4K=N` and the shipped module prints nothing.
 */
unsigned int npu_mmu_large_pages = 2;
module_param_named(npu_mmu_large_pages, npu_mmu_large_pages, uint, 0644);
MODULE_PARM_DESC(npu_mmu_large_pages,
	"0 = 4 KiB MMU pages (upstream); 2 = 64 KiB pages over contiguous VA-phase-matched backing where a mapping owns whole 4 MiB MTLB slots (default, qi:N1543); 1 = 1 MiB pages is REFUSED (corrupts in 4K MTLB mode, qi:N1541) and reads as 0");

static unsigned int npu_mmu_page_shift(void)
{
	switch (npu_mmu_large_pages) {
	case 2:  return 16;
	default: return 0;	/* 0, and the refused 1 (qi:N1541 D2) */
	}
}

static void etnaviv_context_unmap(struct etnaviv_iommu_context *context,
				 unsigned long iova, size_t size)
{
	size_t unmapped_page, unmapped = 0;
	size_t pgsize = SZ_4K;

	while (unmapped < size) {
		unmapped_page = context->global->ops->unmap(context, iova,
							    pgsize);
		if (!unmapped_page)
			break;

		iova += unmapped_page;
		unmapped += unmapped_page;
	}
}

static int etnaviv_context_map_pg(struct etnaviv_iommu_context *context,
				  unsigned long iova, phys_addr_t paddr,
				  size_t size, int prot, size_t pgsize)
{
	unsigned long orig_iova = iova;
	size_t orig_size = size;
	int ret = 0;

	while (size) {
		ret = context->global->ops->map(context, iova, paddr, pgsize,
						prot);
		if (ret)
			break;

		iova += pgsize;
		paddr += pgsize;
		size -= pgsize;
	}

	/* unroll mapping in case something went wrong */
	if (ret)
		etnaviv_context_unmap(context, orig_iova, orig_size - size);

	return ret;
}

static int etnaviv_context_map(struct etnaviv_iommu_context *context,
			      unsigned long iova, phys_addr_t paddr,
			      size_t size, int prot)
{
	return etnaviv_context_map_pg(context, iova, paddr, size, prot, SZ_4K);
}

/* qi:N1541 -- one physically contiguous run of the mapping */
struct etnaviv_map_run {
	u32 da;
	phys_addr_t pa;
	u32 len;
};

/*
 * Is the large page at VA `page` (size 1 << shift) backed by ONE run,
 * contiguously, at a naturally aligned physical address?
 */
static bool etnaviv_run_backs_page(const struct etnaviv_map_run *runs,
				   unsigned int nruns, u32 page,
				   unsigned int shift)
{
	u64 pg = 1ull << shift;
	unsigned int i;

	for (i = 0; i < nruns; i++) {
		const struct etnaviv_map_run *r = &runs[i];

		if (r->da > page)
			break;
		if ((u64)r->da + r->len < (u64)page + pg)
			continue;
		return IS_ALIGNED(r->pa + (page - r->da), pg);
	}
	return false;
}

/*
 * qi:N1541 -- map with large pages where the mapping OWNS whole MTLB slots
 * and the backing allows it, 4 KiB everywhere else.  `runs` is the mapping's
 * sg list coalesced into physically contiguous runs, in VA order.
 */
static int etnaviv_iommu_map_large(struct etnaviv_iommu_context *context,
				   u32 iova, u32 va_len,
				   const struct etnaviv_map_run *runs,
				   unsigned int nruns, int prot,
				   unsigned int shift, u32 *mapped_len,
				   unsigned int *n_large, unsigned int *n_small,
				   u32 *first_slot, u32 *last_slot)
{
	const u32 slot = 1u << 22;		/* MMUv2 4K-mode MTLB span */
	u32 pg = 1u << shift;
	/* u64: a mapping ending at the top of the 4 GiB space must not wrap */
	u64 lo = ALIGN((u64)iova, slot);
	u64 hi = ALIGN_DOWN((u64)iova + va_len, slot);
	unsigned long *ok = NULL;
	unsigned int nslots = 0, i;
	int ret = 0;

	*n_large = *n_small = 0;
	*first_slot = *last_slot = 0;
	*mapped_len = 0;

	/* which whole slots of this mapping are fully large-page backed */
	if (hi > lo) {
		nslots = (hi - lo) / slot;
		ok = bitmap_zalloc(nslots, GFP_KERNEL);
		if (!ok)
			return -ENOMEM;
		for (i = 0; i < nslots; i++) {
			u64 s = lo + (u64)i * slot, p;
			bool all = true;

			for (p = s; p < s + slot; p += pg)
				if (!etnaviv_run_backs_page(runs, nruns, (u32)p, shift)) {
					all = false;
					break;
				}
			if (all) {
				__set_bit(i, ok);
				if (!*first_slot)
					*first_slot = (u32)s;
				*last_slot = (u32)(s + slot - 1);
			}
		}
	}

	for (i = 0; i < nruns && !ret; i++) {
		u64 da = runs[i].da, end = (u64)runs[i].da + runs[i].len;
		phys_addr_t pa = runs[i].pa;

		while (da < end) {
			u64 seg_end = min(end, ALIGN(da + 1, (u64)slot));
			bool large = da >= lo && da < hi &&
				     test_bit((da - lo) / slot, ok);
			size_t ps = large ? pg : SZ_4K;

			ret = etnaviv_context_map_pg(context, da, pa,
						     seg_end - da, prot, ps);
			if (ret)
				break;
			if (large)
				*n_large += (seg_end - da) / pg;
			else
				*n_small += (seg_end - da) / SZ_4K;
			*mapped_len += seg_end - da;
			pa += seg_end - da;
			da = seg_end;
		}
	}

	bitmap_free(ok);
	return ret;
}

static int etnaviv_iommu_map(struct etnaviv_iommu_context *context,
			     u32 iova, unsigned int va_len,
			     struct sg_table *sgt, int prot)
{
	struct scatterlist *sg;
	struct etnaviv_map_run *runs = NULL;
	unsigned int nruns = 0, shift = npu_mmu_page_shift();
	unsigned int da = iova;
	unsigned int i;
	int ret;

	if (!context || !sgt)
		return -EINVAL;

	/*
	 * MMUv2 always goes through the run list so the per-mapping entry-count
	 * line is printed at EVERY knob value (a patched-but-off arm reads
	 * `1024K=0 4K=N`); the shipped module has no such line.
	 */
	if (context->global->version == ETNAVIV_IOMMU_V2) {
		runs = kvcalloc(sgt->nents, sizeof(*runs), GFP_KERNEL);
		if (!runs)
			return -ENOMEM;
	}

	for_each_sgtable_dma_sg(sgt, sg, i) {
		phys_addr_t pa = sg_dma_address(sg);
		unsigned int da_len = sg_dma_len(sg);
		unsigned int bytes = min_t(unsigned int, da_len, va_len);

		VERB("map[%d]: %08x %pap(%x)", i, da, &pa, bytes);

		if (!IS_ALIGNED(iova | pa | bytes, SZ_4K)) {
			dev_err(context->global->dev,
				"unaligned: iova 0x%x pa %pa size 0x%x\n",
				iova, &pa, bytes);
			ret = -EINVAL;
			goto fail;
		}

		if (runs) {
			/* coalesce physically contiguous neighbours */
			if (nruns && runs[nruns - 1].pa + runs[nruns - 1].len == pa &&
			    runs[nruns - 1].da + runs[nruns - 1].len == da) {
				runs[nruns - 1].len += bytes;
			} else {
				runs[nruns].da = da;
				runs[nruns].pa = pa;
				runs[nruns].len = bytes;
				nruns++;
			}
		} else {
			ret = etnaviv_context_map(context, da, pa, bytes, prot);
			if (ret)
				goto fail;
		}

		va_len -= bytes;
		da += bytes;
	}

	if (runs) {
		unsigned int n_large = 0, n_small = 0;
		u32 mapped = 0, s0 = 0, s1 = 0, total = da - iova;

		if (shift) {
			ret = etnaviv_iommu_map_large(context, iova, total, runs,
						      nruns, prot, shift, &mapped,
						      &n_large, &n_small, &s0, &s1);
		} else {
			for (i = 0; i < nruns; i++) {
				ret = etnaviv_context_map(context, runs[i].da,
							  runs[i].pa, runs[i].len,
							  prot);
				if (ret)
					break;
				mapped += runs[i].len;
				n_small += runs[i].len / SZ_4K;
			}
		}
		if (ret) {
			da = iova + mapped;
			goto fail;
		}
		if (total >= SZ_1M || n_large)
			dev_info(context->global->dev,
				 "npu_mmu_large_pages=%u: %u B mapping iova 0x%08x (phase 0x%05x, %u runs): entries %uK=%u 4K=%u, large slots 0x%08x..0x%08x\n",
				 npu_mmu_large_pages, total, iova,
				 iova & (SZ_1M - 1), nruns,
				 shift ? (1u << shift) >> 10 : 1024,
				 n_large, n_small, s0, s1);
		kvfree(runs);
	}

	context->flush_seq++;

	return 0;

fail:
	kvfree(runs);
	etnaviv_context_unmap(context, iova, da - iova);
	return ret;
}

static void etnaviv_iommu_unmap(struct etnaviv_iommu_context *context, u32 iova,
				struct sg_table *sgt, unsigned len)
{
	etnaviv_context_unmap(context, iova, len);

	context->flush_seq++;
}

static void etnaviv_iommu_remove_mapping(struct etnaviv_iommu_context *context,
	struct etnaviv_vram_mapping *mapping)
{
	struct etnaviv_gem_object *etnaviv_obj = mapping->object;

	lockdep_assert_held(&context->lock);

	etnaviv_iommu_unmap(context, mapping->vram_node.start,
			    etnaviv_obj->sgt, etnaviv_obj->size);
	drm_mm_remove_node(&mapping->vram_node);
}

void etnaviv_iommu_reap_mapping(struct etnaviv_vram_mapping *mapping)
{
	struct etnaviv_iommu_context *context = mapping->context;

	lockdep_assert_held(&context->lock);
	WARN_ON(mapping->use);

	etnaviv_iommu_remove_mapping(context, mapping);
	etnaviv_iommu_context_put(mapping->context);
	mapping->context = NULL;
	list_del_init(&mapping->mmu_node);
}

static int etnaviv_iommu_find_iova(struct etnaviv_iommu_context *context,
				   struct drm_mm_node *node, size_t size)
{
	struct etnaviv_vram_mapping *free = NULL;
	enum drm_mm_insert_mode mode = DRM_MM_INSERT_LOW;
	int ret;

	lockdep_assert_held(&context->lock);

	while (1) {
		struct etnaviv_vram_mapping *m, *n;
		struct drm_mm_scan scan;
		struct list_head list;
		bool found;

		ret = drm_mm_insert_node_in_range(&context->mm, node,
						  size, 0, 0, 0, U64_MAX, mode);
		if (ret != -ENOSPC)
			break;

		/* Try to retire some entries */
		drm_mm_scan_init(&scan, &context->mm, size, 0, 0, mode);

		found = 0;
		INIT_LIST_HEAD(&list);
		list_for_each_entry(free, &context->mappings, mmu_node) {
			/* If this vram node has not been used, skip this. */
			if (!free->vram_node.mm)
				continue;

			/*
			 * If the iova is pinned, then it's in-use,
			 * so we must keep its mapping.
			 */
			if (free->use)
				continue;

			list_add(&free->scan_node, &list);
			if (drm_mm_scan_add_block(&scan, &free->vram_node)) {
				found = true;
				break;
			}
		}

		if (!found) {
			/* Nothing found, clean up and fail */
			list_for_each_entry_safe(m, n, &list, scan_node)
				BUG_ON(drm_mm_scan_remove_block(&scan, &m->vram_node));
			break;
		}

		/*
		 * drm_mm does not allow any other operations while
		 * scanning, so we have to remove all blocks first.
		 * If drm_mm_scan_remove_block() returns false, we
		 * can leave the block pinned.
		 */
		list_for_each_entry_safe(m, n, &list, scan_node)
			if (!drm_mm_scan_remove_block(&scan, &m->vram_node))
				list_del_init(&m->scan_node);

		/*
		 * Unmap the blocks which need to be reaped from the MMU.
		 * Clear the mmu pointer to prevent the mapping_get finding
		 * this mapping.
		 */
		list_for_each_entry_safe(m, n, &list, scan_node) {
			etnaviv_iommu_reap_mapping(m);
			list_del_init(&m->scan_node);
		}

		mode = DRM_MM_INSERT_EVICT;

		/*
		 * We removed enough mappings so that the new allocation will
		 * succeed, retry the allocation one more time.
		 */
	}

	return ret;
}

static int etnaviv_iommu_insert_exact(struct etnaviv_iommu_context *context,
		   struct drm_mm_node *node, size_t size, u64 va)
{
	struct etnaviv_vram_mapping *m, *n;
	struct drm_mm_node *scan_node;
	LIST_HEAD(scan_list);
	int ret;

	lockdep_assert_held(&context->lock);

	ret = drm_mm_insert_node_in_range(&context->mm, node, size, 0, 0, va,
					  va + size, DRM_MM_INSERT_LOWEST);
	if (ret != -ENOSPC)
		return ret;

	/*
	 * When we can't insert the node, due to a existing mapping blocking
	 * the address space, there are two possible reasons:
	 * 1. Userspace genuinely messed up and tried to reuse address space
	 * before the last job using this VMA has finished executing.
	 * 2. The existing buffer mappings are idle, but the buffers are not
	 * destroyed yet (likely due to being referenced by another context) in
	 * which case the mappings will not be cleaned up and we must reap them
	 * here to make space for the new mapping.
	 */

	drm_mm_for_each_node_in_range(scan_node, &context->mm, va, va + size) {
		m = container_of(scan_node, struct etnaviv_vram_mapping,
				 vram_node);

		if (m->use)
			return -ENOSPC;

		list_add(&m->scan_node, &scan_list);
	}

	list_for_each_entry_safe(m, n, &scan_list, scan_node) {
		etnaviv_iommu_reap_mapping(m);
		list_del_init(&m->scan_node);
	}

	return drm_mm_insert_node_in_range(&context->mm, node, size, 0, 0, va,
					   va + size, DRM_MM_INSERT_LOWEST);
}

int etnaviv_iommu_map_gem(struct etnaviv_iommu_context *context,
	struct etnaviv_gem_object *etnaviv_obj, u32 memory_base,
	struct etnaviv_vram_mapping *mapping, u64 va)
{
	struct sg_table *sgt = etnaviv_obj->sgt;
	struct drm_mm_node *node;
	int ret;

	lockdep_assert_held(&etnaviv_obj->lock);

	mutex_lock(&context->lock);

	/* v1 MMU can optimize single entry (contiguous) scatterlists */
	if (context->global->version == ETNAVIV_IOMMU_V1 &&
	    sgt->nents == 1 && !(etnaviv_obj->flags & ETNA_BO_FORCE_MMU)) {
		u32 iova;

		iova = sg_dma_address(sgt->sgl) - memory_base;
		if (iova < 0x80000000 - sg_dma_len(sgt->sgl)) {
			mapping->iova = iova;
			mapping->context = etnaviv_iommu_context_get(context);
			list_add_tail(&mapping->mmu_node, &context->mappings);
			ret = 0;
			goto unlock;
		}
	}

	node = &mapping->vram_node;

	if (va)
		ret = etnaviv_iommu_insert_exact(context, node, etnaviv_obj->size, va);
	else
		ret = etnaviv_iommu_find_iova(context, node, etnaviv_obj->size);
	if (ret < 0)
		goto unlock;

	mapping->iova = node->start;
	ret = etnaviv_iommu_map(context, node->start, etnaviv_obj->size, sgt,
				ETNAVIV_PROT_READ | ETNAVIV_PROT_WRITE);

	if (ret < 0) {
		drm_mm_remove_node(node);
		goto unlock;
	}

	mapping->context = etnaviv_iommu_context_get(context);
	list_add_tail(&mapping->mmu_node, &context->mappings);
unlock:
	mutex_unlock(&context->lock);

	return ret;
}

void etnaviv_iommu_unmap_gem(struct etnaviv_iommu_context *context,
	struct etnaviv_vram_mapping *mapping)
{
	WARN_ON(mapping->use);

	mutex_lock(&context->lock);

	/* Bail if the mapping has been reaped by another thread */
	if (!mapping->context) {
		mutex_unlock(&context->lock);
		return;
	}

	/* If the vram node is on the mm, unmap and remove the node */
	if (mapping->vram_node.mm == &context->mm)
		etnaviv_iommu_remove_mapping(context, mapping);

	list_del(&mapping->mmu_node);
	mutex_unlock(&context->lock);
	etnaviv_iommu_context_put(context);
}

static void etnaviv_iommu_context_free(struct kref *kref)
{
	struct etnaviv_iommu_context *context =
		container_of(kref, struct etnaviv_iommu_context, refcount);

	etnaviv_cmdbuf_suballoc_unmap(context, &context->cmdbuf_mapping);
	mutex_destroy(&context->lock);
	context->global->ops->free(context);
}
void etnaviv_iommu_context_put(struct etnaviv_iommu_context *context)
{
	kref_put(&context->refcount, etnaviv_iommu_context_free);
}

/*
 * Reserve and flat-map the AXI SRAM flat-map region at
 * ETNAVIV_AXI_SRAM_VA in a freshly-created MMUv2 context. Call after
 * context_alloc (drm_mm is initialised) and before the context is
 * handed back to the caller.
 *
 * Returns 0 on success, -errno on failure. Callers that don't care
 * about AXI SRAM (MMUv1 contexts, or global->axi_sram_cpu == NULL)
 * skip this call entirely.
 */
int etnaviv_iommu_install_axi_sram(struct etnaviv_iommu_context *ctx)
{
	struct etnaviv_iommu_global *global = ctx->global;
	int ret;

	if (!global->axi_sram_cpu)
		return 0;

	ctx->axi_sram_node.start = ETNAVIV_AXI_SRAM_VA;
	ctx->axi_sram_node.size  = global->axi_sram_size;
	ctx->axi_sram_node.color = 0;

	ret = drm_mm_reserve_node(&ctx->mm, &ctx->axi_sram_node);
	if (ret) {
		dev_warn(global->dev,
			 "AXI SRAM flat-map: drm_mm reserve failed at VA 0x%08x: %d\n",
			 (u32)ETNAVIV_AXI_SRAM_VA, ret);
		return ret;
	}

	ret = etnaviv_context_map(ctx, ETNAVIV_AXI_SRAM_VA,
				  global->axi_sram_dma,
				  global->axi_sram_size,
				  ETNAVIV_PROT_READ | ETNAVIV_PROT_WRITE);
	if (ret) {
		drm_mm_remove_node(&ctx->axi_sram_node);
		dev_warn(global->dev,
			 "AXI SRAM flat-map: iommu map failed at VA 0x%08x: %d\n",
			 (u32)ETNAVIV_AXI_SRAM_VA, ret);
		return ret;
	}

	ctx->flush_seq++;
	return 0;
}

/*
 * Reserve and flat-map the flop-reset DDR page at ETNAVIV_FLOP_RESET_VA
 * in a freshly-created MMUv2 context, so the A311D per-submit flop-reset
 * descriptor is reachable from every client context. Models
 * etnaviv_iommu_install_axi_sram but DDR-backed (audit y6). Skipped when
 * global->flop_cpu == NULL (non-A311D / alloc failed).
 */
int etnaviv_iommu_install_flop_reset(struct etnaviv_iommu_context *ctx)
{
	struct etnaviv_iommu_global *global = ctx->global;
	int ret;

	if (!global->flop_cpu)
		return 0;

	ctx->flop_node.start = ETNAVIV_FLOP_RESET_VA;
	ctx->flop_node.size  = global->flop_size;
	ctx->flop_node.color = 0;

	ret = drm_mm_reserve_node(&ctx->mm, &ctx->flop_node);
	if (ret) {
		dev_warn(global->dev,
			 "flop-reset flat-map: drm_mm reserve failed at VA 0x%08x: %d\n",
			 (u32)ETNAVIV_FLOP_RESET_VA, ret);
		return ret;
	}

	ret = etnaviv_context_map(ctx, ETNAVIV_FLOP_RESET_VA,
				  global->flop_dma,
				  global->flop_size,
				  ETNAVIV_PROT_READ | ETNAVIV_PROT_WRITE);
	if (ret) {
		drm_mm_remove_node(&ctx->flop_node);
		dev_warn(global->dev,
			 "flop-reset flat-map: iommu map failed at VA 0x%08x: %d\n",
			 (u32)ETNAVIV_FLOP_RESET_VA, ret);
		return ret;
	}

	ctx->flush_seq++;
	return 0;
}

struct etnaviv_iommu_context *
etnaviv_iommu_context_init(struct etnaviv_iommu_global *global,
			   struct etnaviv_cmdbuf_suballoc *suballoc)
{
	struct etnaviv_iommu_context *ctx;
	int ret;

	if (global->version == ETNAVIV_IOMMU_V1)
		ctx = etnaviv_iommuv1_context_alloc(global);
	else
		ctx = etnaviv_iommuv2_context_alloc(global);

	if (!ctx)
		return NULL;

	ret = etnaviv_cmdbuf_suballoc_map(suballoc, ctx, &ctx->cmdbuf_mapping,
					  global->memory_base);
	if (ret)
		goto out_free;

	if (global->version == ETNAVIV_IOMMU_V1 &&
	    ctx->cmdbuf_mapping.iova > 0x80000000) {
		dev_err(global->dev,
		        "command buffer outside valid memory window\n");
		goto out_unmap;
	}

	/*
	 * AXI SRAM flat-map -- best effort. Failure is non-fatal: the
	 * context still works for normal GEM submits; only OCB_REMAP-
	 * dependent NPU paths (V8 SRAM caching, A311D NN flop reset)
	 * will see AXI_SRAM_START = 0 and skip their programming.
	 */
	if (global->version == ETNAVIV_IOMMU_V2) {
		etnaviv_iommu_install_axi_sram(ctx);
		etnaviv_iommu_install_flop_reset(ctx);
	}

	return ctx;

out_unmap:
	etnaviv_cmdbuf_suballoc_unmap(ctx, &ctx->cmdbuf_mapping);
out_free:
	global->ops->free(ctx);
	return NULL;
}

void etnaviv_iommu_restore(struct etnaviv_gpu *gpu,
			   struct etnaviv_iommu_context *context)
{
	context->global->ops->restore(gpu, context);
}

int etnaviv_iommu_get_suballoc_va(struct etnaviv_iommu_context *context,
				  struct etnaviv_vram_mapping *mapping,
				  u32 memory_base, dma_addr_t paddr,
				  size_t size)
{
	mutex_lock(&context->lock);

	if (mapping->use > 0) {
		mapping->use++;
		mutex_unlock(&context->lock);
		return 0;
	}

	/*
	 * For MMUv1 we don't add the suballoc region to the pagetables, as
	 * those GPUs can only work with cmdbufs accessed through the linear
	 * window. Instead we manufacture a mapping to make it look uniform
	 * to the upper layers.
	 */
	if (context->global->version == ETNAVIV_IOMMU_V1) {
		mapping->iova = paddr - memory_base;
	} else {
		struct drm_mm_node *node = &mapping->vram_node;
		int ret;

		ret = etnaviv_iommu_find_iova(context, node, size);
		if (ret < 0) {
			mutex_unlock(&context->lock);
			return ret;
		}

		mapping->iova = node->start;
		ret = etnaviv_context_map(context, node->start, paddr, size,
					  ETNAVIV_PROT_READ | ETNAVIV_PROT_WRITE);
		if (ret < 0) {
			drm_mm_remove_node(node);
			mutex_unlock(&context->lock);
			return ret;
		}

		context->flush_seq++;
	}

	list_add_tail(&mapping->mmu_node, &context->mappings);
	mapping->use = 1;

	mutex_unlock(&context->lock);

	return 0;
}

void etnaviv_iommu_put_suballoc_va(struct etnaviv_iommu_context *context,
		  struct etnaviv_vram_mapping *mapping)
{
	struct drm_mm_node *node = &mapping->vram_node;

	mutex_lock(&context->lock);
	mapping->use--;

	if (mapping->use > 0 || context->global->version == ETNAVIV_IOMMU_V1) {
		mutex_unlock(&context->lock);
		return;
	}

	etnaviv_context_unmap(context, node->start, node->size);
	drm_mm_remove_node(node);
	mutex_unlock(&context->lock);
}

size_t etnaviv_iommu_dump_size(struct etnaviv_iommu_context *context)
{
	return context->global->ops->dump_size(context);
}

void etnaviv_iommu_dump(struct etnaviv_iommu_context *context, void *buf)
{
	context->global->ops->dump(context, buf);
}

int etnaviv_iommu_global_init(struct etnaviv_gpu *gpu)
{
	enum etnaviv_iommu_version version = ETNAVIV_IOMMU_V1;
	struct etnaviv_drm_private *priv = gpu->drm->dev_private;
	struct etnaviv_iommu_global *global;
	struct device *dev = gpu->drm->dev;

	if (gpu->identity.minor_features1 & chipMinorFeatures1_MMU_VERSION)
		version = ETNAVIV_IOMMU_V2;

	if (priv->mmu_global) {
		if (priv->mmu_global->version != version) {
			dev_err(gpu->dev,
				"MMU version doesn't match global version\n");
			return -ENXIO;
		}

		priv->mmu_global->use++;
		return 0;
	}

	global = kzalloc(sizeof(*global), GFP_KERNEL);
	if (!global)
		return -ENOMEM;

	global->bad_page_cpu = dma_alloc_wc(dev, SZ_4K, &global->bad_page_dma,
					    GFP_KERNEL);
	if (!global->bad_page_cpu)
		goto free_global;

	memset32(global->bad_page_cpu, 0xdead55aa, SZ_4K / sizeof(u32));

	if (version == ETNAVIV_IOMMU_V2) {
		global->v2.pta_cpu = dma_alloc_wc(dev, ETNAVIV_PTA_SIZE,
					       &global->v2.pta_dma, GFP_KERNEL);
		if (!global->v2.pta_cpu)
			goto free_bad_page;
	}

	/*
	 * AXI SRAM flat-map backing: mapped into each MMUv2 context at
	 * ETNAVIV_AXI_SRAM_VA by etnaviv_iommu_install_axi_sram.
	 *
	 * Two sources, in preference order:
	 *   1. On-chip AXI SRAM advertised by an mmio-sram gen_pool reached
	 *      via the GPU node's "sram" phandle. A311D has 1 MB at phys
	 *      0xFF000000; putting the flat-map there lets hot NN BOs live
	 *      in on-die SRAM instead of DDR, attacking the DDR-bandwidth
	 *      bottleneck on the FC layer.
	 *   2. Coherent DDR via dma_alloc_coherent -- the historical
	 *      fallback. Functional but carries DDR latency. Kept so
	 *      pre-SRAM-DT kernels, or platforms where the SRAM is reserved
	 *      for other masters, still benefit from the flat-map window.
	 *
	 * Skipped entirely on chips with axi_sram_size == 0 (e.g. S905D3).
	 * On soft failure the size is left at zero and OCB_REMAP programming
	 * is disabled userspace-side.
	 */
	if (version == ETNAVIV_IOMMU_V2 && gpu->identity.axi_sram_size) {
		size_t sz = gpu->identity.axi_sram_size;

		/*
		 * Use gpu->dev->of_node, not dev->of_node: dev here is the
		 * DRM aggregate device (has no DT node with the SRAM
		 * phandle), while gpu->dev is the per-GPU platform device
		 * whose of_node is npu@ff100000 carrying "sram = <...>".
		 */
		global->axi_sram_pool = of_gen_pool_get(gpu->dev->of_node,
							"sram", 0);
		if (global->axi_sram_pool) {
			unsigned long va = gen_pool_alloc(global->axi_sram_pool,
							  sz);
			if (va) {
				global->axi_sram_cpu = (void *)va;
				global->axi_sram_dma =
					gen_pool_virt_to_phys(
						global->axi_sram_pool, va);
				global->axi_sram_size = sz;
				dev_info(dev,
					 "AXI SRAM flat-map: %zu bytes @ phys 0x%llx (on-chip mmio-sram pool)\n",
					 sz, (unsigned long long)global->axi_sram_dma);
			} else {
				dev_warn(dev,
					 "AXI SRAM flat-map: on-chip gen_pool_alloc(%zu) failed; falling back to DDR\n",
					 sz);
				global->axi_sram_pool = NULL;
			}
		}

		if (!global->axi_sram_cpu) {
			global->axi_sram_cpu = dma_alloc_coherent(dev, sz,
				&global->axi_sram_dma, GFP_KERNEL);
			if (global->axi_sram_cpu) {
				global->axi_sram_size = sz;
				dev_info(dev,
					 "AXI SRAM flat-map: %zu bytes DDR-backed (no on-chip SRAM pool)\n",
					 sz);
			} else {
				dev_warn(dev,
					 "AXI SRAM flat-map: %zu-byte alloc failed; OCB_REMAP disabled\n",
					 sz);
			}
		}
	}

	/*
	 * Flop-reset DDR page (A311D only). One coherent DDR page,
	 * flat-mapped into every MMUv2 context at ETNAVIV_FLOP_RESET_VA by
	 * etnaviv_iommu_install_flop_reset, holding the per-submit flop-reset
	 * descriptor. DDR (not the AXI-SRAM/OCB flat-map) because the NN
	 * cannot fetch a descriptor from on-chip OCB (audit y6). Soft failure
	 * leaves flop_cpu NULL and the per-submit emission stays disabled.
	 */
	if (version == ETNAVIV_IOMMU_V2 &&
	    gpu->identity.customer_id == 0x88) {
		global->flop_cpu = dma_alloc_coherent(dev, PAGE_SIZE,
						      &global->flop_dma,
						      GFP_KERNEL);
		if (global->flop_cpu) {
			global->flop_size = PAGE_SIZE;
			dev_info(dev,
				 "flop-reset flat-map: %lu bytes DDR @ dma 0x%llx -> VA 0x%08x\n",
				 PAGE_SIZE,
				 (unsigned long long)global->flop_dma,
				 (u32)ETNAVIV_FLOP_RESET_VA);
		} else {
			dev_warn(dev,
				 "flop-reset flat-map: page alloc failed; A311D per-submit flop-reset disabled\n");
		}
	}

	global->dev = dev;
	global->version = version;
	global->use = 1;
	mutex_init(&global->lock);

	if (version == ETNAVIV_IOMMU_V1)
		global->ops = &etnaviv_iommuv1_ops;
	else
		global->ops = &etnaviv_iommuv2_ops;

	priv->mmu_global = global;

	return 0;

free_bad_page:
	dma_free_wc(dev, SZ_4K, global->bad_page_cpu, global->bad_page_dma);
free_global:
	kfree(global);

	return -ENOMEM;
}

void etnaviv_iommu_global_fini(struct etnaviv_gpu *gpu)
{
	struct etnaviv_drm_private *priv = gpu->drm->dev_private;
	struct etnaviv_iommu_global *global = priv->mmu_global;

	if (!global)
		return;

	if (--global->use > 0)
		return;

	if (global->v2.pta_cpu)
		dma_free_wc(global->dev, ETNAVIV_PTA_SIZE,
			    global->v2.pta_cpu, global->v2.pta_dma);

	if (global->v2.hw)
		etnaviv_iommuv2_free_hw_tables(global);

	if (global->bad_page_cpu)
		dma_free_wc(global->dev, SZ_4K,
			    global->bad_page_cpu, global->bad_page_dma);

	if (global->axi_sram_cpu) {
		if (global->axi_sram_pool)
			gen_pool_free(global->axi_sram_pool,
				      (unsigned long)global->axi_sram_cpu,
				      global->axi_sram_size);
		else
			dma_free_coherent(global->dev, global->axi_sram_size,
					  global->axi_sram_cpu,
					  global->axi_sram_dma);
	}

	if (global->flop_cpu)
		dma_free_coherent(global->dev, global->flop_size,
				  global->flop_cpu, global->flop_dma);

	mutex_destroy(&global->lock);
	kfree(global);

	priv->mmu_global = NULL;
}
