// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2016-2018 Etnaviv Project
 */

#include <linux/bitops.h>
#include <linux/dma-mapping.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include "etnaviv_cmdbuf.h"
#include "etnaviv_gpu.h"
#include "etnaviv_mmu.h"
#include "state.xml.h"
#include "state_hi.xml.h"

#define MMUv2_PTE_PRESENT		BIT(0)
#define MMUv2_PTE_EXCEPTION		BIT(1)
#define MMUv2_PTE_WRITEABLE		BIT(2)

#define MMUv2_MTLB_MASK			0xffc00000
#define MMUv2_MTLB_SHIFT		22
#define MMUv2_STLB_MASK			0x003ff000
#define MMUv2_STLB_SHIFT		12

#define MMUv2_MAX_STLB_ENTRIES		1024

/*
 * qi:N1541 -- MTLB entry page-size selector, bits [3:2].  Read from the
 * vendor driver that drives this same MMU on this same silicon
 * (gc_hal_kernel_mmu.h: gcdMMU_MTLB_4K_PAGE (0 << 2), gcdMMU_MTLB_64K_PAGE
 * (1 << 2), gcdMMU_MTLB_1M_PAGE (2 << 2); gc_hal_kernel_mmu.c _SetupAddressArea
 * writes `address | (1 << 3)` for its 1 MiB areas).  In the 4K-mode geometry
 * etnaviv programs (MTLB_SHIFT 22: one MTLB entry spans 4 MiB) a 1 MiB-page
 * STLB holds 4 entries and a 64 KiB-page STLB 64; the STLB entry format is the
 * same as the 4 KiB one (galcore _SetPage: address | ext << 4 | writable << 2
 * | present).  The size selector is written per MTLB slot, so ONE page size
 * rules a whole 4 MiB slot.
 */
#define MMUv2_MTLB_PAGE_4K		(0 << 2)
#define MMUv2_MTLB_PAGE_64K		(1 << 2)
#define MMUv2_MTLB_PAGE_1M		(2 << 2)

struct etnaviv_iommuv2_context {
	struct etnaviv_iommu_context base;
	unsigned short id;
	/* M(aster) TLB aka first level pagetable */
	u32 *mtlb_cpu;
	dma_addr_t mtlb_dma;
	/* S(lave) TLB aka second level pagetable */
	u32 *stlb_cpu[MMUv2_MAX_STLB_ENTRIES];
	dma_addr_t stlb_dma[MMUv2_MAX_STLB_ENTRIES];
	/*
	 * qi:N1541 -- page shift the slot's STLB is typed for (12 / 16 / 20);
	 * 0 = not allocated yet.  A slot changes type only while EMPTY.
	 */
	u8 stlb_shift[MMUv2_MAX_STLB_ENTRIES];
};

/*
 * VIPPico v3 (0x99): the page tables the MMU actually walks.
 *
 * That chip's ring carries no PTA_CONFIG (the FE hangs on it) and flushes the
 * MMU by MMIO, so the PTA load in etnaviv_iommuv2_restore_sec is the only one
 * the MMU takes, and an MMIO flush drops cached PTEs but not the master->slave
 * pointers (a process reusing an iova sees its new PTE; a second process, with
 * its own slave pages, kept walking the first one's -- pico:N1251).  So the
 * MMU is given ONE set of tables owned by the GPU: a master page latched by
 * the PTA load and a fixed slave page per master slot.  Before each MMU flush
 * etnaviv_iommuv2_sync_hw_mtlb() copies the running context's slave pages into
 * them; the master entries change only when a slot first appears or changes
 * size, and the slave contents are what the flush is seen to refresh.
 */
struct etnaviv_iommuv2_hw_tables {
	u32 *mtlb_cpu;
	dma_addr_t mtlb_dma;
	u32 *stlb_cpu[MMUv2_MAX_STLB_ENTRIES];
	dma_addr_t stlb_dma[MMUv2_MAX_STLB_ENTRIES];
};

/*
 * qi:N1541 -- read-only effect counters: PTEs written since load, by size.
 * The arm witness is what the MMU code DID, not what the knob was set to.
 */
static unsigned int npu_mmu_1m_entries;
module_param_named(npu_mmu_1m_entries, npu_mmu_1m_entries, uint, 0444);
MODULE_PARM_DESC(npu_mmu_1m_entries, "read-only: 1 MiB STLB entries written since load (qi:N1541)");
static unsigned int npu_mmu_64k_entries;
module_param_named(npu_mmu_64k_entries, npu_mmu_64k_entries, uint, 0444);
MODULE_PARM_DESC(npu_mmu_64k_entries, "read-only: 64 KiB STLB entries written since load (qi:N1541)");
static unsigned int npu_mmu_4k_entries;
module_param_named(npu_mmu_4k_entries, npu_mmu_4k_entries, uint, 0444);
MODULE_PARM_DESC(npu_mmu_4k_entries, "read-only: 4 KiB STLB entries written since load (qi:N1541)");
static unsigned int npu_mmu_slot_conflicts;
module_param_named(npu_mmu_slot_conflicts, npu_mmu_slot_conflicts, uint, 0444);
MODULE_PARM_DESC(npu_mmu_slot_conflicts, "read-only: map requests refused because a live slot had another page size (qi:N1541)");

static inline unsigned int mmuv2_slot_entries(unsigned int shift)
{
	return 1u << (MMUv2_MTLB_SHIFT - shift);
}

static inline u32 mmuv2_mtlb_size_bits(unsigned int shift)
{
	switch (shift) {
	case 20: return MMUv2_MTLB_PAGE_1M;
	case 16: return MMUv2_MTLB_PAGE_64K;
	default: return MMUv2_MTLB_PAGE_4K;
	}
}

static struct etnaviv_iommuv2_context *
to_v2_context(struct etnaviv_iommu_context *context)
{
	return container_of(context, struct etnaviv_iommuv2_context, base);
}

static void etnaviv_iommuv2_free(struct etnaviv_iommu_context *context)
{
	struct etnaviv_iommuv2_context *v2_context = to_v2_context(context);
	int i;

	if (drm_mm_node_allocated(&context->axi_sram_node))
		drm_mm_remove_node(&context->axi_sram_node);

	/*
	 * The A311D per-submit flop-reset page reserves a drm_mm node
	 * (etnaviv_iommu_install_flop_reset), mirroring axi_sram_node. It must
	 * likewise be released before drm_mm_takedown, or the context teardown
	 * trips "Memory manager not clean during takedown" (drm_mm_takedown WARN)
	 * once per submit via the etnaviv_iommuv2_restore context-reap path.
	 */
	if (drm_mm_node_allocated(&context->flop_node))
		drm_mm_remove_node(&context->flop_node);

	drm_mm_takedown(&context->mm);

	for (i = 0; i < MMUv2_MAX_STLB_ENTRIES; i++) {
		if (v2_context->stlb_cpu[i])
			dma_free_wc(context->global->dev, SZ_4K,
				    v2_context->stlb_cpu[i],
				    v2_context->stlb_dma[i]);
	}

	dma_free_wc(context->global->dev, SZ_4K, v2_context->mtlb_cpu,
		    v2_context->mtlb_dma);

	clear_bit(v2_context->id, context->global->v2.pta_alloc);

	vfree(v2_context);
}
/* qi:N1541 -- true when no entry of the slot's CURRENT type is present. */
static bool etnaviv_iommuv2_slot_empty(struct etnaviv_iommuv2_context *v2_context,
				       int stlb)
{
	unsigned int n = mmuv2_slot_entries(v2_context->stlb_shift[stlb]);
	unsigned int i;

	for (i = 0; i < n; i++)
		if (v2_context->stlb_cpu[stlb][i] & MMUv2_PTE_PRESENT)
			return false;
	return true;
}

/*
 * Make sure slot `stlb` has an STLB typed for pages of 1 << shift bytes.
 * A slot that already holds live entries of another size is refused: the
 * caller (etnaviv_mmu.c) only asks for a large type on a slot it covers
 * WHOLE, and drm_mm guarantees no other mapping lives there, so a refusal
 * here is a bug and is counted, never silently downgraded.
 */
static int
etnaviv_iommuv2_ensure_stlb(struct etnaviv_iommuv2_context *v2_context,
			    int stlb, unsigned int shift)
{
	if (v2_context->stlb_cpu[stlb]) {
		if (v2_context->stlb_shift[stlb] == shift)
			return 0;
		if (!etnaviv_iommuv2_slot_empty(v2_context, stlb)) {
			npu_mmu_slot_conflicts++;
			WARN_ONCE(1, "etnaviv: MTLB slot %d live at shift %u, refused shift %u\n",
				  stlb, v2_context->stlb_shift[stlb], shift);
			return -EBUSY;
		}
		/* empty: retype it below */
	} else {
		v2_context->stlb_cpu[stlb] =
				dma_alloc_wc(v2_context->base.global->dev, SZ_4K,
					     &v2_context->stlb_dma[stlb],
					     GFP_KERNEL);

		if (!v2_context->stlb_cpu[stlb])
			return -ENOMEM;
	}

	memset32(v2_context->stlb_cpu[stlb], MMUv2_PTE_EXCEPTION,
		 SZ_4K / sizeof(u32));
	v2_context->stlb_shift[stlb] = shift;

	v2_context->mtlb_cpu[stlb] =
			v2_context->stlb_dma[stlb] | MMUv2_PTE_PRESENT |
			mmuv2_mtlb_size_bits(shift);

	return 0;
}

static int etnaviv_iommuv2_map(struct etnaviv_iommu_context *context,
			       unsigned long iova, phys_addr_t paddr,
			       size_t size, int prot)
{
	struct etnaviv_iommuv2_context *v2_context = to_v2_context(context);
	int mtlb_entry, stlb_entry, ret;
	unsigned int shift;
	u32 entry = lower_32_bits(paddr) | MMUv2_PTE_PRESENT;

	switch (size) {
	case SZ_4K:  shift = 12; break;
	case SZ_64K: shift = 16; break;
	case SZ_1M:  shift = 20; break;
	default:     return -EINVAL;
	}
	if (!IS_ALIGNED(iova, size) || !IS_ALIGNED(paddr, size))
		return -EINVAL;

	if (IS_ENABLED(CONFIG_PHYS_ADDR_T_64BIT))
		entry |= (upper_32_bits(paddr) & 0xff) << 4;

	if (prot & ETNAVIV_PROT_WRITE)
		entry |= MMUv2_PTE_WRITEABLE;

	mtlb_entry = (iova & MMUv2_MTLB_MASK) >> MMUv2_MTLB_SHIFT;
	stlb_entry = (iova & MMUv2_STLB_MASK) >> shift;

	ret = etnaviv_iommuv2_ensure_stlb(v2_context, mtlb_entry, shift);
	if (ret)
		return ret;

	v2_context->stlb_cpu[mtlb_entry][stlb_entry] = entry;
	switch (shift) {
	case 20: npu_mmu_1m_entries++; break;
	case 16: npu_mmu_64k_entries++; break;
	default: npu_mmu_4k_entries++; break;
	}

	return 0;
}

/*
 * Unmap whatever page holds `iova` and return the bytes from `iova` to the
 * end of that page, so etnaviv_context_unmap()'s 4 KiB-stepped walk crosses a
 * large-page slot in one step per page.  `size` is the caller's step hint.
 */
static size_t etnaviv_iommuv2_unmap(struct etnaviv_iommu_context *context,
				    unsigned long iova, size_t size)
{
	struct etnaviv_iommuv2_context *etnaviv_domain = to_v2_context(context);
	int mtlb_entry, stlb_entry;
	unsigned int shift;
	size_t pg;

	if (size != SZ_4K)
		return -EINVAL;

	mtlb_entry = (iova & MMUv2_MTLB_MASK) >> MMUv2_MTLB_SHIFT;
	if (!etnaviv_domain->stlb_cpu[mtlb_entry])
		return SZ_4K;		/* nothing was ever mapped here */
	shift = etnaviv_domain->stlb_shift[mtlb_entry] ?: 12;
	pg = 1ul << shift;
	stlb_entry = (iova & MMUv2_STLB_MASK) >> shift;

	etnaviv_domain->stlb_cpu[mtlb_entry][stlb_entry] = MMUv2_PTE_EXCEPTION;

	return pg - (iova & (pg - 1));
}

static size_t etnaviv_iommuv2_dump_size(struct etnaviv_iommu_context *context)
{
	struct etnaviv_iommuv2_context *v2_context = to_v2_context(context);
	size_t dump_size = SZ_4K;
	int i;

	for (i = 0; i < MMUv2_MAX_STLB_ENTRIES; i++)
		if (v2_context->mtlb_cpu[i] & MMUv2_PTE_PRESENT)
			dump_size += SZ_4K;

	return dump_size;
}

static void etnaviv_iommuv2_dump(struct etnaviv_iommu_context *context, void *buf)
{
	struct etnaviv_iommuv2_context *v2_context = to_v2_context(context);
	int i;

	memcpy(buf, v2_context->mtlb_cpu, SZ_4K);
	buf += SZ_4K;
	for (i = 0; i < MMUv2_MAX_STLB_ENTRIES; i++)
		if (v2_context->mtlb_cpu[i] & MMUv2_PTE_PRESENT) {
			memcpy(buf, v2_context->stlb_cpu[i], SZ_4K);
			buf += SZ_4K;
		}
}

static void etnaviv_iommuv2_restore_nonsec(struct etnaviv_gpu *gpu,
	struct etnaviv_iommu_context *context)
{
	struct etnaviv_iommuv2_context *v2_context = to_v2_context(context);
	u16 prefetch;

	/* If the MMU is already enabled the state is still there. */
	if (gpu_read(gpu, VIVS_MMUv2_CONTROL) & VIVS_MMUv2_CONTROL_ENABLE)
		return;

	if (gpu->mmu_context)
		etnaviv_iommu_context_put(gpu->mmu_context);
	gpu->mmu_context = etnaviv_iommu_context_get(context);

	prefetch = etnaviv_buffer_config_mmuv2(gpu,
				(u32)v2_context->mtlb_dma,
				(u32)context->global->bad_page_dma);
	etnaviv_gpu_start_fe(gpu, (u32)etnaviv_cmdbuf_get_pa(&gpu->buffer),
			     prefetch);
	etnaviv_gpu_wait_idle(gpu, 100);

	gpu_write(gpu, VIVS_MMUv2_CONTROL, VIVS_MMUv2_CONTROL_ENABLE);
}

static void etnaviv_iommuv2_restore_sec(struct etnaviv_gpu *gpu,
	struct etnaviv_iommu_context *context)
{
	struct etnaviv_iommuv2_context *v2_context = to_v2_context(context);
	u16 prefetch;

	/* If the MMU is already enabled the state is still there. */
	if (gpu_read(gpu, VIVS_MMUv2_SEC_CONTROL) & VIVS_MMUv2_SEC_CONTROL_ENABLE)
		return;

	if (gpu->mmu_context)
		etnaviv_iommu_context_put(gpu->mmu_context);
	gpu->mmu_context = etnaviv_iommu_context_get(context);

	gpu_write(gpu, VIVS_MMUv2_PTA_ADDRESS_LOW,
		  lower_32_bits(context->global->v2.pta_dma));
	gpu_write(gpu, VIVS_MMUv2_PTA_ADDRESS_HIGH,
		  upper_32_bits(context->global->v2.pta_dma));
	gpu_write(gpu, VIVS_MMUv2_PTA_CONTROL, VIVS_MMUv2_PTA_CONTROL_ENABLE);

	gpu_write(gpu, VIVS_MMUv2_NONSEC_SAFE_ADDR_LOW,
		  lower_32_bits(context->global->bad_page_dma));
	gpu_write(gpu, VIVS_MMUv2_SEC_SAFE_ADDR_LOW,
		  lower_32_bits(context->global->bad_page_dma));
	gpu_write(gpu, VIVS_MMUv2_SAFE_ADDRESS_CONFIG,
		  VIVS_MMUv2_SAFE_ADDRESS_CONFIG_NON_SEC_SAFE_ADDR_HIGH(
		  upper_32_bits(context->global->bad_page_dma)) |
		  VIVS_MMUv2_SAFE_ADDRESS_CONFIG_SEC_SAFE_ADDR_HIGH(
		  upper_32_bits(context->global->bad_page_dma)));

	/*
	 * VIPPico v3 (0x99): this PTA load is the only one the MMU ever takes,
	 * and what it latches here is walked for every later context.  Latch
	 * the GPU-owned tables (struct etnaviv_iommuv2_hw_tables), filled from
	 * this context.  Latching the context's own master TLB left every
	 * later process translating through the first one's tables, and
	 * through freed pages once it exited.
	 */
	if (gpu->identity.customer_id == 0x99 && !context->global->v2.hw) {
		struct etnaviv_iommuv2_hw_tables *hw = kzalloc(sizeof(*hw), GFP_KERNEL);

		if (hw) {
			hw->mtlb_cpu = dma_alloc_wc(context->global->dev, SZ_4K,
						    &hw->mtlb_dma, GFP_KERNEL);
			if (hw->mtlb_cpu) {
				memset32(hw->mtlb_cpu, MMUv2_PTE_EXCEPTION,
					 MMUv2_MAX_STLB_ENTRIES);
				context->global->v2.hw = hw;
			} else {
				kfree(hw);
			}
		}
	}

	if (gpu->identity.customer_id == 0x99 && context->global->v2.hw)
		context->global->v2.pta_cpu[v2_context->id] =
			etnaviv_iommuv2_sync_hw_mtlb(context) |
			VIVS_MMUv2_CONFIGURATION_MODE_MODE4_K;
	else
		context->global->v2.pta_cpu[v2_context->id] = v2_context->mtlb_dma |
					 VIVS_MMUv2_CONFIGURATION_MODE_MODE4_K;

	/* trigger a PTA load through the FE */
	prefetch = etnaviv_buffer_config_pta(gpu, v2_context->id);
	etnaviv_gpu_start_fe(gpu, (u32)etnaviv_cmdbuf_get_pa(&gpu->buffer),
			     prefetch);
	etnaviv_gpu_wait_idle(gpu, 100);

	gpu_write(gpu, VIVS_MMUv2_SEC_CONTROL, VIVS_MMUv2_SEC_CONTROL_ENABLE);
}

u32 etnaviv_iommuv2_get_mtlb_addr(struct etnaviv_iommu_context *context)
{
	struct etnaviv_iommuv2_context *v2_context = to_v2_context(context);

	return v2_context->mtlb_dma;
}

/*
 * VIPPico v3 (0x99): make the GPU-owned tables the MMU walks (struct
 * etnaviv_iommuv2_hw_tables) describe @context, and return their master TLB
 * address.  Without them (every other chip), nothing is copied and the
 * context's own master TLB address is returned.  Called before each MMU flush:
 * the flush then drops what the MMU cached from the previous contents.
 */
u32 etnaviv_iommuv2_sync_hw_mtlb(struct etnaviv_iommu_context *context)
{
	struct etnaviv_iommuv2_context *v2_context = to_v2_context(context);
	struct etnaviv_iommuv2_hw_tables *hw = context->global->v2.hw;
	int i;

	if (!hw)
		return (u32)v2_context->mtlb_dma;

	for (i = 0; i < MMUv2_MAX_STLB_ENTRIES; i++) {
		u32 entry;

		if (!v2_context->stlb_cpu[i]) {
			/* no slave page here: unmapped in this context */
			if (hw->stlb_cpu[i])
				memset32(hw->stlb_cpu[i], MMUv2_PTE_EXCEPTION,
					 SZ_4K / sizeof(u32));
			continue;
		}

		if (!hw->stlb_cpu[i]) {
			hw->stlb_cpu[i] = dma_alloc_wc(context->global->dev, SZ_4K,
						       &hw->stlb_dma[i], GFP_KERNEL);
			if (!hw->stlb_cpu[i]) {
				dev_err_ratelimited(context->global->dev,
					"etnaviv: no GPU-owned slave page for MTLB slot %d\n", i);
				hw->mtlb_cpu[i] = MMUv2_PTE_EXCEPTION;
				continue;
			}
		}

		memcpy(hw->stlb_cpu[i], v2_context->stlb_cpu[i], SZ_4K);
		/* the context's entry with its slave page swapped for ours:
		 * the low bits carry present + the slot's page size */
		entry = (u32)hw->stlb_dma[i] | (v2_context->mtlb_cpu[i] & 0xfff);
		hw->mtlb_cpu[i] = entry;
	}
	/* the tables are visible to the MMU before the flush that follows */
	wmb();

	return (u32)hw->mtlb_dma;
}

void etnaviv_iommuv2_free_hw_tables(struct etnaviv_iommu_global *global)
{
	struct etnaviv_iommuv2_hw_tables *hw = global->v2.hw;
	int i;

	for (i = 0; i < MMUv2_MAX_STLB_ENTRIES; i++)
		if (hw->stlb_cpu[i])
			dma_free_wc(global->dev, SZ_4K, hw->stlb_cpu[i],
				    hw->stlb_dma[i]);
	dma_free_wc(global->dev, SZ_4K, hw->mtlb_cpu, hw->mtlb_dma);
	kfree(hw);
	global->v2.hw = NULL;
}

unsigned short etnaviv_iommuv2_get_pta_id(struct etnaviv_iommu_context *context)
{
	struct etnaviv_iommuv2_context *v2_context = to_v2_context(context);

	return v2_context->id;
}
static void etnaviv_iommuv2_restore(struct etnaviv_gpu *gpu,
				    struct etnaviv_iommu_context *context)
{
	switch (gpu->sec_mode) {
	case ETNA_SEC_NONE:
		etnaviv_iommuv2_restore_nonsec(gpu, context);
		break;
	case ETNA_SEC_KERNEL:
		etnaviv_iommuv2_restore_sec(gpu, context);
		break;
	default:
		WARN(1, "unhandled GPU security mode\n");
		break;
	}
}

const struct etnaviv_iommu_ops etnaviv_iommuv2_ops = {
	.free = etnaviv_iommuv2_free,
	.map = etnaviv_iommuv2_map,
	.unmap = etnaviv_iommuv2_unmap,
	.dump_size = etnaviv_iommuv2_dump_size,
	.dump = etnaviv_iommuv2_dump,
	.restore = etnaviv_iommuv2_restore,
};

struct etnaviv_iommu_context *
etnaviv_iommuv2_context_alloc(struct etnaviv_iommu_global *global)
{
	struct etnaviv_iommuv2_context *v2_context;
	struct etnaviv_iommu_context *context;

	v2_context = vzalloc(sizeof(*v2_context));
	if (!v2_context)
		return NULL;

	mutex_lock(&global->lock);
	v2_context->id = find_first_zero_bit(global->v2.pta_alloc,
					     ETNAVIV_PTA_ENTRIES);
	if (v2_context->id < ETNAVIV_PTA_ENTRIES) {
		set_bit(v2_context->id, global->v2.pta_alloc);
	} else {
		mutex_unlock(&global->lock);
		goto out_free;
	}
	mutex_unlock(&global->lock);

	v2_context->mtlb_cpu = dma_alloc_wc(global->dev, SZ_4K,
					    &v2_context->mtlb_dma, GFP_KERNEL);
	if (!v2_context->mtlb_cpu)
		goto out_free_id;

	memset32(v2_context->mtlb_cpu, MMUv2_PTE_EXCEPTION,
		 MMUv2_MAX_STLB_ENTRIES);

	global->v2.pta_cpu[v2_context->id] = v2_context->mtlb_dma;

	context = &v2_context->base;
	context->global = global;
	kref_init(&context->refcount);
	mutex_init(&context->lock);
	INIT_LIST_HEAD(&context->mappings);
	drm_mm_init(&context->mm, SZ_4K, (u64)SZ_1G * 4 - SZ_4K);

	return context;

out_free_id:
	clear_bit(v2_context->id, global->v2.pta_alloc);
out_free:
	vfree(v2_context);
	return NULL;
}
