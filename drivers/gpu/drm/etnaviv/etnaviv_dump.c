// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2015-2018 Etnaviv Project
 */

#include <linux/devcoredump.h>
#include <linux/moduleparam.h>

#include "etnaviv_cmdbuf.h"
#include "etnaviv_dump.h"
#include "etnaviv_gem.h"
#include "etnaviv_gpu.h"
#include "etnaviv_mmu.h"
#include "etnaviv_sched.h"
#include "state.xml.h"
#include "state_hi.xml.h"

static bool etnaviv_dump_core = true;
module_param_named(dump_core, etnaviv_dump_core, bool, 0600);

/*
 * qi:N1462 -- etnaviv_core_dump() runs on the line immediately BEFORE
 * etnaviv_gpu_recover_hang() on the timeout path, and reads 37 registers off
 * the same block whose AXI is wedged, six of them the very registers qi:N1453
 * removed from recover_hang(). The qi:N1453 fix does not touch it, so "zero
 * MMIO between the timeout and the reset" is false of the path as a whole.
 *
 * qi:N1464/qi:N1465 -- THESE 37 READS ARE NOT THE HAZARD, and this knob is NOT
 * a workaround for one. Measured on a wedged A311D 0x88 at the only position
 * where the abort reproduces (the first inference cell after a boot): four
 * clean cells took all 37 reads off the wedged block -- 148 reads -- with zero
 * aborts, while a SINGLE gpu_read() issued a few hundred ms later from
 * etnaviv_gpu_recover_hang() killed the board in 11 armed cells out of 11.
 * The abort is POSITIONAL, not per-register: VIVS_FE_DMA_ADDRESS is read here
 * and again by etnaviv_sched_timedout_job() without harm, and fatally on the
 * third read from inside recover_hang. Turning this knob off does NOT make a
 * board safer; the only knob that can hurt you is npu_recover_predump.
 *
 * So this stays a DIAGNOSTIC LEVER, kept for one open question: what changes
 * in the ~376 ms between the dump's reads and the predump block. It is the only
 * way to vary the dump's MMIO, and the exoneration above rests on four cells at
 * p = 1.0 -- no power at all -- so removing it would close the only channel that
 * can raise n.
 *
 * This knob gates the register read loop only: the dump buffer keeps its shape
 * and its register addresses, and the values read back as 0 -- so the arm is a
 * MMIO change, not a layout change.
 */
static bool npu_coredump_regs = true;
module_param_named(npu_coredump_regs, npu_coredump_regs, bool, 0644);
MODULE_PARM_DESC(npu_coredump_regs,
	"read the 37 diagnostic registers in the core dump (default 1). 0 = record their addresses with value 0 and do NO MMIO. DIAGNOSTIC ONLY: these reads are measured SAFE on a wedged A311D 0x88 and changing this fixes nothing -- the fatal MMIO is npu_recover_predump's, not this.");

struct core_dump_iterator {
	void *start;
	struct etnaviv_dump_object_header *hdr;
	void *data;
};

static const unsigned short etnaviv_dump_registers[] = {
	VIVS_HI_AXI_STATUS,
	VIVS_HI_CLOCK_CONTROL,
	VIVS_HI_IDLE_STATE,
	VIVS_HI_AXI_CONFIG,
	VIVS_HI_INTR_ENBL,
	VIVS_HI_CHIP_IDENTITY,
	VIVS_HI_CHIP_FEATURE,
	VIVS_HI_CHIP_MODEL,
	VIVS_HI_CHIP_REV,
	VIVS_HI_CHIP_DATE,
	VIVS_HI_CHIP_TIME,
	VIVS_HI_CHIP_MINOR_FEATURE_0,
	VIVS_HI_CACHE_CONTROL,
	VIVS_HI_AXI_CONTROL,
	VIVS_PM_POWER_CONTROLS,
	VIVS_PM_MODULE_CONTROLS,
	VIVS_PM_MODULE_STATUS,
	VIVS_PM_PULSE_EATER,
	VIVS_MC_MMU_FE_PAGE_TABLE,
	VIVS_MC_MMU_TX_PAGE_TABLE,
	VIVS_MC_MMU_PE_PAGE_TABLE,
	VIVS_MC_MMU_PEZ_PAGE_TABLE,
	VIVS_MC_MMU_RA_PAGE_TABLE,
	VIVS_MC_DEBUG_MEMORY,
	VIVS_MC_MEMORY_BASE_ADDR_RA,
	VIVS_MC_MEMORY_BASE_ADDR_FE,
	VIVS_MC_MEMORY_BASE_ADDR_TX,
	VIVS_MC_MEMORY_BASE_ADDR_PEZ,
	VIVS_MC_MEMORY_BASE_ADDR_PE,
	VIVS_MC_MEMORY_TIMING_CONTROL,
	VIVS_MC_BUS_CONFIG,
	VIVS_FE_DMA_STATUS,
	VIVS_FE_DMA_DEBUG_STATE,
	VIVS_FE_DMA_ADDRESS,
	VIVS_FE_DMA_LOW,
	VIVS_FE_DMA_HIGH,
	VIVS_FE_AUTO_FLUSH,
};

static void etnaviv_core_dump_header(struct core_dump_iterator *iter,
	u32 type, void *data_end)
{
	struct etnaviv_dump_object_header *hdr = iter->hdr;

	hdr->magic = cpu_to_le32(ETDUMP_MAGIC);
	hdr->type = cpu_to_le32(type);
	hdr->file_offset = cpu_to_le32(iter->data - iter->start);
	hdr->file_size = cpu_to_le32(data_end - iter->data);

	iter->hdr++;
	iter->data += le32_to_cpu(hdr->file_size);
}

static void etnaviv_core_dump_registers(struct core_dump_iterator *iter,
	struct etnaviv_gpu *gpu)
{
	struct etnaviv_dump_registers *reg = iter->data;
	unsigned int i;
	u32 read_addr;

	for (i = 0; i < ARRAY_SIZE(etnaviv_dump_registers); i++, reg++) {
		read_addr = etnaviv_dump_registers[i];
		if (read_addr >= VIVS_PM_POWER_CONTROLS &&
		    read_addr <= VIVS_PM_PULSE_EATER)
			read_addr = gpu_fix_power_address(gpu, read_addr);
		reg->reg = cpu_to_le32(etnaviv_dump_registers[i]);
		reg->value = npu_coredump_regs ?
			cpu_to_le32(gpu_read(gpu, read_addr)) : 0;
	}

	dev_err(gpu->dev, "core dump: %u register read(s)\n",
		npu_coredump_regs ?
			(unsigned int)ARRAY_SIZE(etnaviv_dump_registers) : 0u);

	etnaviv_core_dump_header(iter, ETDUMP_BUF_REG, reg);
}

static void etnaviv_core_dump_mmu(struct core_dump_iterator *iter,
	struct etnaviv_iommu_context *mmu, size_t mmu_size)
{
	etnaviv_iommu_dump(mmu, iter->data);

	etnaviv_core_dump_header(iter, ETDUMP_BUF_MMU, iter->data + mmu_size);
}

static void etnaviv_core_dump_mem(struct core_dump_iterator *iter, u32 type,
	void *ptr, size_t size, u64 iova)
{
	memcpy(iter->data, ptr, size);

	iter->hdr->iova = cpu_to_le64(iova);

	etnaviv_core_dump_header(iter, type, iter->data + size);
}

void etnaviv_core_dump(struct etnaviv_gem_submit *submit)
{
	struct etnaviv_gpu *gpu = submit->gpu;
	struct core_dump_iterator iter;
	struct etnaviv_gem_object *obj;
	unsigned int n_obj, n_bomap_pages;
	size_t file_size, mmu_size;
	__le64 *bomap, *bomap_start;
	int i;

	/* Only catch the first event, or when manually re-armed */
	if (!etnaviv_dump_core)
		return;
	etnaviv_dump_core = false;

	mutex_lock(&submit->mmu_context->lock);

	mmu_size = etnaviv_iommu_dump_size(submit->mmu_context);

	/* We always dump registers, mmu, ring, hanging cmdbuf and end marker */
	n_obj = 5;
	n_bomap_pages = 0;
	file_size = ARRAY_SIZE(etnaviv_dump_registers) *
			sizeof(struct etnaviv_dump_registers) +
		    mmu_size + gpu->buffer.size + submit->cmdbuf.size;

	/* Add in the active buffer objects */
	for (i = 0; i < submit->nr_bos; i++) {
		obj = submit->bos[i].obj;
		file_size += obj->base.size;
		n_bomap_pages += obj->base.size >> PAGE_SHIFT;
		n_obj++;
	}

	/* If we have any buffer objects, add a bomap object */
	if (n_bomap_pages) {
		file_size += n_bomap_pages * sizeof(__le64);
		n_obj++;
	}

	/* Add the size of the headers */
	file_size += sizeof(*iter.hdr) * n_obj;

	/* Allocate the file in vmalloc memory, it's likely to be big */
	iter.start = __vmalloc(file_size, GFP_NOWAIT);
	if (!iter.start) {
		mutex_unlock(&submit->mmu_context->lock);
		dev_warn(gpu->dev, "failed to allocate devcoredump file\n");
		return;
	}

	/* Point the data member after the headers */
	iter.hdr = iter.start;
	iter.data = &iter.hdr[n_obj];

	memset(iter.hdr, 0, iter.data - iter.start);

	etnaviv_core_dump_registers(&iter, gpu);
	etnaviv_core_dump_mmu(&iter, submit->mmu_context, mmu_size);
	etnaviv_core_dump_mem(&iter, ETDUMP_BUF_RING, gpu->buffer.vaddr,
			      gpu->buffer.size,
			      etnaviv_cmdbuf_get_va(&gpu->buffer,
					&submit->mmu_context->cmdbuf_mapping));

	etnaviv_core_dump_mem(&iter, ETDUMP_BUF_CMD,
			      submit->cmdbuf.vaddr, submit->cmdbuf.size,
			      etnaviv_cmdbuf_get_va(&submit->cmdbuf,
					&submit->mmu_context->cmdbuf_mapping));

	mutex_unlock(&submit->mmu_context->lock);

	/* Reserve space for the bomap */
	if (n_bomap_pages) {
		bomap_start = bomap = iter.data;
		memset(bomap, 0, sizeof(*bomap) * n_bomap_pages);
		etnaviv_core_dump_header(&iter, ETDUMP_BUF_BOMAP,
					 bomap + n_bomap_pages);
	} else {
		/* Silence warning */
		bomap_start = bomap = NULL;
	}

	for (i = 0; i < submit->nr_bos; i++) {
		struct etnaviv_vram_mapping *vram;
		struct page **pages;
		void *vaddr;

		obj = submit->bos[i].obj;
		vram = submit->bos[i].mapping;

		mutex_lock(&obj->lock);
		pages = etnaviv_gem_get_pages(obj);
		mutex_unlock(&obj->lock);
		if (!IS_ERR(pages)) {
			int j;

			iter.hdr->data[0] = cpu_to_le32((bomap - bomap_start));

			for (j = 0; j < obj->base.size >> PAGE_SHIFT; j++)
				*bomap++ = cpu_to_le64(page_to_phys(*pages++));
		}

		iter.hdr->iova = cpu_to_le64(vram->iova);

		vaddr = etnaviv_gem_vmap(&obj->base);
		if (vaddr)
			memcpy(iter.data, vaddr, obj->base.size);

		etnaviv_core_dump_header(&iter, ETDUMP_BUF_BO, iter.data +
					 obj->base.size);
	}

	etnaviv_core_dump_header(&iter, ETDUMP_BUF_END, iter.data);

	dev_coredumpv(gpu->dev, iter.start, iter.data - iter.start, GFP_NOWAIT);
}
