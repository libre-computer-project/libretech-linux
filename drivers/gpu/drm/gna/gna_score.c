// SPDX-License-Identifier: GPL-2.0-only
// Copyright(c) 2017-2022 Intel Corporation

#include <drm/drm_gem.h>
#include <drm/drm_gem_shmem_helper.h>

#include <linux/dma-buf.h>
#include <linux/kernel.h>
#include <linux/math.h>
#include <linux/mm.h>
#include <linux/scatterlist.h>
#include <linux/string.h>
#include <linux/types.h>

#include <uapi/drm/gna_drm.h>

/*
 * 6.18 API drift: dropped the "#include ../drm_internal.h" the original
 * carried -- this TU references no core-internal DRM symbol, and out-of-core
 * helper drivers must not pull in drivers/gpu/drm/drm_internal.h.
 */

#include "gna_device.h"
#include "gna_gem.h"
#include "gna_hw.h"
#include "gna_mem.h"
#include "gna_request.h"
#include "gna_score.h"

static int gna_do_patch_memory(struct gna_device *gna_priv,
			       struct gna_memory_patch *patch, void *vaddr)
{
	size_t size;
	void *dest;
	u64 value;

	value = patch->value;
	size = patch->size;
	dest = (u8 *)vaddr + patch->offset;

	switch (size) {
	case 0:
		return -EFAULT;
	case sizeof(u8):
		*((u8 *)dest) = (u8)value;
		break;
	case sizeof(u16):
		*((u16 *)dest) = (u16)value;
		break;
	case sizeof(u32):
		*((u32 *)dest) = (u32)value;
		break;
	case sizeof(u64):
		*((u64 *)dest) = (u64)value;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int gna_patch_memory(struct gna_device *gna_priv, struct gna_buffer_with_object *buffer)
{
	struct drm_gem_shmem_object *drmshmemo = &buffer->gem->base;
	struct gna_gem_object *gnagemo = buffer->gem;
	struct gna_buffer *gnab = &buffer->gna;
	struct gna_memory_patch *patch;
	struct iosys_map vmap;
	struct sg_table *sgt;
	int ret = 0;
	u32 i;

	dev_dbg(gna_dev(gna_priv), "handle: %u, patch_count, %llu\n",
		gnab->handle, gnab->patch_count);

	sgt = drm_gem_shmem_get_pages_sgt(drmshmemo);

	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		goto err;
	}

	if (gnab->patch_count) {
		ret = drm_gem_vmap(&drmshmemo->base, &vmap);

		if (ret)
			goto err_pages_sgt;

		patch = (struct gna_memory_patch *)(uintptr_t)gnab->patches_ptr;
		for (i = 0; i < gnab->patch_count; i++, patch++) {
			ret = gna_do_patch_memory(gna_priv, patch, vmap.vaddr);
			if (ret)
				break;
		}

		kvfree((void *)(uintptr_t)gnab->patches_ptr);
		gnab->patches_ptr = 0;
		drm_gem_vunmap(&drmshmemo->base, &vmap);
		if (ret) // ret from gna_do_patch_memory
			goto err_pages_sgt;
	}

	gna_mmu_add(gna_priv, drmshmemo);

	return 0;

err_pages_sgt:
	gna_gem_object_put_pages_sgt(gnagemo);
err:
	return ret;
}

static struct gna_buffer_with_object *gna_find_buffer(struct gna_buffer_with_object *buffer_list,
						u32 buffer_count, u32 mmu_offset, u32 *memory_offset)
{
	struct gna_buffer_with_object *buffer;
	u32 memory_size;
	u32 offset;
	u32 i;

	offset = 0;
	for (i = 0; i < buffer_count; i++) {
		buffer = buffer_list + i;
		memory_size = round_up(buffer->gna.size, PAGE_SIZE);
		if (mmu_offset < offset + memory_size) {
			*memory_offset = offset;
			return buffer;
		}
		offset += memory_size;
	}

	return NULL;
}

static int gna_copy_gmm_config(struct gna_device *gna_priv,
			struct gna_buffer_with_object *buffer_list,
			u32 buffer_count, u32 mmu_offset)
{
	struct gna_buffer_with_object *buffer;
	struct gna_hw_descriptor *hwdesc;
	struct drm_gem_object *drmgemo;
	struct gna_mmu_object *mmu;
	struct iosys_map vmap;
	u32 memory_offset;
	u8 *gmm_desc;
	int ret = 0;

	mmu = &gna_priv->mmu;
	hwdesc = mmu->hwdesc;

	buffer = gna_find_buffer(buffer_list, buffer_count, mmu_offset, &memory_offset);
	if (!buffer)
		return -EINVAL;

	drmgemo = &buffer->gem->base.base;

	ret = drm_gem_vmap(drmgemo, &vmap);

	if (ret)
		return ret;

	gmm_desc = (u8 *)vmap.vaddr + (mmu_offset - memory_offset);
	memcpy(&hwdesc->xnn_config, gmm_desc, sizeof(struct gna_xnn_descriptor));
	drm_gem_vunmap(drmgemo, &vmap);

	return 0;
}

int gna_score(struct gna_request *score_request)
{
	struct gna_buffer_with_object *buffer;
	struct gna_xnn_descriptor *xnn_config;
	struct gna_compute_cfg *compute_cfg;
	struct gna_device *gna_priv;
	struct gna_mmu_object *mmu;
	u64 buffer_count;
	u32 desc_base;
	int ret;
	u64 i;

	ret = 0;

	gna_priv = to_gna_device(score_request->drm_f->minor->dev);

	mmu = &gna_priv->mmu;
	xnn_config = &mmu->hwdesc->xnn_config;
	compute_cfg = &score_request->compute_cfg;

	buffer_count = score_request->buffer_count;

	for (i = 0, buffer = score_request->buffer_list; i < buffer_count; i++, buffer++) {
		ret = gna_patch_memory(gna_priv, buffer);
		if (ret)
			goto err;
	}

	switch (compute_cfg->gna_mode) {
	case GNA_MODE_XNN:
		dev_dbg(gna_dev(gna_priv), "xNN mode; labase: %d, lacount: %d\n",
			compute_cfg->layer_base, compute_cfg->layer_count);
		xnn_config->labase = compute_cfg->layer_base;
		xnn_config->lacount = compute_cfg->layer_count;
		break;
	case GNA_MODE_GMM:
		dev_dbg(gna_dev(gna_priv), "GMM mode; offset: %d\n", compute_cfg->layer_base);
		ret = gna_copy_gmm_config(gna_priv, score_request->buffer_list,
					buffer_count, compute_cfg->layer_base);
		if (ret)
			goto err;
		break;
	default:
		ret = -EINVAL;
		goto err;
	}

	desc_base = (u32)(mmu->hwdesc_dma >> PAGE_SHIFT);
	gna_reg_write(gna_priv, GNA_MMIO_DESBASE, desc_base);

	gna_start_scoring(gna_priv, compute_cfg);

err:
	return ret;
}
