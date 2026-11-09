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

/* Apply the request's patches to one buffer object (the mapping is gna_mmu_map_request's). */
static int gna_patch_memory(struct gna_device *gna_priv, struct gna_buffer_with_object *buffer)
{
	struct drm_gem_shmem_object *drmshmemo = &buffer->gem->base;
	struct gna_buffer *gnab = &buffer->gna;
	struct gna_memory_patch *patch;
	struct iosys_map vmap;
	int ret = 0;
	u32 i;

	dev_dbg(gna_dev(gna_priv), "handle: %u, patch_count, %llu\n",
		gnab->handle, gnab->patch_count);

	if (!gnab->patch_count)
		return 0;

	ret = drm_gem_vmap(&drmshmemo->base, &vmap);
	if (ret)
		return ret;

	patch = (struct gna_memory_patch *)(uintptr_t)gnab->patches_ptr;
	for (i = 0; i < gnab->patch_count; i++, patch++) {
		ret = gna_do_patch_memory(gna_priv, patch, vmap.vaddr);
		if (ret)
			break;
	}

	kvfree((void *)(uintptr_t)gnab->patches_ptr);
	gnab->patches_ptr = 0;
	gnab->patch_count = 0;   /* applied: a request scored again (gna_request_start_direct's retry) skips them */
	drm_gem_vunmap(&drmshmemo->base, &vmap);
	return ret;
}

static struct gna_buffer_with_object *gna_find_buffer(struct gna_buffer_with_object *buffer_list,
						u32 buffer_count, u32 mmu_offset, u32 *memory_offset)
{
	struct gna_buffer_with_object *buffer;
	u32 memory_size;
	u32 offset;
	u32 i;

	/* the device address space as gna_mmu_add_locked() lays it out: each object's every page, in order */
	offset = 0;
	for (i = 0; i < buffer_count; i++) {
		buffer = buffer_list + i;
		memory_size = buffer->gem->base.base.size;
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

	/*
	 * The descriptor must lie inside the buffer, not only start in it: gna_find_buffer() matches an offset anywhere
	 * in the buffer's page-rounded span, so an offset in its last bytes read past the end of the object's vmap.
	 */
	if (mmu_offset - memory_offset > buffer->gna.size ||
	    buffer->gna.size - (mmu_offset - memory_offset) < sizeof(struct gna_xnn_descriptor))
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
	unsigned long flags;
	u64 buffer_count;
	u32 desc_base;
	int ret;
	u64 i;

	ret = 0;

	gna_priv = score_request->gna_priv;   /* the worker may run after the client closed its file */

	mmu = &gna_priv->mmu;
	xnn_config = &mmu->hwdesc->xnn_config;
	compute_cfg = &score_request->compute_cfg;

	buffer_count = score_request->buffer_count;

	for (i = 0, buffer = score_request->buffer_list; i < buffer_count; i++, buffer++) {
		ret = gna_patch_memory(gna_priv, buffer);
		if (ret)
			goto err;
	}

	ret = gna_mmu_map_request(gna_priv, score_request->buffer_list, buffer_count);
	if (ret)
		goto err;

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

	/* the interrupt is about to be enabled: clear a polled request's latched completion first, or it fires now */
	if (!score_request->polled && gna_priv->completion_latched) {
		ret = gna_abort_hw(gna_priv);
		if (ret)
			goto err;
	}
	/*
	 * START and hw_started together, under direct_lock: until START, STS still carries the previous request's
	 * completion (a clean finish clears only the saturation bit), so everything that reads STS for this request --
	 * the interrupt, a reclaim, a timed-out WAIT -- does so only once hw_started is set, and so only after START.
	 */
	spin_lock_irqsave(&gna_priv->direct_lock, flags);
	gna_start_scoring(gna_priv, compute_cfg, !score_request->polled);
	score_request->hw_started = true;
	spin_unlock_irqrestore(&gna_priv->direct_lock, flags);

err:
	return ret;
}
