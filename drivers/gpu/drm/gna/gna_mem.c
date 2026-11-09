// SPDX-License-Identifier: GPL-2.0-only
// Copyright(c) 2017-2022 Intel Corporation

#include <drm/drm_drv.h>
#include <drm/drm_gem.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_managed.h>

#include <linux/atomic.h>
#include <linux/dma-resv.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/kref.h>
#include <linux/list.h>
#include <linux/math.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include "gna_device.h"
#include "gna_gem.h"
#include "gna_mem.h"
#include "gna_request.h"

static void gna_mmu_set(struct gna_device *gna_priv)
{
	struct gna_mmu_object *mmu;
	dma_addr_t pagetable_dma;
	u32 *pgdirn;
	int i;

	mmu = &gna_priv->mmu;

	pgdirn = mmu->hwdesc->mmu.pagedir_n;

	for (i = 0; i < mmu->num_pagetables; i++) {
		pagetable_dma = mmu->pagetables_dma[i];
		pgdirn[i] = pagetable_dma >> PAGE_SHIFT;
	}

	for (; i < GNA_PGDIRN_LEN; i++)
		pgdirn[i] = GNA_PGDIR_INVALID;
}

/* descriptor and page tables allocation */
int gna_mmu_init(struct gna_device *gna_priv)
{
	struct device *parent = gna_dev(gna_priv);
	struct gna_mmu_object *mmu;
	int desc_size;
	int i;

	if (gna_priv->info.num_pagetables > GNA_PGDIRN_LEN) {
		dev_dbg(gna_dev(gna_priv), "number of pagetables requested too large: %u\n", gna_priv->info.num_pagetables);
		return -EINVAL;
	}

	mmu = &gna_priv->mmu;

	desc_size = round_up(gna_priv->info.desc_info.desc_size, PAGE_SIZE);

	mmu->hwdesc = dmam_alloc_coherent(parent, desc_size, &mmu->hwdesc_dma,
					GFP_KERNEL);
	if (!mmu->hwdesc)
		return -ENOMEM;

	mmu->num_pagetables = gna_priv->info.num_pagetables;

	mmu->pagetables_dma = drmm_kmalloc_array(&gna_priv->drm, mmu->num_pagetables, sizeof(*mmu->pagetables_dma),
						GFP_KERNEL);
	if (!mmu->pagetables_dma)
		return -ENOMEM;

	mmu->pagetables = drmm_kmalloc_array(&gna_priv->drm, mmu->num_pagetables, sizeof(*mmu->pagetables), GFP_KERNEL);

	if (!mmu->pagetables)
		return -ENOMEM;

	for (i = 0; i < mmu->num_pagetables; i++) {
		mmu->pagetables[i] = dmam_alloc_coherent(parent, PAGE_SIZE,
							&mmu->pagetables_dma[i], GFP_KERNEL);
		if (!mmu->pagetables[i])
			return -ENOMEM;
	}

	gna_mmu_set(gna_priv);

	return 0;
}

/*
 * Append the object's pages to the GNA page tables, in order, so its GNA virtual addresses follow the objects
 * added before it. Walk the DMA-mapped segments only (for_each_sgtable_dma_sg): with an IOMMU, dma_map_sgtable()
 * merges the CPU entries into fewer DMA segments, and the entries past the mapped count carry DMA_MAPPING_ERROR
 * with length 0. Walking the CPU list instead never left such an entry, so it filled every remaining page-table
 * slot with addresses counted up from DMA_MAPPING_ERROR (0x0, 0x1000, ...), and the next object got no slots.
 */
static void gna_mmu_add_locked(struct gna_device *gna_priv, struct drm_gem_shmem_object *drmshmemo)
{
	struct gna_mmu_object *mmu;
	struct scatterlist *sg;
	dma_addr_t addr;
	unsigned int i;
	size_t pages;

	mmu = &gna_priv->mmu;
	lockdep_assert_held(&gna_priv->mmu_lock);

	if (!drmshmemo->sgt || !drmshmemo->sgt->nents) {
		dev_warn(gna_dev(gna_priv), "empty scatter list in memory object\n");
		return;
	}

	for_each_sgtable_dma_sg(drmshmemo->sgt, sg, i) {
		addr = sg_dma_address(sg);
		for (pages = DIV_ROUND_UP(sg_dma_len(sg), PAGE_SIZE); pages; pages--) {
			if (mmu->filled_pts >= mmu->num_pagetables) {
				dev_warn(gna_dev(gna_priv), "GNA page tables full: memory object only partly mapped\n");
				goto full;
			}
			mmu->pagetables[mmu->filled_pts][mmu->filled_pages++] = addr >> PAGE_SHIFT;
			addr += PAGE_SIZE;
			if (mmu->filled_pages == GNA_PT_LENGTH) {
				mmu->filled_pages = 0;
				mmu->filled_pts++;
			}
		}
	}

full:
	mmu->hwdesc->mmu.vamaxaddr =
		(mmu->filled_pts * PAGE_SIZE * GNA_PGDIR_ENTRIES) +
		(mmu->filled_pages * PAGE_SIZE) - 1;
	dev_dbg(gna_dev(gna_priv), "vamaxaddr: %u\n", mmu->hwdesc->mmu.vamaxaddr);
}

static void gna_mmu_clear_locked(struct gna_device *gna_priv)
{
	struct gna_mmu_object *mmu;
	int i;

	mmu = &gna_priv->mmu;
	lockdep_assert_held(&gna_priv->mmu_lock);

	for (i = 0; i < mmu->filled_pts; i++)
		memset(mmu->pagetables[i], 0, PAGE_SIZE);

	if (mmu->filled_pages > 0)
		memset(mmu->pagetables[mmu->filled_pts], 0, mmu->filled_pages * GNA_PT_ENTRY_SIZE);

	mmu->filled_pts = 0;
	mmu->filled_pages = 0;
	mmu->hwdesc->mmu.vamaxaddr = 0;
}

/* Clear the page tables and release the objects they were kept for. */
static void gna_mmu_drop_locked(struct gna_device *gna_priv)
{
	struct gna_mmu_object *mmu = &gna_priv->mmu;
	u32 i;

	lockdep_assert_held(&gna_priv->mmu_lock);
	gna_mmu_clear_locked(gna_priv);
	for (i = 0; i < mmu->mapped_count; i++)
		drm_gem_object_put(mmu->mapped[i]);
	mmu->mapped_count = 0;
}

/*
 * Map a request's buffer objects into the GNA page tables, in order -- or keep the tables as they are when the
 * previous request mapped the same objects in the same order, which a streaming client does on every request.
 * Mapping an object pins its pages and DMA-maps them once (drm_gem_shmem_get_pages_sgt keeps the sg table on the
 * object; drm_gem_shmem_free unmaps it), so a kept mapping costs nothing per request. The kept objects hold a
 * reference: a freed object can never be mistaken for a new one at the same address, and its pages stay valid
 * while the tables point at them. gna_mmu_forget() drops the set when one of them is freed.
 * A request with more than GNA_MMU_MAPPED_MAX buffers is mapped every time and nothing is kept.
 * Runs on the ordered request workqueue.
 */
int gna_mmu_map_request(struct gna_device *gna_priv, struct gna_buffer_with_object *buffers, u64 count)
{
	struct gna_mmu_object *mmu = &gna_priv->mmu;
	struct sg_table *sgt;
	bool same;
	u64 i;
	int ret = 0;

	mutex_lock(&gna_priv->mmu_lock);

	same = count <= GNA_MMU_MAPPED_MAX && count == mmu->mapped_count;
	for (i = 0; same && i < count; i++)
		same = mmu->mapped[i] == &buffers[i].gem->base.base;
	if (same)
		goto out;

	gna_mmu_drop_locked(gna_priv);
	for (i = 0; i < count; i++) {
		sgt = drm_gem_shmem_get_pages_sgt(&buffers[i].gem->base);
		if (IS_ERR(sgt)) {
			ret = PTR_ERR(sgt);
			gna_mmu_clear_locked(gna_priv);
			goto out;
		}
		gna_mmu_add_locked(gna_priv, &buffers[i].gem->base);
	}
	if (count <= GNA_MMU_MAPPED_MAX) {
		for (i = 0; i < count; i++) {
			mmu->mapped[i] = &buffers[i].gem->base.base;
			drm_gem_object_get(mmu->mapped[i]);
		}
		mmu->mapped_count = count;
	}

out:
	mutex_unlock(&gna_priv->mmu_lock);
	return ret;
}

/* An object is being freed: if the page tables are kept for it, drop them (with the hardware held). */
void gna_mmu_forget(struct gna_device *gna_priv, struct drm_gem_object *obj)
{
	struct gna_mmu_object *mmu = &gna_priv->mmu;
	u32 i;

	mutex_lock(&gna_priv->mmu_lock);
	for (i = 0; i < mmu->mapped_count; i++) {
		if (mmu->mapped[i] == obj) {
			gna_mmu_drop_locked(gna_priv);
			break;
		}
	}
	mutex_unlock(&gna_priv->mmu_lock);
}

/* Device going away: release whatever the page tables were kept for. */
void gna_mmu_release(struct gna_device *gna_priv)
{
	mutex_lock(&gna_priv->mmu_lock);
	gna_mmu_drop_locked(gna_priv);
	mutex_unlock(&gna_priv->mmu_lock);
}

/*
 * GEM_FREE of a handle: drop the client's requests still to run that name it, and the page tables if they are kept
 * for its object.
 * Only this file's requests: handles are per file, so the same number names a different object in another client.
 * Runs in the ioctl, not on the request workqueue: the hardware is held only while the requests are taken off the list
 * and, at the end, while the tables are dropped; a queued request is cancelled -- or waited for, if it is already
 * waiting for the hardware in gna_request_process() -- in between, with the hardware given back.
 */
void gna_gem_free_requests(struct gna_device *gna_priv, struct drm_file *file, u32 handle,
			   struct drm_gem_object *obj)
{
	struct gna_request *req, *tmp;
	LIST_HEAD(gone);
	u64 i;

	gna_hw_acquire(gna_priv);   /* no request on the hardware: one started from COMPUTE has completed */
	mutex_lock(&gna_priv->reqlist_lock);
	list_for_each_entry_safe(req, tmp, &gna_priv->request_list, node) {
		/* only requests still to run: one that has completed holds a result for its WAIT, whichever way it ran
		 * (a queued one has released its buffers by now, one started from COMPUTE not), so it stays collectable */
		if (req->drm_f != file || READ_ONCE(req->state) == DONE)
			continue;
		for (i = 0; i < req->buffer_count; i++) {
			if (req->buffer_list[i].gna.handle == handle) {
				/* off the list for good (an empty node tells a racing WAIT the list's reference is taken),
				 * onto ours through reap, which nothing else touches */
				list_del_init(&req->node);
				list_add_tail(&req->reap, &gone);
				break;
			}
		}
	}
	mutex_unlock(&gna_priv->reqlist_lock);
	gna_hw_release(gna_priv);

	list_for_each_entry_safe(req, tmp, &gone, reap) {
		list_del(&req->reap);
		/* queued and cancelled: it never runs -- complete it -ECANCELED and count it out (a WAIT of another thread
		 * may be sleeping on it), and its work's own reference is ours to put; one that already ran did all that */
		if (cancel_work_sync(&req->work)) {
			gna_request_cancelled(gna_priv, req);
			kref_put(&req->refcount, gna_request_release);
		}
		kref_put(&req->refcount, gna_request_release);   /* the list's reference */
	}

	/* after the requests the loop waited for: one already waiting for the hardware may have mapped the object again */
	gna_hw_acquire(gna_priv);
	if (!drm_dev_is_unplugged(&gna_priv->drm))   /* after remove the descriptor is freed (see gna_request_file_close) */
		gna_mmu_forget(gna_priv, obj);
	gna_hw_release(gna_priv);
}
