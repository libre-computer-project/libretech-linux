// SPDX-License-Identifier: GPL-2.0-only
// Copyright(c) 2017-2022 Intel Corporation

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

static struct scatterlist *gna_iterate_sgl(u64 sg_elems, struct scatterlist *sgl, dma_addr_t *sg_page,
					int *sg_page_len, int *sg_pages)
{
	while (sg_elems-- > 0) {
		(*sg_page) += PAGE_SIZE;
		(*sg_pages)++;
		if (*sg_pages == *sg_page_len) {
			sgl = sg_next(sgl);
			if (!sgl)
				break;

			*sg_page = sg_dma_address(sgl);
			*sg_page_len =
				round_up(sg_dma_len(sgl), PAGE_SIZE)
				>> PAGE_SHIFT;
			*sg_pages = 0;
		}
	}

	return sgl;
}


void gna_mmu_add(struct gna_device *gna_priv, struct drm_gem_shmem_object *drmshmemo)
{
	struct gna_mmu_object *mmu;
	struct scatterlist *sgl;
	dma_addr_t sg_page;
	int sg_page_len;
	u32 *pagetable;
	u32 mmu_page;
	int sg_pages;
	int i;
	int j;

	mmu = &gna_priv->mmu;
	mutex_lock(&gna_priv->mmu_lock);

	j = mmu->filled_pages;
	sgl = drmshmemo->sgt->sgl;

	if (!sgl) {
		dev_warn(gna_dev(gna_priv), "empty scatter list in memory object\n");
		goto warn_empty_sgl;
	}
	sg_page = sg_dma_address(sgl);
	sg_page_len = round_up(sg_dma_len(sgl), PAGE_SIZE) >> PAGE_SHIFT;
	sg_pages = 0;

	for (i = mmu->filled_pts; i < mmu->num_pagetables; i++) {
		if (!sgl)
			break;

		pagetable = mmu->pagetables[i];

		for (j = mmu->filled_pages; j < GNA_PT_LENGTH; j++) {
			mmu_page = sg_page >> PAGE_SHIFT;
			pagetable[j] = mmu_page;

			mmu->filled_pages++;

			sgl = gna_iterate_sgl(1, sgl, &sg_page, &sg_page_len,
					&sg_pages);
			if (!sgl)
				break;
		}

		if (j == GNA_PT_LENGTH) {
			mmu->filled_pages = 0;
			mmu->filled_pts++;
		}
	}

	mmu->hwdesc->mmu.vamaxaddr =
		(mmu->filled_pts * PAGE_SIZE * GNA_PGDIR_ENTRIES) +
		(mmu->filled_pages * PAGE_SIZE) - 1;
	dev_dbg(gna_dev(gna_priv), "vamaxaddr: %u\n", mmu->hwdesc->mmu.vamaxaddr);

warn_empty_sgl:
	mutex_unlock(&gna_priv->mmu_lock);
}

void gna_mmu_clear(struct gna_device *gna_priv)
{
	struct gna_mmu_object *mmu;
	int i;

	mmu = &gna_priv->mmu;
	mutex_lock(&gna_priv->mmu_lock);

	for (i = 0; i < mmu->filled_pts; i++)
		memset(mmu->pagetables[i], 0, PAGE_SIZE);

	if (mmu->filled_pages > 0)
		memset(mmu->pagetables[mmu->filled_pts], 0, mmu->filled_pages * GNA_PT_ENTRY_SIZE);

	mmu->filled_pts = 0;
	mmu->filled_pages = 0;
	mmu->hwdesc->mmu.vamaxaddr = 0;

	mutex_unlock(&gna_priv->mmu_lock);
}

bool gna_gem_object_put_pages_sgt(struct gna_gem_object *gnagemo)
{
	struct drm_gem_shmem_object *shmem = &gnagemo->base;
	struct drm_gem_object *drmgemo = &shmem->base;

	/*
	 * 6.18 API drift: drm_gem_shmem_object lost its private @pages_lock
	 * mutex (~6.7); the backing-store state is now serialised by the GEM
	 * object's dma_resv lock. drm_gem_shmem_put_pages() was likewise
	 * replaced by drm_gem_shmem_put_pages_locked(), which asserts the
	 * resv lock is held. Take the resv lock for the whole teardown.
	 */
	if (!dma_resv_trylock(drmgemo->resv))
		return false;

	if (shmem->sgt) {
		dma_unmap_sgtable(drmgemo->dev->dev, shmem->sgt,
				  DMA_BIDIRECTIONAL, 0);
		sg_free_table(shmem->sgt);
		kfree(shmem->sgt);
		shmem->sgt = NULL;
	}

	drm_gem_shmem_put_pages_locked(shmem);

	dma_resv_unlock(drmgemo->resv);

	return true;
}

static void gna_delete_score_requests(u32 handle, struct gna_device *gna_priv)
{
	struct gna_request *req, *temp_req;
	struct list_head *reqs_list;
	int i;

	mutex_lock(&gna_priv->reqlist_lock);

	reqs_list = &gna_priv->request_list;
	if (!list_empty(reqs_list)) {
		list_for_each_entry_safe(req, temp_req, reqs_list, node) {
			for (i = 0; i < req->buffer_count; ++i) {
				if (req->buffer_list[i].gna.handle == handle) {
					list_del_init(&req->node);
					cancel_work_sync(&req->work);
					atomic_dec(&gna_priv->enqueued_requests);
					kref_put(&req->refcount, gna_request_release);
					break;
				}
			}
		}
	}

	mutex_unlock(&gna_priv->reqlist_lock);
}

void gna_gem_obj_release_work(struct work_struct *work)
{
	struct gna_gem_object *gnagemo;

	gnagemo = container_of(work, struct gna_gem_object, work);

	gna_delete_score_requests(gnagemo->handle, to_gna_device(gnagemo->base.base.dev));

	wake_up_interruptible(&gnagemo->waitq);
}
