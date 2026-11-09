/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright 2019 Collabora ltd. */

#ifndef __PANFROST_JOB_H__
#define __PANFROST_JOB_H__

#include <uapi/drm/panfrost_drm.h>
#include <drm/gpu_scheduler.h>

struct panfrost_device;
struct panfrost_gem_object;
struct panfrost_file_priv;

struct panfrost_job {
	struct drm_sched_job base;

	struct kref refcount;

	struct panfrost_device *pfdev;
	struct panfrost_mmu *mmu;

	/* Fence to be signaled by IRQ handler when the job is complete. */
	struct dma_fence *done_fence;

	__u64 jc;
	__u32 requirements;
	__u32 flush_id;

	struct panfrost_gem_mapping **mappings;
	struct drm_gem_object **bos;
	u32 bo_count;

	/* Fence to be signaled by drm-sched once its done with the job */
	struct dma_fence *render_done_fence;

	struct panfrost_engine_usage *engine_usage;
	bool is_profiled;
	ktime_t start_time;
	u64 start_cycles;

	/* SUBMIT_BATCH: if true, bos/mappings are shared with another job in
	 * the batch (they point at the first job's arrays). This job must NOT
	 * free bos/mappings in panfrost_job_cleanup; the owning job does.
	 * The job struct itself is always heap-allocated on 6.18 and is freed
	 * unconditionally by panfrost_job_cleanup (kfree). */
	bool shared_bos;
	/* SUBMIT_BATCH secondary: the primary job that owns the shared
	 * bos/mappings arrays.  This secondary holds a refcount on it so the
	 * arrays stay alive until this job completes -- otherwise a secondary
	 * that outlives the primary dereferences freed bos (e.g. in
	 * panfrost_core_dump on a GPU timeout). NULL for a non-shared job. */
	struct panfrost_job *shared_owner;
};

int panfrost_job_init(struct panfrost_device *pfdev);
void panfrost_job_fini(struct panfrost_device *pfdev);
int panfrost_job_open(struct panfrost_file_priv *panfrost_priv);
void panfrost_job_close(struct panfrost_file_priv *panfrost_priv);
int panfrost_job_get_slot(struct panfrost_job *job);
int panfrost_job_push(struct panfrost_job *job);
void panfrost_job_put(struct panfrost_job *job);
void panfrost_job_enable_interrupts(struct panfrost_device *pfdev);
void panfrost_job_suspend_irq(struct panfrost_device *pfdev);
int panfrost_job_is_idle(struct panfrost_device *pfdev);

#endif
