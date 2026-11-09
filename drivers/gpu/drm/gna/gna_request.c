// SPDX-License-Identifier: GPL-2.0-only
// Copyright(c) 2017-2022 Intel Corporation

#include <drm/drm_drv.h>
#include <drm/drm_file.h>
#include <drm/drm_gem.h>
#include <drm/drm_gem_shmem_helper.h>

#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/math.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/sched/signal.h>
#include <linux/slab.h>
#include <linux/timekeeping.h>
#include <linux/uaccess.h>

#include "gna_device.h"
#include "gna_hw.h"
#include "gna_mem.h"
#include "gna_request.h"
#include "gna_score.h"

int gna_validate_score_config(struct gna_compute_cfg *compute_cfg,
			struct gna_device *gna_priv)
{
	size_t buffers_size;

	if (compute_cfg->gna_mode > GNA_MODE_XNN) {
		dev_dbg(gna_dev(gna_priv), "invalid mode: %d\n", compute_cfg->gna_mode);
		return -EINVAL;
	}

	if (compute_cfg->layer_count > gna_priv->info.max_layer_count) {
		dev_dbg(gna_dev(gna_priv), "max layer count exceeded: %u > %u\n",
			compute_cfg->layer_count, gna_priv->info.max_layer_count);
		return -EINVAL;
	}

	if (compute_cfg->buffer_count == 0) {
		dev_dbg(gna_dev(gna_priv), "no buffers\n");
		return -EINVAL;
	}

	/* a buffer is at least a page of the device's address space; more cannot be mapped, and the duplicate check is
	 * quadratic in the count */
	if (compute_cfg->buffer_count > gna_priv->info.max_hw_mem / PAGE_SIZE) {
		dev_dbg(gna_dev(gna_priv), "too many buffers: %llu\n", compute_cfg->buffer_count);
		return -EINVAL;
	}

	buffers_size = sizeof(struct gna_buffer) * compute_cfg->buffer_count;
	if (!access_ok(u64_to_user_ptr(compute_cfg->buffers_ptr), buffers_size))
		return -EACCES;

	return 0;
}

static void gna_request_update_status(struct gna_request *score_request)
{
	struct gna_device *gna_priv = score_request->gna_priv;
	/* The gna_priv's hw_status should be updated first */
	u32 hw_status = gna_priv->hw_status;
	u32 stall_cycles;
	u32 total_cycles;

	/* Technically, the time stamp can be a bit later than
	 * when the hw actually completed scoring. Here we just
	 * do our best in a deferred work, unless we want to
	 * tax isr for a more accurate record.
	 */
	score_request->drv_perf.hw_completed = ktime_get_ns();

	score_request->hw_status = hw_status;

	score_request->status = gna_parse_hw_status(gna_priv, hw_status);

	if (gna_hw_perf_enabled(gna_priv)) {
		if (hw_status & GNA_STS_STATISTICS_VALID) {
			total_cycles = gna_reg_read(gna_priv, GNA_MMIO_PTC);
			stall_cycles = gna_reg_read(gna_priv, GNA_MMIO_PSC);
			score_request->hw_perf.total = total_cycles;
			score_request->hw_perf.stall = stall_cycles;
		} else
			dev_warn(gna_dev(gna_priv), "GNA statistics missing\n");
	}
	if (unlikely(hw_status & GNA_ERROR))
		gna_print_error_status(gna_priv, hw_status);
}

static void gna_request_make_zombie(struct gna_request *score_request)
{
	int i;

	for (i = 0; i < score_request->buffer_count; i++) {
		kvfree((void *)(uintptr_t)score_request->buffer_list[i].gna.patches_ptr);
		drm_gem_object_put(&score_request->buffer_list[i].gem->base.base);
	}
	kvfree(score_request->buffer_list);
	score_request->buffer_list = NULL;
	score_request->buffer_count = 0;
}

/*
 * Drop a request's runtime-PM reference. The device uses autosuspend, so re-arm its timer instead of
 * pm_runtime_put(): that queues the runtime-idle work and wakes a kernel worker on every request, only for the work
 * to find the autosuspend delay not yet expired. Asynchronous, so allowed in the interrupt.
 */
static int gna_pm_put(struct gna_device *gna_priv)
{
	pm_runtime_mark_last_busy(gna_dev(gna_priv));
	return pm_runtime_put_autosuspend(gna_dev(gna_priv));
}

/*
 * Mark a request DONE and wake its waiters, under the wait queue's lock. Whoever completes a request calls this last
 * before giving back the hardware, and touches the request no more: a WAIT that has seen DONE takes the lock once
 * (gna_request_wait_done()), and the paths that drop requests take the hardware first, so neither can free the
 * request while its completion still uses it.
 */
static void gna_request_set_done(struct gna_request *score_request)
{
	unsigned long flags;

	spin_lock_irqsave(&score_request->waitq.lock, flags);
	score_request->state = DONE;
	wake_up_all_locked(&score_request->waitq);
	spin_unlock_irqrestore(&score_request->waitq.lock, flags);
}

void gna_request_wait_done(struct gna_request *score_request)
{
	spin_lock_irq(&score_request->waitq.lock);
	spin_unlock_irq(&score_request->waitq.lock);
}

/* Queue a listed request's work, which holds its own reference: put when it has run, or by whoever cancels it. */
static void gna_request_queue_locked(struct gna_device *gna_priv, struct gna_request *score_request)
{
	lockdep_assert_held(&gna_priv->reqlist_lock);
	kref_get(&score_request->refcount);
	queue_work(gna_priv->request_wq, &score_request->work);
}

/*
 * A request that will never run -- its queued work cancelled, or dropped before it was queued -- by GEM_FREE or close:
 * count it out and complete it, -ECANCELED, so a WAIT already sleeping on it (another thread of the client) returns.
 */
void gna_request_cancelled(struct gna_device *gna_priv, struct gna_request *score_request)
{
	score_request->status = -ECANCELED;
	score_request->drv_perf.completion = ktime_get_ns();
	atomic_dec(&gna_priv->enqueued_requests);
	gna_request_set_done(score_request);
}

/* the queued request on the hardware, for the interrupt to recognise its completion; NULL when there is none */
static void gna_set_queued_req(struct gna_device *gna_priv, struct gna_request *score_request)
{
	unsigned long flags;

	spin_lock_irqsave(&gna_priv->direct_lock, flags);
	gna_priv->queued_req = score_request;
	spin_unlock_irqrestore(&gna_priv->direct_lock, flags);
}

static void gna_request_process(struct work_struct *work)
{
	struct gna_request *score_request;
	struct gna_device *gna_priv;
	unsigned long hw_timeout;
	int ret;

	score_request = container_of(work, struct gna_request, work);
	gna_priv = score_request->gna_priv;

	gna_hw_acquire(gna_priv);        /* after a directly started request, if one is on the hardware */
	/* a queued request always completes by interrupt; a WAIT that read it polled meanwhile polls its state, which
	 * reaches DONE just the same */
	WRITE_ONCE(score_request->polled, false);
	WRITE_ONCE(score_request->state, ACTIVE);

	score_request->drv_perf.pre_processing = ktime_get_ns();

	ret = pm_runtime_get_sync(gna_dev(gna_priv));
	if (ret < 0 && ret != -EACCES) {
		dev_warn(gna_dev(gna_priv), "pm_runtime_get_sync() failed: %d\n", ret);
		score_request->status = -ENODEV;
		pm_runtime_put_noidle(gna_dev(gna_priv));
		goto tail;
	}

	/* Set busy flag before kicking off HW. The isr will clear it and wake up us. There is
	 * no difference if isr is missed in a timeout situation of the last request. We just
	 * always set it busy and let the wait_event_timeout check the reset.
	 * wq:  X -> true
	 * isr: X -> false
	 */
	gna_priv->dev_busy = true;
	gna_set_queued_req(gna_priv, score_request);   /* the interrupt claims only its completion (gna_interrupt) */

	ret = gna_score(score_request);
	if (ret) {
		gna_set_queued_req(gna_priv, NULL);
		if (gna_pm_put(gna_priv) < 0)
			dev_warn(gna_dev(gna_priv), "pm_runtime_put_autosuspend() failed: %d\n", ret);
		score_request->status = ret;
		goto tail;
	}

	score_request->drv_perf.processing = ktime_get_ns();

	hw_timeout = gna_priv->recovery_timeout_jiffies;

	hw_timeout = wait_event_timeout(gna_priv->dev_busy_waitq,
			!gna_priv->dev_busy, hw_timeout);
	gna_set_queued_req(gna_priv, NULL);

	if (!hw_timeout)
		dev_warn(gna_dev(gna_priv), "hardware timeout occurred\n");

	gna_priv->hw_status = gna_reg_read(gna_priv, GNA_MMIO_STS);

	gna_request_update_status(score_request);

	ret = gna_finish_hw(gna_priv, hw_timeout && score_request->status == 0);
	if (ret < 0 && score_request->status == 0)
		score_request->status = ret; // -ETIMEDOUT

	ret = gna_pm_put(gna_priv);
	if (ret < 0)
		dev_warn(gna_dev(gna_priv), "pm_runtime_put_autosuspend() failed: %d\n", ret);

	/* the page tables stay mapped for the next request (gna_mmu_map_request) */
tail:
	score_request->drv_perf.completion = ktime_get_ns();
	gna_request_make_zombie(score_request);
	atomic_dec(&gna_priv->enqueued_requests);
	gna_request_set_done(score_request);
	gna_hw_release(gna_priv);
	kref_put(&score_request->refcount, gna_request_release);   /* the work's own reference (gna_enqueue_request) */
}

/*
 * Finish a request started from COMPUTE (gna_request_start_direct): from the interrupt -- atomic context, so
 * nothing here sleeps or frees; the request's buffers are released with its last reference, in WAIT -- or from WAIT
 * when the interrupt never came (timed_out).
 */
void gna_request_complete_direct(struct gna_request *score_request, bool timed_out)
{
	struct gna_device *gna_priv = score_request->gna_priv;
	int ret;

	gna_priv->hw_status = gna_reg_read(gna_priv, GNA_MMIO_STS);
	gna_request_update_status(score_request);
	/*
	 * A request run with its interrupt masked (polled) leaves its completion latched: a request that enables the
	 * interrupt would take it at once, before its own work is done, and read "not completed". A following polled
	 * request is unaffected (START clears the completion bit), so the abort that clears the latch waits for the
	 * start that needs it (gna_score); the abort, whenever it runs, clears the mark.
	 */
	if (!timed_out && score_request->status == 0 && score_request->polled)
		gna_priv->completion_latched = true;
	ret = gna_finish_hw(gna_priv, !timed_out && score_request->status == 0);
	if (timed_out)
		score_request->status = -ETIMEDOUT;
	else if (ret < 0 && score_request->status == 0)
		score_request->status = ret;
	gna_pm_put(gna_priv);

	score_request->drv_perf.completion = ktime_get_ns();
	atomic_dec(&gna_priv->enqueued_requests);
	gna_request_set_done(score_request);
	gna_hw_release(gna_priv);
}

/* how often a request started with its interrupt masked was completed, and by whom */
static unsigned int polled_reclaim_acquire;    /* by the next user of the hardware */
static unsigned int polled_reclaim_wait;       /* by its WAIT, already finished */
static unsigned int polled_wait_polls;         /* WAITs that found the hardware still running and polled */
static unsigned int polled_sts_done_at_start;  /* STS bit 0 still set right after START: polling would be wrong */
module_param(polled_sts_done_at_start, uint, 0444);
module_param(polled_reclaim_acquire, uint, 0444);
module_param(polled_reclaim_wait, uint, 0444);
module_param(polled_wait_polls, uint, 0444);

/*
 * The hardware is owned by one request at a time (hw_sem). A request started with its completion interrupt masked
 * (GNA_FLAG_SCORE_POLLED) keeps owning it after the hardware has finished, until someone reads the status: complete
 * it here if it has finished. True when a request was completed. Process context.
 */
bool gna_hw_reclaim(struct gna_device *gna_priv)
{
	struct gna_request *req = NULL;
	unsigned long flags;

	/*
	 * Only once its START is written (hw_started): until then STS still carries the previous request's completion --
	 * a clean finish clears only the saturation bit -- and the request would be completed before it ran, giving the
	 * hardware to another request that its own START then overwrites.
	 */
	spin_lock_irqsave(&gna_priv->direct_lock, flags);
	if (gna_priv->direct_req && gna_priv->direct_req->polled && gna_priv->direct_req->hw_started &&
	    (gna_reg_read(gna_priv, GNA_MMIO_STS) & (GNA_STS_SCORE_COMPLETED | GNA_ERROR))) {
		req = gna_priv->direct_req;
		gna_priv->direct_req = NULL;
	}
	spin_unlock_irqrestore(&gna_priv->direct_lock, flags);
	if (!req)
		return false;
	gna_priv->dev_busy = false;
	gna_request_complete_direct(req, false);
	return true;
}

/*
 * A request started from COMPUTE that has held the hardware past the recovery timeout: its completion is not coming
 * (the accelerator hung) and its client may never WAIT, so nothing else would give the hardware back. Complete it as
 * timed out, as its WAIT would -- the timeout a queued request gets in gna_request_process(). True when one was.
 */
static bool gna_hw_reclaim_stuck(struct gna_device *gna_priv)
{
	u64 limit_ns = jiffies_to_nsecs(gna_priv->recovery_timeout_jiffies);
	struct gna_request *req;
	unsigned long flags;

	spin_lock_irqsave(&gna_priv->direct_lock, flags);
	req = gna_priv->direct_req;
	if (req && req->hw_started && ktime_get_ns() - req->drv_perf.processing > limit_ns)
		gna_priv->direct_req = NULL;
	else
		req = NULL;
	spin_unlock_irqrestore(&gna_priv->direct_lock, flags);
	if (!req)
		return false;
	dev_warn(gna_dev(gna_priv), "hardware timeout occurred\n");
	gna_request_complete_direct(req, true);
	return true;
}

/*
 * Take the hardware. Every path that needs it goes through these, so a finished masked request is always reclaimed
 * by whoever needs the hardware next -- none of them can wait for an interrupt that will not come -- and a hung one
 * after the recovery timeout.
 */
static bool gna_hw_try_acquire(struct gna_device *gna_priv)
{
	if (!down_trylock(&gna_priv->hw_sem))
		return true;
	if (!gna_hw_reclaim(gna_priv))
		return false;
	polled_reclaim_acquire++;
	return !down_trylock(&gna_priv->hw_sem);
}

void gna_hw_acquire(struct gna_device *gna_priv)
{
	while (down_timeout(&gna_priv->hw_sem, msecs_to_jiffies(1))) {
		if (gna_hw_reclaim(gna_priv))
			polled_reclaim_acquire++;
		else
			gna_hw_reclaim_stuck(gna_priv);
	}
}

void gna_hw_release(struct gna_device *gna_priv)
{
	up(&gna_priv->hw_sem);
}

/*
 * WAIT on a request submitted with GNA_FLAG_SCORE_POLLED: once it runs with its interrupt masked, complete it if the
 * hardware has finished, else poll at a short interval until it has or the timeout passes. Until then -- not started
 * yet, or queued behind another request, which the worker runs with its interrupt enabled -- there is nothing to
 * poll: sleep until it completes or starts from COMPUTE (both wake the wait queue). Returns as
 * wait_event_interruptible_timeout does: > 0 done, 0 timed out, -ERESTARTSYS on a signal.
 */
long gna_request_wait_polled(struct gna_device *gna_priv, struct gna_request *score_request, u32 timeout_ms)
{
	unsigned long end = jiffies + msecs_to_jiffies(timeout_ms);
	bool polled = false;
	long left;

	while (READ_ONCE(score_request->state) != DONE) {
		if (READ_ONCE(score_request->state) == NEW || !READ_ONCE(score_request->polled)) {
			left = (long)(end - jiffies);   /* one read: the macro below evaluates its timeout more than once */
			if (left <= 0)
				return 0;
			left = wait_event_interruptible_timeout(score_request->waitq,
					READ_ONCE(score_request->state) == DONE ||
					(READ_ONCE(score_request->state) != NEW && READ_ONCE(score_request->polled)),
					left);
			if (left < 0)
				return left;
			continue;
		}
		if (gna_hw_reclaim(gna_priv)) {
			if (!polled)
				polled_reclaim_wait++;
			continue;
		}
		if (signal_pending(current))
			return -ERESTARTSYS;
		if (time_after(jiffies, end))
			return 0;
		if (!polled) {
			polled = true;
			polled_wait_polls++;
		}
		usleep_range(50, 100);
	}
	return 1;
}

/*
 * Nothing queued and nothing on the hardware: start the request here, in the submitting thread, and return
 * without waiting -- no workqueue thread is woken, and the interrupt completes it (gna_request_complete_direct).
 * Called with hw_sem held; false (hw_sem given back) when the device could not be resumed, for the workqueue to
 * report the failure as before, or when a signal interrupted the mapping, for the workqueue to run it.
 */
static bool gna_request_start_direct(struct gna_device *gna_priv, struct gna_request *score_request)
{
	unsigned long flags;
	bool owned;
	int ret;

	ret = pm_runtime_get_sync(gna_dev(gna_priv));
	if (ret < 0 && ret != -EACCES) {
		pm_runtime_put_noidle(gna_dev(gna_priv));
		up(&gna_priv->hw_sem);
		return false;
	}

	WRITE_ONCE(score_request->state, ACTIVE);
	score_request->drv_perf.pre_processing = ktime_get_ns();
	gna_priv->dev_busy = true;
	spin_lock_irqsave(&gna_priv->direct_lock, flags);
	gna_priv->direct_req = score_request;       /* before the hardware starts: its interrupt completes it */
	spin_unlock_irqrestore(&gna_priv->direct_lock, flags);
	if (score_request->polled)   /* a WAIT that slept on it before it started polls it from now on */
		wake_up_all(&score_request->waitq);
	score_request->drv_perf.processing = ktime_get_ns();

	ret = gna_score(score_request);
	/* completion is read from STS bit 0 for a masked request, so it must not still read set from the previous one */
	if (!ret && score_request->polled &&
	    (gna_reg_read(gna_priv, GNA_MMIO_STS) & GNA_STS_SCORE_COMPLETED))
		polled_sts_done_at_start++;
	if (ret) {   /* on success gna_score() set hw_started with START */
		spin_lock_irqsave(&gna_priv->direct_lock, flags);
		owned = gna_priv->direct_req == score_request;
		if (owned)
			gna_priv->direct_req = NULL;
		spin_unlock_irqrestore(&gna_priv->direct_lock, flags);
		if (owned) {
			gna_pm_put(gna_priv);
			if (ret == -EINTR || ret == -ERESTARTSYS) {
				/*
				 * A signal of the submitting thread cut short a lock wait in the mapping
				 * (drm_gem_shmem_get_pages_sgt() takes the reservation lock interruptibly): no failure of
				 * the request, and COMPUTE has succeeded. Give it to the worker, which takes no signals.
				 */
				WRITE_ONCE(score_request->state, NEW);
				up(&gna_priv->hw_sem);
				return false;
			}
			score_request->status = ret;
			score_request->drv_perf.completion = ktime_get_ns();
			atomic_dec(&gna_priv->enqueued_requests);
			gna_request_set_done(score_request);
			up(&gna_priv->hw_sem);
		}
	}
	return true;
}

static struct gna_request *gna_request_create(struct drm_file *file,
				       struct gna_compute_cfg *compute_cfg)
{

	struct gna_device *gna_priv = to_gna_device(file->minor->dev);
	struct gna_request *score_request;

	score_request = kzalloc(sizeof(*score_request), GFP_KERNEL);
	if (!score_request)
		return NULL;
	kref_init(&score_request->refcount);

	dev_dbg(gna_dev(gna_priv), "labase: %d, lacount: %d\n",
		compute_cfg->layer_base, compute_cfg->layer_count);

	score_request->request_id = atomic64_inc_return(&gna_priv->request_count);
	score_request->compute_cfg = *compute_cfg;
	score_request->drm_f = file;
	score_request->gna_priv = gna_priv;
	score_request->state = NEW;
	/* before it is listed: a WAIT can find it from then on and picks how to wait by this (gna_wait_ioctl) */
	score_request->polled = !!(compute_cfg->flags & GNA_FLAG_SCORE_POLLED);
	init_waitqueue_head(&score_request->waitq);
	INIT_WORK(&score_request->work, gna_request_process);
	INIT_LIST_HEAD(&score_request->node);

	return score_request;
}

/*
 * returns true if [inner_offset, inner_size) is embraced by [0, outer_size). False otherwise.
 */
static bool gna_validate_ranges(u64 outer_size, u64 inner_offset, u64 inner_size)
{
	return inner_offset < outer_size &&
		inner_size <= (outer_size - inner_offset);
}

static int gna_validate_patches(struct gna_device *gna_priv, __u64 buffer_size,
				struct gna_memory_patch *patches, u64 count)
{
	u64 idx;

	for (idx = 0; idx < count; ++idx) {
		if (patches[idx].size > 8) {
			dev_dbg(gna_dev(gna_priv), "invalid patch size: %llu\n", patches[idx].size);
			return -EINVAL;
		}

		if (!gna_validate_ranges(buffer_size, patches[idx].offset, patches[idx].size)) {
			dev_dbg(gna_dev(gna_priv),
				"patch out of bounds. buffer size: %llu, patch offset/size:%llu/%llu\n",
				buffer_size, patches[idx].offset, patches[idx].size);
			return -EINVAL;
		}
	}

	return 0;
}

static int gna_buffer_fill_patches(struct gna_buffer *buffer, struct gna_device *gna_priv)
{
	__u64 patches_user = buffer->patches_ptr;
	struct gna_memory_patch *patches;
	/* At this point, the buffer points to a memory region in kernel space where the copied
	 * patches_ptr also lives, but the value of it is still an address from user space. This
	 * function will set patches_ptr to either an address in kernel space or null before it
	 * exits.
	 */
	u64 patch_count;
	int ret;

	buffer->patches_ptr = 0;
	patch_count = buffer->patch_count;
	if (!patch_count)
		return 0;

	/* every patch writes at least one byte of the buffer: more patches than bytes is no request at all */
	if (patch_count > buffer->size)
		return -EINVAL;

	/* a client-sized allocation: past INT_MAX kvmalloc refuses it, and without __GFP_NOWARN it WARNs (a panic with
	 * panic_on_warn) on the client's behalf */
	patches = kvmalloc_array(patch_count, sizeof(struct gna_memory_patch), GFP_KERNEL_ACCOUNT | __GFP_NOWARN);
	if (!patches)
		return -ENOMEM;

	if (copy_from_user(patches, u64_to_user_ptr(patches_user),
				sizeof(struct gna_memory_patch) * patch_count)) {
		ret = -EFAULT;
		goto err_fill_patches;
	}

	ret = gna_validate_patches(gna_priv, buffer->size, patches, patch_count);
	if (ret) {
		dev_dbg(gna_dev(gna_priv), "buffer %p: patches' validation failed\n", buffer);
		goto err_fill_patches;
	}

	buffer->patches_ptr = (uintptr_t)patches;

	return 0;

err_fill_patches:
	kvfree(patches);
	return ret;
}

static int gna_request_fill_buffers(struct gna_request *score_request,
				    struct gna_compute_cfg *compute_cfg)
{
	struct gna_buffer_with_object *buffer_list;
	struct gna_buffer_with_object *buffer;
	struct gna_buffer *cfg_buffers;
	struct drm_gem_object *drmgemo;
	struct gna_device *gna_priv;
	u64 buffers_total_size = 0;
	size_t gem_obj_size;
	u64 buffer_count;
	u32 handle;
	u64 i, j;
	int ret;


	gna_priv = score_request->gna_priv;

	buffer_count = compute_cfg->buffer_count;
	buffer_list = kvmalloc_array(buffer_count, sizeof(*buffer_list), GFP_KERNEL_ACCOUNT | __GFP_NOWARN);
	if (!buffer_list)
		return -ENOMEM;

	cfg_buffers = u64_to_user_ptr(compute_cfg->buffers_ptr);
	for (i = 0; i < buffer_count; ++i) {
		if (copy_from_user(&buffer_list[i].gna, cfg_buffers+i,
					sizeof(buffer_list[i].gna))) {
			ret = -EFAULT;
			goto err_free_buffers;
		}
		buffer_list[i].gem = NULL;
	}

	for (i = 0; i < buffer_count; i++) {
		buffer = &buffer_list[i];
		handle = buffer->gna.handle;

		if (buffer->gna.offset != 0) {
			dev_dbg(gna_dev(gna_priv), "buffer->offset = %llu for handle %u in score config\n",
				buffer->gna.offset, buffer->gna.handle);
			ret = -EINVAL;
			goto err_zero_patch_user_ptr;   /* not a bare return: the buffers before this one hold references */
		}

		cond_resched();   /* the duplicate check is quadratic in a count the client chooses */
		for (j = 0; j < i; j++) {
			if (buffer_list[j].gna.handle == handle) {
				dev_dbg(gna_dev(gna_priv),
					"doubled memory id in score config; id:%u\n", handle);
				ret = -EINVAL;
				goto err_zero_patch_user_ptr;
			}
		}

		drmgemo = drm_gem_object_lookup(score_request->drm_f, handle);

		if (!drmgemo) {
			dev_dbg(gna_dev(gna_priv), "memory object %u not found\n", handle);
			ret = -EINVAL;
			goto err_zero_patch_user_ptr;
		}

		// we are still in sys call context, but prior request is enqueued.
		// request may slip into queue while some gna_gem_object being deleted
		// border case + not too much harm.
		buffer->gem = to_gna_gem_obj(to_drm_gem_shmem_obj(drmgemo));

		gem_obj_size = drmgemo->size;

		if (!gna_validate_ranges(gem_obj_size, 0, buffer->gna.size)) {
			dev_dbg(gna_dev(gna_priv),
				"buffer out of bounds. mo size: %zu, buffer size:%llu\n",
				gem_obj_size, buffer->gna.size);
			ret = -EINVAL;
			goto err_zero_patch_user_ptr;
		}

		/*
		 * The page tables map every page of the object (gna_mmu_add_locked), not only buffer->gna.size of it: count
		 * the object, or a request whose objects outgrow the tables is mapped in part and run anyway.
		 */
		buffers_total_size += gem_obj_size;
		if (buffers_total_size > gna_priv->info.max_hw_mem) {
			dev_dbg(gna_dev(gna_priv), "buffers' %p total size too big\n", buffer);
			ret = -EINVAL;
			goto err_zero_patch_user_ptr;
		}

		ret = gna_buffer_fill_patches(&buffer->gna, gna_priv);
		if (ret)
			goto err_free_patches;
	}

	score_request->buffer_list = buffer_list;
	score_request->buffer_count = buffer_count;

	return 0;

err_zero_patch_user_ptr:
	/* patches_ptr may still hold an address in userspace.
	 * Don't pass it to kvfree().
	 */
	buffer->gna.patches_ptr = 0;

err_free_patches:
	/* patches_ptr of each processed buffer should be either
	 * null or pointing to an allocated memory block in the
	 * kernel at this point.
	 */
	for (j = 0; j <= i; j++) {
		kvfree((void *)(uintptr_t)buffer_list[j].gna.patches_ptr);
		drm_gem_object_put(&buffer_list[j].gem->base.base);
	}

err_free_buffers:
	kvfree(buffer_list);
	return ret;
}

int gna_enqueue_request(struct gna_compute_cfg *compute_cfg,
			struct drm_file *file, u64 *request_id)
{
	bool is_qos = !!(compute_cfg->flags & GNA_FLAG_SCORE_QOS);
	struct gna_device *gna_priv = to_gna_device(file->minor->dev);
	struct gna_request *score_request;
	u64 pos_in_queue;
	bool direct;
	int ret;

	/* a finished masked request still counts as queued until reclaimed: reclaim it first, so this one can start
	 * directly instead of queueing behind it */
	if (gna_hw_reclaim(gna_priv))
		polled_reclaim_acquire++;
	pos_in_queue = atomic_inc_return(&gna_priv->enqueued_requests);
	if (is_qos && pos_in_queue != 1) {
		ret = -EBUSY;
		goto ERR_UNQUEUE_REQUEST;
	}

	score_request = gna_request_create(file, compute_cfg);
	if (!score_request) {
		ret = -ENOMEM;
		goto ERR_UNQUEUE_REQUEST;
	}

	ret = gna_request_fill_buffers(score_request, compute_cfg);
	if (ret) {
		kref_put(&score_request->refcount, gna_request_release);
		goto ERR_UNQUEUE_REQUEST;
	}

	*request_id = score_request->request_id;   /* now: once it is on the list it may be dropped before the put below */

	/*
	 * A request to start here takes the hardware BEFORE it is listed: GEM_FREE and close take requests off the list
	 * only with the hardware held, so they cannot drop one between its listing and its start -- it lives on the
	 * list's and this function's references alone, and would be freed on the hardware.
	 */
	direct = pos_in_queue == 1 && gna_hw_try_acquire(gna_priv);

	/* a queued request is listed and queued under one hold of reqlist_lock: GEM_FREE and close, which take requests
	 * off the list under it, never find one listed but not yet queued -- cancel_work_sync() would find nothing, and
	 * the request would run (and map its objects) after they were done with it */
	kref_get(&score_request->refcount);
	mutex_lock(&gna_priv->reqlist_lock);
	list_add_tail(&score_request->node, &gna_priv->request_list);
	if (!direct)
		gna_request_queue_locked(gna_priv, score_request);
	mutex_unlock(&gna_priv->reqlist_lock);

	if (direct && !gna_request_start_direct(gna_priv, score_request)) {
		/* not started (the device could not be resumed, or a signal interrupted the mapping) and the hardware
		 * given back: queue it instead -- unless GEM_FREE or close took it off the list in between, which leaves it
		 * to be counted out here */
		mutex_lock(&gna_priv->reqlist_lock);
		if (!list_empty(&score_request->node))
			gna_request_queue_locked(gna_priv, score_request);
		else
			gna_request_cancelled(gna_priv, score_request);
		mutex_unlock(&gna_priv->reqlist_lock);
	}
	kref_put(&score_request->refcount, gna_request_release);

	return 0;

ERR_UNQUEUE_REQUEST:
	atomic_dec(&gna_priv->enqueued_requests);
	return ret;
}

void gna_request_release(struct kref *ref)
{
	struct gna_request *score_request =
		container_of(ref, struct gna_request, refcount);
	gna_request_make_zombie(score_request);
	wake_up_interruptible_all(&score_request->waitq);
	kfree(score_request);
}

/*
 * The client's file is closing: drop the requests it never collected. WAIT takes a request off the list only when it
 * completes in time, so one the client never waited for, or whose WAIT timed out, stayed there with its buffers' GEM
 * references until the device went away ("Memory manager not clean during takedown" on unbind). No ioctl of this
 * file can be running here.
 */
void gna_request_file_close(struct gna_device *gna_priv, struct drm_file *file)
{
	struct gna_request *req, *tmp;
	unsigned long flags;
	LIST_HEAD(closing);

	/* one of them started from COMPUTE and still on the hardware: nobody will read it, so stop it as a timed-out
	 * WAIT does -- before its buffers go */
	spin_lock_irqsave(&gna_priv->direct_lock, flags);
	req = gna_priv->direct_req;
	if (req && req->drm_f == file)
		gna_priv->direct_req = NULL;
	else
		req = NULL;
	spin_unlock_irqrestore(&gna_priv->direct_lock, flags);
	if (req)
		gna_request_complete_direct(req, true);

	/* with the hardware held, a completion already under way (the interrupt, a reclaim) has finished with its
	 * request: completions give the hardware back last (gna_request_set_done) */
	gna_hw_acquire(gna_priv);
	mutex_lock(&gna_priv->reqlist_lock);
	list_for_each_entry_safe(req, tmp, &gna_priv->request_list, node) {
		if (req->drm_f == file) {
			list_del_init(&req->node);
			list_add_tail(&req->reap, &closing);
		}
	}
	mutex_unlock(&gna_priv->reqlist_lock);
	gna_hw_release(gna_priv);

	list_for_each_entry_safe(req, tmp, &closing, reap) {
		list_del(&req->reap);
		/* queued but not started: it will not run, so it is no longer queued either and its own reference is ours
		 * to put; already running: wait for it (bounded by the worker's own hardware timeout) */
		if (cancel_work_sync(&req->work)) {
			gna_request_cancelled(gna_priv, req);
			kref_put(&req->refcount, gna_request_release);
		}
		kref_put(&req->refcount, gna_request_release);   /* the list's reference */
	}

	/* last, after any request of this client the loop waited for has mapped them: the page tables may be kept for
	 * its objects, pinning their pages (up to the whole 256 MiB) after it has gone -- drop them, the next request
	 * maps its own */
	gna_hw_acquire(gna_priv);
	/* not after remove: devres has freed the descriptor the tables hang off (gna_remove() dropped them already).
	 * Checked with the hardware held: remove marks the device unplugged before it waits for the hardware. */
	if (!drm_dev_is_unplugged(&gna_priv->drm))
		gna_mmu_release(gna_priv);
	gna_hw_release(gna_priv);
}
