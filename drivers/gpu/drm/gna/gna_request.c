// SPDX-License-Identifier: GPL-2.0-only
// Copyright(c) 2017-2022 Intel Corporation

#include <drm/drm_file.h>
#include <drm/drm_gem.h>
#include <drm/drm_gem_shmem_helper.h>

#include <linux/atomic.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/math.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
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

	buffers_size = sizeof(struct gna_buffer) * compute_cfg->buffer_count;
	if (!access_ok(u64_to_user_ptr(compute_cfg->buffers_ptr), buffers_size))
		return -EACCES;

	return 0;
}

static void gna_request_update_status(struct gna_request *score_request)
{
	struct gna_device *gna_priv = to_gna_device(score_request->drm_f->minor->dev);
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

static void gna_request_process(struct work_struct *work)
{
	struct gna_buffer_with_object *buffer;
	struct gna_request *score_request;
	struct gna_device *gna_priv;
	unsigned long hw_timeout;
	int ret;
	u64 i;

	score_request = container_of(work, struct gna_request, work);
	gna_priv = to_gna_device(score_request->drm_f->minor->dev);

	score_request->state = ACTIVE;

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

	ret = gna_score(score_request);
	if (ret) {
		if (pm_runtime_put(gna_dev(gna_priv)) < 0)
			dev_warn(gna_dev(gna_priv), "pm_runtime_put() failed: %d\n", ret);
		score_request->status = ret;
		goto tail;
	}

	score_request->drv_perf.processing = ktime_get_ns();

	hw_timeout = gna_priv->recovery_timeout_jiffies;

	hw_timeout = wait_event_timeout(gna_priv->dev_busy_waitq,
			!gna_priv->dev_busy, hw_timeout);

	if (!hw_timeout)
		dev_warn(gna_dev(gna_priv), "hardware timeout occurred\n");

	gna_priv->hw_status = gna_reg_read(gna_priv, GNA_MMIO_STS);

	gna_request_update_status(score_request);

	ret = gna_abort_hw(gna_priv);
	if (ret < 0 && score_request->status == 0)
		score_request->status = ret; // -ETIMEDOUT

	ret = pm_runtime_put(gna_dev(gna_priv));
	if (ret < 0)
		dev_warn(gna_dev(gna_priv), "pm_runtime_put() failed: %d\n", ret);

	gna_mmu_clear(gna_priv);

	for (i = 0, buffer = score_request->buffer_list; i < score_request->buffer_count; i++, buffer++)
		gna_gem_object_put_pages_sgt(buffer->gem);

tail:
	score_request->drv_perf.completion = ktime_get_ns();
	score_request->state = DONE;
	gna_request_make_zombie(score_request);

	atomic_dec(&gna_priv->enqueued_requests);
	wake_up_interruptible_all(&score_request->waitq);
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

	score_request->request_id = atomic_inc_return(&gna_priv->request_count);
	score_request->compute_cfg = *compute_cfg;
	score_request->drm_f = file;
	score_request->state = NEW;
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

	patches = kvmalloc_array(patch_count, sizeof(struct gna_memory_patch), GFP_KERNEL);
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


	gna_priv = to_gna_device(score_request->drm_f->minor->dev);

	buffer_count = compute_cfg->buffer_count;
	buffer_list = kvmalloc_array(buffer_count, sizeof(*buffer_list), GFP_KERNEL);
	if (!buffer_list)
		return -ENOMEM;

	cfg_buffers = u64_to_user_ptr(compute_cfg->buffers_ptr);
	for (i = 0; i < buffer_count; ++i) {
		if (copy_from_user(&buffer_list[i].gna, cfg_buffers+i,
					sizeof(*buffer_list))) {
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
			return -EINVAL;
		}

		for (j = 0; j < i; j++) {
			if (buffer_list[j].gna.handle == handle) {
				dev_dbg(gna_dev(gna_priv),
					"doubled memory id in score config; id:%u\n", handle);
				ret = -EINVAL;
				goto err_zero_patch_user_ptr;
			}
		}

		buffers_total_size +=
			round_up(buffer->gna.size, PAGE_SIZE);
		if (buffers_total_size > gna_priv->info.max_hw_mem) {
			dev_dbg(gna_dev(gna_priv), "buffers' %p total size too big\n", buffer);
			ret = -EINVAL;
			goto err_zero_patch_user_ptr;
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
	int ret;

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

	kref_get(&score_request->refcount);
	mutex_lock(&gna_priv->reqlist_lock);
	list_add_tail(&score_request->node, &gna_priv->request_list);
	mutex_unlock(&gna_priv->reqlist_lock);

	queue_work(gna_priv->request_wq, &score_request->work);
	kref_put(&score_request->refcount, gna_request_release);

	*request_id = score_request->request_id;

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
