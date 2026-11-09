// SPDX-License-Identifier: GPL-2.0-only
// Copyright(c) 2017-2022 Intel Corporation

#include <drm/drm_device.h>
#include <drm/drm_drv.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_file.h>

#include <linux/jiffies.h>
#include <linux/kref.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <uapi/drm/gna_drm.h>

#include "gna_device.h"
#include "gna_gem.h"
#include "gna_request.h"

int gna_score_ioctl(struct drm_device *dev, void *data,
		struct drm_file *file)
{
	union gna_compute *score_args = data;
	u64 request_id;
	int ret, idx;

	/* may start the hardware: gna_remove()'s drm_dev_unplug() waits for this section, then refuses it */
	if (!drm_dev_enter(dev, &idx))
		return -ENODEV;

	ret = gna_validate_score_config(&score_args->in.config, to_gna_device(dev));
	if (!ret)
		ret = gna_enqueue_request(&score_args->in.config, file, &request_id);
	drm_dev_exit(idx);
	if (ret)
		return ret;

	score_args->out.request_id = request_id;

	return 0;
}

static struct gna_request *gna_find_request_by_id(u64 req_id, struct gna_device *gna_priv)
{
	struct gna_request *req, *found_req;
	struct list_head *reqs_list;

	mutex_lock(&gna_priv->reqlist_lock);

	reqs_list = &gna_priv->request_list;
	found_req = NULL;
	if (!list_empty(reqs_list)) {
		list_for_each_entry(req, reqs_list, node) {
			if (req_id == req->request_id) {
				found_req = req;
				kref_get(&found_req->refcount);
				break;
			}
		}
	}

	mutex_unlock(&gna_priv->reqlist_lock);

	return found_req;
}

int gna_wait_ioctl(struct drm_device *dev, void *data,
		struct drm_file *file)
{
	struct gna_device *gna_priv = to_gna_device(dev);
	union gna_wait *wait_data = data;
	struct gna_request *score_request;
	u64 request_id;
	u32 timeout;
	long left;
	int ret = 0;

	request_id = wait_data->in.request_id;
	timeout = wait_data->in.timeout;

	score_request = gna_find_request_by_id(request_id, gna_priv);

	if (!score_request) {
		dev_dbg(gna_dev(gna_priv), "could not find request, id: %llu\n", request_id);
		return -EINVAL;
	}

	if (score_request->drm_f != file) {
		dev_dbg(gna_dev(gna_priv), "illegal file_priv: %p != %p\n", score_request->drm_f, file);
		ret = -EINVAL;
		goto out;
	}

	/* long: the jiffies left of a timeout near U32_MAX ms do not fit an int (truncated, they read as -ERESTARTSYS) */
	if (READ_ONCE(score_request->polled))   /* started with its interrupt masked: nothing will wake this wait */
		left = gna_request_wait_polled(gna_priv, score_request, timeout);
	else
		left = wait_event_interruptible_timeout(score_request->waitq, score_request->state == DONE,
							msecs_to_jiffies(timeout));
	if (left == 0 || left == -ERESTARTSYS) {
		dev_dbg(gna_dev(gna_priv), "request timed out, id: %llu\n", request_id);
		ret = -ERESTARTSYS;
		if (left == 0) {
			/* a request started from COMPUTE whose interrupt never came: complete it here */
			unsigned long flags;
			bool stolen;

			/* only once its START is written: before that, its submitter (another thread of this client) is still
			 * starting it with the hardware held, and completing it here would hand the hardware on under it */
			spin_lock_irqsave(&gna_priv->direct_lock, flags);
			stolen = gna_priv->direct_req == score_request && score_request->hw_started;
			if (stolen)
				gna_priv->direct_req = NULL;
			spin_unlock_irqrestore(&gna_priv->direct_lock, flags);
			if (stolen) {
				dev_warn(gna_dev(gna_priv), "hardware timeout occurred\n");
				gna_request_complete_direct(score_request, true);
			}
			ret = -EBUSY;
		}
		/* interrupted by a signal: not a timeout -- the request is left as it is and the call restarts */
		goto out;
	}

	gna_request_wait_done(score_request);   /* its completion may still be waking us: let it finish first */

	wait_data->out.hw_perf = score_request->hw_perf;
	wait_data->out.drv_perf = score_request->drv_perf;
	wait_data->out.hw_status = score_request->hw_status;

	ret = score_request->status;

	dev_dbg(gna_dev(gna_priv), "request status: %d, hw status: %#x\n",
		score_request->status, score_request->hw_status);

	if (cancel_work_sync(&score_request->work))   /* not after DONE, but a cancelled work's reference is ours */
		kref_put(&score_request->refcount, gna_request_release);
	mutex_lock(&gna_priv->reqlist_lock);
	if (!list_empty(&score_request->node)) {
		list_del_init(&score_request->node);
		kref_put(&score_request->refcount, gna_request_release); // due to gna_priv->request_list removal!
	}
	mutex_unlock(&gna_priv->reqlist_lock);

out:
	kref_put(&score_request->refcount, gna_request_release);
	return ret;
}

int gna_gem_free_ioctl(struct drm_device *dev, void *data,
		struct drm_file *file)
{
	struct gna_gem_free *args = data;
	struct drm_gem_object *drmgemo;
	int ret;

	drmgemo = drm_gem_object_lookup(file, args->handle);
	if (!drmgemo)
		return -ENOENT;

	gna_gem_free_requests(to_gna_device(dev), file, args->handle, drmgemo);
	ret = drm_gem_handle_delete(file, args->handle);

	drm_gem_object_put(drmgemo);
	return ret;
}

int gna_getparam_ioctl(struct drm_device *dev, void *data,
		struct drm_file *file)
{
	struct gna_device *gna_priv = to_gna_device(dev);
	union gna_parameter *param = data;

	return gna_getparam(gna_priv, param);
}

int gna_gem_new_ioctl(struct drm_device *dev, void *data,
		struct drm_file *file)
{
	struct drm_gem_shmem_object *drmgemshm;
	union gna_gem_new *args = data;
	u32 handle;
	int ret;

	drmgemshm = drm_gem_shmem_create(dev, args->in.size);
	if (IS_ERR(drmgemshm))
		return PTR_ERR(drmgemshm);

	/*
	 * Everything is read from the object before its handle exists: another thread of the client can close the
	 * handle, and so free the object, the moment it is published.
	 */
	args->out.size_granted = drmgemshm->base.size;
	args->out.vma_fake_offset = drm_vma_node_offset_addr(&drmgemshm->base.vma_node);

	ret = drm_gem_handle_create(file, &drmgemshm->base, &handle);
	drm_gem_object_put(&drmgemshm->base);   /* the handle holds it now */
	if (ret)
		return ret;

	args->out.handle = handle;
	return 0;
}
