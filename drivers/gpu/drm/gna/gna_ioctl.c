// SPDX-License-Identifier: GPL-2.0-only
// Copyright(c) 2017-2022 Intel Corporation

#include <drm/drm_device.h>
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
	int ret;

	ret = gna_validate_score_config(&score_args->in.config, to_gna_device(dev));
	if (ret)
		return ret;

	ret = gna_enqueue_request(&score_args->in.config, file, &request_id);
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

	ret = wait_event_interruptible_timeout(score_request->waitq, score_request->state == DONE,
					       msecs_to_jiffies(timeout));
	if (ret == 0 || ret == -ERESTARTSYS) {
		dev_dbg(gna_dev(gna_priv), "request timed out, id: %llu\n", request_id);
		ret = -EBUSY;
		goto out;
	}

	wait_data->out.hw_perf = score_request->hw_perf;
	wait_data->out.drv_perf = score_request->drv_perf;
	wait_data->out.hw_status = score_request->hw_status;

	ret = score_request->status;

	dev_dbg(gna_dev(gna_priv), "request status: %d, hw status: %#x\n",
		score_request->status, score_request->hw_status);

	cancel_work_sync(&score_request->work);
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
	struct gna_device *gna_priv = to_gna_device(dev);
	struct gna_gem_free *args = data;
	struct gna_gem_object *gnagemo;
	struct drm_gem_object *drmgemo;
	int ret;

	drmgemo = drm_gem_object_lookup(file, args->handle);
	if (!drmgemo)
		return -ENOENT;

	gnagemo = to_gna_gem_obj(to_drm_gem_shmem_obj(drmgemo));

	queue_work(gna_priv->request_wq, &gnagemo->work);
	if (wait_event_interruptible(gnagemo->waitq, true)) {
		ret = -ERESTARTSYS;
		goto out;
	}

	cancel_work_sync(&gnagemo->work);

	ret = drm_gem_handle_delete(file, args->handle);

out:
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

static struct drm_gem_shmem_object *
drm_gem_shmem_create_with_handle(struct drm_file *file_priv,
				struct drm_device *dev, size_t size,
				uint32_t *handle)
{
	struct drm_gem_shmem_object *shmem;
	int ret;

	shmem = drm_gem_shmem_create(dev, size);
	if (IS_ERR(shmem))
		return shmem;

	/*
	 * Allocate an id of idr table where the obj is registered
	 * and handle has the id what user can see.
	 */
	ret = drm_gem_handle_create(file_priv, &shmem->base, handle);
	/* drop reference from allocate - handle holds it now. */
	drm_gem_object_put(&shmem->base);
	if (ret)
		return ERR_PTR(ret);

	return shmem;
}

int gna_gem_new_ioctl(struct drm_device *dev, void *data,
		struct drm_file *file)
{
	struct drm_gem_shmem_object *drmgemshm;
	struct gna_gem_object *gnagemo;
	union gna_gem_new *args = data;

	drmgemshm = drm_gem_shmem_create_with_handle(file, dev, args->in.size,
						&args->out.handle);

	if (IS_ERR(drmgemshm))
		return PTR_ERR(drmgemshm);

	args->out.size_granted = drmgemshm->base.size;
	args->out.vma_fake_offset = drm_vma_node_offset_addr(&drmgemshm->base.vma_node);

	gnagemo = to_gna_gem_obj(drmgemshm);
	gnagemo->handle = args->out.handle;

	INIT_WORK(&gnagemo->work, gna_gem_obj_release_work);
	init_waitqueue_head(&gnagemo->waitq);

	return 0;
}
