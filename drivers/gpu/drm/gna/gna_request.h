/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright(c) 2017-2022 Intel Corporation */

#ifndef __GNA_REQUEST_H__
#define __GNA_REQUEST_H__

#include <linux/kref.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <uapi/drm/gna_drm.h>

struct gna_device;
struct gna_gem_object;
struct drm_file;

enum gna_request_state {
	NEW,
	ACTIVE,
	DONE,
};

struct gna_buffer_with_object {
	struct gna_buffer gna;
	struct gna_gem_object *gem;
};

struct gna_request {
	u64 request_id;

	struct kref refcount;

	struct drm_file *drm_f;

	u32 hw_status;

	enum gna_request_state state;

	int status;

	struct gna_hw_perf hw_perf;
	struct gna_drv_perf drv_perf;

	struct list_head node;

	struct gna_compute_cfg compute_cfg;

	struct gna_buffer_with_object *buffer_list;
	u64 buffer_count;

	struct work_struct work;
	struct wait_queue_head waitq;
};

int gna_validate_score_config(struct gna_compute_cfg *compute_cfg,
			struct gna_device *gna_priv);

int gna_enqueue_request(struct gna_compute_cfg *compute_cfg,
			struct drm_file *file, u64 *request_id);

void gna_request_release(struct kref *ref);

#endif // __GNA_REQUEST_H__
