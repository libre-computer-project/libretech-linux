/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright(c) 2017-2022 Intel Corporation */

#ifndef __GNA_GEM_H__
#define __GNA_GEM_H__

#include <drm/drm_gem_shmem_helper.h>

#include <linux/wait.h>
#include <linux/workqueue.h>

struct gna_gem_object {
	struct drm_gem_shmem_object base;

	uint32_t handle;

	struct work_struct work;

	struct wait_queue_head waitq;
};

#endif /* __GNA_GEM_H__ */
