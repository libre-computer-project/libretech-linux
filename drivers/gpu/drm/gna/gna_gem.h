/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright(c) 2017-2022 Intel Corporation */

#ifndef __GNA_GEM_H__
#define __GNA_GEM_H__

#include <drm/drm_gem_shmem_helper.h>

struct gna_gem_object {
	struct drm_gem_shmem_object base;
};

#endif /* __GNA_GEM_H__ */
