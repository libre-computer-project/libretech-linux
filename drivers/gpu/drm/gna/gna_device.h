/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright(c) 2017-2022 Intel Corporation */

#ifndef __GNA_DEVICE_H__
#define __GNA_DEVICE_H__

#include <drm/drm_device.h>
#include <drm/drm_gem_shmem_helper.h>

#include <linux/atomic.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/types.h>

#include "gna_gem.h"
#include "gna_hw.h"
#include "gna_mem.h"

#define DRIVER_NAME		"gna"
#define DRIVER_DESC		"Intel(R) Gaussian & Neural Accelerator (Intel(R) GNA)"
#define DRIVER_DATE		"20211201"

#define DRIVER_MAJOR		1
#define DRIVER_MINOR		0
#define DRIVER_PATCHLEVEL	0

struct workqueue_struct;
union gna_parameter;
struct drm_file;
struct device;

struct gna_device {
	struct drm_device drm;

	int recovery_timeout_jiffies;

	/* hardware status set by interrupt handler */
	u32 hw_status;

	/* device related resources */
	void __iomem *iobase;
	struct gna_dev_info info;
	struct gna_hw_info hw_info;

	struct gna_mmu_object mmu;
	struct mutex mmu_lock;

	/* if true, then gna device is processing */
	bool dev_busy;
	struct wait_queue_head dev_busy_waitq;

	struct list_head request_list;
	/* protects request_list */
	struct mutex reqlist_lock;
	struct workqueue_struct *request_wq;
	atomic_t request_count;

	/* requests that are in queue to be run +1 for currently processed one */
	atomic_t enqueued_requests;
};

int gna_probe(struct device *parent, struct gna_dev_info *dev_info, void __iomem *iobase, int irq);
int gna_getparam(struct gna_device *gna_priv, union gna_parameter *param);

int gna_getparam_ioctl(struct drm_device *dev, void *data,
		struct drm_file *file);

int gna_gem_new_ioctl(struct drm_device *dev, void *data,
		struct drm_file *file);

int gna_gem_free_ioctl(struct drm_device *dev, void *data,
		struct drm_file *file);

int gna_score_ioctl(struct drm_device *dev, void *data,
		struct drm_file *file);

int gna_wait_ioctl(struct drm_device *dev, void *data,
		struct drm_file *file);

extern const struct dev_pm_ops __maybe_unused gna_pm;

static inline u32 gna_reg_read(struct gna_device *gna_priv, u32 reg)
{
	return readl(gna_priv->iobase + reg);
}

static inline void gna_reg_write(struct gna_device *gna_priv, u32 reg, u32 val)
{
	writel(val, gna_priv->iobase + reg);
}

static inline const char *gna_name(struct gna_device *gna_priv)
{
	return gna_priv->drm.unique;
}

static inline struct device *gna_dev(struct gna_device *gna_priv)
{
	return gna_priv->drm.dev;
}

static inline struct gna_device *to_gna_device(struct drm_device *dev)
{
	return container_of(dev, struct gna_device, drm);
}

static inline struct gna_gem_object *to_gna_gem_obj(struct drm_gem_shmem_object *drm_gem_shmem)
{
	return container_of(drm_gem_shmem, struct gna_gem_object, base);
}

#endif /* __GNA_DEVICE_H__ */
