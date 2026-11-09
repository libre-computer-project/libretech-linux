// SPDX-License-Identifier: GPL-2.0+
/*
 * Amlogic GE2D Kernel API shim
 *
 * Built-in API layer that forwards to the GE2D V4L2 module driver
 * via registered callbacks. This allows built-in consumers (e.g. DRM
 * writeback) to use GE2D without requiring the V4L2/media stack to
 * be built-in.
 *
 * Copyright (C) 2025 Libre Computer
 */

#include <linux/export.h>
#include <linux/mutex.h>
#include <linux/soc/amlogic/meson-ge2d.h>

static const struct meson_ge2d_ops *ge2d_ops;
static DEFINE_MUTEX(ge2d_ops_lock);

int meson_ge2d_register(const struct meson_ge2d_ops *ops)
{
	int ret = 0;

	mutex_lock(&ge2d_ops_lock);
	if (ge2d_ops)
		ret = -EBUSY;
	else
		ge2d_ops = ops;
	mutex_unlock(&ge2d_ops_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(meson_ge2d_register);

void meson_ge2d_unregister(const struct meson_ge2d_ops *ops)
{
	mutex_lock(&ge2d_ops_lock);
	if (ge2d_ops == ops)
		ge2d_ops = NULL;
	mutex_unlock(&ge2d_ops_lock);
}
EXPORT_SYMBOL_GPL(meson_ge2d_unregister);

struct meson_ge2d_dev *meson_ge2d_get(struct device *dev)
{
	struct meson_ge2d_dev *gdev;

	mutex_lock(&ge2d_ops_lock);
	if (!ge2d_ops) {
		mutex_unlock(&ge2d_ops_lock);
		return ERR_PTR(-EPROBE_DEFER);
	}
	gdev = ge2d_ops->get(dev);
	mutex_unlock(&ge2d_ops_lock);

	return gdev;
}
EXPORT_SYMBOL_GPL(meson_ge2d_get);

int meson_ge2d_blit(struct meson_ge2d_dev *ge2d, struct meson_ge2d_job *job)
{
	int ret;

	mutex_lock(&ge2d_ops_lock);
	if (!ge2d_ops) {
		mutex_unlock(&ge2d_ops_lock);
		return -ENODEV;
	}
	ret = ge2d_ops->blit(ge2d, job);
	mutex_unlock(&ge2d_ops_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(meson_ge2d_blit);
