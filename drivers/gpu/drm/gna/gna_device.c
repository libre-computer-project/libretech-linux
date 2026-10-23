// SPDX-License-Identifier: GPL-2.0-only
// Copyright(c) 2017-2022 Intel Corporation

#include <drm/drm_drv.h>
#include <drm/drm_file.h>
#include <drm/drm_gem.h>
#include <drm/drm_ioctl.h>
#include <drm/drm_managed.h>

#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include <uapi/drm/gna_drm.h>

#include "gna_device.h"
#include "gna_gem.h"
#include "gna_request.h"

#define GNA_DDI_VERSION_CURRENT GNA_DDI_VERSION_3

DEFINE_DRM_GEM_FOPS(gna_drm_fops);

static const struct drm_ioctl_desc gna_drm_ioctls[] = {
	DRM_IOCTL_DEF_DRV(GNA_GET_PARAMETER, gna_getparam_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(GNA_GEM_NEW, gna_gem_new_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(GNA_GEM_FREE, gna_gem_free_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(GNA_COMPUTE, gna_score_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(GNA_WAIT, gna_wait_ioctl, DRM_RENDER_ALLOW),
};

static int __maybe_unused gna_runtime_suspend(struct device *dev)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);
	struct gna_device *gna_priv = to_gna_device(drm_dev);
	u32 val = gna_reg_read(gna_priv, GNA_MMIO_D0I3C);

	dev_dbg(dev, "%s D0I3, reg %.8x\n", __func__, val);

	return 0;
}

static int __maybe_unused gna_runtime_resume(struct device *dev)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);
	struct gna_device *gna_priv = to_gna_device(drm_dev);
	u32 val = gna_reg_read(gna_priv, GNA_MMIO_D0I3C);

	dev_dbg(dev, "%s D0I3, reg %.8x\n", __func__, val);

	return 0;
}

const struct dev_pm_ops __maybe_unused gna_pm = {
	SET_RUNTIME_PM_OPS(gna_runtime_suspend, gna_runtime_resume, NULL)
};


static void gna_drm_dev_fini(struct drm_device *dev, void *ptr)
{
	drm_dev_unregister(dev);
}

static int gna_drm_dev_init(struct drm_device *dev)
{
	int err;

	err = drm_dev_register(dev, 0);
	if (err)
		return err;

	return drmm_add_action_or_reset(dev, gna_drm_dev_fini, NULL);
}

static void gna_pm_init(struct device *dev)
{
	pm_runtime_set_autosuspend_delay(dev, 2000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_mark_last_busy(dev);
	pm_runtime_allow(dev);
	pm_runtime_put_noidle(dev);
}

static void gna_pm_fini(struct drm_device *drm, void *data)
{
	struct device *dev = data;

	pm_runtime_get_noresume(dev);
}

static irqreturn_t gna_interrupt(int irq, void *priv)
{
	struct gna_device *gna_priv;

	gna_priv = (struct gna_device *)priv;
	gna_priv->dev_busy = false;
	wake_up(&gna_priv->dev_busy_waitq);
	return IRQ_HANDLED;
}

static void gna_workqueue_fini(struct drm_device *drm, void *data)
{
	struct workqueue_struct *request_wq = data;

	destroy_workqueue(request_wq);
}

static int gna_workqueue_init(struct gna_device *gna_priv)
{
	const char *name = gna_name(gna_priv);

	gna_priv->request_wq = create_singlethread_workqueue(name);
	if (!gna_priv->request_wq)
		return -EFAULT;

	return drmm_add_action_or_reset(&gna_priv->drm, gna_workqueue_fini, gna_priv->request_wq);
}

static struct drm_gem_object *gna_create_gem_object(struct drm_device *dev,
						size_t size)
{
	struct drm_gem_shmem_object *dshmem;
	struct gna_gem_object *shmem;

	shmem = kzalloc(sizeof(*shmem), GFP_KERNEL);
	if (!shmem)
		return NULL;

	dshmem = &shmem->base;

	return &dshmem->base;
}

static const struct drm_driver gna_drm_driver = {
	.driver_features = DRIVER_GEM | DRIVER_RENDER,
	.gem_create_object = gna_create_gem_object,

	.ioctls = gna_drm_ioctls,
	.num_ioctls = ARRAY_SIZE(gna_drm_ioctls),
	.fops = &gna_drm_fops,

	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	/* .date was removed from struct drm_driver in 6.18 (commit
	 * dropping drm_driver::date); DRIVER_DATE no longer wired here.
	 */
	.major = DRIVER_MAJOR,
	.minor = DRIVER_MINOR,
	.patchlevel = DRIVER_PATCHLEVEL,
};

int gna_probe(struct device *parent, struct gna_dev_info *dev_info, void __iomem *iobase, int irq)
{
	struct gna_device *gna_priv;
	struct drm_device *drm_dev;
	u32 bld_reg;
	int err;

	gna_priv = devm_drm_dev_alloc(parent, &gna_drm_driver, struct gna_device, drm);
	if (IS_ERR(gna_priv))
		return PTR_ERR(gna_priv);

	drm_dev = &gna_priv->drm;
	gna_priv->recovery_timeout_jiffies = msecs_to_jiffies(60*1000);
	gna_priv->iobase = iobase;
	gna_priv->info = *dev_info;

	atomic_set(&gna_priv->enqueued_requests, 0);

	if (!(sizeof(dma_addr_t) > 4) ||
		dma_set_mask(parent, DMA_BIT_MASK(64))) {
		err = dma_set_mask(parent, DMA_BIT_MASK(32));
		if (err)
			return err;
	}

	bld_reg = gna_reg_read(gna_priv, GNA_MMIO_IBUFFS);
	gna_priv->hw_info.in_buf_s = bld_reg & GENMASK(7, 0);

	err = gna_mmu_init(gna_priv);
	if (err)
		return err;

	dev_dbg(parent, "maximum memory size %llu num pd %d\n",
		gna_priv->info.max_hw_mem, gna_priv->info.num_pagetables);
	dev_dbg(parent, "desc rsvd size %d mmu vamax size %d\n",
		gna_priv->info.desc_info.rsvd_size,
		gna_priv->info.desc_info.mmu_info.vamax_size);

	mutex_init(&gna_priv->mmu_lock);

	atomic_set(&gna_priv->request_count, 0);

	mutex_init(&gna_priv->reqlist_lock);
	INIT_LIST_HEAD(&gna_priv->request_list);

	init_waitqueue_head(&gna_priv->dev_busy_waitq);

	err = gna_workqueue_init(gna_priv);
	if (err)
		return err;

	err = devm_request_irq(parent, irq, gna_interrupt,
			IRQF_SHARED, gna_name(gna_priv), gna_priv);
	if (err)
		return err;

	dev_set_drvdata(parent, drm_dev);

	err = gna_drm_dev_init(drm_dev);
	if (err)
		return err;

	gna_pm_init(parent);
	err = drmm_add_action(drm_dev, gna_pm_fini, parent);
	if (err)
		return err;

	return 0;
}

static u32 gna_device_type_by_hwid(u32 hwid)
{
	switch (hwid) {
	case GNA_DEV_HWID_CNL:
		return GNA_DEV_TYPE_0_9;
	case GNA_DEV_HWID_GLK:
	case GNA_DEV_HWID_EHL:
	case GNA_DEV_HWID_ICL:
		return GNA_DEV_TYPE_1_0;
	case GNA_DEV_HWID_JSL:
	case GNA_DEV_HWID_TGL:
	case GNA_DEV_HWID_RKL:
		return GNA_DEV_TYPE_2_0;
	case GNA_DEV_HWID_ADL:
	case GNA_DEV_HWID_RPL:
		return GNA_DEV_TYPE_3_0;
	case GNA_DEV_HWID_MTL:
		return GNA_DEV_TYPE_3_5;
	default:
		return 0;
	}
}

int gna_getparam(struct gna_device *gna_priv, union gna_parameter *param)
{
	switch (param->in.id) {
	case GNA_PARAM_RECOVERY_TIMEOUT:
		param->out.value = jiffies_to_msecs(gna_priv->recovery_timeout_jiffies) / 1000;
		break;
	case GNA_PARAM_INPUT_BUFFER_S:
		param->out.value = gna_priv->hw_info.in_buf_s;
		break;
	case GNA_PARAM_DEVICE_TYPE:
		param->out.value = gna_device_type_by_hwid(gna_priv->info.hwid);
		break;
	case GNA_PARAM_DDI_VERSION:
		param->out.value = GNA_DDI_VERSION_CURRENT;
		break;
	default:
		dev_dbg(gna_dev(gna_priv), "unknown parameter id: %llu\n", param->in.id);
		return -EINVAL;
	}

	return 0;
}

MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("Intel(R) Gaussian & Neural Accelerator (Intel(R) GNA) Driver");
MODULE_LICENSE("GPL");
