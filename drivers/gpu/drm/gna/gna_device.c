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


/* undone by drm_dev_unplug() in gna_remove(), not at the final put: a client can keep its file open past the device */
static int gna_drm_dev_init(struct drm_device *dev)
{
	return drm_dev_register(dev, 0);
}

static void gna_pm_init(struct device *dev)
{
	pm_runtime_set_autosuspend_delay(dev, 2000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_mark_last_busy(dev);
	pm_runtime_allow(dev);
	pm_runtime_put_noidle(dev);
}

/*
 * Undo gna_pm_init() from the bus driver's .remove. It must run there: pci_device_remove() drops the PCI core's
 * runtime-PM reference (pm_runtime_put_sync) before devres and drmm cleanup, so balancing from a drmm action (as
 * this driver did) came too late and underflowed the usage count on every unbind ("Runtime PM usage count
 * underflow!"). pm_runtime_forbid() gives back the reference pm_runtime_allow() took over from pci_pm_init();
 * pm_runtime_get_noresume() gives back the one probe dropped.
 */
void gna_remove(struct device *parent)
{
	struct drm_device *drm_dev = dev_get_drvdata(parent);
	struct gna_device *gna_priv = to_gna_device(drm_dev);

	/*
	 * Devres unmaps the registers, frees the interrupt, the descriptor and the page tables once this returns, while a
	 * client may still hold its file open. So: refuse every new ioctl and wait out a COMPUTE in flight (it may start
	 * the hardware), run what is queued while the registers are still mapped, and finish what is on the hardware.
	 * After that nothing reaches the hardware: direct_req stays NULL, and nothing can be queued again.
	 */
	drm_dev_unplug(drm_dev);
	flush_workqueue(gna_priv->request_wq);
	gna_hw_acquire(gna_priv);             /* no request on the hardware (a hung one after the recovery timeout) */
	gna_mmu_release(gna_priv);            /* objects the page tables were kept for */
	gna_hw_release(gna_priv);
	pm_runtime_dont_use_autosuspend(parent);
	pm_runtime_forbid(parent);
	pm_runtime_get_noresume(parent);
}

static irqreturn_t gna_interrupt(int irq, void *priv)
{
	struct gna_device *gna_priv;
	struct gna_request *direct, *req;
	unsigned long flags;
	u32 sts;

	gna_priv = (struct gna_device *)priv;

	/*
	 * Ours only when a request is on the hardware -- started from COMPUTE (completed here, with no thread woken:
	 * gna_request_complete_direct) or by the worker (woken) -- with its START written (hw_started, set with START
	 * under this lock), and STS reports a completion or an error. The line may be shared: before START STS still
	 * carries the previous request's completion, all ones means the device is suspended, and an idle device has
	 * nothing to complete.
	 */
	spin_lock_irqsave(&gna_priv->direct_lock, flags);
	direct = gna_priv->direct_req;
	req = direct ? direct : gna_priv->queued_req;
	if (!req || !req->hw_started) {
		spin_unlock_irqrestore(&gna_priv->direct_lock, flags);
		return IRQ_NONE;
	}
	sts = gna_reg_read(gna_priv, GNA_MMIO_STS);
	if (sts == ~0U || !(sts & (GNA_STS_SCORE_COMPLETED | GNA_ERROR))) {
		spin_unlock_irqrestore(&gna_priv->direct_lock, flags);
		return IRQ_NONE;
	}
	if (direct)
		gna_priv->direct_req = NULL;
	spin_unlock_irqrestore(&gna_priv->direct_lock, flags);
	if (direct) {
		gna_priv->dev_busy = false;
		gna_request_complete_direct(direct, false);
		return IRQ_HANDLED;
	}

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
		return ERR_PTR(-ENOMEM);   /* the shmem helper tests IS_ERR() only: NULL would be used as the object */

	dshmem = &shmem->base;

	return &dshmem->base;
}

static void gna_postclose(struct drm_device *dev, struct drm_file *file)
{
	gna_request_file_close(to_gna_device(dev), file);
}

static const struct drm_driver gna_drm_driver = {
	.driver_features = DRIVER_GEM | DRIVER_RENDER,
	.gem_create_object = gna_create_gem_object,
	.postclose = gna_postclose,

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

	/*
	 * The descriptor base, the page directory and the page tables hold 32-bit page frame numbers (addr >> PAGE_SHIFT,
	 * gna_mmu_set / gna_mmu_add_locked / gna_score): the device reaches 44 bits of address, not 64. With a 64-bit mask
	 * an IOMMU could hand out an IOVA above 2^44 (its 32-bit space used up, or iommu.forcedac) and the tables would
	 * hold it truncated -- DMA to the wrong address.
	 */
	if (!(sizeof(dma_addr_t) > 4) ||
		dma_set_mask(parent, DMA_BIT_MASK(32 + PAGE_SHIFT))) {
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

	atomic64_set(&gna_priv->request_count, 0);

	mutex_init(&gna_priv->reqlist_lock);
	INIT_LIST_HEAD(&gna_priv->request_list);

	init_waitqueue_head(&gna_priv->dev_busy_waitq);
	sema_init(&gna_priv->hw_sem, 1);
	spin_lock_init(&gna_priv->direct_lock);
	gna_priv->direct_req = NULL;

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
	case GNA_DEV_HWID_ARL:
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
