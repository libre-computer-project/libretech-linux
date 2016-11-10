/*
 * Copyright (C) 2016 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 * Copyright (C) 2014 Endless Mobile
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 * 02111-1307, USA.
 *
 * Written by:
 *     Jasper St. Pierre <jstpierre@mecheye.net>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <drm/drmP.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_flip_work.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_rect.h>
#include <drm/drm_fb_helper.h>

#include "meson_drv.h"
#include "meson_cvbs.h"
#include "meson_plane.h"
#include "meson_crtc.h"

#include "meson_vpu.h"
#include "meson_vpp.h"

#define DRIVER_NAME "meson"
#define DRIVER_DESC "Amlogic Meson DRM driver"

/* DRM Driver */

static void meson_fb_output_poll_changed(struct drm_device *dev)
{
	struct meson_drm *priv = dev->dev_private;

	drm_fbdev_cma_hotplug_event(priv->fbdev);
}

static const struct drm_mode_config_funcs meson_mode_config_funcs = {
	.output_poll_changed = meson_fb_output_poll_changed,
	.atomic_check        = drm_atomic_helper_check,
	.atomic_commit       = drm_atomic_helper_commit,
	.fb_create           = drm_fb_cma_create,
};

#if 0
static void write_scaling_filter_coefs(const unsigned int *coefs,
				       bool is_horizontal)
{
	int i;

	/*aml_write_reg32(P_VPP_OSD_SCALE_COEF_IDX, (is_horizontal ? 1 : 0) << 8);
	for (i = 0; i < 33; i++)
		aml_write_reg32(P_VPP_OSD_SCALE_COEF, coefs[i]);
		*/
}

static unsigned int vpp_filter_coefs_4point_bspline[] = {
    0x15561500, 0x14561600, 0x13561700, 0x12561800,
    0x11551a00, 0x11541b00, 0x10541c00, 0x0f541d00,
    0x0f531e00, 0x0e531f00, 0x0d522100, 0x0c522200,
    0x0b522300, 0x0b512400, 0x0a502600, 0x0a4f2700,
    0x094e2900, 0x084e2a00, 0x084d2b00, 0x074c2c01,
    0x074b2d01, 0x064a2f01, 0x06493001, 0x05483201,
    0x05473301, 0x05463401, 0x04453601, 0x04433702,
    0x04423802, 0x03413a02, 0x03403b02, 0x033f3c02,
    0x033d3d03
};

/* Configure the VPP to act like how we expect it to. Other drivers,
 * like the ones included in U-Boot, might turn on weird features
 * like the HW scaler or special planes. Reset the VPP to a sane mode
 * that expects like we behave.
 */
static void meson_reset_vpp(void)
{
	/* Turn off the HW scalers -- U-Boot turns these on and we
	 * need to clear them to make things work. */
	aml_clr_reg32_mask(P_VPP_OSD_SC_CTRL0, 1 << 3);
	aml_clr_reg32_mask(P_VPP_OSD_VSC_CTRL0, 1 << 24);
	aml_clr_reg32_mask(P_VPP_OSD_HSC_CTRL0, 1 << 22);

	BUILD_BUG_ON(ARRAY_SIZE(vpp_filter_coefs_4point_bspline) != 33);
	/* Write in the proper filter coefficients. */
	write_scaling_filter_coefs(vpp_filter_coefs_4point_bspline, 0);
	write_scaling_filter_coefs(vpp_filter_coefs_4point_bspline, 1);

	/* Force all planes off -- U-Boot might configure them and
	 * we shouldn't have any stale planes. */
	aml_clr_reg32_mask(P_VPP_MISC, VPP_OSD1_POSTBLEND | VPP_OSD2_POSTBLEND);
	aml_clr_reg32_mask(P_VPP_MISC, VPP_VD1_POSTBLEND | VPP_VD2_POSTBLEND);

	/* Turn on POSTBLEND. */
	aml_set_reg32_mask(P_VPP_MISC, VPP_POSTBLEND_EN);

	/* Put OSD2 (cursor) on top of OSD1. */
	aml_set_reg32_mask(P_VPP_MISC, VPP_POST_FG_OSD2 | VPP_PRE_FG_OSD2);

	/* In its default configuration, the display controller can be starved
	 * of memory bandwidth when the CPU and GPU are busy, causing scanout
	 * to sometimes get behind where it should be (with parts of the
	 * display appearing momentarily shifted to the right).
	 * Increase the priority and burst size of RAM access using the same
	 * values as Amlogic's driver. */
	aml_set_reg32_mask(P_VIU_OSD1_FIFO_CTRL_STAT,
			   1 << 0 | /* Urgent DDR request priority */
			   3 << 10 /* Increase burst length from 24 to 64 */
			   );
	aml_set_reg32_mask(P_VIU_OSD2_FIFO_CTRL_STAT,
			   1 << 0 | /* Urgent DDR request priority */
			   3 << 10 /* Increase burst length from 24 to 64 */
			   );

	/* Increase the number of lines that the display controller waits
	 * after vsync before starting RAM access. This gives the vsync
	 * interrupt handler more time to update the registers, avoiding
	 * visual glitches. */
	aml_set_reg32_bits(P_VIU_OSD1_FIFO_CTRL_STAT, 12, 5, 5);
	aml_set_reg32_bits(P_VIU_OSD2_FIFO_CTRL_STAT, 12, 5, 5);
}
#endif

static int meson_enable_vblank(struct drm_device *dev, unsigned int crtc)
{
	/* TODO */

	return 0;
}

static void meson_disable_vblank(struct drm_device *dev, unsigned int crtc)
{
	/* TODO */
}

static irqreturn_t meson_irq(int irq, void *arg)
{
	struct drm_device *dev = arg;
	struct meson_drm *priv = dev->dev_private;

	meson_crtc_irq(priv);

	return IRQ_HANDLED;
}

static const struct file_operations fops = {
	.owner		= THIS_MODULE,
	.open		= drm_open,
	.release	= drm_release,
	.unlocked_ioctl	= drm_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= drm_compat_ioctl,
#endif
	.poll		= drm_poll,
	.read		= drm_read,
	.llseek		= no_llseek,
	.mmap		= drm_gem_cma_mmap,
};

static struct drm_driver meson_driver = {
	.driver_features	= DRIVER_HAVE_IRQ | DRIVER_GEM |
			      	  DRIVER_MODESET | DRIVER_PRIME |
				  DRIVER_ATOMIC,

	/* Vblank */
	.enable_vblank		= meson_enable_vblank,
	.disable_vblank		= meson_disable_vblank,
	.get_vblank_counter	= drm_vblank_no_hw_counter,

	/* IRQ */
	.irq_handler		= meson_irq,

	/* PRIME Ops */
	.prime_handle_to_fd	= drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle	= drm_gem_prime_fd_to_handle,
	.gem_prime_import	= drm_gem_prime_import,
	.gem_prime_export	= drm_gem_prime_export,
	.gem_prime_get_sg_table	= drm_gem_cma_prime_get_sg_table,
	.gem_prime_import_sg_table = drm_gem_cma_prime_import_sg_table,
	.gem_prime_vmap		= drm_gem_cma_prime_vmap,
	.gem_prime_vunmap	= drm_gem_cma_prime_vunmap,
	.gem_prime_mmap		= drm_gem_cma_prime_mmap,

	/* GEM Ops */
	.dumb_create		= drm_gem_cma_dumb_create,
	.dumb_destroy		= drm_gem_dumb_destroy,
	.dumb_map_offset	= drm_gem_cma_dumb_map_offset,
	.gem_free_object_unlocked = drm_gem_cma_free_object,
	.gem_vm_ops		= &drm_gem_cma_vm_ops,

	/* Misc */
	.fops			= &fops,
	.name			= DRIVER_NAME,
	.desc			= DRIVER_DESC,
	.date			= "20161109",
	.major			= 1,
	.minor			= 0,
};

static int meson_pdev_probe(struct platform_device *pdev)
{
	struct meson_drm *priv;
	struct drm_device *drm;
	int ret;

	drm = drm_dev_alloc(&meson_driver, &pdev->dev);
	if (IS_ERR(drm))
		return PTR_ERR(drm);

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		goto free_drm;
	}
	drm->dev_private = priv;
	priv->drm = drm;

	priv->vsync_irq = platform_get_irq(pdev, 0);

	ret = drm_irq_install(drm, priv->vsync_irq);
	if (ret)
		goto free_drm;

	drm_vblank_init(drm, 1);
	drm_mode_config_init(drm);

	ret = meson_plane_create(priv);
	if (ret)
		goto free_drm;

	ret = meson_crtc_create(priv);
	if (ret)
		goto free_drm;

	ret = meson_cvbs_create(priv);
	if (ret)
		goto free_drm;

	drm_mode_config_reset(drm);
	drm->mode_config.max_width = 8192;
	drm->mode_config.max_height = 8192;
	drm->mode_config.funcs = &meson_mode_config_funcs;

	priv->fbdev = drm_fbdev_cma_init(drm, 32,
					 drm->mode_config.num_crtc,
					 drm->mode_config.num_connector);
	if (IS_ERR(priv->fbdev)) {
		ret = PTR_ERR(priv->fbdev);
		goto free_drm;
	}

	drm_kms_helper_poll_init(drm);

	meson_vpu_init(priv);
	meson_vpp_reset(priv);

	ret = drm_dev_register(drm, 0);
	if (ret)
		goto free_drm;

	platform_set_drvdata(pdev, priv);

	return 0;

free_drm:
	drm_dev_unref(drm);
	return ret;
}

static int meson_pdev_remove(struct platform_device *pdev)
{
	struct meson_drm *priv = platform_get_drvdata(pdev);
	struct drm_device *drm = priv->drm;

	drm_dev_unregister(drm);
	drm_kms_helper_poll_fini(drm);
	drm_fbdev_cma_fini(priv->fbdev);
	drm_mode_config_cleanup(drm);
	drm_vblank_cleanup(drm);
	drm_dev_unref(drm);

	return 0;
}

static const struct of_device_id dt_match[] = {
	{ .compatible = "amlogic,meson-gxbb-display" },
	{}
};
MODULE_DEVICE_TABLE(of, dt_match);

static struct platform_driver meson_drm_platform_driver = {
	.probe      = meson_pdev_probe,
	.remove     = meson_pdev_remove,
	.driver     = {
		.owner  = THIS_MODULE,
		.name   = DRIVER_NAME,
		.of_match_table = dt_match,
	},
};

module_platform_driver(meson_drm_platform_driver);

MODULE_AUTHOR("Jasper St. Pierre <jstpierre@mecheye.net>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
