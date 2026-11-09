// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2016 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 * Copyright (C) 2014 Endless Mobile
 *
 * Written by:
 *     Jasper St. Pierre <jstpierre@mecheye.net>
 */

#include <linux/aperture.h>
#include <linux/component.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/seq_file.h>
#include <linux/sys_soc.h>
#include <linux/mfd/syscon.h>
#include <linux/platform_device.h>
#include <linux/soc/amlogic/meson-canvas.h>

#include <drm/clients/drm_client_setup.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_module.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#include "meson_crtc.h"
#include "meson_drv.h"
#include "meson_overlay.h"
#include "meson_overlay2.h"
#include "meson_writeback.h"
#include "meson_plane.h"
#include "meson_plane2.h"
#include "meson_osd_afbcd.h"
#include "meson_registers.h"
#include "meson_encoder_cvbs.h"
#include "meson_encoder_hdmi.h"
#include "meson_encoder_dsi.h"
#include "meson_viu.h"
#include "meson_venc.h"
#include "meson_vpp.h"
#include "meson_rdma.h"

#define DRIVER_NAME "meson"
#define DRIVER_DESC "Amlogic Meson DRM driver"

/**
 * DOC: Video Processing Unit
 *
 * VPU Handles the Global Video Processing, it includes management of the
 * clocks gates, blocks reset lines and power domains.
 *
 * What is missing :
 *
 * - Full reset of entire video processing HW blocks
 * - Scaling and setup of the VPU clock
 * - Bus clock gates
 * - Powering up video processing HW blocks
 * - Powering Up HDMI controller and PHY
 */

static const struct drm_mode_config_funcs meson_mode_config_funcs = {
	.atomic_check        = drm_atomic_helper_check,
	.atomic_commit       = drm_atomic_helper_commit,
	.fb_create           = drm_gem_fb_create,
};

static const struct drm_mode_config_helper_funcs meson_mode_config_helpers = {
	.atomic_commit_tail = drm_atomic_helper_commit_tail_rpm,
};

static irqreturn_t meson_irq(int irq, void *arg)
{
	struct drm_device *dev = arg;
	struct meson_drm *priv = dev->dev_private;

	(void)readl_relaxed(priv->io_base + _REG(VENC_INTFLAG));

	meson_crtc_irq(priv);

	return IRQ_HANDLED;
}

static int meson_dumb_create(struct drm_file *file, struct drm_device *dev,
			     struct drm_mode_create_dumb *args)
{
	/*
	 * We need 64bytes aligned stride, and PAGE aligned size
	 */
	args->pitch = ALIGN(DIV_ROUND_UP(args->width * args->bpp, 8), SZ_64);
	args->size = PAGE_ALIGN(args->pitch * args->height);

	return drm_gem_dma_dumb_create_internal(file, dev, args);
}

/* VENC Test Pattern Generator (debugfs) */

static int meson_venc_tpg_show(struct seq_file *m, void *data)
{
	struct drm_device *dev = m->private;
	struct meson_drm *priv = dev->dev_private;

	seq_printf(m, "%d\n",
		   readl_relaxed(priv->io_base + _REG(VENC_VIDEO_TST_EN)) & 1);
	return 0;
}

/*
 * Audit #25: mirrors meson_crtc_vrr_encp_active() in meson_crtc.c -- ENCP
 * only drives the link in MESON_VENC_MODE_HDMI with !hdmi_use_enci. 480i/576i
 * (DBLCLK) and every interlaced HDMI mode run on ENCI instead, and CVBS/DSI
 * never touch ENCP at all. Reading HAVON_BEGIN/END or writing
 * ENCP_VIDEO_MODE_ADV/VENC_VIDEO_TST_* on those modes touches a dead ENCP
 * (HAVON falls back to the hardcoded 1920 clamp) instead of the encoder that
 * is actually live.
 */
static bool meson_venc_tpg_encp_active(struct meson_drm *priv)
{
	return priv->venc.current_mode == MESON_VENC_MODE_HDMI &&
	       !priv->venc.hdmi_use_enci;
}

static ssize_t meson_venc_tpg_write(struct file *file,
				    const char __user *ubuf,
				    size_t len, loff_t *offp)
{
	struct seq_file *m = file->private_data;
	struct drm_device *dev = m->private;
	struct meson_drm *priv = dev->dev_private;
	int val;

	if (kstrtoint_from_user(ubuf, len, 0, &val))
		return -EFAULT;

	if (val) {
		u32 hb, he, hactive, adv;

		if (!meson_venc_tpg_encp_active(priv))
			return -ENODEV;

		/*
		 * The ENCP test pattern is gated off by ENCP_VIDEO_MODE_ADV bit3
		 * (without clearing it, TST_EN has no visible effect -- the GXL
		 * output stays the normal scanout). Clear the gate, then program the
		 * 8-bar color-bar generator: bar START and width both read live from
		 * the ENCP active window (HAVON_BEGIN .. HAVON_END) so the bars align
		 * to active video at any resolution. TST_CLRBAR_STRT is in the same
		 * VENC H-counter space as HAVON_BEGIN, so the bars must START at
		 * HAVON_BEGIN -- NOT a fixed constant. venc-repeat (pixel-doubled)
		 * modes (480p/576p/720p) double the ENCP H-timing, pushing HAVON_BEGIN
		 * out to ~648 at 720p; a hardcoded start (was 0x113=275) then began the
		 * bars in blanking, clipping the leftmost (white) bar off-screen and
		 * leaving black on the right. Anchoring to HAVON_BEGIN fixes every mode.
		 */
		hb = readl_relaxed(priv->io_base + _REG(ENCP_VIDEO_HAVON_BEGIN));
		he = readl_relaxed(priv->io_base + _REG(ENCP_VIDEO_HAVON_END));
		hactive = (he > hb) ? (he - hb + 1) : 1920;

		/*
		 * Audit #25: save the gate bit as it actually was, so "off"
		 * can restore that exact value instead of unconditionally
		 * force-setting it.
		 */
		adv = readl_relaxed(priv->io_base + _REG(ENCP_VIDEO_MODE_ADV));
		priv->venc.tpg_saved_mode_adv = adv;
		priv->venc.tpg_active = true;

		writel_relaxed(adv & ~BIT(3),
			       priv->io_base + _REG(ENCP_VIDEO_MODE_ADV));
		writel_relaxed(1, priv->io_base + _REG(VENC_VIDEO_TST_MDSEL));
		writel_relaxed(hb, priv->io_base + _REG(VENC_VIDEO_TST_CLRBAR_STRT));
		writel_relaxed(hactive / 8,
			       priv->io_base + _REG(VENC_VIDEO_TST_CLRBAR_WIDTH));
		writel_relaxed(1, priv->io_base + _REG(VENC_VIDEO_TST_EN));
	} else if (priv->venc.tpg_active) {
		writel_relaxed(0, priv->io_base + _REG(VENC_VIDEO_TST_EN));
		/* restore the exact ENCP_VIDEO_MODE_ADV value saved on "on",
		 * not a hardcoded bit3=1 (audit #25). */
		writel_relaxed(priv->venc.tpg_saved_mode_adv,
			       priv->io_base + _REG(ENCP_VIDEO_MODE_ADV));
		priv->venc.tpg_active = false;
	}

	return len;
}

DEFINE_SHOW_STORE_ATTRIBUTE(meson_venc_tpg);

static void meson_debugfs_init(struct drm_minor *minor)
{
	debugfs_create_file("venc_tpg", 0600, minor->debugfs_root,
			    minor->dev, &meson_venc_tpg_fops);
}

DEFINE_DRM_GEM_DMA_FOPS(fops);

static const struct drm_driver meson_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,

	/* DMA Ops */
	DRM_GEM_DMA_DRIVER_OPS_WITH_DUMB_CREATE(meson_dumb_create),
	DRM_FBDEV_DMA_DRIVER_OPS,

	/* Misc */
	.fops			= &fops,
	.debugfs_init		= meson_debugfs_init,
	.name			= DRIVER_NAME,
	.desc			= DRIVER_DESC,
	.major			= 1,
	.minor			= 0,
};

static bool meson_vpu_has_available_connectors(struct device *dev)
{
	struct device_node *ep, *remote;

	/* Parses each endpoint and check if remote exists */
	for_each_endpoint_of_node(dev->of_node, ep) {
		/* If the endpoint node exists, consider it enabled */
		remote = of_graph_get_remote_port(ep);
		if (remote) {
			of_node_put(remote);
			of_node_put(ep);
			return true;
		}
	}

	return false;
}

static const struct regmap_config meson_regmap_config = {
	.reg_bits       = 32,
	.val_bits       = 32,
	.reg_stride     = 4,
	.max_register   = 0x1000,
};

static void meson_vpu_init(struct meson_drm *priv)
{
	u32 value;

	/*
	 * L1C1 read arbiter: keep all slaves on master port 0.
	 * BSP uses 0 here; splitting dc0/dc5 to port 1 can cause
	 * bandwidth starvation at high resolutions.
	 */
	writel_relaxed(0, priv->io_base + _REG(VPU_RDARB_MODE_L1C1));

	/* Slave dc0 connected to master port 1 */
	value = VPU_RDARB_SLAVE_TO_MASTER_PORT(0, 1);
	writel_relaxed(value, priv->io_base + _REG(VPU_RDARB_MODE_L1C2));

	/* Slave dc4 and dc7 connected to master port 1 */
	value = VPU_RDARB_SLAVE_TO_MASTER_PORT(4, 1) |
		VPU_RDARB_SLAVE_TO_MASTER_PORT(7, 1);
	writel_relaxed(value, priv->io_base + _REG(VPU_RDARB_MODE_L2C1));

	/* Slave dc1 connected to master port 1 */
	value = VPU_RDARB_SLAVE_TO_MASTER_PORT(1, 1);
	writel_relaxed(value, priv->io_base + _REG(VPU_WRARB_MODE_L2C1));
}

struct meson_drm_soc_attr {
	struct meson_drm_soc_limits limits;
	const struct soc_device_attribute *attrs;
};

static const struct meson_drm_soc_attr meson_drm_soc_attrs[] = {
	/* S805X/S805Y HDMI PLL won't lock for HDMI PHY freq > 1,65GHz */
	{
		.limits = {
			.max_hdmi_phy_freq = 1650000000,
		},
		.attrs = (const struct soc_device_attribute []) {
			{ .soc_id = "GXL (S805*)", },
			{ /* sentinel */ }
		}
	},
	/*
	 * GXL S905X/S905D/S905W: the full HDMI 2.0 range, 5.94GHz TMDS.
	 *
	 * This used to read 2.97GHz, justified by a garbled (green) VENC test
	 * pattern at 5.94GHz being "simply beyond the GXL TMDS PHY". That was
	 * RETRACTED by b6a37bf9577ee: the green pattern was the dw-hdmi RGB-wire
	 * CSC bug (since fixed by keeping the link in YUV444), not a PHY limit.
	 * The 2.97GHz cap was ~2x below the silicon and pruned every mode needing
	 * a 296..593MHz char rate -- 3440x1440@60/@100, 2560x1440@120 -- which now
	 * run at full 8-bit 4:4:4 (3440x1440@100 = 543.5MHz char, 5435MHz VCO,
	 * loopback-clean).
	 *
	 * Note this ceiling does NOT decide 4K60's chroma. 4K60 is observed to
	 * come out 4:2:0 (measured: hdmi_pll 2970000000 / cts_encp 594000000), but
	 * not because 5940MHz is unreachable -- the PLL synthesizes it as
	 * m=247/frac=512/od=1 (meson_vclk.c even hard-codes that pair for
	 * 5940000000), and this sink advertises 4K60 as 4:2:0-ALSO, not 4:2:0-only.
	 * Why 4:4:4 is not selected there is an OPEN question; do not re-derive it
	 * from this constant.
	 */
	{
		.limits = {
			.max_hdmi_phy_freq = 5940000000, /* TEST: full VCO range (od=1) to map the real PHY ceiling */
		},
		.attrs = (const struct soc_device_attribute []) {
			{ .soc_id = "GXL (S905*)", },
			{ /* sentinel */ }
		}
	},
};

static int meson_drv_bind_master(struct device *dev, bool has_components)
{
	struct platform_device *pdev = to_platform_device(dev);
	const struct meson_drm_match_data *match;
	struct meson_drm *priv;
	struct drm_device *drm;
	struct resource *res;
	void __iomem *regs;
	int ret, i;

	/* Checks if an output connector is available */
	if (!meson_vpu_has_available_connectors(dev)) {
		dev_err(dev, "No output connector available\n");
		return -ENODEV;
	}

	match = of_device_get_match_data(dev);
	if (!match)
		return -ENODEV;

	drm = drm_dev_alloc(&meson_driver, dev);
	if (IS_ERR(drm))
		return PTR_ERR(drm);

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		goto free_drm;
	}
	drm->dev_private = priv;
	priv->drm = drm;
	priv->dev = dev;
	priv->compat = match->compat;
	priv->afbcd.ops = match->afbcd_ops;

	regs = devm_platform_ioremap_resource_byname(pdev, "vpu");
	if (IS_ERR(regs)) {
		ret = PTR_ERR(regs);
		goto free_drm;
	}

	priv->io_base = regs;

	/*
	 * Use the HHI syscon regmap if available (shares lock with clock
	 * driver, preventing PLL register races). Fall back to independent
	 * ioremap for old DTBs without the amlogic,hhi phandle.
	 */
	priv->hhi = syscon_regmap_lookup_by_phandle(dev->of_node,
						    "amlogic,hhi");
	if (IS_ERR(priv->hhi)) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "hhi");
		if (!res) {
			ret = -EINVAL;
			goto free_drm;
		}
		regs = devm_ioremap(dev, res->start, resource_size(res));
		if (!regs) {
			ret = -EADDRNOTAVAIL;
			goto free_drm;
		}
		priv->hhi = devm_regmap_init_mmio(dev, regs,
						  &meson_regmap_config);
		if (IS_ERR(priv->hhi)) {
			dev_err(&pdev->dev, "Couldn't create the HHI regmap\n");
			ret = PTR_ERR(priv->hhi);
			goto free_drm;
		}
	}

	priv->canvas = meson_canvas_get(dev);
	if (IS_ERR(priv->canvas)) {
		ret = PTR_ERR(priv->canvas);
		goto free_drm;
	}

	ret = meson_canvas_alloc(priv->canvas, &priv->canvas_id_osd1);
	if (ret)
		goto free_drm;
	ret = meson_canvas_alloc(priv->canvas, &priv->canvas_id_osd2);
	if (ret)
		goto free_canvas_osd1;
	ret = meson_canvas_alloc(priv->canvas, &priv->canvas_id_vd1_0);
	if (ret)
		goto free_canvas_osd2;
	ret = meson_canvas_alloc(priv->canvas, &priv->canvas_id_vd1_1);
	if (ret)
		goto free_canvas_vd1_0;
	ret = meson_canvas_alloc(priv->canvas, &priv->canvas_id_vd1_2);
	if (ret)
		goto free_canvas_vd1_1;
	ret = meson_canvas_alloc(priv->canvas, &priv->canvas_id_vd2_0);
	if (ret)
		goto free_canvas_vd1_2;
	ret = meson_canvas_alloc(priv->canvas, &priv->canvas_id_vd2_1);
	if (ret)
		goto free_canvas_vd2_0;
	ret = meson_canvas_alloc(priv->canvas, &priv->canvas_id_vd2_2);
	if (ret)
		goto free_canvas_vd2_1;

	priv->vsync_irq = platform_get_irq(pdev, 0);

	ret = drm_vblank_init(drm, 1);
	if (ret)
		goto free_canvas_vd2_2;

	/* Assign limits per soc revision/package */
	for (i = 0 ; i < ARRAY_SIZE(meson_drm_soc_attrs) ; ++i) {
		if (soc_device_match(meson_drm_soc_attrs[i].attrs)) {
			priv->limits = &meson_drm_soc_attrs[i].limits;
			break;
		}
	}

	/*
	 * Remove early framebuffers (ie. simplefb). The framebuffer can be
	 * located anywhere in RAM
	 */
	ret = aperture_remove_all_conflicting_devices(meson_driver.name);
	if (ret)
		goto free_canvas_vd2_2;

	ret = drmm_mode_config_init(drm);
	if (ret)
		goto free_canvas_vd2_2;
	/*
	 * VPU hardware max. Supersedes the older per-SoC gate (3840 on G12A,
	 * 1920 otherwise): the GXL 4K bandwidth work that accompanies this
	 * (OSD FIFO hold_fifo_lines/burst_length, VPP_OFIFO_SIZE_DEFAULT for
	 * all variants, L1C1 read-arbiter port fix) lifts the 1080p ceiling on
	 * the older SoCs, and meson_venc_hdmi_supported_mode() now validates
	 * up to 4096 to match.
	 */
	drm->mode_config.max_width = 4096;
	drm->mode_config.max_height = 4096;
	drm->mode_config.funcs = &meson_mode_config_funcs;
	drm->mode_config.helper_private	= &meson_mode_config_helpers;

	/* Hardware Initialization */

	meson_vpu_init(priv);
	meson_venc_init(priv);
	meson_vpp_init(priv);
	meson_viu_init(priv);
	/*
	 * Audit #10: at cold probe the cache is all-zero (bypass/disabled),
	 * matching hardware reset state, so this is a no-op here -- it exists
	 * so meson_drv_pm_resume() below can share the exact same two calls
	 * instead of duplicating the "what does resume need to redo" logic.
	 */
	meson_viu_apply_ctm(priv);
	meson_viu_apply_gamma(priv);
	if (priv->afbcd.ops) {
		ret = priv->afbcd.ops->init(priv);
		if (ret)
			goto free_canvas_vd2_2;
	}

	/* Encoder Initialization */

	ret = meson_encoder_cvbs_probe(priv);
	if (ret)
		goto exit_afbcd;

	if (has_components) {
		ret = component_bind_all(dev, drm);
		if (ret) {
			dev_err(drm->dev, "Couldn't bind all components\n");
			/* Do not try to unbind */
			has_components = false;
			goto exit_afbcd;
		}
	}

	ret = meson_encoder_hdmi_probe(priv);
	if (ret)
		goto exit_afbcd;

	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A)) {
		ret = meson_encoder_dsi_probe(priv);
		if (ret)
			goto exit_afbcd;
	}

	ret = meson_plane_create(priv);
	if (ret)
		goto exit_afbcd;

	ret = meson_plane2_create(priv);
	if (ret)
		goto exit_afbcd;

	ret = meson_overlay_create(priv);
	if (ret)
		goto exit_afbcd;

	ret = meson_overlay2_create(priv);
	if (ret)
		goto exit_afbcd;

	ret = meson_crtc_create(priv);
	if (ret)
		goto exit_afbcd;

	ret = meson_writeback_create(priv);
	if (ret)
		goto exit_afbcd;

	/*
	 * Audit #24: meson_writeback_create() (meson_writeback.c) already
	 * sets up possible_clones correctly and narrowly -- the writeback
	 * encoder clones with each display encoder sharing its CRTC (and
	 * vice versa, via OR, preserving each encoder's own bits), which is
	 * the only combination the hardware actually supports (writeback
	 * taps the composed VPP output, independent of which VENC mux is
	 * live). This used to be followed by a blanket
	 * "possible_clones = all_mask" on every encoder, overwriting that
	 * setup and making HDMI/CVBS/DSI mutually cloneable -- accepting a
	 * commit that clones HDMI and CVBS onto one CRTC, which need
	 * different VENC muxes and cannot actually coexist. Removed;
	 * meson_writeback_create()'s narrower setup stands alone.
	 */

	ret = request_irq(priv->vsync_irq, meson_irq, 0, drm->driver->name, drm);
	if (ret)
		goto exit_afbcd;

	drm_mode_config_reset(drm);

	drm_kms_helper_poll_init(drm);

	platform_set_drvdata(pdev, priv);

	ret = drm_dev_register(drm, 0);
	if (ret)
		goto uninstall_irq;

	drm_client_setup(drm, NULL);

	return 0;

uninstall_irq:
	free_irq(priv->vsync_irq, drm);
exit_afbcd:
	if (priv->afbcd.ops)
		priv->afbcd.ops->exit(priv);
free_canvas_vd2_2:
	meson_canvas_free(priv->canvas, priv->canvas_id_vd2_2);
free_canvas_vd2_1:
	meson_canvas_free(priv->canvas, priv->canvas_id_vd2_1);
free_canvas_vd2_0:
	meson_canvas_free(priv->canvas, priv->canvas_id_vd2_0);
free_canvas_vd1_2:
	meson_canvas_free(priv->canvas, priv->canvas_id_vd1_2);
free_canvas_vd1_1:
	meson_canvas_free(priv->canvas, priv->canvas_id_vd1_1);
free_canvas_vd1_0:
	meson_canvas_free(priv->canvas, priv->canvas_id_vd1_0);
free_canvas_osd2:
	meson_canvas_free(priv->canvas, priv->canvas_id_osd2);
free_canvas_osd1:
	meson_canvas_free(priv->canvas, priv->canvas_id_osd1);
free_drm:
	drm_dev_put(drm);

	meson_encoder_dsi_remove(priv);
	meson_encoder_hdmi_remove(priv);
	meson_encoder_cvbs_remove(priv);

	if (has_components)
		component_unbind_all(dev, drm);

	return ret;
}

static int meson_drv_bind(struct device *dev)
{
	return meson_drv_bind_master(dev, true);
}

static void meson_drv_unbind(struct device *dev)
{
	struct meson_drm *priv = dev_get_drvdata(dev);
	struct drm_device *drm = priv->drm;

	if (priv->canvas) {
		meson_canvas_free(priv->canvas, priv->canvas_id_osd1);
		meson_canvas_free(priv->canvas, priv->canvas_id_osd2);
		meson_canvas_free(priv->canvas, priv->canvas_id_vd1_0);
		meson_canvas_free(priv->canvas, priv->canvas_id_vd1_1);
		meson_canvas_free(priv->canvas, priv->canvas_id_vd1_2);
		meson_canvas_free(priv->canvas, priv->canvas_id_vd2_0);
		meson_canvas_free(priv->canvas, priv->canvas_id_vd2_1);
		meson_canvas_free(priv->canvas, priv->canvas_id_vd2_2);
	}

	drm_dev_unregister(drm);
	drm_kms_helper_poll_fini(drm);
	drm_atomic_helper_shutdown(drm);
	free_irq(priv->vsync_irq, drm);
	drm_dev_put(drm);

	meson_encoder_dsi_remove(priv);
	meson_encoder_hdmi_remove(priv);
	meson_encoder_cvbs_remove(priv);

	component_unbind_all(dev, drm);

	if (priv->afbcd.ops)
		priv->afbcd.ops->exit(priv);
}

static const struct component_master_ops meson_drv_master_ops = {
	.bind	= meson_drv_bind,
	.unbind	= meson_drv_unbind,
};

static int __maybe_unused meson_drv_pm_suspend(struct device *dev)
{
	struct meson_drm *priv = dev_get_drvdata(dev);

	if (!priv)
		return 0;

	return drm_mode_config_helper_suspend(priv->drm);
}

static int __maybe_unused meson_drv_pm_resume(struct device *dev)
{
	struct meson_drm *priv = dev_get_drvdata(dev);

	if (!priv)
		return 0;

	meson_vpu_init(priv);
	meson_venc_init(priv);
	meson_vpp_init(priv);
	meson_viu_init(priv);
	/*
	 * Audit #10: meson_viu_init() above reprograms the OSD matrix from
	 * scratch (bypass on GXL/GXM, fixed RGB709->YUV709l on G12A+), and
	 * VPU power-domain cycling across suspend/resume resets the L_GAMMA
	 * LUT hardware regardless of what the driver explicitly rewrites.
	 * Neither is reflected back into drm_crtc_state -- duplicated atomic
	 * state doesn't set color_mgmt_changed on its own -- so without this,
	 * DRM and the driver keep reporting the user's last CTM/gamma while
	 * hardware is actually neutral. Re-apply from the same cache #6/#8
	 * use, independent of whether a future commit ever touches the
	 * CTM/gamma properties again.
	 */
	meson_viu_apply_ctm(priv);
	meson_viu_apply_gamma(priv);
	if (priv->afbcd.ops)
		priv->afbcd.ops->init(priv);

	return drm_mode_config_helper_resume(priv->drm);
}

static void meson_drv_shutdown(struct platform_device *pdev)
{
	struct meson_drm *priv = dev_get_drvdata(&pdev->dev);

	if (!priv)
		return;

	drm_kms_helper_poll_fini(priv->drm);
	drm_atomic_helper_shutdown(priv->drm);
}

/*
 * Only devices to use as components
 * TOFIX: get rid of components when we can finally
 * get meson_dx_hdmi to stop using the meson_drm
 * private structure for HHI registers.
 */
static const struct of_device_id components_dev_match[] = {
	{ .compatible = "amlogic,meson-gxbb-dw-hdmi" },
	{ .compatible = "amlogic,meson-gxl-dw-hdmi" },
	{ .compatible = "amlogic,meson-gxm-dw-hdmi" },
	{ .compatible = "amlogic,meson-g12a-dw-hdmi" },
	{}
};

static int meson_drv_probe(struct platform_device *pdev)
{
	struct component_match *match = NULL;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *ep, *remote;
	int count = 0;

	for_each_endpoint_of_node(np, ep) {
		remote = of_graph_get_remote_port_parent(ep);
		if (!remote || !of_device_is_available(remote)) {
			of_node_put(remote);
			continue;
		}

		if (of_match_node(components_dev_match, remote)) {
			component_match_add(&pdev->dev, &match, component_compare_of, remote);

			dev_dbg(&pdev->dev, "parent %pOF remote match add %pOF parent %s\n",
				np, remote, dev_name(&pdev->dev));
		}

		of_node_put(remote);

		++count;
	}

	if (count && !match)
		return meson_drv_bind_master(&pdev->dev, false);

	/* If some endpoints were found, initialize the nodes */
	if (count) {
		dev_info(&pdev->dev, "Queued %d outputs on vpu\n", count);

		return component_master_add_with_match(&pdev->dev,
						       &meson_drv_master_ops,
						       match);
	}

	/* If no output endpoints were available, simply bail out */
	return 0;
};

static void meson_drv_remove(struct platform_device *pdev)
{
	component_master_del(&pdev->dev, &meson_drv_master_ops);
}

static struct meson_drm_match_data meson_drm_gxbb_data = {
	.compat = VPU_COMPATIBLE_GXBB,
};

static struct meson_drm_match_data meson_drm_gxl_data = {
	.compat = VPU_COMPATIBLE_GXL,
};

static struct meson_drm_match_data meson_drm_gxm_data = {
	.compat = VPU_COMPATIBLE_GXM,
	.afbcd_ops = &meson_afbcd_gxm_ops,
};

static struct meson_drm_match_data meson_drm_g12a_data = {
	.compat = VPU_COMPATIBLE_G12A,
	.afbcd_ops = &meson_afbcd_g12a_ops,
};

static const struct of_device_id dt_match[] = {
	{ .compatible = "amlogic,meson-gxbb-vpu",
	  .data       = (void *)&meson_drm_gxbb_data },
	{ .compatible = "amlogic,meson-gxl-vpu",
	  .data       = (void *)&meson_drm_gxl_data },
	{ .compatible = "amlogic,meson-gxm-vpu",
	  .data       = (void *)&meson_drm_gxm_data },
	{ .compatible = "amlogic,meson-g12a-vpu",
	  .data       = (void *)&meson_drm_g12a_data },
	{}
};
MODULE_DEVICE_TABLE(of, dt_match);

static const struct dev_pm_ops meson_drv_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(meson_drv_pm_suspend, meson_drv_pm_resume)
};

static struct platform_driver meson_drm_platform_driver = {
	.probe      = meson_drv_probe,
	.remove     = meson_drv_remove,
	.shutdown   = meson_drv_shutdown,
	.driver     = {
		.name	= "meson-drm",
		.of_match_table = dt_match,
		.pm = &meson_drv_pm_ops,
	},
};

drm_module_platform_driver(meson_drm_platform_driver);

MODULE_AUTHOR("Jasper St. Pierre <jstpierre@mecheye.net>");
MODULE_AUTHOR("Neil Armstrong <narmstrong@baylibre.com>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
