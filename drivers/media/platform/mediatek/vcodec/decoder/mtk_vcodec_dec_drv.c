// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2016 MediaTek Inc.
 * Author: PC Chen <pc.chen@mediatek.com>
 *         Tiffany Lin <tiffany.lin@mediatek.com>
 */

#include <linux/bitfield.h>
#include <linux/dma-mapping.h>
#include <linux/iommu.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <media/v4l2-event.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-device.h>

#include "mtk_vcodec_dec.h"
#include "mtk_vcodec_dec_hw.h"
#include "mtk_vcodec_dec_pm.h"
#include "../common/mtk_vcodec_intr.h"

/*
 * VDEC core interrupt accounting.
 *
 * The decoder IRQ is requested with flags 0 -- it is NOT IRQF_SHARED -- so
 * every interrupt delivered here is ours, and a return that leaves MISC+0xa4
 * uncleared leaves the level line asserted. It then re-fires until
 * mtk_vcodec_dec_disable_hardware() masks it, and is delivered again at the
 * next decode's enable_irq(), which runs BEFORE the context is published (see
 * the curr_ctx test below).
 *
 * The four counters below partition every vector -- total == noctx +
 * inactive + nosucc + done -- and are advanced unconditionally, so the
 * frequency of each path is a measurement rather than an inference.
 */
static int vdec_irq_total;
module_param(vdec_irq_total, int, 0644);
MODULE_PARM_DESC(vdec_irq_total,
		 "VDEC core interrupts delivered (denominator for the per-path counters)");

static int vdec_irq_nosucc;
module_param(vdec_irq_nosucc, int, 0644);
MODULE_PARM_DESC(vdec_irq_nosucc,
		 "interrupts seen with DEC_SUCCESS clear -- acknowledged and dropped without a wake");

static int vdec_irq_inactive;
module_param(vdec_irq_inactive, int, 0644);
MODULE_PARM_DESC(vdec_irq_inactive,
		 "interrupts seen with the HW-active gate reading inactive (NOT acknowledged -- the block is gated)");

static int vdec_irq_done;
module_param(vdec_irq_done, int, 0644);
MODULE_PARM_DESC(vdec_irq_done,
		 "interrupts taking the DEC_SUCCESS path (acknowledged, context woken)");

static int vdec_irq_noctx;
module_param(vdec_irq_noctx, int, 0644);
MODULE_PARM_DESC(vdec_irq_noctx,
		 "interrupts delivered with no decode in flight (curr_ctx == NULL), acknowledged and dropped");

/*
 * N45 (2026-08-20) -- bitstream visibility for the four MT8365 direct-register
 * backends, and the audit that makes its exposure observable.
 *
 * Those backends used to open their decode with
 *
 *	dma_sync_single_for_device(&ctx->dev->plat_dev->dev,
 *				   bs->dma_addr, bs->size, DMA_TO_DEVICE);
 *
 * on the theory that the client's write might still be sitting in the CPU
 * cache. That call is an ILLEGAL use of the STREAMING DMA API on this handle,
 * and on an IOMMU-backed SoC it is a crash rather than a nuisance:
 *
 *  - bs->dma_addr is an IOVA minted by vb2-dma-contig. MMAP planes come from
 *    dma_alloc_attrs() -> iommu_dma_alloc() -> iommu_dma_alloc_remap(), i.e. a
 *    set of PHYSICALLY SCATTERED pages that are contiguous only in IOVA space
 *    (and vmap()ed for the CPU). DMABUF/USERPTR planes are an imported
 *    scatterlist. Neither is a dma_map_single()/dma_map_page() mapping.
 *  - iommu_dma_sync_single_for_device() translates the FIRST byte only
 *    (iommu_iova_to_phys()) and hands (phys, size) to
 *    arch_sync_dma_for_device(), which cleans `size` bytes of PHYSICALLY
 *    CONTIGUOUS memory starting there. Past the first page that is not the
 *    buffer -- it is whatever else happens to live at those physical
 *    addresses.
 *  - On med-mt8365-pumpkin, 2026-08-20, one such walk ran off the end of the
 *    first System RAM block (40000000-42ffffff) into the no-map secmon
 *    carve-out at 43000000 and took a level-3 translation fault in
 *    dcache_clean_poc(), killing the vdec workqueue. Once in 415 armed HEVC
 *    decodes: the HEVC conformance clip submits ~216 kB (53 pages) per access
 *    unit, so it reaches a hole far more often than the ~2-page H.264 / VP8 /
 *    VP9 submissions do -- but the defect was identical in all four backends.
 *
 * No CPU-side cache maintenance belongs here at all; vb2 owns the plane's
 * cache state and already does it correctly:
 *
 *  - coherent MMAP planes are mapped uncached by dma_pgprot() on this
 *    non-coherent device, which is exactly why vb2_dc_prepare() is a no-op for
 *    them;
 *  - non-coherent MMAP planes (V4L2_MEMORY_FLAG_NON_COHERENT, permitted here
 *    because both queues set allow_cache_hints) are flushed by
 *    vb2_dc_prepare() -> dma_sync_sgtable_for_device(), which IS scatter-aware,
 *    before ->decode() is ever entered;
 *  - DMABUF planes are the exporter's business.
 *
 * The removed sync was also never load-bearing: for every page but the first it
 * was cleaning unrelated memory, and the H.264 / VP8 / VP9 conformance scores
 * on record were obtained with it doing precisely that.
 *
 * What replaces it is an unconditional AUDIT of the same address and length, so
 * the denominator is observable from userspace and any future fault on this
 * path is attributable instead of mysterious. It performs no cache maintenance
 * and issues no sync; dma_wmb() is the whole of the CPU's obligation before the
 * engine is kicked.
 */
static int mt8365_bs_total;
module_param(mt8365_bs_total, int, 0444);
MODULE_PARM_DESC(mt8365_bs_total,
		 "MT8365 N45: bitstream submissions audited (the denominator)");

static int mt8365_bs_multipage;
module_param(mt8365_bs_multipage, int, 0444);
MODULE_PARM_DESC(mt8365_bs_multipage,
		 "MT8365 N45: submissions spanning more than one page -- the ones the removed dma_sync_single_for_device() mis-handled");

static int mt8365_bs_scattered;
module_param(mt8365_bs_scattered, int, 0444);
MODULE_PARM_DESC(mt8365_bs_scattered,
		 "MT8365 N45: submissions whose IOVA range is NOT physically contiguous -- the removed sync would have cleaned foreign memory");

static int mt8365_bs_untranslated;
module_param(mt8365_bs_untranslated, int, 0444);
MODULE_PARM_DESC(mt8365_bs_untranslated,
		 "MT8365 N45: submissions whose IOVA did not translate at all (stale or unmapped handle)");

static int mt8365_bs_foreign_pages;
module_param(mt8365_bs_foreign_pages, int, 0444);
MODULE_PARM_DESC(mt8365_bs_foreign_pages,
		 "MT8365 N45: cumulative pages of FOREIGN cache maintenance the removed sync would have performed");

void mtk_vcodec_mt8365_bs_audit(struct mtk_vcodec_dec_ctx *ctx,
				dma_addr_t dma_addr, size_t len)
{
	struct device *dev = &ctx->dev->plat_dev->dev;
	unsigned long first, last;
	struct iommu_domain *dom;

	mt8365_bs_total++;

	if (!len)
		goto out;

	first = (unsigned long)(dma_addr >> PAGE_SHIFT);
	last = (unsigned long)((dma_addr + len - 1) >> PAGE_SHIFT);
	if (last != first) {
		mt8365_bs_multipage++;
		mt8365_bs_foreign_pages += last - first;
	}

	/*
	 * The check the removed sync never made: does [dma_addr, +len) map to
	 * ONE physically contiguous run? Only then would the streaming helper
	 * have been operating on the buffer at all.
	 */
	dom = iommu_get_domain_for_dev(dev);
	if (dom) {
		phys_addr_t p0 = iommu_iova_to_phys(dom, dma_addr);
		phys_addr_t pn = iommu_iova_to_phys(dom, dma_addr + len - 1);

		if (!p0 || !pn) {
			mt8365_bs_untranslated++;
			mtk_v4l2_vdec_err(ctx,
					  "mt8365 N45: bitstream IOVA %pad+%zx does not translate (p0=%pa pn=%pa)",
					  &dma_addr, len, &p0, &pn);
		} else if (pn - p0 != len - 1) {
			mt8365_bs_scattered++;
			if (mt8365_bs_scattered <= 2)
				mtk_v4l2_vdec_err(ctx,
						  "mt8365 N45: bitstream IOVA %pad+%zx is physically SCATTERED (p0=%pa pn=%pa) -- no streaming sync is legal on it",
						  &dma_addr, len, &p0, &pn);
		}
	}

out:
	/*
	 * Order the CPU's writes to the (uncached) plane ahead of the register
	 * writes that arm and kick the engine. This is a barrier, NOT cache
	 * maintenance -- see the block comment above.
	 */
	dma_wmb();
}

static int mtk_vcodec_get_hw_count(struct mtk_vcodec_dec_ctx *ctx, struct mtk_vcodec_dec_dev *dev)
{
	switch (dev->vdec_pdata->hw_arch) {
	case MTK_VDEC_PURE_SINGLE_CORE:
		return MTK_VDEC_ONE_CORE;
	case MTK_VDEC_LAT_SINGLE_CORE:
		return MTK_VDEC_ONE_LAT_ONE_CORE;
	default:
		mtk_v4l2_vdec_err(ctx, "hw arch %d not supported", dev->vdec_pdata->hw_arch);
		return MTK_VDEC_NO_HW;
	}
}

static bool mtk_vcodec_is_hw_active(struct mtk_vcodec_dec_dev *dev)
{
	u32 cg_status;

	if (dev->vdecsys_regmap)
		return !regmap_test_bits(dev->vdecsys_regmap, VDEC_HW_ACTIVE_ADDR,
					 VDEC_HW_ACTIVE_MASK);

	cg_status = readl(dev->reg_base[VDEC_SYS] + VDEC_HW_ACTIVE_ADDR);
	return !FIELD_GET(VDEC_HW_ACTIVE_MASK, cg_status);
}

static irqreturn_t mtk_vcodec_dec_irq_handler(int irq, void *priv)
{
	struct mtk_vcodec_dec_dev *dev = priv;
	struct mtk_vcodec_dec_ctx *ctx;
	unsigned int dec_done_status = 0;
	void __iomem *vdec_misc_addr = dev->reg_base[VDEC_MISC] +
					VDEC_IRQ_CFG_REG;

	vdec_irq_total++;

	ctx = mtk_vcodec_get_curr_ctx(dev, MTK_VDEC_CORE);
	if (!ctx) {
		/*
		 * No decode owns this interrupt.
		 *
		 * vdec_if_decode() clears curr_ctx when a decode returns, and
		 * the IRQ line is IRQ_NOAUTOEN: it is re-enabled per decode by
		 * mtk_vcodec_dec_enable_hardware() -> enable_irq(), which runs
		 * BEFORE mtk_vcodec_set_curr_ctx() publishes the next context.
		 * An interrupt a previous decode left latched (the handler has
		 * two paths that return without acknowledging) is therefore
		 * delivered inside that enable_irq() with curr_ctx == NULL.
		 * Without this test the very next statement writes through the
		 * NULL ctx at offsetof(irq_status) == 0x200 and, being in
		 * interrupt context, panics the machine.
		 *
		 * Acknowledge and drop it: it belongs to no context, and waking
		 * the next one with it would be a false decode-done.
		 */
		vdec_irq_noctx++;
		writel((readl(vdec_misc_addr) | VDEC_IRQ_CFG),
		       dev->reg_base[VDEC_MISC] + VDEC_IRQ_CFG_REG);
		writel((readl(vdec_misc_addr) & ~VDEC_IRQ_CLR),
		       dev->reg_base[VDEC_MISC] + VDEC_IRQ_CFG_REG);
		return IRQ_HANDLED;
	}

	/*
	 * Do NOT acknowledge here. The gate reports the VDEC block clock-gated
	 * (VDEC_SYS+0x0 bit4), and the acknowledge is a read-modify-write of
	 * VDEC_MISC+0xa4 -- programming a gated block, which on this SoC is how
	 * bus aborts and hard resets are produced. There is nothing this
	 * handler can safely do with the interrupt, so returning without an
	 * acknowledge is correct; it is the one early return here that is not a
	 * missing acknowledge.
	 */
	if (!mtk_vcodec_is_hw_active(dev)) {
		vdec_irq_inactive++;
		mtk_v4l2_vdec_err(ctx, "DEC ISR, VDEC active is not 0x0");
		return IRQ_HANDLED;
	}

	dec_done_status = readl(vdec_misc_addr);
	ctx->irq_status = dec_done_status;
	if ((dec_done_status & MTK_VDEC_IRQ_STATUS_DEC_SUCCESS) !=
		MTK_VDEC_IRQ_STATUS_DEC_SUCCESS) {
		/*
		 * The block is active and its status register read back, so
		 * this interrupt is ours and the hardware raised it for
		 * something other than a completed picture. Returning without
		 * clearing MISC+0xa4 leaves the level line asserted: it
		 * re-fires until disable_hardware() masks it, and is then
		 * delivered at the next decode's enable_irq() with curr_ctx
		 * still NULL -- the exact precondition of the NULL-ctx panic.
		 *
		 * Acknowledge it with the success path's own clear sequence
		 * and wake nobody: ctx->irq_status already carries the status
		 * for the waiter, and this is not a decode-done.
		 */
		vdec_irq_nosucc++;
		writel((readl(vdec_misc_addr) | VDEC_IRQ_CFG),
		       dev->reg_base[VDEC_MISC] + VDEC_IRQ_CFG_REG);
		writel((readl(vdec_misc_addr) & ~VDEC_IRQ_CLR),
		       dev->reg_base[VDEC_MISC] + VDEC_IRQ_CFG_REG);
		return IRQ_HANDLED;
	}

	vdec_irq_done++;

	/* clear interrupt */
	writel((readl(vdec_misc_addr) | VDEC_IRQ_CFG),
		dev->reg_base[VDEC_MISC] + VDEC_IRQ_CFG_REG);
	writel((readl(vdec_misc_addr) & ~VDEC_IRQ_CLR),
		dev->reg_base[VDEC_MISC] + VDEC_IRQ_CFG_REG);

	wake_up_dec_ctx(ctx, MTK_INST_IRQ_RECEIVED, 0);

	mtk_v4l2_vdec_dbg(3, ctx, "wake up ctx %d, dec_done_status=%x", ctx->id, dec_done_status);

	return IRQ_HANDLED;
}

static int mtk_vcodec_get_reg_bases(struct mtk_vcodec_dec_dev *dev)
{
	struct platform_device *pdev = dev->plat_dev;
	int reg_num, i;
	struct resource *res;
	bool has_vdecsys_reg;
	int num_max_vdec_regs;
	static const char * const mtk_dec_reg_names[] = {
		"misc",
		"ld",
		"top",
		"cm",
		"ad",
		"av",
		"pp",
		"hwd",
		"hwq",
		"hwb",
		"hwg"
	};

	/*
	 * If we have reg-names in devicetree, this means that we're on a new
	 * register organization, which implies that the VDEC_SYS iospace gets
	 * R/W through a syscon (regmap).
	 * Here we try to get the "misc" iostart only to check if we have reg-names
	 */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "misc");
	if (res)
		has_vdecsys_reg = false;
	else
		has_vdecsys_reg = true;

	num_max_vdec_regs = has_vdecsys_reg ? NUM_MAX_VDEC_REG_BASE :
					      ARRAY_SIZE(mtk_dec_reg_names);

	/* Sizeof(u32) * 4 bytes for each register base. */
	reg_num = of_property_count_elems_of_size(pdev->dev.of_node, "reg",
						  sizeof(u32) * 4);
	if (reg_num <= 0 || reg_num > num_max_vdec_regs) {
		dev_err(&pdev->dev, "Invalid register property size: %d\n", reg_num);
		return -EINVAL;
	}

	if (has_vdecsys_reg) {
		for (i = 0; i < reg_num; i++) {
			dev->reg_base[i] = devm_platform_ioremap_resource(pdev, i);
			if (IS_ERR(dev->reg_base[i]))
				return PTR_ERR(dev->reg_base[i]);

			dev_dbg(&pdev->dev, "reg[%d] base=%p", i, dev->reg_base[i]);
		}
	} else {
		for (i = 0; i < reg_num; i++) {
			dev->reg_base[i+1] = devm_platform_ioremap_resource_byname(pdev, mtk_dec_reg_names[i]);
			if (IS_ERR(dev->reg_base[i+1]))
				return PTR_ERR(dev->reg_base[i+1]);

			dev_dbg(&pdev->dev, "reg[%d] base=%p", i + 1, dev->reg_base[i + 1]);
		}

		dev->vdecsys_regmap = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
								      "mediatek,vdecsys");
		if (IS_ERR(dev->vdecsys_regmap)) {
			dev_err(&pdev->dev, "Missing mediatek,vdecsys property");
			return PTR_ERR(dev->vdecsys_regmap);
		}
	}

	return 0;
}

static int mtk_vcodec_init_dec_resources(struct mtk_vcodec_dec_dev *dev)
{
	struct platform_device *pdev = dev->plat_dev;
	int ret;

	ret = mtk_vcodec_get_reg_bases(dev);
	if (ret)
		return ret;

	if (dev->vdec_pdata->is_subdev_supported)
		return 0;

	dev->dec_irq = platform_get_irq(pdev, 0);
	if (dev->dec_irq < 0)
		return dev->dec_irq;

	irq_set_status_flags(dev->dec_irq, IRQ_NOAUTOEN);
	ret = devm_request_irq(&pdev->dev, dev->dec_irq,
			       mtk_vcodec_dec_irq_handler, 0, pdev->name, dev);
	if (ret) {
		dev_err(&pdev->dev, "failed to install dev->dec_irq %d (%d)",
			dev->dec_irq, ret);
		return ret;
	}

	ret = mtk_vcodec_init_dec_clk(pdev, &dev->pm);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to get mt vcodec clock source");
		return ret;
	}

	pm_runtime_enable(&pdev->dev);
	return 0;
}

static int fops_vcodec_open(struct file *file)
{
	struct mtk_vcodec_dec_dev *dev = video_drvdata(file);
	struct mtk_vcodec_dec_ctx *ctx = NULL;
	int ret = 0, i, hw_count;
	struct vb2_queue *src_vq;
	unsigned long flags;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mutex_lock(&dev->dev_mutex);
	ctx->id = dev->id_counter++;
	v4l2_fh_init(&ctx->fh, video_devdata(file));
	v4l2_fh_add(&ctx->fh, file);
	INIT_LIST_HEAD(&ctx->list);
	ctx->dev = dev;
	if (ctx->dev->vdec_pdata->is_subdev_supported) {
		hw_count = mtk_vcodec_get_hw_count(ctx, dev);
		if (!hw_count || !dev->subdev_prob_done) {
			ret = -EINVAL;
			goto err_ctrls_setup;
		}

		ret = dev->subdev_prob_done(dev);
		if (ret)
			goto err_ctrls_setup;

		for (i = 0; i < hw_count; i++)
			init_waitqueue_head(&ctx->queue[i]);
	} else {
		init_waitqueue_head(&ctx->queue[0]);
	}
	mutex_init(&ctx->lock);

	ctx->type = MTK_INST_DECODER;
	ret = dev->vdec_pdata->ctrls_setup(ctx);
	if (ret) {
		mtk_v4l2_vdec_err(ctx, "Failed to setup mt vcodec controls");
		goto err_ctrls_setup;
	}
	ctx->m2m_ctx = v4l2_m2m_ctx_init(dev->m2m_dev_dec, ctx,
		&mtk_vcodec_dec_queue_init);
	if (IS_ERR((__force void *)ctx->m2m_ctx)) {
		ret = PTR_ERR((__force void *)ctx->m2m_ctx);
		mtk_v4l2_vdec_err(ctx, "Failed to v4l2_m2m_ctx_init() (%d)", ret);
		goto err_m2m_ctx_init;
	}
	src_vq = v4l2_m2m_get_vq(ctx->m2m_ctx,
				V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	ctx->empty_flush_buf.vb.vb2_buf.vb2_queue = src_vq;
	mtk_vcodec_dec_set_default_params(ctx);

	if (v4l2_fh_is_singular(&ctx->fh)) {
		if (dev->vdec_pdata->no_firmware) {
			/*
			 * MT8365: no SCP/codec firmware -- the capability is not
			 * firmware-reported, set it statically (H.264 decode, MM21
			 * output) for the direct-register backend.
			 */
			dev->dec_capability =
				MTK_VDEC_FORMAT_H264_SLICE |
				MTK_VDEC_FORMAT_VP8_FRAME |
				MTK_VDEC_FORMAT_MM21;
		} else {
			/*
			 * Does nothing if firmware was already loaded.
			 */
			ret = mtk_vcodec_fw_load_firmware(dev->fw_handler);
			if (ret < 0) {
				/*
				 * Return 0 if downloading firmware successfully,
				 * otherwise it is failed
				 */
				mtk_v4l2_vdec_err(ctx, "failed to load firmware!");
				goto err_load_fw;
			}

			dev->dec_capability =
				mtk_vcodec_fw_get_vdec_capa(dev->fw_handler);

			mtk_v4l2_vdec_dbg(0, ctx, "decoder capability %x", dev->dec_capability);
		}
	}

	ctx->dev->vdec_pdata->init_vdec_params(ctx);

	spin_lock_irqsave(&dev->dev_ctx_lock, flags);
	list_add(&ctx->list, &dev->ctx_list);
	spin_unlock_irqrestore(&dev->dev_ctx_lock, flags);
	mtk_vcodec_dbgfs_create(ctx);

	mutex_unlock(&dev->dev_mutex);
	mtk_v4l2_vdec_dbg(0, ctx, "%s decoder [%d]", dev_name(&dev->plat_dev->dev), ctx->id);
	return ret;

	/* Deinit when failure occurred */
err_load_fw:
	v4l2_m2m_ctx_release(ctx->m2m_ctx);
err_m2m_ctx_init:
	v4l2_ctrl_handler_free(&ctx->ctrl_hdl);
err_ctrls_setup:
	v4l2_fh_del(&ctx->fh, file);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	mutex_unlock(&dev->dev_mutex);

	return ret;
}

static int fops_vcodec_release(struct file *file)
{
	struct mtk_vcodec_dec_dev *dev = video_drvdata(file);
	struct mtk_vcodec_dec_ctx *ctx = file_to_dec_ctx(file);
	unsigned long flags;

	mtk_v4l2_vdec_dbg(0, ctx, "[%d] decoder", ctx->id);
	mutex_lock(&dev->dev_mutex);

	/*
	 * Call v4l2_m2m_ctx_release before mtk_vcodec_dec_release. First, it
	 * makes sure the worker thread is not running after vdec_if_deinit.
	 * Second, the decoder will be flushed and all the buffers will be
	 * returned in stop_streaming.
	 */
	v4l2_m2m_ctx_release(ctx->m2m_ctx);
	mtk_vcodec_dec_release(ctx);

	v4l2_fh_del(&ctx->fh, file);
	v4l2_fh_exit(&ctx->fh);
	v4l2_ctrl_handler_free(&ctx->ctrl_hdl);

	mtk_vcodec_dbgfs_remove(dev, ctx->id);
	spin_lock_irqsave(&dev->dev_ctx_lock, flags);
	list_del_init(&ctx->list);
	spin_unlock_irqrestore(&dev->dev_ctx_lock, flags);
	kfree(ctx);
	mutex_unlock(&dev->dev_mutex);
	return 0;
}

static const struct v4l2_file_operations mtk_vcodec_fops = {
	.owner		= THIS_MODULE,
	.open		= fops_vcodec_open,
	.release	= fops_vcodec_release,
	.poll		= v4l2_m2m_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= v4l2_m2m_fop_mmap,
};

static void mtk_vcodec_dec_get_chip_name(struct mtk_vcodec_dec_dev *vdec_dev)
{
	struct device *dev = &vdec_dev->plat_dev->dev;

	if (of_device_is_compatible(dev->of_node, "mediatek,mt8173-vcodec-dec"))
		vdec_dev->chip_name = MTK_VDEC_MT8173;
	else if (of_device_is_compatible(dev->of_node, "mediatek,mt8183-vcodec-dec"))
		vdec_dev->chip_name = MTK_VDEC_MT8183;
	else if (of_device_is_compatible(dev->of_node, "mediatek,mt8192-vcodec-dec"))
		vdec_dev->chip_name = MTK_VDEC_MT8192;
	else if (of_device_is_compatible(dev->of_node, "mediatek,mt8195-vcodec-dec"))
		vdec_dev->chip_name = MTK_VDEC_MT8195;
	else if (of_device_is_compatible(dev->of_node, "mediatek,mt8186-vcodec-dec"))
		vdec_dev->chip_name = MTK_VDEC_MT8186;
	else if (of_device_is_compatible(dev->of_node, "mediatek,mt8188-vcodec-dec"))
		vdec_dev->chip_name = MTK_VDEC_MT8188;
	else if (of_device_is_compatible(dev->of_node, "mediatek,mt8365-vcodec-dec"))
		vdec_dev->chip_name = MTK_VDEC_MT8365;
	else
		vdec_dev->chip_name = MTK_VDEC_INVAL;
}

/*
 * A file handle outlives the driver binding, so `struct mtk_vcodec_dec_dev` and
 * the objects it owns cannot be freed by remove(). They are freed here instead,
 * from the v4l2_device kref, which the video_device holds a reference on for as
 * long as any fd is open.
 *
 * Everything torn down here used to be torn down in remove() while an open
 * context still pointed at it: v4l2_m2m_ctx_release() from fops_vcodec_release()
 * dereferences m2m_ctx->m2m_dev, and vdec_if_deinit() dereferences ctx->dev.
 */
static void mtk_vcodec_dec_dev_release(struct v4l2_device *v4l2_dev)
{
	struct mtk_vcodec_dec_dev *dev =
		container_of(v4l2_dev, struct mtk_vcodec_dec_dev, v4l2_dev);

	if (dev->m2m_dev_dec)
		v4l2_m2m_release(dev->m2m_dev_dec);
	if (dev->decode_workqueue)
		destroy_workqueue(dev->decode_workqueue);
	if (dev->core_workqueue)
		destroy_workqueue(dev->core_workqueue);
	if (dev->mdev_dec.dev)
		media_device_cleanup(&dev->mdev_dec);
	mtk_vcodec_fw_release(dev->fw_handler);
	kfree(dev);
}

static int mtk_vcodec_probe(struct platform_device *pdev)
{
	struct mtk_vcodec_dec_dev *dev;
	struct video_device *vfd_dec;
	phandle rproc_phandle;
	enum mtk_vcodec_fw_type fw_type;
	int i, ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	INIT_LIST_HEAD(&dev->ctx_list);
	dev->plat_dev = pdev;

	mtk_vcodec_dec_get_chip_name(dev);
	if (dev->chip_name == MTK_VDEC_INVAL) {
		dev_err(&pdev->dev, "Failed to get decoder chip name");
		ret = -EINVAL;
		goto err_free_dev;
	}

	dev->vdec_pdata = of_device_get_match_data(&pdev->dev);
	if (dev->vdec_pdata->no_firmware) {
		/*
		 * MT8365: no SCP/VPU codec coprocessor -- the decode is driven by
		 * direct register programming, so there is no firmware handler
		 * and no mediatek,vpu/mediatek,scp phandle.
		 */
		dev->fw_handler = NULL;
		dma_set_max_seg_size(&pdev->dev, UINT_MAX);
	} else {
		if (!of_property_read_u32(pdev->dev.of_node, "mediatek,vpu",
					  &rproc_phandle)) {
			fw_type = VPU;
		} else if (!of_property_read_u32(pdev->dev.of_node, "mediatek,scp",
						 &rproc_phandle)) {
			fw_type = SCP;
		} else {
			dev_dbg(&pdev->dev, "Could not get vdec IPI device");
			ret = -ENODEV;
			goto err_free_dev;
		}
		dma_set_max_seg_size(&pdev->dev, UINT_MAX);

		dev->fw_handler = mtk_vcodec_fw_select(dev, fw_type, DECODER);
		if (IS_ERR(dev->fw_handler)) {
			ret = PTR_ERR(dev->fw_handler);
			dev->fw_handler = NULL;
			goto err_free_dev;
		}
	}

	ret = mtk_vcodec_init_dec_resources(dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to init dec resources");
		goto err_dec_pm;
	}

	if (IS_VDEC_LAT_ARCH(dev->vdec_pdata->hw_arch)) {
		dev->core_workqueue =
			alloc_ordered_workqueue("core-decoder",
						WQ_MEM_RECLAIM | WQ_FREEZABLE);
		if (!dev->core_workqueue) {
			dev_dbg(&pdev->dev, "Failed to create core workqueue");
			ret = -EINVAL;
			goto err_res;
		}
	}

	for (i = 0; i < MTK_VDEC_HW_MAX; i++)
		mutex_init(&dev->dec_mutex[i]);
	mutex_init(&dev->dev_mutex);
	spin_lock_init(&dev->dev_ctx_lock);
	spin_lock_init(&dev->irqlock);

	snprintf(dev->v4l2_dev.name, sizeof(dev->v4l2_dev.name), "%s",
		"[/MTK_V4L2_VDEC]");
	/*
	 * From here on the device is refcounted: video_register_device() takes a
	 * reference and the video_device's own release drops it, so `dev` is
	 * freed only once the last file handle is gone.
	 */
	dev->v4l2_dev.release = mtk_vcodec_dec_dev_release;

	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret) {
		dev_err(&pdev->dev, "v4l2_device_register err=%d", ret);
		goto err_core_workq;
	}

	vfd_dec = video_device_alloc();
	if (!vfd_dec) {
		dev_err(&pdev->dev, "Failed to allocate video device");
		ret = -ENOMEM;
		goto err_dec_alloc;
	}
	vfd_dec->fops		= &mtk_vcodec_fops;
	vfd_dec->ioctl_ops	= &mtk_vdec_ioctl_ops;
	vfd_dec->release	= video_device_release;
	vfd_dec->lock		= &dev->dev_mutex;
	vfd_dec->v4l2_dev	= &dev->v4l2_dev;
	vfd_dec->vfl_dir	= VFL_DIR_M2M;
	vfd_dec->device_caps	= V4L2_CAP_VIDEO_M2M_MPLANE |
			V4L2_CAP_STREAMING;

	snprintf(vfd_dec->name, sizeof(vfd_dec->name), "%s",
		MTK_VCODEC_DEC_NAME);
	video_set_drvdata(vfd_dec, dev);
	dev->vfd_dec = vfd_dec;
	platform_set_drvdata(pdev, dev);

	dev->m2m_dev_dec = v4l2_m2m_init(&mtk_vdec_m2m_ops);
	if (IS_ERR((__force void *)dev->m2m_dev_dec)) {
		dev_err(&pdev->dev, "Failed to init mem2mem dec device");
		ret = PTR_ERR((__force void *)dev->m2m_dev_dec);
		goto err_dec_alloc;
	}

	dev->decode_workqueue =
		alloc_ordered_workqueue(MTK_VCODEC_DEC_NAME,
			WQ_MEM_RECLAIM | WQ_FREEZABLE);
	if (!dev->decode_workqueue) {
		dev_err(&pdev->dev, "Failed to create decode workqueue");
		ret = -EINVAL;
		goto err_event_workq;
	}

	if (dev->vdec_pdata->is_subdev_supported) {
		ret = of_platform_populate(pdev->dev.of_node, NULL, NULL,
					   &pdev->dev);
		if (ret) {
			dev_err(&pdev->dev, "Main device of_platform_populate failed.");
			goto err_reg_cont;
		}
	} else {
		set_bit(MTK_VDEC_CORE, dev->subdev_bitmap);
	}

	atomic_set(&dev->dec_active_cnt, 0);
	memset(dev->vdec_racing_info, 0, sizeof(dev->vdec_racing_info));
	mutex_init(&dev->dec_racing_info_mutex);

	/*
	 * BEFORE video_register_device(), not after: registering the node makes
	 * it openable immediately, and fops_vcodec_open() -> mtk_vcodec_dbgfs_create()
	 * does list_add_tail() onto dev->dbgfs.dbgfs_head. Initialising that head
	 * afterwards leaves a window in which an open() adds to a zeroed list head
	 * -- "list_add corruption. prev is NULL." with the driver still probing.
	 */
	mtk_vcodec_dbgfs_init(dev, false);

	ret = video_register_device(vfd_dec, VFL_TYPE_VIDEO, -1);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register video device");
		goto err_dbgfs;
	}

	if (dev->vdec_pdata->uses_stateless_api) {
		v4l2_disable_ioctl(vfd_dec, VIDIOC_DECODER_CMD);
		v4l2_disable_ioctl(vfd_dec, VIDIOC_TRY_DECODER_CMD);

		dev->mdev_dec.dev = &pdev->dev;
		strscpy(dev->mdev_dec.model, MTK_VCODEC_DEC_NAME,
			sizeof(dev->mdev_dec.model));

		media_device_init(&dev->mdev_dec);
		dev->mdev_dec.ops = &mtk_vcodec_media_ops;
		dev->v4l2_dev.mdev = &dev->mdev_dec;

		ret = v4l2_m2m_register_media_controller(dev->m2m_dev_dec, dev->vfd_dec,
							 MEDIA_ENT_F_PROC_VIDEO_DECODER);
		if (ret) {
			dev_err(&pdev->dev, "Failed to register media controller");
			goto err_dec_mem_init;
		}

		ret = media_device_register(&dev->mdev_dec);
		if (ret) {
			dev_err(&pdev->dev, "Failed to register media device");
			goto err_media_reg;
		}

		dev_dbg(&pdev->dev, "media registered as /dev/media%d", vfd_dec->minor);
	}

	dev_dbg(&pdev->dev, "decoder registered as /dev/video%d", vfd_dec->minor);

	return 0;

err_media_reg:
	v4l2_m2m_unregister_media_controller(dev->m2m_dev_dec);
err_dec_mem_init:
	video_unregister_device(vfd_dec);
err_dbgfs:
	mtk_vcodec_dbgfs_deinit(&dev->dbgfs);
err_reg_cont:
err_event_workq:
err_dec_alloc:
	/*
	 * The v4l2_device is registered, so the workqueues, the m2m device, the
	 * media device and `dev` itself are all released by
	 * mtk_vcodec_dec_dev_release() -- do not free them here as well.
	 */
	if (!dev->vdec_pdata->is_subdev_supported)
		pm_runtime_disable(dev->pm.dev);
	v4l2_device_unregister(&dev->v4l2_dev);
	v4l2_device_put(&dev->v4l2_dev);
	return ret;
err_core_workq:
	if (IS_VDEC_LAT_ARCH(dev->vdec_pdata->hw_arch))
		destroy_workqueue(dev->core_workqueue);
err_res:
	if (!dev->vdec_pdata->is_subdev_supported)
		pm_runtime_disable(dev->pm.dev);
err_dec_pm:
	mtk_vcodec_fw_release(dev->fw_handler);
err_free_dev:
	kfree(dev);
	return ret;
}

static const struct of_device_id mtk_vcodec_match[] = {
	{
		.compatible = "mediatek,mt8173-vcodec-dec",
		.data = &mtk_vdec_8173_pdata,
	},
	{
		.compatible = "mediatek,mt8183-vcodec-dec",
		.data = &mtk_vdec_8183_pdata,
	},
	{
		.compatible = "mediatek,mt8365-vcodec-dec",
		.data = &mtk_vdec_mt8365_pdata,
	},
	{
		.compatible = "mediatek,mt8192-vcodec-dec",
		.data = &mtk_lat_sig_core_pdata,
	},
	{
		.compatible = "mediatek,mt8186-vcodec-dec",
		.data = &mtk_vdec_single_core_pdata,
	},
	{
		.compatible = "mediatek,mt8195-vcodec-dec",
		.data = &mtk_lat_sig_core_pdata,
	},
	{
		.compatible = "mediatek,mt8188-vcodec-dec",
		.data = &mtk_lat_sig_core_pdata,
	},
	{},
};

MODULE_DEVICE_TABLE(of, mtk_vcodec_match);

static void mtk_vcodec_dec_remove(struct platform_device *pdev)
{
	struct mtk_vcodec_dec_dev *dev = platform_get_drvdata(pdev);

	/*
	 * Fence the file operations FIRST, and record the unbind in the device
	 * BEFORE anything is released.
	 *
	 * video_unregister_device() stops new open()s and new ioctls, but it
	 * waits for nothing: the release() of an fd that is already open still
	 * runs -- whenever userspace gets round to closing it -- and an ioctl
	 * that was already inside still finishes. Both used to run against a
	 * `struct mtk_vcodec_dec_dev` that devres had freed, register windows
	 * devres had unmapped, and an IRQ and clocks devres had released. That
	 * is a plain use-after-free and it needs no hardware fault to reach:
	 * opening the node, doing one S_FMT and closing the fd after an unbind
	 * hits it every time.
	 *
	 * ->unplugged tells the stragglers to touch no hardware, and the device
	 * itself now lives on the v4l2_device kref, so the memory they still
	 * walk stays valid until the last handle is gone.
	 */
	if (dev->vfd_dec)
		video_unregister_device(dev->vfd_dec);
	WRITE_ONCE(dev->unplugged, true);

	if (media_devnode_is_registered(dev->mdev_dec.devnode)) {
		media_device_unregister(&dev->mdev_dec);
		v4l2_m2m_unregister_media_controller(dev->m2m_dev_dec);
	}

	mtk_vcodec_dbgfs_deinit(&dev->dbgfs);
	if (!dev->vdec_pdata->is_subdev_supported)
		pm_runtime_disable(dev->pm.dev);
	v4l2_device_unregister(&dev->v4l2_dev);
	v4l2_device_put(&dev->v4l2_dev);
}

static struct platform_driver mtk_vcodec_dec_driver = {
	.probe	= mtk_vcodec_probe,
	.remove = mtk_vcodec_dec_remove,
	.driver	= {
		.name	= MTK_VCODEC_DEC_NAME,
		.of_match_table = mtk_vcodec_match,
	},
};

module_platform_driver(mtk_vcodec_dec_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Mediatek video codec V4L2 decoder driver");
