// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2016 MediaTek Inc.
 * Author: Tiffany Lin <tiffany.lin@mediatek.com>
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>

#include "mtk_vcodec_dec.h"
#include "mtk_vcodec_dec_hw.h"
#include "mtk_vcodec_dec_pm.h"

/*
 * ---- masked-completion probe (2026-08-18) --------------------------------
 *
 * The NULL-ctx panic is an interrupt a previous decode left LATCHED, delivered
 * inside the next decode's enable_irq() before the context is published. The
 * two non-acknowledging handler returns were measured over 20,185 interrupts
 * and never entered, so they cannot be its source (per the internal
 * mt8365 VDEC IRQ-ack-paths audit).
 *
 * The remaining named candidate: a completion asserted while the line is
 * MASKED. mtk_vcodec_dec_disable_hardware() masks the IRQ after a decode gives
 * up at its timeout; if the hardware then finishes and sets DEC_SUCCESS in
 * VDEC_MISC+0xa4, nobody can take it and it sits there until the next
 * enable_irq().
 *
 * So: read VDEC_MISC+0xa4 immediately BEFORE the mask, immediately AFTER it,
 * and once more after a delay (that third read only on a decode that failed --
 * it sleeps, and the shared decode path must not be perturbed); then read it
 * once more just BEFORE the next enable_irq().
 *
 * SAFETY, and why VDEC_SYS+0x0 bit4 is recorded but does NOT gate the read.
 * The first cut of this probe used the ISR's HW-active test as a veto and
 * measured itself off-path: 58 of 72 mask-side probes and 72 of 72 enable-side
 * probes were skipped as "gated", including every probe after a timeout -- the
 * ones the round exists to take. The bit is an engine busy/idle indicator, not
 * an is-the-clock-on indicator: it read idle immediately after child_dev_on()
 * had just enabled the clocks and the power domain.
 *
 * What establishes safety here is the CALL ORDER. Both probe points sit
 * strictly inside the clock/power lifecycle -- the enable-side read is after
 * mtk_vcodec_dec_child_dev_on() (clocks + power domain up), the mask-side reads
 * are before mtk_vcodec_dec_child_dev_off() (still up). And the driver's own
 * timeout path already reads this exact register unconditionally, rd(ctx,
 * VDEC_MISC, 0xa4) in vdec_hevc_mt8365_if.c, across every archived wedge. So
 * bit4 is read and recorded alongside each sample as data, never used to
 * withhold a read.
 *
 * No read-modify-write is issued. The acknowledge sequence is the operation
 * that must not touch a gated block; a plain readl() is not that operation.
 *
 * Counters advance unconditionally so a zero is a measured rate with a visible
 * denominator, not an unfalsifiable absence.
 */
#define MTK_VDEC_PROBE_MISC_STATUS	VDEC_IRQ_CFG_REG	/* MISC +0xa4 */

static int vdec_probe = 1;
module_param(vdec_probe, int, 0644);
MODULE_PARM_DESC(vdec_probe, "masked-completion probe: read VDEC_MISC+0xa4 around the IRQ mask/unmask");

static int vdec_probe_dly_us = 5000;
module_param(vdec_probe_dly_us, int, 0644);
MODULE_PARM_DESC(vdec_probe_dly_us, "delay before the third post-mask read, us (failed decodes only)");

static unsigned int vdec_mask_probe_n;
module_param(vdec_mask_probe_n, uint, 0644);
MODULE_PARM_DESC(vdec_mask_probe_n, "mask-side probes taken (denominator)");

static unsigned int vdec_mask_idle;
module_param(vdec_mask_idle, uint, 0644);
MODULE_PARM_DESC(vdec_mask_idle, "mask-side probes where VDEC_SYS+0x0 bit4 read idle (observation, not a veto)");

static unsigned int vdec_mask_fail_n;
module_param(vdec_mask_fail_n, uint, 0644);
MODULE_PARM_DESC(vdec_mask_fail_n, "mask-side probes on a decode that returned non-zero (the timeout path)");

static unsigned int vdec_mask_pre_succ;
module_param(vdec_mask_pre_succ, uint, 0644);
MODULE_PARM_DESC(vdec_mask_pre_succ, "DEC_SUCCESS already set BEFORE the mask");

static unsigned int vdec_mask_post_succ;
module_param(vdec_mask_post_succ, uint, 0644);
MODULE_PARM_DESC(vdec_mask_post_succ, "DEC_SUCCESS appeared in the read immediately AFTER the mask");

static unsigned int vdec_mask_dly_succ;
module_param(vdec_mask_dly_succ, uint, 0644);
MODULE_PARM_DESC(vdec_mask_dly_succ, "DEC_SUCCESS appeared only in the delayed post-mask read");

static unsigned int vdec_mask_last_pre, vdec_mask_last_post, vdec_mask_last_dly;
module_param(vdec_mask_last_pre, uint, 0644);
module_param(vdec_mask_last_post, uint, 0644);
module_param(vdec_mask_last_dly, uint, 0644);

static int vdec_mask_last_active = -1;
module_param(vdec_mask_last_active, int, 0644);
MODULE_PARM_DESC(vdec_mask_last_active, "VDEC_SYS+0x0 bit4 gate at the last mask-side probe (1 = active)");

/* sticky: the last probe that followed a FAILED decode, never overwritten by a good one */
static unsigned int vdec_mask_fail_pre, vdec_mask_fail_post, vdec_mask_fail_dly;
module_param(vdec_mask_fail_pre, uint, 0644);
module_param(vdec_mask_fail_post, uint, 0644);
module_param(vdec_mask_fail_dly, uint, 0644);

static unsigned int vdec_enable_probe_n;
module_param(vdec_enable_probe_n, uint, 0644);
MODULE_PARM_DESC(vdec_enable_probe_n, "enable-side probes taken (denominator)");

static unsigned int vdec_enable_idle;
module_param(vdec_enable_idle, uint, 0644);
MODULE_PARM_DESC(vdec_enable_idle, "enable-side probes where VDEC_SYS+0x0 bit4 read idle (observation, not a veto)");

/*
 * The read that catches the latched interrupt in the act: the first
 * pre-enable_irq() sample AFTER a decode that failed. Sticky -- a later good
 * decode never overwrites it.
 */
static unsigned int vdec_enable_postfail_n;
module_param(vdec_enable_postfail_n, uint, 0644);
MODULE_PARM_DESC(vdec_enable_postfail_n, "pre-enable_irq() samples taken on the decode following a failed one");

static unsigned int vdec_enable_postfail_pre, vdec_enable_postfail_succ;
module_param(vdec_enable_postfail_pre, uint, 0644);
module_param(vdec_enable_postfail_succ, uint, 0644);
MODULE_PARM_DESC(vdec_enable_postfail_succ, "of those, how many carried DEC_SUCCESS");

static unsigned int vdec_enable_stale_succ;
module_param(vdec_enable_stale_succ, uint, 0644);
MODULE_PARM_DESC(vdec_enable_stale_succ, "DEC_SUCCESS already latched in MISC+0xa4 BEFORE enable_irq()");

static unsigned int vdec_enable_nonzero;
module_param(vdec_enable_nonzero, uint, 0644);
MODULE_PARM_DESC(vdec_enable_nonzero, "MISC+0xa4 non-zero (but no DEC_SUCCESS) before enable_irq()");

static unsigned int vdec_enable_last_pre, vdec_enable_stale_val;
module_param(vdec_enable_last_pre, uint, 0644);
module_param(vdec_enable_stale_val, uint, 0644);

static int vdec_enable_last_active = -1;
module_param(vdec_enable_last_active, int, 0644);

/*
 * ---- GIC pending-state probe (2026-08-18) ---------------------------------
 *
 * Every mechanism eliminated so far lives INSIDE the child_dev_on()..
 * child_dev_off() clock/power lifecycle, and no VDEC_MISC+0xa4 read can
 * observe anything outside it. The interrupt controller can. If the VDEC line
 * is latched/asserted at the GIC rather than reported by the block, every
 * block-register probe reads clean while enable_irq() still delivers it.
 *
 * Five sample points around the lifecycle, all via irq_get_irqchip_state(),
 * which walks the MT_SYSIRQ hierarchy up to the GIC and reads GICD_ISPENDR /
 * GICD_ISACTIVER / GICD_ISENABLER. No raw distributor MMIO, no writes.
 *
 *   e0  enable_hardware, BEFORE child_dev_on()   -- clocks + power domain DOWN
 *   e1  enable_hardware, AFTER child_dev_on(), just before enable_irq()
 *   m0  disable_hardware, BEFORE disable_irq()
 *   m1  disable_hardware, AFTER disable_irq()
 *   m2  disable_hardware, AFTER child_dev_off()  -- clocks + power domain DOWN
 *
 * e0 and m2 are the reads the previous rounds structurally could not take.
 *
 * Every point counts its own denominator unconditionally; a *_last value is a
 * bitmask: bit0 pending, bit1 active, bit2 masked (masked == NOT enabled at
 * the distributor).
 */
static int vdec_gic_probe = 1;
module_param(vdec_gic_probe, int, 0644);
MODULE_PARM_DESC(vdec_gic_probe, "sample the GIC state of the VDEC line around the decode lifecycle");

/* identity, so the report does not have to trust an off-board derivation */
static int vdec_gic_irqno = -1;
module_param(vdec_gic_irqno, int, 0444);
MODULE_PARM_DESC(vdec_gic_irqno, "Linux IRQ number of the VDEC line");

static int vdec_gic_hwirq = -1;
module_param(vdec_gic_hwirq, int, 0444);
MODULE_PARM_DESC(vdec_gic_hwirq, "hwirq of the VDEC line in its own (MT_SYSIRQ) domain");

static int vdec_gic_parent_hwirq = -1;
module_param(vdec_gic_parent_hwirq, int, 0444);
MODULE_PARM_DESC(vdec_gic_parent_hwirq, "hwirq of the VDEC line in the parent (GIC) domain");

static unsigned int vdec_gic_err;
module_param(vdec_gic_err, uint, 0644);
MODULE_PARM_DESC(vdec_gic_err, "irq_get_irqchip_state() failures (a non-zero here invalidates the round)");

static int vdec_gic_last_err;
module_param(vdec_gic_last_err, int, 0644);

#define MTK_VDEC_GIC_COUNTERS(pt)					\
	static unsigned int vdec_gic_##pt##_n;				\
	module_param(vdec_gic_##pt##_n, uint, 0644);			\
	static unsigned int vdec_gic_##pt##_pend;			\
	module_param(vdec_gic_##pt##_pend, uint, 0644);			\
	static unsigned int vdec_gic_##pt##_act;			\
	module_param(vdec_gic_##pt##_act, uint, 0644);			\
	static unsigned int vdec_gic_##pt##_mask;			\
	module_param(vdec_gic_##pt##_mask, uint, 0644);			\
	static unsigned int vdec_gic_##pt##_last;			\
	module_param(vdec_gic_##pt##_last, uint, 0644)

MTK_VDEC_GIC_COUNTERS(e0);
MTK_VDEC_GIC_COUNTERS(e1);
MTK_VDEC_GIC_COUNTERS(m0);
MTK_VDEC_GIC_COUNTERS(m1);
MTK_VDEC_GIC_COUNTERS(m2);

/* sticky: the decode that FOLLOWS a failed one (the panic's own shape) */
static unsigned int vdec_gic_pf_n;
module_param(vdec_gic_pf_n, uint, 0644);
static unsigned int vdec_gic_pf_e0pend, vdec_gic_pf_e1pend;
module_param(vdec_gic_pf_e0pend, uint, 0644);
module_param(vdec_gic_pf_e1pend, uint, 0644);
static unsigned int vdec_gic_pf_e0, vdec_gic_pf_e1;
module_param(vdec_gic_pf_e0, uint, 0644);
module_param(vdec_gic_pf_e1, uint, 0644);

/* sticky: the FAILED decode's own mask side (the wedge) */
static unsigned int vdec_gic_fail_n;
module_param(vdec_gic_fail_n, uint, 0644);
static unsigned int vdec_gic_fail_m0, vdec_gic_fail_m1, vdec_gic_fail_m2;
module_param(vdec_gic_fail_m0, uint, 0644);
module_param(vdec_gic_fail_m1, uint, 0644);
module_param(vdec_gic_fail_m2, uint, 0644);

/* set by vdec_if_decode() with the return of the backend decode() */
static int vdec_probe_last_ret;
/* latched by the mask side when that decode failed; consumed by the enable side */
static int vdec_probe_fail_seen;

void mtk_vcodec_dec_probe_note_ret(int ret)
{
	WRITE_ONCE(vdec_probe_last_ret, ret);
}

/*
 * Is the VDEC block ungated right now?  Same test as the ISR's HW-active gate:
 * VDEC_SYS+0x0 bit4 clear == active. Returns -1 when it cannot be read (regmap
 * variant / no mapping), in which case no MISC read is taken at all.
 */
static int mtk_vdec_probe_active(struct mtk_vcodec_dec_dev *dev)
{
	if (dev->vdecsys_regmap || !dev->reg_base[VDEC_SYS])
		return -1;
	return !(readl(dev->reg_base[VDEC_SYS] + VDEC_HW_ACTIVE_ADDR) &
		 VDEC_HW_ACTIVE_MASK);
}

/*
 * The Linux IRQ number of the VDEC line, or -1 when this device/probe cannot
 * be sampled. Records the identity (irq, own hwirq, parent/GIC hwirq) the
 * first time it resolves, so the report reads it off the board.
 */
static int mtk_vdec_gic_irq(struct mtk_vcodec_dec_dev *dev)
{
	struct irq_data *d;
	int irq;

	if (!vdec_gic_probe)
		return -1;
	if (dev->vdec_pdata->is_subdev_supported)	/* MT8365 is pure single core */
		return -1;
	irq = dev->dec_irq;
	if (irq <= 0)
		return -1;

	if (vdec_gic_irqno != irq) {
		vdec_gic_irqno = irq;
		vdec_gic_hwirq = -1;
		vdec_gic_parent_hwirq = -1;
		d = irq_get_irq_data(irq);
		if (d) {
			vdec_gic_hwirq = (int)irqd_to_hwirq(d);
#ifdef CONFIG_IRQ_DOMAIN_HIERARCHY
			if (d->parent_data)
				vdec_gic_parent_hwirq =
					(int)irqd_to_hwirq(d->parent_data);
#endif
		}
	}
	return irq;
}

/*
 * One sample of the interrupt controller's view of @irq. Accumulates into the
 * caller's counters and returns the bitmask (bit0 pending, bit1 active,
 * bit2 masked), or 0 when the sample could not be taken.
 *
 * Read-only: irq_get_irqchip_state() reaches the GIC through the irqchip
 * hierarchy and peeks GICD_ISPENDR/ISACTIVER/ISENABLER. It touches no VDEC
 * register, so it is valid with the block's clocks and power domain DOWN --
 * which is the whole point of the e0 and m2 points.
 */
static unsigned int mtk_vdec_gic_acc(int irq, unsigned int *n, unsigned int *pend,
				     unsigned int *act, unsigned int *masked,
				     unsigned int *last)
{
	unsigned int bits = 0;
	bool v = false;
	int err;

	if (irq < 0)
		return 0;

	err = irq_get_irqchip_state(irq, IRQCHIP_STATE_PENDING, &v);
	if (err) {
		vdec_gic_err++;
		vdec_gic_last_err = err;
		return 0;
	}
	if (v)
		bits |= 1;

	v = false;
	if (!irq_get_irqchip_state(irq, IRQCHIP_STATE_ACTIVE, &v) && v)
		bits |= 2;

	v = false;
	if (!irq_get_irqchip_state(irq, IRQCHIP_STATE_MASKED, &v) && v)
		bits |= 4;

	(*n)++;
	if (bits & 1)
		(*pend)++;
	if (bits & 2)
		(*act)++;
	if (bits & 4)
		(*masked)++;
	*last = bits;
	return bits;
}

#define MTK_VDEC_GIC_ACC(irq, pt)					\
	mtk_vdec_gic_acc((irq), &vdec_gic_##pt##_n, &vdec_gic_##pt##_pend, \
			 &vdec_gic_##pt##_act, &vdec_gic_##pt##_mask,	\
			 &vdec_gic_##pt##_last)

/* MISC+0xa4, or NULL when this device/probe cannot be read safely */
static void __iomem *mtk_vdec_probe_misc(struct mtk_vcodec_dec_dev *dev)
{
	if (!vdec_probe)
		return NULL;
	if (dev->vdec_pdata->is_subdev_supported)	/* MT8365 is pure single core */
		return NULL;
	if (!dev->reg_base[VDEC_MISC])
		return NULL;
	return dev->reg_base[VDEC_MISC] + MTK_VDEC_PROBE_MISC_STATUS;
}

int mtk_vcodec_init_dec_clk(struct platform_device *pdev, struct mtk_vcodec_pm *pm)
{
	struct mtk_vcodec_clk *dec_clk;
	struct mtk_vcodec_clk_info *clk_info;
	int i = 0, ret;

	dec_clk = &pm->vdec_clk;
	pm->dev = &pdev->dev;

	dec_clk->clk_num =
		of_property_count_strings(pdev->dev.of_node, "clock-names");
	if (dec_clk->clk_num > 0) {
		dec_clk->clk_info = devm_kcalloc(&pdev->dev,
			dec_clk->clk_num, sizeof(*clk_info),
			GFP_KERNEL);
		if (!dec_clk->clk_info)
			return -ENOMEM;
	} else {
		dev_err(&pdev->dev, "Failed to get vdec clock count");
		return -EINVAL;
	}

	for (i = 0; i < dec_clk->clk_num; i++) {
		clk_info = &dec_clk->clk_info[i];
		ret = of_property_read_string_index(pdev->dev.of_node,
			"clock-names", i, &clk_info->clk_name);
		if (ret) {
			dev_err(&pdev->dev, "Failed to get clock name id = %d", i);
			return ret;
		}
		clk_info->vcodec_clk = devm_clk_get(&pdev->dev,
			clk_info->clk_name);
		if (IS_ERR(clk_info->vcodec_clk)) {
			dev_err(&pdev->dev, "devm_clk_get (%d)%s fail", i, clk_info->clk_name);
			return PTR_ERR(clk_info->vcodec_clk);
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(mtk_vcodec_init_dec_clk);

static int mtk_vcodec_dec_pw_on(struct mtk_vcodec_pm *pm)
{
	int ret;

	ret = pm_runtime_resume_and_get(pm->dev);
	if (ret)
		dev_err(pm->dev, "pm_runtime_resume_and_get fail %d", ret);

	return ret;
}

static void mtk_vcodec_dec_pw_off(struct mtk_vcodec_pm *pm)
{
	int ret;

	ret = pm_runtime_put(pm->dev);
	if (ret && ret != -EAGAIN)
		dev_err(pm->dev, "pm_runtime_put fail %d", ret);
}

static void mtk_vcodec_dec_clock_on(struct mtk_vcodec_pm *pm)
{
	struct mtk_vcodec_clk *dec_clk;
	int ret, i;

	dec_clk = &pm->vdec_clk;
	for (i = 0; i < dec_clk->clk_num; i++) {
		ret = clk_prepare_enable(dec_clk->clk_info[i].vcodec_clk);
		if (ret) {
			dev_err(pm->dev, "clk_prepare_enable %d %s fail %d", i,
				dec_clk->clk_info[i].clk_name, ret);
			goto error;
		}
	}

	return;
error:
	for (i -= 1; i >= 0; i--)
		clk_disable_unprepare(dec_clk->clk_info[i].vcodec_clk);
}

static void mtk_vcodec_dec_clock_off(struct mtk_vcodec_pm *pm)
{
	struct mtk_vcodec_clk *dec_clk;
	int i;

	dec_clk = &pm->vdec_clk;
	for (i = dec_clk->clk_num - 1; i >= 0; i--)
		clk_disable_unprepare(dec_clk->clk_info[i].vcodec_clk);
}

static void mtk_vcodec_dec_enable_irq(struct mtk_vcodec_dec_dev *vdec_dev, int hw_idx)
{
	struct mtk_vdec_hw_dev *subdev_dev;

	if (!test_bit(hw_idx, vdec_dev->subdev_bitmap))
		return;

	if (vdec_dev->vdec_pdata->is_subdev_supported) {
		subdev_dev = mtk_vcodec_get_hw_dev(vdec_dev, hw_idx);
		if (subdev_dev)
			enable_irq(subdev_dev->dec_irq);
		else
			dev_err(&vdec_dev->plat_dev->dev, "Failed to get hw dev\n");
	} else {
		enable_irq(vdec_dev->dec_irq);
	}
}

static void mtk_vcodec_dec_disable_irq(struct mtk_vcodec_dec_dev *vdec_dev, int hw_idx)
{
	struct mtk_vdec_hw_dev *subdev_dev;

	if (!test_bit(hw_idx, vdec_dev->subdev_bitmap))
		return;

	if (vdec_dev->vdec_pdata->is_subdev_supported) {
		subdev_dev = mtk_vcodec_get_hw_dev(vdec_dev, hw_idx);
		if (subdev_dev)
			disable_irq(subdev_dev->dec_irq);
		else
			dev_err(&vdec_dev->plat_dev->dev, "Failed to get hw dev\n");
	} else {
		disable_irq(vdec_dev->dec_irq);
	}
}

static void mtk_vcodec_load_racing_info(struct mtk_vcodec_dec_ctx *ctx)
{
	void __iomem *vdec_racing_addr;
	int j;

	mutex_lock(&ctx->dev->dec_racing_info_mutex);
	if (atomic_inc_return(&ctx->dev->dec_active_cnt) == 1) {
		vdec_racing_addr = ctx->dev->reg_base[VDEC_MISC] + 0x100;
		for (j = 0; j < 132; j++)
			writel(ctx->dev->vdec_racing_info[j], vdec_racing_addr + j * 4);
	}
	mutex_unlock(&ctx->dev->dec_racing_info_mutex);
}

static void mtk_vcodec_record_racing_info(struct mtk_vcodec_dec_ctx *ctx)
{
	void __iomem *vdec_racing_addr;
	int j;

	mutex_lock(&ctx->dev->dec_racing_info_mutex);
	if (atomic_dec_and_test(&ctx->dev->dec_active_cnt)) {
		vdec_racing_addr = ctx->dev->reg_base[VDEC_MISC] + 0x100;
		for (j = 0; j < 132; j++)
			ctx->dev->vdec_racing_info[j] = readl(vdec_racing_addr + j * 4);
	}
	mutex_unlock(&ctx->dev->dec_racing_info_mutex);
}

static struct mtk_vcodec_pm *mtk_vcodec_dec_get_pm(struct mtk_vcodec_dec_dev *vdec_dev,
						   int hw_idx)
{
	struct mtk_vdec_hw_dev *subdev_dev;

	if (!test_bit(hw_idx, vdec_dev->subdev_bitmap))
		return NULL;

	if (vdec_dev->vdec_pdata->is_subdev_supported) {
		subdev_dev = mtk_vcodec_get_hw_dev(vdec_dev, hw_idx);
		if (subdev_dev)
			return &subdev_dev->pm;

		dev_err(&vdec_dev->plat_dev->dev, "Failed to get hw dev\n");
		return NULL;
	}

	return &vdec_dev->pm;
}

static void mtk_vcodec_dec_child_dev_on(struct mtk_vcodec_dec_dev *vdec_dev,
					int hw_idx)
{
	struct mtk_vcodec_pm *pm;

	pm = mtk_vcodec_dec_get_pm(vdec_dev, hw_idx);
	if (pm) {
		mtk_vcodec_dec_pw_on(pm);
		mtk_vcodec_dec_clock_on(pm);
	}

	if (hw_idx == MTK_VDEC_LAT0) {
		pm = mtk_vcodec_dec_get_pm(vdec_dev, MTK_VDEC_LAT_SOC);
		if (pm) {
			mtk_vcodec_dec_pw_on(pm);
			mtk_vcodec_dec_clock_on(pm);
		}
	}
}

static void mtk_vcodec_dec_child_dev_off(struct mtk_vcodec_dec_dev *vdec_dev,
					 int hw_idx)
{
	struct mtk_vcodec_pm *pm;

	pm = mtk_vcodec_dec_get_pm(vdec_dev, hw_idx);
	if (pm) {
		mtk_vcodec_dec_clock_off(pm);
		mtk_vcodec_dec_pw_off(pm);
	}

	if (hw_idx == MTK_VDEC_LAT0) {
		pm = mtk_vcodec_dec_get_pm(vdec_dev, MTK_VDEC_LAT_SOC);
		if (pm) {
			mtk_vcodec_dec_clock_off(pm);
			mtk_vcodec_dec_pw_off(pm);
		}
	}
}

void mtk_vcodec_dec_enable_hardware(struct mtk_vcodec_dec_ctx *ctx, int hw_idx)
{
	int gic_irq = -1;
	bool gic_postfail = false;

	mutex_lock(&ctx->dev->dec_mutex[hw_idx]);

	/*
	 * The driver has been unbound underneath this file handle. Keep the
	 * lock pairing with disable_hardware() and do nothing else: the power
	 * domain, the clocks, the IRQ and the register windows this function
	 * would touch were all released by remove(), and reaching any of them
	 * is a use-after-free.
	 */
	if (READ_ONCE(ctx->dev->unplugged))
		return;

	/*
	 * GIC point e0: the controller's view while the block is still powered
	 * DOWN. Sampled before the postfail flag is consumed by the MISC probe
	 * below, so both points see the same decode classification.
	 */
	if (hw_idx == MTK_VDEC_CORE) {
		gic_irq = mtk_vdec_gic_irq(ctx->dev);
		gic_postfail = READ_ONCE(vdec_probe_fail_seen) != 0;
		if (gic_irq >= 0) {
			unsigned int b = MTK_VDEC_GIC_ACC(gic_irq, e0);

			if (gic_postfail) {
				vdec_gic_pf_n++;
				vdec_gic_pf_e0 = b;
				if (b & 1)
					vdec_gic_pf_e0pend++;
			}
		}
	}

	if (IS_VDEC_LAT_ARCH(ctx->dev->vdec_pdata->hw_arch) &&
	    hw_idx == MTK_VDEC_CORE)
		mtk_vcodec_dec_child_dev_on(ctx->dev, MTK_VDEC_LAT0);
	mtk_vcodec_dec_child_dev_on(ctx->dev, hw_idx);

	/*
	 * Probe point 2: the status the hardware is holding just before the
	 * line is unmasked. A DEC_SUCCESS here is the latched interrupt caught
	 * in the act -- it is about to be delivered inside enable_irq(), before
	 * mtk_vcodec_set_curr_ctx() publishes this context.
	 */
	if (hw_idx == MTK_VDEC_CORE) {
		void __iomem *misc = mtk_vdec_probe_misc(ctx->dev);

		if (misc) {
			int active = mtk_vdec_probe_active(ctx->dev);
			u32 pre = readl(misc);

			vdec_enable_last_active = active;
			if (active != 1)
				vdec_enable_idle++;
			vdec_enable_probe_n++;
			vdec_enable_last_pre = pre;
			if (pre & MTK_VDEC_IRQ_STATUS_DEC_SUCCESS) {
				vdec_enable_stale_succ++;
				vdec_enable_stale_val = pre;
			} else if (pre) {
				vdec_enable_nonzero++;
			}
			if (READ_ONCE(vdec_probe_fail_seen)) {
				WRITE_ONCE(vdec_probe_fail_seen, 0);
				vdec_enable_postfail_n++;
				vdec_enable_postfail_pre = pre;
				if (pre & MTK_VDEC_IRQ_STATUS_DEC_SUCCESS)
					vdec_enable_postfail_succ++;
			}
		}
	}

	/*
	 * GIC point e1: the last instant before the line is unmasked. A PENDING
	 * here is the latched interrupt caught at the controller -- it is about
	 * to be delivered inside enable_irq(), before mtk_vcodec_set_curr_ctx()
	 * publishes this context.
	 */
	if (hw_idx == MTK_VDEC_CORE && gic_irq >= 0) {
		unsigned int b = MTK_VDEC_GIC_ACC(gic_irq, e1);

		if (gic_postfail) {
			vdec_gic_pf_e1 = b;
			if (b & 1)
				vdec_gic_pf_e1pend++;
		}
	}

	mtk_vcodec_dec_enable_irq(ctx->dev, hw_idx);

	if (IS_VDEC_INNER_RACING(ctx->dev->dec_capability))
		mtk_vcodec_load_racing_info(ctx);
}
EXPORT_SYMBOL_GPL(mtk_vcodec_dec_enable_hardware);

void mtk_vcodec_dec_disable_hardware(struct mtk_vcodec_dec_ctx *ctx, int hw_idx)
{
	int gic_irq = -1;
	bool gic_failed = false;

	/* Unbound: release the lock enable_hardware() took, touch nothing. */
	if (READ_ONCE(ctx->dev->unplugged)) {
		mutex_unlock(&ctx->dev->dec_mutex[hw_idx]);
		return;
	}

	if (hw_idx == MTK_VDEC_CORE) {
		gic_irq = mtk_vdec_gic_irq(ctx->dev);
		gic_failed = READ_ONCE(vdec_probe_last_ret) != 0;
	}

	if (IS_VDEC_INNER_RACING(ctx->dev->dec_capability))
		mtk_vcodec_record_racing_info(ctx);

	/*
	 * Probe point 1: VDEC_MISC+0xa4 either side of the mask, and once more
	 * after a delay when the decode FAILED (the timeout path). The delayed
	 * read sleeps, so it is confined to the failing decode -- the shared
	 * decode path keeps its timing.
	 */
	if (hw_idx == MTK_VDEC_CORE) {
		void __iomem *misc = mtk_vdec_probe_misc(ctx->dev);
		int failed = READ_ONCE(vdec_probe_last_ret) != 0;
		u32 pre = 0, post = 0, dly = 0;
		int active = -1;

		if (misc) {
			active = mtk_vdec_probe_active(ctx->dev);
			vdec_mask_last_active = active;
			if (active != 1)
				vdec_mask_idle++;
			pre = readl(misc);
			vdec_mask_probe_n++;
			if (failed) {
				vdec_mask_fail_n++;
				WRITE_ONCE(vdec_probe_fail_seen, 1);
			}
		}

		/* GIC point m0: controller state before the driver masks */
		if (gic_irq >= 0) {
			unsigned int b = MTK_VDEC_GIC_ACC(gic_irq, m0);

			if (gic_failed) {
				vdec_gic_fail_n++;
				vdec_gic_fail_m0 = b;
			}
		}

		mtk_vcodec_dec_disable_irq(ctx->dev, hw_idx);

		/*
		 * GIC point m1: immediately after disable_irq(). genirq disables
		 * LAZILY when the chip has no ->irq_disable (the GIC has none), so
		 * a cleared "masked" bit here means the line is still ENABLED at
		 * the distributor while the driver believes it masked.
		 */
		if (gic_irq >= 0) {
			unsigned int b = MTK_VDEC_GIC_ACC(gic_irq, m1);

			if (gic_failed)
				vdec_gic_fail_m1 = b;
		}

		if (misc) {
			post = readl(misc);
			dly = post;
			if (failed && vdec_probe_dly_us > 0) {
				usleep_range(vdec_probe_dly_us,
					     vdec_probe_dly_us + 1000);
				dly = readl(misc);
			}

			if (pre & MTK_VDEC_IRQ_STATUS_DEC_SUCCESS)
				vdec_mask_pre_succ++;
			else if (post & MTK_VDEC_IRQ_STATUS_DEC_SUCCESS)
				vdec_mask_post_succ++;
			else if (dly & MTK_VDEC_IRQ_STATUS_DEC_SUCCESS)
				vdec_mask_dly_succ++;

			vdec_mask_last_pre = pre;
			vdec_mask_last_post = post;
			vdec_mask_last_dly = dly;
			if (failed) {
				vdec_mask_fail_pre = pre;
				vdec_mask_fail_post = post;
				vdec_mask_fail_dly = dly;
			}
		}
		WRITE_ONCE(vdec_probe_last_ret, 0);
	} else {
		mtk_vcodec_dec_disable_irq(ctx->dev, hw_idx);
	}

	mtk_vcodec_dec_child_dev_off(ctx->dev, hw_idx);
	if (IS_VDEC_LAT_ARCH(ctx->dev->vdec_pdata->hw_arch) &&
	    hw_idx == MTK_VDEC_CORE)
		mtk_vcodec_dec_child_dev_off(ctx->dev, MTK_VDEC_LAT0);

	/*
	 * GIC point m2: clocks and power domain are DOWN. This is the first
	 * read any round has been able to take outside the block's lifecycle;
	 * a PENDING here is a latched or asserted line that no VDEC_MISC read
	 * could ever have seen.
	 */
	if (gic_irq >= 0) {
		unsigned int b = MTK_VDEC_GIC_ACC(gic_irq, m2);

		if (gic_failed)
			vdec_gic_fail_m2 = b;
	}

	mutex_unlock(&ctx->dev->dec_mutex[hw_idx]);
}
EXPORT_SYMBOL_GPL(mtk_vcodec_dec_disable_hardware);
