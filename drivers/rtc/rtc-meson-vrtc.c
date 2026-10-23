// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 * Copyright (C) 2015 Amlogic, Inc. All rights reserved.
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/io.h>
#include <linux/irq_work.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/pvclock_gtod.h>
#include <linux/time64.h>
#include <linux/timekeeping.h>
#include <linux/workqueue.h>

/*
 * Kept time (optional, DT "amlogic,keep-sram" = <&sram-partition offset>): four
 * words in share SRAM that the SoC keeps across a WARM reset but not a power
 * cut. On the libre firmware chain the open SC reserves them and never writes
 * them. Layout: magic, epoch low, epoch high, check = ~(magic ^ low ^ high).
 */
#define MESON_VRTC_KEEP_MAGIC		0x4b545256	/* "VRTK" */
#define MESON_VRTC_KEEP_WORD_MAGIC	0x0
#define MESON_VRTC_KEEP_WORD_LO		0x4
#define MESON_VRTC_KEEP_WORD_HI		0x8
#define MESON_VRTC_KEEP_WORD_CHECK	0xc
#define MESON_VRTC_KEEP_SIZE		0x10
#define MESON_VRTC_KEEP_PERIOD		(60 * HZ)

struct meson_vrtc_data {
	void __iomem *io_alarm;
	unsigned long alarm_time;
	bool enabled;

	void __iomem *io_keep;
	bool keep_valid;
	time64_t keep_base;	/* kept epoch at keep_boot */
	time64_t keep_boot;	/* ktime_get_boottime_seconds() then */
	ktime_t wall_offset;	/* meson_vrtc_wall_offset() at probe */
	bool clock_set;		/* the system clock has been set since probe */
	struct delayed_work keep_work;
	struct irq_work set_irq_work;	/* a clock set: store now */
	struct notifier_block set_nb;
};

/* The kept clock: the stored epoch advanced by the time this boot has run. */
static time64_t meson_vrtc_kept_time(struct meson_vrtc_data *vrtc)
{
	return vrtc->keep_base + (ktime_get_boottime_seconds() - vrtc->keep_boot);
}

/*
 * Realtime minus boottime, exactly: it moves only when the system clock is SET
 * (settimeofday, clock_settime, an adjtimex step). NTP slewing and suspend move
 * both clocks together.
 */
static ktime_t meson_vrtc_wall_offset(void)
{
	return ktime_sub(ktime_mono_to_any(0, TK_OFFS_REAL),
			 ktime_mono_to_any(0, TK_OFFS_BOOT));
}

/*
 * The kept clock only bridges the boot. Once anything sets the system clock (a
 * battery RTC, userspace restoring a saved time, NTP), the system clock is the
 * answer, including when it moved BACK from a kept time that was ahead.
 */
static bool meson_vrtc_bridging(struct meson_vrtc_data *vrtc)
{
	if (!vrtc->clock_set && meson_vrtc_wall_offset() != vrtc->wall_offset)
		vrtc->clock_set = true;
	return vrtc->keep_valid && !vrtc->clock_set;
}

/* The best time this driver knows: the system clock, unless the kept clock bridges ahead of it. */
static time64_t meson_vrtc_now(struct meson_vrtc_data *vrtc)
{
	time64_t now = ktime_get_real_seconds();

	if (meson_vrtc_bridging(vrtc) && now < meson_vrtc_kept_time(vrtc))
		now = meson_vrtc_kept_time(vrtc);
	return now;
}

static void meson_vrtc_keep_store(struct meson_vrtc_data *vrtc, time64_t t)
{
	u32 lo = lower_32_bits(t), hi = upper_32_bits(t);

	if (!vrtc->io_keep)
		return;
	/* Invalidate, write, then re-validate: a reset mid-store leaves no magic. */
	writel(0, vrtc->io_keep + MESON_VRTC_KEEP_WORD_MAGIC);
	writel(lo, vrtc->io_keep + MESON_VRTC_KEEP_WORD_LO);
	writel(hi, vrtc->io_keep + MESON_VRTC_KEEP_WORD_HI);
	writel(~(MESON_VRTC_KEEP_MAGIC ^ lo ^ hi), vrtc->io_keep + MESON_VRTC_KEEP_WORD_CHECK);
	writel(MESON_VRTC_KEEP_MAGIC, vrtc->io_keep + MESON_VRTC_KEEP_WORD_MAGIC);
}

static void meson_vrtc_keep_work(struct work_struct *work)
{
	struct meson_vrtc_data *vrtc = container_of(to_delayed_work(work),
						    struct meson_vrtc_data, keep_work);

	meson_vrtc_keep_store(vrtc, meson_vrtc_now(vrtc));
	schedule_delayed_work(&vrtc->keep_work, MESON_VRTC_KEEP_PERIOD);
}

/* A clock set stores at once instead of up to a period later. */
static void meson_vrtc_set_irq_work(struct irq_work *work)
{
	struct meson_vrtc_data *vrtc = container_of(work, struct meson_vrtc_data,
						    set_irq_work);

	mod_delayed_work(system_wq, &vrtc->keep_work, 0);
}

/*
 * Called on every timekeeping update with the timekeeper write-locked, so it
 * must not read the time: it only queues the irq_work.
 */
static int meson_vrtc_clock_notify(struct notifier_block *nb, unsigned long was_set,
				   void *priv)
{
	struct meson_vrtc_data *vrtc = container_of(nb, struct meson_vrtc_data, set_nb);

	if (was_set)
		irq_work_queue(&vrtc->set_irq_work);
	return NOTIFY_OK;
}

static void meson_vrtc_keep_stop(void *data)
{
	struct meson_vrtc_data *vrtc = data;

	pvclock_gtod_unregister_notifier(&vrtc->set_nb);
	irq_work_sync(&vrtc->set_irq_work);
	cancel_delayed_work_sync(&vrtc->keep_work);
}

static int meson_vrtc_keep_init(struct device *dev, struct meson_vrtc_data *vrtc)
{
	struct of_phandle_args args;
	struct resource res;
	u32 magic, lo, hi, check;
	int ret;

	ret = of_parse_phandle_with_fixed_args(dev->of_node, "amlogic,keep-sram", 1, 0, &args);
	if (ret)
		return 0;	/* optional */
	ret = of_address_to_resource(args.np, 0, &res);
	of_node_put(args.np);
	if (ret)
		return ret;
	if (args.args[0] + MESON_VRTC_KEEP_SIZE > resource_size(&res))
		return -EINVAL;

	/* Not requested: the SRAM partition's own driver already maps it. */
	vrtc->io_keep = devm_ioremap(dev, res.start + args.args[0], MESON_VRTC_KEEP_SIZE);
	if (!vrtc->io_keep)
		return -ENOMEM;

	vrtc->wall_offset = meson_vrtc_wall_offset();
	magic = readl(vrtc->io_keep + MESON_VRTC_KEEP_WORD_MAGIC);
	lo = readl(vrtc->io_keep + MESON_VRTC_KEEP_WORD_LO);
	hi = readl(vrtc->io_keep + MESON_VRTC_KEEP_WORD_HI);
	check = readl(vrtc->io_keep + MESON_VRTC_KEEP_WORD_CHECK);
	if (magic == MESON_VRTC_KEEP_MAGIC && check == ~(magic ^ lo ^ hi)) {
		vrtc->keep_base = ((time64_t)hi << 32) | lo;
		vrtc->keep_boot = ktime_get_boottime_seconds();
		vrtc->keep_valid = true;
		dev_info(dev, "kept time %lld\n", vrtc->keep_base);
	}

	INIT_DELAYED_WORK(&vrtc->keep_work, meson_vrtc_keep_work);
	init_irq_work(&vrtc->set_irq_work, meson_vrtc_set_irq_work);
	vrtc->set_nb.notifier_call = meson_vrtc_clock_notify;
	schedule_delayed_work(&vrtc->keep_work, MESON_VRTC_KEEP_PERIOD);
	/* Registering reports a set at once, so the record is also stored at probe. */
	ret = pvclock_gtod_register_notifier(&vrtc->set_nb);
	if (ret) {
		cancel_delayed_work_sync(&vrtc->keep_work);
		return ret;
	}
	/* Also undoes the above if a later probe step fails. */
	return devm_add_action_or_reset(dev, meson_vrtc_keep_stop, vrtc);
}

static int meson_vrtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct meson_vrtc_data *vrtc = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);
	rtc_time64_to_tm(meson_vrtc_now(vrtc), tm);

	return 0;
}

static int meson_vrtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct meson_vrtc_data *vrtc = dev_get_drvdata(dev);
	time64_t t = rtc_tm_to_time64(tm);

	meson_vrtc_keep_store(vrtc, t);
	vrtc->keep_base = t;
	vrtc->keep_boot = ktime_get_boottime_seconds();
	vrtc->keep_valid = true;

	return 0;
}

static void meson_vrtc_set_wakeup_time(struct meson_vrtc_data *vrtc,
				       unsigned long time)
{
	writel_relaxed(time, vrtc->io_alarm);
}

static int meson_vrtc_set_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	struct meson_vrtc_data *vrtc = dev_get_drvdata(dev);

	dev_dbg(dev, "%s: alarm->enabled=%d\n", __func__, alarm->enabled);
	if (alarm->enabled)
		vrtc->alarm_time = rtc_tm_to_time64(&alarm->time);
	else
		vrtc->alarm_time = 0;

	return 0;
}

static int meson_vrtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct meson_vrtc_data *vrtc = dev_get_drvdata(dev);

	vrtc->enabled = enabled;
	return 0;
}

static const struct rtc_class_ops meson_vrtc_ops = {
	.read_time = meson_vrtc_read_time,
	.set_alarm = meson_vrtc_set_alarm,
	.alarm_irq_enable = meson_vrtc_alarm_irq_enable,
};

/* With kept time the clock can also be set (hwclock -w, RTC_SYSTOHC). */
static const struct rtc_class_ops meson_vrtc_keep_ops = {
	.read_time = meson_vrtc_read_time,
	.set_time = meson_vrtc_set_time,
	.set_alarm = meson_vrtc_set_alarm,
	.alarm_irq_enable = meson_vrtc_alarm_irq_enable,
};

static int meson_vrtc_probe(struct platform_device *pdev)
{
	struct meson_vrtc_data *vrtc;
	struct rtc_device *rtc;
	int ret;

	vrtc = devm_kzalloc(&pdev->dev, sizeof(*vrtc), GFP_KERNEL);
	if (!vrtc)
		return -ENOMEM;

	vrtc->io_alarm = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(vrtc->io_alarm))
		return PTR_ERR(vrtc->io_alarm);

	ret = meson_vrtc_keep_init(&pdev->dev, vrtc);
	if (ret)
		dev_warn(&pdev->dev, "kept time unavailable: %d\n", ret);

	device_init_wakeup(&pdev->dev, true);

	platform_set_drvdata(pdev, vrtc);

	rtc = devm_rtc_allocate_device(&pdev->dev);
	if (IS_ERR(rtc))
		return PTR_ERR(rtc);

	rtc->ops = vrtc->io_keep ? &meson_vrtc_keep_ops : &meson_vrtc_ops;
	return devm_rtc_register_device(rtc);
}

static void meson_vrtc_shutdown(struct platform_device *pdev)
{
	struct meson_vrtc_data *vrtc = platform_get_drvdata(pdev);

	if (vrtc->io_keep)
		meson_vrtc_keep_store(vrtc, meson_vrtc_now(vrtc));
}

static int __maybe_unused meson_vrtc_suspend(struct device *dev)
{
	struct meson_vrtc_data *vrtc = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);
	if (vrtc->io_keep)
		meson_vrtc_keep_store(vrtc, meson_vrtc_now(vrtc));
	if (vrtc->alarm_time) {
		unsigned long local_time;
		long alarm_secs;
		struct timespec64 time;

		ktime_get_real_ts64(&time);
		local_time = time.tv_sec;

		dev_dbg(dev, "alarm_time = %lus, local_time=%lus\n",
			vrtc->alarm_time, local_time);
		alarm_secs = vrtc->alarm_time - local_time;
		if (alarm_secs > 0) {
			meson_vrtc_set_wakeup_time(vrtc, alarm_secs);
			dev_dbg(dev, "system will wakeup in %lds.\n",
				alarm_secs);
		} else {
			dev_err(dev, "alarm time already passed: %lds.\n",
				alarm_secs);
		}
	}

	return 0;
}

static int __maybe_unused meson_vrtc_resume(struct device *dev)
{
	struct meson_vrtc_data *vrtc = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);

	vrtc->alarm_time = 0;
	meson_vrtc_set_wakeup_time(vrtc, 0);
	return 0;
}

static SIMPLE_DEV_PM_OPS(meson_vrtc_pm_ops,
			 meson_vrtc_suspend, meson_vrtc_resume);

static const struct of_device_id meson_vrtc_dt_match[] = {
	{ .compatible = "amlogic,meson-vrtc"},
	{},
};
MODULE_DEVICE_TABLE(of, meson_vrtc_dt_match);

static struct platform_driver meson_vrtc_driver = {
	.probe = meson_vrtc_probe,
	.shutdown = meson_vrtc_shutdown,
	.driver = {
		.name = "meson-vrtc",
		.of_match_table = meson_vrtc_dt_match,
		.pm = &meson_vrtc_pm_ops,
	},
};

module_platform_driver(meson_vrtc_driver);

MODULE_DESCRIPTION("Amlogic Virtual Wakeup RTC Timer driver");
MODULE_LICENSE("GPL");
