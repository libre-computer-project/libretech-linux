/*
 * drivers/rtc/rtc-meson-vrtc.c
 *
 * Copyright (C) 2016 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 * Copyright (C) 2015 Amlogic, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/pm_wakeup.h>
#include <linux/time64.h>

struct meson_vrtc_data {
	struct platform_device *pdev;
	void __iomem *io_alarm;
	struct rtc_device *rtc;
	unsigned long alarm_time;
};

static int meson_vrtc_read_time(struct device *dev, struct rtc_time *tm)
{
	unsigned long local_time;
	struct timeval time;

	do_gettimeofday(&time);
	local_time = time.tv_sec - (sys_tz.tz_minuteswest * 60);
	rtc_time_to_tm(local_time, tm);

	return 0;
}

static void meson_vrtc_set_wakeup_time(struct meson_vrtc_data *vrtc,
				       unsigned long time)
{
	writel_relaxed(time, vrtc->io_alarm);

	dev_dbg(&vrtc->pdev->dev, "set_wakeup_time: %lu\n", time);
}

static int meson_vrtc_set_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	struct meson_vrtc_data *vrtc = dev_get_drvdata(dev);
	struct timeval time;
	unsigned long local_time;
	unsigned long alarm_secs;
	int ret;

	if (alarm->enabled) {
		ret = rtc_tm_to_time(&alarm->time, &alarm_secs);
		if (ret)
			return ret;

		do_gettimeofday(&time);
		local_time = time.tv_sec - (sys_tz.tz_minuteswest * 60);

		vrtc->alarm_time = alarm_secs;

		if (alarm_secs >= local_time) {
			alarm_secs = alarm_secs - local_time;

			meson_vrtc_set_wakeup_time(vrtc, alarm_secs);

			pr_debug("system will wakeup %lus later\n", alarm_secs);
		}
	} else {
		vrtc->alarm_time = 0;
		meson_vrtc_set_wakeup_time(vrtc, 0);
	}

	return 0;
}

static int meson_vrtc_read_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	struct meson_vrtc_data *vrtc = dev_get_drvdata(dev);

	if (!vrtc->alarm_time) {
		alm->enabled = true;

		rtc_time_to_tm(vrtc->alarm_time, &alm->time);
	}

	return 0;
}

static const struct rtc_class_ops meson_vrtc_ops = {
	.read_time = meson_vrtc_read_time,
	.set_alarm = meson_vrtc_set_alarm,
	.read_alarm = meson_vrtc_read_alarm,
};

static int meson_vrtc_probe(struct platform_device *pdev)
{
	struct meson_vrtc_data *vrtc;
	struct resource *res;

	vrtc = devm_kzalloc(&pdev->dev, sizeof(*vrtc), GFP_KERNEL);
	if (!vrtc)
		return -ENOMEM;

	vrtc->pdev = pdev;

	/* Alarm registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	vrtc->io_alarm = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(vrtc->io_alarm))
		return PTR_ERR(vrtc->io_alarm);

	device_init_wakeup(&pdev->dev, 1);

	platform_set_drvdata(pdev, vrtc);

	vrtc->rtc = devm_rtc_device_register(&pdev->dev, "meson-vrtc",
			&meson_vrtc_ops, THIS_MODULE);
	if (IS_ERR(vrtc->rtc))
		return PTR_ERR(vrtc->rtc);

	return 0;
}

int meson_vrtc_resume(struct platform_device *pdev)
{
	struct meson_vrtc_data *vrtc = platform_get_drvdata(pdev);

	meson_vrtc_set_wakeup_time(vrtc, 0);

	return 0;
}

static const struct of_device_id meson_vrtc_dt_match[] = {
	{ .compatible = "amlogic,meson-vrtc"},
	{},
};
MODULE_DEVICE_TABLE(of, meson_vrtc_dt_match);

struct platform_driver meson_vrtc_driver = {
	.driver = {
		.name = "meson-vrtc",
		.of_match_table = meson_vrtc_dt_match,
	},
	.probe = meson_vrtc_probe,
	.resume = meson_vrtc_resume,
};

module_platform_driver(meson_vrtc_driver);

MODULE_DESCRIPTION("Amlogic Virtual Wakeup RTC Timer driver");
MODULE_LICENSE("GPL");
