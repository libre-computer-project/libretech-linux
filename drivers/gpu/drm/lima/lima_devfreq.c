// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2020 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * Based on panfrost_devfreq.c:
 *   Copyright 2019 Collabora ltd.
 */
#include <linux/clk.h>
#include <linux/devfreq.h>
#include <linux/devfreq_cooling.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/property.h>
#include <linux/units.h>

#include "lima_device.h"
#include "lima_sched.h"
#include "lima_devfreq.h"

static void lima_devfreq_update_utilization(struct lima_devfreq *devfreq)
{
	ktime_t now, last;

	now = ktime_get();
	last = devfreq->time_last_update;

	if (devfreq->busy_count > 0)
		devfreq->busy_time += ktime_sub(now, last);
	else
		devfreq->idle_time += ktime_sub(now, last);

	devfreq->time_last_update = now;
}

static int lima_devfreq_target(struct device *dev, unsigned long *freq,
			       u32 flags)
{
	struct lima_device *ldev = dev_get_drvdata(dev);
	struct dev_pm_opp *opp;
	int i, ret;

	opp = devfreq_recommended_opp(dev, freq, flags);
	if (IS_ERR(opp))
		return PTR_ERR(opp);
	dev_pm_opp_put(opp);

	/*
	 * The Mali-450 hardware requires all GP/PP groups to be idle
	 * before changing the clock frequency. The ARM BSP enforces
	 * this via mali_dev_pause()/mali_dev_resume(). Without this,
	 * frequency changes under GPU load cause hard lockups.
	 *
	 * Stop the scheduler work queues to prevent new job submission,
	 * then wait for in-flight jobs to drain before changing the
	 * clock. This is the lima equivalent of mali_dev_pause().
	 */
	for (i = 0; i < lima_pipe_num; i++)
		drm_sched_wqueue_stop(&ldev->pipe[i].base);

	/* Wait for in-flight jobs to complete */
	for (i = 0; i < lima_pipe_num; i++) {
		int timeout = 100; /* 100ms max wait */

		while (atomic_read(&ldev->pipe[i].base.credit_count) &&
		       timeout-- > 0)
			usleep_range(1000, 2000);
	}

	ret = dev_pm_opp_set_rate(dev, *freq);

	for (i = 0; i < lima_pipe_num; i++)
		drm_sched_wqueue_start(&ldev->pipe[i].base);

	return ret;
}

static void lima_devfreq_reset(struct lima_devfreq *devfreq)
{
	devfreq->busy_time = 0;
	devfreq->idle_time = 0;
	devfreq->time_last_update = ktime_get();
}

static int lima_devfreq_get_dev_status(struct device *dev,
				       struct devfreq_dev_status *status)
{
	struct lima_device *ldev = dev_get_drvdata(dev);
	struct lima_devfreq *devfreq = &ldev->devfreq;
	unsigned long irqflags;

	status->current_frequency = clk_get_rate(ldev->clk_gpu);

	spin_lock_irqsave(&devfreq->lock, irqflags);

	lima_devfreq_update_utilization(devfreq);

	status->total_time = ktime_to_ns(ktime_add(devfreq->busy_time,
						   devfreq->idle_time));
	status->busy_time = ktime_to_ns(devfreq->busy_time);

	lima_devfreq_reset(devfreq);

	spin_unlock_irqrestore(&devfreq->lock, irqflags);

	dev_dbg(ldev->dev, "busy %lu total %lu %lu %% freq %lu MHz\n",
		status->busy_time, status->total_time,
		status->busy_time / (status->total_time / 100),
		status->current_frequency / 1000 / 1000);

	return 0;
}

static struct devfreq_dev_profile lima_devfreq_profile = {
	.timer = DEVFREQ_TIMER_DELAYED,
	.polling_ms = 50, /* ~3 frames */
	.target = lima_devfreq_target,
	.get_dev_status = lima_devfreq_get_dev_status,
};

void lima_devfreq_fini(struct lima_device *ldev)
{
	struct lima_devfreq *devfreq = &ldev->devfreq;

	if (devfreq->cooling) {
		devfreq_cooling_unregister(devfreq->cooling);
		devfreq->cooling = NULL;
	}

	if (devfreq->devfreq) {
		devm_devfreq_remove_device(ldev->dev, devfreq->devfreq);
		devfreq->devfreq = NULL;
	}
}

/*
 * Does the GPU's OPP table carry voltages?  The OPP core refuses a table that
 * manages a regulator but has no opp-microvolt ("opp-microvolt missing although
 * OPP managing regulators", -EINVAL), which fails the whole probe.  Old
 * bootloader DTs exist that name a mali-supply (a fixed rail) next to a
 * frequency-only table; on those the regulator has nothing to scale, so the
 * table is loaded without it.  Only the first available OPP is read, matching
 * the OPP core's own check.
 */
static bool lima_opp_table_has_microvolt(struct device *dev)
{
	struct device_node *table __free(device_node) =
		of_parse_phandle(dev->of_node, "operating-points-v2", 0);
	struct property *prop;

	if (!table)
		return false;

	struct device_node *opp __free(device_node) =
		of_get_next_available_child(table, NULL);
	if (!opp)
		return false;

	/* "opp-microvolt" or a named "opp-microvolt-<name>" variant */
	for_each_property_of_node(opp, prop)
		if (!strncmp(prop->name, "opp-microvolt", strlen("opp-microvolt")))
			return true;

	return false;
}

/*
 * Amlogic GXL per-SoC default GPU frequency cap in Hz, 0 for none.  See the
 * comment at its use in lima_devfreq_init() for the per-die characterisation.
 */
static unsigned long lima_devfreq_default_cap_hz(void)
{
	if (of_machine_is_compatible("amlogic,s905x"))
		return 744000000;
	if (of_machine_is_compatible("amlogic,meson-gxl"))
		return 666666666;
	return 0;
}

int lima_devfreq_init(struct lima_device *ldev)
{
	struct thermal_cooling_device *cooling;
	struct device *dev = ldev->dev;
	struct devfreq *devfreq;
	struct lima_devfreq *ldevfreq = &ldev->devfreq;
	struct dev_pm_opp *opp;
	unsigned long cur_freq;
	int ret;
	const char *regulator_names[] = { "mali", NULL };

	if (!device_property_present(dev, "operating-points-v2"))
		/* Optional, continue without devfreq */
		return 0;

	spin_lock_init(&ldevfreq->lock);

	/*
	 * clkname is set separately so it is not affected by the optional
	 * regulator setting which may return error.
	 */
	ret = devm_pm_opp_set_clkname(dev, "core");
	if (ret)
		return ret;

	if (lima_opp_table_has_microvolt(dev)) {
		ret = devm_pm_opp_set_regulators(dev, regulator_names);
		if (ret) {
			/* Continue if the optional regulator is missing */
			if (ret != -ENODEV)
				return ret;
		}
	} else if (device_property_present(dev, "mali-supply")) {
		dev_info(dev, "OPP table has no opp-microvolt, mali-supply left unmanaged (old DTB)\n");
	}

	ret = devm_pm_opp_of_add_table(dev);
	if (ret)
		return ret;

	lima_devfreq_reset(ldevfreq);

	cur_freq = clk_get_rate(ldev->clk_gpu);

	opp = devfreq_recommended_opp(dev, &cur_freq, 0);
	if (IS_ERR(opp))
		return PTR_ERR(opp);

	/*
	 * If the actual GPU clock is far below the minimum OPP frequency,
	 * the clock parent chain is not configured for DVFS.  This happens
	 * with old DTBs that have operating-points-v2 but lack
	 * assigned-clocks for the GPU (leaving the clock on the 24 MHz
	 * crystal oscillator).  Repeated devfreq clock changes in this
	 * state reparent the Mali clock to FCLK_DIV4 without the full
	 * clock-tree setup that assigned-clocks provides; on Amlogic GXL
	 * this leaves the GPU APB bus inaccessible at suspend time,
	 * causing a synchronous external abort in lima_pmu_hw_fini.
	 *
	 * Set the GPU to the maximum OPP frequency once so the hardware
	 * runs at full speed, but skip devfreq entirely.  The per-SoC
	 * default cap applies here too: an old S805X table tops out at
	 * 744 MHz, above that die's 666 MHz cap.
	 */
	if (clk_get_rate(ldev->clk_gpu) * 2 < cur_freq) {
		unsigned long max_freq = lima_devfreq_default_cap_hz() ?: ULONG_MAX;
		struct dev_pm_opp *max_opp;
		unsigned long safe_freq;

		dev_pm_opp_put(opp);
		max_opp = dev_pm_opp_find_freq_floor(dev, &max_freq);
		if (IS_ERR(max_opp))
			max_freq = cur_freq;
		else
			dev_pm_opp_put(max_opp);

		safe_freq = clk_round_rate(ldev->clk_gpu, max_freq);
		if (safe_freq > 0 && !clk_set_rate(ldev->clk_gpu, safe_freq))
			dev_info(dev, "GPU clock set to %lu Hz (no devfreq, old DTB)\n",
				 safe_freq);
		else
			dev_info(dev, "GPU clock at %lu Hz (no devfreq, old DTB)\n",
				 clk_get_rate(ldev->clk_gpu));
		return 0;
	}

	lima_devfreq_profile.initial_freq = cur_freq;
	dev_pm_opp_put(opp);

	/*
	 * Setup default thresholds for the simple_ondemand governor.
	 * The values are chosen based on experiments.
	 */
	ldevfreq->gov_data.upthreshold = 50;
	ldevfreq->gov_data.downdifferential = 10;

	devfreq = devm_devfreq_add_device(dev, &lima_devfreq_profile,
					  DEVFREQ_GOV_SIMPLE_ONDEMAND,
					  &ldevfreq->gov_data);
	if (IS_ERR(devfreq)) {
		dev_err(dev, "Couldn't initialize GPU devfreq\n");
		return PTR_ERR(devfreq);
	}

	ldevfreq->devfreq = devfreq;

	cooling = of_devfreq_cooling_register(dev->of_node, devfreq);
	if (IS_ERR(cooling))
		dev_info(dev, "Failed to register cooling device\n");
	else
		ldevfreq->cooling = cooling;

	/*
	 * Amlogic GXL per-SoC default cap.
	 *
	 * S805X die (`amlogic,s805x`): cap 666 MHz. Runtime-stable
	 * ceiling on the OD=1 gp0_pll path is 708 MHz; 720 MHz under load
	 * panics with SError 0x96000210 at lima_l2_cache_flush. 666 MHz is
	 * fclk_div3/1 -- no PLL, no relock cost, safely below the per-die
	 * panic threshold. Cap matches the highest stock OPP in
	 * meson-gxl-s805x.dtsi.
	 *
	 * S905X die (`amlogic,s905x`): cap 744 MHz. Runtime-stable ceiling
	 * is 828 MHz (M=69 OD=1 DCO=1656) -- significantly higher than
	 * S805X. 744 MHz (M=62 OD=1 DCO=1488) is the highest historical
	 * "stock" OPP from upstream meson-gxl.dtsi, well below the
	 * per-die ceiling, and a clean reproducible path on every S905X
	 * die. Cap matches the highest stock OPP in meson-gxl-s905x.dtsi
	 * (792 MHz removed at the SoC level).
	 *
	 * Other GXL SoCs (`amlogic,meson-gxl` fallback): cap 666 MHz.
	 * S905D and S912 dies have not been characterised under the
	 * canonical methodology (gp0-pll-dco-ceiling-per-die.md); fall
	 * back to the most conservative per-SoC value until per-SoC data
	 * exists.
	 *
	 * OC OPPs (1008/1200/1392/1584 etc) remain in some boards' OPP
	 * tables; the default cap keeps the governor away from them. To
	 * opt into OC from userspace:
	 *     echo 1584000000 > /sys/class/devfreq/d00c0000.gpu/max_freq
	 * To restore the default cap:
	 *     echo  666000000 > /sys/class/devfreq/d00c0000.gpu/max_freq   (S805X)
	 *     echo  744000000 > /sys/class/devfreq/d00c0000.gpu/max_freq   (S905X)
	 *
	 * Mechanism: update devfreq's `user_max_freq_req`. PM QoS
	 * frequencies are in kHz; 666666666 Hz rounded up == 666667 kHz,
	 * 744000000 Hz == 744000 kHz.
	 */
	if (lima_devfreq_default_cap_hz())
		dev_pm_qos_update_request(&devfreq->user_max_freq_req,
					  DIV_ROUND_UP(lima_devfreq_default_cap_hz(),
						       HZ_PER_KHZ));

	return 0;
}

void lima_devfreq_record_busy(struct lima_devfreq *devfreq)
{
	unsigned long irqflags;

	if (!devfreq->devfreq)
		return;

	spin_lock_irqsave(&devfreq->lock, irqflags);

	lima_devfreq_update_utilization(devfreq);

	devfreq->busy_count++;

	spin_unlock_irqrestore(&devfreq->lock, irqflags);
}

void lima_devfreq_record_idle(struct lima_devfreq *devfreq)
{
	unsigned long irqflags;

	if (!devfreq->devfreq)
		return;

	spin_lock_irqsave(&devfreq->lock, irqflags);

	lima_devfreq_update_utilization(devfreq);

	WARN_ON(--devfreq->busy_count < 0);

	spin_unlock_irqrestore(&devfreq->lock, irqflags);
}

int lima_devfreq_resume(struct lima_devfreq *devfreq)
{
	unsigned long irqflags;

	if (!devfreq->devfreq)
		return 0;

	spin_lock_irqsave(&devfreq->lock, irqflags);

	lima_devfreq_reset(devfreq);

	spin_unlock_irqrestore(&devfreq->lock, irqflags);

	return devfreq_resume_device(devfreq->devfreq);
}

int lima_devfreq_suspend(struct lima_devfreq *devfreq)
{
	if (!devfreq->devfreq)
		return 0;

	return devfreq_suspend_device(devfreq->devfreq);
}
