// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2014, Fuzhou Rockchip Electronics Co., Ltd
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/hw_bitfield.h>
#include <linux/mmc/host.h>
#include <linux/of_address.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

#include "dw_mmc.h"
#include "dw_mmc-pltfm.h"

#define RK3288_CLKGEN_DIV		2
#define SDMMC_TIMING_CON0		0x130
#define SDMMC_TIMING_CON1		0x134
#define SDMMC_MISC_CON			0x138
#define MEM_CLK_AUTOGATE_ENABLE		BIT(5)
#define ROCKCHIP_MMC_DELAY_SEL		BIT(10)
#define ROCKCHIP_MMC_DEGREE_MASK	0x3
#define ROCKCHIP_MMC_DEGREE_OFFSET	1
#define ROCKCHIP_MMC_DELAYNUM_OFFSET	2
#define ROCKCHIP_MMC_DELAYNUM_MASK	(0xff << ROCKCHIP_MMC_DELAYNUM_OFFSET)
#define ROCKCHIP_MMC_DELAY_ELEMENT_PSEC	60

static const unsigned int freqs[] = { 100000, 200000, 300000, 400000 };

/*
 * Card-clock fallback ladder for the tuned modes (SDR104 / HS200 and, where
 * the card asks for it, SDR50 / DDR50).
 *
 * The top rung is whatever the core asks for -- the DT max-frequency clamps
 * it through f_max -- and the rungs below are the SD spec's own bands: the
 * 200 MHz SDR104 rate this clock tree produces exactly (CPLL / 2 / 2), the
 * 150 MHz Rockchip integration ceiling (rk3399-base.dtsi), SDR50's 100 MHz
 * and SDR25's 50 MHz. Fixed rates rather than ratios of the top rung because
 * the CCF picks the parent + divider closest BELOW a request, so a ratio
 * lands on whatever the tree happens to round to, while these are rates the
 * card is specified to run at. Two things step the ladder down, both logged
 * on one line and both re-tuning at the new rung:
 *
 *  (a) at tuning time, a passing sample-phase window narrower than
 *      tuning_min_window degrees -- the eye is what the sweep measures, so
 *      it is the honest margin reading; a marginal eye that happens to pass
 *      CMD19 today is exactly the one that CRC-storms tomorrow;
 *  (b) at run time, a persistent data-CRC storm (the core's
 *      MMC_CRC_STORM_LIMIT) -- mmc_host_ops->lower_bus_clock.
 *
 * The ceiling never climbs back while the card stays powered; a power-off
 * (card removal, mmc_hw_reset) clears it so the next card starts at the top.
 * When no rung is left, tuning accepts whatever window the bottom rung shows
 * (never worse than upstream, which accepted any window at any clock) and
 * the storm hook reports the ladder exhausted so the core drops the bus
 * grade instead.
 */
static const unsigned int rockchip_mmc_clk_rungs[] = {
	200000000, 150000000, 100000000, 50000000,
};

/*
 * One third of the bit period. Measured on the ROC-RK3399-PC's SD slot with
 * a Samsung EVO Plus U3: 227 degrees at 148.5 MHz, 162-165 at 200 MHz, so
 * a healthy eye clears this with margin and a collapsing one (the 35-37
 * degree side islands the same sweep sees) does not.
 */
static unsigned int tuning_min_window = 120;
module_param(tuning_min_window, uint, 0644);
MODULE_PARM_DESC(tuning_min_window,
		 "Minimum passing sample-phase window, in degrees of the card clock period, before a tuned mode's clock is accepted; narrower steps the clock down one rung (default 120)");

struct dw_mci_rockchip_priv_data {
	struct clk		*drv_clk;
	struct clk		*sample_clk;
	int			default_sample_phase;
	int			num_phases;
	bool			internal_phase;
	int                     sample_phase;
	int                     drv_phase;
	unsigned int		clk_ceiling;	/* ladder rung in force, 0 = top */
};

/* The largest rung strictly below @clock, or 0 when the ladder is exhausted */
static unsigned int rockchip_mmc_next_rung(unsigned int clock)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(rockchip_mmc_clk_rungs); i++)
		if (rockchip_mmc_clk_rungs[i] < clock)
			return rockchip_mmc_clk_rungs[i];
	return 0;
}

static const char *rockchip_mmc_timing_name(unsigned char timing)
{
	switch (timing) {
	case MMC_TIMING_UHS_SDR50:	return "SDR50";
	case MMC_TIMING_UHS_SDR104:	return "SDR104";
	case MMC_TIMING_UHS_DDR50:	return "DDR50";
	case MMC_TIMING_MMC_HS200:	return "HS200";
	default:			return "tuned-mode";
	}
}

/*
 * Each fine delay is between 44ps-77ps. Assume each fine delay is 60ps to
 * simplify calculations. So 45degs could be anywhere between 33deg and 57.8deg.
 */
static int rockchip_mmc_get_internal_phase(struct dw_mci *host, bool sample)
{
	unsigned long rate = clk_get_rate(host->ciu_clk) / RK3288_CLKGEN_DIV;
	u32 raw_value;
	u16 degrees;
	u32 delay_num = 0;

	/* Constant signal, no measurable phase shift */
	if (!rate)
		return 0;

	if (sample)
		raw_value = mci_readl(host, TIMING_CON1);
	else
		raw_value = mci_readl(host, TIMING_CON0);

	raw_value >>= ROCKCHIP_MMC_DEGREE_OFFSET;
	degrees = (raw_value & ROCKCHIP_MMC_DEGREE_MASK) * 90;

	if (raw_value & ROCKCHIP_MMC_DELAY_SEL) {
		/* degrees/delaynum * 1000000 */
		unsigned long factor = (ROCKCHIP_MMC_DELAY_ELEMENT_PSEC / 10) *
					36 * (rate / 10000);

		delay_num = (raw_value & ROCKCHIP_MMC_DELAYNUM_MASK);
		delay_num >>= ROCKCHIP_MMC_DELAYNUM_OFFSET;
		degrees += DIV_ROUND_CLOSEST(delay_num * factor, 1000000);
	}

	return degrees % 360;
}

static int rockchip_mmc_get_phase(struct dw_mci *host, bool sample)
{
	struct dw_mci_rockchip_priv_data *priv = host->priv;
	struct clk *clock = sample ? priv->sample_clk : priv->drv_clk;

	if (priv->internal_phase)
		return rockchip_mmc_get_internal_phase(host, sample);
	else
		return clk_get_phase(clock);
}

static int rockchip_mmc_set_internal_phase(struct dw_mci *host, bool sample, int degrees)
{
	unsigned long rate = clk_get_rate(host->ciu_clk) / RK3288_CLKGEN_DIV;
	u8 nineties, remainder;
	u8 delay_num;
	u32 raw_value;
	u32 delay;

	/*
	 * The below calculation is based on the output clock from
	 * MMC host to the card, which expects the phase clock inherits
	 * the clock rate from its parent, namely the output clock
	 * provider of MMC host. However, things may go wrong if
	 * (1) It is orphan.
	 * (2) It is assigned to the wrong parent.
	 *
	 * This check help debug the case (1), which seems to be the
	 * most likely problem we often face and which makes it difficult
	 * for people to debug unstable mmc tuning results.
	 */
	if (!rate) {
		dev_err(host->dev, "%s: invalid clk rate\n", __func__);
		return -EINVAL;
	}

	nineties = degrees / 90;
	remainder = (degrees % 90);

	/*
	 * Due to the inexact nature of the "fine" delay, we might
	 * actually go non-monotonic.  We don't go _too_ monotonic
	 * though, so we should be OK.  Here are options of how we may
	 * work:
	 *
	 * Ideally we end up with:
	 *   1.0, 2.0, ..., 69.0, 70.0, ...,  89.0, 90.0
	 *
	 * On one extreme (if delay is actually 44ps):
	 *   .73, 1.5, ..., 50.6, 51.3, ...,  65.3, 90.0
	 * The other (if delay is actually 77ps):
	 *   1.3, 2.6, ..., 88.6. 89.8, ..., 114.0, 90
	 *
	 * It's possible we might make a delay that is up to 25
	 * degrees off from what we think we're making.  That's OK
	 * though because we should be REALLY far from any bad range.
	 */

	/*
	 * Convert to delay; do a little extra work to make sure we
	 * don't overflow 32-bit / 64-bit numbers.
	 */
	delay = 10000000; /* PSECS_PER_SEC / 10000 / 10 */
	delay *= remainder;
	delay = DIV_ROUND_CLOSEST(delay,
			(rate / 1000) * 36 *
				(ROCKCHIP_MMC_DELAY_ELEMENT_PSEC / 10));

	delay_num = (u8) min_t(u32, delay, 255);

	raw_value = delay_num ? ROCKCHIP_MMC_DELAY_SEL : 0;
	raw_value |= delay_num << ROCKCHIP_MMC_DELAYNUM_OFFSET;
	raw_value |= nineties;

	if (sample)
		mci_writel(host, TIMING_CON1,
			   FIELD_PREP_WM16(GENMASK(11, 1), raw_value));
	else
		mci_writel(host, TIMING_CON0,
			   FIELD_PREP_WM16(GENMASK(11, 1), raw_value));

	dev_dbg(host->dev, "set %s_phase(%d) delay_nums=%u actual_degrees=%d\n",
		sample ? "sample" : "drv", degrees, delay_num,
		rockchip_mmc_get_phase(host, sample)
	);

	return 0;
}

static int rockchip_mmc_set_phase(struct dw_mci *host, bool sample, int degrees)
{
	struct dw_mci_rockchip_priv_data *priv = host->priv;
	struct clk *clock = sample ? priv->sample_clk : priv->drv_clk;

	if (priv->internal_phase)
		return rockchip_mmc_set_internal_phase(host, sample, degrees);
	else
		return clk_set_phase(clock, degrees);
}

static void dw_mci_rk3288_set_ios(struct dw_mci *host, struct mmc_ios *ios)
{
	struct dw_mci_rockchip_priv_data *priv = host->priv;
	int ret;
	unsigned int clock = ios->clock;
	unsigned int cclkin;
	u32 bus_hz;

	/* A power-off ends the card session: the fallback ladder re-arms */
	if (ios->power_mode == MMC_POWER_OFF && priv->clk_ceiling) {
		dev_dbg(host->dev, "power off, clearing the %u Hz clock ceiling\n",
			priv->clk_ceiling);
		priv->clk_ceiling = 0;
	}

	if (clock == 0)
		return;

	/*
	 * The fallback ladder's rung in force. The core keeps asking for its
	 * nominal rate (ios->clock); the CIU runs at the rung, and
	 * dw_mci_setup_bus() then sees bus_hz below the request and takes
	 * the bypass divider, so actual_clock reports the rung.
	 */
	if (priv->clk_ceiling && clock > priv->clk_ceiling)
		clock = priv->clk_ceiling;

	/*
	 * cclkin: source clock of mmc controller
	 * bus_hz: card interface clock generated by CLKGEN
	 * bus_hz = cclkin / RK3288_CLKGEN_DIV
	 * ios->clock = (div == 0) ? bus_hz : (bus_hz / (2 * div))
	 *
	 * Note: div can only be 0 or 1, but div must be set to 1 for eMMC
	 * DDR52 8-bit mode.
	 */
	if (ios->bus_width == MMC_BUS_WIDTH_8 &&
	    ios->timing == MMC_TIMING_MMC_DDR52)
		cclkin = 2 * clock * RK3288_CLKGEN_DIV;
	else
		cclkin = clock * RK3288_CLKGEN_DIV;

	ret = clk_set_rate(host->ciu_clk, cclkin);
	if (ret)
		dev_warn(host->dev, "failed to set rate %uHz err: %d\n", cclkin, ret);

	bus_hz = clk_get_rate(host->ciu_clk) / RK3288_CLKGEN_DIV;
	if (bus_hz != host->bus_hz) {
		host->bus_hz = bus_hz;
		/* force dw_mci_setup_bus() */
		host->current_speed = 0;
	}

	/* Make sure we use phases which we can enumerate with */
	if (!IS_ERR(priv->sample_clk) && ios->timing <= MMC_TIMING_SD_HS)
		rockchip_mmc_set_phase(host, true, priv->default_sample_phase);

	/*
	 * Set the drive phase offset based on speed mode to achieve hold times.
	 *
	 * NOTE: this is _not_ a value that is dynamically tuned and is also
	 * _not_ a value that will vary from board to board.  It is a value
	 * that could vary between different SoC models if they had massively
	 * different output clock delays inside their dw_mmc IP block (delay_o),
	 * but since it's OK to overshoot a little we don't need to do complex
	 * calculations and can pick values that will just work for everyone.
	 *
	 * When picking values we'll stick with picking 0/90/180/270 since
	 * those can be made very accurately on all known Rockchip SoCs.
	 *
	 * Note that these values match values from the DesignWare Databook
	 * tables for the most part except for SDR12 and "ID mode".  For those
	 * two modes the databook calculations assume a clock in of 50MHz.  As
	 * seen above, we always use a clock in rate that is exactly the
	 * card's input clock (times RK3288_CLKGEN_DIV, but that gets divided
	 * back out before the controller sees it).
	 *
	 * From measurement of a single device, it appears that delay_o is
	 * about .5 ns.  Since we try to leave a bit of margin, it's expected
	 * that numbers here will be fine even with much larger delay_o
	 * (the 1.4 ns assumed by the DesignWare Databook would result in the
	 * same results, for instance).
	 */
	if (!IS_ERR(priv->drv_clk)) {
		int phase;

		/*
		 * In almost all cases a 90 degree phase offset will provide
		 * sufficient hold times across all valid input clock rates
		 * assuming delay_o is not absurd for a given SoC.  We'll use
		 * that as a default.
		 */
		phase = 90;

		switch (ios->timing) {
		case MMC_TIMING_MMC_DDR52:
			/*
			 * Since clock in rate with MMC_DDR52 is doubled when
			 * bus width is 8 we need to double the phase offset
			 * to get the same timings.
			 */
			if (ios->bus_width == MMC_BUS_WIDTH_8)
				phase = 180;
			break;
		case MMC_TIMING_UHS_SDR104:
		case MMC_TIMING_MMC_HS200:
			/*
			 * In the case of 150 MHz clock (typical max for
			 * Rockchip SoCs), 90 degree offset will add a delay
			 * of 1.67 ns.  That will meet min hold time of .8 ns
			 * as long as clock output delay is < .87 ns.  On
			 * SoCs measured this seems to be OK, but it doesn't
			 * hurt to give margin here, so we use 180.
			 */
			phase = 180;
			break;
		}

		rockchip_mmc_set_phase(host, false, phase);
	}
}

#define TUNING_ITERATION_TO_PHASE(i, num_phases) \
		(DIV_ROUND_UP((i) * 360, num_phases))

/*
 * One sweep of the sample phase at the current card clock: pick the middle
 * of the widest passing range. @window receives that range's width in
 * degrees (the whole turn when every phase passes).
 */
static int dw_mci_rk3288_tuning_sweep(struct dw_mci_slot *slot, u32 opcode,
				      int *window)
{
	struct dw_mci *host = slot->host;
	struct dw_mci_rockchip_priv_data *priv = host->priv;
	struct mmc_host *mmc = slot->mmc;
	int ret = 0;
	int i;
	bool v, prev_v = 0, first_v;
	struct range_t {
		int start;
		int end; /* inclusive */
	};
	struct range_t *ranges;
	unsigned int range_count = 0;
	int longest_range_len = -1;
	int longest_range = -1;
	int middle_phase;
	int phase;

	*window = 0;

	if (IS_ERR(priv->sample_clk)) {
		dev_err(host->dev, "Tuning clock (sample_clk) not defined.\n");
		return -EIO;
	}

	ranges = kmalloc_array(priv->num_phases / 2 + 1,
			       sizeof(*ranges), GFP_KERNEL);
	if (!ranges)
		return -ENOMEM;

	/* Try each phase and extract good ranges */
	for (i = 0; i < priv->num_phases; ) {
		rockchip_mmc_set_phase(host, true,
				       TUNING_ITERATION_TO_PHASE(
						i,
						priv->num_phases));

		v = !mmc_send_tuning(mmc, opcode, NULL);

		if (i == 0)
			first_v = v;

		if ((!prev_v) && v) {
			range_count++;
			ranges[range_count-1].start = i;
		}
		if (v) {
			ranges[range_count-1].end = i;
			i++;
		} else if (i == priv->num_phases - 1) {
			/* No extra skipping rules if we're at the end */
			i++;
		} else {
			/*
			 * No need to check too close to an invalid
			 * one since testing bad phases is slow.  Skip
			 * 20 degrees.
			 */
			i += DIV_ROUND_UP(20 * priv->num_phases, 360);

			/* Always test the last one */
			if (i >= priv->num_phases)
				i = priv->num_phases - 1;
		}

		prev_v = v;
	}

	if (range_count == 0) {
		dev_warn(host->dev, "All phases bad!");
		ret = -EIO;
		goto free;
	}

	/* wrap around case, merge the end points */
	if ((range_count > 1) && first_v && v) {
		ranges[0].start = ranges[range_count-1].start;
		range_count--;
	}

	if (ranges[0].start == 0 && ranges[0].end == priv->num_phases - 1) {
		rockchip_mmc_set_phase(host, true, priv->default_sample_phase);

		dev_info(host->dev, "All phases work, using default phase %d.",
			 priv->default_sample_phase);
		*window = 360;
		goto free;
	}

	/* Find the longest range */
	for (i = 0; i < range_count; i++) {
		int len = (ranges[i].end - ranges[i].start + 1);

		if (len < 0)
			len += priv->num_phases;

		if (longest_range_len < len) {
			longest_range_len = len;
			longest_range = i;
		}

		dev_dbg(host->dev, "Good phase range %d-%d (%d len)\n",
			TUNING_ITERATION_TO_PHASE(ranges[i].start,
						  priv->num_phases),
			TUNING_ITERATION_TO_PHASE(ranges[i].end,
						  priv->num_phases),
			len
		);
	}

	dev_dbg(host->dev, "Best phase range %d-%d (%d len)\n",
		TUNING_ITERATION_TO_PHASE(ranges[longest_range].start,
					  priv->num_phases),
		TUNING_ITERATION_TO_PHASE(ranges[longest_range].end,
					  priv->num_phases),
		longest_range_len
	);

	middle_phase = ranges[longest_range].start + longest_range_len / 2;
	middle_phase %= priv->num_phases;
	phase = TUNING_ITERATION_TO_PHASE(middle_phase, priv->num_phases);
	dev_info(host->dev, "Successfully tuned phase to %d\n", phase);

	rockchip_mmc_set_phase(host, true, phase);
	*window = TUNING_ITERATION_TO_PHASE(longest_range_len, priv->num_phases);

free:
	kfree(ranges);
	return ret;
}

/*
 * Tuning with the clock-fallback ladder: sweep at the clock in force; a
 * closed eye (all phases bad) or one narrower than tuning_min_window steps
 * the ceiling down one rung, re-rates the CIU clock through the ios and
 * sweeps again. The bottom rung accepts any open eye.
 */
static int dw_mci_rk3288_execute_tuning(struct dw_mci_slot *slot, u32 opcode)
{
	struct dw_mci *host = slot->host;
	struct dw_mci_rockchip_priv_data *priv = host->priv;
	struct mmc_host *mmc = slot->mmc;
	unsigned int cur, next;
	int window, ret;

	if (IS_ERR(priv->sample_clk))
		return dw_mci_rk3288_tuning_sweep(slot, opcode, &window);

	for (;;) {
		ret = dw_mci_rk3288_tuning_sweep(slot, opcode, &window);
		if (ret && ret != -EIO)
			return ret;

		cur = priv->clk_ceiling ?: mmc->ios.clock;
		next = rockchip_mmc_next_rung(cur);
		if (!next) {
			if (!ret && window < tuning_min_window)
				dev_warn(host->dev,
					 "%s tuning eye %d deg < %u deg at %u MHz, no lower rung, accepting\n",
					 rockchip_mmc_timing_name(mmc->ios.timing),
					 window, tuning_min_window,
					 cur / 1000000);
			return ret;
		}
		if (!ret && window >= tuning_min_window)
			return 0;

		if (ret)
			dev_warn(host->dev,
				 "%s tuning: all phases bad at %u MHz, falling back to %u MHz\n",
				 rockchip_mmc_timing_name(mmc->ios.timing),
				 cur / 1000000, next / 1000000);
		else
			dev_warn(host->dev,
				 "%s tuning eye %d deg < %u deg at %u MHz, falling back to %u MHz\n",
				 rockchip_mmc_timing_name(mmc->ios.timing),
				 window, tuning_min_window, cur / 1000000,
				 next / 1000000);

		priv->clk_ceiling = next;
		mmc->ops->set_ios(mmc, &mmc->ios);
	}
}

/*
 * CRC-storm rung (mmc_host_ops->lower_bus_clock via dw_mmc.c): the core
 * has seen MMC_CRC_STORM_LIMIT data CRCs since the last step at this clock.
 * Lower the ceiling one rung; dw_mmc.c re-applies the ios and the core
 * re-tunes at the new rate. -ERANGE when no rung is left, so the core
 * drops the bus grade instead.
 */
static int dw_mci_rockchip_lower_bus_clock(struct dw_mci *host)
{
	struct dw_mci_rockchip_priv_data *priv = host->priv;
	struct mmc_host *mmc = host->slot->mmc;
	unsigned int cur = priv->clk_ceiling ?: mmc->ios.clock;
	unsigned int next = rockchip_mmc_next_rung(cur);

	if (!next) {
		dev_warn(host->dev,
			 "%s data CRC storm at %u MHz, clock ladder exhausted\n",
			 rockchip_mmc_timing_name(mmc->ios.timing), cur / 1000000);
		return -ERANGE;
	}

	dev_warn(host->dev,
		 "%s data CRC storm at %u MHz, falling back to %u MHz\n",
		 rockchip_mmc_timing_name(mmc->ios.timing), cur / 1000000,
		 next / 1000000);
	priv->clk_ceiling = next;
	return 0;
}

static int dw_mci_common_parse_dt(struct dw_mci *host)
{
	struct device_node *np = host->dev->of_node;
	struct dw_mci_rockchip_priv_data *priv;

	priv = devm_kzalloc(host->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	if (of_property_read_u32(np, "rockchip,desired-num-phases",
				 &priv->num_phases))
		priv->num_phases = 360;

	if (of_property_read_u32(np, "rockchip,default-sample-phase",
				 &priv->default_sample_phase))
		priv->default_sample_phase = 0;

	host->priv = priv;

	return 0;
}

static int dw_mci_rk2928_parse_dt(struct dw_mci *host)
{
	struct dw_mci_rockchip_priv_data *priv;
	int err;

	err = dw_mci_common_parse_dt(host);
	if (err)
		return err;

	priv = host->priv;

	priv->internal_phase = false;

	return 0;
}

static int dw_mci_rk3288_parse_dt(struct dw_mci *host)
{
	struct dw_mci_rockchip_priv_data *priv;
	int err;

	err = dw_mci_common_parse_dt(host);
	if (err)
		return err;

	priv = host->priv;

	priv->drv_clk = devm_clk_get(host->dev, "ciu-drive");
	if (IS_ERR(priv->drv_clk))
		dev_dbg(host->dev, "ciu-drive not available\n");

	priv->sample_clk = devm_clk_get(host->dev, "ciu-sample");
	if (IS_ERR(priv->sample_clk))
		dev_dbg(host->dev, "ciu-sample not available\n");

	priv->internal_phase = false;

	return 0;
}

static int dw_mci_rk3576_parse_dt(struct dw_mci *host)
{
	struct dw_mci_rockchip_priv_data *priv;
	int err = dw_mci_common_parse_dt(host);
	if (err)
		return err;

	priv = host->priv;

	priv->internal_phase = true;

	return 0;
}

static int dw_mci_rockchip_init(struct dw_mci *host)
{
	struct dw_mci_rockchip_priv_data *priv = host->priv;
	int ret, i;

	/* It is slot 8 on Rockchip SoCs */
	host->sdio_id0 = 8;

	if (of_device_is_compatible(host->dev->of_node, "rockchip,rk3288-dw-mshc")) {
		host->bus_hz /= RK3288_CLKGEN_DIV;

		/* clock driver will fail if the clock is less than the lowest source clock
		 * divided by the internal clock divider. Test for the lowest available
		 * clock and set the minimum freq to clock / clock divider.
		 */

		for (i = 0; i < ARRAY_SIZE(freqs); i++) {
			ret = clk_round_rate(host->ciu_clk, freqs[i] * RK3288_CLKGEN_DIV);
			if (ret > 0) {
				host->minimum_speed = ret / RK3288_CLKGEN_DIV;
				break;
			}
		}
		if (ret < 0)
			dev_warn(host->dev, "no valid minimum freq: %d\n", ret);
	}

	if (priv->internal_phase)
		mci_writel(host, MISC_CON, MEM_CLK_AUTOGATE_ENABLE);

	return 0;
}

static const struct dw_mci_drv_data rk2928_drv_data = {
	.init			= dw_mci_rockchip_init,
	.parse_dt		= dw_mci_rk2928_parse_dt,
};

static const struct dw_mci_drv_data rk3288_drv_data = {
	.common_caps		= MMC_CAP_CMD23,
	.set_ios		= dw_mci_rk3288_set_ios,
	.execute_tuning		= dw_mci_rk3288_execute_tuning,
	.lower_bus_clock	= dw_mci_rockchip_lower_bus_clock,
	.parse_dt		= dw_mci_rk3288_parse_dt,
	.init			= dw_mci_rockchip_init,
};

static const struct dw_mci_drv_data rk3576_drv_data = {
	.common_caps		= MMC_CAP_CMD23,
	.set_ios		= dw_mci_rk3288_set_ios,
	.execute_tuning		= dw_mci_rk3288_execute_tuning,
	.lower_bus_clock	= dw_mci_rockchip_lower_bus_clock,
	.parse_dt		= dw_mci_rk3576_parse_dt,
	.init			= dw_mci_rockchip_init,
};

static const struct of_device_id dw_mci_rockchip_match[] = {
	{ .compatible = "rockchip,rk2928-dw-mshc",
		.data = &rk2928_drv_data },
	{ .compatible = "rockchip,rk3288-dw-mshc",
		.data = &rk3288_drv_data },
	{ .compatible = "rockchip,rk3576-dw-mshc",
		.data = &rk3576_drv_data },
	{},
};
MODULE_DEVICE_TABLE(of, dw_mci_rockchip_match);

static int dw_mci_rockchip_probe(struct platform_device *pdev)
{
	const struct dw_mci_drv_data *drv_data;
	const struct of_device_id *match;
	int ret;

	if (!pdev->dev.of_node)
		return -ENODEV;

	match = of_match_node(dw_mci_rockchip_match, pdev->dev.of_node);
	drv_data = match->data;

	pm_runtime_get_noresume(&pdev->dev);
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_set_autosuspend_delay(&pdev->dev, 50);
	pm_runtime_use_autosuspend(&pdev->dev);

	ret = dw_mci_pltfm_register(pdev, drv_data);
	if (ret) {
		pm_runtime_disable(&pdev->dev);
		pm_runtime_set_suspended(&pdev->dev);
		pm_runtime_put_noidle(&pdev->dev);
		return ret;
	}

	pm_runtime_put_autosuspend(&pdev->dev);

	return 0;
}

static void dw_mci_rockchip_remove(struct platform_device *pdev)
{
	pm_runtime_get_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	pm_runtime_put_noidle(&pdev->dev);

	dw_mci_pltfm_remove(pdev);
}

static int dw_mci_rockchip_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dw_mci *host = platform_get_drvdata(pdev);
	struct dw_mci_rockchip_priv_data *priv = host->priv;

	if (priv->internal_phase) {
		priv->sample_phase = rockchip_mmc_get_phase(host, true);
		priv->drv_phase = rockchip_mmc_get_phase(host, false);
	}

	return dw_mci_runtime_suspend(dev);
}

static int dw_mci_rockchip_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dw_mci *host = platform_get_drvdata(pdev);
	struct dw_mci_rockchip_priv_data *priv = host->priv;
	int ret;

	ret = dw_mci_runtime_resume(dev);
	if (ret)
		return ret;

	if (priv->internal_phase) {
		rockchip_mmc_set_phase(host, true, priv->sample_phase);
		rockchip_mmc_set_phase(host, false, priv->drv_phase);
		mci_writel(host, MISC_CON, MEM_CLK_AUTOGATE_ENABLE);
	}

	return ret;
}

static const struct dev_pm_ops dw_mci_rockchip_dev_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
	RUNTIME_PM_OPS(dw_mci_rockchip_runtime_suspend, dw_mci_rockchip_runtime_resume, NULL)
};

static struct platform_driver dw_mci_rockchip_pltfm_driver = {
	.probe		= dw_mci_rockchip_probe,
	.remove		= dw_mci_rockchip_remove,
	.driver		= {
		.name		= "dwmmc_rockchip",
		.probe_type	= PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table	= dw_mci_rockchip_match,
		.pm		= pm_ptr(&dw_mci_rockchip_dev_pm_ops),
	},
};

module_platform_driver(dw_mci_rockchip_pltfm_driver);

MODULE_AUTHOR("Addy Ke <addy.ke@rock-chips.com>");
MODULE_DESCRIPTION("Rockchip Specific DW-MSHC Driver Extension");
MODULE_ALIAS("platform:dwmmc_rockchip");
MODULE_LICENSE("GPL v2");
