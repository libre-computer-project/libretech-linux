// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2026 Libre Computer
 * Author: Libre Computer
 *
 * Amlogic Mali glitch-free clock switch
 *
 * The Mali GPU on Amlogic SoCs uses a dual-path glitch-free mux:
 *
 *   mali_0_sel -> mali_0_div -> mali_0 (gate) --+
 *                                                +--> final_mux --> GPU
 *   mali_1_sel -> mali_1_div -> mali_1 (gate) --+
 *
 * When CCF naively changes the rate, it ungates the inactive path, changes
 * its parent (causing a clock glitch on that path), sets the divider, then
 * switches the final mux.  The parent change while the path is still
 * feeding stale configuration through the gate creates a transient glitch
 * that crashes the Mali-450 PP and GP MMU.
 *
 * This driver implements set_rate atomically:
 *   1. Read which path is currently active (bit 31)
 *   2. Configure the INACTIVE path: set parent mux + divider while gated
 *   3. Enable the inactive path's gate
 *   4. Flip bit 31 to switch to the newly configured path
 *   5. Disable the old path's gate
 *
 * This matches the BSP approach (direct HHI register writes under spinlock)
 * while remaining a proper CCF citizen.
 */

#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/module.h>

#include "clk-regmap.h"
#include "clk-mali.h"

/*
 * v6.18: the per-SoC clock-controller headers (gxbb.h/g12a.h) were removed
 * upstream and their register offsets folded into the controller .c files.
 * HHI_GP0_PLL_CNTL is needed here for the PLL-lock poll, so define it
 * locally (same offset as gxbb.c's copy).
 */
#define HHI_GP0_PLL_CNTL		0x40

/* Register field definitions for HHI_MALI_CLK_CNTL */
#define MALI_0_DIV_MASK		GENMASK(6, 0)
#define MALI_0_DIV_SHIFT	0
#define MALI_0_GATE		BIT(8)
#define MALI_0_SEL_MASK		GENMASK(11, 9)
#define MALI_0_SEL_SHIFT	9

#define MALI_1_DIV_MASK		GENMASK(22, 16)
#define MALI_1_DIV_SHIFT	16
#define MALI_1_GATE		BIT(24)
#define MALI_1_SEL_MASK		GENMASK(27, 25)
#define MALI_1_SEL_SHIFT	25

#define MALI_MUX_BIT		BIT(31)

static inline struct meson_mali_clk_data *
meson_mali_clk_data(struct clk_regmap *clk)
{
	return (struct meson_mali_clk_data *)clk->data;
}

/*
 * Get the rate of a given parent by index.  The parent_hws array in
 * clk_init_data lists the available parents.
 */
static unsigned long mali_get_parent_rate(struct clk_hw *hw, u8 parent_idx)
{
	struct clk_hw *parent = clk_hw_get_parent_by_index(hw, parent_idx);

	if (!parent)
		return 0;
	return clk_hw_get_rate(parent);
}

/*
 * Read the current active path's configuration and compute the output rate.
 */
static unsigned long meson_mali_recalc_rate(struct clk_hw *hw,
					    unsigned long parent_rate)
{
	struct clk_regmap *clk = to_clk_regmap(hw);
	struct meson_mali_clk_data *data = meson_mali_clk_data(clk);
	unsigned int val, parent_idx, div;
	unsigned long prate;

	regmap_read(clk->map, data->offset, &val);

	if (val & MALI_MUX_BIT) {
		/* mali_1 is active */
		parent_idx = (val & MALI_1_SEL_MASK) >> MALI_1_SEL_SHIFT;
		div = ((val & MALI_1_DIV_MASK) >> MALI_1_DIV_SHIFT) + 1;
	} else {
		/* mali_0 is active */
		parent_idx = (val & MALI_0_SEL_MASK) >> MALI_0_SEL_SHIFT;
		div = ((val & MALI_0_DIV_MASK) >> MALI_0_DIV_SHIFT) + 1;
	}

	prate = mali_get_parent_rate(hw, parent_idx);
	if (!prate)
		return 0;

	return DIV_ROUND_UP_ULL((u64)prate, div);
}

/* Parent index of GP0_PLL in gxbb_mali_0_1_parent_data[]. */
#define MALI_PARENT_GP0_PLL	1

/*
 * Find the best parent + divider combination for a target rate.
 *
 * Two-pass smart iteration:
 *   Pass 1: skip GP0_PLL. Find the best (fixed-parent, div) combo whose
 *           output rate is <= the target. If an exact match is found,
 *           use it -- no PLL reprogramming needed.
 *   Pass 2: only if Pass 1 didn't find an exact match, consider GP0_PLL
 *           as a "programmable to any rate" source -- treat its prate
 *           as the target rate (with CLK_SET_RATE_PARENT, CCF will then
 *           ask gp0_pll to set itself to req->rate before our set_rate
 *           runs). div=1 (output == prate).
 *
 * Effect: low rates (125-666 MHz, all exactly reachable from
 * fclk_div3/4/5 or mpll) use fixed parents -- no PLL relock per
 * governor scale. Only OC rates above 666 MHz (792/1008/1200/1392/1584
 * etc.) trigger GP0_PLL reprogramming.
 */
static int meson_mali_determine_rate(struct clk_hw *hw,
				     struct clk_rate_request *req)
{
	unsigned long best_rate = 0;
	unsigned long best_parent_rate = 0;
	int best_parent = -1;
	unsigned int best_div = 1;
	unsigned int num_parents = clk_hw_get_num_parents(hw);
	unsigned int i;

	/* Pass 1: fixed parents only. */
	for (i = 0; i < num_parents; i++) {
		struct clk_hw *parent;
		unsigned long prate;
		unsigned int div;
		unsigned long rate;

		if (i == MALI_PARENT_GP0_PLL)
			continue;

		parent = clk_hw_get_parent_by_index(hw, i);
		if (!parent)
			continue;

		prate = clk_hw_get_rate(parent);
		if (!prate)
			continue;

		for (div = 1; div <= 128; div++) {
			rate = DIV_ROUND_UP_ULL((u64)prate, div);

			if (rate > req->rate)
				continue;

			if (rate > best_rate ||
			    (rate == best_rate && div < best_div)) {
				best_rate = rate;
				best_parent_rate = prate;
				best_parent = i;
				best_div = div;
			}

			if (rate == req->rate)
				break;
		}
	}

	/*
	 * Pass 2: if pass 1 didn't yield an exact match, try GP0_PLL with
	 * the target rate as the parent's programmed rate. CLK_SET_RATE_PARENT
	 * on the composite makes CCF actually reprogram gp0_pll to that
	 * rate before set_rate runs here.
	 */
	if (best_rate != req->rate) {
		struct clk_hw *gp0_pll_hw =
			clk_hw_get_parent_by_index(hw, MALI_PARENT_GP0_PLL);

		if (gp0_pll_hw && req->rate > 0) {
			best_rate = req->rate;
			best_parent_rate = req->rate;
			best_parent = MALI_PARENT_GP0_PLL;
			best_div = 1;
		}
	}

	if (best_parent < 0)
		return -EINVAL;

	req->rate = best_rate;
	req->best_parent_rate = best_parent_rate;
	req->best_parent_hw = clk_hw_get_parent_by_index(hw, best_parent);

	return 0;
}

/*
 * Poll GP0_PLL lock bit after selecting it as a parent.
 * GP0_PLL relock can take 50-200us; the blind 10us delay is insufficient.
 * Use 20us intervals with a 20ms timeout (1000 iterations) matching the
 * kernel PLL driver's polling strategy.
 */
static int meson_mali_wait_pll_lock(struct clk_regmap *clk)
{
	unsigned int val;
	int timeout = 1000;

	do {
		regmap_read(clk->map, HHI_GP0_PLL_CNTL, &val);
		if (val & BIT(31))
			return 0;
		udelay(20);
	} while (--timeout);

	pr_warn("mali clk: GP0_PLL failed to lock (reg=0x%08x)\n", val);
	return -ETIMEDOUT;
}

/*
 * Atomically switch the mali clock to a new rate.
 *
 * This is the core of the glitch-free switch:
 *   1. Determine which path is currently active
 *   2. Program the inactive path (parent + divider) while it's gated
 *   3. Enable the inactive path's gate
 *   4. Flip the final mux bit
 *   5. Disable the old path's gate
 */
static int meson_mali_set_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long parent_rate)
{
	struct clk_regmap *clk = to_clk_regmap(hw);
	struct meson_mali_clk_data *data = meson_mali_clk_data(clk);
	unsigned int num_parents = clk_hw_get_num_parents(hw);
	unsigned int val, new_parent = 0, new_div = 0;
	bool found = false;
	unsigned int i;

	/* Find the parent + divider that gives us the requested rate */
	for (i = 0; i < num_parents; i++) {
		struct clk_hw *parent = clk_hw_get_parent_by_index(hw, i);
		unsigned long prate;
		unsigned int div;

		if (!parent)
			continue;

		prate = clk_hw_get_rate(parent);
		if (!prate)
			continue;

		for (div = 1; div <= 128; div++) {
			unsigned long r = DIV_ROUND_UP_ULL((u64)prate, div);

			if (r == rate) {
				new_parent = i;
				new_div = div - 1; /* hw encoding */
				found = true;
				goto done;
			}
		}
	}

done:
	if (!found)
		return -EINVAL;

	regmap_read(clk->map, data->offset, &val);

	pr_debug("mali clk: set_rate %lu, reg=0x%08x, new_parent=%u new_div=%u active=%s\n",
		 rate, val, new_parent, new_div,
		 (val & MALI_MUX_BIT) ? "mali_1" : "mali_0");

	if (val & MALI_MUX_BIT) {
		/*
		 * mali_1 is active.  Configure mali_0 (inactive).
		 * 1. Ensure mali_0 gate is off
		 * 2. Set mali_0 parent and divider
		 * 3. Enable mali_0 gate
		 * 4. Switch mux to mali_0 (clear bit 31)
		 * 5. Disable mali_1 gate
		 */
		val &= ~MALI_0_GATE;
		regmap_write(clk->map, data->offset, val);

		val &= ~(MALI_0_SEL_MASK | MALI_0_DIV_MASK);
		val |= (new_parent << MALI_0_SEL_SHIFT) & MALI_0_SEL_MASK;
		val |= (new_div << MALI_0_DIV_SHIFT) & MALI_0_DIV_MASK;
		regmap_write(clk->map, data->offset, val);

		/*
		 * If switching to GP0_PLL, wait for it to lock. If the PLL
		 * fails to lock (DCO outside silicon's lockable range),
		 * abort -- enabling the gate and flipping the mux would
		 * route an unlocked clock to the GPU, which CCF still
		 * reports as the target rate while HW free-runs at some
		 * other frequency. Mali jobs then either silently produce
		 * wrong results or crash.
		 */
		if (new_parent == 1 && meson_mali_wait_pll_lock(clk))
			return -EIO;

		/* Enable mali_0 gate */
		val |= MALI_0_GATE;
		regmap_write(clk->map, data->offset, val);

		/* Wait for clock to stabilize before switching mux */
		udelay(100);

		/* Switch to mali_0 */
		val &= ~MALI_MUX_BIT;
		regmap_write(clk->map, data->offset, val);

		/* Wait for mux switch to complete */
		udelay(100);

		/* Disable mali_1 gate */
		val &= ~MALI_1_GATE;
		regmap_write(clk->map, data->offset, val);
	} else {
		/*
		 * mali_0 is active.  Configure mali_1 (inactive).
		 * 1. Ensure mali_1 gate is off
		 * 2. Set mali_1 parent and divider
		 * 3. Enable mali_1 gate
		 * 4. Switch mux to mali_1 (set bit 31)
		 * 5. Disable mali_0 gate
		 */
		val &= ~MALI_1_GATE;
		regmap_write(clk->map, data->offset, val);

		val &= ~(MALI_1_SEL_MASK | MALI_1_DIV_MASK);
		val |= (new_parent << MALI_1_SEL_SHIFT) & MALI_1_SEL_MASK;
		val |= (new_div << MALI_1_DIV_SHIFT) & MALI_1_DIV_MASK;
		regmap_write(clk->map, data->offset, val);

		/* Abort transition if GP0_PLL fails to lock (see comment above) */
		if (new_parent == 1 && meson_mali_wait_pll_lock(clk))
			return -EIO;

		/* Enable mali_1 gate */
		val |= MALI_1_GATE;
		regmap_write(clk->map, data->offset, val);

		/* Wait for clock to stabilize before switching mux */
		udelay(100);

		/* Switch to mali_1 */
		val |= MALI_MUX_BIT;
		regmap_write(clk->map, data->offset, val);

		/* Wait for mux switch to complete */
		udelay(100);

		/* Disable mali_0 gate */
		val &= ~MALI_0_GATE;
		regmap_write(clk->map, data->offset, val);
	}

	/* Readback for debug */
	regmap_read(clk->map, data->offset, &val);
	pr_debug("mali clk: set_rate done, reg=0x%08x\n", val);

	return 0;
}

/*
 * Get the index of the currently active parent.  Since this composite
 * handles its own muxing internally, we report the parent of the
 * currently active path.
 */
static u8 meson_mali_get_parent(struct clk_hw *hw)
{
	struct clk_regmap *clk = to_clk_regmap(hw);
	struct meson_mali_clk_data *data = meson_mali_clk_data(clk);
	unsigned int val;

	regmap_read(clk->map, data->offset, &val);

	if (val & MALI_MUX_BIT)
		return (val & MALI_1_SEL_MASK) >> MALI_1_SEL_SHIFT;
	else
		return (val & MALI_0_SEL_MASK) >> MALI_0_SEL_SHIFT;
}

static int meson_mali_set_parent(struct clk_hw *hw, u8 index)
{
	/*
	 * OF's `assigned-clock-parents` invokes this directly (CCF's
	 * automatic-reparent-during-set_rate path can be inhibited with
	 * CLK_SET_RATE_NO_REPARENT, but the direct path cannot). Earlier
	 * iterations had this as a `return 0` stub on the assumption that
	 * set_parent would never be called -- which is wrong, and silently
	 * desynced the hardware mux state from CCF's cached parent.
	 *
	 * Do the same glitch-free dual-path switch as set_rate, with the
	 * parent index given explicitly and the divider computed from the
	 * currently reported rate (so the output frequency is preserved
	 * across the reparent). Relies on CLK_OPS_PARENT_ENABLE on the
	 * composite so CCF has enabled the new parent for us before we
	 * flip the mux.
	 */
	struct clk_regmap *clk = to_clk_regmap(hw);
	struct meson_mali_clk_data *data = meson_mali_clk_data(clk);
	struct clk_hw *new_parent;
	unsigned long cur_rate, new_parent_rate;
	unsigned int val, new_div, div;

	new_parent = clk_hw_get_parent_by_index(hw, index);
	if (!new_parent)
		return -EINVAL;

	new_parent_rate = clk_hw_get_rate(new_parent);
	if (!new_parent_rate)
		return -EINVAL;

	/*
	 * Choose a divider that gives the closest output <= the current
	 * reported rate, capped at div=128. If the recalc rate is 0 or
	 * exceeds the new parent rate, fall back to div=1 (output =
	 * parent_rate).
	 */
	cur_rate = clk_hw_get_rate(hw);
	if (!cur_rate || cur_rate >= new_parent_rate) {
		new_div = 0;
	} else {
		div = DIV_ROUND_UP_ULL((u64)new_parent_rate, cur_rate);
		if (div > 128)
			div = 128;
		new_div = div - 1;
	}

	regmap_read(clk->map, data->offset, &val);

	if (val & MALI_MUX_BIT) {
		/* mali_1 active -- configure mali_0 then switch to it */
		val &= ~MALI_0_GATE;
		regmap_write(clk->map, data->offset, val);

		val &= ~(MALI_0_SEL_MASK | MALI_0_DIV_MASK);
		val |= (index << MALI_0_SEL_SHIFT) & MALI_0_SEL_MASK;
		val |= (new_div << MALI_0_DIV_SHIFT) & MALI_0_DIV_MASK;
		regmap_write(clk->map, data->offset, val);

		if (index == 1 && meson_mali_wait_pll_lock(clk))
			return -EIO;

		val |= MALI_0_GATE;
		regmap_write(clk->map, data->offset, val);
		udelay(100);

		val &= ~MALI_MUX_BIT;
		regmap_write(clk->map, data->offset, val);
		udelay(100);

		val &= ~MALI_1_GATE;
		regmap_write(clk->map, data->offset, val);
	} else {
		/* mali_0 active -- configure mali_1 then switch to it */
		val &= ~MALI_1_GATE;
		regmap_write(clk->map, data->offset, val);

		val &= ~(MALI_1_SEL_MASK | MALI_1_DIV_MASK);
		val |= (index << MALI_1_SEL_SHIFT) & MALI_1_SEL_MASK;
		val |= (new_div << MALI_1_DIV_SHIFT) & MALI_1_DIV_MASK;
		regmap_write(clk->map, data->offset, val);

		if (index == 1 && meson_mali_wait_pll_lock(clk))
			return -EIO;

		val |= MALI_1_GATE;
		regmap_write(clk->map, data->offset, val);
		udelay(100);

		val |= MALI_MUX_BIT;
		regmap_write(clk->map, data->offset, val);
		udelay(100);

		val &= ~MALI_0_GATE;
		regmap_write(clk->map, data->offset, val);
	}

	return 0;
}

/*
 * Enable: ensure the currently active path's gate is on.
 * This is called by CCF when the clock consumer enables the clock.
 */
static int meson_mali_enable(struct clk_hw *hw)
{
	struct clk_regmap *clk = to_clk_regmap(hw);
	struct meson_mali_clk_data *data = meson_mali_clk_data(clk);
	unsigned int val;

	regmap_read(clk->map, data->offset, &val);

	if (val & MALI_MUX_BIT)
		val |= MALI_1_GATE;
	else
		val |= MALI_0_GATE;

	regmap_write(clk->map, data->offset, val);
	return 0;
}

static void meson_mali_disable(struct clk_hw *hw)
{
	struct clk_regmap *clk = to_clk_regmap(hw);
	struct meson_mali_clk_data *data = meson_mali_clk_data(clk);
	unsigned int val;

	regmap_read(clk->map, data->offset, &val);

	/* Disable both gates */
	val &= ~(MALI_0_GATE | MALI_1_GATE);
	regmap_write(clk->map, data->offset, val);
}

static int meson_mali_is_enabled(struct clk_hw *hw)
{
	struct clk_regmap *clk = to_clk_regmap(hw);
	struct meson_mali_clk_data *data = meson_mali_clk_data(clk);
	unsigned int val;

	regmap_read(clk->map, data->offset, &val);

	if (val & MALI_MUX_BIT)
		return !!(val & MALI_1_GATE);
	else
		return !!(val & MALI_0_GATE);
}

const struct clk_ops meson_mali_clk_ops = {
	/*
	 * MANDATORY on v6.18+.  Up to v6.12 the controller probe
	 * (meson_eeclkc_probe) walked gxbb_clk_regmaps[] and preset
	 * ->map on every clk_regmap before registration, so a clk_ops
	 * without an .init still had a valid regmap.  v6.18 deleted that
	 * array and meson_clkc_syscon_probe() no longer touches ->map;
	 * each clk_regmap type now populates it lazily from its own
	 * .init = clk_regmap_init (see clk-regmap.c, where every
	 * clk_regmap_{gate,divider,mux}[_ro]_ops sets .init).
	 * Omitting it compiles cleanly and then NULL-derefs inside
	 * regmap_read() on the first recalc_rate.
	 */
	.init = clk_regmap_init,
	.enable = meson_mali_enable,
	.disable = meson_mali_disable,
	.is_enabled = meson_mali_is_enabled,
	.recalc_rate = meson_mali_recalc_rate,
	.determine_rate = meson_mali_determine_rate,
	.set_rate = meson_mali_set_rate,
	.get_parent = meson_mali_get_parent,
	.set_parent = meson_mali_set_parent,
};
EXPORT_SYMBOL_NS_GPL(meson_mali_clk_ops, "CLK_MESON");

MODULE_DESCRIPTION("Amlogic Mali glitch-free clock switch");
MODULE_AUTHOR("Libre Computer");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("CLK_MESON");
