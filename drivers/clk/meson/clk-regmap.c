// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018 BayLibre, SAS.
 * Author: Jerome Brunet <jbrunet@baylibre.com>
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include "clk-regmap.h"

int clk_regmap_init(struct clk_hw *hw)
{
	struct clk_regmap *clk = to_clk_regmap(hw);
	struct device_node *np, *parent_np;
	struct device *dev;

	/* Allow regmap to be preset as it was historically done */
	if (clk->map)
		return 0;

	/*
	 * FIXME: what follows couples the controller implementation
	 * and clk_regmap clock type. This situation is not desirable
	 * but temporary, until the controller is able to register
	 * a hook to initialize a clock type
	 */

	/* Check the usual dev enabled controller with an basic IO regmap */
	dev = clk_hw_get_dev(hw);
	if (dev) {
		clk->map = dev_get_regmap(dev, NULL);
		if (clk->map)
			return 0;
	}

	/* Move on to early and syscon based controllers */
	np = clk_hw_get_of_node(hw);
	if (np) {
		parent_np = of_get_parent(np);
		clk->map = syscon_node_to_regmap(parent_np);
		of_node_put(parent_np);

		if (!IS_ERR_OR_NULL(clk->map))
			return 0;
	}

	/* Bail out if regmap can't be found */
	return -EINVAL;
}
EXPORT_SYMBOL_NS_GPL(clk_regmap_init, "CLK_MESON");

static int clk_regmap_gate_endisable(struct clk_hw *hw, int enable)
{
	struct clk_regmap *clk = to_clk_regmap(hw);
	struct clk_regmap_gate_data *gate = clk_get_regmap_gate_data(clk);
	int set = gate->flags & CLK_GATE_SET_TO_DISABLE ? 1 : 0;

	set ^= enable;

	return regmap_update_bits(clk->map, gate->offset, BIT(gate->bit_idx),
				  set ? BIT(gate->bit_idx) : 0);
}

static int clk_regmap_gate_enable(struct clk_hw *hw)
{
	return clk_regmap_gate_endisable(hw, 1);
}

static void clk_regmap_gate_disable(struct clk_hw *hw)
{
	clk_regmap_gate_endisable(hw, 0);
}

static int clk_regmap_gate_is_enabled(struct clk_hw *hw)
{
	struct clk_regmap *clk = to_clk_regmap(hw);
	struct clk_regmap_gate_data *gate = clk_get_regmap_gate_data(clk);
	unsigned int val;

	regmap_read(clk->map, gate->offset, &val);
	if (gate->flags & CLK_GATE_SET_TO_DISABLE)
		val ^= BIT(gate->bit_idx);

	val &= BIT(gate->bit_idx);

	return val ? 1 : 0;
}

const struct clk_ops clk_regmap_gate_ops = {
	.init		= clk_regmap_init,
	.enable = clk_regmap_gate_enable,
	.disable = clk_regmap_gate_disable,
	.is_enabled = clk_regmap_gate_is_enabled,
};
EXPORT_SYMBOL_NS_GPL(clk_regmap_gate_ops, "CLK_MESON");

const struct clk_ops clk_regmap_gate_ro_ops = {
	.init		= clk_regmap_init,
	.is_enabled = clk_regmap_gate_is_enabled,
};
EXPORT_SYMBOL_NS_GPL(clk_regmap_gate_ro_ops, "CLK_MESON");

static unsigned long clk_regmap_div_recalc_rate(struct clk_hw *hw,
						unsigned long prate)
{
	struct clk_regmap *clk = to_clk_regmap(hw);
	struct clk_regmap_div_data *div = clk_get_regmap_div_data(clk);
	unsigned int val;
	int ret;

	ret = regmap_read(clk->map, div->offset, &val);
	if (ret)
		/* Gives a hint that something is wrong */
		return 0;

	val >>= div->shift;
	val &= clk_div_mask(div->width);
	return divider_recalc_rate(hw, prate, val, div->table, div->flags,
				   div->width);
}

static int clk_regmap_div_determine_rate(struct clk_hw *hw,
					 struct clk_rate_request *req)
{
	struct clk_regmap *clk = to_clk_regmap(hw);
	struct clk_regmap_div_data *div = clk_get_regmap_div_data(clk);
	unsigned int val;
	int ret;

	/* if read only, just return current value */
	if (div->flags & CLK_DIVIDER_READ_ONLY) {
		ret = regmap_read(clk->map, div->offset, &val);
		if (ret)
			return ret;

		val >>= div->shift;
		val &= clk_div_mask(div->width);

		return divider_ro_determine_rate(hw, req, div->table,
						 div->width, div->flags, val);
	}

	return divider_determine_rate(hw, req, div->table, div->width,
				      div->flags);
}

static int clk_regmap_div_set_rate(struct clk_hw *hw, unsigned long rate,
				   unsigned long parent_rate)
{
	struct clk_regmap *clk = to_clk_regmap(hw);
	struct clk_regmap_div_data *div = clk_get_regmap_div_data(clk);
	unsigned int val;
	int ret;

	ret = divider_get_val(rate, parent_rate, div->table, div->width,
			      div->flags);
	if (ret < 0)
		return ret;

	val = (unsigned int)ret << div->shift;
	return regmap_update_bits(clk->map, div->offset,
				  clk_div_mask(div->width) << div->shift, val);
};

/*
 * A parent already running within this of the request is the request: the
 * fixed dividers of the 2 GHz fixed PLL read a few Hz off their nominal rate
 * (fclk_div2p5 reads 799999987 Hz), and a PLL that could make the exact
 * nominal rate must not win over them by those few Hz.
 */
#define CLK_REGMAP_RETUNE_PPM	1

static unsigned long clk_regmap_retune_tol(unsigned long rate)
{
	return rate / 1000000 * CLK_REGMAP_RETUNE_PPM;
}

/*
 * For a divider with CLK_SET_RATE_PARENT over a mux whose parents include
 * programmable PLLs shared with other consumers (G12A/SM1 GP0, HiFi).
 *
 * A request no faster than some running mux input is made from the running
 * inputs as they are, through a divisor, and no PLL is retuned for it: the
 * best rate at or below the request (within the tolerance; never above it,
 * so CLK_DIVIDER_ROUND_CLOSEST picks the same rate).  This covers the rates
 * the fixed dividers make only approximately (a 267 MHz thermal level is
 * fclk_div2p5 / 3), which must never start a PLL.
 *
 * Only a request above every running input is passed to the mux at divisor 1,
 * where clk_regmap_mux_retune_ops picks a PLL to make it.
 *
 * A running input is a prepared one, or one whose rate cannot move (a fixed
 * divider rounds every request to its own rate).  A PLL nobody holds keeps
 * the last rate it was set to; counted, a GP0 left at 1150 MHz would turn
 * every later request below it into a division of the fixed dividers until
 * reboot.  An idle fixed divider still counts, or a 267 MHz request made
 * while the NPU is unbound would lose fclk_div2p5 / 3 to fclk_div4 / 2.
 */
static int clk_regmap_div_retune_determine_rate(struct clk_hw *hw,
						struct clk_rate_request *req)
{
	struct clk_regmap *clk = to_clk_regmap(hw);
	struct clk_regmap_div_data *div = clk_get_regmap_div_data(clk);
	struct clk_hw *mux = clk_hw_get_parent(hw);
	unsigned long tol = clk_regmap_retune_tol(req->rate);
	unsigned long best = 0, best_prate = 0, top = 0;
	struct clk_rate_request preq;
	unsigned int i, d;

	if (!mux || div->table || (div->flags & ~CLK_DIVIDER_ROUND_CLOSEST))
		return clk_regmap_div_determine_rate(hw, req);

	for (i = 0; i < clk_hw_get_num_parents(mux); i++) {
		struct clk_hw *p = clk_hw_get_parent_by_index(mux, i);
		unsigned long r = p ? clk_hw_get_rate(p) : 0;

		/* an idle input that rounds the request elsewhere can be retuned */
		if (r && !clk_hw_is_prepared(p) && clk_hw_round_rate(p, req->rate) != r)
			r = 0;
		top = max(top, r);
		for (d = 1; r && d <= clk_div_mask(div->width) + 1; d++) {
			unsigned long now = DIV_ROUND_UP(r, d);

			/* never above the request, so the largest is also the closest */
			if (now <= req->rate + tol && now > best) {
				best = now;
				best_prate = r;
			}
		}
	}
	if (best && req->rate <= top + tol) {
		req->best_parent_hw = mux;
		req->best_parent_rate = best_prate;
		req->rate = best;
		return 0;
	}

	clk_hw_init_rate_request(mux, &preq, req->rate);
	if (__clk_determine_rate(mux, &preq) || !preq.rate || preq.rate > req->rate)
		return clk_regmap_div_determine_rate(hw, req);

	req->best_parent_hw = mux;
	req->best_parent_rate = preq.rate;
	req->rate = preq.rate;
	return 0;
}

const struct clk_ops clk_regmap_divider_retune_ops = {
	.init = clk_regmap_init,
	.recalc_rate = clk_regmap_div_recalc_rate,
	.determine_rate = clk_regmap_div_retune_determine_rate,
	.set_rate = clk_regmap_div_set_rate,
};
EXPORT_SYMBOL_NS_GPL(clk_regmap_divider_retune_ops, "CLK_MESON");

/* Would prefer clk_regmap_div_ro_ops but clashes with qcom */

const struct clk_ops clk_regmap_divider_ops = {
	.init = clk_regmap_init,
	.recalc_rate = clk_regmap_div_recalc_rate,
	.determine_rate = clk_regmap_div_determine_rate,
	.set_rate = clk_regmap_div_set_rate,
};
EXPORT_SYMBOL_NS_GPL(clk_regmap_divider_ops, "CLK_MESON");

const struct clk_ops clk_regmap_divider_ro_ops = {
	.init = clk_regmap_init,
	.recalc_rate = clk_regmap_div_recalc_rate,
	.determine_rate = clk_regmap_div_determine_rate,
};
EXPORT_SYMBOL_NS_GPL(clk_regmap_divider_ro_ops, "CLK_MESON");

static u8 clk_regmap_mux_get_parent(struct clk_hw *hw)
{
	struct clk_regmap *clk = to_clk_regmap(hw);
	struct clk_regmap_mux_data *mux = clk_get_regmap_mux_data(clk);
	unsigned int val;
	int ret;

	ret = regmap_read(clk->map, mux->offset, &val);
	if (ret)
		return ret;

	val >>= mux->shift;
	val &= mux->mask;
	return clk_mux_val_to_index(hw, mux->table, mux->flags, val);
}

static int clk_regmap_mux_set_parent(struct clk_hw *hw, u8 index)
{
	struct clk_regmap *clk = to_clk_regmap(hw);
	struct clk_regmap_mux_data *mux = clk_get_regmap_mux_data(clk);
	unsigned int val = clk_mux_index_to_val(mux->table, mux->flags, index);

	return regmap_update_bits(clk->map, mux->offset,
				  mux->mask << mux->shift,
				  val << mux->shift);
}

static int clk_regmap_mux_determine_rate(struct clk_hw *hw,
					 struct clk_rate_request *req)
{
	struct clk_regmap *clk = to_clk_regmap(hw);
	struct clk_regmap_mux_data *mux = clk_get_regmap_mux_data(clk);

	return clk_mux_determine_rate_flags(hw, req, mux->flags);
}

const struct clk_ops clk_regmap_mux_ops = {
	.init		= clk_regmap_init,
	.get_parent = clk_regmap_mux_get_parent,
	.set_parent = clk_regmap_mux_set_parent,
	.determine_rate = clk_regmap_mux_determine_rate,
};
EXPORT_SYMBOL_NS_GPL(clk_regmap_mux_ops, "CLK_MESON");

/*
 * For a mux with CLK_SET_RATE_PARENT whose parents include programmable PLLs
 * shared with other consumers (G12A/SM1 GP0, HiFi): a parent already at the
 * request is taken as it is, so no PLL is retuned for a rate a running clock
 * gives; only when none is does a parent get retuned, the closest rate at or
 * below the request winning.  A parent whose rate a consumer holds
 * (clk_rate_exclusive_get) offers only that rate, so it is never retuned
 * underneath its holder.
 */
static int clk_regmap_mux_retune_determine_rate(struct clk_hw *hw,
						struct clk_rate_request *req)
{
	unsigned long tol = clk_regmap_retune_tol(req->rate);
	unsigned int i, n = clk_hw_get_num_parents(hw);
	struct clk_hw *best = NULL;
	unsigned long best_rate = 0;

	for (i = 0; i < n; i++) {
		struct clk_hw *p = clk_hw_get_parent_by_index(hw, i);
		unsigned long r = p ? clk_hw_get_rate(p) : 0;

		if (r && r <= req->rate && req->rate - r <= tol && r > best_rate) {
			best = p;
			best_rate = r;
		}
	}

	for (i = 0; !best && i < n; i++) {
		struct clk_hw *p = clk_hw_get_parent_by_index(hw, i);
		struct clk_rate_request preq;

		if (!p)
			continue;
		clk_hw_init_rate_request(p, &preq, req->rate);
		if (__clk_determine_rate(p, &preq))
			continue;
		if (preq.rate <= req->rate && preq.rate > best_rate) {
			req->best_parent_hw = p;
			best_rate = preq.rate;
		}
	}
	if (best)
		req->best_parent_hw = best;
	if (!best_rate)
		return -EINVAL;

	req->best_parent_rate = best_rate;
	req->rate = best_rate;
	return 0;
}

const struct clk_ops clk_regmap_mux_retune_ops = {
	.init		= clk_regmap_init,
	.get_parent = clk_regmap_mux_get_parent,
	.set_parent = clk_regmap_mux_set_parent,
	.determine_rate = clk_regmap_mux_retune_determine_rate,
};
EXPORT_SYMBOL_NS_GPL(clk_regmap_mux_retune_ops, "CLK_MESON");

const struct clk_ops clk_regmap_mux_ro_ops = {
	.init		= clk_regmap_init,
	.get_parent = clk_regmap_mux_get_parent,
};
EXPORT_SYMBOL_NS_GPL(clk_regmap_mux_ro_ops, "CLK_MESON");

MODULE_DESCRIPTION("Amlogic regmap backed clock driver");
MODULE_AUTHOR("Jerome Brunet <jbrunet@baylibre.com>");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("CLK_MESON");
