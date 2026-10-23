// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2016 Rockchip Electronics Co. Ltd.
 * Author: Lin Huang <hl@rock-chips.com>
 */

#include <linux/arm-smccc.h>
#include <linux/atomic.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/cpumask.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/stop_machine.h>
#include <soc/rockchip/rockchip_sip.h>
#include "clk.h"

struct rockchip_ddrclk {
	struct clk_hw	hw;
	void __iomem	*reg_base;
	int		mux_offset;
	int		mux_shift;
	int		mux_width;
	int		div_shift;
	int		div_width;
	int		ddr_flag;
	spinlock_t	*lock;
};

#define to_rockchip_ddrclk_hw(hw) container_of(hw, struct rockchip_ddrclk, hw)

/*
 * The DRAM frequency switch is a cross-CPU rendezvous, not an SMC.
 *
 * BL31's ddr_set_rate() takes the DDR controller offline for the CIC change
 * and then retrains the PHY with the NoC live; during the retrain a read
 * through the PHY returns garbage and a cache write-back lands as garbage.
 * The calling CPU's caches BL31 cleans itself, but it has no way to reach
 * the other five -- an EL3-initiated SGI is not deliverable on RK3399's
 * insecure GIC integration -- so the OS, which owns its CPUs, parks them:
 * the switch runs under stop_machine(), every CPU enters the callback with
 * interrupts off, the first to claim it issues DRAM_SET_RATE and every other
 * one issues DRAM_PARK, which cleans that CPU's caches in BL31 and spins it
 * in SRAM until the switch is over. DRAM_SET_RATE waits for every other
 * online CPU to have parked before it starts and REFUSES the switch (old
 * rate kept) if one does not arrive, so the ordering between the callbacks
 * does not matter and nothing can hang.
 *
 * _cpuslocked: the caller holds cpus_read_lock(). It cannot be taken here
 * because clk_set_rate() already holds the prepare lock, and the hotplug
 * path nests the prepare lock inside the hotplug lock (cpufreq online).
 */
struct rockchip_ddrclk_switch {
	unsigned long rate;
	unsigned long parkers;
	atomic_t claimed;
	unsigned long got;
};

static int rockchip_ddrclk_sip_switch(void *arg)
{
	struct rockchip_ddrclk_switch *sw = arg;
	struct arm_smccc_res res;

	if (atomic_cmpxchg(&sw->claimed, 0, 1) == 0) {
		/*
		 * x2 tells BL31 how many other CPUs are parking for this
		 * switch. It has its own online mask, but that is a PSCI
		 * on/off/suspend snapshot and under cpuidle churn a CPU can
		 * be running with its bit clear (measured: five parked, mask
		 * said one); this count is authoritative, the mask its floor.
		 */
		arm_smccc_smc(ROCKCHIP_SIP_DRAM_FREQ, sw->rate, sw->parkers,
			      ROCKCHIP_SIP_CONFIG_DRAM_SET_RATE,
			      0, 0, 0, 0, &res);
		sw->got = res.a0;
	} else {
		arm_smccc_smc(ROCKCHIP_SIP_DRAM_FREQ, 0, 0,
			      ROCKCHIP_SIP_CONFIG_DRAM_PARK,
			      0, 0, 0, 0, &res);
	}

	return 0;
}

static int rockchip_ddrclk_sip_set_rate(struct clk_hw *hw, unsigned long drate,
					unsigned long prate)
{
	struct rockchip_ddrclk_switch sw = {
		.rate = drate,
		.parkers = num_online_cpus() - 1,
		.claimed = ATOMIC_INIT(0),
	};
	struct arm_smccc_res res;
	int ret;

	/*
	 * cpu_online_mask, NOT NULL: with a NULL cpumask stop_machine() runs
	 * the callback on the first online CPU only and merely parks the rest
	 * in its own loop -- in DRAM, unparked -- and BL31 then refuses every
	 * switch with "0 of N other CPUs parked" (measured 2026-09-12).
	 */
	ret = stop_machine_cpuslocked(rockchip_ddrclk_sip_switch, &sw,
				      cpu_online_mask);
	if (ret)
		return ret;

	arm_smccc_smc(ROCKCHIP_SIP_DRAM_FREQ, 0, 0,
		      ROCKCHIP_SIP_CONFIG_DRAM_GET_RENDEZVOUS,
		      0, 0, 0, 0, &res);
	if (res.a0 & ROCKCHIP_SIP_DRAM_RENDEZVOUS_REFUSED)
		pr_warn_ratelimited("%s: %lu Hz refused by BL31: rendezvous timed out, %lu of %lu other CPUs parked; staying at %lu MHz\n",
				    clk_hw_get_name(hw), drate,
				    FIELD_GET(ROCKCHIP_SIP_DRAM_RENDEZVOUS_PARKED, res.a0),
				    sw.parkers, sw.got);
	else
		pr_debug("%s: %lu Hz: got %lu MHz, %lu CPUs parked, rendezvous %lu us\n",
			 clk_hw_get_name(hw), drate, sw.got,
			 FIELD_GET(ROCKCHIP_SIP_DRAM_RENDEZVOUS_PARKED, res.a0),
			 FIELD_GET(ROCKCHIP_SIP_DRAM_RENDEZVOUS_US, res.a0));

	return 0;
}

static unsigned long
rockchip_ddrclk_sip_recalc_rate(struct clk_hw *hw,
				unsigned long parent_rate)
{
	struct arm_smccc_res res;

	arm_smccc_smc(ROCKCHIP_SIP_DRAM_FREQ, 0, 0,
		      ROCKCHIP_SIP_CONFIG_DRAM_GET_RATE,
		      0, 0, 0, 0, &res);

	return res.a0;
}

static int rockchip_ddrclk_sip_determine_rate(struct clk_hw *hw,
					      struct clk_rate_request *req)
{
	struct arm_smccc_res res;

	arm_smccc_smc(ROCKCHIP_SIP_DRAM_FREQ, req->rate, 0,
		      ROCKCHIP_SIP_CONFIG_DRAM_ROUND_RATE,
		      0, 0, 0, 0, &res);

	req->rate = res.a0;

	return 0;
}

static u8 rockchip_ddrclk_get_parent(struct clk_hw *hw)
{
	struct rockchip_ddrclk *ddrclk = to_rockchip_ddrclk_hw(hw);
	u32 val;

	val = readl(ddrclk->reg_base +
			ddrclk->mux_offset) >> ddrclk->mux_shift;
	val &= GENMASK(ddrclk->mux_width - 1, 0);

	return val;
}

static const struct clk_ops rockchip_ddrclk_sip_ops = {
	.recalc_rate = rockchip_ddrclk_sip_recalc_rate,
	.set_rate = rockchip_ddrclk_sip_set_rate,
	.determine_rate = rockchip_ddrclk_sip_determine_rate,
	.get_parent = rockchip_ddrclk_get_parent,
};

struct clk *rockchip_clk_register_ddrclk(const char *name, int flags,
					 const char *const *parent_names,
					 u8 num_parents, int mux_offset,
					 int mux_shift, int mux_width,
					 int div_shift, int div_width,
					 int ddr_flag, void __iomem *reg_base,
					 spinlock_t *lock)
{
	struct rockchip_ddrclk *ddrclk;
	struct clk_init_data init;
	struct clk *clk;

	ddrclk = kzalloc(sizeof(*ddrclk), GFP_KERNEL);
	if (!ddrclk)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.parent_names = parent_names;
	init.num_parents = num_parents;

	init.flags = flags;
	init.flags |= CLK_SET_RATE_NO_REPARENT;

	switch (ddr_flag) {
	case ROCKCHIP_DDRCLK_SIP:
		init.ops = &rockchip_ddrclk_sip_ops;
		break;
	default:
		pr_err("%s: unsupported ddrclk type %d\n", __func__, ddr_flag);
		kfree(ddrclk);
		return ERR_PTR(-EINVAL);
	}

	ddrclk->reg_base = reg_base;
	ddrclk->lock = lock;
	ddrclk->hw.init = &init;
	ddrclk->mux_offset = mux_offset;
	ddrclk->mux_shift = mux_shift;
	ddrclk->mux_width = mux_width;
	ddrclk->div_shift = div_shift;
	ddrclk->div_width = div_width;
	ddrclk->ddr_flag = ddr_flag;

	clk = clk_register(NULL, &ddrclk->hw);
	if (IS_ERR(clk))
		kfree(ddrclk);

	return clk;
}
EXPORT_SYMBOL_GPL(rockchip_clk_register_ddrclk);
