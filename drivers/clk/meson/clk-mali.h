/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2026 Libre Computer
 *
 * Amlogic Mali glitch-free clock switch
 *
 * The Mali GPU uses a dual-path clock mux where mali_0 and mali_1 each have
 * their own parent selector, divider, and gate, feeding into a final mux
 * (bit 31).  Naive CCF handling causes clock glitches during parent changes
 * that crash the Mali-450.  This driver performs the switch atomically:
 * configure the inactive path, then flip the mux bit.
 */

#ifndef __MESON_CLK_MALI_H
#define __MESON_CLK_MALI_H

#include <linux/clk-provider.h>

/**
 * struct meson_mali_clk_data - data for the mali glitch-free clock
 *
 * @offset:        register offset of HHI_MALI_CLK_CNTL
 * @parent_table:  array of { parent_rate, parent_index } for each OPP,
 *                 terminated by { 0, 0 }.  parent_index is the 3-bit mux
 *                 value (matching the hw encoding in bits [11:9] / [27:25]).
 *
 * Register layout (single 32-bit register):
 *   [6:0]   mali_0 divider
 *   [8]     mali_0 gate
 *   [11:9]  mali_0 parent select
 *   [22:16] mali_1 divider
 *   [24]    mali_1 gate
 *   [27:25] mali_1 parent select
 *   [31]    final mux: 0 = mali_0, 1 = mali_1
 */
struct meson_mali_clk_data {
	unsigned int offset;
};

extern const struct clk_ops meson_mali_clk_ops;

#endif /* __MESON_CLK_MALI_H */
