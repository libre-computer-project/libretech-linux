/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2016, Fuzhou Rockchip Electronics Co., Ltd
 * Author: Lin Huang <hl@rock-chips.com>
 */
#ifndef __SOC_ROCKCHIP_SIP_H
#define __SOC_ROCKCHIP_SIP_H

#include <linux/bits.h>

#define ROCKCHIP_SIP_SUSPEND_MODE		0x82000003
#define ROCKCHIP_SLEEP_PD_CONFIG		0xff

#define ROCKCHIP_SIP_DRAM_FREQ			0x82000008
#define ROCKCHIP_SIP_CONFIG_DRAM_INIT		0x00
#define ROCKCHIP_SIP_CONFIG_DRAM_SET_RATE	0x01
#define ROCKCHIP_SIP_CONFIG_DRAM_ROUND_RATE	0x02
#define ROCKCHIP_SIP_CONFIG_DRAM_SET_AT_SR	0x03
#define ROCKCHIP_SIP_CONFIG_DRAM_GET_BW		0x04
#define ROCKCHIP_SIP_CONFIG_DRAM_GET_RATE	0x05
#define ROCKCHIP_SIP_CONFIG_DRAM_CLR_IRQ	0x06
#define ROCKCHIP_SIP_CONFIG_DRAM_SET_PARAM	0x07
#define ROCKCHIP_SIP_CONFIG_DRAM_SET_ODT_PD	0x08
/*
 * The cooperative SMP rendezvous around DRAM_SET_RATE, for a BL31 whose
 * switch retrains the DDR PHY with the NoC live (Libre Computer's RK3399
 * BL31): every other online CPU issues DRAM_PARK from the stop_machine
 * callback -- BL31 cleans that CPU's caches and spins it in SRAM until the
 * switch is over (returns 0) or 50 ms pass (1) -- and DRAM_SET_RATE, with
 * the number of parking CPUs in x2, waits for that many to have parked,
 * refusing the switch (old rate returned) if they do not.
 * DRAM_GET_RENDEZVOUS reports the last switch: bit 31 refused, bits 30..24
 * CPUs parked, bits 23..0 microseconds waited.
 * A BL31 without them returns 0 to both, which is harmless: the CPUs simply
 * stop in the kernel for the switch, as they always did.
 */
#define ROCKCHIP_SIP_CONFIG_DRAM_PARK		0x40
#define ROCKCHIP_SIP_CONFIG_DRAM_GET_RENDEZVOUS	0x41
/*
 * LPDDR4 MR4 refresh poll. BL31 reads MR4 on each channel (cs0). With
 * RK3399_LPDDR4_MR4_REFRESH the interval starts at 0.25x of 3.9 us, which
 * is legal for every MR4 code, and a longer interval is written only after
 * repeated cool readings. A failed read, an unread rank, a long gap, or
 * suspend returns to that short interval. A BL31 without the opcode
 * returns 0. rk3399_dmc calls it every 200 ms from its own work, whatever
 * the devfreq governor, so a fixed-rate board still samples; the result is
 * BL31's.
 */
#define ROCKCHIP_SIP_CONFIG_DRAM_MR4_REFRESH	0x42
/*
 * Its result: MR4 byte per channel, AC derate asked (110b), busy (a switch
 * held the lock, no sample), AC derate armed for BL31's next timing.
 */
#define ROCKCHIP_SIP_DRAM_MR4_CH0		GENMASK(7, 0)
#define ROCKCHIP_SIP_DRAM_MR4_CH1		GENMASK(15, 8)
#define ROCKCHIP_SIP_DRAM_MR4_DERATE		BIT(20)
#define ROCKCHIP_SIP_DRAM_MR4_BUSY		BIT(23)
#define ROCKCHIP_SIP_DRAM_MR4_AC_DERATE		BIT(24)
#define ROCKCHIP_SIP_DRAM_RENDEZVOUS_REFUSED	BIT(31)
#define ROCKCHIP_SIP_DRAM_RENDEZVOUS_PARKED	GENMASK(30, 24)
#define ROCKCHIP_SIP_DRAM_RENDEZVOUS_US		GENMASK(23, 0)

#endif
