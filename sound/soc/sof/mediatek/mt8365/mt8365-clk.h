/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2025 MediaTek Inc.
 *
 * Author: Aary Patil <aary.patil@mediatek.com>
 *
 * Header file for the mt8365 DSP clock definition
 *
 * Ported from MediaTek's official Genio BSP kernel
 * (gitlab.com/mediatek/aiot/bsp/linux, branch mtk-v6.6):
 * sound/soc/sof/mediatek/mt8365/mt8365-clk.h
 */

#ifndef __MT8365_CLK_H
#define __MT8365_CLK_H

struct snd_sof_dev;

/*DSP clock*/
enum adsp_clk_id {
	CLK_TOP_DSP_SEL,
	CLK_TOP_CLK26M_D52,
	CLK_TOP_SYS_26M_D2,
	CLK_TOP_DSPPLL,
	CLK_TOP_DSPPLL_D2,
	CLK_TOP_DSPPLL_D4,
	CLK_TOP_DSPPLL_D8,
	CLK_TOP_DSP_26M,
	CLK_TOP_DSP_32K,
	ADSP_CLK_MAX
};

int mt8365_adsp_init_clock(struct snd_sof_dev *sdev);
int adsp_clock_on(struct snd_sof_dev *sdev);
int adsp_clock_off(struct snd_sof_dev *sdev);
#endif
