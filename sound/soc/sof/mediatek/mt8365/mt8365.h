/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause) */

/*
 * Copyright (c) 2024 MediaTek Corporation. All rights reserved.
 *
 *  Header file for the mt8365 (Genio 350) DSP register definition
 *
 * The mt8365 carries a Cadence Tensilica HiFi 4 audio DSP -- the same core
 * generation as mt8195 -- and its in-SoC DSP config block (DSPCFG) has a
 * register-offset map identical to mt8195's (JTAGMUX 0x00, ALTRESETVEC 0x04,
 * PDEBUGPC 0x20, RESET_SW 0x24, INT2CIRQ 0x114, ...). The offsets and the
 * SW-interrupt IPC protocol below are transcribed from the open SOF mt8365
 * platform (sof/src/platform/mt8365/include/platform/drivers/mt_reg_base.h and
 * sof/src/drivers/mediatek/mt8365/ipc.c). Unlike mt8186/mt8195, mt8365 has no
 * hardware adsp-mbox: host<->DSP IPC is the in-DSPCFG SW-interrupt path
 * (DSP_RG_INT2CIRQ + the SRAM SW registers).
 */

#ifndef __MT8365_H
#define __MT8365_H

struct mtk_adsp_chip_info;
struct snd_sof_dev;

#define DSP_REG_BAR			4

/*****************************************************************************
 *                  D S P C F G   R E G I S T E R   T A B L E
 * (cfg-bar relative; cfg base = 0x1d062000 = firmware DSP_REG_BASE)
 *****************************************************************************/
#define DSP_JTAGMUX			0x0000
#define DSP_ALTRESETVEC			0x0004
#define DSP_PDEBUGDATA			0x0008
#define DSP_PDEBUGBUS0			0x000c
#define PDEBUG_ENABLE			BIT(0)
#define DSP_PDEBUGBUS1			0x0010
#define DSP_PDEBUGINST			0x0014
#define DSP_PDEBUGLS0STAT		0x0018
#define DSP_PDEBUGLS1STAT		0x001c
#define DSP_PDEBUGPC			0x0020
#define DSP_RESET_SW			0x0024 /* reset sw */
/*
 * DSP_RESET_SW bit layout is shared with mt8195 (identical HiFi 4 DSPCFG
 * block); see sound/soc/sof/mediatek/mt8195/mt8195.h.
 */
#define ADSP_BRESET_SW			BIT(0)
#define ADSP_DRESET_SW			BIT(1)
#define ADSP_RUNSTALL			BIT(3)
#define STATVECTOR_SEL			BIT(4)
#define ADSP_PWAIT			BIT(16)
#define DSP_PFAULTBUS			0x0028
#define DSP_PFAULTINFO			0x002c
#define DSP_INTFDSP			0x00d0
#define DSP_INTFDSP_CLR			0x00d4
#define DSP_SRAM_PD_SW1			0x00d8
#define DSP_SRAM_PD_SW2			0x00dc
#define DSP_OCD				0x00e0
/* host<->DSP SW-interrupt trigger (INT2CIRQ) */
#define DSP_RG_INT2CIRQ			0x0114
#define CPU2DSP_IRQ			BIT(0)
#define DSP2CPU_IRQ			BIT(1)
#define DSP2SPM_IRQ_B			BIT(2)
#define DSP_RG_INT_POL_CTL0		0x0120
#define DSP_RG_INT_EN_CTL0		0x0130
#define DSP_RG_INT_LV_CTL0		0x0140
#define DSP_RG_INT_STATUS0		0x0150
#define DSP_PDEBUGSTATUS0		0x0200

/*
 * SW registers + mailbox live in the host-shared DRAM window (SRAM1_BASE).
 * They are addressed via the DRAM (SOF_FW_BLK_TYPE_SRAM) bar at MBOX_OFFSET.
 * Layout (from the SOF mt8365 firmware memory.h):
 *   SRAM1_BASE + 0x0000 : OUTBOX (dspbox, DSP->host)   0x1000
 *   SRAM1_BASE + 0x1000 : INBOX  (hostbox, host->DSP)  0x1000
 *   SRAM1_BASE + 0x2000 : DEBUG  (SW registers)        0x0800
 */
#define MBOX_OFFSET			0x800000 /* SRAM1_BASE - DRAM_BASE */
#define MBOX_SIZE			0x1000
#define SW_REG_BASE_OFFSET		(MBOX_OFFSET + 0x2000) /* debug box base */
#define SRAM_REG_FW_STATUS		0x4
#define SRAM_REG_OP_CPU2DSP		0x8
#define SRAM_REG_OP_DSP2CPU		0xc

/* SW-interrupt IPC op codes (firmware ADSP_IPI_OP_*) */
#define ADSP_IPI_OP_REQ			0x1
#define ADSP_IPI_OP_RSP			0x2

/* DSP memories */
#define DSP_DRAM_SIZE			0x1000000 /* 16M */

/*
 * DSP -> external-DDR EMI base-address remap. The mt8365 DSP issues its DRAM
 * accesses at DSP-view DRAM_PHYS_BASE_FROM_DSP_VIEW (0x60000000); the EMI
 * fabric only routes those to physical DRAM once the host programs the
 * DSP-view-to-physical offset into DSP_EMI_MAP (infracfg_ao 0x1000181C, field
 * dsp_emi_base_remap[19:0] = (pa_dram - 0x60000000) >> 12). At reset the field
 * is 0, so DSP-view 0x60000000 maps to physical 0x60000000 -- but the adsp
 * carveout is at pa_dram (memory@61000000 on the g350 Pumpkin), so the DSP
 * reads the wrong DRAM and its reset stub faults with an invalid firmware
 * header (faultbus 0x1000000 == pa_dram - 0x60000000, the un-applied offset).
 * This is the exact analog of mt8195's adsp_memory_remap_init(): the register
 * holds a DSP-view-relative OFFSET, not an absolute base. The mt8365 DSPCFG
 * AUDIO_DSP_EMI_BASE_ADDR (cfg+0x2E0) is deprecated in the TRM in favour of
 * this register ("please use DSP_EMI_MAP, dsp_emi_base_remap").
 * MT8365 TRM v1.0 p463 (DSP_EMI_MAP) + p3291 (deprecated-companion note).
 */
#define MT8365_DSP_EMI_MAP_ADDR		0x1000181C
#define DRAM_PHYS_BASE_FROM_DSP_VIEW	0x60000000
#define DRAM_REMAP_SHIFT		12
#define DRAM_REMAP_MASK			(BIT(DRAM_REMAP_SHIFT) - 1)

/*
 * DSP SRAM POOL power-on (SYSAO block, distinct from the DSPCFG cfg region).
 * The DSP-local SRAM banks are power-gated at reset; the PD bits must be
 * cleared before the DSP is released, otherwise the DSP boots from SRAM but
 * its memory subsystem cannot reach external DRAM (faultbus on the first
 * 0x60000000 access). Ported from MediaTek's official Genio BSP (mtk-v6.6)
 * adsp_sram_power_on(): ADSP_SRAM_POOL_CON at DSP_SYSAO_BASE + 0x30, PD mask 0xf.
 */
#define DSP_SYSAO_BASE			0x1d071000
#define ADSP_SRAM_POOL_CON		(DSP_SYSAO_BASE + 0x30)
#define DSP_SRAM_POOL_PD_MASK		0xf

/*
 * DSP-view reset vector = XCHAL_RESET_VECTOR0_PADDR from the mt8365 Xtensa
 * core config (core-isa.h). The host points ALTRESETVEC here and selects it
 * with STATVECTOR_SEL so the core starts in the loaded firmware.
 */
#define ADSP_BOOTUP_ADDR		0x40020000

#define SIZE_SHARED_DRAM_DL		0x40000
#define SIZE_SHARED_DRAM_UL		0x40000
#define TOTAL_SIZE_SHARED_DRAM_FROM_TAIL	\
	(SIZE_SHARED_DRAM_DL + SIZE_SHARED_DRAM_UL)

/* suspend dsp idle check interval and timeout */
#define SUSPEND_DSP_IDLE_TIMEOUT_US		1000000
#define SUSPEND_DSP_IDLE_POLL_INTERVAL_US	500

void mt8365_sof_hifixdsp_boot_sequence(struct snd_sof_dev *sdev, u32 boot_addr);
void mt8365_sof_hifixdsp_shutdown(struct snd_sof_dev *sdev);
#endif
