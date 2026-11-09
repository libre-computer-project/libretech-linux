// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
//
// Copyright(c) 2024 MediaTek Inc. All rights reserved.
//
// Hardware interface for audio DSP on mt8365 (Genio 350)
//
// mt8365 carries a Cadence Tensilica HiFi 4 audio DSP -- the same core
// generation as mt8195, and it shares mt8195's DSPCFG register-offset map and
// HiFi-4 reset/boot sequence. It diverges from both mt8186 and mt8195 in two
// ways, transcribed from the open SOF mt8365 platform:
//   - the DSP node exposes only "cfg" + "sram" (no "sec"/"bus" banks), and the
//     DRAM mapping is identity (DSP view == AP phys == 0x60000000), so there is
//     no EMI-remap step;
//   - host<->DSP IPC is the in-DSPCFG SW-interrupt path (DSP_RG_INT2CIRQ + the
//     SRAM SW registers SRAM_REG_OP_CPU2DSP/DSP2CPU), NOT the hardware
//     adsp-mbox / mtk-adsp-ipc that mt8186/mt8195 use.
//
// The DSP clock setup (mt8365-clk.c: the DSPPLL + DSP clocks) is ported from
// MediaTek's official Genio BSP kernel (gitlab.com/mediatek/aiot/bsp/linux,
// branch mtk-v6.6) -- it is the piece our from-scratch RE lacked. Without the
// DSPPLL enabled the DSP boots from SRAM but faults on its first external-DDR
// access (DSP-view 0x60000000) because its EMI/bus clock path was never on.

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/module.h>
#include <linux/workqueue.h>

#include <sound/sof.h>
#include <sound/sof/xtensa.h>
#include "../../ops.h"
#include "../../sof-audio.h"
#include "../../sof-of-dev.h"
#include "../adsp_helper.h"
#include "../mtk-adsp-common.h"
#include "mt8365.h"
#include "mt8365-clk.h"

/*
 * mt8365-specific IPC state, stashed in adsp_priv->private_data. mt8365 has no
 * hardware mailbox device, so the DSP->host notification is driven either by a
 * DT GIC interrupt (INT2CIRQ -> host) or, when the board's DT node carries no
 * "interrupts" property, by a 1 ms poll of the SRAM SW op register.
 */
struct mt8365_ipc {
	struct snd_sof_dev *sdev;
	int irq;
	bool polling;
	struct delayed_work poll_work;
};

static inline u32 mt8365_sw_reg_read(struct snd_sof_dev *sdev, u32 reg)
{
	struct adsp_priv *priv = sdev->pdata->hw_pdata;

	return readl(priv->adsp->va_dram + SW_REG_BASE_OFFSET + reg);
}

static inline void mt8365_sw_reg_write(struct snd_sof_dev *sdev, u32 reg, u32 val)
{
	struct adsp_priv *priv = sdev->pdata->hw_pdata;

	writel(val, priv->adsp->va_dram + SW_REG_BASE_OFFSET + reg);
}

static int mt8365_get_mailbox_offset(struct snd_sof_dev *sdev)
{
	return MBOX_OFFSET;
}

static int mt8365_get_window_offset(struct snd_sof_dev *sdev, u32 id)
{
	return MBOX_OFFSET;
}

/* HiFi-4 boot sequence (identical DSPCFG reset semantics to mt8195) */
void mt8365_sof_hifixdsp_boot_sequence(struct snd_sof_dev *sdev, u32 boot_addr)
{
	/* ADSP bootup base */
	snd_sof_dsp_write(sdev, DSP_REG_BAR, DSP_ALTRESETVEC, boot_addr);

	/* pull high RunStall */
	snd_sof_dsp_update_bits(sdev, DSP_REG_BAR, DSP_RESET_SW,
				ADSP_RUNSTALL, ADSP_RUNSTALL);

	/* pull high StatVectorSel to use AltResetVec */
	snd_sof_dsp_update_bits(sdev, DSP_REG_BAR, DSP_RESET_SW,
				STATVECTOR_SEL, STATVECTOR_SEL);

	/* toggle DReset & BReset : pull high */
	snd_sof_dsp_update_bits(sdev, DSP_REG_BAR, DSP_RESET_SW,
				ADSP_BRESET_SW | ADSP_DRESET_SW,
				ADSP_BRESET_SW | ADSP_DRESET_SW);

	/* about 1us by IP vendor's suggestion */
	udelay(1);

	/* pull low DReset & BReset */
	snd_sof_dsp_update_bits(sdev, DSP_REG_BAR, DSP_RESET_SW,
				ADSP_BRESET_SW | ADSP_DRESET_SW, 0);

	/* Enable PDebug */
	snd_sof_dsp_update_bits(sdev, DSP_REG_BAR, DSP_PDEBUGBUS0,
				PDEBUG_ENABLE, PDEBUG_ENABLE);

	/* release RunStall */
	snd_sof_dsp_update_bits(sdev, DSP_REG_BAR, DSP_RESET_SW,
				ADSP_RUNSTALL, 0);
}

void mt8365_sof_hifixdsp_shutdown(struct snd_sof_dev *sdev)
{
	/* RunStall pull high again to reset */
	snd_sof_dsp_update_bits(sdev, DSP_REG_BAR, DSP_RESET_SW,
				ADSP_RUNSTALL, ADSP_RUNSTALL);

	/* pull high DReset & BReset */
	snd_sof_dsp_update_bits(sdev, DSP_REG_BAR, DSP_RESET_SW,
				ADSP_BRESET_SW | ADSP_DRESET_SW,
				ADSP_BRESET_SW | ADSP_DRESET_SW);
}

/*
 * SW-interrupt IPC.
 *
 * host -> DSP: write the message to the hostbox, set OP_CPU2DSP, and pulse
 * CPU2DSP_IRQ in INT2CIRQ (mirrors the firmware irq_handler which reads
 * OP_CPU2DSP and clears CPU2DSP_IRQ).
 *
 * DSP -> host: the DSP writes OP_DSP2CPU (REQ for a request/notification, RSP
 * for a reply to a host command) and pulses DSP2CPU_IRQ; we read OP_DSP2CPU,
 * dispatch it into the SOF IPC RX path, ack by clearing OP_DSP2CPU and
 * DSP2CPU_IRQ, and for a REQ send an RSP back.
 */
static int mt8365_send_msg(struct snd_sof_dev *sdev, struct snd_sof_ipc_msg *msg)
{
	sof_mailbox_write(sdev, sdev->host_box.offset, msg->msg_data,
			  msg->msg_size);

	mt8365_sw_reg_write(sdev, SRAM_REG_OP_CPU2DSP, ADSP_IPI_OP_REQ);
	snd_sof_dsp_update_bits(sdev, DSP_REG_BAR, DSP_RG_INT2CIRQ,
				CPU2DSP_IRQ, CPU2DSP_IRQ);

	return 0;
}

static void mt8365_ipc_handle(struct snd_sof_dev *sdev)
{
	unsigned long flags;
	u32 op;

	op = mt8365_sw_reg_read(sdev, SRAM_REG_OP_DSP2CPU);

	if (op == ADSP_IPI_OP_REQ) {
		u32 panic_code;

		/* is this a panic message from the DSP? */
		sof_mailbox_read(sdev, sdev->debug_box.offset + 4,
				 &panic_code, sizeof(panic_code));

		if ((panic_code & SOF_IPC_PANIC_MAGIC_MASK) == SOF_IPC_PANIC_MAGIC) {
			snd_sof_dsp_panic(sdev, panic_code, true);
		} else {
			snd_sof_ipc_msgs_rx(sdev);

			/* tell the DSP the request is done */
			mt8365_sw_reg_write(sdev, SRAM_REG_OP_CPU2DSP,
					    ADSP_IPI_OP_RSP);
			snd_sof_dsp_update_bits(sdev, DSP_REG_BAR,
						DSP_RG_INT2CIRQ,
						CPU2DSP_IRQ, CPU2DSP_IRQ);
		}
	} else if (op == ADSP_IPI_OP_RSP) {
		spin_lock_irqsave(&sdev->ipc_lock, flags);
		snd_sof_ipc_process_reply(sdev, 0);
		spin_unlock_irqrestore(&sdev->ipc_lock, flags);
	} else {
		/* nothing pending */
		return;
	}

	/* consume the op and clear the DSP->host interrupt */
	mt8365_sw_reg_write(sdev, SRAM_REG_OP_DSP2CPU, 0);
	snd_sof_dsp_update_bits(sdev, DSP_REG_BAR, DSP_RG_INT2CIRQ,
				DSP2CPU_IRQ, 0);
}

static irqreturn_t mt8365_irq_thread(int irq, void *data)
{
	struct snd_sof_dev *sdev = data;

	mt8365_ipc_handle(sdev);

	return IRQ_HANDLED;
}

static void mt8365_ipc_poll_work(struct work_struct *work)
{
	struct mt8365_ipc *ipc = container_of(work, struct mt8365_ipc,
					      poll_work.work);

	mt8365_ipc_handle(ipc->sdev);

	if (ipc->polling)
		schedule_delayed_work(&ipc->poll_work, msecs_to_jiffies(1));
}

static int platform_parse_resource(struct platform_device *pdev, void *data)
{
	struct resource *mmio;
	struct resource res;
	struct device *dev = &pdev->dev;
	struct mtk_adsp_chip_info *adsp = data;
	int ret;

	ret = of_reserved_mem_device_init(dev);
	if (ret) {
		dev_err(dev, "of_reserved_mem_device_init failed\n");
		return ret;
	}

	ret = of_reserved_mem_region_to_resource(dev->of_node, 1, &res);
	if (ret) {
		dev_err(dev, "of_address_to_resource sysmem failed\n");
		return ret;
	}

	adsp->pa_dram = (phys_addr_t)res.start;
	adsp->dramsize = resource_size(&res);
	if (adsp->pa_dram & DRAM_REMAP_MASK) {
		dev_err(dev, "adsp memory(%#x) is not 4K-aligned\n",
			(u32)adsp->pa_dram);
		return -EINVAL;
	}

	if (adsp->dramsize < TOTAL_SIZE_SHARED_DRAM_FROM_TAIL) {
		dev_err(dev, "adsp memory(%#x) is not enough for share\n",
			adsp->dramsize);
		return -EINVAL;
	}

	dev_dbg(dev, "dram pbase=%pa, dramsize=%#x\n",
		&adsp->pa_dram, adsp->dramsize);

	/* Parse CFG base */
	mmio = platform_get_resource_byname(pdev, IORESOURCE_MEM, "cfg");
	if (!mmio) {
		dev_err(dev, "no ADSP-CFG register resource\n");
		return -ENXIO;
	}

	adsp->va_cfgreg = devm_ioremap_resource(dev, mmio);
	if (IS_ERR(adsp->va_cfgreg))
		return PTR_ERR(adsp->va_cfgreg);

	adsp->pa_cfgreg = (phys_addr_t)mmio->start;
	adsp->cfgregsize = resource_size(mmio);

	dev_dbg(dev, "cfgreg-vbase=%p, cfgregsize=%#x\n",
		adsp->va_cfgreg, adsp->cfgregsize);

	/* Parse SRAM */
	mmio = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sram");
	if (!mmio) {
		dev_err(dev, "no SRAM resource\n");
		return -ENXIO;
	}

	adsp->pa_sram = (phys_addr_t)mmio->start;
	adsp->sramsize = resource_size(mmio);

	dev_dbg(dev, "sram pbase=%pa,%#x\n", &adsp->pa_sram, adsp->sramsize);

	return 0;
}

/*
 * Power on the DSP-local SRAM pool (SYSAO ADSP_SRAM_POOL_CON) before the DSP is
 * released from reset. The SRAM banks are power-gated at reset; clearing the PD
 * bits powers them on. Ported verbatim from MediaTek's official Genio BSP
 * (mtk-v6.6) adsp_sram_power_on(). This is a SEPARATE register block from the
 * DSPCFG cfg region and from the DSPPLL clock setup -- our from-scratch driver
 * had neither the clock nor this SRAM-pool power-on; the clock alone (at full
 * 600 MHz) still faults on the first DRAM access, so this is the second missing
 * host-side piece the official driver performs and we did not.
 */
static int mt8365_adsp_sram_power_on(struct device *dev, bool on)
{
	void __iomem *va_dspsysreg;
	u32 srampool_con;

	va_dspsysreg = ioremap(ADSP_SRAM_POOL_CON, 0x4);
	if (!va_dspsysreg) {
		dev_err(dev, "failed to ioremap sram pool con %#x\n",
			ADSP_SRAM_POOL_CON);
		return -ENOMEM;
	}

	srampool_con = readl(va_dspsysreg);
	if (on)
		writel(srampool_con & ~DSP_SRAM_POOL_PD_MASK, va_dspsysreg);
	else
		writel(srampool_con | DSP_SRAM_POOL_PD_MASK, va_dspsysreg);

	iounmap(va_dspsysreg);
	return 0;
}

/*
 * Program the DSP -> external-DDR EMI base-address remap (DSP_EMI_MAP) before
 * the DSP is released from reset. The register holds a DSP-view-relative offset
 * (see mt8365.h): DSP-view DRAM_PHYS_BASE_FROM_DSP_VIEW (0x60000000) is routed
 * to physical (0x60000000 + (dsp_emi_base_remap << 12)), so the value is
 * (pa_dram - 0x60000000) >> 12 -- exactly mt8195's adsp_memory_remap_init().
 * Without it the DSP reset stub faults on its first DRAM access (faultbus
 * 0x1000000) and firmware boot times out before fw_ready, so no SOF card.
 */
static int mt8365_adsp_memory_remap_init(struct device *dev,
					 struct mtk_adsp_chip_info *adsp)
{
	void __iomem *vaddr_emi_map;
	u32 remap;

	vaddr_emi_map = devm_ioremap(dev, MT8365_DSP_EMI_MAP_ADDR, 0x4);
	if (!vaddr_emi_map) {
		dev_err(dev, "failed to ioremap DSP_EMI_MAP %#x\n",
			MT8365_DSP_EMI_MAP_ADDR);
		return -ENOMEM;
	}

	remap = ((u32)adsp->pa_dram - DRAM_PHYS_BASE_FROM_DSP_VIEW) >>
		DRAM_REMAP_SHIFT;
	writel(remap, vaddr_emi_map);
	if (readl(vaddr_emi_map) != remap) {
		dev_err(dev, "DSP_EMI_MAP write failed: %#x\n",
			readl(vaddr_emi_map));
		return -EIO;
	}
	dev_dbg(dev, "DSP_EMI_MAP set to %#x (pa_dram %pa)\n",
		remap, &adsp->pa_dram);

	return 0;
}

static int mt8365_run(struct snd_sof_dev *sdev)
{
	dev_dbg(sdev->dev, "HIFI4DSP boot from base : 0x%08X\n", ADSP_BOOTUP_ADDR);
	mt8365_sof_hifixdsp_boot_sequence(sdev, ADSP_BOOTUP_ADDR);

	return 0;
}

static int mt8365_dsp_probe(struct snd_sof_dev *sdev)
{
	struct platform_device *pdev = to_platform_device(sdev->dev);
	struct adsp_priv *priv;
	struct mt8365_ipc *ipc;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	sdev->pdata->hw_pdata = priv;
	priv->dev = sdev->dev;
	priv->sdev = sdev;

	priv->adsp = devm_kzalloc(&pdev->dev, sizeof(struct mtk_adsp_chip_info),
				  GFP_KERNEL);
	if (!priv->adsp)
		return -ENOMEM;

	ret = platform_parse_resource(pdev, priv->adsp);
	if (ret)
		return ret;

	/*
	 * Enable the DSP clocks (DSPPLL + DSP_SEL + DSPPLL dividers +
	 * PDN_DSP_26M/32K). This is the load-bearing gate: without the DSPPLL
	 * enabled the DSP reaches SRAM but not external DRAM (mt8365-clk.c,
	 * ported from MediaTek's official Genio BSP mtk-v6.6).
	 */
	ret = mt8365_adsp_init_clock(sdev);
	if (ret) {
		dev_err(sdev->dev, "mt8365_adsp_init_clock failed: %d\n", ret);
		return ret;
	}

	ret = adsp_clock_on(sdev);
	if (ret) {
		dev_err(sdev->dev, "adsp_clock_on failed: %d\n", ret);
		return ret;
	}

	/*
	 * Power on the DSP SRAM pool before the EMI remap + DSP release. The
	 * official BSP probe order is clock-on -> sram-power-on -> remap-init.
	 */
	ret = mt8365_adsp_sram_power_on(&pdev->dev, true);
	if (ret) {
		dev_err(sdev->dev, "mt8365_adsp_sram_power_on failed: %d\n", ret);
		goto err_clk_disable;
	}

	/* Open the DSP->external-DDR EMI window before the DSP is run. */
	ret = mt8365_adsp_memory_remap_init(&pdev->dev, priv->adsp);
	if (ret) {
		dev_err(sdev->dev, "mt8365_adsp_memory_remap_init failed: %d\n",
			ret);
		goto err_clk_disable;
	}

	sdev->bar[SOF_FW_BLK_TYPE_IRAM] = devm_ioremap(sdev->dev,
						       priv->adsp->pa_sram,
						       priv->adsp->sramsize);
	if (!sdev->bar[SOF_FW_BLK_TYPE_IRAM]) {
		dev_err(sdev->dev, "failed to ioremap base %pa size %#x\n",
			&priv->adsp->pa_sram, priv->adsp->sramsize);
		ret = -EINVAL;
		goto err_clk_disable;
	}

	priv->adsp->va_sram = sdev->bar[SOF_FW_BLK_TYPE_IRAM];

	sdev->bar[SOF_FW_BLK_TYPE_SRAM] = devm_ioremap(sdev->dev,
						       priv->adsp->pa_dram,
						       priv->adsp->dramsize);
	if (!sdev->bar[SOF_FW_BLK_TYPE_SRAM]) {
		dev_err(sdev->dev, "failed to ioremap base %pa size %#x\n",
			&priv->adsp->pa_dram, priv->adsp->dramsize);
		ret = -EINVAL;
		goto err_clk_disable;
	}
	priv->adsp->va_dram = sdev->bar[SOF_FW_BLK_TYPE_SRAM];

	sdev->bar[DSP_REG_BAR] = priv->adsp->va_cfgreg;

	sdev->mmio_bar = SOF_FW_BLK_TYPE_SRAM;
	sdev->mailbox_bar = SOF_FW_BLK_TYPE_SRAM;

	/* set default mailbox offset for FW ready message */
	sdev->dsp_box.offset = mt8365_get_mailbox_offset(sdev);

	/* clear the SW IPC op registers before boot */
	mt8365_sw_reg_write(sdev, SRAM_REG_FW_STATUS, 0);
	mt8365_sw_reg_write(sdev, SRAM_REG_OP_CPU2DSP, 0);
	mt8365_sw_reg_write(sdev, SRAM_REG_OP_DSP2CPU, 0);

	/* set up the SW-interrupt IPC receive path */
	ipc = devm_kzalloc(&pdev->dev, sizeof(*ipc), GFP_KERNEL);
	if (!ipc) {
		ret = -ENOMEM;
		goto err_clk_disable;
	}
	ipc->sdev = sdev;
	priv->private_data = ipc;

	ipc->irq = platform_get_irq_optional(pdev, 0);
	if (ipc->irq > 0) {
		ret = devm_request_threaded_irq(&pdev->dev, ipc->irq, NULL,
						mt8365_irq_thread, IRQF_ONESHOT,
						"adsp-ipc", sdev);
		if (ret) {
			dev_err(sdev->dev, "failed to request ipc irq %d: %d\n",
				ipc->irq, ret);
			goto err_clk_disable;
		}
		dev_dbg(sdev->dev, "mt8365 adsp IPC on GIC irq %d\n", ipc->irq);
	} else {
		/*
		 * No DT "interrupts" for the DSP->host INT2CIRQ line: drive the
		 * SW-interrupt RX by polling the SRAM op register.
		 */
		ipc->polling = true;
		INIT_DELAYED_WORK(&ipc->poll_work, mt8365_ipc_poll_work);
		schedule_delayed_work(&ipc->poll_work, msecs_to_jiffies(1));
		dev_info(sdev->dev, "mt8365 adsp IPC has no GIC irq; polling\n");
	}

	return 0;

err_clk_disable:
	adsp_clock_off(sdev);
	return ret;
}

static void mt8365_ipc_stop(struct snd_sof_dev *sdev)
{
	struct adsp_priv *priv = sdev->pdata->hw_pdata;
	struct mt8365_ipc *ipc = priv ? priv->private_data : NULL;

	if (ipc && ipc->polling) {
		ipc->polling = false;
		cancel_delayed_work_sync(&ipc->poll_work);
	}
}

static void mt8365_dsp_remove(struct snd_sof_dev *sdev)
{
	mt8365_ipc_stop(sdev);
	mt8365_sof_hifixdsp_shutdown(sdev);
	adsp_clock_off(sdev);
}

static int mt8365_dsp_shutdown(struct snd_sof_dev *sdev)
{
	return snd_sof_suspend(sdev->dev);
}

static int mt8365_dsp_suspend(struct snd_sof_dev *sdev, u32 target_state)
{
	mt8365_sof_hifixdsp_shutdown(sdev);

	return adsp_clock_off(sdev);
}

static int mt8365_dsp_resume(struct snd_sof_dev *sdev)
{
	return adsp_clock_on(sdev);
}

static void mt8365_adsp_dump(struct snd_sof_dev *sdev, u32 flags)
{
	u32 dbg_pc, dbg_data, dbg_inst, dbg_ls0stat, faultbus, faultinfo, swrest;

	dbg_pc = snd_sof_dsp_read(sdev, DSP_REG_BAR, DSP_PDEBUGPC);
	dbg_data = snd_sof_dsp_read(sdev, DSP_REG_BAR, DSP_PDEBUGDATA);
	dbg_inst = snd_sof_dsp_read(sdev, DSP_REG_BAR, DSP_PDEBUGINST);
	dbg_ls0stat = snd_sof_dsp_read(sdev, DSP_REG_BAR, DSP_PDEBUGLS0STAT);
	faultbus = snd_sof_dsp_read(sdev, DSP_REG_BAR, DSP_PFAULTBUS);
	faultinfo = snd_sof_dsp_read(sdev, DSP_REG_BAR, DSP_PFAULTINFO);
	swrest = snd_sof_dsp_read(sdev, DSP_REG_BAR, DSP_RESET_SW);

	dev_info(sdev->dev, "adsp dump : pc %#x, data %#x, inst %#x, swrest %#x",
		 dbg_pc, dbg_data, dbg_inst, swrest);
	dev_info(sdev->dev, "ls0stat %#x, faultbus %#x, faultinfo %#x",
		 dbg_ls0stat, faultbus, faultinfo);

	mtk_adsp_dump(sdev, flags);
}

/*
 * DL2/DL3/UL4/UL5 are registered as SOF FE (same shape as mt8188), so the
 * corresponding SOF BE completes the pipeline.
 */
static struct snd_soc_dai_driver mt8365_dai[] = {
{
	.name = "SOF_DL2",
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
	},
},
{
	.name = "SOF_DL3",
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
	},
},
{
	.name = "SOF_UL4",
	.capture = {
		.channels_min = 1,
		.channels_max = 2,
	},
},
{
	.name = "SOF_UL5",
	.capture = {
		.channels_min = 1,
		.channels_max = 2,
	},
},
};

/* mt8365 ops */
static const struct snd_sof_dsp_ops sof_mt8365_ops = {
	/* probe and remove */
	.probe		= mt8365_dsp_probe,
	.remove		= mt8365_dsp_remove,
	.shutdown	= mt8365_dsp_shutdown,

	/* DSP core boot */
	.run		= mt8365_run,

	/* Block IO */
	.block_read	= sof_block_read,
	.block_write	= sof_block_write,

	/* Mailbox IO */
	.mailbox_read	= sof_mailbox_read,
	.mailbox_write	= sof_mailbox_write,

	/* Register IO */
	.write		= sof_io_write,
	.read		= sof_io_read,
	.write64	= sof_io_write64,
	.read64		= sof_io_read64,

	/* ipc */
	.send_msg		= mt8365_send_msg,
	.get_mailbox_offset	= mt8365_get_mailbox_offset,
	.get_window_offset	= mt8365_get_window_offset,
	.ipc_msg_data		= sof_ipc_msg_data,
	.set_stream_data_offset = sof_set_stream_data_offset,

	/* misc */
	.get_bar_index	= mtk_adsp_get_bar_index,

	/* stream callbacks */
	.pcm_open	= sof_stream_pcm_open,
	.pcm_hw_params	= mtk_adsp_stream_pcm_hw_params,
	.pcm_pointer	= mtk_adsp_stream_pcm_pointer,
	.pcm_close	= sof_stream_pcm_close,

	/* firmware loading */
	.load_firmware	= snd_sof_load_firmware_memcpy,

	/* Firmware ops */
	.dsp_arch_ops = &sof_xtensa_arch_ops,

	/* Debug information */
	.dbg_dump = mt8365_adsp_dump,
	.debugfs_add_region_item = snd_sof_debugfs_add_region_item_iomem,

	/* DAI drivers */
	.drv		= mt8365_dai,
	.num_drv	= ARRAY_SIZE(mt8365_dai),

	/* PM */
	.suspend	= mt8365_dsp_suspend,
	.resume		= mt8365_dsp_resume,

	/* ALSA HW info flags */
	.hw_info =	SNDRV_PCM_INFO_MMAP |
			SNDRV_PCM_INFO_MMAP_VALID |
			SNDRV_PCM_INFO_INTERLEAVED |
			SNDRV_PCM_INFO_PAUSE |
			SNDRV_PCM_INFO_NO_PERIOD_WAKEUP,
};

static struct snd_sof_of_mach sof_mt8365_machs[] = {
	{
		.compatible = "mediatek,mt8365",
		.sof_tplg_filename = "sof-mt8365.tplg",
	},
	{}
};

static const struct sof_dev_desc sof_of_mt8365_desc = {
	.of_machines = sof_mt8365_machs,
	.ipc_supported_mask	= BIT(SOF_IPC_TYPE_3),
	.ipc_default		= SOF_IPC_TYPE_3,
	.default_fw_path = {
		[SOF_IPC_TYPE_3] = "mediatek/sof",
	},
	.default_tplg_path = {
		[SOF_IPC_TYPE_3] = "mediatek/sof-tplg",
	},
	.default_fw_filename = {
		[SOF_IPC_TYPE_3] = "sof-mt8365.ri",
	},
	.nocodec_tplg_filename = "sof-mt8365-nocodec.tplg",
	.ops = &sof_mt8365_ops,
	.ipc_timeout = 1000,
};

static const struct of_device_id sof_of_mt8365_ids[] = {
	{ .compatible = "mediatek,mt8365-dsp", .data = &sof_of_mt8365_desc},
	{ }
};
MODULE_DEVICE_TABLE(of, sof_of_mt8365_ids);

/* DT driver definition */
static struct platform_driver snd_sof_of_mt8365_driver = {
	.probe = sof_of_probe,
	.remove = sof_of_remove,
	.shutdown = sof_of_shutdown,
	.driver = {
	.name = "sof-audio-of-mt8365",
		.pm = pm_ptr(&sof_of_pm),
		.of_match_table = sof_of_mt8365_ids,
	},
};
module_platform_driver(snd_sof_of_mt8365_driver);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("SOF support for MT8365 platforms");
MODULE_IMPORT_NS("SND_SOC_SOF_XTENSA");
MODULE_IMPORT_NS("SND_SOC_SOF_MTK_COMMON");
