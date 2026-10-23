// SPDX-License-Identifier: GPL-2.0-only
/*
 * Amlogic SD/eMMC driver for the GX/S905 family SoCs
 *
 * Copyright (c) 2016 BayLibre, SAS.
 * Author: Kevin Hilman <khilman@baylibre.com>
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/iopoll.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/ioport.h>
#include <linux/dma-mapping.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/io.h>
#include <linux/seq_file.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/interrupt.h>
#include <linux/bitfield.h>
#include <linux/bitmap.h>
#include <linux/pinctrl/consumer.h>

#define DRIVER_NAME "meson-gx-mmc"

/*
 * eMMC HS400 / HS400-ES opt-in. Default OFF: the V3 data-strobe port is not
 * validated on hardware, and enabling it on a board whose SPI NOR shares the
 * eMMC d4/d5/d6 lines silently corrupts 8-bit HS400 writes. The strobe state
 * is torn down on every timing change (meson_mmc_check_resampling()), but the
 * driver has no .hs400_downgrade, so the core's HS400 -> HS200 downgrade runs
 * its status polls with the strobe still armed; and a failed HS400 tuning is
 * returned to the core, which then frees the card (no step-down to HS200), so
 * an eMMC rootfs is lost. Only set hs400_enable=1 on hardware where HS400 is
 * safe (SPI NOR de-populated) and recoverable.
 */
static bool hs400_enable;
module_param(hs400_enable, bool, 0644);
MODULE_PARM_DESC(hs400_enable,
		 "Enable experimental eMMC HS400/HS400-ES on V3 controllers (default 0; UNTESTED, needs SPI NOR de-populated)");

#define SD_EMMC_CLOCK 0x0
#define   CLK_DIV_MASK GENMASK(5, 0)
#define   CLK_SRC_MASK GENMASK(7, 6)
#define   CLK_CORE_PHASE_MASK GENMASK(9, 8)
#define   CLK_TX_PHASE_MASK GENMASK(11, 10)
#define   CLK_RX_PHASE_MASK GENMASK(13, 12)
#define   CLK_PHASE_0 0
#define   CLK_PHASE_180 2
#define   CLK_V2_TX_DELAY_MASK GENMASK(19, 16)
#define   CLK_V2_RX_DELAY_MASK GENMASK(23, 20)
#define   CLK_V2_ALWAYS_ON BIT(24)
#define   CLK_V2_IRQ_SDIO_SLEEP BIT(25)

#define   CLK_V3_TX_DELAY_MASK GENMASK(21, 16)
#define   CLK_V3_RX_DELAY_MASK GENMASK(27, 22)
#define   CLK_V3_ALWAYS_ON BIT(28)
#define   CLK_V3_IRQ_SDIO_SLEEP BIT(29)

#define   CLK_TX_DELAY_MASK(h)		(h->data->tx_delay_mask)
#define   CLK_RX_DELAY_MASK(h)		(h->data->rx_delay_mask)
#define   CLK_ALWAYS_ON(h)		(h->data->always_on)
#define   CLK_IRQ_SDIO_SLEEP(h)		(h->data->irq_sdio_sleep)

#define SD_EMMC_DELAY 0x4
#define SD_EMMC_ADJUST 0x8
#define   ADJUST_ADJ_DELAY_MASK GENMASK(21, 16)
#define   ADJUST_DS_EN BIT(15)
#define   ADJUST_ADJ_EN BIT(13)

#define SD_EMMC_DELAY1 0x4
#define SD_EMMC_DELAY2 0x8
#define   DELAY2_CMD_MASK GENMASK(29, 24)	/* CMD-line delay tap in DELAY2 */
#define SD_EMMC_V3_ADJUST 0xc
#define   CALI_SEL_MASK GENMASK(11, 8)	/* eye/clk calibration lane: 0-7 = D0-D7, 8 = CMD */
#define   CALI_ENABLE BIT(12)		/* enable the eye/clk calibration engine */

#define SD_EMMC_CALOUT 0x10

/*
 * Hardware eye/clock measurement engine (BSP aml_sd.h; unused by mainline).
 * EYETEST sweeps the controller's RX sampling point across a read transfer and
 * reports, per data line, a 64-bit pass bitmap (OUT1<<32 | OUT0); CLKTEST counts
 * the actual bus clock. Only ever driven from the debugfs eye_scan diagnostic
 * below -- never in the tuning path. The EYETEST/CLKTEST *control* bits (below)
 * live in INTF3.
 */
#define SD_EMMC_CLKTEST_LOG 0x24
#define   CLKTEST_DONE BIT(31)		/* clock measurement complete */
#define SD_EMMC_CLKTEST_OUT 0x28	/* measured clock count (>> 8 = cycles) */
#define SD_EMMC_EYETEST_LOG 0x2c
#define   EYETEST_DONE BIT(31)		/* eye measurement complete */
#define SD_EMMC_EYETEST_OUT0 0x30	/* eye pass bitmap, taps 31:0 */
#define SD_EMMC_EYETEST_OUT1 0x34	/* eye pass bitmap, taps 63:32 */

/*
 * INTF3 (0x38) carries the eMMC data-strobe controls used by HS400. BSP:
 * aml_sd.h:420-427. SD_INTF3 enables the device data-strobe input path;
 * DS_SHT_M / DS_SHT_EXP select the strobe-to-data sampling shift. The BSP
 * meson_mmc_check_resampling() HS400 branch only sets SD_INTF3 and leaves the
 * DS_SHT_* shift fields at their reset default 0, so we mirror that -- the
 * masks are defined so the strobe teardown can clear any stale shift.
 */
#define SD_EMMC_INTF3 0x38
#define   CLKTEST_EXP_MASK GENMASK(4, 0)	/* CLKTEST window exponent (BSP: 8) */
#define   CLKTEST_ON BIT(5)		/* start clock measurement */
#define   EYETEST_EXP_MASK GENMASK(10, 6)	/* EYETEST window exponent (BSP: 7 data / 3 CMD) */
#define   EYETEST_ON BIT(11)		/* start eye measurement */
#define   DS_SHT_M_MASK GENMASK(17, 12)	/* data strobe shift mantissa */
#define   DS_SHT_EXP_MASK GENMASK(21, 18)	/* data strobe shift exponent */
#define   SD_INTF3 BIT(22)		/* enable data-strobe input interface */

#define SD_EMMC_START 0x40
#define   START_DESC_INIT BIT(0)
#define   START_DESC_BUSY BIT(1)
#define   START_DESC_ADDR_MASK GENMASK(31, 2)

#define SD_EMMC_CFG 0x44
#define   CFG_BUS_WIDTH_MASK GENMASK(1, 0)
#define   CFG_BUS_WIDTH_1 0x0
#define   CFG_BUS_WIDTH_4 0x1
#define   CFG_BUS_WIDTH_8 0x2
#define   CFG_DDR BIT(2)
#define   CFG_BLK_LEN_MASK GENMASK(7, 4)
#define   CFG_RESP_TIMEOUT_MASK GENMASK(11, 8)
#define   CFG_RC_CC_MASK GENMASK(15, 12)
#define   CFG_STOP_CLOCK BIT(22)
#define   CFG_CLK_ALWAYS_ON BIT(18)
#define   CFG_CHK_DS BIT(20)
#define   CFG_AUTO_CLK BIT(23)
#define   CFG_ERR_ABORT BIT(27)

#define SD_EMMC_STATUS 0x48
#define   STATUS_BUSY BIT(31)
#define   STATUS_DESC_BUSY BIT(30)
#define   STATUS_DATI GENMASK(23, 16)

#define SD_EMMC_IRQ_EN 0x4c
#define   IRQ_RXD_ERR_MASK GENMASK(7, 0)
#define   IRQ_TXD_ERR BIT(8)
#define   IRQ_DESC_ERR BIT(9)
#define   IRQ_RESP_ERR BIT(10)
#define   IRQ_CRC_ERR \
	(IRQ_RXD_ERR_MASK | IRQ_TXD_ERR | IRQ_DESC_ERR | IRQ_RESP_ERR)
#define   IRQ_RESP_TIMEOUT BIT(11)
#define   IRQ_DESC_TIMEOUT BIT(12)
#define   IRQ_TIMEOUTS \
	(IRQ_RESP_TIMEOUT | IRQ_DESC_TIMEOUT)
#define   IRQ_END_OF_CHAIN BIT(13)
#define   IRQ_RESP_STATUS BIT(14)
#define   IRQ_SDIO BIT(15)
/*
 * CFG_CMD_SETUP lives in the IRQ_EN register (0x4c), not CFG. BSP:
 * aml_sd.h:482. HS400 sets it so the command line is launched with the
 * data-strobe-aligned setup timing. It is above the interrupt-enable bits
 * (0..15), so the RMW that sets/clears it preserves IRQ_EN_MASK.
 */
#define   CFG_CMD_SETUP BIT(17)
#define   IRQ_EN_MASK \
	(IRQ_CRC_ERR | IRQ_TIMEOUTS | IRQ_END_OF_CHAIN)

#define SD_EMMC_CMD_CFG 0x50
#define SD_EMMC_CMD_ARG 0x54
#define SD_EMMC_CMD_DAT 0x58
#define SD_EMMC_CMD_RSP 0x5c
#define SD_EMMC_CMD_RSP1 0x60
#define SD_EMMC_CMD_RSP2 0x64
#define SD_EMMC_CMD_RSP3 0x68

#define SD_EMMC_RXD 0x94
#define SD_EMMC_TXD 0x94
#define SD_EMMC_LAST_REG SD_EMMC_TXD

#define SD_EMMC_SRAM_DATA_BUF_LEN 1536
#define SD_EMMC_SRAM_DATA_BUF_OFF 0x200

#define SD_EMMC_CFG_BLK_SIZE 512 /* internal buffer max: 512 bytes */
#define SD_EMMC_CFG_RESP_TIMEOUT 256 /* in clock cycles */
#define SD_EMMC_CMD_TIMEOUT 1024 /* in ms */
#define SD_EMMC_CMD_TIMEOUT_DATA 4096 /* in ms */
#define SD_EMMC_CFG_CMD_GAP 16 /* in clock cycles */
#define SD_EMMC_DESC_BUF_LEN PAGE_SIZE

#define SD_EMMC_PRE_REQ_DONE BIT(0)
#define SD_EMMC_DESC_CHAIN_MODE BIT(1)

#define MUX_CLK_NUM_PARENTS 2

/*
 * TUNING_NUM_PER_POINT - number of CMD21 transfers that must all pass for a
 * tuning point (an RX delay tap) to count as good. BSP: aml_sd.h:573.
 */
#define TUNING_NUM_PER_POINT 40

/*
 * Per-line 6-bit RX delay packing helpers. BSP: meson-g12a-mmc.c:88-101.
 * DELAY1 packs five 6-bit data-line delays (d0..d4 at bits 0/6/12/18/24);
 * DELAY2 packs d5..d7 (bits 0/6/12) plus the CMD-line delay (bits 24..29,
 * DELAY2_CMD_MASK). The DDR200 eye scan writes the same tap to every data
 * line and leaves the CMD-line delay clear.
 */
static inline u32 aml_mv_dly1(u32 x)
{
	return (x) | ((x) << 6) | ((x) << 12) | ((x) << 18) | ((x) << 24);
}

static inline u32 aml_mv_dly2(u32 x)
{
	return (x) | ((x) << 6) | ((x) << 12) | ((x) << 24);
}

struct meson_mmc_data {
	unsigned int tx_delay_mask;
	unsigned int rx_delay_mask;
	unsigned int always_on;
	unsigned int adjust;
	unsigned int irq_sdio_sleep;
};

struct sd_emmc_desc {
	u32 cmd_cfg;
	u32 cmd_arg;
	u32 cmd_data;
	u32 cmd_resp;
};

struct meson_host {
	struct	device		*dev;
	const struct meson_mmc_data *data;
	struct	mmc_host	*mmc;
	struct	mmc_command	*cmd;

	void __iomem *regs;
	struct clk *mux_clk;
	struct clk *mmc_clk;
	struct clk *clkin0;	/* programmable mux parent, raised for DDR200 400 MHz */
	unsigned long clkin0_rate;	/* clkin0 as configured at probe (the 24 MHz
					 * the low-rate path relies on); restored
					 * after a DDR200 raise */
	unsigned long req_rate;
	bool ddr;
	unsigned char timing;	/* last ios->timing, to skip no-op set_ios */

	unsigned int ddr200_clk_ceiling;	/* 0 = full DDR200 rate; else the
						 * max SD clock to use, lowered on
						 * CRC storms; re-armed on a fresh
						 * (re-)init */
	unsigned int sdr104_clk_ceiling;	/* same, for SDR104: lets the
						 * clock ladder step 200->166->...
						 * within SDR104 before the SD
						 * core drops the protocol grade */

	bool dram_access_quirk;

	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_clk_gate;

	unsigned int bounce_buf_size;
	void *bounce_buf;
	void __iomem *bounce_iomem_buf;
	dma_addr_t bounce_dma_addr;
	struct sd_emmc_desc *descs;
	dma_addr_t descs_dma_addr;

	int irq;

	bool needs_pre_post_req;

	/* eye-scan sweep snapshot (mmc_eye_ops backend, drivers/mmc/core/eye.c) */
	u32 eye_save_clk, eye_save_dly1, eye_save_dly2, eye_save_adj;

	spinlock_t lock;
};

#define CMD_CFG_LENGTH_MASK GENMASK(8, 0)
#define CMD_CFG_BLOCK_MODE BIT(9)
#define CMD_CFG_R1B BIT(10)
#define CMD_CFG_END_OF_CHAIN BIT(11)
#define CMD_CFG_TIMEOUT_MASK GENMASK(15, 12)
#define CMD_CFG_NO_RESP BIT(16)
#define CMD_CFG_NO_CMD BIT(17)
#define CMD_CFG_DATA_IO BIT(18)
#define CMD_CFG_DATA_WR BIT(19)
#define CMD_CFG_RESP_NOCRC BIT(20)
#define CMD_CFG_RESP_128 BIT(21)
#define CMD_CFG_RESP_NUM BIT(22)
#define CMD_CFG_DATA_NUM BIT(23)
#define CMD_CFG_CMD_INDEX_MASK GENMASK(29, 24)
#define CMD_CFG_ERROR BIT(30)
#define CMD_CFG_OWNER BIT(31)

#define CMD_DATA_MASK GENMASK(31, 2)
#define CMD_DATA_BIG_ENDIAN BIT(1)
#define CMD_DATA_SRAM BIT(0)
#define CMD_RESP_MASK GENMASK(31, 1)
#define CMD_RESP_SRAM BIT(0)

static unsigned int meson_mmc_get_timeout_msecs(struct mmc_data *data)
{
	unsigned int timeout = data->timeout_ns / NSEC_PER_MSEC;

	if (!timeout)
		return SD_EMMC_CMD_TIMEOUT_DATA;

	timeout = roundup_pow_of_two(timeout);

	return min(timeout, 32768U); /* max. 2^15 ms */
}

static struct mmc_command *meson_mmc_get_next_command(struct mmc_command *cmd)
{
	if (cmd->opcode == MMC_SET_BLOCK_COUNT && !cmd->error)
		return cmd->mrq->cmd;
	else if (mmc_op_multi(cmd->opcode) &&
		 (!cmd->mrq->sbc || cmd->error || cmd->data->error))
		return cmd->mrq->stop;
	else
		return NULL;
}

static void meson_mmc_get_transfer_mode(struct mmc_host *mmc,
					struct mmc_request *mrq)
{
	struct meson_host *host = mmc_priv(mmc);
	struct mmc_data *data = mrq->data;
	struct scatterlist *sg;
	int i;

	/*
	 * When Controller DMA cannot directly access DDR memory, disable
	 * support for Chain Mode to directly use the internal SRAM using
	 * the bounce buffer mode.
	 */
	if (host->dram_access_quirk)
		return;

	/* SD_IO_RW_EXTENDED (CMD53) can also use block mode under the hood */
	if (data->blocks > 1 || mrq->cmd->opcode == SD_IO_RW_EXTENDED) {
		/*
		 * In block mode DMA descriptor format, "length" field indicates
		 * number of blocks and there is no way to pass DMA size that
		 * is not multiple of SDIO block size, making it impossible to
		 * tie more than one memory buffer with single SDIO block.
		 * Block mode sg buffer size should be aligned with SDIO block
		 * size, otherwise chain mode could not be used.
		 */
		for_each_sg(data->sg, sg, data->sg_len, i) {
			if (sg->length % data->blksz) {
				dev_warn_once(mmc_dev(mmc),
					      "unaligned sg len %u blksize %u, disabling descriptor DMA for transfer\n",
					      sg->length, data->blksz);
				return;
			}
		}
	}

	for_each_sg(data->sg, sg, data->sg_len, i) {
		/* check for 8 byte alignment */
		if (sg->offset % 8) {
			dev_warn_once(mmc_dev(mmc),
				      "unaligned sg offset %u, disabling descriptor DMA for transfer\n",
				      sg->offset);
			return;
		}
	}

	data->host_cookie |= SD_EMMC_DESC_CHAIN_MODE;
}

static inline bool meson_mmc_desc_chain_mode(const struct mmc_data *data)
{
	return data->host_cookie & SD_EMMC_DESC_CHAIN_MODE;
}

static inline bool meson_mmc_bounce_buf_read(const struct mmc_data *data)
{
	return data && data->flags & MMC_DATA_READ &&
	       !meson_mmc_desc_chain_mode(data);
}

static void meson_mmc_pre_req(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct mmc_data *data = mrq->data;

	if (!data)
		return;

	meson_mmc_get_transfer_mode(mmc, mrq);
	data->host_cookie |= SD_EMMC_PRE_REQ_DONE;

	if (!meson_mmc_desc_chain_mode(data))
		return;

	data->sg_count = dma_map_sg(mmc_dev(mmc), data->sg, data->sg_len,
                                   mmc_get_dma_dir(data));
	if (!data->sg_count)
		dev_err(mmc_dev(mmc), "dma_map_sg failed");
}

static void meson_mmc_post_req(struct mmc_host *mmc, struct mmc_request *mrq,
			       int err)
{
	struct mmc_data *data = mrq->data;

	if (data && meson_mmc_desc_chain_mode(data) && data->sg_count)
		dma_unmap_sg(mmc_dev(mmc), data->sg, data->sg_len,
			     mmc_get_dma_dir(data));
}

/*
 * Gating the clock on this controller is tricky.  It seems the mmc clock
 * is also used by the controller.  It may crash during some operation if the
 * clock is stopped.  The safest thing to do, whenever possible, is to keep
 * clock running at stop it at the pad using the pinmux.
 */
static void meson_mmc_clk_gate(struct meson_host *host)
{
	u32 cfg;

	if (host->pins_clk_gate) {
		pinctrl_select_state(host->pinctrl, host->pins_clk_gate);
	} else {
		/*
		 * If the pinmux is not provided - default to the classic and
		 * unsafe method
		 */
		cfg = readl(host->regs + SD_EMMC_CFG);
		cfg |= CFG_STOP_CLOCK;
		writel(cfg, host->regs + SD_EMMC_CFG);
	}
}

static void meson_mmc_clk_ungate(struct meson_host *host)
{
	u32 cfg;

	if (host->pins_clk_gate)
		pinctrl_select_default_state(host->dev);

	/* Make sure the clock is not stopped in the controller */
	cfg = readl(host->regs + SD_EMMC_CFG);
	cfg &= ~CFG_STOP_CLOCK;
	writel(cfg, host->regs + SD_EMMC_CFG);
}

static int meson_mmc_clk_set(struct meson_host *host, unsigned long rate,
			     bool ddr)
{
	struct mmc_host *mmc = host->mmc;
	int ret;
	u32 cfg;

	/* Same request - bail-out */
	if (host->ddr == ddr && host->req_rate == rate)
		return 0;

	/* stop clock */
	meson_mmc_clk_gate(host);
	host->req_rate = 0;
	mmc->actual_clock = 0;

	/* return with clock being stopped */
	if (!rate)
		return 0;

	/* Stop the clock during rate change to avoid glitches */
	cfg = readl(host->regs + SD_EMMC_CFG);
	cfg |= CFG_STOP_CLOCK;
	writel(cfg, host->regs + SD_EMMC_CFG);

	if (ddr) {
		/* DDR modes require higher module clock */
		rate <<= 1;
		cfg |= CFG_DDR;
	} else {
		cfg &= ~CFG_DDR;
	}
	writel(cfg, host->regs + SD_EMMC_CFG);
	host->ddr = ddr;

	/*
	 * The fclk_div2 mux parent (1 GHz) cannot integer-divide to the
	 * 400 MHz module clock DDR200 needs (only 500/333/250/200 MHz). Raise
	 * the programmable clkin0 parent to the target rate so the clock mux's
	 * determine_rate selects it (clkin0 reaches 400 MHz exactly via
	 * fclk_div5). Harmless at lower rates: it just offers the mux an
	 * equal-or-worse second candidate that determine_rate will not pick.
	 *
	 * The inverse matters too: once clkin0 sits at 400 MHz the 6-bit
	 * divider floors the reachable clock at ~6.35 MHz, so the ~400 kHz
	 * card init/reset/recovery clock becomes unreachable (clk_set_rate
	 * returns -EINVAL, cascading into a failed HW reset and eMMC/SD I/O
	 * errors on the DDR200 error-recovery path). So for those very low
	 * rates put clkin0 BACK to the rate it had at probe -- the 24 MHz
	 * (xtal) the DT assigns it and that this driver has always assumed for
	 * its low-rate path (see the assigned-clocks in meson-g12-common.dtsi),
	 * so the identification clock is again xtal / 60 exactly as on a
	 * mainline kernel. Do NOT set clkin0 to the bus rate itself: that
	 * drags the SoC-side clock down to 400 kHz and leaves the controller
	 * at divider 1 from a 400 kHz source (measured 2026-09-13 on
	 * AML-S905D3-CC: `sd_emmc_c_clk0 400000` then `#div 400000` at
	 * div=1), a configuration neither mainline nor U-Boot nor the vendor
	 * driver ever run the card-identification phase in, and under which
	 * the eMMC never answered CMD1 on that board. The normal 50-200 MHz
	 * rates are left to the framework (the mux's 1 GHz parent covers them).
	 */
	if (host->clkin0) {
		if (rate > 333333333)
			clk_set_rate(host->clkin0, rate);
		else if (rate < 6500000 && host->clkin0_rate)
			clk_set_rate(host->clkin0, host->clkin0_rate);
	}

	ret = clk_set_rate(host->mmc_clk, rate);
	if (ret) {
		dev_err(host->dev, "Unable to set cfg_div_clk to %lu. ret=%d\n",
			rate, ret);
		return ret;
	}

	host->req_rate = rate;
	mmc->actual_clock = clk_get_rate(host->mmc_clk);

	/* We should report the real output frequency of the controller */
	if (ddr) {
		host->req_rate >>= 1;
		mmc->actual_clock >>= 1;
	}

	dev_dbg(host->dev, "clk rate: %u Hz\n", mmc->actual_clock);
	if (rate != mmc->actual_clock)
		dev_dbg(host->dev, "requested rate was %lu\n", rate);

	/* (re)start clock */
	meson_mmc_clk_ungate(host);

	return 0;
}

/*
 * The SD/eMMC IP block has an internal mux and divider used for
 * generating the MMC clock.  Use the clock framework to create and
 * manage these clocks.
 */
static int meson_mmc_clk_init(struct meson_host *host)
{
	struct clk_init_data init;
	struct clk_mux *mux;
	struct clk_divider *div;
	char clk_name[32];
	int i, ret = 0;
	const char *mux_parent_names[MUX_CLK_NUM_PARENTS];
	const char *clk_parent[1];
	u32 clk_reg;

	/* init SD_EMMC_CLOCK to sane defaults w/min clock rate */
	clk_reg = CLK_ALWAYS_ON(host);
	clk_reg |= CLK_DIV_MASK;
	clk_reg |= FIELD_PREP(CLK_CORE_PHASE_MASK, CLK_PHASE_180);
	clk_reg |= FIELD_PREP(CLK_TX_PHASE_MASK, CLK_PHASE_0);
	clk_reg |= FIELD_PREP(CLK_RX_PHASE_MASK, CLK_PHASE_0);
	if (host->mmc->caps & MMC_CAP_SDIO_IRQ)
		clk_reg |= CLK_IRQ_SDIO_SLEEP(host);
	writel(clk_reg, host->regs + SD_EMMC_CLOCK);

	/* get the mux parents */
	for (i = 0; i < MUX_CLK_NUM_PARENTS; i++) {
		struct clk *clk;
		char name[16];

		snprintf(name, sizeof(name), "clkin%d", i);
		clk = devm_clk_get(host->dev, name);
		if (IS_ERR(clk))
			return dev_err_probe(host->dev, PTR_ERR(clk),
					     "Missing clock %s\n", name);

		mux_parent_names[i] = __clk_get_name(clk);

		/*
		 * clkin0 (sd_emmc_X_clk0) is a programmable CCF clock that can
		 * reach 400 MHz (via fclk_div5); keep a handle so the DDR200
		 * path can raise it -- the fclk_div2 mux parent (1 GHz) cannot
		 * integer-divide to the 400 MHz module clock DDR200 needs.
		 */
		if (i == 0) {
			host->clkin0 = clk;
			/*
			 * Remember the probe-time rate (the DT's assigned 24 MHz
			 * xtal on g12/sm1) so the low-rate path can restore it
			 * after a DDR200 raise instead of pulling clkin0 down
			 * to the bus rate.
			 */
			host->clkin0_rate = clk_get_rate(clk);
		}
	}

	/* create the mux */
	mux = devm_kzalloc(host->dev, sizeof(*mux), GFP_KERNEL);
	if (!mux)
		return -ENOMEM;

	snprintf(clk_name, sizeof(clk_name), "%s#mux", dev_name(host->dev));
	init.name = clk_name;
	init.ops = &clk_mux_ops;
	init.flags = 0;
	init.parent_names = mux_parent_names;
	init.num_parents = MUX_CLK_NUM_PARENTS;

	mux->reg = host->regs + SD_EMMC_CLOCK;
	mux->shift = __ffs(CLK_SRC_MASK);
	mux->mask = CLK_SRC_MASK >> mux->shift;
	mux->hw.init = &init;

	host->mux_clk = devm_clk_register(host->dev, &mux->hw);
	if (WARN_ON(IS_ERR(host->mux_clk)))
		return PTR_ERR(host->mux_clk);

	/* create the divider */
	div = devm_kzalloc(host->dev, sizeof(*div), GFP_KERNEL);
	if (!div)
		return -ENOMEM;

	snprintf(clk_name, sizeof(clk_name), "%s#div", dev_name(host->dev));
	init.name = clk_name;
	init.ops = &clk_divider_ops;
	init.flags = CLK_SET_RATE_PARENT;
	clk_parent[0] = __clk_get_name(host->mux_clk);
	init.parent_names = clk_parent;
	init.num_parents = 1;

	div->reg = host->regs + SD_EMMC_CLOCK;
	div->shift = __ffs(CLK_DIV_MASK);
	div->width = __builtin_popcountl(CLK_DIV_MASK);
	div->hw.init = &init;
	div->flags = CLK_DIVIDER_ONE_BASED;

	host->mmc_clk = devm_clk_register(host->dev, &div->hw);
	if (WARN_ON(IS_ERR(host->mmc_clk)))
		return PTR_ERR(host->mmc_clk);

	/* init SD_EMMC_CLOCK to sane defaults w/min clock rate */
	host->mmc->f_min = clk_round_rate(host->mmc_clk, 400000);
	ret = clk_set_rate(host->mmc_clk, host->mmc->f_min);
	if (ret)
		return ret;

	return clk_prepare_enable(host->mmc_clk);
}

static void meson_mmc_disable_resampling(struct meson_host *host)
{
	unsigned int val = readl(host->regs + host->data->adjust);

	val &= ~ADJUST_ADJ_EN;
	writel(val, host->regs + host->data->adjust);
}

static void meson_mmc_reset_resampling(struct meson_host *host)
{
	unsigned int val;

	meson_mmc_disable_resampling(host);

	val = readl(host->regs + host->data->adjust);
	val &= ~ADJUST_ADJ_DELAY_MASK;
	writel(val, host->regs + host->data->adjust);
}

static int meson_mmc_resampling_tuning(struct mmc_host *mmc, u32 opcode)
{
	struct meson_host *host = mmc_priv(mmc);
	unsigned int val, dly, max_dly, i;
	int ret;

	/* Resampling is done using the source clock */
	max_dly = DIV_ROUND_UP(clk_get_rate(host->mux_clk),
			       clk_get_rate(host->mmc_clk));

	val = readl(host->regs + host->data->adjust);
	val |= ADJUST_ADJ_EN;
	writel(val, host->regs + host->data->adjust);

	if (mmc_doing_retune(mmc))
		dly = FIELD_GET(ADJUST_ADJ_DELAY_MASK, val) + 1;
	else
		dly = 0;

	for (i = 0; i < max_dly; i++) {
		val &= ~ADJUST_ADJ_DELAY_MASK;
		val |= FIELD_PREP(ADJUST_ADJ_DELAY_MASK, (dly + i) % max_dly);
		writel(val, host->regs + host->data->adjust);

		ret = mmc_send_tuning(mmc, opcode, NULL);
		if (!ret) {
			dev_dbg(mmc_dev(mmc), "resampling delay: %u\n",
				(dly + i) % max_dly);
			return 0;
		}
	}

	meson_mmc_reset_resampling(host);
	return -EIO;
}

/*
 * One SD DDR200 RX eye scan at the current clock. DDR200 has no data strobe and
 * at 200 MHz DDR the coarse ADJUST resampling range collapses (max_dly =
 * mux_clk/mmc_clk is ~1), so scan the per-line DELAY taps (0..63) as the fine RX
 * knob across the four CLK_RX_PHASE settings (0/90/180/270), running 3 CMD19
 * reads per point.
 *
 * Selection: the centre of the LOWEST-delay passing window that is at least 8
 * taps wide. A large DELAY tap costs read throughput, so the first adequate eye
 * is preferred over the widest one. The full per-phase eye is emitted at
 * dev_dbg. Returns 0 with *phase_o/*tap_o/*len_o set, or -EIO if no window.
 */
static int meson_mmc_ddr200_eye_scan(struct mmc_host *mmc, u32 opcode,
				     int *phase_o, int *tap_o, int *len_o)
{
	struct meson_host *host = mmc_priv(mmc);
	unsigned long mux_rate, mmc_rate;
	int phase, tap, best_phase = -1, best_tap = -1, best_len = 0;
	u32 clk, val, max_dly;
	char row[65];

	mux_rate = clk_get_rate(host->mux_clk);
	mmc_rate = clk_get_rate(host->mmc_clk);
	max_dly = DIV_ROUND_UP(mux_rate, mmc_rate);
	dev_dbg(host->dev,
		"DDR200 eye: mux=%lu mmc=%lu sd_clk=%u max_adj_dly=%u; scan rx_phase x delay-tap\n",
		mux_rate, mmc_rate, mmc->actual_clock, max_dly);

	/* fine RX timing via the per-line DELAY taps; disable the coarse ADJUST */
	val = readl(host->regs + host->data->adjust);
	val &= ~(ADJUST_ADJ_EN | ADJUST_ADJ_DELAY_MASK);
	writel(val, host->regs + host->data->adjust);

	for (phase = 0; phase < 4; phase++) {
		int cur_start = -1, cur_len = 0;

		clk = readl(host->regs + SD_EMMC_CLOCK);
		clk &= ~CLK_RX_PHASE_MASK;
		clk |= FIELD_PREP(CLK_RX_PHASE_MASK, phase);
		writel(clk, host->regs + SD_EMMC_CLOCK);

		for (tap = 0; tap < 64; tap++) {
			int ok = 1, i;

			writel(aml_mv_dly1(tap), host->regs + SD_EMMC_DELAY1);
			writel(aml_mv_dly2(tap) & ~DELAY2_CMD_MASK,
			       host->regs + SD_EMMC_DELAY2);

			for (i = 0; i < 3; i++)
				if (mmc_send_tuning(mmc, opcode, NULL)) {
					ok = 0;
					break;
				}

			row[tap] = ok ? '#' : '.';
			if (ok) {
				if (cur_start < 0)
					cur_start = tap;
				cur_len++;
			} else {
				/*
				 * Lock the first (lowest-delay) window that is
				 * wide enough for margin (>= 8 taps) and never
				 * overwrite it -- prefer low delay over width.
				 */
				if (best_phase < 0 && cur_len >= 8) {
					best_phase = phase;
					best_len = cur_len;
					best_tap = cur_start + cur_len / 2;
				}
				cur_start = -1;
				cur_len = 0;
			}
		}
		/* a passing window may run to the last tap */
		if (best_phase < 0 && cur_len >= 8) {
			best_phase = phase;
			best_len = cur_len;
			best_tap = cur_start + cur_len / 2;
		}
		row[64] = '\0';
		dev_dbg(host->dev, "DDR200 eye rx_phase=%d (%3ddeg): %s\n",
			phase, phase * 90, row);
	}

	if (best_phase < 0)
		return -EIO;

	*phase_o = best_phase;
	*tap_o = best_tap;
	*len_o = best_len;
	return 0;
}

/* Do not step DDR200 below this SD clock -- at/under it SDR104 is as fast. */
#define MESON_DDR200_MIN_CLK	100000000

/*
 * SD DDR200 RX eye-scan tuning. Scan for a passing eye at the negotiated DDR200
 * clock; if none is found, step the clock down 25% and rescan, down to
 * MESON_DDR200_MIN_CLK -- a marginal card with no eye at the full rate may open
 * one slightly slower (still well above SDR104). This is the removable SD slot,
 * so a total failure (-EIO) just falls back to SDR104.
 *
 * NOTE: the reduced-clock retry is not yet hardware-validated (it needs a card
 * that fails the eye only at the full DDR200 rate). The full-rate scan is
 * unchanged, so cards that tune today are unaffected, and a card with no eye at
 * any rate ends at the same -EIO -> SDR104 fallback as before.
 */
static int meson_mmc_ddr200_tuning(struct mmc_host *mmc, u32 opcode)
{
	struct meson_host *host = mmc_priv(mmc);
	unsigned int orig_rate = mmc->ios.clock;
	unsigned int cur_rate;
	int phase, tap, len;
	u32 clk;

	/*
	 * A fresh (re-)init re-arms the clock ceiling to the full DDR200 rate.
	 * A runtime CRC storm lowers the ceiling via meson_mmc_lower_bus_clock(),
	 * so restart the eye-scan from the reduced ceiling instead of the
	 * marginal full rate the card storms at.
	 */
	if (!mmc_doing_retune(mmc))
		host->ddr200_clk_ceiling = 0;
	if (host->ddr200_clk_ceiling && host->ddr200_clk_ceiling < orig_rate) {
		orig_rate = host->ddr200_clk_ceiling;
		meson_mmc_clk_set(host, orig_rate, true);
	}
	cur_rate = orig_rate;

	while (meson_mmc_ddr200_eye_scan(mmc, opcode, &phase, &tap, &len)) {
		if (cur_rate * 3 / 4 < MESON_DDR200_MIN_CLK) {
			meson_mmc_clk_set(host, orig_rate, true);
			dev_warn(host->dev,
				 "DDR200 eye: no passing window found (incl. reduced clocks)\n");
			return -EIO;
		}
		cur_rate = cur_rate * 3 / 4;
		dev_info(host->dev,
			 "DDR200 eye: no window; retrying at reduced clock %u Hz\n",
			 cur_rate);
		meson_mmc_clk_set(host, cur_rate, true);
	}

	/* apply the centre of the selected (lowest-delay) passing window */
	clk = readl(host->regs + SD_EMMC_CLOCK);
	clk &= ~CLK_RX_PHASE_MASK;
	clk |= FIELD_PREP(CLK_RX_PHASE_MASK, phase);
	writel(clk, host->regs + SD_EMMC_CLOCK);
	writel(aml_mv_dly1(tap), host->regs + SD_EMMC_DELAY1);
	writel(aml_mv_dly2(tap) & ~DELAY2_CMD_MASK,
	       host->regs + SD_EMMC_DELAY2);
	dev_info(host->dev,
		 "DDR200 eye: selected rx_phase=%d tap=%d (window=%d, low-delay)\n",
		 phase, tap, len);
	return 0;
}

/*
 * CRC-storm clock-ladder step (mmc_host_ops->lower_bus_clock). A card can pass
 * the quick tuning yet CRC-storm under sustained traffic at the full rate while
 * running clean slower. Step the clock ceiling down to the next rate the
 * controller's divider can actually produce -- NOT a fixed ratio, which rounds
 * past achievable rungs (a fixed x3/4 from 200 MHz asks for 150 and lands on
 * 125, skipping the divider's 166 MHz). The next re-tune re-tunes at the new
 * ceiling. Below MESON_DDR200_MIN_CLK, or when the framework cannot go any
 * lower, return an error so the SD core drops the protocol grade instead.
 *
 * Handles both SD DDR200 and SDR104. DDR200 runs mmc_clk at 2x the SD bus rate
 * (CFG_DDR), so the current mmc_clk rate is the 2x-bus module clock and the bus
 * ladder is module/2 (200 -> 166 -> 125 -> 100); SDR104 runs mmc_clk at the bus
 * rate directly (200 -> 166 -> 143 -> ... -> 100). Use the real rate
 * (clk_get_rate), NOT the nominal ios.clock: the achieved rate is a few Hz low,
 * so a nominal-based request would still include the current rung.
 */
static int meson_mmc_lower_bus_clock(struct mmc_host *mmc)
{
	struct meson_host *host = mmc_priv(mmc);
	unsigned long cur_mod, cur, next;
	long module;
	bool ddr;

	if (mmc->ios.timing == MMC_TIMING_SD_DDR200)
		ddr = true;
	else if (mmc->ios.timing == MMC_TIMING_UHS_SDR104)
		ddr = false;
	else
		return -EOPNOTSUPP;

	cur_mod = clk_get_rate(host->mmc_clk);
	if (!cur_mod)
		return -ERANGE;
	cur = ddr ? (cur_mod >> 1) : cur_mod;

	module = clk_round_rate(host->mmc_clk, cur_mod - (cur_mod >> 4));
	if (module <= 0)
		return -ERANGE;
	next = ddr ? ((unsigned long)module >> 1) : (unsigned long)module;

	/*
	 * Floor at MESON_DDR200_MIN_CLK (100 MHz): a strobe-less DDR bus there
	 * ties SDR104's ~104 MB/s, and SDR104 there ties SDR50 -- below it the SD
	 * core drops the protocol grade (DDR200->SDR104, SDR104->DDR50) instead.
	 */
	if (next >= cur || next < MESON_DDR200_MIN_CLK)
		return -ERANGE;		/* clock ladder exhausted; core drops grade */

	if (ddr)
		host->ddr200_clk_ceiling = next;
	else
		host->sdr104_clk_ceiling = next;
	dev_info(host->dev,
		 "%s: CRC storm -- lowering clock ceiling to next achievable %lu Hz\n",
		 ddr ? "DDR200" : "SDR104", next);
	return 0;
}

/*
 * SDR104 reduced-clock tuning retry (B1). Resampling has only mux/mmc ADJUST
 * steps at the current clock (e.g. 5 at 200 MHz); a poorer-signal card may find
 * no passing point at the raised clock but tune cleanly a rung lower. Step the
 * clock down by real divider rungs (clk_round_rate, so 200->166 first -- the
 * upstream-safe rung -- not the DDR200 path's x3/4 which skips 166) and re-run
 * resampling, down to MESON_DDR200_MIN_CLK, before failing so the core drops to
 * DDR50. A fresh (re-)init re-arms the ceiling to the full rate; a runtime storm
 * lowered it via meson_mmc_lower_bus_clock().
 */
static int meson_mmc_sdr104_tuning(struct mmc_host *mmc, u32 opcode)
{
	struct meson_host *host = mmc_priv(mmc);
	unsigned long orig_rate = mmc->ios.clock;
	unsigned long cur_rate, next;
	long module;
	int ret;

	if (!mmc_doing_retune(mmc))
		host->sdr104_clk_ceiling = 0;
	if (host->sdr104_clk_ceiling && host->sdr104_clk_ceiling < orig_rate) {
		orig_rate = host->sdr104_clk_ceiling;
		meson_mmc_clk_set(host, orig_rate, false);
	}
	cur_rate = orig_rate;

	for (;;) {
		ret = meson_mmc_resampling_tuning(mmc, opcode);
		if (!ret)
			return 0;

		/* no resampling point here -- step down to the next real rung */
		module = clk_round_rate(host->mmc_clk, cur_rate - (cur_rate >> 4));
		next = (module > 0) ? (unsigned long)module : 0;
		if (next >= cur_rate || next < MESON_DDR200_MIN_CLK) {
			meson_mmc_clk_set(host, orig_rate, false);
			dev_warn(host->dev,
				 "SDR104: no resampling point (incl. reduced clocks)\n");
			return -EIO;
		}
		cur_rate = next;
		dev_info(host->dev,
			 "SDR104: no point; retrying at reduced clock %lu Hz\n",
			 cur_rate);
		meson_mmc_clk_set(host, cur_rate, false);
	}
}

/*
 * Branch tuning: SD DDR200 uses the eye scan, SDR104 uses resampling with a
 * reduced-clock retry (both can ladder the clock down), everything else
 * (HS200/eMMC, ...) plain resampling.
 */
static int meson_mmc_execute_tuning(struct mmc_host *mmc, u32 opcode)
{
	if (mmc->ios.timing == MMC_TIMING_SD_DDR200)
		return meson_mmc_ddr200_tuning(mmc, opcode);

	if (mmc->ios.timing == MMC_TIMING_UHS_SDR104)
		return meson_mmc_sdr104_tuning(mmc, opcode);

	return meson_mmc_resampling_tuning(mmc, opcode);
}

/*
 * HS400 tuning helpers, ported from the BSP meson-g12a-mmc.c fixed-adjust
 * (rx adjust delay, ADJUST_ADJ_DELAY_MASK == the BSP CLK_ADJUST_DELAY) tuning.
 * That fixed-adjust sweep is no longer used for HS400: meson_mmc_hs400_tuning()
 * below disables the ADJUST resampling and sweeps the per-line DELAY1/DELAY2
 * taps instead. meson_mmc_set_adjust() has no caller left;
 * meson_mmc_tuning_transfer() is the per-point pass test of the DELAY-tap scan.
 */

/* Program the rx adjust delay (V3 adjust reg 0xc). BSP: set_adjust:753-766. */
static void meson_mmc_set_adjust(struct meson_host *host, u32 value)
{
	u32 val;

	val = readl(host->regs + SD_EMMC_V3_ADJUST);
	val &= ~ADJUST_ADJ_DELAY_MASK;	/* clear old delay (bits 21:16) */
	val |= ADJUST_ADJ_EN;		/* enable rx adjust (bit 13) */
	val |= value << __ffs(ADJUST_ADJ_DELAY_MASK);
	writel(val, host->regs + SD_EMMC_V3_ADJUST);
}

/*
 * Run TUNING_NUM_PER_POINT tuning transfers at the current adjust point and
 * return how many passed. A point counts as good only if all of them pass.
 * BSP: meson_mmc_tuning_transfer:768-789.
 */
static int meson_mmc_tuning_transfer(struct mmc_host *mmc, u32 opcode)
{
	int n, nmatch;

	for (n = 0, nmatch = 0; n < TUNING_NUM_PER_POINT; n++) {
		if (!mmc_send_tuning(mmc, opcode, NULL)) {
			nmatch++;
		} else {
			/* let the card return from data to transfer state */
			usleep_range(20, 30);
			break;
		}
	}
	return nmatch;
}

/*
 * HS400 read-eye tuning. Replaces the BSP fixed-adjust (coarse ADJUST-delay)
 * sweep, which could NOT centre a sampling point at HS400's clk_div=1: its retry
 * ladder dropped the module clock below the 200 MHz DDR floor HS400 requires, so
 * it returned -EIO and mmc_select_hs400() failed with -5 -- the eMMC never
 * enumerated at HS400. On this V3 controller the operative HS400 RX knob is the
 * per-line DELAY tap (the same one the working DDR200 path tunes at this same
 * clk_div=1), NOT the coarse ADJUST resampling -- mirroring the eye-scan RX
 * finding that coarse-resampling modes use ADJUST while DDR200/HS400 use the
 * DELAY taps.
 *
 * Disable the coarse ADJUST resampling but KEEP the device strobe (ADJUST_DS_EN,
 * set by meson_mmc_check_resampling), sweep the per-line DELAY taps 0..63, test
 * each with TUNING_NUM_PER_POINT reads, and apply the centre of the widest
 * passing window. A failure (no adequate window) returns -EIO.
 */
static int meson_mmc_hs400_tuning(struct mmc_host *mmc, u32 opcode)
{
	struct meson_host *host = mmc_priv(mmc);
	int tap, cur_start = -1, cur_len = 0;
	int best_start = -1, best_len = 0;
	char row[65];
	u32 val;

	/*
	 * Disable the coarse ADJUST resampling but preserve ADJUST_DS_EN -- the
	 * device-strobe enable set by meson_mmc_check_resampling is what latches
	 * HS400 data; only the resampling knob is replaced by the DELAY eye scan.
	 */
	val = readl(host->regs + SD_EMMC_V3_ADJUST);
	val &= ~(ADJUST_ADJ_EN | ADJUST_ADJ_DELAY_MASK);
	writel(val, host->regs + SD_EMMC_V3_ADJUST);

	for (tap = 0; tap < 64; tap++) {
		writel(aml_mv_dly1(tap), host->regs + SD_EMMC_DELAY1);
		writel(aml_mv_dly2(tap) & ~DELAY2_CMD_MASK,
		       host->regs + SD_EMMC_DELAY2);

		if (meson_mmc_tuning_transfer(mmc, opcode) ==
		    TUNING_NUM_PER_POINT) {
			row[tap] = '#';
			if (cur_start < 0)
				cur_start = tap;
			cur_len++;
			/* track the widest contiguous passing window */
			if (cur_len > best_len) {
				best_len = cur_len;
				best_start = cur_start;
			}
		} else {
			row[tap] = '.';
			cur_start = -1;
			cur_len = 0;
		}
	}
	row[64] = '\0';
	dev_info(host->dev, "HS400 eye delay-tap: %s\n", row);

	/* require a few taps of margin so the centre sits solidly in the eye */
	if (best_len < 4) {
		dev_err(host->dev, "HS400 eye: no window (best_len=%d)\n",
			best_len);
		return -EIO;
	}

	/* apply the centre of the widest passing window */
	tap = best_start + best_len / 2;
	writel(aml_mv_dly1(tap), host->regs + SD_EMMC_DELAY1);
	writel(aml_mv_dly2(tap) & ~DELAY2_CMD_MASK,
	       host->regs + SD_EMMC_DELAY2);
	dev_info(host->dev, "HS400 eye: selected tap=%d window=%d\n",
		 tap, best_len);

	return 0;
}

/*
 * HS400 tuning hook. The MMC core does NOT route HS400 through .execute_tuning;
 * it calls .execute_hs400_tuning() from mmc_select_hs400() right after the
 * HS400 switch, and from mmc_init_card() after mmc_select_hs400es() for
 * HS400-ES (drivers/mmc/core/mmc.c). The data-strobe path is already
 * programmed by meson_mmc_check_resampling() at that point; here we run the
 * per-line DELAY-tap eye scan (meson_mmc_hs400_tuning()) against the eMMC
 * HS200 tuning block. Its error is returned to the core as is. Routing it as
 * .execute_hs400_tuning (not branching .execute_tuning on ios.timing) is
 * required: the core never calls .execute_tuning while in HS400 timing, so a
 * timing-branch in .execute_tuning would leave HS400 running with zeroed line
 * delays -> silent data corruption.
 */
static int meson_mmc_execute_hs400_tuning(struct mmc_host *mmc,
					  struct mmc_card *card)
{
	return meson_mmc_hs400_tuning(mmc, MMC_SEND_TUNING_BLOCK_HS200);
}

/*
 * HS400 Enhanced Strobe. In HS400-ES the device drives the data strobe for
 * CMD responses as well as data, so the CMD line no longer needs per-line
 * DELAY-tap tuning. mmc_select_hs400es() itself does not call
 * .execute_hs400_tuning, but mmc_init_card() calls it right afterwards, so
 * the DELAY-tap eye scan still runs for HS400-ES. The core calls this op from
 * mmc_select_hs400es() after switching the bus to HS400-ES
 * (drivers/mmc/core/mmc.c:1457).
 *
 * meson_mmc_check_resampling() has already programmed the HS400 strobe state
 * when the timing became HS400; re-assert the enhanced-strobe register state
 * here idempotently: gate the device strobe into the DLL (ADJUST_DS_EN) and
 * enable the strobe input interface (SD_INTF3), leaving the DS_SHT_* shift at
 * the BSP default 0 (the BSP meson_mmc_check_resampling() programs ES exactly
 * like plain HS400 -- DS_ENABLE + SD_INTF3, DS_SHT untouched).
 */
static void meson_mmc_hs400_enhanced_strobe(struct mmc_host *mmc,
					    struct mmc_ios *ios)
{
	struct meson_host *host = mmc_priv(mmc);
	u32 val;

	val = readl(host->regs + SD_EMMC_V3_ADJUST);
	val |= ADJUST_DS_EN;
	writel(val, host->regs + SD_EMMC_V3_ADJUST);

	val = readl(host->regs + SD_EMMC_INTF3);
	val &= ~(DS_SHT_M_MASK | DS_SHT_EXP_MASK);
	val |= SD_INTF3;
	writel(val, host->regs + SD_EMMC_INTF3);
}

static int meson_mmc_prepare_ios_clock(struct meson_host *host,
				       struct mmc_ios *ios)
{
	bool ddr;

	switch (ios->timing) {
	case MMC_TIMING_MMC_DDR52:
	case MMC_TIMING_UHS_DDR50:
	case MMC_TIMING_SD_DDR200:
	case MMC_TIMING_MMC_HS400:
		/*
		 * HS400 is double-data-rate at 200 MHz, so it needs the same
		 * 400 MHz module clock (raised clkin0/fclk_div5) as DDR200.
		 * Reuse that path; CFG_DDR is set by meson_mmc_clk_set().
		 */
		ddr = true;
		break;

	default:
		ddr = false;
		break;
	}

	return meson_mmc_clk_set(host, ios->clock, ddr);
}

/*
 * Set up the eMMC data-strobe path for HS400 on the V3 adjust block. The
 * strobe state is already torn down by meson_mmc_check_resampling() (below),
 * so this only sets the HS400 bits. Mirrors the BSP
 * meson_mmc_check_resampling() HS400 branch (meson-g12a-mmc.c:1307-1316):
 * DS_ENABLE (gate the device strobe into the DLL) + CFG_CMD_SETUP (launch the
 * command line with strobe-aligned setup) + SD_INTF3 (enable the strobe input
 * interface). The DS_SHT_* shift fields stay at the reset default 0.
 *
 * ADJUST_DS_EN is the V3 adjust reg (0xc) bit 15; the existing DDR200/
 * resampling code already uses the same ADJUST_* bit names against
 * host->data->adjust, so this reuses them rather than aliasing DS_ENABLE.
 */
static void meson_mmc_set_hs400_strobe(struct meson_host *host)
{
	u32 val;

	val = readl(host->regs + SD_EMMC_V3_ADJUST);
	val |= ADJUST_DS_EN;
	writel(val, host->regs + SD_EMMC_V3_ADJUST);

	val = readl(host->regs + SD_EMMC_IRQ_EN);
	val |= CFG_CMD_SETUP;
	writel(val, host->regs + SD_EMMC_IRQ_EN);

	val = readl(host->regs + SD_EMMC_INTF3);
	val |= SD_INTF3;
	writel(val, host->regs + SD_EMMC_INTF3);
}

static void meson_mmc_check_resampling(struct meson_host *host,
				       struct mmc_ios *ios)
{
	u32 val;

	/*
	 * On the V3 adjust block (axg/g12/sm1) the eMMC data strobe shares the
	 * DELAY1/DELAY2 taps, the INTF3 strobe controls and the V3 adjust
	 * DS_ENABLE bit with the resampling / DDR200 eye-scan tuning of the
	 * other speed modes. Mirror the BSP meson_mmc_check_resampling()
	 * (meson-g12a-mmc.c:1284-1316): on EVERY timing change, FIRST tear the
	 * strobe state down -- before programming the new timing -- so a mode
	 * that does not drive a device strobe (HS200, SDR104, DDR50, DDR200,
	 * legacy) is never left waiting on one. The previous port did the
	 * teardown only inside the HS400 branch, so de-selecting HS400 left
	 * DS_ENABLE / SD_INTF3 / CFG_CMD_SETUP set and the controller kept
	 * waiting for a strobe the re-inited HS200 card never drives -> the
	 * rootfs eMMC went unreachable.
	 *
	 * The ADJUST_ADJ_DELAY resampling field (and its ADJUST_ADJ_EN enable)
	 * that SDR104/HS200/DDR50/DDR200 rely on is preserved: only the
	 * strobe-specific ADJUST_DS_EN bit is cleared here.
	 */
	if (host->data->adjust == SD_EMMC_V3_ADJUST) {
		/* a no-op set_ios at the same timing must not wipe live tuning */
		if (host->timing == ios->timing)
			return;
		host->timing = ios->timing;

		writel(0, host->regs + SD_EMMC_DELAY1);
		writel(0, host->regs + SD_EMMC_DELAY2);

		val = readl(host->regs + SD_EMMC_INTF3);
		val &= ~(SD_INTF3 | DS_SHT_M_MASK | DS_SHT_EXP_MASK);
		writel(val, host->regs + SD_EMMC_INTF3);

		val = readl(host->regs + SD_EMMC_V3_ADJUST);
		val &= ~ADJUST_DS_EN;	/* keep ADJUST_ADJ_DELAY / ADJUST_ADJ_EN */
		writel(val, host->regs + SD_EMMC_V3_ADJUST);

		val = readl(host->regs + SD_EMMC_IRQ_EN);
		val &= ~CFG_CMD_SETUP;
		writel(val, host->regs + SD_EMMC_IRQ_EN);
	}

	switch (ios->timing) {
	case MMC_TIMING_LEGACY:
	case MMC_TIMING_MMC_HS:
	case MMC_TIMING_SD_HS:
	case MMC_TIMING_MMC_DDR52:
		meson_mmc_disable_resampling(host);
		break;
	case MMC_TIMING_MMC_HS400:
		/*
		 * HS400 uses the device data strobe instead of rx resampling.
		 * Only the V3 adjust block has the strobe registers; HS400 is
		 * cap-cleared on GX/GXL so the core never selects it there.
		 */
		if (host->data->adjust == SD_EMMC_V3_ADJUST)
			meson_mmc_set_hs400_strobe(host);
		break;
	}
}

static void meson_mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct meson_host *host = mmc_priv(mmc);
	u32 bus_width, val;
	int err;

	/*
	 * GPIO regulator, only controls switching between 1v8 and
	 * 3v3, doesn't support MMC_POWER_OFF, MMC_POWER_ON.
	 */
	switch (ios->power_mode) {
	case MMC_POWER_OFF:
		mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, 0);
		mmc_regulator_disable_vqmmc(mmc);
		msleep(50);

		break;

	case MMC_POWER_UP:
		mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, ios->vdd);

		break;

	case MMC_POWER_ON:
		mmc_regulator_enable_vqmmc(mmc);

		break;
	}

	/* Bus width */
	switch (ios->bus_width) {
	case MMC_BUS_WIDTH_1:
		bus_width = CFG_BUS_WIDTH_1;
		break;
	case MMC_BUS_WIDTH_4:
		bus_width = CFG_BUS_WIDTH_4;
		break;
	case MMC_BUS_WIDTH_8:
		bus_width = CFG_BUS_WIDTH_8;
		break;
	default:
		dev_err(host->dev, "Invalid ios->bus_width: %u.  Setting to 4.\n",
			ios->bus_width);
		bus_width = CFG_BUS_WIDTH_4;
	}

	val = readl(host->regs + SD_EMMC_CFG);
	val &= ~CFG_BUS_WIDTH_MASK;
	val |= FIELD_PREP(CFG_BUS_WIDTH_MASK, bus_width);
	writel(val, host->regs + SD_EMMC_CFG);

	meson_mmc_check_resampling(host, ios);
	err = meson_mmc_prepare_ios_clock(host, ios);
	if (err)
		dev_err(host->dev, "Failed to set clock: %d\n,", err);

	dev_dbg(host->dev, "SD_EMMC_CFG:  0x%08x\n", val);
}

static void meson_mmc_request_done(struct mmc_host *mmc,
				   struct mmc_request *mrq)
{
	struct meson_host *host = mmc_priv(mmc);

	host->cmd = NULL;
	if (host->needs_pre_post_req)
		meson_mmc_post_req(mmc, mrq, 0);
	mmc_request_done(host->mmc, mrq);
}

static void meson_mmc_set_blksz(struct mmc_host *mmc, unsigned int blksz)
{
	struct meson_host *host = mmc_priv(mmc);
	u32 cfg, blksz_old;

	cfg = readl(host->regs + SD_EMMC_CFG);
	blksz_old = FIELD_GET(CFG_BLK_LEN_MASK, cfg);

	if (!is_power_of_2(blksz))
		dev_err(host->dev, "blksz %u is not a power of 2\n", blksz);

	blksz = ilog2(blksz);

	/* check if block-size matches, if not update */
	if (blksz == blksz_old)
		return;

	dev_dbg(host->dev, "%s: update blk_len %d -> %d\n", __func__,
		blksz_old, blksz);

	cfg &= ~CFG_BLK_LEN_MASK;
	cfg |= FIELD_PREP(CFG_BLK_LEN_MASK, blksz);
	writel(cfg, host->regs + SD_EMMC_CFG);
}

static void meson_mmc_set_response_bits(struct mmc_command *cmd, u32 *cmd_cfg)
{
	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136)
			*cmd_cfg |= CMD_CFG_RESP_128;
		*cmd_cfg |= CMD_CFG_RESP_NUM;

		if (!(cmd->flags & MMC_RSP_CRC))
			*cmd_cfg |= CMD_CFG_RESP_NOCRC;

		if (cmd->flags & MMC_RSP_BUSY)
			*cmd_cfg |= CMD_CFG_R1B;
	} else {
		*cmd_cfg |= CMD_CFG_NO_RESP;
	}
}

static void meson_mmc_desc_chain_transfer(struct mmc_host *mmc, u32 cmd_cfg)
{
	struct meson_host *host = mmc_priv(mmc);
	struct sd_emmc_desc *desc = host->descs;
	struct mmc_data *data = host->cmd->data;
	struct scatterlist *sg;
	u32 start;
	int i;

	if (data->flags & MMC_DATA_WRITE)
		cmd_cfg |= CMD_CFG_DATA_WR;

	if (data->blocks > 1) {
		cmd_cfg |= CMD_CFG_BLOCK_MODE;
		meson_mmc_set_blksz(mmc, data->blksz);
	}

	for_each_sg(data->sg, sg, data->sg_count, i) {
		unsigned int len = sg_dma_len(sg);

		if (data->blocks > 1)
			len /= data->blksz;

		desc[i].cmd_cfg = cmd_cfg;
		desc[i].cmd_cfg |= FIELD_PREP(CMD_CFG_LENGTH_MASK, len);
		if (i > 0)
			desc[i].cmd_cfg |= CMD_CFG_NO_CMD;
		desc[i].cmd_arg = host->cmd->arg;
		desc[i].cmd_resp = 0;
		desc[i].cmd_data = sg_dma_address(sg);
	}
	desc[data->sg_count - 1].cmd_cfg |= CMD_CFG_END_OF_CHAIN;

	dma_wmb(); /* ensure descriptor is written before kicked */
	start = host->descs_dma_addr | START_DESC_BUSY;
	writel(start, host->regs + SD_EMMC_START);
}

/* local sg copy for dram_access_quirk */
static void meson_mmc_copy_buffer(struct meson_host *host, struct mmc_data *data,
				  size_t buflen, bool to_buffer)
{
	unsigned int sg_flags = SG_MITER_ATOMIC;
	struct scatterlist *sgl = data->sg;
	unsigned int nents = data->sg_len;
	struct sg_mapping_iter miter;
	unsigned int offset = 0;

	if (to_buffer)
		sg_flags |= SG_MITER_FROM_SG;
	else
		sg_flags |= SG_MITER_TO_SG;

	sg_miter_start(&miter, sgl, nents, sg_flags);

	while ((offset < buflen) && sg_miter_next(&miter)) {
		unsigned int buf_offset = 0;
		unsigned int len, left;
		u32 *buf = miter.addr;

		len = min(miter.length, buflen - offset);
		left = len;

		if (to_buffer) {
			do {
				writel(*buf++, host->bounce_iomem_buf + offset + buf_offset);

				buf_offset += 4;
				left -= 4;
			} while (left);
		} else {
			do {
				*buf++ = readl(host->bounce_iomem_buf + offset + buf_offset);

				buf_offset += 4;
				left -= 4;
			} while (left);
		}

		offset += len;
	}

	sg_miter_stop(&miter);
}

static void meson_mmc_start_cmd(struct mmc_host *mmc, struct mmc_command *cmd)
{
	struct meson_host *host = mmc_priv(mmc);
	struct mmc_data *data = cmd->data;
	u32 cmd_cfg = 0, cmd_data = 0;
	unsigned int xfer_bytes = 0;

	/* Setup descriptors */
	dma_rmb();

	host->cmd = cmd;

	cmd_cfg |= FIELD_PREP(CMD_CFG_CMD_INDEX_MASK, cmd->opcode);
	cmd_cfg |= CMD_CFG_OWNER;  /* owned by CPU */

	meson_mmc_set_response_bits(cmd, &cmd_cfg);

	/* data? */
	if (data) {
		data->bytes_xfered = 0;
		cmd_cfg |= CMD_CFG_DATA_IO;
		cmd_cfg |= FIELD_PREP(CMD_CFG_TIMEOUT_MASK,
				      ilog2(meson_mmc_get_timeout_msecs(data)));

		if (meson_mmc_desc_chain_mode(data)) {
			meson_mmc_desc_chain_transfer(mmc, cmd_cfg);
			return;
		}

		if (data->blocks > 1) {
			cmd_cfg |= CMD_CFG_BLOCK_MODE;
			cmd_cfg |= FIELD_PREP(CMD_CFG_LENGTH_MASK,
					      data->blocks);
			meson_mmc_set_blksz(mmc, data->blksz);
		} else {
			cmd_cfg |= FIELD_PREP(CMD_CFG_LENGTH_MASK, data->blksz);
		}

		xfer_bytes = data->blksz * data->blocks;
		if (data->flags & MMC_DATA_WRITE) {
			cmd_cfg |= CMD_CFG_DATA_WR;
			WARN_ON(xfer_bytes > host->bounce_buf_size);
			if (host->dram_access_quirk)
				meson_mmc_copy_buffer(host, data, xfer_bytes, true);
			else
				sg_copy_to_buffer(data->sg, data->sg_len,
						  host->bounce_buf, xfer_bytes);
			dma_wmb();
		}

		cmd_data = host->bounce_dma_addr & CMD_DATA_MASK;
	} else {
		cmd_cfg |= FIELD_PREP(CMD_CFG_TIMEOUT_MASK,
				      ilog2(SD_EMMC_CMD_TIMEOUT));
	}

	/* Last descriptor */
	cmd_cfg |= CMD_CFG_END_OF_CHAIN;
	writel(cmd_cfg, host->regs + SD_EMMC_CMD_CFG);
	writel(cmd_data, host->regs + SD_EMMC_CMD_DAT);
	writel(0, host->regs + SD_EMMC_CMD_RSP);
	wmb(); /* ensure descriptor is written before kicked */
	writel(cmd->arg, host->regs + SD_EMMC_CMD_ARG);
}

static int meson_mmc_validate_dram_access(struct mmc_host *mmc, struct mmc_data *data)
{
	struct scatterlist *sg;
	int i;

	/* Reject request if any element offset or size is not 32bit aligned */
	for_each_sg(data->sg, sg, data->sg_len, i) {
		if (!IS_ALIGNED(sg->offset, sizeof(u32)) ||
		    !IS_ALIGNED(sg->length, sizeof(u32))) {
			dev_err(mmc_dev(mmc), "unaligned sg offset %u len %u\n",
				data->sg->offset, data->sg->length);
			return -EINVAL;
		}
	}

	return 0;
}

static void meson_mmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct meson_host *host = mmc_priv(mmc);
	host->needs_pre_post_req = mrq->data &&
			!(mrq->data->host_cookie & SD_EMMC_PRE_REQ_DONE);

	/*
	 * The memory at the end of the controller used as bounce buffer for
	 * the dram_access_quirk only accepts 32bit read/write access,
	 * check the alignment and length of the data before starting the request.
	 */
	if (host->dram_access_quirk && mrq->data) {
		mrq->cmd->error = meson_mmc_validate_dram_access(mmc, mrq->data);
		if (mrq->cmd->error) {
			mmc_request_done(mmc, mrq);
			return;
		}
	}

	if (host->needs_pre_post_req) {
		meson_mmc_get_transfer_mode(mmc, mrq);
		if (!meson_mmc_desc_chain_mode(mrq->data))
			host->needs_pre_post_req = false;
	}

	if (host->needs_pre_post_req)
		meson_mmc_pre_req(mmc, mrq);

	/* Stop execution */
	writel(0, host->regs + SD_EMMC_START);

	meson_mmc_start_cmd(mmc, mrq->sbc ?: mrq->cmd);
}

static void meson_mmc_read_resp(struct mmc_host *mmc, struct mmc_command *cmd)
{
	struct meson_host *host = mmc_priv(mmc);

	if (cmd->flags & MMC_RSP_136) {
		cmd->resp[0] = readl(host->regs + SD_EMMC_CMD_RSP3);
		cmd->resp[1] = readl(host->regs + SD_EMMC_CMD_RSP2);
		cmd->resp[2] = readl(host->regs + SD_EMMC_CMD_RSP1);
		cmd->resp[3] = readl(host->regs + SD_EMMC_CMD_RSP);
	} else if (cmd->flags & MMC_RSP_PRESENT) {
		cmd->resp[0] = readl(host->regs + SD_EMMC_CMD_RSP);
	}
}

static void __meson_mmc_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct meson_host *host = mmc_priv(mmc);
	u32 reg_irqen = IRQ_EN_MASK;

	if (enable)
		reg_irqen |= IRQ_SDIO;
	writel(reg_irqen, host->regs + SD_EMMC_IRQ_EN);
}

static irqreturn_t meson_mmc_irq(int irq, void *dev_id)
{
	struct meson_host *host = dev_id;
	struct mmc_command *cmd;
	u32 status, raw_status, irq_mask = IRQ_EN_MASK;
	irqreturn_t ret = IRQ_NONE;

	if (host->mmc->caps & MMC_CAP_SDIO_IRQ)
		irq_mask |= IRQ_SDIO;
	raw_status = readl(host->regs + SD_EMMC_STATUS);
	status = raw_status & irq_mask;

	if (!status) {
		dev_dbg(host->dev,
			"Unexpected IRQ! irq_en 0x%08x - status 0x%08x\n",
			 irq_mask, raw_status);
		return IRQ_NONE;
	}

	/* ack all raised interrupts */
	writel(status, host->regs + SD_EMMC_STATUS);

	cmd = host->cmd;

	if (status & IRQ_SDIO) {
		spin_lock(&host->lock);
		__meson_mmc_enable_sdio_irq(host->mmc, 0);
		sdio_signal_irq(host->mmc);
		spin_unlock(&host->lock);
		status &= ~IRQ_SDIO;
		if (!status)
			return IRQ_HANDLED;
	}

	if (WARN_ON(!cmd))
		return IRQ_NONE;

	cmd->error = 0;
	if (status & IRQ_CRC_ERR) {
		dev_dbg(host->dev, "CRC Error - status 0x%08x\n", status);
		cmd->error = -EILSEQ;
		/*
		 * Account the CRC in the core err_stats so a debugfs/telemetry
		 * reader sees it. The driver otherwise only sets cmd->error, so
		 * the mmcN/err_stats "Data/Command CRC" counters stayed 0 even as
		 * writes hard-failed (the HS400 write-CRC gap seen on
		 * aml-s905d3-cc-demo-alpha). A data-phase command attributes to
		 * DAT_CRC, a bare command to CMD_CRC.
		 */
		mmc_debugfs_err_stats_inc(host->mmc,
			cmd->data ? MMC_ERR_DAT_CRC : MMC_ERR_CMD_CRC);
		ret = IRQ_WAKE_THREAD;
		goto out;
	}

	if (status & IRQ_TIMEOUTS) {
		dev_dbg(host->dev, "Timeout - status 0x%08x\n", status);
		cmd->error = -ETIMEDOUT;
		mmc_debugfs_err_stats_inc(host->mmc,
			cmd->data ? MMC_ERR_DAT_TIMEOUT : MMC_ERR_CMD_TIMEOUT);
		ret = IRQ_WAKE_THREAD;
		goto out;
	}

	meson_mmc_read_resp(host->mmc, cmd);

	if (status & (IRQ_END_OF_CHAIN | IRQ_RESP_STATUS)) {
		struct mmc_data *data = cmd->data;

		if (data && !cmd->error)
			data->bytes_xfered = data->blksz * data->blocks;

		return IRQ_WAKE_THREAD;
	}

out:
	if (cmd->error) {
		/* Stop desc in case of errors */
		u32 start = readl(host->regs + SD_EMMC_START);

		start &= ~START_DESC_BUSY;
		writel(start, host->regs + SD_EMMC_START);
	}

	return ret;
}

static int meson_mmc_wait_desc_stop(struct meson_host *host)
{
	u32 status;

	/*
	 * It may sometimes take a while for it to actually halt. Here, we
	 * are giving it 5ms to comply
	 *
	 * If we don't confirm the descriptor is stopped, it might raise new
	 * IRQs after we have called mmc_request_done() which is bad.
	 */

	return readl_poll_timeout(host->regs + SD_EMMC_STATUS, status,
				  !(status & (STATUS_BUSY | STATUS_DESC_BUSY)),
				  100, 5000);
}

static irqreturn_t meson_mmc_irq_thread(int irq, void *dev_id)
{
	struct meson_host *host = dev_id;
	struct mmc_command *next_cmd, *cmd = host->cmd;
	struct mmc_data *data;
	unsigned int xfer_bytes;

	if (WARN_ON(!cmd))
		return IRQ_NONE;

	if (cmd->error) {
		meson_mmc_wait_desc_stop(host);
		meson_mmc_request_done(host->mmc, cmd->mrq);

		return IRQ_HANDLED;
	}

	data = cmd->data;
	if (meson_mmc_bounce_buf_read(data)) {
		xfer_bytes = data->blksz * data->blocks;
		WARN_ON(xfer_bytes > host->bounce_buf_size);
		if (host->dram_access_quirk)
			meson_mmc_copy_buffer(host, data, xfer_bytes, false);
		else
			sg_copy_from_buffer(data->sg, data->sg_len,
					    host->bounce_buf, xfer_bytes);
	}

	next_cmd = meson_mmc_get_next_command(cmd);
	if (next_cmd)
		meson_mmc_start_cmd(host->mmc, next_cmd);
	else
		meson_mmc_request_done(host->mmc, cmd->mrq);

	return IRQ_HANDLED;
}

static void meson_mmc_cfg_init(struct meson_host *host)
{
	u32 cfg = 0;

	cfg |= FIELD_PREP(CFG_RESP_TIMEOUT_MASK,
			  ilog2(SD_EMMC_CFG_RESP_TIMEOUT));
	cfg |= FIELD_PREP(CFG_RC_CC_MASK, ilog2(SD_EMMC_CFG_CMD_GAP));
	cfg |= FIELD_PREP(CFG_BLK_LEN_MASK, ilog2(SD_EMMC_CFG_BLK_SIZE));

	/* abort chain on R/W errors */
	cfg |= CFG_ERR_ABORT;

	writel(cfg, host->regs + SD_EMMC_CFG);
}

static int meson_mmc_card_busy(struct mmc_host *mmc)
{
	struct meson_host *host = mmc_priv(mmc);
	u32 regval;

	regval = readl(host->regs + SD_EMMC_STATUS);

	/* We are only interrested in lines 0 to 3, so mask the other ones */
	return !(FIELD_GET(STATUS_DATI, regval) & 0xf);
}

static int meson_mmc_voltage_switch(struct mmc_host *mmc, struct mmc_ios *ios)
{
	int ret;

	/* vqmmc regulator is available */
	if (!IS_ERR(mmc->supply.vqmmc)) {
		/*
		 * The usual amlogic setup uses a GPIO to switch from one
		 * regulator to the other. While the voltage ramp up is
		 * pretty fast, care must be taken when switching from 3.3v
		 * to 1.8v. Please make sure the regulator framework is aware
		 * of your own regulator constraints
		 */
		ret = mmc_regulator_set_vqmmc(mmc, ios);
		return ret < 0 ? ret : 0;
	}

	/* no vqmmc regulator, assume fixed regulator at 3/3.3V */
	if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_330)
		return 0;

	return -EINVAL;
}

static void meson_mmc_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct meson_host *host = mmc_priv(mmc);
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	__meson_mmc_enable_sdio_irq(mmc, enable);
	spin_unlock_irqrestore(&host->lock, flags);
}

static void meson_mmc_ack_sdio_irq(struct mmc_host *mmc)
{
	meson_mmc_enable_sdio_irq(mmc, 1);
}

/*
 * Hardware per-lane RX eye + clock measurement for the debugfs "eye_scan"
 * diagnostic (V3 controllers only -- caller gates on host->data->adjust ==
 * SD_EMMC_V3_ADJUST). Unlike the generic software RX eye (eye/read_eye), the
 * SD_EMMC EYETEST engine reports a 64-bit pass bitmap PER data line, pinpointing
 * which lane is marginal; CLKTEST then measures the actual bus clock. Read-only:
 * every touched register (V3_ADJUST/INTF3/DELAY1/DELAY2/CFG) is saved and
 * restored. Runs with the host already claimed and re-tune held by the core.
 *
 * EYETEST is a READ-eye only -- it cannot see the write (TX) eye; that still
 * needs a software write-readback sweep (not implemented here).
 */
static void meson_mmc_hw_eyetest(struct meson_host *host, struct seq_file *s)
{
	struct mmc_host *mmc = host->mmc;
	u32 adj_save, intf3_save, dly1_save, dly2_save, cfg_save;
	u32 intf3, cfg, log, out;
	int lane, opcode, nlanes, i;

	opcode = (mmc->ios.timing == MMC_TIMING_MMC_HS200 ||
		  mmc->ios.timing == MMC_TIMING_MMC_HS400) ?
		 MMC_SEND_TUNING_BLOCK_HS200 : MMC_SEND_TUNING_BLOCK;
	nlanes = (mmc->ios.bus_width == MMC_BUS_WIDTH_8) ? 8 :
		 (mmc->ios.bus_width == MMC_BUS_WIDTH_4) ? 4 : 1;

	adj_save = readl(host->regs + SD_EMMC_V3_ADJUST);
	intf3_save = readl(host->regs + SD_EMMC_INTF3);
	dly1_save = readl(host->regs + SD_EMMC_DELAY1);
	dly2_save = readl(host->regs + SD_EMMC_DELAY2);
	cfg_save = readl(host->regs + SD_EMMC_CFG);

	seq_printf(s, "hardware RX eye: timing=%d lanes=%d sd_clk=%u  per-lane (64 taps)\n",
		   mmc->ios.timing, nlanes, mmc->actual_clock);

	for (lane = 0; lane < nlanes; lane++) {
		u64 bitmap = 0;
		int retry, done = 0;
		u32 adj;

		adj = readl(host->regs + SD_EMMC_V3_ADJUST);
		adj |= CALI_ENABLE;
		adj &= ~CALI_SEL_MASK;
		adj |= FIELD_PREP(CALI_SEL_MASK, lane);
		writel(adj, host->regs + SD_EMMC_V3_ADJUST);

		intf3 = readl(host->regs + SD_EMMC_INTF3);
		intf3 &= ~EYETEST_EXP_MASK;
		intf3 |= FIELD_PREP(EYETEST_EXP_MASK, 7);

		for (retry = 0; retry < 3 && !done; retry++) {
			intf3 |= EYETEST_ON;
			writel(intf3, host->regs + SD_EMMC_INTF3);
			udelay(5);
			/* drive read transfers so the HW sweeps its sample point */
			for (i = 0; i < 40; i++)
				mmc_send_tuning(mmc, opcode, NULL);
			udelay(1);
			log = readl(host->regs + SD_EMMC_EYETEST_LOG);
			if (log & EYETEST_DONE) {
				u32 o0 = readl(host->regs + SD_EMMC_EYETEST_OUT0);
				u32 o1 = readl(host->regs + SD_EMMC_EYETEST_OUT1);

				bitmap = ((u64)o1 << 32) | o0;
				done = 1;
			}
			intf3 &= ~EYETEST_ON;
			writel(intf3, host->regs + SD_EMMC_INTF3);
		}
		writel(adj_save, host->regs + SD_EMMC_V3_ADJUST);

		if (done) {
			char rowlo[65], rowhi[65];
			int t;

			/*
			 * bit0..63; orientation (bit0 = lowest vs highest delay
			 * tap) is TBD -- confirm on HW by cross-checking against
			 * the generic software RX eye (eye/read_eye). Print both
			 * ways until confirmed.
			 */
			for (t = 0; t < 64; t++) {
				rowlo[t] = (bitmap & BIT_ULL(t)) ? '#' : '.';
				rowhi[t] = (bitmap & BIT_ULL(63 - t)) ? '#' : '.';
			}
			rowlo[64] = rowhi[64] = '\0';
			seq_printf(s, "D%d: 0x%016llx\n     bit0..63:  %s\n     bit63..0:  %s\n",
				   lane, bitmap, rowlo, rowhi);
		} else {
			seq_printf(s, "D%d: EYETEST timeout\n", lane);
		}
	}

	/* CLKTEST: measure the actual bus clock (needs auto-clk off + DELAY=0) */
	cfg = cfg_save & ~CFG_AUTO_CLK;
	writel(cfg, host->regs + SD_EMMC_CFG);
	writel(0, host->regs + SD_EMMC_DELAY1);
	writel(0, host->regs + SD_EMMC_DELAY2);
	intf3 = readl(host->regs + SD_EMMC_INTF3);
	intf3 &= ~CLKTEST_EXP_MASK;
	intf3 |= FIELD_PREP(CLKTEST_EXP_MASK, 8);
	intf3 |= CLKTEST_ON;
	writel(intf3, host->regs + SD_EMMC_INTF3);
	for (i = 0, log = 0; i < 5 && !(log & CLKTEST_DONE); i++) {
		mdelay(1);
		log = readl(host->regs + SD_EMMC_CLKTEST_LOG);
	}
	out = readl(host->regs + SD_EMMC_CLKTEST_OUT);
	if (log & CLKTEST_DONE)
		seq_printf(s, "clktest: raw=0x%x count=%u\n", out, out >> 8);
	else
		seq_puts(s, "clktest: timeout\n");
	intf3 &= ~CLKTEST_ON;
	writel(intf3, host->regs + SD_EMMC_INTF3);

	/* restore everything touched */
	writel(adj_save, host->regs + SD_EMMC_V3_ADJUST);
	writel(intf3_save, host->regs + SD_EMMC_INTF3);
	writel(dly1_save, host->regs + SD_EMMC_DELAY1);
	writel(dly2_save, host->regs + SD_EMMC_DELAY2);
	writel(cfg_save, host->regs + SD_EMMC_CFG);
}

/*
 * ->eye_scan callback for the core debugfs "eye_scan" file. The core has
 * already claimed the host and held re-tune, so this must not do so again.
 *
 * This emits ONLY the hardware per-lane EYETEST + CLKTEST measurement, which is
 * unique to the V3 controller and has no equivalent anywhere else. The software
 * RX eye map that used to precede it has been retired: the generic, core-owned
 * eye facility (eye/read_eye, drivers/mmc/core/eye.c + meson_mmc_eye_ops below)
 * already does that sweep correctly -- it picks the operative RX knob per mode
 * (coarse ADJUST at HS200/SDR104, per-line DELAY at DDR200/HS400), whereas the
 * old in-file software sweep always swept DELAY and so produced an empty (all
 * '.') map at the coarse-resampling modes. Keeping both was duplicate work that
 * diverged at HS200; eye/read_eye is now the single software read-eye path.
 */
static void meson_mmc_eye_scan(struct mmc_host *mmc, struct seq_file *s)
{
	struct meson_host *host = mmc_priv(mmc);

	if (!mmc->card) {
		seq_puts(s, "no card\n");
		return;
	}

	if (host->data->adjust == SD_EMMC_V3_ADJUST)
		meson_mmc_hw_eyetest(host, s);
	else
		seq_puts(s, "hardware eyetest unsupported (non-V3 controller); use eye/read_eye for the software RX eye\n");
}

/*
 * Platform-agnostic eye-scan backend (struct mmc_eye_ops). The core owns the
 * sweep + oracle (drivers/mmc/core/eye.c); we only expose meson's timing knobs
 * as a 1-D point index per mode, holding the tuned phase:
 *
 *   read  (RX) -- whichever RX knob is operative for the current mode, mirroring
 *                 the driver's own tuning split (meson_mmc_execute_tuning):
 *                 * coarse-resampling modes (HS200/SDR104/...) tune the coarse
 *                   ADJUST_ADJ_DELAY (max_dly = mux/mmc source-clock periods, the
 *                   axis meson_mmc_resampling_tuning uses) -- sweep that;
 *                 * DDR200/HS400, where that resampling range collapses to ~1,
 *                   tune the per-line DELAY1/DELAY2 taps (as
 *                   meson_mmc_ddr200_eye_scan does) -- sweep those.
 *                 Sweeping the WRONG knob yields an all-'.' map (e.g. clearing
 *                 ADJUST and sweeping DELAY at HS200), so pick by max_dly.
 *   write (TX) -- the CLK_TX_DELAY field in SD_EMMC_CLOCK (6-bit/64 on V3,
 *                 4-bit/16 on V2), at the tuned CLK_TX_PHASE.
 *
 * ->begin snapshots, ->restore re-applies, so the validated tuning state is
 * left untouched. Works on any meson-gx controller; the write eye needs a card.
 */

/* RX read-eye ADJUST range = source-clock periods per bus clock; >1 means the
 * coarse ADJUST resampling is the operative RX knob, ==1 means use DELAY taps. */
static u32 meson_mmc_eye_rx_max_dly(struct meson_host *host)
{
	unsigned long mux = clk_get_rate(host->mux_clk);
	unsigned long mmc = clk_get_rate(host->mmc_clk);

	return mmc ? DIV_ROUND_UP(mux, mmc) : 1;
}

static bool meson_mmc_eye_rx_use_adjust(struct meson_host *host)
{
	return meson_mmc_eye_rx_max_dly(host) > 1;
}

static int meson_mmc_eye_num_points(struct mmc_host *mmc, enum mmc_eye_mode mode)
{
	struct meson_host *host = mmc_priv(mmc);
	u32 mask = CLK_TX_DELAY_MASK(host);

	if (mode == MMC_EYE_READ)
		return meson_mmc_eye_rx_use_adjust(host) ?
		       meson_mmc_eye_rx_max_dly(host) :	/* ADJUST steps */
		       64;				/* per-line DELAY taps */
	/* write: number of CLK_TX_DELAY field values = field max + 1 */
	return (mask >> __ffs(mask)) + 1;
}

static void meson_mmc_eye_begin(struct mmc_host *mmc, enum mmc_eye_mode mode)
{
	struct meson_host *host = mmc_priv(mmc);

	host->eye_save_clk = readl(host->regs + SD_EMMC_CLOCK);
	host->eye_save_dly1 = readl(host->regs + SD_EMMC_DELAY1);
	host->eye_save_dly2 = readl(host->regs + SD_EMMC_DELAY2);
	host->eye_save_adj = readl(host->regs + host->data->adjust);

	if (mode == MMC_EYE_READ && !meson_mmc_eye_rx_use_adjust(host)) {
		/*
		 * DELAY-tap axis (DDR200/HS400): disable the coarse ADJUST so the
		 * per-line taps are the sole RX knob. For the ADJUST axis we
		 * instead leave ADJUST + the tuned per-line DELAY in place and
		 * sweep ADJUST in ->set_point.
		 */
		u32 adj = host->eye_save_adj &
			  ~(ADJUST_ADJ_EN | ADJUST_ADJ_DELAY_MASK);

		writel(adj, host->regs + host->data->adjust);
	}
}

static void meson_mmc_eye_set_point(struct mmc_host *mmc,
				    enum mmc_eye_mode mode, int n)
{
	struct meson_host *host = mmc_priv(mmc);
	u32 clk;

	if (mode == MMC_EYE_READ && meson_mmc_eye_rx_use_adjust(host)) {
		/*
		 * Coarse-resampling modes (HS200/SDR104): sweep ADJUST_ADJ_DELAY,
		 * the operative RX knob, holding the tuned per-line DELAY. Mirrors
		 * meson_mmc_set_adjust() (clear delay, enable, set by shift).
		 */
		u32 adj = readl(host->regs + host->data->adjust);

		adj &= ~ADJUST_ADJ_DELAY_MASK;
		adj |= ADJUST_ADJ_EN;
		adj |= ((u32)n << __ffs(ADJUST_ADJ_DELAY_MASK)) &
		       ADJUST_ADJ_DELAY_MASK;
		writel(adj, host->regs + host->data->adjust);
	} else if (mode == MMC_EYE_READ) {
		/* hold the tuned CLK_RX_PHASE; move only the per-line delay */
		writel(aml_mv_dly1(n), host->regs + SD_EMMC_DELAY1);
		writel(aml_mv_dly2(n) & ~DELAY2_CMD_MASK,
		       host->regs + SD_EMMC_DELAY2);
	} else {
		/*
		 * Hold the tuned CLK_TX_PHASE; move only CLK_TX_DELAY. The mask
		 * is per-SoC runtime data (not a compile-time constant), so set
		 * the field by shift rather than FIELD_PREP.
		 */
		u32 mask = CLK_TX_DELAY_MASK(host);

		clk = readl(host->regs + SD_EMMC_CLOCK);
		clk &= ~mask;
		clk |= ((u32)n << __ffs(mask)) & mask;
		writel(clk, host->regs + SD_EMMC_CLOCK);
	}
}

static void meson_mmc_eye_restore(struct mmc_host *mmc, enum mmc_eye_mode mode)
{
	struct meson_host *host = mmc_priv(mmc);

	writel(host->eye_save_clk, host->regs + SD_EMMC_CLOCK);
	writel(host->eye_save_dly1, host->regs + SD_EMMC_DELAY1);
	writel(host->eye_save_dly2, host->regs + SD_EMMC_DELAY2);
	writel(host->eye_save_adj, host->regs + host->data->adjust);
}

static void meson_mmc_eye_describe(struct mmc_host *mmc, enum mmc_eye_mode mode,
				   int n, char *buf, size_t len)
{
	struct meson_host *host = mmc_priv(mmc);

	if (mode == MMC_EYE_READ && meson_mmc_eye_rx_use_adjust(host))
		snprintf(buf, len, "rx adjust=%d", n);
	else
		snprintf(buf, len, "%s delay tap=%d",
			 mode == MMC_EYE_WRITE ? "tx" : "rx", n);
}

static const struct mmc_eye_ops meson_mmc_eye_ops = {
	.num_points	= meson_mmc_eye_num_points,
	.begin		= meson_mmc_eye_begin,
	.set_point	= meson_mmc_eye_set_point,
	.restore	= meson_mmc_eye_restore,
	.describe	= meson_mmc_eye_describe,
};

static const struct mmc_host_ops meson_mmc_ops = {
	.request	= meson_mmc_request,
	.set_ios	= meson_mmc_set_ios,
	.get_cd         = mmc_gpio_get_cd,
	.pre_req	= meson_mmc_pre_req,
	.post_req	= meson_mmc_post_req,
	.execute_tuning = meson_mmc_execute_tuning,
	.lower_bus_clock = meson_mmc_lower_bus_clock,
	.execute_hs400_tuning = meson_mmc_execute_hs400_tuning,
	.hs400_enhanced_strobe = meson_mmc_hs400_enhanced_strobe,
	.card_busy	= meson_mmc_card_busy,
	.start_signal_voltage_switch = meson_mmc_voltage_switch,
	.enable_sdio_irq = meson_mmc_enable_sdio_irq,
	.ack_sdio_irq	= meson_mmc_ack_sdio_irq,
	.eye_scan	= meson_mmc_eye_scan,
};

static int meson_mmc_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct meson_host *host;
	struct mmc_host *mmc;
	struct clk *core_clk;
	int cd_irq, ret;

	mmc = devm_mmc_alloc_host(&pdev->dev, sizeof(struct meson_host));
	if (!mmc)
		return -ENOMEM;
	host = mmc_priv(mmc);
	host->mmc = mmc;
	host->dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, host);

	/* The G12A SDIO Controller needs an SRAM bounce buffer */
	host->dram_access_quirk = device_property_read_bool(&pdev->dev,
					"amlogic,dram-access-quirk");

	/* Get regulators and the supported OCR mask */
	ret = mmc_regulator_get_supply(mmc);
	if (ret)
		return ret;

	ret = mmc_of_parse(mmc);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "error parsing DT\n");

	mmc->caps |= MMC_CAP_CMD23;

	if (mmc->caps & MMC_CAP_SDIO_IRQ)
		mmc->caps2 |= MMC_CAP2_SDIO_IRQ_NOTHREAD;

	host->data = of_device_get_match_data(&pdev->dev);
	if (!host->data)
		return -EINVAL;

	ret = device_reset_optional(&pdev->dev);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "device reset failed\n");

	host->regs = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(host->regs))
		return PTR_ERR(host->regs);

	host->irq = platform_get_irq(pdev, 0);
	if (host->irq < 0)
		return host->irq;

	cd_irq = platform_get_irq_optional(pdev, 1);
	mmc_gpio_set_cd_irq(mmc, cd_irq);

	host->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(host->pinctrl))
		return PTR_ERR(host->pinctrl);

	host->pins_clk_gate = pinctrl_lookup_state(host->pinctrl,
						   "clk-gate");
	if (IS_ERR(host->pins_clk_gate)) {
		dev_warn(&pdev->dev,
			 "can't get clk-gate pinctrl, using clk_stop bit\n");
		host->pins_clk_gate = NULL;
	}

	core_clk = devm_clk_get_enabled(&pdev->dev, "core");
	if (IS_ERR(core_clk))
		return PTR_ERR(core_clk);

	ret = meson_mmc_clk_init(host);
	if (ret)
		return ret;

	/* set config to sane default */
	meson_mmc_cfg_init(host);

	/* Stop execution */
	writel(0, host->regs + SD_EMMC_START);

	/* clear, ack and enable interrupts */
	writel(0, host->regs + SD_EMMC_IRQ_EN);
	writel(IRQ_EN_MASK, host->regs + SD_EMMC_STATUS);
	writel(IRQ_EN_MASK, host->regs + SD_EMMC_IRQ_EN);

	ret = request_threaded_irq(host->irq, meson_mmc_irq,
				   meson_mmc_irq_thread, IRQF_ONESHOT,
				   dev_name(&pdev->dev), host);
	if (ret)
		goto err_init_clk;

	spin_lock_init(&host->lock);

	if (host->dram_access_quirk) {
		/* Limit segments to 1 due to low available sram memory */
		mmc->max_segs = 1;
		/* Limit to the available sram memory */
		mmc->max_blk_count = SD_EMMC_SRAM_DATA_BUF_LEN /
				     mmc->max_blk_size;
	} else {
		mmc->max_blk_count = CMD_CFG_LENGTH_MASK;
		mmc->max_segs = SD_EMMC_DESC_BUF_LEN /
				sizeof(struct sd_emmc_desc);
	}
	mmc->max_req_size = mmc->max_blk_count * mmc->max_blk_size;
	mmc->max_seg_size = mmc->max_req_size;

	/*
	 * HS400 / HS400-ES (eMMC data-strobe + per-line DELAY-tap eye-scan
	 * tuning) is only wired up for the V3 adjust block (axg/g12/sm1
	 * match-data, .adjust == SD_EMMC_V3_ADJUST): the strobe registers (INTF3,
	 * DS_ENABLE, CFG_CMD_SETUP) and the per-line DELAY1/DELAY2 taps the
	 * tuning sweeps only exist there.
	 *
	 * The port is not validated on hardware and is default-OFF: keep
	 * HS400/HS400-ES (gated by mmc-hs400-1_8v / mmc-hs400-enhanced-strobe in
	 * DT) only when BOTH the V3 strobe path exists AND the hs400_enable
	 * module param is set. With the default hs400_enable=false the caps are
	 * cleared on every controller, so behaviour is identical to mainline
	 * (HS200) -- nothing changes unless an operator opts in on hardware where
	 * it is safe (SPI NOR de-populated; see emmc-hs400-plan.md), and even
	 * then the risks in the hs400_enable comment at the top of this file
	 * (no .hs400_downgrade, no step-down on a tuning failure) stand. GX/GXL
	 * never has the strobe path at all.
	 */
	if (host->data->adjust != SD_EMMC_V3_ADJUST || !hs400_enable)
		mmc->caps2 &= ~(MMC_CAP2_HS400 | MMC_CAP2_HS400_ES);

	if (host->dram_access_quirk) {
		/*
		 * The MMC Controller embeds 1,5KiB of internal SRAM
		 * that can be used to be used as bounce buffer.
		 * In the case of the G12A SDIO controller, use these
		 * instead of the DDR memory
		 */
		host->bounce_buf_size = SD_EMMC_SRAM_DATA_BUF_LEN;
		host->bounce_iomem_buf = host->regs + SD_EMMC_SRAM_DATA_BUF_OFF;
		host->bounce_dma_addr = res->start + SD_EMMC_SRAM_DATA_BUF_OFF;
	} else {
		/* data bounce buffer */
		host->bounce_buf_size = mmc->max_req_size;
		host->bounce_buf =
			dmam_alloc_coherent(host->dev, host->bounce_buf_size,
					    &host->bounce_dma_addr, GFP_KERNEL);
		if (host->bounce_buf == NULL) {
			dev_err(host->dev, "Unable to map allocate DMA bounce buffer.\n");
			ret = -ENOMEM;
			goto err_free_irq;
		}
	}

	host->descs = dmam_alloc_coherent(host->dev, SD_EMMC_DESC_BUF_LEN,
					  &host->descs_dma_addr, GFP_KERNEL);
	if (!host->descs) {
		dev_err(host->dev, "Allocating descriptor DMA buffer failed\n");
		ret = -ENOMEM;
		goto err_free_irq;
	}

	mmc->ops = &meson_mmc_ops;
	mmc->eye_ops = &meson_mmc_eye_ops;
	ret = mmc_add_host(mmc);
	if (ret)
		goto err_free_irq;

	return 0;

err_free_irq:
	free_irq(host->irq, host);
err_init_clk:
	clk_disable_unprepare(host->mmc_clk);
	return ret;
}

static void meson_mmc_remove(struct platform_device *pdev)
{
	struct meson_host *host = dev_get_drvdata(&pdev->dev);

	mmc_remove_host(host->mmc);

	/* disable interrupts */
	writel(0, host->regs + SD_EMMC_IRQ_EN);
	free_irq(host->irq, host);

	clk_disable_unprepare(host->mmc_clk);
}

static const struct meson_mmc_data meson_gx_data = {
	.tx_delay_mask	= CLK_V2_TX_DELAY_MASK,
	.rx_delay_mask	= CLK_V2_RX_DELAY_MASK,
	.always_on	= CLK_V2_ALWAYS_ON,
	.adjust		= SD_EMMC_ADJUST,
	.irq_sdio_sleep	= CLK_V2_IRQ_SDIO_SLEEP,
};

static const struct meson_mmc_data meson_axg_data = {
	.tx_delay_mask	= CLK_V3_TX_DELAY_MASK,
	.rx_delay_mask	= CLK_V3_RX_DELAY_MASK,
	.always_on	= CLK_V3_ALWAYS_ON,
	.adjust		= SD_EMMC_V3_ADJUST,
	.irq_sdio_sleep	= CLK_V3_IRQ_SDIO_SLEEP,
};

static const struct of_device_id meson_mmc_of_match[] = {
	{ .compatible = "amlogic,meson-gx-mmc",		.data = &meson_gx_data },
	{ .compatible = "amlogic,meson-gxbb-mmc", 	.data = &meson_gx_data },
	{ .compatible = "amlogic,meson-gxl-mmc",	.data = &meson_gx_data },
	{ .compatible = "amlogic,meson-gxm-mmc",	.data = &meson_gx_data },
	{ .compatible = "amlogic,meson-axg-mmc",	.data = &meson_axg_data },
	{}
};
MODULE_DEVICE_TABLE(of, meson_mmc_of_match);

static struct platform_driver meson_mmc_driver = {
	.probe		= meson_mmc_probe,
	.remove		= meson_mmc_remove,
	.driver		= {
		.name = DRIVER_NAME,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = meson_mmc_of_match,
	},
};

module_platform_driver(meson_mmc_driver);

MODULE_DESCRIPTION("Amlogic S905*/GX*/AXG SD/eMMC driver");
MODULE_AUTHOR("Kevin Hilman <khilman@baylibre.com>");
MODULE_LICENSE("GPL v2");
