/*
 * Driver for Amlogic Meson SPI communication controller (SPICC)
 *
 * Copyright (C) BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 *
 * SPDX-License-Identifier: GPL-2.0+
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/reset.h>
#include <linux/pinctrl/consumer.h>
#include <linux/dma-mapping.h>
#include <linux/mmzone.h>
#include <linux/delay.h>

/*
 * There are two modes for data transmission: PIO and DMA.
 * When bits_per_word is a multiple of 8, data is transferred using DMA.
 * When bits_per_word is less than 8 or words fewer than fifo, then IO.
 *
 * DMA achieves a transfer with one or more SPI bursts, each SPI burst is made
 * up of one or more DMA bursts. The DMA burst implementation mechanism is,
 * For TX, when the number of words in TXFIFO is less than the preset
 * reading threshold, SPICC starts a reading DMA burst, which reads the preset
 * number of words from TX buffer, then writes them into TXFIFO.
 * For RX, when the number of words in RXFIFO is greater than the preset
 * writing threshold, SPICC starts a writing request burst, which reads the
 * preset number of words from RXFIFO, then write them into RX buffer.
 * DMA works if the transfer meets the following conditions,
 * - bits_per_word is a multiple of 8
 * - The transfer length in words must be multiples of the dma_burst_words,
 *   and the dma_burst_words should be one of 8,7...2, otherwise, it will be
 *   split into several SPI bursts by this driver
 *
 * The IP has quirks that need to be worked around
 * - Using FIFO entry 16 causes stall on G12 and newer
 * DMA quirks:
 * - Second transfers of odd bytes fail, use PIO to suppliment
 * PIO quirks:
 * - XCH occasionally fails while firing TC
*/

#define SPICC_PIO_WIDTH	32
#define SPICC_PIO_WIDTH_BYTES 4

#define SPICC_DMA_WIDTH 64
#define SPICC_DMA_WIDTH_BYTES 8

#define SPICC_DMA_MIN_WORDS 2
#define SPICC_DMA_MIN_FIFOS 2
#define SPICC_DMA_MIN_BPW 8

#define BITS_PER_TWO_BYTES 16
#define BITS_PER_HALF_BYTE 4

/* Register Map */
#define SPICC_RXDATA	0x00

#define SPICC_TXDATA	0x04

#define SPICC_CONREG	0x08
#define SPICC_ENABLE		BIT(0)
#define SPICC_MODE_MASTER	BIT(1)
#define SPICC_XCH		BIT(2)
#define SPICC_SMC		BIT(3)
#define SPICC_POL		BIT(4)
#define SPICC_PHA		BIT(5)
#define SPICC_SSCTL		BIT(6)
#define SPICC_SSPOL		BIT(7)
#define SPICC_DRCTL_MASK	GENMASK(9, 8)
#define SPICC_DRCTL_IGNORE	0
#define SPICC_DRCTL_FALLING	1
#define SPICC_DRCTL_LOWLEVEL	2
#define SPICC_CS_MASK		GENMASK(13, 12)
#define SPICC_DATARATE_MASK	GENMASK(18, 16)
#define SPICC_DATARATE_DIV4	0
#define SPICC_DATARATE_DIV8	1
#define SPICC_DATARATE_DIV16	2
#define SPICC_DATARATE_DIV32	3
#define SPICC_BITLENGTH_MASK	GENMASK(24, 19)
#define SPICC_BURSTLENGTH_MASK	GENMASK(31, 25)

#define SPICC_INTREG	0x0c
#define SPICC_TE_EN	BIT(0) /* TX FIFO Empty Interrupt */
#define SPICC_TH_EN	BIT(1) /* TX FIFO Half-Full Interrupt */
#define SPICC_TF_EN	BIT(2) /* TX FIFO Full Interrupt */
#define SPICC_RR_EN	BIT(3) /* RX FIFO Ready Interrupt */
#define SPICC_RH_EN	BIT(4) /* RX FIFO Half-Full Interrupt */
#define SPICC_RF_EN	BIT(5) /* RX FIFO Full Interrupt */
#define SPICC_RO_EN	BIT(6) /* RX FIFO Overflow Interrupt */
#define SPICC_TC_EN	BIT(7) /* Transfer Complete Interrupt */

#define SPICC_DMAREG	0x10
#define SPICC_DMA_ENABLE		BIT(0)
#define SPICC_TXFIFO_THRESHOLD_MASK	GENMASK(5, 1)
#define SPICC_RXFIFO_THRESHOLD_MASK	GENMASK(10, 6)
#define SPICC_READ_BURST_MASK		GENMASK(14, 11)
#define SPICC_WRITE_BURST_MASK		GENMASK(18, 15)
#define SPICC_DMA_URGENT		BIT(19)
#define SPICC_DMA_THREADID_MASK		GENMASK(25, 20)
#define SPICC_DMA_BURSTNUM_MASK		GENMASK(31, 26)

#define SPICC_STATREG	0x14
#define SPICC_TE	BIT(0) /* TX FIFO Empty Interrupt */
#define SPICC_TH	BIT(1) /* TX FIFO Half-Full Interrupt */
#define SPICC_TF	BIT(2) /* TX FIFO Full Interrupt */
#define SPICC_RR	BIT(3) /* RX FIFO Ready Interrupt */
#define SPICC_RH	BIT(4) /* RX FIFO Half-Full Interrupt */
#define SPICC_RF	BIT(5) /* RX FIFO Full Interrupt */
#define SPICC_RO	BIT(6) /* RX FIFO Overflow Interrupt */
#define SPICC_TC	BIT(7) /* Transfer Complete Interrupt */

#define SPICC_PERIODREG	0x18
#define SPICC_PERIOD	GENMASK(14, 0)	/* Wait cycles */

#define SPICC_TESTREG	0x1c
#define SPICC_TXCNT_MASK	GENMASK(4, 0)	/* TX FIFO Counter */
#define SPICC_RXCNT_MASK	GENMASK(9, 5)	/* RX FIFO Counter */
#define SPICC_SMSTATUS_MASK	GENMASK(12, 10)	/* State Machine Status */
#define SPICC_LBC_RO		BIT(13)	/* Loop Back Control Read-Only */
#define SPICC_LBC_W1		BIT(14) /* Loop Back Control Write-Only */
#define SPICC_SWAP_RO		BIT(14) /* RX FIFO Data Swap Read-Only */
#define SPICC_SWAP_W1		BIT(15) /* RX FIFO Data Swap Write-Only */ /* INVERT */
#define SPICC_DLYCTL_RO_MASK	GENMASK(20, 15) /* Delay Control Read-Only */
#define SPICC_MO_DELAY_MASK	GENMASK(17, 16) /* Master Output Delay */
#define SPICC_MO_NO_DELAY	0
#define SPICC_MO_DELAY_1_CYCLE	1
#define SPICC_MO_DELAY_2_CYCLE	2
#define SPICC_MO_DELAY_3_CYCLE	3
#define SPICC_MI_DELAY_MASK	GENMASK(19, 18) /* Master Input Delay */
#define SPICC_MI_NO_DELAY	0
#define SPICC_MI_DELAY_1_CYCLE	1
#define SPICC_MI_DELAY_2_CYCLE	2
#define SPICC_MI_DELAY_3_CYCLE	3
#define SPICC_MI_CAP_DELAY_MASK	GENMASK(21, 20) /* Master Capture Delay */
#define SPICC_CAP_AHEAD_2_CYCLE	0
#define SPICC_CAP_AHEAD_1_CYCLE	1
#define SPICC_CAP_NO_DELAY	2
#define SPICC_CAP_DELAY_1_CYCLE	3
#define SPICC_FIFORST_RO_MASK	GENMASK(22, 21) /* FIFO Softreset Read-Only */
#define SPICC_FIFORST_W1_MASK	GENMASK(23, 22) /* FIFO Softreset Write-Only */

#define SPICC_DRADDR	0x20	/* Read Address of DMA */

#define SPICC_DWADDR	0x24	/* Write Address of DMA */

#define SPICC_LD_CNTL0	0x28
#define VSYNC_IRQ_SRC_SELECT		BIT(0)
#define DMA_EN_SET_BY_VSYNC		BIT(2)
#define XCH_EN_SET_BY_VSYNC		BIT(3)
#define DMA_READ_COUNTER_EN		BIT(4)
#define DMA_WRITE_COUNTER_EN		BIT(5)
#define DMA_RADDR_LOAD_BY_VSYNC		BIT(6)
#define DMA_WADDR_LOAD_BY_VSYNC		BIT(7)
#define DMA_ADDR_LOAD_FROM_LD_ADDR	BIT(8)

#define SPICC_LD_CNTL1	0x2c
#define DMA_READ_COUNTER		GENMASK(15, 0)
#define DMA_WRITE_COUNTER		GENMASK(31, 16)
#define DMA_BURST_WORDS_DEFAULT		8
#define DMA_BURST_COUNT_MAX		0xffff

#define SPICC_ENH_CTL0	0x38	/* Enhanced Feature */
#define SPICC_ENH_CLK_CS_DELAY_MASK	GENMASK(15, 0)
#define SPICC_ENH_DATARATE_MASK		GENMASK(23, 16)
#define SPICC_ENH_DATARATE_EN		BIT(24)
#define SPICC_ENH_MOSI_OEN		BIT(25)
#define SPICC_ENH_CLK_OEN		BIT(26)
#define SPICC_ENH_CS_OEN		BIT(27)
#define SPICC_ENH_CLK_CS_DELAY_EN	BIT(28)
#define SPICC_ENH_MAIN_CLK_AO		BIT(29)

#define SPICC_ENH_CTL1	0x3c
#define SPICC_ENH_MISO_I_CAP_EN		BIT(0)
#define SPICC_ENH_CLK_TCNT		GENMASK(9,1)
#define SPICC_ENH_MOSI_I_CAP_EN		BIT(14)
#define SPICC_ENH_FCLK_EN		BIT(15)
#define SPICC_ENH_FCLK_MOSI_I_DELAY_EN	BIT(16)
#define SPICC_ENH_FCLK_MOSI_I_DELAY	GENMASK(19,17)
#define SPICC_ENH_FCLK_MISO_I_DELAY_EN	BIT(20)
#define SPICC_ENH_FCLK_MISO_I_DELAY	GENMASK(23,21)
#define SPICC_ENH_FCLK_MOSI_O_DELAY_EN	BIT(24)
#define SPICC_ENH_FCLK_MOSI_O_DELAY	GENMASK(27,25)
#define SPICC_ENH_FCLK_MOSI_OEN_DELAY_EN	BIT(28)
#define SPICC_ENH_FCLK_MOSI_OEN_DELAY	GENMASK(31,29)
#define SPICC_ENH_CTL2	0x40

#define writel_bits_relaxed(mask, val, addr) \
	writel_relaxed((readl_relaxed(addr) & ~(mask)) | (val), addr)

struct meson_spicc_data {
	unsigned int			max_speed_hz;
	unsigned int			min_speed_hz;
	unsigned int			fifo_size;
	unsigned int			dma_bursts_max;
	unsigned int			dma_xfers_max;
	bool				has_oen;
	bool				has_enhance_clk_div;
	bool				has_pclk;
};

struct meson_spicc_device {
	struct spi_controller		*host;
	struct platform_device		*pdev;
	void __iomem			*base;
	struct clk			*core;
	struct clk			*pclk;
	struct clk_divider		pow2_div;
	struct clk			*clk;
	struct spi_message		*message;
	struct spi_transfer		*xfer;
	struct completion		done;
	const struct meson_spicc_data	*data;
	u8				*tx_buf;
	u8				*rx_buf;
	u8				bpw;
	u8				bpw_packed;
	u8				bpw_mask;
	u8				Bpw; /* DMA only to reduce calculation */
	u8				fifo_active; /* PIO only to constrain RX */
	u8				clk_us;
	u32				irqs;
	u32				irq_count;
	u32				xfers_last;
	unsigned int			words;
	unsigned long			words_remain;
	unsigned long			tx_remain;
	unsigned long			rx_remain;
	struct pinctrl			*pinctrl;
	struct pinctrl_state		*pins_idle_high;
	struct pinctrl_state		*pins_idle_low;
	dma_addr_t			tx_dma;
	dma_addr_t			rx_dma;
	bool				using_dma;
	void				*tx_dma_buf;
	void				*rx_dma_buf;
	size_t				padded_len;
};

#define pow2_clk_to_spicc(_div) container_of(_div, struct meson_spicc_device, pow2_div)

static void meson_spicc_dump_reg(struct meson_spicc_device *spicc){
	void *regs[10] = {
		spicc->base + SPICC_CONREG,
		spicc->base + SPICC_INTREG,
		spicc->base + SPICC_DMAREG,
		spicc->base + SPICC_STATREG,
		spicc->base + SPICC_TESTREG,
		spicc->base + SPICC_LD_CNTL0,
		spicc->base + SPICC_LD_CNTL1,
		spicc->base + SPICC_ENH_CTL0,
		spicc->base + SPICC_ENH_CTL1,
		spicc->base + SPICC_ENH_CTL2
	};
	const char *names[] = {"con", "int", "dma", "stat", "test", "ld_cntl0", "ld_cntl1", "enh_cntl0" , "enh_cntl1", "enh_cntl2"};
	char bin[36];
	for (int r = 0; r < 10; r++) {
		int pos = 0;
		u32 reg = readl_relaxed(regs[r]);
		for (int i = 31; i >= 0; i--) {
			bin[pos++] = ((reg >> i) & 1) ? '1' : '0';
			if (i % 8 == 0 && i > 0) bin[pos++] = ' ';
		}
		bin[pos] = '\0';
		dev_warn(spicc->host->dev.parent, "%s: %s\n", names[r], bin);
	}
}

static void meson_spicc_oen_enable(struct meson_spicc_device *spicc)
{
	u32 conf;

	if (!spicc->data->has_oen) {
		/* Try to get pinctrl states for idle high/low */
		spicc->pins_idle_high = pinctrl_lookup_state(spicc->pinctrl,
							     "idle-high");
		if (IS_ERR(spicc->pins_idle_high)) {
			dev_warn(&spicc->pdev->dev, "can't get idle-high pinctrl\n");
			spicc->pins_idle_high = NULL;
		}
		spicc->pins_idle_low = pinctrl_lookup_state(spicc->pinctrl,
							     "idle-low");
		if (IS_ERR(spicc->pins_idle_low)) {
			dev_warn(&spicc->pdev->dev, "can't get idle-low pinctrl\n");
			spicc->pins_idle_low = NULL;
		}
		return;
	}

	conf = readl_relaxed(spicc->base + SPICC_ENH_CTL0) |
		SPICC_ENH_MOSI_OEN | SPICC_ENH_CLK_OEN | SPICC_ENH_CS_OEN;

	writel_relaxed(conf, spicc->base + SPICC_ENH_CTL0);
}

static inline void meson_spicc_pad_data(struct meson_spicc_device *spicc,
					const u8 *src, u8 *dst) {
	unsigned int i;
	for (i = 0; i < spicc->words; i++)
		memcpy(dst + i * SPICC_DMA_WIDTH_BYTES, src + i * spicc->Bpw, spicc->Bpw);
}

static inline void meson_spicc_unpad_data(struct meson_spicc_device *spicc,
					  const u8 *src, u8 *dst) {
	unsigned int i;
	for (i = 0; i < spicc->words; i++)
		memcpy(dst + i * spicc->Bpw, src + i * SPICC_DMA_WIDTH_BYTES, spicc->Bpw);
}

static int meson_spicc_dma_map(struct meson_spicc_device *spicc,
			       struct spi_transfer *xfer)
{
	struct device *dev = spicc->host->dev.parent;
	size_t padded_len;

	if (!(xfer->tx_buf && xfer->rx_buf))
		return -EINVAL;

	if (spicc->bpw == SPICC_DMA_WIDTH) {
		padded_len = xfer->len;
		/* No padding for 64-bit transfers */
		xfer->tx_dma = dma_map_single(dev, (void *)xfer->tx_buf, padded_len, DMA_TO_DEVICE);
		if (dma_mapping_error(dev, xfer->tx_dma)) {
			dev_err(dev, "TX DMA mapping failed\n");
			return -ENOMEM;
		}
		xfer->rx_dma = dma_map_single(dev, (void *)xfer->rx_buf, padded_len, DMA_FROM_DEVICE);
		if (dma_mapping_error(dev, xfer->rx_dma)) {
			dev_err(dev, "RX DMA mapping failed\n");
			dma_unmap_single(dev, xfer->tx_dma, xfer->len, DMA_TO_DEVICE);
			return -ENOMEM;
		}
	} else {
		/* Calculate number of words and padded length */
		padded_len = spicc->words * SPICC_DMA_WIDTH_BYTES;

		/* Allocate padded buffers */
		spicc->tx_dma_buf = kzalloc(padded_len, GFP_KERNEL | GFP_DMA);
		if (!spicc->tx_dma_buf) {
			kfree(spicc->tx_dma_buf);
			return -ENOMEM;
		}
		spicc->rx_dma_buf = kzalloc(padded_len, GFP_KERNEL | GFP_DMA);
		if (!spicc->rx_dma_buf) {
			kfree(spicc->tx_dma_buf);
			kfree(spicc->rx_dma_buf);
			return -ENOMEM;
		}

		/* Pad TX data */
		meson_spicc_pad_data(spicc, xfer->tx_buf, spicc->tx_dma_buf);

		/* Map padded buffers */
		xfer->tx_dma = dma_map_single(dev, spicc->tx_dma_buf, padded_len, DMA_TO_DEVICE);
		if (dma_mapping_error(dev, xfer->tx_dma)) {
			kfree(spicc->tx_dma_buf);
			kfree(spicc->rx_dma_buf);
			return -ENOMEM;
		}

		xfer->rx_dma = dma_map_single(dev, spicc->rx_dma_buf, padded_len, DMA_FROM_DEVICE);
		if (dma_mapping_error(dev, xfer->rx_dma)) {
			dma_unmap_single(dev, xfer->tx_dma, padded_len, DMA_TO_DEVICE);
			kfree(spicc->tx_dma_buf);
			kfree(spicc->rx_dma_buf);
			return -ENOMEM;
		}
	}

	spicc->tx_dma = xfer->tx_dma;
	spicc->rx_dma = xfer->rx_dma;
	spicc->padded_len = padded_len;

	return 0;
}

static void meson_spicc_dma_unmap(struct meson_spicc_device *spicc,
				  struct spi_transfer *xfer)
{
	struct device *dev = spicc->host->dev.parent;

	if (spicc->Bpw != SPICC_DMA_WIDTH_BYTES) {
		/* Unpad RX data */
		if (xfer->rx_buf && spicc->rx_dma_buf && spicc->rx_dma && spicc->padded_len) {
			dma_sync_single_for_cpu(dev, spicc->rx_dma, spicc->padded_len, DMA_FROM_DEVICE);
			meson_spicc_unpad_data(spicc, spicc->rx_dma_buf, xfer->rx_buf);
		}
	}

	/* Unmap buffers */
	if (xfer->tx_dma && spicc->padded_len) {
		dma_unmap_single(dev, xfer->tx_dma, spicc->padded_len, DMA_TO_DEVICE);
		xfer->tx_dma = 0;
		spicc->tx_dma = 0;
	}
	if (xfer->rx_dma && spicc->padded_len) {
		dma_unmap_single(dev, xfer->rx_dma, spicc->padded_len, DMA_FROM_DEVICE);
		xfer->rx_dma = 0;
		spicc->rx_dma = 0;
	}

	if (spicc->Bpw != SPICC_DMA_WIDTH_BYTES) {
		/* Free padded buffers */
		if (spicc->tx_dma_buf) {
			kfree(spicc->tx_dma_buf);
			spicc->tx_dma_buf = NULL;
		}
		if (spicc->rx_dma_buf) {
			kfree(spicc->rx_dma_buf);
			spicc->rx_dma_buf = NULL;
		}
	}

	/* Reset padded_len */
	spicc->padded_len = 0;
	spicc->using_dma = false;
}

static inline u32 meson_spicc_calc_dma_burst_count(struct meson_spicc_device *spicc,
						   u32 xfers, u32 *dma_burst_words)
{
	u32 i, burst_count;
	unsigned int dma_words_max;

	if (xfers <= spicc->data->fifo_size) {
		*dma_burst_words = xfers;
		return 1;
	}

	*dma_burst_words = DMA_BURST_WORDS_DEFAULT;

	dma_words_max = spicc->data->dma_bursts_max * DMA_BURST_WORDS_DEFAULT;
	/*
	if (xfers == (dma_words_max + 1))
		return spicc->data->dma_bursts_max - 1;
	*/
	if (xfers >= dma_words_max)
		return spicc->data->dma_bursts_max;

	for (i = DMA_BURST_WORDS_DEFAULT; i > 1; i--)
		if ((xfers % i) == 0) {
			*dma_burst_words = i;
			burst_count = xfers / i;
			if (burst_count < spicc->data->dma_bursts_max)
				return burst_count;
			return spicc->data->dma_bursts_max;
		}

	/* for xfers < dma_words_max not divisible by 2..8 */
	/*
	i = xfers % DMA_BURST_WORDS_DEFAULT;
	xfers -= i;

	// DMA does not support 1 word transfers
	// TODO: DMA fails for secondary transfers
	if (i == 1)
		xfers -= DMA_BURST_WORDS_DEFAULT;
	*/
	return xfers / DMA_BURST_WORDS_DEFAULT; // xfers >> 3;
}

static inline bool meson_spicc_txfull(struct meson_spicc_device *spicc)
{
	return !!FIELD_GET(SPICC_TF,
				readl_relaxed(spicc->base + SPICC_STATREG));
}

static inline bool meson_spicc_txhalf(struct meson_spicc_device *spicc)
{
	return !!FIELD_GET(SPICC_TH,
				readl_relaxed(spicc->base + SPICC_STATREG));
}

static inline bool meson_spicc_txempty(struct meson_spicc_device *spicc)
{
	return !!FIELD_GET(SPICC_TE,
				readl_relaxed(spicc->base + SPICC_STATREG));
}

static inline u8 meson_spicc_txcount(struct meson_spicc_device *spicc)
{
	return FIELD_GET(SPICC_TXCNT_MASK, readl_relaxed(spicc->base + SPICC_TESTREG));
}

static inline bool meson_spicc_rxover(struct meson_spicc_device *spicc)
{
	return !!FIELD_GET(SPICC_RO,
			 readl_relaxed(spicc->base + SPICC_STATREG));
}

static inline bool meson_spicc_rxfull(struct meson_spicc_device *spicc)
{
	return !!FIELD_GET(SPICC_RF,
				readl_relaxed(spicc->base + SPICC_STATREG));
}

static inline bool meson_spicc_rxhalf(struct meson_spicc_device *spicc)
{
	return !!FIELD_GET(SPICC_RH,
				readl_relaxed(spicc->base + SPICC_STATREG));
}

static inline bool meson_spicc_rxready(struct meson_spicc_device *spicc)
{
	return !!FIELD_GET(SPICC_RR,
				readl_relaxed(spicc->base + SPICC_STATREG));
}

static inline u8 meson_spicc_rxcount(struct meson_spicc_device *spicc)
{
	return FIELD_GET(SPICC_RXCNT_MASK, readl_relaxed(spicc->base + SPICC_TESTREG));
}

static inline void meson_spicc_reset_fifo(struct meson_spicc_device *spicc)
{
	if (spicc->data->has_oen)
		writel_bits_relaxed(SPICC_ENH_MAIN_CLK_AO,
				    SPICC_ENH_MAIN_CLK_AO,
				    spicc->base + SPICC_ENH_CTL0);

	writel_bits_relaxed(SPICC_FIFORST_W1_MASK, SPICC_FIFORST_W1_MASK,
				spicc->base + SPICC_TESTREG);

	while (meson_spicc_rxready(spicc))
		readl_relaxed(spicc->base + SPICC_RXDATA);

	if (spicc->data->has_oen)
		writel_bits_relaxed(SPICC_ENH_MAIN_CLK_AO, 0,
				    spicc->base + SPICC_ENH_CTL0);
}

static void meson_spicc_setup_dma(struct meson_spicc_device *spicc, bool setup_int)
{
	unsigned int xfers, bytes, dma_burst_words, dma_burst_count;
	unsigned int txfifo_thres, read_req;
	unsigned int rxfifo_thres, write_req;
	unsigned int count_en = 0;
	unsigned int ld_ctr1 = 0;

	writel_relaxed(spicc->tx_dma, spicc->base + SPICC_DRADDR);
	writel_relaxed(spicc->rx_dma, spicc->base + SPICC_DWADDR);

	dma_burst_count = meson_spicc_calc_dma_burst_count(spicc, spicc->words_remain, &dma_burst_words);
	xfers = dma_burst_count * dma_burst_words;
	bytes = xfers * spicc->Bpw;

	if (setup_int)
		spicc->irqs = DIV_ROUND_UP(spicc->words_remain, xfers);

	spicc->words_remain -= xfers;

	//pr_debug("%s: xfers %d dma_burst_words %d dma_burst_count %d\n", __func__, xfers, dma_burst_words, dma_burst_count);
	dma_burst_words--;

	/* GXL requires this to be set */
	if (!spicc->data->has_oen){
		writel_bits_relaxed(SPICC_BURSTLENGTH_MASK,
				FIELD_PREP(SPICC_BURSTLENGTH_MASK, xfers > 127 ? 127 : xfers - 1),
				spicc->base + SPICC_CONREG);
	}

	/* try to fix dma hang, doesn't work */
	if (spicc->clk_us) udelay(spicc->clk_us);

	if (spicc->tx_dma) {
		count_en |= DMA_READ_COUNTER_EN;
		// 8, 15 - 7 = 8, when < 8, read 7 + 1
		// 4, 15 - 3 = 12, when < 12, read 3 + 1
		// 8, 16 - 7 = 9, when < 9, read 7 + 1
		// 4, 16 - 3 = 13, when < 13, read 3 + 1
		txfifo_thres = spicc->data->fifo_size - dma_burst_words;
		read_req = dma_burst_words;
		ld_ctr1 |= FIELD_PREP(DMA_READ_COUNTER, dma_burst_count);
	}

	if (spicc->rx_dma) {
		count_en |= DMA_WRITE_COUNTER_EN;
		// 8, when > 7, write 7 + 1
		// 4, when > 3, read 3 + 1
		rxfifo_thres = dma_burst_words;
		write_req = dma_burst_words;
		ld_ctr1 |= FIELD_PREP(DMA_WRITE_COUNTER, dma_burst_count);
	}

	writel_relaxed(count_en, spicc->base + SPICC_LD_CNTL0);
	writel_relaxed(ld_ctr1, spicc->base + SPICC_LD_CNTL1);
	writel_relaxed(SPICC_DMA_ENABLE | SPICC_DMA_URGENT
			| FIELD_PREP(SPICC_TXFIFO_THRESHOLD_MASK, txfifo_thres)
			| FIELD_PREP(SPICC_READ_BURST_MASK, read_req)
			| FIELD_PREP(SPICC_RXFIFO_THRESHOLD_MASK, rxfifo_thres)
			| FIELD_PREP(SPICC_WRITE_BURST_MASK, write_req),
			spicc->base + SPICC_DMAREG);

	//if (spicc->clk_us) udelay(spicc->clk_us);

	/* DMA uses TE for fewer interrupts, TC generates an interrupt every burst */
	/*
	if (spicc->words_remain == 0 && dma_burst_count == 1 && dma_burst_words <= spicc->data->fifo_size){
		writel_relaxed(SPICC_TC_EN, spicc->base + SPICC_INTREG); // IRQ on TC
		pr_debug("%s: use TC, rxfifo_thres %d write_req %d\n", __func__, rxfifo_thres, write_req);
	} else
	*/
	if (setup_int){
		writel_relaxed(SPICC_TE_EN | SPICC_RO_EN, spicc->base + SPICC_INTREG);
		//pr_debug("%s: use TE, rxfifo_thres %d write_req %d\n", __func__, rxfifo_thres, write_req);
		//if (spicc->clk_us) udelay(spicc->clk_us);
	}

	/* Start DMA Burst*/
	writel_bits_relaxed(SPICC_SMC, SPICC_SMC, spicc->base + SPICC_CONREG); // TX on Data
	udelay(spicc->clk_us);
	if (!(readl_relaxed(spicc->base + SPICC_CONREG) & SPICC_SMC)){
		dev_err(spicc->host->dev.parent, "%s: SMC TX failed on DMA, DMAREG 0x%x, STATREG 0x%x, TESTREG 0x%x, retrying\n", __func__, readl_relaxed(spicc->base + SPICC_DMAREG), readl_relaxed(spicc->base + SPICC_STATREG), readl_relaxed(spicc->base + SPICC_TESTREG));
		writel_bits_relaxed(SPICC_SMC, SPICC_SMC, spicc->base + SPICC_CONREG); // TX on Data
	}

	spicc->tx_buf += bytes;
	spicc->rx_buf += bytes;
}

static inline void meson_spicc_rx(struct meson_spicc_device *spicc)
{
	u8 i, byte, xfers;
	u32 data, data2;

	if (spicc->clk_us) udelay(spicc->clk_us);
	/* rxcount can return 0 and still rx so below may add delay */
	/*
	if (!meson_spicc_rxcount(spicc)){
		dev_warn(spicc->host->dev.parent, "%s: RX on empty FIFO, delay another cycle\n", __func__);
		udelay(spicc->clk_us);
		if (!meson_spicc_rxcount(spicc)){
			dev_err(spicc->host->dev.parent, "%s: RX on empty FIFO\n", __func__);
		}
	}
	*/

	if (spicc->bpw_packed == BITS_PER_BYTE) { /* 5..8 */
		while (meson_spicc_rxready(spicc) && spicc->rx_remain) {
			data = readl_relaxed(spicc->base + SPICC_RXDATA);
			//pr_debug("%s: 0x%x %u\n", __func__, data, meson_spicc_rxcount(spicc));
			*spicc->rx_buf++ = (u8)data;
			spicc->rx_remain--;
		}
	} else if (spicc->bpw_packed == BITS_PER_TWO_BYTES) { /* 9..16 */
		while (meson_spicc_rxready(spicc) && spicc->rx_remain) {
			data = readl_relaxed(spicc->base + SPICC_RXDATA);
			//pr_debug("%s: 0x%x %u\n", __func__, data, meson_spicc_rxcount(spicc));
			*(u16 *)spicc->rx_buf = (u16)data;
			spicc->rx_buf += 2;
			spicc->rx_remain--;
		}
	} else if (spicc->bpw_packed == SPICC_PIO_WIDTH) { /* 25..32 */
		while (meson_spicc_rxready(spicc) && spicc->rx_remain) {
			data = readl_relaxed(spicc->base + SPICC_RXDATA);
			//pr_debug("%s: 0x%x %u\n", __func__, data, meson_spicc_rxcount(spicc));
			*(u32 *)spicc->rx_buf = data;
			spicc->rx_buf += SPICC_PIO_WIDTH_BYTES;
			spicc->rx_remain--;
		}
	} else if (spicc->bpw_packed == SPICC_DMA_WIDTH) { /* 57..64 */
		while (meson_spicc_rxready(spicc) && spicc->rx_remain) {
			data = readl_relaxed(spicc->base + SPICC_RXDATA);
			data2 = readl_relaxed(spicc->base + SPICC_RXDATA);
			//pr_debug("%s: 0x%x %u\n", __func__, data, meson_spicc_rxcount(spicc));
			*(u32 *)spicc->rx_buf = data2;
			spicc->rx_buf += SPICC_PIO_WIDTH_BYTES;
			*(u32 *)spicc->rx_buf = data;
			spicc->rx_buf += SPICC_PIO_WIDTH_BYTES;
			spicc->rx_remain--;
		}
	} else if (spicc->bpw < BITS_PER_BYTE) { /* 1..4 */
		xfers = BITS_PER_BYTE / spicc->bpw_packed;
		while (meson_spicc_rxready(spicc) && spicc->rx_remain) {
			byte = 0;
			for (i = (BITS_PER_BYTE - spicc->bpw_packed); i < BITS_PER_BYTE; i -= spicc->bpw_packed){
				byte |= (readl_relaxed(spicc->base + SPICC_RXDATA) & spicc->bpw_mask) << i;
				//pr_debug("%s: 0x%x %u\n", __func__, byte, meson_spicc_rxcount(spicc));
			}
			*spicc->rx_buf++ = byte;
			spicc->rx_remain -= xfers;
		}
	} else if (spicc->bpw_packed < SPICC_PIO_WIDTH) { /* 17..24 */
		while (meson_spicc_rxready(spicc) && spicc->rx_remain) {
			data = readl_relaxed(spicc->base + SPICC_RXDATA);
			//pr_debug("%s: 0x%x %u\n", __func__, data, meson_spicc_rxcount(spicc));
			for (i = 0; i < spicc->bpw_packed; i += BITS_PER_BYTE)
				*spicc->rx_buf++ = (u8)(data >> i);
			spicc->rx_remain--;
		}
	} else { /* 33..56 */
		while (meson_spicc_rxready(spicc) && spicc->rx_remain) {
			data = readl_relaxed(spicc->base + SPICC_RXDATA);
			data2 = readl_relaxed(spicc->base + SPICC_RXDATA);
			//pr_debug("%s: 0x%x 0x%x %u\n", __func__, data, data2, meson_spicc_rxcount(spicc));
			*(u32 *)spicc->rx_buf = data2;
			spicc->rx_buf += SPICC_PIO_WIDTH_BYTES;
			for (i = 0; i < spicc->bpw - SPICC_PIO_WIDTH; i += BITS_PER_BYTE)
				*spicc->rx_buf++ = (u8)(data >> i);
			spicc->rx_remain--;
		}
	}
}

static inline void meson_spicc_tx(struct meson_spicc_device *spicc, u8 xfers)
{
	u8 i, j, bytes;
	u32 data, data2;

	/* optimize for performance */
	if (spicc->bpw_packed == BITS_PER_BYTE){ /* 5..8 */
		for (i = 0; i < xfers; i++){
			data = *spicc->tx_buf++;
			writel_relaxed(data, spicc->base + SPICC_TXDATA);
			//pr_debug("%s: 0x%x %u\n", __func__, data, meson_spicc_txcount(spicc));
			spicc->tx_remain--;
		}
	} else if (spicc->bpw_packed == BITS_PER_TWO_BYTES){ /* 9..16 */
		for (i = 0; i < xfers; i++){
			data = (*(u16 *)spicc->tx_buf);
			spicc->tx_buf += 2;
			writel_relaxed(data, spicc->base + SPICC_TXDATA);
			//pr_debug("%s: 0x%x %u\n", __func__, data, meson_spicc_txcount(spicc));
			spicc->tx_remain--;
		}
	} else if (spicc->bpw_packed == SPICC_PIO_WIDTH){ /* 25..32*/
		for (i = 0; i < xfers; i++){
			data = *(u32 *)spicc->tx_buf;
			spicc->tx_buf += SPICC_PIO_WIDTH_BYTES;
			writel_relaxed(data, spicc->base + SPICC_TXDATA);
			//pr_debug("%s: 0x%x %u\n", __func__, data, meson_spicc_txcount(spicc));
			spicc->tx_remain--;
		}
	} else if (spicc->bpw_packed == SPICC_DMA_WIDTH){ /* 57..64 */
		for (i = 0; i < xfers; i++){
			data2 = *(u32 *)spicc->tx_buf; // LSB
			spicc->tx_buf += SPICC_PIO_WIDTH_BYTES;
			data = (*(u32 *)spicc->tx_buf); // MSB
			spicc->tx_buf += SPICC_PIO_WIDTH_BYTES;
			writel_relaxed(data, spicc->base + SPICC_TXDATA);
			writel_relaxed(data2, spicc->base + SPICC_TXDATA);
			//pr_debug("%s: 0x%x 0x%x %u\n", __func__, data, data2, meson_spicc_txcount(spicc));
			spicc->tx_remain--;
		}
	} else if (spicc->bpw_packed <= BITS_PER_HALF_BYTE) { /* 1..4 */
		bytes = (spicc->bpw_packed * xfers) >> 3;
		for (i = 0; i < bytes; i++){
			data = *spicc->tx_buf++;
			for (j = (BITS_PER_BYTE - spicc->bpw_packed); j < BITS_PER_BYTE; j -= spicc->bpw_packed){
				writel_relaxed((data >> j) & spicc->bpw_mask, spicc->base + SPICC_TXDATA);
				//pr_debug("%s: 0x%x %u\n", __func__, (data >> j) & data2, meson_spicc_txcount(spicc));
			}
			spicc->tx_remain -= xfers;
		}
	} else if (spicc->bpw < SPICC_PIO_WIDTH) { /* 17..24 */
		for (i = 0; i < xfers; i++){
			data = 0;
			for (j = 0; j < spicc->bpw; j += BITS_PER_BYTE){
				data2 = *spicc->tx_buf++;
				data |= data2 << j;
				//pr_debug("%s: %u %u %p\n", __func__, data2, data, spicc->tx_buf);
			}
			writel_relaxed(data, spicc->base + SPICC_TXDATA);
			//pr_debug("%s: 0x%x %u\n", __func__, data, meson_spicc_txcount(spicc));
			spicc->tx_remain--;
		}
	} else { /* 33..56 */
		for (i = 0; i < xfers; i++){
			/*
			data2 = 0; // LSB
			for (j = 0; j < SPICC_PIO_WIDTH; j += BITS_PER_BYTE)
				data2 |= ((u8)*spicc->tx_buf++) << j;
			*/
			data2 = *(u32 *)spicc->tx_buf; // LSB
			spicc->tx_buf += SPICC_PIO_WIDTH_BYTES;
			data = 0; // MSB
			for (j = 0; j < (spicc->bpw - SPICC_PIO_WIDTH); j += BITS_PER_BYTE)
				data |= (*spicc->tx_buf++) << j;
			writel_relaxed(data, spicc->base + SPICC_TXDATA);
			writel_relaxed(data2, spicc->base + SPICC_TXDATA);
			//pr_debug("%s: 0x%x 0x%x %u\n", __func__, data, data2, meson_spicc_txcount(spicc));
			spicc->tx_remain--;
		}
	}

	/* Avoids XCH failure on GXL but not G12+ */
	/* For G12+, this should be multiplied by xfers? */
	if (spicc->clk_us) udelay(spicc->clk_us);

}

static void meson_spicc_setup_pio(struct meson_spicc_device *spicc, bool setup_int)
{
	/* Determine number of word transfers per burst */
	u8 xfers = min_t(u32, spicc->words_remain, spicc->fifo_active);

	/* Calculate expected number of IRQs for transfer */
	if (setup_int)
		spicc->irqs = DIV_ROUND_UP(spicc->words_remain, xfers);

	//pr_debug("%s: %lu / %u = %u, %u\n", __func__, spicc->words_remain, xfers, spicc->irqs, spicc->fifo_active);

	/* Setup Xfer variables */
	spicc->tx_remain = spicc->rx_remain = xfers;
	spicc->words_remain -= xfers;

	/* Setup burst length */
	if (setup_int || spicc->xfers_last != xfers){
		writel_bits_relaxed(SPICC_BURSTLENGTH_MASK,
				    FIELD_PREP(SPICC_BURSTLENGTH_MASK, xfers - 1),
				    spicc->base + SPICC_CONREG);
		spicc->xfers_last = xfers;

		/* Wait one cycle for CONREG */
		udelay(spicc->clk_us);
	}

	if (meson_spicc_txcount(spicc)){
		dev_err(spicc->host->dev.parent, "%s: TX FIFO has %u entries before setup\n", __func__, meson_spicc_txcount(spicc));
	}

	/* Fill TX FIFO */
	meson_spicc_tx(spicc, xfers);

	/* Setup TC IRQ */
	if (setup_int)
		writel_relaxed(SPICC_TC_EN, spicc->base + SPICC_INTREG);

	/* Start Burst */
	writel_bits_relaxed(SPICC_XCH, SPICC_XCH, spicc->base + SPICC_CONREG);
}

static irqreturn_t meson_spicc_irq(int irq, void *data)
{
	struct meson_spicc_device *spicc = (void *) data;
	/*
	pr_debug("%s: INT 0x%x DMA 0x%x STAT 0x%x TEST 0x%x\n", __func__,
			readl_relaxed(spicc->base + SPICC_INTREG),
			readl_relaxed(spicc->base + SPICC_DMAREG),
			readl_relaxed(spicc->base + SPICC_STATREG),
			readl_relaxed(spicc->base + SPICC_TESTREG));
	*/

	/* Clear TC */
	u32 statreg = readl_relaxed(spicc->base + SPICC_STATREG);

	if (statreg & SPICC_RO){
		dev_err(spicc->host->dev.parent, "%s: RX overflow %u\n", __func__, meson_spicc_rxcount(spicc));
		if (!(statreg ^ SPICC_RO))
			return IRQ_HANDLED;
	}
	/*
	if (statreg & SPICC_RF){
		pr_debug("%s: RX full %u\n", __func__, meson_spicc_rxcount(spicc));
		if (!(statreg ^ SPICC_RF))
			return IRQ_HANDLED;
	}
	*/
	writel_relaxed(statreg, spicc->base + SPICC_STATREG); /* Reset TC via W1C */

	/* IRQ after time limit */
	if (!spicc->xfer) {
		writel(0, spicc->base + SPICC_INTREG);
		dev_err(spicc->host->dev.parent, "%s: IRQ without xfer\n", __func__);
		return IRQ_HANDLED;
	}

	if (spicc->using_dma){
		if (readl_relaxed(spicc->base + SPICC_DMAREG) & SPICC_DMA_ENABLE)
			return IRQ_HANDLED;

		if (spicc->words_remain) {
			if (spicc->irqs == ++spicc->irq_count){
				dev_err(spicc->host->dev.parent, "%s: IRQ count exceeded during DMA\n", __func__);
			} else if (spicc->words_remain >= spicc->data->fifo_size){
				dev_err(spicc->host->dev.parent, "%s: DMA again?\n", __func__);
				meson_spicc_setup_dma(spicc, false);
				return IRQ_HANDLED;
			}
		}

		if (spicc->xfer){
			meson_spicc_dma_unmap(spicc, spicc->xfer);
			if (spicc->words_remain){
				spicc->irq_count = 0;
				spicc->fifo_active = spicc->data->fifo_size;
				meson_spicc_setup_pio(spicc, true);
				return IRQ_HANDLED;
			}
			spicc->xfer = NULL;
		}

		writel_bits_relaxed(SPICC_SMC, 0, spicc->base + SPICC_CONREG);
		writel_relaxed(0, spicc->base + SPICC_DMAREG);
	} else {
		u8 txcount;
		if ((txcount = meson_spicc_txcount(spicc))){
			/* For low hz/bpw, this prints too much, see meson_spicc_tx for more info */
			//dev_warn(spicc->host->dev.parent, "%s: TX FIFO has %u entries, retrying XCH\n", __func__, txcount);
			writel_bits_relaxed(SPICC_XCH, SPICC_XCH, spicc->base + SPICC_CONREG);
			return IRQ_HANDLED;
		}

		/* Empty RX FIFO */
		meson_spicc_rx(spicc);
		if (spicc->rx_remain){
			dev_warn(spicc->host->dev.parent, "%s: RX FIFO has %u entries, %lu expected, retrying\n", __func__, meson_spicc_rxcount(spicc), spicc->rx_remain);
			meson_spicc_rx(spicc);

			if (spicc->rx_remain){
				dev_err(spicc->host->dev.parent, "%s: RX FIFO has %u entries, %lu expected, failed\n", __func__, meson_spicc_rxcount(spicc), spicc->rx_remain);
			}
		}

		if (spicc->words_remain){
			if (spicc->irqs == ++spicc->irq_count){
				dev_err(spicc->host->dev.parent, "%s: IRQ count exceeded during PIO\n", __func__);
			} else {
				meson_spicc_setup_pio(spicc, false);
				return IRQ_HANDLED;
			}
		}
	}

	/* Disable all IRQs */
	writel(0, spicc->base + SPICC_INTREG);
	complete(&spicc->done);
	return IRQ_HANDLED;
}

static void meson_spicc_auto_io_delay(struct meson_spicc_device *spicc, u32 speed_hz)
{
	u32 div;
	u32 mi_delay, cap_delay;
	u32 conf;
	/*
	bool enh_delay = false;
	u32 enh_cap_delay = 0;
	u32 enh_cap_delay_max;
	*/
	if (spicc->data->has_enhance_clk_div) {
	//	enh_cap_delay_max =
		div = FIELD_GET(SPICC_ENH_DATARATE_MASK,
			readl_relaxed(spicc->base + SPICC_ENH_CTL0));
		div++;
		div <<= 1;
	} else {
		div = FIELD_GET(SPICC_DATARATE_MASK,
				readl_relaxed(spicc->base + SPICC_CONREG));
		div += 2;
		div = 1 << div;
	}

	mi_delay = SPICC_MI_NO_DELAY;
	cap_delay = SPICC_CAP_AHEAD_2_CYCLE;

	// delay pushes output left, ahead pushes output right
	/*
	if (speed_hz >= 333333328){
		cap_delay = SPICC_CAP_DELAY_1_CYCLE; // ahead_1 01010101 -> 00101010 no 01010101 10010101
	} else if (speed_hz >= 199999997){
		cap_delay = SPICC_CAP_DELAY_1_CYCLE; // ahead_1 01010101 -> 00111111 no 01010101 10101010

		enh_delay = 1; // LOOP PASS
		enh_cap_delay = 0; // LOOP PASS

	} else
	*/
	if (speed_hz >= 166666664)
		//cap_delay = SPICC_CAP_DELAY_1_CYCLE; // PHYSICAL PASS
		cap_delay = SPICC_CAP_AHEAD_1_CYCLE; // LOOP PASS
	else if (speed_hz >= 124999998)
		cap_delay = SPICC_CAP_AHEAD_1_CYCLE;
	else if (speed_hz >= 99999999)
		//cap_delay = SPICC_CAP_DELAY_1_CYCLE; // PHYSICAL PASS
		cap_delay = SPICC_CAP_AHEAD_1_CYCLE; // LOOP PASS
	else if (speed_hz >= 83333332)
		//cap_delay = SPICC_CAP_NO_DELAY; // PHYSICAL PASS
		cap_delay = SPICC_CAP_AHEAD_1_CYCLE; // LOOP PASS
	else if (speed_hz >= 33333333)
		cap_delay = SPICC_CAP_AHEAD_1_CYCLE;
	else if (speed_hz <= 200000)
		mi_delay = SPICC_MI_NO_DELAY;
	else if (speed_hz <= 8000000 && div == 2)
		mi_delay = SPICC_MI_DELAY_3_CYCLE;
	else if (div >= 16)
		mi_delay = SPICC_MI_DELAY_3_CYCLE;
	else if (div >= 8)
		mi_delay = SPICC_MI_DELAY_2_CYCLE;
	else if (div >= 6)
		mi_delay = SPICC_MI_DELAY_1_CYCLE;

	//pr_debug("%s: %uhz div %d mi_delay %d cap_delay %d\n", __func__, speed_hz, div, mi_delay, cap_delay);

	conf = readl_relaxed(spicc->base + SPICC_TESTREG);
	conf &= ~(SPICC_MO_DELAY_MASK | SPICC_MI_DELAY_MASK
		  | SPICC_MI_CAP_DELAY_MASK);
	conf |= FIELD_PREP(SPICC_MI_DELAY_MASK, mi_delay);
	conf |= FIELD_PREP(SPICC_MI_CAP_DELAY_MASK, cap_delay);
	writel_relaxed(conf, spicc->base + SPICC_TESTREG);
	/*
	pr_debug("%s: div %d cap_delay %d enh_cap_delay %d enh_cap_delay_max %d\n", __func__, div, cap_delay, enh_cap_delay, enh_cap_delay_max);

	if (enh_delay)
		writel_relaxed(0, spicc->base + SPICC_ENH_CTL1);
	else {
		writel_bits_relaxed(SPICC_ENH_CLK_CS_DELAY_EN | SPICC_ENH_CLK_CS_DELAY_MASK, SPICC_ENH_CLK_CS_DELAY_EN | FIELD_PREP(SPICC_ENH_CLK_CS_DELAY_MASK, 2),
				    spicc->base + SPICC_ENH_CTL0);
		writel_relaxed(
			SPICC_ENH_MISO_I_CAP_EN |
			FIELD_PREP(SPICC_ENH_CLK_TCNT, 0) |
			SPICC_ENH_FCLK_EN |
			SPICC_ENH_FCLK_MISO_I_DELAY_EN |
			FIELD_PREP(SPICC_ENH_FCLK_MISO_I_DELAY, 1) |
			SPICC_ENH_FCLK_MOSI_O_DELAY_EN |
			FIELD_PREP(SPICC_ENH_FCLK_MOSI_O_DELAY, 3) |
			SPICC_ENH_FCLK_MOSI_OEN_DELAY_EN |
			FIELD_PREP(SPICC_ENH_FCLK_MOSI_OEN_DELAY, 2),
			spicc->base + SPICC_ENH_CTL1);
	}
	*/
}

static void meson_spicc_setup_xfer(struct meson_spicc_device *spicc,
				   struct spi_transfer *xfer)
{
	u32 conf, conf_orig;

	/* Read original configuration */
	conf = conf_orig = readl_relaxed(spicc->base + SPICC_CONREG);

	/* Setup word width */
	conf &= ~SPICC_BITLENGTH_MASK;
	conf |= FIELD_PREP(SPICC_BITLENGTH_MASK, xfer->bits_per_word - 1);

	/* Ignore if unchanged */
	if (conf != conf_orig)
		writel_relaxed(conf, spicc->base + SPICC_CONREG);

	clk_set_rate(spicc->clk, xfer->speed_hz);

	meson_spicc_auto_io_delay(spicc, xfer->effective_speed_hz = clk_get_rate(spicc->clk));

	writel_relaxed(0, spicc->base + SPICC_DMAREG);
}

static inline u8 meson_spicc_calc_bpw_packed(u8 bpw){
	return ((bpw & (bpw - 1)) == 0) ? bpw : ((bpw < BITS_PER_HALF_BYTE) ? BITS_PER_HALF_BYTE : (bpw + BITS_PER_BYTE - 1) >> 3 << 3);
}

static size_t meson_spicc_max_transfer_size(struct spi_device *spi)
{
	struct meson_spicc_device *spicc = spi_controller_get_devdata(spi->controller);

	/* assuming MAX_PAGE_ORDER is 10, padded DMA buffers is limited to 2MB */
	unsigned int padded_dma_word_limit = (PAGE_SIZE << (MAX_PAGE_ORDER - 1)) / SPICC_DMA_WIDTH_BYTES;
	size_t max_transfer_size;
	u8 bpw = spi->bits_per_word;
	u8 Bpw = (bpw + BITS_PER_BYTE - 1) >> 3;

	/*
	if (bpw % 8 != 0 && (Bpw & (Bpw - 1)) != 0)
		max_transfer_size = SIZE_MAX;
	*/
	if (bpw < BITS_PER_BYTE)
		max_transfer_size = SIZE_MAX;
	else if (Bpw == SPICC_DMA_WIDTH_BYTES)
		max_transfer_size = spicc->data->dma_xfers_max * SPICC_DMA_WIDTH_BYTES;
	else if (spi->bits_per_word >= SPICC_DMA_MIN_BPW)
		max_transfer_size = min_t(size_t, padded_dma_word_limit * Bpw, spicc->data->dma_xfers_max * Bpw);
	else
		max_transfer_size = SIZE_MAX;

	//pr_debug("%s: bpw %u Bpw %u padded_dma_word_limit %u max_transfer_size %lu\n", __func__, spi->bits_per_word, Bpw, padded_dma_word_limit, max_transfer_size);

	return max_transfer_size;
}

static int meson_spicc_transfer_one(struct spi_controller *host,
				struct spi_device *spi,
				struct spi_transfer *xfer)
{
	struct meson_spicc_device *spicc = spi_controller_get_devdata(host);
	uint64_t max_time;
	u8 fifo_shift, time_shift;
	bool dma;
	int ret;

	ktime_t start_time;
	u64 init_time, run_time;

	start_time = ktime_get();

	/* Store current transfer */
	spicc->xfer = xfer;

	/* Setup transfer parameters */
	spicc->tx_buf = (u8 *)xfer->tx_buf;
	spicc->rx_buf = (u8 *)xfer->rx_buf;

	/* Setup transfer parameters */
	meson_spicc_setup_xfer(spicc, xfer);

	meson_spicc_reset_fifo(spicc);

	/* Setup wait for completion */
	reinit_completion(&spicc->done);

	spicc->irq_count = 0;
	spicc->clk_us = 1000000 / xfer->effective_speed_hz;
	spicc->bpw = xfer->bits_per_word;
	spicc->bpw_packed = meson_spicc_calc_bpw_packed(spicc->bpw);
	spicc->bpw_mask = (1 << spicc->bpw) - 1; /* only used for bpw <= 4 */
	spicc->Bpw = (spicc->bpw + BITS_PER_BYTE - 1) >> 3;
	spicc->words = spicc->words_remain = (xfer->len << 3) / spicc->bpw_packed;

	/* Use DMA for sufficient width and length */
	spicc->using_dma = dma = spicc->bpw >= SPICC_DMA_MIN_BPW && spicc->words >= (spicc->data->fifo_size * SPICC_DMA_MIN_FIFOS);
	//spicc->using_dma = dma = spicc->bpw >= SPICC_DMA_MIN_BPW && spicc->words >= (spicc->data->fifo_size * SPICC_DMA_MIN_FIFOS) && (spicc->bpw % BITS_PER_BYTE) == 0;
/*
	pr_debug("%s: %s\t%uB\t%ubpw\t%upbpw\t%uw\t%uhz\n", __func__,
				dma ? "DMA" : "PIO", xfer->len, spicc->bpw, spicc->bpw_packed, spicc->words, xfer->effective_speed_hz);
*/
	if (dma){
		ret = meson_spicc_dma_map(spicc, xfer);
		if (ret) {
			meson_spicc_dma_unmap(spicc, xfer);
			spicc->xfer = NULL;
			dev_err(host->dev.parent, "dma map failed\n");
			return ret;
		}

		meson_spicc_setup_dma(spicc, true);

		init_time = ktime_to_us(ktime_sub(ktime_get(), start_time));

		if (spicc->data->has_oen){
			if (spicc->bpw > SPICC_PIO_WIDTH)
				time_shift = 26;
			else
				time_shift = 33;
		} else {
			time_shift = 35;
		}

		max_time = (((1ULL * xfer->len) << time_shift) / xfer->effective_speed_hz) + ((spicc->irqs + 1) << 14);
	} else {
		/* bits_per_word < 8 require fifo size to be constrained so rx is byte aligned */
		fifo_shift = 0;
		if (spicc->bpw <= BITS_PER_HALF_BYTE){
			if (spicc->bpw == 1)
				fifo_shift = 3;
			else if (spicc->bpw == 2)
				fifo_shift = 2;
			else
				fifo_shift = 1;
		}
		spicc->fifo_active = spicc->data->fifo_size >> fifo_shift << fifo_shift;

		meson_spicc_setup_pio(spicc, true);

		init_time = ktime_to_us(ktime_sub(ktime_get(), start_time));

		/* transfer time x 2 + 2 millisecond irq service time + 4 millisecond */
		max_time = (((1ULL << 24) * xfer->len) / xfer->effective_speed_hz) + ((spicc->irqs + 1) << 12);
		//if (xfer->effective_speed_hz < 4000000) max_time <<= 4;
	}


	//if (debug) pr_debug("%s\n", __func__);
	if (!wait_for_completion_timeout(&spicc->done, usecs_to_jiffies(max_time))) {
		meson_spicc_dump_reg(spicc);
		writel(0, spicc->base + SPICC_INTREG);

		run_time = ktime_to_us(ktime_sub(ktime_get(), start_time)) - init_time;
		dev_err(host->dev.parent, "%s: %s\t%uB\t%ubpw\t%upbpw\t%uw\t%u irqs\t%uhz\t%lluus init\t%lluus run\t%lluus max\t%llu%%\n",
				__func__, dma ? "DMA" : "PIO",
				xfer->len, spicc->bpw, spicc->bpw_packed, spicc->words, spicc->irqs, xfer->effective_speed_hz,
				init_time, run_time, max_time, run_time * 100 / max_time);


		if (spicc->using_dma){
			writel_bits_relaxed(SPICC_SMC, 0, spicc->base + SPICC_CONREG);
			writel_relaxed(0, spicc->base + SPICC_DMAREG);
			meson_spicc_dma_unmap(spicc, xfer);
		}

		writel_relaxed(readl_relaxed(spicc->base + SPICC_STATREG), spicc->base + SPICC_STATREG);

		spicc->xfer = NULL;
		return -ETIMEDOUT;
	} else if (spicc->words_remain){
		run_time = ktime_to_us(ktime_sub(ktime_get(), start_time)) - init_time;
		dev_err(host->dev.parent, "%s: failed with %lu words remain\n", __func__, spicc->words_remain);
		dev_err(host->dev.parent, "%s: %s\t%uB\t%ubpw\t%upbpw\t%uw\t%u irqs\t%uhz\t%lluus init\t%lluus run\t%lluus max\t%llu%%\n",
				__func__, dma ? "DMA" : "PIO",
				xfer->len, spicc->bpw, spicc->bpw_packed, spicc->words, spicc->irqs, xfer->effective_speed_hz,
				init_time, run_time, max_time, run_time * 100 / max_time);

		spicc->xfer = NULL;
		return -EINVAL;
	}

	run_time = ktime_to_us(ktime_sub(ktime_get(), start_time)) - init_time;
	pr_debug("%s: %s\t%uB\t%ubpw\t%upbpw\t%uw\t%u irqs\t%uhz\t%llu us init\t%llu us run\t%llu us max\t%llu%%\n",
				__func__, dma ? "DMA" : "PIO",
				xfer->len, spicc->bpw, spicc->bpw_packed, spicc->words, spicc->irqs, xfer->effective_speed_hz,
				init_time, run_time, max_time, run_time * 100 / max_time);

	spicc->xfer = NULL;
	return 0;
}

static int meson_spicc_prepare_message(struct spi_controller *host,
				       struct spi_message *message)
{
	struct meson_spicc_device *spicc = spi_controller_get_devdata(host);
	struct spi_device *spi = message->spi;
	u32 conf = readl_relaxed(spicc->base + SPICC_CONREG) & SPICC_DATARATE_MASK;

	/* Store current message */
	spicc->message = message;

	/* Enable Master */
	conf |= SPICC_ENABLE;
	conf |= SPICC_MODE_MASTER;

	/* SMC = 0 */

	/* Setup transfer mode */
	if (spi->mode & SPI_CPOL)
		conf |= SPICC_POL;
	else
		conf &= ~SPICC_POL;

	if (!spicc->data->has_oen) {
		if (spi->mode & SPI_CPOL) {
			if (spicc->pins_idle_high)
				pinctrl_select_state(spicc->pinctrl, spicc->pins_idle_high);
		} else {
			if (spicc->pins_idle_low)
				pinctrl_select_state(spicc->pinctrl, spicc->pins_idle_low);
		}
	}

	if (spi->mode & SPI_CPHA)
		conf |= SPICC_PHA;
	else
		conf &= ~SPICC_PHA;

	/* SSCTL = 0 */

	if (spi->mode & SPI_CS_HIGH)
		conf |= SPICC_SSPOL;
	else
		conf &= ~SPICC_SSPOL;

	if (spi->mode & SPI_READY)
		conf |= FIELD_PREP(SPICC_DRCTL_MASK, SPICC_DRCTL_LOWLEVEL);
	else
		conf |= FIELD_PREP(SPICC_DRCTL_MASK, SPICC_DRCTL_IGNORE);

	/* Select CS */
	conf |= FIELD_PREP(SPICC_CS_MASK, spi_get_chipselect(spi, 0));

	/* use bits_per_word from spi_device */
	conf |= FIELD_PREP(SPICC_BITLENGTH_MASK, spi->bits_per_word - 1);

	writel_relaxed(conf, spicc->base + SPICC_CONREG);

	/* Setup no wait cycles by default */
	writel_relaxed(0, spicc->base + SPICC_PERIODREG);

	writel_bits_relaxed(SPICC_LBC_W1,
				spi->mode & SPI_LOOP ? SPICC_LBC_W1 : 0,
				spicc->base + SPICC_TESTREG);

	return 0;
}

static int meson_spicc_unprepare_transfer(struct spi_controller *host)
{
	struct meson_spicc_device *spicc = spi_controller_get_devdata(host);
	u32 conf = readl_relaxed(spicc->base + SPICC_CONREG) & SPICC_DATARATE_MASK;

	/* Disable all IRQs */
	writel(0, spicc->base + SPICC_INTREG);

	device_reset_optional(&spicc->pdev->dev);

	/* Set default configuration, keeping datarate field */
	writel_relaxed(conf, spicc->base + SPICC_CONREG);

	if (!spicc->data->has_oen)
		pinctrl_select_default_state(&spicc->pdev->dev);

	return 0;
}

static int meson_spicc_setup(struct spi_device *spi)
{
	if (!spi->controller_state)
		spi->controller_state = spi_controller_get_devdata(spi->controller);

	return 0;
}

static void meson_spicc_cleanup(struct spi_device *spi)
{
	spi->controller_state = NULL;
}

/*
 * The Clock Mux
 *            x-----------------x   x------------x    x------\
 *        |---| pow2 fixed div  |---| pow2 div   |----|      |
 *        |   x-----------------x   x------------x    |      |
 * src ---|                                           | mux  |-- out
 *        |   x-----------------x   x------------x    |      |
 *        |---| enh fixed div   |---| enh div    |0---|      |
 *            x-----------------x   x------------x    x------/
 *
 * Clk path for GX series:
 *    src -> pow2 fixed div -> pow2 div -> out
 *
 * Clk path for AXG series:
 *    src -> pow2 fixed div -> pow2 div -> mux -> out
 *    src -> enh fixed div -> enh div -> mux -> out
 *
 * Clk path for G12A series:
 *    pclk -> pow2 fixed div -> pow2 div -> mux -> out
 *    pclk -> enh fixed div -> enh div -> mux -> out
 *
 * The pow2 divider is tied to the controller HW state, and the
 * divider is only valid when the controller is initialized.
 *
 * A set of clock ops is added to make sure we don't read/set this
 * clock rate while the controller is in an unknown state.
 */

static unsigned long meson_spicc_pow2_recalc_rate(struct clk_hw *hw,
						  unsigned long parent_rate)
{
	struct clk_divider *divider = to_clk_divider(hw);
	struct meson_spicc_device *spicc = pow2_clk_to_spicc(divider);

	if (!spicc->host->cur_msg)
		return 0;

	return clk_divider_ops.recalc_rate(hw, parent_rate);
}

static int meson_spicc_pow2_determine_rate(struct clk_hw *hw,
					   struct clk_rate_request *req)
{
	struct clk_divider *divider = to_clk_divider(hw);
	struct meson_spicc_device *spicc = pow2_clk_to_spicc(divider);

	if (!spicc->host->cur_msg)
		return -EINVAL;

	return clk_divider_ops.determine_rate(hw, req);
}

static int meson_spicc_pow2_set_rate(struct clk_hw *hw, unsigned long rate,
				     unsigned long parent_rate)
{
	struct clk_divider *divider = to_clk_divider(hw);
	struct meson_spicc_device *spicc = pow2_clk_to_spicc(divider);

	if (!spicc->host->cur_msg)
		return -EINVAL;

	return clk_divider_ops.set_rate(hw, rate, parent_rate);
}

static const struct clk_ops meson_spicc_pow2_clk_ops = {
	.recalc_rate = meson_spicc_pow2_recalc_rate,
	.determine_rate = meson_spicc_pow2_determine_rate,
	.set_rate = meson_spicc_pow2_set_rate,
};

static int meson_spicc_pow2_clk_init(struct meson_spicc_device *spicc)
{
	struct device *dev = &spicc->pdev->dev;
	struct clk_fixed_factor *pow2_fixed_div;
	struct clk_init_data init;
	struct clk *clk;
	struct clk_parent_data parent_data[2];
	char name[64];

	memset(&init, 0, sizeof(init));
	memset(&parent_data, 0, sizeof(parent_data));

	init.parent_data = parent_data;

	/* algorithm for pow2 div: rate = freq / 4 / (2 ^ N) */

	pow2_fixed_div = devm_kzalloc(dev, sizeof(*pow2_fixed_div), GFP_KERNEL);
	if (!pow2_fixed_div)
		return -ENOMEM;

	snprintf(name, sizeof(name), "%s#pow2_fixed_div", dev_name(dev));
	init.name = name;
	init.ops = &clk_fixed_factor_ops;
	if (spicc->data->has_pclk) {
		init.flags = CLK_SET_RATE_PARENT;
		parent_data[0].hw = __clk_get_hw(spicc->pclk);
	} else {
		init.flags = 0;
		parent_data[0].hw = __clk_get_hw(spicc->core);
	}
	init.num_parents = 1;

	pow2_fixed_div->mult = 1;
	pow2_fixed_div->div = 4;
	pow2_fixed_div->hw.init = &init;

	clk = devm_clk_register(dev, &pow2_fixed_div->hw);
	if (WARN_ON(IS_ERR(clk)))
		return PTR_ERR(clk);

	snprintf(name, sizeof(name), "%s#pow2_div", dev_name(dev));
	init.name = name;
	init.ops = &meson_spicc_pow2_clk_ops;
	/*
	 * Set NOCACHE here to make sure we read the actual HW value
	 * since we reset the HW after each transfer.
	 */
	init.flags = CLK_SET_RATE_PARENT | CLK_GET_RATE_NOCACHE;
	parent_data[0].hw = &pow2_fixed_div->hw;
	init.num_parents = 1;

	spicc->pow2_div.shift = 16;
	spicc->pow2_div.width = 3;
	spicc->pow2_div.flags = CLK_DIVIDER_POWER_OF_TWO;
	spicc->pow2_div.reg = spicc->base + SPICC_CONREG;
	spicc->pow2_div.hw.init = &init;

	spicc->clk = devm_clk_register(dev, &spicc->pow2_div.hw);
	if (WARN_ON(IS_ERR(spicc->clk)))
		return PTR_ERR(spicc->clk);

	return 0;
}

static int meson_spicc_enh_clk_init(struct meson_spicc_device *spicc)
{
	struct device *dev = &spicc->pdev->dev;
	struct clk_fixed_factor *enh_fixed_div;
	struct clk_divider *enh_div;
	struct clk_mux *mux;
	struct clk_init_data init;
	struct clk *clk;
	struct clk_parent_data parent_data[2];
	char name[64];

	memset(&init, 0, sizeof(init));
	memset(&parent_data, 0, sizeof(parent_data));

	init.parent_data = parent_data;

	/* algorithm for enh div: rate = freq / 2 / (N + 1) */

	enh_fixed_div = devm_kzalloc(dev, sizeof(*enh_fixed_div), GFP_KERNEL);
	if (!enh_fixed_div)
		return -ENOMEM;

	snprintf(name, sizeof(name), "%s#enh_fixed_div", dev_name(dev));
	init.name = name;
	init.ops = &clk_fixed_factor_ops;
	if (spicc->data->has_pclk) {
		init.flags = CLK_SET_RATE_PARENT;
		parent_data[0].hw = __clk_get_hw(spicc->pclk);
	} else {
		init.flags = 0;
		parent_data[0].hw = __clk_get_hw(spicc->core);
	}
	init.num_parents = 1;

	enh_fixed_div->mult = 1;
	enh_fixed_div->div = 2;
	enh_fixed_div->hw.init = &init;

	clk = devm_clk_register(dev, &enh_fixed_div->hw);
	if (WARN_ON(IS_ERR(clk)))
		return PTR_ERR(clk);

	enh_div = devm_kzalloc(dev, sizeof(*enh_div), GFP_KERNEL);
	if (!enh_div)
		return -ENOMEM;

	snprintf(name, sizeof(name), "%s#enh_div", dev_name(dev));
	init.name = name;
	init.ops = &clk_divider_ops;
	init.flags = CLK_SET_RATE_PARENT;
	parent_data[0].hw = &enh_fixed_div->hw;
	init.num_parents = 1;

	enh_div->shift	= 16;
	enh_div->width	= 8;
	enh_div->reg = spicc->base + SPICC_ENH_CTL0;
	enh_div->hw.init = &init;

	clk = devm_clk_register(dev, &enh_div->hw);
	if (WARN_ON(IS_ERR(clk)))
		return PTR_ERR(clk);

	mux = devm_kzalloc(dev, sizeof(*mux), GFP_KERNEL);
	if (!mux)
		return -ENOMEM;

	snprintf(name, sizeof(name), "%s#sel", dev_name(dev));
	init.name = name;
	init.ops = &clk_mux_ops;
	parent_data[0].hw = &spicc->pow2_div.hw;
	parent_data[1].hw = &enh_div->hw;
	init.num_parents = 2;
	init.flags = CLK_SET_RATE_PARENT;

	mux->mask = 0x1;
	mux->shift = 24;
	mux->reg = spicc->base + SPICC_ENH_CTL0;
	mux->hw.init = &init;

	spicc->clk = devm_clk_register(dev, &mux->hw);
	if (WARN_ON(IS_ERR(spicc->clk)))
		return PTR_ERR(spicc->clk);

	return 0;
}

static int meson_spicc_probe(struct platform_device *pdev)
{
	struct spi_controller *host;
	struct meson_spicc_device *spicc;
	int ret, irq;

	host = spi_alloc_host(&pdev->dev, sizeof(*spicc));
	if (!host) {
		dev_err(&pdev->dev, "host allocation failed\n");
		return -ENOMEM;
	}
	spicc = spi_controller_get_devdata(host);
	spicc->host = host;

	spicc->data = of_device_get_match_data(&pdev->dev);
	if (!spicc->data) {
		dev_err(&pdev->dev, "failed to get match data\n");
		ret = -EINVAL;
		goto out_host;
	}

	spicc->pdev = pdev;
	platform_set_drvdata(pdev, spicc);

	init_completion(&spicc->done);

	spicc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(spicc->base)) {
		dev_err(&pdev->dev, "io resource mapping failed\n");
		ret = PTR_ERR(spicc->base);
		goto out_host;
	}

	/* Set master mode and enable controller */
	writel_relaxed(SPICC_ENABLE | SPICC_MODE_MASTER,
			spicc->base + SPICC_CONREG);

	/* Disable all IRQs */
	writel_relaxed(0, spicc->base + SPICC_INTREG);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		ret = irq;
		goto out_host;
	}

	ret = devm_request_irq(&pdev->dev, irq, meson_spicc_irq,
				0, NULL, spicc);
	if (ret) {
		dev_err(&pdev->dev, "irq request failed\n");
		goto out_host;
	}

	spicc->core = devm_clk_get_enabled(&pdev->dev, "core");
	if (IS_ERR(spicc->core)) {
		dev_err(&pdev->dev, "core clock request failed\n");
		ret = PTR_ERR(spicc->core);
		goto out_host;
	}

	if (spicc->data->has_pclk) {
		spicc->pclk = devm_clk_get_enabled(&pdev->dev, "pclk");
		if (IS_ERR(spicc->pclk)) {
			dev_err(&pdev->dev, "pclk clock request failed\n");
			ret = PTR_ERR(spicc->pclk);
			goto out_host;
		}
	}

	spicc->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(spicc->pinctrl)) {
		ret = PTR_ERR(spicc->pinctrl);
		goto out_host;
	}

	device_reset_optional(&pdev->dev);

	host->num_chipselect = 4;
	host->dev.of_node = pdev->dev.of_node;
	host->mode_bits = SPI_CPHA | SPI_CPOL | SPI_CS_HIGH | SPI_LOOP;
	host->bits_per_word_mask = SPI_BPW_RANGE_MASK(1,64);
	host->flags = (SPI_CONTROLLER_MUST_RX | SPI_CONTROLLER_MUST_TX);
	host->min_speed_hz = spicc->data->min_speed_hz;
	host->max_speed_hz = spicc->data->max_speed_hz;
	host->setup = meson_spicc_setup;
	host->cleanup = meson_spicc_cleanup;
	host->prepare_message = meson_spicc_prepare_message;
	host->unprepare_transfer_hardware = meson_spicc_unprepare_transfer;
	host->transfer_one = meson_spicc_transfer_one;
	host->max_transfer_size = meson_spicc_max_transfer_size;
	host->use_gpio_descriptors = true;

	meson_spicc_oen_enable(spicc);

	ret = meson_spicc_pow2_clk_init(spicc);
	if (ret) {
		dev_err(&pdev->dev, "pow2 clock registration failed\n");
		goto out_host;
	}

	if (spicc->data->has_enhance_clk_div) {
		ret = meson_spicc_enh_clk_init(spicc);
		if (ret) {
			dev_err(&pdev->dev, "clock registration failed\n");
			goto out_host;
		}
	}

	ret = devm_spi_register_controller(&pdev->dev, host);
	if (ret) {
		dev_err(&pdev->dev, "spi registration failed\n");
		goto out_host;
	}

	return 0;

out_host:
	spi_controller_put(host);

	return ret;
}

static void meson_spicc_remove(struct platform_device *pdev)
{
	struct meson_spicc_device *spicc = platform_get_drvdata(pdev);

	/* Disable SPI */
	writel(0, spicc->base + SPICC_CONREG);

	spi_controller_put(spicc->host);
}

static const struct meson_spicc_data meson_spicc_gx_data = {
	.max_speed_hz		= 41666666,
	.min_speed_hz		= 325521,
	.fifo_size		= 16,
	.dma_bursts_max		= 64,
	.dma_xfers_max		= 128,
};

static const struct meson_spicc_data meson_spicc_axg_data = {
	.max_speed_hz		= 83333332,
	.min_speed_hz		= 325000,
	.fifo_size		= 16,
	.dma_bursts_max		= 65535,
	.dma_xfers_max		= 524280,
	.has_oen		= true,
	.has_enhance_clk_div	= true,
};

static const struct meson_spicc_data meson_spicc_g12a_data = {
	.max_speed_hz		= 166666664,
	.min_speed_hz		= 50000,
	.fifo_size		= 15, /* 16 causes stalls */
	.dma_bursts_max		= 65535,
	.dma_xfers_max		= 524280,
	.has_oen		= true,
	.has_enhance_clk_div	= true,
	.has_pclk		= true,
};

static const struct of_device_id meson_spicc_of_match[] = {
	{
		.compatible	= "amlogic,meson-gx-spicc",
		.data		= &meson_spicc_gx_data,
	},
	{
		.compatible = "amlogic,meson-axg-spicc",
		.data		= &meson_spicc_axg_data,
	},
	{
		.compatible = "amlogic,meson-g12a-spicc",
		.data		= &meson_spicc_g12a_data,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, meson_spicc_of_match);

static struct platform_driver meson_spicc_driver = {
	.probe   = meson_spicc_probe,
	.remove_new = meson_spicc_remove,
	.driver  = {
		.name = "meson-spicc",
		.of_match_table = of_match_ptr(meson_spicc_of_match),
	},
};

module_platform_driver(meson_spicc_driver);

MODULE_DESCRIPTION("Meson SPI Communication Controller driver");
MODULE_AUTHOR("Neil Armstrong <narmstrong@baylibre.com>");
MODULE_LICENSE("GPL");
