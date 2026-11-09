// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2019 Xilinx, Inc.
 *
 * Author: Naga Sureshkumar Relli <nagasure@xilinx.com>
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <linux/spi/spi-mem.h>

/* Register offset definitions */
#define ZYNQ_QSPI_CONFIG_OFFSET		0x00 /* Configuration  Register, RW */
#define ZYNQ_QSPI_STATUS_OFFSET		0x04 /* Interrupt Status Register, RO */
#define ZYNQ_QSPI_IEN_OFFSET		0x08 /* Interrupt Enable Register, WO */
#define ZYNQ_QSPI_IDIS_OFFSET		0x0C /* Interrupt Disable Reg, WO */
#define ZYNQ_QSPI_IMASK_OFFSET		0x10 /* Interrupt Enabled Mask Reg,RO */
#define ZYNQ_QSPI_ENABLE_OFFSET		0x14 /* Enable/Disable Register, RW */
#define ZYNQ_QSPI_DELAY_OFFSET		0x18 /* Delay Register, RW */
#define ZYNQ_QSPI_TXD_00_00_OFFSET	0x1C /* Transmit 4-byte inst, WO */
#define ZYNQ_QSPI_TXD_00_01_OFFSET	0x80 /* Transmit 1-byte inst, WO */
#define ZYNQ_QSPI_TXD_00_10_OFFSET	0x84 /* Transmit 2-byte inst, WO */
#define ZYNQ_QSPI_TXD_00_11_OFFSET	0x88 /* Transmit 3-byte inst, WO */
#define ZYNQ_QSPI_RXD_OFFSET		0x20 /* Data Receive Register, RO */
#define ZYNQ_QSPI_SIC_OFFSET		0x24 /* Slave Idle Count Register, RW */
#define ZYNQ_QSPI_TX_THRESH_OFFSET	0x28 /* TX FIFO Watermark Reg, RW */
#define ZYNQ_QSPI_RX_THRESH_OFFSET	0x2C /* RX FIFO Watermark Reg, RW */
#define ZYNQ_QSPI_GPIO_OFFSET		0x30 /* GPIO Register, RW */
#define ZYNQ_QSPI_LINEAR_CFG_OFFSET	0xA0 /* Linear Adapter Config Ref, RW */
#define ZYNQ_QSPI_MOD_ID_OFFSET		0xFC /* Module ID Register, RO */

/*
 * QSPI Configuration Register bit Masks
 *
 * This register contains various control bits that effect the operation
 * of the QSPI controller
 */
#define ZYNQ_QSPI_CONFIG_IFMODE_MASK	BIT(31) /* Flash Memory Interface */
#define ZYNQ_QSPI_CONFIG_MANSRT_MASK	BIT(16) /* Manual TX Start */
#define ZYNQ_QSPI_CONFIG_MANSRTEN_MASK	BIT(15) /* Enable Manual TX Mode */
#define ZYNQ_QSPI_CONFIG_SSFORCE_MASK	BIT(14) /* Manual Chip Select */
#define ZYNQ_QSPI_CONFIG_BDRATE_MASK	GENMASK(5, 3) /* Baud Rate Mask */
#define ZYNQ_QSPI_CONFIG_CPHA_MASK	BIT(2) /* Clock Phase Control */
#define ZYNQ_QSPI_CONFIG_CPOL_MASK	BIT(1) /* Clock Polarity Control */
#define ZYNQ_QSPI_CONFIG_FWIDTH_MASK	GENMASK(7, 6) /* FIFO width */
#define ZYNQ_QSPI_CONFIG_MSTREN_MASK	BIT(0) /* Master Mode */

/*
 * QSPI Configuration Register - Baud rate and target select
 *
 * These are the values used in the calculation of baud rate divisor and
 * setting the target select.
 */
#define ZYNQ_QSPI_CONFIG_BAUD_DIV_MAX	GENMASK(2, 0) /* Baud rate maximum */
#define ZYNQ_QSPI_CONFIG_BAUD_DIV_SHIFT	3 /* Baud rate divisor shift */
#define ZYNQ_QSPI_CONFIG_PCS		BIT(10) /* Peripheral Chip Select */

/*
 * QSPI Interrupt Registers bit Masks
 *
 * All the four interrupt registers (Status/Mask/Enable/Disable) have the same
 * bit definitions.
 */
#define ZYNQ_QSPI_IXR_RX_OVERFLOW_MASK	BIT(0) /* QSPI RX FIFO Overflow */
#define ZYNQ_QSPI_IXR_TXNFULL_MASK	BIT(2) /* QSPI TX FIFO Overflow */
#define ZYNQ_QSPI_IXR_TXFULL_MASK	BIT(3) /* QSPI TX FIFO is full */
#define ZYNQ_QSPI_IXR_RXNEMTY_MASK	BIT(4) /* QSPI RX FIFO Not Empty */
#define ZYNQ_QSPI_IXR_RXF_FULL_MASK	BIT(5) /* QSPI RX FIFO is full */
#define ZYNQ_QSPI_IXR_TXF_UNDRFLOW_MASK	BIT(6) /* QSPI TX FIFO Underflow */
#define ZYNQ_QSPI_IXR_ALL_MASK		(ZYNQ_QSPI_IXR_RX_OVERFLOW_MASK | \
					ZYNQ_QSPI_IXR_TXNFULL_MASK | \
					ZYNQ_QSPI_IXR_TXFULL_MASK | \
					ZYNQ_QSPI_IXR_RXNEMTY_MASK | \
					ZYNQ_QSPI_IXR_RXF_FULL_MASK | \
					ZYNQ_QSPI_IXR_TXF_UNDRFLOW_MASK)
#define ZYNQ_QSPI_IXR_RXTX_MASK		(ZYNQ_QSPI_IXR_TXNFULL_MASK | \
					ZYNQ_QSPI_IXR_RXNEMTY_MASK)

/*
 * QSPI Enable Register bit Masks
 *
 * This register is used to enable or disable the QSPI controller
 */
#define ZYNQ_QSPI_ENABLE_ENABLE_MASK	BIT(0) /* QSPI Enable Bit Mask */

/*
 * Bounded spin, in the IRQ handler, waiting for the shifter to land the next
 * RX FIFO entry before it is drained.  Deliberately short: this is a latency
 * hedge against a drain that runs ahead of the shifter, NOT a correctness
 * gate -- the read after it is unconditional (see zynq_qspi_read_op()).
 */
#define ZYNQ_QSPI_RX_WAIT_SPINS		1000

/*
 * QSPI Linear Configuration Register
 *
 * It is named Linear Configuration but it controls other modes when not in
 * linear mode also.
 */
#define ZYNQ_QSPI_LCFG_TWO_MEM		BIT(30) /* LQSPI Two memories */
#define ZYNQ_QSPI_LCFG_SEP_BUS		BIT(29) /* LQSPI Separate bus */
#define ZYNQ_QSPI_LCFG_U_PAGE		BIT(28) /* LQSPI Upper Page */

#define ZYNQ_QSPI_LCFG_DUMMY_SHIFT	8

#define ZYNQ_QSPI_FAST_READ_QOUT_CODE	0x6B /* read instruction code */
#define ZYNQ_QSPI_FIFO_DEPTH		63 /* FIFO depth in words */
#define ZYNQ_QSPI_RX_THRESHOLD		32 /* Words drained/filled per IRQ */
#define ZYNQ_QSPI_TX_THRESHOLD		1 /* Tx FIFO threshold level */

/*
 * RX FIFO watermark programmed into RX_THRESH.
 *
 * STATUS.RXNEMTY (BIT(4)) is not a literal "FIFO non-empty" flag: the
 * controller asserts it only once the RX FIFO holds at least RX_THRESH
 * entries.  The driver's only FIFO-occupancy gate -- the bounded spin in
 * zynq_qspi_read_op() -- polls exactly that bit, so with RX_THRESH left at
 * the reset/batch value of 32 the gate can never cover the last entries of a
 * transfer, nor any transfer shorter than 32 words.  Every spi-nor control op
 * (RDID, RDSR, RDSFDP, the 4-byte address prologue) is shorter than that, so
 * for those ops the spin always ran to its 1000-iteration bound and returned
 * without the entry ever being confirmed present.
 *
 * MEASURED on an ANTSDR E310 (Zynq-7020 + W25Q256), 2026-07-28, with
 * RX_THRESH = 32: a 32 MiB `dd if=/dev/mtd0` incremented rx_starved by
 * exactly 1024 -- one per 32 KiB spi_mem_op, i.e. the tail word of every op
 * -- and a 4 KiB write incremented it by 513.  With RX_THRESH = 1 both
 * deltas are 0.
 *
 * Kept distinct from ZYNQ_QSPI_RX_THRESHOLD above, which is a software batch
 * size (words moved per interrupt) and must stay large for throughput; the
 * two were conflated only because 32 happened to be the reset value of the
 * register.
 */
#define ZYNQ_QSPI_RX_WATERMARK		1

/*
 * The modebits configurable by the driver to make the SPI support different
 * data formats
 */
#define ZYNQ_QSPI_MODEBITS			(SPI_CPOL | SPI_CPHA)

/* Maximum number of chip selects */
#define ZYNQ_QSPI_MAX_NUM_CS		2

/**
 * struct zynq_qspi - Defines qspi driver instance
 * @dev:		Pointer to the this device's information
 * @regs:		Virtual address of the QSPI controller registers
 * @refclk:		Pointer to the peripheral clock
 * @pclk:		Pointer to the APB clock
 * @irq:		IRQ number
 * @txbuf:		Pointer to the TX buffer
 * @rxbuf:		Pointer to the RX buffer
 * @tx_bytes:		Number of bytes left to transfer
 * @rx_bytes:		Number of bytes left to receive
 * @data_completion:	completion structure
 * @rx_starved:		Count of drains aborted because the RX FIFO ran dry
 * @rx_underflow:	Count of RX byte-accounting underflows (framing desync)
 */
/* Max data bytes a single debugfs raw op may read */
#define ZYNQ_QSPI_RAWOP_MAX	256

struct zynq_qspi {
	struct device *dev;
	void __iomem *regs;
	struct clk *refclk;
	struct clk *pclk;
	int irq;
	u32 rx_starved;
	u32 rx_underflow;
	struct dentry *dbgdir;
	u8 *txbuf;
	u8 *rxbuf;
	int tx_bytes;
	int rx_bytes;
	struct completion data_completion;
	struct spi_controller *ctlr;
	/* debugfs raw spi_mem_op probe -- see zynq_qspi_rawop_write() */
	struct mutex rawop_lock;
	int rawop_ret;
	unsigned int rawop_len;
	u8 rawop_buf[ZYNQ_QSPI_RAWOP_MAX];
};

/*
 * Inline functions for the QSPI controller read/write
 */
static inline u32 zynq_qspi_read(struct zynq_qspi *xqspi, u32 offset)
{
	return readl_relaxed(xqspi->regs + offset);
}

static inline void zynq_qspi_write(struct zynq_qspi *xqspi, u32 offset,
				   u32 val)
{
	writel_relaxed(val, xqspi->regs + offset);
}

/**
 * zynq_qspi_init_hw - Initialize the hardware
 * @xqspi:	Pointer to the zynq_qspi structure
 * @num_cs:	Number of connected CS (to enable dual memories if needed)
 *
 * The default settings of the QSPI controller's configurable parameters on
 * reset are
 *	- Host mode
 *	- Baud rate divisor is set to 2
 *	- Tx threshold set to 1l Rx threshold set to 32
 *	- Flash memory interface mode enabled
 *	- Size of the word to be transferred as 8 bit
 * This function performs the following actions
 *	- Disable and clear all the interrupts
 *	- Enable manual target select
 *	- Enable manual start
 *	- Deselect all the chip select lines
 *	- Set the size of the word to be transferred as 32 bit
 *	- Set the little endian mode of TX FIFO and
 *	- Enable the QSPI controller
 */
static void zynq_qspi_init_hw(struct zynq_qspi *xqspi, unsigned int num_cs)
{
	u32 config_reg;

	zynq_qspi_write(xqspi, ZYNQ_QSPI_ENABLE_OFFSET, 0);
	zynq_qspi_write(xqspi, ZYNQ_QSPI_IDIS_OFFSET, ZYNQ_QSPI_IXR_ALL_MASK);

	/* Disable linear mode as the boot loader may have used it */
	config_reg = 0;
	/* At the same time, enable dual mode if more than 1 CS is available */
	if (num_cs > 1)
		config_reg |= ZYNQ_QSPI_LCFG_TWO_MEM;

	zynq_qspi_write(xqspi, ZYNQ_QSPI_LINEAR_CFG_OFFSET, config_reg);

	/*
	 * Program the RX watermark BEFORE the flush below: STATUS.RXNEMTY is
	 * gated on RX_THRESH, so at the register's reset value of 32 the flush
	 * loop cannot see -- and therefore cannot drain -- a residue of fewer
	 * than 32 entries left by the boot loader's linear-mode reads.
	 */
	zynq_qspi_write(xqspi, ZYNQ_QSPI_RX_THRESH_OFFSET,
			ZYNQ_QSPI_RX_WATERMARK);

	/* Clear the RX FIFO */
	while (zynq_qspi_read(xqspi, ZYNQ_QSPI_STATUS_OFFSET) &
			      ZYNQ_QSPI_IXR_RXNEMTY_MASK)
		zynq_qspi_read(xqspi, ZYNQ_QSPI_RXD_OFFSET);

	zynq_qspi_write(xqspi, ZYNQ_QSPI_STATUS_OFFSET, ZYNQ_QSPI_IXR_ALL_MASK);
	config_reg = zynq_qspi_read(xqspi, ZYNQ_QSPI_CONFIG_OFFSET);
	config_reg &= ~(ZYNQ_QSPI_CONFIG_MSTREN_MASK |
			ZYNQ_QSPI_CONFIG_CPOL_MASK |
			ZYNQ_QSPI_CONFIG_CPHA_MASK |
			ZYNQ_QSPI_CONFIG_BDRATE_MASK |
			ZYNQ_QSPI_CONFIG_SSFORCE_MASK |
			ZYNQ_QSPI_CONFIG_MANSRTEN_MASK |
			ZYNQ_QSPI_CONFIG_MANSRT_MASK);
	config_reg |= (ZYNQ_QSPI_CONFIG_MSTREN_MASK |
		       ZYNQ_QSPI_CONFIG_SSFORCE_MASK |
		       ZYNQ_QSPI_CONFIG_FWIDTH_MASK |
		       ZYNQ_QSPI_CONFIG_IFMODE_MASK);
	zynq_qspi_write(xqspi, ZYNQ_QSPI_CONFIG_OFFSET, config_reg);

	zynq_qspi_write(xqspi, ZYNQ_QSPI_TX_THRESH_OFFSET,
			ZYNQ_QSPI_TX_THRESHOLD);

	zynq_qspi_write(xqspi, ZYNQ_QSPI_ENABLE_OFFSET,
			ZYNQ_QSPI_ENABLE_ENABLE_MASK);
}

static bool zynq_qspi_supports_op(struct spi_mem *mem,
				  const struct spi_mem_op *op)
{
	if (!spi_mem_default_supports_op(mem, op))
		return false;

	/* zynq_qspi_exec_mem_op() stages the address in a u8[8]. */
	if (op->addr.nbytes > 4)
		return false;

	/*
	 * A 4-byte address is fine on its own -- exec_mem_op() is the generic
	 * I/O-mode path and shifts out exactly the bytes it is given -- but ONLY
	 * while every phase stays single-lane.
	 *
	 * The controller runs with CONFIG.IFMODE (flash memory interface mode)
	 * set, in which it decodes the opcode itself to know when to turn the
	 * data lanes around for a multi-I/O read.  That decoder assumes the
	 * 3-byte-address form of those opcodes.  Hand it a 4-byte address and it
	 * turns the bus around one prologue byte early, so the captured stream
	 * begins with three lane-fill bytes (0x88) and every real byte lands
	 * three positions late.
	 *
	 * MEASURED on an ANTSDR E310 (Zynq-7020 + W25Q256), 2026-07-28, with the
	 * address staging already fixed:
	 *
	 *   0x0C 4-byte addr, 1 dummy, all 1S  @0x0000000  fe ff ff ea fe ...
	 *   0x13 4-byte addr, no dummy, all 1S @0x1400000  f9 ef f5 6b 57 ...
	 *      -- byte-identical to each other and to the 3-byte 0x03 read of
	 *         the same offsets; the 0x1400010 read is the 0x1400000 read
	 *         shifted by exactly 16 bytes, so >16 MiB addressing tracks.
	 *
	 *   0x6B 4-byte addr, 1 dummy, data 4S (what spi-nor picks when the DT
	 *   says spi-rx-bus-width = <4>), read back through /dev/mtd0:
	 *      88 88 88 fe ff ff ea fe ...   <- the same content, 3 bytes late
	 *
	 * This is the residual that survived BOTH earlier attempts at >16 MiB
	 * support (e728d7f9b3a34, reverted by 1c1b349fa1d8a; and the deleted
	 * lc618-e310-qspi ba32d8ac9bb43): each fixed a real bug elsewhere and
	 * each left spi-rx-bus-width = <4> in place, so each kept selecting
	 * 0x6B and kept seeing the identical three-byte displacement.
	 *
	 * Refusing the combination makes spi_nor_spimem_adjust_hwcaps() drop the
	 * multi-I/O read candidates for a >16 MiB part and keep the 1S-1S-1S
	 * ones, i.e. FAST_READ_4B (0x0C).  Parts at or below 16 MiB use a 3-byte
	 * address and are completely unaffected, so every other board using this
	 * controller keeps its quad reads.
	 */
	if (op->addr.nbytes > 3 &&
	    (op->cmd.buswidth > 1 || op->addr.buswidth > 1 ||
	     op->dummy.buswidth > 1 || op->data.buswidth > 1))
		return false;

	return true;
}

/*
 * Raw spi_mem_op probe (debugfs).
 *
 * The flash's spi_device is created from DT and survives a spi-nor bind
 * failure, so a spi_mem can be built around it here to issue identification
 * opcodes (JEDEC ID, SFDP, unique ID, status registers) that no other path on
 * this board can reach: U-Boot has no `sspi` command and Linux has no /dev/mtd.
 *
 * Write:  "<op> <addr_nbytes> <addr> <dummy_nbytes> <data_nbytes>"  (all hex,
 *         addr/dummy/data may be omitted and default to 0)
 * Read:   "ret=<n> len=<n> <hex bytes>"
 *
 * READ-ONLY BY CONSTRUCTION: data is always SPI_MEM_DATA_IN, and the opcode
 * must be in the allowlist below.  This is the board's boot flash -- a stray
 * erase or program bricks the SPL.
 */
static const u8 zynq_qspi_rawop_allowed[] = {
	0x9f,		/* RDID -- JEDEC ID */
	0x5a,		/* RDSFDP */
	0x4b,		/* Read Unique ID */
	0x90, 0x92, 0x94,	/* Read Manufacturer/Device ID (+dual/quad) */
	0xab,		/* Release Power-Down / Device ID */
	0x05, 0x35, 0x15,	/* RDSR1 / RDSR2 / RDSR3 */
	0x03, 0x0b,	/* READ / FAST_READ (3-byte address) */
	0x13, 0x0c,	/* READ4B / FAST_READ4B */
	0x16,		/* RDEAR/RDBR (extended address register, some parts) */
	0xc8,		/* Read Extended Address Register (Winbond) */
};

static int zynq_qspi_rawop_match_child(struct device *dev, const void *data)
{
	return 1;
}

static ssize_t zynq_qspi_rawop_write(struct file *file,
				     const char __user *ubuf,
				     size_t count, loff_t *ppos)
{
	struct zynq_qspi *xqspi = file->private_data;
	unsigned int opcode, addr_nbytes = 0, dummy_nbytes = 0, data_nbytes = 0;
	unsigned long long addr = 0;
	struct spi_device *spi;
	struct spi_mem mem = { };
	struct spi_mem_op op;
	struct device *child;
	char buf[128];
	bool allowed = false;
	int i, ret;

	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';

	if (sscanf(buf, "%x %x %llx %x %x", &opcode, &addr_nbytes, &addr,
		   &dummy_nbytes, &data_nbytes) < 1)
		return -EINVAL;

	if (opcode > 0xff || addr_nbytes > 4 || dummy_nbytes > 16 ||
	    data_nbytes > ZYNQ_QSPI_RAWOP_MAX)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(zynq_qspi_rawop_allowed); i++)
		if (zynq_qspi_rawop_allowed[i] == opcode)
			allowed = true;
	if (!allowed)
		return -EPERM;

	child = device_find_child(&xqspi->ctlr->dev, NULL,
				  zynq_qspi_rawop_match_child);
	if (!child)
		return -ENODEV;
	spi = to_spi_device(child);

	mutex_lock(&xqspi->rawop_lock);

	memset(xqspi->rawop_buf, 0, sizeof(xqspi->rawop_buf));
	mem.spi = spi;
	op = (struct spi_mem_op)
		SPI_MEM_OP(SPI_MEM_OP_CMD(opcode, 1),
			   SPI_MEM_OP_ADDR(addr_nbytes, addr, 1),
			   SPI_MEM_OP_DUMMY(dummy_nbytes, 1),
			   SPI_MEM_OP_DATA_IN(data_nbytes, xqspi->rawop_buf, 1));
	if (!addr_nbytes)
		op.addr.buswidth = 0;
	if (!dummy_nbytes)
		op.dummy.buswidth = 0;
	if (!data_nbytes)
		op.data.buswidth = 0;

	ret = spi_mem_exec_op(&mem, &op);

	xqspi->rawop_ret = ret;
	xqspi->rawop_len = ret ? 0 : data_nbytes;

	mutex_unlock(&xqspi->rawop_lock);
	put_device(child);

	*ppos += count;
	return count;
}

static ssize_t zynq_qspi_rawop_read(struct file *file, char __user *ubuf,
				    size_t count, loff_t *ppos)
{
	struct zynq_qspi *xqspi = file->private_data;
	char *out;
	int len = 0, i;
	ssize_t ret;

	out = kmalloc(ZYNQ_QSPI_RAWOP_MAX * 3 + 64, GFP_KERNEL);
	if (!out)
		return -ENOMEM;

	mutex_lock(&xqspi->rawop_lock);
	len += scnprintf(out + len, 64, "ret=%d len=%u",
			 xqspi->rawop_ret, xqspi->rawop_len);
	for (i = 0; i < xqspi->rawop_len; i++)
		len += scnprintf(out + len, 4, " %02x", xqspi->rawop_buf[i]);
	mutex_unlock(&xqspi->rawop_lock);

	len += scnprintf(out + len, 2, "\n");

	ret = simple_read_from_buffer(ubuf, count, ppos, out, len);
	kfree(out);

	return ret;
}

static const struct file_operations zynq_qspi_rawop_fops = {
	.owner	= THIS_MODULE,
	.open	= simple_open,
	.read	= zynq_qspi_rawop_read,
	.write	= zynq_qspi_rawop_write,
	.llseek	= default_llseek,
};

/**
 * zynq_qspi_rxfifo_op - Read 1..4 bytes from RxFIFO to RX buffer
 * @xqspi:	Pointer to the zynq_qspi structure
 * @size:	Number of bytes to be read (1..4)
 */
static void zynq_qspi_rxfifo_op(struct zynq_qspi *xqspi, unsigned int size)
{
	u32 data;

	data = zynq_qspi_read(xqspi, ZYNQ_QSPI_RXD_OFFSET);

	if (xqspi->rxbuf) {
		/*
		 * A partial RX FIFO entry is TOP-aligned: the controller
		 * right-justifies the entry into the MSB end of the word, so
		 * the valid bytes are the HIGH `size` bytes and the low bytes
		 * are pad.  A full 4-byte entry has no pad and is copied
		 * verbatim by the bulk path, which is why only the tail of a
		 * non-multiple-of-4 transfer is affected.
		 *
		 * MEASURED on an ANTSDR E310 (Zynq-7020 + W25Q256) 2026-07-28
		 * via the debugfs rawop probe, reading the same offset at
		 * several lengths.  Lengths 4 and 8 take no partial path and
		 * are the ground truth:
		 *
		 *   03 3-byte addr @0x000000, 8 bytes: fe ff ff ea fe ff ff ea
		 *                              7 bytes: fe ff ff ea ea fe ff
		 *
		 * The 7-byte read's partial entry held b0..b2 = ea fe ff, i.e.
		 * the correct tail (fe ff ff) sits at b1..b3.  Bottom-aligned
		 * extraction returns b0..b2 and is one byte late.
		 *
		 * This restores the mainline / Xilinx-vendor behaviour that
		 * b2cf37f8bbd79 inverted.  That commit justified the inversion
		 * by citing U-Boot's zynq_qspi_read_data() as bottom-aligned,
		 * but U-Boot's zynq_qspi_fill_tx_fifo() forces the 4-byte TXD0
		 * register whenever a read buffer is set, so U-Boot never
		 * creates a partial RX entry and says nothing about this.
		 *
		 * The damage was not limited to a cosmetic ID tail.  RDSR is a
		 * 1-byte data-in, so spi_nor_wait_till_ready() read the pad
		 * byte instead of SR1 and saw WIP clear while the chip was
		 * still programming; spi_nor_write() then issued the next WREN
		 * and PAGE PROGRAM into a busy chip, which ignored both, and
		 * every second page of a multi-page write was silently lost.
		 */
		memcpy(xqspi->rxbuf, ((u8 *)&data) + 4 - size, size);
		xqspi->rxbuf += size;
	}

	xqspi->rx_bytes -= size;
	if (xqspi->rx_bytes < 0) {
		/*
		 * Consuming more bytes than were accounted for means the RX
		 * FIFO framing has desynchronised from our byte counts; the
		 * leftover entry will corrupt the next phase of the op.  This
		 * used to be clamped away silently.
		 */
		xqspi->rx_underflow++;
		dev_err_ratelimited(xqspi->dev,
				    "RX accounting underflow (%d bytes); FIFO framing desynchronised\n",
				    xqspi->rx_bytes);
		xqspi->rx_bytes = 0;
	}
}

/**
 * zynq_qspi_txfifo_op - Write 1..4 bytes from TX buffer to TxFIFO
 * @xqspi:	Pointer to the zynq_qspi structure
 * @size:	Number of bytes to be written (1..4)
 */
static void zynq_qspi_txfifo_op(struct zynq_qspi *xqspi, unsigned int size)
{
	static const unsigned int offset[4] = {
		ZYNQ_QSPI_TXD_00_01_OFFSET, ZYNQ_QSPI_TXD_00_10_OFFSET,
		ZYNQ_QSPI_TXD_00_11_OFFSET, ZYNQ_QSPI_TXD_00_00_OFFSET };
	u32 data;

	if (xqspi->txbuf) {
		data = 0xffffffff;
		memcpy(&data, xqspi->txbuf, size);
		xqspi->txbuf += size;
	} else {
		data = 0;
	}

	xqspi->tx_bytes -= size;
	zynq_qspi_write(xqspi, offset[size - 1], data);
}

/**
 * zynq_qspi_chipselect - Select or deselect the chip select line
 * @spi:	Pointer to the spi_device structure
 * @assert:	1 for select or 0 for deselect the chip select line
 */
static void zynq_qspi_chipselect(struct spi_device *spi, bool assert)
{
	struct spi_controller *ctlr = spi->controller;
	struct zynq_qspi *xqspi = spi_controller_get_devdata(ctlr);
	u32 config_reg;

	/* Select the lower (CS0) or upper (CS1) memory */
	if (ctlr->num_chipselect > 1) {
		config_reg = zynq_qspi_read(xqspi, ZYNQ_QSPI_LINEAR_CFG_OFFSET);
		if (!spi_get_chipselect(spi, 0))
			config_reg &= ~ZYNQ_QSPI_LCFG_U_PAGE;
		else
			config_reg |= ZYNQ_QSPI_LCFG_U_PAGE;

		zynq_qspi_write(xqspi, ZYNQ_QSPI_LINEAR_CFG_OFFSET, config_reg);
	}

	/* Ground the line to assert the CS */
	config_reg = zynq_qspi_read(xqspi, ZYNQ_QSPI_CONFIG_OFFSET);
	if (assert)
		config_reg &= ~ZYNQ_QSPI_CONFIG_PCS;
	else
		config_reg |= ZYNQ_QSPI_CONFIG_PCS;

	zynq_qspi_write(xqspi, ZYNQ_QSPI_CONFIG_OFFSET, config_reg);
}

/**
 * zynq_qspi_config_op - Configure QSPI controller for specified transfer
 * @xqspi:	Pointer to the zynq_qspi structure
 * @spi:	Pointer to the spi_device structure
 * @op:		The memory operation to execute
 *
 * Sets the operational mode of QSPI controller for the next QSPI transfer and
 * sets the requested clock frequency.
 *
 * Return:	0 on success and -EINVAL on invalid input parameter
 *
 * Note: If the requested frequency is not an exact match with what can be
 * obtained using the prescalar value, the driver sets the clock frequency which
 * is lower than the requested frequency (maximum lower) for the transfer. If
 * the requested frequency is higher or lower than that is supported by the QSPI
 * controller the driver will set the highest or lowest frequency supported by
 * controller.
 */
static int zynq_qspi_config_op(struct zynq_qspi *xqspi, struct spi_device *spi,
			       const struct spi_mem_op *op)
{
	u32 config_reg, baud_rate_val = 0;

	/*
	 * Set the clock frequency
	 * The baud rate divisor is not a direct mapping to the value written
	 * into the configuration register (config_reg[5:3])
	 * i.e. 000 - divide by 2
	 *      001 - divide by 4
	 *      ----------------
	 *      111 - divide by 256
	 */
	while ((baud_rate_val < ZYNQ_QSPI_CONFIG_BAUD_DIV_MAX)  &&
	       (clk_get_rate(xqspi->refclk) / (2 << baud_rate_val)) >
		op->max_freq)
		baud_rate_val++;

	config_reg = zynq_qspi_read(xqspi, ZYNQ_QSPI_CONFIG_OFFSET);

	/* Set the QSPI clock phase and clock polarity */
	config_reg &= (~ZYNQ_QSPI_CONFIG_CPHA_MASK) &
		      (~ZYNQ_QSPI_CONFIG_CPOL_MASK);
	if (spi->mode & SPI_CPHA)
		config_reg |= ZYNQ_QSPI_CONFIG_CPHA_MASK;
	if (spi->mode & SPI_CPOL)
		config_reg |= ZYNQ_QSPI_CONFIG_CPOL_MASK;

	config_reg &= ~ZYNQ_QSPI_CONFIG_BDRATE_MASK;
	config_reg |= (baud_rate_val << ZYNQ_QSPI_CONFIG_BAUD_DIV_SHIFT);
	zynq_qspi_write(xqspi, ZYNQ_QSPI_CONFIG_OFFSET, config_reg);

	return 0;
}

/**
 * zynq_qspi_setup_op - Configure the QSPI controller
 * @spi:	Pointer to the spi_device structure
 *
 * Sets the operational mode of QSPI controller for the next QSPI transfer, baud
 * rate and divisor value to setup the requested qspi clock.
 *
 * Return:	0 on success and error value on failure
 */
static int zynq_qspi_setup_op(struct spi_device *spi)
{
	struct spi_controller *ctlr = spi->controller;
	struct zynq_qspi *qspi = spi_controller_get_devdata(ctlr);

	if (ctlr->busy)
		return -EBUSY;

	zynq_qspi_write(qspi, ZYNQ_QSPI_ENABLE_OFFSET,
			ZYNQ_QSPI_ENABLE_ENABLE_MASK);

	return 0;
}

/**
 * zynq_qspi_write_op - Fills the TX FIFO with as many bytes as possible
 * @xqspi:	Pointer to the zynq_qspi structure
 * @txcount:	Maximum number of words to write
 * @txempty:	Indicates that TxFIFO is empty
 */
static void zynq_qspi_write_op(struct zynq_qspi *xqspi, int txcount,
			       bool txempty)
{
	int count, len, k;

	len = xqspi->tx_bytes;
	if (len && len < 4) {
		/*
		 * We must empty the TxFIFO between accesses to TXD0,
		 * TXD1, TXD2, TXD3.
		 */
		if (txempty)
			zynq_qspi_txfifo_op(xqspi, len);

		return;
	}

	count = len / 4;
	if (count > txcount)
		count = txcount;

	if (xqspi->txbuf) {
		iowrite32_rep(xqspi->regs + ZYNQ_QSPI_TXD_00_00_OFFSET,
			      xqspi->txbuf, count);
		xqspi->txbuf += count * 4;
	} else {
		for (k = 0; k < count; k++)
			writel_relaxed(0, xqspi->regs +
					  ZYNQ_QSPI_TXD_00_00_OFFSET);
	}

	xqspi->tx_bytes -= count * 4;
}

/**
 * zynq_qspi_read_op - Drains the RX FIFO by as many bytes as possible
 * @xqspi:	Pointer to the zynq_qspi structure
 * @rxcount:	Maximum number of words to read
 */
static void zynq_qspi_read_op(struct zynq_qspi *xqspi, int rxcount)
{
	int count, len, k;

	len = xqspi->rx_bytes - xqspi->tx_bytes;
	count = len / 4;
	if (count > rxcount)
		count = rxcount;

	for (k = 0; k < count; k++) {
		u32 data;
		int spins;

		/*
		 * tx_bytes is decremented when bytes are pushed INTO the TX
		 * FIFO, not when they are shifted out, so `len` above counts
		 * the bytes that will EVENTUALLY arrive -- not the bytes
		 * available right now.  When the drain runs ahead of the
		 * shifter, RXD returns the controller's pad value and
		 * displaces every subsequent byte of the transfer.
		 *
		 * Give the shifter a bounded window to land the word.  The
		 * read below is then unconditional: this runs in the IRQ
		 * handler, and the interrupt is level-triggered on TXNFULL,
		 * so skipping the read would leave rx_bytes non-zero forever
		 * -- the completion never fires and the handler re-enters
		 * without limit.  (Measured on an ANTSDR E310, 2026-07-26: an
		 * earlier version of this code that skipped the read livelocked
		 * the board in spi_nor_read_id() with an RCU stall on CPU0 and
		 * ~3.7M suppressed log callbacks per 5 s.)  rx_starved counts
		 * the words the wait failed to cover; a non-zero value means
		 * data returned for that op is suspect.
		 */
		for (spins = 0; spins < ZYNQ_QSPI_RX_WAIT_SPINS; spins++) {
			if (zynq_qspi_read(xqspi, ZYNQ_QSPI_STATUS_OFFSET) &
			    ZYNQ_QSPI_IXR_RXNEMTY_MASK)
				break;
			cpu_relax();
		}
		if (spins == ZYNQ_QSPI_RX_WAIT_SPINS)
			xqspi->rx_starved++;

		data = zynq_qspi_read(xqspi, ZYNQ_QSPI_RXD_OFFSET);
		if (xqspi->rxbuf) {
			memcpy(xqspi->rxbuf, &data, 4);
			xqspi->rxbuf += 4;
		}
		xqspi->rx_bytes -= 4;
		len -= 4;
	}

	if (len && len < 4 && count < rxcount)
		zynq_qspi_rxfifo_op(xqspi, len);
}

/**
 * zynq_qspi_irq - Interrupt service routine of the QSPI controller
 * @irq:	IRQ number
 * @dev_id:	Pointer to the xqspi structure
 *
 * This function handles TX empty only.
 * On TX empty interrupt this function reads the received data from RX FIFO and
 * fills the TX FIFO if there is any data remaining to be transferred.
 *
 * Return:	IRQ_HANDLED when interrupt is handled; IRQ_NONE otherwise.
 */
static irqreturn_t zynq_qspi_irq(int irq, void *dev_id)
{
	u32 intr_status;
	bool txempty;
	struct zynq_qspi *xqspi = (struct zynq_qspi *)dev_id;

	intr_status = zynq_qspi_read(xqspi, ZYNQ_QSPI_STATUS_OFFSET);
	zynq_qspi_write(xqspi, ZYNQ_QSPI_STATUS_OFFSET, intr_status);

	if ((intr_status & ZYNQ_QSPI_IXR_TXNFULL_MASK) ||
	    (intr_status & ZYNQ_QSPI_IXR_RXNEMTY_MASK)) {
		/*
		 * This bit is set when Tx FIFO has < THRESHOLD entries.
		 * We have the THRESHOLD value set to 1,
		 * so this bit indicates Tx FIFO is empty.
		 */
		txempty = !!(intr_status & ZYNQ_QSPI_IXR_TXNFULL_MASK);
		/* Read out the data from the RX FIFO */
		zynq_qspi_read_op(xqspi, ZYNQ_QSPI_RX_THRESHOLD);
		if (xqspi->tx_bytes) {
			/* There is more data to send */
			zynq_qspi_write_op(xqspi, ZYNQ_QSPI_RX_THRESHOLD,
					   txempty);
		} else {
			/*
			 * If transfer and receive is completed then only send
			 * complete signal.
			 */
			if (!xqspi->rx_bytes) {
				zynq_qspi_write(xqspi,
						ZYNQ_QSPI_IDIS_OFFSET,
						ZYNQ_QSPI_IXR_RXTX_MASK);
				complete(&xqspi->data_completion);
			}
		}
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

/**
 * zynq_qspi_exec_mem_op() - Initiates the QSPI transfer
 * @mem: the SPI memory
 * @op: the memory operation to execute
 *
 * Executes a memory operation.
 *
 * This function first selects the chip and starts the memory operation.
 *
 * Return: 0 in case of success, a negative error code otherwise.
 */
static int zynq_qspi_exec_mem_op(struct spi_mem *mem,
				 const struct spi_mem_op *op)
{
	struct zynq_qspi *xqspi = spi_controller_get_devdata(mem->spi->controller);
	int err = 0, i;
	u8 addrbuf[8];
	u8 *tmpbuf;

	zynq_qspi_chipselect(mem->spi, true);
	zynq_qspi_config_op(xqspi, mem->spi, op);

	if (op->cmd.opcode) {
		reinit_completion(&xqspi->data_completion);
		xqspi->txbuf = (u8 *)&op->cmd.opcode;
		xqspi->rxbuf = NULL;
		xqspi->tx_bytes = op->cmd.nbytes;
		xqspi->rx_bytes = op->cmd.nbytes;
		zynq_qspi_write_op(xqspi, ZYNQ_QSPI_FIFO_DEPTH, true);
		zynq_qspi_write(xqspi, ZYNQ_QSPI_IEN_OFFSET,
				ZYNQ_QSPI_IXR_RXTX_MASK);
		if (!wait_for_completion_timeout(&xqspi->data_completion,
							       msecs_to_jiffies(1000)))
			err = -ETIMEDOUT;
	}

	if (op->addr.nbytes) {
		/*
		 * Stage the address in a local buffer.  Historically this loop
		 * wrote through xqspi->txbuf, which at this point still aims
		 * into the CONST struct spi_mem_op -- zynq_qspi_txfifo_op()
		 * left it at &op->cmd.opcode + cmd.nbytes.  With a 3-byte
		 * address the four bytes that follow are the opcode's high
		 * half plus padding, so the damage was invisible; with a
		 * 4-byte address the last store lands exactly on
		 * op->addr.nbytes (offsetof(struct spi_mem_op, addr) == 8,
		 * &op->cmd.opcode == 4, cmd.nbytes == 1 => 5,6,7,8) and
		 * replaces it with the address' low byte, which the two
		 * assignments below then read back.
		 */
		for (i = 0; i < op->addr.nbytes; i++) {
			addrbuf[i] = op->addr.val >>
					(8 * (op->addr.nbytes - i - 1));
		}

		reinit_completion(&xqspi->data_completion);
		xqspi->txbuf = addrbuf;
		xqspi->rxbuf = NULL;
		xqspi->tx_bytes = op->addr.nbytes;
		xqspi->rx_bytes = op->addr.nbytes;
		zynq_qspi_write_op(xqspi, ZYNQ_QSPI_FIFO_DEPTH, true);
		zynq_qspi_write(xqspi, ZYNQ_QSPI_IEN_OFFSET,
				ZYNQ_QSPI_IXR_RXTX_MASK);
		if (!wait_for_completion_timeout(&xqspi->data_completion,
							       msecs_to_jiffies(1000)))
			err = -ETIMEDOUT;
	}

	if (op->dummy.nbytes) {
		tmpbuf = kmalloc(op->dummy.nbytes, GFP_KERNEL);
		if (!tmpbuf)
			return -ENOMEM;

		memset(tmpbuf, 0xff, op->dummy.nbytes);
		reinit_completion(&xqspi->data_completion);
		xqspi->txbuf = tmpbuf;
		xqspi->rxbuf = NULL;
		xqspi->tx_bytes = op->dummy.nbytes;
		xqspi->rx_bytes = op->dummy.nbytes;
		zynq_qspi_write_op(xqspi, ZYNQ_QSPI_FIFO_DEPTH, true);
		zynq_qspi_write(xqspi, ZYNQ_QSPI_IEN_OFFSET,
				ZYNQ_QSPI_IXR_RXTX_MASK);
		if (!wait_for_completion_timeout(&xqspi->data_completion,
							       msecs_to_jiffies(1000)))
			err = -ETIMEDOUT;

		kfree(tmpbuf);
	}

	if (op->data.nbytes) {
		reinit_completion(&xqspi->data_completion);
		if (op->data.dir == SPI_MEM_DATA_OUT) {
			xqspi->txbuf = (u8 *)op->data.buf.out;
			xqspi->tx_bytes = op->data.nbytes;
			xqspi->rxbuf = NULL;
			xqspi->rx_bytes = op->data.nbytes;
		} else {
			xqspi->txbuf = NULL;
			xqspi->rxbuf = (u8 *)op->data.buf.in;
			xqspi->rx_bytes = op->data.nbytes;
			xqspi->tx_bytes = op->data.nbytes;
		}

		zynq_qspi_write_op(xqspi, ZYNQ_QSPI_FIFO_DEPTH, true);
		zynq_qspi_write(xqspi, ZYNQ_QSPI_IEN_OFFSET,
				ZYNQ_QSPI_IXR_RXTX_MASK);
		if (!wait_for_completion_timeout(&xqspi->data_completion,
							       msecs_to_jiffies(1000)))
			err = -ETIMEDOUT;
	}
	zynq_qspi_chipselect(mem->spi, false);

	return err;
}

static const struct spi_controller_mem_ops zynq_qspi_mem_ops = {
	.supports_op = zynq_qspi_supports_op,
	.exec_op = zynq_qspi_exec_mem_op,
};

static const struct spi_controller_mem_caps zynq_qspi_mem_caps = {
	.per_op_freq = true,
};

/**
 * zynq_qspi_probe - Probe method for the QSPI driver
 * @pdev:	Pointer to the platform_device structure
 *
 * This function initializes the driver data structures and the hardware.
 *
 * Return:	0 on success and error value on failure
 */
static int zynq_qspi_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct spi_controller *ctlr;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct zynq_qspi *xqspi;
	u32 num_cs;

	ctlr = spi_alloc_host(&pdev->dev, sizeof(*xqspi));
	if (!ctlr)
		return -ENOMEM;

	xqspi = spi_controller_get_devdata(ctlr);
	xqspi->dev = dev;
	platform_set_drvdata(pdev, ctlr);
	xqspi->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(xqspi->regs)) {
		ret = PTR_ERR(xqspi->regs);
		goto remove_ctlr;
	}

	xqspi->pclk = devm_clk_get_enabled(&pdev->dev, "pclk");
	if (IS_ERR(xqspi->pclk)) {
		dev_err(&pdev->dev, "pclk clock not found.\n");
		ret = PTR_ERR(xqspi->pclk);
		goto remove_ctlr;
	}

	init_completion(&xqspi->data_completion);
	mutex_init(&xqspi->rawop_lock);
	xqspi->ctlr = ctlr;

	xqspi->refclk = devm_clk_get_enabled(&pdev->dev, "ref_clk");
	if (IS_ERR(xqspi->refclk)) {
		dev_err(&pdev->dev, "ref_clk clock not found.\n");
		ret = PTR_ERR(xqspi->refclk);
		goto remove_ctlr;
	}

	xqspi->irq = platform_get_irq(pdev, 0);
	if (xqspi->irq < 0) {
		ret = xqspi->irq;
		goto remove_ctlr;
	}
	ret = devm_request_irq(&pdev->dev, xqspi->irq, zynq_qspi_irq,
			       0, pdev->name, xqspi);
	if (ret != 0) {
		ret = -ENXIO;
		dev_err(&pdev->dev, "request_irq failed\n");
		goto remove_ctlr;
	}

	ret = of_property_read_u32(np, "num-cs",
				   &num_cs);
	if (ret < 0) {
		ctlr->num_chipselect = 1;
	} else if (num_cs > ZYNQ_QSPI_MAX_NUM_CS) {
		ret = -EINVAL;
		dev_err(&pdev->dev, "only 2 chip selects are available\n");
		goto remove_ctlr;
	} else {
		ctlr->num_chipselect = num_cs;
	}

	ctlr->mode_bits =  SPI_RX_DUAL | SPI_RX_QUAD |
			    SPI_TX_DUAL | SPI_TX_QUAD;
	ctlr->mem_ops = &zynq_qspi_mem_ops;
	ctlr->mem_caps = &zynq_qspi_mem_caps;
	ctlr->setup = zynq_qspi_setup_op;
	ctlr->max_speed_hz = clk_get_rate(xqspi->refclk) / 2;
	ctlr->dev.of_node = np;

	/* QSPI controller initializations */
	zynq_qspi_init_hw(xqspi, ctlr->num_chipselect);

	ret = spi_register_controller(ctlr);
	if (ret) {
		dev_err(&pdev->dev, "failed to register controller\n");
		goto remove_ctlr;
	}

	/*
	 * RX framing health counters.  Both should stay at 0; a non-zero
	 * value means this driver's byte accounting lost sync with the RX
	 * FIFO and any data it returned for that op is suspect.
	 */
	xqspi->dbgdir = debugfs_create_dir(dev_name(&pdev->dev), NULL);
	debugfs_create_u32("rx_starved", 0444, xqspi->dbgdir,
			   &xqspi->rx_starved);
	debugfs_create_u32("rx_underflow", 0444, xqspi->dbgdir,
			   &xqspi->rx_underflow);
	debugfs_create_file("rawop", 0600, xqspi->dbgdir, xqspi,
			    &zynq_qspi_rawop_fops);

	return ret;

remove_ctlr:
	spi_controller_put(ctlr);

	return ret;
}

/**
 * zynq_qspi_remove - Remove method for the QSPI driver
 * @pdev:	Pointer to the platform_device structure
 *
 * This function is called if a device is physically removed from the system or
 * if the driver module is being unloaded. It frees all resources allocated to
 * the device.
 *
 * Return:	0 on success and error value on failure
 */
static void zynq_qspi_remove(struct platform_device *pdev)
{
	struct spi_controller *ctlr = platform_get_drvdata(pdev);
	struct zynq_qspi *xqspi = spi_controller_get_devdata(ctlr);

	debugfs_remove_recursive(xqspi->dbgdir);

	spi_controller_get(ctlr);

	spi_unregister_controller(ctlr);

	zynq_qspi_write(xqspi, ZYNQ_QSPI_ENABLE_OFFSET, 0);

	spi_controller_put(ctlr);
}

static const struct of_device_id zynq_qspi_of_match[] = {
	{ .compatible = "xlnx,zynq-qspi-1.0", },
	{ /* end of table */ }
};

MODULE_DEVICE_TABLE(of, zynq_qspi_of_match);

/*
 * zynq_qspi_driver - This structure defines the QSPI platform driver
 */
static struct platform_driver zynq_qspi_driver = {
	.probe = zynq_qspi_probe,
	.remove = zynq_qspi_remove,
	.driver = {
		.name = "zynq-qspi",
		.of_match_table = zynq_qspi_of_match,
	},
};

module_platform_driver(zynq_qspi_driver);

MODULE_AUTHOR("Xilinx, Inc.");
MODULE_DESCRIPTION("Xilinx Zynq QSPI driver");
MODULE_LICENSE("GPL");
