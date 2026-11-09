// SPDX-License-Identifier: GPL-2.0+
/*
 * Amlogic Meson CSI-2 Capture Bridge
 *
 * Provides V4L2 video capture device for the Meson MIPI CSI-2 pipeline.
 * Connects to meson-csi2 subdev via async notifier, creating the
 * v4l2_device that enables cascading subdev registration (CSI-2 -> sensor).
 *
 * Data path: PHY → DWC Host → Adapter (DIR_MODE) → VDIN → Canvas DMA → DDR
 *
 * The adapter routes CSI-2 data through the DIR_MODE (direct to ISP/VDIN)
 * path. VDIN0 performs the actual DMA to system memory using the Amlogic
 * canvas/MIF interface.
 *
 * On SM1, the adapter supports DIR_MODE windowing registers but not DDR_MODE
 * registers (DDR_START_PIX, VC_MODE etc. are not present). Frame data exits
 * through VDIN's own DMA engine, not the adapter's DDR path.
 *
 * Copyright (C) 2024 Da Xue <da@libre.computer>
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/timer.h>
#include <linux/workqueue.h>

#include <linux/soc/amlogic/meson-canvas.h>

#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/videobuf2-dma-contig.h>

/* -------------------------------------------------------------------
 * CSI Adapter registers
 *
 * The adapter has multiple sub-blocks:
 *   Frontend (FRONTEND_IO): base + 0x000
 *   Reader/Pixel/Align:     base + 0x800
 * ------------------------------------------------------------------- */

/* Frontend registers (byte offsets from adapter base) */
#define ADAP_CLK_RESET			0x00
#define ADAP_GEN_CTRL0			0x04
#define ADAP_GEN_CTRL1			0x08
#define ADAP_X_START_END_ISP		0x0C
#define ADAP_Y_START_END_ISP		0x10
#define ADAP_INTERRUPT_CTRL_STAT	0x50
#define ADAP_VC_MODE			0x1C

#define ADAP_GEN_STAT0			0x80
#define ADAP_ERR_STAT0			0x84
#define ADAP_PIC_SIZE_STAT		0x88
#define ADAP_STAT_TYPE_RCVD_L		0xC0
#define ADAP_STAT_TYPE_RCVD_H		0xC4

/*
 * Reader/Pixel/Alignment sub-block registers
 * Offset 0x800 from frontend base (BSP: RD_BASE - FRONTEND_BASE)
 */
#define ADAP_RPA_OFF			0x800
#define ADAP_DDR_RD0_CNTL0		(ADAP_RPA_OFF + 0x00)
#define ADAP_DDR_RD0_CNTL1		(ADAP_RPA_OFF + 0x04)
#define ADAP_PIXEL0_CNTL0		(ADAP_RPA_OFF + 0x80)
#define ADAP_PIXEL0_CNTL1		(ADAP_RPA_OFF + 0x84)
#define ADAP_ALIG_CNTL0		(ADAP_RPA_OFF + 0xC0)
#define ADAP_ALIG_CNTL1		(ADAP_RPA_OFF + 0xC4)
#define ADAP_ALIG_CNTL2		(ADAP_RPA_OFF + 0xC8)
#define ADAP_ALIG_CNTL6		(ADAP_RPA_OFF + 0xD8)
#define ADAP_ALIG_CNTL7		(ADAP_RPA_OFF + 0xDC)
#define ADAP_ALIG_CNTL8		(ADAP_RPA_OFF + 0xE0)
#define ADAP_OTHER_CNTL0		(ADAP_RPA_OFF + 0x100)

/* -------------------------------------------------------------------
 * VDIN0 registers (byte offsets from VDIN0 base 0xFF804800)
 *
 * VDIN0 is the DMA engine that writes CSI frame data to memory.
 * Uses the Amlogic canvas/MIF (Memory Interface) system.
 * ------------------------------------------------------------------- */
#define VPU_PHYS_BASE			0xFF900000
#define VDIN0_PHYS_BASE			(VPU_PHYS_BASE + 0x4800)
#define VDIN0_MAP_SIZE			0x400

/* Register offsets from VDIN0 base */
#define VDIN_COM_CTRL0			0x08
#define VDIN_COM_STATUS0		0x14
#define VDIN_COM_STATUS1		0x18
#define VDIN_ASFIFO_CTRL0		0x20
#define VDIN_WIDTHM1I_WIDTHM1O		0x28
#define VDIN_LFIFO_CTRL			0x68
#define VDIN_INTF_WIDTHM1		0x70
#define VDIN_WR_CTRL2			0x7C
#define VDIN_WR_CTRL			0x80
#define VDIN_WR_H_START_END		0x84
#define VDIN_WR_V_START_END		0x88

/* ASFIFO_CTRL3 at VPU offset 0x49BC = VDIN0_BASE + 0x1BC */
#define VDIN_ASFIFO_CTRL3		0x1BC

/* WR_CTRL canvas ID field (bits [7:0]) */
#define WR_CANVAS_BIT			0
#define WR_CANVAS_WID			8

/* VDIN internal clock gating */
#define VDIN_COM_GCLK_CTRL		0x6C	/* word 0x121B */
#define VDIN_COM_GCLK_CTRL2		0x1C0	/* word 0x1270 */

/* VDIN_MISC_CTRL at VPU word 0x2782 */
#define VPU_VDIN_MISC_PHYS		(VPU_PHYS_BASE + (0x2782 << 2))
#define VDIN0_MIF_ENABLE_BIT		19
#define VDIN0_OUT_MIF_BIT		20
#define VDIN0_OUT_AFBCE_BIT		21
#define VDIN0_MIF_RST_BIT		3

/* VPU_MISC_CTRL at VPU word 0x2740 — CSI2 input routing */
#define VPU_MISC_CTRL_PHYS		(VPU_PHYS_BASE + (0x2740 << 2))
#define ISP_IN_SEL_BIT			1
#define ISP_IN_SEL_WID			2
#define ISP_IN_SEL_MIPI_CSI2		2

/* COM_CTRL0 bits */
#define VDIN_SEL_BIT			0
#define VDIN_SEL_WID			4
#define VDIN_MUX_MIPI			8

/* ASFIFO_CTRL3 bits */
#define VDI7_ASFIFO_CTRL_BIT		8
#define VDI_ASFIFO_CTRL_WID		8

/* WR_CTRL bits */
#define WR_REQ_EN_BIT			8
#define WR_REQ_URGENT_BIT		9
#define WR_FMT_BIT			12
#define HCONV_MODE_BIT			30
#define VCONV_MODE_BIT			16
#define SWAP_CBCR_BIT			18

/* Write MIF formats */
#define MIF_FMT_YUV422			0
#define MIF_FMT_YUV444			1

/* Adapter interrupt bits */
#define ADAP_INT_WR_DONE_EN		BIT(1)
#define ADAP_INT_WR_DONE		BIT(2)

/* VDIN frame completion (COM_STATUS0 bit 2, clear via WR_CTRL bit 21) */
#define DIRECT_DONE_STATUS_BIT		2
#define DIRECT_DONE_CLR_BIT		21

#define MESON_CSI_CAP_NAME		"meson-csi-capture"
#define MESON_CSI_CAP_BUF_MIN		3

/* Frame completion polling interval */
#define FRAME_POLL_MS			5

struct meson_csi_cap_fmt {
	u32	fourcc;
	u32	mbus_code;
	u8	bpp;
};

static const struct meson_csi_cap_fmt cap_formats[] = {
	{
		.fourcc    = V4L2_PIX_FMT_SBGGR8,
		.mbus_code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.bpp       = 8,
	},
	{
		.fourcc    = V4L2_PIX_FMT_SGBRG8,
		.mbus_code = MEDIA_BUS_FMT_SGBRG8_1X8,
		.bpp       = 8,
	},
	{
		.fourcc    = V4L2_PIX_FMT_SGRBG8,
		.mbus_code = MEDIA_BUS_FMT_SGRBG8_1X8,
		.bpp       = 8,
	},
	{
		.fourcc    = V4L2_PIX_FMT_SRGGB8,
		.mbus_code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.bpp       = 8,
	},
	{
		.fourcc    = V4L2_PIX_FMT_SBGGR10P,
		.mbus_code = MEDIA_BUS_FMT_SBGGR10_1X10,
		.bpp       = 10,
	},
	{
		.fourcc    = V4L2_PIX_FMT_SGBRG10P,
		.mbus_code = MEDIA_BUS_FMT_SGBRG10_1X10,
		.bpp       = 10,
	},
	{
		.fourcc    = V4L2_PIX_FMT_SGRBG10P,
		.mbus_code = MEDIA_BUS_FMT_SGRBG10_1X10,
		.bpp       = 10,
	},
	{
		.fourcc    = V4L2_PIX_FMT_SRGGB10P,
		.mbus_code = MEDIA_BUS_FMT_SRGGB10_1X10,
		.bpp       = 10,
	},
	{
		.fourcc    = V4L2_PIX_FMT_YUYV,
		.mbus_code = MEDIA_BUS_FMT_YUYV8_1X16,
		.bpp       = 16,
	},
	{
		.fourcc    = V4L2_PIX_FMT_UYVY,
		.mbus_code = MEDIA_BUS_FMT_UYVY8_1X16,
		.bpp       = 16,
	},
};

struct meson_csi_cap_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
	dma_addr_t paddr;
};

struct meson_csi_cap {
	struct device			*dev;
	void __iomem			*adap_base;
	void __iomem			*vdin_base;
	void __iomem			*vdin_misc;
	void __iomem			*vpu_misc;
	struct meson_canvas		*canvas;
	u8				canvas_id;
	struct clk			*clk_gate;
	struct clk			*clk_isp;
	struct clk			*clk_adapt;

	struct v4l2_device		v4l2_dev;
	struct media_device		media_dev;
	struct video_device		vdev;
	struct vb2_queue		queue;
	struct v4l2_async_notifier	notifier;

	struct v4l2_subdev		*csi2_sd;
	struct media_pad		pad;

	struct mutex			lock;
	spinlock_t			qlock;

	struct v4l2_pix_format		fmt;
	const struct meson_csi_cap_fmt	*cap_fmt;

	struct list_head		buf_list;
	struct meson_csi_cap_buffer	*cur_buf;
	unsigned int			sequence;
	bool				streaming;

	struct delayed_work		frame_work;
};

static const struct meson_csi_cap_fmt *find_fmt_by_fourcc(u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(cap_formats); i++)
		if (cap_formats[i].fourcc == fourcc)
			return &cap_formats[i];
	return NULL;
}

static const struct meson_csi_cap_fmt *find_fmt_by_mbus(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(cap_formats); i++)
		if (cap_formats[i].mbus_code == code)
			return &cap_formats[i];
	return NULL;
}

static inline u32 cap_bytesperline(const struct v4l2_pix_format *f,
				   const struct meson_csi_cap_fmt *fmt)
{
	/*
	 * VDIN outputs YUV422 (2 bytes/pixel) regardless of input format.
	 * For raw Bayer, the actual sensor data is in the Y component.
	 */
	return f->width * 2;
}

/* -------------------------------------------------------------------
 * VDIN0 register helpers
 * ------------------------------------------------------------------- */

static inline u32 vdin_read(struct meson_csi_cap *cap, u32 reg)
{
	return readl(cap->vdin_base + reg);
}

static inline void vdin_write(struct meson_csi_cap *cap, u32 reg, u32 val)
{
	writel(val, cap->vdin_base + reg);
}

static inline void vdin_write_bits(struct meson_csi_cap *cap, u32 reg,
				   u32 val, u32 start, u32 len)
{
	u32 tmp = vdin_read(cap, reg);
	u32 mask = ((1 << len) - 1) << start;

	tmp &= ~mask;
	tmp |= (val << start) & mask;
	vdin_write(cap, reg, tmp);
}

/* -------------------------------------------------------------------
 * CSI Adapter configuration (BSP-compatible, no DDR mode)
 * ------------------------------------------------------------------- */

static void adap_init(struct meson_csi_cap *cap)
{
	u32 width = cap->fmt.width;
	u32 height = cap->fmt.height;

	if (!cap->adap_base)
		return;

	/*
	 * BSP init_am_mipi_csi2_adapter() sequence (mipi_hw.c:200-238):
	 * 1. Release reset (CLK_RESET=0)
	 * 2. Configure GEN_CTRL0, windows, interrupts
	 * 3. Enable clock LAST (CLK_RESET=0x2, bit[1]=clock enable)
	 *
	 * BSP uses CLK_RESET=0x2 (only bit[1]), NOT 0x6 (bits[2:1]).
	 * The ISP-path code uses 0x6 but the TVIN/amvdec_csi path uses 0x2.
	 */
	writel(0x0, cap->adap_base + ADAP_CLK_RESET);
	udelay(100);

	/*
	 * GEN_CTRL0: BSP amvdec_csi init_am_mipi_csi2_adapter() value.
	 *   bit[21]     = 1 (CSI2_CFG_422TO444_MODE)
	 *   bits[20:16] = 0x1F (color format mapping, all types)
	 *   bit[11]     = 1 (CSI2_CFG_BUFFER_PIC_SIZE)
	 *   bit[4]      = 0 (CSI2_CFG_ALL_TO_MEM=0, direct path)
	 *   bits[3:0]   = 0xF (enable)
	 *
	 * Full BSP value: 0x003F080F (vs our previous 0x001F080F)
	 */
	writel(0x003F080F, cap->adap_base + ADAP_GEN_CTRL0);

	/*
	 * GEN_CTRL1: BSP g12b adapter writes 0x1B.
	 * SM1 adapter only has bits[13:0] writable.
	 */
	writel(0x0000001B, cap->adap_base + ADAP_GEN_CTRL1);

	/* ISP window: actual frame dimensions */
	writel(((width - 1) << 16) | 0, cap->adap_base + ADAP_X_START_END_ISP);
	writel(((height - 1) << 16) | 0, cap->adap_base + ADAP_Y_START_END_ISP);

	/* Enable adapter interrupt (BSP writes (1 << 1)) */
	writel(0x2, cap->adap_base + ADAP_INTERRUPT_CTRL_STAT);

	/* Enable module clock LAST (BSP sequence) */
	writel(0x2, cap->adap_base + ADAP_CLK_RESET);

	dev_info(cap->dev, "adap_init: GEN_CTRL0=0x%08x GEN_CTRL1=0x%08x CLK_RST=0x%08x INT=0x%08x\n",
		 readl(cap->adap_base + ADAP_GEN_CTRL0),
		 readl(cap->adap_base + ADAP_GEN_CTRL1),
		 readl(cap->adap_base + ADAP_CLK_RESET),
		 readl(cap->adap_base + ADAP_INTERRUPT_CTRL_STAT));
}

static void adap_stop(struct meson_csi_cap *cap)
{
	if (!cap->adap_base)
		return;

	/* Assert reset */
	writel(0x1, cap->adap_base + ADAP_CLK_RESET);
}

/* -------------------------------------------------------------------
 * VDIN0 DMA engine configuration
 * ------------------------------------------------------------------- */

static void vdin_start(struct meson_csi_cap *cap)
{
	u32 width = cap->fmt.width;
	u32 height = cap->fmt.height;
	u32 wr_ctrl;

	if (!cap->vdin_base)
		return;

	/*
	 * Route CSI2 adapter output through VPU fabric.
	 * VPU_MISC_CTRL bits [2:1] = 10b selects MIPI CSI2 as input.
	 */
	if (cap->vpu_misc) {
		u32 val = readl(cap->vpu_misc);

		val &= ~(3 << ISP_IN_SEL_BIT);
		val |= (ISP_IN_SEL_MIPI_CSI2 << ISP_IN_SEL_BIT);
		writel(val, cap->vpu_misc);
		dev_info(cap->dev, "VPU_MISC_CTRL=0x%08x (CSI2 routed)\n",
			 readl(cap->vpu_misc));
	}

	/*
	 * Enable VDIN0 MIF (Memory Interface) via VDIN_MISC_CTRL.
	 * BSP: set MIF_ENABLE (bit 19) and OUT_MIF (bit 20), clear AFBCE (bit 21).
	 */
	if (cap->vdin_misc) {
		u32 misc = readl(cap->vdin_misc);

		/* Reset MIF first */
		misc |= BIT(VDIN0_MIF_RST_BIT);
		writel(misc, cap->vdin_misc);
		udelay(1);
		misc &= ~BIT(VDIN0_MIF_RST_BIT);
		writel(misc, cap->vdin_misc);

		/* Enable MIF and output to MIF (not AFBCE) */
		misc |= BIT(VDIN0_MIF_ENABLE_BIT);
		misc |= BIT(VDIN0_OUT_MIF_BIT);
		misc &= ~BIT(VDIN0_OUT_AFBCE_BIT);
		writel(misc, cap->vdin_misc);
	}

	/*
	 * Enable all VDIN internal clocks (0 = all auto/enabled).
	 * BSP vdin_clk_on_off() writes both GCLK_CTRL and GCLK_CTRL2.
	 * Default 0x5555... = all gated; must write 0 to un-gate.
	 */
	vdin_write(cap, VDIN_COM_GCLK_CTRL, 0x0);
	vdin_write(cap, VDIN_COM_GCLK_CTRL2, 0x0);

	/*
	 * COM_CTRL0: Enable data input, select MIPI, set component mapping.
	 * BSP vdin_set_top() + vdin_set_color_map():
	 *   bit[4]     = 1 (COMMON_DATA_IN_EN)
	 *   bit[23]    = 1 (FRAME_SOFT_RST_EN — auto-reset on vsync)
	 *   bits[3:0]  = 8 (VDIN_MUX_MIPI)
	 *   bits[8:6]  = 0 (COMP0=Y)
	 *   bits[10:8] = 1 (COMP1=Cb/B)
	 *   bits[12:10]= 2 (COMP2=Cr/R)
	 */
	{
		u32 ctrl0 = vdin_read(cap, VDIN_COM_CTRL0);

		ctrl0 &= ~0x1FFF;	/* clear mux + comp mapping */
		ctrl0 |= VDIN_MUX_MIPI;		/* bits[3:0] = 8 */
		ctrl0 |= BIT(4);			/* enable data input */
		ctrl0 |= (0 << 6);			/* COMP0 = Y */
		ctrl0 |= (1 << 8);			/* COMP1 = Cb */
		ctrl0 |= (2 << 10);			/* COMP2 = Cr */
		ctrl0 |= BIT(23);			/* FRAME_SOFT_RST_EN */
		vdin_write(cap, VDIN_COM_CTRL0, ctrl0);
	}

	/* Configure ASFIFO for MIPI input (VDI7) */
	vdin_write_bits(cap, VDIN_ASFIFO_CTRL3,
			0xe0, VDI7_ASFIFO_CTRL_BIT, VDI_ASFIFO_CTRL_WID);

	/*
	 * Input/output width (BSP: VDIN_WIDTHM1I_WIDTHM1O).
	 * bits[27:16] = input width - 1
	 * bits[11:0]  = output width - 1
	 */
	vdin_write(cap, VDIN_WIDTHM1I_WIDTHM1O,
		   ((width - 1) << 16) | (width - 1));

	/* Interface width */
	vdin_write(cap, VDIN_INTF_WIDTHM1, width - 1);

	/* Line FIFO control */
	vdin_write(cap, VDIN_LFIFO_CTRL, width - 1);

	/* Write window */
	vdin_write(cap, VDIN_WR_H_START_END, width - 1);
	vdin_write(cap, VDIN_WR_V_START_END, height - 1);

	/*
	 * Write control: YUV422 format, enable write request.
	 * VDIN processes CSI data and outputs in YUV422 format (2 bytes/pixel).
	 * For raw Bayer, sensor data appears in the Y component.
	 */
	wr_ctrl = 0;
	wr_ctrl |= (2 << HCONV_MODE_BIT);	/* hconv_mode = 2 (average) */
	wr_ctrl |= (MIF_FMT_YUV422 << WR_FMT_BIT);
	wr_ctrl |= (1 << WR_REQ_URGENT_BIT);
	wr_ctrl |= (1 << WR_REQ_EN_BIT);
	vdin_write(cap, VDIN_WR_CTRL, wr_ctrl);
}

static void vdin_set_buf(struct meson_csi_cap *cap, dma_addr_t addr)
{
	if (!cap->vdin_base || !cap->canvas)
		return;

	/*
	 * SM1 uses canvas-based addressing: configure the canvas LUT entry
	 * with the buffer's physical address, stride, and height.
	 * The canvas ID is written to VDIN_WR_CTRL bits [7:0].
	 */
	meson_canvas_config(cap->canvas, cap->canvas_id,
			    addr, cap->fmt.bytesperline,
			    cap->fmt.height,
			    MESON_CANVAS_WRAP_NONE,
			    MESON_CANVAS_BLKMODE_LINEAR,
			    MESON_CANVAS_ENDIAN_SWAP64);

	/* Set canvas ID in WR_CTRL */
	vdin_write_bits(cap, VDIN_WR_CTRL, cap->canvas_id,
			WR_CANVAS_BIT, WR_CANVAS_WID);

	dev_info(cap->dev, "vdin_set_buf: addr=0x%llx canvas=%u stride=%u\n",
		 (u64)addr, cap->canvas_id, cap->fmt.bytesperline);
}

static void vdin_stop(struct meson_csi_cap *cap)
{
	if (!cap->vdin_base)
		return;

	/* Disable write request */
	vdin_write_bits(cap, VDIN_WR_CTRL, 0, WR_REQ_EN_BIT, 1);

	/* Deselect input source */
	vdin_write_bits(cap, VDIN_COM_CTRL0, 0, VDIN_SEL_BIT, VDIN_SEL_WID);

	/* Disable VDIN0 MIF */
	if (cap->vdin_misc) {
		u32 misc = readl(cap->vdin_misc);

		misc &= ~BIT(VDIN0_MIF_ENABLE_BIT);
		misc &= ~BIT(VDIN0_OUT_MIF_BIT);
		writel(misc, cap->vdin_misc);
	}

	/* Disable CSI2 routing in VPU */
	if (cap->vpu_misc) {
		u32 val = readl(cap->vpu_misc);

		val &= ~(3 << ISP_IN_SEL_BIT);
		writel(val, cap->vpu_misc);
	}
}

/* -------------------------------------------------------------------
 * Frame completion polling
 *
 * VDIN0 write-done IRQ requires integration with the VDIN IRQ
 * infrastructure. For initial bring-up, poll the adapter status
 * to detect frame boundaries.
 * ------------------------------------------------------------------- */

static void meson_csi_cap_frame_work(struct work_struct *work)
{
	struct meson_csi_cap *cap =
		container_of(work, struct meson_csi_cap, frame_work.work);
	struct meson_csi_cap_buffer *buf;
	unsigned long flags;
	u32 stat;

	if (!cap->streaming)
		return;

	if (!cap->vdin_base)
		goto reschedule;

	stat = vdin_read(cap, VDIN_COM_STATUS0);

	/* Periodic debug: print VDIN status every ~2 seconds */
	{
		static unsigned int poll_cnt;

		if ((poll_cnt++ % 400) == 0) {
			dev_info(cap->dev, "poll: status0=0x%08x status1=0x%08x\n",
				 stat, vdin_read(cap, VDIN_COM_STATUS1));
			if (cap->adap_base)
				dev_info(cap->dev, "  adap: stat0=0x%08x err=0x%08x pic=0x%08x int=0x%08x\n",
					 readl(cap->adap_base + ADAP_GEN_STAT0),
					 readl(cap->adap_base + ADAP_ERR_STAT0),
					 readl(cap->adap_base + ADAP_PIC_SIZE_STAT),
					 readl(cap->adap_base + ADAP_INTERRUPT_CTRL_STAT));
		}
	}

	/* Check for VDIN write done (COM_STATUS0 bit 2) */
	if (stat & BIT(DIRECT_DONE_STATUS_BIT)) {
		/* Clear done status via WR_CTRL bit 21 */
		vdin_write_bits(cap, VDIN_WR_CTRL, 1,
				DIRECT_DONE_CLR_BIT, 1);
		vdin_write_bits(cap, VDIN_WR_CTRL, 0,
				DIRECT_DONE_CLR_BIT, 1);

		spin_lock_irqsave(&cap->qlock, flags);

		/* Complete current buffer */
		if (cap->cur_buf) {
			cap->cur_buf->vb.vb2_buf.timestamp = ktime_get_ns();
			cap->cur_buf->vb.sequence = cap->sequence++;
			cap->cur_buf->vb.field = V4L2_FIELD_NONE;
			vb2_buffer_done(&cap->cur_buf->vb.vb2_buf,
					VB2_BUF_STATE_DONE);
		}

		/* Queue next buffer */
		if (!list_empty(&cap->buf_list)) {
			buf = list_first_entry(&cap->buf_list,
					       struct meson_csi_cap_buffer,
					       list);
			list_del(&buf->list);
			cap->cur_buf = buf;
			vdin_set_buf(cap, buf->paddr);
		} else {
			cap->cur_buf = NULL;
		}

		spin_unlock_irqrestore(&cap->qlock, flags);
	}

reschedule:
	if (cap->streaming)
		schedule_delayed_work(&cap->frame_work,
				      msecs_to_jiffies(FRAME_POLL_MS));
}

/* -------------------------------------------------------------------
 * videobuf2 operations
 * ------------------------------------------------------------------- */

static int meson_csi_cap_queue_setup(struct vb2_queue *q,
				     unsigned int *nbuffers,
				     unsigned int *nplanes,
				     unsigned int sizes[],
				     struct device *alloc_devs[])
{
	struct meson_csi_cap *cap = vb2_get_drv_priv(q);
	unsigned int size = cap->fmt.sizeimage;

	if (*nplanes) {
		if (sizes[0] < size)
			return -EINVAL;
		return 0;
	}

	*nplanes = 1;
	sizes[0] = size;

	return 0;
}

static int meson_csi_cap_buf_prepare(struct vb2_buffer *vb)
{
	struct meson_csi_cap *cap = vb2_get_drv_priv(vb->vb2_queue);

	if (vb2_plane_size(vb, 0) < cap->fmt.sizeimage)
		return -EINVAL;

	vb2_set_plane_payload(vb, 0, cap->fmt.sizeimage);

	return 0;
}

static void meson_csi_cap_buf_queue(struct vb2_buffer *vb)
{
	struct meson_csi_cap *cap = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct meson_csi_cap_buffer *buf =
		container_of(vbuf, struct meson_csi_cap_buffer, vb);
	unsigned long flags;

	buf->paddr = vb2_dma_contig_plane_dma_addr(vb, 0);

	spin_lock_irqsave(&cap->qlock, flags);
	list_add_tail(&buf->list, &cap->buf_list);
	spin_unlock_irqrestore(&cap->qlock, flags);
}

static int meson_csi_cap_start_streaming(struct vb2_queue *q,
					 unsigned int count)
{
	struct meson_csi_cap *cap = vb2_get_drv_priv(q);
	struct meson_csi_cap_buffer *buf;
	unsigned long flags;
	int ret;

	cap->sequence = 0;

	/* Configure adapter (CSI data routing, no DDR) */
	adap_init(cap);

	/* Configure VDIN0 DMA engine */
	vdin_start(cap);

	/* Set up first buffer for VDIN DMA */
	spin_lock_irqsave(&cap->qlock, flags);
	if (!list_empty(&cap->buf_list)) {
		buf = list_first_entry(&cap->buf_list,
				       struct meson_csi_cap_buffer, list);
		list_del(&buf->list);
		cap->cur_buf = buf;
		vdin_set_buf(cap, buf->paddr);
	}
	spin_unlock_irqrestore(&cap->qlock, flags);

	/* Start CSI-2 pipeline (PHY + sensor) */
	ret = v4l2_subdev_call(cap->csi2_sd, video, s_stream, 1);
	if (ret) {
		dev_err(cap->dev, "Failed to start CSI-2 stream: %d\n", ret);
		goto err_stop;
	}

	cap->streaming = true;

	/*
	 * GEN_CTRL0 already has bits[3:0]=0xF from adap_init().
	 * BSP amvdec_csi sets them during init, not as a separate start step.
	 * Read back to confirm.
	 */
	if (cap->adap_base) {
		dev_info(cap->dev, "adapter active: GEN_CTRL0=0x%08x\n",
			 readl(cap->adap_base + ADAP_GEN_CTRL0));
	}

	/* Debug: dump register state */
	dev_info(cap->dev, "stream on: WR_CTRL=0x%08x canvas=%u\n",
		 vdin_read(cap, VDIN_WR_CTRL), cap->canvas_id);
	dev_info(cap->dev, "  COM_CTRL0=0x%08x ASFIFO3=0x%08x MISC=0x%08x\n",
		 vdin_read(cap, VDIN_COM_CTRL0),
		 vdin_read(cap, VDIN_ASFIFO_CTRL3),
		 cap->vdin_misc ? readl(cap->vdin_misc) : 0);
	if (cap->adap_base) {
		dev_info(cap->dev, "  ADAP: GEN_CTRL0=0x%08x CTRL1=0x%08x STAT0=0x%08x ERR=0x%08x PIC_SIZE=0x%08x\n",
			 readl(cap->adap_base + ADAP_GEN_CTRL0),
			 readl(cap->adap_base + ADAP_GEN_CTRL1),
			 readl(cap->adap_base + ADAP_GEN_STAT0),
			 readl(cap->adap_base + ADAP_ERR_STAT0),
			 readl(cap->adap_base + ADAP_PIC_SIZE_STAT));
		dev_info(cap->dev, "  ADAP: TYPE_RCVD_L=0x%08x TYPE_RCVD_H=0x%08x VC_MODE=0x%08x\n",
			 readl(cap->adap_base + ADAP_STAT_TYPE_RCVD_L),
			 readl(cap->adap_base + ADAP_STAT_TYPE_RCVD_H),
			 readl(cap->adap_base + ADAP_VC_MODE));
		/* Dump short packet stats */
		dev_info(cap->dev, "  ADAP SHORT: 0xa0=0x%08x 0xa4=0x%08x 0xa8=0x%08x 0xac=0x%08x\n",
			 readl(cap->adap_base + 0xa0),
			 readl(cap->adap_base + 0xa4),
			 readl(cap->adap_base + 0xa8),
			 readl(cap->adap_base + 0xac));
	}

	/*
	 * Check if DMA buffer content changed — data may flow through
	 * VDIN even if DIRECT_DONE status never fires.
	 */
	if (cap->cur_buf) {
		void *vaddr = vb2_plane_vaddr(&cap->cur_buf->vb.vb2_buf, 0);

		if (vaddr) {
			u32 pre[4];

			memcpy(pre, vaddr, sizeof(pre));
			msleep(500);
			dev_info(cap->dev,
				 "DMA buf check: pre=[%08x %08x %08x %08x] post=[%08x %08x %08x %08x]\n",
				 pre[0], pre[1], pre[2], pre[3],
				 ((u32 *)vaddr)[0], ((u32 *)vaddr)[1],
				 ((u32 *)vaddr)[2], ((u32 *)vaddr)[3]);
		}
	}

	/* Start frame completion polling */
	schedule_delayed_work(&cap->frame_work,
			      msecs_to_jiffies(FRAME_POLL_MS));

	return 0;

err_stop:
	vdin_stop(cap);
	adap_stop(cap);

	spin_lock_irqsave(&cap->qlock, flags);
	if (cap->cur_buf) {
		vb2_buffer_done(&cap->cur_buf->vb.vb2_buf,
				VB2_BUF_STATE_QUEUED);
		cap->cur_buf = NULL;
	}
	list_for_each_entry(buf, &cap->buf_list, list)
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_QUEUED);
	INIT_LIST_HEAD(&cap->buf_list);
	spin_unlock_irqrestore(&cap->qlock, flags);

	return ret;
}

static void meson_csi_cap_stop_streaming(struct vb2_queue *q)
{
	struct meson_csi_cap *cap = vb2_get_drv_priv(q);
	struct meson_csi_cap_buffer *buf;
	unsigned long flags;

	cap->streaming = false;

	/* Stop frame polling */
	cancel_delayed_work_sync(&cap->frame_work);

	/* Stop CSI-2 pipeline */
	v4l2_subdev_call(cap->csi2_sd, video, s_stream, 0);

	/* Stop VDIN and adapter */
	vdin_stop(cap);
	adap_stop(cap);

	/* Return all buffers */
	spin_lock_irqsave(&cap->qlock, flags);
	if (cap->cur_buf) {
		vb2_buffer_done(&cap->cur_buf->vb.vb2_buf,
				VB2_BUF_STATE_ERROR);
		cap->cur_buf = NULL;
	}
	while (!list_empty(&cap->buf_list)) {
		buf = list_first_entry(&cap->buf_list,
				       struct meson_csi_cap_buffer, list);
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}
	spin_unlock_irqrestore(&cap->qlock, flags);
}

static const struct vb2_ops meson_csi_cap_vb2_ops = {
	.queue_setup     = meson_csi_cap_queue_setup,
	.buf_prepare     = meson_csi_cap_buf_prepare,
	.buf_queue       = meson_csi_cap_buf_queue,
	.start_streaming = meson_csi_cap_start_streaming,
	.stop_streaming  = meson_csi_cap_stop_streaming,
};

/* -------------------------------------------------------------------
 * V4L2 ioctl operations
 * ------------------------------------------------------------------- */

static int meson_csi_cap_querycap(struct file *file, void *fh,
				  struct v4l2_capability *cap)
{
	strscpy(cap->driver, MESON_CSI_CAP_NAME, sizeof(cap->driver));
	strscpy(cap->card, "Meson CSI Capture", sizeof(cap->card));

	return 0;
}

static int meson_csi_cap_enum_fmt(struct file *file, void *fh,
				  struct v4l2_fmtdesc *f)
{
	if (f->index >= ARRAY_SIZE(cap_formats))
		return -EINVAL;

	f->pixelformat = cap_formats[f->index].fourcc;

	return 0;
}

static int meson_csi_cap_g_fmt(struct file *file, void *fh,
			       struct v4l2_format *f)
{
	struct meson_csi_cap *cap = video_drvdata(file);

	f->fmt.pix = cap->fmt;

	return 0;
}

static int meson_csi_cap_try_fmt(struct file *file, void *fh,
				 struct v4l2_format *f)
{
	const struct meson_csi_cap_fmt *fmt;
	struct v4l2_pix_format *pix = &f->fmt.pix;

	fmt = find_fmt_by_fourcc(pix->pixelformat);
	if (!fmt) {
		fmt = &cap_formats[0];
		pix->pixelformat = fmt->fourcc;
	}

	pix->width = clamp(pix->width, 16U, 4096U);
	pix->height = clamp(pix->height, 16U, 4096U);
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = cap_bytesperline(pix, fmt);
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->colorspace = V4L2_COLORSPACE_RAW;

	return 0;
}

static int meson_csi_cap_s_fmt(struct file *file, void *fh,
			       struct v4l2_format *f)
{
	struct meson_csi_cap *cap = video_drvdata(file);
	const struct meson_csi_cap_fmt *cfmt;
	struct v4l2_subdev_format sd_fmt = {};
	int ret;

	if (vb2_is_busy(&cap->queue))
		return -EBUSY;

	ret = meson_csi_cap_try_fmt(file, fh, f);
	if (ret)
		return ret;

	cfmt = find_fmt_by_fourcc(f->fmt.pix.pixelformat);

	/*
	 * Propagate format to the sensor subdev so it switches
	 * output mode (e.g., RAW10 vs YUV422).
	 */
	if (cap->csi2_sd && cfmt) {
		sd_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
		sd_fmt.pad = 0;
		sd_fmt.format.width = f->fmt.pix.width;
		sd_fmt.format.height = f->fmt.pix.height;
		sd_fmt.format.code = cfmt->mbus_code;
		sd_fmt.format.field = V4L2_FIELD_NONE;

		ret = v4l2_subdev_call(cap->csi2_sd, pad, set_fmt,
				       NULL, &sd_fmt);
		if (ret && ret != -ENOIOCTLCMD)
			dev_warn(cap->dev,
				 "subdev set_fmt failed: %d\n", ret);
	}

	cap->fmt = f->fmt.pix;
	cap->cap_fmt = cfmt;

	return 0;
}

static int meson_csi_cap_enum_input(struct file *file, void *fh,
				    struct v4l2_input *i)
{
	if (i->index > 0)
		return -EINVAL;

	strscpy(i->name, "CSI-2", sizeof(i->name));
	i->type = V4L2_INPUT_TYPE_CAMERA;

	return 0;
}

static int meson_csi_cap_g_input(struct file *file, void *fh,
				 unsigned int *i)
{
	*i = 0;
	return 0;
}

static int meson_csi_cap_s_input(struct file *file, void *fh,
				 unsigned int i)
{
	return i ? -EINVAL : 0;
}

static const struct v4l2_ioctl_ops meson_csi_cap_ioctl_ops = {
	.vidioc_querycap		= meson_csi_cap_querycap,
	.vidioc_enum_fmt_vid_cap	= meson_csi_cap_enum_fmt,
	.vidioc_g_fmt_vid_cap		= meson_csi_cap_g_fmt,
	.vidioc_try_fmt_vid_cap		= meson_csi_cap_try_fmt,
	.vidioc_s_fmt_vid_cap		= meson_csi_cap_s_fmt,
	.vidioc_enum_input		= meson_csi_cap_enum_input,
	.vidioc_g_input			= meson_csi_cap_g_input,
	.vidioc_s_input			= meson_csi_cap_s_input,
	.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	.vidioc_querybuf		= vb2_ioctl_querybuf,
	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
	.vidioc_create_bufs		= vb2_ioctl_create_bufs,
	.vidioc_prepare_buf		= vb2_ioctl_prepare_buf,
	.vidioc_streamon		= vb2_ioctl_streamon,
	.vidioc_streamoff		= vb2_ioctl_streamoff,
};

static const struct v4l2_file_operations meson_csi_cap_fops = {
	.owner		= THIS_MODULE,
	.open		= v4l2_fh_open,
	.release	= vb2_fop_release,
	.read		= vb2_fop_read,
	.poll		= vb2_fop_poll,
	.mmap		= vb2_fop_mmap,
	.unlocked_ioctl	= video_ioctl2,
};

/* -------------------------------------------------------------------
 * Async notifier: connects to meson-csi2 subdev
 * ------------------------------------------------------------------- */

static int meson_csi_cap_bound(struct v4l2_async_notifier *notifier,
			       struct v4l2_subdev *sd,
			       struct v4l2_async_connection *asc)
{
	struct meson_csi_cap *cap =
		container_of(notifier, struct meson_csi_cap, notifier);

	cap->csi2_sd = sd;

	dev_info(cap->dev, "Bound subdev %s\n", sd->name);

	return 0;
}

static void meson_csi_cap_unbind(struct v4l2_async_notifier *notifier,
				 struct v4l2_subdev *sd,
				 struct v4l2_async_connection *asc)
{
	struct meson_csi_cap *cap =
		container_of(notifier, struct meson_csi_cap, notifier);

	cap->csi2_sd = NULL;
}

static int meson_csi_cap_complete(struct v4l2_async_notifier *notifier)
{
	struct meson_csi_cap *cap =
		container_of(notifier, struct meson_csi_cap, notifier);
	int ret;

	/* Guard against double registration (e.g., subdev rebind) */
	if (video_is_registered(&cap->vdev)) {
		dev_info(cap->dev, "Video device already registered\n");
		return 0;
	}

	ret = video_register_device(&cap->vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		dev_err(cap->dev, "Failed to register video device: %d\n", ret);
		return ret;
	}

	/* Create media link: CSI-2 source pad -> capture sink pad */
	ret = media_create_pad_link(&cap->csi2_sd->entity, 1,
				    &cap->vdev.entity, 0,
				    MEDIA_LNK_FL_ENABLED |
				    MEDIA_LNK_FL_IMMUTABLE);
	if (ret) {
		dev_err(cap->dev, "Failed to create media link: %d\n", ret);
		video_unregister_device(&cap->vdev);
		return ret;
	}

	dev_info(cap->dev, "Registered %s as /dev/video%d\n",
		 cap->vdev.name, cap->vdev.num);

	return 0;
}

static const struct v4l2_async_notifier_operations meson_csi_cap_nf_ops = {
	.bound		= meson_csi_cap_bound,
	.unbind		= meson_csi_cap_unbind,
	.complete	= meson_csi_cap_complete,
};

/* -------------------------------------------------------------------
 * Platform driver
 * ------------------------------------------------------------------- */

static int meson_csi_cap_parse_dt(struct meson_csi_cap *cap)
{
	struct v4l2_async_connection *asc;
	struct fwnode_handle *ep;

	ep = fwnode_graph_get_endpoint_by_id(dev_fwnode(cap->dev), 0, 0, 0);
	if (!ep)
		return -ENODEV;

	v4l2_async_nf_init(&cap->notifier, &cap->v4l2_dev);

	asc = v4l2_async_nf_add_fwnode_remote(&cap->notifier, ep,
					      struct v4l2_async_connection);
	fwnode_handle_put(ep);
	if (IS_ERR(asc)) {
		v4l2_async_nf_cleanup(&cap->notifier);
		return PTR_ERR(asc);
	}

	cap->notifier.ops = &meson_csi_cap_nf_ops;

	return v4l2_async_nf_register(&cap->notifier);
}

static int meson_csi_cap_probe(struct platform_device *pdev)
{
	struct meson_csi_cap *cap;
	struct resource *res;
	int ret;

	cap = devm_kzalloc(&pdev->dev, sizeof(*cap), GFP_KERNEL);
	if (!cap)
		return -ENOMEM;

	cap->dev = &pdev->dev;
	platform_set_drvdata(pdev, cap);
	mutex_init(&cap->lock);
	spin_lock_init(&cap->qlock);
	INIT_LIST_HEAD(&cap->buf_list);
	INIT_DELAYED_WORK(&cap->frame_work, meson_csi_cap_frame_work);

	/* Adapter clocks (optional) */
	cap->clk_gate = devm_clk_get_optional(&pdev->dev, "gate");
	if (IS_ERR(cap->clk_gate))
		return dev_err_probe(&pdev->dev, PTR_ERR(cap->clk_gate),
				     "Failed to get gate clock\n");

	cap->clk_isp = devm_clk_get_optional(&pdev->dev, "isp");
	if (IS_ERR(cap->clk_isp))
		return dev_err_probe(&pdev->dev, PTR_ERR(cap->clk_isp),
				     "Failed to get isp clock\n");

	cap->clk_adapt = devm_clk_get_optional(&pdev->dev, "adapt");
	if (IS_ERR(cap->clk_adapt))
		return dev_err_probe(&pdev->dev, PTR_ERR(cap->clk_adapt),
				     "Failed to get adapt clock\n");

	/* Enable clocks before accessing registers */
	if (cap->clk_isp) {
		ret = clk_prepare_enable(cap->clk_isp);
		if (ret) {
			dev_err(&pdev->dev, "Failed to enable isp clock: %d\n", ret);
			return ret;
		}
	}

	if (cap->clk_gate) {
		ret = clk_prepare_enable(cap->clk_gate);
		if (ret) {
			dev_err(&pdev->dev, "Failed to enable gate clock: %d\n", ret);
			goto err_clk_isp;
		}
	}

	if (cap->clk_adapt) {
		ret = clk_prepare_enable(cap->clk_adapt);
		if (ret) {
			dev_err(&pdev->dev, "Failed to enable adapt clock: %d\n", ret);
			goto err_clk_gate;
		}
	}

	/*
	 * Configure adapter clock to 666MHz from fclk_div3.
	 * HHI_CSI2_ADAPT_CLK_CNTL (0xFF63C3C0):
	 *   bits [11:9] = mux (2 = fclk_div3 = 666MHz)
	 *   bits [6:0]  = divider (0 = /1)
	 *   bit 8       = gate enable
	 * The upstream clock driver only has a gate clock for this register,
	 * so the mux defaults to 0 (xtal 24MHz) which is way too slow.
	 * BSP uses 666MHz (fclk_div3) for the adapter.
	 */
	{
		void __iomem *hhi = ioremap(0xFF63C000, 0x400);

		if (hhi) {
			/* mux=2 (fclk_div3), div=0, gate=1 → 0x500 */
			writel(0x500, hhi + 0x3C0);
			dev_info(&pdev->dev,
				 "Adapter clock: HHI_CSI2_ADAPT_CLK_CNTL=0x%08x (666MHz fclk_div3)\n",
				 readl(hhi + 0x3C0));
			iounmap(hhi);
		}
	}

	/*
	 * Enable CSI power domain manually.
	 * SM1 upstream pwrc driver is missing the CSI domain entry
	 * (PWRC_SM1_CSI_ID=11). BSP: SM1_EE_PD(6, 6) in power_ee_domain.c.
	 *
	 * AO_RTI_GEN_PWR_SLEEP0 (0xFF8000E8): bit 6 = CSI power sleep
	 * AO_RTI_GEN_PWR_ISO0   (0xFF8000EC): bit 6 = CSI isolation
	 * HHI_MEM_PD_REG0       (0xFF63C100): bits [7:6] = CSI memory PD
	 *
	 * Sequence: clear sleep → clear mem PD → clear isolation
	 */
	{
		void __iomem *ao_base = ioremap(0xFF800000, 0x100);
		void __iomem *hhi_base = ioremap(0xFF63C000, 0x400);

		if (ao_base && hhi_base) {
			u32 val;

			/* Power on: clear sleep bit */
			val = readl(ao_base + 0xE8);
			val &= ~BIT(6);
			writel(val, ao_base + 0xE8);

			/* Clear memory power-down */
			val = readl(hhi_base + 0x100);
			val &= ~(BIT(7) | BIT(6));
			writel(val, hhi_base + 0x100);

			/* Remove isolation */
			val = readl(ao_base + 0xEC);
			val &= ~BIT(6);
			writel(val, ao_base + 0xEC);

			dev_info(&pdev->dev,
				 "CSI power domain enabled (SLEEP0=0x%08x MEM_PD0=0x%08x ISO0=0x%08x)\n",
				 readl(ao_base + 0xE8),
				 readl(hhi_base + 0x100),
				 readl(ao_base + 0xEC));
		}

		if (ao_base)
			iounmap(ao_base);
		if (hhi_base)
			iounmap(hhi_base);
	}

	/* CSI Adapter registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res) {
		cap->adap_base = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(cap->adap_base)) {
			dev_warn(&pdev->dev,
				 "Adapter registers not available\n");
			cap->adap_base = NULL;
		}
	}

	/*
	 * VDIN0 registers (VPU register space).
	 * Map directly since VDIN0 is shared VPU infrastructure,
	 * not described in our device tree node.
	 */
	cap->vdin_base = ioremap(VDIN0_PHYS_BASE, VDIN0_MAP_SIZE);
	if (!cap->vdin_base) {
		dev_err(&pdev->dev, "Failed to map VDIN0 registers\n");
		ret = -ENOMEM;
		goto err_clk_adapt;
	}

	/* VDIN_MISC_CTRL — controls MIF enable for VDIN0 */
	cap->vdin_misc = ioremap(VPU_VDIN_MISC_PHYS, 4);
	if (!cap->vdin_misc) {
		dev_err(&pdev->dev, "Failed to map VDIN_MISC_CTRL\n");
		ret = -ENOMEM;
		goto err_vdin_base;
	}

	/* VPU_MISC_CTRL — CSI2 input routing through VPU fabric */
	cap->vpu_misc = ioremap(VPU_MISC_CTRL_PHYS, 4);
	if (!cap->vpu_misc) {
		dev_err(&pdev->dev, "Failed to map VPU_MISC_CTRL\n");
		ret = -ENOMEM;
		goto err_vdin_misc;
	}

	dev_info(&pdev->dev, "VDIN0 mapped, VDIN_MISC_CTRL=0x%08x VPU_MISC_CTRL=0x%08x\n",
		 readl(cap->vdin_misc), readl(cap->vpu_misc));

	/* Canvas for VDIN buffer addressing */
	cap->canvas = meson_canvas_get(&pdev->dev);
	if (IS_ERR(cap->canvas)) {
		ret = dev_err_probe(&pdev->dev, PTR_ERR(cap->canvas),
				    "Failed to get canvas\n");
		goto err_vdin_misc;
	}

	ret = meson_canvas_alloc(cap->canvas, &cap->canvas_id);
	if (ret) {
		dev_err(&pdev->dev, "Failed to alloc canvas: %d\n", ret);
		goto err_vdin_misc;
	}

	dev_info(&pdev->dev, "Allocated canvas ID %u\n", cap->canvas_id);

	/* Media device */
	strscpy(cap->media_dev.model, "Meson CSI Capture",
		sizeof(cap->media_dev.model));
	cap->media_dev.dev = &pdev->dev;
	media_device_init(&cap->media_dev);

	/* V4L2 device */
	cap->v4l2_dev.mdev = &cap->media_dev;
	ret = v4l2_device_register(&pdev->dev, &cap->v4l2_dev);
	if (ret)
		goto err_vdin;

	/* VB2 queue */
	cap->queue.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	cap->queue.io_modes = VB2_MMAP | VB2_DMABUF;
	cap->queue.drv_priv = cap;
	cap->queue.ops = &meson_csi_cap_vb2_ops;
	cap->queue.mem_ops = &vb2_dma_contig_memops;
	cap->queue.buf_struct_size = sizeof(struct meson_csi_cap_buffer);
	cap->queue.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	cap->queue.min_queued_buffers = MESON_CSI_CAP_BUF_MIN;
	cap->queue.lock = &cap->lock;
	cap->queue.dev = &pdev->dev;

	ret = vb2_queue_init(&cap->queue);
	if (ret)
		goto err_v4l2;

	/* Default format: 640x480 Bayer through VDIN (YUV422 output) */
	cap->cap_fmt = &cap_formats[0];
	cap->fmt.width = 640;
	cap->fmt.height = 480;
	cap->fmt.pixelformat = cap->cap_fmt->fourcc;
	cap->fmt.field = V4L2_FIELD_NONE;
	cap->fmt.bytesperline = cap_bytesperline(&cap->fmt, cap->cap_fmt);
	cap->fmt.sizeimage = cap->fmt.bytesperline * cap->fmt.height;
	cap->fmt.colorspace = V4L2_COLORSPACE_RAW;

	/* Video device */
	cap->vdev.fops = &meson_csi_cap_fops;
	cap->vdev.ioctl_ops = &meson_csi_cap_ioctl_ops;
	cap->vdev.release = video_device_release_empty;
	cap->vdev.v4l2_dev = &cap->v4l2_dev;
	cap->vdev.queue = &cap->queue;
	cap->vdev.lock = &cap->lock;
	cap->vdev.device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	strscpy(cap->vdev.name, MESON_CSI_CAP_NAME, sizeof(cap->vdev.name));
	video_set_drvdata(&cap->vdev, cap);

	/* Media entity for video device */
	cap->pad.flags = MEDIA_PAD_FL_SINK;
	cap->vdev.entity.function = MEDIA_ENT_F_IO_V4L;
	ret = media_entity_pads_init(&cap->vdev.entity, 1, &cap->pad);
	if (ret)
		goto err_v4l2;

	/* Parse DT and register async notifier */
	ret = meson_csi_cap_parse_dt(cap);
	if (ret) {
		dev_err(&pdev->dev, "Failed to parse DT: %d\n", ret);
		goto err_entity;
	}

	ret = media_device_register(&cap->media_dev);
	if (ret)
		goto err_nf;

	dev_info(&pdev->dev, "Probed CSI capture bridge (VDIN DMA)\n");

	return 0;

err_nf:
	v4l2_async_nf_unregister(&cap->notifier);
	v4l2_async_nf_cleanup(&cap->notifier);
err_entity:
	media_entity_cleanup(&cap->vdev.entity);
err_v4l2:
	v4l2_device_unregister(&cap->v4l2_dev);
err_vdin:
	media_device_cleanup(&cap->media_dev);
	meson_canvas_free(cap->canvas, cap->canvas_id);
err_vdin_misc:
	if (cap->vpu_misc)
		iounmap(cap->vpu_misc);
	if (cap->vdin_misc)
		iounmap(cap->vdin_misc);
err_vdin_base:
	iounmap(cap->vdin_base);
err_clk_adapt:
	clk_disable_unprepare(cap->clk_adapt);
err_clk_gate:
	clk_disable_unprepare(cap->clk_gate);
err_clk_isp:
	clk_disable_unprepare(cap->clk_isp);
	return ret;
}

static void meson_csi_cap_remove(struct platform_device *pdev)
{
	struct meson_csi_cap *cap = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&cap->frame_work);
	video_unregister_device(&cap->vdev);
	v4l2_async_nf_unregister(&cap->notifier);
	v4l2_async_nf_cleanup(&cap->notifier);
	media_entity_cleanup(&cap->vdev.entity);
	media_device_unregister(&cap->media_dev);
	v4l2_device_unregister(&cap->v4l2_dev);
	media_device_cleanup(&cap->media_dev);
	meson_canvas_free(cap->canvas, cap->canvas_id);
	if (cap->vpu_misc)
		iounmap(cap->vpu_misc);
	if (cap->vdin_misc)
		iounmap(cap->vdin_misc);
	if (cap->vdin_base)
		iounmap(cap->vdin_base);
	clk_disable_unprepare(cap->clk_adapt);
	clk_disable_unprepare(cap->clk_gate);
	clk_disable_unprepare(cap->clk_isp);
}

static const struct of_device_id meson_csi_cap_of_table[] = {
	{ .compatible = "amlogic,g12a-csi-capture" },
	{ },
};
MODULE_DEVICE_TABLE(of, meson_csi_cap_of_table);

static struct platform_driver meson_csi_cap_driver = {
	.probe		= meson_csi_cap_probe,
	.remove		= meson_csi_cap_remove,
	.driver		= {
		.name		= MESON_CSI_CAP_NAME,
		.of_match_table	= meson_csi_cap_of_table,
	},
};
module_platform_driver(meson_csi_cap_driver);

MODULE_AUTHOR("Da Xue <da@libre.computer>");
MODULE_DESCRIPTION("Amlogic Meson CSI-2 Capture Bridge");
MODULE_LICENSE("GPL");
