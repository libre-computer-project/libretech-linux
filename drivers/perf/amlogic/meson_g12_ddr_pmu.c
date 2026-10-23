// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022 Amlogic, Inc. All rights reserved.
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/perf_event.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/types.h>

#include <soc/amlogic/meson_ddr_pmu.h>

#define PORT_MAJOR		32
#define DEFAULT_XTAL_FREQ	24000000UL

#define DMC_QOS_IRQ		BIT(30)

/* DMC bandwidth monitor register address offset */
#define DMC_MON_G12_CTRL0		(0x0  << 2)
#define DMC_MON_G12_CTRL1		(0x1  << 2)
#define DMC_MON_G12_CTRL2		(0x2  << 2)
#define DMC_MON_G12_CTRL3		(0x3  << 2)
#define DMC_MON_G12_CTRL4		(0x4  << 2)
#define DMC_MON_G12_CTRL5		(0x5  << 2)
#define DMC_MON_G12_CTRL6		(0x6  << 2)
#define DMC_MON_G12_CTRL7		(0x7  << 2)
#define DMC_MON_G12_CTRL8		(0x8  << 2)

#define DMC_MON_G12_ALL_REQ_CNT		(0x9  << 2)
#define DMC_MON_G12_ALL_GRANT_CNT	(0xa  << 2)
#define DMC_MON_G12_ONE_GRANT_CNT	(0xb  << 2)
#define DMC_MON_G12_SEC_GRANT_CNT	(0xc  << 2)
#define DMC_MON_G12_THD_GRANT_CNT	(0xd  << 2)
#define DMC_MON_G12_FOR_GRANT_CNT	(0xe  << 2)
#define DMC_MON_G12_TIMER		(0xf  << 2)

/* Each bit represent a axi line */
PMU_FORMAT_ATTR(event, "config:0-7");
PMU_FORMAT_ATTR(arm, "config1:0");
PMU_FORMAT_ATTR(gpu, "config1:1");
PMU_FORMAT_ATTR(pcie, "config1:2");
PMU_FORMAT_ATTR(hdcp, "config1:3");
PMU_FORMAT_ATTR(hevc_front, "config1:4");
PMU_FORMAT_ATTR(usb3_0, "config1:6");
PMU_FORMAT_ATTR(device, "config1:7");
PMU_FORMAT_ATTR(hevc_back, "config1:8");
PMU_FORMAT_ATTR(h265enc, "config1:9");
PMU_FORMAT_ATTR(vpu_read1, "config1:16");
PMU_FORMAT_ATTR(vpu_read2, "config1:17");
PMU_FORMAT_ATTR(vpu_read3, "config1:18");
PMU_FORMAT_ATTR(vpu_write1, "config1:19");
PMU_FORMAT_ATTR(vpu_write2, "config1:20");
PMU_FORMAT_ATTR(vdec, "config1:21");
PMU_FORMAT_ATTR(hcodec, "config1:22");
PMU_FORMAT_ATTR(ge2d, "config1:23");

PMU_FORMAT_ATTR(spicc1, "config1:32");
PMU_FORMAT_ATTR(usb0, "config1:33");
PMU_FORMAT_ATTR(dma, "config1:34");
PMU_FORMAT_ATTR(arb0, "config1:35");
PMU_FORMAT_ATTR(sd_emmc_b, "config1:36");
PMU_FORMAT_ATTR(usb1, "config1:37");
PMU_FORMAT_ATTR(audio, "config1:38");
PMU_FORMAT_ATTR(aififo, "config1:39");
PMU_FORMAT_ATTR(parser, "config1:41");
PMU_FORMAT_ATTR(ao_cpu, "config1:42");
PMU_FORMAT_ATTR(sd_emmc_c, "config1:43");
PMU_FORMAT_ATTR(spicc2, "config1:44");
PMU_FORMAT_ATTR(ethernet, "config1:45");
PMU_FORMAT_ATTR(sana, "config1:46");

/* for sm1 and g12b */
PMU_FORMAT_ATTR(nna, "config1:10");

/* for g12b only */
PMU_FORMAT_ATTR(gdc, "config1:11");
PMU_FORMAT_ATTR(mipi_isp, "config1:12");
PMU_FORMAT_ATTR(arm1, "config1:13");
PMU_FORMAT_ATTR(sd_emmc_a, "config1:40");

static struct attribute *g12_pmu_format_attrs[] = {
	&format_attr_event.attr,
	&format_attr_arm.attr,
	&format_attr_gpu.attr,
	&format_attr_nna.attr,
	&format_attr_gdc.attr,
	&format_attr_arm1.attr,
	&format_attr_mipi_isp.attr,
	&format_attr_sd_emmc_a.attr,
	&format_attr_pcie.attr,
	&format_attr_hdcp.attr,
	&format_attr_hevc_front.attr,
	&format_attr_usb3_0.attr,
	&format_attr_device.attr,
	&format_attr_hevc_back.attr,
	&format_attr_h265enc.attr,
	&format_attr_vpu_read1.attr,
	&format_attr_vpu_read2.attr,
	&format_attr_vpu_read3.attr,
	&format_attr_vpu_write1.attr,
	&format_attr_vpu_write2.attr,
	&format_attr_vdec.attr,
	&format_attr_hcodec.attr,
	&format_attr_ge2d.attr,
	&format_attr_spicc1.attr,
	&format_attr_usb0.attr,
	&format_attr_dma.attr,
	&format_attr_arb0.attr,
	&format_attr_sd_emmc_b.attr,
	&format_attr_usb1.attr,
	&format_attr_audio.attr,
	&format_attr_aififo.attr,
	&format_attr_parser.attr,
	&format_attr_ao_cpu.attr,
	&format_attr_sd_emmc_c.attr,
	&format_attr_spicc2.attr,
	&format_attr_ethernet.attr,
	&format_attr_sana.attr,
	NULL,
};

/* calculate ddr clock */
static unsigned long dmc_g12_get_freq_quick(struct dmc_info *info)
{
	unsigned int val;
	unsigned int n, m, od1;
	unsigned int od_div = 0xfff;
	unsigned long freq = 0;

	val = readl(info->pll_reg);
	val = val & 0xfffff;
	switch ((val >> 16) & 7) {
	case 0:
		od_div = 2;
		break;

	case 1:
		od_div = 3;
		break;

	case 2:
		od_div = 4;
		break;

	case 3:
		od_div = 6;
		break;

	case 4:
		od_div = 8;
		break;

	default:
		break;
	}

	m = val & 0x1ff;
	n = ((val >> 10) & 0x1f);
	od1 = (((val >> 19) & 0x1)) == 1 ? 2 : 1;
	freq = DEFAULT_XTAL_FREQ / 1000;        /* avoid overflow */
	if (n)
		freq = ((((freq * m) / n) >> od1) / od_div) * 1000;

	return freq;
}

#ifdef DEBUG
static void g12_dump_reg(struct dmc_info *db)
{
	int s = 0, i;
	unsigned int r;

	for (i = 0; i < 9; i++) {
		r  = readl(db->ddr_reg[0] + (DMC_MON_G12_CTRL0 + (i << 2)));
		pr_notice("DMC_MON_CTRL%d:        %08x\n", i, r);
	}
	r  = readl(db->ddr_reg[0] + DMC_MON_G12_ALL_REQ_CNT);
	pr_notice("DMC_MON_ALL_REQ_CNT:  %08x\n", r);
	r  = readl(db->ddr_reg[0] + DMC_MON_G12_ALL_GRANT_CNT);
	pr_notice("DMC_MON_ALL_GRANT_CNT:%08x\n", r);
	r  = readl(db->ddr_reg[0] + DMC_MON_G12_ONE_GRANT_CNT);
	pr_notice("DMC_MON_ONE_GRANT_CNT:%08x\n", r);
	r  = readl(db->ddr_reg[0] + DMC_MON_G12_SEC_GRANT_CNT);
	pr_notice("DMC_MON_SEC_GRANT_CNT:%08x\n", r);
	r  = readl(db->ddr_reg[0] + DMC_MON_G12_THD_GRANT_CNT);
	pr_notice("DMC_MON_THD_GRANT_CNT:%08x\n", r);
	r  = readl(db->ddr_reg[0] + DMC_MON_G12_FOR_GRANT_CNT);
	pr_notice("DMC_MON_FOR_GRANT_CNT:%08x\n", r);
	r  = readl(db->ddr_reg[0] + DMC_MON_G12_TIMER);
	pr_notice("DMC_MON_TIMER:        %08x\n", r);
}
#endif

static void dmc_g12_counter_enable(struct dmc_info *info)
{
	unsigned int val;
	unsigned long clock_count = dmc_g12_get_freq_quick(info) / 10; /* 100ms */

	writel(clock_count, info->ddr_reg[0] + DMC_MON_G12_TIMER);

	val = readl(info->ddr_reg[0] + DMC_MON_G12_CTRL0);

	/* enable all channel */
	val =  BIT(31) |	/* enable bit */
	       BIT(20) |	/* use timer  */
	       0x0f;		/* 4 channels */

	writel(val, info->ddr_reg[0] + DMC_MON_G12_CTRL0);

#ifdef DEBUG
	g12_dump_reg(info);
#endif
}

static void dmc_g12_config_fiter(struct dmc_info *info,
				 int port, int channel)
{
	u32 val;
	u32 rp[MAX_CHANNEL_NUM] = {DMC_MON_G12_CTRL1, DMC_MON_G12_CTRL3,
					DMC_MON_G12_CTRL5, DMC_MON_G12_CTRL7};
	u32 rs[MAX_CHANNEL_NUM] = {DMC_MON_G12_CTRL2, DMC_MON_G12_CTRL4,
					DMC_MON_G12_CTRL6, DMC_MON_G12_CTRL8};
	int subport = -1;

	/* clear all port mask */
	if (port < 0) {
		writel(0, info->ddr_reg[0] + rp[channel]);
		writel(0, info->ddr_reg[0] + rs[channel]);
		return;
	}

	if (port >= PORT_MAJOR)
		subport = port - PORT_MAJOR;

	if (subport < 0) {
		val = readl(info->ddr_reg[0] + rp[channel]);
		val |=  (1 << port);
		writel(val, info->ddr_reg[0] + rp[channel]);
		val = 0xffff;
		writel(val, info->ddr_reg[0] + rs[channel]);
	} else {
		val = BIT(23);		/* select device */
		writel(val, info->ddr_reg[0] + rp[channel]);
		val = readl(info->ddr_reg[0] + rs[channel]);
		val |= (1 << subport);
		writel(val, info->ddr_reg[0] + rs[channel]);
	}
}

static void dmc_g12_set_axi_filter(struct dmc_info *info, int axi_id, int channel)
{
	if (channel > info->hw_info->chann_nr)
		return;

	dmc_g12_config_fiter(info, axi_id, channel);
}

static void dmc_g12_counter_disable(struct dmc_info *info)
{
	int i;

	/* clear timer */
	writel(0, info->ddr_reg[0] + DMC_MON_G12_CTRL0);
	writel(0, info->ddr_reg[0] + DMC_MON_G12_TIMER);

	writel(0, info->ddr_reg[0] + DMC_MON_G12_ALL_REQ_CNT);
	writel(0, info->ddr_reg[0] + DMC_MON_G12_ALL_GRANT_CNT);
	writel(0, info->ddr_reg[0] + DMC_MON_G12_ONE_GRANT_CNT);
	writel(0, info->ddr_reg[0] + DMC_MON_G12_SEC_GRANT_CNT);
	writel(0, info->ddr_reg[0] + DMC_MON_G12_THD_GRANT_CNT);
	writel(0, info->ddr_reg[0] + DMC_MON_G12_FOR_GRANT_CNT);

	/* clear port channel mapping */
	for (i = 0; i < info->hw_info->chann_nr; i++)
		dmc_g12_config_fiter(info, -1, i);
}

static void dmc_g12_get_counters(struct dmc_info *info,
				 struct dmc_counter *counter)
{
	int i;
	unsigned int reg;

	counter->all_cnt = readl(info->ddr_reg[0] + DMC_MON_G12_ALL_GRANT_CNT);
	counter->all_req   = readl(info->ddr_reg[0] + DMC_MON_G12_ALL_REQ_CNT);

	for (i = 0; i < info->hw_info->chann_nr; i++) {
		reg = DMC_MON_G12_ONE_GRANT_CNT + (i << 2);
		counter->channel_cnt[i] = readl(info->ddr_reg[0] + reg);
	}
}

static void dmc_g12_counter_clear(struct dmc_info *info,
				  struct dmc_counter *drained)
{
	writel(0, info->ddr_reg[0] + DMC_MON_G12_CTRL0);

	dmc_g12_get_counters(info, drained);

	writel(0, info->ddr_reg[0] + DMC_MON_G12_ALL_REQ_CNT);
	writel(0, info->ddr_reg[0] + DMC_MON_G12_ALL_GRANT_CNT);
	writel(0, info->ddr_reg[0] + DMC_MON_G12_ONE_GRANT_CNT);
	writel(0, info->ddr_reg[0] + DMC_MON_G12_SEC_GRANT_CNT);
	writel(0, info->ddr_reg[0] + DMC_MON_G12_THD_GRANT_CNT);
	writel(0, info->ddr_reg[0] + DMC_MON_G12_FOR_GRANT_CNT);

	/*
	 * CTRL1..CTRL8 (the axi-port filters) are deliberately untouched: they
	 * belong to the event that programmed them, not to the counting window.
	 */
}

static int dmc_g12_irq_handler(struct dmc_info *info,
			       struct dmc_counter *counter)
{
	unsigned int val;
	int ret = -EINVAL;

	val = readl(info->ddr_reg[0] + DMC_MON_G12_CTRL0);
	if (val & DMC_QOS_IRQ) {
		dmc_g12_get_counters(info, counter);
		/* clear irq flags */
		writel(val, info->ddr_reg[0] + DMC_MON_G12_CTRL0);
		ret = 0;
	}
	return ret;
}

/*
 * GXL/GXM family backend. Same class of DMC bandwidth monitor as G12 but at a
 * different base (DMC 0xc8838000) with a different register layout. The DT
 * reg[0] points at a window starting at DMC+0x60 -- above canvas@48, so the
 * core's exclusive ioremap does not clash with the canvas driver. The offsets
 * below are relative to that window (absolute DMC offset in the comment).
 * Register model from the Amlogic BSP ddr_band_op_gxl.c.
 */
#define DMC_MON_GXL_CTRL1		0x34	/* abs 0x94: channel 0 port filter */
#define DMC_MON_GXL_CTRL2		0x38	/* abs 0x98: enable(31)/use-timer(20)/chan[3:0] */
#define DMC_MON_GXL_CTRL3		0x3c	/* abs 0x9c: timer clock_count */
#define DMC_MON_GXL_CTRL4		0x00	/* abs 0x60: channel 1 port filter */
#define DMC_MON_GXL_CTRL5		0x04	/* abs 0x64: channel 2 port filter */
#define DMC_MON_GXL_CTRL6		0x08	/* abs 0x68: channel 3 port filter */
#define DMC_MON_GXL_ALL_REQ_CNT		0x40	/* abs 0xa0 */
#define DMC_MON_GXL_ALL_GRANT_CNT	0x44	/* abs 0xa4: all-channel grant (x16 = bytes) */
#define DMC_MON_GXL_ONE_GRANT_CNT	0x48	/* abs 0xa8: ch0 grant; ch1..3 at +0x4/0x8/0xc */

/*
 * GXL per-master axi-line filters. The config1 bit is the port_id from the BSP
 * ddr_port_desc_gxl[]. GXL differs from G12 (VPU/VDEC/HCODEC/GE2D at bits 8-15,
 * not 16-23), so it needs its own format-attr set. Clean userspace names with
 * gxl_-prefixed symbols so they don't collide with the G12 format_attr_* of the
 * same name in this file.
 */
#define GXL_FMT_ATTR(_name, _cfg)					\
	static ssize_t gxl_##_name##_show(struct device *dev,		\
			struct device_attribute *attr, char *page)	\
	{								\
		return sprintf(page, _cfg "\n");			\
	}								\
	static struct device_attribute gxl_format_attr_##_name =	\
		__ATTR(_name, 0444, gxl_##_name##_show, NULL)

GXL_FMT_ATTR(arm,        "config1:0");
GXL_FMT_ATTR(mali0,      "config1:1");
GXL_FMT_ATTR(mali1,      "config1:2");
GXL_FMT_ATTR(hdcp,       "config1:3");
GXL_FMT_ATTR(hevc,       "config1:4");
GXL_FMT_ATTR(usb3_0,     "config1:6");
GXL_FMT_ATTR(device,     "config1:7");
GXL_FMT_ATTR(vpu_read0,  "config1:8");
GXL_FMT_ATTR(vpu_read1,  "config1:9");
GXL_FMT_ATTR(vpu_read2,  "config1:10");
GXL_FMT_ATTR(vpu_write0, "config1:11");
GXL_FMT_ATTR(vpu_write1, "config1:12");
GXL_FMT_ATTR(vdec,       "config1:13");
GXL_FMT_ATTR(hcodec,     "config1:14");
GXL_FMT_ATTR(ge2d,       "config1:15");
GXL_FMT_ATTR(sd_emmc_a,  "config1:32");
GXL_FMT_ATTR(usb0,       "config1:33");
GXL_FMT_ATTR(dma,        "config1:34");
GXL_FMT_ATTR(arb0,       "config1:35");
GXL_FMT_ATTR(sd_emmc_b,  "config1:36");
GXL_FMT_ATTR(usb1,       "config1:37");
GXL_FMT_ATTR(audio_out,  "config1:38");
GXL_FMT_ATTR(audio_in,   "config1:39");
GXL_FMT_ATTR(aiu,        "config1:40");
GXL_FMT_ATTR(parser,     "config1:41");
GXL_FMT_ATTR(ao_cpu,     "config1:42");
GXL_FMT_ATTR(sd_emmc_c,  "config1:43");
GXL_FMT_ATTR(spicc,      "config1:44");
GXL_FMT_ATTR(ethernet,   "config1:45");
GXL_FMT_ATTR(sana,       "config1:46");

static struct attribute *gxl_pmu_format_attrs[] = {
	&format_attr_event.attr,
	&gxl_format_attr_arm.attr,
	&gxl_format_attr_mali0.attr,
	&gxl_format_attr_mali1.attr,
	&gxl_format_attr_hdcp.attr,
	&gxl_format_attr_hevc.attr,
	&gxl_format_attr_usb3_0.attr,
	&gxl_format_attr_device.attr,
	&gxl_format_attr_vpu_read0.attr,
	&gxl_format_attr_vpu_read1.attr,
	&gxl_format_attr_vpu_read2.attr,
	&gxl_format_attr_vpu_write0.attr,
	&gxl_format_attr_vpu_write1.attr,
	&gxl_format_attr_vdec.attr,
	&gxl_format_attr_hcodec.attr,
	&gxl_format_attr_ge2d.attr,
	&gxl_format_attr_sd_emmc_a.attr,
	&gxl_format_attr_usb0.attr,
	&gxl_format_attr_dma.attr,
	&gxl_format_attr_arb0.attr,
	&gxl_format_attr_sd_emmc_b.attr,
	&gxl_format_attr_usb1.attr,
	&gxl_format_attr_audio_out.attr,
	&gxl_format_attr_audio_in.attr,
	&gxl_format_attr_aiu.attr,
	&gxl_format_attr_parser.attr,
	&gxl_format_attr_ao_cpu.attr,
	&gxl_format_attr_sd_emmc_c.attr,
	&gxl_format_attr_spicc.attr,
	&gxl_format_attr_ethernet.attr,
	&gxl_format_attr_sana.attr,
	NULL,
};

/*
 * The GXL DDR PLL is HHI_DPLL_TOP_I (0xc883c318 -- "DPLL" = DDR PLL, named in
 * clk-gxbb.h but not modelled as a clock), in the HHI block, supplied as
 * reg[1]/pll_reg. Found by RE of the board BL2/BL30 (which program the HHI clock
 * registers) and confirmed by reading it on an S805X (decodes to ~the measured
 * 540 MHz DMC clock). Unlike G12 (PLL at DMC+0xc00, same block as the monitor),
 * the GXL DPLL is in a different block from the DMC monitor, so the DT node sits
 * at soc level with absolute reg cells.
 */
static unsigned long dmc_gxl_get_freq_quick(struct dmc_info *info)
{
	unsigned int val, m, n, od, od1;
	unsigned long freq = 0;

	val = readl(info->pll_reg);
	od  = (val >> 2) & 0x03;
	m   = (val >> 4) & 0x1ff;
	n   = (val >> 16) & 0x1f;
	od1 = (val >> 0) & 0x03;
	freq = DEFAULT_XTAL_FREQ / 1000;	/* avoid overflow */
	if (n)
		freq = ((freq * m / (n * (1 + od))) >> od1) * 1000;

	return freq;
}

static void dmc_gxl_counter_enable(struct dmc_info *info)
{
	unsigned long clock_count = dmc_gxl_get_freq_quick(info) / 10; /* 100ms */

	writel(clock_count, info->ddr_reg[0] + DMC_MON_GXL_CTRL3);

	/* enable all four channels, counted over the timer window */
	writel(BIT(31) | BIT(20) | 0x0f, info->ddr_reg[0] + DMC_MON_GXL_CTRL2);
}

static void dmc_gxl_config_fiter(struct dmc_info *info, int port, int channel)
{
	u32 port_reg[MAX_CHANNEL_NUM] = { DMC_MON_GXL_CTRL1, DMC_MON_GXL_CTRL4,
					  DMC_MON_GXL_CTRL5, DMC_MON_GXL_CTRL6 };
	int subport = -1;
	u32 val;

	/* clear all port mask */
	if (port < 0) {
		writel(0, info->ddr_reg[0] + port_reg[channel]);
		return;
	}

	if (port >= PORT_MAJOR)
		subport = port - PORT_MAJOR;

	val = readl(info->ddr_reg[0] + port_reg[channel]);
	if (port < 16)
		val |= (1 << (16 + port)) | 0xffff;
	else if (subport > 0)
		val = BIT(23) | (1 << subport);
	else
		return;
	writel(val, info->ddr_reg[0] + port_reg[channel]);
}

static void dmc_gxl_set_axi_filter(struct dmc_info *info, int axi_id, int channel)
{
	if (channel > info->hw_info->chann_nr)
		return;

	dmc_gxl_config_fiter(info, axi_id, channel);
}

static void dmc_gxl_counter_disable(struct dmc_info *info)
{
	int i;

	writel(0, info->ddr_reg[0] + DMC_MON_GXL_CTRL2);
	writel(0, info->ddr_reg[0] + DMC_MON_GXL_CTRL3);
	writel(0, info->ddr_reg[0] + DMC_MON_GXL_ALL_REQ_CNT);
	writel(0, info->ddr_reg[0] + DMC_MON_GXL_ALL_GRANT_CNT);
	for (i = 0; i < info->hw_info->chann_nr; i++)
		writel(0, info->ddr_reg[0] + DMC_MON_GXL_ONE_GRANT_CNT + (i << 2));

	/* clear port channel mapping */
	for (i = 0; i < info->hw_info->chann_nr; i++)
		dmc_gxl_config_fiter(info, -1, i);
}

static void dmc_gxl_get_counters(struct dmc_info *info,
				 struct dmc_counter *counter)
{
	int i;
	unsigned int reg;

	counter->all_cnt = readl(info->ddr_reg[0] + DMC_MON_GXL_ALL_GRANT_CNT);
	counter->all_req = readl(info->ddr_reg[0] + DMC_MON_GXL_ALL_REQ_CNT);

	for (i = 0; i < info->hw_info->chann_nr; i++) {
		reg = DMC_MON_GXL_ONE_GRANT_CNT + (i << 2);
		counter->channel_cnt[i] = readl(info->ddr_reg[0] + reg);
	}
}

static void dmc_gxl_counter_clear(struct dmc_info *info,
				  struct dmc_counter *drained)
{
	int i;

	writel(0, info->ddr_reg[0] + DMC_MON_GXL_CTRL2);

	dmc_gxl_get_counters(info, drained);

	writel(0, info->ddr_reg[0] + DMC_MON_GXL_ALL_REQ_CNT);
	writel(0, info->ddr_reg[0] + DMC_MON_GXL_ALL_GRANT_CNT);
	for (i = 0; i < info->hw_info->chann_nr; i++)
		writel(0, info->ddr_reg[0] + DMC_MON_GXL_ONE_GRANT_CNT + (i << 2));

	/* CTRL1/CTRL4/CTRL5/CTRL6 (the axi-port filters) are left as set */
}

static int dmc_gxl_irq_handler(struct dmc_info *info,
			       struct dmc_counter *counter)
{
	unsigned int val;
	int ret = -EINVAL;

	val = readl(info->ddr_reg[0] + DMC_MON_GXL_CTRL2);
	if (val & DMC_QOS_IRQ) {
		dmc_gxl_get_counters(info, counter);
		/* clear irq flags */
		writel(val, info->ddr_reg[0] + DMC_MON_GXL_CTRL2);
		ret = 0;
	}
	return ret;
}

static const struct dmc_hw_info gxl_dmc_info = {
	.enable		= dmc_gxl_counter_enable,
	.disable	= dmc_gxl_counter_disable,
	.irq_handler	= dmc_gxl_irq_handler,
	.get_counters	= dmc_gxl_get_counters,
	.clear_counters = dmc_gxl_counter_clear,
	.set_axi_filter	= dmc_gxl_set_axi_filter,

	.dmc_nr = 1,
	.chann_nr = 4,
	.capability = {0x00007FFF0000FFDFULL, 0},	/* GXL axi-lines: ports 0-4, 6-15, 32-46 */
	.fmt_attr = gxl_pmu_format_attrs,
};

static const struct dmc_hw_info g12a_dmc_info = {
	.enable		= dmc_g12_counter_enable,
	.disable	= dmc_g12_counter_disable,
	.irq_handler	= dmc_g12_irq_handler,
	.get_counters	= dmc_g12_get_counters,
	.clear_counters = dmc_g12_counter_clear,
	.set_axi_filter	= dmc_g12_set_axi_filter,

	.dmc_nr = 1,
	.chann_nr = 4,
	.capability = {0X7EFF00FF03DF, 0},
	.fmt_attr = g12_pmu_format_attrs,
};

static const struct dmc_hw_info g12b_dmc_info = {
	.enable		= dmc_g12_counter_enable,
	.disable	= dmc_g12_counter_disable,
	.irq_handler	= dmc_g12_irq_handler,
	.get_counters	= dmc_g12_get_counters,
	.clear_counters = dmc_g12_counter_clear,
	.set_axi_filter	= dmc_g12_set_axi_filter,

	.dmc_nr = 1,
	.chann_nr = 4,
	.capability = {0X7FFF00FF3FDF, 0},
	.fmt_attr = g12_pmu_format_attrs,
};

static const struct dmc_hw_info sm1_dmc_info = {
	.enable		= dmc_g12_counter_enable,
	.disable	= dmc_g12_counter_disable,
	.irq_handler	= dmc_g12_irq_handler,
	.get_counters	= dmc_g12_get_counters,
	.clear_counters = dmc_g12_counter_clear,
	.set_axi_filter	= dmc_g12_set_axi_filter,

	.dmc_nr = 1,
	.chann_nr = 4,
	.capability = {0X7EFF00FF07DF, 0},
	.fmt_attr = g12_pmu_format_attrs,
};

static int g12_ddr_pmu_probe(struct platform_device *pdev)
{
	return meson_ddr_pmu_create(pdev);
}

static void g12_ddr_pmu_remove(struct platform_device *pdev)
{
	meson_ddr_pmu_remove(pdev);
}

static const struct of_device_id meson_ddr_pmu_dt_match[] = {
	{
		.compatible = "amlogic,g12a-ddr-pmu",
		.data = &g12a_dmc_info,
	},
	{
		.compatible = "amlogic,g12b-ddr-pmu",
		.data = &g12b_dmc_info,
	},
	{
		.compatible = "amlogic,sm1-ddr-pmu",
		.data = &sm1_dmc_info,
	},
	{
		.compatible = "amlogic,gxl-ddr-pmu",
		.data = &gxl_dmc_info,
	},
	{}
};
MODULE_DEVICE_TABLE(of, meson_ddr_pmu_dt_match);

static struct platform_driver g12_ddr_pmu_driver = {
	.probe = g12_ddr_pmu_probe,
	.remove = g12_ddr_pmu_remove,

	.driver = {
		.name = "meson-g12-ddr-pmu",
		.of_match_table = meson_ddr_pmu_dt_match,
	},
};

module_platform_driver(g12_ddr_pmu_driver);
MODULE_AUTHOR("Jiucheng Xu");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Amlogic G12/GXL series SoC DDR PMU");
