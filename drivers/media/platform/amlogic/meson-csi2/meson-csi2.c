// SPDX-License-Identifier: GPL-2.0+
/*
 * Amlogic Meson MIPI CSI-2 Receiver
 *
 * Copyright (C) 2018 Amlogic, Inc. All rights reserved
 */

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

/*
 * Runtime-tunable CSI-2 host parameters.
 * Set via /sys/module/meson_csi2/parameters/
 */
static unsigned int settle_ms = 120;
module_param(settle_ms, uint, 0644);
MODULE_PARM_DESC(settle_ms, "Post-stream settle delay in ms (default 120)");

static unsigned int debug_regs;
module_param(debug_regs, uint, 0644);
MODULE_PARM_DESC(debug_regs, "Dump extra host registers on stream start");

/*
 * CSI-2 Host registers (Synopsys DWC MIPI CSI-2 v1.02)
 *
 * Register map for v1.02 (VERSION = 0x3130322a):
 *   0x020: ERR1 (interrupt status 1)
 *   0x024: ERR2 (interrupt status 2)
 *   0x028: MASK1 (interrupt mask 1)
 *   0x02c: MASK2 (interrupt mask 2)
 *   0x030: PHY_TST_CTRL0
 *   0x034: PHY_TST_CTRL1
 *
 * Note: v1.30+ has a different interrupt register layout at 0x020+
 * with separate PHY_FATAL, PKT_FATAL, FRAME_FATAL etc. registers.
 * Those do NOT exist on v1.02 and overlap with PHY_TST_CTRL0/1.
 */
#define CSI2_HOST_N_LANES		0x004
#define CSI2_HOST_PHY_SHUTDOWNZ		0x008
#define CSI2_HOST_DPHY_RSTZ		0x00c
#define CSI2_HOST_CSI2_RESETN		0x010
#define CSI2_HOST_PHY_STATE		0x014
#define CSI2_HOST_DATA_IDS_1		0x018
#define CSI2_HOST_DATA_IDS_2		0x01c
#define CSI2_HOST_ERR1			0x020
#define CSI2_HOST_ERR2			0x024
#define CSI2_HOST_MASK1			0x028
#define CSI2_HOST_MASK2			0x02c
#define CSI2_HOST_PHY_TST_CTRL0		0x030
#define CSI2_HOST_PHY_TST_CTRL1		0x034
#define CSI2_HOST_VERSION		0x000

#define MESON_CSI2_LANES_MAX	4

enum meson_csi2_pads {
	MESON_CSI2_PAD_SINK,
	MESON_CSI2_PAD_SOURCE,
	MESON_CSI2_PAD_MAX,
};

struct meson_csi2_fmt {
	u32	code;
	u8	bpp;
};

struct meson_csi2_priv {
	struct device			*dev;
	void __iomem			*base;
	struct phy			*dphy;

	struct mutex			lock;

	u8				num_lanes;

	struct v4l2_subdev		subdev;
	struct v4l2_async_notifier	notifier;
	struct media_pad		pads[MESON_CSI2_PAD_MAX];

	struct v4l2_subdev		*source_subdev;
	int				source_pad;
};

static const struct meson_csi2_fmt formats[] = {
	{ .code = MEDIA_BUS_FMT_YUYV8_1X16,   .bpp = 16, },
	{ .code = MEDIA_BUS_FMT_UYVY8_1X16,   .bpp = 16, },
	{ .code = MEDIA_BUS_FMT_YVYU8_1X16,   .bpp = 16, },
	{ .code = MEDIA_BUS_FMT_VYUY8_1X16,   .bpp = 16, },
	{ .code = MEDIA_BUS_FMT_SBGGR8_1X8,   .bpp = 8, },
	{ .code = MEDIA_BUS_FMT_SGBRG8_1X8,   .bpp = 8, },
	{ .code = MEDIA_BUS_FMT_SGRBG8_1X8,   .bpp = 8, },
	{ .code = MEDIA_BUS_FMT_SRGGB8_1X8,   .bpp = 8, },
	{ .code = MEDIA_BUS_FMT_Y8_1X8,       .bpp = 8, },
	{ .code = MEDIA_BUS_FMT_SBGGR10_1X10, .bpp = 10, },
	{ .code = MEDIA_BUS_FMT_SGBRG10_1X10, .bpp = 10, },
	{ .code = MEDIA_BUS_FMT_SGRBG10_1X10, .bpp = 10, },
	{ .code = MEDIA_BUS_FMT_SRGGB10_1X10, .bpp = 10, },
	{ .code = MEDIA_BUS_FMT_RGB565_1X16,  .bpp = 16, },
	{ .code = MEDIA_BUS_FMT_RGB888_1X24,  .bpp = 24, },
	{ .code = MEDIA_BUS_FMT_BGR888_1X24,  .bpp = 24, },
};

static const struct meson_csi2_fmt *meson_csi2_get_fmt_by_code(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(formats); i++)
		if (formats[i].code == code)
			return &formats[i];

	return NULL;
}

static inline
struct meson_csi2_priv *v4l2_subdev_to_meson_csi2(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct meson_csi2_priv, subdev);
}

static int meson_csi2_configure_dphy(struct meson_csi2_priv *csi2)
{
	struct media_pad *src_pad =
		&csi2->source_subdev->entity.pads[csi2->source_pad];
	union phy_configure_opts opts = { };
	struct phy_configure_opts_mipi_dphy *cfg = &opts.mipi_dphy;
	struct v4l2_subdev_format sd_fmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.pad = MESON_CSI2_PAD_SINK,
	};
	const struct meson_csi2_fmt *fmt;
	s64 link_freq;
	int ret;

	ret = v4l2_subdev_call_state_active(&csi2->subdev, pad, get_fmt,
					    &sd_fmt);
	if (ret < 0)
		return ret;

	fmt = meson_csi2_get_fmt_by_code(sd_fmt.format.code);

	link_freq = v4l2_get_link_freq(src_pad,
				       fmt->bpp, 2 * csi2->num_lanes);
	if (link_freq < 0)
		return link_freq;

	dev_info(csi2->dev, "PHY config: link_freq=%lld bpp=%u lanes=%u fmt=0x%x\n",
		 link_freq, fmt->bpp, csi2->num_lanes, sd_fmt.format.code);

	ret = phy_mipi_dphy_get_default_config_for_hsclk(link_freq,
							 csi2->num_lanes, cfg);
	if (ret)
		return ret;

	ret = phy_configure(csi2->dphy, &opts);
	if (ret)
		return ret;

	return 0;
}

static irqreturn_t meson_csi2_irq(int irq, void *data)
{
	struct meson_csi2_priv *csi2 = data;
	u32 err1, err2;

	/* v1.02: read ERR1/ERR2 to clear interrupt status */
	err1 = readl(csi2->base + CSI2_HOST_ERR1);
	err2 = readl(csi2->base + CSI2_HOST_ERR2);

	if (!err1 && !err2)
		return IRQ_NONE;

	return IRQ_HANDLED;
}

static int meson_csi2_start(struct meson_csi2_priv *csi2)
{
	int ret;

	ret = phy_init(csi2->dphy);
	if (ret)
		return ret;

	ret = meson_csi2_configure_dphy(csi2);
	if (ret)
		goto err_phy_exit;

	/*
	 * Init sequence matching BSP (aphy_cfg + dphy_cfg + host_cfg):
	 * 1. Power on D-PHY (analog bias + digital timing/mux)
	 * 2. Configure DWC host: RESETN toggle, DPHY_RSTZ, N_LANES, PHY_SHUTDOWNZ
	 * 3. Start sensor
	 */

	/* Power on D-PHY: analog bias + CNTL3, then digital timing + lane mux */
	ret = phy_power_on(csi2->dphy);
	if (ret)
		goto err_phy_exit;

	/*
	 * DWC Host register dump BEFORE any writes — understand defaults.
	 * Critical for debugging: do DPHY_RSTZ/PHY_SHUTDOWNZ default to
	 * active (1) or shutdown (0) after power-on?
	 */
	dev_info(csi2->dev,
		 "host VERSION=0x%08x pre-init: RESETN=0x%x DPHY_RSTZ=0x%x PHY_SHUTDOWNZ=0x%x N_LANES=0x%x PHY_STATE=0x%x\n",
		 readl(csi2->base + CSI2_HOST_VERSION),
		 readl(csi2->base + CSI2_HOST_CSI2_RESETN),
		 readl(csi2->base + CSI2_HOST_DPHY_RSTZ),
		 readl(csi2->base + CSI2_HOST_PHY_SHUTDOWNZ),
		 readl(csi2->base + CSI2_HOST_N_LANES),
		 readl(csi2->base + CSI2_HOST_PHY_STATE));

	/*
	 * BSP init_am_mipi_csi2_host() (mipi_hw.c:190-198):
	 *   1. RESETN = 1 (just write 1, no toggle)
	 *   2. N_LANES = (lanes - 1) & 3
	 *   3. MASK1 = 0
	 *   4. MASK2 = 0
	 *   udelay(1)
	 *
	 * BSP does NOT write PHY_SHUTDOWNZ or DPHY_RSTZ.
	 * BSP does NOT toggle RESETN (0→1), just writes 1.
	 *
	 * However, we also write PHY_SHUTDOWNZ=1 and DPHY_RSTZ=1
	 * in case they defaulted to 0 after power domain enable.
	 */
	writel(1, csi2->base + CSI2_HOST_CSI2_RESETN);
	writel(1, csi2->base + CSI2_HOST_DPHY_RSTZ);
	writel((csi2->num_lanes - 1) & 3, csi2->base + CSI2_HOST_N_LANES);
	writel(1, csi2->base + CSI2_HOST_PHY_SHUTDOWNZ);
	writel(0, csi2->base + CSI2_HOST_MASK1);
	writel(0, csi2->base + CSI2_HOST_MASK2);
	udelay(1);

	dev_info(csi2->dev,
		 "host post-init: RESETN=0x%x DPHY_RSTZ=0x%x PHY_SHUTDOWNZ=0x%x N_LANES=0x%x PHY_STATE=0x%x\n",
		 readl(csi2->base + CSI2_HOST_CSI2_RESETN),
		 readl(csi2->base + CSI2_HOST_DPHY_RSTZ),
		 readl(csi2->base + CSI2_HOST_PHY_SHUTDOWNZ),
		 readl(csi2->base + CSI2_HOST_N_LANES),
		 readl(csi2->base + CSI2_HOST_PHY_STATE));

	/* Start sensor streaming */
	ret = v4l2_subdev_call(csi2->source_subdev, video, s_stream, true);
	if (ret)
		goto err_host_reset;

	msleep(settle_ms);

	{
		u32 phy1, phy2, err1, err2, ids1, ids2;
		int i;

		err1 = readl(csi2->base + CSI2_HOST_ERR1);
		err2 = readl(csi2->base + CSI2_HOST_ERR2);
		phy1 = readl(csi2->base + CSI2_HOST_PHY_STATE);
		ids1 = readl(csi2->base + CSI2_HOST_DATA_IDS_1);
		ids2 = readl(csi2->base + CSI2_HOST_DATA_IDS_2);

		dev_info(csi2->dev,
			 "post-stream: PHY_STATE=0x%x ERR1=0x%08x ERR2=0x%08x\n",
			 phy1, err1, err2);

		/* Decode ERR1 bits */
		if (err1 & 0xf)
			dev_info(csi2->dev, "  ERR1: SoT_sync_err lanes=0x%x\n",
				 err1 & 0xf);
		if (err1 & 0xf0)
			dev_info(csi2->dev, "  ERR1: frame_bndry_match VC=0x%x\n",
				 (err1 >> 4) & 0xf);
		if (err1 & 0xf00)
			dev_info(csi2->dev, "  ERR1: frame_seq VC=0x%x\n",
				 (err1 >> 8) & 0xf);
		if (err1 & 0xf000)
			dev_info(csi2->dev, "  ERR1: frame_data VC=0x%x\n",
				 (err1 >> 12) & 0xf);
		if (err1 & 0xf0000)
			dev_info(csi2->dev, "  ERR1: line_bndry_match DI=0x%x\n",
				 (err1 >> 16) & 0xf);
		if (err1 & 0xf00000)
			dev_info(csi2->dev, "  ERR1: line_seq DI=0x%x\n",
				 (err1 >> 20) & 0xf);
		if (err1 & 0xf000000)
			dev_info(csi2->dev, "  ERR1: CRC VC=0x%x\n",
				 (err1 >> 24) & 0xf);
		if (err1 & BIT(28))
			dev_info(csi2->dev, "  ERR1: ECC_DOUBLE (uncorrectable)\n");

		/* Decode ERR2 bits */
		if (err2 & 0xf)
			dev_info(csi2->dev, "  ERR2: esc_err lanes=0x%x\n",
				 err2 & 0xf);
		if (err2 & 0xf0)
			dev_info(csi2->dev, "  ERR2: SoT_err lanes=0x%x\n",
				 (err2 >> 4) & 0xf);
		if (err2 & 0xf00)
			dev_info(csi2->dev, "  ERR2: ECC_corrected VC=0x%x\n",
				 (err2 >> 8) & 0xf);
		if (err2 & 0xf000)
			dev_info(csi2->dev, "  ERR2: data_id VC=0x%x\n",
				 (err2 >> 12) & 0xf);

		dev_info(csi2->dev,
			 "  DATA_IDS_1=0x%x DATA_IDS_2=0x%x\n", ids1, ids2);

		/*
		 * Rapid PHY_STATE sampling: capture all unique values over
		 * 1000 reads to build histogram of PHY states.
		 * This tells us if data lanes EVER leave stop state.
		 */
		{
			u32 seen[16];
			int seen_cnt = 0;
			int j;

			for (i = 0; i < 1000; i++) {
				phy2 = readl(csi2->base + CSI2_HOST_PHY_STATE);
				for (j = 0; j < seen_cnt; j++)
					if (seen[j] == phy2)
						break;
				if (j == seen_cnt && seen_cnt < 16)
					seen[seen_cnt++] = phy2;
			}
			dev_info(csi2->dev, "  PHY_STATE histogram (%d unique):\n",
				 seen_cnt);
			for (j = 0; j < seen_cnt; j++)
				dev_info(csi2->dev,
					 "    0x%03x: clk_stop=%d clk_hs=%d d0_stop=%d d1_stop=%d d0_ulps=%d d1_ulps=%d\n",
					 seen[j],
					 !!(seen[j] & BIT(10)),
					 !!(seen[j] & BIT(8)),
					 !!(seen[j] & BIT(4)),
					 !!(seen[j] & BIT(5)),
					 !!(seen[j] & BIT(0)),
					 !!(seen[j] & BIT(1)));
		}

		/* Clear errors and sample new ones over 500ms */
		err1 = readl(csi2->base + CSI2_HOST_ERR1);
		err2 = readl(csi2->base + CSI2_HOST_ERR2);
		for (i = 0; i < 5; i++) {
			msleep(100);
			err1 = readl(csi2->base + CSI2_HOST_ERR1);
			err2 = readl(csi2->base + CSI2_HOST_ERR2);
			if (err1 || err2)
				dev_info(csi2->dev,
					 "  sample[%d]: ERR1=0x%08x ERR2=0x%08x\n",
					 i, err1, err2);
		}
	}

	return 0;

err_host_reset:
	writel(0, csi2->base + CSI2_HOST_CSI2_RESETN);
	phy_power_off(csi2->dphy);
err_phy_exit:
	phy_exit(csi2->dphy);
	return ret;
}

static void meson_csi2_stop(struct meson_csi2_priv *csi2)
{
	v4l2_subdev_call(csi2->source_subdev, video, s_stream, false);

	/* Mask interrupts */
	writel(0xffffffff, csi2->base + CSI2_HOST_MASK1);
	writel(0xffffffff, csi2->base + CSI2_HOST_MASK2);

	/* BSP csiphy_reset() order: PHY_SHUTDOWNZ=0, DPHY_RSTZ=0, CSI2_RESETN=0 */
	writel(0, csi2->base + CSI2_HOST_PHY_SHUTDOWNZ);
	writel(0, csi2->base + CSI2_HOST_DPHY_RSTZ);
	writel(0, csi2->base + CSI2_HOST_CSI2_RESETN);

	phy_power_off(csi2->dphy);
	phy_exit(csi2->dphy);
}

static int meson_csi2_s_stream(struct v4l2_subdev *subdev, int enable)
{
	struct meson_csi2_priv *csi2 = v4l2_subdev_to_meson_csi2(subdev);
	int ret = 0;

	mutex_lock(&csi2->lock);

	if (enable)
		ret = meson_csi2_start(csi2);
	else
		meson_csi2_stop(csi2);

	mutex_unlock(&csi2->lock);
	return ret;
}

static int meson_csi2_enum_mbus_code(struct v4l2_subdev *subdev,
				     struct v4l2_subdev_state *state,
				     struct v4l2_subdev_mbus_code_enum *code_enum)
{
	if (code_enum->index >= ARRAY_SIZE(formats))
		return -EINVAL;

	code_enum->code = formats[code_enum->index].code;

	return 0;
}

static int meson_csi2_set_fmt(struct v4l2_subdev *subdev,
			      struct v4l2_subdev_state *state,
			      struct v4l2_subdev_format *format)
{
	struct meson_csi2_priv *csi2 = container_of(subdev, struct meson_csi2_priv, subdev);
	struct v4l2_mbus_framefmt *fmt;

	/* No transcoding, source and sink formats must match. */
	if (format->pad != MESON_CSI2_PAD_SINK)
		return v4l2_subdev_get_fmt(subdev, state, format);

	if (!meson_csi2_get_fmt_by_code(format->format.code))
		format->format.code = formats[0].code;

	format->format.field = V4L2_FIELD_NONE;

	/* Set sink format */
	fmt = v4l2_subdev_state_get_format(state, format->pad);
	*fmt = format->format;

	/* Propagate to source */
	fmt = v4l2_subdev_state_get_format(state, MESON_CSI2_PAD_SOURCE);
	*fmt = format->format;

	/* Propagate format to the sensor subdev */
	if (csi2->source_subdev) {
		struct v4l2_subdev_format sd_fmt = *format;

		sd_fmt.pad = 0;
		v4l2_subdev_call(csi2->source_subdev, pad, set_fmt,
				 NULL, &sd_fmt);
	}

	return 0;
}

static int meson_csi2_init_state(struct v4l2_subdev *subdev,
				 struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format format = {
		.pad = MESON_CSI2_PAD_SINK,
		.format = {
			.width = 640,
			.height = 480,
			.code = MEDIA_BUS_FMT_UYVY8_1X16,
			.field = V4L2_FIELD_NONE,
			.colorspace = V4L2_COLORSPACE_SRGB,
			.ycbcr_enc = V4L2_YCBCR_ENC_601,
			.quantization = V4L2_QUANTIZATION_LIM_RANGE,
			.xfer_func = V4L2_XFER_FUNC_SRGB,
		},
	};

	return meson_csi2_set_fmt(subdev, state, &format);
}

static const struct v4l2_subdev_pad_ops meson_csi2_pad_ops = {
	.enum_mbus_code	= meson_csi2_enum_mbus_code,
	.get_fmt	= v4l2_subdev_get_fmt,
	.set_fmt	= meson_csi2_set_fmt,
};

static const struct v4l2_subdev_video_ops meson_csi2_video_ops = {
	.s_stream	= meson_csi2_s_stream,
};

static const struct v4l2_subdev_ops meson_csi2_subdev_ops = {
	.video		= &meson_csi2_video_ops,
	.pad		= &meson_csi2_pad_ops,
};

static const struct v4l2_subdev_internal_ops meson_csi2_internal_ops = {
	.init_state	= meson_csi2_init_state,
};

static const struct media_entity_operations meson_csi2_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int meson_csi2_async_bound(struct v4l2_async_notifier *notifier,
				  struct v4l2_subdev *s_subdev,
				  struct v4l2_async_connection *asd)
{
	struct v4l2_subdev *subdev = notifier->sd;
	struct meson_csi2_priv *csi2 = v4l2_subdev_to_meson_csi2(subdev);

	csi2->source_pad = media_entity_get_fwnode_pad(&s_subdev->entity,
						       asd->match.fwnode,
						       MEDIA_PAD_FL_SOURCE);
	if (csi2->source_pad < 0) {
		dev_err(csi2->dev, "Couldn't find output pad for subdev %s\n",
			s_subdev->name);
		return csi2->source_pad;
	}

	csi2->source_subdev = s_subdev;

	dev_dbg(csi2->dev, "Bound %s pad: %d\n", s_subdev->name,
		csi2->source_pad);

	return media_create_pad_link(&csi2->source_subdev->entity,
				     csi2->source_pad,
				     &csi2->subdev.entity, 0,
				     MEDIA_LNK_FL_ENABLED |
				     MEDIA_LNK_FL_IMMUTABLE);
}

static const struct v4l2_async_notifier_operations meson_csi2_notifier_ops = {
	.bound		= meson_csi2_async_bound,
};

static int meson_csi2_parse_dt(struct meson_csi2_priv *csi2)
{
	struct v4l2_fwnode_endpoint v4l2_ep = { .bus_type = 0 };
	struct v4l2_async_connection *asd;
	struct fwnode_handle *fwh;
	struct device_node *ep;
	int ret;

	ep = of_graph_get_endpoint_by_regs(csi2->dev->of_node, 0, 0);
	if (!ep)
		return -EINVAL;

	fwh = of_fwnode_handle(ep);
	ret = v4l2_fwnode_endpoint_parse(fwh, &v4l2_ep);
	if (ret) {
		dev_err(csi2->dev, "Could not parse v4l2 endpoint\n");
		of_node_put(ep);
		return ret;
	}

	if (v4l2_ep.bus_type != V4L2_MBUS_CSI2_DPHY) {
		dev_err(csi2->dev, "Unsupported media bus type: 0x%x\n",
			v4l2_ep.bus_type);
		of_node_put(ep);
		return -EINVAL;
	}

	csi2->num_lanes = v4l2_ep.bus.mipi_csi2.num_data_lanes;
	if (csi2->num_lanes > MESON_CSI2_LANES_MAX) {
		dev_err(csi2->dev, "Unsupported number of data-lanes: %d\n",
			csi2->num_lanes);
		of_node_put(ep);
		return -EINVAL;
	}

	v4l2_async_subdev_nf_init(&csi2->notifier, &csi2->subdev);

	asd = v4l2_async_nf_add_fwnode_remote(&csi2->notifier, fwh,
					      struct v4l2_async_connection);
	of_node_put(ep);
	if (IS_ERR(asd)) {
		v4l2_async_nf_cleanup(&csi2->notifier);
		return PTR_ERR(asd);
	}

	csi2->notifier.ops = &meson_csi2_notifier_ops;

	ret = v4l2_async_nf_register(&csi2->notifier);
	if (ret)
		v4l2_async_nf_cleanup(&csi2->notifier);

	return ret;
}

static int meson_csi2_probe(struct platform_device *pdev)
{
	struct meson_csi2_priv *csi2;
	int ret, irq;

	csi2 = kzalloc(sizeof(*csi2), GFP_KERNEL);
	if (!csi2)
		return -ENOMEM;

	platform_set_drvdata(pdev, csi2);
	csi2->dev = &pdev->dev;
	mutex_init(&csi2->lock);

	csi2->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(csi2->base)) {
		ret = PTR_ERR(csi2->base);
		goto err_free_priv;
	}

	/* Register IRQ handler to prevent spurious interrupt lockups */
	irq = platform_get_irq_optional(pdev, 0);
	if (irq > 0) {
		ret = devm_request_irq(&pdev->dev, irq, meson_csi2_irq,
				       IRQF_SHARED, "meson-csi2", csi2);
		if (ret)
			dev_warn(&pdev->dev, "Failed to request IRQ: %d\n", ret);
	}

	csi2->dphy = devm_phy_get(&pdev->dev, "dphy");
	if (IS_ERR(csi2->dphy)) {
		ret = PTR_ERR(csi2->dphy);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Couldn't get D-PHY\n");
		goto err_free_priv;
	}

	ret = meson_csi2_parse_dt(csi2);
	if (ret)
		goto err_free_priv;

	csi2->subdev.owner = THIS_MODULE;
	csi2->subdev.dev = &pdev->dev;
	v4l2_subdev_init(&csi2->subdev, &meson_csi2_subdev_ops);
	csi2->subdev.internal_ops = &meson_csi2_internal_ops;
	v4l2_set_subdevdata(&csi2->subdev, &pdev->dev);
	snprintf(csi2->subdev.name, sizeof(csi2->subdev.name),
		 "%s.%s", KBUILD_MODNAME, dev_name(&pdev->dev));

	csi2->subdev.entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	csi2->pads[MESON_CSI2_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	csi2->pads[MESON_CSI2_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	csi2->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	csi2->subdev.entity.ops = &meson_csi2_media_ops;

	ret = media_entity_pads_init(&csi2->subdev.entity, MESON_CSI2_PAD_MAX,
				     csi2->pads);
	if (ret)
		goto err_cleanup;

	ret = v4l2_subdev_init_finalize(&csi2->subdev);
	if (ret)
		goto err_cleanup;

	ret = v4l2_async_register_subdev(&csi2->subdev);
	if (ret < 0)
		goto err_free_state;

	dev_info(&pdev->dev, "Probed CSI-2 receiver with %u lanes\n",
		 csi2->num_lanes);

	return 0;

err_free_state:
	v4l2_subdev_cleanup(&csi2->subdev);
err_cleanup:
	v4l2_async_nf_unregister(&csi2->notifier);
	v4l2_async_nf_cleanup(&csi2->notifier);
	media_entity_cleanup(&csi2->subdev.entity);
err_free_priv:
	kfree(csi2);
	return ret;
}

static void meson_csi2_remove(struct platform_device *pdev)
{
	struct meson_csi2_priv *csi2 = platform_get_drvdata(pdev);

	v4l2_async_nf_unregister(&csi2->notifier);
	v4l2_async_nf_cleanup(&csi2->notifier);
	v4l2_async_unregister_subdev(&csi2->subdev);
	v4l2_subdev_cleanup(&csi2->subdev);
	media_entity_cleanup(&csi2->subdev.entity);
	kfree(csi2);
}

static const struct of_device_id meson_csi2_of_table[] = {
	{ .compatible = "amlogic,g12a-csi2" },
	{ },
};
MODULE_DEVICE_TABLE(of, meson_csi2_of_table);

static struct platform_driver meson_csi2_driver = {
	.probe	= meson_csi2_probe,
	.remove = meson_csi2_remove,
	.driver	= {
		.name		= "meson-csi2",
		.of_match_table	= meson_csi2_of_table,
	},
};
module_platform_driver(meson_csi2_driver);

MODULE_AUTHOR("Da Xue <da@libre.computer>");
MODULE_DESCRIPTION("Amlogic Meson MIPI CSI-2 Receiver");
MODULE_LICENSE("GPL");
