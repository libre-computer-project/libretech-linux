// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek MT8188 HDMI CEC controller.
 *
 * Ported from the MediaTek Genio BSP kernel (mtk-v6.6)
 * drivers/gpu/drm/mediatek/mtk_cec_v2.c (Tommy Chen, Can Zeng): the register
 * programming and the 27 MHz timing values are the vendor's. The controller is
 * its own device beside the HDMI transmitter; the HDMI side reports the sink's
 * physical address through the CEC notifier.
 *
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <media/cec.h>
#include <media/cec-notifier.h>

#include "mtk-cec.h"

static const char * const mtk_cec_clk_names[] = {
	"cec_66m_h", "cec_66m_b", "hdmi_32k", "hdmi_26m",
};

struct mtk_cec {
	struct device *dev;
	void __iomem *regs;
	struct clk_bulk_data clks[ARRAY_SIZE(mtk_cec_clk_names)];
	struct cec_adapter *adap;
	struct cec_notifier *notifier;
	struct cec_msg rx_msg;
	bool enabled;
	bool tx_broadcast;
	u32 prev_tx_fsm;
};

static u32 mtk_cec_read(struct mtk_cec *cec, u32 reg)
{
	return readl(cec->regs + reg);
}

static void mtk_cec_write(struct mtk_cec *cec, u32 reg, u32 val)
{
	writel(val, cec->regs + reg);
}

static void mtk_cec_mask(struct mtk_cec *cec, u32 reg, u32 val, u32 mask)
{
	u32 tmp = mtk_cec_read(cec, reg) & ~mask;

	mtk_cec_write(cec, reg, tmp | (val & mask));
}

/* An INT_CLR bit is a pulse: set it, then clear it. */
static void mtk_cec_int_clear(struct mtk_cec *cec, u32 bits)
{
	mtk_cec_mask(cec, CEC2_INT_CLR, bits, bits);
	mtk_cec_mask(cec, CEC2_INT_CLR, 0, bits);
}

static void mtk_cec_rx_reset(struct mtk_cec *cec)
{
	mtk_cec_mask(cec, CEC2_TR_CONFIG, 0, CEC2_RX_RESET_WRITE);
	mtk_cec_mask(cec, CEC2_TR_CONFIG, CEC2_RX_RESET_WRITE, CEC2_RX_RESET_WRITE);
}

static void mtk_cec_tx_reset(struct mtk_cec *cec)
{
	mtk_cec_mask(cec, CEC2_TR_CONFIG, 0, CEC2_TX_RESET_WRITE);
	mtk_cec_mask(cec, CEC2_TR_CONFIG, CEC2_TX_RESET_WRITE, CEC2_TX_RESET_WRITE);
}

static u32 mtk_cec_tx_fsm(struct mtk_cec *cec)
{
	return FIELD_GET(CEC2_TX_FSM, mtk_cec_read(cec, CEC2_TX_STATUS));
}

static void mtk_cec_hw_init(struct mtk_cec *cec)
{
	/* 27 MHz source divided to the CEC bit clock */
	mtk_cec_mask(cec, CEC2_CKGEN, CEC2_CLK_27M_EN, CEC2_CLK_27M_EN);
	mtk_cec_mask(cec, CEC2_CKGEN, CEC2_DIV_SEL_100K, CEC2_CLK_DIV);
	mtk_cec_mask(cec, CEC2_CKGEN, 0, CEC2_CLK_SEL_DIV);

	/* accept only frames addressed to us */
	mtk_cec_mask(cec, CEC2_TR_CONFIG, CEC2_RX_CHK_DST, CEC2_RX_CHK_DST);
	mtk_cec_mask(cec, CEC2_TR_CONFIG, 0, CEC2_BYPASS);

	/* RX timing at 27 MHz */
	mtk_cec_write(cec, CEC2_RX_TIMER_START_R, 0x0186015e);
	mtk_cec_write(cec, CEC2_RX_TIMER_START_F, 0x01d601ae);
	mtk_cec_write(cec, CEC2_RX_TIMER_DATA, 0x011300cd);
	mtk_cec_write(cec, CEC2_RX_TIMER_ACK, 0x00690082);
	mtk_cec_write(cec, CEC2_RX_TIMER_ERROR, 0x01680000);
	mtk_cec_mask(cec, CEC2_TR_CONFIG, CEC2_RX_ACK_ERROR_BYPASS,
		     CEC2_RX_ACK_ERROR_BYPASS);
	mtk_cec_rx_reset(cec);

	/* TX timing at 27 MHz */
	mtk_cec_write(cec, CEC2_TX_TIMER_START, 0x01c20172);
	mtk_cec_write(cec, CEC2_TX_TIMER_DATA_R, 0x003c0096);
	mtk_cec_write(cec, CEC2_TX_T_DATA_F, 0x00f000f0);
	mtk_cec_write(cec, CEC2_TX_TIMER_DATA_S, 0x00040069);
	mtk_cec_write(cec, CEC2_LINE_DET, 0x1f4004b0);
	mtk_cec_mask(cec, CEC2_TX_TIMER_ACK, 0x00aa0028,
		     CEC2_TX_TIMER_ACK_MAX | CEC2_TX_TIMER_ACK_MIN);

	/* one retry per failure class; the CEC core retries on top */
	mtk_cec_mask(cec, CEC2_TX_ARB,
		     FIELD_PREP(CEC2_TX_MAX_RETRANSMIT_NUM_ARB, 1) |
		     FIELD_PREP(CEC2_TX_MAX_RETRANSMIT_NUM_COL, 1) |
		     FIELD_PREP(CEC2_TX_MAX_RETRANSMIT_NUM_NAK, 1) |
		     FIELD_PREP(CEC2_TX_BCNT_RETRANSMIT, 4) |
		     FIELD_PREP(CEC2_TX_BCNT_NEW_MSG, 7) |
		     FIELD_PREP(CEC2_TX_BCNT_NEW_INIT, 5),
		     CEC2_TX_MAX_RETRANSMIT_NUM_ARB |
		     CEC2_TX_MAX_RETRANSMIT_NUM_COL |
		     CEC2_TX_MAX_RETRANSMIT_NUM_NAK |
		     CEC2_TX_BCNT_RETRANSMIT | CEC2_TX_BCNT_NEW_MSG |
		     CEC2_TX_BCNT_NEW_INIT);
	mtk_cec_mask(cec, CEC2_CKGEN, CEC2_CLK_TX_EN, CEC2_CLK_TX_EN);
	mtk_cec_tx_reset(cec);

	/* all interrupts cleared; RX on (without FSM change), TX armed per frame */
	mtk_cec_write(cec, CEC2_INT_CLR, 0xffffffff);
	mtk_cec_write(cec, CEC2_INT_CLR, 0);
	mtk_cec_write(cec, CEC2_INT_EN, CEC2_RX_INT_ALL & ~CEC2_RX_FSM_CHG_INT);
}

static int mtk_cec_adap_enable(struct cec_adapter *adap, bool enable)
{
	struct mtk_cec *cec = cec_get_drvdata(adap);
	int ret;

	if (enable == cec->enabled)
		return 0;

	if (enable) {
		ret = clk_bulk_prepare_enable(ARRAY_SIZE(cec->clks), cec->clks);
		if (ret)
			return ret;
		mtk_cec_mask(cec, CEC2_CKGEN, 0, CEC2_CLK_PDN);
		mtk_cec_hw_init(cec);
	} else {
		mtk_cec_write(cec, CEC2_INT_EN, 0);
		mtk_cec_mask(cec, CEC2_CKGEN, CEC2_CLK_PDN, CEC2_CLK_PDN);
		clk_bulk_disable_unprepare(ARRAY_SIZE(cec->clks), cec->clks);
	}
	cec->enabled = enable;
	return 0;
}

static int mtk_cec_adap_log_addr(struct cec_adapter *adap, u8 logical_addr)
{
	struct mtk_cec *cec = cec_get_drvdata(adap);

	/* the controller matches up to three addresses; the core claims one */
	if (logical_addr == CEC_LOG_ADDR_INVALID)
		logical_addr = CEC_LOG_ADDR_UNREGISTERED;
	mtk_cec_mask(cec, CEC2_TR_CONFIG,
		     FIELD_PREP(CEC2_DEVICE_ADDR1, logical_addr),
		     CEC2_DEVICE_ADDR1);
	return 0;
}

static int mtk_cec_adap_transmit(struct cec_adapter *adap, u8 attempts,
				 u32 signal_free_time, struct cec_msg *msg)
{
	struct mtk_cec *cec = cec_get_drvdata(adap);
	unsigned int len = msg->len - 1;
	unsigned int i;

	if (mtk_cec_tx_fsm(cec) != CEC2_TX_FSM_IDLE)
		mtk_cec_tx_reset(cec);

	mtk_cec_mask(cec, CEC2_TX_HEADER,
		     FIELD_PREP(CEC2_TX_HEADER_SRC, cec_msg_initiator(msg)) |
		     FIELD_PREP(CEC2_TX_HEADER_DST, cec_msg_destination(msg)),
		     CEC2_TX_HEADER_SRC | CEC2_TX_HEADER_DST);

	mtk_cec_mask(cec, CEC2_INT_EN, 0, CEC2_TX_INT_ALL);
	mtk_cec_int_clear(cec, CEC2_TX_INT_ALL);

	if (!len) {
		mtk_cec_mask(cec, CEC2_TX_HEADER, CEC2_TX_HEADER_EOM, CEC2_TX_HEADER_EOM);
	} else {
		mtk_cec_mask(cec, CEC2_TX_HEADER, 0, CEC2_TX_HEADER_EOM);
		mtk_cec_mask(cec, CEC2_TX_HEADER, FIELD_PREP(CEC2_TX_SEND_CNT, len),
			     CEC2_TX_SEND_CNT);
		for (i = 0; i < len; i++) {
			u32 shift = (i % 4) * 8;

			mtk_cec_mask(cec, CEC2_TX_DATA0 + (i / 4) * 4,
				     (u32)msg->msg[i + 1] << shift, 0xffu << shift);
		}
	}
	cec->tx_broadcast = cec_msg_is_broadcast(msg);

	mtk_cec_mask(cec, CEC2_INT_EN,
		     CEC2_TX_DATA_FINISH_INT | CEC2_LINE_LOW_LONG_INT |
		     CEC2_TX_FAIL_RETRANSMIT_INT,
		     CEC2_TX_DATA_FINISH_INT | CEC2_LINE_LOW_LONG_INT |
		     CEC2_TX_FAIL_RETRANSMIT_INT);
	mtk_cec_mask(cec, CEC2_TX_HEADER, CEC2_TX_READY, CEC2_TX_READY);
	return 0;
}

static void mtk_cec_rx_irq(struct mtk_cec *cec, u32 sta)
{
	struct cec_msg *msg = &cec->rx_msg;
	u32 hdr;
	unsigned int len, i;

	if (sta & CEC2_RX_BUF_READY_INT) {
		hdr = mtk_cec_read(cec, CEC2_RX_BUF_HEADER);
		len = (hdr & CEC2_RX_HEADER_EOM) ? 0 : FIELD_GET(CEC2_RX_BUF_CNT, hdr);

		memset(msg, 0, sizeof(*msg));
		msg->msg[0] = FIELD_GET(CEC2_RX_HEADER_SRC, hdr) << 4 |
			      FIELD_GET(CEC2_RX_HEADER_DST, hdr);
		msg->len = len + 1;
		for (i = 0; i < len; i++)
			msg->msg[i + 1] = mtk_cec_read(cec, CEC2_RX_DATA0 + (i / 4) * 4) >>
					  ((i % 4) * 8);
		mtk_cec_mask(cec, CEC2_RX_BUF_HEADER, CEC2_RX_BUF_RISC_ACK,
			     CEC2_RX_BUF_RISC_ACK);
		/* a polling message is acked by the hardware and not reported */
		if (len)
			cec_received_msg(cec->adap, msg);
	}
	if (sta & CEC2_RX_BUF_FULL_INT)
		dev_dbg(cec->dev, "rx buffer overflow\n");

	mtk_cec_int_clear(cec, CEC2_RX_INT_ALL);
}

static void mtk_cec_tx_done(struct mtk_cec *cec, u8 status)
{
	mtk_cec_mask(cec, CEC2_INT_EN, 0, CEC2_TX_INT_ALL);
	mtk_cec_int_clear(cec, CEC2_TX_INT_ALL);
	cec_transmit_attempt_done(cec->adap, status);
}

static void mtk_cec_tx_irq(struct mtk_cec *cec, u32 sta)
{
	u32 fsm = mtk_cec_tx_fsm(cec);

	/* a start-bit error: the FSM dropped from 0x20 to 0x08 */
	if (fsm == 0x08 && cec->prev_tx_fsm == 0x20) {
		cec->prev_tx_fsm = fsm;
		mtk_cec_tx_reset(cec);
		mtk_cec_tx_done(cec, CEC_TX_STATUS_ERROR);
		return;
	}
	cec->prev_tx_fsm = fsm;

	if (sta & CEC2_LINE_LOW_LONG_INT) {
		mtk_cec_tx_reset(cec);
		mtk_cec_tx_done(cec, CEC_TX_STATUS_LOW_DRIVE);
		return;
	}

	if (sta & CEC2_TX_FAIL_RETRANSMIT_INT) {
		bool nack = sta & (CEC2_TX_FAIL_HEADER_ACK_INT | CEC2_TX_FAIL_DATA_ACK_INT);

		/* an unacked broadcast reached everyone it could */
		if (cec->tx_broadcast)
			mtk_cec_tx_done(cec, nack ? CEC_TX_STATUS_OK : CEC_TX_STATUS_ERROR);
		else
			mtk_cec_tx_done(cec, nack ? CEC_TX_STATUS_NACK : CEC_TX_STATUS_ERROR);
		return;
	}

	if (sta & CEC2_TX_DATA_FINISH_INT)
		mtk_cec_tx_done(cec, CEC_TX_STATUS_OK);
}

static irqreturn_t mtk_cec_irq_thread(int irq, void *data)
{
	struct mtk_cec *cec = data;
	u32 sta = mtk_cec_read(cec, CEC2_INT_STA);
	u32 en = mtk_cec_read(cec, CEC2_INT_EN);

	if (!(sta & en))
		return IRQ_NONE;

	mtk_cec_rx_irq(cec, sta);
	if (sta & en & CEC2_TX_INT_ALL)
		mtk_cec_tx_irq(cec, sta);
	return IRQ_HANDLED;
}

static const struct cec_adap_ops mtk_cec_adap_ops = {
	.adap_enable = mtk_cec_adap_enable,
	.adap_log_addr = mtk_cec_adap_log_addr,
	.adap_transmit = mtk_cec_adap_transmit,
};

static int mtk_cec_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device *hdmi_dev;
	struct mtk_cec *cec;
	unsigned int i;
	int irq, ret;

	hdmi_dev = cec_notifier_parse_hdmi_phandle(dev);
	if (IS_ERR(hdmi_dev))
		return PTR_ERR(hdmi_dev);

	cec = devm_kzalloc(dev, sizeof(*cec), GFP_KERNEL);
	if (!cec)
		return -ENOMEM;
	cec->dev = dev;
	platform_set_drvdata(pdev, cec);

	cec->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(cec->regs))
		return PTR_ERR(cec->regs);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	for (i = 0; i < ARRAY_SIZE(mtk_cec_clk_names); i++)
		cec->clks[i].id = mtk_cec_clk_names[i];
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(cec->clks), cec->clks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get clocks\n");

	cec->adap = cec_allocate_adapter(&mtk_cec_adap_ops, cec, "mtk-cec",
					 CEC_CAP_DEFAULTS | CEC_CAP_CONNECTOR_INFO, 1);
	if (IS_ERR(cec->adap))
		return PTR_ERR(cec->adap);

	ret = devm_request_threaded_irq(dev, irq, NULL, mtk_cec_irq_thread,
					IRQF_ONESHOT, dev_name(dev), cec);
	if (ret)
		goto err_delete_adapter;

	cec->notifier = cec_notifier_cec_adap_register(hdmi_dev, NULL, cec->adap);
	if (!cec->notifier) {
		ret = -ENOMEM;
		goto err_delete_adapter;
	}

	ret = cec_register_adapter(cec->adap, dev);
	if (ret < 0)
		goto err_notifier;

	return 0;

err_notifier:
	cec_notifier_cec_adap_unregister(cec->notifier, cec->adap);
err_delete_adapter:
	cec_delete_adapter(cec->adap);
	return ret;
}

static void mtk_cec_remove(struct platform_device *pdev)
{
	struct mtk_cec *cec = platform_get_drvdata(pdev);

	cec_notifier_cec_adap_unregister(cec->notifier, cec->adap);
	cec_unregister_adapter(cec->adap);
}

static const struct of_device_id mtk_cec_of_match[] = {
	{ .compatible = "mediatek,mt8188-cec" },
	{ }
};
MODULE_DEVICE_TABLE(of, mtk_cec_of_match);

static struct platform_driver mtk_cec_driver = {
	.probe = mtk_cec_probe,
	.remove = mtk_cec_remove,
	.driver = {
		.name = "mtk-cec",
		.of_match_table = mtk_cec_of_match,
	},
};
module_platform_driver(mtk_cec_driver);

MODULE_DESCRIPTION("MediaTek MT8188 HDMI CEC driver");
MODULE_LICENSE("GPL");
