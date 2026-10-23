// SPDX-License-Identifier: GPL-2.0
/*
 * Rockchip RK3399 M0 I2C offload service
 *
 * The Cortex-M0 SCMI server living in the BL31 firmware needs a handful
 * of RK808 register accesses to service CPU OPP voltage requests. Linux
 * owns the I2C0 bus that the PMIC hangs off, so the M0 does not touch the
 * bus itself: it posts one request at a time into a shared DRAM page,
 * rings mailbox1 channel 1 (A2B doorbell), and waits for the answer in
 * the same page.
 *
 * This service answers those requests. The request path is:
 *
 *	M0 wakes on the SCMI voltage message, finds a rail value it must
 *	not compute locally, writes REQ_* and REQ_CTRL with PENDING and a
 *	new sequence number, rings channel 1, and goes back to sleep with
 *	the SCMI message still held open.
 *
 * The answer path is this driver's work function: run the I2C transfer on
 * the PMIC's adapter, write RESP_VAL/RESP_STATUS, then RESP_CTRL with
 * DONE and the echoed sequence number, and ring channel 1 back so the M0
 * wakes immediately (it also has a wake cap that aborts a request whose
 * answer never lands, so a lost ring costs one aborted message, not a
 * deadlock).
 *
 * Shared page layout (little-endian u32, M0 and kernel agree on it):
 *
 *	0x00 FLAGS	 kernel-owned: SVC_ON bit0, QUIESCE bit1
 *	0x04 REQ_CTRL	 M0-owned: PENDING bit31, seq[15:0]
 *	0x08 REQ_CMD	 0 = register read, 1 = register write
 *	0x0c REQ_DEV	 I2C slave address pre-shifted (slave << 1)
 *	0x10 REQ_REG	 PMIC register offset
 *	0x14 REQ_VAL	 write value (reads ignore it)
 *	0x18 RESP_CTRL	 kernel-owned: DONE bit31, echoed seq[15:0]
 *	0x1c RESP_STATUS 0 = ok, else a negative kernel errno
 *	0x20 RESP_VAL	 read value (writes ignore it)
 *
 * PENDING is the M0's ownership bit and only the M0 clears it; DONE with
 * a matching sequence number is this driver's answer and a stale DONE can
 * never be mistaken for a fresh one. Only one request is in flight at a
 * time, guaranteed by the M0 holding its SCMI message open until DONE.
 */

#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/slab.h>

#define OFF_FLAGS		0x00
#define  OFF_SVC_ON		BIT(0)
#define  OFF_QUIESCE		BIT(1)
#define OFF_REQ_CTRL		0x04
#define  OFF_REQ_PENDING	BIT(31)
#define  OFF_REQ_SEQ_MASK	GENMASK(15, 0)
#define OFF_REQ_CMD		0x08
#define OFF_REQ_DEV		0x0c
#define OFF_REQ_REG		0x10
#define OFF_REQ_VAL		0x14
#define OFF_RESP_CTRL		0x18
#define  OFF_RESP_DONE		BIT(31)
#define OFF_RESP_STATUS		0x1c
#define OFF_RESP_VAL		0x20

#define OFF_CMD_READ		0
#define OFF_CMD_WRITE		1

struct rockchip_m0_i2c_offload {
	struct device *dev;
	void __iomem *regs;
	struct mbox_client client;
	struct mbox_chan *chan;
	struct i2c_client *target;
	struct work_struct work;
};

static void rockchip_m0_i2c_offload_rx(struct mbox_client *client, void *data)
{
	struct rockchip_m0_i2c_offload *off =
		container_of(client, struct rockchip_m0_i2c_offload, client);

	schedule_work(&off->work);
}

static void rockchip_m0_i2c_offload_answer(struct rockchip_m0_i2c_offload *off,
					   u32 ctrl, int status, u32 val)
{
	writel(val, off->regs + OFF_RESP_VAL);
	writel(status, off->regs + OFF_RESP_STATUS);
	/* Payload and status must be visible before DONE. */
	wmb();
	writel(OFF_RESP_DONE | (ctrl & OFF_REQ_SEQ_MASK),
	       off->regs + OFF_RESP_CTRL);
	wmb();

	/*
	 * Ring channel 1 so the M0 stops waiting on its wake cap. The
	 * doorbell is complete the moment it is written, so tell the
	 * mailbox core as well and the channel is free for the next one.
	 */
	if (!mbox_send_message(off->chan, NULL))
		mbox_client_txdone(off->chan, 0);
}

static void rockchip_m0_i2c_offload_work(struct work_struct *work)
{
	struct rockchip_m0_i2c_offload *off =
		container_of(work, struct rockchip_m0_i2c_offload, work);
	struct i2c_client *target = off->target;
	u8 buf[2];
	u32 ctrl, resp, cmd, dev;
	int status, ret;

	ctrl = readl(off->regs + OFF_REQ_CTRL);
	if (!(ctrl & OFF_REQ_PENDING))
		return;

	/* A spurious or duplicate ring must not run the bus twice. */
	resp = readl(off->regs + OFF_RESP_CTRL);
	if ((resp & OFF_RESP_DONE) &&
	    (resp & OFF_REQ_SEQ_MASK) == (ctrl & OFF_REQ_SEQ_MASK))
		return;

	cmd = readl(off->regs + OFF_REQ_CMD);
	dev = readl(off->regs + OFF_REQ_DEV);
	buf[0] = readl(off->regs + OFF_REQ_REG);

	if (cmd > OFF_CMD_WRITE || dev != (u32)(target->addr << 1)) {
		rockchip_m0_i2c_offload_answer(off, ctrl, -EINVAL, 0);
		return;
	}

	if (cmd == OFF_CMD_WRITE) {
		struct i2c_msg msg = {
			.addr = target->addr,
			.len = 2,
			.buf = buf,
		};

		buf[1] = readl(off->regs + OFF_REQ_VAL);
		ret = i2c_transfer(target->adapter, &msg, 1);
		status = ret == 1 ? 0 : (ret < 0 ? ret : -EIO);
		rockchip_m0_i2c_offload_answer(off, ctrl, status, 0);
		return;
	} else {
		struct i2c_msg msgs[2] = {
			{
				.addr = target->addr,
				.len = 1,
				.buf = buf,
			},
			{
				.addr = target->addr,
				.flags = I2C_M_RD,
				.len = 1,
				.buf = &buf[1],
			},
		};

		ret = i2c_transfer(target->adapter, msgs, 2);
		status = ret == 2 ? 0 : (ret < 0 ? ret : -EIO);
		rockchip_m0_i2c_offload_answer(off, ctrl, status,
					       ret == 2 ? buf[1] : 0);
		return;
	}
}

static int rockchip_m0_i2c_offload_suspend(struct device *dev)
{
	struct rockchip_m0_i2c_offload *off = dev_get_drvdata(dev);

	/*
	 * Tell the M0 not to post anything new, then let the request
	 * already in flight finish its transfer while the I2C controller
	 * is still up. A request posted after QUIESCE is refused inside
	 * the M0 and never reaches this driver.
	 */
	writel(OFF_SVC_ON | OFF_QUIESCE, off->regs + OFF_FLAGS);
	wmb();
	flush_work(&off->work);

	return 0;
}

static int rockchip_m0_i2c_offload_resume(struct device *dev)
{
	struct rockchip_m0_i2c_offload *off = dev_get_drvdata(dev);

	writel(OFF_SVC_ON, off->regs + OFF_FLAGS);

	return 0;
}

static const struct dev_pm_ops rockchip_m0_i2c_offload_pm = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(rockchip_m0_i2c_offload_suspend,
				     rockchip_m0_i2c_offload_resume)
};

static const struct of_device_id rockchip_m0_i2c_offload_of_match[] = {
	{ .compatible = "rockchip,rk3399-m0-i2c-offload" },
	{ }
};
MODULE_DEVICE_TABLE(of, rockchip_m0_i2c_offload_of_match);

static int rockchip_m0_i2c_offload_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *target_np;
	struct rockchip_m0_i2c_offload *off;
	struct i2c_client *target;

	off = devm_kzalloc(dev, sizeof(*off), GFP_KERNEL);
	if (!off)
		return -ENOMEM;

	off->dev = dev;
	off->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(off->regs))
		return PTR_ERR(off->regs);

	target_np = of_parse_phandle(dev->of_node, "rockchip,pwrctrl", 0);
	if (!target_np) {
		dev_err(dev, "missing rockchip,pwrctrl phandle\n");
		return -EINVAL;
	}
	target = of_find_i2c_device_by_node(target_np);
	of_node_put(target_np);
	if (!target)
		return -EPROBE_DEFER;
	off->target = target;

	off->client.dev = dev;
	off->client.rx_callback = rockchip_m0_i2c_offload_rx;
	INIT_WORK(&off->work, rockchip_m0_i2c_offload_work);

	platform_set_drvdata(pdev, off);

	off->chan = mbox_request_channel(&off->client, 0);
	if (IS_ERR(off->chan))
		return dev_err_probe(dev, PTR_ERR(off->chan),
				     "failed to request offload mailbox\n");

	/*
	 * The page survives warm reboots, so start from a clean slate:
	 * service off, no answer claimed. SVC_ON goes up last -- the M0
	 * refuses to offload (and answers NOT_SUPPORTED) until it sees it.
	 */
	writel(0, off->regs + OFF_FLAGS);
	writel(0, off->regs + OFF_RESP_CTRL);
	wmb();
	writel(OFF_SVC_ON, off->regs + OFF_FLAGS);

	dev_info(dev, "serving M0 I2C offload on %s addr 0x%02x\n",
		 dev_name(&target->dev), target->addr);

	return 0;
}

static void rockchip_m0_i2c_offload_remove(struct platform_device *pdev)
{
	struct rockchip_m0_i2c_offload *off = platform_get_drvdata(pdev);

	writel(OFF_SVC_ON | OFF_QUIESCE, off->regs + OFF_FLAGS);
	wmb();
	flush_work(&off->work);
	writel(0, off->regs + OFF_FLAGS);
	mbox_free_channel(off->chan);
}

static struct platform_driver rockchip_m0_i2c_offload_driver = {
	.driver = {
		.name = "rockchip-m0-i2c-offload",
		.of_match_table = rockchip_m0_i2c_offload_of_match,
		.pm = pm_sleep_ptr(&rockchip_m0_i2c_offload_pm),
	},
	.probe = rockchip_m0_i2c_offload_probe,
	.remove = rockchip_m0_i2c_offload_remove,
};
module_platform_driver(rockchip_m0_i2c_offload_driver);

MODULE_AUTHOR("Libre Computer");
MODULE_DESCRIPTION("Rockchip RK3399 M0 I2C offload service");
MODULE_LICENSE("GPL");
