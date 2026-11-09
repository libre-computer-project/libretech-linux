// SPDX-License-Identifier: GPL-2.0
/*
 * UCSI over the Libre Computer SCMI vendor protocol 0x80 ("lc-typec")
 *
 * The SCMI server (on ROC-RK3399-PC the BL31 Cortex-M0) owns the Type-C
 * power port and runs the PD sink policy itself; Linux sees it as a UCSI
 * v1.2 PPM whose register file is read and written through SCMI. The PPM
 * completes every command inside the UCSI_WRITE of CONTROL, so CCI is
 * final when that SCMI reply returns.
 *
 * Asynchronous connector changes are found by a 100 ms CCI poll, or by the
 * protocol's UCSI_CCI_CHANGED notification when the server advertises it.
 *
 * The driver holds no clocks: the I2C bus the server drives is kept
 * running by the clock provider itself (CLK_IS_CRITICAL), so a probe
 * error, a defer or an unbind here can never gate it.
 */

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/pm.h>
#include <linux/scmi_lc_typec.h>
#include <linux/scmi_protocol.h>
#include <linux/spinlock.h>
#include <linux/usb/typec.h>
#include <linux/workqueue.h>

#include "ucsi.h"

#define UCSI_SCMI_POLL_MS		100

/* CCI bits 7:1, the connector that has a change pending */
#define UCSI_SCMI_CCI_CONNECTOR_MASK	GENMASK(7, 1)

struct ucsi_scmi {
	struct device *dev;
	struct scmi_device *sdev;
	struct ucsi *ucsi;
	const struct scmi_lc_typec_proto_ops *ops;
	struct scmi_protocol_handle *ph;

	struct delayed_work poll;
	bool polling;
	bool poll_disabled;

	struct notifier_block nb;
	bool nb_registered;

	/* CCI value last handed to ucsi_notify_common() */
	spinlock_t cci_lock;
	u32 last_cci;
	bool last_cci_valid;

	bool yield_reported;
};

static int ucsi_scmi_read(struct ucsi_scmi *us, unsigned int offset,
			  void *val, size_t len)
{
	u8 *buf = val;
	int ret;

	while (len) {
		size_t n = min_t(size_t, len, SCMI_LC_TYPEC_READ_MAX);

		ret = us->ops->ucsi_read(us->ph, offset, n, buf);
		if (ret) {
			dev_warn_ratelimited(us->dev,
					     "UCSI_READ @%u+%zu failed: %d\n",
					     offset, n, ret);
			return ret;
		}
		offset += n;
		buf += n;
		len -= n;
	}

	return 0;
}

static int ucsi_scmi_read_version(struct ucsi *ucsi, u16 *version)
{
	struct ucsi_scmi *us = ucsi_get_drvdata(ucsi);
	__le16 v;
	int ret;

	ret = ucsi_scmi_read(us, UCSI_VERSION, &v, sizeof(v));
	if (!ret)
		*version = le16_to_cpu(v);

	return ret;
}

static int ucsi_scmi_read_cci(struct ucsi *ucsi, u32 *cci)
{
	struct ucsi_scmi *us = ucsi_get_drvdata(ucsi);
	__le32 v;
	int ret;

	ret = ucsi_scmi_read(us, UCSI_CCI, &v, sizeof(v));
	if (!ret)
		*cci = le32_to_cpu(v);

	return ret;
}

static int ucsi_scmi_read_message_in(struct ucsi *ucsi, void *val,
				     size_t val_len)
{
	struct ucsi_scmi *us = ucsi_get_drvdata(ucsi);

	return ucsi_scmi_read(us, UCSI_MESSAGE_IN, val, val_len);
}

/* Record @cci as handled; false if it equals the value handled last. */
static bool ucsi_scmi_cci_is_new(struct ucsi_scmi *us, u32 cci)
{
	unsigned long flags;
	bool changed;

	spin_lock_irqsave(&us->cci_lock, flags);
	changed = !us->last_cci_valid || us->last_cci != cci;
	us->last_cci = cci;
	us->last_cci_valid = true;
	spin_unlock_irqrestore(&us->cci_lock, flags);

	return changed;
}

static void ucsi_scmi_cci_invalidate(struct ucsi_scmi *us)
{
	unsigned long flags;

	spin_lock_irqsave(&us->cci_lock, flags);
	us->last_cci_valid = false;
	spin_unlock_irqrestore(&us->cci_lock, flags);
}

/*
 * A CCI seen outside a command (poll or notification) only ever reports a
 * connector change. Its command bits belong to whichever command ran last
 * and are delivered by async_control; forwarding them from here could
 * complete a command that has been marked pending but not yet written.
 */
static void ucsi_scmi_async_cci(struct ucsi_scmi *us, u32 cci)
{
	if (!ucsi_scmi_cci_is_new(us, cci))
		return;

	if (cci & UCSI_CCI_BUSY)
		return;

	if (cci & UCSI_SCMI_CCI_CONNECTOR_MASK)
		ucsi_notify_common(us->ucsi, cci & UCSI_SCMI_CCI_CONNECTOR_MASK);
}

static int ucsi_scmi_async_control(struct ucsi *ucsi, u64 command)
{
	struct ucsi_scmi *us = ucsi_get_drvdata(ucsi);
	__le64 cmd = cpu_to_le64(command);
	u32 cci;
	int ret;

	ret = us->ops->ucsi_write(us->ph, UCSI_CONTROL, sizeof(cmd), &cmd);
	if (ret) {
		dev_warn_ratelimited(us->dev,
				     "UCSI_WRITE CONTROL 0x%llx failed: %d\n",
				     command, ret);
		return ret;
	}

	/* The PPM completes the command inside the write: CCI is final. */
	ret = ucsi_scmi_read_cci(ucsi, &cci);
	if (ret)
		return ret;

	ucsi_scmi_cci_is_new(us, cci);
	ucsi_notify_common(ucsi, cci);

	return 0;
}

static void ucsi_scmi_update_connector(struct ucsi_connector *con)
{
	con->typec_cap.orientation_aware = true;
}

static void ucsi_scmi_connector_status(struct ucsi_connector *con)
{
	struct ucsi_scmi *us = ucsi_get_drvdata(con->ucsi);
	struct scmi_lc_typec_ext_status st;
	enum typec_orientation orientation;
	int ret;

	if (!UCSI_CONSTAT(con, CONNECTED)) {
		typec_set_orientation(con->port, TYPEC_ORIENTATION_NONE);
		return;
	}

	ret = us->ops->connector_ext_status(us->ph, con->num - 1, &st);
	if (ret) {
		dev_warn_ratelimited(us->dev,
				     "con%d: CONNECTOR_EXT_STATUS failed: %d\n",
				     con->num, ret);
		return;
	}

	if (st.owner == SCMI_LC_TYPEC_OWNER_YIELDED && !us->yield_reported) {
		us->yield_reported = true;
		dev_warn(us->dev,
			 "con%d: firmware has yielded the port (foreign I2C writer)\n",
			 con->num);
	}

	switch (st.orientation) {
	case SCMI_LC_TYPEC_ORIENT_NORMAL:
		orientation = TYPEC_ORIENTATION_NORMAL;
		break;
	case SCMI_LC_TYPEC_ORIENT_REVERSE:
		orientation = TYPEC_ORIENTATION_REVERSE;
		break;
	default:
		orientation = TYPEC_ORIENTATION_NONE;
		break;
	}

	typec_set_orientation(con->port, orientation);
}

static const struct ucsi_operations ucsi_scmi_ops = {
	.read_version = ucsi_scmi_read_version,
	.read_cci = ucsi_scmi_read_cci,
	.poll_cci = ucsi_scmi_read_cci,
	.read_message_in = ucsi_scmi_read_message_in,
	.sync_control = ucsi_sync_control_common,
	.async_control = ucsi_scmi_async_control,
	.update_connector = ucsi_scmi_update_connector,
	.connector_status = ucsi_scmi_connector_status,
};

static void ucsi_scmi_poll_work(struct work_struct *work)
{
	struct ucsi_scmi *us = container_of(to_delayed_work(work),
					    struct ucsi_scmi, poll);
	u32 cci;

	if (!ucsi_scmi_read_cci(us->ucsi, &cci))
		ucsi_scmi_async_cci(us, cci);

	schedule_delayed_work(&us->poll, msecs_to_jiffies(UCSI_SCMI_POLL_MS));
}

static int ucsi_scmi_notifier(struct notifier_block *nb, unsigned long event,
			      void *data)
{
	struct ucsi_scmi *us = container_of(nb, struct ucsi_scmi, nb);
	struct scmi_lc_typec_cci_report *r = data;

	ucsi_scmi_async_cci(us, r->cci);

	return NOTIFY_OK;
}

static void ucsi_scmi_poll_start(struct ucsi_scmi *us)
{
	if (!us->polling)
		return;

	if (us->poll_disabled) {
		enable_delayed_work(&us->poll);
		us->poll_disabled = false;
	}
	schedule_delayed_work(&us->poll, msecs_to_jiffies(UCSI_SCMI_POLL_MS));
}

/* The work re-arms itself, so it is disabled, not merely cancelled. */
static void ucsi_scmi_poll_stop(struct ucsi_scmi *us)
{
	if (!us->polling || us->poll_disabled)
		return;

	disable_delayed_work_sync(&us->poll);
	us->poll_disabled = true;
}

static int ucsi_scmi_probe(struct scmi_device *sdev)
{
	const struct scmi_handle *handle = sdev->handle;
	struct device *dev = &sdev->dev;
	struct scmi_protocol_handle *ph;
	struct ucsi_scmi *us;
	u32 num_connectors, flags;
	int ret;

	if (!handle)
		return -ENODEV;

	us = devm_kzalloc(dev, sizeof(*us), GFP_KERNEL);
	if (!us)
		return -ENOMEM;

	us->dev = dev;
	us->sdev = sdev;
	spin_lock_init(&us->cci_lock);
	INIT_DELAYED_WORK(&us->poll, ucsi_scmi_poll_work);

	us->ops = handle->devm_protocol_get(sdev, SCMI_PROTOCOL_LC_TYPEC, &ph);
	if (IS_ERR(us->ops))
		return PTR_ERR(us->ops);
	us->ph = ph;

	ret = us->ops->info_get(ph, &num_connectors, &flags);
	if (ret)
		return ret;
	if (!num_connectors)
		return dev_err_probe(dev, -ENODEV, "no connectors\n");

	us->ucsi = ucsi_create(dev, &ucsi_scmi_ops);
	if (IS_ERR(us->ucsi))
		return PTR_ERR(us->ucsi);

	ucsi_set_drvdata(us->ucsi, us);
	dev_set_drvdata(dev, us);

	if (flags & SCMI_LC_TYPEC_ATTR_NOTIFY) {
		us->nb.notifier_call = ucsi_scmi_notifier;
		ret = handle->notify_ops->event_notifier_register(handle,
				SCMI_PROTOCOL_LC_TYPEC,
				SCMI_EVENT_LC_TYPEC_UCSI_CCI_CHANGED,
				NULL, &us->nb);
		if (ret)
			dev_warn(dev, "CCI notifications unavailable (%d), polling\n",
				 ret);
		else
			us->nb_registered = true;
	}
	us->polling = !us->nb_registered;

	ret = ucsi_register(us->ucsi);
	if (ret) {
		dev_err_probe(dev, ret, "failed to register UCSI\n");
		goto err_notifier;
	}

	ucsi_scmi_poll_start(us);

	dev_info(dev, "UCSI over SCMI, %u connector(s), %s\n", num_connectors,
		 us->nb_registered ? "notified" : "polled");

	return 0;

err_notifier:
	if (us->nb_registered)
		handle->notify_ops->event_notifier_unregister(handle,
				SCMI_PROTOCOL_LC_TYPEC,
				SCMI_EVENT_LC_TYPEC_UCSI_CCI_CHANGED,
				NULL, &us->nb);
	ucsi_destroy(us->ucsi);
	return ret;
}

static void ucsi_scmi_remove(struct scmi_device *sdev)
{
	const struct scmi_handle *handle = sdev->handle;
	struct ucsi_scmi *us = dev_get_drvdata(&sdev->dev);

	/* Stop every event source before the core goes away. */
	ucsi_scmi_poll_stop(us);
	if (us->nb_registered)
		handle->notify_ops->event_notifier_unregister(handle,
				SCMI_PROTOCOL_LC_TYPEC,
				SCMI_EVENT_LC_TYPEC_UCSI_CCI_CHANGED,
				NULL, &us->nb);

	/* ucsi_unregister() still issues commands through async_control. */
	ucsi_unregister(us->ucsi);
	ucsi_destroy(us->ucsi);
}

static int ucsi_scmi_suspend(struct device *dev)
{
	struct ucsi_scmi *us = dev_get_drvdata(dev);

	ucsi_scmi_poll_stop(us);

	return 0;
}

static int ucsi_scmi_resume(struct device *dev)
{
	struct ucsi_scmi *us = dev_get_drvdata(dev);

	/* Deep S3 loses the server's state: no CCI value is "seen" any more. */
	ucsi_scmi_cci_invalidate(us);
	ucsi_scmi_poll_start(us);

	return ucsi_resume(us->ucsi);
}

static DEFINE_SIMPLE_DEV_PM_OPS(ucsi_scmi_pm_ops, ucsi_scmi_suspend,
				ucsi_scmi_resume);

static const struct scmi_device_id ucsi_scmi_id_table[] = {
	{ SCMI_PROTOCOL_LC_TYPEC, "lc-typec" },
	{ },
};
MODULE_DEVICE_TABLE(scmi, ucsi_scmi_id_table);

static struct scmi_driver ucsi_scmi_driver = {
	.name = "ucsi-scmi",
	.probe = ucsi_scmi_probe,
	.remove = ucsi_scmi_remove,
	.id_table = ucsi_scmi_id_table,
	.driver = {
		.pm = pm_sleep_ptr(&ucsi_scmi_pm_ops),
	},
};
module_scmi_driver(ucsi_scmi_driver);

MODULE_DESCRIPTION("UCSI driver for the Libre Computer SCMI Type-C protocol");
MODULE_LICENSE("GPL");
