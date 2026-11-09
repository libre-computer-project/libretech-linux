// SPDX-License-Identifier: GPL-2.0
/*
 * SCMI vendor protocol 0x80 "lc-typec" (Libre Computer)
 *
 * The RK3399 Cortex-M0 SCMI server owns the rear Type-C power port and
 * exposes it as a UCSI v1.2 PPM. This protocol carries the UCSI register
 * file over SCMI. Every message is answered from the server's RAM state:
 * the A2P channel is shared with clock and reset, so no request waits on
 * I2C or on the PD partner.
 *
 *   id 0  PROTOCOL_VERSION
 *   id 1  PROTOCOL_ATTRIBUTES         -> num_connectors, flags
 *   id 2  PROTOCOL_MESSAGE_ATTRIBUTES
 *   id 3  UCSI_READ   offset, len     -> bytes (len <= 32)
 *   id 4  UCSI_WRITE  offset, len, bytes (len <= 16)
 *   id 5  CONNECTOR_EXT_STATUS conn   -> orientation, mV, mA, pd_rev, owner
 *   id 6  NOTIFY_ENABLE enable
 *   P2A 0 UCSI_CCI_CHANGED            -> cci
 */

#define pr_fmt(fmt) "SCMI lc-typec - " fmt

#include <linux/bits.h>
#include <linux/module.h>
#include <linux/scmi_protocol.h>
#include <linux/scmi_lc_typec.h>
#include <linux/unaligned.h>

#include "../../protocols.h"
#include "../../notify.h"

#define SCMI_PROTOCOL_SUPPORTED_VERSION		0x10000

enum scmi_lc_typec_protocol_cmd {
	LC_TYPEC_UCSI_READ = 0x3,
	LC_TYPEC_UCSI_WRITE = 0x4,
	LC_TYPEC_CONNECTOR_EXT_STATUS = 0x5,
	LC_TYPEC_NOTIFY_ENABLE = 0x6,
};

struct scmi_lc_typec_info {
	u32 num_connectors;
	u32 flags;
};

struct scmi_msg_lc_typec_attributes {
	__le32 num_connectors;
	__le32 flags;
};

struct scmi_msg_lc_typec_ucsi_read {
	__le32 offset;
	__le32 len;
};

struct scmi_msg_lc_typec_ucsi_write {
	__le32 offset;
	__le32 len;
	u8 data[SCMI_LC_TYPEC_WRITE_MAX];
};

struct scmi_msg_resp_lc_typec_ext_status {
	__le32 orientation;
	__le32 contract_mv;
	__le32 contract_ma;
	__le32 pd_rev;
	__le32 owner;
};

struct scmi_lc_typec_cci_payld {
	__le32 cci;
};

static int scmi_lc_typec_attributes_get(const struct scmi_protocol_handle *ph,
					struct scmi_lc_typec_info *pi)
{
	struct scmi_msg_lc_typec_attributes *attr;
	struct scmi_xfer *t;
	int ret;

	ret = ph->xops->xfer_get_init(ph, PROTOCOL_ATTRIBUTES, 0,
				      sizeof(*attr), &t);
	if (ret)
		return ret;

	attr = t->rx.buf;

	ret = ph->xops->do_xfer(ph, t);
	if (!ret) {
		pi->num_connectors = le32_to_cpu(attr->num_connectors);
		pi->flags = le32_to_cpu(attr->flags);
	}

	ph->xops->xfer_put(ph, t);

	return ret;
}

static int scmi_lc_typec_info_get(const struct scmi_protocol_handle *ph,
				  u32 *num_connectors, u32 *flags)
{
	struct scmi_lc_typec_info *pi = ph->get_priv(ph);

	if (num_connectors)
		*num_connectors = pi->num_connectors;
	if (flags)
		*flags = pi->flags;

	return 0;
}

static int scmi_lc_typec_ucsi_read(const struct scmi_protocol_handle *ph,
				   u32 offset, u32 len, void *buf)
{
	struct scmi_msg_lc_typec_ucsi_read *req;
	struct scmi_xfer *t;
	int ret;

	if (!len || len > SCMI_LC_TYPEC_READ_MAX)
		return -EINVAL;

	ret = ph->xops->xfer_get_init(ph, LC_TYPEC_UCSI_READ, sizeof(*req),
				      len, &t);
	if (ret)
		return ret;

	req = t->tx.buf;
	req->offset = cpu_to_le32(offset);
	req->len = cpu_to_le32(len);

	ret = ph->xops->do_xfer(ph, t);
	if (!ret) {
		/* The transport trims rx.len to what the server returned. */
		if (t->rx.len != len)
			ret = -EPROTO;
		else
			memcpy(buf, t->rx.buf, len);
	}

	ph->xops->xfer_put(ph, t);

	return ret;
}

static int scmi_lc_typec_ucsi_write(const struct scmi_protocol_handle *ph,
				    u32 offset, u32 len, const void *buf)
{
	struct scmi_msg_lc_typec_ucsi_write *req;
	struct scmi_xfer *t;
	int ret;

	if (!len || len > SCMI_LC_TYPEC_WRITE_MAX)
		return -EINVAL;

	ret = ph->xops->xfer_get_init(ph, LC_TYPEC_UCSI_WRITE,
				      offsetof(typeof(*req), data) + len, 0,
				      &t);
	if (ret)
		return ret;

	req = t->tx.buf;
	req->offset = cpu_to_le32(offset);
	req->len = cpu_to_le32(len);
	memcpy(req->data, buf, len);

	ret = ph->xops->do_xfer(ph, t);

	ph->xops->xfer_put(ph, t);

	return ret;
}

static int
scmi_lc_typec_connector_ext_status(const struct scmi_protocol_handle *ph,
				   u32 conn,
				   struct scmi_lc_typec_ext_status *st)
{
	struct scmi_lc_typec_info *pi = ph->get_priv(ph);
	struct scmi_msg_resp_lc_typec_ext_status *resp;
	struct scmi_xfer *t;
	int ret;

	if (conn >= pi->num_connectors)
		return -EINVAL;

	ret = ph->xops->xfer_get_init(ph, LC_TYPEC_CONNECTOR_EXT_STATUS,
				      sizeof(__le32), sizeof(*resp), &t);
	if (ret)
		return ret;

	put_unaligned_le32(conn, t->tx.buf);

	ret = ph->xops->do_xfer(ph, t);
	if (!ret) {
		resp = t->rx.buf;
		st->orientation = le32_to_cpu(resp->orientation);
		st->contract_mv = le32_to_cpu(resp->contract_mv);
		st->contract_ma = le32_to_cpu(resp->contract_ma);
		st->pd_rev = le32_to_cpu(resp->pd_rev);
		st->owner = le32_to_cpu(resp->owner);
	}

	ph->xops->xfer_put(ph, t);

	return ret;
}

static int scmi_lc_typec_notify_enable(const struct scmi_protocol_handle *ph,
				       bool enable)
{
	struct scmi_xfer *t;
	int ret;

	ret = ph->xops->xfer_get_init(ph, LC_TYPEC_NOTIFY_ENABLE,
				      sizeof(__le32), 0, &t);
	if (ret)
		return ret;

	put_unaligned_le32(enable ? 1 : 0, t->tx.buf);

	ret = ph->xops->do_xfer(ph, t);

	ph->xops->xfer_put(ph, t);

	return ret;
}

static int scmi_lc_typec_set_notify_enabled(const struct scmi_protocol_handle *ph,
					    u8 evt_id, u32 src_id, bool enable)
{
	int ret;

	if (evt_id != SCMI_EVENT_LC_TYPEC_UCSI_CCI_CHANGED)
		return -EINVAL;

	ret = scmi_lc_typec_notify_enable(ph, enable);
	if (ret)
		pr_debug("FAIL_ENABLED - evt[%X] dom[%d] - ret:%d\n",
			 evt_id, src_id, ret);

	return ret;
}

static void *
scmi_lc_typec_fill_custom_report(const struct scmi_protocol_handle *ph,
				 u8 evt_id, ktime_t timestamp,
				 const void *payld, size_t payld_sz,
				 void *report, u32 *src_id)
{
	const struct scmi_lc_typec_cci_payld *p = payld;
	struct scmi_lc_typec_cci_report *r = report;

	if (evt_id != SCMI_EVENT_LC_TYPEC_UCSI_CCI_CHANGED ||
	    payld_sz != sizeof(*p))
		return NULL;

	r->timestamp = timestamp;
	r->cci = le32_to_cpu(p->cci);
	*src_id = 0;

	return r;
}

/* The P2A message id equals the event's index in this table. */
static const struct scmi_event scmi_lc_typec_events[] = {
	{
		.id = SCMI_EVENT_LC_TYPEC_UCSI_CCI_CHANGED,
		.max_payld_sz = sizeof(struct scmi_lc_typec_cci_payld),
		.max_report_sz = sizeof(struct scmi_lc_typec_cci_report),
	},
};

static const struct scmi_event_ops scmi_lc_typec_event_ops = {
	.set_notify_enabled = scmi_lc_typec_set_notify_enabled,
	.fill_custom_report = scmi_lc_typec_fill_custom_report,
};

static const struct scmi_protocol_events scmi_lc_typec_protocol_events = {
	.queue_sz = SCMI_PROTO_QUEUE_SZ,
	.ops = &scmi_lc_typec_event_ops,
	.evts = scmi_lc_typec_events,
	.num_events = ARRAY_SIZE(scmi_lc_typec_events),
	.num_sources = 1,
};

static const struct scmi_lc_typec_proto_ops scmi_lc_typec_proto_ops = {
	.info_get = scmi_lc_typec_info_get,
	.ucsi_read = scmi_lc_typec_ucsi_read,
	.ucsi_write = scmi_lc_typec_ucsi_write,
	.connector_ext_status = scmi_lc_typec_connector_ext_status,
	.notify_enable = scmi_lc_typec_notify_enable,
};

static int scmi_lc_typec_protocol_init(const struct scmi_protocol_handle *ph)
{
	struct scmi_lc_typec_info *pi;
	u32 version;
	int ret;

	ret = ph->xops->version_get(ph, &version);
	if (ret)
		return ret;

	pi = devm_kzalloc(ph->dev, sizeof(*pi), GFP_KERNEL);
	if (!pi)
		return -ENOMEM;

	ret = scmi_lc_typec_attributes_get(ph, pi);
	if (ret)
		return ret;

	dev_info(ph->dev,
		 "lc-typec Version %d.%d, %u connector(s), notifications %s\n",
		 PROTOCOL_REV_MAJOR(version), PROTOCOL_REV_MINOR(version),
		 pi->num_connectors,
		 pi->flags & SCMI_LC_TYPEC_ATTR_NOTIFY ? "yes" : "no");

	return ph->set_priv(ph, pi, version);
}

static const struct scmi_protocol scmi_lc_typec = {
	.id = SCMI_PROTOCOL_LC_TYPEC,
	.owner = THIS_MODULE,
	.instance_init = &scmi_lc_typec_protocol_init,
	.ops = &scmi_lc_typec_proto_ops,
	.events = &scmi_lc_typec_protocol_events,
	.supported_version = SCMI_PROTOCOL_SUPPORTED_VERSION,
	/*
	 * Vendor-only: matched for any sub-vendor (the RK3399 server reports
	 * SCMI_LC_SUBVENDOR) and any implementation version.
	 */
	.vendor_id = SCMI_LC_VENDOR,
};
module_scmi_protocol(scmi_lc_typec);

MODULE_ALIAS("scmi-protocol-" __stringify(SCMI_PROTOCOL_LC_TYPEC) "-" SCMI_LC_VENDOR);
MODULE_DESCRIPTION("Libre Computer SCMI Type-C (UCSI) vendor protocol");
MODULE_LICENSE("GPL");
