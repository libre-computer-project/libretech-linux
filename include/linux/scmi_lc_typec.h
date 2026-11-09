/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * SCMI vendor protocol 0x80 "lc-typec" (Libre Computer).
 *
 * Served by the RK3399 Cortex-M0 SCMI server, which owns the Type-C PD
 * port and exposes it as a UCSI PPM: the UCSI register file (VERSION,
 * CCI, CONTROL, MESSAGE_IN, MESSAGE_OUT) is read and written through
 * UCSI_READ / UCSI_WRITE, and every reply is served from the server's
 * RAM state, never from I2C or the PD partner.
 */

#ifndef _LINUX_SCMI_LC_TYPEC_H
#define _LINUX_SCMI_LC_TYPEC_H

#include <linux/ktime.h>
#include <linux/scmi_protocol.h>
#include <linux/types.h>

#define SCMI_PROTOCOL_LC_TYPEC		0x80

#define SCMI_LC_VENDOR			"libre-computer"
#define SCMI_LC_SUBVENDOR		"rk3399-m0"

/* UCSI_READ and UCSI_WRITE size limits of one message */
#define SCMI_LC_TYPEC_READ_MAX		32
#define SCMI_LC_TYPEC_WRITE_MAX		16

/* PROTOCOL_ATTRIBUTES flags */
#define SCMI_LC_TYPEC_ATTR_NOTIFY	BIT(0)

enum scmi_lc_typec_orientation {
	SCMI_LC_TYPEC_ORIENT_NONE = 0,
	SCMI_LC_TYPEC_ORIENT_NORMAL = 1,
	SCMI_LC_TYPEC_ORIENT_REVERSE = 2,
};

enum scmi_lc_typec_owner {
	SCMI_LC_TYPEC_OWNER_M0 = 0,
	SCMI_LC_TYPEC_OWNER_YIELDED = 1,
};

/**
 * struct scmi_lc_typec_ext_status - CONNECTOR_EXT_STATUS reply
 * @orientation: enum scmi_lc_typec_orientation
 * @contract_mv: negotiated contract voltage in mV (0 without a contract)
 * @contract_ma: negotiated contract current in mA (0 without a contract)
 * @pd_rev: PD revision of the contract
 * @owner: enum scmi_lc_typec_owner
 */
struct scmi_lc_typec_ext_status {
	u32 orientation;
	u32 contract_mv;
	u32 contract_ma;
	u32 pd_rev;
	u32 owner;
};

/**
 * struct scmi_lc_typec_proto_ops - lc-typec protocol operations
 * @info_get: number of connectors and PROTOCOL_ATTRIBUTES flags
 * @ucsi_read: read @len bytes of the UCSI register file at @offset
 * @ucsi_write: write @len bytes to the UCSI register file at @offset
 * @connector_ext_status: CONNECTOR_EXT_STATUS of connector @conn (0-based)
 * @notify_enable: enable or disable the UCSI_CCI_CHANGED notification
 */
struct scmi_lc_typec_proto_ops {
	int (*info_get)(const struct scmi_protocol_handle *ph,
			u32 *num_connectors, u32 *flags);
	int (*ucsi_read)(const struct scmi_protocol_handle *ph,
			 u32 offset, u32 len, void *buf);
	int (*ucsi_write)(const struct scmi_protocol_handle *ph,
			  u32 offset, u32 len, const void *buf);
	int (*connector_ext_status)(const struct scmi_protocol_handle *ph,
				    u32 conn,
				    struct scmi_lc_typec_ext_status *st);
	int (*notify_enable)(const struct scmi_protocol_handle *ph,
			     bool enable);
};

enum scmi_lc_typec_notification_events {
	SCMI_EVENT_LC_TYPEC_UCSI_CCI_CHANGED = 0x0,
};

struct scmi_lc_typec_cci_report {
	ktime_t timestamp;
	u32 cci;
};

#endif /* _LINUX_SCMI_LC_TYPEC_H */
