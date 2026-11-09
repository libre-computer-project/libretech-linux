/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * SCMI vendor protocol 0x81 "lc-wake-cfg" (Libre Computer).
 *
 * Served by the GXL open SC (vendor "LibreComputer", sub-vendor "Amlogic"):
 * the run-time wake configuration the SC uses while the board sleeps.  The
 * first setting is the IR wake list -- up to SCMI_LC_WAKE_CFG_IR_MAX
 * (frame, mask) pairs; the SC wakes on a decoded NEC frame for which
 * (frame & mask) == (code & mask).  A frame is the AO IR hardware decoder's
 * 32-bit word, which is NOT an rc-core scancode: see meson-ir.
 *
 * Until an agent sets a list the SC uses its board default; U-Boot may set
 * one at boot, and the last writer before a suspend wins.
 */

#ifndef _LINUX_SCMI_LC_WAKE_CFG_H
#define _LINUX_SCMI_LC_WAKE_CFG_H

#include <linux/bits.h>
#include <linux/errno.h>
#include <linux/types.h>

#define SCMI_PROTOCOL_LC_WAKE_CFG	0x81
#define SCMI_LC_WAKE_CFG_VENDOR		"LibreComputer"
#define SCMI_LC_WAKE_CFG_SUBVENDOR	"Amlogic"

#define SCMI_LC_WAKE_CFG_IR_MAX		4

struct scmi_protocol_handle;

struct scmi_lc_wake_cfg_ir_list {
	u32 count; /* 0..SCMI_LC_WAKE_CFG_IR_MAX; 0 = IR never wakes */
	u32 code[SCMI_LC_WAKE_CFG_IR_MAX];
	u32 mask[SCMI_LC_WAKE_CFG_IR_MAX];
};

/*
 * HDMI-CEC wake (msgs 5/6, present when PROTOCOL_ATTRIBUTES bit 8 is set):
 * the SC wakes on User Control Pressed Power / Power Toggle / Power On to the
 * logical address the controller holds, and -- when it knows our physical
 * address -- on Set Stream Path / Routing Change to it.
 */
#define SCMI_LC_WAKE_CFG_CEC_ENABLE	BIT(0)
#define SCMI_LC_WAKE_CFG_CEC_PA_NONE	0xffff /* = CEC_PHYS_ADDR_INVALID */

struct scmi_lc_wake_cfg_cec {
	u32 flags;     /* SCMI_LC_WAKE_CFG_CEC_* */
	u32 phys_addr; /* [15:0], or SCMI_LC_WAKE_CFG_CEC_PA_NONE */
};

struct scmi_lc_wake_cfg_proto_ops {
	int (*ir_wake_set)(const struct scmi_protocol_handle *ph,
			   const struct scmi_lc_wake_cfg_ir_list *list);
	int (*ir_wake_get)(const struct scmi_protocol_handle *ph,
			   struct scmi_lc_wake_cfg_ir_list *list);
	int (*cec_wake_set)(const struct scmi_protocol_handle *ph,
			    const struct scmi_lc_wake_cfg_cec *cfg);
	int (*cec_wake_get)(const struct scmi_protocol_handle *ph,
			    struct scmi_lc_wake_cfg_cec *cfg);
};

/*
 * For consumers outside the SCMI bus (meson-ir, meson-ao-cec): forwarded to
 * the bound lc-wake-cfg client.  -ENODEV when no SC serves the protocol,
 * -EOPNOTSUPP when it serves no CEC messages.
 */
#if IS_REACHABLE(CONFIG_LIBRE_SCMI_WAKE_CFG)
int lc_scmi_ir_wake_set(const struct scmi_lc_wake_cfg_ir_list *list);
int lc_scmi_ir_wake_get(struct scmi_lc_wake_cfg_ir_list *list);
int lc_scmi_cec_wake_set(const struct scmi_lc_wake_cfg_cec *cfg);
int lc_scmi_cec_wake_get(struct scmi_lc_wake_cfg_cec *cfg);
#else
static inline int lc_scmi_ir_wake_set(const struct scmi_lc_wake_cfg_ir_list *list)
{
	return -ENODEV;
}

static inline int lc_scmi_ir_wake_get(struct scmi_lc_wake_cfg_ir_list *list)
{
	return -ENODEV;
}

static inline int lc_scmi_cec_wake_set(const struct scmi_lc_wake_cfg_cec *cfg)
{
	return -ENODEV;
}

static inline int lc_scmi_cec_wake_get(struct scmi_lc_wake_cfg_cec *cfg)
{
	return -ENODEV;
}
#endif

#endif /* _LINUX_SCMI_LC_WAKE_CFG_H */
