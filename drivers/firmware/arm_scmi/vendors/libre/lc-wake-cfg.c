// SPDX-License-Identifier: GPL-2.0
/*
 * SCMI vendor protocol 0x81 "lc-wake-cfg" (Libre Computer)
 *
 * The GXL open SC's wake configuration (scp-firmware mod_gxl_wake_cfg.c):
 *
 *   id 0  PROTOCOL_VERSION
 *   id 1  PROTOCOL_ATTRIBUTES         -> [7:0] most IR entries per SET
 *   id 2  PROTOCOL_MESSAGE_ATTRIBUTES
 *   id 3  IR_WAKE_SET  count, code[4], mask[4]   (fixed 36 B)
 *   id 4  IR_WAKE_GET  -> count, code[4], mask[4]
 *
 * The SC's board DT carries no protocol@81 node: the SCMI core activates
 * this protocol from base discovery (driver.c scmi_dtless_protocols).  The
 * client below exports the two calls to meson-ir, which turns rc-core's
 * wakeup filter into a list.
 */

#define pr_fmt(fmt) "SCMI lc-wake-cfg - " fmt

#include <linux/bitfield.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/scmi_protocol.h>
#include <linux/scmi_lc_wake_cfg.h>

#include "../../protocols.h"

#define SCMI_PROTOCOL_SUPPORTED_VERSION		0x10000

enum scmi_lc_wake_cfg_protocol_cmd {
	LC_WAKE_CFG_IR_WAKE_SET = 0x3,
	LC_WAKE_CFG_IR_WAKE_GET = 0x4,
	LC_WAKE_CFG_CEC_WAKE_SET = 0x5,
	LC_WAKE_CFG_CEC_WAKE_GET = 0x6,
};

/* PROTOCOL_ATTRIBUTES: [7:0] IR list length; bit 8 the CEC messages exist */
#define LC_WAKE_CFG_ATTR_IR_MAX		GENMASK(7, 0)
#define LC_WAKE_CFG_ATTR_CEC		BIT(8)

/* The wire list, exactly the SC's gxl_ir_wake.h GXL_IR_WAKE_WIRE_WORDS. */
struct scmi_msg_lc_wake_cfg_ir_list {
	__le32 count;
	__le32 code[SCMI_LC_WAKE_CFG_IR_MAX];
	__le32 mask[SCMI_LC_WAKE_CFG_IR_MAX];
};

/* The SC's gxl_cec_wake.h GXL_CEC_WAKE_WIRE_WORDS: flags, phys_addr. */
struct scmi_msg_lc_wake_cfg_cec {
	__le32 flags;
	__le32 phys_addr;
};

struct scmi_lc_wake_cfg_info {
	u32 ir_max;
	bool cec;
};

static int scmi_lc_wake_cfg_cec_wake_set(const struct scmi_protocol_handle *ph,
					 const struct scmi_lc_wake_cfg_cec *cfg)
{
	struct scmi_lc_wake_cfg_info *pi = ph->get_priv(ph);
	struct scmi_msg_lc_wake_cfg_cec *req;
	struct scmi_xfer *t;
	int ret;

	if (!pi->cec)
		return -EOPNOTSUPP;
	if (cfg->flags & ~SCMI_LC_WAKE_CFG_CEC_ENABLE || cfg->phys_addr > 0xffff)
		return -EINVAL;

	ret = ph->xops->xfer_get_init(ph, LC_WAKE_CFG_CEC_WAKE_SET,
				      sizeof(*req), 0, &t);
	if (ret)
		return ret;

	req = t->tx.buf;
	req->flags = cpu_to_le32(cfg->flags);
	req->phys_addr = cpu_to_le32(cfg->phys_addr);

	ret = ph->xops->do_xfer(ph, t);

	ph->xops->xfer_put(ph, t);

	return ret;
}

static int scmi_lc_wake_cfg_cec_wake_get(const struct scmi_protocol_handle *ph,
					 struct scmi_lc_wake_cfg_cec *cfg)
{
	struct scmi_lc_wake_cfg_info *pi = ph->get_priv(ph);
	struct scmi_msg_lc_wake_cfg_cec *resp;
	struct scmi_xfer *t;
	int ret;

	if (!pi->cec)
		return -EOPNOTSUPP;

	ret = ph->xops->xfer_get_init(ph, LC_WAKE_CFG_CEC_WAKE_GET, 0,
				      sizeof(*resp), &t);
	if (ret)
		return ret;

	ret = ph->xops->do_xfer(ph, t);
	if (!ret) {
		resp = t->rx.buf;
		if (t->rx.len != sizeof(*resp)) {
			ret = -EPROTO;
		} else {
			cfg->flags = le32_to_cpu(resp->flags);
			cfg->phys_addr = le32_to_cpu(resp->phys_addr);
		}
	}

	ph->xops->xfer_put(ph, t);

	return ret;
}

static int scmi_lc_wake_cfg_ir_wake_set(const struct scmi_protocol_handle *ph,
					const struct scmi_lc_wake_cfg_ir_list *list)
{
	struct scmi_lc_wake_cfg_info *pi = ph->get_priv(ph);
	struct scmi_msg_lc_wake_cfg_ir_list *req;
	struct scmi_xfer *t;
	int ret, i;

	if (list->count > pi->ir_max || list->count > SCMI_LC_WAKE_CFG_IR_MAX)
		return -EINVAL;

	ret = ph->xops->xfer_get_init(ph, LC_WAKE_CFG_IR_WAKE_SET,
				      sizeof(*req), 0, &t);
	if (ret)
		return ret;

	req = t->tx.buf;
	req->count = cpu_to_le32(list->count);
	for (i = 0; i < SCMI_LC_WAKE_CFG_IR_MAX; i++) {
		bool used = i < list->count;

		req->code[i] = cpu_to_le32(used ? list->code[i] : 0);
		req->mask[i] = cpu_to_le32(used ? list->mask[i] : 0);
	}

	ret = ph->xops->do_xfer(ph, t);

	ph->xops->xfer_put(ph, t);

	return ret;
}

static int scmi_lc_wake_cfg_ir_wake_get(const struct scmi_protocol_handle *ph,
					struct scmi_lc_wake_cfg_ir_list *list)
{
	struct scmi_msg_lc_wake_cfg_ir_list *resp;
	struct scmi_xfer *t;
	int ret, i;

	ret = ph->xops->xfer_get_init(ph, LC_WAKE_CFG_IR_WAKE_GET, 0,
				      sizeof(*resp), &t);
	if (ret)
		return ret;

	ret = ph->xops->do_xfer(ph, t);
	if (!ret) {
		resp = t->rx.buf;
		if (t->rx.len != sizeof(*resp)) {
			ret = -EPROTO;
		} else {
			list->count = min_t(u32, le32_to_cpu(resp->count),
					    SCMI_LC_WAKE_CFG_IR_MAX);
			for (i = 0; i < SCMI_LC_WAKE_CFG_IR_MAX; i++) {
				list->code[i] = le32_to_cpu(resp->code[i]);
				list->mask[i] = le32_to_cpu(resp->mask[i]);
			}
		}
	}

	ph->xops->xfer_put(ph, t);

	return ret;
}

static const struct scmi_lc_wake_cfg_proto_ops scmi_lc_wake_cfg_proto_ops = {
	.ir_wake_set = scmi_lc_wake_cfg_ir_wake_set,
	.ir_wake_get = scmi_lc_wake_cfg_ir_wake_get,
	.cec_wake_set = scmi_lc_wake_cfg_cec_wake_set,
	.cec_wake_get = scmi_lc_wake_cfg_cec_wake_get,
};

static int scmi_lc_wake_cfg_protocol_init(const struct scmi_protocol_handle *ph)
{
	struct scmi_lc_wake_cfg_info *pi;
	struct scmi_xfer *t;
	u32 version;
	int ret;

	ret = ph->xops->version_get(ph, &version);
	if (ret)
		return ret;

	pi = devm_kzalloc(ph->dev, sizeof(*pi), GFP_KERNEL);
	if (!pi)
		return -ENOMEM;

	ret = ph->xops->xfer_get_init(ph, PROTOCOL_ATTRIBUTES, 0,
				      sizeof(__le32), &t);
	if (ret)
		return ret;
	ret = ph->xops->do_xfer(ph, t);
	if (!ret) {
		u32 attr = le32_to_cpu(*(__le32 *)t->rx.buf);

		pi->ir_max = FIELD_GET(LC_WAKE_CFG_ATTR_IR_MAX, attr);
		pi->cec = !!(attr & LC_WAKE_CFG_ATTR_CEC);
	}
	ph->xops->xfer_put(ph, t);
	if (ret)
		return ret;

	dev_info(ph->dev, "lc-wake-cfg Version %d.%d, IR wake list of %u, CEC wake %s\n",
		 PROTOCOL_REV_MAJOR(version), PROTOCOL_REV_MINOR(version),
		 pi->ir_max, pi->cec ? "yes" : "no");

	return ph->set_priv(ph, pi, version);
}

static const struct scmi_protocol scmi_lc_wake_cfg = {
	.id = SCMI_PROTOCOL_LC_WAKE_CFG,
	.owner = THIS_MODULE,
	.instance_init = &scmi_lc_wake_cfg_protocol_init,
	.ops = &scmi_lc_wake_cfg_proto_ops,
	.supported_version = SCMI_PROTOCOL_SUPPORTED_VERSION,
	.vendor_id = SCMI_LC_WAKE_CFG_VENDOR,
	.sub_vendor_id = SCMI_LC_WAKE_CFG_SUBVENDOR,
};

/* ---- the client: one SC per system, published to meson-ir ---- */

static DEFINE_MUTEX(lc_wake_cfg_lock);
static const struct scmi_lc_wake_cfg_proto_ops *lc_wake_cfg_ops;
static const struct scmi_protocol_handle *lc_wake_cfg_ph;

int lc_scmi_ir_wake_set(const struct scmi_lc_wake_cfg_ir_list *list)
{
	int ret = -ENODEV;

	mutex_lock(&lc_wake_cfg_lock);
	if (lc_wake_cfg_ops)
		ret = lc_wake_cfg_ops->ir_wake_set(lc_wake_cfg_ph, list);
	mutex_unlock(&lc_wake_cfg_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(lc_scmi_ir_wake_set);

int lc_scmi_ir_wake_get(struct scmi_lc_wake_cfg_ir_list *list)
{
	int ret = -ENODEV;

	mutex_lock(&lc_wake_cfg_lock);
	if (lc_wake_cfg_ops)
		ret = lc_wake_cfg_ops->ir_wake_get(lc_wake_cfg_ph, list);
	mutex_unlock(&lc_wake_cfg_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(lc_scmi_ir_wake_get);

int lc_scmi_cec_wake_set(const struct scmi_lc_wake_cfg_cec *cfg)
{
	int ret = -ENODEV;

	mutex_lock(&lc_wake_cfg_lock);
	if (lc_wake_cfg_ops)
		ret = lc_wake_cfg_ops->cec_wake_set(lc_wake_cfg_ph, cfg);
	mutex_unlock(&lc_wake_cfg_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(lc_scmi_cec_wake_set);

int lc_scmi_cec_wake_get(struct scmi_lc_wake_cfg_cec *cfg)
{
	int ret = -ENODEV;

	mutex_lock(&lc_wake_cfg_lock);
	if (lc_wake_cfg_ops)
		ret = lc_wake_cfg_ops->cec_wake_get(lc_wake_cfg_ph, cfg);
	mutex_unlock(&lc_wake_cfg_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(lc_scmi_cec_wake_get);

/*
 * debugfs lc-wake-cfg/cec_wake: the SC's own answer to CEC_WAKE_GET,
 * "flags F phys_addr P" -- what the SC will wake on at the next suspend.
 */
static int lc_wake_cfg_cec_show(struct seq_file *s, void *unused)
{
	struct scmi_lc_wake_cfg_cec cfg;
	int ret;

	ret = lc_scmi_cec_wake_get(&cfg);
	if (ret)
		return ret;

	seq_printf(s, "flags %x phys_addr %04x\n", cfg.flags, cfg.phys_addr);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(lc_wake_cfg_cec);

/*
 * debugfs lc-wake-cfg/ir_wake: the SC's own answer to IR_WAKE_GET, one
 * "frame/mask" pair per line -- the list the SC will match at the next
 * suspend, whoever (U-Boot, meson-ir) set it last.
 */
static int lc_wake_cfg_ir_show(struct seq_file *s, void *unused)
{
	struct scmi_lc_wake_cfg_ir_list list;
	int ret, i;

	ret = lc_scmi_ir_wake_get(&list);
	if (ret)
		return ret;

	seq_printf(s, "count %u\n", list.count);
	for (i = 0; i < list.count; i++)
		seq_printf(s, "%08x/%08x\n", list.code[i], list.mask[i]);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(lc_wake_cfg_ir);

static struct dentry *lc_wake_cfg_debugfs;

static int lc_wake_cfg_probe(struct scmi_device *sdev)
{
	const struct scmi_lc_wake_cfg_proto_ops *ops;
	struct scmi_protocol_handle *ph;

	if (!sdev->handle)
		return -ENODEV;

	ops = sdev->handle->devm_protocol_get(sdev, SCMI_PROTOCOL_LC_WAKE_CFG,
					      &ph);
	if (IS_ERR(ops))
		return PTR_ERR(ops);

	mutex_lock(&lc_wake_cfg_lock);
	if (lc_wake_cfg_ops) {
		mutex_unlock(&lc_wake_cfg_lock);
		return -EBUSY; /* one SC per system */
	}
	lc_wake_cfg_ph = ph;
	lc_wake_cfg_ops = ops;
	mutex_unlock(&lc_wake_cfg_lock);

	lc_wake_cfg_debugfs = debugfs_create_dir("lc-wake-cfg", NULL);
	debugfs_create_file("ir_wake", 0444, lc_wake_cfg_debugfs, NULL,
			    &lc_wake_cfg_ir_fops);
	debugfs_create_file("cec_wake", 0444, lc_wake_cfg_debugfs, NULL,
			    &lc_wake_cfg_cec_fops);

	return 0;
}

static void lc_wake_cfg_remove(struct scmi_device *sdev)
{
	debugfs_remove_recursive(lc_wake_cfg_debugfs);
	lc_wake_cfg_debugfs = NULL;

	mutex_lock(&lc_wake_cfg_lock);
	lc_wake_cfg_ops = NULL;
	lc_wake_cfg_ph = NULL;
	mutex_unlock(&lc_wake_cfg_lock);
}

static const struct scmi_device_id lc_wake_cfg_id_table[] = {
	{ SCMI_PROTOCOL_LC_WAKE_CFG, "lc-wake-cfg" },
	{ }
};
MODULE_DEVICE_TABLE(scmi, lc_wake_cfg_id_table);

static struct scmi_driver lc_wake_cfg_driver = {
	.name = "scmi-lc-wake-cfg",
	.probe = lc_wake_cfg_probe,
	.remove = lc_wake_cfg_remove,
	.id_table = lc_wake_cfg_id_table,
};

static int __init lc_wake_cfg_init(void)
{
	int ret;

	ret = scmi_protocol_register(&scmi_lc_wake_cfg);
	if (ret)
		return ret;

	ret = scmi_register(&lc_wake_cfg_driver);
	if (ret)
		scmi_protocol_unregister(&scmi_lc_wake_cfg);

	return ret;
}
module_init(lc_wake_cfg_init);

static void __exit lc_wake_cfg_exit(void)
{
	scmi_unregister(&lc_wake_cfg_driver);
	scmi_protocol_unregister(&scmi_lc_wake_cfg);
}
module_exit(lc_wake_cfg_exit);

MODULE_ALIAS("scmi-protocol-" __stringify(SCMI_PROTOCOL_LC_WAKE_CFG) "-" SCMI_LC_WAKE_CFG_VENDOR);
MODULE_DESCRIPTION("Libre Computer SCMI wake-configuration vendor protocol");
MODULE_LICENSE("GPL");
