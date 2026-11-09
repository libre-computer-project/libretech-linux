// SPDX-License-Identifier: GPL-2.0-only
/*
 * Amlogic EL3 device credential service -- the normal world's door to it.
 *
 * The secure monitor (TF-A built with AML_DEVID) holds a server-issued serial
 * and secret in eFuse and answers challenges with them; it never hands the
 * secret out.  This driver exposes its three calls as two device nodes
 * (include/uapi/linux/meson-devid.h): /dev/devid-enroll for Identify and Burn,
 * which one process at a time may hold open, and /dev/devid for Answer.  Who
 * may open them is udev's call (a group), not this driver's.
 *
 * Registered by meson-sm only when the firmware answers the service's VERSION
 * call, so a board without the service has no node at all.  /dev/devid-enroll
 * burns eFuse, so it is created only when the module is loaded with enroll=1
 * (meson_devid.enroll=1 on the command line for a built-in driver);
 * /dev/devid is always created.
 */

#include <linux/arm-smccc.h>
#include <linux/atomic.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>

#include <linux/firmware/meson/meson_sm.h>
#include <uapi/linux/meson-devid.h>

#define SM_DEVID_IDENTIFY	0xC20000D0
#define SM_DEVID_BURN		0xC20000D1
#define SM_DEVID_ANSWER		0xC20000D2

static bool enroll;
module_param(enroll, bool, 0444);
MODULE_PARM_DESC(enroll, "create /dev/devid-enroll (Identify and Burn); default off");

struct meson_devid {
	struct meson_sm_firmware *fw;
	struct miscdevice enroll;
	struct miscdevice answer;
	atomic_t enroll_open;
	bool has_enroll;	/* enroll was registered at probe */
};

static struct meson_devid *devid_from_file(struct file *f)
{
	struct miscdevice *m = f->private_data;

	if (!strcmp(m->name, "devid-enroll"))
		return container_of(m, struct meson_devid, enroll);
	return container_of(m, struct meson_devid, answer);
}

static int devid_enroll_open(struct inode *inode, struct file *f)
{
	struct meson_devid *d = container_of(f->private_data,
					     struct meson_devid, enroll);

	if (atomic_cmpxchg(&d->enroll_open, 0, 1) != 0)
		return -EBUSY;
	return 0;
}

static int devid_enroll_release(struct inode *inode, struct file *f)
{
	struct meson_devid *d = container_of(f->private_data,
					     struct meson_devid, enroll);

	atomic_set(&d->enroll_open, 0);
	return 0;
}

static long devid_identify(struct meson_devid *d, void __user *arg)
{
	struct arm_smccc_1_2_regs r = { .a0 = SM_DEVID_IDENTIFY };
	struct meson_devid_ident id;
	int ret;

	ret = meson_sm_call_smc64(d->fw, &r, NULL, 0, &id, sizeof(id));
	if (ret)
		return ret;
	if ((long)r.a0 != (long)sizeof(id))
		return -EIO;
	return copy_to_user(arg, &id, sizeof(id)) ? -EFAULT : 0;
}

static long devid_burn(struct meson_devid *d, void __user *arg)
{
	struct arm_smccc_1_2_regs r = { .a0 = SM_DEVID_BURN,
					.a1 = MESON_DEVID_BLOB_BYTES };
	struct meson_devid_burn *b;
	long ret;

	b = memdup_user(arg, sizeof(*b));
	if (IS_ERR(b))
		return PTR_ERR(b);
	ret = meson_sm_call_smc64(d->fw, &r, b->blob, sizeof(b->blob), NULL, 0);
	if (!ret) {
		b->result = (s32)r.a0;
		ret = copy_to_user(arg, b, sizeof(*b)) ? -EFAULT : 0;
	}
	kfree(b);
	return ret;
}

static long devid_answer(struct meson_devid *d, void __user *arg)
{
	struct arm_smccc_1_2_regs r = { .a0 = SM_DEVID_ANSWER };
	struct meson_devid_answer a;
	long ret;

	if (copy_from_user(&a, arg, sizeof(a)))
		return -EFAULT;
	if (a.nonce_len == 0 || a.nonce_len > MESON_DEVID_NONCE_MAX)
		return -EINVAL;
	r.a1 = a.nonce_len;
	ret = meson_sm_call_smc64(d->fw, &r, a.nonce, a.nonce_len, NULL, 0);
	if (ret)
		return ret;
	a.result = (s32)r.a0;
	a.serial = a.result == 0 ? (u32)r.a1 : 0;
	memset(a.mac, 0, sizeof(a.mac));
	if (a.result == 0) {
		memcpy(a.mac, &r.a2, 8);
		memcpy(a.mac + 8, &r.a3, 8);
		memcpy(a.mac + 16, &r.a4, 8);
		memcpy(a.mac + 24, &r.a5, 8);
	}
	ret = copy_to_user(arg, &a, sizeof(a)) ? -EFAULT : 0;
	memzero_explicit(&r, sizeof(r));
	return ret;
}

static long devid_enroll_ioctl(struct file *f, unsigned int cmd,
			       unsigned long arg)
{
	struct meson_devid *d = devid_from_file(f);

	switch (cmd) {
	case MESON_DEVID_IOC_IDENTIFY:
		return devid_identify(d, (void __user *)arg);
	case MESON_DEVID_IOC_BURN:
		return devid_burn(d, (void __user *)arg);
	default:
		return -ENOTTY;
	}
}

static long devid_answer_ioctl(struct file *f, unsigned int cmd,
			       unsigned long arg)
{
	struct meson_devid *d = devid_from_file(f);

	if (cmd != MESON_DEVID_IOC_ANSWER)
		return -ENOTTY;
	return devid_answer(d, (void __user *)arg);
}

static const struct file_operations devid_enroll_fops = {
	.owner		= THIS_MODULE,
	.open		= devid_enroll_open,
	.release	= devid_enroll_release,
	.unlocked_ioctl	= devid_enroll_ioctl,
	.compat_ioctl	= compat_ptr_ioctl,
};

static const struct file_operations devid_answer_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= devid_answer_ioctl,
	.compat_ioctl	= compat_ptr_ioctl,
};

static int meson_devid_probe(struct platform_device *pdev)
{
	struct meson_devid *d;
	int ret;

	d = devm_kzalloc(&pdev->dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;
	d->fw = dev_get_drvdata(pdev->dev.parent);
	if (!d->fw)
		return -EPROBE_DEFER;

	d->enroll.minor = MISC_DYNAMIC_MINOR;
	d->enroll.name = "devid-enroll";
	d->enroll.fops = &devid_enroll_fops;
	d->enroll.mode = 0600;
	d->answer.minor = MISC_DYNAMIC_MINOR;
	d->answer.name = "devid";
	d->answer.fops = &devid_answer_fops;
	d->answer.mode = 0600;

	if (enroll) {
		ret = misc_register(&d->enroll);
		if (ret)
			return ret;
		d->has_enroll = true;
	}
	ret = misc_register(&d->answer);
	if (ret) {
		if (d->has_enroll)
			misc_deregister(&d->enroll);
		return ret;
	}
	platform_set_drvdata(pdev, d);
	return 0;
}

static void meson_devid_remove(struct platform_device *pdev)
{
	struct meson_devid *d = platform_get_drvdata(pdev);

	misc_deregister(&d->answer);
	if (d->has_enroll)
		misc_deregister(&d->enroll);
}

static struct platform_driver meson_devid_driver = {
	.probe	= meson_devid_probe,
	.remove	= meson_devid_remove,
	.driver	= {
		.name = "meson-sm-devid",
	},
};
module_platform_driver(meson_devid_driver);

MODULE_ALIAS("platform:meson-sm-devid");
MODULE_DESCRIPTION("Amlogic EL3 device credential service");
MODULE_LICENSE("GPL");
