// SPDX-License-Identifier: GPL-2.0-only
/*
 * Amlogic Secure Monitor driver
 *
 * Copyright (C) 2016 Endless Mobile, Inc.
 * Author: Carlo Caione <carlo@endlessm.com>
 */

#define pr_fmt(fmt) "meson-sm: " fmt

#include <linux/arm-smccc.h>
#include <linux/bug.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/property.h>
#include <linux/types.h>
#include <linux/sizes.h>
 #include <linux/slab.h>

#include <linux/firmware/meson/meson_sm.h>

struct meson_sm_cmd {
	unsigned int index;
	u32 smc_id;
};
#define CMD(d, s) { .index = (d), .smc_id = (s), }

struct meson_sm_chip {
	unsigned int shmem_size;
	u32 cmd_shmem_in_base;
	u32 cmd_shmem_out_base;
	struct meson_sm_cmd cmd[];
};

static const struct meson_sm_chip gxbb_chip = {
	.shmem_size		= SZ_4K,
	.cmd_shmem_in_base	= 0x82000020,
	.cmd_shmem_out_base	= 0x82000021,
	.cmd = {
		CMD(SM_EFUSE_READ,	0x82000030),
		CMD(SM_EFUSE_WRITE,	0x82000031),
		CMD(SM_EFUSE_USER_MAX,	0x82000033),
		CMD(SM_GET_CHIP_ID,	0x82000044),
		CMD(SM_TEST_N_DIR,	0x82000046),
		CMD(SM_A1_PWRC_SET,	0x82000093),
		CMD(SM_A1_PWRC_GET,	0x82000095),
		{ /* sentinel */ },
	},
};

struct meson_sm_firmware {
	const struct meson_sm_chip *chip;
	void __iomem *sm_shmem_in_base;
	void __iomem *sm_shmem_out_base;
	/* One SMC at a time owns the two shared buffers. */
	struct mutex shmem_lock;
};

static u32 meson_sm_get_cmd(const struct meson_sm_chip *chip,
			    unsigned int cmd_index)
{
	const struct meson_sm_cmd *cmd = chip->cmd;

	while (cmd->smc_id && cmd->index != cmd_index)
		cmd++;

	return cmd->smc_id;
}

static s32 __meson_sm_call(u32 cmd, u32 arg0, u32 arg1, u32 arg2,
			   u32 arg3, u32 arg4)
{
	struct arm_smccc_res res;

	arm_smccc_smc(cmd, arg0, arg1, arg2, arg3, arg4, 0, 0, &res);
	return res.a0;
}

static void __iomem *meson_sm_map_shmem(u32 cmd_shmem, unsigned int size)
{
	u32 sm_phy_base;

	sm_phy_base = __meson_sm_call(cmd_shmem, 0, 0, 0, 0, 0);
	if (!sm_phy_base)
		return NULL;

	return ioremap_cache(sm_phy_base, size);
}

/**
 * meson_sm_call - generic SMC32 call to the secure-monitor
 *
 * @fw:		Pointer to secure-monitor firmware
 * @cmd_index:	Index of the SMC32 function ID
 * @ret:	Returned value
 * @arg0:	SMC32 Argument 0
 * @arg1:	SMC32 Argument 1
 * @arg2:	SMC32 Argument 2
 * @arg3:	SMC32 Argument 3
 * @arg4:	SMC32 Argument 4
 *
 * Return:	0 on success, a negative value on error
 */
int meson_sm_call(struct meson_sm_firmware *fw, unsigned int cmd_index,
		  s32 *ret, u32 arg0, u32 arg1, u32 arg2, u32 arg3, u32 arg4)
{
	u32 cmd;
	s32 lret;

	if (!fw->chip)
		return -ENOENT;

	cmd = meson_sm_get_cmd(fw->chip, cmd_index);
	if (!cmd)
		return -EINVAL;

	lret = __meson_sm_call(cmd, arg0, arg1, arg2, arg3, arg4);

	if (ret)
		*ret = lret;

	return 0;
}
EXPORT_SYMBOL(meson_sm_call);

/**
 * meson_sm_call_read - retrieve data from secure-monitor
 *
 * @fw:		Pointer to secure-monitor firmware
 * @buffer:	Buffer to store the retrieved data
 * @bsize:	Size of the buffer
 * @cmd_index:	Index of the SMC32 function ID
 * @arg0:	SMC32 Argument 0
 * @arg1:	SMC32 Argument 1
 * @arg2:	SMC32 Argument 2
 * @arg3:	SMC32 Argument 3
 * @arg4:	SMC32 Argument 4
 *
 * Return:	size of read data on success, a negative value on error
 *		When 0 is returned there is no guarantee about the amount of
 *		data read and bsize bytes are copied in buffer.
 */
int meson_sm_call_read(struct meson_sm_firmware *fw, void *buffer,
		       unsigned int bsize, unsigned int cmd_index, u32 arg0,
		       u32 arg1, u32 arg2, u32 arg3, u32 arg4)
{
	s32 size;
	int ret;

	if (!fw->chip)
		return -ENOENT;

	if (!fw->chip->cmd_shmem_out_base)
		return -EINVAL;

	if (bsize > fw->chip->shmem_size)
		return -EINVAL;

	mutex_lock(&fw->shmem_lock);
	if (meson_sm_call(fw, cmd_index, &size, arg0, arg1, arg2, arg3, arg4) < 0) {
		ret = -EINVAL;
		goto out;
	}

	if (size < 0 || size > bsize) {
		ret = -EINVAL;
		goto out;
	}

	ret = size;

	/* In some cases (for example GET_CHIP_ID command),
	 * SMC doesn't return the number of bytes read, even
	 * though the bytes were actually read into sm_shmem_out.
	 * So this check is needed.
	 */
	if (!size)
		size = bsize;

	if (buffer)
		memcpy(buffer, fw->sm_shmem_out_base, size);
out:
	mutex_unlock(&fw->shmem_lock);
	return ret;
}
EXPORT_SYMBOL(meson_sm_call_read);

/**
 * meson_sm_call_write - send data to secure-monitor
 *
 * @fw:		Pointer to secure-monitor firmware
 * @buffer:	Buffer containing data to send
 * @size:	Size of the data to send
 * @cmd_index:	Index of the SMC32 function ID
 * @arg0:	SMC32 Argument 0
 * @arg1:	SMC32 Argument 1
 * @arg2:	SMC32 Argument 2
 * @arg3:	SMC32 Argument 3
 * @arg4:	SMC32 Argument 4
 *
 * Return:	size of sent data on success, a negative value on error
 */
int meson_sm_call_write(struct meson_sm_firmware *fw, void *buffer,
			unsigned int size, unsigned int cmd_index, u32 arg0,
			u32 arg1, u32 arg2, u32 arg3, u32 arg4)
{
	s32 written;

	if (!fw->chip)
		return -ENOENT;

	if (size > fw->chip->shmem_size)
		return -EINVAL;

	if (!fw->chip->cmd_shmem_in_base)
		return -EINVAL;

	mutex_lock(&fw->shmem_lock);
	memcpy(fw->sm_shmem_in_base, buffer, size);

	if (meson_sm_call(fw, cmd_index, &written, arg0, arg1, arg2, arg3, arg4) < 0)
		written = -EINVAL;
	mutex_unlock(&fw->shmem_lock);

	if (written <= 0 || written > size)
		return -EINVAL;

	return written;
}
EXPORT_SYMBOL(meson_sm_call_write);

/**
 * meson_sm_call_smc64 - SMC64 call through the shared buffers, SMCCC 1.2
 *
 * @fw:		Pointer to secure-monitor firmware
 * @regs:	x0 = function id, x1..x17 arguments in; x0..x17 results out
 * @in:		Bytes copied into the input buffer before the call, or NULL
 * @in_len:	Their count
 * @out:	Buffer filled from the output buffer after the call, or NULL
 * @out_len:	Bytes to copy out
 *
 * For services that return more than one register (the device credential
 * service, meson_devid.c).  Holds the buffers for the whole call.
 *
 * Return:	0, or a negative value if a length exceeds the buffers
 */
#ifdef CONFIG_ARM64
int meson_sm_call_smc64(struct meson_sm_firmware *fw,
			struct arm_smccc_1_2_regs *regs, const void *in,
			size_t in_len, void *out, size_t out_len)
{
	struct arm_smccc_1_2_regs args = *regs;

	if (!fw->chip)
		return -ENOENT;
	if ((in && in_len > fw->chip->shmem_size) ||
	    (out && out_len > fw->chip->shmem_size))
		return -EINVAL;

	mutex_lock(&fw->shmem_lock);
	if (in)
		memcpy(fw->sm_shmem_in_base, in, in_len);
	arm_smccc_1_2_smc(&args, regs);
	if (out)
		memcpy(out, fw->sm_shmem_out_base, out_len);
	mutex_unlock(&fw->shmem_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(meson_sm_call_smc64);
#endif /* SMCCC 1.2 register calls exist on arm64 only (MESON_SM_DEVID) */

/**
 * meson_sm_get - get pointer to meson_sm_firmware structure.
 *
 * @sm_node:		Pointer to the secure-monitor Device Tree node.
 *
 * Return:		NULL is the secure-monitor device is not ready.
 */
struct meson_sm_firmware *meson_sm_get(struct device_node *sm_node)
{
	struct platform_device *pdev = of_find_device_by_node(sm_node);
	struct meson_sm_firmware *fw;

	if (!pdev)
		return NULL;

	fw = platform_get_drvdata(pdev);

	put_device(&pdev->dev);

	return fw;
}
EXPORT_SYMBOL_GPL(meson_sm_get);

#define SM_CHIP_ID_LENGTH	119
#define SM_CHIP_ID_OFFSET	4

/**
 * meson_sm_get_serial - read the SoC's unique chip id (GET_CHIP_ID version 1)
 *
 * @fw:		Pointer to secure-monitor firmware
 * @serial:	Buffer of MESON_SM_SERIAL_SIZE bytes for the id
 *
 * Return:	0 on success, a negative errno otherwise (an all-zero id, an
 *		unburned chip, is -ENODATA).
 */
int meson_sm_get_serial(struct meson_sm_firmware *fw, u8 *serial)
{
	u8 *id_buf;
	int ret;

	id_buf = kmalloc(SM_CHIP_ID_LENGTH, GFP_KERNEL);
	if (!id_buf)
		return -ENOMEM;

	ret = meson_sm_call_read(fw, id_buf, SM_CHIP_ID_LENGTH, SM_GET_CHIP_ID,
				 0, 0, 0, 0, 0);
	if (ret >= 0) {
		memcpy(serial, &id_buf[SM_CHIP_ID_OFFSET], MESON_SM_SERIAL_SIZE);
		ret = memchr_inv(serial, 0, MESON_SM_SERIAL_SIZE) ? 0 : -ENODATA;
	}

	kfree(id_buf);

	return ret;
}
EXPORT_SYMBOL_GPL(meson_sm_get_serial);

static ssize_t serial_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct meson_sm_firmware *fw = dev_get_drvdata(dev);
	u8 serial[MESON_SM_SERIAL_SIZE];
	int ret;

	ret = meson_sm_get_serial(fw, serial);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%*phN\n", MESON_SM_SERIAL_SIZE, serial);
}

/*
 * Root only: the chip id identifies one board for its whole life.  Its
 * standard home is /sys/devices/soc0/serial_number (meson-gx-socinfo).
 */
static DEVICE_ATTR_ADMIN_RO(serial);

static struct attribute *meson_sm_sysfs_attrs[] = {
	&dev_attr_serial.attr,
	NULL,
};
ATTRIBUTE_GROUPS(meson_sm_sysfs);

/*
 * The device credential service (TF-A AML_DEVID) answers its VERSION call
 * with the protocol number; any other firmware answers SMC_UNK.  Only a
 * firmware that has the service gets the child the meson-sm-devid driver
 * binds to.
 */
#define SM_DEVID_VERSION	0xC20000D3
#define SM_DEVID_PROTOCOL	1

static void meson_sm_add_devid(struct platform_device *pdev)
{
	struct platform_device *child;
	struct arm_smccc_res res;

	arm_smccc_smc(SM_DEVID_VERSION, 0, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0 != SM_DEVID_PROTOCOL)
		return;
	child = platform_device_register_data(&pdev->dev, "meson-sm-devid",
					      PLATFORM_DEVID_NONE, NULL, 0);
	if (IS_ERR(child))
		dev_warn(&pdev->dev, "device credential service: %pe\n", child);
	else
		dev_info(&pdev->dev, "device credential service, protocol %lu\n",
			 res.a0);
}

static const struct of_device_id meson_sm_ids[] = {
	{ .compatible = "amlogic,meson-gxbb-sm", .data = &gxbb_chip },
	{ /* sentinel */ },
};

static int __init meson_sm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct meson_sm_chip *chip;
	struct meson_sm_firmware *fw;

	fw = devm_kzalloc(dev, sizeof(*fw), GFP_KERNEL);
	if (!fw)
		return -ENOMEM;

	chip = device_get_match_data(dev);
	if (!chip)
		return -EINVAL;

	if (chip->cmd_shmem_in_base) {
		fw->sm_shmem_in_base = meson_sm_map_shmem(chip->cmd_shmem_in_base,
							  chip->shmem_size);
		if (WARN_ON(!fw->sm_shmem_in_base))
			goto out;
	}

	if (chip->cmd_shmem_out_base) {
		fw->sm_shmem_out_base = meson_sm_map_shmem(chip->cmd_shmem_out_base,
							   chip->shmem_size);
		if (WARN_ON(!fw->sm_shmem_out_base))
			goto unmap_in_base;
	}

	mutex_init(&fw->shmem_lock);
	fw->chip = chip;

	platform_set_drvdata(pdev, fw);

	if (devm_of_platform_populate(dev))
		goto unmap_out_base;

	pr_info("secure-monitor enabled\n");

	if (IS_ENABLED(CONFIG_MESON_SM_DEVID))
		meson_sm_add_devid(pdev);

	return 0;

unmap_out_base:
	iounmap(fw->sm_shmem_out_base);
unmap_in_base:
	iounmap(fw->sm_shmem_in_base);
out:
	return -EINVAL;
}

static struct platform_driver meson_sm_driver = {
	.driver = {
		.name = "meson-sm",
		.of_match_table = of_match_ptr(meson_sm_ids),
		.dev_groups = meson_sm_sysfs_groups,
	},
};

/*
 * subsys_initcall when built in: meson-gx-socinfo, a device_initcall, reads
 * the chip id through this driver for soc0's serial_number.
 */
static int __init meson_sm_init(void)
{
	return platform_driver_probe(&meson_sm_driver, meson_sm_probe);
}
subsys_initcall(meson_sm_init);

static void __exit meson_sm_exit(void)
{
	platform_driver_unregister(&meson_sm_driver);
}
module_exit(meson_sm_exit);

MODULE_DESCRIPTION("Amlogic Secure Monitor driver");
MODULE_LICENSE("GPL v2");
