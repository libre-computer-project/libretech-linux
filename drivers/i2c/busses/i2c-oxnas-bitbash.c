/*
 * drivers/i2c/busses/i2c_oxnas_bitbash.c
 *
 * Copyright (C) 2006-2008 Oxford Semiconductor Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <asm/hardware.h>
#include <asm/io.h>

#define I2C_OXNAS_BITBASH_I2C_SDA_OUT   (1UL << (CONFIG_OXNAS_I2C_SDA))
#define I2C_OXNAS_BITBASH_I2C_SCL_OUT   (1UL << (CONFIG_OXNAS_I2C_SCL))
#define	I2C_OXNAS_BB_PULSEWIDTH (40)
#define OPEN_COLLECTOR_CLOCK    1

extern spinlock_t oxnas_gpio_spinlock;

static void i2c_oxnas_bitbash_setsda(void *data,int state)
{
	if (state) {
		// tristae as input to set line on bus
		writel(I2C_OXNAS_BITBASH_I2C_SDA_OUT, GPIO_A_OUTPUT_ENABLE_CLEAR);
	} else {
		// tristate as output (with latch to zero) to assert zero on the bus
		writel(I2C_OXNAS_BITBASH_I2C_SDA_OUT, GPIO_A_OUTPUT_CLEAR);
		writel(I2C_OXNAS_BITBASH_I2C_SDA_OUT, GPIO_A_OUTPUT_ENABLE_SET);
	}
}

static void i2c_oxnas_bitbash_setscl(void *data,int state)
{
#if OPEN_COLLECTOR_CLOCK
	if (state) {
		// tristae as input to set line on bus
		writel(I2C_OXNAS_BITBASH_I2C_SCL_OUT, GPIO_A_OUTPUT_ENABLE_CLEAR);
	} else {
		// tristate as output (with latch to zero) to assert zero on the bus
		writel(I2C_OXNAS_BITBASH_I2C_SCL_OUT, GPIO_A_OUTPUT_CLEAR);
		writel(I2C_OXNAS_BITBASH_I2C_SCL_OUT, GPIO_A_OUTPUT_ENABLE_SET);
	}
#else // driven clock
	if (state) {
		// tristae as input to set line on bus
		writel(I2C_OXNAS_BITBASH_I2C_SCL_OUT, GPIO_A_OUTPUT_SET);
		writel(I2C_OXNAS_BITBASH_I2C_SCL_OUT, GPIO_A_OUTPUT_ENABLE_CLEAR);
	} else {
		// tristate as output (with latch to zero) to assert zero on the bus
		writel(I2C_OXNAS_BITBASH_I2C_SCL_OUT, GPIO_A_OUTPUT_CLEAR);
		writel(I2C_OXNAS_BITBASH_I2C_SCL_OUT, GPIO_A_OUTPUT_ENABLE_SET);
	}

#endif
}

static int i2c_oxnas_bitbash_getsda(void *data)
{
	return ((readl(GPIO_A_DATA ) & I2C_OXNAS_BITBASH_I2C_SDA_OUT) != 0);
}

static int i2c_oxnas_bitbash_getscl(void *data)
{
	return ((readl(GPIO_A_DATA ) & I2C_OXNAS_BITBASH_I2C_SCL_OUT) != 0);
}

static struct i2c_algo_bit_data bit_i2c_oxnas_bitbash_data = {
	.setsda    = i2c_oxnas_bitbash_setsda,
	.setscl    = i2c_oxnas_bitbash_setscl,
	.getsda    = i2c_oxnas_bitbash_getsda,
	.getscl    = i2c_oxnas_bitbash_getscl,
	.udelay    = I2C_OXNAS_BB_PULSEWIDTH,
	.timeout   = HZ
};

static struct i2c_adapter oxnas_i2c_bitbash_adapter = {
	.owner		= THIS_MODULE,
	.name		= "i2c_oxnas_bitbash adapter driver",
	.id		    = I2C_HW_B_OXNAS,
	.algo_data	= &bit_i2c_oxnas_bitbash_data,
};

static int __init i2c_oxnas_bitbash_init(void)
{
    unsigned long flags;
    unsigned long mask = I2C_OXNAS_BITBASH_I2C_SDA_OUT | I2C_OXNAS_BITBASH_I2C_SCL_OUT;
	int ret = 0;

	/* Dedicate the GPIO over to i2c.
     * NOTE: This may be confusing, but we are not using the i2c core here we
     * are using bit-bashed GPIO, so we must disable the primary, secondary and
     * tertiary functions of the relevant GPIO pins
     */
    spin_lock_irqsave(&oxnas_gpio_spinlock, flags);
    writel(readl(SYS_CTRL_GPIO_PRIMSEL_CTRL_0) & ~mask, SYS_CTRL_GPIO_PRIMSEL_CTRL_0);
    writel(readl(SYS_CTRL_GPIO_SECSEL_CTRL_0)  & ~mask, SYS_CTRL_GPIO_SECSEL_CTRL_0);
    writel(readl(SYS_CTRL_GPIO_TERTSEL_CTRL_0) & ~mask, SYS_CTRL_GPIO_TERTSEL_CTRL_0);
    spin_unlock_irqrestore(&oxnas_gpio_spinlock, flags);

	i2c_oxnas_bitbash_setsda(NULL, 1);
	i2c_oxnas_bitbash_setscl(NULL, 1);
	ret = i2c_bit_add_bus(&oxnas_i2c_bitbash_adapter);
	if (!ret) {
#if defined(CONFIG_OXNAS_RTC) || defined(CONFIG_OXNAS_RTC_MODULE)
		/* Register the ST MT4100 RTC */
		struct i2c_board_info rtc_info  = {
			.driver_name   = "rtc-ds1307",
			.type          = "m41t00",
			.flags         = 0,
			.addr          = 0x68,
			.platform_data = NULL,
			.irq           = 0
		};

		struct i2c_client *client = i2c_new_device(&oxnas_i2c_bitbash_adapter, &rtc_info);
		if (!client) {
			printk(KERN_WARNING "OXNAS bit-bash I2C driver failed to register RTC device\n");
			ret = -EIO;
		}
#endif // CONFIG_OXNAS_RTC || CONFIG_OXNAS_RTC_MODULE
	}

	printk(KERN_INFO "OXNAS bit-bash I2C driver initialisation %s\n", ret ? "failed": "OK");
	return ret;
}

static void __exit i2c_oxnas_bitbash_exit(void)
{
	i2c_oxnas_bitbash_setsda(NULL, 1);
	i2c_oxnas_bitbash_setscl(NULL, 1);
	i2c_del_adapter(&oxnas_i2c_bitbash_adapter);
}

MODULE_AUTHOR("Brian Clarke");
MODULE_DESCRIPTION("OXNAS bit-bash I2C bus driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("v2.0");

module_init (i2c_oxnas_bitbash_init);
module_exit (i2c_oxnas_bitbash_exit);

