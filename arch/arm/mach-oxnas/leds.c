/*
 * linux/arch/arm/mach-oxnas/leds.c
 *
 * Copyright (C) 2005 Oxford Semiconductor Ltd
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
#define DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/interrupt.h>

#include <linux/platform_device.h>


#include <linux/leds.h>
 
#include <asm/hardware.h>

#define DEBUG_PRINT(A) printk(KERN_NOTICE A)

#define writel(data,address) (*(volatile u32 *) address = data)
#define readl(address)       (*(volatile u32 *) address)

/* run pwm refresh at approximately 100Hz to avoid flicker */
/* resolution is 8bits, sys clock 200MHz divider is therefore 7812 less 1 cycle */
#define PWM_PERIOD  (7811)

#define MAX_PWMS     16

static void ramp_power_on_leds(unsigned long data);

DEFINE_TIMER (power_ramp_timer, ramp_power_on_leds, 0, 0);

enum { POWER_ON,
	NUMBER_LEDS};
	
static struct platform_device *oxnas_leds;	
static u16 offset[NUMBER_LEDS] = {25};

static u16 led [NUMBER_LEDS];

#define MAX_BRIGHTNESS   255

static void set_led(u16 led, u16 value)
{
	u16 led_index = offset[led] % MAX_PWMS;
	
	writel(value, (PWM_DATA_REGISTER_BASE+4*led_index));
	
}

static void ramp_power_on_leds(unsigned long data)
{
	if (led[POWER_ON] < MAX_BRIGHTNESS) {
		 set_led(POWER_ON, ++led[POWER_ON]);
		 mod_timer(&power_ramp_timer, (power_ramp_timer.expires + msecs_to_jiffies(64)) );
	}
	else del_timer(&power_ramp_timer);
}

static void oxnasled_power_on_set(struct led_classdev *led_cdev, enum led_brightness value)
{
	if (value == 0) {
		current_bright = 0;
		led[POWER_ON]=0;
	    set_led(POWER_ON, 0);
		
	}
	else
	{
		power_ramp_timer.expires = jiffies + msecs_to_jiffies(64);
		add_timer(&power_ramp_timer);
	}
}

static struct led_classdev oxnas_power_on_led = {
	.name			= "oxnas:power_on",
	.brightness_set		= oxnasled_power_on_set,
};


#ifdef CONFIG_PM

// TODO implement led suspend operation on NAS
static int oxnasled_suspend(struct platform_device *dev, pm_message_t state)
{
#ifdef CONFIG_LEDS_TRIGGERS
	if (oxnas_amber_led.trigger && strcmp(oxnas_amber_led.trigger->name, "sharpsl-charge"))
#endif
		led_classdev_suspend(&oxnas_amber_led);
	led_classdev_suspend(&oxnas_green_led);
	return 0;
}
// TODO implement led resume operation on NAS
static int oxnasled_resume(struct platform_device *dev)
{
	led_classdev_resume(&oxnas_amber_led);
	led_classdev_resume(&oxnas_green_led);
	return 0;
}
#endif

static int oxnasled_probe(struct platform_device *pdev)
{
	int ret;
	int i;

	writel(PWM_PERIOD, PWM_CLOCK_REGISTER);

	
	/* enable PWM drives outputs */
	for (i=0; i < NUMBER_LEDS ; ++i)
	{
		if (offset[i] < 32) {
			writel(readl(SYS_CTRL_GPIO_PWMSEL_CTRL_0) | (1 << offset[i]), SYS_CTRL_GPIO_PWMSEL_CTRL_0); 
		}
		else {
			writel(readl(SYS_CTRL_GPIO_PWMSEL_CTRL_1) | (1 << (offset[i]% 32)), SYS_CTRL_GPIO_PWMSEL_CTRL_1);
		}
	}

	ret = led_classdev_register(&pdev->dev, &oxnas_power_on_led);
	
	if (ret < 0) goto error_1;
	
	return ret;
	
error_1:		
	return ret;
}

static int oxnasled_remove(struct platform_device *pdev)
{
	int i;
	
	led_classdev_unregister(&oxnas_power_on_led);

	/* disable PWM drives outputs */
	for (i=0; i < NUMBER_LEDS ; ++i)
	{
		if (offset[i] < 32) {
			writel(readl(SYS_CTRL_GPIO_PWMSEL_CTRL_0) & ~((u32)1 << offset[i]), SYS_CTRL_GPIO_PWMSEL_CTRL_0); 
		}
		else {
			writel(readl(SYS_CTRL_GPIO_PWMSEL_CTRL_1) & ~((u32)1 << (offset[i]% 32)), SYS_CTRL_GPIO_PWMSEL_CTRL_1);
		}
	}
	
	writel(PWM_CLOCK_REGISTER, 0);
	
	return 0;
}


static struct platform_driver oxnasled_driver = {
	.probe		= oxnasled_probe,
	.remove		= oxnasled_remove,
#ifdef CONFIG_PM
	.suspend	= oxnasled_suspend,
	.resume		= oxnasled_resume,
#endif
	.driver		= {
		.name		= "oxnas-leds",
		.owner      = THIS_MODULE,
	},
};

static int __init oxnasled_init(void)
{
	int ret;

	ret = platform_driver_register(&oxnasled_driver);
	
	
	
	/* now register the devices on the bus so they can be associated with the driver */
	if (!ret) 
			oxnas_leds=platform_device_register_simple("oxnas-leds", -1, NULL, 0);
	return ret;
}

static void __exit oxnasled_exit(void)
{
	if (oxnas_leds) {
		platform_device_unregister(oxnas_leds);
	}

 	platform_driver_unregister(&oxnasled_driver);
}

module_init(oxnasled_init);
module_exit(oxnasled_exit);

MODULE_AUTHOR("John Larkworthy <john.larkworthy@oxsem.com");
MODULE_DESCRIPTION("OXNAS front panel LED driver");
MODULE_LICENSE("GPL");
