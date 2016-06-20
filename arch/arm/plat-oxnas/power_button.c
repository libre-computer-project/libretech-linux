/*
 * linux/arch/arm/mach-oxnas/power_button.c
 *
 * Copyright (C) 2006 Oxford Semiconductor Ltd
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
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/kobject.h>
#include <linux/workqueue.h>
#include <asm/hardware.h>
#include <asm/io.h>

MODULE_LICENSE("GPL v2");

// Global variable to hold LED inversion state
extern int oxnas_global_invert_leds;

// Make a module parameter to set whether LEDs are inverted 
static int invert_leds = 0;
module_param(invert_leds, bool, S_IRUGO|S_IWUSR);

#define DEFAULT_TIMER_COUNT_LIMIT 24	/* In eigths of a second */

static int timer_count_limit = DEFAULT_TIMER_COUNT_LIMIT;
module_param(timer_count_limit, int, S_IRUGO|S_IWUSR);

#if (CONFIG_OXNAS_POWER_BUTTON_GPIO < 32)
#define SWITCH_NUM          CONFIG_OXNAS_POWER_BUTTON_GPIO
#define IRQ_NUM             GPIO_1_INTERRUPT
#define INT_STATUS_REG      GPIO_A_INTERRUPT_STATUS_REGISTER
#define SWITCH_PRISEL_REG   SYS_CTRL_GPIO_PRIMSEL_CTRL_0
#define SWITCH_SECSEL_REG   SYS_CTRL_GPIO_SECSEL_CTRL_0
#define SWITCH_TERSEL_REG   SYS_CTRL_GPIO_TERTSEL_CTRL_0
#define SWITCH_CLR_OE_REG   GPIO_A_OUTPUT_ENABLE_CLEAR
#define DEBOUNCE_REG        GPIO_A_INPUT_DEBOUNCE_ENABLE
#define LEVEL_INT_REG       GPIO_A_LEVEL_INTERRUPT_ENABLE
#define FALLING_INT_REG     GPIO_A_FALLING_EDGE_ACTIVE_LOW_ENABLE
#define DATA_REG            GPIO_A_DATA
#else
#define SWITCH_NUM          ((CONFIG_OXNAS_POWER_BUTTON_GPIO) - 32)
#define IRQ_NUM             GPIO_2_INTERRUPT
#define INT_STATUS_REG      GPIO_B_INTERRUPT_STATUS_REGISTER
#define SWITCH_PRISEL_REG   SYS_CTRL_GPIO_PRIMSEL_CTRL_1
#define SWITCH_SECSEL_REG   SYS_CTRL_GPIO_SECSEL_CTRL_1
#define SWITCH_TERSEL_REG   SYS_CTRL_GPIO_TERTSEL_CTRL_1
#define SWITCH_CLR_OE_REG   GPIO_B_OUTPUT_ENABLE_CLEAR
#define DEBOUNCE_REG        GPIO_B_INPUT_DEBOUNCE_ENABLE
#define LEVEL_INT_REG       GPIO_B_LEVEL_INTERRUPT_ENABLE
#define FALLING_INT_REG     GPIO_B_FALLING_EDGE_ACTIVE_LOW_ENABLE
#define DATA_REG            GPIO_B_DATA
#endif

#define SWITCH_MASK (1UL << (SWITCH_NUM))

#define TIMER_INTERVAL_JIFFIES  ((HZ) >> 3) /* An eigth of a second */

extern spinlock_t oxnas_gpio_spinlock;

static unsigned long count;
static struct timer_list timer;

/** Have to use active low level interupt generation, as otherwise might miss
 *  interrupts that arrive concurrently with a PCI interrupt, as PCI interrupts
 *  are generated via GPIO pins and std PCI drivers will not know that there
 *  may be other pending GPIO interrupt sources waiting to be serviced and will
 *  simply return IRQ_HANDLED if they see themselves as having generated the
 *  interrupt, thus preventing later chained handlers from being called
 */
static irqreturn_t int_handler(int irq, void* dev_id)
{
	int status = IRQ_NONE;
	unsigned int int_status = readl((volatile unsigned long *)INT_STATUS_REG);

	/* Is the interrupt for us? */
	if (int_status & SWITCH_MASK) {
		/* Disable the power button GPIO line interrupt */
		spin_lock(&oxnas_gpio_spinlock);
		writel(readl(FALLING_INT_REG) & ~SWITCH_MASK, FALLING_INT_REG);
		spin_unlock(&oxnas_gpio_spinlock);

		/* Zeroise button hold down counter */
		count = 0;

		/* Start hold down timer with a timeout of 1/8 second */
		mod_timer(&timer, jiffies + TIMER_INTERVAL_JIFFIES);

		/* Only mark interrupt as serviced if no other unmasked GPIO interrupts
		are pending */
		if (!readl((volatile unsigned long *)INT_STATUS_REG)) {
			status = IRQ_HANDLED;
		}
	}

	return status;
}

/*
 * Device driver object
 */
typedef struct power_button_driver_s {
	/** sysfs dir tree root for power button driver */
	struct kset kset;
	struct kobject power_button;
} power_button_driver_t;

static power_button_driver_t power_button_driver;

static void work_handler(struct work_struct * not_used) {
	kobject_uevent(&power_button_driver.power_button, KOBJ_OFFLINE);
}

DECLARE_WORK(power_button_hotplug_work, work_handler);

static void timer_handler(unsigned long data)
{
	unsigned long flags;

	/* Is the power button still pressed? */
	if (!(readl(DATA_REG) & SWITCH_MASK)) {
		/* Yes, so increment count of how many timer intervals have passed since
		power button was pressed */
		if (++count == timer_count_limit) {
			schedule_work(&power_button_hotplug_work);
		} else {
			/* Restart timer with a timeout of 1/8 second */
			mod_timer(&timer, jiffies + TIMER_INTERVAL_JIFFIES);
		}
	} else {
		/* The h/w debounced power button has been released, so reenable the
		active low interrupt detection to trap the user's next attempt to
		power down */
		spin_lock_irqsave(&oxnas_gpio_spinlock, flags);
		writel(readl(FALLING_INT_REG) | SWITCH_MASK, FALLING_INT_REG);
		spin_unlock_irqrestore(&oxnas_gpio_spinlock, flags);
	}
}

static struct kobj_type ktype_power_button = {
	.release = 0,
	.sysfs_ops = 0,
	.default_attrs = 0,
};

static int power_button_hotplug_filter(struct kset* kset, struct kobject* kobj) {
	return get_ktype(kobj) == &ktype_power_button;
}

static const char* power_button_hotplug_name(struct kset* kset, struct kobject* kobj) {
	return "oxnas_power_button";
}

static struct kset_uevent_ops power_button_uevent_ops = {
	.filter = power_button_hotplug_filter,
	.name   = power_button_hotplug_name,
	.uevent = NULL,
};

static int power_button_prep_sysfs(void)
{
	int err = 0;

	/* prep the sysfs interface for use */
	kobject_set_name(&power_button_driver.kset.kobj, "power-button");
	power_button_driver.kset.ktype = &ktype_power_button;

	err = subsystem_register(&power_button_driver.kset);
	if (err)
		return err;

	/* setup hotplugging */
	power_button_driver.kset.uevent_ops = &power_button_uevent_ops;

	/* setup the heirarchy, the name will be set on detection */
	kobject_init(&power_button_driver.power_button);
	power_button_driver.power_button.kset = kset_get(&power_button_driver.kset);
	power_button_driver.power_button.parent = &power_button_driver.kset.kobj;

	return 0;
}

static int power_button_build_sysfs(void) {
	kobject_set_name(&power_button_driver.power_button, "power-button-1");
	return kobject_add(&power_button_driver.power_button);
}

static int __init power_button_init(void)
{
	int err = 0;
	unsigned long flags;

	/* Copy the LED inversion module parameter into the global variable */
	oxnas_global_invert_leds = invert_leds;

	err = power_button_prep_sysfs();
	if (err)
		return -EINVAL;

	err = power_button_build_sysfs();
	if (err)
		return -EINVAL;

	/* Setup the timer that will time how long the user holds down the power
	button */
	init_timer(&timer);
	timer.data = 0;
	timer.function = timer_handler;

	/* Install a shared interrupt handler on the appropriate GPIO bank's
	interrupt line */
	if (request_irq(IRQ_NUM, int_handler, IRQF_SHARED, "Power Button", &power_button_driver)) {
		printk(KERN_ERR "Power Button: cannot register IRQ %d\n", IRQ_NUM);
		del_timer_sync(&timer);
		return -EIO;
	}

	spin_lock_irqsave(&oxnas_gpio_spinlock, flags);
	/* Disable primary, secondary and teriary GPIO functions on switch lines */
	writel(readl(SWITCH_PRISEL_REG) & ~SWITCH_MASK, SWITCH_PRISEL_REG);
	writel(readl(SWITCH_SECSEL_REG) & ~SWITCH_MASK, SWITCH_SECSEL_REG);
	writel(readl(SWITCH_TERSEL_REG) & ~SWITCH_MASK, SWITCH_TERSEL_REG);

	/* Enable GPIO input on switch line */
	writel(SWITCH_MASK, SWITCH_CLR_OE_REG);

	/* Set up the power button GPIO line for active low, debounced interrupt */
	writel(readl(DEBOUNCE_REG)    | SWITCH_MASK, DEBOUNCE_REG);
	writel(readl(LEVEL_INT_REG)   | SWITCH_MASK, LEVEL_INT_REG);
	writel(readl(FALLING_INT_REG) | SWITCH_MASK, FALLING_INT_REG);
	spin_unlock_irqrestore(&oxnas_gpio_spinlock, flags);

	printk(KERN_INFO "Power button driver registered\n");
	return 0;
}

static void __exit power_button_exit(void)
{
	unsigned long flags;

	kobject_del(&power_button_driver.power_button);
	subsystem_unregister(&power_button_driver.kset);

	/* Deactive the timer */
	del_timer_sync(&timer);

	/* Disable interrupt generation by the power button GPIO line */
	spin_lock_irqsave(&oxnas_gpio_spinlock, flags);
	writel(readl(FALLING_INT_REG) & ~SWITCH_MASK, FALLING_INT_REG);
	spin_unlock_irqrestore(&oxnas_gpio_spinlock, flags);

	/* Remove the handler for the shared interrupt line */
	free_irq(IRQ_NUM, &power_button_driver);
}

/** 
 * macros to register intiialisation and exit functions with kernal
 */
module_init(power_button_init);
module_exit(power_button_exit);
