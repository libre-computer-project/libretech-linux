/*
 * linux/arch/arm/mach-oxnas/leds.c
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/leds.h>

static void ledtrig_ide_timerfunc(unsigned long data);

DEFINE_LED_TRIGGER(ledtrig_ide);
static DEFINE_TIMER(ledtrig_ide_timer, ledtrig_ide_timerfunc, 0, 0);
static int ide_activity;
static int ide_lastactivity;

void ledtrig_sata_activity(void)
{
	ide_activity++;
	if (!timer_pending(&ledtrig_ide_timer))
		mod_timer(&ledtrig_ide_timer, jiffies + msecs_to_jiffies(10));
}
EXPORT_SYMBOL(ledtrig_sata_activity);

static void ledtrig_ide_timerfunc(unsigned long data)
{
	if (ide_lastactivity != ide_activity) {
		ide_lastactivity = ide_activity;
		led_trigger_event(ledtrig_ide, LED_FULL);
	    	mod_timer(&ledtrig_ide_timer, jiffies + msecs_to_jiffies(10));
	} else {
		led_trigger_event(ledtrig_ide, LED_OFF);
	}
}

static int __init ledtrig_ide_init(void)
{
	led_trigger_register_simple("sata-disk", &ledtrig_ide);
	return 0;
}

static void __exit ledtrig_ide_exit(void)
{
	led_trigger_unregister_simple(ledtrig_ide);
}

module_init(ledtrig_ide_init);
module_exit(ledtrig_ide_exit);

MODULE_AUTHOR("John Larkworthy <john.larkworthy@oxnas.com>");
MODULE_DESCRIPTION("LED SATA Disk Activity Trigger");
MODULE_LICENSE("GPL");
