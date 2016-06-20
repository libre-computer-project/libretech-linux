/*
 * linux/arch/arm/mach-oxnas/wdc-ledtrig-sata.c
 *
 * Copyright (C) 2006 Western Digital
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/leds.h>


DEFINE_LED_TRIGGER(wdc_ledtrig_sata);


/***************************************************************************/
/* FUNCTION: wdc_ledtrig_sata_activity                                     */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Entry point to trip the trigger (called from within SATA driver)      */
/***************************************************************************/
void wdc_ledtrig_sata_activity(void)
{
   led_trigger_event(wdc_ledtrig_sata, LED_FULL);
}
EXPORT_SYMBOL(wdc_ledtrig_sata_activity);


/***************************************************************************/
/* FUNCTION: wdc_ledtrig_sata_init                                         */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Perform module initialization                                         */
/***************************************************************************/
static int __init wdc_ledtrig_sata_init(void)
{
   printk("<1>Hello, LED trigger\n");

   led_trigger_register_simple("sata-disk", &wdc_ledtrig_sata);
   return 0;
}


/***************************************************************************/
/* FUNCTION: wdc_ledtrig_sata_exit                                         */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Perform module unloading and cleanup                                  */
/***************************************************************************/
static void __exit wdc_ledtrig_sata_exit(void)
{
   led_trigger_unregister_simple(wdc_ledtrig_sata);

   printk("<1>Goodbye LED trigger\n");
}


module_init(wdc_ledtrig_sata_init);
module_exit(wdc_ledtrig_sata_exit);

MODULE_AUTHOR("Michael Webster");
MODULE_DESCRIPTION("WDC 2NC LED trigger");
MODULE_LICENSE("GPL");

/******************************* End of File *********************************/
