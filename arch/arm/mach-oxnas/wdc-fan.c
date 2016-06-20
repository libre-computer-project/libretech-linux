/*
 * linux/arch/arm/mach-oxnas/wdc-fan.c
 *
 * Copyright (C) 2006-2007 Western Digital
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
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <asm/hardware.h>

/* Human recognizable driver name. */
#define DRIVER_NAME           "WDC_Fan"

/* GPIOs for the fan on the Galaxy 2NC platform. All are on GPIO_A */
#define FAN_MASK_LOW          (1 << GPIO_29)
#define FAN_MASK_HIGH         (1 << GPIO_8)

#define FAN_MASK              ( FAN_MASK_LOW  | FAN_MASK_HIGH )


/* I/O register access (FIXME: why not use the standard linux macros?) */
#define ox_writel(data, addr) (*(volatile unsigned long*)addr = (data))
#define ox_readl(addr)        (*(volatile unsigned long*)addr)
#define writel(data, addr)    (*(volatile u32*)addr = (data))
#define readl(addr)           (*(volatile u32*)addr)
#define CLEAR(addr, mask)     writel(readl(addr) & ~mask, addr)

enum fan_speeds
{
   FAN_OFF  = 0,
   FAN_SPEED_MAX  = 100
};

typedef struct s_fan_device_state
{
   unsigned char speed;       /* Range FAN_OFF .. FAN_SPEED_MAX */
} fan_device_state;


/*
 * Driver-global variables.
 */

/* Number of successful probes. */
/* TODO: protect this variable??!? */
static int  fans_found;

/* This spinlock protects the GPIO setup code from
   interrupts but is not SMP-correct. */
static spinlock_t oxnas_gpio_spinlock;

/* Device instance state.
   Only one fan is supported, so this can be a global variable. */
static fan_device_state  fan_state;


/*
 * Device attribute getter/setters
 */
static ssize_t fan_speed_show (struct device*, struct device_attribute*,
                               char *buf);

static ssize_t fan_speed_store(struct device*, struct device_attribute*,
                               const char *buf, size_t count);

/* Declare device attributes using the functions above.
   Contrary to the macro's name, DEVICE_ATTR declares a dev_attr_...  */
static DEVICE_ATTR(speed, 0644, fan_speed_show, fan_speed_store);


static void set_fan_speed(unsigned char speed);

static int  fan_probe (struct platform_device *pdev);
static int  fan_remove(struct platform_device *pdev);



/***************************************************************************/
/* FUNCTION: fan_probe                                                     */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Look for fans and do initialize.                                      */
/***************************************************************************/
int fan_probe(struct platform_device *pdev)
{
   int  rc = 0;

   do {
      unsigned long lock_flags;

      printk(KERN_DEBUG "Fan probe\n");

      memset(&fan_state, sizeof(fan_state), 0);

      /* Setup the fan-control GPIO lines properly. */
      spin_lock_irqsave(&oxnas_gpio_spinlock, lock_flags);

      do {
         /* Enable desired GPIO drivers by disabling other functions. */
         CLEAR(SYS_CTRL_GPIO_PRIMSEL_CTRL_0, FAN_MASK);
         CLEAR(SYS_CTRL_GPIO_SECSEL_CTRL_0,  FAN_MASK);
         CLEAR(SYS_CTRL_GPIO_TERTSEL_CTRL_0, FAN_MASK);

         /* Turn off the fan... then enable its outputs. */
         writel(FAN_MASK, GPIO_A_OUTPUT_CLEAR);
         writel(FAN_MASK, GPIO_A_OUTPUT_ENABLE_SET);

      } while (0);

      spin_unlock_irqrestore(&oxnas_gpio_spinlock, lock_flags);


      /* Create an entry in sysfs so user apps can control the fan. */
      rc = device_create_file(&pdev->dev, &dev_attr_speed);
      if (rc < 0)  break;

      fans_found++;

      set_fan_speed(FAN_OFF);

   } while(0);


   /* Cleanup if any errors occured. */
   if(rc < 0)
   {
      fan_remove(pdev);
   }

   return rc;
}


/***************************************************************************/
/* FUNCTION: fan_remove                                                    */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Deactive the fan device(s)                                            */
/***************************************************************************/
int fan_remove(struct platform_device *pdev)
{
   printk(KERN_DEBUG "Fan remove\n");

   /* Undo everything that might have been done by  fan_probe() */

   device_remove_file(&pdev->dev, &dev_attr_speed);

   writel(FAN_MASK, GPIO_A_OUTPUT_ENABLE_CLEAR);

   return 0;
}


/***************************************************************************/
/*                                                                         */
/* sysfs attribute I/O - user-space control points                         */
/*                                                                         */
/***************************************************************************/

/* fan_speed_show()
 *
 * Called when the speed setting attribute is read;
 * returns the current fan speed.
 */
ssize_t fan_speed_show(struct device *dev, struct device_attribute *attr,
                       char *buf)
{
   /* TODO: Check sizeof  buf?  Nobody else does*/

	return  sprintf(buf, "%u\n", fan_state.speed);
}

/* fan_speed_store()
 *
 * Called when the speed setting attribute is written;
 * sets a new fan speed.
 */
ssize_t fan_speed_store(struct device *dev, struct device_attribute *attr,
                        const char *buf, size_t count)
{
   char *end;
   long newSpeed;

   newSpeed = simple_strtol(buf, &end, 10);
   while (*end)
   {
      if (!isspace(*end++))  return -EINVAL;
   }

   if (newSpeed < 0)  newSpeed = 0;
   if (newSpeed > FAN_SPEED_MAX)  newSpeed = FAN_SPEED_MAX;

   set_fan_speed( (unsigned char)newSpeed );

   return count;              /* Entire string was consumed and validated. */
}

/* set_fan_speed()
 *
 * Sets the fan's speed by twiddling its power control (GPO) lines.
 */
void set_fan_speed(unsigned char newSpeed)
{
int oldSpeed = fan_state.speed;

   /* Round the requested fan speed to the nearest supported
      value and set the fan's power lines appropriately. */

   /* The 2NC platform has three fan settings: off, low, hi-speed. */

   if (newSpeed < 10)
   {
      fan_state.speed = FAN_OFF;       /* Turn off the fan. */
      writel(FAN_MASK_LOW | FAN_MASK_HIGH, GPIO_A_OUTPUT_CLEAR);
   }
   else if (newSpeed < 75)
   {
      fan_state.speed = 50;            /* Set to low speed. */
      writel(FAN_MASK_HIGH, GPIO_A_OUTPUT_CLEAR);
      writel(FAN_MASK_LOW,  GPIO_A_OUTPUT_SET);
   }
   else
   {
      fan_state.speed = FAN_SPEED_MAX; /* Set to max (high) speed. */
      writel(FAN_MASK_LOW | FAN_MASK_HIGH, GPIO_A_OUTPUT_SET);
   }

   if (oldSpeed != fan_state.speed)
   {
#if 0
      printk(KERN_DEBUG "WDC Fan speed set %u\n", fan_state.speed);
#endif
   }
}


/***************************************************************************/
/* DATA STRUCTURE: wdc_leds_driver                                         */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Describe the wdc-leds platform device driver                          */
/***************************************************************************/
static struct platform_driver fan_driver =
{
   .probe    = fan_probe,
   .remove   = fan_remove,
   .driver   =
   {
      .name  = "wdc-fan",
   },
};

static struct platform_device *fan_device;

/***************************************************************************/
/* FUNCTION: wdc_leds_init                                                 */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Perform module initialization                                         */
/***************************************************************************/
static int __init wdc_fan_init(void)
{
   int rc = 0;

   fans_found = 0;
   spin_lock_init(&oxnas_gpio_spinlock);

   rc = platform_driver_register(&fan_driver);
   if (rc)  goto quit;

   fan_device = platform_device_register_simple("wdc-fan", -1, NULL, 0);
   if (IS_ERR(fan_device))
   {
      rc = PTR_ERR(fan_device);
      fan_device = NULL;

      platform_driver_unregister(&fan_driver);
      goto quit;
   }


quit:
   if (rc)
   {
      printk(KERN_ERR  DRIVER_NAME " init failed, rc=%i\n", rc);
   }
   else
   {
      printk(KERN_INFO DRIVER_NAME " initialized\n");
   }

   return rc;
}


/***************************************************************************/
/* FUNCTION: wdc_leds_exit                                                 */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Perform module unloading and cleanup                                  */
/***************************************************************************/
static void __exit wdc_fan_exit(void)
{
   platform_device_unregister(fan_device);
   platform_driver_unregister(&fan_driver);

   printk(KERN_INFO DRIVER_NAME " goodbye!\n");
}


module_init(wdc_fan_init);
module_exit(wdc_fan_exit);

MODULE_AUTHOR("James Lin");
MODULE_DESCRIPTION("Western Digital NetCenter/2NC Fan Control");
MODULE_LICENSE("GPL");

/*EOF*/
