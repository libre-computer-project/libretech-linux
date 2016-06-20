/*
 *  arch/arm/mach-oxnas/oxnas-ahb-monitor.c
 *
 *  Copyright (C) 2006 Oxford Semiconductor Ltd
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
 *
 */

#include <linux/types.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/smp_lock.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>
#include <linux/capability.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/smp_lock.h>
#include <linux/wait.h>
#include <linux/suspend.h>
#include <linux/kthread.h>
#include <linux/moduleparam.h>

#include <asm/io.h>
#include <asm/system.h>
#include <asm/sections.h>
#include <asm/uaccess.h>
#include <asm/bitops.h>

#include <asm/hardware.h>


/* usb test masks and offsets */
#define TEST_MASK    0xF
#define TEST_OFFSET  16

#define MODULE_VERS "0.1"
#define MODULE_NAME "oxnas-test"
MODULE_AUTHOR(		"John Larkworthy"					);
MODULE_DESCRIPTION(	"Driver to access the test hardware in oxnas units"	);
MODULE_LICENSE(		"GPL"							);


static struct proc_dir_entry *proc_dir_usb_test_read, *oxnas_test_dir;


static struct {
	void * address;
	char * name;
	long unsigned low_add;
	long unsigned high_add;
	unsigned burst;
	unsigned burst_mask;
	unsigned hprot;
	unsigned hprot_mask;
	unsigned mode;
} monitor[] =
{
	{ (void *) 0x00000, "ARM_Data", 0, 0xFFFFFFFFl, 0, 0, 0, 0, 0 },
	{ (void *) 0x10000, "Arm_Inst", 0, 0xFFFFFFFFl, 0, 0, 0, 0, 0 },
	{ (void *) 0x20000, "DMA_A", 0, 0xFFFFFFFFl, 0, 0, 0, 0, 0 },
	{ (void *) 0x30000, "DMA_B", 0, 0xFFFFFFFFl, 0, 0, 0, 0, 0 },
	{ (void *) 0x40000, "CoPro", 0, 0xFFFFFFFFl, 0, 0, 0, 0, 0 },
	{ (void *) 0x50000, "USBHS", 0, 0xFFFFFFFFl, 0, 0, 0, 0, 0 },
	{ (void *) 0x60000, "GMAC", 0, 0xFFFFFFFFl, 0, 0, 0, 0, 0 },
	{ (void *) 0x70000, "PCI", 0, 0xFFFFFFFFl, 0, 0, 0, 0, 0 }
};		
#define NO_MONITORS (sizeof(monitor)/sizeof(monitor[0]))

/* create proc filing system entries to accept configuration data */
static int usb_test_write_entries(const char *name, write_proc_t *w, read_proc_t *r, int data)
{
	struct proc_dir_entry * entry = create_proc_entry(name, 0222, oxnas_test_dir);
	if (entry) {
		entry->write_proc = w;
		entry->read_proc =r;
		entry->data = (void *)data;
		entry->owner = THIS_MODULE;
		return 0;
	}
	else
	{
		return -ENOMEM;
	}
}

static int
oxnas_test_read(char *buf, char **start, off_t offset,
		  int count, int *eof, void *unused)
{
	
	int i;
	int len = 0;
	long unsigned *rd_monitor;
	
	
	for (i=0; i < NO_MONITORS; i++)
	{
	rd_monitor = (long unsigned *) (AHB_MON_BASE + monitor[i].address);
		len += sprintf(buf+len, "%s	CL:%ld EV:%ld hld:%ld slw:%lx tm:%ld\n", 
			monitor[i].name, 
			*rd_monitor, 
			*(rd_monitor+1),
			*(rd_monitor+2),
			*(rd_monitor+3),
			(*(rd_monitor+4) & 0xFFFF));
	}
	*eof=1;
	return len;
}
/*
 * function to clear and start all the timers mainly together.
 */
#define MAX_CMD 5
static int
oxnas_test_control(struct file *file, const char *buf, unsigned long count, void * data)
{
	int len;
	int i;
	long unsigned *rd_monitor;
	unsigned cmd = 0;
	
	char local[MAX_CMD];
	int result;

	if (count > MAX_CMD-1)
		len= MAX_CMD-1;
	else 
		len=count;
		
	if (copy_from_user(&local, buf, len))
		return -EFAULT;
	
	result=sscanf(local, "%d", &cmd);
	
	switch (cmd) 
	{
		case 0: 
			printk(KERN_INFO "oxnas-test: stop command\n");
			break;
		case 1:
			printk(KERN_INFO "oxnas-test: run command\n");
			break;
		case 2:
			printk(KERN_INFO "oxnas-test: reset command\n");
			break;
		default:
			printk(KERN_INFO "oxnas-test: ignored command\n");
			return len;
			break;
	}
	
	for (i=0; i < NO_MONITORS; i++)
	{
		rd_monitor = (long unsigned *) (AHB_MON_BASE + monitor[i].address);
		*rd_monitor = (long unsigned) cmd;
	}
	return len;
}


/*
 * The write function accepts a line as below:
 * start_addr, end_addr, mode, burst, burst_mask, hprot, hprot_mask 
 * expected string length is 10 + 10 + 3 + 3 + 4 + 3 + 4 < 40char.
 * This is decoded by the scanf function into the separate items. - missing items are defaulted.
 */
 
#define MAX_STRING 40
static int
oxnas_test_config_write(struct file *file, const char *buf, unsigned long count, void * data)
{
	
	int len;	
	int i = (int) data;
	char local[MAX_STRING];
	int result;
	unsigned long * mon_ptr;

	if (count > MAX_STRING-1)
		len= MAX_STRING-1;
	else 
		len=count;
		
	if (copy_from_user(&local, buf, len))
		return -EFAULT;
	
	/* extract value from buffer and store */
	
	result = sscanf(local, "%li,%li,%i,%i,%i,%i,%i", 
		&monitor[i].low_add, 
		&monitor[i].high_add, 
		&monitor[i].mode, 
		&monitor[i].burst, 
		&monitor[i].burst_mask, 
		&monitor[i].hprot,
		&monitor[i].hprot_mask
	);
	if (result != 7)
		return -EINVAL;
		
	/* load values on hardware */
	
	mon_ptr=(unsigned long *) (AHB_MON_BASE + monitor[i].address);

	*(mon_ptr + 1) = monitor[i].mode & 0x3;
	*(mon_ptr + 2) = monitor[i].low_add;
	*(mon_ptr + 3) = monitor[i].high_add;
	*(mon_ptr + 4) = ((monitor[i].burst & 0x7) << 4 | (monitor[i].burst_mask & 0x7));
	*(mon_ptr + 5) = ((monitor[i].hprot & 0xf) << 4 | (monitor[i].hprot_mask &0xf));
		
	return len;
}

static int
oxnas_test_config_read(char *buf, char **start, off_t offset,
		  int count, int *eof, void *data)
{
	
	int len = 0;	
	int i = (int) data;
	
	len += sprintf(buf+len, "name low  high  mode burst/mask hprot/mask\n"); 

	len += sprintf(buf+len, "%s	0x%08lx 0x%08lx %d 0x%x/0x%x 0x%x/0x%x\n", 
		monitor[i].name, 
		monitor[i].low_add, 
		monitor[i].high_add, 
		monitor[i].mode, 
		monitor[i].burst, 
		monitor[i].burst_mask, 
		monitor[i].hprot,
		monitor[i].hprot_mask);
		
		
	*eof=1;
	return len;
}

static int __init oxnas_test_init(void)
{
	int rv=0;
	int i;
	
	oxnas_test_dir = proc_mkdir(MODULE_NAME, NULL);
	if (oxnas_test_dir == NULL) {
		printk(KERN_ERR "oxnas-test: unable to register /proc/usb-test\n");
		rv= -ENOMEM;
		goto out;
	}
	
	oxnas_test_dir->owner= THIS_MODULE;
	
	proc_dir_usb_test_read = create_proc_entry("read", 0444, oxnas_test_dir);
	if (proc_dir_usb_test_read) {
		proc_dir_usb_test_read->read_proc = oxnas_test_read;
	} else {
		printk(KERN_ERR "oxnas-test: unable to register /proc/usb-test/read\n");
		rv = -ENOMEM;
		goto no_read;
	}

	/* create port write file entries */
	for (i=0;i<NO_MONITORS;i++) 
	{
		rv = usb_test_write_entries(monitor[i].name, &oxnas_test_config_write, &oxnas_test_config_read, i);
		if (rv < 0)
		{
			while (i != 0)
			{
				i--;
				/* remove any allocated entries */
				remove_proc_entry (monitor[i].name, oxnas_test_dir);
			} 
			goto no_write;
		}
	}

	{
		struct proc_dir_entry * entry = create_proc_entry("control", 0666, oxnas_test_dir);
		if (entry) {
			entry->write_proc = oxnas_test_control;
			entry->owner = THIS_MODULE;
			return 0;
		}
		else
		{
			goto no_control;
		}
	}


	printk(KERN_INFO "%s %s initialised\n", MODULE_NAME, MODULE_VERS);

	return 0;

	no_control:
		for (i = NO_MONITORS; i != 0; )
		{
			i--;
			/* remove any allocated entries */
			remove_proc_entry (monitor[i].name, oxnas_test_dir);
		} 
	
	no_write:
		remove_proc_entry("read", oxnas_test_dir);
	no_read:
		remove_proc_entry(MODULE_NAME, NULL);
	out:
		return rv;
}


static void __exit oxnas_test_exit(void)
{
	int i;
	
	remove_proc_entry("control", oxnas_test_dir);
	
	for (i = 0; i < NO_MONITORS; i++)
	{
		remove_proc_entry(monitor[i].name, oxnas_test_dir);
	}
	
	remove_proc_entry("read", oxnas_test_dir);
	remove_proc_entry(MODULE_NAME, NULL);

	printk(KERN_INFO "%s %s removed\n", MODULE_NAME, MODULE_VERS);
	
}


module_init(oxnas_test_init);
module_exit(oxnas_test_exit);






















