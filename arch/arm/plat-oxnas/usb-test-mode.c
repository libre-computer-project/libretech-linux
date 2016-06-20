/*
 *  arch/arm/mach-oxnas/usb-test-mode.c
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
#include <linux/interrupt.h>

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
#define MODULE_NAME "usb_test_mode"
MODULE_AUTHOR(		"John Larkworthy"					);
MODULE_DESCRIPTION(	"Driver to put usb ports in test modes"		);
MODULE_LICENSE(		"GPL"						);


static struct proc_dir_entry *proc_dir_usb_test_read, *usb_test_dir;


/* create proc filing system entries to accept configuration data */
static int usb_test_write_entries(const char *name, write_proc_t *w, int data)
{
	struct proc_dir_entry * entry = create_proc_entry(name, 0222, usb_test_dir);
	if (entry) {
		entry->write_proc = w;
		entry->data = (void *)data;
		entry->owner = THIS_MODULE;
		return 0;
	}
	else
	{
		return -ENOMEM;
	}
}



#if 0
static int
oxsemi_usb_test_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	//int i = (int __user *)arg;

	printk(KERN_INFO "usb test:: usb_test_ioctl\n");
	switch(cmd) {
	case SET_TEST_MODE:
		break;
	// etc...
		
	default:
		return -ENOIOCTLCMD;
	}

	return 0;
}
#endif

static int
oxsemi_usb_test_read(char *buf, char **start, off_t offset,
		  int count, int *eof, void *unused)
{
	
	int i;
	int len = 0;
	long unsigned *usbport;
	
	usbport = (long unsigned *) (USB_BASE+0x184);
	
	for (i=0; i < 3; i++)
	{
		len += sprintf(buf+len, "usb port %d [%p]:%08lx \n", i, (usbport+4*i) , *(usbport+i));
	}
	*eof=1;
	return len;
}

static int
oxsemi_usb_test_write(struct file *file, const char *buf, unsigned long count, void * data)
{
	int len;	
	long int *usbport;
	char local[10];
	int result;
	int test_mode;
	unsigned long flags;
	
	if (count > 9)
		len= 9;
	else 
		len=count;
		
	if (copy_from_user(&local, buf, len))
		return -EFAULT;
	
	/* extract value from buffer and store */
	result = sscanf(local, "%1d", &test_mode);
	if (result != 1)
		return -EINVAL;
		
	usbport = (long int *) (USB_BASE+0X184 + (4* (int) data));
	printk(KERN_ERR "usb-test-write : [%08lx] <- %08lx \n",
        (long unsigned) usbport,
        (long unsigned) ((test_mode & TEST_MASK) << TEST_OFFSET));
	/* lock system while this is updated */
	local_irq_save(flags);
 	*usbport = (*usbport & ~(TEST_MASK<<TEST_OFFSET)) | ((test_mode & TEST_MASK) << TEST_OFFSET);
	local_irq_restore(flags);
	printk(KERN_ERR "usb-test-writen: [%08lx]:%08lx \n", (long unsigned) usbport, (long unsigned) *usbport);
	return len;
}


static int __init oxsemi_usb_test_init(void)
{
	int rv;
	int i;
	char name[] = "usb-test/write0"; /* overwritten with new name below */
	
	usb_test_dir = proc_mkdir(MODULE_NAME, NULL);
	if (usb_test_dir == NULL) {
		printk(KERN_ERR "usb-test: unable to register /proc/usb-test\n");
		rv= -ENOMEM;
		goto out;
	}
	
	usb_test_dir->owner= THIS_MODULE;
	
	proc_dir_usb_test_read = create_proc_entry("read", 0444, usb_test_dir);
	if (proc_dir_usb_test_read) {
		proc_dir_usb_test_read->read_proc = oxsemi_usb_test_read;
	} else {
		printk(KERN_ERR "usb-test: unable to register /proc/usb-test/read\n");
		rv = -ENOMEM;
		goto no_read;
	}
	/* create port write file entries */
	for (i=0;i<3;i++) 
	{
		sprintf(name,"write%d",i+1);
		rv = usb_test_write_entries(name, &oxsemi_usb_test_write, i);
		if (rv < 0)
		{
			while (i != 0)
			{
				i--;
				/* remove any allocated entries */
				sprintf(name,"usb-test/write%d",i+1);
				remove_proc_entry (name, usb_test_dir);
			} 
			goto no_write;
		}
	}
	printk(KERN_INFO "%s %s initialised\n", MODULE_NAME, MODULE_VERS);

	return 0;
	no_write:
		remove_proc_entry("usb-test/read", usb_test_dir);
	no_read:
		remove_proc_entry(MODULE_NAME, NULL);
	out:
		return rv;
}


static void __exit oxsemi_usb_test_exit(void)
{
	char name[] =  "usb-test/write0";
	int i;
	
	for (i = 0; i < 3; i++)
	{
		sprintf(name, "write%1d", (i+1));
		remove_proc_entry(name, usb_test_dir);
	}
	
	remove_proc_entry("read", usb_test_dir);
	remove_proc_entry(MODULE_NAME, NULL);

	printk(KERN_INFO "%s %s removed\n", MODULE_NAME, MODULE_VERS);
	
}


module_init(oxsemi_usb_test_init);
module_exit(oxsemi_usb_test_exit);






















