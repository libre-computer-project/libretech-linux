/*
	Evevt.c - Receiver and send the wix events

	Copyright (c) 2004 - 2006  Aldofo Lin <aldofo_lin@wistron.com.tw>

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/


#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/pci.h>
#include <linux/blkdev.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/miscdevice.h>
#include <linux/vmalloc.h>
#include <linux/mman.h>
#include <linux/random.h>
#include <linux/raw.h>
#include <linux/tty.h>
#include <linux/capability.h>
#include <linux/smp_lock.h>
#include <linux/ptrace.h>
#include <linux/device.h>
#include <linux/time.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/pgalloc.h>
#include "wix.h"
#include "wixevent.h"


#define	EVENT_MAJOR	0
#define	EQUEUE_MAX_NUM	1024

#define DRV_NAME        "wixevent"
#define DRV_VERSION     "1.00"

#ifdef	DEBUG
	#define WIXDEBUG(fmt, args...) printk(KERN_ERR "(%s) : " fmt, __FUNCTION__, ## args)
#else
	#define	WIXDEBUG(fmt, args...)
#endif
#define WIXPRINT(fmt, args...) printk(KERN_INFO "(%s) " fmt, __FUNCTION__, ## args)


static ssize_t wixevent_read(struct file * file, char * buf, size_t count, loff_t *ppos);
static ssize_t wixevent_write(struct file * file, const char * buf, size_t count, loff_t *ppos);
static int wixevent_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);
static int proc_wixevent_count_read(char *page, char **start, off_t off, int count, int *eof, void *data);
int wixevent_send(wixEvent *event);


static wixEventList *WixEventHead, *WixEventTail;
static int wixEventCount;
static struct semaphore wixevent_semaphore;
static int wixevent_devmajor;
static struct file_operations wixevent_fops = {
	.read		= wixevent_read,
	.write		= wixevent_write,
	.ioctl		= wixevent_ioctl,
};
static struct class *wixevent_class;
struct proc_dir_entry *proc_wixevent;


static ssize_t wixevent_read ( struct file * file, char * buf, size_t count, loff_t *ppos ) {
	unsigned long left, written = 0;
	wixEventList *el;
	wixEvent *event;

	if ( !count )
		return 0;

	if ( !access_ok(VERIFY_WRITE, buf, count) )
		return -EFAULT;

	left = count;

	down( &wixevent_semaphore );

		while ( (WixEventHead != NULL) && (left >= sizeof(wixEvent)) ) {

			event = &WixEventHead->event;

			if ( copy_to_user(buf, event, sizeof(wixEvent)) ) {
				up( &wixevent_semaphore );
				return -EFAULT;
			}

			written += sizeof(wixEvent);
			buf += sizeof(wixEvent);
			left -= sizeof(wixEvent);

			if ( WixEventHead != WixEventTail ) {
				el = WixEventHead;
				WixEventHead = WixEventHead->next;
				kfree( el );
			} else {
				el = WixEventHead;
				WixEventHead = WixEventTail = NULL;
				kfree( el );
			}
		}

	up( &wixevent_semaphore );

	return written;
}


static ssize_t wixevent_write ( struct file * file, const char * buf, size_t count, loff_t *ppos ) {
	unsigned long left, read = 0;
	wixEventList *el;

	if ( !count )
		return 0;

	if ( !access_ok(VERIFY_READ, buf, count) )
		return -EFAULT;

	left = count;

	down( &wixevent_semaphore );

		while ( left >= sizeof(wixEventList) ) {

			if ( (el=kmalloc(sizeof(wixEventList), GFP_ATOMIC)) == NULL ) {
				WIXPRINT( "Unable to allocate memory !\n" );
				up( &wixevent_semaphore );
				return -ENOMEM;
			}

			if (copy_from_user(&el->event, buf, sizeof(wixEvent))) {
				up( &wixevent_semaphore );
				return -EFAULT;
			}

			read += sizeof(wixEvent);
			buf += sizeof(wixEvent);
			left -= sizeof(wixEvent);

			if ( WixEventHead == NULL ) {
				WixEventHead = WixEventTail = el;
			} else {
				WixEventTail->next = el;
				WixEventTail = el;
			}
			WixEventTail->next = NULL;

		}

	up( &wixevent_semaphore );

	return read;

}


static int wixevent_ioctl ( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg ) {
	wixEventList *last, *el;
	wixEvent event;
	int	res;

	if ( !access_ok(VERIFY_WRITE, arg, sizeof(wixEvent)) )
		return -EFAULT;

	switch ( cmd ) {
		case WIX_EVENT_IOCTL_SND:
		case WIX_EVENT_IOCTL_RCV:
		case WIX_EVENT_IOCTL_PEEK:
			if ( copy_from_user(&event, (wixEvent *)arg, sizeof(wixEvent)) ) {
				return -EFAULT;
			}
			break;
	}

	last = el = NULL;

	switch ( cmd ) {
		/* Push event to the event queue */
		case WIX_EVENT_IOCTL_SND:

			res = wixevent_send( &event );
			if ( res != WIX_RC_OK ) {
				return res;
			}
			break;

		/* Pull for currently pending events */
		/* If event.type == 0, pull the first event */
		/* If event.type != 0, pull the specific event of that type */
		case WIX_EVENT_IOCTL_RCV:

			down( &wixevent_semaphore );
			if ( WixEventHead == NULL ) {
				up( &wixevent_semaphore );
				return -EPERM;
			}
			if ( event.type == 0 ) {
				if ( copy_to_user((wixEvent *)arg, (wixEvent *)&WixEventHead->event, sizeof(wixEvent)) ) {
					up( &wixevent_semaphore );
					return -EFAULT;
				}
				if ( WixEventHead != WixEventTail ) {
					el = WixEventHead;
					WixEventHead = WixEventHead->next;
					kfree(el);
				} else {
					el = WixEventHead;
					WixEventHead = WixEventTail = NULL;
					kfree( el );
				}
				--wixEventCount;
			} else {
				el = WixEventHead;
				while ( event.type != el->event.type ) {
					last = el;
					el = el->next;
					if ( el == NULL ) {
						up( &wixevent_semaphore );
						return -EPERM;
					}
				}
				if ( el == WixEventHead ) {
					if ( WixEventHead != WixEventTail ) {
						WixEventHead = WixEventHead->next;
					} else {
						WixEventHead = WixEventTail = NULL;
					}
				} else if ( el == WixEventTail ) {
					WixEventTail = last;
					WixEventTail->next = NULL;
				} else {
					last->next = el->next;
				}
				if ( copy_to_user((wixEvent *)arg, (wixEvent *)&el->event, sizeof(wixEvent)) ) {
					up( &wixevent_semaphore );
					return -EFAULT;
				}
				kfree( el );
				--wixEventCount;
			}
			up( &wixevent_semaphore );
			break;

		/* Peek the event queue */
		/* If event.type == 0, peek the first event */
		/* If event.type != 0, peek the specific event of that type */
		/* Use event.value to peek the Nth event from the event queue */
		case WIX_EVENT_IOCTL_PEEK:

			down( &wixevent_semaphore );
			if ( WixEventHead == NULL ) {
				up( &wixevent_semaphore );
				return -EPERM;
			}
			if ( event.type == 0 ) {
				el = WixEventHead;
				while ( event.value > 1 ) {
					el = el->next;
					if ( el == NULL ) {
						up( &wixevent_semaphore );
						return -EPERM;
					}
					--event.value;
				}
				if ( copy_to_user((wixEvent *)arg, (wixEvent *)&el->event, sizeof(wixEvent)) ) {
					up( &wixevent_semaphore );
					return -EFAULT;
				}
			} else {
				el = WixEventHead;
				while ( (event.type!=el->event.type) || (event.value>1) ) {
					if ( event.type == el->event.type )
						--event.value;
					el = el->next;
					if ( el == NULL ) {
						up( &wixevent_semaphore );
						return -EPERM;
					}
				}
				if ( copy_to_user((wixEvent *)arg, (wixEvent *)&el->event, sizeof(wixEvent)) ) {
					up( &wixevent_semaphore );
					return -EFAULT;
				}
			}
			up( &wixevent_semaphore );
			break;

		case WIX_EVENT_IOCTL_COUNT:

			down( &wixevent_semaphore );
			if ( copy_to_user((int *)arg, (int *)&wixEventCount, sizeof(int)) ) {
				up( &wixevent_semaphore );
				return -EFAULT;
			}
			up( &wixevent_semaphore );
			break;

		default:
			return WIX_RC_ERR;
	}

	return WIX_RC_OK;
}


static int proc_wixevent_count_read ( char *page, char **start, off_t off, int count, int *eof, void *data ) {
	int len;
	char *p = page;

	down( &wixevent_semaphore );

	p += sprintf( p, "%d\n", wixEventCount );

	up( &wixevent_semaphore );

	len = p - page;
	if ( len <= off+count )
			*eof = 1;
	*start = page + off;
	len -= off;

	if ( len > count )
			len = count;
	if ( len < 0 )
			len = 0;

	return len;
}


int wixevent_send ( wixEvent *event ) {
	wixEventList *el;
	wixEventList *new_el;

	if ( (new_el = kmalloc(sizeof(wixEventList), GFP_ATOMIC)) == NULL ) {
		WIXPRINT("wixevent_send: Unable to allocate memory !\n");
		return -ENOMEM;
	}
	memcpy( &new_el->event, event, sizeof(wixEvent) );
	new_el->event.time = CURRENT_TIME;

	down( &wixevent_semaphore );

	if ( WixEventHead != NULL ) {
		/* Avoid the number of the event queue's event exceed EQUEUE_MAX_NUM. */
		while ( wixEventCount >= EQUEUE_MAX_NUM ) {
			el = WixEventHead;
			WixEventHead = WixEventHead->next;
			kfree(el);
			--wixEventCount;
		}
	}

	if ( WixEventHead == NULL ) {
		WixEventHead = WixEventTail = new_el;
	} else {
		WixEventTail->next = new_el;
		WixEventTail = new_el;
	}
	WixEventTail->next = NULL;
	++wixEventCount;

	up( &wixevent_semaphore );

	return WIX_RC_OK;
}


static int __init wixevent_init ( void ) {
	struct proc_dir_entry *entry;

	WixEventHead = WixEventTail = NULL;
	wixEventCount = 0;
	init_MUTEX_LOCKED( &wixevent_semaphore );
	up( &wixevent_semaphore );

	if ( (wixevent_devmajor = register_chrdev(EVENT_MAJOR, DRV_NAME,&wixevent_fops)) < 0 ) {
		WIXPRINT( "unable to get major %d for wixevent device\n", EVENT_MAJOR );
		return -1;
	}
	WIXDEBUG( "wixevent major %d for the wixevent devs\n", wixevent_devmajor );

	wixevent_class = class_create( THIS_MODULE, "wixevent" );
	//class_device_create( wixevent_class, MKDEV(wixevent_devmajor,0), NULL, DRV_NAME, 0 );
	class_device_create( wixevent_class, NULL, MKDEV(wixevent_devmajor,0), NULL, DRV_NAME );

	proc_wixevent = proc_mkdir( "wixevent", NULL );
	entry = create_proc_entry( "count", 0, proc_wixevent );
	if ( entry ) {
		entry->read_proc = proc_wixevent_count_read;
		entry->data = NULL;
	}

	return 0;
}


static void __exit wixevent_exit ( void ) {
	wixEventList *el;

	class_device_destroy( wixevent_class, MKDEV(wixevent_devmajor,0) );
	class_destroy( wixevent_class );
	unregister_chrdev( wixevent_devmajor, DRV_NAME );

	down( &wixevent_semaphore );

		while ( WixEventHead != NULL ) {
			if ( WixEventHead != WixEventTail ) {
				el = WixEventHead;
				WixEventHead = WixEventHead->next;
				kfree( el );
			} else {
				el = WixEventHead;
				WixEventHead = WixEventTail = NULL;
				kfree( el );
			}
		}

	up( &wixevent_semaphore );

	remove_proc_entry( "count", proc_wixevent );
	remove_proc_entry( "wixevent", NULL );

	return;
}


module_init( wixevent_init );
module_exit( wixevent_exit );

EXPORT_SYMBOL( wixevent_send );

MODULE_AUTHOR("Aldofo Lin");
MODULE_DESCRIPTION("Wistron kernel event queue driver");
MODULE_LICENSE("GPL");
