#ifndef _WIX_EVENT_H
#define _WIX_EVENT_H


#define WIX_EVENT_IOCTL_SND	0x01
#define WIX_EVENT_IOCTL_RCV	0x02
#define WIX_EVENT_IOCTL_PEEK	0x04
#define WIX_EVENT_IOCTL_COUNT	0x08


/*************************/
/* Event Type Definition */
/*************************/
/* NOTE: the MAX type is 0x8000, for we use 16 bit to store criticalevent in wixalarmdaemon */
/* NOTE: the id MUST be sorted ascending and MUST be continuous */
/* Leading with WIX_EVENTTYPE_ prefix */
#define WIX_EVENTTYPE_SYSTEM	0x0001		/* System event */
#define WIX_EVENTTYPE_UPS	0x0002		/* UPS event */
#define WIX_EVENTTYPE_FAN	0x0004		/* Fan event */
#define WIX_EVENTTYPE_THERMAL	0x0008		/* Thermal event */
#define WIX_EVENTTYPE_HDD	0x0010		/* HDD event */
#define WIX_EVENTTYPE_HDDSMART	0x0020		/* HDD SMART event */
#define WIX_EVENTTYPE_VOLUME	0x0040		/* Volume event */
#define WIX_EVENTTYPE_VOLUMEUSE	0x0080		/* Volume Usage event */
#define WIX_EVENTTYPE_NETWORK	0x0100		/* Network event */
#define WIX_EVENTTYPE_USB	0x0200		/* USB event */
#define WIX_EVENTTYPE_USER	0x0400		/* User event */
#define WIX_EVENTTYPE_GROUP	0x0800		/* Group event */
#define WIX_EVENTTYPE_FSCK	0x1000		/* fs check event */
#define WIX_EVENTTYPE_VOLUMEEXT	0x2000		/* Volume Extend event */
#define WIX_EVENTTYPE_INTERNAL	0x4000		/* Internal event */


/***********************/
/* Event ID Definition */
/***********************/
/* Leading with WIX_EVENT_ prefix */
#define WIX_EVENT_ID_ALL	0		/* Dummy Event ID */

/* WIX_EVENTTYPE_SYSTEM */
#define WIX_EVENT_STARTUP		0x0001	/* System - Startup event */
#define WIX_EVENT_ABNORMAL_SHUTDOWN	0x0002	/* System - Shutdown event */
#define WIX_EVENT_SHUTDOWN		0x0004	/* System - Shutdown event */
#define WIX_EVENT_REBOOT		0x0008	/* System - Reboot event */
#define WIX_EVENT_IDENTIFY		0x0010	/* System - Identify event */
#define WIX_EVENT_SOFTWARE_UPDATE	0x0020	/* System - Software Update */
#define WIX_EVENT_FACTORY_RESET		0x0040	/* System - Factory Reset */
#define WIX_EVENT_CONFIGURATION_RESTORE	0x0080	/* System - Configuration Restore */
#define WIX_EVENT_ADMINSTRATOR_LOGIN	0x0100	/* System - Administrator Login */

/* WIX_EVENTTYPE_UPS */
#define WIX_EVENT_UPS_COMM_OK	0x0001	/* Communciations restored with UPS */
#define WIX_EVENT_UPS_COMM_FAILED 0x0002/* Communications lost with UPS */
#define WIX_EVENT_UPS_ON_BTY	0x0004	/* Power failure on UPS. Running on batteries. */
#define WIX_EVENT_UPS_OFF_BTY	0x0008	/* Power has returned. Running on power. */
#define WIX_EVENT_UPS_ON_PWR	0x0010	/* Power has returned on UPS or run on power.*/
#define WIX_EVENT_UPS_OFF_PWR	0x0020	/* Power loss detected on UPS */
#define WIX_EVENT_UPS_SHUTDOWN	0x0040	/* UPS Doing shutdown. */
#define WIX_EVENT_UPS_BTY_LOW	0x0080	/* Battery power exhaused on UPS. */

/* WIX_EVENTTYPE_FAN */
#define WIX_EVENT_FAN1		0x0001		/* Fan - Fan 1 */

/* WIX_EVENTTYPE_THERMAL */
#define WIX_EVENT_THERMAL1	0x0001		/* Thermal - Thermal 1 */

/* WIX_EVENTTYPE_HDD */
/* WIX_EVENTTYPE_HDDSMART */
#define WIX_EVENT_HDD1		0x0001		/* HDD - HDD 1 */
#define WIX_EVENT_HDD2		0x0002		/* HDD - HDD 2 */
#define WIX_EVENT_HDD3		0x0004		/* HDD - HDD 3 */
#define WIX_EVENT_HDD4		0x0008		/* HDD - HDD 4 */

/* WIX_EVENTTYPE_VOLUME */
/* WIX_EVENTTYPE_VOLUMEUSE */
#define WIX_EVENT_VOL1		0x0001		/* Volume - Volume 1 */
#define WIX_EVENT_VOL2		0x0002		/* Volume - Volume 2 */
#define WIX_EVENT_VOL3		0x0004		/* Volume - Volume 3 */
#define WIX_EVENT_VOL4		0x0008		/* Volume - Volume 4 */


/* WIX_EVENTTYPE_NETWORK */
#define WIX_EVENT_LAN1		0x0001		/* Network - LAN 1 event */
#define WIX_EVENT_LAN2		0x0002		/* Network - LAN 2 event */
#define WIX_EVENT_WLAN		0x0004		/* Network - Wireless LAN event */

/* WIX_EVENTTYPE_USB */
#define WIX_EVENT_USB1		0x0001		/* USB - USB 1 */
#define WIX_EVENT_USB2		0x0002		/* USB - USB 2 */
#define WIX_EVENT_USB3		0x0004		/* USB - USB 3 - reserved for future use */
#define WIX_EVENT_USB4		0x0008		/* USB - USB 4 - reserved for future use */

/* WIX_EVENTTYPE_USER */
/* The event ID will be the user ID */

/* WIX_EVENTTYPE_GROUP */
/* The event ID will be the group ID */

/* WIX_EVENTTYPE_INTERNAL */
#define WIX_EVENT_USB_BACKUP	0x0001
#define WIX_EVENT_CHECKMD	0x0002

/* WIX_EVENTTYPE_FSCK */
/* WIX_EVENTTYPE_VOLUMEEXT */
/* same as WIX_EVENTTYPE_VOLUME */


/**************************/
/* Event State Definition */
/**************************/
/* Leading with WIX_STATE_ prefix */
/* WIX_EVENTTYPE_SYSTEM */
    /* WIX_EVENT_IDENTIFY */
	#define WIX_STATE_IDENTIFY_ON	0x01	/* Identify On */
	#define WIX_STATE_IDENTIFY_OFF	0x02	/* Identify Off */
    /* WIX_EVENT_SOFTWARE_UPDATE */
    /* WIX_EVENT_FACTORY_RESET */
    /* WIX_EVENT_CONFIGURATION_RESTORE */
    /* WIX_EVENT_ADMIN_LOGIN */
	#define WIX_STATE_SUCCEEDED	0	/* Operation Succeeded */
	#define WIX_STATE_FAILED	0x01	/* Operation Failed */
/* WIX_EVENTTYPE_FAN */
	#define WIX_STATE_NORMAL	0	/* Fan normal */
	//#define WIX_STATE_FAILED	0x01	/* Fan failed */
/* WIX_EVENTTYPE_THERMAL */
	//#define WIX_STATE_NORMAL	0	/* Thermal normal */
	#define WIX_STATE_OVERHEATED	0x01	/* Thermal overheated */
/* WIX_EVENTTYPE_HDD */
	#define WIX_STATE_HDD_FOUND	0	/* HDD Found */
	#define WIX_STATE_HDD_ABSENT	0x01	/* HDD Absent */
/* WIX_EVENTTYPE_HDDSMART */
	#define WIX_STATE_HDD_SMART_OK	0	/* HDD SMART OK */
	#define WIX_STATE_HDD_SMART_FAILED 0x01	/* HDD SMART Fail */
/* WIX_EVENTTYPE_VOLUME */
	#define WIX_STATE_VOL_NORMAL	0	/* RAID volume in normal mode */
	#define WIX_STATE_VOL_DEGRADED	0x01	/* RAID volume in degraded mode */
	#define WIX_STATE_VOL_RESYNC	0x02	/* RAID(0,1,5) volume resyncing */
	#define WIX_STATE_VOL_FAILED	0x04	/* Sent if volume could not be recovered */
	#define WIX_STATE_VOL_NOT_EXIST	0x08	/* Volume doesn't exist */
	#define WIX_STATE_CREATE	0x1000	/* Created */
	#define WIX_STATE_DELETE	0x2000	/* Deleted */
	#define WIX_STATE_MODIFY	0x4000	/* Modified */
/* WIX_EVENTTYPE_VOLUMEUSE */
	#define WIX_STATE_VOL_25FREE	0	/* Volume has 25~100% free capacity */
	#define	WIX_STATE_VOL_10FREE	0x01	/* Volume has more than 10% free capacity */
	#define	WIX_STATE_VOL_5FREE	0x02	/* Volume has more than 5% free capacity */
	#define WIX_STATE_VOL_80FULL	0x04	/* Volume has reached 80% capacity */
	#define WIX_STATE_VOL_95FULL	0x08	/* Volume has reached 95% capacity */
	#define WIX_STATE_VOL_FULL	0x10	/* Volume has reached full capacity */
	#define WIX_STATE_VOL_NOT_MOUNT	0x20	/* Volume has NOT been mounted */
/* WIX_EVENTTYPE_NETWORK */
	#define WIX_STATE_LINK_OK	0	/* Network Link OK */
	#define WIX_STATE_LINK_DOWN	0x01	/* Network Link DOWN */
	#define WIX_STATE_IP_DHCP_OK	0x02	/* Use dynamic IP address got from DHCP */
	#define WIX_STATE_IP_DHCP_FAILED 0x04	/* Can not get IP address from DHCP, use default IP address */
	#define WIX_STATE_IP_STATIC	0x08	/* Use static IP address */
/* WIX_EVENTTYPE_USB	*/
	#define WIX_STATE_USB_INSERT	0	/* USB Insertion */
	#define WIX_STATE_USB_SAFE_REMOVE 0x01	/* USB Safely removal */
	#define WIX_STATE_USB_UNSAFE_REMOVE 0x02	/* USB unsafely removal */
	#define WIX_STATE_USB_FORMAT_OK	0x04	/* USB format OK */
	#define WIX_STATE_USB_FORMAT_FAILED 0x08	/* USB format fail */
	#define WIX_STATE_USB_PRINTER_FOUND	0x10	/* USB Printer Found */
	#define WIX_STATE_USB_PRINTER_ABSENT	0x20	/* USB Printer Absent */
/* WIX_EVENTTYPE_USER */
	#define WIX_STATE_QUOTA_NORMAL	0	/* Quota below 70% utilization */
	#define WIX_STATE_QUOTA_75	0x01	/* Quota exceed 75% utilization */
	#define WIX_STATE_QUOTA_90	0x02	/* Quota exceed 90% utilization */
	#define WIX_STATE_QUOTA_FULL	0x04	/* Quota reached 100% utilization */
	#define WIX_STATE_LOGIN_FAILED	0x08	/* Login failed 3 times */
	//#define WIX_STATE_CREATE	0x1000	/* Created */
	//#define WIX_STATE_DELETE	0x2000	/* Deleted */
	//#define WIX_STATE_MODIFY	0x4000	/* Modified */
/* WIX_EVENTTYPE_GROUP */
	//#define WIX_STATE_CREATE	0x1000	/* Created */
	//#define WIX_STATE_DELETE	0x2000	/* Deleted */
	//#define WIX_STATE_MODIFY	0x4000	/* Modified */

/* WIX_EVENTTYPE_FSCK */
	//#define WIX_STATE_NORMAL	0	/* FS check ok */
	//#define WIX_STATE_FAILED	0x01	/* FS check failed */
	#define WIX_STATE_VOL_NEEDCHECK	0x02	/* FS need to be checked */

/* WIX_EVENTTYPE_VOLUMEEXT */
	//#define WIX_STATE_NORMAL	0	/* Volume extend ok */
	//#define WIX_STATE_FAILED	0x01	/* Volume extend failed */


/**************************/
/* Event Level Definition */
/**************************/
/* Leading with WIX_EVENT_LEVEL_ prefix */
#define WIX_EVENT_LEVEL_INTERNAL	0	/* Internel use only */
#define WIX_EVENT_LEVEL_INFO		0x0001	/* Event level - Information */
#define WIX_EVENT_LEVEL_WARNING		0x0002	/* Event level - Warning */
#define WIX_EVENT_LEVEL_CRITICAL	0x0004	/* Event level - Critical */


#define WIXEVENT_LOG( PRIORITY, SUBJECT, CONTENT )		\
        openlog( "wixEvent", LOG_PID, LOG_DAEMON );		\
        syslog( PRIORITY, "%s - %s", SUBJECT, CONTENT );	\
        closelog();
#define WIX_LOG( PRIORITY, CONTENT )				\
        openlog( "wixEvent", LOG_PID, LOG_DAEMON );		\
        syslog( PRIORITY, "%s - %s", __FUNCTION__, CONTENT );	\
        closelog();


typedef struct {
	unsigned int	type;
	unsigned int	id;
	unsigned int	state;
	unsigned int	value;
	struct timespec	time;
} wixEvent;


typedef struct wixEventList {
	wixEvent event;
	struct wixEventList *next;
} wixEventList;


/* Push event to the event queue, and returns "WIX_RC_OK" if succeeded */
extern int wixevent_push(wixEvent *event);


/* Pull for currently pending events, and returns "WIX_RC_OK" if
   there are any pending events, or "WIX_RC_EVENT_EMPTY" if there is none
   available.  If 'event' is not NULL, the next event is removed from the
   queue and stored in that area.
   If event.type == 0, pull the first event
   If event.type != 0, pull the specific event of that type */
extern int wixevent_pull(wixEvent *event);


/* Wait indefinitely for the next available event, returning "WIX_RC_OK",
   or "WIX_RC_ERR" if there was an error while waiting for events.  If
   'event' is not NULL, the next event is removed from the queue and stored
   in that area.
   If event.type == 0, wait for the first event
   If event.type != 0, wait for the specific event of that type */
extern int wixevent_wait(wixEvent *event);


/* Peek for event in the event queue, and returns "WIX_RC_OK" if
   there are any pending events, or "WIX_RC_EVENT_EMPTY" if there is none
   available.  No event will be removed from the event queue.
   If event.type == 0, peek the first event
   If event.type != 0, peek the specific event of that type
   You can also use event.value to peek the Nth event from the event queue */
extern int wixevent_peek(wixEvent *event);


/* Get the current event count of the event queue */
extern int wixevent_getcount(int *count);


#endif	/* _WIX_EVENT_H */
