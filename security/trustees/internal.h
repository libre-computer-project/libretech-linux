/*
 * Trustees ACL Project
 *
 * Copyright (c) 1999-2000 Vyacheslav Zavadsky
 * Copyright (c) 2004 Andrew Ruder (aeruder@ksu.edu)
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 * Private methods and definitions used only within the module.
 *
 */

#ifndef _LINUX_TRUSTEES_H
#define _LINUX_TRUSTEES_H
#include <linux/types.h>
#include <linux/dcache.h>
#include <linux/kdev_t.h>
#include <linux/list.h>
#include <linux/version.h>
#include <linux/trustees.h>

#define TRUSTEE_DEFAULT_MASK TRUSTEE_USE_UNIX_MASK

struct trustee_ic {
	dev_t dev;
	char *devname;		/* ONLY if MAJOR(dev)==0 */
	struct list_head ic_list;
};

struct trustee_name {
	dev_t dev;
	char *filename;
	char *devname;		/* ONLY if MAJOR(dev)==0 */
};

struct trustee_permission_capsule {
	struct list_head perm_list;
	struct trustee_permission permission;
};

/* For the usage field */
#define TRUSTEE_HASH_ELEMENT_USED 2
#define TRUSTEE_HASH_ELEMENT_DELETED 1
#define TRUSTEE_HASH_ELEMENT_NOTUSED 0

struct trustee_hash_element {
	struct trustee_name name;
	struct list_head perm_list;
	struct hlist_node hash_list;
	struct list_head device_list;
};

extern char *trustees_filename_for_dentry(struct dentry *dentry, int *d, int trunc);

extern int trustees_funcs_init_globals(void);
extern int trustees_funcs_cleanup_globals(void);

int trustee_has_child(struct vfsmount *mnt, char *file_name);
int trustee_perm(struct dentry *dentry, struct vfsmount *mnt,
		 char *file_name, int unix_ret, int depth, int is_dir,
		 struct trustee_hash_element **deepest);

extern int trustees_process_command(struct trustee_command command,
                                    void **arg, size_t *argsize);

extern unsigned int trustee_hash_size;
extern rwlock_t trustee_hash_lock;

#define TRUSTEE_INITIAL_NAME_BUFFER 256
#define TRUSTEE_HASDEVNAME(TNAME) (MAJOR((TNAME).dev)==0)

#define TS_ERR_MSG(...) printk(KERN_ERR "Trustees: " __VA_ARGS__)

#ifdef TRUSTEES_DEBUG
#define TS_DEBUG_MSG(...) printk(KERN_ERR "Trustees: " __VA_ARGS__)
#else
#define TS_DEBUG_MSG(...)
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,15)
#define NAMESPACE_SEM(_ns) (namespace_sem)
#else
#define NAMESPACE_SEM(_ns) ((_ns)->sem)
#endif

/*
 * Magic number!
 *
 * FIXME: Do I just make this up or is there some system for coming
 * up with magic numbers?
 */
#define TRUSTEES_MAGIC 0x32236975

int trustees_init_fs(void);
void trustees_deinit_fs(void);

int trustees_init_security(void);
#endif				/* _LINUX_TRUSTEES_H */
