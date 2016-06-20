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
 * The security module (LSM API) component of the trustees system
 *
 * One quick note: generally security modules with the LSM are supposed
 * to be solely restrictive modules.  Unless the trustees module were to
 * require that people set all files rwx by all, it could not function
 * as it is meant to function as a solely restrictive module.
 *
 * To compensate, every process is given the capability CAP_DAC_OVERRIDE.
 * In other words, every process is first given full rights to the filesystem.
 * This is the only non-restricting portion of this module, since it -does-
 * in fact give additional permissions.  However, in the inode_permission hook,
 * any rights the user should not have are taken away.
 *
 * Side effects: Posix ACLs or other filesystem-specific permissions are not
 * honored.  Trustees ACLs can (and do) take into account the standard unix
 * permissions, but any permissions further than that are difficult, to say
 * the least, to take into account.  I, personally, do not find this to
 * be a problem since if you are using Trustees ACLs, why also require the use
 * of another ACL system?
 */

#include <linux/security.h>
#include <linux/capability.h>
#include <linux/mount.h>
#include <linux/namei.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/nsproxy.h>
#include <linux/mnt_namespace.h>

#include "internal.h"

static int trustees_capable(struct task_struct *tsk, int cap);
static int trustees_inode_permission(struct inode *inode,
				     int mask, struct nameidata *nd);

/* Checks if user has access to the inode due to root status
 */
static inline int has_root_perm(struct inode *inode, int mask)
{
	umode_t mode = inode->i_mode;

	if (!(mask & MAY_EXEC) || (mode & S_IXUGO) || S_ISDIR(mode))
		if (current->fsuid == 0)
			return 0;

	return -EACCES;
}

/* The logic for this was mostly stolen from vfs_permission.  The security API
 * doesn't give a good way to use the actual vfs_permission for this since our
 * CAP_DAC_OVERRIDE causes it to always return 0.  But if we didn't return
 * CAP_DAC_OVERRIDE, we'd never get to handle permissions!  Since we don't need
 * to handle capabilities and dealing with ACLs with trustees loaded isn't an
 * issue for me, the function ends up being pretty simple.
 */

static inline int has_unix_perm(struct inode *inode, int mask)
{
	umode_t mode = inode->i_mode;
	mask &= ~MAY_APPEND;

	if (current->fsuid == inode->i_uid)
		mode >>= 6;
	else if (in_group_p(inode->i_gid))
		mode >>= 3;

	if (((mode & mask & (MAY_READ | MAY_WRITE | MAY_EXEC)) == mask))
		return 0;

	return -EACCES;
}

/* Find a vfsmount given an inode */
static inline struct vfsmount *find_inode_mnt(struct inode *inode,
					      struct nameidata *nd)
{
	struct mnt_namespace *ns = NULL;
	struct vfsmount *mnt = NULL;

	if (likely(nd))
		return mntget(nd->mnt);

	/* Okay, we need to find the vfsmount by looking
	 * at the namespace now.
	 */
	task_lock(current);
	if (current->nsproxy) {
		ns = current->nsproxy->mnt_ns;
		if (ns)
			get_mnt_ns(ns);
	}
	task_unlock(current);

	if (!ns) return NULL;

	list_for_each_entry(mnt, &ns->list, mnt_list) {
		if (mnt->mnt_sb == inode->i_sb) {
			mntget(mnt);
			goto out;
		}
	}

  out:
	put_mnt_ns(ns);

	return mnt;
}

/* Find a dentry given an inode */
static inline struct dentry *find_inode_dentry(struct inode *inode,
					       struct nameidata *nd)
{
	struct dentry *dentry;

	if (likely(nd))
		return dget(nd->dentry);

	dentry = d_find_alias(inode);

	return dentry;
}

/*
 * Return 1 if they are under the same set of trustees
 * otherwise return 0.  In the case that we are handling
 * a directory, we also check to see if there are subdirectories
 * with trustees.
 */
static inline int have_same_trustees(struct dentry *old_dentry,
				     struct dentry *new_dentry)
{
	struct vfsmount *mnt;
	char *old_file_name, *new_file_name;
	int old_depth, new_depth;
	struct trustee_hash_element *old_deep, *new_deep;
	int is_dir;
	int ret = 0;

	mnt = find_inode_mnt(old_dentry->d_inode, NULL);
	if (unlikely(!mnt)) {
		TS_ERR_MSG("inode does not have a mnt!\n");
		return 0;
	}

	old_file_name = trustees_filename_for_dentry(old_dentry, &old_depth, 1);
	if (!old_file_name) {
		TS_ERR_MSG("Couldn't allocate filename\n");
		goto out_old_dentry;
	}

	new_file_name = trustees_filename_for_dentry(new_dentry, &new_depth, 1);
	if (!new_file_name) {
		TS_ERR_MSG("Couldn't allocate filename\n");
		goto out_new_dentry;
	}

	is_dir = S_ISDIR(old_dentry->d_inode->i_mode);

	read_lock(&trustee_hash_lock);
	trustee_perm(old_dentry, mnt, old_file_name, ret, old_depth, is_dir,
		     &old_deep);
	trustee_perm(new_dentry, mnt, new_file_name, ret, new_depth, is_dir,
		     &new_deep);
	if (old_deep == new_deep) {
		ret = 1;
		if (is_dir) {
			if (trustee_has_child(mnt, old_file_name) ||
				trustee_has_child(mnt, new_file_name)) ret = 0;
		}
	}
	read_unlock(&trustee_hash_lock);

	kfree(new_file_name);
out_new_dentry:
	kfree(old_file_name);
out_old_dentry:
	mntput(mnt);

	return ret;
}


static int trustees_inode_rename(struct inode *old_dir,
				 struct dentry *old_dentry,
				 struct inode *new_dir,
				 struct dentry *new_dentry);
static int trustees_inode_link(struct dentry *old_dentry,
			       struct inode *dir,
			       struct dentry *new_dentry);

/* Structure where we fill in the various hooks we are implementing in this module
 */
struct security_operations trustees_security_ops = {
	.capable = trustees_capable,
	.inode_permission = trustees_inode_permission,
	.inode_link = trustees_inode_link,
	.inode_rename = trustees_inode_rename,

	.ptrace =			cap_ptrace,
	.capget =			cap_capget,
	.capset_check =			cap_capset_check,
	.capset_set =			cap_capset_set,
	.settime =			cap_settime,
	.netlink_send =			cap_netlink_send,
	.netlink_recv =			cap_netlink_recv,

	.bprm_apply_creds =		cap_bprm_apply_creds,
	.bprm_set_security =		cap_bprm_set_security,
	.bprm_secureexec =		cap_bprm_secureexec,

	.inode_setxattr =		cap_inode_setxattr,
	.inode_removexattr =		cap_inode_removexattr,

	.task_post_setuid =		cap_task_post_setuid,
	.task_reparent_to_init =	cap_task_reparent_to_init,

	.syslog =                       cap_syslog,

	.vm_enough_memory =             cap_vm_enough_memory
};

#define ALL_MAYS (MAY_WRITE | MAY_EXEC | MAY_READ)
/* Converts a trustee_mask to a normal unix mask
 */
static int inline trustee_mask_to_normal_mask(int mask, int isdir)
{
	int r = 0;
	if ((mask & TRUSTEE_READ_MASK) && !isdir)
		r |= MAY_READ;
	if ((mask & TRUSTEE_READ_DIR_MASK) && isdir)
		r |= MAY_READ;
	if (mask & TRUSTEE_WRITE_MASK)
		r |= MAY_WRITE;
	if ((mask & TRUSTEE_BROWSE_MASK) && isdir)
		r |= MAY_EXEC;
	if ((mask & TRUSTEE_EXECUTE_MASK) && !isdir)
		r |= MAY_EXEC;
	return r;
}

/* This is the meat of the permissions checking.  First it checks for root,
 * otherwise it first checks for any errors finding the dentry/vfsmount for
 * the inode, and then it looks up the dentry in the trustees hash.
 */
static int trustees_inode_permission(struct inode *inode,
				     int mask, struct nameidata *nd)
{
	struct dentry *dentry;
	struct vfsmount *mnt;
	char *file_name;
	int is_dir;
	int ret;
	int depth;
	int amask;
	int dmask;
	umode_t mode = inode->i_mode;

	if (has_root_perm(inode, mask) == 0)
		return 0;

	ret = has_unix_perm(inode, mask);

	mnt = find_inode_mnt(inode, nd);
	if (unlikely(!mnt)) {
		TS_ERR_MSG("inode does not have a mnt!\n");
		return -EACCES;	/* has_unix_perm(inode, mask); */
	}

	dentry = find_inode_dentry(inode, nd);
	if (unlikely(!dentry)) {
		/* Most of the time when this happens, it is the /
		 * If it is not, we need to dump as much information
		 * as possible on it and dump it to logs, because
		 * I'm really not sure how it happens.
		 */
		if (inode == mnt->mnt_root->d_inode) {
			dentry = dget(mnt->mnt_root);
		} else {
			/* I have seen this happen once but I did not have any
			 * way to see what caused it.  I am gonna dump_stack
			 * until I have that happen again to see if the cause
			 * is something that I need to worry about.
			 */
			dump_stack();	/* DEBUG FIXME */
			TS_ERR_MSG("Inode number: %ld\n", inode->i_ino);
			TS_ERR_MSG("dentry does not exist!\n");
			goto out_mnt;
		}
	}
	file_name = trustees_filename_for_dentry(dentry, &depth, 1);
	if (!file_name) {
		TS_ERR_MSG("Couldn't allocate filename\n");
		ret = -EACCES;
		goto out_dentry;
	}

	is_dir = S_ISDIR(inode->i_mode);

	read_lock(&trustee_hash_lock);
	amask = trustee_perm(dentry, mnt, file_name, ret, depth, is_dir,
			     (struct trustee_hash_element **)NULL);
	read_unlock(&trustee_hash_lock);
	dmask = amask >> TRUSTEE_NUM_ACL_BITS;

	/* no permission if denied */
	if (trustee_mask_to_normal_mask(dmask, is_dir) & mask & ALL_MAYS) {
		ret = -EACCES;
		goto out;
	}
	/* use unix perms */
	if (!(dmask & TRUSTEE_USE_UNIX_MASK) &&
	    (amask & TRUSTEE_USE_UNIX_MASK) && (!ret))
		goto out;

	/* if the file isn't executable, then the trustees shouldn't
	 * make it executable
	 */
	if ((mask & MAY_EXEC) && !(mode & S_IXOTH) &&
	    !((mode >> 3) & S_IXOTH) & !((mode >> 6) & S_IXOTH) &&
	    (!is_dir)) {
		ret = -EACCES;
		goto out;
	}
	/* Check trustees for permission
	 */
	if ((trustee_mask_to_normal_mask(amask, is_dir) & mask & ALL_MAYS)
	    == mask) {
		ret = 0;
		goto out;
	} else
		ret = -EACCES;

      out:
	kfree(file_name);
      out_dentry:
	dput(dentry);
      out_mnt:
	mntput(mnt);

	return ret;
}

/* We should only allow hard links under one of two conditions:
 *   1. Its in the same trustee
 *        - if the two dentries are covered by the same trustee, there shouldn't
 *          be much of a problem with allowing the hardlink to occur.
 *   2. fsuid = 0
 */
static int trustees_inode_link(struct dentry *old_dentry,
			       struct inode *dir,
			       struct dentry *new_dentry)
{
	if (current->fsuid == 0)
		return 0;

	if (have_same_trustees(old_dentry, new_dentry))
		return 0;

	return -EXDEV;
}

/* We have a few renames to protect against:
 *   1. Any file or directory that is affected by different trustees at its
 *      old location than at its new location.
 *   2. In the case of a directory, we should protect against moving a directory
 *      that has trustees set inside of it.
 *
 * In any case above, we return -EXDEV which signifies to the calling program that
 * the files are on different devices, and assuming the program is written correctly
 * it should then handle the situation by copying the files and removing the originals
 * ( which will then use the trustees permissions as they are meant to be used )
 */
static int trustees_inode_rename(struct inode *old_dir,
				 struct dentry *old_dentry,
				 struct inode *new_dir,
				 struct dentry *new_dentry)
{
	if (current->fsuid == 0)
		return 0;

	if (have_same_trustees(old_dentry, new_dentry)) return 0;

	return -EXDEV;
}

/* Return CAP_DAC_OVERRIDE on everything.  We want to handle our own
 * permissions (overriding those normally allowed by unix permissions)
 */
static int trustees_capable(struct task_struct *tsk, int cap)
{
	if (cap == CAP_DAC_OVERRIDE)
		return 0;

	return cap_capable(tsk, cap);
}

/* Register the security module
 */
int trustees_init_security(void)
{
	/* FIXME: add in secondary module register
	 * not worry about it now since I have better
	 * things to worry about. Comprende?
	 */
	if (register_security(&trustees_security_ops)) {
		TS_ERR_MSG("Could not register security component\n");
		return -EINVAL;
	}

	return 0;
}
