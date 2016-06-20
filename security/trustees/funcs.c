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
 * This code contains the functions for handling the actual trustees data
 * and returning the permissions for a given file, etc.
 *
 *
 */

#include <linux/fs.h>
#include <linux/mount.h>
#include <linux/dcache.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/limits.h>
#include <linux/list.h>
#include <linux/vmalloc.h>
#include <linux/ctype.h>

#include "internal.h"

/*
 * This is a hash of all the trustee_names currently added.  These values
 * are hashed on a combination of device/filename.  Before reading/writing
 * be sure to take care of the locking of trustee_hash_lock.
 */
rwlock_t trustee_hash_lock;
static struct hlist_head *trustee_hash = NULL;

/*
 * This is the deepest level trustee.  When calculating filenames, we can
 * skip several of the levels in many case since we know it won't be any
 * deeper than this.
 *
 * Kept up to date by calculate_deepest_level
 *
 * /           => 0
 * /test       => 1
 * /test/blah  => 2
 */
static int deepest_level = 0;

/*
 * A list of filesystems that need to have their case
 * ignored.  This is protected by trustee_hash_lock.
 */
static LIST_HEAD(trustee_ic_list);


/* The calling method needs to free the buffer created by this function
 * This method returns the filename for a dentry.  This is, of course,
 * relative to the device.  The filename can be truncated to be as deep as
 * the deepest trustee.  The depth returned in d will always be the true
 * depth, however.
 *
 * Args:
 *   dentry: The dentry we are interested in.
 *   d: a pointer to the place where the depth can be stored.
 *   trunc: ok to truncate the name to the longest that needs to be figured out.
 */

#define FN_CHUNK_SIZE 64
char *trustees_filename_for_dentry(struct dentry *dentry, int *d, int trunc)
{
	char *buffer = NULL, *tmpbuf = NULL;
	int bufsize = FN_CHUNK_SIZE;
	char c;
	int i, j, k;
	int depth = 0;
	struct dentry *temp_dentry;

	if (dentry->d_parent == NULL) {
		TS_ERR_MSG("d_parent is null\n");
		return NULL;
	}

	if (dentry->d_name.name == NULL) {
		TS_ERR_MSG("name is null\n");
		return NULL;
	}

	buffer = kmalloc(FN_CHUNK_SIZE, GFP_KERNEL);
	if (!buffer) {
		TS_ERR_MSG("could not allocate filename buffer\n");
		return NULL;
	}

	buffer[0] = '/';
	buffer[i = 1] = '\0';
	for (temp_dentry = dentry; !IS_ROOT(temp_dentry); temp_dentry = temp_dentry->d_parent)
		depth++;
	if (d) *d = depth;
	if (deepest_level <= 0) return buffer;

	for (;;) {
		if (IS_ROOT(dentry))
			break;
		if (depth-- > deepest_level) continue;

		j = i + strlen(dentry->d_name.name);
		if ((j + 2) > bufsize) {	/* reallocate - won't fit */
			bufsize = (((j + 2) / FN_CHUNK_SIZE) + 1) * FN_CHUNK_SIZE;
			tmpbuf = kmalloc(bufsize, GFP_KERNEL);
			if (!tmpbuf) {
				kfree(buffer);
				TS_ERR_MSG
				    ("Out of memory allocating tmpbuf\n");
				return NULL;
			}
			memcpy(tmpbuf, buffer, i);
			kfree(buffer);
			buffer = tmpbuf;
		}
		/* Throw the name in there backward */
		for (k = 0; dentry->d_name.name[k]; k++) {
			buffer[j - 1 - k] = dentry->d_name.name[k];
		}
		i = j;
		buffer[i++] = '/';
		dentry = dentry->d_parent;
	}
	buffer[i] = 0;

	/* buffer is backwards, reverse it */
	for (j = 0; j < (i / 2); ++j) {
		c = buffer[j];
		buffer[j] = buffer[i - j - 1];
		buffer[i - j - 1] = c;
	}

	return buffer;
}

/**
 * Allocate memory using vmalloc and return a duplicate of the passed in string.
 * Returns NULL if a problem occurs
 */
static char *vmalloc_strdup(const char *str, size_t len)
{
	char *r;

	if (!str) return NULL;
	len = strlen(str);
	r = vmalloc(len + 1);
	if (!r) return NULL;
	memcpy(r, str, len + 1);

	return r;
}

/*
 * Add a filesystem as a ignored-case dev.
 */
static inline void add_ic_dev(u32 dev, char *devname)
{
	char *devname2;
	struct trustee_ic *ic;
	size_t dev_len;

	dev_len = strlen(devname);

	if (dev_len > PATH_MAX) {
		TS_ERR_MSG("devname bad, add_ic_dev ignored.\n");
		return;
	}

	if (!dev_len) {
		TS_ERR_MSG("No devname specified in add_ic_dev.\n");
		return;
	}

	devname2 = vmalloc_strdup(devname, dev_len);
	if (!devname2) {
		TS_ERR_MSG
		    ("Seems that we have ran out of memory adding ic dev!\n");
		return;
	}

	ic = vmalloc(sizeof(struct trustee_ic));
	if (!ic) {
		TS_ERR_MSG
		    ("Seems that we ran out of memory allocating ic!\n");
		return;
	}

	ic->dev = new_decode_dev(dev);
	ic->devname = devname2;

	write_lock(&trustee_hash_lock);
	list_add(&ic->ic_list, &trustee_ic_list);
	write_unlock(&trustee_hash_lock);
}

/*
 * Remove all ignored-case filesystems.
 */
static inline void remove_ic_devs(void)
{
	struct trustee_ic *ic, *temp_ic;
	struct list_head temp_ic_list;

	INIT_LIST_HEAD(&temp_ic_list);
	list_splice_init(&trustee_ic_list, &temp_ic_list);

	list_for_each_entry_safe(ic, temp_ic, &temp_ic_list, ic_list) {
		vfree(ic->devname);
		vfree(ic);
	}
}

/*
 * This frees all the capsules in a trustee element.
 */
static inline void free_hash_element_list(struct trustee_hash_element *e)
{
	struct trustee_permission_capsule *capsule, *temp;

	list_for_each_entry_safe(capsule, temp, &e->perm_list, perm_list) {
		list_del(&capsule->perm_list);
		vfree(capsule);
	}
}

/*
 * Free a trustee name.  This frees the devname and the filename
 */
static inline void free_trustee_name(struct trustee_name *name)
{
	vfree(name->filename);
	vfree(name->devname);
}

/*
 * Frees the capsules, and the filenames for a trustee hash element.
 * Also marks it as unused in the hash.
 */
static inline void free_hash_element(struct trustee_hash_element *e)
{
	free_hash_element_list(e);
	free_trustee_name(&e->name);
	vfree(e);
}

/**
 * Copies from src to dest (duplicating the strings in the
 * trustee_name structure.  Returns zero for unsuccesful.
 */
static int copy_trustee_name(struct trustee_name *dst, struct trustee_name *src)
{
	*dst = *src;
	if (dst->filename) {
		dst->filename = vmalloc_strdup(src->filename, strlen(src->filename));
		if (!dst->filename) {
			TS_ERR_MSG("Ran out of memory duplicating src->filename\n");
			return 0;
		}
	}

	if (dst->devname) {
		dst->devname = vmalloc_strdup(src->devname, strlen(src->devname));
		if (!dst->devname) {
			TS_ERR_MSG("Ran out of memory duplicating src->devname\n");
			vfree(dst->filename);
			return 0;
		}
	}

	return 1;
}


/*
 * hashing function researched by Karl Nelson <kenelson @ ece ucdavis edu>
 * and is used in glib.
 */
static inline unsigned int hash_string(const char *s)
{
	unsigned int v = 0;

	while (*s) {
		v = (v << 5) - v + tolower(*s);
		s++;
	}

	return v;
}

/*
 * Return the hash for a device.
 */
static inline unsigned int hash_device(const char *name, dev_t device)
{
	if (MAJOR(device) == 0) {
		return hash_string(name);
	}

	return new_encode_dev(device);
}

/*
 * Return the hash for a file.  This is a combination of the
 * hash of the filename and the hash for the device.
 */
static inline unsigned int hash(const struct trustee_name *name)
{
	return hash_string(name->filename) ^
	       hash_device(name->devname, name->dev);
}

/*
 * Return the slot in the trustees_hash where a trustee is located
 */
static inline unsigned int hash_slot(const struct trustee_name *name)
{
    return hash(name) % trustee_hash_size;
}

/*
 * Compare two devices.  Return 1 if they are equal otherwise return 0
 */
static inline int trustee_dev_cmp(dev_t dev1, dev_t dev2, char *devname1,
				  char *devname2)
{
	if ((MAJOR(dev1) == 0) && (MAJOR(dev2) == 0))
		return (strcmp(devname1, devname2) == 0);
	else if ((MAJOR(dev1) != 0) && (MAJOR(dev2) != 0))
		return (dev1 == dev2);
	return 0;
}

/*
 * Compare two trustee_name's.  Returns 1 if they are are equal
 * otherwise return 0
 */
static inline int trustee_name_cmp(const struct trustee_name *n1,
				   const struct trustee_name *n2,
				   unsigned ignore_case)
{
	if (trustee_dev_cmp(n1->dev, n2->dev, n1->devname, n2->devname))
		return ignore_case ?
		    (strnicmp(n1->filename, n2->filename, PATH_MAX) == 0) :
		    (strcmp(n1->filename, n2->filename) == 0);
	return 0;
}

/*
 * Calculate the deepest level.
 */
static inline void calculate_deepest_level(const struct trustee_name *name)
{
	char *fn = name->filename;
	char *x;
	int level = 0;

	for (x = fn; *x; ++x) {
		if (*x == '/')
			++level;
	}

	/* If it is the root, it should have
	 * a level of 0.
	 */
	if (x == (fn + 1)) level = 0;

	if (level > deepest_level) deepest_level = level;
}

/*
 * Return the trustee element for a name.
 * This should be called with a lock on the trustee_hash (which should
 * not be released until you are done with the returned hash_element)!
 */
static struct trustee_hash_element *get_trustee_for_name(const struct trustee_name *name,
							 unsigned ignore_case)
{
	struct trustee_hash_element *item = NULL;
	struct hlist_node *iter = NULL;

	hlist_for_each_entry(item, iter, &trustee_hash[hash_slot(name)], hash_list) {
		if (trustee_name_cmp(&item->name, name, ignore_case))
			return item;
	}

	return NULL;
}

/**
 * Add a new blank trustee to the hash.
 *
 * If this returns zero, then the adding failed and name should be freed
 * (assuming must_copy is 0), otherwise assume we used its memory.
 */
static unsigned add_trustee(struct trustee_name *name, int must_copy) {
	struct trustee_name newname;
	struct trustee_name rootname;
	unsigned is_root = 1;
	unsigned r = 0;
	struct trustee_hash_element *new;
	struct trustee_hash_element *root;

	if (!name->filename || !name->filename[0]) goto err0;

	if (!copy_trustee_name(&rootname, name)) goto err0;
	rootname.filename[1] = '\0';

	if (strlen(name->filename) > 1 && strcmp(name->filename, "/")) {
		add_trustee(&rootname, 1);
		is_root = 0;
	}

	if (must_copy) {
		if (!copy_trustee_name(&newname, name)) goto err1;
	} else {
		newname = *name;
	}

	new = vmalloc(sizeof(struct trustee_hash_element));
	if (!new) goto err2;
	new->name = newname;
	INIT_HLIST_NODE(&new->hash_list);
	INIT_LIST_HEAD(&new->perm_list);
	INIT_LIST_HEAD(&new->device_list);

	write_lock(&trustee_hash_lock);
	if (get_trustee_for_name(&newname, 0)) goto err3;

	if (is_root) {
		root = NULL;
	} else if (!(root = get_trustee_for_name(&rootname, 0))) {
		TS_ERR_MSG("Root trustee disappeared on us!\n");
		goto err3;
	}
	hlist_add_head(&new->hash_list, &trustee_hash[hash_slot(name)]);
	if (!is_root) {
		list_add_tail(&new->device_list, &root->device_list);
	}
	calculate_deepest_level(&newname);
	TS_DEBUG_MSG("Created '%s' trustee\n", newname.filename);
	r = 1;
err3:
	write_unlock(&trustee_hash_lock);
	if (!r) vfree(new);
err2:
	if (must_copy && !r) free_trustee_name(&newname);
err1:
	free_trustee_name(&rootname);
err0:
	return r;
}

/**
 * Add a permissions module to the trustee specified by name.
 */
static unsigned add_trustee_perm
    (struct trustee_name *name, struct trustee_permission acl)
{
	struct trustee_hash_element *r = NULL;
	struct trustee_permission_capsule *capsule;

	capsule = vmalloc(sizeof(struct trustee_permission_capsule));
	if (!capsule) {
		TS_ERR_MSG
		    ("Can not allocate memory for trustee capsule\n");
		return 0;
	}
	capsule->permission = acl;

	write_lock(&trustee_hash_lock);
	r = get_trustee_for_name(name, 0);

	if (r) {
		list_add_tail(&capsule->perm_list, &r->perm_list);
		write_unlock(&trustee_hash_lock);
		TS_DEBUG_MSG("Added permission capsule to '%s' trustee\n", name->filename);
		return 1;
	}
	write_unlock(&trustee_hash_lock);
	TS_ERR_MSG("trustee disappeared under us while trying to add perms\n");
	vfree(capsule);

	return 0;
}

/*
 * Get the mask for a trustee name.
 * This should be called with a lock on the trustee_hash (which should
 * not be released until you are done with the returned hash_element)!
 */
static int get_trustee_mask_for_name(struct trustee_name *name,
				     int oldmask, int height,
				     struct trustee_hash_element **element,
				     unsigned ignore_case)
{
	struct trustee_hash_element *e;
	int m;
	struct trustee_permission_capsule *l;
	int appl;
	e = get_trustee_for_name(name, ignore_case);
	if (!e) {
		return oldmask;
	}
	list_for_each_entry(l, &e->perm_list, perm_list) {
		if ((height < 0)
		    && (l->permission.mask & TRUSTEE_ONE_LEVEL_MASK))
			continue;
		if (element) {
			*element = e;
			element = NULL;
		}
		appl = ((!(l->permission.mask & TRUSTEE_IS_GROUP_MASK))
			&& (current->fsuid == l->permission.u.uid))
		    || (((l->permission.mask & TRUSTEE_IS_GROUP_MASK))
			&& (in_group_p(l->permission.u.gid)))
		    || (l->permission.mask & TRUSTEE_ALL_MASK);
		if (l->permission.mask & TRUSTEE_NOT_MASK)
			appl = !appl;

		if (!appl)
			continue;

		m = l->permission.mask & TRUSTEE_ACL_MASK;

		if (l->permission.mask & TRUSTEE_ALLOW_DENY_MASK)
			m <<= TRUSTEE_NUM_ACL_BITS;

		oldmask =
		    l->permission.
		    mask & TRUSTEE_CLEAR_SET_MASK ? (oldmask & (~m))
		    : (oldmask | m);
	}

	return oldmask;
}

/*
 * Return non-zero if a trustee exists in a subpath.
 *
 * WARNING!
 * This function requires that you lock/unlock the trustees_hash_lock
 */
int trustee_has_child(struct vfsmount *mnt, char *file_name)
{
	struct trustee_name trustee_name;
	char tempchar;
	unsigned ignore_case = 0;
	struct trustee_hash_element *root;
	size_t len;
	struct trustee_ic *iter;
	struct trustee_hash_element *r;

	if (!file_name || !*file_name) return 0;

	list_for_each_entry(iter, &trustee_ic_list, ic_list) {
		if (trustee_dev_cmp
		    (iter->dev, trustee_name.dev, iter->devname,
		     trustee_name.devname)) {
			ignore_case = 1;
			break;
		}
	}

	trustee_name.dev = mnt->mnt_sb->s_dev;
	trustee_name.devname = mnt->mnt_devname;
	trustee_name.filename = file_name;
	tempchar = file_name[1];
	file_name[1] = '\0';

	root = get_trustee_for_name(&trustee_name, ignore_case);
	if (!root) return 0;

	file_name[1] = tempchar;

	len = strlen(file_name);

	list_for_each_entry(r, &root->device_list, device_list) {
		size_t this_len = strlen(r->name.filename);
		if (this_len <= len) continue;
		if (!strncmp(file_name, r->name.filename, len) &&
			r->name.filename[len] != '\0')
			return 1;
	}

	return 0;
}

/*
 * Return the mask for a file.
 *
 * WARNING!
 * This function requires that you lock/unlock the trustees_hash_lock
 */
int trustee_perm(struct dentry *dentry, struct vfsmount *mnt,
		 char *file_name, int unix_ret, int depth, int is_dir,
		 struct trustee_hash_element **deepest)
{
	static char dbl_nul_slash[3] = { '/', '\0', '\0' };
	int oldmask = trustee_default_acl;
	int height = 0;
	char *filecount;
	char c;
	struct trustee_name trustee_name;
	struct trustee_ic *iter;
	unsigned ignore_case = 0;

	trustee_name.dev = mnt->mnt_sb->s_dev;
	trustee_name.devname = mnt->mnt_devname;
	trustee_name.filename = file_name;

	list_for_each_entry(iter, &trustee_ic_list, ic_list) {
		if (trustee_dev_cmp
		    (iter->dev, trustee_name.dev, iter->devname,
		     trustee_name.devname)) {
			ignore_case = 1;
			break;
		}
	}

	if (deepest) *deepest = NULL;

	filecount = file_name + 1;
	/* Try to handle the unlikely case where the string will be '/'
	 * out here to simplify the logic inside the loop.  We do this
	 * by giving it a string with two nul byte terminators so that it
	 * will gracefully (and safely) make it through the loop below.
	 */
	if (*filecount == '\0') {
		file_name = dbl_nul_slash;
		filecount = file_name + 1;
	}
	do {
		c = *filecount;
		*filecount = 0;
		oldmask =
		    get_trustee_mask_for_name(&trustee_name, oldmask,
					      height - depth + !is_dir,
					      deepest, ignore_case);
		height++;
		*filecount = c;
		++filecount;
		while ((*filecount) && (*filecount != '/')) filecount++;

	} while(*filecount);

	return oldmask;
}

/* Clear out the hash of trustees and release the hash itself.
 * Also gets rid of the ignore-case list
 */
static void trustees_clear_all(void)
{
	struct trustee_hash_element *item = NULL;
	struct hlist_node *iter, *temp = NULL;
	unsigned i;
	write_lock(&trustee_hash_lock);

	for (i = 0; i < trustee_hash_size; i++) {
		hlist_for_each_entry_safe(item, iter, temp, &trustee_hash[i], hash_list) {
			free_hash_element(item);
		}
		INIT_HLIST_HEAD(&trustee_hash[i]);
	}

	deepest_level = 0;

	remove_ic_devs();

	write_unlock(&trustee_hash_lock);
}

/*
 * Initialize globals
 */
int trustees_funcs_init_globals(void)
{
	unsigned int iter;

	if (trustee_hash_size <= 0)
		return 1;

	rwlock_init(&trustee_hash_lock);

	trustee_hash = vmalloc(sizeof(*trustee_hash) * trustee_hash_size);
	if (!trustee_hash)
		return 1;

	for (iter = 0; iter < trustee_hash_size; iter++)
		INIT_HLIST_HEAD(trustee_hash + iter);

	return 0;
}

/*
 * Clear globals
 */
int trustees_funcs_cleanup_globals(void)
{
	trustees_clear_all();

	vfree(trustee_hash);

	return 0;
}

/*
 * Prepare a trustee name from a passed in trustee name.
 */
static int prepare_trustee_name(u32 device, char *devname, char *filename, struct trustee_name *name)
{
	size_t devl, filel;
	char *devb = NULL, *fileb = NULL;

	if ((!name))
		return 0;

	filel = strlen(filename);
	devl = strlen(devname);

	if (devl > PATH_MAX) {
		TS_ERR_MSG("device name bad, command ignored.\n");
		return 0;
	}
	if (filel > PATH_MAX) {
		TS_ERR_MSG("file name bad, command ignored.\n");
		return 0;
	}

	devb = vmalloc_strdup(devname, devl);
	if (!devb) {
		TS_ERR_MSG("Couldn't allocate mem for devb.\n");
		return 0;
	}

	fileb = vmalloc_strdup(filename, filel);
	if (!fileb) {
		TS_ERR_MSG("Couldn't allocate mem for fileb.\n");
		return 0;
	}

	name->devname = devb;
	name->filename = fileb;

	name->dev = new_decode_dev(device);

	return 1;
}

/*
 * Process a user command
 */
extern int trustees_process_command(struct trustee_command command,
                                    void **arg,
                                    size_t *argsize)
{
	int r = -ENOSYS;
	int must_free = 0;
	struct trustee_name name;

	if ((current->euid != 0) && !capable(CAP_SYS_ADMIN)) {
		r = -EACCES;
		return r;
	}

	switch (command.command) {
	case TRUSTEE_COMMAND_MAKE_IC:
		if (command.numargs != 2 ||
		    argsize[1] != sizeof(u32)) goto unlk;
		add_ic_dev(*(u32 *)arg[1], arg[0]);
		r = 0;
		break;
	case TRUSTEE_COMMAND_REMOVE_ALL:
		if (command.numargs != 0) goto unlk;
		trustees_clear_all();
		r = 0;
		break;
	case TRUSTEE_COMMAND_ADD:
		if (command.numargs != 4 ||
		    argsize[3] != sizeof(u32) ||
		    argsize[1] != sizeof(struct trustee_permission))
			goto unlk;
		if (!prepare_trustee_name(*(u32 *)arg[3], arg[2], arg[0], &name)) {
			r = -ENOMEM;
			goto unlk;
		}
		if (!add_trustee(&name, 0)) {
			must_free = 1;
		}
		if (!add_trustee_perm(&name, *(struct trustee_permission *)arg[1]))
			r = -ENOMEM;
		else
			r = 0;

		if (must_free) free_trustee_name(&name);
		break;
	}
   unlk:

	return r;
}
