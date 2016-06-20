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
 * This code handles the virtual filesystem for trustees.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/vmalloc.h>
#include <linux/security.h>
#include <asm/atomic.h>
#include <asm/uaccess.h>

#include "internal.h"


/* initialization code for the trustees filesystem */

/* File operations
 *
 * this is all the code for handling the file operations done on the few files
 * in the trustees filesystem
 */
static int trustees_open(struct inode *inode, struct file *filp);
static ssize_t trustees_read_bogus(struct file *filp, char __user * buf,
				   size_t count, loff_t * offset);
static ssize_t trustees_write_bogus(struct file *filp,
				    const char __user * buf, size_t count,
				    loff_t * offset);
static ssize_t trustees_read_status(struct file *filp, char __user * buf,
				    size_t count, loff_t * offset);
static ssize_t trustees_read_apiversion(struct file *filp, char __user * buf,
				    size_t count, loff_t * offset);
static ssize_t trustees_write_trustees(struct file *filp,
				       const char __user * buf,
				       size_t count, loff_t * offset);
static int trustees_open_trustees(struct inode *inode, struct file *file);
static int trustees_release_trustees(struct inode *inode, struct file *file);

/* Various structs
 */

static struct file_operations trustees_ops_apiversion = {
	.open = trustees_open,
	.read = trustees_read_apiversion,
	.write = trustees_write_bogus,
};

static struct file_operations trustees_ops_status = {
	.open = trustees_open,
	.read = trustees_read_status,
	.write = trustees_write_bogus
};

static struct file_operations trustees_ops_trustees = {
	.open = trustees_open_trustees,
	.read = trustees_read_bogus,
	.write = trustees_write_trustees,
	.release = trustees_release_trustees
};

static struct trustees_file_info {
	const char *name;
	struct file_operations *fops;
	int mode;
	struct dentry *dentry;
} trustees_files[] = {
	{.name = "device",
	 .fops = &trustees_ops_trustees,
	 .mode = S_IWUSR,
	 .dentry = 0
	 },
	{.name = "status",
	 .fops = &trustees_ops_status,
	 .mode = S_IRUSR,
	 .dentry = 0
	 },
	{.name = "apiversion",
	 .fops = &trustees_ops_apiversion,
	 .mode = S_IRUSR | S_IRGRP | S_IROTH,
	 .dentry = 0
	 },
	{"", NULL, 0, 0}
};

struct trustee_command_reader {
	struct trustee_command command;
	unsigned curarg;
	void *arg[TRUSTEE_MAX_ARGS];
	size_t argsize[TRUSTEE_MAX_ARGS];
};


static struct dentry *toplevel = NULL;

int trustees_init_fs(void)
{
	struct trustees_file_info *iter;
	toplevel = securityfs_create_dir("trustees", NULL);
	if (!toplevel) trustees_deinit_fs();
	for (iter = trustees_files; iter->fops && toplevel; iter++) {
		iter->dentry = securityfs_create_file(
		   iter->name, iter->mode, toplevel, NULL, iter->fops);
		if (!iter->dentry) trustees_deinit_fs();
	}
	return !toplevel;
}

void trustees_deinit_fs(void)
{
	struct trustees_file_info *iter;
	for (iter = trustees_files; iter->fops; iter++) {
		securityfs_remove(iter->dentry);
		iter->dentry = NULL;
	}
	securityfs_remove(toplevel);
	toplevel = NULL;
}

/*
 * They're opening the file...
 */

static int trustees_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int trustees_open_trustees(struct inode *inode, struct file *file)
{
	file->private_data = vmalloc(sizeof(struct trustee_command_reader));
	if (!file->private_data)
		return -ENOMEM;

	memset(file->private_data, 0, sizeof(struct trustee_command_reader));

	return 0;
}

static int trustees_release_trustees(struct inode *inode, struct file *file)
{
	vfree(file->private_data);
	return 0;
}

/* Do a read on a bogus file.  Just return nothing :) */
static ssize_t trustees_read_bogus(struct file *filp, char __user * buf,
				   size_t count, loff_t * offset)
{
	return 0;
}

/* Similar way to handle writes.  Just say we wrote the data and return */
static ssize_t trustees_write_bogus(struct file *filp,
				    const char __user * buf, size_t count,
				    loff_t * offset)
{
	return count;
}

/* Function for handling reading of the status. */
static ssize_t trustees_read_status(struct file *filp, char __user * buf,
				    size_t count, loff_t * offset)
{
	static const char msg[] = "Damnit, it works, OK?!\n";
	unsigned long nocopy;

	if (*offset >= (sizeof(msg) - 1)) {
		return 0;
	}

	if (count > (sizeof(msg) - 1 - *offset)) {
		count = sizeof(msg) - 1 - *offset;
	}
	nocopy = copy_to_user(buf, msg, count);
	(*offset) += count;
	(*offset) -= nocopy;

	return count;
}

/* Function for handling reading of the apiversion. */
static ssize_t trustees_read_apiversion(struct file *filp, char __user * buf,
				    size_t count, loff_t * offset)
{
	static const char msg[] = TRUSTEES_APIVERSION_STR "\n";
	unsigned long nocopy;

	if (*offset >= (sizeof(msg) - 1)) {
		return 0;
	}

	if (count > (sizeof(msg) - 1 - *offset)) {
		count = sizeof(msg) - 1 - *offset;
	}
	nocopy = copy_to_user(buf, msg, count);
	(*offset) += count;
	(*offset) -= nocopy;

	return count;
}

/* Cleanup our reader (deallocate all the allocated memory) */
static void cleanup_reader(struct trustee_command_reader *reader) {
	int z;
	if (!reader) {
		TS_ERR_MSG("How does reader disappear on us?\n");
		return;
	}

	for (z = reader->curarg - 1; z >= 0; z--) {
		vfree(reader->arg[z]);
		reader->argsize[z] = 0;
	}
	reader->command.command = 0;
	reader->curarg = 0;
}

static ssize_t trustees_write_trustees(struct file *filp,
				       const char __user * buf,
				       size_t count, loff_t * offset)
{
	struct trustee_command_reader *reader = filp->private_data;

	if (reader->command.command == 0) {
		reader->curarg = 0;
		if (count != sizeof(struct trustee_command)) {
			return -EIO;
		}
		if (copy_from_user(&reader->command, buf, count)) {
			reader->command.command = 0;
			TS_ERR_MSG("copy_from_user failed on command\n");
			return -EIO;
		}
		if (reader->command.numargs > TRUSTEE_MAX_ARGS) {
			TS_ERR_MSG("Too many arguments specified for command %d\n",
			  reader->command.command);
			return -EIO;
		}
	} else {
		unsigned curarg = reader->curarg;
		if (!(reader->arg[curarg] = vmalloc(count+1))) {
			cleanup_reader(reader);
			return -EIO;
		}
		reader->argsize[curarg] = count;
		((char *)reader->arg[curarg])[count] = '\0';
		reader->curarg++;
		if (copy_from_user(reader->arg[curarg], buf, count)) {
			cleanup_reader(reader);
			TS_ERR_MSG("copy_from_user failed on arg\n");
			return -EIO;
		}
	}

	if (reader->command.command && reader->curarg == reader->command.numargs) {
		int ret = trustees_process_command(reader->command, reader->arg,
		                                   reader->argsize);
		cleanup_reader(reader);
		if (ret) return -EIO;
	}

	return count;
}
