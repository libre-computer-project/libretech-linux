/*
 * linux/arch/arm/mach-oxnas/samba_receive.c
 *
 * Copyright (C) 2008 Oxford Semiconductor Ltd
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
#include <linux/errno.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/kernel.h>

typedef struct xfs_flock64 {
	__s16		l_type;
	__s16		l_whence;
	__s64		l_start;
	__s64		l_len;		/* len == 0 means until end of file */
	__s32		l_sysid;
	__u32		l_pid;
	__s32		l_pad[4];	/* reserve area			    */
} xfs_flock64_t;

#define XFS_IOC_RESVSP64	_IOW ('X', 42, struct xfs_flock64)

asmlinkage long sys_samba_reserve(
	int          fd,
	void __user *info)
{
	struct file *file = fget(fd);
	long         ret = -EINVAL;

	/* Do I need any locking around the unlocked_ioctl() call? */
	ret = file->f_op->unlocked_ioctl(file, XFS_IOC_RESVSP64, (unsigned long)info);

	fput(file);

	return ret;
}
