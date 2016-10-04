/*
 * kexec: Linux boots Linux
 *
 * Created by: Mohan Kumar M (mohan@in.ibm.com)
 *
 * Copyright (C) IBM Corporation, 2005. All rights reserved
 *
 * Code taken from kexec-tools.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation (version 2 of the License).
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "../boot/string.h"
#include "crashdump-ppc64.h"

extern unsigned long backup_start;

/* Backup first 32KB of memory to backup region reserved by kexec */
void crashdump_backup_memory(void)
{
	void *dest, *src;

	src = (void *)BACKUP_SRC_START;

	if (backup_start) {
		dest = (void *)(backup_start);
		memcpy(dest, src, BACKUP_SRC_SIZE);
	}
}
