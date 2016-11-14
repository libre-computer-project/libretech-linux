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

#include "hvCall.h"
#include <asm/byteorder.h>
#include "purgatory.h"

void putchar(int c)
{
	char buff[8];
	unsigned long *lbuf = (unsigned long *)buff;

	if (!debug) /* running on non pseries */
		return;

	if (c == '\n')
		putchar('\r');

	buff[0] = c;
	plpar_hcall_norets(H_PUT_TERM_CHAR, 0, 1, __cpu_to_be64(*lbuf), 0);
}
