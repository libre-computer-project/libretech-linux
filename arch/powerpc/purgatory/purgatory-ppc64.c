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

#include "purgatory.h"

unsigned long stack __section(".data");
unsigned long dt_offset __section(".data");
unsigned long my_toc __section(".data");
unsigned long kernel __section(".data");
int debug __section(".data");
unsigned long opal_base __section(".data");
unsigned long opal_entry __section(".data");

void setup_arch(void)
{
}

void post_verification_setup_arch(void)
{
}
