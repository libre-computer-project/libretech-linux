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
 * Module initialization and cleanup
 *
 * History:
 *  2002-12-16 trustees 2.10 released by Vyacheslav Zavadsky
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/security.h>
#include <linux/capability.h>

#include "internal.h"

unsigned int trustee_hash_size = 256;

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Trustees ACL System");
MODULE_AUTHOR("Vyacheslav Zavadsky and Andrew E. Ruder <aeruder@ksu.edu>");
MODULE_VERSION("2.11");

MODULE_PARM_DESC(hash_size, "Trustees hash size");
module_param_named(hash_size, trustee_hash_size, uint, 0444);


static int __init trustees_init(void)
{
	if (trustees_funcs_init_globals() != 0) {
		return -EINVAL;
	}

	if (trustees_init_fs() != 0) {
		trustees_funcs_cleanup_globals();
		return -EINVAL;
	}

	if (trustees_init_security() != 0) {
		trustees_deinit_fs();
		trustees_funcs_cleanup_globals();
		return -EINVAL;
	}

	return 0;
}

fs_initcall(trustees_init);
