/*
 * arch/arm64/include/asm/setup.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_SETUP_H
#define __ASM_SETUP_H

#include <uapi/asm/setup.h>

static inline unsigned long kaslr_offset(void)
{
	return kimage_vaddr - KIMAGE_VADDR;
}

#endif
