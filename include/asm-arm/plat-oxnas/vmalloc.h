/* linux/include/asm-arm/arch-oxnas/vmalloc.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_VMALLOC_H
#define __ASM_ARCH_VMALLOC_H

/*
 * Just any arbitrary offset to the start of the vmalloc VM area: the
 * current 8MB value just means that there will be a 8MB "hole" after the
 * physical memory until the kernel virtual memory starts.  That means that
 * any out-of-bounds memory accesses will hopefully be caught.
 * The vmalloc() routines leaves a hole of 4kB between each vmalloced
 * area for the same reason. ;)
 */

#define VMALLOC_OFFSET      (8*1024*1024)
/* Fix the VMALLOC start adr from the maximum possible SDRAM adr, so that
 * it's possible to have Linux use only part of the available SDRAM without
 * vmalloc/ioremap aliasing with the kernel mapping of the entire SDRAM */
#define MAX_SDRAM_ADR       (__phys_to_virt(SDRAM_PA) + (SDRAM_SIZE))
#define VMALLOC_START       (((unsigned long)MAX_SDRAM_ADR + VMALLOC_OFFSET) & ~(VMALLOC_OFFSET-1))
#define VMALLOC_VMADDR(x)   ((unsigned long)(x))
#define VMALLOC_END	         (0xE0000000)

#endif // __ASM_ARCH_VMALLOC_H
