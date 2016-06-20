/*
 * linux/include/asm-arm/arch-oxnas/io.h
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARM_ARCH_IO_H
#define __ASM_ARM_ARCH_IO_H

#define IO_SPACE_LIMIT 0xffffffff

#define __io(a) 	  ((void __iomem*)(a))
#define __mem_pci(a) (a)

#endif //__ASM_ARM_ARCH_IO_H
