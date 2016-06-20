/*
 *  linux/include/asm-arm/arch-oxnas/memory.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

/* Max. size of each memory node */
#ifdef CONFIG_OXNAS_VERSION_0X800
#define NODE_MAX_MEM_SHIFT (26) /* 64MB*/
#elif defined (CONFIG_OXNAS_VERSION_0X810)
#define NODE_MAX_MEM_SHIFT (28) /* 256MB */
#elif defined (CONFIG_OXNAS_VERSION_0X820)
/*#define NODE_MAX_MEM_SHIFT (29)*/ /* 512MB */
#define NODE_MAX_MEM_SHIFT (28) /* 256MB */
#endif // CONFIG_OXNAS_VERSION_0X800

#ifdef CONFIG_OXNAS_VERSION_0X800
#define MEM_MAP_ALIAS_SHIFT 28
#elif defined (CONFIG_OXNAS_VERSION_0X810)
#define MEM_MAP_ALIAS_SHIFT 30
#elif defined (CONFIG_OXNAS_VERSION_0X820)
#define MEM_MAP_ALIAS_SHIFT 30
#endif // CONFIG_OXNAS_VERSION_0X800

#define SDRAM_PA    (0x48000000)
#define SDRAM_SIZE  (1UL << (NODE_MAX_MEM_SHIFT))

/* 800/810 have SRAM following on contiguously from SDRAM, this may not be so for 820 */
#ifdef CONFIG_OXNAS_VERSION_0X820
#define SRAM_PA	    (0x58000000)
#else
#define SRAM_PA     ((SDRAM_PA) + (SDRAM_SIZE))
#endif // CONFIG_OXNAS_VERSION_0X820

/* Only a portion of the SRAM may be available for the use of Linux */
#define SRAM_SIZE   (CONFIG_SRAM_NUM_PAGES * PAGE_SIZE)

#define SDRAM_END   (SDRAM_PA + SDRAM_SIZE - 1)
#define SRAM_END    (SRAM_PA  + SRAM_SIZE  - 1)

#define PHYS_OFFSET SDRAM_PA
#define PAGE_OFFSET 0xC0000000

#define __virt_to_phys(x)   ((x) - PAGE_OFFSET + PHYS_OFFSET)
#define __phys_to_virt(x)   ((x) - PHYS_OFFSET + PAGE_OFFSET)

#define __virt_to_bus(x) __virt_to_phys(x)
#define __bus_to_virt(x) __phys_to_virt(x)

#ifdef CONFIG_DISCONTIGMEM
/*
 * Memory map aliased every 1GByte, i.e. top 2 bits are ignored.
 *
 * Currently (0X800) we have:
 *
 *  Start of physical memory: 0x08000000
 *                            0x48000000 - alias
 *                            0x88000000 - alias
 *                            0xC8000000 - alias
 *
 *  Node 0 SDRAM: 0x08000000 - 0x09FFFFFF   32MB populated
 *              : 0x48000000 - alias
 *              : 0x88000000 - alias
 *              : 0xC8000000 - alias
 *
 *  Node 1 SRAM : 0x0C000000 - 0x00008000   32KB populated
 *              : 0x4C000000 - alias
 *              : 0x8C000000 - alias
 *              : 0xCC000000 - alias
 *
 * It will be assumed that no single memory node can be larger than
 * (1 << NODE_MAX_MEM_SIZE) and that nodes will be contiguous, although
 * individual nodes may not be fully populated
 */

/*
 * Given a kernel address, find the home node of the underlying memory.
 */
#define KVADDR_TO_NID(addr) (((unsigned long)(addr) - PAGE_OFFSET) >> NODE_MAX_MEM_SHIFT)

/*
 * Given a page frame number, convert it to a node id.
 */
#define PFN_TO_NID(pfn) (((pfn) - PHYS_PFN_OFFSET) >> (NODE_MAX_MEM_SHIFT - PAGE_SHIFT))

/*
 * Given a kaddr, ADDR_TO_MAPBASE finds the owning node of the memory
 * and return the mem_map of that node.
 */
#define ADDR_TO_MAPBASE(kaddr) NODE_MEM_MAP(KVADDR_TO_NID(kaddr))

/*
 * Given a page frame number, find the owning node of the memory
 * and return the mem_map of that node.
 */
#define PFN_TO_MAPBASE(pfn) NODE_MEM_MAP(PFN_TO_NID(pfn))

/*
 * Given a kaddr, LOCAL_MAP_NR finds the owning node of the memory
 * and returns the index corresponding to the appropriate page in the
 * node's mem_map.
 */
#define LOCAL_MAP_NR(addr) (((unsigned long)(addr) & ((1 << NODE_MAX_MEM_SHIFT) - 1)) >> PAGE_SHIFT)

#else

#define PFN_TO_NID(addr) (0)

#endif

#endif // __ASM_ARCH_MEMORY_H
