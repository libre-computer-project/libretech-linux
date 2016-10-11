/*
 * Utility functions to work with ELF files.
 *
 * Copyright (C) 2016, IBM Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _ASM_POWERPC_ELF_UTIL_H
#define _ASM_POWERPC_ELF_UTIL_H

#include <linux/elf.h>

struct elf_info {
	struct elf_shdr *sechdrs;

	/* Index of stubs section. */
	unsigned int stubs_section;
	/* Index of TOC section. */
	unsigned int toc_section;
};

#ifdef __powerpc64__
#ifdef PPC64_ELF_ABI_v2

/* An address is simply the address of the function. */
typedef unsigned long func_desc_t;
#else

/* An address is address of the OPD entry, which contains address of fn. */
typedef struct ppc64_opd_entry func_desc_t;
#endif /* PPC64_ELF_ABI_v2 */

/* Like PPC32, we need little trampolines to do > 24-bit jumps (into
   the kernel itself).  But on PPC64, these need to be used for every
   jump, actually, to reset r2 (TOC+0x8000). */
struct ppc64_stub_entry
{
	/* 28 byte jump instruction sequence (7 instructions). We only
	 * need 6 instructions on ABIv2 but we always allocate 7 so
	 * so we don't have to modify the trampoline load instruction. */
	u32 jump[7];
	/* Used by ftrace to identify stubs */
	u32 magic;
	/* Data for the above code */
	func_desc_t funcdata;
};
#endif

/* r2 is the TOC pointer: it actually points 0x8000 into the TOC (this
   gives the value maximum span in an instruction which uses a signed
   offset) */
static inline unsigned long my_r2(const struct elf_info *elf_info)
{
	return elf_info->sechdrs[elf_info->toc_section].sh_addr + 0x8000;
}

int elf64_apply_relocate_add(const struct elf_info *elf_info,
			     const char *strtab, unsigned int symindex,
			     unsigned int relsec, const char *obj_name);

#endif /* _ASM_POWERPC_ELF_UTIL_H */
