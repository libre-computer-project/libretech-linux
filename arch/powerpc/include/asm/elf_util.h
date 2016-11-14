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
	/*
	 * Where the ELF binary contents are kept.
	 * Memory managed by the user of the struct.
	 */
	const char *buffer;

	const struct elfhdr *ehdr;
	const struct elf_phdr *proghdrs;
	struct elf_shdr *sechdrs;
};

/*
 * r2 is the TOC pointer: it actually points 0x8000 into the TOC (this
 * gives the value maximum span in an instruction which uses a signed
 * offset)
 */
static inline unsigned long elf_my_r2(const struct elf_shdr *sechdrs,
				      unsigned int toc_section)
{
	return sechdrs[toc_section].sh_addr + 0x8000;
}

unsigned int elf_toc_section(const struct elfhdr *ehdr,
			     const struct elf_shdr *sechdrs);

int elf64_apply_relocate_add_item(const Elf64_Shdr *sechdrs, const char *strtab,
				  const Elf64_Rela *rela, const Elf64_Sym *sym,
				  unsigned long *location,
				  unsigned long address, unsigned long value,
				  unsigned long my_r2, const char *obj_name,
				  struct module *me);

static inline bool elf_is_elf_file(const struct elfhdr *ehdr)
{
	return memcmp(ehdr->e_ident, ELFMAG, SELFMAG) == 0;
}

int elf_read_from_buffer(const char *buf, size_t len, struct elfhdr *ehdr,
			 struct elf_info *elf_info);
void elf_free_info(struct elf_info *elf_info);

#endif /* _ASM_POWERPC_ELF_UTIL_H */
