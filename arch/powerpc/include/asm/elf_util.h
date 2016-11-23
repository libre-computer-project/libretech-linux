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

static inline bool elf_is_elf_file(const struct elfhdr *ehdr)
{
	return memcmp(ehdr->e_ident, ELFMAG, SELFMAG) == 0;
}

int elf_read_from_buffer(const char *buf, size_t len, struct elfhdr *ehdr,
			 struct elf_info *elf_info);
void elf_free_info(struct elf_info *elf_info);

#endif /* _ASM_POWERPC_ELF_UTIL_H */
