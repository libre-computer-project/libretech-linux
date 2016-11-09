/*
 * Utility functions to work with ELF files.
 *
 * Copyright (C) 2016, IBM Corporation
 *
 * Based on kexec-tools' kexec-elf.c. Heavily modified for the
 * kernel by Thiago Jung Bauermann <bauerman@linux.vnet.ibm.com>.
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

#include <asm/elf_util.h>

/**
 * elf_toc_section - find the toc section in the file with the given ELF headers
 * @ehdr:	Pointer to already loaded ELF header.
 * @sechdrs:	Pointer to already loaded section headers contents.
 *
 * Return: TOC section index or 0 if one wasn't found.
 */
unsigned int elf_toc_section(const struct elfhdr *ehdr,
			     const struct elf_shdr *sechdrs)
{
	int i;
	const char *shstrtab;

	/* Section header string table. */
	shstrtab = (const char *) sechdrs[ehdr->e_shstrndx].sh_offset;

	for (i = 0; i < ehdr->e_shnum; i++) {
		if (sechdrs[i].sh_size == 0)
			continue;

		if (!strcmp(&shstrtab[sechdrs[i].sh_name], ".toc"))
			return i;
	}

	return 0;
}
