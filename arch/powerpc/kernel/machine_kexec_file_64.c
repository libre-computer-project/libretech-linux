/*
 * ppc64 code to implement the kexec_file_load syscall
 *
 * Copyright (C) 2004  Adam Litke (agl@us.ibm.com)
 * Copyright (C) 2004  IBM Corp.
 * Copyright (C) 2005  R Sharada (sharada@in.ibm.com)
 * Copyright (C) 2006  Mohan Kumar M (mohan@in.ibm.com)
 * Copyright (C) 2016  IBM Corporation
 *
 * Based on kexec-tools' kexec-elf-ppc64.c.
 * Heavily modified for the kernel by
 * Thiago Jung Bauermann <bauerman@linux.vnet.ibm.com>.
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

#include <linux/slab.h>
#include <linux/kexec.h>
#include <linux/memblock.h>
#include <linux/libfdt.h>
#include <asm/elf_util.h>

#define SLAVE_CODE_SIZE		256

static struct kexec_file_ops *kexec_file_loaders[] = { };

int arch_kexec_kernel_image_probe(struct kimage *image, void *buf,
				  unsigned long buf_len)
{
	int i, ret = -ENOEXEC;
	struct kexec_file_ops *fops;

	/* We don't support crash kernels yet. */
	if (image->type == KEXEC_TYPE_CRASH)
		return -ENOTSUPP;

	for (i = 0; i < ARRAY_SIZE(kexec_file_loaders); i++) {
		fops = kexec_file_loaders[i];
		if (!fops || !fops->probe)
			continue;

		ret = fops->probe(buf, buf_len);
		if (!ret) {
			image->fops = fops;
			return ret;
		}
	}

	return ret;
}

void *arch_kexec_kernel_image_load(struct kimage *image)
{
	if (!image->fops || !image->fops->load)
		return ERR_PTR(-ENOEXEC);

	return image->fops->load(image, image->kernel_buf,
				 image->kernel_buf_len, image->initrd_buf,
				 image->initrd_buf_len, image->cmdline_buf,
				 image->cmdline_buf_len);
}

int arch_kimage_file_post_load_cleanup(struct kimage *image)
{
	if (!image->fops || !image->fops->cleanup)
		return 0;

	return image->fops->cleanup(image->image_loader_data);
}

/**
 * arch_kexec_walk_mem - call func(data) for each unreserved memory block
 * @kbuf:	Context info for the search. Also passed to @func.
 * @func:	Function to call for each memory block.
 *
 * This function is used by kexec_add_buffer and kexec_locate_mem_hole
 * to find unreserved memory to load kexec segments into.
 *
 * Return: The memory walk will stop when func returns a non-zero value
 * and that value will be returned. If all free regions are visited without
 * func returning non-zero, then zero will be returned.
 */
int arch_kexec_walk_mem(struct kexec_buf *kbuf, int (*func)(u64, u64, void *))
{
	int ret = 0;
	u64 i;
	phys_addr_t mstart, mend;

	if (kbuf->top_down) {
		for_each_free_mem_range_reverse(i, NUMA_NO_NODE, 0,
						&mstart, &mend, NULL) {
			/*
			 * In memblock, end points to the first byte after the
			 * range while in kexec, end points to the last byte
			 * in the range.
			 */
			ret = func(mstart, mend - 1, kbuf);
			if (ret)
				break;
		}
	} else {
		for_each_free_mem_range(i, NUMA_NO_NODE, 0, &mstart, &mend,
					NULL) {
			/*
			 * In memblock, end points to the first byte after the
			 * range while in kexec, end points to the last byte
			 * in the range.
			 */
			ret = func(mstart, mend - 1, kbuf);
			if (ret)
				break;
		}
	}

	return ret;
}

/**
 * arch_kexec_apply_relocations_add - apply purgatory relocations
 * @ehdr:	Pointer to ELF headers.
 * @sechdrs:	Pointer to section headers.
 * @relsec:	Section index of SHT_RELA section.
 *
 * Elf64_Shdr.sh_offset has been modified to keep the pointer to the section
 * contents, while Elf64_Shdr.sh_addr points to the final address of the
 * section in memory.
 */
int arch_kexec_apply_relocations_add(const Elf64_Ehdr *ehdr,
				     Elf64_Shdr *sechdrs, unsigned int relsec)
{
	unsigned int i;
	int ret;
	int reloc_type;
	unsigned long *location;
	unsigned long address;
	unsigned long value;
	const char *name;
	Elf64_Sym *sym;
	/* Section containing the relocation entries. */
	Elf64_Shdr *rel_section = &sechdrs[relsec];
	const Elf64_Rela *rela = (const Elf64_Rela *) rel_section->sh_offset;
	/* Section to which relocations apply. */
	Elf64_Shdr *target_section = &sechdrs[rel_section->sh_info];
	/* Associated symbol table. */
	Elf64_Shdr *symtabsec = &sechdrs[rel_section->sh_link];
	void *syms_base = (void *) symtabsec->sh_offset;
	void *loc_base = (void *) target_section->sh_offset;
	Elf64_Addr addr_base = target_section->sh_addr;
	unsigned long sec_base;
	unsigned long r2;
	unsigned int toc;
	const char *strtab;

	if (symtabsec->sh_link >= ehdr->e_shnum) {
		/* Invalid strtab section number */
		pr_err("Invalid string table section index %d\n",
		       symtabsec->sh_link);
		return -ENOEXEC;
	}

	toc = elf_toc_section(ehdr, sechdrs);
	if (!toc) {
		pr_err("Purgatory TOC section not found.");
		return -ENOEXEC;
	}

	r2 = elf_my_r2(sechdrs, toc);

	/* String table for the associated symbol table. */
	strtab = (const char *) sechdrs[symtabsec->sh_link].sh_offset;

	for (i = 0; i < rel_section->sh_size / sizeof(Elf64_Rela); i++) {
		/*
		 * rels[i].r_offset contains the byte offset from the beginning
		 * of section to the storage unit affected.
		 *
		 * This is the location to update in the temporary buffer where
		 * the section is currently loaded. The section will finally
		 * be loaded to a different address later, pointed to by
		 * addr_base.
		 */
		location = loc_base + rela[i].r_offset;

		/* Final address of the location. */
		address = addr_base + rela[i].r_offset;

		/* This is the symbol the relocation is referring to. */
		sym = (Elf64_Sym *) syms_base + ELF64_R_SYM(rela[i].r_info);

		if (sym->st_name)
			name = strtab + sym->st_name;
		else
			name = "<unnamed symbol>";

		reloc_type = ELF64_R_TYPE(rela[i].r_info);

		pr_debug("RELOC at %p: %i-type as %s (0x%lx) + %li\n",
		       location, reloc_type, name, (unsigned long)sym->st_value,
		       (long)rela[i].r_addend);

		/*
		 * TOC symbols appear as undefined but should be
		 * resolved as well, so allow them to be processed.
		 */
		if (sym->st_shndx == SHN_UNDEF && strcmp(name, ".TOC.") != 0 &&
				reloc_type != R_PPC64_TOC) {
			pr_err("Undefined symbol: %s\n", name);
			return -ENOEXEC;
		} else if (sym->st_shndx == SHN_COMMON) {
			pr_err("Symbol '%s' in common section.\n",
			       name);
			return -ENOEXEC;
		}

		if (sym->st_shndx != SHN_ABS) {
			if (sym->st_shndx >= ehdr->e_shnum) {
				pr_err("Invalid section %d for symbol %s\n",
				       sym->st_shndx, name);
				return -ENOEXEC;
			}

			sec_base = sechdrs[sym->st_shndx].sh_addr;
		} else
			sec_base = 0;

		/* `Everything is relative'. */
		value = sym->st_value + sec_base + rela[i].r_addend;

		ret = elf64_apply_relocate_add_item(sechdrs, strtab, &rela[i],
						    sym, location, address,
						    value, r2,
						    "kexec purgatory", NULL);
		if (ret)
			return ret;
	}

	return 0;
}
