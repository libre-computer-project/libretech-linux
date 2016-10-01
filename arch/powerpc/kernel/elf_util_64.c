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

#include <asm/ppc-opcode.h>
#include <asm/elf_util.h>

/*
 * We just need to use the functions defined in <asm/module.h>, so just declare
 * struct module here and avoid having to import <linux/module.h>.
 */
struct module;
#include <asm/module.h>

#ifdef PPC64_ELF_ABI_v2
/* PowerPC64 specific values for the Elf64_Sym st_other field.  */
#define STO_PPC64_LOCAL_BIT	5
#define STO_PPC64_LOCAL_MASK	(7 << STO_PPC64_LOCAL_BIT)
#define PPC64_LOCAL_ENTRY_OFFSET(other)					\
 (((1 << (((other) & STO_PPC64_LOCAL_MASK) >> STO_PPC64_LOCAL_BIT)) >> 2) << 2)

static unsigned int local_entry_offset(const Elf64_Sym *sym)
{
	/* sym->st_other indicates offset to local entry point
	 * (otherwise it will assume r12 is the address of the start
	 * of function and try to derive r2 from it). */
	return PPC64_LOCAL_ENTRY_OFFSET(sym->st_other);
}
#else
static unsigned int local_entry_offset(const Elf64_Sym *sym)
{
	return 0;
}
#endif

#ifdef CC_USING_MPROFILE_KERNEL
/*
 * In case of _mcount calls, do not save the current callee's TOC (in r2) into
 * the original caller's stack frame. If we did we would clobber the saved TOC
 * value of the original caller.
 */
static void squash_toc_save_inst(const char *name, unsigned long addr)
{
	struct ppc64_stub_entry *stub = (struct ppc64_stub_entry *)addr;

	/* Only for calls to _mcount */
	if (strcmp("_mcount", name) != 0)
		return;

	stub->jump[2] = PPC_INST_NOP;
}
#else
static void squash_toc_save_inst(const char *name, unsigned long addr) { }
#endif

/**
 * elf64_apply_relocate_add - apply 64 bit RELA relocations
 * @elf_info:		Support information for the ELF binary being relocated.
 * @strtab:		String table for the associated symbol table.
 * @symindex:		Section header index for the associated symbol table.
 * @relsec:		Section header index for the relocations to apply.
 * @obj_name:		The name of the ELF binary, for information messages.
 */
int elf64_apply_relocate_add(const struct elf_info *elf_info,
			     const char *strtab, unsigned int symindex,
			     unsigned int relsec, const char *obj_name)
{
	unsigned int i;
	Elf64_Shdr *sechdrs = elf_info->sechdrs;
	Elf64_Rela *rela = (void *)sechdrs[relsec].sh_addr;
	Elf64_Sym *sym;
	unsigned long *location;
	unsigned long value;


	for (i = 0; i < sechdrs[relsec].sh_size / sizeof(*rela); i++) {
		/* This is where to make the change */
		location = (void *)sechdrs[sechdrs[relsec].sh_info].sh_addr
			+ rela[i].r_offset;
		/* This is the symbol it is referring to */
		sym = (Elf64_Sym *)sechdrs[symindex].sh_addr
			+ ELF64_R_SYM(rela[i].r_info);

		pr_debug("RELOC at %p: %li-type as %s (0x%lx) + %li\n",
		       location, (long)ELF64_R_TYPE(rela[i].r_info),
		       strtab + sym->st_name, (unsigned long)sym->st_value,
		       (long)rela[i].r_addend);

		/* `Everything is relative'. */
		value = sym->st_value + rela[i].r_addend;

		switch (ELF64_R_TYPE(rela[i].r_info)) {
		case R_PPC64_ADDR32:
			/* Simply set it */
			*(u32 *)location = value;
			break;

		case R_PPC64_ADDR64:
			/* Simply set it */
			*(unsigned long *)location = value;
			break;

		case R_PPC64_TOC:
			*(unsigned long *)location = my_r2(elf_info);
			break;

		case R_PPC64_TOC16:
			/* Subtract TOC pointer */
			value -= my_r2(elf_info);
			if (value + 0x8000 > 0xffff) {
				pr_err("%s: bad TOC16 relocation (0x%lx)\n",
				       obj_name, value);
				return -ENOEXEC;
			}
			*((uint16_t *) location)
				= (*((uint16_t *) location) & ~0xffff)
				| (value & 0xffff);
			break;

		case R_PPC64_TOC16_LO:
			/* Subtract TOC pointer */
			value -= my_r2(elf_info);
			*((uint16_t *) location)
				= (*((uint16_t *) location) & ~0xffff)
				| (value & 0xffff);
			break;

		case R_PPC64_TOC16_DS:
			/* Subtract TOC pointer */
			value -= my_r2(elf_info);
			if ((value & 3) != 0 || value + 0x8000 > 0xffff) {
				pr_err("%s: bad TOC16_DS relocation (0x%lx)\n",
				       obj_name, value);
				return -ENOEXEC;
			}
			*((uint16_t *) location)
				= (*((uint16_t *) location) & ~0xfffc)
				| (value & 0xfffc);
			break;

		case R_PPC64_TOC16_LO_DS:
			/* Subtract TOC pointer */
			value -= my_r2(elf_info);
			if ((value & 3) != 0) {
				pr_err("%s: bad TOC16_LO_DS relocation (0x%lx)\n",
				       obj_name, value);
				return -ENOEXEC;
			}
			*((uint16_t *) location)
				= (*((uint16_t *) location) & ~0xfffc)
				| (value & 0xfffc);
			break;

		case R_PPC64_TOC16_HA:
			/* Subtract TOC pointer */
			value -= my_r2(elf_info);
			value = ((value + 0x8000) >> 16);
			*((uint16_t *) location)
				= (*((uint16_t *) location) & ~0xffff)
				| (value & 0xffff);
			break;

		case R_PPC_REL24:
			/* FIXME: Handle weak symbols here --RR */
			if (sym->st_shndx == SHN_UNDEF) {
				/* External: go via stub */
				value = stub_for_addr(elf_info, value, obj_name);
				if (!value)
					return -ENOENT;
				if (!restore_r2((u32 *)location + 1, obj_name))
					return -ENOEXEC;

				squash_toc_save_inst(strtab + sym->st_name, value);
			} else
				value += local_entry_offset(sym);

			/* Convert value to relative */
			value -= (unsigned long)location;
			if (value + 0x2000000 > 0x3ffffff || (value & 3) != 0){
				pr_err("%s: REL24 %li out of range!\n",
				       obj_name, (long int)value);
				return -ENOEXEC;
			}

			/* Only replace bits 2 through 26 */
			*(uint32_t *)location
				= (*(uint32_t *)location & ~0x03fffffc)
				| (value & 0x03fffffc);
			break;

		case R_PPC64_REL64:
			/* 64 bits relative (used by features fixups) */
			*location = value - (unsigned long)location;
			break;

		case R_PPC64_TOCSAVE:
			/*
			 * Marker reloc indicates we don't have to save r2.
			 * That would only save us one instruction, so ignore
			 * it.
			 */
			break;

		case R_PPC64_ENTRY:
			/*
			 * Optimize ELFv2 large code model entry point if
			 * the TOC is within 2GB range of current location.
			 */
			value = my_r2(elf_info) - (unsigned long)location;
			if (value + 0x80008000 > 0xffffffff)
				break;
			/*
			 * Check for the large code model prolog sequence:
			 *	ld r2, ...(r12)
			 *	add r2, r2, r12
			 */
			if ((((uint32_t *)location)[0] & ~0xfffc)
			    != 0xe84c0000)
				break;
			if (((uint32_t *)location)[1] != 0x7c426214)
				break;
			/*
			 * If found, replace it with:
			 *	addis r2, r12, (.TOC.-func)@ha
			 *	addi r2, r12, (.TOC.-func)@l
			 */
			((uint32_t *)location)[0] = 0x3c4c0000 + PPC_HA(value);
			((uint32_t *)location)[1] = 0x38420000 + PPC_LO(value);
			break;

		case R_PPC64_REL16_HA:
			/* Subtract location pointer */
			value -= (unsigned long)location;
			value = ((value + 0x8000) >> 16);
			*((uint16_t *) location)
				= (*((uint16_t *) location) & ~0xffff)
				| (value & 0xffff);
			break;

		case R_PPC64_REL16_LO:
			/* Subtract location pointer */
			value -= (unsigned long)location;
			*((uint16_t *) location)
				= (*((uint16_t *) location) & ~0xffff)
				| (value & 0xffff);
			break;

		default:
			pr_err("%s: Unknown ADD relocation: %lu\n",
			       obj_name,
			       (unsigned long)ELF64_R_TYPE(rela[i].r_info));
			return -ENOEXEC;
		}
	}

	return 0;
}
