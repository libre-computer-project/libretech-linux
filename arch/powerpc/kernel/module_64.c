/*  Kernel module help for PPC64.
    Copyright (C) 2001, 2003 Rusty Russell IBM Corporation.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <asm/elf_util.h>
#include <linux/moduleloader.h>
#include <linux/err.h>
#include <linux/vmalloc.h>
#include <linux/ftrace.h>
#include <linux/bug.h>
#include <linux/uaccess.h>
#include <asm/module.h>
#include <asm/firmware.h>
#include <asm/code-patching.h>
#include <linux/sort.h>
#include <asm/setup.h>
#include <asm/sections.h>

/* FIXME: We don't do .init separately.  To do this, we'd need to have
   a separate r2 value in the init and core section, and stub between
   them, too.

   Using a magic allocator which places modules within 32MB solves
   this, and makes other things simpler.  Anton?
   --RR.  */

#ifdef PPC64_ELF_ABI_v2

static func_desc_t func_desc(unsigned long addr)
{
	return addr;
}
static unsigned long func_addr(unsigned long addr)
{
	return addr;
}
static unsigned long stub_func_addr(func_desc_t func)
{
	return func;
}
#else

static func_desc_t func_desc(unsigned long addr)
{
	return *(struct ppc64_opd_entry *)addr;
}
static unsigned long func_addr(unsigned long addr)
{
	return func_desc(addr).funcaddr;
}
static unsigned long stub_func_addr(func_desc_t func)
{
	return func.funcaddr;
}
#endif

#define STUB_MAGIC 0x73747562 /* stub */

/*
 * PPC64 uses 24 bit jumps, but we need to jump into other modules or
 * the kernel which may be further.  So we jump to a stub.
 *
 * For ELFv1 we need to use this to set up the new r2 value (aka TOC
 * pointer).  For ELFv2 it's the callee's responsibility to set up the
 * new r2, but for both we need to save the old r2.
 *
 * We could simply patch the new r2 value and function pointer into
 * the stub, but it's significantly shorter to put these values at the
 * end of the stub code, and patch the stub address (32-bits relative
 * to the TOC ptr, r2) into the stub.
 */

static u32 ppc64_stub_insns[] = {
	0x3d620000,			/* addis   r11,r2, <high> */
	0x396b0000,			/* addi    r11,r11, <low> */
	/* Save current r2 value in magic place on the stack. */
	0xf8410000|R2_STACK_OFFSET,	/* std     r2,R2_STACK_OFFSET(r1) */
	0xe98b0020,			/* ld      r12,32(r11) */
#ifdef PPC64_ELF_ABI_v1
	/* Set up new r2 from function descriptor */
	0xe84b0028,			/* ld      r2,40(r11) */
#endif
	0x7d8903a6,			/* mtctr   r12 */
	0x4e800420			/* bctr */
};

#ifdef CONFIG_DYNAMIC_FTRACE
int module_trampoline_target(struct module *mod, unsigned long addr,
			     unsigned long *target)
{
	struct ppc64_stub_entry *stub;
	func_desc_t funcdata;
	u32 magic;

	if (!within_module_core(addr, mod)) {
		pr_err("%s: stub %lx not in module %s\n", __func__, addr, mod->name);
		return -EFAULT;
	}

	stub = (struct ppc64_stub_entry *)addr;

	if (probe_kernel_read(&magic, &stub->magic, sizeof(magic))) {
		pr_err("%s: fault reading magic for stub %lx for %s\n", __func__, addr, mod->name);
		return -EFAULT;
	}

	if (magic != STUB_MAGIC) {
		pr_err("%s: bad magic for stub %lx for %s\n", __func__, addr, mod->name);
		return -EFAULT;
	}

	if (probe_kernel_read(&funcdata, &stub->funcdata, sizeof(funcdata))) {
		pr_err("%s: fault reading funcdata for stub %lx for %s\n", __func__, addr, mod->name);
                return -EFAULT;
	}

	*target = stub_func_addr(funcdata);

	return 0;
}
#endif

/* Count how many different 24-bit relocations (different symbol,
   different addend) */
static unsigned int count_relocs(const Elf64_Rela *rela, unsigned int num)
{
	unsigned int i, r_info, r_addend, _count_relocs;

	/* FIXME: Only count external ones --RR */
	_count_relocs = 0;
	r_info = 0;
	r_addend = 0;
	for (i = 0; i < num; i++)
		/* Only count 24-bit relocs, others don't need stubs */
		if (ELF64_R_TYPE(rela[i].r_info) == R_PPC_REL24 &&
		    (r_info != ELF64_R_SYM(rela[i].r_info) ||
		     r_addend != rela[i].r_addend)) {
			_count_relocs++;
			r_info = ELF64_R_SYM(rela[i].r_info);
			r_addend = rela[i].r_addend;
		}

	return _count_relocs;
}

static int relacmp(const void *_x, const void *_y)
{
	const Elf64_Rela *x, *y;

	y = (Elf64_Rela *)_x;
	x = (Elf64_Rela *)_y;

	/* Compare the entire r_info (as opposed to ELF64_R_SYM(r_info) only) to
	 * make the comparison cheaper/faster. It won't affect the sorting or
	 * the counting algorithms' performance
	 */
	if (x->r_info < y->r_info)
		return -1;
	else if (x->r_info > y->r_info)
		return 1;
	else if (x->r_addend < y->r_addend)
		return -1;
	else if (x->r_addend > y->r_addend)
		return 1;
	else
		return 0;
}

static void relaswap(void *_x, void *_y, int size)
{
	uint64_t *x, *y, tmp;
	int i;

	y = (uint64_t *)_x;
	x = (uint64_t *)_y;

	for (i = 0; i < sizeof(Elf64_Rela) / sizeof(uint64_t); i++) {
		tmp = x[i];
		x[i] = y[i];
		y[i] = tmp;
	}
}

/* Get size of potential trampolines required. */
static unsigned long get_stubs_size(const Elf64_Ehdr *hdr,
				    const Elf64_Shdr *sechdrs)
{
	/* One extra reloc so it's always 0-funcaddr terminated */
	unsigned long relocs = 1;
	unsigned i;

	/* Every relocated section... */
	for (i = 1; i < hdr->e_shnum; i++) {
		if (sechdrs[i].sh_type == SHT_RELA) {
			pr_debug("Found relocations in section %u\n", i);
			pr_debug("Ptr: %p.  Number: %Lu\n",
			       (void *)sechdrs[i].sh_addr,
			       sechdrs[i].sh_size / sizeof(Elf64_Rela));

			/* Sort the relocation information based on a symbol and
			 * addend key. This is a stable O(n*log n) complexity
			 * alogrithm but it will reduce the complexity of
			 * count_relocs() to linear complexity O(n)
			 */
			sort((void *)sechdrs[i].sh_addr,
			     sechdrs[i].sh_size / sizeof(Elf64_Rela),
			     sizeof(Elf64_Rela), relacmp, relaswap);

			relocs += count_relocs((void *)sechdrs[i].sh_addr,
					       sechdrs[i].sh_size
					       / sizeof(Elf64_Rela));
		}
	}

#ifdef CONFIG_DYNAMIC_FTRACE
	/* make the trampoline to the ftrace_caller */
	relocs++;
#endif

	pr_debug("Looks like a total of %lu stubs, max\n", relocs);
	return relocs * sizeof(struct ppc64_stub_entry);
}

/* Still needed for ELFv2, for .TOC. */
static void dedotify_versions(struct modversion_info *vers,
			      unsigned long size)
{
	struct modversion_info *end;

	for (end = (void *)vers + size; vers < end; vers++)
		if (vers->name[0] == '.') {
			memmove(vers->name, vers->name+1, strlen(vers->name));
#ifdef ARCH_RELOCATES_KCRCTAB
			/* The TOC symbol has no CRC computed. To avoid CRC
			 * check failing, we must force it to the expected
			 * value (see CRC check in module.c).
			 */
			if (!strcmp(vers->name, "TOC."))
				vers->crc = -(unsigned long)reloc_start;
#endif
		}
}

/*
 * Undefined symbols which refer to .funcname, hack to funcname. Make .TOC.
 * seem to be defined (value set later).
 */
static void dedotify(Elf64_Sym *syms, unsigned int numsyms, char *strtab)
{
	unsigned int i;

	for (i = 1; i < numsyms; i++) {
		if (syms[i].st_shndx == SHN_UNDEF) {
			char *name = strtab + syms[i].st_name;
			if (name[0] == '.') {
				if (strcmp(name+1, "TOC.") == 0)
					syms[i].st_shndx = SHN_ABS;
				syms[i].st_name++;
			}
		}
	}
}

static Elf64_Sym *find_dot_toc(Elf64_Shdr *sechdrs,
			       const char *strtab,
			       unsigned int symindex)
{
	unsigned int i, numsyms;
	Elf64_Sym *syms;

	syms = (Elf64_Sym *)sechdrs[symindex].sh_addr;
	numsyms = sechdrs[symindex].sh_size / sizeof(Elf64_Sym);

	for (i = 1; i < numsyms; i++) {
		if (syms[i].st_shndx == SHN_ABS
		    && strcmp(strtab + syms[i].st_name, "TOC.") == 0)
			return &syms[i];
	}
	return NULL;
}

int module_frob_arch_sections(Elf64_Ehdr *hdr,
			      Elf64_Shdr *sechdrs,
			      char *secstrings,
			      struct module *me)
{
	unsigned int i;

	/* Find .toc and .stubs sections, symtab and strtab */
	for (i = 1; i < hdr->e_shnum; i++) {
		char *p;
		if (strcmp(secstrings + sechdrs[i].sh_name, ".stubs") == 0)
			me->arch.elf_info.stubs_section = i;
		else if (strcmp(secstrings + sechdrs[i].sh_name, ".toc") == 0)
			me->arch.elf_info.toc_section = i;
		else if (strcmp(secstrings+sechdrs[i].sh_name,"__versions")==0)
			dedotify_versions((void *)hdr + sechdrs[i].sh_offset,
					  sechdrs[i].sh_size);

		/* We don't handle .init for the moment: rename to _init */
		while ((p = strstr(secstrings + sechdrs[i].sh_name, ".init")))
			p[0] = '_';

		if (sechdrs[i].sh_type == SHT_SYMTAB)
			dedotify((void *)hdr + sechdrs[i].sh_offset,
				 sechdrs[i].sh_size / sizeof(Elf64_Sym),
				 (void *)hdr
				 + sechdrs[sechdrs[i].sh_link].sh_offset);
	}

	if (!me->arch.elf_info.stubs_section) {
		pr_err("%s: doesn't contain .stubs.\n", me->name);
		return -ENOEXEC;
	}

	/* If we don't have a .toc, just use .stubs.  We need to set r2
	   to some reasonable value in case the module calls out to
	   other functions via a stub, or if a function pointer escapes
	   the module by some means.  */
	if (!me->arch.elf_info.toc_section)
		me->arch.elf_info.toc_section = me->arch.elf_info.stubs_section;

	/* Override the stubs size */
	sechdrs[me->arch.elf_info.stubs_section].sh_size = get_stubs_size(hdr, sechdrs);

	/* For the elf_util functions. */
	me->arch.elf_info.sechdrs = sechdrs;

	return 0;
}

/* Patch stub to reference function and correct r2 value. */
static inline int create_stub(const struct elf_info *elf_info,
			      struct ppc64_stub_entry *entry,
			      unsigned long addr, const char *obj_name)
{
	long reladdr;

	memcpy(entry->jump, ppc64_stub_insns, sizeof(ppc64_stub_insns));

	/* Stub uses address relative to r2. */
	reladdr = (unsigned long)entry - my_r2(elf_info);
	if (reladdr > 0x7FFFFFFF || reladdr < -(0x80000000L)) {
		pr_err("%s: Address %p of stub out of range of %p.\n",
		       obj_name, (void *)reladdr, (void *)my_r2);
		return 0;
	}
	pr_debug("Stub %p get data from reladdr %li\n", entry, reladdr);

	entry->jump[0] |= PPC_HA(reladdr);
	entry->jump[1] |= PPC_LO(reladdr);
	entry->funcdata = func_desc(addr);
	entry->magic = STUB_MAGIC;

	return 1;
}

/* Create stub to jump to function described in this OPD/ptr: we need the
   stub to set up the TOC ptr (r2) for the function. */
unsigned long stub_for_addr(const struct elf_info *elf_info, unsigned long addr,
			    const char *obj_name)
{
	struct elf_shdr *stubs_sec = &elf_info->sechdrs[elf_info->stubs_section];
	struct ppc64_stub_entry *stubs;
	unsigned int i, num_stubs;

	num_stubs = stubs_sec->sh_size / sizeof(*stubs);

	/* Find this stub, or if that fails, the next avail. entry */
	stubs = (void *) stubs_sec->sh_addr;
	for (i = 0; stub_func_addr(stubs[i].funcdata); i++) {
		BUG_ON(i >= num_stubs);

		if (stub_func_addr(stubs[i].funcdata) == func_addr(addr))
			return (unsigned long)&stubs[i];
	}

	if (!create_stub(elf_info, &stubs[i], addr, obj_name))
		return 0;

	return (unsigned long)&stubs[i];
}

#ifdef CC_USING_MPROFILE_KERNEL
static bool is_early_mcount_callsite(u32 *instruction)
{
	/*
	 * Check if this is one of the -mprofile-kernel sequences.
	 */
	if (instruction[-1] == PPC_INST_STD_LR &&
	    instruction[-2] == PPC_INST_MFLR)
		return true;

	if (instruction[-1] == PPC_INST_MFLR)
		return true;

	return false;
}

#else
/* without -mprofile-kernel, mcount calls are never early */
static bool is_early_mcount_callsite(u32 *instruction)
{
	return false;
}
#endif

/* We expect a noop next: if it is, replace it with instruction to
   restore r2. */
int restore_r2(u32 *instruction, const char *obj_name)
{
	if (is_early_mcount_callsite(instruction - 1))
		return 1;

	if (*instruction != PPC_INST_NOP) {
		pr_err("%s: Expect noop after relocate, got %08x\n",
		       obj_name, *instruction);
		return 0;
	}
	/* ld r2,R2_STACK_OFFSET(r1) */
	*instruction = PPC_INST_LD_TOC;
	return 1;
}

/*
 * When this function is called, the module is already at its final location in
 * memory, so Elf64_Shdr.sh_addr can be used for accessing the section
 * contents as well as the base address for relocations.
 */
int apply_relocate_add(Elf64_Shdr *sechdrs,
		       const char *strtab,
		       unsigned int symindex,
		       unsigned int relsec,
		       struct module *me)
{
	Elf64_Shdr *rel_section = &sechdrs[relsec];
	void *syms_base = (void *) sechdrs[symindex].sh_addr;
	Elf64_Addr addr_base = sechdrs[rel_section->sh_info].sh_addr;
	const Elf64_Rela *rela = (const Elf64_Rela *) rel_section->sh_addr;
	unsigned int num_rela = rel_section->sh_size / sizeof(Elf64_Rela);
	Elf64_Sym *sym;

	pr_debug("Applying ADD relocate section %u to %u\n", relsec,
		 rel_section->sh_info);

	/* First time we're called, we can fix up .TOC. */
	if (!me->arch.toc_fixed) {
		sym = find_dot_toc(sechdrs, strtab, symindex);
		/* It's theoretically possible that a module doesn't want a
		 * .TOC. so don't fail it just for that. */
		if (sym)
			sym->st_value = my_r2(&me->arch.elf_info);
		me->arch.toc_fixed = true;
	}

	return elf64_apply_relocate_add(&me->arch.elf_info, strtab, rela,
					num_rela, syms_base, (void *) addr_base,
					addr_base, me->name);
}

#ifdef CONFIG_DYNAMIC_FTRACE

#ifdef CC_USING_MPROFILE_KERNEL

#define PACATOC offsetof(struct paca_struct, kernel_toc)

/*
 * For mprofile-kernel we use a special stub for ftrace_caller() because we
 * can't rely on r2 containing this module's TOC when we enter the stub.
 *
 * That can happen if the function calling us didn't need to use the toc. In
 * that case it won't have setup r2, and the r2 value will be either the
 * kernel's toc, or possibly another modules toc.
 *
 * To deal with that this stub uses the kernel toc, which is always accessible
 * via the paca (in r13). The target (ftrace_caller()) is responsible for
 * saving and restoring the toc before returning.
 */
static unsigned long create_ftrace_stub(const Elf64_Shdr *sechdrs, struct module *me)
{
	struct ppc64_stub_entry *entry;
	unsigned int i, num_stubs;
	static u32 stub_insns[] = {
		0xe98d0000 | PACATOC, 	/* ld      r12,PACATOC(r13)	*/
		0x3d8c0000,		/* addis   r12,r12,<high>	*/
		0x398c0000, 		/* addi    r12,r12,<low>	*/
		0x7d8903a6, 		/* mtctr   r12			*/
		0x4e800420, 		/* bctr				*/
	};
	long reladdr;

	num_stubs = sechdrs[me->arch.elf_info.stubs_section].sh_size / sizeof(*entry);

	/* Find the next available stub entry */
	entry = (void *)sechdrs[me->arch.elf_info.stubs_section].sh_addr;
	for (i = 0; i < num_stubs && stub_func_addr(entry->funcdata); i++, entry++);

	if (i >= num_stubs) {
		pr_err("%s: Unable to find a free slot for ftrace stub.\n", me->name);
		return 0;
	}

	memcpy(entry->jump, stub_insns, sizeof(stub_insns));

	/* Stub uses address relative to kernel toc (from the paca) */
	reladdr = (unsigned long)ftrace_caller - kernel_toc_addr();
	if (reladdr > 0x7FFFFFFF || reladdr < -(0x80000000L)) {
		pr_err("%s: Address of ftrace_caller out of range of kernel_toc.\n", me->name);
		return 0;
	}

	entry->jump[1] |= PPC_HA(reladdr);
	entry->jump[2] |= PPC_LO(reladdr);

	/* Eventhough we don't use funcdata in the stub, it's needed elsewhere. */
	entry->funcdata = func_desc((unsigned long)ftrace_caller);
	entry->magic = STUB_MAGIC;

	return (unsigned long)entry;
}
#else
static unsigned long create_ftrace_stub(const Elf64_Shdr *sechdrs, struct module *me)
{
	return stub_for_addr(&me->arch.elf_info, (unsigned long)ftrace_caller,
			     me->name);
}
#endif

int module_finalize_ftrace(struct module *mod, const Elf_Shdr *sechdrs)
{
	mod->arch.toc = my_r2(&mod->arch.elf_info);
	mod->arch.tramp = create_ftrace_stub(sechdrs, mod);

	if (!mod->arch.tramp)
		return -ENOENT;

	return 0;
}
#endif
