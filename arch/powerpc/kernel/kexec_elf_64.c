/*
 * Load ELF vmlinux file for the kexec_file_load syscall.
 *
 * Copyright (C) 2004  Adam Litke (agl@us.ibm.com)
 * Copyright (C) 2004  IBM Corp.
 * Copyright (C) 2005  R Sharada (sharada@in.ibm.com)
 * Copyright (C) 2006  Mohan Kumar M (mohan@in.ibm.com)
 * Copyright (C) 2016  IBM Corporation
 *
 * Based on kexec-tools' kexec-elf-exec.c and kexec-elf-ppc64.c.
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

#define pr_fmt(fmt)	"kexec_elf: " fmt

#include <linux/types.h>
#include <linux/slab.h>
#include <linux/elf.h>
#include <linux/kexec.h>
#include <linux/of_fdt.h>
#include <linux/libfdt.h>
#include <asm/elf_util.h>

#define PURGATORY_STACK_SIZE	(16 * 1024)

/**
 * build_elf_exec_info - read ELF executable and check that we can use it
 */
static int build_elf_exec_info(const char *buf, size_t len, struct elfhdr *ehdr,
			       struct elf_info *elf_info)
{
	int i;
	int ret;

	ret = elf_read_from_buffer(buf, len, ehdr, elf_info);
	if (ret)
		return ret;

	/* Big endian vmlinux has type ET_DYN. */
	if (ehdr->e_type != ET_EXEC && ehdr->e_type != ET_DYN) {
		pr_err("Not an ELF executable.\n");
		goto error;
	} else if (!elf_info->proghdrs) {
		pr_err("No ELF program header.\n");
		goto error;
	}

	for (i = 0; i < ehdr->e_phnum; i++) {
		/*
		 * Kexec does not support loading interpreters.
		 * In addition this check keeps us from attempting
		 * to kexec ordinay executables.
		 */
		if (elf_info->proghdrs[i].p_type == PT_INTERP) {
			pr_err("Requires an ELF interpreter.\n");
			goto error;
		}
	}

	return 0;
error:
	elf_free_info(elf_info);
	return -ENOEXEC;
}

static int elf64_probe(const char *buf, unsigned long len)
{
	struct elfhdr ehdr;
	struct elf_info elf_info;
	int ret;

	ret = build_elf_exec_info(buf, len, &ehdr, &elf_info);
	if (ret)
		return ret;

	elf_free_info(&elf_info);

	return elf_check_arch(&ehdr) ? 0 : -ENOEXEC;
}

/**
 * elf_exec_load - load ELF executable image
 * @lowest_load_addr:	On return, will be the address where the first PT_LOAD
 *			section will be loaded in memory.
 *
 * Return:
 * 0 on success, negative value on failure.
 */
static int elf_exec_load(struct kimage *image, struct elfhdr *ehdr,
			 struct elf_info *elf_info,
			 unsigned long *lowest_load_addr)
{
	unsigned long base = 0, lowest_addr = UINT_MAX;
	int ret;
	size_t i;
	struct kexec_buf kbuf = { .image = image, .buf_max = ppc64_rma_size,
				  .top_down = false };

	/* Read in the PT_LOAD segments. */
	for (i = 0; i < ehdr->e_phnum; i++) {
		unsigned long load_addr;
		size_t size;
		const struct elf_phdr *phdr;

		phdr = &elf_info->proghdrs[i];
		if (phdr->p_type != PT_LOAD)
			continue;

		size = phdr->p_filesz;
		if (size > phdr->p_memsz)
			size = phdr->p_memsz;

		kbuf.buffer = (void *) elf_info->buffer + phdr->p_offset;
		kbuf.bufsz = size;
		kbuf.memsz = phdr->p_memsz;
		kbuf.buf_align = phdr->p_align;
		kbuf.buf_min = phdr->p_paddr + base;
		ret = kexec_add_buffer(&kbuf);
		if (ret)
			goto out;
		load_addr = kbuf.mem;

		if (load_addr < lowest_addr)
			lowest_addr = load_addr;
	}

	/* Update entry point to reflect new load address. */
	ehdr->e_entry += base;

	*lowest_load_addr = lowest_addr;
	ret = 0;
 out:
	return ret;
}

static void *elf64_load(struct kimage *image, char *kernel_buf,
			unsigned long kernel_len, char *initrd,
			unsigned long initrd_len, char *cmdline,
			unsigned long cmdline_len)
{
	int i, ret;
	unsigned int fdt_size;
	unsigned long kernel_load_addr, purgatory_load_addr;
	unsigned long initrd_load_addr = 0, fdt_load_addr, stack_top;
	void *fdt;
	const void *slave_code;
	struct elfhdr ehdr;
	struct elf_info elf_info;
	struct fdt_reserve_entry *rsvmap;
	struct kexec_buf kbuf = { .image = image, .buf_min = 0,
				  .buf_max = ppc64_rma_size };

	ret = build_elf_exec_info(kernel_buf, kernel_len, &ehdr, &elf_info);
	if (ret)
		goto out;

	ret = elf_exec_load(image, &ehdr, &elf_info, &kernel_load_addr);
	if (ret)
		goto out;

	pr_debug("Loaded the kernel at 0x%lx\n", kernel_load_addr);

	ret = kexec_load_purgatory(image, 0, ppc64_rma_size, true,
				   &purgatory_load_addr);
	if (ret) {
		pr_err("Loading purgatory failed.\n");
		goto out;
	}

	pr_debug("Loaded purgatory at 0x%lx\n", purgatory_load_addr);

	if (initrd != NULL) {
		kbuf.buffer = initrd;
		kbuf.bufsz = kbuf.memsz = initrd_len;
		kbuf.buf_align = PAGE_SIZE;
		kbuf.top_down = false;
		ret = kexec_add_buffer(&kbuf);
		if (ret)
			goto out;
		initrd_load_addr = kbuf.mem;

		pr_debug("Loaded initrd at 0x%lx\n", initrd_load_addr);
	}

	fdt_size = fdt_totalsize(initial_boot_params) * 2;
	fdt = kmalloc(fdt_size, GFP_KERNEL);
	if (!fdt) {
		pr_err("Not enough memory for the device tree.\n");
		ret = -ENOMEM;
		goto out;
	}
	ret = fdt_open_into(initial_boot_params, fdt, fdt_size);
	if (ret < 0) {
		pr_err("Error setting up the new device tree.\n");
		ret = -EINVAL;
		goto out;
	}

	ret = setup_new_fdt(image, fdt, initrd_load_addr, initrd_len, cmdline);
	if (ret)
		goto out;

	/*
	 * Documentation/devicetree/booting-without-of.txt says we need to
	 * add a reservation entry for the device tree block, but
	 * early_init_fdt_reserve_self reserves the memory even if there's no
	 * such entry. We'll add a reservation entry anyway, to be safe and
	 * compliant.
	 *
	 * Use dummy values, we will correct them in a moment.
	 */
	ret = fdt_add_mem_rsv(fdt, 1, 1);
	if (ret) {
		pr_err("Error reserving device tree memory: %s\n",
		       fdt_strerror(ret));
		ret = -EINVAL;
		goto out;
	}
	fdt_pack(fdt);

	kbuf.buffer = fdt;
	kbuf.bufsz = kbuf.memsz = fdt_size;
	kbuf.buf_align = PAGE_SIZE;
	kbuf.top_down = true;
	ret = kexec_add_buffer(&kbuf);
	if (ret)
		goto out;
	fdt_load_addr = kbuf.mem;

	/*
	 * Fix fdt reservation, now that we now where it will be loaded
	 * and how big it is.
	 */
	rsvmap = fdt + fdt_off_mem_rsvmap(fdt);
	i = fdt_num_mem_rsv(fdt) - 1;
	rsvmap[i].address = cpu_to_fdt64(fdt_load_addr);
	rsvmap[i].size = cpu_to_fdt64(fdt_totalsize(fdt));

	pr_debug("Loaded device tree at 0x%lx\n", fdt_load_addr);

	kbuf.memsz = PURGATORY_STACK_SIZE;
	kbuf.buf_align = PAGE_SIZE;
	kbuf.top_down = true;
	ret = kexec_locate_mem_hole(&kbuf);
	if (ret) {
		pr_err("Couldn't find free memory for the purgatory stack.\n");
		ret = -ENOMEM;
		goto out;
	}
	stack_top = kbuf.mem + PURGATORY_STACK_SIZE - 1;
	pr_debug("Purgatory stack is at 0x%lx\n", stack_top);

	slave_code = elf_info.buffer + elf_info.proghdrs[0].p_offset;
	ret = setup_purgatory(image, slave_code, fdt, kernel_load_addr,
			      fdt_load_addr, stack_top,
			      find_debug_console(fdt));
	if (ret)
		pr_err("Error setting up the purgatory.\n");

out:
	elf_free_info(&elf_info);

	/* Make kimage_file_post_load_cleanup free the fdt buffer for us. */
	return ret ? ERR_PTR(ret) : fdt;
}

struct kexec_file_ops kexec_elf64_ops = {
	.probe = elf64_probe,
	.load = elf64_load,
};
