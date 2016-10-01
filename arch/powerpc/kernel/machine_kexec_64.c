/*
 * PPC64 code to handle Linux booting another kernel.
 *
 * Copyright (C) 2004-2005, IBM Corp.
 *
 * Created by: Milton D Miller II
 *
 * This source code is licensed under the GNU General Public License,
 * Version 2.  See the file COPYING for more details.
 */


#include <linux/kexec.h>
#include <linux/smp.h>
#include <linux/thread_info.h>
#include <linux/init_task.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/cpu.h>
#include <linux/hardirq.h>
#include <linux/memblock.h>
#include <linux/libfdt.h>

#include <asm/page.h>
#include <asm/current.h>
#include <asm/machdep.h>
#include <asm/cacheflush.h>
#include <asm/paca.h>
#include <asm/mmu.h>
#include <asm/sections.h>	/* _end */
#include <asm/prom.h>
#include <asm/smp.h>
#include <asm/hw_breakpoint.h>
#include <asm/asm-prototypes.h>
#include <asm/kexec_elf_64.h>

#define SLAVE_CODE_SIZE		256

#ifdef CONFIG_KEXEC_FILE
static struct kexec_file_ops *kexec_file_loaders[] = {
	&kexec_elf64_ops,
};
#endif

#ifdef CONFIG_PPC_BOOK3E
int default_machine_kexec_prepare(struct kimage *image)
{
	int i;
	/*
	 * Since we use the kernel fault handlers and paging code to
	 * handle the virtual mode, we must make sure no destination
	 * overlaps kernel static data or bss.
	 */
	for (i = 0; i < image->nr_segments; i++)
		if (image->segment[i].mem < __pa(_end))
			return -ETXTBSY;
	return 0;
}
#else
int default_machine_kexec_prepare(struct kimage *image)
{
	int i;
	unsigned long begin, end;	/* limits of segment */
	unsigned long low, high;	/* limits of blocked memory range */
	struct device_node *node;
	const unsigned long *basep;
	const unsigned int *sizep;

	if (!mmu_hash_ops.hpte_clear_all)
		return -ENOENT;

	/*
	 * Since we use the kernel fault handlers and paging code to
	 * handle the virtual mode, we must make sure no destination
	 * overlaps kernel static data or bss.
	 */
	for (i = 0; i < image->nr_segments; i++)
		if (image->segment[i].mem < __pa(_end))
			return -ETXTBSY;

	/*
	 * For non-LPAR, we absolutely can not overwrite the mmu hash
	 * table, since we are still using the bolted entries in it to
	 * do the copy.  Check that here.
	 *
	 * It is safe if the end is below the start of the blocked
	 * region (end <= low), or if the beginning is after the
	 * end of the blocked region (begin >= high).  Use the
	 * boolean identity !(a || b)  === (!a && !b).
	 */
#ifdef CONFIG_PPC_STD_MMU_64
	if (htab_address) {
		low = __pa(htab_address);
		high = low + htab_size_bytes;

		for (i = 0; i < image->nr_segments; i++) {
			begin = image->segment[i].mem;
			end = begin + image->segment[i].memsz;

			if ((begin < high) && (end > low))
				return -ETXTBSY;
		}
	}
#endif /* CONFIG_PPC_STD_MMU_64 */

	/* We also should not overwrite the tce tables */
	for_each_node_by_type(node, "pci") {
		basep = of_get_property(node, "linux,tce-base", NULL);
		sizep = of_get_property(node, "linux,tce-size", NULL);
		if (basep == NULL || sizep == NULL)
			continue;

		low = *basep;
		high = low + (*sizep);

		for (i = 0; i < image->nr_segments; i++) {
			begin = image->segment[i].mem;
			end = begin + image->segment[i].memsz;

			if ((begin < high) && (end > low))
				return -ETXTBSY;
		}
	}

	return 0;
}
#endif /* !CONFIG_PPC_BOOK3E */

static void copy_segments(unsigned long ind)
{
	unsigned long entry;
	unsigned long *ptr;
	void *dest;
	void *addr;

	/*
	 * We rely on kexec_load to create a lists that properly
	 * initializes these pointers before they are used.
	 * We will still crash if the list is wrong, but at least
	 * the compiler will be quiet.
	 */
	ptr = NULL;
	dest = NULL;

	for (entry = ind; !(entry & IND_DONE); entry = *ptr++) {
		addr = __va(entry & PAGE_MASK);

		switch (entry & IND_FLAGS) {
		case IND_DESTINATION:
			dest = addr;
			break;
		case IND_INDIRECTION:
			ptr = addr;
			break;
		case IND_SOURCE:
			copy_page(dest, addr);
			dest += PAGE_SIZE;
		}
	}
}

void kexec_copy_flush(struct kimage *image)
{
	long i, nr_segments = image->nr_segments;
	struct  kexec_segment ranges[KEXEC_SEGMENT_MAX];

	/* save the ranges on the stack to efficiently flush the icache */
	memcpy(ranges, image->segment, sizeof(ranges));

	/*
	 * After this call we may not use anything allocated in dynamic
	 * memory, including *image.
	 *
	 * Only globals and the stack are allowed.
	 */
	copy_segments(image->head);

	/*
	 * we need to clear the icache for all dest pages sometime,
	 * including ones that were in place on the original copy
	 */
	for (i = 0; i < nr_segments; i++)
		flush_icache_range((unsigned long)__va(ranges[i].mem),
			(unsigned long)__va(ranges[i].mem + ranges[i].memsz));
}

#ifdef CONFIG_SMP

static int kexec_all_irq_disabled = 0;

static void kexec_smp_down(void *arg)
{
	local_irq_disable();
	hard_irq_disable();

	mb(); /* make sure our irqs are disabled before we say they are */
	get_paca()->kexec_state = KEXEC_STATE_IRQS_OFF;
	while(kexec_all_irq_disabled == 0)
		cpu_relax();
	mb(); /* make sure all irqs are disabled before this */
	hw_breakpoint_disable();
	/*
	 * Now every CPU has IRQs off, we can clear out any pending
	 * IPIs and be sure that no more will come in after this.
	 */
	if (ppc_md.kexec_cpu_down)
		ppc_md.kexec_cpu_down(0, 1);

	kexec_smp_wait();
	/* NOTREACHED */
}

static void kexec_prepare_cpus_wait(int wait_state)
{
	int my_cpu, i, notified=-1;

	hw_breakpoint_disable();
	my_cpu = get_cpu();
	/* Make sure each CPU has at least made it to the state we need.
	 *
	 * FIXME: There is a (slim) chance of a problem if not all of the CPUs
	 * are correctly onlined.  If somehow we start a CPU on boot with RTAS
	 * start-cpu, but somehow that CPU doesn't write callin_cpu_map[] in
	 * time, the boot CPU will timeout.  If it does eventually execute
	 * stuff, the secondary will start up (paca[].cpu_start was written) and
	 * get into a peculiar state.  If the platform supports
	 * smp_ops->take_timebase(), the secondary CPU will probably be spinning
	 * in there.  If not (i.e. pseries), the secondary will continue on and
	 * try to online itself/idle/etc. If it survives that, we need to find
	 * these possible-but-not-online-but-should-be CPUs and chaperone them
	 * into kexec_smp_wait().
	 */
	for_each_online_cpu(i) {
		if (i == my_cpu)
			continue;

		while (paca[i].kexec_state < wait_state) {
			barrier();
			if (i != notified) {
				printk(KERN_INFO "kexec: waiting for cpu %d "
				       "(physical %d) to enter %i state\n",
				       i, paca[i].hw_cpu_id, wait_state);
				notified = i;
			}
		}
	}
	mb();
}

/*
 * We need to make sure each present CPU is online.  The next kernel will scan
 * the device tree and assume primary threads are online and query secondary
 * threads via RTAS to online them if required.  If we don't online primary
 * threads, they will be stuck.  However, we also online secondary threads as we
 * may be using 'cede offline'.  In this case RTAS doesn't see the secondary
 * threads as offline -- and again, these CPUs will be stuck.
 *
 * So, we online all CPUs that should be running, including secondary threads.
 */
static void wake_offline_cpus(void)
{
	int cpu = 0;

	for_each_present_cpu(cpu) {
		if (!cpu_online(cpu)) {
			printk(KERN_INFO "kexec: Waking offline cpu %d.\n",
			       cpu);
			WARN_ON(cpu_up(cpu));
		}
	}
}

static void kexec_prepare_cpus(void)
{
	wake_offline_cpus();
	smp_call_function(kexec_smp_down, NULL, /* wait */0);
	local_irq_disable();
	hard_irq_disable();

	mb(); /* make sure IRQs are disabled before we say they are */
	get_paca()->kexec_state = KEXEC_STATE_IRQS_OFF;

	kexec_prepare_cpus_wait(KEXEC_STATE_IRQS_OFF);
	/* we are sure every CPU has IRQs off at this point */
	kexec_all_irq_disabled = 1;

	/* after we tell the others to go down */
	if (ppc_md.kexec_cpu_down)
		ppc_md.kexec_cpu_down(0, 0);

	/*
	 * Before removing MMU mappings make sure all CPUs have entered real
	 * mode:
	 */
	kexec_prepare_cpus_wait(KEXEC_STATE_REAL_MODE);

	put_cpu();
}

#else /* ! SMP */

static void kexec_prepare_cpus(void)
{
	/*
	 * move the secondarys to us so that we can copy
	 * the new kernel 0-0x100 safely
	 *
	 * do this if kexec in setup.c ?
	 *
	 * We need to release the cpus if we are ever going from an
	 * UP to an SMP kernel.
	 */
	smp_release_cpus();
	if (ppc_md.kexec_cpu_down)
		ppc_md.kexec_cpu_down(0, 0);
	local_irq_disable();
	hard_irq_disable();
}

#endif /* SMP */

/*
 * kexec thread structure and stack.
 *
 * We need to make sure that this is 16384-byte aligned due to the
 * way process stacks are handled.  It also must be statically allocated
 * or allocated as part of the kimage, because everything else may be
 * overwritten when we copy the kexec image.  We piggyback on the
 * "init_task" linker section here to statically allocate a stack.
 *
 * We could use a smaller stack if we don't care about anything using
 * current, but that audit has not been performed.
 */
static union thread_union kexec_stack __init_task_data =
	{ };

/*
 * For similar reasons to the stack above, the kexecing CPU needs to be on a
 * static PACA; we switch to kexec_paca.
 */
struct paca_struct kexec_paca;

/* Our assembly helper, in misc_64.S */
extern void kexec_sequence(void *newstack, unsigned long start,
			   void *image, void *control,
			   void (*clear_all)(void)) __noreturn;

/* too late to fail here */
void default_machine_kexec(struct kimage *image)
{
	/* prepare control code if any */

	/*
        * If the kexec boot is the normal one, need to shutdown other cpus
        * into our wait loop and quiesce interrupts.
        * Otherwise, in the case of crashed mode (crashing_cpu >= 0),
        * stopping other CPUs and collecting their pt_regs is done before
        * using debugger IPI.
        */

	if (!kdump_in_progress())
		kexec_prepare_cpus();

	pr_debug("kexec: Starting switchover sequence.\n");

	/* switch to a staticly allocated stack.  Based on irq stack code.
	 * We setup preempt_count to avoid using VMX in memcpy.
	 * XXX: the task struct will likely be invalid once we do the copy!
	 */
	kexec_stack.thread_info.task = current_thread_info()->task;
	kexec_stack.thread_info.flags = 0;
	kexec_stack.thread_info.preempt_count = HARDIRQ_OFFSET;
	kexec_stack.thread_info.cpu = current_thread_info()->cpu;

	/* We need a static PACA, too; copy this CPU's PACA over and switch to
	 * it.  Also poison per_cpu_offset to catch anyone using non-static
	 * data.
	 */
	memcpy(&kexec_paca, get_paca(), sizeof(struct paca_struct));
	kexec_paca.data_offset = 0xedeaddeadeeeeeeeUL;
	paca = (struct paca_struct *)RELOC_HIDE(&kexec_paca, 0) -
		kexec_paca.paca_index;
	setup_paca(&kexec_paca);

	/* XXX: If anyone does 'dynamic lppacas' this will also need to be
	 * switched to a static version!
	 */

	/* Some things are best done in assembly.  Finding globals with
	 * a toc is easier in C, so pass in what we can.
	 */
	kexec_sequence(&kexec_stack, image->start, image,
			page_address(image->control_code_page),
#ifdef CONFIG_PPC_STD_MMU
			mmu_hash_ops.hpte_clear_all
#else
			NULL
#endif
	);
	/* NOTREACHED */
}

#ifdef CONFIG_PPC_STD_MMU_64
/* Values we need to export to the second kernel via the device tree. */
static unsigned long htab_base;
static unsigned long htab_size;

static struct property htab_base_prop = {
	.name = "linux,htab-base",
	.length = sizeof(unsigned long),
	.value = &htab_base,
};

static struct property htab_size_prop = {
	.name = "linux,htab-size",
	.length = sizeof(unsigned long),
	.value = &htab_size,
};

static int __init export_htab_values(void)
{
	struct device_node *node;

	/* On machines with no htab htab_address is NULL */
	if (!htab_address)
		return -ENODEV;

	node = of_find_node_by_path("/chosen");
	if (!node)
		return -ENODEV;

	/* remove any stale propertys so ours can be found */
	of_remove_property(node, of_find_property(node, htab_base_prop.name, NULL));
	of_remove_property(node, of_find_property(node, htab_size_prop.name, NULL));

	htab_base = cpu_to_be64(__pa(htab_address));
	of_add_property(node, &htab_base_prop);
	htab_size = cpu_to_be64(htab_size_bytes);
	of_add_property(node, &htab_size_prop);

	of_node_put(node);
	return 0;
}
late_initcall(export_htab_values);
#endif /* CONFIG_PPC_STD_MMU_64 */

#ifdef CONFIG_KEXEC_FILE
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
 * arch_kexec_walk_mem() - call func(data) for each unreserved memory block
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
 * arch_kexec_apply_relocations_add() - apply purgatory relocations
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
	/* Section containing the relocation entries. */
	Elf64_Shdr *rel_section = &sechdrs[relsec];
	const Elf64_Rela *rela = (const Elf64_Rela *) rel_section->sh_offset;
	unsigned int num_rela = rel_section->sh_size / sizeof(Elf64_Rela);
	/* Section to which relocations apply. */
	Elf64_Shdr *target_section = &sechdrs[rel_section->sh_info];
	/* Associated symbol table. */
	Elf64_Shdr *symtabsec = &sechdrs[rel_section->sh_link];
	void *syms_base = (void *) symtabsec->sh_offset;
	void *loc_base = (void *) target_section->sh_offset;
	Elf64_Addr addr_base = target_section->sh_addr;
	struct elf_info elf_info;
	const char *strtab;

	if (symtabsec->sh_link >= ehdr->e_shnum) {
		/* Invalid strtab section number */
		pr_err("Invalid string table section index %d\n",
		       symtabsec->sh_link);
		return -ENOEXEC;
	}
	/* String table for the associated symbol table. */
	strtab = (const char *) sechdrs[symtabsec->sh_link].sh_offset;

	elf_init_elf_info(ehdr, sechdrs, &elf_info);

	return elf64_apply_relocate_add(&elf_info, strtab, rela, num_rela,
					syms_base, loc_base, addr_base,
					true, true, "kexec purgatory");
}

/**
 * setup_purgatory() - setup the purgatory runtime variables
 * @image:		kexec image.
 * @slave_code:		Slave code for the purgatory.
 * @fdt:		Flattened device tree for the next kernel.
 * @kernel_load_addr:	Address where the kernel is loaded.
 * @fdt_load_addr:	Address where the flattened device tree is loaded.
 * @stack_top:		Address where the purgatory can place its stack.
 * @debug:		Can the purgatory print messages to the console?
 *
 * Return: 0 on success, or negative errno on error.
 */
int setup_purgatory(struct kimage *image, const void *slave_code,
		    const void *fdt, unsigned long kernel_load_addr,
		    unsigned long fdt_load_addr, unsigned long stack_top,
		    int debug)
{
	int ret, tree_node;
	const void *prop;
	unsigned long opal_base, opal_entry;
	uint64_t toc;
	unsigned int *slave_code_buf, master_entry;
	struct elf_info purg_info;

	slave_code_buf = kmalloc(SLAVE_CODE_SIZE, GFP_KERNEL);
	if (!slave_code_buf)
		return -ENOMEM;

	/* Get the slave code from the new kernel and put it in purgatory. */
	ret = kexec_purgatory_get_set_symbol(image, "purgatory_start",
					     slave_code_buf, SLAVE_CODE_SIZE,
					     true);
	if (ret) {
		kfree(slave_code_buf);
		return ret;
	}

	master_entry = slave_code_buf[0];
	memcpy(slave_code_buf, slave_code, SLAVE_CODE_SIZE);
	slave_code_buf[0] = master_entry;
	ret = kexec_purgatory_get_set_symbol(image, "purgatory_start",
					     slave_code_buf, SLAVE_CODE_SIZE,
					     false);
	kfree(slave_code_buf);

	ret = kexec_purgatory_get_set_symbol(image, "kernel", &kernel_load_addr,
					     sizeof(kernel_load_addr), false);
	if (ret)
		return ret;
	ret = kexec_purgatory_get_set_symbol(image, "dt_offset", &fdt_load_addr,
					     sizeof(fdt_load_addr), false);
	if (ret)
		return ret;

	tree_node = fdt_path_offset(fdt, "/ibm,opal");
	if (tree_node >= 0) {
		prop = fdt_getprop(fdt, tree_node, "opal-base-address", NULL);
		if (!prop) {
			pr_err("OPAL address not found in the device tree.\n");
			return -EINVAL;
		}
		opal_base = fdt64_to_cpu((const fdt64_t *) prop);

		prop = fdt_getprop(fdt, tree_node, "opal-entry-address", NULL);
		if (!prop) {
			pr_err("OPAL address not found in the device tree.\n");
			return -EINVAL;
		}
		opal_entry = fdt64_to_cpu((const fdt64_t *) prop);

		ret = kexec_purgatory_get_set_symbol(image, "opal_base",
						     &opal_base,
						     sizeof(opal_base), false);
		if (ret)
			return ret;
		ret = kexec_purgatory_get_set_symbol(image, "opal_entry",
						     &opal_entry,
						     sizeof(opal_entry), false);
		if (ret)
			return ret;
	}

	ret = kexec_purgatory_get_set_symbol(image, "stack", &stack_top,
					     sizeof(stack_top), false);
	if (ret)
		return ret;

	elf_init_elf_info(image->purgatory_info.ehdr,
			  image->purgatory_info.sechdrs, &purg_info);
	toc = my_r2(&purg_info);
	ret = kexec_purgatory_get_set_symbol(image, "my_toc", &toc, sizeof(toc),
					     false);
	if (ret)
		return ret;

	pr_debug("Purgatory TOC is at 0x%llx\n", toc);

	ret = kexec_purgatory_get_set_symbol(image, "debug", &debug,
					     sizeof(debug), false);
	if (ret)
		return ret;
	if (!debug)
		pr_debug("Disabling purgatory output.\n");

	return 0;
}

/*
 * setup_new_fdt() - modify /chosen and memory reservation for the next kernel
 * @fdt:
 * @initrd_load_addr:	Address where the next initrd will be loaded.
 * @initrd_len:		Size of the next initrd, or 0 if there will be none.
 * @cmdline:		Command line for the next kernel, or NULL if there will
 *			be none.
 *
 * Return: 0 on success, or negative errno on error.
 */
int setup_new_fdt(void *fdt, unsigned long initrd_load_addr,
		  unsigned long initrd_len, const char *cmdline)
{
	uint64_t oldfdt_addr;
	int i, ret, chosen_node;
	const void *prop;

	/* Remove memory reservation for the current device tree. */
	oldfdt_addr = __pa(initial_boot_params);
	for (i = 0; i < fdt_num_mem_rsv(fdt); i++) {
		uint64_t rsv_start, rsv_size;

		ret = fdt_get_mem_rsv(fdt, i, &rsv_start, &rsv_size);
		if (ret) {
			pr_err("Malformed device tree.\n");
			return -EINVAL;
		}

		if (rsv_start == oldfdt_addr &&
		    rsv_size == fdt_totalsize(initial_boot_params)) {
			ret = fdt_del_mem_rsv(fdt, i);
			if (ret) {
				pr_err("Error deleting fdt reservation.\n");
				return -EINVAL;
			}

			pr_debug("Removed old device tree reservation.\n");
			break;
		}
	}

	chosen_node = fdt_path_offset(fdt, "/chosen");
	if (chosen_node == -FDT_ERR_NOTFOUND) {
		chosen_node = fdt_add_subnode(fdt, fdt_path_offset(fdt, "/"),
					      "chosen");
		if (chosen_node < 0) {
			pr_err("Error creating /chosen.\n");
			return -EINVAL;
		}
	} else if (chosen_node < 0) {
		pr_err("Malformed device tree: error reading /chosen.\n");
		return -EINVAL;
	}

	/* Did we boot using an initrd? */
	prop = fdt_getprop(fdt, chosen_node, "linux,initrd-start", NULL);
	if (prop) {
		uint64_t tmp_start, tmp_end, tmp_size, tmp_sizepg;

		tmp_start = fdt64_to_cpu(*((const fdt64_t *) prop));

		prop = fdt_getprop(fdt, chosen_node, "linux,initrd-end", NULL);
		if (!prop) {
			pr_err("Malformed device tree.\n");
			return -EINVAL;
		}
		tmp_end = fdt64_to_cpu(*((const fdt64_t *) prop));

		/*
		 * kexec reserves exact initrd size, while firmware may
		 * reserve a multiple of PAGE_SIZE, so check for both.
		 */
		tmp_size = tmp_end - tmp_start;
		tmp_sizepg = round_up(tmp_size, PAGE_SIZE);

		/* Remove memory reservation for the current initrd. */
		for (i = 0; i < fdt_num_mem_rsv(fdt); i++) {
			uint64_t rsv_start, rsv_size;

			ret = fdt_get_mem_rsv(fdt, i, &rsv_start, &rsv_size);
			if (ret) {
				pr_err("Malformed device tree.\n");
				return -EINVAL;
			}

			if (rsv_start == tmp_start &&
			    (rsv_size == tmp_size || rsv_size == tmp_sizepg)) {
				ret = fdt_del_mem_rsv(fdt, i);
				if (ret) {
					pr_err("Error deleting fdt reservation.\n");
					return -EINVAL;
				}
				pr_debug("Removed old initrd reservation.\n");

				break;
			}
		}

		/* If there's no new initrd, delete the old initrd's info. */
		if (initrd_len == 0) {
			ret = fdt_delprop(fdt, chosen_node,
					  "linux,initrd-start");
			if (ret) {
				pr_err("Error deleting linux,initrd-start.\n");
				return -EINVAL;
			}

			ret = fdt_delprop(fdt, chosen_node, "linux,initrd-end");
			if (ret) {
				pr_err("Error deleting linux,initrd-end.\n");
				return -EINVAL;
			}
		}
	}

	if (initrd_len) {
		ret = fdt_setprop_u64(fdt, chosen_node,
				      "linux,initrd-start",
				      initrd_load_addr);
		if (ret < 0) {
			pr_err("Error setting up the new device tree.\n");
			return -EINVAL;
		}

		/* initrd-end is the first address after the initrd image. */
		ret = fdt_setprop_u64(fdt, chosen_node, "linux,initrd-end",
				      initrd_load_addr + initrd_len);
		if (ret < 0) {
			pr_err("Error setting up the new device tree.\n");
			return -EINVAL;
		}

		ret = fdt_add_mem_rsv(fdt, initrd_load_addr, initrd_len);
		if (ret) {
			pr_err("Error reserving initrd memory: %s\n",
			       fdt_strerror(ret));
			return -EINVAL;
		}
	}

	if (cmdline != NULL) {
		ret = fdt_setprop_string(fdt, chosen_node, "bootargs", cmdline);
		if (ret < 0) {
			pr_err("Error setting up the new device tree.\n");
			return -EINVAL;
		}
	} else {
		ret = fdt_delprop(fdt, chosen_node, "bootargs");
		if (ret && ret != -FDT_ERR_NOTFOUND) {
			pr_err("Error deleting bootargs.\n");
			return -EINVAL;
		}
	}

	ret = fdt_setprop(fdt, chosen_node, "linux,booted-from-kexec", NULL, 0);
	if (ret) {
		pr_err("Error setting up the new device tree.\n");
		return -EINVAL;
	}

	return 0;
}

/**
 * find_debug_console() - find out whether there is a console for the purgatory
 * @fdt:		Flattened device tree to search.
 */
bool find_debug_console(const void *fdt)
{
	int len;
	int console_node, chosen_node;
	const void *prop, *colon;

	chosen_node = fdt_path_offset(fdt, "/chosen");
	if (chosen_node < 0) {
		pr_err("Malformed device tree: /chosen not found.\n");
		return false;
	}

	prop = fdt_getprop(fdt, chosen_node, "stdout-path", &len);
	if (prop == NULL) {
		if (len == -FDT_ERR_NOTFOUND) {
			prop = fdt_getprop(fdt, chosen_node,
					   "linux,stdout-path", &len);
			if (prop == NULL) {
				pr_debug("Unable to find [linux,]stdout-path.\n");
				return false;
			}
		} else {
			pr_debug("Error finding console: %s\n",
				 fdt_strerror(len));
			return false;
		}
	}

	/*
	 * stdout-path can have a ':' separating the path from device-specific
	 * information, so we should only consider what's before it.
	 */
	colon = strchr(prop, ':');
	if (colon != NULL)
		len = colon - prop;
	else
		len -= 1;	/* Ignore the terminating NUL. */

	console_node = fdt_path_offset_namelen(fdt, prop, len);
	if (console_node < 0) {
		pr_debug("Error finding console: %s\n",
			 fdt_strerror(console_node));
		return false;
	}

	if (fdt_node_check_compatible(fdt, console_node, "hvterm1") == 0)
		return true;
	else if (fdt_node_check_compatible(fdt, console_node,
					   "hvterm-protocol") == 0)
		return true;

	return false;
}

#endif /* CONFIG_KEXEC_FILE */
