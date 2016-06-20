/*
 *  linux/arch/arm/mach-realview/platsmp.c
 *
 *  Copyright (C) 2002 ARM Ltd.
 *  All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/smp.h>

#include <asm/cacheflush.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/mach-types.h>

#include <asm/arch/scu.h>

/*
 * control for which core is the next to come out of the secondary
 * boot "holding pen"
 */
volatile int pen_release = -1;

static unsigned int __init get_core_count(void)
{
	unsigned int ncores;
	void __iomem *scu_base = 0;

	scu_base = __io_address(OX820_ARM11MP_SCU_BASE);

	if (scu_base) {
		ncores = __raw_readl(scu_base + SCU_CONFIG);
		ncores = (ncores & 0x03) + 1;
	} else
		ncores = 1;

	return ncores;
}

/*
 * Setup the SCU
 */
static void scu_enable(void)
{
	u32 scu_ctrl;
	void __iomem *scu_base;

	scu_base = __io_address(OX820_ARM11MP_SCU_BASE);

	scu_ctrl = __raw_readl(scu_base + SCU_CTRL);
	scu_ctrl |= 1;
	__raw_writel(scu_ctrl, scu_base + SCU_CTRL);
}

static DEFINE_SPINLOCK(boot_lock);

void __cpuinit platform_secondary_init(unsigned int cpu)
{
	/*
	 * the primary core may have used a "cross call" soft interrupt
	 * to get this processor out of WFI in the BootMonitor - make
	 * sure that we are no longer being sent this soft interrupt
	 */
	smp_cross_call_done(cpumask_of_cpu(cpu));

	/*
	 * if any interrupts are already enabled for the primary
	 * core (e.g. timer irq), then they will not have been enabled
	 * for us: do so
	 */
	gic_cpu_init(0, __io_address(OX820_GIC_CPU_BASE_ADDR));

	/*
	 * let the primary processor know we're out of the
	 * pen, then head off into the C entry point
	 */
	pen_release = -1;
	smp_wmb();

	/*
	 * Synchronise with the boot thread.
	 */
	spin_lock(&boot_lock);
	spin_unlock(&boot_lock);
}

static void __init release_secondary_cpu(u32 cpu)
{
	extern void secondary_startup(void);
    
    /* enable gic interrupts on the secondary CPU */
    gic_legacy_mode(0, __io_address(OX820_GIC_CPUN_BASE_ADDR(cpu)) );
    
	/* Write the address that we want the cpu to start at. */
    *((volatile u32*)HOLDINGPEN_LOCATION) = virt_to_phys(secondary_startup);
    wmb();
    *((volatile u32*)HOLDINGPEN_CPU) = cpu;
	wmb();
    
    /* An interrupt may be required to wake the CPU */
	smp_cross_call( cpumask_of_cpu(cpu) );
}

int __cpuinit boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	unsigned long timeout;

	/*
	 * set synchronisation state between this boot processor
	 * and the secondary one
	 */
	spin_lock(&boot_lock);

	/*
	 * The secondary processor is waiting to be released from
	 * the holding pen - release it, then wait for it to flag
	 * that it has been released by resetting pen_release.
	 *
	 * Note that "pen_release" is the hardware CPU ID, whereas
	 * "cpu" is Linux's internal ID.
	 */
    pen_release = 0;
	flush_cache_all();
    release_secondary_cpu(cpu);
	timeout = jiffies + (1 * HZ);
	while (time_before(jiffies, timeout)) {
		smp_rmb();
		if (pen_release == -1)
			break;
		udelay(10);
	}
    
	/*
	 * now the secondary core is starting up let it run its
	 * calibrations, then wait for it to finish
	 */
	spin_unlock(&boot_lock);

	return (pen_release != -1) ? -ENOSYS : 0;
}

/*
 * Initialise the CPU possible map early - this describes the CPUs
 * which may be present or become present in the system.
 */
void __init smp_init_cpus(void)
{
	unsigned int i, ncores = get_core_count();

	for (i = 0; i < ncores; i++)
		cpu_set(i, cpu_possible_map);
}

void __init smp_prepare_cpus(unsigned int max_cpus)
{
	unsigned int ncores = get_core_count();
	unsigned int cpu = smp_processor_id();
	int i;

	/* sanity check */
	if (ncores == 0) {
		printk(KERN_ERR"Aparrantly there are no CPUs at all, pretending there "
            "is at least one.\n");

		ncores = 1;
	}

	if (ncores > NR_CPUS) {
		printk(KERN_WARNING"Found more CPUs (%d) than configured to use (%d)\n",
		    ncores, NR_CPUS);
		ncores = NR_CPUS;
	}

	smp_store_cpu_info(cpu);

	/*
	 * are we trying to boot more cores than exist?
	 */
	if (max_cpus > ncores)
		max_cpus = ncores;

#ifdef CONFIG_LOCAL_TIMERS
	/*
	 * Enable the local timer for primary CPU.
	 */
	local_timer_setup(cpu);
#endif

	/*
	 * Initialise the present map, which describes the set of CPUs
	 * actually populated at the present time.
	 */
	for (i = 0; i < max_cpus; i++)
		cpu_set(i, cpu_present_map);

	/*
	 * Initialise the SCU if there are more than one CPU and let
	 * them know where to start. 
	 */
	if (max_cpus > 1) {
		scu_enable();
	}
}
