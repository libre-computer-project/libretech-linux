/*
 *  linux/arch/arm/mach-oxnas/irq.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/init.h>
#include <linux/list.h>
#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>

static void OXNAS_mask_irq(unsigned int irq)
{
    *((volatile unsigned long*)(RPS_IRQ_DISABLE)) = (1UL << irq);
}

static void OXNAS_unmask_irq(unsigned int irq)
{
    *((volatile unsigned long*)RPS_IRQ_ENABLE) = (1UL << irq);
}

static struct irq_chip OXNAS_chip = {
	.name	= "OXNAS",
    .ack	= OXNAS_mask_irq,
    .mask	= OXNAS_mask_irq,
    .unmask = OXNAS_unmask_irq,
};

void __init oxnas_init_irq(void)
{
    unsigned irq;

    // Disable all IRQs
    *((volatile unsigned long*)(RPS_IRQ_DISABLE)) = ~0UL;

    // Disable FIQ
    *((volatile unsigned long*)(RPS_FIQ_DISABLE)) = ~0UL;

    //Disable GPIO interrupts.
    *((volatile unsigned long *) (GPIO_A_LEVEL_INTERRUPT_ENABLE)) = 0;
    *((volatile unsigned long *) (GPIO_B_LEVEL_INTERRUPT_ENABLE)) = 0;
    *((volatile unsigned long *) (GPIO_A_INTERRUPT_ENABLE))       = 0;
    *((volatile unsigned long *) (GPIO_B_INTERRUPT_ENABLE))       = 0;



    // Initialise IRQ tracking structures
    for (irq=0; irq < NR_IRQS; irq++)
    {
        set_irq_chip(irq, &OXNAS_chip);
        set_irq_handler(irq, handle_level_irq);
        set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);
    }
}

