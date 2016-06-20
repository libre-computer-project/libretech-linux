/*
 * include/asm-arm/arch-oxnas/entry-macro.S
 *
 * Low-level IRQ helper macros for Integrator platforms
 *
 * This file is licensed under  the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include <asm/arch/irqs.h>
#include <asm/plat-oxnas/hardware.h>

		.macro  disable_fiq
		.endm

		.macro  get_irqnr_preamble, base, tmp
		.endm

		.macro  arch_ret_to_user, tmp1, tmp2
		.endm

		.macro get_irqnr_and_base, irqnr, irqstat, base, tmp
		ldr \irqstat, =RPS_IRQ_STATUS
		ldr \irqstat, [\irqstat, #0]

		mov \irqnr, #0
1001:
		tst \irqstat, #1
		bne 1002f
		add \irqnr, \irqnr, #1
		mov \irqstat, \irqstat, lsr #1
		cmp \irqnr, #NR_IRQS
		bcc 1001b

1002:
		.endm

		.macro  irq_prio_table
		.endm

