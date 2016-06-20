/* linux/include/asm-arm/arch-oxnas/irqs.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_IRQS_H
#define __ASM_ARCH_IRQS_H

#define FIQ_INTERRUPT              0
#define SOFTWARE_INTERRUPT         1
#define TIMER_1_INTERRUPT          4
#define TIMER_2_INTERRUPT          5
#define USB_FS_INTERRUPT           7
#define MAC_INTERRUPT              8
#define SEM_A_INTERRUPT           10
#define SEM_B_INTERRUPT           11
#define DMA_INTERRUPT_0           13
#define DMA_INTERRUPT_1           14
#define DMA_INTERRUPT_2           15
#define DMA_INTERRUPT_3           16
#define DPE_INTERRUPT             17
#define SATA_1_INTERRUPT          18
#define SATA_2_INTERRUPT          19
#define DMA_INTERRUPT_4           20
#define GPIO_1_INTERRUPT          21
#define GPIO_2_INTERRUPT          22
#define UART_1_INTERRUPT          23
#define UART_2_INTERRUPT          24
#define I2S_INTERRUPT             25
#define SATA_1_ERROR              26
#define SATA_2_ERROR              27
#define I2C_INTERRUPT             28
#define UART_3_INTERRUPT          29
#define UART_4_INTERRUPT          30

#define PCI_A_INTERRUPT           GPIO_1_INTERRUPT

#define NR_IRQS 32

#endif // __ASM_ARCH_IRQ_H
