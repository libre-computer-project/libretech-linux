/* linux/include/asm-arm/arch-oxnas/irqs.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_IRQS_H
#define __ASM_ARCH_IRQS_H

/* Definitions for the RPS core irq controller */
#define OX820_RPS_IRQ_START 64
#define NR_RPS_IRQS 32

#define RPS_FIQ_INTERRUPT             (OX820_RPS_IRQ_START +  0)
#define RPS_SOFTWARE_INTERRUPT        (OX820_RPS_IRQ_START +  1)
#define RPS_TIMER_1_INTERRUPT         (OX820_RPS_IRQ_START +  4)
#define RPS_TIMER_2_INTERRUPT         (OX820_RPS_IRQ_START +  5)
#define RPS_USB_FS_INTERRUPT          (OX820_RPS_IRQ_START +  7)
#define RPS_MAC_INTERRUPT             (OX820_RPS_IRQ_START +  8)
#define RPS_SEM_A_INTERRUPT           (OX820_RPS_IRQ_START + 10)
#define RPS_SEM_B_INTERRUPT           (OX820_RPS_IRQ_START + 11)
#define RPS_DMA_INTERRUPT_0           (OX820_RPS_IRQ_START + 13)
#define RPS_DMA_INTERRUPT_1           (OX820_RPS_IRQ_START + 14)
#define RPS_DMA_INTERRUPT_2           (OX820_RPS_IRQ_START + 15)
#define RPS_DMA_INTERRUPT_3           (OX820_RPS_IRQ_START + 16)
#define RPS_DPE_INTERRUPT             (OX820_RPS_IRQ_START + 17)
#define RPS_SATA_1_INTERRUPT          (OX820_RPS_IRQ_START + 18)
#define RPS_SATA_2_INTERRUPT          (OX820_RPS_IRQ_START + 19)
#define RPS_DMA_INTERRUPT_4           (OX820_RPS_IRQ_START + 20)
#define RPS_GPIO_1_INTERRUPT          (OX820_RPS_IRQ_START + 21)
#define RPS_GPIO_2_INTERRUPT          (OX820_RPS_IRQ_START + 22)
#define RPS_UART_1_INTERRUPT          (OX820_RPS_IRQ_START + 23)
#define RPS_UART_2_INTERRUPT          (OX820_RPS_IRQ_START + 24)
#define RPS_I2S_INTERRUPT             (OX820_RPS_IRQ_START + 25)
#define RPS_SATA_1_ERROR              (OX820_RPS_IRQ_START + 26)
#define RPS_SATA_2_ERROR              (OX820_RPS_IRQ_START + 27)
#define RPS_I2C_INTERRUPT             (OX820_RPS_IRQ_START + 28)
#define RPS_UART_3_INTERRUPT          (OX820_RPS_IRQ_START + 29)
#define RPS_UART_4_INTERRUPT          (OX820_RPS_IRQ_START + 30)

#define PCI_A_INTERRUPT           GPIO_1_INTERRUPT



#define OX820_GIC_IRQ_START  32
#define NR_GIC_IRQS (OX820_GIC_IRQ_START + 32)

/* The per CPU timer interrupt */
#define IRQ_LOCALTIMER            (29)


#define FIQ_INTERRUPT             (OX820_GIC_IRQ_START +  0)
#define IRQ_INTERRUPT             (OX820_GIC_IRQ_START +  1)


#define IRRX_INTERRUPT            (OX820_GIC_IRQ_START +  6)
#define USB_FS_INTERRUPT          (OX820_GIC_IRQ_START +  7)
#define MAC_INTERRUPT             (OX820_GIC_IRQ_START +  8)
#define PCI_INTERRUPT             (OX820_GIC_IRQ_START +  9)
#define SEM_A_INTERRUPT           (OX820_GIC_IRQ_START + 10)
#define SEM_B_INTERRUPT           (OX820_GIC_IRQ_START + 11)

#define DMA_INTERRUPT_0           (OX820_GIC_IRQ_START + 13)
#define DMA_INTERRUPT_1           (OX820_GIC_IRQ_START + 14)
#define DMA_INTERRUPT_2           (OX820_GIC_IRQ_START + 15)
#define DMA_INTERRUPT_3           (OX820_GIC_IRQ_START + 16)
#define DPE_INTERRUPT             (OX820_GIC_IRQ_START + 17)
#define SATA_1_INTERRUPT          (OX820_GIC_IRQ_START + 18)

#define DMA_INTERRUPT_4           (OX820_GIC_IRQ_START + 20)
#define GPIO_1_INTERRUPT          (OX820_GIC_IRQ_START + 21)
#define GPIO_2_INTERRUPT          (OX820_GIC_IRQ_START + 22)
#define UART_1_INTERRUPT          (OX820_GIC_IRQ_START + 23)
#define UART_2_INTERRUPT          (OX820_GIC_IRQ_START + 24)
#define SDCOMBO_INTERRUPT         (OX820_GIC_IRQ_START + 25)



#define UART_3_INTERRUPT          (OX820_GIC_IRQ_START + 29)
#define UART_4_INTERRUPT          (OX820_GIC_IRQ_START + 30)
#define PCI_A_INTERRUPT           GPIO_1_INTERRUPT


/* Definitions for the ARM11's GIC IRQ controller */ 
#define NR_IRQS (NR_GIC_IRQS + NR_RPS_IRQS) /* 96 RPS*/



#endif // __ASM_ARCH_IRQ_H
