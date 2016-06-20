/* linux/include/asm-arm/arch-oxnas/system.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H

#include <asm/hardware.h>
#include <asm/io.h>

extern void sata_power_off(void);
extern int oxnas_global_invert_leds;

#if defined(CONFIG_LEON_POWER_BUTTON_MONITOR) || defined(CONFIG_LEON_POWER_BUTTON_MONITOR_MODULE)
#include <asm/plat-oxnas/leon.h>
#include <asm/plat-oxnas/leon-power-button-prog.h>
#endif // CONFIG_LEON_POWER_BUTTON_MONITOR

static inline void arch_idle(void)
{
    /*
     * This should do all the clock switching
     * and wait for interrupt tricks
     */
    cpu_do_idle();
}

static void arch_poweroff(void)
{
#if defined(CONFIG_LEON_POWER_BUTTON_MONITOR) || defined(CONFIG_LEON_POWER_BUTTON_MONITOR_MODULE)
    // Load CoPro program and start it running
    init_copro(leon_srec, oxnas_global_invert_leds);
#endif // CONFIG_LEON_POWER_BUTTON_MONITOR

    // Turn of power to SATA disk if possible
    sata_power_off();
}

static void arch_reset(char mode)
{
    // Assert reset to cores as per power on defaults
    writel((1UL << SYS_CTRL_RSTEN_COPRO_BIT)    |
           (1UL << SYS_CTRL_RSTEN_USBHS_BIT)    |
           (1UL << SYS_CTRL_RSTEN_USBHSPHY_BIT) |
           (1UL << SYS_CTRL_RSTEN_MAC_BIT)      |
           (1UL << SYS_CTRL_RSTEN_PCI_BIT)      |
           (1UL << SYS_CTRL_RSTEN_DMA_BIT)      |
           (1UL << SYS_CTRL_RSTEN_DPE_BIT)      |
           (1UL << SYS_CTRL_RSTEN_SATA_BIT)     |
           (1UL << SYS_CTRL_RSTEN_SATA_PHY_BIT) |
           (1UL << SYS_CTRL_RSTEN_STATIC_BIT)   |
           (1UL << SYS_CTRL_RSTEN_UART1_BIT)    |
           (1UL << SYS_CTRL_RSTEN_UART2_BIT)    |
           (1UL << SYS_CTRL_RSTEN_MISC_BIT)     |
           (1UL << SYS_CTRL_RSTEN_I2S_BIT)      |
           (1UL << SYS_CTRL_RSTEN_AHB_MON_BIT)  |
           (1UL << SYS_CTRL_RSTEN_UART3_BIT)    |
           (1UL << SYS_CTRL_RSTEN_UART4_BIT)    |
           (1UL << SYS_CTRL_RSTEN_SGDMA_BIT), SYS_CTRL_RSTEN_SET_CTRL);

    // Release reset to cores as per power on defaults
    writel((1UL << SYS_CTRL_RSTEN_GPIO_BIT), SYS_CTRL_RSTEN_CLR_CTRL);

    // Disable clocks to cores as per power-on defaults
    writel((1UL << SYS_CTRL_CKEN_COPRO_BIT) |
           (1UL << SYS_CTRL_CKEN_DMA_BIT)   |
           (1UL << SYS_CTRL_CKEN_DPE_BIT)   |
           (1UL << SYS_CTRL_CKEN_SATA_BIT)  |
           (1UL << SYS_CTRL_CKEN_I2S_BIT)   |
           (1UL << SYS_CTRL_CKEN_USBHS_BIT) |
           (1UL << SYS_CTRL_CKEN_MAC_BIT)   |
           (1UL << SYS_CTRL_CKEN_STATIC_BIT), SYS_CTRL_CKEN_CLR_CTRL);

    // Enable clocks to cores as per power-on defaults
    writel((1UL << SYS_CTRL_CKEN_PCI_BIT), SYS_CTRL_CKEN_SET_CTRL);

    // Set sys-control pin mux'ing as per power-on defaults
    writel(0x800UL, SYS_CTRL_GPIO_PRIMSEL_CTRL_0);
    writel(0x0UL,   SYS_CTRL_GPIO_PRIMSEL_CTRL_1);
    writel(0x0UL,   SYS_CTRL_GPIO_SECSEL_CTRL_0);
    writel(0x0UL,   SYS_CTRL_GPIO_SECSEL_CTRL_1);
    writel(0x0UL,   SYS_CTRL_GPIO_TERTSEL_CTRL_0);
    writel(0x0UL,   SYS_CTRL_GPIO_TERTSEL_CTRL_1);

    // No need to save any state, as the ROM loader can determine whether reset
    // is due to power cycling or programatic action, just hit the (self-
    // clearing) CPU reset bit of the block reset register
    writel(1UL << SYS_CTRL_RSTEN_ARM_BIT, SYS_CTRL_RSTEN_SET_CTRL);
}

#endif // __ASM_ARCH_SYSTEM_H

