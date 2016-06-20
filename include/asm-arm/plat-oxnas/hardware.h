/* linux/include/asm-arm/arch-oxnas/hardware.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#include <asm/page.h>
#include <asm/memory.h>
#include <asm/sizes.h>
#include <asm/arch/vmalloc.h>
#include <asm/arch/timex.h>

/* Required for MPCore */
#define IO_ADDRESS(x)              (x)
#define __io_address(n)            ((void __iomem *)IO_ADDRESS(n))

// The base of virtual address mappings for hardware cores starts directly after
// the end of the vmalloc mapping region
#define OXNAS_HW_PA_TO_VA(x) (VMALLOC_END + (x))

// Virtual address mapping of hardware cores
#define APB_BRIDGE_A_BASE    OXNAS_HW_PA_TO_VA(0)
#define STATIC_CONTROL_BASE  OXNAS_HW_PA_TO_VA(0x1000000)
#define CORE_MODULE_BASE     OXNAS_HW_PA_TO_VA(0x2000000)
#define EXTERNAL_UART_BASE   OXNAS_HW_PA_TO_VA(0x3000000)
#define EXTERNAL_UART_2_BASE OXNAS_HW_PA_TO_VA(0x3800000)

/* 16 Mbyte address range on AMBA bus */
#define APB_BRIDGE_BASE_MASK    0x00FFFFFF

#define APB_BRIDGE_B_BASE OXNAS_HW_PA_TO_VA(0x04000000)
#define SATA_DATA_BASE    OXNAS_HW_PA_TO_VA(0x05000000)
#define DPE_BASE          OXNAS_HW_PA_TO_VA(0x06000000)
#define USB_BASE          OXNAS_HW_PA_TO_VA(0x07000000)
#define MAC_BASE          OXNAS_HW_PA_TO_VA(0x08000000)
#define PCI_CSRS_BASE     OXNAS_HW_PA_TO_VA(0x0A000000)
#define STATIC_CS0_BASE   OXNAS_HW_PA_TO_VA(0x0B000000)
#define STATIC_CS1_BASE   OXNAS_HW_PA_TO_VA(0x0C000000)
#define STATIC_CS2_BASE   OXNAS_HW_PA_TO_VA(0x0D000000)
#define ROM_BASE          OXNAS_HW_PA_TO_VA(0x0E000000)
#define SDRAM_CTRL_BASE   OXNAS_HW_PA_TO_VA(0x0F000000)
#define LEON_IMAGE_BASE   OXNAS_HW_PA_TO_VA(0x10000000)
#define SRAM_BASE         OXNAS_HW_PA_TO_VA(0x11000000)
#define PERIPH_BASE       OXNAS_HW_PA_TO_VA(0x12000000)

#ifdef CONFIG_SUPPORT_LEON
#define LEON_IMAGE_SIZE			(CONFIG_LEON_PAGES * PAGE_SIZE)
#define LEON_IMAGE_BASE_PA		(((SRAM_END) + 1) - (LEON_IMAGE_SIZE))
#else // CONFIG_SUPPORT_LEON
#define LEON_IMAGE_SIZE		0
#define LEON_IMAGE_BASE_PA	0
#endif // CONFIG_SUPPORT_LEON

#if (LEON_IMAGE_BASE_PA >= SRAM_PA) && (LEON_IMAGE_BASE_PA <= SRAM_END)
#define LEON_IMAGE_IN_SRAM
#endif

#define DESCRIPTORS_SIZE        (CONFIG_DESCRIPTORS_PAGES * PAGE_SIZE)

#ifdef CONFIG_DESCRIPTORS_PAGES
#if (CONFIG_DESCRIPTORS_PAGES > CONFIG_SRAM_NUM_PAGES)
#error "Too many descriptor pages defined - greater than total SRAM pages"
#endif
#endif // CONFIG_DESCRIPTORS_PAGES

#define DESCRIPTORS_BASE_PA     (SRAM_PA)
#define DESCRIPTORS_END_PA      (SRAM_PA + DESCRIPTORS_SIZE)

#ifdef LEON_IMAGE_IN_SRAM
#if (DESCRIPTORS_END_PA > LEON_IMAGE_BASE_PA)
#error "Overlapping LEON and Descriptor areas in SRAM"
#endif
#endif

/*
 * Location of flags and vectors in SRAM for controlling the booting of the 
 * secondary ARM11 processors
 */
#define HOLDINGPEN_CPU          (SRAM_BASE + 0x1ffec)
#define HOLDINGPEN_LOCATION     (SRAM_BASE + 0x1fff0)

#define CORE_MODULE_BASE_PA    0x10000000

/*
* Periph. base is the MPCore processors' private memory area. It's in a 32-bit
* address space (where as the rest of the system is in 30-bit) so it will not
* conflict with SDRAM
*/
#define PERIPH_BASE_PA		   0x10000000

#define ROM_BASE_PA            0x40000000
#define USB_BASE_PA            0x40200000
#define MAC_BASE_PA            0x40400000
#define PCI_CSRS_BASE_PA       0x40600000
#define PCI_BASE_PA            0x40800000

#define STATIC_CS0_BASE_PA     0x41000000
#define STATIC_CS1_BASE_PA     0x41400000
#define STATIC_CS2_BASE_PA     0x41800000
#define STATIC_CONTROL_BASE_PA 0x41C00000

#define SATA_DATA_BASE_PA      0x42000000
#define DPE_BASE_PA            0x43000000
#define APB_BRIDGE_A_BASE_PA   0x44000000
#define APB_BRIDGE_B_BASE_PA   0x45000000

#define UART_1_BASE_PA         (APB_BRIDGE_A_BASE_PA + 0x200000)
#define UART_2_BASE_PA         (APB_BRIDGE_A_BASE_PA + 0x300000)
#define UART_3_BASE_PA         (APB_BRIDGE_A_BASE_PA + 0x900000)
#define UART_4_BASE_PA         (APB_BRIDGE_A_BASE_PA + 0xA00000)

#define GPIO_A_BASE             APB_BRIDGE_A_BASE
#define GPIO_B_BASE            (APB_BRIDGE_A_BASE + 0x100000)
#define UART_1_BASE            (APB_BRIDGE_A_BASE + 0x200000)
#define UART_2_BASE            (APB_BRIDGE_A_BASE + 0x300000)
#define UART_3_BASE            (APB_BRIDGE_A_BASE + 0x900000)
#define UART_4_BASE            (APB_BRIDGE_A_BASE + 0xA00000)
#define I2C_BASE               (APB_BRIDGE_A_BASE + 0x400000)
#define I2S_BASE               (APB_BRIDGE_A_BASE + 0x500000)
#define FAN_MON_BASE           (APB_BRIDGE_A_BASE + 0x600000)
#define PWM_BASE               (APB_BRIDGE_A_BASE + 0x700000)
#define IRRX_BASE              (APB_BRIDGE_A_BASE + 0x800000)

#define SYS_CONTROL_BASE        APB_BRIDGE_B_BASE
#define CLOCK_CONTROL_BASE     (APB_BRIDGE_B_BASE + 0x100000)
#define RTC_BASE               (APB_BRIDGE_B_BASE + 0x200000)
#define RPS_BASE               (APB_BRIDGE_B_BASE + 0x300000)
#define COPRO_RPS_BASE         (APB_BRIDGE_B_BASE + 0x400000)
#define AHB_MON_BASE           (APB_BRIDGE_B_BASE + 0x500000)
#define DMA_BASE               (APB_BRIDGE_B_BASE + 0x600000)
#define DPE_REGS_BASE          (APB_BRIDGE_B_BASE + 0x700000)
#define IBW_REGS_BASE          (APB_BRIDGE_B_BASE + 0x780000)
#define DDR_REGS_BASE          (APB_BRIDGE_B_BASE + 0x800000)

#ifdef CONFIG_OXNAS_VERSION_0X800
#define SATA0_REGS_BASE        (APB_BRIDGE_B_BASE + 0x900000)
#define SATA0_LINK_REGS_BASE   (APB_BRIDGE_B_BASE + 0x940000)
#define SATA1_REGS_BASE        (APB_BRIDGE_B_BASE + 0x980000)
#define SATA1_LINK_REGS_BASE   (APB_BRIDGE_B_BASE + 0x9C0000)
#elif defined(CONFIG_OXNAS_VERSION_0X810)
#define SATA_REG_BASE          (APB_BRIDGE_B_BASE + 0x900000)
#define SATA0_REGS_BASE        (SATA_REG_BASE)
#define SATA1_REGS_BASE        (SATA_REG_BASE + 0x10000)
#define SATACORE_REGS_BASE     (SATA_REG_BASE + 0xe0000)
#define SATARAID_REGS_BASE     (SATA_REG_BASE + 0xf0000)
#elif defined(CONFIG_OXNAS_VERSION_0X820)
#define SATA_REG_BASE          (APB_BRIDGE_B_BASE + 0x900000)
#define SATA0_REGS_BASE        (SATA_REG_BASE)
#define SATA1_REGS_BASE        (SATA_REG_BASE + 0x10000)
#define SATACORE_REGS_BASE     (SATA_REG_BASE + 0xe0000)
#define SATARAID_REGS_BASE     (SATA_REG_BASE + 0xf0000)
#endif // CONFIG_OXNAS_VERSION_0X8xx

#define DMA_CHECKSUM_BASE      (APB_BRIDGE_B_BASE + 0xA00000)
#define COPRO_REGS_BASE        (APB_BRIDGE_B_BASE + 0xB00000)
#define DMA_SG_BASE            (APB_BRIDGE_B_BASE + 0xC00000)

/* Interrupt Controller registers */
#define RPS_IC_BASE         RPS_BASE
#define RPS_IRQ_STATUS     (RPS_IC_BASE)
#define RPS_IRQ_RAW_STATUS (RPS_IC_BASE + 0x04)
#define RPS_IRQ_ENABLE     (RPS_IC_BASE + 0x08)
#define RPS_IRQ_DISABLE    (RPS_IC_BASE + 0x0C)
#define RPS_IRQ_SOFT       (RPS_IC_BASE + 0x10)

/* FIQ registers */
#define RPS_FIQ_BASE       (RPS_IC_BASE + 0x100)
#define RPS_FIQ_STATUS     (RPS_FIQ_BASE + 0x00)
#define RPS_FIQ_RAW_STATUS (RPS_FIQ_BASE + 0x04)
#define RPS_FIQ_ENABLE     (RPS_FIQ_BASE + 0x08)
#define RPS_FIQ_DISABLE    (RPS_FIQ_BASE + 0x0C)

/* Timer registers */
#define RPS_TIMER_BASE  (RPS_BASE + 0x200)
#define RPS_TIMER1_BASE (RPS_TIMER_BASE)
#define RPS_TIMER2_BASE (RPS_TIMER_BASE + 0x20)

#define TIMER1_LOAD    (RPS_TIMER1_BASE + 0x0)
#define TIMER1_VALUE   (RPS_TIMER1_BASE + 0x4)
#define TIMER1_CONTROL (RPS_TIMER1_BASE + 0x8)
#define TIMER1_CLEAR   (RPS_TIMER1_BASE + 0xc)

#define TIMER2_LOAD    (RPS_TIMER2_BASE + 0x0)
#define TIMER2_VALUE   (RPS_TIMER2_BASE + 0x4)
#define TIMER2_CONTROL (RPS_TIMER2_BASE + 0x8)
#define TIMER2_CLEAR   (RPS_TIMER2_BASE + 0xc)

#define TIMER_MODE_BIT          6
#define TIMER_MODE_FREE_RUNNING 0
#define TIMER_MODE_PERIODIC     1
#define TIMER_ENABLE_BIT        7
#define TIMER_ENABLE_DISABLE    0
#define TIMER_ENABLE_ENABLE     1

/* System clock frequencies */
#ifdef CONFIG_ARCH_OXNAS_FPGA
/* FPGA CPU clock is entirely independent of rest of SoC */
#define NOMINAL_ARMCLK (CONFIG_OXNAS_CORE_CLK)
#else // CONFIG_ARCH_OXNAS_FPGA
/* ASIC CPU clock is derived from SoC master clock */
#define NOMINAL_ARMCLK (CONFIG_NOMINAL_PLL400_FREQ / 2)
#endif // CONFIG_ARCH_OXNAS_FPGA

#define NOMINAL_SYSCLK (CONFIG_NOMINAL_PLL400_FREQ / 4)
#define NOMINAL_PCICLK 33000000

#ifdef CONFIG_ARCH_OXNAS_FPGA
/* FPGA has no PCI clock divider */
#define PCI_CLOCK_DIVIDER 1
#else // CONFIG_ARCH_OXNAS_FPGA
/* ASIC PCI divider divides CONFIG_NOMINAL_PLL400_FREQ by 2(n + 1) to get 33MHz */
#define PCI_CLOCK_DIVIDER (((CONFIG_NOMINAL_PLL400_FREQ) / (2 * NOMINAL_PCICLK)) - 1)
#endif //CONFIG_ARCH_OXNAS_FPGA

/* RPS timer setup */
#define TIMER_1_MODE           TIMER_MODE_PERIODIC
#define TIMER_2_PRESCALE_ENUM  TIMER_PRESCALE_256
#define TIMER_2_MODE           TIMER_MODE_FREE_RUNNING

/* Useful macros for dealing with sub timer-interrupt intervals - preserve
 * as much precision as possible without using floating point and minimising
 * runtime CPU requirement */
#define TIMER_1_LOAD_VALUE     ((CLOCK_TICK_RATE) / HZ)
#define TICKS_TO_US_SCALING    1024
#define TICKS_TO_US_FACTOR     (((2 * TICKS_TO_US_SCALING * 1000000) + CLOCK_TICK_RATE) / (2 * CLOCK_TICK_RATE))
#define TICKS_TO_US(ticks)     ((((ticks) * TICKS_TO_US_FACTOR * 2) + TICKS_TO_US_SCALING) / (2 * TICKS_TO_US_SCALING))

/* Remap and pause */
#define RPS_REMAP_AND_PAUSE    (RPS_BASE + 0x300)

/* GPIO */
#define GPIO_0               (0x00)
#define GPIO_1               (0x01)
#define GPIO_2               (0x02)
#define GPIO_3               (0x03)
#define GPIO_4               (0x04)
#define GPIO_5               (0x05)
#define GPIO_6               (0x06)
#define GPIO_7               (0x07)
#define GPIO_8               (0x08)
#define GPIO_9               (0x09)
#define GPIO_10              (0x0a)
#define GPIO_11              (0x0b)
#define GPIO_12              (0x0c)
#define GPIO_13              (0x0d)
#define GPIO_14              (0x0e)
#define GPIO_15              (0x0f)
#define GPIO_16              (0x10)
#define GPIO_17              (0x11)
#define GPIO_18              (0x12)
#define GPIO_19              (0x13)
#define GPIO_20              (0x14)
#define GPIO_21              (0x15)
#define GPIO_22              (0x16)
#define GPIO_23              (0x17)
#define GPIO_24              (0x18)
#define GPIO_25              (0x19)
#define GPIO_26              (0x1a)
#define GPIO_27              (0x1b)
#define GPIO_28              (0x1c)
#define GPIO_29              (0x1d)
#define GPIO_30              (0x1e)
#define GPIO_31              (0x1f)
#define GPIO_32              (0x00)
#define GPIO_33              (0x01)
#define GPIO_34              (0x02)

/* Symbols for functions mapped onto GPIO lines */
#ifdef CONFIG_ARCH_OXNAS_FPGA
#define PCI_GPIO_INTA_PLANAR    8
#define PCI_GPIO_INTA_MINIPCI   9
#else // CONFIG_ARCH_OXNAS_FPGA
#ifdef CONFIG_OXNAS_VERSION_0X800
#define PCI_GPIO_INTA_PLANAR    3
#define PCI_GPIO_INTA_MINIPCI   9
#elif defined(CONFIG_OXNAS_VERSION_0X810)
#define PCI_GPIO_INTA_PLANAR    3
#define PCI_GPIO_INTA_MINIPCI   9
#endif // CONFIG_OXNAS_VERSION_0X8xx
#endif // CONFIG_ARCH_OXNAS_FPGA

#define PCI_GPIO_CLKO_0 8
#define PCI_GPIO_CLKO_1 9
#define PCI_GPIO_CLKO_2 10
#define PCI_GPIO_CLKO_3 11
#define PCI_GPIO_CLKO_4 23

#define PCI_GNT_N0 0
#define PCI_GNT_N1 1
#define PCI_GNT_N2 2
#define PCI_GNT_N3 3
#define PCI_REQ_N0 4
#define PCI_REQ_N1 5
#define PCI_REQ_N2 6
#define PCI_REQ_N3 7

#define IBW_GPIO_DATA   (GPIO_33)

#define USBA_POWO_GPIO  23
#define USBA_OVERI_GPIO 24
#define USBB_POWO_GPIO  25
#define USBB_OVERI_GPIO 26
#define USBC_POWO_GPIO  27
#define USBC_OVERI_GPIO 28

/* RPS GPIO registers */
#define RPS_GPIO_BASE                         (RPS_BASE + 0x3C0)
#define RPS_GPIO_OUTPUT                       (RPS_BASE + 0x3C0)
#define RPS_GPIO_OUTPUT_ENABLE                (RPS_BASE + 0x3C4)
#define RPS_GPIO_INTERRUPT_ENABLE             (RPS_BASE + 0x3C8)
#define RPS_GPIO_INTERRUPT_EVENT              (RPS_BASE + 0x3CC)
#define RPS_GPIO_CHIPID                       (RPS_BASE + 0x3FC)

/* GPIO A registers */
#define GPIO_A_DATA                            GPIO_A_BASE
#define GPIO_A_OUTPUT_ENABLE                  (GPIO_A_BASE + 0x0004)
#define GPIO_A_INTERRUPT_ENABLE               (GPIO_A_BASE + 0x0008)
#define GPIO_A_INTERRUPT_EVENT                (GPIO_A_BASE + 0x000C)
#define GPIO_A_OUTPUT_VALUE                   (GPIO_A_BASE + 0x0010)
#define GPIO_A_OUTPUT_SET                     (GPIO_A_BASE + 0x0014)
#define GPIO_A_OUTPUT_CLEAR                   (GPIO_A_BASE + 0x0018)
#define GPIO_A_OUTPUT_ENABLE_SET              (GPIO_A_BASE + 0x001C)
#define GPIO_A_OUTPUT_ENABLE_CLEAR            (GPIO_A_BASE + 0x0020)
#define GPIO_A_INPUT_DEBOUNCE_ENABLE          (GPIO_A_BASE + 0x0024)
#define GPIO_A_RISING_EDGE_ACTIVE_HIGH_ENABLE (GPIO_A_BASE + 0x0028)
#define GPIO_A_FALLING_EDGE_ACTIVE_LOW_ENABLE (GPIO_A_BASE + 0x002C)
#define GPIO_A_RISING_EDGE_DETECT             (GPIO_A_BASE + 0x0030)
#define GPIO_A_FALLING_EDGE_DETECT            (GPIO_A_BASE + 0x0034)
#define GPIO_A_LEVEL_INTERRUPT_ENABLE         (GPIO_A_BASE + 0x0038)
#define GPIO_A_INTERRUPT_STATUS_REGISTER      (GPIO_A_BASE + 0x003C)

/* GPIO B registers */
#define GPIO_B_DATA                            GPIO_B_BASE
#define GPIO_B_OUTPUT_ENABLE                  (GPIO_B_BASE + 0x0004)
#define GPIO_B_INTERRUPT_ENABLE               (GPIO_B_BASE + 0x0008)
#define GPIO_B_INTERRUPT_EVENT                (GPIO_B_BASE + 0x000C)
#define GPIO_B_OUTPUT_VALUE                   (GPIO_B_BASE + 0x0010)
#define GPIO_B_OUTPUT_SET                     (GPIO_B_BASE + 0x0014)
#define GPIO_B_OUTPUT_CLEAR                   (GPIO_B_BASE + 0x0018)
#define GPIO_B_OUTPUT_ENABLE_SET              (GPIO_B_BASE + 0x001C)
#define GPIO_B_OUTPUT_ENABLE_CLEAR            (GPIO_B_BASE + 0x0020)
#define GPIO_B_INPUT_DEBOUNCE_ENABLE          (GPIO_B_BASE + 0x0024)
#define GPIO_B_RISING_EDGE_ACTIVE_HIGH_ENABLE (GPIO_B_BASE + 0x0028)
#define GPIO_B_FALLING_EDGE_ACTIVE_LOW_ENABLE (GPIO_B_BASE + 0x002C)
#define GPIO_B_RISING_EDGE_DETECT             (GPIO_B_BASE + 0x0030)
#define GPIO_B_FALLING_EDGE_DETECT            (GPIO_B_BASE + 0x0034)
#define GPIO_B_LEVEL_INTERRUPT_ENABLE         (GPIO_B_BASE + 0x0038)
#define GPIO_B_INTERRUPT_STATUS_REGISTER      (GPIO_B_BASE + 0x003C)

/* CoProcessor RPS GPIO registers */
#define COPRO_GPIO_A_BASE            (COPRO_RPS_BASE + 0x3C0)
#define COPRO_GPIO_A_DATA             COPRO_GPIO_A_BASE
#define COPRO_GPIO_A_OUTPUT_ENABLE   (COPRO_GPIO_A_BASE + 0x04)
#define COPRO_GPIO_A_INTERRUPT_MASK  (COPRO_GPIO_A_BASE + 0x08)
#define COPRO_GPIO_A_INTERRUPT_EVENT (COPRO_GPIO_A_BASE + 0x0C)

/* Static bus registers */
#define STATIC_CONTROL_VERSION (STATIC_CONTROL_BASE + 0x0)
#define STATIC_CONTROL_BANK0   (STATIC_CONTROL_BASE + 0x4)
#define STATIC_CONTROL_BANK1   (STATIC_CONTROL_BASE + 0x8)
#define STATIC_CONTROL_BANK2   (STATIC_CONTROL_BASE + 0xC)

/* System Control registers */
#define SYS_CTRL_USB11_CTRL             (SYS_CONTROL_BASE + 0x00)
#define SYS_CTRL_PCI_CTRL0              (SYS_CONTROL_BASE + 0x04)
#define SYS_CTRL_PCI_CTRL1              (SYS_CONTROL_BASE + 0x08)
#define SYS_CTRL_GPIO_PRIMSEL_CTRL_0    (SYS_CONTROL_BASE + 0x0C)
#define SYS_CTRL_GPIO_PRIMSEL_CTRL_1    (SYS_CONTROL_BASE + 0x10)
#define SYS_CTRL_GPIO_SECSEL_CTRL_0     (SYS_CONTROL_BASE + 0x14)
#define SYS_CTRL_GPIO_SECSEL_CTRL_1     (SYS_CONTROL_BASE + 0x18)
#define SYS_CTRL_GPIO_TERTSEL_CTRL_0    (SYS_CONTROL_BASE + 0x8C)
#define SYS_CTRL_GPIO_TERTSEL_CTRL_1    (SYS_CONTROL_BASE + 0x90)
#define SYS_CTRL_USB11_STAT             (SYS_CONTROL_BASE + 0x1c)
#define SYS_CTRL_PCI_STAT               (SYS_CONTROL_BASE + 0x20)
#define SYS_CTRL_CKEN_CTRL              (SYS_CONTROL_BASE + 0x24)
#define SYS_CTRL_RSTEN_CTRL             (SYS_CONTROL_BASE + 0x28)
#define SYS_CTRL_CKEN_SET_CTRL          (SYS_CONTROL_BASE + 0x2C)
#define SYS_CTRL_CKEN_CLR_CTRL          (SYS_CONTROL_BASE + 0x30)
#define SYS_CTRL_RSTEN_SET_CTRL         (SYS_CONTROL_BASE + 0x34)
#define SYS_CTRL_RSTEN_CLR_CTRL         (SYS_CONTROL_BASE + 0x38)
#define SYS_CTRL_USBHSMPH_CTRL          (SYS_CONTROL_BASE + 0x40)
#define SYS_CTRL_USBHSMPH_STAT          (SYS_CONTROL_BASE + 0x44)
#define SYS_CTRL_PLLSYS_CTRL            (SYS_CONTROL_BASE + 0x48)
#define SYS_CTRL_SEMA_STAT              (SYS_CONTROL_BASE + 0x4C)
#define SYS_CTRL_SEMA_SET_CTRL          (SYS_CONTROL_BASE + 0x50)
#define SYS_CTRL_SEMA_CLR_CTRL          (SYS_CONTROL_BASE + 0x54)
#define SYS_CTRL_SEMA_MASKA_CTRL        (SYS_CONTROL_BASE + 0x58)
#define SYS_CTRL_SEMA_MASKB_CTRL        (SYS_CONTROL_BASE + 0x5C)
#define SYS_CTRL_SEMA_MASKC_CTRL        (SYS_CONTROL_BASE + 0x60)
#define SYS_CTRL_CKCTRL_CTRL            (SYS_CONTROL_BASE + 0x64)
#define SYS_CTRL_COPRO_CTRL             (SYS_CONTROL_BASE + 0x68)
#define SYS_CTRL_PLLSYS_KEY_CTRL        (SYS_CONTROL_BASE + 0x6C)
#define SYS_CTRL_GMAC_CTRL              (SYS_CONTROL_BASE + 0x78)
#define SYS_CTRL_USBHSPHY_CTRL          (SYS_CONTROL_BASE + 0x84)
#define SYS_CTRL_UART_CTRL              (SYS_CONTROL_BASE + 0x94)
#define SYS_CTRL_GPIO_PWMSEL_CTRL_0    (SYS_CONTROL_BASE + 0x9C)
#define SYS_CTRL_GPIO_PWMSEL_CTRL_1    (SYS_CONTROL_BASE + 0xA0)
#define SYSCTRL_GPIO_MONSEL_CTRL_0 	(SYS_CONTROL_BASE + 0xA4)
#define SYSCTRL_GPIO_MONSEL_CTRL_1 	(SYS_CONTROL_BASE + 0xA8)
#define SYSCTRL_GPIO_PULLUP_CTRL_0 	(SYS_CONTROL_BASE + 0xAC)
#define SYSCTRL_GPIO_PULLUP_CTRL_1 	(SYS_CONTROL_BASE + 0xB0)
#define SYSCTRL_GPIO_ODRIVEHI_CTRL_0 	(SYS_CONTROL_BASE + 0xB4)
#define SYSCTRL_GPIO_ODRIVEHI_CTRL_1 	(SYS_CONTROL_BASE + 0xB8)
#define SYSCTRL_GPIO_ODRIVELO_CTRL_0 	(SYS_CONTROL_BASE + 0xBC)
#define SYSCTRL_GPIO_ODRIVELO_CTRL_1 	(SYS_CONTROL_BASE + 0xC0)


/* System control register field definitions */
#define SYSCTL_CB_CIS_OFFSET_0  SYS_CTRL_PCI_CTRL0

#define SYSCTL_PCI_CTRL1_PULL_UP_ENABLE_GPIO5   19
#define SYSCTL_PCI_CTRL1_PULL_UP_ENABLE_GPIO4   18
#define SYSCTL_PCI_CTRL1_PULL_UP_ENABLE_GPIO3   17
#define SYSCTL_PCI_CTRL1_PULL_UP_ENABLE_GPIO2   16
#define SYSCTL_PCI_CTRL1_PULL_UP_ENABLE_GPIO1   15
#define SYSCTL_PCI_CTRL1_PULL_UP_ENABLE_GPIO0   14
#define SYSCTL_PCI_CTRL1_ENPU                   13
#define SYSCTL_PCI_CTRL1_ENCB                   12
#define SYSCTL_PCI_CTRL1_SYSPCI_STATIC_REQ      11
#define SYSCTL_PCI_CTRL1_SS_HOST_E              10
#define SYSCTL_PCI_CTRL1_SYSPCI_PAKING_ENABLE   9
#define SYSCTL_PCI_CTRL1_SYSPCI_PAKING_MASTE    7
#define SYSCTL_PCI_CTRL1_SS_CADBUS_E            6
#define SYSCTL_PCI_CTRL1_SS_MINIPCI_            5
#define SYSCTL_PCI_CTRL1_SS_INT_MASK_0          4
#define SYSCTL_PCI_CTRL1_INT_STATUS_0           3
#define SYSCTL_PCI_CTRL1_APP_EQUIES_NOM_CLK     2
#define SYSCTL_PCI_CTRL1_APP_CBUS_INT_N         1
#define SYSCTL_PCI_CTRL1_APP_CSTSCHG_N          0

#define SYSCTL_PCI_STAT_SYSPCI_CLKCHG_EQ    3
#define SYSCTL_PCI_STAT_SYSPCI_STATIC_GNT   2
#define SYSCTL_PCI_STAT_INT_DISABLE_0       1
#define SYSCTL_PCI_STAT_CLK_CHANGED         0

#define SYS_CTRL_CKEN_COPRO_BIT  0
#define SYS_CTRL_CKEN_DMA_BIT    1
#define SYS_CTRL_CKEN_DPE_BIT    2
#define SYS_CTRL_CKEN_DDR_BIT    3
#define SYS_CTRL_CKEN_SATA_BIT   4
#define SYS_CTRL_CKEN_I2S_BIT    5
#define SYS_CTRL_CKEN_USBHS_BIT  6
#define SYS_CTRL_CKEN_MAC_BIT    7
#define SYS_CTRL_CKEN_PCI_BIT    8
#define SYS_CTRL_CKEN_STATIC_BIT 9

#define SYS_CTRL_RSTEN_ARM_BIT      0
#define SYS_CTRL_RSTEN_COPRO_BIT    1
#define SYS_CTRL_RSTEN_USBHS_BIT    4
#define SYS_CTRL_RSTEN_USBHSPHY_BIT 5
#define SYS_CTRL_RSTEN_MAC_BIT      6
#define SYS_CTRL_RSTEN_PCI_BIT      7
#define SYS_CTRL_RSTEN_DMA_BIT      8
#define SYS_CTRL_RSTEN_DPE_BIT      9
#define SYS_CTRL_RSTEN_DDR_BIT      10

#if defined(CONFIG_OXNAS_VERSION_0X800)
    #define SYS_CTRL_RSTEN_SATA_BIT      11
    #define SYS_CTRL_RSTEN_SATA_PHY_BIT  12
#elif defined(CONFIG_OXNAS_VERSION_0X810)
    #define SYS_CTRL_RSTEN_SATA_BIT      11
    #define SYS_CTRL_RSTEN_SATA_LINK_BIT 12
    #define SYS_CTRL_RSTEN_SATA_PHY_BIT  13
#elif defined(CONFIG_OXNAS_VERSION_0X820)
    #define SYS_CTRL_RSTEN_SATA_BIT      11
    #define SYS_CTRL_RSTEN_SATA_LINK_BIT 12
    #define SYS_CTRL_RSTEN_SATA_PHY_BIT  13
#endif

#define SYS_CTRL_RSTEN_STATIC_BIT   15
#define SYS_CTRL_RSTEN_GPIO_BIT     16
#define SYS_CTRL_RSTEN_UART1_BIT    17
#define SYS_CTRL_RSTEN_UART2_BIT    18
#define SYS_CTRL_RSTEN_MISC_BIT     19
#define SYS_CTRL_RSTEN_I2S_BIT      20
#define SYS_CTRL_RSTEN_AHB_MON_BIT  21
#define SYS_CTRL_RSTEN_UART3_BIT    22
#define SYS_CTRL_RSTEN_UART4_BIT    23
#define SYS_CTRL_RSTEN_SGDMA_BIT    24
#define SYS_CTRL_RSTEN_BUS_BIT      31

#define SYS_CTRL_USBHSMPH_IP_POL_A_BIT  0
#define SYS_CTRL_USBHSMPH_IP_POL_B_BIT  1
#define SYS_CTRL_USBHSMPH_IP_POL_C_BIT  2
#define SYS_CTRL_USBHSMPH_OP_POL_A_BIT  3
#define SYS_CTRL_USBHSMPH_OP_POL_B_BIT  4
#define SYS_CTRL_USBHSMPH_OP_POL_C_BIT  5

#define SYS_CTRL_GMAC_RGMII         2
#define SYS_CTRL_GMAC_SIMPLE_MAX    1
#define SYS_CTRL_GMAC_CKEN_GTX      0

#define SYS_CTRL_CKCTRL_CTRL_PCIDIV_BIT 0
#define SYS_CTRL_CKCTRL_CTRL_PCIDIV_NUM_BITS 4

#define SYS_CTRL_USBHSPHY_SUSPENDM_MANUAL_ENABLE    16
#define SYS_CTRL_USBHSPHY_SUSPENDM_MANUAL_STATE     15
#define SYS_CTRL_USBHSPHY_ATE_ESET                  14
#define SYS_CTRL_USBHSPHY_TEST_DIN                   6
#define SYS_CTRL_USBHSPHY_TEST_ADD                   2
#define SYS_CTRL_USBHSPHY_TEST_DOUT_SEL              1
#define SYS_CTRL_USBHSPHY_TEST_CLK                   0

#define SYS_CTRL_UART2_DEQ_EN       0
#define SYS_CTRL_UART3_DEQ_EN       1
#define SYS_CTRL_UART3_IQ_EN        2
#define SYS_CTRL_UART4_IQ_EN        3
#define SYS_CTRL_UART4_NOT_PCI_MODE 4

/* DDR core registers */
#define DDR_CFG_REG     (DDR_REGS_BASE + 0x00)
#define DDR_BLKEN_REG   (DDR_REGS_BASE + 0x04)
#define DDR_STAT_REG    (DDR_REGS_BASE + 0x08)
#define DDR_CMD_REG     (DDR_REGS_BASE + 0x0C)
#define DDR_AHB_REG     (DDR_REGS_BASE + 0x10)
#define DDR_DLL_REG     (DDR_REGS_BASE + 0x14)
#define DDR_MON_REG     (DDR_REGS_BASE + 0x18)
#define DDR_DIAG_REG    (DDR_REGS_BASE + 0x1C)
#define DDR_DIAG2_REG   (DDR_REGS_BASE + 0x20)
#define DDR_IOC_REG     (DDR_REGS_BASE + 0x24)
#define DDR_ARB_REG     (DDR_REGS_BASE + 0x28)
#define DDR_AHB2_REG    (DDR_REGS_BASE + 0x2C)
#define DDR_BUSY_REG    (DDR_REGS_BASE + 0x30)
#define DDR_TIMING0_REG (DDR_REGS_BASE + 0x34)
#define DDR_TIMING1_REG (DDR_REGS_BASE + 0x38)
#define DDR_TIMING2_REG (DDR_REGS_BASE + 0x3C)
#define DDR_AHB3_REG    (DDR_REGS_BASE + 0x40)
#define DDR_AHB4_REG    (DDR_REGS_BASE + 0x44)
#define DDR_PHY0_REG    (DDR_REGS_BASE + 0x48)
#define DDR_PHY1_REG    (DDR_REGS_BASE + 0x4C)
#define DDR_PHY2_REG    (DDR_REGS_BASE + 0x50)
#define DDR_PHY3_REG    (DDR_REGS_BASE + 0x54)

/* DDR core register field definitions */
#define DDR_BLKEN_CLIENTS_BIT       0
#define DDR_BLKEN_CLIENTS_NUM_BITS  16
#define DDR_BLKEN_DDR_BIT           31

#define DDR_AHB_FLUSH_RCACHE_BIT       0
#define DDR_AHB_FLUSH_RCACHE_NUM_BITS  16

#define DDR_AHB_FLUSH_RCACHE_ARMD_BIT  0
#define DDR_AHB_FLUSH_RCACHE_ARMI_BIT  1
#define DDR_AHB_FLUSH_RCACHE_COPRO_BIT 2
#define DDR_AHB_FLUSH_RCACHE_DMAA_BIT  3
#define DDR_AHB_FLUSH_RCACHE_DMAB_BIT  4
#define DDR_AHB_FLUSH_RCACHE_PCI_BIT   5
#define DDR_AHB_FLUSH_RCACHE_GMAC_BIT  6
#define DDR_AHB_FLUSH_RCACHE_USB_BIT   7

#define DDR_AHB_NO_RCACHE_BIT       16
#define DDR_AHB_NO_RCACHE_NUM_BITS  16

#define DDR_AHB_NO_RCACHE_ARMD_BIT  16
#define DDR_AHB_NO_RCACHE_ARMI_BIT  17
#define DDR_AHB_NO_RCACHE_COPRO_BIT 18
#define DDR_AHB_NO_RCACHE_DMAA_BIT  19
#define DDR_AHB_NO_RCACHE_DMAB_BIT  20
#define DDR_AHB_NO_RCACHE_PCI_BIT   21
#define DDR_AHB_NO_RCACHE_GMAC_BIT  22
#define DDR_AHB_NO_RCACHE_USB_BIT   23

#define DDR_MON_CLIENT_BIT          0
#define DDR_MON_ALL_BIT             4

#define DDR_DIAG_HOLDOFFS_BIT       20
#define DDR_DIAG_HOLDOFFS_NUM_BITS  12
#define DDR_DIAG_WRITES_BIT         10
#define DDR_DIAG_WRITES_NUM_BITS    10
#define DDR_DIAG_READS_BIT          0
#define DDR_DIAG_READS_NUM_BITS     10

#define DDR_DIAG2_DIRCHANGES_BIT        0
#define DDR_DIAG2_DIRCHANGES_NUM_BITS   10

#define DDR_ARB_DATDIR_NCH_BIT  0
#define DDR_ARB_DATDIR_EN_BIT   1
#define DDR_ARB_REQAGE_EN_BIT   2
#define DDR_ARB_LRUBANK_EN_BIT  3
#define DDR_ARB_MIDBUF_BIT      4

#define DDR_AHB2_IGNORE_HPROT_BIT       0
#define DDR_AHB2_IGNORE_HPROT_NUM_BITS  16

#define DDR_AHB2_IGNORE_HPROT_ARMD_BIT  0
#define DDR_AHB2_IGNORE_HPROT_ARMI_BIT  1
#define DDR_AHB2_IGNORE_HPROT_COPRO_BIT 2
#define DDR_AHB2_IGNORE_HPROT_DMAA_BIT  3
#define DDR_AHB2_IGNORE_HPROT_DMAB_BIT  4
#define DDR_AHB2_IGNORE_HPROT_PCI_BIT   5
#define DDR_AHB2_IGNORE_HPROT_GMAC_BIT  6
#define DDR_AHB2_IGNORE_HPROT_USB_BIT   7

#define DDR_AHB2_IGNORE_WRAP_BIT        16
#define DDR_AHB2_IGNORE_WRAP_NUM_BITS   16

#define DDR_AHB2_IGNORE_WRAP_ARMD_BIT  16
#define DDR_AHB2_IGNORE_WRAP_ARMI_BIT  17
#define DDR_AHB2_IGNORE_WRAP_COPRO_BIT 18
#define DDR_AHB2_IGNORE_WRAP_DMAA_BIT  19
#define DDR_AHB2_IGNORE_WRAP_DMAB_BIT  20
#define DDR_AHB2_IGNORE_WRAP_PCI_BIT   21
#define DDR_AHB2_IGNORE_WRAP_GMAC_BIT  22
#define DDR_AHB2_IGNORE_WRAP_US_BIT    23

#define DDR_AHB3_DIS_BURST_BIT          0
#define DDR_AHB3_DIS_BURST_NUM_BITS     16

#define DDR_AHB3_DIS_BURST_ARMD_BIT     0
#define DDR_AHB3_DIS_BURST_ARMI_BIT     1
#define DDR_AHB3_DIS_BURST_COPRO_BIT    2
#define DDR_AHB3_DIS_BURST_DMAA_BIT     3
#define DDR_AHB3_DIS_BURST_DMAB_BIT     4
#define DDR_AHB3_DIS_BURST_PCI_BIT      5
#define DDR_AHB3_DIS_BURST_GMAC_BIT     6
#define DDR_AHB3_DIS_BURST_USB_BIT      7

#define DDR_AHB3_DIS_NONCACHE_BIT       16
#define DDR_AHB3_DIS_NONCACHE_NUM_BITS  16

#define DDR_AHB3_DIS_NONCACHE_ARMD_BIT  16
#define DDR_AHB3_DIS_NONCACHE_ARMI_BIT  17
#define DDR_AHB3_DIS_NONCACHE_COPRO_BIT 18
#define DDR_AHB3_DIS_NONCACHE_DMAA_BIT  19
#define DDR_AHB3_DIS_NONCACHE_DMAB_BIT  20
#define DDR_AHB3_DIS_NONCACHE_PCI_BIT   21
#define DDR_AHB3_DIS_NONCACHE_GMAC_BIT  22
#define DDR_AHB3_DIS_NONCACHE_USB_BIT   23

#define DDR_AHB4_EN_TIMEOUT_BIT        0
#define DDR_AHB4_EN_TIMEOUT_NUM_BITS   16

#define DDR_AHB4_EN_TIMEOUT_ARMD_BIT   0
#define DDR_AHB4_EN_TIMEOUT_ARMI_BIT   1
#define DDR_AHB4_EN_TIMEOUT_COPRO_BIT  2
#define DDR_AHB4_EN_TIMEOUT_DMAA_BIT   3
#define DDR_AHB4_EN_TIMEOUT_DMAB_BIT   4
#define DDR_AHB4_EN_TIMEOUT_PCI_BIT    5
#define DDR_AHB4_EN_TIMEOUT_GMAC_BIT   6
#define DDR_AHB4_EN_TIMEOUT_USB_BIT    7

#define DDR_AHB4_EN_WRBEHIND_BIT       16
#define DDR_AHB4_EN_WRBEHIND_NUM_BITS  16

#define DDR_AHB4_EN_WRBEHIND_ARMD_BIT  16
#define DDR_AHB4_EN_WRBEHIND_ARMI_BIT  17
#define DDR_AHB4_EN_WRBEHIND_COPRO_BIT 18
#define DDR_AHB4_EN_WRBEHIND_DMAA_BIT  19
#define DDR_AHB4_EN_WRBEHIND_DMAB_BIT  20
#define DDR_AHB4_EN_WRBEHIND_PCI_BIT   21
#define DDR_AHB4_EN_WRBEHIND_GMAC_BIT  22
#define DDR_AHB4_EN_WRBEHIND_USB_BIT   23

/* AHB monitor base addresses */
#define AHB_MON_ARM_D (AHB_MON_BASE + 0x00000)
#define AHB_MON_ARM_I (AHB_MON_BASE + 0x10000)
#define AHB_MON_DMA_A (AHB_MON_BASE + 0x20000)
#define AHB_MON_DMA_B (AHB_MON_BASE + 0x30000)
#define AHB_MON_LEON  (AHB_MON_BASE + 0x40000)
#define AHB_MON_USB   (AHB_MON_BASE + 0x50000)
#define AHB_MON_MAC   (AHB_MON_BASE + 0x60000)
#define AHB_MON_PCI   (AHB_MON_BASE + 0x70000)

/* AHB write monitor registers */
#define AHB_MON_MODE_REG_OFFSET         0x00
#define AHB_MON_HWRITE_REG_OFFSET       0x04
#define AHB_MON_HADDR_LOW_REG_OFFSET    0x08
#define AHB_MON_HADDR_HIGH_REG_OFFSET   0x0C
#define AHB_MON_HBURST_REG_OFFSET       0x10
#define AHB_MON_HPROT_REG_OFFSET        0x14

/* AHB monitor write register field definitions */
#define AHB_MON_MODE_MODE_BIT           0
#define AHB_MON_MODE_MODE_NUM_BITS      2
#define AHB_MON_HWRITE_COUNT_BIT        0
#define AHB_MON_HWRITE_COUNT_NUM_BITS   2
#define AHB_MON_HBURST_MASK_BIT         0
#define AHB_MON_HBURST_MASK_NUM_BITS    3
#define AHB_MON_HBURST_MATCH_BIT        4
#define AHB_MON_HBURST_MATCH_NUM_BITS   3
#define AHB_MON_HPROT_MASK_BIT          0
#define AHB_MON_HPROT_MASK_NUM_BITS     4
#define AHB_MON_HPROT_MATCH_BIT         4
#define AHB_MON_HPROT_MATCH_NUM_BITS    4

#ifndef __ASSEMBLY__
typedef enum AHB_MON_MODES {
    AHB_MON_MODE_IDLE,
    AHB_MON_MODE_ACTIVE,
    AHB_MON_MODE_RESET
} AHB_MON_MODES_T;

typedef enum AHB_MON_HWRITE {
    AHB_MON_HWRITE_INACTIVE,
    AHB_MON_HWRITE_WRITES,
    AHB_MON_HWRITE_READS,
    AHB_MON_HWRITE_READS_AND_WRITES
} AHB_MON_HWRITE_T;

typedef enum AHB_MON_HBURST {
    AHB_MON_HBURST_SINGLE,
    AHB_MON_HBURST_INCR,
    AHB_MON_HBURST_WRAP4,
    AHB_MON_HBURST_INCR4,
    AHB_MON_HBURST_WRAP8,
    AHB_MON_HBURST_INCR8,
    AHB_MON_HBURST_WRAP16,
    AHB_MON_HBURST_INCR16
} AHB_MON_HBURST_T;
#endif // __ASSEMBLY__

/* AHB read monitor registers */
#define AHB_MON_CYCLES_REG_OFFSET       0x00
#define AHB_MON_TRANSFERS_REG_OFFSET    0x04
#define AHB_MON_WAITS_REG_OFFSET        0x08

#define STATIC_BUS1_CONTROL_VALUE   0x04010484  /*  200nS rd/wr cycles to allow DMAing to SMC91x on static bus */

/* PCI bus definitions */
#define pcibios_assign_all_busses() 1
#define PCIBIOS_MIN_IO              0x1000      /* used for memory alignememnt guesstimate */
#define PCIBIOS_MIN_MEM             0x00100000  /* used for memory alignememnt guesstimate */

/* PCI bus commands - see pci spec */
#define PCI_BUS_CMD_INTERRUPT_ACKNOWLEDGE       0x00  
#define PCI_BUS_CMD_SPECIAL_CYCLE               0x01  
#define PCI_BUS_CMD_IO_READ                     0x02  
#define PCI_BUS_CMD_IO_WRITE                    0x03  
/*#define PCI_BUS_RESERVED                      0x04  */  
/*#define PCI_BUS_RESERVED                      0x05  */  
#define PCI_BUS_CMD_MEMORY_READ                 0x06  
#define PCI_BUS_CMD_MEMORY_WRITE                0x07  
/*#define PCI_BUS_RESERVED                      0x08  */
/*#define PCI_BUS_RESERVED                      0x09  */
#define PCI_BUS_CMD_CONFIGURATION_READ          0x0a  
#define PCI_BUS_CMD_CONFIGURATION_WRITE         0x0b  
#define PCI_BUS_CMD_MEMORY_READ_MULTIPLE        0x0c  
#define PCI_BUS_CMD_DUAL_ADDRESS_CYCLE          0x0d  
#define PCI_BUS_CMD_MEMORY_READ_LINE            0x0e  
#define PCI_BUS_CMD_MEMORY_WRITE_AND_INVALIDATE 0x0f  

/* synopsis PCI core register set */
#define PCI_CORE_REG_START                     PCI_CSRS_BASE
#define PCI_CRP_CMD_AND_ADDR                   PCI_CORE_REG_START
#define PCI_CRP_WRITE_DATA                    (PCI_CRP_CMD_AND_ADDR       + 4)
#define PCI_CRP_READ_DATA                     (PCI_CRP_WRITE_DATA         + 4)
#define PCI_CONFIG_IO_CYCLE_ADDR              (PCI_CRP_READ_DATA          + 4)
#define PCI_CONFIG_IO_BYTE_CMD                (PCI_CONFIG_IO_CYCLE_ADDR   + 4)
#define PCI_CONFIG_IO_WRITE_DATA              (PCI_CONFIG_IO_BYTE_CMD     + 4)
#define PCI_CONFIG_IO_READ_DATA               (PCI_CONFIG_IO_WRITE_DATA   + 4)
#define PCI_ERROR_MSG                         (PCI_CONFIG_IO_READ_DATA    + 4)
#define PCI_TRANS_ERROR_START_ADDR            (PCI_ERROR_MSG              + 4)
#define PCI_AHB_ERROR_LSB                     (PCI_TRANS_ERROR_START_ADDR + 4)
#define PCI_AHB_ERROR_START_ADDR              (PCI_AHB_ERROR_LSB          + 4)
#define PCI_FLUSH_FIFO_ON_ERR                 (PCI_AHB_ERROR_START_ADDR   + 4)
#define PCI_TAR_ID                            (PCI_FLUSH_FIFO_ON_ERR      + 4)
#define PCI_MAS_ID_IN                         (PCI_TAR_ID                 + 4)
#define PCI_CORE_REG_END                      (PCI_MAS_ID_IN              + 4)

/* register bit offsets */
#define PCI_CRP_ADDRESS_START                   0
#define PCI_CRP_ADDRESS_END                     10
#define PCI_CRP_CMD_START                       16
#define PCI_CRP_CMD_END                         19
#define PCI_CRP_BYTE_ENABLES_START              20
#define PCI_CRP_BYTE_ENABLES_END                23

#define PCI_CONFIG_IO_CMD_START                 0
#define PCI_CONFIG_IO_CMD_END                   3
#define PCI_CONFIG_IO_BYTE_ENABLES_START        4
#define PCI_CONFIG_IO_BYTE_ENABLES_END          7

#define PCI_ERR_MESSAGE_START                   0
#define PCI_ERR_MESSAGE_END                     1

#define PCI_AHB_ERR_BIT                         0

#define PCI_FLUSH_FIFO_ON_ERR_BIT               0

#define PCI_TAR_ID_START                        0
#define PCI_TAR_ID_END                          2

#define PCI_MAS_ID_IN_START                     0
#define PCI_MAS_ID_IN_END                       2

/* PWM register definitions */
#define PWM_DATA_REGISTER_BASE (PWM_BASE)
#define PWM_CLOCK_REGISTER  (PWM_BASE+0X400)

/* MPCore specific registers */
#define OX820_ARM11MP_SCU_BASE       (PERIPH_BASE | 0x0000)
#define OX820_GIC_CPU_BASE_ADDR      (PERIPH_BASE | 0x0100)
#define OX820_GIC_CPUN_BASE_ADDR(n)  (PERIPH_BASE + 0x0200 + ((n) * 0x100))
#define OX820_TWD_BASE               (PERIPH_BASE | 0x0600)
#define OX820_TWD_CPU0_BASE          (PERIPH_BASE | 0x0700)
#define OX820_TWD_CPU1_BASE          (PERIPH_BASE | 0x0800)
#define OX820_TWD_CPU2_BASE          (PERIPH_BASE | 0x0900)
#define OX820_TWD_CPU3_BASE          (PERIPH_BASE | 0x0A00)
#define OX820_GIC_DIST_BASE_ADDR     (PERIPH_BASE | 0x1000)

#define OX820_ARM11MP_TWD_SIZE 0x100

#endif // __ASM_ARCH_HARDWARE_H
