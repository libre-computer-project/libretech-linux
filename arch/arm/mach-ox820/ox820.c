/*
 * linux/arch/arm/mach-oxnas/oxnas.c
 *
 * Copyright (C) 2005 Oxford Semiconductor Ltd
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
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/serial_8250.h>

#include <asm/sizes.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/arch/hardware.h>
#include <asm/arch/dma.h>
#include <asm/arch/rps_irq.h>
#include <asm/hardware/gic.h>

#ifdef CONFIG_DO_MEM_TEST
#include <linux/dma-mapping.h>
#include <asm/io.h>
#include <asm/arch/ahb_mon.h>
#endif // CONFIG_DO_MEM_TEST

#include <asm/io.h>

#ifdef CONFIG_LEON_START_EARLY
#include <asm/arch/leon.h>
#include <asm/arch/leon-early-prog.h>
#endif // CONFIG_LEON_START_EARLY

#ifdef CONFIG_OXNAS_PCI_RESET_GPIO
#if (CONFIG_OXNAS_PCI_RESET_GPIO < 32)
#define PCI_RESET_NUM               CONFIG_OXNAS_PCI_RESET_GPIO
#define PCI_RESET_PRISEL_REG        SYS_CTRL_GPIO_PRIMSEL_CTRL_0
#define PCI_RESET_SECSEL_REG        SYS_CTRL_GPIO_SECSEL_CTRL_0
#define PCI_RESET_TERSEL_REG        SYS_CTRL_GPIO_TERTSEL_CTRL_0
#define PCI_RESET_SET_OE_REG        GPIO_A_OUTPUT_ENABLE_SET
#define PCI_RESET_OUTPUT_SET_REG    GPIO_A_OUTPUT_SET
#define PCI_RESET_OUTPUT_CLR_REG    GPIO_A_OUTPUT_CLEAR
#else
#define PCI_RESET_NUM               ((CONFIG_OXNAS_PCI_RESET_GPIO) - 32)
#define PCI_RESET_PRISEL_REG        SYS_CTRL_GPIO_PRIMSEL_CTRL_1
#define PCI_RESET_SECSEL_REG        SYS_CTRL_GPIO_SECSEL_CTRL_1
#define PCI_RESET_TERSEL_REG        SYS_CTRL_GPIO_TERTSEL_CTRL_1
#define PCI_RESET_SET_OE_REG        GPIO_B_OUTPUT_ENABLE_SET
#define PCI_RESET_OUTPUT_SET_REG    GPIO_B_OUTPUT_SET
#define PCI_RESET_OUTPUT_CLR_REG    GPIO_B_OUTPUT_CLEAR
#endif

#define PCI_RESET_MASK (1UL << (PCI_RESET_NUM))
#endif // CONFIG_OXNAS_PCI_RESET_GPIO

#define PCI_CLOCK_NUM               10
#define PCI_CLOCK_PRISEL_REG        SYS_CTRL_GPIO_PRIMSEL_CTRL_0
#define PCI_CLOCK_SET_OE_REG        GPIO_A_OUTPUT_ENABLE_SET
#define PCI_CLOCK_MASK              (1UL << (PCI_CLOCK_NUM))

#ifdef CONFIG_OXNAS_SATA_POWER_GPIO_1
#if (CONFIG_OXNAS_SATA_POWER_GPIO_1 < 32)
#define SATA_POWER_1_NUM            CONFIG_OXNAS_SATA_POWER_GPIO_1
#define SATA_POWER_1_PRISEL_REG     SYS_CTRL_GPIO_PRIMSEL_CTRL_0
#define SATA_POWER_1_SECSEL_REG     SYS_CTRL_GPIO_SECSEL_CTRL_0
#define SATA_POWER_1_TERSEL_REG     SYS_CTRL_GPIO_TERTSEL_CTRL_0
#define SATA_POWER_1_SET_OE_REG     GPIO_A_OUTPUT_ENABLE_SET
#define SATA_POWER_1_OUTPUT_SET_REG GPIO_A_OUTPUT_SET
#define SATA_POWER_1_OUTPUT_CLR_REG GPIO_A_OUTPUT_CLEAR
#else
#define SATA_POWER_1_NUM            ((CONFIG_OXNAS_SATA_POWER_GPIO_1) - 32)
#define SATA_POWER_1_PRISEL_REG     SYS_CTRL_GPIO_PRIMSEL_CTRL_1
#define SATA_POWER_1_SECSEL_REG     SYS_CTRL_GPIO_SECSEL_CTRL_1
#define SATA_POWER_1_TERSEL_REG     SYS_CTRL_GPIO_TERTSEL_CTRL_1
#define SATA_POWER_1_SET_OE_REG     GPIO_B_OUTPUT_ENABLE_SET
#define SATA_POWER_1_OUTPUT_SET_REG GPIO_B_OUTPUT_SET
#define SATA_POWER_1_OUTPUT_CLR_REG GPIO_B_OUTPUT_CLEAR
#endif

#define SATA_POWER_1_MASK   (1UL << (SATA_POWER_1_NUM))
#endif // CONFIG_OXNAS_SATA_POWER_GPIO_1

#ifdef CONFIG_OXNAS_SATA_POWER_GPIO_2
#if (CONFIG_OXNAS_SATA_POWER_GPIO_2 < 32)
#define SATA_POWER_2_NUM            CONFIG_OXNAS_SATA_POWER_GPIO_2
#define SATA_POWER_2_PRISEL_REG     SYS_CTRL_GPIO_PRIMSEL_CTRL_0
#define SATA_POWER_2_SECSEL_REG     SYS_CTRL_GPIO_SECSEL_CTRL_0
#define SATA_POWER_2_TERSEL_REG     SYS_CTRL_GPIO_TERTSEL_CTRL_0
#define SATA_POWER_2_SET_OE_REG     GPIO_A_OUTPUT_ENABLE_SET
#define SATA_POWER_2_OUTPUT_SET_REG GPIO_A_OUTPUT_SET
#define SATA_POWER_2_OUTPUT_CLR_REG GPIO_A_OUTPUT_CLEAR
#else
#define SATA_POWER_2_NUM            ((CONFIG_OXNAS_SATA_POWER_GPIO_2) - 32)
#define SATA_POWER_2_PRISEL_REG     SYS_CTRL_GPIO_PRIMSEL_CTRL_1
#define SATA_POWER_2_SECSEL_REG     SYS_CTRL_GPIO_SECSEL_CTRL_1
#define SATA_POWER_2_TERSEL_REG     SYS_CTRL_GPIO_TERTSEL_CTRL_1
#define SATA_POWER_2_SET_OE_REG     GPIO_B_OUTPUT_ENABLE_SET
#define SATA_POWER_2_OUTPUT_SET_REG GPIO_B_OUTPUT_SET
#define SATA_POWER_2_OUTPUT_CLR_REG GPIO_B_OUTPUT_CLEAR
#endif

#define SATA_POWER_2_MASK   (1UL << (SATA_POWER_2_NUM))
#endif // CONFIG_OXNAS_SATA_POWER_GPIO_2

#ifdef CONFIG_OXNAS_USB_HUB_RESET_GPIO
#if (CONFIG_OXNAS_USB_HUB_RESET_GPIO < 32)
#define USB_HUB_RESET_NUM            CONFIG_OXNAS_USB_HUB_RESET_GPIO
#define USB_HUB_RESET_PRISEL_REG     SYS_CTRL_GPIO_PRIMSEL_CTRL_0
#define USB_HUB_RESET_SECSEL_REG     SYS_CTRL_GPIO_SECSEL_CTRL_0
#define USB_HUB_RESET_TERSEL_REG     SYS_CTRL_GPIO_TERTSEL_CTRL_0
#define USB_HUB_RESET_SET_OE_REG     GPIO_A_OUTPUT_ENABLE_SET
#define USB_HUB_RESET_OUTPUT_SET_REG GPIO_A_OUTPUT_SET
#define USB_HUB_RESET_OUTPUT_CLR_REG GPIO_A_OUTPUT_CLEAR
#else
#define USB_HUB_RESET_NUM            ((CONFIG_OXNAS_USB_HUB_RESET_GPIO) - 32)
#define USB_HUB_RESET_PRISEL_REG     SYS_CTRL_GPIO_PRIMSEL_CTRL_1
#define USB_HUB_RESET_SECSEL_REG     SYS_CTRL_GPIO_SECSEL_CTRL_1
#define USB_HUB_RESET_TERSEL_REG     SYS_CTRL_GPIO_TERTSEL_CTRL_1
#define USB_HUB_RESET_SET_OE_REG     GPIO_B_OUTPUT_ENABLE_SET
#define USB_HUB_RESET_OUTPUT_SET_REG GPIO_B_OUTPUT_SET
#define USB_HUB_RESET_OUTPUT_CLR_REG GPIO_B_OUTPUT_CLEAR
#endif

#define USB_HUB_RESET_MASK	(1UL << (USB_HUB_RESET_NUM))
#endif // CONFIG_OXNAS_USB_HUB_RESET_GPIO

extern struct sys_timer oxnas_timer;

// The spinlock exported to allow atomic use of GPIO register set
spinlock_t oxnas_gpio_spinlock;

// To hold LED inversion state
int oxnas_global_invert_leds = 0;
#include <linux/module.h>
EXPORT_SYMBOL(oxnas_global_invert_leds);

static struct map_desc oxnas_io_desc[] __initdata = {
    { CORE_MODULE_BASE,     __phys_to_pfn(CORE_MODULE_BASE_PA),     SZ_4K,   MT_DEVICE },
    { APB_BRIDGE_A_BASE,    __phys_to_pfn(APB_BRIDGE_A_BASE_PA),    SZ_16M,  MT_DEVICE },
    { STATIC_CONTROL_BASE,  __phys_to_pfn(STATIC_CONTROL_BASE_PA),  SZ_4K,   MT_DEVICE },
    { STATIC_CS0_BASE,      __phys_to_pfn(STATIC_CS0_BASE_PA),      SZ_4K,   MT_DEVICE },
    { STATIC_CS1_BASE,      __phys_to_pfn(STATIC_CS1_BASE_PA),      SZ_4K,   MT_DEVICE },
    { STATIC_CS2_BASE,      __phys_to_pfn(STATIC_CS2_BASE_PA),      SZ_4K,   MT_DEVICE },
    { APB_BRIDGE_B_BASE,    __phys_to_pfn(APB_BRIDGE_B_BASE_PA),    SZ_16M,  MT_DEVICE },
    { USB_BASE,             __phys_to_pfn(USB_BASE_PA),             SZ_4M,   MT_DEVICE },
    { MAC_BASE,             __phys_to_pfn(MAC_BASE_PA),             SZ_4M,   MT_DEVICE },
    { ROM_BASE,             __phys_to_pfn(ROM_BASE_PA),             SZ_16K,  MT_DEVICE },
    { PCI_CSRS_BASE,        __phys_to_pfn(PCI_CSRS_BASE_PA),        SZ_4K,   MT_DEVICE }
#ifdef CONFIG_SUPPORT_LEON
#if (CONFIG_LEON_PAGES == 1)
   ,{ LEON_IMAGE_BASE,			__phys_to_pfn(LEON_IMAGE_BASE_PA),			SZ_4K, MT_DEVICE }
#elif (CONFIG_LEON_PAGES == 2)
   ,{ LEON_IMAGE_BASE,			__phys_to_pfn(LEON_IMAGE_BASE_PA),			SZ_8K, MT_DEVICE }
#elif (CONFIG_LEON_PAGES == 3)
   ,{ LEON_IMAGE_BASE,		    __phys_to_pfn(LEON_IMAGE_BASE_PA),			SZ_8K, MT_DEVICE }
   ,{ LEON_IMAGE_BASE+0x2000,	__phys_to_pfn(LEON_IMAGE_BASE_PA+0x2000),	SZ_4K, MT_DEVICE }
#elif (CONFIG_LEON_PAGES == 4)
   ,{ LEON_IMAGE_BASE,		    __phys_to_pfn(LEON_IMAGE_BASE_PA),	  		SZ_8K, MT_DEVICE }
   ,{ LEON_IMAGE_BASE+0x2000,	__phys_to_pfn(LEON_IMAGE_BASE_PA+0x2000),	SZ_8K, MT_DEVICE }
#else
#error "Unsupported number of Leon code pages"
#endif // CONFIG_LEON_PAGES
#endif // CONFIG_SUPPORT_LEON
	/*
	 * Upto 8 pages for GMAC/DMA descriptors plus ARM/Leon TSO workspace if
	 * Leon TSO is in use
	 */
   ,{ SRAM_BASE,            __phys_to_pfn(SRAM_PA),                 SZ_128K, MT_DEVICE }
   ,{ PERIPH_BASE,          __phys_to_pfn(PERIPH_BASE_PA),          SZ_16K, MT_DEVICE }
};

static struct resource usb_resources[] = {
	[0] = {
		.start		= USB_BASE_PA,
		.end		= USB_BASE_PA + 0x10000 - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= USB_FS_INTERRUPT,
		.end		= USB_FS_INTERRUPT,
		.flags		= IORESOURCE_IRQ,
	},
};

static u64 usb_dmamask = ~(u32)0;

static struct platform_device usb_device = {
	.name		= "oxnas-ehci",
	.id		= 0,
	.dev = {
		.dma_mask		= &usb_dmamask,
		.coherent_dma_mask	= 0xffffffff,
	},
	.num_resources	= ARRAY_SIZE(usb_resources),
	.resource	= usb_resources,
};

static struct platform_device *platform_devices[] __initdata = {
	&usb_device,
};

/* used by entry-macro.S */
void __iomem *gic_cpu_base_addr;


#define STD_COM_FLAGS (ASYNC_BOOT_AUTOCONF | ASYNC_SKIP_TEST | ASYNC_AUTO_IRQ)

#define INT_UART_BASE_BAUD (NOMINAL_SYSCLK)

#ifdef CONFIG_ARCH_OXNAS_UART1
static struct uart_port internal_serial_port_1 = {
	.membase	= (char *)(UART_1_BASE),
	.mapbase	= UART_1_BASE_PA,
	.irq		= UART_1_INTERRUPT,
	.flags		= STD_COM_FLAGS,
	.iotype		= UPIO_MEM,
	.regshift	= 0,
	.uartclk	= INT_UART_BASE_BAUD,
	.line		= 0,
	.type		= PORT_16550A,
	.fifosize	= 16
};
#endif // CONFIG_ARCH_OXNAS_UART1

#ifdef CONFIG_ARCH_OXNAS_UART2
static struct uart_port internal_serial_port_2 = {
	.membase	= (char *)(UART_2_BASE),
	.mapbase	= UART_2_BASE_PA,
	.irq		= UART_2_INTERRUPT,
	.flags		= STD_COM_FLAGS,
	.iotype		= UPIO_MEM,
	.regshift	= 0,
	.uartclk	= INT_UART_BASE_BAUD,
	.line		= 0,
	.type		= PORT_16550A,
	.fifosize	= 16
};
#endif // CONFIG_ARCH_OXNAS_UART2

#ifdef CONFIG_ARCH_OXNAS_UART3
static struct uart_port internal_serial_port_3 = {
	.membase	= (char *)(UART_3_BASE),
	.mapbase	= UART_3_BASE_PA,
	.irq		= UART_3_INTERRUPT,
	.flags		= STD_COM_FLAGS,
	.iotype		= UPIO_MEM,
	.regshift	= 0,
	.uartclk	= INT_UART_BASE_BAUD,
	.line		= 0,
	.type		= PORT_16550A,
	.fifosize	= 16
};
#endif // CONFIG_ARCH_OXNAS_UART3

#ifdef CONFIG_ARCH_OXNAS_UART4
static struct uart_port internal_serial_port_4 = {
	.membase	= (char *)(UART_4_BASE),
	.mapbase	= UART_4_BASE_PA,
	.irq		= UART_4_INTERRUPT,
	.flags		= STD_COM_FLAGS,
	.iotype		= UPIO_MEM,
	.regshift	= 0,
	.uartclk	= INT_UART_BASE_BAUD,
	.line		= 0,
	.type		= PORT_16550A,
	.fifosize	= 16
};
#endif // CONFIG_ARCH_OXNAS_UART4

static void __init oxnas_mapio(void)
{
    unsigned int uart_line=0;

    //printk("oxnas_mapio()\n");

    // Setup kernel mappings for hardware cores
    iotable_init(oxnas_io_desc, ARRAY_SIZE(oxnas_io_desc));

    //printk("oxnas_mapio() done.\n");
    
    // Configure the DDR controller arbitration scheme
    *(volatile u32*)DDR_ARB_REG = ((1UL << DDR_ARB_DATDIR_NCH_BIT) |
                                   (1UL << DDR_ARB_DATDIR_EN_BIT)  |
                                   (1UL << DDR_ARB_REQAGE_EN_BIT)  |
                                   (1UL << DDR_ARB_LRUBANK_EN_BIT) |
                                   (1UL << DDR_ARB_MIDBUF_BIT));

	// Configure read buffers - Do not disable any read buffers
	*(volatile u32*)DDR_AHB_REG = 0UL;

	// Configure wrapping - Ignore wrap
	// Configure HPROT - Ignore all HPROT except ARM data
	*(volatile u32*)DDR_AHB2_REG = ((1UL << DDR_AHB2_IGNORE_WRAP_ARMD_BIT)   |
                                    (1UL << DDR_AHB2_IGNORE_WRAP_ARMI_BIT)   |
                                    (1UL << DDR_AHB2_IGNORE_WRAP_COPRO_BIT)  |
                                    (1UL << DDR_AHB2_IGNORE_WRAP_DMAA_BIT)   |
                                    (1UL << DDR_AHB2_IGNORE_WRAP_DMAB_BIT)   |
                                    (1UL << DDR_AHB2_IGNORE_WRAP_PCI_BIT)    |
                                    (1UL << DDR_AHB2_IGNORE_WRAP_GMAC_BIT)   |
                                    (1UL << DDR_AHB2_IGNORE_WRAP_US_BIT)     |
                                    (1UL << DDR_AHB2_IGNORE_HPROT_ARMI_BIT)  |
                                    (1UL << DDR_AHB2_IGNORE_HPROT_COPRO_BIT) |
                                    (1UL << DDR_AHB2_IGNORE_HPROT_DMAA_BIT)  |
                                    (1UL << DDR_AHB2_IGNORE_HPROT_DMAB_BIT)  |
                                    (1UL << DDR_AHB2_IGNORE_HPROT_PCI_BIT)   |
                                    (1UL << DDR_AHB2_IGNORE_HPROT_GMAC_BIT)  |
                                    (1UL << DDR_AHB2_IGNORE_HPROT_USB_BIT));

	// Configure burst ordering - Do not disable burst ordering
	// Configure non-cachable - Do not prevent non-cachable accesses from using read buffers
	*(volatile u32*)DDR_AHB3_REG = 0UL;

	// Configure read buffer timeout - Do not enable read buffer invalidate after timeout
	// Configure write behind - Enable write behind coherency
	*(volatile u32*)DDR_AHB4_REG = ((1UL << DDR_AHB4_EN_WRBEHIND_ARMD_BIT)  |
									 (1UL << DDR_AHB4_EN_WRBEHIND_ARMI_BIT)  |
									 (1UL << DDR_AHB4_EN_WRBEHIND_COPRO_BIT) |
									 (1UL << DDR_AHB4_EN_WRBEHIND_DMAA_BIT)  |
									 (1UL << DDR_AHB4_EN_WRBEHIND_DMAB_BIT)  |
									 (1UL << DDR_AHB4_EN_WRBEHIND_PCI_BIT)   |
									 (1UL << DDR_AHB4_EN_WRBEHIND_GMAC_BIT)  |
									 (1UL << DDR_AHB4_EN_WRBEHIND_USB_BIT));


    // Enable all DDR client interfaces
    *(volatile u32*)DDR_BLKEN_REG |= (((1UL << DDR_BLKEN_CLIENTS_NUM_BITS) - 1) << DDR_BLKEN_CLIENTS_BIT);

    //printk("DDR setup done.\n");
    
    
#ifdef CONFIG_ARCH_OXNAS_UART1
    // Block reset UART1
    *(volatile u32*)SYS_CTRL_RSTEN_SET_CTRL = (1UL << SYS_CTRL_RSTEN_UART1_BIT);
    *(volatile u32*)SYS_CTRL_RSTEN_CLR_CTRL = (1UL << SYS_CTRL_RSTEN_UART1_BIT);

    // Route UART1 SOUT onto external pin
    *(volatile u32*)SYS_CTRL_GPIO_PRIMSEL_CTRL_0 &= ~0x80000000;
    *(volatile u32*)SYS_CTRL_GPIO_SECSEL_CTRL_0  &= ~0x80000000;
    *(volatile u32*)SYS_CTRL_GPIO_TERTSEL_CTRL_0 |=  0x80000000;

    // Route UART1 SIN onto external pin
    *(volatile u32*)SYS_CTRL_GPIO_PRIMSEL_CTRL_1 &= ~0x00000001;
    *(volatile u32*)SYS_CTRL_GPIO_SECSEL_CTRL_1  &= ~0x00000001;
    *(volatile u32*)SYS_CTRL_GPIO_TERTSEL_CTRL_1 |=  0x00000001;

    // Setup GPIO line direction for UART1 SOUT
    *(volatile u32*)GPIO_A_OUTPUT_ENABLE_SET   |= 0x80000000;

    // Setup GPIO line direction for UART1 SIN
    *(volatile u32*)GPIO_B_OUTPUT_ENABLE_CLEAR |= 0x00000001;

#ifdef CONFIG_ARCH_OXNAS_UART1_MODEM
    // Route UART1 modem control lines onto external pins
    *(volatile u32*)SYS_CTRL_GPIO_PRIMSEL_CTRL_0 &= ~0x78000000;
    *(volatile u32*)SYS_CTRL_GPIO_SECSEL_CTRL_0  &= ~0x78000000;
    *(volatile u32*)SYS_CTRL_GPIO_TERTSEL_CTRL_0 |=  0x78000000;

    *(volatile u32*)SYS_CTRL_GPIO_PRIMSEL_CTRL_1 &= ~0x00000006;
    *(volatile u32*)SYS_CTRL_GPIO_SECSEL_CTRL_1  &= ~0x00000006;
    *(volatile u32*)SYS_CTRL_GPIO_TERTSEL_CTRL_1 |=  0x00000006;

    // Setup GPIO line directions for UART1 modem control lines
    *(volatile u32*)GPIO_A_OUTPUT_ENABLE_SET   |= 0x08000000;
    *(volatile u32*)GPIO_A_OUTPUT_ENABLE_CLEAR |= 0x70000000;

    *(volatile u32*)GPIO_B_OUTPUT_ENABLE_SET   |= 0x00000004;
    *(volatile u32*)GPIO_B_OUTPUT_ENABLE_CLEAR |= 0x00000002;
#endif // CONFIG_ARCH_OXNAS_UART1_MODEM

    // Give Linux a contiguous numbering scheme for available UARTs
    internal_serial_port_1.line = uart_line++;
    early_serial_setup(&internal_serial_port_1);
#endif // CONFIG_ARCH_OXNAS_UART1

#ifdef CONFIG_ARCH_OXNAS_UART2
    //printk("UART2 setup beginning.\n");

    // Block reset UART2
//    *(volatile u32*)SYS_CTRL_RSTEN_SET_CTRL = (1UL << SYS_CTRL_RSTEN_UART2_BIT);
//    *(volatile u32*)SYS_CTRL_RSTEN_CLR_CTRL = (1UL << SYS_CTRL_RSTEN_UART2_BIT);

    // Route UART2 SIN/SOUT onto external pin
    *(volatile u32*)SYS_CTRL_GPIO_PRIMSEL_CTRL_0 &= ~0x00500000;
    *(volatile u32*)SYS_CTRL_GPIO_SECSEL_CTRL_0  &= ~0x00500000;
    *(volatile u32*)SYS_CTRL_GPIO_TERTSEL_CTRL_0 |=  0x00500000;

    // Setup GPIO line directions for UART2 SIN/SOUT
    *(volatile u32*)GPIO_A_OUTPUT_ENABLE_SET   |= 0x00100000;
    *(volatile u32*)GPIO_A_OUTPUT_ENABLE_CLEAR |= 0x00400000;

#ifdef CONFIG_ARCH_OXNAS_UART2_MODEM
    // Route UART2 modem control lines onto external pins
    *(volatile u32*)SYS_CTRL_GPIO_PRIMSEL_CTRL_0 &= ~0x07800300;
    *(volatile u32*)SYS_CTRL_GPIO_SECSEL_CTRL_0  &= ~0x07800300;
    *(volatile u32*)SYS_CTRL_GPIO_TERTSEL_CTRL_0 |=  0x07800300;

    // Setup GPIO line directions for UART2 modem control lines
    *(volatile u32*)GPIO_A_OUTPUT_ENABLE_SET   |= 0x02000200;
    *(volatile u32*)GPIO_A_OUTPUT_ENABLE_CLEAR |= 0x05800100;
#endif // CONFIG_ARCH_OXNAS_UART2_MODEM

    // Give Linux a contiguous numbering scheme for available UARTs
    internal_serial_port_2.line = uart_line++;
    early_serial_setup(&internal_serial_port_2);
    //printk("UART2 setup done.\n");
#endif // CONFIG_ARCH_OXNAS_UART2

#ifdef CONFIG_ARCH_OXNAS_UART3
    // Block reset UART3
    *(volatile u32*)SYS_CTRL_RSTEN_SET_CTRL = (1UL << SYS_CTRL_RSTEN_UART3_BIT);
    *(volatile u32*)SYS_CTRL_RSTEN_CLR_CTRL = (1UL << SYS_CTRL_RSTEN_UART3_BIT);

    // Route UART3 SIN/SOUT onto external pin
    *(volatile u32*)SYS_CTRL_GPIO_PRIMSEL_CTRL_0 &= ~0x000000C0;
    *(volatile u32*)SYS_CTRL_GPIO_SECSEL_CTRL_0  &= ~0x000000C0;
    *(volatile u32*)SYS_CTRL_GPIO_TERTSEL_CTRL_0 |=  0x000000C0;

    // Setup GPIO line directions for UART3 SIN/SOUT
    *(volatile u32*)GPIO_A_OUTPUT_ENABLE_SET   |= 0x00000080;
    *(volatile u32*)GPIO_A_OUTPUT_ENABLE_CLEAR |= 0x00000040;

    // Enable UART3 interrupt
    *(volatile u32*)SYS_CTRL_UART_CTRL |= (1UL << SYS_CTRL_UART3_IQ_EN);

#ifdef CONFIG_ARCH_OXNAS_UART3_MODEM
    // Route UART3 modem control lines onto external pins
    *(volatile u32*)SYS_CTRL_GPIO_PRIMSEL_CTRL_0 &= ~0x0000003f;
    *(volatile u32*)SYS_CTRL_GPIO_SECSEL_CTRL_0  &= ~0x0000003f;
    *(volatile u32*)SYS_CTRL_GPIO_TERTSEL_CTRL_0 |=  0x0000003f;

    // Setup GPIO line directions for UART3 modem control lines
    *(volatile u32*)GPIO_A_OUTPUT_ENABLE_SET   |= 0x00000030;
    *(volatile u32*)GPIO_A_OUTPUT_ENABLE_CLEAR |= 0x0000000f;
#endif // CONFIG_ARCH_OXNAS_UART3_MODEM

    // Give Linux a contiguous numbering scheme for available UARTs
    internal_serial_port_3.line = uart_line++;
    early_serial_setup(&internal_serial_port_3);
#endif // CONFIG_ARCH_OXNAS_UART3

#ifdef CONFIG_ARCH_OXNAS_UART4
    // Block reset UART4
    *(volatile u32*)SYS_CTRL_RSTEN_SET_CTRL = (1UL << SYS_CTRL_RSTEN_UART4_BIT);
    *(volatile u32*)SYS_CTRL_RSTEN_CLR_CTRL = (1UL << SYS_CTRL_RSTEN_UART4_BIT);

    // Enable UART4 interrupt
    *(volatile u32*)SYS_CTRL_UART_CTRL |= (1UL << SYS_CTRL_UART4_IQ_EN);

    // Enable UART4 to override PCI functions onto GPIOs
    *(volatile u32*)SYS_CTRL_UART_CTRL |= (1UL << SYS_CTRL_UART4_NOT_PCI_MODE);

    internal_serial_port_4.line = uart_line++;
    early_serial_setup(&internal_serial_port_4);
#endif // CONFIG_ARCH_OXNAS_UART4

#ifdef CONFIG_PCI
    // Block reset PCI core
    *(volatile u32*)SYS_CTRL_RSTEN_SET_CTRL = (1UL << SYS_CTRL_RSTEN_PCI_BIT);
    *(volatile u32*)SYS_CTRL_RSTEN_CLR_CTRL = (1UL << SYS_CTRL_RSTEN_PCI_BIT);

    // Setup the PCI clock divider
    {
    static const u32 PCIDIV_MASK = (((1UL << SYS_CTRL_CKCTRL_CTRL_PCIDIV_NUM_BITS) - 1) << SYS_CTRL_CKCTRL_CTRL_PCIDIV_BIT);
    *(volatile u32*)SYS_CTRL_CKCTRL_CTRL &= ~PCIDIV_MASK;
    *(volatile u32*)SYS_CTRL_CKCTRL_CTRL |= (PCI_CLOCK_DIVIDER << SYS_CTRL_CKCTRL_CTRL_PCIDIV_BIT);
    }

    // Enable clock to PCI core
    *(volatile u32*)SYS_CTRL_CKEN_SET_CTRL = (1UL << SYS_CTRL_CKEN_PCI_BIT);

    // Enable auto-arbitration between static and PCI
    *(u32*)SYS_CTRL_PCI_CTRL1 &= ~(1UL << SYSCTL_PCI_CTRL1_SYSPCI_STATIC_REQ);

    // Enable primary function on PCI clock line to be looped back
    writel(readl(PCI_CLOCK_PRISEL_REG) | PCI_CLOCK_MASK, PCI_CLOCK_PRISEL_REG);

    // Enable GPIO output on PCI clock line to be looped back
    writel(PCI_CLOCK_MASK, PCI_CLOCK_SET_OE_REG);

#ifdef CONFIG_OXNAS_PCI_RESET
    // Disable primary, secondary and teriary GPIO functions on PCI reset line
    writel(readl(PCI_RESET_PRISEL_REG) & ~PCI_RESET_MASK, PCI_RESET_PRISEL_REG);
    writel(readl(PCI_RESET_SECSEL_REG) & ~PCI_RESET_MASK, PCI_RESET_SECSEL_REG);
    writel(readl(PCI_RESET_TERSEL_REG) & ~PCI_RESET_MASK, PCI_RESET_TERSEL_REG);

    // Assert PCI reset from GPIO line
    writel(PCI_RESET_MASK, PCI_RESET_OUTPUT_CLR_REG);

    // Enable GPIO output on PCI reset line
    writel(PCI_RESET_MASK, PCI_RESET_SET_OE_REG);

    // Wait awhile for PCI reset to take effect
    mdelay(100);

    // Deassert PCI reset from GPIO line
    writel(PCI_RESET_MASK, PCI_RESET_OUTPUT_SET_REG);
#endif // CONFIG_OXNAS_PCI_RESET
#endif // CONFIG_PCI

#ifdef CONFIG_OXNAS_SATA_POWER_1
    // Disable primary, secondary and teriary GPIO functions on SATA 1 power line
    writel(readl(SATA_POWER_1_PRISEL_REG) & ~SATA_POWER_1_MASK, SATA_POWER_1_PRISEL_REG);
    writel(readl(SATA_POWER_1_SECSEL_REG) & ~SATA_POWER_1_MASK, SATA_POWER_1_SECSEL_REG);
    writel(readl(SATA_POWER_1_TERSEL_REG) & ~SATA_POWER_1_MASK, SATA_POWER_1_TERSEL_REG);

    // Enable power to SATA 1
    writel(SATA_POWER_1_MASK, SATA_POWER_1_OUTPUT_SET_REG);

    // Enable GPIO output on SATA 1 power line
    writel(SATA_POWER_1_MASK, SATA_POWER_1_SET_OE_REG);
#endif // CONFIG_OXNAS_SATA_POWER_1

#ifdef CONFIG_OXNAS_SATA_POWER_2
    // Disable primary, secondary and teriary GPIO functions on SATA 2 power line
    writel(readl(SATA_POWER_2_PRISEL_REG) & ~SATA_POWER_2_MASK, SATA_POWER_2_PRISEL_REG);
    writel(readl(SATA_POWER_2_SECSEL_REG) & ~SATA_POWER_2_MASK, SATA_POWER_2_SECSEL_REG);
    writel(readl(SATA_POWER_2_TERSEL_REG) & ~SATA_POWER_2_MASK, SATA_POWER_2_TERSEL_REG);

    // Enable power to SATA 2
    writel(SATA_POWER_2_MASK, SATA_POWER_2_OUTPUT_SET_REG);

    // Enable GPIO output on SATA 2 power line
    writel(SATA_POWER_2_MASK, SATA_POWER_2_SET_OE_REG);
#endif // CONFIG_OXNAS_SATA_POWER_2

#ifdef CONFIG_OXNAS_INSTRUMENT_COPIES_GPIO
    // Use GPIO 6 (normally PCI Req 6) for copies instrumentation
    #define INSTRUMENT_COPIES_GPIO_MASK ((1UL << 6) | (1UL << 7))

    // Enable normal GPIO on line
    writel(readl(SYS_CTRL_GPIO_PRIMSEL_CTRL_0) & ~INSTRUMENT_COPIES_GPIO_MASK, SYS_CTRL_GPIO_PRIMSEL_CTRL_0);
    writel(readl(SYS_CTRL_GPIO_SECSEL_CTRL_0)  & ~INSTRUMENT_COPIES_GPIO_MASK, SYS_CTRL_GPIO_SECSEL_CTRL_0);
    writel(readl(SYS_CTRL_GPIO_TERTSEL_CTRL_0) & ~INSTRUMENT_COPIES_GPIO_MASK, SYS_CTRL_GPIO_TERTSEL_CTRL_0);

    // Set line inactive to begin with
    writel(INSTRUMENT_COPIES_GPIO_MASK, GPIO_A_OUTPUT_CLEAR);

    // Enable line as an output
    writel(INSTRUMENT_COPIES_GPIO_MASK, GPIO_A_OUTPUT_ENABLE_SET);
#endif // CONFIG_OXNAS_INSTRUMENT_COPIES_GPIO

#ifdef CONFIG_OXNAS_USB_CKOUT
    // Enable secondary function (USB clock out) on GPIO 10
    writel(readl(SYS_CTRL_GPIO_PRIMSEL_CTRL_0) & ~(1UL << 10), SYS_CTRL_GPIO_PRIMSEL_CTRL_0);
    writel(readl(SYS_CTRL_GPIO_SECSEL_CTRL_0)  |  (1UL << 10), SYS_CTRL_GPIO_SECSEL_CTRL_0);
#endif // CONFIG_OXNAS_USB_CKOUT

#ifdef CONFIG_OXNAS_USB_HUB_RESET_CONTROL
    // Disable primary, secondary and teriary GPIO functions on USB hub reset control line
    writel(readl(USB_HUB_RESET_PRISEL_REG) & ~USB_HUB_RESET_MASK, USB_HUB_RESET_PRISEL_REG);
    writel(readl(USB_HUB_RESET_SECSEL_REG) & ~USB_HUB_RESET_MASK, USB_HUB_RESET_SECSEL_REG);
    writel(readl(USB_HUB_RESET_TERSEL_REG) & ~USB_HUB_RESET_MASK, USB_HUB_RESET_TERSEL_REG);

#ifdef CONFIG_OXNAS_USB_HUB_RESET_TOGGLE
	// Assert USB hub reset
	writel(USB_HUB_RESET_MASK, CONFIG_OXNAS_USB_HUB_RESET_ACTIVE_HIGH ? USB_HUB_RESET_OUTPUT_SET_REG : USB_HUB_RESET_OUTPUT_CLR_REG);
#else
	// Deassert USB hub reset
	writel(USB_HUB_RESET_MASK, CONFIG_OXNAS_USB_HUB_RESET_ACTIVE_HIGH ? USB_HUB_RESET_OUTPUT_CLR_REG : USB_HUB_RESET_OUTPUT_SET_REG);
#endif // CONFIG_OXNAS_USB_HUB_RESET_TOGGLE

    // Enable GPIO output on USB hub reset line
    writel(USB_HUB_RESET_MASK, USB_HUB_RESET_SET_OE_REG);

#ifdef CONFIG_OXNAS_USB_HUB_RESET_TOGGLE
	if (CONFIG_OXNAS_USB_HUB_RESET_PERIOD_MS > 0) {
		// Wait for USB hub reset toggle assertion time
		mdelay(CONFIG_OXNAS_USB_HUB_RESET_PERIOD_MS);
	}

	// Deassert USB hub reset
	writel(USB_HUB_RESET_MASK, CONFIG_OXNAS_USB_HUB_RESET_ACTIVE_HIGH ? USB_HUB_RESET_OUTPUT_CLR_REG : USB_HUB_RESET_OUTPUT_SET_REG);
#endif // CONFIG_OXNAS_USB_HUB_RESET_TOGGLE

#endif // CONFIG_OXNAS_USB_HUB_RESET_CONTROL
}

static void __init oxnas_fixup(
    struct machine_desc *desc,
    struct tag *tags,
    char **cmdline,
    struct meminfo *mi)
{

    mi->nr_banks = 0;
    mi->bank[mi->nr_banks].start = SDRAM_PA;
    mi->bank[mi->nr_banks].size  = SDRAM_SIZE;
    mi->bank[mi->nr_banks].node = mi->nr_banks;
    ++mi->nr_banks;
#ifdef CONFIG_DISCONTIGMEM
    mi->bank[mi->nr_banks].start = SRAM_PA;
    mi->bank[mi->nr_banks].size  = SRAM_SIZE;
#ifdef LEON_IMAGE_IN_SRAM
    mi->bank[mi->nr_banks].size -= LEON_IMAGE_SIZE;
#endif
    mi->bank[mi->nr_banks].node = mi->nr_banks;
    ++mi->nr_banks;
#endif

//printk(KERN_NOTICE "%d memory %s\n", mi->nr_banks, (mi->nr_banks > 1) ? "regions" : "region");
}

#ifdef CONFIG_DO_MEM_TEST
static void __init oxnas_asm_copy(void* dst, void* src, u32 length)
{
    // Assume the length is consistent with transfering 8 quads per load/store
    asm volatile(
        "1:ldmia %0!, {r3, r4, r5, r6, r7, r8, r9, r12};"
        "subs %2, %2, #32;"
        "stmia %1!, {r3, r4, r5, r6, r7, r8, r9, r12};"
        "bne 1b;"
        :
        : "r" (src), "r" (dst), "r" (length)
        : "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r12");
}

static void __init oxnas_mem_test(void)
{
    static const unsigned BUFFER_SIZE_CHARS = 16*1024;
    static const unsigned BUFFER_ELEMENTS = (BUFFER_SIZE_CHARS / sizeof(unsigned long));

    dma_addr_t dma_address;
    unsigned long* buffer;

    buffer = dma_alloc_coherent(0, BUFFER_SIZE_CHARS, &dma_address, GFP_KERNEL | GFP_DMA);
    if (!buffer) {
        printk(KERN_ERR "$RFailed to allocate ucached/unbuffered memory test buffer\n");
    } else {
        static const int ITERATIONS = 10;

        unsigned long* buf1 = buffer;
        unsigned long* buf2 = buffer + (BUFFER_ELEMENTS/2);
        int j;
        u32* time1 = (u32*)kmalloc(ITERATIONS *sizeof(u32), GFP_KERNEL);
        u32* time2 = (u32*)kmalloc(ITERATIONS *sizeof(u32), GFP_KERNEL);

        BUG_ON(!time1 || !time2);
        
        printk("Uncached/unbuffered: src = 0x%08x, dst = 0x%08x, length = %u, elements = %u, dma_address = 0x%08x\n", (u32)buf1, (u32)buf2, BUFFER_SIZE_CHARS/2, BUFFER_ELEMENTS/2, dma_address);

        printk("\nAll accesses:\n");
        init_ahb_monitors(AHB_MON_HWRITE_READS_AND_WRITES, 0, 0, 0, 0);

        for (j=0; j < ITERATIONS; j++) {
            unsigned long* src = buf1;
            unsigned long* dst = buf2;
//            int i;

            time1[j] = readl(TIMER2_VALUE);
//            memcpy(dst, src, BUFFER_SIZE_CHARS/2);
            oxnas_asm_copy(dst, src, BUFFER_SIZE_CHARS/2);
//            for (i=0; i<BUFFER_ELEMENTS/2; i++) {
//                *dst++ = *src++;
//            }
            time2[j] = readl(TIMER2_VALUE);
        }
        read_ahb_monitors();
        for (j=0; j < ITERATIONS; j++) {
            printk("%u->%lu Bytes/s\n", time1[j]-time2[j], 100000UL * (BUFFER_SIZE_CHARS/2) / (time1[j]-time2[j]));
        }

        printk("\nNon-burst accesses:\n");
        init_ahb_monitors(AHB_MON_HWRITE_READS_AND_WRITES, ~0, AHB_MON_HBURST_SINGLE, 0, 0);
        for (j=0; j < ITERATIONS; j++) {
            unsigned long* src = buf1;
            unsigned long* dst = buf2;
//            int i;

            time1[j] = readl(TIMER2_VALUE);
//            memcpy(dst, src, BUFFER_SIZE_CHARS/2);
            oxnas_asm_copy(dst, src, BUFFER_SIZE_CHARS/2);
//            for (i=0; i<BUFFER_ELEMENTS/2; i++) {
//                *dst++ = *src++;
//            }
            time2[j] = readl(TIMER2_VALUE);
        }
        read_ahb_monitors();
        for (j=0; j < ITERATIONS; j++) {
            printk("%u->%lu Bytes/s\n", time1[j]-time2[j], 100000UL * (BUFFER_SIZE_CHARS/2) / (time1[j]-time2[j]));
        }

        printk("\nINCR accesses:\n");
        init_ahb_monitors(AHB_MON_HWRITE_READS_AND_WRITES, ~0, AHB_MON_HBURST_INCR, 0, 0);
        for (j=0; j < ITERATIONS; j++) {
            unsigned long* src = buf1;
            unsigned long* dst = buf2;
//            int i;

            time1[j] = readl(TIMER2_VALUE);
//            memcpy(dst, src, BUFFER_SIZE_CHARS/2);
            oxnas_asm_copy(dst, src, BUFFER_SIZE_CHARS/2);
//            for (i=0; i<BUFFER_ELEMENTS/2; i++) {
//                *dst++ = *src++;
//            }
            time2[j] = readl(TIMER2_VALUE);
        }
        read_ahb_monitors();
        for (j=0; j < ITERATIONS; j++) {
            printk("%u->%lu Bytes/s\n", time1[j]-time2[j], 100000UL * (BUFFER_SIZE_CHARS/2) / (time1[j]-time2[j]));
        }

        printk("\nWRAP4 accesses:\n");
        init_ahb_monitors(AHB_MON_HWRITE_READS_AND_WRITES, ~0, AHB_MON_HBURST_WRAP4, 0, 0);
        for (j=0; j < ITERATIONS; j++) {
            unsigned long* src = buf1;
            unsigned long* dst = buf2;
//            int i;

            time1[j] = readl(TIMER2_VALUE);
//            memcpy(dst, src, BUFFER_SIZE_CHARS/2);
            oxnas_asm_copy(dst, src, BUFFER_SIZE_CHARS/2);
//            for (i=0; i<BUFFER_ELEMENTS/2; i++) {
//                *dst++ = *src++;
//            }
            time2[j] = readl(TIMER2_VALUE);
        }
        read_ahb_monitors();
        for (j=0; j < ITERATIONS; j++) {
            printk("%u->%lu Bytes/s\n", time1[j]-time2[j], 100000UL * (BUFFER_SIZE_CHARS/2) / (time1[j]-time2[j]));
        }

        printk("\nINCR4 accesses:\n");
        init_ahb_monitors(AHB_MON_HWRITE_READS_AND_WRITES, ~0, AHB_MON_HBURST_INCR4, 0, 0);
        for (j=0; j < ITERATIONS; j++) {
            unsigned long* src = buf1;
            unsigned long* dst = buf2;
//            int i;

            time1[j] = readl(TIMER2_VALUE);
//            memcpy(dst, src, BUFFER_SIZE_CHARS/2);
            oxnas_asm_copy(dst, src, BUFFER_SIZE_CHARS/2);
//            for (i=0; i<BUFFER_ELEMENTS/2; i++) {
//                *dst++ = *src++;
//            }
            time2[j] = readl(TIMER2_VALUE);
        }
        read_ahb_monitors();
        for (j=0; j < ITERATIONS; j++) {
            printk("%u->%lu Bytes/s\n", time1[j]-time2[j], 100000UL * (BUFFER_SIZE_CHARS/2) / (time1[j]-time2[j]));
        }

        printk("\nWRAP8 accesses:\n");
        init_ahb_monitors(AHB_MON_HWRITE_READS_AND_WRITES, ~0, AHB_MON_HBURST_WRAP8, 0, 0);
        for (j=0; j < ITERATIONS; j++) {
            unsigned long* src = buf1;
            unsigned long* dst = buf2;
//            int i;

            time1[j] = readl(TIMER2_VALUE);
//            memcpy(dst, src, BUFFER_SIZE_CHARS/2);
            oxnas_asm_copy(dst, src, BUFFER_SIZE_CHARS/2);
//            for (i=0; i<BUFFER_ELEMENTS/2; i++) {
//                *dst++ = *src++;
//            }
            time2[j] = readl(TIMER2_VALUE);
        }
        read_ahb_monitors();
        for (j=0; j < ITERATIONS; j++) {
            printk("%u->%lu Bytes/s\n", time1[j]-time2[j], 100000UL * (BUFFER_SIZE_CHARS/2) / (time1[j]-time2[j]));
        }

        printk("\nINCR8 accesses:\n");
        init_ahb_monitors(AHB_MON_HWRITE_READS_AND_WRITES, ~0, AHB_MON_HBURST_INCR8, 0, 0);
        for (j=0; j < ITERATIONS; j++) {
            unsigned long* src = buf1;
            unsigned long* dst = buf2;
//            int i;

            time1[j] = readl(TIMER2_VALUE);
//            memcpy(dst, src, BUFFER_SIZE_CHARS/2);
            oxnas_asm_copy(dst, src, BUFFER_SIZE_CHARS/2);
//            for (i=0; i<BUFFER_ELEMENTS/2; i++) {
//                *dst++ = *src++;
//            }
            time2[j] = readl(TIMER2_VALUE);
        }
        read_ahb_monitors();
        for (j=0; j < ITERATIONS; j++) {
            printk("%u->%lu Bytes/s\n", time1[j]-time2[j], 100000UL * (BUFFER_SIZE_CHARS/2) / (time1[j]-time2[j]));
        }

        printk("\nWRAP16 accesses:\n");
        init_ahb_monitors(AHB_MON_HWRITE_READS_AND_WRITES, ~0, AHB_MON_HBURST_WRAP16, 0, 0);
        for (j=0; j < ITERATIONS; j++) {
            unsigned long* src = buf1;
            unsigned long* dst = buf2;
//            int i;

            time1[j] = readl(TIMER2_VALUE);
//            memcpy(dst, src, BUFFER_SIZE_CHARS/2);
            oxnas_asm_copy(dst, src, BUFFER_SIZE_CHARS/2);
//            for (i=0; i<BUFFER_ELEMENTS/2; i++) {
//                *dst++ = *src++;
//            }
            time2[j] = readl(TIMER2_VALUE);
        }
        read_ahb_monitors();
        for (j=0; j < ITERATIONS; j++) {
            printk("%u->%lu Bytes/s\n", time1[j]-time2[j], 100000UL * (BUFFER_SIZE_CHARS/2) / (time1[j]-time2[j]));
        }

        printk("\nINCR16 accesses:\n");
        init_ahb_monitors(AHB_MON_HWRITE_READS_AND_WRITES, ~0, AHB_MON_HBURST_INCR16, 0, 0);
        for (j=0; j < ITERATIONS; j++) {
            unsigned long* src = buf1;
            unsigned long* dst = buf2;
//            int i;

            time1[j] = readl(TIMER2_VALUE);
//            memcpy(dst, src, BUFFER_SIZE_CHARS/2);
            oxnas_asm_copy(dst, src, BUFFER_SIZE_CHARS/2);
//            for (i=0; i<BUFFER_ELEMENTS/2; i++) {
//                *dst++ = *src++;
//            }
            time2[j] = readl(TIMER2_VALUE);
        }
        read_ahb_monitors();
        for (j=0; j < ITERATIONS; j++) {
            printk("%u->%lu Bytes/s\n", time1[j]-time2[j], 100000UL * (BUFFER_SIZE_CHARS/2) / (time1[j]-time2[j]));
        }

        dma_free_coherent(0, BUFFER_SIZE_CHARS, buffer, dma_address);

        kfree(time1);
        kfree(time2);
    }

    buffer = kmalloc(BUFFER_SIZE_CHARS, GFP_KERNEL | GFP_DMA);
    if (!buffer) {
        printk(KERN_ERR "$RFailed to allocate cached memory test buffer\n");
    } else {
        static const int ITERATIONS = 100;

        unsigned long* buf1 = buffer;
        unsigned long* buf2 = buffer + (BUFFER_ELEMENTS/2);
        unsigned long* src = buf1;
        unsigned long* dst = buf2;
        int j;
        u32* time1 = (u32*)kmalloc(ITERATIONS *sizeof(u32), GFP_KERNEL);
        u32* time2 = (u32*)kmalloc(ITERATIONS *sizeof(u32), GFP_KERNEL);

        BUG_ON(!time1 || !time2);

        printk("Cached/: src = 0x%08x, dst = 0x%08x, length = %u, elements = %u\n", (u32)buf1, (u32)buf2, BUFFER_SIZE_CHARS/2, BUFFER_ELEMENTS/2);

        init_ahb_monitors(AHB_MON_HWRITE_READS_AND_WRITES, 0, 0, 0, 0);

        // Measure the first cached iteration separately
        printk("1st iteration:\n");
        restart_ahb_monitors();
        time1[0] = readl(TIMER2_VALUE);
//            memcpy(dst, src, BUFFER_SIZE_CHARS/2);
        oxnas_asm_copy(dst, src, BUFFER_SIZE_CHARS/2);
//            for (i=0; i<BUFFER_ELEMENTS/2; i++) {
//                *dst++ = *src++;
//            }
        time2[0] = readl(TIMER2_VALUE);
        read_ahb_monitors();
        printk("%u->%lu Bytes/s\n", time1[0]-time2[0], 100000UL * (BUFFER_SIZE_CHARS/2) / (time1[0]-time2[0]));

        printk("Subsequent iterations:\n");
        restart_ahb_monitors();
        for (j=1; j < ITERATIONS; j++) {
            src = buf1;
            dst = buf2;
//            int i;

            time1[j] = readl(TIMER2_VALUE);
//            memcpy(dst, src, BUFFER_SIZE_CHARS/2);
            oxnas_asm_copy(dst, src, BUFFER_SIZE_CHARS/2);
//            for (i=0; i<BUFFER_ELEMENTS/2; i++) {
//                *dst++ = *src++;
//            }
            time2[j] = readl(TIMER2_VALUE);
        }
        read_ahb_monitors();

        for (j=1; j < ITERATIONS; j++) {
            printk("%u->%lu Bytes/s\n", time1[j]-time2[j], 100000UL * (BUFFER_SIZE_CHARS/2) / (time1[j]-time2[j]));
        }

        kfree(time1);
        kfree(time2);

        kfree(buffer);
    }
}
#endif // CONFIG_DO_MEM_TEST

#ifdef CONFIG_OXNAS_LED_TEST

#define LED_D1  (1UL << 6)
#define LED_D2  (1UL << 7)
#define LED_D3  (1UL << 13)
#define LED_D4  (1UL << 14)
#define LED_D5  (1UL << 19)
#define LED_D6  (1UL << 21)
#define LED_D7  (1UL << 25)
#define LED_D8  (1UL << 26)
#define LED_D9  (1UL << 27)
#define FIRST_LEDS_MASK (LED_D1 | LED_D2 | LED_D3 | LED_D4 | LED_D5 | LED_D6 | LED_D7 | LED_D8 | LED_D9)

#define LED_D10 (1UL << 1)
#define SECOND_LEDS_MASK (LED_D10)

#define PWM_MASK (1UL << 8)

static void test_leds_and_pwm(void)
{
    // Disable primary, secondary and teriary GPIO functions for first nine LEDS
    writel(readl(SYS_CTRL_GPIO_PRIMSEL_CTRL_0) & ~FIRST_LEDS_MASK, SYS_CTRL_GPIO_PRIMSEL_CTRL_0);
    writel(readl(SYS_CTRL_GPIO_SECSEL_CTRL_0)  & ~FIRST_LEDS_MASK, SYS_CTRL_GPIO_SECSEL_CTRL_0);
    writel(readl(SYS_CTRL_GPIO_TERTSEL_CTRL_0) & ~FIRST_LEDS_MASK, SYS_CTRL_GPIO_TERTSEL_CTRL_0);

    // Disable primary, secondary and teriary GPIO functions for last LED
    writel(readl(SYS_CTRL_GPIO_PRIMSEL_CTRL_1) & ~SECOND_LEDS_MASK, SYS_CTRL_GPIO_PRIMSEL_CTRL_1);
    writel(readl(SYS_CTRL_GPIO_SECSEL_CTRL_1)  & ~SECOND_LEDS_MASK, SYS_CTRL_GPIO_SECSEL_CTRL_1);
    writel(readl(SYS_CTRL_GPIO_TERTSEL_CTRL_1) & ~SECOND_LEDS_MASK, SYS_CTRL_GPIO_TERTSEL_CTRL_1);

    // Turn off first nine LEDs
    writel(FIRST_LEDS_MASK, GPIO_A_OUTPUT_SET);

    // Turn off tenth LED
    writel(SECOND_LEDS_MASK, GPIO_B_OUTPUT_SET);

    // Enable first nine LEDs as outputs
    writel(FIRST_LEDS_MASK, GPIO_A_OUTPUT_ENABLE_SET);

    // Enable tenth LED as output
    writel(SECOND_LEDS_MASK, GPIO_B_OUTPUT_ENABLE_SET);

    // Turn on first nine LEDs sequentially
    mdelay(1000);
    writel(LED_D1, GPIO_A_OUTPUT_CLEAR);
    mdelay(1000);
    writel(LED_D2, GPIO_A_OUTPUT_CLEAR);
    mdelay(1000);
    writel(LED_D3, GPIO_A_OUTPUT_CLEAR);
    mdelay(1000);
    writel(LED_D4, GPIO_A_OUTPUT_CLEAR);
    mdelay(1000);
    writel(LED_D5, GPIO_A_OUTPUT_CLEAR);
    mdelay(1000);
    writel(LED_D6, GPIO_A_OUTPUT_CLEAR);
    mdelay(1000);
    writel(LED_D7, GPIO_A_OUTPUT_CLEAR);
    mdelay(1000);
    writel(LED_D8, GPIO_A_OUTPUT_CLEAR);
    mdelay(1000);
    writel(LED_D9, GPIO_A_OUTPUT_CLEAR);

    // Turn on tenth LED
    mdelay(1000);
    writel(LED_D10, GPIO_B_OUTPUT_CLEAR);

    // Disable primary, secondary and teriary GPIO functions for PWN line
    writel(readl(SYS_CTRL_GPIO_PRIMSEL_CTRL_0) & ~PWM_MASK, SYS_CTRL_GPIO_PRIMSEL_CTRL_0);
    writel(readl(SYS_CTRL_GPIO_SECSEL_CTRL_0)  & ~PWM_MASK, SYS_CTRL_GPIO_SECSEL_CTRL_0);
    writel(readl(SYS_CTRL_GPIO_TERTSEL_CTRL_0) & ~PWM_MASK, SYS_CTRL_GPIO_TERTSEL_CTRL_0);

    // Turn off PWM line
    writel(PWM_MASK, GPIO_A_OUTPUT_SET);

    // Enable PWM line as output
    writel(PWM_MASK, GPIO_A_OUTPUT_ENABLE_SET);

    // Turn on PWM line
    mdelay(1000);
    writel(PWM_MASK, GPIO_A_OUTPUT_CLEAR);
}
#endif // CONFIG_OXNAS_LED_TEST

static void __init oxnas_init_machine(void)
{
    /* Initialise the spinlock used to make GPIO register set access atomic */
    spin_lock_init(&oxnas_gpio_spinlock);

    /*
     * Initialise the support for our multi-channel memory-to-memory DMAC
     * The interrupt subsystem needs to be available before we can initialise
     * the DMAC support
     */
    oxnas_dma_init();

#ifdef CONFIG_DO_MEM_TEST
    /*
     * Do memory performance test
     */
    oxnas_mem_test();
#endif // CONFIG_DO_MEM_TEST

#ifdef CONFIG_LEON_START_EARLY
    init_copro(leon_early_srec, 0);
#endif // CONFIG_LEON_START_EARLY

#ifdef CONFIG_OXNAS_LED_TEST
    test_leds_and_pwm();
#endif // CONFIG_OXNAS_LED_TEST

	// Add any platform bus devices
	platform_add_devices(platform_devices, ARRAY_SIZE(platform_devices));
}

void sata_power_off(void)
{
#ifdef CONFIG_OXNAS_SATA_POWER_1
    // Disable power to SATA 1
    printk(KERN_INFO "Turning off disk 1\n");
    writel(SATA_POWER_1_MASK, SATA_POWER_1_OUTPUT_CLR_REG);
#endif // CONFIG_OXNAS_SATA_POWER_1

#ifdef CONFIG_OXNAS_SATA_POWER_2
    // Disable power to SATA 2
    printk(KERN_INFO "Turning off disk 2\n");
    writel(SATA_POWER_2_MASK, SATA_POWER_2_OUTPUT_CLR_REG);
#endif // CONFIG_OXNAS_SATA_POWER_2
}

/*
 * Code to setup the interrupts
 */
static void __init oxnas_init_irq(void)
{
    /* initialise the RPS interrupt controller */
    OX820_RPS_init_irq(OX820_RPS_IRQ_START, OX820_RPS_IRQ_START + NR_RPS_IRQS);

    /* initialise the GIC */
	gic_cpu_base_addr = __io_address(OX820_GIC_CPU_BASE_ADDR);
#if 0
    gic_legacy_mode(1, __io_address(OX820_GIC_CPU_BASE_ADDR));
#else
    /* @debug don't know why this has and offset of 29, just copying realview */
	gic_dist_init(0, __io_address(OX820_GIC_DIST_BASE_ADDR), 29);
	gic_cpu_init(0, gic_cpu_base_addr);
    OX820_RPS_cascade_irq( IRQ_INTERRUPT);
#endif
}



MACHINE_START(OXNAS, "Oxsemi NAS")
    /* Maintainer: Oxford Semiconductor Ltd */
#ifdef CONFIG_ARCH_OXNAS_UART1
    .phys_io = UART_1_BASE_PA,
    .io_pg_offst = (((u32)UART_1_BASE) >> 18) & 0xfffc,
#elif defined(CONFIG_ARCH_OXNAS_UART2)
    .phys_io = UART_2_BASE_PA,
    .io_pg_offst = (((u32)UART_2_BASE) >> 18) & 0xfffc,
#elif defined(CONFIG_ARCH_OXNAS_UART3)
    .phys_io = UART_3_BASE_PA,
    .io_pg_offst = (((u32)UART_3_BASE) >> 18) & 0xfffc,
#elif defined(CONFIG_ARCH_OXNAS_UART4)
    .phys_io = UART_4_BASE_PA,
    .io_pg_offst = (((u32)UART_4_BASE) >> 18) & 0xfffc,
#endif
    .boot_params = SDRAM_PA + 0x100,
    .fixup = oxnas_fixup,
    .map_io = oxnas_mapio,
    .init_irq = oxnas_init_irq,
    .timer = &oxnas_timer,
    .init_machine = oxnas_init_machine,
MACHINE_END

