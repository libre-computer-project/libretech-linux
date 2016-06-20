/*
 *  arch/arm/mach-oxnas/pci.c
 *
 *  Copyright (C) 2006 Oxford Semiconductor Ltd
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
 *
 */
#include <linux/kernel.h>

#include <linux/pci.h>
#include <linux/ptrace.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/mach/pci.h>
#include <asm/mach-types.h>

#ifndef CONFIG_PCI

inline void outb(unsigned char  v, u32 p)	{ *((volatile  u8*)(__io(p))) = (v);   	 	}
inline void outw(unsigned short v, u32 p)	{ *((volatile u16*)(__io(p))) = cpu_to_le16(v); }
inline void outl(unsigned long  v, u32 p)	{ *((volatile u32*)(__io(p))) = cpu_to_le32(v); }

inline unsigned char   inb(u32 p)		{ return            (*((volatile  u8*)(__io(p)))); }
inline unsigned short  inw(u32 p)		{ return le16_to_cpu(*((volatile u16*)(__io(p)))); }
inline unsigned long   inl(u32 p)		{ return le32_to_cpu(*((volatile u32*)(__io(p)))); }

inline void outsb(u32 p, unsigned char  * from, u32 len)	{ while (len--) { outb((*from++),(p) ); } }
inline void outsw(u32 p, unsigned short * from, u32 len)	{ while (len--) { outw((*from++),(p) ); } }
inline void outsl(u32 p, unsigned long  * from, u32 len)	{ while (len--) { outl((*from++),(p) ); } }
                                         
inline void insb(u32 p, unsigned char  * to, u32 len)	{ while (len--) { *to++ = inb(p); } }
inline void insw(u32 p, unsigned short * to, u32 len)	{ while (len--) { *to++ = inw(p); } }
inline void insl(u32 p, unsigned long  * to, u32 len)	{ while (len--) { *to++ = inl(p); } }

EXPORT_SYMBOL( inb );
EXPORT_SYMBOL( inw );
EXPORT_SYMBOL( inl );

EXPORT_SYMBOL( outb );
EXPORT_SYMBOL( outw );
EXPORT_SYMBOL( outl );

EXPORT_SYMBOL( insb );
EXPORT_SYMBOL( insw );
EXPORT_SYMBOL( insl );

EXPORT_SYMBOL( outsb );
EXPORT_SYMBOL( outsw );
EXPORT_SYMBOL( outsl );

#else // #ifdef CONFIG_PCI

extern spinlock_t oxnas_gpio_spinlock;

#define PCI_BUS_NONMEM_START			0x00000000
#define PCI_BUS_NONMEM_SIZE	    		0x00080000
                               
                               
#define PCI_BUS_PREMEM_START			PCI_BUS_NONMEM_START + PCI_BUS_NONMEM_SIZE
#define PCI_BUS_PREMEM_SIZE	    		0x00080000

#define SYNOPSYS_PCI_MEMORY_BASE_ADDRESS        PCI_BASE_ADDRESS_0
#define SYNOPSYS_PCI_DUAL_CYCLE_BASE_ADDRESS    PCI_BASE_ADDRESS_2
#define SYNOPSYS_PCI_IO_MEM_BASE_ADDRESS        PCI_BASE_ADDRESS_1  // PLEASE NOTE - THESE ARE INCORRECT IN THE DOCUMENT!!


inline void outb(unsigned char  v, u32 p)	{ pciio_write(v,(u32)__io(p),             sizeof(char ) );  }
inline void outw(unsigned short v, u32 p)	{ pciio_write(cpu_to_le16(v),(u32)__io(p),sizeof(short) );  }
inline void outl(unsigned long  v, u32 p)	{ pciio_write(cpu_to_le32(v),(u32)__io(p),sizeof(long ) );  }

inline unsigned char   inb(u32 p)		{ unsigned int __v =            (pciio_read((u32)__io(p),sizeof(char ))); return __v; }
inline unsigned short  inw(u32 p)		{ unsigned int __v = le16_to_cpu(pciio_read((u32)__io(p),sizeof(short))); return __v; }
inline unsigned long   inl(u32 p)		{ unsigned int __v = le32_to_cpu(pciio_read((u32)__io(p),sizeof(long ))); return __v; }

inline void outsb(volatile u32 p, unsigned char  * from, u32 len)	{ while (len--) { pciio_write(	         (*from++),(u32)__io(p),sizeof(char ) ); } }
inline void outsw(volatile u32 p, unsigned short * from, u32 len)	{ while (len--) { pciio_write(cpu_to_le16(*from++),(u32)__io(p),sizeof(short) ); } }
inline void outsl(volatile u32 p, unsigned long  * from, u32 len)	{ while (len--) { pciio_write(cpu_to_le32(*from++),(u32)__io(p),sizeof(long ) ); } }
                                         
inline void insb(volatile u32 p, unsigned char  * to, u32 len)	{ while (len--) { *to++ =            (pciio_read((u32)__io(p),sizeof(char ))); } }
inline void insw(volatile u32 p, unsigned short * to, u32 len)	{ while (len--) { *to++ = le16_to_cpu(pciio_read((u32)__io(p),sizeof(short))); } }
inline void insl(volatile u32 p, unsigned long  * to, u32 len)	{ while (len--) { *to++ = le32_to_cpu(pciio_read((u32)__io(p),sizeof(long ))); } }

EXPORT_SYMBOL( inb );
EXPORT_SYMBOL( inw );
EXPORT_SYMBOL( inl );

EXPORT_SYMBOL( outb );
EXPORT_SYMBOL( outw );
EXPORT_SYMBOL( outl );

EXPORT_SYMBOL( insb );
EXPORT_SYMBOL( insw );
EXPORT_SYMBOL( insl );

EXPORT_SYMBOL( outsb );
EXPORT_SYMBOL( outsw );
EXPORT_SYMBOL( outsl );

EXPORT_SYMBOL( pciio_read );
EXPORT_SYMBOL( pciio_write );

static spinlock_t oxnas_lock = SPIN_LOCK_UNLOCKED;

//static int oxnas_pci_read_core_config( unsigned int config_register )
//{
//	unsigned long val, flags;
//	//printk(KERN_DEBUG "PCI: oxnas_pci_read_core_config( 0x%x )\n", config_register );
//	spin_lock_irqsave(&oxnas_lock, flags);
//	writel(
//		0x00                            << PCI_CRP_BYTE_ENABLES_START | 
//		PCI_BUS_CMD_CONFIGURATION_READ  << PCI_CRP_CMD_START          |
//		config_register                 << PCI_CRP_ADDRESS_START, 
//		PCI_CRP_CMD_AND_ADDR );
//    	wmb();
//	val = readl( PCI_CRP_READ_DATA );
//   	spin_unlock_irqrestore(&oxnas_lock, flags);
//	return val; 
//}


static void oxnas_pci_write_core_config( unsigned int value, unsigned int config_register )
{
	unsigned long flags;

	//printk(KERN_DEBUG "PCI: oxnas_pci_write_core_config( 0x%x, 0x%x )\n", config_register , value);
	/* printk(KERN_DEBUG "PCI: writel( 0x%lx, 0x%lx )\n", 
		0x00                            << PCI_CRP_BYTE_ENABLES_START | 
		PCI_BUS_CMD_CONFIGURATION_WRITE << PCI_CRP_CMD_START          |
		config_register                 << PCI_CRP_ADDRESS_START,
		PCI_CRP_CMD_AND_ADDR ); */
    
	spin_lock_irqsave(&oxnas_lock, flags);
	writel(
		0x00                            << PCI_CRP_BYTE_ENABLES_START | 
		PCI_BUS_CMD_CONFIGURATION_WRITE << PCI_CRP_CMD_START          |
		config_register                 << PCI_CRP_ADDRESS_START,
		PCI_CRP_CMD_AND_ADDR );
	wmb();

	// printk(KERN_DEBUG "PCI: writel( 0x%lx, 0x%lx )\n", value, PCI_CRP_WRITE_DATA ); 
	writel( value, PCI_CRP_WRITE_DATA ); 
	spin_unlock_irqrestore(&oxnas_lock, flags);    
}



inline unsigned int CheckAndClearBusError(void)
{
    unsigned int value = readl( PCI_ERROR_MSG ) & 0x00000003;
    if ( value )
    {
//        printk(KERN_DEBUG "PCI: %s ERROR ON PCI BUS Clearing error\n", value & 0x00000001 ? "FATAL" : "PARITY" );
        writel( ( value ), PCI_ERROR_MSG );
    }
    
    return value;    
}


void pciio_write(unsigned int  data, u32 addr, unsigned int size)
{
	// setup byte enables
	unsigned long flags;
	unsigned int be     = 0x0000000f >> (4-size);
	unsigned int trunc  = (addr & 0x00000003);
	be                <<= trunc;
	be	            = (~be) & 0x00000000f;
	

	data  &= (0xffffffff >> ((4-size)*8));
	data <<= (trunc*8);

	 //printk(KERN_DEBUG "$YPCI: pciio_write( 0x%08x = 0x%08x (%x:%x) )\n", addr, data, size, be);
	
	/* Setup the io read address (rounded down to word boundry) */
	spin_lock_irqsave(&oxnas_lock, flags);
	writel( addr , PCI_CONFIG_IO_CYCLE_ADDR );
	wmb();
	
	/* issue the config io read command to the config io cmd reg */
	writel( ( be                     << PCI_CONFIG_IO_BYTE_ENABLES_START) | 
	        ( PCI_BUS_CMD_IO_WRITE   << PCI_CONFIG_IO_CMD_START ), 
	          PCI_CONFIG_IO_BYTE_CMD );
	wmb();
	
	writel( data, PCI_CONFIG_IO_WRITE_DATA );	
	
	if ( CheckAndClearBusError() )
	{
	    printk(KERN_DEBUG "PCI: failed to write io\n");
	}
	spin_unlock_irqrestore(&oxnas_lock, flags);
}
	

unsigned int pciio_read(u32 addr, unsigned int size)
{
	// setup byte enables
	unsigned long flags;
	unsigned int be     = 0x0000000f >> (4-size);
	unsigned int trunc  = (addr & 0x00000003);
	be                <<= trunc;
	be	            = (~be) & 0x00000000f;
	
		
	//printk(KERN_DEBUG "$YPCI: pciio_read[ 0x%x ] ( 0x%x == ", size, addr );
	
	/* Setup the io read address (rounded down to word boundry) */
	spin_lock_irqsave(&oxnas_lock, flags);
	writel( addr, PCI_CONFIG_IO_CYCLE_ADDR );
	wmb();
	
	/* issue the config io read command to the config io cmd reg */
	writel( ( be                     << PCI_CONFIG_IO_BYTE_ENABLES_START) | 
	        ( PCI_BUS_CMD_IO_READ    << PCI_CONFIG_IO_CMD_START ), 
	          PCI_CONFIG_IO_BYTE_CMD );
	wmb();
	
	if ( CheckAndClearBusError() )
	{
	    printk(KERN_DEBUG "PCI: failed to read io\n");
	}
	
	be   = readl( PCI_CONFIG_IO_READ_DATA );	
	//printk("0x%x )\n", be);
	
	if ( CheckAndClearBusError() )
	{
	    printk(KERN_DEBUG "PCI: failed to read io\n");
	}
	
	spin_unlock_irqrestore(&oxnas_lock, flags);
	
	be >>= (trunc*8);
	be  &= (0xffffffff >> ((4-size)*8));
	
	
	return be;	
}


static int oxnas_read_config(struct pci_bus *bus, unsigned int devfn, int where,
			  int size, u32 *value)
{
	// unsigned long flags;
	unsigned long flags;
	unsigned int temp;
	unsigned long addr = ( 0x00000800      << (PCI_SLOT(devfn)-1) )  | 
        			     ( PCI_FUNC(devfn) << 8 )                    |  
			     ( where & 0xfc );
    
	/* Setup the config io read address (rounded down to word boundry) */
	temp = addr;

        // printk(KERN_DEBUG "PCI: %s::%u oxnas_read_config( %u, %d, %d, )\n", bus->name, bus->number, devfn, where, size );
	spin_lock_irqsave(&oxnas_lock, flags);
	CheckAndClearBusError();

	// printk(KERN_DEBUG "PCI: writel( 0x%08lx, 0x%08lx)\n", temp, PCI_CONFIG_IO_CYCLE_ADDR );
	writel( temp, PCI_CONFIG_IO_CYCLE_ADDR );
	wmb();
    
	/* issue the config io read command to the config io cmd reg */
	temp = ( ( 0x00                           << PCI_CONFIG_IO_BYTE_ENABLES_START) | 
		 ( PCI_BUS_CMD_CONFIGURATION_READ << PCI_CONFIG_IO_CMD_START ) ); 
	// printk(KERN_DEBUG "PCI: writel( 0x%08lx, 0x%08lx)\n", temp, PCI_CONFIG_IO_BYTE_CMD);
	writel( temp, PCI_CONFIG_IO_BYTE_CMD );
	wmb();

	if ( CheckAndClearBusError() )
	{
		spin_unlock_irqrestore(&oxnas_lock, flags);
//		printk(KERN_DEBUG "PCI: failed to read config\n" );
		*value = 0xffffffff;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}
    
	wmb();
	*value=readl(PCI_CONFIG_IO_READ_DATA);
	spin_unlock_irqrestore(&oxnas_lock, flags);
    
	/* Read the result from the config io read data reg */
	switch (size) {
	case 1:
		// printk(KERN_DEBUG "PCI: readb( 0x%lx )\n", PCI_CONFIG_IO_READ_DATA );
		*value>>=(where&3);
		*value&=0x000000ff;
		break;
	case 2:
		// printk(KERN_DEBUG "PCI: readw( 0x%lx )\n", PCI_CONFIG_IO_READ_DATA );
		*value>>=(where&2);
		*value&=0x0000ffff;
		break;
	case 4:
		// printk(KERN_DEBUG "PCI: readl( 0x%lx )\n", PCI_CONFIG_IO_READ_DATA );
		break;
	}
	// printk(KERN_DEBUG "PCI: $Goxnas_read_config_%s( 0x%lx ) == 0x%lx\n", 
	//       size == 1 ? "byte" : size == 2 ? "short" : "word",
	//        (unsigned long) addr,
	//        (unsigned long) *value);
    
	return PCIBIOS_SUCCESSFUL;
}

static int oxnas_write_config(struct pci_bus *bus, unsigned int devfn, int where,
			   int size, u32 value)
{   
	unsigned long flags;
	unsigned long byteEnables = ~(( 0xffffffff >> (32 - size) ) << (where&3));
	unsigned long addr        =   ( 0x00000800      << (PCI_SLOT(devfn)-1) )  | 
				      ( PCI_FUNC(devfn) << 8 )                    |  
				      ( where & 0xfc );
	value                   <<= 8*(where & 0x00000003);
    
    	// printk(KERN_DEBUG "$GPCI: %s::%u oxnas_write_config_%s( 0x%lx, 0x%lx & 0x%lx)\n",
	// 	bus->name,
	// 	bus->number,
	// 	size == 1 ? "byte" : size == 2 ? "short" : "word",
	// 	(unsigned long) addr,
	// 	(unsigned long) value, 
	// 	(unsigned long) byteEnables );

    	if ( PCI_SLOT(devfn) > 15 )
    	{
	    	/* only 16 devices supported */
		return PCIBIOS_DEVICE_NOT_FOUND;
    	}
    
        
	spin_lock_irqsave(&oxnas_lock, flags);
	CheckAndClearBusError();
    
	/* Setup the config io read address (rounded down to word boundry) */
	// printk(KERN_DEBUG "PCI: writel( 0x%lx, 0x%lx)\n", addr, PCI_CONFIG_IO_CYCLE_ADDR );
	writel( addr, PCI_CONFIG_IO_CYCLE_ADDR );
	wmb();

	/* issue the config io read command to the config io cmd reg */
	// printk(KERN_DEBUG "PCI: writel( 0x%lx, 0x%lx )\n", 
	//         ( (byteEnables & 0xf)             << PCI_CONFIG_IO_BYTE_ENABLES_START) | 
	//         ( PCI_BUS_CMD_CONFIGURATION_WRITE << PCI_CONFIG_IO_CMD_START ),
	//           PCI_CONFIG_IO_BYTE_CMD );
              
	writel( 	( (byteEnables & 0xf)             << PCI_CONFIG_IO_BYTE_ENABLES_START) | 
		( PCI_BUS_CMD_CONFIGURATION_WRITE << PCI_CONFIG_IO_CMD_START ),
		  PCI_CONFIG_IO_BYTE_CMD );
	wmb();
        
	/* write the value... */
	// printk(KERN_DEBUG "PCI: writel( 0x%lx, 0x%lx )\n",  value, PCI_CONFIG_IO_WRITE_DATA );
	writel( value, PCI_CONFIG_IO_WRITE_DATA );
	wmb();
    
	if ( CheckAndClearBusError() )
	{
		printk(KERN_DEBUG "PCI: failed to write config\n");
		return PCIBIOS_DEVICE_NOT_FOUND;
	}
    
	spin_unlock_irqrestore(&oxnas_lock, flags);
	return PCIBIOS_SUCCESSFUL;
}


// #if PCI_BUS_NONMEM_START & 0x000fffff
// #error PCI_BUS_NONMEM_START must be megabyte aligned
// #endif
// #if PCI_BUS_PREMEM_START & 0x000fffff
// #error PCI_BUS_PREMEM_START must be megabyte aligned
// #endif
// 

static struct resource io_mem = {
	.name	= "PCI I/O Space",
	.start	= 0x00001000,
	.end	= 0xffff0000,
	.flags	= IORESOURCE_IO,
};

static struct resource non_mem = {
	.name	= "PCI non-prefetchable",
	.start	= PCI_BASE_PA + PCI_BUS_NONMEM_START,
	.end	= PCI_BASE_PA + PCI_BUS_NONMEM_START + PCI_BUS_NONMEM_SIZE - 1,
	.flags	= IORESOURCE_MEM,
};

static struct resource pre_mem = {
	.name	= "PCI prefetchable",
	.start	= PCI_BASE_PA + PCI_BUS_PREMEM_START,
	.end	= PCI_BASE_PA + PCI_BUS_PREMEM_START + PCI_BUS_PREMEM_SIZE - 1,
	.flags	= IORESOURCE_MEM | IORESOURCE_PREFETCH,
};


/*
 * This routine handles multiple bridges.
 */
static u8 __init oxnas_swizzle(struct pci_dev *dev, u8 *pinp)
{
//	printk(KERN_DEBUG "PCI: oxnas_swizzle\n");
	return pci_std_swizzle(dev, pinp);
}


// static int irq_tab[4] __initdata = {
//  	IRQ_AP_PCIINT0,	IRQ_AP_PCIINT1,	IRQ_AP_PCIINT2,	IRQ_AP_PCIINT3
// };


/*
 * map the specified device/slot/pin to an IRQ.  This works out such
 * that ..
 */
static int __init oxnas_map_irq(struct pci_dev *dev, u8 slot, u8 pin)
{
	BUG_ON(pin < 1 || pin > 4);

//	printk(KERN_DEBUG "PCI: oxnas_map_irq %d,%d,%d = %d\n", dev->bus->number, dev->devfn, slot, PCI_A_INTERRUPT /*pci_irq_table[pin-1]*/ );
	return PCI_A_INTERRUPT;
}


static int __init  oxnas_pci_setup_resources(struct resource **resource)
{	
	/*
	 * bus->resource[0] is the IO resource for this bus
	 * bus->resource[1] is the mem resource for this bus
	 * bus->resource[2] is the prefetch mem resource for this bus
	 */
    
	resource[0] = &io_mem;
	resource[1] = &pre_mem;
	resource[2] = &non_mem;
    
	// these regions apply to incomming transactions on PCI
    oxnas_pci_write_core_config( 0xffffffff ,  SYNOPSYS_PCI_MEMORY_BASE_ADDRESS      );
    oxnas_pci_write_core_config( 0xffffffff ,  SYNOPSYS_PCI_DUAL_CYCLE_BASE_ADDRESS  );
    oxnas_pci_write_core_config( 0xffffffff ,  SYNOPSYS_PCI_IO_MEM_BASE_ADDRESS      );
    	
//    printk(KERN_DEBUG "PCI:  SYNOPSYS_PCI_MEMORY_BASE_ADDRESS     $YWindow Size == 0x%08x\n", (u32) oxnas_pci_read_core_config(SYNOPSYS_PCI_MEMORY_BASE_ADDRESS     ) );
//    printk(KERN_DEBUG "PCI:  SYNOPSYS_PCI_DUAL_CYCLE_BASE_ADDRESS $YWindow Size == 0x%08x\n", (u32) oxnas_pci_read_core_config(SYNOPSYS_PCI_DUAL_CYCLE_BASE_ADDRESS ) );
//    printk(KERN_DEBUG "PCI:  SYNOPSYS_PCI_IO_MEM_BASE_ADDRESS     $YWindow Size == 0x%08x\n", (u32) oxnas_pci_read_core_config(SYNOPSYS_PCI_IO_MEM_BASE_ADDRESS     ) );

    oxnas_pci_write_core_config( SDRAM_PA   ,  SYNOPSYS_PCI_MEMORY_BASE_ADDRESS      );
    oxnas_pci_write_core_config( SDRAM_PA   ,  SYNOPSYS_PCI_DUAL_CYCLE_BASE_ADDRESS  );
    oxnas_pci_write_core_config( SDRAM_PA   ,  SYNOPSYS_PCI_IO_MEM_BASE_ADDRESS      );
    	
//	printk(KERN_DEBUG "PCI:  SYNOPSYS_PCI_MEMORY_BASE_ADDRESS     == 0x%08x\n", (u32) oxnas_pci_read_core_config(SYNOPSYS_PCI_MEMORY_BASE_ADDRESS     ) );
//	printk(KERN_DEBUG "PCI:  SYNOPSYS_PCI_DUAL_CYCLE_BASE_ADDRESS == 0x%08x\n", (u32) oxnas_pci_read_core_config(SYNOPSYS_PCI_DUAL_CYCLE_BASE_ADDRESS ) );
//	printk(KERN_DEBUG "PCI:  SYNOPSYS_PCI_IO_MEM_BASE_ADDRESS     == 0x%08x\n", (u32) oxnas_pci_read_core_config(SYNOPSYS_PCI_IO_MEM_BASE_ADDRESS     ) );
	return 1;
}


int __init oxnas_pci_setup(int nr, struct pci_sys_data *sys)
{
	int ret = 0;

//	printk(KERN_DEBUG "PCI: oxnas_pci_setup nr == %u\n", nr);
	if (nr == 0) {
        /* the PCI core has been setup so that the top nybble is forced to 0, so 
        we need to offset by whatever is in the top nybble or the devices won't 
        recognise their memory accesses */ 
        sys->mem_offset = PCI_BASE_PA & 0xf0000000 ; 
    
		// ioremap is not called on IO ports. this should shift the physical 
		// address to the statically mapped virtual one after the BARS have been
		// setup.
		sys->io_offset  = 0;

		spin_lock_init(&oxnas_lock);
		ret = oxnas_pci_setup_resources(sys->resource);
	}

	return ret;
}


static struct pci_ops oxnas_pci_ops = {
	.read	= oxnas_read_config,
	.write	= oxnas_write_config,
};


struct pci_bus *oxnas_pci_scan_bus(int nr, struct pci_sys_data *sys)
{
//	printk(KERN_DEBUG "PCI: oxnas_pci_scan_bus\n");
	return pci_scan_bus(sys->busnr, &oxnas_pci_ops, sys);
}

void __init oxnas_pci_preinit(void)
{
	unsigned int temp; 
	unsigned long flags;
//	printk(KERN_DEBUG "PCI: oxnas_pci_preinit\n");

    // Configure GPIO lines which map PCI INTA for both minipci and planar as active low
    spin_lock_irqsave(&oxnas_gpio_spinlock, flags);
	*((volatile unsigned long*)GPIO_A_FALLING_EDGE_ACTIVE_LOW_ENABLE) |= ((1UL <<  PCI_GPIO_INTA_MINIPCI) | (1UL <<  PCI_GPIO_INTA_PLANAR));
	*((volatile unsigned long*)GPIO_A_LEVEL_INTERRUPT_ENABLE) |= ((1UL <<  PCI_GPIO_INTA_MINIPCI) | (1UL <<  PCI_GPIO_INTA_PLANAR));
    spin_unlock_irqrestore(&oxnas_gpio_spinlock, flags);

	/*
	printk(KERN_DEBUG "\n\nPCI: GPIO ABse 0x%08x\n", GPIO_1_BASE );
	for ( temp=0;temp<0x40; temp += 4 )
	{
		printk(KERN_DEBUG "       GPIO ABse + 0x%02x == 0x%08x\n",temp, readl( GPIO_1_BASE+temp ) );
	}
	*/

	// put pci into host mode
	temp = 	( 0 << SYSCTL_PCI_CTRL1_PULL_UP_ENABLE_GPIO5 ) |
		    ( 0 << SYSCTL_PCI_CTRL1_PULL_UP_ENABLE_GPIO4 ) |
		    ( 0 << SYSCTL_PCI_CTRL1_PULL_UP_ENABLE_GPIO3 ) |
		    ( 0 << SYSCTL_PCI_CTRL1_PULL_UP_ENABLE_GPIO2 ) |
		    ( 0 << SYSCTL_PCI_CTRL1_PULL_UP_ENABLE_GPIO1 ) |
		    ( 0 << SYSCTL_PCI_CTRL1_PULL_UP_ENABLE_GPIO0 ) |
		    ( 0 << SYSCTL_PCI_CTRL1_ENPU                 ) |
		    ( 0 << SYSCTL_PCI_CTRL1_ENCB                 ) |
		    ( 0 << SYSCTL_PCI_CTRL1_SYSPCI_STATIC_REQ    ) |
		    ( 1 << SYSCTL_PCI_CTRL1_SS_HOST_E            ) |
		    ( 0 << SYSCTL_PCI_CTRL1_SYSPCI_PAKING_ENABLE ) |
		    ( 0 << SYSCTL_PCI_CTRL1_SYSPCI_PAKING_MASTE  ) |
		    ( 0 << SYSCTL_PCI_CTRL1_SS_CADBUS_E          ) |
		    ( 0 << SYSCTL_PCI_CTRL1_SS_MINIPCI_          ) |
		    ( 0 << SYSCTL_PCI_CTRL1_SS_INT_MASK_0        ) |
		    ( 0 << SYSCTL_PCI_CTRL1_INT_STATUS_0         ) |
		    ( 0 << SYSCTL_PCI_CTRL1_APP_EQUIES_NOM_CLK   ) |
		    ( 0 << SYSCTL_PCI_CTRL1_APP_CBUS_INT_N       ) |
		    ( 0 << SYSCTL_PCI_CTRL1_APP_CSTSCHG_N        );

//	printk(KERN_DEBUG "PCI: pci into host mode - writel( 0x%08x, 0x%08x )\n", (u32) temp, (u32) (SYS_CTRL_PCI_CTRL1) );
	writel( temp, SYS_CTRL_PCI_CTRL1 );

	/* the interrupt lines map directly to the GPIO lines, so disable any 
    primary, secondary and tertiary functionality */ 
    spin_lock_irqsave(&oxnas_gpio_spinlock, flags);

    // Interrupt line the cardbus/mini-PCI slot
	*((volatile unsigned long*)SYS_CTRL_GPIO_PRIMSEL_CTRL_0) &= ~(1UL << PCI_GPIO_INTA_MINIPCI);
	*((volatile unsigned long*)SYS_CTRL_GPIO_SECSEL_CTRL_0)  &= ~(1UL << PCI_GPIO_INTA_MINIPCI);
	*((volatile unsigned long*)SYS_CTRL_GPIO_TERTSEL_CTRL_0) &= ~(1UL << PCI_GPIO_INTA_MINIPCI);

    // Interrupt line for VIA-SATA PCI device
	*((volatile unsigned long*)SYS_CTRL_GPIO_PRIMSEL_CTRL_0) &= ~(1UL << PCI_GPIO_INTA_PLANAR);
	*((volatile unsigned long*)SYS_CTRL_GPIO_SECSEL_CTRL_0)  &= ~(1UL << PCI_GPIO_INTA_PLANAR);
	*((volatile unsigned long*)SYS_CTRL_GPIO_TERTSEL_CTRL_0) &= ~(1UL << PCI_GPIO_INTA_PLANAR);

#ifdef CONFIG_ARCH_OXNAS_PCI_CLKOUT_0
	*((volatile unsigned long*)SYS_CTRL_GPIO_PRIMSEL_CTRL_0) |= (1UL << PCI_GPIO_CLKO_0);
	*((volatile unsigned long*)SYS_CTRL_GPIO_SECSEL_CTRL_0)  |= (1UL << PCI_GPIO_CLKO_0);
	*((volatile unsigned long*)SYS_CTRL_GPIO_TERTSEL_CTRL_0) |= (1UL << PCI_GPIO_CLKO_0);
#endif // CONFIG_ARCH_OXNAS_PCI_CLKOUT_0

#ifdef CONFIG_ARCH_OXNAS_PCI_CLKOUT_1
	*((volatile unsigned long*)SYS_CTRL_GPIO_PRIMSEL_CTRL_0) |= (1UL << PCI_GPIO_CLKO_1);
	*((volatile unsigned long*)SYS_CTRL_GPIO_SECSEL_CTRL_0)  |= (1UL << PCI_GPIO_CLKO_1);
	*((volatile unsigned long*)SYS_CTRL_GPIO_TERTSEL_CTRL_0) |= (1UL << PCI_GPIO_CLKO_1);
#endif // CONFIG_ARCH_OXNAS_PCI_CLKOUT_1

#ifdef CONFIG_ARCH_OXNAS_PCI_CLKOUT_2
	*((volatile unsigned long*)SYS_CTRL_GPIO_PRIMSEL_CTRL_0) |= (1UL << PCI_GPIO_CLKO_2);
	*((volatile unsigned long*)SYS_CTRL_GPIO_SECSEL_CTRL_0)  |= (1UL << PCI_GPIO_CLKO_2);
	*((volatile unsigned long*)SYS_CTRL_GPIO_TERTSEL_CTRL_0) |= (1UL << PCI_GPIO_CLKO_2);
#endif // CONFIG_ARCH_OXNAS_PCI_CLKOUT_2

#ifdef CONFIG_ARCH_OXNAS_PCI_CLKOUT_3
	*((volatile unsigned long*)SYS_CTRL_GPIO_PRIMSEL_CTRL_0) |= (1UL << PCI_GPIO_CLKO_3);
	*((volatile unsigned long*)SYS_CTRL_GPIO_SECSEL_CTRL_0)  |= (1UL << PCI_GPIO_CLKO_3);
	*((volatile unsigned long*)SYS_CTRL_GPIO_TERTSEL_CTRL_0) |= (1UL << PCI_GPIO_CLKO_3);
#endif // CONFIG_ARCH_OXNAS_PCI_CLKOUT_3

//	printk(KERN_DEBUG "PCI: set gnt and req functions for pci arbiters\n" );

#ifdef CONFIG_ARCH_OXNAS_PCI_REQGNT_0
	*((volatile unsigned long*) SYS_CTRL_GPIO_PRIMSEL_CTRL_0 ) |= (1UL << PCI_REQ_N0);
	*((volatile unsigned long*) SYS_CTRL_GPIO_PRIMSEL_CTRL_0 ) |= (1UL << PCI_GNT_N0);
#endif // CONFIG_ARCH_OXNAS_PCI_REQGNT_0

#ifdef CONFIG_ARCH_OXNAS_PCI_REQGNT_1
	*((volatile unsigned long*) SYS_CTRL_GPIO_PRIMSEL_CTRL_0 ) |= (1UL << PCI_REQ_N1);
	*((volatile unsigned long*) SYS_CTRL_GPIO_PRIMSEL_CTRL_0 ) |= (1UL << PCI_GNT_N1);
#endif // CONFIG_ARCH_OXNAS_PCI_REQGNT_1

#ifdef CONFIG_ARCH_OXNAS_PCI_REQGNT_2
	*((volatile unsigned long*) SYS_CTRL_GPIO_PRIMSEL_CTRL_0 ) |= (1UL << PCI_REQ_N2);
	*((volatile unsigned long*) SYS_CTRL_GPIO_PRIMSEL_CTRL_0 ) |= (1UL << PCI_GNT_N2);
#endif // CONFIG_ARCH_OXNAS_PCI_REQGNT_2

#ifdef CONFIG_ARCH_OXNAS_PCI_REQGNT_3
	*((volatile unsigned long*) SYS_CTRL_GPIO_PRIMSEL_CTRL_0 ) |= (1UL << PCI_REQ_N2);
	*((volatile unsigned long*) SYS_CTRL_GPIO_PRIMSEL_CTRL_0 ) |= (1UL << PCI_GNT_N2);
#endif // CONFIG_ARCH_OXNAS_PCI_REQGNT_3

    spin_unlock_irqrestore(&oxnas_gpio_spinlock, flags);

	// no eeprom to setup core, so perform eeporm functions --------------------
	// setup the data to write to enable pci config
//	printk(KERN_DEBUG "PCI: enable core features\n" );
	oxnas_pci_write_core_config( 
		PCI_COMMAND_IO              |
		PCI_COMMAND_MEMORY          |
		PCI_COMMAND_MASTER          |
		PCI_COMMAND_SPECIAL         |
		PCI_COMMAND_INVALIDATE      |
		PCI_COMMAND_VGA_PALETTE     |
		PCI_COMMAND_PARITY          |
		PCI_COMMAND_WAIT            | 
		PCI_COMMAND_SERR            |
		PCI_COMMAND_FAST_BACK /*    |       
		PCI_COMMAND_INTX_DISABLE, */, 
		PCI_COMMAND );    
		
//	printk(KERN_DEBUG "PCI:  PCI_COMMAND == 0x%08x\n", (u32) oxnas_pci_read_core_config(PCI_COMMAND) );
}

void __init oxnas_pci_postinit(void)
{
//	printk(KERN_DEBUG "PCI: oxnas_pci_postinit\n");
}

static struct hw_pci oxnas_pci __initdata = {
	.swizzle			= oxnas_swizzle,
	.map_irq			= oxnas_map_irq,
	.setup			= oxnas_pci_setup,
	.nr_controllers		= 1,
	.scan			= oxnas_pci_scan_bus,
	.preinit			= oxnas_pci_preinit,
	.postinit		= oxnas_pci_postinit,
};

static int __init oxnas_pci_init(void)
{
    pci_common_init(&oxnas_pci);
	return 0;
}

static void __exit oxnas_pci_exit(void)
{
	// if ( resource[0] ) {
	//     int errVal = release_resource(resource[0];
	//     if ( errVal ) {
	//         printk(KERN_ERR "PCI: unable to release csrRegister space %d", errVal );
	//     }
	// }

    // Put the PCI core into reset, but don't stop the clock as the PCI arbiter
    // still requires it in order to be able to grant the static bus access to
    // the PCI I/Os
    writel(1UL << SYS_CTRL_RSTEN_PCI_BIT, SYS_CTRL_RSTEN_SET_CTRL);

	return;
}

subsys_initcall(oxnas_pci_init);
module_exit(oxnas_pci_exit);

#endif
