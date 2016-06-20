/**************************************************************************
 *
 *  Copyright (c) 2004 Oxford Semiconductor Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  Module Name:
 *      ox800sata.c
 *
 *  Abstract:
 *      A driver to interface the 924 based sata core present in the ox800
 *      with libata and scsi
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/sysdev.h>
#include <linux/module.h>
#include <linux/leds.h>

#include <scsi/scsi_host.h>
#include <scsi/scsi_cmnd.h>
#include <scsi/scsi_device.h>
#include <asm/io.h>

#include <asm/arch/hardware.h>
#include <asm/arch/dma.h>
#include <asm/arch/memory.h>
#include <asm/arch/sata.h>
#include <linux/platform_device.h>

/***************************************************************************
* DEBUG CONTROL
***************************************************************************/
//#define SATA_DEBUG
//#define SATA_DUMP_REGS
//#define SATA_TF_DUMP
//#define CRAZY_DUMP_DEBUG

#if 0
    #ifdef writel
    #undef writel
    #endif
    #define writel(v,a) {printk("[%p]<=%08x\n",a,v);*((volatile u32*)(a)) = v;} 
#endif

#if 0    
    #ifdef readl
    #undef readl
    #endif
    static inline u32 myreadl(u32 a) {u32 v =(*((volatile u32*)(a)));printk("[%p]=>%08x\n",a,v);return v;}
    #define readl(a) (myreadl(a))
#endif


#include <linux/libata.h>
/***************************************************************************
* CONSTANTS
***************************************************************************/

#define DRIVER_AUTHOR   "Oxford Semiconductor Ltd."
#define DRIVER_DESC     "924 SATA core controler"
#define DRIVER_NAME     "oxnassata"

#define SATA_ABORT_WAIT_MS 5000
#define SATA_SRST_WAIT_MS  5000

/**************************************************************************
* PROTOTYPES
**************************************************************************/
static int  ox800sata_init_one(struct platform_device *);
static int  ox800sata_remove_one(struct platform_device *);

static void ox800sata_port_disable(struct ata_port *);
static void ox800sata_dev_config(struct ata_device *);
static void ox800sata_set_piomode(struct ata_port *, struct ata_device *);
static void ox800sata_set_dmamode(struct ata_port *, struct ata_device *);
static void ox800sata_tf_load(struct ata_port *ap, const struct ata_taskfile *tf);
static void ox800sata_tf_read(struct ata_port *ap, struct ata_taskfile *tf);
static void ox800sata_exec_command(struct ata_port *ap, const struct ata_taskfile *tf);
static u8   ox800sata_check_status(struct ata_port *ap);
static u8   ox800sata_check_altstatus(struct ata_port *ap);
static void ox800sata_dev_select(struct ata_port *ap, unsigned int device);
static void ox800sata_phy_reset(struct ata_port *ap);
static void ox800sata_bmdma_setup(struct ata_queued_cmd *qc);
static void ox800sata_bmdma_start(struct ata_queued_cmd *qc);
static u8   ox800sata_bmdma_status(struct ata_port *ap);
static struct ata_queued_cmd* ox800sata_qc_new(struct ata_port *ap);
static void ox800sata_qc_free(struct ata_queued_cmd *qc);
static unsigned int  ox800sata_qc_issue(struct ata_queued_cmd *qc);
static void ox800sata_eng_timeout(struct ata_port *ap);
static irqreturn_t ox800sata_irq_handler(int, void *);
static void ox800sata_eng_timeout(struct ata_port *ap);
static void ox800sata_irq_clear(struct ata_port *);
static int  ox800sata_scr_read(struct ata_port *ap, unsigned int sc_reg, u32 *val);
static int  ox800sata_scr_write(struct ata_port *ap, unsigned int sc_reg, u32 val);
static int  ox800sata_port_start(struct ata_port *ap);
static void ox800sata_port_stop(struct ata_port *ap);
static void ox800sata_host_stop(struct ata_host *host_set);
static unsigned int ox800sata_devchk(struct ata_port *ap,unsigned int device);
static u32* ox800sata_get_io_base(struct ata_port* ap);
static u32* ox800sata_get_bbp_base(void);
static u8   ox800sata_irq_ack(struct ata_port *ap, unsigned int chk_drq);
static u8 ox800sata_irq_on(struct ata_port *ap);
static void ox800sata_bmdma_stop(struct ata_queued_cmd *qc);
static void CrazyDumpDebug(struct ata_port *ap);
static void ox800sata_spot_the_end(struct work_struct *work);
static void ox800sata_timeout_cleanup( struct ata_port *ap );
static void ox800sata_reset_core( void );
static void ox800sata_pio_start(struct work_struct *work);
static void ox800sata_pio_task(struct work_struct *work);
static void ox800sata_post_reset_init(struct ata_port* ap);
static u32  __ox800sata_scr_read(u32* core_addr, unsigned int sc_reg);
static void __ox800sata_scr_write(u32* core_addr, unsigned int sc_reg, u32 val);

/**************************************************************************
* STRUCTURES
**************************************************************************/
typedef struct
{
    struct platform_driver driver;
    struct ata_port* ap[2];
    struct workqueue_struct* spot_the_end_q;
} ox800sata_driver_t;

/**
 * Struct to hold private (specific to this driver) data for each queued
 * command, all queued commands will point to a per-port private data
 * structure. This is a completely unresearched decision that will surely 
 * cause some untracable bug in the future.
 */
typedef struct
{
    oxnas_dma_channel_t* DmaChannel;
	oxnas_dma_sg_entry_t* sg_entries;
    struct spot_the_end_work_s  {
        struct work_struct worker;
        struct ata_port* ap;
    } spot_the_end_work;
    int port_disabled;
    u32 ErrorsWithNoCommamnd;
    u32 int_status;
    u32 in_cleanup;
} ox800sata_private_data;

ox800sata_driver_t ox800sata_driver = 
{
    .driver =
    {
        .driver.name = DRIVER_NAME,
        .driver.bus = &platform_bus_type,
        .probe = ox800sata_init_one, 
        .remove = ox800sata_remove_one,
    },
    .ap = {0,0},
};

/** If we were writing this in C++ then we would be deriving a subclass of 
ata_port, these would be the overridden functions*/
static struct ata_port_operations ox800sata_port_ops =
{
    .port_disable = ox800sata_port_disable,
    .dev_config = ox800sata_dev_config,
    .set_piomode = ox800sata_set_piomode,
    .set_dmamode = ox800sata_set_dmamode,
    .tf_load = ox800sata_tf_load,
    .tf_read = ox800sata_tf_read,
    .exec_command = ox800sata_exec_command,
    .check_status = ox800sata_check_status,
    .check_altstatus = ox800sata_check_altstatus,
    .dev_select = ox800sata_dev_select,
    .phy_reset = ox800sata_phy_reset,
    .bmdma_setup = ox800sata_bmdma_setup,
    .bmdma_start = ox800sata_bmdma_start,
    .bmdma_stop = ox800sata_bmdma_stop,
    .bmdma_status = ox800sata_bmdma_status,
    .qc_new = ox800sata_qc_new,
    .qc_free = ox800sata_qc_free,
    .qc_prep = ata_qc_prep,
    .qc_issue = ox800sata_qc_issue,
    .eng_timeout = ox800sata_eng_timeout,
    .irq_handler = ox800sata_irq_handler,
    .irq_clear = ox800sata_irq_clear,
    .scr_read = ox800sata_scr_read,
    .scr_write = ox800sata_scr_write,
    .port_start = ox800sata_port_start,
    .port_stop = ox800sata_port_stop,
    .host_stop = ox800sata_host_stop,
    .dev_chk = ox800sata_devchk,
    .irq_on = ox800sata_irq_on,
    .irq_ack = ox800sata_irq_ack,
    .pio_task = ox800sata_pio_start,
};

/** the scsi_host_template structure describes the basic capabilities of libata
and our 921 core to the SCSI framework, it contains the addresses of functions 
in the libata library that handle top level comands from the SCSI library */
static struct scsi_host_template ox800sata_sht = 
{
    .module             = THIS_MODULE,
    .name               = DRIVER_NAME,
    .ioctl              = ata_scsi_ioctl,
    .queuecommand       = ata_scsi_queuecmd,
/*    .eh_strategy_handler= ata_scsi_error,*/
    .can_queue          = ATA_DEF_QUEUE,
    .this_id            = ATA_SHT_THIS_ID,
/*    .sg_tablesize       = LIBATA_MAX_PRD,*/
    .sg_tablesize       = CONFIG_ARCH_OXNAS_MAX_SATA_SG_ENTRIES,
    .max_sectors        = 256,  // Use the full 28-bit SATA value
    .cmd_per_lun        = ATA_SHT_CMD_PER_LUN,
    .emulated           = ATA_SHT_EMULATED,
    .use_clustering     = ATA_SHT_USE_CLUSTERING,
    .proc_name          = DRIVER_NAME,
    .dma_boundary       = ~0UL, // NAS has no DMA boundary restrictions
    .slave_configure    = ata_scsi_slave_config,
    .bios_param         = ata_std_bios_param,
    .unchecked_isa_dma  = 0,

};

/** after PIO read operations, DRQ can remain set even when all the data has 
been read, when set, PretendDRQIsClear will mask out the DRQ bit in 
ox800sata_check_status operation */  
static char PretendDRQIsClear;

/**
 * used as a store for atomic test and set operations used to coordinate so
 * that only one port is processing comnmands at any time */
static unsigned long ox800sata_command_active;

/**
 * A record of which drives have accumulated raid faults. A set bit indicates
 * a fault has occured on that drive */
static u32 ox800sata_accumulated_RAID_faults = 0;

/**************************************************************************/
MODULE_LICENSE("GPL");
MODULE_VERSION(1.0);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);

/**************************************************************************
* FUCTIONS
* prefix all with "ox800sata_"
**************************************************************************/

/**
 * Gets the base address of the ata core from the ata_port structure
 * @param ap pointer to the appropriate ata_port structure
 * @return the base address of the SATA core
 */
static u32* ox800sata_get_io_base(struct ata_port* ap)
{
    // return (u32* )SATA0_REGS_BASE;
    
    return (u32* )ap->host->iomap;
}

/**
 * Gets the base address of the core that contains the BBP registers
 * @return the base address of the SATA core that contains the BBP registers
 */
static u32* ox800sata_get_bbp_base(void)
{
    return (u32* )SATA0_REGS_BASE;;
}

/**
 * Gets the base address of the sata link registers core from the
 * ata_port structure
 * @param ap pointer to the appropriate ata_port structure
 * @return the base address of the SATA core
 */
static u32* ox800sata_get_link_base(struct ata_port* ap)
{
    u8* link_base = (u8* )ap->host->iomap + 
        ((u8* )SATA0_LINK_REGS_BASE - (u8* )SATA0_REGS_BASE);
        
    return (u32* )link_base;
}

/**
 * Turns on the cores clock and resets it
 */
static void ox800sata_reset_core( void ){
    // Enable the clock to the SATA block
    writel(1UL << SYS_CTRL_CKEN_SATA_BIT, SYS_CTRL_CKEN_SET_CTRL);
    wmb();

    // reset both MAC and PHY
    writel(1UL << SYS_CTRL_RSTEN_SATA_BIT, SYS_CTRL_RSTEN_SET_CTRL);
    writel(1UL << SYS_CTRL_RSTEN_SATA_PHY_BIT, SYS_CTRL_RSTEN_SET_CTRL);
    wmb();
    udelay(50);
    
    // un-reset both MAC and PHY
    writel(1UL << SYS_CTRL_RSTEN_SATA_BIT, SYS_CTRL_RSTEN_CLR_CTRL);
    writel(1UL << SYS_CTRL_RSTEN_SATA_PHY_BIT, SYS_CTRL_RSTEN_CLR_CTRL);
    wmb();
    udelay(50);
}

/**
 * port capabilities for the ox800 sata ports.
 */
static const struct ata_port_info ox800sata_port_info = {
    .flags = ATA_FLAG_SATA | ATA_FLAG_SATA_RESET | ATA_FLAG_NO_LEGACY,
    .pio_mask   = 0x1f, /* pio modes 0..4*/
    .mwdma_mask = 0x07, /* mwdma0-2 */
    .udma_mask  = 0x7f, /* udma0-5 */
    .port_ops   = &ox800sata_port_ops,
};

/** 
 * The driver probe function.
 * Registered with the amba bus driver as a parameter of ox800sata_driver.bus
 * it will register the ata device with kernel first performing any 
 * initialisation required (if the correct device is present).
 * @param pdev Pointer to the 921 device structure 
 * @param port where on the bus the port was found, ignored and probably wrong
 * @return 0 if no errors
 */
static int ox800sata_init_one(struct platform_device* pdev)
{
    
    u32 version;
#ifdef CONFIG_SATA_OXNAS_DISK_LIGHT
    unsigned long reg;
#endif // CONFIG_SATA_OXNAS_DISK_LIGHT
    struct ata_host *host;
    const struct ata_port_info* port_info[] = { &ox800sata_port_info, NULL };

    void __iomem* iomem;
    struct resource* memres = platform_get_resource(pdev, IORESOURCE_MEM, 0 );
    int irq = platform_get_irq(pdev, 0);
    
    DPRINTK("\n");

    /* check resourses for sanity */
    if ((memres == NULL) || (irq < 0)) {
        return 0;
    }
    iomem = (void __iomem* ) memres->start;
    
    /* check we support this version of the core */
    version = readl(((u32* )iomem) + OX800SATA_VERSION) & 0xff;
    switch (version)
    {
        case OX800SATA_CORE_VERSION:
            printk(KERN_INFO"ox800sata: OX800 sata core.\n");   
            break;
        default:
            printk(KERN_ERR"ox800sata: unknown sata core (version register = 0x%08x)\n",version);     
            return 0;
            break;
    }

    /* reset the core */
    ox800sata_reset_core();

    /* initialise a work queue to spot the end of transfers */
    ox800sata_driver.spot_the_end_q = create_singlethread_workqueue("sata-endQ");
    if (!ox800sata_driver.spot_the_end_q) {
        printk(KERN_ERR DRIVER_NAME " Couldn't create a work queue.\n");
        return -1;
    }

#ifdef CONFIG_SATA_OXNAS_DISK_LIGHT
    /* setup path */
    reg = ~(1 << CONFIG_OX800SATA_DISK_LIGHT_GPIO_LINE) & readl(SYS_CTRL_GPIO_PRIMSEL_CTRL_0);
    writel(reg, SYS_CTRL_GPIO_PRIMSEL_CTRL_0);
    reg = ~(1 << CONFIG_OX800SATA_DISK_LIGHT_GPIO_LINE) & readl(SYS_CTRL_GPIO_SECSEL_CTRL_0);
    writel(reg, SYS_CTRL_GPIO_SECSEL_CTRL_0);
    reg = ~(1 << CONFIG_OX800SATA_DISK_LIGHT_GPIO_LINE) & readl(SYS_CTRL_GPIO_TERTSEL_CTRL_0);
    writel(reg, SYS_CTRL_GPIO_TERTSEL_CTRL_0);
    
    /* enable output */
    writel(1 << CONFIG_OX800SATA_DISK_LIGHT_GPIO_LINE, GPIO_A_OUTPUT_ENABLE);
    
    /* disk light off */
    writel(1 << CONFIG_OX800SATA_DISK_LIGHT_GPIO_LINE, GPIO_A_OUTPUT_CLEAR);
#endif  /* CONFIG_SATA_OXNAS_DISK_LIGHT */

    /* setup the probe_ent structure which is basically info about the ports 
    capabilities */
    
    /* allocate memory and check */
    host = ata_host_alloc_pinfo(&(pdev->dev), port_info, OX800SATA_MAX_PORTS );
    if (!host) {
        printk(KERN_ERR DRIVER_NAME " Couldn't create an ata host.\n");
        destroy_workqueue(ox800sata_driver.spot_the_end_q);
    }
    
    /* set to base of ata core */
    host->iomap  = iomem;
    
    /* call ata_device_add and begin probing for drives*/
    ata_host_activate(host,
        irq,
        ox800sata_irq_handler,
        0,
        &ox800sata_sht );
    
    return 0;
}

/** 
 * Called when the amba bus tells this device to remove itself.
 * @param pdev pointer to the device that needs to be shutdown
 */
static int ox800sata_remove_one(struct platform_device* pdev)
{
    struct ata_host *host_set = dev_get_drvdata( &(pdev->dev) );
    struct ata_port *ap;
    unsigned int i;
    
    
    for (i = 0; i < host_set->n_ports; i++) 
    {
        ap = host_set->ports[i];
        scsi_remove_host( ap->scsi_host );
    }
    
    /** @TODO etc. */

    // Disable the clock to the SATA block
    writel(1UL << SYS_CTRL_CKEN_SATA_BIT, SYS_CTRL_CKEN_CLR_CTRL);
    
    return 0;
}

/** 
 * module initialisation
 * @return success
 */
static int __init ox800sata_init( void )
{
    int ret;
    
    printk(KERN_INFO"ox800sata init \n");
    
    ret = platform_driver_register( &ox800sata_driver.driver );
    
    return ret; 
}

/** 
 * module cleanup
 */
static void __exit ox800sata_exit( void )
{
    platform_driver_unregister( &ox800sata_driver.driver );
}

/** 
 * macros to register intiialisation and exit functions with kernal
 */
module_init(ox800sata_init);
module_exit(ox800sata_exit);

/** 
 * Called from ata_bus_probe() and ata_bus_reset() error paths, as well as
 * when unregistering from the SCSI module (rmmod, hot unplug).
 * @param port The port to disable
 */
static void ox800sata_port_disable(struct ata_port* port)
{
    DPRINTK("\n");
}

/** 
 * Called after IDENTIFY [PACKET] DEVICE is issued to each device found.
 * Typically used to apply device-specific fixups prior to issue of
 * SET FEATURES - XFER MODE, and prior to operation.
 * @param port The port to configure
 * @param pdev The hardware associated with controlling the port
 */
static void ox800sata_dev_config(struct ata_device* pdev)
{
    u32 reg;
    u32 *ioaddr = ox800sata_get_io_base(pdev->ap);

    DPRINTK("\n");

    /* if needed, set the bits to put the interface into 48-bit node */
    if (pdev->flags & ATA_DFLAG_LBA48) {
        reg = readl( ioaddr + OX800SATA_DRIVE_CONTROL );
        
        /* mask out the pair of bits associaed with each port */
        reg &= ~( 3 << (pdev->ap->port_no * 2) );
        
        /* set the mode pair associated with each port */
        reg |= 2 << (pdev->ap->port_no * 2);
        writel(reg  ,ioaddr + OX800SATA_DRIVE_CONTROL);
    }
}

/** 
 * Hooks called prior to the issue of SET FEATURES - XFER MODE command. 
 * dev->pio_mode is guaranteed to be valid when ->set_piomode() is called
 *
 * If we're doing PIO, we need to disable the burst buffer port to stop it 
 * trying to do a DMA transfer of the data.
 * 
 * @param port The port to configure
 * @param pdev The hardware associated with controlling the port
 */
static void ox800sata_set_piomode(struct ata_port* port, struct ata_device* pdev)
{
    u32 Register;
    u32 *ioaddr = ox800sata_get_bbp_base();

    DPRINTK("\n");

    /* disable burst buffer DMA */
    Register = readl( ioaddr + OX800SATA_BURST_CONTROL );
    Register |= OX800SATA_BBC_DREQ_DIS ;
    writel(Register  ,ioaddr + OX800SATA_BURST_CONTROL);
    PretendDRQIsClear = 0;
}

/** 
 * Hooks called prior to the issue of SET FEATURES - XFER MODE command.
 * dev->dma_mode is guaranteed to be valid when ->set_dmamode() is called.
 *
 * When doing a DMA transfer, we need to enable the burst buffer port as this
 * may have been disabled by a previous command.
 * 
 * @param port The port to configure
 * @param pdev The hardware associated with controlling the port
 */
static void ox800sata_set_dmamode(struct ata_port* port, struct ata_device* pdev)
{
    u32 Register;
    u32 *ioaddr = ox800sata_get_bbp_base();

    DPRINTK("\n");

    /* enable burst buffer DMA */
    Register = readl( ioaddr + OX800SATA_BURST_CONTROL );
    Register &= ~OX800SATA_BBC_DREQ_DIS;
    writel(Register  ,ioaddr + OX800SATA_BURST_CONTROL);
}

/** 
 * Output the taskfile for diagnostic reasons, it will always appear in the 
 * debug output as if it's a task file being written.
 * @param tf The taskfile to output
 */
static void tfdump(const struct ata_taskfile* tf)
{
    if (tf->flags & ATA_TFLAG_LBA48) {
#ifdef SATA_TF_DUMP
    printk("Cmd %x Ft %x%x, LBA-48 %02x%02x%02x%02x%02x%02x, nsect %02x%02x, ctl %02x, dev %x\n",
#else // SATA_TF_DUMP
    DPRINTK("Cmd %x Ft %x%x, LBA-48 %02x%02x%02x%02x%02x%02x, nsect %02x%02x, ctl %02x, dev %x\n",
#endif // SATA_TF_DUMP
        tf->command,
        
        tf->hob_feature,
        tf->feature,
        
        tf->hob_lbah,
        tf->hob_lbam,
        tf->hob_lbal,
        tf->lbah,
        tf->lbam,
        tf->lbal,
        
        tf->hob_nsect,
        tf->nsect,
        tf->ctl,
        tf->device );
    }else{
#ifdef SATA_TF_DUMP
    printk("Cmd %x Ft %x, LBA-28 %01x%02x%02x%02x, nsect %02x, ctl %02x, dev %x\n",
#else // SATA_TF_DUMP
    DPRINTK("Cmd %x Ft %x, LBA-28 %01x%02x%02x%02x, nsect %02x, ctl %02x, dev %x\n",
#endif // SATA_TF_DUMP
        tf->command,
        
        tf->feature,

        tf->device & 0x0f,        
        tf->lbah,
        tf->lbam,
        tf->lbal,
        
        tf->nsect,
        tf->ctl,
        tf->device );
    }
}

/** 
 * called to write a taskfile into the ORB registers
 * @param ap hardware with the registers in
 * @param tf taskfile to write to the registers
 */
static void ox800sata_tf_load(struct ata_port *ap, const struct ata_taskfile *tf)
{
    u32 count = 0;
    u32 Orb1 = 0; 
    u32 Orb2 = 0; 
    u32 Orb3 = 0;
    u32 Orb4 = 0;
    u32 Command_Reg;
    u32 *ioaddr = ox800sata_get_io_base(ap);
    unsigned int is_addr = tf->flags & ATA_TFLAG_ISADDR;

    do {
        Command_Reg = readl(ioaddr + OX800SATA_SATA_COMMAND );
        if (!(Command_Reg & CMD_CORE_BUSY))
            break;
        count++;
        if ( in_atomic() ) {
            mdelay(1);
        } else {
            msleep(1);
        }
    } while (count < 10);

    /* if the control register has changed, write it */
    if (tf->ctl != ap->last_ctl) 
    {
        //DPRINTK("ap->last_ctl = %02x",ap->last_ctl);
        Orb4 |= (tf->ctl) << 24;

        /* write value to register */
        writel(Orb4, ioaddr + OX800SATA_ORB4 );

        ap->last_ctl = tf->ctl;
    }

    /* check if the ctl register has interrupts disabled or enabled and
    modify the interrupt enable registers on the ata core as required */
    if (tf->ctl & ATA_NIEN)
    {
        /* interrupts disabled */
        ox800sata_irq_clear(ap);
    }
    else
    {
        /* interrupts enabled */
        ox800sata_irq_on(ap);
    }
    
    /* write 48 or 28 bit tf parameters */
    if (is_addr)
    {
        /* set LBA bit as it's an address */
        Orb1 |= (tf->device & ATA_LBA) << 24;
        
        if (tf->flags & ATA_TFLAG_LBA48) 
        {
            //DPRINTK(KERN_INFO" 48 bit tf load \n");
            Orb1 |= ATA_LBA << 24;
            
            Orb2 |= (tf->hob_nsect)  << 8 ;

            Orb3 |= (tf->hob_lbal)   << 24;

            Orb4 |= (tf->hob_lbam)   << 0 ;
            Orb4 |= (tf->hob_lbah)   << 8 ;
            Orb4 |= (tf->hob_feature)<< 16;
        } else {
            Orb1 |= (tf->device & 0xf)<< 24;
        }

        /* write 28-bit lba */
        //DPRINTK(KERN_INFO" 28 bit tf load\n");
        Orb1 |= (tf->lbal)       << 0 ;
        Orb1 |= (tf->lbam)       << 8 ;
        Orb1 |= (tf->lbah)       << 16;

        Orb2 |= (tf->nsect)      << 0 ;
        Orb2 |= (tf->feature)    << 16;
        Orb2 |= (tf->command)    << 24;
        
        Orb3 |= (tf->lbal)       << 0 ;
        Orb3 |= (tf->lbam)       << 8 ;
        Orb3 |= (tf->lbah)       << 16;

        Orb4 |= (tf->ctl)        << 24;
        
        /* write values to registers */
        writel(Orb1, ioaddr + OX800SATA_ORB1 );
        writel(Orb2, ioaddr + OX800SATA_ORB2 );
        writel(Orb3, ioaddr + OX800SATA_ORB3 );
        writel(Orb4, ioaddr + OX800SATA_ORB4 );
    }

    if (tf->flags & ATA_TFLAG_DEVICE) 
    {
        Orb1 |= (tf->device)     << 24;

        /* write value to register */
        writel(Orb1, ioaddr + OX800SATA_ORB1 );
    }
    
    tfdump(tf);
    
}

/** 
 * Called to read the hardware registers / DMA buffers, to
 * obtain the current set of taskfile register values.
 * @param ap hardware with the registers in
 * @param tf taskfile to read the registers into
 */
static void ox800sata_tf_read(struct ata_port *ap, struct ata_taskfile *tf)
{
    u32 *ioaddr = ox800sata_get_io_base(ap);

    /* read the orb registers */
    u32 Orb1 = readl( ioaddr + OX800SATA_ORB1 ); 
    u32 Orb2 = readl( ioaddr + OX800SATA_ORB2 ); 
    u32 Orb3 = readl( ioaddr + OX800SATA_ORB3 );
    u32 Orb4 = readl( ioaddr + OX800SATA_ORB4 );

    DPRINTK("\n");

    /* read common 28/48 bit tf parameters */
    tf->device  = (Orb1 >> 24);
    tf->nsect   = (Orb2 >> 0);
    tf->feature = (Orb2 >> 16);
    tf->command = ox800sata_check_status(ap);

    /* read 48 or 28 bit tf parameters */
    if (tf->flags & ATA_TFLAG_LBA48) 
    {
        //DPRINTK(KERN_INFO" 48 bit tf read \n");
        tf->hob_nsect = (Orb2 >> 8 ) ;
        
        tf->lbal      = (Orb3 >> 0 ) ;
        tf->lbam      = (Orb3 >> 8 ) ;
        tf->lbah      = (Orb3 >> 16) ;
        tf->hob_lbal  = (Orb3 >> 24) ;
        
        tf->hob_lbam  = (Orb4 >> 0 ) ;
        tf->hob_lbah  = (Orb4 >> 8 ) ;
        /* feature ext and control are write only */

    }
    else
    {
        /* read 28-bit lba */
        //DPRINTK(KERN_INFO" 28 bit tf read\n");
        tf->lbal      = (Orb1 >> 0 ) ;
        tf->lbam      = (Orb1 >> 8 ) ;
        tf->lbah      = (Orb1 >> 16) ;
    }

    /* fixup NAS SATA core's non-std status reporting */
    if (PretendDRQIsClear)
    {
         tf->command &= ~ATA_DRQ;
    }

    tfdump(tf);
}

/** 
 * Causes an ATA command, previously loaded with ->tf_load(), to be
 * initiated in hardware. The command is written into the registers again just
 * to be sure. All the other registers that are in Orb2 are also written at the 
 * same time. The command is initiated in hardware by a poke to the COMMAND
 * register.
 * @param ap hardware with the registers in
 * @param tf taskfile to write to the registers
 */
static void ox800sata_exec_command(struct ata_port *ap, const struct ata_taskfile *tf)
{
    u32 count =0;
    u32 *ioaddr = ox800sata_get_io_base(ap);
    u32 Orb2 ;
    u32 Command_Reg;
    ox800sata_private_data* private_data ;

    //DPRINTK(KERN_INFO"ox800sata_exec_command cmd %02x\n", tf->command);
    do {
        Command_Reg = readl(ioaddr + OX800SATA_SATA_COMMAND );
        if (!(Command_Reg & CMD_CORE_BUSY))
            break;
        count++;
        if ( in_atomic() ) {
            mdelay(1);
        } else {
            msleep(1);
        }
    } while (count < 10);
    
    /* write all the things in Orb 2 */
    Orb2  = (tf->nsect)     << 0 ;
    if (tf->flags & ATA_TFLAG_LBA48) 
    {
        Orb2 |= (tf->hob_nsect) << 8 ;
    }
    Orb2 |= (tf->feature)   << 16;
    Orb2 |= (tf->command)   << 24;
    writel( Orb2 , ioaddr + OX800SATA_ORB2 );
    wmb();

    do {
        Command_Reg = readl(ioaddr + OX800SATA_SATA_COMMAND );
        if (!(Command_Reg & CMD_CORE_BUSY))
            break;
        count++;
        if ( in_atomic() ) {
            mdelay(1);
        } else {
            msleep(1);
        }
    } while (count < 10);

    /* if the drive has become disconnected, executing a command will be a
    problem */ 
    private_data = (ox800sata_private_data*)ap->private_data;
    
    /* Command that the orb registers get written to drive */
    Command_Reg &= ~SATA_OPCODE_MASK;
    Command_Reg |= CMD_WRITE_TO_ORB_REGS;
    writel( Command_Reg , ioaddr + OX800SATA_SATA_COMMAND );
    wmb();
}


/** 
 * Reads the Status ATA shadow register from hardware. Due to a fault with PIO
 * transfers, it it sometimes necessary to mask out the DRQ bit
 * @param ap hardware with the registers in
 * @return The status register
 */
static u8   ox800sata_check_status(struct ata_port *ap)
{
    u32 Reg;
    u8 result;
    u8 state;
    u32 *ioaddr = ox800sata_get_io_base(ap);
    
//    VPRINTK(KERN_INFO"ox800sata_check_status ");
    
    /* read byte 3 of Orb2 register */
    result = readl(ioaddr + OX800SATA_ORB2 ) >> 24;
    
    /* set DRQ when core is in PIO transfer states */
    state = readl(ioaddr + OX800SATA_SATA_CONTROL) & OX800SATA_SATA_CONTROL_TRANS_MASK;
    if ((state == OX800SATA_TRANS_PIOITRANS) || (state == OX800SATA_TRANS_PIOOTRANS)) {
        result |= ATA_DRQ;
    }
    
    if (PretendDRQIsClear)
    {
//        VPRINTK("(ignoring DRQ) ");    
        result &= ~ATA_DRQ;
    }
            
    /* use error informatian from raw interupt status */
    if (readl(ioaddr + OX800SATA_INT_STATUS) & OX800SATA_RAW_ERROR) { 
        result |= ATA_ERR;
    } else {
        result &= ~ATA_ERR;
    }
    
    /* check for the drive going missing indicated by SCR status bits 0-3 = 0 */
    ox800sata_scr_read(ap, SCR_STATUS, &Reg);
    if (!(Reg & 0x1)) { 
        result |= ATA_DF;
        result |= ATA_ERR;
    }
    //VPRINTK("%02x\n",result);    

    return result;
}

/** 
 * Reads the alt status ATA shadow register from hardware. 
 * @param ap hardware with the registers in
 * @return The alt status register
 */
static u8   ox800sata_check_altstatus(struct ata_port *ap)
{
    u8 result;
    u32 *ioaddr = ox800sata_get_io_base(ap);
    
    //DPRINTK(KERN_INFO"ox800sata_check_altstatus base=%p\n",ioaddr);
    
    /* read byte 3 of Orb4 register */
    result = ( readl(ioaddr + OX800SATA_ORB4 ) >> 24 ) ;

    //DPRINTK(KERN_INFO"alternate status register %02x\n",result);    

    return result;
}

/** 
 * Use the method defined in the ATA specification to make either device 0,
 * or device 1, active on the ATA channel. If we ever get port multipliers
 * to work, this will be where they would switch.
 *
 * @param ap hardware with the registers in
 * @param number of the device to talk to (0..)
 */
static void ox800sata_dev_select(struct ata_port *ap, unsigned int device)
{
    /* currently only one i/f, but this may not always be the case */
    const unsigned char interface_no = 0;
    
    u32 *ioaddr = ox800sata_get_io_base(ap);
    DPRINTK(" i/f=%d dev=%d\n", interface_no, device);
    
    writel(
        (interface_no << 4) | device ,
        ioaddr + OX800SATA_DEVICE_SELECT );
}

/** 
 * The very first step in the probe phase. Actions vary depending on the bus
 * type, typically. After waking up the device and probing for device presence
 * (PATA and SATA), typically a soft reset (SRST) will be performed. Drivers
 * typically use the helper functions ata_bus_reset() or sata_phy_reset() for
 * this hook.
 *
 * This should reset the SATA core and Phisical layer then jump back into the 
 * libata libraries for lots of other resetting
 *
 * @param ap hardware with the registers in
 */
static void ox800sata_phy_reset(struct ata_port *ap)
{
    u32 *ioaddr = ox800sata_get_io_base(ap);

    //DPRINTK(KERN_INFO"ox800sata_phy_reset base = %p\n", ioaddr);
    
    /* turn ata core on */
    writel( (1 << SYS_CTRL_CKEN_SATA_BIT), SYS_CTRL_CKEN_SET_CTRL);
    
    /* stop all the interrupts in the ata core */
    writel( ~0, ioaddr + OX800SATA_INT_DISABLE);
    writel( ~0, ioaddr + OX800SATA_INT_CLEAR);
    
    /* get libata to perform a soft reset */
    sata_phy_reset(ap);
    
}

/** 
 * When setting up an IDE BMDMA transaction, these hooks arm (->bmdma_setup) 
 * and fire (->bmdma_start) the hardware's DMA engine.
 *
 * @param qc the queued command to issue
 */
static void ox800sata_bmdma_setup(struct ata_queued_cmd *qc)
{
    ox800sata_private_data* PrivateData ;
    oxnas_dma_direction_t direction; 

#ifdef SATA_DEBUG
    printk(KERN_INFO"ox800sata_bmdma_setup: %s, %d element%s\n", (qc->dma_dir == DMA_FROM_DEVICE) ? "Read" : "Write", qc->n_elem, qc->n_elem ? "s" : "");
#else // SATA_DEBUG
    DPRINTK(" %s, %d element%s\n", (qc->dma_dir == DMA_FROM_DEVICE) ? "Read" : "Write", qc->n_elem, qc->n_elem ? "s" : "");
#endif // SATA_DEBUG
    
    qc->private_data = qc->ap->private_data;
    PrivateData = (ox800sata_private_data* )qc->private_data;

	// We check for DMA completion from ISR which cannot wait for all DMA channel
	// housekeeping to complete, so need to wait here is case we try to reuse
	// channel before that housekeeping has completed
	while (oxnas_dma_is_active(PrivateData->DmaChannel)) {
		printk("DMA Setup Channel still active\n");
	}

    /* Do not use DMA callback */
	oxnas_dma_set_callback(PrivateData->DmaChannel, OXNAS_DMA_CALLBACK_NUL, OXNAS_DMA_CALLBACK_ARG_NUL);
    
    /* decide on DMA direction */
    direction = (qc->dma_dir == DMA_FROM_DEVICE) ? OXNAS_DMA_FROM_DEVICE :
                                                   OXNAS_DMA_TO_DEVICE;

/* Expect transfers to be multiples of 4KB */
//struct scatterlist* sle = qc->sg;
//while (sg_dma_len(sle)) {
//    BUG_ON(sg_dma_len(sle) % 4096);
//    ++sle;
//}

    /* now set-up the DMA transfer */
    if (qc->n_elem > 1)
    {
#ifdef SATA_DEBUG
    u32 total=0;
    int i=0;
    struct scatterlist* sg = qc->__sg;
    printk("Lengths: ");
    do {
        u32 len = sg_dma_len(sg++);
        printk("%u ", len); 
        total += len;
    } while (++i < qc->n_elem);
    printk("\nTotal len = %u\n", total);
#endif //  SATA_DEBUG
        /* try and setup scatter gather controller */
/*        if (oxnas_dma_device_set_sg(PrivateData->DmaChannel,
                                    direction,
                                    qc->__sg,
                                    qc->n_elem,
                                    &oxnas_sata_dma_settings,
                                    OXNAS_DMA_MODE_INC )) {
            printk(KERN_ERR"Failed to setup DMA with disk.\n");
            return;
        }*/
        if (oxnas_dma_device_set_prd(
                PrivateData->DmaChannel,
                direction,
                qc->ap->prd,
                &oxnas_sata_dma_settings,
                OXNAS_DMA_MODE_INC,
				 PrivateData->sg_entries)) {
            printk(KERN_ERR"Failed to setup DMA with disk.\n");
            return;
        }
    }
    else
    {
#ifdef SATA_DEBUG
    printk("Total len = %u\n", sg_dma_len(qc->__sg));
#endif //  SATA_DEBUG
        /* setup a single dma */
        oxnas_dma_device_set(   PrivateData->DmaChannel,
                                direction,
                                (unsigned char* )sg_dma_address(qc->__sg),
                                sg_dma_len(qc->__sg),
                                &oxnas_sata_dma_settings,
                                OXNAS_DMA_MODE_INC,
                                1); /* paused */
    }
}

/** 
 * When setting up an IDE BMDMA transaction, these hooks arm (->ignedmdma_setup) 
 * and fire (->bmdma_start) the hardware's DMA engine.
 *
 * @param qc the queued command to issue
 */
static void ox800sata_bmdma_start(struct ata_queued_cmd *qc)
{
    ox800sata_private_data* PrivateData ;
    DPRINTK("\n");
    PrivateData = (ox800sata_private_data* )(qc->private_data);    

    {
        /* turn on the fifo */    
        u32 Register;
        u32 *ioaddr = ox800sata_get_bbp_base();
        Register = readl( ioaddr + OX800SATA_BURST_CONTROL );
        Register &= ~OX800SATA_BBC_FIFO_DIS;
        writel(Register  ,ioaddr + OX800SATA_BURST_CONTROL);
    }
    
    /* if the drive has become disconnected, executing a command will be a
    problem */ 
    {
        /* start DMA transfer */
        oxnas_dma_start( PrivateData->DmaChannel );
        qc->ap->ops->exec_command(qc->ap, &(qc->tf));
    }
}


/**
 *  ata_qc_new - Request an available ATA command, for queueing
 *  @ap: Port associated with device @dev
 *  @dev: Device from whom we request an available command structure
 *
 *  LOCKING:
 */

static struct ata_queued_cmd* ox800sata_qc_new(struct ata_port *ap)
{
    struct ata_queued_cmd *qc = NULL;

    /* first see if we're not doing a command */
    if (!test_and_set_bit(0, &ox800sata_command_active)) {
            /* now set the standard bits for compatibility */
            set_bit(0, &ap->qc_allocated); 
            qc = ata_qc_from_tag(ap, 0);
            
#ifdef CONFIG_SATA_OXNAS_DISK_LIGHT
            /* disk light on */
            writel(1 << CONFIG_OX800SATA_DISK_LIGHT_GPIO_LINE, GPIO_A_OUTPUT_SET);
#endif  // CONFIG_SATA_OXNAS_DISK_LIGHT
#ifdef CONFIG_WDC_LEDS_TRIGGER_SATA_DISK
            wdc_ledtrig_sata_activity();
#endif // CONFIG_WDC_LEDS_TRIGGER_SATA_DISK
    } else
        DPRINTK("Command active flag still set\n");

    if (qc)
        qc->tag = 0;

    return qc;
}


/**
 *
 */
static void ox800sata_qc_free(struct ata_queued_cmd *qc)
{
    struct ata_port *ap = qc->ap;
    unsigned int tag, do_clear = 0;

    DPRINTK("\n");

    qc->flags = 0;
    tag = qc->tag;
    if (likely(ata_tag_valid(tag))) {
        if (tag == ap->active_tag)
            ap->active_tag = ATA_TAG_POISON;
        qc->tag = ATA_TAG_POISON;
        do_clear = 1;
    }

    if (likely(do_clear)) {
        clear_bit(tag, &ap->qc_allocated);
        clear_bit(0, &ox800sata_command_active);
        
#ifdef CONFIG_SATA_OXNAS_DISK_LIGHT            
        /* disk light off */
        writel(1 << CONFIG_OX800SATA_DISK_LIGHT_GPIO_LINE, GPIO_A_OUTPUT_CLEAR);
#endif  /* CONFIG_SATA_OXNAS_DISK_LIGHT */
    }
}

/** 
 * qc_issue is used to make a command active, once the hardware and S/G tables
 * have been prepared. IDE BMDMA drivers use the helper function
 * ata_qc_issue_prot() for taskfile protocol-based dispatch. More advanced drivers
 * roll their own ->qc_issue implementation, using this as the "issue new ATA
 * command to hardware" hook.
 * @param qc the queued command to issue
 */
static unsigned int  ox800sata_qc_issue(struct ata_queued_cmd *qc)
{
    u32 Register;
    int this_port_fail;
    ox800sata_private_data* private_data = (ox800sata_private_data*)qc->ap->private_data ;
    u32 *ioaddr = ox800sata_get_bbp_base();
	u32 reg;
    u32 raid_reg = 0; /* default to no raid */

    DPRINTK("\n");

    /* get raid settings from the bio if they exist */    
    if (qc->scsicmd && qc->scsicmd->request && qc->scsicmd->request->bio) {
        struct bio* bio;
        bio = qc->scsicmd->request->bio;
        raid_reg = bio->bi_raid ;
        if (raid_reg) DPRINTK(" raid reg 0x%08x\n",raid_reg);
    }

    /* check cable is still connected */
	ox800sata_scr_read(qc->ap, SCR_STATUS, &reg);
    private_data->port_disabled |= (!(reg & 1));
        
    this_port_fail = private_data->port_disabled;
    
    if (raid_reg) {
        int port0fail, port1fail;
        port0fail = (! (__ox800sata_scr_read((u32* )SATA0_LINK_REGS_BASE, 0x20 + (4 * SCR_STATUS) ) & 1 ) ); 
        port1fail = (! (__ox800sata_scr_read((u32* )SATA1_LINK_REGS_BASE, 0x20 + (4 * SCR_STATUS) ) & 1 ) );
        this_port_fail |= port1fail;
        
        ox800sata_accumulated_RAID_faults |= port0fail ? 1 : 0 ;
        ox800sata_accumulated_RAID_faults |= port1fail ? 2 : 0 ;
    }

    if (!this_port_fail ) { 
        writel(raid_reg  ,ioaddr + OX800SATA_RAID_CONTROL);

        DPRINTK(" enabling burst buffer DMA\n");        
        Register = readl( ioaddr + OX800SATA_BURST_CONTROL );
        Register &= ~OX800SATA_BBC_DREQ_DIS;
        writel(Register  ,ioaddr + OX800SATA_BURST_CONTROL);
        
        /* call the default, this should be changed to take advantage of orb
        registers, etc... */
        return ata_qc_issue_prot(qc);
    } else {
        /* record the error */
        qc->err_mask |= AC_ERR_ATA_BUS;

        /* offline the SCSI device */        
        printk(KERN_ERR"ata%u offline\n", qc->ap->print_id);
        scsi_device_set_state(qc->scsicmd->device, SDEV_OFFLINE);
        return 0;
    }
}

/** 
 * This is a high level error handling function, called from the error
 * handling thread, when a command times out.
 *
 * @todo possibly remove this function and revert to only calling the default
 *
 * @param ap hardware with the registers in
 */
static void ox800sata_eng_timeout(struct ata_port *ap)
{
    struct ata_queued_cmd *qc;
    ox800sata_private_data* pd = (ox800sata_private_data*)ap->private_data;    
    DPRINTK("\n");
    
    /* set the in cleanup flag */
    pd->in_cleanup = 1;
    
    /* if we're a PIO command existing cleanup won't be called */
	qc = ata_qc_from_tag(ap, ap->active_tag);
	if (qc->tf.protocol == ATA_PROT_PIO) {
        /* reset the core */
        ox800sata_timeout_cleanup(ap);
    }
        
    /* call strandard lib ata function */
    ata_eng_timeout( ap );

    /* clear the in cleanup flag */
    pd->in_cleanup = 0;
}

/** 
 * irq_handler is the interrupt handling routine registered with the system,
 * by libata.
 */
static irqreturn_t ox800sata_irq_handler(int irq,
                                        void* dev_instance)
{
    struct ata_port        *ap = ((struct ata_host *)dev_instance)->ports[0];
    ox800sata_private_data *pd;
    u32                    *ioaddr;
    u32                     int_status;    

    DPRINTK("irq = %d\n", irq);

    if (!ap || !ap->private_data)
        BUG();

    pd = (ox800sata_private_data*)ap->private_data;
    ioaddr = ox800sata_get_io_base(ap);

    int_status = readl(ioaddr + OX800SATA_INT_STATUS);
    while (int_status & OX800SATA_INT_MASKABLE) {
        /* store interrupt status for the bottom end */
        pd->int_status |= int_status;

        /* Clear and mask pending interrupts */
        writel(int_status, ioaddr + OX800SATA_INT_CLEAR);
        writel(int_status, ioaddr + OX800SATA_INT_DISABLE);

        int_status = readl(ioaddr + OX800SATA_INT_STATUS);
    }

	// Wait a short while for the DMA to finish and if it doesn't start a thread
	// to poll for the finish
    pd->spot_the_end_work.ap = ap;
	if (!oxnas_dma_raw_isactive(pd->DmaChannel)) {
		ox800sata_spot_the_end(&(pd->spot_the_end_work.worker));
	} else {
		udelay(100);
		if (!oxnas_dma_raw_isactive(pd->DmaChannel)) {
			ox800sata_spot_the_end(&(pd->spot_the_end_work.worker));
		} else {
			/* Start a worker thread looking for the DMA channel to become idle */
			queue_work(ox800sata_driver.spot_the_end_q, &pd->spot_the_end_work.worker);
		}
	}

    return IRQ_HANDLED;
}

/**
 * Work for a work queue, this will check for errors then wait for the DMA to
 * complete. On the DMA completing it will call ata_qc_complete
 */
static void ox800sata_spot_the_end(struct work_struct *work)
{
    struct spot_the_end_work_s* stew = 
        container_of(work, struct spot_the_end_work_s, worker);
    struct ata_port* ap = stew->ap;
    ox800sata_private_data* PrivateData = (ox800sata_private_data* )ap->private_data;
    struct ata_queued_cmd* qc = ata_qc_from_tag(ap, ap->active_tag);
    unsigned long flags = 0;

    /* If there's no command ending associated with this IRQ, ignore it. */
    if ((qc == NULL) ||
        !(PrivateData->int_status & OX800SATA_INT_END_OF_CMD)) {
        DPRINTK(" qc=null\n");
        return;
    }

    /* Look to see if the core is indicating an error condition after a RAID 
     * command */
    if (qc->scsicmd &&
        qc->scsicmd->request &&
        qc->scsicmd->request->bio &&
        qc->scsicmd->request->bio->bi_raid ) {
        unsigned long Port0Irq = readl(((u32)(SATA0_REGS_BASE)) + OX800SATA_INT_STATUS);
        unsigned long Port1Irq = readl(((u32)(SATA1_REGS_BASE)) + OX800SATA_INT_STATUS);

        if (test_bit(OX800SATA_INT_ERROR,  &Port0Irq)) {
            printk("disk 0 error in raid\n");
            ox800sata_accumulated_RAID_faults |= 1;
        }
        if (test_bit(OX800SATA_INT_ERROR,  &Port1Irq)) {
            printk("disk 1 error in raid\n");
            ox800sata_accumulated_RAID_faults |= 2;
        }
    }
    
	if (!in_irq()) {
		/* wait for the DMA to finish */
		while (oxnas_dma_is_active(PrivateData->DmaChannel)) {
			schedule();
		}
	}

	/* The command may have aborted, this is indicated by the interrupt bit
	 * being masked */   
	 if (PrivateData->in_cleanup) {
		return;
	 }

	 if (!(qc->flags & ATA_QCFLAG_ACTIVE)) {
		 printk(KERN_WARNING "**** QC already completed! ****\n");
		 return;
	 }

    /* get the error status */    
    qc->err_mask = ac_err_mask(ata_chk_status(ap));

    /* tell libata we're done */
    DPRINTK(" returning err_mask=0x%x\n", qc->err_mask);
    local_irq_save(flags);
    PrivateData->int_status = 0;
    local_irq_restore(flags);
    ata_qc_complete(qc);
}
 
/** 
 * ox800sata_irq_clear is called during probe just before the interrupt handler is
 * registered, to be sure hardware is quiet. It clears and masks interrupt bits
 * in the SATA core.
 *
 * @param ap hardware with the registers in
 */
static void ox800sata_irq_clear(struct ata_port* ap)
{
    u32 *ioaddr = ox800sata_get_io_base(ap);
    //DPRINTK(KERN_INFO"ox800sata_irq_clear\n");
    
    writel( ~0, ioaddr + OX800SATA_INT_DISABLE );
    writel( ~0, ioaddr + OX800SATA_INT_CLEAR );
}

static u32  __ox800sata_scr_read(u32* core_addr, unsigned int sc_reg) 
{
    u32 result;
    u32 patience;

    /* we've got 8 other registers in before the start of the standard ones */    
    writel(sc_reg, core_addr + OX800SATA_LINK_RD_ADDR );

    for (patience = 0x100000;patience > 0;--patience)
    {
        if (readl(core_addr + OX800SATA_LINK_CONTROL) & 0x00000001)
            break;
    }

    result = readl(core_addr + OX800SATA_LINK_DATA);
    
    //DPRINTK(KERN_INFO"ox800sata_scr_read: [0x%02x]->0x%08x\n", sc_reg, result);
    return result;
}

/** 
 *  Read standard SATA phy registers. Currently only used if 
 * ->phy_reset hook called the sata_phy_reset() helper function.
 *
 * These registers are in another clock domain to the processor, access is via
 * some bridging registers
 *
 * @param ap hardware with the registers in
 * @param sc_reg the SATA PHY register
 * @return the value in the register
 */
static int ox800sata_scr_read(struct ata_port *ap, unsigned int sc_reg, u32 *val)
{
    u32* ioaddr = ox800sata_get_link_base(ap);
    *val = __ox800sata_scr_read(ioaddr, 0x20 + (sc_reg*4) );
	return 0;
}

static void __ox800sata_scr_write(u32* core_addr, unsigned int sc_reg, u32 val)
{
    u32 patience;

    //DPRINTK(KERN_INFO"ox800sata_scr_write: [0x%02x]<-0x%08x\n", sc_reg, val);
    writel(val, core_addr + OX800SATA_LINK_DATA );
    writel(sc_reg , core_addr + OX800SATA_LINK_WR_ADDR );

    for (patience = 0x100000;patience > 0;--patience)
    {
        if (readl(core_addr + OX800SATA_LINK_CONTROL) & 0x00000001)
            break;
    }
}
/** 
 *  Write standard SATA phy registers. Currently only used if 
 * phy_reset hook called the sata_phy_reset() helper function.
 *
 * These registers are in another clock domain to the processor, access is via
 * some bridging registers
 *
 * @param ap hardware with the registers in
 * @param sc_reg the SATA PHY register
 * @param val the value to write into the register
 */
static int ox800sata_scr_write(struct ata_port *ap, unsigned int sc_reg, u32 val)
{
    u32 *ioaddr = ox800sata_get_link_base(ap);
    __ox800sata_scr_write(ioaddr, 0x20 + (sc_reg * 4), val);
	return 0;
}

/** 
 * port_start() is called just after the data structures for each port are
 * initialized. Typically this is used to alloc per-port DMA buffers, tables
 * rings, enable DMA engines and similar tasks.
 *
 * @return 0 = success
 * @param ap hardware with the registers in
 */
static int  ox800sata_port_start(struct ata_port *ap)
{
    ox800sata_private_data* pd;
    struct device* pdev = ap->host->dev;

    ap->prd = dma_alloc_coherent(pdev, ATA_PRD_TBL_SZ, &ap->prd_dma, GFP_DMA);
    if (!ap->prd) {
        return -ENOMEM;
    }

    /* allocate port private data memory and attach to port */    
    if (!ap->private_data) {
        ap->private_data = kmalloc(sizeof(ox800sata_private_data), GFP_KERNEL);
    }

    if (!ap->private_data) {
        return -ENOMEM;
    }

	pd = (ox800sata_private_data* )ap->private_data;
	pd->DmaChannel = 0;
	pd->sg_entries = 0;

    DPRINTK("ap = %p, pd = %p\n",ap,ap->private_data);

	// Allocate DMA SG entries
	if (oxnas_dma_alloc_sg_entries(&pd->sg_entries, CONFIG_ARCH_OXNAS_MAX_SATA_SG_ENTRIES)) {
		printk(KERN_WARNING "ox800sata_port_start() Failed to obtain DMA SG entries\n");
		return -ENOMEM;
	}

	// Hold on to a DMA channel for the life of the SATA driver
    pd->DmaChannel = oxnas_dma_request(1);
	if (!pd->DmaChannel) {
		printk(KERN_WARNING "ox800sata_port_start() Failed to obtain DMA channel\n");
        return -ENOMEM;
	}

    /* declare a work item to spot when a command finishes */
    INIT_WORK(&(pd->spot_the_end_work.worker), &ox800sata_spot_the_end);

    /* initialise to zero */
    pd->ErrorsWithNoCommamnd = 0;
    pd->port_disabled = 0;
    pd->int_status = 0;
    pd->in_cleanup = 0;

    /* store the ata_port painter in the driver structure (BAD, should really 
    be in the device) */
    if (ox800sata_get_io_base(ap) == (u32*)SATA0_REGS_BASE) {
        ox800sata_driver.ap[0] = ap;
    } else if (ox800sata_get_io_base(ap) == (u32*)SATA1_REGS_BASE) {
        ox800sata_driver.ap[1] = ap;
	}

    // turn ata core on
    writel((1 << SYS_CTRL_CKEN_SATA_BIT), SYS_CTRL_CKEN_SET_CTRL);

    /* post reset init needs to be called for both ports as there's one reset
    for both ports*/
    if (ox800sata_driver.ap[0]) {
        ox800sata_post_reset_init(ox800sata_driver.ap[0]);
	}
    if (ox800sata_driver.ap[1]) {
        ox800sata_post_reset_init(ox800sata_driver.ap[1]);
	}

    return 0;
}

static void ox800sata_post_reset_init(struct ata_port* ap) 
{
    u32  patience;
    u32* link_addr = ox800sata_get_link_base(ap);
    u32* ioaddr = ox800sata_get_io_base(ap);
    uint dev;
    
    /* turn on phy error detection by removing the masks */ 
    writel(0x30003, link_addr + OX800SATA_LINK_DATA );
    wmb();
    writel(0x0C, link_addr + OX800SATA_LINK_WR_ADDR );
    wmb();
    for (patience = 0x100000;patience > 0;--patience)
    {
        if (readl(link_addr + OX800SATA_LINK_CONTROL) & 0x00000001)
            break;
    }
    
    /* Set FIS modes to flush rather than softtrans */
    writel(0xff, ioaddr + OX800SATA_REG_ACCESS);
    
    /* go through all the devices and configure them */
    for (dev = 0; dev < ATA_MAX_DEVICES; ++dev) {
        if ( ap->device[dev].class == ATA_DEV_ATA )
            ox800sata_dev_config( &(ap->device[dev]) );
    }
}

/** 
 * port_stop() is called after ->host_stop(). It's sole function is to 
 * release DMA/memory resources, now that they are no longer actively being
 * used.
 */
static void ox800sata_port_stop(struct ata_port *ap)
{
    ox800sata_private_data* pd = (ox800sata_private_data* )ap->private_data;

    DPRINTK("\n");

	if (pd->DmaChannel) {
		oxnas_dma_free(pd->DmaChannel);
		pd->DmaChannel = 0;
	}

	if (pd->sg_entries) {
		oxnas_dma_free_sg_entries(pd->sg_entries);
		pd->sg_entries = 0;
	}

    kfree(pd);
}

/** 
 * host_stop() is called when the rmmod or hot unplug process begins. The
 * hook must stop all hardware interrupts, DMA engines, etc.
 *
 * @param ap hardware with the registers in
 */
static void ox800sata_host_stop(struct ata_host *host_set)
{
    DPRINTK("\n");
}

/** 
 * PATA device presence detection
 * @param ap ATA channel to examine
 * @param device Device to examine (starting at zero)
 * @return true if something found 
 *
 * This technique was originally described in
 * Hale Landis's ATADRVR (www.ata-atapi.com), and
 * later found its way into the ATA/ATAPI spec.
 * 
 * Write a pattern to the ATA shadow registers,
 * and if a device is present, it will respond by
 * correctly storing and echoing back the
 * ATA shadow register contents.
 * 
 * LOCKING:
 * caller.
 */
static unsigned int ox800sata_devchk(struct ata_port *ap,unsigned int device)
{
    DPRINTK("\n");

    return 0;       /* nothing found */
}

static void ox800sata_pio_start(struct work_struct *work)
{
    u32 burst_reg;
	struct ata_port *ap = container_of(work, struct ata_port, port_task.work);
	ox800sata_private_data* pd = (ox800sata_private_data*)ap->private_data;
    struct ata_queued_cmd* qc = ap->port_task_data;
    u32* ioaddr = ox800sata_get_io_base(ap);
    unsigned long flags = 0;

    // We check for DMA completion from ISR which cannot wait for all DMA channel
	// housekeeping to complete, so need to wait here is case we try to reuse
	// channel before that housekeeping has completed
	while (oxnas_dma_is_active(pd->DmaChannel)) {
		printk(KERN_WARNING "PIO start Channel still active\n");
	}

	if (qc->tf.protocol != ATA_PROT_NODATA) {
		oxnas_dma_direction_t direction = (qc->dma_dir == DMA_FROM_DEVICE) ?
										   OXNAS_DMA_FROM_DEVICE :
										   OXNAS_DMA_TO_DEVICE;

		/* Do not use DMA callback */
		oxnas_dma_set_callback(pd->DmaChannel, OXNAS_DMA_CALLBACK_NUL, OXNAS_DMA_CALLBACK_ARG_NUL);

		/* map memory for dma */
		dma_map_sg(NULL, qc->__sg, qc->n_elem, qc->dma_dir);

		/* setup a scatter gather dma */
		oxnas_dma_device_set_sg(pd->DmaChannel,
								 direction,
								 qc->__sg,
								 qc->n_elem,
								 &oxnas_sata_dma_settings,
								 OXNAS_DMA_MODE_INC);

		oxnas_dma_start(pd->DmaChannel);

		/* turn on the fifo */    
		burst_reg = readl( ioaddr + OX800SATA_BURST_CONTROL );
		burst_reg &= ~OX800SATA_BBC_DREQ_DIS;
		writel(burst_reg, ioaddr + OX800SATA_BURST_CONTROL);

		if (oxnas_dma_is_active(pd->DmaChannel)) {
			/* if the DMA is still busy, schedule a task to poll again in 1 ms */
			ata_port_queue_task(ap, ox800sata_pio_task, qc, ATA_SHORT_PAUSE);
			return;
		}

		/* cleanup DMA */
		dma_unmap_sg(NULL, qc->__sg, qc->n_elem, qc->dma_dir);
	} else {
        /* if the core is still busy, reschedule */
        if (readl(ioaddr + OX800SATA_SATA_COMMAND) & CMD_CORE_BUSY) {
            ata_port_queue_task(ap, ox800sata_pio_task, qc, ATA_SHORT_PAUSE);
            return;
        }
	}

    /* notify of completion */
    PretendDRQIsClear = 1;
    qc->err_mask = ac_err_mask(ata_chk_status(ap));
    spin_lock_irqsave(ap->lock, flags);
    ap->ops->irq_on(ap);
    ata_qc_complete(qc);
    spin_unlock_irqrestore(ap->lock, flags);
}

/**
 * This is the top level of the PIO task. It is responsible for organising the
 * transfer of data, collecting and reacting to status changes and notification
 * of command completion.
 *
 */
static void ox800sata_pio_task(struct work_struct *work)
{
	struct ata_port *ap = container_of(work, struct ata_port, port_task.work);
	struct ata_queued_cmd *qc = ap->port_task_data;
    unsigned long flags = 0;

	if (qc->tf.protocol != ATA_PROT_NODATA) {
		ox800sata_private_data* pd = (ox800sata_private_data* )ap->private_data;

		if (oxnas_dma_is_active(pd->DmaChannel)) {
			/* if the DMA is still busy, re-schedule the task */
			/* try again in 1 ms */
			ata_port_queue_task(ap, ox800sata_pio_task, qc, ATA_SHORT_PAUSE);
			return;
		}

		/* cleanup DMA */
		dma_unmap_sg(NULL, qc->__sg, qc->n_elem, qc->dma_dir);
	} else {
		u32* ioaddr = ox800sata_get_io_base(ap);

        /* if the core is still busy, reschedule */
        if (readl(ioaddr + OX800SATA_SATA_COMMAND) & CMD_CORE_BUSY) {
            ata_port_queue_task(ap, ox800sata_pio_task, qc, ATA_SHORT_PAUSE);
            return;
        }
	}

    /* notify of completion */
    PretendDRQIsClear = 1;
    qc->err_mask = ac_err_mask(ata_chk_status(ap));
    spin_lock_irqsave(ap->lock, flags);
    ap->ops->irq_on(ap);
    ata_qc_complete(qc);
    spin_unlock_irqrestore(ap->lock, flags);
}

static void ox800sata_bmdma_stop(struct ata_queued_cmd *qc)
{
    struct ata_port *ap = qc->ap;
    ox800sata_private_data* private_data = (ox800sata_private_data*)ap->private_data;

    /* Check if DMA is in progress, if so abort */
	if (oxnas_dma_is_active(private_data->DmaChannel)) {
		/* 
		 * Attempt to abort any current transfer:
		 *   Abort DMA transfer at the DMA controller,
		 */
		printk(KERN_ERR "ox800sata_bmdma_stop - aborting DMA\n");

		oxnas_dma_abort(private_data->DmaChannel);

		/* perform core cleanups and resets */
		ox800sata_timeout_cleanup(ap);
	}
}

/**
 *
 */
static void ox800sata_timeout_cleanup( struct ata_port *ap ) {
    u32* io_base  = ox800sata_get_io_base(ap);
    u32* bbp_base = ox800sata_get_bbp_base();
    int idle;
    u32 reg;
    int loops;
    
    /* Test SATA core idle state */
    CrazyDumpDebug(ap);
    idle = !(readl(io_base + OX800SATA_SATA_COMMAND) & CMD_CORE_BUSY);
    

    printk(KERN_ERR "ox800sata_timeout_cleanup() ata%u idle = %d\n", ap->print_id, idle);
    
    if (!idle) {
        /*
         * Assert SATA core and burst buffer port Force_EOT
         */
        printk(KERN_INFO "ox800sata_timeout_cleanup - aborting SATA... (may take upto 5 seconds)\n");

        reg = readl(io_base + OX800SATA_DEVICE_CONTROL);
        reg |= OX800SATA_DEVICE_CONTROL_ABORT;
        writel(reg, io_base + OX800SATA_DEVICE_CONTROL);

        reg = readl(bbp_base + OX800SATA_BURST_CONTROL);
        reg |= OX800SATA_BBC_FORCE_EOT;
        writel(reg, bbp_base + OX800SATA_BURST_CONTROL);

        /* Wait for SATA core to go idle */
        idle = 0;
        loops = SATA_ABORT_WAIT_MS;
        while(1) {
            /* Test SATA core idle state */
            idle = !(readl(io_base + OX800SATA_SATA_COMMAND) & CMD_CORE_BUSY);
            if (idle || (loops-- <= 0)) {
                break;
            }
            /* Wait a millisecond before testing again */
            udelay(1000);
        }

        /* Deassert SATA core abort - BBP Force_EOT is self-clearing` */
        reg = readl(io_base + OX800SATA_DEVICE_CONTROL);
        reg &= ~OX800SATA_DEVICE_CONTROL_ABORT;
        writel(reg, io_base + OX800SATA_DEVICE_CONTROL);

        DPRINTK("idle = %d, %d loops remaining\n", idle, loops);

        if (!idle) {
            /*
             * SATA core did not go idle, so attempt a core reset:
             *   Assert both SATA core internal reset and ORB4 srst
             *   Deassert both SATA core internal reset and ORB4 srst
             */
#if 0
             printk(KERN_INFO "ox800sata_timeout_cleanup - internal SATA reset... (may take upto 5 seconds)\n");

            reg = readl(io_base + OX800SATA_SATA_CONTROL);
            reg |= OX800SATA_SCTL_RESET;
            writel(reg, io_base + OX800SATA_SATA_CONTROL);

            reg = readl(io_base + OX800SATA_ORB4);
            reg |= OX800SATA_ORB4_SRST;
            writel(reg, io_base + OX800SATA_ORB4);

            /* Wait for SATA core to go idle */
            idle = 0;
            loops = SATA_SRST_WAIT_MS;
            while(1) {
                /* Test SATA core idle state */
                idle = !(readl(io_base + OX800SATA_SATA_COMMAND) & CMD_CORE_BUSY);
                if (idle || (loops-- <= 0)) {
                    break;
                }
                /* Wait a millisecond before testing again */
                udelay(1000);
            }

            reg = readl(io_base + OX800SATA_ORB4);
            reg &= ~OX800SATA_ORB4_SRST;
            writel(reg, io_base + OX800SATA_ORB4);

            reg = readl(io_base + OX800SATA_SATA_CONTROL);
            reg &= ~OX800SATA_SCTL_RESET;
            writel(reg, io_base + OX800SATA_SATA_CONTROL);
            udelay(1000);

            DPRINTK("idle = %d, %d loops remaining\n", idle, loops);
#endif
            idle = !(readl(io_base + OX800SATA_SATA_COMMAND) & CMD_CORE_BUSY);
            if (!idle) {
                /*
                 * SATA core did not go idle, so cause a SATA core reset from the RPS
                 *  NB It may be required to reset both SATA cores if have a dual system
                 */
                printk(KERN_INFO "ox800sata_timeout_cleanup - RPS SATA core reset\n");

                writel(1UL << SYS_CTRL_RSTEN_SATA_BIT, SYS_CTRL_RSTEN_SET_CTRL);
                udelay(1000);
                writel(1UL << SYS_CTRL_RSTEN_SATA_BIT, SYS_CTRL_RSTEN_CLR_CTRL);

                /* Read SATA core idle state */
                idle = !(readl(io_base + OX800SATA_SATA_COMMAND) & CMD_CORE_BUSY);

                printk(KERN_INFO"idle = %d\n", idle);

                /* Perform any SATA core re-initialisation after reset */
                /* post reset init needs to be called for both ports as there's one reset
                for both ports*/
                if (ox800sata_driver.ap[0])
                    ox800sata_post_reset_init(ox800sata_driver.ap[0]);
                if (ox800sata_driver.ap[1])
                    ox800sata_post_reset_init(ox800sata_driver.ap[1]);
            }
        }
    }
}

/** 
 * bmdma_status return a made up version of a BMDMA status register
 *
 * @param ap Hardware with the registers in
 * @return the value ATA_DMA_INTR if the interrupt came from the DMA finishing
 */
static u8   ox800sata_bmdma_status(struct ata_port *ap)
{
    ox800sata_private_data* PrivateData ;
    PrivateData = (ox800sata_private_data* )ap->private_data;    

    {
    u32 interrupt_status;
    u32 *ioaddr = ox800sata_get_io_base(ap);
    interrupt_status = readl(ioaddr + OX800SATA_INT_STATUS );
    DPRINTK(" irq bits are %08x \n",interrupt_status);
    }
/*    if( oxnas_dma_is_active( PrivateData->DmaChannel ) )
    {
        CrazyDumpDebug(ap);
        return 0;
    }
    else*/
    {
        return ATA_DMA_INTR;
    }
}
 
/** 
 * turn on the interrupts from the ata drive
 * wait for idle, clear any pending interrupts.
 *
 * @param ap Hardware with the registers in
 */
static u8 ox800sata_irq_on(struct ata_port *ap)
{
    u32* ioaddr = ox800sata_get_io_base(ap);
    u8 tmp;

    //DPRINTK(KERN_INFO"ox800sata_irq_on\n");
    
    /* enable End of command interrupt */
    writel(OX800SATA_INT_END_OF_CMD, ioaddr + OX800SATA_INT_CLEAR);
    writel(OX800SATA_INT_END_OF_CMD, ioaddr + OX800SATA_INT_ENABLE);
	tmp = ata_wait_idle(ap);

    return tmp;
}
 
/** 
 * Acknowledges any pending interrupts, by clearing them, but not disabling 
 * them.
 *
 * @param ap Hardware with the registers in
 */
static u8 ox800sata_irq_ack(struct ata_port *ap, unsigned int chk_drq)
{
    u32* ioaddr = ox800sata_get_io_base(ap);
    unsigned int bits = chk_drq ? ATA_BUSY | ATA_DRQ : ATA_BUSY;
    u8 status;

    //DPRINTK(KERN_INFO"921ish_irq_ack\n");
    status = ata_busy_wait(ap, bits, 1000);
    if (status & bits)
    {
        DPRINTK("abnormal status 0x%X\n", status);
    }

    /* clear the end of command interrupt bit */
    writel(OX800SATA_INT_END_OF_CMD, ioaddr + OX800SATA_INT_CLEAR);

    return status;
}
 
/** 
 * Outputs all the registers in the SATA core for diagnosis of faults.
 *
 * @param ap Hardware with the registers in
 */
static void CrazyDumpDebug(struct ata_port *ap)
{
#ifdef CRAZY_DUMP_DEBUG
    u32 offset;
    u32 result;
    u32 patience;
    u32* ioaddr;

    /* IRQ flags for calling port */
    {
        ox800sata_private_data* PrivateData = (ox800sata_private_data* )ap->private_data;
        printk("IRQ %08x\n",PrivateData->int_status);
    }
    
    /* port 0 */
    ioaddr = (u32* )SATA0_REGS_BASE;
    printk("Port 0 High level registers\n");
    for(offset = 0; offset < 32;offset++)
    {
        printk("[%02x] %08x\n", offset * 4, readl(ioaddr + offset));
    }

    printk("Port 0 link layer registers\n");
    ioaddr = (u32* )SATA0_LINK_REGS_BASE;
    for(offset = 0; offset < 15;++offset)
    {
        writel( (offset*4), ioaddr + OX800SATA_LINK_RD_ADDR );
        wmb();
    
        for (patience = 0x100000;patience > 0;--patience)
        {
            if (readl(ioaddr + OX800SATA_LINK_CONTROL) & 0x00000001)
                break;
        }
    
        result = readl(ioaddr + OX800SATA_LINK_DATA);
        printk("[%02x] %08x\n", offset*4, result);
    }

    /* port 1 */
    ioaddr = (u32* )SATA1_REGS_BASE;
    printk("Port 1 High level registers\n");
    for(offset = 0; offset < 32;offset++)
    {
        printk("[%02x] %08x\n", offset * 4, readl(ioaddr + offset));
    }

    printk("Port 1 link layer registers\n");
    ioaddr = (u32* )SATA1_LINK_REGS_BASE;
    for(offset = 0; offset < 15;++offset)
    {
        writel( (offset*4), ioaddr + OX800SATA_LINK_RD_ADDR );
        wmb();
    
        for (patience = 0x100000;patience > 0;--patience)
        {
            if (readl(ioaddr + OX800SATA_LINK_CONTROL) & 0x00000001)
                break;
        }
    
        result = readl(ioaddr + OX800SATA_LINK_DATA);
        printk("[%02x] %08x\n", offset*4, result);
    }
    
    oxnas_dma_dump_registers();
#endif
}

/**************************************************************************
* DEVICE CODE
**************************************************************************/

/**
 * Describes the identity of the SATA core and the resources it requires
 */ 
static struct resource ox800sata_port0_resources[] = {
	{
        .name       = "sata_port_0_registers",
		.start		= SATA0_REGS_BASE,
        .end        = (SATA0_LINK_REGS_BASE + 64),
		.flags		= IORESOURCE_MEM,
	},
    {
        .name       = "sata_irq",
        .start      = SATA_1_INTERRUPT,
		.flags		= IORESOURCE_IRQ,
    }
};

static struct resource ox800sata_port1_resources[] = {
	{
        .name       = "sata_port_1_registers",
		.start		= SATA1_REGS_BASE,
        .end        = (SATA1_LINK_REGS_BASE + 64),
		.flags		= IORESOURCE_MEM,
	},
    {
        .name       = "sata_irq",
        .start      = SATA_2_INTERRUPT,
		.flags		= IORESOURCE_IRQ,
    },
};

static struct platform_device ox800sata_dev0 = 
{
    .name = DRIVER_NAME,
    .id = 0,
    .num_resources = 2,
	.resource  = ox800sata_port0_resources,
    .dev.coherent_dma_mask = 0xffffffff,
}; 

static struct platform_device ox800sata_dev1 = 
{
    .name = DRIVER_NAME,
    .id = 1,
    .num_resources = 2,
	.resource  = ox800sata_port1_resources,
    .dev.coherent_dma_mask = 0xffffffff,
}; 

/** 
 * module initialisation
 * @return success is 0
 */
static int __init ox800sata_device_init( void )
{
    int ret;

    DPRINTK("\n");

    {
        // register the ata device for the driver to find
        ret = platform_device_register( &ox800sata_dev0 );
        DPRINTK(" %i\n", ret);
    }
    
#ifndef SATA_OXNAS_SINGLE_SATA
    {
        // register the ata device for the driver to find
        ret = platform_device_register( &ox800sata_dev1 );
        DPRINTK(" %i\n", ret);
    }
#endif /* SATA_OXNAS_SINGLE_SATA */

    return ret;
}

/** 
 * module cleanup
 */
static void __exit ox800sata_device_exit( void )
{
    platform_device_unregister( &ox800sata_dev0 );
    platform_device_unregister( &ox800sata_dev1 );
}

/**
 * Returns accumulated RAID faults and then clears the accumulation
 * @return accumulated RAID faults indicated by set bits
 */
int  oxnassata_RAID_faults( void ) {
    int temp = ox800sata_accumulated_RAID_faults;
    ox800sata_accumulated_RAID_faults = 0;
    return temp;
}

/**
 * Returns ox800 port number the request queue is serviced by.
 *
 * @param queue The queue under investigation.
 * @return The ox800 sata port number servicing the queue or -1 if not found.
 */
int oxnassata_get_port_no(struct request_queue* q)
{
    struct ata_port* ap = 0;
    struct scsi_device* sdev = 0;
    
    /* check port 0 */
    ap = ox800sata_driver.ap[0];
    if (ap)
        shost_for_each_device(sdev, ap->scsi_host) {
            if (sdev->request_queue == q) {
                DPRINTK("Queue %p on port 0\n", q);
                return 0;
            }
        }
    
    /* check port 1 */
    ap = ox800sata_driver.ap[1];
    if (ap)
        shost_for_each_device(sdev, ap->scsi_host) {
            if (sdev->request_queue == q) {
                DPRINTK("Queue %p on port 1\n", q);
                return 1;
            }
        }

    /* not found */
    return -1;  
}

/**
 * @return true if all the drives attached to the internal SATA ports use the
 * same LBA size.
 */
int oxnassata_LBA_schemes_compatible( void )
{
    unsigned long flags0 ;
    unsigned long flags1 ;
    struct ata_port* ap ;
    
    /* check port 0 */
    ap = ox800sata_driver.ap[0];
    if (ap)
        flags0 = ap->device[0].flags & ATA_DFLAG_LBA48 ;
    else
        return 0;
    
    /* check port 1 */
    ap = ox800sata_driver.ap[1];
    if (ap)
        flags1 = ap->device[0].flags & ATA_DFLAG_LBA48 ;
    else
        return 0;

    /* compare */
    return (flags0 == flags1);  
}

/** 
 * macros to register intiialisation and exit functions with kernal
 */
module_init(ox800sata_device_init);
module_exit(ox800sata_device_exit);

EXPORT_SYMBOL( oxnassata_RAID_faults );
EXPORT_SYMBOL( oxnassata_get_port_no );
EXPORT_SYMBOL( oxnassata_LBA_schemes_compatible );
