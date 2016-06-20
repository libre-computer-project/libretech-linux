/**************************************************************************
 *
 *  Copyright (c) 2007 Oxford Semiconductor Ltd.
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
 *      ox810sata.c
 *
 *  Abstract:
 *      A driver to interface the 934 based sata core present in the ox810
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

#include <linux/platform_device.h>
#include <asm/arch/hardware.h>
#include <asm/arch/dma.h>
#include <asm/arch/memory.h>
#include <asm/arch/ox810sata.h>

#include <linux/proc_fs.h>

/***************************************************************************
* DEBUG CONTROL
***************************************************************************/
//#define SATA_DEBUG
//#define SATA_DUMP_REGS
//#define SATA_TF_DUMP
//#define DEBUG_EOT_FAILURE
#define ERROR_INJECTION

#define CRAZY_DUMP_DEBUG
#if 0
typedef struct {
    u32 a;
    u32 d;
    u32 w;
} regaccess;
static u32 regindex = 0;
static regaccess regarray[1024];
#endif

#if 0
    #ifdef writel
    #undef writel
    #endif
    #define writel(v,a) {printk("[%p]<=%08x\n",a,v);*((volatile u32*)(a)) = v;} 
    //#define writel(vv,aa) {regarray[regindex].a=(aa); regarray[regindex].d=(vv); regarray[regindex].w=1; ++regindex; regindex &= 1023;*((volatile u32*)(aa)) = (vv);} 
#endif

#if 0
    #ifdef readl
    #undef readl
    #endif
    static inline u32 myreadl(u32 a) {u32 v =(*((volatile u32*)(a)));printk("[%p]=>%08x\n",a,v);return v;}
    //static inline u32 myreadl(u32 a) {u32 v =(*((volatile u32*)(a)));regarray[regindex].a=a; regarray[regindex].d=v; regarray[regindex].w=0; ++regindex; regindex &= 1023;return v;}
    #define readl(a) (myreadl(a))
#endif


#include <linux/libata.h>
/***************************************************************************
* CONSTANTS
***************************************************************************/

#define DRIVER_AUTHOR   "Oxford Semiconductor Ltd."
#define DRIVER_DESC     "934 SATA core controler"
#define DRIVER_NAME     "oxnassata"

#define SATA_ABORT_WAIT_MS 5000
#define SATA_SRST_WAIT_MS  5000

/**************************************************************************
* PROTOTYPES
**************************************************************************/
static int  ox810sata_init_one(struct platform_device *);
static int  ox810sata_remove_one(struct platform_device *);

static void ox810sata_dev_config(struct ata_port *, struct ata_device *);
static void ox810sata_set_piomode(struct ata_port *, struct ata_device *);
static void ox810sata_set_dmamode(struct ata_port *, struct ata_device *);
static void ox810sata_tf_load(struct ata_port *ap, const struct ata_taskfile *tf);
static void ox810sata_tf_read(struct ata_port *ap, struct ata_taskfile *tf);
static void ox810sata_exec_command(struct ata_port *ap, const struct ata_taskfile *tf);
static u8 ox810sata_check_status(struct ata_port *ap);
static inline u8 ox810sata_check_altstatus(struct ata_port *ap);
static void ox810sata_dev_select(struct ata_port *ap, unsigned int device);
static void ox810sata_phy_reset(struct ata_port *ap);
static void ox810sata_bmdma_setup(struct ata_queued_cmd *qc);
static void ox810sata_bmdma_start(struct ata_queued_cmd *qc);
static u8 ox810sata_bmdma_status(struct ata_port *ap);
static struct ata_queued_cmd* ox810sata_qc_new(struct ata_port *ap);
static void ox810sata_qc_free(struct ata_queued_cmd *qc);
static unsigned int ox810sata_qc_issue(struct ata_queued_cmd *qc);
static void ox810sata_eng_timeout(struct ata_port *ap);
static irqreturn_t ox810sata_irq_handler(int, void *);
static void ox810sata_eng_timeout(struct ata_port *ap);
static void ox810sata_irq_clear(struct ata_port *);
static int  ox810sata_scr_read(struct ata_port *ap, unsigned int sc_reg, u32 *val);
static int  ox810sata_scr_write(struct ata_port *ap, unsigned int sc_reg, u32 val);
static int ox810sata_port_start(struct ata_port *ap);
static void ox810sata_port_stop(struct ata_port *ap);
static void ox810sata_host_stop(struct ata_host *host_set);
static unsigned int ox810sata_devchk(struct ata_port *ap,unsigned int device);
static u32* ox810sata_get_io_base(struct ata_port* ap);
static u8 ox810sata_irq_on(struct ata_port *ap);
static void ox810sata_bmdma_stop(struct ata_queued_cmd *qc);
static void CrazyDumpDebug( void );
static void ox810sata_spot_the_end(struct work_struct *work);
static void ox810sata_timeout_cleanup(struct ata_port *ap);
static void ox810sata_error_handler(struct ata_port *ap);
static void ox810sata_reset_core(void);
static int ox810sata_prereset(struct ata_link *link, unsigned long deadline);
static int ox810sata_hardreset(struct ata_link *link, unsigned int *class, unsigned long deadline);
static void ox810sata_postreset(struct ata_link *link, unsigned int *classes);
static void ox810sata_pio_start(struct work_struct *work);
static void ox810sata_pio_task(struct work_struct *work);
static void ox810sata_post_reset_init(struct ata_port* ap);
static u32 inline __ox810sata_scr_read(u32* core_addr, unsigned int sc_reg);
static void __ox810sata_scr_write(u32* core_addr, unsigned int sc_reg, u32 val);
#ifdef ERROR_INJECTION
static int ox810sata_error_inject_show(char *page, char **start, off_t off, int count, int *eof, void *data);
static int ox810sata_error_inject_store(struct file *file,const char __user *buffer,unsigned long count,void *data);
#endif

/**************************************************************************
* STRUCTURES
**************************************************************************/
typedef struct
{
    struct kobject kobj;
    struct platform_driver driver;
    struct ata_port* ap[2];
    u32 error_inject;
    struct workqueue_struct* spot_the_end_q;
    u32 hw_raid_active;
    struct ata_port* active_port;
} ox810sata_driver_t;

/**
 * Struct to hold per-port private (specific to this driver) data (still 
 * un-researched).
 */
typedef struct
{
    oxnas_dma_channel_t* DmaChannel;
	oxnas_dma_sg_entry_t* sg_entries;
    struct spot_the_end_work_s  {
        struct work_struct worker;
        struct ata_port* ap;
    } spot_the_end_work;
    u32 ErrorsWithNoCommamnd;
    u32 int_status;
    u32 in_cleanup;
} ox810sata_private_data;

ox810sata_driver_t ox810sata_driver = 
{
    .driver =
    {
        .driver.name = DRIVER_NAME,
        .driver.bus = &platform_bus_type,
        .probe = ox810sata_init_one, 
        .remove = ox810sata_remove_one,
    },
    .ap = {0,0},
    .error_inject = 0,
    .hw_raid_active = 0,
    .active_port = 0,
};

/** If we were writing this in C++ then we would be deriving a subclass of 
ata_port, these would be the overridden functions*/
static struct ata_port_operations ox810sata_port_ops =
{
    .dev_config = ox810sata_dev_config,
    .set_piomode = ox810sata_set_piomode,
    .set_dmamode = ox810sata_set_dmamode,
    .tf_load = ox810sata_tf_load,
    .tf_read = ox810sata_tf_read,
    .exec_command = ox810sata_exec_command,
    .check_status = ox810sata_check_status,
    .check_altstatus = ox810sata_check_altstatus,
    .dev_select = ox810sata_dev_select,
    .phy_reset = ox810sata_phy_reset,
    .bmdma_setup = ox810sata_bmdma_setup,
    .bmdma_start = ox810sata_bmdma_start,
    .bmdma_stop = ox810sata_bmdma_stop,
    .bmdma_status = ox810sata_bmdma_status,
    .qc_new = ox810sata_qc_new,
    .qc_free = ox810sata_qc_free,
    .qc_prep = ata_qc_prep,
    .qc_issue = ox810sata_qc_issue,
    .eng_timeout = ox810sata_eng_timeout,
    .irq_handler = ox810sata_irq_handler,
    .irq_clear = ox810sata_irq_clear,
    .scr_read = ox810sata_scr_read,
    .scr_write = ox810sata_scr_write,
    .port_start = ox810sata_port_start,
    .port_stop = ox810sata_port_stop,
    .host_stop = ox810sata_host_stop,
    .dev_chk = ox810sata_devchk,
    .irq_on = ox810sata_irq_on,
    .pio_task = ox810sata_pio_start,
    .error_handler = ox810sata_error_handler,
    .post_internal_cmd = ox810sata_bmdma_stop,
};

/** the scsi_host_template structure describes the basic capabilities of libata
and our 921 core to the SCSI framework, it contains the addresses of functions 
in the libata library that handle top level comands from the SCSI library */
static struct scsi_host_template ox810sata_sht = 
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
ox810sata_check_status operation */  
static char PretendDRQIsClear;

/**
 * used as a store for atomic test and set operations used to coordinate so
 * that only one port is processing comnmands at any time */
static unsigned long ox810sata_command_active = 0;

/**
 * A record of which drives have accumulated raid faults. A set bit indicates
 * a fault has occured on that drive */
static u32 ox810sata_accumulated_RAID_faults = 0;

/**************************************************************************/
MODULE_LICENSE("GPL");
MODULE_VERSION(1.0);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);

/**************************************************************************
* FUCTIONS
* prefix all with "ox810sata_"
**************************************************************************/

/**
 * Gets the base of the ox810 port associated with the ata-port as known
 * by lib-ata, The value returned changes to the single RAID port when
 * hardware RAID commands are active.
 * 
 * @param ap pointer to the appropriate ata_port structure
 * @return the base address of the SATA core
 */
static inline u32* ox810sata_get_tfio_base(struct ata_port* ap)
{
    if ((ox810sata_driver.hw_raid_active) &&
        (ox810sata_driver.active_port == ap)) {
        return (u32* )SATARAID_REGS_BASE;
    } else {
        return (u32* )ap->host->iomap;
    }
}

/**
 * Gets the base address of the ata core from the ata_port structure. The value
 * returned will remain the same when hardware raid is active.
 *
 * @param ap pointer to the appropriate ata_port structure
 * @return the base address of the SATA core
 */
static inline u32* ox810sata_get_io_base(struct ata_port* ap)
{
    return (u32* )ap->host->iomap;
}

/**
 * Turns on the cores clock and resets it
 */
static void ox810sata_reset_core( void ){
    // Enable the clock to the SATA block
    writel(1UL << SYS_CTRL_CKEN_SATA_BIT, SYS_CTRL_CKEN_SET_CTRL);
    wmb();

    // reset Controller, Link and PHY
    writel( (1UL << SYS_CTRL_RSTEN_SATA_BIT)      |
            (1UL << SYS_CTRL_RSTEN_SATA_LINK_BIT) |
            (1UL << SYS_CTRL_RSTEN_SATA_PHY_BIT), SYS_CTRL_RSTEN_SET_CTRL);
    wmb();
    udelay(50);
    
    // un-reset the PHY, then Link and Controller
    writel(1UL << SYS_CTRL_RSTEN_SATA_PHY_BIT, SYS_CTRL_RSTEN_CLR_CTRL);
    udelay(50);
    writel( (1UL << SYS_CTRL_RSTEN_SATA_LINK_BIT) |
            (1UL << SYS_CTRL_RSTEN_SATA_BIT), SYS_CTRL_RSTEN_CLR_CTRL);
    udelay(50);
}

/**
 * port capabilities for the ox810 sata ports.
 */
static const struct ata_port_info ox810sata_port_info = {
    .flags = ATA_FLAG_SATA | ATA_FLAG_SATA_RESET | ATA_FLAG_NO_LEGACY,
    .pio_mask   = 0x1f, /* pio modes 0..4*/
    .mwdma_mask = 0x07, /* mwdma0-2 */
    .udma_mask  = 0x7f, /* udma0-5 */
    .port_ops   = &ox810sata_port_ops,
};

/** 
 * The driver probe function.
 * Registered with the amba bus driver as a parameter of ox810sata_driver.bus
 * it will register the ata device with kernel first performing any 
 * initialisation required (if the correct device is present).
 * @param pdev Pointer to the 921 device structure 
 * @return 0 if no errors
 */
static int ox810sata_init_one(struct platform_device* pdev)
{
    u32 version;
#ifdef CONFIG_SATA_OXNAS_DISK_LIGHT
    unsigned long reg;
#endif // CONFIG_SATA_OXNAS_DISK_LIGHT
    struct ata_host *host;
    void __iomem* iomem;
    const struct ata_port_info* port_info[] = { &ox810sata_port_info, NULL };
    struct resource* memres = platform_get_resource(pdev, IORESOURCE_MEM, 0 );
    int irq = platform_get_irq(pdev, 0);
    
    /* check resourses for sanity */
    if ((memres == NULL) || (irq < 0)) {
        return 0;
    }
    iomem = (void __iomem* ) memres->start;
    
    /* check we support this version of the core */
    version = readl(((u32* )iomem) + OX810SATA_VERSION);
    switch (version) {
        case OX810SATA_CORE_VERSION:
            printk(KERN_INFO"ox810sata: OX810 sata core.\n");   
            break;
        default:
            printk(KERN_ERR"ox810sata: unknown sata core (version register = 0x%08x)\n",version);     
            return 0;
            break;
    }

    /* initialise a work queue to spot the end of transfers */
    ox810sata_driver.spot_the_end_q = create_singlethread_workqueue("sata-endQ");
    if (!ox810sata_driver.spot_the_end_q) {
        printk(KERN_ERR DRIVER_NAME " Couldn't create a work queue.\n");
        return -1;
    }

#ifdef CONFIG_SATA_OXNAS_DISK_LIGHT
    /* setup path */
    reg = ~(1 << CONFIG_OX810SATA_DISK_LIGHT_GPIO_LINE) & readl(SYS_CTRL_GPIO_PRIMSEL_CTRL_0);
    writel(reg, SYS_CTRL_GPIO_PRIMSEL_CTRL_0);
    reg = ~(1 << CONFIG_OX810SATA_DISK_LIGHT_GPIO_LINE) & readl(SYS_CTRL_GPIO_SECSEL_CTRL_0);
    writel(reg, SYS_CTRL_GPIO_SECSEL_CTRL_0);
    reg = ~(1 << CONFIG_OX810SATA_DISK_LIGHT_GPIO_LINE) & readl(SYS_CTRL_GPIO_TERTSEL_CTRL_0);
    writel(reg, SYS_CTRL_GPIO_TERTSEL_CTRL_0);

    /* enable output */
    writel(1 << CONFIG_OX810SATA_DISK_LIGHT_GPIO_LINE, GPIO_A_OUTPUT_ENABLE);

    /* disk light off */
    writel(1 << CONFIG_OX810SATA_DISK_LIGHT_GPIO_LINE, GPIO_A_OUTPUT_CLEAR);
#endif  /* CONFIG_SATA_OXNAS_DISK_LIGHT */

    /* setup the probe_ent structure which is basically info about the ports 
    capabilities */

    /* allocate memory and check */
    host = ata_host_alloc_pinfo(&(pdev->dev), port_info, OX810SATA_MAX_PORTS);
    if (!host) {
        printk(KERN_ERR DRIVER_NAME " Couldn't create an ata host.\n");
        destroy_workqueue(ox810sata_driver.spot_the_end_q);
    }

    /* set to base of ata core */
    host->iomap  = iomem;

    /* call ata_device_add and begin probing for drives*/
    ata_host_activate(host, irq, ox810sata_irq_handler, IRQF_SHARED, &ox810sata_sht);

    return 0;
}

/** 
 * Called when the amba bus tells this device to remove itself.
 * @param pdev pointer to the device that needs to be shutdown
 */
static int ox810sata_remove_one(struct platform_device* pdev)
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
static int __init ox810sata_init( void )
{
    int ret;
    
    ret = platform_driver_register( &ox810sata_driver.driver );
    DPRINTK(" %i\n", ret);    
    
#ifdef ERROR_INJECTION
{
    struct proc_dir_entry *res=create_proc_entry("ox810sata_errorinject",0,NULL);
	if (res) {
		res->read_proc=ox810sata_error_inject_show;
        res->write_proc=ox810sata_error_inject_store;
		res->data=NULL;
	}
    //create_proc_read_entry("ox810sata_errorinject", 0, NULL, ox810sata_error_inject_show, NULL);
}   
#endif
    return ret; 
}

/** 
 * module cleanup
 */
static void __exit ox810sata_exit( void )
{
    platform_driver_unregister( &ox810sata_driver.driver );
}

/** 
 * macros to register intiialisation and exit functions with kernal
 */
module_init(ox810sata_init);
module_exit(ox810sata_exit);

/** 
 * Called after IDENTIFY [PACKET] DEVICE is issued to each device found.
 * Typically used to apply device-specific fixups prior to issue of
 * SET FEATURES - XFER MODE, and prior to operation.
 * @param port The port to configure
 * @param pdev The hardware associated with controlling the port
 */
static void ox810sata_dev_config(struct ata_port *ap, struct ata_device* pdev)
{
    u32 reg;
    u32 *ioaddr = ox810sata_get_io_base(ap);

    /* Set the bits to put the interface into 28 or 48-bit node */
    reg = readl(ioaddr + OX810SATA_DRIVE_CONTROL);

    /* mask out the pair of bits associaed with each port */
    reg &= ~(3 << (ap->port_no * 2));

    /* set the mode pair associated with each port */
    reg |= ((pdev->flags & ATA_DFLAG_LBA48) ? OX810SATA_DR_CON_48 :
                                              OX810SATA_DR_CON_28) << (ap->port_no * 2);
    writel(reg, ioaddr + OX810SATA_DRIVE_CONTROL);

    /* if this is an ATA-6 disk, put the port into ATA-5 auto translate mode */
    if (pdev->flags & ATA_DFLAG_LBA48) {
        reg = readl(ioaddr + OX810SATA_PORT_CONTROL);
        reg |= 2;
        writel(reg, ioaddr + OX810SATA_PORT_CONTROL);
    }
}

/**
 * nothing to do
 * 
 * @param port The port to configure
 * @param pdev The hardware associated with controlling the port
 */
static void ox810sata_set_piomode(struct ata_port* port, struct ata_device* pdev)
{
}

/** 
 * nothing to do
 * 
 * @param port The port to configure
 * @param pdev The hardware associated with controlling the port
 */
static void ox810sata_set_dmamode(struct ata_port* port, struct ata_device* pdev)
{
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
    } else {
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
static void ox810sata_tf_load(struct ata_port *ap, const struct ata_taskfile *tf)
{
    u32 count = 0;
    u32 Orb1 = 0; 
    u32 Orb2 = 0; 
    u32 Orb3 = 0;
    u32 Orb4 = 0;
    u32 Command_Reg;
    u32 *ioaddr = ox810sata_get_tfio_base(ap);
    unsigned int is_addr = tf->flags & ATA_TFLAG_ISADDR;

    /* wait a maximum of 10ms for the core to be idle */
    do {
        Command_Reg = readl(ioaddr + OX810SATA_SATA_COMMAND);
        if (!(Command_Reg & CMD_CORE_BUSY)) {
            break;
		}
        count++;
		udelay(50);
    } while (count < 200);

    /* if the control register has changed, write it */
    if (tf->ctl != ap->last_ctl) 
    {
        //DPRINTK("ap->last_ctl = %02x",ap->last_ctl);
        Orb4 |= (tf->ctl) << 24;

        /* write value to register */
        writel(Orb4, ioaddr + OX810SATA_ORB4);

        ap->last_ctl = tf->ctl;

        /** @todo find a more elegant way to do this */
        /* if the new control value is a soft reset, command the core to send a
        control FIS */
        if (tf->ctl & ATA_SRST) {
            writel(CMD_WRITE_TO_ORB_REGS_NO_COMMAND, ioaddr + OX810SATA_SATA_COMMAND);
        }
    }

    /* check if the ctl register has interrupts disabled or enabled and
    modify the interrupt enable registers on the ata core as required */
    if (tf->ctl & ATA_NIEN) {
        /* interrupts disabled */
        ox810sata_irq_clear(ap);
    } else {
        /* interrupts enabled */
        ox810sata_irq_on(ap);
    }

    /* write 48 or 28 bit tf parameters */
    if (is_addr) {
        /* set LBA bit as it's an address */
        Orb1 |= (tf->device & ATA_LBA) << 24;

        if (tf->flags & ATA_TFLAG_LBA48) {
            //DPRINTK(KERN_INFO" 48 bit tf load \n");
            Orb1 |= ATA_LBA << 24;

            Orb2 |= (tf->hob_nsect)  << 8 ;

            Orb3 |= (tf->hob_lbal)   << 24;

            Orb4 |= (tf->hob_lbam)   << 0 ;
            Orb4 |= (tf->hob_lbah)   << 8 ;
            Orb4 |= (tf->hob_feature)<< 16;
        } else {
            Orb3 |= (tf->device & 0xf)<< 24;
        }

        /* write 28-bit lba */
        //DPRINTK(KERN_INFO" 28 bit tf load\n");
        Orb2 |= (tf->nsect)      << 0 ;
        Orb2 |= (tf->feature)    << 16;
        Orb2 |= (tf->command)    << 24;

        Orb3 |= (tf->lbal)       << 0 ;
        Orb3 |= (tf->lbam)       << 8 ;
        Orb3 |= (tf->lbah)       << 16;

        Orb4 |= (tf->ctl)        << 24;

        /* write values to registers */
        writel(Orb1, ioaddr + OX810SATA_ORB1 );
        writel(Orb2, ioaddr + OX810SATA_ORB2 );
        writel(Orb3, ioaddr + OX810SATA_ORB3 );
        writel(Orb4, ioaddr + OX810SATA_ORB4 );
    }

    if (tf->flags & ATA_TFLAG_DEVICE) {
        Orb1 |= (tf->device) << 24;

        /* write value to register */
        writel(Orb1, ioaddr + OX810SATA_ORB1);
    }

    tfdump(tf);
}

/** 
 * Called to read the hardware registers / DMA buffers, to
 * obtain the current set of taskfile register values.
 * @param ap hardware with the registers in
 * @param tf taskfile to read the registers into
 */
static void ox810sata_tf_read(struct ata_port *ap, struct ata_taskfile *tf)
{
    u32 *ioaddr = ox810sata_get_tfio_base(ap);

    /* read the orb registers */
    u32 Orb1 = readl(ioaddr + OX810SATA_ORB1); 
    u32 Orb2 = readl(ioaddr + OX810SATA_ORB2); 
    u32 Orb3 = readl(ioaddr + OX810SATA_ORB3);
    u32 Orb4 = readl(ioaddr + OX810SATA_ORB4);

    /* read common 28/48 bit tf parameters */
    tf->device  = (Orb1 >> 24);
    tf->nsect   = (Orb2 >> 0);
    tf->feature = (Orb2 >> 16);
    tf->command = ox810sata_check_status(ap);

    /* read 48 or 28 bit tf parameters */
    if (tf->flags & ATA_TFLAG_LBA48) {
        //DPRINTK(KERN_INFO" 48 bit tf read \n");
        tf->hob_nsect = (Orb2 >> 8) ;
        
        tf->lbal      = (Orb3 >> 0) ;
        tf->lbam      = (Orb3 >> 8) ;
        tf->lbah      = (Orb3 >> 16) ;
        tf->hob_lbal  = (Orb3 >> 24) ;
        
        tf->hob_lbam  = (Orb4 >> 0) ;
        tf->hob_lbah  = (Orb4 >> 8) ;
        /* feature ext and control are write only */
    } else {
        /* read 28-bit lba */
        //DPRINTK(KERN_INFO" 28 bit tf read\n");
        tf->lbal      = (Orb3 >> 0) ;
        tf->lbam      = (Orb3 >> 8) ;
        tf->lbah      = (Orb3 >> 16) ;
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
static void ox810sata_exec_command(struct ata_port *ap, const struct ata_taskfile *tf)
{
    u32 count =0;
    u32 *ioaddr = ox810sata_get_tfio_base(ap);
    u32 Orb2;
    u32 Command_Reg;

#ifdef ERROR_INJECTION
    static u32 prand = 10;
    if (ox810sata_driver.error_inject) {
        u32 *portaddr = ox810sata_get_io_base(ap);
        prand = prand ? prand - 1 : 100;
        if ( prand < ox810sata_driver.error_inject) {
            DPRINTK("ox810sata_exec_command: error injection on\n");
            __ox810sata_scr_write( portaddr, 0x14 , 0xd );
        } else {
            __ox810sata_scr_write( portaddr, 0x14 , 0x1 );
            DPRINTK("ox810sata_exec_command: error injection off\n");
        }
    }
#endif

    VPRINTK("\n");
    /* Wait a maximum af 10ms for the core to go idle */
    do {
        Command_Reg = readl(ioaddr + OX810SATA_SATA_COMMAND);
        if (!(Command_Reg & CMD_CORE_BUSY)) {
            break;
		}
        count++;
        udelay(50);
    } while (count < 200);

    /* write all the things in Orb 2 */
    Orb2  = (tf->nsect) << 0 ;
    if (tf->flags & ATA_TFLAG_LBA48) {
        Orb2 |= (tf->hob_nsect) << 8;
    }
    Orb2 |= (tf->feature) << 16;
    Orb2 |= (tf->command) << 24;
    writel(Orb2 , ioaddr + OX810SATA_ORB2);
    wmb();

    do {
        Command_Reg = readl(ioaddr + OX810SATA_SATA_COMMAND);
        if (!(Command_Reg & CMD_CORE_BUSY)) {
            break;
		}
        count++;
        udelay(50);
    } while (count < 200);

    /* Command that the orb registers get written to drive */
    Command_Reg &= ~SATA_OPCODE_MASK;
    Command_Reg |= CMD_WRITE_TO_ORB_REGS;
    writel(Command_Reg , ioaddr + OX810SATA_SATA_COMMAND);
    wmb();
}


/** 
 * Reads the Status ATA shadow register from hardware. Due to a fault with PIO
 * transfers, it it sometimes necessary to mask out the DRQ bit
 * @param ap hardware with the registers in
 * @return The status register
 */
static u8 ox810sata_check_status(struct ata_port *ap)
{
    u32 Reg;
    u8 status;
    u32 *ioaddr = ox810sata_get_tfio_base(ap);

//    VPRINTK(KERN_INFO"ox810sata_check_status ");

    /* read byte 3 of Orb2 register */
    status = readl(ioaddr + OX810SATA_ORB2) >> 24;

    /* check for the drive going missing indicated by SCR status bits 0-3 = 0 */
    if ( ox810sata_driver.hw_raid_active ) {
        u32 Temp;
        ox810sata_scr_read(ox810sata_driver.ap[0], SCR_STATUS, &Temp );
        ox810sata_scr_read(ox810sata_driver.ap[1], SCR_STATUS, &Reg);
        Reg |= Temp;
    } else {
        ox810sata_scr_read(ap, SCR_STATUS, &Reg );
    }
    if (!(Reg & 0x1)) { 
        status |= ATA_DF;
        status |= ATA_ERR;
    }
    //VPRINTK("%02x\n",result);

    return status;
}

/** 
 * Reads the alt status ATA shadow register from hardware. 
 * @param ap hardware with the registers in
 * @return The alt status register
 */
static u8 ox810sata_check_altstatus(struct ata_port *ap)
{
    return readl(ox810sata_get_tfio_base(ap) + OX810SATA_ORB4) >> 24;
}

/** 
 * Use the method defined in the ATA specification to make either device 0,
 * or device 1, active on the ATA channel. If we ever get port multipliers
 * to work, this will be where they would switch.
 *
 * @param ap hardware with the registers in
 * @param number of the device to talk to (0..)
 */
static void ox810sata_dev_select(struct ata_port *ap, unsigned int device)
{
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
static void ox810sata_phy_reset(struct ata_port *ap)
{
    u32 *ioaddr = ox810sata_get_io_base(ap);

    DPRINTK(KERN_INFO "base = %p\n", ioaddr);

    /* turn ata core on */
    writel((1 << SYS_CTRL_CKEN_SATA_BIT), SYS_CTRL_CKEN_SET_CTRL);

    /* stop all the interrupts in the ata core */
    writel(~0, ioaddr + OX810SATA_INT_DISABLE);
    writel(~0, ioaddr + OX810SATA_INT_CLEAR);

    /* get libata to perform a soft reset */
	ata_port_probe(ap);
    ata_bus_reset(ap);
}

/** 
 * When setting up an IDE BMDMA transaction, these hooks arm (->bmdma_setup) 
 * and fire (->bmdma_start) the hardware's DMA engine.
 *
 * @param qc the queued command to issue
 */
static void ox810sata_bmdma_setup(struct ata_queued_cmd *qc)
{
    ox810sata_private_data* PrivateData ;
    oxnas_dma_direction_t direction; 

#ifdef SATA_DEBUG
    printk(KERN_INFO"ox810sata_bmdma_setup: %s, %d element%s\n", (qc->dma_dir == DMA_FROM_DEVICE) ? "Read" : "Write", qc->n_elem, qc->n_elem ? "s" : "");
#else // SATA_DEBUG
    DPRINTK(" %s, %d element%s\n", (qc->dma_dir == DMA_FROM_DEVICE) ? "Read" : "Write", qc->n_elem, qc->n_elem ? "s" : "");
#endif // SATA_DEBUG

    qc->private_data = qc->ap->private_data;
    PrivateData = (ox810sata_private_data* )qc->private_data;

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
static void ox810sata_bmdma_start(struct ata_queued_cmd *qc)
{
    ox810sata_private_data* PrivateData ;
    DPRINTK("\n");
    PrivateData = (ox810sata_private_data*)(qc->private_data);    

	/* start DMA transfer */
	oxnas_dma_start(PrivateData->DmaChannel);
	qc->ap->ops->exec_command(qc->ap, &(qc->tf));
}


/**
 *  ata_qc_new - Request an available ATA command, for queueing
 *  @ap: Port associated with device @dev
 *  @dev: Device from whom we request an available command structure
 *
 *  LOCKING:
 */

static struct ata_queued_cmd* ox810sata_qc_new(struct ata_port *ap)
{
    struct ata_queued_cmd *qc = NULL;

    VPRINTK("\n");
	/* no command while frozen */
	if (unlikely(ap->pflags & ATA_PFLAG_FROZEN))
		return NULL;
    
    /* check to see that there are no internal commands active on either port*/
    if ((ox810sata_driver.ap[0]) && (ox810sata_driver.ap[0]->qc_allocated))
        return qc;
    if ((ox810sata_driver.ap[1]) && (ox810sata_driver.ap[1]->qc_allocated))
        return qc;

    /* check if we're not doing a command */
    if (test_and_set_bit(0, &ox810sata_command_active))
        return qc;

    /* now set the standard bits for compatibility */
    set_bit(0, &ap->qc_allocated); 
    qc = __ata_qc_from_tag(ap, 0);

#ifdef CONFIG_SATA_OXNAS_DISK_LIGHT
    /* disk light on */
    writel(1 << CONFIG_OX810SATA_DISK_LIGHT_GPIO_LINE, GPIO_A_OUTPUT_SET);
#endif  // CONFIG_SATA_OXNAS_DISK_LIGHT
#ifdef CONFIG_WDC_LEDS_TRIGGER_SATA_DISK
    wdc_ledtrig_sata_activity();
#endif // CONFIG_WDC_LEDS_TRIGGER_SATA_DISK

	if (qc) {
        qc->tag = 0;
	}

    return qc;
}


/**
 *
 */
static void ox810sata_qc_free(struct ata_queued_cmd *qc)
{
    unsigned int tag;
    struct ata_port *ap = qc->ap;

    DPRINTK("\n");

    qc->flags = 0;
    tag = qc->tag;
    if (likely(ata_tag_valid(tag))) {
        qc->tag = ATA_TAG_POISON;
        VPRINTK("clearing active bits\n");
        clear_bit(tag, &ap->qc_allocated);
        clear_bit(0, &ox810sata_command_active);

#ifdef CONFIG_SATA_OXNAS_DISK_LIGHT            
        /* disk light off */
        writel(1 << CONFIG_OX810SATA_DISK_LIGHT_GPIO_LINE, GPIO_A_OUTPUT_CLEAR);
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
static unsigned int ox810sata_qc_issue(struct ata_queued_cmd *qc)
{
    int this_port_fail;
    struct bio* bio;
    u32 reg;
    ox810sata_private_data* private_data = (ox810sata_private_data*)qc->ap->private_data ;
    u32 raid_reg = 0; /* default to no raid */
    int port0fail = 0;
    int port1fail = 0;
    unsigned long flags = 0;

    DPRINTK("\n");

    /* get raid settings from the bio if they exist */    
    if (qc->scsicmd && qc->scsicmd->request && qc->scsicmd->request->bio) {
        bio = qc->scsicmd->request->bio;
        raid_reg = bio->bi_raid ;
        //if (raid_reg) {printk("Hardware RAID :");tfdump(&qc->tf);}
        
    }
    ox810sata_driver.hw_raid_active = raid_reg;
    ox810sata_driver.active_port = qc->ap;

    /* check cable is still connected */
	ox810sata_scr_read(qc->ap, SCR_STATUS, &reg);
    this_port_fail = (!(reg & 1));

    /* check for failed ports prior to issuing raid-ed commands */ 
    if (raid_reg) {
        port0fail = (! (__ox810sata_scr_read((u32* )SATA0_REGS_BASE, 0x20 + (4 * SCR_STATUS) ) & 1 ) ); 
        port1fail = (! (__ox810sata_scr_read((u32* )SATA1_REGS_BASE, 0x20 + (4 * SCR_STATUS) ) & 1 ) );
        this_port_fail |= port1fail;
        
        ox810sata_accumulated_RAID_faults |= port0fail ? 1 : 0 ;
        ox810sata_accumulated_RAID_faults |= port1fail ? 2 : 0 ;
    }
    
    if (!this_port_fail) {
        /* clear phy/link errors */
        if (ox810sata_driver.ap[0])
            ox810sata_scr_write(ox810sata_driver.ap[0], SCR_ERROR, ~0);
        if (ox810sata_driver.ap[1])
            ox810sata_scr_write(ox810sata_driver.ap[1], SCR_ERROR, ~0);
        
        /* clear errors on both hosts */
        reg = readl((u32* )SATA0_REGS_BASE + OX810SATA_SATA_CONTROL);
        reg |= OX810SATA_SCTL_CLR_ERR ;
        writel(reg, (u32* )SATA0_REGS_BASE + OX810SATA_SATA_CONTROL);
        reg = readl((u32* )SATA1_REGS_BASE + OX810SATA_SATA_CONTROL);
        reg |= OX810SATA_SCTL_CLR_ERR ;
        writel(reg, (u32* )SATA1_REGS_BASE + OX810SATA_SATA_CONTROL);
        reg = readl((u32* )SATARAID_REGS_BASE + OX810SATA_SATA_CONTROL);
        reg |= OX810SATA_SCTL_CLR_ERR ;
        writel(reg, (u32* )SATARAID_REGS_BASE + OX810SATA_SATA_CONTROL);
        
        /* clear all interrupt bits */
        writel(~0, (u32* )SATA0_REGS_BASE + OX810SATA_INT_CLEAR);
        writel(~0, (u32* )SATA1_REGS_BASE + OX810SATA_INT_CLEAR);
        writel(~0, (u32* )SATARAID_REGS_BASE + OX810SATA_INT_CLEAR);
        writel(~0, OX810SATA_CORE_IEC );  
            
        /* hardware RAID */
        if ( raid_reg ) {
            /* set interrupt mode for raid controller interrupts only */
            writel(OX810SATA_RAID_INTS_WANTED, OX810SATA_CORE_IES );  

            /* at the moment we only do raid-1 */
            writel(OXNASSATA_RAID1, OX810SATA_RAID_CONTROL);
            writel(OXNASSATA_RAID_TWODISKS, OX810SATA_RAID_SET);
        } else {
            /* set normal interrupt scheme */
            writel(OX810SATA_NORMAL_INTS_WANTED, OX810SATA_CORE_IES);  

            /* Set the RAID controller hardware to idle */
            writel(OXNASSATA_NOTRAID, OX810SATA_RAID_CONTROL);
        }

        /* call the default, this should be changed to take advantage of orb
        registers, etc... */
        return ata_qc_issue_prot(qc);
    } else {
        /* record the error */
        qc->err_mask |= AC_ERR_ATA_BUS;

        /* maybee call ata qc complete? */
        local_irq_save(flags);
        ox810sata_irq_clear(qc->ap);
        private_data->int_status = 0;
        local_irq_restore(flags);
        ata_qc_complete(qc);
    
        return 0;
    }
}

/** 
 * This is a high level error handling function, called from the error
 * handling thread, when a command times out.
 *
 * @param ap hardware with the registers in
 */
static void ox810sata_eng_timeout(struct ata_port *ap)
{
    struct ata_queued_cmd *qc;
    ox810sata_private_data* pd = (ox810sata_private_data*)ap->private_data;    
    DPRINTK("\n");

    /* set the in cleanup flag */
    pd->in_cleanup = 1;

    /* if we're a PIO command existing cleanup won't be called */
	qc = ata_qc_from_tag(ap, ap->link.active_tag);
	if (qc->tf.protocol == ATA_PROT_PIO) {
        /* reset the core */
        ox810sata_timeout_cleanup(ap);
    }

    /* clear the in cleanup flag */
    pd->in_cleanup = 0;
}

/** 
 * irq_handler is the interrupt handling routine registered with the system,
 * by libata.
 */
static irqreturn_t ox810sata_irq_handler(
	int   irq,
	void *dev_instance)
{
    struct ata_port        *ap = ((struct ata_host *)dev_instance)->ports[0];
    ox810sata_private_data *pd;
    u32                    *ioaddr;
    u32                     int_status;
    u32 count ;    

    DPRINTK("irq = %d\n", irq);

    if (!ap || !ap->private_data)
        BUG();

    /* check the ISR for the port to see if it created the interrupt */
    ioaddr = ox810sata_get_tfio_base(ap);
    int_status = readl(ioaddr + OX810SATA_INT_STATUS);
    if ( !(int_status & OX810SATA_INT_WANT ) ) {
        VPRINTK("not me!\n");
        return IRQ_NONE;
    }

    pd = (ox810sata_private_data*)ap->private_data;
    while (int_status & OX810SATA_INT_MASKABLE) {
        /* store interrupt status for the bottom end */
        pd->int_status |= int_status;

        /* Clear and mask pending interrupts */
        writel(int_status, ioaddr + OX810SATA_INT_CLEAR);
        writel(int_status, ioaddr + OX810SATA_INT_DISABLE);

        int_status = readl(ioaddr + OX810SATA_INT_STATUS);
    }
    
	// Wait a short while for the DMA to finish and if it doesn't start a thread
	// to poll for the finish
    pd->spot_the_end_work.ap = ap;
    for (count = 0; count < 10; ++count) {
        if (!oxnas_dma_raw_isactive(pd->DmaChannel)) {
            ox810sata_spot_the_end(&(pd->spot_the_end_work.worker));
            break;
        }
        udelay(100);
    } 
    
    if (count == 10) {
        /* Start a worker thread looking for the DMA channel to become idle */
        VPRINTK("queueing work \n");
        queue_work(ox810sata_driver.spot_the_end_q, &pd->spot_the_end_work.worker);
    }
    VPRINTK("done\n");
    return IRQ_HANDLED;
}

/**
 * Work for a work queue, this will check for errors then wait for the DMA to
 * complete. On the DMA completing it will call ata_qc_complete
 */
static void ox810sata_spot_the_end(struct work_struct *work)
{
    struct spot_the_end_work_s* stew = 
        container_of(work, struct spot_the_end_work_s, worker);
    struct ata_port* ap = stew->ap;
    ox810sata_private_data* PrivateData = (ox810sata_private_data* )ap->private_data;
    struct ata_queued_cmd* qc = ata_qc_from_tag(ap, ap->link.active_tag);
    unsigned long flags = 0;

    VPRINTK("\n");

    /* If there's no command ending associated with this IRQ, ignore it. */
    if (qc == NULL) {
        DPRINTK(" qc=null\n");
        return;
    }
    
	/* The command may have aborted, this is indicated by the interrupt bit
	* being masked */   
    if (PrivateData->in_cleanup) {
        DPRINTK("cleanup\n");
        return;
    }

    /* get the status before any error cleanup */
    qc->err_mask = ac_err_mask(ata_chk_status(ap));

    /* Look to see if the core is indicating an error condition after a RAID 
     * command */
    if (qc->scsicmd &&
        qc->scsicmd->request &&
        qc->scsicmd->request->bio &&
        qc->scsicmd->request->bio->bi_raid ) {
        unsigned long Port0Irq = readl(((u32)(SATA0_REGS_BASE)) + OX810SATA_INT_STATUS);
        unsigned long Port1Irq = readl(((u32)(SATA1_REGS_BASE)) + OX810SATA_INT_STATUS);

        if (OX810SATA_RAW_ERROR & Port0Irq) {
            printk(KERN_DEBUG"disk 0 error in hw-raid\n");
            ox810sata_accumulated_RAID_faults |= 1;
        }
        if (OX810SATA_RAW_ERROR & Port1Irq) {
            printk(KERN_DEBUG"disk 1 error in hw-raid\n");
            ox810sata_accumulated_RAID_faults |= 2;
        }
    }

    /* if there was an error and the command hasn't finished, then we need to
     * abort the command */
	if ((PrivateData->int_status & 0xfffe) &&
        !(PrivateData->int_status & 0x1) &&
        (qc->tf.protocol == ATA_PROT_DMA)) 
    {
        DPRINTK(" int status 0x%08x, ata%u \n",PrivateData->int_status,ap->id);
        local_irq_save(flags);
        ox810sata_irq_clear(ap);
        PrivateData->int_status = 0;
        local_irq_restore(flags);
        ata_qc_complete(qc);
        return;
    }
     
	if (!in_irq()) {
		/* wait for the DMA to finish */
		while (oxnas_dma_is_active(PrivateData->DmaChannel)) {
			schedule();
		}
	}

    if (!(qc->flags & ATA_QCFLAG_ACTIVE)) {
        printk(KERN_WARNING "OX810 SATA: Command already completed!\n");
        return;
    }

    /** @debug check for any padding */
    if(0)
    {
        unsigned int reg;
        reg = readl(OX810SATA_EXCESS);
        if (reg) {
            printk("command finished with a hint of padding\n");
            CrazyDumpDebug();
        }
    }

    /* tell libata we're done */
    DPRINTK(" returning err_mask=0x%x\n", qc->err_mask);
    local_irq_save(flags);
    ox810sata_irq_clear(ap);
    PrivateData->int_status = 0;
    local_irq_restore(flags);
    ata_qc_complete(qc);
}
 
/** 
 * ox810sata_irq_clear is called during probe just before the interrupt handler is
 * registered, to be sure hardware is quiet. It clears and masks interrupt bits
 * in the SATA core.
 *
 * @param ap hardware with the registers in
 */
static void ox810sata_irq_clear(struct ata_port* ap)
{
    u32 *ioaddr = ox810sata_get_tfio_base(ap);
    //DPRINTK(KERN_INFO"ox810sata_irq_clear\n");

    writel(~0, ioaddr + OX810SATA_INT_DISABLE);
    writel(~0, ioaddr + OX810SATA_INT_CLEAR);
}

static inline u32 __ox810sata_scr_read(u32* core_addr, unsigned int sc_reg) 
{
    u32 result;
    u32 patience;

    /* we've got 8 other registers in before the start of the standard ones */    
    writel(sc_reg, core_addr + OX810SATA_LINK_RD_ADDR );

    for (patience = 0x100000; patience > 0; --patience) {
        if (readl(core_addr + OX810SATA_LINK_CONTROL) & 0x00000001) {
            break;
		}
    }

    result = readl(core_addr + OX810SATA_LINK_DATA);

#ifdef CONFIG_ARCH_OX820    
    /* debug: report a speed limit of 1.5Gb */
    if ( sc_reg == 0x28 ) {
        VPRINTK("Reporting a 1.5Gb speed limit\n");
        result |= 0x00000010 ;
    }
#endif    
    //DPRINTK(KERN_INFO"ox810sata_scr_read: [0x%02x]->0x%08x\n", sc_reg, result);
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
static int ox810sata_scr_read(struct ata_port *ap, unsigned int sc_reg, u32 *val)
{
    u32* ioaddr = ox810sata_get_io_base(ap);
	*val = __ox810sata_scr_read(ioaddr, 0x20 + (sc_reg*4));
    return 0;
}

static void __ox810sata_scr_write(u32* core_addr, unsigned int sc_reg, u32 val)
{
    u32 patience;

    //DPRINTK(KERN_INFO"ox810sata_scr_write: [0x%02x]<-0x%08x\n", sc_reg, val);
    writel(val, core_addr + OX810SATA_LINK_DATA );
    wmb();
    writel(sc_reg , core_addr + OX810SATA_LINK_WR_ADDR );
    wmb();

    for (patience = 0x100000; patience > 0;--patience) {
        if (readl(core_addr + OX810SATA_LINK_CONTROL) & 0x00000001) {
            break;
		}
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
static int ox810sata_scr_write(struct ata_port *ap, unsigned int sc_reg, u32 val)
{
    u32 *ioaddr = ox810sata_get_io_base(ap);
    __ox810sata_scr_write(ioaddr, 0x20 + (sc_reg * 4), val);
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
static int  ox810sata_port_start(struct ata_port *ap)
{
    ox810sata_private_data* pd;
    struct device* pdev = ap->host->dev;

    ap->prd = dma_alloc_coherent(pdev, ATA_PRD_TBL_SZ, &ap->prd_dma, GFP_DMA);
    if (!ap->prd) {
        return -ENOMEM;
    }

    /* allocate port private data memory and attach to port */    
    if (!ap->private_data) {
        ap->private_data = kmalloc(sizeof(ox810sata_private_data), GFP_KERNEL);
    }

    if (!ap->private_data) {
        return -ENOMEM;
    }

	pd = (ox810sata_private_data* )ap->private_data;
	pd->DmaChannel = 0;
	pd->sg_entries = 0;

    DPRINTK("ap = %p, pd = %p\n",ap,ap->private_data);

	// Allocate DMA SG entries
	if (oxnas_dma_alloc_sg_entries(&pd->sg_entries, CONFIG_ARCH_OXNAS_MAX_SATA_SG_ENTRIES, 0)) {
		printk(KERN_WARNING "ox810sata_port_start() Failed to obtain DMA SG entries\n");
		return -ENOMEM;
	}

	// Hold on to a DMA channel for the life of the SATA driver
    pd->DmaChannel = oxnas_dma_request(1);
    
	if (!pd->DmaChannel) {
		printk(KERN_WARNING "ox810sata_port_start() Failed to obtain DMA channel\n");
        return -ENOMEM;
	}

    /* declare a work item to spot when a command finishes */
    INIT_WORK(&(pd->spot_the_end_work.worker), &ox810sata_spot_the_end);

    /* initialise to zero */
    pd->ErrorsWithNoCommamnd = 0;
    pd->int_status = 0;
    pd->in_cleanup = 0;

    /* store the ata_port pointer in the driver structure */
    if (ox810sata_get_io_base(ap) == (u32*)SATA0_REGS_BASE) {
        ox810sata_driver.ap[0] = ap;
    } else if (ox810sata_get_io_base(ap) == (u32*)SATA1_REGS_BASE) {
        ox810sata_driver.ap[1] = ap;
	}

    // turn ata core on
    writel((1 << SYS_CTRL_CKEN_SATA_BIT), SYS_CTRL_CKEN_SET_CTRL);

    /* post reset init needs to be called for both ports as there's one reset
    for both ports*/
    if (ox810sata_driver.ap[0]) {
        ox810sata_post_reset_init(ox810sata_driver.ap[0]);
	}
    if (ox810sata_driver.ap[1]) {
        ox810sata_post_reset_init(ox810sata_driver.ap[1]);
	}

    return 0;
}

static void ox810sata_post_reset_init(struct ata_port* ap) 
{
    u32  patience;
    u32* ioaddr = ox810sata_get_io_base(ap);
    uint dev;

    /* turn on phy error detection by removing the masks */ 
    writel(0x30003, ioaddr + OX810SATA_LINK_DATA );
    wmb();
    writel(0x0C, ioaddr + OX810SATA_LINK_WR_ADDR );
    wmb();
    for (patience = 0x100000; patience > 0;--patience) {
        if (readl(ioaddr + OX810SATA_LINK_CONTROL) & 0x00000001) {
            break;
		}
    }

    /* enable interrupts for ports  */
    VPRINTK("Enable interrupts\n");
    writel(~0, OX810SATA_CORE_IEC);  
    writel(OX810SATA_NORMAL_INTS_WANTED, OX810SATA_CORE_IES);  

    /* go through all the devices and configure them */
    for (dev = 0; dev < ATA_MAX_DEVICES; ++dev) {
        if (ap->link.device[dev].class == ATA_DEV_ATA) {
			ox810sata_phy_reset(ap);
            ox810sata_dev_config(ap, &(ap->link.device[dev]));
		}
    }

    /* disable padding */
/*    {
        unsigned int reg = readl(OX810SATA_DEVICE_CONTROL);
        reg &= ~OX810SATA_DEVICE_CONTROL_PAD  ;
        writel(reg, OX810SATA_DEVICE_CONTROL);
    }*/
    {
        unsigned int reg = readl(OX810SATA_DEVICE_CONTROL);
        reg |= OX810SATA_DEVICE_CONTROL_PADPAT  ;
        writel(reg, OX810SATA_DEVICE_CONTROL);
    }
}

/** 
 * port_stop() is called after ->host_stop(). It's sole function is to 
 * release DMA/memory resources, now that they are no longer actively being
 * used.
 */
static void ox810sata_port_stop(struct ata_port *ap)
{
    ox810sata_private_data* pd = (ox810sata_private_data* )ap->private_data;

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
static void ox810sata_host_stop(struct ata_host *host_set)
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
static unsigned int ox810sata_devchk(struct ata_port *ap,unsigned int device)
{
    DPRINTK("\n");

    return 0;       /* nothing found */
}

static void ox810sata_pio_start(struct work_struct *work)
{
	struct ata_port *ap = container_of(work, struct ata_port, port_task.work);
	ox810sata_private_data* pd = (ox810sata_private_data*)ap->private_data;
    struct ata_queued_cmd* qc = ap->port_task_data;
    u32* ioaddr = ox810sata_get_io_base(ap);
    unsigned long flags = 0;

    VPRINTK("\n");
    // We check for DMA completion from ISR which cannot wait for all DMA channel
	// housekeeping to complete, so need to wait here is case we try to reuse
	// channel before that housekeeping has completed
	if (oxnas_dma_is_active(pd->DmaChannel)) {
		printk(KERN_WARNING "PIO start Channel still active\n");
        /* if the DMA is still busy, schedule a task to poll again in 1 ms */
        ata_port_queue_task(ap, ox810sata_pio_start, qc, ATA_SHORT_PAUSE);
        return;
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
								 OXNAS_DMA_MODE_INC,
								 0);

		oxnas_dma_start(pd->DmaChannel);

		if (oxnas_dma_is_active(pd->DmaChannel)) {
			/* if the DMA is still busy, schedule a task to poll again in 1 ms */
			ata_port_queue_task(ap, ox810sata_pio_task, qc, ATA_SHORT_PAUSE);
			return;
		}

		/* cleanup DMA */
		dma_unmap_sg(NULL, qc->__sg, qc->n_elem, qc->dma_dir);
	} else {
        /* if the core is still busy, reschedule */
        if (readl(ioaddr + OX810SATA_SATA_COMMAND) & CMD_CORE_BUSY) {
            ata_port_queue_task(ap, ox810sata_pio_task, qc, ATA_SHORT_PAUSE);
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
static void ox810sata_pio_task(struct work_struct *work)
{
	struct ata_port *ap = container_of(work, struct ata_port, port_task.work);
	struct ata_queued_cmd *qc = ap->port_task_data;
    u32* ioaddr = ox810sata_get_io_base(ap);
    unsigned long flags = 0;
    VPRINTK("\n");

	if (qc->tf.protocol != ATA_PROT_NODATA) {
		ox810sata_private_data* pd = (ox810sata_private_data* )ap->private_data;

        /* if the DMA is still busy and there is no error, re-schedule the task */
        /* try again in 1 ms */
		if ((oxnas_dma_is_active(pd->DmaChannel)) && 
            !(readl(ioaddr + OX810SATA_INT_STATUS) & OX810SATA_RAW_ERROR) ) {
			ata_port_queue_task(ap, ox810sata_pio_task, qc, ATA_SHORT_PAUSE);
			return;
		}

		/* cleanup DMA */
		dma_unmap_sg(NULL, qc->__sg, qc->n_elem, qc->dma_dir);
	} else {
        /* if the core is still busy, reschedule */
        if ((readl(ioaddr + OX810SATA_SATA_COMMAND) & CMD_CORE_BUSY) &&
            !(readl(ioaddr + OX810SATA_INT_STATUS) & OX810SATA_RAW_ERROR) ) {
            ata_port_queue_task(ap, ox810sata_pio_task, qc, ATA_SHORT_PAUSE);
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

static void ox810sata_bmdma_stop(struct ata_queued_cmd *qc)
{
    struct ata_port *ap = qc->ap;
    ox810sata_private_data* private_data = (ox810sata_private_data*)ap->private_data;

    /* Check if DMA is in progress, if so abort */
	if (oxnas_dma_is_active(private_data->DmaChannel)) {
		/* 
		 * Attempt to abort any current transfer:
		 *   Abort DMA transfer at the DMA controller,
		 */
		printk(KERN_ERR "ox810sata_bmdma_stop - aborting DMA\n");

		oxnas_dma_abort(private_data->DmaChannel, 1);
    }
    
    /* perform core cleanups and resets as required */
    ox810sata_timeout_cleanup(ap);
}

/**
 * @param ap ata port, not used
 */
static void ox810sata_timeout_cleanup(struct ata_port *ap) {
    u32 reg;
    u32 patience;

//    CrazyDumpDebug();

    /* Clear error bits in both ports */
    reg = readl((u32*)SATA0_REGS_BASE + OX810SATA_SATA_CONTROL);
    reg |= OX810SATA_SCTL_CLR_ERR ;
    writel(reg, (u32*)SATA0_REGS_BASE + OX810SATA_SATA_CONTROL);
    reg = readl((u32*)SATA1_REGS_BASE + OX810SATA_SATA_CONTROL);
    reg |= OX810SATA_SCTL_CLR_ERR ;
    writel(reg, (u32*)SATA1_REGS_BASE + OX810SATA_SATA_CONTROL);
    reg = readl((u32* )SATARAID_REGS_BASE + OX810SATA_SATA_CONTROL);
    reg |= OX810SATA_SCTL_CLR_ERR ;
    writel(reg, (u32* )SATARAID_REGS_BASE + OX810SATA_SATA_CONTROL);

    /* Test SATA core idle state */
    if ( !(~readl(OX810SATA_IDLE_STATUS) & OX810SATA_IDLE_CORES) ) {
        return;
	}

//    CrazyDumpDebug();

    /* abort DMA */
    printk(KERN_INFO"ox810sata aborting DMA.\n");
    reg = readl(OX810SATA_DEVICE_CONTROL);
    writel(reg | OX810SATA_DEVICE_CONTROL_ABORT, OX810SATA_DEVICE_CONTROL);

    /* wait until patience runs out for the core to go idle */
    patience = 50;
    do {
        /* if the core is idle, clear the abort bit and return */
        if ( !(~readl(OX810SATA_IDLE_STATUS) & OX810SATA_IDLE_CORES) ) {
            writel(reg & ~OX810SATA_DEVICE_CONTROL_ABORT, OX810SATA_DEVICE_CONTROL);
            return;
        }
        mdelay(1);
    } while (--patience);
    writel(reg & ~OX810SATA_DEVICE_CONTROL_ABORT, OX810SATA_DEVICE_CONTROL);

//    CrazyDumpDebug();

    /* command a sync escape on both ports */
    printk(KERN_INFO"ox810sata sending sync escapes\n");

    /* port 0 */
    reg = readl((u32*)SATA0_REGS_BASE + OX810SATA_SATA_COMMAND);
    reg &= ~SATA_OPCODE_MASK;
    reg |= CMD_SYNC_ESCAPE;
    writel(reg, (u32*)SATA0_REGS_BASE + OX810SATA_SATA_COMMAND);

    /* wait until patience runs out for the core to go idle */
    patience = 50;
    do {
        /* if the core is idle, clear the abort bit and return */
        if ( !(~readl(OX810SATA_IDLE_STATUS) & OX810SATA_IDLE_CORES) ) {
            return;
        }
        mdelay(1);
    } while (--patience);

    /* port 1 */
    reg = readl((u32*)SATA1_REGS_BASE + OX810SATA_SATA_COMMAND);
    reg &= ~SATA_OPCODE_MASK;
    reg |= CMD_SYNC_ESCAPE;
    writel(reg, (u32*)SATA1_REGS_BASE + OX810SATA_SATA_COMMAND);

    /* wait until patience runs out for the core to go idle */
    patience = 50;
    do {
        /* if the core is idle, clear the abort bit and return */
        if ( !(~readl(OX810SATA_IDLE_STATUS) & OX810SATA_IDLE_CORES) ) {
            return;
        }
        mdelay(1);
    } while (--patience);

//    CrazyDumpDebug();

    /* SATA core did not go idle, so cause a SATA core reset from the RPS */
        CrazyDumpDebug();
    printk(KERN_INFO "ox810sata core reset\n");
    ox810sata_reset_core();

    /* Read SATA core idle state */
    if (~readl(OX810SATA_IDLE_STATUS) & OX810SATA_IDLE_CORES) {
        printk(KERN_INFO"ox810sata core still busy\n");
        CrazyDumpDebug();
    }

    /* Perform any SATA core re-initialisation after reset */
    /* post reset init needs to be called for both ports as there's one reset
    for both ports*/
    if (ox810sata_driver.ap[0]) {
        ox810sata_post_reset_init(ox810sata_driver.ap[0]);
	}
    if (ox810sata_driver.ap[1]) {
        ox810sata_post_reset_init(ox810sata_driver.ap[1]);
	}
}


static void ox810sata_error_handler(struct ata_port *ap)
{
	return ata_bmdma_drive_eh(ap, ox810sata_prereset, ata_std_softreset,
				  ox810sata_hardreset, ox810sata_postreset);
}



/** 
 * bmdma_status return a made up version of a BMDMA status register
 *
 * @param ap Hardware with the registers in
 * @return the value ATA_DMA_INTR if the interrupt came from the DMA finishing
 */
static u8   ox810sata_bmdma_status(struct ata_port *ap)
{
	return ATA_DMA_INTR;
}
 
/** 
 * turn on the interrupts from the ata drive
 * wait for idle, clear any pending interrupts.
 *
 * @param ap Hardware with the registers in
 */
static u8 ox810sata_irq_on(struct ata_port *ap)
{
    u32* ioaddr = ox810sata_get_tfio_base(ap);
    u8 tmp;

    //DPRINTK(KERN_INFO"ox810sata_irq_on\n");

    /* enable End of command interrupt */
    writel(~0, ioaddr + OX810SATA_INT_CLEAR);
    writel(OX810SATA_INT_WANT, ioaddr + OX810SATA_INT_ENABLE);
	tmp = ata_wait_idle(ap);

    return tmp;
}

/**
 *	ox810_prereset - prepare for reset
 *	@param link  ATA link to be reset
 *	@param deadline  deadline jiffies for the operation
 *
 *	link is about to be reset.  Initialize it.  Failure from
 *	prereset makes libata abort whole reset sequence and give up
 *	that port, so prereset should be best-effort.  It does its
 *	best to prepare for reset sequence but if things go wrong, it
 *	should just whine, not fail.
 *
 *	LOCKING:
 *	Kernel thread context (may sleep)
 *
 *	RETURNS:
 *	0 on success, -errno otherwise.
 */
static int ox810sata_prereset(struct ata_link *link, unsigned long deadline) {
	struct ata_port *ap = link->ap;
	struct ata_eh_context *ehc = &link->eh_context;
	const unsigned long *timing = sata_ehc_deb_timing(ehc);
	ox810sata_private_data* private_data;
    int rc;

    VPRINTK("\n");
	/* handle link resume */
	if ((ehc->i.flags & ATA_EHI_RESUME_LINK) && (link->flags & ATA_LFLAG_HRST_TO_RESUME))
		ehc->i.action |= ATA_EH_HARDRESET;

	/* Some PMPs don't work with only SRST, force hardreset if PMP
	 * is supported.
	 */
	if (ap->flags & ATA_FLAG_PMP)
		ehc->i.action |= ATA_EH_HARDRESET;

	/* if we're about to do hardreset, nothing more to do */
	if (ehc->i.action & ATA_EH_HARDRESET)
		return 0;

    /* we want both ports to be idle as soft-reset requires being able to send
    commands. If a command is running, abort it. */
    private_data = (ox810sata_private_data*)ap->private_data;

    /* Check if DMA is in progress, if so abort */
    if (oxnas_dma_is_active(private_data->DmaChannel)) {
        /* 
         * Attempt to abort any current transfer:
         *   Abort DMA transfer at the DMA controller,
         */
        printk(KERN_ERR "aborting DMA\n");
        oxnas_dma_abort(private_data->DmaChannel, 1);
    }
    
    /* perform core cleanups and resets */
    ox810sata_timeout_cleanup(NULL);
   
	/* if SATA, resume link */
	if (ap->flags & ATA_FLAG_SATA) {
		rc = sata_link_resume(link, timing, deadline);
		/* whine about phy resume failure but proceed */
		if (rc && rc != -EOPNOTSUPP)
			ata_link_printk(link, KERN_WARNING, "failed to resume "
					"link for reset (errno=%d)\n", rc);
	}
    
	/* Wait for !BSY if the controller can wait for the first D2H
	 * Reg FIS and we don't know that no device is attached.
	 */
	if (!(link->flags & ATA_LFLAG_SKIP_D2H_BSY) && !ata_link_offline(link)) {
		rc = ata_wait_ready(ap, deadline);
		if (rc && rc != -ENODEV) {
			ata_link_printk(link, KERN_WARNING, "device not ready "
					"(errno=%d), forcing hardreset\n", rc);
			ehc->i.action |= ATA_EH_HARDRESET;
		}
	}

	return 0;
}
/**
 *	sata_std_hardreset - reset host port via SATA phy reset
 *	@link: link to reset
 *	@class: resulting class of attached device
 *	@deadline: deadline jiffies for the operation
 *
 *	SATA phy-reset host port using DET bits of SControl register,
 *	wait for !BSY and classify the attached device.
 *
 *	LOCKING:
 *	Kernel thread context (may sleep)
 *
 *	RETURNS:
 *	0 on success, -errno otherwise.
 */
static int ox810sata_hardreset(struct ata_link *link, unsigned int *class,
		       unsigned long deadline)
{
	struct ata_port *ap = link->ap;
	const unsigned long *timing = sata_ehc_deb_timing(&link->eh_context);
	int rc;

	DPRINTK("ENTER\n");
    
	/* do hardreset */
	rc = sata_link_hardreset(link, timing, deadline);
	if (rc) {
		ata_link_printk(link, KERN_ERR,
				"COMRESET failed (errno=%d)\n", rc);
		return rc;
	}

	/* TODO: phy layer with polling, timeouts, etc. */
	if (ata_link_offline(link)) {
		*class = ATA_DEV_NONE;
		DPRINTK("EXIT, link offline\n");
		return 0;
	}

	/* wait a while before checking status */
	ata_wait_after_reset(ap, deadline);

	/* If PMP is supported, we have to do follow-up SRST.  Note
	 * that some PMPs don't send D2H Reg FIS after hardreset at
	 * all if the first port is empty.  Wait for it just for a
	 * second and request follow-up SRST.
	 */
	if (ap->flags & ATA_FLAG_PMP) {
		ata_wait_ready(ap, jiffies + HZ);
		return -EAGAIN;
	}

	rc = ata_wait_ready(ap, deadline);
	/* link occupied, -ENODEV too is an error */
	if (rc) {
		ata_link_printk(link, KERN_ERR,
				"COMRESET failed (errno=%d)\n", rc);
		return rc;
	}

	*class = ata_dev_try_classify(link->device, 1, NULL);

	DPRINTK("EXIT, class=%u\n", *class);
	return 0;
}

/**
 *	ata_std_postreset - standard postreset callback
 *	@link: the target ata_link
 *	@classes: classes of attached devices
 *
 *	This function is invoked after a successful reset.  Note that
 *	the device might have been reset more than once using
 *	different reset methods before postreset is invoked.
 *
 *	LOCKING:
 *	Kernel thread context (may sleep)
 */
static void ox810sata_postreset(struct ata_link *link, unsigned int *classes)
{
	struct ata_port *ap = link->ap;
	u32 serror;
    unsigned int dev;

	DPRINTK("ENTER\n");

    /* print link status */
	sata_print_link_status(link);

	/* clear SError */
	if (sata_scr_read(link, SCR_ERROR, &serror) == 0)
		sata_scr_write(link, SCR_ERROR, serror);
	link->eh_info.serror = 0;
    
    /* turn on phy error detection by removing the masks */ 
    __ox810sata_scr_write((u32* )SATA0_REGS_BASE , 0x0c, 0x30003 );
    __ox810sata_scr_write((u32* )SATA1_REGS_BASE , 0x0c, 0x30003 );

	/* bail out if no device is present */
	if (classes[0] == ATA_DEV_NONE && classes[1] == ATA_DEV_NONE) {
		DPRINTK("EXIT, no device\n");
		return;
	}

    /* go through all the devices and configure them */
    for (dev = 0; dev < ATA_MAX_DEVICES; ++dev) {
        if (ap->link.device[dev].class == ATA_DEV_ATA) {
            ox810sata_dev_config(ap, &(ap->link.device[dev]));
		}
    }

    /* disable padding */
    {
        unsigned int reg = readl(OX810SATA_DEVICE_CONTROL);
        reg &= ~OX810SATA_DEVICE_CONTROL_PAD  ;
        writel(reg, OX810SATA_DEVICE_CONTROL);
    }

    /** @todo fix by using tf/tf_load as in ata_bus_softreset */
    #if 0
	/* set up device control */
	if (ap->ioaddr.ctl_addr)
		iowrite8(ap->ctl, ap->ioaddr.ctl_addr);
    #endif

	DPRINTK("EXIT\n");
}

/** 
 * Outputs all the registers in the SATA core for diagnosis of faults.
 *
 * @param ap Hardware with the registers in
 */
static void CrazyDumpDebug()
{
#ifdef CRAZY_DUMP_DEBUG
    u32 offset;
    u32 result;
    u32 patience;
    volatile u32* ioaddr;

#if 0
    {
        u32 i ;
        for(i = 0;i < 1024;++i) {
            printk("[%08x]%s%08x\n",
                regarray[regindex].a,
                regarray[regindex].w ? "<=" : "=>",
                regarray[regindex].d
                );
            ++regindex;
            regindex &= 1023;
        }
    }
#endif

    /* port 0 */
    ioaddr = (u32* )SATA0_REGS_BASE;
    printk("Port 0 High level registers\n");
    for(offset = 0; offset < 48;offset++)
    {
        printk("[%02x] %08x\n", offset * 4, *(ioaddr + offset));
    }

    printk("Port 0 link layer registers\n");
    for(offset = 0; offset < 16;++offset)
    {
        *(ioaddr + OX810SATA_LINK_RD_ADDR ) = (offset*4);
        wmb();
    
        for (patience = 0x100000;patience > 0;--patience)
        {
            if (*(ioaddr + OX810SATA_LINK_CONTROL) & 0x00000001)
                break;
        }
    
        result = *(ioaddr + OX810SATA_LINK_DATA);
        printk("[%02x] %08x\n", offset*4, result);
    }

    /* port 1 */
    ioaddr = (u32* )SATA1_REGS_BASE;
    printk("Port 1 High level registers\n");
    for(offset = 0; offset < 48;offset++)
    {
        printk("[%02x] %08x\n", offset * 4, *(ioaddr + offset));
    }

    printk("Port 1 link layer registers\n");
    for(offset = 0; offset < 16;++offset)
    {
        *(ioaddr + OX810SATA_LINK_RD_ADDR ) = (offset*4);
        wmb();
    
        for (patience = 0x100000;patience > 0;--patience)
        {
            if (*(ioaddr + OX810SATA_LINK_CONTROL) & 0x00000001)
                break;
        }
    
        result = *(ioaddr + OX810SATA_LINK_DATA);
        printk("[%02x] %08x\n", offset*4, result);
    }
    
    /* port 14 */
    ioaddr = (u32* )SATARAID_REGS_BASE;
    printk("RAID registers\n");
    for(offset = 0; offset < 48;offset++)
    {
        printk("[%02x] %08x\n", offset * 4, *(ioaddr + offset));
    }

    /* port 15 */
    ioaddr = (u32* )SATACORE_REGS_BASE;
    printk("CORE registers\n");
    for(offset = 0; offset < 48;offset++)
    {
        printk("[%02x] %08x\n", offset * 4, *(ioaddr + offset));
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
static struct resource ox810sata_port0_resources[] = {
	{
        .name       = "sata_port_0_registers",
		.start		= SATA0_REGS_BASE,
		.end		= SATA0_REGS_BASE + 0xff,
		.flags		= IORESOURCE_MEM,
	},
    {
        .name       = "sata_irq",
        .start      = SATA_1_INTERRUPT,
		.flags		= IORESOURCE_IRQ,
    }
};

static struct resource ox810sata_port1_resources[] = {
	{
        .name       = "sata_port_1_registers",
		.start		= SATA1_REGS_BASE,
		.end		= SATA1_REGS_BASE + 0xff,
		.flags		= IORESOURCE_MEM,
	},
    {
        .name       = "sata_irq",
        .start      = SATA_1_INTERRUPT,
		.flags		= IORESOURCE_IRQ,
    },
};

static struct platform_device ox810sata_dev0 = 
{
    .name = DRIVER_NAME,
    .id = 0,
    .num_resources = 2,
	.resource  = ox810sata_port0_resources,
    .dev.coherent_dma_mask = 0xffffffff,
}; 

static struct platform_device ox810sata_dev1 = 
{
    .name = DRIVER_NAME,
    .id = 1,
    .num_resources = 2,
	.resource  = ox810sata_port1_resources,
    .dev.coherent_dma_mask = 0xffffffff,
}; 

/** 
 * module initialisation
 * @return success is 0
 */
static int __init ox810sata_device_init( void )
{
    int ret;

    /* reset the core */
    ox810sata_reset_core();

    {
        // register the ata device for the driver to find
        ret = platform_device_register( &ox810sata_dev0 );
        DPRINTK(" %i\n", ret);
    }
    
#ifndef CONFIG_OX810SATA_SINGLE_SATA
    {
        // register the ata device for the driver to find
        ret = platform_device_register( &ox810sata_dev1 );
        DPRINTK(" %i\n", ret);
    }
#endif /* CONFIG_OX810_SINGLE_SATA */

    return ret;
}

/** 
 * module cleanup
 */
static void __exit ox810sata_device_exit(void)
{
    platform_device_unregister( &ox810sata_dev0 );
    platform_device_unregister( &ox810sata_dev1 );
}

/**
 * Returns accumulated RAID faults and then clears the accumulation
 * @return accumulated RAID faults indicated by set bits
 */
int  oxnassata_RAID_faults( void ) {
    int temp = ox810sata_accumulated_RAID_faults;
    ox810sata_accumulated_RAID_faults = 0;
    return temp;
}

/**
 * Returns ox810 port number the request queue is serviced by.
 *
 * @param queue The queue under investigation.
 * @return The ox810 sata port number servicing the queue or -1 if not found.
 */
int oxnassata_get_port_no(struct request_queue* q)
{
    struct ata_port* ap = 0;
    struct scsi_device* sdev = 0;
    
    /* check port 0 */
    ap = ox810sata_driver.ap[0];
    if (ap)
        shost_for_each_device(sdev, ap->scsi_host) {
            if (sdev->request_queue == q) {
                DPRINTK("Queue %p on port 0\n", q);
                return 0;
            }
        }
    
    /* check port 1 */
    ap = ox810sata_driver.ap[1];
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
    ap = ox810sata_driver.ap[0];
    if (ap)
        flags0 = ap->link.device[0].flags & ATA_DFLAG_LBA48 ;
    else
        return 0;
    
    /* check port 1 */
    ap = ox810sata_driver.ap[1];
    if (ap)
        flags1 = ap->link.device[0].flags & ATA_DFLAG_LBA48 ;
    else
        return 0;

    /* compare */
    return (flags0 == flags1);  
}

EXPORT_SYMBOL( oxnassata_RAID_faults );
EXPORT_SYMBOL( oxnassata_get_port_no );
EXPORT_SYMBOL( oxnassata_LBA_schemes_compatible );


#ifdef ERROR_INJECTION

/**
 * @param kobj Not Used
 * @param attr Used to determine which file is being accessed
 * @param buffer Space to put the file contents
 * @return The number of bytes transferred or an error
 */
static int ox810sata_error_inject_show(
    char *page, char **start, off_t off, int count, int *eof, void *data)
{
    if (page)
    {
        if ( ox810sata_driver.error_inject ) {
            page[0] = ox810sata_driver.error_inject + '0';
            page[1] = '\n';
            page[2] = 0;
            return 3;
        } else {
            strcpy(page, "off\n" );
            return 5;
        }
    }
    
    /* if we get here, there's been an error */
    return -EIO;
}


static int ox810sata_error_inject_store(struct file *file,
                                        const char __user *buffer,
                                        unsigned long count,
                                        void *data) {
    if (count)
    {
        if ((buffer[0] >= '0') &&
            (buffer[0] <= '9')) {
            ox810sata_driver.error_inject = buffer[0] - '0';
        }
        return count;
    }
    
    /* if we get here, there's been an error */
    return -EIO;
}

#endif /* ERROR_INJECTION */



/** 
 * macros to register intiialisation and exit functions with kernal
 */
module_init(ox810sata_device_init);
module_exit(ox810sata_device_exit);
