/* 
 *  procfs3.c -  create a "file" in /proc, use the file_operation way
 *  		to manage the file.
 */
 
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>	/* We're doing kernel work */
#include <linux/module.h>	/* Specifically, a module */
#include <linux/proc_fs.h>	/* Necessary because we use proc fs */
#include <asm/uaccess.h>		/* for copy_*_user */
#include "asm/arch-oxnas/i2s.h"
#include "asm/io.h"

#define DRV_NAME		"i2s"
#define DRV_VERSION	"0.1"
#define PROC_ENTRY_FILENAME 	"i2s"
#define PROCFS_MAX_SIZE 	2048



MODULE_AUTHOR("Chris Ford");
MODULE_DESCRIPTION("I2S Test module");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

/**
 * The buffer (2k) for this module
 *
 */
static char procfs_buffer[PROCFS_MAX_SIZE];

/**
 * The size of the data hold in the buffer
 *
 */
static unsigned long procfs_buffer_size = 0;

/**
 * The structure keeping information about the /proc file
 *
 */
static struct proc_dir_entry *Our_Proc_File;


void RefreshI2SRegisters(void)
{
	int iLen = sprintf( procfs_buffer,  
		"DumpI2SRegisters----------------------------------\n"
		"                                                  \n"
		"     TX_CONTROL                = 0x%08x\n"
		"     TX_SETUP                  = 0x%08x\n"
		"     TX_SETUP1                 = 0x%08x\n"
		"     TX_STATUS                 = 0x%08x\n"
		"     RX_CONTROL                = 0x%08x\n"
		"     RX_SETUP                  = 0x%08x\n"
		"     RX_SETUP1                 = 0x%08x\n"
		"     RX_STATUS                 = 0x%08x\n"
		"     TX_DEBUG                  = 0x%08x\n"
		"     TX_DEBUG2                 = 0x%08x\n"
		"     TX_DEBUG3                 = 0x%08x\n"
		"     RX_DEBUG_                 = 0x%08x\n"
		"     RX_DEBUG2                 = 0x%08x\n"
		"     RX_DEBUG3                 = 0x%08x\n"
		"     TX_BUFFER_LEVEL           = 0x%08x\n"
		"     TX_BUFFER_INTERRUPT_LEVEL = 0x%08x\n"
		"     RX_BUFFER_LEVEL           = 0x%08x\n"
		"     RX_BUFFER_INTERRUPT_LEVEL = 0x%08x\n"
		"     RX_SPDIF_DEBUG            = 0x%08x\n"
		"     RX_SPDIF_DEBUG2           = 0x%08x\n",
		(u32) __raw_readl( TX_CONTROL                 ),
		(u32) __raw_readl( TX_SETUP                   ),
		(u32) __raw_readl( TX_SETUP1                  ),
		(u32) __raw_readl( TX_STATUS                  ),
		(u32) __raw_readl( RX_CONTROL                 ),
		(u32) __raw_readl( RX_SETUP                   ),
		(u32) __raw_readl( RX_SETUP1                  ),
		(u32) __raw_readl( RX_STATUS                  ),
		(u32) __raw_readl( TX_DEBUG                   ),
		(u32) __raw_readl( TX_DEBUG2                  ),
		(u32) __raw_readl( TX_DEBUG3                  ),
		(u32) __raw_readl( RX_DEBUG_                  ),
		(u32) __raw_readl( RX_DEBUG2                  ),
		(u32) __raw_readl( RX_DEBUG3                  ),
		(u32) __raw_readl( TX_BUFFER_LEVEL            ),
		(u32) __raw_readl( TX_BUFFER_INTERRUPT_LEVEL  ),
		(u32) __raw_readl( RX_BUFFER_LEVEL            ),
		(u32) __raw_readl( RX_BUFFER_INTERRUPT_LEVEL  ),
		(u32) __raw_readl( RX_SPDIF_DEBUG             ),
		(u32) __raw_readl( RX_SPDIF_DEBUG2            ) );
	
	procfs_buffer_size = iLen + sprintf( procfs_buffer + iLen, 
		"     INTERRUPT_CONTROL_STATUS  = 0x%08x\n"
		"     INTERRUPT_MASK            = 0x%08x\n"
		"     VERSION                   = 0x%08x\n"
		"     TX_DATA_IN_FORMAT         = 0x%08x\n"
		"     TX_CHANNELS_ENABLE        = 0x%08x\n"
		"     TX_WRITES_TO              = 0x%08x\n"
		"     RX_DATA_OUT_FORMAT        = 0x%08x\n"
		"     RX_CHANNELS_ENABLE        = 0x%08x\n"
		"     RX_READS_FROM             = 0x%08x\n"
		"     TX_CPU_DATA_WRITES_ALT    = 0x%08x\n"
		"     RX_CPU_DATA_READS_ALT     = 0x%08x\n"
		"     TX_CPU_DATA_WRITES        = 0x%08x\n"
		"     RX_CPU_DATA_READS         = 0x%08x\n"
		"\n"
		"--------------------------------------------------\n",
		(u32) __raw_readl( INTERRUPT_CONTROL_STATUS   ),
		(u32) __raw_readl( INTERRUPT_MASK             ),
		(u32) __raw_readl( VERSION                    ),
		(u32) __raw_readl( TX_DATA_IN_FORMAT          ),
		(u32) __raw_readl( TX_CHANNELS_ENABLE         ),
		(u32) __raw_readl( TX_WRITES_TO               ),
		(u32) __raw_readl( RX_DATA_OUT_FORMAT         ),
		(u32) __raw_readl( RX_CHANNELS_ENABLE         ),
		(u32) __raw_readl( RX_READS_FROM              ),
		(u32) __raw_readl( TX_CPU_DATA_WRITES_ALT     ),
		(u32) __raw_readl( RX_CPU_DATA_READS_ALT      ),
		(u32) __raw_readl( TX_CPU_DATA_WRITES         ),
		(u32) __raw_readl( RX_CPU_DATA_READS          ) );
}


void DumpI2SRegisters(void)
{
	RefreshI2SRegisters();
	printk( KERN_INFO "%s", procfs_buffer );  
	return;
}


/**
 * This funtion is called when the /proc file is read
 *
 */
static ssize_t procfs_read(
	struct file *filp,	/* see include/linux/fs.h   */
	char *buffer,		/* buffer to fill with data */
	size_t length,		/* length of the buffer     */
	loff_t * offset)
{
	static int finished = 0;

	/* 
	 * We return 0 to indicate end of file, that we have
	 * no more information. Otherwise, processes will
	 * continue to read from us in an endless loop. 
	 */
	if ( finished ) {
		printk(KERN_INFO "procfs_read: END\n");
		finished = 0;
		return 0;
	}
	
	finished = 1;
		
	/* 
	 * We use put_to_user to copy the string from the kernel's
	 * memory segment to the memory segment of the process
	 * that called us. get_from_user, BTW, is
	 * used for the reverse. 
	 */
	if ( copy_to_user(buffer, procfs_buffer, procfs_buffer_size) ) {
		return -EFAULT;
	}

	printk(KERN_INFO "procfs_read: read %lu bytes\n", procfs_buffer_size);

	return procfs_buffer_size;	/* Return the number of bytes "read" */
}

/*
 * This function is called when /proc is written
 */
static ssize_t
procfs_write(struct file *file, const char *buffer, size_t len, loff_t * off)
{
	if ( len > PROCFS_MAX_SIZE )	{
		procfs_buffer_size = PROCFS_MAX_SIZE;
	}
	else	{
		procfs_buffer_size = len;
	}
	
	if ( copy_from_user(procfs_buffer, buffer, procfs_buffer_size) ) {
		return -EFAULT;
	}

	printk(KERN_INFO "procfs_write: write %s\n",buffer);
	printk(KERN_INFO "procfs_write: write %lu bytes\n", procfs_buffer_size);
	
	return procfs_buffer_size;
}

/* 
 * This function decides whether to allow an operation
 * (return zero) or not allow it (return a non-zero
 * which indicates why it is not allowed).
 *
 * The operation can be one of the following values:
 * 0 - Execute (run the "file" - meaningless in our case)
 * 2 - Write (input to the kernel module)
 * 4 - Read (output from the kernel module)
 *
 * This is the real function that checks file
 * permissions. The permissions returned by ls -l are
 * for referece only, and can be overridden here.
 */

static int module_permission(struct inode *inode, int op, struct nameidata *foo)
{
	/* 
	 * We allow everybody to read from our module, but
	 * only root (uid 0) may write to it 
	 */
	 if (op == 4 || (op == 2 && current->euid == 0)) {
		 printk( KERN_INFO "Insufficient permissions\n");
		return 0;
	 }

	/* 
	 * If it's anything else, access is denied 
	 */
	return -EACCES;
}

/* 
 * The file is opened - we don't really care about
 * that, but it does mean we need to increment the
 * module's reference count. 
 */
int procfs_open(struct inode *inode, struct file *file)
{
	u32 temp = 0;
	
	/* Open an entry on the proc filesystem */
	printk(KERN_INFO "I2S::procfs_open\n");
	try_module_get(THIS_MODULE);
	
	// printk(KERN_INFO "I2S::pre-reg set..\n");
	// RefreshI2SRegisters();

	/* Setup the I2S TX Core... */
	temp  = 1                 << TX_CONTROL_ENABLE        |
		1                 << TX_CONTROL_FLUSH         |
		0                 << TX_CONTROL_MUTE          |
		0                 << TX_CONTROL_TRICK         |
		0                 << TX_CONTROL_SPEED         |
		1                 << TX_CONTROL_ABORT_DMA     |
		0                 << TX_CONTROL_AHB_ENABLE    |
		1                 << TX_CONTROL_QUAD_BURSTS;
	__raw_writel( temp, TX_CONTROL );
	
	temp  = TRUE_I2S          << TX_SETUP_FORMAT          |
		I2S_SLAVE         << TX_SETUP_MODE            |
		0                 << TX_SETUP_FLOW_INVERT     |
		0                 << TX_SETUP_POS_EDGE        |
		0                 << TX_SETUP_CLOCK_STOP      |
		0                 << TX_SETUP_SPLIT_QUAD      |
		0                 << TX_SETUP_SPDIF_EN;
	__raw_writel( temp, TX_SETUP );
	
	temp  = TWOS_COMPLIMENT   << TX_SETUP1_INPUT          |
		0		  << TX_SETUP1_REVERSE        |
		0         	  << TX_SETUP1_INVERT         |
		0                 << TX_SETUP1_BIG_ENDIAN     |
		0                 << TX_SETUP1_QUAD_ENDIAN    |
		0                 << TX_SETUP1_QUAD_SAMPLES   |
		0                 << TX_SETUP1_FLOW_CONTROL;
	__raw_writel( temp, TX_SETUP1 );
	
	/* Setup the I2S RX Core... */
	
	printk(KERN_INFO "\n\nI2S::post-reg set..\n");
	RefreshI2SRegisters();
	return 0;
}

/* 
 * The file is closed - again, interesting only because
 * of the reference count. 
 */
int procfs_close(struct inode *inode, struct file *file)
{
	printk(KERN_INFO "I2S::procfs_close\n");
	module_put(THIS_MODULE);
	return 0;		/* success */
}

static struct file_operations File_Ops_4_Our_Proc_File = {
	.read 	 = procfs_read,
	.write 	 = procfs_write,
	.open 	 = procfs_open,
	.release = procfs_close,
};

/* 
 * Inode operations for our proc file. We need it so
 * we'll have some place to specify the file operations
 * structure we want to use, and the function we use for
 * permissions. It's also possible to specify functions
 * to be called for anything else which could be done to
 * an inode (although we don't bother, we just put
 * NULL). 
 */

static struct inode_operations Inode_Ops_4_Our_Proc_File = {
	.permission = module_permission,	/* check for permissions */
};

/* 
 * Module initialization and cleanup 
 */
static int __init oxnas_i2s_init_module(void)
{
	printk(KERN_INFO "I2S::init_module\n");

	/* create the /proc file */
	Our_Proc_File = create_proc_entry(PROC_ENTRY_FILENAME, 0644, NULL);
	
	/* check if the /proc file was created successfuly */
	if (Our_Proc_File == NULL){
		printk(KERN_ALERT "Error: Could not initialize /proc/%s\n",
		       PROC_ENTRY_FILENAME);
		return -ENOMEM;
	}
	
	Our_Proc_File->owner 	  = THIS_MODULE;
	Our_Proc_File->proc_iops  = &Inode_Ops_4_Our_Proc_File;
	Our_Proc_File->proc_fops  = &File_Ops_4_Our_Proc_File;
	Our_Proc_File->mode 	  = S_IFREG | S_IRUGO | S_IWUSR;
	Our_Proc_File->uid 	  = 0;
	Our_Proc_File->gid 	  = 0;
	Our_Proc_File->size	  = 80;

	printk(KERN_INFO "/proc/%s created\n", PROC_ENTRY_FILENAME);

	return 0;	/* success */
}

static void __exit oxnas_i2s_cleanup_module(void)
{
	printk(KERN_INFO "I2S::cleanup_module\n");
	remove_proc_entry(PROC_ENTRY_FILENAME, &proc_root);
	printk(KERN_INFO "/proc/%s removed\n", PROC_ENTRY_FILENAME);
}

module_init(oxnas_i2s_init_module);
module_exit(oxnas_i2s_cleanup_module);


