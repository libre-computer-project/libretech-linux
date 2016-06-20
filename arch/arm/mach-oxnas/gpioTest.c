#include <asm/io.h>
#include <linux/bitops.h>
#include <asm/uaccess.h>         // copy_to_user and copy_from_user
#include <linux/init.h>          // modules
#include <linux/module.h>        // module
#include <linux/types.h>         // dev_t type
#include <linux/fs.h>            // chrdev allocation
#include <linux/slab.h>          // kmalloc and kfree
#include <linux/cdev.h>          // struct cdev
#include <linux/errno.h>         // error codes
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <asm/arch/hardware.h>

MODULE_LICENSE("GPL v2");

MODULE_AUTHOR("C Ford");
MODULE_DESCRIPTION("gpioTest first version");

struct gpioTest_dev {
  struct semaphore sem;    // Mutual exclusion semaphore
  struct cdev      cdev;        // Char device structure
  int	           found;
};

struct gpioTest_dev * gpioTest_device;  // Contains the gpioTest devices

dev_t dev;                         // Contains major and first minor number
static wait_queue_head_t ir_block;

////////////////////////////
// LINKED LIST OPERATIONS //
////////////////////////////


///////////////
// CALLBACKS //
///////////////

#define GPIO_TEST_SCL (1UL << CONFIG_OXNAS_I2C_SCL)
#define GPIO_TEST_SDA (1UL << CONFIG_OXNAS_I2C_SDA)

void DumpGPIO( void )
{
	printk( KERN_INFO "================= GPIO Dump\n"
	  "     GPIO_A_DATA                            0x%08x\n"
	  "     GPIO_A_OUTPUT_ENABLE                   0x%08x\n"
	  "     GPIO_A_INTERRUPT_ENABLE                0x%08x\n"
	  "     GPIO_A_INTERRUPT_EVENT                 0x%08x\n"
	  "     GPIO_A_OUTPUT_VALUE                    0x%08x\n"
	  "     GPIO_A_OUTPUT_SET                      0x%08x\n"
	  "     GPIO_A_OUTPUT_CLEAR                    0x%08x\n"
	  "     GPIO_A_OUTPUT_ENABLE_SET               0x%08x\n"
	  "     GPIO_A_OUTPUT_ENABLE_CLEAR             0x%08x\n"
	  "     GPIO_A_INPUT_DEBOUNCE_ENABLE           0x%08x\n"
	  "     GPIO_A_RISING_EDGE_ACTIVE_HIGH_ENABLE  0x%08x\n"
	  "     GPIO_A_FALLING_EDGE_ACTIVE_LOW_ENABLE  0x%08x\n"
	  "     GPIO_A_RISING_EDGE_DETECT              0x%08x\n"
	  "     GPIO_A_FALLING_EDGE_DETECT             0x%08x\n"
	  "     GPIO_A_LEVEL_INTERRUPT_ENABLE          0x%08x\n"
	  "     GPIO_A_INTERRUPT_STATUS_REGISTER       0x%08x\n",
        readl( GPIO_A_DATA                           ),
        readl( GPIO_A_OUTPUT_ENABLE                  ),
        readl( GPIO_A_INTERRUPT_ENABLE               ),
        readl( GPIO_A_INTERRUPT_EVENT                ),
        readl( GPIO_A_OUTPUT_VALUE                   ),
        readl( GPIO_A_OUTPUT_SET                     ),
        readl( GPIO_A_OUTPUT_CLEAR                          ),
        readl( GPIO_A_OUTPUT_ENABLE_SET              ),
        readl( GPIO_A_OUTPUT_ENABLE_CLEAR            ),
        readl( GPIO_A_INPUT_DEBOUNCE_ENABLE          ),
        readl( GPIO_A_RISING_EDGE_ACTIVE_HIGH_ENABLE ),
        readl( GPIO_A_FALLING_EDGE_ACTIVE_LOW_ENABLE ),
        readl( GPIO_A_RISING_EDGE_DETECT             ),
        readl( GPIO_A_FALLING_EDGE_DETECT            ),
        readl( GPIO_A_LEVEL_INTERRUPT_ENABLE         ),
        readl( GPIO_A_INTERRUPT_STATUS_REGISTER      ) );	
	
}

irqreturn_t irqHandler( int irq, void* dev_id)
{
	// is this interrupt fors us??
	struct gpioTest_dev* pGPIO = (struct gpioTest_dev*) dev_id;
	unsigned int temp = readl( (volatile unsigned long *) GPIO_A_INTERRUPT_STATUS_REGISTER );
	
	if ( !(temp & (GPIO_TEST_SCL | GPIO_TEST_SDA) ) )
	{
		printk("Not for us...\n");
		return IRQ_NONE;
	}	

	// apparantly it is, for simplicity, we will stop the intterupting pin, 
	// and clear its source, then signal the bottmo half to continue
	if ( test_bit( CONFIG_OXNAS_I2C_SCL, (volatile unsigned long *) &temp ) )
	{
		printk("Int Found CONFIG_OXNAS_I2C_SCL\n");
		*( (volatile unsigned long *) GPIO_A_RISING_EDGE_ACTIVE_HIGH_ENABLE ) &= ~(1UL <<  CONFIG_OXNAS_I2C_SCL);
		*( (volatile unsigned long *) GPIO_A_FALLING_EDGE_ACTIVE_LOW_ENABLE ) &= ~(1UL <<  CONFIG_OXNAS_I2C_SCL);
		*( (volatile unsigned long *) GPIO_A_LEVEL_INTERRUPT_ENABLE ) &= ~(1UL <<  CONFIG_OXNAS_I2C_SCL);
		*( (volatile unsigned long *) GPIO_A_RISING_EDGE_DETECT ) &= ~(1UL <<  CONFIG_OXNAS_I2C_SCL);
		*( (volatile unsigned long *) GPIO_A_FALLING_EDGE_DETECT ) &= ~(1UL <<  CONFIG_OXNAS_I2C_SCL);
		pGPIO->found |= GPIO_TEST_SCL;
	}
	
	if ( test_bit( CONFIG_OXNAS_I2C_SDA, (volatile unsigned long *) &temp ) )
	{
		printk("Int Found CONFIG_OXNAS_I2C_SDA\n");
		*( (volatile unsigned long *) GPIO_A_RISING_EDGE_ACTIVE_HIGH_ENABLE ) &= ~(1UL <<  CONFIG_OXNAS_I2C_SDA);
		*( (volatile unsigned long *) GPIO_A_FALLING_EDGE_ACTIVE_LOW_ENABLE ) &= ~(1UL <<  CONFIG_OXNAS_I2C_SDA);
		*( (volatile unsigned long *) GPIO_A_LEVEL_INTERRUPT_ENABLE ) &= ~(1UL <<  CONFIG_OXNAS_I2C_SDA);
		*( (volatile unsigned long *) GPIO_A_RISING_EDGE_DETECT ) &= ~(1UL <<  CONFIG_OXNAS_I2C_SDA);
		*( (volatile unsigned long *) GPIO_A_FALLING_EDGE_DETECT ) &= ~(1UL <<  CONFIG_OXNAS_I2C_SDA);
		pGPIO->found |= GPIO_TEST_SDA;
	}
	
	if ( pGPIO->found )
	{
		printk("Int cleared\n");
		wake_up_interruptible(&ir_block);
	}
	
	return IRQ_HANDLED;
}

int gpioTest_go(void)
{
	unsigned long flags;
	unsigned int  temp;
	unsigned int  i;
	
	//assert( CONFIG_OXNAS_I2C_SDA < CONFIG_OXNAS_I2C_SCL );

	printk( KERN_ERR "****************************************************************\n");
	printk( KERN_ERR "**************************************************  gpioTest_go:\n");
	printk( KERN_ERR "Please connect I2c SDA to I2C SCLK, and optionally attach scope:\n\n");
	
	printk( KERN_ERR "Test 1: setup all inputs (and check for pull up res)\n");

	// Setting lines to GPIO
	*( (volatile unsigned long *) SYS_CTRL_GPIO_PRIMSEL_CTRL_0 ) &= ~(1UL <<  (CONFIG_OXNAS_I2C_SDA  & 0x0000001F) );
	*( (volatile unsigned long *) SYS_CTRL_GPIO_PRIMSEL_CTRL_0 ) &= ~(1UL <<  (CONFIG_OXNAS_I2C_SCL  & 0x0000001F) );
	                                                                     
	*( (volatile unsigned long *) SYS_CTRL_GPIO_SECSEL_CTRL_0 ) &= ~(1UL <<  (CONFIG_OXNAS_I2C_SDA  & 0x0000001F) );	
	*( (volatile unsigned long *) SYS_CTRL_GPIO_SECSEL_CTRL_0 ) &= ~(1UL <<  (CONFIG_OXNAS_I2C_SCL  & 0x0000001F) );	
  
// Setting lines to Input 
/*
temp  = readl( GPIO_A_OUTPUT_ENABLE );
temp |= (GPIO_TEST_SCL | GPIO_TEST_SDA | GPIO_TEST_SCS);
writel( temp, GPIO_A_OUTPUT_ENABLE );
for (i=0; i<65000; ++i)
{
	temp = (i & 0x00000007) << CONFIG_OXNAS_I2C_SCL;
	flags = readl( GPIO_A_OUTPUT_VALUE );
	flags &= ~(GPIO_TEST_SCL | GPIO_TEST_SDA | GPIO_TEST_SCS);
	flags |= temp;
	writel( flags, GPIO_A_OUTPUT_VALUE );
	printk( "reads 0x%08x  ",readl( GPIO_A_DATA ) );
	udelay(100);
	printk( " 0x%08x\n",readl( GPIO_A_DATA ) );
	
}
*/
DumpGPIO();	
	// Setting lines to Input
	temp  = readl( GPIO_A_OUTPUT_ENABLE );
	temp &= ~(GPIO_TEST_SCL | GPIO_TEST_SDA);
	writel( temp, GPIO_A_OUTPUT_ENABLE );
	udelay(1);
	if ( *( (volatile unsigned long *) GPIO_A_OUTPUT_ENABLE) & (1UL << CONFIG_OXNAS_I2C_SDA) ||
	     *( (volatile unsigned long *) GPIO_A_OUTPUT_ENABLE) & (1UL << CONFIG_OXNAS_I2C_SCL))
	{
		printk( KERN_ERR "Test 1: Failed to clear GPIO Output Enable register)\n");
		return -1;
	}
	
	udelay(1);
	// read the input value: it should be one due to the pull up
	if ( !( *( (volatile unsigned long *) GPIO_A_DATA) & (1UL << CONFIG_OXNAS_I2C_SDA) ) ||
	     !( *( (volatile unsigned long *) GPIO_A_DATA) & (1UL << CONFIG_OXNAS_I2C_SCL) ))
	{
		printk( KERN_ERR "Test 1: Failed to read 1 on the input(check for pull up res.))\n");
		return -1;
	}
	
	printk( KERN_ERR "===== Passed Test 1 =======   Created GPIO inputs, and read correct values\n");
	
	// Test 2: set output using direct setting, first set the output latches to zero on all lines
	printk( KERN_ERR "Test 2: output testing and OE testing\n");
	*( (volatile unsigned long *) GPIO_A_OUTPUT_CLEAR   ) |= (1UL <<  (CONFIG_OXNAS_I2C_SDA  & 0x0000001F) );	
	*( (volatile unsigned long *) GPIO_A_OUTPUT_CLEAR   ) |= (1UL <<  (CONFIG_OXNAS_I2C_SCL  & 0x0000001F) );	
	for (i=CONFIG_OXNAS_I2C_SCL; i<=CONFIG_OXNAS_I2C_SDA; ++i)
	{
		// from all input, set output using direct write. Assume all input at start.
		temp  = readl( GPIO_A_OUTPUT_ENABLE );
		temp |= ((0x00000001) << i);
		writel( temp, GPIO_A_OUTPUT_ENABLE );
	
		udelay(1);
		if ( ( *( (volatile unsigned long *) GPIO_A_DATA) & (1UL << CONFIG_OXNAS_I2C_SDA) ) ||
		     ( *( (volatile unsigned long *) GPIO_A_DATA) & (1UL << CONFIG_OXNAS_I2C_SCL) ))
		{
			printk( KERN_ERR "Test 2: Failed to read 0 on the %d input (output enabled with direct write.))\n", i);
			return -1;
		}
		
		// Setting lines to Input
		temp  = readl( GPIO_A_OUTPUT_ENABLE );
		temp &= ~(GPIO_TEST_SCL | GPIO_TEST_SDA);
		writel( temp, GPIO_A_OUTPUT_ENABLE );
			
		udelay(1);
		if ( ( *( (volatile unsigned long *) GPIO_A_OUTPUT_ENABLE) & (1UL << CONFIG_OXNAS_I2C_SDA) ) ||
		     ( *( (volatile unsigned long *) GPIO_A_OUTPUT_ENABLE) & (1UL << CONFIG_OXNAS_I2C_SCL) ) ||
		     !( *( (volatile unsigned long *) GPIO_A_DATA) & (1UL << CONFIG_OXNAS_I2C_SDA) )         ||
		     !( *( (volatile unsigned long *) GPIO_A_DATA) & (1UL << CONFIG_OXNAS_I2C_SCL) ) )
		{
			printk( KERN_ERR "Test 2: Failed to reset to clean state 1)\n");
			return -1;
		}
			
		// from all inputs, set an output enable usign output enable set
		*( (volatile unsigned long *) GPIO_A_OUTPUT_ENABLE_SET ) |= (1UL <<  i);		
		
		udelay(1);
		if ( ( *( (volatile unsigned long *) GPIO_A_DATA) & (1UL << CONFIG_OXNAS_I2C_SDA) ) ||
		     ( *( (volatile unsigned long *) GPIO_A_DATA) & (1UL << CONFIG_OXNAS_I2C_SCL) ) )
		{
			printk( KERN_ERR "Test 2: Failed to read 0 on the input %d (output enabled with direct write.))\n", i);
			return -1;
		}
		
		// now exercise the setting and clearing of the output value
		*( (volatile unsigned long *) GPIO_A_OUTPUT_SET ) |= (1UL <<  i);		
		
		udelay(1);
		if ( !( *( (volatile unsigned long *) GPIO_A_DATA) & (1UL << CONFIG_OXNAS_I2C_SDA) ) ||
		     !( *( (volatile unsigned long *) GPIO_A_DATA) & (1UL << CONFIG_OXNAS_I2C_SCL) )    )
		{
			printk( KERN_ERR "Test 2: Failed to read 1 on the input after settin %d ahs high)\n", i);
			return -1;
		}
		
		// now exercise the setting and clearing of the output value
		*( (volatile unsigned long *) GPIO_A_OUTPUT_CLEAR ) |= (1UL <<  i);		
		
		udelay(1);
		if ( ( *( (volatile unsigned long *) GPIO_A_DATA) & (1UL << CONFIG_OXNAS_I2C_SDA) ) ||
		     ( *( (volatile unsigned long *) GPIO_A_DATA) & (1UL << CONFIG_OXNAS_I2C_SCL) ) )
		{
			printk( KERN_ERR "Test 2: Failed to read 1 on the input after settin %d ahs high)\n", i);
			return -1;
		}
		
		// from i as output, set all inputs , set an output enable usign output enable set
		*( (volatile unsigned long *) GPIO_A_OUTPUT_ENABLE_CLEAR ) |= (1UL <<  i);		
		
		udelay(1);
		if ( temp != readl( GPIO_A_OUTPUT_ENABLE )        ||
		     ( *( (volatile unsigned long *) GPIO_A_OUTPUT_ENABLE) & (1UL << CONFIG_OXNAS_I2C_SDA) ) ||
		     ( *( (volatile unsigned long *) GPIO_A_OUTPUT_ENABLE) & (1UL << CONFIG_OXNAS_I2C_SCL) ) ||
		     !( *( (volatile unsigned long *) GPIO_A_DATA) & (1UL << CONFIG_OXNAS_I2C_SDA) )         ||
		     !( *( (volatile unsigned long *) GPIO_A_DATA) & (1UL << CONFIG_OXNAS_I2C_SCL) )           )
		{
			printk( KERN_ERR "Test 2: Failed to reset to clean state 2)\n");
			return -1;
		}
	}
	
	printk( KERN_ERR "===== Passed Test 2 =======   Used all Set and clear methods for both OE and OV\n");
	
	// Test 3, now exercise the interrupt.
	printk( KERN_ERR "Test 3: Interrupt testing\n");
	init_waitqueue_head(&ir_block);

	request_irq( 
		(int) GPIO_1_INTERRUPT,  	// unsigned int 	irq, 
		&irqHandler,             	// irqreturn_t (*handler)(int, void *, struct pt_regs *),
		IRQF_SHARED,                	// unsigned long 	irq_flags,
		"GPIO Test Module",      	// const char * 	devname,
		(void*) gpioTest_device  );   	// void * dev_id)
	
	
	// set int condiiton....
	for (i=CONFIG_OXNAS_I2C_SCL; i<=CONFIG_OXNAS_I2C_SDA; ++i)
	{
		int tmo;
		int drv = i+1 > CONFIG_OXNAS_I2C_SDA ? CONFIG_OXNAS_I2C_SCL : i+1;	// BHC - This is rubbish, there's no contract saying SDA has to be on a higher numbered GPIO than SCL
		
		
		// ================================================= level detect
		// set an interrupt on i being high-level
		printk( KERN_INFO "Test %d level int High against OP %d\n", i, drv);
		local_irq_save(flags);
		*( (volatile unsigned long *) GPIO_A_RISING_EDGE_ACTIVE_HIGH_ENABLE ) |= (1UL <<  i);
		*( (volatile unsigned long *) GPIO_A_FALLING_EDGE_ACTIVE_LOW_ENABLE ) &= ~(1UL <<  i);		
		*( (volatile unsigned long *) GPIO_A_LEVEL_INTERRUPT_ENABLE )         |= (1UL <<  i);
		*( (volatile unsigned long *) GPIO_A_INTERRUPT_EVENT ) |= (1UL <<  i);
		gpioTest_device->found = 0;
		local_irq_restore(flags);

 		tmo = wait_event_interruptible_timeout(ir_block, gpioTest_device->found, 1*HZ);
printk("tmo1 == 0x%08x / 0x%08x\n", tmo, 1*HZ );
		if ( !(gpioTest_device->found & ((0x00000001 << i))) )
		{
			
			printk( KERN_ERR "$R FAILED Test 3 timed out 1 sec with no interrupt "
					  "While waiting for level high int....\n");
			return -1;
		}

		// Next try setting the lines low, and ensure that the devices times out
		local_irq_save(flags);
		*( (volatile unsigned long *) GPIO_A_OUTPUT_CLEAR 			      ) |= (1UL <<  drv);		
		*( (volatile unsigned long *) GPIO_A_OUTPUT_ENABLE_SET		      ) |= (1UL <<  drv);		
		*( (volatile unsigned long *) GPIO_A_RISING_EDGE_ACTIVE_HIGH_ENABLE ) |= (1UL <<  i);
		*( (volatile unsigned long *) GPIO_A_FALLING_EDGE_ACTIVE_LOW_ENABLE ) &= ~(1UL <<  i);		
		*( (volatile unsigned long *) GPIO_A_INTERRUPT_EVENT 		      ) |= (1UL <<  i);
		*( (volatile unsigned long *) GPIO_A_LEVEL_INTERRUPT_ENABLE         ) |= (1UL <<  i);	
		gpioTest_device->found = 0;

		local_irq_restore(flags);
		tmo = wait_event_interruptible_timeout(ir_block, gpioTest_device->found, 1*HZ);
printk("tmo2 == 0x%08x\n", tmo );
		if ( gpioTest_device->found )
		{
			
			printk( KERN_ERR "$R FAILED Test 3 did not timed out "
					  "While waiting for level high int with 0 input....\n");
			return -1;
		}

		// set the int acive low level now.
		printk( KERN_INFO "Test %d level int Low against OP %d\n", i, drv);
		local_irq_save(flags);
		*( (volatile unsigned long *) GPIO_A_RISING_EDGE_ACTIVE_HIGH_ENABLE ) &= ~(1UL <<  i);	
		*( (volatile unsigned long *) GPIO_A_FALLING_EDGE_ACTIVE_LOW_ENABLE ) |= (1UL <<  i);	
		*( (volatile unsigned long *) GPIO_A_LEVEL_INTERRUPT_ENABLE         ) |= (1UL <<  i);	
		*( (volatile unsigned long *) GPIO_A_INTERRUPT_EVENT                ) |= (1UL <<  i);
		gpioTest_device->found = 0;
		
		local_irq_restore(flags);
		tmo = wait_event_interruptible_timeout(ir_block, gpioTest_device->found, 1*HZ);
printk("tmo3 == 0x%08x / 0x%08x\n", tmo, 1*HZ );
		if ( !(gpioTest_device->found & ((0x00000001 << i))) )
		{
			
			printk( KERN_ERR "$R FAILED Test 3 timed out 1 sec with no interrupt "
					  "While waiting for level low int....\n");
			return -1;
		}
		
		// Next try setting the lines high, and ensure that the devices times out
		*( (volatile unsigned long *) GPIO_A_OUTPUT_ENABLE_CLEAR            ) |= ~(1UL <<  drv);		
		*( (volatile unsigned long *) GPIO_A_RISING_EDGE_ACTIVE_HIGH_ENABLE ) &= ~(1UL <<  i);	
		*( (volatile unsigned long *) GPIO_A_FALLING_EDGE_ACTIVE_LOW_ENABLE ) |= (1UL <<  i);	
		*( (volatile unsigned long *) GPIO_A_LEVEL_INTERRUPT_ENABLE         ) |= (1UL <<  i);	
		*( (volatile unsigned long *) GPIO_A_INTERRUPT_EVENT                ) |= (1UL <<  i);
		gpioTest_device->found = 0;

		local_irq_restore(flags);
		tmo = wait_event_interruptible_timeout(ir_block, gpioTest_device->found, 1*HZ);
printk("tmo4 == 0x%08x\n", tmo );
		if ( gpioTest_device->found )
		{
			
			printk( KERN_ERR "$R FAILED Test 3 did not timed out "
					  "While waiting for level low int with 1 input....\n");
			return -1;
		}
		
		// Setting lines to Input --------------------------------------
		temp  = readl( GPIO_A_OUTPUT_ENABLE );
		temp &= ~(GPIO_TEST_SCL | GPIO_TEST_SDA);
		writel( temp, GPIO_A_OUTPUT_ENABLE );
		// ================================================= Edge detect
		
		// set an interrupt on i being high-level
		local_irq_save(flags);
		*( (volatile unsigned long *) GPIO_A_RISING_EDGE_ACTIVE_HIGH_ENABLE ) |=  (1UL <<  i);
		*( (volatile unsigned long *) GPIO_A_FALLING_EDGE_ACTIVE_LOW_ENABLE ) &= ~(1UL <<  i);		
		*( (volatile unsigned long *) GPIO_A_LEVEL_INTERRUPT_ENABLE         ) &= ~(1UL <<  i);	
		*( (volatile unsigned long *) GPIO_A_INTERRUPT_EVENT                ) |=  (1UL <<  i);
		gpioTest_device->found = 0;
		
		printk( KERN_INFO "Test %d rising edge int High against falling edge on OP %d\n", i, drv);
		local_irq_restore(flags);
		
		*( (volatile unsigned long *) GPIO_A_OUTPUT_ENABLE_SET ) |=  (1UL <<  drv);		
		*( (volatile unsigned long *) GPIO_A_OUTPUT_CLEAR             ) |=  (1UL <<  drv);
		tmo = wait_event_interruptible_timeout(ir_block, gpioTest_device->found, 1*HZ);
		if ( gpioTest_device->found || tmo )
		{
			
			printk( KERN_ERR "$R FAILED Test 3 found int on %d falling edge, when set to rising...\n", i);
			return -1;
		}
		
		// now do th rising edge...
		*( (volatile unsigned long *) GPIO_A_OUTPUT_SET ) |=  (1UL <<  drv);
		
		tmo = wait_event_interruptible_timeout(ir_block, gpioTest_device->found, 1*HZ);
		if ( !(gpioTest_device->found & (0x00000001 << i)) || (tmo == 0) )
		{
			
			printk( KERN_ERR "$R FAILED Test 3 did not find int on %d rising edge, when set to rising...\n", i);
			return -1;
		}

		// set the int acive low level now.
		local_irq_save(flags);
		
		*( (volatile unsigned long *) GPIO_A_OUTPUT_ENABLE_SET ) |=  (1UL <<  drv);		
		*( (volatile unsigned long *) GPIO_A_OUTPUT_CLEAR             ) |=  (1UL <<  drv);
		
		*( (volatile unsigned long *) GPIO_A_RISING_EDGE_ACTIVE_HIGH_ENABLE ) &= ~(1UL <<  i);
		*( (volatile unsigned long *) GPIO_A_FALLING_EDGE_ACTIVE_LOW_ENABLE ) |=  (1UL <<  i);		
		*( (volatile unsigned long *) GPIO_A_LEVEL_INTERRUPT_ENABLE         ) &= ~(1UL <<  i);	
		*( (volatile unsigned long *) GPIO_A_INTERRUPT_EVENT                ) |=  (1UL <<  i);
		gpioTest_device->found = 0;
		
		printk( KERN_INFO "Test %d falling edge int against rising edge on OP %d\n", i, drv);
		local_irq_restore(flags);
		
		*( (volatile unsigned long *) GPIO_A_OUTPUT_ENABLE_SET ) |=  (1UL <<  drv);		
		*( (volatile unsigned long *) GPIO_A_OUTPUT_SET        ) |=  (1UL <<  drv);
		tmo = wait_event_interruptible_timeout(ir_block, gpioTest_device->found, 1*HZ);
		if ( gpioTest_device->found || tmo )
		{
			
			printk( KERN_ERR "$R FAILED Test 3 found int on %d falling edge, when set to rising...\n", i);
			return -1;
		}
		
		// now do th rising edge...
		*( (volatile unsigned long *) GPIO_A_OUTPUT_CLEAR) |=  (1UL <<  drv);
		
		tmo = wait_event_interruptible_timeout(ir_block, gpioTest_device->found, 1*HZ);
		if ( !(gpioTest_device->found & (0x00000001 << i)) || (tmo == 0) )
		{
			
			printk( KERN_ERR "$R FAILED Test 3 did not find int on %d rising edge, when set to rising...\n", i);
			return -1;
		}

		
		
		// Setting lines to Input --------------------------------------
		temp  = readl( GPIO_A_OUTPUT_ENABLE );
		temp &= ~(GPIO_TEST_SCL | GPIO_TEST_SDA);
		writel( temp, GPIO_A_OUTPUT_ENABLE );
	}
	
	
	printk("$G All gpiuo tests passed.\n");
	
	return 0;
}


/*
Read callback:
-> filp,  contains the device in its private data
-> buf,   the buffer in userspace
-> count, amount of that to be read
-> f_pos, the starting point of the data
-> return:number of bytes read
*/
ssize_t gpioTest_read(struct file *filp, char __user *buf, size_t count,loff_t *f_pos){
// dev was stored in filp during the open call.
// struct gpioTest_dev *dev = filp->private_data;
  
  printk( KERN_ERR "gpioTest_read:\n");

// Copy this quantum (from the offset, to the end of this quantum)
// to userspace
// if(copy_to_user(buf, dptr->data[s_pos] + q_pos, count)){
//   retval = -EFAULT;
//   goto out;
// }

// Update the file offset
// *f_pos += count;
// retval = count;
// out:
// up(&dev->sem);
  return 0;
}

/*
Write callback
-> filp,  contains the device in its private data
-> buf,   the buffer in userspace
-> count, amount of that to be written
-> f_pos, the starting point of the data
-> return:number of bytes written
*/
ssize_t gpioTest_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos){

//  struct gpioTest_dev *dev = filp->private_data;
  
  printk( KERN_ERR "gpioTest_write:\n");

 
  // Copy the data to be written from userspace
  // if(copy_from_user(dptr->data[s_pos]+q_pos, buf, count)){
  //   retval = -EFAULT;
  // }
  // 
  // *f_pos+=count;
  // retval = count;

  // Update the size
  // if(dev->size < *f_pos){
  //   dev->size = *f_pos;
  // }

// out:
  // up(&dev->sem);
  return 0;
}



/*
Release callback
*/
int gpioTest_release(struct inode *inode, struct file *filp)
{
  printk( KERN_ERR "gpioTest_release:\n");
  return 0;
}

/*
Open callback
*/
int gpioTest_open(struct inode *inode, struct file *filp){
//  struct gpioTest_dev *dev;

  printk( KERN_ERR "gpioTest_open:\n");
  // This macro takes a pointer to a field of type 'container_field', within
  // a strcture of type 'container_type' and returns a pointer to the containing
  // structure.
  // dev = container_of(inode->i_cdev, struct gpioTest_dev, cdev);
  // Store the pointer for future access
  // filp->private_data = dev;

  return 0;
}




//////////////////
// MODULE STUFF //
//////////////////

// File operations for this charater device
struct file_operations gpioTest_fops = {
    .owner =    THIS_MODULE,
    //.llseek =   gpioTest_llseek,
    .read =     gpioTest_read,
    .write =    gpioTest_write,
    //    .ioctl =    gpioTest_ioctl,
    .open =     gpioTest_open,
    .release =  gpioTest_release,
};


/*
 Module loading
*/
static int gpioTest_init(void){
	
	// alloc_chrdev_region return 0 on success
	int res = alloc_chrdev_region(
		&dev,
		0,
		1, 
		"GPIOTest");

	printk( KERN_ERR "gpioTest_init:\n");
	
	if(res){
		printk(KERN_WARNING "gpioTest: could not allocate device\n");
		return res;
	}else{
		printk(KERN_WARNING "gpioTest: registered with major number:%i\n", MAJOR(dev));
	}
	
	
	// Allocate memory for gpioTest_COUNT gpioTest_devices
	gpioTest_device = kmalloc(sizeof(struct gpioTest_dev), GFP_KERNEL);
	if(gpioTest_device == NULL){
		res = -ENOMEM;
		goto fail;
	}
	
	// Fill the gpioTest_devices region with zeros
	memset(gpioTest_device, 0, sizeof(struct gpioTest_dev));
	
	// Initialise the devices
   
	// Register the cdev, gpioTest_fops contains all the defined callbacks
	cdev_init(&gpioTest_device->cdev,&gpioTest_fops);
	gpioTest_device->cdev.owner 	= THIS_MODULE;
	gpioTest_device->cdev.ops 	= &gpioTest_fops;
	res 				= cdev_add(&gpioTest_device->cdev,MKDEV(MAJOR(dev), MINOR(dev)) ,1);
	if(res){
		printk(KERN_NOTICE "Error %d adding gpioTest\n", res);
	}else{
		printk(KERN_NOTICE "gpioTest added\n");
	}
	
	// perform a test
	res  = readl( GPIO_A_INPUT_DEBOUNCE_ENABLE );
		res &= ~(GPIO_TEST_SCL | GPIO_TEST_SDA);
		writel( res, GPIO_A_INPUT_DEBOUNCE_ENABLE );
	if ( gpioTest_go() )
	{
		DumpGPIO();
	}
	
	// perform a test again with debounce.
	res  = readl( GPIO_A_INPUT_DEBOUNCE_ENABLE );
		res |= (GPIO_TEST_SCL | GPIO_TEST_SDA);
		writel( res, GPIO_A_INPUT_DEBOUNCE_ENABLE );
	if ( gpioTest_go() )
	{
		DumpGPIO();
	}
	


  return 0;

fail:
  // do cleanup;
  return res;
}


/*
 Module unloading
*/
static void gpioTest_exit(void){
	// Free the devices
	printk( KERN_ERR "gpioTest_exit:\n");
	cdev_del(&gpioTest_device->cdev);
	kfree(gpioTest_device);
	gpioTest_device = NULL;
	unregister_chrdev_region(dev,1);
}

module_init(gpioTest_init);
module_exit(gpioTest_exit);
