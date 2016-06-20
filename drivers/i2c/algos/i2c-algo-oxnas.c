/*
 * i2c-algo-oxnas.c i2x driver algorithms for MPCoxnas 
 * Copyright (c) 1999 Dan Malek (dmalek@jlc.net).
 *
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

// XXX todo
// timeout sleep?


/* $Id: i2c-algo-oxnas.c,v 1.15 2004/11/20 08:02:24 khali Exp $ */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include "linux/i2c.h"
#include "linux/i2c-algo-oxnas.h"
#include <asm/bitops.h>



#define OXNAS_MAX_READ	513
/* #define I2C_CHIP_ERRATA */ /* Try uncomment this if you have an older CPU(earlier than rev D4) */ //NOTE
static wait_queue_head_t iic_wait;

int oxnas_debug = 1;
int oxnas_scan  = 1;

static inline  void oxnas_iic_algo_dump_reg( void )
{
      i2c_registers_oxnas_t* i2c = (i2c_registers_oxnas_t*) I2C_BASE; // SERIAL_MASTER_CONTROL_BASE;
                                                                                                        
      printk( KERN_INFO "\n\n ==================================================================" );
      printk( KERN_INFO "    i2c->SerialControlRegister;                == 0x%08x @ %p\n", i2c->SerialControlRegister                 , &(i2c->SerialControlRegister                ) );      /* SERIAL_MASTER_CONTROL_BASE + 0x00;  */
      printk( KERN_INFO "    i2c->SerialAddressRegister;                == 0x%08x @ %p\n", i2c->SerialAddressRegister                 , &(i2c->SerialAddressRegister                ) );      /* SERIAL_MASTER_CONTROL_BASE + 0x04;  */
      printk( KERN_INFO "    i2c->SerialSWControlOutRegister;           == 0x%08x @ %p\n", i2c->SerialSWControlOutRegister            , &(i2c->SerialSWControlOutRegister           ) );      /* SERIAL_MASTER_CONTROL_BASE + 0x08;  */
      printk( KERN_INFO "    i2c->SerialSWControlInRegister;            == 0x%08x @ %p\n", i2c->SerialSWControlInRegister             , &(i2c->SerialSWControlInRegister            ) );      /* SERIAL_MASTER_CONTROL_BASE + 0x0C;  */
      printk( KERN_INFO "    i2c->SerialInterruptStatusRegister;        == 0x%08x @ %p\n", i2c->SerialInterruptStatusRegister         , &(i2c->SerialInterruptStatusRegister        ) );      /* SERIAL_MASTER_CONTROL_BASE + 0x10;  */
      printk( KERN_INFO "    i2c->SerialInterruptEnableRegister;        == 0x%08x @ %p\n", i2c->SerialInterruptEnableRegister         , &(i2c->SerialInterruptEnableRegister        ) );      /* SERIAL_MASTER_CONTROL_BASE + 0x14;  */
      printk( KERN_INFO "\n" );      /* PAD REGISTER PACKING ***/
      printk( KERN_INFO "    i2c->SerialReadData1Register;              == 0x%08x @ %p\n", i2c->SerialReadData1Register               , &(i2c->SerialReadData1Register              ) );      /* SERIAL_MASTER_CONTROL_BASE + 0x20;  */
      printk( KERN_INFO "    i2c->SerialReadData2Register;              == 0x%08x @ %p\n", i2c->SerialReadData2Register               , &(i2c->SerialReadData2Register              ) );      /* SERIAL_MASTER_CONTROL_BASE + 0x24;  */
      printk( KERN_INFO "    i2c->SerialReadData3Register;              == 0x%08x @ %p\n", i2c->SerialReadData3Register               , &(i2c->SerialReadData3Register              ) );      /* SERIAL_MASTER_CONTROL_BASE + 0x28;  */
      printk( KERN_INFO "    i2c->SerialReadData4Register;              == 0x%08x @ %p\n", i2c->SerialReadData4Register               , &(i2c->SerialReadData4Register              ) );      /* SERIAL_MASTER_CONTROL_BASE + 0x2C;  */
      printk( KERN_INFO "\n" );      /* PAD REGISTER PACKING ***/
      printk( KERN_INFO "    i2c->SerialWriteData1Register;             == 0x%08x @ %p\n", i2c->SerialWriteData1Register              , &(i2c->SerialWriteData1Register             ) );      /* SERIAL_MASTER_CONTROL_BASE + 0x40;  */
      printk( KERN_INFO "    i2c->SerialWriteData2Register;             == 0x%08x @ %p\n", i2c->SerialWriteData2Register              , &(i2c->SerialWriteData2Register             ) );      /* SERIAL_MASTER_CONTROL_BASE + 0x44;  */
      printk( KERN_INFO "    i2c->SerialWriteData3Register;             == 0x%08x @ %p\n", i2c->SerialWriteData3Register              , &(i2c->SerialWriteData3Register             ) );      /* SERIAL_MASTER_CONTROL_BASE + 0x48;  */
      printk( KERN_INFO "    i2c->SerialWriteData4Register;             == 0x%08x @ %p\n", i2c->SerialWriteData4Register              , &(i2c->SerialWriteData4Register             ) );      /* SERIAL_MASTER_CONTROL_BASE + 0x4C;  */
      printk( KERN_INFO "\n" );      /* PAD REGISTER PACKING ***/
      printk( KERN_INFO "    i2c->GenericSerialControlRegister;         == 0x%08x @ %p\n", i2c->GenericSerialControlRegister          , &(i2c->GenericSerialControlRegister         ) );      /* SERIAL_MASTER_CONTROL_BASE + 0x80;  */        
      printk( KERN_INFO "\n" );      /* PAD REGISTER PACKING ***/
      printk( KERN_INFO "    i2c->GenericSerialInterruptStatusRegister; == 0x%08x @ %p\n", i2c->GenericSerialInterruptStatusRegister  , &(i2c->GenericSerialInterruptStatusRegister ) );      /* SERIAL_MASTER_CONTROL_BASE + 0x90;  */
      printk( KERN_INFO "    i2c->GenericSerialInterruptEnableRegister; == 0x%08x @ %p\n", i2c->GenericSerialInterruptEnableRegister  , &(i2c->GenericSerialInterruptEnableRegister ) );      /* SERIAL_MASTER_CONTROL_BASE + 0x94;  */
      printk( KERN_INFO "\n" );      /* PAD REGISTER PACKING ***/
      printk( KERN_INFO "    i2c->GenericSerialReadData1Register;       == 0x%08x @ %p\n", i2c->GenericSerialReadData1Register        , &(i2c->GenericSerialReadData1Register       ) );      /* SERIAL_MASTER_CONTROL_BASE + 0xA0;  */
      printk( KERN_INFO "    i2c->GenericSerialReadData2Register;       == 0x%08x @ %p\n", i2c->GenericSerialReadData2Register        , &(i2c->GenericSerialReadData2Register       ) );      /* SERIAL_MASTER_CONTROL_BASE + 0xA4;  */
      printk( KERN_INFO "    i2c->GenericSerialReadData3Register;       == 0x%08x @ %p\n", i2c->GenericSerialReadData3Register        , &(i2c->GenericSerialReadData3Register       ) );      /* SERIAL_MASTER_CONTROL_BASE + 0xA8;  */
      printk( KERN_INFO "    i2c->GenericSerialReadData4Register;       == 0x%08x @ %p\n", i2c->GenericSerialReadData4Register        , &(i2c->GenericSerialReadData4Register       ) );      /* SERIAL_MASTER_CONTROL_BASE + 0xAC;  */
      printk( KERN_INFO "\n" );      /* PAD REGISTER PACKING ***/
      printk( KERN_INFO "    i2c->GenericSerialWriteData1Register;      == 0x%08x @ %p\n", i2c->GenericSerialWriteData1Register       , &(i2c->GenericSerialWriteData1Register      ) );      /* SERIAL_MASTER_CONTROL_BASE + 0xC0;  */
      printk( KERN_INFO "    i2c->GenericSerialWriteData2Register;      == 0x%08x @ %p\n", i2c->GenericSerialWriteData2Register       , &(i2c->GenericSerialWriteData2Register      ) );      /* SERIAL_MASTER_CONTROL_BASE + 0xC4;  */
      printk( KERN_INFO "    i2c->GenericSerialWriteData3Register;      == 0x%08x @ %p\n", i2c->GenericSerialWriteData3Register       , &(i2c->GenericSerialWriteData3Register      ) );      /* SERIAL_MASTER_CONTROL_BASE + 0xC8;  */
      printk( KERN_INFO "    i2c->GenericSerialWriteData4Register;      == 0x%08x @ %p\n", i2c->GenericSerialWriteData4Register       , &(i2c->GenericSerialWriteData4Register      ) );      /* SERIAL_MASTER_CONTROL_BASE + 0xCC;  */
      printk( KERN_INFO "\n\n =================================================================="  );

	
}

static inline int oxnas_iic_algo_bus_reset( volatile struct i2c_algo_oxnas_data* oxnas )
{
	/* perform a bus reset to clean up */
	
	unsigned long flags, tmo;
	volatile i2c_registers_oxnas_t *pI2C = (i2c_registers_oxnas_t *) oxnas;
	
	local_irq_save(flags);
	oxnas->iTransferInProgress_ = 1;
	
        pI2C->SerialControlRegister = 
		(I2C_SCR_RESET << I2C_SCR_TRANSACTION_TYPE_BIT     ) |
		(I2C_SCR_RESET << I2C_SCR_TRANSACTION_PROGRESS_BIT );
		
	/* Wait for IIC transfer */
	tmo = interruptible_sleep_on_timeout(&iic_wait,1*HZ);
		
	// Flag that the transfer has finished
	oxnas->iTransferInProgress_ = 0;

	local_irq_restore(flags);
	
	return (tmo < 1*HZ);
}


static  void
oxnas_iic_algo_interrupt(void *dev_id, struct pt_regs *regs)
{
	volatile i2c_registers_oxnas_t *i2cReg = (i2c_registers_oxnas_t *)dev_id;
	if (oxnas_debug)
		printk("oxnas_iic_algo_interrupt(dev_id=%p)\n", dev_id);

	/* Clear interrupt.
	*/
	i2cReg->SerialInterruptStatusRegister &= ~(1UL <<  I2C_ISR_INTERRUPT_STATUS_BIT);

	/* Get 'me going again.
	*/
	wake_up_interruptible(&iic_wait);
}

static void
oxnas_iic_algo_init(struct i2c_algo_oxnas_data *oxnas)
{
	u32 temp;
	volatile i2c_registers_oxnas_t*	i2c = oxnas->i2c;

	if (oxnas_debug) printk(KERN_INFO "oxnas_iic_algo_init()\n");

	/* Initialize 
	 * Set up the IIC parameters 
	 */
	temp   = i2c->SerialControlRegister & I2C_SCR_READ_MASK;
	temp >>= I2C_SCR_AUTO_INCREMENT_BUFFER_SIZE_BIT;
	temp  &= ~(0xffffffff << ( I2C_SCR_AUTO_INCREMENT_BUFFER_SIZE_NUM_BITS ));
	oxnas->iMaxAutoIncTransfer_ = temp;

        // Initialise the Serial controller(s)
        {
		// syslib::Lock lock(mutex);
		
		// TODO: Ensure the Serial block is properly reset
		// BlockResetRegister& blockResetRegister = BlockResetRegister::Acquire();
		// blockResetRegister.ResetSerial();
		// blockResetRegister.CommitWrites();
		// blockResetRegister.Release();
		
		// TODO: Enable the clock to the Serial block
		// ClockStartRegister& clockStartRegister = ClockStartRegister::Acquire();
		// clockStartRegister.RefreshReadData();
		// clockStartRegister.StartSerialClock();
		// clockStartRegister.CommitWrites();
		// clockStartRegister.Release();
		
		// TODO: Set the Serial clock rate
		// SerialClockSelectRegister& serialClockSelectRegister = SerialClockSelectRegister::GetInstance();
		// serialClockSelectRegister.SetClockRate(SerialClockSelectRegister::PLL_DIV_32768);
		// serialClockSelectRegister.CommitWrites();
		
		// Disable the Serial Interrupt
		if (oxnas_debug) printk(KERN_INFO "    - Disabling pre existing i2c interrupt\n");
		i2c->SerialInterruptEnableRegister &= ~(1UL <<  I2C_IER_SERIAL_ENABLE_BIT);
		
		// Disable the Generic serial Interrupt
		if (oxnas_debug) printk(KERN_INFO "    - Disabling pre existing gen serial interrupt\n");
		i2c->SerialInterruptEnableRegister &= ~(1UL <<  I2C_IER_GEN_ENABLE_BIT);
				
		// Clear any pending Serial interrupts
		if ( i2c->SerialInterruptStatusRegister | (1UL <<  I2C_ISR_INTERRUPT_STATUS_BIT) )
		{
			// Yes, so clear the interrupt
			if (oxnas_debug) printk(KERN_INFO "    - Clearing pre existing i2c interrupt\n");
			*( &(i2c->SerialInterruptStatusRegister) ) |= (1UL <<  I2C_ISR_INTERRUPT_STATUS_BIT);
		}
		
		// Clear any pending Generic serial interrupts
		if (  i2c->SerialInterruptStatusRegister | (1UL <<  I2C_ISR_GEN_INTERRUPT_STATUS_BIT) )
		{
			// Yes, so clear the interrupt
			if (oxnas_debug) printk(KERN_INFO "    - Clearing pre existing generic serial interrupt\n");
			*( &(i2c->SerialInterruptStatusRegister) ) |= (1UL <<  I2C_ISR_GEN_INTERRUPT_STATUS_BIT);
		}
		
				
		// Initialise the generic serial hardware, which shares reset,
		// clock and interrupt hardware with the Serial controller(s)
		// TODO: GenericSerialHelper::Init();
		
	}
	
	init_waitqueue_head(&iic_wait);

	/* Install interrupt handler.
	*/
	if (oxnas_debug) {
		printk ("%s[%d] Install ISR for IRQ %d\n",
			__func__,__LINE__, I2C_INTERRUPT  );
	}
	
	(*oxnas->setisr)( (int) I2C_INTERRUPT, &oxnas_iic_algo_interrupt, (void *)i2c);
if (oxnas_debug)oxnas_iic_algo_dump_reg();
}


static int
oxnas_iic_algo_shutdown(struct i2c_algo_oxnas_data *oxnas)
{
	volatile i2c_registers_oxnas_t *i2c = oxnas->i2c;

	if (oxnas_debug) printk("oxnas_iic_algo_shutdown()\n");

	/* Shut down IIC.
	*/
        // TODO: syslib::Lock lock(mutex);

	// TODO: Reset Serial block to ensure there are no actve transfers
	// BlockResetRegister& blockResetRegister = BlockResetRegister::Acquire();
	// blockResetRegister.ResetSerial();
	// blockResetRegister.CommitWrites();
	// blockResetRegister.Release();
	
	// Disable the Serial Interrupt
	if (oxnas_debug) printk(KERN_INFO "    - Disabling pre existing i2c interrupt\n");
	i2c->SerialInterruptEnableRegister &= ~(1UL <<  I2C_IER_SERIAL_ENABLE_BIT);
	
	// Disable the Generic serial Interrupt
	if (oxnas_debug) printk(KERN_INFO "    - Disabling pre existing gen serial interrupt\n");
	i2c->SerialInterruptEnableRegister &= ~(1UL <<  I2C_IER_GEN_ENABLE_BIT);
			

	// Shutdown the generic serial hardware, which shares reset, clock and
	// interrupt hardware with the Serial controller(s)
	// TODO: GenericSerialHelper::Shutdown();
	
	// TODO: Disable the clock to the Serial block
	// ClockStopRegister& clockStopRegister = ClockStopRegister::Acquire();
	// clockStopRegister.RefreshReadData();
	// clockStopRegister.StopSerialClock();
	// clockStopRegister.CommitWrites();
	// clockStopRegister.Release();
	
	(*oxnas->clearisr)( (int) I2C_INTERRUPT, (void *)i2c);
if (oxnas_debug)oxnas_iic_algo_dump_reg();

	return(0);
}


#define BD_SC_NAK		((ushort)0x0004) /* NAK - did not respond */
#define BD_SC_OV			((ushort)0x0002) /* OV - receive overrun */
#define OXNAS_CR_CLOSE_RXBD	((ushort)0x0007)

static void force_close(struct i2c_algo_oxnas_data *oxnas)
{	
	volatile i2c_registers_oxnas_t *i2c = oxnas->i2c;
	
	if (oxnas_debug) printk("force_close()\n");

	*( &(i2c->SerialControlRegister) ) |= (1UL <<  I2C_SCR_ABORT_BIT);
	
	/* perform a bus reset to clean up */
	oxnas_iic_algo_bus_reset(oxnas);
	
if (oxnas_debug)oxnas_iic_algo_dump_reg();
}


/* Read from IIC...
 * abyte = address byte, with r/w flag already set
 */
static int
oxnas_iic_algo_read(struct i2c_algo_oxnas_data *oxnas, u_char abyte, char *readBuffer, int readBufferLength)
{
	volatile i2c_registers_oxnas_t *i2c = oxnas->i2c;
	const unsigned char* pData;
	unsigned long flags, tmo, temp, bytesTransfered;
	

	if (oxnas_debug) printk("oxnas_iic_algo_read(abyte=0x%x)\n", abyte);
	
	if (readBufferLength >= oxnas->iMaxAutoIncTransfer_ ) {
		if (oxnas_debug) printk("oxnas_iic_algo_read $RFailed to reaad %d auto. max is %d\n", readBufferLength, oxnas->iMaxAutoIncTransfer_ );
		return -EINVAL;
	}


	if( 1 /*TODO: Split into multipacks. */ ) 
	{
		
		local_irq_save(flags);
		
		/* QUESTION: Does this get locked by the parent? it should be!! */
		oxnas->iTransferInProgress_  = 1;
		oxnas->iError_               = 0;
	
		// Set up the 7-bit slave address 
		i2c->SerialAddressRegister = I2C_SAR_WRITE_MASK & ((abyte >> 1) << I2C_SAR_SEVEN_BIT_ADDRESS_BIT);
	
		// Setup the control register
		temp = ( I2C_SCR_READ       << I2C_SCR_READ_WRITE_BIT            ) |
                       ( I2C_SCR_NORMAL     << I2C_SCR_TRANSACTION_TYPE_BIT      ) |
                       ( I2C_SCR_SEVEN_BIT  << I2C_SCR_ADDRESS_MODE_BIT	         ) |
                       ( 0                  << I2C_SCR_SCCB_MODE_ENABLE_BIT      ) |
                       ( 1                  << I2C_SCR_SCCB_MODE_RESPECT_ACK_BIT ) |       
		       ( 1		    << I2C_SCR_AUTO_INCREMENT_ENABLE_BIT ) |
		       ( 1		    << I2C_SCR_ENABLE_SLAVE_HOLD_OFF_BIT ) |
		       ( 0 		    << I2C_SCR_HIGH_SPEED_DRIVE_BIT      ) |
                       ( readBufferLength   << I2C_SCR_BYTES_TO_TRANSFER_BIT     );
		       
		temp &= I2C_SCR_WRITE_MASK;		       
		i2c->SerialControlRegister = temp;
		
		/* Enable some interupts */
		*( &(i2c->SerialInterruptEnableRegister) ) |= (1UL <<  I2C_IER_SERIAL_ENABLE_BIT);
		
		/* Begin transmission */
		*( &(i2c->SerialControlRegister) ) |= (1UL <<  I2C_SCR_TRANSACTION_PROGRESS_BIT);

		/* Wait for IIC transfer */
		tmo = interruptible_sleep_on_timeout(&iic_wait,1*HZ);
	
		/* Woken. Copy data out of special registers. */

		temp = i2c->SerialControlRegister;

		// How many bytes were read from the slave?
		bytesTransfered = (temp >> I2C_SCR_BYTES_TO_TRANSFER_BIT) & 
				  ~(0xffffffff << I2C_SCR_BYTES_TO_TRANSFER_NUM_BITS);

		// Did the transfer fail
		if ( temp | (1UL <<  I2C_SCR_TRANSACTION_STATUS_BIT)  )
		{
			// Yes, so remember the error
			oxnas->iError_    = 1;
			readBufferLength  = 0;
		}
		else if (readBuffer)
		{
			if (bytesTransfered > readBufferLength)
			{
				// More bytes were read than we have buffer space to
				// store them
				oxnas->iError_ = 1;
			}
			else
			{
				// Copy the received data into the buffer that was provided by the
				// original caller to the Read() or ReadImmediate() method
				if (bytesTransfered > 0)
				{
					int i=0;
					temp = i2c->SerialReadData1Register;
					pData = (const unsigned char*) (&temp);
					readBuffer[i++] = *pData++;
					if (bytesTransfered > 1)
					{
						readBuffer[i++] = *pData++;
						if (bytesTransfered > 2)
						{
							readBuffer[i++] = *pData++;
							if (bytesTransfered > 3)
							{
								readBuffer[i++] = *pData++;
							}
						}
					}
			
					if (bytesTransfered > 4)
					{
						temp = i2c->SerialReadData2Register;
						pData = (const unsigned char*) (&temp);
						readBuffer[i++] = *pData++;
						if (bytesTransfered > 5)
						{
							readBuffer[i++] = *pData++;
							if (bytesTransfered > 6)
							{
								readBuffer[i++] = *pData++;
								if (bytesTransfered > 7)
								{
									readBuffer[i++] = *pData++;
								}
							}
						}
					}
			
					if (bytesTransfered > 8)
					{
						temp = i2c->SerialReadData3Register;
						pData = (const unsigned char*) (&temp);
						if (bytesTransfered > 9)
						{
							readBuffer[i++] = *pData++;
							if (bytesTransfered > 10)
							{
								readBuffer[i++] = *pData++;
								if (bytesTransfered > 11)
								{
									readBuffer[i++] = *pData++;
								}
							}
						}
					}
					
					if (bytesTransfered > 12)
					{
						temp = i2c->SerialReadData4Register;
						pData = (const unsigned char*) (&temp);
						if (bytesTransfered > 13)
						{
							readBuffer[i++] = *pData++;
							if (bytesTransfered > 14)
							{
								readBuffer[i++] = *pData++;
								if (bytesTransfered > 15)
								{
									readBuffer[i++] = *pData++;
								}
							}
						}
					}
				}
			}
		}
            

		// Flag that the transfer has finished
		oxnas->iTransferInProgress_ = 0;
	
		local_irq_restore(flags);
	}
	
	/* IDEA:  busy wait for small transfers, its faster  time_after(jiffies, tmo) */

	if (signal_pending(current) || !tmo){
		force_close(oxnas);
		if(oxnas_debug) 
			printk("IIC read: timeout!\n");
		return -EIO;
	}
	
	if ( i2c->SerialControlRegister | (1UL <<  I2C_SCR_TRANSACTION_STATUS_BIT) ) {
		if (oxnas_debug)
			printk("IIC read; no ack\n");
		return -EREMOTEIO;
	}

	if (bytesTransfered > readBufferLength) {
		if (oxnas_debug)
			printk("IIC read; Overrun\n");
		return -EREMOTEIO;;
	}

	if (oxnas_debug) printk("read %u bytes\n", readBufferLength);

	if (bytesTransfered < readBufferLength) {
		if (oxnas_debug)
			printk("IIC read; short, wanted %lu got %ld\n",
			       bytesTransfered, readBufferLength);
		return 0;
	}

	return bytesTransfered;
}


static void LoadWriteRegisters(
	volatile i2c_registers_oxnas_t *i2c, 
	char *data,
	int length )
{
	// Copy the data to be transmited into the write registers
	u32 temp;
	if (length > 0)
	{
		int i=0;
		unsigned char* pData = (unsigned char*) &temp;
		*pData++ = (data[i++]);
		if (length > 1)
		{
			*pData++ = (data[i++]);
			if (length > 2)
			{
				*pData++ = (data[i++]);
				if (length > 3)
				{
					*pData++ = (data[i++]);
				}
			}
		}
		i2c->SerialWriteData1Register = temp;
		
		if (length > 4)
		{
			pData = (unsigned char*) (&temp);
			*pData++ = (data[i++]);
			if (length > 5)
			{
				*pData++ = (data[i++]);
				if (length > 6)
				{
					*pData++ = (data[i++]);
					if (length > 7)
					{
						*pData++ = (data[i++]);
					}
				}
			}
			i2c->SerialWriteData2Register = temp;
		}
		
		if (length > 8)
		{
			pData = (unsigned char*) (&temp);
			*pData++ = (data[i++]);
			if (length > 9)
			{
				*pData++ = (data[i++]);
				if (length > 10)
				{
					*pData++ = (data[i++]);
					if (length > 11)	
					{
						*pData++ = (data[i++]);
					}
				}
			}
			i2c->SerialWriteData3Register = temp;
		}
		
		if (length > 12)
		{
			pData = (unsigned char*) (&temp);
			*pData++ = (data[i++]);	
			if (length > 13)
			{	
				*pData++ = (data[i++]);
				if (length > 14)
				{
					*pData++ = (data[i++]);
					if (length > 15)
					{
						*pData++ = (data[i++]);
					}	
				}
			}
			i2c->SerialWriteData4Register = temp;
		}
	}
}


/* Write to IIC...
 * addr = address byte, with r/w flag already set
 */
static int
oxnas_iic_algo_write(struct i2c_algo_oxnas_data *oxnas, u_char abyte, char *buf,int count)
{
	volatile i2c_registers_oxnas_t *i2c = oxnas->i2c;
	unsigned long flags, tmo, bytesTransfered, temp;

	if (oxnas_debug) printk("oxnas_iic_algo_write(abyte=0x%x)\n", abyte);

	if (count >= oxnas->iMaxAutoIncTransfer_ ) {
		if (oxnas_debug) printk("oxnas_iic_algo_read $RFailed to reaad %d auto. max is %d\n", count, oxnas->iMaxAutoIncTransfer_ );
		return -EINVAL;
	}

	if( 1 /* TODO: Split longer messages */ )
	{
		LoadWriteRegisters( i2c, buf, count );
		
		local_irq_save(flags);
		
		/* QUESTION: Does this get locked by the parent? it should be!! */
		oxnas->iTransferInProgress_ = 1;
		oxnas->iError_              = 0;
	
		// Set up the 7-bit slave address 
		i2c->SerialAddressRegister = I2C_SAR_WRITE_MASK & ((abyte >> 1) << I2C_SAR_SEVEN_BIT_ADDRESS_BIT);
	
		// Setup the control register
		temp = ( I2C_SCR_WRITE      << I2C_SCR_READ_WRITE_BIT            ) |
                       ( I2C_SCR_NORMAL     << I2C_SCR_TRANSACTION_TYPE_BIT      ) |
                       ( I2C_SCR_SEVEN_BIT  << I2C_SCR_ADDRESS_MODE_BIT	         ) |
                       ( 0                  << I2C_SCR_SCCB_MODE_ENABLE_BIT      ) |
                       ( 1                  << I2C_SCR_SCCB_MODE_RESPECT_ACK_BIT ) |       
		       ( 1		    << I2C_SCR_AUTO_INCREMENT_ENABLE_BIT ) |
		       ( 1		    << I2C_SCR_ENABLE_SLAVE_HOLD_OFF_BIT ) |
		       ( 0 		    << I2C_SCR_HIGH_SPEED_DRIVE_BIT      ) |
                       ( count              << I2C_SCR_BYTES_TO_TRANSFER_BIT     );
		       
		temp &= I2C_SCR_WRITE_MASK;		       
		i2c->SerialControlRegister = temp;
		
		/* Enable some interupts */
		*( &(i2c->SerialInterruptEnableRegister) ) |= (1UL <<  I2C_IER_SERIAL_ENABLE_BIT);
		
		/* Begin transmission */
		*( &i2c->SerialControlRegister ) |= (1UL <<  I2C_SCR_TRANSACTION_PROGRESS_BIT);

		/* Begin transmission */
		
		/* Wait for IIC transfer */
		tmo = interruptible_sleep_on_timeout(&iic_wait,1*HZ);
		local_irq_restore(flags);
	} 		

	/* IDEA:  busy wait for small transfers, its faster  time_after(jiffies, tmo) */
	
	
	if (signal_pending(current) || !tmo){
		force_close(oxnas);
		if(oxnas_debug) 
			printk("IIC read: timeout!\n");
		return -EIO;
	}
	
	if ( i2c->SerialControlRegister | (1UL <<  I2C_SCR_TRANSACTION_STATUS_BIT)) {
		if (oxnas_debug)
			printk("IIC read; no ack\n");
		return -EREMOTEIO;
	}

	// How many bytes were read from the slave?
	bytesTransfered = (temp >> I2C_SCR_BYTES_TO_TRANSFER_BIT) & 
			  ~(0xffffffff << I2C_SCR_BYTES_TO_TRANSFER_NUM_BITS);

	if (bytesTransfered > count) {
		if (oxnas_debug)
			printk("IIC read; Overrun\n");
		return -EREMOTEIO;;
	}

	if (oxnas_debug) printk("read %lu bytes\n", bytesTransfered);

	if (bytesTransfered < count) {
		if (oxnas_debug)
			printk("IIC read; short, wanted %u got %lu\n",
			       count, bytesTransfered);
		return 0;
	}

	return bytesTransfered;
}

/* See if an IIC address exists..
 * addr = 7 bit address, unshifted
 */
static int
oxnas_iic_algo_tryaddress(struct i2c_algo_oxnas_data *oxnas, int addr)
{
	volatile i2c_registers_oxnas_t *i2c = oxnas->i2c;
	unsigned long flags, length, tmo, temp, bytesTransfered;

	if (oxnas_debug) printk("oxnas_iic_algo_tryaddress(oxnas=%p/%p,addr=%d)\n", oxnas, i2c, addr);

	/* do a simple read */
	length = 2;
	
	{
		local_irq_save(flags);
		
		/* QUESTION: Does this get locked by the parent? it should be!! */
		oxnas->iTransferInProgress_ = 1;
		oxnas->iError_              = 0;
	
		// Set up the 7-bit slave address 
		i2c->SerialAddressRegister = I2C_SAR_WRITE_MASK & ((addr) << I2C_SAR_SEVEN_BIT_ADDRESS_BIT);
	
		// Setup the control register
		temp = ( I2C_SCR_WRITE      << I2C_SCR_READ_WRITE_BIT            ) |
                       ( I2C_SCR_NORMAL     << I2C_SCR_TRANSACTION_TYPE_BIT      ) |
                       ( I2C_SCR_SEVEN_BIT  << I2C_SCR_ADDRESS_MODE_BIT	         ) |
                       ( 0                  << I2C_SCR_SCCB_MODE_ENABLE_BIT      ) |
                       ( 1                  << I2C_SCR_SCCB_MODE_RESPECT_ACK_BIT ) |       
		       ( 1		    << I2C_SCR_AUTO_INCREMENT_ENABLE_BIT ) |
		       ( 1		    << I2C_SCR_ENABLE_SLAVE_HOLD_OFF_BIT ) |
		       ( 0 		    << I2C_SCR_HIGH_SPEED_DRIVE_BIT      ) |
                       ( length             << I2C_SCR_BYTES_TO_TRANSFER_BIT     );
		       
		temp &= I2C_SCR_WRITE_MASK;		       
		i2c->SerialControlRegister = temp;
		
		/* Enable some interupts */
		*( &(i2c->SerialInterruptEnableRegister) ) |= (1UL <<  I2C_IER_SERIAL_ENABLE_BIT);
		
		/* Begin transmission */
		*( &i2c->SerialControlRegister ) |= (1UL <<  I2C_SCR_TRANSACTION_PROGRESS_BIT);

		/* Begin transmission */
		
		/* Wait for IIC transfer */
		tmo = interruptible_sleep_on_timeout(&iic_wait,1*HZ);
		local_irq_restore(flags);
	}
	
	/* IDEA:  busy wait for small transfers, its faster  time_after(jiffies, tmo) */
	
	
	if (signal_pending(current) || !tmo){
		force_close(oxnas);
		if(oxnas_debug) 
			printk("$rIIC test_addr: timeout!\n");
		return -EIO;
	}
	
	if ( i2c->SerialControlRegister | (1UL <<  I2C_SCR_TRANSACTION_STATUS_BIT)) {
		if (oxnas_debug)
			printk("$rIIC test_addr; no ack\n");
		return -EREMOTEIO;
	}

	// How many bytes were read from the slave?
	bytesTransfered = (temp >> I2C_SCR_BYTES_TO_TRANSFER_BIT) & 
			  ~(0xffffffff << I2C_SCR_BYTES_TO_TRANSFER_NUM_BITS);
			  
	if (bytesTransfered > 2) {
		if (oxnas_debug)
			printk("$rIIC test_addr; Overrun\n");
		return -EREMOTEIO;;
	}

	if (oxnas_debug) printk("$rtest_addr %lu bytes\n", bytesTransfered);

	if (bytesTransfered < 2) {
		if (oxnas_debug)
			printk("$rIIC test_addr; short, wanted %d got %lu\n",
			       2, bytesTransfered);
		return 0;
	}
	printk("$GIIC found @ test_addr (oxnas=%p,addr=%d)\n", oxnas, addr);
	return 1;
}

static int oxnas_xfer(
	struct  i2c_adapter *adap,
	struct  i2c_msg msgs[], 
	int 	num)
{
	struct i2c_algo_oxnas_data *oxnas = adap->algo_data;
	struct i2c_msg 		   *pmsg;
	int i, ret;
	u_char addr;
    
	if (oxnas_debug > 1) printk("oxnas_xfer()\n");
	for (i = 0; i < num; i++) {
		pmsg = &msgs[i];

		if (oxnas_debug)
			printk("i2c-algo-oxnas.o: "
			       "#%d addr=0x%x flags=0x%x len=%d\n buf=%lx\n",
			       i, pmsg->addr, pmsg->flags, pmsg->len, (unsigned long)pmsg->buf);

		addr = pmsg->addr << 1;
		if (pmsg->flags & I2C_M_RD )
			addr |= 1;
		if (pmsg->flags & I2C_M_REV_DIR_ADDR )
			addr ^= 1;
    
		if (!(pmsg->flags & I2C_M_NOSTART)) {
		}
		if (pmsg->flags & I2C_M_RD ) {
			/* read bytes into buffer*/
			ret = oxnas_iic_algo_read(oxnas, addr, pmsg->buf, pmsg->len);
			if (oxnas_debug)
				printk("i2c-algo-oxnas.o: read %d bytes\n", ret);
			if (ret < pmsg->len ) {
				return (ret<0)? ret : -EREMOTEIO;
			}
		} else {
			/* write bytes from buffer */
			ret = oxnas_iic_algo_write(oxnas, addr, pmsg->buf, pmsg->len);
			if (oxnas_debug)
				printk("i2c-algo-oxnas.o: wrote %d\n", ret);
			if (ret < pmsg->len ) {
				return (ret<0) ? ret : -EREMOTEIO;
			}
		}
	}
	return (num);
}

static u32 oxnas_func(struct i2c_adapter *adap)
{
	if (oxnas_debug > 1) printk("oxnas_func(I2C_FUNC_SMBUS_EMUL | I2C_FUNC_10BIT_ADDR | I2C_FUNC_PROTOCOL_MANGLING)\n");
	
	return I2C_FUNC_SMBUS_EMUL | I2C_FUNC_10BIT_ADDR | 
	       I2C_FUNC_PROTOCOL_MANGLING; 
}

/* -----exported algorithm data: -------------------------------------	*/

static struct i2c_algorithm oxnas_algo = {
	.name		= "Oxnas algorithm",
	.id		= I2C_ALGO_OCP,
	.master_xfer	= oxnas_xfer,
	.functionality	= oxnas_func,
};

/* 
 * registering functions to load algorithms at runtime 
 */
int i2c_oxnas_algo_add_bus(struct i2c_adapter *adap)
{
	int i;
	struct i2c_algo_oxnas_data *oxnas = adap->algo_data;

	if (oxnas_debug) printk("i2c_oxnas_algo_add_bus: hw routines for %s registered.\n", adap->name);

	/* register new adapter to i2c module... */

	adap->id |= oxnas_algo.id;
	adap->algo = &oxnas_algo;

	oxnas_iic_algo_init(oxnas);
	i2c_add_adapter(adap);
	
	/* scan bus */
	if ( oxnas_scan ) {
		if (oxnas_debug) printk(KERN_INFO " i2c_oxnas_algo_add_bus: scanning bus %s...\n", adap->name);
		for (i = 0; i < 128; i++) {
			if (oxnas_debug) printk(KERN_INFO "   scanning addr %d...\n", i);
			if (oxnas_iic_algo_tryaddress(oxnas, i)) {
				printk("(%02x)",i<<1); 
			}
		}
		printk("\n");
	}
	
	return 0; 
}


int i2c_oxnas_algo_del_bus(struct i2c_adapter *adap)
{
	struct i2c_algo_oxnas_data *oxnas = adap->algo_data;

	oxnas_iic_algo_shutdown(oxnas);

	return i2c_del_adapter(adap);
}

EXPORT_SYMBOL(i2c_oxnas_algo_add_bus);
EXPORT_SYMBOL(i2c_oxnas_algo_del_bus);

MODULE_AUTHOR("Chris FOrd <....>");
MODULE_DESCRIPTION("I2C-Bus oxnas algorithm");
MODULE_LICENSE("GPL");

