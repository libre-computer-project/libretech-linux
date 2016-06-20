/*
 * /arch/=arm/mach-oxnas/dpe-test.c
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
 
/** 
 * Test driver for the cipher core
 *
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/sysdev.h>
#include <linux/fs.h>
#include <asm/arch/cipher.h>
#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <linux/dma-mapping.h>
#include <asm/arch/dma.h>

/***************************************************************************
* CONSTANTS
***************************************************************************/
#define DRIVER_AUTHOR "Oxford Semiconductor Inc."
#define DRIVER_DESC   "Cipher block testing"
MODULE_LICENSE("GPL");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);

// uses /dev/dv940led
#define DEVICE_NAME "ox800dpetst"
MODULE_SUPPORTED_DEVICE(DEVICE_NAME);

#define FAILED(reason) {printk(KERN_ERR"%s failed %s\n",__FUNCTION__,reason);++failed;}
/**************************************************************************
* PROTOTYPES
**************************************************************************/
#if 0
static u32 READL(int a) {u32 v = readl(a);printk("0x%08x <- [0x%08x]\n",v,a);return v;}
static void WRITEL(int v,int a){printk("0x%08x -> [0x%08x]\n",v,a);writel(v,a);}
#else
static u32 READL(int a) {u32 v = readl(a);return v;}
static void WRITEL(int v,int a){writel(v,a);}
#endif
/**************************************************************************
* STRUCTURES
**************************************************************************/
typedef int (ox800dpe_test_t)(void) ;

/**************************************************************************
* FUCTIONS
* prefix all with "ox800dpe"
**************************************************************************/

/*************************************************************************/
static int test1(void) {
    int failed = 0;
    u32 reg;

    // authenticate
    WRITEL(OX800IBW_STAT_AUTHENTICATED ,OX800IBW_STATUS);
    
    // toggle cleardown bit to start
    WRITEL(OX800DPE_CTL_ABORT  ,OX800DPE_CONTROL);
    WRITEL(0                   ,OX800DPE_CONTROL);
    
    
    // shouldn't be busy or full
    reg = READL( OX800DPE_STATUS );
    if (! (reg & OX800DPE_STAT_IDLE) )
        FAILED("not idle after abort toggle");
    if (reg & OX800DPE_STAT_TX_NOTEMPTY)
        FAILED("tx fifo not empty after abort toggle");
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx not empty after abort toggle");
    
    // setup for encryption 
    reg = OX800DPE_CTL_DIRECTION_ENC | OX800DPE_CTL_MODE_ECB_AES;
    WRITEL(reg  ,OX800DPE_CONTROL);

    reg = READL( OX800DPE_STATUS );
    // shouldn't be busy
    if (! (reg & OX800DPE_STAT_IDLE) )
        FAILED("not idle even with incomplete data");
    if (reg & OX800DPE_STAT_TX_NOTEMPTY)
        FAILED("tx not empty before data input ");
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx fifo filling without data");
    
    
    // key no 1
    WRITEL(be32_to_cpu( 0x00010203), OX800DPE_KEY00 );
    WRITEL(be32_to_cpu( 0x04050607), OX800DPE_KEY01 );
    WRITEL(be32_to_cpu( 0x08090a0b), OX800DPE_KEY02 );
    WRITEL(be32_to_cpu( 0x0c0d0e0f), OX800DPE_KEY03 );

    /* wait until done */
    while(  !(OX800DPE_STAT_IDLE & READL( OX800DPE_STATUS )) );
    
    // 1st data set 
    WRITEL(be32_to_cpu( 0x00112233), OX800DPE_DATA_IN0 );

    /* in fifo no longer empty */
    reg = READL( OX800DPE_STATUS );
    if ( (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx fifo still empty after data input")
    
    WRITEL(be32_to_cpu( 0x44556677), OX800DPE_DATA_IN1 );
    WRITEL(be32_to_cpu( 0x8899aabb), OX800DPE_DATA_IN2 );

    // shouldn't be busy as not enough data
    reg = READL( OX800DPE_STATUS );
    if (! (reg & OX800DPE_STAT_IDLE) )
        FAILED("core lept into action before putting in all the data");
    
    WRITEL(be32_to_cpu( 0xccddeeff), OX800DPE_DATA_IN3 );

    /* wait until done */
    while(  !(OX800DPE_STAT_IDLE & READL( OX800DPE_STATUS )) );
    
    reg = READL( OX800DPE_STATUS );

    /* output should be full */
    if ( !(reg & OX800DPE_STAT_TX_NOTEMPTY) )
        FAILED("tx still empty after encrypting data ");

    /* in empty */
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx still full after encrypting data ");
    
    /* check output */
    {
        u32 data[4];

        data[0] = READL( OX800DPE_DATA_OUT0 );
        data[1] = READL( OX800DPE_DATA_OUT1 );
        data[2] = READL( OX800DPE_DATA_OUT2 );
        data[3] = READL( OX800DPE_DATA_OUT3 );

        if ((data[0] != cpu_to_be32(0x69c4e0d8)) ||
            (data[1] != cpu_to_be32(0x6a7b0430)) ||
            (data[2] != cpu_to_be32(0xd8cdb780)) ||
            (data[3] != cpu_to_be32(0x70b4c55a)))
        {
            FAILED("encryption output incorrect");
            printk("%08x%08x%08x%08x\n",data[0],data[1],data[2],data[3]);
        }
    }
        
    /* output should be empty again */
    reg = READL( OX800DPE_STATUS );
    if ( (reg & OX800DPE_STAT_TX_NOTEMPTY) )
        FAILED("tx still full after data read");
    
    return failed;

}

/*************************************************************************/
static int test2(void) {
    int failed = 0;
    u32 reg;
    oxnas_dma_channel_t* dma_in;
    oxnas_dma_channel_t* dma_out;
    dma_addr_t in_address;
    dma_addr_t out_address;
    
    u32* data_in;
    u32* data_out;
    
    // setup dmas
    dma_in = oxnas_dma_request(1);
    dma_out = oxnas_dma_request(1);
    
    // get some dma accessable memory
    data_in = (u32*)kmalloc( 4 * sizeof(u32), GFP_KERNEL|GFP_DMA);
    data_out = (u32*)kmalloc( 4 * sizeof(u32), GFP_KERNEL|GFP_DMA);
    
    // fill with input and non expected output
    data_in[0] = be32_to_cpu( 0x00112233);
    data_in[1] = be32_to_cpu( 0x44556677);
    data_in[2] = be32_to_cpu( 0x8899aabb);
    data_in[3] = be32_to_cpu( 0xccddeeff);
    data_out[0] = ~0;
    data_out[1] = ~0;
    data_out[2] = ~0;
    data_out[3] = ~0;
    
    // map the dma regons
    in_address = dma_map_single(
        0,
        data_in,
        4 * sizeof(u32),
        DMA_TO_DEVICE);
        
    // map the dma regons
    out_address = dma_map_single(
        0,
        data_out,
        4 * sizeof(u32),
        DMA_FROM_DEVICE);
        
    // setup the transfers
    oxnas_dma_device_set(
        dma_in,
        OXNAS_DMA_TO_DEVICE,
        (char*)in_address,
        4 * sizeof(u32),
        &oxnas_dpe_rx_dma_settings,
        OXNAS_DMA_MODE_INC, 1);
        
    oxnas_dma_device_set(
        dma_out,
        OXNAS_DMA_FROM_DEVICE,
        (char*)out_address,
        4 * sizeof(u32),
        &oxnas_dpe_tx_dma_settings,
        OXNAS_DMA_MODE_INC, 1);

    // don't authenticate
    WRITEL(0 ,OX800IBW_STATUS);
    
    // toggle cleardown bit to start
    WRITEL(OX800DPE_CTL_ABORT  ,OX800DPE_CONTROL);
    WRITEL(0                   ,OX800DPE_CONTROL);
    
    
    // shouldn't be busy or full
    reg = READL( OX800DPE_STATUS );
    if (! (reg & OX800DPE_STAT_IDLE) )
        FAILED("not idle after abort toggle");
    if (reg & OX800DPE_STAT_TX_NOTEMPTY)
        FAILED("tx fifo not empty after abort toggle");
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx not empty after abort toggle");
    
    // setup for encryption 
    reg = OX800DPE_CTL_DIRECTION_ENC | OX800DPE_CTL_MODE_ECB_AES;
    WRITEL(reg  ,OX800DPE_CONTROL);

    reg = READL( OX800DPE_STATUS );
    // shouldn't be busy
    if (! (reg & OX800DPE_STAT_IDLE) )
        FAILED("not idle even with incomplete data");
    if (reg & OX800DPE_STAT_TX_NOTEMPTY)
        FAILED("tx not empty before data input ");
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx fifo filling without data");
    
    
    // key no 1
    WRITEL(be32_to_cpu( 0x00010203), OX800DPE_KEY00 );
    WRITEL(be32_to_cpu( 0x04050607), OX800DPE_KEY01 );
    WRITEL(be32_to_cpu( 0x08090a0b), OX800DPE_KEY02 );
    WRITEL(be32_to_cpu( 0x0c0d0e0f), OX800DPE_KEY03 );

    /* wait until done */
    while(  !(OX800DPE_STAT_IDLE & READL( OX800DPE_STATUS )) );
    
    // start dma transfers
    oxnas_dma_start(dma_out);
    oxnas_dma_start(dma_in);
    
    /* wait until done */
    while(  oxnas_dma_is_active( dma_out ) );
    
    reg = READL( OX800DPE_STATUS );

    /* output should be empty */
    if ( (reg & OX800DPE_STAT_TX_NOTEMPTY) )
        FAILED("tx still full after fetching ");

    /* in empty */
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx still full after encrypting data ");
    
    dma_unmap_single(0, in_address, 4 * sizeof(u32), DMA_TO_DEVICE);
    dma_unmap_single(0, out_address, 4 * sizeof(u32), DMA_FROM_DEVICE);

    /* check output */
    if ((data_out[0] != cpu_to_be32(0x69c4e0d8)) ||
        (data_out[1] != cpu_to_be32(0x6a7b0430)) ||
        (data_out[2] != cpu_to_be32(0xd8cdb780)) ||
        (data_out[3] != cpu_to_be32(0x70b4c55a)))
    {
        FAILED("encryption output incorrect");
        printk("%08x%08x%08x%08x\n",data_out[0],data_out[1],data_out[2],data_out[3]);
    }
        
    // free dmas
    oxnas_dma_free( dma_in ); 
    oxnas_dma_free( dma_out ); 
    
    kfree(data_in);
    kfree(data_out);

    return failed;

}


/*************************************************************************/
static int test3(void) {
    int failed = 0;
    u32 reg;
    oxnas_dma_channel_t* dma_in;
    oxnas_dma_channel_t* dma_out;
    dma_addr_t in_address;
    dma_addr_t out_address;
    
    u32* data_in;
    u32* data_out;
    
    // setup dmas
    dma_in = oxnas_dma_request(1);
    dma_out = oxnas_dma_request(1);
    
    // get some dma accessable memory
    data_in = (u32*)kmalloc( 4 * sizeof(u32), GFP_KERNEL|GFP_DMA);
    data_out = (u32*)kmalloc( 4 * sizeof(u32), GFP_KERNEL|GFP_DMA);
    
    // fill with input and non expected output
    data_in[0] = be32_to_cpu( 0x69c4e0d8);
    data_in[1] = be32_to_cpu( 0x6a7b0430);
    data_in[2] = be32_to_cpu( 0xd8cdb780);
    data_in[3] = be32_to_cpu( 0x70b4c55a);
    data_out[0] = ~0;
    data_out[1] = ~0;
    data_out[2] = ~0;
    data_out[3] = ~0;
    
    // map the dma regons
    in_address = dma_map_single(
        0,
        data_in,
        4 * sizeof(u32),
        DMA_TO_DEVICE);
        
    // map the dma regons
    out_address = dma_map_single(
        0,
        data_out,
        4 * sizeof(u32),
        DMA_FROM_DEVICE);
        
    // setup the transfers
    oxnas_dma_device_set(
        dma_in,
        OXNAS_DMA_TO_DEVICE,
        (char*)in_address,
        4 * sizeof(u32),
        &oxnas_dpe_rx_dma_settings,
        OXNAS_DMA_MODE_INC, 1);
        
    oxnas_dma_device_set(
        dma_out,
        OXNAS_DMA_FROM_DEVICE,
        (char*)out_address,
        4 * sizeof(u32),
        &oxnas_dpe_tx_dma_settings,
        OXNAS_DMA_MODE_INC, 1);

    // authenticate
    WRITEL(OX800IBW_STAT_AUTHENTICATED ,OX800IBW_STATUS);
    
    // toggle cleardown bit to start
    WRITEL(OX800DPE_CTL_ABORT  ,OX800DPE_CONTROL);
    WRITEL(0                   ,OX800DPE_CONTROL);
    
    
    // shouldn't be busy or full
    reg = READL( OX800DPE_STATUS );
    if (! (reg & OX800DPE_STAT_IDLE) )
        FAILED("not idle after abort toggle");
    if (reg & OX800DPE_STAT_TX_NOTEMPTY)
        FAILED("tx fifo not empty after abort toggle");
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx not empty after abort toggle");
    
    // setup for decryption 
    reg =  OX800DPE_CTL_MODE_ECB_AES;
    WRITEL(reg  ,OX800DPE_CONTROL);

    reg = READL( OX800DPE_STATUS );
    // shouldn't be busy
    if (! (reg & OX800DPE_STAT_IDLE) )
        FAILED("not idle even with incomplete data");
    if (reg & OX800DPE_STAT_TX_NOTEMPTY)
        FAILED("tx not empty before data input ");
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx fifo filling without data");
    
    
    // key no 1
    WRITEL(be32_to_cpu( 0x00010203), OX800DPE_KEY00 );
    WRITEL(be32_to_cpu( 0x04050607), OX800DPE_KEY01 );
    WRITEL(be32_to_cpu( 0x08090a0b), OX800DPE_KEY02 );
    WRITEL(be32_to_cpu( 0x0c0d0e0f), OX800DPE_KEY03 );

    /* wait until done */
    while(  !(OX800DPE_STAT_IDLE & READL( OX800DPE_STATUS )) );
    
    // start dma transfers
    oxnas_dma_start(dma_out);
    oxnas_dma_start(dma_in);
    
    /* wait until dma done */
    while(  oxnas_dma_is_active( dma_out ) );
    
    reg = READL( OX800DPE_STATUS );

    /* output should be empty */
    if ( (reg & OX800DPE_STAT_TX_NOTEMPTY) )
        FAILED("tx still full after fetching ");

    /* in empty */
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx still full after encrypting data ");
    
    dma_unmap_single(0, in_address, 4 * sizeof(u32), DMA_TO_DEVICE);
    dma_unmap_single(0, out_address, 4 * sizeof(u32), DMA_FROM_DEVICE);

    /* check output */
    if ((data_out[0] != cpu_to_be32(0x00112233)) ||
        (data_out[1] != cpu_to_be32(0x44556677)) ||
        (data_out[2] != cpu_to_be32(0x8899aabb)) ||
        (data_out[3] != cpu_to_be32(0xccddeeff)))
    {
        FAILED("encryption output incorrect");
        printk("%08x%08x%08x%08x\n",data_out[0],data_out[1],data_out[2],data_out[3]);
    }
    
    // free dmas
    oxnas_dma_free( dma_in ); 
    oxnas_dma_free( dma_out ); 
    
    kfree(data_in);
    kfree(data_out);

    return failed;

}

/*************************************************************************/
static int test4(void) {
    int failed = 0;
    u32 reg;
    oxnas_dma_channel_t* dma_out;
    dma_addr_t out_address;
    
    u32* data_out;
    
    // setup dmas
    dma_out = oxnas_dma_request(1);
    
    // get some dma accessable memory
    data_out = (u32*)kmalloc( 4 * sizeof(u32), GFP_KERNEL|GFP_DMA);
    
    // fill with non expected output
    data_out[0] = ~0;
    data_out[1] = ~0;
    data_out[2] = ~0;
    data_out[3] = ~0;
    
    // map the dma regons
    out_address = dma_map_single(
        0,
        data_out,
        4 * sizeof(u32),
        DMA_FROM_DEVICE);
        
    oxnas_dma_device_set(
        dma_out,
        OXNAS_DMA_FROM_DEVICE,
        (char*)out_address,
        4 * sizeof(u32),
        &oxnas_dpe_tx_dma_settings,
        OXNAS_DMA_MODE_INC, 1);

    // toggle cleardown bit to start
    WRITEL(OX800DPE_CTL_ABORT  ,OX800DPE_CONTROL);
    WRITEL(0                   ,OX800DPE_CONTROL);
    
    // don't authenticate
    WRITEL(0 ,OX800IBW_STATUS);
    
    
    // shouldn't be busy or full
    reg = READL( OX800DPE_STATUS );
    if (! (reg & OX800DPE_STAT_IDLE) )
        FAILED("not idle after abort toggle");
    if (reg & OX800DPE_STAT_TX_NOTEMPTY)
        FAILED("tx fifo not empty after abort toggle");
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx not empty after abort toggle");
    
    // setup for decryption 
    reg = OX800DPE_CTL_PRIMARY_IS_KEY3 | OX800DPE_CTL_MODE_ECB_AES;
    WRITEL(reg  ,OX800DPE_CONTROL);

    reg = READL( OX800DPE_STATUS );
    // shouldn't be busy
    if (! (reg & OX800DPE_STAT_IDLE) )
        FAILED("not idle even with incomplete data");
    if (reg & OX800DPE_STAT_TX_NOTEMPTY)
        FAILED("tx not empty before data input ");
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx fifo filling without data");
    
    
    // key no 1
    WRITEL(be32_to_cpu( 0x00010203), OX800DPE_KEY20 );
    WRITEL(be32_to_cpu( 0x04050607), OX800DPE_KEY21 );
    WRITEL(be32_to_cpu( 0x08090a0b), OX800DPE_KEY22 );
    WRITEL(be32_to_cpu( 0x0c0d0e0f), OX800DPE_KEY23 );

    /* wait until done */
    while(  !(OX800DPE_STAT_IDLE & READL( OX800DPE_STATUS )) );

    // start dma transfers
    oxnas_dma_start(dma_out);
    
    // 1st data set 
    WRITEL(be32_to_cpu( 0x69c4e0d8), OX800DPE_DATA_IN0 );
    WRITEL(be32_to_cpu( 0x6a7b0430), OX800DPE_DATA_IN1 );
    WRITEL(be32_to_cpu( 0xd8cdb780), OX800DPE_DATA_IN2 );
    WRITEL(be32_to_cpu( 0x70b4c55a), OX800DPE_DATA_IN3 );

    /* wait until done */
    while(  oxnas_dma_is_active( dma_out ) );
    
    reg = READL( OX800DPE_STATUS );

    /* output should be empty */
    if ( (reg & OX800DPE_STAT_TX_NOTEMPTY) )
        FAILED("tx still full after fetching ");

    /* in empty */
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx still full after encrypting data ");
    

    dma_unmap_single(0, out_address, 4 * sizeof(u32), DMA_FROM_DEVICE);

    /* check output */
    if ((data_out[0] != cpu_to_be32(0x00112233)) ||
        (data_out[1] != cpu_to_be32(0x44556677)) ||
        (data_out[2] != cpu_to_be32(0x8899aabb)) ||
        (data_out[3] != cpu_to_be32(0xccddeeff)))
    {
        FAILED("encryption output incorrect");
        printk("%08x%08x%08x%08x\n",data_out[0],data_out[1],data_out[2],data_out[3]);
    }
        
    // free dmas
    oxnas_dma_free( dma_out ); 
    kfree(data_out);
    
    return failed;

}

/*************************************************************************/
static int test5(void) {
    int failed = 0;
    u32 reg;
    oxnas_dma_channel_t* dma_in;
    dma_addr_t in_address;
    
    u32* data_in;
    
    // setup dmas
    dma_in = oxnas_dma_request(1);
    
    // get some dma accessable memory
    data_in = (u32*)kmalloc( 4 * sizeof(u32), GFP_KERNEL|GFP_DMA);
    
    // fill with input and non expected output
    data_in[0] = be32_to_cpu( 0x00112233);
    data_in[1] = be32_to_cpu( 0x44556677);
    data_in[2] = be32_to_cpu( 0x8899aabb);
    data_in[3] = be32_to_cpu( 0xccddeeff);
    
    // map the dma regons
    in_address = dma_map_single(
        0,
        data_in,
        4 * sizeof(u32),
        DMA_TO_DEVICE);
        
    // setup the transfers
    oxnas_dma_device_set(
        dma_in,
        OXNAS_DMA_TO_DEVICE,
        (char*)in_address,
        4 * sizeof(u32),
        &oxnas_dpe_rx_dma_settings,
        OXNAS_DMA_MODE_INC, 1);
        
    // toggle cleardown bit to start
    WRITEL(OX800DPE_CTL_ABORT  ,OX800DPE_CONTROL);
    WRITEL(0                   ,OX800DPE_CONTROL);
    
    // don't authenticate
    WRITEL(0 ,OX800IBW_STATUS);
    
    
    // shouldn't be busy or full
    reg = READL( OX800DPE_STATUS );
    if (! (reg & OX800DPE_STAT_IDLE) )
        FAILED("not idle after abort toggle");
    if (reg & OX800DPE_STAT_TX_NOTEMPTY)
        FAILED("tx fifo not empty after abort toggle");
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx not empty after abort toggle");
    
    // setup for encryption 
    reg = OX800DPE_CTL_DIRECTION_ENC |
          OX800DPE_CTL_MODE_ECB_AES  |
          OX800DPE_CTL_PRIMARY_IS_KEY3 ;
    WRITEL(reg  ,OX800DPE_CONTROL);

    reg = READL( OX800DPE_STATUS );
    // shouldn't be busy
    if (! (reg & OX800DPE_STAT_IDLE) )
        FAILED("not idle even with incomplete data");
    if (reg & OX800DPE_STAT_TX_NOTEMPTY)
        FAILED("tx not empty before data input ");
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx fifo filling without data");
    
    
    // key no 1
    WRITEL(be32_to_cpu( 0x00010203), OX800DPE_KEY20 );
    WRITEL(be32_to_cpu( 0x04050607), OX800DPE_KEY21 );
    WRITEL(be32_to_cpu( 0x08090a0b), OX800DPE_KEY22 );
    WRITEL(be32_to_cpu( 0x0c0d0e0f), OX800DPE_KEY23 );

    /* wait until done */
    while(  !(OX800DPE_STAT_IDLE & READL( OX800DPE_STATUS )) );
    
    // start dma transfers
    oxnas_dma_start(dma_in);
    
    /* wait until done */
    while(  oxnas_dma_is_active( dma_in ) );
    while(  !(OX800DPE_STAT_IDLE & READL( OX800DPE_STATUS )) );
    
    reg = READL( OX800DPE_STATUS );

    /* output should be full */
    if ( !(reg & OX800DPE_STAT_TX_NOTEMPTY) )
        FAILED("tx still empty after encrypting data ");

    /* in empty */
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx still full after encrypting data ");
    
    dma_unmap_single(0, in_address, 4 * sizeof(u32), DMA_TO_DEVICE);

    /* check output */
    {
        u32 data[4];

        data[0] = READL( OX800DPE_DATA_OUT0 );
        data[1] = READL( OX800DPE_DATA_OUT1 );
        data[2] = READL( OX800DPE_DATA_OUT2 );
        data[3] = READL( OX800DPE_DATA_OUT3 );

        if ((data[0] != cpu_to_be32(0x69c4e0d8)) ||
            (data[1] != cpu_to_be32(0x6a7b0430)) ||
            (data[2] != cpu_to_be32(0xd8cdb780)) ||
            (data[3] != cpu_to_be32(0x70b4c55a)))
        {
            FAILED("encryption output incorrect");
            printk("%08x%08x%08x%08x\n",data[0],data[1],data[2],data[3]);
        }
    }
        
    /* output should be empty again */
    reg = READL( OX800DPE_STATUS );
    if ( (reg & OX800DPE_STAT_TX_NOTEMPTY) )
        FAILED("tx still full after data read");
    
    // free dmas
    kfree(data_in);

    oxnas_dma_free( dma_in ); 
    
    return failed;

}

/*************************************************************************/
static int test6(void) {
    int failed = 0;
    u32 reg;

    // toggle cleardown bit to start
    WRITEL(OX800DPE_CTL_ABORT  ,OX800DPE_CONTROL);
    WRITEL(0                   ,OX800DPE_CONTROL);
    
    // don't authenticate
    WRITEL(0 ,OX800IBW_STATUS);
    
    // shouldn't be busy or full
    reg = READL( OX800DPE_STATUS );
    if (! (reg & OX800DPE_STAT_IDLE) )
        FAILED("not idle after abort toggle");
    if (reg & OX800DPE_STAT_TX_NOTEMPTY)
        FAILED("tx fifo not empty after abort toggle");
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx not empty after abort toggle");
    
    // setup for key encryption
    reg = OX800DPE_CTL_DIRECTION_ENC |
          OX800DPE_CTL_ENCRYPT_KEY |
          OX800DPE_CTL_MODE_ECB_AES;
    WRITEL(reg  ,OX800DPE_CONTROL);

    reg = READL( OX800DPE_STATUS );
    
    // shouldn't be busy
    if (! (reg & OX800DPE_STAT_IDLE) )
        FAILED("not idle even with incomplete data");
    if (reg & OX800DPE_STAT_TX_NOTEMPTY)
        FAILED("tx not empty before data input ");
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx fifo filling without data");

    // data to be encrypted to form a key    
    WRITEL(be32_to_cpu( 0x00010203), OX800DPE_DATA_IN0 );
    WRITEL(be32_to_cpu( 0x04050607), OX800DPE_DATA_IN1 );
    WRITEL(be32_to_cpu( 0x08090a0b), OX800DPE_DATA_IN2 );
    WRITEL(be32_to_cpu( 0x0c0d0e0f), OX800DPE_DATA_IN3 );

    /* wait until done */
    while(  !(OX800DPE_STAT_IDLE & READL( OX800DPE_STATUS )) );
    
    /* output should be empty */
    reg = READL( OX800DPE_STATUS );
    if ( (reg & OX800DPE_STAT_TX_NOTEMPTY) )
        FAILED("tx still full after fetching ");

    /* in empty */
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx still full after encrypting data ");
    
    {
        u32 data[4];

        data[0] = READL( OX800DPE_KEY00 );
        data[1] = READL( OX800DPE_KEY01 );
        data[2] = READL( OX800DPE_KEY02 );
        data[3] = READL( OX800DPE_KEY03 );
        
        if ((data[0] != cpu_to_be32(0x4791b833)) ||
            (data[1] != cpu_to_be32(0x7e2d8a69)) ||
            (data[2] != cpu_to_be32(0x290233f1)) ||
            (data[3] != cpu_to_be32(0xf3dff5a9)))
        {
            FAILED("encrypted key incorrect");
            printk("%08x%08x%08x%08x\n",data[0],data[1],data[2],data[3]);
        }
    }
        
    return failed;

}


/*************************************************************************/
static int test7(void) {
    int failed = 0;
    u32 reg;

    // toggle cleardown bit to start
    WRITEL(OX800DPE_CTL_ABORT  ,OX800DPE_CONTROL);
    WRITEL(0                   ,OX800DPE_CONTROL);
    
    // shouldn't be busy or full
    reg = READL( OX800DPE_STATUS );
    if (! (reg & OX800DPE_STAT_IDLE) )
        FAILED("not idle after abort toggle");
    if (reg & OX800DPE_STAT_TX_NOTEMPTY)
        FAILED("tx fifo not empty after abort toggle");
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx not empty after abort toggle");
    
    // authenticate
    WRITEL(OX800IBW_STAT_AUTHENTICATED ,OX800IBW_STATUS);
    
    // setup for key encryption
    reg = OX800DPE_CTL_DIRECTION_ENC |
          OX800DPE_CTL_ENCRYPT_KEY |
          OX800DPE_CTL_MODE_ECB_AES;
    WRITEL(reg  ,OX800DPE_CONTROL);

    reg = READL( OX800DPE_STATUS );
    
    // shouldn't be busy
    if (! (reg & OX800DPE_STAT_IDLE) )
        FAILED("not idle even with incomplete data");
    if (reg & OX800DPE_STAT_TX_NOTEMPTY)
        FAILED("tx not empty before data input ");
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx fifo filling without data");

    // data to be encrypted to form a key    
    WRITEL(be32_to_cpu( 0x00010203), OX800DPE_DATA_IN0 );
    WRITEL(be32_to_cpu( 0x04050607), OX800DPE_DATA_IN1 );
    WRITEL(be32_to_cpu( 0x08090a0b), OX800DPE_DATA_IN2 );
    WRITEL(be32_to_cpu( 0x0c0d0e0f), OX800DPE_DATA_IN3 );

    /* wait until done */
    while(  !(OX800DPE_STAT_IDLE & READL( OX800DPE_STATUS )) );
    
    /* output should be empty */
    reg = READL( OX800DPE_STATUS );
    if ( (reg & OX800DPE_STAT_TX_NOTEMPTY) )
        FAILED("tx still full after fetching ");

    /* in empty */
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx still full after encrypting data ");
    
    {
        u32 data[4];

        data[0] = READL( OX800DPE_KEY00 );
        data[1] = READL( OX800DPE_KEY01 );
        data[2] = READL( OX800DPE_KEY02 );
        data[3] = READL( OX800DPE_KEY03 );
        
        if ((data[0] != cpu_to_be32(0x4791b833)) ||
            (data[1] != cpu_to_be32(0x7e2d8a69)) ||
            (data[2] != cpu_to_be32(0x290233f1)) ||
            (data[3] != cpu_to_be32(0xf3dff5a9)))
        {
            FAILED("encrypted key incorrect");
            printk("%08x%08x%08x%08x\n",data[0],data[1],data[2],data[3]);
        }
    }
        
    return failed;

}



/**********************************************************************/
/* CBC tests                                                          */
/**********************************************************************/

static int test8(void) {
    int failed = 0;
    u32 reg;
    oxnas_dma_channel_t* dma_in;
    oxnas_dma_channel_t* dma_out;
    dma_addr_t in_address;
    dma_addr_t out_address;
    
    u32* data_in;
    u32* data_out;
    
    // setup dmas
    dma_in = oxnas_dma_request(1);
    dma_out = oxnas_dma_request(1);
    
    // get some dma accessable memory
    data_in = (u32*)kmalloc( 8 * sizeof(u32), GFP_KERNEL|GFP_DMA);
    data_out = (u32*)kmalloc( 8 * sizeof(u32), GFP_KERNEL|GFP_DMA);
    
    // fill with input and non expected output
    data_in[0] = be32_to_cpu( 0x00112233);
    data_in[1] = be32_to_cpu( 0x44556677);
    data_in[2] = be32_to_cpu( 0x8899aabb);
    data_in[3] = be32_to_cpu( 0xccddeeff);
    data_in[4] = be32_to_cpu( 0x00112233);
    data_in[5] = be32_to_cpu( 0x44556677);
    data_in[6] = be32_to_cpu( 0x8899aabb);
    data_in[7] = be32_to_cpu( 0xccddeeff);
    data_out[0] = ~0;
    data_out[1] = ~0;
    data_out[2] = ~0;
    data_out[3] = ~0;
    data_out[4] = ~0;
    data_out[5] = ~0;
    data_out[6] = ~0;
    data_out[7] = ~0;
    
    // map the dma regons
    in_address = dma_map_single(
        0,
        data_in,
        8 * sizeof(u32),
        DMA_TO_DEVICE);
        
    // map the dma regons
    out_address = dma_map_single(
        0,
        data_out,
        8 * sizeof(u32),
        DMA_FROM_DEVICE);
        
    // setup the transfers
    oxnas_dma_device_set(
        dma_in,
        OXNAS_DMA_TO_DEVICE,
        (char*)in_address,
        8 * sizeof(u32),
        &oxnas_dpe_rx_dma_settings,
        OXNAS_DMA_MODE_INC, 1);
        
    oxnas_dma_device_set(
        dma_out,
        OXNAS_DMA_FROM_DEVICE,
        (char*)out_address,
        8 * sizeof(u32),
        &oxnas_dpe_tx_dma_settings,
        OXNAS_DMA_MODE_INC, 1);

    // toggle cleardown bit to start
    WRITEL(OX800DPE_CTL_ABORT  ,OX800DPE_CONTROL);
    WRITEL(0                   ,OX800DPE_CONTROL);
    
    // don't authenticate
    WRITEL(0 ,OX800IBW_STATUS);
    
    
    // shouldn't be busy or full
    reg = READL( OX800DPE_STATUS );
    if (! (reg & OX800DPE_STAT_IDLE) )
        FAILED("not idle after abort toggle");
    if (reg & OX800DPE_STAT_TX_NOTEMPTY)
        FAILED("tx fifo not empty after abort toggle");
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx not empty after abort toggle");
    
    // setup for encryption 
    reg = OX800DPE_CTL_DIRECTION_ENC |
          OX800DPE_CTL_MODE_CBC_AES;
    WRITEL(reg  ,OX800DPE_CONTROL);

    reg = READL( OX800DPE_STATUS );
    // shouldn't be busy
    if (! (reg & OX800DPE_STAT_IDLE) )
        FAILED("not idle even with incomplete data");
    if (reg & OX800DPE_STAT_TX_NOTEMPTY)
        FAILED("tx not empty before data input ");
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx fifo filling without data");


    // key no 1
    WRITEL(be32_to_cpu( 0x00010203), OX800DPE_KEY00 );
    WRITEL(be32_to_cpu( 0x04050607), OX800DPE_KEY01 );
    WRITEL(be32_to_cpu( 0x08090a0b), OX800DPE_KEY02 );
    WRITEL(be32_to_cpu( 0x0c0d0e0f), OX800DPE_KEY03 );

    /* wait until done */
    while(  !(OX800DPE_STAT_IDLE & READL( OX800DPE_STATUS )) );

    // setup initialisation vector
    WRITEL(be32_to_cpu(0), OX800DPE_DATA_CBC0 );
    WRITEL(be32_to_cpu(0), OX800DPE_DATA_CBC1 );

    // start dma transfers
    oxnas_dma_start(dma_out);
    oxnas_dma_start(dma_in);
    
    /* wait until dma done */
    while(  oxnas_dma_is_active( dma_out ) );
    
    reg = READL( OX800DPE_STATUS );

    /* output should be empty */
    if ( (reg & OX800DPE_STAT_TX_NOTEMPTY) )
        FAILED("tx still full after fetching ");
    /* in empty */
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx still full after encrypting data ");
    
    dma_unmap_single(0, in_address, 8 * sizeof(u32), DMA_TO_DEVICE);
    dma_unmap_single(0, out_address, 8 * sizeof(u32), DMA_FROM_DEVICE);

    /* check output */
    if ((data_out[0] != cpu_to_be32(0x69c4e0d8)) ||
        (data_out[1] != cpu_to_be32(0x6a7b0430)) ||
        (data_out[2] != cpu_to_be32(0xd8cdb780)) ||
        (data_out[3] != cpu_to_be32(0x70b4c55a)) ||

        (data_out[4] != cpu_to_be32(0x7d7786be)) ||
        (data_out[5] != cpu_to_be32(0x32d059a6)) ||
        (data_out[6] != cpu_to_be32(0x0ca8021a)) ||
        (data_out[7] != cpu_to_be32(0x65dd9f09)))
    {
        FAILED("encryption output incorrect");
        printk("%08x%08x%08x%08x\n",data_out[0],data_out[1],data_out[2],data_out[3]);
        printk("%08x%08x%08x%08x\n",data_out[4],data_out[5],data_out[6],data_out[7]);
    }
        
    // free dmas
    oxnas_dma_free( dma_in ); 
    oxnas_dma_free( dma_out ); 
    
    kfree(data_in);
    kfree(data_out);

    return failed;

}


/*************************************************************************/
static int test9(void) {
    int failed = 0;
    u32 reg;
    oxnas_dma_channel_t* dma_in;
    oxnas_dma_channel_t* dma_out;
    dma_addr_t in_address;
    dma_addr_t out_address;
    
    u32* data_in;
    u32* data_out;
    
    // setup dmas
    dma_in = oxnas_dma_request(1);
    dma_out = oxnas_dma_request(1);
    
    // get some dma accessable memory
    data_in = (u32*)kmalloc( 8 * sizeof(u32), GFP_KERNEL|GFP_DMA);
    data_out = (u32*)kmalloc( 8 * sizeof(u32), GFP_KERNEL|GFP_DMA);
    
    // fill with input and non expected output
    data_in[0] = be32_to_cpu( 0x69c4e0d8);
    data_in[1] = be32_to_cpu( 0x6a7b0430);
    data_in[2] = be32_to_cpu( 0xd8cdb780);
    data_in[3] = be32_to_cpu( 0x70b4c55a);                              
    data_in[4] = be32_to_cpu( 0x7d7786be);
    data_in[5] = be32_to_cpu( 0x32d059a6);
    data_in[6] = be32_to_cpu( 0x0ca8021a);
    data_in[7] = be32_to_cpu( 0x65dd9f09);
    data_out[0] = ~0;
    data_out[1] = ~0;
    data_out[2] = ~0;
    data_out[3] = ~0;
    data_out[4] = ~0;
    data_out[5] = ~0;
    data_out[6] = ~0;
    data_out[7] = ~0;
    
    // map the dma regons
    in_address = dma_map_single(
        0,
        data_in,
        8 * sizeof(u32),
        DMA_TO_DEVICE);
        
    // map the dma regons
    out_address = dma_map_single(
        0,
        data_out,
        8 * sizeof(u32),
        DMA_FROM_DEVICE);
        
    // setup the transfers
    oxnas_dma_device_set(
        dma_in,
        OXNAS_DMA_TO_DEVICE,
        (char*)in_address,
        8 * sizeof(u32),
        &oxnas_dpe_rx_dma_settings,
        OXNAS_DMA_MODE_INC, 1);
        
    oxnas_dma_device_set(
        dma_out,
        OXNAS_DMA_FROM_DEVICE,
        (char*)out_address,
        8 * sizeof(u32),
        &oxnas_dpe_tx_dma_settings,
        OXNAS_DMA_MODE_INC, 1);

    // authenticate
    WRITEL(OX800IBW_STAT_AUTHENTICATED ,OX800IBW_STATUS);
    
    // toggle cleardown bit to start
    WRITEL(OX800DPE_CTL_ABORT  ,OX800DPE_CONTROL);
    WRITEL(0                   ,OX800DPE_CONTROL);
    
    
    // shouldn't be busy or full
    reg = READL( OX800DPE_STATUS );
    if (! (reg & OX800DPE_STAT_IDLE) )
        FAILED("not idle after abort toggle");
    if (reg & OX800DPE_STAT_TX_NOTEMPTY)
        FAILED("tx fifo not empty after abort toggle");
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx not empty after abort toggle");
    
    // setup for decryption 
    reg = OX800DPE_CTL_MODE_CBC_AES;
    WRITEL(reg  ,OX800DPE_CONTROL);

    reg = READL( OX800DPE_STATUS );
    // shouldn't be busy
    if (! (reg & OX800DPE_STAT_IDLE) )
        FAILED("not idle even with incomplete data");
    if (reg & OX800DPE_STAT_TX_NOTEMPTY)
        FAILED("tx not empty before data input ");
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx fifo filling without data");
    
    
    // key no 1
    WRITEL(be32_to_cpu( 0x00010203), OX800DPE_KEY00 );
    WRITEL(be32_to_cpu( 0x04050607), OX800DPE_KEY01 );
    WRITEL(be32_to_cpu( 0x08090a0b), OX800DPE_KEY02 );
    WRITEL(be32_to_cpu( 0x0c0d0e0f), OX800DPE_KEY03 );

    /* wait until done */
    while(  !(OX800DPE_STAT_IDLE & READL( OX800DPE_STATUS )) );
    
    // setup initialisation vector
    WRITEL(be32_to_cpu(0), OX800DPE_DATA_CBC0 );
    WRITEL(be32_to_cpu(0), OX800DPE_DATA_CBC1 );

    // start dma transfers
    oxnas_dma_start(dma_out);
    oxnas_dma_start(dma_in);
    
    /* wait until done */
    while(  oxnas_dma_is_active( dma_out ) );
    
    reg = READL( OX800DPE_STATUS );

    /* output should be empty */
    if ( (reg & OX800DPE_STAT_TX_NOTEMPTY) )
        FAILED("tx still full after fetching ");

    /* in empty */
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx still full after encrypting data ");
    
    dma_unmap_single(0, in_address, 8 * sizeof(u32), DMA_TO_DEVICE);
    dma_unmap_single(0, out_address, 8 * sizeof(u32), DMA_FROM_DEVICE);

    /* check output */
    if ((data_out[0] != cpu_to_be32(0x00112233)) ||
        (data_out[1] != cpu_to_be32(0x44556677)) ||
        (data_out[2] != cpu_to_be32(0x8899aabb)) ||
        (data_out[3] != cpu_to_be32(0xccddeeff)) ||
        (data_out[4] != cpu_to_be32(0x00112233)) ||
        (data_out[5] != cpu_to_be32(0x44556677)) ||
        (data_out[6] != cpu_to_be32(0x8899aabb)) ||
        (data_out[7] != cpu_to_be32(0xccddeeff)))
    {
        FAILED("encryption output incorrect");
        printk("%08x%08x%08x%08x\n",data_out[0],data_out[1],data_out[2],data_out[3]);
        printk("%08x%08x%08x%08x\n",data_out[4],data_out[5],data_out[6],data_out[7]);
    }
        
    // free dmas
    oxnas_dma_free( dma_in ); 
    oxnas_dma_free( dma_out ); 
    
    kfree(data_in);
    kfree(data_out);

    return failed;

}

/*************************************************************************/
static int test10(void) {
    int failed = 0;
    u32 reg;
    oxnas_dma_channel_t* dma_in;
    oxnas_dma_channel_t* dma_out;
    dma_addr_t in_address;
    dma_addr_t out_address;
    
    u32* data_in;
    u32* data_out;
    
    // setup dmas
    dma_in = oxnas_dma_request(1);
    dma_out = oxnas_dma_request(1);
    
    // get some dma accessable memory
    data_in = (u32*)kmalloc( 4 * sizeof(u32), GFP_KERNEL|GFP_DMA);
    data_out = (u32*)kmalloc( 4 * sizeof(u32), GFP_KERNEL|GFP_DMA);
    
    // fill with input and non expected output
    data_in[0] = be32_to_cpu( 0x00112233);
    data_in[1] = be32_to_cpu( 0x44556677);
    data_in[2] = be32_to_cpu( 0x8899aabb);
    data_in[3] = be32_to_cpu( 0xccddeeff);
    data_out[0] = ~0;
    data_out[1] = ~0;
    data_out[2] = ~0;
    data_out[3] = ~0;
    
    // map the dma regons
    in_address = dma_map_single(
        0,
        data_in,
        4 * sizeof(u32),
        DMA_TO_DEVICE);
        
    // map the dma regons
    out_address = dma_map_single(
        0,
        data_out,
        4 * sizeof(u32),
        DMA_FROM_DEVICE);
        
    // setup the transfers
    oxnas_dma_device_set(
        dma_in,
        OXNAS_DMA_TO_DEVICE,
        (char*)in_address,
        4 * sizeof(u32),
        &oxnas_dpe_rx_dma_settings,
        OXNAS_DMA_MODE_INC, 1);
        
    oxnas_dma_device_set(
        dma_out,
        OXNAS_DMA_FROM_DEVICE,
        (char*)out_address,
        4 * sizeof(u32),
        &oxnas_dpe_tx_dma_settings,
        OXNAS_DMA_MODE_INC, 1);

    // don't authenticate
    WRITEL(0 ,OX800IBW_STATUS);
    
    // toggle cleardown bit to start
    WRITEL(OX800DPE_CTL_ABORT  ,OX800DPE_CONTROL);
    WRITEL(0                   ,OX800DPE_CONTROL);
    
    
    // shouldn't be busy or full
    reg = READL( OX800DPE_STATUS );
    if (! (reg & OX800DPE_STAT_IDLE) )
        FAILED("not idle after abort toggle");
    if (reg & OX800DPE_STAT_TX_NOTEMPTY)
        FAILED("tx fifo not empty after abort toggle");
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx not empty after abort toggle");
    
    // setup for encryption 
    reg = OX800DPE_CTL_DIRECTION_ENC |
          OX800DPE_CTL_MODE_CBC_AES;
    WRITEL(reg  ,OX800DPE_CONTROL);

    reg = READL( OX800DPE_STATUS );
    // shouldn't be busy
    if (! (reg & OX800DPE_STAT_IDLE) )
        FAILED("not idle even with incomplete data");
    if (reg & OX800DPE_STAT_TX_NOTEMPTY)
        FAILED("tx not empty before data input ");
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx fifo filling without data");


    // key no 1
    WRITEL(be32_to_cpu( 0x00010203), OX800DPE_KEY00 );
    WRITEL(be32_to_cpu( 0x04050607), OX800DPE_KEY01 );
    WRITEL(be32_to_cpu( 0x08090a0b), OX800DPE_KEY02 );
    WRITEL(be32_to_cpu( 0x0c0d0e0f), OX800DPE_KEY03 );

    /* wait until done */
    while(  !(OX800DPE_STAT_IDLE & READL( OX800DPE_STATUS )) );

    // setup initialisation vector
    WRITEL(be32_to_cpu(~0), OX800DPE_DATA_CBC0 );
    WRITEL(be32_to_cpu(~0), OX800DPE_DATA_CBC1 );

    // start dma transfers
    oxnas_dma_start(dma_out);
    oxnas_dma_start(dma_in);

    /* wait until done */
    while(  oxnas_dma_is_active( dma_out ) );
    
    reg = READL( OX800DPE_STATUS );

    /* output should be empty */
    if ( (reg & OX800DPE_STAT_TX_NOTEMPTY) )
        FAILED("tx still full after fetching ");

    /* in empty */
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx still full after encrypting data ");
    
    dma_unmap_single(0, in_address, 4 * sizeof(u32), DMA_TO_DEVICE);
    dma_unmap_single(0, out_address, 4 * sizeof(u32), DMA_FROM_DEVICE);

    /* check output */
    if ((data_out[0] != cpu_to_be32(0x0bde5b88)) ||
        (data_out[1] != cpu_to_be32(0x114ac430)) ||
        (data_out[2] != cpu_to_be32(0x134e99ee)) ||
        (data_out[3] != cpu_to_be32(0xd3557046)))
    {
        FAILED("encryption output incorrect");
        printk("%08x%08x%08x%08x\n",data_out[0],data_out[1],data_out[2],data_out[3]);
    }
        
    // free dmas
    oxnas_dma_free( dma_in ); 
    oxnas_dma_free( dma_out ); 
    
    kfree(data_in);
    kfree(data_out);

    return failed;

}


/*************************************************************************/
static int test11(void) {
    int failed = 0;
    u32 reg;
    oxnas_dma_channel_t* dma_in;
    oxnas_dma_channel_t* dma_out;
    dma_addr_t in_address;
    dma_addr_t out_address;
    
    u32* data_in;
    u32* data_out;
    
    // setup dmas
    dma_in = oxnas_dma_request(1);
    dma_out = oxnas_dma_request(1);
    
    // get some dma accessable memory
    data_in = (u32*)kmalloc( 4 * sizeof(u32), GFP_KERNEL|GFP_DMA);
    data_out = (u32*)kmalloc( 4 * sizeof(u32), GFP_KERNEL|GFP_DMA);
    
    // fill with input and non expected output
    data_in[0] = be32_to_cpu( 0x69c4e0d8);
    data_in[1] = be32_to_cpu( 0x6a7b0430);
    data_in[2] = be32_to_cpu( 0xd8cdb780);
    data_in[3] = be32_to_cpu( 0x70b4c55a);
    data_out[0] = ~0;
    data_out[1] = ~0;
    data_out[2] = ~0;
    data_out[3] = ~0;
    
    // map the dma regons
    in_address = dma_map_single(
        0,
        data_in,
        4 * sizeof(u32),
        DMA_TO_DEVICE);
        
    // map the dma regons
    out_address = dma_map_single(
        0,
        data_out,
        4 * sizeof(u32),
        DMA_FROM_DEVICE);
        
    // setup the transfers
    oxnas_dma_device_set(
        dma_in,
        OXNAS_DMA_TO_DEVICE,
        (char*)in_address,
        4 * sizeof(u32),
        &oxnas_dpe_rx_dma_settings,
        OXNAS_DMA_MODE_INC, 1);
        
    oxnas_dma_device_set(
        dma_out,
        OXNAS_DMA_FROM_DEVICE,
        (char*)out_address,
        4 * sizeof(u32),
        &oxnas_dpe_tx_dma_settings,
        OXNAS_DMA_MODE_INC, 1);

    // authenticate
    WRITEL(OX800IBW_STAT_AUTHENTICATED ,OX800IBW_STATUS);
    
    // toggle cleardown bit to start
    WRITEL(OX800DPE_CTL_ABORT  ,OX800DPE_CONTROL);
    WRITEL(0                   ,OX800DPE_CONTROL);
    
    
    // shouldn't be busy or full
    reg = READL( OX800DPE_STATUS );
    if (! (reg & OX800DPE_STAT_IDLE) )
        FAILED("not idle after abort toggle");
    if (reg & OX800DPE_STAT_TX_NOTEMPTY)
        FAILED("tx fifo not empty after abort toggle");
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx not empty after abort toggle");
    
    // setup for decryption 
    reg = OX800DPE_CTL_MODE_CBC_AES;
    WRITEL(reg  ,OX800DPE_CONTROL);

    reg = READL( OX800DPE_STATUS );
    // shouldn't be busy
    if (! (reg & OX800DPE_STAT_IDLE) )
        FAILED("not idle even with incomplete data");
    if (reg & OX800DPE_STAT_TX_NOTEMPTY)
        FAILED("tx not empty before data input ");
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx fifo filling without data");
    
    
    // key no 1
    WRITEL(be32_to_cpu( 0x00010203), OX800DPE_KEY00 );
    WRITEL(be32_to_cpu( 0x04050607), OX800DPE_KEY01 );
    WRITEL(be32_to_cpu( 0x08090a0b), OX800DPE_KEY02 );
    WRITEL(be32_to_cpu( 0x0c0d0e0f), OX800DPE_KEY03 );

    /* wait until done */
    while(  !(OX800DPE_STAT_IDLE & READL( OX800DPE_STATUS )) );
    
    // setup initialisation vector
    WRITEL(be32_to_cpu(~0), OX800DPE_DATA_CBC0 );
    WRITEL(be32_to_cpu(~0), OX800DPE_DATA_CBC1 );

    // start dma transfers
    oxnas_dma_start(dma_out);
    oxnas_dma_start(dma_in);
    
    /* wait until done */
    while(  oxnas_dma_is_active( dma_out ) );
    
    reg = READL( OX800DPE_STATUS );

    /* output should be empty */
    if ( (reg & OX800DPE_STAT_TX_NOTEMPTY) )
        FAILED("tx still full after fetching ");

    /* in empty */
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx still full after encrypting data ");
    
    dma_unmap_single(0, in_address, 4 * sizeof(u32), DMA_TO_DEVICE);
    dma_unmap_single(0, out_address, 4 * sizeof(u32), DMA_FROM_DEVICE);

    /* check output */
    if ((data_out[0] != cpu_to_be32(0x00112233)) ||
        (data_out[1] != cpu_to_be32(0x44556677)) ||
        (data_out[2] != cpu_to_be32(0x8899aab4)) ||
        (data_out[3] != cpu_to_be32(0x33221100)))
    {
        FAILED("encryption output incorrect");
        printk("%08x%08x%08x%08x\n",data_out[0],data_out[1],data_out[2],data_out[3]);
    }
        
    // free dmas
    oxnas_dma_free( dma_in ); 
    oxnas_dma_free( dma_out ); 
    
    kfree(data_in);
    kfree(data_out);

    return failed;

}

/*************************************************************************/
static int test12(void) {
    int failed = 0;
    u32 reg;
    oxnas_dma_channel_t* dma_in;
    oxnas_dma_channel_t* dma_out;
    dma_addr_t in_address;
    dma_addr_t out_address;
    
    u32* data_in;
    u32* data_out;
    
    // setup dmas
    dma_in = oxnas_dma_request(1);
    dma_out = oxnas_dma_request(1);
    
    // get some dma accessable memory
    data_in = (u32*)kmalloc( 4 * sizeof(u32), GFP_KERNEL|GFP_DMA);
    data_out = (u32*)kmalloc( 4 * sizeof(u32), GFP_KERNEL|GFP_DMA);
    
    // fill with input and non expected output
    data_in[0] = be32_to_cpu( 0x00112233);
    data_in[1] = be32_to_cpu( 0x44556677);
    data_in[2] = be32_to_cpu( 0x8899aabb);
    data_in[3] = be32_to_cpu( 0xccddeeff);
    data_out[0] = ~0;
    data_out[1] = ~0;
    data_out[2] = ~0;
    data_out[3] = ~0;
    
    // map the dma regons
    in_address = dma_map_single(
        0,
        data_in,
        4 * sizeof(u32),
        DMA_TO_DEVICE);
        
    // map the dma regons
    out_address = dma_map_single(
        0,
        data_out,
        4 * sizeof(u32),
        DMA_FROM_DEVICE);
        
    // setup the transfers
    oxnas_dma_device_set(
        dma_in,
        OXNAS_DMA_TO_DEVICE,
        (char*)in_address,
        4 * sizeof(u32),
        &oxnas_dpe_rx_dma_settings,
        OXNAS_DMA_MODE_INC, 1);
        
    oxnas_dma_device_set(
        dma_out,
        OXNAS_DMA_FROM_DEVICE,
        (char*)out_address,
        4 * sizeof(u32),
        &oxnas_dpe_tx_dma_settings,
        OXNAS_DMA_MODE_INC, 1);

    // don't authenticate
    WRITEL(0 ,OX800IBW_STATUS);
    
    // toggle cleardown bit to start
    WRITEL(OX800DPE_CTL_ABORT  ,OX800DPE_CONTROL);
    WRITEL(0                   ,OX800DPE_CONTROL);
    
    
    // shouldn't be busy or full
    reg = READL( OX800DPE_STATUS );
    if (! (reg & OX800DPE_STAT_IDLE) )
        FAILED("not idle after abort toggle");
    if (reg & OX800DPE_STAT_TX_NOTEMPTY)
        FAILED("tx fifo not empty after abort toggle");
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx not empty after abort toggle");
    
    // setup for encryption 
    reg = OX800DPE_CTL_DIRECTION_ENC |
          OX800DPE_CTL_MODE_CBC_AES;
    WRITEL(reg  ,OX800DPE_CONTROL);

    reg = READL( OX800DPE_STATUS );
    // shouldn't be busy
    if (! (reg & OX800DPE_STAT_IDLE) )
        FAILED("not idle even with incomplete data");
    if (reg & OX800DPE_STAT_TX_NOTEMPTY)
        FAILED("tx not empty before data input ");
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx fifo filling without data");


    // key no 1
    WRITEL(be32_to_cpu( 0x00010203), OX800DPE_KEY00 );
    WRITEL(be32_to_cpu( 0x04050607), OX800DPE_KEY01 );
    WRITEL(be32_to_cpu( 0x08090a0b), OX800DPE_KEY02 );
    WRITEL(be32_to_cpu( 0x0c0d0e0f), OX800DPE_KEY03 );

    /* wait until done */
    while(  !(OX800DPE_STAT_IDLE & READL( OX800DPE_STATUS )) );

    // setup initialisation vector
    WRITEL(be32_to_cpu(1), OX800DPE_DATA_CBC0 );
    WRITEL(be32_to_cpu(0), OX800DPE_DATA_CBC1 );

    // start dma transfers
    oxnas_dma_start(dma_out);
    oxnas_dma_start(dma_in);
    
    /* wait until done */
    while(  oxnas_dma_is_active( dma_out ) );
    
    reg = READL( OX800DPE_STATUS );

    /* output should be empty */
    if ( (reg & OX800DPE_STAT_TX_NOTEMPTY) )
        FAILED("tx still full after fetching ");

    /* in empty */
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx still full after encrypting data ");
    
    dma_unmap_single(0, in_address, 4 * sizeof(u32), DMA_TO_DEVICE);
    dma_unmap_single(0, out_address, 4 * sizeof(u32), DMA_FROM_DEVICE);

    
    
    
    
    /* check output */
    if ((data_out[0] != 0x850d2506) ||
        (data_out[1] != 0xd71056c7) ||
        (data_out[2] != 0xe62c220f) ||
        (data_out[3] != 0xc3dedaf9))
    {
        FAILED("encryption output incorrect");
        printk("%08x%08x%08x%08x\n",data_out[0],data_out[1],data_out[2],data_out[3]);
    }
        
    // free dmas
    oxnas_dma_free( dma_in ); 
    oxnas_dma_free( dma_out ); 
    
    kfree(data_in);
    kfree(data_out);

    return failed;

}


/*************************************************************************/
static int test13(void) {
    int failed = 0;
    u32 reg;
    oxnas_dma_channel_t* dma_in;
    oxnas_dma_channel_t* dma_out;
    dma_addr_t in_address;
    dma_addr_t out_address;
    
    u32* data_in;
    u32* data_out;
    
    // setup dmas
    dma_in = oxnas_dma_request(1);
    dma_out = oxnas_dma_request(1);
    
    // get some dma accessable memory
    data_in = (u32*)kmalloc( 4 * sizeof(u32), GFP_KERNEL|GFP_DMA);
    data_out = (u32*)kmalloc( 4 * sizeof(u32), GFP_KERNEL|GFP_DMA);
    
    // fill with input and non expected output
    data_in[0] = 0x850d2506;
    data_in[1] = 0xd71056c7;
    data_in[2] = 0xe62c220f;
    data_in[3] = 0xc3dedaf9;
    data_out[0] = ~0;
    data_out[1] = ~0;
    data_out[2] = ~0;
    data_out[3] = ~0;
    
    // map the dma regons
    in_address = dma_map_single(
        0,
        data_in,
        4 * sizeof(u32),
        DMA_TO_DEVICE);
        
    // map the dma regons
    out_address = dma_map_single(
        0,
        data_out,
        4 * sizeof(u32),
        DMA_FROM_DEVICE);
        
    // setup the transfers
    oxnas_dma_device_set(
        dma_in,
        OXNAS_DMA_TO_DEVICE,
        (char*)in_address,
        4 * sizeof(u32),
        &oxnas_dpe_rx_dma_settings,
        OXNAS_DMA_MODE_INC, 1);
        
    oxnas_dma_device_set(
        dma_out,
        OXNAS_DMA_FROM_DEVICE,
        (char*)out_address,
        4 * sizeof(u32),
        &oxnas_dpe_tx_dma_settings,
        OXNAS_DMA_MODE_INC, 1);

    // authenticate
    WRITEL(OX800IBW_STAT_AUTHENTICATED ,OX800IBW_STATUS);
    
    // toggle cleardown bit to start
    WRITEL(OX800DPE_CTL_ABORT  ,OX800DPE_CONTROL);
    WRITEL(0                   ,OX800DPE_CONTROL);
    
    
    // shouldn't be busy or full
    reg = READL( OX800DPE_STATUS );
    if (! (reg & OX800DPE_STAT_IDLE) )
        FAILED("not idle after abort toggle");
    if (reg & OX800DPE_STAT_TX_NOTEMPTY)
        FAILED("tx fifo not empty after abort toggle");
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx not empty after abort toggle");
    
    // setup for decryption 
    reg = OX800DPE_CTL_MODE_CBC_AES;
    WRITEL(reg  ,OX800DPE_CONTROL);

    reg = READL( OX800DPE_STATUS );
    // shouldn't be busy
    if (! (reg & OX800DPE_STAT_IDLE) )
        FAILED("not idle even with incomplete data");
    if (reg & OX800DPE_STAT_TX_NOTEMPTY)
        FAILED("tx not empty before data input ");
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx fifo filling without data");
    
    
    // key no 1
    WRITEL(be32_to_cpu( 0x00010203), OX800DPE_KEY00 );
    WRITEL(be32_to_cpu( 0x04050607), OX800DPE_KEY01 );
    WRITEL(be32_to_cpu( 0x08090a0b), OX800DPE_KEY02 );
    WRITEL(be32_to_cpu( 0x0c0d0e0f), OX800DPE_KEY03 );

    /* wait until done */
    while(  !(OX800DPE_STAT_IDLE & READL( OX800DPE_STATUS )) );
    
    // setup initialisation vector
    WRITEL(be32_to_cpu(1), OX800DPE_DATA_CBC0 );
    WRITEL(be32_to_cpu(0), OX800DPE_DATA_CBC1 );

    // start dma transfers
    oxnas_dma_start(dma_out);
    oxnas_dma_start(dma_in);
    
    /* wait until done */
    while(  oxnas_dma_is_active( dma_out ) );
    
    reg = READL( OX800DPE_STATUS );

    /* output should be empty */
    if ( (reg & OX800DPE_STAT_TX_NOTEMPTY) )
        FAILED("tx still full after fetching ");

    /* in empty */
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx still full after encrypting data ");
    
    dma_unmap_single(0, in_address, 4 * sizeof(u32), DMA_TO_DEVICE);
    dma_unmap_single(0, out_address, 4 * sizeof(u32), DMA_FROM_DEVICE);

    /* check output */
    if ((data_out[0] != cpu_to_be32(0x00112233)) ||
        (data_out[1] != cpu_to_be32(0x44556677)) ||
        (data_out[2] != cpu_to_be32(0x8899aabb)) ||
        (data_out[3] != cpu_to_be32(0xccddeeff)))
    {
        FAILED("encryption output incorrect");
        printk("%08x%08x%08x%08x\n",data_out[0],data_out[1],data_out[2],data_out[3]);
    }
        
    // free dmas
    oxnas_dma_free( dma_in ); 
    oxnas_dma_free( dma_out ); 
    
    kfree(data_in);
    kfree(data_out);

    return failed;

}


/**********************************************************************/
/* LRW tests                                                          */
/**********************************************************************/

static int test14(void) {
    int failed = 0;
    u32 reg;
    oxnas_dma_channel_t* dma_in;
    oxnas_dma_channel_t* dma_out;
    dma_addr_t in_address;
    dma_addr_t out_address;
    
    u32* data_in;
    u32* data_out;
    
    // setup dmas
    dma_in = oxnas_dma_request(1);
    dma_out = oxnas_dma_request(1);
    
    // get some dma accessable memory
    data_in = (u32*)kmalloc( 8 * sizeof(u32), GFP_KERNEL|GFP_DMA);
    data_out = (u32*)kmalloc( 8 * sizeof(u32), GFP_KERNEL|GFP_DMA);
    
    // fill with input and non expected output
    data_in[0] = be32_to_cpu( 0x30313233);
    data_in[1] = be32_to_cpu( 0x34353637);
    data_in[2] = be32_to_cpu( 0x38394142);
    data_in[3] = be32_to_cpu( 0x43444546);
    data_in[4] = be32_to_cpu( 0x30313233);
    data_in[5] = be32_to_cpu( 0x34353637);
    data_in[6] = be32_to_cpu( 0x38394142);
    data_in[7] = be32_to_cpu( 0x43444546);
    data_out[0] = ~0;
    data_out[1] = ~0;
    data_out[2] = ~0;
    data_out[3] = ~0;
    data_out[4] = ~0;
    data_out[5] = ~0;
    data_out[6] = ~0;
    data_out[7] = ~0;
    
    // map the dma regons
    in_address = dma_map_single(
        0,
        data_in,
        8 * sizeof(u32),
        DMA_TO_DEVICE);
        
    // map the dma regons
    out_address = dma_map_single(
        0,
        data_out,
        8 * sizeof(u32),
        DMA_FROM_DEVICE);
        
    // setup the transfers
    oxnas_dma_device_set(
        dma_in,
        OXNAS_DMA_TO_DEVICE,
        (char*)in_address,
        8 * sizeof(u32),
        &oxnas_dpe_rx_dma_settings,
        OXNAS_DMA_MODE_INC, 1);
        
    oxnas_dma_device_set(
        dma_out,
        OXNAS_DMA_FROM_DEVICE,
        (char*)out_address,
        8 * sizeof(u32),
        &oxnas_dpe_tx_dma_settings,
        OXNAS_DMA_MODE_INC, 1);

    // don't authenticate
    WRITEL(0 ,OX800IBW_STATUS);
    
    // toggle cleardown bit to start
    WRITEL(OX800DPE_CTL_ABORT  ,OX800DPE_CONTROL);
    WRITEL(0                   ,OX800DPE_CONTROL);
    
    
    // shouldn't be busy or full
    reg = READL( OX800DPE_STATUS );
    if (! (reg & OX800DPE_STAT_IDLE) )
        FAILED("not idle after abort toggle");
    if (reg & OX800DPE_STAT_TX_NOTEMPTY)
        FAILED("tx fifo not empty after abort toggle");
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx not empty after abort toggle");
    
    // setup for encryption 
    reg = OX800DPE_CTL_DIRECTION_ENC |
          OX800DPE_CTL_MODE_LRW_AES;
    WRITEL(reg  ,OX800DPE_CONTROL);

    // key no 1
    WRITEL(be32_to_cpu( 0x4562ac25), OX800DPE_KEY00 );
    WRITEL(be32_to_cpu( 0xf828176d), OX800DPE_KEY01 );
    WRITEL(be32_to_cpu( 0x4c268414), OX800DPE_KEY02 );
    WRITEL(be32_to_cpu( 0xb5680185), OX800DPE_KEY03 );
    
    // key no 2
    WRITEL(be32_to_cpu( 0x258e2a05), OX800DPE_KEY10 );
    WRITEL(be32_to_cpu( 0xe73e9d03), OX800DPE_KEY11 );
    WRITEL(be32_to_cpu( 0xee5a830c), OX800DPE_KEY12 );
    WRITEL(be32_to_cpu( 0xcc094c87), OX800DPE_KEY13 );

    // setup index
    WRITEL(0, OX800DPE_DATA_LRW0 );
    WRITEL(0, OX800DPE_DATA_LRW1 );

    /* wait until done */
    while(  !(OX800DPE_STAT_IDLE & READL( OX800DPE_STATUS )) );

    // start dma transfers
    oxnas_dma_start(dma_out);
    oxnas_dma_start(dma_in);
    
    /* wait until dma done */
    while(  oxnas_dma_is_active( dma_out ) );
    
    reg = READL( OX800DPE_STATUS );

    /* output should be empty */
    if ( (reg & OX800DPE_STAT_TX_NOTEMPTY) )
        FAILED("tx still full after fetching ");

    /* in empty */
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx still full after encrypting data ");
    
    dma_unmap_single(0, in_address, 8 * sizeof(u32), DMA_TO_DEVICE);
    dma_unmap_single(0, out_address, 8 * sizeof(u32), DMA_FROM_DEVICE);

    /*
    check output.
    note: first 4 quads contain unwanted data used to set
    tweek location to 1, they are not checked.
    */
    if ((data_out[4] != cpu_to_be32(0xf1b273cd)) ||
        (data_out[5] != cpu_to_be32(0x65a3df5f)) ||
        (data_out[6] != cpu_to_be32(0xe95d4892)) ||
        (data_out[7] != cpu_to_be32(0x54634eb8)))
    {
        FAILED("encryption output incorrect");
        printk("%08x%08x%08x%08x\n",data_out[0],data_out[1],data_out[2],data_out[3]);
        printk("%08x%08x%08x%08x\n",data_out[4],data_out[5],data_out[6],data_out[7]);
    }
        
    // free dmas
    oxnas_dma_free( dma_in ); 
    oxnas_dma_free( dma_out ); 
    
    kfree(data_in);
    kfree(data_out);

    return failed;

}


/*************************************************************************/
static int test15(void) {
    int failed = 0;
    u32 reg;
    oxnas_dma_channel_t* dma_in;
    oxnas_dma_channel_t* dma_out;
    dma_addr_t in_address;
    dma_addr_t out_address;
    
    u32* data_in;
    u32* data_out;
    
    // setup dmas
    dma_in = oxnas_dma_request(1);
    dma_out = oxnas_dma_request(1);
    
    // get some dma accessable memory
    data_in = (u32*)kmalloc( 8 * sizeof(u32), GFP_KERNEL|GFP_DMA);
    data_out = (u32*)kmalloc( 8 * sizeof(u32), GFP_KERNEL|GFP_DMA);
    
    // fill with input and non expected output
    data_in[0] = be32_to_cpu( 0xf1b273cd);
    data_in[1] = be32_to_cpu( 0x65a3df5f);
    data_in[2] = be32_to_cpu( 0xe95d4892);
    data_in[3] = be32_to_cpu( 0x54634eb8);                              
    data_in[4] = be32_to_cpu( 0xf1b273cd);
    data_in[5] = be32_to_cpu( 0x65a3df5f);
    data_in[6] = be32_to_cpu( 0xe95d4892);
    data_in[7] = be32_to_cpu( 0x54634eb8);
    data_out[0] = ~0;
    data_out[1] = ~0;
    data_out[2] = ~0;
    data_out[3] = ~0;
    data_out[4] = ~0;
    data_out[5] = ~0;
    data_out[6] = ~0;
    data_out[7] = ~0;
    
    // map the dma regons
    in_address = dma_map_single(
        0,
        data_in,
        8 * sizeof(u32),
        DMA_TO_DEVICE);
        
    // map the dma regons
    out_address = dma_map_single(
        0,
        data_out,
        8 * sizeof(u32),
        DMA_FROM_DEVICE);
        
    // setup the transfers
    oxnas_dma_device_set(
        dma_in,
        OXNAS_DMA_TO_DEVICE,
        (char*)in_address,
        8 * sizeof(u32),
        &oxnas_dpe_rx_dma_settings,
        OXNAS_DMA_MODE_INC, 1);
        
    oxnas_dma_device_set(
        dma_out,
        OXNAS_DMA_FROM_DEVICE,
        (char*)out_address,
        8 * sizeof(u32),
        &oxnas_dpe_tx_dma_settings,
        OXNAS_DMA_MODE_INC, 1);

    // authenticate
    WRITEL(OX800IBW_STAT_AUTHENTICATED ,OX800IBW_STATUS);
    
    // toggle cleardown bit to start
    WRITEL(OX800DPE_CTL_ABORT  ,OX800DPE_CONTROL);
    WRITEL(0                   ,OX800DPE_CONTROL);
    
    // shouldn't be busy or full
    reg = READL( OX800DPE_STATUS );
    if (! (reg & OX800DPE_STAT_IDLE) )
        FAILED("not idle after abort toggle");
    if (reg & OX800DPE_STAT_TX_NOTEMPTY)
        FAILED("tx fifo not empty after abort toggle");
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx not empty after abort toggle");
    
    // setup for decryption 
    reg = OX800DPE_CTL_MODE_LRW_AES |
          OX800DPE_CTL_PRIMARY_IS_KEY3;
    WRITEL(reg  ,OX800DPE_CONTROL);

    // key no 1
    WRITEL(be32_to_cpu( 0x4562ac25), OX800DPE_KEY20 );
    WRITEL(be32_to_cpu( 0xf828176d), OX800DPE_KEY21 );
    WRITEL(be32_to_cpu( 0x4c268414), OX800DPE_KEY22 );
    WRITEL(be32_to_cpu( 0xb5680185), OX800DPE_KEY23 );
    
    // key no 2
    WRITEL(be32_to_cpu( 0x258e2a05), OX800DPE_KEY10 );
    WRITEL(be32_to_cpu( 0xe73e9d03), OX800DPE_KEY11 );
    WRITEL(be32_to_cpu( 0xee5a830c), OX800DPE_KEY12 );
    WRITEL(be32_to_cpu( 0xcc094c87), OX800DPE_KEY13 );
    
    // setup index
    WRITEL(0, OX800DPE_DATA_LRW0 );
    WRITEL(0, OX800DPE_DATA_LRW1 );

    /* wait until done */
    while(  !(OX800DPE_STAT_IDLE & READL( OX800DPE_STATUS )) );
    
    // start dma transfers
    oxnas_dma_start(dma_out);
    oxnas_dma_start(dma_in);
    
    /* wait until done */
    while(  oxnas_dma_is_active( dma_out ) );
    
    reg = READL( OX800DPE_STATUS );

    /* output should be empty */
    if ( (reg & OX800DPE_STAT_TX_NOTEMPTY) )
        FAILED("tx still full after fetching ");

    /* in empty */
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx still full after encrypting data ");
    
    dma_unmap_single(0, in_address, 8 * sizeof(u32), DMA_TO_DEVICE);
    dma_unmap_single(0, out_address, 8 * sizeof(u32), DMA_FROM_DEVICE);

    /* check output */
    if ((data_out[4] != cpu_to_be32(0x30313233)) ||
        (data_out[5] != cpu_to_be32(0x34353637)) ||
        (data_out[6] != cpu_to_be32(0x38394142)) ||
        (data_out[7] != cpu_to_be32(0x43444546)))
    {
        FAILED("encryption output incorrect");
        printk("%08x%08x%08x%08x\n",data_out[0],data_out[1],data_out[2],data_out[3]);
        printk("%08x%08x%08x%08x\n",data_out[4],data_out[5],data_out[6],data_out[7]);
    }
        
    // free dmas
    oxnas_dma_free( dma_in ); 
    oxnas_dma_free( dma_out ); 
    
    kfree(data_in);
    kfree(data_out);

    return failed;

}

/*************************************************************************/
static int test16(void) {
    int failed = 0;
    u32 reg;
    oxnas_dma_channel_t* dma_in;
    oxnas_dma_channel_t* dma_out;
    dma_addr_t in_address;
    dma_addr_t out_address;
    
    u32* data_in;
    u32* data_out;
    
    // setup dmas
    dma_in = oxnas_dma_request(1);
    dma_out = oxnas_dma_request(1);
    
    // get some dma accessable memory
    data_in = (u32*)kmalloc( 12 * sizeof(u32), GFP_KERNEL|GFP_DMA);
    data_out = (u32*)kmalloc( 12 * sizeof(u32), GFP_KERNEL|GFP_DMA);
    
    // fill with input and non expected output
    data_in[8 ] = be32_to_cpu( 0x30313233);
    data_in[9 ] = be32_to_cpu( 0x34353637);
    data_in[10] = be32_to_cpu( 0x38394142);
    data_in[11] = be32_to_cpu( 0x43444546);
    data_out[0] = ~0;
    data_out[1] = ~0;
    data_out[2] = ~0;
    data_out[3] = ~0;
    data_out[4] = ~0;
    data_out[5] = ~0;
    data_out[6] = ~0;
    data_out[7] = ~0;
    data_out[8] = ~0;
    data_out[9] = ~0;
    data_out[10] = ~0;
    data_out[11] = ~0;
    
    // map the dma regons
    in_address = dma_map_single(
        0,
        data_in,
        12 * sizeof(u32),
        DMA_TO_DEVICE);
        
    // map the dma regons
    out_address = dma_map_single(
        0,
        data_out,
        12 * sizeof(u32),
        DMA_FROM_DEVICE);
        
    // setup the transfers
    oxnas_dma_device_set(
        dma_in,
        OXNAS_DMA_TO_DEVICE,
        (char*)in_address,
        12 * sizeof(u32),
        &oxnas_dpe_rx_dma_settings,
        OXNAS_DMA_MODE_INC, 1);
        
    oxnas_dma_device_set(
        dma_out,
        OXNAS_DMA_FROM_DEVICE,
        (char*)out_address,
        12 * sizeof(u32),
        &oxnas_dpe_tx_dma_settings,
        OXNAS_DMA_MODE_INC, 1);

    // don't authenticate
    WRITEL(0 ,OX800IBW_STATUS);
    
    // toggle cleardown bit to start
    WRITEL(OX800DPE_CTL_ABORT  ,OX800DPE_CONTROL);
    WRITEL(0                   ,OX800DPE_CONTROL);
    
    
    // shouldn't be busy or full
    reg = READL( OX800DPE_STATUS );
    if (! (reg & OX800DPE_STAT_IDLE) )
        FAILED("not idle after abort toggle");
    if (reg & OX800DPE_STAT_TX_NOTEMPTY)
        FAILED("tx fifo not empty after abort toggle");
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx not empty after abort toggle");
    
    // setup for encryption 
    reg = OX800DPE_CTL_DIRECTION_ENC |
          OX800DPE_CTL_PRIMARY_IS_KEY3 |
          OX800DPE_CTL_MODE_LRW_AES;
    WRITEL(reg  ,OX800DPE_CONTROL);

    // key no 1
    WRITEL(be32_to_cpu( 0x59704714), OX800DPE_KEY20 );
    WRITEL(be32_to_cpu( 0xf557478c), OX800DPE_KEY21 );
    WRITEL(be32_to_cpu( 0xd779e80f), OX800DPE_KEY22 );
    WRITEL(be32_to_cpu( 0x54887944), OX800DPE_KEY23 );
    
    // key no 2
    WRITEL(be32_to_cpu( 0x0d48f0b7), OX800DPE_KEY10 );
    WRITEL(be32_to_cpu( 0xb15a53ea), OX800DPE_KEY11 );
    WRITEL(be32_to_cpu( 0x1caa6b29), OX800DPE_KEY12 );
    WRITEL(be32_to_cpu( 0xc2cafbaf), OX800DPE_KEY13 );

    // setup index
    WRITEL(0, OX800DPE_DATA_LRW0 );
    WRITEL(0, OX800DPE_DATA_LRW1 );

    /* wait until done */
    while(  !(OX800DPE_STAT_IDLE & READL( OX800DPE_STATUS )) );

    // start dma transfers
    oxnas_dma_start(dma_out);
    oxnas_dma_start(dma_in);
    
    /* wait until dma done */
    while(  oxnas_dma_is_active( dma_out ) );
    
    reg = READL( OX800DPE_STATUS );

    /* output should be empty */
    if ( (reg & OX800DPE_STAT_TX_NOTEMPTY) )
        FAILED("tx still full after fetching ");

    /* in empty */
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx still full after encrypting data ");
    
    dma_unmap_single(0, in_address, 12 * sizeof(u32), DMA_TO_DEVICE);
    dma_unmap_single(0, out_address, 12 * sizeof(u32), DMA_FROM_DEVICE);

    /*
    check output.
    note: first 4 quads contain unwanted data used to set
    tweek location to 1, they are not checked.
    */
    if ((data_out[ 8] != cpu_to_be32(0x00c82bae)) ||
        (data_out[ 9] != cpu_to_be32(0x95bbcde5)) ||
        (data_out[10] != cpu_to_be32(0x274f0769)) ||
        (data_out[11] != cpu_to_be32(0xb260e136)))
    {
        FAILED("encryption output incorrect");
        printk("%08x%08x%08x%08x\n",data_out[8],data_out[9],data_out[10],data_out[11]);
    }
        
    // free dmas
    oxnas_dma_free( dma_in ); 
    oxnas_dma_free( dma_out ); 
    
    kfree(data_in);
    kfree(data_out);

    return failed;

}


/*************************************************************************/
static int test17(void) {
    int failed = 0;
    u32 reg;
    oxnas_dma_channel_t* dma_in;
    oxnas_dma_channel_t* dma_out;
    dma_addr_t in_address;
    dma_addr_t out_address;
    
    u32* data_in;
    u32* data_out;
    
    // setup dmas
    dma_in = oxnas_dma_request(1);
    dma_out = oxnas_dma_request(1);
    
    // get some dma accessable memory
    data_in = (u32*)kmalloc( 12 * sizeof(u32), GFP_KERNEL|GFP_DMA);
    data_out = (u32*)kmalloc( 12 * sizeof(u32), GFP_KERNEL|GFP_DMA);
    
    // fill with input and non expected output
    data_in[ 8] = be32_to_cpu( 0x00c82bae);
    data_in[ 9] = be32_to_cpu( 0x95bbcde5);
    data_in[10] = be32_to_cpu( 0x274f0769);
    data_in[11] = be32_to_cpu( 0xb260e136);
    data_out[ 0] = ~0;
    data_out[ 1] = ~0;
    data_out[ 2] = ~0;
    data_out[ 3] = ~0;
    data_out[ 4] = ~0;
    data_out[ 5] = ~0;
    data_out[ 6] = ~0;
    data_out[ 7] = ~0;
    data_out[ 8] = ~0;
    data_out[ 9] = ~0;
    data_out[10] = ~0;
    data_out[11] = ~0;
    
    // map the dma regons
    in_address = dma_map_single(
        0,
        data_in,
        12 * sizeof(u32),
        DMA_TO_DEVICE);
        
    // map the dma regons
    out_address = dma_map_single(
        0,
        data_out,
        12 * sizeof(u32),
        DMA_FROM_DEVICE);
        
    // setup the transfers
    oxnas_dma_device_set(
        dma_in,
        OXNAS_DMA_TO_DEVICE,
        (char*)in_address,
        12 * sizeof(u32),
        &oxnas_dpe_rx_dma_settings,
        OXNAS_DMA_MODE_INC, 1);
        
    oxnas_dma_device_set(
        dma_out,
        OXNAS_DMA_FROM_DEVICE,
        (char*)out_address,
        12 * sizeof(u32),
        &oxnas_dpe_tx_dma_settings,
        OXNAS_DMA_MODE_INC, 1);

    // authenticate
    WRITEL(OX800IBW_STAT_AUTHENTICATED ,OX800IBW_STATUS);
    
    // toggle cleardown bit to start
    WRITEL(OX800DPE_CTL_ABORT  ,OX800DPE_CONTROL);
    WRITEL(0                   ,OX800DPE_CONTROL);
    
    // shouldn't be busy or full
    reg = READL( OX800DPE_STATUS );
    if (! (reg & OX800DPE_STAT_IDLE) )
        FAILED("not idle after abort toggle");
    if (reg & OX800DPE_STAT_TX_NOTEMPTY)
        FAILED("tx fifo not empty after abort toggle");
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx not empty after abort toggle");
    
    // setup for decryption 
    reg = OX800DPE_CTL_MODE_LRW_AES ;
    WRITEL(reg  ,OX800DPE_CONTROL);

    // key no 1
    WRITEL(be32_to_cpu( 0x59704714), OX800DPE_KEY00 );
    WRITEL(be32_to_cpu( 0xf557478c), OX800DPE_KEY01 );
    WRITEL(be32_to_cpu( 0xd779e80f), OX800DPE_KEY02 );
    WRITEL(be32_to_cpu( 0x54887944), OX800DPE_KEY03 );
                        
    // key no 2         
    WRITEL(be32_to_cpu( 0x0d48f0b7), OX800DPE_KEY10 );
    WRITEL(be32_to_cpu( 0xb15a53ea), OX800DPE_KEY11 );
    WRITEL(be32_to_cpu( 0x1caa6b29), OX800DPE_KEY12 );
    WRITEL(be32_to_cpu( 0xc2cafbaf), OX800DPE_KEY13 );
    
    // setup index
    WRITEL(0, OX800DPE_DATA_LRW0 );
    WRITEL(0, OX800DPE_DATA_LRW1 );

    /* wait until done */
    while(  !(OX800DPE_STAT_IDLE & READL( OX800DPE_STATUS )) );
    
    // start dma transfers
    oxnas_dma_start(dma_out);
    oxnas_dma_start(dma_in);
    
    /* wait until done */
    while(  oxnas_dma_is_active( dma_out ) );
    
    reg = READL( OX800DPE_STATUS );

    /* output should be empty */
    if ( (reg & OX800DPE_STAT_TX_NOTEMPTY) )
        FAILED("tx still full after fetching ");

    /* in empty */
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx still full after encrypting data ");
    
    dma_unmap_single(0, in_address, 12 * sizeof(u32), DMA_TO_DEVICE);
    dma_unmap_single(0, out_address, 12 * sizeof(u32), DMA_FROM_DEVICE);

    /* check output */
    if ((data_out[ 8] != cpu_to_be32(0x30313233)) ||
        (data_out[ 9] != cpu_to_be32(0x34353637)) ||
        (data_out[10] != cpu_to_be32(0x38394142)) ||
        (data_out[11] != cpu_to_be32(0x43444546)))
    {
        FAILED("encryption output incorrect");
        printk("%08x%08x%08x%08x\n",data_out[8],data_out[9],data_out[10],data_out[11]);
    }
        
    // free dmas
    oxnas_dma_free( dma_in ); 
    oxnas_dma_free( dma_out ); 
    
    kfree(data_in);
    kfree(data_out);

    return failed;

}

/*************************************************************************/
static int test18(void) {
    int failed = 0;
    u32 reg;
    oxnas_dma_channel_t* dma_in;
    oxnas_dma_channel_t* dma_out;
    dma_addr_t in_address;
    dma_addr_t out_address;
    
    u32* data_in;
    u32* data_out;
    
    // setup dmas
    dma_in = oxnas_dma_request(1);
    dma_out = oxnas_dma_request(1);
    
    // get some dma accessable memory
    data_in = (u32*)kmalloc( 4 * sizeof(u32), GFP_KERNEL|GFP_DMA);
    data_out = (u32*)kmalloc( 4 * sizeof(u32), GFP_KERNEL|GFP_DMA);
    
    // fill with input and non expected output
    data_in[0] = be32_to_cpu( 0x30313233);
    data_in[1] = be32_to_cpu( 0x34353637);
    data_in[2] = be32_to_cpu( 0x38394142);
    data_in[3] = be32_to_cpu( 0x43444546);
    data_out[0] = ~0;
    data_out[1] = ~0;
    data_out[2] = ~0;
    data_out[3] = ~0;
    
    // map the dma regons
    in_address = dma_map_single(
        0,
        data_in,
        4 * sizeof(u32),
        DMA_TO_DEVICE);
        
    // map the dma regons
    out_address = dma_map_single(
        0,
        data_out,
        4 * sizeof(u32),
        DMA_FROM_DEVICE);
        
    // setup the transfers
    oxnas_dma_device_set(
        dma_in,
        OXNAS_DMA_TO_DEVICE,
        (char*)in_address,
        4 * sizeof(u32),
        &oxnas_dpe_rx_dma_settings,
        OXNAS_DMA_MODE_INC, 1);
        
    oxnas_dma_device_set(
        dma_out,
        OXNAS_DMA_FROM_DEVICE,
        (char*)out_address,
        4 * sizeof(u32),
        &oxnas_dpe_tx_dma_settings,
        OXNAS_DMA_MODE_INC, 1);

    // don't authenticate
    WRITEL(0 ,OX800IBW_STATUS);
    
    // toggle cleardown bit to start
    WRITEL(OX800DPE_CTL_ABORT  ,OX800DPE_CONTROL);
    WRITEL(0                   ,OX800DPE_CONTROL);
    
    
    // shouldn't be busy or full
    reg = READL( OX800DPE_STATUS );
    if (! (reg & OX800DPE_STAT_IDLE) )
        FAILED("not idle after abort toggle");
    if (reg & OX800DPE_STAT_TX_NOTEMPTY)
        FAILED("tx fifo not empty after abort toggle");
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx not empty after abort toggle");
    
    // setup for encryption 
    reg = OX800DPE_CTL_DIRECTION_ENC |
          OX800DPE_CTL_MODE_LRW_AES;
    WRITEL(reg  ,OX800DPE_CONTROL);

    // key no 1
    WRITEL(be32_to_cpu( 0xd82a9134), OX800DPE_KEY00 );
    WRITEL(be32_to_cpu( 0xb26a5650), OX800DPE_KEY01 );
    WRITEL(be32_to_cpu( 0x30fe69e2), OX800DPE_KEY02 );
    WRITEL(be32_to_cpu( 0x377f9847), OX800DPE_KEY03 );
    
    // key no 2
    WRITEL(be32_to_cpu( 0xcdf90b16), OX800DPE_KEY10 );
    WRITEL(be32_to_cpu( 0x0c648fb6), OX800DPE_KEY11 );
    WRITEL(be32_to_cpu( 0xb00d0d1b), OX800DPE_KEY12 );
    WRITEL(be32_to_cpu( 0xae85871f), OX800DPE_KEY13 );

    // setup index
    WRITEL(0x10000000, OX800DPE_DATA_LRW0 );
    WRITEL(0, OX800DPE_DATA_LRW1 );

    /* wait until done */
    while(  !(OX800DPE_STAT_IDLE & READL( OX800DPE_STATUS )) );

    // start dma transfers
    oxnas_dma_start(dma_out);
    oxnas_dma_start(dma_in);
    
    /* wait until dma done */
    while(  oxnas_dma_is_active( dma_out ) );
    
    reg = READL( OX800DPE_STATUS );

    /* output should be empty */
    if ( (reg & OX800DPE_STAT_TX_NOTEMPTY) )
        FAILED("tx still full after fetching ");

    /* in empty */
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx still full after encrypting data ");
    
    dma_unmap_single(0, in_address, 4 * sizeof(u32), DMA_TO_DEVICE);
    dma_unmap_single(0, out_address, 4 * sizeof(u32), DMA_FROM_DEVICE);

    /*
    check output.
    */
    if ((data_out[0] != cpu_to_be32(0x76322183)) ||
        (data_out[1] != cpu_to_be32(0xed8ff182)) ||
        (data_out[2] != cpu_to_be32(0xf9596203)) ||
        (data_out[3] != cpu_to_be32(0x690e5e01)))
    {
        FAILED("encryption output incorrect");
        printk("%08x%08x%08x%08x\n",data_out[0],data_out[1],data_out[2],data_out[3]);
    }
        
    // free dmas
    oxnas_dma_free( dma_in ); 
    oxnas_dma_free( dma_out ); 
    
    kfree(data_in);
    kfree(data_out);

    return failed;

}


/*************************************************************************/
static int test19(void) {
    int failed = 0;
    u32 reg;
    oxnas_dma_channel_t* dma_in;
    oxnas_dma_channel_t* dma_out;
    dma_addr_t in_address;
    dma_addr_t out_address;
    
    u32* data_in;
    u32* data_out;
    
    // setup dmas
    dma_in = oxnas_dma_request(1);
    dma_out = oxnas_dma_request(1);
    
    // get some dma accessable memory
    data_in = (u32*)kmalloc( 8 * sizeof(u32), GFP_KERNEL|GFP_DMA);
    data_out = (u32*)kmalloc( 8 * sizeof(u32), GFP_KERNEL|GFP_DMA);
    
    // fill with input and non expected output
    data_in[0] = be32_to_cpu( 0x76322183);
    data_in[1] = be32_to_cpu( 0xed8ff182);
    data_in[2] = be32_to_cpu( 0xf9596203);
    data_in[3] = be32_to_cpu( 0x690e5e01);                              
    data_out[0] = ~0;
    data_out[1] = ~0;
    data_out[2] = ~0;
    data_out[3] = ~0;
    
    // map the dma regons
    in_address = dma_map_single(
        0,
        data_in,
        4 * sizeof(u32),
        DMA_TO_DEVICE);
        
    // map the dma regons
    out_address = dma_map_single(
        0,
        data_out,
        4 * sizeof(u32),
        DMA_FROM_DEVICE);
        
    // setup the transfers
    oxnas_dma_device_set(
        dma_in,
        OXNAS_DMA_TO_DEVICE,
        (char*)in_address,
        4 * sizeof(u32),
        &oxnas_dpe_rx_dma_settings,
        OXNAS_DMA_MODE_INC, 1);
        
    oxnas_dma_device_set(
        dma_out,
        OXNAS_DMA_FROM_DEVICE,
        (char*)out_address,
        4 * sizeof(u32),
        &oxnas_dpe_tx_dma_settings,
        OXNAS_DMA_MODE_INC, 1);

    // authenticate
    WRITEL(OX800IBW_STAT_AUTHENTICATED ,OX800IBW_STATUS);
    
    // toggle cleardown bit to start
    WRITEL(OX800DPE_CTL_ABORT  ,OX800DPE_CONTROL);
    WRITEL(0                   ,OX800DPE_CONTROL);
    
    // shouldn't be busy or full
    reg = READL( OX800DPE_STATUS );
    if (! (reg & OX800DPE_STAT_IDLE) )
        FAILED("not idle after abort toggle");
    if (reg & OX800DPE_STAT_TX_NOTEMPTY)
        FAILED("tx fifo not empty after abort toggle");
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx not empty after abort toggle");
    
    // setup for decryption 
    reg = OX800DPE_CTL_MODE_LRW_AES |
          OX800DPE_CTL_PRIMARY_IS_KEY3;
    WRITEL(reg  ,OX800DPE_CONTROL);

    // key no 1
    WRITEL(be32_to_cpu( 0xd82a9134), OX800DPE_KEY20 );
    WRITEL(be32_to_cpu( 0xb26a5650), OX800DPE_KEY21 );
    WRITEL(be32_to_cpu( 0x30fe69e2), OX800DPE_KEY22 );
    WRITEL(be32_to_cpu( 0x377f9847), OX800DPE_KEY23 );
    
    // key no 2
    WRITEL(be32_to_cpu( 0xcdf90b16), OX800DPE_KEY10 );
    WRITEL(be32_to_cpu( 0x0c648fb6), OX800DPE_KEY11 );
    WRITEL(be32_to_cpu( 0xb00d0d1b), OX800DPE_KEY12 );
    WRITEL(be32_to_cpu( 0xae85871f), OX800DPE_KEY13 );
    
    // setup index
    WRITEL(0x10000000, OX800DPE_DATA_LRW0 );
    WRITEL(0, OX800DPE_DATA_LRW1 );

    /* wait until done */
    while(  !(OX800DPE_STAT_IDLE & READL( OX800DPE_STATUS )) );

    // start dma transfers
    oxnas_dma_start(dma_out);
    oxnas_dma_start(dma_in);
    
    /* wait until done */
    while(  oxnas_dma_is_active( dma_out ) );
    
    reg = READL( OX800DPE_STATUS );

    /* output should be empty */
    if ( (reg & OX800DPE_STAT_TX_NOTEMPTY) )
        FAILED("tx still full after fetching ");

    /* in empty */
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx still full after encrypting data ");
    
    dma_unmap_single(0, in_address, 4 * sizeof(u32), DMA_TO_DEVICE);
    dma_unmap_single(0, out_address, 4 * sizeof(u32), DMA_FROM_DEVICE);

    /* check output */
    if ((data_out[0] != cpu_to_be32(0x30313233)) ||
        (data_out[1] != cpu_to_be32(0x34353637)) ||
        (data_out[2] != cpu_to_be32(0x38394142)) ||
        (data_out[3] != cpu_to_be32(0x43444546)))
    {
        FAILED("encryption output incorrect");
        printk("%08x%08x%08x%08x\n",data_out[0],data_out[1],data_out[2],data_out[3]);
    }
        
    // free dmas
    oxnas_dma_free( dma_in ); 
    oxnas_dma_free( dma_out ); 
    
    kfree(data_in);
    kfree(data_out);

    return failed;

}

/*************************************************************************/
static int test20(void) {
    int failed = 0;
    u32 reg;
    oxnas_dma_channel_t* dma_in;
    oxnas_dma_channel_t* dma_out;
    dma_addr_t in_address;
    dma_addr_t out_address;
    
    u32* data_in;
    u32* data_out;
    
    // setup dmas
    dma_in = oxnas_dma_request(1);
    dma_out = oxnas_dma_request(1);
    
    // get some dma accessable memory
    data_in = (u32*)kmalloc( 4 * sizeof(u32), GFP_KERNEL|GFP_DMA);
    data_out = (u32*)kmalloc( 4 * sizeof(u32), GFP_KERNEL|GFP_DMA);
    
    // fill with input and non expected output
    data_in[0] = be32_to_cpu( 0x30313233);
    data_in[1] = be32_to_cpu( 0x34353637);
    data_in[2] = be32_to_cpu( 0x38394142);
    data_in[3] = be32_to_cpu( 0x43444546);
    data_out[0] = ~0;
    data_out[1] = ~0;
    data_out[2] = ~0;
    data_out[3] = ~0;
    
    // map the dma regons
    in_address = dma_map_single(
        0,
        data_in,
        4 * sizeof(u32),
        DMA_TO_DEVICE);
        
    // map the dma regons
    out_address = dma_map_single(
        0,
        data_out,
        4 * sizeof(u32),
        DMA_FROM_DEVICE);
        
    // setup the transfers
    oxnas_dma_device_set(
        dma_in,
        OXNAS_DMA_TO_DEVICE,
        (char*)in_address,
        4 * sizeof(u32),
        &oxnas_dpe_rx_dma_settings,
        OXNAS_DMA_MODE_INC, 1);
        
    oxnas_dma_device_set(
        dma_out,
        OXNAS_DMA_FROM_DEVICE,
        (char*)out_address,
        4 * sizeof(u32),
        &oxnas_dpe_tx_dma_settings,
        OXNAS_DMA_MODE_INC, 1);

    // don't authenticate
    WRITEL(0 ,OX800IBW_STATUS);
    
    // toggle cleardown bit to start
    WRITEL(OX800DPE_CTL_ABORT  ,OX800DPE_CONTROL);
    WRITEL(0                   ,OX800DPE_CONTROL);
    
    
    // shouldn't be busy or full
    reg = READL( OX800DPE_STATUS );
    if (! (reg & OX800DPE_STAT_IDLE) )
        FAILED("not idle after abort toggle");
    if (reg & OX800DPE_STAT_TX_NOTEMPTY)
        FAILED("tx fifo not empty after abort toggle");
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx not empty after abort toggle");
    
    // setup for encryption 
    reg = OX800DPE_CTL_DIRECTION_ENC |
          OX800DPE_CTL_MODE_LRW_AES;
    WRITEL(reg  ,OX800DPE_CONTROL);

    // key no 1
    WRITEL(be32_to_cpu( 0x4562ac25), OX800DPE_KEY00 );
    WRITEL(be32_to_cpu( 0xf828176d), OX800DPE_KEY01 );
    WRITEL(be32_to_cpu( 0x4c268414), OX800DPE_KEY02 );
    WRITEL(be32_to_cpu( 0xb5680185), OX800DPE_KEY03 );
    
    // key no 2
    WRITEL(be32_to_cpu( 0x258e2a05), OX800DPE_KEY10 );
    WRITEL(be32_to_cpu( 0xe73e9d03), OX800DPE_KEY11 );
    WRITEL(be32_to_cpu( 0xee5a830c), OX800DPE_KEY12 );
    WRITEL(be32_to_cpu( 0xcc094c87), OX800DPE_KEY13 );

    // setup index
    WRITEL(0x00000000, OX800DPE_DATA_LRW0 );
    WRITEL(0x00000008, OX800DPE_DATA_LRW1 );

    /* wait until done */
    while(  !(OX800DPE_STAT_IDLE & READL( OX800DPE_STATUS )) );

    // start dma transfers
    oxnas_dma_start(dma_out);
    oxnas_dma_start(dma_in);
    
    /* wait until dma done */
    while(  oxnas_dma_is_active( dma_out ) );
    
    reg = READL( OX800DPE_STATUS );

    /* output should be empty */
    if ( (reg & OX800DPE_STAT_TX_NOTEMPTY) )
        FAILED("tx still full after fetching ");

    /* in empty */
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx still full after encrypting data ");
    
    dma_unmap_single(0, in_address, 4 * sizeof(u32), DMA_TO_DEVICE);
    dma_unmap_single(0, out_address, 4 * sizeof(u32), DMA_FROM_DEVICE);

    /*
    check output.
    */
    if ((data_out[0] == 0xc0a37fda) )
    {
        FAILED("encryption output indicates most significant bit is ignored.");
        printk("%08x%08x%08x%08x\n",data_out[0],data_out[1],data_out[2],data_out[3]);
    }
        
    // free dmas
    oxnas_dma_free( dma_in ); 
    oxnas_dma_free( dma_out ); 
    
    kfree(data_in);
    kfree(data_out);

    return failed;

}


/*************************************************************************/
static int test21(void) {
    int failed = 0;
    u32 reg;
    oxnas_dma_channel_t* dma_in;
    oxnas_dma_channel_t* dma_out;
    dma_addr_t in_address;
    dma_addr_t out_address;
    
    u32* data_in;
    u32* data_out;
    
    // setup dmas
    dma_in = oxnas_dma_request(1);
    dma_out = oxnas_dma_request(1);
    
    // get some dma accessable memory (512B + 16B
    data_in = (u32*)kmalloc( 132 * sizeof(u32), GFP_KERNEL|GFP_DMA);
    data_out = (u32*)kmalloc( 132 * sizeof(u32), GFP_KERNEL|GFP_DMA);
    
    // fill with input and non expected output
    data_in[128] = be32_to_cpu( 0x30313233);
    data_in[129] = be32_to_cpu( 0x34353637);
    data_in[130] = be32_to_cpu( 0x38394142);
    data_in[131] = be32_to_cpu( 0x43444546);                              
    data_out[128] = ~0;
    data_out[129] = ~0;
    data_out[130] = ~0;
    data_out[131] = ~0;
    
    // map the dma regons
    in_address = dma_map_single(
        0,
        data_in,
        132 * sizeof(u32),
        DMA_TO_DEVICE);
        
    // map the dma regons
    out_address = dma_map_single(
        0,
        data_out,
        132 * sizeof(u32),
        DMA_FROM_DEVICE);
        
    // setup the transfers
    oxnas_dma_device_set(
        dma_in,
        OXNAS_DMA_TO_DEVICE,
        (char*)in_address,
        132 * sizeof(u32),
        &oxnas_dpe_rx_dma_settings,
        OXNAS_DMA_MODE_INC, 1);
        
    oxnas_dma_device_set(
        dma_out,
        OXNAS_DMA_FROM_DEVICE,
        (char*)out_address,
        132 * sizeof(u32),
        &oxnas_dpe_tx_dma_settings,
        OXNAS_DMA_MODE_INC, 1);

    // authenticate
    WRITEL(OX800IBW_STAT_AUTHENTICATED ,OX800IBW_STATUS);
    
    // toggle cleardown bit to start
    WRITEL(OX800DPE_CTL_ABORT  ,OX800DPE_CONTROL);
    WRITEL(0                   ,OX800DPE_CONTROL);
    
    // shouldn't be busy or full
    reg = READL( OX800DPE_STATUS );
    if (! (reg & OX800DPE_STAT_IDLE) )
        FAILED("not idle after abort toggle");
    if (reg & OX800DPE_STAT_TX_NOTEMPTY)
        FAILED("tx fifo not empty after abort toggle");
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx not empty after abort toggle");
    
    // setup for decryption 
    reg = OX800DPE_CTL_DIRECTION_ENC |
          OX800DPE_CTL_MODE_LRW_AES ;
    WRITEL(reg  ,OX800DPE_CONTROL);

    // key no 1
    WRITEL(be32_to_cpu( 0xd82a9134), OX800DPE_KEY00 );
    WRITEL(be32_to_cpu( 0xb26a5650), OX800DPE_KEY01 );
    WRITEL(be32_to_cpu( 0x30fe69e2), OX800DPE_KEY02 );
    WRITEL(be32_to_cpu( 0x377f9847), OX800DPE_KEY03 );
    
    // key no 2
    WRITEL(be32_to_cpu( 0xcdf90b16), OX800DPE_KEY10 );
    WRITEL(be32_to_cpu( 0x0c648fb6), OX800DPE_KEY11 );
    WRITEL(be32_to_cpu( 0xb00d0d1b), OX800DPE_KEY12 );
    WRITEL(be32_to_cpu( 0xae85871f), OX800DPE_KEY13 );

    // setup index
    WRITEL(0x0fffffff, OX800DPE_DATA_LRW0 );
    WRITEL(0x00000000, OX800DPE_DATA_LRW1 );

    /* wait until done */
    while(  !(OX800DPE_STAT_IDLE & READL( OX800DPE_STATUS )) );
    
    // start dma transfers
    oxnas_dma_start(dma_out);
    oxnas_dma_start(dma_in);
    
    /* wait until done */
    while(  oxnas_dma_is_active( dma_out ) );
    
    reg = READL( OX800DPE_STATUS );

    /* output should be empty */
    if ( (reg & OX800DPE_STAT_TX_NOTEMPTY) )
        FAILED("tx still full after fetching ");

    /* in empty */
    if (! (reg & OX800DPE_STAT_RX_SPACE) )
        FAILED("rx still full after encrypting data ");
    
    dma_unmap_single(0, in_address, 132 * sizeof(u32), DMA_TO_DEVICE);
    dma_unmap_single(0, out_address, 132 * sizeof(u32), DMA_FROM_DEVICE);

    /* check output */
    if ((data_out[128] != cpu_to_be32(0x76322183)) ||
        (data_out[129] != cpu_to_be32(0xed8ff182)) ||
        (data_out[130] != cpu_to_be32(0xf9596203)) ||
        (data_out[131] != cpu_to_be32(0x690e5e01)))
    {
        FAILED("encryption output incorrect");
        printk("%08x%08x%08x%08x\n",data_out[128],data_out[129],data_out[130],data_out[131]);
    }
        
    // free dmas
    oxnas_dma_free( dma_in ); 
    oxnas_dma_free( dma_out ); 
    
    kfree(data_in);
    kfree(data_out);

    return failed;

}


/*****************************************************************************/
#define NOCIPHERTESTS 21
static ox800dpe_test_t*  tests[NOCIPHERTESTS] = {
    /* ordinary AES tests */
    test1,
    test2,
    test3,
    test4,
    test5,
    test6,
    test7,
    
    /* CBC AES tests */
    test8,
    test9,
    test10,
    test11,
    test12,
    test13,
    
    /* LRW AES tests */
    test14,
    test15,
    test16,
    test17,
    test18,
    test19,
    test20,
    test21
    
} ;

 
static int __init ox800dpe_init(void)
{
    int i;
    int result;
    printk("*******************************************************************\n");
    printk("* CIPHER CORE TESTING START                                       *\n");
    printk("*******************************************************************\n");
    
        /* Enable the clock to the DPE block */
        writel(1UL << SYS_CTRL_CKEN_DPE_BIT, SYS_CTRL_CKEN_SET_CTRL);
    
        /* Bring out of reset */
        writel(1UL << SYS_CTRL_RSTEN_DPE_BIT, SYS_CTRL_RSTEN_CLR_CTRL);
        
    
    
    
    for (i = 0; i < NOCIPHERTESTS; ++i)
    {
        printk("\nTest %d start\n",i+1);
        result = (tests[i])();
        if (result)
            printk("Test %d failed with %d faults.\n",i+1,result);
        else
            printk("Test %d passed\n",i+1);
            
    }
    
    printk("*******************************************************************\n");
    printk("* CIPHER CORE TESTING END                                         *\n");
    printk("*******************************************************************\n");

	return 0;
}



/***************************************************************************/
module_init(ox800dpe_init);
