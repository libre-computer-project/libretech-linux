/* linux/arch/arm/mach-oxnas/cipher.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/sysdev.h>
#include <linux/highmem.h>
#include <asm/semaphore.h>
#include <asm/arch/cipher.h>
#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <linux/dma-mapping.h>
#include <asm/arch/dma.h>
#include <asm-arm/page.h>
 

#if 0
#define DPRINTK(fmt, args...) printk(KERN_ERR "%s: " fmt, __FUNCTION__, ## args)
#define VPRINTK(fmt, args...) printk(KERN_ERR "%s: " fmt, __FUNCTION__, ## args)
#else
#define DPRINTK(fmt, args...)
#define VPRINTK(fmt, args...)
#endif

//#define CIPHER_USE_SG_DMA

/*****************************************************************************/
 
typedef struct ox800_aeslrw_driver ox800_aeslrw_driver_t;

struct ox800_aeslrw_driver {
    struct device dev;
    struct semaphore   core;
    int result;
    u8 cipher_key[OX800DPE_KEYSIZE];
    u8 tweak_key[OX800DPE_KEYSIZE];
};

static ox800_aeslrw_driver_t ox800_aeslrw_driver;
 

/*****************************************************************************/

/**
 * Sets the keys only if they have changed.
 * @param cipher_key 16 byte array that is the cipher key
 * @param tweak_key  16 byte array that is the I-Value tweak key
 */
static void ox800_aeslrw_setkeys(u8* cipher_key, u8* tweak_key)
{
    VPRINTK(KERN_INFO"\n");

    /*
     * changing the keys can take a long time as the core will
     * compute internal values based on the keys
     */
    if (memcmp(&(ox800_aeslrw_driver.cipher_key[0]), cipher_key, OX800DPE_KEYSIZE) ||
        memcmp(&(ox800_aeslrw_driver.tweak_key[0]),  tweak_key,  OX800DPE_KEYSIZE) )
    {
        u32* key;
        unsigned int  i;
    
        DPRINTK(KERN_INFO"cipher key ="); 
        for (i = 0; i < OX800DPE_KEYSIZE; ++i)
            DPRINTK("%02x", cipher_key[i]);
        DPRINTK("\n");
        DPRINTK(KERN_INFO"tweak key ="); 
        for (i = 0; i < OX800DPE_KEYSIZE; ++i)
            DPRINTK("%02x", tweak_key[i]);
        DPRINTK("\n");
            
        /* update stored values */
        memcpy(&(ox800_aeslrw_driver.cipher_key[0]), cipher_key, OX800DPE_KEYSIZE);
        memcpy(&(ox800_aeslrw_driver.tweak_key[0]),  tweak_key,  OX800DPE_KEYSIZE);
        
        /* update hardware values */
        key = (u32* )cipher_key;
        writel(key[0], OX800DPE_KEY00 );
        writel(key[1], OX800DPE_KEY01 );
        writel(key[2], OX800DPE_KEY02 );
        writel(key[3], OX800DPE_KEY03 );
    
        key = (u32* )tweak_key;
        writel(key[0], OX800DPE_KEY10 );
        writel(key[1], OX800DPE_KEY11 );
        writel(key[2], OX800DPE_KEY12 );
        writel(key[3], OX800DPE_KEY13 );
    }
}

/**
 * Generic LRW-AES en/decryption
 * @param encrypt non-zero to encrypt, zero to decrypt
 * @param in Source of data
 * @param out Location to place en/decrypted data
 * @param nents Number of entries in scatter list, in and out must have the same
 *              number of entries
 * @param iv 8 byte array containing the I-Value
 * @return error code or 0 for success
 */
static int ox800_aeslrw_gencrypt(  u8  encrypt,
                            struct scatterlist* in,
                            struct scatterlist* out,
                            unsigned int nents,
                            u8  iv[])
{
    oxnas_dma_channel_t* dma_in;
    oxnas_dma_channel_t* dma_out;
    struct scatterlist* out_;
    char same_buffer;
    int status = 0;

    /* get dma resources (non blocking) */
    dma_in = oxnas_dma_request(0);
    dma_out = oxnas_dma_request(0);
    
    VPRINTK("dma in %d out %d \n", 
        dma_in->channel_number_,  
        dma_out->channel_number_);  

    if ((dma_in) && (dma_out)) {
        u32 reg;
        
        // shouldn't be busy or full
        reg = readl( OX800DPE_STATUS );
        if (! (reg & OX800DPE_STAT_IDLE) )
            printk("not idle after abort toggle");
        if (reg & OX800DPE_STAT_TX_NOTEMPTY)
            printk("tx fifo not empty after abort toggle");
        if (! (reg & OX800DPE_STAT_RX_SPACE) )
            printk("rx not empty after abort toggle");
        
        /* check to see if the destination buffer is the same as the source */
        same_buffer = (in->page == out->page);
        
        /* map transfers */
        if (same_buffer) {
            dma_map_sg(NULL, in, nents, DMA_BIDIRECTIONAL);
            out_ = in;
        } else {
            /* map transfers */
            dma_map_sg(NULL, in, nents, DMA_TO_DEVICE);
            dma_map_sg(NULL, out, nents, DMA_FROM_DEVICE);
            out_ = out;
        }
#ifdef CIPHER_USE_SG_DMA        
        /* setup DMA transfers */ 
        oxnas_dma_device_set_sg(
            dma_in,
            OXNAS_DMA_TO_DEVICE,
            in,
            nents,
            &oxnas_dpe_rx_dma_settings,
            OXNAS_DMA_MODE_INC);
            
        oxnas_dma_device_set_sg(
            dma_out,
            OXNAS_DMA_FROM_DEVICE,
            out_,
            nents,
            &oxnas_dpe_tx_dma_settings,
            OXNAS_DMA_MODE_INC);

#else
        oxnas_dma_device_set(
            dma_in,
            OXNAS_DMA_TO_DEVICE,
            (unsigned char* )sg_dma_address(in),
            sg_dma_len(in),
            &oxnas_dpe_rx_dma_settings,
            OXNAS_DMA_MODE_INC,
            1 /*paused */ );
            
        oxnas_dma_device_set(
            dma_out,
            OXNAS_DMA_FROM_DEVICE,
            (unsigned char* )sg_dma_address(out_),
            sg_dma_len(out_),
            &oxnas_dpe_tx_dma_settings,
            OXNAS_DMA_MODE_INC,
            1 /*paused */ );
#endif

        /* set dma callbacks */
        oxnas_dma_set_callback(
            dma_in,
            OXNAS_DMA_CALLBACK_ARG_NUL,
            OXNAS_DMA_CALLBACK_ARG_NUL);
        
        oxnas_dma_set_callback(
            dma_out,
            OXNAS_DMA_CALLBACK_ARG_NUL,
            OXNAS_DMA_CALLBACK_ARG_NUL);
        
        
        /* set for AES LRW encryption or decryption */
        writel( (encrypt ? OX800DPE_CTL_DIRECTION_ENC : 0 ) |
               OX800DPE_CTL_MODE_LRW_AES, 
               OX800DPE_CONTROL);
        wmb();
        
        /* write in I-value */
        writel(*((u32* )&(iv[0])), OX800DPE_DATA_LRW0 );
        writel(*((u32* )&(iv[4])), OX800DPE_DATA_LRW1 );
        
        wmb();

        /* wait until done */
        while(  !(OX800DPE_STAT_IDLE & readl( OX800DPE_STATUS )) );
        
        /* start dma */
        oxnas_dma_start(dma_out);
        oxnas_dma_start(dma_in);
    
        /* wait (once for each channel) */
        while ( oxnas_dma_is_active( dma_out ) ||
                oxnas_dma_is_active( dma_in  ) )
        {
            schedule();
        }
        
        /* free any allocated dma channels */
        oxnas_dma_free( dma_in );
        oxnas_dma_free( dma_out );

        /* unmap transfers */
        if (same_buffer) {
            dma_unmap_sg(NULL, in, nents, DMA_BIDIRECTIONAL);
        } else {
            dma_unmap_sg(NULL, in, nents, DMA_TO_DEVICE);
            dma_unmap_sg(NULL, out, nents, DMA_FROM_DEVICE);
        }
        
        status = ox800_aeslrw_driver.result;
    } else {        
        /* free any allocated dma channels */
        if (dma_in) 
            oxnas_dma_free( dma_in );
        if (dma_out)
            oxnas_dma_free( dma_out );
        status = -EBUSY;        
    }
    /* return an indication of success */
    return status;
}

/**
 * Performs LRW-AES encryption.
 * @param in Source of data
 * @param out Location to place encrypted data
 * @param nents Number of entries in scatter list, in and out must have the same
 *              number of entries
 * @param iv I-Value
 * @param cipher_key 16 byte array that is the cipher key
 * @param tweak_key  16 byte array that is the I-Value tweak key
 * @return error code or 0 for success
 */
int ox800_aeslrw_encrypt(   struct scatterlist* in,
                            struct scatterlist* out,
                            unsigned int nents,
                            u8* iv,
                            u8* cipher_key,
                            u8* tweak_key)
{
    int localresult;
    
    VPRINTK(KERN_INFO"in %p, out %p, nents %d, iv %08x%08x, ckey %p, tkey %p\n",
        in, out, nents, *((u32* )(&iv[4])), *((u32* )(&iv[0])), cipher_key, tweak_key );

    /* get cipher core */
    while( down_interruptible(&ox800_aeslrw_driver.core) ) ;

    VPRINTK(KERN_INFO"got core\n");

    ox800_aeslrw_setkeys(cipher_key, tweak_key);
    localresult = ox800_aeslrw_gencrypt( 1, in, out, nents, iv);
    
    up(&ox800_aeslrw_driver.core);
    VPRINTK(KERN_INFO"released\n");
    
    return localresult;
}

/**
 * Performs LRW-AES decryption.
 * @param in Source of data
 * @param out Location to place decrypted data
 * @param nents Number of entries in scatter list, in and out must have the same
 *              number of entries
 * @param iv I-Value
 * @param cipher_key 16 byte array that is the cipher key
 * @param tweak_key  16 byte array that is the I-Value tweak key
 * @return error code or 0 for success
 */
int ox800_aeslrw_decrypt(   struct scatterlist* in,
                            struct scatterlist* out,
                            unsigned int nents,
                            u8* iv,
                            u8* cipher_key,
                            u8* tweak_key)
{
    int localresult;
    
    VPRINTK(KERN_INFO"in %p, out %p, nents %d, iv %08x%08x, ckey %p, tkey%p\n",
        in, out, nents, *((u32* )(&iv[4])), *((u32* )(&iv[0])), cipher_key, tweak_key );

    /* get cipher core */
    while( down_interruptible(&ox800_aeslrw_driver.core) ) ;

    VPRINTK(KERN_INFO"got core\n");

    ox800_aeslrw_setkeys(cipher_key, tweak_key);
    localresult = ox800_aeslrw_gencrypt( 0, in, out, nents, iv);
    
    up(&ox800_aeslrw_driver.core);
    VPRINTK(KERN_INFO"released core \n");
    
    return localresult;
}

/** 
 * module initialisation
 * @return success is 0
 */
static int __init ox800_aeslrw_init( void )
{
    VPRINTK(KERN_INFO"\n");
    
    /* Enable the clock to the DPE block */
    writel(1UL << SYS_CTRL_CKEN_DPE_BIT, SYS_CTRL_CKEN_SET_CTRL);

    /* Bring out of reset */
    writel(1UL << SYS_CTRL_RSTEN_DPE_BIT, SYS_CTRL_RSTEN_CLR_CTRL);
    
    /* initialise in unlocked state */
    init_MUTEX(&ox800_aeslrw_driver.core);
    
    return 0;
}

/** 
 * module cleanup
 */
static void __exit ox800_aeslrw_exit( void )
{
}

/** 
 * macros to register intiialisation and exit functions with kernal
 */
module_init(ox800_aeslrw_init);
module_exit(ox800_aeslrw_exit);
