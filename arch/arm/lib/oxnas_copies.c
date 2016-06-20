/*
 * linux/arch/arm/lib/nas_copies.c
 *
 * Copyright (C) 2006 Oxford Semiconductor Ltd
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
#include <linux/compiler.h>
#include <linux/dma-mapping.h>
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <asm/scatterlist.h>
#include <asm/semaphore.h>
#include <asm/arch/dma.h>

static DECLARE_MUTEX(copy_mutex);
static __DECLARE_SEMAPHORE_GENERIC(callback_semaphore, 0);

static void dma_callback(
    oxnas_dma_channel_t         *channel,
    oxnas_callback_arg_t         arg,
    oxnas_dma_callback_status_t  error_code,
    u16                          checksum,
    int                          interrupt_count)
{
    up(&callback_semaphore);
}

unsigned long oxnas_copy_from_user(void *to, const void __user *from, unsigned long count)
{
    int pages_mapped, i;
    unsigned long transfered = 0;
    struct page *pages[2];
    oxnas_dma_channel_t *channel;
    unsigned long uaddr = (unsigned long)from;
    unsigned long end_page = (uaddr + count + PAGE_SIZE - 1) >> PAGE_SHIFT;
    unsigned long start_page = uaddr >> PAGE_SHIFT;
    int nr_pages = end_page - start_page;
//printk("F 0x%08lx 0x%08lx %lu -> %lu, %lu, %d\n", (unsigned long)to, (unsigned long)from, count, start_page, end_page, nr_pages);

    might_sleep();

    BUG_ON(nr_pages > 2);

    // Only support a single concurrent copy operation, as only using a single
    // DMA channel for now
    while (down_interruptible(&copy_mutex));

    // Get kernel mappings for the user pages
    down_read(&current->mm->mmap_sem);
    pages_mapped = get_user_pages(current, current->mm, uaddr, nr_pages, 0, 0, pages, NULL);
    up_read(&current->mm->mmap_sem);

    if (pages_mapped != nr_pages) {
        // Didn't get mappings for all pages requested, so release any we did get
        for (i=0; i < pages_mapped; ++i) {
            page_cache_release(pages[i]);
        }
        pages_mapped = 0;
    }

    if (pages_mapped) {
        int i;
        struct scatterlist sl;
        struct scatterlist gl[2];

        // Fill gathering DMA descriptors
        gl[0].page = pages[0]; 
        gl[0].offset = uaddr & ~PAGE_MASK;
        if (pages_mapped > 1) {
            gl[0].length = PAGE_SIZE - gl[0].offset;
            gl[1].offset = 0;
            gl[1].page = pages[1]; 
            gl[1].length = count - gl[0].length;
        } else {
            gl[0].length = count;
        }

        // Create DMA mappings for all the user pages
        for (i=0; i < pages_mapped; ++i) {
            gl[i].dma_address = dma_map_single(0, page_address(gl[i].page) + gl[i].offset, gl[i].length, DMA_TO_DEVICE);
        }

        // Create a DMA mapping for the kernel memory range
        sl.dma_address = dma_map_single(0, to, count, DMA_FROM_DEVICE);
        sl.length = count;

        // Allocate a DMA channel
        channel = oxnas_dma_request(1);
        BUG_ON(channel == OXNAS_DMA_CHANNEL_NUL);

        // Do DMA from user to kernel memory
        oxnas_dma_set_sg(
            channel,
            gl,
            pages_mapped,
            &sl,
            1,
            OXNAS_DMA_MODE_INC,
            OXNAS_DMA_MODE_INC,
            0);

        // Using notification callback
        oxnas_dma_set_callback(channel, dma_callback, OXNAS_DMA_CALLBACK_ARG_NUL);
        oxnas_dma_start(channel);

        // Sleep until transfer complete
        while (down_interruptible(&callback_semaphore));
        oxnas_dma_set_callback(channel, OXNAS_DMA_CALLBACK_NUL, OXNAS_DMA_CALLBACK_ARG_NUL);
        transfered += count;

        // Release the DMA channel
        oxnas_dma_free(channel);

        // Release kernel DMA mapping
        dma_unmap_single(0, sl.length, count, DMA_FROM_DEVICE);
        // Release user DMA mappings
        for (i=0; i < pages_mapped; ++i) {
            dma_unmap_single(0, gl[i].dma_address, gl[i].length, DMA_TO_DEVICE);
        }

        // Release user pages
        for (i=0; i < pages_mapped; ++i) {
            page_cache_release(pages[i]);
        }
    }

    up(&copy_mutex);

    return count - transfered;
}

//unsigned long oxnas_copy_to_user(void __user *to, const void *from, unsigned long count)
//{
//    int pages_mapped, i;
//    unsigned long transfered = 0;
//    struct page *pages[2];
//    oxnas_dma_channel_t *channel;
//    unsigned long uaddr = (unsigned long)to;
//    unsigned long end_page = (uaddr + count + PAGE_SIZE - 1) >> PAGE_SHIFT;
//    unsigned long start_page = uaddr >> PAGE_SHIFT;
//    int nr_pages = end_page - start_page;
////printk("T 0x%08lx 0x%08lx %lu -> %lu, %lu, %d\n", (unsigned long)to, (unsigned long)from, count, start_page, end_page, nr_pages);
//
//    might_sleep();
//
//    BUG_ON(nr_pages > 2);
//
//    // Only support a single concurrent copy operation, as only using a single
//    // DMA channel for now
//    while (down_interruptible(&copy_mutex));
//
//    // Get kernel mappings for the user pages
//    down_read(&current->mm->mmap_sem);
//    pages_mapped = get_user_pages(current, current->mm, uaddr, nr_pages, 1, 0, pages, NULL);
//    up_read(&current->mm->mmap_sem);
//
//    if (pages_mapped != nr_pages) {
//        // Didn't get mapping for all the pages we requested, so release any
//        // user page mappings we did get
//        for (i=0; i < pages_mapped; ++i) {
//            page_cache_release(pages[i]);
//        }
//        pages_mapped = 0;
//    }
//
//    if (pages_mapped) {
//        int i;
//        struct scatterlist gl;
//        struct scatterlist sl[2];
//
//        // Fill scattering DMA descriptors for writing to user space
//        sl[0].page = pages[0]; 
//        sl[0].offset = uaddr & ~PAGE_MASK;
//        if (pages_mapped > 1) {
//            sl[0].length = PAGE_SIZE - sl[0].offset;
//            sl[1].offset = 0;
//            sl[1].page = pages[1]; 
//            sl[1].length = count - sl[0].length;
//        } else {
//            sl[0].length = count;
//        }
//
//        // Create DMA mappings for all the user pages
//        for (i=0; i < pages_mapped; ++i) {
//            sl[i].__address = page_address(sl[i].page) + sl[i].offset;
//            sl[i].dma_address = dma_map_single(0, sl[i].__address, sl[i].length, DMA_FROM_DEVICE);
//        }
//
//        // Create a DMA mapping for the kernel memory range
//        gl.dma_address = dma_map_single(0, (void*)from, count, DMA_TO_DEVICE);
//        gl.length = count;
//
////        {
//////flush_cache_all();
////          // Do copy with CPU to test
////            const char* src = from;
//////printk("T Copying...\n");
////            for (i=0; i < pages_mapped; ++i) {
////                void* kadr = kmap(sl[i].page) + sl[i].offset;
////                memcpy(kadr, src, sl[i].length);
////                kunmap(sl[i].page);
////                src += sl[i].length;
////                transfered += sl[i].length;
////            }
////        }
//
//flush_cache_all();
//        // Allocate a DMA channel
//        channel = oxnas_dma_request(1);
//        BUG_ON(channel == OXNAS_DMA_CHANNEL_NUL);
//
//        // Do DMA from kernel to user memory
//        oxnas_dma_set_sg(
//            channel,
//            &gl,
//            1,
//            sl,
//            pages_mapped,
//            OXNAS_DMA_MODE_INC,
//            OXNAS_DMA_MODE_INC,
//            0);
//        oxnas_dma_start(channel);
//        while (oxnas_dma_is_active(channel));
//        transfered += count;
//
//        // Release the DMA channel
//        oxnas_dma_free(channel);
////flush_cache_all();
//
//        // Release kernel DMA mapping
//        dma_unmap_single(0, gl.dma_address, count, DMA_TO_DEVICE);
//        // Release user DMA mappings
//        for (i=0; i < pages_mapped; ++i) {
//            dma_unmap_single(0, sl[i].dma_address, sl[i].length, DMA_FROM_DEVICE);
//        }
//
//        // Release user pages
//        for (i=0; i < pages_mapped; ++i) {
//            SetPageDirty(pages[i]);
//            page_cache_release(pages[i]);
//        }
//    }
//
//    up(&copy_mutex);
//
//   return count - transfered;
//}

