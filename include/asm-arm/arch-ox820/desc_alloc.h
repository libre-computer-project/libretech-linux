/*
 * linux/include/asm-arm/arch-oxnas/dma.h
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
 *
 *
 * Here we partition the available internal SRAM between the GMAC and generic
 * DMA descriptors. This requires defining the gmac_dma_desc_t structure here,
 * rather than in its more natural position within gmac.h
 */
#if !defined(__DESC_ALLOC_H__)
#define __DESC_ALLOC_H__

#include <asm/arch/hardware.h>

// GMAC DMA in-memory descriptor structures
typedef struct gmac_dma_desc
{
    /** The encoded status field of the GMAC descriptor */
    u32 status;
    /** The encoded length field of GMAC descriptor */
    u32 length;
    /** Buffer 1 pointer field of GMAC descriptor */
    u32 buffer1;
    /** Buffer 2 pointer or next descriptor pointer field of GMAC descriptor */
    u32 buffer2;
} __attribute ((aligned(4),packed)) gmac_dma_desc_t;

#define NUM_GMAC_DMA_DESCRIPTORS CONFIG_ARCH_OXNAS_NUM_GMAC_DESCRIPTORS

#define GMAC_DESC_ALLOC_START       (SRAM_BASE)
#define GMAC_DESC_ALLOC_START_PA    (DESCRIPTORS_BASE_PA)
/** Must be modified to track gmac_dma_desc_t definition above */
#define GMAC_DESC_ALLOC_SIZE        ((NUM_GMAC_DMA_DESCRIPTORS) * (4*4))

#if ((GMAC_DESC_ALLOC_SIZE) > (DESCRIPTORS_SIZE))
#error "Too many GMAC descriptors - descriptor SRAM allocation exceeded"
#endif

#define DMA_DESC_ALLOC_START        ((GMAC_DESC_ALLOC_START)    + (GMAC_DESC_ALLOC_SIZE))
#define DMA_DESC_ALLOC_START_PA     ((GMAC_DESC_ALLOC_START_PA) + (GMAC_DESC_ALLOC_SIZE))
#define DMA_DESC_ALLOC_SIZE         ((DESCRIPTORS_SIZE) - (GMAC_DESC_ALLOC_SIZE))

#define descriptors_virt_to_phys(x) ((x) - (SRAM_BASE) + (DESCRIPTORS_BASE_PA))
#define descriptors_phys_to_virt(x) ((x) - (DESCRIPTORS_BASE_PA) + (SRAM_BASE))

#endif        //  #if !defined(__DESC_ALLOC_H__)

