/*
 * linux/arch/arm/mach-oxnas/gmac.c
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

#include <linux/crc32.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/in.h>
#include <net/ip.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <asm/io.h>
#include <asm/plat-oxnas/hardware.h>
#include <asm/arch/irqs.h>
#include <linux/platform_device.h>

#ifdef CONFIG_LEON_COPRO
#include <linux/firmware.h>
#endif // CONFIG_LEON_COPRO

//#define GMAC_DEBUG
#undef GMAC_DEBUG

#include "gmac.h"
#include "gmac_ethtool.h"
#include "gmac_phy.h"
#include "gmac_desc.h"
#include "gmac_reg.h"

//#define DUMP_REGS_ON_GMAC_UP

#define MAX_GMAC_UNITS 1

#define ALLOW_AUTONEG

//#define ALLOW_OX800_1000M

//#define SUPPORT_IPV6

#ifdef CONFIG_LEON_COPRO
//#define TEST_COPRO
#define COPRO_RX_MITIGATION 0   /* No Rx mitigation in CoPro */
#define COPRO_RX_MITIGATION_FRAMES 5
#define COPRO_RX_MITIGATION_USECS 500

#define COPRO_TX_QUEUE_NUM_ENTRIES  4
#define COPRO_CMD_QUEUE_NUM_ENTRIES 6
#endif // CONFIG_LEON_COPRO

#ifdef CONFIG_LEON_OFFLOAD_TX
#define NUM_RX_DMA_DESCRIPTORS  NUM_GMAC_DMA_DESCRIPTORS
#define NUM_TX_DMA_DESCRIPTORS  0
#else
#define NUM_RX_DMA_DESCRIPTORS (NUM_GMAC_DMA_DESCRIPTORS / 2)
#define NUM_TX_DMA_DESCRIPTORS (NUM_GMAC_DMA_DESCRIPTORS - NUM_RX_DMA_DESCRIPTORS)
#endif // CONFIG_LEON_OFFLOAD_TX

#if (((NUM_RX_DMA_DESCRIPTORS) + (NUM_TX_DMA_DESCRIPTORS)) > (NUM_GMAC_DMA_DESCRIPTORS))
#error "GMAC TX+RX descriptors exceed allocation"
#endif

#define DESC_SINCE_REFILL_LIMIT ((NUM_RX_DMA_DESCRIPTORS) / 4)

static const u32 MAC_BASE_OFFSET = 0x0000;
static const u32 DMA_BASE_OFFSET = 0x1000;

static const int MIN_PACKET_SIZE = 68;
static const int NORMAL_PACKET_SIZE = 1500;
static const int MAX_JUMBO = 9000;

#define RX_BUFFER_SIZE 796	// Must be multiple of 4, If not defined will size buffer to hold a single MTU-sized packet

#ifdef CONFIG_OXNAS_VERSION_0X800
static const int EXTRA_RX_SKB_SPACE = 24;	// Has extra 2 bytes of Rx payload csum
#else // CONFIG_OXNAS_VERSION_0X800
static const int EXTRA_RX_SKB_SPACE = 22;	// Ethernet header 14, VLAN 4, CRC 4
#endif // CONFIG_OXNAS_VERSION_0X800

// The amount of header to copy from a receive packet into the skb buffer
static const int GMAC_HLEN = 66;

#define GMAC_ALLOC_ORDER 0
static const int GMAC_ALLOC_SIZE = ((1 << GMAC_ALLOC_ORDER) * PAGE_SIZE);

static const u32 AUTO_NEGOTIATION_WAIT_TIMEOUT_MS = 5000;

static const u32 NAPI_POLL_WEIGHT = 64;
static const u32 NAPI_OOM_POLL_INTERVAL_MS = 50;

static const int WATCHDOG_TIMER_INTERVAL = 500*HZ/1000;

#define AUTO_NEG_MS_WAIT 500
static const int AUTO_NEG_INTERVAL = (AUTO_NEG_MS_WAIT)*HZ/1000;
static const int START_RESET_INTERVAL = 50*HZ/1000;
static const int RESET_INTERVAL = 10*HZ/1000;

static const int GMAC_RESET_TIMEOUT_MS = 10000;

static int debug = 0;

MODULE_AUTHOR("Brian Clarke (Oxford Semiconductor Ltd)");
MODULE_DESCRIPTION("GMAC Network Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("v2.0");

/* Ethernet MAC adr to assign to interface */
static int mac_adr[] = { 0x00, 0x30, 0xe0, 0x00, 0x00, 0x00 };
module_param_array(mac_adr, int, NULL, S_IRUGO);

/* PHY type kernel cmdline options */
static int phy_is_rgmii = 0;
module_param(phy_is_rgmii, int, S_IRUGO);

static struct platform_driver plat_driver = {
	.driver	= {
		.name	= "gmac",
	},
};

#ifdef DUMP_REGS_ON_GMAC_UP
static void dump_mac_regs(u32 macBase, u32 dmaBase)
{
    int n = 0;

    for (n=0; n<0x60; n+=4) {
        printk(KERN_INFO "MAC Register %08x (%08x) = %08x\n", n, macBase+n, readl(macBase+n));
    }

    for (n=0; n<0x60; n+=4) {
        printk(KERN_INFO "DMA Register %08x (%08x) = %08x\n", n, dmaBase+n, readl(dmaBase+n));
    }
}
#endif // DUMP_REGS_ON_GMAC_UP

#ifdef CONFIG_LEON_COPRO
static struct semaphore copro_update_semaphore;

static void copro_update_callback(volatile gmac_cmd_que_ent_t* entry)
{
    up(&copro_update_semaphore);
}

static struct semaphore copro_start_semaphore;

static void copro_start_callback(volatile gmac_cmd_que_ent_t* entry)
{
    up(&copro_start_semaphore);
}

static struct semaphore copro_rx_enable_semaphore;

static void copro_rx_enable_callback(volatile gmac_cmd_que_ent_t* entry)
{
    up(&copro_rx_enable_semaphore);
}
#endif // CONFIG_LEON_COPRO

static void gmac_int_en_set(
    gmac_priv_t *priv,
    u32          mask)
{
    unsigned long irq_flags = 0;

#ifdef CONFIG_LEON_COPRO
    int cmd_queue_result = -1;
    while (cmd_queue_result) {
        spin_lock_irqsave(&priv->cmd_que_lock_, irq_flags);
        cmd_queue_result = cmd_que_queue_cmd(&priv->cmd_queue_, GMAC_CMD_INT_EN_SET, mask, 0);
        spin_unlock_irqrestore(&priv->cmd_que_lock_, irq_flags);
    }

    // Interrupt the CoPro so it sees the new command
    writel(1UL << COPRO_SEM_INT_CMD, SYS_CTRL_SEMA_SET_CTRL);
#else
    spin_lock_irqsave(&priv->cmd_que_lock_, irq_flags);
    dma_reg_set_mask(priv, DMA_INT_ENABLE_REG, mask);
    spin_unlock_irqrestore(&priv->cmd_que_lock_, irq_flags);
#endif // CONFIG_LEON_COPRO
}

#ifdef CONFIG_LEON_COPRO
static struct semaphore copro_int_clr_semaphore;
static unsigned long    copro_int_clr_return;

static void copro_int_clr_callback(volatile gmac_cmd_que_ent_t* entry)
{
    copro_int_clr_return = entry->operand_;
    up(&copro_int_clr_semaphore);
}
#endif // CONFIG_LEON_COPRO

#ifdef CONFIG_LEON_COPRO
static void gmac_copro_int_en_clr(
    gmac_priv_t *priv,
    u32          mask,
    u32         *new_value)
{
    unsigned long irq_flags = 0;

    int cmd_queue_result = -1;
    while (cmd_queue_result) {
        spin_lock_irqsave(&priv->cmd_que_lock_, irq_flags);
        cmd_queue_result = cmd_que_queue_cmd(&priv->cmd_queue_, GMAC_CMD_INT_EN_CLR, mask, copro_int_clr_callback);
        spin_unlock_irqrestore(&priv->cmd_que_lock_, irq_flags);
    }

    // Interrupt the CoPro so it sees the new command
    writel(1UL << COPRO_SEM_INT_CMD, SYS_CTRL_SEMA_SET_CTRL);

    if (new_value) {
        down_interruptible(&copro_int_clr_semaphore);
        *new_value = copro_int_clr_return;
    }
}
#else // CONFIG_LEON_COPRO
static void gmac_int_en_clr(
    gmac_priv_t *priv,
    u32          mask,
    u32         *new_value,
    int          in_irq)
{
    unsigned long temp;
    unsigned long irq_flags = 0;

    if (in_irq)
        spin_lock(&priv->cmd_que_lock_);
    else
        spin_lock_irqsave(&priv->cmd_que_lock_, irq_flags);

    temp = dma_reg_clear_mask(priv, DMA_INT_ENABLE_REG, mask);

    if (in_irq)
        spin_unlock(&priv->cmd_que_lock_);
    else
        spin_unlock_irqrestore(&priv->cmd_que_lock_, irq_flags);

    if (new_value) {
        *new_value = temp;
    }
}
#endif // CONFIG_LEON_COPRO

/**
 * May be invoked from either ISR or process context
 */
static void change_rx_enable(
    gmac_priv_t *priv,
    u32          start,
    int          waitForAck,
    int          in_irq)
{
#ifdef CONFIG_LEON_COPRO
	unsigned long irq_flags = 0;
	int cmd_queue_result = -1;

	while (cmd_queue_result) {
		if (in_irq)
			spin_lock(&priv->cmd_que_lock_);
		else
			spin_lock_irqsave(&priv->cmd_que_lock_, irq_flags);

		// Request the CoPro to start/stop GMAC receiver
		cmd_queue_result =
			cmd_que_queue_cmd(&priv->cmd_queue_,
							  GMAC_CMD_CHANGE_RX_ENABLE,
							  start,
							  waitForAck ? copro_rx_enable_callback : 0);

		if (in_irq)
			spin_unlock(&priv->cmd_que_lock_);
		else
			spin_unlock_irqrestore(&priv->cmd_que_lock_, irq_flags);
	}

	// Interrupt the CoPro so it sees the new command
	writel(1UL << COPRO_SEM_INT_CMD, SYS_CTRL_SEMA_SET_CTRL);

	if (waitForAck) {
		// Wait until the CoPro acknowledges that the receiver has been stopped
		if (in_irq) {
			while (down_trylock(&copro_rx_enable_semaphore)) {
				udelay(100);
			}
		} else {
			down_interruptible(&copro_rx_enable_semaphore);
		}
	}
#else // CONFIG_LEON_COPRO
    start ? dma_reg_set_mask(priv, DMA_OP_MODE_REG, 1UL << DMA_OP_MODE_SR_BIT) :
            dma_reg_clear_mask(priv, DMA_OP_MODE_REG, 1UL << DMA_OP_MODE_SR_BIT);
#endif // CONFIG_LEON_COPRO
}

/**
 * Invoked from the watchdog timer action routine which runs as softirq, so
 * must disable interrupts when obtaining locks
 */
static void change_gig_mode(gmac_priv_t *priv)
{
	unsigned int is_gig = priv->mii.using_1000;

#ifdef CONFIG_LEON_COPRO
    unsigned long irq_flags = 0;
    int cmd_queue_result = -1;

    while (cmd_queue_result) {
        spin_lock_irqsave(&priv->cmd_que_lock_, irq_flags);
        // Request the CoPro to change gigabit mode
        cmd_queue_result = cmd_que_queue_cmd(&priv->cmd_queue_, GMAC_CMD_CHANGE_GIG_MODE, is_gig, 0);
        spin_unlock_irqrestore(&priv->cmd_que_lock_, irq_flags);
    }

    // Interrupt the CoPro so it sees the new command
    writel(1UL << COPRO_SEM_INT_CMD, SYS_CTRL_SEMA_SET_CTRL);
#else // CONFIG_LEON_COPRO
	static const int MAX_TRIES = 1000;
	int tries = 0;

    // Mask to extract the transmit status field from the status register
    u32 ts_mask = ((1UL << DMA_STATUS_TS_NUM_BITS) - 1) << DMA_STATUS_TS_BIT;

    // Must stop transmission in order to change store&forward mode
    dma_reg_clear_mask(priv, DMA_OP_MODE_REG, (1UL << DMA_OP_MODE_ST_BIT));

    // Transmission only stops after current Tx frame has completed trans-
    // mission, so wait for the Tx state machine to enter the stopped state
    while ((dma_reg_read(priv, DMA_STATUS_REG) & ts_mask) != (DMA_STATUS_TS_STOPPED << DMA_STATUS_TS_BIT)) {
		mdelay(1);
		if (unlikely(++tries == MAX_TRIES)) {
			break;
		}
	}

	if (unlikely(tries == MAX_TRIES)) {
		printk(KERN_WARNING "Timed out of wait for Tx to stop\n");
	}

    if (is_gig) {
        mac_reg_clear_mask(priv, MAC_CONFIG_REG, (1UL << MAC_CONFIG_PS_BIT));
#ifdef CONFIG_OXNAS_VERSION_0X800
        // In gigabit mode the OX800 cannot support the required data rate to
        // the GMAC, so must use store&forward and OX800 doesn't support Tx
		// checksumming in the GMAC
        dma_reg_set_mask(priv, DMA_OP_MODE_REG, (1UL << DMA_OP_MODE_SF_BIT));
#endif // CONFIG_OXNAS_VERSION_0X800
    } else {
        mac_reg_set_mask(priv, MAC_CONFIG_REG, (1UL << MAC_CONFIG_PS_BIT));
#ifdef CONFIG_OXNAS_VERSION_0X800
        dma_reg_clear_mask(priv, DMA_OP_MODE_REG, (1UL << DMA_OP_MODE_SF_BIT));
#endif // CONFIG_OXNAS_VERSION_0X800
    }

    // Re-start transmission after store&forward change applied
    dma_reg_set_mask(priv, DMA_OP_MODE_REG, (1UL << DMA_OP_MODE_ST_BIT));
#endif // CONFIG_LEON_COPRO
}

/**
 * Invoked from the watchdog timer action routine which runs as softirq, so
 * must disable interrupts when obtaining locks
 */
static void change_pause_mode(gmac_priv_t *priv)
{
#ifdef CONFIG_LEON_COPRO
	unsigned int enable_pause = priv->mii.using_pause;
    unsigned long irq_flags = 0;
    int cmd_queue_result = -1;

    while (cmd_queue_result) {
        spin_lock_irqsave(&priv->cmd_que_lock_, irq_flags);
        // Request the CoPro to change pause mode
        cmd_queue_result = cmd_que_queue_cmd(&priv->cmd_queue_, GMAC_CMD_CHANGE_PAUSE_MODE, enable_pause, 0);
        spin_unlock_irqrestore(&priv->cmd_que_lock_, irq_flags);
    }

    // Interrupt the CoPro so it sees the new command
    writel(1UL << COPRO_SEM_INT_CMD, SYS_CTRL_SEMA_SET_CTRL);
#else // CONFIG_LEON_COPRO
#ifndef CONFIG_OXNAS_VERSION_0X800
	unsigned int enable_pause = priv->mii.using_pause;

    if (enable_pause) {
        mac_reg_set_mask(priv, MAC_FLOW_CNTL_REG, (1UL << MAC_FLOW_CNTL_TFE_BIT));
    } else {
        mac_reg_clear_mask(priv, MAC_FLOW_CNTL_REG, (1UL << MAC_FLOW_CNTL_TFE_BIT));
    }
#endif // !CONFIG_OXNAS_VERSION_0X800
#endif // CONFIG_LEON_COPRO
}

static void refill_rx_ring(struct net_device *dev)
{
    gmac_priv_t *priv = (gmac_priv_t*)netdev_priv(dev);
	int filled = 0;

	if (unlikely(priv->rx_buffers_per_page)) {
		// Receive into pages
		struct page *page = 0;
		int offset = 0;
		dma_addr_t phys_adr = 0;

		// While there are empty RX descriptor ring slots
		while (1) {
			int available;
			int desc;
			rx_frag_info_t frag_info;

			// Have we run out of space in the current page?
			if (offset + NET_IP_ALIGN + priv->rx_buffer_size_ > GMAC_ALLOC_SIZE) {
				page = 0;
				offset = 0;
			}

			if (!page) {
				// Start a new page
				available = available_for_write(&priv->rx_gmac_desc_list_info);
				if (available < priv->rx_buffers_per_page) {
					break;
				}

				// Allocate a page to hold a received packet
				page = alloc_pages(GFP_ATOMIC, GMAC_ALLOC_ORDER);
				if (unlikely(page == NULL)) {
					printk(KERN_WARNING "refill_rx_ring() Could not alloc page\n");
					break;
				}

				// Get a consistent DMA mapping for the entire page that will be
				// DMAed to - causing an invalidation of any entries in the CPU's
				// cache covering the memory region
				phys_adr = dma_map_page(0, page, 0, GMAC_ALLOC_SIZE, DMA_FROM_DEVICE);
				BUG_ON(dma_mapping_error(phys_adr));
			} else {
				// Using the current page again
				get_page(page);
			}

			// Ensure IP header is quad aligned
			offset += NET_IP_ALIGN;
			frag_info.page = page;
			frag_info.length = priv->rx_buffer_size_;
			frag_info.phys_adr = phys_adr + offset;

			// Try to associate a descriptor with the fragment info
			desc = set_rx_descriptor(priv, &frag_info);
			if (desc >= 0) {
				filled = 1;
			} else {
				// Failed to associate the descriptor, so release the DMA mapping
				// for the socket buffer
				dma_unmap_page(0, frag_info.phys_adr, frag_info.length, DMA_FROM_DEVICE);

				// No more RX descriptor ring entries to refill
				break;
			}

			// Account for the space used in the current page
			offset += frag_info.length;

			// Start next packet on a cacheline boundary
			offset = SKB_DATA_ALIGN(offset);
		}
	} else {
		// Preallocate MTU-sized SKBs
		while (1) {
			struct sk_buff *skb;
			rx_frag_info_t frag_info;
			int desc;

			if (!available_for_write(&priv->rx_gmac_desc_list_info)) {
				break;
			}

			// Allocate a new skb for the descriptor ring which is large enough
			// for any packet received from the link
			skb = dev_alloc_skb(priv->rx_buffer_size_ + NET_IP_ALIGN);
			if (!skb) {
				// Can't refill any more RX descriptor ring entries
				break;
			} else {
				// Despite what the comments in the original code from Synopsys
				// claimed, the GMAC DMA can cope with non-quad aligned buffers
				// - it will always perform quad transfers but zero/ignore the
				// unwanted bytes.
				skb_reserve(skb, NET_IP_ALIGN);
			}

			// Get a consistent DMA mapping for the memory to be DMAed to
			// causing invalidation of any entries in the CPU's cache covering
			// the memory region
			frag_info.page = (struct page*)skb;
			frag_info.length = skb_tailroom(skb);
			frag_info.phys_adr = dma_map_single(0, skb->tail, frag_info.length, DMA_FROM_DEVICE);
			BUG_ON(dma_mapping_error(frag_info.phys_adr));

			// Associate the skb with the descriptor
			desc = set_rx_descriptor(priv, &frag_info);
			if (desc >= 0) {
				filled = 1;
			} else {
				// No, so release the DMA mapping for the socket buffer
				dma_unmap_single(0, frag_info.phys_adr, frag_info.length, DMA_FROM_DEVICE);

				// Free the socket buffer
				dev_kfree_skb(skb);

				// No more RX descriptor ring entries to refill
				break;
			}
		}
	}

#ifndef CONFIG_OXNAS_VERSION_0X800
	if (likely(filled)) {
		// Issue a RX poll demand to restart RX descriptor processing and DFF
		// mode does not automatically restart descriptor processing after a
		// descriptor unavailable event
		dma_reg_write(priv, DMA_RX_POLL_REG, 0);
	}
#endif // !CONFIG_OXNAS_VERSION_0X800
}

static inline void set_phy_type_rgmii(void)
{
#ifndef CONFIG_OXNAS_VERSION_0X800
	// Use sysctrl to switch MAC link lines into either (G)MII or RGMII mode
	u32 reg_contents = readl(SYS_CTRL_GMAC_CTRL);
	reg_contents |= (1UL << SYS_CTRL_GMAC_RGMII);
    writel(reg_contents, SYS_CTRL_RSTEN_SET_CTRL);
#endif // !CONFIG_OXNAS_VERSION_0X800
}

static void initialise_phy(gmac_priv_t* priv)
{
	switch (priv->phy_type) {
		case PHY_TYPE_VITESSE_VSC8201XVZ:
			{
				// Allow s/w to override mode/duplex pin settings
				u32 acsr = priv->mii.mdio_read(priv->netdev, priv->mii.phy_id, VSC8201_MII_ACSR);

				printk(KERN_INFO "%s: PHY is Vitesse VSC8201XVZ\n", priv->netdev->name);
				acsr |= (1UL << VSC8201_MII_ACSR_MDPPS_BIT);
				priv->mii.mdio_write(priv->netdev, priv->mii.phy_id, VSC8201_MII_ACSR, acsr);
			}
			break;
		case PHY_TYPE_REALTEK_RTL8211BGR:
			printk(KERN_INFO "%s: PHY is Realtek RTL8211BGR\n", priv->netdev->name);
			set_phy_type_rgmii();
			break;
		case PHY_TYPE_LSI_ET1011C:
		case PHY_TYPE_LSI_ET1011C2:
			{
				u32 phy_reg;

				printk(KERN_INFO "%s: PHY is LSI ET1011C\n", priv->netdev->name);

				// Configure clocks
				phy_reg = priv->mii.mdio_read(priv->netdev, priv->mii.phy_id, ET1011C_MII_CONFIG);
				phy_reg &= ~(((1UL << ET1011C_MII_CONFIG_IFMODESEL_NUM_BITS) - 1) << ET1011C_MII_CONFIG_IFMODESEL);
				phy_reg |= (ET1011C_MII_CONFIG_IFMODESEL_GMII_MII << ET1011C_MII_CONFIG_IFMODESEL);
				phy_reg |= ((1UL << ET1011C_MII_CONFIG_SYSCLKEN) |
                              (1UL << ET1011C_MII_CONFIG_TXCLKEN) |
							   (1UL << ET1011C_MII_CONFIG_TBI_RATESEL) |
							   (1UL << ET1011C_MII_CONFIG_CRS_TX_EN));
				priv->mii.mdio_write(priv->netdev, priv->mii.phy_id, ET1011C_MII_CONFIG, phy_reg);

				// Enable Tx/Rx LED
				phy_reg = priv->mii.mdio_read(priv->netdev, priv->mii.phy_id, ET1011C_MII_LED2);
				phy_reg &= ~(((1UL << ET1011C_MII_LED2_LED_NUM_BITS) - 1) << ET1011C_MII_LED2_LED_TXRX);
				phy_reg |= (ET1011C_MII_LED2_LED_TXRX_ACTIVITY << ET1011C_MII_LED2_LED_TXRX);
				priv->mii.mdio_write(priv->netdev, priv->mii.phy_id, ET1011C_MII_LED2, phy_reg);
			}
			break;
		case PHY_TYPE_ICPLUS_IP1001:
			printk(KERN_INFO "%s: PHY is ICPlus 1001\n", priv->netdev->name);
			break;
	}
}

static void do_pre_reset_actions(gmac_priv_t* priv)
{
	switch (priv->phy_type) {
		case PHY_TYPE_LSI_ET1011C:
		case PHY_TYPE_LSI_ET1011C2:
			{
				u32 phy_reg;

				printk(KERN_INFO "%s: LSI ET1011C PHY no Rx clk workaround start\n", priv->netdev->name);

				// Enable all digital loopback
				phy_reg = priv->mii.mdio_read(priv->netdev, priv->mii.phy_id, ET1011C_MII_LOOPBACK_CNTL);
				phy_reg &= ~(1UL << ET1011C_MII_LOOPBACK_MII_LOOPBACK);
				phy_reg |=  (1UL << ET1011C_MII_LOOPBACK_DIGITAL_LOOPBACK);
				priv->mii.mdio_write(priv->netdev, priv->mii.phy_id, ET1011C_MII_LOOPBACK_CNTL, phy_reg);

				// Disable auto-negotiation and enable loopback
				phy_reg = priv->mii.mdio_read(priv->netdev, priv->mii.phy_id, MII_BMCR);
				phy_reg &= ~BMCR_ANENABLE;
				phy_reg |=  BMCR_LOOPBACK;
				priv->mii.mdio_write(priv->netdev, priv->mii.phy_id, MII_BMCR, phy_reg);
			}
			break;
	}
}

static void do_post_reset_actions(gmac_priv_t* priv)
{
	switch (priv->phy_type) {
		case PHY_TYPE_LSI_ET1011C:
		case PHY_TYPE_LSI_ET1011C2:
			{
				u32 phy_reg;

				printk(KERN_INFO "%s: LSI ET1011C PHY no Rx clk workaround end\n", priv->netdev->name);

				// Disable loopback and enable auto-negotiation
				phy_reg = priv->mii.mdio_read(priv->netdev, priv->mii.phy_id, MII_BMCR);
				phy_reg |=  BMCR_ANENABLE;
				phy_reg &= ~BMCR_LOOPBACK;
				priv->mii.mdio_write(priv->netdev, priv->mii.phy_id, MII_BMCR, phy_reg);
			}
			break;
	}
}

static struct kobj_type ktype_gmac_link_state = {
	.release = 0,
	.sysfs_ops = 0,
	.default_attrs = 0,
};

static int gmac_link_state_hotplug_filter(struct kset* kset, struct kobject* kobj) {
	return get_ktype(kobj) == &ktype_gmac_link_state;
}

static const char* gmac_link_state_hotplug_name(struct kset* kset, struct kobject* kobj) {
	return "gmac_link_state";
}

static struct kset_uevent_ops gmac_link_state_uevent_ops = {
	.filter = gmac_link_state_hotplug_filter,
	.name   = gmac_link_state_hotplug_name,
	.uevent = NULL,
};

static int gmac_link_state_init_sysfs(gmac_priv_t* priv)
{
	int err = 0;

	/* Prepare the sysfs interface for use */
	kobject_set_name(&priv->link_state_kset.kobj, "gmac_link_state");
	priv->link_state_kset.ktype = &ktype_gmac_link_state;

	err = subsystem_register(&priv->link_state_kset);
	if (err)
		return err;

	/* Setup hotplugging */
	priv->link_state_kset.uevent_ops = &gmac_link_state_uevent_ops;

	/* Setup the heirarchy, the name will be set on detection */
	kobject_init(&priv->link_state_kobject);
	priv->link_state_kobject.kset = kset_get(&priv->link_state_kset);
	priv->link_state_kobject.parent = &priv->link_state_kset.kobj;

	/* Build the sysfs entry */
	kobject_set_name(&priv->link_state_kobject, "gmac_link_state-1");
	return kobject_add(&priv->link_state_kobject);
}

static void work_handler(struct work_struct *ws) {
	gmac_priv_t *priv = container_of(ws, gmac_priv_t, link_state_change_work);

	kobject_uevent(&priv->link_state_kobject, priv->link_state ? KOBJ_ONLINE : KOBJ_OFFLINE);
}

static void link_state_change_callback(
	int   link_state,
	void *arg)
{
	gmac_priv_t* priv = (gmac_priv_t*)arg;

	priv->link_state = link_state;
	schedule_work(&priv->link_state_change_work);
}

static void start_watchdog_timer(gmac_priv_t* priv)
{
    priv->watchdog_timer.expires = jiffies + WATCHDOG_TIMER_INTERVAL;
    priv->watchdog_timer_shutdown = 0;
    mod_timer(&priv->watchdog_timer, priv->watchdog_timer.expires);
}

static void delete_watchdog_timer(gmac_priv_t* priv)
{
    // Ensure link/PHY state watchdog timer won't be invoked again
    priv->watchdog_timer_shutdown = 1;
    del_timer_sync(&priv->watchdog_timer);
}

static inline int is_auto_negotiation_in_progress(gmac_priv_t* priv)
{
    return !(phy_read(priv->netdev, priv->phy_addr, MII_BMSR) & BMSR_ANEGCOMPLETE);
}

static void watchdog_timer_action(unsigned long arg)
{
    typedef enum watchdog_state {
        WDS_IDLE,
        WDS_RESETTING,
        WDS_NEGOTIATING
    } watchdog_state_t;

    static int state = WDS_IDLE;

    gmac_priv_t* priv = (gmac_priv_t*)arg;
    unsigned long new_timeout = jiffies + WATCHDOG_TIMER_INTERVAL;
#ifndef ARMULATING
    int ready;
    int duplex_changed;
    int gigabit_changed;
    int pause_changed;

	// Interpret the PHY/link state.
	if (priv->phy_force_negotiation || (state == WDS_RESETTING)) {
		mii_check_link(&priv->mii);
		ready = 0;
	} else {
		duplex_changed = mii_check_media_ex(&priv->mii, 1, priv->mii_init_media, &gigabit_changed, &pause_changed, link_state_change_callback, priv);
		priv->mii_init_media = 0;
		ready = netif_carrier_ok(priv->netdev);
	}

    if (!ready) {
        if (priv->phy_force_negotiation) {
            if (netif_carrier_ok(priv->netdev)) {
                state = WDS_RESETTING;
            } else {
                state = WDS_IDLE;
            }

            priv->phy_force_negotiation = 0;
        }

        // May be a good idea to restart everything here, in an attempt to clear
        // out any fault conditions
        if ((state == WDS_NEGOTIATING) && is_auto_negotiation_in_progress(priv)) {
            new_timeout = jiffies + AUTO_NEG_INTERVAL;
        } else {
            switch (state) {
                case WDS_IDLE:
                    // Reset the PHY to get it into a known state
                    start_phy_reset(priv);
                    new_timeout = jiffies + START_RESET_INTERVAL;
                    state = WDS_RESETTING;
                    break;
                case WDS_RESETTING:
                    if (!is_phy_reset_complete(priv)) {
                        new_timeout = jiffies + RESET_INTERVAL;
                    } else {
                        // Force or auto-negotiate PHY mode
                        set_phy_negotiate_mode(priv->netdev);

                        // Set PHY specfic features
                        initialise_phy(priv);

                        state = WDS_NEGOTIATING;
                        new_timeout = jiffies + AUTO_NEG_INTERVAL;
                    }
                    break;
                default:
                    DBG(1, KERN_ERR "watchdog_timer_action() %s: Unexpected state\n", priv->netdev->name);
                    state = WDS_IDLE;
                    break;
            }
        }
    } else {
        state = WDS_IDLE;
        if (duplex_changed) {
            priv->mii.full_duplex ? mac_reg_set_mask(priv,   MAC_CONFIG_REG, (1UL << MAC_CONFIG_DM_BIT)) :
                                    mac_reg_clear_mask(priv, MAC_CONFIG_REG, (1UL << MAC_CONFIG_DM_BIT));
        }

        if (gigabit_changed) {
            change_gig_mode(priv);
        }

		if (pause_changed) {
			change_pause_mode(priv);
		}
    }
#endif // !ARMULATING

    // Re-trigger the timer, unless some other thread has requested it be stopped
    if (!priv->watchdog_timer_shutdown) {
        // Restart the timer
        mod_timer(&priv->watchdog_timer, new_timeout);
    }
}

static int inline is_ip_packet(unsigned short eth_protocol)
{
    return (eth_protocol == ETH_P_IP)
#ifdef SUPPORT_IPV6
		|| (eth_protocol == ETH_P_IPV6)
#endif // SUPPORT_IPV6
		;
}

static int inline is_ipv4_packet(unsigned short eth_protocol)
{
    return eth_protocol == ETH_P_IP;
}

#ifdef SUPPORT_IPV6
static int inline is_ipv6_packet(unsigned short eth_protocol)
{
    return eth_protocol == ETH_P_IPV6;
}
#endif // SUPPORT_IPV6

static int inline is_hw_checksummable(unsigned short protocol)
{
    return (protocol == IPPROTO_TCP) || (protocol == IPPROTO_UDP)
#ifndef CONFIG_OXNAS_VERSION_0X800
           || (protocol == IPPROTO_ICMP)
#endif // !CONFIG_OXNAS_VERSION_0X800
           ;
}

static u32 unmap_rx_page(
	gmac_priv_t *priv,
	dma_addr_t   phys_adr)
{
	u32 offset = phys_adr & ~PAGE_MASK;
	u32 next_offset = offset + priv->rx_buffer_size_;
	next_offset = SKB_DATA_ALIGN(next_offset);
	next_offset += NET_IP_ALIGN;

	// If this is the last packet in a page
	if (next_offset > GMAC_ALLOC_SIZE) {
		// Release the DMA mapping for the page
		dma_unmap_page(0, phys_adr & PAGE_MASK, GMAC_ALLOC_SIZE, DMA_FROM_DEVICE);
	}

	return offset;
}

#define FCS_LEN 4		// Ethernet CRC length
#ifdef CONFIG_OXNAS_VERSION_0X800
#define HW_CSUM_LEN 2	// The OX800 H/W appending partial csum length
#endif // CONFIG_OXNAS_VERSION_0X800

static inline int get_desc_len(
	u32 desc_status,
	int last)
{
	int length = get_rx_length(desc_status);

	if (last) {
		length -= FCS_LEN;
#if defined(CONFIG_OXNAS_VERSION_0X800) && defined(USE_RX_CSUM)
	    length -= HW_CSUM_LEN;
#endif // CONFIG_OXNAS_VERSION_0X800 && USE_RX_CSUM
	}

	return length;
}

static int process_rx_packet_skb(gmac_priv_t *priv)
{
	int             desc;
	int             last;
	u32             desc_status = 0;
	rx_frag_info_t  frag_info;
	int             packet_len;
	struct sk_buff *skb;
	int             valid;
	int             ip_summed;

	desc = get_rx_descriptor(priv, &last, &desc_status, &frag_info);
	if (desc < 0) {
		return 0;
	}

	// Release the DMA mapping for the received data
    dma_unmap_single(0, frag_info.phys_adr, frag_info.length, DMA_FROM_DEVICE);

	// Get the packet data length
	packet_len = get_desc_len(desc_status, last);

	// Get pointer to the SKB
	skb = (struct sk_buff*)frag_info.page;

	// Is the packet entirely contained within the descriptors and without errors?
	valid = !(desc_status & (1UL << RDES0_ES_BIT));

	if (unlikely(!valid)) {
		goto not_valid_skb;
	}

	ip_summed = CHECKSUM_NONE;

#ifdef USE_RX_CSUM
	// Has the h/w flagged an IP header checksum failure?
	valid = !(desc_status & (1UL << RDES0_IPC_BIT));

	if (likely(valid)) {
		// Determine whether Ethernet frame contains an IP packet -
		// only bother with Ethernet II frames, but do cope with
		// 802.1Q VLAN tag presence
		int vlan_offset = 0;
		unsigned short eth_protocol = ntohs(((struct ethhdr*)skb->data)->h_proto);
		int is_ip = is_ip_packet(eth_protocol);

		if (!is_ip) {
			// Check for VLAN tag
			if (eth_protocol == ETH_P_8021Q) {
				// Extract the contained protocol type from after
				// the VLAN tag
				eth_protocol = ntohs(*(unsigned short*)(skb->data + ETH_HLEN));
				is_ip = is_ip_packet(eth_protocol);

				// Adjustment required to skip the VLAN stuff and
				// get to the IP header
				vlan_offset = 4;
			}
		}

		// Only offload checksum calculation for IP packets
		if (is_ip) {
#ifdef CONFIG_OXNAS_VERSION_0X800
			u16 payload_length = 0;
#endif // CONFIG_OXNAS_VERSION_0X800
			struct iphdr* ipv4_header = 0;

#ifndef CONFIG_OXNAS_VERSION_0X800
			if (unlikely(desc_status & (1UL << RDES0_PCE_BIT))) {
				valid = 0;
			} else
#endif // !CONFIG_OXNAS_VERSION_0X800
			if (is_ipv4_packet(eth_protocol)) {
				ipv4_header = (struct iphdr*)(skb->data + ETH_HLEN + vlan_offset);

				// H/W can only checksum non-fragmented IP packets
				if (!(ipv4_header->frag_off & htons(IP_MF | IP_OFFSET))) {
#ifdef CONFIG_OXNAS_VERSION_0X800
					switch (ipv4_header->protocol) {
						case IPPROTO_TCP:
							// Compute TCP pseudo-header checksum
							payload_length = ntohs(ipv4_header->tot_len) - (ipv4_header->ihl*4);
							break;
						case IPPROTO_UDP:
							{
								struct udphdr* udp_header = (struct udphdr*)((u8*)ipv4_header + (ipv4_header->ihl*4));
								payload_length = ntohs(udp_header->len);
							}
							break;
						default:
							// Not supporting any other than TCP/UDP
							break;
					}
#else // CONFIG_OXNAS_VERSION_0X800
				if (is_hw_checksummable(ipv4_header->protocol)) {
					ip_summed = CHECKSUM_UNNECESSARY;
				}
#endif // CONFIG_OXNAS_VERSION_0X800
				}
			}
#ifndef CONFIG_OXNAS_VERSION_0X800
			 else {
#ifdef SUPPORT_IPV6
				struct ipv6hdr* ipv6_header = (struct ipv6hdr*)(skb->data + ETH_HLEN + vlan_offset);

				if (is_hw_checksummable(ipv6_header->nexthdr)) {
					ip_summed = CHECKSUM_UNNECESSARY;
				}
#endif // SUPPORT_IPV6
			}
#endif // !CONFIG_OXNAS_VERSION_0X800

#ifdef CONFIG_OXNAS_VERSION_0X800
			if (payload_length) {
				// Get the hardware generated payload checksum from
				// the end of the received packet, reverse the 1's
				// complement operation that the h/w applies and add
				// to the pseudo-header checksum, in network order
				u16 hw_csum = ~(*(u16*)(skb->data + packet_len + FCS_LEN));

				// Calculate checksum of pseudo header and payload
				if (csum_tcpudp_magic(
						ipv4_header->saddr,
						ipv4_header->daddr,
						payload_length,
						ipv4_header->protocol,
						hw_csum)) {
					// Bad checksum, so indicate in descriptor status
					desc_status |= (1UL << RDES0_IPC_BIT);
					valid = 0;
				} else {
					ip_summed = CHECKSUM_UNNECESSARY;
				}
			}
#endif // CONFIG_OXNAS_VERSION_0X800
		}
	}

	if (unlikely(!valid)) {
		goto not_valid_skb;
	}
#endif // USE_RX_CSUM

	// Increase the skb's data pointer to account for the RX packet that has
	// been DMAed into it
	skb_put(skb, packet_len);

	// Set the device for the skb
	skb->dev = priv->netdev;

	// Set packet protocol
	skb->protocol = eth_type_trans(skb, priv->netdev);

	// Record whether h/w checksumed the packet
	skb->ip_summed = ip_summed;

	// Send the packet up the network stack
	netif_receive_skb(skb);

	// Update receive statistics
	priv->netdev->last_rx = jiffies;
	++priv->stats.rx_packets;
	priv->stats.rx_bytes += packet_len;

	return 1;

not_valid_skb:
	dev_kfree_skb(skb);

	DBG(2, KERN_WARNING "process_rx_packet() %s: Received packet has bad desc_status = 0x%08x\n", priv->netdev->name, desc_status);

	// Update receive statistics from the descriptor status
	if (is_rx_collision_error(desc_status)) {
		DBG(20, KERN_INFO "process_rx_packet() %s: Collision (0x%08x:%u bytes)\n", priv->netdev->name, desc_status, desc_len);
		++priv->stats.collisions;
	}
	if (is_rx_crc_error(desc_status)) {
		DBG(20, KERN_INFO "process_rx_packet() %s: CRC error (0x%08x:%u bytes)\n", priv->netdev->name, desc_status, desc_len);
		++priv->stats.rx_crc_errors;
		++priv->stats.rx_errors;
	}
	if (is_rx_frame_error(desc_status)) {
		DBG(20, KERN_INFO "process_rx_packet() %s: frame error (0x%08x:%u bytes)\n", priv->netdev->name, desc_status, desc_len);
		++priv->stats.rx_frame_errors;
		++priv->stats.rx_errors;
	}
	if (is_rx_length_error(desc_status)) {
		DBG(20, KERN_INFO "process_rx_packet() %s: Length error (0x%08x:%u bytes)\n", priv->netdev->name, desc_status, desc_len);
		++priv->stats.rx_length_errors;
		++priv->stats.rx_errors;
	}
	if (is_rx_csum_error(desc_status)) {
		DBG(20, KERN_INFO "process_rx_packet() %s: Checksum error (0x%08x:%u bytes)\n", priv->netdev->name, desc_status, desc_len);
		++priv->stats.rx_frame_errors;
		++priv->stats.rx_errors;
	}

	return 0;
}

static int process_rx_packet(gmac_priv_t *priv)
{
	struct sk_buff *skb = NULL;
	int             last;
	u32             desc_status;
	rx_frag_info_t  frag_info;
	int             desc;
	u32             offset;
	int             desc_len;
	unsigned char  *packet;
	int             valid;
	int             desc_used = 0;
	int             hlen = 0;
	int             partial_len = 0;
	int             first = 1;

	// Check that there is at least one Rx descriptor available. Cache the
	// descriptor information so we don't have to touch the uncached/unbuffered
	// descriptor memory more than necessary when we come to use that descriptor
	if (!rx_available_for_read(&priv->rx_gmac_desc_list_info, &desc_status)) {
		return 0;
	}

	// Attempt to allocate an skb before we change anything on the Rx descriptor ring
	skb = dev_alloc_skb(GMAC_HLEN + NET_IP_ALIGN);
	if (unlikely(skb == NULL)) {
		return 0;
	}

	// Align IP header start in header storage
	skb_reserve(skb, NET_IP_ALIGN);

	// Process all descriptors associated with the packet
	while (1) {
		int prev_len;

		// First call to get_rx_descriptor() will use the status read from the
		// first descriptor by the call to rx_available_for_read() above
		while ((desc = get_rx_descriptor(priv, &last, &desc_status, &frag_info)) < 0) {
			// We are part way through processing a multi-descriptor packet
			// and the GMAC hasn't finished with the next descriptor for the
			// packet yet, so have to poll until it becomes available
			desc_status = 0;
			udelay(1);
		}

		// We've consumed a descriptor
		++desc_used;

		if (!frag_info.page) {
			panic("process_rx_packet() %s: Found RX descriptor without attached page\n", priv->netdev->name);
		}

		// If this is the last packet in the page, release the DMA mapping
		offset = unmap_rx_page(priv, frag_info.phys_adr);
		if (!first) {
			// The buffer adr of descriptors associate with middle or last
			// parts of a packet have ls 2 bits of buffer adr ignored by GMAC DMA
			offset &= ~0x3;
		}

		// Get the length of the packet excluding CRC, h/w csum etc.
		prev_len = partial_len;
		partial_len = get_desc_len(desc_status, last);
		desc_len = partial_len - prev_len;

		// Get a pointer to the start of the packet data received into page
		packet = page_address(frag_info.page) + offset;

		// Is the packet entirely contained within the desciptors and without errors?
		valid = !(desc_status & (1UL << RDES0_ES_BIT));

		if (unlikely(!valid)) {
			goto not_valid;
		}

		if (first) {
			// Store headers in skb buffer
			hlen = min(GMAC_HLEN, desc_len);

			// Copy header into skb buffer
			memcpy(skb->data, packet, hlen);
			skb->tail += hlen;

			if (desc_len > hlen) {
				// Point skb frags array at remaining packet data in pages
				skb_shinfo(skb)->nr_frags = 1;
				skb_shinfo(skb)->frags[0].page = frag_info.page;
				skb_shinfo(skb)->frags[0].page_offset = offset + hlen;
				skb_shinfo(skb)->frags[0].size = desc_len - hlen;
			} else {
				// Entire packet now in skb buffer so don't require page anymore
				put_page(frag_info.page);
			}

			first = 0;
		} else {
			// Store intermediate descriptor data into packet
			int frag_index = skb_shinfo(skb)->nr_frags;
			skb_shinfo(skb)->frags[frag_index].page = frag_info.page;
			skb_shinfo(skb)->frags[frag_index].page_offset = offset;
			skb_shinfo(skb)->frags[frag_index].size = desc_len;
			++skb_shinfo(skb)->nr_frags;
		}

		if (last) {
			int ip_summed = CHECKSUM_NONE;

			// Update total packet length skb metadata
			skb->len = partial_len;
			skb->data_len = skb->len - hlen;
			skb->truesize = skb->len + sizeof(struct sk_buff);

#ifdef USE_RX_CSUM
			// Has the h/w flagged an IP header checksum failure?
			valid = !(desc_status & (1UL << RDES0_IPC_BIT));

			// Are we offloading RX checksuming?
			if (likely(valid)) {
				// Determine whether Ethernet frame contains an IP packet -
				// only bother with Ethernet II frames, but do cope with
				// 802.1Q VLAN tag presence
				int vlan_offset = 0;
				unsigned short eth_protocol = ntohs(((struct ethhdr*)skb->data)->h_proto);
				int is_ip = is_ip_packet(eth_protocol);

				if (!is_ip) {
					// Check for VLAN tag
					if (eth_protocol == ETH_P_8021Q) {
						// Extract the contained protocol type from after
						// the VLAN tag
						eth_protocol = ntohs(*(unsigned short*)(skb->data + ETH_HLEN));
						is_ip = is_ip_packet(eth_protocol);

						// Adjustment required to skip the VLAN stuff and
						// get to the IP header
						vlan_offset = 4;
					}
				}

				// Only offload checksum calculation for IP packets
				if (is_ip) {
#ifdef CONFIG_OXNAS_VERSION_0X800
					u16 payload_length = 0;
#endif // CONFIG_OXNAS_VERSION_0X800
					struct iphdr* ipv4_header = 0;

#ifndef CONFIG_OXNAS_VERSION_0X800
					if (unlikely(desc_status & (1UL << RDES0_PCE_BIT))) {
						valid = 0;
					} else
#endif // !CONFIG_OXNAS_VERSION_0X800
					if (is_ipv4_packet(eth_protocol)) {
						ipv4_header = (struct iphdr*)(skb->data + ETH_HLEN + vlan_offset);

						// H/W can only checksum non-fragmented IP packets
						if (!(ipv4_header->frag_off & htons(IP_MF | IP_OFFSET))) {
#ifdef CONFIG_OXNAS_VERSION_0X800
							switch (ipv4_header->protocol) {
								case IPPROTO_TCP:
									// Compute TCP pseudo-header checksum
									payload_length = ntohs(ipv4_header->tot_len) - (ipv4_header->ihl*4);
									break;
								case IPPROTO_UDP:
									{
										struct udphdr* udp_header = (struct udphdr*)((u8*)ipv4_header + (ipv4_header->ihl*4));
										payload_length = ntohs(udp_header->len);
									}
									break;
								default:
									// Not supporting any other than TCP/UDP
									break;
							}
#else // CONFIG_OXNAS_VERSION_0X800
						if (is_hw_checksummable(ipv4_header->protocol)) {
							ip_summed = CHECKSUM_UNNECESSARY;
						}
#endif // CONFIG_OXNAS_VERSION_0X800
						}
					}
#ifndef CONFIG_OXNAS_VERSION_0X800
					 else {
#ifdef SUPPORT_IPV6
						struct ipv6hdr* ipv6_header = (struct ipv6hdr*)(skb->data + ETH_HLEN + vlan_offset);

						if (is_hw_checksummable(ipv6_header->nexthdr)) {
							ip_summed = CHECKSUM_UNNECESSARY;
						}
#endif // SUPPORT_IPV6
					}
#endif // !CONFIG_OXNAS_VERSION_0X800

#ifdef CONFIG_OXNAS_VERSION_0X800
					if (payload_length) {
						// Get the hardware generated payload checksum from
						// the end of the received packet, reverse the 1's
						// complement operation that the h/w applies and add
						// to the pseudo-header checksum, in network order
						u16 hw_csum = ~(*(u16*)(packet + desc_len + FCS_LEN));

						// Calculate checksum of pseudo header and payload
						if (csum_tcpudp_magic(
								ipv4_header->saddr,
								ipv4_header->daddr,
								payload_length,
								ipv4_header->protocol,
								hw_csum)) {
							// Bad checksum, so indicate in descriptor status
							desc_status |= (1UL << RDES0_IPC_BIT);
							valid = 0;
						} else {
							ip_summed = CHECKSUM_UNNECESSARY;
						}
					}
#endif // CONFIG_OXNAS_VERSION_0X800
				}
			}

			if (unlikely(!valid)) {
				goto not_valid;
			}
#endif // USE_RX_CSUM

			// Initialise other required skb header fields
			skb->dev = priv->netdev;
			skb->protocol = eth_type_trans(skb, priv->netdev);

			// Record whether h/w checksumed the packet
			skb->ip_summed = ip_summed;

			// Send the skb up the network stack
			netif_receive_skb(skb);

			// Update receive statistics
			priv->netdev->last_rx = jiffies;
			++priv->stats.rx_packets;
			priv->stats.rx_bytes += partial_len;

			break;
		}

		// Want next call to get_rx_descriptor() to read status from descriptor
		desc_status = 0;
	}
    return desc_used;

not_valid:
	if (!skb_shinfo(skb)->nr_frags) {
		// Free the page as it wasn't attached to the skb
		put_page(frag_info.page);
	}

	dev_kfree_skb(skb);

	DBG(2, KERN_WARNING "process_rx_packet() %s: Received packet has bad desc_status = 0x%08x\n", priv->netdev->name, desc_status);

	// Update receive statistics from the descriptor status
	if (is_rx_collision_error(desc_status)) {
		DBG(20, KERN_INFO "process_rx_packet() %s: Collision (0x%08x:%u bytes)\n", priv->netdev->name, desc_status, desc_len);
		++priv->stats.collisions;
	}
	if (is_rx_crc_error(desc_status)) {
		DBG(20, KERN_INFO "process_rx_packet() %s: CRC error (0x%08x:%u bytes)\n", priv->netdev->name, desc_status, desc_len);
		++priv->stats.rx_crc_errors;
		++priv->stats.rx_errors;
	}
	if (is_rx_frame_error(desc_status)) {
		DBG(20, KERN_INFO "process_rx_packet() %s: frame error (0x%08x:%u bytes)\n", priv->netdev->name, desc_status, desc_len);
		++priv->stats.rx_frame_errors;
		++priv->stats.rx_errors;
	}
	if (is_rx_length_error(desc_status)) {
		DBG(20, KERN_INFO "process_rx_packet() %s: Length error (0x%08x:%u bytes)\n", priv->netdev->name, desc_status, desc_len);
		++priv->stats.rx_length_errors;
		++priv->stats.rx_errors;
	}
	if (is_rx_csum_error(desc_status)) {
		DBG(20, KERN_INFO "process_rx_packet() %s: Checksum error (0x%08x:%u bytes)\n", priv->netdev->name, desc_status, desc_len);
		++priv->stats.rx_frame_errors;
		++priv->stats.rx_errors;
	}

	return desc_used;
}

/*
 * NAPI receive polling method
 */
static int poll(
	struct napi_struct *napi,
	int                 budget)
{
	gmac_priv_t *priv = container_of(napi, gmac_priv_t, napi_struct);
	struct net_device *dev = priv->netdev;
    int rx_work_limit = budget;
    int work_done = 0;
    int continue_polling;
    int finished;
    int available;
	int desc_since_refill = 0;

    finished = 0;
    do {
        u32 status;

        // While there are receive polling jobs to be done
        while (rx_work_limit) {
			int desc_used;

			if (unlikely(priv->rx_buffers_per_page)) {
				desc_used = process_rx_packet(priv);
			} else {
				desc_used = process_rx_packet_skb(priv);
			}

			if (!desc_used) {
				break;
			}

            // Increment count of processed packets
            ++work_done;

            // Decrement our remaining budget
            if (rx_work_limit > 0) {
                --rx_work_limit;
            }

            // Rx overflows seem to upset the GMAC, so try to ensure we never see them
			desc_since_refill += desc_used;
            if (desc_since_refill >= DESC_SINCE_REFILL_LIMIT) {
                desc_since_refill = 0;
                refill_rx_ring(dev);
            }
        }

        if (rx_work_limit) {
            // We have unused budget remaining, but apparently no Rx packets to
            // process
            available = 0;

            // Clear any RI status so we don't immediately get reinterrupted
            // when we leave polling, due to either a new RI event, or a left
            // over interrupt from one of the RX descriptors we've already
            // processed
            status = dma_reg_read(priv, DMA_STATUS_REG);
            if (status & (1UL << DMA_STATUS_RI_BIT)) {
                // Ack the RI, including the normal summary sticky bit
                dma_reg_write(priv, DMA_STATUS_REG, ((1UL << DMA_STATUS_RI_BIT)  |
                                                     (1UL << DMA_STATUS_NIS_BIT)));

                // Must check again for available RX descriptors, in case the RI
                // status came from a new RX descriptor
                available = rx_available_for_read(&priv->rx_gmac_desc_list_info, 0);
            }

            if (!available) {
                // We have budget left but no Rx packets to process so stop
                // polling
                continue_polling = 0;
                finished = 1;
            }
        } else {
            // If we have consumed all our budget, don't cancel the
            // poll, the NAPI instructure assumes we won't
            continue_polling = 1;

            // Must leave poll() routine as no budget left
            finished = 1;
        }
    } while (!finished);

    // Attempt to fill any empty slots in the RX ring
    refill_rx_ring(dev);

    // Decrement the budget even if we didn't process any packets
    if (!work_done) {
        work_done = 1;
    }

    if (!continue_polling) {
        // No more received packets to process so return to interrupt mode
        netif_rx_complete(dev, napi);

        // Enable interrupts caused by received packets that may have been
		// disabled in the ISR before entering polled mode
        gmac_int_en_set(priv, (1UL << DMA_INT_ENABLE_RI_BIT) |
                              (1UL << DMA_INT_ENABLE_RU_BIT) |
							  (1UL << DMA_INT_ENABLE_OV_BIT));
    }

    return work_done;
}

#if defined(CONFIG_LEON_COPRO) && defined(CONFIG_LEON_OFFLOAD_TX)
static void copro_fill_tx_job(
    volatile gmac_tx_que_ent_t *job,
    struct sk_buff             *skb)
{
    int i;
    int nr_frags = skb_shinfo(skb)->nr_frags;
    unsigned short flags = 0;
    dma_addr_t hdr_dma_address;

    // if too many fragments call sbk_linearize()
    // and take the CPU memory copies hit 
    if (nr_frags > COPRO_NUM_TX_FRAGS_DIRECT) {
        int err;
        printk(KERN_WARNING "Fill: linearizing socket buffer as required %d frags and have only %d\n", nr_frags, COPRO_NUM_TX_FRAGS_DIRECT);
        err = skb_linearize(skb);
        if (err) {
            panic("Fill: No free memory");
        }

        // update nr_frags
        nr_frags = skb_shinfo(skb)->nr_frags;
    }

    // Get a DMA mapping of the packet's data
    hdr_dma_address = dma_map_single(0, skb->data, skb_headlen(skb), DMA_TO_DEVICE);
    BUG_ON(dma_mapping_error(hdr_dma_address));

    // Allocate storage for remainder of fragments and create DMA mappings
    // Get a DMA mapping for as many fragments as will fit into the first level
    // fragment info. storage within the job structure
    for (i=0; i < nr_frags; ++i) {
        struct skb_frag_struct *frag = &skb_shinfo(skb)->frags[i];

#ifdef CONFIG_OXNAS_GMAC_AVOID_CACHE_CLEAN
        if (PageMappedToDisk(frag->page) && !PageDirty(frag->page)) {
            job->frag_ptr_[i] = virt_to_dma(0, page_address(frag->page) + frag->page_offset);
        } else {
#endif // CONFIG_OXNAS_GMAC_AVOID_CACHE_CLEAN
            job->frag_ptr_[i] = dma_map_page(0, frag->page, frag->page_offset, frag->size, DMA_TO_DEVICE);
#ifdef CONFIG_OXNAS_GMAC_AVOID_CACHE_CLEAN
        }
#endif // CONFIG_OXNAS_GMAC_AVOID_CACHE_CLEAN
        job->frag_len_[i] = frag->size;
    }

    // Is h/w checksumming and possibly TSO required
    if (likely((skb->ip_summed == CHECKSUM_PARTIAL) &&
               (ntohs(skb->protocol) == ETH_P_IP))) {
        flags |= (1UL << TX_JOB_FLAGS_ACCELERATE_BIT);
    }

    // Fill the job description with information about the packet
    job->skb_         = (u32)skb;
    job->len_         = skb->len;
    job->data_len_    = skb->data_len;
    job->ethhdr_      = hdr_dma_address;
    job->iphdr_       = hdr_dma_address + (skb_network_header(skb) - skb->data);
    job->iphdr_csum_  = ((struct iphdr*)skb_network_header(skb))->check;
    job->tso_segs_    = skb_shinfo(skb)->gso_segs;
    job->tso_size_    = skb_shinfo(skb)->gso_size;
    job->flags_       = flags;
    job->statistics_  = 0;
}

static void copro_free_tx_resources(volatile gmac_tx_que_ent_t* job)
{
    int i;
    struct sk_buff* skb = (struct sk_buff*)job->skb_;
    int nr_frags = skb_shinfo(skb)->nr_frags;

    // This should never happen, since we check space when we filled
    // the job in copro_fill_tx_job
    if (nr_frags > COPRO_NUM_TX_FRAGS_DIRECT) {
        panic("Free: Insufficient fragment storage, required %d, have only %d", nr_frags, COPRO_NUM_TX_FRAGS_DIRECT);
    }

    // Release the DMA mapping for the data directly referenced by the SKB
    dma_unmap_single(0, job->ethhdr_, skb_headlen(skb), DMA_TO_DEVICE);

    // Release the DMA mapping for any fragments in the first level fragment
    // info. storage within the job structure
    for (i=0; (i < nr_frags) && (i < COPRO_NUM_TX_FRAGS_DIRECT); ++i) {
        dma_unmap_page(0, job->frag_ptr_[i], job->frag_len_[i], DMA_TO_DEVICE);
    }

    // Inform the network stack that we've finished with the packet
    dev_kfree_skb_irq(skb);
}

static void finish_xmit(struct net_device *dev)
{
    gmac_priv_t                *priv = (gmac_priv_t*)netdev_priv(dev);
    volatile gmac_tx_que_ent_t *job;

	// Make SMP safe - we could do with making this locking more fine grained
	spin_lock(&priv->tx_spinlock_);

    // Process all available completed jobs
    while ((job = tx_que_get_finished_job(dev))) {
        int aborted;
        int carrier;
        int collisions;
        u32 statistics = job->statistics_;

        copro_free_tx_resources(job);

        // Accumulate TX statistics returned by CoPro in the job structure
        priv->stats.tx_bytes   += (statistics & TX_JOB_STATS_BYTES_MASK)   >> TX_JOB_STATS_BYTES_BIT;
        priv->stats.tx_packets += (statistics & TX_JOB_STATS_PACKETS_MASK) >> TX_JOB_STATS_PACKETS_BIT;
        aborted    = (statistics & TX_JOB_STATS_ABORT_MASK)     >> TX_JOB_STATS_ABORT_BIT;
        carrier    = (statistics & TX_JOB_STATS_CARRIER_MASK)   >> TX_JOB_STATS_CARRIER_BIT;
        collisions = (statistics & TX_JOB_STATS_COLLISION_MASK) >> TX_JOB_STATS_COLLISION_BIT;
        priv->stats.tx_aborted_errors += aborted;
        priv->stats.tx_carrier_errors += carrier;
        priv->stats.collisions        += collisions;
        priv->stats.tx_errors += (aborted + carrier);
    }

    // If the network stack's Tx queue was stopped and we now have resources
    // to process more Tx offload jobs
    if (netif_queue_stopped(dev) &&
        !tx_que_is_full(&priv->tx_queue_)) {
        // Restart the network stack's TX queue
        netif_wake_queue(dev);
    }

	// Make SMP safe - we could do with making this locking more fine grained
	spin_unlock(&priv->tx_spinlock_);
}
#else
static void finish_xmit(struct net_device *dev)
{
    gmac_priv_t *priv = (gmac_priv_t*)netdev_priv(dev);
    unsigned     descriptors_freed = 0;
    u32          desc_status = 0;

    // Handle transmit descriptors for the completed packet transmission
    while (1) {
        struct sk_buff *skb;
        tx_frag_info_t  fragment;
        int             buffer_owned;
		int				 desc_index;

        // Get tx descriptor content, accumulating status for all buffers
        // contributing to each packet
		spin_lock(&priv->tx_spinlock_);
		desc_index = get_tx_descriptor(priv, &skb, &desc_status, &fragment, &buffer_owned);
		spin_unlock(&priv->tx_spinlock_);

		if (desc_index < 0) {
			// No more completed Tx packets
			break;
		}

        // Only unmap DMA buffer if descriptor owned the buffer
        if (buffer_owned) {
            // Release the DMA mapping for the buffer
            dma_unmap_single(0, fragment.phys_adr, fragment.length, DMA_TO_DEVICE);
        }

        // When all buffers contributing to a packet have been processed
        if (skb) {
            // Check the status of the transmission
            if (likely(is_tx_valid(desc_status))) {
                priv->stats.tx_bytes += skb->len;
                priv->stats.tx_packets++;
            } else {
                priv->stats.tx_errors++;
                if (is_tx_aborted(desc_status)) {
                    ++priv->stats.tx_aborted_errors;
                }
                if (is_tx_carrier_error(desc_status)) {
                    ++priv->stats.tx_carrier_errors;
                }
            }

            if (unlikely(is_tx_collision_error(desc_status))) {
                ++priv->stats.collisions;
            }

            // Inform the network stack that packet transmission has finished
            dev_kfree_skb_irq(skb);

            // Start accumulating status for the next packet
            desc_status = 0;
        }

        // Track how many descriptors we make available, so we know
        // if we need to re-start of network stack's TX queue processing
        ++descriptors_freed;
    }

    // If the TX queue is stopped, there may be a pending TX packet waiting to
    // be transmitted
    if (unlikely(netif_queue_stopped(dev))) {
		// No locking with hard_start_xmit() required, as queue is already
		// stopped so hard_start_xmit() won't touch the h/w

        // If any TX descriptors have been freed and there is an outstanding TX
        // packet waiting to be queued due to there not having been a TX
        // descriptor available when hard_start_xmit() was presented with an skb
        // by the network stack
        if (priv->tx_pending_skb) {
            // Construct the GMAC specific DMA descriptor
            if (set_tx_descriptor(priv,
                                  priv->tx_pending_skb,
                                  priv->tx_pending_fragments,
                                  priv->tx_pending_fragment_count,
                                  priv->tx_pending_skb->ip_summed == CHECKSUM_PARTIAL) >= 0) {
                // No TX packets now outstanding
                priv->tx_pending_skb = 0;
                priv->tx_pending_fragment_count = 0;

                // We have used one of the TX descriptors freed by transmission
                // completion processing having occured above
                --descriptors_freed;

                // Issue a TX poll demand to restart TX descriptor processing, as we
                // have just added one, in case it had found there were no more
                // pending transmission
                dma_reg_write(priv, DMA_TX_POLL_REG, 0);
            }
        }

        // If there are TX descriptors available we should restart the TX queue
        if (descriptors_freed) {
            // The TX queue had been stopped by hard_start_xmit() due to lack of
            // TX descriptors, so restart it now that we've freed at least one
            netif_wake_queue(dev);
        }
    }
}
#endif // CONFIG_LEON_COPRO && CONFIG_LEON_OFFLOAD_TX

#ifndef CONFIG_LEON_COPRO
static void process_non_dma_ints(u32 raw_status)
{
    printk(KERN_ERR "Found GPI/GMI/GLI interrupt\n");
}
#endif // !CONFIG_LEON_COPRO

#ifdef CONFIG_LEON_COPRO
static void copro_fwd_intrs_handler(
    void *dev_id,
    u32   status)
{
    struct net_device *dev = (struct net_device *)dev_id;
    gmac_priv_t       *priv = (gmac_priv_t*)netdev_priv(dev);
    int                restart_watchdog = 0;
    int                restart_tx = 0;
    int                poll_tx = 0;

    // Test for normal receive interrupt
    if (status & (1UL << DMA_STATUS_RI_BIT)) {
        if (netif_rx_schedule_prep(dev, &priv->napi_struct)) {
            // Tell system we have work to be done
            __netif_rx_schedule(dev, &priv->napi_struct);
        } else {
            printk(KERN_ERR "copro_fwd_intrs_handler() %s: RX interrupt while in poll\n", dev->name);
        }
    }

    // Test for unavailable RX buffers - CoPro should have disabled
    if (unlikely(status & (1UL << DMA_STATUS_RU_BIT))) {
        DBG(30, KERN_INFO "int_handler() %s: RX buffer unavailable\n", dev->name);
        // Accumulate receive statistics
        ++priv->stats.rx_over_errors;
        ++priv->stats.rx_errors;
    }

    // Test for Rx overflow - CoPro should have disabled
	if (unlikely(status & (1UL << DMA_STATUS_OVF_BIT))) {
		DBG(30, KERN_INFO "int_handler() %s: Rx overflow\n", dev->name);
		// Accumulate receive statistics
		++priv->stats.rx_fifo_errors;
		++priv->stats.rx_errors;
	}

    // Test for normal TX interrupt
    if (status & ((1UL << DMA_STATUS_TI_BIT) |
                  (1UL << DMA_STATUS_ETI_BIT))) {
#ifndef CONFIG_LEON_OFFLOAD_TX
        // Finish packet transmision started by start_xmit
        finish_xmit(dev);
#endif // !CONFIG_LEON_OFFLOAD_TX
    }

    // Test for abnormal transmitter interrupt where there may be completed
    // packets waiting to be processed
    if (unlikely(status & ((1UL << DMA_STATUS_TJT_BIT) |
                           (1UL << DMA_STATUS_UNF_BIT)))) {
#ifndef CONFIG_LEON_OFFLOAD_TX
        // Complete processing of any TX packets closed by the DMA
        finish_xmit(dev);
#endif // !CONFIG_LEON_OFFLOAD_TX

        if (status & (1UL << DMA_STATUS_TJT_BIT)) {
            // A transmit jabber timeout causes the transmitter to enter the
            // stopped state
            DBG(50, KERN_INFO "int_handler() %s: TX jabber timeout\n", dev->name);
            restart_tx = 1;
        } else {
            DBG(51, KERN_INFO "int_handler() %s: TX underflow\n", dev->name);
        }

        // Issue a TX poll demand in an attempt to restart TX descriptor
        // processing
        poll_tx = 1;
    }

    // Test for any of the error states which we deal with directly within
    // this interrupt service routine.
    if (unlikely(status & ((1UL << DMA_STATUS_ERI_BIT) |
                           (1UL << DMA_STATUS_RWT_BIT) |
                           (1UL << DMA_STATUS_RPS_BIT) |
                           (1UL << DMA_STATUS_TPS_BIT) |
                           (1UL << DMA_STATUS_FBE_BIT)))) {
        // Test for early RX interrupt
        if (status & (1UL << DMA_STATUS_ERI_BIT)) {
            // Don't expect to see this, as never enable it
            DBG(30, KERN_WARNING "int_handler() %s: Early RX \n", dev->name);
        }

        if (status & (1UL << DMA_STATUS_RWT_BIT)) {
            DBG(30, KERN_INFO "int_handler() %s: RX watchdog timeout\n", dev->name);
            // Accumulate receive statistics
            ++priv->stats.rx_frame_errors;
            ++priv->stats.rx_errors;
            restart_watchdog = 1;
        }

        if (status & (1UL << DMA_STATUS_RPS_BIT)) {
            // Mask to extract the receive status field from the status register
//            u32 rs_mask = ((1UL << DMA_STATUS_RS_NUM_BITS) - 1) << DMA_STATUS_RS_BIT;
//            u32 rs = (status & rs_mask) >> DMA_STATUS_RS_BIT;
//            printk("int_handler() %s: RX process stopped 0x%x\n", dev->name, rs);
            ++priv->stats.rx_errors;
            restart_watchdog = 1;

            // Restart the receiver
            DBG(35, KERN_INFO "int_handler() %s: Restarting receiver\n", dev->name);
            change_rx_enable(priv, 1, 0, 1);
        }

        if (status & (1UL << DMA_STATUS_TPS_BIT)) {
            // Mask to extract the transmit status field from the status register
//            u32 ts_mask = ((1UL << DMA_STATUS_TS_NUM_BITS) - 1) << DMA_STATUS_TS_BIT;
//            u32 ts = (status & ts_mask) >> DMA_STATUS_TS_BIT;
//            printk("int_handler() %s: TX process stopped 0x%x\n", dev->name, ts);
            ++priv->stats.tx_errors;
            restart_watchdog = 1;
            restart_tx = 1;
        }

        // Test for pure error interrupts
        if (status & (1UL << DMA_STATUS_FBE_BIT)) {
            // Mask to extract the bus error status field from the status register
//            u32 eb_mask = ((1UL << DMA_STATUS_EB_NUM_BITS) - 1) << DMA_STATUS_EB_BIT;
//            u32 eb = (status & eb_mask) >> DMA_STATUS_EB_BIT;
//            printk("int_handler() %s: Bus error 0x%x\n", dev->name, eb);
            restart_watchdog = 1;
        }

        if (restart_watchdog) {
            // Restart the link/PHY state watchdog immediately, which will
            // attempt to restart the system
            mod_timer(&priv->watchdog_timer, jiffies);
            restart_watchdog = 0;
        }
    }

    if (unlikely(restart_tx)) {
        // Restart the transmitter, causes am implicit Tx descriptor list poll
        DBG(35, KERN_INFO "int_handler() %s: Restarting transmitter\n", dev->name);
#ifndef CONFIG_LEON_OFFLOAD_TX
        dma_reg_set_mask(priv, DMA_OP_MODE_REG, (1UL << DMA_OP_MODE_ST_BIT));
#endif // !CONFIG_LEON_OFFLOAD_TX
        poll_tx = 0;
    }

    if (unlikely(poll_tx)) {
        // Issue a TX poll demand in an attempt to restart TX descriptor
        // processing
        DBG(33, KERN_INFO "int_handler() %s: Issuing Tx poll demand\n", dev->name);
#ifndef CONFIG_LEON_OFFLOAD_TX
        dma_reg_write(priv, DMA_TX_POLL_REG, 0);
#endif // !CONFIG_LEON_OFFLOAD_TX
    }
}
#else // CONFIG_LEON_COPRO 
static irqreturn_t int_handler(int int_num, void* dev_id)
{
    struct net_device *dev = (struct net_device *)dev_id;
    gmac_priv_t* priv = (gmac_priv_t*)netdev_priv(dev);
    u32 int_enable;
    int rx_polling;
    u32 raw_status;
    u32 status;

    /** Read the interrupt enable register to determine if we're in rx poll mode
     *  Id like to get rid of this read, if a more efficient way of determining
     *  whether we are polling is available */
    spin_lock(&priv->cmd_que_lock_);
    int_enable = dma_reg_read(priv, DMA_INT_ENABLE_REG);
    spin_unlock(&priv->cmd_que_lock_);

    rx_polling = !(int_enable & (1UL << DMA_INT_ENABLE_RI_BIT));

    // Get interrupt status
    raw_status = dma_reg_read(priv, DMA_STATUS_REG);

    // MMC, PMT and GLI interrupts are not masked by the interrupt enable
    // register, so must deal with them on the raw status
    if (unlikely(raw_status & ((1UL << DMA_STATUS_GPI_BIT) |
                      (1UL << DMA_STATUS_GMI_BIT) |
                      (1UL << DMA_STATUS_GLI_BIT)))) {
        process_non_dma_ints(raw_status);
    }

    // Get status of enabled interrupt sources
    status = raw_status & int_enable;

    while (status) {
        // Whether the link/PHY watchdog timer should be restarted
        int restart_watchdog = 0;
        int restart_tx       = 0;
        int poll_tx          = 0;
        u32 int_disable_mask = 0;

        // Test for RX interrupt resulting from sucessful reception of a packet-
        // must do this before ack'ing, else otherwise can get into trouble with
        // the sticky summary bits when we try to disable further RI interrupts
        if (status & (1UL << DMA_STATUS_RI_BIT)) {
//printk("RI ");
            // Disable interrupts caused by received packets as henceforth
            // we shall poll for packet reception
            int_disable_mask |= (1UL << DMA_INT_ENABLE_RI_BIT);

            // Do NAPI compatible receive processing for RI interrupts
            if (likely(netif_rx_schedule_prep(dev, &priv->napi_struct))) {
                // Remember that we are polling, so we ignore RX events for the
                // remainder of the ISR
                rx_polling = 1;

                // Tell system we have work to be done
                __netif_rx_schedule(dev, &priv->napi_struct);
            } else {
                printk(KERN_ERR "int_handler() %s: RX interrupt while in poll\n", dev->name);
            }
        }

        // Test for unavailable RX buffers - must do this before ack'ing, else
        // otherwise can get into trouble with the sticky summary bits
        if (unlikely(status & (1UL << DMA_STATUS_RU_BIT))) {
            printk(/*DBG(30, KERN_INFO */"int_handler() %s: RX buffer unavailable\n", dev->name);
            // Accumulate receive statistics
            ++priv->stats.rx_over_errors;
            ++priv->stats.rx_errors;

            // Disable RX buffer unavailable reporting, so we don't get swamped
            int_disable_mask |= (1UL << DMA_INT_ENABLE_RU_BIT);
        }

		if (unlikely(status & (1UL << DMA_STATUS_OVF_BIT))) {
			printk(/*DBG(30, KERN_INFO */"int_handler() %s: RX overflow\n", dev->name);
			// Accumulate receive statistics
			++priv->stats.rx_fifo_errors;
			++priv->stats.rx_errors;

            // Disable RX overflow reporting, so we don't get swamped
            int_disable_mask |= (1UL << DMA_INT_ENABLE_OV_BIT);
		}

        // Do any interrupt disabling with a single register write
        if (int_disable_mask) {
            gmac_int_en_clr(priv, int_disable_mask, 0, 1);

            // Update our record of the current interrupt enable status
            int_enable &= ~int_disable_mask;
        }

        // The broken GMAC interrupt mechanism with its sticky summary bits
        // means that we have to ack all asserted interrupts here; we can't not
        // ack the RI interrupt source as we might like to (in order that the
        // poll() routine could examine the status) because if it was asserted
        // prior to being masked above, then the summary bit(s) would remain
        // asserted and cause an immediate re-interrupt.
        dma_reg_write(priv, DMA_STATUS_REG, status | ((1UL << DMA_STATUS_NIS_BIT) |
                                                      (1UL << DMA_STATUS_AIS_BIT)));

        // Test for normal TX interrupt
        if (status & ((1UL << DMA_STATUS_TI_BIT) |
                      (1UL << DMA_STATUS_ETI_BIT))) {
            // Finish packet transmision started by start_xmit
            finish_xmit(dev);
        }

        // Test for abnormal transmitter interrupt where there may be completed
        // packets waiting to be processed
        if (unlikely(status & ((1UL << DMA_STATUS_TJT_BIT) |
                               (1UL << DMA_STATUS_UNF_BIT)))) {
            // Complete processing of any TX packets closed by the DMA
            finish_xmit(dev);

            if (status & (1UL << DMA_STATUS_TJT_BIT)) {
                // A transmit jabber timeout causes the transmitter to enter the
                // stopped state
                DBG(50, KERN_INFO "int_handler() %s: TX jabber timeout\n", dev->name);
                restart_tx = 1;
            } else {
                DBG(51, KERN_INFO "int_handler() %s: TX underflow\n", dev->name);
            }

            // Issue a TX poll demand in an attempt to restart TX descriptor
            // processing
            poll_tx = 1;
        }

        // Test for any of the error states which we deal with directly within
        // this interrupt service routine.
        if (unlikely(status & ((1UL << DMA_STATUS_ERI_BIT) |
                               (1UL << DMA_STATUS_RWT_BIT) |
                               (1UL << DMA_STATUS_RPS_BIT) |
                               (1UL << DMA_STATUS_TPS_BIT) |
                               (1UL << DMA_STATUS_FBE_BIT)))) {
            // Test for early RX interrupt
            if (status & (1UL << DMA_STATUS_ERI_BIT)) {
                // Don't expect to see this, as never enable it
                DBG(30, KERN_WARNING "int_handler() %s: Early RX \n", dev->name);
            }

            if (status & (1UL << DMA_STATUS_RWT_BIT)) {
                DBG(30, KERN_INFO "int_handler() %s: RX watchdog timeout\n", dev->name);
                // Accumulate receive statistics
                ++priv->stats.rx_frame_errors;
                ++priv->stats.rx_errors;
                restart_watchdog = 1;
            }

            if (status & (1UL << DMA_STATUS_RPS_BIT)) {
				// Mask to extract the receive status field from the status register
				u32 rs_mask = ((1UL << DMA_STATUS_RS_NUM_BITS) - 1) << DMA_STATUS_RS_BIT;
				u32 rs = (status & rs_mask) >> DMA_STATUS_RS_BIT;
				printk(/*DBG(30, KERN_INFO */"int_handler() %s: RX process stopped 0x%x\n", dev->name, rs);
				++priv->stats.rx_errors;
				restart_watchdog = 1;

                // Restart the receiver
                printk(/*DBG(35, KERN_INFO */"int_handler() %s: Restarting receiver\n", dev->name);
                dma_reg_set_mask(priv, DMA_OP_MODE_REG, (1UL << DMA_OP_MODE_SR_BIT));
            }

            if (status & (1UL << DMA_STATUS_TPS_BIT)) {
				// Mask to extract the transmit status field from the status register
//				u32 ts_mask = ((1UL << DMA_STATUS_TS_NUM_BITS) - 1) << DMA_STATUS_TS_BIT;
//				u32 ts = (status & ts_mask) >> DMA_STATUS_TS_BIT;
//				DBG(30, KERN_INFO "int_handler() %s: TX process stopped 0x%x\n", dev->name, ts);
                ++priv->stats.tx_errors;
                restart_watchdog = 1;
                restart_tx = 1;
            }

            // Test for pure error interrupts
            if (status & (1UL << DMA_STATUS_FBE_BIT)) {
				// Mask to extract the bus error status field from the status register
//				u32 eb_mask = ((1UL << DMA_STATUS_EB_NUM_BITS) - 1) << DMA_STATUS_EB_BIT;
//				u32 eb = (status & eb_mask) >> DMA_STATUS_EB_BIT;
//				DBG(30, KERN_INFO "int_handler() %s: Bus error 0x%x\n", dev->name, eb);
                restart_watchdog = 1;
            }

            if (restart_watchdog) {
                // Restart the link/PHY state watchdog immediately, which will
                // attempt to restart the system
                mod_timer(&priv->watchdog_timer, jiffies);
                restart_watchdog = 0;
            }
        }

        if (unlikely(restart_tx)) {
            // Restart the transmitter
            DBG(35, KERN_INFO "int_handler() %s: Restarting transmitter\n", dev->name);
            dma_reg_set_mask(priv, DMA_OP_MODE_REG, (1UL << DMA_OP_MODE_ST_BIT));
        }

        if (unlikely(poll_tx)) {
            // Issue a TX poll demand in an attempt to restart TX descriptor
            // processing
            DBG(33, KERN_INFO "int_handler() %s: Issuing Tx poll demand\n", dev->name);
            dma_reg_write(priv, DMA_TX_POLL_REG, 0);
        }

        // Read the record of current interrupt requests again, in case some
        // more arrived while we were processing
        raw_status = dma_reg_read(priv, DMA_STATUS_REG);

        // MMC, PMT and GLI interrupts are not masked by the interrupt enable
        // register, so must deal with them on the raw status
        if (unlikely(raw_status & ((1UL << DMA_STATUS_GPI_BIT) |
                                   (1UL << DMA_STATUS_GMI_BIT) |
                                   (1UL << DMA_STATUS_GLI_BIT))))  {
            process_non_dma_ints(raw_status);
        }

        // Get status of enabled interrupt sources.
        status = raw_status & int_enable;
    }

    return IRQ_HANDLED;
}
#endif // CONFIG_LEON_COPRO 

#ifdef CONFIG_LEON_COPRO
static struct semaphore copro_stop_semaphore;

static void copro_stop_callback(volatile gmac_cmd_que_ent_t* entry)
{
    up(&copro_stop_semaphore);
}
#endif // CONFIG_LEON_COPRO

static void gmac_down(struct net_device *dev)
{
    gmac_priv_t* priv = (gmac_priv_t*)netdev_priv(dev);
    int desc;
    u32 int_enable;
#ifdef CONFIG_LEON_COPRO
    tx_que_t *tx_queue = &priv->tx_queue_;
    int cmd_queue_result;
    unsigned long irq_flags = 0;
#endif // CONFIG_LEON_COPRO

	if (priv->napi_enabled) {
		// Stop NAPI
		napi_disable(&priv->napi_struct);
		priv->napi_enabled = 0;
	}

    // Stop further TX packets being delivered to hard_start_xmit();
    netif_stop_queue(dev);
    netif_carrier_off(dev);

#ifdef CONFIG_LEON_COPRO
	if (priv->copro_started) {
		// Disable all GMAC interrupts and wait for change to be acknowledged
		gmac_copro_int_en_clr(priv, ~0UL, &int_enable);

		// Tell the CoPro to stop network offload operations
		cmd_queue_result = -1;
		while (cmd_queue_result) {
			spin_lock_irqsave(&priv->cmd_que_lock_, irq_flags);
			cmd_queue_result =
				cmd_que_queue_cmd(&priv->cmd_queue_,
								  GMAC_CMD_STOP, 0, copro_stop_callback);
			spin_unlock_irqrestore(&priv->cmd_que_lock_, irq_flags);
		}

		// Interrupt the CoPro so it sees the new command
		writel(1UL << COPRO_SEM_INT_CMD, SYS_CTRL_SEMA_SET_CTRL);

		// Wait until the CoPro acknowledges the STOP command
		down_interruptible(&copro_stop_semaphore);

		// Wait until the CoPro acknowledges that it has completed stopping
		down_interruptible(&priv->copro_stop_complete_semaphore_);

		priv->copro_started = 0;
	}

    // Clear out the Tx offload job queue, deallocating associated resources
    while (tx_que_not_empty(tx_queue)) {
        // Free any dynamic fragment ptr/len storage
        /** @todo */
        tx_que_inc_r_ptr(tx_queue);
    }

    // Reinitialise the Tx offload queue metadata
    tx_que_init(
        tx_queue,
        (gmac_tx_que_ent_t*)descriptors_phys_to_virt(priv->copro_params_.tx_que_head_),
        priv->copro_tx_que_num_entries_);
#else // CONFIG_LEON_COPRO
    // Disable all GMAC interrupts
    gmac_int_en_clr(priv, ~0UL, 0, 0);
#endif // CONFIG_LEON_COPRO

#ifndef CONFIG_LEON_OFFLOAD_TX
    // Stop transmitter, take ownership of all tx descriptors
    dma_reg_clear_mask(priv, DMA_OP_MODE_REG, 1UL << DMA_OP_MODE_ST_BIT);
    if (priv->desc_vaddr) {
        tx_take_ownership(&priv->tx_gmac_desc_list_info);
    }
#endif // !CONFIG_LEON_OFFLOAD_TX

    // Stop receiver, waiting until it's really stopped and then take ownership
    // of all rx descriptors
    change_rx_enable(priv, 0, 1, 0);

    if (priv->desc_vaddr) {
        rx_take_ownership(&priv->rx_gmac_desc_list_info);
    }

    // Stop all timers
    delete_watchdog_timer(priv);

    if (priv->desc_vaddr) {
        // Free receive descriptors
        do {
            int first_last = 0;
            rx_frag_info_t frag_info;

            desc = get_rx_descriptor(priv, &first_last, 0, &frag_info);
            if (desc >= 0) {
				if (unlikely(priv->rx_buffers_per_page)) {
					// If this is the last packet in the page, release the DMA mapping
					unmap_rx_page(priv, frag_info.phys_adr);
					put_page(frag_info.page);
				} else {
                    // Release the DMA mapping for the packet buffer
                    dma_unmap_single(0, frag_info.phys_adr, frag_info.length, DMA_FROM_DEVICE);

                    // Free the skb
                    dev_kfree_skb((struct sk_buff *)frag_info.page);
				}
            }
        } while (desc >= 0);

        // Free transmit descriptors
        do {
            struct sk_buff *skb;
            tx_frag_info_t  frag_info;
            int             buffer_owned;

            desc = get_tx_descriptor(priv, &skb, 0, &frag_info, &buffer_owned);
            if (desc >= 0) {
                if (buffer_owned) {
                    // Release the DMA mapping for the packet buffer
                    dma_unmap_single(0, frag_info.phys_adr, frag_info.length, DMA_FROM_DEVICE);
                }

                if (skb) {
                    // Free the skb
                    dev_kfree_skb(skb);
                }
            }
        } while (desc >= 0);

        // Free any resources associated with the buffers of a pending packet
        if (priv->tx_pending_fragment_count) {
            tx_frag_info_t *frag_info = priv->tx_pending_fragments;

            while (priv->tx_pending_fragment_count--) {
                dma_unmap_single(0, frag_info->phys_adr, frag_info->length, DMA_FROM_DEVICE);
                ++frag_info;
            }
        }

        // Free the socket buffer of a pending packet
        if (priv->tx_pending_skb) {
            dev_kfree_skb(priv->tx_pending_skb);
            priv->tx_pending_skb = 0;
        }
    }

    // Power down the PHY
    phy_powerdown(dev);
}

static int stop(struct net_device *dev)
{
    gmac_priv_t* priv = (gmac_priv_t*)netdev_priv(dev);

    gmac_down(dev);

#ifdef CONFIG_LEON_COPRO
    shutdown_copro();

    if (priv->shared_copro_params_) {
        // Free the DMA coherent parameter space
        dma_free_coherent(0, sizeof(copro_params_t), priv->shared_copro_params_, priv->shared_copro_params_pa_);
        priv->shared_copro_params_ = 0;
    }

    // Disable semaphore register from causing ARM interrupts
    *((volatile unsigned long*)SYS_CTRL_SEMA_MASKA_CTRL) = 0;
    *((volatile unsigned long*)SYS_CTRL_SEMA_MASKB_CTRL) = 0;

    // Release interrupts lines used by semaphore register interrupts
    if (priv->copro_a_irq_alloced_) {
        free_irq(priv->copro_a_irq_, dev);
        priv->copro_a_irq_alloced_ = 0;
    }
    if (priv->copro_b_irq_alloced_) {
        free_irq(priv->copro_b_irq_, dev);
        priv->copro_b_irq_alloced_ = 0;
    }
#endif // CONFIG_LEON_COPRO 

	// Free the shadow descriptor memory
	kfree(priv->tx_desc_shadow_);
	priv->tx_desc_shadow_ = 0;

	kfree(priv->rx_desc_shadow_);
	priv->rx_desc_shadow_ = 0;

    // Release the IRQ
    if (priv->have_irq) {
        free_irq(dev->irq, dev);
        priv->have_irq = 0;
    }

    // Disable the clock to the MAC block and put it into reset
    writel(1UL << SYS_CTRL_CKEN_MAC_BIT, SYS_CTRL_CKEN_CLR_CTRL);
    writel(1UL << SYS_CTRL_RSTEN_MAC_BIT, SYS_CTRL_RSTEN_SET_CTRL);

    // Free the sysfs resources
    kobject_del(&priv->link_state_kobject);
    subsystem_unregister(&priv->link_state_kset);

	priv->interface_up = 0;
    return 0;
}

static void hw_set_mac_address(struct net_device *dev, unsigned char* addr)
{
    u32 mac_lo;
    u32 mac_hi;

    mac_lo  =  (u32)addr[0];
    mac_lo |= ((u32)addr[1] << 8);
    mac_lo |= ((u32)addr[2] << 16);
    mac_lo |= ((u32)addr[3] << 24);

    mac_hi  =  (u32)addr[4];
    mac_hi |= ((u32)addr[5] << 8);

    mac_reg_write(netdev_priv(dev), MAC_ADR0_LOW_REG, mac_lo);
    mac_reg_write(netdev_priv(dev), MAC_ADR0_HIGH_REG, mac_hi);
}

static int set_mac_address(struct net_device *dev, void *p)
{
    struct sockaddr *addr = p;

    if (!is_valid_ether_addr(addr->sa_data)) {
        return -EADDRNOTAVAIL;
    }

    memcpy(dev->dev_addr, addr->sa_data, dev->addr_len);
    hw_set_mac_address(dev, addr->sa_data);

    return 0;
}

static void multicast_hash(struct dev_mc_list *dmi, u32 *hash_lo, u32 *hash_hi)
{
    u32 crc = ether_crc_le(dmi->dmi_addrlen, dmi->dmi_addr);
    u32 mask = 1 << ((crc >> 26) & 0x1F);

    if (crc >> 31) {
        *hash_hi |= mask;
    } else {
        *hash_lo |= mask;
    }
}

static void set_multicast_list(struct net_device *dev)
{
    gmac_priv_t* priv = netdev_priv(dev);
    u32 hash_lo=0;
    u32 hash_hi=0;
    u32 mode = 0;
    int i;

    // Disable promiscuous mode and uni/multi-cast matching
    mac_reg_write(priv, MAC_FRAME_FILTER_REG, mode);

    // Disable all perfect match registers
    for (i=0; i < NUM_PERFECT_MATCH_REGISTERS; ++i) {
        mac_adrhi_reg_write(priv, i, 0);
    }

    // Promiscuous mode overrides all-multi which overrides other filtering
    if (dev->flags & IFF_PROMISC) {
        mode |= (1 << MAC_FRAME_FILTER_PR_BIT);
    } else if (dev->flags & IFF_ALLMULTI) {
        mode |= (1 << MAC_FRAME_FILTER_PM_BIT);
    } else {
        struct dev_mc_list *dmi;

        if (dev->mc_count <= NUM_PERFECT_MATCH_REGISTERS) {
            // Use perfect matching registers
            for (i=0, dmi = dev->mc_list; dmi; dmi = dmi->next, ++i) {
                u32 addr;

                addr  =      dmi->dmi_addr[0];
                addr |= (u32)dmi->dmi_addr[1] << 8;
                addr |= (u32)dmi->dmi_addr[2] << 16;
                addr |= (u32)dmi->dmi_addr[3] << 24;
                mac_adrlo_reg_write(priv, i, addr);

                addr  =      dmi->dmi_addr[4];
                addr |= (u32)dmi->dmi_addr[5] << 8;
                addr |= (1 << MAC_ADR1_HIGH_AE_BIT);
                mac_adrhi_reg_write(priv, i, addr);
            }
        } else {
            // Use hashing
            mode |= (1 << MAC_FRAME_FILTER_HUC_BIT);
            mode |= (1 << MAC_FRAME_FILTER_HMC_BIT);

            for (dmi = dev->mc_list; dmi; dmi = dmi->next) {
                multicast_hash(dmi, &hash_lo, &hash_hi);
            }
        }
    }

    // Update the filtering rules
    mac_reg_write(priv, MAC_FRAME_FILTER_REG, mode);

    // Update the filtering hash table
    mac_reg_write(priv, MAC_HASH_LOW_REG,  hash_lo);
    mac_reg_write(priv, MAC_HASH_HIGH_REG, hash_hi);
}

static int gmac_up(struct net_device *dev)
{
    int status = 0;
    gmac_priv_t *priv = (gmac_priv_t*)netdev_priv(dev);
    u32 reg_contents;
#ifdef CONFIG_LEON_COPRO
    int cmd_queue_result;
    unsigned long irq_flags = 0;
#endif // CONFIG_LEON_COPRO

	// Perform any actions required before GMAC reset
	do_pre_reset_actions(priv);

    // Reset the entire GMAC
    dma_reg_write(priv, DMA_BUS_MODE_REG, 1UL << DMA_BUS_MODE_SWR_BIT);

    // Ensure reset is performed before testing for completion
    wmb();

    // Wait for the reset operation to complete
    status = -EIO;
	printk(KERN_INFO "Resetting GMAC\n");
    for (;;) {
        if (!(dma_reg_read(priv, DMA_BUS_MODE_REG) & (1UL << DMA_BUS_MODE_SWR_BIT))) {
            status = 0;
            break;
        }
    }

	// Perform any actions required after GMAC reset
	do_post_reset_actions(priv);

    // Did the GMAC reset operation fail?
    if (status) {
        printk(KERN_ERR "open() %s: GMAC reset failed\n", dev->name);
        goto gmac_up_err_out;
    }
	printk(KERN_INFO "GMAC reset complete\n");

	/* Initialise MAC config register contents
	 */
    reg_contents = 0;
    if (!priv->mii.using_1000) {
        DBG(1, KERN_INFO "open() %s: PHY in 10/100Mb mode\n", dev->name);
        reg_contents |= (1UL << MAC_CONFIG_PS_BIT);
    } else {
        DBG(1, KERN_INFO "open() %s: PHY in 1000Mb mode\n", dev->name);
    }
    if (priv->mii.full_duplex) {
        reg_contents |= (1UL << MAC_CONFIG_DM_BIT);
    }

#ifdef USE_RX_CSUM
	reg_contents |= (1UL << MAC_CONFIG_IPC_BIT);
#endif // USE_RX_CSUM

    if (priv->jumbo_) {
		// Allow passage of jumbo frames through both transmitter and receiver
		reg_contents |=	((1UL << MAC_CONFIG_JE_BIT) |
                        (1UL << MAC_CONFIG_JD_BIT) |
						 (1UL << MAC_CONFIG_WD_BIT));
	}

	// Enable transmitter and receiver
    reg_contents |= ((1UL << MAC_CONFIG_TE_BIT) |
                     (1UL << MAC_CONFIG_RE_BIT));

	// Select the minimum IFG - I found that 80 bit times caused very poor
	// IOZone performance, so stcik with the 96 bit times default
	reg_contents |= (0UL << MAC_CONFIG_IFG_BIT);

    // Write MAC config setup to the GMAC
    mac_reg_write(priv, MAC_CONFIG_REG, reg_contents);

	/* Initialise MAC VLAN register contents
	 */
    reg_contents = 0;
    mac_reg_write(priv, MAC_VLAN_TAG_REG, reg_contents);

    // Initialise the hardware's record of our primary MAC address
    hw_set_mac_address(dev, dev->dev_addr);

    // Initialise multicast and promiscuous modes
    set_multicast_list(dev);

    // Disable all MMC interrupt sources
    mac_reg_write(priv, MMC_RX_MASK_REG, ~0UL);
    mac_reg_write(priv, MMC_TX_MASK_REG, ~0UL);

    // Remember how large the unified descriptor array is to be
    priv->total_num_descriptors = NUM_TX_DMA_DESCRIPTORS + NUM_RX_DMA_DESCRIPTORS;

    // Initialise the structures managing the TX descriptor list
    init_tx_desc_list(&priv->tx_gmac_desc_list_info,
                      priv->desc_vaddr,
                      priv->tx_desc_shadow_,
                      NUM_TX_DMA_DESCRIPTORS);

    // Initialise the structures managing the RX descriptor list
    init_rx_desc_list(&priv->rx_gmac_desc_list_info,
                      priv->desc_vaddr + NUM_TX_DMA_DESCRIPTORS,
                      priv->rx_desc_shadow_,
                      NUM_RX_DMA_DESCRIPTORS,
                      priv->rx_buffer_size_);

    // Reset record of pending Tx packet
    priv->tx_pending_skb = 0;
    priv->tx_pending_fragment_count = 0;

#ifndef CONFIG_LEON_OFFLOAD_TX
    // Write the physical DMA consistent address of the start of the tx descriptor array
    dma_reg_write(priv, DMA_TX_DESC_ADR_REG, priv->desc_dma_addr);
#endif // !CONFIG_LEON_OFFLOAD_TX

    // Write the physical DMA consistent address of the start of the rx descriptor array
    dma_reg_write(priv, DMA_RX_DESC_ADR_REG, priv->desc_dma_addr +
                        (priv->tx_gmac_desc_list_info.num_descriptors * sizeof(gmac_dma_desc_t)));

    // Initialise the GMAC DMA bus mode register
    dma_reg_write(priv, DMA_BUS_MODE_REG, ((1UL << DMA_BUS_MODE_FB_BIT)   |	// Force bursts
                                           (8UL << DMA_BUS_MODE_PBL_BIT)  |	// AHB burst size
                                           (1UL << DMA_BUS_MODE_DA_BIT)));	// Round robin Rx/Tx

    // Prepare receive descriptors
    refill_rx_ring(dev);

    // Clear any pending interrupt requests
    dma_reg_write(priv, DMA_STATUS_REG, dma_reg_read(priv, DMA_STATUS_REG));

	/* Initialise flow control register contents
	 */
	// Enable Rx flow control
    reg_contents = (1UL << MAC_FLOW_CNTL_RFE_BIT);

#ifndef CONFIG_OXNAS_VERSION_0X800
	if (priv->mii.using_pause) {
		// Enable Tx flow control
		reg_contents |= (1UL << MAC_FLOW_CNTL_TFE_BIT);
	}

    // Set the duration of the pause frames generated by the transmitter when
	// the Rx fifo fill threshold is exceeded
	reg_contents |= ((0x100UL << MAC_FLOW_CNTL_PT_BIT) |	// Pause for 256 slots
					  (0x1UL << MAC_FLOW_CNTL_PLT_BIT));
#endif // !CONFIG_OXNAS_VERSION_0X800

	// Write flow control setup to the GMAC
    mac_reg_write(priv, MAC_FLOW_CNTL_REG, reg_contents);

	/* Initialise operation mode register contents
	 */
    // Initialise the GMAC DMA operation mode register. Set Tx/Rx FIFO thresholds
    // to make best use of our limited SDRAM bandwidth when operating in gigabit
	reg_contents = ((DMA_OP_MODE_TTC_256 << DMA_OP_MODE_TTC_BIT) |    // Tx threshold
                    (1UL << DMA_OP_MODE_FUF_BIT) |    					// Forward Undersized good Frames
                    (DMA_OP_MODE_RTC_128 << DMA_OP_MODE_RTC_BIT) |	// Rx threshold 128 bytes
                    (1UL << DMA_OP_MODE_OSF_BIT));						// Operate on 2nd frame

#ifndef CONFIG_OXNAS_VERSION_0X800
	// Enable hardware flow control
	reg_contents |= (1UL << DMA_OP_MODE_EFC_BIT);

	// Set threshold for enabling hardware flow control at (full-4KB) to give
	// space for upto two in-flight std MTU packets to arrive after pause frame
	// has been sent.
	reg_contents |= ((0UL << DMA_OP_MODE_RFA2_BIT) |
					  (3UL << DMA_OP_MODE_RFA_BIT));

	// Set threshold for disabling hardware flow control (-7KB)
	reg_contents |= ((1UL << DMA_OP_MODE_RFD2_BIT) |
					  (2UL << DMA_OP_MODE_RFD_BIT));

    // Don't flush Rx frames from FIFO just because there's no descriptor available
	reg_contents |= (1UL << DMA_OP_MODE_DFF_BIT);
#endif // !CONFIG_OXNAS_VERSION_0X800

	// Write settings to operation mode register
    dma_reg_write(priv, DMA_OP_MODE_REG, reg_contents);

#ifdef CONFIG_OXNAS_VERSION_0X800
    // Use store&forward when operating in gigabit mode, as OX800 does not have
    // sufficient SDRAM bandwidth to support gigabit Tx without it and OX800
	// does not support Tx checksumming in the GMAC
    if (priv->mii.using_1000) {
        dma_reg_set_mask(priv, DMA_OP_MODE_REG, (1UL << DMA_OP_MODE_SF_BIT));
    } else {
        dma_reg_clear_mask(priv, DMA_OP_MODE_REG, (1UL << DMA_OP_MODE_SF_BIT));
    }
#else // CONFIG_OXNAS_VERSION_0X800
    // GMAC requires store&forward in order to compute Tx checksums
    dma_reg_set_mask(priv, DMA_OP_MODE_REG, (1UL << DMA_OP_MODE_SF_BIT));
#endif // CONFIG_OXNAS_VERSION_0X800

    // Ensure setup is complete, before enabling TX and RX
    wmb();

#ifdef CONFIG_LEON_COPRO
    // Update the CoPro's parameters with the current MTU
    priv->copro_params_.mtu_ = dev->mtu;

    // Only attempt to write to uncached/unbuffered shared parameter storage if
    // CoPro is started and thus storage has been allocated
    if (priv->shared_copro_params_) {
        // Fill the CoPro parameter block
        memcpy(priv->shared_copro_params_, &priv->copro_params_, sizeof(copro_params_t));
    }

    // Make sure the CoPro parameter block updates have made it to memory (which
    // is uncached/unbuffered, so just compiler issues to overcome)
    wmb();

    // Tell the CoPro to re-read parameters
    cmd_queue_result = -1;
    while (cmd_queue_result) {
        spin_lock_irqsave(&priv->cmd_que_lock_, irq_flags);
        cmd_queue_result = cmd_que_queue_cmd(&priv->cmd_queue_, GMAC_CMD_UPDATE_PARAMS, 0, copro_update_callback);
        spin_unlock_irqrestore(&priv->cmd_que_lock_, irq_flags);
    }

    // Interrupt the CoPro so it sees the new command
    writel(1UL << COPRO_SEM_INT_CMD, SYS_CTRL_SEMA_SET_CTRL);

    // Wait until the CoPro acknowledges that the update of parameters is complete
    down_interruptible(&copro_update_semaphore);

    // Tell the CoPro to begin network offload operations
    cmd_queue_result = -1;
    while (cmd_queue_result) {
        spin_lock_irqsave(&priv->cmd_que_lock_, irq_flags);
        cmd_queue_result = cmd_que_queue_cmd(&priv->cmd_queue_, GMAC_CMD_START, 0, copro_start_callback);
        spin_unlock_irqrestore(&priv->cmd_que_lock_, irq_flags);
    }

    // Interrupt the CoPro so it sees the new command
    writel(1UL << COPRO_SEM_INT_CMD, SYS_CTRL_SEMA_SET_CTRL);

    // Wait until the CoPro acknowledges that it has started
    down_interruptible(&copro_start_semaphore);

    priv->copro_started = 1;
#endif // CONFIG_LEON_COPRO

	// Start NAPI
	BUG_ON(priv->napi_enabled);
	napi_enable(&priv->napi_struct);
	priv->napi_enabled = 1;

    // Start the transmitter and receiver
#ifndef CONFIG_LEON_OFFLOAD_TX
    dma_reg_set_mask(priv, DMA_OP_MODE_REG, (1UL << DMA_OP_MODE_ST_BIT));
#endif // !LEON_OFFLOAD_TX
    change_rx_enable(priv, 1, 0, 0);

    // Enable interesting GMAC interrupts
    gmac_int_en_set(priv, ((1UL << DMA_INT_ENABLE_NI_BIT)  |
                           (1UL << DMA_INT_ENABLE_AI_BIT)  |
                           (1UL << DMA_INT_ENABLE_FBE_BIT) |
                           (1UL << DMA_INT_ENABLE_RI_BIT)  |
                           (1UL << DMA_INT_ENABLE_RU_BIT)  |
                           (1UL << DMA_INT_ENABLE_OV_BIT)  |
                           (1UL << DMA_INT_ENABLE_RW_BIT)  |
                           (1UL << DMA_INT_ENABLE_RS_BIT)  |
                           (1UL << DMA_INT_ENABLE_TI_BIT)  |
                           (1UL << DMA_INT_ENABLE_UN_BIT)  |
                           (1UL << DMA_INT_ENABLE_TJ_BIT)  |
                           (1UL << DMA_INT_ENABLE_TS_BIT)));

    // (Re)start the link/PHY state monitoring timer
    start_watchdog_timer(priv);

    // Allow the network stack to call hard_start_xmit()
    netif_start_queue(dev);

#ifdef DUMP_REGS_ON_GMAC_UP
    dump_mac_regs(priv->macBase, priv->dmaBase);
#endif // DUMP_REGS_ON_GMAC_UP

    return status;

gmac_up_err_out:
    stop(dev);

    return status;
}

static void set_rx_packet_info(struct net_device *dev)
{
    gmac_priv_t *priv = (gmac_priv_t*)netdev_priv(dev);
	int max_packet_buffer_size = dev->mtu + EXTRA_RX_SKB_SPACE;

	if (max_packet_buffer_size > max_descriptor_length()) {
#ifndef RX_BUFFER_SIZE
		priv->rx_buffer_size_ = max_packet_buffer_size;
#else // !RX_BUFFER_SIZE
		priv->rx_buffer_size_ = RX_BUFFER_SIZE;
#endif // ! RX_BUFFER_SIZE
		priv->rx_buffers_per_page = GMAC_ALLOC_SIZE / (priv->rx_buffer_size_ + NET_IP_ALIGN);
	} else {
		priv->rx_buffer_size_ = max_packet_buffer_size;
		priv->rx_buffers_per_page = 0;
	}
}

static int change_mtu(struct net_device *dev, int new_mtu)
{
    int status = 0;
    gmac_priv_t *priv = (gmac_priv_t*)netdev_priv(dev);
    int original_mtu = dev->mtu;

    // Check that new MTU is within supported range
    if ((new_mtu < MIN_PACKET_SIZE) || (new_mtu > MAX_JUMBO)) {
        DBG(1, KERN_WARNING "change_mtu() %s: Invalid MTU %d\n", dev->name, new_mtu);
        status = -EINVAL;
    } else if (priv->interface_up) {
        // Put MAC/PHY into quiesent state, causing all current buffers to be
        // deallocated and the PHY to powerdown
        gmac_down(dev);

        // Record the new MTU, so bringing the MAC back up will allocate
        // resources to suit the new MTU
        dev->mtu = new_mtu;

		// Set length etc. of rx packets
		set_rx_packet_info(dev);

        // Reset the PHY to get it into a known state and ensure we have TX/RX
        // clocks to allow the GMAC reset to complete
        if (phy_reset(priv->netdev)) {
            DBG(1, KERN_ERR "change_mtu() %s: Failed to reset PHY\n", dev->name);
            status = -EIO;
        } else {
			// Set PHY specfic features
			initialise_phy(priv);

            // Record whether jumbo frames should be enabled
            priv->jumbo_ = (dev->mtu > NORMAL_PACKET_SIZE);

            // Force or auto-negotiate PHY mode
            priv->phy_force_negotiation = 1;

            // Reallocate buffers with new MTU
            gmac_up(dev);
        }
	} else {
        // Record the new MTU, so bringing the interface up will allocate
        // resources to suit the new MTU
        dev->mtu = new_mtu;
    }

    // If there was a failure
    if (status) {
        // Return the MTU to its original value
        DBG(1, KERN_INFO "change_mtu() Failed, returning MTU to original value\n");
        dev->mtu = original_mtu;
    }

    return status;
}

#ifdef TEST_COPRO
DECLARE_MUTEX_LOCKED(start_sem);
DECLARE_MUTEX_LOCKED(heartbeat_sem);

void start_callback(volatile gmac_cmd_que_ent_t* entry)
{
    printk("START callback, operand = 0x%08x\n", entry->operand_);
    up(&start_sem);
}

void heartbeat_callback(volatile gmac_cmd_que_ent_t* entry)
{
    printk("Heartbeat callback, operand = 0x%08x\n", entry->operand_);
    up(&heartbeat_sem);
}

static void test_copro(gmac_priv_t* priv)
{
    unsigned long irq_flags;

    spin_lock(&priv->cmd_que_lock_);
    cmd_que_queue_cmd(&priv->cmd_queue_, GMAC_CMD_STOP, 0, 0);
    spin_unlock(&priv->cmd_que_lock_);
    writel(1UL << COPRO_SEM_INT_CMD, SYS_CTRL_SEMA_SET_CTRL);
    mdelay(500);

    spin_lock(&priv->cmd_que_lock_);
    cmd_que_queue_cmd(&priv->cmd_queue_, GMAC_CMD_START, 0, start_callback);
    spin_unlock(&priv->cmd_que_lock_);
    writel(1UL << COPRO_SEM_INT_CMD, SYS_CTRL_SEMA_SET_CTRL);
    mdelay(500);

    spin_lock(&priv->cmd_que_lock_);
    cmd_que_queue_cmd(&priv->cmd_queue_, GMAC_CMD_HEARTBEAT, 0, heartbeat_callback);
    spin_unlock(&priv->cmd_que_lock_);
    writel(1UL << COPRO_SEM_INT_CMD, SYS_CTRL_SEMA_SET_CTRL);
    mdelay(500);

    printk("Waiting for start ack...\n");
    down_interruptible(&start_sem);
    printk("Start ack received\n");

    printk("Waiting for heartbeat ack...\n");
    down_interruptible(&heartbeat_sem);
    printk("Heartbeat ack received\n");
}
#endif // TEST_COPRO

#ifdef CONFIG_LEON_COPRO 
#define SEM_INT_FWD      8
#define SEM_INT_ACK      16
#define SEM_INT_TX       17
#define SEM_INT_STOP_ACK 18

#define SEM_INTA_MASK  (1UL << SEM_INT_FWD)
#define SEM_INTB_MASK ((1UL << SEM_INT_ACK) | (1UL << SEM_INT_TX) | (1UL << SEM_INT_STOP_ACK))

static irqreturn_t copro_sema_intr(int irq, void *dev_id)
{
    struct net_device *dev = (struct net_device *)dev_id;
    gmac_priv_t       *priv = (gmac_priv_t*)netdev_priv(dev);
    u32                asserted;
    u32                fwd_intrs_status = 0;
    int                is_fwd_intr;

    // Read the contents of semaphore A register
    asserted = (*((volatile unsigned long*)SYS_CTRL_SEMA_STAT) & SEM_INTA_MASK);

    while (asserted) {
        // Extract any forwarded interrupts info
        is_fwd_intr = asserted & (1UL << SEM_INT_FWD);
        if (is_fwd_intr) {
            fwd_intrs_status = ((volatile gmac_fwd_intrs_t*)descriptors_phys_to_virt(priv->copro_params_.fwd_intrs_mailbox_))->status_;
        }

        // Clear any interrupts directed at the ARM
        *((volatile unsigned long*)SYS_CTRL_SEMA_CLR_CTRL) = asserted;

        if (is_fwd_intr) {
            // Process any forwarded GMAC interrupts
            copro_fwd_intrs_handler(dev_id, fwd_intrs_status);
        }

        // Stay in interrupt routine if interrupt has been re-asserted
        asserted = (*((volatile unsigned long*)SYS_CTRL_SEMA_STAT) & SEM_INTA_MASK);
    }

    return IRQ_HANDLED;
}

static irqreturn_t copro_semb_intr(int irq, void *dev_id)
{
    struct net_device *dev = (struct net_device *)dev_id;
    gmac_priv_t       *priv = (gmac_priv_t*)netdev_priv(dev);
    u32                asserted;

    // Read the contents of semaphore B register
    asserted = (*((volatile unsigned long*)SYS_CTRL_SEMA_STAT) & SEM_INTB_MASK);

    while (asserted) {
        // Clear any interrupts directed at the ARM
        *((volatile unsigned long*)SYS_CTRL_SEMA_CLR_CTRL) = asserted;

        // Process any outstanding command acknowledgements
        if (asserted & (1UL << SEM_INT_ACK)) {
            while (!cmd_que_dequeue_ack(&priv->cmd_queue_));
        }

        // Process STOP completion signal
        if (asserted & (1UL << SEM_INT_STOP_ACK)) {
            up(&priv->copro_stop_complete_semaphore_);
        }

#ifdef CONFIG_LEON_OFFLOAD_TX
        // Process any completed TX offload jobs
        if (asserted & (1UL << SEM_INT_TX)) {
            finish_xmit(dev);
        }
#endif // CONFIG_LEON_OFFLOAD_TX

        // Stay in interrupt routine if interrupt has been re-asserted
        asserted = (*((volatile unsigned long*)SYS_CTRL_SEMA_STAT) & SEM_INTB_MASK);
    }

    return IRQ_HANDLED;
}
#endif // CONFIG_LEON_COPRO 

static int open(struct net_device *dev)
{
    gmac_priv_t* priv = (gmac_priv_t*)netdev_priv(dev);
    int status;
#ifdef CONFIG_LEON_COPRO 
    const struct firmware* firmware = NULL;
#endif // CONFIG_LEON_COPRO 

    // Ensure the MAC block is properly reset
    writel(1UL << SYS_CTRL_RSTEN_MAC_BIT, SYS_CTRL_RSTEN_SET_CTRL);
    writel(1UL << SYS_CTRL_RSTEN_MAC_BIT, SYS_CTRL_RSTEN_CLR_CTRL);

    // Enable the clock to the MAC block
    writel(1UL << SYS_CTRL_CKEN_MAC_BIT, SYS_CTRL_CKEN_SET_CTRL);

    // Ensure reset and clock operations are complete
    wmb();

    // Reset the PHY to get it into a known state and ensure we have TX/RX clocks
    // to allow the GMAC reset to complete
    if (phy_reset(priv->netdev)) {
        DBG(1, KERN_ERR "open() %s: Failed to reset PHY\n", dev->name);
        status = -EIO;
        goto open_err_out;
    }

	// Set PHY specfic features
	initialise_phy(priv);

    // Check that the MAC address is valid.  If it's not, refuse to bring the
    // device up
    if (!is_valid_ether_addr(dev->dev_addr)) {
        DBG(1, KERN_ERR "open() %s: MAC address invalid\n", dev->name);
        status = -EINVAL;
        goto open_err_out;
    }

#ifdef CONFIG_LEON_COPRO 
    // Register ISRs for the semaphore register interrupt sources, which will
    // originate from the CoPro
    if (request_irq(priv->copro_a_irq_, &copro_sema_intr, 0, "SEMA", dev)) {
        panic("open: Failed to allocate semaphore A %u\n", priv->copro_a_irq_);
        status = -ENODEV;
        goto open_err_out;
    }
    priv->copro_a_irq_alloced_ = 1;

    if (request_irq(priv->copro_b_irq_, &copro_semb_intr, 0, "SEMB", dev)) {
        panic("open: Failed to allocate semaphore B %u\n", priv->copro_b_irq_);
        status = -ENODEV;
        goto open_err_out;
    }
    priv->copro_b_irq_alloced_ = 1;
#else // CONFIG_LEON_COPRO 
    // Allocate the IRQ
    if (request_irq(dev->irq, &int_handler, 0, dev->name, dev)) {
        DBG(1, KERN_ERR "open() %s: Failed to allocate irq %d\n", dev->name, dev->irq);
        status = -ENODEV;
        goto open_err_out;
    }
    priv->have_irq = 1;
#endif // CONFIG_LEON_COPRO 

    // Need a consistent DMA mapping covering all the memory occupied by DMA
    // unified descriptor array, as both CPU and DMA engine will be reading and
    // writing descriptor fields.
    priv->desc_vaddr    = (gmac_dma_desc_t*)GMAC_DESC_ALLOC_START;
    priv->desc_dma_addr = GMAC_DESC_ALLOC_START_PA;

    if (!priv->desc_vaddr) {
        DBG(1, KERN_ERR "open() %s: Failed to allocate consistent memory for DMA descriptors\n", dev->name);
        status = -ENOMEM;
        goto open_err_out;
    }

	// Allocate memory to hold shadow of GMAC descriptors
	if (!(priv->tx_desc_shadow_ = kmalloc(NUM_TX_DMA_DESCRIPTORS * sizeof(gmac_dma_desc_t), GFP_KERNEL))) {
        DBG(1, KERN_ERR "open() %s: Failed to allocate memory for Tx descriptor shadows\n", dev->name);
        status = -ENOMEM;
        goto open_err_out;
	}
	if (!(priv->rx_desc_shadow_ = kmalloc(NUM_RX_DMA_DESCRIPTORS * sizeof(gmac_dma_desc_t), GFP_KERNEL))) {
        DBG(1, KERN_ERR "open() %s: Failed to allocate memory for Rx descriptor shadows\n", dev->name);
        status = -ENOMEM;
        goto open_err_out;
	}

	// Record whether jumbo frames should be enabled
    priv->jumbo_ = (dev->mtu > NORMAL_PACKET_SIZE);

	set_rx_packet_info(dev);

#ifdef CONFIG_LEON_COPRO 
    // Allocate SRAM for the command queue entries
    priv->copro_params_.cmd_que_head_ = DESCRIPTORS_BASE_PA + DESCRIPTORS_SIZE;

    priv->copro_params_.cmd_que_tail_ =
        (u32)((gmac_cmd_que_ent_t*)(priv->copro_params_.cmd_que_head_) + priv->copro_cmd_que_num_entries_);
    priv->copro_params_.fwd_intrs_mailbox_ = priv->copro_params_.cmd_que_tail_;
    priv->copro_params_.tx_que_head_ = priv->copro_params_.fwd_intrs_mailbox_ + sizeof(gmac_fwd_intrs_t);
    priv->copro_params_.tx_que_tail_ =
        (u32)((gmac_tx_que_ent_t*)(priv->copro_params_.tx_que_head_) + priv->copro_tx_que_num_entries_);
    priv->copro_params_.free_start_ = priv->copro_params_.tx_que_tail_;

    // Set RX interrupt mitigation behaviour
    priv->copro_params_.rx_mitigation_        = COPRO_RX_MITIGATION;
    priv->copro_params_.rx_mitigation_frames_ = COPRO_RX_MITIGATION_FRAMES;
    priv->copro_params_.rx_mitigation_usec_   = COPRO_RX_MITIGATION_USECS;

    // Initialise command queue metadata
    cmd_que_init(
        &priv->cmd_queue_,
        (gmac_cmd_que_ent_t*)descriptors_phys_to_virt(priv->copro_params_.cmd_que_head_),
        priv->copro_cmd_que_num_entries_);

    // Initialise tx offload queue metadata
    tx_que_init(
        &priv->tx_queue_,
        (gmac_tx_que_ent_t*)descriptors_phys_to_virt(priv->copro_params_.tx_que_head_),
        priv->copro_tx_que_num_entries_);

    // Allocate DMA coherent space for the parameter block shared with the CoPro
    priv->shared_copro_params_ = dma_alloc_coherent(0, sizeof(copro_params_t), &priv->shared_copro_params_pa_, GFP_KERNEL);
    if (!priv->shared_copro_params_) {
        DBG(1, KERN_ERR "open() %s: Failed to allocate DMA coherent space for parameters\n", dev->name);
        status = -ENOMEM;
        goto open_err_out;
    }

    // Update the CoPro's parameters with the current MTU
    priv->copro_params_.mtu_ = dev->mtu;

    // Fill the shared CoPro parameter block from the ARM's local copy
    memcpy(priv->shared_copro_params_, &priv->copro_params_, sizeof(copro_params_t));

    // Request CoPro firmware from userspace
    if (request_firmware(&firmware, "gmac_copro_firmware", priv->device)) {
        printk(KERN_ERR "open() %s: Failed to load CoPro firmware\n", dev->name);
        status = -EIO;
        goto open_err_out;
    }

    // Load CoPro program and start it running
    init_copro(firmware->data, priv->shared_copro_params_pa_);

    // Finished with the firmware so release it
    release_firmware(firmware);

    // Enable selected semaphore register bits to cause ARM interrupts
    *((volatile unsigned long*)SYS_CTRL_SEMA_MASKA_CTRL) = SEM_INTA_MASK;
    *((volatile unsigned long*)SYS_CTRL_SEMA_MASKB_CTRL) = SEM_INTB_MASK;

#ifdef TEST_COPRO
    // Send test commands to the CoPro
    test_copro(priv);
#endif // TEST_COPRO
#endif // CONFIG_LEON_COPRO 

    // Initialise sysfs for link state reporting
    status = gmac_link_state_init_sysfs(priv);
    if (status) {
        printk(KERN_ERR "open() %s: Failed to initialise sysfs support\n", dev->name);
        goto open_err_out;
    }

    // Initialise the work queue entry to be used to issue hotplug events to userspace
    INIT_WORK(&priv->link_state_change_work, work_handler);

    // Do startup operations that are in common with gmac_down()/_up() processing
    priv->mii_init_media = 1;
    priv->phy_force_negotiation = 1;
    status = gmac_up(dev);
    if (status) {
        goto open_err_out;
    }

	priv->interface_up = 1;
    return 0;

open_err_out:
    stop(dev);

    return status;
}

#if defined(CONFIG_LEON_COPRO) && defined(CONFIG_LEON_OFFLOAD_TX)
static int hard_start_xmit(
    struct sk_buff *skb,
    struct net_device *dev)
{
    gmac_priv_t                *priv = (gmac_priv_t*)netdev_priv(dev);
    volatile gmac_tx_que_ent_t *job;
    unsigned long               irq_flags;

    if (skb_shinfo(skb)->frag_list) {
        panic("Frag list - can't handle this!\n");
    }

    // Protection against concurrent operations in ISR and hard_start_xmit()
    if (!spin_trylock_irqsave(&priv->tx_spinlock_, irq_flags)) {
        return NETDEV_TX_LOCKED;
    }

    // NETIF_F_LLTX apparently introduces a potential for hard_start_xmit() to
    // be called when the queue has been stopped (although I think only in SMP)
    // so do a check here to make sure we should proceed
    if (unlikely(netif_queue_stopped(dev))) {
        spin_unlock_irqrestore(&priv->tx_spinlock_, irq_flags);
        return NETDEV_TX_BUSY;
    }

    job = tx_que_get_idle_job(dev);
    if (unlikely(!job)) {
		netif_stop_queue(dev);
        spin_unlock_irqrestore(&priv->tx_spinlock_, irq_flags);
        return NETDEV_TX_BUSY;
    }

	// Fill the Tx offload job with the network packet's details
	copro_fill_tx_job(job, skb);

	// Enqueue the new Tx offload job with the CoPro
	tx_que_new_job(dev, job);

	// Record start of transmission, so timeouts will work once they're
	// implemented
	dev->trans_start = jiffies;

	// Interrupt the CoPro to cause it to examine the Tx offload queue
	wmb();
	writel(1UL << COPRO_SEM_INT_TX, SYS_CTRL_SEMA_SET_CTRL);

    spin_unlock_irqrestore(&priv->tx_spinlock_, irq_flags);

    return NETDEV_TX_OK;
}
#else
static inline void unmap_fragments(
    tx_frag_info_t *frags,
    int             count)
{
    while (count--) {
        dma_unmap_single(0, frags->phys_adr, frags->length, DMA_TO_DEVICE);
        ++frags;
    }
}

static int hard_start_xmit(
    struct sk_buff    *skb,
    struct net_device *dev)
{
    gmac_priv_t            *priv = (gmac_priv_t*)netdev_priv(dev);
    unsigned long           irq_flags;
    struct skb_shared_info *shinfo = skb_shinfo(skb);
    int                     fragment_count = shinfo->nr_frags + 1;
    tx_frag_info_t          fragments[fragment_count];
    int                     frag_index;

    // Get consistent DMA mappings for the SDRAM to be DMAed from by the GMAC,
    // causing a flush from the CPU's cache to the memory.

    // Do the DMA mappings before acquiring the tx lock, even though it complicates
    // the later code, as this can be a long operation involving cache flushing

    // Map the main buffer
    fragments[0].length = skb_headlen(skb);
    fragments[0].phys_adr = dma_map_single(0, skb->data, skb_headlen(skb), DMA_TO_DEVICE);
    BUG_ON(dma_mapping_error(fragments[0].phys_adr));

    // Map any SG fragments
    for (frag_index = 0; frag_index < shinfo->nr_frags; ++frag_index) {
        skb_frag_t *frag = &shinfo->frags[frag_index];

        fragments[frag_index + 1].length = frag->size;
        fragments[frag_index + 1].phys_adr = dma_map_page(0, frag->page, frag->page_offset, frag->size, DMA_TO_DEVICE);
        BUG_ON(dma_mapping_error(fragments[frag_index + 1].phys_adr));
    }

    // Protection against concurrent operations in ISR and hard_start_xmit()
    if (unlikely(!spin_trylock_irqsave(&priv->tx_spinlock_, irq_flags))) {
        unmap_fragments(fragments, fragment_count);
        return NETDEV_TX_LOCKED;
    }

    // NETIF_F_LLTX apparently introduces a potential for hard_start_xmit() to
    // be called when the queue has been stopped (although I think only in SMP)
    // so do a check here to make sure we should proceed
    if (unlikely(netif_queue_stopped(dev))) {
        unmap_fragments(fragments, fragment_count);
        spin_unlock_irqrestore(&priv->tx_spinlock_, irq_flags);
        return NETDEV_TX_BUSY;
    }

    // Construct the GMAC DMA descriptor
    if (unlikely(set_tx_descriptor(priv,
                          skb,
                          fragments,
                          fragment_count,
                          skb->ip_summed == CHECKSUM_PARTIAL) < 0)) {
        // Shouldn't see a full ring without the queue having already been
        // stopped, and the queue should already have been stopped if we have
        // already queued a single pending packet
        if (priv->tx_pending_skb) {
            printk(KERN_WARNING "hard_start_xmit() Ring full and pending packet already queued\n");
            unmap_fragments(fragments, fragment_count);
            spin_unlock_irqrestore(&priv->tx_spinlock_, irq_flags);
            return NETDEV_TX_BUSY;
        }

        // Should keep a record of the skb that we haven't been able to queue
        // for transmission and queue it as soon as a descriptor becomes free
        priv->tx_pending_skb = skb;
        priv->tx_pending_fragment_count = fragment_count;

        // Copy the fragment info to the allocated storage
        memcpy(priv->tx_pending_fragments, fragments, sizeof(tx_frag_info_t) * fragment_count);

        // Stop further calls to hard_start_xmit() until some descriptors are
        // freed up by already queued TX packets being completed
        netif_stop_queue(dev);
    } else {
        // Record start of transmission, so timeouts will work once they're
        // implemented
        dev->trans_start = jiffies;

        // Poke the transmitter to look for available TX descriptors, as we have
        // just added one, in case it had previously found there were no more
        // pending transmission
        dma_reg_write(priv, DMA_TX_POLL_REG, 0);
    }

    spin_unlock_irqrestore(&priv->tx_spinlock_, irq_flags);

    return NETDEV_TX_OK;
}
#endif // CONFIG_LEON_COPRO && CONFIG_LEON_OFFLOAD_TX

static struct net_device_stats *get_stats(struct net_device *dev)
{
    gmac_priv_t* priv = (gmac_priv_t*)netdev_priv(dev);
    return &priv->stats;
}

#ifdef CONFIG_NET_POLL_CONTROLLER
/**
 * Polling 'interrupt' - used by things like netconsole to send skbs without
 * having to re-enable interrupts. It's not called while the interrupt routine
 * is executing.
 */
static void netpoll(struct net_device *netdev)
{
    disable_irq(netdev->irq);
    int_handler(netdev->irq, netdev, NULL);
    enable_irq(netdev->irq);
}
#endif // CONFIG_NET_POLL_CONTROLLER

static int probe(
    struct net_device      *netdev,
    struct platform_device *plat_device,
    u32                     vaddr,
    u32                     irq,
    int                     copro_a_irq,
    int                     copro_b_irq)
{
    int err = 0;
    u32 version;
    int i;
    unsigned synopsis_version;
    unsigned vendor_version;
    gmac_priv_t* priv;
    u32 reg_contents;

    // Ensure the MAC block is properly reset
    writel(1UL << SYS_CTRL_RSTEN_MAC_BIT, SYS_CTRL_RSTEN_SET_CTRL);
    writel(1UL << SYS_CTRL_RSTEN_MAC_BIT, SYS_CTRL_RSTEN_CLR_CTRL);

    // Enable the clock to the MAC block
    writel(1UL << SYS_CTRL_CKEN_MAC_BIT, SYS_CTRL_CKEN_SET_CTRL);

    // Ensure reset and clock operations are complete
    wmb();

    // Ensure all of the device private data are zero, so we can clean up in
    // the event of a later failure to initialise all fields
    priv = (gmac_priv_t*)netdev_priv(netdev);
    memset(priv, 0, sizeof(gmac_priv_t));

    // No debug messages allowed
    priv->msg_level = 0UL;

    // Initialise the ISR/hard_start_xmit() lock
    spin_lock_init(&priv->tx_spinlock_);
    
    // Initialise the PHY access lock
    spin_lock_init(&priv->phy_lock);

    // Set hardware device base addresses
    priv->macBase = vaddr + MAC_BASE_OFFSET;
    priv->dmaBase = vaddr + DMA_BASE_OFFSET;

    // Initialise IRQ ownership to not owned
    priv->have_irq = 0;

    // Lock protecting access to CoPro command queue functions or direct access
    // to the GMAC interrupt enable register if CoPro is not in use
    spin_lock_init(&priv->cmd_que_lock_);

#ifdef CONFIG_LEON_COPRO
    sema_init(&copro_stop_semaphore, 0);
    sema_init(&copro_start_semaphore, 0);
    sema_init(&copro_int_clr_semaphore, 0);
    sema_init(&copro_update_semaphore, 0);
    sema_init(&copro_rx_enable_semaphore, 0);
    priv->copro_a_irq_alloced_ = 0;
    priv->copro_b_irq_alloced_ = 0;
    sema_init(&priv->copro_stop_complete_semaphore_, 0);
#endif // CONFIG_LEON_COPRO

    init_timer(&priv->watchdog_timer);
    priv->watchdog_timer.function = &watchdog_timer_action;
    priv->watchdog_timer.data = (unsigned long)priv;

    // Set pointer to device in private data
    priv->netdev = netdev;
    priv->plat_dev = plat_device;
    priv->device = &plat_device->dev;

    /** Do something here to detect the present or otherwise of the MAC
     *  Read the version register as a first test */
    version = mac_reg_read(priv, MAC_VERSION_REG);
    synopsis_version = version & 0xff;
    vendor_version   = (version >> 8) & 0xff;

    /** Assume device is at the adr and irq specified until have probing working */
    netdev->base_addr  = vaddr;
    netdev->irq        = irq;
#ifdef CONFIG_LEON_COPRO
    priv->copro_a_irq_ = copro_a_irq;
    priv->copro_b_irq_ = copro_b_irq;
#endif // CONFIG_LEON_COPRO

#ifdef CONFIG_LEON_COPRO
    // Allocate the CoPro A IRQ
    err = request_irq(priv->copro_a_irq_, &copro_sema_intr, 0, "SEMA", netdev);
    if (err) {
        printk(KERN_ERR "probe() %s: Failed to allocate CoPro irq A (%d)\n", netdev->name, priv->copro_a_irq_);
        goto probe_err_out;
    }
    // Release the CoPro A IRQ again, as open()/stop() should manage IRQ ownership
    free_irq(priv->copro_a_irq_, netdev);

    // Allocate the CoPro B IRQ
    err = request_irq(priv->copro_b_irq_, &copro_semb_intr, 0, "SEMB", netdev);
    if (err) {
        printk(KERN_ERR "probe() %s: Failed to allocate CoPro irq B (%d)\n", netdev->name, priv->copro_b_irq_);
        goto probe_err_out;
    }
    // Release the CoPro B IRQ again, as open()/stop() should manage IRQ ownership
    free_irq(priv->copro_b_irq_, netdev);
#else // CONFIG_LEON_COPRO
    // Allocate the IRQ
    err = request_irq(netdev->irq, &int_handler, 0, netdev->name, netdev);
    if (err, err) {
        printk(KERN_ERR "probe() %s: Failed to allocate irq %d\n", netdev->name, netdev->irq);
        goto probe_err_out;
    }

    // Release the IRQ again, as open()/stop() should manage IRQ ownership
    free_irq(netdev->irq, netdev);
#endif // CONFIG_LEON_COPRO

    // Initialise the ethernet device with std. contents
    ether_setup(netdev);

    // Tell the kernel of our MAC address
	for (i = 0; i < netdev->addr_len; i++) {
		netdev->dev_addr[i] = (unsigned char)mac_adr[i];
	}

    // Setup operations pointers
    netdev->open               = &open;
    netdev->hard_start_xmit    = &hard_start_xmit;
    netdev->stop               = &stop;
    netdev->get_stats          = &get_stats;
    netdev->change_mtu         = &change_mtu;
#ifdef CONFIG_NET_POLL_CONTROLLER
    netdev->poll_controller    = &netpoll;
#endif // CONFIG_NET_POLL_CONTROLLER
    netdev->set_mac_address    = &set_mac_address;
    netdev->set_multicast_list = &set_multicast_list;

	// Initialise NAPI support
	netif_napi_add(netdev, &priv->napi_struct, &poll, NAPI_POLL_WEIGHT);

    set_ethtool_ops(netdev);

    if (debug) {
      netdev->flags |= IFF_DEBUG;
    }

#if defined(CONFIG_LEON_COPRO) && defined(CONFIG_LEON_OFFLOAD_TX)
    // Do TX H/W checksum and SG list processing
    netdev->features |= NETIF_F_HW_CSUM;
    netdev->features |= NETIF_F_SG;

    // Do hardware TCP/IP Segmentation Offload
    netdev->features |= NETIF_F_TSO;
#elif !defined(CONFIG_LEON_COPRO) && !defined(CONFIG_OXNAS_VERSION_0X800)
    // Do TX H/W checksum and SG list processing
    netdev->features |= NETIF_F_HW_CSUM;
    netdev->features |= NETIF_F_SG;
#endif // USE_TX_CSUM

    // We take care of our own TX locking
    netdev->features |= NETIF_F_LLTX;

    // Initialise PHY support
    priv->mii.phy_id_mask   = 0x1f;
    priv->mii.reg_num_mask  = 0x1f;
    priv->mii.force_media   = 0;
    priv->mii.full_duplex   = 1;
    priv->mii.using_100     = 0;
    priv->mii.using_1000    = 1;
	priv->mii.using_pause   = 1;
    priv->mii.dev           = netdev;
    priv->mii.mdio_read     = phy_read;
    priv->mii.mdio_write    = phy_write;

    priv->gmii_csr_clk_range = 5;   // Slowest for now

    // Use simple mux for 25/125 Mhz clock switching and
    // enable GMII_GTXCLK to follow GMII_REFCLK - required for gigabit PHY
	reg_contents = readl(SYS_CTRL_GMAC_CTRL);
	reg_contents |= ((1UL << SYS_CTRL_GMAC_SIMPLE_MAX) |
					  (1UL << SYS_CTRL_GMAC_CKEN_GTX));
    writel(reg_contents, SYS_CTRL_GMAC_CTRL);

    // Remember whether auto-negotiation is allowed
#ifdef ALLOW_AUTONEG
    priv->ethtool_cmd.autoneg = 1;
	priv->ethtool_pauseparam.autoneg = 1;
#else // ALLOW_AUTONEG
    priv->ethtool_cmd.autoneg = 0;
	priv->ethtool_pauseparam.autoneg = 0;
#endif // ALLOW_AUTONEG

    // Set up PHY mode for when auto-negotiation is not allowed
    priv->ethtool_cmd.speed = SPEED_1000;
    priv->ethtool_cmd.duplex = DUPLEX_FULL;
    priv->ethtool_cmd.port = PORT_MII;
    priv->ethtool_cmd.transceiver = XCVR_INTERNAL;

#ifndef CONFIG_OXNAS_VERSION_0X800
	// We can support both reception and generation of pause frames
	priv->ethtool_pauseparam.rx_pause = 1;
	priv->ethtool_pauseparam.tx_pause = 1;
#endif // !CONFIG_OXNAS_VERSION_0X800

    // Initialise the set of features we would like to advertise as being
	// available for negotiation
    priv->ethtool_cmd.advertising = (ADVERTISED_10baseT_Half |
                                     ADVERTISED_10baseT_Full |
                                     ADVERTISED_100baseT_Half |
                                     ADVERTISED_100baseT_Full |
#if !defined(CONFIG_OXNAS_VERSION_0X800) || defined(ALLOW_OX800_1000M)
                                     ADVERTISED_1000baseT_Half |
                                     ADVERTISED_1000baseT_Full |
									  ADVERTISED_Pause |
									  ADVERTISED_Asym_Pause |
#endif
                                     ADVERTISED_Autoneg |
                                     ADVERTISED_MII);

    // Attempt to locate the PHY
    phy_detect(netdev);
    priv->ethtool_cmd.phy_address = priv->mii.phy_id;

    // Did we find a PHY?
	if (priv->phy_type == PHY_TYPE_NONE) {
		printk(KERN_WARNING "%s: No PHY found\n", netdev->name);
		err = ENXIO;
		goto probe_err_out;
    }

	// Setup the PHY
	initialise_phy(priv);

	// Find out what modes the PHY supports
	priv->ethtool_cmd.supported = get_phy_capabilies(priv);
#if defined(CONFIG_OXNAS_VERSION_0X800) && !defined(ALLOW_OX800_1000M)
	// OX800 has broken 1000M support in the MAC
	priv->ethtool_cmd.supported &= ~(SUPPORTED_1000baseT_Full | SUPPORTED_1000baseT_Half);
#endif

    // Register the device with the network intrastructure
    err = register_netdev(netdev);
    if (err) {
        printk(KERN_ERR "probe() %s: Failed to register device\n", netdev->name);
        goto probe_err_out;
    }

    // Record details about the hardware we found
    printk(KERN_NOTICE "%s: GMAC ver = %u, vendor ver = %u at 0x%lx, IRQ %d\n", netdev->name, synopsis_version, vendor_version, netdev->base_addr, netdev->irq);
#ifndef ARMULATING
    printk(KERN_NOTICE "%s: Found PHY at address %u, type 0x%08x -> %s\n", priv->netdev->name, priv->phy_addr, priv->phy_type, (priv->ethtool_cmd.supported & SUPPORTED_1000baseT_Full) ? "10/100/1000" : "10/100");
#endif // !ARMULATING
    printk(KERN_NOTICE "%s: Ethernet addr: ", priv->netdev->name);
    for (i = 0; i < 5; i++) {
        printk("%02x:", netdev->dev_addr[i]);
    }
    printk("%02x\n", netdev->dev_addr[5]);

#ifdef CONFIG_LEON_COPRO
    // Define sizes of queues for communicating with the CoPro
    priv->copro_cmd_que_num_entries_ = COPRO_CMD_QUEUE_NUM_ENTRIES;
    priv->copro_tx_que_num_entries_ = COPRO_TX_QUEUE_NUM_ENTRIES;

    // Zeroise the queues, so checks for empty etc will work before storage
    // for queue entries has been allocated
    tx_que_init(&priv->tx_queue_, 0, 0);
    cmd_que_init(&priv->cmd_queue_, 0, 0);
#endif // CONFIG_LEON_COPRO 

	priv->interface_up = 0;
    return 0;

probe_err_out:
#ifdef CONFIG_LEON_COPRO 
    shutdown_copro();

    if (priv->shared_copro_params_) {
        // Free the DMA coherent parameter space
        dma_free_coherent(0, sizeof(copro_params_t), priv->shared_copro_params_, priv->shared_copro_params_pa_);
        priv->shared_copro_params_ = 0;
    }
#endif // CONFIG_LEON_COPRO 

    // Disable the clock to the MAC block
    writel(1UL << SYS_CTRL_CKEN_MAC_BIT, SYS_CTRL_CKEN_CLR_CTRL);

    return err;
}

static int gmac_found_count = 0;
static struct net_device* gmac_netdev[MAX_GMAC_UNITS];

/**
 * External entry point to the driver, called from Space.c to detect a card
 */
struct net_device* __init synopsys_gmac_probe(int unit)
{
    int err = 0;
    struct net_device *netdev = alloc_etherdev(sizeof(gmac_priv_t));
    struct platform_device *pdev;

    printk(KERN_NOTICE "Probing for Synopsis GMAC, unit %d\n", unit);

    // Will allocate private data later, as may want descriptors etc in special memory
    if (!netdev) {
        printk(KERN_WARNING "synopsys_gmac_probe() failed to alloc device\n");
        err = -ENODEV;
    } else {
        if (unit >= 0) {
            sprintf(netdev->name, "eth%d", unit);

            netdev_boot_setup_check(netdev);

            if (gmac_found_count >= MAX_GMAC_UNITS) {
                err = -ENODEV;
            } else {
		pdev = platform_device_register_simple("gmac", unit, NULL, 0);
		if (IS_ERR(pdev)) {
                    printk(KERN_WARNING "synopsys_gmac_probe() Failed to register platform device %s, err = %d\n", netdev->name, err);
		} else {
                    err = probe(netdev, pdev, MAC_BASE, MAC_INTERRUPT, SEM_A_INTERRUPT, SEM_B_INTERRUPT);
                    if (err) {
                        printk(KERN_WARNING "synopsys_gmac_probe() Probing failed for %s, err = %d\n", netdev->name, err);
			platform_device_unregister(pdev);
                    } else {
                        ++gmac_found_count;
                    }
		}
            }
        }

        if (err) {
            netdev->reg_state = NETREG_UNREGISTERED;
            free_netdev(netdev);
        } else {
			gmac_netdev[unit] = netdev;
		}
    }

    return ERR_PTR(err);
}

static int __init gmac_module_init(void)
{
	int i;
	int err = platform_driver_register(&plat_driver);
	if (err) {
		printk(KERN_WARNING "gmac_module_init() Failed to register platform driver\n");
		return err;
	}

	for (i=0; i < MAX_GMAC_UNITS; i++) {
		err = (int)synopsys_gmac_probe(i);
		if (err) {
			printk(KERN_WARNING "gmac_module_init() Failed to register platform driver instance %d\n", i);
		}
	}

	if (!gmac_found_count) {
		platform_driver_unregister(&plat_driver);
	}

	return 0;
}
module_init(gmac_module_init);

static void __exit gmac_module_cleanup(void)
{
	int i;
	for (i=0; i < gmac_found_count; i++) {
		struct net_device* netdev = gmac_netdev[i];
		gmac_priv_t* priv = (gmac_priv_t*)netdev_priv(netdev);

		unregister_netdev(netdev);
		netdev->reg_state = NETREG_UNREGISTERED;
		free_netdev(netdev);
		platform_device_unregister(priv->plat_dev);
	}

	platform_driver_unregister(&plat_driver);
}
module_exit(gmac_module_cleanup);

