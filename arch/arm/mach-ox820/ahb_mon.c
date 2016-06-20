/*
 *  linux/arch/arm/mach-oxnas/ahb_mon.c
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

#include <linux/kernel.h>
#include <asm/io.h>
#include <asm/arch/hardware.h>

#ifdef CONFIG_OXNAS_AHB_MON
static void start_ahb_monitors(void)
{
    writel(AHB_MON_MODE_ACTIVE << AHB_MON_MODE_MODE_BIT, AHB_MON_ARM_D + AHB_MON_MODE_REG_OFFSET);
    writel(AHB_MON_MODE_ACTIVE << AHB_MON_MODE_MODE_BIT, AHB_MON_ARM_I + AHB_MON_MODE_REG_OFFSET);
    writel(AHB_MON_MODE_ACTIVE << AHB_MON_MODE_MODE_BIT, AHB_MON_DMA_A + AHB_MON_MODE_REG_OFFSET);
    writel(AHB_MON_MODE_ACTIVE << AHB_MON_MODE_MODE_BIT, AHB_MON_DMA_B + AHB_MON_MODE_REG_OFFSET);
    writel(AHB_MON_MODE_ACTIVE << AHB_MON_MODE_MODE_BIT, AHB_MON_LEON  + AHB_MON_MODE_REG_OFFSET);
    writel(AHB_MON_MODE_ACTIVE << AHB_MON_MODE_MODE_BIT, AHB_MON_USB  + AHB_MON_MODE_REG_OFFSET);
    writel(AHB_MON_MODE_ACTIVE << AHB_MON_MODE_MODE_BIT, AHB_MON_MAC  + AHB_MON_MODE_REG_OFFSET);
    writel(AHB_MON_MODE_ACTIVE << AHB_MON_MODE_MODE_BIT, AHB_MON_PCI  + AHB_MON_MODE_REG_OFFSET);
}

void init_ahb_monitors(
    AHB_MON_HWRITE_T ahb_mon_hwrite,
    unsigned hburst_mask,
    unsigned hburst_match,
    unsigned hprot_mask,
    unsigned hprot_match)
{
    u32 hburst_mask_value  = (hburst_mask  & ((1 << AHB_MON_HBURST_MASK_NUM_BITS)  - 1));
    u32 hburst_match_value = (hburst_match & ((1 << AHB_MON_HBURST_MATCH_NUM_BITS) - 1));
    u32 hprot_mask_value   = (hprot_mask   & ((1 << AHB_MON_HPROT_MASK_NUM_BITS)   - 1));
    u32 hprot_match_value  = (hprot_match  & ((1 << AHB_MON_HPROT_MATCH_NUM_BITS)  - 1));

    u32 hburst_reg_value = (hburst_mask_value << AHB_MON_HBURST_MASK_BIT) | (hburst_match_value << AHB_MON_HBURST_MATCH_BIT);
    u32 hprot_reg_value  = (hprot_mask_value  << AHB_MON_HPROT_MASK_BIT)  | (hprot_match_value  << AHB_MON_HPROT_MATCH_BIT);

printk("$Ghburst reg value = 0x%08x\n", hburst_reg_value);
printk("$Ghprot reg value  = 0x%08x\n", hprot_reg_value);

    // Reset all the counters and set their operating mode
    writel(AHB_MON_MODE_RESET << AHB_MON_MODE_MODE_BIT, AHB_MON_ARM_D + AHB_MON_MODE_REG_OFFSET);
    writel(ahb_mon_hwrite << AHB_MON_HWRITE_COUNT_BIT,  AHB_MON_ARM_D + AHB_MON_HWRITE_REG_OFFSET);
    writel(0UL,                                         AHB_MON_ARM_D + AHB_MON_HADDR_LOW_REG_OFFSET);
    writel(~0UL,                                        AHB_MON_ARM_D + AHB_MON_HADDR_HIGH_REG_OFFSET);
    writel(hburst_reg_value,                            AHB_MON_ARM_D + AHB_MON_HBURST_REG_OFFSET);
    writel(hprot_reg_value,                             AHB_MON_ARM_D + AHB_MON_HPROT_REG_OFFSET);

    writel(AHB_MON_MODE_RESET << AHB_MON_MODE_MODE_BIT, AHB_MON_ARM_I + AHB_MON_MODE_REG_OFFSET);
    writel(ahb_mon_hwrite << AHB_MON_HWRITE_COUNT_BIT,  AHB_MON_ARM_I + AHB_MON_HWRITE_REG_OFFSET);
    writel(0UL,                                         AHB_MON_ARM_I + AHB_MON_HADDR_LOW_REG_OFFSET);
    writel(~0UL,                                        AHB_MON_ARM_I + AHB_MON_HADDR_HIGH_REG_OFFSET);
    writel(hburst_reg_value,                            AHB_MON_ARM_I + AHB_MON_HBURST_REG_OFFSET);
    writel(hprot_reg_value,                             AHB_MON_ARM_I + AHB_MON_HPROT_REG_OFFSET);

    writel(AHB_MON_MODE_RESET << AHB_MON_MODE_MODE_BIT, AHB_MON_DMA_A + AHB_MON_MODE_REG_OFFSET);
    writel(ahb_mon_hwrite << AHB_MON_HWRITE_COUNT_BIT,  AHB_MON_DMA_A + AHB_MON_HWRITE_REG_OFFSET);
    writel(0UL,                                         AHB_MON_DMA_A + AHB_MON_HADDR_LOW_REG_OFFSET);
    writel(~0UL,                                        AHB_MON_DMA_A + AHB_MON_HADDR_HIGH_REG_OFFSET);
    writel(hburst_reg_value,                            AHB_MON_DMA_A + AHB_MON_HBURST_REG_OFFSET);
    writel(hprot_reg_value,                             AHB_MON_DMA_A + AHB_MON_HPROT_REG_OFFSET);

    writel(AHB_MON_MODE_RESET << AHB_MON_MODE_MODE_BIT, AHB_MON_DMA_B + AHB_MON_MODE_REG_OFFSET);
    writel(ahb_mon_hwrite << AHB_MON_HWRITE_COUNT_BIT,  AHB_MON_DMA_B + AHB_MON_HWRITE_REG_OFFSET);
    writel(0UL,                                         AHB_MON_DMA_B + AHB_MON_HADDR_LOW_REG_OFFSET);
    writel(~0UL,                                        AHB_MON_DMA_B + AHB_MON_HADDR_HIGH_REG_OFFSET);
    writel(hburst_reg_value,                            AHB_MON_DMA_B + AHB_MON_HBURST_REG_OFFSET);
    writel(hprot_reg_value,                             AHB_MON_DMA_B + AHB_MON_HPROT_REG_OFFSET);

    writel(AHB_MON_MODE_RESET << AHB_MON_MODE_MODE_BIT, AHB_MON_LEON  + AHB_MON_MODE_REG_OFFSET);
    writel(ahb_mon_hwrite << AHB_MON_HWRITE_COUNT_BIT,  AHB_MON_LEON  + AHB_MON_HWRITE_REG_OFFSET);
    writel(0UL,                                         AHB_MON_LEON  + AHB_MON_HADDR_LOW_REG_OFFSET);
    writel(~0UL,                                        AHB_MON_LEON  + AHB_MON_HADDR_HIGH_REG_OFFSET);
    writel(hburst_reg_value,                            AHB_MON_LEON  + AHB_MON_HBURST_REG_OFFSET);
    writel(hprot_reg_value,                             AHB_MON_LEON  + AHB_MON_HPROT_REG_OFFSET);

    writel(AHB_MON_MODE_RESET << AHB_MON_MODE_MODE_BIT, AHB_MON_USB   + AHB_MON_MODE_REG_OFFSET);
    writel(ahb_mon_hwrite << AHB_MON_HWRITE_COUNT_BIT,  AHB_MON_USB   + AHB_MON_HWRITE_REG_OFFSET);
    writel(0UL,                                         AHB_MON_USB   + AHB_MON_HADDR_LOW_REG_OFFSET);
    writel(~0UL,                                        AHB_MON_USB   + AHB_MON_HADDR_HIGH_REG_OFFSET);
    writel(hburst_reg_value,                            AHB_MON_USB   + AHB_MON_HBURST_REG_OFFSET);
    writel(hprot_reg_value,                             AHB_MON_USB   + AHB_MON_HPROT_REG_OFFSET);

    writel(AHB_MON_MODE_RESET << AHB_MON_MODE_MODE_BIT, AHB_MON_MAC   + AHB_MON_MODE_REG_OFFSET);
    writel(ahb_mon_hwrite << AHB_MON_HWRITE_COUNT_BIT,  AHB_MON_MAC   + AHB_MON_HWRITE_REG_OFFSET);
    writel(0UL,                                         AHB_MON_MAC   + AHB_MON_HADDR_LOW_REG_OFFSET);
    writel(~0UL,                                        AHB_MON_MAC   + AHB_MON_HADDR_HIGH_REG_OFFSET);
    writel(hburst_reg_value,                            AHB_MON_MAC   + AHB_MON_HBURST_REG_OFFSET);
    writel(hprot_reg_value,                             AHB_MON_MAC   + AHB_MON_HPROT_REG_OFFSET);

    writel(AHB_MON_MODE_RESET << AHB_MON_MODE_MODE_BIT, AHB_MON_PCI   + AHB_MON_MODE_REG_OFFSET);
    writel(ahb_mon_hwrite << AHB_MON_HWRITE_COUNT_BIT,  AHB_MON_PCI   + AHB_MON_HWRITE_REG_OFFSET);
    writel(0UL,                                         AHB_MON_PCI   + AHB_MON_HADDR_LOW_REG_OFFSET);
    writel(~0UL,                                        AHB_MON_PCI   + AHB_MON_HADDR_HIGH_REG_OFFSET);
    writel(hburst_reg_value,                            AHB_MON_PCI   + AHB_MON_HBURST_REG_OFFSET);
    writel(hprot_reg_value,                             AHB_MON_PCI   + AHB_MON_HPROT_REG_OFFSET);

    // Start all the counters
    start_ahb_monitors();
}

void restart_ahb_monitors(void)
{
    // Reset the counters
    writel(AHB_MON_MODE_RESET << AHB_MON_MODE_MODE_BIT, AHB_MON_ARM_D + AHB_MON_MODE_REG_OFFSET);
    writel(AHB_MON_MODE_RESET << AHB_MON_MODE_MODE_BIT, AHB_MON_ARM_I + AHB_MON_MODE_REG_OFFSET);
    writel(AHB_MON_MODE_RESET << AHB_MON_MODE_MODE_BIT, AHB_MON_DMA_A + AHB_MON_MODE_REG_OFFSET);
    writel(AHB_MON_MODE_RESET << AHB_MON_MODE_MODE_BIT, AHB_MON_DMA_B + AHB_MON_MODE_REG_OFFSET);
    writel(AHB_MON_MODE_RESET << AHB_MON_MODE_MODE_BIT, AHB_MON_LEON  + AHB_MON_MODE_REG_OFFSET);
    writel(AHB_MON_MODE_RESET << AHB_MON_MODE_MODE_BIT, AHB_MON_USB   + AHB_MON_MODE_REG_OFFSET);
    writel(AHB_MON_MODE_RESET << AHB_MON_MODE_MODE_BIT, AHB_MON_MAC   + AHB_MON_MODE_REG_OFFSET);
    writel(AHB_MON_MODE_RESET << AHB_MON_MODE_MODE_BIT, AHB_MON_PCI   + AHB_MON_MODE_REG_OFFSET);

    // Start the counters
    start_ahb_monitors();
}

void read_ahb_monitors(void)
{
    // Prepare the counters for reading
    writel(AHB_MON_MODE_IDLE << AHB_MON_MODE_MODE_BIT, AHB_MON_ARM_D + AHB_MON_MODE_REG_OFFSET);
    writel(AHB_MON_MODE_IDLE << AHB_MON_MODE_MODE_BIT, AHB_MON_ARM_I + AHB_MON_MODE_REG_OFFSET);
    writel(AHB_MON_MODE_IDLE << AHB_MON_MODE_MODE_BIT, AHB_MON_DMA_A + AHB_MON_MODE_REG_OFFSET);
    writel(AHB_MON_MODE_IDLE << AHB_MON_MODE_MODE_BIT, AHB_MON_DMA_B + AHB_MON_MODE_REG_OFFSET);
    writel(AHB_MON_MODE_IDLE << AHB_MON_MODE_MODE_BIT, AHB_MON_LEON  + AHB_MON_MODE_REG_OFFSET);
    writel(AHB_MON_MODE_IDLE << AHB_MON_MODE_MODE_BIT, AHB_MON_USB   + AHB_MON_MODE_REG_OFFSET);
    writel(AHB_MON_MODE_IDLE << AHB_MON_MODE_MODE_BIT, AHB_MON_MAC   + AHB_MON_MODE_REG_OFFSET);
    writel(AHB_MON_MODE_IDLE << AHB_MON_MODE_MODE_BIT, AHB_MON_PCI   + AHB_MON_MODE_REG_OFFSET);

    // Read the counters
    printk("ARM-D: B=%u, T=%u, W=%u\n", readl(AHB_MON_ARM_D + AHB_MON_CYCLES_REG_OFFSET),
                                        readl(AHB_MON_ARM_D + AHB_MON_TRANSFERS_REG_OFFSET),
                                        readl(AHB_MON_ARM_D + AHB_MON_WAITS_REG_OFFSET));

    printk("ARM-I: B=%u, T=%u, W=%u\n", readl(AHB_MON_ARM_I + AHB_MON_CYCLES_REG_OFFSET),
                                        readl(AHB_MON_ARM_I + AHB_MON_TRANSFERS_REG_OFFSET),
                                        readl(AHB_MON_ARM_I + AHB_MON_WAITS_REG_OFFSET));

    printk("DMA-A: B=%u, T=%u, W=%u\n", readl(AHB_MON_DMA_A + AHB_MON_CYCLES_REG_OFFSET),
                                        readl(AHB_MON_DMA_A + AHB_MON_TRANSFERS_REG_OFFSET),
                                        readl(AHB_MON_DMA_A + AHB_MON_WAITS_REG_OFFSET));

    printk("DMA-B: B=%u, T=%u, W=%u\n", readl(AHB_MON_DMA_B + AHB_MON_CYCLES_REG_OFFSET),
                                        readl(AHB_MON_DMA_B + AHB_MON_TRANSFERS_REG_OFFSET),
                                        readl(AHB_MON_DMA_B + AHB_MON_WAITS_REG_OFFSET));

    printk("LEON:  B=%u, T=%u, W=%u\n", readl(AHB_MON_LEON  + AHB_MON_CYCLES_REG_OFFSET),
                                        readl(AHB_MON_LEON  + AHB_MON_TRANSFERS_REG_OFFSET),
                                        readl(AHB_MON_LEON  + AHB_MON_WAITS_REG_OFFSET));

    printk("USB:   B=%u, T=%u, W=%u\n", readl(AHB_MON_USB   + AHB_MON_CYCLES_REG_OFFSET),
                                        readl(AHB_MON_USB   + AHB_MON_TRANSFERS_REG_OFFSET),
                                        readl(AHB_MON_USB   + AHB_MON_WAITS_REG_OFFSET));

    printk("MAC:   B=%u, T=%u, W=%u\n", readl(AHB_MON_MAC   + AHB_MON_CYCLES_REG_OFFSET),
                                        readl(AHB_MON_MAC   + AHB_MON_TRANSFERS_REG_OFFSET),
                                        readl(AHB_MON_MAC   + AHB_MON_WAITS_REG_OFFSET));

    printk("PCI:   B=%u, T=%u, W=%u\n", readl(AHB_MON_PCI   + AHB_MON_CYCLES_REG_OFFSET),
                                        readl(AHB_MON_PCI   + AHB_MON_TRANSFERS_REG_OFFSET),
                                        readl(AHB_MON_PCI   + AHB_MON_WAITS_REG_OFFSET));
}
#endif // CONFIG_OXNAS_AHB_MON

