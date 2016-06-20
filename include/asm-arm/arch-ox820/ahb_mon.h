/* linux/include/asm-arm/arch-oxnas/ahb_mon.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifdef CONFIG_OXNAS_AHB_MON

#if !defined(__AHB_MON_H__)
#define __AHB_MON_H__

extern void init_ahb_monitors(
    AHB_MON_HWRITE_T ahb_mon_hwrite,
    unsigned hburst_mask,
    unsigned hburst_match,
    unsigned hprot_mask,
    unsigned hprot_match);
extern void restart_ahb_monitors(void);
extern void read_ahb_monitors(void);
#else // CONFIG_OXNAS_AHB_MON
#define init_ahb_monitors(a, b, c, d, e) {}
#define restart_ahb_monitors(x) {}
#define read_ahb_monitors(x) {}

#endif        //  #if !defined(__AHB_MON_H__)
#endif // CONFIG_OXNAS_AHB_MON

