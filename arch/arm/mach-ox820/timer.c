/*
 * linux/arch/arm/mach-ox820/timer.c
 *
 * Copyright (C) 2008 Oxford Semiconductor Ltd
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

#include <asm/arch/rps-timer.h>
#include <asm/mach/time.h>
#include <asm/arch/hardware.h>
 
static void /*__init*/ oxnas_init_time(void) {
    
#ifdef CONFIG_SMP
	local_timer_setup(smp_processor_id());
#endif
    /* initialise the RPS timer */
    oxnas_rps_init_time();

#ifdef CONFIG_LOCAL_TIMERS
    {
        extern void __iomem *twd_base_addr;
        extern unsigned int twd_size;
        /* initialise the local timers, These are global variables defined in
        localtimer.c */ 
        twd_base_addr = __io_address(OX820_TWD_CPU0_BASE);
        twd_size = OX820_ARM11MP_TWD_SIZE;
    }   
#endif //CONFIG_LOCAL_TIMERS
    
}
 
static unsigned long oxnas_gettimeoffset(void) {
    return oxnas_rps_gettimeoffset();
}

struct sys_timer oxnas_timer = {
    .init   = oxnas_init_time,
    .offset = oxnas_gettimeoffset,
};
