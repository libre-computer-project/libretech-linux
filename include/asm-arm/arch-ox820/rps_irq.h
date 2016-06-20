/*
 *  linux/include/asm-arm/arch-ox820/rps_irq.h
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
#include <linux/init.h>
#include <linux/list.h>
#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>

struct OX820_RPS_chip_data {
	unsigned int irq_offset;
};

void __init OX820_RPS_init_irq(unsigned int start, unsigned int end) ;
void __init OX820_RPS_cascade_irq(unsigned int irq);
