/*
 * linux/arch/arm/mach-oxnas/leon.h
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
#ifdef CONFIG_SUPPORT_LEON

#if !defined(__LEON_H__)
#define __LEON_H__

/**
 * Load the LEON's program image into the memory as defined by the s-records
 * holding the LEON program image, and begin execution at the start address
 * defined by the s-records
 */
extern void init_copro(const s8 *srec, unsigned long arg);

extern void shutdown_copro(void);

#endif        //  #if !defined(__LEON_H__)
#endif // CONFIG_SUPPORT_LEON

