/*
 * linux/include/asm-arm/arch-oxnas/sata.h
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
 * Definitions for using the SATA core in the ox800
 */

#ifndef __ASM_ARCH_SATA_H__
#define __ASM_ARCH_SATA_H__

/* number of ports per interface */
#define OX810SATA_MAX_PORTS 1


/** sata host port register offsets */
#define OX810SATA_ORB1          (0x00 / sizeof(u32))
#define OX810SATA_ORB2          (0x04 / sizeof(u32))
#define OX810SATA_ORB3          (0x08 / sizeof(u32))
#define OX810SATA_ORB4          (0x0C / sizeof(u32))
#define OX810SATA_ORB5          (0x10 / sizeof(u32))

#define OX810SATA_MASTER_STATUS  (0x10 / sizeof(u32))
#define OX810SATA_FIS_CTRL       (0x18 / sizeof(u32))
#define OX810SATA_FIS_DATA       (0x1C / sizeof(u32))

#define OX810SATA_INT_STATUS     (0x30 / sizeof(u32))
#define OX810SATA_INT_CLEAR      (0x30 / sizeof(u32))
#define OX810SATA_INT_ENABLE     (0x34 / sizeof(u32))
#define OX810SATA_INT_DISABLE    (0x38 / sizeof(u32))
#define OX810SATA_VERSION        (0x3C / sizeof(u32))
#define OX810SATA_SATA_CONTROL   (0x5C / sizeof(u32))
#define OX810SATA_SATA_COMMAND   (0x60 / sizeof(u32))
#define OX810SATA_HID_FEATURES   (0x64 / sizeof(u32))
#define OX810SATA_PORT_CONTROL   (0x68 / sizeof(u32))
#define OX810SATA_DRIVE_CONTROL  (0x6C / sizeof(u32))

/** These registers allow access to the link layer registers
that reside in a different clock domain to the processor bus */
#define OX810SATA_LINK_DATA      (0x70 / sizeof(u32))
#define OX810SATA_LINK_RD_ADDR   (0x74 / sizeof(u32))
#define OX810SATA_LINK_WR_ADDR   (0x78 / sizeof(u32))
#define OX810SATA_LINK_CONTROL   (0x7C / sizeof(u32))

/** Backup registers contain a copy of the command sent to the disk */
#define OX810SATA_BACKUP1        (0xB0 / sizeof(u32))
#define OX810SATA_BACKUP2        (0xB4 / sizeof(u32))
#define OX810SATA_BACKUP3        (0xB8 / sizeof(u32))
#define OX810SATA_BACKUP4        (0xBC / sizeof(u32))

/**
 * commands to issue in the master status to tell it to move shadow 
 * registers to the actual device 
 */
#define SATA_OPCODE_MASK                    0x00000007
#define CMD_WRITE_TO_ORB_REGS_NO_COMMAND    0x4
#define CMD_WRITE_TO_ORB_REGS               0x2
#define CMD_SYNC_ESCAPE                     0x7
#define CMD_CORE_BUSY                       (1 << 7)

/** interrupt bits */
#define OX810SATA_INT_END_OF_CMD      (1 << 0)
#define OX810SATA_INT_LINK_SERROR     (1 << 1)
#define OX810SATA_INT_ERROR           (1 << 2)
#define OX810SATA_INT_LINK_IRQ        (1 << 3)
#define OX810SATA_INT_REG_ACCESS_ERR  (1 << 7)
#define OX810SATA_INT_BIST_FIS        (1 << 11)
#define OX810SATA_INT_MASKABLE        (OX810SATA_INT_END_OF_CMD     |\
                                       OX810SATA_INT_LINK_SERROR    |\
                                       OX810SATA_INT_ERROR          |\
                                       OX810SATA_INT_LINK_IRQ       |\
                                       OX810SATA_INT_REG_ACCESS_ERR |\
                                       OX810SATA_INT_BIST_FIS       )

#define OX810SATA_INT_WANT            (OX810SATA_INT_END_OF_CMD  |\
                                       OX810SATA_INT_LINK_SERROR |\
                                       OX810SATA_INT_REG_ACCESS_ERR |\
                                       OX810SATA_INT_ERROR       )                                        
                                       
/** raw interrupt bits, unmaskable, but do not generate interrupts */
#define OX810SATA_RAW_END_OF_CMD      (OX810SATA_INT_END_OF_CMD     << 16)
#define OX810SATA_RAW_LINK_SERROR     (OX810SATA_INT_LINK_SERROR    << 16)
#define OX810SATA_RAW_ERROR           (OX810SATA_INT_ERROR          << 16)
#define OX810SATA_RAW_LINK_IRQ        (OX810SATA_INT_LINK_IRQ       << 16)
#define OX810SATA_RAW_REG_ACCESS_ERR  (OX810SATA_INT_REG_ACCESS_ERR << 16)
#define OX810SATA_RAW_BIST_FIS        (OX810SATA_INT_BIST_FIS       << 16)

/** SATA core register offsets */
#define OX810SATA_DM_DEBUG1           ( SATACORE_REGS_BASE + 0x000 )
#define OX810SATA_RAID_SET            ( SATACORE_REGS_BASE + 0x004 )
#define OX810SATA_DM_DEBUG2           ( SATACORE_REGS_BASE + 0x008 )
#define OX810SATA_CORE_ISR            ( SATACORE_REGS_BASE + 0x030 )
#define OX810SATA_CORE_IES            ( SATACORE_REGS_BASE + 0x034 )
#define OX810SATA_CORE_IEC            ( SATACORE_REGS_BASE + 0x038 )
#define OX810SATA_DEVICE_CONTROL      ( SATACORE_REGS_BASE + 0x068 )
#define OX810SATA_EXCESS              ( SATACORE_REGS_BASE + 0x06C )
#define OX810SATA_IDLE_STATUS         ( SATACORE_REGS_BASE + 0x07C )
#define OX810SATA_RAID_CONTROL        ( SATACORE_REGS_BASE + 0x090 )

/** sata core control register bits */
#define OX810SATA_SCTL_CLR_ERR        (0x00003016)

/* Interrupts direct from the ports */
#define OX810SATA_NORMAL_INTS_WANTED  (0x00000003)

/* Interrupts from the RAID controller only */
#define OX810SATA_RAID_INTS_WANTED    (0x00008000)

/* The bits in the OX810SATA_IDLE_STATUS that, when set indicate an idle core */
#define OX810SATA_IDLE_CORES          ((1 << 18) | (1 << 19))

/** Device Control register bits */
#define OX810SATA_DEVICE_CONTROL_ABORT (1 << 2)
#define OX810SATA_DEVICE_CONTROL_PAD   (1 << 3)
#define OX810SATA_DEVICE_CONTROL_PADPAT (1 << 16)

/** ORB4 register bits */
#define OX810SATA_ORB4_SRST	       (1 << 26)

/** standard HW raid flags */
#define OXNASSATA_NOTRAID 3
#define OXNASSATA_RAID1 1
#define OXNASSATA_RAID0 0
#define OXNASSATA_RAID_TWODISKS 3

/**
 * variables to write to the device control register to set the current device
 * ie, master or slave
 */
#define OX810SATA_DR_CON_48 2
#define OX810SATA_DR_CON_28 0

/** A ficticious device id used for matching device and driver */
#define OX810SATA_DEVICEID 0x00100002

/** The different Oxsemi SATA core version numbers */
#define OX810SATA_CORE_VERSION 0x1f2

/** Occasionally we get interrupts, even though there is no outstanding command,
these can be caused by dodgy SATA cables, this is a divider for reporting these
interrupts */
#define OX810SATA_NO_CMD_ERROR_RPT_COUNT 8
            
extern int oxnassata_RAID_faults( void );
extern int oxnassata_get_port_no(struct request_queue* );
extern int oxnassata_LBA_schemes_compatible( void );

#endif  /*  #if !defined(__ASM_ARCH_SATA_H__) */
