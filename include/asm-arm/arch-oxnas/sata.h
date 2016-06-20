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

#include <linux/blkdev.h>

/* number of ports per interface */
#define OX800SATA_MAX_PORTS 1


/** ata core register offsets */
#define OX800SATA_ORB1          (0x00 / sizeof(u32))
#define OX800SATA_ORB2          (0x04 / sizeof(u32))
#define OX800SATA_ORB3          (0x08 / sizeof(u32))
#define OX800SATA_ORB4          (0x0C / sizeof(u32))
#define OX800SATA_ORB5          (0x10 / sizeof(u32))

#define OX800SATA_MASTER_STATUS  (0x10 / sizeof(u32))
#define OX800SATA_DEVICE_CTRL    (0x18 / sizeof(u32))
#define OX800SATA_REG_ACCESS     (0x2c / sizeof(u32))
#define OX800SATA_INT_STATUS     (0x30 / sizeof(u32))
#define OX800SATA_INT_CLEAR      (0x30 / sizeof(u32))
#define OX800SATA_INT_ENABLE     (0x34 / sizeof(u32))
#define OX800SATA_INT_DISABLE    (0x38 / sizeof(u32))
#define OX800SATA_VERSION        (0x3C / sizeof(u32))
#define OX800SATA_SATA_CONTROL   (0x5C / sizeof(u32))
#define OX800SATA_SATA_COMMAND   (0x60 / sizeof(u32))
#define OX800SATA_DEVICE_SELECT  (0x64 / sizeof(u32))
#define OX800SATA_DEVICE_CONTROL (0x68 / sizeof(u32))
#define OX800SATA_DRIVE_CONTROL  (0x6C / sizeof(u32))

/** ata core registers that only work on port 0 */
#define OX800SATA_BURST_BUFFER  (0x1C / sizeof(u32))
#define OX800SATA_BURST_CONTROL (0x48 / sizeof(u32))
#define OX800SATA_RAID_CONTROL  (0x70 / sizeof(u32))




/** These registers allow access to the link layer registers
that reside in a different clock domain to the processor bus */
#define OX800SATA_LINK_DATA     (0x00000000)
#define OX800SATA_LINK_RD_ADDR  (0x00000001)
#define OX800SATA_LINK_WR_ADDR  (0x00000002)
#define OX800SATA_LINK_CONTROL  (0x00000003)

/**
 * commands to issue in the master status to tell it to move shhadow 
 * registers to the actual device 
 */
#define OX800SATA_MASTER_STATUS_WRITEOP 6 | (1 << 31)  
#define OX800SATA_MASTER_STATUS_READOP  5 | (1 << 31) | (1 << 29)
#define OX800SATA_MASTER_STATUS_ORBWRITEOP 1 | (1 << 31)  
#define OX800SATA_MASTER_STATUS_ORBWRITE_RUN  2 | (1 << 31) | (1 << 29)
#define OX800SATA_MASTER_STATUS_READY 64
#define OX800SATA_MASTER_STATUS_BUSY 128

#define OX800SATA_ORB2_SECTORS_MASK (0x0000ffff)

#define SATA_OPCODE_MASK                    0x00000003
#define CMD_WRITE_TO_ORB_REGS_NO_COMMAND    0x01
#define CMD_WRITE_TO_ORB_REGS               0x02
#define CMD_READ_ALL_REGISTERS              0x03
#define CMD_READ_STATUS_REG                 0x04
#define CMD_CORE_BUSY                       (1 << 7)

/** interrupt bits */
#define OX800SATA_INT_END_OF_CMD      (1 << 0)
#define OX800SATA_INT_END_OF_DATA_CMD (1 << 1)
#define OX800SATA_INT_ERROR           (1 << 2)
#define OX800SATA_INT_FIFO_EMPTY      (1 << 3)
#define OX800SATA_INT_FIFO_FULL       (1 << 4)
#define OX800SATA_INT_END_OF_TRANSF   (1 << 5)
#define OX800SATA_INT_MASKABLE        (OX800SATA_INT_END_OF_CMD      |\
                                       OX800SATA_INT_END_OF_DATA_CMD |\
                                       OX800SATA_INT_ERROR           |\
                                       OX800SATA_INT_FIFO_EMPTY      |\
                                       OX800SATA_INT_FIFO_FULL       |\
                                       OX800SATA_INT_END_OF_TRANSF   )

/** raw interrupt bits, unmaskable, but do not generate interrupts */
#define OX800SATA_RAW_END_OF_CMD      (1 << 8)
#define OX800SATA_RAW_END_OF_DATA_CMD (1 << 9)
#define OX800SATA_RAW_ERROR           (1 << 10)
#define OX800SATA_RAW_FIFO_EMPTY      (1 << 11)
#define OX800SATA_RAW_FIFO_FULL       (1 << 12)
#define OX800SATA_RAW_END_OF_TRANSF   (1 << 13)

/** burst buffer control bits */
#define OX800SATA_BBC_FORCE_EOT       (1 << 0)
#define OX800SATA_BBC_DIRECTION       (1 << 2)
#define OX800SATA_BBC_FIFO_DIS        (1 << 4)
#define OX800SATA_BBC_DREQ_DIS        (1 << 5)
#define OX800SATA_BBC_DREQ            (1 << 6)

/** sata control register bits */
#define OX800SATA_SCTL_RESET          (1 << 0)
#define OX800SATA_SCTL_ABORT          (1 << 2)

/** Device Control register bits */
#define OX800SATA_DEVICE_CONTROL_ABORT (1 << 2)

/** ORB4 register bits */
#define OX800SATA_ORB4_SRST	       (1 << 26)

/** SATA control transport state machine mask */
#define OX800SATA_SATA_CONTROL_TRANS_MASK  (0x0000001e)
#define OX800SATA_TRANS_CHECKTYPE     (0x00000008)
#define OX800SATA_TRANS_PIOITRANS     (0x00000018)
#define OX800SATA_TRANS_PIOOTRANS     (0x0000001C)

/** RAID control bit definitions */
#define OX800SATA_RAID_SPAN_EN        (1 <<  0)
#define OX800SATA_RAID_STRI_EN        (1 <<  1)
#define OX800SATA_RAID_STRI16         (1 <<  2)
#define OX800SATA_RAID_STRI32         (1 <<  3)
#define OX800SATA_RAID_STRI64         (1 <<  4)
#define OX800SATA_RAID_STRI128        (1 <<  5)
#define OX800SATA_RAID_STRI256        (1 <<  6)
#define OX800SATA_RAID_STRI512        (1 <<  7)
#define OX800SATA_RAID_STRI1M         (1 <<  8)
#define OX800SATA_RAID_STRI2M         (1 <<  9)
#define OX800SATA_RAID_STRI_TEST      (1 << 10)
#define OX800SATA_RAID_XSOFF2         (0 << 11)
#define OX800SATA_RAID_XSOFF4         (1 << 11)
#define OX800SATA_RAID_XSOFF6         (2 << 11)
#define OX800SATA_RAID_XSOFF8         (3 << 11)
#define OX800SATA_RAID_LOOP_BK        (1 << 12)
#define OX800SATA_RAID_MIR0           (0 << 13)
#define OX800SATA_RAID_MIR1           (1 << 13)
#define OX800SATA_RAID_MIRALT         (2 << 13)
#define OX800SATA_RAID_MIR_EN         (1 << 16)
#define OX800SATA_RAID_OVERLAP        (1 << 23)

/* standard HW raid flags */
#define OXNASSATA_RAID1 (OX800SATA_RAID_MIR0 | OX800SATA_RAID_MIR_EN | OX800SATA_RAID_OVERLAP )
#define OXNASSATA_RAID0 (OX800SATA_RAID_OVERLAP | OX800SATA_RAID_STRI_EN )
/**
 * variables to write to the device control register to set the current device
 * ie, master or slave
 */
#define OX800SATA_DEVICE_CONTROL_MASTER 0
#define OX800SATA_DEVICE_CONTROL_SLAVE  1

/** A ficticious device id used for matching device and driver */
#define OX800SATA_DEVICEID 0x00100001

/** The different Oxsemi SATA core version numbers */
#define OX800SATA_CORE_VERSION 0xf0

/** Occasionally we get interrupts, even though there is no outstanding command,
these can be caused by dodgy SATA cables, this is a divider for reporting these
interrupts */
#define OX800SATA_NO_CMD_ERROR_RPT_COUNT 8
            
extern int oxnassata_RAID_faults( void );
extern int oxnassata_get_port_no(struct request_queue* );
extern int oxnassata_LBA_schemes_compatible( void );

#endif  /*  #if !defined(__ASM_ARCH_SATA_H__) */
