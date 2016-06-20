/*
 * linux/include/asm-arm/arch-oxnas/cipher.h
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
 * Register locations in the cipher core
 *
 */

#ifndef __ASM_ARCH_CIPHER_H
#define __ASM_ARCH_CIPHER_H

#define OX800DPE_CTL_DIRECTION_ENC      0x002
#define OX800DPE_CTL_PRIMARY_IS_KEY3    0x004
#define OX800DPE_CTL_ENCRYPT_KEY        0x010
#define OX800DPE_CTL_ABORT              0x040
#define OX800DPE_CTL_MODE_ECB_AES       0x000
#define OX800DPE_CTL_MODE_LRW_AES       0x080   
#define OX800DPE_CTL_MODE_CBC_AES       0x100


#define OX800DPE_STAT_IDLE              0x1
#define OX800DPE_STAT_RX_SPACE          0x4
#define OX800DPE_STAT_TX_NOTEMPTY       0x8


#define OX800DPE_KEYSIZE 16
typedef volatile u32 oxnas_cipher_key_t[4];

#define OX800DPE_CONTROL     ((DPE_REGS_BASE) + 0x00)
#define OX800DPE_STATUS      ((DPE_REGS_BASE) + 0x04)
#define OX800DPE_KEY00       ((DPE_REGS_BASE) + 0x10)
#define OX800DPE_KEY01       ((DPE_REGS_BASE) + 0x14)
#define OX800DPE_KEY02       ((DPE_REGS_BASE) + 0x18)
#define OX800DPE_KEY03       ((DPE_REGS_BASE) + 0x1c)
#define OX800DPE_KEY10       ((DPE_REGS_BASE) + 0x20)
#define OX800DPE_KEY11       ((DPE_REGS_BASE) + 0x24)
#define OX800DPE_KEY12       ((DPE_REGS_BASE) + 0x28)
#define OX800DPE_KEY13       ((DPE_REGS_BASE) + 0x2c)
#define OX800DPE_KEY20       ((DPE_REGS_BASE) + 0x30)
#define OX800DPE_KEY21       ((DPE_REGS_BASE) + 0x34)
#define OX800DPE_KEY22       ((DPE_REGS_BASE) + 0x38)
#define OX800DPE_KEY23       ((DPE_REGS_BASE) + 0x3c)
#define OX800DPE_DATA_IN0    ((DPE_REGS_BASE) + 0x40)
#define OX800DPE_DATA_IN1    ((DPE_REGS_BASE) + 0x44)
#define OX800DPE_DATA_IN2    ((DPE_REGS_BASE) + 0x48)
#define OX800DPE_DATA_IN3    ((DPE_REGS_BASE) + 0x4c)
#define OX800DPE_DATA_OUT0   ((DPE_REGS_BASE) + 0x50)
#define OX800DPE_DATA_OUT1   ((DPE_REGS_BASE) + 0x54)
#define OX800DPE_DATA_OUT2   ((DPE_REGS_BASE) + 0x58)
#define OX800DPE_DATA_OUT3   ((DPE_REGS_BASE) + 0x5c)
#define OX800DPE_DATA_LRW0   ((DPE_REGS_BASE) + 0x60)
#define OX800DPE_DATA_LRW1   ((DPE_REGS_BASE) + 0x64)
#define OX800DPE_DATA_CBC0   ((DPE_REGS_BASE) + 0x68)
#define OX800DPE_DATA_CBC1   ((DPE_REGS_BASE) + 0x6c)


#define OX800IBW_STAT_AUTHENTICATED       0x10

#define OX800IBW_CONTROL	((IBW_REGS_BASE) + 0x00)
#define OX800IBW_STATUS		((IBW_REGS_BASE) + 0x04)
#define OX800IBW_SERIAL_LO	((IBW_REGS_BASE) + 0x08)
#define OX800IBW_SERIAL_HI	((IBW_REGS_BASE) + 0x0C)


// IBIW Control register bits

#define OX800IBW_CTRL_LOAD_AES_KEY 		0x00100
#define OX800IBW_CTRL_BUFF_WR_SRC		0x00200
#define OX800IBW_CTRL_CRC_WR_SRC		0x00400
#define OX800IBW_CTRL_RESET 			0x00800
#define OX800IBW_CTRL_WR 				0x01000
#define OX800IBW_CTRL_RD  				0x02000
#define OX800IBW_CTRL_RX_INIT 			0x04000
#define OX800IBW_CTRL_TX_INIT			0x08000
#define OX800IBW_CTRL_DONE  			0x10000
#define OX800IBW_CTRL_ENABLE  			0x20000


// IBIW Status register bits

#define OX800IBW_STATUS_PRESENT 		0x0001
#define OX800IBW_STATUS_ARRIVAL 		0x0002
#define OX800IBW_STATUS_DEPARTURE 		0x0004
#define OX800IBW_STATUS_HOLDOFF 		0x0008
#define OX800IBW_STATUS_CRC_OK 			0x0010
#define OX800IBW_STATUS_SERIAL_MATCH 	0x0020
#define OX800IBW_STATUS_KEY_MATCH 		0x0040
#define OX800IBW_STATUS_VALID_KEY	 	0x0080
#define OX800IBW_STATUS_AUTHENTICATED 	0x0100
#define OX800IBW_STATUS_RD_ENABLED		0x0200

/*The number of storage fields in a DS1991 iButton */
#define DS1991_IBUTTON_FIELDS 4
#define DS1991_PASSWORD_SIZE 8
#define DS1991_ID_SIZE 8
#define DS1991_DATA_SIZE 48
#define DS1991_PLAINTEXT_SIZE 64

/* DS1991 COMMANDS */
#define DS1991_WRITE_SCRATCHPAD 0x96
#define DS1991_READ_SCRATCHPAD  0x69
#define DS1991_COPY_SCRATCHPAD  0x3c
#define DS1991_READ_SUBKEY      0x66
#define DS1991_WRITE_SUBKEY     0x99
#define DS1991_WRITE_PASSWORD   0x5a

/* prototype */
struct scatterlist;

int ox800_aeslrw_encrypt(   struct scatterlist* in,
                            struct scatterlist* out,
                            unsigned int length,
                            u8  iv[],
                            u8 cipher_key[],
                            u8 tweak_key[]);
                            
int ox800_aeslrw_decrypt(   struct scatterlist* in,
                            struct scatterlist* out,
                            unsigned int length,
                            u8  iv[],
                            u8 cipher_key[], 
                            u8 tweak_key[]);

#endif
