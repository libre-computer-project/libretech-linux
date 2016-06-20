/*
 * linux/arch/arm/mach-oxnas/leon.c
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

#include <asm/io.h>
#include <asm/types.h>
#include <asm/plat-oxnas/hardware.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <asm/plat-oxnas/leon.h>

static u8 asciihex_to_decimal(u8 ascii)
{
    return isdigit(ascii) ? (ascii - '0') : (isalpha(ascii) ? ((toupper(ascii) - 'A') + 10)  : 0);
}

static u8 srec_read_u8(const s8** srec)
{
    u8 first_ascii  = **srec;
    u8 second_ascii = *++*srec;
    ++*srec;
    return ((asciihex_to_decimal(first_ascii) << 4) | asciihex_to_decimal(second_ascii));
}

static u32 srec_read_u32(const s8** srec)
{
    u32 word  = ((u32)srec_read_u8(srec) << 24);
    word     |= ((u32)srec_read_u8(srec) << 16);
    word     |= ((u16)srec_read_u8(srec) << 8);
    word     |= srec_read_u8(srec);
    return word;
}

static void skip_to_next_record(const s8** srec)
{
    while (*++*srec != '\n');
    ++*srec;
}

/**
 * @param  srec An const s8** pointing to the position in the input s-record
 *         array at which to begin parsing
 * @param  buf An u8* into which any extracted record in to be placed
 * @param  adr An u8** into which either the extracted record's load address is
 *         to be written, or the execution start address
 * @param  len An u8* into which the length in bytes of the extracted record is
 *         to be written
 * @return An int which is zero if another record is available, else if non-zero
 *         indicated that the execution start address is available in the
 *         adr argument
 */
static void read_record(u8 len, const s8** srec, u8* buf)
{
    int quads = len/sizeof(u32);
    int spare = len - (quads*sizeof(u32));

    int i=0;
    while (i < quads) {
        ((u32*)buf)[i++] = srec_read_u32(srec);
    }
    i = len-spare;
    while (i < len) {
        buf[i++] = srec_read_u8(srec);
    }
}

static int get_next_record(const s8** srec, u8* buf, u8** adr, u8* len)
{
    int again;
    int last = 1;

    *adr = 0;
    do {
        again = 0;
        if (**srec == 'S') {
            switch (*++*srec) {
                case '0':
                    skip_to_next_record(srec);
                    again = 1;
                    break;
                case '3':
                    ++*srec;
                    *len = srec_read_u8(srec) - sizeof(u32) - 1;
                    *adr = (u8*)srec_read_u32(srec);
                    read_record(*len, srec, buf);
                    skip_to_next_record(srec);
                    last = 0;
                    break;
                case '7':
                    ++*srec;
                    *len = srec_read_u8(srec) - 1;
                    if (*len >= sizeof(u32)) {
                        *adr = (u8*)srec_read_u32(srec);
                    }
                    break;
                default:
                    break;
            }
        }
    } while (again);

    return last;
}

static const u32 ENDIAN_LITTLE_READ_BIT = 30;
static const u32 ENDIAN_BIG_WRITE_BIT   = 31;

static u8* convert_adr_to_virt(u8* adr)
{
	static const u32 ARM_HIGH_ORDER_ADR_BIT = 30;

    u32 virt = (u32)adr;

	// Zero the Leon endian control bits
	virt &= ~((1UL << ENDIAN_BIG_WRITE_BIT) | (1UL << ENDIAN_LITTLE_READ_BIT));

	// Convert to an ARM physical address
	virt |= (1UL << ARM_HIGH_ORDER_ADR_BIT);

    // Is address sane?
    if (virt < LEON_IMAGE_BASE_PA) {
        panic("CoPro SRAM load address 0x%08x below mapped region beginning at 0x%08lx\n", (u32)adr, LEON_IMAGE_BASE_PA);
    } else {
        virt -= LEON_IMAGE_BASE_PA;
        virt += LEON_IMAGE_BASE;
    }

    return (u8*)virt;
}

static void leon_load_image(const s8 *srec)
{
    u8       *buf;
    u8       *adr;
    u8        len;
    u32       code_base;

    // Copy each record to the specified address
    // Convert the LEON physical address to an ARM virtual address before
	// attempting to get the ARM to access it
    // NB must endian-swap any trailing non-quad multiple bytes, as LEON will
    // expect its instruction data in big endian format, whereas the ARM is
    // little endian
    buf  = kmalloc(512, GFP_KERNEL);
    while (!get_next_record(&srec, buf, &adr, &len)) {
        int i=0;
        int quads = len/sizeof(u32);
        int spare = len - (quads*sizeof(u32));
        int padded_len = len+(sizeof(u32)-spare);
        u32* quad_ptr;

        adr = convert_adr_to_virt(adr);

        quad_ptr = (u32*)adr;
        while (i < quads) {
            *quad_ptr++ = ((u32*)buf)[i++];
        }
        adr = (u8*)quad_ptr;
        for (i=len; i < padded_len; i++) {
            buf[i] = 0;
        }
        i = padded_len-1;
        while (i >= (len-spare)) {
            *adr++ = buf[i--];
        }
    }
    kfree(buf);

    // Start LEON execution at the address specified by the S-records, with
    // correct endianess. Use the address unchanged, as the LEON required
    // physical addresses and may make use of alternative upper nibble values
    code_base = (((u32)adr & ~((1UL << ENDIAN_BIG_WRITE_BIT) | (1UL << ENDIAN_LITTLE_READ_BIT))) | (1UL << ENDIAN_BIG_WRITE_BIT));

    // Set the LEON's start address
    printk(KERN_NOTICE "CoPro: Programming start address as 0x%08x (basic adr = 0x%08x)\n", code_base, (u32)adr);
    writel(code_base, SYS_CTRL_COPRO_CTRL);

    // Ensure start address has been loaded before release the LEON from reset
    wmb();
}

void init_copro(const s8 *srec, unsigned long arg)
{
    // Ensure the LEON is in reset
    writel(1UL << SYS_CTRL_RSTEN_COPRO_BIT, SYS_CTRL_RSTEN_SET_CTRL);

    // Enable the clock to the LEON
    writel(1UL << SYS_CTRL_CKEN_COPRO_BIT, SYS_CTRL_CKEN_SET_CTRL);

    // Ensure reset and clock operations are complete
    wmb();

    // Place LEON context argument in top quad of SRAM
	*((u32*)(LEON_IMAGE_BASE+LEON_IMAGE_SIZE-sizeof(u32))) = arg;

    // Load LEON's program and data and execution start address
    leon_load_image(srec);

    // Release the LEON from reset so it begins execution of the loaded code
    writel(1UL << SYS_CTRL_RSTEN_COPRO_BIT, SYS_CTRL_RSTEN_CLR_CTRL);

    // Give the LEON a chance to stabilise before giving it any commands
    mdelay(100);
    return;
}
EXPORT_SYMBOL_GPL(init_copro);

void shutdown_copro(void)
{
    // Ensure the LEON is in reset
    writel(1UL << SYS_CTRL_RSTEN_COPRO_BIT, SYS_CTRL_RSTEN_SET_CTRL);

    // Disable the clock to the LEON
    writel(1UL << SYS_CTRL_CKEN_COPRO_BIT, SYS_CTRL_CKEN_CLR_CTRL);

    // Ensure reset and clock operations are complete
    wmb();
}
EXPORT_SYMBOL_GPL(shutdown_copro);

#endif // CONFIG_SUPPORT_LEON
