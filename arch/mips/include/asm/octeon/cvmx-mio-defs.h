/***********************license start***************
 * Author: Cavium Networks
 *
 * Contact: support@caviumnetworks.com
 * This file is part of the OCTEON SDK
 *
 * Copyright (c) 2003-2017 Cavium, Inc.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, Version 2, as
 * published by the Free Software Foundation.
 *
 * This file is distributed in the hope that it will be useful, but
 * AS-IS and WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE, TITLE, or
 * NONINFRINGEMENT.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this file; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 * or visit http://www.gnu.org/licenses/.
 *
 * This file may also be available under a different license from Cavium.
 * Contact Cavium Networks for more information
 ***********************license end**************************************/

#ifndef __CVMX_MIO_DEFS_H__
#define __CVMX_MIO_DEFS_H__

#define CVMX_MIO_BOOT_BIST_STAT (CVMX_ADD_IO_SEG(0x00011800000000F8ull))
#define CVMX_MIO_BOOT_PIN_DEFS (CVMX_ADD_IO_SEG(0x00011800000000C0ull))
#define CVMX_MIO_BOOT_REG_CFGX(offset)					       \
		(CVMX_ADD_IO_SEG(0x0001180000000000ull) + ((offset) & 7) * 8)
#define CVMX_MIO_BOOT_REG_TIMX(offset)					       \
		(CVMX_ADD_IO_SEG(0x0001180000000040ull) + ((offset) & 7) * 8)
#define CVMX_MIO_FUS_DAT2 (CVMX_ADD_IO_SEG(0x0001180000001410ull))
#define CVMX_MIO_FUS_DAT3 (CVMX_ADD_IO_SEG(0x0001180000001418ull))
#define CVMX_MIO_FUS_RCMD (CVMX_ADD_IO_SEG(0x0001180000001500ull))
#define CVMX_MIO_PTP_CLOCK_CFG (CVMX_ADD_IO_SEG(0x0001070000000F00ull))
#define CVMX_MIO_PTP_EVT_CNT (CVMX_ADD_IO_SEG(0x0001070000000F28ull))
#define CVMX_MIO_RST_BOOT (CVMX_ADD_IO_SEG(0x0001180000001600ull))
#define CVMX_MIO_RST_CTLX(offset)					       \
		(CVMX_ADD_IO_SEG(0x0001180000001618ull) + ((offset) & 1) * 8)
#define CVMX_MIO_QLMX_CFG(offset)					       \
		(CVMX_ADD_IO_SEG(0x0001180000001590ull) + ((offset) & 7) * 8)
#define CVMX_MIO_UARTX_LSR(offset)					       \
		(CVMX_ADD_IO_SEG(0x0001180000000828ull) + ((offset) & 1) * 1024)
#define CVMX_MIO_UARTX_THR(offset)					       \
		(CVMX_ADD_IO_SEG(0x0001180000000840ull) + ((offset) & 1) * 1024)
#define CVMX_MIO_FUS_PDF CVMX_MIO_FUS_PDF_FUNC()
static inline uint64_t CVMX_MIO_FUS_PDF_FUNC(void)
{
	switch (cvmx_get_octeon_family()) {
	case OCTEON_CNF71XX & OCTEON_FAMILY_MASK:
	case OCTEON_CN50XX & OCTEON_FAMILY_MASK:
	case OCTEON_CN52XX & OCTEON_FAMILY_MASK:
	case OCTEON_CN56XX & OCTEON_FAMILY_MASK:
	case OCTEON_CN58XX & OCTEON_FAMILY_MASK:
	case OCTEON_CN61XX & OCTEON_FAMILY_MASK:
	case OCTEON_CN63XX & OCTEON_FAMILY_MASK:
	case OCTEON_CN66XX & OCTEON_FAMILY_MASK:
	case OCTEON_CN68XX & OCTEON_FAMILY_MASK:
		return CVMX_ADD_IO_SEG(0x0001180000001420ull);
	case OCTEON_CNF75XX & OCTEON_FAMILY_MASK:
	case OCTEON_CN70XX & OCTEON_FAMILY_MASK:
	case OCTEON_CN73XX & OCTEON_FAMILY_MASK:
	case OCTEON_CN78XX & OCTEON_FAMILY_MASK:
	default:
		return CVMX_ADD_IO_SEG(0x0001180000001428ull);
	}
}


union cvmx_mio_boot_dma_cfgx {
	uint64_t u64;
	struct cvmx_mio_boot_dma_cfgx_s {
		__BITFIELD_FIELD(uint64_t en:1,
		__BITFIELD_FIELD(uint64_t rw:1,
		__BITFIELD_FIELD(uint64_t clr:1,
		__BITFIELD_FIELD(uint64_t reserved_60_60:1,
		__BITFIELD_FIELD(uint64_t swap32:1,
		__BITFIELD_FIELD(uint64_t swap16:1,
		__BITFIELD_FIELD(uint64_t swap8:1,
		__BITFIELD_FIELD(uint64_t endian:1,
		__BITFIELD_FIELD(uint64_t size:20,
		__BITFIELD_FIELD(uint64_t adr:36,
		;))))))))))
	} s;
};

union cvmx_mio_boot_dma_intx {
	uint64_t u64;
	struct cvmx_mio_boot_dma_intx_s {
		__BITFIELD_FIELD(uint64_t reserved_2_63:62,
		__BITFIELD_FIELD(uint64_t dmarq:1,
		__BITFIELD_FIELD(uint64_t done:1,
		;)))
	} s;
};

union cvmx_mio_boot_dma_timx {
	uint64_t u64;
	struct cvmx_mio_boot_dma_timx_s {
		__BITFIELD_FIELD(uint64_t dmack_pi:1,
		__BITFIELD_FIELD(uint64_t dmarq_pi:1,
		__BITFIELD_FIELD(uint64_t tim_mult:2,
		__BITFIELD_FIELD(uint64_t rd_dly:3,
		__BITFIELD_FIELD(uint64_t ddr:1,
		__BITFIELD_FIELD(uint64_t width:1,
		__BITFIELD_FIELD(uint64_t reserved_48_54:7,
		__BITFIELD_FIELD(uint64_t pause:6,
		__BITFIELD_FIELD(uint64_t dmack_h:6,
		__BITFIELD_FIELD(uint64_t we_n:6,
		__BITFIELD_FIELD(uint64_t we_a:6,
		__BITFIELD_FIELD(uint64_t oe_n:6,
		__BITFIELD_FIELD(uint64_t oe_a:6,
		__BITFIELD_FIELD(uint64_t dmack_s:6,
		__BITFIELD_FIELD(uint64_t dmarq:6,
		;)))))))))))))))
	} s;
};

union cvmx_mio_boot_pin_defs {
	uint64_t u64;
	struct cvmx_mio_boot_pin_defs_s {
		__BITFIELD_FIELD(uint64_t reserved_32_63:32,
		__BITFIELD_FIELD(uint64_t user1:16,
		__BITFIELD_FIELD(uint64_t ale:1,
		__BITFIELD_FIELD(uint64_t width:1,
		__BITFIELD_FIELD(uint64_t dmack_p2:1,
		__BITFIELD_FIELD(uint64_t dmack_p1:1,
		__BITFIELD_FIELD(uint64_t dmack_p0:1,
		__BITFIELD_FIELD(uint64_t term:2,
		__BITFIELD_FIELD(uint64_t nand:1,
		__BITFIELD_FIELD(uint64_t user0:8,
		;))))))))))
	} s;
};

union cvmx_mio_boot_reg_cfgx {
	uint64_t u64;
	struct cvmx_mio_boot_reg_cfgx_s {
		__BITFIELD_FIELD(uint64_t reserved_44_63:20,
		__BITFIELD_FIELD(uint64_t dmack:2,
		__BITFIELD_FIELD(uint64_t tim_mult:2,
		__BITFIELD_FIELD(uint64_t rd_dly:3,
		__BITFIELD_FIELD(uint64_t sam:1,
		__BITFIELD_FIELD(uint64_t we_ext:2,
		__BITFIELD_FIELD(uint64_t oe_ext:2,
		__BITFIELD_FIELD(uint64_t en:1,
		__BITFIELD_FIELD(uint64_t orbit:1,
		__BITFIELD_FIELD(uint64_t ale:1,
		__BITFIELD_FIELD(uint64_t width:1,
		__BITFIELD_FIELD(uint64_t size:12,
		__BITFIELD_FIELD(uint64_t base:16,
		;)))))))))))))
	} s;
};

union cvmx_mio_boot_reg_timx {
	uint64_t u64;
	struct cvmx_mio_boot_reg_timx_s {
		__BITFIELD_FIELD(uint64_t pagem:1,
		__BITFIELD_FIELD(uint64_t waitm:1,
		__BITFIELD_FIELD(uint64_t pages:2,
		__BITFIELD_FIELD(uint64_t ale:6,
		__BITFIELD_FIELD(uint64_t page:6,
		__BITFIELD_FIELD(uint64_t wait:6,
		__BITFIELD_FIELD(uint64_t pause:6,
		__BITFIELD_FIELD(uint64_t wr_hld:6,
		__BITFIELD_FIELD(uint64_t rd_hld:6,
		__BITFIELD_FIELD(uint64_t we:6,
		__BITFIELD_FIELD(uint64_t oe:6,
		__BITFIELD_FIELD(uint64_t ce:6,
		__BITFIELD_FIELD(uint64_t adr:6,
		;)))))))))))))
	} s;
};

union cvmx_mio_fus_dat2 {
	uint64_t u64;
	struct cvmx_mio_fus_dat2_s {
		__BITFIELD_FIELD(uint64_t reserved_59_63:5,
		__BITFIELD_FIELD(uint64_t run_platform:3,
		__BITFIELD_FIELD(uint64_t gbl_pwr_throttle:8,
		__BITFIELD_FIELD(uint64_t fus118:1,
		__BITFIELD_FIELD(uint64_t rom_info:10,
		__BITFIELD_FIELD(uint64_t power_limit:2,
		__BITFIELD_FIELD(uint64_t dorm_crypto:1,
		__BITFIELD_FIELD(uint64_t fus318:1,
		__BITFIELD_FIELD(uint64_t raid_en:1,
		__BITFIELD_FIELD(uint64_t reserved_30_31:2,
		__BITFIELD_FIELD(uint64_t nokasu:1,
		__BITFIELD_FIELD(uint64_t nodfa_cp2:1,
		__BITFIELD_FIELD(uint64_t nomul:1,
		__BITFIELD_FIELD(uint64_t nocrypto:1,
		__BITFIELD_FIELD(uint64_t rst_sht:1,
		__BITFIELD_FIELD(uint64_t bist_dis:1,
		__BITFIELD_FIELD(uint64_t chip_id:8,
		__BITFIELD_FIELD(uint64_t reserved_0_15:16,
		;))))))))))))))))))
	} s;
	struct cvmx_mio_fus_dat2_s cn56xx;
	struct cvmx_mio_fus_dat2_s cn58xx;
	struct cvmx_mio_fus_dat2_s cn61xx;
	struct cvmx_mio_fus_dat2_s cn63xx;
	struct cvmx_mio_fus_dat2_s cn66xx;
	struct cvmx_mio_fus_dat2_s cn68xx;
	struct cvmx_mio_fus_dat2_s cn70xx;
	struct cvmx_mio_fus_dat2_s cn73xx;
	struct cvmx_mio_fus_dat2_s cn78xx;
};

union cvmx_mio_fus_dat3 {
	uint64_t u64;
	struct cvmx_mio_fus_dat3_s {
		__BITFIELD_FIELD(uint64_t reserved_59_63:6,
		__BITFIELD_FIELD(uint64_t pll_ctl:10,
		__BITFIELD_FIELD(uint64_t dfa_info_dte:3,
		__BITFIELD_FIELD(uint64_t dfa_info_clm:4,
		__BITFIELD_FIELD(uint64_t pll_alt_matrix:1,
		__BITFIELD_FIELD(uint64_t reserved_39_40:2,
		__BITFIELD_FIELD(uint64_t efus_lck_rsv:1,
		__BITFIELD_FIELD(uint64_t efus_lck_man:1,
		__BITFIELD_FIELD(uint64_t pll_half_dis:1,
		__BITFIELD_FIELD(uint64_t l2c_crip:3,
		__BITFIELD_FIELD(uint64_t use_int_refclk:1,
		__BITFIELD_FIELD(uint64_t zip_info:2,
		__BITFIELD_FIELD(uint64_t bar2_en:1,
		__BITFIELD_FIELD(uint64_t efus_lck:1,
		__BITFIELD_FIELD(uint64_t efus_ign:1,
		__BITFIELD_FIELD(uint64_t nozip:1,
		__BITFIELD_FIELD(uint64_t nodfa_dte:1,
		__BITFIELD_FIELD(uint64_t icache:24,
		;))))))))))))))))))
	} s;
	struct cvmx_mio_fus_dat3_s cn56xx;
	struct cvmx_mio_fus_dat3_s cn61xx;
	struct cvmx_mio_fus_dat3_s cn63xx;
	struct cvmx_mio_fus_dat3_s cn66xx;
	struct cvmx_mio_fus_dat3_s cn68xx;
	struct cvmx_mio_fus_dat3_s cn70xx;
	struct cvmx_mio_fus_dat3_cn73xx {
		__BITFIELD_FIELD(uint64_t reserved_59_63:6,
		__BITFIELD_FIELD(uint64_t pll_ctl:10,
		__BITFIELD_FIELD(uint64_t dfa_info_dte:3,
		__BITFIELD_FIELD(uint64_t dfa_info_clm:4,
		__BITFIELD_FIELD(uint64_t pll_alt_matrix:1,
		__BITFIELD_FIELD(uint64_t reserved_39_40:2,
		__BITFIELD_FIELD(uint64_t efus_lck_rsv:1,
		__BITFIELD_FIELD(uint64_t efus_lck_man:1,
		__BITFIELD_FIELD(uint64_t pll_half_dis:1,
		__BITFIELD_FIELD(uint64_t l2c_crip:3,
		__BITFIELD_FIELD(uint64_t use_int_refclk:1,
		__BITFIELD_FIELD(uint64_t zip_info:2,
		__BITFIELD_FIELD(uint64_t bar2_en:1,
		__BITFIELD_FIELD(uint64_t efus_lck:1,
		__BITFIELD_FIELD(uint64_t efus_ign:1,
		__BITFIELD_FIELD(uint64_t nozip:1,
		__BITFIELD_FIELD(uint64_t nodfa_dte:1,
		__BITFIELD_FIELD(uint64_t reserved_19_23:6,
		__BITFIELD_FIELD(uint64_t nohna_dte:1,
		__BITFIELD_FIELD(uint64_t hna_info_dte:3,
		__BITFIELD_FIELD(uint64_t hna_info_clm:4,
		__BITFIELD_FIELD(uint64_t reserved_9_9:1,
		__BITFIELD_FIELD(uint64_t core_pll_mul:5,
		__BITFIELD_FIELD(uint64_t pnr_pll_mul:4,
		;))))))))))))))))))))))))
	} cn73xx;
	struct cvmx_mio_fus_dat3_cn73xx cn78xx;
	struct cvmx_mio_fus_dat3_s cnf71xx;
};

union cvmx_mio_fus_rcmd {
	uint64_t u64;
	struct cvmx_mio_fus_rcmd_s {
		__BITFIELD_FIELD(uint64_t reserved_24_63:40,
		__BITFIELD_FIELD(uint64_t dat:8,
		__BITFIELD_FIELD(uint64_t reserved_13_15:3,
		__BITFIELD_FIELD(uint64_t pend:1,
		__BITFIELD_FIELD(uint64_t reserved_9_11:3,
		__BITFIELD_FIELD(uint64_t efuse:1,
		__BITFIELD_FIELD(uint64_t addr:8,
		;)))))))
	} s;
};

union cvmx_mio_ptp_clock_cfg {
	uint64_t u64;
	struct cvmx_mio_ptp_clock_cfg_s {
		__BITFIELD_FIELD(uint64_t reserved_42_63:22,
		__BITFIELD_FIELD(uint64_t pps:1,
		__BITFIELD_FIELD(uint64_t ckout:1,
		__BITFIELD_FIELD(uint64_t ext_clk_edge:2,
		__BITFIELD_FIELD(uint64_t ckout_out4:1,
		__BITFIELD_FIELD(uint64_t pps_out:5,
		__BITFIELD_FIELD(uint64_t pps_inv:1,
		__BITFIELD_FIELD(uint64_t pps_en:1,
		__BITFIELD_FIELD(uint64_t ckout_out:4,
		__BITFIELD_FIELD(uint64_t ckout_inv:1,
		__BITFIELD_FIELD(uint64_t ckout_en:1,
		__BITFIELD_FIELD(uint64_t evcnt_in:6,
		__BITFIELD_FIELD(uint64_t evcnt_edge:1,
		__BITFIELD_FIELD(uint64_t evcnt_en:1,
		__BITFIELD_FIELD(uint64_t tstmp_in:6,
		__BITFIELD_FIELD(uint64_t tstmp_edge:1,
		__BITFIELD_FIELD(uint64_t tstmp_en:1,
		__BITFIELD_FIELD(uint64_t ext_clk_in:6,
		__BITFIELD_FIELD(uint64_t ext_clk_en:1,
		__BITFIELD_FIELD(uint64_t ptp_en:1,
		;))))))))))))))))))))
	} s;
};

union cvmx_mio_qlmx_cfg {
	uint64_t u64;
	struct cvmx_mio_qlmx_cfg_s {
		__BITFIELD_FIELD(uint64_t reserved_15_63:49,
		__BITFIELD_FIELD(uint64_t prtmode:1,
		__BITFIELD_FIELD(uint64_t reserved_12_13:2,
		__BITFIELD_FIELD(uint64_t qlm_spd:4,
		__BITFIELD_FIELD(uint64_t reserved_4_7:4,
		__BITFIELD_FIELD(uint64_t qlm_cfg:4,
		;))))))
	} s;
};

union cvmx_mio_rst_boot {
	uint64_t u64;
	struct cvmx_mio_rst_boot_s {
		__BITFIELD_FIELD(uint64_t chipkill:1,
		__BITFIELD_FIELD(uint64_t jtcsrdis:1,
		__BITFIELD_FIELD(uint64_t ejtagdis:1,
		__BITFIELD_FIELD(uint64_t romen:1,
		__BITFIELD_FIELD(uint64_t ckill_ppdis:1,
		__BITFIELD_FIELD(uint64_t jt_tstmode:1,
		__BITFIELD_FIELD(uint64_t reserved_50_57:8,
		__BITFIELD_FIELD(uint64_t lboot_ext:2,
		__BITFIELD_FIELD(uint64_t reserved_44_47:4,
		__BITFIELD_FIELD(uint64_t qlm4_spd:4,
		__BITFIELD_FIELD(uint64_t qlm3_spd:4,
		__BITFIELD_FIELD(uint64_t c_mul:6,
		__BITFIELD_FIELD(uint64_t pnr_mul:6,
		__BITFIELD_FIELD(uint64_t qlm2_spd:4,
		__BITFIELD_FIELD(uint64_t qlm1_spd:4,
		__BITFIELD_FIELD(uint64_t qlm0_spd:4,
		__BITFIELD_FIELD(uint64_t lboot:10,
		__BITFIELD_FIELD(uint64_t rboot:1,
		__BITFIELD_FIELD(uint64_t rboot_pin:1,
		;)))))))))))))))))))
	} s;
};

union cvmx_mio_rst_ctlx {
	uint64_t u64;
	struct cvmx_mio_rst_ctlx_s {
		__BITFIELD_FIELD(uint64_t reserved_13_63:51,
		__BITFIELD_FIELD(uint64_t in_rev_ln:1,
		__BITFIELD_FIELD(uint64_t rev_lanes:1,
		__BITFIELD_FIELD(uint64_t gen1_only:1,
		__BITFIELD_FIELD(uint64_t prst_link:1,
		__BITFIELD_FIELD(uint64_t rst_done:1,
		__BITFIELD_FIELD(uint64_t rst_link:1,
		__BITFIELD_FIELD(uint64_t host_mode:1,
		__BITFIELD_FIELD(uint64_t prtmode:2,
		__BITFIELD_FIELD(uint64_t rst_drv:1,
		__BITFIELD_FIELD(uint64_t rst_rcv:1,
		__BITFIELD_FIELD(uint64_t rst_chip:1,
		__BITFIELD_FIELD(uint64_t rst_val:1,
		;)))))))))))))
	} s;
};

#endif
