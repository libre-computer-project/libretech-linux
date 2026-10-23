// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018 Maxime Jourdan <mjourdan@baylibre.com>
 * Copyright (C) 2015 Amlogic, Inc. All rights reserved.
 */

#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "dos_regs.h"
#include "hevc_regs.h"
#include "codec_vp9.h"
#include "esparser.h"
#include "vdec_helpers.h"
#include "vdec_debugfs.h"
#include "vdec_hevc.h"
#include "codec_hevc_common.h"

/* HEVC reg mapping */
#define VP9_DEC_STATUS_REG	HEVC_ASSIST_SCRATCH_0
	#define VP9_10B_DECODE_SLICE	5
	#define VP9_10B_DISCARD_NAL	6
	#define VP9_HEAD_PARSER_DONE	0xf0
	#define VP9_EOS			0xf2
#define VP9_RPM_BUFFER		HEVC_ASSIST_SCRATCH_1
#define VP9_SHORT_TERM_RPS	HEVC_ASSIST_SCRATCH_2
#define VP9_ADAPT_PROB_REG	HEVC_ASSIST_SCRATCH_3
#define VP9_MMU_MAP_BUFFER	HEVC_ASSIST_SCRATCH_4
#define VP9_PPS_BUFFER		HEVC_ASSIST_SCRATCH_5
#define VP9_SAO_UP		HEVC_ASSIST_SCRATCH_6
#define VP9_STREAM_SWAP_BUFFER	HEVC_ASSIST_SCRATCH_7
#define VP9_STREAM_SWAP_BUFFER2 HEVC_ASSIST_SCRATCH_8
#define VP9_PROB_SWAP_BUFFER	HEVC_ASSIST_SCRATCH_9
#define VP9_COUNT_SWAP_BUFFER	HEVC_ASSIST_SCRATCH_A
#define VP9_SEG_MAP_BUFFER	HEVC_ASSIST_SCRATCH_B
#define VP9_SCALELUT		HEVC_ASSIST_SCRATCH_D
#define VP9_WAIT_FLAG		HEVC_ASSIST_SCRATCH_E
#define LMEM_DUMP_ADR		HEVC_ASSIST_SCRATCH_F
#define NAL_SEARCH_CTL		HEVC_ASSIST_SCRATCH_I
#define VP9_DECODE_MODE		HEVC_ASSIST_SCRATCH_J
	#define DECODE_MODE_SINGLE 0
#define DECODE_STOP_POS		HEVC_ASSIST_SCRATCH_K
#define HEVC_DECODE_COUNT	HEVC_ASSIST_SCRATCH_M
#define HEVC_DECODE_SIZE	HEVC_ASSIST_SCRATCH_N

/* VP9 Constants */
#define LCU_SIZE		64
#define MAX_REF_PIC_NUM		24
#define REFS_PER_FRAME		3
#define REF_FRAMES		8
#define MV_MEM_UNIT		0x240
#define ADAPT_PROB_SIZE		0xf80

/*
 * Deblock filter threshold register packing (HEVC_DBLK_CFG9):
 * Each 32-bit write packs thresholds for two filter levels (even, odd).
 * Bits [15:8] = lim (6 bits, mask 0x3f), bits [7:0] = mblim (8 bits).
 * Upper halfword = odd level, lower halfword = even level.
 */
#define DBLK_LIM_MASK		0x3f
#define DBLK_MBLIM_MASK		0xff
#define DBLK_LIM_SHIFT		8
#define DBLK_THR_PAIR_SHIFT	16

/*
 * Loop filter level register packing (HEVC_DBLK_CFGA):
 * Each 32-bit write packs levels for one segment, all 4 ref frames.
 * Bits [31:24] = ALTREF, [23:16] = GOLDEN, [15:8] = LAST, [7:0] = INTRA.
 * Each field is 6 bits (mask 0x3f).
 */
#define DBLK_LVL_MASK		0x3f

/*
 * HEVC_DBLK_CFGB bitfield definitions (BSP: vvp9.c loop filter init).
 * The register layout differs across SoC generations.
 */
/* BIT(0): VP9 video format select (1=VP9, 0=HEVC) -- all platforms */
#define DBLK_CFGB_VP9_MODE		BIT(0)
/* BIT(4): dblk pipeline mode (1=pipelined for >=1280 width) -- G12A+ */
#define DBLK_CFGB_PIPELINE_MODE		BIT(4)
/* BIT(8): compressed write enable (FBC/MMU first write) -- G12A+ */
#define DBLK_CFGB_COMPRESSED_WRITE	BIT(8)
/* BIT(9): uncompressed write enable (NV12 second write) -- G12A+ */
#define DBLK_CFGB_UNCOMP_WRITE		BIT(9)
/* Bits [15:10]: DW FIFO thresholds -- SM1+ only */
#define DBLK_CFGB_SM1_DW_FIFO_THRES	((0x3 << 14) | (0x3 << 12) | (0x3 << 10))
/*
 * G12A DBLK_CFGB: 0x54 << 8 sets deblock strength parameters.
 * BSP vvp9.c: 0x54 = recommended deblock configuration for G12A.
 */
#define DBLK_CFGB_G12A_STRENGTH		(0x54 << 8)
/*
 * GXL DBLK_CFGB: 0x40400001 -- combined deblock config + VP9 mode.
 * BSP vvp9.c: GXL uses a single write with embedded FIFO thresholds
 * and VP9 mode bit.  Bits [23:16]=0x40 and [31:24]=0x40 are GXL-specific
 * deblock FIFO thresholds (different layout than SM1 DW FIFO fields).
 */
#define DBLK_CFGB_GXL			0x40400001

/*
 * HEVC_DBLK_CFG3: deblock left column storage offset (SM1+ only).
 * BSP vvp9.c: 0x404010 configures internal left-column storage offsets
 * for the deblocking filter.  Not needed on G12A or GXL which use
 * different internal memory layouts.
 */
#define DBLK_CFG3_SM1_LEFT_STORAGE	0x404010

/*
 * HEVC_MPRED_CTRL3: motion prediction control (BSP: vvp9.c).
 * 0x24122412 configures MPRED above-line MV storage layout:
 *   Each 16-bit half: bits[15:12]=above_mode, bits[11:8]=above_ref,
 *   bits[7:4]=cur_mode, bits[3:0]=cur_ref.
 * Value 0x2412 means: above_mode=2, above_ref=4, cur_mode=1, cur_ref=2.
 * Repeated for two halves (upper/lower 16 bits) for dual-pipe.
 */
#define MPRED_CTRL3_VP9_DEFAULT		0x24122412

/*
 * Motion compensation reference cache controller (MCRCC) values.
 * HEVCD_MCRCC_CTL1:
 *   0x2 = reset MCRCC (flush cached references)
 *   0x0 = disable MCRCC (for I-frames, no inter references)
 *   0xff0 = enable progressive mode (cache LAST + GOLDEN ref frames)
 */
#define MCRCC_CTL1_RESET		0x2
#define MCRCC_CTL1_DISABLE		0x0
#define MCRCC_CTL1_ENABLE_PROGRESSIVE	0xff0

/*
 * HEVC_SAO_CTRL1 bitfield definitions.
 * Bits [15:0] = endianness/byte-swap configuration.
 * 0xff0 = little-endian per 2-byte swap (NV12 direct DMA pixel order).
 * Bits [15:14] = AXI line alignment: 2 = 64 bytes.
 * BIT(0) = disable compressed mode (cm).
 */
#define SAO_CTRL1_ENDIAN_MASK		0xffff
#define SAO_CTRL1_BYTE_SWAP_NV12	0xff0
#define SAO_CTRL1_LINE_ALIGN_64		(2 << 14)
#define SAO_CTRL1_DISABLE_CM		BIT(0)

/*
 * HEVC_SAO_CTRL5 bitmask definitions.
 * BIT(9): mem_saving_mode -- 1=8-bit FBC (3200 bytes/block), 0=10-bit (4096).
 * BIT(10): use HEVC_CM_HEADER_START_ADDR for MMU header pointer.
 * Bits [23:16] (mask 0xff0000): downscale control -- clear for no downscale.
 * Mask 0xff0200: clear bits [23:16] and bit 9 together for 10-bit FBC setup.
 */
#define SAO_CTRL5_MEM_SAVING_MODE	BIT(9)
#define SAO_CTRL5_USE_CM_HEADER		BIT(10)
#define SAO_CTRL5_DOWNSCALE_MASK	0xff0000
#define SAO_CTRL5_10BIT_FBC_CLEAR_MASK	0xff0200

/*
 * HEVCD_IPP_AXIIF_CONFIG bitfield definitions.
 * Bits [3:0] = endianness/byte-swap (0xf = LE per 2-byte swap, matches SAO).
 * Bits [9:8] = line alignment: 2 = 64 bytes.
 * BIT(12) = UV swap: 1=NV21 (VU order), 0=NV12 (UV order).
 * Mask 0x30f = clear endian + alignment fields for reconfiguration.
 */
#define IPP_AXIIF_ENDIAN_NV12		0xf
#define IPP_AXIIF_LINE_ALIGN_64		(2 << 8)
#define IPP_AXIIF_CLEAR_MASK		0x30f
#define IPP_AXIIF_UV_SWAP		BIT(12)

/*
 * HEVC_PARSER_INT_CONTROL bitfield definitions.
 * BIT(31): clear -- keep bit 31 low (mask 0x7fffffff).
 * Bits [30:29] = 3: parser interrupt mode.
 * BIT(24): enable FIFO overflow interrupt.
 * BIT(22): enable startcode-found interrupt.
 * BIT(7): enable parser done interrupt.
 * BIT(4): enable stream-end interrupt.
 * BIT(0): enable parser interrupt.
 */
#define PARSER_INT_CTL_INIT_MASK	0x7fffffff
#define PARSER_INT_CTL_DEFAULT		((3 << 29) | BIT(24) | BIT(22) | \
					 BIT(7) | BIT(4) | BIT(0))

/*
 * HEVC_SHIFT_CONTROL bitfield definitions.
 * BIT(10): enable emulation prevention.
 * BIT(9): enable startcode search.
 * Bits [7:6] = 3: shift mode = HEVC/VP9.
 * BIT(5): enable auto-flush.
 * BIT(2): shift enable.
 * BIT(1): shifter reset.
 * BIT(0): shifter start.
 */
#define SHIFT_CTRL_VP9_DEFAULT		(BIT(10) | BIT(9) | (3 << 6) | \
					 BIT(5) | BIT(2) | BIT(1) | BIT(0))

/*
 * HEVC_PARSER_IF_CONTROL bitfield definitions.
 * BIT(5): parser IF control enable.
 * BIT(2): auto-command mode.
 * BIT(0): enable parser IF.
 */
#define PARSER_IF_CTL_DEFAULT		(BIT(5) | BIT(2) | BIT(0))

/*
 * NAL_SEARCH_CTL: 0x8 = let firmware do all bitstream parsing.
 * VP9 firmware handles NAL unit search internally.
 */
#define NAL_SEARCH_CTL_VP9		0x8

/*
 * Canvas access configuration for HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR.
 * BIT(1): auto-increment read mode.
 * (16 << 8) | 1: canvas index offset 16 with auto-increment for second pass.
 */
#define CANVAS_ACCCONFIG_READ_AUTO	BIT(1)
#define CANVAS_ACCCONFIG_OFFSET16	((16 << 8) | 1)

/*
 * VP9 probability adaptation constants (ISR).
 * prob_status[7:0] == 0xfd: firmware requests probability adaptation.
 * prob_status[15:8]: frame context index (0-3), selects 0x1000-byte prob set.
 * 0x4000: offset to current (output) probability table within PROB workspace.
 */
#define VP9_REQ_ADAPT_PROB		0xfd
#define VP9_PROB_SET_SIZE		0x1000
#define VP9_CUR_PROB_OFFSET		0x4000

/*
 * Segmentation loop filter info bitmasks (from rpm_param.p.seg_lf_info[]).
 * Firmware packs segment LF data as:
 *   BIT(15): feature enabled flag (1=segment has ALT_LF).
 *   BIT(8): sign bit (1=negative delta).
 *   Bits [5:0]: absolute LF delta value (6-bit, mask 0x3f).
 */
#define SEG_LF_INFO_ENABLED		0x8000
#define SEG_LF_INFO_SIGN		0x100
#define SEG_LF_INFO_DATA_MASK		0x3f

enum FRAME_TYPE {
	KEY_FRAME = 0,
	INTER_FRAME = 1,
	FRAME_TYPES,
};

/* VP9 Workspace layout */
/*
 * MPRED motion-vector buffer: one per reference frame, indexed by
 * frame->index (0..MAX_REF_PIC_NUM-1).  BSP sizes this dynamically
 * via cal_mv_buf_size(pic_width, pic_height) which depends on the LCU
 * grid (lcu_total * 36 * 16 + lcu_w * 16 * lcu_h_factor).
 *
 * We compute the per-slot size at start() from the NEGOTIATED coded
 * resolution (sess->width/height), clamped to the platform max, and grow
 * it in resume() if the bitstream turns out larger.  MAX_REF_PIC_NUM (24)
 * slots are always laid out because frame->index indexes this array.
 * The whole workspace is one contiguous DMA allocation, so its size is
 * a direct contiguous-CMA demand:
 * (total = MPRED_MV_OFFSET 0x19D800 + slot array + RPM/LMEM 0x900, 64K-aligned)
 *   320x240:   0x10000/slot ->   1.5 MiB array, total 0x320000  (3.1 MiB)
 *   1280x720:  0x30000/slot ->   4.5 MiB array, total 0x620000  (6.1 MiB)
 *   4096x2304: 0x150000/slot -> 31.5 MiB array, total 0x2120000 (33.1 MiB)
 *   8192x4320: 0x4D0000/slot -> 115.5 MiB array, total 0x7520000 (117.1 MiB)
 *
 * Sizing against the platform max unconditionally made every session
 * demand the 8K figure, which no 128 MB CMA board can satisfy.
 *
 * The old fixed 0x120000 caused overflow at resolutions above ~3584x2048,
 * corrupting the RPM/LMEM workspace regions (TODO-VP9-011b).
 */

#define IPP_SIZE	0x4000
#define SAO_ABV_SIZE	0x30000
#define SAO_VB_SIZE	0x30000
#define SH_TM_RPS_SIZE	0x800
#define VPS_SIZE	0x800
#define SPS_SIZE	0x800
#define PPS_SIZE	0x2000
#define SAO_UP_SIZE	0x2800
#define SWAP_BUF_SIZE	0x800
#define SWAP_BUF2_SIZE	0x800
#define SCALELUT_SIZE	0x8000
#define DBLK_PARA_SIZE	0x80000
#define DBLK_DATA_SIZE	0x80000
#define SEG_MAP_SIZE	0xd800
#define PROB_SIZE	0x5000
#define COUNT_SIZE	0x3000
#define MMU_VBH_SIZE	0x5000
#define MPRED_ABV_SIZE	0x10000
#define RPM_BUF_SIZE	0x100
#define LMEM_SIZE	0x800

/* Fixed offsets: everything before MPRED_MV is resolution-independent */
#define IPP_OFFSET       0x00
#define SAO_ABV_OFFSET   (IPP_OFFSET + IPP_SIZE)
#define SAO_VB_OFFSET    (SAO_ABV_OFFSET + SAO_ABV_SIZE)
#define SH_TM_RPS_OFFSET (SAO_VB_OFFSET + SAO_VB_SIZE)
#define VPS_OFFSET       (SH_TM_RPS_OFFSET + SH_TM_RPS_SIZE)
#define SPS_OFFSET       (VPS_OFFSET + VPS_SIZE)
#define PPS_OFFSET       (SPS_OFFSET + SPS_SIZE)
#define SAO_UP_OFFSET    (PPS_OFFSET + PPS_SIZE)
#define SWAP_BUF_OFFSET  (SAO_UP_OFFSET + SAO_UP_SIZE)
#define SWAP_BUF2_OFFSET (SWAP_BUF_OFFSET + SWAP_BUF_SIZE)
#define SCALELUT_OFFSET  (SWAP_BUF2_OFFSET + SWAP_BUF2_SIZE)
#define DBLK_PARA_OFFSET (SCALELUT_OFFSET + SCALELUT_SIZE)
#define DBLK_DATA_OFFSET (DBLK_PARA_OFFSET + DBLK_PARA_SIZE)
#define SEG_MAP_OFFSET   (DBLK_DATA_OFFSET + DBLK_DATA_SIZE)
#define PROB_OFFSET      (SEG_MAP_OFFSET + SEG_MAP_SIZE)
#define COUNT_OFFSET     (PROB_OFFSET + PROB_SIZE)
#define MMU_VBH_OFFSET   (COUNT_OFFSET + COUNT_SIZE)
#define MPRED_ABV_OFFSET (MMU_VBH_OFFSET + MMU_VBH_SIZE)
#define MPRED_MV_OFFSET  (MPRED_ABV_OFFSET + MPRED_ABV_SIZE)

/* Dynamic offsets: depend on mpred_mv_buf_size which varies per platform */
/* These are computed at init time and stored in codec_vp9 */

/*
 * BSP cal_mv_buf_size: compute MPRED MV buffer size for a given max resolution.
 * Each 64x64 superblock contributes 36*16 bytes to size_a, plus a per-row
 * component (lcu_w * 16 * lcu_h_factor) in size_b.  Result is 64K-aligned.
 */
static u32 cal_mv_buf_size(u32 pic_width, u32 pic_height)
{
	u32 pic_width_64  = ALIGN(pic_width, 64);
	u32 pic_height_32 = ALIGN(pic_height, 32);
	u32 pic_width_lcu  = DIV_ROUND_UP(pic_width_64, 64);
	u32 pic_height_lcu = DIV_ROUND_UP(pic_height_32, 64);
	u32 lcu_total = pic_width_lcu * pic_height_lcu;
	u32 size_a = lcu_total * 36 * 16;
	u32 size_b = pic_width_lcu * 16 *
		     ((pic_height_lcu >> 3) + (pic_height_lcu & 0x7));

	return ALIGN(size_a + size_b, SZ_64K);
}

#define NONE           -1
#define INTRA_FRAME     0
#define LAST_FRAME      1
#define GOLDEN_FRAME    2
#define ALTREF_FRAME    3
#define MAX_REF_FRAMES  4

/*
 * Defines, declarations, sub-functions for vp9 de-block loop
	filter Thr/Lvl table update
 * - struct segmentation is for loop filter only (removed something)
 * - function "vp9_loop_filter_init" and "vp9_loop_filter_frame_init" will
	be instantiated in C_Entry
 * - vp9_loop_filter_init run once before decoding start
 * - vp9_loop_filter_frame_init run before every frame decoding start
 * - set video format to VP9 is in vp9_loop_filter_init
 */
#define MAX_LOOP_FILTER		63
#define MAX_REF_LF_DELTAS	4
#define MAX_MODE_LF_DELTAS	2
#define SEGMENT_DELTADATA	0
#define SEGMENT_ABSDATA		1
#define MAX_SEGMENTS		8

/* VP9 PROB processing defines */
#define VP9_PARTITION_START      0
#define VP9_PARTITION_SIZE_STEP  (3 * 4)
#define VP9_PARTITION_ONE_SIZE   (4 * VP9_PARTITION_SIZE_STEP)
#define VP9_PARTITION_KEY_START  0
#define VP9_PARTITION_P_START    VP9_PARTITION_ONE_SIZE
#define VP9_PARTITION_SIZE       (2 * VP9_PARTITION_ONE_SIZE)
#define VP9_SKIP_START           (VP9_PARTITION_START + VP9_PARTITION_SIZE)
#define VP9_SKIP_SIZE            4 /* only use 3*/
#define VP9_TX_MODE_START        (VP9_SKIP_START + VP9_SKIP_SIZE)
#define VP9_TX_MODE_8_0_OFFSET   0
#define VP9_TX_MODE_8_1_OFFSET   1
#define VP9_TX_MODE_16_0_OFFSET  2
#define VP9_TX_MODE_16_1_OFFSET  4
#define VP9_TX_MODE_32_0_OFFSET  6
#define VP9_TX_MODE_32_1_OFFSET  9
#define VP9_TX_MODE_SIZE         12
#define VP9_COEF_START           (VP9_TX_MODE_START + VP9_TX_MODE_SIZE)
#define VP9_COEF_BAND_0_OFFSET   0
#define VP9_COEF_BAND_1_OFFSET   (VP9_COEF_BAND_0_OFFSET + 3 * 3 + 1)
#define VP9_COEF_BAND_2_OFFSET   (VP9_COEF_BAND_1_OFFSET + 6 * 3)
#define VP9_COEF_BAND_3_OFFSET   (VP9_COEF_BAND_2_OFFSET + 6 * 3)
#define VP9_COEF_BAND_4_OFFSET   (VP9_COEF_BAND_3_OFFSET + 6 * 3)
#define VP9_COEF_BAND_5_OFFSET   (VP9_COEF_BAND_4_OFFSET + 6 * 3)
#define VP9_COEF_SIZE_ONE_SET    100 /* ((3 + 5 * 6) * 3 + 1 padding)*/
#define VP9_COEF_4X4_START       (VP9_COEF_START + 0 * VP9_COEF_SIZE_ONE_SET)
#define VP9_COEF_8X8_START       (VP9_COEF_START + 4 * VP9_COEF_SIZE_ONE_SET)
#define VP9_COEF_16X16_START     (VP9_COEF_START + 8 * VP9_COEF_SIZE_ONE_SET)
#define VP9_COEF_32X32_START     (VP9_COEF_START + 12 * VP9_COEF_SIZE_ONE_SET)
#define VP9_COEF_SIZE_PLANE      (2 * VP9_COEF_SIZE_ONE_SET)
#define VP9_COEF_SIZE            (4 * 2 * 2 * VP9_COEF_SIZE_ONE_SET)
#define VP9_INTER_MODE_START     (VP9_COEF_START + VP9_COEF_SIZE)
#define VP9_INTER_MODE_SIZE      24 /* only use 21 (# * 7)*/
#define VP9_INTERP_START         (VP9_INTER_MODE_START + VP9_INTER_MODE_SIZE)
#define VP9_INTERP_SIZE          8
#define VP9_INTRA_INTER_START    (VP9_INTERP_START + VP9_INTERP_SIZE)
#define VP9_INTRA_INTER_SIZE     4
#define VP9_INTERP_INTRA_INTER_START  VP9_INTERP_START
#define VP9_INTERP_INTRA_INTER_SIZE   (VP9_INTERP_SIZE + VP9_INTRA_INTER_SIZE)
#define VP9_COMP_INTER_START     \
		(VP9_INTERP_INTRA_INTER_START + VP9_INTERP_INTRA_INTER_SIZE)
#define VP9_COMP_INTER_SIZE      5
#define VP9_COMP_REF_START       (VP9_COMP_INTER_START + VP9_COMP_INTER_SIZE)
#define VP9_COMP_REF_SIZE        5
#define VP9_SINGLE_REF_START     (VP9_COMP_REF_START + VP9_COMP_REF_SIZE)
#define VP9_SINGLE_REF_SIZE      10
#define VP9_REF_MODE_START       VP9_COMP_INTER_START
#define VP9_REF_MODE_SIZE        \
		(VP9_COMP_INTER_SIZE + VP9_COMP_REF_SIZE + VP9_SINGLE_REF_SIZE)
#define VP9_IF_Y_MODE_START      (VP9_REF_MODE_START + VP9_REF_MODE_SIZE)
#define VP9_IF_Y_MODE_SIZE       36
#define VP9_IF_UV_MODE_START     (VP9_IF_Y_MODE_START + VP9_IF_Y_MODE_SIZE)
#define VP9_IF_UV_MODE_SIZE      92 /* only use 90*/
#define VP9_MV_JOINTS_START      (VP9_IF_UV_MODE_START + VP9_IF_UV_MODE_SIZE)
#define VP9_MV_JOINTS_SIZE       3
#define VP9_MV_SIGN_0_START      (VP9_MV_JOINTS_START + VP9_MV_JOINTS_SIZE)
#define VP9_MV_SIGN_0_SIZE       1
#define VP9_MV_CLASSES_0_START   (VP9_MV_SIGN_0_START + VP9_MV_SIGN_0_SIZE)
#define VP9_MV_CLASSES_0_SIZE    10
#define VP9_MV_CLASS0_0_START    \
		(VP9_MV_CLASSES_0_START + VP9_MV_CLASSES_0_SIZE)
#define VP9_MV_CLASS0_0_SIZE     1
#define VP9_MV_BITS_0_START      (VP9_MV_CLASS0_0_START + VP9_MV_CLASS0_0_SIZE)
#define VP9_MV_BITS_0_SIZE       10
#define VP9_MV_SIGN_1_START      (VP9_MV_BITS_0_START + VP9_MV_BITS_0_SIZE)
#define VP9_MV_SIGN_1_SIZE       1
#define VP9_MV_CLASSES_1_START   \
			(VP9_MV_SIGN_1_START + VP9_MV_SIGN_1_SIZE)
#define VP9_MV_CLASSES_1_SIZE    10
#define VP9_MV_CLASS0_1_START    \
			(VP9_MV_CLASSES_1_START + VP9_MV_CLASSES_1_SIZE)
#define VP9_MV_CLASS0_1_SIZE     1
#define VP9_MV_BITS_1_START      \
			(VP9_MV_CLASS0_1_START + VP9_MV_CLASS0_1_SIZE)
#define VP9_MV_BITS_1_SIZE       10
#define VP9_MV_CLASS0_FP_0_START \
			(VP9_MV_BITS_1_START + VP9_MV_BITS_1_SIZE)
#define VP9_MV_CLASS0_FP_0_SIZE  9
#define VP9_MV_CLASS0_FP_1_START \
			(VP9_MV_CLASS0_FP_0_START + VP9_MV_CLASS0_FP_0_SIZE)
#define VP9_MV_CLASS0_FP_1_SIZE  9
#define VP9_MV_CLASS0_HP_0_START \
			(VP9_MV_CLASS0_FP_1_START + VP9_MV_CLASS0_FP_1_SIZE)
#define VP9_MV_CLASS0_HP_0_SIZE  2
#define VP9_MV_CLASS0_HP_1_START \
			(VP9_MV_CLASS0_HP_0_START + VP9_MV_CLASS0_HP_0_SIZE)
#define VP9_MV_CLASS0_HP_1_SIZE  2
#define VP9_MV_START             VP9_MV_JOINTS_START
#define VP9_MV_SIZE              72 /*only use 69*/

#define VP9_TOTAL_SIZE           (VP9_MV_START + VP9_MV_SIZE)

/* VP9 COUNT mem processing defines */
#define VP9_COEF_COUNT_START           0
#define VP9_COEF_COUNT_BAND_0_OFFSET   0
#define VP9_COEF_COUNT_BAND_1_OFFSET   \
			(VP9_COEF_COUNT_BAND_0_OFFSET + 3 * 5)
#define VP9_COEF_COUNT_BAND_2_OFFSET   \
			(VP9_COEF_COUNT_BAND_1_OFFSET + 6 * 5)
#define VP9_COEF_COUNT_BAND_3_OFFSET   \
			(VP9_COEF_COUNT_BAND_2_OFFSET + 6 * 5)
#define VP9_COEF_COUNT_BAND_4_OFFSET   \
			(VP9_COEF_COUNT_BAND_3_OFFSET + 6 * 5)
#define VP9_COEF_COUNT_BAND_5_OFFSET   \
			(VP9_COEF_COUNT_BAND_4_OFFSET + 6 * 5)
#define VP9_COEF_COUNT_SIZE_ONE_SET    165 /* ((3 + 5 * 6) * 5 */
#define VP9_COEF_COUNT_4X4_START       \
		(VP9_COEF_COUNT_START + 0 * VP9_COEF_COUNT_SIZE_ONE_SET)
#define VP9_COEF_COUNT_8X8_START       \
		(VP9_COEF_COUNT_START + 4 * VP9_COEF_COUNT_SIZE_ONE_SET)
#define VP9_COEF_COUNT_16X16_START     \
		(VP9_COEF_COUNT_START + 8 * VP9_COEF_COUNT_SIZE_ONE_SET)
#define VP9_COEF_COUNT_32X32_START     \
		(VP9_COEF_COUNT_START + 12 * VP9_COEF_COUNT_SIZE_ONE_SET)
#define VP9_COEF_COUNT_SIZE_PLANE      (2 * VP9_COEF_COUNT_SIZE_ONE_SET)
#define VP9_COEF_COUNT_SIZE            (4 * 2 * 2 * VP9_COEF_COUNT_SIZE_ONE_SET)

#define VP9_INTRA_INTER_COUNT_START    \
		(VP9_COEF_COUNT_START + VP9_COEF_COUNT_SIZE)
#define VP9_INTRA_INTER_COUNT_SIZE     (4 * 2)
#define VP9_COMP_INTER_COUNT_START     \
		(VP9_INTRA_INTER_COUNT_START + VP9_INTRA_INTER_COUNT_SIZE)
#define VP9_COMP_INTER_COUNT_SIZE      (5 * 2)
#define VP9_COMP_REF_COUNT_START       \
		(VP9_COMP_INTER_COUNT_START + VP9_COMP_INTER_COUNT_SIZE)
#define VP9_COMP_REF_COUNT_SIZE        (5 * 2)
#define VP9_SINGLE_REF_COUNT_START     \
		(VP9_COMP_REF_COUNT_START + VP9_COMP_REF_COUNT_SIZE)
#define VP9_SINGLE_REF_COUNT_SIZE      (10 * 2)
#define VP9_TX_MODE_COUNT_START        \
		(VP9_SINGLE_REF_COUNT_START + VP9_SINGLE_REF_COUNT_SIZE)
#define VP9_TX_MODE_COUNT_SIZE         (12 * 2)
#define VP9_SKIP_COUNT_START           \
		(VP9_TX_MODE_COUNT_START + VP9_TX_MODE_COUNT_SIZE)
#define VP9_SKIP_COUNT_SIZE            (3 * 2)
#define VP9_MV_SIGN_0_COUNT_START      \
		(VP9_SKIP_COUNT_START + VP9_SKIP_COUNT_SIZE)
#define VP9_MV_SIGN_0_COUNT_SIZE       (1 * 2)
#define VP9_MV_SIGN_1_COUNT_START      \
		(VP9_MV_SIGN_0_COUNT_START + VP9_MV_SIGN_0_COUNT_SIZE)
#define VP9_MV_SIGN_1_COUNT_SIZE       (1 * 2)
#define VP9_MV_BITS_0_COUNT_START      \
		(VP9_MV_SIGN_1_COUNT_START + VP9_MV_SIGN_1_COUNT_SIZE)
#define VP9_MV_BITS_0_COUNT_SIZE       (10 * 2)
#define VP9_MV_BITS_1_COUNT_START      \
		(VP9_MV_BITS_0_COUNT_START + VP9_MV_BITS_0_COUNT_SIZE)
#define VP9_MV_BITS_1_COUNT_SIZE       (10 * 2)
#define VP9_MV_CLASS0_HP_0_COUNT_START \
		(VP9_MV_BITS_1_COUNT_START + VP9_MV_BITS_1_COUNT_SIZE)
#define VP9_MV_CLASS0_HP_0_COUNT_SIZE  (2 * 2)
#define VP9_MV_CLASS0_HP_1_COUNT_START \
		(VP9_MV_CLASS0_HP_0_COUNT_START + VP9_MV_CLASS0_HP_0_COUNT_SIZE)
#define VP9_MV_CLASS0_HP_1_COUNT_SIZE  (2 * 2)

/* Start merge_tree */
#define VP9_INTER_MODE_COUNT_START     \
		(VP9_MV_CLASS0_HP_1_COUNT_START + VP9_MV_CLASS0_HP_1_COUNT_SIZE)
#define VP9_INTER_MODE_COUNT_SIZE      (7 * 4)
#define VP9_IF_Y_MODE_COUNT_START      \
		(VP9_INTER_MODE_COUNT_START + VP9_INTER_MODE_COUNT_SIZE)
#define VP9_IF_Y_MODE_COUNT_SIZE       (10 * 4)
#define VP9_IF_UV_MODE_COUNT_START     \
		(VP9_IF_Y_MODE_COUNT_START + VP9_IF_Y_MODE_COUNT_SIZE)
#define VP9_IF_UV_MODE_COUNT_SIZE      (10 * 10)
#define VP9_PARTITION_P_COUNT_START    \
		(VP9_IF_UV_MODE_COUNT_START + VP9_IF_UV_MODE_COUNT_SIZE)
#define VP9_PARTITION_P_COUNT_SIZE     (4 * 4 * 4)
#define VP9_INTERP_COUNT_START         \
		(VP9_PARTITION_P_COUNT_START + VP9_PARTITION_P_COUNT_SIZE)
#define VP9_INTERP_COUNT_SIZE          (4 * 3)
#define VP9_MV_JOINTS_COUNT_START      \
		(VP9_INTERP_COUNT_START + VP9_INTERP_COUNT_SIZE)
#define VP9_MV_JOINTS_COUNT_SIZE       (1 * 4)
#define VP9_MV_CLASSES_0_COUNT_START   \
		(VP9_MV_JOINTS_COUNT_START + VP9_MV_JOINTS_COUNT_SIZE)
#define VP9_MV_CLASSES_0_COUNT_SIZE    (1 * 11)
#define VP9_MV_CLASS0_0_COUNT_START    \
		(VP9_MV_CLASSES_0_COUNT_START + VP9_MV_CLASSES_0_COUNT_SIZE)
#define VP9_MV_CLASS0_0_COUNT_SIZE     (1 * 2)
#define VP9_MV_CLASSES_1_COUNT_START   \
		(VP9_MV_CLASS0_0_COUNT_START + VP9_MV_CLASS0_0_COUNT_SIZE)
#define VP9_MV_CLASSES_1_COUNT_SIZE    (1 * 11)
#define VP9_MV_CLASS0_1_COUNT_START    \
		(VP9_MV_CLASSES_1_COUNT_START + VP9_MV_CLASSES_1_COUNT_SIZE)
#define VP9_MV_CLASS0_1_COUNT_SIZE     (1 * 2)
#define VP9_MV_CLASS0_FP_0_COUNT_START \
		(VP9_MV_CLASS0_1_COUNT_START + VP9_MV_CLASS0_1_COUNT_SIZE)
#define VP9_MV_CLASS0_FP_0_COUNT_SIZE  (3 * 4)
#define VP9_MV_CLASS0_FP_1_COUNT_START \
		(VP9_MV_CLASS0_FP_0_COUNT_START + VP9_MV_CLASS0_FP_0_COUNT_SIZE)
#define VP9_MV_CLASS0_FP_1_COUNT_SIZE  (3 * 4)

#define DC_PRED    0	/* Average of above and left pixels */
#define V_PRED     1	/* Vertical */
#define H_PRED     2	/* Horizontal */
#define D45_PRED   3	/* Directional 45 deg = round(arctan(1/1) * 180/pi) */
#define D135_PRED  4	/* Directional 135 deg = 180 - 45 */
#define D117_PRED  5	/* Directional 117 deg = 180 - 63 */
#define D153_PRED  6	/* Directional 153 deg = 180 - 27 */
#define D207_PRED  7	/* Directional 207 deg = 180 + 27 */
#define D63_PRED   8	/* Directional 63 deg = round(arctan(2/1) * 180/pi) */
#define TM_PRED    9	/* True-motion */

/* Forward declarations */
static void codec_vp9_set_sao(struct amvdec_session *sess,
			      struct vb2_buffer *vb);

/* Use a static inline to avoid possible side effect from num being reused */
static inline int round_power_of_two(int value, int num)
{
	return (value + (1 << (num - 1))) >> num;
}

#define MODE_MV_COUNT_SAT 20
static const int count_to_update_factor[MODE_MV_COUNT_SAT + 1] = {
	0, 6, 12, 19, 25, 32, 38, 44, 51, 57, 64,
	70, 76, 83, 89, 96, 102, 108, 115, 121, 128
};

union rpm_param {
	struct {
		u16 data[RPM_BUF_SIZE];
	} l;
	struct {
		u16 profile;
		u16 show_existing_frame;
		u16 frame_to_show_idx;
		u16 frame_type; /*1 bit*/
		u16 show_frame; /*1 bit*/
		u16 error_resilient_mode; /*1 bit*/
		u16 intra_only; /*1 bit*/
		u16 display_size_present; /*1 bit*/
		u16 reset_frame_context;
		u16 refresh_frame_flags;
		u16 width;
		u16 height;
		u16 display_width;
		u16 display_height;
		u16 ref_info;
		u16 same_frame_size;
		u16 mode_ref_delta_enabled;
		u16 ref_deltas[4];
		u16 mode_deltas[2];
		u16 filter_level;
		u16 sharpness_level;
		u16 bit_depth;
		u16 seg_quant_info[8];
		u16 seg_enabled;
		u16 seg_abs_delta;
		/* bit 15: feature enabled; bit 8, sign; bit[5:0], data */
		u16 seg_lf_info[8];
	} p;
};

enum SEG_LVL_FEATURES {
	SEG_LVL_ALT_Q = 0,	/* Use alternate Quantizer */
	SEG_LVL_ALT_LF = 1,	/* Use alternate loop filter value */
	SEG_LVL_REF_FRAME = 2,	/* Optional Segment reference frame */
	SEG_LVL_SKIP = 3,	/* Optional Segment (0,0) + skip mode */
	SEG_LVL_MAX = 4		/* Number of features supported */
};

struct segmentation {
	u8 enabled;
	u8 update_map;
	u8 update_data;
	u8 abs_delta;
	u8 temporal_update;
	s16 feature_data[MAX_SEGMENTS][SEG_LVL_MAX];
	unsigned int feature_mask[MAX_SEGMENTS];
};

struct loop_filter_thresh {
	u8 mblim;
	u8 lim;
	u8 hev_thr;
};

struct loop_filter_info_n {
	struct loop_filter_thresh lfthr[MAX_LOOP_FILTER + 1];
	u8 lvl[MAX_SEGMENTS][MAX_REF_FRAMES][MAX_MODE_LF_DELTAS];
};

struct loopfilter {
	int filter_level;

	int sharpness_level;
	int last_sharpness_level;

	u8 mode_ref_delta_enabled;
	u8 mode_ref_delta_update;

	/*0 = Intra, Last, GF, ARF*/
	signed char ref_deltas[MAX_REF_LF_DELTAS];
	signed char last_ref_deltas[MAX_REF_LF_DELTAS];

	/*0 = ZERO_MV, MV*/
	signed char mode_deltas[MAX_MODE_LF_DELTAS];
	signed char last_mode_deltas[MAX_MODE_LF_DELTAS];
};

struct vp9_frame {
	struct list_head list;
	struct vb2_v4l2_buffer *vbuf;
	int index;
	int intra_only;
	int show;
	int type;
	int done;
	unsigned int width;
	unsigned int height;
};

struct codec_vp9 {
	/* VP9 context lock */
	struct mutex lock;

	/* Common part with the HEVC decoder */
	struct codec_hevc_common common;

	/* Buffer for the VP9 Workspace */
	void      *workspace_vaddr;
	dma_addr_t workspace_paddr;
	u32        workspace_size;

	/* Per-platform MPRED MV buffer sizing (from cal_mv_buf_size) */
	u32 mpred_mv_buf_size;	/* per-slot size */
	u32 mpred_mv_total;	/* mpred_mv_buf_size * MAX_REF_PIC_NUM */
	u32 rpm_offset;		/* MPRED_MV_OFFSET + mpred_mv_total */
	u32 lmem_offset;	/* rpm_offset + RPM_BUF_SIZE */

	/* Contains many information parsed from the bitstream */
	union rpm_param rpm_param;

	/* Whether we detected the bitstream as 10-bit */
	int is_10bit;

	/* Whether the per-session 10-bit capture-mode warn has fired
	 * yet.  The message is informational ("how is 10-bit being
	 * delivered?") and depends only on capture pixfmt, so once
	 * per session is enough -- per-frame logging is noise.
	 */
	bool warned_10bit_capture;

	/* Coded resolution reported by the hardware */
	u32 width, height;

	/* All ref frames used by the HW at a given time */
	struct list_head ref_frames_list;
	u32 frames_num;

	/* In case of downsampling (decoding with FBC but outputting in NV12M),
	 * we need to allocate additional buffers for FBC.
	 */
	void      *fbc_buffer_vaddr[MAX_REF_PIC_NUM];
	dma_addr_t fbc_buffer_paddr[MAX_REF_PIC_NUM];

	int ref_frame_map[REF_FRAMES];
	int next_ref_frame_map[REF_FRAMES];
	int prev_ref_frame_map[REF_FRAMES];
	struct vp9_frame *frame_refs[REFS_PER_FRAME];

	u32 lcu_total;
	u32 inter_lcu_count;	/* accumulated inter-frame LCUs for VBH ring tracking */

	/* loop filter */
	int default_filt_lvl;
	struct loop_filter_info_n lfi;
	struct loopfilter lf;
	struct segmentation seg_4lf;

	struct vp9_frame *cur_frame;
	struct vp9_frame *prev_frame;

	struct amvdec_session *sess;

	/*
	 * Arm-on-available: set while a parsed frame waits for a free
	 * CAPTURE buffer.  While set, the engine has not been touched,
	 * so neither a frame-done wait nor the watchdog can fire; the
	 * work below replays process_frame() once vdec_vb2_buf_queue()
	 * reports a buffer.
	 */
	atomic_t arm_deferred;
	struct work_struct deferred_arm_work;

	/* Wakes flush_output() when the 0xf2 VP9_EOS ISR has marked
	 * cur_frame->done.  Replaces the prior msleep(10) poll loop in
	 * flush_output (VP9-001 S3).  On current firmware revisions
	 * the 0xf2 ISR empirically never fires (vp9_eos counter stays
	 * 0 across every test session), so this wait_event will time
	 * out and flush_output falls through to the direct-delivery
	 * path -- but if a future firmware fires 0xf2, the waiter
	 * exits with sub-ms latency instead of polling 10ms ticks.
	 */
	wait_queue_head_t cur_frame_done_wq;
};

static int div_r32(s64 m, int n)
{
	s64 qu = div_s64(m, n);

	return (int)qu;
}

static int clip_prob(int p)
{
	return clamp_val(p, 1, 255);
}

static int segfeature_active(struct segmentation *seg, int segment_id,
			     enum SEG_LVL_FEATURES feature_id)
{
	return seg->enabled &&
		(seg->feature_mask[segment_id] & (1 << feature_id));
}

static int get_segdata(struct segmentation *seg, int segment_id,
		       enum SEG_LVL_FEATURES feature_id)
{
	return seg->feature_data[segment_id][feature_id];
}

static void vp9_update_sharpness(struct loop_filter_info_n *lfi,
				 int sharpness_lvl)
{
	int lvl;

	/* For each possible value for the loop filter fill out limits*/
	for (lvl = 0; lvl <= MAX_LOOP_FILTER; lvl++) {
		/* Set loop filter parameters that control sharpness.*/
		int block_inside_limit = lvl >> ((sharpness_lvl > 0) +
					(sharpness_lvl > 4));

		if (sharpness_lvl > 0) {
			if (block_inside_limit > (9 - sharpness_lvl))
				block_inside_limit = (9 - sharpness_lvl);
		}

		if (block_inside_limit < 1)
			block_inside_limit = 1;

		lfi->lfthr[lvl].lim = (u8)block_inside_limit;
		lfi->lfthr[lvl].mblim = (u8)(2 * (lvl + 2) +
				block_inside_limit);
	}
}

/* Instantiate this function once when decode is started */
static void
vp9_loop_filter_init(struct amvdec_core *core, struct codec_vp9 *vp9)
{
	struct loop_filter_info_n *lfi = &vp9->lfi;
	struct loopfilter *lf = &vp9->lf;
	struct segmentation *seg_4lf = &vp9->seg_4lf;
	int i;

	memset(lfi, 0, sizeof(struct loop_filter_info_n));
	memset(lf, 0, sizeof(struct loopfilter));
	memset(seg_4lf, 0, sizeof(struct segmentation));
	lf->sharpness_level = 0;
	vp9_update_sharpness(lfi, lf->sharpness_level);
	lf->last_sharpness_level = lf->sharpness_level;

	for (i = 0; i < 32; i++) {
		unsigned int thr;

		thr = ((lfi->lfthr[i * 2 + 1].lim & DBLK_LIM_MASK)
			<< DBLK_LIM_SHIFT) |
			(lfi->lfthr[i * 2 + 1].mblim & DBLK_MBLIM_MASK);
		thr = (thr << DBLK_THR_PAIR_SHIFT) |
			((lfi->lfthr[i * 2].lim & DBLK_LIM_MASK)
			 << DBLK_LIM_SHIFT) |
			(lfi->lfthr[i * 2].mblim & DBLK_MBLIM_MASK);

		amvdec_write_dos(core, HEVC_DBLK_CFG9, thr);
	}

	if (core->platform->revision >= VDEC_REVISION_SM1)
		amvdec_write_dos(core, HEVC_DBLK_CFGB,
				 DBLK_CFGB_SM1_DW_FIFO_THRES |
				 DBLK_CFGB_VP9_MODE);
	else if (core->platform->revision >= VDEC_REVISION_G12A)
		amvdec_write_dos(core, HEVC_DBLK_CFGB,
				 DBLK_CFGB_G12A_STRENGTH |
				 DBLK_CFGB_VP9_MODE);
	else
		amvdec_write_dos(core, HEVC_DBLK_CFGB,
				 DBLK_CFGB_GXL);
}

static void
vp9_loop_filter_frame_init(struct amvdec_core *core, struct segmentation *seg,
			   struct loop_filter_info_n *lfi,
			   struct loopfilter *lf, int default_filt_lvl)
{
	int i;
	int seg_id;

	/*
	 * n_shift is the multiplier for lf_deltas
	 * the multiplier is:
	 * - 1 for when filter_lvl is between 0 and 31
	 * - 2 when filter_lvl is between 32 and 63
	 */
	const int scale = 1 << (default_filt_lvl >> 5);

	/* update limits if sharpness has changed */
	if (lf->last_sharpness_level != lf->sharpness_level) {
		vp9_update_sharpness(lfi, lf->sharpness_level);
		lf->last_sharpness_level = lf->sharpness_level;

		/* Write to register */
		for (i = 0; i < 32; i++) {
			unsigned int thr;

			thr = ((lfi->lfthr[i * 2 + 1].lim & DBLK_LIM_MASK)
				<< DBLK_LIM_SHIFT) |
			      (lfi->lfthr[i * 2 + 1].mblim & DBLK_MBLIM_MASK);
			thr = (thr << DBLK_THR_PAIR_SHIFT) |
			      ((lfi->lfthr[i * 2].lim & DBLK_LIM_MASK)
			       << DBLK_LIM_SHIFT) |
			      (lfi->lfthr[i * 2].mblim & DBLK_MBLIM_MASK);

			amvdec_write_dos(core, HEVC_DBLK_CFG9, thr);
		}
	}

	for (seg_id = 0; seg_id < MAX_SEGMENTS; seg_id++) {
		int lvl_seg = default_filt_lvl;

		if (segfeature_active(seg, seg_id, SEG_LVL_ALT_LF)) {
			const int data = get_segdata(seg, seg_id,
						SEG_LVL_ALT_LF);
			lvl_seg = clamp_t(int,
					  seg->abs_delta == SEGMENT_ABSDATA ?
						data : default_filt_lvl + data,
					  0, MAX_LOOP_FILTER);
		}

		if (!lf->mode_ref_delta_enabled) {
			/*
			 * We could get rid of this if we assume that deltas
			 * are set to zero when not in use.
			 * encoder always uses deltas
			 */
			memset(lfi->lvl[seg_id], lvl_seg,
			       sizeof(lfi->lvl[seg_id]));
		} else {
			int ref, mode;
			const int intra_lvl =
				lvl_seg + lf->ref_deltas[INTRA_FRAME] * scale;
			lfi->lvl[seg_id][INTRA_FRAME][0] =
				clamp_val(intra_lvl, 0, MAX_LOOP_FILTER);

			for (ref = LAST_FRAME; ref < MAX_REF_FRAMES; ++ref) {
				for (mode = 0; mode < MAX_MODE_LF_DELTAS;
				     ++mode) {
					const int inter_lvl =
						lvl_seg +
						lf->ref_deltas[ref] * scale +
						lf->mode_deltas[mode] * scale;
					lfi->lvl[seg_id][ref][mode] =
						clamp_val(inter_lvl, 0,
							  MAX_LOOP_FILTER);
				}
			}
		}
	}

	for (i = 0; i < 16; i++) {
		unsigned int level;

		level = ((lfi->lvl[i >> 1][3][i & 1] & DBLK_LVL_MASK) << 24) |
			((lfi->lvl[i >> 1][2][i & 1] & DBLK_LVL_MASK) << 16) |
			((lfi->lvl[i >> 1][1][i & 1] & DBLK_LVL_MASK) << 8) |
			(lfi->lvl[i >> 1][0][i & 1] & DBLK_LVL_MASK);
		if (!default_filt_lvl)
			level = 0;

		amvdec_write_dos(core, HEVC_DBLK_CFGA, level);
	}
}

/*
 * VP9 EOS sentinel: AMLV-wrapped packet with 0x55555555 payload.
 * When the firmware encounters 0x55555555 as the first payload word
 * of an AMLV packet (sm1_vp9_mmu.asm 0x0952-0x0958), it calls the
 * SAO barrier subroutine (flushing the double-write pipeline) and
 * fires VP9_EOS (0xf2) via MBOX1.  The driver's 0xf2 ISR handler
 * delivers the last frame with SAO/DW pipeline fully drained.
 *
 * Format (matches vp9_update_header AMLV framing):
 *   [0-3]   framesize+4 = 8 (big-endian)
 *   [4-7]   XOR complement = ~8
 *   [8-11]  startcode 0x00000001
 *   [12-15] "AMLV"
 *   [16-19] sentinel 0x55555555
 */
static const u8 vp9_eos_sequence[SZ_1K] __aligned(64) = {
	/* AMLV header: framesize+4, complement, startcode, "AMLV" */
	0x00, 0x00, 0x04, 0x04,	/* framesize + 4 = 1028 (0x404) */
	0xff, 0xff, 0xfb, 0xfb,	/* XOR complement */
	0x00, 0x00, 0x00, 0x01,	/* startcode */
	0x41, 0x4d, 0x4c, 0x56,	/* "AMLV" */
	0x55, 0x55, 0x55, 0x55,	/* sentinel (first 4 bytes of payload) */
	/* remaining 1004 bytes are zero (padding for firmware size check) */
};

/* Wait condition for the cur_frame_done_wq wait_event in flush_output.
 * Reads vp9->cur_frame and ->done outside vp9->lock with READ_ONCE.
 * The 0xf2 ISR sets ->done before clearing cur_frame (cur_frame is only
 * cleared on reset, which can't happen until drain_finalize), so the
 * snapshotted pointer is stable for the dereference.  False negative
 * just causes an extra wait_event iteration; false positive is not
 * possible because done is monotonic 0->1 within a frame's lifetime.
 */
static bool codec_vp9_flush_done(struct codec_vp9 *vp9)
{
	struct vp9_frame *f = READ_ONCE(vp9->cur_frame);

	return f && READ_ONCE(f->done);
}

static void codec_vp9_flush_output(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_vp9 *vp9 = sess->priv;
	bool cap_live;

	/*
	 * VP9-001 fix: wait for all real frames to be parsed, then inject
	 * the 0x55555555 sentinel to flush the double-write pipeline.
	 *
	 * Step 1: Wait for esparser_queued_bufs == 0 (all input buffers
	 * processed, all 0xf0 ISRs fired).  This is faster than the old
	 * 50ms-idle heuristic and critical for GXL where firmware fires
	 * IRQs for ~10s post-EOS (the 50ms idle wait would take 10s,
	 * by which time the firmware stops searching the VIFIFO).
	 *
	 * Step 2: Inject AMLV-wrapped 0x55555555 sentinel.  The firmware
	 * is actively searching for the next AMLV header and will find
	 * the sentinel immediately.  It calls the SAO barrier (flushing
	 * the DW pipeline) and fires VP9_EOS (0xf2).
	 *
	 * Step 3: Wait for 0xf2 ISR to deliver cur_frame (done==1).
	 * Fallback to direct delivery if 0xf2 doesn't fire in 500ms.
	 */
	/*
	 * VP9-001 fix: inject sentinel immediately, then wait for 0xf2.
	 *
	 * The sentinel sits in the VIFIFO behind any remaining real frame
	 * data.  The firmware processes frames sequentially, so it will
	 * find the sentinel only after all real frames are parsed/decoded.
	 * No pre-wait needed -- the VIFIFO is a FIFO.
	 *
	 * On GXL, the firmware fires post-EOS IRQs for ~10s.  Waiting for
	 * queued_bufs==0 or 50ms idle causes ffmpeg to STREAMOFF before
	 * the sentinel can be injected.  Injecting immediately avoids this.
	 */
	/*
	 * VP9-001: inject AMLV-wrapped 0x55 sentinel and wait for the
	 * firmware's VP9_EOS (0xf2) ISR.  Applies to ALL platforms
	 * including GXL.
	 *
	 * Historical note: the GXL revision was gated out of this path
	 * on the belief that the sentinel "races with decode and causes
	 * mid-stream 0xf2/frame loss".  A 2026-04-19 GXL EOS probe
	 * showed that without the sentinel, frame 30's SAO output is
	 * empty (SAO_Y_WPTR == SAO_Y_START, SAO_C_WPTR == 0) -- frame
	 * 30 is simply dropped on GXL, delivering 29/30.  Re-enabling
	 * the sentinel on GXL (post-ABBA-fix + drain_finalize) now
	 * delivers 30/30 with frame 30 at ~31 dB (the documented
	 * bottom-rows DW tail artifact), with zero mid-stream loss
	 * verified on 30-frame and 90-frame test clips.  The old race
	 * was likely a symptom of pre-fix teardown / drain ordering.
	 */
	if (1) {
		int eos_ret;

		/*
		 * VP9-001: wait for all input buffers to be consumed before
		 * injecting the sentinel.  Without this, the sentinel enters
		 * the VIFIFO ahead of the last frame's data and the firmware
		 * finds it before decoding the last frame.
		 *
		 * esparser_queued_bufs tracks input buffers pushed to the
		 * esparser minus decoded frames delivered.  When it reaches
		 * 0, all frames have been decoded and delivered.
		 *
		 * Timeout scales linearly with pixel count (anchored at 6K =
		 * 10s), floored at 1s and capped at 10s.  Reference: SM1 6K
		 * decodes ~2.4 s/frame; 10s gives 4-frame headroom for the
		 * decode-catch-up case (`aa944aada4dc` empirical baseline).
		 * Smaller resolutions decode faster -- 720p ~106ms/frame,
		 * 1080p ~238ms/frame, 4K ~953ms/frame -- so a 4-frame
		 * catch-up budget for them is well under 10s.  Paying the
		 * full 10s on every EOS for a 720p clip was wasted: the wait
		 * times out in the DPB-stuck case (firmware holds frame N
		 * with no N+1 to advance the pipeline; queued_bufs stays at
		 * 1) and frame N is then delivered via the direct-delivery
		 * fallback regardless.  Adaptive timeout cuts ~9s of EOS
		 * wall-clock at 720p/1080p without changing the 6K behaviour.
		 *
		 * Linear scale formula: timeout_ms = pixels * 10000 /
		 * pixels_at_6k.  6K = 6144x3456 = 21,233,664 px.  At 720p
		 * (921,600 px) this gives 434ms -> floor 1000ms.  At 4K it
		 * gives 3906ms.  At 6K it gives 10000ms (unchanged).  At
		 * 8K it gives 15625ms -> ceiling 10000ms (unchanged).
		 *
		 * VP9-001 S3: wait_event_timeout on existing
		 * esparser_queue_done_wq (wakeups fire on ISR + esparser
		 * completion).  Long-clip case where decode genuinely can't
		 * catch up at 6K still falls to skip-sentinel +
		 * direct-delivery (trigger is queued_bufs > 0 at exit, not
		 * wall-clock).  Bounded by vdec_drain_work's own
		 * wait_event_timeout (5s) above.
		 */
		{
			unsigned int pixels = sess->width * sess->height;
			unsigned int timeout_ms = clamp_t(unsigned int,
				div_u64((u64)pixels * 10000ULL, 21233664ULL),
				1000U, 10000U);
			unsigned int short_wait_ms = 200;
			int queued;

			/*
			 * Two-stage wait:
			 *
			 * Stage A: short 200ms wait for queued_bufs to drain
			 * naturally.  Catches the steady-state EOS case at
			 * small resolutions (decode is fast; queued_bufs hits
			 * 0 in <100ms typical).
			 *
			 * Stage B: only entered if Stage A timed out with
			 * queued_bufs > 1.  Slow-decode case (4K+ on
			 * decode-bound platforms, 6K on SM1).  Wait up to
			 * (adaptive_timeout - 200ms) for the remaining frames
			 * to drain.
			 *
			 * The "queued_bufs == 1" case (DPB-stuck end-of-stream)
			 * exits Stage A without entering Stage B -- frame N
			 * will never decode without help and waiting longer
			 * doesn't change that.  This is the most common path
			 * at 1080p and below; saving the long wait here is
			 * the dominant wall-clock win.
			 */
			wait_event_timeout(sess->esparser_queue_done_wq,
					   atomic_read(&sess->esparser_queued_bufs)
					   <= 0,
					   msecs_to_jiffies(short_wait_ms));

			queued = atomic_read(&sess->esparser_queued_bufs);
			if (queued > 1) {
				wait_event_timeout(sess->esparser_queue_done_wq,
						   atomic_read(&sess->esparser_queued_bufs)
						   <= 0,
						   msecs_to_jiffies(timeout_ms - short_wait_ms));
				queued = atomic_read(&sess->esparser_queued_bufs);
			}

			/*
			 * VP9-001 safety: skip the sentinel when stuck-frame
			 * count exceeds what the firmware can flush via a
			 * single sentinel push.
			 *
			 * queued > 1 (any platform): real frames still in
			 * flight; injecting sentinel would enter the VIFIFO
			 * mid-stream and cause decode loss.  Skip and fall
			 * through to direct-delivery.  At worst this reverts
			 * to the prior 29/30 behavior for that session.
			 *
			 * queued == 1 on GXL: KI-102 (DW-tail HW limit) means
			 * SAO_Y_WPTR doesn't advance for frame N even with
			 * sentinel injected, so the post-sentinel
			 * cur_frame_done wait times out and direct-delivery
			 * runs anyway -- but we paid 100ms for nothing.
			 * Skip-sentinel on GXL preserves the option-3 floor
			 * benefit without that 100ms tax.  (Tested 2026-04-21
			 * topic 0c13e62a05a3: GXL 720p-300f wall-time
			 * regressed +2s pre-2a2a17b 100ms-floor; post-floor
			 * it's +100ms but still no benefit on GXL.)
			 *
			 * queued == 1 on G12A+: DPB-stuck end-of-stream.
			 * Inject the sentinel anyway -- the firmware DW
			 * pipeline advance completes frame N's tail before
			 * drain_finalize halts the firmware, and frame N is
			 * then delivered via direct-delivery the same way as
			 * the queued == 0 path.  100ms post-sentinel wait is
			 * the cost; the saving is the much larger Stage B
			 * timeout (700ms+ at 720p) we now bypass.
			 */
			if (queued > 1 ||
			    (queued == 1 && core->platform->revision == VDEC_REVISION_GXL)) {
				dev_info(core->dev_dec,
					 "VP9-001: queued_bufs wait timed out (%d remaining); skipping sentinel\n",
					 queued);
				goto sentinel_skipped;
			}
		}

		eos_ret = esparser_queue_eos(core, vp9_eos_sequence,
					     sizeof(vp9_eos_sequence));
		dev_dbg(core->dev_dec,
			"VP9-001: sentinel injected (%zu bytes, ret=%d)\n",
			sizeof(vp9_eos_sequence), eos_ret);

		/*
		 * Wait for 0xf2 ISR to deliver cur_frame.
		 *
		 * Empirically, the 0xf2 VP9_EOS ISR NEVER fires on any
		 * current Amlogic VP9 platform (GXL, G12B, SM1).  A 2026-04-22
		 * probe on SM1 `12b73ca824a8` showed the
		 * `/sys/kernel/debug/meson-vdec/irq_counters` vp9_eos counter
		 * stays 0 after every 30f + 300f decode session.  The sentinel
		 * mechanism works by triggering set_sao() on a fake "frame"
		 * that advances the DW pipeline by one stage, completing the
		 * real frame N's tail.  Frame N is then delivered via the
		 * direct-delivery fallback below, not via an 0xf2 ISR.
		 *
		 * This loop therefore always times out on current firmware.
		 * 100ms is enough for the sentinel to propagate through the
		 * VIFIFO + be parsed by the firmware's AMLV search; a real
		 * 0xf2 fire (if a future firmware revision starts firing it)
		 * would complete well under 100ms.  Prior 2s value was pure
		 * wait for a contract that never holds.
		 *
		 * Savings: up to ~1.9s per sentinel-injected flush_output
		 * invocation.  Frame counts unchanged -- the fallback path
		 * was always delivering frame N anyway.
		 */
		/* VP9-001 S3: wait_event on cur_frame_done_wq instead of
		 * the prior msleep(10) poll loop.  The 0xf2 VP9_EOS ISR
		 * (codec_vp9_threaded_isr) wakes the wq after marking
		 * cur_frame->done; on current firmware that ISR never
		 * fires and the wait simply times out, falling through to
		 * the direct-delivery path below.  If a future firmware
		 * fires 0xf2 the waiter exits with sub-ms latency.
		 *
		 * Exit ONLY when cur_frame is non-NULL and done=1.  Do NOT
		 * exit when cur_frame is NULL: between ISRs cur_frame is
		 * NULL (cleared by flush_output cleanup on reset, or set
		 * to NULL if get_new_frame fails).  Exiting on NULL causes
		 * premature DPB teardown (TODO-VP9-010).
		 *
		 * Reading cur_frame outside vp9->lock here is safe for the
		 * wait condition because:
		 *   - if cur_frame->done flips 0->1, the ISR wakes the wq
		 *     and the next condition re-evaluation under lock
		 *     observes the new value;
		 *   - if cur_frame is NULL we wait the full timeout, same
		 *     as the prior poll.
		 * The actual delivery path (codec_vp9_set_sao + dst_buf_done)
		 * still runs under vp9->lock below.
		 */
		wait_event_timeout(vp9->cur_frame_done_wq,
				   codec_vp9_flush_done(vp9),
				   msecs_to_jiffies(100));
sentinel_skipped:
		;
	}

	mutex_lock(&vp9->lock);
	/*
	 * Once CAPTURE has stopped, the vbufs the frame list still points
	 * at are no longer ours: vb2 completed them in __vb2_queue_cancel()
	 * and the client may already have freed their memory (Android's
	 * v4l2_codec2 stops CAPTURE before OUTPUT, so this flush runs from
	 * the OUTPUT stop's poweroff).  Programming SAO on one dereferenced a
	 * NULL plane cookie (GXL, end of a 300-frame clip).  Then only the
	 * bookkeeping below is freed: nothing is delivered or requeued.
	 */
	cap_live = sess->streamon_cap;

	/* Deliver cur_frame if 0xf2 ISR didn't (fallback) */
	if (cap_live && vp9->cur_frame && vp9->cur_frame->show &&
	    !vp9->cur_frame->done) {
		/*
		 * VP9-001 GXL: re-call codec_vp9_set_sao() to flush the
		 * DW pipeline.  For frames 1-N-1, the NEXT frame's set_sao()
		 * implicitly completes the DW write for the current frame
		 * (confirmed: bottom ~2 LCU rows corrupt without this).
		 * At EOS, no next frame exists.  Re-calling set_sao() with
		 * the same buffer triggers the DW hardware to advance its
		 * internal pipeline and complete the write.
		 */
		codec_vp9_set_sao(sess,
				  &vp9->cur_frame->vbuf->vb2_buf);

		{
			u32 sao_if = 0;
			int i, idle_stable = 0;

			/*
			 * Wait for this last frame's SAO double-write to drain
			 * before delivery.  The old !sao_if + read-only C_WPTR
			 * test never matched (SAO_IF_STATUS idles at
			 * HEVC_SAO_IF_IDLE_VALUE -- or 0x0 once the engine has
			 * quiesced -- never literally 0 with the high bits;
			 * SAO_C_WPTR is read-only) so it always burned the full
			 * cap.  Exit when SAO_IF_STATUS shows no bit OUTSIDE the
			 * idle baseline for HEVC_DRAIN_STABLE consecutive reads
			 * (SAO idle).  The codec_vp9_set_sao() call just above
			 * re-triggers the DW flush, which holds a bit outside the
			 * baseline until it completes, so a still-running flush
			 * keeps us looping rather than exiting into a
			 * half-flushed pipeline (VP9 bottom-LCU-row chroma).
			 */
			for (i = 0; i < 2000; i++) {
				sao_if = amvdec_read_dos(core,
							 HEVC_SAO_IF_STATUS);
				if ((sao_if & ~HEVC_SAO_IF_IDLE_VALUE) == 0) {
					if (++idle_stable >= HEVC_DRAIN_STABLE)
						break;
				} else {
					idle_stable = 0;
				}
				udelay(50);
			}
			dev_dbg(core->dev_dec,
				"VP9 drain: SAO poll %d loops (if=0x%x)\n",
				i, sao_if);
		}

		amvdec_dst_buf_done(sess, vp9->cur_frame->vbuf,
				    V4L2_FIELD_NONE);
		vp9->cur_frame->done = 1;
		vp9->frames_num--;
	}

	/*
	 * Return all remaining reference frame buffers to VB2.
	 * Without this, buffers held as VP9 references are never returned
	 * before queue teardown, causing a __vb2_queue_cancel WARNING
	 * (KI-007).  Show frames that haven't been output yet are delivered
	 * to userspace; non-show frames are re-queued so the
	 * v4l2_m2m_dst_buf_remove loop in vdec_stop_streaming picks them up.
	 */
	{
		struct vp9_frame *tmp, *n;

		list_for_each_entry_safe(tmp, n, &vp9->ref_frames_list, list) {
			if (!tmp->done && cap_live) {
				if (tmp->show)
					amvdec_dst_buf_done(sess, tmp->vbuf,
							    V4L2_FIELD_NONE);
				else
					v4l2_m2m_buf_queue(sess->m2m_ctx,
							   tmp->vbuf);
			}
			if (!tmp->done)
				vp9->frames_num--;

			list_del(&tmp->list);
			kfree(tmp);
		}
	}
	vp9->cur_frame = NULL;
	vp9->prev_frame = NULL;
	/*
	 * Reset ref maps to -1 so any post-flush ISRs that fire before the
	 * session fully tears down do not attempt stale buffer-index lookups
	 * in sync_ref() (TODO-VP9-010).
	 */
	memset(vp9->ref_frame_map, -1, sizeof(vp9->ref_frame_map));
	memset(vp9->next_ref_frame_map, -1, sizeof(vp9->next_ref_frame_map));
	memset(vp9->prev_ref_frame_map, -1, sizeof(vp9->prev_ref_frame_map));
	mutex_unlock(&vp9->lock);
}

static u32 codec_vp9_num_pending_bufs(struct amvdec_session *sess)
{
	struct codec_vp9 *vp9 = sess->priv;

	if (!vp9)
		return 0;

	return vp9->frames_num;
}

/*
 * Lay the workspace out for a given per-slot MPRED MV size.  Everything
 * below MPRED_MV_OFFSET is fixed; the RPM and LMEM regions float above
 * the MV slot array, so they move whenever the MV size changes.
 */
static void codec_vp9_layout_workspace(struct codec_vp9 *vp9, u32 mv_buf_size)
{
	vp9->mpred_mv_buf_size = mv_buf_size;
	vp9->mpred_mv_total = vp9->mpred_mv_buf_size * MAX_REF_PIC_NUM;
	vp9->rpm_offset = MPRED_MV_OFFSET + vp9->mpred_mv_total;
	vp9->lmem_offset = vp9->rpm_offset + RPM_BUF_SIZE;
	vp9->workspace_size = ALIGN(vp9->lmem_offset + LMEM_SIZE, 64 * SZ_1K);
}

/* Per-slot MPRED MV size needed for a resolution, clamped to the platform max */
static u32 codec_vp9_mv_size_for(struct amvdec_session *sess, u32 w, u32 h)
{
	u32 max_w = sess->fmt_out->max_width;
	u32 max_h = sess->fmt_out->max_height;

	/* sess->width/height default to 1280x720 at session init and are set
	 * by S_FMT before poweron, so they are always non-zero here; clamp
	 * defensively anyway.
	 */
	w = clamp(w ? w : max_w, 64u, max_w);
	h = clamp(h ? h : max_h, 64u, max_h);

	return cal_mv_buf_size(w, h);
}

static int codec_vp9_alloc_workspace(struct amvdec_session *sess,
				     struct codec_vp9 *vp9)
{
	struct amvdec_core *core = sess->core;

	/*
	 * Size the MPRED MV slot array from the NEGOTIATED resolution, not
	 * from the platform maximum.  The workspace is a single contiguous
	 * DMA allocation addressed by the hardware as a physical base plus
	 * fixed offsets, so the MMU (which only covers the FBC compressed
	 * frame bodies) cannot scatter it -- sizing it for the 8192x4320
	 * silicon ceiling made every VP9 session, at any resolution, demand
	 * 0x7520000 (122 MB) of contiguous CMA.  On a 128 MB CMA board that
	 * is unsatisfiable, so even a 320x240 clip failed at
	 * dma_alloc_coherent.
	 *
	 * cal_mv_buf_size() is the same BSP formula, just evaluated at the
	 * resolution actually being decoded.  codec_vp9_grow_workspace()
	 * below re-allocates if the bitstream turns out to be larger than
	 * what S_FMT declared.
	 */
	codec_vp9_layout_workspace(vp9,
				   codec_vp9_mv_size_for(sess, sess->width,
							 sess->height));

	dev_info(core->dev,
		 "VP9 workspace: mpred_mv=%#x/slot (%ux%u coded, %ux%u max), total=%#x bytes\n",
		 vp9->mpred_mv_buf_size, sess->width, sess->height,
		 sess->fmt_out->max_width, sess->fmt_out->max_height,
		 vp9->workspace_size);

	vp9->workspace_vaddr = dma_alloc_coherent(core->dev,
						  vp9->workspace_size,
						  &vp9->workspace_paddr,
						  GFP_KERNEL);
	if (!vp9->workspace_vaddr) {
		dev_err(core->dev, "Failed to allocate VP9 Workspace\n");
		return -ENOMEM;
	}

	return 0;
}

/*
 * Re-allocate the workspace when the bitstream's coded resolution needs
 * larger MPRED MV slots than the one the workspace was sized for (S_FMT
 * under-declared the resolution, or the stream resized mid-decode).
 *
 * Called from resume(), which is the single point where the workspace
 * registers are (re)programmed and no decode is in flight.  MV slot
 * contents are per-reference-frame prediction state; at a resolution
 * change the references are invalidated anyway, and on the fast-resume
 * path (resolution unchanged) this is a no-op.
 */
static int codec_vp9_grow_workspace(struct amvdec_session *sess,
				    struct codec_vp9 *vp9)
{
	struct amvdec_core *core = sess->core;
	u32 need = codec_vp9_mv_size_for(sess, sess->width, sess->height);
	u32 old_mv = vp9->mpred_mv_buf_size;
	u32 old_size = vp9->workspace_size;
	dma_addr_t paddr;
	void *vaddr;

	if (need <= old_mv)
		return 0;

	codec_vp9_layout_workspace(vp9, need);

	vaddr = dma_alloc_coherent(core->dev, vp9->workspace_size, &paddr,
				   GFP_KERNEL);
	if (!vaddr) {
		dev_err(core->dev,
			"Failed to grow VP9 Workspace to %#x bytes for %ux%u\n",
			vp9->workspace_size, sess->width, sess->height);
		codec_vp9_layout_workspace(vp9, old_mv);
		return -ENOMEM;
	}

	dma_free_coherent(core->dev, old_size, vp9->workspace_vaddr,
			  vp9->workspace_paddr);
	vp9->workspace_vaddr = vaddr;
	vp9->workspace_paddr = paddr;

	dev_info(core->dev,
		 "VP9 workspace grown for %ux%u: mpred_mv=%#x/slot, total=%#x bytes\n",
		 sess->width, sess->height, vp9->mpred_mv_buf_size,
		 vp9->workspace_size);

	return 0;
}

static void codec_vp9_setup_workspace(struct amvdec_session *sess,
				      struct codec_vp9 *vp9)
{
	struct amvdec_core *core = sess->core;
	u32 revision = core->platform->revision;
	dma_addr_t wkaddr = vp9->workspace_paddr;

	amvdec_write_dos(core, HEVCD_IPP_LINEBUFF_BASE, wkaddr + IPP_OFFSET);
	amvdec_write_dos(core, VP9_RPM_BUFFER, wkaddr + vp9->rpm_offset);
	amvdec_write_dos(core, VP9_SHORT_TERM_RPS, wkaddr + SH_TM_RPS_OFFSET);
	amvdec_write_dos(core, VP9_PPS_BUFFER, wkaddr + PPS_OFFSET);
	amvdec_write_dos(core, VP9_SAO_UP, wkaddr + SAO_UP_OFFSET);

	amvdec_write_dos(core, VP9_STREAM_SWAP_BUFFER,
			 wkaddr + SWAP_BUF_OFFSET);
	amvdec_write_dos(core, VP9_STREAM_SWAP_BUFFER2,
			 wkaddr + SWAP_BUF2_OFFSET);
	amvdec_write_dos(core, VP9_SCALELUT, wkaddr + SCALELUT_OFFSET);

	if (core->platform->revision >= VDEC_REVISION_G12A)
		amvdec_write_dos(core, HEVC_DBLK_CFGE,
				 wkaddr + DBLK_PARA_OFFSET);

	amvdec_write_dos(core, HEVC_DBLK_CFG4, wkaddr + DBLK_PARA_OFFSET);
	amvdec_write_dos(core, HEVC_DBLK_CFG5, wkaddr + DBLK_DATA_OFFSET);

	/* SM1+ needs DBLK_CFG3 for deblocking left storage offsets */
	if (sess->core->platform->revision >= VDEC_REVISION_SM1)
		amvdec_write_dos(core, HEVC_DBLK_CFG3,
				 DBLK_CFG3_SM1_LEFT_STORAGE);

	amvdec_write_dos(core, VP9_SEG_MAP_BUFFER, wkaddr + SEG_MAP_OFFSET);
	amvdec_write_dos(core, VP9_PROB_SWAP_BUFFER, wkaddr + PROB_OFFSET);
	amvdec_write_dos(core, VP9_COUNT_SWAP_BUFFER, wkaddr + COUNT_OFFSET);
	amvdec_write_dos(core, LMEM_DUMP_ADR, wkaddr + vp9->lmem_offset);

	if (codec_hevc_use_mmu(revision, sess->pixfmt_cap, vp9->is_10bit)) {
		amvdec_write_dos(core, HEVC_SAO_MMU_VH0_ADDR,
				 wkaddr + MMU_VBH_OFFSET);
		amvdec_write_dos(core, HEVC_SAO_MMU_VH1_ADDR,
				 wkaddr + MMU_VBH_OFFSET + (MMU_VBH_SIZE / 2));

		if (revision >= VDEC_REVISION_G12A)
			amvdec_write_dos(core, HEVC_ASSIST_MMU_MAP_ADDR,
					 vp9->common.mmu_map_paddr);
		else
			amvdec_write_dos(core, VP9_MMU_MAP_BUFFER,
					 vp9->common.mmu_map_paddr);
	}
}

static void codec_vp9_process_frame(struct amvdec_session *sess);
static void codec_vp9_deferred_arm_work(struct work_struct *work);

static int codec_vp9_start(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_vp9 *vp9;
	u32 val;
	int i;
	int ret;

	vp9 = kzalloc(sizeof(*vp9), GFP_KERNEL);
	if (!vp9)
		return -ENOMEM;

	ret = codec_vp9_alloc_workspace(sess, vp9);
	if (ret)
		goto free_vp9;

	codec_vp9_setup_workspace(sess, vp9);
	amvdec_write_dos_bits(core, HEVC_STREAM_CONTROL, BIT(0));
	/* stream_fifo_hole */
	if (core->platform->revision >= VDEC_REVISION_G12A)
		amvdec_write_dos_bits(core, HEVC_STREAM_FIFO_CTL, BIT(29));

	val = amvdec_read_dos(core, HEVC_PARSER_INT_CONTROL) &
	      PARSER_INT_CTL_INIT_MASK;
	val |= PARSER_INT_CTL_DEFAULT;
	amvdec_write_dos(core, HEVC_PARSER_INT_CONTROL, val);
	amvdec_write_dos_bits(core, HEVC_SHIFT_STATUS, BIT(0));
	amvdec_write_dos(core, HEVC_SHIFT_CONTROL, SHIFT_CTRL_VP9_DEFAULT);
	amvdec_write_dos(core, HEVC_CABAC_CONTROL, BIT(0));
	amvdec_write_dos(core, HEVC_PARSER_CORE_CONTROL, BIT(0));
	amvdec_write_dos(core, HEVC_SHIFT_STARTCODE, 0x00000001);

	amvdec_write_dos(core, VP9_DEC_STATUS_REG, 0);

	amvdec_write_dos(core, HEVC_PARSER_CMD_WRITE, BIT(16));
	for (i = 0; i < ARRAY_SIZE(vdec_hevc_parser_cmd); ++i)
		amvdec_write_dos(core, HEVC_PARSER_CMD_WRITE,
				 vdec_hevc_parser_cmd[i]);

	amvdec_write_dos(core, HEVC_PARSER_CMD_SKIP_0, PARSER_CMD_SKIP_CFG_0);
	amvdec_write_dos(core, HEVC_PARSER_CMD_SKIP_1, PARSER_CMD_SKIP_CFG_1);
	amvdec_write_dos(core, HEVC_PARSER_CMD_SKIP_2, PARSER_CMD_SKIP_CFG_2);
	amvdec_write_dos(core, HEVC_PARSER_IF_CONTROL,
			 PARSER_IF_CTL_DEFAULT);

	amvdec_write_dos(core, HEVCD_IPP_TOP_CNTL, BIT(0));
	amvdec_write_dos(core, HEVCD_IPP_TOP_CNTL, BIT(1));

	amvdec_write_dos(core, VP9_WAIT_FLAG, 1);

	/* clear mailbox interrupt */
	amvdec_write_dos(core, HEVC_ASSIST_MBOX1_CLR_REG, 1);
	/* enable mailbox interrupt */
	amvdec_write_dos(core, HEVC_ASSIST_MBOX1_MASK, 1);
	/* disable PSCALE for hardware sharing */
	amvdec_write_dos(core, HEVC_PSCALE_CTRL, 0);
	/* Let the uCode do all the parsing */
	amvdec_write_dos(core, NAL_SEARCH_CTL, NAL_SEARCH_CTL_VP9);

	amvdec_write_dos(core, DECODE_STOP_POS, 0);
	amvdec_write_dos(core, VP9_DECODE_MODE, DECODE_MODE_SINGLE);

	pr_debug("decode_count: %u; decode_size: %u\n",
		 amvdec_read_dos(core, HEVC_DECODE_COUNT),
		 amvdec_read_dos(core, HEVC_DECODE_SIZE));

	vp9_loop_filter_init(core, vp9);

	INIT_LIST_HEAD(&vp9->ref_frames_list);
	mutex_init(&vp9->lock);
	init_waitqueue_head(&vp9->cur_frame_done_wq);
	memset(&vp9->ref_frame_map, -1, sizeof(vp9->ref_frame_map));
	memset(&vp9->next_ref_frame_map, -1, sizeof(vp9->next_ref_frame_map));
	memset(&vp9->prev_ref_frame_map, -1, sizeof(vp9->prev_ref_frame_map));
	for (i = 0; i < REFS_PER_FRAME; ++i)
		vp9->frame_refs[i] = NULL;
	vp9->sess = sess;
	atomic_set(&vp9->arm_deferred, 0);
	INIT_WORK(&vp9->deferred_arm_work, codec_vp9_deferred_arm_work);
	sess->priv = vp9;

	return 0;

free_vp9:
	kfree(vp9);
	return ret;
}

static int codec_vp9_stop(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_vp9 *vp9 = sess->priv;

	/* No deferred arm may survive stop: it would touch a dead
	 * session's engine from the QBUF wake.
	 */
	cancel_work_sync(&vp9->deferred_arm_work);
	atomic_set(&vp9->arm_deferred, 0);

	mutex_lock(&vp9->lock);
	if (vp9->workspace_vaddr) {
		dma_free_coherent(core->dev, vp9->workspace_size,
				  vp9->workspace_vaddr,
				  vp9->workspace_paddr);
		vp9->workspace_vaddr = NULL;
	}

	codec_hevc_free_fbc_buffers(sess, &vp9->common);
	codec_hevc_free_mmu_headers(sess, &vp9->common);
	mutex_unlock(&vp9->lock);

	return 0;
}

/*
 * Program two reference frames into the motion compensation reference cache
 * controller
 */
static void codec_vp9_set_mcrcc(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_vp9 *vp9 = sess->priv;
	u32 val;

	/* Reset mcrcc */
	amvdec_write_dos(core, HEVCD_MCRCC_CTL1, MCRCC_CTL1_RESET);
	/* Disable on I-frame */
	if (vp9->cur_frame->type == KEY_FRAME || vp9->cur_frame->intra_only) {
		amvdec_write_dos(core, HEVCD_MCRCC_CTL1, MCRCC_CTL1_DISABLE);
		return;
	}

	/*
	 * Read LAST, GOLDEN, ALTREF canvas IDs from the ANC table.
	 * Assign the two most useful refs to the two MCRCC cache lines.
	 * If GOLDEN == LAST (same buffer), use ALTREF for the second line
	 * to avoid wasting a cache line on a duplicate. (BSP optimization)
	 */
	{
		u32 last_c, golden_c, altref_c;

		amvdec_write_dos(core, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR,
				 CANVAS_ACCCONFIG_READ_AUTO);
		last_c = amvdec_read_dos(core, HEVCD_MPP_ANC_CANVAS_DATA_ADDR) & 0xffff;
		golden_c = amvdec_read_dos(core, HEVCD_MPP_ANC_CANVAS_DATA_ADDR) & 0xffff;
		altref_c = amvdec_read_dos(core, HEVCD_MPP_ANC_CANVAS_DATA_ADDR) & 0xffff;

		amvdec_write_dos(core, HEVCD_MCRCC_CTL2,
				 last_c | (last_c << 16));

		if (golden_c != last_c)
			val = golden_c;
		else if (altref_c != last_c)
			val = altref_c;
		else
			val = last_c;
		amvdec_write_dos(core, HEVCD_MCRCC_CTL3,
				 val | (val << 16));
	}
	/* Enable mcrcc progressive-mode */
	amvdec_write_dos(core, HEVCD_MCRCC_CTL1, MCRCC_CTL1_ENABLE_PROGRESSIVE);
}

static void codec_vp9_set_sao(struct amvdec_session *sess,
			      struct vb2_buffer *vb)
{
	struct amvdec_core *core = sess->core;
	struct codec_vp9 *vp9 = sess->priv;

	dma_addr_t buf_y_paddr;
	dma_addr_t buf_u_v_paddr;
	u32 val;

	if (codec_hevc_use_downsample(sess->pixfmt_cap, vp9->is_10bit))
		buf_y_paddr =
			vp9->common.fbc_buffer_paddr[vb->index];
	else
		buf_y_paddr =
		       vb2_dma_contig_plane_dma_addr(vb, 0);

	if (codec_hevc_use_fbc(sess->pixfmt_cap, vp9->is_10bit)) {
		/* Clear FBC_body_uncouple(17) + mem_saving(9) for 10-bit FBC */
		val = amvdec_read_dos(core, HEVC_SAO_CTRL5) &
		      ~SAO_CTRL5_10BIT_FBC_CLEAR_MASK;
		amvdec_write_dos(core, HEVC_SAO_CTRL5, val);
		amvdec_write_dos(core, HEVC_CM_BODY_START_ADDR, buf_y_paddr);
	} else if (codec_hevc_use_mmu(core->platform->revision,
				       sess->pixfmt_cap, vp9->is_10bit)) {
		/*
		 * 8-bit MMU: set bit 9 for 8-bit compression mode.
		 * BSP sets bit 9 = 1 for 8-bit, 0 for 10-bit.
		 */
		amvdec_write_dos_bits(core, HEVC_SAO_CTRL5,
				      SAO_CTRL5_MEM_SAVING_MODE);
	}

	if (codec_hevc_is_nv12(sess->pixfmt_cap)) {
		buf_y_paddr =
		       vb2_dma_contig_plane_dma_addr(vb, 0);
		if (vb->num_planes > 1)
			buf_u_v_paddr =
			       vb2_dma_contig_plane_dma_addr(vb, 1);
		else
			buf_u_v_paddr =
			       buf_y_paddr + amvdec_get_output_size(sess);
		/*
		 * Single-buffer NV12M import (Android CCodec / v4l2_codec2):
		 * both MPLANE planes are the same gralloc dmabuf with
		 * data_offset=0, so plane 1 resolves to the Y address and the
		 * SAO double-write puts chroma over the top rows of luma
		 * (measured on GXL: luma rows 0..h/2-1 byte-identical to the
		 * software decoder's chroma plane, ~9.5 dB).  Place chroma
		 * where the client's allocator put it: right after the CODED
		 * luma plane it was told about in G_FMT, bytesperline x
		 * coded height = ALIGN(w, 64) x ALIGN(h, 32), with no further
		 * rounding.  Measured at 1280x720 (coded 1280x736): chroma at
		 * w64 x h came out 16 chroma rows early, and at
		 * amvdec_get_output_size() -- which also rounds to 64 KiB, so
		 * 983040 instead of 942080 -- 32 rows late.
		 */
		if (vb->num_planes > 1 && buf_u_v_paddr == buf_y_paddr)
			buf_u_v_paddr = buf_y_paddr +
				(u64)ALIGN(sess->width, 64) *
				ALIGN(sess->height, 32);
		amvdec_write_dos(core, HEVC_SAO_Y_START_ADDR, buf_y_paddr);
		amvdec_write_dos(core, HEVC_SAO_C_START_ADDR, buf_u_v_paddr);
		amvdec_write_dos(core, HEVC_SAO_Y_WPTR, buf_y_paddr);
		amvdec_write_dos(core, HEVC_SAO_C_WPTR, buf_u_v_paddr);
	}

	if (codec_hevc_use_mmu(core->platform->revision, sess->pixfmt_cap,
			       vp9->is_10bit)) {
		amvdec_write_dos(core, HEVC_CM_HEADER_START_ADDR,
				 vp9->common.mmu_header_paddr[vb->index]);
		/* use HEVC_CM_HEADER_START_ADDR for MMU header pointer */
		amvdec_write_dos_bits(core, HEVC_SAO_CTRL5,
				      SAO_CTRL5_USE_CM_HEADER);
	}

	amvdec_write_dos(core, HEVC_SAO_Y_LENGTH,
			 amvdec_get_output_size(sess));
	amvdec_write_dos(core, HEVC_SAO_C_LENGTH,
			 (amvdec_get_output_size(sess) / 2));

	if (core->platform->revision >= VDEC_REVISION_G12A) {
		/* Clear write-mode and pipeline bits before reconfiguring */
		amvdec_clear_dos_bits(core, HEVC_DBLK_CFGB,
				      DBLK_CFGB_PIPELINE_MODE | BIT(5) |
				      DBLK_CFGB_COMPRESSED_WRITE |
				      DBLK_CFGB_UNCOMP_WRITE);
		/* enable first, compressed write (FBC or MMU) */
		if (codec_hevc_use_fbc(sess->pixfmt_cap, vp9->is_10bit) ||
		    codec_hevc_use_mmu(core->platform->revision,
				       sess->pixfmt_cap, vp9->is_10bit))
			amvdec_write_dos_bits(core, HEVC_DBLK_CFGB,
					      DBLK_CFGB_COMPRESSED_WRITE);

		/* enable second, uncompressed write */
		if (codec_hevc_is_nv12(sess->pixfmt_cap))
			amvdec_write_dos_bits(core, HEVC_DBLK_CFGB,
					      DBLK_CFGB_UNCOMP_WRITE);

		/* dblk pipeline mode=1 for performance */
		if (sess->width >= 1280)
			amvdec_write_dos_bits(core, HEVC_DBLK_CFGB,
					      DBLK_CFGB_PIPELINE_MODE);

		pr_debug("HEVC_DBLK_CFGB: %08X\n",
			 amvdec_read_dos(core, HEVC_DBLK_CFGB));
	}

	val = amvdec_read_dos(core, HEVC_SAO_CTRL1) & ~SAO_CTRL1_ENDIAN_MASK;
	val |= SAO_CTRL1_LINE_ALIGN_64;
	val |= SAO_CTRL1_BYTE_SWAP_NV12;
	if (core->platform->revision < VDEC_REVISION_G12A)
		val &= ~0x3; /* clear GXL-specific SAO mode bits [1:0] */
	/*
	 * Disable cm (compressed mode) only when neither FBC nor MMU is
	 * active.  With MMU, reference frames go through the MMU page
	 * table and cm must stay enabled -- even on GXL/GXBB.
	 */
	if (!codec_hevc_use_fbc(sess->pixfmt_cap, vp9->is_10bit) &&
	    !codec_hevc_use_mmu(core->platform->revision,
				sess->pixfmt_cap, vp9->is_10bit))
		val |= SAO_CTRL1_DISABLE_CM;

	amvdec_write_dos(core, HEVC_SAO_CTRL1, val);
	pr_debug("HEVC_SAO_CTRL1: %08X\n", val);

	/* no downscale for NV12 */
	val = amvdec_read_dos(core, HEVC_SAO_CTRL5) & ~SAO_CTRL5_DOWNSCALE_MASK;
	amvdec_write_dos(core, HEVC_SAO_CTRL5, val);

	val = amvdec_read_dos(core, HEVCD_IPP_AXIIF_CONFIG);
	val &= ~(IPP_AXIIF_CLEAR_MASK | IPP_AXIIF_UV_SWAP);
	val |= IPP_AXIIF_ENDIAN_NV12;    /* Little-endian per 2-byte swap */
	val |= IPP_AXIIF_LINE_ALIGN_64;  /* LineAlignment = 64 bytes */
	if (codec_hevc_is_nv21(sess->pixfmt_cap))
		val |= IPP_AXIIF_UV_SWAP;
	amvdec_write_dos(core, HEVCD_IPP_AXIIF_CONFIG, val);
}

static dma_addr_t codec_vp9_get_frame_mv_paddr(struct codec_vp9 *vp9,
					       struct vp9_frame *frame)
{
	return vp9->workspace_paddr + MPRED_MV_OFFSET +
	       (frame->index * vp9->mpred_mv_buf_size);
}

static void codec_vp9_set_mpred_mv(struct amvdec_core *core,
				   struct codec_vp9 *vp9)
{
	int mpred_mv_rd_end_addr;
	int use_prev_frame_mvs = vp9->prev_frame &&
				 vp9->prev_frame->width ==
					vp9->cur_frame->width &&
				 vp9->prev_frame->height ==
					vp9->cur_frame->height &&
				 !vp9->prev_frame->intra_only &&
				 vp9->prev_frame->show &&
				 vp9->prev_frame->type != KEY_FRAME;

	amvdec_write_dos(core, HEVC_MPRED_CTRL3, MPRED_CTRL3_VP9_DEFAULT);
	amvdec_write_dos(core, HEVC_MPRED_ABV_START_ADDR,
			 vp9->workspace_paddr + MPRED_ABV_OFFSET);

	amvdec_clear_dos_bits(core, HEVC_MPRED_CTRL4, BIT(6));
	if (use_prev_frame_mvs)
		amvdec_write_dos_bits(core, HEVC_MPRED_CTRL4, BIT(6));

	amvdec_write_dos(core, HEVC_MPRED_MV_WR_START_ADDR,
			 codec_vp9_get_frame_mv_paddr(vp9, vp9->cur_frame));
	amvdec_write_dos(core, HEVC_MPRED_MV_WPTR,
			 codec_vp9_get_frame_mv_paddr(vp9, vp9->cur_frame));

	if (vp9->prev_frame) {
		amvdec_write_dos(core, HEVC_MPRED_MV_RD_START_ADDR,
				 codec_vp9_get_frame_mv_paddr(vp9,
							vp9->prev_frame));
		amvdec_write_dos(core, HEVC_MPRED_MV_RPTR,
				 codec_vp9_get_frame_mv_paddr(vp9,
							vp9->prev_frame));

		mpred_mv_rd_end_addr =
			codec_vp9_get_frame_mv_paddr(vp9, vp9->prev_frame) +
			(vp9->lcu_total * MV_MEM_UNIT);
		amvdec_write_dos(core, HEVC_MPRED_MV_RD_END_ADDR,
				 mpred_mv_rd_end_addr);
	}
}

static void codec_vp9_update_next_ref(struct codec_vp9 *vp9)
{
	union rpm_param *param = &vp9->rpm_param;
	u32 buf_idx = vp9->cur_frame->index;
	int ref_index = 0;
	int refresh_frame_flags;
	int mask;

	refresh_frame_flags = vp9->cur_frame->type == KEY_FRAME ?
				0xff : param->p.refresh_frame_flags;

	for (mask = refresh_frame_flags; mask; mask >>= 1) {
		pr_debug("mask=%08X; ref_index=%d\n", mask, ref_index);
		if (mask & 1)
			vp9->next_ref_frame_map[ref_index] = buf_idx;
		else
			vp9->next_ref_frame_map[ref_index] =
				vp9->ref_frame_map[ref_index];

		++ref_index;
	}

	for (; ref_index < REF_FRAMES; ++ref_index)
		vp9->next_ref_frame_map[ref_index] =
			vp9->ref_frame_map[ref_index];
}


static void codec_vp9_update_ref(struct codec_vp9 *vp9)
{
	union rpm_param *param = &vp9->rpm_param;
	int ref_index = 0;
	int mask;
	int refresh_frame_flags;

	if (!vp9->cur_frame)
		return;

	/* Save current ref_frame_map before overwriting.  is_ref() checks
	 * this to prevent premature frame deletion: a buffer might not be
	 * in the new ref_frame_map or next_ref_frame_map but could still
	 * be referenced by a future frame's firmware-provided ref_info.
	 */
	memcpy(vp9->prev_ref_frame_map, vp9->ref_frame_map,
	       sizeof(vp9->prev_ref_frame_map));

	refresh_frame_flags = vp9->cur_frame->type == KEY_FRAME ?
				0xff : param->p.refresh_frame_flags;

	for (mask = refresh_frame_flags; mask; mask >>= 1) {
		vp9->ref_frame_map[ref_index] =
			vp9->next_ref_frame_map[ref_index];
		++ref_index;
	}

	if (param->p.show_existing_frame)
		return;

	for (; ref_index < REF_FRAMES; ++ref_index)
		vp9->ref_frame_map[ref_index] =
			vp9->next_ref_frame_map[ref_index];
}

static struct vp9_frame *codec_vp9_get_frame_by_idx(struct codec_vp9 *vp9,
						    int idx)
{
	struct vp9_frame *frame;

	list_for_each_entry(frame, &vp9->ref_frames_list, list) {
		if (frame->index == idx)
			return frame;
	}

	return NULL;
}

static void codec_vp9_sync_ref(struct codec_vp9 *vp9)
{
	union rpm_param *param = &vp9->rpm_param;
	int i;

	for (i = 0; i < REFS_PER_FRAME; ++i) {
		const int ref = (param->p.ref_info >>
				 (((REFS_PER_FRAME - i - 1) * 4) + 1)) & 0x7;
		const int idx = vp9->ref_frame_map[ref];

		vp9->frame_refs[i] = codec_vp9_get_frame_by_idx(vp9, idx);
		if (!vp9->frame_refs[i])
			pr_warn("%s: couldn't find VP9 ref %d\n", __func__,
				idx);
	}
}

static void codec_vp9_set_refs(struct amvdec_session *sess,
			       struct codec_vp9 *vp9)
{
	struct amvdec_core *core = sess->core;
	int i;

	for (i = 0; i < REFS_PER_FRAME; ++i) {
		struct vp9_frame *frame = vp9->frame_refs[i];
		int id_y;
		int id_u_v;

		if (!frame)
			continue;

		if (codec_hevc_use_fbc(sess->pixfmt_cap, vp9->is_10bit) ||
		    codec_hevc_use_mmu(core->platform->revision,
				       sess->pixfmt_cap, vp9->is_10bit)) {
			id_y = frame->index;
			id_u_v = id_y;
		} else {
			id_y = frame->index * 2;
			id_u_v = id_y + 1;
		}

		amvdec_write_dos(core, HEVCD_MPP_ANC_CANVAS_DATA_ADDR,
				 (id_u_v << 16) | (id_u_v << 8) | id_y);
	}
}

static void codec_vp9_set_mc(struct amvdec_session *sess,
			     struct codec_vp9 *vp9)
{
	struct amvdec_core *core = sess->core;
	u32 scale = 0;
	u32 sz, w64, h32;
	int i;

	/* First pass: write ref canvas at index 0 with auto-increment */
	amvdec_write_dos(core, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, 1);
	codec_vp9_set_refs(sess, vp9);
	/* Second pass: write ref canvas at index 16 with auto-increment */
	amvdec_write_dos(core, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR,
			 CANVAS_ACCCONFIG_OFFSET16);
	codec_vp9_set_refs(sess, vp9);

	amvdec_write_dos(core, VP9D_MPP_REFINFO_TBL_ACCCONFIG, BIT(2));
	for (i = 0; i < REFS_PER_FRAME; ++i) {
		if (!vp9->frame_refs[i])
			continue;

		if (vp9->frame_refs[i]->width != vp9->width ||
		    vp9->frame_refs[i]->height != vp9->height)
			scale = 1;

		w64 = ALIGN(vp9->frame_refs[i]->width, 64) / 64;
		h32 = ALIGN(vp9->frame_refs[i]->height, 32) / 32;
		sz = (vp9->is_10bit ? 4096 : 3200) * w64 * h32;

		amvdec_write_dos(core, VP9D_MPP_REFINFO_DATA,
				 vp9->frame_refs[i]->width);
		amvdec_write_dos(core, VP9D_MPP_REFINFO_DATA,
				 vp9->frame_refs[i]->height);
		amvdec_write_dos(core, VP9D_MPP_REFINFO_DATA,
				 (vp9->frame_refs[i]->width << 14) /
				 vp9->width);
		amvdec_write_dos(core, VP9D_MPP_REFINFO_DATA,
				 (vp9->frame_refs[i]->height << 14) /
				 vp9->height);
		if (codec_hevc_use_mmu(core->platform->revision,
				       sess->pixfmt_cap, vp9->is_10bit))
			amvdec_write_dos(core, VP9D_MPP_REFINFO_DATA, 0);
		else
			amvdec_write_dos(core, VP9D_MPP_REFINFO_DATA,
					 sz >> 5);
	}

	amvdec_write_dos(core, VP9D_MPP_REF_SCALE_ENBL, scale);
}

static struct vp9_frame *codec_vp9_get_new_frame(struct amvdec_session *sess)
{
	struct codec_vp9 *vp9 = sess->priv;
	union rpm_param *param = &vp9->rpm_param;
	struct vb2_v4l2_buffer *vbuf;
	struct vp9_frame *new_frame;

	new_frame = kzalloc(sizeof(*new_frame), GFP_KERNEL);
	if (!new_frame)
		return NULL;

	vbuf = v4l2_m2m_dst_buf_remove(sess->m2m_ctx);
	if (!vbuf) {
		dev_err(sess->core->dev, "VP9: No dst buffer available (frames_num=%d)\n",
			vp9->frames_num);
		kfree(new_frame);
		return NULL;
	}

	while (codec_vp9_get_frame_by_idx(vp9, vbuf->vb2_buf.index)) {
		struct vb2_v4l2_buffer *old_vbuf = vbuf;

		vbuf = v4l2_m2m_dst_buf_remove(sess->m2m_ctx);
		v4l2_m2m_buf_queue(sess->m2m_ctx, old_vbuf);
		if (!vbuf) {
			dev_err(sess->core->dev, "No dst buffer available\n");
			kfree(new_frame);
			return NULL;
		}
	}

	new_frame->vbuf = vbuf;
	new_frame->index = vbuf->vb2_buf.index;
	new_frame->intra_only = param->p.intra_only;
	new_frame->show = param->p.show_frame;
	new_frame->type = param->p.frame_type;
	new_frame->width = vp9->width;
	new_frame->height = vp9->height;
	list_add_tail(&new_frame->list, &vp9->ref_frames_list);
	vp9->frames_num++;

	return new_frame;
}

static void codec_vp9_show_existing_frame(struct codec_vp9 *vp9)
{
	union rpm_param *param = &vp9->rpm_param;

	if (!param->p.show_existing_frame)
		return;

	pr_debug("showing frame %u\n", param->p.frame_to_show_idx);
}

static void codec_vp9_rm_noshow_frame(struct amvdec_session *sess)
{
	struct codec_vp9 *vp9 = sess->priv;
	struct vp9_frame *tmp;

	list_for_each_entry(tmp, &vp9->ref_frames_list, list) {
		if (tmp->show)
			continue;

		pr_debug("rm noshow: %u\n", tmp->index);
		v4l2_m2m_buf_queue(sess->m2m_ctx, tmp->vbuf);
		list_del(&tmp->list);
		kfree(tmp);
		vp9->frames_num--;
		return;
	}
}

static void codec_vp9_process_frame(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_vp9 *vp9 = sess->priv;
	union rpm_param *param = &vp9->rpm_param;
	int intra_only;

	dev_dbg(core->dev_dec, "VP9 process_frame: show=%u type=%u %ux%u\n",
		 param->p.show_frame, param->p.frame_type,
		 param->p.width, param->p.height);

	if (!param->p.show_frame)
		codec_vp9_rm_noshow_frame(sess);

	/*
	 * Arm-on-available: never submit a frame to the engine while no
	 * CAPTURE buffer can receive it.  No frame state is rotated --
	 * rpm still holds the parsed picture -- and the pending frame is
	 * armed by the next CAPTURE QBUF (vdec_vb2_buf_queue ->
	 * codec_vp9_arm_pending -> deferred_arm_work).  This is not a
	 * failure: amvdec_abort() stays reserved for broken streams, not
	 * an empty pool, and because the engine was never started the
	 * watchdog has nothing to re-kick -- the livelock shape that sank
	 * the v8 DEFER cannot recur here.
	 */
	if (!v4l2_m2m_num_dst_bufs_ready(sess->m2m_ctx)) {
		if (!atomic_xchg(&vp9->arm_deferred, 1))
			dev_info(core->dev_dec,
				 "VP9: dst pool empty; arming deferred (frames_num=%u cap_qbufs=%u)\n",
				 vp9->frames_num, sess->cap_qbufs);
		return;
	}
	atomic_set(&vp9->arm_deferred, 0);

	vp9->cur_frame = codec_vp9_get_new_frame(sess);
	if (!vp9->cur_frame) {
		/*
		 * The pop lost a race (resume runs trylock-locked while the
		 * parse ISR holds vp9->lock, so both can pass the guard and
		 * only one wins the buffer) or the pool just drained. Nothing
		 * was armed and the engine was NOT touched for this frame --
		 * returning here is exactly the arm-on-available wait: the
		 * winner decodes, its ISR re-enters process_frame, and a CAPTURE
		 * QBUF wake covers the idle case. The k24 amvdec_abort() here
		 * was the zero-frame cliff: it killed the session (and the
		 * in-flight previous frame) over a benign pop race.
		 */
		if (!atomic_xchg(&vp9->arm_deferred, 1))
			dev_info(core->dev_dec,
				 "VP9: dst pop race; arming deferred (frames_num=%u cap_qbufs=%u)\n",
				 vp9->frames_num, sess->cap_qbufs);
		return;
	}

	dev_dbg(core->dev_dec,
		"VP9 frame idx=%d type=%s show=%u show_exist=%u intra_only=%u %ux%u\n",
		 vp9->cur_frame->index,
		 param->p.frame_type == KEY_FRAME ? "KEY" : "INTER",
		 param->p.show_frame, param->p.show_existing_frame,
		 param->p.intra_only, vp9->width, vp9->height);

	if (param->p.frame_type != KEY_FRAME) {
		codec_vp9_sync_ref(vp9);
		/*
		 * KI-049: periodic VBH ring reset for inter-frames (GXL/pre-G12A only).
		 * On GXL (< VDEC_REVISION_G12A) the driver must manage the VBH ring;
		 * on G12A+ (G12B, SM1) the firmware manages it internally -- driver
		 * writes disrupt the firmware's state and advance corruption onset.
		 * The ring holds MMU_VBH_SIZE/2 / sizeof(u32) = 2560 LCU entries.
		 * Track accumulated inter-frame LCUs and reset before overflow.
		 */
		if (core->platform->revision < VDEC_REVISION_G12A &&
		    codec_hevc_use_mmu(core->platform->revision, sess->pixfmt_cap,
				       vp9->is_10bit)) {
			vp9->inter_lcu_count += vp9->lcu_total;
			if (vp9->inter_lcu_count >= MMU_VBH_SIZE / 2 / sizeof(u32)) {
				vp9->inter_lcu_count = 0;
				amvdec_write_dos(core, HEVC_SAO_MMU_VH0_ADDR,
						 vp9->workspace_paddr + MMU_VBH_OFFSET);
				amvdec_write_dos(core, HEVC_SAO_MMU_VH1_ADDR,
						 vp9->workspace_paddr + MMU_VBH_OFFSET +
						 (MMU_VBH_SIZE / 2));
			}
		}
	} else if (core->platform->revision < VDEC_REVISION_G12A &&
		   codec_hevc_use_mmu(core->platform->revision, sess->pixfmt_cap,
				      vp9->is_10bit)) {
		/*
		 * KI-049: reset VBH ring head on KEY_FRAME (GXL/pre-G12A only).
		 * On GXL the firmware does not reset the ring pointer on keyframe;
		 * the ring wraps and corrupts reference frame descriptors.
		 * G12A+ firmware manages VBH internally -- do not interfere.
		 */
		vp9->inter_lcu_count = 0;
		amvdec_write_dos(core, HEVC_SAO_MMU_VH0_ADDR,
				 vp9->workspace_paddr + MMU_VBH_OFFSET);
		amvdec_write_dos(core, HEVC_SAO_MMU_VH1_ADDR,
				 vp9->workspace_paddr + MMU_VBH_OFFSET + (MMU_VBH_SIZE / 2));
	}
	codec_vp9_update_next_ref(vp9);
	codec_vp9_show_existing_frame(vp9);

	if (codec_hevc_use_mmu(core->platform->revision, sess->pixfmt_cap,
			       vp9->is_10bit))
		codec_hevc_fill_mmu_map(sess, &vp9->common,
					vp9->cur_frame->vbuf->vb2_buf.index,
					&vp9->cur_frame->vbuf->vb2_buf);

	intra_only = param->p.show_frame ? 0 : param->p.intra_only;

	/* clear mpred (for keyframe only) */
	if (param->p.frame_type != KEY_FRAME && !intra_only) {
		codec_vp9_set_mc(sess, vp9);
		codec_vp9_set_mpred_mv(core, vp9);
	} else {
		amvdec_clear_dos_bits(core, HEVC_MPRED_CTRL4, BIT(6));
	}

	amvdec_write_dos(core, HEVC_PARSER_PICTURE_SIZE,
			 (vp9->height << 16) | vp9->width);
	codec_vp9_set_mcrcc(sess);
	codec_vp9_set_sao(sess, &vp9->cur_frame->vbuf->vb2_buf);

	vp9_loop_filter_frame_init(core, &vp9->seg_4lf,
				   &vp9->lfi, &vp9->lf,
				   vp9->default_filt_lvl);

	/* ask uCode to start decoding */
	amvdec_write_dos(core, VP9_DEC_STATUS_REG, VP9_10B_DECODE_SLICE);
}

static void codec_vp9_process_lf(struct codec_vp9 *vp9)
{
	union rpm_param *param = &vp9->rpm_param;
	int i;

	vp9->lf.mode_ref_delta_enabled = param->p.mode_ref_delta_enabled;
	vp9->lf.sharpness_level = param->p.sharpness_level;
	vp9->default_filt_lvl = param->p.filter_level;
	vp9->seg_4lf.enabled = param->p.seg_enabled;
	vp9->seg_4lf.abs_delta = param->p.seg_abs_delta;

	for (i = 0; i < 4; i++)
		vp9->lf.ref_deltas[i] = param->p.ref_deltas[i];

	for (i = 0; i < 2; i++)
		vp9->lf.mode_deltas[i] = param->p.mode_deltas[i];

	for (i = 0; i < MAX_SEGMENTS; i++)
		vp9->seg_4lf.feature_mask[i] =
			(param->p.seg_lf_info[i] & SEG_LF_INFO_ENABLED) ?
				(1 << SEG_LVL_ALT_LF) : 0;

	for (i = 0; i < MAX_SEGMENTS; i++)
		vp9->seg_4lf.feature_data[i][SEG_LVL_ALT_LF] =
			(param->p.seg_lf_info[i] & SEG_LF_INFO_SIGN) ?
				-(param->p.seg_lf_info[i] &
				  SEG_LF_INFO_DATA_MASK)
				: (param->p.seg_lf_info[i] &
				   SEG_LF_INFO_DATA_MASK);
}

static void codec_vp9_resume(struct amvdec_session *sess)
{
	struct codec_vp9 *vp9 = sess->priv;
	bool locked;

	/*
	 * resume() can be called from two contexts:
	 * 1. vdec_start_streaming() on STREAMON(CAPTURE) -- lock not held
	 * 2. amvdec_src_change() fast-resume path, called from within
	 *    codec_vp9_threaded_isr() which already holds vp9->lock
	 * Use trylock to avoid recursive deadlock in case 2.
	 */
	locked = mutex_trylock(&vp9->lock);

	/*
	 * KI26 (fixed) / KI-169: a capture-only DRC stop (session kept
	 * alive) skips codec teardown, so vp9_frame entries from the
	 * pre-DRC resolution survive.  Their dst buffers were removed from
	 * the m2m list by get_new_frame, so the v4l2_m2m_dst_buf_remove
	 * loop in vdec_stop_streaming never recycled them -- the plain
	 * kfree here lost those buffers permanently (one frame after DRC,
	 * then "No dst buffer available").  Return them first, mirroring
	 * the teardown loop: undelivered show frames are delivered,
	 * non-show frames are re-queued.
	 */
	{
		struct vp9_frame *tmp, *n;

		list_for_each_entry_safe(tmp, n, &vp9->ref_frames_list, list) {
			if (!tmp->done) {
				if (tmp->show)
					amvdec_dst_buf_done(sess, tmp->vbuf,
							    V4L2_FIELD_NONE);
				else
					v4l2_m2m_buf_queue(sess->m2m_ctx,
							   tmp->vbuf);
				vp9->frames_num--;
			}

			list_del(&tmp->list);
			kfree(tmp);
		}
	}
	vp9->cur_frame = NULL;
	vp9->prev_frame = NULL;
	vp9->frames_num = 0;
	memset(vp9->ref_frame_map, -1, sizeof(vp9->ref_frame_map));
	memset(vp9->next_ref_frame_map, -1, sizeof(vp9->next_ref_frame_map));
	memset(vp9->prev_ref_frame_map, -1, sizeof(vp9->prev_ref_frame_map));

	if (codec_hevc_setup_buffers(sess, &vp9->common, vp9->is_10bit)) {
		if (locked)
			mutex_unlock(&vp9->lock);
		amvdec_abort(sess);
		return;
	}

	if (codec_vp9_grow_workspace(sess, vp9)) {
		if (locked)
			mutex_unlock(&vp9->lock);
		amvdec_abort(sess);
		return;
	}

	codec_vp9_setup_workspace(sess, vp9);
	codec_hevc_setup_decode_head(sess, vp9->is_10bit);
	codec_vp9_process_lf(vp9);

	/*
	 * Arm-on-available: with the CAPTURE queue not fed yet (some
	 * clients STREAMON before re-queuing after a source change),
	 * process_frame() below refuses to submit, marks the frame
	 * pending, and the next CAPTURE QBUF
	 * (vdec_vb2_buf_queue -> codec_vp9_arm_pending) arms it.  With
	 * a compliant reinit (QBUF before STREAMON) the pool is
	 * non-empty here and this call decodes directly.
	 */
	codec_vp9_process_frame(sess);

	if (locked)
		mutex_unlock(&vp9->lock);
}

/*
 * The RPM section within the workspace contains
 * many information regarding the parsed bitstream
 */
static void codec_vp9_fetch_rpm(struct amvdec_session *sess)
{
	struct codec_vp9 *vp9 = sess->priv;
	u16 *rpm_vaddr = vp9->workspace_vaddr + vp9->rpm_offset;
	int i, j;

	for (i = 0; i < RPM_BUF_SIZE; i += 4)
		for (j = 0; j < 4; j++)
			vp9->rpm_param.l.data[i + j] = rpm_vaddr[i + 3 - j];
}

static int codec_vp9_process_rpm(struct codec_vp9 *vp9)
{
	union rpm_param *param = &vp9->rpm_param;
	int src_changed = 0;
	int is_10bit = 0;
	int pic_width_64 = ALIGN(param->p.width, 64);
	int pic_height_32 = ALIGN(param->p.height, 32);
	int pic_width_lcu  = (pic_width_64 % LCU_SIZE) ?
				pic_width_64 / LCU_SIZE  + 1
				: pic_width_64 / LCU_SIZE;
	int pic_height_lcu = (pic_height_32 % LCU_SIZE) ?
				pic_height_32 / LCU_SIZE + 1
				: pic_height_32 / LCU_SIZE;
	vp9->lcu_total = pic_width_lcu * pic_height_lcu;

	if (param->p.bit_depth == 10)
		is_10bit = 1;

	if (vp9->width != param->p.width || vp9->height != param->p.height ||
	    vp9->is_10bit != is_10bit)
		src_changed = 1;

	vp9->width = param->p.width;
	vp9->height = param->p.height;
	vp9->is_10bit = is_10bit;

	pr_debug("width: %u; height: %u; is_10bit: %d; src_changed: %d\n",
		 vp9->width, vp9->height, is_10bit, src_changed);

	return src_changed;
}

static bool codec_vp9_is_ref(struct codec_vp9 *vp9, struct vp9_frame *frame)
{
	int i;

	for (i = 0; i < REF_FRAMES; ++i) {
		if (vp9->ref_frame_map[i] == frame->index)
			return true;
		/*
		 * Also check next_ref_frame_map: update_next_ref() writes the
		 * reference slots for the NEXT frame during process_frame(),
		 * but show_frame() runs immediately after and must not free a
		 * frame that next_ref_frame_map still points to.  On the next
		 * IRQ, update_ref() copies next_ref_frame_map -> ref_frame_map
		 * and sync_ref() looks up the buffer index; if the frame was
		 * already freed and its VB2 buffer recycled, get_frame_by_idx()
		 * returns NULL -> "couldn't find VP9 ref N" and the frame that
		 * references it is dropped.
		 *
		 * With max_buffers=12 (reduced from 24 for CMA savings), buffer
		 * indices recycle quickly enough to trigger this reliably on
		 * the last frame of a stream (KI-088).
		 */
		if (vp9->next_ref_frame_map[i] == frame->index)
			return true;
		/*
		 * Also check prev_ref_frame_map: the previous ref_frame_map
		 * values (saved in update_ref before overwrite) may still be
		 * needed by frames 2 ISRs ahead.  The firmware's internal DPB
		 * updates asynchronously; the driver's ref maps lag by 1 ISR.
		 * Checking prev ensures a buffer stays alive for at least 1
		 * extra ISR cycle after leaving the current ref maps.
		 */
		if (vp9->prev_ref_frame_map[i] == frame->index)
			return true;
	}

	return false;
}

static void codec_vp9_show_frame(struct amvdec_session *sess)
{
	struct codec_vp9 *vp9 = sess->priv;
	struct vp9_frame *tmp, *n;

	list_for_each_entry_safe(tmp, n, &vp9->ref_frames_list, list) {
		if (!tmp->show || tmp == vp9->cur_frame)
			continue;

		if (!tmp->done) {
			pr_debug("Doning %u\n", tmp->index);
			amvdec_dst_buf_done(sess, tmp->vbuf, V4L2_FIELD_NONE);
			tmp->done = 1;
			vp9->frames_num--;
		}

		if (codec_vp9_is_ref(vp9, tmp) || tmp == vp9->prev_frame)
			continue;

		pr_debug("deleting %d\n", tmp->index);
		list_del(&tmp->list);
		kfree(tmp);
	}
}

static void vp9_tree_merge_probs(unsigned int *prev_prob,
				 unsigned int *cur_prob,
				 int coef_node_start, int tree_left,
				 int tree_right,
				 int tree_i, int node)
{
	int prob_32, prob_res, prob_shift;
	int pre_prob, new_prob;
	int den, m_count, get_prob, factor;

	prob_32 = prev_prob[coef_node_start / 4 * 2];
	prob_res = coef_node_start & 3;
	prob_shift = prob_res * 8;
	pre_prob = (prob_32 >> prob_shift) & 0xff;

	den = tree_left + tree_right;

	if (den == 0) {
		new_prob = pre_prob;
	} else {
		m_count = min(den, MODE_MV_COUNT_SAT);
		get_prob =
			clip_prob(div_r32(((int64_t)tree_left * 256 +
					   (den >> 1)),
					  den));

		/* weighted_prob */
		factor = count_to_update_factor[m_count];
		new_prob = round_power_of_two(pre_prob * (256 - factor) +
					      get_prob * factor, 8);
	}

	cur_prob[coef_node_start / 4 * 2] =
		(cur_prob[coef_node_start / 4 * 2] & (~(0xff << prob_shift))) |
		(new_prob << prob_shift);
}

static void adapt_coef_probs_cxt(unsigned int *prev_prob,
				 unsigned int *cur_prob,
				 unsigned int *count,
				 int update_factor,
				 int cxt_num,
				 int coef_cxt_start,
				 int coef_count_cxt_start)
{
	int prob_32, prob_res, prob_shift;
	int pre_prob, new_prob;
	int num, den, m_count, get_prob, factor;
	int node, coef_node_start;
	int count_sat = 24;
	int cxt;

	for (cxt = 0; cxt < cxt_num; cxt++) {
		const int n0 = count[coef_count_cxt_start];
		const int n1 = count[coef_count_cxt_start + 1];
		const int n2 = count[coef_count_cxt_start + 2];
		const int neob = count[coef_count_cxt_start + 3];
		const int nneob = count[coef_count_cxt_start + 4];
		const unsigned int branch_ct[3][2] = {
			{ neob, nneob },
			{ n0, n1 + n2 },
			{ n1, n2 }
		};

		coef_node_start = coef_cxt_start;
		for (node = 0 ; node < 3 ; node++) {
			prob_32 = prev_prob[coef_node_start / 4 * 2];
			prob_res = coef_node_start & 3;
			prob_shift = prob_res * 8;
			pre_prob = (prob_32 >> prob_shift) & 0xff;

			/* get binary prob */
			num = branch_ct[node][0];
			den = branch_ct[node][0] + branch_ct[node][1];
			m_count = min(den, count_sat);

			get_prob = (den == 0) ?
					128u :
					clip_prob(div_r32(((int64_t)num * 256 +
							  (den >> 1)), den));

			factor = update_factor * m_count / count_sat;
			new_prob =
				round_power_of_two(pre_prob * (256 - factor) +
						   get_prob * factor, 8);

			cur_prob[coef_node_start / 4 * 2] =
				(cur_prob[coef_node_start / 4 * 2] &
				 (~(0xff << prob_shift))) |
				(new_prob << prob_shift);

			coef_node_start += 1;
		}

		coef_cxt_start = coef_cxt_start + 3;
		coef_count_cxt_start = coef_count_cxt_start + 5;
	}
}

static void adapt_coef_probs(int prev_kf, int cur_kf, int pre_fc,
			     unsigned int *prev_prob, unsigned int *cur_prob,
			     unsigned int *count)
{
	int tx_size, coef_tx_size_start, coef_count_tx_size_start;
	int plane, coef_plane_start, coef_count_plane_start;
	int type, coef_type_start, coef_count_type_start;
	int band, coef_band_start, coef_count_band_start;
	int cxt_num;
	int coef_cxt_start, coef_count_cxt_start;
	int node, coef_node_start, coef_count_node_start;

	int tree_i, tree_left, tree_right;
	int mvd_i;

	int update_factor = cur_kf ? 112 : (prev_kf ? 128 : 112);

	int prob_32;
	int prob_res;
	int prob_shift;
	int pre_prob;

	int den;
	int get_prob;
	int m_count;
	int factor;

	int new_prob;

	for (tx_size = 0 ; tx_size < 4 ; tx_size++) {
		coef_tx_size_start = VP9_COEF_START +
				tx_size * 4 * VP9_COEF_SIZE_ONE_SET;
		coef_count_tx_size_start = VP9_COEF_COUNT_START +
				tx_size * 4 * VP9_COEF_COUNT_SIZE_ONE_SET;
		coef_plane_start = coef_tx_size_start;
		coef_count_plane_start = coef_count_tx_size_start;

		for (plane = 0 ; plane < 2 ; plane++) {
			coef_type_start = coef_plane_start;
			coef_count_type_start = coef_count_plane_start;

			for (type = 0 ; type < 2 ; type++) {
				coef_band_start = coef_type_start;
				coef_count_band_start = coef_count_type_start;

				for (band = 0 ; band < 6 ; band++) {
					if (band == 0)
						cxt_num = 3;
					else
						cxt_num = 6;
					coef_cxt_start = coef_band_start;
					coef_count_cxt_start =
						coef_count_band_start;

					adapt_coef_probs_cxt(prev_prob,
							     cur_prob,
							     count,
							     update_factor,
							     cxt_num,
							     coef_cxt_start,
							coef_count_cxt_start);

					if (band == 0) {
						coef_band_start += 10;
						coef_count_band_start += 15;
					} else {
						coef_band_start += 18;
						coef_count_band_start += 30;
					}
				}
				coef_type_start += VP9_COEF_SIZE_ONE_SET;
				coef_count_type_start +=
					VP9_COEF_COUNT_SIZE_ONE_SET;
			}

			coef_plane_start += 2 * VP9_COEF_SIZE_ONE_SET;
			coef_count_plane_start +=
				2 * VP9_COEF_COUNT_SIZE_ONE_SET;
		}
	}

	if (cur_kf == 0) {
		/* mode_mv_merge_probs - merge_intra_inter_prob */
		for (coef_count_node_start = VP9_INTRA_INTER_COUNT_START;
		     coef_count_node_start < (VP9_MV_CLASS0_HP_1_COUNT_START +
					      VP9_MV_CLASS0_HP_1_COUNT_SIZE);
		     coef_count_node_start += 2) {
			if (coef_count_node_start ==
					VP9_INTRA_INTER_COUNT_START)
				coef_node_start = VP9_INTRA_INTER_START;
			else if (coef_count_node_start ==
					VP9_COMP_INTER_COUNT_START)
				coef_node_start = VP9_COMP_INTER_START;
			else if (coef_count_node_start ==
					VP9_TX_MODE_COUNT_START)
				coef_node_start = VP9_TX_MODE_START;
			else if (coef_count_node_start ==
					VP9_SKIP_COUNT_START)
				coef_node_start = VP9_SKIP_START;
			else if (coef_count_node_start ==
					VP9_MV_SIGN_0_COUNT_START)
				coef_node_start = VP9_MV_SIGN_0_START;
			else if (coef_count_node_start ==
					VP9_MV_SIGN_1_COUNT_START)
				coef_node_start = VP9_MV_SIGN_1_START;
			else if (coef_count_node_start ==
					VP9_MV_BITS_0_COUNT_START)
				coef_node_start = VP9_MV_BITS_0_START;
			else if (coef_count_node_start ==
					VP9_MV_BITS_1_COUNT_START)
				coef_node_start = VP9_MV_BITS_1_START;
			else /* node_start == VP9_MV_CLASS0_HP_0_COUNT_START */
				coef_node_start = VP9_MV_CLASS0_HP_0_START;

			den = count[coef_count_node_start] +
			      count[coef_count_node_start + 1];

			prob_32 = prev_prob[coef_node_start / 4 * 2];
			prob_res = coef_node_start & 3;
			prob_shift = prob_res * 8;
			pre_prob = (prob_32 >> prob_shift) & 0xff;

			if (den == 0) {
				new_prob = pre_prob;
			} else {
				m_count = min(den, MODE_MV_COUNT_SAT);
				get_prob =
				clip_prob(div_r32(((int64_t)
					count[coef_count_node_start] * 256 +
					(den >> 1)),
					den));

				/* weighted prob */
				factor = count_to_update_factor[m_count];
				new_prob =
					round_power_of_two(pre_prob *
							   (256 - factor) +
							   get_prob * factor,
							   8);
			}

			cur_prob[coef_node_start / 4 * 2] =
				(cur_prob[coef_node_start / 4 * 2] &
				 (~(0xff << prob_shift))) |
				(new_prob << prob_shift);

			coef_node_start = coef_node_start + 1;
		}

		coef_node_start = VP9_INTER_MODE_START;
		coef_count_node_start = VP9_INTER_MODE_COUNT_START;
		for (tree_i = 0 ; tree_i < 7 ; tree_i++) {
			for (node = 0 ; node < 3 ; node++) {
				unsigned int start = coef_count_node_start;

				switch (node) {
				case 2:
					tree_left = count[start + 1];
					tree_right = count[start + 3];
					break;
				case 1:
					tree_left = count[start + 0];
					tree_right = count[start + 1] +
						     count[start + 3];
					break;
				default:
					tree_left = count[start + 2];
					tree_right = count[start + 0] +
						     count[start + 1] +
						     count[start + 3];
					break;
				}

				vp9_tree_merge_probs(prev_prob, cur_prob,
						     coef_node_start,
						     tree_left, tree_right,
						     tree_i, node);

				coef_node_start = coef_node_start + 1;
			}

			coef_count_node_start = coef_count_node_start + 4;
		}

		coef_node_start = VP9_IF_Y_MODE_START;
		coef_count_node_start = VP9_IF_Y_MODE_COUNT_START;
		for (tree_i = 0 ; tree_i < 14 ; tree_i++) {
			for (node = 0 ; node < 9 ; node++) {
				unsigned int start = coef_count_node_start;

				switch (node) {
				case 8:
					tree_left =
						count[start + D153_PRED];
					tree_right =
						count[start + D207_PRED];
					break;
				case 7:
					tree_left =
						count[start + D63_PRED];
					tree_right =
						count[start + D207_PRED] +
						count[start + D153_PRED];
					break;
				case 6:
					tree_left =
						count[start + D45_PRED];
					tree_right =
						count[start + D207_PRED] +
						count[start + D153_PRED] +
						count[start + D63_PRED];
					break;
				case 5:
					tree_left =
						count[start + D135_PRED];
					tree_right =
						count[start + D117_PRED];
					break;
				case 4:
					tree_left =
						count[start + H_PRED];
					tree_right =
						count[start + D117_PRED] +
						count[start + D135_PRED];
					break;
				case 3:
					tree_left =
						count[start + H_PRED] +
						count[start + D117_PRED] +
						count[start + D135_PRED];
					tree_right =
						count[start + D45_PRED] +
						count[start + D207_PRED] +
						count[start + D153_PRED] +
						count[start + D63_PRED];
					break;
				case 2:
					tree_left =
						count[start + V_PRED];
					tree_right =
						count[start + H_PRED] +
						count[start + D117_PRED] +
						count[start + D135_PRED] +
						count[start + D45_PRED] +
						count[start + D207_PRED] +
						count[start + D153_PRED] +
						count[start + D63_PRED];
					break;
				case 1:
					tree_left =
						count[start + TM_PRED];
					tree_right =
						count[start + V_PRED] +
						count[start + H_PRED] +
						count[start + D117_PRED] +
						count[start + D135_PRED] +
						count[start + D45_PRED] +
						count[start + D207_PRED] +
						count[start + D153_PRED] +
						count[start + D63_PRED];
					break;
				default:
					tree_left =
						count[start + DC_PRED];
					tree_right =
						count[start + TM_PRED] +
						count[start + V_PRED] +
						count[start + H_PRED] +
						count[start + D117_PRED] +
						count[start + D135_PRED] +
						count[start + D45_PRED] +
						count[start + D207_PRED] +
						count[start + D153_PRED] +
						count[start + D63_PRED];
					break;
				}

				vp9_tree_merge_probs(prev_prob, cur_prob,
						     coef_node_start,
						     tree_left, tree_right,
						     tree_i, node);

				coef_node_start = coef_node_start + 1;
			}
			coef_count_node_start = coef_count_node_start + 10;
		}

		coef_node_start = VP9_PARTITION_P_START;
		coef_count_node_start = VP9_PARTITION_P_COUNT_START;
		for (tree_i = 0 ; tree_i < 16 ; tree_i++) {
			for (node = 0 ; node < 3 ; node++) {
				unsigned int start = coef_count_node_start;

				switch (node) {
				case 2:
					tree_left = count[start + 2];
					tree_right = count[start + 3];
					break;
				case 1:
					tree_left = count[start + 1];
					tree_right = count[start + 2] +
						     count[start + 3];
					break;
				default:
					tree_left = count[start + 0];
					tree_right = count[start + 1] +
						     count[start + 2] +
						     count[start + 3];
					break;
				}

				vp9_tree_merge_probs(prev_prob, cur_prob,
						     coef_node_start,
						     tree_left, tree_right,
						     tree_i, node);

				coef_node_start = coef_node_start + 1;
			}

			coef_count_node_start = coef_count_node_start + 4;
		}

		coef_node_start = VP9_INTERP_START;
		coef_count_node_start = VP9_INTERP_COUNT_START;
		for (tree_i = 0 ; tree_i < 4 ; tree_i++) {
			for (node = 0 ; node < 2 ; node++) {
				unsigned int start = coef_count_node_start;

				switch (node) {
				case 1:
					tree_left = count[start + 1];
					tree_right = count[start + 2];
					break;
				default:
					tree_left = count[start + 0];
					tree_right = count[start + 1] +
						     count[start + 2];
					break;
				}

				vp9_tree_merge_probs(prev_prob, cur_prob,
						     coef_node_start,
						     tree_left, tree_right,
						     tree_i, node);

				coef_node_start = coef_node_start + 1;
			}
			coef_count_node_start = coef_count_node_start + 3;
		}

		coef_node_start = VP9_MV_JOINTS_START;
		coef_count_node_start = VP9_MV_JOINTS_COUNT_START;
		for (tree_i = 0 ; tree_i < 1 ; tree_i++) {
			for (node = 0 ; node < 3 ; node++) {
				unsigned int start = coef_count_node_start;

				switch (node) {
				case 2:
					tree_left = count[start + 2];
					tree_right = count[start + 3];
					break;
				case 1:
					tree_left = count[start + 1];
					tree_right = count[start + 2] +
						     count[start + 3];
					break;
				default:
					tree_left = count[start + 0];
					tree_right = count[start + 1] +
						     count[start + 2] +
						     count[start + 3];
					break;
				}

				vp9_tree_merge_probs(prev_prob, cur_prob,
						     coef_node_start,
						     tree_left, tree_right,
						     tree_i, node);

				coef_node_start = coef_node_start + 1;
			}
			coef_count_node_start = coef_count_node_start + 4;
		}

		for (mvd_i = 0 ; mvd_i < 2 ; mvd_i++) {
			coef_node_start = mvd_i ? VP9_MV_CLASSES_1_START :
						  VP9_MV_CLASSES_0_START;
			coef_count_node_start = mvd_i ?
					VP9_MV_CLASSES_1_COUNT_START :
					VP9_MV_CLASSES_0_COUNT_START;
			tree_i = 0;
			for (node = 0; node < 10; node++) {
				unsigned int start = coef_count_node_start;

				switch (node) {
				case 9:
					tree_left = count[start + 9];
					tree_right = count[start + 10];
					break;
				case 8:
					tree_left = count[start + 7];
					tree_right = count[start + 8];
					break;
				case 7:
					tree_left = count[start + 7] +
						     count[start + 8];
					tree_right = count[start + 9] +
						     count[start + 10];
					break;
				case 6:
					tree_left = count[start + 6];
					tree_right = count[start + 7] +
						     count[start + 8] +
						     count[start + 9] +
						     count[start + 10];
					break;
				case 5:
					tree_left = count[start + 4];
					tree_right = count[start + 5];
					break;
				case 4:
					tree_left = count[start + 4] +
						    count[start + 5];
					tree_right = count[start + 6] +
						     count[start + 7] +
						     count[start + 8] +
						     count[start + 9] +
						     count[start + 10];
					break;
				case 3:
					tree_left = count[start + 2];
					tree_right = count[start + 3];
					break;
				case 2:
					tree_left = count[start + 2] +
						    count[start + 3];
					tree_right = count[start + 4] +
						     count[start + 5] +
						     count[start + 6] +
						     count[start + 7] +
						     count[start + 8] +
						     count[start + 9] +
						     count[start + 10];
					break;
				case 1:
					tree_left = count[start + 1];
					tree_right = count[start + 2] +
						     count[start + 3] +
						     count[start + 4] +
						     count[start + 5] +
						     count[start + 6] +
						     count[start + 7] +
						     count[start + 8] +
						     count[start + 9] +
						     count[start + 10];
					break;
				default:
					tree_left = count[start + 0];
					tree_right = count[start + 1] +
						     count[start + 2] +
						     count[start + 3] +
						     count[start + 4] +
						     count[start + 5] +
						     count[start + 6] +
						     count[start + 7] +
						     count[start + 8] +
						     count[start + 9] +
						     count[start + 10];
					break;
				}

				vp9_tree_merge_probs(prev_prob, cur_prob,
						     coef_node_start,
						     tree_left, tree_right,
						     tree_i, node);

				coef_node_start = coef_node_start + 1;
			}

			coef_node_start = mvd_i ? VP9_MV_CLASS0_1_START :
						  VP9_MV_CLASS0_0_START;
			coef_count_node_start =	mvd_i ?
						VP9_MV_CLASS0_1_COUNT_START :
						VP9_MV_CLASS0_0_COUNT_START;
			tree_i = 0;
			node = 0;
			tree_left = count[coef_count_node_start + 0];
			tree_right = count[coef_count_node_start + 1];

			vp9_tree_merge_probs(prev_prob, cur_prob,
					     coef_node_start,
					     tree_left, tree_right,
					     tree_i, node);
			coef_node_start = mvd_i ? VP9_MV_CLASS0_FP_1_START :
						  VP9_MV_CLASS0_FP_0_START;
			coef_count_node_start =	mvd_i ?
					VP9_MV_CLASS0_FP_1_COUNT_START :
					VP9_MV_CLASS0_FP_0_COUNT_START;

			for (tree_i = 0; tree_i < 3; tree_i++) {
				for (node = 0; node < 3; node++) {
					unsigned int start =
						coef_count_node_start;
					switch (node) {
					case 2:
						tree_left = count[start + 2];
						tree_right = count[start + 3];
						break;
					case 1:
						tree_left = count[start + 1];
						tree_right = count[start + 2] +
							     count[start + 3];
						break;
					default:
						tree_left = count[start + 0];
						tree_right = count[start + 1] +
							     count[start + 2] +
							     count[start + 3];
						break;
					}

					vp9_tree_merge_probs(prev_prob,
							     cur_prob,
							     coef_node_start,
							     tree_left,
							     tree_right,
							     tree_i, node);

					coef_node_start = coef_node_start + 1;
				}
				coef_count_node_start =
					coef_count_node_start + 4;
			}
		}
	}
}

static irqreturn_t codec_vp9_threaded_isr(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_vp9 *vp9 = sess->priv;
	u32 dec_status = amvdec_read_dos(core, VP9_DEC_STATUS_REG);
	u32 prob_status = amvdec_read_dos(core, VP9_ADAPT_PROB_REG);
	int i;

	if (!vp9)
		return IRQ_HANDLED;

	mutex_lock(&vp9->lock);
	dev_dbg(core->dev_dec, "VP9 ISR: dec_status=%08X prob_status=%08X frames_num=%d\n",
		dec_status, prob_status, vp9->frames_num);
	if (dec_status == VP9_EOS) {
		atomic_inc(&core->debugfs.irq.vp9_eos);
		/*
		 * Firmware signals end-of-stream after finding the
		 * 0x55555555 sentinel.  The DW pipeline may still have
		 * 2-3 LCU rows buffered (confirmed: bottom ~144 rows
		 * corrupt on GXL 720p).
		 *
		 * Wait for the DW pipeline to drain before delivering.  Exit
		 * when SAO_IF_STATUS shows no bit OUTSIDE HEVC_SAO_IF_IDLE_VALUE
		 * for HEVC_DRAIN_STABLE consecutive reads (SAO idle -- includes
		 * the 0x0 "engine already quiesced" reading seen here on
		 * G12A+).  A still-running flush holds a bit outside the
		 * baseline and keeps us looping.  The old !sao_if + read-only
		 * C_WPTR test never matched and burned the full cap.
		 */
		u32 sao_if = 0;
		int sao_wait, idle_stable = 0;

		for (sao_wait = 0; sao_wait < 2000; sao_wait++) {
			sao_if = amvdec_read_dos(core, HEVC_SAO_IF_STATUS);
			if ((sao_if & ~HEVC_SAO_IF_IDLE_VALUE) == 0) {
				if (++idle_stable >= HEVC_DRAIN_STABLE)
					break;
			} else {
				idle_stable = 0;
			}
			udelay(50);
		}
		dev_dbg(core->dev_dec,
			"VP9 EOS: SAO settled in %d loops (if=0x%x)\n",
			sao_wait, sao_if);

		if (vp9->cur_frame && vp9->cur_frame->show &&
		    !vp9->cur_frame->done) {
			amvdec_dst_buf_done(sess, vp9->cur_frame->vbuf,
					    V4L2_FIELD_NONE);
			vp9->cur_frame->done = 1;
			vp9->frames_num--;
			/* VP9-001 S3: wake flush_output's wait_event on
			 * cur_frame->done.  No-op if no waiter is parked
			 * (e.g. ISR fires outside drain).
			 */
			wake_up(&vp9->cur_frame_done_wq);
		}

		/*
		 * The sentinel was queued via esparser_queue_eos which
		 * incremented esparser_queued_bufs.  The sentinel doesn't
		 * produce a frame (0xf2 is not HEAD_PARSER_DONE), so no
		 * amvdec_dst_buf_done decrements it.  Without this, the
		 * drain_work post-drain wait sees queued_bufs==1 and stalls
		 * for 5s before sending LAST/EOS.
		 */
		atomic_dec_if_positive(&sess->esparser_queued_bufs);
		wake_up(&sess->esparser_queue_done_wq);

		amvdec_write_dos(core, VP9_DEC_STATUS_REG,
				 VP9_10B_DISCARD_NAL);
		goto unlock;
	}

	if (dec_status != VP9_HEAD_PARSER_DONE) {
		/*
		 * VP9-001 GXL fix: the AMLV header search subroutine at
		 * firmware address 0x0a2b fires MBOX1 with various status
		 * codes (2, 3, etc.) during the inter-frame search loop.
		 * The firmware expects the driver to ACK by writing 0 to
		 * DEC_STATUS_REG, then continues searching.
		 *
		 * Without this handler, amvdec_abort() halts the firmware
		 * (MPSR=0) before it can find the 0x55555555 EOS sentinel,
		 * preventing VP9_EOS (0xf2) from ever firing on GXL.
		 *
		 * ACK and return -- the firmware will continue searching
		 * and either find the sentinel (fire 0xf2) or exhaust the
		 * VIFIFO (stall harmlessly until session teardown).
		 */
		dev_dbg(core->dev_dec,
			"VP9 ISR: AMLV search status %08X, ACK\n",
			dec_status);
		amvdec_write_dos(core, VP9_DEC_STATUS_REG, 0);
		goto unlock;
	}

	/* dec_status == VP9_HEAD_PARSER_DONE at this point */
	atomic_inc(&core->debugfs.irq.vp9_head_parser_done);

	pr_debug("ISR: %08X;%08X\n", dec_status, prob_status);
	sess->keyframe_found = 1;

	if ((prob_status & 0xff) == VP9_REQ_ADAPT_PROB && vp9->cur_frame) {
		/* Firmware requests probability table adaptation */
		u8 *prev_prob_b = ((u8 *)vp9->workspace_vaddr +
					 PROB_OFFSET) +
					((prob_status >> 8) *
					 VP9_PROB_SET_SIZE);
		u8 *cur_prob_b = ((u8 *)vp9->workspace_vaddr +
					 PROB_OFFSET) + VP9_CUR_PROB_OFFSET;
		u8 *count_b = (u8 *)vp9->workspace_vaddr +
				   COUNT_OFFSET;
		int last_frame_type = vp9->prev_frame ?
						vp9->prev_frame->type :
						KEY_FRAME;

		adapt_coef_probs(last_frame_type == KEY_FRAME,
				 vp9->cur_frame->type == KEY_FRAME ? 1 : 0,
				 prob_status >> 8,
				 (unsigned int *)prev_prob_b,
				 (unsigned int *)cur_prob_b,
				 (unsigned int *)count_b);

		memcpy(prev_prob_b, cur_prob_b, ADAPT_PROB_SIZE);
		amvdec_write_dos(core, VP9_ADAPT_PROB_REG, 0);
	}

	/* Invalidate first 3 refs */
	for (i = 0; i < REFS_PER_FRAME ; ++i)
		vp9->frame_refs[i] = NULL;

	vp9->prev_frame = vp9->cur_frame;
	codec_vp9_update_ref(vp9);

	codec_vp9_fetch_rpm(sess);

	/* VP9 Profile 0 (8-bit 4:2:0) is supported.
	 * Profiles 1 and 3 (4:2:2/4:4:4 chroma) should be rejected at the
	 * esparser level, but check here as a safety net in case one
	 * somehow reaches the ISR.
	 * Profile 2 (10-bit 4:2:0) is explicitly rejected below: the DW
	 * hardware cannot perform 10-bit-to-8-bit chroma conversion, causing
	 * corrupt U/V channels on G12B/SM1 and silent frame drops on GXL.
	 */
	if (vp9->rpm_param.p.profile == 1 || vp9->rpm_param.p.profile == 3) {
		dev_err(core->dev_dec,
			"VP9 Profile %u not supported (only Profile 0 and 2)\n",
			vp9->rpm_param.p.profile);
		amvdec_abort(sess);
		goto unlock;
	}

	if (vp9->rpm_param.p.bit_depth == 10 &&
	    core->platform->revision < VDEC_REVISION_G12A) {
		dev_err(core->dev_dec,
			"VP9 Profile 2 (10-bit) unsupported on GXL\n");
		amvdec_abort(sess);
		goto unlock;
	}
	if (vp9->rpm_param.p.bit_depth == 10 && !vp9->warned_10bit_capture) {
		if (codec_hevc_is_fbc_output(sess->pixfmt_cap))
			dev_info(core->dev_dec,
				 "VP9 Profile 2 (10-bit): FBC capture preserves full 10-bit pixels in compressed body\n");
		else
			dev_info(core->dev_dec,
				 "VP9 Profile 2 (10-bit): decode enabled, NV12 capture truncates to 8-bit (use MESON_FBC10_SCATTER for 10-bit)\n");
		vp9->warned_10bit_capture = true;
	}

	/*
	 * N18: reject a frame whose CODED width or height is not a multiple
	 * of 8 (the VP9 8x8 MI grid unit) BEFORE submitting it to the HEVC
	 * decode engine.  The firmware (sm1_vp9_mmu.bin and the GXL/G12A
	 * variants) hard-hangs the engine on the partial bottom MI row / right
	 * MI column of a non-8-aligned frame: after VP9_10B_DECODE_SLICE the
	 * SAO/deblock write never completes, no IRQ fires, and the SoC
	 * watchdog resets the whole chip -- a full reboot with NO oops/panic.
	 * Confirmed on G12B (A311D) via netconsole with the fluster
	 * VP9-TEST-VECTORS size-* conformance vectors: 132x130, 130x132,
	 * 132x132, 180x180, 32x34, and subpixel-00's 320x180 keyframe each
	 * reset the board on the first decode.  8-aligned coded frames decode
	 * normally (384x288 and 1920x1080 verified), and VP9 encoders always
	 * emit 8-aligned coded frames, so this never rejects a real stream.
	 * The BSP guard (format_resolution_fatal_error) checks only oversize,
	 * not alignment; upstream must gate it here.  Abort cleanly -- the
	 * engine has not been handed the decode yet, so the board survives.
	 */
	if ((vp9->rpm_param.p.width & 7) || (vp9->rpm_param.p.height & 7)) {
		dev_err(core->dev_dec,
			"VP9: coded size %ux%u not 8-aligned; the HW would hard-reset the SoC (N18) -- rejecting\n",
			vp9->rpm_param.p.width, vp9->rpm_param.p.height);
		amvdec_abort(sess);
		goto unlock;
	}

	if (codec_vp9_process_rpm(vp9)) {
		/*
		 * VP9 defines exactly REF_FRAMES (8) reference frame slots.
		 * The old value of 16 was double the actual spec maximum and
		 * inflated the DPB size used to size the capture buffer pool,
		 * contributing to CMA exhaustion at 4K with >8 capture buffers.
		 */
		amvdec_src_change(sess, vp9->width, vp9->height, REF_FRAMES);

		/*
		 * Grow the workspace to the size the STREAM just declared.
		 * A client that sizes its S_FMT from the ENUM_FRAMESIZES min
		 * (c2: 256x144) starts the session with MPRED MV slots far
		 * too small for the real coded size; on this plain (cap not
		 * streaming) src_change path resume() never runs, so the
		 * only grow call site -- codec_vp9_resume() -- is skipped
		 * and the firmware stalls with every buffer queued. Do it
		 * here instead: the decode for this frame has not been
		 * submitted yet and reference state is invalidated by the
		 * size change anyway, so nothing references the old layout.
		 */
		if (codec_vp9_grow_workspace(sess, vp9)) {
			amvdec_abort(sess);
			goto unlock;
		}
		codec_vp9_setup_workspace(sess, vp9);

		/*
		 * When amvdec_src_change() triggers a fast resume
		 * (resolution unchanged, capture queue already streaming),
		 * resume() calls process_frame() which sets cur_frame to
		 * the new frame and starts its decode.  In that case,
		 * cur_frame is valid and must NOT be cleared -- clearing it
		 * causes show_frame() to prematurely deliver and free the
		 * frame, destroying the reference that INTER frames need.
		 *
		 * Only clear cur_frame when no fast resume occurred (i.e.,
		 * the resolution actually changed and we're waiting for
		 * userspace to reconfigure the capture queue).
		 */
		if (!vp9->cur_frame) {
			/* Show the remaining frame */
			codec_vp9_show_frame(sess);

		}

		goto unlock;
	}

	codec_vp9_process_lf(vp9);
	/* VP9-001: removed should_stop guard -- let final frames decode */
	codec_vp9_process_frame(sess);
	codec_vp9_show_frame(sess);

unlock:
	mutex_unlock(&vp9->lock);
	return IRQ_HANDLED;
}

static irqreturn_t codec_vp9_isr(struct amvdec_session *sess)
{
	return IRQ_WAKE_THREAD;
}

/*
 * Halt the VP9 firmware at end of drain, matching BSP amhevc_stop().
 * gxl_vp9_mmu.bin spins at PC 0x0955 (amlv_search_entry) reading
 * VIFIFO forever after the last frame is consumed -- on GXL, the
 * sentinel-injection path that would fire VP9_EOS (0xf2) is skipped
 * (VP9-023 workaround), so the firmware never exits the loop on its
 * own until STREAMOFF.  Halting here stops the spin, ends the
 * spurious MBOX1 status-code-2/3 IRQs from amlv_search_consume, and
 * closes the esparser_queued_bufs protocol now that no more ISRs can
 * decrement it.
 *
 * Called from vdec_drain_work() AFTER the 5s esparser-counter wait,
 * vdec_wait_inactive(), and flush_pending_frames() -- so late
 * post-EOS ISRs have already delivered any frames they were going
 * to.  This is why halting here does NOT regress the GXL frame-count
 * baseline (unlike halting inside codec_ops->drain / flush_output).
 */
static void codec_vp9_drain_finalize(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;

	amvdec_hevc_halt_firmware(core);
	sess->firmware_halted = 1;

	if (atomic_read(&sess->esparser_queued_bufs) > 0) {
		dev_dbg(core->dev,
			"VP9 drain finalize: forcing queued count from %d to 0 (firmware halted, no more ISRs)\n",
			atomic_read(&sess->esparser_queued_bufs));
		atomic_set(&sess->esparser_queued_bufs, 0);
		wake_up(&sess->esparser_queue_done_wq);
	}
}

/*
 * Arm-on-available wake.  The work replays process_frame() for the frame
 * that was left pending on an empty CAPTURE pool.  It takes vp9->lock --
 * the same lock the threaded ISR uses -- so it serialises against the
 * parse path; if the ISR already armed in the meantime (flag cleared),
 * this is a no-op.  No engine access happens between the
 * defer and here, which is exactly why the v8 re-drive livelock cannot
 * recur: there is nothing armed for the watchdog to re-kick.
 */
static void codec_vp9_deferred_arm_work(struct work_struct *work)
{
	struct codec_vp9 *vp9 = container_of(work, struct codec_vp9,
					    deferred_arm_work);
	struct amvdec_session *sess = vp9->sess;

	if (!atomic_read(&vp9->arm_deferred) || sess->aborted ||
	    !sess->streamon_cap)
		return;

	/*
	 * The flag alone says whether the frame is still pending:
	 * process_frame() clears it whenever it arms, under this lock.
	 * cur_frame is NOT a usable test -- a deferral from the ISR path
	 * leaves it pointing at the previous, already-decoded frame
	 * (prev_frame = cur_frame happens before process_frame), so a
	 * "!cur_frame" guard never re-arms and the session stalls.
	 */
	mutex_lock(&vp9->lock);
	if (atomic_read(&vp9->arm_deferred))
		codec_vp9_process_frame(sess);
	mutex_unlock(&vp9->lock);
}

/*
 * Called from vdec_vb2_buf_queue() with the just-queued CAPTURE buffer
 * already on the m2m list.  buf_queue context forbids the engine work
 * itself, so schedule it.
 */
static void codec_vp9_arm_pending(struct amvdec_session *sess)
{
	struct codec_vp9 *vp9 = sess->priv;

	if (!vp9 || !atomic_read(&vp9->arm_deferred))
		return;

	if (v4l2_m2m_num_dst_bufs_ready(sess->m2m_ctx) > 0)
		schedule_work(&vp9->deferred_arm_work);
}

struct amvdec_codec_ops codec_vp9_ops = {
	.start = codec_vp9_start,
	.stop = codec_vp9_stop,
	.isr = codec_vp9_isr,
	.threaded_isr = codec_vp9_threaded_isr,
	.num_pending_bufs = codec_vp9_num_pending_bufs,
	.drain = codec_vp9_flush_output,
	.drain_finalize = codec_vp9_drain_finalize,
	/* VP9-001: sentinel injected by flush_output after all frames decoded */
	.resume = codec_vp9_resume,
	.arm_pending = codec_vp9_arm_pending,
};

static int __init codec_vp9_mod_init(void)
{
	return amvdec_register_codec(V4L2_PIX_FMT_VP9, &codec_vp9_ops);
}

static void __exit codec_vp9_mod_exit(void)
{
	amvdec_unregister_codec(V4L2_PIX_FMT_VP9, &codec_vp9_ops);
}

module_init(codec_vp9_mod_init);
module_exit(codec_vp9_mod_exit);
MODULE_DESCRIPTION("Meson VDEC VP9 codec");
MODULE_LICENSE("GPL");
