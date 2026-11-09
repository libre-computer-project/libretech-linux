/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * HCODEC register byte offsets within the DOS address space.
 *
 * BSP uses word addresses with WRITE_HREG which ORs 0x1000 then shifts left 2.
 * Byte offset = ((bsp_word_addr | 0x1000) << 2)
 *
 * For registers whose BSP word address already has bit 12 set (>= 0x1000),
 * the formula simplifies to (bsp_word_addr << 2) = (bsp_word_addr * 4).
 *
 * Copyright (C) 2024 LibreTech
 */

#ifndef __MESON_HCODEC_REGS_H
#define __MESON_HCODEC_REGS_H

/*
 * DOS top-level registers (shared with decoder, byte offsets from DOS base)
 */
#define DOS_SW_RESET1		0xfc1c	/* 0x3f07 << 2 */
#define DOS_MEM_PD_HCODEC	0xfcc8	/* 0x3f32 << 2 */

/*
 * MFDIN - Media Front-end Data Input (GXL+ addresses, post-GXBB)
 * BSP uses (reg - 8) for GXBB+; t5w header has the corrected values.
 */
#define HCODEC_MFDIN_REG1_CTRL	0x4024	/* 0x1009 << 2 */
#define HCODEC_MFDIN_REG3_CANV	0x402c	/* 0x100b << 2 */
#define HCODEC_MFDIN_REG4_LNR0	0x4030	/* 0x100c << 2 */
#define HCODEC_MFDIN_REG5_LNR1	0x4034	/* 0x100d << 2 */
#define HCODEC_MFDIN_REG7_SCMD	0x403c	/* 0x100f << 2 */
#define HCODEC_MFDIN_REG8_DMBL	0x4040	/* 0x1010 << 2 */
#define HCODEC_MFDIN_REG9_ENDN	0x4044	/* 0x1011 << 2 */
#define HCODEC_MFDIN_REGB_AMPC	0x404c	/* 0x1013 << 2 */

/* ASSIST / Mailbox */
#define HCODEC_ASSIST_MMC_CTRL1	0x4008	/* 0x1002 << 2 */
#define HCODEC_ASSIST_AMR1_INT0	0x4094	/* 0x1025 << 2 */
#define HCODEC_ASSIST_AMR1_INT1	0x4098	/* 0x1026 << 2 */
#define HCODEC_ASSIST_AMR1_INT3	0x40a0	/* 0x1028 << 2 */
#define HCODEC_ASSIST_MBOX2_CLR_REG	0x41e4	/* 0x1079 << 2 */
#define HCODEC_ASSIST_MBOX2_MASK	0x41e8	/* 0x107a << 2 */

/* AMRISC CPU control */
#define HCODEC_MPSR		0x4c04	/* 0x1301 << 2 */
#define HCODEC_CPSR		0x4c84	/* 0x1321 << 2 */
#define HCODEC_MPC_E		0x4c20	/* 0x1308 << 2 */
#define HCODEC_IMEM_DMA_CTRL	0x4d00	/* 0x1340 << 2 */
#define HCODEC_IMEM_DMA_ADR	0x4d04	/* 0x1341 << 2 */
#define HCODEC_IMEM_DMA_COUNT	0x4d08	/* 0x1342 << 2 */

/* MC / Canvas address registers */
#define HCODEC_HDEC_MC_OMEM_AUTO	0x64c0	/* 0x1930 << 2 */
#define HCODEC_ANC0_CANVAS_ADDR	0x6640	/* 0x1990 << 2 */
#define HCODEC_DBKR_CANVAS_ADDR	0x66c0	/* 0x19b0 << 2 */
#define HCODEC_DBKW_CANVAS_ADDR	0x66c4	/* 0x19b1 << 2 */
#define HCODEC_REC_CANVAS_ADDR	0x66c8	/* 0x19b2 << 2 */
#define HCODEC_CURR_CANVAS_CTRL	0x66cc	/* 0x19b3 << 2 */

/*
 * Canvas LUT (HCODEC-local)
 * BSP word addr 0x09ea..0x09ed, WRITE_HREG ORs 0x1000 → 0x19ea..0x19ed
 */
#define HCODEC_MDEC_CAV_LUT_DATAL	0x67a8	/* 0x19ea << 2 */
#define HCODEC_MDEC_CAV_LUT_DATAH	0x67ac	/* 0x19eb << 2 */
#define HCODEC_MDEC_CAV_LUT_ADDR	0x67b0	/* 0x19ec << 2 */
#define HCODEC_MDEC_CAV_CFG0		0x67b4	/* 0x19ed << 2 */

/* HENC_SCRATCH registers (firmware interface) */
#define HCODEC_HENC_SCRATCH_0	0x6b00	/* 0x1ac0 << 2 */
#define HCODEC_HENC_SCRATCH_1	0x6b04
#define HCODEC_HENC_SCRATCH_2	0x6b08
#define HCODEC_HENC_SCRATCH_3	0x6b0c
#define HCODEC_HENC_SCRATCH_4	0x6b10
#define HCODEC_HENC_SCRATCH_5	0x6b14
#define HCODEC_HENC_SCRATCH_6	0x6b18
#define HCODEC_HENC_SCRATCH_7	0x6b1c
#define HCODEC_HENC_SCRATCH_8	0x6b20
#define HCODEC_HENC_SCRATCH_9	0x6b24
#define HCODEC_HENC_SCRATCH_A	0x6b28
#define HCODEC_HENC_SCRATCH_B	0x6b2c
#define HCODEC_HENC_SCRATCH_C	0x6b30
#define HCODEC_HENC_SCRATCH_D	0x6b34
#define HCODEC_HENC_SCRATCH_E	0x6b38
#define HCODEC_HENC_SCRATCH_F	0x6b3c
/* Extended scratch registers (G-L) */
#define HCODEC_HENC_SCRATCH_G	0x6b40	/* 0x1ad0 << 2 */
#define HCODEC_HENC_SCRATCH_H	0x6b44
#define HCODEC_HENC_SCRATCH_I	0x6b48
#define HCODEC_HENC_SCRATCH_J	0x6b4c
#define HCODEC_HENC_SCRATCH_K	0x6b50
#define HCODEC_HENC_SCRATCH_L	0x6b54

/* IE data feed */
#define HCODEC_IE_DATA_FEED_BUFF_INFO	0x6b60	/* 0x1ad8 << 2 */

/* Firmware scratch register aliases (must match BSP encoder.h) */
#define ENCODER_STATUS		HCODEC_HENC_SCRATCH_0
#define MEM_OFFSET_REG		HCODEC_HENC_SCRATCH_1
#define IDR_PIC_ID		HCODEC_HENC_SCRATCH_5
#define FRAME_NUMBER		HCODEC_HENC_SCRATCH_6
#define PIC_ORDER_CNT_LSB	HCODEC_HENC_SCRATCH_7
#define LOG2_MAX_PIC_ORDER_CNT_LSB HCODEC_HENC_SCRATCH_8
#define LOG2_MAX_FRAME_NUM	HCODEC_HENC_SCRATCH_9
#define ANC0_BUFFER_ID		HCODEC_HENC_SCRATCH_A
#define QPPICTURE		HCODEC_HENC_SCRATCH_B
#define IE_ME_MB_TYPE		HCODEC_HENC_SCRATCH_D
#define IE_ME_MODE		HCODEC_HENC_SCRATCH_E
#define H264_ENC_CBR_CTL	HCODEC_HENC_SCRATCH_G
#define H264_ENC_CBR_TARGET_SIZE HCODEC_HENC_SCRATCH_H
#define INFO_DUMP_START_ADDR	HCODEC_HENC_SCRATCH_I
#define H264_ENC_CBR_REGION_SIZE HCODEC_HENC_SCRATCH_J
#define H264_ENC_CBR_TABLE_ADDR	HCODEC_HENC_SCRATCH_3
#define H264_ENC_CBR_MB_SIZE_ADDR HCODEC_HENC_SCRATCH_4
#define H264_ENC_SVC_PIC_TYPE	HCODEC_HENC_SCRATCH_K
#define FIXED_SLICE_CFG		HCODEC_HENC_SCRATCH_L

/* VLC - Variable Length Coding */
#define HCODEC_VLC_STATUS_CTRL	0x7400	/* 0x1d00 << 2 */
#define HCODEC_VLC_CONFIG	0x7404	/* 0x1d01 << 2 */
#define HCODEC_VLC_VB_START_PTR	0x7440	/* 0x1d10 << 2 */
#define HCODEC_VLC_VB_END_PTR	0x7444	/* 0x1d11 << 2 */
#define HCODEC_VLC_VB_WR_PTR	0x7448	/* 0x1d12 << 2 */
#define HCODEC_VLC_VB_SW_RD_PTR	0x7450	/* 0x1d14 << 2 */
#define HCODEC_VLC_VB_CONTROL	0x7458	/* 0x1d16 << 2 */
#define HCODEC_VLC_VB_MEM_CTL	0x745c	/* 0x1d17 << 2 */
#define HCODEC_VLC_TOTAL_BYTES	0x7468	/* 0x1d1a << 2 */
#define HCODEC_VLC_ADV_CONFIG	0x7494	/* 0x1d25 << 2 */
#define HCODEC_VLC_INT_CONTROL	0x74c0	/* 0x1d30 << 2 */
#define HCODEC_VLC_PIC_SIZE	0x74c4	/* 0x1d31 << 2 */
#define HCODEC_VLC_PIC_POSITION	0x74cc	/* 0x1d33 << 2 */
#define HCODEC_VLC_MB_INFO	0x74d4	/* 0x1d35 << 2 */
#define HCODEC_VLC_HCMD_CONFIG	0x7550	/* 0x1d54 << 2 */

/* VLC - JPEG specific */
#define HCODEC_VLC_JPEG_CTRL	0x7560	/* 0x1d58 << 2 */
#define HCODEC_VLC_HUFFMAN_ADDR	0x7568	/* 0x1d5a << 2 */
#define HCODEC_VLC_HUFFMAN_DATA	0x756c	/* 0x1d5b << 2 */

/* QDCT - Quantization and DCT */
#define HCODEC_QDCT_STATUS_CTRL	0x7c00	/* 0x1f00 << 2 */
#define HCODEC_QDCT_CONFIG	0x7c04	/* 0x1f01 << 2 */
#define HCODEC_IGNORE_CONFIG	0x7c08	/* 0x1f02 << 2 */
#define HCODEC_IGNORE_CONFIG_2	0x7c0c	/* 0x1f03 << 2 */
#define HCODEC_QDCT_MB_START_PTR	0x7c40	/* 0x1f10 << 2 */
#define HCODEC_QDCT_MB_END_PTR	0x7c44	/* 0x1f11 << 2 */
#define HCODEC_QDCT_MB_WR_PTR	0x7c48	/* 0x1f12 << 2 */
#define HCODEC_QDCT_MB_RD_PTR	0x7c4c	/* 0x1f13 << 2 */
#define HCODEC_QDCT_MB_CONTROL	0x7c54	/* 0x1f15 << 2 */
#define HCODEC_QDCT_MB_BUFF	0x7c5c	/* 0x1f17 << 2 */
#define HCODEC_QDCT_Q_QUANT_I	0x7c70	/* 0x1f1c << 2 */
#define HCODEC_QDCT_Q_QUANT_P	0x7c74	/* 0x1f1d << 2 */
#define HCODEC_QDCT_ADV_CONFIG	0x7cd0	/* 0x1f34 << 2 */
#define HCODEC_IE_WEIGHT	0x7cd4	/* 0x1f35 << 2 */
#define HCODEC_Q_QUANT_CONTROL	0x7cd8	/* 0x1f36 << 2 */
#define HCODEC_QUANT_TABLE_DATA	0x7ce4	/* 0x1f39 << 2 */

/* QDCT - JPEG specific */
#define HCODEC_QDCT_JPEG_CTRL		0x7c94	/* 0x1f25 << 2 */
#define HCODEC_QDCT_JPEG_X_START_END	0x7c98	/* 0x1f26 << 2 */
#define HCODEC_QDCT_JPEG_Y_START_END	0x7c9c	/* 0x1f27 << 2 */
#define HCODEC_QDCT_JPEG_QUANT_ADDR	0x7ca0	/* 0x1f28 << 2 */
#define HCODEC_QDCT_JPEG_QUANT_DATA	0x7ca4	/* 0x1f29 << 2 */
#define HCODEC_SAD_CONTROL_0	0x7ce8	/* 0x1f3a << 2 */
#define HCODEC_SAD_CONTROL_1	0x7cec	/* 0x1f3b << 2 */
#define HCODEC_QDCT_VLC_QUANT_CTL_0	0x7cf0	/* 0x1f3c << 2 */
#define HCODEC_QDCT_VLC_QUANT_CTL_1	0x7cf4	/* 0x1f3d << 2 */

/* IE - Intra Estimation */
#define HCODEC_IE_CONTROL	0x7d00	/* 0x1f40 << 2 */
#define HCODEC_SAD_CONTROL	0x7d0c	/* 0x1f43 << 2 */
#define HCODEC_IE_RESULT_BUFFER	0x7d10	/* 0x1f44 << 2 */

/* ME - Motion Estimation */
#define HCODEC_ME_SKIP_LINE	0x7d34	/* 0x1f4d << 2 */
#define HCODEC_ME_SAD_ENOUGH_01	0x7d40	/* 0x1f50 << 2 */
#define HCODEC_ME_SAD_ENOUGH_23	0x7d44	/* 0x1f51 << 2 */
#define HCODEC_ME_STEP0_CLOSE_MV	0x7d48	/* 0x1f52 << 2 */
#define HCODEC_ME_F_SKIP_SAD	0x7d4c	/* 0x1f53 << 2 */
#define HCODEC_ME_F_SKIP_WEIGHT	0x7d50	/* 0x1f54 << 2 */
#define HCODEC_ME_MV_MERGE_CTL	0x7d54	/* 0x1f55 << 2 */
#define HCODEC_ME_MV_WEIGHT_01	0x7d58	/* 0x1f56 << 2 */
#define HCODEC_ME_MV_WEIGHT_23	0x7d5c	/* 0x1f57 << 2 */
#define HCODEC_ME_SAD_RANGE_INC	0x7d60	/* 0x1f58 << 2 */
#define HCODEC_ME_WEIGHT	0x7d80	/* 0x1f60 << 2 */

/* ADV MV */
#define HCODEC_ADV_MV_CTL0	0x7da4	/* 0x1f69 << 2 */
#define HCODEC_ADV_MV_CTL1	0x7da8	/* 0x1f6a << 2 */

/* V3 skip/force */
#define HCODEC_V3_SKIP_CONTROL	0x7db0	/* 0x1f6c << 2 */
#define HCODEC_V3_SKIP_WEIGHT	0x7dc0	/* 0x1f70 << 2 */
#define HCODEC_V3_L1_SKIP_MAX_SAD	0x7dc4	/* 0x1f71 << 2 */
#define HCODEC_V3_L2_SKIP_WEIGHT	0x7dc8	/* 0x1f72 << 2 */
#define HCODEC_V3_MV_SAD_TABLE	0x7dcc	/* 0x1f73 << 2 */
#define HCODEC_V3_F_ZERO_CTL_0	0x7dd0	/* 0x1f74 << 2 */
#define HCODEC_V3_F_ZERO_CTL_1	0x7dd4	/* 0x1f75 << 2 */
#define HCODEC_V3_IPRED_TYPE_WEIGHT_0	0x7de0	/* 0x1f78 << 2 */
#define HCODEC_V3_IPRED_TYPE_WEIGHT_1	0x7de4	/* 0x1f79 << 2 */
#define HCODEC_V3_LEFT_SMALL_MAX_SAD	0x7de8	/* 0x1f7a << 2 */

/* V4/V5 */
#define HCODEC_V4_FORCE_SKIP_CFG	0x7dec	/* 0x1f7b << 2 */
#define HCODEC_V5_MB_DIFF_SUM	0x7df0	/* 0x1f7c << 2 */
#define HCODEC_V5_SMALL_DIFF_CNT	0x7df4	/* 0x1f7d << 2 */
#define HCODEC_V5_SIMPLE_MB_CTL	0x7df8	/* 0x1f7e << 2 */
#define HCODEC_V5_SIMPLE_MB_DQUANT	0x7dfc	/* 0x1f7f << 2 */
#define HCODEC_V5_SIMPLE_MB_ME_WEIGHT	0x7e00	/* 0x1f80 << 2 */

/* IMEM DMA control bits */
#define IMEM_DMA_CTRL_FILL		BIT(15)
#define IMEM_DMA_CTRL_CTRL_BITS		(0xf << 16)

/* ASSIST_MMC_CTRL1: memory controller init */
#define ASSIST_MMC_CTRL1_INIT		0x32

/* DOS_GCLK_EN0: HCODEC internal clock gate mask */
#define GCLK_EN0_HCODEC_MASK		0x7fff000

/* VLC_VB_MEM_CTL init value (BSP: avc_init_encoder) */
#define VLC_VB_MEM_CTL_INIT		((1 << 31) | (0x3f << 24) | \
					 (0x20 << 16) | (2 << 0))

/* VLC_VB_CONTROL init value (BSP: avc_init_encoder) */
#define VLC_VB_CTRL_INIT		((7 << 3) | (1 << 1))

/* VLC_CONFIG init value */
#define VLC_CONFIG_INIT			0x07

/* VLC_ADV_CONFIG init value (BSP: avc_prot_init) */
#define VLC_ADV_CONFIG_INIT		((1 << 9) | (1 << 8) | (1 << 5) | \
					 (1 << 4) | (1 << 3) | (1 << 2) | \
					 (1 << 1))

/* ASSIST_AMR1 interrupt mapping (BSP: encode_manager_init) */
#define AMR1_INT0_HCODEC		0x15
#define AMR1_INT1_HCODEC		0x08
#define AMR1_INT3_HCODEC		0x14

/*
 * QDCT_ADV_CONFIG bit fields (BSP: avc_prot_init)
 * Controls IE/ME pipeline, SRAM management, MB info latching.
 */
#define QDCT_ADV_MB_INFO_NO_I16		BIT(29)
#define QDCT_ADV_IE_DMA_MBXY_I_PRED	BIT(28)
#define QDCT_ADV_IE_DMA_RD_MBRDY	BIT(27)
#define QDCT_ADV_IE_DMA_MBXY_WR		BIT(26)
#define QDCT_ADV_IE_START_QUANT		BIT(25)
#define QDCT_ADV_I_PRED_TOP_WR_DIS	BIT(24) /* IE SRAM flush disable */
#define QDCT_ADV_QUANT_ZERO_TH		BIT(22)
#define QDCT_ADV_USE_DCT		BIT(21)
#define QDCT_ADV_QUANT_ENC		BIT(19)
#define QDCT_ADV_USE_QUANT		BIT(18)
#define QDCT_ADV_MVSH_RDY		BIT(14)
#define QDCT_ADV_MB_INFO_INTER		BIT(12)
#define QDCT_ADV_IE_WEIGHT_SEL		BIT(10)
#define QDCT_ADV_IE_CBP_QUANT		BIT(9)
#define QDCT_ADV_IE_MV_MERGE		BIT(8)
#define QDCT_ADV_QUANT_TBL_MODE		BIT(6)
#define QDCT_ADV_QPB_EN			BIT(5)
#define QDCT_ADV_DEBLOCK_EN		BIT(4)
#define QDCT_ADV_RES_QUANT		BIT(3)
#define QDCT_ADV_DCT_EN			BIT(2)
#define QDCT_ADV_CONFIG_INIT		(QDCT_ADV_MB_INFO_NO_I16 | \
					 QDCT_ADV_IE_DMA_MBXY_I_PRED | \
					 QDCT_ADV_IE_DMA_RD_MBRDY | \
					 QDCT_ADV_IE_DMA_MBXY_WR | \
					 QDCT_ADV_IE_START_QUANT | \
					 QDCT_ADV_I_PRED_TOP_WR_DIS | \
					 QDCT_ADV_QUANT_ZERO_TH | \
					 QDCT_ADV_USE_DCT | \
					 QDCT_ADV_QUANT_ENC | \
					 QDCT_ADV_USE_QUANT | \
					 QDCT_ADV_MVSH_RDY | \
					 QDCT_ADV_MB_INFO_INTER | \
					 QDCT_ADV_IE_WEIGHT_SEL | \
					 QDCT_ADV_IE_CBP_QUANT | \
					 QDCT_ADV_IE_MV_MERGE | \
					 QDCT_ADV_QUANT_TBL_MODE | \
					 QDCT_ADV_QPB_EN | \
					 QDCT_ADV_DEBLOCK_EN | \
					 QDCT_ADV_RES_QUANT | \
					 QDCT_ADV_DCT_EN | \
					 (3 << 0))

/* QDCT_MB_CONTROL: reset + run values (BSP: avc_prot_init) */
#define QDCT_MB_CTRL_RESET		(BIT(9) | BIT(0))
#define QDCT_MB_CTRL_RUN		(BIT(28) | BIT(29) | BIT(19) | BIT(20) | \
					 BIT(18) | BIT(17) | BIT(14) | BIT(13) | \
					 BIT(12) | BIT(10))

/* Q_QUANT_CONTROL bits */
#define Q_QUANT_CTRL_ADDR_SHIFT		23
#define Q_QUANT_CTRL_WRITE_EN		BIT(22)

/*
 * HDEC_MC_OMEM_AUTO bits
 * [31] auto_fill_en, [30:16] pic_width_in_mb - 1
 */
#define MC_OMEM_AUTO_FILL_EN		BIT(31)
#define MC_OMEM_AUTO_WIDTH_SHIFT	16

/*
 * SAD_CONTROL bits (IE/ME SAD pipeline control)
 * [3] ie_result_rd_en, [2] ie_sad_shift_sel, [1] me_sad_shift_sel, [0] sad_switch
 */
#define SAD_CTRL_IE_RESULT_RD		BIT(3)
#define SAD_CTRL_IE_SAD_SHIFT		BIT(2)
#define SAD_CTRL_ME_SAD_SHIFT		BIT(1)
#define SAD_CTRL_SWITCH			BIT(0)

/*
 * IE_CONTROL bits
 * [30] active_sel, [1] ie_sad_en, [0] ie_start
 */
#define IE_CTRL_ACTIVE			BIT(30)
#define IE_CTRL_IE_SAD_EN		BIT(1)
#define IE_CTRL_IE_START		BIT(0)

/*
 * ME_SKIP_LINE bits (BSP: avc_prot_init)
 * [29:24] me_skip_y_3, [23:18] me_skip_x_3, [17:12] me_skip_y_2
 */
#define ME_SKIP_Y3_SHIFT		24
#define ME_SKIP_X3_SHIFT		18
#define ME_SKIP_Y2_SHIFT		12

/*
 * ME_MV_MERGE_CTL bits (BSP: encoder.c me_mv_merge_ctl)
 * [31] merge_mv_en_16, [30] merge_small_mv_en_16,
 * [29] merge_flex_en_16, [28] merge_sad_en_16,
 * [27] merge_mv_en_8, [26] merge_small_mv_en_8,
 * [25] merge_flex_en_8, [24] merge_sad_en_8,
 * [23:18] merge_mv_diff_16 (MV diff <= n pixels merged),
 * [17:12] merge_mv_diff_8,
 * [11:0] merge_min_sad (SAD >= value can merge with other MV)
 */
#define MV_MERGE_EN_16			BIT(31)
#define MV_MERGE_SMALL_EN_16		BIT(30)
#define MV_MERGE_FLEX_EN_16		BIT(29)
#define MV_MERGE_SAD_EN_16		BIT(28)
#define MV_MERGE_EN_8			BIT(27)
#define MV_MERGE_SMALL_EN_8		BIT(26)
#define MV_MERGE_FLEX_EN_8		BIT(25)
#define MV_MERGE_SAD_EN_8		BIT(24)
#define MV_MERGE_DIFF_16_SHIFT		18
#define MV_MERGE_DIFF_8_SHIFT		12
#define MV_MERGE_MIN_SAD_SHIFT		0
#define MV_MERGE_CTL_INIT		(MV_MERGE_EN_16 | MV_MERGE_SMALL_EN_16 | \
					 MV_MERGE_FLEX_EN_16 | MV_MERGE_SAD_EN_16 | \
					 MV_MERGE_EN_8 | MV_MERGE_SMALL_EN_8 | \
					 MV_MERGE_FLEX_EN_8 | MV_MERGE_SAD_EN_8 | \
					 (0x12 << MV_MERGE_DIFF_16_SHIFT) | \
					 (0x2b << MV_MERGE_DIFF_8_SHIFT) | \
					 (0x80 << MV_MERGE_MIN_SAD_SHIFT))

/*
 * ME_STEP0_CLOSE_MV bits (BSP: me_step0_close_mv)
 * [25:10] close_mv_threshold, [9:5] close_mv_y, [4:0] close_mv_x
 */
#define STEP0_CLOSE_THRESHOLD_SHIFT	10
#define STEP0_CLOSE_Y_SHIFT		5
#define STEP0_CLOSE_X_SHIFT		0
#define STEP0_CLOSE_MV_INIT		((0x100 << STEP0_CLOSE_THRESHOLD_SHIFT) | \
					 (2 << STEP0_CLOSE_Y_SHIFT) | \
					 (2 << STEP0_CLOSE_X_SHIFT))

/*
 * ME_SAD_ENOUGH_01/23 bits (BSP: avc_prot_init writes me_sad_enough_01/23,
 * values from InitEncodeWeight ME_SAD_ENOUGH_1/2_DATA, ADV_MV_8x8_ENOUGH_DATA,
 * encoder.c:288-291)
 * [27:12] enough data for search step 1 / 3, [11:0] step 0 / 2
 */
#define SAD_ENOUGH_DATA_SHIFT		12
#define ME_SAD_ENOUGH_01_INIT		((0x04 << SAD_ENOUGH_DATA_SHIFT) | 0x00)
#define ME_SAD_ENOUGH_23_INIT		((0x20 << SAD_ENOUGH_DATA_SHIFT) | 0x11)

/*
 * ME_SAD_RANGE_INC bits (BSP: ME_SAD_RANGE_0..3)
 * [31:24] range_3, [23:16] range_2, [15:8] range_1, [7:0] range_0
 */
#define SAD_RANGE_3_SHIFT		24
#define SAD_RANGE_2_SHIFT		16
#define SAD_RANGE_1_SHIFT		8
#define SAD_RANGE_0_SHIFT		0
#define SAD_RANGE_INC_INIT		(1 << SAD_RANGE_0_SHIFT) /* BSP: range_0 only, ME_SAD_RANGE_1..3 = 0 */

/*
 * V3_SKIP_CONTROL bits (BSP: avc_prot_init)
 * [31] v3_skip_en, [30] v3_step1_en,
 * [28] v3_mv_sad_en, [27] v3_ipred_type_en,
 * [23:12] v3_force_skip_sad_1 (4-block threshold),
 * [11:0] v3_force_skip_sad_0 (color block fix)
 */
#define V3_SKIP_EN			BIT(31)
#define V3_MV_SAD_EN			BIT(28)
#define V3_IPRED_TYPE_EN		BIT(27)
#define V3_FORCE_SKIP_SAD_1_SHIFT	12
#define V3_FORCE_SKIP_SAD_0_SHIFT	0
#define V3_SKIP_CONTROL_INIT		(V3_SKIP_EN | V3_MV_SAD_EN | V3_IPRED_TYPE_EN | \
					 (0x60 << V3_FORCE_SKIP_SAD_1_SHIFT) | \
					 (0x10 << V3_FORCE_SKIP_SAD_0_SHIFT))

/*
 * V3_SKIP_WEIGHT bits: [31:16] weight_1, [15:0] weight_0
 * V3_L1_SKIP_MAX_SAD: [31:16] level_1_f_max_sad, [15:0] level_1_max_sad
 * V3_L2_SKIP_WEIGHT: [31:16] force_skip_sad_2 (16-block), [15:0] weight_2
 */
#define V3_SKIP_W1_SHIFT		16
#define V3_SKIP_W0_SHIFT		0
#define V3_L1_FMAX_SAD_SHIFT		16
#define V3_L1_MAX_SAD_SHIFT		0
#define V3_L2_FSAD2_SHIFT		16
#define V3_L2_W2_SHIFT			0

/*
 * V4_FORCE_SKIP_CFG bits (BSP: avc_prot_init)
 * [31:26] qp_thresh_hi, [25:20] qp_thresh_lo,
 * [19:16] skip_sad_mult, [15:12] skip_coeff_mult
 */
#define V4_QP_THRESH_HI_SHIFT		26
#define V4_QP_THRESH_LO_SHIFT		20
#define V4_SKIP_SAD_MULT_SHIFT		16
#define V4_SKIP_COEFF_MULT_SHIFT	12

/*
 * V5_SIMPLE_MB_CTL bits (BSP: avc_prot_init)
 * [6] v5_use_small_diff_cnt, [5] v5_use_simple_mb,
 * [4] v5_pskip_sad_en, [3] v5_use_me_sad,
 * [2] v5_use_dquant, [0] v5_simple_mb_en
 */
#define V5_USE_SMALL_DIFF		BIT(6)
#define V5_USE_SIMPLE_MB		BIT(5)
#define V5_PSKIP_SAD_EN			BIT(4)
#define V5_USE_ME_SAD			BIT(3)
#define V5_USE_DQUANT			BIT(2)
#define V5_SIMPLE_MB_EN			BIT(0)
#define V5_SIMPLE_MB_CTL_INIT		(V5_USE_SMALL_DIFF | V5_USE_SIMPLE_MB | \
					 V5_PSKIP_SAD_EN | V5_USE_ME_SAD | \
					 V5_USE_DQUANT | V5_SIMPLE_MB_EN)

/*
 * V5_SMALL_DIFF_CNT bits: [31:16] small_diff_threshold, [15:0] diff_count_init
 * V5_SIMPLE_MB_DQUANT: 8 x 4-bit signed delta-QP per MB complexity level
 *   BSP IDR: 0x43210fed = {4,3,2,1,0,-1,-2,-3} (each nibble is a dQP)
 *   BSP P:   0x23110fed = {2,3,1,1,0,-1,-2,-3}
 */
#define V5_SMALL_DIFF_THR_SHIFT		16
#define V5_DIFF_COUNT_SHIFT		0
#define V5_DQUANT_IDR			0x43210fed
#define V5_DQUANT_P			0x23110fed

/*
 * QDCT_Q_QUANT_I / QDCT_Q_QUANT_P bits
 * [27:22] chroma_qp, [21:16] luma_qp,
 * [15:12] chroma_qp % 6, [11:8] chroma_qp / 6,
 * [7:4] luma_qp % 6, [3:0] luma_qp / 6
 */
#define Q_QUANT_CHROMA_QP_SHIFT		22
#define Q_QUANT_LUMA_QP_SHIFT		16
#define Q_QUANT_CHROMA_MOD6_SHIFT	12
#define Q_QUANT_CHROMA_DIV6_SHIFT	8
#define Q_QUANT_LUMA_MOD6_SHIFT		4
#define Q_QUANT_LUMA_DIV6_SHIFT		0

/*
 * QDCT_VLC_QUANT_CTL_0 bits
 * [24:19] quant_mode, [18:13] i_pic_qp (copy), [12:6] reserved, [5:0] i_pic_qp
 */
#define VLC_QUANT_CTL0_QP_HI_SHIFT	13
#define VLC_QUANT_CTL0_QP_LO_SHIFT	0

/*
 * QDCT_VLC_QUANT_CTL_1 bits (BSP: cabac_init_qp)
 * [11:6] cabac_init_qp_1, [5:0] cabac_init_qp_0
 */
#define VLC_QUANT_CTL1_QP1_SHIFT	6
#define VLC_QUANT_CTL1_QP0_SHIFT	0

/* MFDIN iformat additions */
#define MFDIN_FMT_RGB_SINGLE		1  /* RGB888/YUV444 single plane */
#define MFDIN_FMT_LINEAR		12 /* RGBA8888 linear DMA */

/* Encoder firmware status values */
#define ENCODER_IDLE		0
#define ENCODER_SEQUENCE	1
#define ENCODER_PICTURE		2
#define ENCODER_IDR		3
#define ENCODER_NON_IDR		4
#define ENCODER_SEQUENCE_DONE	7
#define ENCODER_PICTURE_DONE	8
#define ENCODER_IDR_DONE	9
#define ENCODER_NON_IDR_DONE	10
#define ENCODER_ERROR		0xff

/* JPEG encoder firmware status values */
#define JPEGENC_IDLE		0
#define JPEGENC_START		1
#define JPEGENC_MCU		3
#define JPEGENC_DONE		4

/*
 * MB type values for IE_ME_MB_TYPE (HENC_SCRATCH_D), which the firmware
 * relays verbatim into IE_ME_MB_INFO at frame start.  Values match the
 * vendor encoder.h (BSP V2603 V2.5 media_modules frame_sink/encoder/h264):
 * a P frame writes (SKIP_RUN_AUTO << 16) | (AUTO << 4) | AUTO = 0xffff00ff.
 */
#define HENC_MB_Type_I4MB	0x9
#define HENC_MB_Type_AUTO	0xf
#define HENC_SKIP_RUN_AUTO	0xffff

/* AO_RTI_PWR_CNTL_REG0: HCODEC power control bits [4:3] */
#define AO_PWR_CNTL_HCODEC_BITS    (BIT(4) | BIT(3))

/* HHI_VDEC_CLK_CNTL: upper 16 bits control HCODEC clock */
#define HHI_VDEC_CLK_HCODEC_MASK   GENMASK(31, 16)

/* V3 intra prediction mode weight defaults (BSP: avc_prot_init) */
#define V3_IPRED_W0_CH     0x08    /* mode weights for I-pred cost */
#define V3_IPRED_W0_CV     0x04
#define V3_IPRED_W0_DDL    0x28
#define V3_IPRED_W0_DDR    0x18
#define V3_IPRED_W1_VL     0x0c
#define V3_IPRED_W1_VR     0x08
#define V3_IPRED_W1_HD     0x04
#define V3_IPRED_W1_HU     0x0c

/*
 * V3 skip/SAD values, GXL+ branch of BSP avc_prot_init (encoder.c, the
 * get_cpu_type() >= MESON_CPU_MAJOR_ID_GXL block, lines 2683-2708).
 * V3_SKIP_WEIGHT_0/1 and the L2 weight_2 field are written from
 * ted_adv_skip_weight, which is 0 (encoder.c:2006); the GXTVBB branch
 * (encoder.c:2738-2755) uses the nonzero V3_SKIP_WEIGHT_* (0x10/0x8/0x3,
 * encoder.c:305-308) instead, and is not the GXL path.
 */
#define V3_SKIP_W1_DEFAULT          0x0		/* encoder.c:2006 ted_adv_skip_weight, written at :2700 */
#define V3_SKIP_W0_DEFAULT          0x0		/* encoder.c:2006 ted_adv_skip_weight, written at :2700 */
#define V3_L1_FMAX_SAD_DEFAULT      0x0		/* encoder.c:310 V3_LEVEL_1_F_SKIP_MAX_SAD */
#define V3_L1_MAX_SAD_DEFAULT       0x6		/* encoder.c:311 V3_LEVEL_1_SKIP_MAX_SAD */
#define V3_L2_FSAD2_DEFAULT         0x250	/* encoder.c:298 V3_FORCE_SKIP_SAD_2, written at :2706 */
#define V3_L2_W2_DEFAULT            0x0		/* encoder.c:2006 ted_adv_skip_weight, written at :2706 */

/* V3_F_ZERO_CTL weight offsets (BSP: avc_prot_init lambda calc) */
#define V3_I4_WEIGHT_OFFSET         0x20
#define V3_I16_WEIGHT_OFFSET        0x10
#define V3_ME_WEIGHT_OFFSET         0x10
#define V3_F_ZERO_CTL1_HI           3   /* always 3 in BSP */

/* V4 force-skip multiplier defaults */
#define V4_SKIP_SAD_MULT_DEFAULT    5
#define V4_SKIP_COEFF_MULT_DEFAULT  6

/* V5 small diff defaults */
#define V5_SMALL_DIFF_THR_DEFAULT   0x18
#define V5_DIFF_COUNT_DEFAULT       0x10

/* V3_LEFT_SMALL_MAX_SAD defaults */
#define V3_LEFT_SMALL_MAX_SAD_HI    0x40
#define V3_LEFT_SMALL_MAX_SAD_LO    0x00

/* Encoder canvas offset (well above decoder range) */
#define ENC_CANVAS_OFFSET	0x64

/* IE pipeline */
#define IE_PIPELINE_BLOCK_SHIFT	0
#define IE_PIPELINE_BLOCK_MASK	0x1f

/* SVC pic type (firmware expects these exact values) */
#define ENC_SLC_REF		0x8410
#define ENC_SLC_NON_REF		0x8010

/* MFDIN input format values */
#define MFDIN_FMT_YUV422_SINGLE	0
#define MFDIN_FMT_YUV444_SINGLE	1
#define MFDIN_FMT_NV21		2
#define MFDIN_FMT_NV12		3
#define MFDIN_FMT_YUV420	4
#define MFDIN_FMT_YUV444_PLANE	5

/* MFDIN NR registers (GXL+ only, post-GXBB offset adjustment) */
#define HCODEC_MFDIN_REG0D	0x4054	/* 0x1015 << 2 - Y SNR config */
#define HCODEC_MFDIN_REG0E	0x4058	/* 0x1016 << 2 - Y TNR config */
#define HCODEC_MFDIN_REG0F	0x405c	/* 0x1017 << 2 - Y TNR motion pt1 */
#define HCODEC_MFDIN_REG10	0x4060	/* 0x1018 << 2 - Y TNR motion pt2 */
#define HCODEC_MFDIN_REG11	0x4064	/* 0x1019 << 2 - Y blend config */
#define HCODEC_MFDIN_REG12	0x4068	/* 0x101a << 2 - C SNR config */
#define HCODEC_MFDIN_REG13	0x406c	/* 0x101b << 2 - C TNR config */
#define HCODEC_MFDIN_REG14	0x4070	/* 0x101c << 2 - C TNR motion pt1 */
#define HCODEC_MFDIN_REG15	0x4074	/* 0x101d << 2 - C TNR motion pt2 */
#define HCODEC_MFDIN_REG16	0x4078	/* 0x101e << 2 - C blend config */

#endif /* __MESON_HCODEC_REGS_H */
