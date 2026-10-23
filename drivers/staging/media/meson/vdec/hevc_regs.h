/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2015 Amlogic, Inc. All rights reserved.
 */

#ifndef __MESON_VDEC_HEVC_REGS_H_
#define __MESON_VDEC_HEVC_REGS_H_

#define HEVC_ASSIST_MMU_MAP_ADDR 0xc024

/* HEVC engine has TWO mailboxes (MBOX0 + MBOX1) for firmware-to-ARM
 * signalling.  Upstream firmware uses MBOX1 only.  BSP firmware
 * (vh265.c:13272-13275) uses MBOX0 -- the BSP_PROTO path enables it
 * additionally so the shared GIC vdec-IRQ line wakes the ARM ISR for
 * BSP firmware events.
 */
#define HEVC_ASSIST_MBOX0_CLR_REG 0xc1c4
#define HEVC_ASSIST_MBOX0_MASK    0xc1c8
#define HEVC_ASSIST_MBOX1_CLR_REG 0xc1d4
#define HEVC_ASSIST_MBOX1_MASK 0xc1d8

#define HEVC_ASSIST_SCRATCH_0 0xc300
#define HEVC_ASSIST_SCRATCH_1 0xc304
#define HEVC_ASSIST_SCRATCH_2 0xc308
#define HEVC_ASSIST_SCRATCH_3 0xc30c
#define HEVC_ASSIST_SCRATCH_4 0xc310
#define HEVC_ASSIST_SCRATCH_5 0xc314
#define HEVC_ASSIST_SCRATCH_6 0xc318
#define HEVC_ASSIST_SCRATCH_7 0xc31c
#define HEVC_ASSIST_SCRATCH_8 0xc320
#define HEVC_ASSIST_SCRATCH_9 0xc324
#define HEVC_ASSIST_SCRATCH_A 0xc328
#define HEVC_ASSIST_SCRATCH_B 0xc32c
#define HEVC_ASSIST_SCRATCH_C 0xc330
#define HEVC_ASSIST_SCRATCH_D 0xc334
#define HEVC_ASSIST_SCRATCH_E 0xc338
#define HEVC_ASSIST_SCRATCH_F 0xc33c
#define HEVC_ASSIST_SCRATCH_G 0xc340
#define HEVC_ASSIST_SCRATCH_H 0xc344
#define HEVC_ASSIST_SCRATCH_I 0xc348
#define HEVC_ASSIST_SCRATCH_J 0xc34c
#define HEVC_ASSIST_SCRATCH_K 0xc350
#define HEVC_ASSIST_SCRATCH_L 0xc354
#define HEVC_ASSIST_SCRATCH_M 0xc358
#define HEVC_ASSIST_SCRATCH_N 0xc35c

#define HEVC_PARSER_VERSION 0xc400
#define HEVC_STREAM_CONTROL 0xc404
	#define HEVC_STREAM_CTRL_ENABLE	BIT(0)
	#define HEVC_STREAM_CTRL_USE_PARSER	BIT(3)
#define HEVC_STREAM_START_ADDR 0xc408
#define HEVC_STREAM_END_ADDR 0xc40c
#define HEVC_STREAM_WR_PTR 0xc410
#define HEVC_STREAM_RD_PTR 0xc414
#define HEVC_STREAM_LEVEL 0xc418
#define HEVC_STREAM_FIFO_CTL 0xc41c
	#define HEVC_STREAM_FIFO_CTL_FILL_EN	BIT(29)
#define HEVC_SHIFT_CONTROL 0xc420
#define HEVC_SHIFT_STARTCODE 0xc424
#define HEVC_SHIFT_EMULATECODE 0xc428
#define HEVC_SHIFT_STATUS 0xc42c
#define HEVC_SHIFTED_DATA 0xc430
#define HEVC_SHIFT_BYTE_COUNT 0xc434
#define HEVC_SHIFT_COMMAND 0xc438
#define HEVC_ELEMENT_RESULT 0xc43c
#define HEVC_CABAC_CONTROL 0xc440
#define HEVC_PARSER_SLICE_INFO 0xc444
#define HEVC_PARSER_CMD_WRITE 0xc448
#define HEVC_PARSER_CORE_CONTROL 0xc44c
#define HEVC_PARSER_CMD_FETCH 0xc450
#define HEVC_PARSER_CMD_STATUS 0xc454
#define HEVC_PARSER_LCU_INFO 0xc458
#define HEVC_PARSER_HEADER_INFO 0xc45c
#define HEVC_PARSER_INT_CONTROL 0xc480
#define HEVC_PARSER_INT_STATUS 0xc484
#define HEVC_PARSER_IF_CONTROL 0xc488
#define HEVC_PARSER_PICTURE_SIZE 0xc48c
#define HEVC_PARSER_LCU_START 0xc490
#define HEVC_PARSER_HEADER_INFO2 0xc494
#define HEVC_PARSER_QUANT_READ 0xc498
#define HEVC_PARSER_RESERVED_27 0xc49c
#define HEVC_PARSER_CMD_SKIP_0 0xc4a0
#define HEVC_PARSER_CMD_SKIP_1 0xc4a4
#define HEVC_PARSER_CMD_SKIP_2 0xc4a8
#define HEVC_SAO_IF_STATUS 0xc4c0
#define HEVC_SAO_IF_DATA_Y 0xc4c4
#define HEVC_SAO_IF_DATA_U 0xc4c8
#define HEVC_SAO_IF_DATA_V 0xc4cc
#define HEVC_STREAM_SWAP_ADDR 0xc4d0
#define HEVC_STREAM_SWAP_CTRL 0xc4d4
#define HEVC_IQIT_IF_WAIT_CNT 0xc4d8
#define HEVC_MPRED_IF_WAIT_CNT 0xc4dc
#define HEVC_SAO_IF_WAIT_CNT 0xc4e0

#define HEVC_MPRED_VERSION 0xc800
#define HEVC_MPRED_CTRL0 0xc804
	#define MPRED_CTRL0_NEW_PIC	BIT(2)
	#define MPRED_CTRL0_NEW_TILE	BIT(3)
	#define MPRED_CTRL0_NEW_SLI_SEG	BIT(4)
	#define MPRED_CTRL0_TMVP	BIT(5)
	#define MPRED_CTRL0_LDC		BIT(6)
	#define MPRED_CTRL0_COL_FROM_L0	BIT(7)
	#define MPRED_CTRL0_ABOVE_EN	BIT(9)
	#define MPRED_CTRL0_MV_WR_EN	BIT(10)
	#define MPRED_CTRL0_MV_RD_EN	BIT(11)
	#define MPRED_CTRL0_BUF_LINEAR	BIT(13)
#define HEVC_MPRED_CTRL1 0xc808
	/* CTRL1 bit layout:
	 *   [3:0]  max_merge_cand (1..5)
	 *   [7:4]  AMVP_MAX_NUM_CANDS (=2)
	 *   [11:8] AMVP_MAX_NUM_CANDS_MEM (=3)
	 *   [15:12] NUM_CHROMA_MODE (=5)
	 *   [23:16] DM_CHROMA_IDX (=36)
	 *   [24]   clk_forceon
	 */
	#define MPRED_CTRL1_AMVP_MAX		(2 << 4)
	#define MPRED_CTRL1_AMVP_MAX_MEM	(3 << 8)
	#define MPRED_CTRL1_NUM_CHROMA		(5 << 12)
	#define MPRED_CTRL1_DM_CHROMA_IDX	(36 << 16)
	#define MPRED_CTRL1_CLK_FORCEON		BIT(24)
#define HEVC_MPRED_INT_EN 0xc80c
#define HEVC_MPRED_INT_STATUS 0xc810
#define HEVC_MPRED_PIC_SIZE 0xc814
#define HEVC_MPRED_PIC_SIZE_LCU 0xc818
#define HEVC_MPRED_TILE_START 0xc81c
#define HEVC_MPRED_TILE_SIZE_LCU 0xc820
#define HEVC_MPRED_REF_NUM 0xc824
/* Long-term reference bitmask, written per-slice by codec_hevc_set_mpred().
 * bits[15:0] = L0 refs, bits[31:16] = L1 refs; a set bit marks that
 * reference index as long-term (excluded from TMVP/AMVP MV scaling).
 * BSP config_mpred_hw() (vh265.c:5916) writes it every slice; upstream
 * left it untouched, so it held a stale power-on value.  See
 * codec_hevc_set_mpred() for the value derivation.
 */
#define HEVC_MPRED_LT_REF 0xc828
#define HEVC_MPRED_REF_EN_L0 0xc830
#define HEVC_MPRED_REF_EN_L1 0xc834
#define HEVC_MPRED_COLREF_EN_L0 0xc838
#define HEVC_MPRED_COLREF_EN_L1 0xc83c
#define HEVC_MPRED_AXI_WCTRL 0xc840
#define HEVC_MPRED_AXI_RCTRL 0xc844
#define HEVC_MPRED_ABV_START_ADDR 0xc848
#define HEVC_MPRED_MV_WR_START_ADDR 0xc84c
#define HEVC_MPRED_MV_RD_START_ADDR 0xc850
#define HEVC_MPRED_MV_WPTR 0xc854
#define HEVC_MPRED_MV_RPTR 0xc858
#define HEVC_MPRED_MV_WR_ROW_JUMP 0xc85c
#define HEVC_MPRED_MV_RD_ROW_JUMP 0xc860
#define HEVC_MPRED_CURR_LCU 0xc864
#define HEVC_MPRED_ABV_WPTR 0xc868
#define HEVC_MPRED_ABV_RPTR 0xc86c
#define HEVC_MPRED_CTRL2 0xc870
#define HEVC_MPRED_CTRL3 0xc874
	/* CTRL3: MPRED constant configuration (BSP literal 0x24122412).
	 * Encodes motion vector precision and search range parameters.
	 * Same value used for both HEVC and VP9.
	 */
	#define MPRED_CTRL3_DEFAULT	0x24122412
#define HEVC_MPRED_L0_REF00_POC 0xc880
#define HEVC_MPRED_L1_REF00_POC 0xc8c0

#define HEVC_MPRED_CTRL4 0xc930

#define HEVC_MPRED_CUR_POC 0xc980
#define HEVC_MPRED_COL_POC 0xc984
#define HEVC_MPRED_MV_RD_END_ADDR 0xc988

#define HEVC_MSP 0xcc00
#define HEVC_MPSR 0xcc04
#define HEVC_MCPU_INTR_MSK 0xcc10
#define HEVC_MCPU_INTR_REQ 0xcc14
#define HEVC_MPC_E 0xcc20	/* 0x3308 << 2 */
#define HEVC_CPSR 0xcc84

#define HEVC_IMEM_DMA_CTRL 0xcd00
#define HEVC_IMEM_DMA_ADR 0xcd04
#define HEVC_IMEM_DMA_COUNT 0xcd08

#define HEVCD_IPP_TOP_CNTL 0xd000
#define HEVCD_IPP_TOP_FRMCONFIG 0xd008	/* [31:16]=height [15:0]=width */
#define HEVCD_IPP_TOP_LCUCONFIG 0xd018	/* LCU size: 16 for H.264 MBs */
#define HEVCD_IPP_LINEBUFF_BASE 0xd024
#define HEVCD_IPP_AXIIF_CONFIG 0xd02c
	/* IPP_AXIIF_CONFIG bit layout:
	 *   [3:0]  endianness: 0xf = little-endian per 2-byte swap
	 *   [9:8]  linear_line_alignment: 2 = 64-byte
	 *   [12]   uv_swap: 1=NV21 (VU order), 0=NV12 (UV order)
	 */
	#define IPP_AXIIF_ENDIAN_MASK	0xf
	#define IPP_AXIIF_ALIGN_64	(2 << 8)
	#define IPP_AXIIF_UV_SWAP	BIT(12)
#define HEVCD_IPP_BITDEPTH_CONFIG 0xd030	/* 0=8-bit luma+chroma */
#define HEVCD_IPP_DYNCLKGATE_CONFIG 0xd080	/* 0x3420 << 2 */

#define VP9D_MPP_REF_SCALE_ENBL		0xd104
#define VP9D_MPP_REFINFO_TBL_ACCCONFIG	0xd108
#define VP9D_MPP_REFINFO_DATA		0xd10c

#define HEVCD_MPP_ANC2AXI_TBL_CONF_ADDR 0xd180
	/* ANC2AXI_TBL_CONF values:
	 *   0:         reset table index to 0 (GXBB write mode)
	 *   BIT(2)|BIT(1): GXL+ auto-increment data write mode
	 *   1:         finalize/lock table after writing entries
	 */
	#define ANC2AXI_CONF_AUTO_INC	(BIT(2) | BIT(1))
#define HEVCD_MPP_ANC2AXI_TBL_CMD_ADDR 0xd184
#define HEVCD_MPP_ANC2AXI_TBL_DATA 0xd190

#define HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR 0xd300
#define HEVCD_MPP_ANC_CANVAS_DATA_ADDR 0xd304
#define HEVCD_MPP_DECOMP_CTL1 0xd308
	/* DECOMP_CTL1 bit layout:
	 *   BIT(4):  MMU mode (G12A+)
	 *   BIT(31): 2-plane NV12 reference mode (non-FBC, non-MMU)
	 *   0:       compressed reference mode (FBC body buffers)
	 */
#define HEVCD_MPP_DECOMP_CTL2 0xd30c
#define HEVCD_MPP_DECOMP_CTL3 0xd310
#define HEVCD_MPP_DECOMP_CTL2 0xd30c
#define HEVCD_MCRCC_CTL1 0xd3c0
	/* MCRCC_CTL1 values (BSP):
	 *   0x2:   reset MCRCC (motion compensation reference cache)
	 *   0xff0: enable progressive-mode MCRCC
	 */
	#define MCRCC_CTL1_RESET	0x2
	#define MCRCC_CTL1_ENABLE	0xff0
#define HEVCD_MCRCC_CTL2 0xd3c4
#define HEVCD_MCRCC_CTL3 0xd3c8

#define HEVC_DBLK_CFG0 0xd400
#define HEVC_DBLK_CFG1 0xd404
	/* CFG1 bit layout (HEVC):
	 *   [1:0]  lcu_size: 0=64, 1=32, 2=16
	 *   [3]    pcm_loop_filter_disabled (when PCM enabled)
	 *   [8:4]  pps_cb_qp_offset (signed, 5-bit)
	 *   [13:9] pps_cr_qp_offset (signed, 5-bit)
	 *   [20]   width_le_64 (set when pic_width <= 64)
	 */
#define HEVC_DBLK_CFG2 0xd408
#define HEVC_DBLK_CFG3 0xd40c
	/* CFG3: SM1 deblock pipeline config.
	 *   0x4010 = pipeline threshold for SM1 DBLK (BSP literal).
	 */
	#define DBLK_CFG3_SM1_PIPELINE	0x4010
#define HEVC_DBLK_CFG4 0xd410
#define HEVC_DBLK_CFG5 0xd414
#define HEVC_DBLK_CFG6 0xd418
#define HEVC_DBLK_CFG7 0xd41c
#define HEVC_DBLK_CFG8 0xd420
#define HEVC_DBLK_CFG9 0xd424
	/* CFG9 bit layout (HEVC per-slice deblocking):
	 *   [0]    loop_filter_across_tiles_enabled
	 *   [1]    loop_filter_across_slices_enabled
	 *   [2]    deblocking_filter_disabled
	 *   [6:3]  slice_beta_offset_div2 (signed, 4-bit)
	 *   [10:7] slice_tc_offset_div2 (signed, 4-bit)
	 */
#define HEVC_DBLK_CFGA 0xd428
#define HEVC_DBLK_STS0 0xd42c
#define HEVC_DBLK_CFGB 0xd42c
	/* CFGB bit layout (G12A+ output mode control):
	 *   [0]    video_format: 0=HEVC, 1=VP9
	 *   [4]    dblk_pipeline_mode (set for width >= 1280)
	 *   [5]    reserved
	 *   [8]    cm_compress_write_enable (FBC body write)
	 *   [9]    sao_double_write_enable (NV12 output)
	 *
	 * SM1 init values (HEVC, no video_format bit):
	 *   [15:14] dw_fifo_thres_r_and_b = 3
	 *   [13:12] dw_fifo_thres_r_or_b = 3
	 *   [11:10] dw_fifo_thres_not_rb = 3
	 *
	 * Pre-G12A init (HEVC): 0x40400000 (BSP literal, legacy pipeline)
	 * G12A init (HEVC): 0x54 << 8 = 0x5400 (BSP literal, pipeline config)
	 */
	#define DBLK_CFGB_VIDEO_FMT_VP9	BIT(0)
	#define DBLK_CFGB_PIPELINE_MODE		BIT(4)
	#define DBLK_CFGB_COMPRESS_WR		BIT(8)
	#define DBLK_CFGB_DOUBLE_WR		BIT(9)
	#define DBLK_CFGB_SM1_DW_FIFO_THRES	((0x3 << 14) | (0x3 << 12) | (0x3 << 10))
	#define DBLK_CFGB_G12A_INIT		(0x54 << 8)
	#define DBLK_CFGB_PRE_G12A_INIT	0x40400000
#define HEVC_DBLK_STS1 0xd430
	/* STS1+4 = 0xd434: G12A+ dblk_main pipeline select.
	 *   BIT(28): select dblk_main for HEVC decode (BSP config_sao_hw).
	 */
#define HEVC_DBLK_CFGE 0xd438

#define HEVC_SAO_VERSION 0xd800
#define HEVC_SAO_CTRL0 0xd804
	/* CTRL0 bit layout:
	 *   [3:0]  lcu_size_log2
	 *   [8]    loop_filter_across_tiles_enabled
	 *   [9]    loop_filter_across_slices_enabled
	 */
#define HEVC_SAO_CTRL1 0xd808
	/* CTRL1 bit layout:
	 *   [0]    cm_disable (1=compressed mode off, double-write only)
	 *   [1]    dw_disable (1=double-write off, compressed only)
	 *   [2]    clk_forceon (CONFIG_HEVC_CLK_FORCED_ON)
	 *   [11:4] endianness: 0xff0 = little-endian per 2-byte swap
	 *   [15:14] axi_linealign: 2 = 64-byte alignment
	 */
	#define SAO_CTRL1_CM_DISABLE	BIT(0)
	#define SAO_CTRL1_DW_DISABLE	BIT(1)
	#define SAO_CTRL1_CLK_FORCEON	BIT(2)
	#define SAO_CTRL1_ENDIAN_MASK	0xff0
	#define SAO_CTRL1_AXI_ALIGN_64	(2 << 14)
#define HEVC_SAO_PIC_SIZE 0xd814
#define HEVC_SAO_PIC_SIZE_LCU 0xd818
#define HEVC_SAO_TILE_START 0xd81c
#define HEVC_SAO_TILE_SIZE_LCU 0xd820
#define HEVC_SAO_Y_START_ADDR 0xd82c
#define HEVC_SAO_Y_LENGTH 0xd830
#define HEVC_SAO_C_START_ADDR 0xd834
#define HEVC_SAO_C_LENGTH 0xd838
#define HEVC_SAO_Y_WPTR 0xd83c
#define HEVC_SAO_C_WPTR 0xd840
#define HEVC_SAO_ABV_START_ADDR 0xd844
#define HEVC_SAO_VB_WR_START_ADDR 0xd848
#define HEVC_SAO_VB_RD_START_ADDR 0xd84c
#define HEVC_SAO_ABV_WPTR 0xd850
#define HEVC_SAO_ABV_RPTR 0xd854
#define HEVC_SAO_VB_WPTR 0xd858
#define HEVC_SAO_VB_RPTR 0xd85c
#define HEVC_SAO_CTRL2 0xd880
#define HEVC_SAO_CTRL3 0xd884
#define HEVC_SAO_CTRL4 0xd888
#define HEVC_SAO_CTRL5 0xd88c
	/* CTRL5 bit layout:
	 *   [7]     cb_chroma_rounding_workaround (G12A+, BSP workaround_enable & 1)
	 *   [9]     mem_saving_mode (8-bit FBC uses 3200 bytes/block vs 4096)
	 *   [10]    use_cm_header_start_addr (MMU mode)
	 *   [17]    FBC_body_uncouple (BSP)
	 *   [23:16] dw_scale (0=no downscale, full-res NV12 output)
	 *
	 * Masks used in code:
	 *   ~0xff0200: clear FBC_body_uncouple(17) + endian bits(11..4 partial)
	 *              BSP: reset for FBC mode setup
	 *   ~0xff0000: clear dw_scale[23:16] (set to 0 = no downscale)
	 */
	#define SAO_CTRL5_CB_ROUND_WA	BIT(7)
	#define SAO_CTRL5_MEM_SAVING	BIT(9)
	#define SAO_CTRL5_USE_CM_HDR	BIT(10)
	#define SAO_CTRL5_DW_SCALE_MASK	0xff0000
#define HEVC_SAO_CTRL6 0xd890
#define HEVC_SAO_CTRL7 0xd894
#define HEVC_CM_BODY_START_ADDR 0xd898
#define HEVC_CM_BODY_LENGTH 0xd89c
#define HEVC_CM_HEADER_START_ADDR 0xd8a0
#define HEVC_CM_HEADER_LENGTH 0xd8a4
#define HEVC_CM_HEADER_OFFSET 0xd8ac
#define HEVC_SAO_CTRL8 0xd8b0
#define HEVC_SAO_CTRL9 0xd8b4
#define HEVC_SAO_MMU_VH0_ADDR 0xd8e8
#define HEVC_SAO_MMU_VH1_ADDR 0xd8ec

#define HEVC_IQIT_CLK_RST_CTRL 0xdc00
#define HEVC_IQIT_SCALELUT_WR_ADDR 0xdc08
#define HEVC_IQIT_SCALELUT_RD_ADDR 0xdc0c
#define HEVC_IQIT_SCALELUT_DATA 0xdc10

#define HEVC_LMEM_DMA_CTRL 0xcd40
#define HEVC_WRRSP_LMEM 0xcd4c
	/* Bits 11:0: pending LMEM write response count */
	#define HEVC_WRRSP_LMEM_PENDING_MASK	0xfff
#define HEVC_CORE_ENABLE 0xfd58

#define HEVC_PSCALE_CTRL 0xe444

#define HEVCD_MPP_VDEC_MCR_CTL 0xe720

#endif
