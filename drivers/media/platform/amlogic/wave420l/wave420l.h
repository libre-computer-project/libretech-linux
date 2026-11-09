/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Chips&Media Wave420L HEVC encoder driver for Amlogic SoCs
 *
 * Copyright (C) 2025 Libre Computer
 */

#ifndef __WAVE420L_H__
#define __WAVE420L_H__

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/semaphore.h>
#include <linux/types.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-mem2mem.h>

/* ------------------------------------------------------------------ */
/* Driver-specific V4L2 controls                                      */
/* ------------------------------------------------------------------ */
#define WAVE420L_CID_BASE		(V4L2_CID_USER_BASE + 0x2000)
#define WAVE420L_CID_INTRA_REFRESH_MODE	(WAVE420L_CID_BASE + 0)
#define WAVE420L_CID_LONG_TERM_REF_ENABLE (WAVE420L_CID_BASE + 1)
#define WAVE420L_CID_SCALING_LIST	 (WAVE420L_CID_BASE + 2)
#define WAVE420L_CID_COLOUR_PRIMARIES	 (WAVE420L_CID_BASE + 3)
#define WAVE420L_CID_TRANSFER_CHARS	 (WAVE420L_CID_BASE + 4)
#define WAVE420L_CID_MATRIX_COEFFS	 (WAVE420L_CID_BASE + 5)
#define WAVE420L_CID_VIDEO_FULL_RANGE	 (WAVE420L_CID_BASE + 6)
#define WAVE420L_CID_SEI_DATA		 (WAVE420L_CID_BASE + 7)
/* Temporal layer QP controls (UND-TEMPORAL-QP) */
#define WAVE420L_CID_TEMPORAL_LAYER_QP_0 (WAVE420L_CID_BASE + 8)
#define WAVE420L_CID_TEMPORAL_LAYER_QP_1 (WAVE420L_CID_BASE + 9)
#define WAVE420L_CID_TEMPORAL_LAYER_QP_2 (WAVE420L_CID_BASE + 10)
/* Per-frame force picture type (UND-FORCE-PIC) */
#define WAVE420L_CID_FORCE_PIC_TYPE	 (WAVE420L_CID_BASE + 11)
/* Lossless encoding (UND-LOSSLESS): sets PIC_PARAM[20], disables RC */
#define WAVE420L_CID_LOSSLESS		 (WAVE420L_CID_BASE + 12)
/* ROI encoding and CTU QP map (UND-ROI-CTU) */
#define WAVE420L_CID_ROI_ENABLE		 (WAVE420L_CID_BASE + 13)
#define WAVE420L_CID_ROI_DELTA_QP	 (WAVE420L_CID_BASE + 14)
#define WAVE420L_CID_CTU_QP_ENABLE	 (WAVE420L_CID_BASE + 15)
/* Custom GOP controls (UND-CUSTOM-GOP) */
#define WAVE420L_CID_CUSTOM_GOP_ENABLE	 (WAVE420L_CID_BASE + 16)
#define WAVE420L_CID_CUSTOM_GOP_PARAM	 (WAVE420L_CID_BASE + 17)

/* ------------------------------------------------------------------ */
/* Wave4 host register offsets (relative to register base)            */
/* ------------------------------------------------------------------ */

#define W4_REG_BASE			0x0000

/* System / control */
#define W4_PO_CONF			(W4_REG_BASE + 0x0000)
#define W4_VPU_VINT_REASON		(W4_REG_BASE + 0x0030)
#define W4_VPU_VINT_REASON_USR		(W4_REG_BASE + 0x0030)
#define W4_VPU_VINT_REASON_CLR		(W4_REG_BASE + 0x0034)
#define W4_VPU_VINT_CLEAR		(W4_REG_BASE + 0x003C)
#define W4_VPU_VINT_ENABLE		(W4_REG_BASE + 0x0048)
#define W4_VPU_INT_REASON		(W4_REG_BASE + 0x004C)

#define W4_VPU_RESET_REQ		(W4_REG_BASE + 0x0050)
#define W4_VPU_RESET_STATUS		(W4_REG_BASE + 0x0054)

#define W4_VPU_REMAP_CTRL		(W4_REG_BASE + 0x0060)
#define W4_VPU_REMAP_VADDR		(W4_REG_BASE + 0x0064)
#define W4_VPU_REMAP_PADDR		(W4_REG_BASE + 0x0068)
#define W4_VPU_REMAP_CORE_START		(W4_REG_BASE + 0x006C)
#define W4_VPU_BUSY_STATUS		(W4_REG_BASE + 0x0070)

#define W4_RET_FW_VERSION		(W4_REG_BASE + 0x0104)
#define W4_RET_PRODUCT_VERSION		(W4_REG_BASE + 0x0108)

/* Host interrupt request (trigger VPU to process command) */
#define W4_VPU_HOST_INT_REQ		(W4_REG_BASE + 0x0038)

/* Command interface */
#define W4_COMMAND			(W4_REG_BASE + 0x0100)
#define W4_CORE_INDEX			(W4_REG_BASE + 0x0104)
#define W4_INST_INDEX			(W4_REG_BASE + 0x0108)
#define W4_RET_SUCCESS			(W4_REG_BASE + 0x0110)
#define W4_RET_FAIL_REASON		(W4_REG_BASE + 0x0114)

/* Firmware loading */
#define W4_ADDR_CODE_BASE		(W4_REG_BASE + 0x0118)
#define W4_CODE_SIZE			(W4_REG_BASE + 0x011C)
#define W4_CODE_PARAM			(W4_REG_BASE + 0x0120)
#define W4_HW_OPTION			(W4_REG_BASE + 0x0124)
#define W4_TIMEOUT_CNT			(W4_REG_BASE + 0x0134)

/* Bitstream buffer */
#define W4_BS_START_ADDR		(W4_REG_BASE + 0x0120)
#define W4_BS_SIZE			(W4_REG_BASE + 0x0124)
#define W4_BS_PARAM			(W4_REG_BASE + 0x0128)
#define W4_BS_OPTION			(W4_REG_BASE + 0x012C)
#define W4_BS_RD_PTR			(W4_REG_BASE + 0x0130)
#define W4_BS_WR_PTR			(W4_REG_BASE + 0x0134)

/* Work buffer */
#define W4_ADDR_WORK_BASE		(W4_REG_BASE + 0x0138)
#define W4_WORK_SIZE			(W4_REG_BASE + 0x013C)
#define W4_WORK_PARAM			(W4_REG_BASE + 0x0140)

/* Temp buffer */
#define W4_ADDR_TEMP_BASE		(W4_REG_BASE + 0x0144)
#define W4_TEMP_SIZE			(W4_REG_BASE + 0x0148)
#define W4_TEMP_PARAM			(W4_REG_BASE + 0x014C)

/* Secondary AXI */
#define W4_ADDR_SEC_AXI			(W4_REG_BASE + 0x0150)
#define W4_SEC_AXI_SIZE			(W4_REG_BASE + 0x0154)
#define W4_USE_SEC_AXI			(W4_REG_BASE + 0x0158)

/* ------------------------------------------------------------------ */
/* Encoder SET_PARAM registers                                        */
/* ------------------------------------------------------------------ */
#define W4_CMD_ENC_SET_PARAM_OPTION	(W4_REG_BASE + 0x010C)
#define W4_CMD_ENC_SET_PARAM_ENABLE	(W4_REG_BASE + 0x015C)
#define W4_CMD_ENC_SEQ_SRC_SIZE		(W4_REG_BASE + 0x0160)
#define W4_CMD_ENC_SEQ_PIC_PARAM	(W4_REG_BASE + 0x016C)
#define W4_CMD_ENC_SEQ_GOP_PARAM	(W4_REG_BASE + 0x0170)
#define W4_CMD_ENC_SEQ_INTRA_PARAM	(W4_REG_BASE + 0x0174)
#define W4_CMD_ENC_SEQ_CONF_WIN_TOP_BOT	(W4_REG_BASE + 0x0178)
#define W4_CMD_ENC_SEQ_CONF_WIN_LR	(W4_REG_BASE + 0x017C)
#define W4_CMD_ENC_SEQ_FRAME_RATE	(W4_REG_BASE + 0x0180)
#define W4_CMD_ENC_SEQ_INDEP_SLICE	(W4_REG_BASE + 0x0184)
#define W4_CMD_ENC_SEQ_DEP_SLICE	(W4_REG_BASE + 0x0188)
#define W4_CMD_ENC_SEQ_INTRA_REFRESH	(W4_REG_BASE + 0x018C)
#define W4_CMD_ENC_PARAM		(W4_REG_BASE + 0x0190)
#define W4_CMD_ENC_SEQ_LAYER_PERIOD	(W4_REG_BASE + 0x0194)
#define W4_CMD_ENC_RC_PARAM		(W4_REG_BASE + 0x0198)
#define W4_CMD_ENC_RC_MIN_MAX_QP	(W4_REG_BASE + 0x019C)
/*
 * W4_CMD_ENC_RC_INTRA_MIN_MAX_QP shares offset 0x0194 with
 * W4_CMD_ENC_SEQ_LAYER_PERIOD.  During SET_PARAM initial setup both are
 * in the same register window; the last write before the command wins.
 * For WAVE420L we don't use temporal layers (LAYER_PERIOD=0), so we
 * write intra min/max QP last to claim this register.
 */
#define W4_CMD_ENC_RC_INTRA_MIN_MAX_QP	(W4_REG_BASE + 0x0194)
#define W4_CMD_ENC_RC_BIT_RATIO_LAYER_0_3 (W4_REG_BASE + 0x01A0)
#define W4_CMD_ENC_RC_BIT_RATIO_LAYER_4_7 (W4_REG_BASE + 0x01A4)
#define W4_CMD_ENC_NR_PARAM		(W4_REG_BASE + 0x01A8)
#define W4_CMD_ENC_NR_WEIGHT		(W4_REG_BASE + 0x01AC)
#define W4_CMD_ENC_NUM_UNITS_IN_TICK	(W4_REG_BASE + 0x01B0)
#define W4_CMD_ENC_TIME_SCALE		(W4_REG_BASE + 0x01B4)
#define W4_CMD_ENC_NUM_TICKS_POC_DIFF_ONE (W4_REG_BASE + 0x01B8)
#define W4_CMD_ENC_RC_TRANS_RATE	(W4_REG_BASE + 0x01BC)
#define W4_CMD_ENC_RC_TARGET_RATE	(W4_REG_BASE + 0x01C0)
#define W4_CMD_ENC_ROT_PARAM		(W4_REG_BASE + 0x01C4)
#define W4_RET_ENC_MIN_FB_NUM		(W4_REG_BASE + 0x01CC)
#define W4_RET_ENC_MIN_SRC_BUF_NUM	(W4_REG_BASE + 0x01D8)

/* ------------------------------------------------------------------ */
/* Encoder ENC_PIC registers                                          */
/* ------------------------------------------------------------------ */
#define W4_CMD_ENC_ADDR_REPORT_BASE	(W4_REG_BASE + 0x015C)
#define W4_CMD_ENC_REPORT_SIZE		(W4_REG_BASE + 0x0160)
#define W4_CMD_ENC_REPORT_PARAM		(W4_REG_BASE + 0x0164)
#define W4_CMD_ENC_CODE_OPTION		(W4_REG_BASE + 0x0168)
#define W4_CMD_ENC_PIC_PARAM		(W4_REG_BASE + 0x016C)
#define W4_CMD_ENC_SRC_PIC_IDX		(W4_REG_BASE + 0x0170)
#define W4_CMD_ENC_SRC_ADDR_Y		(W4_REG_BASE + 0x0174)
#define W4_CMD_ENC_SRC_ADDR_U		(W4_REG_BASE + 0x0178)
#define W4_CMD_ENC_SRC_ADDR_V		(W4_REG_BASE + 0x017C)
#define W4_CMD_ENC_SRC_STRIDE		(W4_REG_BASE + 0x0180)
#define W4_CMD_ENC_SRC_FORMAT		(W4_REG_BASE + 0x0184)
/* ENC_PIC registers at shared offsets (reused per command) */
#define W4_CMD_ENC_PREFIX_SEI_NAL_ADDR	(W4_REG_BASE + 0x0188)
#define W4_CMD_ENC_PREFIX_SEI_INFO	(W4_REG_BASE + 0x018C)
#define W4_CMD_ENC_SUFFIX_SEI_NAL_ADDR	(W4_REG_BASE + 0x0190)
#define W4_CMD_ENC_SUFFIX_SEI_INFO	(W4_REG_BASE + 0x0194)
#define W4_CMD_ENC_LONGTERM_PIC		(W4_REG_BASE + 0x0198)
#define W4_CMD_ENC_SUB_FRAME_SYNC_CONFIG (W4_REG_BASE + 0x019C)
#define W4_CMD_ENC_CTU_OPT_PARAM	(W4_REG_BASE + 0x01A0)
#define W4_CMD_ENC_ROI_ADDR_CTU_MAP	(W4_REG_BASE + 0x01A4)
#define W4_CMD_ENC_CTU_QP_MAP_ADDR	(W4_REG_BASE + 0x01AC)
#define W4_CMD_ENC_SRC_TIMESTAMP_LOW	(W4_REG_BASE + 0x01B0)
#define W4_CMD_ENC_SRC_TIMESTAMP_HIGH	(W4_REG_BASE + 0x01B4)

/* Encoder ENC_PIC return values */
#define W4_RET_ENC_PIC_TYPE		(W4_REG_BASE + 0x01E0)
#define W4_RET_ENC_PIC_BYTE		(W4_REG_BASE + 0x01C8)
#define W4_RET_ENC_PIC_IDX		(W4_REG_BASE + 0x01A8)
#define W4_RET_ENC_PIC_SLICE_NUM	(W4_REG_BASE + 0x01AC)
#define W4_RET_ENC_PIC_SKIP		(W4_REG_BASE + 0x01B0)
#define W4_RET_ENC_PIC_FLAG		(W4_REG_BASE + 0x01BC)
#define W4_RET_ENC_PIC_NUM_INTRA	(W4_REG_BASE + 0x01B4)
#define W4_RET_ENC_PIC_NUM_MERGE	(W4_REG_BASE + 0x01B8)
#define W4_RET_ENC_PIC_NUM_SKIP		(W4_REG_BASE + 0x01C0)
#define W4_RET_ENC_PIC_AVG_CU_QP	(W4_REG_BASE + 0x01C4)
#define W4_RET_ENC_GOP_PIC_IDX		(W4_REG_BASE + 0x01CC)
#define W4_RET_ENC_USED_SRC_IDX		(W4_REG_BASE + 0x01D8)
#define W4_RET_ENC_PIC_NUM		(W4_REG_BASE + 0x01DC)
#define W4_RET_ENC_VCL_NUT		(W4_REG_BASE + 0x01E4)

/* SET_FRAMEBUF command registers (shared cmd register space) */
#define W4_SFB_OPTION			(W4_REG_BASE + 0x010C)
#define W4_COMMON_PIC_INFO		(W4_REG_BASE + 0x0120)
#define W4_PIC_SIZE			(W4_REG_BASE + 0x0124)
#define W4_SET_FB_NUM			(W4_REG_BASE + 0x0128)
#define W4_FBC_STRIDE			(W4_REG_BASE + 0x0154)
#define W4_ADDR_SUB_SAMPLED_FB_BASE	(W4_REG_BASE + 0x0158)
#define W4_SUB_SAMPLED_ONE_FB_SIZE	(W4_REG_BASE + 0x015C)
#define W4_ADDR_LUMA_BASE0		(W4_REG_BASE + 0x0160)
#define W4_ADDR_CB_BASE0		(W4_REG_BASE + 0x0164)
#define W4_ADDR_CR_BASE0		(W4_REG_BASE + 0x0168)
#define W4_ADDR_FBC_Y_OFFSET0		(W4_REG_BASE + 0x0168)
#define W4_ADDR_FBC_C_OFFSET0		(W4_REG_BASE + 0x016C)
#define W4_ADDR_MV_COL0			(W4_REG_BASE + 0x01E0)

/* ------------------------------------------------------------------ */
/* Sleep/Wake VPU registers (from common_regdefine.h)                 */
/* ------------------------------------------------------------------ */
#define W4_RET_CUR_SP			(W4_REG_BASE + 0x0124)
#define W4_SET_CUR_SP			(W4_REG_BASE + 0x0120)

/* ------------------------------------------------------------------ */
/* Wave4 commands (from common_regdefine.h)                           */
/* ------------------------------------------------------------------ */
#define W4_CMD_INIT_VPU			0x0001
#define W4_CMD_INIT_SEQ			0x0002	/* DEC_PIC_HDR / SET_PARAM */
#define W4_CMD_SET_PARAM		0x0002
#define W4_CMD_ENC_PIC			0x0008
#define W4_CMD_SET_FB			0x0010	/* SET_FRAMEBUF */
#define W4_CMD_FLUSH_INSTANCE		0x0020	/* FLUSH_DECODER */
#define W4_CMD_QUERY			0x0200	/* QUERY_DECODER */
#define W4_CMD_SLEEP_VPU		0x0400
#define W4_CMD_WAKEUP_VPU		0x0800
#define W4_CMD_CREATE_INSTANCE		0x4000
#define W4_CMD_DESTROY_INSTANCE		0x0004	/* FINI_SEQ */

/* Interrupt bit flags (from SDK common_regdefine.h) */
#define W4_INT_INIT_VPU			0
#define W4_INT_DEC_PIC_HDR		1
#define W4_INT_SET_PARAM		1
#define W4_INT_ENC_INIT_SEQ		1
#define W4_INT_FINI_SEQ			2
#define W4_INT_DEC_PIC			3
#define W4_INT_ENC_PIC			3
#define W4_INT_SET_FRAMEBUF		4
#define W4_INT_FLUSH_DEC		5
#define W4_INT_ENC_SLICE_INT		7
#define W4_INT_GET_FW_VERSION		8
#define W4_INT_QUERY_DEC		9
#define W4_INT_SLEEP_VPU		10
#define W4_INT_WAKEUP_VPU		11
#define W4_INT_CREATE_INSTANCE		14

/* Codec modes (upper 16 bits of W4_INST_INDEX) */
#define W4_CODEC_MODE_HEVC_DEC		0
#define W4_CODEC_MODE_HEVC_ENC		1

/* SET_PARAM_OPTION values */
#define W4_OPT_COMMON			0
#define W4_OPT_CUSTOM_GOP		1
#define W4_OPT_CUSTOM_HEADER		2
#define W4_OPT_VUI			3
#define W4_OPT_CHANGE_PARAM		0x10

/* ------------------------------------------------------------------ */
/* Encoder SET_PARAM registers (used with OPT_CUSTOM_GOP)             */
/* ------------------------------------------------------------------ */
#define W4_CMD_ENC_SET_CUSTOM_GOP_ENABLE  (W4_REG_BASE + 0x015C)
#define W4_CMD_ENC_CUSTOM_GOP_PARAM       (W4_REG_BASE + 0x0160)
#define W4_CMD_ENC_CUSTOM_GOP_PIC_PARAM_0 (W4_REG_BASE + 0x0164)
/* Entries 1-7 are at PIC_PARAM_0 + i*4 (i=1..7) */
#define W4_CMD_ENC_CUSTOM_GOP_PIC_LAMBDA_0 (W4_REG_BASE + 0x0188)
/* Entries 1-7 are at PIC_LAMBDA_0 + i*4 (i=1..7) */

/*
 * Custom GOP picture register bit layout (SDK wave4.c:923-928):
 *   picType[1:0]    | pocOffset[7:2]   | picQp[13:8]    | useMultiRefP[14]
 *   | refPocL0[18:14] | refPocL1[23:19] | temporalId[26:24]
 * Note: refPocL0/L1 are signed 5-bit values masked with 0x1F.
 * useDeriveLambdaWeight is bit [4] of W4_CMD_ENC_CUSTOM_GOP_PARAM (gopSize[3:0]).
 */

/* ------------------------------------------------------------------ */
/* Encoder SET_PARAM VUI registers (used with OPT_VUI)                */
/* ------------------------------------------------------------------ */
#define W4_CMD_ENC_VUI_PARAM_FLAGS	(W4_REG_BASE + 0x015C)
#define W4_CMD_ENC_VUI_ASPECT_RATIO_IDC	(W4_REG_BASE + 0x0160)
#define W4_CMD_ENC_VUI_SAR_SIZE		(W4_REG_BASE + 0x0164)
#define W4_CMD_ENC_VUI_OVERSCAN_APPROPRIATE (W4_REG_BASE + 0x0168)
#define W4_CMD_ENC_VUI_VIDEO_SIGNAL	(W4_REG_BASE + 0x016C)
#define W4_CMD_ENC_VUI_CHROMA_SAMPLE_LOC (W4_REG_BASE + 0x0170)
#define W4_CMD_ENC_VUI_DISP_WIN_LEFT_RIGHT (W4_REG_BASE + 0x0174)
#define W4_CMD_ENC_VUI_DISP_WIN_TOP_BOT	(W4_REG_BASE + 0x0178)
#define W4_CMD_ENC_VUI_HRD_RBSP_PARAM_FLAG (W4_REG_BASE + 0x017C)

/* SET_PARAM_ENABLE bitmask: which parameters changed (SDK: ChangeCommonParam) */
#define W4_ENC_INTRA_PARAM_CHANGE	BIT(3)
#define W4_ENC_FRAME_RATE_CHANGE	BIT(6)
#define W4_ENC_RC_PARAM_CHANGE		BIT(12)
#define W4_ENC_RC_MIN_MAX_QP_CHANGE	BIT(13)
#define W4_ENC_RC_TRANS_RATE_CHANGE	BIT(21)
#define W4_ENC_RC_TARGET_RATE_CHANGE	BIT(22)

/* SET_PARAM_ENABLE: enable all parameter changes */
#define W4_ENC_CHANGE_SET_PARAM_ALL	0xFFFFFFFF

/* Encoder source format values */
#define W4_ENC_SRC_FORMAT_420		0
#define W4_ENC_SRC_FORMAT_NV12		0
#define W4_ENC_SRC_FORMAT_NV21		1
#define W4_ENC_SRC_FORMAT_I420		2

/* REMAP control value construction */
#define W4_REMAP_MAX_SIZE		0x8000
#define W4_REMAP_CTRL_VALUE(idx, size) \
	(0x80000000 | ((idx) << 12) | (0 << 16) | BIT(11) | \
	 (((size) >> 12) & 0x1ff))

/* ------------------------------------------------------------------ */
/* Amlogic SoC integration registers                                  */
/* ------------------------------------------------------------------ */

/* DOS register offsets (word addresses, need <<2 for byte offset) */
#define DOS_SW_RESET4			0x3F37
#define DOS_MEM_PD_WAVE420L		0x3F39
#define DOS_WAVE420L_CNTL_STAT		0x3F3A

/* HHI register offsets (word addresses, relative to HIU bus base) */
#define HHI_WAVE420L_CLK_CNTL		0x9A
#define HHI_WAVE420L_CLK_CNTL2		0x9B

/* AO register byte offsets (relative to AO bus base) */
#define AO_RTI_GEN_PWR_SLEEP0		0x00E8
#define AO_RTI_GEN_PWR_ISO0		0x00EC

/* SM1 power bits */
#define SM1_WAVE420L_PWR_SLEEP_BIT	BIT(3)
#define SM1_WAVE420L_PWR_ISO_BIT	BIT(3)
/* G12A/G12B power bits */
#define G12A_WAVE420L_PWR_SLEEP_BITS	(BIT(24) | BIT(25))
#define G12A_WAVE420L_PWR_ISO_BITS	(BIT(12) | BIT(13))

/* SM1 reset bits */
#define SM1_WAVE420L_RESET_BITS		0x0F00

/* Clock level 6 = 667MHz from fclk_div3 (BSP default is L4=400MHz, but L6 is tested working) */
#define WAVE420L_CLK_L6			((2 << 25) | (0 << 16) | (2 << 9) | (0 << 0))
#define WAVE420L_CLK_GATE_BITS		(BIT(8) | BIT(24))

/* ------------------------------------------------------------------ */
/* Driver structures                                                  */
/* ------------------------------------------------------------------ */

/* ------------------------------------------------------------------ */
/* Custom GOP structures (UND-CUSTOM-GOP)                             */
/* ------------------------------------------------------------------ */

#define WAVE420L_MAX_CUSTOM_GOP_SIZE	8

/*
 * Per-picture entry for a custom GOP definition.
 *
 * Bit layout written to W4_CMD_ENC_CUSTOM_GOP_PIC_PARAM_N (SDK wave4.c:923):
 *   picType[1:0] | pocOffset<<2 | picQp<<6 | refPocL0<<14 | refPocL1<<19
 *   | temporalId<<24
 * refPocL0/L1 are signed values stored masked with 0x1F (5-bit two's complement).
 */
struct wave420l_custom_gop_pic {
	u8 pic_type;	/* 0=I, 1=P, 2=B */
	u8 poc_offset;	/* display order offset from GOP start (1..gopSize) */
	u8 pic_qp;	/* QP for this picture (0=use sequence QP) */
	u8 temporal_id;	/* temporal layer ID (0-6) */
	s8 ref_poc_l0;	/* L0 reference POC offset (signed) */
	s8 ref_poc_l1;	/* L1 reference POC offset (signed, 0 if none) */
} __packed;

/*
 * Top-level custom GOP parameter blob.
 * Userspace sets this via WAVE420L_CID_CUSTOM_GOP_PARAM.
 * gop_size=0: driver uses built-in IPPP default.
 */
struct wave420l_custom_gop_param {
	u8 gop_size;	/* 1-8 pictures per GOP cycle */
	u8 use_derive_lambda; /* 1 = firmware derives lambda weights */
	u8 pad[2];
	struct wave420l_custom_gop_pic pics[WAVE420L_MAX_CUSTOM_GOP_SIZE];
} __packed;

#define WAVE420L_FW_NAME		"meson/wave420l/monet.bin"
#define WAVE420L_TIMEOUT_MS		2000
#define WAVE420L_MAX_INSTANCES		3
#define WAVE420L_CODE_BUF_SIZE		(1024 * 1024)	/* 1MB code space (SDK: WAVE4_MAX_CODE_BUF_SIZE) */
#define WAVE420L_WORK_BUF_SIZE		(512 * 1024)
#define WAVE420L_TEMP_BUF_SIZE		(256 * 1024)
#define WAVE420L_BS_BUF_SIZE_HW	(2 * 1024 * 1024)  /* Ring buffer for firmware BS output */
#define WAVE420L_SEI_BUF_SIZE		(8 * 1024)	/* DMA buffer for SEI NAL data */

/* Minimum / maximum resolution */
#define WAVE420L_MIN_WIDTH		256
#define WAVE420L_MIN_HEIGHT		128
#define WAVE420L_MAX_WIDTH		4096
#define WAVE420L_MAX_HEIGHT		2304

struct wave420l_dev;

enum wave420l_inst_state {
	WAVE420L_STATE_NONE,
	WAVE420L_STATE_OPEN,
	WAVE420L_STATE_INIT_SEQ,
	WAVE420L_STATE_PIC_RUN,
	WAVE420L_STATE_STOP,
};

struct wave420l_buf {
	dma_addr_t dma;
	void *virt;
	size_t size;
};

struct wave420l_instance {
	struct list_head list;
	struct v4l2_fh fh;
	struct v4l2_ctrl_handler ctrl_handler;
	struct wave420l_dev *dev;
	struct v4l2_m2m_dev *m2m_dev;
	struct completion irq_done;

	s32 id;
	enum wave420l_inst_state state;

	/* Format state */
	struct v4l2_pix_format_mplane src_fmt;
	struct v4l2_pix_format_mplane dst_fmt;

	/* Encoder parameters (from V4L2 controls) */
	u32 bitrate;
	u32 gop_size;
	u32 min_qp;
	u32 max_qp;
	u32 init_qp;
	u32 profile;
	u32 level;
	u32 tier;
	u32 frame_rate;
	u32 gop_preset;
	bool rc_enable;
	bool cu_rc_enable;
	bool force_key_frame;
	u32 bitrate_mode;	/* V4L2_MPEG_VIDEO_BITRATE_MODE_* */
	u32 vbv_size;		/* VBV buffer size in bits */
	u32 i_min_qp;
	u32 i_max_qp;
	bool prepend_spspps;
	u32 params_changed;	/* bitmask of W4_ENC_*_CHANGE flags */

	/*
	 * KI-092: cached VPS+SPS+PPS Annex-B bytes emitted by the firmware
	 * via the standalone ENC_PUT_VIDEO_HEADER command (CODE_OPTION =
	 * CODEOPT_ENC_VPS|SPS|PPS = 0x1C, no VCL bit) at start_streaming
	 * time.  Prepended to the first CAPTURE buffer in
	 * wave420l_enc_irq_complete so the output bitstream is self-
	 * contained for downstream decoders (ffprobe was reporting
	 * width=0 height=0 profile=unknown without this).
	 */
	u8 header_nal[256];
	u32 header_nal_len;

	/* Hardware rotation/mirror */
	u32 rotation;		/* 0, 90, 180, 270 */
	bool hflip;
	bool vflip;

	/* Loop filter controls */
	u32 lf_mode;		/* V4L2_CID_MPEG_VIDEO_HEVC_LOOP_FILTER_MODE */
	s32 lf_beta_offset;	/* V4L2_CID_MPEG_VIDEO_HEVC_LF_BETA_OFFSET_DIV2 */
	s32 lf_tc_offset;	/* V4L2_CID_MPEG_VIDEO_HEVC_LF_TC_OFFSET_DIV2 */

	/* B-frame control */
	u32 b_frames;		/* V4L2_CID_MPEG_VIDEO_B_FRAMES */
	u32 bframe_delay;	/* frames buffered in encoder pipeline (0, 1, or 3) */

	/* Intra refresh controls */
	u32 refresh_type;	/* V4L2_CID_MPEG_VIDEO_HEVC_REFRESH_TYPE */
	u32 refresh_period;	/* V4L2_CID_MPEG_VIDEO_HEVC_REFRESH_PERIOD */

	/* Per-picture QP override */
	u32 p_frame_qp;	/* V4L2_CID_MPEG_VIDEO_HEVC_P_FRAME_QP (0=disabled) */
	u32 b_frame_qp;	/* V4L2_CID_MPEG_VIDEO_HEVC_B_FRAME_QP (0=disabled) */

	/* Encoding structure controls */
	u32 general_pb;		/* V4L2_CID_MPEG_VIDEO_HEVC_GENERAL_PB */
	u32 cu_size_mode;	/* CU size bitmask: bit0=8x8, bit1=16x16, bit2=32x32 */

	/* Chroma QP offset controls (PPS-level, HEVC spec range: -12..+12) */
	s32 chroma_cb_qp_offset;	/* pps_cb_qp_offset */
	s32 chroma_cr_qp_offset;	/* pps_cr_qp_offset */

	/* Scaling list enable (firmware handles matrix internally) */
	bool scaling_list_enable;

	/* Lossless encoding: PIC_PARAM[20]=losslessEnable; requires rc_enable=0 */
	bool lossless;

	/* Slice encoding: independent slice mode/arg for SET_PARAM */
	u16 slice_mode;		/* 0=no slice, 1=N CTUs per slice */
	u16 slice_arg;		/* CTU count per slice (used when slice_mode=1) */

	/* Noise reduction controls (W4_CMD_ENC_NR_PARAM / W4_CMD_ENC_NR_WEIGHT) */
	u32 nr_y_enable;	/* enable NR for Y component */
	u32 nr_cb_enable;	/* enable NR for Cb component */
	u32 nr_cr_enable;	/* enable NR for Cr component */
	u32 nr_noise_est_enable; /* enable HW noise estimation (auto sigma) */
	u32 nr_noise_sigma_y;	/* Y noise sigma (0-255, used when est_enable=0) */
	u32 nr_noise_sigma_cb;	/* Cb noise sigma (0-255) */
	u32 nr_noise_sigma_cr;	/* Cr noise sigma (0-255) */
	u32 nr_intra_weight_y;	/* SDK default: 7 */
	u32 nr_intra_weight_cb;	/* SDK default: 7 */
	u32 nr_intra_weight_cr;	/* SDK default: 7 */
	u32 nr_inter_weight_y;	/* SDK default: 4 */
	u32 nr_inter_weight_cb;	/* SDK default: 4 */
	u32 nr_inter_weight_cr;	/* SDK default: 4 */

	/* Motion estimation and intra refresh controls */
	u32 max_num_merge;	/* maxNumMerge (0-4, ENC_PARAM[11:9]) */
	bool tmv_prediction;	/* tmvpEnable (ENC_PARAM[7]) */
	u32 intra_refresh_mode;	/* 0=none, 1=row, 2=col, 3=step */
	u32 intra_refresh_num;	/* CTUs per refresh step */
	bool long_term_ref_enable; /* useLongTerm (PIC_PARAM[23]) */

	/* VUI parameters (colour_primaries, transfer_characteristics, matrix_coefficients) */
	u8 colour_primaries;
	u8 transfer_characteristics;
	u8 matrix_coefficients;
	bool video_full_range;
	u8 video_format;		/* 0=component, 1=PAL, 2=NTSC, 3=SECAM, 4=MAC, 5=unspecified */

	/*
	 * Temporal layer QP offsets (GOP_PARAM[26:9]).
	 * When non-zero (any layer non-zero), enTemporalLayerQp[8] is set.
	 * tidQp0[14:9], tidQp1[20:15], tidQp2[26:21]: signed 6-bit, -32..+31.
	 * Per SDK wave4.c:791 (gopParam.enTemporalLayerQp/tidQp0..2).
	 */
	s8 temporal_layer_qp[3];	/* WAVE420L_CID_TEMPORAL_LAYER_QP_{0,1,2} */

	/*
	 * Per-frame force picture type (ENC_PIC_PARAM[22:20]).
	 * 0=auto, 1=I/IDR (forceIntra), 2=P (forcePicType=1), 3=B (forcePicType=2).
	 * V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME takes precedence over this.
	 * Per SDK wave4.c:1184 (forcePicTypeEnable<<20 | forcePicType<<21).
	 */
	u32 force_pic_type;		/* WAVE420L_CID_FORCE_PIC_TYPE */

	/* ROI / CTU QP map */
	bool roi_enable;		/* roiEnable -- requires rc_enable=true */
	s8 roi_delta_qp;		/* delta QP applied to all CTUs in ROI map */
	bool ctu_qp_enable;		/* ctuQpEnable -- requires rc_enable=false */
	struct {
		void *virt;		/* virtual address of CTU map buffer */
		dma_addr_t dma;		/* DMA/physical address */
		size_t size;		/* w_ctu * h_ctu bytes */
	} ctu_map;			/* shared buffer for ROI or CTU QP map */

	/*
	 * Custom GOP (UND-CUSTOM-GOP):
	 * When custom_gop_enable=true, gopPresetIdx=0 is written to GOP_PARAM
	 * and a second SET_PARAM (OPT_CUSTOM_GOP) command is issued after
	 * INIT_SEQ to program per-picture GOP entries.
	 * If custom_gop.gop_size=0, a built-in IPPP default is used.
	 */
	bool custom_gop_enable;		/* WAVE420L_CID_CUSTOM_GOP_ENABLE */
	struct wave420l_custom_gop_param custom_gop; /* WAVE420L_CID_CUSTOM_GOP_PARAM */

	/* Per-frame encode statistics (read from firmware result registers) */
	u32 last_avg_qp;	/* Average CTU QP for last encoded frame */
	u32 last_num_intra;	/* Number of intra-coded CTUs */
	u32 last_num_merge;	/* Number of merge-coded CTUs */
	u32 last_num_skip;	/* Number of skipped CTUs */
	u32 last_num_slices;	/* Number of slices in last frame */

	/* SEI (Supplemental Enhancement Information) */
	struct wave420l_buf sei_buf;	/* DMA buffer for SEI NAL payload */
	u32 sei_data_size;		/* Current prefix SEI data size in bytes */
	bool sei_enabled;		/* Whether SEI insertion is active */

	/* Hardware buffers */
	struct wave420l_buf work_buf;
	struct wave420l_buf temp_buf;
	struct wave420l_buf bs_buf;
	struct wave420l_buf *fb_bufs;
	struct wave420l_buf sub_sam_buf;
	struct wave420l_buf fbc_y_tbl_buf;
	struct wave420l_buf fbc_c_tbl_buf;
	struct wave420l_buf mv_buf;
	struct wave420l_buf test_src_buf;  /* DEBUG: coherent test source */
	u32 fb_count;
	u32 raw_stride;     /* ALIGN32(width) — used for buffer layout */
	u32 min_fb_count;
	u32 min_src_count;

	/* Encoding state */
	u32 sequence;
	u32 queued_src_count;
	u32 queued_dst_count;

	/*
	 * Async device_run: set when device_run has issued ENC_PIC and returned
	 * without calling v4l2_m2m_job_finish().  The IRQ thread detects this
	 * flag and calls wave420l_enc_irq_complete() instead of completing
	 * irq_done (which is reserved for synchronous paths such as flush).
	 */
	bool async_enc_pending;
	struct vb2_v4l2_buffer *pending_src_buf;
	struct vb2_v4l2_buffer *pending_dst_buf;
};

struct wave420l_match_data {
	u32 pwr_sleep_bits;
	u32 pwr_iso_bits;
	u32 reset_bits;
	phys_addr_t dos_base;
	bool use_ccf_clocks;
};

struct wave420l_dev {
	struct device *dev;
	void __iomem *reg_base;
	void __iomem *dos_base;
	struct regmap *hhi_map;
	struct regmap *ao_map;

	struct v4l2_device v4l2_dev;
	struct v4l2_m2m_dev *m2m_dev;
	struct video_device *vdev;

	struct mutex dev_lock;		/* V4L2 serialization (also vdev->lock and vq->lock) */
	struct mutex hw_lock;		/* VPU command serialization */
	struct mutex vpu_lock;		/* VPU power/init refcount serialization */
	int irq;

	struct ida inst_ida;
	struct list_head instances;
	int vpu_refcount;		/* VPU init reference count (protected by vpu_lock) */
	struct wave420l_instance *active_inst;	/* instance owning current VPU command */
	/*
	 * VPU command slot: one command in flight on the VPU at a time, held
	 * from issue until the firmware has finished it (hw_lock covers only
	 * the register writes).  Binary semaphore so the IRQ thread can
	 * release it for an async ENC_PIC.  Taken BEFORE hw_lock.
	 */
	struct semaphore cmd_slot;
	spinlock_t irq_lock;		/* protects irq_reason (hardirq -> thread) */
	u32 irq_reason;			/* W4_VPU_INT_REASON bits latched by the hardirq */

	const struct wave420l_match_data *match_data;

	/* Firmware */
	struct wave420l_buf fw_buf;
	bool fw_loaded;
	bool vpu_initialized;		/* VPU firmware initialized */
	bool vpu_sleeping;		/* VPU in firmware sleep state */
	u32 saved_sp;			/* Stack pointer saved during sleep */

	/* Bus clock (DOS gate) */
	struct clk *clk_dos;

	/* Clocks (CCF path for SC2+) */
	struct clk *clk_a;
	struct clk *clk_b;
	struct clk *clk_c;
};

/* ------------------------------------------------------------------ */
/* Hardware operations (wave420l-hw.c)                                 */
/* ------------------------------------------------------------------ */

static inline u32 wave420l_read(struct wave420l_dev *dev, u32 offset)
{
	return readl(dev->reg_base + offset);
}

static inline void wave420l_write(struct wave420l_dev *dev, u32 offset, u32 val)
{
	writel(val, dev->reg_base + offset);
}

int wave420l_hw_power_on(struct wave420l_dev *dev);
void wave420l_hw_power_off(struct wave420l_dev *dev);
int wave420l_hw_load_firmware(struct wave420l_dev *dev);
int wave420l_hw_init_vpu(struct wave420l_dev *dev);
int wave420l_hw_vpu_acquire(struct wave420l_dev *dev);
void wave420l_hw_vpu_release(struct wave420l_dev *dev);

int wave420l_hw_create_instance(struct wave420l_instance *inst);
int wave420l_hw_destroy_instance(struct wave420l_instance *inst);
int wave420l_hw_init_seq(struct wave420l_instance *inst);
int wave420l_hw_emit_header(struct wave420l_instance *inst);
int wave420l_hw_set_fb(struct wave420l_instance *inst);
int wave420l_hw_set_param_change(struct wave420l_instance *inst);
int wave420l_hw_start_encode(struct wave420l_instance *inst,
			     dma_addr_t src_y, dma_addr_t src_u, dma_addr_t src_v,
			     u32 src_stride,
			     dma_addr_t bs_addr, u32 bs_size,
			     u32 src_idx);
int wave420l_hw_get_encode_result(struct wave420l_instance *inst,
				  u32 *bs_size, u32 *pic_type, u32 *src_idx);

/*
 * wave420l_enc_irq_complete - called from the IRQ thread after an async
 * ENC_PIC command completes.  Reads encode results, completes the V4L2
 * buffers, and calls v4l2_m2m_job_finish() to release the M2M scheduler.
 * Defined in wave420l-enc.c.
 */
void wave420l_enc_irq_complete(struct wave420l_instance *inst);
int wave420l_hw_set_vui(struct wave420l_instance *inst);
int wave420l_hw_flush_instance(struct wave420l_instance *inst);
int wave420l_hw_sleep_vpu(struct wave420l_dev *dev);
int wave420l_hw_wakeup_vpu(struct wave420l_dev *dev);
int wave420l_hw_flush_instance(struct wave420l_instance *inst);
int wave420l_hw_sleep_vpu(struct wave420l_dev *dev);
int wave420l_hw_wakeup_vpu(struct wave420l_dev *dev);

int wave420l_wait_busy(struct wave420l_dev *dev);
int wave420l_issue_command(struct wave420l_dev *dev, u32 inst_idx, u32 cmd);
void wave420l_hw_cmd_slot_put(struct wave420l_dev *dev);

/* ------------------------------------------------------------------ */
/* V4L2 encoder (wave420l-enc.c)                                      */
/* ------------------------------------------------------------------ */

int wave420l_enc_register(struct wave420l_dev *dev);
void wave420l_enc_unregister(struct wave420l_dev *dev);

#endif /* __WAVE420L_H__ */
