/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 MediaTek Inc.
 * Author: Ping-Hsun Wu <ping-hsun.wu@mediatek.com>
 */

#ifndef __MDP_REG_WROT_H__
#define __MDP_REG_WROT_H__

#define VIDO_CTRL                   0x000
#define VIDO_MAIN_BUF_SIZE          0x008
#define VIDO_SOFT_RST               0x010
#define VIDO_SOFT_RST_STAT          0x014
#define VIDO_CROP_OFST              0x020
#define VIDO_TAR_SIZE               0x024
#define VIDO_OFST_ADDR              0x02c
#define VIDO_STRIDE                 0x030
#define VIDO_OFST_ADDR_C            0x038
#define VIDO_STRIDE_C               0x03c
#define VIDO_CTRL_2                 0x048
#define VIDO_DITHER                 0x054
/* WROT internal output dither sub-registers (SVPP WROT block, mt2712/mt6799p
 * mdp_reg_wrot.h offsets; identical block on mt8195). Used by the gated
 * mdp_kd_dither prototype (error-diffusion / round dither for <8bpc output). */
#define VIDO_DITHER_00              0x0a4
#define VIDO_DITHER_02              0x0ac
#define VIDO_DITHER_03              0x0b0
#define VIDO_DITHER_04              0x0b4
#define VIDO_DITHER_05              0x0b8
#define VIDO_DITHER_06              0x0bc
#define VIDO_DITHER_07              0x0c0
#define VIDO_DITHER_08              0x0c4
#define VIDO_DITHER_09              0x0c8
#define VIDO_DITHER_00_MASK             0x00000003
#define VIDO_DITHER_02_MASK             0x00003cc0
#define VIDO_DITHER_05_MASK             0x00004003
#define VIDO_DITHER_06_MASK             0x00087773
#define VIDO_DITHER_07_MASK             0x00007777
#define VIDO_DITHER_08_MASK             0x00007777
#define VIDO_DITHER_09_MASK             0x00007777
#define VIDO_STRIDE_V               0x06c
#define VIDO_OFST_ADDR_V            0x068
#define VIDO_RSV_1                  0x070
#define VIDO_DMA_PREULTRA           0x074
#define VIDO_IN_SIZE                0x078
#define VIDO_ROT_EN                 0x07c
#define VIDO_FIFO_TEST              0x080
#define VIDO_MAT_CTRL               0x084
/* WROT external (programmable) color-matrix coefficient block. The WROT CSC
 * stage can use either a built-in matrix (VIDO_MAT_CTRL.matrix_sel) or an
 * external, fully user-programmable 3x3 matrix (VIDO_MAT_CTRL.ext_mat, bit1).
 * Offsets 0x88..0xa0 from the SCP's own DpEngine_WROT (dpframework mt8183/mt2712
 * mdp_reg_wrot.h; the WROT IP block is register-identical on mt8195 -- board
 * -validated by the gated mdp_kd_ccm proof). Coeffs are S2.10 signed fixed
 * (13-bit, mask 0x1fff); each *MY/*MV reg packs two coeffs (low <<0, high <<16).
 * RMY={c00,c01} RMV={c02,c10} GMY={c11,c12} BMY={c20,c21} BMV={c22};
 * out[i] = sum_j coef[i][j]*(in[j]+preadd[j]) + postadd[i]. */
#define VIDO_MAT_RMY                0x088
#define VIDO_MAT_RMV                0x08c
#define VIDO_MAT_GMY                0x090
#define VIDO_MAT_BMY                0x094
#define VIDO_MAT_BMV                0x098
#define VIDO_MAT_PREADD             0x09c
#define VIDO_MAT_POSTADD            0x0a0
#define VIDO_SCAN_10BIT             0x0dc
#define VIDO_PENDING_ZERO           0x0e0
#define VIDO_BASE_ADDR              0xf00
#define VIDO_BASE_ADDR_C            0xf04
#define VIDO_BASE_ADDR_V            0xf08

/* MASK */
#define VIDO_CTRL_MASK                  0xf530711f
#define VIDO_MAIN_BUF_SIZE_MASK         0x1fff7f77
#define VIDO_SOFT_RST_MASK              0x00000001
#define VIDO_SOFT_RST_STAT_MASK         0x00000001
#define VIDO_TAR_SIZE_MASK              0x1fff1fff
#define VIDO_CROP_OFST_MASK             0x1fff1fff
#define VIDO_OFST_ADDR_MASK             0x0fffffff
#define VIDO_STRIDE_MASK                0x0000ffff
#define VIDO_OFST_ADDR_C_MASK           0x0fffffff
#define VIDO_STRIDE_C_MASK              0x0000ffff
#define VIDO_CTRL_2_MASK                0x0000000f
#define VIDO_DITHER_MASK                0xff000001
#define VIDO_STRIDE_V_MASK              0x0000ffff
#define VIDO_OFST_ADDR_V_MASK           0x0fffffff
#define VIDO_RSV_1_MASK                 0xffffffff
#define VIDO_DMA_PREULTRA_MASK		0x00ffffff
#define VIDO_IN_SIZE_MASK               0x1fff1fff
#define VIDO_ROT_EN_MASK                0x00000001
#define VIDO_FIFO_TEST_MASK             0x00000fff
#define VIDO_MAT_CTRL_MASK              0x000000f3
#define VIDO_MAT_RMY_MASK               0x1fff1fff
#define VIDO_MAT_RMV_MASK               0x1fff1fff
#define VIDO_MAT_GMY_MASK               0x1fff1fff
#define VIDO_MAT_BMY_MASK               0x1fff1fff
#define VIDO_MAT_BMV_MASK               0x00001fff
#define VIDO_MAT_PREADD_MASK            0x1ff7fdff
#define VIDO_MAT_POSTADD_MASK           0x1ff7fdff
#define VIDO_SCAN_10BIT_MASK            0x0000000f
#define VIDO_PENDING_ZERO_MASK          0x07ffffff
#define VIDO_BASE_ADDR_MASK             0xffffffff
#define VIDO_BASE_ADDR_C_MASK           0xffffffff
#define VIDO_BASE_ADDR_V_MASK           0xffffffff

#endif  // __MDP_REG_WROT_H__
