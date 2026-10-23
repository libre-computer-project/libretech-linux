/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 MediaTek Inc.
 * Author: Ping-Hsun Wu <ping-hsun.wu@mediatek.com>
 */

#ifndef __MTK_MDP3_M2M_H__
#define __MTK_MDP3_M2M_H__

#include <media/v4l2-ctrls.h>
#include "mtk-mdp3-core.h"
#include "mtk-mdp3-vpu.h"
#include "mtk-mdp3-regs.h"
#include "mtk-img-ipi.h"

struct mdp_cmdq_cmd;

#define MDP_MAX_CTRLS	10

enum {
	MDP_M2M_SRC = 0,
	MDP_M2M_DST = 1,
	MDP_M2M_MAX,
};

/* V4L2_CID_MTK_MDP_CCM payload: a flat s32[15] array --
 *   [0..8]  = 3x3 matrix coeffs c00,c01,c02,c10,c11,c12,c20,c21,c22
 *             (Q12 fixed-point, 4096 = 1.0; masked 0x1fff in HW)
 *   [9..11] = pre-add  offsets p0,p1,p2 (signed 9-bit, packed <<0/<<10/<<20)
 *   [12..14]= post-add offsets q0,q1,q2 (signed 9-bit, packed <<0/<<10/<<20)
 * All-zero => control inactive => the WROT uses its built-in CSC (default). */
#define MDP_CCM_NELEMS		15

struct mdp_m2m_ctrls {
	struct v4l2_ctrl	*hflip;
	struct v4l2_ctrl	*vflip;
	struct v4l2_ctrl	*rotate;
	struct v4l2_ctrl	*ccm;
	struct v4l2_ctrl	*rsz_table;
};

struct mdp_m2m_ctx {
	u32				id;
	struct mdp_dev			*mdp_dev;
	struct v4l2_fh			fh;
	struct v4l2_ctrl_handler	ctrl_handler;
	struct mdp_m2m_ctrls		ctrls;
	struct v4l2_m2m_ctx		*m2m_ctx;
	u32				frame_count[MDP_M2M_MAX];

	struct mdp_frameparam		curr_param;
	/* synchronization protect for mdp m2m context */
	struct mutex			ctx_lock;

	/* Per-context WROT programmable CCM, decoded from V4L2_CID_MTK_MDP_CCM in
	 * mdp_m2m_s_ctrl and read at device_run time. .en=false => built-in CSC. */
	struct mdp_ccm_cfg		ccm;

	/* Per-context RSZ coefficient-table select (V4L2_CID_MTK_MDP_RSZ_COEFF_TABLE),
	 * mt8195. -1 => driver default (byte-identical); 0..27 selects a HW scaler
	 * coeff table (0=sharpest..6=softest on a downscale, board-characterized).
	 * Read at device_run and threaded to config_rsz_frame. */
	int				rsz_table;

	/* mdp_kd_pktreuse: cached built GCE packet for this streaming session and
	 * the frameparam it was built for. While the transform (dims/format/
	 * selection/rotation) is unchanged, subsequent device_runs patch only the
	 * src/dst iovas in @reuse_cmd and re-submit instead of rebuilding. */
	struct mdp_cmdq_cmd		*reuse_cmd;
	struct img_ipi_frameparam	reuse_param;
};

int mdp_m2m_device_register(struct mdp_dev *mdp);
void mdp_m2m_device_unregister(struct mdp_dev *mdp);
void mdp_m2m_job_finish(struct mdp_m2m_ctx *ctx);

#endif  /* __MTK_MDP3_M2M_H__ */
