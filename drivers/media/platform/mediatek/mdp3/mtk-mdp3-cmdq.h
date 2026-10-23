/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 MediaTek Inc.
 * Author: Ping-Hsun Wu <ping-hsun.wu@mediatek.com>
 */

#ifndef __MTK_MDP3_CMDQ_H__
#define __MTK_MDP3_CMDQ_H__

#include <linux/platform_device.h>
#include <linux/videodev2.h>
#include <linux/soc/mediatek/mtk-cmdq.h>
#include "mtk-img-ipi.h"

/* Per-context WROT programmable color-correction matrix (CCM), sourced from the
 * V4L2_CID_MTK_MDP_CCM control on the m2m ctx. @en gates the ext-matrix override
 * in config_wrot_frame; when false the WROT uses its built-in CSC (production
 * byte-identical). @coef are the nine 3x3 coeffs c00..c22 as raw signed
 * fixed-point (Q12: 4096 = 1.0), masked 0x1fff before packing into
 * VIDO_MAT_RMY..BMV. @preadd/@postadd are the already-packed VIDO_MAT_PRE/POSTADD
 * words (three signed 9-bit offsets at <<0/<<10/<<20). Carried by value in the
 * cmd so config_wrot_frame is per-job (not a global module param). */
struct mdp_ccm_cfg {
	bool	en;
	s32	coef[9];
	u32	preadd;
	u32	postadd;
};

struct mdp_cmdq_param {
	struct img_config *config;
	struct img_ipi_frameparam *param;
	const struct v4l2_rect *composes[IMG_MAX_HW_OUTPUTS];
	/* Per-context CCM (WROT ext-matrix); NULL/!en => built-in CSC. */
	const struct mdp_ccm_cfg *ccm;
	/* Per-context RSZ coeff-table select (mt8195); -1 => driver default. */
	int rsz_table;
	/* True iff this job was computed kernel-direct (mdp_vpu_process). The RSZ
	 * coeff-table override applies only when true (never on the SCP path). */
	bool rsz_kd;

	void (*cmdq_cb)(struct cmdq_cb_data data);
	void *cb_data;
	void *mdp_ctx;

	/* mdp_kd_pktreuse: if non-NULL and reuse is eligible, mdp_cmdq_prepare
	 * marks the built (single-pp) cmd for reuse, records the src/dst iova
	 * patch slots, and stashes the cmd pointer here for the caller to cache. */
	struct mdp_cmdq_cmd **reuse_slot;

	/* Imported (VB2_MEMORY_DMABUF) SOURCE plane dma_bufs, BORROWED from the src
	 * vb2_buffer by mdp_m2m_device_run. mdp_cmdq_prepare takes a get_dma_buf()
	 * ref on each into the job's cmd, held for the whole async job, so a
	 * concurrent context cannot unmap/recycle the imported RDMA source mapping
	 * while the in-flight SVPP2 MDP_RDMA still reads the baked iova
	 * (use-after-unmap iommu-fault fix). src_dbuf_n==0 for MMAP sources
	 * (vb2-allocated, persistent mapping -- nothing to pin). */
	struct dma_buf *src_dbuf[IMG_MAX_PLANES];
	u8 src_dbuf_n;
};

struct mdp_cmdq_cmd {
	struct work_struct auto_release_work;
	struct cmdq_pkt pkt;
	s32 *event;
	struct mdp_dev *mdp;
	struct cmdq_cb_data *data;
	void (*user_cmdq_cb)(struct cmdq_cb_data data);
	void *user_cb_data;
	struct mdp_comp *comps;
	void *mdp_ctx;
	/* Per-job CCM snapshot (copied from mdp_cmdq_param.ccm at prepare time, so
	 * config_wrot_frame can program the WROT ext-matrix per-context). */
	struct mdp_ccm_cfg ccm;
	/* Per-job RSZ coeff-table snapshot (copied from mdp_cmdq_param.rsz_table at
	 * prepare time, so config_rsz_frame can override the table per-context).
	 * -1 => driver default. */
	int rsz_table;
	/* KERNEL-DIRECT gate for the RSZ table override (see mdp_cmdq_param.rsz_kd). */
	bool rsz_kd;
	u8 num_comps;
	u8 pp_idx;

	/* Packet-reuse fast-path (mdp_kd_pktreuse). When @reuse is set the
	 * completion callback keeps this cmd (pkt + comps) alive instead of
	 * destroying it, so a subsequent same-geometry device_run can patch only
	 * the src/dst plane iovas in place and re-submit, skipping the img_config
	 * compute + the whole GCE-packet rebuild. */
	bool reuse;
	u8 patch_n;
	u16 patch_off[8];	/* byte offset in pkt.va_base of an iova write value */
	u8 patch_tag[8];	/* 0 = src (RDMA), 1 = dst (WROT/WDMA) */
	u32 src_base;		/* src plane-0 iova baked in the packet right now */
	u32 dst_base;		/* dst plane-0 iova baked in the packet right now */

	/* OWNED get_dma_buf() refs on the imported SOURCE plane dma_bufs, pinning
	 * the imported RDMA source mapping for the whole async job (SVPP2
	 * use-after-unmap iommu-fault fix). Acquired in mdp_cmdq_prepare (before the
	 * packet goes live on the GCE, so no completion race); dropped exactly once
	 * per cmd at every cmd-destroy site via mdp_cmd_put_src_dbufs(). Never held
	 * by a reuse-cached cmd -- the pin is gated on the non-cacheable full-build
	 * path (a cached cmd outlives one job and its source changes each reuse). */
	struct dma_buf *src_dbuf[IMG_MAX_PLANES];
	u8 src_dbuf_n;
};

extern bool mdp_kd_pktreuse;

struct dma_buf;
struct mdp_dev;
struct mdp_vpu_dev;
struct img_ipi_frameparam;

/* Per-job GCE (cmdq) watchdog delayed-work handler (see mdp_dev.timeout_work).
 * On a wedged packet it mbox_flush()es the stuck GCE thread so the normal
 * completion path self-recovers and close() unblocks. */
void mdp_gce_timeout_work(struct work_struct *work);

int mdp_cmdq_send(struct mdp_dev *mdp, struct mdp_cmdq_param *param);
/* Packet-reuse fast-path: patch the src/dst iovas of a previously-built cmd and
 * re-submit it, skipping mdp_vpu_process + the packet rebuild. */
int mdp_cmdq_reuse(struct mdp_dev *mdp, struct mdp_cmdq_cmd *cmd,
		   struct img_ipi_frameparam *fp);
/* Free a cmd cached for reuse (invalidate on geometry change / streamoff). */
void mdp_cmdq_reuse_free(struct mdp_cmdq_cmd *cmd);
/* Async multi-job submit for the mt8183 kernel-direct detile: nchunks packets,
 * one GCE-chained batch, one pm ref, one job_count, buf_done on the last. */
int mdp_cmdq_send_detile(struct mdp_dev *mdp, struct mdp_cmdq_param *param,
			 u32 nchunks,
			 void (*recompute_chunk)(struct mdp_vpu_dev *vpu,
						 struct img_ipi_frameparam *p,
						 u32 chunk_idx));

#endif  /* __MTK_MDP3_CMDQ_H__ */
