/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 MediaTek Inc.
 * Author: Ping-Hsun Wu <ping-hsun.wu@mediatek.com>
 */

#ifndef __MTK_MDP3_CORE_H__
#define __MTK_MDP3_CORE_H__

#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>
#include <linux/soc/mediatek/mtk-mmsys.h>
#include <linux/soc/mediatek/mtk-mutex.h>
#include "mtk-mdp3-comp.h"
#include "mtk-mdp3-vpu.h"

#define MDP_MODULE_NAME		"mtk-mdp3"
#define MDP_DEVICE_NAME		"MediaTek MDP3"
#define MDP_PHANDLE_NAME	"mediatek,mdp3"

enum mdp_infra_id {
	/*
	 * Due to the sequential nature of function "mdp_mm_subsys_deploy",
	 * adding new enum. necessitates careful consideration.
	 */
	MDP_INFRA_MMSYS,
	MDP_INFRA_MMSYS2,
	MDP_INFRA_MUTEX,
	MDP_INFRA_MUTEX2,
	MDP_INFRA_SCP,
	MDP_INFRA_MAX
};

enum mdp_mm_subsys_id {
	MDP_MM_SUBSYS_0,
	MDP_MM_SUBSYS_1,
	MDP_MM_SUBSYS_MAX,
};

enum mdp_buffer_usage {
	MDP_BUFFER_USAGE_HW_READ,
	MDP_BUFFER_USAGE_MDP,
	MDP_BUFFER_USAGE_MDP2,
	MDP_BUFFER_USAGE_ISP,
	MDP_BUFFER_USAGE_WPE,
};

struct mdp_platform_config {
	bool	rdma_support_10bit;
	bool	rdma_rsz1_sram_sharing;
	bool	rdma_upsample_repeat_only;
	bool	rdma_esl_setting;
	u32	rdma_event_num;
	bool	rsz_disable_dcm_small_sample;
	bool	rsz_etc_control;
	bool	wrot_filter_constraint;
	bool	wrot_support_10bit;
	u32	wrot_event_num;
	u32	tdshp_hist_num;
	bool	tdshp_constrain;
	bool	tdshp_contour;
};

/* indicate which mutex is used by each pipepline */
enum mdp_pipe_id {
	MDP_PIPE_WPEI,
	MDP_PIPE_WPEI2,
	MDP_PIPE_IMGI,
	MDP_PIPE_RDMA0,
	MDP_PIPE_RDMA1,
	MDP_PIPE_RDMA2,
	MDP_PIPE_RDMA3,
	MDP_PIPE_SPLIT,
	MDP_PIPE_SPLIT2,
	MDP_PIPE_VPP0_SOUT,
	MDP_PIPE_VPP1_SOUT,
	MDP_PIPE_MAX
};

/* MDP parallel pipe control */
enum {
	MDP_PP_USED_1 = 1,
	MDP_PP_USED_2 = 2,
};

#define MDP_PP_MAX MDP_PP_USED_2

struct mtk_mdp_driver_data {
	const int mdp_plat_id;
	const resource_size_t mdp_con_res;
	const struct of_device_id *mdp_probe_infra;
	const struct mdp_platform_config *mdp_cfg;
	const u32 *mdp_mutex_table_idx;
	const struct mdp_comp_data *comp_data;
	unsigned int comp_data_len;
	const struct of_device_id *mdp_sub_comp_dt_ids;
	const struct mdp_format *format;
	unsigned int format_len;
	const struct mdp_limit *def_limit;
	const struct mdp_pipe_info *pipe_info;
	unsigned int pipe_info_len;
	const struct v4l2_rect *pp_criteria;
	const u8 pp_used;
	/*
	 * The SCP/VPU on this SoC emits component inner-ids in the OLD compact
	 * MDP-component numbering (RDMA2=4 .. WROT2=36) instead of the mt8195
	 * numbering the driver enum + kernel-direct tables now use (RDMA2=10 ..
	 * WROT2=55). Set true (mt8188/geralt SCP) to remap every SCP-returned
	 * config component id compact->mt8195 in mdp_vpu_process before the path
	 * build reverse-maps it. mt8195's own SCP already emits mt8195 numbering.
	 */
	const bool scp_comp_id_compact;
};

struct mdp_mm_subsys {
	struct device *mmsys;
	struct device *mutex;
	struct mtk_mutex *mdp_mutex[MDP_PIPE_MAX];
};

struct mdp_dev {
	struct platform_device			*pdev;
	struct mdp_mm_subsys			mm_subsys[MDP_MM_SUBSYS_MAX];
	struct mdp_comp				*comp[MDP_MAX_COMP_COUNT];
	const struct mtk_mdp_driver_data	*mdp_data;

	struct workqueue_struct			*job_wq;
	struct workqueue_struct			*clock_wq;
	struct mdp_vpu_dev			vpu;
	struct mtk_scp				*scp;
	struct rproc				*rproc_handle;
	/* synchronization protect for accessing vpu working buffer info */
	struct mutex				vpu_lock;
	s32					vpu_count;
	u32					id_count;
	struct ida				mdp_ida;
	struct cmdq_client			*cmdq_clt[MDP_PP_MAX];
	u8					cmdq_shift_pa[MDP_PP_MAX];
	wait_queue_head_t			callback_wq;

	struct v4l2_device			v4l2_dev;
	struct video_device			*m2m_vdev;
	struct v4l2_m2m_dev			*m2m_dev;
	/* synchronization protect for m2m device operation */
	struct mutex				m2m_lock;
	atomic_t				suspended;
	refcount_t				job_count;

	/* Per-job GCE (cmdq) watchdog. A wedged cmdq packet (a SOF/EOF event
	 * that never arrives) otherwise leaves the completion callback un-fired
	 * forever -- job_count never reaches 0, so the m2m job never finishes
	 * and close()/v4l2-ctl D-state-blocks indefinitely (and the clk
	 * prepare_lock cascade can take wifi down with it). timeout_work is
	 * armed at submit and cancelled the instant the job completes; on expiry
	 * it mbox_flush()es the wedged GCE thread (WARM_RESET), which re-injects
	 * the rx_callback for the stuck packet(s) and drives the SAME completion
	 * cleanup, so the job self-recovers and the box stays ssh-reachable.
	 * timeout_chan[] snapshots the distinct in-flight GCE channels to flush.
	 * job_done is the timer-vs-callback race winner: exactly one of
	 * {last completion, timeout} owns finalisation. */
	struct delayed_work			timeout_work;
	struct mbox_chan			*timeout_chan[MDP_PP_MAX];
	u8					timeout_nchan;
	atomic_t				job_done;
};

struct mdp_pipe_info {
	enum mdp_pipe_id pipe_id;
	enum mdp_mm_subsys_id sub_id;
	u32 mutex_id;
};

int mdp_vpu_get_locked(struct mdp_dev *mdp);
void mdp_vpu_put_locked(struct mdp_dev *mdp);
int mdp_vpu_register(struct mdp_dev *mdp);
void mdp_vpu_unregister(struct mdp_dev *mdp);
void mdp_video_device_release(struct video_device *vdev);

#endif  /* __MTK_MDP3_CORE_H__ */
