// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 MediaTek Inc.
 * Author: Ping-Hsun Wu <ping-hsun.wu@mediatek.com>
 */

#include <linux/debugfs.h>
#include <linux/dma-buf.h>
#include <linux/fs.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mailbox_controller.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include "mtk-mdp3-cfg.h"
#include "mtk-mdp3-cmdq.h"
#include "mtk-mdp3-comp.h"
#include "mtk-mdp3-core.h"
#include "mtk-mdp3-m2m.h"
#include "mtk-img-ipi.h"
#include "mdp_reg_rsz.h"
#include "mdp_reg_rdma.h"
#include "mdp_reg_wrot.h"

#include <linux/mailbox_client.h>

#define MDP_PATH_MAX_COMPS	IMG_MAX_COMPONENTS

/*
 * Per-job GCE (cmdq) watchdog bound. Every MDP m2m op finishes in <10 ms; 500 ms
 * is a deliberately generous ceiling so it never trips a healthy job, only a
 * genuinely wedged one (a SOF/EOF GCE event that never arrives). On expiry the
 * watchdog WARM_RESETs the stuck GCE thread so the job self-recovers instead of
 * D-state-blocking close() forever.
 */
#define MDP_GCE_TIMEOUT_MS	500

/* Diag override for the per-job GCE watchdog bound (ms). 0 = use MDP_GCE_TIMEOUT_MS.
 * Bump this (e.g. 30000) to HOLD a wedged kernel-direct MDP job's GCE thread parked
 * at its WFE long enough to probe the stuck engine state over /dev/mem before the
 * watchdog WARM_RESETs the thread. Reset to 0/500 after probing. */
static int mdp_kd_gce_timeout_ms;
module_param(mdp_kd_gce_timeout_ms, int, 0644);
MODULE_PARM_DESC(mdp_kd_gce_timeout_ms,
		 "override per-job GCE watchdog bound in ms to hold a wedged job (0=default 500)");

/* kernel-direct GCE-exec debug gate (defined in mtk-mdp3-vpu.c) */
bool mdp_kd_debug_on(void);
bool mdp_kd_debug_raw(void);	/* un-ratelimited knob read */

/*
 * MDPKD-CFGHEX snapshot: lossless byte-image of the last mdp_path_config
 * img_config_8195 (MT8195), captured under mdp_kd_debug. Exposed read-only via
 * debugfs (mtk-mdp3/cfg8195_snapshot) so userspace reads the EXACT ~62 KB struct
 * image with no printk-ring loss, for a byte-for-byte SCP-vs-kernel-direct diff.
 * Sized to the full struct (no #include of mdp_sm headers needed for the size --
 * IMG cfg union is the largest; 64 KB is a safe ceiling, asserted at fill time).
 */
#define MDP_CFGHEX_MAX 65536
/*
 * cfg8195_snapshot = the LEGACY last-configured-pipe buffer (backward compat:
 * a single-pipe job or the last half of a dual job). For a >1920 DUAL_BITBLT job
 * mdp_path_config fires once PER PIXEL-PIPE and both fires overwrite this one
 * buffer, so only ONE pipe survived -- which blocked the dual-pipe xform/rotate
 * RE (the pp0 config was clobbered by pp1). The per-pipe buffers below keep BOTH
 * pp0 and pp1 (indexed by cmd->pp_idx), exposed as cfg8195_snapshot_pp0 / _pp1,
 * so a >1920 SCP oracle capture (mdp_kernel_direct=0) yields a byte-accurate
 * config for EACH half. All three are debug-only (mdp_kd_debug).
 */
static u8 mdp_cfghex_snap[MDP_CFGHEX_MAX];		/* legacy: last pipe */
static size_t mdp_cfghex_len;
static u32 mdp_cfghex_seq;
#define MDP_CFGHEX_PP 2
static u8 mdp_cfghex_snap_pp[MDP_CFGHEX_PP][MDP_CFGHEX_MAX];	/* per-pipe */
static size_t mdp_cfghex_len_pp[MDP_CFGHEX_PP];
static u32 mdp_cfghex_seq_pp[MDP_CFGHEX_PP];
/*
 * MUST be a mutex, not a spinlock: the read path calls simple_read_from_buffer
 * -> copy_to_user, which may fault/sleep and is ILLEGAL under a spinlock (it
 * EFAULTs -- "Bad address" -- with preemption disabled). Both the writer
 * (mdp_path_config) and the reader run in process context, so a mutex is safe.
 */
static DEFINE_MUTEX(mdp_cfghex_lock);
static struct dentry *mdp_cfghex_dir;

/* debugfs slot descriptor (i_private): which snapshot buffer this file reads. */
struct mdp_cfghex_slot {
	const u8 *buf;
	const size_t *len;
};
static const struct mdp_cfghex_slot mdp_cfghex_slot_legacy = {
	mdp_cfghex_snap, &mdp_cfghex_len };
static const struct mdp_cfghex_slot mdp_cfghex_slot_pp0 = {
	mdp_cfghex_snap_pp[0], &mdp_cfghex_len_pp[0] };
static const struct mdp_cfghex_slot mdp_cfghex_slot_pp1 = {
	mdp_cfghex_snap_pp[1], &mdp_cfghex_len_pp[1] };

static ssize_t mdp_cfghex_read(struct file *f, char __user *buf, size_t len,
			       loff_t *ppos)
{
	const struct mdp_cfghex_slot *slot = file_inode(f)->i_private;
	ssize_t ret;

	if (!slot)
		slot = &mdp_cfghex_slot_legacy;
	mutex_lock(&mdp_cfghex_lock);
	ret = simple_read_from_buffer(buf, len, ppos, slot->buf, *slot->len);
	mutex_unlock(&mdp_cfghex_lock);
	return ret;
}

static const struct file_operations mdp_cfghex_fops = {
	.owner = THIS_MODULE,
	.read = mdp_cfghex_read,
	.llseek = default_llseek,
};

/*
 * Lazily create the debugfs tree on the first CFGHEX fire. The mtk_mdp3
 * platform driver owns the real module_init/exit (so an independent
 * late_initcall would never run in a modular build), hence one-time lazy
 * creation under the snapshot lock. Not torn down on module unload -- this is a
 * WIP-DIAG; the dir persists until reboot, fine since the module is not reloaded
 * mid-session.
 */
static void mdp_cfghex_debugfs_ensure(void)
{
	if (mdp_cfghex_dir)
		return;
	mdp_cfghex_dir = debugfs_create_dir("mtk-mdp3", NULL);
	debugfs_create_file("cfg8195_snapshot", 0444, mdp_cfghex_dir,
			    (void *)&mdp_cfghex_slot_legacy, &mdp_cfghex_fops);
	debugfs_create_u32("cfg8195_seq", 0444, mdp_cfghex_dir, &mdp_cfghex_seq);
	/* Per-pixel-pipe snapshots (both survive a >1920 DUAL_BITBLT job). */
	debugfs_create_file("cfg8195_snapshot_pp0", 0444, mdp_cfghex_dir,
			    (void *)&mdp_cfghex_slot_pp0, &mdp_cfghex_fops);
	debugfs_create_file("cfg8195_snapshot_pp1", 0444, mdp_cfghex_dir,
			    (void *)&mdp_cfghex_slot_pp1, &mdp_cfghex_fops);
	debugfs_create_u32("cfg8195_seq_pp0", 0444, mdp_cfghex_dir,
			   &mdp_cfghex_seq_pp[0]);
	debugfs_create_u32("cfg8195_seq_pp1", 0444, mdp_cfghex_dir,
			   &mdp_cfghex_seq_pp[1]);
}

struct mdp_path {
	struct mdp_dev		*mdp_dev;
	struct mdp_comp_ctx	comps[MDP_PATH_MAX_COMPS];
	u32			num_comps;
	const struct img_config	*config;
	const struct img_ipi_frameparam *param;
	const struct v4l2_rect	*composes[IMG_MAX_HW_OUTPUTS];
	struct v4l2_rect	bounds[IMG_MAX_HW_OUTPUTS];
};

#define has_op(ctx, op) \
	((ctx)->comp->ops && (ctx)->comp->ops->op)
 #define call_op(ctx, op, ...) \
	(has_op(ctx, op) ? (ctx)->comp->ops->op(ctx, ##__VA_ARGS__) : 0)

static bool is_output_disabled(int p_id, const struct img_compparam *param, u32 count)
{
	u32 num = 0;
	bool dis_output = false;
	bool dis_tile = false;

	if (CFG_CHECK(MT8183, p_id)) {
		num = CFG_COMP(MT8183, param, num_subfrms);
		dis_output = CFG_COMP(MT8183, param, frame.output_disable);
		dis_tile = CFG_COMP(MT8183, param, frame.output_disable);
	} else if (CFG_CHECK(MT8195, p_id)) {
		num = CFG_COMP(MT8195, param, num_subfrms);
		dis_output = CFG_COMP(MT8195, param, frame.output_disable);
		dis_tile = CFG_COMP(MT8195, param, frame.output_disable);
	}

	return (count < num) ? (dis_output || dis_tile) : true;
}

static struct mtk_mutex *__get_mutex(const struct mdp_dev *mdp_dev,
				     const struct mdp_pipe_info *p)
{
	return mdp_dev->mm_subsys[p->sub_id].mdp_mutex[p->mutex_id];
}

static u8 __get_pp_num(enum mdp_stream_type type)
{
	switch (type) {
	case MDP_STREAM_TYPE_DUAL_BITBLT:
		return MDP_PP_USED_2;
	default:
		return MDP_PP_USED_1;
	}
}

static enum mdp_pipe_id __get_pipe(const struct mdp_dev *mdp_dev,
				   enum mtk_mdp_comp_id id)
{
	enum mdp_pipe_id pipe_id;

	switch (id) {
	case MDP_COMP_RDMA0:
		pipe_id = MDP_PIPE_RDMA0;
		break;
	case MDP_COMP_ISP_IMGI:
		pipe_id = MDP_PIPE_IMGI;
		break;
	case MDP_COMP_WPEI:
		pipe_id = MDP_PIPE_WPEI;
		break;
	case MDP_COMP_WPEI2:
		pipe_id = MDP_PIPE_WPEI2;
		break;
	case MDP_COMP_RDMA1:
		pipe_id = MDP_PIPE_RDMA1;
		break;
	case MDP_COMP_RDMA2:
		pipe_id = MDP_PIPE_RDMA2;
		break;
	case MDP_COMP_RDMA3:
		pipe_id = MDP_PIPE_RDMA3;
		break;
	default:
		/* Avoid exceptions when operating MUTEX */
		pipe_id = MDP_PIPE_RDMA0;
		dev_err(&mdp_dev->pdev->dev, "Unknown pipeline id %d", id);
		break;
	}

	return pipe_id;
}

static struct img_config *__get_config_offset(struct mdp_dev *mdp,
					      struct mdp_cmdq_param *param,
					      u8 pp_idx)
{
	const int p_id = mdp->mdp_data->mdp_plat_id;
	struct device *dev = &mdp->pdev->dev;
	void *cfg_c, *cfg_n;
	long bound = mdp->vpu.config_size;

	if (pp_idx >= mdp->mdp_data->pp_used)
		goto err_param;

	if (CFG_CHECK(MT8183, p_id)) {
		cfg_c = CFG_OFST(MT8183, param->config, pp_idx);
		cfg_n = CFG_OFST(MT8183, param->config, pp_idx + 1);
	} else if (CFG_CHECK(MT8195, p_id)) {
		cfg_c = CFG_OFST(MT8195, param->config, pp_idx);
		cfg_n = CFG_OFST(MT8195, param->config, pp_idx + 1);
	} else {
		goto err_param;
	}

	if ((long)cfg_n - (long)mdp->vpu.config > bound) {
		dev_err(dev, "config offset %ld OOB %ld\n", (long)cfg_n, bound);
		cfg_c = ERR_PTR(-EFAULT);
	}

	return (struct img_config *)cfg_c;

err_param:
	cfg_c = ERR_PTR(-EINVAL);
	return (struct img_config *)cfg_c;
}

static int mdp_path_subfrm_require(const struct mdp_path *path,
				   struct mdp_cmdq_cmd *cmd,
				   struct mdp_pipe_info *p, u32 count)
{
	const int p_id = path->mdp_dev->mdp_data->mdp_plat_id;
	const struct mdp_comp_ctx *ctx;
	const struct mtk_mdp_driver_data *data = path->mdp_dev->mdp_data;
	struct mtk_mutex *mutex;
	int id, index;
	u32 num_comp = 0;

	if (CFG_CHECK(MT8183, p_id))
		num_comp = CFG_GET(MT8183, path->config, num_components);
	else if (CFG_CHECK(MT8195, p_id))
		num_comp = CFG_GET(MT8195, path->config, num_components);

	/* Decide which mutex to use based on the current pipeline */
	index = __get_pipe(path->mdp_dev, path->comps[0].comp->public_id);
	memcpy(p, &data->pipe_info[index], sizeof(struct mdp_pipe_info));
	mutex = __get_mutex(path->mdp_dev, p);

	/* Set mutex mod */
	for (index = 0; index < num_comp; index++) {
		s32 inner_id = MDP_COMP_NONE;
		const u32 *mutex_idx;
		const struct mdp_comp_blend *b;

		if (CFG_CHECK(MT8183, p_id))
			inner_id = CFG_GET(MT8183, path->config, components[index].type);
		else if (CFG_CHECK(MT8195, p_id))
			inner_id = CFG_GET(MT8195, path->config, components[index].type);

		if (mdp_cfg_comp_is_dummy(path->mdp_dev, inner_id))
			continue;

		ctx = &path->comps[index];
		if (is_output_disabled(p_id, ctx->param, count))
			continue;

		mutex_idx = data->mdp_mutex_table_idx;
		id = ctx->comp->public_id;
		mtk_mutex_write_mod(mutex, mutex_idx[id], false);

		b = &data->comp_data[id].blend;
		if (b && b->aid_mod)
			mtk_mutex_write_mod(mutex, mutex_idx[b->b_id], false);
	}

	mtk_mutex_write_sof(mutex, MUTEX_SOF_IDX_SINGLE_MODE);

	return 0;
}

static int mdp_path_subfrm_run(const struct mdp_path *path,
			       struct mdp_cmdq_cmd *cmd,
			       struct mdp_pipe_info *p, u32 count)
{
	const int p_id = path->mdp_dev->mdp_data->mdp_plat_id;
	const struct mdp_comp_ctx *ctx;
	struct device *dev = &path->mdp_dev->pdev->dev;
	struct mtk_mutex *mutex;
	int index;
	u32 num_comp = 0;
	s32 event;
	s32 inner_id = MDP_COMP_NONE;

	if (-1 == p->mutex_id) {
		dev_err(dev, "Incorrect mutex id");
		return -EINVAL;
	}

	if (CFG_CHECK(MT8183, p_id))
		num_comp = CFG_GET(MT8183, path->config, num_components);
	else if (CFG_CHECK(MT8195, p_id))
		num_comp = CFG_GET(MT8195, path->config, num_components);

	/* Wait WROT SRAM shared to DISP RDMA */
	/* Clear SOF event for each engine */
	for (index = 0; index < num_comp; index++) {
		if (CFG_CHECK(MT8183, p_id))
			inner_id = CFG_GET(MT8183, path->config, components[index].type);
		else if (CFG_CHECK(MT8195, p_id))
			inner_id = CFG_GET(MT8195, path->config, components[index].type);

		if (mdp_cfg_comp_is_dummy(path->mdp_dev, inner_id))
			continue;
		ctx = &path->comps[index];
		if (is_output_disabled(p_id, ctx->param, count))
			continue;
		event = ctx->comp->gce_event[MDP_GCE_EVENT_SOF];
		if (event != MDP_GCE_NO_EVENT)
			MM_REG_CLEAR(cmd, event);
	}

	/* Enable the mutex */
	mutex = __get_mutex(path->mdp_dev, p);
	if (mdp_kd_debug_on())
		dev_info(dev, "MDPKD-MSF count=%u MUTEX-EN id=%d (re-pulse SOF)\n",
			 count, p->mutex_id);
	mtk_mutex_enable_by_cmdq(mutex, (void *)&cmd->pkt);

	/* Wait SOF events and clear mutex modules (optional) */
	for (index = 0; index < num_comp; index++) {
		if (CFG_CHECK(MT8183, p_id))
			inner_id = CFG_GET(MT8183, path->config, components[index].type);
		else if (CFG_CHECK(MT8195, p_id))
			inner_id = CFG_GET(MT8195, path->config, components[index].type);

		if (mdp_cfg_comp_is_dummy(path->mdp_dev, inner_id))
			continue;
		ctx = &path->comps[index];
		if (is_output_disabled(p_id, ctx->param, count))
			continue;
		event = ctx->comp->gce_event[MDP_GCE_EVENT_SOF];
		if (event != MDP_GCE_NO_EVENT)
			MM_REG_WAIT(cmd, event);
	}

	return 0;
}

static int mdp_path_ctx_init(struct mdp_dev *mdp, struct mdp_path *path)
{
	const int p_id = mdp->mdp_data->mdp_plat_id;
	void *param = NULL;
	int index, ret;
	u32 num_comp = 0;

	if (CFG_CHECK(MT8183, p_id))
		num_comp = CFG_GET(MT8183, path->config, num_components);
	else if (CFG_CHECK(MT8195, p_id))
		num_comp = CFG_GET(MT8195, path->config, num_components);

	if (num_comp < 1)
		return -EINVAL;

	for (index = 0; index < num_comp; index++) {
		s32 inner_id = MDP_COMP_NONE;

		if (CFG_CHECK(MT8183, p_id))
			inner_id = CFG_GET(MT8183, path->config, components[index].type);
		else if (CFG_CHECK(MT8195, p_id))
			inner_id = CFG_GET(MT8195, path->config, components[index].type);

		if (mdp_cfg_comp_is_dummy(path->mdp_dev, inner_id))
			continue;
		if (CFG_CHECK(MT8183, p_id))
			param = (void *)CFG_ADDR(MT8183, path->config, components[index]);
		else if (CFG_CHECK(MT8195, p_id))
			param = (void *)CFG_ADDR(MT8195, path->config, components[index]);
		ret = mdp_comp_ctx_config(mdp, &path->comps[index],
					  param, path->param);
		if (ret)
			return ret;
	}

	return 0;
}

static int mdp_path_config_subfrm(struct mdp_cmdq_cmd *cmd,
				  struct mdp_path *path, u32 count)
{
	const int p_id = path->mdp_dev->mdp_data->mdp_plat_id;
	const struct img_mmsys_ctrl *ctrl = NULL;
	const struct img_mux *set;
	struct mdp_comp_ctx *ctx;
	struct mdp_pipe_info pipe;
	int index, ret;
	u32 num_comp = 0;
	s32 inner_id = MDP_COMP_NONE;

	if (CFG_CHECK(MT8183, p_id))
		num_comp = CFG_GET(MT8183, path->config, num_components);
	else if (CFG_CHECK(MT8195, p_id))
		num_comp = CFG_GET(MT8195, path->config, num_components);

	if (CFG_CHECK(MT8183, p_id))
		ctrl = CFG_ADDR(MT8183, path->config, ctrls[count]);
	else if (CFG_CHECK(MT8195, p_id))
		ctrl = CFG_ADDR(MT8195, path->config, ctrls[count]);

	/* Acquire components */
	ret = mdp_path_subfrm_require(path, cmd, &pipe, count);
	if (ret)
		return ret;
	/* Enable mux settings */
	for (index = 0; index < ctrl->num_sets; index++) {
		set = &ctrl->sets[index];
		cmdq_pkt_write(&cmd->pkt, set->subsys_id, set->reg, set->value);
	}
	/* Config sub-frame information */
	for (index = (num_comp - 1); index >= 0; index--) {
		if (CFG_CHECK(MT8183, p_id))
			inner_id = CFG_GET(MT8183, path->config, components[index].type);
		else if (CFG_CHECK(MT8195, p_id))
			inner_id = CFG_GET(MT8195, path->config, components[index].type);

		if (mdp_cfg_comp_is_dummy(path->mdp_dev, inner_id))
			continue;
		ctx = &path->comps[index];
		if (is_output_disabled(p_id, ctx->param, count))
			continue;
		ret = call_op(ctx, config_subfrm, cmd, count);
		if (ret)
			return ret;
	}
	/* Run components */
	ret = mdp_path_subfrm_run(path, cmd, &pipe, count);
	if (ret)
		return ret;
	/* Wait components done */
	for (index = 0; index < num_comp; index++) {
		if (CFG_CHECK(MT8183, p_id))
			inner_id = CFG_GET(MT8183, path->config, components[index].type);
		else if (CFG_CHECK(MT8195, p_id))
			inner_id = CFG_GET(MT8195, path->config, components[index].type);

		if (mdp_cfg_comp_is_dummy(path->mdp_dev, inner_id))
			continue;
		ctx = &path->comps[index];
		if (is_output_disabled(p_id, ctx->param, count))
			continue;
		if (mdp_kd_debug_on())
			dev_info(&path->mdp_dev->pdev->dev,
				 "MDPKD-MSF count=%u WAIT-EOF comp[%d] pubid=%d SOF=%d EOF=%d\n",
				 count, index, ctx->comp->public_id,
				 ctx->comp->gce_event[MDP_GCE_EVENT_SOF],
				 ctx->comp->gce_event[MDP_GCE_EVENT_EOF]);
		ret = call_op(ctx, wait_comp_event, cmd);
		if (ret)
			return ret;
	}
	/* Advance to the next sub-frame */
	for (index = 0; index < num_comp; index++) {
		if (CFG_CHECK(MT8183, p_id))
			inner_id = CFG_GET(MT8183, path->config, components[index].type);
		else if (CFG_CHECK(MT8195, p_id))
			inner_id = CFG_GET(MT8195, path->config, components[index].type);

		if (mdp_cfg_comp_is_dummy(path->mdp_dev, inner_id))
			continue;
		ctx = &path->comps[index];
		ret = call_op(ctx, advance_subfrm, cmd, count);
		if (ret)
			return ret;
	}
	/* Disable mux settings */
	for (index = 0; index < ctrl->num_sets; index++) {
		set = &ctrl->sets[index];
		cmdq_pkt_write(&cmd->pkt, set->subsys_id, set->reg, 0);
	}

	return 0;
}

static int mdp_path_config(struct mdp_dev *mdp, struct mdp_cmdq_cmd *cmd,
			   struct mdp_path *path)
{
	const int p_id = mdp->mdp_data->mdp_plat_id;
	struct mdp_comp_ctx *ctx;
	int index, count, ret;
	u32 num_comp = 0;
	u32 num_sub = 0;
	s32 inner_id = MDP_COMP_NONE;

	if (CFG_CHECK(MT8183, p_id))
		num_comp = CFG_GET(MT8183, path->config, num_components);
	else if (CFG_CHECK(MT8195, p_id))
		num_comp = CFG_GET(MT8195, path->config, num_components);

	if (CFG_CHECK(MT8183, p_id))
		num_sub = CFG_GET(MT8183, path->config, num_subfrms);
	else if (CFG_CHECK(MT8195, p_id))
		num_sub = CFG_GET(MT8195, path->config, num_subfrms);

	/*
	 * MDPKD-CFGHEX: COMPLETE byte-image SNAPSHOT of the whole struct
	 * img_config_8195 (MT8195 only), gated on mdp_kd_debug. Both the SCP
	 * path (mdp_kernel_direct=0) and the kernel-direct path (=1) reach
	 * mdp_path_config with a fully-populated img_config. The whole struct is
	 * ~62 KB, far larger than the printk ring buffer, so a per-line dmesg
	 * dump LOSES the middle under load (verified: comps 9-16 dropped). Instead
	 * snapshot the EXACT bytes into a static buffer here and expose them via
	 * debugfs (mtk-mdp3/cfg8195_snapshot, binary, lossless) so userspace can
	 * read the complete image and diff the two paths byte-for-byte. If the two
	 * snapshots are byte-identical the difference is OUTSIDE img_config; if any
	 * byte differs that field is the true remaining bug. One BEGIN line still
	 * goes to dmesg as a fired/metadata marker.
	 */
	if (CFG_CHECK(MT8195, p_id) && mdp_kd_debug_raw()) {
		const struct img_config_8195 *c8 = &path->config->config_8195;
		struct device *d = &mdp->pdev->dev;
		size_t n = min(sizeof(*c8), (size_t)MDP_CFGHEX_MAX);
		u8 pp = cmd->pp_idx < MDP_CFGHEX_PP ? cmd->pp_idx : 0;

		mdp_cfghex_debugfs_ensure();	/* process context, idempotent */
		mutex_lock(&mdp_cfghex_lock);
		memcpy(mdp_cfghex_snap, c8, n);		/* legacy last-pipe */
		mdp_cfghex_len = n;
		mdp_cfghex_seq++;
		memcpy(mdp_cfghex_snap_pp[pp], c8, n);	/* this pixel pipe */
		mdp_cfghex_len_pp[pp] = n;
		mdp_cfghex_seq_pp[pp]++;
		mutex_unlock(&mdp_cfghex_lock);

		if (mdp_kd_debug_on())
			dev_info(d,
			 "MDPKD-CFGHEX BEGIN seq=%u pp=%u sizeof(img_config_8195)=%zu snap_len=%zu num_components=%u num_subfrms=%u MAX_COMP=%d MAX_SUB=%d (snapshot -> debugfs mtk-mdp3/cfg8195_snapshot[_pp%u])\n",
			 mdp_cfghex_seq, pp, sizeof(*c8), n, c8->num_components,
			 c8->num_subfrms, IMG_MAX_COMPONENTS,
			 IMG_MAX_SUBFRAMES_8195, pp);
	}

	/* Config path frame */
	/* Reset components */
	for (index = 0; index < num_comp; index++) {
		if (CFG_CHECK(MT8183, p_id))
			inner_id = CFG_GET(MT8183, path->config, components[index].type);
		else if (CFG_CHECK(MT8195, p_id))
			inner_id = CFG_GET(MT8195, path->config, components[index].type);

		if (mdp_cfg_comp_is_dummy(path->mdp_dev, inner_id))
			continue;
		ctx = &path->comps[index];
		ret = call_op(ctx, init_comp, cmd);
		if (ret)
			return ret;
	}
	/* Config frame mode */
	for (index = 0; index < num_comp; index++) {
		const struct v4l2_rect *compose;
		u32 out = 0;

		ctx = &path->comps[index];
		if (CFG_CHECK(MT8183, p_id))
			inner_id = CFG_GET(MT8183, path->config, components[index].type);
		else if (CFG_CHECK(MT8195, p_id))
			inner_id = CFG_GET(MT8195, path->config, components[index].type);

		if (mdp_cfg_comp_is_dummy(path->mdp_dev, inner_id))
			continue;

		if (CFG_CHECK(MT8183, p_id))
			out = CFG_COMP(MT8183, ctx->param, outputs[0]);
		else if (CFG_CHECK(MT8195, p_id))
			out = CFG_COMP(MT8195, ctx->param, outputs[0]);

		compose = path->composes[out];
		ret = call_op(ctx, config_frame, cmd, compose);
		if (ret)
			return ret;
	}

	/* Config path sub-frames */
	for (count = 0; count < num_sub; count++) {
		ret = mdp_path_config_subfrm(cmd, path, count);
		if (ret)
			return ret;
	}
	/* Post processing information */
	for (index = 0; index < num_comp; index++) {
		if (CFG_CHECK(MT8183, p_id))
			inner_id = CFG_GET(MT8183, path->config, components[index].type);
		else if (CFG_CHECK(MT8195, p_id))
			inner_id = CFG_GET(MT8195, path->config, components[index].type);

		if (mdp_cfg_comp_is_dummy(path->mdp_dev, inner_id))
			continue;
		ctx = &path->comps[index];
		ret = call_op(ctx, post_process, cmd);
		if (ret)
			return ret;
	}
	return 0;
}

bool mdp_kd_debug_on(void);	/* exported from mtk-mdp3-vpu.c */

/* Runtime PRZ (RSZ) register READBACK -- gated by mdp_kd_debug. After the GCE
 * job completes the RSZ registers hold the last-latched frame/subframe values;
 * read them from the CPU and print, to diff what the HW ACTUALLY got against the
 * intended img_config (the MDPKD-8183-SCALE dump). A documented field that reads
 * back wrong/zero = a write that did not land; an undocumented reg (TAP_ADAPT /
 * IBSE / line-buffer at 0x40..0x5c) that the executor never writes is a candidate
 * for the missing setup behind the flat-vertical mt8183 scale artifact. Runs in
 * mdp_auto_release_work BEFORE mdp_comp_clocks_off(), while the RSZ clock is on;
 * only reads if RSZ0 is part of THIS job (clock guaranteed on). RSZ0 reg window
 * is 0x1000, so 0x00..0x5c is in range. */
/* Full RDMA/RSZ/WROT register-block dump for the rot270 effective-HW-state diff. The GCE
 * packet diff proved the in-job rot270 packet is identical to the standalone except buffer
 * addresses -- but masked writes (MASK+WRITE) leave bits dependent on the PRIOR state (the
 * scale chunk's leftovers vs a fresh engine), invisible to the packet. This dumps the
 * effective post-job register state so the two rot270 runs can be diffed register-by-
 * register. Keyed on the WROT CTRL so the rot270 job's block (CTRL=0x305007) is pickable.
 * mdp_kd_regdump default 0; needs mdp_kd_debug on (the readback's gate). */
static int mdp_kd_regdump;
module_param(mdp_kd_regdump, int, 0644);
MODULE_PARM_DESC(mdp_kd_regdump, "dump full RDMA/RSZ/WROT register blocks post-job (default 0)");

static void mdp_kd_dump_block(struct device *dev, const char *tag,
			      void __iomem *base, u32 len)
{
	u32 o;

	for (o = 0; o < len; o += 32)
		dev_info(dev, "REGB %s +%03x: %08x %08x %08x %08x %08x %08x %08x %08x\n",
			 tag, o, readl(base + o), readl(base + o + 4),
			 readl(base + o + 8), readl(base + o + 12),
			 readl(base + o + 16), readl(base + o + 20),
			 readl(base + o + 24), readl(base + o + 28));
}

/* Return a job-component's ioremapped regs if it is part of THIS job (clock
 * guaranteed on), else NULL. */
static void __iomem *mdp_kd_job_regs(struct mdp_cmdq_cmd *cmd, enum mtk_mdp_comp_id id)
{
	struct mdp_dev *mdp = cmd->mdp;
	u32 i;

	for (i = 0; i < cmd->num_comps; i++)
		if (cmd->comps[i].public_id == id)
			return mdp->comp[id] ? mdp->comp[id]->regs : NULL;
	return NULL;
}

static void mdp_kd_rsz_readback(struct mdp_cmdq_cmd *cmd)
{
	struct mdp_dev *mdp = cmd->mdp;
	void __iomem *r, *rd, *wr;

	if (!mdp_kd_debug_on())
		return;

	/* RDMA0 feed -- what the input DMA actually streamed (size/stride/offset).
	 * A height < the frame height = the active RSZ was starved of rows
	 * (flat-vertical suspect). */
	rd = mdp_kd_job_regs(cmd, MDP_COMP_RDMA0);
	wr = mdp_kd_job_regs(cmd, MDP_COMP_WROT0);
	r = mdp_kd_job_regs(cmd, MDP_COMP_RSZ0);
	if (mdp_kd_regdump) {
		struct device *d = &mdp->pdev->dev;

		dev_info(d, "REGDUMP START wrot_ctrl=%#x\n",
			 wr ? readl(wr + VIDO_CTRL) : 0);
		if (rd)
			mdp_kd_dump_block(d, "RDMA", rd, 0x200);
		if (r)
			mdp_kd_dump_block(d, "RSZ", r, 0x100);
		if (wr)
			mdp_kd_dump_block(d, "WROT", wr, 0x100);
		dev_info(d, "REGDUMP END\n");
	}

	if (rd)
		dev_info(&mdp->pdev->dev,
			 "MDPKD-RDMA-RB EN=%#x GMCIF=%#x SRC_CON=%#x BKGD_PXL=%#x SRC_SIZE=%#x CLIP=%#x OFST0=%#x\n",
			 readl(rd + MDP_RDMA_EN), readl(rd + MDP_RDMA_GMCIF_CON),
			 readl(rd + MDP_RDMA_SRC_CON), readl(rd + MDP_RDMA_MF_BKGD_SIZE_IN_PXL),
			 readl(rd + MDP_RDMA_MF_SRC_SIZE), readl(rd + MDP_RDMA_MF_CLIP_SIZE),
			 readl(rd + MDP_RDMA_SRC_OFFSET_0));

	/* WROT0 write -- VIDO_IN_SIZE is what the WROT received from the RSZ; if its
	 * height != the RSZ output height the resized rows never reached the WROT. */
	wr = mdp_kd_job_regs(cmd, MDP_COMP_WROT0);
	if (wr)
		dev_info(&mdp->pdev->dev,
			 "MDPKD-WROT-RB CTRL=%#x MAIN_BUF=%#x TAR=%#x IN_SIZE=%#x CROP_OFST=%#x OFST=%#x\n",
			 readl(wr + VIDO_CTRL), readl(wr + VIDO_MAIN_BUF_SIZE),
			 readl(wr + VIDO_TAR_SIZE), readl(wr + VIDO_IN_SIZE),
			 readl(wr + VIDO_CROP_OFST), readl(wr + VIDO_OFST_ADDR));

	r = mdp_kd_job_regs(cmd, MDP_COMP_RSZ0);
	if (!r)
		return;
	dev_info(&mdp->pdev->dev,
		 "MDPKD-RSZ-RB EN=%#x CTL1=%#x CTL2=%#x HCOEF=%#x VCOEF=%#x IN=%#x OUT=%#x\n",
		 readl(r + PRZ_ENABLE), readl(r + PRZ_CONTROL_1),
		 readl(r + PRZ_CONTROL_2), readl(r + PRZ_HORIZONTAL_COEFF_STEP),
		 readl(r + PRZ_VERTICAL_COEFF_STEP), readl(r + PRZ_INPUT_IMAGE),
		 readl(r + PRZ_OUTPUT_IMAGE));
	dev_info(&mdp->pdev->dev,
		 "MDPKD-RSZ-RB LHI=%#x LHS=%#x LVI=%#x LVS=%#x CHI=%#x CHS=%#x\n",
		 readl(r + PRZ_LUMA_HORIZONTAL_INTEGER_OFFSET),
		 readl(r + PRZ_LUMA_HORIZONTAL_SUBPIXEL_OFFSET),
		 readl(r + PRZ_LUMA_VERTICAL_INTEGER_OFFSET),
		 readl(r + PRZ_LUMA_VERTICAL_SUBPIXEL_OFFSET),
		 readl(r + PRZ_CHROMA_HORIZONTAL_INTEGER_OFFSET),
		 readl(r + PRZ_CHROMA_HORIZONTAL_SUBPIXEL_OFFSET));
	/* Undocumented tail (dpframework: 0x40 RSV/DEBUG_SEL, 0x44/0x48 DEBUG,
	 * 0x4c TAP_ADAPT, 0x50 IBSE_SOFTCLIP, 0x54/0x58/0x5c IBSE_YLEVEL). The
	 * executor writes NONE of these -- non-reset values here would be HW
	 * defaults; PRZ_DEBUG (0x44/0x48) may expose live line-buffer/pipe state. */
	dev_info(&mdp->pdev->dev,
		 "MDPKD-RSZ-RB tail[0x38..0x5c]=%#x %#x %#x %#x %#x %#x %#x %#x %#x %#x\n",
		 readl(r + 0x38), readl(r + 0x3c), readl(r + 0x40), readl(r + 0x44),
		 readl(r + 0x48), readl(r + 0x4c), readl(r + 0x50), readl(r + 0x54),
		 readl(r + 0x58), readl(r + 0x5c));

	/* --- progress/debug counters to localize a horizontal-downscale STALL ---
	 * Read post-GCE-timeout, clocks still on, engines NOT hard-reset (GCE only
	 * resets its thread), so these reflect the wedged pipe state. Offsets from
	 * the dpframework mt8183 hardware headers (not in the upstream reg defs):
	 *   RDMA MDP_RDMA_INTERRUPT_STATUS 0x18 (bit0 done / err bits)
	 *   WROT VIDO_DEBUG 0xd0
	 *   RSZ  PRZ_INT_FLAG 0x0c, PRZ_DEBUG_SEL 0x44 (mux) -> PRZ_DEBUG 0x48
	 * Sweeping PRZ_DEBUG_SEL 0..15 exposes the RSZ's internal H/V line+column
	 * counters: a counter stuck below the tile width/height pinpoints whether
	 * the RSZ stalled mid-row (horizontal) vs the WROT/RDMA handshake stalled. */
	if (rd)
		dev_info(&mdp->pdev->dev, "MDPKD-RDMA-DBG INT_STATUS=%#x\n",
			 readl(rd + 0x18));
	if (wr)
		dev_info(&mdp->pdev->dev, "MDPKD-WROT-DBG VIDO_DEBUG=%#x\n",
			 readl(wr + 0xd0));
	{
		int sel;
		u32 dbg[16];

		for (sel = 0; sel < 16; sel++) {
			writel(sel, r + 0x44);
			dbg[sel] = readl(r + 0x48);
		}
		dev_info(&mdp->pdev->dev,
			 "MDPKD-RSZ-DBG INT_FLAG=%#x DBG[0-7]=%#x %#x %#x %#x %#x %#x %#x %#x\n",
			 readl(r + 0x0c), dbg[0], dbg[1], dbg[2], dbg[3],
			 dbg[4], dbg[5], dbg[6], dbg[7]);
		dev_info(&mdp->pdev->dev,
			 "MDPKD-RSZ-DBG DBG[8-15]=%#x %#x %#x %#x %#x %#x %#x %#x\n",
			 dbg[8], dbg[9], dbg[10], dbg[11],
			 dbg[12], dbg[13], dbg[14], dbg[15]);
	}
}

/* mdp_kd_wakeearly (backlog #2, "completion wake-early"): signal the m2m job
 * done (mdp_m2m_job_finish -> wakes the DQBUF / next-frame waiter) and release
 * the per-job gate (job_count -> 0 + wake callback_wq) BEFORE the
 * mtk_mutex_unprepare + mdp_comp_clocks_off teardown, shaving that ~11 us off
 * the per-frame completion critical path (~12-14 % of a cheap crop's wall).
 *
 * SAFE against the reuse-cache use-after-free that made the naive reorder
 * unsafe: the deferred teardown runs entirely off a SNAPSHOT taken while cmd is
 * still pinned -- the comp public-ids (mapped to the device-persistent
 * mdp->comp[id], never freed) + the stable per-pipe mutex pointer -- so it never
 * dereferences cmd->comps, which a racing mdp_ctx_reuse_invalidate may kfree the
 * instant job_count hits 0. Clocks and the per-pipe mutex are both
 * clk-refcounted (mtk_mutex_prepare/unprepare == clk_prepare_enable/disable), so
 * a next frame's clocks_on/mutex_prepare that overlaps this deferred
 * clocks_off/unprepare simply nets out (the engine keeps its own ref while it
 * runs). Default on (board-validated bit-exact + measurably faster, else the
 * legacy teardown-before-wake order is restored with =0). */
bool mdp_kd_wakeearly = true;
module_param(mdp_kd_wakeearly, bool, 0644);
MODULE_PARM_DESC(mdp_kd_wakeearly,
		 "wake the m2m completion waiter before the mutex/clocks teardown (default 1)");

/* clocks_off addressed by comp public_id via the device-persistent mdp->comp[]
 * (not the freeable cmd->comps copy), so it is safe on the mdp_kd_wakeearly
 * deferred path after the cmd may have been freed. Mirrors mdp_comp_clocks_off's
 * aid_clk blend-companion handling. */
static void mdp_clocks_off_by_id(struct mdp_dev *mdp, const u8 *ids, int num)
{
	struct device *dev = &mdp->pdev->dev;
	int i;

	for (i = 0; i < num; i++) {
		enum mtk_mdp_comp_id id = ids[i];
		struct mdp_comp *comp = mdp->comp[id];
		const struct mdp_comp_blend *b;

		if (!comp || !comp->mdp_dev)
			continue;
		mdp_comp_clock_off(dev, comp);
		b = &mdp->mdp_data->comp_data[id].blend;
		if (b && b->aid_clk && mdp->comp[b->b_id])
			mdp_comp_clock_off(dev, mdp->comp[b->b_id]);
	}
}

/* Drop the get_dma_buf() refs the job held on its imported SOURCE plane dma_bufs
 * (the SVPP2 MDP_RDMA use-after-unmap pin acquired in mdp_cmdq_prepare). Called
 * exactly once per cmd, at every cmd-destroy site (completion callback, the
 * queue_work-failure teardown, reuse-cache free, detile error free). Idempotent:
 * clears each slot so a defensive double call is a no-op. */
static void mdp_cmd_put_src_dbufs(struct mdp_cmdq_cmd *cmd)
{
	u8 i;

	for (i = 0; i < cmd->src_dbuf_n && i < IMG_MAX_PLANES; i++) {
		if (cmd->src_dbuf[i]) {
			dma_buf_put(cmd->src_dbuf[i]);
			cmd->src_dbuf[i] = NULL;
		}
	}
	cmd->src_dbuf_n = 0;
}

static void mdp_auto_release_work(struct work_struct *work)
{
	struct mdp_cmdq_cmd *cmd;
	struct mdp_dev *mdp;
	struct mtk_mutex *mutex;
	enum mdp_pipe_id pipe_id;
	bool reuse;
	bool wakeearly;
	u8 clk_ids[MDP_PATH_MAX_COMPS];
	int clk_num = 0;

	cmd = container_of(work, struct mdp_cmdq_cmd, auto_release_work);
	mdp = cmd->mdp;
	/* Snapshot before the job_count wake_up below: once job_count hits 0 the
	 * m2m ctx may free this cached cmd, so no cmd deref is allowed afterwards. */
	reuse = cmd->reuse;
	wakeearly = mdp_kd_wakeearly;

	mdp_kd_rsz_readback(cmd);	/* before clocks_off: RSZ clock still on */

	pipe_id = __get_pipe(mdp, cmd->comps[0].public_id);
	mutex = __get_mutex(mdp, &mdp->mdp_data->pipe_info[pipe_id]);

	if (wakeearly) {
		/* Snapshot the comp ids for the deferred (post-wake) clocks_off so
		 * it never touches the possibly-freed cmd->comps. */
		int i;

		clk_num = cmd->num_comps;
		for (i = 0; i < clk_num && i < MDP_PATH_MAX_COMPS; i++)
			clk_ids[i] = cmd->comps[i].public_id;
	} else {
		/* Legacy order: teardown before the wake (mdp_kd_wakeearly=0). */
		mtk_mutex_unprepare(mutex);
		mdp_comp_clocks_off(&mdp->pdev->dev, cmd->comps,
				    cmd->num_comps);
	}

	if (refcount_dec_and_test(&mdp->job_count)) {
		/* Job completed normally: disarm the GCE watchdog and claim the
		 * finalisation so a timer that may already be running bails out
		 * (atomic_xchg guard in mdp_gce_timeout_work). Non-sync cancel:
		 * a sync cancel here could deadlock against the watchdog work if
		 * it is mid-flight, and is unnecessary -- the job_done flag makes
		 * the loser of the race a no-op. The per-packet cleanup below
		 * (mutex/clocks/pkt-destroy/free) runs once per packet regardless;
		 * the watchdog never frees a packet itself, so no double-free. */
		cancel_delayed_work(&mdp->timeout_work);
		atomic_set(&mdp->job_done, 1);

		if (cmd->mdp_ctx)
			mdp_m2m_job_finish(cmd->mdp_ctx);

		if (cmd->user_cmdq_cb) {
			struct cmdq_cb_data user_cb_data;

			user_cb_data.sta = cmd->data->sta;
			user_cb_data.pkt = cmd->data->pkt;
			cmd->user_cmdq_cb(user_cb_data);
		}
		/* Release the GCE runtime-PM ref held by mdp_cmdq_send for the
		 * whole async job (paired get_sync there). Without this the GCE
		 * autosuspends mid-flight and long multi-engine (SVPP) jobs hang. */
		pm_runtime_mark_last_busy(mdp->cmdq_clt[cmd->pp_idx]->chan->mbox->dev);
		pm_runtime_put_autosuspend(mdp->cmdq_clt[cmd->pp_idx]->chan->mbox->dev);
		wake_up(&mdp->callback_wq);
	}

	if (wakeearly) {
		/* Deferred teardown off the snapshot -- no cmd->comps deref, so a
		 * racing invalidate that freed the cached cmd once job_count hit 0
		 * cannot be use-after-free'd. Refcounted clocks/mutex net out
		 * against any next-frame prepare/on that already ran. */
		mtk_mutex_unprepare(mutex);
		mdp_clocks_off_by_id(mdp, clk_ids, clk_num);
	}

	/* Reuse cache: keep pkt + comps + cmd alive for the next same-geometry
	 * frame. mdp_cmdq_reuse_free() releases it on geometry change / streamoff. */
	if (reuse)
		return;

	/* Job done + the RDMA clock is now gated (clocks_off above) -> the source
	 * RDMA has drained, so releasing the imported-source pin here cannot
	 * use-after-unmap. */
	mdp_cmd_put_src_dbufs(cmd);
	cmdq_pkt_destroy(mdp->cmdq_clt[cmd->pp_idx], &cmd->pkt);
	kfree(cmd->comps);
	cmd->comps = NULL;
	kfree(cmd);
	cmd = NULL;
}

static void mdp_handle_cmdq_callback(struct mbox_client *cl, void *mssg)
{
	struct mdp_cmdq_cmd *cmd;
	struct cmdq_cb_data *data;
	struct mdp_dev *mdp;
	struct device *dev;
	enum mdp_pipe_id pipe_id;

	if (!mssg) {
		pr_info("%s:no callback data\n", __func__);
		return;
	}

	data = (struct cmdq_cb_data *)mssg;
	cmd = container_of(data->pkt, struct mdp_cmdq_cmd, pkt);
	cmd->data = data;
	mdp = cmd->mdp;
	dev = &mdp->pdev->dev;

	INIT_WORK(&cmd->auto_release_work, mdp_auto_release_work);
	if (!queue_work(mdp->clock_wq, &cmd->auto_release_work)) {
		struct mtk_mutex *mutex;

		dev_err(dev, "%s:queue_work fail!\n", __func__);
		pipe_id = __get_pipe(mdp, cmd->comps[0].public_id);
		mutex = __get_mutex(mdp, &mdp->mdp_data->pipe_info[pipe_id]);
		mtk_mutex_unprepare(mutex);
		mdp_comp_clocks_off(&mdp->pdev->dev, cmd->comps,
				    cmd->num_comps);

		if (refcount_dec_and_test(&mdp->job_count)) {
			/* Last packet on the queue_work-failure path: disarm the
			 * watchdog + claim finalisation, same as the normal
			 * mdp_auto_release_work last-packet branch, so the timer
			 * cannot double the pm_put. */
			cancel_delayed_work(&mdp->timeout_work);
			atomic_set(&mdp->job_done, 1);
			pm_runtime_mark_last_busy(mdp->cmdq_clt[cmd->pp_idx]->chan->mbox->dev);
			pm_runtime_put_autosuspend(mdp->cmdq_clt[cmd->pp_idx]->chan->mbox->dev);
			wake_up(&mdp->callback_wq);
		}

		if (cmd->reuse)	/* cached: owned by the m2m ctx, freed on invalidate */
			return;

		mdp_cmd_put_src_dbufs(cmd);	/* release the imported-source pin */
		cmdq_pkt_destroy(mdp->cmdq_clt[cmd->pp_idx], &cmd->pkt);
		kfree(cmd->comps);
		cmd->comps = NULL;
		kfree(cmd);
		cmd = NULL;
	}
}

/*
 * Un-brick the physical MDP engines from the GCE-timeout watchdog. The
 * mbox_flush() WARM_RESET below only resets the GCE *thread* -- enough to drive
 * the job cleanup so close() unblocks -- but it does NOT touch the downstream
 * MDP engines (WROT/RDMA). The design premise was that a wedge leaves the engine
 * latched so every SUBSEQUENT mtk-mdp3 job hangs until reboot; this pulses the
 * same engine SW-RESET the normal GCE path emits (init_rdma / the WROT
 * config_subfrm soft-reset), but from the CPU because the GCE thread that would
 * otherwise carry those reg writes is itself wedged.
 *
 * VALIDATION (2026-07-09, demo-mt8195-chromebook, SCP fw v23042->v26100, A/B via
 * this param): the "reboot required" premise did NOT reproduce. mbox_flush()
 * below already releases the wedged job, and the next job re-programs the engines
 * from scratch, so BOTH param=0 and param=1 recover to bit-exact output with no
 * reboot -- the two arms are observationally identical. So this SW-reset is a
 * no-op for the production recovery path. It is KEPT as defensive belt-and-
 * suspenders for a hypothetical SCP-config-path-after-SCP-config-path wedge --
 * and note that wedge only occurs on the deprecated mdp_kernel_direct=0 route;
 * production defaults to kernel-direct (mtk-mdp3-vpu.c mdp_kernel_direct=true),
 * which computes the config in-kernel and does not wedge. Left on by default (the
 * reset is cheap and bounded on the rare error path); set 0 to A/B.
 */
static bool mdp_engine_reset_on_timeout = true;
module_param(mdp_engine_reset_on_timeout, bool, 0644);
MODULE_PARM_DESC(mdp_engine_reset_on_timeout,
		 "on a GCE timeout, CPU-side SW-RESET the wedged RDMA/WROT engines so the next job is not bricked (default 1)");

/*
 * CPU-side (readl/writel) SW-RESET of one MDP engine. Runs in the watchdog work
 * (process context -> may sleep), so it wraps the reg access in
 * mdp_comp_clock_on/off: that resumes the engine's power domain (genpd) and
 * enables its clocks around the pulse, then drops them. The refcount NETS OUT
 * -- for the wedged job's own comps (clocks/PM already held by the in-flight
 * job at timeout time) it is a harmless +1/-1 bump; for the other enumerated
 * engines (idle, powered off) it powers them up just long enough to reset and
 * powers them back down. So the caller may reset EVERY enumerated RDMA/WROT
 * engine without knowing which one wedged (the per-job comp list is not
 * reachable from the per-device timeout_work) and without a stray reg access to
 * an unpowered engine (which would bus-abort).
 *
 * The pulse mirrors the driver's own GCE-packet reset idioms exactly:
 *   RDMA (init_rdma): assert MDP_RDMA_RESET -> poll MON_STA_1 bit8 (warm-reset
 *                     done) -> deassert.
 *   WROT (soft-reset): assert VIDO_SOFT_RST -> poll RST_STAT==1 -> deassert ->
 *                      poll RST_STAT==0.
 */
static void mdp_engine_sw_reset(struct mdp_dev *mdp, struct mdp_comp *comp)
{
	struct device *dev = &mdp->pdev->dev;
	void __iomem *base = comp->regs;
	u32 val;
	int ret;

	if (!base)
		return;

	/* Guarantee clocks + power domain for the reg access; nets out. */
	if (mdp_comp_clock_on(dev, comp))
		return;

	switch (comp->type) {
	case MDP_COMP_TYPE_RDMA:
		writel(MDP_RDMA_RESET_MASK, base + MDP_RDMA_RESET);
		ret = readl_poll_timeout(base + MDP_RDMA_MON_STA_1, val,
					 val & BIT(8), 1, 1000);
		if (ret)
			dev_err(dev, "MDP timeout: RDMA(id=%d) reset ack timeout\n",
				comp->public_id);
		writel(0, base + MDP_RDMA_RESET);
		break;
	case MDP_COMP_TYPE_WROT:
		writel(VIDO_SOFT_RST_MASK, base + VIDO_SOFT_RST);
		ret = readl_poll_timeout(base + VIDO_SOFT_RST_STAT, val,
					 val & BIT(0), 1, 1000);
		if (ret)
			dev_err(dev, "MDP timeout: WROT(id=%d) reset ack timeout\n",
				comp->public_id);
		writel(0, base + VIDO_SOFT_RST);
		readl_poll_timeout(base + VIDO_SOFT_RST_STAT, val,
				   !(val & BIT(0)), 1, 1000);
		break;
	default:
		break;
	}

	mdp_comp_clock_off(dev, comp);
}

/*
 * Per-job GCE watchdog handler. Fires MDP_GCE_TIMEOUT_MS after a successful
 * submit if -- and only if -- the completion callback has NOT already finished
 * the job (mdp_auto_release_work cancels this work + sets job_done on the last
 * packet). Wins the timer-vs-callback race with a single atomic_xchg: if the
 * callback path already claimed finalisation (job_done==1), bail. Otherwise the
 * job is genuinely wedged; mbox_flush() WARM_RESETs each in-flight GCE thread.
 * cmdq_mbox_flush() re-injects the rx_callback (sta=-ECONNABORTED) for every
 * still-busy packet, which queues mdp_auto_release_work -- the SAME completion
 * cleanup (mutex_unprepare, clocks_off, mdp_m2m_job_finish so close() unblocks,
 * pm_runtime_put of the GCE ref, wake_up, job_count->0, pkt destroy + free) runs
 * exactly once per packet. The watchdog itself frees nothing, so there is no
 * double-free even if a packet completed on its own a hair before the flush
 * (then its task is already off the busy list and the flush is a no-op for it).
 */
void mdp_gce_timeout_work(struct work_struct *work)
{
	struct mdp_dev *mdp = container_of(to_delayed_work(work),
					   struct mdp_dev, timeout_work);
	struct device *dev = &mdp->pdev->dev;
	u8 i;

	/* Race winner check: if the normal completion already finalised the
	 * job, do nothing. atomic_xchg(.,1) returns the prior value -- 1 means
	 * the callback path won, 0 means this watchdog won and must recover. */
	if (atomic_xchg(&mdp->job_done, 1))
		return;

	dev_err(dev, "GCE job timed out, reset thread\n");

	/*
	 * Un-brick the physical engines BEFORE the flush. The mbox_flush() below
	 * only warm-resets the GCE thread; it leaves the wedged RDMA/WROT engines
	 * latched, so without this the NEXT mtk-mdp3 job hangs -> reboot. The
	 * in-flight (wedged) job still holds the clocks + pm_runtime ref here
	 * (mdp_comp_clocks_on/mtk_mutex_prepare from submit are dropped only by
	 * the release work the flush queues), so the reg access is valid; the
	 * per-comp clock_on/off in mdp_engine_sw_reset makes it equally safe for
	 * the other (idle) enumerated engines. Doing it before the flush also
	 * keeps it off the clocks_off path the release work runs. The per-job
	 * comp list is not reachable from the per-device timeout_work, so reset
	 * every enumerated RDMA/WROT engine -- an idle engine tolerates the pulse.
	 */
	if (mdp_engine_reset_on_timeout) {
		int c;

		for (c = 0; c < MDP_MAX_COMP_COUNT; c++) {
			struct mdp_comp *comp = mdp->comp[c];

			if (comp && (comp->type == MDP_COMP_TYPE_RDMA ||
				     comp->type == MDP_COMP_TYPE_WROT))
				mdp_engine_sw_reset(mdp, comp);
		}
	}

	/* WARM_RESET each wedged GCE thread. The flush re-injects the
	 * rx_callback for the stuck packet(s), driving the normal
	 * mdp_auto_release_work cleanup which does job_finish + pm_put + frees
	 * (so v4l2_m2m_cancel_job / close() unblocks and job_count reaches 0). */
	for (i = 0; i < mdp->timeout_nchan; i++)
		mbox_flush(mdp->timeout_chan[i], 0);
}

/* Snapshot the distinct in-flight GCE channels and arm the watchdog. Called at
 * the END of a successful submit (after the mbox_send_message loop), so a
 * completion that races in immediately can cancel a fully-initialised timer.
 * job_done is cleared to 0 here (under the just-set job_count) to open the race
 * window; the loser of {last completion, timeout} becomes a no-op via job_done. */
static void mdp_arm_gce_watchdog(struct mdp_dev *mdp,
				 struct mbox_chan **chans, u8 nchan)
{
	u8 i;

	for (i = 0; i < nchan && i < MDP_PP_MAX; i++)
		mdp->timeout_chan[i] = chans[i];
	mdp->timeout_nchan = min_t(u8, nchan, (u8)MDP_PP_MAX);
	atomic_set(&mdp->job_done, 0);
	schedule_delayed_work(&mdp->timeout_work,
			      msecs_to_jiffies(mdp_kd_gce_timeout_ms ?
					       mdp_kd_gce_timeout_ms :
					       MDP_GCE_TIMEOUT_MS));
}

/* GCE instruction-stream dump for the transpose rot270 diff: prints each 8-byte GCE
 * instruction (a=arg_a=op|subsys|offset, b=arg_b=value) of a finalized packet, so the
 * in-job rot270 chunk's packet can be diffed register-by-register against the standalone
 * rot270 packet to localize the cross-chunk wrapper (mutex / GCE event / mmsys mux)
 * difference that the per-chunk config does not re-establish. mdp_kd_pktdump default 0. */
static int mdp_kd_pktdump;
module_param(mdp_kd_pktdump, int, 0644);
MODULE_PARM_DESC(mdp_kd_pktdump, "dump GCE packet instructions for the rot270 diff (default 0)");

static void mdp_kd_dump_pkt_instrs(struct device *dev, struct cmdq_pkt *pkt,
				   const char *tag, int idx)
{
	const u32 *p = pkt->va_base;
	size_t n, i;

	if (!mdp_kd_pktdump || !p)
		return;
	n = pkt->cmd_buf_size / 8;	/* 8 bytes per GCE instruction */
	dev_info(dev, "PKTI %s idx=%d START n=%zu\n", tag, idx, n);
	for (i = 0; i < n; i++)
		dev_info(dev, "PKTI %s i=%zu a=%08x b=%08x\n",
			 tag, i, p[2 * i + 1], p[2 * i]);
	dev_info(dev, "PKTI %s idx=%d END\n", tag, idx);
}

/* mdp_kd_pktreuse: build the GCE packet ONCE for a fixed transform, then on
 * subsequent same-geometry device_runs patch only the src/dst plane iovas in
 * place and re-submit -- skipping the img_config compute + the packet rebuild.
 * Default ON (board-validated bit-exact: reuse-ON output md5 == reuse-OFF over an
 * 8-frame stream on crop/scale/blit/CSC, reuse_fail=0; scoped to fall back to a
 * full rebuild for 4:4:4-hybrid / multi-chunk / DUAL_BITBLT jobs). A V4L2 m2m
 * streaming session with a fixed dims/format/selection is the win case (the
 * transform is constant, only the buffer iovas move); mdp_kernel_direct=N remains
 * the global master switch, and mdp_kd_pktreuse=0 restores the per-frame rebuild. */
bool mdp_kd_pktreuse = true;
module_param(mdp_kd_pktreuse, bool, 0644);
MODULE_PARM_DESC(mdp_kd_pktreuse,
		 "reuse+iova-patch the GCE packet across same-geometry m2m frames (default 1)");

/* Scan a finalized packet for the RDMA-src / WROT-dst plane-iova WRITE
 * instructions and record their in-packet value-word offsets so a later reuse
 * can delta-patch them. src vs dst is disambiguated by value range (the two
 * buffers are distinct, non-overlapping allocations). */
static void mdp_cmdq_record_patch(struct mdp_cmdq_cmd *cmd,
				  struct mdp_cmdq_param *param)
{
	const struct img_ipi_frameparam *p = param->param;
	const u32 *w = cmd->pkt.va_base;
	u32 src = p->inputs[0].buffer.iova[0];
	u32 dst = p->outputs[0].buffer.iova[0];
	u32 ssz = 0, dsz = 0;
	size_t n, i;
	int k;

	cmd->patch_n = 0;
	cmd->src_base = src;
	cmd->dst_base = dst;
	if (!w)
		return;
	for (k = 0; k < IMG_MAX_PLANES; k++) {
		ssz += p->inputs[0].buffer.format.plane_fmt[k].size;
		dsz += p->outputs[0].buffer.format.plane_fmt[k].size;
	}
	n = cmd->pkt.cmd_buf_size / 8;	/* 8 bytes / GCE instruction */
	for (i = 0; i < n; i++) {
		u32 val = w[2 * i];		/* value word (low)  */
		u32 a   = w[2 * i + 1];		/* op<<24 | subsys<<16 | offset */
		/* MM_REG_WRITE emits offset = comp reg_base + reg_ofst; the component
		 * register blocks are 4 KiB-aligned, so the low 12 bits are the raw
		 * register offset (write-enable-mask bit0 dropped). */
		u16 off = ((a & 0xffff) & ~1u) & 0xfff;

		if ((a >> 24) != CMDQ_CODE_WRITE)
			continue;
		if (off != (MDP_RDMA_SRC_BASE_0 & 0xfff) &&
		    off != (MDP_RDMA_SRC_BASE_1 & 0xfff) &&
		    off != (MDP_RDMA_SRC_END_0 & 0xfff) &&
		    off != (MDP_RDMA_SRC_END_1 & 0xfff) &&
		    off != (VIDO_BASE_ADDR & 0xfff) &&
		    off != (VIDO_BASE_ADDR_C & 0xfff))
			continue;
		if (cmd->patch_n >= ARRAY_SIZE(cmd->patch_off))
			break;
		if ((u32)(val - src) <= ssz) {
			cmd->patch_off[cmd->patch_n] = (u16)(8 * i);
			cmd->patch_tag[cmd->patch_n] = 0;	/* src */
			cmd->patch_n++;
		} else if ((u32)(val - dst) <= dsz) {
			cmd->patch_off[cmd->patch_n] = (u16)(8 * i);
			cmd->patch_tag[cmd->patch_n] = 1;	/* dst */
			cmd->patch_n++;
		}
	}
}

/* ------------------------------------------------------------------------
 * MDP-RDMATRACE (WIP-DIAG, default off): the DECISIVE baked-vs-param probe for
 * the SVPP2 MDP_RDMA region-2 iommu fault. At the point the GCE packet is
 * finalized (tag=BUILD, mdp_cmdq_prepare) or delta-patched + re-submitted
 * (tag=REUSE, mdp_cmdq_reuse), read the RDMA SRC_BASE_0/SRC_BASE_1 IMMEDIATE
 * ACTUALLY baked into the packet and print it beside the resolved
 * param.inputs[0].buffer.iova[0..1].
 *
 * CRITICAL FIX vs the earlier scan: the RDMA MDP_RDMA_SRC_BASE_0 (0xf00) /
 * SRC_BASE_1 (0xf08) offsets COLLIDE, in their low 12 bits, with the downstream
 * WROT VIDO_BASE_ADDR (0xf00) / VIDO_BASE_ADDR_V (0xf08) and WDMA WDMA_DST_ADDR
 * (0xf00) / WDMA_DST_V_ADDR (0xf08) -- all component register blocks are 4 KiB
 * aligned, so `& 0xfff` cannot tell the RDMA source apart from the WROT/WDMA
 * DESTINATION. The old scan kept the LAST 0xf00/0xf08 write, which in a
 * RDMA->...->WROT pipeline packet is the WROT DEST base -- so it reported the
 * output Y iova as "baked SRC_BASE_0" (always != the source param.iova, hence
 * every frame "SUSPECT") and the WROT V-plane (0 for 2-plane NV12 out) as
 * "SRC_BASE_1" (always 0, no bearing on the RDMA chroma). That data was a pure
 * measurement artifact.
 *
 * The RDMA is component 0 (configured FIRST); the WROT/WDMA is the LAST
 * component. So the FIRST write to 0xf00/0xf08 is the RDMA source; the LAST is
 * the WROT/WDMA dest. Capture both (with subsys, to make the collision visible)
 * and test SUSPECT against the REAL RDMA source base.
 *
 * This settles the two candidate mechanisms for a faulting frame:
 *   (1) config-compute / packet EMITS 0/stale  -> rdma SRC_BASE is 0 (or
 *       outside [iova, iova+plane_size]) while param.iova is valid, AT BUILD;
 *   (2) runtime corruption (use-after-free / dmabuf use-after-unmap) -> rdma
 *       SRC_BASE == a VALID param.iova at BUILD, yet the RDMA still faults pa=0
 *       (cross-check VB2DC-UNMAP for that iova in videobuf2-dma-contig).
 * mdp_iova_trace=1 logs only structurally SUSPECT frames; =2 logs every job.
 * Ratelimited so a fault storm can't make the console/journal worse. Not for
 * mainline. */
static int mdp_iova_trace;
module_param(mdp_iova_trace, int, 0644);
MODULE_PARM_DESC(mdp_iova_trace,
		 "trace baked RDMA SRC_BASE_0/1 vs param.iova per job (0=off, 1=suspect-only, 2=all)");

static void mdp_iovatrace_scan(struct mdp_dev *mdp, struct mdp_cmdq_cmd *cmd,
			       const struct img_ipi_frameparam *fp,
			       u8 pp_idx, const char *tag)
{
	const u32 *w = cmd->pkt.va_base;
	u32 iova0 = fp->inputs[0].buffer.iova[0];
	u32 iova1 = fp->inputs[0].buffer.iova[1];
	u32 sz0 = fp->inputs[0].buffer.format.plane_fmt[0].size;
	u32 sz1 = fp->inputs[0].buffer.format.plane_fmt[1].size;
	u32 cf  = fp->inputs[0].buffer.format.colorformat;
	u32 rbase0 = 0, rbase1 = 0, wbase0 = 0;	/* rdma src 0/1 ; wrot/wdma dst 0 */
	u8  rsub0 = 0xff, wsub0 = 0xff;
	bool haveR0 = false, haveR1 = false, haveW0 = false;
	bool suspect = false;
	size_t n, i;

	if (!mdp_iova_trace || !w)
		return;

	n = cmd->pkt.cmd_buf_size / 8;	/* 8 bytes / GCE instruction */
	for (i = 0; i < n; i++) {
		u32 val = w[2 * i];		/* value word (baked immediate) */
		u32 a   = w[2 * i + 1];		/* op<<24 | subsys<<16 | offset */
		u8  sub = (a >> 16) & 0xff;
		u16 off = ((a & 0xffff) & ~1u) & 0xfff;

		if ((a >> 24) != CMDQ_CODE_WRITE)
			continue;
		if (off == (MDP_RDMA_SRC_BASE_0 & 0xfff)) {
			if (!haveR0) {		/* FIRST 0xf00 = RDMA source Y */
				rbase0 = val;
				rsub0 = sub;
				haveR0 = true;
			}
			wbase0 = val;		/* LAST 0xf00 = WROT/WDMA dest Y */
			wsub0 = sub;
			haveW0 = true;
		} else if (off == (MDP_RDMA_SRC_BASE_1 & 0xfff)) {
			if (!haveR1) {		/* FIRST 0xf08 = RDMA source chroma */
				rbase1 = val;
				haveR1 = true;
			}
		}
	}

	/* SUSPECT = the REAL RDMA source base is NOT inside [iova_p, iova_p+size]
	 * (0, stale, or offset past the plane). Unsigned wrap makes base=0 (with a
	 * nonzero iova) land far above plane_size, so it is flagged. wbase0 (the
	 * WROT/WDMA dest) is logged only for contrast; it is not the source. */
	if (haveR0 && (u32)(rbase0 - iova0) > sz0)
		suspect = true;
	if (sz1 && haveR1 && (u32)(rbase1 - iova1) > sz1)
		suspect = true;

	if (suspect || mdp_iova_trace >= 2)
		dev_err_ratelimited(&mdp->pdev->dev,
			"MDP-RDMATRACE%s %s frame=%u pp=%u cf=%#x rdma[SRC_BASE_0=%#x(sub%u) SRC_BASE_1=%#x] wrot[BASE_0=%#x(sub%u)] param.iova[0]=%#x iova[1]=%#x psz[%#x,%#x]\n",
			suspect ? "-SUSPECT" : "", tag, fp->frame_no, pp_idx,
			cf, rbase0, rsub0, rbase1, wbase0, wsub0,
			iova0, iova1, sz0, sz1);
}

static struct mdp_cmdq_cmd *mdp_cmdq_prepare(struct mdp_dev *mdp,
					     struct mdp_cmdq_param *param,
					     u8 pp_idx)
{
	struct mdp_path *path = NULL;
	struct mdp_cmdq_cmd *cmd = NULL;
	struct mdp_comp *comps = NULL;
	struct device *dev = &mdp->pdev->dev;
	const int p_id = mdp->mdp_data->mdp_plat_id;
	struct img_config *config;
	struct mtk_mutex *mutex = NULL;
	enum mdp_pipe_id pipe_id;
	int i, ret = -ECANCELED;
	u32 num_comp;

	config = __get_config_offset(mdp, param, pp_idx);
	if (IS_ERR(config)) {
		ret = PTR_ERR(config);
		goto err_uninit;
	}

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto err_uninit;
	}
	/* Set early: mdp_path_config() (below) reads cmd->pp_idx for the per-pipe
	 * cfg8195 debug snapshot so a >1920 DUAL_BITBLT job captures BOTH pp0/pp1. */
	cmd->pp_idx = pp_idx;

	ret = cmdq_pkt_create(mdp->cmdq_clt[pp_idx], &cmd->pkt, SZ_16K);
	if (ret)
		goto err_free_cmd;

	/* Snapshot the per-context CCM (WROT ext-matrix) BEFORE mdp_path_config()
	 * below, since config_wrot_frame() (invoked from it) reads cmd->ccm. The
	 * later cmd->mdp_ctx assignment happens after path config, so it cannot be
	 * used as the channel. Default (kzalloc) is {.en = false} => built-in CSC. */
	if (param->ccm)
		cmd->ccm = *param->ccm;
	/* Snapshot the per-context RSZ coeff-table select (same BEFORE-path_config
	 * reasoning as ccm); config_rsz_frame reads cmd->rsz_table. -1 => default. */
	cmd->rsz_table = param->rsz_table;
	cmd->rsz_kd = param->rsz_kd;

	if (CFG_CHECK(MT8183, p_id)) {
		num_comp = CFG_GET(MT8183, param->config, num_components);
	} else if (CFG_CHECK(MT8195, p_id)) {
		num_comp = CFG_GET(MT8195, param->config, num_components);
	} else {
		ret = -EINVAL;
		goto err_destroy_pkt;
	}

	comps = kcalloc(num_comp, sizeof(*comps), GFP_KERNEL);
	if (!comps) {
		ret = -ENOMEM;
		goto err_destroy_pkt;
	}

	path = kzalloc(sizeof(*path), GFP_KERNEL);
	if (!path) {
		ret = -ENOMEM;
		goto err_free_comps;
	}

	path->mdp_dev = mdp;
	path->config = config;
	path->param = param->param;
	for (i = 0; i < param->param->num_outputs; i++) {
		path->bounds[i].left = 0;
		path->bounds[i].top = 0;
		path->bounds[i].width =
			param->param->outputs[i].buffer.format.width;
		path->bounds[i].height =
			param->param->outputs[i].buffer.format.height;
		path->composes[i] = param->composes[i] ?
			param->composes[i] : &path->bounds[i];
	}
	ret = mdp_path_ctx_init(mdp, path);
	if (ret) {
		dev_err(dev, "mdp_path_ctx_init error %d\n", pp_idx);
		goto err_free_path;
	}

	pipe_id = __get_pipe(mdp, path->comps[0].comp->public_id);
	mutex = __get_mutex(mdp, &mdp->mdp_data->pipe_info[pipe_id]);
	ret = mtk_mutex_prepare(mutex);
	if (ret) {
		dev_err(dev, "Fail to enable mutex %d clk\n", pp_idx);
		goto err_free_path;
	}

	ret = mdp_path_config(mdp, cmd, path);
	if (ret) {
		dev_err(dev, "mdp_path_config error %d\n", pp_idx);
		goto err_free_path;
	}
	cmdq_pkt_eoc(&cmd->pkt);
	cmdq_pkt_jump_rel(&cmd->pkt, CMDQ_INST_SIZE, mdp->cmdq_shift_pa[pp_idx]);
	mdp_kd_dump_pkt_instrs(dev, &cmd->pkt, "single", pp_idx);
	/* DECISIVE probe: the freshly-built packet's baked SRC_BASE vs param.iova
	 * (fresh-build path = detile + first frame of a reused transform). */
	mdp_iovatrace_scan(mdp, cmd, param->param, pp_idx, "BUILD");

	for (i = 0; i < num_comp; i++) {
		s32 inner_id = MDP_COMP_NONE;

		if (CFG_CHECK(MT8183, p_id))
			inner_id = CFG_GET(MT8183, path->config, components[i].type);
		else if (CFG_CHECK(MT8195, p_id))
			inner_id = CFG_GET(MT8195, path->config, components[i].type);

		if (mdp_cfg_comp_is_dummy(mdp, inner_id))
			continue;
		memcpy(&comps[i], path->comps[i].comp,
		       sizeof(struct mdp_comp));
	}

	mdp->cmdq_clt[pp_idx]->client.rx_callback = mdp_handle_cmdq_callback;
	cmd->mdp = mdp;
	cmd->user_cmdq_cb = param->cmdq_cb;
	cmd->user_cb_data = param->cb_data;
	cmd->comps = comps;
	cmd->num_comps = num_comp;
	cmd->mdp_ctx = param->mdp_ctx;
	cmd->pp_idx = pp_idx;

	/* Pin the imported SOURCE mapping for the whole async job (SVPP2 MDP_RDMA
	 * use-after-unmap iommu-fault fix): take a get_dma_buf() ref on each imported
	 * source plane dma_buf, held until the GCE completion callback drops it
	 * (mdp_cmd_put_src_dbufs at cmd destroy). This is done BEFORE the packet is
	 * submitted (in mdp_cmdq_send), so the cmd already owns the refs when its
	 * completion may race in. Skip the reuse-cached path: a cached cmd outlives a
	 * single job and its source changes each reuse, so it must not hold a stale
	 * pin (cacheable transforms are non-block-mode -- not the faulting detile --
	 * and their sources are vb2-MMAP in practice, so src_dbuf_n is 0 anyway). */
	if (!param->reuse_slot) {
		u8 p;

		for (p = 0; p < param->src_dbuf_n && p < IMG_MAX_PLANES; p++) {
			if (param->src_dbuf[p]) {
				get_dma_buf(param->src_dbuf[p]);
				cmd->src_dbuf[cmd->src_dbuf_n++] = param->src_dbuf[p];
			}
		}
	}

	/* Eligible for packet reuse: mark the cmd so the completion callback keeps
	 * it alive, record the iova patch slots, and hand the pointer to the caller
	 * to cache. Only single-pp (reuse_slot set by device_run for that case). */
	if (param->reuse_slot) {
		cmd->reuse = true;
		mdp_cmdq_record_patch(cmd, param);
		*param->reuse_slot = cmd;
	}

	if (mdp_kd_debug_on()) {
		dev_info(dev, "MDPKD-GCE pp=%u pkt: va=%px pa=%pad cmd_buf_size=%zu buf_size=%zu num_comps=%u\n",
			 pp_idx, cmd->pkt.va_base, &cmd->pkt.pa_base,
			 cmd->pkt.cmd_buf_size, cmd->pkt.buf_size, num_comp);
		for (i = 0; i < num_comp; i++) {
			struct mdp_comp *c = path->comps[i].comp;

			if (!c)
				continue;
			dev_info(dev, "MDPKD-GCE comp[%d]: public_id=%d inner_id=%d type=%d subsys=%d reg_base=%pa alias=%u gce_event[SOF=%d EOF=%d]\n",
				 i, c->public_id, c->inner_id, c->type,
				 c->subsys_id, &c->reg_base, c->alias_id,
				 c->gce_event[MDP_GCE_EVENT_SOF],
				 c->gce_event[MDP_GCE_EVENT_EOF]);
		}
	}

	kfree(path);
	return cmd;

err_free_path:
	if (mutex)
		mtk_mutex_unprepare(mutex);
	kfree(path);
err_free_comps:
	kfree(comps);
err_destroy_pkt:
	cmdq_pkt_destroy(mdp->cmdq_clt[pp_idx], &cmd->pkt);
err_free_cmd:
	kfree(cmd);
err_uninit:
	return ERR_PTR(ret);
}

int mdp_cmdq_send(struct mdp_dev *mdp, struct mdp_cmdq_param *param)
{
	struct mdp_cmdq_cmd *cmd[MDP_PP_MAX] = {NULL};
	struct device *dev = &mdp->pdev->dev;
	int i, ret;
	u8 pp_used = __get_pp_num(param->param->type);

	refcount_set(&mdp->job_count, pp_used);
	if (atomic_read(&mdp->suspended)) {
		refcount_set(&mdp->job_count, 0);
		return -ECANCELED;
	}

	/* Hold the GCE runtime-PM ref for the WHOLE async job (paired put in the
	 * completion callback / error path). Without this the GCE autosuspends
	 * (CMDQ_MBOX_AUTOSUSPEND_DELAY_MS=100) mid-flight and long multi-engine
	 * (SVPP) jobs -- e.g. the MM21 detile -- never complete (KI-001). */
	pm_runtime_get_sync(mdp->cmdq_clt[0]->chan->mbox->dev);

	for (i = 0; i < pp_used; i++) {
		cmd[i] = mdp_cmdq_prepare(mdp, param, i);
		if (IS_ERR_OR_NULL(cmd[i])) {
			ret = PTR_ERR(cmd[i]);
			goto err_cancel_job;
		}
	}

	for (i = 0; i < pp_used; i++) {
		ret = mdp_comp_clocks_on(&mdp->pdev->dev, cmd[i]->comps, cmd[i]->num_comps);
		if (ret)
			goto err_clock_off;
	}

	for (i = 0; i < pp_used; i++) {
		dma_sync_single_for_device(mdp->cmdq_clt[i]->chan->mbox->dev,
					   cmd[i]->pkt.pa_base, cmd[i]->pkt.cmd_buf_size,
					   DMA_TO_DEVICE);

		ret = mbox_send_message(mdp->cmdq_clt[i]->chan, &cmd[i]->pkt);
		if (ret < 0) {
			dev_err(dev, "mbox send message fail %d!\n", ret);
			i = pp_used;
			goto err_clock_off;
		}
		mbox_client_txdone(mdp->cmdq_clt[i]->chan, 0);
	}

	/* All pp_used packets are live on the GCE: arm the per-job watchdog over
	 * their distinct channels (one chan per pp). A completion that already
	 * raced in will cancel it from mdp_auto_release_work. */
	{
		struct mbox_chan *chans[MDP_PP_MAX];

		for (i = 0; i < pp_used; i++)
			chans[i] = mdp->cmdq_clt[i]->chan;
		mdp_arm_gce_watchdog(mdp, chans, pp_used);
	}
	return 0;

err_clock_off:
	while (--i >= 0)
		mdp_comp_clocks_off(&mdp->pdev->dev, cmd[i]->comps,
				    cmd[i]->num_comps);
err_cancel_job:
	pm_runtime_mark_last_busy(mdp->cmdq_clt[0]->chan->mbox->dev);
	pm_runtime_put_autosuspend(mdp->cmdq_clt[0]->chan->mbox->dev);
	refcount_set(&mdp->job_count, 0);

	return ret;
}
EXPORT_SYMBOL_GPL(mdp_cmdq_send);

/* Reuse a previously-built single-pp packet: delta-patch the recorded src/dst
 * plane iovas in place and re-submit, mirroring mdp_cmdq_send's PM ref + mutex
 * prepare + clocks_on + watchdog (the completion callback is shared, and keeps
 * the cmd alive because cmd->reuse is set). Skips mdp_vpu_process + the whole
 * mdp_path_config packet rebuild. Returns 0 on submit; on error the caller
 * drops the cache and falls back to a full rebuild. */
int mdp_cmdq_reuse(struct mdp_dev *mdp, struct mdp_cmdq_cmd *cmd,
		   struct img_ipi_frameparam *fp)
{
	struct device *gce_dev = mdp->cmdq_clt[cmd->pp_idx]->chan->mbox->dev;
	u32 nsrc = fp->inputs[0].buffer.iova[0];
	u32 ndst = fp->outputs[0].buffer.iova[0];
	s32 sdelta = (s32)(nsrc - cmd->src_base);
	s32 ddelta = (s32)(ndst - cmd->dst_base);
	u32 *w = cmd->pkt.va_base;
	struct mtk_mutex *mutex;
	enum mdp_pipe_id pipe_id;
	int i, ret;

	if (!cmd->patch_n || !w)
		return -EINVAL;

	refcount_set(&mdp->job_count, 1);
	if (atomic_read(&mdp->suspended)) {
		refcount_set(&mdp->job_count, 0);
		return -ECANCELED;
	}

	/* Patch the recorded iova write values by their per-plane delta; the fixed
	 * crop/plane offset already baked in each value is preserved. */
	for (i = 0; i < cmd->patch_n; i++)
		w[cmd->patch_off[i] / 4] += cmd->patch_tag[i] ? ddelta : sdelta;
	cmd->src_base = nsrc;
	cmd->dst_base = ndst;

	/* DECISIVE probe: the delta-patched packet's baked SRC_BASE vs param.iova
	 * (reuse fast-path = frames 2..N of a fixed-geometry stream). */
	mdp_iovatrace_scan(mdp, cmd, fp, cmd->pp_idx, "REUSE");

	pm_runtime_get_sync(gce_dev);

	pipe_id = __get_pipe(mdp, cmd->comps[0].public_id);
	mutex = __get_mutex(mdp, &mdp->mdp_data->pipe_info[pipe_id]);
	ret = mtk_mutex_prepare(mutex);
	if (ret)
		goto err_pm;

	ret = mdp_comp_clocks_on(&mdp->pdev->dev, cmd->comps, cmd->num_comps);
	if (ret)
		goto err_mutex;

	dma_sync_single_for_device(gce_dev, cmd->pkt.pa_base,
				   cmd->pkt.cmd_buf_size, DMA_TO_DEVICE);
	ret = mbox_send_message(mdp->cmdq_clt[cmd->pp_idx]->chan, &cmd->pkt);
	if (ret < 0) {
		dev_err(&mdp->pdev->dev, "reuse mbox send fail %d!\n", ret);
		goto err_clocks;
	}
	mbox_client_txdone(mdp->cmdq_clt[cmd->pp_idx]->chan, 0);

	{
		struct mbox_chan *chans[1] = { mdp->cmdq_clt[cmd->pp_idx]->chan };

		mdp_arm_gce_watchdog(mdp, chans, 1);
	}
	return 0;

err_clocks:
	mdp_comp_clocks_off(&mdp->pdev->dev, cmd->comps, cmd->num_comps);
err_mutex:
	mtk_mutex_unprepare(mutex);
err_pm:
	pm_runtime_mark_last_busy(gce_dev);
	pm_runtime_put_autosuspend(gce_dev);
	refcount_set(&mdp->job_count, 0);
	return ret;
}
EXPORT_SYMBOL_GPL(mdp_cmdq_reuse);

/* Release a cmd cached for reuse (called by the m2m ctx on geometry change or
 * streamoff, when no job is in flight). */
void mdp_cmdq_reuse_free(struct mdp_cmdq_cmd *cmd)
{
	if (!cmd)
		return;
	mdp_cmd_put_src_dbufs(cmd);	/* no-op: cached cmds never hold the pin */
	cmdq_pkt_destroy(cmd->mdp->cmdq_clt[cmd->pp_idx], &cmd->pkt);
	kfree(cmd->comps);
	kfree(cmd);
}
EXPORT_SYMBOL_GPL(mdp_cmdq_reuse_free);

/* Destroy a built-but-not-completed packet, mirroring mdp_auto_release_work's
 * cleanup (mutex unprepare + optional clocks_off + pkt destroy + frees). Used
 * only on the multi-job error paths -- a submitted packet is freed by its own
 * completion, never here. */
static void mdp_detile_destroy_cmd(struct mdp_dev *mdp,
				   struct mdp_cmdq_cmd *cmd, bool clocks_were_on)
{
	enum mdp_pipe_id pipe_id = __get_pipe(mdp, cmd->comps[0].public_id);
	struct mtk_mutex *mutex =
		__get_mutex(mdp, &mdp->mdp_data->pipe_info[pipe_id]);

	if (clocks_were_on)
		mdp_comp_clocks_off(&mdp->pdev->dev, cmd->comps, cmd->num_comps);
	mtk_mutex_unprepare(mutex);
	mdp_cmd_put_src_dbufs(cmd);	/* release the imported-source pin, if any */
	cmdq_pkt_destroy(mdp->cmdq_clt[0], &cmd->pkt);
	kfree(cmd->comps);
	kfree(cmd);
}

/* HW-iter knob: detile chunk-configs packed into ONE cmdq packet. Fewer packets
 * = less per-packet build/clocks_on/completion CPU. 1 = one chunk/packet (the
 * proven mdp_cmdq_prepare path). 2 fits SZ_16K (a 14-tile chunk ~= 880 instr,
 * 2048 budget). >2 would need a bigger contiguous pkt alloc -- avoid. */
static int mdp_kd_chunks_per_pkt = 1;
module_param(mdp_kd_chunks_per_pkt, int, 0644);
MODULE_PARM_DESC(mdp_kd_chunks_per_pkt,
		 "mt8183 detile: chunks per cmdq packet (1=safe per-chunk; 2 fits SZ_16K)");

/* Build ONE cmdq packet walking k_in_pkt detile chunk-configs (global chunks
 * [base_chunk, base_chunk+k_in_pkt)). Mirrors mdp_cmdq_prepare but loops the
 * configs into the same pkt, emitting EOC/JUMP once. Each config is a full
 * pipeline pass (init -> frame config -> subframe walk), identical to the
 * 14-subframe chain inside one chunk, so the GCE runs them in sequence via the
 * same mux/mutex/SOF-EOF machinery. comps[]/mutex come from chunk 0 (the pipe is
 * identical across chunks). recompute_chunk fills vpu->config per chunk before
 * it is walked (the pkt copies the register writes, so the buffer is reused). */
static struct mdp_cmdq_cmd *
mdp_cmdq_prepare_multi(struct mdp_dev *mdp, struct mdp_cmdq_param *param,
		       u32 base_chunk, u32 k_in_pkt,
		       void (*recompute_chunk)(struct mdp_vpu_dev *,
					       struct img_ipi_frameparam *, u32))
{
	struct mdp_cmdq_cmd *cmd;
	struct mdp_comp *comps = NULL;
	const int p_id = mdp->mdp_data->mdp_plat_id;
	struct mtk_mutex *mutex = NULL;
	int i, ret = -ECANCELED;
	u32 num_comp = 0, c;
	size_t pkt_sz = (k_in_pkt > 2) ? SZ_64K : SZ_16K;

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto err_uninit;
	}
	ret = cmdq_pkt_create(mdp->cmdq_clt[0], &cmd->pkt, pkt_sz);
	if (ret)
		goto err_free_cmd;

	for (c = 0; c < k_in_pkt; c++) {
		struct mdp_path *path;
		struct img_config *config;

		recompute_chunk(&mdp->vpu, param->param, base_chunk + c);
		param->config = mdp->vpu.config;
		config = __get_config_offset(mdp, param, 0);
		if (IS_ERR(config)) {
			ret = PTR_ERR(config);
			goto err_cleanup;
		}
		if (CFG_CHECK(MT8183, p_id)) {
			num_comp = CFG_GET(MT8183, param->config, num_components);
		} else {
			ret = -EINVAL;
			goto err_cleanup;
		}

		path = kzalloc(sizeof(*path), GFP_KERNEL);
		if (!path) {
			ret = -ENOMEM;
			goto err_cleanup;
		}
		path->mdp_dev = mdp;
		path->config = config;
		path->param = param->param;
		for (i = 0; i < param->param->num_outputs; i++) {
			path->bounds[i].left = 0;
			path->bounds[i].top = 0;
			path->bounds[i].width =
				param->param->outputs[i].buffer.format.width;
			path->bounds[i].height =
				param->param->outputs[i].buffer.format.height;
			path->composes[i] = param->composes[i] ?
				param->composes[i] : &path->bounds[i];
		}
		ret = mdp_path_ctx_init(mdp, path);
		if (ret) {
			kfree(path);
			goto err_cleanup;
		}

		if (c == 0) {
			enum mdp_pipe_id pipe_id =
				__get_pipe(mdp, path->comps[0].comp->public_id);

			comps = kcalloc(num_comp, sizeof(*comps), GFP_KERNEL);
			if (!comps) {
				kfree(path);
				ret = -ENOMEM;
				goto err_cleanup;
			}
			mutex = __get_mutex(mdp, &mdp->mdp_data->pipe_info[pipe_id]);
			ret = mtk_mutex_prepare(mutex);
			if (ret) {
				kfree(path);
				goto err_cleanup;
			}
		}

		ret = mdp_path_config(mdp, cmd, path);
		if (ret) {
			kfree(path);
			goto err_cleanup;
		}

		if (c == 0) {
			for (i = 0; i < num_comp; i++) {
				s32 inner = MDP_COMP_NONE;

				if (CFG_CHECK(MT8183, p_id))
					inner = CFG_GET(MT8183, path->config,
							components[i].type);
				if (mdp_cfg_comp_is_dummy(mdp, inner))
					continue;
				memcpy(&comps[i], path->comps[i].comp,
				       sizeof(struct mdp_comp));
			}
		}
		kfree(path);
	}

	cmdq_pkt_eoc(&cmd->pkt);
	cmdq_pkt_jump_rel(&cmd->pkt, CMDQ_INST_SIZE, mdp->cmdq_shift_pa[0]);
	mdp_kd_dump_pkt_instrs(&mdp->pdev->dev, &cmd->pkt, "chunk", base_chunk);
	mdp->cmdq_clt[0]->client.rx_callback = mdp_handle_cmdq_callback;
	cmd->mdp = mdp;
	cmd->user_cmdq_cb = param->cmdq_cb;
	cmd->user_cb_data = param->cb_data;
	cmd->comps = comps;
	cmd->num_comps = num_comp;
	cmd->mdp_ctx = param->mdp_ctx;
	cmd->pp_idx = 0;
	return cmd;

err_cleanup:
	if (mutex)
		mtk_mutex_unprepare(mutex);
	kfree(comps);
	cmdq_pkt_destroy(mdp->cmdq_clt[0], &cmd->pkt);
err_free_cmd:
	kfree(cmd);
err_uninit:
	return ERR_PTR(ret);
}

/*
 * Async multi-job submit for the mt8183 kernel-direct detile. Builds
 * ceil(nchunks/mdp_kd_chunks_per_pkt) cmdq
 * packet per chunk (recomputing vpu->config per chunk via recompute_chunk; the
 * pkt copies the register writes so the shared config buffer is reused), then
 * submits ALL packets to the single GCE thread, which chains them in submit
 * order -- so chunk k's WROT EOF executes before chunk k+1's RDMA (the
 * inter-chunk DRAM barrier holds without a driver wait). ONE pm_runtime ref +
 * ONE job_count (= nchunks) for the whole frame; only the LAST packet carries
 * mdp_ctx, so exactly one completion (job_count -> 0) runs v4l2_m2m_buf_done.
 * Replaces the per-chunk submit+wait loop, removing (nchunks-1) wait_event
 * round-trips + pm_runtime cycles -- the bulk of the per-frame CPU.
 */
int mdp_cmdq_send_detile(struct mdp_dev *mdp, struct mdp_cmdq_param *param,
			 u32 nchunks,
			 void (*recompute_chunk)(struct mdp_vpu_dev *vpu,
						 struct img_ipi_frameparam *p,
						 u32 chunk_idx))
{
	struct device *dev = &mdp->pdev->dev;
	struct mdp_cmdq_cmd **cmd;
	struct device *gce_dev = mdp->cmdq_clt[0]->chan->mbox->dev;
	u32 K = mdp_kd_chunks_per_pkt > 0 ? mdp_kd_chunks_per_pkt : 1;
	u32 npkts = DIV_ROUND_UP(nchunks, K);
	u32 c;
	int i, ret, sent = 0;

	if (atomic_read(&mdp->suspended))
		return -ECANCELED;

	cmd = kcalloc(npkts, sizeof(*cmd), GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	refcount_set(&mdp->job_count, npkts);
	pm_runtime_get_sync(gce_dev);

	for (c = 0; c < npkts; c++) {
		u32 base = c * K;
		u32 k = min(K, nchunks - base);

		/* k==1 uses the proven single-config prepare; k>1 packs k configs
		 * into one packet (fewer packets -> less per-packet CPU). */
		if (k == 1) {
			recompute_chunk(&mdp->vpu, param->param, base);
			param->config = mdp->vpu.config;
			cmd[c] = mdp_cmdq_prepare(mdp, param, 0);
		} else {
			cmd[c] = mdp_cmdq_prepare_multi(mdp, param, base, k,
							recompute_chunk);
		}
		if (IS_ERR_OR_NULL(cmd[c])) {
			ret = IS_ERR(cmd[c]) ? PTR_ERR(cmd[c]) : -EINVAL;
			cmd[c] = NULL;
			goto err_destroy_built;
		}
		cmd[c]->mdp_ctx = (c == npkts - 1) ? param->mdp_ctx : NULL;
	}

	for (i = 0; i < (int)npkts; i++) {
		ret = mdp_comp_clocks_on(dev, cmd[i]->comps, cmd[i]->num_comps);
		if (ret)
			goto err_clocks_off;
	}

	for (i = 0; i < (int)npkts; i++) {
		dma_sync_single_for_device(gce_dev, cmd[i]->pkt.pa_base,
					   cmd[i]->pkt.cmd_buf_size, DMA_TO_DEVICE);
		ret = mbox_send_message(mdp->cmdq_clt[0]->chan, &cmd[i]->pkt);
		if (ret < 0) {
			dev_err(dev, "mbox send fail %d at pkt %d/%u\n",
				ret, i, npkts);
			goto err_sent_partial;
		}
		mbox_client_txdone(mdp->cmdq_clt[0]->chan, 0);
		sent++;
	}

	/* All npkts packets are chained on the single GCE thread (cmdq_clt[0]);
	 * arm the watchdog over that one channel. The last packet's completion
	 * (the mdp_ctx one) cancels it via mdp_auto_release_work. A flush on a
	 * wedge re-injects callbacks for every still-busy packet on the thread,
	 * so all npkts decrements run and job_count reaches 0. */
	{
		struct mbox_chan *chan = mdp->cmdq_clt[0]->chan;

		mdp_arm_gce_watchdog(mdp, &chan, 1);
	}
	kfree(cmd);
	return 0;

err_clocks_off:
	while (--i >= 0)
		mdp_comp_clocks_off(dev, cmd[i]->comps, cmd[i]->num_comps);
err_destroy_built:
	/* Nothing submitted: destroy every built packet (clocks already off),
	 * drop the pm ref + zero job_count. */
	for (c = 0; c < npkts; c++)
		if (cmd[c])
			mdp_detile_destroy_cmd(mdp, cmd[c], false);
	pm_runtime_mark_last_busy(gce_dev);
	pm_runtime_put_autosuspend(gce_dev);
	refcount_set(&mdp->job_count, 0);
	kfree(cmd);
	return ret;

err_sent_partial:
	/* Rare. Packets [0,sent) are live on the GCE and self-clean via their
	 * completions (each decrements job_count). Destroy the unsent [sent,N)
	 * (clocks were on) and decrement job_count for each; whichever decrement
	 * (a completion or ours) reaches 0 drops the pm ref + wakes the wq. The
	 * mdp_ctx packet is the unsent last one, so no completion runs buf_done --
	 * device_run's worker_end does buf_done(ERROR). */
	for (c = sent; c < npkts; c++) {
		mdp_detile_destroy_cmd(mdp, cmd[c], true);
		if (refcount_dec_and_test(&mdp->job_count)) {
			pm_runtime_mark_last_busy(gce_dev);
			pm_runtime_put_autosuspend(gce_dev);
			wake_up(&mdp->callback_wq);
		}
	}
	kfree(cmd);
	return ret;
}
EXPORT_SYMBOL_GPL(mdp_cmdq_send_detile);
