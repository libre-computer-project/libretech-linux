// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 MediaTek Inc.
 * Author: Ping-Hsun Wu <ping-hsun.wu@mediatek.com>
 */

#include <linux/remoteproc.h>
#include <linux/remoteproc/mtk_scp.h>
#include <linux/moduleparam.h>
#include <linux/ratelimit.h>
#include "mtk-mdp3-vpu.h"
#include "mtk-mdp3-core.h"
#include "mtk-mdp3-regs.h"	/* MDP_STREAM_TYPE_DUAL_BITBLT */

#define MDP_VPU_MESSAGE_TIMEOUT 500U

static inline struct mdp_dev *vpu_to_mdp(struct mdp_vpu_dev *vpu)
{
	return container_of(vpu, struct mdp_dev, vpu);
}

/* kernel-direct GCE-exec debug: dump computed img_config + GCE packet/engine bases
 * (default off). Toggle: echo 1 > /sys/module/mtk_mdp3/parameters/mdp_kd_debug */
static bool mdp_kd_debug;
module_param(mdp_kd_debug, bool, 0644);
MODULE_PARM_DESC(mdp_kd_debug,
	"kernel-direct GCE-exec debug: dump computed img_config + GCE packet/engine bases (default off)");

/* Transpose debug: the last transpose's {W,H,W',H'} (set by mdp_kd_tpose_pass chunk 0),
 * so the post-GCE mdp_vpu_scratch_dump can sample scratch_a (HxW) + scratch_b (H'xW'). */
static u32 mdp_kd_tpose_dims[4];

static int mdp_vpu_shared_mem_alloc(struct mdp_vpu_dev *vpu)
{
	struct device *dev;

	if (IS_ERR_OR_NULL(vpu))
		goto err_return;

	/* No SCP (kernel-direct only): the img_config buffer is CPU-computed and
	 * CPU-read, so it lives on the MDP device -- it never needs the SCP or its
	 * DMA domain. The param (frameparam IPI) and work (SCP scratch) buffers are
	 * SCP-only and are skipped. */
	dev = vpu->scp ? scp_get_device(vpu->scp) : &vpu_to_mdp(vpu)->pdev->dev;

	if (vpu->scp && !vpu->param) {
		vpu->param = dma_alloc_wc(dev, vpu->param_size,
					  &vpu->param_addr, GFP_KERNEL);
		if (!vpu->param)
			goto err_return;
	}

	if (vpu->scp && !vpu->work) {
		vpu->work = dma_alloc_wc(dev, vpu->work_size,
					 &vpu->work_addr, GFP_KERNEL);
		if (!vpu->work)
			goto err_free_param;
	}

	if (!vpu->config) {
		vpu->config = dma_alloc_wc(dev, vpu->config_size,
					   &vpu->config_addr, GFP_KERNEL);
		if (!vpu->config)
			goto err_free_work;
	}

	return 0;

err_free_work:
	dma_free_wc(dev, vpu->work_size, vpu->work, vpu->work_addr);
	vpu->work = NULL;
err_free_param:
	dma_free_wc(dev, vpu->param_size, vpu->param, vpu->param_addr);
	vpu->param = NULL;
err_return:
	return -ENOMEM;
}

void mdp_vpu_shared_mem_free(struct mdp_vpu_dev *vpu)
{
	struct device *dev;

	if (IS_ERR_OR_NULL(vpu))
		return;

	dev = vpu->scp ? scp_get_device(vpu->scp) : &vpu_to_mdp(vpu)->pdev->dev;

	if (vpu->param && vpu->param_addr)
		dma_free_wc(dev, vpu->param_size, vpu->param, vpu->param_addr);

	if (vpu->work && vpu->work_addr)
		dma_free_wc(dev, vpu->work_size, vpu->work, vpu->work_addr);

	if (vpu->config && vpu->config_addr)
		dma_free_wc(dev, vpu->config_size, vpu->config, vpu->config_addr);

	if (vpu->scratch_a)
		dma_free_wc(&vpu_to_mdp(vpu)->pdev->dev, vpu->scratch_size,
			    vpu->scratch_a, vpu->scratch_a_addr);
	if (vpu->scratch_b)
		dma_free_wc(&vpu_to_mdp(vpu)->pdev->dev, vpu->scratch_size,
			    vpu->scratch_b, vpu->scratch_b_addr);
}

/*
 * Ensure two I420-sized scratch buffers exist for the mt8183 5-pass NV12 rotate.
 * dma_alloc_wc on the MDP pdev -> iovas in the same IOMMU domain the RDMA/WROT use
 * (the device that backs the vb2 pixel buffers + vpu->config). Re-allocates if the
 * required size grew. Returns 0 on success. Freed in mdp_vpu_shared_mem_free. */
static int mdp_vpu_scratch_ensure(struct mdp_vpu_dev *vpu, size_t size)
{
	struct device *dev = &vpu_to_mdp(vpu)->pdev->dev;

	if (vpu->scratch_a && vpu->scratch_b && vpu->scratch_size >= size)
		return 0;
	if (vpu->scratch_a)
		dma_free_wc(dev, vpu->scratch_size, vpu->scratch_a, vpu->scratch_a_addr);
	if (vpu->scratch_b)
		dma_free_wc(dev, vpu->scratch_size, vpu->scratch_b, vpu->scratch_b_addr);
	vpu->scratch_a = vpu->scratch_b = NULL;
	vpu->scratch_a = dma_alloc_wc(dev, size, &vpu->scratch_a_addr, GFP_KERNEL);
	vpu->scratch_b = dma_alloc_wc(dev, size, &vpu->scratch_b_addr, GFP_KERNEL);
	if (!vpu->scratch_a || !vpu->scratch_b) {
		if (vpu->scratch_a)
			dma_free_wc(dev, size, vpu->scratch_a, vpu->scratch_a_addr);
		if (vpu->scratch_b)
			dma_free_wc(dev, size, vpu->scratch_b, vpu->scratch_b_addr);
		vpu->scratch_a = vpu->scratch_b = NULL;
		return -ENOMEM;
	}
	vpu->scratch_size = size;
	return 0;
}

/* Debug: dump scratch_a I420 plane samples after a 5-pass batch (call post-GCE).
 * Assumes the test's 240x160 geometry (Y@0, U@w*h=38400, V@+wc*hc=48000). */
void mdp_vpu_scratch_dump(struct mdp_vpu_dev *vpu)
{
	const u8 *a = vpu->scratch_a, *b = vpu->scratch_b;
	u32 W = mdp_kd_tpose_dims[0], H = mdp_kd_tpose_dims[1];
	u32 Wp = mdp_kd_tpose_dims[2], Hp = mdp_kd_tpose_dims[3];
	struct device *dev = &vpu_to_mdp(vpu)->pdev->dev;

	if (!mdp_kd_debug || !a || !b || !H || !Hp)
		return;
	/* scratch_a = rot90(input) = H wide x W tall, stride H. col0 (byte row*H) is the
	 * row axis = the input WIDTH after rot90 (the drifting axis); row0 = bytes 0..7.
	 * scratch_b = V-scale(scratch_a) = H' wide x W' tall, stride H'. */
	dev_info(dev, "SA(%ux%u) col0[r=0,W/8,..]=%u %u %u %u %u %u %u %u\n", H, W,
		 a[0], a[(W/8)*H], a[(2*W/8)*H], a[(3*W/8)*H],
		 a[(4*W/8)*H], a[(5*W/8)*H], a[(6*W/8)*H], a[(7*W/8)*H]);
	dev_info(dev, "SA row0[c=0..7]=%u %u %u %u %u %u %u %u\n",
		 a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7]);
	dev_info(dev, "SB(%ux%u) col0[r=0,W'/8,..]=%u %u %u %u %u %u %u %u\n", Hp, Wp,
		 b[0], b[(Wp/8)*Hp], b[(2*Wp/8)*Hp], b[(3*Wp/8)*Hp],
		 b[(4*Wp/8)*Hp], b[(5*Wp/8)*Hp], b[(6*Wp/8)*Hp], b[(7*Wp/8)*Hp]);
	dev_info(dev, "SB row0[c=0..7]=%u %u %u %u %u %u %u %u\n",
		 b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]);
	/* The drifting output row0 maps via rot270 to scratch_b's HIGH columns; sample
	 * col(H'-1) + col(H'/2), varying row, to split scale-bug (high cols drift) from
	 * rot270-bug (high cols clean). */
	dev_info(dev, "SB colHi[c=%u,r=0,W'/8,..]=%u %u %u %u %u %u %u %u\n", Hp - 1,
		 b[Hp - 1], b[(Wp/8)*Hp + Hp-1], b[(2*Wp/8)*Hp + Hp-1], b[(3*Wp/8)*Hp + Hp-1],
		 b[(4*Wp/8)*Hp + Hp-1], b[(5*Wp/8)*Hp + Hp-1], b[(6*Wp/8)*Hp + Hp-1], b[(7*Wp/8)*Hp + Hp-1]);
	dev_info(dev, "SB colMid[c=%u,r=0,W'/8,..]=%u %u %u %u %u %u %u %u\n", Hp/2,
		 b[Hp/2], b[(Wp/8)*Hp + Hp/2], b[(2*Wp/8)*Hp + Hp/2], b[(3*Wp/8)*Hp + Hp/2],
		 b[(4*Wp/8)*Hp + Hp/2], b[(5*Wp/8)*Hp + Hp/2], b[(6*Wp/8)*Hp + Hp/2], b[(7*Wp/8)*Hp + Hp/2]);
}

static void mdp_vpu_ipi_handle_init_ack(void *data, unsigned int len,
					void *priv)
{
	struct mdp_ipi_init_msg *msg = (struct mdp_ipi_init_msg *)data;
	struct mdp_vpu_dev *vpu =
		(struct mdp_vpu_dev *)(unsigned long)msg->drv_data;

	if (!vpu->work_size)
		vpu->work_size = msg->work_size;

	vpu->status = msg->status;
	complete(&vpu->ipi_acked);
}

static void mdp_vpu_ipi_handle_deinit_ack(void *data, unsigned int len,
					  void *priv)
{
	struct mdp_ipi_deinit_msg *msg = (struct mdp_ipi_deinit_msg *)data;
	struct mdp_vpu_dev *vpu =
		(struct mdp_vpu_dev *)(unsigned long)msg->drv_data;

	vpu->status = msg->status;
	complete(&vpu->ipi_acked);
}

static void mdp_vpu_ipi_handle_frame_ack(void *data, unsigned int len,
					 void *priv)
{
	struct img_sw_addr *addr = (struct img_sw_addr *)data;
	struct img_ipi_frameparam *param =
		(struct img_ipi_frameparam *)(unsigned long)addr->va;
	struct mdp_vpu_dev *vpu =
		(struct mdp_vpu_dev *)(unsigned long)param->drv_data;

	if (param->state) {
		struct mdp_dev *mdp = vpu_to_mdp(vpu);

		dev_err(&mdp->pdev->dev, "VPU MDP failure:%d\n", param->state);
	}
	vpu->status = param->state;
	complete(&vpu->ipi_acked);
}

int mdp_vpu_register(struct mdp_dev *mdp)
{
	int err;
	struct mtk_scp *scp = mdp->scp;
	struct device *dev = &mdp->pdev->dev;

	err = scp_ipi_register(scp, SCP_IPI_MDP_INIT,
			       mdp_vpu_ipi_handle_init_ack, NULL);
	if (err) {
		dev_err(dev, "scp_ipi_register failed %d\n", err);
		goto err_ipi_init;
	}
	err = scp_ipi_register(scp, SCP_IPI_MDP_DEINIT,
			       mdp_vpu_ipi_handle_deinit_ack, NULL);
	if (err) {
		dev_err(dev, "scp_ipi_register failed %d\n", err);
		goto err_ipi_deinit;
	}
	err = scp_ipi_register(scp, SCP_IPI_MDP_FRAME,
			       mdp_vpu_ipi_handle_frame_ack, NULL);
	if (err) {
		dev_err(dev, "scp_ipi_register failed %d\n", err);
		goto err_ipi_frame;
	}
	return 0;

err_ipi_frame:
	scp_ipi_unregister(scp, SCP_IPI_MDP_DEINIT);
err_ipi_deinit:
	scp_ipi_unregister(scp, SCP_IPI_MDP_INIT);
err_ipi_init:

	return err;
}

void mdp_vpu_unregister(struct mdp_dev *mdp)
{
	scp_ipi_unregister(mdp->scp, SCP_IPI_MDP_INIT);
	scp_ipi_unregister(mdp->scp, SCP_IPI_MDP_DEINIT);
	scp_ipi_unregister(mdp->scp, SCP_IPI_MDP_FRAME);
}

static int mdp_vpu_sendmsg(struct mdp_vpu_dev *vpu, enum scp_ipi_id id,
			   void *buf, unsigned int len)
{
	struct mdp_dev *mdp = vpu_to_mdp(vpu);
	unsigned int t = MDP_VPU_MESSAGE_TIMEOUT;
	int ret;

	if (!vpu->scp) {
		dev_dbg(&mdp->pdev->dev, "vpu scp is NULL");
		return -EINVAL;
	}
	ret = scp_ipi_send(vpu->scp, id, buf, len, 2000);

	if (ret) {
		dev_err(&mdp->pdev->dev, "scp_ipi_send failed %d\n", ret);
		return -EPERM;
	}
	ret = wait_for_completion_timeout(&vpu->ipi_acked,
					  msecs_to_jiffies(t));
	if (!ret)
		ret = -ETIME;
	else if (vpu->status)
		ret = -EINVAL;
	else
		ret = 0;
	return ret;
}

int mdp_vpu_dev_init(struct mdp_vpu_dev *vpu, struct mtk_scp *scp,
		     struct mutex *lock)
{
	struct mdp_ipi_init_msg msg = {
		.drv_data = (unsigned long)vpu,
	};
	struct mdp_dev *mdp = vpu_to_mdp(vpu);
	int err;
	u8 pp_num = mdp->mdp_data->pp_used;

	init_completion(&vpu->ipi_acked);
	vpu->scp = scp;
	vpu->lock = lock;

	if (!scp) {
		/* Kernel-direct only: no SCP INIT handshake and no SCP-side
		 * param/work buffers -- just the CPU-computed img_config buffer (on
		 * the MDP device). The detile path owns the engines via the GCE. */
		mutex_lock(vpu->lock);
		vpu->param_size = 0;
		vpu->work_size = 0;
		vpu->config_size = ALIGN(sizeof(struct img_config) * pp_num, 64);
		err = mdp_vpu_shared_mem_alloc(vpu);
		mutex_unlock(vpu->lock);
		if (err)
			dev_err(&mdp->pdev->dev, "kernel-direct config alloc failed\n");
		return err;
	}

	vpu->work_size = 0;
	err = mdp_vpu_sendmsg(vpu, SCP_IPI_MDP_INIT, &msg, sizeof(msg));
	if (err)
		goto err_work_size;
	/* vpu work_size was set in mdp_vpu_ipi_handle_init_ack */

	mutex_lock(vpu->lock);
	vpu->work_size = ALIGN(vpu->work_size, 64);
	vpu->param_size = ALIGN(sizeof(struct img_ipi_frameparam), 64);
	vpu->config_size = ALIGN(sizeof(struct img_config) * pp_num, 64);
	err = mdp_vpu_shared_mem_alloc(vpu);
	mutex_unlock(vpu->lock);
	if (err) {
		dev_err(&mdp->pdev->dev, "VPU memory alloc fail!");
		goto err_mem_alloc;
	}

	dev_dbg(&mdp->pdev->dev,
		"VPU param:%p pa:%pad sz:%zx, work:%p pa:%pad sz:%zx, config:%p pa:%pad sz:%zx",
		vpu->param, &vpu->param_addr, vpu->param_size,
		vpu->work, &vpu->work_addr, vpu->work_size,
		vpu->config, &vpu->config_addr, vpu->config_size);

	msg.work_addr = vpu->work_addr;
	msg.work_size = vpu->work_size;
	err = mdp_vpu_sendmsg(vpu, SCP_IPI_MDP_INIT, &msg, sizeof(msg));
	if (err)
		goto err_work_size;

	return 0;

err_work_size:
	switch (vpu->status) {
	case -MDP_IPI_EBUSY:
		err = -EBUSY;
		break;
	case -MDP_IPI_ENOMEM:
		err = -ENOSPC;	/* -ENOMEM */
		break;
	}
	return err;
err_mem_alloc:
	return err;
}

int mdp_vpu_dev_deinit(struct mdp_vpu_dev *vpu)
{
	struct mdp_ipi_deinit_msg msg = {
		.drv_data = (unsigned long)vpu,
		.work_addr = vpu->work_addr,
	};

	return mdp_vpu_sendmsg(vpu, SCP_IPI_MDP_DEINIT, &msg, sizeof(msg));
}

/* Free param/work/config from the device that owns them: the SCP while it is
 * attached, the MDP device in kernel-direct mode. Caller holds vpu->lock. */
static void mdp_vpu_ipi_mem_free(struct mdp_vpu_dev *vpu)
{
	struct device *dev = vpu->scp ? scp_get_device(vpu->scp) :
					&vpu_to_mdp(vpu)->pdev->dev;

	if (vpu->param)
		dma_free_wc(dev, vpu->param_size, vpu->param, vpu->param_addr);
	if (vpu->work)
		dma_free_wc(dev, vpu->work_size, vpu->work, vpu->work_addr);
	if (vpu->config)
		dma_free_wc(dev, vpu->config_size, vpu->config, vpu->config_addr);
	vpu->param = NULL;
	vpu->work = NULL;
	vpu->config = NULL;
}

/*
 * Attach the SCP for a frame no kernel-direct branch claimed: boot it,
 * register the MDP IPIs and run the INIT handshake. MDP opens without the
 * SCP, so on a board where every op is kernel-direct the SCP (and its
 * firmware) is never touched. The SCP writes the config itself, so the
 * kernel-direct config buffer is reallocated in the SCP's DMA domain.
 * Caller holds vpu->attach_lock.
 */
static int mdp_vpu_scp_attach(struct mdp_vpu_dev *vpu)
{
	struct mdp_dev *mdp = vpu_to_mdp(vpu);
	struct device *dev = &mdp->pdev->dev;
	int ret;

	if (!mdp->scp || !mdp->rproc_handle) {
		dev_err(dev, "op needs the SCP config fallback; no SCP\n");
		return -ENODEV;
	}
	ret = rproc_boot(mdp->rproc_handle);
	if (ret) {
		dev_err(dev, "SCP boot for the MDP config fallback failed %d\n",
			ret);
		return ret;
	}
	ret = mdp_vpu_register(mdp);
	if (ret)
		goto err_shutdown;

	mutex_lock(vpu->lock);
	mdp_vpu_ipi_mem_free(vpu);
	mutex_unlock(vpu->lock);

	ret = mdp_vpu_dev_init(vpu, mdp->scp, vpu->lock);
	if (ret) {
		dev_err(dev, "SCP MDP init failed %d\n", ret);
		goto err_unregister;
	}
	dev_info(dev, "SCP attached for the MDP config fallback\n");
	return 0;

err_unregister:
	mutex_lock(vpu->lock);
	mdp_vpu_ipi_mem_free(vpu);
	vpu->scp = NULL;
	mutex_unlock(vpu->lock);
	mdp_vpu_unregister(mdp);
err_shutdown:
	rproc_shutdown(mdp->rproc_handle);
	return ret;
}

/* Last close: if the SCP was attached, end its MDP session and release it. */
void mdp_vpu_scp_detach(struct mdp_vpu_dev *vpu)
{
	struct mdp_dev *mdp = vpu_to_mdp(vpu);

	mutex_lock(&vpu->attach_lock);
	if (vpu->scp) {
		mdp_vpu_dev_deinit(vpu);
		mdp_vpu_unregister(mdp);
		mutex_lock(vpu->lock);
		mdp_vpu_ipi_mem_free(vpu);
		vpu->scp = NULL;
		mutex_unlock(vpu->lock);
		rproc_shutdown(mdp->rproc_handle);
	}
	mutex_unlock(&vpu->attach_lock);
}

/* Stage the frameparam + work buffers for the SCP FRAME IPI. */
static void mdp_vpu_stage_frame(struct mdp_vpu_dev *vpu,
				struct img_ipi_frameparam *param,
				struct img_sw_addr *addr)
{
	memset(vpu->param, 0, vpu->param_size);
	memset(vpu->work, 0, vpu->work_size);
	param->config_data.va = (unsigned long)vpu->config;
	param->config_data.pa = vpu->config_addr;
	param->self_data.va = (unsigned long)vpu->work;
	param->self_data.pa = vpu->work_addr;
	memcpy(vpu->param, param, sizeof(*param));
	addr->pa = vpu->param_addr;
	addr->va = (unsigned long)vpu->param;
}

/*
 * Kernel-direct MDP config-compute (mt8195 MM21->NV12 detile).
 *
 * Reproduces the SCP-computed img_config for the single-tile (num_subfrm=1)
 * MM21 block-mode -> linear NV12 detile from geometry alone, so the
 * SCP_IPI_MDP_FRAME round-trip can be bypassed (gated by mdp_kernel_direct=1).
 * The 10-component SVPP2 pipe (RDMA->RSZ->relay->PQ chain->WROT), the per-comp
 * frame/subfrm geometry, and the engine register udata were reverse-engineered
 * and offline-validated bit-exact vs SCP oracles at 1280x720 / 640x360 /
 * 1280x360 (10/10 components); the word-level fills below mirror that validator.
 * The DAG (input=0,num_out=1,out0=0 for every comp) and the 7-set SVPP2 mmsys
  * mux ctrls[0] are geometry-independent constants captured from the same
  * oracles.
 */
/*
 * Default ON: the in-kernel detile is HW-validated bit-exact vs an ffmpeg
 * software decode across single-pipe, dual-pipe, and 4K (mtk-mdp-kernel-direct-
 * sweep.sh, 19/19). The gate (mdp_frame_is_mm21_detile + the per-mode size
 * bounds in mdp_vpu_process) routes only the pure 64-aligned-width MM21->NV12
 * detile here; everything else -- scale/rotate/non-MM21, non-64-aligned widths
 * (MM21 luma stride = ALIGN(w,64)*32 != w*32), out width > 3840 -- still falls
 * through to the SCP. Set mdp_kernel_direct=0 to force the old SCP path (escape
 * hatch for an unforeseen field regression).
 */
static bool mdp_kernel_direct = true;
module_param(mdp_kernel_direct, bool, 0644);
MODULE_PARM_DESC(mdp_kernel_direct,
	"mt8195/mt8183: compute the MM21->NV12 detile img_config in-kernel, bypassing the SCP (default on; =0 forces SCP)");

/*
 * GCE-exec debug: when set, dump the kernel-direct computed img_config key
 * fields (RDMA src_ctrl/control/iova[]/mf_bkgd, WROT iova/control/stride, the
 * subfrm src/clip, and the ctrls[] mmsys mux) plus, in mdp_cmdq_send, the GCE
 * packet pa_base/size and per-component engine reg_base/subsys_id/gce_event.
 * Non-default; lets the coordinator /dev/mem-probe the engine MMIO at a hang to
 * see which engine is mis-programmed / never produced its EOF. Output is at
 * dev_info level, gated entirely behind this param (=0 => silent). The mt8195
 * MDPDBG analog. Toggle live: echo 1 > /sys/module/mtk_mdp3/parameters/mdp_kd_debug
 */
/*
 * Rate-limit the entire mdp_kd_debug print family so a stream physically cannot
 * flood the log. A dbg=1 stream emitted tens of thousands of per-op / per-subframe
 * / per-component KERN_INFO lines (config-compute, MSF, RDMA/RSZ-SUB, GCE, BATCH).
 * At console_loglevel=7 those reach console=ttyS0 (the SPI-backed AP serial
 * console): saturating it starves the SPI controller and has exposed latent kernel
 * bugs elsewhere -- an account_kernel_stack memcg NULL-deref (reported once) and,
 * board-repro on mt8195, an mtk_spi_interrupt_thread (spi_mt65xx) write-to-RO oops
 * after cros-ec-spi transfer timeouts. The full per-op detail lives losslessly in
 * the debugfs CFGHEX oracle (mtk-mdp3/cfg8195_snapshot, via mdp_kd_debug_raw(),
 * unaffected by the limiter), so the printk only needs to be a bounded liveness
 * SAMPLE. burst=30/5s caps it at ~6 gate-passes/s (~10 lines/s, <10% of the
 * ~128 line/s 115200 serial ceiling) with the usual "N callbacks suppressed"
 * summary. dbg=0 short-circuits before __ratelimit() (zero cost in production).
 */
static DEFINE_RATELIMIT_STATE(mdp_kd_rs, 5 * HZ, 30);
bool mdp_kd_debug_on(void)
{
	return mdp_kd_debug && __ratelimit(&mdp_kd_rs);
}
EXPORT_SYMBOL_GPL(mdp_kd_debug_on);

/* Un-limited knob read: ONLY for the debugfs CFGHEX snapshot capture, which must
 * refresh on every op while the knob is on (read on-demand, never printed per-op). */
bool mdp_kd_debug_raw(void) { return mdp_kd_debug; }
EXPORT_SYMBOL_GPL(mdp_kd_debug_raw);

/* HW-iter sweep knob: force the per-tile WROT main_buf line_num to this value
 * (0 = use the calWROTSetting-computed value). Lets the coordinator sweep the WROT
 * FIFO strip height in a single boot instead of rebuilding+rebooting per candidate. */
static int mdp_kd_wrot_lines;
module_param(mdp_kd_wrot_lines, int, 0644);
MODULE_PARM_DESC(mdp_kd_wrot_lines, "vertical strip height (0=64; HW-iter sweep)");

/* DIAG ONLY: force COL_REV (rot270) >14-tile multi-chunk (normally SCP-gated -- it
 * corrupts) so the WROT register state can be probed under the failing condition.
 * Default 0 (safe). Set to 1 only with mdp_kd_regdump for the rot270 RE. */
static int mdp_kd_swap_colrev_mc;
module_param(mdp_kd_swap_colrev_mc, int, 0644);
MODULE_PARM_DESC(mdp_kd_swap_colrev_mc, "DIAG: force rot270 COL_REV >14 multi-chunk for WROT-state probing (default 0)");

/* PROVE-BEFORE-DELETE: when set, the SCP_IPI_MDP_FRAME config fallthrough at the
 * end of mdp_vpu_process (reached only when NO kernel-direct branch claimed the
 * frame) becomes a LOUD dev_WARN + graceful -EINVAL instead of calling the SCP.
 * With every 2D op kernel-direct, the FULL matrix gate + a wide real-resolution
 * sweep must fire ZERO WARNs -- proving the MDP config path needs no SCP firmware
 * and the SCP_IPI_MDP_FRAME config IPI can be compiled out (the SCP stays only for
 * video DECODE, a separate IPI on the vcodec nodes). Any WARN identifies a real
 * remaining SCP-config dependency (report it -- keep SCP for that op). This knob
 * does NOT touch the SCP init/deinit path (mdp_vpu_dev_init's if(!scp) foundation
 * is unaffected). Default 0 = SCP config fallback allowed (production). */
static bool mdp_scp_fallback_forbid;
module_param(mdp_scp_fallback_forbid, bool, 0644);
MODULE_PARM_DESC(mdp_scp_fallback_forbid, "PROVE: forbid the SCP MDP-config fallback (LOUD dev_WARN + -EINVAL) to prove kernel-direct coverage (default 0)");

/* mdp_kd_vcap>0: bound each VERTICAL scale tile so its INPUT span <= vcap lines, the RSZ
 * vertical-emit capacity (HW probe 2026-06-21: clean <=~640 lines, racy ~854-906, hard
 * stall >=~933). For the transpose V-scale of a wide image the input is the rotated height
 * (= original width, e.g. 1280) which exceeds that; capping forces nv>1 so each phase-0
 * strip streams within capacity -- the only avenue to generalize the transpose to
 * shallow-wide downscale. 0 = off (the original nv=min(2046,oh) behaviour). */
static int mdp_kd_vcap;
module_param(mdp_kd_vcap, int, 0644);
MODULE_PARM_DESC(mdp_kd_vcap, "cap vertical scale tile INPUT span to this many lines (0=off)");

/* HW-iter knob: horizontal tile width. ≤64px detiles bit-exact on mt8183; wider
 * tiles shift rows (a WROT line-buffer addressing limit). 0 = the 512px default. */
static int mdp_kd_tile_w;
module_param(mdp_kd_tile_w, int, 0644);
MODULE_PARM_DESC(mdp_kd_tile_w, "force WROT horizontal tile width (0=64; HW-iter sweep)");

/* DIAG knob: route I420/YV12 (planar 4:2:0) through the 64x32-tiled multi-plane
 * fmtconv path for HW RE of the planar chroma sub-tile geometry. Default 0 =
 * planar -> SCP (the planar chroma offset is not yet correct; NV21 interleaved is
 * production). =1 enables the WIP planar path for on-board diagnosis only. */
static int mdp_kd_fmtmp_planar;
module_param(mdp_kd_fmtmp_planar, int, 0644);
MODULE_PARM_DESC(mdp_kd_fmtmp_planar, "DIAG: route I420/YV12 through the 64x32 multi-plane path (default 0=SCP)");

/* mt8183 scale RSZ vertical coeff-table select (PRZ_CONTROL_1 bits 21..25).
 * Default 17 (CUB_ACC, matches the dpframework rsz8183 downscale). Set 0 to try
 * the SRC_ACC table -- HW probe for the flat-vertical artifact. */
static int mdp_kd_vertable = 17;
module_param(mdp_kd_vertable, int, 0644);
MODULE_PARM_DESC(mdp_kd_vertable, "mt8183 scale RSZ vertical_table_select (0..19; default 17)");

/* HW-iter knob: PRZ_CONTROL_1 bit27 = rsz_dcm_dis (disable Dynamic Clock Mgmt /
 * clock-gating on the RSZ). Lead for the flat-vertical artifact: the active RSZ
 * likely convolves an UNFILLED vertical line-buffer SRAM because DCM gates the
 * fill clock (the working detile RSZ is DISABLED/passthrough, never exercising
 * the fill path). Set 1 to keep the RSZ clock always on. Live-sweepable. */
static int mdp_kd_dcm_dis;
module_param(mdp_kd_dcm_dis, int, 0644);
MODULE_PARM_DESC(mdp_kd_dcm_dis, "mt8183 scale RSZ rsz_dcm_dis (PRZ_CONTROL_1 bit27; 0=DCM on, 1=disable clock-gating)");

/* HW-iter knob: WROT main_buf line_num, DECOUPLED from the subframe strip height.
 * 0 = use the strip height (sh). calWROTSetting derives this from the FIFO/Y-buffer
 * fit (line_num*width <= int_y_buf); a wide tile that overflows the buffer at
 * line_num=sh may be bit-exact at a smaller line_num. Lets us test whether the 64px
 * width cap is a buffer-fit (fixable) vs a hard FIFO depth. */
static int mdp_kd_linenum;
module_param(mdp_kd_linenum, int, 0644);
MODULE_PARM_DESC(mdp_kd_linenum, "force WROT main_buf line_num (0=strip height; HW-iter sweep)");

/* nh>=3 downscale gate: a STEEP (>=2:1, coeff_x<=0.5) wide downscale tiles cleanly
 * across interior boundaries (HW-validated: 1024x768->512x384, 1920x1080->640x360),
 * so nh>=3 is allowed kernel-direct for steep ratios. A SHALLOW (coeff>0.5) nh>=3
 * downscale is a HARDWARE WALL -- the RSZ cannot stream a shallow resample across an
 * interior tile boundary (the interior strip's last-output-column 6-tap filter needs
 * input past the strip's logical right edge; the emit is rigidly tied to the
 * contiguous PRZ_INPUT span, so feeding the filter extra margin corrupts the emit --
 * confirmed by the 2026-06-21 input-extension experiment, which broke even nh=2).
 * Full output for shallow nh>=3 requires a transpose-decomposition (route the wide
 * horizontal downscale through the no-stall vertical-streaming path) -- future work.
 * mdp_kd_nh3 forces ALL nh>=3 kernel-direct for HW probing (default 0 = coeff gate). */
static int mdp_kd_nh3;
module_param(mdp_kd_nh3, int, 0644);
MODULE_PARM_DESC(mdp_kd_nh3, "force ALL nh>=3 downscale kernel-direct, incl shallow (HW probe; default 0)");

/* mt8195 multi-tile UPSCALE (output >512) kernel-direct gate. DEFAULT ON (2026-06-29):
 * the upscale multi-tile march has been CMP-FITTED to the SCP oracle and validated
 * bit-exact on demo-mt8195-chromebook -- kernel-direct img_config == SCP img_config
 * (cmp=0) AND kernel-direct output == SCP output (md5) for 512->1024 (2 tiles) and
 * 640->1280 (3 tiles), bit-identical on BOTH SCP firmware builds (v23042 + v26100,
 * proving firmware-independence), with downscale unregressed (cmp=0) and the 650-cell
 * mtk-mdp format matrix green. The param is kept as a per-boot OFF switch (set 0 to
 * force >512 pure-upscale back to the SCP); mdp_kernel_direct=N remains the global
 * kernel-direct master switch. Single-tile up (ow<=512) was always kernel-direct and
 * is unaffected by this default. */
static bool mdp_kd_upscale = true;
module_param(mdp_kd_upscale, bool, 0644);
MODULE_PARM_DESC(mdp_kd_upscale,
	"mt8195: route multi-tile (>512) pure-upscale kernel-direct (default 1 = on, cmp-fit-validated; set 0 to force SCP)");

/* mt8195 MIXED-direction scale (one axis up, the other down) kernel-direct gate.
 * DEFAULT ON (2026-06-29): the per-dimension march has been validated bit-exact on
 * demo-mt8195-chromebook -- the kernel-direct output md5 == the SCP output for both
 * H-down/V-up (1280x360->640x720, 3-tile) and H-up/V-down (640x720->1280x360, 5-tile),
 * bit-identical on BOTH SCP firmware builds (v23042 995bcf81 + v26100 b94569d1, which
 * wedges the SCP path -- proving firmware-independence), with pure up+down unregressed
 * (scale-gate 8/8 bit-exact) and the 650-cell MDP format matrix green. The march picks
 * each axis's coeff independently (cx from up_h, cy from up_v), the RSZ control1 H/V
 * mode bits separately (HMASK 0x000a0050 / VMASK 0x01400100, SCP-decoded), the
 * horizontal multi-tile march + per-tile H phase shift (q15 up / q20 down) off the
 * WIDTH direction, and forces an even RSZ input-tile width (the 4:2:0 chroma sampler
 * wedges on an odd span). The param stays as a per-boot OFF switch (set 0 to force a
 * mixed scale back to the SCP); mdp_kernel_direct=N remains the global master switch. */
static bool mdp_kd_mixed = true;
module_param(mdp_kd_mixed, bool, 0644);
MODULE_PARM_DESC(mdp_kd_mixed,
	"mt8195: route MIXED-direction (one axis up, other down) scale kernel-direct (default 1 = on, validated bit-exact; set 0 to force SCP)");

/* mt8195 kernel-direct CROP (sub-region read via VIDIOC_S_SELECTION on the output
 * queue). The SCP does NOT pixel-exact-extract the crop rect: it routes it through
 * the RSZ pipe as a cubic resample (cx=cy=0x8000 q15-unity, control1=0x037b0003)
 * whose input window is the crop rect plus a tap back-margin and whose per-axis
 * phase carries the crop origin (H: rsz_left=cl-4 / phase 4; V: rsz_top=ct-3 /
 * phase 3 -- the dpframework 6-tap backward with the crop as the input frame:
 * inMaxEnd=crop_right, cropOffset=cl). So the output is RSZ-softened, NOT a plain
 * pixel-exact ROI -- a bit-exact KD crop must replicate the SCP's crop-via-RSZ
 * config (verified: an integer-offset crop still differs from the exact ROI by the
 * cubic ring). Scope (this first cut): a 1:1 crop (crop W/H == output W/H) of an
 * NV12 frame, single RSZ tile, with both margins inside the frame -- the canonical
 * "extract a sub-region at native resolution" case. crop+scale (crop dims != out
 * dims) and origin crops (cl==0 && ct==0, which the SCP bypasses the RSZ for) stay
 * on the SCP. Default-OFF until board-validated bit-exact; mdp_kernel_direct=N is
 * the global master switch. Validated bit-exact (KD output md5 == SCP output md5)
 * on demo-mt8195-chromebook for crop(64,64,128,128)/(32,96,..)/(66,66,..) ->128x128
 * and crop(128,64,256,256)/(256,128,..) ->256x256, snapshot cmp = per-run iova only;
 * firmware-independent (KD byte-identical on v23042 995bcf81 + v26100 b94569d1, the
 * build that wedges the SCP path); scale-gate 8/8 + format matrix 650/650 unregressed
 * -> default-ON. */
static bool mdp_kd_crop = true;
module_param(mdp_kd_crop, bool, 0644);
MODULE_PARM_DESC(mdp_kd_crop,
	"mt8195: route a 1:1 sub-region CROP (S_SELECTION) kernel-direct via the crop-as-RSZ-resample config (default 1 = on, validated bit-exact; set 0 to force SCP)");

/* CROP+SCALE (cw!=ow / ch!=oh): the crop rect is resampled to the output by the
 * RSZ -- same crop-as-RSZ-resample pipe as the 1:1 crop, but the scale coeff is
 * keyed off the crop dims and the crop origin folds into the RSZ start phase
 * (mdp_crop_scale_params + mdp_config_compute_8195_crop). Pure up OR pure down,
 * single RSZ tile, off-origin; MIXED + >512 route to the SCP. Validated bit-exact
 * (KD output md5 == SCP output md5) on demo-mt8195-chromebook (kver 00600) for
 * crop+downscale (512^2 crop(64,64,256,256)->128^2, 256^2 crop(64,64,128,128)->64^2,
 * crop(66,66,128,128)->96^2, crop(80,48,160,120)->80x60) + crop+upscale
 * (crop(64,64,64,64)->128^2, crop(64,64,128,128)->256^2); firmware-independent (KD
 * byte-identical on v23042 5840c41b + v26100 7b0ef935, the SCP-wedging build);
 * crop-gate 11/11 + scale 8/8 + matrix 650/650 unregressed -> default-ON. */
static bool mdp_kd_cropscale = true;
module_param(mdp_kd_cropscale, bool, 0644);
MODULE_PARM_DESC(mdp_kd_cropscale,
	"mt8195: route an off-origin sub-region CROP+SCALE (crop dims != output dims) kernel-direct via the crop-as-RSZ-resample config (default 1 = on, validated bit-exact; set 0 to force SCP)");

/* Transpose-decomposition (shallow nh>=3) component 1: scratch I/O override. A
 * config-compute (yrot / yrot_swap / scale) normally reads its input from
 * param->inputs[0].buffer and writes param->outputs[0].buffer. To chain
 * rotate->scale->rotate through scratch, a per-pass override redirects the input
 * and/or output to a scratch iova (and overrides dims/stride). Resolved AFTER the
 * param locals, so a NULL pointer / zero field leaves the function unchanged.
 * Implemented as a file-static (not a new param on every signature): the mt8183
 * m2m serializes device_run, so the config-compute is never re-entered concurrently;
 * the override is set immediately before a config-compute and cleared right after,
 * within one synchronous call -- never held across an await. */
struct mdp_kd_io {
	u32 iw, ih, istride, iy, ic;	/* input override  (iw==0 -> use param) */
	u32 ow, oh, ostride, oy, oc;	/* output override (ow==0 -> use param) */
};
static const struct mdp_kd_io *mdp_kd_io_ovr;

/* Component-1 self-test: 2-pass Y rotate via scratch (rot90 input->scratch_a, then
 * rot270 scratch_a->output == identity) -- exercises the override on yrot_swap AND
 * proves engine RDMA-read + WROT-write of scratch. */
static int mdp_kd_yrt;
module_param(mdp_kd_yrt, int, 0644);
MODULE_PARM_DESC(mdp_kd_yrt, "self-test: 2-pass Y rot90/rot270 round-trip via scratch (1=on)");

/* Component 2: 3-pass Y transpose-decomposition for a (shallow nh>=3) downscale.
 * rot90 input(WxH)->scratch_a(HxW); NV12 V-scale scratch_a->scratch_b(H'xW') (width
 * H->H' nh<=2, height W->W' streamed -> no horizontal-interior-boundary stall); rot270
 * scratch_b->output(W'xH'). Net Y = the WxH->W'xH' downscale, computed without the
 * stalling wide-shallow horizontal RSZ. The scale pass itself is nh_scale (<=2) strips,
 * so the job is nh_scale+2 chunks. Y only (chroma is CPU-side in component 3); until
 * then the output chroma is garbage. mdp_kd_tpose forces this path for any downscale. */
static int mdp_kd_tpose;
module_param(mdp_kd_tpose, int, 0644);
MODULE_PARM_DESC(mdp_kd_tpose, "transpose-decomposition 3-pass Y downscale via scratch (1=on; Y only)");

/* HW-iter knobs for the mt8183 CSC (NV12->RGB24) RDMA read, which hangs the GCE
 * on a wrong value. Defaults are DETILE-FAITHFUL (the working mt8183 detile reads
 * with src_ctrl=0x860c block / control=0x10 simple_mode): linear = the detile
 * src_ctrl with the 0x8000 block bit cleared (0x060c), same control=0x10. Live-
 * sweepable: echo VAL > /sys/module/mtk_mdp3/parameters/mdp_csc_{srcctrl,rdmacon}. */
static int mdp_csc_srcctrl = 0x060c;
module_param(mdp_csc_srcctrl, int, 0644);
MODULE_PARM_DESC(mdp_csc_srcctrl, "mt8183 CSC RDMA SRC_CON (default 0x060c = detile linear)");
static int mdp_csc_rdmacon = 0x10;
module_param(mdp_csc_rdmacon, int, 0644);
MODULE_PARM_DESC(mdp_csc_rdmacon, "mt8183 CSC MDP_RDMA_CON (default 0x10 = detile simple_mode)");

/* SVPP2 detile pipe, per component: {type (enum mdp_comp_id), id (alias)}. */
static const struct { u8 type, id; } mdp_dt_comp[10] = {
	{ 10, 2 }, { 15, 2 }, { 17, 4 }, { 27, 2 }, { 31, 2 },
	{ 35, 2 }, { 39, 2 }, { 43, 2 }, { 49, 2 }, { 55, 2 },
};

/* SVPP2 mmsys mux routing for the detile, ctrls[0] (geometry-independent). */
static const struct { u32 reg, value, subsys_id; } mdp_dt_mux[7] = {
	{ 0xf50, 1, 9 }, { 0xf4c, 1, 9 }, { 0xf38, 0, 9 }, { 0xf44, 0, 9 },
	{ 0xf40, 1, 9 }, { 0xf90, 0, 9 }, { 0xf3c, 0, 9 },
};

/*
 * Exactly plain NV12 (or its multi-planar twin NM12): group YUV, hw_format 12,
 * NOT block-mode, SWAP bit clear. NV21/NM21 share hw_format 12 but set SWAP, so a
 * gate that checks only hw_format routes ...->NV21 through a compute that writes
 * NV12 U/V order -> chroma reversed (format-sweep confirmed KD-WRONG). NM12 (same
 * chroma order, separate planes) is fine -- the compute uses iova[0]/iova[1].
 */
static bool mdp_color_is_plain_nv12(u32 c)
{
	return !(c & 0x00800000) && MDP_COLOR_GET_GROUP(c) == 1 &&
	       MDP_COLOR_GET_HW_FORMAT(c) == 12 && !MDP_COLOR_IS_SWAPPED(c);
}

/*
 * MM21/MT21 block-mode 4:2:0 SOURCE (the mtk-vcodec decoder CAPTURE format): group
 * YUV, hw_format 12 (NV12 family), block-mode bit set. MM21 (MDP_COLOR_420_BLK) is
 * plain-tiled NV12; MT21C (MDP_COLOR_420_BLK_UFO) additionally carries the UFO
 * compression bit -- the geometry-only detile read (src_ctrl 0x860c) is FAITHFUL to
 * plain block only, so this predicate matches the SAME set the detile gate accepts
 * (block-mode bit alone; the WROT-write side then does the X-conversion). It is the
 * RDMA read-side twin of mdp_color_is_plain_nv12: the pipe carries 4:2:0 YUV either
 * way, so a block source reaches every NV12->X WROT output.
 */
static bool mdp_color_is_mm21src(u32 c)
{
	return (c & 0x00800000) && MDP_COLOR_GET_GROUP(c) == 1 &&
	       MDP_COLOR_GET_HW_FORMAT(c) == 12;
}

/*
 * mt8183 NV21/NM21 fmtconv SOURCE (default off until HW-validated). NV21 is read
 * with the IDENTICAL 2-plane 4:2:0 RDMA geometry as NV12 (only the chroma byte
 * order differs); the swapped chroma is compensated algebraically by selecting
 * the chroma-swapped-twin fmtconv row, whose WROT control carries the 0x100 swap
 * bit (NV21->D == NV12->swap_twin(D)). No new RDMA read mode, no GCE-stall risk.
 */
static bool mdp_kd_src_nv21 = true;
module_param(mdp_kd_src_nv21, bool, 0644);
MODULE_PARM_DESC(mdp_kd_src_nv21,
	"mt8183 NV21/NM21 fmtconv source via chroma-swap-twin (default on; HW bit-exact validated)");

/* Exactly plain NV21 (or its multi-planar twin NM21): like plain NV12 but the
 * MDP_COLOR SWAP bit is SET (reversed chroma byte order). */
static bool mdp_color_is_plain_nv21(u32 c)
{
	return !(c & 0x00800000) && MDP_COLOR_GET_GROUP(c) == 1 &&
	       MDP_COLOR_GET_HW_FORMAT(c) == 12 && MDP_COLOR_IS_SWAPPED(c);
}

/*
 * mt8183 NV16/NV61 (2-plane 4:2:2) fmtconv SOURCE (default off until HW-validated).
 * Unlike NV21 (increment 1, which reused NV12's read verbatim), a 4:2:2 source is a
 * genuinely NEW RDMA read mode: the chroma plane is FULL-height (V_SUBSAMPLE=0), so
 * the read geometry differs from NV12's half-height 4:2:0 chroma. Per the dpframework
 * mt8183 DpEngine_RDMA (mapColorFormat NV16/NV61 case): bitsPerPixelY=8,
 * bitsPerPixelUV=16, m_horizontalShiftUV=1, m_verticalShiftUV=0 -- identical to NV12
 * EXCEPT the vertical chroma shift. So vs the NV12 read the only deltas are:
 *  (a) SRC_CON sourceFormat = HW_FORMAT(NV16)=0xd (vs NV12's 0xc) -> src_ctrl 0x060d;
 *      filterMode stays 3 (dpframework: V-filter unless 420->422 upsample, N/A here);
 *  (b) chroma plane size iova_end[1] = ic + istride*h (FULL height, not h/2);
 *  (c) per-tile chroma read row offset = y0*istride (not (y0/2)*istride).
 * The source U/V byte order is handled by the SAME chroma-swap-twin mechanism as NV21
 * (NV61 source read as NV16 -> swapped chroma in the pipe -> swap_twin(out) row's WROT
 * 0x100 swap bit compensates), so the only genuinely new thing is the 4:2:2 read.
 */
static bool mdp_kd_src_nv16 = true;
module_param(mdp_kd_src_nv16, bool, 0644);
MODULE_PARM_DESC(mdp_kd_src_nv16,
	"mt8183 NV16/NV61 (2-plane 4:2:2) fmtconv source via the 4:2:2 RDMA read (default on; HW bit-exact validated)");

/* Plain NV16 or NV61 (and their multi-planar NM twins): group YUV, HW_FORMAT 13
 * (V_SUBSAMPLE=0, 4:2:2), 2-plane, linear (not block). NV16 has SWAP clear (U,V),
 * NV61 has it set (V,U). */
static bool mdp_color_is_plain_nv16src(u32 c)
{
	return !(c & 0x00800000) && MDP_COLOR_GET_GROUP(c) == 1 &&
	       MDP_COLOR_GET_HW_FORMAT(c) == 13 &&
	       MDP_COLOR_GET_PLANE_COUNT(c) == 2;
}

/*
 * mt8183 NV24/NV42 (2-plane 4:4:4 semi-planar) fmtconv SOURCE (default OFF until
 * HW-validated). The 4:4:4 analogue of NV16: the chroma plane is FULL-width AND
 * FULL-height (16-bpp interleaved, one U/V pair per luma pixel). Per the dpframework
 * mt8183 DpEngine_RDMA (mapColorFormat NV24/NV42 case): bitsPerPixelY=8,
 * bitsPerPixelUV=16, m_horizontalShiftUV=0, m_verticalShiftUV=0 -- identical to NV16
 * EXCEPT the horizontal chroma shift is also 0 (full-width). HW-NATIVE, no CPU
 * pre-pass: the RDMA reads the full-res chroma into the pipe (which carries 4:4:4,
 * exactly like the RGB source AFTER its RDMA CSC -- increment 5), and the WROT then
 * decimates the pipe chroma to each dst's sampling (uv_xsel/uv_ysel are keyed on the
 * TARGET format alone -- DpEngine_WROT, hSubsample/vSubsample of m_targetFormat -- so
 * the SAME transcribed WROT control words the NV12/NV16/RGB sources already use apply
 * unchanged: 4:4:4->4:2:0 for NV12/NV21, 4:4:4->4:2:2 for NV16/NV61/packed, Y-only for
 * GREY, RGB via the WROT YUV->RGB CSC). So vs the NV16 read the ONLY deltas are:
 *  (a) SRC_CON sourceFormat = HW_FORMAT(NV24)=0xe (vs NV16's 0xd) -> src_ctrl 0x060e;
 *  (b) chroma plane pitch = istride*2 (full-width 16-bpp), iova_end[1] = ic + istride*2*h;
 *  (c) per-tile chroma read offset = y0*(istride*2) + x0*2 (full-height row, 2 B/px).
 * The source U/V byte order is handled by the SAME chroma-swap-twin mechanism as
 * NV21/NV61 (NV42 read as NV24 -> swapped chroma in the pipe -> swap_twin(out) row's
 * WROT 0x100 bit compensates). NV24/NV42 are OUTPUT-queue-only formats (not CAPTURE),
 * so this is a source-side increment; the 4:4:4 DST wall (WROT cannot write 4:4:4)
 * is untouched.
 */
static bool mdp_kd_src_nv24 = true;
module_param(mdp_kd_src_nv24, bool, 0644);
MODULE_PARM_DESC(mdp_kd_src_nv24,
	"mt8183 NV24/NV42 (2-plane 4:4:4) fmtconv source via the full-res-chroma RDMA read + WROT per-target decimate (default on; i500 bit-exact NV24->NV12/NV16 maxabs=0, NV24->RGB24 58.2 dB, NV42 swap-twin; WROT decimate = drop-even both axes)");

/* Plain NV24 or NV42 (2-plane 4:4:4 semi-planar): group YUV, HW_FORMAT 14
 * (H_SUBSAMPLE=0 AND V_SUBSAMPLE=0, full-res chroma), 2-plane, linear (not block).
 * NV24 has SWAP clear (U,V), NV42 set (V,U). */
static bool mdp_color_is_plain_nv24src(u32 c)
{
	return !(c & 0x00800000) && MDP_COLOR_GET_GROUP(c) == 1 &&
	       MDP_COLOR_GET_HW_FORMAT(c) == 14 &&
	       MDP_COLOR_GET_PLANE_COUNT(c) == 2;
}

/*
 * mt8183 packed 4:2:2 (YUYV/YVYU/UYVY/VYUY) fmtconv SOURCE (default off until
 * HW-validated). This is the "unproven standalone single packed-plane RDMA read"
 * the matrix-expansion roadmap flags as a GCE-stall risk -- a SINGLE interleaved
 * plane (Y U Y V), no second chroma plane (iova[1] unused). Per the dpframework
 * mt8183 DpEngine_RDMA (mapColorFormat UYVY/VYUY/YUYV/YVYU case): one plane,
 * bitsPerPixelY=16, bitsPerPixelUV=0, m_horizontalShiftUV=0, m_verticalShiftUV=0,
 * useSimpleMode. So vs the NV12 2-plane read the deltas are:
 *  (a) SRC_CON sourceFormat = the source HW_FORMAT (YUYV/YVYU=5 -> src_ctrl 0x605,
 *      UYVY/VYUY=4 -> 0x604); filterMode stays 3 (V_SUBSAMPLE=0, no 420->422 interp);
 *  (b) ONE plane: iova[1]/iova_end[1]/sf_bkgd all 0, iova_end[0]=iy+istride*h with
 *      istride = width*2 (16 bpp);
 *  (c) per-tile read offset = y0*istride + x0*2 (2 bytes/pixel), no chroma offset.
 * The U/V byte order (YUYV vs YVYU, UYVY vs VYUY) is handled by the SAME chroma-
 * swap-twin mechanism as NV21/NV61: a swapped source (YVYU/VYUY) is read as its
 * unswapped twin (YUYV/UYVY) so the chroma lands byte-swapped in the pipe, and the
 * swap_twin(out) row's WROT 0x100 swap bit compensates. The Y-first vs C-first
 * ordering (HW_FORMAT 5 vs 4) is preserved by src_ctrl. The OUTPUT (WROT) side is
 * unchanged -- packed/4:2:2 WROT writes were already validated by NV12->packed/NV16.
 */
static bool mdp_kd_src_yuyv = true;
module_param(mdp_kd_src_yuyv, bool, 0644);
MODULE_PARM_DESC(mdp_kd_src_yuyv,
	"mt8183 packed 4:2:2 (YUYV/YVYU/UYVY/VYUY) fmtconv source via the single packed-plane RDMA read (default on; HW bit-exact validated)");

/* Plain packed 4:2:2 (YUYV/YVYU/UYVY/VYUY): group YUV, single plane, HW_FORMAT 5
 * (Y first: YUYV/YVYU) or 4 (C first: UYVY/VYUY), 16 bpp, linear (not block, not
 * 10-bit). SWAP clear = U before V (YUYV/UYVY), set = V before U (YVYU/VYUY). */
static bool mdp_color_is_packed422src(u32 c)
{
	u32 hf = MDP_COLOR_GET_HW_FORMAT(c);

	return !(c & 0x00800000) && MDP_COLOR_GET_GROUP(c) == 1 &&
	       MDP_COLOR_GET_PLANE_COUNT(c) == 1 && (hf == 4 || hf == 5) &&
	       MDP_COLOR_BITS_PER_PIXEL(c) == 16;
}

/*
 * mt8183 I420/YV12 (3-plane planar 4:2:0) fmtconv SOURCE (default off until
 * HW-validated). The matrix-expansion roadmap's HIGH-risk increment: a 3-plane
 * RDMA read -- THREE separate planes (Y w*h, U w/2*h/2, V w/2*h/2), iova[0]=Y,
 * iova[1]=U, iova[2]=V. Per the dpframework mt8183 DpEngine_RDMA (mapColorFormat
 * I420/YV12 case): sourceFormat = HW_FORMAT(I420)=8 -> src_ctrl 0x608 (filterMode 3
 * for 4:2:0->4:2:0, same as NV12); bitsPerPixelY=8, bitsPerPixelUV=8 (half-WIDTH
 * 8-bpp chroma PLANES, vs NV12's 16-bpp interleaved). So vs the NV12 2-plane read
 * the deltas are:
 *  (a) src_ctrl 0x608 (sourceFormat 8 = YUV420 3-plane; no plane-count bit -- the
 *      3-plane-ness is conveyed entirely by sourceFormat + the plane base addrs);
 *  (b) THREE plane bases iova[0..2] + iova_end[2]; chroma plane pitch = istride/2
 *      (RDMA SF_BKGD = inUVPitch = w/2), so per-tile chroma offset =
 *      (y0/2)*(istride/2) + (x0/2) (vs NV12's interleaved (y0/2)*istride + x0).
 * YV12 is I420 with the U and V PLANES swapped (buffer Y,V,U -> iova[1]=V, iova[2]=U);
 * feeding RDMA chroma-1<-U, chroma-2<-V (an exact plane-pointer swap in the compute)
 * normalizes the pipe to true YUV, so the OUTPUT row is selected by the plain output
 * color -- NO swap-twin needed (unlike interleaved-chroma sources, the 3-plane pointer
 * swap is bit-exact). This increment admits only the 4:2:0 YUV (NV12/NV21) + GREY
 * outputs (no vertical resample). 4:2:2 dst (needs 4:2:0->4:2:2 upsample), RGB (CSC)
 * and I420/YV12 dst (the 3-plane WROT WRITE wall) are deferred/walled.
 */
static bool mdp_kd_src_i420 = true;
module_param(mdp_kd_src_i420, bool, 0644);
MODULE_PARM_DESC(mdp_kd_src_i420,
	"mt8183 I420/YV12 (3-plane 4:2:0) fmtconv source via the 3-plane RDMA read (default on; HW bit-exact validated)");

/* Plain I420 or YV12 (3-plane planar 4:2:0): group YUV, 3 planes, HW_FORMAT 8,
 * 8 bpp, linear (not 10-bit loose). I420 has SWAP clear (U,V plane order), YV12
 * set (V,U). */
static bool mdp_color_is_plain_i420src(u32 c)
{
	return !(c & 0x00800000) && MDP_COLOR_GET_GROUP(c) == 1 &&
	       MDP_COLOR_GET_HW_FORMAT(c) == 8 &&
	       MDP_COLOR_GET_PLANE_COUNT(c) == 3;
}

/* SCP-tail class #2 -- YM16/YM61 (V4L2 YUV422M/YVU422M) SOURCE: 3-plane planar
 * 4:2:2 (MDP_COLOR_I422/YV16 = PLANE 3, HW_FORMAT 9, V_SUBSAMPLE 0 = FULL-height
 * chroma). The 3-plane 4:2:2 twin of mdp_color_is_plain_i420src (HW_FORMAT 8,
 * 4:2:0, half-height chroma). SWAP distinguishes YM16 (U-first) from YM61
 * (V-first). Read src_ctrl = 0x600 | (SWAP ? 0x4000 : 0) | 0x09 -- oracle-confirmed
 * on the v23042 SCP (YM16>* c0=0x609, YM61>* c0=0x4609). */
static bool mdp_kd_ym16src = true;
module_param(mdp_kd_ym16src, bool, 0644);
MODULE_PARM_DESC(mdp_kd_ym16src,
	"mt8195 YM16/YM61 (3-plane planar 4:2:2) fmtconv SOURCE via the 3-plane RDMA read -> non-3-plane dsts (default on; SCP-tail class #2)");

static bool mdp_color_is_ym16src(u32 c)
{
	return !(c & 0x00800000) && MDP_COLOR_GET_GROUP(c) == 1 &&
	       MDP_COLOR_GET_HW_FORMAT(c) == 9 &&
	       MDP_COLOR_GET_PLANE_COUNT(c) == 3;
}

/*
 * mt8183 RGB (RGB24/BGR24/RGB565/RGB565X/ARGB32/ABGR32) fmtconv SOURCE -> YUV dst
 * (default off until HW-validated). The matrix-expansion roadmap's FINAL increment: a
 * SINGLE packed RGB plane read PLUS the RGB->YUV CSC -- the INVERSE of the existing
 * NV12->RGB CSC (mdp_config_compute_8183_csc, WROT mat_ctrl=0x61 = int_matrix_sel 6 =
 * BT601->RGB). For an RGB SOURCE the CSC is done at the RDMA, NOT the WROT: per the
 * dpframework mt8183 DpEngine_RDMA, mapColorFormat sets colorTran=1 for every RGB format
 * and matrixSel = {2 BT601, 3 BT709, 0 JPEG/full-range}; the value is written to
 * MDP_RDMA_TRANSFORM_0 = (matrixSel<<24)|(colorTran<<16) (DpEngine_RDMA.cpp:793). So
 * RGB->BT601 -> transform 0x02010000. Deltas vs the YUV-source reads:
 *  (a) SRC_CON sourceFormat = the RGB HW_FORMAT (RGB565/BGR565=0, RGB888/BGR888=1,
 *      ARGB/ABGR/RGBA/BGRA8888=3) + sourceSwap<<14 (R/B byte order: BGR/ABGR set) +
 *      filterMode 3 (RGB has V_SUBSAMPLE=0) -> src_ctrl 0x600|(swap?0x4000)|hwfmt;
 *  (b) ONE plane: iova[1]/iova_end[1]/sf_bkgd = 0; bppsrc = bits-per-pixel/8 (2/3/4);
 *      iova_end[0] = iy + istride*h (istride = w*bppsrc); per-tile offset[0] =
 *      y0*istride + x0*bppsrc, no chroma plane;
 *  (c) rdma.transform = mdp_rgb_transform (default 0x02010000; sweepable to pin the
 *      colorspace/range vs the CPU CSC reference -- mt8183 has no SCP oracle).
 * The WROT writes the PLAIN YUV output (mat_ctrl=0, table control word) -- NO swap-twin
 * (RGB has no chroma byte-order relation to a YUV dst; R/B order is the RDMA sourceSwap).
 * The RGB-4:4:4 -> 4:2:0/4:2:2 chroma DOWNSAMPLE is the WROT's (target-format driven),
 * same WROT config as the YUV-source writes. RGB dst (RGB->RGB, not a CSC) and I420/YV12
 * dst (the 3-plane WROT write wall) are excluded. First gate = RGB24->GREY (Y-only, no
 * chroma) isolates the RGB read + the Y-CSC; then RGB24->NV12 (full RGB->YUV 4:2:0).
 */
static bool mdp_kd_src_rgb = true;
module_param(mdp_kd_src_rgb, bool, 0644);
MODULE_PARM_DESC(mdp_kd_src_rgb,
	"mt8183 RGB (RGB24/BGR24/RGB565/ARGB32/ABGR32) fmtconv source -> YUV via the RDMA RGB->YUV CSC (default on; HW bit-exact validated; RGB565X gated out -- wrong BE-565 read)");

/*
 * mt8183 GREY (Y8, luma-only) fmtconv SOURCE -> the full X dst set, via a CPU
 * pre-expand to a neutral-chroma NV12 scratch (default OFF until HW-validated).
 * Unlike every other source increment, a GREY read cannot be done by a new RDMA
 * read mode: the dpframework mt8183 DpEngine_RDMA GREY case reads Y ONLY
 * (bitsPerPixelUV=0) and leaves the pipe chroma UNDEFINED -- there is no HW mode
 * that synthesizes neutral chroma. So the CPU copies the Y plane and fills a
 * neutral (0x80) interleaved UV plane into scratch_b, and the source descriptor is
 * reconfigured to plain NV12 @ scratch_b (see mdp_vpu_process). Every existing
 * NV12->X compute -- fmtconv (GREY/NV21/NV16/NV61/packed 4:2:2), CSC (RGB), the
 * I420/YV12-dst hybrid and the YM12/YM21 multi-planar hybrid -- then yields GREY->X
 * unchanged. The "correct" output is Y + neutral chroma: GREY->YUV = Y + 0x80 UV,
 * GREY->RGB = the standard NV12->RGB CSC of that neutral frame (a gray image,
 * R=G=B~=Y), GREY->GREY = a straight Y copy. GREY->NV12/NM12 (the 4:2:0 identity,
 * which is blit's job and not kernel-direct on mt8183) is routed explicitly through
 * the WROT NV12 write (mdp_fmtconv_nv12), like the I420-dst hybrid. The CPU pass is
 * O(w*h) and runs before the GCE submit; scratch_b is used so the mp arm keeps
 * scratch_a for its UV redirect.
 */
static bool mdp_kd_src_grey = true;
module_param(mdp_kd_src_grey, bool, 0644);
MODULE_PARM_DESC(mdp_kd_src_grey,
	"mt8183 GREY (Y8) fmtconv source -> full X dst set via CPU pre-expand to a neutral-chroma NV12 scratch (default on; i500 bit-exact GREY->GREY/NV12/YUYV/YU12/YM12, GREY->RGB24 gray/56.3 dB)");

/* Plain GREY (Y8, luma-only): a single 8-bit Y plane, no chroma. */
static bool mdp_color_is_grey_src(u32 c)
{
	return c == MDP_COLOR_GREY;
}

/*
 * mt8183 MM21/MT21 (block-mode 4:2:0, the mtk-vcodec decoder CAPTURE format)
 * fmtconv SOURCE -> the full X dst set (default off until HW-validated). The MM21
 * detile (mdp_config_compute_8183, MM21->NV12) and the NV12->X fmtconv
 * (mdp_config_compute_8183_fmtconv) are two ALREADY-VALIDATED single-pass computes
 * that differ ONLY on the RDMA read side (block-mode 0x860c/tiled offsets vs linear
 * 0x060c) -- the RSZ-bypass, WROT-write, 64x32 tile grid and mmsys mux are byte-
 * identical. So MM21->X = (the detile block-mode RDMA read) + (the NV12->X WROT
 * write), combined in ONE pass. This param routes an MM21/MT21 source with a non-
 * NV12 fmtconv dst (MM21->NV12 stays the pure detile arm) through the fmtconv
 * compute's srcmm21 read branch, giving MM21->{NV21/NM21, YUYV/YVYU/UYVY/VYUY,
 * NV16/NV61, GREY, RGB565/RGB565X/BGR24/RGB24/ARGB32/ABGR32}. I420/YV12 dst
 * (3-plane WROT wall) stays deferred. MM21 is 4:2:0 exactly like NV12, so the CPU
 * reference is fmtconv_X(detile(MM21)).
 */
static bool mdp_kd_mm21conv = true;
module_param(mdp_kd_mm21conv, bool, 0644);
MODULE_PARM_DESC(mdp_kd_mm21conv,
	"mt8183 MM21/MT21 block-mode source -> the full fmtconv X dst set (detile read + X-fmtconv WROT write, one pass; default on -- composes the HW bit-exact detile read + fmtconv WROT write)");

/* mt8183 RGB-source CSC matrix: MDP_RDMA_TRANSFORM_0 = (matrixSel<<24)|(colorTran<<16).
 * Default 0x02010000 = matrixSel 2 (RGB->BT601, limited range) + colorTran 1. Sweepable
 * to pin the colorspace/range vs the CPU reference (0x00010000 = RGB->JPEG full-range,
 * 0x03010000 = RGB->BT709) -- mt8183 has no SCP oracle, so HW PSNR vs ffmpeg is the gate. */
static int mdp_rgb_transform = 0x02010000;
module_param(mdp_rgb_transform, int, 0644);
MODULE_PARM_DESC(mdp_rgb_transform,
	"mt8183 RGB-source RDMA CSC TRANSFORM_0 (default 0x02010000 = RGB->BT601 limited range)");

/*
 * mt8183 YUV-source -> RGB-dst CSC un-gate (default off until HW-validated). The
 * landed YUV fmtconv sources (NV12/NV16/packed-4:2:2/I420) GATE OUT RGB
 * destinations -- each source arm rejects f->rgb, deferring RGB to "the CSC path".
 * But the YUV->RGB CSC already works: the fmtconv compute (mdp_config_compute_8183_
 * fmtconv) applies the WROT mat_ctrl=0x61 (YUV->RGB BT.601) for any f->rgb row,
 * identical to the NV12->RGB path, regardless of the source read mode. So a YUV
 * source -> RGB dst is exactly: (that source's already-validated RDMA read) ->
 * (the same WROT CSC the NV12->RGB path uses). This param un-gates RGB dsts for the
 * NON-SWAPPED YUV sources (NV12/NV16/YUYV/UYVY native + I420/YV12 3-plane, all of
 * which feed the CSC true-YUV chroma). SWAPPED-chroma sources (NV21/NV61/YVYU/VYUY)
 * stay gated: their swap is compensated by the swap-twin's WROT 0x100 bit, and RGB
 * has no chroma-swap twin, so feeding the CSC directly would give reversed U/V.
 */
static bool mdp_kd_yuv2rgb = true;
module_param(mdp_kd_yuv2rgb, bool, 0644);
MODULE_PARM_DESC(mdp_kd_yuv2rgb,
	"mt8183 un-gate RGB dsts for non-swapped YUV fmtconv sources via WROT YUV->RGB CSC (default on; HW bit-exact validated)");

/*
 * DIG-DEEPER batch 2 -- three independent default-off increments on the fmtconv
 * matrix (each a distinct src->dst class; validate + activate independently):
 *
 * (1) mdp_kd_rgb2rgb -- RGB SOURCE -> RGB DST (a pure repack, NOT a CSC). The RGB
 *     read (mdp_kd_src_rgb) already lands the source pixels in the pipe; for an RGB
 *     dst there is NO colorspace conversion, so the RDMA CSC (rdma.transform) AND
 *     the WROT CSC (mat_ctrl=0x61) are BOTH disabled -- the WROT just repacks the
 *     RGB pipe to the dst RGB layout (RGB24->RGB565 quantize, ARGB->RGB24 drop
 *     alpha, ...). The dst RGB control word comes from mdp_fmtconv_tbl.
 * (2) mdp_kd_swap2rgb -- SWAPPED YUV source (NV21/NM21, NV61/NM61, YVYU, VYUY) ->
 *     RGB dst. The landed swapped-source path relies on the swap-twin's WROT 0x100
 *     bit, which cannot fix an RGB output; instead UN-SWAP on the READ via the RDMA
 *     sourceSwap bit (SRC_CON bit14=0x4000, DpEngine_RDMA mapColorFormat: sourceSwap
 *     = DP_COLOR_GET_SWAP_ENABLE) -> true YUV in the pipe -> the working WROT
 *     YUV->RGB CSC (mat_ctrl=0x61) with the PLAIN RGB dst row.
 * (3) mdp_kd_up422 -- I420/YV12 (4:2:0 3-plane source) -> 4:2:2 dst (NV16/NV61/
 *     YUYV/YVYU/UYVY/VYUY). The 3-plane read normalizes to internal 4:2:0 (identical
 *     to the NV12 read), so I420->4:2:2 is exactly NV12->4:2:2 downstream: the
 *     WROT's 4:2:0->4:2:2 vertical upsample (uv_ysel=2, table control 0x8000500d)
 *     is already exercised by NV12->NV16. Just un-gate the 4:2:2 dst for the I420 src.
 */
static bool mdp_kd_rgb2rgb = true;
module_param(mdp_kd_rgb2rgb, bool, 0644);
MODULE_PARM_DESC(mdp_kd_rgb2rgb,
	"mt8183 un-gate RGB dsts for an RGB fmtconv source -- pure repack, no CSC (default on; RGB data HW bit-exact; note: 8888 output alpha is not preserved -- WROT writes opaque 0xff, RGB pipe drops source alpha)");

static bool mdp_kd_swap2rgb = true;
module_param(mdp_kd_swap2rgb, bool, 0644);
MODULE_PARM_DESC(mdp_kd_swap2rgb,
	"mt8183 un-gate RGB dsts for SWAPPED YUV sources (NV21/NV61/YVYU/VYUY) via RDMA sourceSwap un-swap read + WROT YUV->RGB CSC (default on; HW bit-exact validated)");

static bool mdp_kd_up422 = true;
module_param(mdp_kd_up422, bool, 0644);
MODULE_PARM_DESC(mdp_kd_up422,
	"mt8183 un-gate 4:2:2 dsts for the I420/YV12 4:2:0 source via the WROT 4:2:0->4:2:2 upsample (default on; HW bit-exact validated)");

/* mt8195 >1920 (DUAL_BITBLT) GENERIC dual-fmtconv catch-all arm: the dual twin of
 * the <=1920 fmtconv compute, closing the wide fmtconv SCP tail (NV21/NM21-src,
 * NV12-src -> packed/GREY/RGB, packed/NV16/i420/ym16-src -> packed/GREY/RGB, and
 * 4:2:0->4:2:2 upsample dsts). Default on. Off => those wide pairs fall to SCP
 * (the pre-arm behaviour) -- the A/B lever for the wide-tail-collapse measurement. */
static bool mdp_kd_genfmt = true;
module_param(mdp_kd_genfmt, bool, 0644);
MODULE_PARM_DESC(mdp_kd_genfmt,
	"mt8195 >1920 generic dual-fmtconv catch-all arm (default on; off => the uncovered wide fmtconv pairs fall to SCP)");

/* mt8195 >1920 (DUAL_BITBLT) MM21/MT21-SOURCE dual arm: the block-detile twin of the
 * generic dual-fmtconv/3-plane arms. The genfmt/gen3p arms use a LINEAR dual read
 * (cannot detile MM21), so an MM21/MT21 tiled source at >1920 falls to the SCP; this
 * knob drives the block-detile dual read (mdp_fill_dual_pipe_8195, the ta*32 block
 * addressing) feeding the SAME parameterized WROT (mdp_dual_wrot_for_dst) -- one-pass
 * detile-read + fmtconv/3-plane-write, the >1920 twin of the <=1920 mm21conv path.
 * Default on. Off => MM21/MT21-src >1920 pairs fall to SCP -- the A/B lever for the
 * residual (54 -> 27 RGBR-only) collapse measurement. */
static bool mdp_kd_genmm21 = true;
module_param(mdp_kd_genmm21, bool, 0644);
MODULE_PARM_DESC(mdp_kd_genmm21,
	"mt8195 >1920 MM21/MT21-source block-detile dual arm (default on; off => MM21/MT21-src >1920 pairs fall to SCP)");

/* mt8195 >1920 block-padded-height MM21 DUAL CROP (out h < coded inh): activate the
 * detile dual's c5 RSZ as a vertical downscale inh->h with num_subfrms=2 horizontal
 * sub-tiling (mdp_fill_dual_pipe_8195_crop), so a real 1440p/2160p MM21 decode goes
 * kernel-direct instead of falling to SCP. DEFAULT ON -- both pixel pipes are now
 * BIT-EXACT: pp0 was corrected from the true SCP-pp0 oracle (cfg8195_snapshot_pp0,
 * captured on the SCP path) + the c9 WROT per-plane bytes-per-pixel offset scaling
 * (fix 1d263507), giving pp0+pp1 cmp-fit 0-byte-diff at 2560x1440 AND 3840x2160 and
 * MM21->{NV12,YM12,YU12,YUYV,RGB3,GREY} md5(KD)==md5(SCP) 18/18. This SUPERSEDES the
 * 941f620 SCP route: with the knob on, mdp_mm21_dual_kd_exact() sends block-padded MM21
 * dual to mdp_fill_dual_pipe_8195_crop (KD), not the SCP. Set to 0 at runtime to fall
 * back to SCP for a block-padded source. See audit/mm21-dual-crop-oracle-mt8195.md. */
static bool mdp_kd_mm21crop = true;
module_param(mdp_kd_mm21crop, bool, 0644);
MODULE_PARM_DESC(mdp_kd_mm21crop,
	"mt8195 >1920 block-padded MM21 dual vertical-crop KD (default ON; both pipes bit-exact -- off => block-padded MM21 falls to SCP)");

/*
 * DIG-DEEPER batch 3.
 * (1) mdp_kd_dn420 -- un-gate 4:2:0 (2-plane NV12/NV21/NM12/NM21) DSTs for a 4:2:2
 *     SOURCE (NV16/NV61/NM16/NM61 2-plane, YUYV/YVYU/UYVY/VYUY packed). This is the
 *     INVERSE of mdp_kd_up422: the WROT vertical chroma DECIMATE 4:2:2->4:2:0. The
 *     RDMA reads the full-height 4:2:2 chroma (src_ctrl 0x060d / packed) into the pipe
 *     (filterMode 3 passthrough); the WROT's 4:2:0-output control word carries uv_ysel=0
 *     (VIDO_CTRL bits31:30, DpEngine_WROT.cpp: YUV420 output rot0 -> uv_ysel=0) which
 *     decimates the 4:2:2 pipe chroma to 4:2:0. HW-MEASURED: the decimate is a DROP-EVEN
 *     (keep chroma row 2c, drop 2c+1), NOT a [1 1] average -- bit-exact vs a drop-even CPU
 *     ref (PSNR=inf); averaging + drop-odd refs both refuted. Same NV12/NV21
 *     output rows the NV12-source path already uses; only the gate + the NV12-out row
 *     lookup for a 4:2:2 source are new. 3-plane I420/YV12 dsts stay walled (WROT 3-plane
 *     write wall).
 * (2) mdp_kd_rgbr_src -- un-gate the RGB565X (V4L2 'RGBR') SOURCE. RGB565X is big-endian
 *     5-6-5 (the two bytes swapped per pixel vs V4L2 RGB565). The mt8183 RDMA SRC_CON has
 *     NO 16-bit byte-swap bit -- only sourceSwap[14] = the R/B *channel* swap (BGR vs RGB).
 *     mdp_cfg_data.c maps RGB565X -> MDP_COLOR_BGR565 (SWAP set), which the RGB-source read
 *     turns into src_ctrl 0x4600 (R/B channel swap) -- wrong for a byte swap (increment 5:
 *     17.7% exact). This param un-gates it for an empirical HW attempt; mdp_kd_rgbr_noswap
 *     additionally strips the R/B swap bit (read as plain LE RGB565) to pin the symptom.
 */
static bool mdp_kd_dn420 = true;
module_param(mdp_kd_dn420, bool, 0644);
MODULE_PARM_DESC(mdp_kd_dn420,
	"mt8183 un-gate 4:2:0 (2-plane) dsts for a 4:2:2 source via the WROT 4:2:2->4:2:0 vertical decimate (default on; HW bit-exact validated -- drop-even chroma row 2c, PSNR=inf on NV16/UYVY->NV12/NV21)");

static bool mdp_kd_rgbr_src = true;
module_param(mdp_kd_rgbr_src, bool, 0644);
MODULE_PARM_DESC(mdp_kd_rgbr_src,
	"admit RGB565X (V4L2 RGBR / MDP_COLOR_BGR565) as a kernel-direct RGB source (default on). Off: RGBR is not mdp_color_is_rgb_src and must not fall through the mt8195 2-plane NV12 fmtconv catch-all (iova[1]==0 → GCE timeout on mt8188 G5). mt8183 keeps RGBR capture-only so this does not re-enumerate the BE-565 wall.");

static int mdp_kd_rgbr_noswap;
module_param(mdp_kd_rgbr_noswap, int, 0644);
MODULE_PARM_DESC(mdp_kd_rgbr_noswap,
	"DIAG: strip the RGB565X R/B sourceSwap bit (read as plain LE RGB565) for the byte-swap symptom probe (default 0)");

/*
 * DIG-DEEPER batch 4 -- mdp_kd_i420dst_hybrid: I420/YV12 (3-plane planar 4:2:0)
 * DESTINATION via a HW+CPU hybrid (the mt8195 mdp_kd_444hybrid analogue, in reverse).
 * The mt8183 WROT physically cannot WRITE a 3-plane planar dst (I420/YV12) -- confirmed
 * HW wall (source compare + register readback; the pure-HW mdp_kd_fmtmp_planar 3-plane
 * write corrupts the chroma sub-tile offset, ~6122/8192 B differ). BUT the WROT writes
 * 2-plane NV12 (Y + interleaved CbCr) bit-exact for every kernel-direct source. I420/YV12
 * is just NV12 with the interleaved UV plane SPLIT into separate U and V planes -- a
 * trivial CPU de-interleave (pure byte reshuffle, no filtering). So: run the HW
 * conversion to NV12 into the SINGLE output dma-buf (whose Y+U+V region exactly spans an
 * NV12 Y+UV image), then the CPU de-interleaves the CbCr plane -> U-plane + V-plane
 * (I420 = U,V order) or V-plane + U-plane (YV12) IN the same buffer. Any source the
 * kernel-direct fmtconv path can convert to NV12 is admitted (NV12/NV21/NV16/NV61/
 * packed-4:2:2/I420/YV12/RGB); the source read + the WROT NV12 write are the already-
 * validated fmtconv machinery, only the dst-format reconfig + the CPU post-pass are new.
 * MULTI-PLANAR YM12/YM21 (V4L2 YUV420M/YVU420M) are NOT admitted (their U/V planes are
 * separate small dma-bufs that cannot hold the compact NV12 UV plane) -- the gate
 * requires a contiguous single-buffer dst. Default OFF until HW bit-exact validated.
 */
static bool mdp_kd_i420dst_hybrid = true;
module_param(mdp_kd_i420dst_hybrid, bool, 0644);
MODULE_PARM_DESC(mdp_kd_i420dst_hybrid,
	"mt8183 I420/YV12 (3-plane) DST via HW NV12 write + CPU chroma de-interleave (default on; HW bit-exact validated)");

/*
 * mt8183 identity / near-identity closers (matrix follow-ups, NOT HW walls):
 *   (1) NV12/NM12 <-> NV12/NM12 blit -- the trivial 1:1 2-plane copy (single-buffer
 *       <-> multi-planar repack). mt8183's SCP is dead (KI-006 -22) so it had no
 *       path; route it through the 64x32-tiled WROT NV12 write (mdp_fmtconv_nv12,
 *       the GREY->NV12 compute) -- byte-identical output (no CSC, no resample).
 *   (2) MM21/MT21 tiled source -> CONTIGUOUS single-buffer I420/YV12 (YU12/YV12):
 *       the single-buffer twin of the already-validated i420dst_mp mm21 arm. The
 *       gate + arm admit mm21 exactly like the mp path; the HW detile-reads MM21
 *       and writes NV12 into the contiguous dst, then the CPU de-interleaves.
 * Default ON -- board bit-exact validated 2026-07-05 on demo-mt8183-chromebook
 * (kver 6.18.35-00686-gda9d15e358a6, taint 0): matrix 576->584/621 (the exact 8
 * pairs NV12/NM12<->NV12/NM12 + MM21/MT21->YU12/YV12), and md5-bit-exact --
 * cluster 1 md5(out)==md5(in) (identity), cluster 3 md5(KD)==md5(the validated
 * mp arm YM12/YM21). module_param 0644 -> still runtime-toggleable.
 */
static bool mdp_kd_8183_ident = true;
module_param(mdp_kd_8183_ident, bool, 0644);
MODULE_PARM_DESC(mdp_kd_8183_ident,
	"mt8183 NV12<->NM12 blit + MM21/MT21->YU12/YV12 single-buffer I420/YV12 (default on; board bit-exact validated)");

/*
 * mt8183 swapped-source -> NV21/NM21 (V-before-U 4:2:0) dst -- the last mt8183
 * matrix follow-up (14 pairs). swap_twin(NV21)==NV12 is NOT an mdp_fmtconv_tbl
 * row (NV12->NV12 is blit), so mdp_fmtconv_for/lookup returns NULL for the
 * swap-twin of an NV21 dst -> the pair fell to the dead SCP (-22). The sibling
 * swapped-src -> NV12 pairs (NV21/NV61/VYUY/NV42 -> NV12) ALREADY pass (the
 * 4:2:2/4:4:4 -> 4:2:0 decimate is done), so the ONLY gap is the output chroma
 * order: supply the local NV12 row (0x500c, NO 0x100 swap) so the byte-swapped
 * pipe chroma writes straight through as NV21/NM21 order. mt8183-only (the arm
 * substitutions CFG_CHECK MT8183; a non-mt8183 SoC keeps f==NULL and rides its
 * healthy SCP), so it cannot touch mt8195's 702/702. Default ON -- board
 * bit-exact validated 2026-07-05 on demo-mt8183-chromebook (kver
 * 6.18.35-00688-g3358d2bb5f2a, taint 0): matrix 584->598/621 (the exact 14
 * swap-family->NV21/NM21 pairs), all 14 md5-bit-exact (out == the S->NV12
 * output with the UV Cb/Cr swapped; NV21->NV21 also md5(out)==md5(in)).
 */
static bool mdp_kd_8183_swapdst = true;
module_param(mdp_kd_8183_swapdst, bool, 0644);
MODULE_PARM_DESC(mdp_kd_8183_swapdst,
	"mt8183 swapped-src -> NV21/NM21 dst via the local NV12 row (default on; board bit-exact validated)");

/*
 * SCP-tail class #1 -- mt8195 swapped-2-plane 1:1 -> NV21/NM21 dst. The exact
 * mt8195 twin of mdp_kd_8183_swapdst above: swap_twin(NV21)==NV12 is not an
 * mdp_fmtconv_tbl row, so mdp_fmtconv_for returns NULL for a plain-NV21 dst and
 * the pair (NV21->NV21, NV61->NV21, NV42->NV21, NM21->..., YM21->...) fell through
 * the mt8195 single-pipe fmtconv arm (which needs a non-NULL f) to the SCP
 * MDP_FRAME IPI -- part of the mdp_scp_fallback_forbid tail. The mt8195 fmtconv
 * compute (mdp_config_compute_8195_fmtconv, case 0) ALREADY reads NV21 (2-plane
 * 4:2:0), NV61 (2-plane 4:2:2, full-height chroma) and NV42 (2-plane 4:4:4)
 * sources bit-exact (board-validated for the NV12-dst siblings), so the only gap
 * is the output chroma order: supply the local NV12 row (WROT 0x500c, NO 0x100
 * swap) so the byte-swapped pipe chroma writes straight through as NV21/NM21
 * order -- identical mechanism to mt8183. Default ON (IPI-removal
 * readiness).
 */
static bool mdp_kd_swap1to1 = true;
module_param(mdp_kd_swap1to1, bool, 0644);
MODULE_PARM_DESC(mdp_kd_swap1to1,
	"mt8195 swapped-2-plane src (NV21/NV61/NV42/NM21) -> NV21/NM21 dst via the local NV12 row (default on; SCP-tail class #1)");

/*
 * DIG-DEEPER batch 6 -- mdp_kd_i420dst_mp: the MULTI-PLANAR twin of the i420dst
 * hybrid above (V4L2 YUV420M/YVU420M -- YM12/YM21). Same MDP colorformat
 * (MDP_COLOR_I420/YV12, 3 planes) as the single-buffer YU12/YV12, but the U and
 * V planes live in SEPARATE dma-bufs (each only ostride/2 * h/2 bytes), so the
 * compact NV12 interleaved-UV plane (ostride * h/2) does NOT fit contiguously
 * after Y -- the single-buffer hybrid's contiguity gate rejects it and it falls
 * to the dead SCP (-22). Extension: run the SAME HW NV12 write but point the
 * interleaved-UV write at a SCRATCH dma-buf (Y still goes straight to the real
 * Y-plane iova[0], HW-final), then the CPU de-interleaves the scratch UV plane
 * -> the two separate U/V plane dma-bufs (iova[1]/iova[2]) instead of contiguous
 * offsets. Pure buffer-layout extension of the same hybrid mechanism. Default
 * OFF until HW bit-exact validated.
 */
static bool mdp_kd_i420dst_mp = true;
module_param(mdp_kd_i420dst_mp, bool, 0644);
MODULE_PARM_DESC(mdp_kd_i420dst_mp,
	"mt8183 multi-planar YM12/YM21 (I420M/YV12M, 3 separate dma-bufs) DST via HW NV12 write to scratch + CPU chroma de-interleave to the 3 plane iovas (default on; HW bit-exact validated)");

/*
 * mt8195 port of BOTH i420dst hybrids (mdp_kd_i420dst_hybrid + mdp_kd_i420dst_mp,
 * above). Same HW-NV12-write + CPU-chroma-de-interleave mechanism, but the HW pass
 * runs mt8195's single-pipe fmtconv compute (mdp_config_compute_8195_fmtconv,
 * nchunks=1, <=1920 single-line-buffer) instead of the mt8183 64x32-tiled multi-chunk
 * path. The CPU de-interleave in device_run is SoC-agnostic (keys only off the
 * i420_deint* flags + plane strides), so it is reused unchanged. This closes mt8195's
 * last SCP dependency for 2D fmtconv (the 3-plane planar WROT write is a HW wall on
 * mt8195 too). Board bit-exact validated (702/702 format-pair matrix gate clean +
 * an 8/8 NV12->{YU12,YV12,YM12,YM21} md5 sample, KD==SCP), so default ON like the
 * mt8183 i420dst params above.
 */
static bool mdp_kd_i420dst8195 = true;
module_param(mdp_kd_i420dst8195, bool, 0644);
MODULE_PARM_DESC(mdp_kd_i420dst8195,
	"mt8195 I420/YV12 + multi-planar YM12/YM21 (3-plane planar) DST via HW NV12 write + CPU chroma de-interleave (default on; HW bit-exact validated)");

/*
 * SCP-tail class #1 -- mt8195 MM21/MT21 (block-mode 4:2:0) SOURCE -> 3-plane
 * planar 4:2:0 DST (YU12/YV12 contiguous + YM12/YM21 multi-planar). mt8195 has
 * BOTH the MM21 detile READ (mdp_config_compute_8195, the pure detile) AND the
 * 3-plane WRITE (the i420dst HW-NV12 + CPU-de-interleave hybrid), but the hybrid
 * arms route MM21 through mt8195's fmtconv compute -- which has NO MM21 read
 * branch (mt8183-only), so an MM21 source there is mis-read (or excluded to SCP).
 * Wire the two together: an MM21 source -> 3-plane dst reconfigures the HW output
 * to 2-plane NV12 (as the i420dst hybrid does), runs the DETILE compute
 * (mdp_config_compute_8195) so the RDMA block-detile-reads MM21 -> writes linear
 * NV12 (into the contiguous dst buffer, or scratch for the multi-planar case),
 * then the CPU de-interleaves the NV12 UV -> the separate U/V planes. MM21 is
 * 4:2:0 exactly like NV12, so the CPU reference is nv12_to_i420(detile(MM21)).
 */
static bool mdp_kd_mm21dst8195 = true;
module_param(mdp_kd_mm21dst8195, bool, 0644);
MODULE_PARM_DESC(mdp_kd_mm21dst8195,
	"mt8195 MM21/MT21 (block-mode 4:2:0) SOURCE -> 3-plane I420/YV12/YM12/YM21 DST via HW MM21-detile NV12 write + CPU chroma de-interleave (default on; SCP-tail class #1)");

/*
 * KI-016 -- mt8195 MM21/MT21 (block-mode 4:2:0) SOURCE -> the 17 NON-NV12,
 * NON-3-plane fmtconv dsts (GREY, packed-4:2:2 YUYV/YVYU/UYVY/VYUY, 2-plane
 * 4:2:2 NV16/NV61 + NM16/NM61, swapped-semi NV21/NM21/NM61, all RGB). mt8195's
 * fmtconv compute (mdp_config_compute_8195_fmtconv) reads its source LINEARLY via
 * mdp_dual_src_for(), which has NO MM21/block-mode case -- an MM21 source falls to
 * the default 2-plane NV12 linear read, so the block-tiled bytes are consumed as if
 * linear -> TILE-SCRAMBLED output (~5-7 dB). The pure MM21->NV12 detile
 * (mdp_frame_is_mm21_detile -> mdp_config_compute_8195) and the MM21->3-plane hybrid
 * arms (mm21dst8195/i420dst_mp/ym16dst) already detile-read correctly; only the
 * NON-NV12 fmtconv dsts have no correct arm.
 *
 * The fix is the mt8195 twin of the mt8183 mdp_kd_mm21conv (mdp_config_compute_8183_
 * fmtconv srcmm21 branch): make the fmtconv compute's RDMA (c0) emit the DETILE
 * block-mode geometry (src_ctrl 0x860c, mf/sf_bkgd = aw*32 / aw*16, block-padded
 * input height inh, crop to ALIGN(h,32)) -- byte-identical to the validated MM21->NV12
 * detile read (mdp_config_compute_8195 c0) -- while the WROT (c9) keeps the per-dst
 * fmtconv output row `f` unchanged. So MM21->X == (the bit-exact detile block read) +
 * (the bit-exact NV12->X WROT write), one pass. Firmware-independent (kernel-direct,
 * no SCP config-IPI). Board-validated bit-exact vs the independent PSNR golden (the 17
 * pairs score at the nv12->dst floor, 0 outliers), so DEFAULT ON like the
 * mdp_kd_i420dst8195 pattern. Set to 0 at runtime to fall the 17 back to the (KI-016)
 * linear read.
 */
static bool mdp_kd_mm21conv8195 = true;
module_param(mdp_kd_mm21conv8195, bool, 0644);
MODULE_PARM_DESC(mdp_kd_mm21conv8195,
	"mt8195 MM21/MT21 (block-mode 4:2:0) SOURCE -> the 17 non-NV12 fmtconv dsts via the detile block-mode RDMA read chained to the NV12->X WROT write (KI-016 fix; default on, board-validated bit-exact)");

/*
 * SCP-tail class #2 -- mt8195 3-plane 4:2:2 DST (YM16 = V4L2 YUV422M/MDP_COLOR_I422,
 * YM61 = YVU422M/MDP_COLOR_YV16). The WROT cannot WRITE a 3-plane planar 4:2:2 dst
 * (the same write wall as the 3-plane 4:2:0 I420/YV12 -- confirmed on mt8195). Use
 * the i420dst-hybrid mechanism but for 4:2:2: reconfigure the HW output to 2-plane
 * NV16 (Y + interleaved CbCr, FULL-height chroma), run the already-validated
 * source->NV16 fmtconv machinery (NV16 is a real mdp_fmtconv_tbl row -- the source
 * families that convert to NV16 in the main fmtconv arm work here unchanged), then
 * the CPU de-interleaves the NV16 CbCr plane -> the separate U/V planes. Unlike the
 * 4:2:0 hybrid the chroma is FULL height (hc == h), so the de-interleave + scratch
 * size are h rows (not h/2). YM16/YM61 are multi-planar (separate U/V dma-bufs), so
 * the interleaved CbCr goes to scratch (like the i420dst_mp arm), never contiguously
 * after Y. Block-mode (MM21) and RGB/packed-source niches are excluded (the fmtconv
 * NV16 read must be plain -- validated per pair by md5).
 */
static bool mdp_kd_ym16dst8195 = true;
module_param(mdp_kd_ym16dst8195, bool, 0644);
MODULE_PARM_DESC(mdp_kd_ym16dst8195,
	"mt8195 YM16/YM61 (3-plane planar 4:2:2) DST via HW NV16 write + CPU full-height chroma de-interleave (default on; SCP-tail class #2)");

/*
 * GENERIC >1920 (DUAL_BITBLT) dual-pipe arm for the 3-plane-multiplanar-dst hybrids
 * (I420/YV12 contiguous, YM12/YM21 multi-planar 4:2:0, YM16/YM61 multi-planar 4:2:2).
 * The <=1920 hybrids reconfigure the dst to 2-plane NV12/NV16, run the single-pipe
 * fmtconv compute, and CPU de-interleave the compact interleaved-UV plane into the
 * separate 3 planes (mtk-mdp3-m2m.c). At >1920 only a handful of specific source->
 * 3-plane-dst combos got a dual arm; the rest fell to the SCP. This knob generalizes
 * the de1a33c generic-dual-fmtconv treatment to the 3-plane-dst branches: for ANY
 * source->3-plane-multiplanar-dst the <=1920 hybrid claims, derive the per-family
 * source read from the shared mdp_dual_src_for(), drive the dual NV12/NV16 write over
 * both pixel pipes (mdp_fill_dual_fmtconv_8195), and reuse the SAME whole-frame CPU
 * de-interleave (which is post-HW and already composes across both pipes). Placed
 * BEFORE the specific 3-plane dual arms so gen3p=Y subsumes them (byte-identical read/
 * WROT for their classes -- proven vs the <=1920 arms) + closes the uncovered families
 * (YM12/YM21 whole branch, YM16/YM61 from most sources, I420/YV12 from NV21/NM21). A
 * clean A/B lever: gen3p=N drops the wide 3-plane tail back to the specific arms + SCP.
 * MM21/MT21-tiled sources need the block-detile dual read and stay excluded (SCP).
 */
static bool mdp_kd_gen3p = true;
module_param(mdp_kd_gen3p, bool, 0644);
MODULE_PARM_DESC(mdp_kd_gen3p,
	"mt8195 generic >1920 dual-pipe 3-plane-multiplanar-dst (I420/YV12/YM12/YM21/YM16/YM61) via HW NV12/NV16 dual write + CPU de-interleave (default on)");

/* 3-plane planar 4:2:2 DST (YM16 = MDP_COLOR_I422, YM61 = MDP_COLOR_YV16): the
 * direction-agnostic twin of mdp_color_is_ym16src (group 1, hw_format 9, 3 planes,
 * full-height chroma). SWAP distinguishes YM16 (U-first) from YM61 (V-first). */
static bool mdp_color_is_ym16dst(u32 c)
{
	return mdp_color_is_ym16src(c);
}

/* Plain packed RGB source (single plane, group RGB=0): RGB565/BGR565 (HW_FORMAT 0,
 * 16bpp), RGB888/BGR888 (1, 24bpp), and the 32-bit 8888 family -- BGRA/RGBA (HW_FORMAT
 * 2) AND ARGB/ABGR (HW_FORMAT 3); the alpha position selects 2 vs 3 (V4L2 AR24/ARGB32
 * == MDP_COLOR_BGRA8888 = fmt 2, V4L2 BA24/ABGR32 == MDP_COLOR_ARGB8888 = fmt 3). The
 * SWAP bit distinguishes R/B order (RGB vs BGR), handled by the RDMA sourceSwap, NOT a
 * twin. (HW-confirmed 2026-06-30: omitting fmt 2 made AR24/ARGB32 fall to the SCP -22.)
 * RGB565X (V4L2 'RGBR') == MDP_COLOR_BGR565 (hf 0 + SWAP) is GATED OUT here: its
 * big-endian-565 RDMA read is wrong (HW-validated only 17.7% exact), so an RGB565X
 * source routes to the SCP (-22), NOT the kernel-direct path. The other RGB sources
 * (RGB888/BGR888/RGB565/ARGB/ABGR) stay accepted. */
static bool mdp_color_is_rgb_src(u32 c)
{
	u32 hf = MDP_COLOR_GET_HW_FORMAT(c);

	return MDP_COLOR_GET_GROUP(c) == 0 &&
	       MDP_COLOR_GET_PLANE_COUNT(c) == 1 &&
	       (mdp_kd_rgbr_src || c != MDP_COLOR_BGR565) &&
	       (hf == 0 || hf == 1 || hf == 2 || hf == 3);
}

/* 4:2:2 semi-planar dst: NV16/NV61 (contiguous) and NM16/NM61 (multi-planar) --
 * both map to MDP_COLOR_NV16 / MDP_COLOR_NV61 (the plane layout differs only in
 * the per-plane iova, read from the out buffer). The single-plane-source fmtconv
 * arm targets exactly this dst class (WROT plain NV16/NV61 row + RDMA CSC). */
static bool mdp_color_is_422semi_dst(u32 c)
{
	return c == MDP_COLOR_NV16 || c == MDP_COLOR_NV61;
}

/*
 * Exactly RGB888 (V4L2 RGB24): group RGB, hw_format 1, SWAP set. The CSC compute
 * only emits RGB888 (control=0xa0005101, stride=w*3, mat_ctrl=0x61); BGR888 (SWAP
 * clear), RGB565 (hw_format 0, 16bpp) and ARGB/ABGR (hw_format 3, 32bpp) share
 * group 0 but would be mis-written (format-sweep confirmed KD-WRONG).
 */
static bool mdp_color_is_rgb888(u32 c)
{
	return MDP_COLOR_GET_GROUP(c) == 0 && MDP_COLOR_GET_HW_FORMAT(c) == 1 &&
	       MDP_COLOR_IS_SWAPPED(c);
}

/*
 * NV12 -> <fmt> 1:1 format-conversion table (mt8195). Every NV12->* convert uses
 * the SAME 1:1 SVPP2 pipe as blit/CSC; only the RDMA read mode + the WROT output
 * format differ, so one parameterized compute (mdp_config_compute_8195_fmtconv)
 * handles them all. Each row is TRANSCRIBED from the v23042 SCP oracle (the WROT
 * control word does NOT follow MDP_COLOR's hw_format -- e.g. ARGB/ABGR -- so it
 * must be a per-format constant, never computed). RE + full capture:
 * the internal 2D-ops oracle record, "Format-conversion matrix".
 *
 *  - src_ctrl: RDMA c0 read mode. 0x60c ONLY for a 4:2:0-YUV output (NV12/NV21/
 *    I420/YV12, same subsample as the input); 0xc for GREY / 4:2:2 / RGB.
 *  - control:  WROT c9 output-format word (low byte = fmt id + swap bit 0x100,
 *    high nibble = class: 0x0 4:2:0-YUV/GREY, 0x8 4:2:2-YUV, 0xa RGB).
 *  - pre_ultra: WROT FIFO pre-ultra throttle (per bpp class; non-correctness, but
 *    transcribed). mat_ctrl=0x61 for the RGB rows (the YUV->RGB matrix select).
 * iova[0..2] + stride[0..2] are read from the OUT buffer (the driver fills the
 * per-plane addresses/pitches), so they are not in the table.
 *
 * NV12 (-> blit), RGB888/RGB24 (-> CSC) and NM12 (NV12 twin -> blit) are handled
 * by their own paths and are NOT listed here.
 */
struct mdp_fmtconv { enum mdp_color color; u32 src_ctrl, control, pre_ultra; bool rgb; };
static const struct mdp_fmtconv mdp_fmtconv_tbl[] = {
	{ MDP_COLOR_NV21,     0x60c, 0x0000510c, 0x000d80c4, false },
	{ MDP_COLOR_I420,     0x60c, 0x00005008, 0x000d80c4, false },
	{ MDP_COLOR_YV12,     0x60c, 0x00005108, 0x000d80c4, false },
	{ MDP_COLOR_GREY,     0xc,   0x00005007, 0x000d80c4, false },
	{ MDP_COLOR_NV16,     0xc,   0x8000500d, 0x000d80c4, false },
	{ MDP_COLOR_NV61,     0xc,   0x8000510d, 0x000d80c4, false },
	{ MDP_COLOR_YUYV,     0xc,   0x80005005, 0x000b0088, false },
	{ MDP_COLOR_YVYU,     0xc,   0x80005105, 0x000b0088, false },
	{ MDP_COLOR_UYVY,     0xc,   0x80005004, 0x000b0088, false },
	{ MDP_COLOR_VYUY,     0xc,   0x80005104, 0x000b0088, false },
	{ MDP_COLOR_RGB565,   0xc,   0xa0005000, 0x0008804c, true  },
	/* RGB565X (V4L2 'RGBR', big-endian 5-6-5) == MDP_COLOR_BGR565 (SWAP set):
	 * same RGB pipe as RGB565; the byte-swap is the WROT control swap bit 0x100
	 * (0xa0005000 -> 0xa0005100). Transcribed from the v23042 SCP oracle -- the
	 * only WROT diff vs RGB565 is that swap bit; the CSC matrix is unchanged
	 * (mat_ctrl=0x61). RE: audit/2d-ops-oracle-mt8195.md "Format-conversion matrix". */
	{ MDP_COLOR_BGR565,   0xc,   0xa0005100, 0x0008804c, true  },
	{ MDP_COLOR_BGR888,   0xc,   0xa0005001, 0x0008804c, true  },
	/* RGB888 (V4L2 RGB24) -- the WROT control (0xa0005101, SWAP set over BGR888's
	 * 0xa0005001) + mat_ctrl=0x61 are the mt8195 CSC oracle's RGB888 values, the
	 * SAME the dedicated NV12->RGB24 CSC compute (mdp_config_compute_8183_csc) uses.
	 * NV12->RGB24 still routes through that CSC path (checked first at dispatch); this
	 * row lets a YUV SOURCE -> RGB24 ride the multi-source fmtconv path (mdp_kd_yuv2rgb). */
	{ MDP_COLOR_RGB888,   0xc,   0xa0005101, 0x0008804c, true  },
	/* The V4L2 ARGB/ABGR naming vs MDP_COLOR is inverted: V4L2 ARGB32 (AR24,
	 * colorformat 0x202002) == MDP_COLOR_BGRA8888 and needs control 0xa0005002;
	 * V4L2 ABGR32 (BA24, 0x202023) == MDP_COLOR_ARGB8888 and needs 0xa0005103.
	 * Transcribed from the oracle by the colorformat VALUE, not the name. */
	{ MDP_COLOR_BGRA8888, 0xc,   0xa0005002, 0x00060010, true  },
	{ MDP_COLOR_ARGB8888, 0xc,   0xa0005103, 0x00060010, true  },
};

static const struct mdp_fmtconv *mdp_fmtconv_lookup(u32 colorformat)
{
	int i;

	for (i = 0; i < (int)ARRAY_SIZE(mdp_fmtconv_tbl); i++)
		if (mdp_fmtconv_tbl[i].color == colorformat)
			return &mdp_fmtconv_tbl[i];
	return NULL;
}

/*
 * NV12 output is intentionally excluded from mdp_fmtconv_tbl (NV12->NV12 is blit's
 * job, NV12->* the convert matrix). But an I420/YV12 3-plane SOURCE can target NV12,
 * so this local row supplies the WROT NV12 control (0x500c) for that path. src_ctrl
 * here is unused on mt8183 (the compute overrides it to the 3-plane 0x608); kept at
 * the NV12 0x60c for documentation parity with the table.
 */
static const struct mdp_fmtconv mdp_fmtconv_nv12 = {
	MDP_COLOR_NV12, 0x60c, 0x0000500c, 0x000d80c4, false
};

/*
 * Chroma-swapped twin of an mdp_color: NV12<->NV21, NV16<->NV61, YUYV<->YVYU,
 * UYVY<->VYUY, I420<->YV12, GREY->GREY (no chroma). Used to route an NV21 source
 * through the existing NV12-source computes: NV21->D == NV12->swap_twin(D).
 */
static enum mdp_color mdp_chroma_swap_twin(enum mdp_color c)
{
	switch (c) {
	case MDP_COLOR_NV12:	return MDP_COLOR_NV21;
	case MDP_COLOR_NV21:	return MDP_COLOR_NV12;
	case MDP_COLOR_NV16:	return MDP_COLOR_NV61;
	case MDP_COLOR_NV61:	return MDP_COLOR_NV16;
	case MDP_COLOR_NV24:	return MDP_COLOR_NV42;
	case MDP_COLOR_NV42:	return MDP_COLOR_NV24;
	case MDP_COLOR_YUYV:	return MDP_COLOR_YVYU;
	case MDP_COLOR_YVYU:	return MDP_COLOR_YUYV;
	case MDP_COLOR_UYVY:	return MDP_COLOR_VYUY;
	case MDP_COLOR_VYUY:	return MDP_COLOR_UYVY;
	case MDP_COLOR_I420:	return MDP_COLOR_YV12;
	case MDP_COLOR_YV12:	return MDP_COLOR_I420;
	case MDP_COLOR_GREY:	return MDP_COLOR_GREY;
	default:		return c;
	}
}

/*
 * Select the fmtconv table row for a frame. A plain-NV12 source uses the output
 * color directly. A plain-NV21 source (mdp_kd_src_nv21) reads with the IDENTICAL
 * NV12 RDMA geometry and compensates the swapped chroma by selecting the
 * chroma-swapped-twin row (its WROT control carries the 0x100 swap bit).
 */
static const struct mdp_fmtconv *
mdp_fmtconv_for(const struct img_ipi_frameparam *param)
{
	u32 out_color = param->outputs[0].buffer.format.colorformat;
	u32 in_color = param->inputs[0].buffer.format.colorformat;

	/* I420/YV12 3-plane source: the read normalizes YV12 (iova[1]<->iova[2] swap in
	 * the compute) to true YUV, so the output row is the PLAIN output color -- no
	 * swap-twin. NV12 output is not in the NV12->* table (NV12->NV12 is blit), so
	 * supply the local NV12 row; NV21/GREY come from the table. */
	if (mdp_kd_src_i420 && mdp_color_is_plain_i420src(in_color)) {
		if (mdp_color_is_plain_nv12(out_color))
			return &mdp_fmtconv_nv12;
		return mdp_fmtconv_lookup(out_color);
	}

	/* YM16/YM61 (3-plane 4:2:2) source: the RDMA reads FULL-height chroma into
	 * the pipe (src_ctrl 0x609/0x4609); YM61's V-first order is un-swapped by the
	 * 0x4000 bit (oracle YM61>*=0x4609), so the pipe carries true U/V and the WROT
	 * writes the PLAIN output row -- no swap-twin, for BOTH YM16 and YM61 (like the
	 * i420/YV12 3-plane read above). NV12 out is not a table row -> the local NV12
	 * row (WROT 4:2:2->4:2:0 vertical decimate). */
	if (mdp_kd_ym16src && mdp_color_is_ym16src(in_color)) {
		if (mdp_color_is_plain_nv12(out_color))
			return &mdp_fmtconv_nv12;
		return mdp_fmtconv_lookup(out_color);
	}

	/* RGB source -> YUV: the RGB->YUV CSC is at the RDMA (rdma.transform); the WROT
	 * writes the PLAIN YUV output. No swap-twin (the R/B order is the RDMA sourceSwap,
	 * not a YUV chroma swap). NV12 out is not in the NV12->* table -> the local row. */
	if (mdp_kd_src_rgb && mdp_color_is_rgb_src(in_color)) {
		if (mdp_color_is_plain_nv12(out_color))
			return &mdp_fmtconv_nv12;
		return mdp_fmtconv_lookup(out_color);
	}

	/* NV24 (unswapped 4:4:4 2-plane) source: the RDMA reads the full-res chroma into
	 * the pipe; the WROT decimates H and/or V per the TARGET's sampling (the transcribed
	 * control words), so EVERY output uses its plain row. NV12 out is not in the NV12->*
	 * table (NV12->NV12 is blit) -> the local NV12 row (WROT 0x500c = uv_xsel=0/uv_ysel=0,
	 * decimate both axes from the 4:4:4 pipe). Swapped NV42 is handled by the swap-twin
	 * block below. */
	if (mdp_kd_src_nv24 && mdp_color_is_plain_nv24src(in_color) &&
	    !MDP_COLOR_IS_SWAPPED(in_color)) {
		if (mdp_color_is_plain_nv12(out_color))
			return &mdp_fmtconv_nv12;
		return mdp_fmtconv_lookup(out_color);
	}

	/* 4:2:2 source (NV16/packed) -> plain NV12 (4:2:0) DECIMATE (mdp_kd_dn420): NV12 is
	 * not in the NV12->* table (NV12->NV12 is blit), so supply the local NV12 row (WROT
	 * uv_ysel=0 vertical decimate of the full-height 4:2:2 pipe chroma). Only for an
	 * UNSWAPPED source -- a swapped one (NV61/YVYU/VYUY) -> NV12 routes through the
	 * swap-twin below (lookup(NV21), whose 0x100 bit compensates the swapped chroma). */
	if (mdp_kd_dn420 && mdp_color_is_plain_nv12(out_color) &&
	    ((mdp_kd_src_nv16 && mdp_color_is_plain_nv16src(in_color) &&
	      !MDP_COLOR_IS_SWAPPED(in_color)) ||
	     (mdp_kd_src_yuyv && mdp_color_is_packed422src(in_color) &&
	      !MDP_COLOR_IS_SWAPPED(in_color))))
		return &mdp_fmtconv_nv12;

	/* Chroma-swap-twin sources: NV21 (4:2:0), NV61 (4:2:2 2-plane) and the swapped
	 * packed 4:2:2 (YVYU/VYUY). All are read with the UNSWAPPED twin's geometry
	 * (NV12 / NV16 / YUYV / UYVY) so the chroma lands byte-swapped in the pipe; the
	 * swap_twin(out) row's WROT 0x100 swap bit compensates. NV16/YUYV/UYVY sources
	 * (unswapped) and NV12 source use the output row directly. */
	if ((mdp_kd_src_nv21 && mdp_color_is_plain_nv21(in_color)) ||
	    (mdp_kd_src_nv16 && mdp_color_is_plain_nv16src(in_color) &&
	     MDP_COLOR_IS_SWAPPED(in_color)) ||
	    (mdp_kd_src_yuyv && mdp_color_is_packed422src(in_color) &&
	     MDP_COLOR_IS_SWAPPED(in_color)) ||
	    (mdp_kd_src_nv24 && mdp_color_is_plain_nv24src(in_color) &&
	     MDP_COLOR_IS_SWAPPED(in_color)))
		return mdp_fmtconv_lookup(mdp_chroma_swap_twin(out_color));
	return mdp_fmtconv_lookup(out_color);
}

/*
 * Only the pure MM21 block-mode -> linear NV12 detile, in the single-SVPP-pipe
 * regime mdp_config_compute_8195 models (it emits ONE full-width pipe, totw=W,
 * nsub=1). Everything else must fall through to the SCP:
 *  - any scale (h or v), rotate or flip -> different pipeline;
 *  - out area >= 2073600 px -> the SCP splits across TWO pixel-pipes (measured
 *    on mt8195: 1920x1080 dual, but 1920x1072 / 1664x1080 / 2560x720 single);
 *  - out width > 1920 -> single-pipe but the SCP adds tile overlap to totw
 *    (2560x720 -> totw 2588 != W), which the compute does not model.
 * Non-64-aligned widths ARE handled (single AND dual): MM21 luma stride =
 * ALIGN(w,64)*32, so the read-side bkgd/iova_end use that aligned width (aw); only
 * the NV12 write side uses the real w. The dual path additionally sets c0/c1/c2
 * (RDMA/RSZ/relay) in_width=aw (the SCP does this for dual but NOT single).
 * The accepted regime is HW-validated bit-exact (tests/mtk-mdp-kernel-direct-test.sh).
 */
/*
 * Full-frame (no sub-region) output crop test that tolerates a block-mode source.
 * mdp_set_dst_config sets out->crop from the SOURCE frame's crop (= the source
 * W/H); for an MM21/MT21 block-mode source that is the block-PADDED height (e.g.
 * 1440 -> the 64-row-aligned 1472 at a >1920 DUAL size), NOT the real output
 * height -- so a genuine full-frame op arrives with crop.height == the padded
 * INPUT height, not the output height. Accept the crop when it equals EITHER the
 * output dims OR the (block-padded) input dims. For a LINEAR source in-dims ==
 * out-dims, so this is byte-for-byte the old `crop == out-dims` test (no-op).
 */
static bool mdp_crop_is_fullframe(const struct img_ipi_frameparam *param)
{
	const struct img_crop *c = &param->outputs[0].crop;
	const struct img_image_buffer *in = &param->inputs[0].buffer;
	const struct img_image_buffer *out = &param->outputs[0].buffer;

	if (c->left || c->top)
		return false;
	if (c->width != out->format.width && c->width != in->format.width)
		return false;
	if (c->height != out->format.height && c->height != in->format.height)
		return false;
	return true;
}

/*
 * Route selector for the >1920 dual-pipe MM21/MT21 KD compute. Two dual primitives:
 * in.height == out.height (64-aligned, NO block padding) uses the num_subfrms=1 detile
 * dual (mdp_fill_dual_pipe_8195, RSZ bypassed) -- bit-exact incl 4K, always KD. A
 * block-padded height (in.height > out.height, e.g. 1440 -> the 64-row-aligned 1472 at
 * a DUAL size) uses the num_subfrms=2 vertical-crop dual (mdp_fill_dual_pipe_8195_crop,
 * RSZ active in=inh out=h) -- now BIT-EXACT on both pixel pipes (pp0+pp1 cmp-fit
 * 0-byte-diff, md5(KD)==md5(SCP) at 2560x1440 + 3840x2160; fix 1d263507). This is
 * default KD (SUPERSEDES the 941f620 SCP route); mdp_kd_mm21crop=0 at runtime falls
 * block-padded MM21 back to SCP. Guards every >1920 MM21 dual arm (detile + genmm21
 * fmtconv/i420dst/i420dst-mp). The <=1920 single-pipe MM21 path handles block padding
 * via its own multi-subframe scheduler and is unaffected.
 */
static bool mdp_mm21_dual_kd_exact(const struct img_ipi_frameparam *param)
{
	u32 ih = param->inputs[0].buffer.format.height;
	u32 oh = param->outputs[0].buffer.format.height;

	/* in==out uses the num_subfrms=1 detile dual (mdp_fill_dual_pipe_8195, always KD).
	 * A block-padded height (in > out, < 64 rows of pad) uses the num_subfrms=2
	 * vertical-crop dual (mdp_fill_dual_pipe_8195_crop) -- bit-exact both pipes, KD by
	 * default (mdp_kd_mm21crop); set the knob to 0 to fall block-padded back to SCP.
	 * The fill sites pick crop vs detile by inh>h. */
	if (ih == oh)
		return true;
	return mdp_kd_mm21crop && ih > oh && ih - oh < 64;
}

static bool mdp_frame_is_mm21_detile(const struct img_ipi_frameparam *param)
{
	const struct img_image_buffer *in, *out;
	u32 w, h;

	if (param->num_inputs != 1 || param->num_outputs != 1)
		return false;
	in = &param->inputs[0].buffer;
	out = &param->outputs[0].buffer;
	w = out->format.width;
	h = out->format.height;

	if (!(in->format.colorformat & 0x00800000))	/* MDP_COLOR block-mode bit */
		return false;
	/* out: plain NV12 only -- the WROT writes NV12, so the gate must not
	 * over-accept the output format (MM21->NV21/NV16/... would be mis-written). */
	if (!mdp_color_is_plain_nv12(out->format.colorformat))
		return false;
	/* No horizontal scale. The input width is the image width (w, single-pipe) or
	 * the 64-aligned buffer width (ALIGN(w,64), dual-pipe) -- both are the same
	 * image (the extra columns are padding the detile crops out, not a scale). */
	if (in->format.width != w && in->format.width != ALIGN(w, 64))
		return false;
	/* no vertical scale: in height is just h block-padded (< 64 rows) */
	if (in->format.height < h || in->format.height - h >= 64)
		return false;
	/* MM21 luma tile-row stride is ALIGN(w,64)*32; the read-side bkgd/iova_end
	 * fields use this aligned width (aw), the NV12 write side uses the real w.
	 * Accepting it lets non-64-aligned widths go kernel-direct too. */
	if (in->format.plane_fmt[0].stride != ALIGN(w, 64) * 32)
		return false;
	if (param->outputs[0].rotation ||
	    (param->outputs[0].flags & IMG_CTRL_FLAG_HFLIP))
		return false;
	/* full-frame output only -- a compose/crop sub-region needs a different config
	 * (block-mode source: crop carries the padded input dims -- see the helper) */
	if (!mdp_crop_is_fullframe(param))
		return false;
	return true;	/* mode-specific size bounds applied at the call site */
}

/*
 * Kernel-direct CSC gate: plain NV12 in -> RGB888 (RGB24) out, 1:1, no rotate/flip,
 * full-frame (no crop sub-region). Sibling of the detile gate.
 *
 * The CSC pipe is topologically identical to the detile (RDMA->RSZ->relay->PQ
 * chain->WROT); the only conversion-specific knob is the WROT mat_ctrl
 * matrix-SELECT (=0x61, YUV->RGB). The RE + design are in the internal
 * CSC oracle record. Unlike the detile, the input
 * is LINEAR NV12 (block-mode bit 0x00800000 CLEAR), so the read-side stride is
 * the plain w-pitch, not the MM21 aw*32 tile-row stride. SoC-independent (copied
 * verbatim from the mt8183 CSC draft 8d56a26b639f).
 */
static bool mdp_frame_is_nv12_rgb_csc(const struct img_ipi_frameparam *param)
{
	const struct img_image_buffer *in, *out;
	u32 w, h;

	if (param->num_inputs != 1 || param->num_outputs != 1)
		return false;
	in = &param->inputs[0].buffer;
	out = &param->outputs[0].buffer;
	w = out->format.width;
	h = out->format.height;

	/* in: plain linear NV12 (rejects NV21/NM21 + block-mode); out: RGB888 only */
	if (!mdp_color_is_plain_nv12(in->format.colorformat))
		return false;
	if (!mdp_color_is_rgb888(out->format.colorformat))
		return false;
	/* 1:1 -- no horizontal/vertical scale (in dims == out dims) */
	if (in->format.width != w || in->format.height != h)
		return false;
	if (param->outputs[0].rotation ||
	    (param->outputs[0].flags & IMG_CTRL_FLAG_HFLIP))
		return false;
	/* full-frame output only -- a compose/crop sub-region needs a different config */
	if (param->outputs[0].crop.left || param->outputs[0].crop.top ||
	    param->outputs[0].crop.width != w || param->outputs[0].crop.height != h)
		return false;
	return true;	/* size bounds applied at the call site */
}

/*
 * Kernel-direct blit gate: linear NV12 in -> NV12 out, 1:1, no rotate/flip,
 * full-frame. Sibling of the CSC + detile gates. blit is the simplest of the
 * remaining SCP-path 2D ops to free from the SCP (KI-009): the same single-SVPP2
 * pipe as CSC, but the WROT writes NV12 (not RGB) so there is no CSC matrix.
 * RE: the internal 2D-ops oracle record.
 */
static bool mdp_frame_is_nv12_blit(const struct img_ipi_frameparam *param)
{
	const struct img_image_buffer *in, *out;
	u32 w, h;

	if (param->num_inputs != 1 || param->num_outputs != 1)
		return false;
	in = &param->inputs[0].buffer;
	out = &param->outputs[0].buffer;
	w = out->format.width;
	h = out->format.height;

	/* in AND out: plain linear NV12 (rejects NV21/NM21, which share hw_format 12) */
	if (!mdp_color_is_plain_nv12(in->format.colorformat) ||
	    !mdp_color_is_plain_nv12(out->format.colorformat))
		return false;
	/* 1:1 -- no scale */
	if (in->format.width != w || in->format.height != h)
		return false;
	if (param->outputs[0].rotation ||
	    (param->outputs[0].flags & IMG_CTRL_FLAG_HFLIP))
		return false;
	/* full-frame output only */
	if (param->outputs[0].crop.left || param->outputs[0].crop.top ||
	    param->outputs[0].crop.width != w || param->outputs[0].crop.height != h)
		return false;
	return true;	/* size bounds applied at the call site */
}

/*
 * Kernel-direct rotate-90-CW gate: linear NV12 in -> NV12 out, rotated 90 deg,
 * full-frame. The output buffer carries the SWAPPED dims (out W == in H, out H ==
 * in W). Sibling of the blit gate. Only pure rotate-90 (no flip); 180/270/flip
 * need their own nsub=1 WROT main_buf RE (audit/2d-ops-oracle-mt8195.md).
 * V4L2_CID_ROTATE maps directly to outputs[0].rotation (90/180/270), and a 90/270
 * rotation swaps the capture compose -> the output buffer dims (mdp_set_orientation
 * / mdp_set_dst_config).
 */
static bool mdp_frame_is_nv12_rotate90(const struct img_ipi_frameparam *param)
{
	const struct img_image_buffer *in, *out;

	if (param->num_inputs != 1 || param->num_outputs != 1)
		return false;
	in = &param->inputs[0].buffer;
	out = &param->outputs[0].buffer;

	/* in AND out: plain linear NV12 (rejects NV21/NM21, which share hw_format 12) */
	if (!mdp_color_is_plain_nv12(in->format.colorformat) ||
	    !mdp_color_is_plain_nv12(out->format.colorformat))
		return false;
	/* pure rotate-90 (no flip), output dims = swapped input dims */
	if (param->outputs[0].rotation != 90 ||
	    (param->outputs[0].flags & IMG_CTRL_FLAG_HFLIP))
		return false;
	if (out->format.width != in->format.height ||
	    out->format.height != in->format.width)
		return false;
	/* full-frame source only. The output-queue crop is defaulted to the SOURCE
	 * dims on S_FMT (mtk-mdp3-m2m.c mdp_default_or_keep_rect, input W x H), NOT
	 * the rotated output dims -- so check against the INPUT frame, not out. */
	if (param->outputs[0].crop.left || param->outputs[0].crop.top ||
	    param->outputs[0].crop.width != in->format.width ||
	    param->outputs[0].crop.height != in->format.height)
		return false;
	return true;	/* size bounds applied at the call site */
}

/*
 * The remaining NV12->NV12 transforms (rotate-180/270, hflip, vflip) share blit's
 * pipe and differ only in the WROT control + the VIDO output-start offset
 * (u[18]=Y, u[19]=u[20]=UV), per the v23042 SCP oracle (nsub=1 captures at
 * 256² + 256x128 + 128x256, audit/2d-ops-oracle-mt8195.md). The offset has three
 * shapes (ostride = output row width, read from the out buffer):
 *   ROW_REV  (hflip):    u[18..20] = ostride - 1            (reverse within a row)
 *   FULL_REV (rotate180): u[18]=w*h-1,      u[19,20]=w*h/2-1
 *   COL_REV  (rotate270, vflip): u[18]=w*h-ostride, u[19,20]=w*h/2-ostride
 * (rotate-90's u[18..20]=ostride-1 is ROW_REV too, but it keeps its own path.)
 */
enum mdp_wrot_ofst { MDP_OFST_ROW_REV, MDP_OFST_FULL_REV, MDP_OFST_COL_REV };

/*
 * Map the frameparam orientation to (WROT control, offset mode). V4L2 maps
 * rotate -> outputs[0].rotation and the flips through mdp_set_orientation:
 * hflip -> rotation=0 + HFLIP; vflip -> rotation=180 + HFLIP; rotate180 ->
 * rotation=180; rotate270 -> rotation=270. rotate-90 (its own path) and any
 * rotate+flip combo are NOT handled here (return false -> SCP). Returns whether
 * the variant is one of the four kernel-direct transforms.
 */
static bool mdp_xform_variant(const struct img_ipi_frameparam *param,
			      u32 *control, enum mdp_wrot_ofst *mode)
{
	s32 rot = param->outputs[0].rotation;
	bool hflip = param->outputs[0].flags & IMG_CTRL_FLAG_HFLIP;

	if (rot == 180 && !hflip) { *control = 0x5020500c; *mode = MDP_OFST_FULL_REV; return true; }
	if (rot == 270 && !hflip) { *control = 0x1030500c; *mode = MDP_OFST_COL_REV;  return true; }
	if (rot == 0   &&  hflip) { *control = 0x1100500c; *mode = MDP_OFST_ROW_REV;  return true; }
	if (rot == 180 &&  hflip) { *control = 0x4120500c; *mode = MDP_OFST_COL_REV;  return true; }
	return false;
}

/*
 * Kernel-direct gate for the rotate-180/270 + hflip/vflip transforms (sibling of
 * the blit + rotate-90 gates). Linear NV12 in/out, full-frame, one of the four
 * variants mdp_xform_variant recognises. rotate-270 swaps the output dims (out W
 * == in H), the others keep them. Like rotate-90, the output-queue crop is in
 * SOURCE dims -- check against the INPUT frame.
 */
static bool mdp_frame_is_nv12_xform(const struct img_ipi_frameparam *param)
{
	const struct img_image_buffer *in, *out;
	u32 control, ew, eh;
	enum mdp_wrot_ofst mode;
	bool swap;

	if (param->num_inputs != 1 || param->num_outputs != 1)
		return false;
	in = &param->inputs[0].buffer;
	out = &param->outputs[0].buffer;

	/* in AND out: plain linear NV12 (rejects NV21/NM21, which share hw_format 12) */
	if (!mdp_color_is_plain_nv12(in->format.colorformat) ||
	    !mdp_color_is_plain_nv12(out->format.colorformat))
		return false;
	if (!mdp_xform_variant(param, &control, &mode))
		return false;
	/* rotate-270 swaps the output dims; rotate-180/hflip/vflip keep them */
	swap = (param->outputs[0].rotation == 270);
	ew = swap ? in->format.height : in->format.width;
	eh = swap ? in->format.width : in->format.height;
	if (out->format.width != ew || out->format.height != eh)
		return false;
	/* full-frame source (output-queue crop is in INPUT dims, see rotate-90) */
	if (param->outputs[0].crop.left || param->outputs[0].crop.top ||
	    param->outputs[0].crop.width != in->format.width ||
	    param->outputs[0].crop.height != in->format.height)
		return false;
	return true;	/* size bounds applied at the call site */
}

/*
 * NV12 -> GREY (Y8) rotation: the single-output-plane proof of the separate-plane
 * workaround for the WROT Y<->UV demux swap (rotate the Y plane alone, no UV write
 * -> no swap). GREY WROT control = the fmtconv GREY base 0x5007 (fmt id 0x07 + crop
 * + pre-ultra) | rot<<20 | flip<<24, with NO uv_xsel/ysel bits. Returns the control
 * + offset mode and whether this is a recognised GREY-out rotation.
 */
static bool mdp_grey_rotate_variant(const struct img_ipi_frameparam *param,
				    u32 *control, enum mdp_wrot_ofst *mode)
{
	const struct img_image_buffer *in, *out;
	s32 rot;
	bool hflip;

	if (param->num_inputs != 1 || param->num_outputs != 1)
		return false;
	in = &param->inputs[0].buffer;
	out = &param->outputs[0].buffer;
	if (!mdp_color_is_plain_nv12(in->format.colorformat) ||
	    out->format.colorformat != MDP_COLOR_GREY)
		return false;
	rot = param->outputs[0].rotation;
	hflip = param->outputs[0].flags & IMG_CTRL_FLAG_HFLIP;
	if (rot == 0 && !hflip)
		return false;			/* not a transform */
	/* output GREY dims = rotated input Y dims */
	if (rot == 90 || rot == 270) {
		if (out->format.width != in->format.height ||
		    out->format.height != in->format.width)
			return false;
	} else if (out->format.width != in->format.width ||
		   out->format.height != in->format.height) {
		return false;
	}
	if (rot == 90  && !hflip) { *control = 0x00105007; *mode = MDP_OFST_ROW_REV;  return true; }
	if (rot == 180 && !hflip) { *control = 0x00205007; *mode = MDP_OFST_FULL_REV; return true; }
	if (rot == 270 && !hflip) { *control = 0x00305007; *mode = MDP_OFST_COL_REV;  return true; }
	if (rot == 0   &&  hflip) { *control = 0x01005007; *mode = MDP_OFST_ROW_REV;  return true; }
	if (rot == 180 &&  hflip) { *control = 0x01205007; *mode = MDP_OFST_COL_REV;  return true; }
	return false;
}

/*
 * Separate-plane NV12->NV12 rotate, NO-SWAP modes (rot180/hflip/vflip): run TWO
 * single-plane WROT passes so the mt8183 WROT never demuxes two planes (the bug).
 * Pass 0 = Y as GREY(Y8) (fmt 0x07); pass 1 = the interleaved-CbCr UV plane as
 * opaque 16bpp (YUYV id 0x05). Returns the offset mode + the Y and UV WROT control
 * words (same rot/flip bits, differing only in the format low byte). rot90/270
 * (dim-swap) are NOT 2-pass here (their UV geometry is a follow-up).
 */
/* The separate-plane 2-pass is gated OFF by default: the Y (GREY) chunk is
 * bit-exact, but the UV chunk (CbCr as opaque 16bpp) STALLS the GCE on mt8183 --
 * there is no opaque-16bpp WROT/RDMA format (YUYV carries 4:2:2 semantics; the
 * RDMA has no precedent for reading a standalone non-NV12 single plane), so the
 * UV-plane rotate is BLOCKED pending format RE (or a de-interleave approach).
 * Set mdp_kd_2pass=1 to re-enable for debugging (will stall NV12 rotate). */
static int mdp_kd_2pass;
module_param(mdp_kd_2pass, int, 0644);
MODULE_PARM_DESC(mdp_kd_2pass, "mt8183 separate-plane NV12 rotate 2-pass (0=off; UV chunk stalls)");

static bool mdp_nv12_rotate_2pass(const struct img_ipi_frameparam *param,
				  enum mdp_wrot_ofst *mode, u32 *yctl, u32 *uvctl)
{
	u32 control;

	if (!mdp_kd_2pass)			/* UV 16bpp chunk stalls -- off by default */
		return false;
	if (!mdp_frame_is_nv12_xform(param))	/* NV12 in/out, recognised variant */
		return false;
	/* no-swap only: rot270 swaps the output dims */
	if (param->outputs[0].buffer.format.width != param->inputs[0].buffer.format.width)
		return false;
	if (!mdp_xform_variant(param, &control, mode))
		return false;
	*yctl  = (control & 0x01300000) | 0x5007;	/* keep rot/flip; GREY (Y8)  */
	*uvctl = (control & 0x01300000) | 0x5005;	/* keep rot/flip; YUYV 16bpp */
	return true;
}

/*
 * NV12 -> I420 rotate gate (de-interleave keystone): in = plain NV12, out = I420
 * (separate Y/U/V planes), no-swap rotation. Returns the offset mode + I420 WROT
 * control (fmt 0x08 | rot/flip). The WROT de-interleaves the chroma into U,V and
 * rotates all three planes in one pass.
 */
static bool mdp_nv12_to_i420_rotate(const struct img_ipi_frameparam *param,
				    enum mdp_wrot_ofst *mode, u32 *i420ctl)
{
	const struct img_image_buffer *in, *out;
	s32 rot;
	bool hflip;

	if (!mdp_kd_2pass)		/* failed keystone: 3-plane rotate swaps too (default off) */
		return false;
	if (param->num_inputs != 1 || param->num_outputs != 1)
		return false;
	in = &param->inputs[0].buffer;
	out = &param->outputs[0].buffer;
	if (!mdp_color_is_plain_nv12(in->format.colorformat) ||
	    out->format.colorformat != MDP_COLOR_I420)
		return false;
	if (out->format.width != in->format.width ||	/* no-swap only */
	    out->format.height != in->format.height)
		return false;
	rot = param->outputs[0].rotation;
	hflip = param->outputs[0].flags & IMG_CTRL_FLAG_HFLIP;
	if (rot == 180 && !hflip) { *i420ctl = 0x00205008; *mode = MDP_OFST_FULL_REV; return true; }
	if (rot == 0   &&  hflip) { *i420ctl = 0x01005008; *mode = MDP_OFST_ROW_REV;  return true; }
	if (rot == 180 &&  hflip) { *i420ctl = 0x01205008; *mode = MDP_OFST_COL_REV;  return true; }
	return false;
}

/*
 * Scratch-backed 5-pass NV12->NV12 rotate (no-swap rot180/hflip/vflip): the only
 * clean chroma path -- de-interleave (non-rotated) -> rot Y/Cb/Cr each as a SINGLE
 * plane (the proven bit-exact GREY primitive) -> re-interleave (non-rotated). All
 * three rotate passes share one GREY control (0x5007 | rot/flip). Gated behind
 * mdp_kd_5pass: 0=off (single-pass fallback), 1=full 5-pass, 2=round-trip test
 * (de-interleave -> re-interleave only, no rotation; output must equal input --
 * validates the scratch DMA + both converts before the rotations).
 */
static int mdp_kd_5pass;
module_param(mdp_kd_5pass, int, 0644);
MODULE_PARM_DESC(mdp_kd_5pass, "mt8183 5-pass NV12 rotate (0=off 1=full 2=roundtrip-test)");

static bool mdp_nv12_rotate_5pass(const struct img_ipi_frameparam *param,
				  enum mdp_wrot_ofst *mode, u32 *gctl)
{
	u32 control;

	if (!mdp_kd_5pass)
		return false;
	if (!mdp_frame_is_nv12_xform(param))		/* NV12 in/out, recognised */
		return false;
	if (param->outputs[0].buffer.format.width !=	/* no-swap only */
	    param->inputs[0].buffer.format.width)
		return false;
	if (!mdp_xform_variant(param, &control, mode))
		return false;
	*gctl = (control & 0x01300000) | 0x5007;	/* GREY rot, shared by Y/Cb/Cr */
	return true;
}

/* Isolation (mdp_kd_5pass==3): NV12 -> I420 forward de-interleave written to the
 * OUTPUT dma-buf (known engine-reachable), no rotation. If the I420 output is a
 * correct de-interleave, the cvt CONFIG is fine and the 5-pass failure is purely
 * the scratch iova not being engine-writable; if wrong, the forward 3-plane I420
 * write config is the bug. */
static bool mdp_nv12_to_i420_deint(const struct img_ipi_frameparam *param)
{
	const struct img_image_buffer *in, *out;

	if (mdp_kd_5pass != 3)
		return false;
	if (param->num_inputs != 1 || param->num_outputs != 1)
		return false;
	in = &param->inputs[0].buffer;
	out = &param->outputs[0].buffer;
	if (!mdp_color_is_plain_nv12(in->format.colorformat) ||
	    out->format.colorformat != MDP_COLOR_I420)
		return false;
	return out->format.width == in->format.width &&
	       out->format.height == in->format.height;
}

/*
 * HYBRID NV12->NV12 rotate (no-swap): HW rotates the Y plane as GREY (bit-exact),
 * the CPU rotates the chroma (the WROT 3-plane I420 write is broken). Returns the
 * Y GREY control + offset mode + the chroma row/col reversal flags (matching the Y
 * rotation). The CPU chroma runs in mdp_m2m_device_run. Gated mdp_kd_hybrid.
 */
static int mdp_kd_hybrid = 1;	/* default ON: bit-exact no-swap NV12 rotate (validated) */
module_param(mdp_kd_hybrid, int, 0644);
MODULE_PARM_DESC(mdp_kd_hybrid, "mt8183 hybrid NV12 rotate: HW Y + CPU chroma (1=on default, 0=off)");

/* Hybrid-rotate size cap (HW-measured on demo-mt8183-chromebook).
 * The HW GREY Y rotate line buffer is bit-exact to 1536 wide and FAILS at 1664;
 * the no-swap Y now TILES horizontally into <=1536-wide column tiles (TILEW) x
 * <=32-tall row strips, a 2D grid spanning multiple GCE chunks, so wider frames
 * work (W_MAX raised 1536 -> 4096). Height already multi-chunk (nv strips). The
 * CPU chroma half is size-agnostic. (Swap rot90/270 uses a separate nsub=1 cap.) */
#define MDP_HYBRID_ROT_TILEW	1536	/* HW GREY Y line-buffer width limit */
#define MDP_HYBRID_ROT_H_MAX	2176
#define MDP_HYBRID_ROT_W_MAX	4096	/* >1536 via horizontal column tiling */

static bool mdp_nv12_rotate_hybrid(const struct img_ipi_frameparam *param,
				   enum mdp_wrot_ofst *mode, u32 *gctl,
				   bool *rev_rows, bool *rev_cols)
{
	u32 control;

	if (!mdp_kd_hybrid)
		return false;
	if (!mdp_frame_is_nv12_xform(param))
		return false;
	/*
	 * NO-SWAP rotations only (rot180 / hflip / vflip). The dim-swap rotations
	 * (rot90 / rot270) belong to the hybrid_swap path. The out.width==in.width
	 * guard below is INSUFFICIENT to exclude them for a SQUARE frame: when W==H the
	 * rot270 output dims (out.width==in.height, out.height==in.width) equal the
	 * input dims numerically, so a square rot270 slips past it. This mattered in the
	 * multi-chunk recompute (mdp_vpu_compute_8183_chunk), which tests this predicate
	 * BEFORE hybrid_swap: for a square rot270 it would recompute chunks 1..n as a
	 * no-swap Y rotate with the wrong dim-swap geometry -> WROT writes a wild IOVA ->
	 * mtk-iommu fault -> all-zero output. Exclude the dim-swap rotations by rotation.
	 */
	if (param->outputs[0].rotation == 90 || param->outputs[0].rotation == 270)
		return false;
	if (param->outputs[0].buffer.format.width !=	/* no-swap only */
	    param->inputs[0].buffer.format.width)
		return false;
	if (!mdp_xform_variant(param, &control, mode))
		return false;
	*gctl = (control & 0x01300000) | 0x5007;	/* GREY Y rotate */
	switch (*mode) {
	case MDP_OFST_ROW_REV:  *rev_rows = false; *rev_cols = true;  break;	/* hflip */
	case MDP_OFST_FULL_REV: *rev_rows = true;  *rev_cols = true;  break;	/* rot180 */
	default:                *rev_rows = true;  *rev_cols = false; break;	/* vflip */
	}
	return true;
}

/* Hybrid rot90/270 (DIM-SWAP): HW rotates the Y plane as GREY; the CPU dim-swaps
 * the chroma (mdp_hybrid_chroma_swap). The swap-Y is WIDTH-TILED: the HW limits
 * the input width (blk_width) to ~768 wide (768 OK; 1024 banded), so tile the
 * input width into <=768 column strips (mdp_config_compute_8183_yrot_swap); the
 * input HEIGHT (= output-width line buffer) is capped per-tile at TILEH; for taller
 * inputs the input height is ALSO tiled into <=TILEH segments (2D grid), so the
 * input width tiles up to W_MAX and the input height up to H_MAX. Returns the GREY Y
 * control + mode + chroma direction (ccw for rot270). Output dims must be the input
 * transposed; rot90/270+flip not handled. Gated mdp_kd_hybrid. */
#define MDP_HYBRID_SWAP_TILEW	768	/* HW swap-Y input-width (blk_width) limit */
#define MDP_HYBRID_SWAP_TILEH	1536	/* per-tile output-width line-buffer limit (= input-height segment) */
#define MDP_HYBRID_SWAP_W_MAX	4096	/* input width, tiled into <=768 strips */
#define MDP_HYBRID_SWAP_H_MAX	4096	/* input height, tiled into <=1536 segments (output-width line buffer) */

/* 2D swap-tile count: (input width / TILEW) column strips x (input height / TILEH)
 * segments. For h<=TILEH this is just the width strips (the validated 1D path). */
static inline u32 mdp_swap_ntiles(u32 w, u32 h)
{
	return DIV_ROUND_UP(w, MDP_HYBRID_SWAP_TILEW) *
	       DIV_ROUND_UP(h, MDP_HYBRID_SWAP_TILEH);
}

static bool mdp_nv12_rotate_hybrid_swap(const struct img_ipi_frameparam *param,
					enum mdp_wrot_ofst *mode, u32 *gctl, bool *ccw)
{
	const struct img_image_buffer *in, *out;
	s32 rot;

	if (!mdp_kd_hybrid)
		return false;
	if (param->num_inputs != 1 || param->num_outputs != 1)
		return false;
	in = &param->inputs[0].buffer;
	out = &param->outputs[0].buffer;
	if (!mdp_color_is_plain_nv12(in->format.colorformat) ||
	    !mdp_color_is_plain_nv12(out->format.colorformat))
		return false;
	if (param->outputs[0].flags & IMG_CTRL_FLAG_HFLIP)
		return false;				/* rot90/270 + flip: not handled */
	rot = param->outputs[0].rotation;
	if (rot == 90)        { *gctl = 0x00105007; *mode = MDP_OFST_ROW_REV; *ccw = false; }
	else if (rot == 270)  { *gctl = 0x00305007; *mode = MDP_OFST_COL_REV; *ccw = true;  }
	else
		return false;
	/* dim-swap: output = input transposed */
	return out->format.width == in->format.height &&
	       out->format.height == in->format.width;
}

/* forward decl: the 2D-tiled swap-Y compute is defined later in this file */
static void mdp_config_compute_8183_yrot_swap(struct mdp_dev *mdp,
					      struct img_ipi_frameparam *param,
					      struct img_config *config,
					      u32 control, enum mdp_wrot_ofst mode,
					      u32 tbase, u32 tcount);

/* Dispatch the rot90/270 hybrid (called from the rotate-90 and xform arms): HW
 * GREY Y (width-tiled swap) + CPU chroma dim-swap. Returns 1 if handled. */
static int mdp_try_hybrid_swap(struct mdp_dev *mdp, struct mdp_vpu_dev *vpu,
			       struct img_ipi_frameparam *param, u32 iw, u32 ih)
{
	enum mdp_wrot_ofst smode;
	u32 sctl;
	bool sccw;

	if (!CFG_CHECK(MT8183, mdp->mdp_data->mdp_plat_id))
		return 0;
	if (!mdp_nv12_rotate_hybrid_swap(param, &smode, &sctl, &sccw))
		return 0;
	if (iw > MDP_HYBRID_SWAP_W_MAX || ih > MDP_HYBRID_SWAP_H_MAX)
		return 0;
	/* <=IMG_MAX_SUBFRAMES_8183 tiles run as ONE GCE config (covers 4K landscape
	 * 4096x2160 = 6x2 = 12). >14 tiles chunk into ceil(ntiles/14) GCE submissions.
	 * ROW_REV (rot90) and COL_REV (rot270) multi-chunk are BOTH bit-exact at ANY
	 * width, including partial-width and square 4096x4096. (The earlier "COL_REV
	 * partial-width >14 corrupts" root cause was WRONG: the true bug was a chunk-
	 * recompute DISPATCH defect -- for a square rot270 (W==H, so out.width==in.width)
	 * mdp_vpu_compute_8183_chunk matched the no-swap mdp_nv12_rotate_hybrid FIRST and
	 * recomputed chunks 1..n as a no-swap Y rotate with the wrong dim-swap geometry ->
	 * WROT IOMMU fault. Fixed by excluding the dim-swap rotations from
	 * mdp_nv12_rotate_hybrid. mdp_kd_swap_colrev_mc stays as a diag knob only.) */
	{
		u32 ntiles = mdp_swap_ntiles(iw, ih);

		vpu->detile_ntiles = ntiles;
		vpu->detile_nchunks = DIV_ROUND_UP(ntiles, IMG_MAX_SUBFRAMES_8183);
	}
	vpu->hybrid_chroma = true;
	vpu->hybrid_swap = true;
	vpu->hybrid_ccw = sccw;
	mdp_config_compute_8183_yrot_swap(mdp, param,
		(struct img_config *)vpu->config, sctl, smode,
		0, min_t(u32, IMG_MAX_SUBFRAMES_8183, mdp_swap_ntiles(iw, ih)));
	dev_dbg(&mdp->pdev->dev,
		"kernel-direct HYBRID rot90/270 (mt8183 %ux%u, HW Y swap + CPU chroma dim-swap)\n",
		iw, ih);
	return 1;
}

/*
 * Kernel-direct NV12 -> <fmt> format-conversion gate: plain NV12 in, 1:1, no
 * transform, full-frame, and the OUTPUT is one of the mdp_fmtconv_tbl formats
 * (NV21/I420/YV12/GREY/NV16/NV61/YUYV/YVYU/UYVY/VYUY/RGB565/RGB565X/BGR24/ARGB32/
 * ABGR32). NV12 (blit), RGB24 (CSC) and the transforms have their own gates.
 */
static bool mdp_frame_is_nv12_fmtconv(const struct img_ipi_frameparam *param)
{
	const struct img_image_buffer *in, *out;

	if (param->num_inputs != 1 || param->num_outputs != 1)
		return false;
	in = &param->inputs[0].buffer;
	out = &param->outputs[0].buffer;

	/*
	 * 3-plane planar dst (I420/YV12) is the WROT write wall for the pure fmtconv
	 * path -- it is handled by the I420/YV12-dst HW-NV12 + CPU-deinterleave hybrid
	 * arm (mdp_frame_is_i420dst_hybrid). Reject it here so an NV12/NV21 source
	 * (whose branches below do NOT check plane count, and I420/YV12 ARE rows in
	 * mdp_fmtconv_tbl) is not claimed by the fmtconv arm -- which then bails to the
	 * dead SCP (-22) -- but instead falls through to the hybrid arm. The other
	 * source branches (NV16/packed/I420/RGB) already reject 3-plane; this makes it
	 * uniform. Note: the hybrid arm reconfigures its dst to NV12 (2-plane) BEFORE
	 * re-calling this predicate, so its own convertibility re-check is unaffected.
	 */
	if (MDP_COLOR_GET_PLANE_COUNT(out->format.colorformat) == 3)
		return false;

	/* mt8183 swapped-src -> NV21/NM21 (mdp_kd_8183_swapdst): swap_twin(NV21)==
	 * NV12 is not a table row, so the normal per-branch lookup returns NULL and
	 * the pair fell to SCP. Accept it up front (the arm supplies the local NV12
	 * row for mt8183; a non-mt8183 SoC finds f==NULL in the arm and rides SCP).
	 * Tight: only a plain-NV21/NM21 dst from a SWAPPED YUV source whose
	 * swapped-src->NV12 sibling already converts (so the decimate is proven). */
	if (mdp_kd_8183_swapdst &&
	    mdp_color_is_plain_nv21(out->format.colorformat) &&
	    ((mdp_kd_src_nv21 && mdp_color_is_plain_nv21(in->format.colorformat)) ||
	     (mdp_kd_src_nv16 && mdp_color_is_plain_nv16src(in->format.colorformat) &&
	      MDP_COLOR_IS_SWAPPED(in->format.colorformat) && mdp_kd_dn420) ||
	     (mdp_kd_src_yuyv && mdp_color_is_packed422src(in->format.colorformat) &&
	      MDP_COLOR_IS_SWAPPED(in->format.colorformat) && mdp_kd_dn420) ||
	     (mdp_kd_src_nv24 && mdp_color_is_plain_nv24src(in->format.colorformat) &&
	      MDP_COLOR_IS_SWAPPED(in->format.colorformat))))
		return true;

	if (mdp_kd_src_nv21 && mdp_color_is_plain_nv21(in->format.colorformat)) {
		/* NV21 source: read identically to NV12, chroma-swap compensated by
		 * the twin row's WROT control. RGB-from-NV21 is deferred (it needs a
		 * CSC U/V-swap not done here), so require a YUV (non-rgb) twin row. */
		const struct mdp_fmtconv *f =
			mdp_fmtconv_lookup(mdp_chroma_swap_twin(out->format.colorformat));

		if (!f)
			return false;
		/* RGB dst: swap_twin(RGB)==RGB so f is the plain RGB row. Allowed only
		 * via mdp_kd_swap2rgb (un-swap on the read + WROT YUV->RGB CSC). */
		if (f->rgb && !mdp_kd_swap2rgb)
			return false;
	} else if (mdp_kd_src_nv16 &&
		   mdp_color_is_plain_nv16src(in->format.colorformat)) {
		/* NV16/NV61 source (2-plane 4:2:2): read with the new 4:2:2 RDMA
		 * geometry (full-height chroma). This increment covers only the
		 * NO-vertical-resample YUV outputs -- 4:2:2 (NV16/NV61/YUYV/YVYU/UYVY/
		 * VYUY) and GREY (all V_SUBSAMPLE=0). 4:2:0 outputs (NV12/NV21/I420/
		 * YV12) need a 4:2:2->4:2:0 vertical decimate (deferred); RGB via the
		 * WROT YUV->RGB CSC is un-gated by mdp_kd_yuv2rgb. */
		const struct mdp_fmtconv *f = mdp_fmtconv_for(param);

		if (!f)
			return false;
		if (f->rgb) {
			/* RGB dst: WROT mat_ctrl=0x61 CSC. NON-swapped (NV16) source
			 * via mdp_kd_yuv2rgb; SWAPPED (NV61) source via mdp_kd_swap2rgb
			 * (un-swap on the read so the CSC sees true U/V). */
			if (MDP_COLOR_IS_SWAPPED(in->format.colorformat)) {
				if (!mdp_kd_swap2rgb)
					return false;
			} else if (!mdp_kd_yuv2rgb) {
				return false;
			}
		} else if (MDP_COLOR_GET_V_SUBSAMPLE(out->format.colorformat)) {
			/* 4:2:0 dst from a 4:2:2 source: WROT 4:2:2->4:2:0 vertical
			 * decimate (uv_ysel=0), un-gated by mdp_kd_dn420. 2-plane
			 * (NV12/NV21/NM12/NM21) only; 3-plane I420/YV12 = WROT wall. */
			if (!mdp_kd_dn420)
				return false;
			if (MDP_COLOR_GET_PLANE_COUNT(out->format.colorformat) == 3)
				return false;
		}
	} else if (mdp_kd_src_nv24 &&
		   mdp_color_is_plain_nv24src(in->format.colorformat)) {
		/* NV24/NV42 source (2-plane 4:4:4): HW-native full-res-chroma RDMA read
		 * (src_ctrl 0x060e). The pipe carries 4:4:4 (like RGB-after-CSC), so the
		 * WROT decimates to ANY target sampling with its plain (target-keyed)
		 * control word -- 4:2:0 (NV12/NV21, uv_xsel=0/uv_ysel=0), 4:2:2 (NV16/NV61/
		 * packed, uv_xsel=0/uv_ysel=2), GREY (Y-only) all reachable with NO extra
		 * gating (unlike a 4:2:2 source, no dn420 vertical-decimate is needed -- the
		 * same control also decimates H). RGB via the WROT YUV->RGB CSC: NON-swapped
		 * (NV24) via mdp_kd_yuv2rgb, SWAPPED (NV42) via mdp_kd_swap2rgb (un-swap on
		 * the read). 3-plane I420/YV12 dst is rejected at the top (the hybrid arm). */
		const struct mdp_fmtconv *f = mdp_fmtconv_for(param);

		if (!f)
			return false;
		if (f->rgb) {
			if (MDP_COLOR_IS_SWAPPED(in->format.colorformat)) {
				if (!mdp_kd_swap2rgb)
					return false;
			} else if (!mdp_kd_yuv2rgb) {
				return false;
			}
		}
	} else if (mdp_kd_src_yuyv &&
		   mdp_color_is_packed422src(in->format.colorformat)) {
		/* Packed 4:2:2 source (YUYV/YVYU/UYVY/VYUY): single-plane interleaved
		 * read. Same NO-vertical-resample YUV output set as NV16 -- 4:2:2
		 * (NV16/NV61/YUYV/YVYU/UYVY/VYUY) and GREY (all V_SUBSAMPLE=0). 4:2:0
		 * outputs (NV12/NV21/I420/YV12) need a 4:2:2->4:2:0 vertical decimate
		 * (deferred); RGB via the WROT YUV->RGB CSC is un-gated by mdp_kd_yuv2rgb. */
		const struct mdp_fmtconv *f = mdp_fmtconv_for(param);

		if (!f)
			return false;
		if (f->rgb) {
			/* RGB dst: WROT mat_ctrl=0x61 CSC. NON-swapped (YUYV/UYVY)
			 * via mdp_kd_yuv2rgb; SWAPPED (YVYU/VYUY) via mdp_kd_swap2rgb
			 * (un-swap on the read so the CSC sees true U/V). */
			if (MDP_COLOR_IS_SWAPPED(in->format.colorformat)) {
				if (!mdp_kd_swap2rgb)
					return false;
			} else if (!mdp_kd_yuv2rgb) {
				return false;
			}
		} else if (MDP_COLOR_GET_V_SUBSAMPLE(out->format.colorformat)) {
			/* 4:2:0 dst from a packed 4:2:2 source: WROT 4:2:2->4:2:0
			 * vertical decimate (uv_ysel=0), un-gated by mdp_kd_dn420.
			 * 2-plane (NV12/NV21) only; 3-plane I420/YV12 = WROT wall. */
			if (!mdp_kd_dn420)
				return false;
			if (MDP_COLOR_GET_PLANE_COUNT(out->format.colorformat) == 3)
				return false;
		}
	} else if (mdp_kd_src_i420 &&
		   mdp_color_is_plain_i420src(in->format.colorformat)) {
		/* I420/YV12 source (3-plane 4:2:0): read all 3 planes (YV12 normalized
		 * by the iova[1]<->iova[2] swap in the compute). This increment covers
		 * only the 4:2:0 YUV outputs (NV12/NV21) + GREY (no vertical resample).
		 * 4:2:2 outputs need a 4:2:0->4:2:2 upsample (deferred); RGB via the WROT
		 * YUV->RGB CSC is un-gated by mdp_kd_yuv2rgb; I420/YV12 dst is the 3-plane
		 * WROT WRITE wall. */
		u32 ocf = out->format.colorformat;
		const struct mdp_fmtconv *f = mdp_fmtconv_for(param);

		if (!f)
			return false;
		if (f->rgb) {
			/* RGB dst: WROT mat_ctrl=0x61 CSC. Both I420 AND YV12 are
			 * normalized to true YUV by the 3-plane pointer swap in the
			 * compute (no swap-twin), so RGB is correct for both. 4:2:0->RGB
			 * is the same sampling as the NV12->RGB path. */
			if (!mdp_kd_yuv2rgb)
				return false;
		} else if (ocf != MDP_COLOR_GREY) {
			if (MDP_COLOR_GET_PLANE_COUNT(ocf) == 3)
				return false;	/* I420/YV12 dst -- 3-plane WROT wall */
			/* 4:2:2 dst: the 3-plane read normalizes to internal 4:2:0, so
			 * the WROT 4:2:0->4:2:2 upsample (as for NV12->NV16) applies.
			 * Un-gated by mdp_kd_up422. */
			if (!MDP_COLOR_GET_V_SUBSAMPLE(ocf) && !mdp_kd_up422)
				return false;	/* 4:2:2 dst -- needs upsample (deferred) */
		}
	} else if (mdp_kd_ym16src &&
		   mdp_color_is_ym16src(in->format.colorformat)) {
		/* YM16/YM61 (3-plane 4:2:2) source (SCP-tail class #2): 3-plane read
		 * (full-height chroma, src_ctrl 0x609/0x4609). The WROT resamples to the
		 * dst sampling exactly like an NV16 source -- 4:2:2 dst (NV16/NV61/packed/
		 * NM16/NM61) no resample, 4:2:0 dst (NV12/NV21/NM12/NM21) via the WROT
		 * vertical decimate (mdp_kd_dn420), GREY Y-only, RGB via the WROT YUV->RGB
		 * CSC. YM61 is UN-swapped at the RDMA (0x4609), so RGB uses the plain
		 * yuv2rgb CSC (not swap2rgb). 3-plane dst (I420/YV12/YM16/YM61/...) is the
		 * WROT WRITE wall -- deferred (stays SCP). */
		const struct mdp_fmtconv *f = mdp_fmtconv_for(param);

		if (!f)
			return false;
		if (MDP_COLOR_GET_PLANE_COUNT(out->format.colorformat) == 3)
			return false;	/* 3-plane dst -- WROT write wall (deferred) */
		if (f->rgb) {
			if (!mdp_kd_yuv2rgb)
				return false;
		} else if (MDP_COLOR_GET_V_SUBSAMPLE(out->format.colorformat)) {
			if (!mdp_kd_dn420)	/* 4:2:2 -> 4:2:0 vertical decimate */
				return false;
		}
	} else if (mdp_kd_src_rgb &&
		   mdp_color_is_rgb_src(in->format.colorformat)) {
		/* RGB source -> YUV: RDMA RGB->YUV CSC (rdma.transform) + plain YUV WROT
		 * write. Admits the YUV (non-rgb) outputs: GREY, NV12/NV21 (4:2:0, WROT
		 * chroma downsample), NV16/NV61 + packed 4:2:2 (YUYV/YVYU/UYVY/VYUY). RGB
		 * dst (RGB->RGB, not a CSC) is rejected by the f->rgb test; I420/YV12 dst
		 * is the 3-plane WROT write wall. */
		u32 ocf = out->format.colorformat;
		const struct mdp_fmtconv *f;

		if (MDP_COLOR_GET_PLANE_COUNT(ocf) == 3)
			return false;	/* I420/YV12 dst -- 3-plane WROT wall */
		f = mdp_fmtconv_for(param);
		if (!f)
			return false;
		/* RGB dst (RGB->RGB repack, no CSC) is un-gated by mdp_kd_rgb2rgb;
		 * otherwise RGB source only targets YUV dsts. */
		if (f->rgb && !mdp_kd_rgb2rgb)
			return false;
	} else if (mdp_kd_mm21conv &&
		   mdp_color_is_mm21src(in->format.colorformat)) {
		/* MM21/MT21 block-mode 4:2:0 source (the mtk-vcodec decoder CAPTURE
		 * format) -> the X dst set. Read via the detile block-mode RDMA
		 * geometry (mdp_config_compute_8183_fmtconv srcmm21 branch); the WROT
		 * writes X exactly as for an NV12 source (MM21 is 4:2:0 like NV12, so
		 * mdp_fmtconv_for -> lookup(out) supplies the right WROT control). The
		 * combine of two already-validated computes -- detile read + fmtconv
		 * write -- in one pass. Excludes plain NV12 out (that is the pure detile
		 * arm) and 3-plane I420/YV12 dst (the 3-plane WROT wall; deferred). */
		u32 ocf = out->format.colorformat;
		const struct mdp_fmtconv *f;

		if (mdp_color_is_plain_nv12(ocf))
			return false;	/* MM21->NV12 is the detile arm */
		if (MDP_COLOR_GET_PLANE_COUNT(ocf) == 3)
			return false;	/* I420/YV12 dst -- 3-plane WROT wall (deferred) */
		f = mdp_fmtconv_for(param);
		if (!f)
			return false;
		if (f->rgb && !mdp_kd_yuv2rgb)	/* RGB dst via WROT YUV->RGB CSC */
			return false;
		/* MM21 geometry (detile-faithful): tiled luma stride ALIGN(w,64)*32,
		 * block-padded input height (h..h+63), input width w or ALIGN(w,64);
		 * no scale. The generic 1:1 check below is skipped for MM21. */
		if (in->format.width != out->format.width &&
		    in->format.width != ALIGN(out->format.width, 64))
			return false;
		if (in->format.height < out->format.height ||
		    in->format.height - out->format.height >= 64)
			return false;
		if (in->format.plane_fmt[0].stride != ALIGN(out->format.width, 64) * 32)
			return false;
	} else if (mdp_color_is_plain_nv12(in->format.colorformat)) {
		if (!mdp_fmtconv_lookup(out->format.colorformat))
			return false;
	} else {
		return false;
	}
	/* 1:1 -- no scale (an MM21 source uses tiled/block-padded dims, validated in
	 * its branch above; skip the linear equality check for it). */
	if (!(mdp_kd_mm21conv && mdp_color_is_mm21src(in->format.colorformat)) &&
	    (in->format.width != out->format.width ||
	     in->format.height != out->format.height))
		return false;
	if (param->outputs[0].rotation ||
	    (param->outputs[0].flags & IMG_CTRL_FLAG_HFLIP))
		return false;
	/* full-frame output only (block-mode source: crop carries padded input dims) */
	if (!mdp_crop_is_fullframe(param))
		return false;
	return true;	/* size bounds applied at the call site */
}

/*
 * GREY (Y8, luma-only) SOURCE shape gate (mdp_kd_src_grey). Structural
 * preconditions only -- the dst reachability is decided by whether the normal
 * NV12-source dispatch (after the source is reconfigured to NV12 @ scratch_b in
 * mdp_vpu_process) claims the frame; an unreachable dst simply falls to the SCP
 * (-22) with no harm. Requires GREY in, 1:1 (no scale), no rotate/flip, full-frame.
 */
static bool mdp_frame_is_grey_src(const struct img_ipi_frameparam *param)
{
	const struct img_image_buffer *in, *out;

	if (param->num_inputs != 1 || param->num_outputs != 1)
		return false;
	in = &param->inputs[0].buffer;
	out = &param->outputs[0].buffer;

	if (!mdp_color_is_grey_src(in->format.colorformat))
		return false;
	if (in->format.width != out->format.width ||
	    in->format.height != out->format.height)
		return false;
	if (param->outputs[0].rotation ||
	    (param->outputs[0].flags & IMG_CTRL_FLAG_HFLIP))
		return false;
	if (param->outputs[0].crop.left || param->outputs[0].crop.top ||
	    param->outputs[0].crop.width != out->format.width ||
	    param->outputs[0].crop.height != out->format.height)
		return false;
	return true;
}

/*
 * I420/YV12-DST HYBRID shape gate (mdp_kd_i420dst_hybrid). Structural preconditions
 * only -- the source-convertibility (that the fmtconv path can produce NV12 from this
 * source) is re-checked in the dispatch arm by reconfiguring the dst to NV12 and
 * calling mdp_frame_is_nv12_fmtconv(). Requires:
 *  - dst = I420 or YV12 (3-plane planar 4:2:0);
 *  - a CONTIGUOUS single-buffer dst (iova[1]/iova[2] are Y+U/Y+U+V offsets into one
 *    dma-buf), so the HW-written 2-plane NV12 (Y + w*h/2 interleaved UV) exactly fills
 *    the Y+U+V region and the CPU can de-interleave in place. Multi-planar YM12/YM21
 *    (separate U/V dma-bufs, each only w*h/4) FAIL this and route to the SCP;
 *  - 1:1 (no scale), no rotate/flip, full-frame (no crop sub-region).
 */
static bool mdp_frame_is_i420dst_hybrid(const struct img_ipi_frameparam *param)
{
	const struct img_image_buffer *in, *out;
	u32 w, h, ostride;

	if (param->num_inputs != 1 || param->num_outputs != 1)
		return false;
	in = &param->inputs[0].buffer;
	out = &param->outputs[0].buffer;
	w = out->format.width;
	h = out->format.height;

	if (out->format.colorformat != MDP_COLOR_I420 &&
	    out->format.colorformat != MDP_COLOR_YV12)
		return false;
	if (MDP_COLOR_GET_PLANE_COUNT(out->format.colorformat) != 3)
		return false;
	ostride = out->format.plane_fmt[0].stride;
	/* contiguous single-buffer: U plane right after Y, V plane right after U */
	if (out->iova[1] != out->iova[0] + ostride * h ||
	    out->iova[2] != out->iova[1] + (ostride / 2) * (h / 2))
		return false;
	if (mdp_kd_8183_ident && mdp_kd_mm21conv &&
	    mdp_color_is_mm21src(in->format.colorformat)) {
		/* MM21/MT21 tiled source (decoder path): the source uses the
		 * block-padded/tiled dims, exactly as the mp gate + mm21conv fmtconv
		 * gate. The HW detile-reads MM21 and writes NV12 into the contiguous
		 * dst buffer; the CPU de-interleaves the UV -> U/V in place. */
		if (in->format.width != w && in->format.width != ALIGN(w, 64))
			return false;
		if (in->format.height < h || in->format.height - h >= 64)
			return false;
		if (in->format.plane_fmt[0].stride != ALIGN(w, 64) * 32)
			return false;
	} else {
		/* 1:1 -- no scale */
		if (in->format.width != w || in->format.height != h)
			return false;
	}
	if (param->outputs[0].rotation ||
	    (param->outputs[0].flags & IMG_CTRL_FLAG_HFLIP))
		return false;
	/* full-frame output only (block-mode source: crop carries padded input dims) */
	if (!mdp_crop_is_fullframe(param))
		return false;
	return true;
}

/*
 * MULTI-PLANAR I420/YV12-DST HYBRID shape gate (mdp_kd_i420dst_mp). Same as
 * mdp_frame_is_i420dst_hybrid EXCEPT the dst is NOT a contiguous single buffer:
 * the U and V planes are SEPARATE dma-bufs (V4L2 YUV420M/YVU420M -- YM12/YM21).
 * The single-buffer hybrid rejects these (its contiguity check fails); here we
 * positively require the NON-contiguous layout so the HW writes the interleaved
 * UV to a scratch buffer and the CPU de-interleaves to the two separate planes.
 * Requires: dst = I420/YV12 (3-plane), separate U/V dma-bufs, 1:1, no
 * rotate/flip, full-frame.
 */
static bool mdp_frame_is_i420dst_mp(const struct img_ipi_frameparam *param)
{
	const struct img_image_buffer *in, *out;
	u32 w, h, ostride;

	if (param->num_inputs != 1 || param->num_outputs != 1)
		return false;
	in = &param->inputs[0].buffer;
	out = &param->outputs[0].buffer;
	w = out->format.width;
	h = out->format.height;

	if (out->format.colorformat != MDP_COLOR_I420 &&
	    out->format.colorformat != MDP_COLOR_YV12)
		return false;
	if (MDP_COLOR_GET_PLANE_COUNT(out->format.colorformat) != 3)
		return false;
	ostride = out->format.plane_fmt[0].stride;
	/* MULTI-PLANAR: the U/V planes are SEPARATE dma-bufs -- i.e. NOT the
	 * contiguous single-buffer layout the mdp_frame_is_i420dst_hybrid arm
	 * handles. If the layout IS contiguous, let that arm take it. */
	if (out->iova[1] == out->iova[0] + ostride * h &&
	    out->iova[2] == out->iova[1] + (ostride / 2) * (h / 2))
		return false;
	if (param->outputs[0].rotation ||
	    (param->outputs[0].flags & IMG_CTRL_FLAG_HFLIP))
		return false;
	/* full-frame output only (block-mode source: crop carries padded input dims) */
	if (!mdp_crop_is_fullframe(param))
		return false;
	if (mdp_kd_mm21conv && mdp_color_is_mm21src(in->format.colorformat)) {
		/* MM21/MT21 tiled source (the decoder path): the source uses the
		 * block-padded/tiled dims, exactly as the mm21conv fmtconv gate. The
		 * HW detile-reads MM21 and writes NV12 to scratch; the CPU splits. */
		if (in->format.width != w && in->format.width != ALIGN(w, 64))
			return false;
		if (in->format.height < h || in->format.height - h >= 64)
			return false;
		if (in->format.plane_fmt[0].stride != ALIGN(w, 64) * 32)
			return false;
	} else {
		/* 1:1 -- no scale */
		if (in->format.width != w || in->format.height != h)
			return false;
	}
	return true;
}

/*
 * 3-plane 4:2:2-DST HYBRID shape gate (mdp_kd_ym16dst8195, SCP-tail class #2).
 * dst = YM16 (I422) or YM61 (YV16): 3-plane planar 4:2:2 (full-height, half-width
 * chroma), multi-planar (separate U/V dma-bufs). The source-convertibility (that
 * the fmtconv path produces NV16 from this source) is re-checked in the dispatch
 * arm by reconfiguring the dst to NV16 and calling mdp_frame_is_nv12_fmtconv().
 * Requires: 1:1 (no scale), no rotate/flip, full-frame, even dims (4:2:2 chroma).
 * MM21/MT21 admitted: dispatch uses compute_8195_fmtconv srcmm21 (MM21>NV16 KD)
 * then ym16_deint. Excluding them sent G5 MM21>YM16/YM61 to SCP → GCE timeout.
 */
static bool mdp_frame_is_ym16dst(const struct img_ipi_frameparam *param)
{
	const struct img_image_buffer *in, *out;
	u32 w, h;

	if (param->num_inputs != 1 || param->num_outputs != 1)
		return false;
	in = &param->inputs[0].buffer;
	out = &param->outputs[0].buffer;
	w = out->format.width;
	h = out->format.height;

	if (!mdp_color_is_ym16dst(out->format.colorformat))
		return false;
	if (MDP_COLOR_GET_PLANE_COUNT(out->format.colorformat) != 3)
		return false;
	if (w & 1 || h & 1)
		return false;		/* 4:2:2 chroma needs even dims */
	/* 1:1 -- no scale */
	if (in->format.width != w || in->format.height != h)
		return false;
	if (param->outputs[0].rotation ||
	    (param->outputs[0].flags & IMG_CTRL_FLAG_HFLIP))
		return false;
	/* full-frame output only (block-mode source: crop carries padded input dims) */
	if (!mdp_crop_is_fullframe(param))
		return false;
	return true;
}

/*
 * Kernel-direct NV12 -> NV12 scale gate: plain NV12 in/out at DIFFERENT dims (a
 * resize), no transform, full-frame. The RSZ resizes iw x ih -> ow x oh (the
 * output-queue crop is in SOURCE/input dims, see rotate-90). 1:1 is blit's job.
 */
static bool mdp_frame_is_nv12_scale(const struct img_ipi_frameparam *param)
{
	const struct img_image_buffer *in, *out;

	if (param->num_inputs != 1 || param->num_outputs != 1)
		return false;
	in = &param->inputs[0].buffer;
	out = &param->outputs[0].buffer;

	if (!mdp_color_is_plain_nv12(in->format.colorformat) ||
	    !mdp_color_is_plain_nv12(out->format.colorformat))
		return false;
	/* must be a scale (1:1 -> blit); RSZ needs in/out >= 2 (coeff_step / (n-1)) */
	if (in->format.width == out->format.width &&
	    in->format.height == out->format.height)
		return false;
	if (in->format.width < 2 || in->format.height < 2 ||
	    out->format.width < 2 || out->format.height < 2)
		return false;
	if (param->outputs[0].rotation ||
	    (param->outputs[0].flags & IMG_CTRL_FLAG_HFLIP))
		return false;
	/* full-frame source (output-queue crop is in INPUT dims) */
	if (param->outputs[0].crop.left || param->outputs[0].crop.top ||
	    param->outputs[0].crop.width != in->format.width ||
	    param->outputs[0].crop.height != in->format.height)
		return false;
	return true;	/* size bounds applied at the call site */
}

/*
 * Kernel-direct 1:1 CROP gate (mt8195). A sub-region extraction set via
 * VIDIOC_S_SELECTION (output-queue CROP) where the crop W/H EQUAL the output W/H
 * (1:1, no resize) -- the SCP routes it through the RSZ as a cubic resample of the
 * crop rect (mdp_config_compute_8195_crop reproduces it bit-exact). Admit only:
 *  - NV12->NV12, both plain (exact, like the scale gate)
 *  - a real sub-region (crop != full frame) -- a full-frame crop is blit's job
 *  - crop W==out W && crop H==out H (1:1; crop+scale stays on the SCP for now)
 *  - the crop origin off (0,0) -- the SCP BYPASSES the RSZ for an origin crop
 *    (byp=1), a different config; route origin crops to the SCP
 *  - margins inside the frame: the RSZ window backs cl off by 12 (RDMA) / the
 *    crop right is the frame-relative far edge, so require cl>=12, ct>=4 and the
 *    far edge within the frame (always true). crop W small enough for one RSZ
 *    tile is enforced at the call site.
 * mdp_try_crop already even-aligns the rect (4:2:0), so cl/ct/cw/ch are even.
 */
static bool mdp_frame_is_nv12_crop(const struct img_ipi_frameparam *param)
{
	const struct img_image_buffer *in, *out;
	const struct img_crop *c;

	if (param->num_inputs != 1 || param->num_outputs != 1)
		return false;
	in = &param->inputs[0].buffer;
	out = &param->outputs[0].buffer;
	c = &param->outputs[0].crop;

	if (!mdp_color_is_plain_nv12(in->format.colorformat) ||
	    !mdp_color_is_plain_nv12(out->format.colorformat))
		return false;
	if (param->outputs[0].rotation ||
	    (param->outputs[0].flags & IMG_CTRL_FLAG_HFLIP))
		return false;
	/* A real off-origin sub-region (full-frame at origin -> blit/plain-scale; an
	 * origin crop -> SCP RSZ-bypass). crop W/H == out W/H is the 1:1 case
	 * (mdp_kd_crop); crop W/H != out W/H is crop+scale (mdp_kd_cropscale) -- the
	 * call site picks which flag gates + applies the per-ratio margin/tile bound. */
	if (c->width == in->format.width && c->height == in->format.height)
		return false;
	/* Two arms: an ORIGIN crop (cl==ct==0) -- the SCP bypasses the back-tap and
	 * emits a plain windowed cw x ch scale (mdp_config_compute_8195_crop_origin) --
	 * OR an OFF-ORIGIN crop (cl>=12, ct>=4 so the RDMA/RSZ back-margins fit inside
	 * the frame, mdp_config_compute_8195_crop). A partial-origin crop (exactly one
	 * of cl/ct zero) or an in-between margin (0<cl<12 / 0<ct<4) falls through to the
	 * SCP. The exact per-ratio off-origin back-margin (downscale widens it) is
	 * re-checked at the call site; this is the floor (rdma_l=cl-12). */
	if (c->left || c->top) {
		if (c->left < 12 || c->top < 4)
			return false;
	}
	if (c->left + c->width > in->format.width ||
	    c->top + c->height > in->format.height)
		return false;
	/* even-aligned (4:2:0) -- mdp_try_crop guarantees this; assert defensively */
	if ((c->left | c->top | c->width | c->height) & 1)
		return false;
	return true;	/* single-RSZ-tile size bound applied at the call site */
}

static void mdp_config_compute_8195(struct img_ipi_frameparam *param,
				    struct img_config *config)
{
	struct img_config_8195 *cfg = &config->config_8195;
	const struct img_image_buffer *in = &param->inputs[0].buffer;
	const struct img_image_buffer *out = &param->outputs[0].buffer;
	u32 w = out->format.width;
	u32 h = out->format.height;
	u32 inh = in->format.height;		/* block-aligned by mdp_set_src_config */
	u32 ostride = out->format.plane_fmt[0].stride;	/* output NV12 row stride */
	u32 iy = in->iova[0], ic = in->iova[1];
	u32 oy = out->iova[0], oc = out->iova[1];
	u32 hw = (h << 16) | w, wh = (w << 16) | h;
	u32 croph = ALIGN(h, 32);
	u32 aw = ALIGN(w, 64);		/* MM21 read-side aligned width (luma stride = aw*32) */
	struct img_mmsys_ctrl *mc;
	int i;

	memset(cfg, 0, sizeof(*cfg));
	cfg->num_components = 10;
	cfg->num_subfrms = 1;

	for (i = 0; i < 10; i++) {
		struct img_compparam_8195 *p = &cfg->components[i];
		struct img_comp_frame_8195 *f = &p->frame;
		struct img_comp_subfrm_8195 *s = &p->subfrms[0];
		u32 *u = (u32 *)&p->rdma;	/* engine union overlay (validated) */

		p->type = mdp_dt_comp[i].type;
		p->id = mdp_dt_comp[i].id;
		p->input = 0;
		p->num_outputs = 1;
		p->outputs[0] = 0;
		p->num_subfrms = 1;

		/* common frame + subfrm; overridden per component below */
		f->in_width = w; f->in_height = h;
		f->out_width = w; f->out_height = h;
		f->in_total_width = w; f->out_total_width = w;
		f->crop.width = w; f->crop.height = h;
		s->in.right = w - 1; s->in.bottom = h - 1;
		s->out.right = w - 1; s->out.bottom = h - 1;

		switch (i) {
		case 0:	/* RDMA: reads MM21 (inh tall), crops to ALIGN(h,32) */
			f->in_height = inh;
			f->crop.height = croph;
			s->in.bottom = croph - 1;
			u[0] = 0x860c; u[2] = 0x10; u[3] = iy; u[4] = ic;
			u[6] = iy + aw * inh; u[7] = ic + aw * inh / 2;	/* iova_end = base + plane size (aligned) */
			u[9] = aw * 32; u[11] = aw * 16;		/* mf/sf_bkgd = luma/chroma tile-row stride */
			u[15] = 0x03000020; u[16] = 0xe800; u[17] = 0x9c00;
			u[18] = 0x01000010; u[19] = 0x7400; u[20] = 0x4c00;
			/* subfrms[0]: src=(crop_h<<16)|w, clip=(h<<16)|w */
			u[29] = (croph << 16) | w; u[30] = hw;
			break;
		case 1:	/* RSZ: 1:1 (vertical inh->h is c0 crop, not scale) */
			f->in_height = inh;
			u[0] = 1; u[1] = 7; u[2] = (w << 16); u[3] = (h << 16);
			break;
		case 2:	/* relay: no geometry, no register config */
			f->in_height = inh;
			f->in_total_width = 0; f->out_total_width = 0;
			f->crop.width = 0; f->crop.height = 0;
			memset(&s->in, 0, sizeof(s->in));
			memset(&s->out, 0, sizeof(s->out));
			break;
		case 3:	f->crop.width = 0; f->crop.height = 0;
			u[0] = 0x10000000; u[1] = 1; u[3] = hw;
			u[4] = (w - 1) << 16; u[5] = (h - 1) << 16;
			u[7] = w - 1; u[8] = 0x60; u[9] = 0x200;
			break;
		case 4:	f->crop.width = 0; f->crop.height = 0;
			u[0] = 0x80; u[1] = 1; u[2] = wh; u[3] = wh;
			break;
		case 5:	f->bypass = 1;
			u[15] = 0x1000; u[16] = hw; u[17] = hw; u[26] = hw;
			break;
		case 6:	f->bypass = 1;
			u[0] = 1; u[1] = wh; u[2] = wh;
			u[4] = (w - 1) << 16; u[5] = (h - 1) << 16;
			break;
		case 7:	f->crop.width = 0; f->crop.height = 0;
			u[0] = 3; u[1] = w; u[2] = h;
			break;
		case 8:	u[0] = ((h - 1) << 16) | w;
			break;
		case 9:	/* WROT: writes linear NV12 (stride[] = output row stride) */
			u[0] = oy; u[1] = oc; u[3] = 0x500c;
			u[4] = ostride; u[5] = ostride; u[6] = ostride;
			u[8] = 0x400; u[10] = 0x000d80c4;
			u[21] = hw; u[22] = hw; u[24] = (w << 16) | 0x4000;
			break;
		}
	}

	/* ctrls[0]: SVPP2 mmsys mux routing (geometry-independent) */
	mc = &cfg->ctrls[0];
	mc->num_sets = 7;
	for (i = 0; i < 7; i++) {
		mc->sets[i].reg = mdp_dt_mux[i].reg;
		mc->sets[i].value = mdp_dt_mux[i].value;
		mc->sets[i].subsys_id = mdp_dt_mux[i].subsys_id;
	}
}

/*
 * Kernel-direct CSC (NV12 -> RGB24) config-compute for mt8195 -- a first
 * HW-validation DRAFT, cloned from mdp_config_compute_8195 (the detile).
 *
 * Same single-SVPP2 pixel-pipe (10 components RDMA->RSZ->relay->PQ chain->WROT,
 * mdp_dt_comp[] / mdp_dt_mux[]) and the SAME parametric per-component geometry
 * formula (validated bit-exact for the detile across 1280x720 / 640x360 /
 * 1280x360). The CSC differs from the detile in exactly TWO engines, per the
  * WORKING v23042 SCP oracle (the internal CSC oracle record + its captured
  * RGB24 dump):
 *
 *  - RDMA (c0) reads LINEAR NV12 instead of MM21 block-mode:
 *      src_ctrl  0x860c -> 0x060c   (clear block-mode bit15)
 *      control   0x10   -> 0        (no block-mode simple_mode)
 *      iova_end  = base + istride*h (Y) / base + istride*h/2 (chroma) -- linear
 *      mf_bkgd = sf_bkgd = istride   (linear Y pitch; NV12 interleaved CbCr pitch
 *                                     == Y pitch), NOT the MM21 aw*32 / aw*16.
 *    The 0xe8/0x9c... dmabuf-con words (u[15..20]) are RDMA ultra/dmabuf throttle
 *    config; kept identical to the detile (the oracle c0 carries the same family).
 *  - WROT (c9) writes RGB24 (1 plane) + does the YUV->RGB matrix:
 *      control   0x500c -> 0xa0005101  (RGB888 out)
 *      stride[0] = w*3 (RGB row); stride[1]=stride[2]=0; iova[1]=iova[2]=0
 *      mat_ctrl  0      -> (6<<4)|1 = 0x61  (VIDO_MAT_CTRL: YUV->RGB select idx 6)
 *    The detile WROT extra words (u[8]=0x400 fifo_test, u[10] pre_ultra, u[21..]
 *    subfrm) are kept from the parent; the oracle c10 carries fifo_test=0x200 /
 *    pre_ultra=0x0008804c (see FLAGGED UNKNOWNS in the commit body).
 *
 * The CSC conversion itself is ONE register: WROT mat_ctrl. There is no
 * programmable CCORR matrix (img_compparam_8195's engine union has no ccorr
 * member); the YUV->RGB is a HW-internal coefficient set picked by int_matrix_sel.
 *
 * num_subfrms = 1 (single full-width pass, cloned from the detile formula).
 * FLAGGED UNKNOWN: the v23042 oracle is num_subfrm=3 (TILED) at 1280 AND an
 * 11-component pipe (types 9/14/24/26/30/34/38/42/46/48/54) -- a DIFFERENT SVPP
 * routing than the 10-comp detile. The oracle dump captured sub0 only, so the
 * per-tile (sub1/sub2) geometry and the alternate middle pipe are NOT
 * reconstructable from it; this draft reuses the PROVEN 10-comp detile formula +
 * the oracle-confirmed RDMA/WROT register diffs. If a full-width pass bands /
 * overruns the WROT line buffer (the oracle tiled for a reason), fall back to the
 * detile per-tile path (mdp_kd_* knobs / the multi-job detile_nchunks loop with a
 * LINEAR per-tile RDMA offset = x0 bytes Y / x0 bytes interleaved UV, NOT the MM21
 * x0*32 block formula). See the commit body.
 *
 * Every adapted/guessed register value is tagged CSC-HW-ITER for the HW sweep.
 */
static void mdp_config_compute_8195_csc(struct img_ipi_frameparam *param,
					struct img_config *config)
{
	struct img_config_8195 *cfg = &config->config_8195;
	const struct img_image_buffer *in = &param->inputs[0].buffer;
	const struct img_image_buffer *out = &param->outputs[0].buffer;
	u32 w = out->format.width;
	u32 h = out->format.height;
	u32 istride = in->format.plane_fmt[0].stride;	/* linear NV12 Y pitch (bytes) */
	u32 ostride = out->format.plane_fmt[0].stride;	/* RGB24 row stride (bytes; = w*3) */
	u32 iy = in->iova[0], ic = in->iova[1];
	u32 oy = out->iova[0];
	u32 hw = (h << 16) | w, wh = (w << 16) | h;
	struct img_mmsys_ctrl *mc;
	int i;

	memset(cfg, 0, sizeof(*cfg));
	cfg->num_components = 10;
	cfg->num_subfrms = 1;	/* CSC-HW-ITER: oracle is nsub=3; this draft is 1 */

	for (i = 0; i < 10; i++) {
		struct img_compparam_8195 *p = &cfg->components[i];
		struct img_comp_frame_8195 *f = &p->frame;
		struct img_comp_subfrm_8195 *s = &p->subfrms[0];
		u32 *u = (u32 *)&p->rdma;	/* engine union overlay (validated) */

		p->type = mdp_dt_comp[i].type;
		p->id = mdp_dt_comp[i].id;
		p->input = 0;
		p->num_outputs = 1;
		p->outputs[0] = 0;
		p->num_subfrms = 1;

		/* common frame + subfrm; overridden per component below. CSC is 1:1
		 * (in == out == h), so unlike the detile there is no inh/croph crop. */
		f->in_width = w; f->in_height = h;
		f->out_width = w; f->out_height = h;
		f->in_total_width = w; f->out_total_width = w;
		f->crop.width = w; f->crop.height = h;
		s->in.right = w - 1; s->in.bottom = h - 1;
		s->out.right = w - 1; s->out.bottom = h - 1;

		switch (i) {
		case 0:	/* RDMA: reads LINEAR NV12 (1:1, no block-mode crop) */
			/* CSC-HW-ITER: src_ctrl 0xc + control 0x1010 are the v23042
			 * oracle's *measured* linear-NV12 read values (c0 udata
			 * "0000000c 00000000 00001010 ff600000"). The earlier guess
			 * (0x060c = detile 0x860c & ~0x8000; control 0) left the 0x600
			 * MM21 swizzle bits set -> RDMA read scrambled -> ~4 dB garbage
			 * RGB even though the job completed. Use the oracle words. */
			u[0] = 0xc; u[2] = 0x1010; u[3] = iy; u[4] = ic;
			/* CSC-HW-ITER: LINEAR iova_end = base + plane size (NOT aw*inh) */
			u[6] = iy + istride * h; u[7] = ic + istride * h / 2;
			/* CSC-HW-ITER: LINEAR mf/sf_bkgd = istride (NOT aw*32 / aw*16) */
			u[9] = istride; u[11] = istride;
			/* RDMA ultra/dmabuf throttle words: kept from the detile (the
			 * oracle c0 carries the same 0x03000020/0xe800/... family). */
			u[15] = 0x03000020; u[16] = 0xe800; u[17] = 0x9c00;
			u[18] = 0x01000010; u[19] = 0x7400; u[20] = 0x4c00;
			/* subfrms[0]: src = clip = (h<<16)|w (no inh crop) */
			u[29] = hw; u[30] = hw;
			break;
		case 1:	/* RSZ: 1:1 */
			u[0] = 1; u[1] = 7; u[2] = (w << 16); u[3] = (h << 16);
			break;
		case 2:	/* relay: no geometry, no register config */
			f->in_total_width = 0; f->out_total_width = 0;
			f->crop.width = 0; f->crop.height = 0;
			memset(&s->in, 0, sizeof(s->in));
			memset(&s->out, 0, sizeof(s->out));
			break;
		case 3:	f->crop.width = 0; f->crop.height = 0;
			u[0] = 0x10000000; u[1] = 1; u[3] = hw;
			u[4] = (w - 1) << 16; u[5] = (h - 1) << 16;
			u[7] = w - 1; u[8] = 0x60; u[9] = 0x200;
			break;
		case 4:	f->crop.width = 0; f->crop.height = 0;
			u[0] = 0x80; u[1] = 1; u[2] = wh; u[3] = wh;
			break;
		case 5:	f->bypass = 1;
			u[15] = 0x1000; u[16] = hw; u[17] = hw; u[26] = hw;
			break;
		case 6:	f->bypass = 1;
			u[0] = 1; u[1] = wh; u[2] = wh;
			u[4] = (w - 1) << 16; u[5] = (h - 1) << 16;
			break;
		case 7:	f->crop.width = 0; f->crop.height = 0;
			u[0] = 3; u[1] = w; u[2] = h;
			break;
		case 8:	u[0] = ((h - 1) << 16) | w;
			break;
		case 9:	/* WROT: writes linear RGB24 (1 plane) + does the CSC */
			u[0] = oy; u[1] = 0; u[2] = 0;
			/* CSC-HW-ITER: control 0xa0005101 (RGB888 out, vs detile 0x500c) */
			u[3] = 0xa0005101;
			/* CSC-HW-ITER: stride[0] = ostride (= w*3 RGB row); RGB is 1 plane */
			u[4] = ostride; u[5] = 0; u[6] = 0;
			/* CSC-HW-ITER: mat_ctrl = (6<<4)|1 = 0x61 (VIDO_MAT_CTRL YUV->RGB
			 * select idx 6 = BT.601). The detile leaves this 0. */
			u[7] = 0x61;
			u[8] = 0x400; u[10] = 0x000d80c4;
			u[21] = hw; u[22] = hw; u[24] = (w << 16) | 0x4000;
			break;
		}
	}

	/* ctrls[0]: SVPP2 mmsys mux routing (geometry-independent; same pipe as
	 * the detile). CSC-HW-ITER: the v23042 oracle's 11-comp pipe uses a
	 * DIFFERENT SVPP routing -- if this mux mis-routes, the alternate set must
	 * be RE'd from a full (sub0..sub2) oracle capture. */
	mc = &cfg->ctrls[0];
	mc->num_sets = 7;
	for (i = 0; i < 7; i++) {
		mc->sets[i].reg = mdp_dt_mux[i].reg;
		mc->sets[i].value = mdp_dt_mux[i].value;
		mc->sets[i].subsys_id = mdp_dt_mux[i].subsys_id;
	}

	if (mdp_kd_debug_on()) {
		struct mdp_dev *mdp = vpu_to_mdp((struct mdp_vpu_dev *)
						 (unsigned long)param->drv_data);
		struct device *dev = &mdp->pdev->dev;
		const u32 *rd = (const u32 *)&cfg->components[0].rdma;
		const u32 *wr = (const u32 *)&cfg->components[9].wrot;

		dev_info(dev, "MDPKD-8195-CSC cfg: w=%u h=%u istride=%u ostride=%u ncomp=%u nsub=%u\n",
			 w, h, istride, ostride, cfg->num_components,
			 cfg->num_subfrms);
		dev_info(dev, "MDPKD-8195-CSC RDMA(c0 type=%u id=%u): src_ctrl=%#x control=%#x iova=[%#x,%#x] iova_end=[%#x,%#x] mf_bkgd=%#x sf_bkgd=%#x src=%#x clip=%#x\n",
			 cfg->components[0].type, cfg->components[0].id,
			 rd[0], rd[2], rd[3], rd[4], rd[6], rd[7], rd[9], rd[11],
			 rd[29], rd[30]);
		dev_info(dev, "MDPKD-8195-CSC WROT(c9 type=%u id=%u): iova=%#x control=%#x stride=%#x mat_ctrl=%#x byp(c5)=%u\n",
			 cfg->components[9].type, cfg->components[9].id,
			 wr[0], wr[3], wr[4], wr[7], cfg->components[5].frame.bypass);
		mc = &cfg->ctrls[0];
		for (i = 0; i < (int)mc->num_sets; i++)
			dev_info(dev, "MDPKD-8195-CSC mux[%d]: reg=%#x val=%#x subsys=%u\n",
				 i, mc->sets[i].reg, mc->sets[i].value,
				 mc->sets[i].subsys_id);
	}
}

/*
 * Kernel-direct blit (NV12 -> NV12, 1:1) config-compute for mt8195. The simplest
 * of the remaining SCP-path 2D ops to free from the SCP (KI-009). Same single-
 * SVPP2 10-component nsub=1 full-width pipe as mdp_config_compute_8195_csc; it
 * differs from the CSC in exactly TWO engines, per the v23042 SCP oracle
 * (the internal 2D-ops oracle record):
 *
 *  - RDMA (c0) reads LINEAR NV12, but src_ctrl = 0x60c, NOT the CSC's 0xc. The
 *    blit/scale/rotate/flip oracles ALL read with src_ctrl=0x60c; only the
 *    NV12->RGB CSC uses 0xc. The 0x600 bits are output-path-dependent (set for
 *    YUV output, clear for RGB) -- NOT swizzle bits to clear (that mis-read was
 *    the entire CSC-session bug). Transcribed from the oracle, not derived.
 *  - WROT (c9) writes NV12 (2 planes) with no CSC matrix: control = 0x500c (the
 *    detile's proven full-frame NV12 WROT), iova=[oy,oc], stride[0..2]=ostride,
 *    mat_ctrl = 0. The full-width main_buf (u[21]=hw, u[24]=(w<<16)|0x4000) is
 *    the detile's bit-exact formula, reused verbatim.
 *
 * So blit = the CSC compute's linear-NV12 RDMA (src_ctrl 0x60c) + the detile's
 * NV12 WROT -- both halves already HW-validated bit-exact. num_subfrms = 1.
 */
static void mdp_config_compute_8195_blit(struct img_ipi_frameparam *param,
					 struct img_config *config)
{
	struct img_config_8195 *cfg = &config->config_8195;
	const struct img_image_buffer *in = &param->inputs[0].buffer;
	const struct img_image_buffer *out = &param->outputs[0].buffer;
	u32 w = out->format.width;
	u32 h = out->format.height;
	u32 istride = in->format.plane_fmt[0].stride;	/* linear NV12 Y pitch (bytes) */
	u32 ostride = out->format.plane_fmt[0].stride;	/* NV12 out row stride (= w) */
	u32 iy = in->iova[0], ic = in->iova[1];
	u32 oy = out->iova[0], oc = out->iova[1];
	u32 hw = (h << 16) | w, wh = (w << 16) | h;
	struct img_mmsys_ctrl *mc;
	int i;

	memset(cfg, 0, sizeof(*cfg));
	cfg->num_components = 10;
	cfg->num_subfrms = 1;

	for (i = 0; i < 10; i++) {
		struct img_compparam_8195 *p = &cfg->components[i];
		struct img_comp_frame_8195 *f = &p->frame;
		struct img_comp_subfrm_8195 *s = &p->subfrms[0];
		u32 *u = (u32 *)&p->rdma;	/* engine union overlay (validated) */

		p->type = mdp_dt_comp[i].type;
		p->id = mdp_dt_comp[i].id;
		p->input = 0;
		p->num_outputs = 1;
		p->outputs[0] = 0;
		p->num_subfrms = 1;

		/* common frame + subfrm; overridden per component below. 1:1, no crop. */
		f->in_width = w; f->in_height = h;
		f->out_width = w; f->out_height = h;
		f->in_total_width = w; f->out_total_width = w;
		f->crop.width = w; f->crop.height = h;
		s->in.right = w - 1; s->in.bottom = h - 1;
		s->out.right = w - 1; s->out.bottom = h - 1;

		switch (i) {
		case 0:	/* RDMA: reads LINEAR NV12 (1:1). src_ctrl=0x60c (oracle): the
			 * NV12-out read register, NOT the CSC's 0xc. */
			u[0] = 0x60c; u[2] = 0x1010; u[3] = iy; u[4] = ic;
			u[6] = iy + istride * h; u[7] = ic + istride * h / 2;
			u[9] = istride; u[11] = istride;
			u[15] = 0x03000020; u[16] = 0xe800; u[17] = 0x9c00;
			u[18] = 0x01000010; u[19] = 0x7400; u[20] = 0x4c00;
			u[29] = hw; u[30] = hw;
			break;
		case 1:	/* RSZ: 1:1 */
			u[0] = 1; u[1] = 7; u[2] = (w << 16); u[3] = (h << 16);
			break;
		case 2:	/* relay: no geometry, no register config */
			f->in_total_width = 0; f->out_total_width = 0;
			f->crop.width = 0; f->crop.height = 0;
			memset(&s->in, 0, sizeof(s->in));
			memset(&s->out, 0, sizeof(s->out));
			break;
		case 3:	f->crop.width = 0; f->crop.height = 0;
			u[0] = 0x10000000; u[1] = 1; u[3] = hw;
			u[4] = (w - 1) << 16; u[5] = (h - 1) << 16;
			u[7] = w - 1; u[8] = 0x60; u[9] = 0x200;
			break;
		case 4:	f->crop.width = 0; f->crop.height = 0;
			u[0] = 0x80; u[1] = 1; u[2] = wh; u[3] = wh;
			break;
		case 5:	f->bypass = 1;
			u[15] = 0x1000; u[16] = hw; u[17] = hw; u[26] = hw;
			break;
		case 6:	f->bypass = 1;
			u[0] = 1; u[1] = wh; u[2] = wh;
			u[4] = (w - 1) << 16; u[5] = (h - 1) << 16;
			break;
		case 7:	f->crop.width = 0; f->crop.height = 0;
			u[0] = 3; u[1] = w; u[2] = h;
			break;
		case 8:	u[0] = ((h - 1) << 16) | w;
			break;
		case 9:	/* WROT: writes linear NV12 (2 planes), no CSC matrix. Same as the
			 * detile WROT (control=0x500c, full-width main_buf). */
			u[0] = oy; u[1] = oc; u[3] = 0x500c;
			u[4] = ostride; u[5] = ostride; u[6] = ostride;
			u[8] = 0x400; u[10] = 0x000d80c4;
			u[21] = hw; u[22] = hw; u[24] = (w << 16) | 0x4000;
			break;
		}
	}

	/* ctrls[0]: SVPP2 mmsys mux routing (geometry-independent; same pipe). */
	mc = &cfg->ctrls[0];
	mc->num_sets = 7;
	for (i = 0; i < 7; i++) {
		mc->sets[i].reg = mdp_dt_mux[i].reg;
		mc->sets[i].value = mdp_dt_mux[i].value;
		mc->sets[i].subsys_id = mdp_dt_mux[i].subsys_id;
	}

	if (mdp_kd_debug_on()) {
		struct mdp_dev *mdp = vpu_to_mdp((struct mdp_vpu_dev *)
						 (unsigned long)param->drv_data);
		struct device *dev = &mdp->pdev->dev;
		const u32 *rd = (const u32 *)&cfg->components[0].rdma;
		const u32 *wr = (const u32 *)&cfg->components[9].wrot;

		dev_info(dev, "MDPKD-8195-BLIT cfg: w=%u h=%u istride=%u ostride=%u ncomp=%u nsub=%u\n",
			 w, h, istride, ostride, cfg->num_components,
			 cfg->num_subfrms);
		dev_info(dev, "MDPKD-8195-BLIT RDMA(c0): src_ctrl=%#x control=%#x iova=[%#x,%#x] iova_end=[%#x,%#x] mf_bkgd=%#x sf_bkgd=%#x src=%#x clip=%#x\n",
			 rd[0], rd[2], rd[3], rd[4], rd[6], rd[7], rd[9], rd[11],
			 rd[29], rd[30]);
		dev_info(dev, "MDPKD-8195-BLIT WROT(c9): iova=[%#x,%#x] control=%#x stride=%#x mat_ctrl=%#x\n",
			 wr[0], wr[1], wr[3], wr[4], wr[7]);
	}
}

/*
 * Kernel-direct rotate-90-CW (NV12 -> NV12) config-compute for mt8195. Same
 * single-SVPP2 10-component nsub=1 pipe as blit, processing the INPUT W x H
 * (rotation happens on the WROT write to the swapped H x W output buffer). Per
 * the v23042 SCP oracle (the internal 2D-ops oracle record),
 * derived from nsub=1 captures at 64/128/256 + 256x128 + 128x256, the rotate-90
 * WROT differs from blit's NV12 WROT in exactly:
 *   - control = 0x4010500c (blit 0x500c | the rotate-90 bits 0x40100000)
 *   - stride[0..2] = ostride = h (the rotated output row = input height)
 *   - u[18..20] = h - 1 (= output width - 1; blit leaves these 0)
 * u[21]=u[22]=(h<<16)|w and u[24]=(w<<16)|0x4000 are identical to blit (they were
 * already (oh<<16)|... forms). RDMA + middle pipe are blit's verbatim (src_ctrl
 * 0x60c linear NV12). Output iova oy/oc address the H x W buffer.
 */
static void mdp_config_compute_8195_rotate90(struct img_ipi_frameparam *param,
					     struct img_config *config)
{
	struct img_config_8195 *cfg = &config->config_8195;
	const struct img_image_buffer *in = &param->inputs[0].buffer;
	const struct img_image_buffer *out = &param->outputs[0].buffer;
	u32 w = in->format.width;	/* pixel-pipe dims = INPUT (pre-rotation) */
	u32 h = in->format.height;
	u32 istride = in->format.plane_fmt[0].stride;	/* linear NV12 Y pitch */
	u32 ostride = out->format.plane_fmt[0].stride;	/* rotated out row stride (= h) */
	u32 iy = in->iova[0], ic = in->iova[1];
	u32 oy = out->iova[0], oc = out->iova[1];
	u32 hw = (h << 16) | w, wh = (w << 16) | h;
	struct img_mmsys_ctrl *mc;
	int i;

	memset(cfg, 0, sizeof(*cfg));
	cfg->num_components = 10;
	cfg->num_subfrms = 1;

	for (i = 0; i < 10; i++) {
		struct img_compparam_8195 *p = &cfg->components[i];
		struct img_comp_frame_8195 *f = &p->frame;
		struct img_comp_subfrm_8195 *s = &p->subfrms[0];
		u32 *u = (u32 *)&p->rdma;

		p->type = mdp_dt_comp[i].type;
		p->id = mdp_dt_comp[i].id;
		p->input = 0;
		p->num_outputs = 1;
		p->outputs[0] = 0;
		p->num_subfrms = 1;

		/* pixel pipe is the INPUT frame W x H (rotation is on the WROT write) */
		f->in_width = w; f->in_height = h;
		f->out_width = w; f->out_height = h;
		f->in_total_width = w; f->out_total_width = w;
		f->crop.width = w; f->crop.height = h;
		s->in.right = w - 1; s->in.bottom = h - 1;
		s->out.right = w - 1; s->out.bottom = h - 1;

		switch (i) {
		case 0:	/* RDMA: linear NV12 in (blit's verbatim, src_ctrl=0x60c) */
			u[0] = 0x60c; u[2] = 0x1010; u[3] = iy; u[4] = ic;
			u[6] = iy + istride * h; u[7] = ic + istride * h / 2;
			u[9] = istride; u[11] = istride;
			u[15] = 0x03000020; u[16] = 0xe800; u[17] = 0x9c00;
			u[18] = 0x01000010; u[19] = 0x7400; u[20] = 0x4c00;
			u[29] = hw; u[30] = hw;
			break;
		case 1:	/* RSZ: 1:1 */
			u[0] = 1; u[1] = 7; u[2] = (w << 16); u[3] = (h << 16);
			break;
		case 2:	/* relay */
			f->in_total_width = 0; f->out_total_width = 0;
			f->crop.width = 0; f->crop.height = 0;
			memset(&s->in, 0, sizeof(s->in));
			memset(&s->out, 0, sizeof(s->out));
			break;
		case 3:	f->crop.width = 0; f->crop.height = 0;
			u[0] = 0x10000000; u[1] = 1; u[3] = hw;
			u[4] = (w - 1) << 16; u[5] = (h - 1) << 16;
			u[7] = w - 1; u[8] = 0x60; u[9] = 0x200;
			break;
		case 4:	f->crop.width = 0; f->crop.height = 0;
			u[0] = 0x80; u[1] = 1; u[2] = wh; u[3] = wh;
			break;
		case 5:	f->bypass = 1;
			u[15] = 0x1000; u[16] = hw; u[17] = hw; u[26] = hw;
			break;
		case 6:	f->bypass = 1;
			u[0] = 1; u[1] = wh; u[2] = wh;
			u[4] = (w - 1) << 16; u[5] = (h - 1) << 16;
			break;
		case 7:	f->crop.width = 0; f->crop.height = 0;
			u[0] = 3; u[1] = w; u[2] = h;
			break;
		case 8:	u[0] = ((h - 1) << 16) | w;
			break;
		case 9:	/* WROT: rotate-90 NV12 write to the H x W output buffer. */
			u[0] = oy; u[1] = oc; u[3] = 0x4010500c;
			u[4] = ostride; u[5] = ostride; u[6] = ostride;
			u[8] = 0x400; u[10] = 0x000d80c4;
			/* rotate main_buf (oracle, nsub=1): u[18..20]=ow-1=h-1, the rest
			 * (u[21..22], u[24]) are identical to blit's full-width form. */
			u[18] = h - 1; u[19] = h - 1; u[20] = h - 1;
			u[21] = hw; u[22] = hw; u[24] = (w << 16) | 0x4000;
			break;
		}
	}

	mc = &cfg->ctrls[0];
	mc->num_sets = 7;
	for (i = 0; i < 7; i++) {
		mc->sets[i].reg = mdp_dt_mux[i].reg;
		mc->sets[i].value = mdp_dt_mux[i].value;
		mc->sets[i].subsys_id = mdp_dt_mux[i].subsys_id;
	}

	if (mdp_kd_debug_on()) {
		struct mdp_dev *mdp = vpu_to_mdp((struct mdp_vpu_dev *)
						 (unsigned long)param->drv_data);
		struct device *dev = &mdp->pdev->dev;
		const u32 *wr = (const u32 *)&cfg->components[9].wrot;

		dev_info(dev, "MDPKD-8195-ROT90 cfg: in=%ux%u ostride=%u; WROT control=%#x stride=%#x u18=%#x u21=%#x u24=%#x\n",
			 w, h, ostride, wr[3], wr[4], wr[18], wr[21], wr[24]);
	}
}

/*
 * Kernel-direct rotate-180/270 + hflip/vflip config-compute for mt8195. Identical
 * to rotate-90 (blit's pipe on the input W x H, WROT writing the output buffer)
 * except the WROT control + the VIDO output-start offset (u[18]=Y, u[19]=u[20]=UV)
 * come from the variant (control) and offset mode -- see mdp_xform_variant +
 * audit/2d-ops-oracle-mt8195.md. ostride is read from the out buffer (= h for the
 * dim-swapping rotate-270, = w otherwise). u[21]=u[22]=(h<<16)|w and
 * u[24]=(w<<16)|0x4000 are blit's verbatim (constant across all transforms).
 */
static void mdp_config_compute_8195_xform(struct img_ipi_frameparam *param,
					  struct img_config *config,
					  u32 control, enum mdp_wrot_ofst mode)
{
	struct img_config_8195 *cfg = &config->config_8195;
	const struct img_image_buffer *in = &param->inputs[0].buffer;
	const struct img_image_buffer *out = &param->outputs[0].buffer;
	u32 w = in->format.width;	/* pixel-pipe dims = INPUT (pre-transform) */
	u32 h = in->format.height;
	u32 istride = in->format.plane_fmt[0].stride;
	u32 ostride = out->format.plane_fmt[0].stride;
	u32 iy = in->iova[0], ic = in->iova[1];
	u32 oy = out->iova[0], oc = out->iova[1];
	u32 hw = (h << 16) | w, wh = (w << 16) | h;
	u32 ysz = w * h, csz = ysz / 2;		/* Y / UV plane pixel counts */
	u32 ofst_y, ofst_uv;
	struct img_mmsys_ctrl *mc;
	int i;

	switch (mode) {
	case MDP_OFST_ROW_REV:  ofst_y = ostride - 1; ofst_uv = ostride - 1; break;
	case MDP_OFST_FULL_REV: ofst_y = ysz - 1;     ofst_uv = csz - 1;     break;
	default: /* MDP_OFST_COL_REV */
				ofst_y = ysz - ostride; ofst_uv = csz - ostride; break;
	}

	memset(cfg, 0, sizeof(*cfg));
	cfg->num_components = 10;
	cfg->num_subfrms = 1;

	for (i = 0; i < 10; i++) {
		struct img_compparam_8195 *p = &cfg->components[i];
		struct img_comp_frame_8195 *f = &p->frame;
		struct img_comp_subfrm_8195 *s = &p->subfrms[0];
		u32 *u = (u32 *)&p->rdma;

		p->type = mdp_dt_comp[i].type;
		p->id = mdp_dt_comp[i].id;
		p->input = 0;
		p->num_outputs = 1;
		p->outputs[0] = 0;
		p->num_subfrms = 1;

		f->in_width = w; f->in_height = h;
		f->out_width = w; f->out_height = h;
		f->in_total_width = w; f->out_total_width = w;
		f->crop.width = w; f->crop.height = h;
		s->in.right = w - 1; s->in.bottom = h - 1;
		s->out.right = w - 1; s->out.bottom = h - 1;

		switch (i) {
		case 0:	/* RDMA: linear NV12 in (blit's verbatim, src_ctrl=0x60c) */
			u[0] = 0x60c; u[2] = 0x1010; u[3] = iy; u[4] = ic;
			u[6] = iy + istride * h; u[7] = ic + istride * h / 2;
			u[9] = istride; u[11] = istride;
			u[15] = 0x03000020; u[16] = 0xe800; u[17] = 0x9c00;
			u[18] = 0x01000010; u[19] = 0x7400; u[20] = 0x4c00;
			u[29] = hw; u[30] = hw;
			break;
		case 1:	u[0] = 1; u[1] = 7; u[2] = (w << 16); u[3] = (h << 16); break;
		case 2:	f->in_total_width = 0; f->out_total_width = 0;
			f->crop.width = 0; f->crop.height = 0;
			memset(&s->in, 0, sizeof(s->in));
			memset(&s->out, 0, sizeof(s->out));
			break;
		case 3:	f->crop.width = 0; f->crop.height = 0;
			u[0] = 0x10000000; u[1] = 1; u[3] = hw;
			u[4] = (w - 1) << 16; u[5] = (h - 1) << 16;
			u[7] = w - 1; u[8] = 0x60; u[9] = 0x200;
			break;
		case 4:	f->crop.width = 0; f->crop.height = 0;
			u[0] = 0x80; u[1] = 1; u[2] = wh; u[3] = wh; break;
		case 5:	f->bypass = 1;
			u[15] = 0x1000; u[16] = hw; u[17] = hw; u[26] = hw; break;
		case 6:	f->bypass = 1;
			u[0] = 1; u[1] = wh; u[2] = wh;
			u[4] = (w - 1) << 16; u[5] = (h - 1) << 16;
			break;
		case 7:	f->crop.width = 0; f->crop.height = 0;
			u[0] = 3; u[1] = w; u[2] = h; break;
		case 8:	u[0] = ((h - 1) << 16) | w; break;
		case 9:	/* WROT: transform NV12 write. control + offsets per variant. */
			u[0] = oy; u[1] = oc; u[3] = control;
			u[4] = ostride; u[5] = ostride; u[6] = ostride;
			u[8] = 0x400; u[10] = 0x000d80c4;
			u[18] = ofst_y; u[19] = ofst_uv; u[20] = ofst_uv;
			u[21] = hw; u[22] = hw; u[24] = (w << 16) | 0x4000;
			break;
		}
	}

	mc = &cfg->ctrls[0];
	mc->num_sets = 7;
	for (i = 0; i < 7; i++) {
		mc->sets[i].reg = mdp_dt_mux[i].reg;
		mc->sets[i].value = mdp_dt_mux[i].value;
		mc->sets[i].subsys_id = mdp_dt_mux[i].subsys_id;
	}

	if (mdp_kd_debug_on()) {
		struct mdp_dev *mdp = vpu_to_mdp((struct mdp_vpu_dev *)
						 (unsigned long)param->drv_data);
		struct device *dev = &mdp->pdev->dev;
		const u32 *wr = (const u32 *)&cfg->components[9].wrot;

		dev_info(dev, "MDPKD-8195-XFORM cfg: in=%ux%u ostride=%u mode=%d; WROT control=%#x stride=%#x ofst_y=%#x ofst_uv=%#x\n",
			 w, h, ostride, mode, wr[3], wr[4], wr[18], wr[19]);
	}
}

/*
 * Source-read geometry override for the LINEAR dual primitive (c0 / RDMA) and the
 * SINGLE source-of-truth for the <=1920 fmtconv compute case-0 source classification.
 * NULL/all-false => the default 2-plane 4:2:0 NV12 read (Y=iy istride, interleaved
 * plane-1 chroma=ic istride HALF-height, per-pipe column read offset rx for both
 * planes). The other flags extend the read to the source families the fmtconv
 * compute reads (nonnv12-source-kd-spec-mt8195.md). Filled + the src_ctrl returned
 * by mdp_dual_src_for(). Oracle-validated vs the v23042 SCP pp0+pp1 cfg8195 snapshot.
 */
struct mdp_dual_src {
	bool chroma_full_h;	/* 4:2:2 source: plane-1/2 chroma spans h rows (not h/2) */
	bool single_plane;	/* RGB / packed-4:2:2: one packed plane, no plane-1/2 read.
				 * The per-half column read offset scales by the source
				 * byte-per-pixel (xbpp): u[25]=rx*xbpp (packed422/RGB565=2,
				 * RGB24=3, ARGB=4); u[26..27] stay 0 (no chroma plane). */
	bool three_plane;	/* I420/YV12 (4:2:0) or YM16/YM61 (4:2:2, +chroma_full_h):
				 * separate U=ic(iova1)/V=iova2 planes, half-WIDTH chroma
				 * (pitch istride/2, per-half offset rx/2). */
	bool colortran;		/* RGB source: RDMA read-CSC (u[14]=0x10000). */
	u32  iova2;		/* plane-2 (V) base -- three_plane only. */
	u8   xbpp;		/* single_plane source bytes-per-pixel (0 => 1). */
};

/*
 * Classify a fmtconv SOURCE color into the per-family RDMA read (struct mdp_dual_src)
 * and return the RDMA src_ctrl -- the ONE place that encodes the mt8195 source-read
 * rules, consumed by BOTH the <=1920 single-pipe compute (mdp_config_compute_8195_
 * fmtconv case 0) and the >1920 dual linear primitive (mdp_fill_dual_pipe_8195_linear).
 * `f` is the dst fmtconv row (some src_ctrls are dst-keyed: I420 filterMode from
 * f->src_ctrl&0x600; NV16->RGB un-swap from f->rgb). All values transcribed from the
 * v23042 SCP oracle; see the case-0 per-family comments + nonnv12-source-kd-spec-mt8195.md.
 */
static u32 mdp_dual_src_for(u32 in_color, const struct mdp_fmtconv *f,
			    const struct img_image_buffer *in,
			    struct mdp_dual_src *src)
{
	u32 istride = in->format.plane_fmt[0].stride;
	u32 iw = in->format.width;
	u32 sw = MDP_COLOR_IS_SWAPPED(in_color) ? 0x4000 : 0;

	memset(src, 0, sizeof(*src));

	if (mdp_color_is_rgb_src(in_color)) {
		/* RGB565/BGR565/RGB24/BGR24/ARGB/ABGR: single packed plane +
		 * RDMA RGB->YUV read-CSC (colortran); per-half offset rx*bpp. */
		src->single_plane = true;
		src->colortran = true;
		src->xbpp = iw ? istride / iw : 1;
		return 0x600 | sw | MDP_COLOR_GET_HW_FORMAT(in_color);
	}
	if (mdp_color_is_packed422src(in_color)) {
		/* YUYV/YVYU/UYVY/VYUY: one packed plane, bpp=2, no plane-1 read. */
		src->single_plane = true;
		src->xbpp = 2;
		return 0x600 | sw | MDP_COLOR_GET_HW_FORMAT(in_color);
	}
	if (mdp_color_is_plain_nv16src(in_color)) {
		/* NV16/NV61 (2-plane 4:2:2): FULL-height plane-1 chroma. src_ctrl
		 * 0x60d (0x600|HW_FORMAT 0xd), NO sourceSwap for a YUV dst (a
		 * swapped NV61 rides the swap-twin WROT row); an RGB dst has no
		 * chroma swap-twin, so un-swap on the read (0x460d). */
		u32 sc = 0x600 | MDP_COLOR_GET_HW_FORMAT(in_color);

		src->chroma_full_h = true;
		if (f && f->rgb && MDP_COLOR_IS_SWAPPED(in_color))
			sc |= 0x4000;
		return sc;
	}
	if (mdp_color_is_plain_i420src(in_color)) {
		/* I420/YV12 (3-plane 4:2:0): half-WIDTH half-height chroma.
		 * filterMode from the dst row; 0x4000 un-swaps YV12's V-first. */
		src->three_plane = true;
		src->iova2 = in->iova[2];
		return (f->src_ctrl & 0x600) | sw | 0x08;
	}
	if (mdp_color_is_ym16src(in_color)) {
		/* YM16/YM61 (3-plane 4:2:2): half-WIDTH FULL-height chroma;
		 * 0x4000 un-swaps YM61's V-first. */
		src->three_plane = true;
		src->chroma_full_h = true;
		src->iova2 = in->iova[2];
		return 0x600 | sw | 0x09;
	}
	/* default 2-plane NV12-family (NV12/NV21/NM12/NM21): src_ctrl keyed off
	 * the output subsample (f->src_ctrl). For a YUV dst an NV21/NM21 source's
	 * swapped (V-before-U) chroma is compensated by the swap-twin WROT row
	 * (mdp_fmtconv_for -> lookup(swap_twin(out))). But an RGB dst has NO chroma
	 * swap-twin (mdp_chroma_swap_twin(RGB)==RGB), so the WROT can't un-swap it;
	 * the swapped chroma must be un-done at the RDMA read (0x4000 sourceSwap)
	 * exactly as the NV16/NV61 and I420/YV12 arms above -- else the YUV->RGB CSC
	 * reads Cb/Cr transposed -> red<->blue (Cr/Cb) swap. Only for a plain
	 * NV21/NM21 (swapped 4:2:0 2-plane) source to an RGB dst; a plain NV12/NM12
	 * source (already correct) and every YUV dst are untouched (f->rgb false). */
	if (f && f->rgb && mdp_color_is_plain_nv21(in_color))
		return f->src_ctrl | 0x4000;
	return f->src_ctrl;
}

/*
 * Kernel-direct NV12 -> <fmt> format-conversion config-compute for mt8195. Same
 * blit pipe (1:1 on the input W x H, no transform), parameterized by an
 * mdp_fmtconv_tbl row (`f`): the RDMA read mode (src_ctrl) and the WROT output
 * format (control / mat_ctrl / pre_ultra) come from the row; the WROT iova[0..2]
 * and stride[0..2] come from the OUT buffer (the driver fills the per-plane
 * addresses + pitches, so 3-plane I420/YV12 and 1-plane packed/RGB are handled
 * uniformly). RE: audit/2d-ops-oracle-mt8195.md "Format-conversion matrix".
 */
static void mdp_config_compute_8195_fmtconv(struct img_ipi_frameparam *param,
					    struct img_config *config,
					    const struct mdp_fmtconv *f)
{
	struct img_config_8195 *cfg = &config->config_8195;
	const struct img_image_buffer *in = &param->inputs[0].buffer;
	const struct img_image_buffer *out = &param->outputs[0].buffer;
	u32 w = out->format.width;
	u32 h = out->format.height;
	u32 istride = in->format.plane_fmt[0].stride;
	u32 iy = in->iova[0], ic = in->iova[1];
	u32 hw = (h << 16) | w, wh = (w << 16) | h;
	u32 in_color = in->format.colorformat;
	/* rgb_src is needed by the WROT CSC-select (case 9); the full per-family
	 * source classification + src_ctrl comes from mdp_dual_src_for() (the shared
	 * source-of-truth, case 0). A single-Y-plane SOURCE (packed RGB / packed
	 * 4:2:2) has no plane-1 chroma: mdp_dual_src_for sets single_plane so the RDMA
	 * reads ONE packed plane (no phantom plane-1 read -> no SVPP2 region-2 IOMMU
	 * fault, 09834f3); RGB adds the read-CSC (colortran) + the WROT RGB->YUV
	 * matrix (case 9 mat_ctrl 0x81). */
	bool rgb_src = mdp_color_is_rgb_src(in_color);
	/* KI-016: MM21/MT21 block-mode source -> the RDMA (c0) does the DETILE block
	 * read (byte-identical to mdp_config_compute_8195 c0) so the pipe carries linear
	 * 4:2:0 YUV; the WROT (c9) then does the per-dst X-conversion unchanged. aw =
	 * MM21 read-side aligned width (luma tile-row stride = aw*32); inh = block-padded
	 * input height; croph = tile-aligned (32) crop height. */
	bool srcmm21 = mdp_kd_mm21conv8195 && mdp_color_is_mm21src(in_color);
	u32 aw = ALIGN(w, 64);
	u32 inh = in->format.height;
	u32 croph = ALIGN(h, 32);
	struct img_mmsys_ctrl *mc;
	int i;

	memset(cfg, 0, sizeof(*cfg));
	cfg->num_components = 10;
	cfg->num_subfrms = 1;

	for (i = 0; i < 10; i++) {
		struct img_compparam_8195 *p = &cfg->components[i];
		struct img_comp_frame_8195 *fr = &p->frame;
		struct img_comp_subfrm_8195 *s = &p->subfrms[0];
		u32 *u = (u32 *)&p->rdma;

		p->type = mdp_dt_comp[i].type;
		p->id = mdp_dt_comp[i].id;
		p->input = 0;
		p->num_outputs = 1;
		p->outputs[0] = 0;
		p->num_subfrms = 1;

		fr->in_width = w; fr->in_height = h;
		fr->out_width = w; fr->out_height = h;
		fr->in_total_width = w; fr->out_total_width = w;
		fr->crop.width = w; fr->crop.height = h;
		s->in.right = w - 1; s->in.bottom = h - 1;
		s->out.right = w - 1; s->out.bottom = h - 1;

		switch (i) {
		case 0: {	/* RDMA: source read. Classification + src_ctrl from the
				 * shared source-of-truth mdp_dual_src_for(); the slot
				 * layout (u[3..11]/u[14]) is byte-identical to the dual
				 * linear primitive's case 0 -- both consume the same
				 * struct mdp_dual_src. See the per-family oracle notes on
				 * mdp_dual_src_for + nonnv12-source-kd-spec-mt8195.md. */
			struct mdp_dual_src src;
			u32 src_ctrl;

			if (srcmm21) {
				/* KI-016: MM21 detile block read -- byte-identical to
				 * the validated MM21->NV12 detile (mdp_config_compute_
				 * 8195 c0): reads inh tall, crops to ALIGN(h,32); the
				 * pipe then carries linear 4:2:0 YUV, so c9's WROT does
				 * the X-conversion exactly as for an NV12 source. */
				fr->in_height = inh;
				fr->crop.height = croph;
				s->in.bottom = croph - 1;
				u[0] = 0x860c; u[2] = 0x10; u[3] = iy; u[4] = ic;
				u[6] = iy + aw * inh; u[7] = ic + aw * inh / 2;
				u[9] = aw * 32; u[11] = aw * 16;
				u[29] = (croph << 16) | w; u[30] = hw;
			} else {
				src_ctrl = mdp_dual_src_for(in_color, f, in, &src);

				u[0] = src_ctrl; u[2] = 0x1010; u[3] = iy;
				u[6] = iy + istride * h; u[9] = istride;
				if (src.single_plane) {
					/* RGB / packed-4:2:2: ONE packed plane, no plane-1
					 * read (no phantom-plane SVPP2 region-2 fault); RGB
					 * adds the RDMA read-CSC (colortran). */
					u[14] = src.colortran ? 0x10000 : 0;
				} else if (src.three_plane) {
					/* I420/YV12 (4:2:0 half-height) or YM16/YM61 (4:2:2
					 * full-height): U=ic, V=iova2, half-WIDTH chroma. */
					u32 icst = istride / 2;
					u32 ch = src.chroma_full_h ? h : h / 2;

					u[4] = ic; u[5] = src.iova2;
					u[7] = ic + icst * ch;
					u[8] = src.iova2 + icst * ch;
					u[11] = icst;
				} else {
					/* 2-plane NV12 (4:2:0) / NV16/NV61 (4:2:2,
					 * chroma_full_h): interleaved plane-1 chroma. */
					u[4] = ic;
					u[7] = ic + istride * (src.chroma_full_h ? h : h / 2);
					u[11] = istride;
				}
				u[29] = hw; u[30] = hw;
			}
			u[15] = 0x03000020; u[16] = 0xe800; u[17] = 0x9c00;
			u[18] = 0x01000010; u[19] = 0x7400; u[20] = 0x4c00;
			break;
		}
		case 1:	if (srcmm21) fr->in_height = inh;
			u[0] = 1; u[1] = 7; u[2] = (w << 16); u[3] = (h << 16); break;
		case 2:	if (srcmm21) fr->in_height = inh;
			fr->in_total_width = 0; fr->out_total_width = 0;
			fr->crop.width = 0; fr->crop.height = 0;
			memset(&s->in, 0, sizeof(s->in));
			memset(&s->out, 0, sizeof(s->out));
			break;
		case 3:	fr->crop.width = 0; fr->crop.height = 0;
			u[0] = 0x10000000; u[1] = 1; u[3] = hw;
			u[4] = (w - 1) << 16; u[5] = (h - 1) << 16;
			u[7] = w - 1; u[8] = 0x60; u[9] = 0x200;
			break;
		case 4:	fr->crop.width = 0; fr->crop.height = 0;
			u[0] = 0x80; u[1] = 1; u[2] = wh; u[3] = wh; break;
		case 5:	fr->bypass = 1;
			u[15] = 0x1000; u[16] = hw; u[17] = hw; u[26] = hw; break;
		case 6:	fr->bypass = 1;
			u[0] = 1; u[1] = wh; u[2] = wh;
			u[4] = (w - 1) << 16; u[5] = (h - 1) << 16;
			break;
		case 7:	fr->crop.width = 0; fr->crop.height = 0;
			u[0] = 3; u[1] = w; u[2] = h; break;
		case 8:	u[0] = ((h - 1) << 16) | w; break;
		case 9: {	/* WROT: per-format output. iova/stride from the out buffer. */
			u32 c1 = out->iova[1], c2 = out->iova[2];

			/* For a planar V-first layout (YV12: 3-plane + SWAP) the buffer's
			 * iova[1]=V, iova[2]=U; the WROT wants the chroma planes in (U,V)
			 * order (+ the control swap bit), so pass them swapped. YU12 (no
			 * SWAP) and the semi-planar/packed formats need no iova swap. */
			if (MDP_COLOR_GET_PLANE_COUNT(out->format.colorformat) == 3 &&
			    MDP_COLOR_IS_SWAPPED(out->format.colorformat)) {
				u32 t = c1; c1 = c2; c2 = t;
			}
			u[0] = out->iova[0]; u[1] = c1; u[2] = c2;
			u[3] = f->control;
			u[4] = out->format.plane_fmt[0].stride;
			u[5] = out->format.plane_fmt[1].stride;
			u[6] = out->format.plane_fmt[2].stride;
			/* WROT CSC matrix select: RGB source -> RGB dst is a repack
			 * (mat 0x41, the RDMA already did RGB->YUV via colorTran); YUV
			 * source -> RGB dst is YUV->RGB (0x61); RGB source -> YUV dst is
			 * RGB->YUV (0x81); YUV->YUV is none (0). v23042 oracle: RGB3>BGR3
			 * mat=0x41, RGB3>NV12 mat=0x81, YUYV>RGB3 mat=0x61. */
			u[7] = (rgb_src && f->rgb) ? 0x41 :
			       f->rgb ? 0x61 : (rgb_src ? 0x81 : 0);
			u[8] = 0x400; u[10] = f->pre_ultra;
			u[21] = hw; u[22] = hw; u[24] = (w << 16) | 0x4000;
			break;
		}
		}
	}

	mc = &cfg->ctrls[0];
	mc->num_sets = 7;
	for (i = 0; i < 7; i++) {
		mc->sets[i].reg = mdp_dt_mux[i].reg;
		mc->sets[i].value = mdp_dt_mux[i].value;
		mc->sets[i].subsys_id = mdp_dt_mux[i].subsys_id;
	}

	if (mdp_kd_debug_on()) {
		struct mdp_dev *mdp = vpu_to_mdp((struct mdp_vpu_dev *)
						 (unsigned long)param->drv_data);
		struct device *dev = &mdp->pdev->dev;
		const u32 *rd = (const u32 *)&cfg->components[0].rdma;
		const u32 *wr = (const u32 *)&cfg->components[9].wrot;

		dev_info(dev, "MDPKD-8195-FMTCONV in=%ux%u color=%#x: RDMA src_ctrl=%#x; WROT iova=[%#x,%#x,%#x] control=%#x stride=[%#x,%#x,%#x] mat=%#x\n",
			 w, h, f->color, rd[0], wr[0], wr[1], wr[2], wr[3],
			 wr[4], wr[5], wr[6], wr[7]);
	}
}

/*
 * Kernel-direct NV12 -> NV12 SCALE config-compute for mt8195. Unlike the 1:1 ops
 * (which reuse blit's 10-component bypassed-RSZ pipe), scale needs the FULL
 * 11-component SVPP pipe with the RSZ ACTIVE -- it uses a DIFFERENT mmsys mux
 * (mdp_scale_mux, regs 0xf1c.. vs blit's 0xf50..) and includes the type-46
 * component (c8) the 10-comp pipe drops. Transcribed VERBATIM from the v23042 SCP
 * oracle (audit/2d-ops-oracle-mt8195.md "Scale"), validated W/H order against a
 * non-square (256x128->128x64) capture: c0-c4 at the INPUT iw x ih, c5 RSZ resizes
 * iw x ih -> ow x oh, c6 (transition, byp) frame.in=iw/out=ow, c7-c10 at OUTPUT.
 *   RSZ coeff_step_x = ((ow-1)<<20)/(iw-1), coeff_step_y = ((oh-1)<<20)/(ih-1);
 *   control1 = 0x02310143 (incl. PRZ enable bit0); subfrm src=hwi, clip=hwo.
 * RDMA reads NV12 (src_ctrl=0x60c), WROT writes NV12 (control=0x500c). nsub=1.
 */
static const struct { u8 type, id; } mdp_scale_comp[11] = {
	{ 9, 1 }, { 14, 1 }, { 24, 1 }, { 26, 1 }, { 30, 1 }, { 34, 1 },
	{ 38, 1 }, { 42, 1 }, { 46, 1 }, { 48, 1 }, { 54, 1 },
};
/* SVPP mmsys mux for the ACTIVE-RSZ scale pipe (geometry-independent); differs
 * from the bypassed-RSZ mdp_dt_mux -- this is what routes the resized data. */
static const struct { u32 reg, value, subsys_id; } mdp_scale_mux[7] = {
	{ 0xf1c, 0, 9 }, { 0xf24, 0, 9 }, { 0xf54, 0, 9 }, { 0xf2c, 1, 9 },
	{ 0xf18, 0, 9 }, { 0xf20, 0, 9 }, { 0xf28, 1, 9 },
};

static void mdp_config_compute_8195_scale(struct img_ipi_frameparam *param,
					  struct img_config *config)
{
	struct img_config_8195 *cfg = &config->config_8195;
	const struct img_image_buffer *in = &param->inputs[0].buffer;
	const struct img_image_buffer *out = &param->outputs[0].buffer;
	u32 iw = in->format.width,  ih = in->format.height;
	u32 ow = out->format.width, oh = out->format.height;
	u32 istride = in->format.plane_fmt[0].stride;
	u32 ostride = out->format.plane_fmt[0].stride;
	u32 iy = in->iova[0], ic = in->iova[1];
	u32 oy = out->iova[0], oc = out->iova[1];
	u32 hwi = (ih << 16) | iw, whi = (iw << 16) | ih;
	u32 hwo = (oh << 16) | ow, who = (ow << 16) | oh;
	/* RSZ phase step + PRZ_CONTROL_1 are PER-DIMENSION -- the horizontal coeff/mode
	 * follow the WIDTH direction (up_h) and the vertical the HEIGHT direction (up_v),
	 * so a MIXED scale (one axis up, the other down) programs each axis independently.
	 * Verbatim-matched to the SCP oracle (pure 2026-06-12; mixed 2026-06-29):
	 *  H/V DOWNSCALE: coeff = CEIL((out-1)<<20 / (in-1)) (ceil, NOT round -- confirmed
	 *    at 1280->640 where they differ; a 1-LSB error perturbs the 21-bit subpixel
	 *    polyphase weights -> content mismatch).
	 *  H/V UPSCALE:   coeff = ROUND((in-1)<<15 / (out-1)) (round-nearest + shift 15;
	 *    note the swapped numerator/denominator + different shift vs downscale).
	 * PRZ_CONTROL_1 is a SEPARABLE bitfield: the H-mode bits (RSZ_CTRL1_HMASK) and the
	 * V-mode bits (RSZ_CTRL1_VMASK) are disjoint, the rest direction-constant. The two
	 * mixed oracles confirm exactly: H-down/V-up 1280x360->640x720 -> 0x03710043, and
	 * H-up/V-down 640x720->1280x360 -> 0x023b0113 (= pure-down 0x02310143 / pure-up
	 * 0x037b0013 reassembled per axis). For a pure scale this reduces to the old
	 * single-direction value (no regression). */
#define RSZ_CTRL1_DOWN	0x02310143u
#define RSZ_CTRL1_UP	0x037b0013u
#define RSZ_CTRL1_HMASK	0x000a0050u	/* horizontal-mode bits (SCP-decoded, mixed oracle) */
#define RSZ_CTRL1_VMASK	0x01400100u	/* vertical-mode bits   (SCP-decoded, mixed oracle) */
	/* PER-DIMENSION direction. For a MIXED scale each axis takes its own strict
	 * direction. For a PURE scale (incl. a 1:1 axis paired with a scaling axis) keep
	 * the OLD lumped `up = (ow>iw)||(oh>ih)` so every previously-validated pure config
	 * is byte-identical (a 1:1 axis paired with an upscaling axis still programs the
	 * up coeff/mode, exactly as before -- no regression on the default-on paths). */
	bool pmix = (ow > iw && oh < ih) || (ow < iw && oh > ih);
	bool up_lump = (ow > iw) || (oh > ih);
	bool up_h = pmix ? (ow > iw) : up_lump;	/* WIDTH scale direction (drives cx + the H tile march) */
	bool up_v = pmix ? (oh > ih) : up_lump;	/* HEIGHT scale direction (drives cy + the V mode bits) */
	bool up = up_h;				/* the horizontal march is keyed off the WIDTH direction */
	u32 ctrl1 = (RSZ_CTRL1_DOWN & ~(RSZ_CTRL1_HMASK | RSZ_CTRL1_VMASK)) |
		    ((up_h ? RSZ_CTRL1_UP : RSZ_CTRL1_DOWN) & RSZ_CTRL1_HMASK) |
		    ((up_v ? RSZ_CTRL1_UP : RSZ_CTRL1_DOWN) & RSZ_CTRL1_VMASK);
	u32 cx = up_h ? (u32)DIV_ROUND_CLOSEST_ULL((u64)(iw - 1) << 15, ow - 1)
		      : (u32)DIV_ROUND_UP_ULL((u64)(ow - 1) << 20, iw - 1);
	u32 cy = up_v ? (u32)DIV_ROUND_CLOSEST_ULL((u64)(ih - 1) << 15, oh - 1)
		      : (u32)DIV_ROUND_UP_ULL((u64)(oh - 1) << 20, ih - 1);
	struct img_mmsys_ctrl *mc;
	int i;

	/* Multi-tile march (HORIZONTAL only; the RSZ line buffer bounds the width,
	 * height streams). OUTPUT-DRIVEN, reproducing the SCP tile boundaries (the
	 * load-bearing KI-009 fix): the prior INPUT-driven LBUF march
	 * (ir = il + LBUF-1, output = (ir-TAP)*coeff) emitted abutting outputs from a
	 * fixed-LBUF input window -- byte-different boundaries from the SCP. With
	 * abfd4256's 3 config fixes (AAL clip_ofst high-half, chroma==luma phase,
	 * window-sum totals) a per-subframe config-dump diff showed the ONLY remaining
	 * difference vs the COMPLETING SCP config was these tile boundaries. The SCP is
	 * OUTPUT-driven; reproduce it:
	 *
	 *  - Output tiles ABUT (out_left[k] = previous out_right + 1), each MDP_RSZ_OCAP
	 *    output columns wide, capped EARLY when the RSZ-input span would exceed the
	 *    line buffer (MDP_RSZ_LBUF). On-board SCP captures (1280->640, 640->320,
	 *    854->480, 1280->854; mtk-mdp-scp-boundaries.sh) show OCAP=278 with the
	 *    LBUF cap binding on tile 1 of 1280->640 (275 cols, not 278).
	 *  - RSZ-input left rsz_il: backed up MDP_RSZ_OV px from the previous tile's
	 *    input-right (even-aligned for the 4:2:0 chroma sampler). The overlap covers
	 *    the left filter footprint so the resample phase stays small (int 0..2,
	 *    matching the SCP). Tile0 starts at 0.
	 *  - RSZ-input right rsz_ir: forward-map of out_right (out_right*coeff>>20 is the
	 *    input sample column) widened by MDP_RSZ_RTAP for the right filter footprint,
	 *    clamped to the frame; the last tile runs to iw-1.
	 *  - RDMA read window [rdma_il..rdma_ir]: the RSZ window WIDENED by MDP_RSZ_TAP
	 *    on both edges (clamped to the frame). The input-side relays then CROP it
	 *    down to the RSZ window (the AAL crop-down, case 4).
	 *
	 * Per-tile RSZ PHASE (verified bit-exact vs the oracle): phase_q20 =
	 * (out_left << 20) - rsz_il * coeff. The boundaries need NOT equal the SCP's
	 * bit-for-bit -- only the phase must be exact, the input window a superset of
	 * the output's input footprint, and the outputs abutting+complete -> bit-
	 * identical output. This march is offline-verified (tests/mdp_scale_march.py)
	 * to reproduce the SCP boundaries within +-3 px and exact phase int across 14
	 * ratios, with full footprint coverage and rsz_ir-rsz_il+1 <= LBUF. The gate
	 * guarantees a pure down-scale. */
#define MDP_RSZ_LBUF	564u	/* RSZ input line-buffer max (SCP tile0 = in[0..563]) */
#define MDP_RSZ_OCAP	278u	/* max output columns per tile (SCP, all ratios) */
#define MDP_RSZ_OV	12u	/* RSZ-input overlap carried back from the previous tile */
#define MDP_RSZ_RTAP	10u	/* RSZ-input right-edge resample-tap margin over out_right */
#define MDP_RSZ_TAP	8u	/* RDMA read widened by this on each edge; relays crop it */
#define MDP_RSZ_MARGIN	2u	/* relay tile-loss added to the dpframework backward right edge */
/* UPSCALE multi-tile constants -- CMP-FITTED to the SCP oracle (2026-06-29, board
 * session; oracles ~/scp_up_1024.bin 512->1024 + ~/scp_up_1280.bin 640->1280).
 * OCAP_UP=512 is the >512 downstream/WROT output cap; <=512 outputs stay single-tile.
 * The input window is OUTPUT-DRIVEN (matching the per-tile phase): for an output tile
 * [ol..orr] the RSZ input window is
 *     il = ((ol*cx)>>15) - ULTAP, even-aligned  (interior tiles; tile0 il=0)
 *     ir = ((orr*cx)>>15) + UTAP                 (non-last tiles; last tile ir=iw-1)
 * with cx = ROUND((iw-1)<<15/(ow-1)) (shift-15). ULTAP=3 / UTAP=6 reproduce the SCP
 * tile boundaries bit-exactly across BOTH oracles (tile0 ir=261, tile1 il=252 for
 * 512->1024; tile1 ir=517/il=252, tile2 il=508 for 640->1280). */
#define MDP_RSZ_OCAP_UP	512u	/* max OUTPUT cols per upscale tile (>512 WROT cap) */
#define MDP_RSZ_UTAP	6u	/* CMP-FIT: upscale right-edge input forward-map tap margin (SCP-matched) */
#define MDP_RSZ_ULTAP	3u	/* CMP-FIT: upscale interior tile input left-edge tap back-off (SCP-matched) */
	u32 t_il[IMG_MAX_SUBFRAMES_8195], t_ir[IMG_MAX_SUBFRAMES_8195];	/* RSZ window */
	u32 t_dl[IMG_MAX_SUBFRAMES_8195], t_dr[IMG_MAX_SUBFRAMES_8195];	/* RDMA window */
	u32 t_ol[IMG_MAX_SUBFRAMES_8195], t_or[IMG_MAX_SUBFRAMES_8195];	/* output tile (WROT write/abutting right) */
	u32 t_ors[IMG_MAX_SUBFRAMES_8195];	/* RSZ/relay output right: the un-odd-forced resample extent, which OVERLAPS the next tile's first column by the cubic footprint (interior tiles). c5-c9 in/out + c10 READ use this; only the final WROT WRITE (c10 out) crops to the abutting t_or. (SCP oracle: c5..c9 sub1 out.r=552 totw=641; c10 out.r=551 totw=640.) */
	/* Per-tile RSZ horizontal phase, pre-split into integer + subpixel. The register
	 * fraction width differs by scale DIRECTION (SCP-confirmed): DOWNSCALE phase is q20
	 * ((ol<<20)-il*cx, cx q20 -> int=>>20, sub=&0xfffff); UPSCALE phase is q15
	 * (ol*cx-(il<<15), cx q15 -> int=>>15, sub=&0x7fff). Storing the split here keeps
	 * the shared relay/engine code direction-agnostic (the old single t_ph[] hard-coded
	 * a >>20/&0xfffff split that was correct for down but WRONG for the q15 up phase --
	 * the #1 cmp-fit bug). */
	u32 t_phi[IMG_MAX_SUBFRAMES_8195];	/* per-tile horizontal phase integer part */
	u32 t_phs[IMG_MAX_SUBFRAMES_8195];	/* per-tile horizontal phase subpixel part */
	u32 ntiles = 0, ox = 0, k;
	u32 prev_ir = 0;
	u32 sum_rdma = 0, sum_rszin = 0, sum_out = 0;

	if (up) {
	  /* UPSCALE multi-tile march (FIRST CUT -- shift-15 coeff geometry).
	   * Structurally correct (correct shift-15 back-map + output-cap split); the
	   * exact constants get CMP-FITTED to the SCP oracle (~/scp_up_1024.bin) in a
	   * later board session. The three knobs are: MDP_RSZ_OCAP_UP (output cap),
	   * MDP_RSZ_UTAP (right-edge tap), and the per-tile phase SHIFT (t_ph below) --
	   * the phase is the #1 thing the cmp will correct.
	   *
	   * Unlike downscale (INPUT LBUF-bounded, OCAP=278), upscale is bounded by the
	   * >512 downstream/WROT OUTPUT limit, so the OUTPUT splits into <=OCAP_UP-wide
	   * tiles. Per-tile input is BACK-MAPPED through the shift-15 coeff:
	   *   in(o) = (o * cx) >> 15      [cx = (iw-1)<<15 / (ow-1), shift 15]
	   * (THE BUG this fixes: the old up march reused the downscale shift-20 back-map
	   * div_u64(cand<<20, cx) against a shift-15 cx -- harmless single-tile because
	   * il=0/ir=iw-1, WRONG for multi-tile.) Tile0 input-left=0; interior tiles back
	   * off MDP_RSZ_OV (even-aligned for the 4:2:0 chroma sampler) from the previous
	   * tile's input-right. Right edge widened by MDP_RSZ_UTAP for the resample
	   * footprint, clamped to iw-1; the last tile runs to iw-1.
	   *
	   * Note: with OCAP_UP=512 a <=512-wide upscale output is ONE tile (il=0,
	   * ir=iw-1, ol=0, phase 0) -- byte-identical to the validated single-tile up
	   * path. Only ow>512 (gated behind mdp_kd_upscale) splits into >1 tile. */
	  /* Horizontal output-tile cap. PURE upscale (both axes up) uses the wide
	   * OCAP_UP=512 cap (SCP oracle 640->1280 = 3 tiles). MIXED H-up/V-down,
	   * however, caps the H output tile at the DOWNSCALE OCAP=278 (SCP oracle
	   * 640x720->1280x360 = 5 tiles of 278, NOT 3 of 512): a vertical downscale
	   * shrinks the shared 2D RSZ line buffer so the horizontal tile narrows. */
	  u32 ocap_h = up_v ? MDP_RSZ_OCAP_UP : MDP_RSZ_OCAP;
	  while (ox < ow && ntiles < IMG_MAX_SUBFRAMES_8195) {
		u32 ol = ox;
		u32 orr = min(ol + ocap_h - 1, ow - 1);
		u32 phase;
		/* OUTPUT-DRIVEN input-left: the shift-15 forward map of the tile's first
		 * output column, backed off ULTAP taps and even-aligned (4:2:0 chroma).
		 * Tile0 starts at 0. (SCP-matched: 512->1024 tile1 il=252; 640->1280
		 * tile1 il=252, tile2 il=508.) NOT the old prev_ir-overlap form, which
		 * mis-placed the window for non-2:1 ratios. */
		u32 il = (ntiles == 0) ? 0 :
			 ((((u32)(((u64)ol * cx) >> 15)) - MDP_RSZ_ULTAP) & ~1u);
		u32 ir, dl, dr;

		if (orr >= ow - 1) {		/* last tile: input runs to the edge */
			orr = ow - 1;
			ir = iw - 1;
		} else {
			/* shift-15 forward map of out_right -> input sample column,
			 * widened by UTAP for the right filter footprint, clamped.
			 * (SCP-matched: tile0 ir=261; 640->1280 tile1 ir=517.) */
			ir = (u32)(((u64)orr * cx) >> 15) + MDP_RSZ_UTAP;
			/* The RSZ input tile width (ir-il+1) MUST be EVEN for the 4:2:0
			 * chroma sampler -- an odd input span wedges the GCE (the chroma
			 * RDMA half-line never completes, SVPP1 frame-done parks). il is
			 * even (tile0=0, interior &~1), so force ir ODD. The SCP does the
			 * same: oracle 640x720->1280x360 ir = (forward-map+UTAP) | 1 exactly
			 * (tiles 145/283/423/561). For pure upscale the forward-map already
			 * landed odd (512->1024 / 640->1280 cmp=0), so this is a no-op there. */
			ir |= 1u;
			if (ir > iw - 1)
				ir = iw - 1;
		}
		if (ir - il + 1 > MDP_RSZ_LBUF)
			ir = (il + MDP_RSZ_LBUF - 1) | 1u;	/* keep the even-width invariant */

		/* RDMA read = RSZ window widened by TAP on both sides, clamped + even. */
		dl = (il > MDP_RSZ_TAP) ? ((il - MDP_RSZ_TAP) & ~1u) : 0;
		dr = (ir >= iw - 1) ? iw - 1 : min(ir + MDP_RSZ_TAP, iw - 1);

		t_il[ntiles] = il; t_ir[ntiles] = ir;
		t_dl[ntiles] = dl; t_dr[ntiles] = dr;
		t_ol[ntiles] = ol; t_or[ntiles] = orr;
		t_ors[ntiles] = orr;	/* upscale: output tiles abut, no resample overlap (SCP-matched) */
		/* CMP-FIT (the #1 corrected bug): per-tile RSZ phase is q15 for UPSCALE.
		 * cx is shift-15 (in-per-out), and the resampler is output-driven, so the
		 * input sample position of the tile's first output column, relative to il, is
		 *     phase_q15 = ol*cx - (il<<15)   (>=0 since il <= (ol*cx)>>15).
		 * int = phase>>15, sub = phase & 0x7fff (the register stores a 15-bit
		 * fraction, NOT the q20 the downscale path uses). SCP-confirmed exact:
		 * 512->1024 tile1 int=3 sub=0x6000; 640->1280 tile1 int=3 sub=0x6600,
		 * tile2 int=3 sub=0x4c00. Single-tile (il=0,ol=0) -> 0, so the validated
		 * single-tile up path is unaffected. */
		phase = (u32)((u64)ol * cx) - (il << 15);
		t_phi[ntiles] = phase >> 15;
		t_phs[ntiles] = phase & 0x7fffu;
		prev_ir = ir;
		ox = orr + 1;
		ntiles++;
	  }
	} else {
	  /* DOWNSCALE: dpframework MDP2.0 CUB_ACC 6N-tap tile split (the source of
	   * truth, the internal dpframework tile-split record).
	   * mt8195 deltas verified vs 4 on-board SCP oracles: 6N tap (constant 3,
	   * NOT mt2712's 4N/2), LBUF=564 (tile0) / 560 (interior), OCAP=278 output
	   * cap, OV=12 input overlap, +MDP_RSZ_MARGIN relay tile-loss on the
	   * backward right edge. Reproduces the SCP boundaries AND per-tile phase
	   * bit-exactly for 1280->640 (the 1280x720->640x360 validation ratio),
	   * 640->320, 854->480 (1280->854 within 2px on one interior tile). This
	   * REPLACES the prior <=3px inverse-map march -- the byte-faithful boundary
	   * is what closes the 232-byte SCP-vs-KD img_config residual (KI-009). */
	  while (ox < ow && ntiles < IMG_MAX_SUBFRAMES_8195) {
		u32 ol = ox;
		u32 il = (ntiles == 0) ? 0 :
			 (((prev_ir + 1 > MDP_RSZ_OV ? prev_ir + 1 - MDP_RSZ_OV : 0)) & ~1u);
		u32 orr_t = min(ol + MDP_RSZ_OCAP - 1, ow - 1);
		u32 lbufi = (il == 0) ? MDP_RSZ_LBUF : (MDP_RSZ_LBUF - 4);
		u32 ir, orr, ors, dl, dr, bend, foe, t;
		u64 thr;

		/* backward_cub_acc(orr_t).inE -- input-right needed to produce out=orr_t.
		 * thr = orr_t*2^20 + coeff + 3*2^20 (6N right tap); result odd-forced
		 * (inAlign==0). Clamp to the frame edge. */
		thr = ((u64)orr_t << 20) + cx + (3ULL << 20);
		if (thr >= (u64)(iw - 1) * cx) {
			bend = iw - 1;
		} else {
			t = (u32)div64_u64(thr, cx);
			if ((u64)t * cx == thr)
				t--;
			bend = (t & 1u) ? t : t + 1;
		}
		ir = min3(bend + MDP_RSZ_MARGIN, il + lbufi - 1, iw - 1);

		/* forward_cub_acc(il,ir).outE -- output-right from the RSZ input-right.
		 * outE = (ir*coeff>>20) - 3 (6N tap), odd-forced (outAlign==0). */
		if (ir >= iw - 1) {
			foe = ow - 1;
			ors = ow - 1;
		} else {
			t = (u32)((u64)ir * cx >> 20) - 3;
			foe = (t & 1u) ? t : t - 1;
			/* RSZ/relay output right = the un-odd-forced extent capped to
			 * OCAP; for interior tiles this is foe+1 (the resample footprint
			 * column the WROT later crops out). SCP oracle 1280->640 tile1:
			 * t=552 -> ors=552, foe=551 (=t_or). */
			ors = min(t, orr_t);
		}
		orr = min(foe, orr_t);
		if (orr >= ow - 1) {		/* last tile: input runs to the edge */
			orr = ow - 1;
			ir = iw - 1;
			ors = ow - 1;
		}

		/* RDMA read = RSZ window widened by TAP on both sides, clamped + even. */
		dl = (il > MDP_RSZ_TAP) ? ((il - MDP_RSZ_TAP) & ~1u) : 0;
		dr = (ir >= iw - 1) ? iw - 1 : min(ir + MDP_RSZ_TAP, iw - 1);

		t_il[ntiles] = il; t_ir[ntiles] = ir;
		t_dl[ntiles] = dl; t_dr[ntiles] = dr;
		t_ol[ntiles] = ol; t_or[ntiles] = orr;
		t_ors[ntiles] = ors;
		/* DOWNSCALE phase is q20 (cx is shift-20): int=>>20, sub=&0xfffff
		 * (unchanged from the validated downscale path). */
		t_phi[ntiles] = (((u32)ol << 20) - il * cx) >> 20;	/* EXACT phase vs rsz_il */
		t_phs[ntiles] = (((u32)ol << 20) - il * cx) & 0xfffffu;
		prev_ir = ir;
		ox = orr + 1;
		ntiles++;
	  }
	}

	memset(cfg, 0, sizeof(*cfg));
	cfg->num_components = 11;
	cfg->num_subfrms = ntiles;

	/* in/out_total_width = SUM of the per-tile window widths (NOT the plain frame
	 * width). The SCP carries the overlap-inclusive total per comp (oracle 1280:
	 * c0-c3 totw=1336=Σrdma_w, c4 in=1336/out=1304=Σrsz_in_w, c5 in=1304/out=641
	 * =Σout_w, c6-c10=641). The MERGE/pipe addressing reads this; a plain-width
	 * total mis-stitches the tiles. */
	for (k = 0; k < ntiles; k++) {
		sum_rdma  += t_dr[k] - t_dl[k] + 1;
		sum_rszin += t_ir[k] - t_il[k] + 1;
		sum_out   += t_ors[k] - t_ol[k] + 1;	/* overlap-inclusive (SCP c5..c9/c10-in totw=641) */
	}

	for (i = 0; i < 11; i++) {
		struct img_compparam_8195 *p = &cfg->components[i];
		struct img_comp_frame_8195 *f = &p->frame;
		struct img_comp_subfrm_8195 *s = &p->subfrms[0];
		u32 *u = (u32 *)&p->rdma;
		bool pre = (i <= 5);	/* component reads the INPUT-side tile (else output) */
		/* frame in = input for c0-c6, output for c7-c10; frame out = input for
		 * c0-c4, output for c5-c10. sub.in = input for c0-c5, output c6-c10;
		 * sub.out = input for c0-c4, output c5-c10 (verified vs the oracle). */
		u32 fiw = (i <= 6) ? iw : ow, fih = (i <= 6) ? ih : oh;
		u32 fow = (i <= 4) ? iw : ow, foh = (i <= 4) ? ih : oh;
		u32 siw = (i <= 5) ? iw : ow, sih = (i <= 5) ? ih : oh;
		u32 sow = (i <= 4) ? iw : ow, soh = (i <= 4) ? ih : oh;

		p->type = mdp_scale_comp[i].type;
		p->id = mdp_scale_comp[i].id;
		p->input = 0;
		p->num_outputs = 1;
		p->outputs[0] = 0;
		p->num_subfrms = ntiles;

		f->in_width = fiw; f->in_height = fih;
		f->out_width = fow; f->out_height = foh;
		/* totw per the SCP per-comp window-sum (oracle-matched). in side:
		 * c0-c3=Σrdma, c4=Σrdma, c5=Σrsz_in, c6-c10=Σout. out side: c0-c3=Σrdma,
		 * c4=Σrsz_in, c5=Σout, c6-c9=Σout, c10=ow (final write totw=ow). */
		f->in_total_width  = (i <= 4) ? sum_rdma  : (i == 5) ? sum_rszin : sum_out;
		f->out_total_width = (i <= 3) ? sum_rdma  : (i == 4) ? sum_rszin :
				     (i == 10) ? ow : sum_out;
		f->crop.width = siw; f->crop.height = sih;	/* c2/c3/c4/c7 override to 0 */

		/* Per-tile generic geometry -- THREE input-side windows now (the KI-009
		 * fix): the RDMA reads a WIDE span [rdma], the input-side relays carry that
		 * wide span, the AAL (c4) CROPS it down to the RSZ window [rsz] on its out,
		 * the RSZ (c5) takes the RSZ window in -> output tile out, and c6-c10 are
		 * output-side. Matching the SCP oracle exactly (c0..c3 in=out=rdma;
		 * c4 in=rdma,out=rsz; c5 in=rsz,out=output; c6.. in=out=output). luma/chroma
		 * phase is non-zero only on the RSZ. */
		for (k = 0; k < ntiles; k++) {
			struct img_comp_subfrm_8195 *sk = &p->subfrms[k];
			u32 dl = t_dl[k], dr = t_dr[k];		/* RDMA wide read window */
			u32 rl = t_il[k], rr = t_ir[k];		/* RSZ input window (cropped) */
			u32 otl = t_ol[k], otr = t_or[k];	/* output tile (WROT write right) */
			u32 otr_r = t_ors[k];			/* RSZ/relay output right (overlap) */
			u32 inl, inr, oul, our;
			u32 ih_in, ih_out;

			/* sub.in: c0-c4 read the wide RDMA span; c5 the cropped RSZ span;
			 * c6-c10 the OUTPUT tile, reading the overlap-inclusive right
			 * (t_ors) -- they consume what the RSZ produced, including the
			 * 1px resample-footprint column the WROT later crops. */
			if (i <= 4)      { inl = dl;  inr = dr;    ih_in = ih; }
			else if (i == 5) { inl = rl;  inr = rr;    ih_in = ih; }
			else             { inl = otl; inr = otr_r; ih_in = oh; }
			/* sub.out: c0-c3 pass the wide span; c4 crops to the RSZ window;
			 * c5-c9 emit the overlap-inclusive output tile (t_ors); only the
			 * final WROT (c10) writes the abutting, cropped tile (t_or). */
			if (i <= 3)       { oul = dl;  our = dr;    ih_out = ih; }
			else if (i == 4)  { oul = rl;  our = rr;    ih_out = ih; }
			else if (i == 10) { oul = otl; our = otr;   ih_out = oh; }
			else              { oul = otl; our = otr_r; ih_out = oh; }

			sk->in.left  = inl; sk->in.right  = inr;
			sk->in.top = 0; sk->in.bottom = ih_in - 1;
			sk->out.left = oul; sk->out.right = our;
			sk->out.top = 0; sk->out.bottom = ih_out - 1;
			/* SCP encodes tile index as out_horizontal=(k+1)%n, out_vertical
			 * =(last)?1:0 (oracle: 3-tile oh1/oh2/oh0 ov0/ov0/ov1). The kernel
			 * GCE exec path does NOT read these (debug-only), but match the SCP
			 * so the config is byte-faithful. */
			sk->out_horizontal = (k + 1 == ntiles) ? 0 : (int)(k + 1);
			sk->out_vertical = (k + 1 == ntiles) ? 1 : 0;
			if (i == 5) {			/* RSZ carries the resample phase */
				sk->luma.left = t_phi[k];
				sk->luma.left_subpix = t_phs[k];
				/* SCP keeps chroma phase == luma phase here (oracle: c5
				 * sub1 luma l2/225768 chroma l2/225768 -- NOT halved). The
				 * prior >>1 was wrong: the RSZ addresses both planes in the
				 * same output-pixel phase for 4:2:0 horizontal resample. */
				sk->chroma.left = t_phi[k];
				sk->chroma.left_subpix = t_phs[k];
			}
		}
		if (ntiles == 1) {
			/* Single-tile: fill subfrms[0]'s generic region with the
			 * full-frame dims. For ntiles>1 the per-tile loop above
			 * already populated subfrms[0..ntiles-1] correctly -- this
			 * leftover would OVERWRITE tile 0 with full-frame width,
			 * corrupting the multi-tile march (tile 1 then wedges on the
			 * SVPP1 RDMA frame-done that never arrives). */
			s->in.right = siw - 1; s->in.bottom = sih - 1;
			s->out.right = sow - 1; s->out.bottom = soh - 1;
		}

		switch (i) {
		case 0:	/* RDMA: reads NV12 at the input size (src_ctrl=0x60c). */
			u[0] = 0x60c; u[2] = 0x1010; u[3] = iy; u[4] = ic;
			u[6] = iy + istride * ih; u[7] = ic + istride * ih / 2;
			u[9] = istride; u[11] = istride;
			u[15] = 0x03000020; u[16] = 0xe800; u[17] = 0x9c00;
			u[18] = 0x01000014; u[19] = 0x7400; u[20] = 0x4c00;
			u[29] = hwi; u[30] = hwi;
			break;
		case 1:	u[0] = 1; u[1] = 7; u[2] = (iw << 16); u[3] = (ih << 16); break;
		case 2:	f->in_total_width = 0; f->out_total_width = 0;
			f->crop.width = 0; f->crop.height = 0;
			/* SCP zeros the in/out region of EVERY subframe (not just [0]);
			 * the generic geometry loop above populated subfrms[1..] with the
			 * RDMA window, so clear all of them to match (oracle: c2 sub0..2
			 * in=out=[0..0]). out_horizontal/out_vertical stay (SCP keeps them). */
			for (k = 0; k < ntiles; k++) {
				memset(&p->subfrms[k].in, 0, sizeof(p->subfrms[k].in));
				memset(&p->subfrms[k].out, 0, sizeof(p->subfrms[k].out));
			}
			break;
		case 3:	f->crop.width = 0; f->crop.height = 0;
			u[0] = 0x10000000; u[1] = 1; u[3] = hwi;
			u[4] = (iw - 1) << 16; u[5] = (ih - 1) << 16;
			u[7] = iw - 1; u[8] = 0x60; u[9] = 0x200;
			break;
		case 4:	f->crop.width = 0; f->crop.height = 0;
			u[0] = 0x80; u[1] = 1; u[2] = whi; u[3] = whi; break;
		case 5:	/* RSZ: iw x ih -> ow x oh, ACTIVE (control1 per up/down). */
			u[0] = cx; u[1] = cy; u[2] = ctrl1;
			u[15] = 0x1000; u[16] = hwi; u[17] = hwo;
			break;
		case 6:	f->bypass = 1;
			u[0] = 1; u[1] = who; u[2] = who;
			u[4] = (ow - 1) << 16; u[5] = (oh - 1) << 16;
			break;
		case 7:	f->crop.width = 0; f->crop.height = 0;
			u[0] = 3; u[1] = ow; u[2] = oh; break;
		case 8:	u[0] = 0x30000000; u[1] = 1; u[2] = hwo; u[3] = hwo; break;
		case 9:	u[0] = ((oh - 1) << 16) | ow; break;
		case 10: /* WROT: writes NV12 at the output size. */
			u[0] = oy; u[1] = oc; u[3] = 0x500c;
			u[4] = ostride; u[5] = ostride; u[6] = ostride;
			u[8] = 0x200; u[10] = 0x000d80c4;
			u[21] = hwo; u[22] = hwo; u[24] = (ow << 16) | 0x4000;
			break;
		}

		/* Per-tile engine-subframe HW-register geometry. TWO widths now (the
		 * KI-009 fix): the RDMA + input-side relays (FG c1, HDR c3) are sized to
		 * the WIDE read [rdma]; the AAL (c4) CROPS wide->RSZ; the RSZ (c5) takes
		 * the narrow RSZ span -> output tile with the exact phase; the WROT (c10)
		 * writes the output tile. Field order is transcribed PER ENGINE from the
		 * SCP oracle (RDMA/HDR/RSZ/WROT = (h<<16)|w; AAL = (w<<16)|h). The relay
		 * size regs were ZERO in the prior draft -- a 0-size relay never passes
		 * data, so the RSZ frame-done never fired -> the SVPP1-RDMA WFE parked. */
		(void)pre;
		for (k = 0; k < ntiles; k++) {
			u32 dl = t_dl[k], dr = t_dr[k];		/* RDMA wide read window */
			u32 rl = t_il[k], rr = t_ir[k];		/* RSZ input window (cropped) */
			u32 otl = t_ol[k], otr = t_or[k];	/* output tile (WROT write right) */
			u32 dtw = dr - dl + 1;			/* RDMA tile width (wide) */
			u32 rtw = rr - rl + 1;			/* RSZ input tile width */
			u32 otw = otr - otl + 1;		/* output tile WRITE width (WROT) */
			u32 otw_r = t_ors[k] - otl + 1;		/* output tile RELAY width (RSZ-out, overlap) */
			u32 crop = rl - dl;			/* left crop: rsz_il - rdma_il */

			switch (i) {
			case 0:	/* RDMA: read the WIDE input tile, full height (@ col rdma_il) */
				p->rdma.subfrms[k].offset[0] = dl;
				p->rdma.subfrms[k].offset[1] = dl;
				p->rdma.subfrms[k].offset[2] = dl;	/* SCP sets the unused 3rd-plane offset = dl too */
				p->rdma.subfrms[k].src  = (ih << 16) | dtw;	/* MF_SRC_SIZE */
				p->rdma.subfrms[k].clip = (ih << 16) | dtw;	/* MF_CLIP_SIZE */
				break;
			case 1:	/* FG relay: pass the WIDE tile. info_0 high = tile RIGHT+1
				 * (absolute boundary, NOT the width) per the SCP oracle
				 * (tile1 dr=1119 -> 1120, not dtw=576). */
				p->fg.subfrms[k].info_0 = (dr + 1) << 16;
				p->fg.subfrms[k].info_1 = ih << 16;
				break;
			case 3:	/* HDR relay: full per-tile MDP_HDR window (SCP oracle). win_size
				 * = rdma left; clip_ofst0 high = tile width-1; clip_ofst1 high =
				 * ih-1; hist_ctrl_0 = overlap with the PREVIOUS tile (0 on tile0);
				 * hist_ctrl_1 = tile width-1; hdr_top = first(0x20)|last(0x40). */
				p->hdr.subfrms[k].win_size   = dl;
				p->hdr.subfrms[k].src        = (ih << 16) | dtw;
				p->hdr.subfrms[k].clip_ofst0 = (dtw - 1) << 16;
				p->hdr.subfrms[k].clip_ofst1 = (ih - 1) << 16;
				p->hdr.subfrms[k].hist_ctrl_0 =
					(k == 0) ? 0 : (t_dr[k - 1] - dl + 1);
				p->hdr.subfrms[k].hist_ctrl_1 = dtw - 1;
				p->hdr.subfrms[k].hdr_top =
					((k == 0) ? 0x20 : 0) |
					((k + 1 == ntiles) ? 0x40 : 0);
				break;
			case 4:	/* AAL: CROP the wide read [rdma] down to the RSZ window [rsz].
				 * Field order is (w<<16)|h here (per the oracle, NOT (h<<16)|w).
				 * clip_ofst = the AAL's MDP_AAL_OUTPUT_OFFSET (mask 0x00FF00FF).
				 * The SCP puts the horizontal crop in the HIGH half (oracle: tile1
				 * crop=8 -> 0x00080000, NOT 0x00000008). The prior low-half value
				 * cropped the WRONG axis -> the RSZ saw mis-positioned data. */
				p->aal.subfrms[k].src  = (dtw << 16) | ih;	/* MDP_AAL_SIZE (wide in) */
				p->aal.subfrms[k].clip = (rtw << 16) | ih;	/* MDP_AAL_OUTPUT_SIZE (narrow out) */
				p->aal.subfrms[k].clip_ofst = (crop & 0xff) << 16;
				break;
			case 5:	/* RSZ: narrow input tile -> output tile + resample phase.
				 * control2 bit 0x2000 = "more tiles follow" (set on every tile
				 * except the LAST). DIRECTION-dependent for the single-tile case
				 * (SCP-confirmed): DOWNSCALE single-tile -> 0x3000 (bit set);
				 * UPSCALE single-tile -> 0x1000 (bit clear -- the single tile IS
				 * the last, oracle 256->512). Multi-tile is identical both ways
				 * (last tile 0x1000, others 0x3000: up 512->1024/640->1280, down
				 * 1280->640 all cmp=0). PRZ_OUTPUT width = the overlap-inclusive
				 * relay width otw_r (the RSZ genuinely emits the footprint column). */
				p->rsz.subfrms[k].control2 =
					((up ? (k + 1 == ntiles)
					     : (ntiles > 1 && k + 1 == ntiles))
					 ? 0x1000 : 0x3000);
				p->rsz.subfrms[k].src  = (ih << 16) | rtw;	/* PRZ_INPUT (narrow) */
				p->rsz.subfrms[k].clip = (oh << 16) | otw_r;	/* PRZ_OUTPUT (overlap) */
				p->rsz.subfrms[k].luma_h_int_ofst = t_phi[k];
				p->rsz.subfrms[k].luma_h_sub_ofst = t_phs[k];
				/* chroma phase == luma phase (SCP oracle: NOT halved) */
				p->rsz.subfrms[k].chroma_h_int_ofst = t_phi[k];
				p->rsz.subfrms[k].chroma_h_sub_ofst = t_phs[k];
				break;
			case 6:	/* TDSHP relay (output side): per-tile size = overlap relay
				 * width. src=clip=(otw_r<<16)|oh; hist_cfg_0 = (otw_r-1)<<16
				 * | (last?1:0); hist_cfg_1 = (oh-1)<<16. */
				p->tdshp.subfrms[k].src  = (otw_r << 16) | oh;
				p->tdshp.subfrms[k].clip = (otw_r << 16) | oh;
				/* hist_cfg_0 low bit = last-tile flag. SCP-confirmed
				 * DIRECTION-dependent: downscale sets it on the last tile
				 * (oracle 1280->640 tile2 = 0x570001); UPSCALE never sets it
				 * (both oracles: all tiles low-bit 0). */
				p->tdshp.subfrms[k].hist_cfg_0 =
					((otw_r - 1) << 16) |
					((!up && k + 1 == ntiles) ? 1 : 0);
				p->tdshp.subfrms[k].hist_cfg_1 = (oh - 1) << 16;
				break;
			case 7:	/* COLOR relay: in_hsize = overlap relay width, in_vsize = oh */
				p->color.subfrms[k].in_hsize = otw_r;
				p->color.subfrms[k].in_vsize = oh;
				break;
			case 8:	/* OVL relay: L0_src_size = roi_size = (oh<<16)|otw_r */
				p->ovl.subfrms[k].L0_src_size = (oh << 16) | otw_r;
				p->ovl.subfrms[k].roi_size    = (oh << 16) | otw_r;
				break;
			case 9:	/* PAD relay: pic_size = ((oh-1)<<16)|otw_r */
				p->pad.subfrms[k].pic_size = ((oh - 1) << 16) | otw_r;
				break;
			case 10: /* WROT: write output tile, full height (NV12 @ col otl).
				 * VIDO_IN_SIZE (src) READS the overlap-inclusive relay width
				 * otw_r; VIDO_TAR_SIZE (clip) + main_buf WRITE the abutting
				 * cropped width otw. offset[2] = otl (unused-plane, SCP sets it). */
				p->wrot.subfrms[k].offset[0] = otl;
				p->wrot.subfrms[k].offset[1] = otl;
				p->wrot.subfrms[k].offset[2] = otl;
				p->wrot.subfrms[k].src  = (oh << 16) | otw_r;	/* VIDO_IN_SIZE (overlap read) */
				p->wrot.subfrms[k].clip = (oh << 16) | otw;	/* VIDO_TAR_SIZE (write) */
				p->wrot.subfrms[k].main_buf = (otw << 16) | 0x4000;
				break;
			}
		}
	}

	for (k = 0; k < ntiles; k++) {
		mc = &cfg->ctrls[k];
		mc->num_sets = 7;
		for (i = 0; i < 7; i++) {
			mc->sets[i].reg = mdp_scale_mux[i].reg;
			mc->sets[i].value = mdp_scale_mux[i].value;
			mc->sets[i].subsys_id = mdp_scale_mux[i].subsys_id;
		}
	}

	if (mdp_kd_debug_on()) {
		struct mdp_dev *mdp = vpu_to_mdp((struct mdp_vpu_dev *)
						 (unsigned long)param->drv_data);
		struct device *dev = &mdp->pdev->dev;

		dev_info(dev, "MDPKD-8195-SCALE %ux%u->%ux%u: ncomp=11 coeff=[%#x,%#x] src=%#x clip=%#x\n",
			 iw, ih, ow, oh, cx, cy, hwi, hwo);

		/* Per-tile UPSCALE geometry dump for the later cmp-fit session: the
		 * march's il/ir/ol/orr/phase for each tile (the geometry the cmp
		 * compares against the SCP oracle ~/scp_up_1024.bin). */
		if (up) {
			dev_info(dev, "MDPKD-8195-UPSCALE %ux%u->%ux%u ntiles=%u cx=%#x cy=%#x (CMP-FIT q15 phase)\n",
				 iw, ih, ow, oh, ntiles, cx, cy);
			for (k = 0; k < ntiles; k++)
				dev_info(dev, "MDPKD-8195-UPSCALE tile%u: il=%u ir=%u dl=%u dr=%u ol=%u or=%u ors=%u ph(int=%u sub=%#x)\n",
					 k, t_il[k], t_ir[k], t_dl[k], t_dr[k],
					 t_ol[k], t_or[k], t_ors[k],
					 t_phi[k], t_phs[k]);
		}
	}
}

/*
 * Kernel-direct 1:1 CROP compute (mt8195). Reproduces the SCP's crop-via-RSZ
 * config BYTE-EXACT (so the GCE output is bit-identical to the SCP, KI-009). A
 * 1:1 crop (crop W/H == out W/H) is NOT a pixel-exact ROI on this HW -- the SCP
 * resamples the crop rect through the RSZ (cx=cy=0x8000 q15-unity,
 * control1=0x037b0003) with the crop origin folded into the per-axis phase. The
 * geometry (transcribed + dpframework-derived from on-board SCP oracles, cases
 * crop(64,64,128,128)/(32,96,..)/(66,66,..)/(128,64,256,256), all even offsets):
 *
 *   intermediate frame  fw = cl+cw,  fh = ct+ch        (the crop far edge)
 *   RSZ input window     rsz_l = cl-4,  rsz_r = cl+cw-1 (== rdma_r; no right tap)
 *                        rsz_t = ct-3,  rsz_b = ct+ch-1 (== rdma_b)
 *   RDMA read window     rdma_l = cl-12 (= rsz_l-8, the AAL crops it back),
 *                        rdma_r = cl+cw-1,
 *                        rdma_t = ct-4  (= rsz_t-1, even; c0 clips 1 row down),
 *                        rdma_b = ct+ch-1
 *   per-axis RSZ phase   H int 4 (= cl-rsz_l), V int 3 (= ct-rsz_t), sub 0
 *
 * (The dpframework backward_6_taps over the crop-as-input-frame -- inMaxEnd =
 * crop_right, cropOffset = cl, 6-tap -3 with even-force on the horizontal /
 * inAlignment=1 vertical -- yields rsz_l = even(cl-3) = cl-4 (phase 4) and rsz_t =
 * cl_v-3 (phase 3); the right/bottom clamp to the crop far edge.) Only c0-c5 carry
 * the crop windows; c6-c10 are the output-side (ow x oh) single tile, byte-
 * identical to a single-tile scale. Gated by mdp_frame_is_nv12_crop + mdp_kd_crop.
 *
 * CROP+SCALE (cw!=ow / ch!=oh, behind mdp_kd_cropscale): the SAME machinery, but
 * the RSZ resamples the crop rect to the output -- the scale coeff is keyed off the
 * CROP dims (cw->ow, ch->oh), control1 picks the down/up RSZ mode, and the crop
 * sub-pixel origin folds into the RSZ start phase. The output (c6-c10) is ow x oh,
 * already parameterized. Reconciled bit-exact vs the SCP CFGHEX oracle (2026-06-30):
 *   DOWN coeff = CEIL((ow-1)<<20/(cw-1)); control1 0x02310143; RSZ-input back-margin
 *     mh (even) / mv (odd) = smallest making the q20 start phase reach integer 2;
 *     start phase q20 = margin*coeff (so output 0 samples the crop origin regardless
 *     of rsz_l). Verified crop(64,64,128,128)->64^2, (64,64,256,256)->128^2,
 *     (66,66,128,128)->96^2 (luma_h/v int+sub byte-exact).
 *   UP   coeff = ROUND((cw-1)<<15/(ow-1)); control1 0x037b0013; window = the 1:1
 *     crop window (mh4 mv3); start phase = integer margin, sub 0 (output 0 samples
 *     the crop origin exactly). Verified crop(64,64,64,64)->128^2, (64,64,128,128)->256^2.
 *   MIXED (one axis up, one down) routes to the SCP (not yet ported).
 */
static void mdp_crop_scale_params(u32 cw, u32 ch, u32 ow, u32 oh,
				  u32 *cx, u32 *cy, u32 *mh, u32 *mv)
{
	bool up_h = (ow >= cw), up_v = (oh >= ch);

	*cx = up_h ? (u32)DIV_ROUND_CLOSEST_ULL((u64)(cw - 1) << 15, ow - 1)
		   : (u32)DIV_ROUND_UP_ULL((u64)(ow - 1) << 20, cw - 1);
	*cy = up_v ? (u32)DIV_ROUND_CLOSEST_ULL((u64)(ch - 1) << 15, oh - 1)
		   : (u32)DIV_ROUND_UP_ULL((u64)(oh - 1) << 20, ch - 1);
	*mh = up_h ? 4 : (u32)DIV_ROUND_UP_ULL(2ull << 20, *cx);
	*mv = up_v ? 3 : (u32)DIV_ROUND_UP_ULL(2ull << 20, *cy);
	if (!up_h && (*mh & 1))
		(*mh)++;		/* even-align rsz_l (4:2:0 chroma sampler) */
	if (!up_v && !(*mv & 1))
		(*mv)++;		/* rsz_t odd (SCP-matched) */
}

static void mdp_config_compute_8195_crop(struct img_ipi_frameparam *param,
					 struct img_config *config)
{
	struct img_config_8195 *cfg = &config->config_8195;
	const struct img_image_buffer *in = &param->inputs[0].buffer;
	const struct img_image_buffer *out = &param->outputs[0].buffer;
	const struct img_crop *cr = &param->outputs[0].crop;
	u32 iw = in->format.width,  ih = in->format.height;
	u32 ow = out->format.width, oh = out->format.height;
	u32 istride = in->format.plane_fmt[0].stride;
	u32 ostride = out->format.plane_fmt[0].stride;
	u32 iy = in->iova[0], ic = in->iova[1];
	u32 oy = out->iova[0], oc = out->iova[1];
	u32 cl = cr->left, ct = cr->top, cw = cr->width, ch = cr->height;
	u32 fw = cl + cw, fh = ct + ch;			/* intermediate frame (far edge) */
	/* per-axis scale coeff + RSZ-input back-margin (1:1 -> 0x8000/mh4/mv3) */
	u32 cx, cy, mh, mv;
	bool up_h = (ow >= cw), up_v = (oh >= ch);
	/* control1: 1:1 crop = unity 0x037b0003; else SEPARABLE per-axis (H bits
	 * 0x000a0050, V bits 0x01400100; SCP-decoded, same as the mixed scale path)
	 * so a MIXED crop+scale (one axis up, the other down) programs each axis
	 * independently (oracle H-down/V-up 0x03710043, H-up/V-down 0x023b0113 =
	 * pure-down 0x02310143 / pure-up 0x037b0013 reassembled per axis). For a pure
	 * crop+scale this reduces to the old single-direction value (no regression). */
	u32 ctrl1 = (cw == ow && ch == oh) ? 0x037b0003u :
		    ((0x02310143u & ~0x014a0150u) |
		     ((up_h ? 0x037b0013u : 0x02310143u) & 0x000a0050u) |
		     ((up_v ? 0x037b0013u : 0x02310143u) & 0x01400100u));
	u32 rsz_l, rsz_r = cl + cw - 1;			/* RSZ input window */
	u32 rsz_t, rsz_b = ct + ch - 1;
	u32 rdma_l, rdma_r = cl + cw - 1;		/* RDMA read window */
	u32 rdma_t, rdma_b = ct + ch - 1;
	u32 rdma_w, rdma_h, rsz_w, rsz_h;
	u32 vcrop, hcrop;				/* RDMA v sub-crop (1) / AAL h crop (8) */
	/* RSZ start phase, pre-split int+sub. DOWN: q20 = margin*coeff. UP/unity:
	 * integer margin, sub 0 (output 0 samples the crop origin exactly). */
	u32 ph_h, ph_v, phase_h, phase_h_sub, phase_v, phase_v_sub;
	u32 hwo = (oh << 16) | ow, who = (ow << 16) | oh;
	struct img_mmsys_ctrl *mc;
	int i;

	mdp_crop_scale_params(cw, ch, ow, oh, &cx, &cy, &mh, &mv);
	rsz_l = cl - mh; rsz_t = ct - mv;
	rdma_l = rsz_l - 8; rdma_t = rsz_t - 1;
	rdma_w = rdma_r - rdma_l + 1; rdma_h = rdma_b - rdma_t + 1;
	rsz_w = rsz_r - rsz_l + 1;   rsz_h = rsz_b - rsz_t + 1;
	vcrop = rsz_t - rdma_t;		/* 1 */
	hcrop = rsz_l - rdma_l;		/* 8 */
	ph_h = up_h ? (mh << 20) : (mh * cx);
	ph_v = up_v ? (mv << 20) : (mv * cy);
	phase_h = ph_h >> 20; phase_h_sub = ph_h & 0xfffff;
	phase_v = ph_v >> 20; phase_v_sub = ph_v & 0xfffff;

	memset(cfg, 0, sizeof(*cfg));
	cfg->num_components = 11;
	cfg->num_subfrms = 1;

	for (i = 0; i < 11; i++) {
		struct img_compparam_8195 *p = &cfg->components[i];
		struct img_comp_frame_8195 *f = &p->frame;
		struct img_comp_subfrm_8195 *s = &p->subfrms[0];
		u32 *u = (u32 *)&p->rdma;
		/* frame in/out dims per the SCP oracle:
		 *   in : c0-c2 = (iw,ih);  c3-c6 = (fw,fh);  c7-c10 = (ow,oh)
		 *   out: c0-c4 = (fw,fh);  c5-c10 = (ow,oh) */
		u32 fiw = (i <= 2) ? iw : (i <= 6) ? fw : ow;
		u32 fih = (i <= 2) ? ih : (i <= 6) ? fh : oh;
		u32 fow = (i <= 4) ? fw : ow;
		u32 foh = (i <= 4) ? fh : oh;
		/* sub.in / sub.out windows */
		u32 inl, inr, intp, inb, oul, our, outp, oub;

		p->type = mdp_scale_comp[i].type;
		p->id = mdp_scale_comp[i].id;
		p->input = 0;
		p->num_outputs = 1;
		p->outputs[0] = 0;
		p->num_subfrms = 1;

		f->in_width = fiw; f->in_height = fih;
		f->out_width = fow; f->out_height = foh;

		/* in/out total width (single tile -- the per-comp window width) */
		f->in_total_width  = (i <= 3) ? rdma_w : (i == 4) ? rdma_w :
				     (i == 5) ? rsz_w : ow;
		f->out_total_width = (i <= 3) ? rdma_w : (i == 4) ? rsz_w : ow;
		if (i == 2) { f->in_total_width = 0; f->out_total_width = 0; }

		/* frame.crop (SCP oracle): c0 rdma rect, c1 rdma rect@rsz_top,
		 * c5 rsz rect, c6/c8/c9/c10 (0,0,ow,oh); c2/c3/c4/c7 = 0 */
		switch (i) {
		case 0:
			f->crop.left = rdma_l; f->crop.top = rdma_t;
			f->crop.width = rdma_w; f->crop.height = rdma_h;
			break;
		case 1:
			f->crop.left = rdma_l; f->crop.top = rsz_t;
			f->crop.width = rdma_w; f->crop.height = rsz_h;
			break;
		case 5:
			f->crop.left = rsz_l; f->crop.top = rsz_t;
			f->crop.width = rsz_w; f->crop.height = rsz_h;
			break;
		case 6: case 8: case 9: case 10:
			f->crop.width = ow; f->crop.height = oh;
			break;
		}

		/* sub.in / sub.out windows (single subframe) */
		switch (i) {
		case 0:	inl = rdma_l; inr = rdma_r; intp = rdma_t; inb = rdma_b;
			oul = rdma_l; our = rdma_r; outp = rsz_t;  oub = rdma_b;
			break;
		case 1: case 3:
			inl = rdma_l; inr = rdma_r; intp = rsz_t; inb = rdma_b;
			oul = rdma_l; our = rdma_r; outp = rsz_t; oub = rdma_b;
			break;
		case 4:	inl = rdma_l; inr = rdma_r; intp = rsz_t; inb = rdma_b;
			oul = rsz_l;  our = rdma_r; outp = rsz_t; oub = rdma_b;
			break;
		case 5:	inl = rsz_l; inr = rsz_r; intp = rsz_t; inb = rsz_b;
			oul = 0; our = ow - 1; outp = 0; oub = oh - 1;
			break;
		case 2:	inl = inr = intp = inb = 0; oul = our = outp = oub = 0;
			break;
		default: /* c6-c10 output side */
			inl = 0; inr = ow - 1; intp = 0; inb = oh - 1;
			oul = 0; our = ow - 1; outp = 0; oub = oh - 1;
			break;
		}
		s->in.left = inl; s->in.right = inr; s->in.top = intp; s->in.bottom = inb;
		s->out.left = oul; s->out.right = our; s->out.top = outp; s->out.bottom = oub;
		if (i == 0)
			s->luma.top = vcrop;	/* c0 RDMA in-tile vertical crop */
		if (i == 5) {
			s->luma.left = phase_h;  s->luma.top = phase_v;
			s->luma.left_subpix = phase_h_sub;
			s->luma.top_subpix = phase_v_sub;
			s->chroma.left = phase_h; s->chroma.top = phase_v;
			s->chroma.left_subpix = phase_h_sub;
			s->chroma.top_subpix = phase_v_sub;
		}
		s->out_vertical = 1;		/* single tile (SCP: ov1 oh0) */
		s->out_horizontal = 0;

		switch (i) {
		case 0:	/* RDMA: read NV12 sub-region at (rdma_l, rdma_t) */
			u[0] = 0x60c; u[2] = 0x1010; u[3] = iy; u[4] = ic;
			u[6] = iy + istride * ih; u[7] = ic + istride * ih / 2;
			u[9] = istride; u[11] = istride;
			u[15] = 0x03000020; u[16] = 0xe800; u[17] = 0x9c00;
			u[18] = 0x01000014; u[19] = 0x7400; u[20] = 0x4c00;
			p->rdma.subfrms[0].offset[0] = rdma_t * istride + rdma_l;
			p->rdma.subfrms[0].offset[1] = (rdma_t / 2) * istride + rdma_l;
			p->rdma.subfrms[0].offset[2] = p->rdma.subfrms[0].offset[1];
			p->rdma.subfrms[0].src  = (rdma_h << 16) | rdma_w;
			p->rdma.subfrms[0].clip = (rsz_h << 16) | rdma_w;
			p->rdma.subfrms[0].clip_ofst = vcrop << 16;
			break;
		case 1:	/* FG relay */
			u[0] = 1; u[1] = 7;
			p->fg.subfrms[0].info_0 = (rdma_r + 1) << 16;
			p->fg.subfrms[0].info_1 = fh << 16;
			break;
		case 2:	/* disabled */
			break;
		case 3:	/* HDR relay */
			u[0] = 0x10000000; u[1] = 1;
			p->hdr.subfrms[0].win_size   = (rsz_t << 16) | rdma_l;
			p->hdr.subfrms[0].src        = (rsz_h << 16) | rdma_w;
			p->hdr.subfrms[0].clip_ofst0 = (rdma_w - 1) << 16;
			p->hdr.subfrms[0].clip_ofst1 = (rsz_h - 1) << 16;
			p->hdr.subfrms[0].hist_ctrl_0 = 0;
			p->hdr.subfrms[0].hist_ctrl_1 = rdma_w - 1;
			p->hdr.subfrms[0].hdr_top = 0x60;	/* first | last */
			p->hdr.subfrms[0].hist_addr = 0x200;
			break;
		case 4:	/* AAL: crop the wide RDMA span down to the RSZ window */
			u[0] = 0x80; u[1] = 1;
			p->aal.subfrms[0].src  = (rdma_w << 16) | rsz_h;
			p->aal.subfrms[0].clip = (rsz_w << 16) | rsz_h;
			p->aal.subfrms[0].clip_ofst = (hcrop & 0xff) << 16;
			break;
		case 5:	/* RSZ: unity cubic resample, crop origin -> per-axis phase */
			u[0] = cx; u[1] = cy; u[2] = ctrl1;
			p->rsz.subfrms[0].control2 = 0x1000;
			p->rsz.subfrms[0].src  = (rsz_h << 16) | rsz_w;
			p->rsz.subfrms[0].clip = (oh << 16) | ow;
			p->rsz.subfrms[0].luma_h_int_ofst = phase_h;
			p->rsz.subfrms[0].luma_h_sub_ofst = phase_h_sub;
			p->rsz.subfrms[0].luma_v_int_ofst = phase_v;
			p->rsz.subfrms[0].luma_v_sub_ofst = phase_v_sub;
			p->rsz.subfrms[0].chroma_h_int_ofst = phase_h;
			p->rsz.subfrms[0].chroma_h_sub_ofst = phase_h_sub;
			break;
		case 6:	/* TDSHP relay (output side, bypass) */
			f->bypass = 1;
			u[0] = 1; u[1] = who; u[2] = who;
			u[4] = (ow - 1) << 16; u[5] = (oh - 1) << 16;
			p->tdshp.subfrms[0].src  = (ow << 16) | oh;
			p->tdshp.subfrms[0].clip = (ow << 16) | oh;
			p->tdshp.subfrms[0].hist_cfg_0 = (ow - 1) << 16;
			p->tdshp.subfrms[0].hist_cfg_1 = (oh - 1) << 16;
			break;
		case 7:	/* COLOR relay */
			u[0] = 3; u[1] = ow; u[2] = oh;
			p->color.subfrms[0].in_hsize = ow;
			p->color.subfrms[0].in_vsize = oh;
			break;
		case 8:	/* OVL relay */
			u[0] = 0x30000000; u[1] = 1; u[2] = hwo; u[3] = hwo;
			p->ovl.subfrms[0].L0_src_size = (oh << 16) | ow;
			p->ovl.subfrms[0].roi_size    = (oh << 16) | ow;
			break;
		case 9:	/* PAD relay */
			u[0] = ((oh - 1) << 16) | ow;
			p->pad.subfrms[0].pic_size = ((oh - 1) << 16) | ow;
			break;
		case 10: /* WROT: write NV12 output */
			u[0] = oy; u[1] = oc; u[3] = 0x500c;
			u[4] = ostride; u[5] = ostride; u[6] = ostride;
			u[8] = 0x200; u[10] = 0x000d80c4;
			u[21] = hwo; u[22] = hwo; u[24] = (ow << 16) | 0x4000;
			p->wrot.subfrms[0].offset[0] = 0;
			p->wrot.subfrms[0].offset[1] = 0;
			p->wrot.subfrms[0].offset[2] = 0;
			p->wrot.subfrms[0].src  = (oh << 16) | ow;
			p->wrot.subfrms[0].clip = (oh << 16) | ow;
			p->wrot.subfrms[0].main_buf = (ow << 16) | 0x4000;
			break;
		}
	}

	mc = &cfg->ctrls[0];
	mc->num_sets = 7;
	for (i = 0; i < 7; i++) {
		mc->sets[i].reg = mdp_scale_mux[i].reg;
		mc->sets[i].value = mdp_scale_mux[i].value;
		mc->sets[i].subsys_id = mdp_scale_mux[i].subsys_id;
	}

	if (mdp_kd_debug_on()) {
		struct mdp_dev *mdp = vpu_to_mdp((struct mdp_vpu_dev *)
						 (unsigned long)param->drv_data);
		dev_info(&mdp->pdev->dev,
			 "MDPKD-8195-CROP %ux%u crop(%u,%u %ux%u)->%ux%u rsz[%u..%u]x[%u..%u] rdma[%u..%u]x[%u..%u] ph(h%u v%u)\n",
			 iw, ih, cl, ct, cw, ch, ow, oh,
			 rsz_l, rsz_r, rsz_t, rsz_b, rdma_l, rdma_r, rdma_t, rdma_b,
			 phase_h, phase_v);
	}
}

/*
 * Kernel-direct ORIGIN (0,0) CROP compute (mt8195). For a crop whose origin IS the
 * frame origin (cl==ct==0, cw<iw / ch<ih) the SCP does NOT use the off-origin
 * crop-as-RSZ-resample config (no back-tap to read -- nothing lies left/above the
 * frame origin): it BYPASSES the back-margin entirely and emits the SAME config as
 * a PLAIN cw x ch -> ow x oh single-tile scale (phase 0, no origin to fold),
 * differing ONLY in that the RDMA reads the cw x ch sub-window from the top-left of
 * the WIDER iw-stride source buffer (so c0-c2 input-frame dims = the buffer iw x ih,
 * the RDMA stride = istride). Proven on-board (2026-06-30): the origin-crop SCP
 * cfg8195_snapshot is byte-identical to the plain cw x ch scale snapshot except the
 * c0-c2 in-frame dims -- 1:1 / down / up all reduce to it.
 *   1:1  (cw==ow,ch==oh): RSZ BYPASSED (byp=1, coeff=0, control1=0) == native-res
 *        extraction of the top-left sub-window (an identity resample).
 *   down (ow<cw): coeff CEIL((ow-1)<<20/(cw-1)), control1 0x02310143, phase 0.
 *   up   (ow>cw): coeff ROUND((cw-1)<<15/(ow-1)), control1 0x037b0013, phase 0.
 * Single RSZ tile (cw<=512, ow/oh<=512). Gated by mdp_frame_is_nv12_crop (origin
 * arm) + mdp_kd_crop (1:1) / mdp_kd_cropscale (resize). Output md5 == the SCP
 * (firmware-independent, KI-009): the RDMA reads the right sub-window, the RSZ
 * resamples at phase 0 (or bypasses), the WROT writes ow x oh.
 */
static void mdp_config_compute_8195_crop_origin(struct img_ipi_frameparam *param,
						struct img_config *config)
{
	struct img_config_8195 *cfg = &config->config_8195;
	const struct img_image_buffer *in = &param->inputs[0].buffer;
	const struct img_image_buffer *out = &param->outputs[0].buffer;
	const struct img_crop *cr = &param->outputs[0].crop;
	u32 iw = in->format.width,  ih = in->format.height;	/* source BUFFER dims */
	u32 cw = cr->width, ch = cr->height;			/* crop = scale INPUT dims */
	u32 ow = out->format.width, oh = out->format.height;
	u32 istride = in->format.plane_fmt[0].stride;
	u32 ostride = out->format.plane_fmt[0].stride;
	u32 iy = in->iova[0], ic = in->iova[1];
	u32 oy = out->iova[0], oc = out->iova[1];
	u32 hwc = (ch << 16) | cw, whc = (cw << 16) | ch;	/* crop (scale input) */
	u32 hwo = (oh << 16) | ow, who = (ow << 16) | oh;
	bool is11 = (cw == ow && ch == oh);
	bool up_h = (ow > cw), up_v = (oh > ch);	/* per-axis (MIXED origin crops) */
	bool up = up_h || up_v;				/* any-up (the debug print) */
	/* control1: 1:1 -> 0; else SEPARABLE per-axis (H bits 0x000a0050, V bits
	 * 0x01400100; SCP-decoded, same as the mixed scale path) so a MIXED origin
	 * crop (one axis up, the other down) programs each axis independently. */
	u32 ctrl1 = is11 ? 0u :
		    ((0x02310143u & ~0x014a0150u) |
		     ((up_h ? 0x037b0013u : 0x02310143u) & 0x000a0050u) |
		     ((up_v ? 0x037b0013u : 0x02310143u) & 0x01400100u));
	u32 cx = is11 ? 0u :
		 up_h ? (u32)DIV_ROUND_CLOSEST_ULL((u64)(cw - 1) << 15, ow - 1)
		      : (u32)DIV_ROUND_UP_ULL((u64)(ow - 1) << 20, cw - 1);
	u32 cy = is11 ? 0u :
		 up_v ? (u32)DIV_ROUND_CLOSEST_ULL((u64)(ch - 1) << 15, oh - 1)
		      : (u32)DIV_ROUND_UP_ULL((u64)(oh - 1) << 20, ch - 1);
	struct img_mmsys_ctrl *mc;
	int i;

	memset(cfg, 0, sizeof(*cfg));
	cfg->num_components = 11;
	cfg->num_subfrms = 1;

	for (i = 0; i < 11; i++) {
		struct img_compparam_8195 *p = &cfg->components[i];
		struct img_comp_frame_8195 *f = &p->frame;
		struct img_comp_subfrm_8195 *s = &p->subfrms[0];
		u32 *u = (u32 *)&p->rdma;
		/* scale geometry keyed off the CROP dims (== plain cw x ch scale); the
		 * c0-c2 INPUT-frame dims are the real BUFFER (iw x ih), everything else
		 * is the crop-scale geometry. (single tile: in/out_totw = cw / ow) */
		u32 fiw = (i <= 6) ? cw : ow, fih = (i <= 6) ? ch : oh;
		u32 fow = (i <= 4) ? cw : ow, foh = (i <= 4) ? ch : oh;
		u32 siw = (i <= 5) ? cw : ow, sih = (i <= 5) ? ch : oh;
		u32 sow = (i <= 4) ? cw : ow, soh = (i <= 4) ? ch : oh;
		u32 ih_in = (i <= 5) ? ch : oh, ih_out = (i <= 4) ? ch : oh;
		u32 inr = (i <= 5) ? cw - 1 : ow - 1;
		u32 our = (i <= 4) ? cw - 1 : ow - 1;

		p->type = mdp_scale_comp[i].type;
		p->id = mdp_scale_comp[i].id;
		p->input = 0;
		p->num_outputs = 1;
		p->outputs[0] = 0;
		p->num_subfrms = 1;

		f->in_width  = (i <= 2) ? iw : fiw;
		f->in_height = (i <= 2) ? ih : fih;
		f->out_width = fow; f->out_height = foh;
		f->in_total_width  = (i <= 5) ? cw : ow;
		f->out_total_width = (i <= 4) ? cw : ow;
		f->crop.width = siw; f->crop.height = sih;	/* c2/c3/c4/c7 -> 0 below */

		/* sub.in / sub.out windows (single subframe, phase 0, origin) */
		s->in.left = 0; s->in.right = inr;
		s->in.top = 0; s->in.bottom = ih_in - 1;
		s->out.left = 0; s->out.right = our;
		s->out.top = 0; s->out.bottom = ih_out - 1;
		s->out_horizontal = 0;
		s->out_vertical = 1;

		switch (i) {
		case 0:	/* RDMA: read NV12 crop sub-window (cw x ch) from the WIDE buffer */
			u[0] = 0x60c; u[2] = 0x1010; u[3] = iy; u[4] = ic;
			u[6] = iy + istride * ih; u[7] = ic + istride * ih / 2;
			u[9] = istride; u[11] = istride;
			u[15] = 0x03000020; u[16] = 0xe800; u[17] = 0x9c00;
			u[18] = 0x01000014; u[19] = 0x7400; u[20] = 0x4c00;
			u[29] = hwc; u[30] = hwc;
			p->rdma.subfrms[0].offset[0] = 0;
			p->rdma.subfrms[0].offset[1] = 0;
			p->rdma.subfrms[0].offset[2] = 0;
			p->rdma.subfrms[0].src  = (ch << 16) | cw;
			p->rdma.subfrms[0].clip = (ch << 16) | cw;
			break;
		case 1:	u[0] = 1; u[1] = 7; u[2] = (cw << 16); u[3] = (ch << 16);
			p->fg.subfrms[0].info_0 = cw << 16;
			p->fg.subfrms[0].info_1 = ch << 16;
			break;
		case 2:	f->in_total_width = 0; f->out_total_width = 0;
			f->crop.width = 0; f->crop.height = 0;
			memset(&p->subfrms[0].in, 0, sizeof(p->subfrms[0].in));
			memset(&p->subfrms[0].out, 0, sizeof(p->subfrms[0].out));
			break;
		case 3:	f->crop.width = 0; f->crop.height = 0;
			u[0] = 0x10000000; u[1] = 1; u[3] = hwc;
			u[4] = (cw - 1) << 16; u[5] = (ch - 1) << 16;
			u[7] = cw - 1; u[8] = 0x60; u[9] = 0x200;
			p->hdr.subfrms[0].win_size   = 0;
			p->hdr.subfrms[0].src        = (ch << 16) | cw;
			p->hdr.subfrms[0].clip_ofst0 = (cw - 1) << 16;
			p->hdr.subfrms[0].clip_ofst1 = (ch - 1) << 16;
			p->hdr.subfrms[0].hist_ctrl_0 = 0;
			p->hdr.subfrms[0].hist_ctrl_1 = cw - 1;
			p->hdr.subfrms[0].hdr_top = 0x60;	/* first | last */
			break;
		case 4:	f->crop.width = 0; f->crop.height = 0;
			u[0] = 0x80; u[1] = 1; u[2] = whc; u[3] = whc;
			p->aal.subfrms[0].src  = (cw << 16) | ch;
			p->aal.subfrms[0].clip = (cw << 16) | ch;
			p->aal.subfrms[0].clip_ofst = 0;
			break;
		case 5:	/* RSZ: cw x ch -> ow x oh at phase 0; 1:1 BYPASSES */
			f->bypass = is11 ? 1 : 0;
			u[0] = cx; u[1] = cy; u[2] = ctrl1;
			u[15] = 0x1000; u[16] = hwc; u[17] = hwo;
			p->rsz.subfrms[0].control2 = up_h ? 0x1000 : 0x3000;
			p->rsz.subfrms[0].src  = (ch << 16) | cw;
			p->rsz.subfrms[0].clip = (oh << 16) | ow;
			/* phase 0 (origin) -- luma/chroma int+sub all zero */
			break;
		case 6:	f->bypass = 1;
			u[0] = 1; u[1] = who; u[2] = who;
			u[4] = (ow - 1) << 16; u[5] = (oh - 1) << 16;
			p->tdshp.subfrms[0].src  = (ow << 16) | oh;
			p->tdshp.subfrms[0].clip = (ow << 16) | oh;
			p->tdshp.subfrms[0].hist_cfg_0 = (ow - 1) << 16;
			p->tdshp.subfrms[0].hist_cfg_1 = (oh - 1) << 16;
			break;
		case 7:	f->crop.width = 0; f->crop.height = 0;
			u[0] = 3; u[1] = ow; u[2] = oh;
			p->color.subfrms[0].in_hsize = ow;
			p->color.subfrms[0].in_vsize = oh;
			break;
		case 8:	u[0] = 0x30000000; u[1] = 1; u[2] = hwo; u[3] = hwo;
			p->ovl.subfrms[0].L0_src_size = (oh << 16) | ow;
			p->ovl.subfrms[0].roi_size    = (oh << 16) | ow;
			break;
		case 9:	u[0] = ((oh - 1) << 16) | ow;
			p->pad.subfrms[0].pic_size = ((oh - 1) << 16) | ow;
			break;
		case 10: /* WROT: write NV12 output (ow x oh) */
			u[0] = oy; u[1] = oc; u[3] = 0x500c;
			u[4] = ostride; u[5] = ostride; u[6] = ostride;
			u[8] = 0x200; u[10] = 0x000d80c4;
			u[21] = hwo; u[22] = hwo; u[24] = (ow << 16) | 0x4000;
			p->wrot.subfrms[0].offset[0] = 0;
			p->wrot.subfrms[0].offset[1] = 0;
			p->wrot.subfrms[0].offset[2] = 0;
			p->wrot.subfrms[0].src  = (oh << 16) | ow;
			p->wrot.subfrms[0].clip = (oh << 16) | ow;
			p->wrot.subfrms[0].main_buf = (ow << 16) | 0x4000;
			break;
		}
		(void)sow; (void)soh; (void)siw; (void)sih;
	}

	mc = &cfg->ctrls[0];
	mc->num_sets = 7;
	for (i = 0; i < 7; i++) {
		mc->sets[i].reg = mdp_scale_mux[i].reg;
		mc->sets[i].value = mdp_scale_mux[i].value;
		mc->sets[i].subsys_id = mdp_scale_mux[i].subsys_id;
	}

	if (mdp_kd_debug_on()) {
		struct mdp_dev *mdp = vpu_to_mdp((struct mdp_vpu_dev *)
						 (unsigned long)param->drv_data);
		dev_info(&mdp->pdev->dev,
			 "MDPKD-8195-CROPORIGIN %ux%u crop(0,0 %ux%u)->%ux%u %s coeff=[%#x,%#x] ctrl1=%#x\n",
			 iw, ih, cw, ch, ow, oh,
			 is11 ? "1:1byp" : up ? "up" : "down", cx, cy, ctrl1);
	}
}

/*
 * Kernel-direct OFF-ORIGIN >512-WIDE CROP compute (mt8195) -- the last crop gap.
 * An off-origin crop (cl,ct != 0) whose RSZ-input span (cw + back-tap) exceeds the
 * ~512 single-tile line buffer needs the resample split into HORIZONTAL tiles. This
 * MERGES the validated multi-tile scale march (mdp_config_compute_8195_scale -- the
 * output-driven tpipe horizontal split + per-tile relays) with the off-origin crop
 * (mdp_config_compute_8195_crop -- intermediate frame (cl+cw)x(ct+ch), vertical
 * back-tap rsz_t=ct-mv / rdma_t=rsz_t-1, RDMA sub-window read, per-axis coeff/control1
 * keyed off the CROP dims, origin phase fold).
 *
 * Decoded from on-board v23042 cfg8195_snapshot oracles (2026-06-30, the off-origin
 * wide capture in audit/2d-ops-oracle-mt8195.md): the per-tile horizontal phase
 * UNIFIES with the scale march when the RSZ-input-left is measured relative to the
 * CROP origin (il_cl = rsz_il - cl):
 *   DOWN: phase_q20 = (ol<<20) - il_cl*cx      (il_cl<0 on tile0 -> phase = mh*cx)
 *   UP:   phase_q15 = ol*cx - (il_cl<<15)      (il_cl<0 on tile0 -> phase = mh<<15)
 * tile0 seeds il_cl = -mh (the back-tap), eating the LBUF-4 interior budget exactly
 * as the SCP does (so tile0 narrows by mh). The vertical resample carries the single-
 * crop vertical phase (DOWN mv*cy / UP mv<<20) and reads ch+mv rows from rdma_t. The
 * tiling need NOT match the SCP's bit-for-bit -- per the scale-march invariant only
 * the per-tile phase must be exact, the input window a superset of the output
 * footprint, and the outputs abutting+complete -> bit-identical output (md5==SCP).
 * Verified crop(128,64,768,512)->384x256, (128,64,1024,512)->512x256,
 * (256,128,768,512)->384x256 (2-tile down) + (128,64,640,360)->1280x720 (3-tile up).
 * Gated mdp_kd_cropscale; pure up OR pure down; off-origin (cl>=mh+8, ct>=mv+1).
 */
static void mdp_config_compute_8195_crop_wide(struct img_ipi_frameparam *param,
					      struct img_config *config)
{
	struct img_config_8195 *cfg = &config->config_8195;
	const struct img_image_buffer *in = &param->inputs[0].buffer;
	const struct img_image_buffer *out = &param->outputs[0].buffer;
	const struct img_crop *cr = &param->outputs[0].crop;
	u32 iw = in->format.width,  ih = in->format.height;	/* source BUFFER dims */
	u32 ow = out->format.width, oh = out->format.height;
	u32 istride = in->format.plane_fmt[0].stride;
	u32 ostride = out->format.plane_fmt[0].stride;
	u32 iy = in->iova[0], ic = in->iova[1];
	u32 oy = out->iova[0], oc = out->iova[1];
	u32 cl = cr->left, ct = cr->top, cw = cr->width, ch = cr->height;
	u32 fw = cl + cw, fh = ct + ch;			/* intermediate frame (far edge) */
	bool up_h = (ow >= cw), up_v = (oh >= ch), up = up_h;
	u32 cx, cy, mh, mv, ctrl1;
	u32 rsz_t, rsz_b = ct + ch - 1, rdma_t, rdma_b = ct + ch - 1;
	u32 rsz_vh, rdma_vh, vcrop;			/* vertical: rsz/rdma heights + v-crop */
	u32 ph_v, phase_v, phase_v_sub;
	u32 hwo = (oh << 16) | ow, who = (ow << 16) | oh;
	/* per-tile horizontal windows in CROP-LOCAL coords (signed -- tile0 back-tap is
	 * negative). Buffer = crop-local + cl (>=0 by the gate cl>=mh+8). */
	int t_il[IMG_MAX_SUBFRAMES_8195], t_ir[IMG_MAX_SUBFRAMES_8195];	/* RSZ input */
	int t_dl[IMG_MAX_SUBFRAMES_8195], t_dr[IMG_MAX_SUBFRAMES_8195];	/* RDMA read */
	u32 t_ol[IMG_MAX_SUBFRAMES_8195], t_or[IMG_MAX_SUBFRAMES_8195];	/* output (WROT) */
	u32 t_ors[IMG_MAX_SUBFRAMES_8195];				/* relay out right */
	u32 t_phi[IMG_MAX_SUBFRAMES_8195], t_phs[IMG_MAX_SUBFRAMES_8195];/* horiz phase */
	struct img_mmsys_ctrl *mc;
	u32 ntiles = 0, ox = 0, k;
	int prev_ir = 0;
	u32 sum_rdma = 0, sum_rszin = 0, sum_out = 0;
	int i;

	mdp_crop_scale_params(cw, ch, ow, oh, &cx, &cy, &mh, &mv);
	rsz_t = ct - mv; rdma_t = rsz_t - 1;
	rsz_vh = rsz_b - rsz_t + 1; rdma_vh = rdma_b - rdma_t + 1;
	vcrop = rsz_t - rdma_t;				/* 1 */
	ctrl1 = (cw == ow && ch == oh) ? 0x037b0003u :
		((0x02310143u & ~0x014a0150u) |
		 ((up_h ? 0x037b0013u : 0x02310143u) & 0x000a0050u) |
		 ((up_v ? 0x037b0013u : 0x02310143u) & 0x01400100u));
	ph_v = up_v ? (mv << 20) : (mv * cy);
	phase_v = ph_v >> 20; phase_v_sub = ph_v & 0xfffff;

	/* HORIZONTAL output-driven multi-tile march in crop-local coords. tile0 RSZ-
	 * input-left = -mh (the crop back-tap), which (being != 0) takes the LBUF-4
	 * interior budget -- matching the SCP's narrowed tile0. The geometry mirrors the
	 * validated mdp_config_compute_8195_scale up/down marches over the CROP width. */
	if (up) {
		while (ox < ow && ntiles < IMG_MAX_SUBFRAMES_8195) {
			u32 ol = ox;
			u32 orr = min(ol + 512u - 1, ow - 1);
			int il = (ntiles == 0) ? -(int)mh :
				 (int)((((u32)(((u64)ol * cx) >> 15)) - 3u) & ~1u);
			int ir;
			long long phase;

			if (orr >= ow - 1) { orr = ow - 1; ir = (int)cw - 1; }
			else {
				ir = (int)((u32)(((u64)orr * cx) >> 15) + 6u);
				ir |= 1;
				if (ir > (int)cw - 1) ir = (int)cw - 1;
			}
			if (ir - il + 1 > 564) ir = (il + 564 - 1) | 1;
			t_il[ntiles] = il; t_ir[ntiles] = ir;
			t_dl[ntiles] = (il - 8) & ~1;
			t_dr[ntiles] = (ir >= (int)cw - 1) ? (int)cw - 1 :
				       min(ir + 8, (int)cw - 1);
			t_ol[ntiles] = ol; t_or[ntiles] = orr; t_ors[ntiles] = orr;
			phase = (long long)((u64)ol * cx) - ((long long)il << 15);
			t_phi[ntiles] = (u32)(phase >> 15);
			t_phs[ntiles] = (u32)(phase & 0x7fff);
			prev_ir = ir; ox = orr + 1; ntiles++;
		}
	} else {
		while (ox < ow && ntiles < IMG_MAX_SUBFRAMES_8195) {
			u32 ol = ox;
			int il = (ntiles == 0) ? -(int)mh :
				 (int)(((prev_ir + 1 > 12 ? prev_ir + 1 - 12 : 0)) & ~1u);
			u32 orr_t = min(ol + 278u - 1, ow - 1);
			u32 lbufi = (il == 0) ? 564u : 560u;
			int ir, bend;
			u32 orr, ors, foe, t;
			u64 thr;

			thr = ((u64)orr_t << 20) + cx + (3ULL << 20);
			if (thr >= (u64)(cw - 1) * cx) {
				bend = (int)cw - 1;
			} else {
				t = (u32)div64_u64(thr, cx);
				if ((u64)t * cx == thr) t--;
				bend = (t & 1u) ? (int)t : (int)t + 1;
			}
			ir = min3(bend + 2, il + (int)lbufi - 1, (int)cw - 1);
			if (ir >= (int)cw - 1) {
				foe = ow - 1; ors = ow - 1;
			} else {
				t = (u32)((u64)ir * cx >> 20) - 3;
				foe = (t & 1u) ? t : t - 1;
				ors = min(t, orr_t);
			}
			orr = min(foe, orr_t);
			if (orr >= ow - 1) { orr = ow - 1; ir = (int)cw - 1; ors = ow - 1; }
			t_il[ntiles] = il; t_ir[ntiles] = ir;
			t_dl[ntiles] = (il - 8) & ~1;
			t_dr[ntiles] = (ir >= (int)cw - 1) ? (int)cw - 1 :
				       min(ir + 8, (int)cw - 1);
			t_ol[ntiles] = ol; t_or[ntiles] = orr; t_ors[ntiles] = ors;
			t_phi[ntiles] = (u32)((((long long)ol << 20) -
					       (long long)il * cx) >> 20);
			t_phs[ntiles] = (u32)((((long long)ol << 20) -
					       (long long)il * cx) & 0xfffff);
			prev_ir = ir; ox = orr + 1; ntiles++;
		}
	}

	for (k = 0; k < ntiles; k++) {
		sum_rdma  += t_dr[k] - t_dl[k] + 1;
		sum_rszin += t_ir[k] - t_il[k] + 1;
		sum_out   += t_ors[k] - t_ol[k] + 1;
	}

	memset(cfg, 0, sizeof(*cfg));
	cfg->num_components = 11;
	cfg->num_subfrms = ntiles;

	for (i = 0; i < 11; i++) {
		struct img_compparam_8195 *p = &cfg->components[i];
		struct img_comp_frame_8195 *f = &p->frame;
		struct img_comp_subfrm_8195 *s = &p->subfrms[0];
		u32 *u = (u32 *)&p->rdma;
		/* frame in/out dims per the off-origin crop scheme: c0-c2 = real BUFFER
		 * (iw,ih); c3-c6 = intermediate frame (fw,fh); c7-c10 = output (ow,oh).
		 * out: c0-c4 = (fw,fh); c5-c10 = (ow,oh). */
		u32 fiw = (i <= 2) ? iw : (i <= 6) ? fw : ow;
		u32 fih = (i <= 2) ? ih : (i <= 6) ? fh : oh;
		u32 fow = (i <= 4) ? fw : ow;
		u32 foh = (i <= 4) ? fh : oh;

		p->type = mdp_scale_comp[i].type;
		p->id = mdp_scale_comp[i].id;
		p->input = 0;
		p->num_outputs = 1;
		p->outputs[0] = 0;
		p->num_subfrms = ntiles;

		f->in_width = fiw; f->in_height = fih;
		f->out_width = fow; f->out_height = foh;
		/* totw per the scale-march window-sum (multi-tile stitching). */
		f->in_total_width  = (i <= 4) ? sum_rdma : (i == 5) ? sum_rszin : sum_out;
		f->out_total_width = (i <= 3) ? sum_rdma : (i == 4) ? sum_rszin :
				     (i == 10) ? ow : sum_out;
		if (i == 2) { f->in_total_width = 0; f->out_total_width = 0; }

		/* per-tile generic sub.in/sub.out windows (horizontal per-tile, vertical
		 * carries the crop back-tap). RSZ (c5) carries the per-tile horiz phase +
		 * the constant vertical phase; c0 carries the RDMA v-crop. */
		for (k = 0; k < ntiles; k++) {
			struct img_comp_subfrm_8195 *sk = &p->subfrms[k];
			u32 dl = t_dl[k] + cl, dr = t_dr[k] + cl;	/* RDMA buffer cols */
			u32 rl = t_il[k] + cl, rr = t_ir[k] + cl;	/* RSZ buffer cols */
			u32 otl = t_ol[k], otr = t_or[k];		/* output tile */
			u32 otr_r = t_ors[k];				/* relay out right */
			u32 inl, inr, intp, inb, oul, our, outp, oub;

			if (i == 0) {		/* RDMA wide read; v out cropped 1 row */
				inl = dl; inr = dr; intp = rdma_t; inb = rdma_b;
				oul = dl; our = dr; outp = rsz_t;  oub = rdma_b;
			} else if (i == 1 || i == 3) {
				inl = dl; inr = dr; intp = rsz_t; inb = rdma_b;
				oul = dl; our = dr; outp = rsz_t; oub = rdma_b;
			} else if (i == 4) {	/* AAL: crop wide RDMA -> RSZ window */
				inl = dl; inr = dr; intp = rsz_t; inb = rdma_b;
				oul = rl; our = dr; outp = rsz_t; oub = rdma_b;
			} else if (i == 5) {	/* RSZ: narrow input -> output tile */
				inl = rl; inr = rr; intp = rsz_t; inb = rsz_b;
				oul = otl; our = otr_r; outp = 0; oub = oh - 1;
			} else if (i == 2) {
				inl = inr = intp = inb = 0; oul = our = outp = oub = 0;
			} else {		/* c6-c10 output side */
				inl = otl; inr = otr_r; intp = 0; inb = oh - 1;
				oul = otl; our = (i == 10) ? otr : otr_r;
				outp = 0; oub = oh - 1;
			}
			sk->in.left = inl; sk->in.right = inr;
			sk->in.top = intp; sk->in.bottom = inb;
			sk->out.left = oul; sk->out.right = our;
			sk->out.top = outp; sk->out.bottom = oub;
			if (i == 0)
				sk->luma.top = vcrop;	/* RDMA in-tile v crop */
			if (i == 5) {
				sk->luma.left = t_phi[k];
				sk->luma.left_subpix = t_phs[k];
				sk->luma.top = phase_v;
				sk->luma.top_subpix = phase_v_sub;
				sk->chroma.left = t_phi[k];
				sk->chroma.left_subpix = t_phs[k];
				sk->chroma.top = phase_v;
				sk->chroma.top_subpix = phase_v_sub;
			}
			sk->out_horizontal = (k + 1 == ntiles) ? 0 : (int)(k + 1);
			sk->out_vertical = (k + 1 == ntiles) ? 1 : 0;
		}

		/* frame.crop (SCP oracle): c0 rdma rect, c1 rdma@rsz_top, c5 rsz rect,
		 * c6/c8/c9/c10 (0,0,ow,oh). (single-tile-style full extents.) */
		switch (i) {
		case 0:
			f->crop.left = t_dl[0] + cl; f->crop.top = rdma_t;
			f->crop.width = (t_dr[ntiles - 1] + cl) - (t_dl[0] + cl) + 1;
			f->crop.height = rdma_vh;
			break;
		case 1:
			f->crop.left = t_dl[0] + cl; f->crop.top = rsz_t;
			f->crop.width = (t_dr[ntiles - 1] + cl) - (t_dl[0] + cl) + 1;
			f->crop.height = rsz_vh;
			break;
		case 5:
			f->crop.left = t_il[0] + cl; f->crop.top = rsz_t;
			f->crop.width = (t_ir[ntiles - 1] + cl) - (t_il[0] + cl) + 1;
			f->crop.height = rsz_vh;
			break;
		case 6: case 8: case 9: case 10:
			f->crop.width = ow; f->crop.height = oh;
			break;
		}

		/* per-engine HW-register geometry (per tile). Field order transcribed per
		 * engine from the SCP oracle, merging the single-crop vertical (rsz_vh /
		 * rdma_vh / vcrop) with the scale-march horizontal tiling. */
		switch (i) {
		case 0:	/* RDMA: read NV12 sub-window at (rdma_t, dl) per tile */
			u[0] = 0x60c; u[2] = 0x1010; u[3] = iy; u[4] = ic;
			u[6] = iy + istride * ih; u[7] = ic + istride * ih / 2;
			u[9] = istride; u[11] = istride;
			u[15] = 0x03000020; u[16] = 0xe800; u[17] = 0x9c00;
			u[18] = 0x01000014; u[19] = 0x7400; u[20] = 0x4c00;
			for (k = 0; k < ntiles; k++) {
				u32 dl = t_dl[k] + cl;
				u32 dtw = (t_dr[k] - t_dl[k] + 1);
				p->rdma.subfrms[k].offset[0] = rdma_t * istride + dl;
				p->rdma.subfrms[k].offset[1] = (rdma_t / 2) * istride + dl;
				p->rdma.subfrms[k].offset[2] =
					p->rdma.subfrms[k].offset[1];
				p->rdma.subfrms[k].src  = (rdma_vh << 16) | dtw;
				p->rdma.subfrms[k].clip = (rsz_vh << 16) | dtw;
				p->rdma.subfrms[k].clip_ofst = vcrop << 16;
			}
			break;
		case 1:	/* FG relay */
			u[0] = 1; u[1] = 7;
			for (k = 0; k < ntiles; k++) {
				p->fg.subfrms[k].info_0 = (t_dr[k] + cl + 1) << 16;
				p->fg.subfrms[k].info_1 = fh << 16;
			}
			break;
		case 2:	/* disabled */
			break;
		case 3:	/* HDR relay */
			u[0] = 0x10000000; u[1] = 1;
			for (k = 0; k < ntiles; k++) {
				u32 dl = t_dl[k] + cl;
				u32 dtw = (t_dr[k] - t_dl[k] + 1);
				p->hdr.subfrms[k].win_size   = (rsz_t << 16) | dl;
				p->hdr.subfrms[k].src        = (rsz_vh << 16) | dtw;
				p->hdr.subfrms[k].clip_ofst0 = (dtw - 1) << 16;
				p->hdr.subfrms[k].clip_ofst1 = (rsz_vh - 1) << 16;
				p->hdr.subfrms[k].hist_ctrl_0 =
					(k == 0) ? 0 :
					((t_dr[k - 1] + cl) - dl + 1);
				p->hdr.subfrms[k].hist_ctrl_1 = dtw - 1;
				p->hdr.subfrms[k].hdr_top =
					((k == 0) ? 0x20 : 0) |
					((k + 1 == ntiles) ? 0x40 : 0);
				p->hdr.subfrms[k].hist_addr = 0x200;
			}
			break;
		case 4:	/* AAL: crop wide RDMA span -> RSZ window */
			u[0] = 0x80; u[1] = 1;
			for (k = 0; k < ntiles; k++) {
				u32 dtw = (t_dr[k] - t_dl[k] + 1);
				u32 rtw = (t_ir[k] - t_il[k] + 1);
				u32 hcrop = t_il[k] - t_dl[k];	/* rsz_l - rdma_l */
				p->aal.subfrms[k].src  = (dtw << 16) | rsz_vh;
				p->aal.subfrms[k].clip = (rtw << 16) | rsz_vh;
				p->aal.subfrms[k].clip_ofst = (hcrop & 0xff) << 16;
			}
			break;
		case 5:	/* RSZ: resample crop -> output, per-tile horiz + const vert phase */
			u[0] = cx; u[1] = cy; u[2] = ctrl1;
			for (k = 0; k < ntiles; k++) {
				u32 rtw = (t_ir[k] - t_il[k] + 1);
				u32 otw_r = t_ors[k] - t_ol[k] + 1;
				p->rsz.subfrms[k].control2 =
					((up ? (k + 1 == ntiles)
					     : (ntiles > 1 && k + 1 == ntiles))
					 ? 0x1000 : 0x3000);
				p->rsz.subfrms[k].src  = (rsz_vh << 16) | rtw;
				p->rsz.subfrms[k].clip = (oh << 16) | otw_r;
				p->rsz.subfrms[k].luma_h_int_ofst = t_phi[k];
				p->rsz.subfrms[k].luma_h_sub_ofst = t_phs[k];
				p->rsz.subfrms[k].luma_v_int_ofst = phase_v;
				p->rsz.subfrms[k].luma_v_sub_ofst = phase_v_sub;
				p->rsz.subfrms[k].chroma_h_int_ofst = t_phi[k];
				p->rsz.subfrms[k].chroma_h_sub_ofst = t_phs[k];
			}
			break;
		case 6:	/* TDSHP relay (output side) */
			f->bypass = 1;
			u[0] = 1; u[1] = who; u[2] = who;
			u[4] = (ow - 1) << 16; u[5] = (oh - 1) << 16;
			for (k = 0; k < ntiles; k++) {
				u32 otw_r = t_ors[k] - t_ol[k] + 1;
				p->tdshp.subfrms[k].src  = (otw_r << 16) | oh;
				p->tdshp.subfrms[k].clip = (otw_r << 16) | oh;
				p->tdshp.subfrms[k].hist_cfg_0 =
					((otw_r - 1) << 16) |
					((!up && k + 1 == ntiles) ? 1 : 0);
				p->tdshp.subfrms[k].hist_cfg_1 = (oh - 1) << 16;
			}
			break;
		case 7:	/* COLOR relay */
			u[0] = 3; u[1] = ow; u[2] = oh;
			for (k = 0; k < ntiles; k++) {
				u32 otw_r = t_ors[k] - t_ol[k] + 1;
				p->color.subfrms[k].in_hsize = otw_r;
				p->color.subfrms[k].in_vsize = oh;
			}
			break;
		case 8:	/* OVL relay */
			u[0] = 0x30000000; u[1] = 1; u[2] = hwo; u[3] = hwo;
			for (k = 0; k < ntiles; k++) {
				u32 otw_r = t_ors[k] - t_ol[k] + 1;
				p->ovl.subfrms[k].L0_src_size = (oh << 16) | otw_r;
				p->ovl.subfrms[k].roi_size    = (oh << 16) | otw_r;
			}
			break;
		case 9:	/* PAD relay */
			u[0] = ((oh - 1) << 16) | ow;
			for (k = 0; k < ntiles; k++) {
				u32 otw_r = t_ors[k] - t_ol[k] + 1;
				p->pad.subfrms[k].pic_size = ((oh - 1) << 16) | otw_r;
			}
			break;
		case 10: /* WROT: write NV12 output tile (read overlap, write abutting) */
			u[0] = oy; u[1] = oc; u[3] = 0x500c;
			u[4] = ostride; u[5] = ostride; u[6] = ostride;
			u[8] = 0x200; u[10] = 0x000d80c4;
			u[21] = hwo; u[22] = hwo; u[24] = (ow << 16) | 0x4000;
			for (k = 0; k < ntiles; k++) {
				u32 otl = t_ol[k];
				u32 otw = t_or[k] - otl + 1;
				u32 otw_r = t_ors[k] - otl + 1;
				p->wrot.subfrms[k].offset[0] = otl;
				p->wrot.subfrms[k].offset[1] = otl;
				p->wrot.subfrms[k].offset[2] = otl;
				p->wrot.subfrms[k].src  = (oh << 16) | otw_r;
				p->wrot.subfrms[k].clip = (oh << 16) | otw;
				p->wrot.subfrms[k].main_buf = (otw << 16) | 0x4000;
			}
			break;
		}
	}

	for (k = 0; k < ntiles; k++) {
		mc = &cfg->ctrls[k];
		mc->num_sets = 7;
		for (i = 0; i < 7; i++) {
			mc->sets[i].reg = mdp_scale_mux[i].reg;
			mc->sets[i].value = mdp_scale_mux[i].value;
			mc->sets[i].subsys_id = mdp_scale_mux[i].subsys_id;
		}
	}

	if (mdp_kd_debug_on()) {
		struct mdp_dev *mdp = vpu_to_mdp((struct mdp_vpu_dev *)
						 (unsigned long)param->drv_data);
		dev_info(&mdp->pdev->dev,
			 "MDPKD-8195-CROPWIDE %ux%u crop(%u,%u %ux%u)->%ux%u %s ntiles=%u coeff=[%#x,%#x] ph0(h%u.%#x v%u.%#x)\n",
			 iw, ih, cl, ct, cw, ch, ow, oh, up ? "up" : "down",
			 ntiles, cx, cy, t_phi[0], t_phs[0], phase_v, phase_v_sub);
	}
}

/*
 * Op-specific WROT (c9) parameters for the dual-pipe primitives (LINEAR + MM21
 * block-detile). The read side (RDMA c0) + relay geometry (c1-c8) are op-independent;
 * only the WROT write differs by op (blit / CSC / fmtconv / detile-NV12). out_ofst[k]
 * (the per-half output-plane column byte offset written to WROT VIDO_OFST u[18..20] on
 * pp1) is derived from outx=pw as (outx * ofmul[k]) >> ofshr[k]: NV12 = {1,1,1}/{0,0,0}
 * (Y + interleaved UV, 1 B/col each); RGB24 = {3,0,0}/{0,0,0}; I420 = {1,1,1}/{0,1,1}
 * (Y outx, U/V half-width outx>>1); NV16 4:2:2-semi = {1,1,0}/{0,0,0} (full-width chroma).
 */
struct mdp_dual_wrot {
	u32 iova[3];		/* full-frame output plane bases (offset is in u[18..20]) */
	u32 stride[3];
	u32 control, mat_ctrl, pre_ultra;
	u32 main_buf;		/* WROT VIDO_MAIN_BUF_SIZE low half (u[24]): NV12 0x4000, RGB24 0x2000 */
	u32 base_ofst[3];	/* pp0 output-plane VIDO offset (0 for identity blit/CSC/fmtconv;
				 * the single-pipe xform offset for rot180/hflip/vflip) */
	u32 pp1_term[3];	/* explicit per-plane pp1 offset delta -- used when
				 * pp1_explicit_term (dim-swapping rot90/270: pw*ostride /
				 * pw*ostride/2, too large for the u8 ofmul form) */
	u8  ofmul[3], ofshr[3];
	bool pp1_reverse;	/* H-mirroring / rot270: pp1 writes the reversed output band,
				 * so its per-plane term is SUBTRACTED from base_ofst, not added */
	bool pp1_explicit_term;	/* use pp1_term[] instead of outx*ofmul>>ofshr */
};

/*
 * Two-pixel-pipe (dual) variant: for out area >= 2,073,600 the mt8195 SCP splits
 * the detile across TWO SVPP pipes (param.type == MDP_STREAM_TYPE_DUAL_BITBLT,
 * pp_used = 2). Fill ONE pipe's config: a clean horizontal split, output W/2 per
 * pipe (pp=0 left, pp=1 right), pp=1 on a different SVPP instance (type=pp0+1,
 * id 2->3, mmsys mux regs +0x2c). Read overlaps 16 (pp0 right) / 8 (pp1 left);
 * full-frame iovas/strides; pp=1 column offset in the engine subfrm.offset[].
 * Offline-validated bit-exact vs the SCP at 1920x1080 / 2560x1088 / 1920x1440
 * (tests/mdp_config_compute_dual_test.c).
 *
 * WROT (c9) is parameterized by an optional `wr`: NULL keeps the fixed NV12-blit
 * detile write (the original MM21->NV12 dual arm, byte-identical); a non-NULL `wr`
 * (mdp_dual_wrot_for_dst) drives an arbitrary fmtconv/3-plane WROT off the SAME
 * detiled 4:2:0 pipe data -- the block-detile-read + generic-write combine used by
 * the >1920 MM21/MT21-source arm (mdp_kd_genmm21). The read side (c0-c8) is
 * format-independent (detile -> linear 4:2:0 in the pipe), so only c9 changes.
 */
static void mdp_fill_dual_pipe_8195(struct img_config_8195 *cfg, int pp,
				    u32 w, u32 h, u32 inh,
				    u32 iy, u32 ic, u32 oy, u32 oc,
				    const struct mdp_dual_wrot *wr)
{
	u32 pw = w / 2, ch = ALIGN(h, 32);
	u32 outx = pp ? pw : 0;
	u32 rw = pp ? (pw + 8) : (pw + 16);	/* read (crop) width */
	u32 sw = pw + 8;			/* rsz width */
	u32 rx = pp ? (pw - 8) : 0;		/* read start col */
	u32 ta = pp ? (rx & ~15u) : 0;		/* tile-aligned read col */
	u32 itx = pp ? (rx - ta) : 0;		/* in-tile x */
	u32 aw = ALIGN(w, 64);			/* MM21 read-side aligned full width */
	static const u8 ty[10] = { 10, 15, 17, 27, 31, 35, 39, 43, 49, 55 };
	static const u8 idc[10] = { 2, 2, 4, 2, 2, 2, 2, 2, 2, 2 };
	/* SVPP2 (pp0) / SVPP3 (pp1) mmsys mux regs -- pp1 = pp0+0x2c EXCEPT set5 */
	static const u32 muxr0[7] = { 0xf50, 0xf4c, 0xf38, 0xf44, 0xf40, 0xf90, 0xf3c };
	static const u32 muxr1[7] = { 0xf7c, 0xf78, 0xf64, 0xf70, 0xf6c, 0xf60, 0xf68 };
	static const u32 muxv[7] = { 1, 1, 0, 0, 1, 0, 0 };
	struct img_mmsys_ctrl *mc;
	int i;

	memset(cfg, 0, sizeof(*cfg));
	cfg->num_components = 10;
	cfg->num_subfrms = 1;

	for (i = 0; i < 10; i++) {
		struct img_compparam_8195 *p = &cfg->components[i];
		struct img_comp_frame_8195 *f = &p->frame;
		struct img_comp_subfrm_8195 *s = &p->subfrms[0];
		u32 *u = (u32 *)&p->rdma;

		p->type = ty[i] + pp;
		p->id = idc[i] + pp;
		p->input = 0; p->num_outputs = 1; p->outputs[0] = 0;
		p->num_subfrms = 1;
		s->out_vertical = 1;

		f->in_width = w; f->in_height = h; f->out_width = w; f->out_height = h;
		f->in_total_width = sw; f->out_total_width = sw;
		f->crop.left = outx; f->crop.width = pw; f->crop.height = h;
		s->in.left = outx; s->in.right = outx + pw - 1; s->in.bottom = h - 1;
		s->out.left = outx; s->out.right = outx + pw - 1; s->out.bottom = h - 1;

		switch (i) {
		case 0:	/* RDMA */
			f->in_width = aw;	/* RDMA/RSZ/relay read the full padded width */
			f->in_height = inh;
			f->crop.left = rx; f->crop.width = rw; f->crop.height = ch;
			f->in_total_width = rw; f->out_total_width = sw;
			s->in.left = rx; s->in.right = rx + rw - 1; s->in.bottom = ch - 1;
			s->out.left = rx; s->out.right = rx + sw - 1;
			u[0] = 0x860c; u[2] = 0x10; u[3] = iy; u[4] = ic;
			u[6] = iy + aw * inh; u[7] = ic + aw * inh / 2;
			u[9] = aw * 32; u[11] = aw * 16;
			u[15] = 0x03000020; u[16] = 0xe800; u[17] = 0x9c00;
			u[18] = 0x01000010; u[19] = 0x7400; u[20] = 0x4c00;
			if (pp) { u[25] = ta * 32; u[26] = ta * 16; u[27] = ta * 16; u[31] = itx; }
			u[29] = (ch << 16) | (pw + 16); u[30] = (h << 16) | sw;
			break;
		case 1:	/* RSZ */
			f->in_width = aw;
			f->in_height = inh;
			f->crop.left = rx; f->crop.width = sw;
			s->in.left = rx; s->in.right = rx + sw - 1;
			s->out.left = rx; s->out.right = rx + sw - 1;
			u[0] = 1; u[1] = 7; u[2] = (pp ? w : sw) << 16; u[3] = h << 16;
			break;
		case 2:	/* relay */
			f->in_width = aw;
			f->in_height = inh;
			f->in_total_width = 0; f->out_total_width = 0;
			f->crop.left = 0; f->crop.width = 0; f->crop.height = 0;
			memset(&s->in, 0, sizeof(s->in));
			memset(&s->out, 0, sizeof(s->out));
			break;
		case 3:
			f->crop.left = 0; f->crop.width = 0; f->crop.height = 0;
			s->in.left = rx; s->in.right = rx + sw - 1;
			s->out.left = rx; s->out.right = rx + sw - 1;
			u[0] = 0x10000000; u[1] = 1; u[2] = pp ? rx : 0;
			u[3] = (h << 16) | sw; u[4] = (sw - 1) << 16; u[5] = (h - 1) << 16;
			u[7] = sw - 1; u[8] = pp ? 0x60 : 0x20; u[9] = 0x200;
			break;
		case 4:
			f->crop.left = 0; f->crop.width = 0; f->crop.height = 0;
			f->in_total_width = sw; f->out_total_width = pw;
			s->in.left = rx; s->in.right = rx + sw - 1;
			s->out.left = outx; s->out.right = outx + pw - 1;
			u[0] = 0x80; u[1] = 1; u[2] = (sw << 16) | h; u[3] = (pw << 16) | h;
			if (pp) u[4] = itx << 16;
			break;
		case 5:	/* byp */
			f->bypass = 1;
			f->in_total_width = pw; f->out_total_width = pw;
			f->crop.left = outx; f->crop.width = pw; f->crop.height = h;
			u[15] = pp ? 0x1000 : 0x3000;	/* 0x3000 = has right-neighbour pipe */
			u[16] = (h << 16) | pw; u[17] = (h << 16) | pw; u[26] = (h << 16) | pw;
			break;
		case 6:	/* byp */
			f->bypass = 1;
			f->in_total_width = pw; f->out_total_width = pw;
			f->crop.left = outx; f->crop.width = pw; f->crop.height = h;
			u[0] = 1; u[1] = (pw << 16) | h; u[2] = (pw << 16) | h;
			u[4] = (pw - 1) << 16; u[5] = (h - 1) << 16;
			break;
		case 7:
			f->crop.left = 0; f->crop.width = 0; f->crop.height = 0;
			f->in_total_width = pw; f->out_total_width = pw;
			u[0] = 3; u[1] = pw; u[2] = h;
			break;
		case 8:
			f->in_total_width = pw; f->out_total_width = pw;
			u[0] = ((h - 1) << 16) | pw;
			break;
		case 9:	/* WROT */
			f->in_total_width = pw; f->out_total_width = pw;
			if (wr) {
				/* Parameterized write off the detiled 4:2:0 pipe --
				 * byte-identical to the LINEAR primitive's case 9. */
				u[0] = wr->iova[0]; u[1] = wr->iova[1]; u[2] = wr->iova[2];
				u[3] = wr->control;
				u[4] = wr->stride[0]; u[5] = wr->stride[1]; u[6] = wr->stride[2];
				u[7] = wr->mat_ctrl;
				u[8] = 0x400; u[10] = wr->pre_ultra;
				u[18] = wr->base_ofst[0];
				u[19] = wr->base_ofst[1];
				u[20] = wr->base_ofst[2];
				if (pp) {
					u32 t0, t1, t2;

					if (wr->pp1_explicit_term) {
						t0 = wr->pp1_term[0];
						t1 = wr->pp1_term[1];
						t2 = wr->pp1_term[2];
					} else {
						t0 = (outx * wr->ofmul[0]) >> wr->ofshr[0];
						t1 = (outx * wr->ofmul[1]) >> wr->ofshr[1];
						t2 = (outx * wr->ofmul[2]) >> wr->ofshr[2];
					}
					if (wr->pp1_reverse) {
						u[18] -= t0; u[19] -= t1; u[20] -= t2;
					} else {
						u[18] += t0; u[19] += t1; u[20] += t2;
					}
				}
				u[21] = (h << 16) | pw; u[22] = (h << 16) | pw;
				u[24] = (pw << 16) | wr->main_buf;
			} else {
				u[0] = oy; u[1] = oc; u[3] = 0x500c;
				u[4] = w; u[5] = w; u[6] = w;
				u[8] = 0x400; u[10] = 0x000d80c4;
				if (pp) { u[18] = outx; u[19] = outx; u[20] = outx; }
				u[21] = (h << 16) | pw; u[22] = (h << 16) | pw;
				u[24] = (pw << 16) | 0x4000;
			}
			break;
		}
	}

	mc = &cfg->ctrls[0];
	mc->num_sets = 7;
	for (i = 0; i < 7; i++) {
		mc->sets[i].reg = pp ? muxr1[i] : muxr0[i];
		mc->sets[i].value = muxv[i];
		mc->sets[i].subsys_id = 9;
	}
}

/*
 * Two-pixel-pipe (dual) BLOCK-PADDED CROP variant of the MM21/MT21 detile dual.
 * For a >1920 (DUAL_BITBLT) MM21 source whose output height is UNPADDED (out h <
 * block-padded coded inh, e.g. 2560x1440 with inh=1472), the SCP does NOT use the
 * num_subfrms=1 RSZ-bypassed detile config -- it activates the c5 RSZ as a VERTICAL
 * DOWNSCALE inh->h (coeff_y = DIV_ROUND_UP((h-1)<<20, inh-1)), which forces a
 * HORIZONTAL sub-tiling of each pipe (num_subfrms=2: tile0 = 974 output cols, the
 * crop-RSZ 1:1-horizontal line-buffer cap; tile1 = the pipe remainder). Horizontal
 * stays 1:1 (coeff_x=0x8000, phase int 4). RE'd bit-exact from the v23042 SCP dual
 * oracle @2560x1440 + 3840x2160 (both pipes' pp1 snapshot; audit/mm21-dual-crop-
 * oracle-mt8195.md). The pure detile (in==out) keeps mdp_fill_dual_pipe_8195
 * (num_subfrms=1) BYTE-IDENTICAL -- this function is only reached for out h < inh.
 *
 * c9 (WROT) is parameterized by the same optional `wr` (mdp_dual_wrot) the detile
 * dual takes: NULL = fixed NV12 write; non-NULL = arbitrary fmtconv/3-plane off the
 * detiled 4:2:0 pipe.
 */
static void mdp_fill_dual_pipe_8195_crop(struct img_config_8195 *cfg, int pp,
					 u32 w, u32 h, u32 inh,
					 u32 iy, u32 ic, u32 oy, u32 oc,
					 const struct mdp_dual_wrot *wr)
{
	u32 pw = w / 2;
	u32 ps = pp ? pw : 0;			/* pipe output start col */
	u32 pe = pp ? (w - 1) : (pw - 1);	/* pipe output end col */
	u32 aw = ALIGN(w, 64);			/* MM21 read-side aligned full width */
	u32 cy = (u32)DIV_ROUND_UP_ULL((u64)(h - 1) << 20, inh - 1);	/* vertical downscale */
	/* per-tile windows (<=2 tiles for w<=3840: pw<=1920, tile0<=974) */
	u32 ol[2], orr[2], rl[2], rr[2], dl[2], dr[2], rdc[2];
	u32 ta[2], itx[2], readw[2], clipw[2], tw[2], rread[2];
	u32 S_rdma_full = 0, S_rdma = 0, S_rsz = 0, S_out = 0;
	u32 nt = 0, ox = ps, k;
	static const u8 ty[10] = { 10, 15, 17, 27, 31, 35, 39, 43, 49, 55 };
	static const u8 idc[10] = { 2, 2, 4, 2, 2, 2, 2, 2, 2, 2 };
	static const u32 muxr0[7] = { 0xf50, 0xf4c, 0xf38, 0xf44, 0xf40, 0xf90, 0xf3c };
	static const u32 muxr1[7] = { 0xf7c, 0xf78, 0xf64, 0xf70, 0xf6c, 0xf60, 0xf68 };
	static const u32 muxv[7] = { 1, 1, 0, 0, 1, 0, 0 };
	struct img_mmsys_ctrl *mc;
	int i;

	/* Horizontal output-driven march (1:1, OCAP_CROP=974, abutting tiles). */
	while (ox <= pe && nt < 2) {
		ol[nt] = ox;
		orr[nt] = min(ox + 974u - 1, pe);
		rl[nt] = (ol[nt] == 0) ? 0 : (ol[nt] - 4);	/* RSZ in-left = out-left - 4 (h-phase); frame edge = 0 */
		rr[nt] = (orr[nt] == w - 1) ? (w - 1) : (orr[nt] + 8);	/* RSZ in-right = out-right + 8; frame edge clamps */
		dl[nt] = (rl[nt] == 0) ? 0 : (rl[nt] - 8);	/* RDMA read-left = RSZ-left - 8 (TAP) */
		dr[nt] = (rr[nt] == w - 1) ? (w - 1) : (rr[nt] + 10);	/* RDMA read-right = RSZ-right + 10 */
		rdc[nt] = (dr[nt] == w - 1) ? dr[nt] : (dr[nt] - 2);	/* RDMA content-right (relay-cropped) */
		ta[nt] = dl[nt] & ~15u;			/* block-aligned read col */
		itx[nt] = dl[nt] - ta[nt];		/* in-tile x */
		tw[nt] = rdc[nt] - dl[nt] + 1;		/* relay tile width (content) */
		ox = orr[nt] + 1;
		nt++;
	}

	for (k = 0; k < nt; k++) {
		/*
		 * A pipe's LAST tile has no successor tile in the same pipe to
		 * overlap, so the RDMA reads only to the content-right rdc (at the
		 * frame right edge rdc==dr, so pp1 is unchanged); internal tiles read
		 * the full tap-extended dr. This "last tile == frame edge" conflation
		 * was the pp0 bug: pp0's last tile is the internal pixel-pipe seam,
		 * not the frame edge.
		 */
		rread[k] = (k + 1 == nt) ? rdc[k] : dr[k];
		readw[k] = rread[k] - ta[k] + 1;	/* block read width from ta */
		clipw[k] = rdc[k] - dl[k] + 1;		/* content clip width (= tw) */
		S_rdma_full += rread[k] - dl[k] + 1;
		S_rdma      += rdc[k] - dl[k] + 1;
		S_rsz       += rr[k] - rl[k] + 1;
		S_out       += orr[k] - ol[k] + 1;
	}

	memset(cfg, 0, sizeof(*cfg));
	cfg->num_components = 10;
	cfg->num_subfrms = nt;

	for (i = 0; i < 10; i++) {
		struct img_compparam_8195 *p = &cfg->components[i];
		struct img_comp_frame_8195 *f = &p->frame;
		u32 *u = (u32 *)&p->rdma;
		/* frame in-height: c0-c6 = inh, c7-c9 = h. out-height: c0-c4 = inh, c5-c9 = h. */
		u32 fih = (i <= 6) ? inh : h;
		u32 foh = (i <= 4) ? inh : h;

		p->type = ty[i] + pp;
		p->id = idc[i] + pp;
		p->input = 0; p->num_outputs = 1; p->outputs[0] = 0;
		p->num_subfrms = nt;

		f->in_width = w; f->in_height = fih;
		f->out_width = w; f->out_height = foh;
		/* frame crop.left / width / height per component (0 for the pure relays). */
		switch (i) {
		case 0: case 1:
			/* crop.width spans the pipe's read content [dl[0], rdc[last]],
			 * NOT the full frame width (= w only at the frame edge / pp1). */
			f->crop.left = dl[0]; f->crop.width = rdc[nt - 1] + 1 - dl[0];
			f->crop.height = inh;
			f->in_total_width  = (i == 0) ? S_rdma_full : S_rdma;
			f->out_total_width = S_rdma;
			break;
		case 3:
			f->in_total_width = S_rdma; f->out_total_width = S_rdma;
			break;
		case 4:
			f->in_total_width = S_rdma; f->out_total_width = S_rsz;
			break;
		case 5:
			f->crop.left = rl[0]; f->crop.width = rr[nt - 1] + 1 - rl[0];
			f->crop.height = inh;
			f->in_total_width = S_rsz; f->out_total_width = S_out;
			break;
		case 6:
			f->crop.left = ol[0]; f->crop.width = orr[nt - 1] + 1 - ol[0];
			f->crop.height = h;
			f->in_total_width = S_out; f->out_total_width = S_out;
			break;
		case 8: case 9:
			f->crop.left = ol[0]; f->crop.width = orr[nt - 1] + 1 - ol[0];
			f->crop.height = h;
			f->in_total_width = S_out; f->out_total_width = S_out;
			break;
		case 7:
			f->in_total_width = S_out; f->out_total_width = S_out;
			break;
		case 2:
			break;	/* full relay, no crop/totw */
		}

		/* per-tile sub-region windows (in/out left,right,top,bottom) */
		for (k = 0; k < nt; k++) {
			struct img_comp_subfrm_8195 *s = &p->subfrms[k];
			u32 il, ir, oul, our, ih_in, ih_out;

			if (i <= 3)      { il = dl[k]; ir = (i == 0) ? rread[k] : rdc[k]; ih_in = inh; }
			else if (i == 4) { il = dl[k]; ir = rdc[k]; ih_in = inh; }
			else if (i == 5) { il = rl[k]; ir = rr[k];  ih_in = inh; }
			else             { il = ol[k]; ir = orr[k]; ih_in = h; }

			if (i <= 3)       { oul = dl[k]; our = rdc[k]; ih_out = inh; }
			else if (i == 4)  { oul = rl[k]; our = rr[k];  ih_out = inh; }
			else              { oul = ol[k]; our = orr[k]; ih_out = h; }

			s->out_horizontal = (k + 1 == nt) ? 0 : (int)(k + 1);
			s->out_vertical = (k + 1 == nt) ? 1 : 0;
			if (i == 2) continue;	/* c2 keeps out_h/out_v, zeros in/out regions */
			s->in.left = il; s->in.right = ir; s->in.top = 0; s->in.bottom = ih_in - 1;
			s->out.left = oul; s->out.right = our; s->out.top = 0; s->out.bottom = ih_out - 1;
			if (i == 5) {	/* RSZ h-phase: int 4, but 0 at the frame LEFT
					 * edge (rl==0, no left back-tap) -- pp0 tile0. */
				s->luma.left = (rl[k] == 0) ? 0 : 4;
				s->chroma.left = (rl[k] == 0) ? 0 : 4;
			}
		}

		switch (i) {
		case 0:	/* RDMA block-detile read (out_h = inh, per-tile block offsets) */
			u[0] = 0x860c; u[2] = 0x10; u[3] = iy; u[4] = ic;
			u[6] = iy + aw * inh; u[7] = ic + aw * inh / 2;
			u[9] = aw * 32; u[11] = aw * 16;
			u[15] = 0x03000020; u[16] = 0xe800; u[17] = 0x9c00;
			u[18] = 0x01000010; u[19] = 0x7400; u[20] = 0x4c00;
			for (k = 0; k < nt; k++) {
				u32 b = 25 + 9 * k;
				u[b] = ta[k] * 32; u[b + 1] = ta[k] * 16; u[b + 2] = ta[k] * 16;
				u[b + 4] = (inh << 16) | readw[k];
				u[b + 5] = (inh << 16) | clipw[k];
				u[b + 6] = itx[k];
			}
			break;
		case 1:	/* FG relay: per-tile (rdc+1)<<16 | inh<<16, stride 2 from u[2] */
			u[0] = 1; u[1] = 7;
			for (k = 0; k < nt; k++) {
				u[2 + 2 * k] = (rdc[k] + 1) << 16;
				u[3 + 2 * k] = inh << 16;
			}
			break;
		case 2:	/* relay: zero (num_subfrms header only) */
			break;
		case 3:	/* HDR relay: per-tile 8-word window from u[2] */
			u[0] = 0x10000000; u[1] = 1;
			for (k = 0; k < nt; k++) {
				u32 b = 2 + 8 * k;
				u[b] = dl[k];
				u[b + 1] = (inh << 16) | tw[k];
				u[b + 2] = (tw[k] - 1) << 16;
				u[b + 3] = (inh - 1) << 16;
				u[b + 4] = (k == 0) ? 0 : (rdc[k - 1] - dl[k] + 1);
				u[b + 5] = tw[k] - 1;
				/* 0x20 = pipe's first tile; 0x40 = FRAME right edge (orr==w-1),
				 * not the pipe's last tile (pp0 last tile = internal seam). */
				u[b + 6] = ((k == 0) ? 0x20 : 0) |
					   ((orr[k] == w - 1) ? 0x40 : 0);
				u[b + 7] = (k == 0) ? 0x200 : 0;
			}
			break;
		case 4:	/* AAL crop wide->rsz: per-tile (tw<<16)|inh, (rtw<<16)|inh, crop<<16 */
			u[0] = 0x80; u[1] = 1;
			for (k = 0; k < nt; k++) {
				u32 rtw = rr[k] - rl[k] + 1;
				u32 crop = rl[k] - dl[k];	/* left crop rsz_il - rdma_il */
				u32 b = 2 + 3 * k;
				u[b] = (tw[k] << 16) | inh;
				u[b + 1] = (rtw << 16) | inh;
				u[b + 2] = (crop & 0xff) << 16;	/* AAL crop_ofst, HIGH half */
			}
			break;
		case 5:	/* RSZ ACTIVE: coeff_x=1:1, coeff_y=vertical downscale, control1 */
			u[0] = 0x8000; u[1] = cy; u[2] = 0x023b0103;
			for (k = 0; k < nt; k++) {
				u32 rtw = rr[k] - rl[k] + 1;
				u32 otw = orr[k] - ol[k] + 1;
				u32 b = 15 + 12 * k;
				u32 ph = (rl[k] == 0) ? 0 : 4;	/* h-phase 0 at frame LEFT edge */
				/* control2 last-tile flag keys on the FRAME right edge
				 * (orr==w-1), not the pipe's last tile -- pp0's last tile is
				 * the internal seam, not the frame edge. */
				u[b] = (orr[k] == w - 1) ? 0x1000 : 0x3000;	/* control2 */
				u[b + 1] = (inh << 16) | rtw;			/* PRZ_INPUT */
				u[b + 2] = (h << 16) | otw;			/* PRZ_OUTPUT */
				u[b + 4] = ph;					/* luma_h_int */
				u[b + 8] = ph;					/* chroma_h_int */
				u[b + 11] = (h << 16) | otw;			/* cv trailing */
			}
			break;
		case 6:	/* TDSHP relay (output side, bypass): per-tile stride 5 from u[1] */
			f->bypass = 1;
			u[0] = 1;
			for (k = 0; k < nt; k++) {
				u32 otw = orr[k] - ol[k] + 1;
				u32 b = 1 + 5 * k;
				u[b] = (otw << 16) | h;
				u[b + 1] = (otw << 16) | h;
				u[b + 3] = (otw - 1) << 16;
				u[b + 4] = (h - 1) << 16;
			}
			break;
		case 7:	/* COLOR relay: u[0]=3, per-tile (otw, h) pairs */
			u[0] = 3;
			for (k = 0; k < nt; k++) {
				u[1 + 2 * k] = orr[k] - ol[k] + 1;
				u[2 + 2 * k] = h;
			}
			break;
		case 8:	/* OVL relay: per-tile ((h-1)<<16)|otw */
			for (k = 0; k < nt; k++)
				u[k] = ((h - 1) << 16) | (orr[k] - ol[k] + 1);
			break;
		case 9:	/* WROT write output tile (NV12 or parameterized wr) */
			if (wr) {
				u[0] = wr->iova[0]; u[1] = wr->iova[1]; u[2] = wr->iova[2];
				u[3] = wr->control;
				u[4] = wr->stride[0]; u[5] = wr->stride[1]; u[6] = wr->stride[2];
				u[7] = wr->mat_ctrl; u[8] = 0x400; u[10] = wr->pre_ultra;
			} else {
				u[0] = oy; u[1] = oc; u[3] = 0x500c;
				u[4] = w; u[5] = w; u[6] = w;
				u[8] = 0x400; u[10] = 0x000d80c4;
			}
			for (k = 0; k < nt; k++) {
				u32 otw = orr[k] - ol[k] + 1;
				u32 b = 18 + 7 * k;
				/* Per-plane output VIDO offset scales by bytes-per-column:
				 * wr->ofmul[0] = dst plane0 bpp (2 packed-4:2:2 / 3 RGB),
				 * ofmul[1] = 1 for a 2-plane interleaved-chroma dst else 0.
				 * A raw ol[k] (bpp=1) is only correct for the NV12 detile
				 * (wr==NULL) + 1-bpp-Y dsts; packed/RGB dsts need the scale
				 * (matches the linear dual's outx*ofmul form). */
				if (wr) {
					u[b]     = (ol[k] * wr->ofmul[0]) >> wr->ofshr[0];
					u[b + 1] = (ol[k] * wr->ofmul[1]) >> wr->ofshr[1];
					u[b + 2] = (ol[k] * wr->ofmul[2]) >> wr->ofshr[2];
				} else {
					u[b] = ol[k]; u[b + 1] = ol[k]; u[b + 2] = ol[k];
				}
				u[b + 3] = (h << 16) | otw;		/* VIDO_IN_SIZE */
				u[b + 4] = (h << 16) | otw;		/* VIDO_TAR_SIZE */
				u[b + 6] = (otw << 16) | 0x4000;	/* main_buf (output-inert) */
			}
			break;
		}
	}

	/* The mmsys mux is applied PER SUBFRAME -- the SCP replicates the same 7-set
	 * mux into ctrls[0..num_subfrms-1] (byte-confirmed vs the crop oracle: ctrls[1]
	 * duplicates ctrls[0]). The num_subfrms=1 detile fills only ctrls[0]; the
	 * num_subfrms=2 crop must fill both, else the 2nd subframe never routes and the
	 * WROT emits 0 bytes. */
	for (k = 0; k < nt; k++) {
		mc = &cfg->ctrls[k];
		mc->num_sets = 7;
		for (i = 0; i < 7; i++) {
			mc->sets[i].reg = pp ? muxr1[i] : muxr0[i];
			mc->sets[i].value = muxv[i];
			mc->sets[i].subsys_id = 9;
		}
	}
}

/*
 * Two-pixel-pipe (dual) primitive for the LINEAR NV12-source 1:1 ops (blit / CSC /
 * fmtconv) -- a clone of mdp_fill_dual_pipe_8195 (the MM21 detile dual) with the
 * RDMA (c0) converted MM21-block -> LINEAR and the WROT (c9) parameterized via
 * struct mdp_dual_wrot. Relay geometry c1-c8, seam overlaps, per-pipe type/id/mux
 * and the WROT outx are op-independent and copied verbatim from the detile dual.
 * Each pipe is a 10-component nsub=1 config; pp0 owns output cols [0,pw), pp1
 * [pw,w); read windows overlap at the seam (pw+16 / pw+8) for the filter margin.
 * The per-half LINEAR read offset is rx BYTES (Y + interleaved UV), replacing the
 * detile block words u[25]=ta*32 / u[26..27]=ta*16 / u[31]=itx. RE + validation:
 * plan/dual-pipe-kd-spec-mt8195.md; SCP blit 2560x1088 pp1 oracle byte-confirmed.
 */
static void mdp_fill_dual_pipe_8195_linear(struct img_config_8195 *cfg, int pp,
					   u32 w, u32 h, u32 istride,
					   u32 iy, u32 ic, u32 src_ctrl,
					   const struct mdp_dual_wrot *wr,
					   const struct mdp_dual_src *src)
{
	u32 pw = w / 2;
	u32 outx = pp ? pw : 0;
	u32 rw = pp ? (pw + 8) : (pw + 16);	/* read (crop) width */
	u32 sw = pw + 8;			/* rsz width */
	u32 rx = pp ? (pw - 8) : 0;		/* linear read start col = byte offset */
	u32 itx = pp ? (rx & 15u) : 0;		/* in-16-block x (c4 seam phase) */
	static const u8 ty[10] = { 10, 15, 17, 27, 31, 35, 39, 43, 49, 55 };
	static const u8 idc[10] = { 2, 2, 4, 2, 2, 2, 2, 2, 2, 2 };
	static const u32 muxr0[7] = { 0xf50, 0xf4c, 0xf38, 0xf44, 0xf40, 0xf90, 0xf3c };
	static const u32 muxr1[7] = { 0xf7c, 0xf78, 0xf64, 0xf70, 0xf6c, 0xf60, 0xf68 };
	static const u32 muxv[7] = { 1, 1, 0, 0, 1, 0, 0 };
	struct img_mmsys_ctrl *mc;
	int i;

	memset(cfg, 0, sizeof(*cfg));
	cfg->num_components = 10;
	cfg->num_subfrms = 1;

	for (i = 0; i < 10; i++) {
		struct img_compparam_8195 *p = &cfg->components[i];
		struct img_comp_frame_8195 *f = &p->frame;
		struct img_comp_subfrm_8195 *s = &p->subfrms[0];
		u32 *u = (u32 *)&p->rdma;

		p->type = ty[i] + pp;
		p->id = idc[i] + pp;
		p->input = 0; p->num_outputs = 1; p->outputs[0] = 0;
		p->num_subfrms = 1;
		s->out_vertical = 1;

		f->in_width = w; f->in_height = h; f->out_width = w; f->out_height = h;
		f->in_total_width = sw; f->out_total_width = sw;
		f->crop.left = outx; f->crop.width = pw; f->crop.height = h;
		s->in.left = outx; s->in.right = outx + pw - 1; s->in.bottom = h - 1;
		s->out.left = outx; s->out.right = outx + pw - 1; s->out.bottom = h - 1;

		switch (i) {
		case 0:	/* RDMA: LINEAR NV12 per-pipe column window */
			f->crop.left = rx; f->crop.width = rw; f->crop.height = h;
			f->in_total_width = rw; f->out_total_width = sw;
			s->in.left = rx; s->in.right = rx + rw - 1; s->in.bottom = h - 1;
			s->out.left = rx; s->out.right = rx + sw - 1;
			u[0] = src_ctrl; u[2] = 0x1010; u[3] = iy;
			u[6] = iy + istride * h;
			u[9] = istride;
			if (src && src->single_plane) {
				/* RGB / packed-4:2:2: ONE packed plane, no plane-1/2
				 * read (no phantom-plane IOMMU fault); RGB adds the
				 * read-CSC. Per-half column offset scales by bpp. */
				u[14] = src->colortran ? 0x10000 : 0;
				if (pp)
					u[25] = rx * (src->xbpp ? src->xbpp : 1);
			} else if (src && src->three_plane) {
				/* 3-plane I420/YV12 (4:2:0 half-height chroma) or
				 * YM16/YM61 (4:2:2 full-height): U=ic, V=iova2;
				 * chroma pitch istride/2, per-half offset rx/2. */
				u32 icst = istride / 2;
				u32 ch = src->chroma_full_h ? h : h / 2;

				u[4] = ic; u[5] = src->iova2;
				u[7] = ic + icst * ch;
				u[8] = src->iova2 + icst * ch;
				u[11] = icst;
				if (pp) { u[25] = rx; u[26] = rx / 2; u[27] = rx / 2; }
			} else {
				/* 2-plane NV12 (4:2:0) / NV16 (4:2:2, chroma_full_h).
				 * Byte-for-byte the proven NV12/NV16 dual read. */
				u[4] = ic;
				u[7] = ic + istride *
					((src && src->chroma_full_h) ? h : h / 2);
				u[11] = istride;
				if (pp) { u[25] = rx; u[26] = rx; u[27] = rx; }
			}
			u[15] = 0x03000020; u[16] = 0xe800; u[17] = 0x9c00;
			u[18] = 0x01000014; u[19] = 0x7400; u[20] = 0x4c00;
			u[29] = (h << 16) | rw; u[30] = (h << 16) | sw;
			break;
		case 1:	/* RSZ */
			f->crop.left = rx; f->crop.width = sw;
			s->in.left = rx; s->in.right = rx + sw - 1;
			s->out.left = rx; s->out.right = rx + sw - 1;
			u[0] = 1; u[1] = 7; u[2] = (pp ? w : sw) << 16; u[3] = h << 16;
			break;
		case 2:	/* relay */
			f->in_total_width = 0; f->out_total_width = 0;
			f->crop.left = 0; f->crop.width = 0; f->crop.height = 0;
			memset(&s->in, 0, sizeof(s->in));
			memset(&s->out, 0, sizeof(s->out));
			break;
		case 3:
			f->crop.left = 0; f->crop.width = 0; f->crop.height = 0;
			s->in.left = rx; s->in.right = rx + sw - 1;
			s->out.left = rx; s->out.right = rx + sw - 1;
			u[0] = 0x10000000; u[1] = 1; u[2] = pp ? rx : 0;
			u[3] = (h << 16) | sw; u[4] = (sw - 1) << 16; u[5] = (h - 1) << 16;
			u[7] = sw - 1; u[8] = pp ? 0x60 : 0x20; u[9] = 0x200;
			break;
		case 4:
			f->crop.left = 0; f->crop.width = 0; f->crop.height = 0;
			f->in_total_width = sw; f->out_total_width = pw;
			s->in.left = rx; s->in.right = rx + sw - 1;
			s->out.left = outx; s->out.right = outx + pw - 1;
			u[0] = 0x80; u[1] = 1; u[2] = (sw << 16) | h; u[3] = (pw << 16) | h;
			if (pp) u[4] = itx << 16;
			break;
		case 5:	/* byp */
			f->bypass = 1;
			f->in_total_width = pw; f->out_total_width = pw;
			f->crop.left = outx; f->crop.width = pw; f->crop.height = h;
			u[15] = pp ? 0x1000 : 0x3000;
			u[16] = (h << 16) | pw; u[17] = (h << 16) | pw; u[26] = (h << 16) | pw;
			break;
		case 6:	/* byp */
			f->bypass = 1;
			f->in_total_width = pw; f->out_total_width = pw;
			f->crop.left = outx; f->crop.width = pw; f->crop.height = h;
			u[0] = 1; u[1] = (pw << 16) | h; u[2] = (pw << 16) | h;
			u[4] = (pw - 1) << 16; u[5] = (h - 1) << 16;
			break;
		case 7:
			f->crop.left = 0; f->crop.width = 0; f->crop.height = 0;
			f->in_total_width = pw; f->out_total_width = pw;
			u[0] = 3; u[1] = pw; u[2] = h;
			break;
		case 8:
			f->in_total_width = pw; f->out_total_width = pw;
			u[0] = ((h - 1) << 16) | pw;
			break;
		case 9:	/* WROT: parameterized per-op write */
			f->in_total_width = pw; f->out_total_width = pw;
			u[0] = wr->iova[0]; u[1] = wr->iova[1]; u[2] = wr->iova[2];
			u[3] = wr->control;
			u[4] = wr->stride[0]; u[5] = wr->stride[1]; u[6] = wr->stride[2];
			u[7] = wr->mat_ctrl;
			u[8] = 0x400; u[10] = wr->pre_ultra;
			/* Per-half output-plane VIDO offset. pp0 = the single-pipe base
			 * (0 for identity blit/CSC/fmtconv). pp1 applies the per-plane pw
			 * term outx*ofmul>>ofshr; an H-mirroring xform mode (pp1_reverse:
			 * pp1 writes the LEFT output half) SUBTRACTS it from base instead of
			 * adding. Byte-confirmed vs the v23042 SCP 2560x1088 dual oracle. */
			u[18] = wr->base_ofst[0];
			u[19] = wr->base_ofst[1];
			u[20] = wr->base_ofst[2];
			if (pp) {
				u32 t0, t1, t2;

				if (wr->pp1_explicit_term) {
					t0 = wr->pp1_term[0];
					t1 = wr->pp1_term[1];
					t2 = wr->pp1_term[2];
				} else {
					t0 = (outx * wr->ofmul[0]) >> wr->ofshr[0];
					t1 = (outx * wr->ofmul[1]) >> wr->ofshr[1];
					t2 = (outx * wr->ofmul[2]) >> wr->ofshr[2];
				}
				if (wr->pp1_reverse) {
					u[18] -= t0; u[19] -= t1; u[20] -= t2;
				} else {
					u[18] += t0; u[19] += t1; u[20] += t2;
				}
			}
			u[21] = (h << 16) | pw; u[22] = (h << 16) | pw;
			u[24] = (pw << 16) | wr->main_buf;
			break;
		}
	}

	mc = &cfg->ctrls[0];
	mc->num_sets = 7;
	for (i = 0; i < 7; i++) {
		mc->sets[i].reg = pp ? muxr1[i] : muxr0[i];
		mc->sets[i].value = muxv[i];
		mc->sets[i].subsys_id = 9;
	}
}

/*
 * >1920 dual-pipe DIM-SWAPPING rotate (rot90 / rot270). Unlike the H-mirror
 * xforms, a 90-degree rotation swaps dims (output W == input H), but the SCP
 * STILL splits the INPUT by columns (pw = iw/2, like blit) -- so the read side
 * (c0-c8) is the linear dual primitive with w=iw,h=ih, and each pipe's WROT
 * writes a half-HEIGHT output band (input columns map to output rows). The
 * per-pipe output offset term is thus pw*ostride (Y) / pw*ostride/2 (interleaved
 * UV) -- too large for the u8 ofmul form, so it is passed explicitly. rot90 ADDS
 * it to base ostride-1; rot270 SUBTRACTS it from base ysz-ostride / csz-ostride
 * (the reversed output band). ostride = output width = ih; the output Y plane is
 * ostride*iw bytes (out_h == iw). Byte-confirmed vs the v23042 SCP 2560x1440
 * pp0+pp1 oracle (mdp_kd_debug cfg8195_snapshot_pp0/_pp1).
 */
static void mdp_fill_dual_rot90_8195(struct img_config *cfgs,
				     const struct img_ipi_frameparam *param,
				     u32 control, bool rot270)
{
	const struct img_image_buffer *in = &param->inputs[0].buffer;
	const struct img_image_buffer *out = &param->outputs[0].buffer;
	u32 iw = in->format.width, ih = in->format.height;
	u32 istride = in->format.plane_fmt[0].stride;
	u32 ostride = out->format.plane_fmt[0].stride;	/* output width == ih */
	u32 pw = iw / 2;
	u32 ysz = ostride * iw;		/* output Y bytes: out_w(ostride) * out_h(iw) */
	u32 csz = ysz / 2;
	struct mdp_dual_wrot wr = {
		.iova = { out->iova[0], out->iova[1], 0 },
		.stride = { ostride, ostride, ostride },
		.control = control, .mat_ctrl = 0,
		.pre_ultra = 0x000d80c4, .main_buf = 0x4000,
		.pp1_explicit_term = true,
		.pp1_term = { pw * ostride, pw * ostride / 2, pw * ostride / 2 },
		.pp1_reverse = rot270,
	};

	if (rot270) {
		wr.base_ofst[0] = ysz - ostride;
		wr.base_ofst[1] = csz - ostride;
		wr.base_ofst[2] = csz - ostride;
	} else {			/* rot90 */
		wr.base_ofst[0] = ostride - 1;
		wr.base_ofst[1] = ostride - 1;
		wr.base_ofst[2] = ostride - 1;
	}

	mdp_fill_dual_pipe_8195_linear(&cfgs[0].config_8195, 0, iw, ih,
		istride, in->iova[0], in->iova[1], 0x60c, &wr, NULL);
	mdp_fill_dual_pipe_8195_linear(&cfgs[1].config_8195, 1, iw, ih,
		istride, in->iova[0], in->iova[1], 0x60c, &wr, NULL);
}

/*
 * Build the dual WROT (dst-write) config for a 1-plane (RGB/GREY/packed-4:2:2)
 * or 2-plane semiplanar YUV dst -- the shared write side of the >1920 dual
 * fmtconv arms (3-plane-src / RGB-src / NV16-src -> non-3-plane dst). control /
 * stride / mat_ctrl and the per-half output offset (ofmul[0]=plane0 bytes-per-
 * pixel, ofmul[1]=1 for a 2-plane interleaved-chroma dst) are the OUTPUT-affecting
 * fields; main_buf (0x4000) + pre_ultra are output-INERT (the <=1920 compute uses
 * 0x4000 for EVERY dst -- incl RGB/GREY -- and is bit-exact; the SCP's 0x2000/
 * 0x2c00 are FIFO-size hints, not pixels). mat_ctrl = the v23042 CSC-select oracle
 * (RGB->RGB 0x41, YUV->RGB 0x61, RGB->YUV 0x81, YUV->YUV 0). 3-plane dsts need the
 * CPU-deinterleave hybrid and are NOT built here.
 */
static void mdp_dual_wrot_for_dst(struct mdp_dual_wrot *wr,
				  const struct img_image_buffer *out,
				  const struct mdp_fmtconv *fp, bool rgb_src)
{
	u32 od = out->format.colorformat;
	u32 pc = MDP_COLOR_GET_PLANE_COUNT(od);
	u32 w = out->format.width;
	u32 bpp0 = w ? out->format.plane_fmt[0].stride / w : 1;

	memset(wr, 0, sizeof(*wr));
	wr->iova[0] = out->iova[0];
	wr->iova[1] = out->iova[1];
	wr->iova[2] = out->iova[2];
	wr->stride[0] = out->format.plane_fmt[0].stride;
	wr->stride[1] = out->format.plane_fmt[1].stride;
	wr->stride[2] = out->format.plane_fmt[2].stride;
	wr->control = fp->control;
	wr->pre_ultra = fp->pre_ultra;
	wr->main_buf = 0x4000;
	wr->mat_ctrl = (rgb_src && fp->rgb) ? 0x41 :
		       fp->rgb ? 0x61 : (rgb_src ? 0x81 : 0);
	wr->ofmul[0] = bpp0 ? bpp0 : 1;
	wr->ofmul[1] = (pc >= 2) ? 1 : 0;
	wr->ofmul[2] = 0;
}

/*
 * Fill BOTH pixel-pipe configs for a >1920 (DUAL_BITBLT) linear 1:1 fmtconv into the
 * (already-reconfigured) 2-plane NV12/NV16 output buffer `out`, driven by the fmtconv
 * row `f`. The per-family source read (src_ctrl + struct mdp_dual_src) is derived from
 * the shared source-of-truth mdp_dual_src_for(); the WROT geometry from
 * mdp_dual_wrot_for_dst(out, f, rgb_src). This is the dual-pipe twin of a single
 * mdp_config_compute_8195_fmtconv(param, cfg, f) call -- the exact body the generic
 * <=1920->dual arm (de1a33c) uses, factored so the 3-plane-multiplanar-dst hybrid arms
 * (mdp_kd_gen3p) can reuse it after reconfiguring their 3-plane dst to NV12/NV16.
 * Caller sets any de-interleave flags (i420_deint / _mp / ym16_deint / _yv12).
 */
static void mdp_fill_dual_fmtconv_8195(struct mdp_vpu_dev *vpu,
				       struct img_ipi_frameparam *param,
				       const struct mdp_fmtconv *f)
{
	const struct img_image_buffer *in = &param->inputs[0].buffer;
	const struct img_image_buffer *out = &param->outputs[0].buffer;
	struct img_config *cfgs = (struct img_config *)vpu->config;
	u32 in_color = in->format.colorformat;
	u32 w = out->format.width, h = out->format.height;
	u32 istride = in->format.plane_fmt[0].stride;
	bool rgb_src = mdp_color_is_rgb_src(in_color);
	struct mdp_dual_src src;
	u32 src_ctrl = mdp_dual_src_for(in_color, f, in, &src);
	u32 ic = src.single_plane ? 0 : in->iova[1];
	struct mdp_dual_wrot wr;

	mdp_dual_wrot_for_dst(&wr, out, f, rgb_src);
	mdp_fill_dual_pipe_8195_linear(&cfgs[0].config_8195, 0, w, h,
		istride, in->iova[0], ic, src_ctrl, &wr, &src);
	mdp_fill_dual_pipe_8195_linear(&cfgs[1].config_8195, 1, w, h,
		istride, in->iova[0], ic, src_ctrl, &wr, &src);
	vpu->detile_nchunks = 1;
}

/*
 * Fill BOTH pixel-pipe configs for a >1920 (DUAL_BITBLT) MM21/MT21 tiled SOURCE ->
 * a non-detile dst (non-NV12 2-plane / packed / RGB / GREY, or a reconfigured-to-NV12
 * 3-plane dst), driven by the fmtconv row `f`. The read side is the BLOCK-DETILE dual
 * (mdp_fill_dual_pipe_8195, ta*32 block addressing) -- the ONLY read that can detile
 * MM21; the WROT geometry is the SAME parameterized write the LINEAR fmtconv dual uses
 * (mdp_dual_wrot_for_dst). MM21 is 4:2:0 like NV12, so the plain dst-row control is
 * correct (a 4:2:0->4:2:2 upsample dst rides the row's uv_ysel bit, orthogonal to the
 * pw=w/2 split); an MM21 source is never RGB, so rgb_src=false. The one-pass
 * detile-read + fmtconv-write combine -- the >1920 twin of the <=1920 mm21conv path.
 * Caller sets any de-interleave flags (i420_deint / _mp) for a 3-plane dst.
 */
static void mdp_fill_dual_mm21conv_8195(struct mdp_vpu_dev *vpu,
					struct img_ipi_frameparam *param,
					const struct mdp_fmtconv *f)
{
	const struct img_image_buffer *in = &param->inputs[0].buffer;
	const struct img_image_buffer *out = &param->outputs[0].buffer;
	struct img_config *cfgs = (struct img_config *)vpu->config;
	u32 w = out->format.width, h = out->format.height;
	u32 inh = in->format.height;	/* MM21 block-padded input height */
	struct mdp_dual_wrot wr;

	mdp_dual_wrot_for_dst(&wr, out, f, false);
	if (inh > h) {	/* block-padded height: vertical-crop dual (num_subfrms=2) */
		mdp_fill_dual_pipe_8195_crop(&cfgs[0].config_8195, 0, w, h, inh,
			in->iova[0], in->iova[1], out->iova[0], out->iova[1], &wr);
		mdp_fill_dual_pipe_8195_crop(&cfgs[1].config_8195, 1, w, h, inh,
			in->iova[0], in->iova[1], out->iova[0], out->iova[1], &wr);
	} else {	/* in==out: RSZ-bypassed detile dual (num_subfrms=1) */
		mdp_fill_dual_pipe_8195(&cfgs[0].config_8195, 0, w, h, inh,
			in->iova[0], in->iova[1], out->iova[0], out->iova[1], &wr);
		mdp_fill_dual_pipe_8195(&cfgs[1].config_8195, 1, w, h, inh,
			in->iova[0], in->iova[1], out->iova[0], out->iova[1], &wr);
	}
	vpu->detile_nchunks = 1;
}

/*
 * Kernel-direct MDP config-compute for mt8183 (single-pipe, MULTI-SUBFRAME
 * MM21->NV12 detile -- port of the SCP/dpframework tile scheduler).
 *
 * mt8183 is single pixel-pipe (pp_used = MDP_PP_USED_1) and uses one path:
 *
 *   RDMA0 -> CCORR0 -> RSZ0(SCL0) -> PATH0_SOUT -> WROT0
 *
 * (5 components). CCORR0 is an identity colour-correction relay sized to the
 * subframe; RSZ0 is bypassed (1:1 -- the vertical inh->h crop is the RDMA's
 * MF_CLIP_SIZE, not the scaler); PATH0_SOUT is a pure routing node.
 *
 * WHY MULTI-SUBFRAME (root cause, RE+HW char 2026-06-06, WIP DIAG b3dac6c71a4e):
 * the mt8183 WROT internal line buffer is only WROT_tile_width=512 px wide
 * (tile_wrot_init in scp.img kukui_scp_v2.0.13324, ARM-Thumb .image.RW @ VMA
 * 0x1f90a: FIFOMaxSize=0x8000 / maxLineCount=64 for NV12 (YUV420) = a 512-wide
 * x 64-line SRAM). The dpframework tile scheduler splits the frame into
 * ceil(w/512) horizontal subframes (each <=512 wide); calWROTSetting
 * (dpframework_prot DpEngineSetting_WROT.cpp) recomputes WROT_main_buf_line_num
 * PER TILE from the tile's WROT_tar_xsize (64 for a <=512-wide tile, 24 for a
 * 1280-wide tile). A single num_subfrms=1 full-width pass overruns that 512x64
 * SRAM -> the HW bands (luma rows 0,16,32,... correct, the rest stale; first
 * wrong column at the 3rd 512-tile boundary for w=1280). The fix is to emit
 * num_subfrm = ceil(w/512) with per-tile RDMA/WROT subframe geometry.
 *
 * The kernel re-runs the WHOLE pipeline once per subframe (mtk-mdp3-cmdq.c
 * mdp_path_config: `for count in [0,num_subfrms)` -> mdp_path_config_subfrm),
 * applying ctrls[count] mux + each component's config_subfrm(count), enabling
 * the mutex (= HW trigger), then waiting each engine's done event. So:
 *  - cfg->num_subfrms = N (= ceil(w/512));
 *  - every participating component sets p->num_subfrms = N (is_output_disabled
 *    skips a component on count >= its num_subfrms);
 *  - each component fills subfrms[0..N-1] (geometry) + engine union subfrms[]
 *    (registers); cfg->ctrls[0..N-1] each carry the mmsys mux (re-applied per
 *    subframe).
 *
 * Per-subframe geometry (tile t covers output cols [x0, x0+tw), x0 = t*512,
 * tw = min(512, w-x0); tiles fall on a 16-px MM21-block boundary since 512%16=0):
 *  - RDMA SRC_OFFSET (MM21 block layout): advancing 16 cols = one 16x32 luma
 *    block = 512 bytes => offset[0] = x0*32; chroma block 16x16 = 256 bytes =>
 *    offset[1]=offset[2] = x0*16. (Matches mt8195 dual-pipe u[25]=ta*32 /
 *    u[26]=ta*16.) The full-frame iova/mf_bkgd/sf_bkgd stay frame-wide; the tile
 *    is selected purely by SRC_OFFSET + MF_SRC_SIZE width = tw.
 *  - WROT VIDO_OFST_ADDR = x0 (Y byte offset; stride=ostride frame-wide), UV
 *    offset = x0; VIDO_IN_SIZE/TAR_SIZE width = tw; VIDO_MAIN_BUF_SIZE line_num
 *    from calWROTSetting(tw).
 *
 * Unlike mt8195 this fills NAMED struct fields of img_config_8183 (IMG_MAX_
 * SUBFRAMES=14, u16 frame bitfields, mdp_rdma_data_8183 has no comp_ctrl /
 * dmabuf_con* block). The field<->register mapping is the mtk-mdp3-comp.c MT8183
 * config_*_frame / config_*_subfrm consumers. RDMA block-mode register values
 * (src_ctrl 0x860c, control 0x10, iova_end/mf_bkgd/sf_bkgd) are the proven
 * mt8195-shared MM21 detile values (RDMA SRC_CON/pitch/end-addr layout is shared
 * between the two platforms; masks SRC_CON 0x03C8FE0F, MF/SF_BKGD 0x001FFFFF).
 *
 * Oracle-less: mt8183's SCP errors -22 (KI-006), so there is no captured config
 * to diff; HW bit-exact vs an ffmpeg/CPU detile is the authoritative gate
 * (COORDINATOR-owned). The vertical-strip handshake the root-cause note posited
 * is NOT a separate vertical subframe split here: the dpframework RDMA/WROT
 * out_tile_height is 8191 (no vertical tiling) -- the 32-line block behaviour is
 * the WROT main_buf line_num striping WITHIN a horizontal subframe, which the
 * per-tile calWROTSetting line_num now supplies. If HW still bands at w<=512
 * (single horizontal subframe) the residual is a per-tile field the coordinator
 * must probe (flagged in the commit body).
 * Full derivation + RE evidence: the internal MDP kernel-direct plan and
 * SCP RE record.
 */

/*
 * Per-tile WROT main-buffer line_num, ported from dpframework_prot
 * DpEngineSetting_WROT.cpp::calWROTSetting for the NV12 (YUV420, 2-plane) case.
 * tar_xsize = tile output width; mt8183 WROT_tile_width=512 => FIFOMaxSize=
 * 512*64=0x8000, maxLineCount=64, int_y_buf_size=512*64, int_uv_buf_size=
 * 512/2*32 (HW_FORMAT 12). Integer-only (kernel has no float); the float ceils
 * in the source are exact here because all operands divide evenly for the tile
 * widths in scope. Returns line_num in [4, 64].
 */
#define MDP_8183_WROT_TILE_WIDTH 512u
#define MDP_8183_WROT_FIFO_MAX   (MDP_8183_WROT_TILE_WIDTH * 64u)	/* 0x8000 */
#define MDP_8183_WROT_MAXLINE    64u

static u32 __maybe_unused mdp_8183_wrot_line_num(u32 tar_xsize)
{
	const u32 fifo = MDP_8183_WROT_FIFO_MAX;
	const u32 int_y_buf_size  = MDP_8183_WROT_TILE_WIDTH * 64u;
	const u32 int_uv_buf_size = (MDP_8183_WROT_TILE_WIDTH / 2u) * 32u;
	u32 line_num, c1, c2, tmp, ybw, ybu, uvw, uvl, uvbw, uvbu;

	if (!tar_xsize)
		return MDP_8183_WROT_MAXLINE;

	/* c1 = floor(FIFO / (tar*2)) * 2; c2 = ceil(tar / c1);
	 * tmp = ceil(tar / (c2*4)) * 4 */
	c1  = (fifo / (tar_xsize * 2u)) * 2u;
	if (!c1)
		c1 = 2u;
	c2  = DIV_ROUND_UP(tar_xsize, c1);
	tmp = DIV_ROUND_UP(tar_xsize, c2 * 4u) * 4u;

	if (tmp > tar_xsize)
		line_num = DIV_ROUND_UP(tar_xsize, 4u) * 4u;
	else
		line_num = tmp;
	if (line_num > MDP_8183_WROT_MAXLINE)
		line_num = MDP_8183_WROT_MAXLINE;

	/* shrink by 4 until both Y and UV internal buffers fit */
	while (line_num >= 4u) {
		ybw = DIV_ROUND_UP(tar_xsize, line_num) * line_num;
		ybu = ybw * line_num;
		if (ybu > int_y_buf_size) {
			line_num -= 4u;
			continue;
		}
		/* NV12: H+V subsample => uv_blk_width = tar/2, uv_blk_line = ln/2 */
		uvw = tar_xsize >> 1;
		uvl = line_num >> 1;
		if (!uvl)
			break;
		uvbw = DIV_ROUND_UP(uvw, uvl) * uvl;
		uvbu = uvbw * uvl;
		if (uvbu > int_uv_buf_size) {
			line_num -= 4u;
			continue;
		}
		break;
	}
	/* HW-iter sweep: force the per-tile WROT main_buf line_num to mdp_kd_wrot_lines
	 * (0 = the calWROTSetting-computed value). Sweep the WROT FIFO strip height in a
	 * single boot. Leading hypothesis: 32 (MM21 block height) so WROT drains per the
	 * RDMA-delivered 32-line block, killing the period-32 stale rows. */
	if (mdp_kd_wrot_lines > 0) {
		line_num = (u32)mdp_kd_wrot_lines;
		if (line_num > MDP_8183_WROT_MAXLINE)
			line_num = MDP_8183_WROT_MAXLINE;
	}
	return line_num ? line_num : 4u;
}

/* mt8183 single-pipe detile, per component: {type (inner mdp_comp_id), id (alias)}. */
static const struct { u16 type, id; } mdp_dt_comp_8183[5] = {
	{ 11, 0 },	/* MT8183_MDP_COMP_RDMA0 */
	{ 13, 0 },	/* MT8183_MDP_COMP_CCORR0 */
	{ 14, 0 },	/* MT8183_MDP_COMP_RSZ0 (SCL0) */
	{ 18, 0 },	/* MT8183_MDP_COMP_PATH0_SOUT (routing relay, no regs) */
	{ 20, 0 },	/* MT8183_MDP_COMP_WROT0 */
};

/*
 * mmsys mux routing for the mt8183 detile path (geometry-independent).
 * subsys_id = 1 (SUBSYS_1400XXXX = mmsys @ 0x14000000, mt8183-gce.h); reg is the
 * offset within that subsys. Values pick the RDMA0->CCORR0->RSZ0->PATH0_SOUT->
 * WROT0 mux index per the dpframework mt8183 DpPathConnectionPlatform map:
 *   RDMA0_MOUT_EN (0xf84): out -> CCORR0   (mout idx 0 -> value 1)
 *   CCORR_SEL_IN  (0xff0): in  <- RDMA0    (sel idx 1 -> value 1)
 *   CCORR_SOUT_SEL(0xff4): out -> SCL0     (sout idx 1 -> value 1)
 *   PRZ0_SEL_IN   (0xfc0): in  <- CCORR0   (sel idx 1 -> value 1)
 *   PRZ0_MOUT_EN  (0xf8c): out -> PATH0_SOUT (mout idx 0 -> value 1)
 *   PATH0_SEL_IN  (0xfe0): in  <- SCL0     (sel idx 0 -> value 0)
 *   PATH0_SOUT_SEL(0xfa8): out -> WROT0    (sout idx 0 -> value 0)
 *   WROT0_SEL_IN  (0xfd0): in  <- PATH0_SOUT (sel idx 0 -> value 0)
 */
static const struct { u32 reg, value; } mdp_dt_mux_8183[8] = {
	{ 0xf84, 1 },	/* MDP_RDMA0_MOUT_EN  -> CCORR0 */
	{ 0xff0, 1 },	/* MDP_CCORR_SEL_IN   <- RDMA0 */
	{ 0xff4, 1 },	/* MDP_CCORR_SOUT_SEL -> SCL0 */
	{ 0xfc0, 1 },	/* MDP_PRZ0_SEL_IN    <- CCORR0 */
	{ 0xf8c, 1 },	/* MDP_PRZ0_MOUT_EN   -> PATH0_SOUT */
	{ 0xfe0, 0 },	/* MDP_PATH0_SEL_IN   <- SCL0 */
	{ 0xfa8, 0 },	/* MDP_PATH0_SOUT_SEL -> WROT0 */
	{ 0xfd0, 0 },	/* MDP_WROT0_SEL_IN   <- PATH0_SOUT */
};
#define MDP_MT8183_MMSYS_SUBSYS_ID 1

static void mdp_config_compute_8183(struct mdp_dev *mdp,
				    struct img_ipi_frameparam *param,
				    struct img_config *config,
				    u32 tile_base, u32 tile_count)
{
	struct img_config_8183 *cfg = &config->config_8183;
	const struct img_image_buffer *in = &param->inputs[0].buffer;
	const struct img_image_buffer *out = &param->outputs[0].buffer;
	u32 w = out->format.width;
	u32 h = out->format.height;
	u32 inh = in->format.height;		/* block-aligned by mdp_set_src_config */
	u32 ostride = out->format.plane_fmt[0].stride;	/* output NV12 row stride */
	u32 iy = in->iova[0], ic = in->iova[1];
	u32 oy = out->iova[0], oc = out->iova[1];
	u32 croph = ALIGN(h, 32);		/* RDMA crop height (RDMA reads inh, crops) */
	u32 aw = ALIGN(w, 64);			/* MM21 read-side aligned width (luma stride = aw*32) */
	/* HW-validated WROT buffer geometry: 64 wide x 32 tall (PSNR=inf bit-exact on
	 * demo-mt8183-chromebook for single-tile, multi-tile, multi-strip and full-grid
	 * frames). Wider (>64) shifts rows; taller (>32) bands -- both confirmed by
	 * bisection. The mdp_kd_tile_w / mdp_kd_wrot_lines params override for HW iter. */
	u32 strip = mdp_kd_wrot_lines > 0 ? mdp_kd_wrot_lines : 32;	/* vertical strip lines (WROT buf height) */
	u32 tilew = mdp_kd_tile_w > 0 ? mdp_kd_tile_w : 64;		/* horizontal tile width (WROT buf width) */
	u32 nh = DIV_ROUND_UP(w, tilew);	/* horizontal tiles -- global tile column = g % nh */
	struct img_mmsys_ctrl *mc;
	int i;
	u32 t;

	/* Fill subframes for the GLOBAL tile window [tile_base, tile_base+tile_count)
	 * of the ceil(w/tilew) x ceil(h/strip) grid. tile_count <= IMG_MAX_SUBFRAMES_8183.
	 * A frame whose grid exceeds 14 tiles is chunked by mdp_m2m_device_run into
	 * several sequential GCE submissions, each calling this with a different
	 * tile_base; every chunk shares the same in/out dma-buf iova (the per-tile
	 * SRC_OFFSET / VIDO_OFST deltas do the addressing), so the output accumulates. */
	if (tile_count > IMG_MAX_SUBFRAMES_8183)
		tile_count = IMG_MAX_SUBFRAMES_8183;

	memset(cfg, 0, sizeof(*cfg));
	cfg->num_components = 5;
	cfg->num_subfrms = tile_count;

	for (i = 0; i < 5; i++) {
		struct img_compparam_8183 *p = &cfg->components[i];
		struct img_comp_frame_8183 *f = &p->frame;

		p->type = mdp_dt_comp_8183[i].type;
		p->id = mdp_dt_comp_8183[i].id;
		p->input = 0;
		p->num_outputs = 1;
		p->outputs[0] = 0;
		p->num_subfrms = tile_count;	/* participate in every subframe of this chunk */

		/* Frame-level (geometry-independent of tile): full output frame. The
		 * RDMA crops inh->croph vertically; the RSZ is bypassed 1:1. */
		f->in_width = w; f->in_height = h;
		f->out_width = w; f->out_height = h;
		f->in_total_width = w; f->out_total_width = w;
		f->crop.width = w; f->crop.height = h;

		switch (i) {
		case 0:	/* RDMA0: reads MM21 (inh tall), crops to ALIGN(h,32).
			 * config_rdma_frame (MT8183): src_ctrl/control/iova/iova_end/
			 * mf_bkgd/sf_bkgd/transform are FRAME-level (written once); the
			 * per-tile SRC_OFFSET/MF_SRC_SIZE/MF_CLIP_SIZE come from
			 * config_rdma_subfrm(count) -> rdma.subfrms[count]. */
			f->in_height = inh;
			f->crop.height = croph;
			p->rdma.src_ctrl = 0x860c;	/* block-mode (bit15) MM21 420_BLKP */
			/* MDP_RDMA_CON simple_mode=1 (0x10): MM21 block-mode needs it or the
			 * RDMA stalls in multi-buffer mode (no EOF -> GCE hang). RE-proven
			 * (DpEngine_RDMA.cpp DP_COLOR_420_BLKP -> simple_mode=1); fixed the
			 * 2026-06-06 first-HW-run hang (ATTEMPT 1 had control=0). */
			p->rdma.control = 0x10;
			p->rdma.iova[0] = iy;
			p->rdma.iova[1] = ic;
			p->rdma.iova_end[0] = iy + aw * inh;		/* base + luma plane size */
			p->rdma.iova_end[1] = ic + aw * inh / 2;	/* base + chroma plane size */
			p->rdma.mf_bkgd = aw * 32;	/* luma tile-row stride (bytes), frame-wide */
			p->rdma.sf_bkgd = aw * 16;	/* chroma tile-row stride (bytes), frame-wide */
			break;
		case 2:	/* RSZ0 (SCL0): BYPASSED 1:1. RE-proven firmware-faithful: the
			 * kukui_scp rsz8183/DpEngine_RSZ.cpp onConfigFrame bypasses (PRZ_
			 * ENABLE bit0=0) for a leaf 1:1 in==out==crop detile. ATTEMPT 3's
			 * RSZ-active table-27 6-tap was a smoothing filter (HW-wrong). The
			 * inh->h vertical crop is the RDMA MF_CLIP_SIZE, not the RSZ. */
			f->bypass = 1;
			break;
		case 4:	/* WROT0: writes linear NV12. config_wrot_frame (MT8183):
			 * iova/control/stride/mat_ctrl are FRAME-level; per-tile
			 * offset/src/clip/main_buf come from config_wrot_subfrm(count). */
			p->wrot.iova[0] = oy;
			p->wrot.iova[1] = oc;
			p->wrot.control = 0x500c;	/* VIDO_CTRL (NV12 out, no rotate) */
			p->wrot.stride[0] = ostride;	/* Y pitch (frame-wide; tile selected by offset) */
			p->wrot.stride[1] = ostride;	/* UV pitch */
			break;
		/* case 1 (CCORR0 identity relay) + case 3 (PATH0_SOUT routing dummy):
		 * no frame-level register fills; CCORR derives MDP_CCORR_SIZE from the
		 * per-tile subfrms[count].in region in config_ccorr_subfrm. */
		}

		/* Per-subframe geometry: a 2D grid of (horizontal tilew-col tile) x
		 * (vertical `strip`-line band), row-major GLOBAL tile index g = ty*nh + tx.
		 * This chunk fills local slots [0,tile_count) from global tiles
		 * [tile_base, tile_base+tile_count); tx/ty/x0/y0 derive from g so every
		 * chunk's WROT/RDMA offsets address the correct rectangle of the shared
		 * dma-buf. Each tile <= one WROT buffer fill (no internal banding). */
		for (t = 0; t < tile_count; t++) {
			u32 g = tile_base + t;
			u32 tx = g % nh, ty = g / nh;
			u32 x0 = tx * tilew;		/* col start (16-aligned) */
			u32 tw = min(tilew, w - x0);	/* tile width */
			u32 y0 = ty * strip;				/* row start (32-aligned) */
			u32 sh = min(strip, h - y0);			/* tile height <=strip */
			u32 csh = ALIGN(sh, 32);			/* RDMA source height (32-line blocks) */
			struct img_comp_subfrm_8183 *s = &p->subfrms[t];

			/* relay-engine region = this tile (tw x sh); the absolute frame
			 * position is carried by the RDMA SRC_OFFSET / WROT VIDO_OFST. */
			s->in.left = x0; s->in.right = x0 + tw - 1;
			s->in.top = 0;   s->in.bottom = sh - 1;
			s->out.left = x0; s->out.right = x0 + tw - 1;
			s->out.top = 0;  s->out.bottom = sh - 1;
			s->out_horizontal = tx;
			s->out_vertical = ty;

			switch (i) {
			case 0:	/* RDMA0: read the (x0,y0) tile, tw x csh, from MM21 */
				s->in.bottom = csh - 1;
				/* SRC_OFFSET = vertical tile-rows + horizontal tile-cols:
				 * luma 16x32 tile = 512 B (=> aw*32 per tile-row, x0*32 per col);
				 * chroma 16x16 tile = 256 B (=> aw*16 per tile-row, x0*16 per col). */
				p->rdma.subfrms[t].offset[0] = (y0 / 32) * aw * 32 + x0 * 32;
				p->rdma.subfrms[t].offset[1] = (y0 / 32) * aw * 16 + x0 * 16;
				p->rdma.subfrms[t].offset[2] = (y0 / 32) * aw * 16 + x0 * 16;
				p->rdma.subfrms[t].src  = (csh << 16) | tw;	/* MF_SRC_SIZE */
				p->rdma.subfrms[t].clip = (sh << 16) | tw;	/* MF_CLIP_SIZE */
				p->rdma.subfrms[t].clip_ofst = 0;
				break;
			case 4:	/* WROT0: write the (x0,y0) tile, tw x sh, to linear NV12 */
				/* VIDO_OFST: Y = y0*stride + x0; UV = (y0/2)*stride + x0
				 * (interleaved CbCr, 1 B/luma-col). */
				p->wrot.subfrms[t].offset[0] = y0 * ostride + x0;
				p->wrot.subfrms[t].offset[1] = (y0 / 2) * ostride + x0;
				p->wrot.subfrms[t].offset[2] = (y0 / 2) * ostride + x0;
				p->wrot.subfrms[t].src  = (sh << 16) | tw;	/* VIDO_IN_SIZE */
				p->wrot.subfrms[t].clip = (sh << 16) | tw;	/* VIDO_TAR_SIZE */
				p->wrot.subfrms[t].clip_ofst = 0;
				/* WROT main_buf: blk_width=tw, line_num = the WROT buffer-fill
				 * (default sh; mdp_kd_linenum overrides to test wide tiles). */
				p->wrot.subfrms[t].main_buf = (tw << 16) |
					((mdp_kd_linenum > 0 ? mdp_kd_linenum : sh) << 8);
				break;
			/* RSZ0 (bypassed) / CCORR0 / PATH0_SOUT: geometry region only. */
			}
		}
	}

	/* ctrls[count]: the mmsys mux is re-applied (and then cleared) per subframe.
	 * Fill one identical ctrl set per subframe (subsys 1 = mmsys @ 0x14000000). */
	for (t = 0; t < tile_count; t++) {
		mc = &cfg->ctrls[t];
		mc->num_sets = ARRAY_SIZE(mdp_dt_mux_8183);
		for (i = 0; i < ARRAY_SIZE(mdp_dt_mux_8183); i++) {
			mc->sets[i].reg = mdp_dt_mux_8183[i].reg;
			mc->sets[i].value = mdp_dt_mux_8183[i].value;
			mc->sets[i].subsys_id = MDP_MT8183_MMSYS_SUBSYS_ID;
		}
	}

	if (mdp_kd_debug_on()) {
		struct device *dev = &mdp->pdev->dev;
		const struct mdp_rdma_data_8183 *rd = &cfg->components[0].rdma;
		const struct mdp_wrot_data_8183 *wr = &cfg->components[4].wrot;

		dev_info(dev, "MDPKD-8183 cfg: w=%u h=%u inh=%u aw=%u croph=%u ostride=%u ncomp=%u nsub=%u\n",
			 w, h, inh, aw, croph, ostride, cfg->num_components,
			 cfg->num_subfrms);
		dev_info(dev, "MDPKD-8183 RDMA(c0 type=%u id=%u): src_ctrl=%#x control=%#x iova=[%#x,%#x] iova_end=[%#x,%#x] mf_bkgd=%#x sf_bkgd=%#x transform=%#x nsub=%u\n",
			 cfg->components[0].type, cfg->components[0].id,
			 rd->src_ctrl, rd->control, rd->iova[0], rd->iova[1],
			 rd->iova_end[0], rd->iova_end[1], rd->mf_bkgd, rd->sf_bkgd,
			 rd->transform, cfg->components[0].num_subfrms);
		dev_info(dev, "MDPKD-8183 WROT(c4 type=%u id=%u): iova=[%#x,%#x] control=%#x stride=[%#x,%#x] byp(rsz)=%u\n",
			 cfg->components[4].type, cfg->components[4].id,
			 wr->iova[0], wr->iova[1], wr->control, wr->stride[0],
			 wr->stride[1], cfg->components[2].frame.bypass);
		/* per-subframe (per horizontal tile) dump */
		for (t = 0; t < cfg->num_subfrms; t++) {
			const struct img_comp_subfrm_8183 *gs = &cfg->components[0].subfrms[t];

			dev_info(dev, "MDPKD-8183 sub[%u]: in=[%d,%d] out=[%d,%d] outh=%d | RDMA off0=%#x off1=%#x src=%#x clip=%#x clipofst=%#x | WROT off0=%#x src=%#x clip=%#x main_buf=%#x(blkw=%u line_num=%u)\n",
				 t, gs->in.left, gs->in.right, gs->out.left, gs->out.right,
				 gs->out_horizontal,
				 rd->subfrms[t].offset[0], rd->subfrms[t].offset[1],
				 rd->subfrms[t].src, rd->subfrms[t].clip,
				 rd->subfrms[t].clip_ofst,
				 wr->subfrms[t].offset[0], wr->subfrms[t].src,
				 wr->subfrms[t].clip, wr->subfrms[t].main_buf,
				 (wr->subfrms[t].main_buf >> 16) & 0x1fff,
				 (wr->subfrms[t].main_buf >> 8) & 0x7f);
		}
		mc = &cfg->ctrls[0];
		for (i = 0; i < (int)mc->num_sets; i++)
			dev_info(dev, "MDPKD-8183 mux[%d]: reg=%#x val=%#x subsys=%u\n",
				 i, mc->sets[i].reg, mc->sets[i].value,
				 mc->sets[i].subsys_id);
	}
}

/*
 * Kernel-direct CSC (NV12 -> RGB24) config-compute for mt8183 -- a first
 * HW-validation DRAFT, adapted ~80% from mdp_config_compute_8183 (the detile).
 * Ported verbatim from the mt8183 CSC draft (8d56a26b639f) so both SoCs share one
 * worktree. Same single pixel-pipe topology: RDMA0 -> CCORR0 -> RSZ0(bypass) ->
 * PATH0_SOUT -> WROT0 (5 components, mdp_dt_comp_8183[] / mdp_dt_mux_8183[]).
 * num_subfrms = 1; the conversion is ONE register: WROT mat_ctrl = 0x61.
 *
 * Diffs vs the detile (mdp_config_compute_8183):
 *  - RDMA reads LINEAR NV12 (src_ctrl=0x060c, block-mode bit CLEAR), control=0,
 *    mf/sf_bkgd = the linear istride, iova_end linear.
 *  - WROT writes RGB24 (1 plane): control=RGB888, stride[0]=RGB row stride,
 *    iova[1..2]=0, mat_ctrl=0x61.
 * Every guessed/adapted field is tagged CSC-HW-ITER (no mt8183 oracle).
 */
/* mt8183 WROT RGB single-pass ceiling: width AND height are each 8-bit-bounded
 * (<=255 px), HW-bisected 2026-06-19 (254 OK, 256 HANGS in both dims). So a frame
 * wider OR taller than 255 is split into a 2D grid of <=254x254 tiles, exactly
 * like the MM21 detile splits into 64x32 tiles -- only the RDMA mode (linear vs
 * block) and the WROT format (RGB+matrix vs NV12) differ. 254 is even (NV12 4:2:0
 * chroma needs an even tile origin) and the largest value that fits the 8-bit
 * blk_width / line_num WROT fields. */
#define MDP_CSC_8183_TILE	254

/* Kernel-direct CSC (NV12 -> RGB24) config-compute for mt8183, multi-tile.
 * Fills the GLOBAL tile window [tile_base, tile_base+tile_count) of the
 * ceil(w/254) x ceil(h/254) grid (tile_count <= IMG_MAX_SUBFRAMES_8183). A frame
 * whose grid exceeds 14 tiles is chunked by mdp_m2m_device_run into several GCE
 * submissions, each calling this with a different tile_base into the SAME output
 * dma-buf (the per-tile SRC_OFFSET / VIDO_OFST deltas address the rectangle).
 * Same single pixel-pipe (RDMA0->CCORR0->RSZ0(bypass)->PATH0_SOUT->WROT0) as the
 * detile; the conversion is the one WROT register mat_ctrl=0x61. */
static void mdp_config_compute_8183_csc(struct mdp_dev *mdp,
					struct img_ipi_frameparam *param,
					struct img_config *config,
					u32 tile_base, u32 tile_count)
{
	struct img_config_8183 *cfg = &config->config_8183;
	const struct img_image_buffer *in = &param->inputs[0].buffer;
	const struct img_image_buffer *out = &param->outputs[0].buffer;
	u32 w = out->format.width;
	u32 h = out->format.height;
	u32 istride = in->format.plane_fmt[0].stride;	/* linear NV12 Y pitch (bytes) */
	u32 ostride = out->format.plane_fmt[0].stride;	/* RGB24 row stride (bytes) */
	u32 iy = in->iova[0], ic = in->iova[1];
	u32 oy = out->iova[0];
	u32 tile = MDP_CSC_8183_TILE;
	u32 nh = DIV_ROUND_UP(w, tile);			/* horizontal tiles: global col = g % nh */
	struct img_mmsys_ctrl *mc;
	int i;
	u32 t;

	if (tile_count > IMG_MAX_SUBFRAMES_8183)
		tile_count = IMG_MAX_SUBFRAMES_8183;

	memset(cfg, 0, sizeof(*cfg));
	cfg->num_components = 5;
	cfg->num_subfrms = tile_count;

	for (i = 0; i < 5; i++) {
		struct img_compparam_8183 *p = &cfg->components[i];
		struct img_comp_frame_8183 *f = &p->frame;

		p->type = mdp_dt_comp_8183[i].type;
		p->id = mdp_dt_comp_8183[i].id;
		p->input = 0;
		p->num_outputs = 1;
		p->outputs[0] = 0;
		p->num_subfrms = tile_count;	/* participate in every subframe of this chunk */

		/* Frame-level: full output frame, 1:1 (in dims == out dims). The
		 * per-tile geometry lives in subfrms[t]. */
		f->in_width = w; f->in_height = h;
		f->out_width = w; f->out_height = h;
		f->in_total_width = w; f->out_total_width = w;
		f->crop.width = w; f->crop.height = h;

		switch (i) {
		case 0:	/* RDMA0: reads LINEAR NV12 (not block-mode). Frame-level
			 * src_ctrl/control/iova/iova_end/mf_bkgd/sf_bkgd; per-tile
			 * SRC_OFFSET/MF_SRC_SIZE/MF_CLIP_SIZE below. */
			/* linear-NV12 RDMA SRC_CON: default 0x060c = the working detile
			 * src_ctrl (0x860c) with the block-mode bit cleared -- the more
			 * accurate read (46 dB vs ideal BT.601; mt8195's 0xc tracks its
			 * own ~33 dB oracle). Live-sweepable via mdp_csc_srcctrl. */
			p->rdma.src_ctrl = mdp_csc_srcctrl;
			/* MDP_RDMA_CON default 0x10 = detile simple_mode (single-buffer);
			 * control=0 stalls in multi-buffer mode (no EOF -> GCE hang). */
			p->rdma.control = mdp_csc_rdmacon;
			p->rdma.iova[0] = iy;
			p->rdma.iova[1] = ic;
			/* linear plane sizes: Y = istride*h, chroma (4:2:0) = istride*h/2 */
			p->rdma.iova_end[0] = iy + istride * h;
			p->rdma.iova_end[1] = ic + istride * h / 2;
			p->rdma.mf_bkgd = istride;	/* linear Y pitch (frame-wide) */
			p->rdma.sf_bkgd = istride;	/* linear chroma pitch (= Y pitch) */
			break;
		case 2:	/* RSZ0 (SCL0): bypassed 1:1 (same as detile). */
			f->bypass = 1;
			break;
		case 4:	/* WROT0: writes linear RGB24 (1 plane) + does the CSC.
			 * Frame-level iova/control/stride/mat_ctrl; per-tile
			 * VIDO_OFST/src/clip/main_buf below. */
			p->wrot.iova[0] = oy;
			p->wrot.iova[1] = 0;	/* RGB24 single plane */
			p->wrot.iova[2] = 0;
			p->wrot.control = 0xa0005101;	/* VIDO_CTRL RGB888 (from the mt8195 CSC oracle) */
			p->wrot.stride[0] = ostride;	/* RGB row stride (frame-wide; tile via offset) */
			p->wrot.stride[1] = 0;
			p->wrot.stride[2] = 0;
			/* VIDO_MAT_CTRL = (int_matrix_sel 6 << 4)|enable = YUV->RGB BT.601 */
			p->wrot.mat_ctrl = 0x61;
			break;
		/* case 1 (CCORR0 identity relay) + case 3 (PATH0_SOUT routing):
		 * no frame-level register fills (same as the detile). */
		}

		/* Per-subframe geometry: a 2D grid of <=254x254 tiles, row-major
		 * global tile index g = ty*nh + tx. This chunk fills local slots
		 * [0,tile_count) from global tiles [tile_base, tile_base+tile_count);
		 * every chunk's RDMA/WROT offsets address the correct rectangle of the
		 * shared linear dma-buf. */
		for (t = 0; t < tile_count; t++) {
			u32 g = tile_base + t;
			u32 tx = g % nh, ty = g / nh;
			u32 x0 = tx * tile;		/* col start (even) */
			u32 tw = min(tile, w - x0);	/* tile width <=254 */
			u32 y0 = ty * tile;		/* row start (even) */
			u32 sh = min(tile, h - y0);	/* tile height <=254 */
			struct img_comp_subfrm_8183 *s = &p->subfrms[t];

			/* relay-engine region = this tile; the absolute frame position
			 * is carried by the RDMA SRC_OFFSET / WROT VIDO_OFST. */
			s->in.left = x0; s->in.right = x0 + tw - 1;
			s->in.top = 0;   s->in.bottom = sh - 1;
			s->out.left = x0; s->out.right = x0 + tw - 1;
			s->out.top = 0;  s->out.bottom = sh - 1;
			s->out_horizontal = tx;
			s->out_vertical = ty;

			switch (i) {
			case 0:	/* RDMA0: read the (x0,y0) tile, tw x sh, from LINEAR NV12.
				 * SRC_OFFSET is plain byte addressing (no MM21 tile arithmetic):
				 * Y = y0*istride + x0; UV (interleaved CbCr, 1 B/luma-col,
				 * chroma row y0/2) = (y0/2)*istride + x0. */
				p->rdma.subfrms[t].offset[0] = y0 * istride + x0;
				p->rdma.subfrms[t].offset[1] = (y0 / 2) * istride + x0;
				p->rdma.subfrms[t].offset[2] = (y0 / 2) * istride + x0;
				p->rdma.subfrms[t].src  = (sh << 16) | tw;	/* MF_SRC_SIZE */
				p->rdma.subfrms[t].clip = (sh << 16) | tw;	/* MF_CLIP_SIZE */
				p->rdma.subfrms[t].clip_ofst = 0;
				break;
			case 4:	/* WROT0: write the (x0,y0) tile, tw x sh, to linear RGB24.
				 * VIDO_OFST = y0*ostride + x0*3 (RGB = 3 B/px, single plane). */
				p->wrot.subfrms[t].offset[0] = y0 * ostride + x0 * 3;
				p->wrot.subfrms[t].offset[1] = 0;
				p->wrot.subfrms[t].offset[2] = 0;
				p->wrot.subfrms[t].src  = (sh << 16) | tw;	/* VIDO_IN_SIZE */
				p->wrot.subfrms[t].clip = (sh << 16) | tw;	/* VIDO_TAR_SIZE */
				p->wrot.subfrms[t].clip_ofst = 0;
				/* WROT main_buf: blk_width=tw, line_num=sh (both <=254 -> fit
				 * the 8-bit fields; a full >255 frame would overrun and hang,
				 * which is exactly why we tile). */
				p->wrot.subfrms[t].main_buf = (tw << 16) | (sh << 8);
				break;
			/* RSZ0 (bypassed) / CCORR0 / PATH0_SOUT: geometry region only. */
			}
		}
	}

	/* ctrls[t]: the mmsys mux re-applied per subframe (same RDMA0->CCORR0->
	 * RSZ0->PATH0_SOUT->WROT0 routing as the detile). */
	for (t = 0; t < tile_count; t++) {
		mc = &cfg->ctrls[t];
		mc->num_sets = ARRAY_SIZE(mdp_dt_mux_8183);
		for (i = 0; i < ARRAY_SIZE(mdp_dt_mux_8183); i++) {
			mc->sets[i].reg = mdp_dt_mux_8183[i].reg;
			mc->sets[i].value = mdp_dt_mux_8183[i].value;
			mc->sets[i].subsys_id = MDP_MT8183_MMSYS_SUBSYS_ID;
		}
	}

	if (mdp_kd_debug_on()) {
		struct device *dev = &mdp->pdev->dev;
		const struct mdp_rdma_data_8183 *rd = &cfg->components[0].rdma;
		const struct mdp_wrot_data_8183 *wr = &cfg->components[4].wrot;

		dev_info(dev, "MDPKD-8183-CSC cfg: w=%u h=%u istride=%u ostride=%u nh=%u base=%u nsub=%u\n",
			 w, h, istride, ostride, nh, tile_base, cfg->num_subfrms);
		dev_info(dev, "MDPKD-8183-CSC RDMA(c0 type=%u id=%u): src_ctrl=%#x control=%#x iova=[%#x,%#x] mf_bkgd=%#x sf_bkgd=%#x sub0(off=%#x src=%#x clip=%#x)\n",
			 cfg->components[0].type, cfg->components[0].id,
			 rd->src_ctrl, rd->control, rd->iova[0], rd->iova[1],
			 rd->mf_bkgd, rd->sf_bkgd, rd->subfrms[0].offset[0],
			 rd->subfrms[0].src, rd->subfrms[0].clip);
		dev_info(dev, "MDPKD-8183-CSC WROT(c4 type=%u id=%u): iova=[%#x] control=%#x stride=%#x mat_ctrl=%#x sub0(off=%#x src=%#x main_buf=%#x) byp(rsz)=%u\n",
			 cfg->components[4].type, cfg->components[4].id,
			 wr->iova[0], wr->control, wr->stride[0], wr->mat_ctrl,
			 wr->subfrms[0].offset[0], wr->subfrms[0].src,
			 wr->subfrms[0].main_buf, cfg->components[2].frame.bypass);
	}
}

/* Kernel-direct NV12->NV12 SCALE (resize) for mt8183, multi-tile DOWNSCALE.
 * Combines the mt8183 5-component CSC/detile tiler (2D <=254x254 OUTPUT tiles, the
 * WROT 8-bit width/height cap, the shared multi-job endpoint) with an ACTIVE RSZ0
 * (coeff_step + control1 from the mt8195 scale, same RSZ IP family) and the
 * oracle-derived "tpipe" per-tile PHASE (audit/scale-tpipe-oracle-mt8195.md):
 *   phase_q20 = (out_left << 20) - in_left * coeff_step,  stored in the GENERIC
 * subframe luma/chroma offset (mt8183's mdp_rsz_subfrm has no phase field; the
 * SCP/HW reads it from img_comp_subfrm.luma like mt8195). Each OUTPUT tile maps
 * back to an INPUT region (out*ratio +/- tap); RDMA reads it, RSZ resamples it to
 * the tile + phase, WROT writes the tile. Downscale only for now (the up-coeff is
 * a different shift-15 mode). Oracle-less (mt8183 SCP -22) -> validate by HW
 * PSNR vs ffmpeg, like the CSC. */
/* mt8183 scale OUTPUT-tile geometry (downscale). Each output tile is <=254 px
 * (WROT 8-bit cap) AND its mapped INPUT region (incl. the 2xTAP overlap) stays
 * within the ~512-px RSZ line buffer. A dimension whose WHOLE input already
 * fits the line buffer is split only by the WROT cap -- crucially NOT by the
 * line-buffer formula, which would force a needless extra tile with a tiny
 * remainder (e.g. 240 -> 238+2). A 2-row RSZ-ACTIVE tile cannot flush the
 * 6-tap vertical filter and STALLS the GCE. Tiles are split EVENLY per
 * dimension so there is never a degenerate tiny remainder. Both call sites
 * (dispatch ntiles + the config compute) MUST use this one helper. */
static void mdp_scale_tiles_8183(u32 iw, u32 ih, u32 ow, u32 oh,
				 u32 *nh, u32 *nv, u32 *tilew, u32 *tileh)
{
	const u32 LBUF = 512, WCAP = 254, TAP = 8;	/* LBUF=256 probe ruled out line-buffer width as the flat-vertical cause */
	/* HORIZONTAL: output tile WIDTH is capped at the WROT 254 limit (real -- a
	 * 320-wide output tile stalls, HW-confirmed 2026-06-20) AND at the RSZ line
	 * buffer (mapped input out*ratio + 2*TAP <= 512). */
	u32 mw = (iw <= LBUF) ? min_t(u32, WCAP, ow)
		: min_t(u32, WCAP, (u32)((u64)(LBUF - 2 * TAP) * (ow - 1) / (iw - 1)));
	/* VERTICAL: the output is STREAMED to DRAM row by row -- height is NOT
	 * line-buffer-limited, so do NOT tile it at the WROT WIDTH cap (254).
	 * Vertical multi-tiling a shallow-ratio (coeff>0.5) scale stalls the RSZ
	 * (line-buffer fill across the bottom tile's top edge -- HW-confirmed
	 * 2026-06-20: nh1/nv2 1.33x stalls, nh2/nv1 1.42x works); a single
	 * full-height vertical strip avoids it. Cap only for register sanity. */
	u32 mh = min_t(u32, 2046u, oh);
	u32 ch, cv, tw, th;

	/* Cap the vertical tile so its INPUT span (~mh*ih/oh) stays within the RSZ
	 * vertical-emit capacity (mdp_kd_vcap). cap_th = vcap*oh/ih is the output height
	 * whose input span == vcap; forces nv>1 for a tall transposed input. */
	if (mdp_kd_vcap > 0 && ih > 1) {
		u32 cap_th = (u32)((u64)mdp_kd_vcap * oh / ih);

		if (cap_th >= 2 && cap_th < mh)
			mh = cap_th;
	}
	if (mw < 2) mw = 2;
	if (mh < 2) mh = 2;
	ch = DIV_ROUND_UP(ow, mw);		/* horizontal (column) tile count */
	cv = DIV_ROUND_UP(oh, mh);		/* vertical (row) tile count */
	tw = (DIV_ROUND_UP(ow, ch) + 1) & ~1u;	/* even output tile width */
	th = (DIV_ROUND_UP(oh, cv) + 1) & ~1u;	/* even output tile height */
	if (tw < 2) tw = 2;
	if (th < 2) th = 2;
	*nh = ch; *nv = cv; *tilew = tw; *tileh = th;
}

/*
 * Kernel-direct NV12->NV12 transform (rotate 90/180/270 + hflip/vflip) for mt8183.
 * Full-frame nsub=1, gated iw<=254 && ih<=254 (the mt8183 WROT 8-bit line-buffer
 * cap in BOTH dims; larger needs the rotated tiler -- deferred). Same 5-component
 * 1:1 pixel pipe as the CSC/detile (RDMA0 linear NV12 -> CCORR0 -> RSZ0 bypass ->
 * PATH0_SOUT -> WROT0); the pixel pipe runs the INPUT w x h and ONLY the WROT does
 * the transform, via (control, output-start offset). The control words are the
 * mt8195 SCP-oracle VIDO_CTRL values, reusable verbatim (shared WROT IP, NV12 fmt
 * id 0x0c): 0x4010500c=rot90 0x5020500c=rot180 0x1030500c=rot270 0x1100500c=hflip
 * 0x4120500c=vflip. The offset has 3 shapes (ostride read from the out buffer,
 * = h for the dim-swapping rotate-90/270, = w otherwise):
 *   ROW_REV  (rot90, hflip):  ofst = ostride - 1
 *   FULL_REV (rot180):        ofst_y = w*h - 1,      ofst_uv = w*h/2 - 1
 *   COL_REV  (rot270, vflip): ofst_y = w*h - ostride, ofst_uv = w*h/2 - ostride
 * VIDO_IN_SIZE/TAR_SIZE = input dims (h<<16)|w; main_buf blk_width=w line_num=64
 * (the WROT FIFO depth) -- all blit's verbatim, constant across transforms.
 */
static void mdp_config_compute_8183_xform(struct mdp_dev *mdp,
					  struct img_ipi_frameparam *param,
					  struct img_config *config,
					  u32 control, enum mdp_wrot_ofst mode,
					  bool grey)
{
	struct img_config_8183 *cfg = &config->config_8183;
	const struct img_image_buffer *in = &param->inputs[0].buffer;
	const struct img_image_buffer *out = &param->outputs[0].buffer;
	u32 w = in->format.width;	/* pixel-pipe dims = INPUT (pre-transform) */
	u32 h = in->format.height;
	u32 istride = in->format.plane_fmt[0].stride;	/* linear NV12 Y pitch */
	u32 ostride = out->format.plane_fmt[0].stride;	/* rotated out row stride */
	u32 iy = in->iova[0], ic = in->iova[1];
	u32 oy = out->iova[0], oc = out->iova[1];
	u32 ysz = w * h, csz = ysz / 2;
	u32 ofst_y, ofst_uv;
	struct img_mmsys_ctrl *mc;
	int i;

	/* The mt8183 WROT line buffer is ~32 lines tall, so a full-height nsub=1
	 * write bands every ~strip rows (the 5% corruption). NO-DIM-SWAP transforms
	 * (rot180/hflip/vflip; out dims == in dims) tile into <=strip-tall full-width
	 * vertical strips, each an independent WROT write -- the per-strip start
	 * offset is the output position of the strip's first input pixel input(y0,0),
	 * which reduces to the nsub=1 ofst at y0=0. DIM-SWAP (rot90/270) tiling needs
	 * rotated-output strips (harder) -- still nsub=1 here (banded), a follow-up.
	 * Detect dim-swap from the rotation bits of the control (rot field odd =>
	 * rot90/270), NOT the dims: a SQUARE rot90/270 has equal in/out dims and would
	 * be mis-detected as no-swap (the square-input garbage, 2026-06-21). */
	bool swap = (control >> 20) & 1;
	u32 strip = mdp_kd_wrot_lines > 0 ? (u32)mdp_kd_wrot_lines : 32;
	u32 nsub = swap ? 1 : DIV_ROUND_UP(h, strip);
	u32 t;

	if (nsub > IMG_MAX_SUBFRAMES_8183)
		nsub = IMG_MAX_SUBFRAMES_8183;
	/* full-frame offset (the swap nsub=1 path uses this) */
	switch (mode) {
	case MDP_OFST_ROW_REV:  ofst_y = ostride - 1; ofst_uv = ostride - 1; break;
	case MDP_OFST_FULL_REV: ofst_y = ysz - 1;     ofst_uv = csz - 1;     break;
	default: /* MDP_OFST_COL_REV */
				ofst_y = ysz - ostride; ofst_uv = csz - ostride; break;
	}

	memset(cfg, 0, sizeof(*cfg));
	cfg->num_components = 5;
	cfg->num_subfrms = nsub;

	for (i = 0; i < 5; i++) {
		struct img_compparam_8183 *p = &cfg->components[i];
		struct img_comp_frame_8183 *f = &p->frame;

		p->type = mdp_dt_comp_8183[i].type;
		p->id = mdp_dt_comp_8183[i].id;
		p->input = 0;
		p->num_outputs = 1;
		p->outputs[0] = 0;
		p->num_subfrms = nsub;

		/* pixel pipe = INPUT w x h; rotation is in the WROT write only */
		f->in_width = w; f->in_height = h;
		f->out_width = w; f->out_height = h;
		f->in_total_width = w; f->out_total_width = w;
		f->crop.width = w; f->crop.height = h;

		if (i == 0) {		/* RDMA0 frame-level: linear NV12 read */
			p->rdma.src_ctrl = 0x060c;
			p->rdma.control = 0x10;
			p->rdma.iova[0] = iy;
			p->rdma.iova[1] = ic;
			p->rdma.iova_end[0] = iy + istride * h;
			p->rdma.iova_end[1] = ic + istride * h / 2;
			p->rdma.mf_bkgd = istride;
			p->rdma.sf_bkgd = istride;
		} else if (i == 2) {	/* RSZ0: bypass 1:1 */
			f->bypass = 1;
		} else if (i == 4) {	/* WROT0 frame-level */
			p->wrot.iova[0] = oy;
			p->wrot.iova[1] = grey ? 0 : oc;	/* GREY = 1 plane (no UV) */
			p->wrot.iova[2] = 0;
			p->wrot.control = control;
			p->wrot.stride[0] = ostride;
			p->wrot.stride[1] = grey ? 0 : ostride;
			p->wrot.stride[2] = 0;
			p->wrot.mat_ctrl = 0;	/* no CSC matrix for NV12->NV12 */
		}

		for (t = 0; t < nsub; t++) {
			struct img_comp_subfrm_8183 *s = &p->subfrms[t];
			u32 y0 = swap ? 0 : t * strip;
			u32 sh = swap ? h : min_t(u32, strip, h - y0);
			u32 wy, wuv;	/* WROT per-strip Y / UV start offset */

			s->in.left = 0; s->in.right = w - 1;
			s->in.top = 0;  s->in.bottom = sh - 1;
			s->out.left = 0; s->out.right = w - 1;
			s->out.top = 0;  s->out.bottom = sh - 1;
			s->out_horizontal = 0;
			s->out_vertical = t;

			if (swap) {		/* nsub=1 full-frame (rot90/270, banded TODO) */
				wy = ofst_y; wuv = ofst_uv;
			} else switch (mode) {	/* output pos of input(y0,0) per strip */
			case MDP_OFST_ROW_REV:	/* hflip: same row, col reversed */
				wy  = y0 * ostride + (w - 1);
				wuv = (y0 / 2) * ostride + (w - 1); break;
			case MDP_OFST_FULL_REV:	/* rot180: row h-1-y0, col reversed */
				wy  = (h - 1 - y0) * ostride + (w - 1);
				wuv = (h / 2 - 1 - y0 / 2) * ostride + (w - 1); break;
			default:		/* COL_REV vflip: row h-1-y0, col forward */
				wy  = (h - 1 - y0) * ostride;
				wuv = (h / 2 - 1 - y0 / 2) * ostride; break;
			}

			if (i == 0) {		/* RDMA0 per-strip: linear source rows */
				p->rdma.subfrms[t].offset[0] = y0 * istride;
				p->rdma.subfrms[t].offset[1] = (y0 / 2) * istride;
				p->rdma.subfrms[t].offset[2] = (y0 / 2) * istride;
				p->rdma.subfrms[t].src  = (sh << 16) | w;
				p->rdma.subfrms[t].clip = (sh << 16) | w;
				p->rdma.subfrms[t].clip_ofst = 0;
			} else if (i == 4) {	/* WROT0 per-strip */
				p->wrot.subfrms[t].offset[0] = wy;
				p->wrot.subfrms[t].offset[1] = grey ? 0 : wuv;
				p->wrot.subfrms[t].offset[2] = grey ? 0 : wuv;
				p->wrot.subfrms[t].src  = (sh << 16) | w;	/* VIDO_IN_SIZE */
				p->wrot.subfrms[t].clip = (sh << 16) | w;	/* VIDO_TAR_SIZE */
				/* main_buf line_num is a 7-bit field; for the swap path (nsub=1,
				 * sh=h) h>127 overflows it (256 wrapped to 0 -> garbage). Cap to
				 * the rotate line-buffer depth (32); the WROT streams the full
				 * height through it. Harmless for the tiled paths (sh<=32). */
				p->wrot.subfrms[t].main_buf =
					(w << 16) | (min_t(u32, sh, 32) << 8);
			}
		}
	}

	/* mmsys mux per subframe */
	for (t = 0; t < nsub; t++) {
		mc = &cfg->ctrls[t];
		mc->num_sets = ARRAY_SIZE(mdp_dt_mux_8183);
		for (i = 0; i < (int)ARRAY_SIZE(mdp_dt_mux_8183); i++) {
			mc->sets[i].reg = mdp_dt_mux_8183[i].reg;
			mc->sets[i].value = mdp_dt_mux_8183[i].value;
			mc->sets[i].subsys_id = MDP_MT8183_MMSYS_SUBSYS_ID;
		}
	}

	if (mdp_kd_debug_on()) {
		struct device *dev = &mdp->pdev->dev;
		const struct mdp_wrot_data_8183 *wr = &cfg->components[4].wrot;

		dev_info(dev, "MDPKD-8183-XFORM: in=%ux%u ostride=%u control=%#x mode=%d swap=%d nsub=%u strip=%u wy0=%#x main0=%#x\n",
			 w, h, ostride, control, mode, swap, nsub, strip,
			 wr->subfrms[0].offset[0], wr->subfrms[0].main_buf);
	}
}

/*
 * UV pass of the separate-plane NV12 rotate (no-swap modes rot180/hflip/vflip):
 * the interleaved-CbCr plane is a (w/2 x h/2) image of 16-bit CbCr-pair pixels;
 * rotate it OPAQUE-16bpp (YUYV id 5, no CSC -> Cb/Cr pairs stay intact) from the
 * input UV plane (in.iova[1]) into the out NV12 UV plane (out.iova[1]). The WROT
 * byte offset is the Y formula with w->wc=w/2, h->hc=h/2 and the COLUMN term x2.
 * `control` = the GREY control with fmt 0x07->0x05 (YUYV id). Single write plane.
 */
static void mdp_config_compute_8183_uv16(struct mdp_dev *mdp,
					 struct img_ipi_frameparam *param,
					 struct img_config *config,
					 u32 control, enum mdp_wrot_ofst mode)
{
	struct img_config_8183 *cfg = &config->config_8183;
	const struct img_image_buffer *in = &param->inputs[0].buffer;
	const struct img_image_buffer *out = &param->outputs[0].buffer;
	u32 wc = in->format.width / 2;		/* UV image width (16-bit pairs) */
	u32 hc = in->format.height / 2;		/* UV image height */
	u32 istride = in->format.plane_fmt[0].stride;	/* UV row pitch = Y pitch */
	u32 ostride = out->format.plane_fmt[0].stride;
	u32 ic = in->iova[1];			/* input UV plane base */
	u32 oc = out->iova[1];			/* output UV plane base */
	u32 strip = mdp_kd_wrot_lines > 0 ? (u32)mdp_kd_wrot_lines : 32;
	u32 nsub = DIV_ROUND_UP(hc, strip);
	struct img_mmsys_ctrl *mc;
	int i;
	u32 t;

	if (nsub > IMG_MAX_SUBFRAMES_8183)
		nsub = IMG_MAX_SUBFRAMES_8183;
	memset(cfg, 0, sizeof(*cfg));
	cfg->num_components = 5;
	cfg->num_subfrms = nsub;

	for (i = 0; i < 5; i++) {
		struct img_compparam_8183 *p = &cfg->components[i];
		struct img_comp_frame_8183 *f = &p->frame;

		p->type = mdp_dt_comp_8183[i].type;
		p->id = mdp_dt_comp_8183[i].id;
		p->input = 0;
		p->num_outputs = 1;
		p->outputs[0] = 0;
		p->num_subfrms = nsub;
		f->in_width = wc; f->in_height = hc;
		f->out_width = wc; f->out_height = hc;
		f->in_total_width = wc; f->out_total_width = wc;
		f->crop.width = wc; f->crop.height = hc;

		if (i == 0) {		/* RDMA0: read input UV as opaque 16bpp (id 5) */
			p->rdma.src_ctrl = 0x5;
			p->rdma.control = 0x10;
			p->rdma.iova[0] = ic;
			p->rdma.iova[1] = 0;
			p->rdma.iova_end[0] = ic + istride * hc;
			p->rdma.mf_bkgd = istride;
			p->rdma.sf_bkgd = istride;
		} else if (i == 2) {
			f->bypass = 1;
		} else if (i == 4) {	/* WROT0: write out UV as opaque 16bpp */
			p->wrot.iova[0] = oc;
			p->wrot.iova[1] = 0;
			p->wrot.iova[2] = 0;
			p->wrot.control = control;
			p->wrot.stride[0] = ostride;
			p->wrot.stride[1] = 0;
			p->wrot.stride[2] = 0;
			p->wrot.mat_ctrl = 0;
		}

		for (t = 0; t < nsub; t++) {
			struct img_comp_subfrm_8183 *s = &p->subfrms[t];
			u32 y0 = t * strip;
			u32 sh = min_t(u32, strip, hc - y0);
			u32 wuv;	/* WROT byte offset (16bpp: column term x2) */

			s->in.left = 0; s->in.right = wc - 1;
			s->in.top = 0;  s->in.bottom = sh - 1;
			s->out.left = 0; s->out.right = wc - 1;
			s->out.top = 0;  s->out.bottom = sh - 1;
			s->out_horizontal = 0;
			s->out_vertical = t;

			switch (mode) {
			case MDP_OFST_ROW_REV:	/* hflip */
				wuv = y0 * ostride + (wc - 1) * 2; break;
			case MDP_OFST_FULL_REV:	/* rot180 */
				wuv = (hc - 1 - y0) * ostride + (wc - 1) * 2; break;
			default:		/* COL_REV vflip */
				wuv = (hc - 1 - y0) * ostride; break;
			}

			if (i == 0) {
				p->rdma.subfrms[t].offset[0] = y0 * istride;
				p->rdma.subfrms[t].src  = (sh << 16) | wc;
				p->rdma.subfrms[t].clip = (sh << 16) | wc;
				p->rdma.subfrms[t].clip_ofst = 0;
			} else if (i == 4) {
				p->wrot.subfrms[t].offset[0] = wuv;
				p->wrot.subfrms[t].src  = (sh << 16) | wc;	/* VIDO_IN_SIZE */
				p->wrot.subfrms[t].clip = (sh << 16) | wc;	/* VIDO_TAR_SIZE */
				p->wrot.subfrms[t].main_buf = (wc << 16) | (sh << 8);
			}
		}
	}

	for (t = 0; t < nsub; t++) {
		mc = &cfg->ctrls[t];
		mc->num_sets = ARRAY_SIZE(mdp_dt_mux_8183);
		for (i = 0; i < (int)ARRAY_SIZE(mdp_dt_mux_8183); i++) {
			mc->sets[i].reg = mdp_dt_mux_8183[i].reg;
			mc->sets[i].value = mdp_dt_mux_8183[i].value;
			mc->sets[i].subsys_id = MDP_MT8183_MMSYS_SUBSYS_ID;
		}
	}

	if (mdp_kd_debug_on())
		dev_info(&mdp->pdev->dev,
			 "MDPKD-8183-UV16: uv=%ux%u istride=%u ostride=%u control=%#x mode=%d nsub=%u\n",
			 wc, hc, istride, ostride, control, mode, nsub);
}

/*
 * NV12 -> I420 ROTATION (de-interleave + rotate in ONE WROT pass): the WROT writes
 * 3 SEPARATE planes (Y, U=Cb, V=Cr), so the chroma is de-interleaved into its own
 * single-stride planes AND rotated. This is the keystone test of the de-interleave
 * approach -- whether a 3-plane (de-interleaved) write avoids the 2-plane Y<->UV
 * demux swap. If bit-exact, the chroma rotates as separate planes here; add an
 * I420->NV12 re-interleave for NV12 out. control = I420 fmt 0x08 | rot/flip; chroma
 * plane = wc=w/2 x hc=h/2, chroma pitch osc=ostride/2. No-swap modes, tiled.
 */
static void mdp_config_compute_8183_i420rot(struct mdp_dev *mdp,
					    struct img_ipi_frameparam *param,
					    struct img_config *config,
					    u32 control, enum mdp_wrot_ofst mode)
{
	struct img_config_8183 *cfg = &config->config_8183;
	const struct img_image_buffer *in = &param->inputs[0].buffer;
	const struct img_image_buffer *out = &param->outputs[0].buffer;
	u32 w = in->format.width, h = in->format.height;
	u32 wc = w / 2, hc = h / 2;
	u32 istride = in->format.plane_fmt[0].stride;
	u32 ostride = out->format.plane_fmt[0].stride;	/* I420 Y pitch */
	u32 osc = ostride / 2;				/* I420 U/V pitch */
	u32 iy = in->iova[0], ic = in->iova[1];
	u32 oy = out->iova[0], ou = out->iova[1], ov = out->iova[2];
	u32 strip = mdp_kd_wrot_lines > 0 ? (u32)mdp_kd_wrot_lines : 32;
	u32 nsub = DIV_ROUND_UP(h, strip);
	struct img_mmsys_ctrl *mc;
	int i;
	u32 t;

	if (nsub > IMG_MAX_SUBFRAMES_8183)
		nsub = IMG_MAX_SUBFRAMES_8183;
	memset(cfg, 0, sizeof(*cfg));
	cfg->num_components = 5;
	cfg->num_subfrms = nsub;

	for (i = 0; i < 5; i++) {
		struct img_compparam_8183 *p = &cfg->components[i];
		struct img_comp_frame_8183 *f = &p->frame;

		p->type = mdp_dt_comp_8183[i].type;
		p->id = mdp_dt_comp_8183[i].id;
		p->input = 0;
		p->num_outputs = 1;
		p->outputs[0] = 0;
		p->num_subfrms = nsub;
		f->in_width = w; f->in_height = h;
		f->out_width = w; f->out_height = h;
		f->in_total_width = w; f->out_total_width = w;
		f->crop.width = w; f->crop.height = h;

		if (i == 0) {		/* RDMA0: linear NV12 read */
			p->rdma.src_ctrl = 0x060c;
			p->rdma.control = 0x10;
			p->rdma.iova[0] = iy;
			p->rdma.iova[1] = ic;
			p->rdma.iova_end[0] = iy + istride * h;
			p->rdma.iova_end[1] = ic + istride * h / 2;
			p->rdma.mf_bkgd = istride;
			p->rdma.sf_bkgd = istride;
		} else if (i == 2) {
			f->bypass = 1;
		} else if (i == 4) {	/* WROT0: I420 3-plane write */
			p->wrot.iova[0] = oy;
			p->wrot.iova[1] = ou;
			p->wrot.iova[2] = ov;
			p->wrot.control = control;
			p->wrot.stride[0] = ostride;
			p->wrot.stride[1] = osc;
			p->wrot.stride[2] = osc;
			p->wrot.mat_ctrl = 0;
		}

		for (t = 0; t < nsub; t++) {
			struct img_comp_subfrm_8183 *s = &p->subfrms[t];
			u32 y0 = t * strip;
			u32 sh = min_t(u32, strip, h - y0);
			u32 y0c = y0 / 2;
			u32 wy, wuv;

			s->in.left = 0; s->in.right = w - 1;
			s->in.top = 0;  s->in.bottom = sh - 1;
			s->out.left = 0; s->out.right = w - 1;
			s->out.top = 0;  s->out.bottom = sh - 1;
			s->out_horizontal = 0;
			s->out_vertical = t;

			switch (mode) {
			case MDP_OFST_ROW_REV:	/* hflip */
				wy = y0 * ostride + (w - 1);
				wuv = y0c * osc + (wc - 1); break;
			case MDP_OFST_FULL_REV:	/* rot180 */
				wy = (h - 1 - y0) * ostride + (w - 1);
				wuv = (hc - 1 - y0c) * osc + (wc - 1); break;
			default:		/* COL_REV vflip */
				wy = (h - 1 - y0) * ostride;
				wuv = (hc - 1 - y0c) * osc; break;
			}

			if (i == 0) {
				p->rdma.subfrms[t].offset[0] = y0 * istride;
				p->rdma.subfrms[t].offset[1] = y0c * istride;
				p->rdma.subfrms[t].offset[2] = y0c * istride;
				p->rdma.subfrms[t].src  = (sh << 16) | w;
				p->rdma.subfrms[t].clip = (sh << 16) | w;
				p->rdma.subfrms[t].clip_ofst = 0;
			} else if (i == 4) {
				p->wrot.subfrms[t].offset[0] = wy;
				p->wrot.subfrms[t].offset[1] = wuv;	/* U plane */
				p->wrot.subfrms[t].offset[2] = wuv;	/* V plane */
				p->wrot.subfrms[t].src  = (sh << 16) | w;
				p->wrot.subfrms[t].clip = (sh << 16) | w;
				p->wrot.subfrms[t].main_buf = (w << 16) | (sh << 8);
			}
		}
	}

	for (t = 0; t < nsub; t++) {
		mc = &cfg->ctrls[t];
		mc->num_sets = ARRAY_SIZE(mdp_dt_mux_8183);
		for (i = 0; i < (int)ARRAY_SIZE(mdp_dt_mux_8183); i++) {
			mc->sets[i].reg = mdp_dt_mux_8183[i].reg;
			mc->sets[i].value = mdp_dt_mux_8183[i].value;
			mc->sets[i].subsys_id = MDP_MT8183_MMSYS_SUBSYS_ID;
		}
	}

	if (mdp_kd_debug_on())
		dev_info(&mdp->pdev->dev,
			 "MDPKD-8183-I420ROT: in=%ux%u ostride=%u osc=%u control=%#x mode=%d nsub=%u\n",
			 w, h, ostride, osc, control, mode, nsub);
}

/*
 * 5-pass scratch-backed NV12->NV12 rotate, building blocks (no-swap modes).
 * The WROT corrupts ANY multi-plane write under rotation, so the chroma is
 * rotated as SEPARATE single-plane GREY ops: de-interleave (non-rotated) -> rot Y
 * + rot Cb + rot Cr (each single-plane) -> re-interleave (non-rotated). Three
 * helpers below; wired as detile_nchunks=5 in mdp_vpu_compute_8183_chunk.
 */

/* Single-plane GREY rotate of a pw x ph 8bpp plane: src (siy[+sic for NV12 src],
 * src_ctrl, sstride) -> dst GREY (doy, control, dstride), tiled, no-swap offsets. */
static void mdp_config_compute_8183_planerot(struct mdp_dev *mdp,
		struct img_config *config,
		u32 siy, u32 sic, u32 src_ctrl, u32 sstride,
		u32 doy, u32 control, u32 dstride,
		u32 pw, u32 ph, enum mdp_wrot_ofst mode)
{
	struct img_config_8183 *cfg = &config->config_8183;
	u32 strip = mdp_kd_wrot_lines > 0 ? (u32)mdp_kd_wrot_lines : 32;
	u32 nsub = DIV_ROUND_UP(ph, strip);
	struct img_mmsys_ctrl *mc;
	int i;
	u32 t;

	if (nsub > IMG_MAX_SUBFRAMES_8183)
		nsub = IMG_MAX_SUBFRAMES_8183;
	memset(cfg, 0, sizeof(*cfg));
	cfg->num_components = 5;
	cfg->num_subfrms = nsub;

	for (i = 0; i < 5; i++) {
		struct img_compparam_8183 *p = &cfg->components[i];
		struct img_comp_frame_8183 *f = &p->frame;

		p->type = mdp_dt_comp_8183[i].type;
		p->id = mdp_dt_comp_8183[i].id;
		p->input = 0;
		p->num_outputs = 1;
		p->outputs[0] = 0;
		p->num_subfrms = nsub;
		f->in_width = pw; f->in_height = ph;
		f->out_width = pw; f->out_height = ph;
		f->in_total_width = pw; f->out_total_width = pw;
		f->crop.width = pw; f->crop.height = ph;

		if (i == 0) {
			p->rdma.src_ctrl = src_ctrl;
			p->rdma.control = 0x10;
			p->rdma.iova[0] = siy;
			p->rdma.iova[1] = sic;
			p->rdma.iova_end[0] = siy + sstride * ph;
			if (sic)
				p->rdma.iova_end[1] = sic + sstride * ph / 2;
			p->rdma.mf_bkgd = sstride;
			p->rdma.sf_bkgd = sstride;
		} else if (i == 2) {
			f->bypass = 1;
		} else if (i == 4) {
			p->wrot.iova[0] = doy;
			p->wrot.iova[1] = 0;
			p->wrot.iova[2] = 0;
			p->wrot.control = control;
			p->wrot.stride[0] = dstride;
			p->wrot.stride[1] = 0;
			p->wrot.stride[2] = 0;
			p->wrot.mat_ctrl = 0;
		}

		for (t = 0; t < nsub; t++) {
			struct img_comp_subfrm_8183 *s = &p->subfrms[t];
			u32 y0 = t * strip;
			u32 sh = min_t(u32, strip, ph - y0);
			u32 wy;

			s->in.left = 0; s->in.right = pw - 1;
			s->in.top = 0;  s->in.bottom = sh - 1;
			s->out.left = 0; s->out.right = pw - 1;
			s->out.top = 0;  s->out.bottom = sh - 1;
			s->out_horizontal = 0;
			s->out_vertical = t;

			switch (mode) {
			case MDP_OFST_ROW_REV:  wy = y0 * dstride + (pw - 1); break;
			case MDP_OFST_FULL_REV: wy = (ph - 1 - y0) * dstride + (pw - 1); break;
			default:                wy = (ph - 1 - y0) * dstride; break;
			}

			if (i == 0) {
				p->rdma.subfrms[t].offset[0] = y0 * sstride;
				if (sic)
					p->rdma.subfrms[t].offset[1] = (y0 / 2) * sstride;
				p->rdma.subfrms[t].src  = (sh << 16) | pw;
				p->rdma.subfrms[t].clip = (sh << 16) | pw;
				p->rdma.subfrms[t].clip_ofst = 0;
			} else if (i == 4) {
				p->wrot.subfrms[t].offset[0] = wy;
				p->wrot.subfrms[t].src  = (sh << 16) | pw;
				p->wrot.subfrms[t].clip = (sh << 16) | pw;
				p->wrot.subfrms[t].main_buf = (pw << 16) | (sh << 8);
			}
		}
	}

	for (t = 0; t < nsub; t++) {
		mc = &cfg->ctrls[t];
		mc->num_sets = ARRAY_SIZE(mdp_dt_mux_8183);
		for (i = 0; i < (int)ARRAY_SIZE(mdp_dt_mux_8183); i++) {
			mc->sets[i].reg = mdp_dt_mux_8183[i].reg;
			mc->sets[i].value = mdp_dt_mux_8183[i].value;
			mc->sets[i].subsys_id = MDP_MT8183_MMSYS_SUBSYS_ID;
		}
	}
}

/* Forward (non-rotated) convert: NV12 in (0x060c) -> I420 out (0x5008), or
 * I420 in (0x0608) -> NV12 out (0x500c). Reads riy/ric/riv @ ristride, writes
 * woy/wou/wov @ wstride/wcstride. wnplanes = 3 (I420 out) or 2 (NV12 out, wov=0).
 * Chroma is 4:2:0 both sides; offsets are forward top-to-bottom. */
static void mdp_config_compute_8183_cvt(struct mdp_dev *mdp,
		struct img_config *config,
		u32 riy, u32 ric, u32 riv, u32 src_ctrl, u32 ristride, u32 ricstride,
		u32 woy, u32 wou, u32 wov, u32 wctrl, u32 wstride, u32 wcstride,
		u32 w, u32 h)
{
	struct img_config_8183 *cfg = &config->config_8183;
	/* Forward convert needs no tiling (tiling was a rotation/line-buffer constraint):
	 * one whole-frame subframe, WROT line_num = the blit FIFO depth (64). */
	u32 strip = mdp_kd_wrot_lines > 0 ? (u32)mdp_kd_wrot_lines : h;
	u32 nsub = DIV_ROUND_UP(h, strip);
	struct img_mmsys_ctrl *mc;
	int i;
	u32 t;

	if (nsub > IMG_MAX_SUBFRAMES_8183)
		nsub = IMG_MAX_SUBFRAMES_8183;
	memset(cfg, 0, sizeof(*cfg));
	cfg->num_components = 5;
	cfg->num_subfrms = nsub;

	for (i = 0; i < 5; i++) {
		struct img_compparam_8183 *p = &cfg->components[i];
		struct img_comp_frame_8183 *f = &p->frame;

		p->type = mdp_dt_comp_8183[i].type;
		p->id = mdp_dt_comp_8183[i].id;
		p->input = 0;
		p->num_outputs = 1;
		p->outputs[0] = 0;
		p->num_subfrms = nsub;
		f->in_width = w; f->in_height = h;
		f->out_width = w; f->out_height = h;
		f->in_total_width = w; f->out_total_width = w;
		f->crop.width = w; f->crop.height = h;

		if (i == 0) {
			p->rdma.src_ctrl = src_ctrl;
			p->rdma.control = 0x10;
			p->rdma.iova[0] = riy;
			p->rdma.iova[1] = ric;
			p->rdma.iova[2] = riv;
			p->rdma.iova_end[0] = riy + ristride * h;
			p->rdma.iova_end[1] = ric + ricstride * h / 2;
			if (riv)
				p->rdma.iova_end[2] = riv + ricstride * h / 2;
			p->rdma.mf_bkgd = ristride;
			p->rdma.sf_bkgd = ristride;
		} else if (i == 2) {
			f->bypass = 1;
		} else if (i == 4) {
			p->wrot.iova[0] = woy;
			p->wrot.iova[1] = wou;
			p->wrot.iova[2] = wov;
			p->wrot.control = wctrl;
			p->wrot.stride[0] = wstride;
			p->wrot.stride[1] = wcstride;
			p->wrot.stride[2] = wov ? wcstride : 0;
			p->wrot.mat_ctrl = 0;
		}

		for (t = 0; t < nsub; t++) {
			struct img_comp_subfrm_8183 *s = &p->subfrms[t];
			u32 y0 = t * strip;
			u32 sh = min_t(u32, strip, h - y0);

			s->in.left = 0; s->in.right = w - 1;
			s->in.top = 0;  s->in.bottom = sh - 1;
			s->out.left = 0; s->out.right = w - 1;
			s->out.top = 0;  s->out.bottom = sh - 1;
			s->out_horizontal = 0;
			s->out_vertical = t;

			if (i == 0) {
				p->rdma.subfrms[t].offset[0] = y0 * ristride;
				p->rdma.subfrms[t].offset[1] = (y0 / 2) * ricstride;
				if (riv)
					p->rdma.subfrms[t].offset[2] = (y0 / 2) * ricstride;
				p->rdma.subfrms[t].src  = (sh << 16) | w;
				p->rdma.subfrms[t].clip = (sh << 16) | w;
				p->rdma.subfrms[t].clip_ofst = 0;
			} else if (i == 4) {
				p->wrot.subfrms[t].offset[0] = y0 * wstride;
				p->wrot.subfrms[t].offset[1] = (y0 / 2) * wcstride;
				if (wov)
					p->wrot.subfrms[t].offset[2] = (y0 / 2) * wcstride;
				p->wrot.subfrms[t].src  = (sh << 16) | w;
				p->wrot.subfrms[t].clip = (sh << 16) | w;
				/* WROT FIFO line_num = blit depth (64); the engine streams the
				 * full strip height through it (sh may be the whole frame). */
				p->wrot.subfrms[t].main_buf =
					(w << 16) | (min_t(u32, sh, 64) << 8);
			}
		}
	}

	for (t = 0; t < nsub; t++) {
		mc = &cfg->ctrls[t];
		mc->num_sets = ARRAY_SIZE(mdp_dt_mux_8183);
		for (i = 0; i < (int)ARRAY_SIZE(mdp_dt_mux_8183); i++) {
			mc->sets[i].reg = mdp_dt_mux_8183[i].reg;
			mc->sets[i].value = mdp_dt_mux_8183[i].value;
			mc->sets[i].subsys_id = MDP_MT8183_MMSYS_SUBSYS_ID;
		}
	}

	if (mdp_kd_debug_on()) {
		struct img_compparam_8183 *p0 = &cfg->components[0];
		struct img_compparam_8183 *p4 = &cfg->components[4];

		dev_info(&mdp->pdev->dev,
			 "MDPKD-CVT: %ux%u src=%#x ristride=%u ricstride=%u RDMAiova=%#x/%#x/%#x | wctrl=%#x WROTiova=%#x/%#x/%#x wstride=%u/%u/%u nsub=%u\n",
			 w, h, p0->rdma.src_ctrl, ristride, ricstride,
			 p0->rdma.iova[0], p0->rdma.iova[1], p0->rdma.iova[2],
			 wctrl, p4->wrot.iova[0], p4->wrot.iova[1], p4->wrot.iova[2],
			 p4->wrot.stride[0], p4->wrot.stride[1], p4->wrot.stride[2], cfg->num_subfrms);
		dev_info(&mdp->pdev->dev,
			 "MDPKD-CVT: s0 RDMAoff=%#x/%#x/%#x WROToff=%#x/%#x/%#x main=%#x | s1 WROToff=%#x/%#x/%#x\n",
			 p0->rdma.subfrms[0].offset[0], p0->rdma.subfrms[0].offset[1], p0->rdma.subfrms[0].offset[2],
			 p4->wrot.subfrms[0].offset[0], p4->wrot.subfrms[0].offset[1], p4->wrot.subfrms[0].offset[2],
			 p4->wrot.subfrms[0].main_buf,
			 p4->wrot.subfrms[1].offset[0], p4->wrot.subfrms[1].offset[1], p4->wrot.subfrms[1].offset[2]);
	}
}

/*
 * 64x32-WROT-tiled NV12 -> multi-plane fmtconv for mt8183 (NV21 / I420 / YV12).
 * The mt8183 WROT output buffer is 64 wide x 32 tall, so a MULTI-PLANE write (Y +
 * chroma sharing the buffer) MUST tile its output <=64x32 -- mdp_config_compute_8183_cvt's
 * full-width single pass corrupts the Y plane mid-frame for >=2-plane outputs (1-plane
 * GREY tolerates it). This models the VALIDATED detile WROT tiling (mdp_config_compute_8183:
 * main_buf=(tw<<16)|(sh<<8), tw<=64, sh<=32) but reads LINEAR NV12 (src_ctrl 0x060c, not
 * MM21 block) and writes the table output format. Per-tile chroma offset: interleaved
 * (NV21) (y0/2)*ocstride+x0; planar (I420/YV12) (y0/2)*ocstride+(x0/2), ocstride=ostride/2.
 * 2D grid g=ty*nh+tx; [tbase,tbase+tcount) is this GCE chunk (>14 tiles -> multi-chunk).
 */
static void mdp_config_compute_8183_fmtconv(struct mdp_dev *mdp,
		struct img_ipi_frameparam *param, struct img_config *config,
		const struct mdp_fmtconv *f, u32 tbase, u32 tcount)
{
	struct img_config_8183 *cfg = &config->config_8183;
	const struct img_image_buffer *in = &param->inputs[0].buffer;
	const struct img_image_buffer *out = &param->outputs[0].buffer;
	u32 w = out->format.width, h = out->format.height;
	u32 istride = in->format.plane_fmt[0].stride;
	u32 ostride = out->format.plane_fmt[0].stride;
	u32 iy = in->iova[0], ic = in->iova[1];
	u32 oy = out->iova[0], oc = out->iova[1], ov = out->iova[2];
	u32 nplanes = MDP_COLOR_GET_PLANE_COUNT(out->format.colorformat);
	bool planar = (nplanes == 3);		/* I420/YV12: planar half-stride chroma */
	u32 ocstride = planar ? ostride / 2 : ostride;
	/* Chroma vertical extent: V_SUBSAMPLE=1 => 4:2:0 (chroma row y0/2, half-height
	 * plane: NV12/NV21/I420/YV12); =0 => 4:2:2 (chroma row y0, full-height plane:
	 * NV16/NV61). The WROT's 4:2:0->4:2:2 vertical upsample (uv_ysel=2) lives in the
	 * table control word (0x8000500d/0x8000510d), same as the packed-422 path; only
	 * the per-tile output chroma row offset differs. */
	u32 cvshift = MDP_COLOR_GET_V_SUBSAMPLE(out->format.colorformat) ? 1 : 0;
	/* output Y/RGB bytes-per-pixel from the buffer pitch (=1 for YUV/GREY,
	 * 2/3/4 for RGB565/BGR888/ARGB) -- the WROT VIDO_OFST x-term is in bytes. */
	u32 bppY = w ? ostride / w : 1;
	/* Read full NV12 (Y+chroma) for every chroma-needing output (4:2:0/4:2:2/RGB);
	 * only GREY (Y-only) uses 0xc. The mt8195 table has 0xc for 4:2:2/RGB, but on
	 * mt8183 0x060c reads the chroma the WROT upsamples (4:2:0->4:2:2) / CSCs
	 * (YUV->RGB, ~46 dB vs 0xc's ~30). For RGB also set the WROT YUV->RGB matrix. */
	/* 4:2:2 SOURCE (NV16/NV61): the chroma plane is FULL-height (V_SUBSAMPLE=0), so
	 * the RDMA read differs from NV12's 4:2:0: sourceFormat = HW_FORMAT(NV16)=0xd
	 * (-> src_ctrl 0x060d, filterMode 3 unchanged), iova_end[1] = ic + istride*h,
	 * per-tile chroma row offset = y0*istride. The source U/V byte order is handled
	 * by the chroma-swap-twin row selection (mdp_fmtconv_for), so the read is always
	 * in NV16 (U,V) order here. The OUTPUT (WROT) side is unchanged. */
	bool src422 = mdp_kd_src_nv16 &&
		      mdp_color_is_plain_nv16src(in->format.colorformat);
	/* 4:4:4 SOURCE (NV24/NV42, 2-plane semi-planar): the chroma plane is FULL-width
	 * AND full-height (V_SUBSAMPLE=0, H_SUBSAMPLE=0), so vs NV12's 4:2:0 read:
	 * sourceFormat = HW_FORMAT(NV24)=0xe (-> src_ctrl 0x060e), chroma plane pitch =
	 * istride*2 (full-width 16-bpp interleaved UV), iova_end[1] = ic + istride*2*h,
	 * per-tile chroma offset = y0*(istride*2) + x0*2. The pipe carries 4:4:4 (like
	 * RGB-after-CSC); the WROT decimates per target. Chroma byte order handled by the
	 * swap-twin (NV42 read as NV24). */
	bool src444 = mdp_kd_src_nv24 &&
		      mdp_color_is_plain_nv24src(in->format.colorformat);
	/* NV24/NV42 chroma plane pitch (full-width 16-bpp) vs the 4:2:0/4:2:2 istride. */
	u32 cstride = src444 ? istride * 2 : istride;
	/* Packed 4:2:2 SOURCE (YUYV/YVYU/UYVY/VYUY): a SINGLE interleaved plane (no
	 * chroma plane), 16 bpp. src_ctrl sourceFormat = the source HW_FORMAT (5 -> 0x605
	 * for YUYV/YVYU, 4 -> 0x604 for UYVY/VYUY), filterMode 3. sourceSwap stays 0: the
	 * U/V byte order is handled by the chroma-swap-twin row selection (mdp_fmtconv_for),
	 * so a swapped source (YVYU/VYUY) is read in its unswapped twin's order here. */
	bool srcpacked = mdp_kd_src_yuyv &&
			 mdp_color_is_packed422src(in->format.colorformat);
	/* RGB SOURCE (RGB24/BGR24/RGB565/RGB565X/ARGB32/ABGR32): a SINGLE packed plane
	 * (like packed-4:2:2) PLUS the RDMA RGB->YUV CSC (rdma.transform). The WROT writes
	 * the plain YUV output (mat_ctrl=0). bppsrc = 2/3/4 from the source bit depth. */
	bool srcrgb = mdp_kd_src_rgb &&
		      mdp_color_is_rgb_src(in->format.colorformat);
	u32 bppsrc = srcrgb ? (MDP_COLOR_BITS_PER_PIXEL(in->format.colorformat) / 8) :
		     srcpacked ? 2 : 1;		/* source plane-0 bytes-per-pixel */
	/* I420/YV12 SOURCE (3-plane planar 4:2:0): THREE separate planes (Y, U, V),
	 * chroma planes half-WIDTH (8 bpp), so the chroma plane pitch = istride/2 and the
	 * read populates iova[2]/iova_end[2]. src_ctrl = 0x608 (sourceFormat 8 = YUV420
	 * 3-plane). The 3-plane read is the same for every admitted output (NV12/NV21/GREY);
	 * the WROT control selects the output format. */
	bool srci420 = mdp_kd_src_i420 &&
		       mdp_color_is_plain_i420src(in->format.colorformat);
	u32 icstride = istride / 2;		/* I420/YV12 chroma PLANE pitch (half width) */
	u32 iu = ic, iv = in->iova[2];		/* I420 U,V plane bases (swapped for YV12) */
	/* MM21/MT21 SOURCE (block-mode 4:2:0, the codec detile): the RDMA reads the
	 * tiled buffer with the EXACT detile geometry (src_ctrl 0x860c, mf_bkgd=aw*32,
	 * sf_bkgd=aw*16, per-tile tile-row offsets, csh=ALIGN(sh,32) source height); the
	 * pipe then carries linear 4:2:0 YUV, and the WROT does the X-conversion exactly
	 * as for an NV12 source. aw = the MM21 read-side aligned width (luma stride =
	 * aw*32); inh = the block-padded input height. This is the single-pass combine of
	 * mdp_config_compute_8183 (the detile read) with this fmtconv WROT write. */
	bool srcmm21 = mdp_kd_mm21conv &&
		       mdp_color_is_mm21src(in->format.colorformat);
	u32 aw = ALIGN(w, 64);			/* MM21 read-side aligned width */
	u32 inh = in->format.height;		/* block-padded input height */
	/* A packed source is ALWAYS read packed (one interleaved plane), even for a GREY
	 * (Y-only) output -- the GREY 0xc read mode is for a contiguous Y plane (NV12/NV16),
	 * which a packed source does NOT have. So srcpacked takes precedence; the WROT GREY
	 * control extracts Y from the packed pipe. An I420/YV12 source is read 3-plane
	 * (0x608) for every output (incl. GREY -- the chroma planes are read within bounds
	 * and discarded by the GREY WROT), so srci420 also overrides the GREY 0xc. */
	/* RGB565X (MDP_COLOR_BGR565) diag: mdp_kd_rgbr_noswap reads it as plain LE RGB565
	 * (strip the R/B sourceSwap bit) to distinguish the byte-swap symptom from R/B. */
	bool rgbr_noswap = srcrgb && mdp_kd_rgbr_noswap &&
			   in->format.colorformat == MDP_COLOR_BGR565;
	u32 src_ctrl = srcmm21 ? 0x860c :	/* block-mode (bit15) MM21 420_BLKP */
		       srcrgb ?
			(0x600 |
			 ((MDP_COLOR_IS_SWAPPED(in->format.colorformat) && !rgbr_noswap) ? 0x4000 : 0) |
			 MDP_COLOR_GET_HW_FORMAT(in->format.colorformat)) :
		       srcpacked ?
			(0x600 | MDP_COLOR_GET_HW_FORMAT(in->format.colorformat)) :
		       srci420 ? 0x608 :
		       (f->color == MDP_COLOR_GREY) ? 0xc :
		       src444 ? 0x060e :
		       (src422 ? 0x060d : 0x060c);
	/* WROT YUV->RGB CSC only for a YUV source -> RGB dst. An RGB source -> RGB dst
	 * (mdp_kd_rgb2rgb) is a pure repack: NO WROT CSC (and NO RDMA CSC -- see
	 * rdma.transform below). */
	u32 wmat = (f->rgb && !srcrgb) ? 0x61 : 0;
	u32 strip = mdp_kd_wrot_lines > 0 ? (u32)mdp_kd_wrot_lines : 32;
	u32 tilew = mdp_kd_tile_w > 0 ? (u32)mdp_kd_tile_w : 64;
	u32 nh = DIV_ROUND_UP(w, tilew);
	u32 ntiles = nh * DIV_ROUND_UP(h, strip);
	struct img_mmsys_ctrl *mc;
	int i;
	u32 t;

	/* Target 2 (mdp_kd_swap2rgb): a SWAPPED YUV source (NV21/NM21, NV61/NM61,
	 * YVYU, VYUY) -> RGB dst has no chroma-swap twin to compensate (the WROT 0x100
	 * bit fixes a YUV output, not an RGB one). UN-SWAP on the READ instead via the
	 * RDMA sourceSwap bit (SRC_CON bit14=0x4000; DpEngine_RDMA mapColorFormat:
	 * sourceSwap = DP_COLOR_GET_SWAP_ENABLE) so the pipe carries true YUV, then the
	 * WROT YUV->RGB CSC (mat_ctrl=0x61, set above) produces correct RGB. Only for
	 * the swapped-source RGB case: the YUV-dst swap-twin path keeps sourceSwap=0. */
	if (f->rgb && MDP_COLOR_IS_SWAPPED(in->format.colorformat) &&
	    (src422 || src444 || srcpacked ||
	     (mdp_kd_src_nv21 && mdp_color_is_plain_nv21(in->format.colorformat))))
		src_ctrl |= 0x4000;

	/* YV12 (3-plane SWAPPED, V before U): the buffer's iova[1]=V, iova[2]=U; the
	 * WROT writes chroma in (U,V) order, so pass them swapped (like the mt8195 path). */
	if (planar && MDP_COLOR_IS_SWAPPED(out->format.colorformat)) {
		u32 sw = oc; oc = ov; ov = sw;
	}
	/* YV12 SOURCE (3-plane SWAPPED): the buffer is Y,V,U so iova[1]=V, iova[2]=U.
	 * Feed RDMA chroma-1<-U (iova[2]), chroma-2<-V (iova[1]) so the pipe is true YUV;
	 * an exact plane-pointer swap (no swap-twin needed -- the output row is the plain
	 * output color). The per-tile chroma offsets are identical for both planes. */
	if (srci420 && MDP_COLOR_IS_SWAPPED(in->format.colorformat)) {
		u32 sw = iu; iu = iv; iv = sw;
	}
	if (tcount > IMG_MAX_SUBFRAMES_8183)
		tcount = IMG_MAX_SUBFRAMES_8183;
	if (tbase + tcount > ntiles)
		tcount = ntiles - tbase;
	memset(cfg, 0, sizeof(*cfg));
	cfg->num_components = 5;
	cfg->num_subfrms = tcount;

	for (i = 0; i < 5; i++) {
		struct img_compparam_8183 *p = &cfg->components[i];
		struct img_comp_frame_8183 *fr = &p->frame;

		p->type = mdp_dt_comp_8183[i].type;
		p->id = mdp_dt_comp_8183[i].id;
		p->input = 0; p->num_outputs = 1; p->outputs[0] = 0;
		p->num_subfrms = tcount;
		fr->in_width = w; fr->in_height = h;
		fr->out_width = w; fr->out_height = h;
		fr->in_total_width = w; fr->out_total_width = w;
		fr->crop.width = w; fr->crop.height = h;

		if (i == 0) {		/* RDMA0: linear read (NV12 2-plane / packed 1-plane / I420 3-plane) */
			p->rdma.src_ctrl = src_ctrl;
			p->rdma.control = 0x10;
			p->rdma.iova[0] = iy;
			if (srcmm21) {	/* MM21/MT21 block-mode read (detile geometry) */
				/* RDMA reads the tiled buffer inh tall, crops to
				 * ALIGN(h,32); tile-row strides aw*32 (luma) / aw*16
				 * (chroma), MM21-sized iova_end. Byte-faithful to the
				 * detile compute (mdp_config_compute_8183 case 0). */
				fr->in_height = inh;
				fr->crop.height = ALIGN(h, 32);
				p->rdma.iova[1] = ic;
				p->rdma.iova_end[0] = iy + aw * inh;
				p->rdma.iova_end[1] = ic + aw * inh / 2;
				p->rdma.mf_bkgd = aw * 32;	/* luma tile-row stride */
				p->rdma.sf_bkgd = aw * 16;	/* chroma tile-row stride */
			} else if (srci420) {	/* 3 separate planes; chroma pitch istride/2 */
				p->rdma.iova[1] = iu;
				p->rdma.iova[2] = iv;
				p->rdma.iova_end[0] = iy + istride * h;
				p->rdma.iova_end[1] = iu + icstride * (h / 2);
				p->rdma.iova_end[2] = iv + icstride * (h / 2);
				p->rdma.mf_bkgd = istride;
				p->rdma.sf_bkgd = icstride;
			} else {
				bool oneplane = srcpacked || srcrgb;
				/* chroma plane rows: full-height for 4:2:2 (NV16) AND 4:4:4
				 * (NV24), half-height for 4:2:0 (NV12). cstride = full-width
				 * (istride*2) for NV24, istride otherwise. */
				u32 chroma_h = (src444 || src422) ? h : (h / 2);

				p->rdma.iova[1] = oneplane ? 0 : ic;
				p->rdma.iova_end[0] = iy + istride * h;
				p->rdma.iova_end[1] = oneplane ? 0 : (ic + cstride * chroma_h);
				p->rdma.mf_bkgd = istride;
				p->rdma.sf_bkgd = oneplane ? 0 : cstride;
			}
			/* RGB source -> YUV dst: enable the RDMA RGB->YUV CSC
			 * (MDP_RDMA_TRANSFORM_0 = (matrixSel<<24)|(colorTran<<16); default
			 * RGB->BT601 0x02010000). RGB source -> RGB dst (mdp_kd_rgb2rgb) is a
			 * pure repack -> NO CSC (transform 0). Every YUV source is true YUV
			 * -> no transform. */
			p->rdma.transform = (srcrgb && !f->rgb) ? mdp_rgb_transform : 0;
		} else if (i == 2) {
			fr->bypass = 1;
		} else if (i == 4) {	/* WROT0: target output format */
			p->wrot.iova[0] = oy;
			p->wrot.iova[1] = oc;
			p->wrot.iova[2] = ov;
			p->wrot.control = f->control;
			p->wrot.stride[0] = ostride;
			p->wrot.stride[1] = ocstride;
			p->wrot.stride[2] = planar ? ocstride : 0;
			p->wrot.mat_ctrl = wmat;	/* 0x61 YUV->RGB for f->rgb, else 0 */
		}

		for (t = 0; t < tcount; t++) {
			u32 g = tbase + t;
			u32 tx = g % nh, ty = g / nh;
			u32 x0 = tx * tilew, tw = min_t(u32, tilew, w - x0);
			u32 y0 = ty * strip, sh = min_t(u32, strip, h - y0);
			u32 choff = planar ? ((y0 / 2) * ocstride + x0 / 2)
					   : ((y0 >> cvshift) * ocstride + x0);
			struct img_comp_subfrm_8183 *s = &p->subfrms[t];

			s->in.left = x0; s->in.right = x0 + tw - 1;
			s->in.top = 0;   s->in.bottom = sh - 1;
			s->out.left = x0; s->out.right = x0 + tw - 1;
			s->out.top = 0;  s->out.bottom = sh - 1;
			s->out_horizontal = tx;
			s->out_vertical = ty;

			if (i == 0) {	/* RDMA: linear tw x sh tile (Y + interleaved UV) */
				/* 4:2:2 (NV16) and 4:4:4 (NV24) source chroma is full-height:
				 * row offset y0 (not y0/2). NV24 chroma is also full-width, so
				 * the per-column byte step is 2 (cxb) and the pitch is cstride. */
				u32 cy0 = (src422 || src444) ? y0 : (y0 / 2);
				u32 cxb = src444 ? 2 : 1;

				if (srcmm21) {
					/* MM21 tile-row addressing (detile geometry): luma
					 * 16x32 tile = aw*32 per tile-row + x0*32 per col;
					 * chroma 16x16 tile = aw*16 per tile-row + x0*16 per col.
					 * The RDMA source height is 32-line block aligned. */
					u32 csh = ALIGN(sh, 32);

					p->rdma.subfrms[t].offset[0] = (y0 / 32) * aw * 32 + x0 * 32;
					p->rdma.subfrms[t].offset[1] = (y0 / 32) * aw * 16 + x0 * 16;
					p->rdma.subfrms[t].offset[2] = (y0 / 32) * aw * 16 + x0 * 16;
					p->rdma.subfrms[t].src  = (csh << 16) | tw;
					p->rdma.subfrms[t].clip = (sh << 16) | tw;
					p->rdma.subfrms[t].clip_ofst = 0;
				} else {
				if (srci420) {
					/* 3-plane: chroma plane pitch istride/2, col x0/2 */
					p->rdma.subfrms[t].offset[0] = y0 * istride + x0;
					p->rdma.subfrms[t].offset[1] = (y0 / 2) * icstride + x0 / 2;
					p->rdma.subfrms[t].offset[2] = (y0 / 2) * icstride + x0 / 2;
				} else {
					/* packed/RGB: one plane, bppsrc bytes/px x-offset, no
					 * chroma plane (RGB = 2/3/4 B/px; packed-422 = 2). */
					bool oneplane = srcpacked || srcrgb;

					p->rdma.subfrms[t].offset[0] = y0 * istride + x0 * bppsrc;
					p->rdma.subfrms[t].offset[1] = oneplane ? 0 : (cy0 * cstride + x0 * cxb);
					p->rdma.subfrms[t].offset[2] = oneplane ? 0 : (cy0 * cstride + x0 * cxb);
				}
				p->rdma.subfrms[t].src  = (sh << 16) | tw;
				p->rdma.subfrms[t].clip = (sh << 16) | tw;
				p->rdma.subfrms[t].clip_ofst = 0;
				}
			} else if (i == 4) {	/* WROT: tw x sh tile (<=64x32 buffer) */
				p->wrot.subfrms[t].offset[0] = y0 * ostride + x0 * bppY;
				p->wrot.subfrms[t].offset[1] = choff;
				p->wrot.subfrms[t].offset[2] = choff;
				p->wrot.subfrms[t].src  = (sh << 16) | tw;
				p->wrot.subfrms[t].clip = (sh << 16) | tw;
				p->wrot.subfrms[t].clip_ofst = 0;
				/* line_num overridable for the planar-de-interleave RE sweep
				 * (mdp_kd_linenum); default sh (=tile height, works for NV21). */
				p->wrot.subfrms[t].main_buf = (tw << 16) |
					((mdp_kd_linenum > 0 ? (u32)mdp_kd_linenum : sh) << 8);
			}
		}
	}

	for (t = 0; t < tcount; t++) {
		mc = &cfg->ctrls[t];
		mc->num_sets = ARRAY_SIZE(mdp_dt_mux_8183);
		for (i = 0; i < (int)ARRAY_SIZE(mdp_dt_mux_8183); i++) {
			mc->sets[i].reg = mdp_dt_mux_8183[i].reg;
			mc->sets[i].value = mdp_dt_mux_8183[i].value;
			mc->sets[i].subsys_id = MDP_MT8183_MMSYS_SUBSYS_ID;
		}
	}

	if (mdp_kd_debug_on()) {
		struct device *dev = &mdp->pdev->dev;
		const struct mdp_rdma_data_8183 *rd = &cfg->components[0].rdma;
		const struct mdp_wrot_data_8183 *wr = &cfg->components[4].wrot;

		dev_info(dev, "MDPKD-8183-FMTCONV cfg: in=%#x out=%#x w=%u h=%u istride=%u ostride=%u bppsrc=%u srcrgb=%d nsub=%u\n",
			 in->format.colorformat, out->format.colorformat, w, h,
			 istride, ostride, bppsrc, srcrgb, cfg->num_subfrms);
		dev_info(dev, "MDPKD-8183-FMTCONV RDMA: src_ctrl=%#x control=%#x transform=%#x iova=[%#x,%#x,%#x] iova_end0=%#x mf_bkgd=%#x sf_bkgd=%#x sub0(off=[%#x,%#x] src=%#x)\n",
			 rd->src_ctrl, rd->control, rd->transform,
			 rd->iova[0], rd->iova[1], rd->iova[2], rd->iova_end[0],
			 rd->mf_bkgd, rd->sf_bkgd, rd->subfrms[0].offset[0],
			 rd->subfrms[0].offset[1], rd->subfrms[0].src);
		dev_info(dev, "MDPKD-8183-FMTCONV WROT: control=%#x mat_ctrl=%#x stride=[%#x,%#x,%#x] iova=[%#x,%#x,%#x] sub0(off=[%#x,%#x] main_buf=%#x)\n",
			 wr->control, wr->mat_ctrl, wr->stride[0], wr->stride[1],
			 wr->stride[2], wr->iova[0], wr->iova[1], wr->iova[2],
			 wr->subfrms[0].offset[0], wr->subfrms[0].offset[1],
			 wr->subfrms[0].main_buf);
	}
}

/*
 * Chunk-aware single-plane GREY Y rotate for the hybrid path: rotate the Y plane
 * of the NV12 input (RDMA reads NV12, WROT writes GREY), emitting vertical strips
 * [sbase, sbase+scount) of the output. A single config holds <= IMG_MAX_SUBFRAMES
 * strips (<=448 rows); taller frames span multiple GCE chunks (detile_nchunks),
 * each calling this with its strip range. Per-strip offsets use the ABSOLUTE input
 * row y0=(sbase+t)*strip, so each chunk lands at the right rotated output position.
 * No-swap modes only. The chroma is handled CPU-side (mdp_hybrid_chroma_rotate).
 */
static void mdp_config_compute_8183_yrot(struct mdp_dev *mdp,
		struct img_ipi_frameparam *param, struct img_config *config,
		u32 control, enum mdp_wrot_ofst mode, u32 tbase, u32 tcount)
{
	struct img_config_8183 *cfg = &config->config_8183;
	const struct img_image_buffer *in = &param->inputs[0].buffer;
	const struct img_image_buffer *out = &param->outputs[0].buffer;
	u32 w = in->format.width, h = in->format.height;
	u32 istride = in->format.plane_fmt[0].stride;
	u32 ostride = out->format.plane_fmt[0].stride;
	u32 iy = in->iova[0], ic = in->iova[1];
	u32 oy = out->iova[0];
	u32 strip = mdp_kd_wrot_lines > 0 ? (u32)mdp_kd_wrot_lines : 32;
	u32 nh;
	struct img_mmsys_ctrl *mc;
	int i;
	u32 t;

	if (mdp_kd_io_ovr) {		/* scratch I/O override (transpose passes) */
		const struct mdp_kd_io *o = mdp_kd_io_ovr;

		if (o->iw) { w = o->iw; h = o->ih; istride = o->istride; iy = o->iy; ic = o->ic; }
		if (o->ow) { ostride = o->ostride; oy = o->oy; }
	}
	nh = DIV_ROUND_UP(w, MDP_HYBRID_ROT_TILEW);	/* column tiles (width >1536) */
	if (tcount > IMG_MAX_SUBFRAMES_8183)
		tcount = IMG_MAX_SUBFRAMES_8183;
	memset(cfg, 0, sizeof(*cfg));
	cfg->num_components = 5;
	cfg->num_subfrms = tcount;

	for (i = 0; i < 5; i++) {
		struct img_compparam_8183 *p = &cfg->components[i];
		struct img_comp_frame_8183 *f = &p->frame;

		p->type = mdp_dt_comp_8183[i].type;
		p->id = mdp_dt_comp_8183[i].id;
		p->input = 0;
		p->num_outputs = 1;
		p->outputs[0] = 0;
		p->num_subfrms = tcount;
		f->in_width = w; f->in_height = h;
		f->out_width = w; f->out_height = h;
		f->in_total_width = w; f->out_total_width = w;
		f->crop.width = w; f->crop.height = h;

		if (i == 0) {		/* RDMA0: linear NV12 read (only Y used by GREY WROT) */
			p->rdma.src_ctrl = 0x060c;
			p->rdma.control = 0x10;
			p->rdma.iova[0] = iy;
			p->rdma.iova[1] = ic;
			p->rdma.iova_end[0] = iy + istride * h;
			p->rdma.iova_end[1] = ic + istride * h / 2;
			p->rdma.mf_bkgd = istride;
			p->rdma.sf_bkgd = istride;
		} else if (i == 2) {
			f->bypass = 1;
		} else if (i == 4) {	/* WROT0: GREY single-plane Y write */
			p->wrot.iova[0] = oy;
			p->wrot.iova[1] = 0;
			p->wrot.iova[2] = 0;
			p->wrot.control = control;
			p->wrot.stride[0] = ostride;
			p->wrot.stride[1] = 0;
			p->wrot.stride[2] = 0;
			p->wrot.mat_ctrl = 0;
		}

		for (t = 0; t < tcount; t++) {
			struct img_comp_subfrm_8183 *s = &p->subfrms[t];
			u32 g = tbase + t;			/* global tile index */
			u32 tx = g % nh, ty = g / nh;		/* column tile, row strip */
			u32 x0 = tx * MDP_HYBRID_ROT_TILEW;
			u32 tw = min_t(u32, MDP_HYBRID_ROT_TILEW, w - x0);
			u32 y0 = ty * strip;
			u32 sh = min_t(u32, strip, h - y0);
			u32 wy;

			s->in.left = x0; s->in.right = x0 + tw - 1;
			s->in.top = 0;   s->in.bottom = sh - 1;
			s->out.left = x0; s->out.right = x0 + tw - 1;
			s->out.top = 0;  s->out.bottom = sh - 1;
			s->out_horizontal = tx;
			s->out_vertical = ty;

			/* output pos of the tile's input(y0,x0); column reversal for
			 * hflip/rot180 (w-1-x0), forward for vflip (x0). */
			switch (mode) {
			case MDP_OFST_ROW_REV:  wy = y0 * ostride + (w - 1 - x0); break;
			case MDP_OFST_FULL_REV: wy = (h - 1 - y0) * ostride + (w - 1 - x0); break;
			default:                wy = (h - 1 - y0) * ostride + x0; break;
			}

			if (i == 0) {
				p->rdma.subfrms[t].offset[0] = y0 * istride + x0;
				p->rdma.subfrms[t].src  = (sh << 16) | tw;
				p->rdma.subfrms[t].clip = (sh << 16) | tw;
				p->rdma.subfrms[t].clip_ofst = 0;
			} else if (i == 4) {
				p->wrot.subfrms[t].offset[0] = wy;
				p->wrot.subfrms[t].src  = (sh << 16) | tw;
				p->wrot.subfrms[t].clip = (sh << 16) | tw;
				p->wrot.subfrms[t].main_buf =
					(tw << 16) | (min_t(u32, sh, 32) << 8);
			}
		}
	}

	for (t = 0; t < tcount; t++) {
		mc = &cfg->ctrls[t];
		mc->num_sets = ARRAY_SIZE(mdp_dt_mux_8183);
		for (i = 0; i < (int)ARRAY_SIZE(mdp_dt_mux_8183); i++) {
			mc->sets[i].reg = mdp_dt_mux_8183[i].reg;
			mc->sets[i].value = mdp_dt_mux_8183[i].value;
			mc->sets[i].subsys_id = MDP_MT8183_MMSYS_SUBSYS_ID;
		}
	}
}

/*
 * 2D-tiled swap-Y rotate for rot90/270: the HW GREY swap-Y line buffer limits the
 * INPUT WIDTH to ~768 (blk_width = input width) AND the per-tile output width (= the
 * input-HEIGHT segment) to ~1536. So tile the input into a 2D grid: width into
 * <=MDP_HYBRID_SWAP_TILEW column strips x height into <=MDP_HYBRID_SWAP_TILEH segments.
 * Each tile [x0,x0+tw) x [y0,y0+th) maps to an independent output sub-rectangle
 * (rot90: rows [x0,x0+tw), cols anchored at h-1-y0; rot270: rows anchored at w-1-x0,
 * cols [y0,..)), so no cross-tile dependency. [tbase, tbase+tcount) selects this GCE
 * chunk's tiles (the whole grid is ceil(ntiles/14) chunks). For h<=TILEH this reduces
 * to the validated 1D width-tiling (nht=1, y0=0, th=h). ostride = output width = h.
 * The chroma is CPU dim-swap (size-free, done once in device_run).
 */
static void mdp_config_compute_8183_yrot_swap(struct mdp_dev *mdp,
		struct img_ipi_frameparam *param, struct img_config *config,
		u32 control, enum mdp_wrot_ofst mode, u32 tbase, u32 tcount)
{
	struct img_config_8183 *cfg = &config->config_8183;
	const struct img_image_buffer *in = &param->inputs[0].buffer;
	const struct img_image_buffer *out = &param->outputs[0].buffer;
	u32 w = in->format.width, h = in->format.height;
	u32 istride = in->format.plane_fmt[0].stride;
	u32 ostride = out->format.plane_fmt[0].stride;	/* output width = h */
	u32 iy = in->iova[0], ic = in->iova[1];
	u32 oy = out->iova[0];
	u32 nw, ntiles;
	struct img_mmsys_ctrl *mc;
	int i;
	u32 t;

	if (mdp_kd_io_ovr) {		/* scratch I/O override (transpose passes) */
		const struct mdp_kd_io *o = mdp_kd_io_ovr;

		if (o->iw) { w = o->iw; h = o->ih; istride = o->istride; iy = o->iy; ic = o->ic; }
		if (o->ow) { ostride = o->ostride; oy = o->oy; }
	}
	nw = DIV_ROUND_UP(w, MDP_HYBRID_SWAP_TILEW);	/* width column strips */
	ntiles = mdp_swap_ntiles(w, h);
	if (tcount > IMG_MAX_SUBFRAMES_8183)
		tcount = IMG_MAX_SUBFRAMES_8183;
	if (tbase + tcount > ntiles)
		tcount = ntiles - tbase;
	memset(cfg, 0, sizeof(*cfg));
	cfg->num_components = 5;
	cfg->num_subfrms = tcount;

	for (i = 0; i < 5; i++) {
		struct img_compparam_8183 *p = &cfg->components[i];
		struct img_comp_frame_8183 *f = &p->frame;

		p->type = mdp_dt_comp_8183[i].type;
		p->id = mdp_dt_comp_8183[i].id;
		p->input = 0;
		p->num_outputs = 1;
		p->outputs[0] = 0;
		p->num_subfrms = tcount;
		f->in_width = w; f->in_height = h;
		f->out_width = w; f->out_height = h;
		f->in_total_width = w; f->out_total_width = w;
		f->crop.width = w; f->crop.height = h;

		if (i == 0) {		/* RDMA0: linear NV12 read (only Y used) */
			p->rdma.src_ctrl = 0x060c;
			p->rdma.control = 0x10;
			p->rdma.iova[0] = iy;
			p->rdma.iova[1] = ic;
			p->rdma.iova_end[0] = iy + istride * h;
			p->rdma.iova_end[1] = ic + istride * h / 2;
			p->rdma.mf_bkgd = istride;
			p->rdma.sf_bkgd = istride;
		} else if (i == 2) {
			f->bypass = 1;
		} else if (i == 4) {	/* WROT0: GREY single-plane Y write (rotated) */
			p->wrot.iova[0] = oy;
			p->wrot.iova[1] = 0;
			p->wrot.iova[2] = 0;
			p->wrot.control = control;
			p->wrot.stride[0] = ostride;
			p->wrot.stride[1] = 0;
			p->wrot.stride[2] = 0;
			p->wrot.mat_ctrl = 0;
		}

		for (t = 0; t < tcount; t++) {
			struct img_comp_subfrm_8183 *s = &p->subfrms[t];
			u32 g = tbase + t;			/* global tile index */
			u32 tx = g % nw, ty = g / nw;		/* width strip, height segment */
			u32 x0 = tx * MDP_HYBRID_SWAP_TILEW;
			u32 tw = min_t(u32, MDP_HYBRID_SWAP_TILEW, w - x0);
			u32 y0 = ty * MDP_HYBRID_SWAP_TILEH;
			u32 th = min_t(u32, MDP_HYBRID_SWAP_TILEH, h - y0);
			u32 wy;

			s->in.left = x0; s->in.right = x0 + tw - 1;
			s->in.top = y0;  s->in.bottom = y0 + th - 1;
			s->out.left = x0; s->out.right = x0 + tw - 1;
			s->out.top = y0;  s->out.bottom = y0 + th - 1;
			s->out_horizontal = tx;
			s->out_vertical = ty;

			/* output pos of the tile's input(col x0,row y0): rot90 (ROW_REV) ->
			 * output row x0, col h-1-y0; rot270 (COL_REV) -> row w-1-x0, col y0.
			 * (h>TILEH adds the -y0 / +y0 column term vs the 1D path.) */
			if (mode == MDP_OFST_ROW_REV)
				wy = x0 * ostride + (h - 1 - y0);
			else
				wy = (w - 1 - x0) * ostride + y0;

			if (i == 0) {
				p->rdma.subfrms[t].offset[0] = y0 * istride + x0;
				p->rdma.subfrms[t].src  = (th << 16) | tw;
				p->rdma.subfrms[t].clip = (th << 16) | tw;
				p->rdma.subfrms[t].clip_ofst = 0;
			} else if (i == 4) {
				p->wrot.subfrms[t].offset[0] = wy;
				p->wrot.subfrms[t].src  = (th << 16) | tw;
				p->wrot.subfrms[t].clip = (th << 16) | tw;
				p->wrot.subfrms[t].main_buf =
					(tw << 16) | (min_t(u32, th, 32) << 8);
			}
		}
	}

	for (t = 0; t < tcount; t++) {
		mc = &cfg->ctrls[t];
		mc->num_sets = ARRAY_SIZE(mdp_dt_mux_8183);
		for (i = 0; i < (int)ARRAY_SIZE(mdp_dt_mux_8183); i++) {
			mc->sets[i].reg = mdp_dt_mux_8183[i].reg;
			mc->sets[i].value = mdp_dt_mux_8183[i].value;
			mc->sets[i].subsys_id = MDP_MT8183_MMSYS_SUBSYS_ID;
		}
	}
}

static void mdp_config_compute_8183_scale(struct mdp_dev *mdp,
					  struct img_ipi_frameparam *param,
					  struct img_config *config,
					  u32 tile_base, u32 tile_count)
{
	struct img_config_8183 *cfg = &config->config_8183;
	const struct img_image_buffer *in = &param->inputs[0].buffer;
	const struct img_image_buffer *out = &param->outputs[0].buffer;
	/* scratch I/O override (transpose pass 2: scratch_a -> scratch_b), applied
	 * INLINE at each local so it precedes the per-axis up/down + tile geometry
	 * below (which read iw/ih/ow/oh) without a declaration-after-statement. */
	const struct mdp_kd_io *o = mdp_kd_io_ovr;
	bool oi = o && o->iw, oo = o && o->ow;
	u32 iw = oi ? o->iw : in->format.width;
	u32 ih = oi ? o->ih : in->format.height;
	u32 ow = oo ? o->ow : out->format.width;
	u32 oh = oo ? o->oh : out->format.height;
	u32 istride = oi ? o->istride : in->format.plane_fmt[0].stride;
	u32 ostride = oo ? o->ostride : out->format.plane_fmt[0].stride;
	u32 iy = oi ? o->iy : in->iova[0], ic = oi ? o->ic : in->iova[1];
	u32 oy = oo ? o->oy : out->iova[0], oc = oo ? o->oc : out->iova[1];
	/* RSZ mode PER AXIS (H and V are independent in HW -- separate algo/table/
	 * coeff registers), so down / up / MIXED (one axis up, one down) are all just
	 * the per-axis choice:
	 *  down axis: CUB_ACC algo (2), table 17, CEIL coeff shift-20 (out-1)<<20/(in-1).
	 *  up   axis: 6-tap  algo (0), table 27, ROUND coeff shift-15 (in-1)<<15/(out-1)
	 *             (dpframework rsz8183 DpEngine_RSZ.cpp:493-511); a 1:1 axis stays
	 *             down-mode -> identity coeff 0x100000.
	 * control1 (mask 0x07ff01f3) = verTable<<21 | horTable<<16 | verAlgo<<7 |
	 * horAlgo<<5 | (hor-upscale ? 1<<4 [vertical_first]) | 0x3 [h_en|v_en]. This
	 * reproduces 0x02310143 (both down) and 0x037b0013 (both up) EXACTLY and picks
	 * per axis for mixed. A DOWN V axis keeps verTable overridable (mdp_kd_vertable
	 * HW-probe knob, default 17). */
	bool hup = (ow > iw), vup = (oh > ih);		/* per-axis upscale? (else down/1:1) */
	bool hsc = (ow != iw), vsc = (oh != ih);	/* per-axis scaled? */
	u32 hT = hup ? 27u : 17u;
	u32 vT = vup ? 27u : (u32)(mdp_kd_vertable & 0x1f);
	u32 ctrl1 = (vT << 21) | (hT << 16) | ((vup ? 0u : 2u) << 7) |
		    ((hup ? 0u : 2u) << 5) | (hup ? (1u << 4) : 0u) | 0x3u;
	u32 cx, cy;

	/* HW-iter: optionally disable RSZ clock-gating (PRZ_CONTROL_1 bit27). */
	ctrl1 |= (mdp_kd_dcm_dis ? (1u << 27) : 0u);
	cx = hup ? (u32)DIV_ROUND_CLOSEST_ULL((u64)(iw - 1) << 15, ow - 1)
		 : (u32)DIV_ROUND_UP_ULL((u64)(ow - 1) << 20, iw - 1);
	cy = vup ? (u32)DIV_ROUND_CLOSEST_ULL((u64)(ih - 1) << 15, oh - 1)
		 : (u32)DIV_ROUND_UP_ULL((u64)(oh - 1) << 20, ih - 1);
	/* OUTPUT-tile geometry: even split per dim, <=254 (WROT) + input within the
	 * ~512-px RSZ line buffer; a dim that already fits is NOT over-tiled. */
	u32 nh, nv, tilew, tileh, t, cx_use, cy_use;
	struct img_mmsys_ctrl *mc;
	int i;
	const u32 TAP = 8;

	mdp_scale_tiles_8183(iw, ih, ow, oh, &nh, &nv, &tilew, &tileh);
	if (tile_count > IMG_MAX_SUBFRAMES_8183)
		tile_count = IMG_MAX_SUBFRAMES_8183;

	/* Scaled axis (up OR down) -> PHASE-0 non-overlapping strips. The mt8183 RSZ
	 * stalls on a non-zero resample phase (HW-confirmed 2026-06-19), so a tile with
	 * out_left>0 (or out_top>0) cannot carry the tpipe phase. Instead each output
	 * strip is an independent out_left=0/out_top=0 / phase=0 scale of its own input
	 * span. The input span is the EXACT contiguous floor boundary [floor(otl*iw/ow),
	 * floor((otl+otw)*iw/ow)) -- strip k ends exactly where strip k+1 begins and the
	 * widths sum to iw, with NO gap/overlap. The per-strip coeff (down: CEIL shift-20
	 * (otw-1)<<20/(in-1); up: ROUND shift-15 (in-1)<<15/(otw-1)) makes the RSZ emit
	 * EXACTLY otw. Odd strip boundaries are handled by the per-tile control2 chroma
	 * sampler below. tpc=1 (mdp_scale_tiles_per_chunk_8183) makes each strip its own
	 * single-tile job. Minor: a <=1px seam at strip boundaries (no tap overlap).
	 * Per axis: hsc/vsc gate the strip override (any scaled axis); hup/vup pick the
	 * coeff direction -- so MIXED (one axis up, one down) just uses each axis's own. */
	cx_use = cx;
	cy_use = cy;
	if (hsc) {
		u32 stx = tile_base % nh;
		u32 sotl = stx * tilew, sotw = min(tilew, ow - sotl);
		u32 sl = (u32)(((u64)sotl * iw) / ow);
		u32 sr = min(iw, (u32)(((u64)(sotl + sotw) * iw) / ow));
		u32 d = (sotw > 1) ? sotw - 1 : 1;

		if (sr < sl + 2)
			sr = sl + 2;
		cx_use = hup ? (u32)DIV_ROUND_CLOSEST_ULL((u64)(sr - sl - 1) << 15, d)
			     : (u32)DIV_ROUND_UP_ULL((u64)(sotw - 1) << 20, sr - sl - 1);
	}
	if (vsc) {
		u32 sty = tile_base / nh;
		u32 soty = sty * tileh, soth = min(tileh, oh - soty);
		u32 st = (u32)(((u64)soty * ih) / oh);
		u32 sb = min(ih, (u32)(((u64)(soty + soth) * ih) / oh));
		u32 d = (soth > 1) ? soth - 1 : 1;

		if (sb < st + 2)
			sb = st + 2;
		cy_use = vup ? (u32)DIV_ROUND_CLOSEST_ULL((u64)(sb - st - 1) << 15, d)
			     : (u32)DIV_ROUND_UP_ULL((u64)(soth - 1) << 20, sb - st - 1);
	}

	memset(cfg, 0, sizeof(*cfg));
	cfg->num_components = 5;
	cfg->num_subfrms = tile_count;

	for (i = 0; i < 5; i++) {
		struct img_compparam_8183 *p = &cfg->components[i];
		struct img_comp_frame_8183 *f = &p->frame;
		bool pre = (i <= 1);	/* RDMA0/CCORR0 are input-side; RSZ0 straddles */

		p->type = mdp_dt_comp_8183[i].type;
		p->id = mdp_dt_comp_8183[i].id;
		p->input = 0;
		p->num_outputs = 1;
		p->outputs[0] = 0;
		p->num_subfrms = tile_count;

		f->in_width = (i <= 2) ? iw : ow;  f->in_height = (i <= 2) ? ih : oh;
		f->out_width = pre ? iw : ow;      f->out_height = pre ? ih : oh;
		f->in_total_width = (i <= 2) ? iw : ow;
		f->out_total_width = pre ? iw : ow;
		f->crop.width = (i <= 2) ? iw : ow; f->crop.height = (i <= 2) ? ih : oh;

		switch (i) {
		case 0:	/* RDMA0: reads linear NV12 input. */
			p->rdma.src_ctrl = mdp_csc_srcctrl;	/* 0x060c linear (detile-faithful) */
			p->rdma.control = mdp_csc_rdmacon;	/* 0x10 simple_mode */
			p->rdma.iova[0] = iy;
			p->rdma.iova[1] = ic;
			p->rdma.iova_end[0] = iy + istride * ih;
			p->rdma.iova_end[1] = ic + istride * ih / 2;
			p->rdma.mf_bkgd = istride;
			p->rdma.sf_bkgd = istride;
			break;
		case 2:	/* RSZ0 (SCL0): ACTIVE downscale (control1 + coeff). */
			f->bypass = 0;
			p->rsz.coeff_step_x = cx_use;	/* per-strip for hdec */
			p->rsz.coeff_step_y = cy_use;	/* per-strip for vdec */
			p->rsz.control1 = ctrl1;
			break;
		case 4:	/* WROT0: writes linear NV12 (scaled output). */
			p->wrot.iova[0] = oy;
			p->wrot.iova[1] = oc;
			p->wrot.control = 0x500c;		/* VIDO_CTRL NV12, no rotate */
			p->wrot.stride[0] = ostride;
			p->wrot.stride[1] = ostride;
			break;
		/* case 1 (CCORR0) + case 3 (PATH0_SOUT): relay, geometry only. */
		}

		for (t = 0; t < tile_count; t++) {
			u32 g = tile_base + t;
			u32 tx = g % nh, ty = g / nh;
			u32 otl = tx * tilew, otw = min(tilew, ow - otl);
			u32 oty = ty * tileh, oth = min(tileh, oh - oty);
			/* INPUT region for this OUTPUT tile (downscale: in = out/coeff). */
			u32 ipl = (u32)(((u64)otl << 20) / cx);
			u32 ipr = (u32)(((u64)(otl + otw - 1) << 20) / cx);
			u32 ipt = (u32)(((u64)oty << 20) / cy);
			u32 ipb = (u32)(((u64)(oty + oth - 1) << 20) / cy);
			u32 inl = (ipl > TAP ? ipl - TAP : 0) & ~1u;
			u32 inr = min(iw - 1, ipr + TAP);
			u32 intp = (ipt > TAP ? ipt - TAP : 0) & ~1u;
			u32 inb = min(ih - 1, ipb + TAP);
			u32 itw = inr - inl + 1, ith = inb - intp + 1;
			u32 phx = ((u32)otl << 20) - inl * cx;	/* EXACT tpipe phase */
			u32 phy = ((u32)oty << 20) - intp * cy;
			struct img_comp_subfrm_8183 *s = &p->subfrms[t];
			bool out_side = (i >= 3);		/* PATH0_SOUT/WROT use output coords */

			/* phase-0 non-overlapping H strip (any scaled H axis, up or down) --
			 * override the tpipe overlap/phase with this strip's own input span
			 * (the RDMA X-offset reads it, the WROT X-offset writes otl). phx=0 so
			 * the RSZ never sees the horizontal phase that stalls the HW. The span
			 * formula is identical for up/down (only the coeff above differs). */
			if (hsc) {
				/* EXACT contiguous floor boundaries: strip [floor(otl*iw/ow),
				 * floor((otl+otw)*iw/ow)) -- abuts its neighbours, no gap/
				 * overlap. coeff (cx_use) = this strip's, emits exactly otw. */
				inl = (u32)(((u64)otl * iw) / ow);
				inr = min(iw, (u32)(((u64)(otl + otw) * iw) / ow)) - 1;
				if (inr < inl + 1)
					inr = inl + 1;
				itw = inr - inl + 1;
				phx = 0;
			}
			/* phase-0 contiguous V strip (any scaled V axis; mirror of H). */
			if (vsc) {
				intp = (u32)(((u64)oty * ih) / oh);
				inb = min(ih, (u32)(((u64)(oty + oth) * ih) / oh)) - 1;
				if (inb < intp + 1)
					inb = intp + 1;
				ith = inb - intp + 1;
				phy = 0;
			}

			/* generic region: input rect (pre-RSZ) or output rect (post-RSZ);
			 * RSZ (i==2) straddles (in=input rect, out=output rect) + phase. */
			s->in.left = (i >= 3) ? otl : inl;
			s->in.right = (i >= 3) ? (otl + otw - 1) : inr;
			s->in.top = (i >= 3) ? oty : intp;
			s->in.bottom = (i >= 3) ? (oty + oth - 1) : inb;
			s->out.left = out_side || i == 2 ? otl : inl;
			s->out.right = (out_side || i == 2) ? (otl + otw - 1) : inr;
			s->out.top = (out_side || i == 2) ? oty : intp;
			s->out.bottom = (out_side || i == 2) ? (oty + oth - 1) : inb;
			s->out_horizontal = tx;
			s->out_vertical = ty;
			if (i == 2) {	/* RSZ carries the resample phase (generic luma/chroma) */
				s->luma.left = phx >> 20;
				s->luma.left_subpix = phx & 0xfffff;
				s->luma.top = phy >> 20;
				s->luma.top_subpix = phy & 0xfffff;
				s->chroma.left = (phx >> 1) >> 20;
				s->chroma.left_subpix = (phx >> 1) & 0xfffff;
				s->chroma.top = (phy >> 1) >> 20;
				s->chroma.top_subpix = (phy >> 1) & 0xfffff;
			}

			switch (i) {
			case 0:	/* RDMA0: read the input region [inl..inr, intp..inb]. */
				p->rdma.subfrms[t].offset[0] = intp * istride + inl;
				p->rdma.subfrms[t].offset[1] = (intp / 2) * istride + inl;
				p->rdma.subfrms[t].offset[2] = (intp / 2) * istride + inl;
				p->rdma.subfrms[t].src  = (ith << 16) | itw;
				p->rdma.subfrms[t].clip = (ith << 16) | itw;
				break;
			case 2:	/* RSZ0: resample input tile -> output tile. */
				/* PRZ_CONTROL_2 bits 11..13 = the 4:2:0<->4:4:4 sampler
				 * controls, computed per-tile from geometry (dpframework
				 * rsz8183 onConfigTile :630-675), NOT copied from the
				 * mt8195 oracle: drs_lclip_en<<11 = use121&&inLeft>0 (0 for
				 * NV12, H-subsampled -> use121=0); drs_padding_dis<<12 =
				 * (inTileXRight odd); urs_clip_en<<13 = (outTileXRight <
				 * outFrameWidth-1). */
				p->rsz.subfrms[t].control2 =
					(((inr & 1u) ? 1u : 0u) << 12) |
					(((otl + otw - 1u) < (ow - 1u) ? 1u : 0u) << 13);
				p->rsz.subfrms[t].src  = (ith << 16) | itw;	/* PRZ_INPUT */
				p->rsz.subfrms[t].clip = (oth << 16) | otw;	/* PRZ_OUTPUT */
				break;
			case 4:	/* WROT0: write the output tile [otl..oty]. */
				p->wrot.subfrms[t].offset[0] = oty * ostride + otl;
				p->wrot.subfrms[t].offset[1] = (oty / 2) * ostride + otl;
				p->wrot.subfrms[t].offset[2] = (oty / 2) * ostride + otl;
				p->wrot.subfrms[t].src  = (oth << 16) | otw;
				p->wrot.subfrms[t].clip = (oth << 16) | otw;
				/* VIDO_MAIN_BUF_SIZE = (blk_width<<16) | (line_num<<8). The
				 * line_num field is 7-bit (mask 0x7f00); it is the WROT FIFO
				 * DEPTH, not the tile height. The old (oth<<8) put oth=240
				 * there -> overflow to 112 -> WROT drain corruption (flat-
				 * vertical). Use 0x4000 (line_num=64), matching the working
				 * detile + the mt8195 scale. */
				p->wrot.subfrms[t].main_buf = (otw << 16) | 0x4000;
				break;
			}
		}
	}

	for (t = 0; t < tile_count; t++) {
		mc = &cfg->ctrls[t];
		mc->num_sets = ARRAY_SIZE(mdp_dt_mux_8183);
		for (i = 0; i < ARRAY_SIZE(mdp_dt_mux_8183); i++) {
			mc->sets[i].reg = mdp_dt_mux_8183[i].reg;
			mc->sets[i].value = mdp_dt_mux_8183[i].value;
			mc->sets[i].subsys_id = MDP_MT8183_MMSYS_SUBSYS_ID;
		}
	}

	if (mdp_kd_debug_on()) {
		struct device *dev = &mdp->pdev->dev;
		const struct mdp_rsz_data_8183 *rz = &cfg->components[2].rsz;

		dev_info(dev, "MDPKD-8183-SCALE %ux%u->%ux%u down: coeff=[%#x,%#x] tile=%ux%u nh=%u nv=%u base=%u nsub=%u\n",
			 iw, ih, ow, oh, cx, cy, tilew, tileh, nh, nv, tile_base, tile_count);
		dev_info(dev, "MDPKD-8183-SCALE RSZ frame: coeff_x=%#x coeff_y=%#x control1=%#x control2=%#x bypass=%u\n",
			 rz->coeff_step_x, rz->coeff_step_y, rz->control1, rz->control2,
			 cfg->components[2].frame.bypass);
		for (i = 0; i < 5; i++) {
			struct img_compparam_8183 *p = &cfg->components[i];

			dev_info(dev, "MDPKD-8183-SCALE c%d type=%u id=%u f_in=%ux%u f_out=%ux%u crop=%ux%u nsub=%u byp=%u\n",
				 i, p->type, p->id, p->frame.in_width, p->frame.in_height,
				 p->frame.out_width, p->frame.out_height,
				 p->frame.crop.width, p->frame.crop.height,
				 p->num_subfrms, p->frame.bypass);
			for (t = 0; t < tile_count; t++) {
				struct img_comp_subfrm_8183 *s = &p->subfrms[t];

				dev_info(dev, "MDPKD-8183-SCALE c%d sub%u in[%d..%d,%d..%d] out[%d..%d,%d..%d] luma(l%d sl%u t%d st%u) ch(l%d sl%u t%d st%u) oh%d ov%d\n",
					 i, t, s->in.left, s->in.right, s->in.top, s->in.bottom,
					 s->out.left, s->out.right, s->out.top, s->out.bottom,
					 s->luma.left, s->luma.left_subpix, s->luma.top, s->luma.top_subpix,
					 s->chroma.left, s->chroma.left_subpix, s->chroma.top, s->chroma.top_subpix,
					 s->out_horizontal, s->out_vertical);
				if (i == 0)
					dev_info(dev, "MDPKD-8183-SCALE   c0 RDMA sub%u off=[%#x,%#x] src=%#x clip=%#x src_ctrl=%#x con=%#x\n",
						 t, p->rdma.subfrms[t].offset[0], p->rdma.subfrms[t].offset[1],
						 p->rdma.subfrms[t].src, p->rdma.subfrms[t].clip,
						 p->rdma.src_ctrl, p->rdma.control);
				if (i == 2)
					dev_info(dev, "MDPKD-8183-SCALE   c2 RSZ  sub%u ctl2=%#x PRZ_in=%#x PRZ_out=%#x\n",
						 t, p->rsz.subfrms[t].control2, p->rsz.subfrms[t].src,
						 p->rsz.subfrms[t].clip);
				if (i == 4)
					dev_info(dev, "MDPKD-8183-SCALE   c4 WROT sub%u off=[%#x,%#x] src=%#x clip=%#x main_buf=%#x ctl=%#x\n",
						 t, p->wrot.subfrms[t].offset[0], p->wrot.subfrms[t].offset[1],
						 p->wrot.subfrms[t].src, p->wrot.subfrms[t].clip,
						 p->wrot.subfrms[t].main_buf, p->wrot.control);
			}
		}
	}
}

/* Tiles per chunk for a kernel-direct scale frame. Multi-tile HORIZONTAL
 * decimation cannot run more than one active-RSZ subframe per GCE job (tile>=1
 * stalls / never executes -- HW-confirmed 2026-06-19: single-tile downscale is
 * correct in every h/v combination, multi-tile horizontal decimation stalls):
 * force 1 tile per chunk so each is a single-tile job. Horizontal 1:1 multi-tile
 * (output >254 with no horizontal scaling) packs up to IMG_MAX_SUBFRAMES_8183. */
static u32 mdp_scale_tiles_per_chunk_8183(const struct img_ipi_frameparam *param)
{
	u32 iw = param->inputs[0].buffer.format.width;
	u32 ih = param->inputs[0].buffer.format.height;
	u32 ow = param->outputs[0].buffer.format.width;
	u32 oh = param->outputs[0].buffer.format.height;

	/* Any SCALED axis (up or down) uses phase-0 strips -> 1 tile per job (each
	 * strip carries its own per-strip coeff, computed from tile_base). Only a
	 * pure 1:1 tiled copy (output >254, no scaling) packs IMG_MAX_SUBFRAMES. */
	return (ow != iw || oh != ih) ? 1 : IMG_MAX_SUBFRAMES_8183;
}

/* Component-1 self-test (mdp_kd_yrt): one pass of a 2-pass Y rotate via scratch.
 * chunk 0 = rot90 input(w x h) -> scratch_a(h x w); chunk 1 = rot270 scratch_a -> the
 * real output(w x h). rot90+rot270 = identity, so the output Y must equal the input Y
 * -- exercising mdp_kd_io_ovr on yrot_swap AND proving engine RDMA-read + WROT-write of
 * scratch (the one part of the transpose foundation not yet directly verified). */
static void mdp_kd_yrt_pass(struct mdp_dev *mdp, struct mdp_vpu_dev *vpu,
			    struct img_ipi_frameparam *param, u32 chunk_idx)
{
	u32 w = param->inputs[0].buffer.format.width;
	u32 h = param->inputs[0].buffer.format.height;
	struct img_config *cfg = (struct img_config *)vpu->config;
	struct mdp_kd_io io = {0};

	if (chunk_idx == 0) {		/* rot90: input (w x h) -> scratch_a (h x w) */
		io.ow = h; io.oh = w; io.ostride = h; io.oy = (u32)vpu->scratch_a_addr;
		mdp_kd_io_ovr = &io;
		mdp_config_compute_8183_yrot_swap(mdp, param, cfg,
						  0x00105007, MDP_OFST_ROW_REV,
						  0, IMG_MAX_SUBFRAMES_8183);
	} else {			/* rot270: scratch_a (h x w) -> real output (w x h) */
		io.iw = h; io.ih = w; io.istride = h; io.iy = (u32)vpu->scratch_a_addr;
		/* GREY rotate's RDMA still arms an NV12 UV read (discarded by the GREY
		 * WROT) -- point ic at scratch_a's post-Y region so it is mapped/readable
		 * (garbage UV is harmless; scratch is sized w*h*2). */
		io.ic = (u32)vpu->scratch_a_addr + h * w;
		mdp_kd_io_ovr = &io;
		mdp_config_compute_8183_yrot_swap(mdp, param, cfg,
						  0x00305007, MDP_OFST_COL_REV,
						  0, IMG_MAX_SUBFRAMES_8183);
	}
	mdp_kd_io_ovr = NULL;
}

/* Component 2: number of scale strips in the transpose pass-2 V-scale (scratch_a HxW
 * -> scratch_b H'xW'). Each strip is one chunk (the scale uses phase-0 strips, tpc=1),
 * so the whole 3-pass job is mdp_kd_tpose_nscale()+2 chunks (rot90 + strips + rot270). */
static u32 mdp_kd_tpose_nscale(const struct img_ipi_frameparam *param)
{
	u32 W = param->inputs[0].buffer.format.width;
	u32 H = param->inputs[0].buffer.format.height;
	u32 Wp = param->outputs[0].buffer.format.width;
	u32 Hp = param->outputs[0].buffer.format.height;
	u32 nh, nv, tw, th;

	mdp_scale_tiles_8183(H, W, Hp, Wp, &nh, &nv, &tw, &th);	/* pass-2 dims (transposed) */
	return nh * nv;
}

/* Component 2: dispatch one chunk of the 3-pass Y transpose. chunk 0 = rot90
 * input(WxH)->scratch_a(HxW); chunks 1..nscale = V-scale strip k of scratch_a(HxW)->
 * scratch_b(H'xW'); chunk nscale+1 = rot270 scratch_b(H'xW')->output(W'xH'). The scale
 * strip reuses mdp_config_compute_8183_scale with the scratch override + tile_base. */
static void mdp_kd_tpose_pass(struct mdp_dev *mdp, struct mdp_vpu_dev *vpu,
			      struct img_ipi_frameparam *param, u32 chunk_idx)
{
	u32 W = param->inputs[0].buffer.format.width;
	u32 H = param->inputs[0].buffer.format.height;
	u32 Wp = param->outputs[0].buffer.format.width;
	u32 Hp = param->outputs[0].buffer.format.height;
	u32 saY = (u32)vpu->scratch_a_addr, saUV = saY + H * W;
	u32 sbY = (u32)vpu->scratch_b_addr, sbUV = sbY + Hp * Wp;
	u32 saUVc = saY + Hp * Wp;	/* mdp_kd_tpose==2: copied H'xW' NV12 UV in scratch_a */
	u32 nscale = mdp_kd_tpose_nscale(param);
	bool cpvar = (mdp_kd_tpose == 2);	/* insert a GREY/cvt copy scratch_b->scratch_a */
	struct img_config *cfg = (struct img_config *)vpu->config;
	struct mdp_kd_io io = {0};

	mdp_kd_tpose_dims[0] = W; mdp_kd_tpose_dims[1] = H;	/* for the post-GCE scratch dump */
	mdp_kd_tpose_dims[2] = Wp; mdp_kd_tpose_dims[3] = Hp;

	if (chunk_idx == 0) {			/* rot90: input(WxH) -> scratch_a(HxW) */
		io.ow = H; io.oh = W; io.ostride = H; io.oy = saY;
		mdp_kd_io_ovr = &io;
		mdp_config_compute_8183_yrot_swap(mdp, param, cfg,
						  0x00105007, MDP_OFST_ROW_REV,
						  0, IMG_MAX_SUBFRAMES_8183);
	} else if (chunk_idx <= nscale) {	/* V-scale strip k: scratch_a(HxW) -> scratch_b(H'xW') */
		io.iw = H;  io.ih = W;  io.istride = H;  io.iy = saY;  io.ic = saUV;
		io.ow = Hp; io.oh = Wp; io.ostride = Hp; io.oy = sbY;  io.oc = sbUV;
		mdp_kd_io_ovr = &io;
		mdp_config_compute_8183_scale(mdp, param, cfg, chunk_idx - 1, 1);
	} else if (cpvar && chunk_idx == nscale + 1) {	/* GREY/cvt copy scratch_b -> scratch_a */
		/* non-active-RSZ WROT write of scratch_b's content into scratch_a, to test
		 * if the active-RSZ scale WROT's scratch_b memory layout is the drift. */
		mdp_config_compute_8183_cvt(mdp, cfg, sbY, sbUV, 0, 0x060c, Hp, Hp,
					    saY, saUVc, 0, 0x500c, Hp, Hp, Hp, Wp);
	} else {				/* rot270: scratch_(b|a)(H'xW') -> output(W'xH') */
		io.iw = Hp; io.ih = Wp; io.istride = Hp;
		io.iy = cpvar ? saY : sbY;	/* tpose==2 reads the cvt copy in scratch_a */
		io.ic = cpvar ? saUVc : sbUV;
		mdp_kd_io_ovr = &io;
		mdp_config_compute_8183_yrot_swap(mdp, param, cfg,
						  0x00305007, MDP_OFST_COL_REV,
						  0, IMG_MAX_SUBFRAMES_8183);
	}
	mdp_kd_io_ovr = NULL;
}

/* Recompute one chunk's img_config (global tiles [tile_base, tile_base+tile_count))
 * into the shared vpu->config, for mdp_m2m_device_run's multi-job loop. Chunk 0 is
 * produced by mdp_vpu_process; chunks 1..n-1 call this. No vpu->lock and no SCP IPI
 * are involved -- the chunks are serialized by the m2m job sequencing, and every
 * chunk addresses the same in/out dma-buf via per-tile offsets. */
void mdp_vpu_compute_8183_chunk(struct mdp_vpu_dev *vpu,
				struct img_ipi_frameparam *param, u32 chunk_idx)
{
	struct mdp_dev *mdp = vpu_to_mdp(vpu);
	enum mdp_wrot_ofst rmode;
	u32 yctl, uvctl, gctl, tpc, base, count;
	bool hrows, hcols;

	if (mdp_kd_yrt) {		/* component-1 self-test: 2-pass Y rotate via scratch */
		mdp_kd_yrt_pass(mdp, vpu, param, chunk_idx);
		return;
	}
	if (mdp_kd_tpose) {		/* component-2: 3-pass Y transpose downscale */
		mdp_kd_tpose_pass(mdp, vpu, param, chunk_idx);
		return;
	}

	/* I420/YV12-dst hybrid, multi-chunk HW NV12 write: mdp_vpu_process already
	 * reconfigured param->outputs to NV12 (persists across chunks -- same param),
	 * so the WROT writes 2-plane NV12 per tile. The CPU chroma de-interleave runs
	 * ONCE in device_run after the last chunk. f = the NV12-output fmtconv row
	 * (mdp_fmtconv_nv12 for an NV12 source, else source-specific via _for). */
	if (vpu->i420_deint) {
		u32 in_color = param->inputs[0].buffer.format.colorformat;
		/* NV12 source AND (MP arm only) MM21/MT21 source both drive the NV12
		 * identity WROT write (the srcmm21 read is keyed off in_color in the
		 * compute); every other admitted source uses its source-specific row. */
		bool nv12ident = mdp_color_is_plain_nv12(in_color) ||
				 (mdp_kd_mm21conv && mdp_color_is_mm21src(in_color));
		const struct mdp_fmtconv *f = nv12ident ?
			&mdp_fmtconv_nv12 : mdp_fmtconv_for(param);

		base = chunk_idx * IMG_MAX_SUBFRAMES_8183;
		count = min_t(u32, IMG_MAX_SUBFRAMES_8183, vpu->detile_ntiles - base);
		if (f)
			mdp_config_compute_8183_fmtconv(mdp, param,
				(struct img_config *)vpu->config, f, base, count);
		return;
	}

	/* GREY->NV12/NM12 src-expand (grey_src, NV12 out): the source was already
	 * reconfigured to NV12 @ scratch_b in mdp_vpu_process; NV12->NV12/NM12 is not an
	 * mdp_fmtconv_tbl row (that is blit's job), so recompute each chunk's WROT NV12
	 * identity write via mdp_fmtconv_nv12. Every OTHER grey dst rides the normal
	 * fmtconv/i420_deint recompute below (source is plain NV12). */
	if (vpu->grey_src &&
	    mdp_color_is_plain_nv12(param->outputs[0].buffer.format.colorformat)) {
		base = chunk_idx * IMG_MAX_SUBFRAMES_8183;
		count = min_t(u32, IMG_MAX_SUBFRAMES_8183, vpu->detile_ntiles - base);
		mdp_config_compute_8183_fmtconv(mdp, param,
			(struct img_config *)vpu->config, &mdp_fmtconv_nv12, base, count);
		return;
	}

	/* Hybrid NV12 rotate, multi-chunk HW Y: chunk k = tiles [k*14, +<=14) of the
	 * 2D grid (nh column tiles <=1536 wide x nv row strips <=32 tall). The chroma
	 * is CPU-side (once, in device_run), so only the Y plane is chunked here. */
	if (mdp_nv12_rotate_hybrid(param, &rmode, &gctl, &hrows, &hcols)) {
		u32 w = param->inputs[0].buffer.format.width;
		u32 h = param->inputs[0].buffer.format.height;
		u32 strip = mdp_kd_wrot_lines > 0 ? (u32)mdp_kd_wrot_lines : 32;
		u32 ntiles = DIV_ROUND_UP(w, MDP_HYBRID_ROT_TILEW) * DIV_ROUND_UP(h, strip);
		u32 base = chunk_idx * IMG_MAX_SUBFRAMES_8183;
		u32 count = min_t(u32, IMG_MAX_SUBFRAMES_8183, ntiles - base);

		mdp_config_compute_8183_yrot(mdp, param,
			(struct img_config *)vpu->config, gctl, rmode, base, count);
		return;
	}

	/* rot90/rot270 hybrid swap, multi-chunk HW Y (the gate allows ROW_REV any-width +
	 * COL_REV full-width; only COL_REV partial-width >14 is SCP'd): chunk k = tiles
	 * [k*14, +<=14) of the 2D swap grid; the CPU chroma dim-swap is done once in
	 * device_run. */
	{
		enum mdp_wrot_ofst smode;
		u32 sctl;
		bool sccw;

		if (mdp_nv12_rotate_hybrid_swap(param, &smode, &sctl, &sccw)) {
			u32 iw = param->inputs[0].buffer.format.width;
			u32 ih = param->inputs[0].buffer.format.height;
			u32 b = chunk_idx * IMG_MAX_SUBFRAMES_8183;
			u32 c = min_t(u32, IMG_MAX_SUBFRAMES_8183,
				      mdp_swap_ntiles(iw, ih) - b);

			mdp_config_compute_8183_yrot_swap(mdp, param,
				(struct img_config *)vpu->config, sctl, smode, b, c);
			return;
		}
	}

	/* Scratch-backed 5-pass NV12 rotate: de-interleave -> rot Y/Cb/Cr (single
	 * plane each) -> re-interleave. All passes write the shared vpu->config; the
	 * GCE serializes them (WROT-EOF -> next RDMA) so pass N reads pass N-1's
	 * output from scratch. (mdp_kd_5pass==2: round-trip de-int->re-int only.) */
	if (mdp_nv12_rotate_5pass(param, &rmode, &gctl)) {
		const struct img_image_buffer *in = &param->inputs[0].buffer;
		const struct img_image_buffer *out = &param->outputs[0].buffer;
		struct img_config *cfg = (struct img_config *)vpu->config;
		u32 w = in->format.width, h = in->format.height, wc = w / 2, hc = h / 2;
		u32 istride = in->format.plane_fmt[0].stride;
		u32 ostride = out->format.plane_fmt[0].stride;
		u32 iy = in->iova[0], ic = in->iova[1];
		u32 oy = out->iova[0], oc = out->iova[1];
		u32 saY = vpu->scratch_a_addr, saU = saY + w * h, saV = saU + wc * hc;
		u32 sbY = vpu->scratch_b_addr, sbU = sbY + w * h, sbV = sbU + wc * hc;
		bool rt = (mdp_kd_5pass == 2);		/* round-trip: skip rotations */
		u32 last = rt ? 1 : 4;			/* re-interleave chunk */
		u32 rsy = rt ? saY : sbY, rsu = rt ? saU : sbU, rsv = rt ? saV : sbV;

		if (chunk_idx == 0)			/* de-interleave NV12 -> I420 scratch_a */
			mdp_config_compute_8183_cvt(mdp, cfg,
				iy, ic, 0, 0x060c, istride, istride,
				saY, saU, saV, 0x00005008, w, wc, w, h);
		else if (chunk_idx == last)		/* re-interleave I420 -> NV12 out */
			mdp_config_compute_8183_cvt(mdp, cfg,
				rsy, rsu, rsv, 0x0608, w, wc,
				oy, oc, 0, 0x0000500c, ostride, ostride, w, h);
		else if (chunk_idx == 1)		/* rot Y: in.Y (NV12) -> scratch_b.Y */
			mdp_config_compute_8183_planerot(mdp, cfg,
				iy, ic, 0x060c, istride, sbY, gctl, w, w, h, rmode);
		else if (chunk_idx == 2)		/* rot Cb: scratch_a.U (GREY) -> scratch_b.U */
			mdp_config_compute_8183_planerot(mdp, cfg,
				saU, 0, 0x0607, wc, sbU, gctl, wc, wc, hc, rmode);
		else					/* chunk 3: rot Cr */
			mdp_config_compute_8183_planerot(mdp, cfg,
				saV, 0, 0x0607, wc, sbV, gctl, wc, wc, hc, rmode);
		return;
	}

	/* Separate-plane NV12 rotate (no-swap): 2 single-plane WROT passes --
	 * chunk 0 = Y as GREY, chunk 1 = the CbCr plane as opaque 16bpp. */
	if (mdp_nv12_rotate_2pass(param, &rmode, &yctl, &uvctl)) {
		if (chunk_idx == 0)
			mdp_config_compute_8183_xform(mdp, param,
				(struct img_config *)vpu->config, yctl, rmode, true);
		else
			mdp_config_compute_8183_uv16(mdp, param,
				(struct img_config *)vpu->config, uvctl, rmode);
		return;
	}

	tpc = mdp_frame_is_nv12_scale(param) ?
		  mdp_scale_tiles_per_chunk_8183(param) : IMG_MAX_SUBFRAMES_8183;
	base = chunk_idx * tpc;
	count = min_t(u32, tpc, vpu->detile_ntiles - base);

	/* Same tiled multi-job machinery for both mt8183 kernel-direct ops; the
	 * per-chunk recompute dispatches on the frame type (the gate functions are
	 * cheap and re-evaluate from param). CSC = NV12->RGB24; detile = MM21->NV12. */
	if (mdp_frame_is_nv12_rgb_csc(param))
		mdp_config_compute_8183_csc(mdp, param,
					    (struct img_config *)vpu->config, base, count);
	else if (mdp_frame_is_nv12_scale(param))
		mdp_config_compute_8183_scale(mdp, param,
					      (struct img_config *)vpu->config, base, count);
	else if (mdp_frame_is_nv12_fmtconv(param)) {
		const struct mdp_fmtconv *f = mdp_fmtconv_for(param);

		/* mt8183 swapped-src -> NV21/NM21 per-chunk recompute: same local NV12
		 * row substitution as the initial dispatch (mdp_kd_8183_swapdst). This
		 * function is mt8183-only (mdp_config_compute_8183_*). */
		if (!f && mdp_kd_8183_swapdst &&
		    mdp_color_is_plain_nv21(param->outputs[0].buffer.format.colorformat))
			f = &mdp_fmtconv_nv12;
		if (f)
			mdp_config_compute_8183_fmtconv(mdp, param,
				(struct img_config *)vpu->config, f, base, count);
	} else
		mdp_config_compute_8183(mdp, param,
					(struct img_config *)vpu->config, base, count);
}

/*
 * mt8188 SCP component-id remap (mdp_data->scp_comp_id_compact). The mt8188 geralt
 * scp.img emits each computed-config component's inner-id in the OLD compact MDP
 * numbering (WPEI=0,WPEO=1,CAMIN=2,RDMA0=3,RDMA2=4 .. WROT2=36,MERGE3=39) -- the
 * numbering enum mt8188_mdp_comp_id originally used. The comp-inner-id fix
 * (52135928a1dc) renumbered that enum to the mt8195 space (RDMA2=10 .. WROT2=55) so
 * the shared kernel-direct component tables (mdp_dt_comp[]/mdp_scale_comp[], written
 * in mt8195 numbering) resolve -- which made every KD pair pass but left every
 * SCP-fallback pair failing: mdp_cfg_get_id_public() now decodes the SCP's compact
 * id 4 as CAMIN (a DL_PATH unprobed on the Genio-510) -> "Uninit component inner id
 * 4" -> mdp_path_ctx_init error -> CMDQ send failed: -22 (the remaining 246/702 of
 * the format-pair matrix after the comp-inner-id fix). Translate the SCP-returned
 * config compact->mt8195 here so the single downstream reverse-map (mdp_path_ctx_init
 * / mdp_comp_ctx_config) sees mt8195 ids for BOTH the SCP and the KD paths. The KD
 * paths return early (never reach the SCP send) so they are untouched; mt8195's own
 * SCP already emits mt8195 ids (scp_comp_id_compact=false). Index = compact id;
 * value = its mt8195_mdp_comp_id twin (== the c-th smallest inner-id present on
 * mt8188, i.e. the pre-fix compact->post-fix map from commit 52135928a1dc).
 */
static const u8 mt8188_scp_compact_to_inner[40] = {
	 0,  1,  4,  8, 10, 11, 13, 15, 16, 17, /* WPEI WPEO CAMIN RDMA0 RDMA2 RDMA3 FG0 FG2 FG3 SVPP2MOUT */
	18, 19, 21, 22, 23, 25, 27, 28, 29, 31, /* SVPP3MOUT WARP0MOUT VPP0/1_SOUT PQ0_SOUT HDR0 HDR2 HDR3 AAL0 AAL2 */
	32, 33, 35, 36, 37, 39, 40, 41, 43, 44, /* AAL3 RSZ0 RSZ2 RSZ3 TDSHP0 TDSHP2 TDSHP3 COLOR0 COLOR2 COLOR3 */
	45, 47, 49, 50, 51, 53, 55, 56, 57, 58, /* OVL0 PAD0 PAD2 PAD3 TCC0 WROT0 WROT2 WROT3 MERGE2 MERGE3 */
};

static void mdp_remap_scp_compact_ids(struct mdp_vpu_dev *vpu)
{
	struct img_config *cfg = (struct img_config *)vpu->config;
	struct img_config_8195 *c = &cfg->config_8195;
	u32 i;

	for (i = 0; i < c->num_components && i < IMG_MAX_COMPONENTS; i++) {
		u32 t = c->components[i].type;

		if (t < ARRAY_SIZE(mt8188_scp_compact_to_inner))
			c->components[i].type = mt8188_scp_compact_to_inner[t];
	}
}

int mdp_vpu_process(struct mdp_vpu_dev *vpu, struct img_ipi_frameparam *param)
{
	struct mdp_dev *mdp = vpu_to_mdp(vpu);
	struct img_sw_addr addr = {};
	u32 gctl;			/* NV12->GREY rotate: WROT control */
	enum mdp_wrot_ofst gmode;	/* NV12->GREY rotate: offset mode */
	int ret;			/* SCP send rc (for the mt8188 comp-id remap) */

	mutex_lock(vpu->lock);
	if (mdp_vpu_shared_mem_alloc(vpu)) {
		dev_err(&mdp->pdev->dev, "VPU memory alloc fail!");
		mutex_unlock(vpu->lock);
		return -ENOMEM;
	}

	memset(vpu->config, 0, vpu->config_size);
	vpu->hybrid_chroma = false;	/* set only by the hybrid-rotate arm below */
	vpu->hybrid_swap = false;
	vpu->i420_deint = false;	/* set only by the I420/YV12-dst hybrid arm below */
	vpu->i420_deint_mp = false;	/* set only by the multi-planar YM12/YM21 arm below */
	vpu->ym16_deint = false;	/* set only by the 3-plane 4:2:2 dst arm below */
	vpu->grey_src = false;		/* set only by the GREY-source pre-expand arm below */
	vpu->kd_used = true;		/* cleared just before the SCP FRAME send below */
	param->config_data.va = (unsigned long)vpu->config;
	param->config_data.pa = vpu->config_addr;
	param->drv_data = (unsigned long)vpu;
	/* The SCP frameparam/work buffers are staged at the fallback below, only
	 * for a frame no kernel-direct branch claims (the branches only read
	 * param, so the SCP gets the same frame it would have got here). */
	mutex_unlock(vpu->lock);

	/* Single GCE submission unless the mt8183 multi-job detile sets it below. */
	vpu->detile_nchunks = 1;

	/* GREY (Y8, luma-only) SOURCE via CPU pre-expand (mdp_kd_src_grey). The RDMA
	 * has no Y8 read mode that presents neutral chroma (dpframework GREY read is
	 * Y-only), so the CPU copies Y + fills a neutral (0x80) UV plane into scratch_b
	 * (done in mdp_m2m_device_run, which has the src vaddr; here we only ensure the
	 * scratch + rewrite the source descriptor to plain NV12 @ scratch_b). Every
	 * existing NV12->X compute (fmtconv / CSC / I420-dst hybrid / YM12-dst mp) then
	 * yields GREY->X unchanged. scratch_b (not scratch_a) holds the NV12 so the mp
	 * arm can still use scratch_a for its UV redirect. The neutral chroma makes
	 * GREY->YUV = Y + 0x80, GREY->RGB = a gray image (CSC of neutral chroma). */
	if (mdp_kernel_direct && mdp_kd_src_grey &&
	    (CFG_CHECK(MT8183, mdp->mdp_data->mdp_plat_id) ||
	     CFG_CHECK(MT8195, mdp->mdp_data->mdp_plat_id)) &&
	    mdp_frame_is_grey_src(param)) {
		u32 w = param->inputs[0].buffer.format.width;
		u32 h = param->inputs[0].buffer.format.height;
		u32 istride = param->inputs[0].buffer.format.plane_fmt[0].stride;
		u32 out_color = param->outputs[0].buffer.format.colorformat;

		if (!mdp_vpu_scratch_ensure(vpu, (size_t)istride * h * 3 / 2)) {
			/* Reconfigure source -> plain NV12 backed by scratch_b. Keep the
			 * source stride so the CPU Y-copy is a straight istride*h memcpy and
			 * the chroma plane lands at scratch_b + istride*h. */
			param->inputs[0].buffer.format.colorformat = MDP_COLOR_NV12;
			param->inputs[0].buffer.iova[0] = (u32)vpu->scratch_b_addr;
			param->inputs[0].buffer.iova[1] =
				(u32)vpu->scratch_b_addr + istride * h;
			param->inputs[0].buffer.iova[2] = 0;
			param->inputs[0].buffer.format.plane_fmt[1].stride = istride;
			vpu->grey_src = true;

			/* GREY->NV12/NM12 (2-plane 4:2:0 identity) is blit's job, which is
			 * not kernel-direct on mt8183, so route it explicitly through the
			 * WROT NV12 write (mdp_fmtconv_nv12), like the I420-dst hybrid. Every
			 * other dst falls through to the normal NV12-source dispatch below.
			 * mt8195 HAS a kernel-direct NV12->NV12 blit, so there GREY->NV12
			 * simply falls through to it (src is now NV12 @ scratch_b). */
			if (mdp_color_is_plain_nv12(out_color) &&
			    CFG_CHECK(MT8183, mdp->mdp_data->mdp_plat_id)) {
				u32 ntiles = DIV_ROUND_UP(w, 64) * DIV_ROUND_UP(h, 32);

				vpu->detile_ntiles = ntiles;
				vpu->detile_nchunks =
					DIV_ROUND_UP(ntiles, IMG_MAX_SUBFRAMES_8183);
				mdp_config_compute_8183_fmtconv(mdp, param,
					(struct img_config *)vpu->config,
					&mdp_fmtconv_nv12, 0,
					min_t(u32, IMG_MAX_SUBFRAMES_8183, ntiles));
				dev_dbg(&mdp->pdev->dev,
					"kernel-direct GREY->NV12 src-expand (mt8183, %u tiles/%u chunk(s))\n",
					ntiles, vpu->detile_nchunks);
				return 0;
			}
			/* source is now NV12 @ scratch_b; fall through so the existing
			 * csc/fmtconv/i420dst/mp arms produce every other GREY->X dst. */
		}
	}

	if (mdp_kd_yrt && mdp_kernel_direct &&
	    CFG_CHECK(MT8183, mdp->mdp_data->mdp_plat_id)) {
		/* COMPONENT-1 SELF-TEST: 2-pass Y rotate via scratch (rot90 input ->
		 * scratch_a, rot270 scratch_a -> output == identity). Validates the
		 * mdp_kd_io_ovr plumbing on yrot_swap + engine scratch read/write. */
		u32 w = param->inputs[0].buffer.format.width;
		u32 h = param->inputs[0].buffer.format.height;

		if (!mdp_vpu_scratch_ensure(vpu, (size_t)w * h * 2)) {
			vpu->detile_ntiles = 2;
			vpu->detile_nchunks = 2;
			mdp_kd_yrt_pass(mdp, vpu, param, 0);
			return 0;
		}
	}

	if (mdp_kd_tpose && mdp_kernel_direct &&
	    CFG_CHECK(MT8183, mdp->mdp_data->mdp_plat_id) &&
	    param->outputs[0].buffer.format.width < param->inputs[0].buffer.format.width &&
	    param->outputs[0].buffer.format.height < param->inputs[0].buffer.format.height) {
		/* COMPONENT 2: 3-pass Y transpose downscale via scratch (rot90 -> V-scale
		 * -> rot270). Chunks = nh_scale + 2. Scratch sized for the HxW intermediate
		 * (>= the H'xW' one). Y only -- chroma is component 3. */
		u32 w = param->inputs[0].buffer.format.width;
		u32 h = param->inputs[0].buffer.format.height;

		if (!mdp_vpu_scratch_ensure(vpu, (size_t)w * h * 2)) {
			u32 nch = mdp_kd_tpose_nscale(param) +
				  (mdp_kd_tpose == 2 ? 3 : 2);	/* +1 for the cvt copy pass */

			vpu->detile_ntiles = nch;
			vpu->detile_nchunks = nch;
			mdp_kd_tpose_pass(mdp, vpu, param, 0);
			return 0;
		}
	}

	/* Kernel-direct: compute the detile img_config in-kernel, skip the SCP's
	 * per-frame SCP_IPI_MDP_FRAME parse. Each platform fills its own union
	 * member (config_8195 / config_8183 -- same union, different layout).
	 * The gate (mdp_frame_is_mm21_detile + the per-mode size bounds below)
	 * routes only the pure MM21->NV12 full-frame detile here; everything else
	 * falls through to the SCP. */
	if (mdp_kernel_direct && mdp_frame_is_mm21_detile(param)) {
		u32 w = param->outputs[0].buffer.format.width;
		u32 h = param->outputs[0].buffer.format.height;

		if (CFG_CHECK(MT8195, mdp->mdp_data->mdp_plat_id)) {
			if (param->type == MDP_STREAM_TYPE_DUAL_BITBLT) {
				/* two pixel pipes (out area >= 2073600); each half <= 1920 wide */
				const struct img_image_buffer *in = &param->inputs[0].buffer;
				const struct img_image_buffer *out = &param->outputs[0].buffer;
				struct img_config *cfgs = (struct img_config *)vpu->config;

				/* in==out: num_subfrms=1 RSZ-bypassed detile dual;
				 * block-padded (in.height > h, <64 rows): num_subfrms=2
				 * vertical-crop dual. Both kernel-direct (mdp_mm21_dual_kd_exact). */
				if (w <= 3840 && mdp_mm21_dual_kd_exact(param)) {	/* each half <= 1920; aw=ALIGN(w,64) handles non-aligned */
					u32 inh = in->format.height;
					void (*fill)(struct img_config_8195 *, int, u32, u32, u32,
						     u32, u32, u32, u32, const struct mdp_dual_wrot *) =
						(inh > h) ? mdp_fill_dual_pipe_8195_crop
							  : mdp_fill_dual_pipe_8195;

					fill(&cfgs[0].config_8195, 0, w, h, inh, in->iova[0],
						in->iova[1], out->iova[0], out->iova[1], NULL);
					fill(&cfgs[1].config_8195, 1, w, h, inh, in->iova[0],
						in->iova[1], out->iova[0], out->iova[1], NULL);
					dev_dbg(&mdp->pdev->dev, "kernel-direct dual config computed\n");
					return 0;
				}
			} else if (w <= 1920 && (unsigned long long)w * h < 2073600ULL) {
				mdp_config_compute_8195(param, (struct img_config *)vpu->config);
				dev_dbg(&mdp->pdev->dev, "kernel-direct config computed\n");
				return 0;
			}
		} else if (CFG_CHECK(MT8183, mdp->mdp_data->mdp_plat_id)) {
			/* mt8183 single pixel-pipe (pp_used=1). The detile is a 2D grid of
			 * ceil(w/tilew) x ceil(h/strip) 64x32 tiles. img_config holds at most
			 * IMG_MAX_SUBFRAMES_8183 (14) tiles, so a frame whose grid exceeds 14
			 * is split into ceil(ntiles/14) chunks, each a separate GCE submission
			 * into the SAME output dma-buf (mdp_m2m_device_run drives the loop).
			 * We compute CHUNK 0 here and publish the plan via vpu->detile_*.
			 * w bounded by the 13-bit MF_SRC_SIZE field (<=8191); decoders <=4096. */
			u32 tilew = mdp_kd_tile_w > 0 ? mdp_kd_tile_w : 64;
			u32 strip = mdp_kd_wrot_lines > 0 ? mdp_kd_wrot_lines : 32;
			u32 ntiles = DIV_ROUND_UP(w, tilew) * DIV_ROUND_UP(h, strip);

			if (w <= 4096) {
				vpu->detile_ntiles = ntiles;
				vpu->detile_nchunks =
					DIV_ROUND_UP(ntiles, IMG_MAX_SUBFRAMES_8183);
				mdp_config_compute_8183(mdp, param,
					(struct img_config *)vpu->config, 0,
					min_t(u32, IMG_MAX_SUBFRAMES_8183, ntiles));
				dev_dbg(&mdp->pdev->dev,
					"kernel-direct config (mt8183, %u tiles, %u chunk(s))\n",
					ntiles, vpu->detile_nchunks);
				return 0;
			}
			dev_dbg(&mdp->pdev->dev,
				"mt8183 detile %ux%u (%u tiles) w>4096; falling back to SCP\n",
				w, h, ntiles);
		}
	} else if (mdp_kernel_direct && mdp_frame_is_nv12_rgb_csc(param)) {
		/* Kernel-direct CSC (NV12 -> RGB24), sibling of the detile gate.
		 * The SCP CSC path wedges on v26100 firmware; kernel-direct bypasses
		 * it. One full-frame GCE submission (num_subfrms=1, detile_nchunks
		 * stays 1 -> mdp_m2m_device_run dispatches it via mdp_cmdq_send).
		 * CSC-HW-ITER: the 1920/4096 bounds mirror the detile (13-bit
		 * MF_SRC_SIZE; mt8195 single-pipe <=1920 like the detile). */
		u32 w = param->outputs[0].buffer.format.width;
		u32 h = param->outputs[0].buffer.format.height;

		if (CFG_CHECK(MT8195, mdp->mdp_data->mdp_plat_id)) {
			/* >1920 (DUAL_BITBLT): two pixel pipes, same linear primitive as
			 * blit with the CSC WROT (control 0xa0005101, mat_ctrl 0x61, RGB24
			 * 1-plane, per-half write offset outx*3 bytes, main_buf 0x2000).
			 * c0 reads linear NV12 with CSC src_ctrl 0xc (vs blit 0x60c).
			 * Byte-confirmed vs the v23042 SCP 2560x1088 pp1 oracle. */
			if (param->type == MDP_STREAM_TYPE_DUAL_BITBLT &&
			    w <= 3840 && !(w & 3)) {
				const struct img_image_buffer *in = &param->inputs[0].buffer;
				const struct img_image_buffer *out = &param->outputs[0].buffer;
				struct img_config *cfgs = (struct img_config *)vpu->config;
				u32 istride = in->format.plane_fmt[0].stride;
				u32 ostride = out->format.plane_fmt[0].stride;
				struct mdp_dual_wrot wr = {
					.iova = { out->iova[0], 0, 0 },
					.stride = { ostride, 0, 0 },
					.control = 0xa0005101, .mat_ctrl = 0x61,
					.pre_ultra = 0x0008804c, .main_buf = 0x2000,
					.ofmul = { 3, 0, 0 }, .ofshr = { 0, 0, 0 },
				};

				mdp_fill_dual_pipe_8195_linear(&cfgs[0].config_8195, 0, w, h,
					istride, in->iova[0], in->iova[1], 0xc, &wr, NULL);
				mdp_fill_dual_pipe_8195_linear(&cfgs[1].config_8195, 1, w, h,
					istride, in->iova[0], in->iova[1], 0xc, &wr, NULL);
				vpu->detile_nchunks = 1;
				dev_dbg(&mdp->pdev->dev,
					"kernel-direct dual CSC config (mt8195, %ux%u)\n", w, h);
				return 0;
			}
			/* Single SVPP2 pipe (cloned from mdp_config_compute_8195, <=1920). */
			if (w <= 1920 && (unsigned long long)w * h < 2073600ULL) {
				vpu->detile_nchunks = 1;
				mdp_config_compute_8195_csc(param,
						(struct img_config *)vpu->config);
				dev_dbg(&mdp->pdev->dev,
					"kernel-direct CSC config (mt8195, NV12->RGB24)\n");
				return 0;
			}
			dev_dbg(&mdp->pdev->dev,
				"mt8195 CSC %ux%u >1920 or dual; falling back to SCP\n",
				w, h);
		} else if (CFG_CHECK(MT8183, mdp->mdp_data->mdp_plat_id) &&
			   w <= 4096) {
			/* mt8183 CSC: the WROT RGB line buffer is 8-bit-bounded in
			 * BOTH width and height (<=255; HW-bisected 254 OK / 256 HANG).
			 * A frame >255 in either dim is split into a 2D grid of
			 * <=254x254 tiles (ceil(w/254) x ceil(h/254)); a grid > 14
			 * tiles chunks into ceil(ntiles/14) sequential GCE submissions
			 * via mdp_m2m_device_run -> mdp_cmdq_send_detile, exactly like
			 * the MM21 detile (the multi-job endpoint is shared; the chunk
			 * recompute dispatches CSC vs detile). w bounded by the 13-bit
			 * MF_SRC_SIZE field (<=8191); decoders <=4096. */
			u32 tile = MDP_CSC_8183_TILE;
			u32 ntiles = DIV_ROUND_UP(w, tile) * DIV_ROUND_UP(h, tile);

			vpu->detile_ntiles = ntiles;
			vpu->detile_nchunks =
				DIV_ROUND_UP(ntiles, IMG_MAX_SUBFRAMES_8183);
			mdp_config_compute_8183_csc(mdp, param,
				(struct img_config *)vpu->config, 0,
				min_t(u32, IMG_MAX_SUBFRAMES_8183, ntiles));
			dev_dbg(&mdp->pdev->dev,
				"kernel-direct CSC config (mt8183, NV12->RGB24, %u tiles, %u chunk(s))\n",
				ntiles, vpu->detile_nchunks);
			return 0;
		}
	} else if (mdp_kernel_direct && mdp_frame_is_nv12_blit(param)) {
		/* Kernel-direct blit (NV12 -> NV12, 1:1) -- frees the simplest 2D op
		 * from the SCP (KI-009). One full-frame GCE submission, like CSC.
		 * mt8195 single-pipe <=1920 (the dual-pipe blit split is not RE'd). */
		u32 w = param->outputs[0].buffer.format.width;
		u32 h = param->outputs[0].buffer.format.height;

		/* >1920 (out area >= 2,073,600): two pixel pipes (DUAL_BITBLT, set one
		 * layer up in mdp_check_pp_enable). Each half <= 1920 wide (w <= 3840);
		 * w divisible by 4 keeps pw + the chroma seam offset even. Mirrors the
		 * detile dual arm; the LINEAR primitive fills both pipe configs. */
		if (CFG_CHECK(MT8195, mdp->mdp_data->mdp_plat_id) &&
		    param->type == MDP_STREAM_TYPE_DUAL_BITBLT &&
		    w <= 3840 && !(w & 3)) {
			const struct img_image_buffer *in = &param->inputs[0].buffer;
			const struct img_image_buffer *out = &param->outputs[0].buffer;
			struct img_config *cfgs = (struct img_config *)vpu->config;
			u32 istride = in->format.plane_fmt[0].stride;
			u32 ostride = out->format.plane_fmt[0].stride;
			struct mdp_dual_wrot wr = {
				.iova = { out->iova[0], out->iova[1], 0 },
				.stride = { ostride, ostride, ostride },
				.control = 0x500c, .mat_ctrl = 0,
				.pre_ultra = 0x000d80c4, .main_buf = 0x4000,
				.ofmul = { 1, 1, 1 }, .ofshr = { 0, 0, 0 },
			};

			mdp_fill_dual_pipe_8195_linear(&cfgs[0].config_8195, 0, w, h,
				istride, in->iova[0], in->iova[1], 0x60c, &wr, NULL);
			mdp_fill_dual_pipe_8195_linear(&cfgs[1].config_8195, 1, w, h,
				istride, in->iova[0], in->iova[1], 0x60c, &wr, NULL);
			vpu->detile_nchunks = 1;
			dev_dbg(&mdp->pdev->dev,
				"kernel-direct dual blit config (mt8195, %ux%u)\n", w, h);
			return 0;
		}

		if (CFG_CHECK(MT8195, mdp->mdp_data->mdp_plat_id) &&
		    w <= 1920 && (unsigned long long)w * h < 2073600ULL) {
			vpu->detile_nchunks = 1;
			mdp_config_compute_8195_blit(param,
					(struct img_config *)vpu->config);
			dev_dbg(&mdp->pdev->dev,
				"kernel-direct blit config (mt8195, NV12->NV12)\n");
			return 0;
		}
		/* mt8183 NV12/NM12 -> NV12/NM12 (1:1 identity / single<->multi-planar
		 * repack): mt8183's SCP is dead (KI-006 -22), so this trivial 2-plane
		 * copy had no path. Route it through the 64x32-tiled WROT NV12 write
		 * (mdp_fmtconv_nv12) -- the SAME compute the GREY->NV12 src-expand uses,
		 * proven bit-exact for a 2-plane NV12 write. No CSC, no resample: the
		 * output is byte-identical to the input (identity). NM12 src/dst differ
		 * only in iova[1] (separate vs contiguous plane), which the compute reads
		 * from param, so all four NV12/NM12 combinations ride this one path. A
		 * grid > IMG_MAX_SUBFRAMES_8183 chunks like the detile (each tile
		 * independent). w,h <= 4096 (decoder range). */
		if (mdp_kd_8183_ident &&
		    CFG_CHECK(MT8183, mdp->mdp_data->mdp_plat_id) &&
		    w <= 4096 && h <= 4096) {
			u32 ntiles = DIV_ROUND_UP(w, 64) * DIV_ROUND_UP(h, 32);

			vpu->detile_ntiles = ntiles;
			vpu->detile_nchunks = DIV_ROUND_UP(ntiles, IMG_MAX_SUBFRAMES_8183);
			mdp_config_compute_8183_fmtconv(mdp, param,
				(struct img_config *)vpu->config, &mdp_fmtconv_nv12, 0,
				min_t(u32, IMG_MAX_SUBFRAMES_8183, ntiles));
			dev_dbg(&mdp->pdev->dev,
				"kernel-direct blit (mt8183, NV12/NM12->NV12/NM12, %u tiles/%u chunk(s))\n",
				ntiles, vpu->detile_nchunks);
			return 0;
		}
		dev_dbg(&mdp->pdev->dev,
			"blit %ux%u >1920 or dual or oversize; falling back to SCP\n",
			w, h);
	} else if (mdp_kernel_direct && mdp_frame_is_nv12_rotate90(param)) {
		/* Kernel-direct rotate-90 (NV12 -> NV12) -- frees rotate from the SCP
		 * (KI-009). One nsub=1 GCE submission, processing the input W x H; the
		 * WROT rotates onto the H x W output. Bound on the input (pipe) dims:
		 * <=1920 each, area < 2,073,600. The nsub=1 WROT-rotation form is RE'd
		 * at <=256; >=1280 may need tiling -- validate before widening.
		 * rotate-180/270 + flips still go through the SCP (own RE pending). */
		u32 iw = param->inputs[0].buffer.format.width;
		u32 ih = param->inputs[0].buffer.format.height;

		/* >1920 (DUAL_BITBLT) dim-swapping rot90: split the INPUT columns across
		 * two pixel pipes (pw=iw/2, like blit), each WROT writing a half-height
		 * output band. rot90 control 0x4010500c, base ostride-1 + pw*ostride.
		 * Byte-confirmed vs the v23042 SCP 2560x1440 pp0+pp1 oracle. */
		if (CFG_CHECK(MT8195, mdp->mdp_data->mdp_plat_id) &&
		    param->type == MDP_STREAM_TYPE_DUAL_BITBLT &&
		    iw <= 3840 && !(iw & 3) && !(ih & 1)) {
			mdp_fill_dual_rot90_8195((struct img_config *)vpu->config,
						 param, 0x4010500c, false);
			vpu->detile_nchunks = 1;
			dev_dbg(&mdp->pdev->dev,
				"kernel-direct dual rot90 (mt8195, %ux%u)\n", iw, ih);
			return 0;
		}

		if (CFG_CHECK(MT8195, mdp->mdp_data->mdp_plat_id) &&
		    iw <= 1920 && ih <= 1920 &&
		    (unsigned long long)iw * ih < 2073600ULL) {
			vpu->detile_nchunks = 1;
			mdp_config_compute_8195_rotate90(param,
					(struct img_config *)vpu->config);
			dev_dbg(&mdp->pdev->dev,
				"kernel-direct rotate90 config (mt8195, NV12->NV12)\n");
			return 0;
		}
		/* mt8183 rot90 HYBRID (HW GREY Y swap + CPU chroma dim-swap), bit-exact */
		if (mdp_try_hybrid_swap(mdp, vpu, param, iw, ih))
			return 0;
		if (CFG_CHECK(MT8183, mdp->mdp_data->mdp_plat_id) &&
		    iw <= 254 && ih <= 254) {
			/* mt8183 rotate-90 = the ROW_REV xform with control 0x4010500c.
			 * Full-frame nsub=1, gated <=254 (WROT 8-bit line-buffer cap). */
			vpu->detile_nchunks = 1;
			mdp_config_compute_8183_xform(mdp, param,
					(struct img_config *)vpu->config,
					0x4010500c, MDP_OFST_ROW_REV, false);
			dev_dbg(&mdp->pdev->dev,
				"kernel-direct rotate90 config (mt8183, NV12->NV12)\n");
			return 0;
		}
		dev_dbg(&mdp->pdev->dev,
			"mt8195 rotate90 %ux%u >1920 or dual; falling back to SCP\n",
			iw, ih);
	} else if (mdp_kernel_direct && mdp_frame_is_nv12_xform(param)) {
		/* Kernel-direct rotate-180/270 + hflip/vflip (NV12 -> NV12) -- frees the
		 * rest of the SCP-path transforms (KI-009). One nsub=1 GCE submission on
		 * the input W x H. Bound on the input (pipe) dims: <=1920 each, area <
		 * 2,073,600. The nsub=1 WROT-offset forms are RE'd at <=256; validate
		 * before widening (rotate-90 held to 1280, these likely do too). */
		u32 iw = param->inputs[0].buffer.format.width;
		u32 ih = param->inputs[0].buffer.format.height;
		u32 control;
		enum mdp_wrot_ofst mode;

		/* >1920 (DUAL_BITBLT): rot180 / hflip / vflip across two pixel pipes.
		 * The read side (c0-c8) is blit's linear dual verbatim; only the WROT
		 * c9 differs -- the per-mode control + the single-pipe base VIDO offset,
		 * with the pp1 offset REVERSED by pw for the H-mirroring modes (ROW_REV
		 * hflip / FULL_REV rot180: pp1 owns the LEFT output half). vflip
		 * (COL_REV) does not mirror columns, so pp1 = base + pw like blit. The
		 * dim-swapping rot90/270 are EXCLUDED here (out W == in W gate) and stay
		 * SCP. Byte-confirmed vs the v23042 SCP 2560x1088 pp0+pp1 oracle
		 * (mdp_kd_debug cfg8195_snapshot_pp0/_pp1). */
		if (CFG_CHECK(MT8195, mdp->mdp_data->mdp_plat_id) &&
		    param->type == MDP_STREAM_TYPE_DUAL_BITBLT &&
		    iw <= 3840 && !(iw & 3) &&
		    param->outputs[0].buffer.format.width == iw &&
		    param->outputs[0].buffer.format.height == ih &&
		    mdp_xform_variant(param, &control, &mode)) {
			const struct img_image_buffer *in = &param->inputs[0].buffer;
			const struct img_image_buffer *out = &param->outputs[0].buffer;
			struct img_config *cfgs = (struct img_config *)vpu->config;
			u32 istride = in->format.plane_fmt[0].stride;
			u32 ostride = out->format.plane_fmt[0].stride;
			u32 ysz = iw * ih, csz = ysz / 2;
			struct mdp_dual_wrot wr = {
				.iova = { out->iova[0], out->iova[1], 0 },
				.stride = { ostride, ostride, ostride },
				.control = control, .mat_ctrl = 0,
				.pre_ultra = 0x000d80c4, .main_buf = 0x4000,
				.ofmul = { 1, 1, 1 }, .ofshr = { 0, 0, 0 },
				.pp1_reverse = (mode != MDP_OFST_COL_REV),
			};

			switch (mode) {
			case MDP_OFST_ROW_REV:	/* hflip: reverse within a row */
				wr.base_ofst[0] = ostride - 1;
				wr.base_ofst[1] = ostride - 1;
				wr.base_ofst[2] = ostride - 1;
				break;
			case MDP_OFST_FULL_REV:	/* rot180: reverse rows AND cols */
				wr.base_ofst[0] = ysz - 1;
				wr.base_ofst[1] = csz - 1;
				wr.base_ofst[2] = csz - 1;
				break;
			default:		/* MDP_OFST_COL_REV: vflip (reverse rows) */
				wr.base_ofst[0] = ysz - ostride;
				wr.base_ofst[1] = csz - ostride;
				wr.base_ofst[2] = csz - ostride;
				break;
			}

			mdp_fill_dual_pipe_8195_linear(&cfgs[0].config_8195, 0, iw, ih,
				istride, in->iova[0], in->iova[1], 0x60c, &wr, NULL);
			mdp_fill_dual_pipe_8195_linear(&cfgs[1].config_8195, 1, iw, ih,
				istride, in->iova[0], in->iova[1], 0x60c, &wr, NULL);
			vpu->detile_nchunks = 1;
			dev_dbg(&mdp->pdev->dev,
				"kernel-direct dual xform (mt8195, %ux%u control=%#x)\n",
				iw, ih, control);
			return 0;
		}

		/* >1920 (DUAL_BITBLT) dim-swapping rot270 (out W == in H): the OTHER
		 * axis split, shared with rot90 (mdp_fill_dual_rot90_8195) -- rot270
		 * control 0x1030500c, base ysz-ostride / csz-ostride minus pw*ostride
		 * (reversed output band). vflip is COL_REV too but NOT dim-swapping
		 * (handled above). Byte-confirmed vs the v23042 SCP 2560x1440 oracle. */
		if (CFG_CHECK(MT8195, mdp->mdp_data->mdp_plat_id) &&
		    param->type == MDP_STREAM_TYPE_DUAL_BITBLT &&
		    iw <= 3840 && !(iw & 3) && !(ih & 1) &&
		    param->outputs[0].buffer.format.width == ih &&
		    param->outputs[0].buffer.format.height == iw &&
		    mdp_xform_variant(param, &control, &mode) &&
		    control == 0x1030500c) {	/* rot270 only (vflip excluded by the dim-swap gate) */
			mdp_fill_dual_rot90_8195((struct img_config *)vpu->config,
						 param, control, true);
			vpu->detile_nchunks = 1;
			dev_dbg(&mdp->pdev->dev,
				"kernel-direct dual rot270 (mt8195, %ux%u)\n", iw, ih);
			return 0;
		}

		if (CFG_CHECK(MT8195, mdp->mdp_data->mdp_plat_id) &&
		    iw <= 1920 && ih <= 1920 &&
		    (unsigned long long)iw * ih < 2073600ULL &&
		    mdp_xform_variant(param, &control, &mode)) {
			vpu->detile_nchunks = 1;
			mdp_config_compute_8195_xform(param,
					(struct img_config *)vpu->config, control, mode);
			dev_dbg(&mdp->pdev->dev,
				"kernel-direct xform config (mt8195, NV12->NV12, control=%#x)\n",
				control);
			return 0;
		}
		/* mt8183 rot270 HYBRID (HW GREY Y swap + CPU chroma dim-swap), bit-exact */
		if (mdp_try_hybrid_swap(mdp, vpu, param, iw, ih))
			return 0;
		if (CFG_CHECK(MT8183, mdp->mdp_data->mdp_plat_id)) {
			/* HYBRID NV12 rotate (no-swap, default-on): HW GREY Y + CPU chroma.
			 * Own size cap, wider than the <=254 SCP-fallback gate below -- the
			 * CPU chroma is size-agnostic; the HW GREY Y tiles vertically. The
			 * height bound is the GREY xform's nsub cap (IMG_MAX_SUBFRAMES_8183 *
			 * 32 = 448 rows for the default strip). Width bound TBD by HW test. */
			enum mdp_wrot_ofst hmode;
			u32 hctl;
			bool hrows, hcols;

			if (mdp_nv12_rotate_hybrid(param, &hmode, &hctl, &hrows, &hcols) &&
			    iw <= MDP_HYBRID_ROT_W_MAX && ih <= MDP_HYBRID_ROT_H_MAX) {
				u32 strip = mdp_kd_wrot_lines > 0 ? (u32)mdp_kd_wrot_lines : 32;
				u32 ntiles = DIV_ROUND_UP(iw, MDP_HYBRID_ROT_TILEW) *
					     DIV_ROUND_UP(ih, strip);
				u32 c0 = min_t(u32, ntiles, IMG_MAX_SUBFRAMES_8183);

				/* HW Y over a 2D tile grid (nh<=1536-wide cols x nv strips) across
				 * ceil(ntiles/14) GCE chunks; CPU chroma (size-free) runs once. */
				vpu->detile_ntiles = ntiles;
				vpu->detile_nchunks = DIV_ROUND_UP(ntiles, IMG_MAX_SUBFRAMES_8183);
				vpu->hybrid_chroma = true;
				vpu->hybrid_rev_rows = hrows;
				vpu->hybrid_rev_cols = hcols;
				mdp_config_compute_8183_yrot(mdp, param,
					(struct img_config *)vpu->config,
					hctl, hmode, 0, c0);	/* chunk 0 = tiles [0,c0) */
				dev_dbg(&mdp->pdev->dev,
					"kernel-direct HYBRID NV12 rotate (mt8183 %ux%u, HW Y %u chunk(s) + CPU chroma)\n",
					iw, ih, vpu->detile_nchunks);
				return 0;
			}
		}
		if (CFG_CHECK(MT8183, mdp->mdp_data->mdp_plat_id) &&
		    iw <= 254 && ih <= 254 &&
		    mdp_xform_variant(param, &control, &mode)) {
			enum mdp_wrot_ofst rmode;
			u32 yctl, uvctl, gctl;

			/* No-swap: scratch-backed 5-pass (de-interleave -> rot Y/Cb/Cr
			 * single-plane -> re-interleave) -- the only fully-clean chroma
			 * path. Allocs 2 I420 scratch buffers; chunk 0 = de-interleave. */
			if (mdp_nv12_rotate_5pass(param, &rmode, &gctl)) {
				size_t scsz = (size_t)iw * ih * 3 / 2;

				if (mdp_vpu_scratch_ensure(vpu, scsz)) {
					dev_warn(&mdp->pdev->dev,
						 "5-pass scratch alloc failed; SCP fallback\n");
				} else {
					vpu->detile_ntiles = (mdp_kd_5pass == 2) ? 2 : 5;
					vpu->detile_nchunks = (mdp_kd_5pass == 2) ? 2 : 5;
					mdp_vpu_compute_8183_chunk(vpu, param, 0);  /* de-interleave */
					dev_dbg(&mdp->pdev->dev,
						"kernel-direct 5-pass NV12 rotate (mt8183, gctl=%#x nchunks=%u)\n",
						gctl, vpu->detile_nchunks);
					return 0;
				}
			}

			/* No-swap (rot180/hflip/vflip): SEPARATE-PLANE 2-pass --
			 * chunk 0 = Y as GREY, chunk 1 = CbCr as 16bpp (bit-exact;
			 * dodges the WROT two-plane Y<->UV demux swap). */
			if (mdp_nv12_rotate_2pass(param, &rmode, &yctl, &uvctl)) {
				vpu->detile_ntiles = 2;
				vpu->detile_nchunks = 2;
				mdp_config_compute_8183_xform(mdp, param,
					(struct img_config *)vpu->config,
					yctl, rmode, true);	/* chunk 0 = Y */
				dev_dbg(&mdp->pdev->dev,
					"kernel-direct 2-pass NV12 rotate (mt8183, yctl=%#x uvctl=%#x)\n",
					yctl, uvctl);
				return 0;
			}
			/* swap (rot270): single-pass (geometry-correct, chroma-bleed WIP) */
			vpu->detile_nchunks = 1;
			mdp_config_compute_8183_xform(mdp, param,
					(struct img_config *)vpu->config, control, mode, false);
			dev_dbg(&mdp->pdev->dev,
				"kernel-direct xform config (mt8183, NV12->NV12, control=%#x)\n",
				control);
			return 0;
		}
		dev_dbg(&mdp->pdev->dev,
			"mt8195 xform %ux%u >1920 or dual; falling back to SCP\n",
			iw, ih);
	} else if (mdp_kernel_direct && mdp_grey_rotate_variant(param, &gctl, &gmode)) {
		/* NV12 -> GREY(Y8) rotate (mt8183): the single-output-plane proof of the
		 * separate-plane workaround -- 1 write plane => no Y<->UV demux swap. */
		u32 iw = param->inputs[0].buffer.format.width;
		u32 ih = param->inputs[0].buffer.format.height;

		if (CFG_CHECK(MT8183, mdp->mdp_data->mdp_plat_id) &&
		    iw <= 254 && ih <= 254) {
			vpu->detile_nchunks = 1;
			mdp_config_compute_8183_xform(mdp, param,
					(struct img_config *)vpu->config, gctl, gmode, true);
			dev_dbg(&mdp->pdev->dev,
				"kernel-direct GREY rotate (mt8183, NV12->Y8, control=%#x)\n",
				gctl);
			return 0;
		}
	} else if (mdp_kernel_direct && mdp_nv12_to_i420_rotate(param, &gmode, &gctl)) {
		/* NV12 -> I420 rotate (mt8183): de-interleave keystone -- the WROT writes
		 * 3 separate planes (Y/U/V) rotated. Tests if a de-interleaved write dodges
		 * the 2-plane demux swap. */
		u32 iw = param->inputs[0].buffer.format.width;
		u32 ih = param->inputs[0].buffer.format.height;

		if (CFG_CHECK(MT8183, mdp->mdp_data->mdp_plat_id) &&
		    iw <= 254 && ih <= 254) {
			vpu->detile_nchunks = 1;
			mdp_config_compute_8183_i420rot(mdp, param,
					(struct img_config *)vpu->config, gctl, gmode);
			dev_dbg(&mdp->pdev->dev,
				"kernel-direct NV12->I420 rotate (mt8183, control=%#x)\n", gctl);
			return 0;
		}
	} else if (mdp_kernel_direct && mdp_nv12_to_i420_deint(param)) {
		/* Isolation: NV12 -> I420 forward de-interleave to the OUTPUT dma-buf. */
		const struct img_image_buffer *in = &param->inputs[0].buffer;
		const struct img_image_buffer *out = &param->outputs[0].buffer;
		u32 iw = in->format.width, ih = in->format.height;

		if (CFG_CHECK(MT8183, mdp->mdp_data->mdp_plat_id) &&
		    iw <= 254 && ih <= 254) {
			u32 istride = in->format.plane_fmt[0].stride;
			u32 ostride = out->format.plane_fmt[0].stride;

			vpu->detile_nchunks = 1;
			mdp_config_compute_8183_cvt(mdp, (struct img_config *)vpu->config,
				in->iova[0], in->iova[1], 0, 0x060c, istride, istride,
				out->iova[0], out->iova[1], out->iova[2], 0x00005008,
				ostride, ostride / 2, iw, ih);
			dev_dbg(&mdp->pdev->dev,
				"kernel-direct NV12->I420 de-interleave to dma-buf (mt8183, isolation)\n");
			return 0;
		}
	} else if (mdp_kernel_direct && mdp_frame_is_nv12_fmtconv(param)) {
		/* Kernel-direct NV12 -> <fmt> 1:1 format conversion -- frees the whole
		 * convert matrix (NV21/I420/YV12/GREY/4:2:2 semi+packed/RGB565/BGR24/
		 * 32-bit) from the SCP (KI-009). One nsub=1 GCE submission; the WROT
		 * does the format conversion (table-driven). mt8195 single-pipe <=1920. */
		u32 w = param->outputs[0].buffer.format.width;
		u32 h = param->outputs[0].buffer.format.height;
		const struct mdp_fmtconv *f = mdp_fmtconv_for(param);

		/* mt8183 swapped-src -> NV21/NM21: swap_twin(NV21)==NV12 is not a table
		 * row, so mdp_fmtconv_for returns NULL. Supply the local NV12 row (WROT
		 * 0x500c, NO 0x100 swap) so the byte-swapped pipe chroma writes straight
		 * through as NV21 order (mdp_kd_8183_swapdst). mt8183-only: mt8195 keeps
		 * f==NULL here and falls to its healthy SCP -> no mt8195 change. */
		if (!f && mdp_kd_8183_swapdst &&
		    CFG_CHECK(MT8183, mdp->mdp_data->mdp_plat_id) &&
		    mdp_color_is_plain_nv21(param->outputs[0].buffer.format.colorformat))
			f = &mdp_fmtconv_nv12;

		/* SCP-tail class #1: the mt8195 twin. swap_twin(NV21)==NV12 is not a
		 * table row -> mdp_fmtconv_for returned NULL and a swapped-2-plane
		 * source -> NV21/NM21 dst (NV21->NV21, NV61->NV21, NV42->NV21, ...) rode
		 * the SCP. The mt8195 single-pipe fmtconv compute reads the swapped
		 * source bit-exact; the local NV12 row (WROT 0x500c, no 0x100 swap)
		 * writes the byte-swapped pipe chroma straight through as NV21 order.
		 * Same substitution as mt8183, gated by mdp_kd_swap1to1. */
		if (!f && mdp_kd_swap1to1 &&
		    CFG_CHECK(MT8195, mdp->mdp_data->mdp_plat_id) &&
		    mdp_color_is_plain_nv21(param->outputs[0].buffer.format.colorformat))
			f = &mdp_fmtconv_nv12;

		/*
		 * mt8195 single-plane SOURCE (packed RGB / packed 4:2:2) -> 4:2:2-
		 * semiplanar dst (NV16/NV61/NM16/NM61). mdp_config_compute_8195_fmtconv
		 * now does a source-derived single packed-plane RDMA read (no phantom
		 * plane-1 read -> no SVPP2 region-2 IOMMU fault) + the RGB->YUV CSC
		 * (RDMA colorTran + WROT mat_ctrl 0x81), mirroring the v23042 SCP config
		 * (src_ctrl=0x4601 etc.). The output row is the PLAIN dst row
		 * (mdp_fmtconv_lookup, NOT the swap-twin): a swapped source (YVYU/VYUY/
		 * BGR/ABGR) is un-swapped by the RDMA sourceSwap bit (0x4000), so the
		 * WROT writes plain -- exactly as the SCP does. This lifts the 09834f3
		 * exclusion for these pairs (bit-exact vs SCP, board-validated).
		 * (SVPP2 region-2 IOMMU-fault audit.)
		 */
		/*
		 * A packed-4:2:2 SOURCE now targets ANY non-3-plane dst (not just the
		 * 4:2:2-semi ones above): the single_plane RDMA read is dst-INDEPENDENT
		 * (src_ctrl 0x600|swap|HW_FORMAT), and the SCP pairs it with the PLAIN
		 * dst-row WROT (lookup(dst); NV12 via the local mdp_fmtconv_nv12 row) --
		 * a swapped source (YVYU/VYUY) is un-swapped by the RDMA 0x4000 bit, so
		 * the WROT writes plain, NEVER the swap-twin (mdp_fmtconv_for would pick
		 * the swap-twin row here and double-swap). The 4:2:2->4:2:0 vertical
		 * decimate for an NV12/NV21 dst is the WROT control (0x500c/0x510c),
		 * exactly as NV16->NV12. Oracle-confirmed (v23042 cfg8195): YUYV>NV12
		 * c0=0x605 WROT=0x500c, YVYU>NV12 c0=0x4605 WROT=0x500c (plain!),
		 * YUYV>RGB3 WROT mat=0x61. 3-plane I420/YV12 dst -> the hybrid arms.
		 *
		 * An RGB SOURCE joins the SAME arm for every non-3-plane dst: its read
		 * is the single packed plane + the RDMA RGB->YUV colorTran (u[14]), and
		 * the WROT does RGB->YUV (mat 0x81) for a YUV dst or, for an RGB dst
		 * (RGB->RGB repack), the mat_ctrl 0x41 the compute now emits (v23042
		 * oracle: RGB3>BGR3 WROT mat=0x41 vs RGB3>NV12/GREY mat=0x81). RGB
		 * un-swap (0x4000) is handled by the same dst-independent single_plane
		 * src_ctrl (RGB3=0x4601, BGR3=0x601, AR24=0x602).
		 */
		if (CFG_CHECK(MT8195, mdp->mdp_data->mdp_plat_id) &&
		    (mdp_color_is_rgb_src(param->inputs[0].buffer.format.colorformat) ||
		     mdp_color_is_packed422src(param->inputs[0].buffer.format.colorformat)) &&
		    (mdp_color_is_422semi_dst(param->outputs[0].buffer.format.colorformat) ||
		     MDP_COLOR_GET_PLANE_COUNT(param->outputs[0].buffer.format.colorformat) != 3) &&
		    w <= 1920 && (unsigned long long)w * h < 2073600ULL) {
			u32 od = param->outputs[0].buffer.format.colorformat;
			const struct mdp_fmtconv *fp = mdp_color_is_plain_nv12(od) ?
				&mdp_fmtconv_nv12 : mdp_fmtconv_lookup(od);

			if (fp) {
				vpu->detile_nchunks = 1;
				mdp_config_compute_8195_fmtconv(param,
					(struct img_config *)vpu->config, fp);
				dev_dbg(&mdp->pdev->dev,
					"kernel-direct single-plane-src fmtconv (mt8195, %#x->%#x)\n",
					param->inputs[0].buffer.format.colorformat,
					fp->color);
				return 0;
			}
		}

		/*
		 * The mt8195/mt8188 compute (mdp_config_compute_8195_fmtconv) reads a
		 * 2-plane 4:2:0 NV12-family SOURCE (case 0 else-branch: iova[1]=ic,
		 * iova_end[1]=ic+istride*h/2, src_ctrl=f->src_ctrl). A single-Y-plane
		 * source (RGB / packed 4:2:2) has inputs[0].iova[1]==0; reading it as
		 * 2-plane would program a phantom plane-1 read at iova 0 -> the SVPP2
		 * region-2 IOMMU fault (09834f3). Those sources -> 4:2:2-semiplanar dst
		 * are now handled by the single-plane arm ABOVE; the remaining RGB /
		 * packed-4:2:2 sources (-> NV12/NV21/GREY/RGB/... dsts, which still need
		 * a validated KD chroma-resample/CSC path) stay excluded here and ride
		 * the healthy SCP.
		 * See the SVPP2 region-2 IOMMU-fault audit.
		 *
		 * NV16/NV61 (2-plane 4:2:2) SOURCES are read by the case 0 nv16_src branch
		 * (full-height chroma + src_ctrl 0x60d); YU12/YV12 (3-plane 4:2:0 I420)
		 * SOURCES by the case 0 i420_src branch (3-plane read, iova[2]/iova_end[2] in
		 * u[5]/u[8], src_ctrl (f->src_ctrl & 0x600)|swap|0x08) -- both admitted,
		 * board bit-exact vs the v23042 SCP oracle (KI-012 fixed). The remaining
		 * excluded SOURCES are RGB / packed-4:2:2 -> the NON-4:2:2-semi dsts (their
		 * 4:2:2->4:2:0 vertical decimate / RGB CSC to those dsts still needs a
		 * validated KD path); RGB/packed -> 4:2:2-semi is the single-plane arm above.
		 */
		/* >1920 (DUAL_BITBLT): NV12 SOURCE -> a 2-plane semiplanar 4:2:0/4:2:2 dst
		 * (NV21 twin here; 3-plane I420/YV12 dsts are gated out by
		 * mdp_frame_is_nv12_fmtconv above and stay >1920->SCP). Same linear
		 * primitive; the WROT format (control/pre_ultra) comes from the fmtconv
		 * row, per-half write offset {outx, outx, -} (interleaved chroma, 1 B/col).
		 * Byte-confirmed vs the v23042 SCP 2560x1088 pp1 oracle (NV21 WROT 0x510c,
		 * u[18..20]=outx). */
		if (f && CFG_CHECK(MT8195, mdp->mdp_data->mdp_plat_id) &&
		    param->type == MDP_STREAM_TYPE_DUAL_BITBLT && w <= 3840 && !(w & 3) &&
		    mdp_color_is_plain_nv12(param->inputs[0].buffer.format.colorformat) &&
		    mdp_color_is_plain_nv21(param->outputs[0].buffer.format.colorformat)) {
			const struct img_image_buffer *in = &param->inputs[0].buffer;
			const struct img_image_buffer *out = &param->outputs[0].buffer;
			struct img_config *cfgs = (struct img_config *)vpu->config;
			struct mdp_dual_wrot wr = {
				.iova = { out->iova[0], out->iova[1], 0 },
				.stride = { out->format.plane_fmt[0].stride,
					    out->format.plane_fmt[1].stride, 0 },
				.control = f->control, .mat_ctrl = 0,
				.pre_ultra = f->pre_ultra, .main_buf = 0x4000,
				.ofmul = { 1, 1, 1 }, .ofshr = { 0, 0, 0 },
			};
			u32 istride = in->format.plane_fmt[0].stride;

			mdp_fill_dual_pipe_8195_linear(&cfgs[0].config_8195, 0, w, h,
				istride, in->iova[0], in->iova[1], f->src_ctrl, &wr, NULL);
			mdp_fill_dual_pipe_8195_linear(&cfgs[1].config_8195, 1, w, h,
				istride, in->iova[0], in->iova[1], f->src_ctrl, &wr, NULL);
			vpu->detile_nchunks = 1;
			dev_dbg(&mdp->pdev->dev,
				"kernel-direct dual fmtconv (mt8195, %ux%u NV12->NV21)\n", w, h);
			return 0;
		}

		/* >1920 (DUAL_BITBLT): 2-plane 4:2:2 SOURCE (NV16/NV61/NM16/NM61) -> a
		 * 2-plane semiplanar YUV dst (NV12/NV21/NV16/NV61 + NM twins). Same
		 * linear primitive as the NV12-source dual; the ONLY source-read delta is
		 * the FULL-height plane-1 chroma (mdp_dual_src.chroma_full_h -> u[7]=
		 * ic+istride*h) and src_ctrl 0x60d (0x600 | HW_FORMAT(13)) -- transcribed
		 * from the <=1920 nv16_src compute (case 0). A swapped NV61 rides the
		 * swap-twin WROT row (f = mdp_fmtconv_for) exactly as at <=1920; the
		 * interleaved-chroma column read offset stays rx (unchanged u[25..27]).
		 * The WROT write is the proven 2-plane form (per-half offset {outx,outx,-},
		 * ofmul {1,1,1}, main_buf 0x4000, mat_ctrl 0). RGB/packed/3-plane dsts
		 * (different WROT offset/plane count) are excluded and ride the SCP.
		 * Validated md5(KD)==md5(SCP) at 1984/2560/3840 + pp0/pp1 cfg8195 byte-diff
		 * vs the v23042 oracle. nonnv12-source-kd-spec-mt8195.md. */
		if (f && CFG_CHECK(MT8195, mdp->mdp_data->mdp_plat_id) &&
		    param->type == MDP_STREAM_TYPE_DUAL_BITBLT && w <= 3840 && !(w & 3) &&
		    mdp_color_is_plain_nv16src(param->inputs[0].buffer.format.colorformat) &&
		    MDP_COLOR_GET_PLANE_COUNT(param->outputs[0].buffer.format.colorformat) == 2 &&
		    !f->rgb) {
			const struct img_image_buffer *in = &param->inputs[0].buffer;
			const struct img_image_buffer *out = &param->outputs[0].buffer;
			u32 in_color = in->format.colorformat;
			struct img_config *cfgs = (struct img_config *)vpu->config;
			u32 istride = in->format.plane_fmt[0].stride;
			u32 src_ctrl = 0x600 | MDP_COLOR_GET_HW_FORMAT(in_color);
			struct mdp_dual_src src = { .chroma_full_h = true };
			struct mdp_dual_wrot wr = {
				.iova = { out->iova[0], out->iova[1], 0 },
				.stride = { out->format.plane_fmt[0].stride,
					    out->format.plane_fmt[1].stride, 0 },
				.control = f->control, .mat_ctrl = 0,
				.pre_ultra = f->pre_ultra, .main_buf = 0x4000,
				.ofmul = { 1, 1, 1 }, .ofshr = { 0, 0, 0 },
			};

			mdp_fill_dual_pipe_8195_linear(&cfgs[0].config_8195, 0, w, h,
				istride, in->iova[0], in->iova[1], src_ctrl, &wr, &src);
			mdp_fill_dual_pipe_8195_linear(&cfgs[1].config_8195, 1, w, h,
				istride, in->iova[0], in->iova[1], src_ctrl, &wr, &src);
			vpu->detile_nchunks = 1;
			dev_dbg(&mdp->pdev->dev,
				"kernel-direct dual fmtconv (mt8195, %ux%u NV16src->%#x)\n",
				w, h, out->format.colorformat);
			return 0;
		}

		/* >1920 (DUAL_BITBLT): packed-4:2:2 SOURCE (YUYV/YVYU/UYVY/VYUY) -> a
		 * 2-plane semiplanar YUV dst. ONE packed plane, bpp=2, no plane-1 read
		 * (mdp_dual_src.single_plane); the per-half column read offset is rx*2
		 * bytes. src_ctrl 0x600|swap|HW_FORMAT (YUYV 0x605, YVYU 0x4605 -- the
		 * 0x4000 un-swaps a V-first source at the RDMA, so the WROT writes the
		 * PLAIN dst row, never the swap-twin). The 4:2:2->4:2:0 vertical decimate
		 * for an NV12/NV21 dst is the WROT control (0x500c/0x510c), exactly as
		 * NV16->NV12. Transcribed from the <=1920 single_plane compute (case 0) +
		 * the plain-dst-row WROT of the <=1920 arm above; pp1 read offset
		 * oracle-confirmed vs the v23042 SCP cfg8195 snapshot. RGB/GREY/packed/
		 * 3-plane dsts (plane_count != 2) -> SCP. */
		if (CFG_CHECK(MT8195, mdp->mdp_data->mdp_plat_id) &&
		    param->type == MDP_STREAM_TYPE_DUAL_BITBLT && w <= 3840 && !(w & 3) &&
		    mdp_color_is_packed422src(param->inputs[0].buffer.format.colorformat) &&
		    MDP_COLOR_GET_PLANE_COUNT(param->outputs[0].buffer.format.colorformat) == 2) {
			const struct img_image_buffer *in = &param->inputs[0].buffer;
			const struct img_image_buffer *out = &param->outputs[0].buffer;
			u32 in_color = in->format.colorformat;
			u32 od = out->format.colorformat;
			const struct mdp_fmtconv *fp = mdp_color_is_plain_nv12(od) ?
				&mdp_fmtconv_nv12 : mdp_fmtconv_lookup(od);

			if (fp) {
				struct img_config *cfgs = (struct img_config *)vpu->config;
				u32 istride = in->format.plane_fmt[0].stride;
				u32 src_ctrl = 0x600 |
					(MDP_COLOR_IS_SWAPPED(in_color) ? 0x4000 : 0) |
					MDP_COLOR_GET_HW_FORMAT(in_color);
				struct mdp_dual_src src = { .single_plane = true, .xbpp = 2 };
				struct mdp_dual_wrot wr = {
					.iova = { out->iova[0], out->iova[1], 0 },
					.stride = { out->format.plane_fmt[0].stride,
						    out->format.plane_fmt[1].stride, 0 },
					.control = fp->control, .mat_ctrl = 0,
					.pre_ultra = fp->pre_ultra, .main_buf = 0x4000,
					.ofmul = { 1, 1, 1 }, .ofshr = { 0, 0, 0 },
				};

				mdp_fill_dual_pipe_8195_linear(&cfgs[0].config_8195, 0, w, h,
					istride, in->iova[0], 0, src_ctrl, &wr, &src);
				mdp_fill_dual_pipe_8195_linear(&cfgs[1].config_8195, 1, w, h,
					istride, in->iova[0], 0, src_ctrl, &wr, &src);
				vpu->detile_nchunks = 1;
				dev_dbg(&mdp->pdev->dev,
					"kernel-direct dual fmtconv (mt8195, %ux%u packed422src->%#x)\n",
					w, h, od);
				return 0;
			}
		}

		/* >1920 (DUAL_BITBLT): 3-plane SOURCE -- I420/YV12 (4:2:0) or YM16/YM61
		 * (4:2:2) -- to a 2-plane semiplanar YUV dst. three_plane read: U=iova1,
		 * V=iova2, chroma pitch istride/2, per-half column offset rx/2 (4:2:0
		 * chroma spans h/2; 4:2:2 spans h via chroma_full_h). src_ctrl per family
		 * -- I420 (fp->src_ctrl&0x600)|swap|0x08, YM16 0x600|swap|0x09 -- the 0x4000
		 * un-swaps YV12/YM61's V-first order at the RDMA so the WROT writes the
		 * PLAIN dst row. Oracle-confirmed (YU12>NV12 c0=0x608 u25=rx u26/27=rx/2;
		 * YV12 c0=0x4608). RGB / 3-plane dst -> SCP. */
		if (CFG_CHECK(MT8195, mdp->mdp_data->mdp_plat_id) &&
		    param->type == MDP_STREAM_TYPE_DUAL_BITBLT && w <= 3840 && !(w & 3) &&
		    (mdp_color_is_plain_i420src(param->inputs[0].buffer.format.colorformat) ||
		     mdp_color_is_ym16src(param->inputs[0].buffer.format.colorformat)) &&
		    MDP_COLOR_GET_PLANE_COUNT(param->outputs[0].buffer.format.colorformat) == 2) {
			const struct img_image_buffer *in = &param->inputs[0].buffer;
			const struct img_image_buffer *out = &param->outputs[0].buffer;
			u32 in_color = in->format.colorformat;
			u32 od = out->format.colorformat;
			bool is422 = mdp_color_is_ym16src(in_color);
			const struct mdp_fmtconv *fp = mdp_color_is_plain_nv12(od) ?
				&mdp_fmtconv_nv12 : mdp_fmtconv_lookup(od);

			if (fp) {
				struct img_config *cfgs = (struct img_config *)vpu->config;
				u32 istride = in->format.plane_fmt[0].stride;
				u32 sw = MDP_COLOR_IS_SWAPPED(in_color) ? 0x4000 : 0;
				u32 src_ctrl = is422 ? (0x600 | sw | 0x09) :
					       ((fp->src_ctrl & 0x600) | sw | 0x08);
				struct mdp_dual_src src = {
					.three_plane = true, .chroma_full_h = is422,
					.iova2 = in->iova[2],
				};
				struct mdp_dual_wrot wr;

				mdp_dual_wrot_for_dst(&wr, out, fp, false);
				mdp_fill_dual_pipe_8195_linear(&cfgs[0].config_8195, 0, w, h,
					istride, in->iova[0], in->iova[1], src_ctrl, &wr, &src);
				mdp_fill_dual_pipe_8195_linear(&cfgs[1].config_8195, 1, w, h,
					istride, in->iova[0], in->iova[1], src_ctrl, &wr, &src);
				vpu->detile_nchunks = 1;
				dev_dbg(&mdp->pdev->dev,
					"kernel-direct dual fmtconv (mt8195, %ux%u 3plane-src->%#x)\n",
					w, h, od);
				return 0;
			}
		}

		/* >1920 (DUAL_BITBLT): RGB SOURCE (RGB565/BGR565/RGB24/BGR24/ARGB/ABGR) ->
		 * a 2-plane semiplanar YUV dst or a 1-plane RGB dst. single_plane read +
		 * RDMA read-CSC (colortran, u[14]=0x10000); per-half column offset rx*bpp
		 * (RGB24 rx*3, RGB565 rx*2, ARGB rx*4). src_ctrl 0x600|swap|HW_FORMAT
		 * (RGB3 0x4601). WROT: RGB->YUV (mat 0x81) for a YUV dst, RGB->RGB repack
		 * (mat 0x41) for an RGB dst; the source swap is un-done at the RDMA so the
		 * WROT writes the PLAIN dst row. Oracle-confirmed (RGB3>NV12 c0=0x4601
		 * u14=0x10000 u25=rx*3 WROT mat=0x81; RGB3>RGB3 mat=0x41). 3-plane dst -> SCP. */
		if (CFG_CHECK(MT8195, mdp->mdp_data->mdp_plat_id) &&
		    param->type == MDP_STREAM_TYPE_DUAL_BITBLT && w <= 3840 && !(w & 3) &&
		    mdp_color_is_rgb_src(param->inputs[0].buffer.format.colorformat) &&
		    MDP_COLOR_GET_PLANE_COUNT(param->outputs[0].buffer.format.colorformat) != 3) {
			const struct img_image_buffer *in = &param->inputs[0].buffer;
			const struct img_image_buffer *out = &param->outputs[0].buffer;
			u32 in_color = in->format.colorformat;
			u32 od = out->format.colorformat;
			const struct mdp_fmtconv *fp = mdp_color_is_plain_nv12(od) ?
				&mdp_fmtconv_nv12 : mdp_fmtconv_lookup(od);

			if (fp) {
				struct img_config *cfgs = (struct img_config *)vpu->config;
				u32 iw = in->format.width;
				u32 istride = in->format.plane_fmt[0].stride;
				u32 src_ctrl = 0x600 |
					(MDP_COLOR_IS_SWAPPED(in_color) ? 0x4000 : 0) |
					MDP_COLOR_GET_HW_FORMAT(in_color);
				struct mdp_dual_src src = {
					.single_plane = true, .colortran = true,
					.xbpp = iw ? istride / iw : 1,
				};
				struct mdp_dual_wrot wr;

				mdp_dual_wrot_for_dst(&wr, out, fp, true);
				mdp_fill_dual_pipe_8195_linear(&cfgs[0].config_8195, 0, w, h,
					istride, in->iova[0], 0, src_ctrl, &wr, &src);
				mdp_fill_dual_pipe_8195_linear(&cfgs[1].config_8195, 1, w, h,
					istride, in->iova[0], 0, src_ctrl, &wr, &src);
				vpu->detile_nchunks = 1;
				dev_dbg(&mdp->pdev->dev,
					"kernel-direct dual fmtconv (mt8195, %ux%u rgbsrc->%#x)\n",
					w, h, od);
				return 0;
			}
		}

		/* >1920 (DUAL_BITBLT): NV16/NV61 (2-plane 4:2:2) SOURCE -> a 1-plane RGB or
		 * GREY dst (the non-2-plane companions of the committed NV16->2-plane arm).
		 * chroma_full_h read (src_ctrl 0x60d; +0x4000 to un-swap a swapped NV61 for
		 * an RGB dst so the YUV->RGB CSC sees true U,V -- oracle NV61>RGB3 c0=0x460d
		 * vs NV16=0x60d). WROT: YUV->RGB (mat 0x61) for RGB, plain GREY (mat 0).
		 * Oracle-confirmed (NV16>RGB3 WROT mat=0x61 u18=outx*3; NV16>GREY
		 * control=0x5007 u18=outx). 2-plane dst -> the arm above; 3-plane / packed
		 * dst (swap-twin) -> SCP. */
		if (CFG_CHECK(MT8195, mdp->mdp_data->mdp_plat_id) &&
		    param->type == MDP_STREAM_TYPE_DUAL_BITBLT && w <= 3840 && !(w & 3) &&
		    mdp_color_is_plain_nv16src(param->inputs[0].buffer.format.colorformat) &&
		    MDP_COLOR_GET_PLANE_COUNT(param->outputs[0].buffer.format.colorformat) == 1) {
			const struct img_image_buffer *in = &param->inputs[0].buffer;
			const struct img_image_buffer *out = &param->outputs[0].buffer;
			u32 in_color = in->format.colorformat;
			u32 od = out->format.colorformat;
			const struct mdp_fmtconv *fp = mdp_fmtconv_lookup(od);

			if (fp && (fp->rgb || od == MDP_COLOR_GREY)) {
				struct img_config *cfgs = (struct img_config *)vpu->config;
				u32 istride = in->format.plane_fmt[0].stride;
				u32 src_ctrl = 0x600 | MDP_COLOR_GET_HW_FORMAT(in_color);
				struct mdp_dual_src src = { .chroma_full_h = true };
				struct mdp_dual_wrot wr;

				if (fp->rgb && MDP_COLOR_IS_SWAPPED(in_color))
					src_ctrl |= 0x4000;
				mdp_dual_wrot_for_dst(&wr, out, fp, false);
				mdp_fill_dual_pipe_8195_linear(&cfgs[0].config_8195, 0, w, h,
					istride, in->iova[0], in->iova[1], src_ctrl, &wr, &src);
				mdp_fill_dual_pipe_8195_linear(&cfgs[1].config_8195, 1, w, h,
					istride, in->iova[0], in->iova[1], src_ctrl, &wr, &src);
				vpu->detile_nchunks = 1;
				dev_dbg(&mdp->pdev->dev,
					"kernel-direct dual fmtconv (mt8195, %ux%u NV16src->%#x 1plane)\n",
					w, h, od);
				return 0;
			}
		}

		/*
		 * >1920 (DUAL_BITBLT): the GENERIC catch-all dual fmtconv -- the dual
		 * twin of the <=1920 catch-all below (mdp_config_compute_8195_fmtconv).
		 * Placed AFTER the specifically-ported dual arms above (which keep
		 * precedence for their classes -- lowest regression risk), this claims
		 * EVERY remaining non-3-plane-dst fmtconv pair mdp_frame_is_nv12_fmtconv
		 * admits, deriving the per-family source read from the shared
		 * mdp_dual_src_for() (identical classification to the <=1920 compute
		 * case 0) and the WROT from mdp_fmtconv_for() + mdp_dual_wrot_for_dst().
		 * Closes the wide fmtconv tail the 9 hard-gated arms miss: NV21/NM21
		 * source, NV12-source -> {packed, GREY, RGB}, packed/NV16/i420/ym16
		 * source -> {packed, GREY, RGB}, and the 4:2:0->4:2:2 vertical-UPSAMPLE
		 * dsts (NV12/I420-src -> NV16/NV61/packed): the WROT upsample is entirely
		 * in mdp_fmtconv_for()'s control word (uv_ysel, the 0x8000 bit of the
		 * NV16/NV61/packed rows), which is orthogonal to the horizontal pw=w/2
		 * split, so passing that row's control to mdp_dual_wrot_for_dst upsamples
		 * correctly over both pipes -- NOT a plain-NV12 substitute (0x500c, which
		 * would vertically DECIMATE).  MM21/MT21-tiled sources need the
		 * block-detile dual read (mdp_fill_dual_pipe_8195) and stay excluded.
		 * 3-plane I420/YV12/YM12/YM21 dsts are handled by the dedicated
		 * i420_deint / ym16_deint hybrid dual arms (mdp_frame_is_i420dst_*), not
		 * here.  Validated md5(KD)==md5(SCP) at 1984/2560/3840; cfg8195 pp0/pp1
		 * byte-diff vs the v23042 oracle. plan/dual-pipe-kd-spec-mt8195.md.
		 */
		if (mdp_kd_genfmt && f &&
		    CFG_CHECK(MT8195, mdp->mdp_data->mdp_plat_id) &&
		    param->type == MDP_STREAM_TYPE_DUAL_BITBLT && w <= 3840 && !(w & 3) &&
		    !mdp_color_is_mm21src(param->inputs[0].buffer.format.colorformat)) {
			mdp_fill_dual_fmtconv_8195(vpu, param, f);
			dev_dbg(&mdp->pdev->dev,
				"kernel-direct GENERIC dual fmtconv (mt8195, %ux%u %#x->%#x)\n",
				w, h, param->inputs[0].buffer.format.colorformat,
				param->outputs[0].buffer.format.colorformat);
			return 0;
		}

		/*
		 * >1920 (DUAL_BITBLT): MM21/MT21 tiled SOURCE -> a non-3-plane dst
		 * (non-NV12 2-plane / packed / RGB / GREY). The genfmt arm above uses the
		 * LINEAR dual read and EXCLUDES MM21 (it cannot detile); this twin uses the
		 * BLOCK-DETILE dual read (mdp_fill_dual_pipe_8195, ta*32 block addressing)
		 * feeding the SAME parameterized WROT (mdp_dual_wrot_for_dst) -- one-pass
		 * detile-read + fmtconv-write, the >1920 twin of the <=1920 mm21conv path
		 * (mdp_frame_is_nv12_fmtconv already admitted this MM21 source). The WROT row
		 * f = mdp_fmtconv_for(param): MM21 is 4:2:0 like NV12, so the plain output
		 * row's control is correct; a 4:2:0->4:2:2 upsample dst rides the row's
		 * uv_ysel bit (orthogonal to the pw=w/2 split). 3-plane I420/YV12/YM12/YM21
		 * dsts -> the i420dst hybrid arms. mm21conv is enforced by the outer gate. */
		if (mdp_kd_genmm21 && f &&
		    CFG_CHECK(MT8195, mdp->mdp_data->mdp_plat_id) &&
		    param->type == MDP_STREAM_TYPE_DUAL_BITBLT && w <= 3840 && !(w & 3) &&
		    mdp_mm21_dual_kd_exact(param) &&
		    mdp_color_is_mm21src(param->inputs[0].buffer.format.colorformat)) {
			mdp_fill_dual_mm21conv_8195(vpu, param, f);
			dev_dbg(&mdp->pdev->dev,
				"kernel-direct MM21 dual fmtconv (mt8195, %ux%u %#x->%#x)\n",
				w, h, param->inputs[0].buffer.format.colorformat,
				param->outputs[0].buffer.format.colorformat);
			return 0;
		}

		if (f && CFG_CHECK(MT8195, mdp->mdp_data->mdp_plat_id) &&
		    !mdp_color_is_rgb_src(param->inputs[0].buffer.format.colorformat) &&
		    param->inputs[0].buffer.format.colorformat != MDP_COLOR_BGR565 &&
		    !mdp_color_is_packed422src(param->inputs[0].buffer.format.colorformat) &&
		    w <= 1920 && (unsigned long long)w * h < 2073600ULL) {
			vpu->detile_nchunks = 1;
			mdp_config_compute_8195_fmtconv(param,
					(struct img_config *)vpu->config, f);
			dev_dbg(&mdp->pdev->dev,
				"kernel-direct fmtconv config (mt8195, NV12->%#x)\n",
				f->color);
			return 0;
		}
		/* mt8183: NV12->GREY (Y8, 1-plane) via cvt's full-width pass (validated).
		 * cvt reads a CONTIGUOUS Y plane (src_ctrl 0xc), so a packed source (which
		 * has no separate Y plane -- Y is interleaved with chroma) must NOT use it:
		 * route packed->GREY to the fmtconv tile path below (1-plane output), which
		 * reads the source packed and lets the WROT extract Y. */
		if (f && CFG_CHECK(MT8183, mdp->mdp_data->mdp_plat_id) &&
		    f->color == MDP_COLOR_GREY &&
		    !(mdp_kd_src_yuyv &&
		      mdp_color_is_packed422src(param->inputs[0].buffer.format.colorformat)) &&
		    !(mdp_kd_src_i420 &&
		      mdp_color_is_plain_i420src(param->inputs[0].buffer.format.colorformat)) &&
		    !(mdp_kd_src_rgb &&
		      mdp_color_is_rgb_src(param->inputs[0].buffer.format.colorformat)) &&
		    !(mdp_kd_mm21conv &&
		      mdp_color_is_mm21src(param->inputs[0].buffer.format.colorformat)) &&
		    w <= 1920 && h <= 1920) {
			const struct img_image_buffer *in = &param->inputs[0].buffer;
			const struct img_image_buffer *out = &param->outputs[0].buffer;
			u32 istride = in->format.plane_fmt[0].stride;
			u32 ostride = out->format.plane_fmt[0].stride;

			vpu->detile_nchunks = 1;
			mdp_config_compute_8183_cvt(mdp,
				(struct img_config *)vpu->config,
				in->iova[0], in->iova[1], 0, f->src_ctrl,
				istride, istride,
				out->iova[0], 0, 0, f->control, ostride, 0, w, h);
			dev_dbg(&mdp->pdev->dev,
				"kernel-direct fmtconv (mt8183, NV12->GREY via cvt)\n");
			return 0;
		}
		/* mt8183: MULTI-PLANE NV12->NV21 via the 64x32-tiled WROT path
		 * (mdp_config_compute_8183_fmtconv) -- cvt's full-width pass corrupts the Y
		 * plane for >=2-plane output; the 64x32 tiling (modeled on the detile) fixes
		 * it and is HW bit-exact for NV21 (interleaved chroma). Single GCE config only
		 * for now (<=14 of the 64x32 tiles, e.g. 128x128=8); larger -> SCP until the
		 * multi-chunk dispatch lands. I420/YV12 (PLANAR chroma) are NOT yet correct
		 * (Y bit-exact via this path, but the planar chroma sub-tile offset is wrong
		 * -- 128x128 content: chroma 6122/8192 B differ; needs WROT-planar-chroma RE),
		 * so they stay SCP. Every 1-PLANE output rides this path: RGB
		 * (RGB565/BGR888/AR24/BA24, f->rgb, WROT YUV->RGB matrix 0x61) AND packed
		 * 4:2:2 (YUYV/YVYU/UYVY/VYUY, uv_ysel=2 in the control for the 4:2:0->4:2:2
		 * vertical chroma interp) -- no planar de-interleave. NV21/NM21 (2-plane
		 * 4:2:0 interleaved) also ride it. NV16/NV61 (2-plane 4:2:2, full-height
		 * interleaved chroma) ride it too: same 2-plane interleaved write as NV21
		 * (the clean class -- NOT the I420/YV12 3-plane de-interleave wall) + the
		 * YUYV 4:2:0->4:2:2 uv_ysel=2 upsample (control 0x8000500d/0x8000510d); the
		 * only delta is the full-height chroma row offset (mdp_config_compute_8183_fmtconv
		 * cvshift). I420/YV12 (PLANAR chroma) stay SCP (3-plane de-interleave is HW-limited).
		 * A grid > IMG_MAX_SUBFRAMES_8183 chunks into ceil(ntiles/14) sequential
		 * GCE submissions via mdp_cmdq_send_detile (the shared multi-job endpoint;
		 * the per-chunk recompute dispatches fmtconv) -- each 64x32 tile is
		 * independent (reads its input tile, writes its output tile), exactly like
		 * the MM21 detile, so chunking is bit-exact. w,h <= 4096 (decoder range). */
		if (f && CFG_CHECK(MT8183, mdp->mdp_data->mdp_plat_id) &&
		    w <= 4096 && h <= 4096 &&
		    (MDP_COLOR_GET_PLANE_COUNT(param->outputs[0].buffer.format.colorformat) == 1 ||
		     f->color == MDP_COLOR_NV12 ||	/* I420/YV12 src -> NV12 (2-plane) */
		     f->color == MDP_COLOR_NV21 ||
		     f->color == MDP_COLOR_NV16 || f->color == MDP_COLOR_NV61 ||
		     (mdp_kd_fmtmp_planar && (f->color == MDP_COLOR_I420 ||
					      f->color == MDP_COLOR_YV12)))) {
			u32 ntiles = DIV_ROUND_UP(w, 64) * DIV_ROUND_UP(h, 32);

			vpu->detile_ntiles = ntiles;
			vpu->detile_nchunks = DIV_ROUND_UP(ntiles, IMG_MAX_SUBFRAMES_8183);
			mdp_config_compute_8183_fmtconv(mdp, param,
				(struct img_config *)vpu->config, f, 0,
				min_t(u32, IMG_MAX_SUBFRAMES_8183, ntiles));
			dev_dbg(&mdp->pdev->dev,
				"kernel-direct fmtconv (mt8183, NV12->%#x, %u tiles, %u chunk(s))\n",
				f->color, ntiles, vpu->detile_nchunks);
			return 0;
		}
		dev_dbg(&mdp->pdev->dev,
			"fmtconv %ux%u unported/oversize for this SoC; falling back to SCP\n",
			w, h);
	} else if (mdp_kernel_direct &&
		   ((mdp_kd_i420dst_hybrid &&
		     CFG_CHECK(MT8183, mdp->mdp_data->mdp_plat_id)) ||
		    (mdp_kd_i420dst8195 &&
		     CFG_CHECK(MT8195, mdp->mdp_data->mdp_plat_id))) &&
		   mdp_frame_is_i420dst_hybrid(param)) {
		/* I420/YV12-DST HYBRID: the WROT cannot write a 3-plane planar dst (HW
		 * wall on BOTH mt8183 and mt8195). Reconfigure the HW output to 2-plane
		 * NV12 (writes Y + interleaved CbCr into the SAME single dma-buf, whose
		 * Y+U+V region == an NV12 Y+UV image), run the already-validated fmtconv
		 * machinery, then de-interleave the CbCr plane -> separate U/V planes on
		 * the CPU in device_run. Any source the fmtconv path can turn into NV12
		 * is admitted; the reconfig + mdp_frame_is_nv12_fmtconv() re-check pins
		 * that. mt8183 uses the 64x32-tiled multi-chunk fmtconv; mt8195 uses the
		 * single-pipe compute (nchunks=1, <=1920). */
		bool is8195 = CFG_CHECK(MT8195, mdp->mdp_data->mdp_plat_id);
		u32 w = param->outputs[0].buffer.format.width;
		u32 h = param->outputs[0].buffer.format.height;
		bool yv12 = (param->outputs[0].buffer.format.colorformat == MDP_COLOR_YV12);
		u32 orig = param->outputs[0].buffer.format.colorformat;
		u32 ostride = param->outputs[0].buffer.format.plane_fmt[0].stride;
		u32 orig_cstride = param->outputs[0].buffer.format.plane_fmt[1].stride;
		u32 in_color = param->inputs[0].buffer.format.colorformat;
		bool nv12_src = mdp_color_is_plain_nv12(in_color);
		/* MM21/MT21 tiled source (decoder path) -> contiguous single-buffer
		 * I420/YV12. The single-buffer twin of the mm21_src arm already wired
		 * into the multi-planar i420dst_mp arm below: the srcmm21 detile-read +
		 * NV12 identity write (mdp_fmtconv_nv12) produces detiled NV12 straight
		 * into the dst dma-buf, then the CPU de-interleaves UV -> U/V in place.
		 * No scratch (unlike the mp arm -- the dst planes are contiguous here).
		 * mt8195 (SCP-tail class #1, mdp_kd_mm21dst8195): mt8195's fmtconv
		 * compute does NOT detile-read MM21, so an MM21 src runs the DETILE
		 * compute (mdp_config_compute_8195) instead -- it block-detile-reads MM21
		 * and writes linear NV12 into the contiguous dst buffer, then the CPU
		 * de-interleaves the UV -> U/V in place (no scratch, contiguous planes). */
		bool mm21_src = mdp_kd_mm21conv && mdp_color_is_mm21src(in_color) &&
				((mdp_kd_8183_ident &&
				  CFG_CHECK(MT8183, mdp->mdp_data->mdp_plat_id)) ||
				 (mdp_kd_mm21dst8195 && is8195));

		/* Reconfigure dst -> NV12 (2-plane) for the HW pass. iova[0]/iova[1] are
		 * unchanged (iova[1] = the I420 U-plane base = where the compact NV12 UV
		 * plane belongs). The compute keys planar/stride off the colorformat, so
		 * setting NV12 makes it a clean 2-plane write. */
		param->outputs[0].buffer.format.colorformat = MDP_COLOR_NV12;
		/* mt8195's compute reads plane_fmt[1].stride directly for the WROT UV
		 * pitch (u[5]); the I420 buffer's chroma stride is w/2, but the compact
		 * NV12 UV plane must be written at the full Y stride. Override it to
		 * ostride for the mt8195 pass (mt8183's compute recomputes the UV stride
		 * internally from the colorformat, so it does not read this field). */
		if (is8195)
			param->outputs[0].buffer.format.plane_fmt[1].stride = ostride;
		/*
		 * >1920 (DUAL_BITBLT): GENERIC 3-plane-dst dual (mdp_kd_gen3p). The dst is
		 * now reconfigured to 2-plane NV12; for ANY source the <=1920 i420dst hybrid
		 * claims, drive the dual NV12 write into the contiguous I420/YV12 Y+UV region
		 * via the shared mdp_fill_dual_fmtconv_8195(), then the SAME whole-frame CPU
		 * de-interleave (vpu->i420_deint) splits the compact UV -> the separate U/V
		 * planes. Subsumes the specific NV12-src + non-NV12-src dual arms below
		 * (byte-identical read/WROT for their classes) and adds the NV21/NM21-src
		 * residual. WROT row: plain NV12 for nv12/packed/nv16-src (identity/decimate,
		 * swap handled by RDMA-un-swap or the de-interleave plane-order XOR); else
		 * mdp_fmtconv_for (i420/ym16 3-plane un-swapped at the RDMA; rgb read-CSC;
		 * nv21 swap-twin WROT). MM21/MT21 sources stay SCP (block-detile dual). */
		if (mdp_kd_gen3p && is8195 &&
		    param->type == MDP_STREAM_TYPE_DUAL_BITBLT && w <= 3840 && !(w & 3) &&
		    !mdp_color_is_mm21src(in_color) &&
		    !mdp_color_is_plain_nv24src(in_color) &&
		    (nv12_src || mdp_frame_is_nv12_fmtconv(param))) {
			bool packed_src = mdp_color_is_packed422src(in_color);
			bool nv16_src = mdp_color_is_plain_nv16src(in_color);
			const struct mdp_fmtconv *f = (nv12_src || packed_src || nv16_src) ?
				&mdp_fmtconv_nv12 : mdp_fmtconv_for(param);

			if (f) {
				mdp_fill_dual_fmtconv_8195(vpu, param, f);
				vpu->i420_deint = true;
				vpu->i420_deint_yv12 = yv12 ^
					(nv16_src && MDP_COLOR_IS_SWAPPED(in_color));
				dev_dbg(&mdp->pdev->dev,
					"kernel-direct GENERIC dual I420/YV12-dst (mt8195, %ux%u %#x->%s)\n",
					w, h, in_color, yv12 ? "YV12" : "I420");
				return 0;
			}
		}
		/*
		 * >1920 (DUAL_BITBLT): MM21/MT21 SOURCE -> 3-plane contiguous I420/YV12. The
		 * gen3p arm above uses the LINEAR dual read and excludes MM21; this twin uses
		 * the BLOCK-DETILE dual read + NV12 WROT write (the dst was reconfigured to
		 * 2-plane NV12 above) into the contiguous Y+UV region, then the SAME whole-frame
		 * CPU de-interleave (vpu->i420_deint) splits the compact UV -> the separate U/V
		 * planes. The >1920 twin of the <=1920 mm21_src i420dst path. */
		if (mdp_kd_genmm21 && is8195 && mm21_src &&
		    param->type == MDP_STREAM_TYPE_DUAL_BITBLT && w <= 3840 && !(w & 3) &&
		    mdp_mm21_dual_kd_exact(param)) {
			mdp_fill_dual_mm21conv_8195(vpu, param, &mdp_fmtconv_nv12);
			vpu->i420_deint = true;
			vpu->i420_deint_yv12 = yv12;
			dev_dbg(&mdp->pdev->dev,
				"kernel-direct MM21 dual I420/YV12-dst (mt8195, %ux%u MM21->%s)\n",
				w, h, yv12 ? "YV12" : "I420");
			return 0;
		}
		/*
		 * >1920 (DUAL_BITBLT): the dual twin of the i420dst hybrid. The dst is
		 * now reconfigured to 2-plane NV12; write it across BOTH pixel-pipes (the
		 * committed NV12->NV12 dual blit) into the I420/YV12 buffer's Y+UV region,
		 * then the SAME CPU chroma de-interleave (vpu->i420_deint, device_run)
		 * splits the compact UV -> the separate U/V planes. NV12 SOURCE only. (The
		 * SCP does a direct dual 3-plane WROT, but the KD 3-plane planar WROT write
		 * is a HW wall -- see mdp_kd_i420dst8195 -- so we mirror the proven <=1920
		 * NV12-write + CPU hybrid over both pipes.) [gen3p=N fallback path.]
		 */
		if (is8195 && nv12_src &&
		    param->type == MDP_STREAM_TYPE_DUAL_BITBLT &&
		    w <= 3840 && !(w & 3)) {
			struct img_config *cfgs = (struct img_config *)vpu->config;
			const struct img_image_buffer *in = &param->inputs[0].buffer;
			const struct img_image_buffer *out = &param->outputs[0].buffer;
			u32 istride = in->format.plane_fmt[0].stride;
			struct mdp_dual_wrot wr;

			mdp_dual_wrot_for_dst(&wr, out, &mdp_fmtconv_nv12, false);
			mdp_fill_dual_pipe_8195_linear(&cfgs[0].config_8195, 0, w, h,
				istride, in->iova[0], in->iova[1], 0x60c, &wr, NULL);
			mdp_fill_dual_pipe_8195_linear(&cfgs[1].config_8195, 1, w, h,
				istride, in->iova[0], in->iova[1], 0x60c, &wr, NULL);
			vpu->i420_deint = true;
			vpu->i420_deint_yv12 = yv12;
			vpu->detile_nchunks = 1;
			dev_dbg(&mdp->pdev->dev,
				"kernel-direct dual I420/YV12-dst hybrid (mt8195, %ux%u NV12->%s)\n",
				w, h, yv12 ? "YV12" : "I420");
			return 0;
		}
		/*
		 * >1920 (DUAL_BITBLT): NON-NV12 SOURCE -> 3-plane I420/YV12 dst. The dst is
		 * reconfigured to 2-plane NV12 (above); read the non-NV12 source with the
		 * SAME per-family mdp_dual_src the >1920 source arms use (packed-4:2:2 /
		 * 3-plane I420/YV12/YM16/YM61 / RGB -- each family's src_ctrl is the
		 * oracle-confirmed value from those arms), write NV12 across both pixel
		 * pipes, then the CPU chroma de-interleave (vpu->i420_deint) splits the
		 * compact UV -> the separate U/V planes. The dual twin of the <=1920
		 * non-NV12-src i420dst hybrid (mdp_config_compute_8195_fmtconv + deint).
		 * Sources: packed-4:2:2 / 3-plane I420/YV12/YM16/YM61 / RGB / NV16/NV61
		 * (2-plane 4:2:2). MM21/MT21 (tiled) sources -> 3-plane dst stay SCP
		 * (the linear primitive cannot detile; needs the block-detile dual).
		 */
		if (is8195 && param->type == MDP_STREAM_TYPE_DUAL_BITBLT &&
		    w <= 3840 && !(w & 3)) {
			const struct img_image_buffer *in = &param->inputs[0].buffer;
			const struct img_image_buffer *out = &param->outputs[0].buffer;
			u32 istride = in->format.plane_fmt[0].stride;
			struct mdp_dual_src src = {};
			u32 src_ctrl = 0;
			bool ok = false, rgb_src = false;
			bool deint_yv12 = yv12;

			if (mdp_color_is_packed422src(in_color)) {
				/* packed-4:2:2 (YUYV/YVYU/UYVY/VYUY): ONE packed plane bpp=2 */
				src.single_plane = true; src.xbpp = 2;
				src_ctrl = 0x600 |
					(MDP_COLOR_IS_SWAPPED(in_color) ? 0x4000 : 0) |
					MDP_COLOR_GET_HW_FORMAT(in_color);
				ok = true;
			} else if (mdp_color_is_plain_i420src(in_color) ||
				   mdp_color_is_ym16src(in_color)) {
				/* 3-plane I420/YV12 (4:2:0) or YM16/YM61 (4:2:2) */
				bool is422 = mdp_color_is_ym16src(in_color);
				u32 sw = MDP_COLOR_IS_SWAPPED(in_color) ? 0x4000 : 0;

				src.three_plane = true; src.chroma_full_h = is422;
				src.iova2 = in->iova[2];
				src_ctrl = is422 ? (0x600 | sw | 0x09) :
					   ((mdp_fmtconv_nv12.src_ctrl & 0x600) | sw | 0x08);
				ok = true;
			} else if (mdp_color_is_plain_nv16src(in_color)) {
				/* NV16/NV61 (2-plane 4:2:2) -> 3-plane 4:2:0 I420/YV12:
				 * FULL-height plane-1 chroma read (chroma_full_h ->
				 * u[7]=ic+istride*h) + src_ctrl 0x60d (0x600 | HW_FORMAT
				 * 0xd, NO sourceSwap -- a swapped NV61 rides the plain
				 * NV12 WROT row, matching the <=1920 fmtconv nv16_src arm).
				 * The 4:2:2->4:2:0 vertical chroma DECIMATE is HW-derived
				 * from the full-height read vs the NV12 WROT write (0x500c),
				 * so the WROT is the plain mdp_fmtconv_nv12 row; the CPU
				 * de-interleave then splits the (half-height) NV12 UV -> the
				 * separate 4:2:0 U/V planes.
				 *
				 * SWAP (NV61 -> V,U interleaved): NV16 and NV61 share the same
				 * HW_FORMAT (0xd); the V,U order is the SWAPPED bit, NOT read-
				 * un-swapped here (an RDMA 0x4000 un-swap gives the WRONG output
				 * -- board-falsified: the SCP does NOT semantically un-swap for
				 * the i420dst path). Instead the swap manifests as the de-
				 * interleave plane ORDER: the compact plane carries the physical
				 * [V,U] bytes and the CPU split flips which plane gets byte 0.
				 * Board oracle: SCP(NV61>YU12)==SCP(NV16>YV12), SCP(NV61>YV12)==
				 * SCP(NV16>YU12) -> XOR the yv12 de-interleave flag with SWAPPED. */
				src.chroma_full_h = true;
				src_ctrl = 0x600 | MDP_COLOR_GET_HW_FORMAT(in_color);
				deint_yv12 = yv12 ^ MDP_COLOR_IS_SWAPPED(in_color);
				ok = true;
			} else if (mdp_color_is_rgb_src(in_color)) {
				/* RGB565/BGR565/RGB24/BGR24/ARGB/ABGR: single plane + read-CSC */
				u32 iw = in->format.width;

				src.single_plane = true; src.colortran = true;
				src.xbpp = iw ? istride / iw : 1;
				src_ctrl = 0x600 |
					(MDP_COLOR_IS_SWAPPED(in_color) ? 0x4000 : 0) |
					MDP_COLOR_GET_HW_FORMAT(in_color);
				rgb_src = true;
				ok = true;
			}

			if (ok) {
				struct img_config *cfgs = (struct img_config *)vpu->config;
				u32 ic = src.single_plane ? 0 : in->iova[1];
				struct mdp_dual_wrot wr;

				mdp_dual_wrot_for_dst(&wr, out, &mdp_fmtconv_nv12, rgb_src);
				mdp_fill_dual_pipe_8195_linear(&cfgs[0].config_8195, 0, w, h,
					istride, in->iova[0], ic, src_ctrl, &wr, &src);
				mdp_fill_dual_pipe_8195_linear(&cfgs[1].config_8195, 1, w, h,
					istride, in->iova[0], ic, src_ctrl, &wr, &src);
				vpu->i420_deint = true;
				vpu->i420_deint_yv12 = deint_yv12;
				vpu->detile_nchunks = 1;
				dev_dbg(&mdp->pdev->dev,
					"kernel-direct dual I420/YV12-dst hybrid (mt8195, %ux%u src %#x->%s)\n",
					w, h, in_color, yv12 ? "YV12" : "I420");
				return 0;
			}
		}
		/*
		 * Source-convertibility check: a NON-NV12 source (NV21/NV16/NV61/packed
		 * 4:2:2/I420/RGB) is admitted iff mdp_frame_is_nv12_fmtconv() accepts it
		 * with the dst now reconfigured to NV12 (that also confirms the src param
		 * gate + any dn420/swap plumbing). A PLAIN-NV12 source is the identity
		 * write -- NV12->NV12 is intentionally NOT an mdp_fmtconv_tbl row (it's
		 * blit's job), so mdp_frame_is_nv12_fmtconv() rejects it; accept it
		 * directly here (the outer mdp_frame_is_i420dst_hybrid already pinned 1:1 /
		 * no-rotate / full-frame / contiguous dst). Without this the whole NV12->
		 * I420/YV12 hybrid fell through to the dead SCP (-22).
		 */
		if ((nv12_src || mm21_src || mdp_frame_is_nv12_fmtconv(param)) &&
		    ((is8195 && w <= 1920 && (unsigned long long)w * h < 2073600ULL) ||
		     (!is8195 && w <= 4096 && h <= 4096))) {
			/*
			 * PACKED-4:2:2 SOURCE (YUYV/YVYU/UYVY/VYUY) -> I420/YV12 dst:
			 * use the PLAIN NV12 WROT row (mdp_fmtconv_nv12), NOT
			 * mdp_fmtconv_for(). The dst is reconfigured to NV12; for a
			 * SWAPPED source (YVYU/VYUY) mdp_fmtconv_for() would return the
			 * swap-twin NV21 row (0x510c), DOUBLE-swapping against the RDMA
			 * un-swap (the mt8195 single_plane read src_ctrl 0x4605) ->
			 * YVYU>YU12 != SCP. The mt8195 fmtconv compute derives the single
			 * packed-plane read (+ 0x4000 un-swap) from the SOURCE color, so
			 * the plain-NV12 row (WROT 0x500c) writes straight through, exactly
			 * as the single-plane fmtconv arm does for packed -> NV12 dst; the
			 * CPU then de-interleaves the NV12 UV -> I420/YV12 U/V planes.
			 * (Unswapped YUYV/UYVY already resolve to mdp_fmtconv_nv12 via
			 * mdp_fmtconv_for; the plain row unifies both.)
			 */
			bool packed_nv12row = is8195 &&
					      mdp_color_is_packed422src(in_color);
			const struct mdp_fmtconv *f = (nv12_src || mm21_src ||
						       packed_nv12row) ?
				&mdp_fmtconv_nv12 : mdp_fmtconv_for(param);

			if (f) {
				vpu->i420_deint = true;
				vpu->i420_deint_yv12 = yv12;
				if (is8195) {
					vpu->detile_nchunks = 1;
					if (mm21_src)
						mdp_config_compute_8195(param,
							(struct img_config *)vpu->config);
					else
						mdp_config_compute_8195_fmtconv(param,
							(struct img_config *)vpu->config, f);
					dev_dbg(&mdp->pdev->dev,
						"kernel-direct I420/YV12-dst hybrid (mt8195, %s%s, HW NV12 single-pipe + CPU chroma de-interleave)\n",
						mm21_src ? "MM21->" : "",
						yv12 ? "YV12" : "I420");
				} else {
					u32 ntiles = DIV_ROUND_UP(w, 64) *
						     DIV_ROUND_UP(h, 32);

					vpu->detile_ntiles = ntiles;
					vpu->detile_nchunks =
						DIV_ROUND_UP(ntiles, IMG_MAX_SUBFRAMES_8183);
					mdp_config_compute_8183_fmtconv(mdp, param,
						(struct img_config *)vpu->config, f, 0,
						min_t(u32, IMG_MAX_SUBFRAMES_8183, ntiles));
					dev_dbg(&mdp->pdev->dev,
						"kernel-direct I420/YV12-dst hybrid (mt8183, %s, HW NV12 %u tiles/%u chunk(s) + CPU chroma de-interleave)\n",
						yv12 ? "YV12" : "I420", ntiles,
						vpu->detile_nchunks);
				}
				return 0;
			}
		}
		/* not convertible -> restore the dst format + chroma stride and fall to SCP */
		param->outputs[0].buffer.format.colorformat = orig;
		if (is8195)
			param->outputs[0].buffer.format.plane_fmt[1].stride = orig_cstride;
		dev_dbg(&mdp->pdev->dev,
			"I420/YV12-dst hybrid %ux%u unconvertible source; falling back to SCP\n",
			w, h);
	} else if (mdp_kernel_direct &&
		   ((mdp_kd_i420dst_mp &&
		     CFG_CHECK(MT8183, mdp->mdp_data->mdp_plat_id)) ||
		    (mdp_kd_i420dst8195 &&
		     CFG_CHECK(MT8195, mdp->mdp_data->mdp_plat_id))) &&
		   mdp_frame_is_i420dst_mp(param)) {
		/* MULTI-PLANAR I420/YV12-DST HYBRID (YM12/YM21): identical to the i420dst
		 * hybrid above, but the dst U/V planes are SEPARATE dma-bufs, so the
		 * compact NV12 interleaved-UV plane cannot be written contiguously after
		 * Y. Instead: HW Y -> the real Y-plane iova[0] (unchanged, HW-final); HW
		 * interleaved-UV -> a SCRATCH dma-buf (override iova[1]); then the CPU
		 * de-interleaves the scratch UV -> the two separate U/V plane dma-bufs
		 * (iova[1]/iova[2]) in device_run. Everything else (source read, WROT NV12
		 * write) is the already-validated fmtconv machinery -- mt8183 via the
		 * 64x32-tiled multi-chunk path, mt8195 via the single-pipe compute. */
		bool is8195 = CFG_CHECK(MT8195, mdp->mdp_data->mdp_plat_id);
		u32 w = param->outputs[0].buffer.format.width;
		u32 h = param->outputs[0].buffer.format.height;
		bool yv12 = (param->outputs[0].buffer.format.colorformat == MDP_COLOR_YV12);
		u32 orig = param->outputs[0].buffer.format.colorformat;
		u32 orig_iova1 = param->outputs[0].buffer.iova[1];
		u32 ostride = param->outputs[0].buffer.format.plane_fmt[0].stride;
		u32 orig_cstride = param->outputs[0].buffer.format.plane_fmt[1].stride;
		u32 in_color = param->inputs[0].buffer.format.colorformat;
		bool nv12_src = mdp_color_is_plain_nv12(in_color);
		/* MM21/MT21 tiled source (decoder path): the fmtconv gate excludes
		 * MM21->NV12 (that is the pure detile arm), so mm21_src is admitted
		 * directly here. mt8183 uses the srcmm21 fmtconv read + NV12 identity
		 * write; mt8195 (SCP-tail class #1) has no MM21 fmtconv read, so it runs
		 * the DETILE compute (mdp_config_compute_8195) -> NV12 into scratch. Gate
		 * mt8195 behind mdp_kd_mm21dst8195 (knob off -> falls to the SCP). */
		bool mm21_src = mdp_kd_mm21conv && mdp_color_is_mm21src(in_color) &&
				(!is8195 || mdp_kd_mm21dst8195);
		size_t uvsz = (size_t)ostride * (h / 2);	/* NV12 interleaved UV plane */

		/* Reconfigure dst -> NV12 (2-plane), and redirect the UV write to scratch.
		 * iova[0] (Y) stays the real Y-plane dma-buf (HW-final). iova[1] becomes the
		 * scratch base so the interleaved CbCr lands somewhere large enough (the real
		 * U-plane dma-buf is only ostride/2 * h/2, half the compact UV size). */
		param->outputs[0].buffer.format.colorformat = MDP_COLOR_NV12;
		/* mt8195 compute reads plane_fmt[1].stride for the WROT UV pitch (u[5]);
		 * the compact NV12 UV plane written to scratch must use the full Y stride,
		 * not the separate chroma plane's w/2. (mt8183's compute recomputes this
		 * internally, so it does not read the field.) */
		if (is8195)
			param->outputs[0].buffer.format.plane_fmt[1].stride = ostride;
		/*
		 * >1920 (DUAL_BITBLT): GENERIC multi-planar 3-plane-4:2:0-dst dual
		 * (mdp_kd_gen3p) -- YM12/YM21 had NO dual arm, so the whole branch fell to
		 * the SCP at >1920. Same as the contiguous i420dst gen3p arm but the compact
		 * interleaved-UV is written to a SCRATCH dma-buf (iova[1] redirected) and the
		 * CPU splits scratch -> the two SEPARATE U/V plane dma-bufs (i420_deint_mp).
		 * For ANY source the <=1920 mp hybrid claims. WROT row selection + swap XOR
		 * identical to the contiguous arm (both target 2-plane NV12 4:2:0).
		 */
		if (mdp_kd_gen3p && is8195 &&
		    param->type == MDP_STREAM_TYPE_DUAL_BITBLT && w <= 3840 && !(w & 3) &&
		    !mdp_color_is_mm21src(in_color) &&
		    !mdp_color_is_plain_nv24src(in_color) &&
		    (nv12_src || mdp_frame_is_nv12_fmtconv(param))) {
			bool packed_src = mdp_color_is_packed422src(in_color);
			bool nv16_src = mdp_color_is_plain_nv16src(in_color);
			const struct mdp_fmtconv *f = (nv12_src || packed_src || nv16_src) ?
				&mdp_fmtconv_nv12 : mdp_fmtconv_for(param);

			if (f && !mdp_vpu_scratch_ensure(vpu, uvsz)) {
				/* redirect the HW UV write to scratch_a */
				param->outputs[0].buffer.iova[1] = (u32)vpu->scratch_a_addr;
				mdp_fill_dual_fmtconv_8195(vpu, param, f);
				vpu->i420_deint = true;
				vpu->i420_deint_mp = true;
				vpu->i420_deint_yv12 = yv12 ^
					(nv16_src && MDP_COLOR_IS_SWAPPED(in_color));
				dev_dbg(&mdp->pdev->dev,
					"kernel-direct GENERIC dual multi-planar I420/YV12-dst (mt8195, %ux%u %#x->%s)\n",
					w, h, in_color, yv12 ? "YV12M" : "I420M");
				return 0;
			}
		}
		/*
		 * >1920 (DUAL_BITBLT): MM21/MT21 SOURCE -> 3-plane multi-planar YM12/YM21. The
		 * gen3p mp arm above excludes MM21; this twin block-detile-dual-reads MM21 and
		 * NV12-writes to the UV scratch (dst reconfigured to 2-plane NV12 above), then
		 * the CPU de-interleaves scratch -> the two SEPARATE U/V dma-bufs (i420_deint_mp).
		 * The >1920 twin of the <=1920 mm21_src multi-planar path. */
		if (mdp_kd_genmm21 && is8195 && mm21_src &&
		    param->type == MDP_STREAM_TYPE_DUAL_BITBLT && w <= 3840 && !(w & 3) &&
		    mdp_mm21_dual_kd_exact(param) &&
		    !mdp_vpu_scratch_ensure(vpu, uvsz)) {
			param->outputs[0].buffer.iova[1] = (u32)vpu->scratch_a_addr;
			mdp_fill_dual_mm21conv_8195(vpu, param, &mdp_fmtconv_nv12);
			vpu->i420_deint = true;
			vpu->i420_deint_mp = true;
			vpu->i420_deint_yv12 = yv12;
			dev_dbg(&mdp->pdev->dev,
				"kernel-direct MM21 dual multi-planar I420/YV12-dst (mt8195, %ux%u MM21->%s)\n",
				w, h, yv12 ? "YV12M" : "I420M");
			return 0;
		}
		if ((nv12_src || mm21_src || mdp_frame_is_nv12_fmtconv(param)) &&
		    ((is8195 && w <= 1920 && (unsigned long long)w * h < 2073600ULL) ||
		     (!is8195 && w <= 4096 && h <= 4096)) &&
		    !mdp_vpu_scratch_ensure(vpu, uvsz)) {
			/* PACKED-4:2:2 SOURCE -> multi-planar 3-plane dst: plain NV12 WROT
			 * row (see the contiguous i420dst hybrid above) -- the swap-twin
			 * would double-swap the RDMA-un-swapped packed read. */
			bool packed_nv12row = is8195 &&
					      mdp_color_is_packed422src(in_color);
			const struct mdp_fmtconv *f = (nv12_src || mm21_src ||
						       packed_nv12row) ?
				&mdp_fmtconv_nv12 : mdp_fmtconv_for(param);

			if (f) {
				/* redirect the HW UV write to scratch_a */
				param->outputs[0].buffer.iova[1] =
					(u32)vpu->scratch_a_addr;
				vpu->i420_deint = true;
				vpu->i420_deint_mp = true;
				vpu->i420_deint_yv12 = yv12;
				if (is8195) {
					vpu->detile_nchunks = 1;
					if (mm21_src)
						mdp_config_compute_8195(param,
							(struct img_config *)vpu->config);
					else
						mdp_config_compute_8195_fmtconv(param,
							(struct img_config *)vpu->config, f);
					dev_dbg(&mdp->pdev->dev,
						"kernel-direct multi-planar I420/YV12-dst hybrid (mt8195, %s%s, HW NV12 single-pipe -> scratch + CPU de-interleave to 3 planes)\n",
						mm21_src ? "MM21->" : "",
						yv12 ? "YV12M" : "I420M");
				} else {
					u32 ntiles = DIV_ROUND_UP(w, 64) *
						     DIV_ROUND_UP(h, 32);

					vpu->detile_ntiles = ntiles;
					vpu->detile_nchunks =
						DIV_ROUND_UP(ntiles, IMG_MAX_SUBFRAMES_8183);
					mdp_config_compute_8183_fmtconv(mdp, param,
						(struct img_config *)vpu->config, f, 0,
						min_t(u32, IMG_MAX_SUBFRAMES_8183, ntiles));
					dev_dbg(&mdp->pdev->dev,
						"kernel-direct multi-planar I420/YV12-dst hybrid (mt8183, %s, HW NV12 %u tiles/%u chunk(s) -> scratch + CPU de-interleave to 3 planes)\n",
						yv12 ? "YV12M" : "I420M", ntiles,
						vpu->detile_nchunks);
				}
				return 0;
			}
		}
		/* not convertible / no scratch -> restore dst format + iova + cstride, fall to SCP */
		param->outputs[0].buffer.format.colorformat = orig;
		param->outputs[0].buffer.iova[1] = orig_iova1;
		if (is8195)
			param->outputs[0].buffer.format.plane_fmt[1].stride = orig_cstride;
		dev_dbg(&mdp->pdev->dev,
			"multi-planar I420/YV12-dst hybrid %ux%u unconvertible; falling back to SCP\n",
			w, h);
	} else if (mdp_kernel_direct && mdp_kd_ym16dst8195 &&
		   CFG_CHECK(MT8195, mdp->mdp_data->mdp_plat_id) &&
		   mdp_frame_is_ym16dst(param)) {
		/* SCP-tail class #2: 3-plane 4:2:2 dst (YM16/YM61). The WROT cannot write
		 * a 3-plane planar 4:2:2 dst; reconfigure the HW output to 2-plane NV16
		 * (Y + interleaved CbCr, FULL-height chroma), run the already-validated
		 * source->NV16 fmtconv (NV16 IS an mdp_fmtconv_tbl row), HW-write the
		 * interleaved CbCr to scratch, then CPU de-interleave (full-height) -> the
		 * separate U/V dma-bufs. Structurally the i420dst_mp arm but 4:2:2. */
		u32 w = param->outputs[0].buffer.format.width;
		u32 h = param->outputs[0].buffer.format.height;
		bool yv16 = MDP_COLOR_IS_SWAPPED(param->outputs[0].buffer.format.colorformat);
		u32 orig = param->outputs[0].buffer.format.colorformat;
		u32 orig_iova1 = param->outputs[0].buffer.iova[1];
		u32 ostride = param->outputs[0].buffer.format.plane_fmt[0].stride;
		u32 orig_cstride = param->outputs[0].buffer.format.plane_fmt[1].stride;
		u32 in_color = param->inputs[0].buffer.format.colorformat;
		bool nv12_src = mdp_color_is_plain_nv12(in_color);
		bool mm21_src = mdp_kd_mm21conv8195 &&
				mdp_color_is_mm21src(in_color);
		size_t uvsz = (size_t)ostride * h;	/* NV16 interleaved UV plane (FULL height) */

		/* Reconfigure dst -> NV16 (2-plane 4:2:2); UV pitch = full Y stride. */
		param->outputs[0].buffer.format.colorformat = MDP_COLOR_NV16;
		param->outputs[0].buffer.format.plane_fmt[1].stride = ostride;
		/*
		 * >1920 (DUAL_BITBLT): GENERIC 3-plane-4:2:2-dst dual (mdp_kd_gen3p). The
		 * dst is now reconfigured to 2-plane NV16 (FULL-height interleaved UV); for
		 * ANY source the <=1920 ym16 hybrid claims, drive the dual NV16 write to the
		 * UV scratch via the shared mdp_fill_dual_fmtconv_8195(), then the CPU
		 * full-height de-interleave (ym16_deint) splits scratch -> the separate U/V
		 * dma-bufs. Subsumes the specific packed/ym16-src dual arm below (byte-
		 * identical read/WROT) + adds the 4:2:0->4:2:2 UPSAMPLE sources (NV12/I420/
		 * NV21) and RGB/NV16: the WROT chroma UPSAMPLE is entirely in
		 * mdp_fmtconv_for()'s NV16 control word (uv_ysel), orthogonal to the pw=w/2
		 * horizontal split -- a plain NV16 row (packed/ym16 already 4:2:2, no
		 * resample). MM21/MT21 sources stay SCP. */
		if (mdp_kd_gen3p &&
		    param->type == MDP_STREAM_TYPE_DUAL_BITBLT && w <= 3840 && !(w & 3) &&
		    !mdp_color_is_mm21src(in_color) &&
		    !mdp_color_is_plain_nv24src(in_color) &&
		    (nv12_src || mdp_frame_is_nv12_fmtconv(param))) {
			bool packed_src = mdp_color_is_packed422src(in_color);
			const struct mdp_fmtconv *f = packed_src ?
				mdp_fmtconv_lookup(MDP_COLOR_NV16) : mdp_fmtconv_for(param);

			if (f && !mdp_vpu_scratch_ensure(vpu, uvsz)) {
				param->outputs[0].buffer.iova[1] = (u32)vpu->scratch_a_addr;
				mdp_fill_dual_fmtconv_8195(vpu, param, f);
				vpu->i420_deint = true;
				vpu->i420_deint_mp = true;
				vpu->ym16_deint = true;
				vpu->i420_deint_yv12 = yv16;
				dev_dbg(&mdp->pdev->dev,
					"kernel-direct GENERIC dual 3-plane 4:2:2-dst (mt8195, %ux%u %#x->%s)\n",
					w, h, in_color, yv16 ? "YM61" : "YM16");
				return 0;
			}
		}
		/*
		 * >1920 (DUAL_BITBLT): the dual twin of the 3-plane 4:2:2 dst hybrid. Write
		 * NV16 (FULL-height interleaved UV) across BOTH pixel pipes into the UV
		 * scratch, then the CPU full-height de-interleave (ym16_deint) splits it ->
		 * the separate U/V dma-bufs. Scoped to 4:2:2->4:2:2 SOURCES (packed-4:2:2 /
		 * 3-plane YM16/YM61) where the write is a plain NV16 row (no vertical
		 * resample) + the source read un-swaps at the RDMA (the proven <=1920
		 * i420dst/#2-dual pattern). 4:2:0->4:2:2 (NV12/I420, needs chroma UPSAMPLE)
		 * + RGB + NV16/NV61-src stay SCP (oracle: mdp_fmtconv_for upsample control,
		 * not the plain row -- a follow-up).
		 */
		if (CFG_CHECK(MT8195, mdp->mdp_data->mdp_plat_id) &&
		    param->type == MDP_STREAM_TYPE_DUAL_BITBLT && w <= 3840 && !(w & 3) &&
		    (mdp_color_is_packed422src(in_color) ||
		     mdp_color_is_ym16src(in_color)) &&
		    !mdp_vpu_scratch_ensure(vpu, uvsz)) {
			const struct img_image_buffer *in = &param->inputs[0].buffer;
			const struct mdp_fmtconv *f = mdp_fmtconv_lookup(MDP_COLOR_NV16);

			if (f) {
				struct img_config *cfgs = (struct img_config *)vpu->config;
				u32 istride = in->format.plane_fmt[0].stride;
				bool ym16 = mdp_color_is_ym16src(in_color);
				u32 sw = MDP_COLOR_IS_SWAPPED(in_color) ? 0x4000 : 0;
				u32 src_ctrl = ym16 ? (0x600 | sw | 0x09) :
					(0x600 | sw | MDP_COLOR_GET_HW_FORMAT(in_color));
				struct mdp_dual_src src = {
					.single_plane = !ym16,
					.three_plane = ym16,
					.chroma_full_h = ym16,	/* 4:2:2 full-height chroma */
					.iova2 = ym16 ? in->iova[2] : 0,
					.xbpp = ym16 ? 0 : 2,	/* packed-4:2:2 = 2 bytes/px */
				};
				u32 ic = ym16 ? in->iova[1] : 0;
				const struct img_image_buffer *out;
				struct mdp_dual_wrot wr;

				param->outputs[0].buffer.iova[1] = (u32)vpu->scratch_a_addr;
				out = &param->outputs[0].buffer;
				mdp_dual_wrot_for_dst(&wr, out, f, false);
				mdp_fill_dual_pipe_8195_linear(&cfgs[0].config_8195, 0, w, h,
					istride, in->iova[0], ic, src_ctrl, &wr, &src);
				mdp_fill_dual_pipe_8195_linear(&cfgs[1].config_8195, 1, w, h,
					istride, in->iova[0], ic, src_ctrl, &wr, &src);
				vpu->i420_deint = true;
				vpu->i420_deint_mp = true;
				vpu->ym16_deint = true;
				vpu->i420_deint_yv12 = yv16;
				vpu->detile_nchunks = 1;
				dev_dbg(&mdp->pdev->dev,
					"kernel-direct dual 3-plane 4:2:2 dst hybrid (mt8195, %ux%u src %#x->%s)\n",
					w, h, in_color, yv16 ? "YM61" : "YM16");
				return 0;
			}
		}
		if ((nv12_src || mm21_src || mdp_frame_is_nv12_fmtconv(param)) &&
		    w <= 1920 && (unsigned long long)w * h < 2073600ULL &&
		    !mdp_vpu_scratch_ensure(vpu, uvsz)) {
			/* PACKED-4:2:2 / MM21 source -> plain NV16 row (the swap-twin would
			 * double-swap the RDMA-un-swapped packed read, as in the i420dst arm).
			 * MM21 uses the same NV16 row as MM21>NV16 (srcmm21 detile + upsample). */
			bool packed_nv16row = mdp_color_is_packed422src(in_color);
			const struct mdp_fmtconv *f = (packed_nv16row || mm21_src) ?
				mdp_fmtconv_lookup(MDP_COLOR_NV16) : mdp_fmtconv_for(param);

			if (f) {
				param->outputs[0].buffer.iova[1] = (u32)vpu->scratch_a_addr;
				vpu->i420_deint = true;
				vpu->i420_deint_mp = true;
				vpu->ym16_deint = true;		/* full-height 4:2:2 de-interleave */
				vpu->i420_deint_yv12 = yv16;	/* YM61 (V-first) U/V plane order */
				vpu->detile_nchunks = 1;
				mdp_config_compute_8195_fmtconv(param,
					(struct img_config *)vpu->config, f);
				dev_dbg(&mdp->pdev->dev,
					"kernel-direct 3-plane 4:2:2 dst hybrid (mt8195, %s, HW NV16 single-pipe -> scratch + CPU full-height de-interleave)\n",
					yv16 ? "YM61" : "YM16");
				return 0;
			}
		}
		/* not convertible / no scratch -> restore dst format + iova + cstride, fall to SCP */
		param->outputs[0].buffer.format.colorformat = orig;
		param->outputs[0].buffer.iova[1] = orig_iova1;
		param->outputs[0].buffer.format.plane_fmt[1].stride = orig_cstride;
		dev_dbg(&mdp->pdev->dev,
			"3-plane 4:2:2 dst hybrid %ux%u unconvertible; falling back to SCP\n",
			w, h);
	} else if (mdp_kernel_direct && (mdp_kd_crop || mdp_kd_cropscale) &&
		   mdp_frame_is_nv12_crop(param)) {
		/* Kernel-direct sub-region CROP (S_SELECTION). The SCP routes the crop
		 * through the RSZ as a cubic resample of the crop rect (NOT a pixel-exact
		 * ROI); mdp_config_compute_8195_crop reproduces that config byte-exact so
		 * the GCE output == the SCP output (firmware-independent on v26100,
		 * KI-009). Two cases:
		 *  - 1:1 (crop W/H == out W/H, mdp_kd_crop): native-res extraction.
		 *  - crop+scale (crop W/H != out W/H, mdp_kd_cropscale): the crop rect is
		 *    resampled to the output (coeff keyed off the crop dims, origin folded
		 *    into the RSZ phase). PURE up OR down only; MIXED -> SCP.
		 * Single RSZ tile: the RSZ input is the crop width + a per-ratio back-tap,
		 * bound to one line buffer (<=512), output <=512; the downscale back-margin
		 * (mh+8 RDMA / mv+1) must fit inside the frame. Wider -> SCP. */
		u32 cl = param->outputs[0].crop.left;
		u32 ct = param->outputs[0].crop.top;
		u32 cw = param->outputs[0].crop.width;
		u32 ch = param->outputs[0].crop.height;
		u32 ow = param->outputs[0].buffer.format.width;
		u32 oh = param->outputs[0].buffer.format.height;
		bool is11 = (cw == ow && ch == oh);
		bool origin = (!cl && !ct);		/* ORIGIN (0,0) crop arm */
		bool pdown = (ow <= cw && oh <= ch);	/* pure down or 1:1 */
		bool pup   = (ow >= cw && oh >= ch);	/* pure up or 1:1 */
		/* MIXED crop+scale = strictly one axis up AND the other down. The crop
		 * compute is per-axis (cx/cy off up_h/up_v, separable control1, per-axis
		 * phase) so it programs each axis independently; gated behind mdp_kd_mixed
		 * (default-on) -- a mixed crop otherwise routes to the SCP. */
		bool pmixed = (ow > cw && oh < ch) || (ow < cw && oh > ch);
		bool gated = is11 ? mdp_kd_crop : mdp_kd_cropscale;
		u32 cx, cy, mh, mv;

		mdp_crop_scale_params(cw, ch, ow, oh, &cx, &cy, &mh, &mv);
		/* ORIGIN 1:1-wide crop (cl==ct==0, cw==ow>512, ch==oh): a genuine
		 * top-left cw x ch sub-window BLIT, NOT a scale. The SCP handles the
		 * origin-1:1 crop as a blit-class op; routing it through the unity-coeff
		 * scale march is KD but NOT bit-exact vs the SCP (board-falsified
		 * 2026-07-10). The linear blit primitive reads w=cw columns at the FULL
		 * input istride (istride > cw) from the origin bases -> the top-left
		 * window, and writes the cw x ch NV12 output byte-for-byte as the proven
		 * NV12->NV12 blit (dual: mdp_fill_dual_pipe_8195_linear src_ctrl 0x60c;
		 * single-pipe: mdp_config_compute_8195_blit, which already keys the read
		 * off the out dims + input istride). DUAL area (crop.area >= pp_criteria
		 * -> MDP_STREAM_TYPE_DUAL_BITBLT) fills both pipes; else single-pipe. */
		if (CFG_CHECK(MT8195, mdp->mdp_data->mdp_plat_id) && origin && is11 &&
		    mdp_kd_crop && cw > 512 && cw <= 3840 && !(cw & 3)) {
			const struct img_image_buffer *in = &param->inputs[0].buffer;
			const struct img_image_buffer *out = &param->outputs[0].buffer;

			vpu->detile_nchunks = 1;
			if (param->type == MDP_STREAM_TYPE_DUAL_BITBLT) {
				struct img_config *cfgs =
					(struct img_config *)vpu->config;
				u32 istride = in->format.plane_fmt[0].stride;
				struct mdp_dual_wrot wr;

				mdp_dual_wrot_for_dst(&wr, out, &mdp_fmtconv_nv12,
						      false);
				mdp_fill_dual_pipe_8195_linear(&cfgs[0].config_8195,
					0, cw, ch, istride, in->iova[0],
					in->iova[1], 0x60c, &wr, NULL);
				mdp_fill_dual_pipe_8195_linear(&cfgs[1].config_8195,
					1, cw, ch, istride, in->iova[0],
					in->iova[1], 0x60c, &wr, NULL);
			} else {
				mdp_config_compute_8195_blit(param,
					(struct img_config *)vpu->config);
				param->type = MDP_STREAM_TYPE_BITBLT;
			}
			dev_dbg(&mdp->pdev->dev,
				"kernel-direct origin 1:1-wide crop (mt8195, crop(0,0 %ux%u) blit %s)\n",
				cw, ch,
				param->type == MDP_STREAM_TYPE_DUAL_BITBLT ?
					"dual" : "single");
			return 0;
		}
		/* ORIGIN WIDE crop (cw > 512 -> the RSZ input span exceeds one line
		 * buffer -> multi-tile). An ORIGIN crop == a PLAIN cw x ch -> ow x oh
		 * scale that reads the cw x ch top-left sub-window of the wider buffer
		 * (proven on-board: the origin-crop SCP config is byte-identical to the
		 * plain cw x ch scale except the c0-c2 in-frame dims). So drive it through
		 * the VALIDATED multi-tile scale march (mdp_config_compute_8195_scale,
		 * tpipe) with the crop dims as the scale input -- an in-place input-dims
		 * swap (restored after) keeps the real istride + base so the RDMA reads
		 * the sub-window. Pure up OR down only (mixed multi-tile crop not ported;
		 * !is11 here -- the ORIGIN 1:1-wide case is a blit and is handled by the
		 * origin-1:1-wide blit arm ABOVE, since the unity-coeff scale march is
		 * KD but NOT bit-exact vs the SCP's blit-class origin-1:1 handling
		 * (board-falsified 2026-07-10)). */
		if (CFG_CHECK(MT8195, mdp->mdp_data->mdp_plat_id) && origin && !is11 &&
		    mdp_kd_cropscale && (pdown || pup) && cw > 512 &&
		    cw <= 4096 && ch <= 4096 && ow <= 4096 && oh <= 4096) {
			struct img_image_buffer *inb = &param->inputs[0].buffer;
			u32 sw = inb->format.width, sh = inb->format.height;

			inb->format.width = cw;		/* scale input = the crop rect */
			inb->format.height = ch;
			vpu->detile_nchunks = 1;
			mdp_config_compute_8195_scale(param,
					(struct img_config *)vpu->config);
			inb->format.width = sw;		/* restore the real buffer dims */
			inb->format.height = sh;
			dev_dbg(&mdp->pdev->dev,
				"kernel-direct origin-wide crop (mt8195, crop(0,0 %ux%u)->%ux%u via scale march)\n",
				cw, ch, ow, oh);
			/* KI-013 twin: a crop whose crop.area >= pp_criteria (1920*1080)
			 * trips mdp_check_pp_enable, so m2m set MDP_STREAM_TYPE_DUAL_BITBLT.
			 * This arm runs the scale march on ONE pipe (to 4096) and fills only
			 * cfgs[0]; pipe 1 would run an empty config -> 0-byte output. Force
			 * single pipe now that this crop arm has claimed the frame. */
			param->type = MDP_STREAM_TYPE_BITBLT;
			return 0;
		}
		if (CFG_CHECK(MT8195, mdp->mdp_data->mdp_plat_id) && gated &&
		    (pdown || pup || (pmixed && mdp_kd_mixed)) &&
		    ow <= 512 && oh <= 512 &&		/* single RSZ tile (output) */
		    (origin
			/* ORIGIN: no back-tap (SCP bypasses it); RSZ input = the crop
			 * width, bound to one line buffer. */
			? (cw <= 512)
			/* OFF-ORIGIN: the RDMA back-margin (mh+8 / mv+1) must fit
			 * inside the frame and the RSZ input (cw + back-tap) fit one
			 * line buffer. */
			: (cl >= mh + 8 && ct >= mv + 1 && cw + mh <= 512))) {
			vpu->detile_nchunks = 1;
			if (origin)
				mdp_config_compute_8195_crop_origin(param,
						(struct img_config *)vpu->config);
			else
				mdp_config_compute_8195_crop(param,
						(struct img_config *)vpu->config);
			dev_dbg(&mdp->pdev->dev,
				"kernel-direct crop config (mt8195, crop(%u,%u %ux%u) -> %ux%u %s%s)\n",
				cl, ct, cw, ch, ow, oh,
				origin ? "origin-" : "",
				is11 ? "1:1" : pmixed ? "mixed" : pup ? "up" : "down");
			/* KI-013 twin: single RSZ tile on ONE pipe -> force single pipe if
			 * DUAL was triggered by a large crop.area (else pipe 1 -> 0-byte). */
			param->type = MDP_STREAM_TYPE_BITBLT;
			return 0;
		}
		/* OFF-ORIGIN >512-WIDE crop: the RSZ-input span (cw + back-tap) exceeds one
		 * line buffer -> the multi-tile horizontal march + the crop back-tap on
		 * tile 0 + the origin phase fold (mdp_config_compute_8195_crop_wide). Pure
		 * up OR pure down, INCLUDING 1:1 (cw==ow: crop_wide computes the unity
		 * ctrl1 0x037b0003 + cx=0x8000 up-coeff, an at-phase pass-through RSZ,
		 * bit-exact vs the SCP off-origin 1:1). off-origin (cl>=mh+8, ct>=mv+1 so
		 * the back-margins fit the frame); single-tile (cw+mh<=512) already handled
		 * above. Mixed-wide stays SCP. */
		if (CFG_CHECK(MT8195, mdp->mdp_data->mdp_plat_id) && gated &&
		    !origin && (pdown || pup) && cw + mh > 512 &&
		    cl >= mh + 8 && ct >= mv + 1 &&
		    cw <= 4096 && ch <= 4096 && ow <= 4096 && oh <= 4096) {
			vpu->detile_nchunks = 1;
			mdp_config_compute_8195_crop_wide(param,
					(struct img_config *)vpu->config);
			dev_dbg(&mdp->pdev->dev,
				"kernel-direct crop config (mt8195, crop(%u,%u %ux%u) -> %ux%u wide-%s)\n",
				cl, ct, cw, ch, ow, oh, pup ? "up" : "down");
			/* KI-013 twin: multi-tile march on ONE pipe -> force single pipe if
			 * DUAL was triggered by a large crop.area (else pipe 1 -> 0-byte). */
			param->type = MDP_STREAM_TYPE_BITBLT;
			return 0;
		}
		dev_dbg(&mdp->pdev->dev,
			"crop(%u,%u %ux%u)->%ux%u unported (mixed/>tile/margin); SCP fallback\n",
			cl, ct, cw, ch, ow, oh);
	} else if (mdp_kernel_direct && mdp_frame_is_nv12_scale(param)) {
		/* Kernel-direct NV12->NV12 scale (resize) -- frees scale from the SCP
		 * (KI-009). The RSZ (c5) resizes in->out in ONE nsub=1 GCE submission.
		 *
		 * Scope (validated bit-exact vs the SCP golden, oracle-diff 2026-06-12):
		 * PURE up- OR down-scale within a SINGLE RSZ tile. `mdp_config_compute_
		 * 8195_scale` picks the RSZ mode by direction (down: control1 0x02310143,
		 * coeff CEIL((out-1)<<20/(in-1)); up: control1 0x037b0013, coeff
		 * ROUND((in-1)<<15/(out-1))). Two cases still fall through to the SCP:
		 *  - MIXED (one axis grows, the other shrinks): the SCP uses a 3rd RSZ
		 *    control not yet captured -- require pure up or pure down.
		 *  - input/output wider than one RSZ tile (~564 px) makes the SCP split
		 *    into nsub>1 horizontal tiles with per-tile phase accumulation (the
		 *    "tpipe" calculator, deliberately offloaded to the SCP). Until ported,
		 *    bound the line-buffer-limited dim <=512 (down: iw, up: ow).
		 * Tiling is horizontal-only (height never split); cap heights for sanity. */
		u32 iw = param->inputs[0].buffer.format.width;
		u32 ih = param->inputs[0].buffer.format.height;
		u32 ow = param->outputs[0].buffer.format.width;
		u32 oh = param->outputs[0].buffer.format.height;
		bool pdown = (ow <= iw && oh <= ih);	/* both axes shrink (or 1:1) */
		bool pup = (ow >= iw && oh >= ih);	/* both axes grow (or 1:1) */
		/* MIXED = strictly one axis up AND the other down (a true cross-direction
		 * resample; NOT a 1:1-on-one-axis case, which pdown/pup already cover). The
		 * per-dimension march handles it, gated default-OFF behind mdp_kd_mixed until
		 * cmp-fit-validated -- a mixed scale otherwise routes to the SCP as before. */
		bool pmixed = (ow > iw && oh < ih) || (ow < iw && oh > ih);
		u32 wmax = pdown ? iw : ow;		/* RSZ single-tile line-buffer limit */

		if (CFG_CHECK(MT8195, mdp->mdp_data->mdp_plat_id) &&
		    (pdown || pup || (pmixed && mdp_kd_mixed)) &&
		    /* DOWNSCALE: multi-tile (the tpipe march handles iw>512). UPSCALE:
		     * single RSZ tile (ow<=512) always kernel-direct; multi-tile (ow>512)
		     * is cmp-fit-validated kernel-direct and ON by default (mdp_kd_upscale,
		     * default 1; set 0 to force >512 up back to the SCP). MIXED: the wider
		     * dim drives the H tile march; bound both dims for sanity. Both
		     * height-untiled (cap for sanity). */
		    (pmixed ? (iw <= 4096 && ow <= 4096)
			    : pdown ? (iw <= 4096)
				    : (mdp_kd_upscale ? (ow <= 4096) : (wmax <= 512))) &&
		    ih <= 4096 && oh <= 4096) {
			vpu->detile_nchunks = 1;
			mdp_config_compute_8195_scale(param,
					(struct img_config *)vpu->config);
			dev_dbg(&mdp->pdev->dev,
				"kernel-direct scale config (mt8195, %ux%u->%ux%u %s, multi-tile)\n",
				iw, ih, ow, oh,
				pmixed ? "mixed" : pdown ? "down" : "up");
			/* KI-013: an input crop area >= pp_criteria (1920*1080) trips
			 * mdp_check_pp_enable, so m2m set param->type =
			 * MDP_STREAM_TYPE_DUAL_BITBLT and mdp_cmdq_send would submit TWO
			 * pixel pipes. But this KD multi-tile scale march marches
			 * internally on ONE pipe (up to 4096 wide) and fills only cfgs[0];
			 * pipe 1 would run an empty config -> 0-byte output. DUAL is
			 * unnecessary/harmful for a KD-handleable scale, so force single
			 * pipe (pp_used=1) now that this arm has claimed the frame. This
			 * override is scoped to exactly this claim -- blit/CSC/fmtconv/
			 * rotate/detile flow through other arms and keep their DUAL path. */
			param->type = MDP_STREAM_TYPE_BITBLT;
			return 0;
		}
		if (CFG_CHECK(MT8183, mdp->mdp_data->mdp_plat_id) &&
		    iw <= 4096 && ih <= 4096) {
			/* mt8183 kernel-direct SCALE -- down / up / MIXED (N1). The per-axis
			 * mdp_config_compute_8183_scale picks each axis's coeff + RSZ table/
			 * algo independently (down: CUB_ACC/table 17/CEIL shift-20; up: 6-tap/
			 * table 27/ROUND shift-15), all as phase-0 strips (the RSZ stalls on a
			 * non-zero phase). 2D OUTPUT tiles (<=254 WROT cap, input-bounded);
			 * tpc=1 per scaled strip; >tpc chunk via the shared multi-job endpoint.
			 * SAFETY GATE nh<=2: >2 HORIZONTAL tiles (output width > ~2x the per-
			 * tile cap) can stall the RSZ across a tile boundary for certain ratios
			 * (HW limit, 2026-06-20) and HANG the device; nh<=2 at any ratio + any
			 * height (vertical never tiles) is HW-validated, nh>=3 falls back to
			 * the SCP (graceful on mt8183 -- SCP dead, GCE never runs). */
			u32 nh, nv, tilew, tileh, ntiles, tpc;
			const char *dir = pup ? "up" : (pdown ? "down" : "mixed");
			/* STEEP (>=2:1, coeff_x<=0.5) wide downscale tiles cleanly across
			 * interior boundaries (HW-validated); allow it at nh>=3. */
			bool h_steep = pdown && (2 * ow <= iw + 1);

			mdp_scale_tiles_8183(iw, ih, ow, oh, &nh, &nv, &tilew, &tileh);
			if (nh <= 2 || h_steep || mdp_kd_nh3) {
				ntiles = nh * nv;
				tpc = mdp_scale_tiles_per_chunk_8183(param);
				vpu->detile_ntiles = ntiles;
				vpu->detile_nchunks = DIV_ROUND_UP(ntiles, tpc);
				mdp_config_compute_8183_scale(mdp, param,
					(struct img_config *)vpu->config, 0,
					min_t(u32, tpc, ntiles));
				dev_dbg(&mdp->pdev->dev,
					"kernel-direct scale config (mt8183, %ux%u->%ux%u %s, %u tiles, %u chunk(s))\n",
					iw, ih, ow, oh, dir, ntiles, vpu->detile_nchunks);
				return 0;
			}
			/* SHALLOW nh>=3 (coeff_x>0.5, output >508): HARDWARE WALL -- the RSZ
			 * cannot stream a shallow resample across an interior tile boundary
			 * (emit is rigidly tied to the contiguous PRZ_INPUT span; feeding the
			 * boundary filter extra margin corrupts the emit -- 2026-06-21 input-
			 * extension experiment). The SCP is dead on mt8183, so fail CLEAN with
			 * -EINVAL (no GCE submission -> no wedge, no no-output SCP fallback)
			 * instead of risking a device hang. Full output for shallow nh>=3 needs
			 * a transpose-decomposition (route via the no-stall vertical path). */
			dev_dbg(&mdp->pdev->dev,
				"mt8183 shallow nh>=3 %ux%u->%ux%u: -EINVAL (HW wall; SCP dead)\n",
				iw, ih, ow, oh);
			return -EINVAL;
		}
		dev_dbg(&mdp->pdev->dev,
			"mt8195/mt8183 scale %ux%u->%ux%u mixed/multi-tile/up; falling back to SCP\n",
			iw, ih, ow, oh);
	}
	/* Reached only when no kernel-direct branch claimed the frame: this job goes
	 * through the SCP. Clear kd_used so mdp_m2m_device_run suppresses the RSZ
	 * coeff-table override on the SCP-computed config (KD-only feature). */
	vpu->kd_used = false;
	/* PROVE-BEFORE-DELETE (mdp_scp_fallback_forbid): make this the single point
	 * where "no KD branch claimed the frame" is observable. A LOUD dev_WARN names
	 * the exact op so a proof sweep can confirm ZERO ops still need the SCP config
	 * IPI (or pinpoint the ones that do). Fail graceful -EINVAL, no SCP send. */
	if (mdp_scp_fallback_forbid) {
		dev_WARN(&mdp->pdev->dev,
			"mdp_scp_fallback_forbid: op hit SCP MDP-config fallback (NOT kernel-direct): in=%#x out=%#x %ux%u rot=%u flags=%#x type=%d\n",
			param->inputs[0].buffer.format.colorformat,
			param->outputs[0].buffer.format.colorformat,
			param->outputs[0].buffer.format.width,
			param->outputs[0].buffer.format.height,
			param->outputs[0].rotation,
			param->outputs[0].flags, param->type);
		return -EINVAL;
	}
	mutex_lock(&vpu->attach_lock);
	ret = vpu->scp ? 0 : mdp_vpu_scp_attach(vpu);
	if (!ret) {
		mutex_lock(vpu->lock);
		mdp_vpu_stage_frame(vpu, param, &addr);
		mutex_unlock(vpu->lock);
	}
	mutex_unlock(&vpu->attach_lock);
	if (ret)
		return ret;
	ret = mdp_vpu_sendmsg(vpu, SCP_IPI_MDP_FRAME, &addr, sizeof(addr));
	/* mt8188 geralt SCP emits compact comp ids: remap the returned config to
	 * the mt8195 numbering the path build expects (see the comment above). */
	if (!ret && mdp->mdp_data->scp_comp_id_compact)
		mdp_remap_scp_compact_ids(vpu);
	return ret;
}
