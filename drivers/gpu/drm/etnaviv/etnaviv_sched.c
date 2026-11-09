// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2017 Etnaviv Project
 */

#include <linux/moduleparam.h>
#include <linux/workqueue.h>

#include "etnaviv_cmdbuf.h"
#include "etnaviv_drv.h"
#include "etnaviv_dump.h"
#include "etnaviv_gem.h"
#include "etnaviv_gpu.h"
#include "etnaviv_mmu.h"
#include "etnaviv_sched.h"
#include "state.xml.h"
#include "state_hi.xml.h"

static int etnaviv_job_hang_limit = 0;
module_param_named(job_hang_limit, etnaviv_job_hang_limit, int , 0444);
static int etnaviv_hw_jobs_limit = 4;
module_param_named(hw_job_limit, etnaviv_hw_jobs_limit, int , 0444);

static struct dma_fence *etnaviv_sched_run_job(struct drm_sched_job *sched_job)
{
	struct etnaviv_gem_submit *submit = to_etnaviv_submit(sched_job);
	struct dma_fence *fence = NULL;

	if (likely(!sched_job->s_fence->finished.error))
		fence = etnaviv_gpu_submit(submit);
	else
		dev_dbg(submit->gpu->dev, "skipping bad job\n");

	return fence;
}

static enum drm_gpu_sched_stat etnaviv_sched_timedout_job(struct drm_sched_job
							  *sched_job)
{
	struct etnaviv_gem_submit *submit = to_etnaviv_submit(sched_job);
	struct etnaviv_gpu *gpu = submit->gpu;
	u32 dma_addr, primid = 0;
	int change;
	bool fast, wedge_live;
	/* qi:N1488 -- true only when a runtime-PM reference was actually taken. */
	bool pm_held = false;
	/* qi:S2b decline instrumentation */
	u32 dma_addr2, idle_now;
	u64 job_seqno;
	s64 trip_age_us;

	/*
	 * qi:S2: did the fast wedge poller ask for this run, rather than the
	 * 2 s drm_sched timeout expiring? A fast fault is a SAMPLE, not a
	 * timeout -- see the two `fast` uses below for why that distinction is
	 * load-bearing for safety. Consume the flag on every path.
	 */
	fast = gpu->npu_wedge.fault_pending;
	gpu->npu_wedge.fault_pending = false;
	/*
	 * qi:S2b: hold the poller off for the duration of this run, so it
	 * cannot rewrite the baseline we are about to test against, and cannot
	 * leave a fault latched that the recovery below will cancel. Cleared on
	 * every exit path.
	 */
	gpu->npu_wedge.tdr_active = true;

	/*
	 * qi:S2b: is the wedge signature still live, as of the poller's last
	 * sample 200 us ago? A fast fault can reach us seconds after it was
	 * raised (a recovery in flight cancels the queued work), by which time
	 * the snapshot it handed us is stale and the FE has legitimately moved
	 * -- the measured cause of the v6 `progress` declines, 7 of 12 faults.
	 * Asking the poller for the LIVE streak is the same question without
	 * the staleness, so a live signature licenses skipping the snapshot
	 * test below. 0x88-gated; every other chip keeps the stock path.
	 */
	wedge_live = fast && gpu->identity.customer_id == 0x88 &&
		     etnaviv_npu_wedge_live(gpu);
	if (fast && !wedge_live)
		gpu->npu_wedge.s.faults_stale++;

	/*
	 * qi:S2b -- report what the poller saw versus what we read, at every
	 * fast TDR run. The v6 attribution said 7 of 12 fast faults are
	 * declined for `progress`; it could not say WHICH quantity moved, by
	 * how much, or whether the job under examination is even the wedged
	 * one. Both DMA reads are taken here, before any other MMIO, so the
	 * pair brackets this handler's own execution.
	 */
	job_seqno = submit->out_fence ? submit->out_fence->seqno : 0;
	if (fast && etnaviv_npu_wedge_verbose()) {
		dma_addr2 = gpu_read(gpu, VIVS_FE_DMA_ADDRESS);
		idle_now = gpu_read(gpu, VIVS_HI_IDLE_STATE);
		trip_age_us = ktime_us_delta(ktime_get(),
					     gpu->npu_wedge.trip_at);
		dev_warn(gpu->dev,
			 "S2b fast-TDR: age=%lldus streak=%u live=%d | dma trip=0x%08x now=0x%08x hc=0x%08x | fence trip=%u completed=%u hc=%u job_seqno=%llu | idle=0x%08x state=%d\n",
			 trip_age_us, gpu->npu_wedge.trip_streak, wedge_live,
			 gpu->npu_wedge.trip_dma, dma_addr2,
			 gpu->hangcheck_dma_addr,
			 gpu->npu_wedge.trip_fence, gpu->completed_fence,
			 gpu->hangcheck_fence, job_seqno,
			 idle_now, gpu->state);
	}

	/*
	 * If the GPU managed to complete this jobs fence, the timeout has
	 * fired before free-job worker. The timeout is spurious, so bail out.
	 */
	if (dma_fence_is_signaled(submit->out_fence)) {
		if (fast) {
			gpu->npu_wedge.s.faults_declined++;
			gpu->npu_wedge.s.faults_declined_signalled++;
			dev_warn(gpu->dev, "S2b decline: branch=signalled\n");
		}
		goto out_no_timeout;
	}

	/*
	 * If the GPU is still making forward progress on the front-end (which
	 * should never loop) we shift out the timeout to give it a chance to
	 * finish the job.
	 */
	dma_addr = gpu_read(gpu, VIVS_FE_DMA_ADDRESS);
	change = dma_addr - gpu->hangcheck_dma_addr;
	if (submit->exec_state == ETNA_PIPE_3D) {
		/* guard against concurrent usage from perfmon_sample */
		mutex_lock(&gpu->lock);
		gpu_write(gpu, VIVS_MC_PROFILE_CONFIG0,
			  VIVS_MC_PROFILE_CONFIG0_FE_CURRENT_PRIM <<
			  VIVS_MC_PROFILE_CONFIG0_FE__SHIFT);
		primid = gpu_read(gpu, VIVS_MC_PROFILE_FE_READ);
		mutex_unlock(&gpu->lock);
	}
	if (!wedge_live && gpu->state == ETNA_GPU_STATE_RUNNING &&
	    (gpu->completed_fence != gpu->hangcheck_fence ||
	     change < 0 || change > 16 ||
	     (submit->exec_state == ETNA_PIPE_3D &&
	      gpu->hangcheck_primid != primid))) {
		gpu->hangcheck_dma_addr = dma_addr;
		gpu->hangcheck_primid = primid;
		gpu->hangcheck_fence = gpu->completed_fence;
		gpu->npu_rearm_count = 0; /* forward progress -> reset N2 re-arm bound */
		gpu->npu_rearm_start = 0; /* ... and its wall-clock twin (qi:S2) */
		if (fast) {
			/* The FE moved: our fault was a false positive. */
			gpu->npu_wedge.s.faults_declined++;
			gpu->npu_wedge.s.faults_declined_progress++;
			if (gpu->completed_fence != gpu->hangcheck_fence)
				gpu->npu_wedge.s.faults_declined_fence++;
			else
				gpu->npu_wedge.s.faults_declined_dma++;
			dev_warn(gpu->dev,
				 "S2b decline: branch=progress sub=%s change=%d fence_delta=%d\n",
				 gpu->completed_fence != gpu->hangcheck_fence ?
					"fence" : "dma",
				 change,
				 (int)(gpu->completed_fence -
				       gpu->hangcheck_fence));
			/*
			 * qi:S2b -- do NOT latch suppress here. Reaching this
			 * branch as a fast fault now means the signature was
			 * not live when we ran, i.e. the fault was STALE (the
			 * poller's own live-streak test above said so). A
			 * stale fault is evidence about fault DELIVERY, not
			 * about the predicate, and disarming the fast path on
			 * it costs the whole rest of the wedge -- measured at
			 * 7 of 12 faults in v6. The genuine false positive is
			 * the `busy` branch, which still latches.
			 */
		}
		goto out_no_timeout;
	}

	/*
	 * A311D (0x88) N2 fast wedge-detect + recover (Candidate B). The 0x88
	 * SH-wedge (idle=0x7ffffff6 = FE+SH busy, NN+TP idle, dma frozen) cannot
	 * be PREVENTED -- the wedged SH never drains without a reset (7 fixes
	 * falsified, per the stepback NN-flush-redesign audit). The GPU
	 * recover IS the only clear and it produces CORRECT output. So recover
	 * FAST on the wedge signature instead of waiting the full timeout. To
	 * avoid the Layer-2 panic (a short timeout false-recovering slow-but-
	 * completing ops -> accumulated recovers -> hung_task_panic), recover
	 * ONLY when NN AND TP are idle (the wedge); if either engine is BUSY the
	 * op is legitimately running (FE stalls at NN_INST, dma frozen) -> re-arm.
	 * Bounded re-arm preserves a ~60 s absolute backstop (30 x 2 s) for any
	 * genuine engine-stuck hang. 0x88-gated; 0x99 keeps its deferred-poll
	 * re-arm below (this branch is skipped for it).
	 */
	if (gpu->state == ETNA_GPU_STATE_RUNNING &&
	    gpu->identity.customer_id == 0x88) {
		u32 idle = gpu_read(gpu, VIVS_HI_IDLE_STATE);
		bool nn_tp_busy = !(idle & VIVS_HI_IDLE_STATE_NN) ||
				  !(idle & VIVS_HI_IDLE_STATE_TP);

		if (nn_tp_busy) {
			/*
			 * qi:S2 -- TWO things change here, both to keep the
			 * ~60 s backstop intact once the fast poller can call
			 * drm_sched_fault():
			 *
			 * 1. A fast fault does NOT consume the re-arm budget.
			 *    NN or TP is busy, so the op is legitimately
			 *    running and our sample was simply premature;
			 *    charging it to the 30-count would let a burst of
			 *    false positives reset a slow-but-healthy op out
			 *    from under itself -- exactly the Layer-2 panic
			 *    the 2 s value was chosen to avoid.
			 * 2. The backstop gains a WALL-CLOCK bound. Every TDR
			 *    run that returns NOMINAL restarts the 2 s timer,
			 *    so a stream of fast faults could re-arm forever
			 *    while npu_rearm_count stood still -- silently
			 *    deleting the absolute backstop. Stamping the
			 *    first re-arm since the last forward progress
			 *    makes the bound immune to timer restarts.
			 *
			 * Either bound alone is sufficient to recover; a
			 * genuine engine-stuck hang still exits within ~60 s.
			 */
			if (!gpu->npu_rearm_start)
				gpu->npu_rearm_start = ktime_get();
			if (!fast)
				gpu->npu_rearm_count++;
			else {
				gpu->npu_wedge.s.faults_declined++;
				gpu->npu_wedge.s.faults_declined_busy++;
				dev_warn(gpu->dev,
					 "S2b decline: branch=busy idle=0x%08x\n",
					 idle);
			}

			if (gpu->npu_rearm_count < 30 &&
			    ktime_ms_delta(ktime_get(),
					   gpu->npu_rearm_start) <
					   ETNA_NPU_REARM_MAX_MS) {
				gpu->hangcheck_dma_addr = dma_addr;
				gpu->hangcheck_fence = gpu->completed_fence;
				/*
				 * The TDR declined our fault. Stand the fast
				 * path down until the signature clears, so a
				 * persistent false positive costs exactly ONE
				 * spurious TDR run per stalled interval rather
				 * than one every npu_wedge_poll_us.
				 */
				if (fast)
					gpu->npu_wedge.suppress = true;
				goto out_no_timeout;
			}
		}
		dev_warn(gpu->dev,
			 "N2 fast-recover: A311D wedge idle=0x%08x rearm=%u held=%lldms path=%s dma=0x%08x -> recover\n",
			 idle, gpu->npu_rearm_count,
			 gpu->npu_rearm_start ?
				ktime_ms_delta(ktime_get(),
					       gpu->npu_rearm_start) : 0,
			 fast ? "poll" : "timeout", dma_addr);
		gpu->npu_rearm_count = 0;
		gpu->npu_rearm_start = 0;
		gpu->npu_wedge.suppress = false;
		/* fall through to GPU recovery (resets + resubmits; output correct) */
	}

	/*
	 * VIPPico v3 (S905D3, customer 0x99) deferred-completion re-arm.
	 *
	 * On NPU chips that use npu_poll_completion (GL_EVENT fires from FE
	 * before NN/TP finish; fence signaling is deferred to npu_poll_work
	 * which polls VIVS_HI_IDLE_STATE_NN/_TP and signals), the FE
	 * legitimately parks at WAIT+LINK in the kernel ring buffer for the
	 * entire duration of NN/TP execution. dma_addr stays in the ring's
	 * 8-byte WAIT/LINK loop -- |change| <= 4 -- which the forward-
	 * progress check above classifies as a hang.
	 *
	 * For models where NN/TP execution + npu_poll_work latency exceed the
	 * effective drm_sched timeout (which the nn_core_count=0 race at
	 * sched_init time pins to the non-NPU 500 ms default), this triggers
	 * a watchdog false-positive on /019 (movenetlightning single-op,
	 * ~640 ms total) and similar long-running ops. The recover_hang path
	 * cascades on customer 0x99 into a kernel WARN at
	 * etnaviv_gem_cpu_fini+0x88, then the genpd autosuspend timer drops
	 * the NNA power domain via meson-ee-pwrc, then the in-flight teflon
	 * userspace stuck on a never-signaled fence under hung_task_panic=1
	 * triggers panic=30 reboot.
	 *
	 * The disease is "recover hangs that aren't hangs". Detect the
	 * deferred-pending state precisely:
	 *   1. npu_poll_completion enabled (only customer 0x99 today)
	 *   2. an event is still pending fence signal from npu_poll_work
	 *      (npu_poll_event >= 0; cleared to -1 once poll_work signals)
	 *   3. dma_addr is inside the kernel ring buffer (FE has returned
	 *      from the user cmdbuf to the WAIT+LINK loop; it has executed
	 *      the GL_EVENT and is waiting)
	 *   4. NN and TP idle bits set in VIVS_HI_IDLE_STATE (idle = 1
	 *      convention; bits 17 and 18). Both set means the engines are
	 *      idle -- either work is done (poll_work will signal soon) or
	 *      never started (a different bug). Either way, this is not an
	 *      FE-side hang -- the FE is correctly parked.
	 *   5. npu_poll_work is actually queued or running (work_busy).
	 *      GL_EVENT fires npu_poll_work; if FE is stuck before GL_EVENT,
	 *      poll_work was never scheduled and will never clear npu_poll_event.
	 *      In that case, all four conditions above would remain true
	 *      forever -- the original re-arm-unconditionally path created an
	 *      infinite-liveness bug (NEXT-37-LB1: 537 s, 2117 re-arms,
	 *      zero GPU recovery observed on c64a48092a00 2026-05-28).
	 *
	 * Re-arm only when condition 5 is met. When poll_work is not running
	 * and npu_poll_event is still set, GL_EVENT did not fire: the FE is
	 * genuinely stuck and GPU recovery must run.
	 *
	 * A311D 0x88 has npu_poll_completion=false; it falls through this
	 * block unchanged and continues to recover_hang on real timeouts as
	 * today. No timing change for either chip; only the recover decision
	 * changes for customer 0x99.
	 */
	if (gpu->npu_poll_completion &&
	    gpu->npu_poll_event >= 0 &&
	    gpu->buffer.vaddr && gpu->buffer.size && gpu->mmu_context) {
		u32 ring_va = etnaviv_cmdbuf_get_va(&gpu->buffer,
				&gpu->mmu_context->cmdbuf_mapping);
		u32 idle = gpu_read(gpu, VIVS_HI_IDLE_STATE);

		if (dma_addr >= ring_va &&
		    dma_addr < ring_va + gpu->buffer.size &&
		    (idle & VIVS_HI_IDLE_STATE_NN) &&
		    (idle & VIVS_HI_IDLE_STATE_TP)) {
			if (work_busy(&gpu->npu_poll_work.work)) {
				/* GL_EVENT fired; poll_work in flight -- re-arm */
				dev_dbg(gpu->dev,
					"sched timeout re-armed: NPU deferred-completion pending (dma=0x%08x ring=0x%08x idle=0x%08x event=%d)\n",
					dma_addr, ring_va, idle, gpu->npu_poll_event);
				gpu->hangcheck_dma_addr = dma_addr;
				gpu->hangcheck_fence = gpu->completed_fence;
				goto out_no_timeout;
			}
			/* poll_work not running + npu_poll_event still set:
			 * GL_EVENT did not fire; FE stuck before completion
			 * event. Fall through to GPU recovery. */
			dev_warn(gpu->dev,
				 "GL_EVENT not fired: recovering hung FE (dma=0x%08x ring=0x%08x idle=0x%08x event=%d)\n",
				 dma_addr, ring_va, idle, gpu->npu_poll_event);
		}
	}

	/*
	 * qi:N1488 -- take the runtime-PM reference BEFORE anything in the
	 * recovery path touches the device, and release it only after the
	 * scheduler has been restarted.  Default 0 = no reference, no print,
	 * shipped behaviour.  See npu_recover_pm_hold in etnaviv_gpu.c for why
	 * this is the layer the SError lives at.
	 */
	pm_held = etnaviv_npu_recover_pm_get(gpu);

	/* block scheduler */
	drm_sched_stop(&gpu->sched, sched_job);

	if(sched_job)
		drm_sched_increase_karma(sched_job);

	/* get the GPU back into the init state */
	etnaviv_core_dump(submit);
	/*
	 * qi:N1453: hand over the FE DMA address this function already read
	 * above, so recover_hang() needs no device access of its own before
	 * the reset. See npu_recover_predump in etnaviv_gpu.c.
	 */
	etnaviv_gpu_recover_hang(submit, dma_addr);

	drm_sched_resubmit_jobs(&gpu->sched);

	drm_sched_start(&gpu->sched, 0);
	/*
	 * qi:S2b -- the engines were just reset and the jobs resubmitted, so
	 * any streak the poller accumulated against the pre-reset state is
	 * meaningless. Start its observation over, then let it trip again.
	 */
	etnaviv_npu_wedge_reseed(gpu);
	gpu->npu_wedge.tdr_active = false;
	/* qi:N1488 -- the recovery path is over; let autosuspend resume. */
	etnaviv_npu_recover_pm_put(gpu, pm_held);
	return DRM_GPU_SCHED_STAT_RESET;

out_no_timeout:
	/*
	 * v6.18: DRM_GPU_SCHED_STAT_NOMINAL was split into _RESET (the job was
	 * reset) and _NO_HANG (no hang happened -- the scheduler restarts the
	 * timer and does NOT touch the job list).  The 6.12 code open-coded the
	 * _NO_HANG semantics by re-adding the job to sched->pending_list under
	 * job_list_lock; drm_sched_job_timedout() now does that for us when we
	 * return _NO_HANG, so the manual re-add would double-add.
	 */
	gpu->npu_wedge.tdr_active = false;
	return DRM_GPU_SCHED_STAT_NO_HANG;
}

static void etnaviv_sched_free_job(struct drm_sched_job *sched_job)
{
	struct etnaviv_gem_submit *submit = to_etnaviv_submit(sched_job);

	drm_sched_job_cleanup(sched_job);

	etnaviv_submit_put(submit);
}

static const struct drm_sched_backend_ops etnaviv_sched_ops = {
	.run_job = etnaviv_sched_run_job,
	.timedout_job = etnaviv_sched_timedout_job,
	.free_job = etnaviv_sched_free_job,
};

int etnaviv_sched_push_job(struct etnaviv_gem_submit *submit)
{
	struct etnaviv_gpu *gpu = submit->gpu;
	int ret;

	/*
	 * Hold the sched lock across the whole operation to avoid jobs being
	 * pushed out of order with regard to their sched fence seqnos as
	 * allocated in drm_sched_job_arm.
	 */
	mutex_lock(&gpu->sched_lock);

	drm_sched_job_arm(&submit->sched_job);

	submit->out_fence = dma_fence_get(&submit->sched_job.s_fence->finished);
	ret = xa_alloc_cyclic(&gpu->user_fences, &submit->out_fence_id,
			      submit->out_fence, xa_limit_32b,
			      &gpu->next_user_fence, GFP_KERNEL);
	if (ret < 0) {
		drm_sched_job_cleanup(&submit->sched_job);
		goto out_unlock;
	}

	/* the scheduler holds on to the job now */
	kref_get(&submit->refcount);

	drm_sched_entity_push_job(&submit->sched_job);

out_unlock:
	mutex_unlock(&gpu->sched_lock);

	return ret;
}

int etnaviv_sched_init(struct etnaviv_gpu *gpu)
{
	/*
	 * NPU ops can take >500ms per layer (e.g. 224x224 conv on 2-core
	 * VIPPico). Use a longer timeout for the NPU to avoid false hang
	 * detection during kernel-side NN/TP idle polling + FLUSH_CACHE.
	 *
	 * NPU poll completion also requires strictly sequential execution:
	 * after each NN/TP operation the CPU must poll for idle and issue
	 * FLUSH_CACHE before the next operation can start.  Limit the credit
	 * budget to 1 in-flight job so the ring buffer isn't overwritten.
	 *
	 * v6.18: drm_sched_init() takes a struct drm_sched_init_args instead
	 * of the long positional argument list; the old `credits`/`timeout`
	 * positions map to .credit_limit/.timeout.
	 */
	const struct drm_sched_init_args args = {
		.ops = &etnaviv_sched_ops,
		.num_rqs = DRM_SCHED_PRIORITY_COUNT,
		.credit_limit = gpu->identity.nn_core_count > 0 ?
				1 : etnaviv_hw_jobs_limit,
		.hang_limit = etnaviv_job_hang_limit,
		.timeout = msecs_to_jiffies(gpu->identity.nn_core_count > 0 ?
					    60000 : 500),
		.name = dev_name(gpu->dev),
		.dev = gpu->dev,
	};

	return drm_sched_init(&gpu->sched, &args);
}

void etnaviv_sched_fini(struct etnaviv_gpu *gpu)
{
	drm_sched_fini(&gpu->sched);
}
