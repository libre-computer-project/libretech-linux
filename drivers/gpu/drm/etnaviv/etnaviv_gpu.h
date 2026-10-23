/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2015-2018 Etnaviv Project
 */

#ifndef __ETNAVIV_GPU_H__
#define __ETNAVIV_GPU_H__

#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include "etnaviv_cmdbuf.h"
#include "etnaviv_gem.h"
#include "etnaviv_mmu.h"
#include "etnaviv_drv.h"
#include "common.xml.h"
#include "state.xml.h"

struct etnaviv_gem_submit;
struct etnaviv_vram_mapping;

struct etnaviv_chip_identity {
	u32 model;
	u32 revision;
	u32 product_id;
	u32 customer_id;
	u32 eco_id;

	/* Supported feature fields. */
	u32 features;

	/* Supported minor feature fields. */
	u32 minor_features0;
	u32 minor_features1;
	u32 minor_features2;
	u32 minor_features3;
	u32 minor_features4;
	u32 minor_features5;
	u32 minor_features6;
	u32 minor_features7;
	u32 minor_features8;
	u32 minor_features9;
	u32 minor_features10;
	u32 minor_features11;

	/* Number of streams supported. */
	u32 stream_count;

	/* Total number of temporary registers per thread. */
	u32 register_max;

	/* Maximum number of threads. */
	u32 thread_count;

	/* Number of shader cores. */
	u32 shader_core_count;

	/* Number of Neural Network cores. */
	u32 nn_core_count;

	/* Number of MAD units per Neural Network core. */
	u32 nn_mad_per_core;

	/* Number of Tensor Processing cores. */
	u32 tp_core_count;

	/* Size in bytes of the SRAM inside the NPU. */
	u32 on_chip_sram_size;

	/* Size in bytes of the SRAM across the AXI bus. */
	u32 axi_sram_size;

	/* Size of the vertex cache. */
	u32 vertex_cache_size;

	/* Number of entries in the vertex output buffer. */
	u32 vertex_output_buffer_size;

	/* Number of pixel pipes. */
	u32 pixel_pipes;

	/* Number of instructions. */
	u32 instruction_count;

	/* Number of constants. */
	u32 num_constants;

	/* Buffer size */
	u32 buffer_size;

	/* Number of varyings */
	u8 varyings_count;
};

enum etnaviv_sec_mode {
	ETNA_SEC_NONE = 0,
	ETNA_SEC_KERNEL,
	ETNA_SEC_TZ
};

struct etnaviv_event {
	struct dma_fence *fence;
	struct etnaviv_gem_submit *submit;

	/*
	 * A311D (0x88): when set, the ISR skips the post-NN-AXI udelay drain
	 * for this event (set from ETNA_SUBMIT_NO_DRAIN on a throwaway submit
	 * whose output is never read back -- the mesa NN-priming prefix).
	 * Defaults false: event_alloc() memsets the whole struct to 0.
	 */
	bool no_drain;

	/*
	 * N9 (A311D 0x88): final-op output byte count of this submit, from
	 * drm_etnaviv_gem_submit.npu_output_bytes. The ISR scales the post-
	 * NN-AXI drain to it (drain grows ~linearly with output bytes). 0 for
	 * non-ML submits / old userspace -> floored to npu_drain_udelay.
	 */
	u32 npu_output_bytes;

	void (*sync_point)(struct etnaviv_gpu *gpu, struct etnaviv_event *event);
};

struct etnaviv_cmdbuf_suballoc;
struct regulator;
struct clk;
struct reset_control;

#define ETNA_NR_EVENTS 30

/*
 * NPU phase-latency ring (debugfs instrumentation, gated by the
 * npu_phase_trace module param -- zero overhead when disabled). Per NPU
 * submit we stamp ktime at four boundaries so a post-hoc decode can
 * separate FE-walk from NN-exec from the drain/signal tail WITHOUT the
 * dev_info-on-the-serial-console observer effect that masqueraded as
 * ~270 ms/invoke NPU latency (audit 2026-06-12-armed-runjob-localized-
 * console-spam.md). Derived (us): FE-walk = t_irq - t_submit;
 * NN-exec = t_idle - t_irq; drain = t_signal - t_idle;
 * total = t_signal - t_submit. On the 0x88 IRQ-direct path NN/TP are
 * already idle at the IRQ, so FE-walk and NN-exec are merged into
 * (t_irq - t_submit) and t_idle == t_irq; the 0x99 deferred-poll path
 * has a real intermediate t_idle so the two separate.
 */
#define ETNA_NPU_PHASE_RING 64
struct etnaviv_npu_phase {
	u64 seqno;        /* fence seqno (submit correlation) */
	ktime_t t_submit; /* FE kicked (end of etnaviv_gpu_submit) */
	ktime_t t_irq;    /* NPU event IRQ fired (FE done walking cmdbuf) */
	ktime_t t_idle;   /* NN+TP idle observed (poll path; == t_irq on 0x88) */
	ktime_t t_signal; /* dma_fence signaled */
	u32 poll_iters;   /* npu_poll_work iterations (0 on the 0x88 IRQ path) */
	u16 customer_id;
	u8 path;          /* 0 = IRQ-direct (0x88), 1 = deferred-poll (0x99) */
	u8 valid;
};

/*
 * qi:S2 -- A311D (0x88) SH-wedge fast poller + predicate instrumentation.
 *
 * The driver already holds a sound wedge discriminator in
 * etnaviv_sched_timedout_job(): "NN idle AND TP idle AND the FE DMA address
 * has not advanced". Its problem is not the predicate, it is the SAMPLING
 * RATE -- the predicate is only evaluated when the 2 s drm_sched timeout
 * expires, and only acted on at the SECOND expiry, so a condition that is
 * decidable in microseconds costs ~4.46 s (95.7 % pure timer latency).
 *
 * This poller samples the SAME predicate on an hrtimer and, when the full
 * signature has been continuously stable for npu_wedge_trip_k samples, calls
 * drm_sched_fault() to run the EXISTING timeout handler early. It never
 * decides to recover on its own: every recovery decision still goes through
 * etnaviv_sched_timedout_job(), whose three bail-outs (fence already
 * signalled / FE made forward progress / NN or TP busy) make a false positive
 * free. The 2 s timeout and the bounded re-arm backstop are untouched.
 *
 * Two streaks are tracked separately so the safety question -- "is NN+TP-idle
 * ever TRANSIENTLY true inside a healthy multi-op container?" -- is answered
 * by measurement rather than assumption:
 *   idle: consecutive samples with NN and TP both idle (ignores the FE)
 *   full: consecutive samples with NN+TP idle AND the FE DMA address
 *         unchanged AND no completed-fence progress (the wedge signature)
 * A healthy inter-op gap should show a short `idle` streak with a `full`
 * streak of ~0, because the FE keeps walking to the next descriptor.
 */
/*
 * Absolute wall-clock ceiling on the 0x88 nn_tp_busy re-arm chain. Matches the
 * pre-S2 count bound (30 x 2 s) that the comment in etnaviv_sched.c documents,
 * but expressed in time so it cannot be extended by TDR runs that restart the
 * drm_sched timer.
 */
#define ETNA_NPU_REARM_MAX_MS 60000

#define ETNA_WEDGE_HIST 20

struct etnaviv_npu_wedge {
	/* sampler state (survives a stats reset) */
	u32 last_dma;
	u32 last_fence;
	u32 streak_idle;
	u32 streak_full;
	bool armed;		/* npu_wedge_timer has been hrtimer_setup'ed */
	bool fault_pending;	/* a drm_sched_fault is queued, not yet handled */
	bool suppress;		/* the TDR declined our last fault -- stand down */
	bool tdr_active;	/* a TDR run is in flight -- do not trip */
	int reset_seen;
	/*
	 * qi:S2b -- the exact sample that tripped the last fast fault, so the
	 * TDR can report what the poller saw versus what it itself reads. The
	 * 7-of-12 `progress` declines cannot be attributed without this pair:
	 * "the FE moved" is only meaningful against a known observation time.
	 */
	u32 trip_dma;
	u32 trip_fence;
	u32 trip_streak;
	ktime_t trip_at;
	/* everything below is zeroed when npu_wedge_reset changes */
	struct {
		u64 samples;
		u64 samples_idle;
		u64 samples_full;
		u64 faults;
		u64 faults_declined;
		/*
		 * Split of faults_declined by the TDR branch that refused the
		 * fault, so "the fast path is ineffective" can be attributed
		 * rather than guessed:
		 *   _progress: the FE moved between our sample and the TDR run
		 *   _busy:     NN or TP had become busy by the TDR run
		 */
		u64 faults_declined_progress;
		u64 faults_declined_busy;
		/*
		 * qi:S2b -- the `progress` branch is an OR of two independent
		 * tests. Split it, plus the previously-uncounted "the job's
		 * fence was already signalled" bail-out.
		 */
		u64 faults_declined_fence;
		u64 faults_declined_dma;
		u64 faults_declined_signalled;
		/*
		 * qi:S2b -- a fast fault whose signature was no longer live by
		 * the time the TDR ran (the fault was deferred, typically by a
		 * concurrent recovery's drm_sched_stop/start).
		 */
		u64 faults_stale;
		u32 max_idle;
		u32 max_full;
		/* log2 histograms of COMPLETED streak lengths, in samples:
		 * bucket b holds streaks in [2^(b-1), 2^b - 1]. */
		u32 hist_idle[ETNA_WEDGE_HIST];
		u32 hist_full[ETNA_WEDGE_HIST];
	} s;
};

enum etnaviv_gpu_state {
	ETNA_GPU_STATE_UNKNOWN = 0,
	ETNA_GPU_STATE_IDENTIFIED,
	ETNA_GPU_STATE_RESET,
	ETNA_GPU_STATE_INITIALIZED,
	ETNA_GPU_STATE_RUNNING,
	ETNA_GPU_STATE_FAULT,
};

struct etnaviv_gpu {
	struct drm_device *drm;
	struct thermal_cooling_device *cooling;
	struct device *dev;
	struct mutex lock;
	struct etnaviv_chip_identity identity;
	enum etnaviv_sec_mode sec_mode;
	struct workqueue_struct *wq;
	struct mutex sched_lock;
	struct drm_gpu_scheduler sched;
	enum etnaviv_gpu_state state;

	/* 'ring'-buffer: */
	struct etnaviv_cmdbuf buffer;
	/* NN/TP flop reset data (instruction/kernel/input/output) */
	void *nn_data_vaddr;
	dma_addr_t nn_data_paddr;
#define ETNAVIV_FLOP_DATA_SIZE	(2 * PAGE_SIZE)
	/*
	 * Persistent drm_mm nodes for the NN/TP flop reset VA reservations.
	 * These MUST live in the gpu struct (not on the stack): the IOMMU
	 * mappings are kept alive for the driver's lifetime because the
	 * NN/TP engines retain register state referencing these VAs, so
	 * the drm_mm_node that owns the VA range cannot be released.
	 */
	struct drm_mm_node nn_flop_node;
	struct drm_mm_node tp_flop_node;
	/*
	 * The MMU contexts whose drm_mm host nn_flop_node / tp_flop_node.
	 * The nodes are GPU-lifetime but the context they were inserted into
	 * is a CLIENT context (whichever one was current at the first
	 * idleloop), so the GPU holds a reference on it for as long as the
	 * node is in its allocator and releases node, mapping and reference
	 * together in etnaviv_gpu_unbind().  Without the reference the
	 * context is freed under the node -- at etnaviv_postclose(), at a
	 * context switch, or at unload -- and drm_mm_takedown() WARNs
	 * "Memory manager not clean during takedown" (TAINT_WARN).
	 */
	struct etnaviv_iommu_context *nn_flop_ctx;
	struct etnaviv_iommu_context *tp_flop_ctx;
	/*
	 * A311D (0x88) per-submit flop-reset: the NN_INST VA of the flop-reset
	 * op, captured during the one-time init flop-reset. Stage 2 emits this
	 * op inline in the ring before each NN submit (etnaviv_buffer_v8_qi.c)
	 * to clear the preceding submit's cross-partition NN-engine state.
	 * Nonzero only after a successful init flop-reset on 0x88.
	 */
	u32 nn_flop_inst_va;
	int exec_state;

	/* event management: */
	DECLARE_BITMAP(event_bitmap, ETNA_NR_EVENTS);
	struct etnaviv_event event[ETNA_NR_EVENTS];
	struct completion event_free;
	spinlock_t event_spinlock;

	u32 idle_mask;

	/* Fencing support */
	struct xarray user_fences;
	u32 next_user_fence;
	u32 next_fence;
	u32 completed_fence;
	wait_queue_head_t fence_event;
	u64 fence_context;
	spinlock_t fence_spinlock;

	/* worker for handling 'sync' points: */
	struct work_struct sync_point_work;
	int sync_point_event;

	/* hang detection */
	u32 hangcheck_dma_addr;
	u32 hangcheck_primid;
	u32 hangcheck_fence;
	/* A311D (0x88) N2 fast-recover: count consecutive sched-timeout re-arms
	 * of a legitimately-running NN/TP op (FE stalled at NN_INST, no fence
	 * progress) so a genuine engine-stuck hang still recovers after a bounded
	 * window (~60 s) while the SH-wedge recovers fast. Reset on progress. */
	unsigned npu_rearm_count;
	/*
	 * qi:S2 wall-clock twin of npu_rearm_count. The count alone bounds the
	 * backstop at 30 x 2 s ONLY while every re-arm is driven by a real
	 * drm_sched timeout expiry. The S2 fast wedge poller calls
	 * drm_sched_fault(), and each TDR run that returns NOMINAL RESTARTS the
	 * 2 s timer -- so a stream of fast faults could re-arm forever without
	 * ever incrementing the count, defeating the ~60 s absolute backstop.
	 * Stamping the first re-arm since the last forward progress makes the
	 * backstop time-based and immune to timer restarts. 0 = not re-arming.
	 */
	ktime_t npu_rearm_start;
	struct etnaviv_npu_wedge npu_wedge;
	struct hrtimer npu_wedge_timer;

	void __iomem *mmio;
	int irq;

	struct etnaviv_iommu_context *mmu_context;
	unsigned int flush_seq;

	/* Power Control: */
	struct clk *clk_bus;
	struct clk *clk_reg;
	struct clk *clk_core;
	struct clk *clk_shader;
	struct reset_control *rst;

	unsigned int freq_scale;
	unsigned int fe_waitcycles;
	unsigned long base_rate_core;
	unsigned long base_rate_shader;

	/* NPU polling completion for chips without SMALL_BATCH support.
	 * FE cannot write GL registers (like GL_EVENT) when SH is busy
	 * from NN/TP operations. Instead, FE stops via END command and
	 * a CPU-side work item polls for NN/TP idle then fires the event.
	 */
	bool npu_poll_completion;
	struct delayed_work npu_poll_work;
	int npu_poll_event;

	/*
	 * N9 deferred size-scaled NN-AXI drain (A311D 0x88). When the ISR's
	 * output-scaled drain exceeds npu_drain_hardirq_max_us it defers the
	 * fence signal to npu_drain_work, which usleep_range()s the budget
	 * (jobs are serialized on 0x88 so one in-flight slot suffices).
	 * SEPARATE from npu_poll_work to keep the 0x99 poll path untouched.
	 */
	struct delayed_work npu_drain_work;
	int npu_drain_event;
	struct dma_fence *npu_drain_fence;
	u64 npu_drain_ns;
	ktime_t npu_drain_t_irq;

	/* NPU phase-latency ring (instrumentation; see npu_phase_trace) */
	struct etnaviv_npu_phase npu_phase[ETNA_NPU_PHASE_RING];
	u32 npu_phase_head;
	int npu_phase_slot[ETNA_NR_EVENTS]; /* event -> ring slot, -1 = none */
	spinlock_t npu_phase_lock;
};

extern int etnaviv_dump_cmdbuf;

static inline void gpu_write(struct etnaviv_gpu *gpu, u32 reg, u32 data)
{
	writel(data, gpu->mmio + reg);
}

static inline u32 gpu_read(struct etnaviv_gpu *gpu, u32 reg)
{
	/* On some variants, such as the GC7000r6009, some FE registers
	 * need two reads to be consistent. Do that extra read here and
	 * throw away the result.
	 */
	if (reg >= VIVS_FE_DMA_STATUS && reg <= VIVS_FE_AUTO_FLUSH)
		readl(gpu->mmio + reg);

	return readl(gpu->mmio + reg);
}

static inline u32 gpu_fix_power_address(struct etnaviv_gpu *gpu, u32 reg)
{
	/* Power registers in GC300 < 2.0 are offset by 0x100 */
	if (gpu->identity.model == chipModel_GC300 &&
	    gpu->identity.revision < 0x2000)
		reg += 0x100;

	return reg;
}

static inline void gpu_write_power(struct etnaviv_gpu *gpu, u32 reg, u32 data)
{
	writel(data, gpu->mmio + gpu_fix_power_address(gpu, reg));
}

static inline u32 gpu_read_power(struct etnaviv_gpu *gpu, u32 reg)
{
	return readl(gpu->mmio + gpu_fix_power_address(gpu, reg));
}

int etnaviv_gpu_get_param(struct etnaviv_gpu *gpu, u32 param, u64 *value);

int etnaviv_gpu_init(struct etnaviv_gpu *gpu);
bool etnaviv_fill_identity_from_hwdb(struct etnaviv_gpu *gpu);

#ifdef CONFIG_DEBUG_FS
int etnaviv_gpu_debugfs(struct etnaviv_gpu *gpu, struct seq_file *m);
#endif

void etnaviv_gpu_recover_hang(struct etnaviv_gem_submit *submit, u32 dma_addr);

/* qi:S2b -- wedge-poller state queried by the TDR. See etnaviv_gpu.c. */
bool etnaviv_npu_wedge_live(struct etnaviv_gpu *gpu);
bool etnaviv_npu_wedge_verbose(void);
void etnaviv_npu_wedge_reseed(struct etnaviv_gpu *gpu);
/* qi:N1488 -- runtime-PM hold across the recovery path. See etnaviv_gpu.c. */
bool etnaviv_npu_recover_pm_get(struct etnaviv_gpu *gpu);
void etnaviv_npu_recover_pm_put(struct etnaviv_gpu *gpu, bool held);
void etnaviv_gpu_retire(struct etnaviv_gpu *gpu);
int etnaviv_gpu_wait_fence_interruptible(struct etnaviv_gpu *gpu,
	u32 fence, struct drm_etnaviv_timespec *timeout);
int etnaviv_gpu_wait_obj_inactive(struct etnaviv_gpu *gpu,
	struct etnaviv_gem_object *etnaviv_obj,
	struct drm_etnaviv_timespec *timeout);
struct dma_fence *etnaviv_gpu_submit(struct etnaviv_gem_submit *submit);
int etnaviv_gpu_pm_get_sync(struct etnaviv_gpu *gpu);
void etnaviv_gpu_pm_put(struct etnaviv_gpu *gpu);
int etnaviv_gpu_wait_idle(struct etnaviv_gpu *gpu, unsigned int timeout_ms);
void etnaviv_gpu_start_fe(struct etnaviv_gpu *gpu, u32 address, u16 prefetch);

/* qi:N1443 -- VIVS_GL_NN_CONFIG.DDR_BURST_SIZE override (npu_nn_ddr_burst). */
u32 etnaviv_npu_nn_config(struct etnaviv_gpu *gpu, u32 val);
void etnaviv_npu_nn_config_note_prelude(u32 val);
void etnaviv_npu_nn_config_patch_stream(struct etnaviv_gpu *gpu, u32 *stream,
					unsigned int size);

extern struct platform_driver etnaviv_gpu_driver;

#endif /* __ETNAVIV_GPU_H__ */
