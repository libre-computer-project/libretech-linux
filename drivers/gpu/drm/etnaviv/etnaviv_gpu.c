// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2015-2018 Etnaviv Project
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/dma-fence.h>
#include <linux/dma-mapping.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/minmax.h>
#include <linux/pm_runtime.h>
#include <linux/ratelimit.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/thermal.h>

#include "etnaviv_cmdbuf.h"
#include "etnaviv_dump.h"
#include "etnaviv_gpu.h"
#include "etnaviv_gem.h"
#include "etnaviv_mmu.h"
#include "etnaviv_perfmon.h"
#include "etnaviv_sched.h"
#include "common.xml.h"
#include "cmdstream.xml.h"
#include "state.xml.h"

/* A311D (0x88) NN-AXI drain wait before the submit fence (ip-gotchas §10).
 * Default 2800us. RETUNED 2600 -> 2800 on 2026-07-16 after a HIGH-SAMPLE
 * stale-rate confirm (board 9e62, mn001, input-transition race detector)
 * corrected an earlier undersampled floor sweep. The NN-AXI settle race is a
 * CONTINUOUS stochastic ramp, not a sharp cliff: measured stale rate 2600us =
 * 0.34% (34/10000, Wilson 95% CI [0.243, 0.475]%), 2700us = 0.18%, 2800us =
 * 0.02% (1/5000, noise floor). The prior "2350 = first-failure / 2600 =
 * tightest-safe" boundary was a ~30-360-sample artifact (a 0.18% rate is
 * absent >50% of the time at n=360). No strict-zero value exists in range;
 * 2800 is the 17x-margin knee at a trivial +0.2ms/invoke (~7.04 -> ~7.24ms
 * warm). Every failure is a benign STALE read (the prior input's output;
 * wrong=0), never corrupt. Below ~2450us the ramp climbs to a ~100%-wrong
 * hard cliff at 2300us. History: 5000 -> 3000 (2026-06-21) -> 2600
 * (2026-07-05, since shown under-margined) -> 2800 (2026-07-16).
 * S905D3 (0x99) does NOT use this path (npu_poll_completion). Override:
 *   echo N > /sys/module/etnaviv/parameters/npu_drain_udelay */
static int npu_drain_udelay = 2800;
module_param_named(npu_drain_udelay, npu_drain_udelay, int, 0644);

/*
 * N9 output-size-scaled NN-AXI drain (A311D 0x88). The A311D post-fence
 * drain settles the last NN/TP output write to DDR before a CPU read. The
 * required time scales ~linearly with the output byte volume (~19 ns/B
 * measured on a 3->64 112x112 conv, 802 KB: 15 ms drains clean), so the
 * fixed npu_drain_udelay -- validated only at mn001 scale (<=~200 KB) --
 * silently corrupts any submit whose final output exceeds what it covers
 * (per the large-spatial-tiling audit). The kernel computes:
 *   drain_ns = out_bytes * npu_drain_ns_per_byte + npu_drain_margin_us*1000
 * floored at npu_drain_udelay (small outputs keep the exact legacy 3 ms).
 * A scaled udelay would block the hardirq for 15-48 ms (dropped IRQs / RCU
 * stalls), so any drain > npu_drain_hardirq_max_us is deferred to a
 * sleepable delayed work (usleep_range) that signals the fence. mesa passes
 * the final-op output byte count per submit via drm_etnaviv_gem_submit
 * .npu_output_bytes. All three are tunable at runtime for calibration.
 */
static int npu_drain_ns_per_byte = 30;   /* ~19 ns/B measured + margin */
module_param_named(npu_drain_ns_per_byte, npu_drain_ns_per_byte, int, 0644);
static int npu_drain_margin_us = 2000;    /* additive fixed cushion (us) */
module_param_named(npu_drain_margin_us, npu_drain_margin_us, int, 0644);
static int npu_drain_hardirq_max_us = 4000; /* defer above this (hardirq cap) */
module_param_named(npu_drain_hardirq_max_us, npu_drain_hardirq_max_us, int, 0644);

/* NPU phase-latency ring: 0 = off (zero overhead, production default),
 * 1 = stamp ktime at submit/irq/idle/signal into gpu->npu_phase[] for
 * post-hoc FE-walk / NN-exec / drain decode via debugfs. Toggle:
 *   echo 1 > /sys/module/etnaviv/parameters/npu_phase_trace
 * Read the ring: cat /sys/kernel/debug/dri/<n>/gpu  (NPU phase ring) */
static int npu_phase_trace;
module_param_named(npu_phase_trace, npu_phase_trace, int, 0644);

/* A311D (0x88) test-only: 1 = in the completion IRQ, print one ratelimited
 * line with VIVS_HI_IDLE_STATE and the NN/TP idle bits, before any drain or
 * fence signal (compute-not-awaited vs write-landed-late discriminator). */
static int npu_tail_idle_trace;
module_param_named(npu_tail_idle_trace, npu_tail_idle_trace, int, 0644);
MODULE_PARM_DESC(npu_tail_idle_trace,
	"A311D 0x88 test-only: log HI_IDLE_STATE (NN/TP bits) at completion IRQ (0 = off, default)");

/*
 * qi:S2 -- A311D (0x88) SH-wedge fast poller. See struct etnaviv_npu_wedge
 * in etnaviv_gpu.h for the design and the safety argument.
 *
 *   npu_wedge_poll_us  sampling period in us. 0 disables the sampler
 *                      entirely (the timer keeps ticking at a slow idle
 *                      rate but touches no MMIO). *** DEFAULT 0 = SAMPLER
 *                      OFF AT LOAD. *** The board this runs on has an NFS
 *                      rootfs and no sideband recovery, so an oops at probe
 *                      hangs the boot itself; the module must therefore be
 *                      inert with zero configuration. 200 us is the intended
 *                      working value -- set it at runtime once the box is
 *                      known good:
 *                        echo 200 > /sys/module/etnaviv/parameters/npu_wedge_poll_us
 *                      No rebuild and no reboot (0644).
 *   npu_wedge_trip_k   consecutive full-signature samples required before
 *                      the poller calls drm_sched_fault(). *** 0 =
 *                      OBSERVE-ONLY: sample and histogram, never fault. ***
 *                      Shipped as 0 deliberately -- the fast recovery path
 *                      stays inert until the measured streak distribution
 *                      licenses a value of K. Runtime-writable, so arming
 *                      it needs no rebuild and no reboot.
 *   npu_wedge_reset    write a CHANGED value to zero the per-GPU stats.
 *
 * Read the stats: cat /sys/kernel/debug/dri/<n>/gpu   ("NPU wedge poller")
 *
 * qi:S2c -- TWO measured corrections to the arming advice above. Both come
 * from widening the safety sweep from 3 containers to the 9-model tflite
 * corpus (the S2c safety-band audit).
 *
 * 1. K=500 IS NOT THE 8x-MARGIN VALUE S2 BELIEVED IT WAS. S2's "empty band,
 *    128..16383 samples, zero observations" was derived from 3 containers.
 *    `movenetthunder` -- healthy, zero recoveries, wall flat at 387 ms --
 *    produces a full-signature streak of 488 samples (97.6 ms) ONCE PER
 *    INVOKE, reproducibly (488 observe-only / 487 armed, 55 observations in
 *    55 invokes). So the healthy ceiling is 488, not 63, and K=500 clears it
 *    by 12 samples = 2.4%, not by 8x. A false positive here is NOT caught by
 *    the nn_tp_busy guard -- the signature is NN+TP IDLE by construction --
 *    so it goes straight to a recovery of a healthy job, and the K margin is
 *    the only thing standing in the way. The measured band is [489..21400]
 *    (97.8 ms .. 4.28 s); its geometric centre is ~3200 samples (~640 ms),
 *    which is the defensible K if this is ever armed. 500 is not.
 *
 * 2. SETTING npu_wedge_poll_us WITHOUT npu_wedge_trip_k CAN RESET THE BOARD.
 *    Observe-only (poll_us=200, trip_k=0) under a heavily-wedging container
 *    (`yolox`) reset 9e62 twice out of two attempts, within ~60 s. The
 *    identical workload with poll_us=0 survived 45 stock recoveries over
 *    211 s, and with the fast path ARMED (200/500) survived and finished
 *    13.5x faster. The distinguishing quantity is how long the sampler
 *    hammers MMIO into a WEDGED front end: armed, a wedge lasts ~116 ms of
 *    sampling; observe-only, it lasts the full 4.5 s, 45 times over. So the
 *    "characterise first with trip_k=0, then arm" procedure is the DANGEROUS
 *    order on a wedging workload. Characterise on healthy containers only.
 *
 * qi:S6 -- K=3200 IS NOW VALIDATED ON SILICON, AND THE DEFAULT STAYS 0.
 * (per the S6 armed-default audit.)
 *
 * 3. THE ARMING VALUE IS K=3200 AT poll_us=200. S2c derived it as the band's
 *    geometric centre but never ran it. Armed at 200/3200 across all twelve
 *    containers S2c measured (the 9-model tflite corpus plus chain4_person,
 *    chain4_frec, mixed_cdp), 50 iters + 5 warm each: ZERO faults on every
 *    healthy container, including the two that sit inside S2's claimed-empty
 *    band (`movenetthunder` 486, `inception` 162). Both wedging containers
 *    still trip every wedge -- `yolox` 46 faults for 45 recoveries, all
 *    path=poll, 0 path=timeout, 0 declined. The healthy ceiling was
 *    re-measured six times across a 69-71 C thermal sweep (a `yolox` soak
 *    between the cold and hot triples) and moved only 486..502, so K=3200
 *    clears the worst healthy streak ever recorded on this board (555, S2c
 *    under K=500 false-firing) by 2645 samples = ~38x the observed
 *    run-to-run drift. K=500 cleared it by -55.
 *
 * 4. THE WIN AT THE SAFE K IS ~5x, NOT THE 13.5x MEASURED AT K=500. Matched
 *    pairs, same session, same module, 20 iters + 3 warm:
 *      yolox            8959.78 ms inert -> 1744.33 ms armed = 5.14x
 *      movenetlightning 4483.82 ms inert ->  913.14 ms armed = 4.91x
 *    The detection window is 6.4x longer by construction, so the residual
 *    per-wedge cost is ~872 ms instead of ~307 ms. Do NOT quote 13.5x for a
 *    safely-armed configuration.
 *
 * 5. WHY THE DEFAULT IS STILL 0 DESPITE (3) AND (4). Not cost -- the
 *    sampler's steady-state throughput cost is below the measurement floor.
 *    Two reasons. First, on the measured corpus the beneficiary set is
 *    EMPTY: after the 5x win `yolox` is still 5.2x slower than its CPU
 *    reference and `movenetlightning` 10.1x, so both remain CPU-routed, and
 *    the one container that wedges terminally only fails sooner. Second,
 *    K=3200 leaves the sampler hammering a WEDGED front end for ~640 ms per
 *    wedge instead of ~116 ms at K=500 -- 5.5x more of the exposure that is
 *    the measured board-reset mechanism in (2). ~150 recoveries survived it
 *    here, but that is one board, one session, against a reset-class hazard.
 *    Arming is a two-line runtime change with no rebuild, so a user with a
 *    wedging NPU-favourable container pays nothing to opt in.
 *
 * TO ARM (both writes, in this order, never poll_us alone -- see (2)):
 *    echo 3200 > /sys/module/etnaviv/parameters/npu_wedge_trip_k
 *    echo  200 > /sys/module/etnaviv/parameters/npu_wedge_poll_us
 * TO DISARM, reverse it: poll_us to 0 FIRST, then trip_k.
 */
static int npu_wedge_poll_us;
module_param_named(npu_wedge_poll_us, npu_wedge_poll_us, int, 0644);
MODULE_PARM_DESC(npu_wedge_poll_us,
	"A311D 0x88 SH-wedge predicate sampling period in us (0 = sampler off, the default; setting this WITHOUT npu_wedge_trip_k has been measured to reset the board on a wedging workload -- see qi:S2c)");

/*
 * Build-provenance marker. The staged module lives on an NFS root that no
 * human can inspect from the target while the target is wedged, so the ONLY
 * way to establish which source a staged etnaviv.ko was built from is a
 * string the source puts there deliberately. Bump the suffix whenever a
 * change must be provable in the binary. Visible three ways without booting:
 *   strings etnaviv.ko | grep qi:S2
 *   nm etnaviv.ko | grep npu_wedge_build
 *   cat /sys/module/etnaviv/parameters/npu_wedge_build   (once booted)
 */
#define ETNA_NPU_WEDGE_BUILD "qi:S2c-v9-band-merged"
static char *npu_wedge_build = ETNA_NPU_WEDGE_BUILD;
module_param_named(npu_wedge_build, npu_wedge_build, charp, 0444);
MODULE_PARM_DESC(npu_wedge_build,
	"qi:S2 wedge-poller source provenance marker (read-only)");
static int npu_wedge_trip_k;
module_param_named(npu_wedge_trip_k, npu_wedge_trip_k, int, 0644);
MODULE_PARM_DESC(npu_wedge_trip_k,
	"consecutive wedge-signature samples before drm_sched_fault (0 = observe only, the default). VALIDATED ARMING VALUE IS 3200 at npu_wedge_poll_us=200 (qi:S6): 0 faults across 12 healthy containers, every real wedge still caught, ~38x the observed healthy-streak drift. 500 (the qi:S2 recommendation) FALSE-FIRES on healthy work -- do not use it. Write trip_k FIRST, then poll_us");
/*
 * qi:S2b -- per-fast-fault diagnostic printk. Off by default: it is a console
 * printk inside the TDR, and the measurement that motivated this knob showed
 * it is not merely noisy but PERTURBING (it stretches the handler by
 * milliseconds, which is many poller periods). Any wall figure quoted as the
 * fast path's cost must be taken with this at 0.
 */
static int npu_wedge_verbose;
module_param_named(npu_wedge_verbose, npu_wedge_verbose, int, 0644);
MODULE_PARM_DESC(npu_wedge_verbose,
		 "log the poller/TDR handshake at every fast fault (perturbs timing)");

static int npu_wedge_reset;
module_param_named(npu_wedge_reset, npu_wedge_reset, int, 0644);
MODULE_PARM_DESC(npu_wedge_reset,
	"write a changed value to zero the wedge-poller statistics");

/*
 * qi:N1437 -- HI_AXI_CONFIG (0x00008) arm.
 *
 * etnaviv_gpu_hw_init() writes AWCACHE(2)|ARCACHE(2) = 0x2200 UNCONDITIONALLY,
 * with a comment naming an iMX6 lockup as the reason. galcore writes that
 * literal only for a short list of model/rev pairs, and 0x8000/0x7120 (the
 * A311D VIPNano-QI) is NOT one of them -- galcore leaves the reset value.
 * qi:N1410 F6 found the difference; qi:N1433 F3 promoted it to the arc's only
 * surviving candidate for the person NNA-port shortfall (ours caps at 84.3 %
 * of the achievable port roof where the vendor reaches 100.3 %), because AXI
 * cache attributes change achieved DDR efficiency at unchanged request
 * addresses and can only bind where the port binds.
 *
 * -1 (default) = unchanged upstream behaviour: write 0x2200.
 *  0           = SKIP the write entirely, i.e. leave the reset value, which is
 *                what galcore does on this chip.
 * >0           = write this literal instead.
 *
 * NPU-scoped: the override applies only when nn_core_count > 0, so a Mali/GC
 * GPU sharing this module keeps the iMX6-safe write no matter what is set.
 */
static int npu_axi_config = -1;
module_param_named(npu_axi_config, npu_axi_config, int, 0644);
MODULE_PARM_DESC(npu_axi_config,
	"NPU-only HI_AXI_CONFIG override: -1 = upstream AWCACHE(2)|ARCACHE(2) (default), 0 = skip the write and leave the reset value (galcore behaviour), >0 = write this literal. Takes effect at the next gpu_init/hw_init");

/*
 * qi:N1437 -- hang-dump compaction arm (N1418-b).
 *
 * The per-word recovery dump is one dev_err per word: up to 24 words of cmdbuf
 * plus 32 words of ring, per recovery, synchronous, at console level. qi:N1418
 * found 3 of 5 board restarts undiagnosable because that burst saturates
 * netconsole at exactly the moment a fault would print. Compaction packs the
 * same words 8-per-line (56 lines -> 7) and rate-limits whole dumps, so the
 * bytes that matter survive the burst.
 *
 * 0 (default) = unchanged upstream behaviour: one line per word, no limit.
 * 1           = 8 words per line + at most 2 dumps per 10 s.
 */
/*
 * qi:N1437 -- credit_limit arm (N1435-b).
 *
 * etnaviv_sched_init() sets .credit_limit = nn_core_count > 0 ? 1 :
 * etnaviv_hw_jobs_limit, but it runs from etnaviv_gpu_bind() at component-bind
 * time, BEFORE etnaviv_hw_identify() populates nn_core_count. So the ternary
 * takes its non-NPU arm and every NPU runs at etnaviv_hw_jobs_limit = 4,
 * against sched_init's own comment: "Limit the credit budget to 1 in-flight
 * job so the ring buffer isn't overwritten." The timeout half of the same race
 * IS post-corrected below; credit_limit is corrected nowhere (qi:N1435 F6 --
 * one assignment site tree-wide, and the board reads hw_job_limit=4).
 *
 * -1 (default) = leave the racy value in place (upstream behaviour).
 * >0           = set drm_sched credit_limit to this after hw_identify.
 */
static int npu_credit_limit = -1;
module_param_named(npu_credit_limit, npu_credit_limit, int, 0644);
MODULE_PARM_DESC(npu_credit_limit,
	"NPU-only drm_sched credit_limit post-correction: -1 = leave the sched_init race in place (default), >0 = set this many in-flight jobs (1 is what sched_init's comment demands). Takes effect at the next gpu_init");

static int npu_hangdump_compact;
module_param_named(npu_hangdump_compact, npu_hangdump_compact, int, 0644);
MODULE_PARM_DESC(npu_hangdump_compact,
	"0 = one dev_err per dumped word (default, upstream), 1 = pack 8 words per line and rate-limit the dump");
static DEFINE_RATELIMIT_STATE(npu_hangdump_rs, 10 * HZ, 2);

/*
 * qi:N1453 -- pre-reset diagnostic MMIO in etnaviv_gpu_recover_hang().
 *
 * qi:N1452 measured a board-killing defect: on the A311D (0x88) SH wedge the
 * scheduler correctly decides to recover, and recover_hang()'s FIRST act was a
 * block of eight diagnostic register reads against a block whose AXI is hung.
 * One of them takes an uncontainable asynchronous external abort --
 * `SError Interrupt on CPU4, code 0x00000000bf000002` at
 * `etnaviv_gpu_recover_hang+0xc0` -- and arm64 panics by design. Two of two
 * induced wedges killed the board, 382 ms after the recover decision, and
 * `recover hung GPU!` never printed because the fault preceded it.
 *
 * The fix is ordering: the snapshot now happens AFTER etnaviv_hw_reset(), and
 * dma_addr comes from the caller, which already read it on the way in. That
 * leaves ZERO device access between entry and the reset.
 *
 * This mask re-enables the pre-reset reads, one bit per register, so a wedged
 * board can be used to name which of them faults. 0 (default) = none.
 *   bit1 VIVS_HI_IDLE_STATE        bit2 VIVS_FE_DMA_DEBUG_STATE
 *   bit3 VIVS_HI_INTR_ENBL         bit4 VIVS_HI_INTR_ACKNOWLEDGE
 *   bit5 VIVS_PM_MODULE_CONTROLS   bit6 VIVS_PM_MODULE_STATUS
 *   bit7 VIVS_PM_POWER_CONTROLS
 * qi:N1462: bit0 is VIVS_FE_DMA_ADDRESS, so 0xff is the byte-for-byte
 * pre-qi:N1453 eight-read block. qi:N1461 could not test that read because
 * bit0 was reserved; note that the read is NOT absent from the timeout path
 * even at predump=0 -- etnaviv_sched_timedout_job() takes it on its way in and
 * hands the value over.
 *
 * qi:N1465 -- SETTLED, AND ANY NON-ZERO VALUE HERE DESTROYS THE BOARD. On a
 * wedged A311D 0x88, in the first inference cell after a boot, the FIRST
 * gpu_read() issued from this function takes the SError. There is no safe
 * subset and no safe register: 0x10 (INTR_ACKNOWLEDGE) alone and 0x01
 * (FE_DMA_ADDRESS) alone each killed the board 2 of 2 at reps=1, i.e. at ONE
 * read per wedge; the disjoint halves 0x10 and 0xef both killed it; 11 armed
 * cells produced 11 panics and the only clean mask is 0 (0/9 pooled,
 * Fisher one-sided p = 5.95e-6). The abort is POSITIONAL: FE_DMA_ADDRESS is
 * read safely by etnaviv_sched_timedout_job() and again by the core dump on
 * the same wedge, and fatally here. So this mask can no longer name "which of
 * them faults" -- that question is answered -- and it survives only as the
 * reproducer for the open question of WHAT changes across that boundary.
 * Setting it costs a board reboot per wedge; do not set it on anything you are
 * not prepared to lose.
 */
static unsigned int npu_recover_predump;
module_param_named(npu_recover_predump, npu_recover_predump, uint, 0644);
MODULE_PARM_DESC(npu_recover_predump,
	"bitmask of PRE-reset diagnostic MMIO reads in recover_hang: 0 = none (default; the reads happen after the reset instead). DANGEROUS: any non-zero value SErrors and kills a wedged A311D NPU on the first cell after a boot -- measured 11 panics in 11 armed cells, one read is enough (qi:N1465)");

/*
 * qi:N1462 -- how many times to REPEAT the enabled pre-reset block.
 *
 * This knob turns the dose up so a rare per-read abort would be separable from
 * a never-faulting read in a handful of wedges: N passes deliver
 * N x popcount(predump) reads onto the wedged block before the reset.
 * Default 1 = unchanged behaviour.
 *
 * qi:N1465 -- THE DOSE IS NOT THE VARIABLE, measured in both directions, and
 * the reason this knob was built is VOID. It was built because qi:N1461 ran
 * predump=0x20 and predump=0xfe as must-fire arms and neither panicked; those
 * arms are now known to have run on a DISARMED board (qi:N1464: the abort
 * reproduces only on the first inference cell after a boot), so they refuted
 * nothing and no dose argument was ever needed. Measured at the armed
 * position: predump=0xff at the SHIPPED reps=1 -- eight reads, no repetition --
 * panics 2 of 2, while predump=0 at reps=64 is clean over 10 wedges and issues
 * ZERO reads, which is what the `predump &&` in the loop guard below requires.
 * Repetition therefore has no independent effect and is not required to
 * reproduce anything.
 */
static unsigned int npu_recover_predump_reps = 1;
module_param_named(npu_recover_predump_reps, npu_recover_predump_reps, uint, 0644);
MODULE_PARM_DESC(npu_recover_predump_reps,
	"repeat the npu_recover_predump block this many times before the reset (default 1); raises the per-wedge MMIO dose so a rare per-read abort becomes observable");

/*
 * qi:N1469 -- MMIO-FREE pre-reset delay.  This is the knob that identified the
 * pre-reset abort's discriminator, and it is kept so the threshold can be
 * narrowed without another build.
 *
 * qi:N1467 measured that the fatal pre-reset read is fatal only once
 * etnaviv_core_dump()'s BODY has run: with etnaviv_dump_core=0 the window
 * collapses from 410.848 ms to 7.574 ms and 48 such reads over 48 wedges were
 * clean, while the identical read with the dump armed panicked 6/6.  But
 * dump_core moves the body's ~403 ms of elapsed time and its multi-MB
 * vmalloc/vmap/memcpy work TOGETHER.
 *
 * This knob re-supplies ONLY the time -- an msleep() in the exact position the
 * dump body occupied, issuing no MMIO, allocating nothing, mapping nothing and
 * touching no device state.  Run at dump_core=0, qi:N1469 measured a clean step
 * function: 0 ms 0/2, 50 ms 0/2, 200 ms 0/2, 400 ms 3/3 PANIC, 800 ms 3/3
 * PANIC.  So the discriminator is ELAPSED TIME, not the body's memory work, and
 * the threshold (200, 400] ms is the length of the window it replaces.
 *
 * DIAGNOSTIC ONLY.  Default 0 = no delay = unchanged behaviour.
 */
static unsigned int npu_recover_predelay_ms;
module_param_named(npu_recover_predelay_ms, npu_recover_predelay_ms, uint, 0644);
MODULE_PARM_DESC(npu_recover_predelay_ms,
	"MMIO-free msleep() in recover_hang before the npu_recover_predump block, in ms (default 0 = none). Diagnostic lever: qi:N1469 used it to show the pre-reset abort's discriminator is ELAPSED TIME, not the core dump's memory work -- 400 ms of pure sleep panics a wedged A311D NPU 3/3, 200 ms is clean 0/2. Issues no MMIO and changes no device state, but it delays every GPU/NPU recovery by that much");

/*
 * qi:N1488 -- FIX THE HAZARD AT THE LAYER IT LIVES IN: THE POWER DOMAIN.
 *
 * The chain is measured end to end.  The reproducer arms on the first cell of a
 * boot (qi:N1464); any single pre-reset gpu_read() suffices and the abort is
 * POSITIONAL, not register-specific (qi:N1465); the discriminator is elapsed
 * TIME, not the dump's memory work (qi:N1469); the timer is runtime-PM
 * autosuspend dropping this device's NNA genpd domain inside the recovery
 * window (qi:N1473); and qi:N1486 closed the ordering -- on 2 of 2 measured
 * fatal reads a power-off had ALREADY BEGUN, against 0 of 32 surviving reads
 * (Fisher two-sided p = 1.78e-3).  One sentence: MMIO to the NNA block after
 * its power domain has begun powering off takes an SError and kills the board.
 *
 * The landed fix b46b42a68bfe7 removes the MMIO from recover_hang() before the
 * reset.  That is a DOWNSTREAM MASK and it is not complete: qi:N1462 counted
 * 8 of ~47 reads on that path, with etnaviv_core_dump_registers() taking 37 and
 * etnaviv_sched_timedout_job() 2 more off the same wedged block.  Deleting
 * reads one call site at a time cannot close a hazard whose cause is that the
 * domain is allowed to drop while the recovery path is still using the device.
 *
 * The correct-layer fix is a runtime-PM reference held for the DURATION of the
 * recovery, taken before drm_sched_stop() and released after drm_sched_start(),
 * so no read anywhere in that path can race the power-off.  With usage_count
 * raised, rpm_suspend() declines and the autosuspend timer cannot drop the
 * domain inside the window; if the device is already suspended when recovery
 * starts, get_sync resumes it -- which recover_hang() itself already does
 * unconditionally at its reset, so that path is not new.
 *
 * The window is a sleepable one: drm_sched's timedout_job work item, where
 * recover_hang() already calls pm_runtime_get_sync() today.
 *
 * DEFAULT 1 SINCE qi:N1500.  It shipped at 0 from qi:N1492 until qi:N1500 built
 * THIS commit's own tree for the board -- every prior measurement in the arc ran
 * on a different base -- and re-ran the armed rung on the resulting object:
 * 3 of 3 cells SURVIVE at npu_recover_predelay_ms=400 with
 * power/autosuspend_delay_ms at 305/312, against a banked 12 of 12 PANIC at that
 * rung with the hold absent (Fisher two-sided p = 2.197802e-03).
 *
 * WHY THE DEFAULT IS UNCONDITIONAL AND NOT GATED TO THE MEASURED SILICON.
 * etnaviv is not an A311D driver -- the same module drives S905D3 / VIPPico v3
 * (customer 0x99) and every other Vivante part -- so a scoped default was the
 * obvious conservative choice and it is the WRONG one, because the unheld state
 * is not safe anywhere.  Read etnaviv_gpu_rpm_suspend(): it vetoes on
 * gpu->sched.credit_count, and drm_sched_stop() is what CLEARS that count.  So
 * the veto -- not luck -- is what protects the reads this function makes before
 * drm_sched_stop(), and every read AFTER it, including etnaviv_core_dump()'s 37
 * off the wedged block, runs today with NO runtime-PM reference on EVERY
 * Vivante part.  Gating the default to 0x88 would have left 0x99 -- the part
 * with no board in this fleet AND the part whose own autosuspend_delay override
 * in etnaviv_gpu_init() blames this very hazard class for an AXI-bus-error
 * hardlockup -- exposed on purpose.  (qi:N1501, source review.)
 *
 * THE PRICE IS UNPRICED AND BOUNDED ABOVE, and this comment does not pretend
 * otherwise.  qi:N1474's 2036.62 vs 2039.81 ms median and its "13 NNA
 * power-cycles per 45 s session" price a DIFFERENT lever -- autosuspend_delay_ms
 * 200 -> 30000, which holds the domain across a whole session and which that
 * round declined to land.  This knob holds the reference only across the
 * recovery window and only on a recovery, so that measurement bounds it from
 * ABOVE and nothing measures it directly (qi:N1501 N1501-d).
 *
 * At an explicit 0 this takes no reference, prints nothing, and the recovery
 * path is byte-for-byte the pre-qi:N1500 shipped one.
 */
static unsigned int npu_recover_pm_hold = 1;
module_param_named(npu_recover_pm_hold, npu_recover_pm_hold, uint, 0644);
MODULE_PARM_DESC(npu_recover_pm_hold,
	"Hold a runtime-PM reference on this device across the WHOLE GPU/NPU recovery path -- taken before drm_sched_stop(), released after drm_sched_start() (default 1 since qi:N1500; 0 = the pre-qi:N1500 shipped behaviour). Correct-layer fix for the qi:N1473/qi:N1486 SError: MMIO to the NNA block after its power domain has begun powering off kills the board, and with usage_count raised the autosuspend timer cannot drop the domain inside the recovery window. Unconditional because etnaviv_gpu_rpm_suspend()'s credit_count veto is cleared by drm_sched_stop(), so every read after that point -- including etnaviv_core_dump()'s 37 -- is unreferenced on every Vivante part. Issues no MMIO of its own; costs one runtime-PM resume plus two KERN_DEBUG log lines per recovery (off the console since qi:N1508), and defers this device's autosuspend by that recovery's duration");

/*
 * qi:N1501 N1501-b: dev->power.usage_count exists only under CONFIG_PM
 * (include/linux/pm.h, struct dev_pm_info), and pm_runtime.h stubs the
 * accessors but not the field.  This driver's Kconfig has no PM dependency,
 * so the two log lines below read it through the same guard the rest of the
 * file's runtime-PM code uses; -1 is "not built with PM", which is never a
 * real usage count.
 */
static int etnaviv_npu_recover_pm_usage(struct device *dev)
{
#ifdef CONFIG_PM
	return atomic_read(&dev->power.usage_count);
#else
	return -1;
#endif
}

/*
 * Returns true when a reference was actually taken, which is the only thing the
 * caller may pass to etnaviv_npu_recover_pm_put().  A failed get still leaves
 * the usage count raised, so it is dropped here with put_noidle and reported --
 * a get that failed is a RESULT, not a silent fall-through to the old path.
 */
bool etnaviv_npu_recover_pm_get(struct etnaviv_gpu *gpu)
{
	int ret;

	if (!READ_ONCE(npu_recover_pm_hold))
		return false;

	ret = pm_runtime_get_sync(gpu->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(gpu->dev);
		dev_err(gpu->dev, "recover pm_hold GET FAILED ret=%d\n", ret);
		return false;
	}

	/*
	 * qi:N1501 N1501-a, folded in by qi:N1500: this was dev_err while the
	 * knob defaulted to 0, where it only ever fired for somebody who had
	 * asked for it.  At default 1 it fires on every recovery of every
	 * Vivante part, and a successful, expected reference is not KERN_ERR.
	 *
	 * qi:N1508 (N1506-a): it is now KERN_DEBUG through dev_printk(), NOT
	 * dev_dbg().  qi:N1504 priced the shipped default at +7.3 ms per
	 * undumped recovery and qi:N1506 measured 5.5 ms of that to be THIS
	 * line's UART time on a console=ttyAML0,115200n8 board at loglevel 7
	 * (the inserted segment fell 6.7 -> 0.1 ms with KERN_INFO kept off the
	 * console).  KERN_DEBUG is below the default console_loglevel (7 prints
	 * levels 0..6), so the line never reaches the serial or net console,
	 * while it still lands in the printk ring -- /dev/kmsg and dmesg -- where
	 * every scorer reads it.  dev_dbg() would compile to a dynamic-debug
	 * site that is OFF by default (CONFIG_DYNAMIC_DEBUG=y on our arm64
	 * defconfig), which would make the instrument depend on a per-boot
	 * enable; dev_printk(KERN_DEBUG) is unconditional.  The string is
	 * unchanged so the record's RE_ON/RE_OFF matchers still fire.
	 * The GET FAILED path above keeps dev_err: that one IS an error.
	 */
	dev_printk(KERN_DEBUG, gpu->dev, "recover pm_hold ON ret=%d usage=%d\n",
		   ret, etnaviv_npu_recover_pm_usage(gpu->dev));
	return true;
}

void etnaviv_npu_recover_pm_put(struct etnaviv_gpu *gpu, bool held)
{
	if (!held)
		return;

	/* qi:N1508 -- KERN_DEBUG, see the ON print above. */
	dev_printk(KERN_DEBUG, gpu->dev, "recover pm_hold OFF usage=%d\n",
		   etnaviv_npu_recover_pm_usage(gpu->dev));
	pm_runtime_mark_last_busy(gpu->dev);
	pm_runtime_put_autosuspend(gpu->dev);
}

/*
 * Emit words[start..end) either one dev_err per word (upstream shape) or, when
 * npu_hangdump_compact is set, eight per line. The hang word is marked in both
 * shapes: "**" in the per-word form, a "<-- word N" suffix in the packed one.
 */
static void etnaviv_hangdump_words(struct etnaviv_gpu *gpu, const char *tag,
				   const u32 *words, int start, int end,
				   int center)
{
	int w;

	if (!npu_hangdump_compact) {
		for (w = start; w < end; w++)
			dev_err(gpu->dev, "  %s %s[%4d]=0x%08x\n",
				w == center ? "**" : "  ", tag, w, words[w]);
		return;
	}

	for (w = start; w < end; w += 8) {
		int n = min(8, end - w);
		bool hit = center >= w && center < w + n;

		switch (n) {
		case 8:
			dev_err(gpu->dev, "  %s[%4d]: %08x %08x %08x %08x %08x %08x %08x %08x%s\n",
				tag, w, words[w], words[w + 1], words[w + 2],
				words[w + 3], words[w + 4], words[w + 5],
				words[w + 6], words[w + 7],
				hit ? " <--" : "");
			break;
		default: {
			/* tail: at most 7 words, one dev_err, built by hand */
			char buf[8 * 9 + 1];
			int i, p = 0;

			for (i = 0; i < n; i++)
				p += scnprintf(buf + p, sizeof(buf) - p,
					       "%08x ", words[w + i]);
			dev_err(gpu->dev, "  %s[%4d]: %s%s\n", tag, w, buf,
				hit ? "<--" : "");
			break;
		}
		}
	}
	if (center >= start && center < end)
		dev_err(gpu->dev, "  %s hang word is [%4d]=0x%08x\n",
			tag, center, words[center]);
}

/*
 * qi:N1443 -- VIVS_GL_NN_CONFIG.DDR_BURST_SIZE sweep arm (N1442-a).
 *
 * etnaviv writes VIVS_GL_NN_CONFIG (0x3930) at three sites and every value it
 * ever writes -- 0, 0x10 -- leaves the 0x60 field named DDR_BURST_SIZE in
 * state.xml.h at 0.  galcore's kernel never writes this register at all, so
 * the vendor's value comes from userspace and cannot be recovered: the
 * register is WRITE-ONLY on this chip (see the AHB note in
 * etnaviv_gpu_init_nn_flop_reset()).  A stock-vs-patched comparison therefore
 * cannot say what the hardware is doing; the settling measurement is a sweep
 * over the field's four encodable values, read as a response curve.
 *
 * -1 (default) = unchanged: every NN_CONFIG value the driver writes is left
 *                exactly as it is, and no user command stream is inspected or
 *                modified.  A module loaded with the default is behaviourally
 *                identical to the stock module.
 * 0..3         = force DDR_BURST_SIZE to this value everywhere NN_CONFIG is
 *                programmed for this device: the two init flop-reset writes,
 *                the per-submit flop prelude, AND any LOAD_STATE covering
 *                0x3930 inside the user's own command stream (see
 *                etnaviv_npu_nn_config_patch_stream()).  The last of those is
 *                what makes the lever reach the NN op even if the compiled
 *                NBG programs the register itself.
 *
 * NPU-scoped: only applied when nn_core_count > 0, so a Mali/GC GPU sharing
 * this module is untouched no matter what is set.
 *
 * npu_nn_prelude_emits / _last, npu_nn_user_writes / _last_pre / _last_post
 * are read-only counters the driver maintains itself.  They are this round's
 * armed-ness witness: the register cannot be read back, so the witness is what
 * actually went into the command stream, not a register readback.
 */
static int npu_nn_ddr_burst = -1;
module_param_named(npu_nn_ddr_burst, npu_nn_ddr_burst, int, 0644);
MODULE_PARM_DESC(npu_nn_ddr_burst,
	"NPU-only VIVS_GL_NN_CONFIG.DDR_BURST_SIZE (mask 0x60) override: -1 = leave every driver-written value untouched and inspect no user stream (default), 0..3 = force the field to this value at the init flop-reset, the per-submit prelude and inside the user command stream");

static unsigned int npu_nn_prelude_emits;
module_param_named(npu_nn_prelude_emits, npu_nn_prelude_emits, uint, 0444);
MODULE_PARM_DESC(npu_nn_prelude_emits,
	"read-only: NN_CONFIG LOAD_STATEs the per-submit flop prelude has emitted");

static unsigned int npu_nn_prelude_last = 0xffffffff;
module_param_named(npu_nn_prelude_last, npu_nn_prelude_last, uint, 0444);
MODULE_PARM_DESC(npu_nn_prelude_last,
	"read-only: the last NN_CONFIG value the per-submit flop prelude actually emitted");

static unsigned int npu_nn_user_writes;
module_param_named(npu_nn_user_writes, npu_nn_user_writes, uint, 0444);
MODULE_PARM_DESC(npu_nn_user_writes,
	"read-only: LOAD_STATEs covering 0x3930 seen in user command streams (only counted while npu_nn_ddr_burst >= 0)");

static unsigned int npu_nn_user_last_pre = 0xffffffff;
module_param_named(npu_nn_user_last_pre, npu_nn_user_last_pre, uint, 0444);
MODULE_PARM_DESC(npu_nn_user_last_pre,
	"read-only: the last user-stream NN_CONFIG value BEFORE the override");

static unsigned int npu_nn_user_last_post = 0xffffffff;
module_param_named(npu_nn_user_last_post, npu_nn_user_last_post, uint, 0444);
MODULE_PARM_DESC(npu_nn_user_last_post,
	"read-only: the last user-stream NN_CONFIG value AFTER the override");

static unsigned int npu_nn_user_stops;
module_param_named(npu_nn_user_stops, npu_nn_user_stops, uint, 0444);
MODULE_PARM_DESC(npu_nn_user_stops,
	"read-only: user-stream walks abandoned at an opcode of unknown length -- a non-zero value means npu_nn_user_writes is a LOWER BOUND");

static unsigned int npu_nn_user_streams;
module_param_named(npu_nn_user_streams, npu_nn_user_streams, uint, 0444);
MODULE_PARM_DESC(npu_nn_user_streams,
	"read-only: user command streams walked");

/*
 * Apply the override to one NN_CONFIG value.  Returns @val unchanged when the
 * knob is off or the device is not an NPU, which is what makes the default
 * build behaviourally identical to stock.
 */
u32 etnaviv_npu_nn_config(struct etnaviv_gpu *gpu, u32 val)
{
	if (npu_nn_ddr_burst < 0 || gpu->identity.nn_core_count == 0)
		return val;

	return (val & ~VIVS_GL_NN_CONFIG_DDR_BURST_SIZE__MASK) |
	       VIVS_GL_NN_CONFIG_DDR_BURST_SIZE(npu_nn_ddr_burst & 3);
}

void etnaviv_npu_nn_config_note_prelude(u32 val)
{
	npu_nn_prelude_emits++;
	npu_nn_prelude_last = val;
}

/*
 * Walk a user command stream and force DDR_BURST_SIZE inside every LOAD_STATE
 * that covers VIVS_GL_NN_CONFIG.  The walk is the same one
 * etnaviv_cmd_validate_one() performs, which is NOT run on this chip because
 * it is skipped for ETNAVIV_IOMMU_V2 -- hence the separate copy here.
 *
 * Runs only while the knob is armed, so the control arm pays none of its cost.
 * @stream is the kernel's own copy of the submitted stream, after relocations
 * and before it is copied into the cmdbuf.
 */
void etnaviv_npu_nn_config_patch_stream(struct etnaviv_gpu *gpu, u32 *stream,
					unsigned int size)
{
	const unsigned int nn_off = VIVS_GL_NN_CONFIG >> 2;
	u32 *buf = stream, *end = stream + size;

	if (npu_nn_ddr_burst < 0 || gpu->identity.nn_core_count == 0)
		return;

	npu_nn_user_streams++;

	while (buf < end) {
		u32 cmd = *buf;
		unsigned int op = cmd >> 27, len, n, off;

		switch (op) {
		case FE_OPCODE_LOAD_STATE:
			n = (cmd & VIV_FE_LOAD_STATE_HEADER_COUNT__MASK) >>
			    VIV_FE_LOAD_STATE_HEADER_COUNT__SHIFT;
			if (n == 0)
				n = 1024;
			len = ALIGN(1 + n, 2);
			if (buf + len > end)
				return;
			off = (cmd & VIV_FE_LOAD_STATE_HEADER_OFFSET__MASK) >>
			      VIV_FE_LOAD_STATE_HEADER_OFFSET__SHIFT;
			if (off <= nn_off && nn_off < off + n) {
				u32 *slot = buf + 1 + (nn_off - off);

				npu_nn_user_writes++;
				npu_nn_user_last_pre = *slot;
				*slot = (*slot &
					 ~VIVS_GL_NN_CONFIG_DDR_BURST_SIZE__MASK) |
					VIVS_GL_NN_CONFIG_DDR_BURST_SIZE(
						npu_nn_ddr_burst & 3);
				npu_nn_user_last_post = *slot;
			}
			break;
		case FE_OPCODE_DRAW_2D:
			n = (cmd & VIV_FE_DRAW_2D_HEADER_COUNT__MASK) >>
			    VIV_FE_DRAW_2D_HEADER_COUNT__SHIFT;
			if (n == 0)
				n = 256;
			len = 2 + n * 2;
			break;
		case FE_OPCODE_END:
		case FE_OPCODE_NOP:
		case FE_OPCODE_WAIT:
		case FE_OPCODE_LINK:
		case FE_OPCODE_STALL:
		case FE_OPCODE_CALL:
		case FE_OPCODE_RETURN:
		case FE_OPCODE_CHIP_SELECT:
			len = 2;
			break;
		case FE_OPCODE_DRAW_PRIMITIVES:
		case FE_OPCODE_DRAW_INSTANCED:
			len = 4;
			break;
		case FE_OPCODE_DRAW_INDEXED_PRIMITIVES:
			len = 6;
			break;
		default:
			/*
			 * Unknown opcode: STOP rather than guess a length.  A
			 * mis-walked stream would write into the wrong dword,
			 * so the walk is deliberately incomplete instead.  The
			 * counter says it happened, so a null cannot be read as
			 * "no NN_CONFIG in the stream" when it was really
			 * "the walk stopped".
			 */
			npu_nn_user_stops++;
			return;
		}

		buf += len;
	}
}

/* Idle tick used when the sampler is disabled: keeps the timer alive so a
 * runtime re-enable needs no re-arm, at ~50 wakeups/s and zero MMIO. */
#define ETNA_WEDGE_IDLE_TICK_US 20000

/* Hard floor on npu_wedge_poll_us. See the softirq-starvation note in
 * etnaviv_npu_wedge_poll(). Applied to the value the timer re-arms with,
 * NOT to the module parameter, so a too-small write is clamped rather than
 * rejected and the parameter still reads back what was written. */
#define ETNA_WEDGE_MIN_POLL_US 50
#include "state.xml.h"
#include "state_hi.xml.h"
#include "cmdstream.xml.h"

static const struct platform_device_id gpu_ids[] = {
	{ .name = "etnaviv-gpu,2d" },
	{ },
};

/*
 * Driver functions:
 */

int etnaviv_gpu_get_param(struct etnaviv_gpu *gpu, u32 param, u64 *value)
{
	struct etnaviv_drm_private *priv = gpu->drm->dev_private;

	switch (param) {
	case ETNAVIV_PARAM_GPU_MODEL:
		*value = gpu->identity.model;
		break;

	case ETNAVIV_PARAM_GPU_REVISION:
		*value = gpu->identity.revision;
		break;

	case ETNAVIV_PARAM_GPU_FEATURES_0:
		*value = gpu->identity.features;
		break;

	case ETNAVIV_PARAM_GPU_FEATURES_1:
		*value = gpu->identity.minor_features0;
		break;

	case ETNAVIV_PARAM_GPU_FEATURES_2:
		*value = gpu->identity.minor_features1;
		break;

	case ETNAVIV_PARAM_GPU_FEATURES_3:
		*value = gpu->identity.minor_features2;
		break;

	case ETNAVIV_PARAM_GPU_FEATURES_4:
		*value = gpu->identity.minor_features3;
		break;

	case ETNAVIV_PARAM_GPU_FEATURES_5:
		*value = gpu->identity.minor_features4;
		break;

	case ETNAVIV_PARAM_GPU_FEATURES_6:
		*value = gpu->identity.minor_features5;
		break;

	case ETNAVIV_PARAM_GPU_FEATURES_7:
		*value = gpu->identity.minor_features6;
		break;

	case ETNAVIV_PARAM_GPU_FEATURES_8:
		*value = gpu->identity.minor_features7;
		break;

	case ETNAVIV_PARAM_GPU_FEATURES_9:
		*value = gpu->identity.minor_features8;
		break;

	case ETNAVIV_PARAM_GPU_FEATURES_10:
		*value = gpu->identity.minor_features9;
		break;

	case ETNAVIV_PARAM_GPU_FEATURES_11:
		*value = gpu->identity.minor_features10;
		break;

	case ETNAVIV_PARAM_GPU_FEATURES_12:
		*value = gpu->identity.minor_features11;
		break;

	case ETNAVIV_PARAM_GPU_STREAM_COUNT:
		*value = gpu->identity.stream_count;
		break;

	case ETNAVIV_PARAM_GPU_REGISTER_MAX:
		*value = gpu->identity.register_max;
		break;

	case ETNAVIV_PARAM_GPU_THREAD_COUNT:
		*value = gpu->identity.thread_count;
		break;

	case ETNAVIV_PARAM_GPU_VERTEX_CACHE_SIZE:
		*value = gpu->identity.vertex_cache_size;
		break;

	case ETNAVIV_PARAM_GPU_SHADER_CORE_COUNT:
		*value = gpu->identity.shader_core_count;
		break;

	case ETNAVIV_PARAM_GPU_PIXEL_PIPES:
		*value = gpu->identity.pixel_pipes;
		break;

	case ETNAVIV_PARAM_GPU_VERTEX_OUTPUT_BUFFER_SIZE:
		*value = gpu->identity.vertex_output_buffer_size;
		break;

	case ETNAVIV_PARAM_GPU_BUFFER_SIZE:
		*value = gpu->identity.buffer_size;
		break;

	case ETNAVIV_PARAM_GPU_INSTRUCTION_COUNT:
		*value = gpu->identity.instruction_count;
		break;

	case ETNAVIV_PARAM_GPU_NUM_CONSTANTS:
		*value = gpu->identity.num_constants;
		break;

	case ETNAVIV_PARAM_GPU_NUM_VARYINGS:
		*value = gpu->identity.varyings_count;
		break;

	case ETNAVIV_PARAM_SOFTPIN_START_ADDR:
		if (priv->mmu_global->version == ETNAVIV_IOMMU_V2)
			*value = ETNAVIV_SOFTPIN_START_ADDRESS;
		else
			*value = ~0ULL;
		break;

	case ETNAVIV_PARAM_GPU_PRODUCT_ID:
		*value = gpu->identity.product_id;
		break;

	case ETNAVIV_PARAM_GPU_CUSTOMER_ID:
		*value = gpu->identity.customer_id;
		break;

	case ETNAVIV_PARAM_GPU_ECO_ID:
		*value = gpu->identity.eco_id;
		break;

	case ETNAVIV_PARAM_GPU_NN_CORE_COUNT:
		*value = gpu->identity.nn_core_count;
		break;

	case ETNAVIV_PARAM_GPU_NN_MAD_PER_CORE:
		*value = gpu->identity.nn_mad_per_core;
		break;

	case ETNAVIV_PARAM_GPU_TP_CORE_COUNT:
		*value = gpu->identity.tp_core_count;
		break;

	case ETNAVIV_PARAM_GPU_ON_CHIP_SRAM_SIZE:
		*value = gpu->identity.on_chip_sram_size;
		break;

	case ETNAVIV_PARAM_GPU_AXI_SRAM_SIZE:
		*value = gpu->identity.axi_sram_size;
		break;

	case ETNAVIV_PARAM_GPU_AXI_SRAM_START:
		/*
		 * Only reports non-zero when the kernel coherent alloc
		 * succeeded AND the chip actually has AXI SRAM. Userspace
		 * treats 0 as "skip OCB_REMAP programming".
		 */
		if (priv->mmu_global && priv->mmu_global->axi_sram_cpu)
			*value = ETNAVIV_AXI_SRAM_VA;
		else
			*value = 0;
		break;

	default:
		DBG("%s: invalid param: %u", dev_name(gpu->dev), param);
		return -EINVAL;
	}

	return 0;
}

static int etnaviv_gpu_reset_deassert(struct etnaviv_gpu *gpu)
{
	int ret;

	/*
	 * 32 core clock cycles (slowest clock) required before deassertion
	 * 1 microsecond might match all implementations without computation
	 */
	usleep_range(1, 2);

	ret = reset_control_deassert(gpu->rst);
	if (ret)
		return ret;

	/*
	 * 128 core clock cycles (slowest clock) required before any activity on AHB
	 * 1 microsecond might match all implementations without computation
	 */
	usleep_range(1, 2);

	return 0;
}

static inline bool etnaviv_is_model_rev(struct etnaviv_gpu *gpu, u32 model, u32 revision)
{
	return gpu->identity.model == model &&
	       gpu->identity.revision == revision;
}

#define etnaviv_field(val, field) \
	(((val) & field##__MASK) >> field##__SHIFT)

static void etnaviv_hw_specs(struct etnaviv_gpu *gpu)
{
	if (gpu->identity.minor_features0 &
	    chipMinorFeatures0_MORE_MINOR_FEATURES) {
		u32 specs[4];
		unsigned int streams;

		specs[0] = gpu_read(gpu, VIVS_HI_CHIP_SPECS);
		specs[1] = gpu_read(gpu, VIVS_HI_CHIP_SPECS_2);
		specs[2] = gpu_read(gpu, VIVS_HI_CHIP_SPECS_3);
		specs[3] = gpu_read(gpu, VIVS_HI_CHIP_SPECS_4);

		gpu->identity.stream_count = etnaviv_field(specs[0],
					VIVS_HI_CHIP_SPECS_STREAM_COUNT);
		gpu->identity.register_max = etnaviv_field(specs[0],
					VIVS_HI_CHIP_SPECS_REGISTER_MAX);
		gpu->identity.thread_count = etnaviv_field(specs[0],
					VIVS_HI_CHIP_SPECS_THREAD_COUNT);
		gpu->identity.vertex_cache_size = etnaviv_field(specs[0],
					VIVS_HI_CHIP_SPECS_VERTEX_CACHE_SIZE);
		gpu->identity.shader_core_count = etnaviv_field(specs[0],
					VIVS_HI_CHIP_SPECS_SHADER_CORE_COUNT);
		gpu->identity.pixel_pipes = etnaviv_field(specs[0],
					VIVS_HI_CHIP_SPECS_PIXEL_PIPES);
		gpu->identity.vertex_output_buffer_size =
			etnaviv_field(specs[0],
				VIVS_HI_CHIP_SPECS_VERTEX_OUTPUT_BUFFER_SIZE);

		gpu->identity.buffer_size = etnaviv_field(specs[1],
					VIVS_HI_CHIP_SPECS_2_BUFFER_SIZE);
		gpu->identity.instruction_count = etnaviv_field(specs[1],
					VIVS_HI_CHIP_SPECS_2_INSTRUCTION_COUNT);
		gpu->identity.num_constants = etnaviv_field(specs[1],
					VIVS_HI_CHIP_SPECS_2_NUM_CONSTANTS);

		gpu->identity.varyings_count = etnaviv_field(specs[2],
					VIVS_HI_CHIP_SPECS_3_VARYINGS_COUNT);

		/* This overrides the value from older register if non-zero */
		streams = etnaviv_field(specs[3],
					VIVS_HI_CHIP_SPECS_4_STREAM_COUNT);
		if (streams)
			gpu->identity.stream_count = streams;
	}

	/* Fill in the stream count if not specified */
	if (gpu->identity.stream_count == 0) {
		if (gpu->identity.model >= 0x1000)
			gpu->identity.stream_count = 4;
		else
			gpu->identity.stream_count = 1;
	}

	/* Convert the register max value */
	if (gpu->identity.register_max)
		gpu->identity.register_max = 1 << gpu->identity.register_max;
	else if (gpu->identity.model == chipModel_GC400)
		gpu->identity.register_max = 32;
	else
		gpu->identity.register_max = 64;

	/* Convert thread count */
	if (gpu->identity.thread_count)
		gpu->identity.thread_count = 1 << gpu->identity.thread_count;
	else if (gpu->identity.model == chipModel_GC400)
		gpu->identity.thread_count = 64;
	else if (gpu->identity.model == chipModel_GC500 ||
		 gpu->identity.model == chipModel_GC530)
		gpu->identity.thread_count = 128;
	else
		gpu->identity.thread_count = 256;

	if (gpu->identity.vertex_cache_size == 0)
		gpu->identity.vertex_cache_size = 8;

	if (gpu->identity.shader_core_count == 0) {
		if (gpu->identity.model >= 0x1000)
			gpu->identity.shader_core_count = 2;
		else
			gpu->identity.shader_core_count = 1;
	}

	if (gpu->identity.pixel_pipes == 0)
		gpu->identity.pixel_pipes = 1;

	/* Convert virtex buffer size */
	if (gpu->identity.vertex_output_buffer_size) {
		gpu->identity.vertex_output_buffer_size =
			1 << gpu->identity.vertex_output_buffer_size;
	} else if (gpu->identity.model == chipModel_GC400) {
		if (gpu->identity.revision < 0x4000)
			gpu->identity.vertex_output_buffer_size = 512;
		else if (gpu->identity.revision < 0x4200)
			gpu->identity.vertex_output_buffer_size = 256;
		else
			gpu->identity.vertex_output_buffer_size = 128;
	} else {
		gpu->identity.vertex_output_buffer_size = 512;
	}

	switch (gpu->identity.instruction_count) {
	case 0:
		if (etnaviv_is_model_rev(gpu, 0x2000, 0x5108) ||
		    gpu->identity.model == chipModel_GC880)
			gpu->identity.instruction_count = 512;
		else
			gpu->identity.instruction_count = 256;
		break;

	case 1:
		gpu->identity.instruction_count = 1024;
		break;

	case 2:
		gpu->identity.instruction_count = 2048;
		break;

	default:
		gpu->identity.instruction_count = 256;
		break;
	}

	if (gpu->identity.num_constants == 0)
		gpu->identity.num_constants = 168;

	if (gpu->identity.varyings_count == 0) {
		if (gpu->identity.minor_features1 & chipMinorFeatures1_HALTI0)
			gpu->identity.varyings_count = 12;
		else
			gpu->identity.varyings_count = 8;
	}

	/*
	 * For some cores, two varyings are consumed for position, so the
	 * maximum varying count needs to be reduced by one.
	 */
	if (etnaviv_is_model_rev(gpu, 0x5000, 0x5434) ||
	    etnaviv_is_model_rev(gpu, 0x4000, 0x5222) ||
	    etnaviv_is_model_rev(gpu, 0x4000, 0x5245) ||
	    etnaviv_is_model_rev(gpu, 0x4000, 0x5208) ||
	    etnaviv_is_model_rev(gpu, 0x3000, 0x5435) ||
	    etnaviv_is_model_rev(gpu, 0x2200, 0x5244) ||
	    etnaviv_is_model_rev(gpu, 0x2100, 0x5108) ||
	    etnaviv_is_model_rev(gpu, 0x2000, 0x5108) ||
	    etnaviv_is_model_rev(gpu, 0x1500, 0x5246) ||
	    etnaviv_is_model_rev(gpu, 0x880, 0x5107) ||
	    etnaviv_is_model_rev(gpu, 0x880, 0x5106))
		gpu->identity.varyings_count -= 1;
}

static void etnaviv_hw_identify(struct etnaviv_gpu *gpu)
{
	u32 chipIdentity;

	chipIdentity = gpu_read(gpu, VIVS_HI_CHIP_IDENTITY);
	dev_dbg(gpu->dev, "hw_identify: CHIP_IDENTITY=0x%08x\n", chipIdentity);

	/* Special case for older graphic cores. */
	if (etnaviv_field(chipIdentity, VIVS_HI_CHIP_IDENTITY_FAMILY) == 0x01) {
		gpu->identity.model    = chipModel_GC500;
		gpu->identity.revision = etnaviv_field(chipIdentity,
					 VIVS_HI_CHIP_IDENTITY_REVISION);
	} else {
		u32 chipDate = gpu_read(gpu, VIVS_HI_CHIP_DATE);

		gpu->identity.model = gpu_read(gpu, VIVS_HI_CHIP_MODEL);
		gpu->identity.revision = gpu_read(gpu, VIVS_HI_CHIP_REV);
		gpu->identity.customer_id = gpu_read(gpu, VIVS_HI_CHIP_CUSTOMER_ID);

		/*
		 * Reading these two registers on GC600 rev 0x19 result in a
		 * unhandled fault: external abort on non-linefetch
		 */
		if (!etnaviv_is_model_rev(gpu, 0x600, 0x19)) {
			gpu->identity.product_id = gpu_read(gpu, VIVS_HI_CHIP_PRODUCT_ID);
			gpu->identity.eco_id = gpu_read(gpu, VIVS_HI_CHIP_ECO_ID);
		}

		/*
		 * !!!! HACK ALERT !!!!
		 * Because people change device IDs without letting software
		 * know about it - here is the hack to make it all look the
		 * same.  Only for GC400 family.
		 */
		if ((gpu->identity.model & 0xff00) == 0x0400 &&
		    gpu->identity.model != chipModel_GC420) {
			gpu->identity.model = gpu->identity.model & 0x0400;
		}

		/* Another special case */
		if (etnaviv_is_model_rev(gpu, 0x300, 0x2201)) {
			u32 chipTime = gpu_read(gpu, VIVS_HI_CHIP_TIME);

			if (chipDate == 0x20080814 && chipTime == 0x12051100) {
				/*
				 * This IP has an ECO; put the correct
				 * revision in it.
				 */
				gpu->identity.revision = 0x1051;
			}
		}

		/*
		 * NXP likes to call the GPU on the i.MX6QP GC2000+, but in
		 * reality it's just a re-branded GC3000. We can identify this
		 * core by the upper half of the revision register being all 1.
		 * Fix model/rev here, so all other places can refer to this
		 * core by its real identity.
		 */
		if (etnaviv_is_model_rev(gpu, 0x2000, 0xffff5450)) {
			gpu->identity.model = chipModel_GC3000;
			gpu->identity.revision &= 0xffff;
		}

		if (etnaviv_is_model_rev(gpu, 0x1000, 0x5037) && (chipDate == 0x20120617))
			gpu->identity.eco_id = 1;

		if (etnaviv_is_model_rev(gpu, 0x320, 0x5303) && (chipDate == 0x20140511))
			gpu->identity.eco_id = 1;
	}

	dev_info(gpu->dev, "model: GC%x, revision: %x\n",
		 gpu->identity.model, gpu->identity.revision);

	gpu->idle_mask = ~VIVS_HI_IDLE_STATE_AXI_LP;
	/*
	 * If there is a match in the HWDB, we aren't interested in the
	 * remaining register values, as they might be wrong.
	 */
	if (etnaviv_fill_identity_from_hwdb(gpu))
		return;

	gpu->identity.features = gpu_read(gpu, VIVS_HI_CHIP_FEATURE);

	/* Disable fast clear on GC700. */
	if (gpu->identity.model == chipModel_GC700)
		gpu->identity.features &= ~chipFeatures_FAST_CLEAR;

	/* These models/revisions don't have the 2D pipe bit */
	if ((gpu->identity.model == chipModel_GC500 &&
	     gpu->identity.revision <= 2) ||
	    gpu->identity.model == chipModel_GC300)
		gpu->identity.features |= chipFeatures_PIPE_2D;

	if ((gpu->identity.model == chipModel_GC500 &&
	     gpu->identity.revision < 2) ||
	    (gpu->identity.model == chipModel_GC300 &&
	     gpu->identity.revision < 0x2000)) {

		/*
		 * GC500 rev 1.x and GC300 rev < 2.0 doesn't have these
		 * registers.
		 */
		gpu->identity.minor_features0 = 0;
		gpu->identity.minor_features1 = 0;
		gpu->identity.minor_features2 = 0;
		gpu->identity.minor_features3 = 0;
		gpu->identity.minor_features4 = 0;
		gpu->identity.minor_features5 = 0;
	} else
		gpu->identity.minor_features0 =
				gpu_read(gpu, VIVS_HI_CHIP_MINOR_FEATURE_0);

	if (gpu->identity.minor_features0 &
	    chipMinorFeatures0_MORE_MINOR_FEATURES) {
		gpu->identity.minor_features1 =
				gpu_read(gpu, VIVS_HI_CHIP_MINOR_FEATURE_1);
		gpu->identity.minor_features2 =
				gpu_read(gpu, VIVS_HI_CHIP_MINOR_FEATURE_2);
		gpu->identity.minor_features3 =
				gpu_read(gpu, VIVS_HI_CHIP_MINOR_FEATURE_3);
		gpu->identity.minor_features4 =
				gpu_read(gpu, VIVS_HI_CHIP_MINOR_FEATURE_4);
		gpu->identity.minor_features5 =
				gpu_read(gpu, VIVS_HI_CHIP_MINOR_FEATURE_5);
	}

	/* GC600/300 idle register reports zero bits where modules aren't present */
	if (gpu->identity.model == chipModel_GC600 ||
	    gpu->identity.model == chipModel_GC300)
		gpu->idle_mask = VIVS_HI_IDLE_STATE_TX |
				 VIVS_HI_IDLE_STATE_RA |
				 VIVS_HI_IDLE_STATE_SE |
				 VIVS_HI_IDLE_STATE_PA |
				 VIVS_HI_IDLE_STATE_SH |
				 VIVS_HI_IDLE_STATE_PE |
				 VIVS_HI_IDLE_STATE_DE |
				 VIVS_HI_IDLE_STATE_FE;

	etnaviv_hw_specs(gpu);
}

static void etnaviv_gpu_load_clock(struct etnaviv_gpu *gpu, u32 clock)
{
	gpu_write(gpu, VIVS_HI_CLOCK_CONTROL, clock |
		  VIVS_HI_CLOCK_CONTROL_FSCALE_CMD_LOAD);
	gpu_write(gpu, VIVS_HI_CLOCK_CONTROL, clock);
}

static void etnaviv_gpu_update_clock(struct etnaviv_gpu *gpu)
{
	if (gpu->identity.minor_features2 &
	    chipMinorFeatures2_DYNAMIC_FREQUENCY_SCALING) {
		clk_set_rate(gpu->clk_core,
			     gpu->base_rate_core >> gpu->freq_scale);
		clk_set_rate(gpu->clk_shader,
			     gpu->base_rate_shader >> gpu->freq_scale);
	} else {
		unsigned int fscale = 1 << (6 - gpu->freq_scale);
		u32 clock = gpu_read(gpu, VIVS_HI_CLOCK_CONTROL);

		clock &= ~VIVS_HI_CLOCK_CONTROL_FSCALE_VAL__MASK;
		clock |= VIVS_HI_CLOCK_CONTROL_FSCALE_VAL(fscale);
		etnaviv_gpu_load_clock(gpu, clock);
	}

	/*
	 * Choose number of wait cycles to target a ~30us (1/32768) max latency
	 * until new work is picked up by the FE when it polls in the idle loop.
	 * If the GPU base frequency is unknown use 200 wait cycles.
	 */
	gpu->fe_waitcycles = clamp(gpu->base_rate_core >> (15 - gpu->freq_scale),
				   200UL, 0xffffUL);
}

static int etnaviv_hw_reset(struct etnaviv_gpu *gpu)
{
	u32 control, idle;
	unsigned long timeout;
	bool failed = true;

	/* We hope that the GPU resets in under one second */
	timeout = jiffies + msecs_to_jiffies(1000);

	while (time_is_after_jiffies(timeout)) {
		unsigned int fscale = 1 << (6 - gpu->freq_scale);
		u32 pulse_eater = 0x01590880;

		/* disable clock gating */
		gpu_write_power(gpu, VIVS_PM_POWER_CONTROLS, 0x0);

		/* disable pulse eater */
		pulse_eater |= BIT(17);
		gpu_write_power(gpu, VIVS_PM_PULSE_EATER, pulse_eater);
		pulse_eater |= BIT(0);
		gpu_write_power(gpu, VIVS_PM_PULSE_EATER, pulse_eater);

		/* enable clock */
		control = VIVS_HI_CLOCK_CONTROL_FSCALE_VAL(fscale);
		etnaviv_gpu_load_clock(gpu, control);

		/* isolate the GPU. */
		control |= VIVS_HI_CLOCK_CONTROL_ISOLATE_GPU;
		gpu_write(gpu, VIVS_HI_CLOCK_CONTROL, control);

		if (gpu->sec_mode == ETNA_SEC_KERNEL) {
			gpu_write(gpu, VIVS_MMUv2_AHB_CONTROL,
			          VIVS_MMUv2_AHB_CONTROL_RESET);
		} else {
			/* set soft reset. */
			control |= VIVS_HI_CLOCK_CONTROL_SOFT_RESET;
			gpu_write(gpu, VIVS_HI_CLOCK_CONTROL, control);
		}

		/* wait for reset. */
		usleep_range(10, 20);

		/* reset soft reset bit. */
		control &= ~VIVS_HI_CLOCK_CONTROL_SOFT_RESET;
		gpu_write(gpu, VIVS_HI_CLOCK_CONTROL, control);

		/* reset GPU isolation. */
		control &= ~VIVS_HI_CLOCK_CONTROL_ISOLATE_GPU;
		gpu_write(gpu, VIVS_HI_CLOCK_CONTROL, control);

		/* read idle register. */
		idle = gpu_read(gpu, VIVS_HI_IDLE_STATE);

		/* try resetting again if FE is not idle */
		if ((idle & VIVS_HI_IDLE_STATE_FE) == 0) {
			dev_dbg(gpu->dev, "FE is not idle\n");
			continue;
		}

		/* read reset register. */
		control = gpu_read(gpu, VIVS_HI_CLOCK_CONTROL);

		/* is the GPU idle? */
		if (((control & VIVS_HI_CLOCK_CONTROL_IDLE_3D) == 0) ||
		    ((control & VIVS_HI_CLOCK_CONTROL_IDLE_2D) == 0)) {
			dev_dbg(gpu->dev, "GPU is not idle\n");
			continue;
		}

		/* enable debug register access */
		control &= ~VIVS_HI_CLOCK_CONTROL_DISABLE_DEBUG_REGISTERS;
		gpu_write(gpu, VIVS_HI_CLOCK_CONTROL, control);

		failed = false;
		break;
	}

	if (failed) {
		idle = gpu_read(gpu, VIVS_HI_IDLE_STATE);
		control = gpu_read(gpu, VIVS_HI_CLOCK_CONTROL);

		dev_err(gpu->dev, "GPU failed to reset: FE %sidle, 3D %sidle, 2D %sidle\n",
			idle & VIVS_HI_IDLE_STATE_FE ? "" : "not ",
			control & VIVS_HI_CLOCK_CONTROL_IDLE_3D ? "" : "not ",
			control & VIVS_HI_CLOCK_CONTROL_IDLE_2D ? "" : "not ");

		return -EBUSY;
	}

	/* We rely on the GPU running, so program the clock */
	etnaviv_gpu_update_clock(gpu);

	gpu->state = ETNA_GPU_STATE_RESET;
	gpu->exec_state = -1;
	if (gpu->mmu_context)
		etnaviv_iommu_context_put(gpu->mmu_context);
	gpu->mmu_context = NULL;

	return 0;
}

static void etnaviv_gpu_enable_mlcg(struct etnaviv_gpu *gpu)
{
	u32 pmc, ppc;

	/* enable clock gating */
	ppc = gpu_read_power(gpu, VIVS_PM_POWER_CONTROLS);
	ppc |= VIVS_PM_POWER_CONTROLS_ENABLE_MODULE_CLOCK_GATING;

	/* Disable stall module clock gating for 4.3.0.1 and 4.3.0.2 revs */
	if (gpu->identity.revision == 0x4301 ||
	    gpu->identity.revision == 0x4302)
		ppc |= VIVS_PM_POWER_CONTROLS_DISABLE_STALL_MODULE_CLOCK_GATING;

	gpu_write_power(gpu, VIVS_PM_POWER_CONTROLS, ppc);

	pmc = gpu_read_power(gpu, VIVS_PM_MODULE_CONTROLS);

	/* Disable PA clock gating for GC400+ without bugfix except for GC420 */
	if (gpu->identity.model >= chipModel_GC400 &&
	    gpu->identity.model != chipModel_GC420 &&
	    !(gpu->identity.minor_features3 & chipMinorFeatures3_BUG_FIXES12))
		pmc |= VIVS_PM_MODULE_CONTROLS_DISABLE_MODULE_CLOCK_GATING_PA;

	/*
	 * Disable PE clock gating on revs < 5.0.0.0 when HZ is
	 * present without a bug fix.
	 */
	if (gpu->identity.revision < 0x5000 &&
	    gpu->identity.minor_features0 & chipMinorFeatures0_HZ &&
	    !(gpu->identity.minor_features1 &
	      chipMinorFeatures1_DISABLE_PE_GATING))
		pmc |= VIVS_PM_MODULE_CONTROLS_DISABLE_MODULE_CLOCK_GATING_PE;

	if (gpu->identity.revision < 0x5422)
		pmc |= BIT(15); /* Unknown bit */

	/* Disable TX clock gating on affected core revisions. */
	if (etnaviv_is_model_rev(gpu, 0x4000, 0x5222) ||
	    etnaviv_is_model_rev(gpu, 0x2000, 0x5108) ||
	    etnaviv_is_model_rev(gpu, 0x7000, 0x6202) ||
	    etnaviv_is_model_rev(gpu, 0x7000, 0x6203))
		pmc |= VIVS_PM_MODULE_CONTROLS_DISABLE_MODULE_CLOCK_GATING_TX;

	/* Disable SE and RA clock gating on affected core revisions. */
	if (etnaviv_is_model_rev(gpu, 0x7000, 0x6202))
		pmc |= VIVS_PM_MODULE_CONTROLS_DISABLE_MODULE_CLOCK_GATING_SE |
		       VIVS_PM_MODULE_CONTROLS_DISABLE_MODULE_CLOCK_GATING_RA;

	/* Disable SH_EU clock gating on affected core revisions. */
	if (etnaviv_is_model_rev(gpu, 0x8000, 0x7200) ||
	    etnaviv_is_model_rev(gpu, 0x8000, 0x8002) ||
	    etnaviv_is_model_rev(gpu, 0x9200, 0x6304))
		pmc |= VIVS_PM_MODULE_CONTROLS_DISABLE_MODULE_CLOCK_GATING_SH_EU;

	pmc |= VIVS_PM_MODULE_CONTROLS_DISABLE_MODULE_CLOCK_GATING_RA_HZ;
	pmc |= VIVS_PM_MODULE_CONTROLS_DISABLE_MODULE_CLOCK_GATING_RA_EZ;

	/* Disable NN clock gating on NPU cores to prevent FE register write stalls */
	if (gpu->identity.nn_core_count > 0)
		pmc |= VIVS_PM_MODULE_CONTROLS_DISABLE_MODULE_CLOCK_GATING_NN;

	gpu_write_power(gpu, VIVS_PM_MODULE_CONTROLS, pmc);
}

void etnaviv_gpu_start_fe(struct etnaviv_gpu *gpu, u32 address, u16 prefetch)
{
	gpu_write(gpu, VIVS_FE_COMMAND_ADDRESS, address);
	gpu_write(gpu, VIVS_FE_COMMAND_CONTROL,
		  VIVS_FE_COMMAND_CONTROL_ENABLE |
		  VIVS_FE_COMMAND_CONTROL_PREFETCH(prefetch));

	if (gpu->sec_mode == ETNA_SEC_KERNEL) {
		gpu_write(gpu, VIVS_MMUv2_SEC_COMMAND_CONTROL,
			  VIVS_MMUv2_SEC_COMMAND_CONTROL_ENABLE |
			  VIVS_MMUv2_SEC_COMMAND_CONTROL_PREFETCH(prefetch));
	}
}

/*
 * NN flop reset: run a minimal NN convolution to initialize hardware state.
 * This is required on NPU-only devices (VIPNano/VIPPico) to reset internal
 * flip-flop state. Based on galcore's gckHARDWARE_ResetFlopWithNN.
 *
 * The nn_data buffer layout (GPU VA mapped via sualloc):
 *   0x000: NN instruction (128 bytes)
 *   0x100: NN kernel coefficients (128 bytes)
 *   0x200: NN input data (64 bytes)
 *   0x300: NN output data (64 bytes)
 */

/* PS/PA/GL_API_MODE registers used by NPU paths -- definitions live in state.xml.h */

/* BSP NN instruction data for VIPPico v3 (customerID 0x99, INT8) */
static const u32 nn_flop_ins[32] = {
	0xA0100048, 0x001000C0, 0xC8000000, 0x00080080,
	0x02084001, 0x037FD200, 0xDFF6F000, 0xDFF6E000,
	0x00012004, 0x00000000, 0x00000000, 0x00000800,
	0x00000880, 0x00000000, 0x00000800, 0x00000000,
	0x00020003, 0x01000002, 0x00000000, 0x03FFFFFF,
	0x00000000, 0x03FFFFFF, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000
};

/* BSP NN kernel coefficients for VIPPico v3 (2x2x1 INT8 conv) */
static const u32 nn_flop_ker[32] = {
	0x01020411, 0x00000100, 0x00000000, 0x00000000,
	0x00000000, 0x26540781, 0x00000000, 0x00000069,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x0073FFE0, 0xC0003800, 0x0000000F, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000
};

/*
 * A311D (VIPNano-QI, customer 0x88) flop-reset template: a minimal 1x1 conv
 * (out 4x4x1) captured from a REAL mesa-emitted A311D NN_INST and verified to
 * execute on the A311D NN engine (audit 2026-06-14-inception-concatfix (y2)),
 * so the encoding is engine-valid -- the 0x88 NN format differs from the 0x99
 * nn_flop_ins above. inst[5/6/7] (kernel/in/out addr) are patched to the
 * flop-reset buffers, identical to the 0x99 path.
 */
static const u32 nn_flop_ins_a311d[32] = {
	0xa0100048, 0x00200100, 0xb8000000, 0x00200100,
	0x08104001, 0x03fffc00, 0xffff4000, 0xffff3000,
	0x00042004, 0x00000000, 0x00000000, 0x00000800,
	0x00000a00, 0x00000000, 0x00000800, 0x00000200,
	0x00040004, 0x00000004, 0x00000000, 0x03ffffff,
	0x00000000, 0x03ffffff, 0x00008080, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x03ffffff, 0x7f800000, 0xff800000, 0x00000000
};
/* A311D 1x1 kernel coefficients (mesa-compressed, 19-u32 nonzero prefix). */
static const u32 nn_flop_ker_a311d[19] = {
	0x00000040, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x6b000100, 0x00000a80, 0x00808080
};

/*
 * nn_flop_node / tp_flop_node are inserted into whichever CLIENT context is
 * current at the first idleloop after a load and are deliberately never
 * removed (the NN/TP engines retain state referencing those VAs).  A drm_mm
 * node must not outlive its drm_mm, and a client context dies when its last
 * reference goes -- at etnaviv_postclose(), at the context switch in
 * etnaviv_iommuv2_restore() once a recovery's etnaviv_hw_reset() has dropped
 * the GPU's own reference, or at unload -- so the GPU holds a reference on
 * the hosting context for the lifetime of the node and releases node,
 * mapping and reference together in etnaviv_gpu_unbind().  Before this every
 * such teardown tripped drm_mm_takedown()'s "Memory manager not clean during
 * takedown" WARN (TAINT_WARN, taint 1026 -> 1538) on the shipped driver.
 */
static void etnaviv_gpu_flop_node_pin(struct etnaviv_iommu_context **slot,
				      struct etnaviv_iommu_context *context)
{
	if (!*slot)
		*slot = etnaviv_iommu_context_get(context);
}

static void etnaviv_gpu_flop_node_release(struct etnaviv_gpu *gpu,
					  struct etnaviv_iommu_context **slot,
					  struct drm_mm_node *node)
{
	struct etnaviv_iommu_context *context = *slot;
	u64 off;

	if (!context)
		return;

	mutex_lock(&context->lock);
	if (drm_mm_node_allocated(node)) {
		/* the flop-reset pages were mapped one SZ_4K at a time */
		for (off = 0; off < node->size; off += SZ_4K)
			context->global->ops->unmap(context, node->start + off,
						    SZ_4K);
		dev_printk(KERN_DEBUG, gpu->dev,
			   "flop node 0x%08llx+0x%llx released with its context since qi:N1510\n",
			   (unsigned long long)node->start,
			   (unsigned long long)node->size);
		drm_mm_remove_node(node);
	}
	mutex_unlock(&context->lock);

	etnaviv_iommu_context_put(context);
	*slot = NULL;
}

static void etnaviv_gpu_nn_flop_reset(struct etnaviv_gpu *gpu)
{
	struct etnaviv_cmdbuf *buffer = &gpu->buffer;
	struct etnaviv_iommu_context *context = gpu->mmu_context;
	struct drm_mm_node *nn_node = &gpu->nn_flop_node;
	void *data_cpu = gpu->nn_data_vaddr;
	dma_addr_t data_paddr = gpu->nn_data_paddr;
	u32 data_va, inst_va, ker_va, in_va, out_va;
	u32 *inst;
	u16 prefetch;
	u32 cmd_va;
	int i, ret;

	if (gpu->identity.nn_core_count == 0 || !data_cpu)
		return;

	/* NN flop reset: 0x99 (VIPPico) and 0x88 (VIPNano-QI/A311D) use DIFFERENT
	 * NN instruction encodings; each has its own captured template
	 * (nn_flop_ins / nn_flop_ins_a311d). Any other customer is unsupported.
	 */
	if (gpu->identity.customer_id != 0x99 &&
	    gpu->identity.customer_id != 0x88)
		return;

	/*
	 * Flop reset is a one-time hardware init. The NN engine retains
	 * register state from this reset and must not be re-initialized
	 * on every context switch -- the reserved VAs are stored in the
	 * gpu struct so they persist across client contexts. Use
	 * drm_mm_node_allocated() to detect previous success (handles
	 * partial completion where TP reset failed on a prior call).
	 */
	if (drm_mm_node_allocated(nn_node))
		return;

	/*
	 * Map the NN data page into GPU VA space above the VIP SRAM window.
	 * SRAM_REMAP_ADDRESS=0x400000 makes VAs [0x400000, 0x400000+sram_size)
	 * map to VIP SRAM instead of the MMU. Our data must be above this
	 * window so the NN engine reads/writes DDR via MMU, not SRAM.
	 */
	{
		u32 sram_end = 0x00400000 + gpu->identity.on_chip_sram_size;

		/* Align up to page boundary */
		sram_end = ALIGN(sram_end, PAGE_SIZE);
		if (sram_end < ETNAVIV_SOFTPIN_START_ADDRESS)
			sram_end = ETNAVIV_SOFTPIN_START_ADDRESS;

		mutex_lock(&context->lock);
		ret = drm_mm_insert_node_in_range(&context->mm, nn_node,
						  PAGE_SIZE, 0, 0,
						  sram_end,
						  U64_MAX, DRM_MM_INSERT_LOW);
	}
	mutex_unlock(&context->lock);
	if (ret) {
		dev_err(gpu->dev, "NN flop reset: VA alloc failed: %d\n", ret);
		return;
	}

	data_va = nn_node->start;
	ret = context->global->ops->map(context, data_va, data_paddr,
					PAGE_SIZE,
					ETNAVIV_PROT_READ | ETNAVIV_PROT_WRITE);
	if (ret) {
		dev_err(gpu->dev, "NN flop reset: IOMMU map failed: %d\n", ret);
		goto out_remove;
	}

	inst_va = data_va + 0x000;
	ker_va  = data_va + 0x100;
	in_va   = data_va + 0x200;
	out_va  = data_va + 0x300;

	/* Copy the per-chip NN instruction template and patch addresses.
	 * DW5/6/7 = kernel/in/out address are at the same offsets for both
	 * encodings, so the patching is shared. */
	inst = data_cpu;
	if (gpu->identity.customer_id == 0x88)
		memcpy(inst, nn_flop_ins_a311d, sizeof(nn_flop_ins_a311d));
	else
		memcpy(inst, nn_flop_ins, sizeof(nn_flop_ins));
	inst[5] = (ker_va >> 6) & 0x03FFFFFF;
	inst[6] = in_va;
	inst[7] = out_va;

	/* Copy NN kernel coefficients at offset 0x100 (per-chip). */
	if (gpu->identity.customer_id == 0x88)
		memcpy(data_cpu + 0x100, nn_flop_ker_a311d,
		       sizeof(nn_flop_ker_a311d));
	else
		memcpy(data_cpu + 0x100, nn_flop_ker, sizeof(nn_flop_ker));

	/* Fill input at offset 0x200: 6 bytes of 0x33 (3x2x1 INT8) */
	memset(data_cpu + 0x200, 0x33, 64);

	/* Pre-fill output at offset 0x300 with 0xAA to detect NN writes */
	memset(data_cpu + 0x300, 0xAA, 64);

	wmb();

	dev_info(gpu->dev,
		 "NN flop reset: inst[0..7]=%08x %08x %08x %08x %08x %08x %08x %08x\n",
		 inst[0], inst[1], inst[2], inst[3],
		 inst[4], inst[5], inst[6], inst[7]);
	dev_info(gpu->dev,
		 "NN flop reset: inst[8..15]=%08x %08x %08x %08x %08x %08x %08x %08x\n",
		 inst[8], inst[9], inst[10], inst[11],
		 inst[12], inst[13], inst[14], inst[15]);
	dev_info(gpu->dev,
		 "NN flop reset: data_paddr=0x%llx data_va=0x%08x\n",
		 (u64)data_paddr, data_va);

	/*
	 * Build the NNCmd in the ring buffer.
	 * SRAM_REMAP_ADDRESS=0x400000: VAs 0-0x3FFFFF → VIP SRAM,
	 * VAs >= 0x400000 → MMU. NN cache uses SRAM, data uses MMU.
	 *
	 * Note: GL registers like SRAM_REMAP and NN_CONFIG cannot be read
	 * via AHB on this chip (sync abort), but can be written via FE
	 * LOAD_STATE commands.
	 */
#define LS(reg, val) do {					\
	cmd[i++] = VIV_FE_LOAD_STATE_HEADER_OP_LOAD_STATE |	\
		   VIV_FE_LOAD_STATE_HEADER_COUNT(1) |		\
		   VIV_FE_LOAD_STATE_HEADER_OFFSET(		\
			(reg) >> VIV_FE_LOAD_STATE_HEADER_OFFSET__SHR); \
	cmd[i++] = (val);					\
} while (0)

	{
		u32 *cmd = buffer->vaddr;

		i = 0;
		/*
		 * HI_REORDER fix: set bit 6 of reg 0x0090 for NN chips
		 * without AXI SRAM. Cannot read this register via AHB
		 * (sync abort), so write via FE LOAD_STATE. After reset
		 * the register is 0, so just write 0x40.
		 */
		LS(VIVS_HI_COMPRESSION_FLAGS, 0x40);
		/*
		 * Flush MMU TLB: new IOMMU mapping was just created
		 * for the NN data page. Without this, the GPU's TLB
		 * may not see the mapping and NN reads/writes go to
		 * the safe address.  On 0x99 the MMU walks the master TLB
		 * latched at restore: sync it now that the data page is
		 * mapped (the context's own address elsewhere).
		 */
		LS(VIVS_MMUv2_CONFIGURATION,
		   VIVS_MMUv2_CONFIGURATION_MODE_MASK |
		   VIVS_MMUv2_CONFIGURATION_FLUSH_FLUSH |
		   etnaviv_iommuv2_sync_hw_mtlb(context));
		/* WAIT for TLB flush to complete (no PE for STALL) */
		cmd[i++] = VIV_FE_WAIT_HEADER_OP_WAIT | 32000;
		cmd[i++] = 0; /* pad to 64-bit alignment */
		/*
		 * PA_SYSTEM_MODE and GL_API_MODE must be set before
		 * NN_INST_ADDR — without OPENCL mode the NN engine
		 * ignores NN_INST_ADDR writes entirely.
		 */
		LS(VIVS_PA_SYSTEM_MODE, 0x11);
		LS(VIVS_GL_API_MODE, VIVS_GL_API_MODE_OPENCL);
		LS(VIVS_GL_SRAM_REMAP_ADDRESS, 0x00400000);
		LS(VIVS_GL_SRAM_REMAP_ADDRESS, 0x00400000);
		LS(VIVS_GL_OCB_REMAP_START, 0x0);
		LS(VIVS_GL_OCB_REMAP_END, 0x0);
		/* A311D (0x88) uses FE-stall at NN_INST, NOT SMALL_BATCH (the
		 * 0x99/VIPPico completion mode). With SMALL_BATCH off, the FE
		 * naturally waits for the NN op before the END. */
		LS(VIVS_GL_NN_CONFIG,
		   etnaviv_npu_nn_config(gpu,
			gpu->identity.customer_id == 0x88 ?
			0 : VIVS_GL_NN_CONFIG_SMALL_BATCH));
		LS(VIVS_GL_UNK03950, 0x0);
		LS(VIVS_GL_SHADER_INDEX, 0x0);
		LS(VIVS_PS_NN_INST_ADDR, (inst_va >> 6) << 6);
		LS(VIVS_PS_UNK10A4, 0x0);
		/*
		 * No FLUSH_CACHE here! SMALL_BATCH doesn't stall the
		 * FE on NN_INST_ADDR on this chip, so FE would race
		 * ahead to FLUSH_CACHE while the NN is still running,
		 * causing the SH/FE to hang permanently. Instead we
		 * END here, poll for NN idle, then issue FLUSH_CACHE
		 * in a separate FE execution.
		 */
		cmd[i++] = VIV_FE_END_HEADER_OP_END;
		cmd[i++] = 0;

		buffer->user_size = i * 4;
	}
#undef LS

	prefetch = buffer->user_size / 8;
	cmd_va = etnaviv_cmdbuf_get_va(buffer,
				       &gpu->mmu_context->cmdbuf_mapping);

	/* Wait for FE to be fully idle before kicking */
	if (etnaviv_gpu_wait_idle(gpu, 200)) {
		dev_warn(gpu->dev,
			 "NN flop reset: FE not idle before kick, idle=0x%08x\n",
			 gpu_read(gpu, VIVS_HI_IDLE_STATE));
	}

	{
		u32 *cmd = buffer->vaddr;
		dev_info(gpu->dev,
			 "NN flop reset: cmd_va=0x%08x data_va=0x%08x idle=0x%08x prefetch=%u\n",
			 cmd_va, data_va, gpu_read(gpu, VIVS_HI_IDLE_STATE), prefetch);
		dev_info(gpu->dev,
			 "NN flop reset: cmd[0..7]=%08x %08x %08x %08x %08x %08x %08x %08x\n",
			 cmd[0], cmd[1], cmd[2], cmd[3],
			 cmd[4], cmd[5], cmd[6], cmd[7]);
	}

	etnaviv_gpu_start_fe(gpu, cmd_va, prefetch);

	/*
	 * Poll for NN idle. SMALL_BATCH doesn't stall the FE on this
	 * chip, so the FE reaches END while the NN is still running.
	 * We must wait for the NN to complete before issuing FLUSH_CACHE.
	 */
	for (i = 0; i < 50000; i++) {
		u32 idle = gpu_read(gpu, VIVS_HI_IDLE_STATE);

		if (idle == 0x7FFFFFFF)
			break;
		udelay(10);
	}

	{
		u32 idle = gpu_read(gpu, VIVS_HI_IDLE_STATE);
		u32 dma = gpu_read(gpu, VIVS_FE_DMA_ADDRESS);
		u8 *out = data_cpu + 0x300;

		dev_info(gpu->dev,
			 "NN flop reset: after NN idle=0x%08x dma=0x%08x loops=%d out=[%02x %02x %02x %02x %02x %02x %02x %02x]\n",
			 idle, dma, i, out[0], out[1], out[2], out[3],
			 out[4], out[5], out[6], out[7]);

		if (idle != 0x7FFFFFFF) {
			dev_warn(gpu->dev,
				 "NN flop reset: NN did not complete, mmu_status=0x%08x exc[0..3]=%08x %08x %08x %08x\n",
				 gpu_read(gpu, VIVS_MMUv2_STATUS),
				 gpu_read(gpu, VIVS_MMUv2_EXCEPTION_ADDR(0)),
				 gpu_read(gpu, VIVS_MMUv2_EXCEPTION_ADDR(1)),
				 gpu_read(gpu, VIVS_MMUv2_EXCEPTION_ADDR(2)),
				 gpu_read(gpu, VIVS_MMUv2_EXCEPTION_ADDR(3)));
			goto out_unmap;
		}
	}

	/*
	 * NN completed. Now issue FLUSH_CACHE in a separate FE
	 * execution — safe because SH is idle now.
	 */
	{
		u32 *cmd = buffer->vaddr;

		i = 0;
		cmd[i++] = VIV_FE_LOAD_STATE_HEADER_OP_LOAD_STATE |
			   VIV_FE_LOAD_STATE_HEADER_COUNT(1) |
			   VIV_FE_LOAD_STATE_HEADER_OFFSET(
				VIVS_GL_FLUSH_CACHE >> VIV_FE_LOAD_STATE_HEADER_OFFSET__SHR);
		cmd[i++] = 0x00000C23;
		cmd[i++] = VIV_FE_LOAD_STATE_HEADER_OP_LOAD_STATE |
			   VIV_FE_LOAD_STATE_HEADER_COUNT(1) |
			   VIV_FE_LOAD_STATE_HEADER_OFFSET(
				VIVS_GL_FLUSH_CACHE >> VIV_FE_LOAD_STATE_HEADER_OFFSET__SHR);
		cmd[i++] = 0x00000C23;
		cmd[i++] = VIV_FE_END_HEADER_OP_END;
		cmd[i++] = 0;

		buffer->user_size = i * 4;
		prefetch = buffer->user_size / 8;
		etnaviv_gpu_start_fe(gpu, cmd_va, prefetch);

		if (etnaviv_gpu_wait_idle(gpu, 200))
			dev_warn(gpu->dev,
				 "NN flop reset: FLUSH_CACHE timeout idle=0x%08x\n",
				 gpu_read(gpu, VIVS_HI_IDLE_STATE));
		else
			dev_info(gpu->dev, "NN flop reset: FLUSH_CACHE OK\n");
	}

	/*
	 * Second NN run: verify we can run another NN operation after
	 * FLUSH_CACHE. Reset output and re-run the same instruction.
	 */
	{
		u8 *out = data_cpu + 0x300;

		memset(data_cpu + 0x300, 0xBB, 64);
		wmb();

		/* Re-build the NNCmd (same as above) */
		i = 0;
#define LS(reg, val) do { \
	cmd[i++] = VIV_FE_LOAD_STATE_HEADER_OP_LOAD_STATE | \
		   VIV_FE_LOAD_STATE_HEADER_COUNT(1) | \
		   VIV_FE_LOAD_STATE_HEADER_OFFSET( \
			(reg) >> VIV_FE_LOAD_STATE_HEADER_OFFSET__SHR); \
	cmd[i++] = (val); \
} while (0)
		{
			u32 *cmd = buffer->vaddr;

			LS(VIVS_PA_SYSTEM_MODE, 0x11);
			LS(VIVS_GL_API_MODE, VIVS_GL_API_MODE_OPENCL);
			LS(VIVS_GL_SRAM_REMAP_ADDRESS, 0x00400000);
			LS(VIVS_GL_SRAM_REMAP_ADDRESS, 0x00400000);
			LS(VIVS_GL_OCB_REMAP_START, 0x0);
			LS(VIVS_GL_OCB_REMAP_END, 0x0);
			LS(VIVS_GL_NN_CONFIG,
			   etnaviv_npu_nn_config(gpu, 0x10));
			LS(VIVS_GL_SHADER_INDEX, 0x0);
			LS(VIVS_PS_NN_INST_ADDR, (inst_va >> 6) << 6);
			LS(VIVS_PS_UNK10A4, 0x0);
			cmd[i++] = VIV_FE_END_HEADER_OP_END;
			cmd[i++] = 0;

			buffer->user_size = i * 4;
			prefetch = buffer->user_size / 8;
			etnaviv_gpu_start_fe(gpu, cmd_va, prefetch);
		}
#undef LS

		for (i = 0; i < 50000; i++) {
			u32 idle = gpu_read(gpu, VIVS_HI_IDLE_STATE);
			if (idle == 0x7FFFFFFF)
				break;
			udelay(10);
		}

		dev_info(gpu->dev,
			 "NN flop reset 2nd: idle=0x%08x loops=%d out=[%02x %02x %02x %02x]\n",
			 gpu_read(gpu, VIVS_HI_IDLE_STATE), i,
			 out[0], out[1], out[2], out[3]);

		/* FLUSH_CACHE after 2nd run */
		{
			u32 *cmd = buffer->vaddr;
			i = 0;
			cmd[i++] = VIV_FE_LOAD_STATE_HEADER_OP_LOAD_STATE |
				   VIV_FE_LOAD_STATE_HEADER_COUNT(1) |
				   VIV_FE_LOAD_STATE_HEADER_OFFSET(
					VIVS_GL_FLUSH_CACHE >> VIV_FE_LOAD_STATE_HEADER_OFFSET__SHR);
			cmd[i++] = 0x00000C23;
			cmd[i++] = VIV_FE_LOAD_STATE_HEADER_OP_LOAD_STATE |
				   VIV_FE_LOAD_STATE_HEADER_COUNT(1) |
				   VIV_FE_LOAD_STATE_HEADER_OFFSET(
					VIVS_GL_FLUSH_CACHE >> VIV_FE_LOAD_STATE_HEADER_OFFSET__SHR);
			cmd[i++] = 0x00000C23;
			cmd[i++] = VIV_FE_END_HEADER_OP_END;
			cmd[i++] = 0;

			buffer->user_size = i * 4;
			prefetch = buffer->user_size / 8;
			etnaviv_gpu_start_fe(gpu, cmd_va, prefetch);

			if (etnaviv_gpu_wait_idle(gpu, 200))
				dev_warn(gpu->dev,
					 "NN flop reset 2nd: FLUSH_CACHE timeout idle=0x%08x\n",
					 gpu_read(gpu, VIVS_HI_IDLE_STATE));
			else
				dev_info(gpu->dev, "NN flop reset 2nd: FLUSH_CACHE OK\n");
		}
	}

	/*
	 * Keep the NN data page mapped in GPU VA space. The NN/TP engines
	 * retain stale register state referencing the flop reset's VAs.
	 * If we unmap, subsequent mesa TP/NN operations hit MMU faults
	 * at these stale addresses (observed: fault at 0x00480400).
	 * The DMA-coherent page persists for the GPU's lifetime anyway.
	 */

	/*
	 * Stage 2 (A311D batching re-enable): stage the flop-reset op in the
	 * CROSS-CONTEXT flop-reset DDR flat-map (ETNAVIV_FLOP_RESET_VA, mapped
	 * into EVERY MMUv2 context by etnaviv_iommu_install_flop_reset). The
	 * per-context DDR nn_data above is mapped only in this init context, so
	 * a per-submit emission from a client context would MMU-fault on it.
	 * DDR-backed (NOT the AXI-SRAM/OCB flat-map): audit y6 proved the NN
	 * cannot fetch a descriptor from the on-chip OCB region (AXI-SRAM
	 * descriptor never completed, MC stuck busy). gpu->nn_flop_inst_va != 0
	 * then activates the per-submit emission in etnaviv_buffer_v8_qi.c.
	 */
	if (gpu->identity.customer_id == 0x88 && context->global->flop_cpu) {
		void *base = context->global->flop_cpu;
		u32 va = ETNAVIV_FLOP_RESET_VA;
		u32 *fi = base;

		memcpy(fi, nn_flop_ins_a311d, sizeof(nn_flop_ins_a311d));
		fi[5] = ((va + 0x100) >> 6) & 0x03FFFFFF; /* kernel coef */
		fi[6] = va + 0x200;                        /* input */
		fi[7] = va + 0x300;                        /* output */
		memcpy((u8 *)base + 0x100, nn_flop_ker_a311d,
		       sizeof(nn_flop_ker_a311d));
		memset((u8 *)base + 0x200, 0x00, 64);      /* input (garbage OK) */
		memset((u8 *)base + 0x300, 0xAA, 64);      /* output (discarded) */
		wmb();
		gpu->nn_flop_inst_va = va;
		dev_info(gpu->dev,
			 "Stage2: DDR flop-reset staged at VA=0x%08x (dma=0x%llx)\n",
			 va, (unsigned long long)context->global->flop_dma);
	}

	/* nn_flop_node stays in this context's drm_mm: keep the context alive */
	etnaviv_gpu_flop_node_pin(&gpu->nn_flop_ctx, context);
	return;

out_unmap:
	context->global->ops->unmap(context, data_va, PAGE_SIZE);
out_remove:
	mutex_lock(&context->lock);
	drm_mm_remove_node(nn_node);
	mutex_unlock(&context->lock);
}

/*
 * TP flop reset: run a minimal TP transpose to initialize TP hardware state.
 * Required on VIPPico v3 (customerID 0x99) and VIPNano-SI+ (0x88) where
 * TP flip-flops are in undefined state after power-on.
 * Based on galcore's gckHARDWARE_ResetFlopWithTP.
 *
 * Data layout within the nn_data buffer (2 pages):
 *   0x400: TP instructions (8 cores x 128 bytes = 1024 bytes)
 *   0x800: TP input (64 bytes)
 *   0x840: TP output (64 bytes)
 *   0x1000: TP kernel (2048 bytes)
 */

/* TP dimensions: 1x1x2 input, 1x1x2 kernel (64 units), 1x64x1 output */
#define TP_CORE_COUNT		8

/*
 * BSP TP instruction template for VIPPico v3 (customerID 0x99, INT8).
 * 8 cores x 32 dwords each. Words [10], [11], [13] are patched with
 * GPU addresses for input, kernel, and output respectively.
 * Word [12] has the chain bit: 0xC0000002 for cores 0-6, 0x80000002 for core 7.
 * Kernel addresses differ per core (+0x100 each), output addresses differ (+8 each).
 */
static const u32 tp_flop_ins[256] = {
	/* Core 0 */
	0x00000001, 0x00020001, 0x00000001, 0x00000001,
	0x00000000, 0x00000000, 0xA0000E1B, 0x00000000,
	0x00010001, 0x00010001, 0xDFF6F000, 0xDFF62000,
	0xC0000002, 0xDFF6E000, 0x00000000, 0x00000000,
	0x00010001, 0x00000000, 0x00000000, 0x00010001,
	0x00000001, 0x00000000, 0x00010008, 0x00000000,
	0x0000240A, 0x00000000, 0x03FFFFFF, 0x00000000,
	0x03FFFFFF, 0x00000000, 0x00008100, 0x00000000,
	/* Core 1 */
	0x00000001, 0x00020001, 0x00000001, 0x00000001,
	0x00000000, 0x00000000, 0xA0000E1B, 0x00000000,
	0x00010001, 0x00010001, 0xDFF6F000, 0xDFF62100,
	0xC0000002, 0xDFF6E008, 0x00000000, 0x00000000,
	0x00010001, 0x00000000, 0x00000000, 0x00010001,
	0x00000001, 0x00000000, 0x00010008, 0x00000000,
	0x0000240A, 0x00000000, 0x03FFFFFF, 0x00000000,
	0x03FFFFFF, 0x00000000, 0x00008100, 0x00000000,
	/* Core 2 */
	0x00000001, 0x00020001, 0x00000001, 0x00000001,
	0x00000000, 0x00000000, 0xA0000E1B, 0x00000000,
	0x00010001, 0x00010001, 0xDFF6F000, 0xDFF62200,
	0xC0000002, 0xDFF6E010, 0x00000000, 0x00000000,
	0x00010001, 0x00000000, 0x00000000, 0x00010001,
	0x00000001, 0x00000000, 0x00010008, 0x00000000,
	0x0000240A, 0x00000000, 0x03FFFFFF, 0x00000000,
	0x03FFFFFF, 0x00000000, 0x00008100, 0x00000000,
	/* Core 3 */
	0x00000001, 0x00020001, 0x00000001, 0x00000001,
	0x00000000, 0x00000000, 0xA0000E1B, 0x00000000,
	0x00010001, 0x00010001, 0xDFF6F000, 0xDFF62300,
	0xC0000002, 0xDFF6E018, 0x00000000, 0x00000000,
	0x00010001, 0x00000000, 0x00000000, 0x00010001,
	0x00000001, 0x00000000, 0x00010008, 0x00000000,
	0x0000240A, 0x00000000, 0x03FFFFFF, 0x00000000,
	0x03FFFFFF, 0x00000000, 0x00008100, 0x00000000,
	/* Core 4 */
	0x00000001, 0x00020001, 0x00000001, 0x00000001,
	0x00000000, 0x00000000, 0xA0000E1B, 0x00000000,
	0x00010001, 0x00010001, 0xDFF6F000, 0xDFF62400,
	0xC0000002, 0xDFF6E020, 0x00000000, 0x00000000,
	0x00010001, 0x00000000, 0x00000000, 0x00010001,
	0x00000001, 0x00000000, 0x00010008, 0x00000000,
	0x0000240A, 0x00000000, 0x03FFFFFF, 0x00000000,
	0x03FFFFFF, 0x00000000, 0x00008100, 0x00000000,
	/* Core 5 */
	0x00000001, 0x00020001, 0x00000001, 0x00000001,
	0x00000000, 0x00000000, 0xA0000E1B, 0x00000000,
	0x00010001, 0x00010001, 0xDFF6F000, 0xDFF62500,
	0xC0000002, 0xDFF6E028, 0x00000000, 0x00000000,
	0x00010001, 0x00000000, 0x00000000, 0x00010001,
	0x00000001, 0x00000000, 0x00010008, 0x00000000,
	0x0000240A, 0x00000000, 0x03FFFFFF, 0x00000000,
	0x03FFFFFF, 0x00000000, 0x00008100, 0x00000000,
	/* Core 6 */
	0x00000001, 0x00020001, 0x00000001, 0x00000001,
	0x00000000, 0x00000000, 0xA0000E1B, 0x00000000,
	0x00010001, 0x00010001, 0xDFF6F000, 0xDFF62600,
	0xC0000002, 0xDFF6E030, 0x00000000, 0x00000000,
	0x00010001, 0x00000000, 0x00000000, 0x00010001,
	0x00000001, 0x00000000, 0x00010008, 0x00000000,
	0x0000240A, 0x00000000, 0x03FFFFFF, 0x00000000,
	0x03FFFFFF, 0x00000000, 0x00008100, 0x00000000,
	/* Core 7 (last: chain bit = 0) */
	0x00000001, 0x00020001, 0x00000001, 0x00000001,
	0x00000000, 0x00000000, 0xA0000E1B, 0x00000000,
	0x00010001, 0x00010001, 0xDFF6F000, 0xDFF62700,
	0x80000002, 0xDFF6E038, 0x00000000, 0x00000000,
	0x00010001, 0x00000000, 0x00000000, 0x00010001,
	0x00000001, 0x00000000, 0x00010008, 0x00000000,
	0x0000240A, 0x00000000, 0x03FFFFFF, 0x00000000,
	0x03FFFFFF, 0x00000000, 0x00008100, 0x00000000,
};

/*
 * BSP TP kernel data for VIPPico v3 (0x99).
 * 512 dwords = 2048 bytes (TPkerLen=0x800).
 * 8 blocks of 64 dwords each. Each block has a header word (0x21),
 * followed by zeros and two sets of {0xFFFFFFF0, 0xFFFFFFFF, 0x0000000F}.
 */
static const u32 tp_flop_ker[512] = {
	/* Block 0 */
	0x00000021, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0xFFFFFFF0, 0xFFFFFFFF, 0x0000000F, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0xFFFFFFF0, 0xFFFFFFFF, 0x0000000F, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	/* Block 1 */
	0x00000021, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0xFFFFFFF0, 0xFFFFFFFF, 0x0000000F, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0xFFFFFFF0, 0xFFFFFFFF, 0x0000000F, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	/* Block 2 */
	0x00000021, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0xFFFFFFF0, 0xFFFFFFFF, 0x0000000F, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0xFFFFFFF0, 0xFFFFFFFF, 0x0000000F, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	/* Block 3 */
	0x00000021, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0xFFFFFFF0, 0xFFFFFFFF, 0x0000000F, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0xFFFFFFF0, 0xFFFFFFFF, 0x0000000F, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	/* Block 4 */
	0x00000021, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0xFFFFFFF0, 0xFFFFFFFF, 0x0000000F, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0xFFFFFFF0, 0xFFFFFFFF, 0x0000000F, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	/* Block 5 */
	0x00000021, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0xFFFFFFF0, 0xFFFFFFFF, 0x0000000F, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0xFFFFFFF0, 0xFFFFFFFF, 0x0000000F, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	/* Block 6 */
	0x00000021, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0xFFFFFFF0, 0xFFFFFFFF, 0x0000000F, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0xFFFFFFF0, 0xFFFFFFFF, 0x0000000F, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	/* Block 7 */
	0x00000021, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0xFFFFFFF0, 0xFFFFFFFF, 0x0000000F, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0xFFFFFFF0, 0xFFFFFFFF, 0x0000000F, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
};

static void etnaviv_gpu_tp_flop_reset(struct etnaviv_gpu *gpu)
{
	struct etnaviv_cmdbuf *buffer = &gpu->buffer;
	struct etnaviv_iommu_context *context = gpu->mmu_context;
	struct drm_mm_node *tp_node = &gpu->tp_flop_node;
	void *data_cpu = gpu->nn_data_vaddr;
	dma_addr_t data_paddr = gpu->nn_data_paddr;
	u32 data_va, inst_va, ker_va, in_va, out_va;
	u32 *inst;
	u16 prefetch;
	u32 cmd_va;
	u32 ker_anchor, out_anchor;
	int i, k, ret;

	if (gpu->identity.nn_core_count == 0 || !data_cpu)
		return;

	/* Only for customerID 0x99 (S905D3 VIPPico v3).
	 * A311D (0x88, VIPNano-QI) has 4 TP cores vs 8 in S905D3, with
	 * different TP instruction encoding.  The current tp_flop_ins[]
	 * template is S905D3-specific and crashes A311D by accessing
	 * nonexistent TP cores 4-7.  Disable until A311D template is added.
	 */
	if (gpu->identity.customer_id != 0x99)
		return;

	/*
	 * Flop reset is a one-time hardware init. The TP engine retains
	 * register state referencing the reserved VAs, so the mapping and
	 * its drm_mm node live in the gpu struct across the driver's
	 * lifetime. Use drm_mm_node_allocated() to detect previous success.
	 */
	if (drm_mm_node_allocated(tp_node))
		return;

	/*
	 * Map 2 pages into GPU VA space for TP data.
	 * Page 1: TP instructions (0x400), TP input (0x800), TP output (0x840)
	 * Page 2: TP kernel (0x1000)
	 */
	{
		u32 sram_end = 0x00400000 + gpu->identity.on_chip_sram_size;

		sram_end = ALIGN(sram_end, PAGE_SIZE);
		if (sram_end < ETNAVIV_SOFTPIN_START_ADDRESS)
			sram_end = ETNAVIV_SOFTPIN_START_ADDRESS;

		mutex_lock(&context->lock);
		ret = drm_mm_insert_node_in_range(&context->mm, tp_node,
						  ETNAVIV_FLOP_DATA_SIZE, 0, 0,
						  sram_end,
						  U64_MAX, DRM_MM_INSERT_LOW);
	}
	mutex_unlock(&context->lock);
	if (ret) {
		dev_err(gpu->dev, "TP flop reset: VA alloc failed: %d\n", ret);
		return;
	}

	data_va = tp_node->start;
	/* Map each page individually (IOMMU only supports SZ_4K per call) */
	ret = context->global->ops->map(context, data_va, data_paddr,
					SZ_4K,
					ETNAVIV_PROT_READ | ETNAVIV_PROT_WRITE);
	if (ret) {
		dev_err(gpu->dev, "TP flop reset: IOMMU map page 0 failed: %d\n", ret);
		goto out_remove;
	}
	ret = context->global->ops->map(context, data_va + SZ_4K,
					data_paddr + SZ_4K,
					SZ_4K,
					ETNAVIV_PROT_READ | ETNAVIV_PROT_WRITE);
	if (ret) {
		dev_err(gpu->dev, "TP flop reset: IOMMU map page 1 failed: %d\n", ret);
		goto out_unmap_p0;
	}

	inst_va = data_va + 0x400;
	in_va   = data_va + 0x800;
	out_va  = data_va + 0x840;
	ker_va  = data_va + 0x1000;

	/* Copy TP instructions and patch addresses */
	inst = data_cpu + 0x400;
	memcpy(inst, tp_flop_ins, sizeof(tp_flop_ins));

	/*
	 * Patch per-core addresses in TP instructions.
	 * BSP patches: word[10] = inputAddress (same for all cores)
	 *              word[11] = kernelAddress + (orig[11] - anchor)
	 *              word[13] = outputAddress + (orig[13] - anchor)
	 * Anchors are from the BSP template's core 0 values.
	 */
	ker_anchor = tp_flop_ins[11]; /* 0xDFF62000 */
	out_anchor = tp_flop_ins[13]; /* 0xDFF6E000 */
	for (k = 0; k < TP_CORE_COUNT; k++) {
		u32 orig_ker = tp_flop_ins[k * 32 + 11];
		u32 orig_out = tp_flop_ins[k * 32 + 13];

		inst[k * 32 + 10] = in_va;
		inst[k * 32 + 11] = ker_va + (orig_ker - ker_anchor);
		inst[k * 32 + 13] = out_va + (orig_out - out_anchor);
	}

	/* Copy TP kernel at offset 0x1000 */
	memcpy(data_cpu + 0x1000, tp_flop_ker, sizeof(tp_flop_ker));

	/* Fill TP input at offset 0x800: 2 bytes of 0x33 (1x1x2 INT8) */
	memset(data_cpu + 0x800, 0x33, 64);

	/* Pre-fill TP output at offset 0x840 with 0xCC to detect writes */
	memset(data_cpu + 0x840, 0xCC, 64);

	wmb();

	dev_info(gpu->dev,
		 "TP flop reset: inst_va=0x%08x ker_va=0x%08x in_va=0x%08x out_va=0x%08x\n",
		 inst_va, ker_va, in_va, out_va);

	/*
	 * Build the TPCmd in the ring buffer.
	 * For each core: SRAM_REMAP, OCB_REMAP_START/END, TP_CONFIG=2,
	 * UNK03950=8, SHADER_INDEX=0, TP_INST_ADDR with chain bit.
	 * Last core: UNK03950=0, no chain bit.
	 * Then: PS_UNK10A4=0, END (poll idle, then FLUSH_CACHE separately).
	 */
#define LS(reg, val) do {					\
	cmd[i++] = VIV_FE_LOAD_STATE_HEADER_OP_LOAD_STATE |	\
		   VIV_FE_LOAD_STATE_HEADER_COUNT(1) |		\
		   VIV_FE_LOAD_STATE_HEADER_OFFSET(		\
			(reg) >> VIV_FE_LOAD_STATE_HEADER_OFFSET__SHR); \
	cmd[i++] = (val);					\
} while (0)

	{
		u32 *cmd = buffer->vaddr;
		u32 core_inst_va;

		i = 0;

		/*
		 * HI_COMPRESSION_FLAGS and MMU TLB flush, same as NN reset
		 * (0x99: the latched master TLB, synced after the TP pages
		 * were mapped).
		 */
		LS(VIVS_HI_COMPRESSION_FLAGS, 0x40);
		LS(VIVS_MMUv2_CONFIGURATION,
		   VIVS_MMUv2_CONFIGURATION_MODE_MASK |
		   VIVS_MMUv2_CONFIGURATION_FLUSH_FLUSH |
		   etnaviv_iommuv2_sync_hw_mtlb(context));
		cmd[i++] = VIV_FE_WAIT_HEADER_OP_WAIT | 32000;
		cmd[i++] = 0;
		LS(VIVS_PA_SYSTEM_MODE, 0x11);
		LS(VIVS_GL_API_MODE, VIVS_GL_API_MODE_OPENCL);

		for (k = 0; k < TP_CORE_COUNT; k++) {
			LS(VIVS_GL_SRAM_REMAP_ADDRESS, 0x00400000);
			LS(VIVS_GL_OCB_REMAP_START, 0x0);
			LS(VIVS_GL_OCB_REMAP_END, 0x0);
			LS(VIVS_GL_TP_CONFIG, 0x2);

			if (k < TP_CORE_COUNT - 1) {
				LS(VIVS_GL_UNK03950, 0x8);
				LS(VIVS_GL_SHADER_INDEX, 0x0);

				/* TP_INST_ADDR with chain bit for cores 0..6 */
				core_inst_va = ((inst_va + k * 128) & 0xFFFFFFC0) | 0x1;
				LS(VIVS_PS_TP_INST_ADDR, core_inst_va);
			} else {
				/* Last core: UNK03950=0, no chain bit */
				LS(VIVS_GL_UNK03950, 0x0);
				LS(VIVS_GL_SHADER_INDEX, 0x0);

				core_inst_va = (inst_va + k * 128) & 0xFFFFFFC0;
				LS(VIVS_PS_TP_INST_ADDR, core_inst_va);
			}
		}

		LS(VIVS_PS_UNK10A4, 0x0);
		cmd[i++] = VIV_FE_END_HEADER_OP_END;
		cmd[i++] = 0;

		buffer->user_size = i * 4;
	}
#undef LS

	prefetch = buffer->user_size / 8;
	cmd_va = etnaviv_cmdbuf_get_va(buffer,
				       &gpu->mmu_context->cmdbuf_mapping);

	if (etnaviv_gpu_wait_idle(gpu, 200)) {
		dev_warn(gpu->dev,
			 "TP flop reset: FE not idle before kick, idle=0x%08x\n",
			 gpu_read(gpu, VIVS_HI_IDLE_STATE));
	}

	dev_info(gpu->dev,
		 "TP flop reset: cmd_va=0x%08x idle=0x%08x prefetch=%u\n",
		 cmd_va, gpu_read(gpu, VIVS_HI_IDLE_STATE), prefetch);

	etnaviv_gpu_start_fe(gpu, cmd_va, prefetch);

	/* Poll for TP idle */
	for (i = 0; i < 50000; i++) {
		u32 idle = gpu_read(gpu, VIVS_HI_IDLE_STATE);

		if (idle == 0x7FFFFFFF)
			break;
		udelay(10);
	}

	{
		u32 idle = gpu_read(gpu, VIVS_HI_IDLE_STATE);
		u8 *out = data_cpu + 0x840;

		dev_info(gpu->dev,
			 "TP flop reset: idle=0x%08x loops=%d out=[%02x %02x %02x %02x %02x %02x %02x %02x]\n",
			 idle, i, out[0], out[1], out[2], out[3],
			 out[4], out[5], out[6], out[7]);

		if (idle != 0x7FFFFFFF) {
			dev_warn(gpu->dev,
				 "TP flop reset: TP did not complete, mmu_status=0x%08x exc[0..3]=%08x %08x %08x %08x\n",
				 gpu_read(gpu, VIVS_MMUv2_STATUS),
				 gpu_read(gpu, VIVS_MMUv2_EXCEPTION_ADDR(0)),
				 gpu_read(gpu, VIVS_MMUv2_EXCEPTION_ADDR(1)),
				 gpu_read(gpu, VIVS_MMUv2_EXCEPTION_ADDR(2)),
				 gpu_read(gpu, VIVS_MMUv2_EXCEPTION_ADDR(3)));
			goto out_unmap;
		}
	}

	/* FLUSH_CACHE after TP completion */
	{
		u32 *cmd = buffer->vaddr;

		i = 0;
		cmd[i++] = VIV_FE_LOAD_STATE_HEADER_OP_LOAD_STATE |
			   VIV_FE_LOAD_STATE_HEADER_COUNT(1) |
			   VIV_FE_LOAD_STATE_HEADER_OFFSET(
				VIVS_GL_FLUSH_CACHE >> VIV_FE_LOAD_STATE_HEADER_OFFSET__SHR);
		cmd[i++] = 0x00000C23;
		cmd[i++] = VIV_FE_LOAD_STATE_HEADER_OP_LOAD_STATE |
			   VIV_FE_LOAD_STATE_HEADER_COUNT(1) |
			   VIV_FE_LOAD_STATE_HEADER_OFFSET(
				VIVS_GL_FLUSH_CACHE >> VIV_FE_LOAD_STATE_HEADER_OFFSET__SHR);
		cmd[i++] = 0x00000C23;
		cmd[i++] = VIV_FE_END_HEADER_OP_END;
		cmd[i++] = 0;

		buffer->user_size = i * 4;
		prefetch = buffer->user_size / 8;
		etnaviv_gpu_start_fe(gpu, cmd_va, prefetch);

		if (etnaviv_gpu_wait_idle(gpu, 200))
			dev_warn(gpu->dev,
				 "TP flop reset: FLUSH_CACHE timeout idle=0x%08x\n",
				 gpu_read(gpu, VIVS_HI_IDLE_STATE));
		else
			dev_info(gpu->dev, "TP flop reset: FLUSH_CACHE OK\n");
	}

	/*
	 * Keep TP data pages mapped. gpu->tp_flop_node is persistent, so
	 * the drm_mm rbtree has a valid long-lived pointer that will not
	 * be invalidated by stack reuse. The node is detected on the next
	 * call via drm_mm_node_allocated() and the function returns early.
	 * The context that hosts it is pinned until etnaviv_gpu_unbind().
	 */
	etnaviv_gpu_flop_node_pin(&gpu->tp_flop_ctx, context);
	return;

out_unmap:
	context->global->ops->unmap(context, data_va + SZ_4K, SZ_4K);
out_unmap_p0:
	context->global->ops->unmap(context, data_va, SZ_4K);
out_remove:
	mutex_lock(&context->lock);
	drm_mm_remove_node(tp_node);
	mutex_unlock(&context->lock);
}

static void etnaviv_gpu_start_fe_idleloop(struct etnaviv_gpu *gpu,
					  struct etnaviv_iommu_context *context)
{
	u16 prefetch;
	u32 address;

	WARN_ON(gpu->state != ETNA_GPU_STATE_INITIALIZED);

	/* setup the MMU */
	etnaviv_iommu_restore(gpu, context);

	/* Run NN + TP flop reset on NPU devices before starting normal FE loop */
	etnaviv_gpu_nn_flop_reset(gpu);
	etnaviv_gpu_tp_flop_reset(gpu);

	/* Start command processor */
	prefetch = etnaviv_buffer_init(gpu);
	address = etnaviv_cmdbuf_get_va(&gpu->buffer,
					&gpu->mmu_context->cmdbuf_mapping);

	etnaviv_gpu_start_fe(gpu, address, prefetch);

	gpu->state = ETNA_GPU_STATE_RUNNING;
}

static void etnaviv_gpu_setup_pulse_eater(struct etnaviv_gpu *gpu)
{
	/*
	 * Base value for VIVS_PM_PULSE_EATER register on models where it
	 * cannot be read, extracted from vivante kernel driver.
	 */
	u32 pulse_eater = 0x01590880;

	if (etnaviv_is_model_rev(gpu, 0x4000, 0x5208) ||
	    etnaviv_is_model_rev(gpu, 0x4000, 0x5222)) {
		pulse_eater |= BIT(23);

	}

	if (etnaviv_is_model_rev(gpu, 0x1000, 0x5039) ||
	    etnaviv_is_model_rev(gpu, 0x1000, 0x5040)) {
		pulse_eater &= ~BIT(16);
		pulse_eater |= BIT(17);
	}

	if ((gpu->identity.revision > 0x5420) &&
	    (gpu->identity.features & chipFeatures_PIPE_3D))
	{
		/* Performance fix: disable internal DFS */
		pulse_eater = gpu_read_power(gpu, VIVS_PM_PULSE_EATER);
		pulse_eater |= BIT(18);
	}

	gpu_write_power(gpu, VIVS_PM_PULSE_EATER, pulse_eater);
}

static void etnaviv_gpu_hw_init(struct etnaviv_gpu *gpu)
{
	WARN_ON(!(gpu->state == ETNA_GPU_STATE_IDENTIFIED ||
		  gpu->state == ETNA_GPU_STATE_RESET));

	if ((etnaviv_is_model_rev(gpu, 0x320, 0x5007) ||
	     etnaviv_is_model_rev(gpu, 0x320, 0x5220)) &&
	    gpu_read(gpu, VIVS_HI_CHIP_TIME) != 0x2062400) {
		u32 mc_memory_debug;

		mc_memory_debug = gpu_read(gpu, VIVS_MC_DEBUG_MEMORY) & ~0xff;

		if (gpu->identity.revision == 0x5007)
			mc_memory_debug |= 0x0c;
		else
			mc_memory_debug |= 0x08;

		gpu_write(gpu, VIVS_MC_DEBUG_MEMORY, mc_memory_debug);
	}

	/* enable module-level clock gating */
	etnaviv_gpu_enable_mlcg(gpu);

	/*
	 * Update GPU AXI cache atttribute to "cacheable, no allocate".
	 * This is necessary to prevent the iMX6 SoC locking up.
	 *
	 * qi:N1437: NPU-only override, see npu_axi_config above. A non-default
	 * value is honoured only on an NPU (nn_core_count > 0), so the iMX6
	 * GPU path is untouched by construction.
	 */
	if (gpu->identity.nn_core_count > 0 && npu_axi_config >= 0) {
		if (npu_axi_config > 0)
			gpu_write(gpu, VIVS_HI_AXI_CONFIG, (u32)npu_axi_config);
		dev_info(gpu->dev,
			 "npu_axi_config=%d: HI_AXI_CONFIG %s (readback 0x%08x)\n",
			 npu_axi_config,
			 npu_axi_config ? "written" : "left at reset value",
			 gpu_read(gpu, VIVS_HI_AXI_CONFIG));
	} else {
		gpu_write(gpu, VIVS_HI_AXI_CONFIG,
			  VIVS_HI_AXI_CONFIG_AWCACHE(2) |
			  VIVS_HI_AXI_CONFIG_ARCACHE(2));
	}

	/* GC2000 rev 5108 needs a special bus config */
	if (etnaviv_is_model_rev(gpu, 0x2000, 0x5108)) {
		u32 bus_config = gpu_read(gpu, VIVS_MC_BUS_CONFIG);
		bus_config &= ~(VIVS_MC_BUS_CONFIG_FE_BUS_CONFIG__MASK |
				VIVS_MC_BUS_CONFIG_TX_BUS_CONFIG__MASK);
		bus_config |= VIVS_MC_BUS_CONFIG_FE_BUS_CONFIG(1) |
			      VIVS_MC_BUS_CONFIG_TX_BUS_CONFIG(0);
		gpu_write(gpu, VIVS_MC_BUS_CONFIG, bus_config);
	}

	if (gpu->sec_mode == ETNA_SEC_KERNEL) {
		/* Deassert AHB reset (set during etnaviv_hw_reset), then
		 * enable non-secure AHB access. Galcore does:
		 * write 1 (reset), write 0 (deassert), write 2 (nonsec).
		 */
		gpu_write(gpu, VIVS_MMUv2_AHB_CONTROL, 0);
		gpu_write(gpu, VIVS_MMUv2_AHB_CONTROL,
			  VIVS_MMUv2_AHB_CONTROL_NONSEC_ACCESS);
	}

	/* setup the pulse eater */
	etnaviv_gpu_setup_pulse_eater(gpu);

	gpu_write(gpu, VIVS_HI_INTR_ENBL, ~0U);

	gpu->state = ETNA_GPU_STATE_INITIALIZED;
}

static void npu_wedge_hist_add(u32 *hist, u32 streak)
{
	if (!streak)
		return;
	hist[min_t(u32, fls(streak), ETNA_WEDGE_HIST - 1)]++;
}

/*
 * qi:S2 SH-wedge predicate sampler (A311D 0x88 only).
 *
 * Runs in softirq context off an hrtimer. It reads two registers and takes no
 * lock, so it is safe at a 200 us period; the only decision it makes is
 * "ask the drm_sched TDR to look NOW", never "recover".
 *
 * MMIO safety: every allocated event holds a runtime-PM reference (see
 * event_alloc()/event_free()), so a non-empty gpu->event_bitmap means the NPU
 * block is powered. The bitmap is read without gpu->event_spinlock -- a racy
 * read is fine here because the last put is a *deferred* autosuspend (200 ms
 * on 0x88), which is five orders of magnitude wider than the read window.
 * pm_runtime_active() is checked as a second, cheap belt.
 */
/*
 * qi:S2b -- is the wedge signature live RIGHT NOW?
 *
 * The TDR's stock forward-progress test compares against a baseline snapshot.
 * v5 improved that snapshot by handing the TDR the poller's tripping sample,
 * but a snapshot is only as good as its age -- and measurement (the S2b
 * TDR-decline audit) showed fast faults reaching the TDR 2.3-3.6 s
 * after they were raised, because a fault raised while a recovery is running
 * is cancelled by that recovery's drm_sched_stop(). Against a baseline that
 * old the FE has legitimately moved, so the TDR declines a real wedge.
 *
 * The poller already maintains the answer continuously: streak_full is the
 * number of CONSECUTIVE samples, up to the last one 200 us ago, for which
 * NN+TP were idle AND the FE DMA address AND the completed fence were all
 * unchanged. Asking it directly is never stale, so it is the sound form of
 * the same question.
 */
bool etnaviv_npu_wedge_live(struct etnaviv_gpu *gpu)
{
	int k = READ_ONCE(npu_wedge_trip_k);

	return k > 0 && READ_ONCE(npu_wedge_poll_us) > 0 &&
	       READ_ONCE(gpu->npu_wedge.streak_full) >= (u32)k;
}

bool etnaviv_npu_wedge_verbose(void)
{
	return READ_ONCE(npu_wedge_verbose) != 0;
}

/*
 * qi:S2b -- start the poller's observation over. Called after a recovery: the
 * engines have been reset and resubmitted, so any streak accumulated against
 * the pre-reset state says nothing about the new one. Without this the streak
 * carries across the reset and the next trip is not a fresh K-sample
 * observation.
 */
void etnaviv_npu_wedge_reseed(struct etnaviv_gpu *gpu)
{
	struct etnaviv_npu_wedge *w = &gpu->npu_wedge;

	w->streak_idle = 0;
	w->streak_full = 0;
	w->last_dma = ~0u;
	w->last_fence = ~0u;
	w->suppress = false;
	w->fault_pending = false;
}

static enum hrtimer_restart etnaviv_npu_wedge_poll(struct hrtimer *t)
{
	struct etnaviv_gpu *gpu = container_of(t, struct etnaviv_gpu,
					       npu_wedge_timer);
	struct etnaviv_npu_wedge *w = &gpu->npu_wedge;
	int period_us = READ_ONCE(npu_wedge_poll_us);
	int trip_k = READ_ONCE(npu_wedge_trip_k);
	u32 idle, dma, fence;
	bool nn_tp_idle, full;

	/*
	 * Floor the period. This callback re-arms itself from inside the
	 * timer softirq, so a period shorter than the time the callback plus
	 * its two MMIO reads actually take makes the softirq re-enter without
	 * ever draining -- the box then services timers and nothing else. The
	 * observable result is a machine that still answers ping (NAPI runs
	 * ahead of the timer softirq) while every userspace path, sshd
	 * included, starves: i.e. a wedge indistinguishable from the NPU hang
	 * this poller exists to detect. The knob is 0644 and world-settable
	 * from a shell, so one mistyped value must not be able to cost a
	 * power cycle.
	 */
	if (period_us > 0 && period_us < ETNA_WEDGE_MIN_POLL_US)
		period_us = ETNA_WEDGE_MIN_POLL_US;

	hrtimer_forward_now(t, ns_to_ktime((u64)(period_us > 0 ? period_us :
					   ETNA_WEDGE_IDLE_TICK_US) *
					   NSEC_PER_USEC));

	if (w->reset_seen != READ_ONCE(npu_wedge_reset)) {
		w->reset_seen = READ_ONCE(npu_wedge_reset);
		memset(&w->s, 0, sizeof(w->s));
	}

	if (period_us <= 0)
		goto no_sample;
	if (gpu->state != ETNA_GPU_STATE_RUNNING)
		goto no_sample;
	if (bitmap_empty(gpu->event_bitmap, ETNA_NR_EVENTS))
		goto no_sample;
	if (!pm_runtime_active(gpu->dev))
		goto no_sample;

	idle = gpu_read(gpu, VIVS_HI_IDLE_STATE);
	dma = gpu_read(gpu, VIVS_FE_DMA_ADDRESS);
	fence = gpu->completed_fence;

	w->s.samples++;
	nn_tp_idle = (idle & VIVS_HI_IDLE_STATE_NN) &&
		     (idle & VIVS_HI_IDLE_STATE_TP);
	/*
	 * The DMA-unchanged conjunct is what separates a WEDGE from a healthy
	 * inter-op gap: between ops the FE keeps walking to the next
	 * descriptor, so its DMA address advances even while NN and TP are
	 * momentarily idle. last_dma/last_fence are seeded to ~0 on every
	 * gap, which guarantees a streak needs at least two samples to start.
	 */
	full = nn_tp_idle && dma == w->last_dma && fence == w->last_fence;
	w->last_dma = dma;
	w->last_fence = fence;

	if (nn_tp_idle) {
		w->s.samples_idle++;
		w->streak_idle++;
		w->s.max_idle = max(w->s.max_idle, w->streak_idle);
	} else {
		npu_wedge_hist_add(w->s.hist_idle, w->streak_idle);
		w->streak_idle = 0;
	}

	if (full) {
		w->s.samples_full++;
		w->streak_full++;
		w->s.max_full = max(w->s.max_full, w->streak_full);
	} else {
		npu_wedge_hist_add(w->s.hist_full, w->streak_full);
		w->streak_full = 0;
		/*
		 * The signature went away on its own, so the previous fault
		 * (if any) is water under the bridge: re-enable the fast path.
		 */
		w->suppress = false;
	}

	/*
	 * qi:S2b -- never trip while a TDR run is in flight. Two defects come
	 * from tripping into a running handler:
	 *   1. the handler consumes fault_pending on entry, so a re-trip
	 *      mid-handler rewrites gpu->hangcheck_* out from under the very
	 *      test the handler is about to make;
	 *   2. that re-trip's drm_sched_fault() is then cancelled by the
	 *      recovery's drm_sched_stop(), leaving fault_pending latched TRUE
	 *      with no queued work -- which permanently gags the poller (it
	 *      requires !fault_pending to trip) and mislabels the next plain
	 *      2 s timeout as `path=poll`.
	 * Both were observed on 9e62: the fast path fired ONCE at age=19 us and
	 * every subsequent wedge degraded to 2 s-timeout latency.
	 */
	if (trip_k > 0 && !w->suppress && !w->fault_pending && !w->tdr_active &&
	    w->streak_full >= (u32)trip_k) {
		w->fault_pending = true;
		w->s.faults++;
		/*
		 * Hand the TDR OUR forward-progress baseline before faulting.
		 *
		 * etnaviv_sched_timedout_job() decides "the FE is still making
		 * progress" from `dma_addr - gpu->hangcheck_dma_addr`, and
		 * refreshes hangcheck_dma_addr/_fence ONLY on its own runs. A
		 * fast fault arrives K*period into the job -- typically
		 * seconds before the 2 s tick that last wrote those fields --
		 * so the FE has legitimately advanced since that stale
		 * baseline. The TDR therefore reads "forward progress",
		 * declines the fault, and latches `suppress` for the rest of
		 * the wedge, which stands the fast path down permanently and
		 * leaves the wedge to be paid for by the 2 s timeout anyway.
		 * Measured on 9e62 before this fix: 9 faults, 8 declined, all
		 * 8 recoveries via path=timeout, wall unchanged at 8937 ms.
		 *
		 * Stamping our own sample here makes the TDR's test mean "did
		 * the FE move since the poller observed it 200 us ago", which
		 * is the question worth asking -- and keeps the check live, so
		 * a genuinely-advancing FE still declines us.
		 *
		 * Racy against the TDR's own writes to these fields, which is
		 * fine: the loser of the race costs at most one spurious
		 * decline or one extra pass into the wedge branch, and that
		 * branch independently re-reads VIVS_HI_IDLE_STATE and still
		 * requires NN+TP idle before it will recover anything.
		 */
		gpu->hangcheck_dma_addr = dma;
		gpu->hangcheck_fence = fence;
		/* qi:S2b -- preserve the tripping sample for the TDR's report. */
		w->trip_dma = dma;
		w->trip_fence = fence;
		w->trip_streak = w->streak_full;
		w->trip_at = ktime_get();
		drm_sched_fault(&gpu->sched);
	}
	return HRTIMER_RESTART;

no_sample:
	npu_wedge_hist_add(w->s.hist_idle, w->streak_idle);
	npu_wedge_hist_add(w->s.hist_full, w->streak_full);
	w->streak_idle = 0;
	w->streak_full = 0;
	w->last_dma = ~0u;
	w->last_fence = ~0u;
	w->suppress = false;
	return HRTIMER_RESTART;
}

static void etnaviv_gpu_npu_poll_work(struct work_struct *work);
static void etnaviv_gpu_npu_drain_work(struct work_struct *work);

int etnaviv_gpu_init(struct etnaviv_gpu *gpu)
{
	struct etnaviv_drm_private *priv = gpu->drm->dev_private;
	dma_addr_t cmdbuf_paddr;
	int ret, i;

	ret = pm_runtime_get_sync(gpu->dev);
	if (ret < 0) {
		dev_err(gpu->dev, "Failed to enable GPU power domain\n");
		goto pm_put;
	}
	dev_dbg(gpu->dev, "gpu_init: pm_runtime done ret=%d\n", ret);

	ret = etnaviv_gpu_reset_deassert(gpu);
	if (ret) {
		dev_err(gpu->dev, "GPU reset deassert failed\n");
		goto fail;
	}

	etnaviv_hw_identify(gpu);

	if (gpu->identity.model == 0) {
		dev_err(gpu->dev, "Unknown GPU model\n");
		ret = -ENXIO;
		goto fail;
	}

	if (gpu->identity.nn_core_count > 0) {
		dev_warn(gpu->dev, "etnaviv has been instantiated on a NPU, "
                                   "for which the UAPI is still experimental\n");

		/*
		 * Correct the drm_sched timeout now that hw_identify has run.
		 * etnaviv_sched_init() runs at component-bind time, BEFORE
		 * hw_identify, so gpu->identity.nn_core_count is still 0 there
		 * and the ternary in etnaviv_sched_init picks the non-NPU
		 * 500 ms default for every NPU too.
		 *
		 * The correct timeout is CHIP-SPECIFIC and the two NPUs need
		 * OPPOSITE values (empirically falsified the "60 s fixes both"
		 * premise on 2026-05-31):
		 *
		 *  - A311D (0x88, VIPNano-QI): the FE completes; NPU layers
		 *    just take >500 ms (movenet whole-model NN ops at
		 *    1156/1201 ms). The 500 ms watchdog false-fires
		 *    recover_hung on every slow-but-completing op, and the
		 *    accumulated false recoveries push the inference past the
		 *    120 s hung_task_panic threshold. 60 s is correct here.
		 *
		 *  - S905D3 (0x99, VIPPico v3): the FE genuinely PARKS
		 *    mid-cmdbuf and never signals completion. The fast 500 ms
		 *    watchdog + strike-3 timedout_job re-arm is the
		 *    load-bearing forward-progress mechanism. Forcing 60 s
		 *    makes every park cost 60 s instead of 500 ms: mobilenetv2
		 *    went 18 s -> 164 s and 7/9 corpus models TIMED OUT at
		 *    200 s on c64a. 0x99 MUST keep the short default.
		 *
		 * So only bump 0x88; leave 0x99 (and any other customer) on the
		 * 500 ms sched_init default where strike-3 handles the parks.
		 */
		if (gpu->identity.customer_id == 0x88)
			/*
			 * N2 fast-recover (2026-06-19): was 60 s (to avoid the
			 * Layer-2 false-recover-on-slow-op panic). Now 2 s, paired
			 * with the timedout_job 0x88 signature gate (recover only on
			 * the SH-wedge = NN+TP idle + SH busy; re-arm while NN/TP busy
			 * = legit slow op). The gate is what makes a short timeout
			 * safe -- it distinguishes the wedge from a slow-but-running
			 * op, so the SH-wedge recovers in ~2 s (was ~60 s) and legit
			 * ops re-arm. See etnaviv_sched.c timedout_job.
			 */
			gpu->sched.timeout = msecs_to_jiffies(2000);
		else if (gpu->identity.customer_id == 0x99)
			/*
			 * With the proactive npu_poll_work schedule
			 * (buffer_v8_pico) signaling on NN/TP idle, FE "parks"
			 * are completed by the poll, not the watchdog -- so the
			 * 500 ms default now FALSE-recovers slow-but-completing
			 * ops (a 512-conv whose NN/TP finish just after 500 ms,
			 * idle=0x7ffffff6 = NN/TP idle + SH busy) before the poll
			 * fires, wiping state -> constant output. Give the poll
			 * room to win: a 2 s watchdog. A genuine never-completing
			 * hang still recovers at 2 s (rare); normal ops are
			 * signaled by the poll in <200 ms so this costs nothing.
			 */
			gpu->sched.timeout = msecs_to_jiffies(2000);

		/*
		 * S905D3 (VIPPico v3): one in-flight NPU job, as sched_init
		 * intends. Its credit_limit ternary ran before hw_identify and
		 * left 4. buffer_v8_pico tracks one deferred completion
		 * (npu_poll_event); a second job overwrites it, the first job's
		 * fence is never signalled and the 2 s watchdog recovers it.
		 * 6.12 had this as 933f3bd8ed65c; the 6.18 port dropped it.
		 * A311D (0x88) keeps the default. npu_credit_limit below still
		 * overrides for A/B.
		 */
		if (gpu->identity.customer_id == 0x99)
			gpu->sched.credit_limit = 1;

		/*
		 * qi:N1437 (N1435-b): the OTHER half of the same race. See
		 * npu_credit_limit above -- opt-in, default off.
		 */
		if (npu_credit_limit > 0) {
			u32 was = gpu->sched.credit_limit;

			gpu->sched.credit_limit = npu_credit_limit;
			dev_info(gpu->dev,
				 "npu_credit_limit=%d: drm_sched credit_limit %u -> %u\n",
				 npu_credit_limit, was,
				 gpu->sched.credit_limit);
		}
	}

	/* Exclude VG cores with FE2.0 */
	if (gpu->identity.features & chipFeatures_PIPE_VG &&
	    gpu->identity.features & chipFeatures_FE20) {
		dev_info(gpu->dev, "Ignoring GPU with VG and FE2.0\n");
		ret = -ENXIO;
		goto fail;
	}

	/*
	 * On cores with security features supported, we claim control over the
	 * security states.
	 */
	if ((gpu->identity.minor_features7 & chipMinorFeatures7_BIT_SECURITY) &&
	    (gpu->identity.minor_features10 & chipMinorFeatures10_SECURITY_AHB))
		gpu->sec_mode = ETNA_SEC_KERNEL;

	gpu->state = ETNA_GPU_STATE_IDENTIFIED;

	ret = etnaviv_hw_reset(gpu);
	if (ret) {
		dev_err(gpu->dev, "GPU reset failed\n");
		goto fail;
	}

	ret = etnaviv_iommu_global_init(gpu);
	if (ret)
		goto fail;

	/* Create buffer: */
	ret = etnaviv_cmdbuf_init(priv->cmdbuf_suballoc, &gpu->buffer, SZ_4K);
	if (ret) {
		dev_err(gpu->dev, "could not create command buffer\n");
		goto fail;
	}

	/* Allocate NN/TP flop reset data buffer for NPU devices */
	if (gpu->identity.nn_core_count > 0) {
		gpu->nn_data_vaddr = dma_alloc_wc(gpu->drm->dev,
						  ETNAVIV_FLOP_DATA_SIZE,
						  &gpu->nn_data_paddr,
						  GFP_KERNEL);
		if (!gpu->nn_data_vaddr) {
			dev_err(gpu->dev, "could not alloc NN/TP data buffer\n");
			ret = -ENOMEM;
			goto fail;
		}
	}

	/*
	 * Set the GPU linear window to cover the cmdbuf region, as the GPU
	 * won't be able to start execution otherwise. The alignment to 128M is
	 * chosen arbitrarily but helps in debugging, as the MMU offset
	 * calculations are much more straight forward this way.
	 *
	 * On MC1.0 cores the linear window offset is ignored by the TS engine,
	 * leading to inconsistent memory views. Avoid using the offset on those
	 * cores if possible, otherwise disable the TS feature. MMUv2 doesn't
	 * expose this issue, as all TS accesses are MMU translated, so the
	 * linear window offset won't be used.
	 */
	cmdbuf_paddr = ALIGN_DOWN(etnaviv_cmdbuf_get_pa(&gpu->buffer), SZ_128M);

	if (!(gpu->identity.features & chipFeatures_PIPE_3D) ||
	    (gpu->identity.minor_features0 & chipMinorFeatures0_MC20) ||
	    (gpu->identity.minor_features1 & chipMinorFeatures1_MMU_VERSION)) {
		if (cmdbuf_paddr >= SZ_2G)
			priv->mmu_global->memory_base = SZ_2G;
		else
			priv->mmu_global->memory_base = cmdbuf_paddr;
	} else if (cmdbuf_paddr + SZ_128M >= SZ_2G) {
		dev_info(gpu->dev,
			 "Need to move linear window on MC1.0, disabling TS\n");
		gpu->identity.features &= ~chipFeatures_FAST_CLEAR;
		priv->mmu_global->memory_base = SZ_2G;
	}

	/* Setup event management */
	spin_lock_init(&gpu->event_spinlock);
	init_completion(&gpu->event_free);
	bitmap_zero(gpu->event_bitmap, ETNA_NR_EVENTS);
	for (i = 0; i < ARRAY_SIZE(gpu->event); i++)
		complete(&gpu->event_free);

	/*
	 * NPU completion strategy:
	 *
	 * All NPU chips use GL_EVENT FROM_FE for interrupt delivery.
	 *
	 * VIPNano-QI (A311D, customer_id 0x88): SMALL_BATCH stalls FE
	 * at NN_INST_ADDR until NN completes. GL_EVENT fires at the
	 * right time. ISR signals fence immediately (npu_poll=false).
	 *
	 * VIPPico v3 (S905D3, customer_id 0x99): SMALL_BATCH does NOT
	 * stall FE. GL_EVENT fires before NN completes. ISR defers
	 * fence to npu_poll_work which waits for NN/TP idle (npu_poll=true).
	 */
	if (gpu->identity.nn_core_count > 0) {
		if (gpu->identity.customer_id == 0x99) {
			gpu->npu_poll_completion = true;
			dev_dbg(gpu->dev,
				"NPU: GL_EVENT + deferred poll (VIPPico v3)\n");
			/*
			 * VIPPico v3 bulk-sweep board-wedge mitigation.
			 * The 200 ms runtime-PM autosuspend default (probe)
			 * lets the genpd / meson-ee-pwrc NNA power domain
			 * power-cycle between slow whole-model fixtures while
			 * the prior (op-1+-hung) NN's AXI writes are still
			 * draining. The power-off mid-AXI-burst raises an AXI
			 * bus error; ~4 back-to-back accumulate into a
			 * hardlockup that reboots the board (per the c64a
			 * fullsweep reboot-triage record).
			 * Keep the NNA domain powered across an active sweep so
			 * a recover never races a power-off; it still suspends
			 * once genuinely idle (>30 s). 0x99-gated: A311D 0x88
			 * keeps the 200 ms default unchanged.
			 */
			pm_runtime_set_autosuspend_delay(gpu->dev, 30000);
		} else {
			gpu->npu_poll_completion = false;
			dev_dbg(gpu->dev,
				"NPU: GL_EVENT immediate (FE stalls)\n");
		}
		gpu->npu_poll_event = -1;
		INIT_DELAYED_WORK(&gpu->npu_poll_work,
				  etnaviv_gpu_npu_poll_work);

		/* N9 deferred size-scaled NN-AXI drain (A311D 0x88). */
		gpu->npu_drain_event = -1;
		gpu->npu_drain_fence = NULL;
		INIT_DELAYED_WORK(&gpu->npu_drain_work,
				  etnaviv_gpu_npu_drain_work);

		/* NPU phase-latency ring init (instrumentation) */
		spin_lock_init(&gpu->npu_phase_lock);
		gpu->npu_phase_head = 0;
		for (i = 0; i < ETNA_NR_EVENTS; i++)
			gpu->npu_phase_slot[i] = -1;

		/*
		 * qi:S2 SH-wedge poller. 0x88 only -- 0x99 reaches the same
		 * register through its own deferred-poll path and must not be
		 * perturbed. The timer free-runs for the life of the device
		 * and self-gates on gpu->event_bitmap, so there is no arm /
		 * disarm race with the submit and IRQ paths to get wrong.
		 */
		/*
		 * etnaviv_gpu_init() can run more than once (state machine +
		 * recover_hang), so guard on our own flag -- NOT on
		 * hrtimer_active(), which dereferences timer->base and faults
		 * on a never-initialised (kzalloc'ed) hrtimer.
		 */
		if (gpu->identity.customer_id == 0x88 && !gpu->npu_wedge.armed) {
			gpu->npu_wedge.armed = true;
			gpu->npu_wedge.last_dma = ~0u;
			gpu->npu_wedge.last_fence = ~0u;
			gpu->npu_wedge.reset_seen = npu_wedge_reset;
			/*
			 * v6.18: hrtimer_init() + a separate ->function
			 * assignment was replaced by hrtimer_setup(), which
			 * takes the callback and installs it itself.
			 */
			hrtimer_setup(&gpu->npu_wedge_timer,
				      etnaviv_npu_wedge_poll, CLOCK_MONOTONIC,
				      HRTIMER_MODE_REL_SOFT);
			hrtimer_start(&gpu->npu_wedge_timer,
				      ns_to_ktime((u64)ETNA_WEDGE_IDLE_TICK_US *
						  NSEC_PER_USEC),
				      HRTIMER_MODE_REL_SOFT);
			dev_info(gpu->dev,
				 "NPU: S2 wedge poller armed [%s] (poll=%d us%s, trip_k=%d%s)\n",
				 npu_wedge_build, npu_wedge_poll_us,
				 npu_wedge_poll_us > 0 ? "" : " = SAMPLER OFF",
				 npu_wedge_trip_k,
				 npu_wedge_trip_k ? "" : " = OBSERVE ONLY");
		}
	}

	/* Now program the hardware */
	mutex_lock(&gpu->lock);
	etnaviv_gpu_hw_init(gpu);
	mutex_unlock(&gpu->lock);

	pm_runtime_mark_last_busy(gpu->dev);
	pm_runtime_put_autosuspend(gpu->dev);

	return 0;

fail:
	pm_runtime_mark_last_busy(gpu->dev);
pm_put:
	pm_runtime_put_autosuspend(gpu->dev);

	return ret;
}

#ifdef CONFIG_DEBUG_FS
struct dma_debug {
	u32 address[2];
	u32 state[2];
};

static void verify_dma(struct etnaviv_gpu *gpu, struct dma_debug *debug)
{
	u32 i;

	debug->address[0] = gpu_read(gpu, VIVS_FE_DMA_ADDRESS);
	debug->state[0]   = gpu_read(gpu, VIVS_FE_DMA_DEBUG_STATE);

	for (i = 0; i < 500; i++) {
		debug->address[1] = gpu_read(gpu, VIVS_FE_DMA_ADDRESS);
		debug->state[1]   = gpu_read(gpu, VIVS_FE_DMA_DEBUG_STATE);

		if (debug->address[0] != debug->address[1])
			break;

		if (debug->state[0] != debug->state[1])
			break;
	}
}

int etnaviv_gpu_debugfs(struct etnaviv_gpu *gpu, struct seq_file *m)
{
	struct dma_debug debug;
	u32 dma_lo, dma_hi, axi, idle;
	int ret;

	seq_printf(m, "%s Status:\n", dev_name(gpu->dev));

	ret = pm_runtime_get_sync(gpu->dev);
	if (ret < 0)
		goto pm_put;

	dma_lo = gpu_read(gpu, VIVS_FE_DMA_LOW);
	dma_hi = gpu_read(gpu, VIVS_FE_DMA_HIGH);
	axi = gpu_read(gpu, VIVS_HI_AXI_STATUS);
	idle = gpu_read(gpu, VIVS_HI_IDLE_STATE);

	verify_dma(gpu, &debug);

	seq_puts(m, "\tidentity\n");
	seq_printf(m, "\t model: 0x%x\n", gpu->identity.model);
	seq_printf(m, "\t revision: 0x%x\n", gpu->identity.revision);
	seq_printf(m, "\t product_id: 0x%x\n", gpu->identity.product_id);
	seq_printf(m, "\t customer_id: 0x%x\n", gpu->identity.customer_id);
	seq_printf(m, "\t eco_id: 0x%x\n", gpu->identity.eco_id);

	seq_puts(m, "\tfeatures\n");
	seq_printf(m, "\t major_features: 0x%08x\n",
		   gpu->identity.features);
	seq_printf(m, "\t minor_features0: 0x%08x\n",
		   gpu->identity.minor_features0);
	seq_printf(m, "\t minor_features1: 0x%08x\n",
		   gpu->identity.minor_features1);
	seq_printf(m, "\t minor_features2: 0x%08x\n",
		   gpu->identity.minor_features2);
	seq_printf(m, "\t minor_features3: 0x%08x\n",
		   gpu->identity.minor_features3);
	seq_printf(m, "\t minor_features4: 0x%08x\n",
		   gpu->identity.minor_features4);
	seq_printf(m, "\t minor_features5: 0x%08x\n",
		   gpu->identity.minor_features5);
	seq_printf(m, "\t minor_features6: 0x%08x\n",
		   gpu->identity.minor_features6);
	seq_printf(m, "\t minor_features7: 0x%08x\n",
		   gpu->identity.minor_features7);
	seq_printf(m, "\t minor_features8: 0x%08x\n",
		   gpu->identity.minor_features8);
	seq_printf(m, "\t minor_features9: 0x%08x\n",
		   gpu->identity.minor_features9);
	seq_printf(m, "\t minor_features10: 0x%08x\n",
		   gpu->identity.minor_features10);
	seq_printf(m, "\t minor_features11: 0x%08x\n",
		   gpu->identity.minor_features11);

	seq_puts(m, "\tspecs\n");
	seq_printf(m, "\t stream_count:  %d\n",
			gpu->identity.stream_count);
	seq_printf(m, "\t register_max: %d\n",
			gpu->identity.register_max);
	seq_printf(m, "\t thread_count: %d\n",
			gpu->identity.thread_count);
	seq_printf(m, "\t vertex_cache_size: %d\n",
			gpu->identity.vertex_cache_size);
	seq_printf(m, "\t shader_core_count: %d\n",
			gpu->identity.shader_core_count);
	seq_printf(m, "\t nn_core_count: %d\n",
			gpu->identity.nn_core_count);
	seq_printf(m, "\t pixel_pipes: %d\n",
			gpu->identity.pixel_pipes);
	seq_printf(m, "\t vertex_output_buffer_size: %d\n",
			gpu->identity.vertex_output_buffer_size);
	seq_printf(m, "\t buffer_size: %d\n",
			gpu->identity.buffer_size);
	seq_printf(m, "\t instruction_count: %d\n",
			gpu->identity.instruction_count);
	seq_printf(m, "\t num_constants: %d\n",
			gpu->identity.num_constants);
	seq_printf(m, "\t varyings_count: %d\n",
			gpu->identity.varyings_count);

	seq_printf(m, "\taxi: 0x%08x\n", axi);
	seq_printf(m, "\tidle: 0x%08x\n", idle);
	idle |= ~gpu->idle_mask & ~VIVS_HI_IDLE_STATE_AXI_LP;
	if ((idle & VIVS_HI_IDLE_STATE_FE) == 0)
		seq_puts(m, "\t FE is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_DE) == 0)
		seq_puts(m, "\t DE is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_PE) == 0)
		seq_puts(m, "\t PE is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_SH) == 0)
		seq_puts(m, "\t SH is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_PA) == 0)
		seq_puts(m, "\t PA is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_SE) == 0)
		seq_puts(m, "\t SE is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_RA) == 0)
		seq_puts(m, "\t RA is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_TX) == 0)
		seq_puts(m, "\t TX is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_VG) == 0)
		seq_puts(m, "\t VG is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_IM) == 0)
		seq_puts(m, "\t IM is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_FP) == 0)
		seq_puts(m, "\t FP is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_TS) == 0)
		seq_puts(m, "\t TS is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_BL) == 0)
		seq_puts(m, "\t BL is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_ASYNCFE) == 0)
		seq_puts(m, "\t ASYNCFE is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_MC) == 0)
		seq_puts(m, "\t MC is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_PPA) == 0)
		seq_puts(m, "\t PPA is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_WD) == 0)
		seq_puts(m, "\t WD is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_NN) == 0)
		seq_puts(m, "\t NN is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_TP) == 0)
		seq_puts(m, "\t TP is not idle\n");
	if (idle & VIVS_HI_IDLE_STATE_AXI_LP)
		seq_puts(m, "\t AXI low power mode\n");

	if (gpu->identity.features & chipFeatures_DEBUG_MODE) {
		u32 read0 = gpu_read(gpu, VIVS_MC_DEBUG_READ0);
		u32 read1 = gpu_read(gpu, VIVS_MC_DEBUG_READ1);
		u32 write = gpu_read(gpu, VIVS_MC_DEBUG_WRITE);

		seq_puts(m, "\tMC\n");
		seq_printf(m, "\t read0: 0x%08x\n", read0);
		seq_printf(m, "\t read1: 0x%08x\n", read1);
		seq_printf(m, "\t write: 0x%08x\n", write);
	}

	seq_puts(m, "\tDMA ");

	if (debug.address[0] == debug.address[1] &&
	    debug.state[0] == debug.state[1]) {
		seq_puts(m, "seems to be stuck\n");
	} else if (debug.address[0] == debug.address[1]) {
		seq_puts(m, "address is constant\n");
	} else {
		seq_puts(m, "is running\n");
	}

	seq_printf(m, "\t address 0: 0x%08x\n", debug.address[0]);
	seq_printf(m, "\t address 1: 0x%08x\n", debug.address[1]);
	seq_printf(m, "\t state 0: 0x%08x\n", debug.state[0]);
	seq_printf(m, "\t state 1: 0x%08x\n", debug.state[1]);
	seq_printf(m, "\t last fetch 64 bit word: 0x%08x 0x%08x\n",
		   dma_lo, dma_hi);

	if (gpu->identity.nn_core_count > 0) {
		unsigned long flags;
		u32 n, idx, head;

		seq_printf(m, "\tNPU phase ring (us; npu_phase_trace=%d)\n",
			   npu_phase_trace);
		seq_puts(m, "\t   seqno  cid path fe_walk nn_exec   drain   total polls\n");
		spin_lock_irqsave(&gpu->npu_phase_lock, flags);
		head = gpu->npu_phase_head;
		for (n = 0; n < ETNA_NPU_PHASE_RING; n++) {
			struct etnaviv_npu_phase *p;
			s64 fe_walk, nn_exec;

			idx = (head + n) % ETNA_NPU_PHASE_RING;
			p = &gpu->npu_phase[idx];
			if (!p->valid || p->t_signal == 0)
				continue;
			/*
			 * t_irq == 0 => completion driven by the proactive
			 * npu_poll timer, not the FROM_PE IRQ: no observed
			 * FE-done boundary, so FE-walk / NN-exec are not
			 * separable (-1 = n/a). drain and total stay valid.
			 */
			fe_walk = p->t_irq ?
				ktime_us_delta(p->t_irq, p->t_submit) : -1;
			nn_exec = p->t_irq ?
				ktime_us_delta(p->t_idle, p->t_irq) : -1;
			seq_printf(m,
				   "\t%8llu 0x%02x %4s %7lld %7lld %7lld %7lld %5u\n",
				   p->seqno, p->customer_id,
				   p->path ? "poll" : "irq",
				   fe_walk, nn_exec,
				   ktime_us_delta(p->t_signal, p->t_idle),
				   ktime_us_delta(p->t_signal, p->t_submit),
				   p->poll_iters);
		}
		spin_unlock_irqrestore(&gpu->npu_phase_lock, flags);
	}

	if (gpu->identity.customer_id == 0x88) {
		struct etnaviv_npu_wedge *w = &gpu->npu_wedge;
		int b, period = npu_wedge_poll_us;

		seq_printf(m,
			   "\tNPU wedge poller (poll=%d us, trip_k=%d%s)\n",
			   period, npu_wedge_trip_k,
			   npu_wedge_trip_k ? "" : " = OBSERVE ONLY");
		seq_printf(m, "\t samples: %llu  nn_tp_idle: %llu  full_sig: %llu\n",
			   w->s.samples, w->s.samples_idle, w->s.samples_full);
		seq_printf(m, "\t max streak: idle=%u (%d us)  full=%u (%d us)\n",
			   w->s.max_idle, w->s.max_idle * period,
			   w->s.max_full, w->s.max_full * period);
		seq_printf(m,
			   "\t faults: %llu  stale: %llu  declined: %llu (progress %llu [fence %llu, dma %llu], busy %llu, signalled %llu)\n",
			   w->s.faults, w->s.faults_stale,
			   w->s.faults_declined,
			   w->s.faults_declined_progress,
			   w->s.faults_declined_fence,
			   w->s.faults_declined_dma,
			   w->s.faults_declined_busy,
			   w->s.faults_declined_signalled);
		seq_puts(m, "\t streak histogram (samples: count) idle / full\n");
		for (b = 1; b < ETNA_WEDGE_HIST; b++) {
			if (!w->s.hist_idle[b] && !w->s.hist_full[b])
				continue;
			seq_printf(m, "\t  [%7u..%7u] (%8d us) %8u %8u\n",
				   1u << (b - 1), (1u << b) - 1,
				   (1u << (b - 1)) * period,
				   w->s.hist_idle[b], w->s.hist_full[b]);
		}
	}

	ret = 0;

	pm_runtime_mark_last_busy(gpu->dev);
pm_put:
	pm_runtime_put_autosuspend(gpu->dev);

	return ret;
}
#endif

/* fence object management */
struct etnaviv_fence {
	struct etnaviv_gpu *gpu;
	struct dma_fence base;
};

static inline struct etnaviv_fence *to_etnaviv_fence(struct dma_fence *fence)
{
	return container_of(fence, struct etnaviv_fence, base);
}

static const char *etnaviv_fence_get_driver_name(struct dma_fence *fence)
{
	return "etnaviv";
}

static const char *etnaviv_fence_get_timeline_name(struct dma_fence *fence)
{
	struct etnaviv_fence *f = to_etnaviv_fence(fence);

	return dev_name(f->gpu->dev);
}

static bool etnaviv_fence_signaled(struct dma_fence *fence)
{
	struct etnaviv_fence *f = to_etnaviv_fence(fence);

	return (s32)(f->gpu->completed_fence - f->base.seqno) >= 0;
}

static void etnaviv_fence_release(struct dma_fence *fence)
{
	struct etnaviv_fence *f = to_etnaviv_fence(fence);

	kfree_rcu(f, base.rcu);
}

static const struct dma_fence_ops etnaviv_fence_ops = {
	.get_driver_name = etnaviv_fence_get_driver_name,
	.get_timeline_name = etnaviv_fence_get_timeline_name,
	.signaled = etnaviv_fence_signaled,
	.release = etnaviv_fence_release,
};

static struct dma_fence *etnaviv_gpu_fence_alloc(struct etnaviv_gpu *gpu)
{
	struct etnaviv_fence *f;

	/*
	 * GPU lock must already be held, otherwise fence completion order might
	 * not match the seqno order assigned here.
	 */
	lockdep_assert_held(&gpu->lock);

	f = kzalloc(sizeof(*f), GFP_KERNEL);
	if (!f)
		return NULL;

	f->gpu = gpu;

	dma_fence_init(&f->base, &etnaviv_fence_ops, &gpu->fence_spinlock,
		       gpu->fence_context, ++gpu->next_fence);

	return &f->base;
}

/* returns true if fence a comes after fence b */
static inline bool fence_after(u32 a, u32 b)
{
	return (s32)(a - b) > 0;
}

/*
 * event management:
 */

static int event_alloc(struct etnaviv_gpu *gpu, unsigned nr_events,
	unsigned int *events)
{
	unsigned long timeout = msecs_to_jiffies(10 * 10000);
	unsigned i, acquired = 0, rpm_count = 0;
	int ret;

	for (i = 0; i < nr_events; i++) {
		unsigned long remaining;

		remaining = wait_for_completion_timeout(&gpu->event_free, timeout);

		if (!remaining) {
			dev_err(gpu->dev, "wait_for_completion_timeout failed");
			ret = -EBUSY;
			goto out;
		}

		acquired++;
		timeout = remaining;
	}

	spin_lock(&gpu->event_spinlock);

	for (i = 0; i < nr_events; i++) {
		int event = find_first_zero_bit(gpu->event_bitmap, ETNA_NR_EVENTS);

		events[i] = event;
		memset(&gpu->event[event], 0, sizeof(struct etnaviv_event));
		set_bit(event, gpu->event_bitmap);
	}

	spin_unlock(&gpu->event_spinlock);

	for (i = 0; i < nr_events; i++) {
		ret = pm_runtime_resume_and_get(gpu->dev);
		if (ret)
			goto out_rpm;
		rpm_count++;
	}

	return 0;

out_rpm:
	for (i = 0; i < rpm_count; i++)
		pm_runtime_put_autosuspend(gpu->dev);
out:
	for (i = 0; i < acquired; i++)
		complete(&gpu->event_free);

	return ret;
}

static void event_free(struct etnaviv_gpu *gpu, unsigned int event)
{
	if (!test_bit(event, gpu->event_bitmap)) {
		dev_warn(gpu->dev, "event %u is already marked as free",
			 event);
	} else {
		clear_bit(event, gpu->event_bitmap);
		complete(&gpu->event_free);
	}

	pm_runtime_put_autosuspend(gpu->dev);
}

/*
 * Cmdstream submission/retirement:
 */
int etnaviv_gpu_wait_fence_interruptible(struct etnaviv_gpu *gpu,
	u32 id, struct drm_etnaviv_timespec *timeout)
{
	struct dma_fence *fence;
	int ret;

	/*
	 * Look up the fence and take a reference. We might still find a fence
	 * whose refcount has already dropped to zero. dma_fence_get_rcu
	 * pretends we didn't find a fence in that case.
	 */
	rcu_read_lock();
	fence = xa_load(&gpu->user_fences, id);
	if (fence)
		fence = dma_fence_get_rcu(fence);
	rcu_read_unlock();

	if (!fence)
		return 0;

	if (!timeout) {
		/* No timeout was requested: just test for completion */
		ret = dma_fence_is_signaled(fence) ? 0 : -EBUSY;
	} else {
		unsigned long remaining = etnaviv_timeout_to_jiffies(timeout);

		ret = dma_fence_wait_timeout(fence, true, remaining);
		if (ret == 0)
			ret = -ETIMEDOUT;
		else if (ret != -ERESTARTSYS)
			ret = 0;

	}

	dma_fence_put(fence);
	return ret;
}

/*
 * Wait for an object to become inactive.  This, on it's own, is not race
 * free: the object is moved by the scheduler off the active list, and
 * then the iova is put.  Moreover, the object could be re-submitted just
 * after we notice that it's become inactive.
 *
 * Although the retirement happens under the gpu lock, we don't want to hold
 * that lock in this function while waiting.
 */
int etnaviv_gpu_wait_obj_inactive(struct etnaviv_gpu *gpu,
	struct etnaviv_gem_object *etnaviv_obj,
	struct drm_etnaviv_timespec *timeout)
{
	unsigned long remaining;
	long ret;

	if (!timeout)
		return !is_active(etnaviv_obj) ? 0 : -EBUSY;

	remaining = etnaviv_timeout_to_jiffies(timeout);

	ret = wait_event_interruptible_timeout(gpu->fence_event,
					       !is_active(etnaviv_obj),
					       remaining);
	if (ret > 0)
		return 0;
	else if (ret == -ERESTARTSYS)
		return -ERESTARTSYS;
	else
		return -ETIMEDOUT;
}

static void sync_point_perfmon_sample(struct etnaviv_gpu *gpu,
	struct etnaviv_event *event, unsigned int flags)
{
	const struct etnaviv_gem_submit *submit = event->submit;
	unsigned int i;

	for (i = 0; i < submit->nr_pmrs; i++) {
		const struct etnaviv_perfmon_request *pmr = submit->pmrs + i;

		if (pmr->flags == flags)
			etnaviv_perfmon_process(gpu, pmr, submit->exec_state);
	}
}

static void sync_point_perfmon_sample_pre(struct etnaviv_gpu *gpu,
	struct etnaviv_event *event)
{
	u32 val;

	mutex_lock(&gpu->lock);

	/* disable clock gating */
	val = gpu_read_power(gpu, VIVS_PM_POWER_CONTROLS);
	val &= ~VIVS_PM_POWER_CONTROLS_ENABLE_MODULE_CLOCK_GATING;
	gpu_write_power(gpu, VIVS_PM_POWER_CONTROLS, val);

	sync_point_perfmon_sample(gpu, event, ETNA_PM_PROCESS_PRE);

	mutex_unlock(&gpu->lock);
}

static void sync_point_perfmon_sample_post(struct etnaviv_gpu *gpu,
	struct etnaviv_event *event)
{
	const struct etnaviv_gem_submit *submit = event->submit;
	unsigned int i;
	u32 val;

	mutex_lock(&gpu->lock);

	sync_point_perfmon_sample(gpu, event, ETNA_PM_PROCESS_POST);

	/* enable clock gating */
	val = gpu_read_power(gpu, VIVS_PM_POWER_CONTROLS);
	val |= VIVS_PM_POWER_CONTROLS_ENABLE_MODULE_CLOCK_GATING;
	gpu_write_power(gpu, VIVS_PM_POWER_CONTROLS, val);

	mutex_unlock(&gpu->lock);

	for (i = 0; i < submit->nr_pmrs; i++) {
		const struct etnaviv_perfmon_request *pmr = submit->pmrs + i;

		*pmr->bo_vma = pmr->sequence;
	}
}


/* add bo's to gpu's ring, and kick gpu: */
/* --- NPU phase-latency ring helpers (all no-ops when npu_phase_trace=0) --- */
static void npu_phase_submit(struct etnaviv_gpu *gpu, int event, u64 seqno)
{
	unsigned long flags;
	u32 slot;

	if (!npu_phase_trace || gpu->identity.nn_core_count == 0 ||
	    event < 0 || event >= ETNA_NR_EVENTS)
		return;

	spin_lock_irqsave(&gpu->npu_phase_lock, flags);
	slot = gpu->npu_phase_head % ETNA_NPU_PHASE_RING;
	gpu->npu_phase_head++;
	gpu->npu_phase[slot] = (struct etnaviv_npu_phase){
		.seqno = seqno,
		.t_submit = ktime_get(),
		.customer_id = gpu->identity.customer_id,
		.valid = 1,
	};
	gpu->npu_phase_slot[event] = slot;
	spin_unlock_irqrestore(&gpu->npu_phase_lock, flags);
}

static void npu_phase_irq(struct etnaviv_gpu *gpu, int event, ktime_t t)
{
	unsigned long flags;
	int slot;

	if (!npu_phase_trace || gpu->identity.nn_core_count == 0 ||
	    event < 0 || event >= ETNA_NR_EVENTS)
		return;

	spin_lock_irqsave(&gpu->npu_phase_lock, flags);
	slot = gpu->npu_phase_slot[event];
	if (slot >= 0 && slot < ETNA_NPU_PHASE_RING &&
	    gpu->npu_phase[slot].valid && gpu->npu_phase[slot].t_irq == 0)
		gpu->npu_phase[slot].t_irq = t;
	spin_unlock_irqrestore(&gpu->npu_phase_lock, flags);
}

static void npu_phase_complete(struct etnaviv_gpu *gpu, int event,
			       ktime_t t_idle, ktime_t t_signal,
			       u32 poll_iters, u8 path)
{
	unsigned long flags;
	int slot;

	if (!npu_phase_trace || gpu->identity.nn_core_count == 0 ||
	    event < 0 || event >= ETNA_NR_EVENTS)
		return;

	spin_lock_irqsave(&gpu->npu_phase_lock, flags);
	slot = gpu->npu_phase_slot[event];
	if (slot >= 0 && slot < ETNA_NPU_PHASE_RING &&
	    gpu->npu_phase[slot].valid) {
		gpu->npu_phase[slot].t_idle = t_idle;
		gpu->npu_phase[slot].t_signal = t_signal;
		gpu->npu_phase[slot].poll_iters = poll_iters;
		gpu->npu_phase[slot].path = path;
	}
	gpu->npu_phase_slot[event] = -1;
	spin_unlock_irqrestore(&gpu->npu_phase_lock, flags);
}

struct dma_fence *etnaviv_gpu_submit(struct etnaviv_gem_submit *submit)
{
	struct etnaviv_gpu *gpu = submit->gpu;
	struct dma_fence *gpu_fence;
	unsigned int i, nr_events = 1, event[3];
	int ret;

	/*
	 * if there are performance monitor requests we need to have
	 * - a sync point to re-configure gpu and process ETNA_PM_PROCESS_PRE
	 *   requests.
	 * - a sync point to re-configure gpu, process ETNA_PM_PROCESS_POST requests
	 *   and update the sequence number for userspace.
	 */
	if (submit->nr_pmrs)
		nr_events = 3;

	ret = event_alloc(gpu, nr_events, event);
	if (ret) {
		DRM_ERROR("no free events\n");
		pm_runtime_put_noidle(gpu->dev);
		return NULL;
	}

	mutex_lock(&gpu->lock);

	gpu_fence = etnaviv_gpu_fence_alloc(gpu);
	if (!gpu_fence) {
		for (i = 0; i < nr_events; i++)
			event_free(gpu, event[i]);

		goto out_unlock;
	}

	if (gpu->state == ETNA_GPU_STATE_INITIALIZED)
		etnaviv_gpu_start_fe_idleloop(gpu, submit->mmu_context);

	if (submit->prev_mmu_context)
		etnaviv_iommu_context_put(submit->prev_mmu_context);
	submit->prev_mmu_context = etnaviv_iommu_context_get(gpu->mmu_context);

	if (submit->nr_pmrs) {
		gpu->event[event[1]].sync_point = &sync_point_perfmon_sample_pre;
		kref_get(&submit->refcount);
		gpu->event[event[1]].submit = submit;
		etnaviv_sync_point_queue(gpu, event[1]);
	}

	gpu->event[event[0]].fence = gpu_fence;
	/*
	 * A311D (0x88) skip-prime-drain lever: propagate ETNA_SUBMIT_NO_DRAIN
	 * to the fence event so the ISR skips the post-NN-AXI udelay drain for
	 * a throwaway-output submit (the mesa NN-priming prefix). Only the
	 * fence-carrying event[0] runs the drain, so this is the only event
	 * that needs the flag.
	 */
	gpu->event[event[0]].no_drain = !!(submit->flags & ETNA_SUBMIT_NO_DRAIN);
	/*
	 * N9 output-size-scaled drain: carry the final-op output byte count
	 * (mesa-supplied) onto the fence event so the ISR can scale the drain.
	 * 0 (non-ML submits / old userspace) -> floored to npu_drain_udelay.
	 */
	gpu->event[event[0]].npu_output_bytes = submit->npu_output_bytes;
	submit->cmdbuf.user_size = submit->cmdbuf.size - 8;
	etnaviv_buffer_queue(gpu, submit->exec_state, submit->mmu_context,
			     event[0], &submit->cmdbuf);
	npu_phase_submit(gpu, event[0], gpu_fence->seqno);

	if (submit->nr_pmrs) {
		gpu->event[event[2]].sync_point = &sync_point_perfmon_sample_post;
		kref_get(&submit->refcount);
		gpu->event[event[2]].submit = submit;
		etnaviv_sync_point_queue(gpu, event[2]);
	}

out_unlock:
	mutex_unlock(&gpu->lock);

	return gpu_fence;
}

/*
 * NPU deferred completion work: called from ISR via schedule_work when
 * GL_EVENT fires before NN/TP complete (VIPPico v3 where SMALL_BATCH
 * doesn't stall FE). Waits for NN/TP idle then signals the fence.
 *
 * Unlike the old CMD_END approach, FE stays alive (GL_EVENT path),
 * so no FE restart needed. Just poll idle and signal.
 */
static void etnaviv_gpu_npu_poll_work(struct work_struct *work)
{
	struct etnaviv_gpu *gpu = container_of(work, struct etnaviv_gpu,
					       npu_poll_work.work);
	int event = gpu->npu_poll_event;
	struct dma_fence *fence;
	ktime_t t_idle;
	u32 idle;
	int i;

	if (event < 0 || event >= ETNA_NR_EVENTS)
		return;

	/*
	 * Poll for NN and TP idle. The GL_EVENT already fired (FE
	 * processed the command buffer) but NN/TP are still executing.
	 * Typical NN ops take 1-500ms depending on layer size.
	 */
	for (i = 0; i < 100000; i++) {
		idle = gpu_read(gpu, VIVS_HI_IDLE_STATE);
		if ((idle & VIVS_HI_IDLE_STATE_NN) &&
		    (idle & VIVS_HI_IDLE_STATE_TP))
			break;
		usleep_range(100, 200);
	}

	t_idle = ktime_get();

	if (i >= 100000)
		dev_err(gpu->dev,
			"NPU poll: timeout idle=0x%08x\n", idle);

	/* Signal the fence -- NN/TP data is in DDR (AXI write-through) */
	fence = gpu->event[event].fence;
	if (fence) {
		gpu->event[event].fence = NULL;
		if (fence_after(fence->seqno, gpu->completed_fence))
			gpu->completed_fence = fence->seqno;
	}

	/* deferred-poll path (0x99): real intermediate t_idle separates
	 * FE-walk (t_irq - t_submit) from NN-exec (t_idle - t_irq). */
	npu_phase_complete(gpu, event, t_idle, ktime_get(), i, 1);

	/*
	 * Release this job's poll slot BEFORE signalling its fence, and only if
	 * it still names this event. Signalling lets drm_sched submit the next
	 * job at once (credit_limit 1), and that submit re-arms npu_poll_event
	 * for its own event; clearing the slot after the signal erased it, the
	 * next job's ISR then took the plain path and its proactive poll found
	 * -1, so a job whose FROM_PE never fired was never signalled and the
	 * 2 s watchdog recovered it (S905D3: FE parked at WAIT after the job's
	 * GL_EVENT, about 1 in 10 fresh-init EfficientDet runs).
	 */
	cmpxchg(&gpu->npu_poll_event, event, -1);
	event_free(gpu, event);

	if (fence)
		dma_fence_signal(fence);
}

/*
 * N9 deferred NN-AXI drain work (A311D 0x88 only). Scheduled from the ISR
 * when the output-size-scaled drain exceeds npu_drain_hardirq_max_us -- too
 * long to udelay() in the hardirq. Sleeps the scaled budget in a sleepable
 * context (usleep_range) then signals the submit fence, so a large-output
 * conv's last DDR write has fully landed before the CPU reads it. This is a
 * SEPARATE delayed_work from npu_poll_work: the 0x99 poll path waits for
 * NN/TP idle (a pollable signal), whereas here NN/TP are ALREADY idle and we
 * must wait a fixed size-scaled time for the AXI write FIFO to drain (no
 * pollable drain-complete register exists -- resolved.md rows 96/97).
 */
static void etnaviv_gpu_npu_drain_work(struct work_struct *work)
{
	struct etnaviv_gpu *gpu = container_of(work, struct etnaviv_gpu,
					       npu_drain_work.work);
	int event = gpu->npu_drain_event;
	struct dma_fence *fence = gpu->npu_drain_fence;
	u64 drain_ns = gpu->npu_drain_ns;
	unsigned long drain_us = div_u64(drain_ns, 1000);

	if (event < 0 || event >= ETNA_NR_EVENTS)
		return;

	/* Sleep the size-scaled drain budget (yields the CPU, unlike the
	 * inline udelay fast path). usleep_range tolerates the tens-of-ms
	 * budgets this path handles; the workqueue wakeup latency is
	 * negligible relative to them. */
	usleep_range(drain_us, drain_us + 1000);

	if (fence) {
		if (fence_after(fence->seqno, gpu->completed_fence))
			gpu->completed_fence = fence->seqno;
		dma_fence_signal(fence);
	}

	/* deferred-drain path (0x88): NN/TP were idle at t_irq; the wait is
	 * the AXI-drain settle, so report it as the drain segment. */
	npu_phase_complete(gpu, event, gpu->npu_drain_t_irq, ktime_get(), 0, 1);

	event_free(gpu, event);
	gpu->npu_drain_event = -1;
	gpu->npu_drain_fence = NULL;
}

static void sync_point_worker(struct work_struct *work)
{
	struct etnaviv_gpu *gpu = container_of(work, struct etnaviv_gpu,
					       sync_point_work);
	struct etnaviv_event *event = &gpu->event[gpu->sync_point_event];
	u32 addr = gpu_read(gpu, VIVS_FE_DMA_ADDRESS);

	event->sync_point(gpu, event);
	etnaviv_submit_put(event->submit);
	event_free(gpu, gpu->sync_point_event);

	/* restart FE last to avoid GPU and IRQ racing against this worker */
	etnaviv_gpu_start_fe(gpu, addr + 2, 2);
}

void etnaviv_gpu_recover_hang(struct etnaviv_gem_submit *submit, u32 dma_addr)
{
	struct etnaviv_gpu *gpu = submit->gpu;
	char *comm = NULL, *cmd = NULL;
	struct task_struct *task;
	unsigned int i;
	unsigned int predump = npu_recover_predump;

	/*
	 * qi:N1453: announce entry BEFORE touching the device. This print
	 * costs no MMIO, so it survives a block whose bus is wedged -- the
	 * defect qi:N1452 measured was that `recover hung GPU!` never reached
	 * the log at all, because the diagnostic reads below faulted first.
	 */
	dev_err(gpu->dev, "recover hung GPU! dma=0x%08x predump=0x%02x\n",
		dma_addr, predump);

	/*
	 * qi:N1469: MMIO-free pre-reset delay.  See npu_recover_predelay_ms.
	 * This runs in the drm_sched timedout_job work item, which is a
	 * sleepable context (the core dump below it takes a mutex and vmaps),
	 * so msleep() is legal here.  The measured elapsed time is printed so a
	 * delay that did not actually happen is visible in the log rather than
	 * assumed.
	 */
	if (npu_recover_predelay_ms) {
		unsigned long t0 = jiffies;
		unsigned int want = npu_recover_predelay_ms;

		msleep(want);
		dev_err(gpu->dev, "  pre-reset predelay want=%u ms elapsed=%u ms\n",
			want, jiffies_to_msecs(jiffies - t0));
	}

	{
		u32 idle = 0, fe_debug = 0, intr_enbl = 0, intr_ack = 0;
		u32 pmc = 0, pms = 0, ppc = 0, dma_rd = 0;
		unsigned int reps = npu_recover_predump_reps;
		unsigned int rep, reads = 0;

		/*
		 * qi:N1453: each of these is opt-in. See npu_recover_predump.
		 * dma_addr is NOT read here by default -- the caller already
		 * has it. qi:N1462: bit0 restores that read, and reps repeats
		 * the whole enabled block to raise the dose.
		 */
		for (rep = 0; predump && rep < reps; rep++) {
			if (predump & BIT(0)) {
				dma_rd = gpu_read(gpu, VIVS_FE_DMA_ADDRESS);
				reads++;
			}
			if (predump & BIT(1)) {
				idle = gpu_read(gpu, VIVS_HI_IDLE_STATE);
				reads++;
			}
			if (predump & BIT(2)) {
				fe_debug = gpu_read(gpu, VIVS_FE_DMA_DEBUG_STATE);
				reads++;
			}
			if (predump & BIT(3)) {
				intr_enbl = gpu_read(gpu, VIVS_HI_INTR_ENBL);
				reads++;
			}
			if (predump & BIT(4)) {
				intr_ack = gpu_read(gpu, VIVS_HI_INTR_ACKNOWLEDGE);
				reads++;
			}
			if (predump & BIT(5)) {
				pmc = gpu_read_power(gpu, VIVS_PM_MODULE_CONTROLS);
				reads++;
			}
			if (predump & BIT(6)) {
				pms = gpu_read_power(gpu, VIVS_PM_MODULE_STATUS);
				reads++;
			}
			if (predump & BIT(7)) {
				ppc = gpu_read_power(gpu, VIVS_PM_POWER_CONTROLS);
				reads++;
			}
		}

		if (predump) {
			dev_err(gpu->dev, "  pre-reset dose=%u reads reps=%u dma_rd=0x%08x\n",
				reads, reps, dma_rd);
			dev_err(gpu->dev, "  pre-reset idle=0x%08x fe_dbg=0x%08x intr_en=0x%08x\n",
				idle, fe_debug, intr_enbl);
			dev_err(gpu->dev, "  pre-reset intr_ack=0x%08x pmc=0x%08x pms=0x%08x ppc=0x%08x\n",
				intr_ack, pmc, pms, ppc);
		}

		/* r91: dump cmdbuf bytes around the hang dma_addr to identify
		 * the stuck instruction. dma_addr is GPU VA; cmdbuf_va is
		 * the active submit's cmdbuf GPU VA. Dump 16 words before +
		 * 16 words after the hang point.
		 *
		 * R44 (2026-05-06): also dump kernel ring buffer when hang is
		 * in ring range -- needed for c64a/0x99 MMU-flush prelude
		 * stall (dma=0x10b0 in ring at va 0x1000). Pre-R44 only
		 * dumped per-op cmdbuf, missing ring-buffer hangs entirely.
		 */
		/*
		 * qi:N1437 (N1418-b): when npu_hangdump_compact is set, the
		 * two per-word loops below are replaced by an 8-words-per-line
		 * form behind a rate limiter, so the dump cannot saturate a
		 * netconsole link at the moment a fault would print.
		 */
		if (npu_hangdump_compact && !__ratelimit(&npu_hangdump_rs)) {
			dev_err(gpu->dev, "hang dump suppressed (rate limit)\n");
			goto hangdump_done;
		}
		if (submit && submit->cmdbuf.vaddr && submit->cmdbuf.size) {
			u32 cmdbuf_va = etnaviv_cmdbuf_get_va(&submit->cmdbuf,
				&gpu->mmu_context->cmdbuf_mapping);
			if (dma_addr >= cmdbuf_va &&
			    dma_addr < cmdbuf_va + submit->cmdbuf.size) {
				u32 dma_off = dma_addr - cmdbuf_va;
				u32 *words = submit->cmdbuf.vaddr;
				int center = dma_off / 4;
				int start = center - 8;
				int end = center + 16;
				if (start < 0) start = 0;
				if (end > (int)(submit->cmdbuf.size / 4))
					end = submit->cmdbuf.size / 4;
				dev_err(gpu->dev, "cmdbuf hang dump (cmdbuf_va=0x%08x dma_off=0x%x word=%d):\n",
					cmdbuf_va, dma_off, center);
				etnaviv_hangdump_words(gpu, "word", words,
						       start, end, center);
			}
		}
		if (gpu->buffer.vaddr && gpu->buffer.size) {
			u32 ring_va = etnaviv_cmdbuf_get_va(&gpu->buffer,
				&gpu->mmu_context->cmdbuf_mapping);
			if (dma_addr >= ring_va &&
			    dma_addr < ring_va + gpu->buffer.size) {
				u32 dma_off = dma_addr - ring_va;
				u32 *words = gpu->buffer.vaddr;
				int center = dma_off / 4;
				int start = center - 16;
				int end = center + 16;
				if (start < 0) start = 0;
				if (end > (int)(gpu->buffer.size / 4))
					end = gpu->buffer.size / 4;
				dev_err(gpu->dev, "ring hang dump (ring_va=0x%08x dma_off=0x%x word=%d):\n",
					ring_va, dma_off, center);
				etnaviv_hangdump_words(gpu, "ring", words,
						       start, end, center);
			}
		}
hangdump_done:
		;
	}

	task = get_pid_task(submit->pid, PIDTYPE_PID);
	if (task) {
		comm = kstrdup(task->comm, GFP_KERNEL);
		cmd = kstrdup_quotable_cmdline(task, GFP_KERNEL);
		put_task_struct(task);
	}

	if (comm && cmd)
		dev_err(gpu->dev, "offending task: %s (%s)\n", comm, cmd);

	kfree(cmd);
	kfree(comm);

	if (pm_runtime_get_sync(gpu->dev) < 0)
		goto pm_put;

	mutex_lock(&gpu->lock);

	etnaviv_hw_reset(gpu);

	/*
	 * qi:N1453: the diagnostic register snapshot, moved to AFTER the
	 * reset. etnaviv_hw_reset() is the access that puts the block back on
	 * the bus, so from here the reads are safe on a wedged 0x88 -- which
	 * is exactly where the pre-reset block took an uncontainable SError
	 * and panicked the board (qi:N1452 D2).
	 */
	dev_err(gpu->dev, "post-reset idle=0x%08x fe_dbg=0x%08x intr_en=0x%08x\n",
		gpu_read(gpu, VIVS_HI_IDLE_STATE),
		gpu_read(gpu, VIVS_FE_DMA_DEBUG_STATE),
		gpu_read(gpu, VIVS_HI_INTR_ENBL));
	dev_err(gpu->dev, "post-reset pmc=0x%08x pms=0x%08x ppc=0x%08x\n",
		gpu_read_power(gpu, VIVS_PM_MODULE_CONTROLS),
		gpu_read_power(gpu, VIVS_PM_MODULE_STATUS),
		gpu_read_power(gpu, VIVS_PM_POWER_CONTROLS));

	/* complete all events, the GPU won't do it after the reset */
	spin_lock(&gpu->event_spinlock);
	for_each_set_bit(i, gpu->event_bitmap, ETNA_NR_EVENTS)
		event_free(gpu, i);
	spin_unlock(&gpu->event_spinlock);

	etnaviv_gpu_hw_init(gpu);

	mutex_unlock(&gpu->lock);
	pm_runtime_mark_last_busy(gpu->dev);
pm_put:
	pm_runtime_put_autosuspend(gpu->dev);
}

static void dump_mmu_fault(struct etnaviv_gpu *gpu)
{
	static const char *fault_reasons[] = {
		"slave not present",
		"page not present",
		"write violation",
		"out of bounds",
		"read security violation",
		"write security violation",
	};

	u32 status_reg, status;
	int i;

	if (gpu->sec_mode == ETNA_SEC_NONE)
		status_reg = VIVS_MMUv2_STATUS;
	else
		status_reg = VIVS_MMUv2_SEC_STATUS;

	status = gpu_read(gpu, status_reg);
	dev_err_ratelimited(gpu->dev, "MMU fault status 0x%08x\n", status);

	for (i = 0; i < 4; i++) {
		const char *reason = "unknown";
		u32 address_reg;
		u32 mmu_status;

		mmu_status = (status >> (i * 4)) & VIVS_MMUv2_STATUS_EXCEPTION0__MASK;
		if (!mmu_status)
			continue;

		if ((mmu_status - 1) < ARRAY_SIZE(fault_reasons))
			reason = fault_reasons[mmu_status - 1];

		if (gpu->sec_mode == ETNA_SEC_NONE)
			address_reg = VIVS_MMUv2_EXCEPTION_ADDR(i);
		else
			address_reg = VIVS_MMUv2_SEC_EXCEPTION_ADDR;

		dev_err_ratelimited(gpu->dev,
				    "MMU %d fault (%s) addr 0x%08x\n",
				    i, reason, gpu_read(gpu, address_reg));
	}
}

static irqreturn_t irq_handler(int irq, void *data)
{
	struct etnaviv_gpu *gpu = data;
	irqreturn_t ret = IRQ_NONE;

	u32 intr = gpu_read(gpu, VIVS_HI_INTR_ACKNOWLEDGE);

	if (gpu->identity.nn_core_count > 0 && etnaviv_dump_cmdbuf)
		dev_info_ratelimited(gpu->dev, "NPU irq: intr=0x%08x\n", intr);

	if (intr != 0) {
		ktime_t now = ktime_get();
		int event;

		pm_runtime_mark_last_busy(gpu->dev);

		dev_dbg(gpu->dev, "intr 0x%08x\n", intr);

		if (intr & VIVS_HI_INTR_ACKNOWLEDGE_AXI_BUS_ERROR) {
			dev_err(gpu->dev, "AXI bus error\n");
			intr &= ~VIVS_HI_INTR_ACKNOWLEDGE_AXI_BUS_ERROR;
		}

		if (intr & VIVS_HI_INTR_ACKNOWLEDGE_MMU_EXCEPTION) {
			dump_mmu_fault(gpu);
			gpu->state = ETNA_GPU_STATE_FAULT;
			drm_sched_fault(&gpu->sched);
			intr &= ~VIVS_HI_INTR_ACKNOWLEDGE_MMU_EXCEPTION;
		}

		while ((event = ffs(intr)) != 0) {
			struct dma_fence *fence;

			event -= 1;

			intr &= ~(1 << event);

			dev_dbg(gpu->dev, "event %u\n", event);

			npu_phase_irq(gpu, event, now);

			if (gpu->event[event].sync_point) {
				gpu->sync_point_event = event;
				queue_work(gpu->wq, &gpu->sync_point_work);
			}

			/*
			 * VIPPico v3 deferred completion: GL_EVENT fires
			 * from FE before NN/TP finish (FE doesn't stall).
			 * Defer fence signaling to npu_poll_work which
			 * waits for NN/TP idle then signals.
			 */
			if (gpu->npu_poll_completion &&
			    event == gpu->npu_poll_event) {
				/* Run the deferred poll now (FROM_PE fired).
				 * mod_delayed_work(.,0) coalesces with the
				 * proactive 100 ms schedule in buffer_v8_pico
				 * so whichever fires first runs npu_poll_work
				 * exactly once. */
				mod_delayed_work(system_wq,
						 &gpu->npu_poll_work, 0);
				continue;
			}

			fence = gpu->event[event].fence;
			if (!fence)
				continue;

			gpu->event[event].fence = NULL;

			/* npu_tail_idle_trace (default 0): one idle-state sample
			 * at IRQ entry, before any drain/signal. */
			if (npu_tail_idle_trace &&
			    gpu->identity.customer_id == 0x88) {
				u32 idle = gpu_read(gpu, VIVS_HI_IDLE_STATE);

				dev_info_ratelimited(gpu->dev,
					"npu tail idle: event %u idle=0x%08x nn_idle=%d tp_idle=%d\n",
					event, idle,
					!!(idle & VIVS_HI_IDLE_STATE_NN),
					!!(idle & VIVS_HI_IDLE_STATE_TP));
			}

			/*
			 * A311D (VIPNano-QI v8, customer_id 0x88) hardware
			 * drain wait. Without this ~1 ms delay before
			 * signaling the submit fence, mesa's
			 * pipe_buffer_read of the output BO returns stale
			 * data from the prior inference (MobileNet v1
			 * reused-interpreter off-by-one bug). The
			 * mechanism is a hardware drain (AXI write
			 * buffer or NN engine output FIFO) that completes
			 * 400-600 us after NN/TP go idle -- the NN/TP idle
			 * bits in VIVS_HI_IDLE_STATE are already set at
			 * fence-signal time, so polling them is useless.
			 *
			 * Matrix-driven sweep on 2026-04-13 showed:
			 *   < 400 us: deterministic 0/7 broken
			 *   400-500 us: stochastic, partial passes
			 *   >= 600 us: deterministic 7/7 fixed
			 * udelay(1000) is a conservative value above the
			 * stochastic boundary. Validated on A311D with
			 * 21/21 inferences correct across 3 runs at
			 * 266 ms/inference (vs 265 ms broken baseline,
			 * ~0.3 percent overhead).
			 *
			 * 1 ms hardirq block is within safe limits at the
			 * few-per-inference rate the NPU produces. An
			 * alternative defer-to-workqueue approach
			 * (schedule_work + npu_poll_work, as customer 0x99
			 * already uses) also fixes correctness but is ~2x
			 * slower due to workqueue wakeup latency. udelay
			 * is simpler and faster.
			 *
			 * Investigation: the A311D MobileNet v1
			 * Off-By-One Bug "BREAKTHROUGH" analysis
			 * and subsequent matrix v1/v2/v3 sections.
			 *
			 * S905D3 (customer 0x99) does NOT need this --
			 * that path uses npu_poll_completion deferral
			 * which polls VIVS_HI_IDLE_STATE_NN/_TP before
			 * signaling, providing the same effective wait.
			 *
			 * skip-prime-drain lever (ETNA_SUBMIT_NO_DRAIN): a
			 * throwaway-output submit (the mesa NN-priming prefix)
			 * is never CPU-read-back, so it needs no settle. mesa
			 * tags it with ETNA_SUBMIT_NO_DRAIN; skip the udelay for
			 * that event. The 2 real-output drains stay -- they
			 * gate the CPU readback handoff. Saves ~3 ms warm/invoke.
			 */
			if (gpu->identity.customer_id == 0x88 &&
			    !gpu->event[event].no_drain) {
				/*
				 * N9: output-size-scaled drain. drain_ns scales
				 * with the final-op output byte volume (mesa-
				 * supplied), floored at the legacy udelay for
				 * small outputs. Small drains stay inline in the
				 * hardirq (mn001 fast path, unchanged); a large
				 * scaled drain (>hardirq_max) is deferred to a
				 * sleepable delayed work so we do not block the
				 * hardirq 15-48 ms. See npu_drain_ns_per_byte.
				 */
				u32 out_bytes = gpu->event[event].npu_output_bytes;
				u64 drain_ns = (u64)out_bytes * npu_drain_ns_per_byte +
					       (u64)npu_drain_margin_us * 1000;
				u64 floor_ns = (u64)npu_drain_udelay * 1000;

				if (drain_ns < floor_ns)
					drain_ns = floor_ns;

				if (drain_ns <= (u64)npu_drain_hardirq_max_us * 1000) {
					/* FAST PATH: inline hardirq udelay. */
					udelay(div_u64(drain_ns, 1000));
				} else {
					/*
					 * DEFER: hand fence + drain budget to a
					 * sleepable delayed work (SEPARATE from
					 * npu_poll_work / the 0x99 path). The
					 * worker usleep_range()s drain_ns then
					 * signals + frees. fence is already
					 * detached from the event above, so skip
					 * the ISR signal/free for this event.
					 */
					gpu->npu_drain_event = event;
					gpu->npu_drain_ns = drain_ns;
					gpu->npu_drain_fence = fence;
					gpu->npu_drain_t_irq = now;
					mod_delayed_work(system_wq,
							 &gpu->npu_drain_work, 0);
					continue;
				}
			}

			/*
			 * Events can be processed out of order.  Eg,
			 * - allocate and queue event 0
			 * - allocate event 1
			 * - event 0 completes, we process it
			 * - allocate and queue event 0
			 * - event 1 and event 0 complete
			 * we can end up processing event 0 first, then 1.
			 */
			if (fence_after(fence->seqno, gpu->completed_fence))
				gpu->completed_fence = fence->seqno;
			dma_fence_signal_timestamp(fence, now);

			/* IRQ-direct path (0x88): NN/TP already idle at the
			 * IRQ, so t_idle == now (FE-walk+NN-exec merge into
			 * t_irq - t_submit); t_signal is post-udelay-drain. */
			npu_phase_complete(gpu, event, now, ktime_get(), 0, 0);

			event_free(gpu, event);
		}

		ret = IRQ_HANDLED;
	}

	return ret;
}

static int etnaviv_gpu_clk_enable(struct etnaviv_gpu *gpu)
{
	int ret;

	dev_dbg(gpu->dev, "clk_enable: reg\n");
	ret = clk_prepare_enable(gpu->clk_reg);
	if (ret)
		return ret;

	dev_dbg(gpu->dev, "clk_enable: bus\n");
	ret = clk_prepare_enable(gpu->clk_bus);
	if (ret)
		goto disable_clk_reg;

	dev_dbg(gpu->dev, "clk_enable: core\n");
	ret = clk_prepare_enable(gpu->clk_core);
	if (ret)
		goto disable_clk_bus;

	dev_dbg(gpu->dev, "clk_enable: shader\n");
	ret = clk_prepare_enable(gpu->clk_shader);
	if (ret)
		goto disable_clk_core;

	return 0;

disable_clk_core:
	clk_disable_unprepare(gpu->clk_core);
disable_clk_bus:
	clk_disable_unprepare(gpu->clk_bus);
disable_clk_reg:
	clk_disable_unprepare(gpu->clk_reg);

	return ret;
}

static int etnaviv_gpu_clk_disable(struct etnaviv_gpu *gpu)
{
	clk_disable_unprepare(gpu->clk_shader);
	clk_disable_unprepare(gpu->clk_core);
	clk_disable_unprepare(gpu->clk_bus);
	clk_disable_unprepare(gpu->clk_reg);

	return 0;
}

int etnaviv_gpu_wait_idle(struct etnaviv_gpu *gpu, unsigned int timeout_ms)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(timeout_ms);

	do {
		u32 idle = gpu_read(gpu, VIVS_HI_IDLE_STATE);

		if ((idle & gpu->idle_mask) == gpu->idle_mask)
			return 0;

		if (time_is_before_jiffies(timeout)) {
			dev_warn(gpu->dev,
				 "timed out waiting for idle: idle=0x%x\n",
				 idle);
			return -ETIMEDOUT;
		}

		udelay(5);
	} while (1);
}

static void etnaviv_gpu_hw_suspend(struct etnaviv_gpu *gpu)
{
	if (gpu->state == ETNA_GPU_STATE_RUNNING) {
		/* Replace the last WAIT with END */
		mutex_lock(&gpu->lock);
		etnaviv_buffer_end(gpu);
		mutex_unlock(&gpu->lock);

		/*
		 * We know that only the FE is busy here, this should
		 * happen quickly (as the WAIT is only 200 cycles).  If
		 * we fail, just warn and continue.
		 */
		etnaviv_gpu_wait_idle(gpu, 100);

		gpu->state = ETNA_GPU_STATE_INITIALIZED;
	}

	gpu->exec_state = -1;
}

static int etnaviv_gpu_hw_resume(struct etnaviv_gpu *gpu)
{
	int ret;

	ret = mutex_lock_killable(&gpu->lock);
	if (ret)
		return ret;

	etnaviv_gpu_update_clock(gpu);
	etnaviv_gpu_hw_init(gpu);

	mutex_unlock(&gpu->lock);

	return 0;
}

static int
etnaviv_gpu_cooling_get_max_state(struct thermal_cooling_device *cdev,
				  unsigned long *state)
{
	*state = 6;

	return 0;
}

static int
etnaviv_gpu_cooling_get_cur_state(struct thermal_cooling_device *cdev,
				  unsigned long *state)
{
	struct etnaviv_gpu *gpu = cdev->devdata;

	*state = gpu->freq_scale;

	return 0;
}

static int
etnaviv_gpu_cooling_set_cur_state(struct thermal_cooling_device *cdev,
				  unsigned long state)
{
	struct etnaviv_gpu *gpu = cdev->devdata;

	mutex_lock(&gpu->lock);
	gpu->freq_scale = state;
	if (!pm_runtime_suspended(gpu->dev))
		etnaviv_gpu_update_clock(gpu);
	mutex_unlock(&gpu->lock);

	return 0;
}

static const struct thermal_cooling_device_ops cooling_ops = {
	.get_max_state = etnaviv_gpu_cooling_get_max_state,
	.get_cur_state = etnaviv_gpu_cooling_get_cur_state,
	.set_cur_state = etnaviv_gpu_cooling_set_cur_state,
};

static int etnaviv_gpu_bind(struct device *dev, struct device *master,
	void *data)
{
	struct drm_device *drm = data;
	struct etnaviv_drm_private *priv = drm->dev_private;
	struct etnaviv_gpu *gpu = dev_get_drvdata(dev);
	int ret;

	if (IS_ENABLED(CONFIG_DRM_ETNAVIV_THERMAL)) {
		gpu->cooling = thermal_of_cooling_device_register(dev->of_node,
				(char *)dev_name(dev), gpu, &cooling_ops);
		if (IS_ERR(gpu->cooling))
			return PTR_ERR(gpu->cooling);
	}

	gpu->wq = alloc_ordered_workqueue(dev_name(dev), 0);
	if (!gpu->wq) {
		ret = -ENOMEM;
		goto out_thermal;
	}

	ret = etnaviv_sched_init(gpu);
	if (ret)
		goto out_workqueue;

	if (!IS_ENABLED(CONFIG_PM)) {
		ret = etnaviv_gpu_clk_enable(gpu);
		if (ret < 0)
			goto out_sched;
	}

	gpu->drm = drm;
	gpu->fence_context = dma_fence_context_alloc(1);
	xa_init_flags(&gpu->user_fences, XA_FLAGS_ALLOC);
	spin_lock_init(&gpu->fence_spinlock);

	INIT_WORK(&gpu->sync_point_work, sync_point_worker);
	init_waitqueue_head(&gpu->fence_event);

	priv->gpu[priv->num_gpus++] = gpu;

	return 0;

out_sched:
	etnaviv_sched_fini(gpu);

out_workqueue:
	destroy_workqueue(gpu->wq);

out_thermal:
	if (IS_ENABLED(CONFIG_DRM_ETNAVIV_THERMAL))
		thermal_cooling_device_unregister(gpu->cooling);

	return ret;
}

static void etnaviv_gpu_unbind(struct device *dev, struct device *master,
	void *data)
{
	struct etnaviv_gpu *gpu = dev_get_drvdata(dev);

	DBG("%s", dev_name(gpu->dev));

	if (gpu->npu_wedge.armed) {
		hrtimer_cancel(&gpu->npu_wedge_timer);
		/*
		 * Clear the flag so an unbind/rebind cycle that reuses this
		 * etnaviv_gpu re-arms instead of silently leaving a cancelled
		 * timer behind (armed == true, timer dead = poller gone with
		 * no diagnostic).
		 */
		gpu->npu_wedge.armed = false;
	}

	destroy_workqueue(gpu->wq);

	etnaviv_sched_fini(gpu);

	if (IS_ENABLED(CONFIG_PM)) {
		pm_runtime_get_sync(gpu->dev);
		pm_runtime_put_sync_suspend(gpu->dev);
	} else {
		etnaviv_gpu_hw_suspend(gpu);
		etnaviv_gpu_clk_disable(gpu);
	}

	/*
	 * The GPU's lifetime ends here: give the flop-reset nodes back to the
	 * contexts that host them BEFORE the last reference on those contexts
	 * can go, so their drm_mm_takedown() sees a clean allocator.
	 */
	etnaviv_gpu_flop_node_release(gpu, &gpu->nn_flop_ctx, &gpu->nn_flop_node);
	etnaviv_gpu_flop_node_release(gpu, &gpu->tp_flop_ctx, &gpu->tp_flop_node);

	if (gpu->mmu_context)
		etnaviv_iommu_context_put(gpu->mmu_context);

	etnaviv_cmdbuf_free(&gpu->buffer);
	if (gpu->nn_data_vaddr)
		dma_free_wc(gpu->drm->dev, ETNAVIV_FLOP_DATA_SIZE,
			    gpu->nn_data_vaddr, gpu->nn_data_paddr);
	etnaviv_iommu_global_fini(gpu);

	gpu->drm = NULL;
	xa_destroy(&gpu->user_fences);

	if (IS_ENABLED(CONFIG_DRM_ETNAVIV_THERMAL))
		thermal_cooling_device_unregister(gpu->cooling);
	gpu->cooling = NULL;
}

static const struct component_ops gpu_ops = {
	.bind = etnaviv_gpu_bind,
	.unbind = etnaviv_gpu_unbind,
};

static const struct of_device_id etnaviv_gpu_match[] = {
	{
		.compatible = "vivante,gc"
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, etnaviv_gpu_match);

static int etnaviv_gpu_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct etnaviv_gpu *gpu;
	int err;

	gpu = devm_kzalloc(dev, sizeof(*gpu), GFP_KERNEL);
	if (!gpu)
		return -ENOMEM;

	gpu->dev = dev;
	mutex_init(&gpu->lock);
	mutex_init(&gpu->sched_lock);

	/* Map registers: */
	gpu->mmio = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(gpu->mmio))
		return PTR_ERR(gpu->mmio);


	/* Get Reset: */
	gpu->rst = devm_reset_control_get_optional_exclusive(&pdev->dev, NULL);
	if (IS_ERR(gpu->rst))
		return dev_err_probe(dev, PTR_ERR(gpu->rst),
				     "failed to get reset\n");

	err = reset_control_assert(gpu->rst);
	if (err)
		return dev_err_probe(dev, err, "failed to assert reset\n");

	/* Get Interrupt: */
	gpu->irq = platform_get_irq(pdev, 0);
	if (gpu->irq < 0)
		return gpu->irq;

	err = devm_request_irq(dev, gpu->irq, irq_handler, 0,
			       dev_name(dev), gpu);
	if (err) {
		dev_err(dev, "failed to request IRQ%u: %d\n", gpu->irq, err);
		return err;
	}

	/* Get Clocks: */
	gpu->clk_reg = devm_clk_get_optional(&pdev->dev, "reg");
	DBG("clk_reg: %p", gpu->clk_reg);
	if (IS_ERR(gpu->clk_reg))
		return PTR_ERR(gpu->clk_reg);

	gpu->clk_bus = devm_clk_get_optional(&pdev->dev, "bus");
	DBG("clk_bus: %p", gpu->clk_bus);
	if (IS_ERR(gpu->clk_bus))
		return PTR_ERR(gpu->clk_bus);

	gpu->clk_core = devm_clk_get(&pdev->dev, "core");
	DBG("clk_core: %p", gpu->clk_core);
	if (IS_ERR(gpu->clk_core))
		return PTR_ERR(gpu->clk_core);
	gpu->base_rate_core = clk_get_rate(gpu->clk_core);

	gpu->clk_shader = devm_clk_get_optional(&pdev->dev, "shader");
	DBG("clk_shader: %p", gpu->clk_shader);
	if (IS_ERR(gpu->clk_shader))
		return PTR_ERR(gpu->clk_shader);
	gpu->base_rate_shader = clk_get_rate(gpu->clk_shader);

	/* TODO: figure out max mapped size */
	dev_set_drvdata(dev, gpu);

	/*
	 * We treat the device as initially suspended.  The runtime PM
	 * autosuspend delay is rather arbitary: no measurements have
	 * yet been performed to determine an appropriate value.
	 */
	pm_runtime_use_autosuspend(dev);
	pm_runtime_set_autosuspend_delay(dev, 200);
	pm_runtime_enable(dev);

	err = component_add(dev, &gpu_ops);
	if (err < 0) {
		dev_err(dev, "failed to register component: %d\n", err);
		return err;
	}

	return 0;
}

static void etnaviv_gpu_platform_remove(struct platform_device *pdev)
{
	struct etnaviv_gpu *gpu = dev_get_drvdata(&pdev->dev);

	component_del(&pdev->dev, &gpu_ops);
	pm_runtime_disable(&pdev->dev);

	mutex_destroy(&gpu->lock);
	mutex_destroy(&gpu->sched_lock);
}

static int etnaviv_gpu_rpm_suspend(struct device *dev)
{
	struct etnaviv_gpu *gpu = dev_get_drvdata(dev);
	u32 idle, mask;

	/* If there are any jobs in the HW queue, we're not idle */
	if (atomic_read(&gpu->sched.credit_count))
		return -EBUSY;

	/* Check whether the hardware (except FE and MC) is idle */
	mask = gpu->idle_mask & ~(VIVS_HI_IDLE_STATE_FE |
				  VIVS_HI_IDLE_STATE_MC);
	idle = gpu_read(gpu, VIVS_HI_IDLE_STATE) & mask;
	if (idle != mask) {
		dev_warn_ratelimited(dev, "GPU not yet idle, mask: 0x%08x\n",
				     idle);
		return -EBUSY;
	}

	etnaviv_gpu_hw_suspend(gpu);

	gpu->state = ETNA_GPU_STATE_IDENTIFIED;

	return etnaviv_gpu_clk_disable(gpu);
}

static int etnaviv_gpu_rpm_resume(struct device *dev)
{
	struct etnaviv_gpu *gpu = dev_get_drvdata(dev);
	int ret;

	ret = etnaviv_gpu_clk_enable(gpu);
	if (ret)
		return ret;

	/* Re-initialise the basic hardware state */
	if (gpu->state == ETNA_GPU_STATE_IDENTIFIED) {
		ret = etnaviv_gpu_hw_resume(gpu);
		if (ret) {
			etnaviv_gpu_clk_disable(gpu);
			return ret;
		}
	}

	return 0;
}

static const struct dev_pm_ops etnaviv_gpu_pm_ops = {
	RUNTIME_PM_OPS(etnaviv_gpu_rpm_suspend, etnaviv_gpu_rpm_resume, NULL)
};

struct platform_driver etnaviv_gpu_driver = {
	.driver = {
		.name = "etnaviv-gpu",
		.pm = pm_ptr(&etnaviv_gpu_pm_ops),
		.of_match_table = etnaviv_gpu_match,
	},
	.probe = etnaviv_gpu_platform_probe,
	.remove = etnaviv_gpu_platform_remove,
	.id_table = gpu_ids,
};
