// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2014-2018 Etnaviv Project
 *
 * Per-chip cmdbuf submission entry point for S905D3 NPU
 * (VIPPico v3, customer_id 0x99).
 *
 * Phase A initial split of the per-chip code path. Bakes in the cumulative
 * R47+R51+R52 work:
 * - R47: no PTA_CONFIG cmdbuf LOAD_STATE
 * - R51: no post-flush SEM+STALL, no pipe_select
 * - R52: no cmdbuf-emitted MMU flush LOAD_STATE; flush done via
 *        direct MMIO write (gpu_write) before the LINK. The BSP
 *        gckMMU model never emits VIVS_MMUv2_CONFIGURATION via
 *        cmdbuf; it uses direct MMIO at probe time and before each
 *        LINK kick. VIPPico v3 silicon hangs FE at CMD_LOAD_STATE
 *        VIVS_MMUv2_CONFIGURATION when it appears in the ring buffer
 *        (confirmed by r91 ring dump: last-fetch 0x08010061 0x04112018
 *        = CMD_LOAD_STATE reg=0x184 in the stalled state).
 *
 * Net 0x99 NPU prelude after R52:
 *   MMIO write VIVS_MMUv2_CONFIGURATION (flush) -> LINK to cmdbuf
 *   (no PTA_CONFIG, no MMU-flush LOAD_STATE in ring, no SEM+STALL,
 *    no pipe_select)
 *
 * The 0x88 customer's full prelude (PTA_CONFIG + MMU flush LOAD_STATE +
 * WAIT + pipe_select) lives in the sibling file etnaviv_buffer_v8_qi.c.
 *
 * R52 was excluded from Phase A because in the shared etnaviv_buffer.c
 * path it interacted with the strike-3 npu_poll_completion-gated re-arm
 * in etnaviv_sched.c:etnaviv_sched_timedout_job, regressing /019.
 * Now that 0x99 logic is isolated to this file, the interaction is gone.
 * Reference: the TP-only single-op stall localization audit.
 */

#include <linux/moduleparam.h>

#include <drm/drm_drv.h>

#include "etnaviv_buffer_internal.h"
#include "etnaviv_cmdbuf.h"
#include "etnaviv_gpu.h"
#include "etnaviv_gem.h"
#include "etnaviv_mmu.h"

#include "common.xml.h"
#include "state.xml.h"
#include "state_blt.xml.h"
#include "state_hi.xml.h"
#include "state_3d.xml.h"
#include "cmdstream.xml.h"

/*
 * pico:N1251 -- see the ring_flush block in etnaviv_buffer_queue_v8_pico().
 * Default 1: with it a second process is exact (with the first still open,
 * and after it exits); with 0 its jobs ran through stale translations, and
 * once the first context was freed the NPU wrote freed kernel memory.
 */
static unsigned int npu_pico_ring_mmu_flush = 1;
module_param_named(npu_pico_ring_mmu_flush, npu_pico_ring_mmu_flush, uint, 0644);
MODULE_PARM_DESC(npu_pico_ring_mmu_flush,
	"0x99 MMU flush: 0 = MMIO write; 1 = ring LINK + LOAD_STATE flush + LINK (galcore shape, default); 2 = 1 plus FE->PE semaphore + stall");

void etnaviv_buffer_queue_v8_pico(struct etnaviv_gpu *gpu, u32 exec_state,
	struct etnaviv_iommu_context *mmu_context, unsigned int event,
	struct etnaviv_cmdbuf *cmdbuf)
{
	struct etnaviv_cmdbuf *buffer = &gpu->buffer;
	unsigned int waitlink_offset = buffer->user_size - 16;
	u32 return_target, return_dwords;
	u32 link_target, link_dwords;
	bool switch_context = gpu->exec_state != exec_state;
	bool switch_mmu_context = gpu->mmu_context != mmu_context;
	unsigned int new_flush_seq = READ_ONCE(mmu_context->flush_seq);
	bool need_flush = switch_mmu_context || gpu->flush_seq != new_flush_seq;
	/* pico:N253 -- GL_EVENT source for THIS submit; see below. */
	bool fe_event = false;

	lockdep_assert_held(&gpu->lock);

	if (drm_debug_enabled(DRM_UT_DRIVER))
		etnaviv_buffer_dump(gpu, buffer, 0, 0x50);

	link_target = etnaviv_cmdbuf_get_va(cmdbuf,
					    &gpu->mmu_context->cmdbuf_mapping);
	link_dwords = cmdbuf->size / 8;

	if (etnaviv_dump_cmdbuf) {
		u32 *cmdbuf_words = cmdbuf->vaddr;
		unsigned int nwords = cmdbuf->size / 4;
		unsigned int w;

		dev_info(gpu->dev, "NPU submit: cmdbuf_va=0x%08x size=%u switch_ctx=%d need_flush=%d flush_seq=%u/%u switch_mmu=%d\n",
			 link_target, cmdbuf->size, switch_context, need_flush,
			 gpu->flush_seq, new_flush_seq, switch_mmu_context);

		/* Scan for NN_INST_ADDR (0x0428) or TP_INST_ADDR (0x042E) */
		for (w = 0; w + 1 < nwords; w += 2) {
			u32 hdr = cmdbuf_words[w];
			u16 off = hdr & VIV_FE_LOAD_STATE_HEADER_OFFSET__MASK;
			if (off == (VIVS_PS_NN_INST_ADDR >> 2) ||
			    off == (VIVS_PS_TP_INST_ADDR >> 2)) {
				dev_info(gpu->dev,
					 "NPU cmdbuf: %s at word %u, addr=0x%08x\n",
					 off == (VIVS_PS_NN_INST_ADDR >> 2) ?
					 "NN_INST" : "TP_INST",
					 w, cmdbuf_words[w+1]);
			}
		}

		/* NEXT-26 (2026-05-25): full cmdbuf word stream dump for
		 * mesa-vs-BSP FE LINK chain diff. Matches galcore-port
		 * CMDBUF_DUMP format so the two captures diff cleanly. */
		{
			unsigned int j;
			dev_info(gpu->dev,
				 "CMDBUF_DUMP_MESA: cmdbuf_va=0x%08x words=%u start\n",
				 link_target, nwords);
			for (j = 0; j < nwords; j += 4) {
				u32 w0 = cmdbuf_words[j];
				u32 w1 = (j + 1 < nwords) ? cmdbuf_words[j + 1] : 0;
				u32 w2 = (j + 2 < nwords) ? cmdbuf_words[j + 2] : 0;
				u32 w3 = (j + 3 < nwords) ? cmdbuf_words[j + 3] : 0;
				dev_info(gpu->dev,
					 "CMDBUF_DUMP_MESA:   [%4u] %08x %08x %08x %08x\n",
					 j, w0, w1, w2, w3);
			}
			dev_info(gpu->dev, "CMDBUF_DUMP_MESA: end\n");
		}
	}

	/*
	 * Prelude is the minimal kernel-ring needed for c64a 0x99
	 * silicon. Cumulative R47+R51 work removed everything that
	 * hangs FE at LOAD_STATE post-MMU-flush:
	 * - PTA_CONFIG (R47)        -- skipped here
	 * - SEM+STALL FE->PE (R51)  -- skipped here
	 * - etnaviv_cmd_select_pipe (R51) -- skipped here
	 * Per-op cmdbuf state writes (NN_CONFIG, NN_INST_ADDR) emitted
	 * by mesa take a different silicon path that doesn't choke.
	 */
	if (need_flush || switch_context) {
		u32 target, extra_dwords;
		unsigned int ring_flush;

		/* link command */
		extra_dwords = 1;

		/*
		 * R52: MMU flush is done via direct MMIO write (gpu_write)
		 * before the LINK, not via cmdbuf LOAD_STATE. VIPPico v3
		 * silicon hangs FE at CMD_LOAD_STATE VIVS_MMUv2_CONFIGURATION
		 * in the ring buffer (audit 2026-05-09-tp-only-single-op-
		 * stall-localization.md, finding 3: last-fetch 0x08010061
		 * 0x04112018 = CMD_LOAD_STATE reg=0x184 = VIVS_MMUv2_CONFIGURATION).
		 * V1 FLUSH_MMU is also done via MMIO for consistency.
		 * No cmdbuf space needed for the flush -- only the LINK.
		 */

		/* No PTA_CONFIG (R47), no MMU-flush LOAD_STATE (R52),
		 * no SEM+STALL (R51), no pipe_select (R51) on 0x99.
		 */

		/*
		 * pico:N1251 -- the MMIO flush does not drop the TLB entries a
		 * context switch must drop (a second process's jobs ran through
		 * the first's translations).  npu_pico_ring_mmu_flush selects a
		 * ring-emitted flush shaped like galcore's gckHARDWARE_FlushMMU
		 * instead: 1 = LINK-next, LOAD_STATE flush (no table address),
		 * LINK-next; 2 = the same with FE->PE semaphore + stall.
		 */
		ring_flush = (need_flush &&
			      mmu_context->global->version != ETNAVIV_IOMMU_V1) ?
			     npu_pico_ring_mmu_flush : 0;
		if (ring_flush == 1)
			extra_dwords += 3;
		else if (ring_flush >= 2)
			extra_dwords += 5;

		target = etnaviv_buffer_reserve(gpu, buffer, extra_dwords);

		/*
		 * Switch MMU context if necessary. Must be done after the
		 * link target has been calculated.
		 */
		if (switch_mmu_context) {
			struct etnaviv_iommu_context *old_context = gpu->mmu_context;

			gpu->mmu_context = etnaviv_iommu_context_get(mmu_context);
			etnaviv_iommu_context_put(old_context);
		}

		if (need_flush) {
			/*
			 * R52: flush MMU via direct MMIO, NOT via cmdbuf
			 * LOAD_STATE. VIPPico v3 hangs FE at CMD_LOAD_STATE
			 * VIVS_MMUv2_CONFIGURATION in the ring buffer. The BSP
			 * gckMMU flushes via a direct register write before
			 * issuing the FE LINK kick; mirror that pattern here.
			 *
			 * The gpu_write() happens while gpu->lock is held and
			 * before etnaviv_buffer_replace_wait() kicks the FE,
			 * so the MMIO flush completes before any FE fetch of
			 * the new cmdbuf.
			 */
			if (gpu->mmu_context->global->version == ETNAVIV_IOMMU_V1) {
				gpu_write(gpu, VIVS_GL_FLUSH_MMU,
					  VIVS_GL_FLUSH_MMU_FLUSH_FEMMU |
					  VIVS_GL_FLUSH_MMU_FLUSH_UNK1 |
					  VIVS_GL_FLUSH_MMU_FLUSH_UNK2 |
					  VIVS_GL_FLUSH_MMU_FLUSH_PEMMU |
					  VIVS_GL_FLUSH_MMU_FLUSH_UNK4);
			} else {
				u32 flush = VIVS_MMUv2_CONFIGURATION_MODE_MASK |
					    VIVS_MMUv2_CONFIGURATION_FLUSH_FLUSH;
				/*
				 * The MMU walks the master TLB it latched at
				 * restore, never this context's own (no
				 * PTA_CONFIG on this chip): copy the running
				 * context's master TLB into it first.  This is
				 * the context switch; without it every later
				 * process ran through the first one's tables.
				 */
				u32 mtlb = etnaviv_iommuv2_sync_hw_mtlb(gpu->mmu_context);

				if (gpu->sec_mode == ETNA_SEC_NONE ||
				    gpu->identity.nn_core_count > 0)
					flush |= mtlb;

				dev_info(gpu->dev,
					 "NPU MMU flush=0x%08x sec_mode=%d ring_flush=%u\n",
					 flush, gpu->sec_mode, ring_flush);

				if (!ring_flush) {
					/* Direct MMIO write -- no cmdbuf LOAD_STATE */
					gpu_write(gpu, VIVS_MMUv2_CONFIGURATION, flush);
				} else {
					u32 va = etnaviv_cmdbuf_get_va(buffer,
							&gpu->mmu_context->cmdbuf_mapping);

					/* LINK to the next slot: drains the FE prefetch */
					buffer->user_size = ALIGN(buffer->user_size, 8);
					CMD_LINK(buffer, ring_flush >= 2 ? 4 : 2,
						 va + buffer->user_size + 8);
					/* galcore's value: FLUSH set, its mask bit
					 * (7) clear, every other bit 1 -- mode and
					 * address masked, no table address */
					CMD_LOAD_STATE(buffer, VIVS_MMUv2_CONFIGURATION,
						       ~VIVS_MMUv2_CONFIGURATION_FLUSH_MASK);
					if (ring_flush >= 2) {
						CMD_SEM(buffer, SYNC_RECIPIENT_FE,
							SYNC_RECIPIENT_PE);
						CMD_STALL(buffer, SYNC_RECIPIENT_FE,
							  SYNC_RECIPIENT_PE);
					}
					CMD_LINK(buffer, 1, va + buffer->user_size + 8);
				}
			}

			gpu->flush_seq = new_flush_seq;
		}

		if (switch_context) {
			/* R51: skip etnaviv_cmd_select_pipe on 0x99
			 * (pipe_select emits GL_FLUSH_CACHE +
			 * GL_PIPE_SELECT LOAD_STATEs which hang FE on
			 * this silicon). NPU has no 2D/3D pipe to
			 * switch -- the call is dead code on NPU.
			 * Track exec_state without emitting commands.
			 */
			gpu->exec_state = exec_state;
		}

		/* And the link to the submitted buffer */
		link_target = etnaviv_cmdbuf_get_va(cmdbuf,
					&gpu->mmu_context->cmdbuf_mapping);
		CMD_LINK(buffer, link_dwords, link_target);

		/* Update the link target to point to above instructions */
		link_target = target;
		link_dwords = extra_dwords;
	}

	/*
	 * Append a LINK to the submitted command buffer to return to
	 * the ring buffer. For 0x99 NPU: 3 dwords (event + wait +
	 * link). GL_EVENT triggers the ISR; ISR defers to
	 * npu_poll_work which waits for NN/TP idle, then signals.
	 * WAIT+LINK keeps FE alive in the idle loop after the event
	 * fires.
	 */
	return_dwords = 3;

	return_target = etnaviv_buffer_reserve(gpu, buffer, return_dwords);
	CMD_LINK(cmdbuf, return_dwords, return_target);

	if (etnaviv_dump_cmdbuf)
		dev_info(gpu->dev, "NPU submit: return_target=0x%08x ring_va=0x%08x\n",
			 return_target,
			 etnaviv_cmdbuf_get_va(buffer, &gpu->mmu_context->cmdbuf_mapping));

	/*
	 * VIPPico v3 (S905D3, 0x99): SMALL_BATCH does NOT stall FE.
	 * GL_EVENT fires before NN completes. ISR defers to
	 * npu_poll_work which waits for NN/TP idle, then signals.
	 *
	 * Only defer events for cmdbufs with NN/TP instructions; a cmdbuf
	 * with none (mesa's once-per-context config prefix -- PA_SYSTEM_MODE,
	 * GL_NN_CONFIG, the SRAM/OCB remap pair, then a LINK) has no NN/TP
	 * work for the poll to wait on, and takes the FROM_FE route below.
	 *
	 * pico:N253 -- what actually decides whether that config-only cmdbuf
	 * raises an interrupt is the DEVICE STATE, not the cmdbuf contents.
	 * An earlier revision of this comment claimed "a cmdbuf with no
	 * pipeline work has no PE completion to fire from"; that is FALSE and
	 * was measured false.  The same config-only cmdbuf raises its GL_EVENT
	 * FROM_PE every time the device is COLD (6/6, in every completion mode
	 * including stock).  It is only on a WARM device -- re-emitting the
	 * config prefix on an already-initialised DRM fd -- that the FROM_PE
	 * never arrives and the submit takes ZERO NPU interrupts -- under
	 * stock it has no phase-ring row at all -- leaving the drm_sched
	 * watchdog to end it.  So the PE can and does complete this cmdbuf;
	 * warmth is the discriminator.
	 */
	if (gpu->npu_poll_completion) {
		bool has_nn_tp = false;
		unsigned int w, nw = cmdbuf->size / 4;
		u32 *cw = cmdbuf->vaddr;

		for (w = 0; w + 1 < nw; w += 2) {
			u16 off = cw[w] & VIV_FE_LOAD_STATE_HEADER_OFFSET__MASK;
			if (off == (VIVS_PS_NN_INST_ADDR >> 2) ||
			    off == (VIVS_PS_TP_INST_ADDR >> 2)) {
				has_nn_tp = true;
				break;
			}
		}
		/*
		 * pico:N253 -- the FE-only (config-only) class takes an
		 * FE-sourced GL_EVENT, which the ISR signals directly; see
		 * the NEXT-14 block below for why that is correct here.  The
		 * poll arming is unchanged from stock: only a cmdbuf that
		 * actually dispatched NN/TP work arms it, so the completion
		 * route for that class is bit-for-bit what it was.
		 */
		fe_event = !has_nn_tp;

		if (has_nn_tp) {
			gpu->npu_poll_event = event;
			/*
			 * Proactively schedule the completion poll. conv12
			 * (first 14x14x512) leaves the SH busy after NN_INST, so
			 * GL_EVENT FROM_PE never fires -> the ISR never schedules
			 * npu_poll_work -> the 500 ms watchdog recovers (wiping
			 * on-chip state -> cascade -> constant output). galcore
			 * completes the same op via an FE-DMA-status poll, NOT the
			 * SH-gated event. Mirror that: schedule npu_poll_work on a
			 * 100 ms delay (< the 500 ms watchdog) so it signals the
			 * fence on NN/TP idle regardless of the SH-gated GL_EVENT.
			 * For ops whose FROM_PE DOES fire, the ISR path
			 * (mod_delayed_work .,0) runs it sooner; npu_poll_work is
			 * one-shot (clears npu_poll_event) so whichever fires first
			 * wins -- NOT the unbounded watchdog re-arm that
			 * infinite-looped (NEXT-37-LB1).
			 */
			mod_delayed_work(system_wq, &gpu->npu_poll_work,
					 msecs_to_jiffies(100));
		}
	}
	/* NEXT-14 (2026-05-23): GL_EVENT source = FROM_PE, matching BSP.
	 *
	 * The 2026-05-05 c64a hswish BSP cmdbuf-walker capture shows BSP
	 * emits `GL_EVENT = event_id | FROM_PE (0x40)` per op. The kernel
	 * previously emitted `event_id | FROM_FE (0x20)`. FROM_FE fires
	 * the event when FE *reaches* the LOAD_STATE -- i.e., immediately
	 * after the NN dispatch is issued, BEFORE NN actually writes its
	 * output BO. The npu_poll_completion defer below was a workaround
	 * for this early-fire (defers fence signal until NN/TP idle), but
	 * NN can briefly idle between compute and writeback flush; if the
	 * deferral races that gap, userspace gets a not-yet-written BO
	 * (0xAA debug fill) -- the universal "NN-not-writing" symptom
	 * documented in an internal NN-writeback audit.
	 *
	 * BSP's FROM_PE makes the event fire only after the pipeline
	 * engine completes the work (including the AXI writeback). The
	 * fence-signal-vs-userspace-read race window closes by construction;
	 * the poll-defer becomes a no-op.
	 *
	 * Companion file etnaviv_buffer_v8_qi.c:265-267 already uses
	 * FROM_PE for A311D BATCHING path (the SMALL_BATCH path on A311D
	 * uses FROM_FE only because that chip's FE stalls at NN_INST_ADDR
	 * -- on c64a 0x99 the FE does NOT stall, so FROM_FE fires far too
	 * early).
	 *
	 * See: the internal FROM_PE event-source audit and the
	 *   NN-not-writing-output audit.
	 *
	 * pico:N253 -- NEXT-14's reason is a statement about NN WRITEBACK and
	 * nothing else, so it applies only to a cmdbuf that dispatches NN/TP
	 * work.  The FE-only (config-only) class allocates no output BO and
	 * produces no writeback, so there is nothing for an FE-sourced event
	 * to be early with respect to.  The GL_EVENT sits in the kernel RING,
	 * after the LINK that returns from the cmdbuf, so "the FE reached it"
	 * means the FE has already consumed every LOAD_STATE that cmdbuf
	 * contained -- FE-reached IS the completion for FE-only work.  The
	 * consumer of those config states is the NEXT submit's NN/TP work,
	 * issued later in the same FE stream, so FE ordering carries the
	 * dependency regardless of when the event fires.
	 *
	 * Measured (pico:N253, whole-model crux arm on board
	 * aml-s905d3-cc-4g-c64a48092a00): 4.918 / 4.797 / 4.919 s on the
	 * FROM_PE-for-everything stock route, vs 0.337 / 0.341 / 0.343 s
	 * here.  The warm config submit's own phase-ring row goes from
	 * ABSENT under stock (no interrupt was ever raised) to
	 * `irq  fe_walk 9  nn_exec 0  drain 27  total 37` us here -- path
	 * `irq`, i.e. signalled straight from the ISR with no workqueue hop.
	 * 9 of 9 NPU-dispatching fixtures stayed bit-identical by full
	 * output-tensor md5, recover=0 on all 33 runs.
	 */
	CMD_LOAD_STATE(buffer, VIVS_GL_EVENT,
		       VIVS_GL_EVENT_EVENT_ID(event) |
		       (fe_event ? VIVS_GL_EVENT_FROM_FE
				 : VIVS_GL_EVENT_FROM_PE));

	/* WAIT+LINK is needed to keep FE alive after GL_EVENT fires.
	 * On 0x99 the npu_poll_work signal is deferred; FE must be in
	 * the ring's idle loop, not executing stale ring buffer content.
	 */
	CMD_WAIT(buffer, gpu->fe_waitcycles);
	CMD_LINK(buffer, 2,
		 etnaviv_cmdbuf_get_va(buffer,
				&gpu->mmu_context->cmdbuf_mapping)
		 + buffer->user_size - 4);

	if (drm_debug_enabled(DRM_UT_DRIVER))
		pr_info("stream link to 0x%08x @ 0x%08x %p\n",
			return_target,
			etnaviv_cmdbuf_get_va(cmdbuf, &gpu->mmu_context->cmdbuf_mapping),
			cmdbuf->vaddr);

	if (drm_debug_enabled(DRM_UT_DRIVER)) {
		print_hex_dump(KERN_INFO, "cmd ", DUMP_PREFIX_OFFSET, 16, 4,
			       cmdbuf->vaddr, cmdbuf->size, 0);

		pr_info("link op: %p\n", buffer->vaddr + waitlink_offset);
		pr_info("addr: 0x%08x\n", link_target);
		pr_info("back: 0x%08x\n", return_target);
		pr_info("event: %d\n", event);
	}

	/*
	 * Kick off the submitted command by replacing the previous
	 * WAIT with a link to the address in the ring buffer.
	 */
	etnaviv_buffer_replace_wait(buffer, waitlink_offset,
				    VIV_FE_LINK_HEADER_OP_LINK |
				    VIV_FE_LINK_HEADER_PREFETCH(link_dwords),
				    link_target);

	if (drm_debug_enabled(DRM_UT_DRIVER))
		etnaviv_buffer_dump(gpu, buffer, 0, 0x50);
}
