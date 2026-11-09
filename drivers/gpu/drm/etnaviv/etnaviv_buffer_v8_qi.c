// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2014-2018 Etnaviv Project
 *
 * Per-chip cmdbuf submission entry point for A311D NPU
 * (VIPNano-QI v8, customer_id 0x88).
 *
 * Phase A initial mechanical split of the per-chip code path. Body is
 * initially a duplicate of
 * etnaviv_buffer_queue to preserve functional equivalence at the
 * cut-over commit. Subsequent commits simplify this file to
 * straight-line A311D logic by removing the customer_id == 0x99
 * gates that don't apply here.
 */

#include <drm/drm_drv.h>
#include <linux/moduleparam.h>
#include <linux/string.h>

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
 * Resident-NBG per-batch drain (A311D 0x88).
 *
 * The open etnaviv_ml runtime (ai/vsi/etnaviv_ml/) emits the whole resident
 * NBG as ONE dispatch stream, byte-identical to the BSP galcore "block 1"
 * (1604 words for yolov2_nntp_graft), and submits it as a single GEM_SUBMIT.
 * Fired back-to-back the FE hangs at the first NN->TP config write while the
 * prior NN op's engine is still busy (idle=0x7ffffff6): the completion drain
 * lives in the BSP's *kernel* FullQueue scheduler ring, NOT the dispatch
 * cmdbuf. See the galcore ioctl-open runtime RE audit, session 6.
 *
 * Port: for a resident 0x88 submit, the KERNEL grows its own suballoc cmdbuf
 * copy and interleaves the BSP FullQueue per-batch drain barrier at the
 * UNK10A4-flush cadence -- WITHOUT touching the userspace dispatch bytes
 * (which stay byte-exact to the BSP). The mesa per-op path (exactly one
 * NN/TP_INST per submit) and S905D3 0x99 are untouched by construction.
 */

/* Cadence: emit one drain barrier after every N-th UNK10A4 flush (the BSP
 * per-batch boundary, ~8 ops). 0 disables the interleave (single end-of-submit
 * event only -- the pre-port behaviour, which is NON-DETERMINISTIC).
 *
 * Default 3. Correctness is a step function at N>=1, not a gradient: the
 * session-54 9e62 A/B sweep measured N=1..12 ALL deterministic + byte-identical
 * to the galcore reference (0x17e6d101a42c6dbb), N=0 non-deterministic, and the
 * cost FLAT across the range (~62 ms, drain tax ~2.9 ms / ~5%). Since cost does
 * not vary with N, the fleet default is the twice-HW-confirmed value 3 -- more
 * serialisation margin than the minimum for as-yet-untested resident graphs, at
 * identical cost. Tune at runtime via /sys/module/etnaviv/parameters/ if needed. */
static unsigned int npu_resident_drain_batch = 3;
module_param_named(npu_resident_drain_batch, npu_resident_drain_batch,
		   uint, 0644);
MODULE_PARM_DESC(npu_resident_drain_batch,
		 "A311D resident-NBG drain: barrier every N UNK10A4 flushes (default 3, 0=off)");

/*
 * A311D (0x88): emit the submit's final completion GL_EVENT with FROM_PE
 * instead of FROM_FE (same two words). FROM_FE signals when the front end
 * reaches the event, before a final NN op's output write has landed, so a
 * reader woken by the fence could see stale output. 0 restores FROM_FE.
 */
static int npu_tail_event_pe = 1;
module_param_named(npu_tail_event_pe, npu_tail_event_pe, int, 0644);
MODULE_PARM_DESC(npu_tail_event_pe,
		 "A311D 0x88: final completion GL_EVENT uses FROM_PE instead of FROM_FE (1 = default, 0 = off)");

/*
 * BSP FullQueue per-batch drain barrier -- byte-exact capture (audit
 * 2026-07-08 session 6, eee46/0x88, yolov2_nntp_graft retiring under nanoq):
 *   SEM 0x701 -> STALL 0x701 -> GL_FLUSH_CACHE 0xc67 ->
 *   NTE_DESCRIPTOR_FLUSH 0 -> SEM 0x30000701 -> STALL 0x30000701.
 * The per-batch GL_EVENT the BSP also emits is galcore completion signalling
 * (FROM_PE), not FE serialisation; it is omitted here (the single end-of-submit
 * GL_EVENT still fences the fence). The load-bearing drain is the SEM/STALL +
 * cache/descriptor flush that makes the FE wait for the NN/TP engine.
 */
static const u32 npu_drain_barrier[] = {
	0x08010e02, 0x00000701,	/* LOAD_STATE GL_SEMAPHORE_TOKEN 0x701     */
	0x48000000, 0x00000701,	/* STALL 0x701                             */
	0x08010e03, 0x00000c67,	/* LOAD_STATE GL_FLUSH_CACHE 0xc67         */
	0x08015311, 0x00000000,	/* LOAD_STATE NTE_DESCRIPTOR_FLUSH 0       */
	0x08010e02, 0x30000701,	/* LOAD_STATE GL_SEMAPHORE_TOKEN 0x30000701*/
	0x48000000, 0x30000701,	/* STALL 0x30000701                        */
};
#define NPU_DRAIN_BARRIER_BYTES sizeof(npu_drain_barrier)

/* A resident stream is a run of count=1 LOAD_STATEs; step in 2-word units. */
static inline bool ls_is_load_state(u32 hdr)
{
	return (hdr >> 27) == 1;
}
static inline u32 ls_offset(u32 hdr)
{
	return hdr & VIV_FE_LOAD_STATE_HEADER_OFFSET__MASK;
}

/*
 * Plan the interleave: count op dispatches + UNK10A4 flushes in the (pre-reloc)
 * userspace stream. Returns the number of drain barriers to insert (0 => not a
 * resident submit -> caller does the plain memcpy) and, via *content_bytes, the
 * grown cmdbuf content length. Reads headers only, so it is reloc-independent.
 */
u32 etnaviv_npu_resident_plan(struct etnaviv_gpu *gpu, const void *stream,
			      u32 stream_bytes, u32 *content_bytes)
{
	const u32 *w = stream;
	u32 nwords = stream_bytes / 4;
	u32 i, n_inst = 0, n_unk = 0, n_barriers;

	*content_bytes = stream_bytes;

	if (!npu_resident_drain_batch)
		return 0;
	if (gpu->identity.customer_id != 0x88 ||
	    gpu->identity.nn_core_count == 0)
		return 0;

	for (i = 0; i + 1 < nwords; i += 2) {
		u32 hdr = w[i];

		if (!ls_is_load_state(hdr))
			return 0;	/* not a uniform LOAD_STATE stream */
		if (ls_offset(hdr) == (VIVS_PS_NN_INST_ADDR >> 2) ||
		    ls_offset(hdr) == (VIVS_PS_TP_INST_ADDR >> 2))
			n_inst++;
		else if (ls_offset(hdr) == (VIVS_PS_UNK10A4 >> 2))
			n_unk++;
	}

	/* The mesa per-op path has exactly one NN/TP_INST per submit; a resident
	 * graph has many. Gate strictly to leave the proven per-op path alone. */
	if (n_inst < 2)
		return 0;

	n_barriers = n_unk / npu_resident_drain_batch;
	if (!n_barriers)
		return 0;

	*content_bytes = stream_bytes + n_barriers * NPU_DRAIN_BARRIER_BYTES;
	return n_barriers;
}

/*
 * Build the interleaved (dispatch + per-batch drain) stream into dst from the
 * reloc-applied stream. Copies every dispatch dword verbatim (byte-exact) and
 * inserts the drain barrier after every npu_resident_drain_batch-th UNK10A4.
 * Returns the content length in bytes (== *content_bytes from _plan()).
 */
u32 etnaviv_npu_resident_build(struct etnaviv_gpu *gpu, void *dst,
			       const void *stream, u32 stream_bytes)
{
	const u32 *src = stream;
	u32 *out = dst;
	u32 nwords = stream_bytes / 4;
	u32 i, o = 0, n_unk = 0;

	for (i = 0; i + 1 < nwords; i += 2) {
		u32 hdr = src[i];

		out[o++] = hdr;
		out[o++] = src[i + 1];

		if (ls_is_load_state(hdr) &&
		    ls_offset(hdr) == (VIVS_PS_UNK10A4 >> 2)) {
			if (++n_unk % npu_resident_drain_batch == 0) {
				memcpy(&out[o], npu_drain_barrier,
				       NPU_DRAIN_BARRIER_BYTES);
				o += NPU_DRAIN_BARRIER_BYTES / 4;
			}
		}
	}

	return o * 4;
}

void etnaviv_buffer_queue_v8_qi(struct etnaviv_gpu *gpu, u32 exec_state,
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
	bool has_blt = !!(gpu->identity.minor_features5 &
			  chipMinorFeatures5_BLT_ENGINE);
	bool is_npu = gpu->identity.nn_core_count > 0;
	/*
	 * Stage 2: A311D (0x88) per-submit flop-reset. Emit the cross-context
	 * AXI-SRAM flop-reset NN op (staged at gpu->nn_flop_inst_va by the init
	 * flop-reset) before each NN submit, clearing the preceding submit's
	 * cross-partition NN-engine state so a fused submit computes correctly
	 * (the alternative to no_batching). 11 8-byte units (10 LOAD_STATE + WAIT).
	 * Active only once the init flop-reset has staged it (nn_flop_inst_va != 0).
	 */
	bool do_flop = is_npu && gpu->identity.customer_id == 0x88 &&
		       gpu->nn_flop_inst_va;

	lockdep_assert_held(&gpu->lock);

	if (drm_debug_enabled(DRM_UT_DRIVER))
		etnaviv_buffer_dump(gpu, buffer, 0, 0x50);

	link_target = etnaviv_cmdbuf_get_va(cmdbuf,
					    &gpu->mmu_context->cmdbuf_mapping);
	link_dwords = cmdbuf->size / 8;

	/*
	 * Per-invoke NN_INST/TP_INST cmdbuf dump. MUST be gated behind
	 * etnaviv_dump_cmdbuf: mn001 emits ~28 NN_INST + several TP_INST per
	 * invoke, and with the fleet default console_loglevel=7 each dev_info
	 * is flushed SYNCHRONOUSLY to the 115200-baud serial console
	 * (console=ttyAML0). That serial blocking added ~270 ms/invoke
	 * (mn001 274 ms -> 9 ms with the dump off; isolated 2026-06-12 via the
	 * armed->run_job probe -- it was masquerading as an "NPU is slow" /
	 * drm_sched-pipeline gap). The S905D3 sibling
	 * (etnaviv_buffer_v8_pico.c) already gates the identical dump behind
	 * etnaviv_dump_cmdbuf; this path was left ungated by the Phase-A
	 * split.
	 */
	if (is_npu && etnaviv_dump_cmdbuf) {
		u32 *cmdbuf_words = cmdbuf->vaddr;
		unsigned int nwords = cmdbuf->size / 4;

		dev_info(gpu->dev, "NPU submit: cmdbuf_va=0x%08x size=%u switch_ctx=%d need_flush=%d flush_seq=%u/%u switch_mmu=%d\n",
			 link_target, cmdbuf->size, switch_context, need_flush,
			 gpu->flush_seq, new_flush_seq, switch_mmu_context);
		/* Scan for NN_INST_ADDR (0x0428) or TP_INST_ADDR (0x042E) */
		{
			unsigned int w;
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
		}
	}

	/*
	 * If we need maintenance prior to submitting this buffer, we will
	 * need to append a mmu flush load state, followed by a new
	 * link to this buffer - a total of four additional words.
	 */
	if (need_flush || switch_context || do_flop) {
		u32 target, extra_dwords;

		/* link command */
		extra_dwords = 1;

		/* flush command */
		if (need_flush) {
			if (gpu->mmu_context->global->version == ETNAVIV_IOMMU_V1)
				extra_dwords += 1;
			else
				extra_dwords += 3;
		}

		/* pipe switch commands */
		if (switch_context)
			extra_dwords += 4;

		/* Stage 2 per-submit flop-reset: 10 CMD_LOAD_STATE + 1 CMD_WAIT
		 * = 11 8-byte units (etnaviv_buffer_reserve counts u64 units; this
		 * is also the LINK prefetch, so it MUST equal the emitted count). */
		if (do_flop)
			extra_dwords += 11;

		/* PTA load command -- emitted on A311D 0x88 (this file) when
		 * SEC_KERNEL switch_mmu_context. The 0x99 customer skips it
		 * (separate file etnaviv_buffer_v8_pico.c).
		 */
		if (switch_mmu_context && gpu->sec_mode == ETNA_SEC_KERNEL)
			extra_dwords += 1;

		target = etnaviv_buffer_reserve(gpu, buffer, extra_dwords);
		/*
		 * Switch MMU context if necessary. Must be done after the
		 * link target has been calculated, as the jump forward in the
		 * kernel ring still uses the last active MMU context before
		 * the switch.
		 */
		if (switch_mmu_context) {
			struct etnaviv_iommu_context *old_context = gpu->mmu_context;

			gpu->mmu_context = etnaviv_iommu_context_get(mmu_context);
			etnaviv_iommu_context_put(old_context);
		}

		if (need_flush) {
			/* Add the MMU flush */
			if (gpu->mmu_context->global->version == ETNAVIV_IOMMU_V1) {
				CMD_LOAD_STATE(buffer, VIVS_GL_FLUSH_MMU,
					       VIVS_GL_FLUSH_MMU_FLUSH_FEMMU |
					       VIVS_GL_FLUSH_MMU_FLUSH_UNK1 |
					       VIVS_GL_FLUSH_MMU_FLUSH_UNK2 |
					       VIVS_GL_FLUSH_MMU_FLUSH_PEMMU |
					       VIVS_GL_FLUSH_MMU_FLUSH_UNK4);
			} else {
				u32 flush = VIVS_MMUv2_CONFIGURATION_MODE_MASK |
					    VIVS_MMUv2_CONFIGURATION_FLUSH_FLUSH;

				if (switch_mmu_context &&
				    gpu->sec_mode == ETNA_SEC_KERNEL) {
					unsigned short id =
						etnaviv_iommuv2_get_pta_id(gpu->mmu_context);
					CMD_LOAD_STATE(buffer,
						VIVS_MMUv2_PTA_CONFIG,
						VIVS_MMUv2_PTA_CONFIG_INDEX(id));
				}

				if (gpu->sec_mode == ETNA_SEC_NONE ||
				    gpu->identity.nn_core_count > 0)
					flush |= etnaviv_iommuv2_get_mtlb_addr(gpu->mmu_context);

				if (gpu->identity.nn_core_count > 0)
					dev_info(gpu->dev,
						 "NPU MMU flush=0x%08x sec_mode=%d\n",
						 flush, gpu->sec_mode);

				CMD_LOAD_STATE(buffer, VIVS_MMUv2_CONFIGURATION,
					       flush);
				if (gpu->identity.nn_core_count > 0) {
					/*
					 * A311D 0x88 NPU: no PE for SEM/STALL.
					 * Use WAIT to ensure MMU TLB flush
					 * completes before NN/TP accesses.
					 */
					CMD_WAIT(buffer, 32000);
				} else {
					CMD_SEM(buffer, SYNC_RECIPIENT_FE,
						SYNC_RECIPIENT_PE);
					CMD_STALL(buffer, SYNC_RECIPIENT_FE,
						SYNC_RECIPIENT_PE);
				}
			}

			gpu->flush_seq = new_flush_seq;
		}

		if (switch_context) {
			etnaviv_cmd_select_pipe(gpu, buffer, exec_state);
			gpu->exec_state = exec_state;
		}

		/*
		 * Stage 2: emit the cross-context AXI-SRAM flop-reset NN op
		 * before the user submit. Clears the preceding submit's
		 * cross-partition NN-engine state so a fused (batched) submit
		 * computes correctly. NN_CONFIG=0 -> FE stalls at NN_INST until
		 * the flop conv completes; CMD_WAIT lets the SH drain before the
		 * user op. A residual SH wedge here is caught by the (merged)
		 * Candidate-B fast-recover. 11 8-byte units (matches extra_dwords).
		 */
		if (do_flop) {
			CMD_LOAD_STATE(buffer, VIVS_PA_SYSTEM_MODE, 0x11);
			CMD_LOAD_STATE(buffer, VIVS_GL_API_MODE,
				       VIVS_GL_API_MODE_OPENCL);
			CMD_LOAD_STATE(buffer, VIVS_GL_SRAM_REMAP_ADDRESS,
				       0x00400000);
			CMD_LOAD_STATE(buffer, VIVS_GL_SRAM_REMAP_ADDRESS,
				       0x00400000);
			CMD_LOAD_STATE(buffer, VIVS_GL_OCB_REMAP_START, 0x0);
			CMD_LOAD_STATE(buffer, VIVS_GL_OCB_REMAP_END, 0x0);
			/* NN_CONFIG=0x10: dispatch value proven by the init
			 * flop-reset's WORKING short-sequence "2nd run" (out=86,
			 * loops=1). The init MAIN run got away with 0x0 only by
			 * pairing it with the longer HI_COMPRESSION+MMU-flush
			 * prelude; this inline short sequence needs 0x10 to
			 * dispatch the NN -- with 0x0 the FE stalled at NN_INST
			 * and the NN never ran (idle=0x7fffbffe, audit y5). */
			/* qi:N1443: DDR_BURST_SIZE (0x60) override, if armed.
			 * Recorded because this register cannot be read back:
			 * the witness is the value that went into the stream. */
			{
				u32 nncfg = etnaviv_npu_nn_config(gpu, 0x10);

				CMD_LOAD_STATE(buffer, VIVS_GL_NN_CONFIG, nncfg);
				etnaviv_npu_nn_config_note_prelude(nncfg);
			}
			CMD_LOAD_STATE(buffer, VIVS_GL_SHADER_INDEX, 0x0);
			CMD_LOAD_STATE(buffer, VIVS_PS_NN_INST_ADDR,
				       gpu->nn_flop_inst_va);
			CMD_LOAD_STATE(buffer, VIVS_PS_UNK10A4, 0x0);
			CMD_WAIT(buffer, 32000);
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
	 * the ring buffer.
	 *
	 * For A311D NPU: 3 dwords (event + wait + link). GL_EVENT
	 * triggers the ISR; ISR signals fence immediately (SMALL_BATCH
	 * stalls FE at NN_INST_ADDR). WAIT+LINK keeps FE alive in the
	 * idle loop after the event fires.
	 */
	if (is_npu)
		return_dwords = 3;
	else
		return_dwords = 7;

	/*
	 * When the BLT engine is present we need 6 more dwords in the return
	 * target: 3 enable/flush/disable + 4 enable/semaphore stall/disable,
	 * but we don't need the normal TS flush state.
	 */
	if (has_blt)
		return_dwords += 6;

	return_target = etnaviv_buffer_reserve(gpu, buffer, return_dwords);
	CMD_LINK(cmdbuf, return_dwords, return_target);

	if (is_npu && etnaviv_dump_cmdbuf)
		dev_info(gpu->dev, "NPU submit: return_target=0x%08x ring_va=0x%08x\n",
			 return_target,
			 etnaviv_cmdbuf_get_va(buffer, &gpu->mmu_context->cmdbuf_mapping));

	/*
	 * Append a cache flush, stall, event, wait and link pointing back to
	 * the wait command to the ring buffer.
	 *
	 * For NPU-only A311D: skip the cache flush and PE semaphore/stall
	 * since there is no Pixel Engine. Use FROM_FE for the event instead
	 * of FROM_PE.
	 */
	if (is_npu) {
		/* A311D 0x88 NPU: GL_EVENT FROM_FE; SMALL_BATCH stalls FE
		 * at NN_INST_ADDR, so GL_EVENT fires after NN completes.
		 * ISR signals fence immediately (no npu_poll deferral on 0x88).
		 */
		CMD_LOAD_STATE(buffer, VIVS_GL_EVENT,
			       VIVS_GL_EVENT_EVENT_ID(event) |
			       (npu_tail_event_pe &&
				gpu->identity.customer_id == 0x88 ?
				VIVS_GL_EVENT_FROM_PE :
				VIVS_GL_EVENT_FROM_FE));
	} else {
		if (gpu->exec_state == ETNA_PIPE_2D) {
			CMD_LOAD_STATE(buffer, VIVS_GL_FLUSH_CACHE,
					       VIVS_GL_FLUSH_CACHE_PE2D);
		} else {
			CMD_LOAD_STATE(buffer, VIVS_GL_FLUSH_CACHE,
					       VIVS_GL_FLUSH_CACHE_DEPTH |
					       VIVS_GL_FLUSH_CACHE_COLOR |
					       VIVS_GL_FLUSH_CACHE_SHADER_L1);
			if (has_blt) {
				CMD_LOAD_STATE(buffer, VIVS_BLT_ENABLE, 0x1);
				CMD_LOAD_STATE(buffer, VIVS_BLT_SET_COMMAND, 0x1);
				CMD_LOAD_STATE(buffer, VIVS_BLT_ENABLE, 0x0);
			} else {
				CMD_LOAD_STATE(buffer, VIVS_TS_FLUSH_CACHE,
						       VIVS_TS_FLUSH_CACHE_FLUSH);
			}
		}
		CMD_SEM(buffer, SYNC_RECIPIENT_FE, SYNC_RECIPIENT_PE);
		CMD_STALL(buffer, SYNC_RECIPIENT_FE, SYNC_RECIPIENT_PE);

		if (has_blt) {
			CMD_LOAD_STATE(buffer, VIVS_BLT_ENABLE, 0x1);
			CMD_SEM(buffer, SYNC_RECIPIENT_FE, SYNC_RECIPIENT_BLT);
			CMD_STALL(buffer, SYNC_RECIPIENT_FE, SYNC_RECIPIENT_BLT);
			CMD_LOAD_STATE(buffer, VIVS_BLT_ENABLE, 0x0);
		}

		CMD_LOAD_STATE(buffer, VIVS_GL_EVENT,
			       VIVS_GL_EVENT_EVENT_ID(event) |
			       VIVS_GL_EVENT_FROM_PE);
	}

	/* WAIT+LINK keeps FE alive in the ring's idle loop after
	 * GL_EVENT fires (signal happens immediately on 0x88; FE must
	 * not execute stale ring buffer content).
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
