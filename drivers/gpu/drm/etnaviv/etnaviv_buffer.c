// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2014-2018 Etnaviv Project
 */

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
 * Command Buffer helper:
 *
 * Inline command builders (OUT, CMD_LOAD_STATE, CMD_END, CMD_WAIT,
 * CMD_LINK, CMD_STALL, CMD_SEM) live in etnaviv_buffer_internal.h
 * as `static inline` so etnaviv_buffer.c, etnaviv_buffer_v8_qi.c, and
 * etnaviv_buffer_v8_pico.c each get their own copy.
 *
 * The larger helpers below (etnaviv_cmd_select_pipe,
 * etnaviv_buffer_dump, etnaviv_buffer_replace_wait,
 * etnaviv_buffer_reserve) live in this file but are exposed via the
 * internal header so the per-chip files can call them.
 */

void etnaviv_cmd_select_pipe(struct etnaviv_gpu *gpu,
	struct etnaviv_cmdbuf *buffer, u8 pipe)
{
	u32 flush = 0;

	lockdep_assert_held(&gpu->lock);

	/*
	 * This assumes that if we're switching to 2D, we're switching
	 * away from 3D, and vice versa.  Hence, if we're switching to
	 * the 2D core, we need to flush the 3D depth and color caches,
	 * otherwise we need to flush the 2D pixel engine cache.
	 */
	if (gpu->exec_state == ETNA_PIPE_2D)
		flush = VIVS_GL_FLUSH_CACHE_PE2D;
	else if (gpu->exec_state == ETNA_PIPE_3D)
		flush = VIVS_GL_FLUSH_CACHE_DEPTH | VIVS_GL_FLUSH_CACHE_COLOR;

	CMD_LOAD_STATE(buffer, VIVS_GL_FLUSH_CACHE, flush);
	if (gpu->identity.nn_core_count == 0) {
		CMD_SEM(buffer, SYNC_RECIPIENT_FE, SYNC_RECIPIENT_PE);
		CMD_STALL(buffer, SYNC_RECIPIENT_FE, SYNC_RECIPIENT_PE);
	}

	CMD_LOAD_STATE(buffer, VIVS_GL_PIPE_SELECT,
		       VIVS_GL_PIPE_SELECT_PIPE(pipe));
}

void etnaviv_buffer_dump(struct etnaviv_gpu *gpu,
	struct etnaviv_cmdbuf *buf, u32 off, u32 len)
{
	u32 size = buf->size;
	u32 *ptr = buf->vaddr + off;

	dev_info(gpu->dev, "virt %p phys 0x%08x free 0x%08x\n",
			ptr, etnaviv_cmdbuf_get_va(buf,
			&gpu->mmu_context->cmdbuf_mapping) +
			off, size - len * 4 - off);

	print_hex_dump(KERN_INFO, "cmd ", DUMP_PREFIX_OFFSET, 16, 4,
			ptr, len * 4, 0);
}

/*
 * Safely replace the WAIT of a waitlink with a new command and argument.
 * The GPU may be executing this WAIT while we're modifying it, so we have
 * to write it in a specific order to avoid the GPU branching to somewhere
 * else.  'wl_offset' is the offset to the first byte of the WAIT command.
 */
void etnaviv_buffer_replace_wait(struct etnaviv_cmdbuf *buffer,
	unsigned int wl_offset, u32 cmd, u32 arg)
{
	u32 *lw = buffer->vaddr + wl_offset;

	lw[1] = arg;
	mb();
	lw[0] = cmd;
	mb();
}

/*
 * Ensure that there is space in the command buffer to contiguously write
 * 'cmd_dwords' 64-bit words into the buffer, wrapping if necessary.
 */
u32 etnaviv_buffer_reserve(struct etnaviv_gpu *gpu,
	struct etnaviv_cmdbuf *buffer, unsigned int cmd_dwords)
{
	if (buffer->user_size + cmd_dwords * sizeof(u64) > buffer->size)
		buffer->user_size = 0;

	return etnaviv_cmdbuf_get_va(buffer,
				     &gpu->mmu_context->cmdbuf_mapping) +
	       buffer->user_size;
}

u16 etnaviv_buffer_init(struct etnaviv_gpu *gpu)
{
	struct etnaviv_cmdbuf *buffer = &gpu->buffer;

	lockdep_assert_held(&gpu->lock);

	/* initialize buffer */
	buffer->user_size = 0;

	CMD_WAIT(buffer, gpu->fe_waitcycles);
	CMD_LINK(buffer, 2,
		 etnaviv_cmdbuf_get_va(buffer, &gpu->mmu_context->cmdbuf_mapping)
		 + buffer->user_size - 4);

	return buffer->user_size / 8;
}

/*
 * Issue FLUSH_CACHE via FE for NPU devices. Called from npu_poll_work
 * after NN/TP engines go idle, to clear SH cache state before the next
 * operation. Overwrites the ring buffer and restarts FE.
 *
 * Returns the prefetch count for etnaviv_gpu_start_fe().
 */
u16 etnaviv_buffer_flush_cache(struct etnaviv_gpu *gpu)
{
	struct etnaviv_cmdbuf *buffer = &gpu->buffer;

	lockdep_assert_held(&gpu->lock);

	buffer->user_size = 0;

	CMD_LOAD_STATE(buffer, VIVS_GL_FLUSH_CACHE, 0x00000C23);
	CMD_LOAD_STATE(buffer, VIVS_GL_FLUSH_CACHE, 0x00000C23);
	/* Use WAIT+LINK instead of CMD_END so FE returns to idle state.
	 * CMD_END leaves FE bit 0 permanently busy on NPU-only chips,
	 * causing etnaviv_gpu_wait_idle() to always time out (~400ms
	 * overhead per inference). WAIT+LINK matches etnaviv_buffer_init()
	 * and allows FE to properly signal idle after FLUSH_CACHE completes.
	 */
	CMD_WAIT(buffer, gpu->fe_waitcycles);
	CMD_LINK(buffer, 2,
		 etnaviv_cmdbuf_get_va(buffer,
			&gpu->mmu_context->cmdbuf_mapping)
		 + buffer->user_size - 4);

	return buffer->user_size / 8;
}

u16 etnaviv_buffer_config_mmuv2(struct etnaviv_gpu *gpu, u32 mtlb_addr, u32 safe_addr)
{
	struct etnaviv_cmdbuf *buffer = &gpu->buffer;

	lockdep_assert_held(&gpu->lock);

	buffer->user_size = 0;

	if (gpu->identity.features & chipFeatures_PIPE_3D) {
		CMD_LOAD_STATE(buffer, VIVS_GL_PIPE_SELECT,
			       VIVS_GL_PIPE_SELECT_PIPE(ETNA_PIPE_3D));
		CMD_LOAD_STATE(buffer, VIVS_MMUv2_CONFIGURATION,
			mtlb_addr | VIVS_MMUv2_CONFIGURATION_MODE_MODE4_K);
		CMD_LOAD_STATE(buffer, VIVS_MMUv2_SAFE_ADDRESS, safe_addr);
		if (gpu->identity.nn_core_count == 0) {
			CMD_SEM(buffer, SYNC_RECIPIENT_FE, SYNC_RECIPIENT_PE);
			CMD_STALL(buffer, SYNC_RECIPIENT_FE, SYNC_RECIPIENT_PE);
		}
	}

	if (gpu->identity.features & chipFeatures_PIPE_2D) {
		CMD_LOAD_STATE(buffer, VIVS_GL_PIPE_SELECT,
			       VIVS_GL_PIPE_SELECT_PIPE(ETNA_PIPE_2D));
		CMD_LOAD_STATE(buffer, VIVS_MMUv2_CONFIGURATION,
			mtlb_addr | VIVS_MMUv2_CONFIGURATION_MODE_MODE4_K);
		CMD_LOAD_STATE(buffer, VIVS_MMUv2_SAFE_ADDRESS, safe_addr);
		if (gpu->identity.nn_core_count == 0) {
			CMD_SEM(buffer, SYNC_RECIPIENT_FE, SYNC_RECIPIENT_PE);
			CMD_STALL(buffer, SYNC_RECIPIENT_FE, SYNC_RECIPIENT_PE);
		}
	}

	CMD_END(buffer);

	buffer->user_size = ALIGN(buffer->user_size, 8);

	return buffer->user_size / 8;
}

u16 etnaviv_buffer_config_pta(struct etnaviv_gpu *gpu, unsigned short id)
{
	struct etnaviv_cmdbuf *buffer = &gpu->buffer;

	lockdep_assert_held(&gpu->lock);

	buffer->user_size = 0;

	CMD_LOAD_STATE(buffer, VIVS_MMUv2_PTA_CONFIG,
		       VIVS_MMUv2_PTA_CONFIG_INDEX(id));

	CMD_END(buffer);

	buffer->user_size = ALIGN(buffer->user_size, 8);

	return buffer->user_size / 8;
}

void etnaviv_buffer_end(struct etnaviv_gpu *gpu)
{
	struct etnaviv_cmdbuf *buffer = &gpu->buffer;
	unsigned int waitlink_offset = buffer->user_size - 16;
	u32 link_target, flush = 0;
	bool has_blt = !!(gpu->identity.minor_features5 &
			  chipMinorFeatures5_BLT_ENGINE);

	lockdep_assert_held(&gpu->lock);

	if (gpu->identity.nn_core_count > 0) {
		/* NPU-only: no cache flush or PE stall needed */
		flush = 0;
	} else if (gpu->exec_state == ETNA_PIPE_2D) {
		flush = VIVS_GL_FLUSH_CACHE_PE2D;
	} else if (gpu->exec_state == ETNA_PIPE_3D) {
		flush = VIVS_GL_FLUSH_CACHE_DEPTH |
			VIVS_GL_FLUSH_CACHE_COLOR |
			VIVS_GL_FLUSH_CACHE_TEXTURE |
			VIVS_GL_FLUSH_CACHE_TEXTUREVS |
			VIVS_GL_FLUSH_CACHE_SHADER_L2;
	}

	if (flush) {
		unsigned int dwords = 7;

		if (has_blt)
			dwords += 10;

		link_target = etnaviv_buffer_reserve(gpu, buffer, dwords);

		CMD_SEM(buffer, SYNC_RECIPIENT_FE, SYNC_RECIPIENT_PE);
		CMD_STALL(buffer, SYNC_RECIPIENT_FE, SYNC_RECIPIENT_PE);
		if (has_blt) {
			CMD_LOAD_STATE(buffer, VIVS_BLT_ENABLE, 0x1);
			CMD_SEM(buffer, SYNC_RECIPIENT_FE, SYNC_RECIPIENT_BLT);
			CMD_STALL(buffer, SYNC_RECIPIENT_FE, SYNC_RECIPIENT_BLT);
			CMD_LOAD_STATE(buffer, VIVS_BLT_ENABLE, 0x0);
		}
		CMD_LOAD_STATE(buffer, VIVS_GL_FLUSH_CACHE, flush);
		if (gpu->exec_state == ETNA_PIPE_3D) {
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
		CMD_END(buffer);

		etnaviv_buffer_replace_wait(buffer, waitlink_offset,
					    VIV_FE_LINK_HEADER_OP_LINK |
					    VIV_FE_LINK_HEADER_PREFETCH(dwords),
					    link_target);
	} else {
		/* Replace the last link-wait with an "END" command */
		etnaviv_buffer_replace_wait(buffer, waitlink_offset,
					    VIV_FE_END_HEADER_OP_END, 0);
	}
}

/* Append a 'sync point' to the ring buffer. */
void etnaviv_sync_point_queue(struct etnaviv_gpu *gpu, unsigned int event)
{
	struct etnaviv_cmdbuf *buffer = &gpu->buffer;
	unsigned int waitlink_offset = buffer->user_size - 16;
	u32 dwords, target;

	lockdep_assert_held(&gpu->lock);

	/*
	 * We need at most 3 dwords in the return target:
	 * 1 event + 1 end + 1 wait + 1 link.
	 */
	dwords = 4;
	target = etnaviv_buffer_reserve(gpu, buffer, dwords);

	/* Signal sync point event */
	CMD_LOAD_STATE(buffer, VIVS_GL_EVENT, VIVS_GL_EVENT_EVENT_ID(event) |
		       (gpu->identity.nn_core_count > 0 ?
			VIVS_GL_EVENT_FROM_FE : VIVS_GL_EVENT_FROM_PE));

	/* Stop the FE to 'pause' the GPU */
	CMD_END(buffer);

	/* Append waitlink */
	CMD_WAIT(buffer, gpu->fe_waitcycles);
	CMD_LINK(buffer, 2,
		 etnaviv_cmdbuf_get_va(buffer, &gpu->mmu_context->cmdbuf_mapping)
		 + buffer->user_size - 4);

	/*
	 * Kick off the 'sync point' command by replacing the previous
	 * WAIT with a link to the address in the ring buffer.
	 */
	etnaviv_buffer_replace_wait(buffer, waitlink_offset,
				    VIV_FE_LINK_HEADER_OP_LINK |
				    VIV_FE_LINK_HEADER_PREFETCH(dwords),
				    target);
}

/*
 * Append a command buffer to the ring buffer.
 *
 * For NPU silicon, dispatch to the per-chip entry point in
 * etnaviv_buffer_v8_qi.c (A311D 0x88) or etnaviv_buffer_v8_pico.c
 * (S905D3 0x99). The rationale: in-line `if (customer_id ==
 * 0xNN)` gates layered across R47/R51/R52 made same-chip
 * regressions hard to catch -- the per-chip files contain
 * straight-line logic with no cross-chip interleaving.
 *
 * GPU (non-NPU) silicon falls through to the body below; unknown
 * NPU customer_id also falls through with a warning so the existing
 * code path remains a working default.
 */
void etnaviv_buffer_queue(struct etnaviv_gpu *gpu, u32 exec_state,
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

	lockdep_assert_held(&gpu->lock);

	/* Dispatch to per-chip entry point for NPU silicon. */
	if (is_npu) {
		switch (gpu->identity.customer_id) {
		case 0x88: /* A311D, VIPNano-QI v8 */
			etnaviv_buffer_queue_v8_qi(gpu, exec_state,
				mmu_context, event, cmdbuf);
			return;
		case 0x99: /* S905D3, VIPPico v3 */
			etnaviv_buffer_queue_v8_pico(gpu, exec_state,
				mmu_context, event, cmdbuf);
			return;
		default:
			WARN_ONCE(1, "etnaviv: unknown NPU customer_id 0x%x; using GPU path\n",
				  gpu->identity.customer_id);
			break;
		}
	}

	if (drm_debug_enabled(DRM_UT_DRIVER))
		etnaviv_buffer_dump(gpu, buffer, 0, 0x50);

	link_target = etnaviv_cmdbuf_get_va(cmdbuf,
					    &gpu->mmu_context->cmdbuf_mapping);
	link_dwords = cmdbuf->size / 8;

	/* Gate behind etnaviv_dump_cmdbuf: per-invoke NN_INST/TP_INST dump to
	 * the slow serial console (console_loglevel=7 default) adds
	 * ~270 ms/invoke on mn001. See etnaviv_buffer_v8_qi.c for the full
	 * rationale (isolated 2026-06-12). */
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
	if (need_flush || switch_context) {
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

		/* PTA load command -- R47 (2026-05-06): skip on c64a 0x99
		 * NPU. The BSP never emits PTA_CONFIG via cmdbuf and uses
		 * a single shared MMU context (gckMMU model). Empirically
		 * c64a 0x99 silicon hangs FE at the FIRST instruction
		 * after PTA_CONFIG (R44 ring dump: word 58 stalls
		 * regardless of what content is there -- R45 changed value,
		 * R46 inserted LINK barrier; both still hang at word 58).
		 * Skipping PTA_CONFIG sidesteps the choke. MTLB reuses
		 * whatever PTA index was last active (id=0 from
		 * restore_sec); cross-process mappings may collide but
		 * for single-process repeated invocation this should
		 * progress past the hang.
		 */
		if (switch_mmu_context && gpu->sec_mode == ETNA_SEC_KERNEL &&
		    !(gpu->identity.customer_id == 0x99 &&
		      gpu->identity.nn_core_count > 0))
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
				    gpu->sec_mode == ETNA_SEC_KERNEL &&
				    !(gpu->identity.customer_id == 0x99 &&
				      gpu->identity.nn_core_count > 0)) {
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
				if (gpu->identity.nn_core_count > 0 &&
				    gpu->identity.customer_id == 0x99) {
					/*
					 * R51 (2026-05-06): on c64a 0x99 emit
					 * NOTHING after the MMU flush. The
					 * cumulative R47..R50 hang-walk
					 * pattern shows FE blocks at every
					 * post-flush LOAD_STATE on this
					 * silicon. Minimal kernel-ring prelude:
					 * MMU flush LOAD_STATE -> LINK to
					 * cmdbuf. Per-op state writes (mesa
					 * NN_CONFIG, NN_INST_ADDR) happen in
					 * the per-op cmdbuf at a different
					 * silicon path.
					 */
				} else if (gpu->identity.nn_core_count > 0) {
					/*
					 * NPU-only (customer 0x88 / A311D
					 * 7a31): no PE for SEM/STALL.
					 * Use WAIT to ensure MMU TLB flush
					 * completes before NN/TP accesses.
					 * Customer 0x99 is handled by the
					 * R51 minimal-prelude branch above.
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

		if (switch_context &&
		    !(gpu->identity.customer_id == 0x99 &&
		      gpu->identity.nn_core_count > 0)) {
			/* R51 (2026-05-06): skip etnaviv_cmd_select_pipe
			 * on c64a 0x99 NPU. The pipe_select emits
			 * GL_FLUSH_CACHE + GL_PIPE_SELECT LOAD_STATEs
			 * which hang FE post-flush on this silicon.
			 * NPU has no 2D/3D pipe to switch -- the
			 * pipe_select is dead code on NPU anyway.
			 */
			etnaviv_cmd_select_pipe(gpu, buffer, exec_state);
			gpu->exec_state = exec_state;
		} else if (switch_context) {
			/* Track exec_state without emitting commands. */
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
	 * the ring buffer.  return_target is the ring target address.
	 * We need at most 7 dwords in the return target: 2 cache flush +
	 * 2 semaphore stall + 1 event + 1 wait + 1 link.
	 *
	 * For all NPU chips: 3 dwords (event + wait + link).
	 * GL_EVENT triggers the ISR (immediate fence signal on A311D,
	 * deferred to npu_poll_work on S905D3). WAIT+LINK keeps FE
	 * alive in the idle loop after the event fires. Both paths
	 * need WAIT+LINK -- omitting it leaves stale ring buffer
	 * content that the FE executes after GL_EVENT, causing hangs.
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
	 * For NPU-only devices, skip the cache flush and PE semaphore/stall
	 * since there is no Pixel Engine. Use FROM_FE for the event instead
	 * of FROM_PE.
	 */
	if (is_npu) {
		/* NPU: use GL_EVENT FROM_FE for all NPU chips.
		 *
		 * VIPNano-QI (A311D, 0x88): SMALL_BATCH stalls FE at
		 * NN_INST_ADDR, so GL_EVENT fires after NN completes.
		 * ISR signals fence immediately.
		 *
		 * VIPPico v3 (S905D3, 0x99): SMALL_BATCH does NOT stall
		 * FE. GL_EVENT fires before NN completes. ISR defers to
		 * npu_poll_work which waits for NN/TP idle, then signals.
		 *
		 * Both paths use the same GL_EVENT command here. The
		 * difference is in irq_handler (immediate vs deferred).
		 */
		/* Only defer events for cmdbuf with NN/TP instructions.
		 * init_npu has no NN/TP ops and must complete normally.
		 *
		 * pico:N253 -- "must complete normally" is a device-STATE
		 * claim, not a cmdbuf-content one, and it only holds COLD.
		 * On warm 0x99 silicon a config-only cmdbuf takes no NPU
		 * interrupt at all and is ended by the drm_sched watchdog;
		 * see the same block in etnaviv_buffer_v8_pico.c, which
		 * routes that class's GL_EVENT FROM_FE.  This copy is
		 * UNREACHABLE for both NPU parts we ship -- customer_id 0x88
		 * and 0x99 both return from the dispatch switch above, so
		 * only an unknown-customer_id NPU (the WARN_ONCE default)
		 * lands here.  It is left behavioural-unchanged deliberately:
		 * no such part has been measured.  Note it already emits
		 * FROM_FE unconditionally, so it would not reproduce the
		 * warm hang in the same form.
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
			if (has_nn_tp)
				gpu->npu_poll_event = event;
		}
		CMD_LOAD_STATE(buffer, VIVS_GL_EVENT,
			       VIVS_GL_EVENT_EVENT_ID(event) |
			       VIVS_GL_EVENT_FROM_FE);
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

	/* WAIT+LINK is needed for both NPU and non-NPU paths.
	 * For non-NPU: normal GPU idle loop.
	 * For NPU: keeps FE alive after GL_EVENT fires (both A311D
	 * immediate signal and S905D3 deferred npu_poll_work need
	 * FE to stay in WAIT+LINK -- not executing stale ring buffer).
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
