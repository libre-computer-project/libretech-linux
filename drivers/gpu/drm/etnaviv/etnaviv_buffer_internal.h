/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Internal header shared by etnaviv_buffer.c and the per-chip files
 * etnaviv_buffer_v8_qi.c (A311D customer 0x88, VIPNano-QI v8) and
 * etnaviv_buffer_v8_pico.c (S905D3 customer 0x99, VIPPico v3).
 *
 * Phase A of the per-chip code-path split. Helpers shared by all three TUs:
 * - CMD_* / OUT inline command builders (header-defined; each TU gets
 *   its own copy via static inline)
 * - etnaviv_buffer_reserve / etnaviv_cmd_select_pipe /
 *   etnaviv_buffer_replace_wait / etnaviv_buffer_dump (definitions live
 *   in etnaviv_buffer.c, declarations exposed here for the chip files)
 *
 * Per-chip entry points:
 * - etnaviv_buffer_queue_v8_qi -- A311D NPU prelude/submit/postlude
 * - etnaviv_buffer_queue_v8_pico -- S905D3 NPU prelude/submit/postlude
 *
 * etnaviv_buffer_queue (in etnaviv_buffer.c) dispatches to one of the
 * above based on customer_id.
 */
#ifndef __ETNAVIV_BUFFER_INTERNAL_H__
#define __ETNAVIV_BUFFER_INTERNAL_H__

#include <linux/align.h>
#include <linux/bug.h>
#include <linux/types.h>

#include "cmdstream.xml.h"
#include "etnaviv_cmdbuf.h"
#include "state.xml.h"

struct etnaviv_gpu;
struct etnaviv_iommu_context;

/* --- Inline command builders. Each TU gets its own copy. --- */

static inline void OUT(struct etnaviv_cmdbuf *buffer, u32 data)
{
	u32 *vaddr = (u32 *)buffer->vaddr;

	BUG_ON(buffer->user_size >= buffer->size);

	vaddr[buffer->user_size / 4] = data;
	buffer->user_size += 4;
}

static inline void CMD_LOAD_STATE(struct etnaviv_cmdbuf *buffer,
	u32 reg, u32 value)
{
	u32 index = reg >> VIV_FE_LOAD_STATE_HEADER_OFFSET__SHR;

	buffer->user_size = ALIGN(buffer->user_size, 8);

	/* write a register via cmd stream */
	OUT(buffer, VIV_FE_LOAD_STATE_HEADER_OP_LOAD_STATE |
		    VIV_FE_LOAD_STATE_HEADER_COUNT(1) |
		    VIV_FE_LOAD_STATE_HEADER_OFFSET(index));
	OUT(buffer, value);
}

static inline void CMD_END(struct etnaviv_cmdbuf *buffer)
{
	buffer->user_size = ALIGN(buffer->user_size, 8);

	OUT(buffer, VIV_FE_END_HEADER_OP_END);
}

static inline void CMD_WAIT(struct etnaviv_cmdbuf *buffer,
			    unsigned int waitcycles)
{
	buffer->user_size = ALIGN(buffer->user_size, 8);

	OUT(buffer, VIV_FE_WAIT_HEADER_OP_WAIT | waitcycles);
}

static inline void CMD_LINK(struct etnaviv_cmdbuf *buffer,
	u16 prefetch, u32 address)
{
	buffer->user_size = ALIGN(buffer->user_size, 8);

	OUT(buffer, VIV_FE_LINK_HEADER_OP_LINK |
		    VIV_FE_LINK_HEADER_PREFETCH(prefetch));
	OUT(buffer, address);
}

static inline void CMD_STALL(struct etnaviv_cmdbuf *buffer,
	u32 from, u32 to)
{
	buffer->user_size = ALIGN(buffer->user_size, 8);

	OUT(buffer, VIV_FE_STALL_HEADER_OP_STALL);
	OUT(buffer, VIV_FE_STALL_TOKEN_FROM(from) | VIV_FE_STALL_TOKEN_TO(to));
}

static inline void CMD_SEM(struct etnaviv_cmdbuf *buffer, u32 from, u32 to)
{
	CMD_LOAD_STATE(buffer, VIVS_GL_SEMAPHORE_TOKEN,
		       VIVS_GL_SEMAPHORE_TOKEN_FROM(from) |
		       VIVS_GL_SEMAPHORE_TOKEN_TO(to));
}

/* --- Larger helpers (defined in etnaviv_buffer.c). --- */

void etnaviv_cmd_select_pipe(struct etnaviv_gpu *gpu,
	struct etnaviv_cmdbuf *buffer, u8 pipe);

void etnaviv_buffer_dump(struct etnaviv_gpu *gpu,
	struct etnaviv_cmdbuf *buf, u32 off, u32 len);

void etnaviv_buffer_replace_wait(struct etnaviv_cmdbuf *buffer,
	unsigned int wl_offset, u32 cmd, u32 arg);

u32 etnaviv_buffer_reserve(struct etnaviv_gpu *gpu,
	struct etnaviv_cmdbuf *buffer, unsigned int cmd_dwords);

/* --- Per-chip entry points. --- */

void etnaviv_buffer_queue_v8_qi(struct etnaviv_gpu *gpu, u32 exec_state,
	struct etnaviv_iommu_context *mmu_context, unsigned int event,
	struct etnaviv_cmdbuf *cmdbuf);

void etnaviv_buffer_queue_v8_pico(struct etnaviv_gpu *gpu, u32 exec_state,
	struct etnaviv_iommu_context *mmu_context, unsigned int event,
	struct etnaviv_cmdbuf *cmdbuf);

/* Resident-NBG per-batch drain (A311D 0x88); defined in etnaviv_buffer_v8_qi.c,
 * called from the submit path (etnaviv_gem_submit.c). See that file's block
 * comment. */
u32 etnaviv_npu_resident_plan(struct etnaviv_gpu *gpu, const void *stream,
	u32 stream_bytes, u32 *content_bytes);
u32 etnaviv_npu_resident_build(struct etnaviv_gpu *gpu, void *dst,
	const void *stream, u32 stream_bytes);

#endif /* __ETNAVIV_BUFFER_INTERNAL_H__ */
