/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * BSP-protocol HEVC firmware support -- declarations.
 *
 * Bucket 3 of the TASK-004 architecture plan (internal HEVC BSP plan).
 *
 * Functions in this header MUST only be called when sess->fw_match
 * has the AMVDEC_FW_QUIRK_HEVC_BSP_PROTO bit set.  Each implementation
 * asserts this with WARN_ON_ONCE on entry.
 */

#ifndef __CODEC_HEVC_BSP_H_
#define __CODEC_HEVC_BSP_H_

#include "vdec.h"
#include "codec_hevc_common.h"

/* RPM buffer layout: BSP firmware DMA-writes the parsed slice-header
 * parameter block (RPM = Reference Picture / Mvc Metadata) to a 256
 * byte buffer at HEVC_RPM_BUFFER (= HEVC_ASSIST_SCRATCH_1).
 *
 * Size derivation: BSP vh265.c defines RPM_BEGIN=0x100 and RPM_END=0x180,
 * so (RPM_END - RPM_BEGIN) * sizeof(u16) = 256 bytes.
 *
 * The buffer is allocated per HEVC session, kept alive for the entire
 * decode session, and freed at session teardown.
 */
#define HEVC_BSP_RPM_BUF_SIZE	256

/* AUX (SEI/HDR10+) buffer layout: BSP firmware DMA-writes parsed
 * prefix-SEI NAL data into a buffer pointed to by HEVC_AUX_ADR
 * (= HEVC_ASSIST_SCRATCH_L), with sizes encoded in HEVC_AUX_DATA_SIZE
 * (= HEVC_ASSIST_SCRATCH_M).  BSP allocates 16 KB prefix (no suffix
 * for the basic stateful path; multi-instance configurations also
 * allocate suffix), per BSP vh265.c hevc_local_init().
 *
 * Without these registers programmed, BSP firmware DMA-writes SEI
 * to physical address 0 -> AXI bus fault -> hard lockup, observed
 * Phase A.1 + Phase A.2 within < 1s of first decode session start.
 *
 * NOTE: HEVC_ASSIST_SCRATCH_M is HEVC_DECODE_COUNT for upstream
 * firmware.  Programming it as AUX_DATA_SIZE is BSP_PROTO-ONLY --
 * doing it under UPSTREAM_PROTO would corrupt the upstream
 * firmware's decode-count protocol.  Bucket discipline is
 * mandatory here.
 */
#define HEVC_BSP_AUX_PREFIX_SIZE	(16 * 1024)
#define HEVC_BSP_AUX_SUFFIX_SIZE	0
#define HEVC_BSP_AUX_BUF_SIZE		(HEVC_BSP_AUX_PREFIX_SIZE + HEVC_BSP_AUX_SUFFIX_SIZE)

/* Allocate a DMA-coherent RPM buffer for a BSP-protocol HEVC session.
 * On success, *rpm_vaddr is set to the kernel virtual address and
 * *rpm_paddr is set to the physical address.
 *
 * Returns 0 on success, negative errno on failure.
 */
int codec_hevc_bsp_alloc_rpm(struct amvdec_session *sess,
			     void **rpm_vaddr, dma_addr_t *rpm_paddr);

/* Free the RPM buffer.  Safe to call with NULL/0 (no-op). */
void codec_hevc_bsp_free_rpm(struct amvdec_session *sess,
			     void *rpm_vaddr, dma_addr_t rpm_paddr);

/* Write the RPM buffer physical address to HEVC_RPM_BUFFER so the
 * firmware can DMA-write into it.  Must be called BEFORE MPSR=1
 * (firmware start) so the address is valid for the firmware's
 * first slice handler.
 */
void codec_hevc_bsp_program_rpm(struct amvdec_session *sess,
				dma_addr_t rpm_paddr);

/* Allocate the BSP AUX (SEI/HDR10+) DMA buffer.  Same lifecycle as
 * the RPM buffer; allocated per session, freed at teardown.
 */
int codec_hevc_bsp_alloc_aux(struct amvdec_session *sess,
			     void **aux_vaddr, dma_addr_t *aux_paddr);

void codec_hevc_bsp_free_aux(struct amvdec_session *sess,
			     void *aux_vaddr, dma_addr_t aux_paddr);

/* Program HEVC_AUX_ADR (= HEVC_ASSIST_SCRATCH_L) and HEVC_AUX_DATA_SIZE
 * (= HEVC_ASSIST_SCRATCH_M) so BSP firmware can DMA prefix-SEI data
 * into the AUX buffer.  Must be called BEFORE MPSR=1.
 *
 * BSP_PROTO-ONLY: SCRATCH_M doubles as HEVC_DECODE_COUNT under
 * upstream firmware, so calling this from an upstream session would
 * corrupt the decode protocol.
 */
void codec_hevc_bsp_program_aux(struct amvdec_session *sess,
				dma_addr_t aux_paddr);

/* Program initial MPRED MV / ABV buffer addresses + codec-type tag.
 * BSP firmware boots, then the HEVC pipeline immediately AXI-writes
 * colocated MVs to HEVC_MPRED_MV_WR_START_ADDR -- which defaults to 0
 * if the driver doesn't pre-program it.  Upstream defers MPRED address
 * programming to per-slice setup (codec_hevc_set_mpred() called from
 * SLICE_SEGMENT_DONE ISR), but BSP firmware needs valid addresses
 * BEFORE MPSR=1 to survive the first picture.
 *
 * Caller passes the workspace base paddr + MV/ABV offset macros.
 * Per-slice handler can later overwrite these with per-frame slot
 * addresses; this is just the safe initial state.
 *
 * Also programs HEVC_ASSIST_SCRATCH_H = 0x01 (codec type tag for
 * 8-bit HEVC single-instance, per BSP firmware reset-vector trace).
 */
void codec_hevc_bsp_program_mpred_init(struct amvdec_session *sess,
				       dma_addr_t mv_paddr,
				       u32 mv_buf_size,
				       dma_addr_t abv_paddr);

/* ===========================================================
 * hevc_bsp SESSION HOOKS -- primary entry points for BSP_PROTO
 * ===========================================================
 *
 * These three hooks encapsulate all BSP-protocol-specific per-session
 * work.  Callers in codec_hevc.c invoke them under a SINGLE quirk gate
 * at session_start / setup_workspace / session_stop; the hooks
 * internally orchestrate the alloc/program/free of RPM, AUX, MPRED-
 * init, SCRATCH_H, and VDEC_1 power.
 *
 * Each hook asserts sess->fw_match->quirks & BSP_PROTO via
 * WARN_ON_ONCE on entry.  Calling them under UPSTREAM_PROTO is a
 * programming error.
 *
 * See task004-architecture-plan-20260420.md Bucket 3 section for
 * the full gap analysis motivating this split.
 */

/* session_start: power VDEC_1 + alloc RPM + alloc AUX.  Must be
 * called BEFORE codec_hevc_setup_workspace() and BEFORE MPSR=1.
 * Sets comm->bsp_rpm_vaddr/paddr and comm->bsp_aux_vaddr/paddr on
 * success.  On error, undoes partial allocations before returning.
 */
int codec_hevc_bsp_session_start(struct amvdec_session *sess,
				 struct codec_hevc_common *comm);

/* session_setup_workspace: program RPM / AUX / MPRED-init / SCRATCH_H
 * into HW registers.  Must be called AFTER session_start has
 * allocated the buffers, AFTER upstream's workspace-register
 * programming has laid down the shared fields, and BEFORE MPSR=1.
 *
 * Caller passes the pre-computed MPRED MV / ABV addresses +
 * MPRED_MV_BUF_SIZE for the first slot.  Per-slice handler in
 * codec_hevc.c will overwrite these with per-frame slot addrs on
 * SLICE_SEGMENT_DONE; this call establishes the safe initial state.
 */
void codec_hevc_bsp_session_setup_workspace(struct amvdec_session *sess,
					    struct codec_hevc_common *comm,
					    dma_addr_t mpred_mv_paddr,
					    u32 mpred_mv_buf_size,
					    dma_addr_t mpred_abv_paddr);

/* session_stop: free AUX + free RPM + release VDEC_1 power.  Safe to
 * call even if session_start was never invoked (NULL/0 fields + no-op
 * power_put is safe).  Clears comm->bsp_* fields after freeing.
 */
void codec_hevc_bsp_session_stop(struct amvdec_session *sess,
				 struct codec_hevc_common *comm);

#endif /* __CODEC_HEVC_BSP_H_ */
