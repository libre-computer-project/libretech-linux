// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024
 *
 * HEVC/H.265 decoder, using VDEC_HEVC engine.
 * Based on BSP vh265.c and upstream codec_vp9.c pattern.
 */

#include <linux/module.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "dos_regs.h"
#include "hevc_regs.h"
#include "codec_hevc.h"
#include "codec_hevc_bsp.h"
#include "codec_hevc_fw.h"
#include "vdec_helpers.h"
#include "vdec_debugfs.h"
#include "vdec_hevc.h"
#include "codec_hevc_common.h"
#include "esparser.h"		/* esparser_queue_eos (KI-061 EOS trailer inject) */

/*
 * meson-DRM FBC-output compress-header registry setter
 * (fbc-output-display-impl-plan.md S1/S2).  Defined + EXPORT_SYMBOL_GPL'd in
 * drivers/gpu/drm/meson/meson_overlay.c.  meson-DRM is built-in (=y) and
 * meson-vdec is a module (=m), so this =m driver may call into the =y registry
 * (the reverse would not link).  Declared extern here rather than including
 * the meson-DRM private header drm/meson/meson_overlay.h, which pulls in
 * meson_drv.h (DRM-private types) that have no business in a media driver.
 */
extern void meson_overlay_fbc_set_header(dma_addr_t body_base,
					 dma_addr_t header_paddr);

/*
 * KI-061 EOS trailer: cap on the stashed first-AU VCL slice copy.  A real
 * intra slice at the gate resolutions is a few KB (320x256 ~3.8 KB, 1080p
 * ~tens of KB); an I-slice larger than this cap is not stashed, so the
 * recovery simply does not engage (falls back to the N-1 KNOWN-CONTRACT-LIMIT
 * -- never a crash, never a truncated/unparseable trailer).
 */
#define HEVC_TRAILER_MAX	(512 * 1024)

/* HEVC register mapping (same HEVC_ASSIST_SCRATCH_* as VP9 but different use) */
#define HEVC_DEC_STATUS_REG	HEVC_ASSIST_SCRATCH_0
#define HEVC_RPM_BUFFER		HEVC_ASSIST_SCRATCH_1
#define HEVC_SHORT_TERM_RPS	HEVC_ASSIST_SCRATCH_2
#define HEVC_VPS_BUFFER		HEVC_ASSIST_SCRATCH_3
#define HEVC_SPS_BUFFER		HEVC_ASSIST_SCRATCH_4
#define HEVC_PPS_BUFFER		HEVC_ASSIST_SCRATCH_5
#define HEVC_SAO_UP		HEVC_ASSIST_SCRATCH_6
#define HEVC_STREAM_SWAP_BUFFER	HEVC_ASSIST_SCRATCH_7
#define HEVC_STREAM_SWAP_BUFFER2 HEVC_ASSIST_SCRATCH_8
#define HEVC_sCALELUT		HEVC_ASSIST_SCRATCH_D
#define HEVC_WAIT_FLAG		HEVC_ASSIST_SCRATCH_E
#define LMEM_DUMP_ADR		HEVC_ASSIST_SCRATCH_F
#define NAL_SEARCH_CTL		HEVC_ASSIST_SCRATCH_I
#define HEVC_DECODE_MODE	HEVC_ASSIST_SCRATCH_J
#define DECODE_STOP_POS		HEVC_ASSIST_SCRATCH_K
#define HEVC_DECODE_COUNT	HEVC_ASSIST_SCRATCH_M
#define HEVC_DECODE_SIZE	HEVC_ASSIST_SCRATCH_N

/* Decode status values from firmware */
#define HEVC_ACTION_DONE			0xff
#define HEVC_CODED_SLICE_SEGMENT_DAT		0x5
#define HEVC_SLICE_SEGMENT_DONE			0x8
#define HEVC_NAL_SEARCH_DONE			0x9  /* Firmware found next NAL header */
#define HEVC_DECPIC_DATA_DONE			0xa
#define HEVC_DECPIC_DATA_ERROR			0xb
#define HEVC_NAL_DECODE_DONE			0xe
#define HEVC_OVER_DECODE			0xf
#define HEVC_DECODE_BUFEMPTY			0x20
#define HEVC_SEARCH_BUFEMPTY			0x22
#define HEVC_DECODE_OVER_SIZE			0x23
#define HEVC_DECODE_BUFEMPTY2			0x24
#define HEVC_FIND_NEXT_PIC_NAL			0x50
#define HEVC_HEAD_PARSER_DONE			0xf0

/* Firmware interrupt request */
#define AMRISC_MAIN_REQ				0x04

/* Driver -> firmware responses */
#define HEVC_ACTION_DEC_CONT			0xfd

/* Decode mode */
#define DECODE_MODE_SINGLE			0

/* Write the MMU map address to the correct register.
 * G12A+ has a dedicated HEVC_ASSIST_MMU_MAP_ADDR register.
 * GXL uses HEVC_ASSIST_SCRATCH_7 (shared with stream swap buffer).
 * BSP: H265_MMU_MAP_BUFFER = HEVC_ASSIST_SCRATCH_7 for pre-G12A.
 */
static inline void codec_hevc_write_mmu_map_addr(struct amvdec_core *core,
						  dma_addr_t paddr)
{
	if (core->platform->revision >= VDEC_REVISION_G12A)
		amvdec_write_dos(core, HEVC_ASSIST_MMU_MAP_ADDR, paddr);
	else
		amvdec_write_dos(core, HEVC_ASSIST_SCRATCH_7, paddr);
}

/* Returns true if GXL HEVC needs legacy compressed reference mode
 * (without MMU).  Now that MMU is enabled on all platforms, this
 * should always return false -- MMU mode handles FBC internally.
 * Kept for reference; the code paths guarded by this are dead.
 */
static inline bool hevc_needs_compress_ref(struct amvdec_core *core,
					   int is_10bit)
{
	return false;
}

/* HEVC workspace layout - same structure as VP9 */
#define LCU_SIZE		64
#define MAX_REF_PIC_NUM		24

/*
 * MPRED motion-vector slot: one per decode slot, indexed by frame->index
 * (0..MAX_REF_PIC_NUM-1).  The hardware footprint of a slot is exactly
 * what codec_hevc_set_mpred() programs as the read window:
 *
 *   lcu_width * lcu_height * MV_MEM_UNIT(lcu_size)
 *
 * with MV_MEM_UNIT = 0x200 / 0x80 / 0x20 for a 64 / 32 / 16 LCU (BSP
 * vh265.c get_mv_buf() + config_mpred_hw()).  All three collapse to
 * W*H/8 bytes, so evaluating the product with the 64-LCU grid is an exact
 * upper bound over every legal LCU size -- see hevc_mv_buf_size() below.
 *
 * This used to be a fixed 0x120000, which is precisely the BSP's
 * MPRED_4K_MV_BUF_SIZE (4096x2304 = 64*36 LCUs * 0x200).  Two problems:
 *
 *   - Oversized for everything below 4K.  A 320x240 stream needs 0x10000;
 *     24 fixed slots put the workspace at SIZE_WORKSPACE 0x1D10000
 *     (29.1 MiB) of contiguous CMA for EVERY session at every resolution.
 *   - UNDERSIZED above 4096x2304.  One platform declares HEVC max
 *     8192x4320 (128*68 LCUs * 0x200 = 0x440000/slot), where the fixed
 *     slot overflows into the next slot and then into RPM/LMEM -- the
 *     same class of bug as TODO-VP9-011b.  BSP handles this with an
 *     explicit MPRED_8K_MV_BUF_SIZE tier; we handle it by computing.
 *
 * The slot size is therefore derived at start() from the negotiated
 * resolution (clamped to the platform max, floored at the 1080p tier) and
 * grown in resume() if the bitstream turns out larger than S_FMT
 * declared.  MAX_REF_PIC_NUM slots are always laid out because
 * frame->index indexes the array.
 *
 * The whole workspace is one dma_alloc_coherent() addressed by the
 * hardware as a physical base plus fixed offsets (the HEVC MMU translates
 * only the FBC frame bodies), so its size is a direct contiguous-CMA
 * demand.  Resulting totals (prefix MPRED_MV_OFFSET 0x208000 + 24 slots
 * + RPM/LMEM 0x900, 64K-aligned):
 *
 *   <=1920x1088: 0x40000/slot ->  6.0 MiB array, total 0x810000  ( 8.1 MiB)
 *   4096x2304:   0x120000/slot -> 27.0 MiB array, total 0x1D10000 (29.1 MiB)
 *   8192x4320:   0x440000/slot -> 102 MiB array,  total 0x6810000 (104 MiB)
 */
#define MPRED_MV_MEM_UNIT_LCU64	0x200
/*
 * Floor: the BSP's sub-4K tier (1920x1088 = 30*17 LCUs * 0x200, rounded
 * to 64K).  Costs 6.0 MiB of workspace instead of 1.5 MiB at 320x240 but
 * makes every stream up to 1080p correct regardless of what S_FMT
 * declared, without depending on the grow path firing.
 */
#define MPRED_MV_BUF_SIZE_MIN	0x40000

#define IPP_SIZE	0x4000
#define SAO_ABV_SIZE	0x30000
#define SAO_VB_SIZE	0x30000
#define SH_TM_RPS_SIZE	0x800
#define VPS_SIZE	0x800
#define SPS_SIZE	0x800
#define PPS_SIZE	0x2000
#define SAO_UP_SIZE	0x2800
#define SWAP_BUF_SIZE	0x800
#define SWAP_BUF2_SIZE	0x800
#define SCALELUT_SIZE	0x8000
#define DBLK_PARA_SIZE	0x80000
#define DBLK_DATA_SIZE	0x80000
#define DBLK_DATA2_SIZE	0x80000
#define MMU_VBH_SIZE	0x5000
#define MPRED_ABV_SIZE	0x10000
#define RPM_BUF_SIZE	0x100
#define LMEM_SIZE	0x800

#define IPP_OFFSET       0x00
#define SAO_ABV_OFFSET   (IPP_OFFSET + IPP_SIZE)
#define SAO_VB_OFFSET    (SAO_ABV_OFFSET + SAO_ABV_SIZE)
#define SH_TM_RPS_OFFSET (SAO_VB_OFFSET + SAO_VB_SIZE)
#define VPS_OFFSET       (SH_TM_RPS_OFFSET + SH_TM_RPS_SIZE)
#define SPS_OFFSET       (VPS_OFFSET + VPS_SIZE)
#define PPS_OFFSET       (SPS_OFFSET + SPS_SIZE)
#define SAO_UP_OFFSET    (PPS_OFFSET + PPS_SIZE)
#define SWAP_BUF_OFFSET  (SAO_UP_OFFSET + SAO_UP_SIZE)
#define SWAP_BUF2_OFFSET (SWAP_BUF_OFFSET + SWAP_BUF_SIZE)
#define SCALELUT_OFFSET  (SWAP_BUF2_OFFSET + SWAP_BUF2_SIZE)
#define DBLK_PARA_OFFSET (SCALELUT_OFFSET + SCALELUT_SIZE)
#define DBLK_DATA_OFFSET (DBLK_PARA_OFFSET + DBLK_PARA_SIZE)
#define DBLK_DATA2_OFFSET (DBLK_DATA_OFFSET + DBLK_DATA_SIZE)
#define MMU_VBH_OFFSET   (DBLK_DATA2_OFFSET + DBLK_DATA2_SIZE)
#define MPRED_ABV_OFFSET (MMU_VBH_OFFSET + MMU_VBH_SIZE)
#define MPRED_MV_OFFSET  (MPRED_ABV_OFFSET + MPRED_ABV_SIZE)

/*
 * Everything at or below MPRED_MV_OFFSET is resolution-independent and
 * stays a compile-time constant.  RPM and LMEM float above the MV slot
 * array, so they live in struct codec_hevc (rpm_offset / lmem_offset),
 * as does the total (workspace_size).  Laid out by
 * codec_hevc_layout_workspace().
 */

/* RPM parameter structure - from firmware LMEM, must match BSP layout */
#define RPM_BEGIN 0x100
#define RPM_END   0x180

/* MISC_FLAG0 bit definitions (from BSP vh265.c) */
#define PCM_LOOP_FILTER_DISABLED_FLAG_BIT		0
#define PCM_ENABLE_FLAG_BIT				1
#define LOOP_FILER_ACROSS_TILES_ENABLED_FLAG_BIT	2
#define PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED_FLAG_BIT	3
#define DEBLOCKING_FILTER_OVERRIDE_ENABLED_FLAG_BIT	4
#define PPS_DEBLOCKING_FILTER_DISABLED_FLAG_BIT		5
#define DEBLOCKING_FILTER_OVERRIDE_FLAG_BIT		6
#define SLICE_DEBLOCKING_FILTER_DISABLED_FLAG_BIT	7
#define SLICE_SAO_LUMA_FLAG_BIT				8
#define SLICE_SAO_CHROMA_FLAG_BIT			9
#define SLICE_LOOP_FILTER_ACROSS_SLICES_ENABLED_FLAG_BIT 10

union hevc_rpm_param {
	struct {
		u16 data[RPM_END - RPM_BEGIN];
	} l;
	struct {
		/* from ucode lmem, do not change this struct */
		u16 CUR_RPS[0x10];
		u16 num_ref_idx_l0_active;
		u16 num_ref_idx_l1_active;
		u16 slice_type;
		u16 slice_temporal_mvp_enable_flag;
		u16 dependent_slice_segment_flag;
		u16 slice_segment_address;
		u16 num_tile_rows_minus1;
		u16 pic_width_in_luma_samples;
		u16 pic_height_in_luma_samples;
		u16 log2_min_coding_block_size_minus3;
		u16 log2_diff_max_min_coding_block_size;
		u16 log2_max_pic_order_cnt_lsb_minus4;
		u16 POClsb;
		u16 collocated_from_l0_flag;
		u16 collocated_ref_idx;
		u16 log2_parallel_merge_level;
		u16 five_minus_max_num_merge_cand;
		u16 sps_num_reorder_pics_0;
		u16 modification_flag;
		u16 tiles_enabled_flag;
		u16 num_tile_columns_minus1;
		u16 num_tile_rows_minus1_2;
		u16 tile_width[12];
		u16 tile_height[8];
		u16 misc_flag0;
		u16 pps_beta_offset_div2;
		u16 pps_tc_offset_div2;
		u16 slice_beta_offset_div2;
		u16 slice_tc_offset_div2;
		u16 pps_cb_qp_offset;
		u16 pps_cr_qp_offset;
		u16 first_slice_segment_in_pic_flag;
		u16 m_temporalId;
		u16 m_nalUnitType;
		u16 vui_num_units_in_tick_hi;
		u16 vui_num_units_in_tick_lo;
		u16 vui_time_scale_hi;
		u16 vui_time_scale_lo;
		u16 bit_depth;
		u16 profile_etc;
		u16 sei_frame_field_info;
		u16 video_signal_type;
		u16 modification_list[0x20];
		u16 conformance_window_flag;
		u16 conf_win_left_offset;
		u16 conf_win_right_offset;
		u16 conf_win_top_offset;
		u16 conf_win_bottom_offset;
		u16 chroma_format_idc;
		u16 color_description;
		u16 aspect_ratio_idc;
		u16 sar_width;
		u16 sar_height;
		u16 sps_max_dec_pic_buffering_minus1_0;
	} p;
};

struct hevc_frame {
	struct list_head list;
	struct vb2_v4l2_buffer *vbuf;
	int index;
	int show;
	int done;
	int output_ready;	/* 1 = decode complete, eligible for POC-ordered output */
	int decoded;		/* KI-061 N14: this picture's DECPIC_DATA_DONE (0xa)
				 * fired -- its CTBs + SAO are fully written, so the
				 * pixels are real.  Used at EOS drain to tell a real
				 * but undelivered last picture (deliver) from a
				 * never-completed phantom such as the injected EOS
				 * trailer (drop). */
	unsigned int width;
	unsigned int height;
	int poc;
	/*
	 * Coded video sequence this picture belongs to (KI-175).  POC restarts
	 * at every IDR/BLA, so POC alone does not order pictures across such a
	 * boundary: display order is (cvs, poc), and a reference POC names a
	 * picture of the CURRENT sequence only.
	 */
	u32 cvs;

	/*
	 * Per-frame CAPTURE timestamp, claimed from sess->timestamps in
	 * DECODE order when the frame is allocated (codec_hevc_take_ts())
	 * and delivered with the frame in DISPLAY order at bump time via
	 * amvdec_dst_buf_done_ts().
	 *
	 * The FIFO amvdec_dst_buf_done() cannot be used here: it pops the
	 * oldest queued (decode-order) ts at OUTPUT time, but this codec
	 * emits in display order (Annex C DPB bumping), so the Nth display
	 * frame would receive the Nth decode ts -- a pure decode-order
	 * permutation whenever sps_max_num_reorder_pics > 0.  That breaks
	 * V4L2_BUF_FLAG_TIMESTAMP_COPY, which vdec.c advertises on both
	 * queues and which timestamp-consumers (VA-API shims) rely on to
	 * map a frame back to its surface.
	 */
	u64 ts;
	struct v4l2_timecode tc;
	u32 flags;
	bool ts_valid;
	bool cvs_start;		/* claimed with the ts: an IDR/BLA picture */
};

struct codec_hevc {
	struct mutex lock;
	struct codec_hevc_common common;

	void      *workspace_vaddr;
	dma_addr_t workspace_paddr;

	/*
	 * Resolution-derived workspace layout (see the MPRED_MV comment near
	 * the top of this file).  Everything below MPRED_MV_OFFSET is fixed;
	 * these four float with the per-slot MV size.
	 */
	u32 mpred_mv_buf_size;	/* per decode-slot MPRED MV bytes */
	u32 mpred_mv_total;	/* mpred_mv_buf_size * MAX_REF_PIC_NUM */
	u32 rpm_offset;
	u32 lmem_offset;
	u32 workspace_size;	/* total dma_alloc_coherent() size */

	/* BSP-protocol DMA buffers live in struct codec_hevc_common so
	 * codec_hevc_bsp.c can access them via the shared-header layout
	 * without exposing the full struct codec_hevc.  hevc_bsp path
	 * owns their lifecycle via codec_hevc_bsp_session_start/stop.
	 */

	union hevc_rpm_param rpm_param;

	/* Cached SPS num_reorder_pics for POC-ordered output bumping.
	 * HEVC B-frame streams are encoded in decode order but must be
	 * displayed in POC order. This is the DPB bumping threshold:
	 * when we have more than this many decode-complete frames buffered,
	 * release the lowest-POC one. 0 for IPPP streams (no reorder).
	 * See hevc-display-reorder-plan.md.
	 */
	u16 sps_num_reorder_pics_0;

	/* First-frame fast output (BSP vh265.c fast_output analog).  The
	 * first displayable frame of the session is delivered without the
	 * reorder-window hold so the c2/CCodec client's first-output wait
	 * does not time out.  Set once the first frame is delivered. */
	bool first_output_done;

	int bitdepth_offset;   /* u16 offset of bit_depth in RPM LMEM (firmware-dependent) */
	int is_10bit;

	u32 width, height;

	struct list_head ref_frames_list;
	u32 frames_num;

	u32 lcu_total;

	/* M2 shadow-copy: number of internal decode slots (= the dpb size
	 * requested at src_change).  get_new_frame picks a free slot in
	 * [0, dpb_size). */
	u32 dpb_size;

	struct hevc_frame *cur_frame;
	struct hevc_frame *prev_frame;

	/*
	 * KI-061 EOS last-frame recovery.  The HEVC firmware in SINGLE
	 * decode mode completes picture N only when the NEXT access unit's
	 * start code arrives to delimit N's last slice (the slice-decode
	 * spin at asm PC 0x07df-0x07e2 waits on PARSER_INT_STATUS bit0,
	 * which the HW sets only once the parser sees the next start code).
	 * At true EOS no next AU follows, so picture N stalls undecoded and
	 * userspace gets N-1 frames.  Empirically (viability audit
	 * 2026-07-18, g12a_hevc_mmu.bin 16384 sha256:f98249d9 UPSTREAM_PROTO)
	 * feeding ONE real, parseable next-picture VCL slice NAL at true EOS
	 * makes the firmware complete picture N with bit-exact real pixels;
	 * a bare start code or truncated header does NOT (the parser needs a
	 * genuine slice to recognise the picture boundary).
	 *
	 * We stash a copy of the first source AU's VCL slice bytes (a real
	 * intra slice the firmware provably parses, matching the active
	 * SPS/PPS; parameter sets are stripped so re-feeding it does not
	 * re-fire SEQ_READY / a spurious mid-drain SOURCE_CHANGE) and
	 * re-inject it once at true EOS (gated on sess->should_stop, so it
	 * never fires on the mid-stream SOURCE_CHANGE drain that reverted the
	 * KI-061 option-A inject in 8f481f2b2ea3).  The injected trailer
	 * itself then becomes the new stalled/undecoded cur_frame (a phantom
	 * with output_ready==0); the existing flush_output recycle path drops
	 * it (show=0 -> v4l2_m2m_buf_queue), keeping the contract N-in/N-out.
	 */
	u8  *trailer_data;	/* stashed first-AU VCL slice bytes, or NULL */
	u32  trailer_len;	/* length of trailer_data */
	bool trailer_injected;	/* one inject per session guard */
	/*
	 * KI-061 N15: set in the 0x22/0x24 SEARCH_BUFEMPTY hardirq.  On the
	 * ~big last-picture shim path the firmware finishes picture N's CTBs,
	 * then searches for the next NAL, hits buffer-low -> 0x22.  The hardirq
	 * must write HEVC_DECODE_SIZE=0 (KI-115 crash-safety) before acking
	 * 0xff, but DECODE_SIZE==0 gates OFF the firmware's DECPIC_DATA_DONE
	 * (0x0a) for picture N (fw ISR_INT1: 0x0a requires DECODE_SIZE!=0), so
	 * the trailer that lands next delimits picture N and SAO runs, but 0x0a
	 * never fires -> pics_decoded stuck at N-1 -> cur_frame is the real,
	 * fully-decoded picture N with output_ready==0 and decoded==0.  This
	 * flag (a 0x22-search happened => the firmware reached the end of
	 * picture N's data, so SAO ran) plus pics_decoded==pics_at_inject (no
	 * picture completed after the trailer was injected => cur_frame is
	 * picture N, not the trailer phantom) lets flush_output recover exactly
	 * that picture -- a distinction N14's decoded flag alone cannot make
	 * because 0x0a was suppressed.  Set unconditionally on 0x22/0x24 (the
	 * pics_at_inject gate, not should_stop-at-park timing, decides
	 * delivery, so this is race-free); reset per session in stop().
	 */
	bool eos_trailer_parked;
	/*
	 * KI-061 N15: pics_decoded captured when the EOS trailer is injected
	 * (resume_after_feed).  At inject the firmware is finishing picture N,
	 * so this is N-1.  If picture N later completes (0x20 pass path, or a
	 * normal large last picture, or KD), pics_decoded advances past this
	 * and flush_output must NOT re-deliver cur_frame (it is the trailer
	 * phantom -> avoids a phantom +1 frame).  If picture N never completes
	 * (0x22 park suppressed its 0x0a), pics_decoded stays == this and
	 * cur_frame is the real undelivered picture N -> deliver it.
	 */
	u32 pics_at_inject;

	int max_ra;

	int new_pic;	/* 1 = first slice of new picture, 0 = continuation */
	int slice_idx;	/* current slice index within picture */

	u32 dec_status; /* saved in hardirq for threaded handler */

	/* ISR rate limiting: count consecutive DONE interrupts with no
	 * SAO progress. If this exceeds a threshold, abort to prevent
	 * the system from becoming unresponsive.
	 */
	u32 done_count;
	u32 last_sao_y_wptr;

	/*
	 * Monotonic count of pictures the firmware has finished decoding
	 * (bumped in the threaded ISR each time a picture becomes
	 * output_ready).  flush_output's drain loop watches this for
	 * quiescence: when it stops advancing, the firmware has exhausted
	 * the frames still queued at EOS and the SAO/decode tail is drained.
	 * Read locklessly in the drain loop (READ_ONCE); written under
	 * hevc->lock in the ISR (WRITE_ONCE).
	 */
	u32 pics_decoded;

	/* POC tracking for MPRED temporal motion vector prediction */
	int curr_poc;
	int iPrevTid0POC;
	/* Current coded video sequence number; see hevc_frame.cvs (KI-175). */
	u32 cvs;

	/* Per-slice reference picture lists (built from CUR_RPS[]) */
	int ref_poc_list0[16];	/* L0 reference POCs */
	int ref_poc_list1[16];	/* L1 reference POCs */
	int ref_num_l0;		/* number of active L0 references */
	int ref_num_l1;		/* number of active L1 references */
	/* apply_rps intermediate state -- preserved for mpred_trace debugfs.
	 * st_curr0/st_curr1 reflect the RPS classification (past / future
	 * in POC order); num_neg/num_pos their counts.  Diagnostic only.
	 */
	int st_curr0_snap[16];
	int st_curr1_snap[16];
	int num_neg_snap;
	int num_pos_snap;

	unsigned long ref_held;	/* bitmap of capture buffers held as refs */

	/* Deferred ACTION_DONE: after outputting a frame, delay sending
	 * ACTION_DONE to the firmware by 1ms to give userspace a chance
	 * to recycle capture buffers.  Without this, the threaded ISR
	 * processes all frames back-to-back at RT priority, starving
	 * userspace and exhausting capture buffers.
	 */
	struct amvdec_session *sess;
	struct delayed_work action_done_work;
};

/* Find the V4L2 buffer index for a reference picture by POC.
 * Returns the buffer index, or -1 if not found.
 */
static int codec_hevc_find_buf_index(struct codec_hevc *hevc, int poc)
{
	struct hevc_frame *frame;

	/* A reference is always a picture of the current coded video
	 * sequence; an undelivered picture of the previous one can carry the
	 * same POC (KI-175).
	 */
	list_for_each_entry(frame, &hevc->ref_frames_list, list) {
		if (frame->poc == poc && frame->cvs == hevc->cvs)
			return frame->index;
	}
	return -1;
}

/* Display order across coded video sequences: (cvs, poc), KI-175. */
static bool hevc_frame_before(const struct hevc_frame *a,
			      const struct hevc_frame *b)
{
	if (a->cvs != b->cvs)
		return (s32)(a->cvs - b->cvs) < 0;
	return a->poc < b->poc;
}

/* IDR (19, 20) and BLA (16-18) start a new coded video sequence with
 * NoRaslOutputFlag = 1: every picture still waiting in the DPB is output
 * before any picture of the new sequence (HEVC C.5.2.2).  A mid-stream CRA
 * (21) does not.  The NAL type comes from the bitstream via the picture's
 * timestamp entry (esparser, codec_hevc_au_starts_cvs()): the RPM's
 * m_nalUnitType reads 30 on every picture with the upstream firmware.
 *
 * Does the picture about to be allocated (the next ts in decode order)
 * start a new coded video sequence?
 */
static bool codec_hevc_next_starts_cvs(struct amvdec_session *sess)
{
	struct amvdec_timestamp *ts;
	unsigned long flags;
	bool ret = false;

	spin_lock_irqsave(&sess->ts_spinlock, flags);
	ts = list_first_entry_or_null(&sess->timestamps,
				      struct amvdec_timestamp, list);
	if (ts)
		ret = ts->cvs_start;
	spin_unlock_irqrestore(&sess->ts_spinlock, flags);
	return ret;
}

/* Assign a newly allocated picture its POC and coded video sequence. */
static void codec_hevc_tag_frame(struct codec_hevc *hevc,
				 struct hevc_frame *frame)
{
	if (frame->cvs_start)
		hevc->cvs++;
	frame->poc = hevc->curr_poc;
	frame->cvs = hevc->cvs;
}

/* Decode CUR_RPS[], build L0/L1 reference lists, and mark ref_held.
 *
 * This combines the BSP's read_ref_pic_set() + set_ref_pic_list() logic.
 * Decodes CUR_RPS[] into StCurr0 (past refs) and StCurr1 (future refs),
 * then builds ordered L0/L1 lists per HEVC spec 8.3.4.
 *
 * Also sets ref_held bitmap to prevent get_new_frame() from recycling
 * capture buffers whose FBC body is still needed as a reference.
 *
 * Must be called after codec_hevc_fetch_rpm() and codec_hevc_compute_poc().
 */
static void codec_hevc_apply_rps(struct codec_hevc *hevc)
{
	union hevc_rpm_param *param = &hevc->rpm_param;
	int cur_poc = hevc->curr_poc;
	unsigned long new_held = 0;
	int st_curr0[16];  /* past reference POCs (negative delta) */
	int st_curr1[16];  /* future reference POCs (positive delta) */
	int num_neg = 0, num_pos = 0;
	int total_num;
	int num_l0_active, num_l1_active;
	u32 slice_type = param->p.slice_type;
	u32 mod_flag = param->p.modification_flag;
	int i;

	/* Step 1: Decode CUR_RPS[] into StCurr0 (past) and StCurr1 (future).
	 * Only entries with the "used by current" bit (bit 13) are included
	 * in the reference lists.  Entries without bit 13 are part of the RPS
	 * but not used as references for this picture.
	 */
	for (i = 0; i < 16; i++) {
		u16 rps = param->p.CUR_RPS[i];
		int delt, poc_tmp;

		if (rps == 0x8000)     /* RPS_END terminator */
			break;

		/* CUR_RPS format -- BSP vh265.c:898 non-LT layout
		 * (SUPPORT_LONG_TERM_RPS NOT defined).  The upstream firmware
		 * we ship emits this layout:
		 *   Bit 14: RPS_USED_BIT (used by current picture)
		 *   Bit 13: RPS_SIGN_BIT (1 = negative/past delta)
		 *   Bits 12..0: absolute delta POC (13-bit)
		 *
		 * Prior to 2026-04-19, the driver used the LT variant
		 * (USED=13, SIGN=12, LT=14) which caused future references to
		 * have bit 13 clear -> skipped as "not used" -> st_curr1
		 * stayed empty -> L1 filled from st_curr0 fallback, giving
		 * every B-slice the wrong collocated POC and ~20 dB PSNR
		 * drift on B-frame content.  See
		 * decoder/hevc/bframe-content-analysis.md "Strike 3".
		 */
		delt = rps & 0x1FFF;   /* bits 12..0: absolute delta POC */
		if (rps & BIT(13))     /* bit 13: sign (1 = past ref) */
			poc_tmp = cur_poc - (0x2000 - delt);
		else
			poc_tmp = cur_poc + delt;

		/* Bit 14: used_by_curr_pic_flag.  Only "used" refs go into
		 * the reference lists.  All RPS entries (used or not) protect
		 * their buffers via ref_held.
		 */
		if (rps & BIT(14)) {
			if (rps & BIT(13))
				st_curr0[num_neg++] = poc_tmp;
			else
				st_curr1[num_pos++] = poc_tmp;
		}

		/* Mark buffer as held regardless of used_by_curr flag */
		{
			int idx = codec_hevc_find_buf_index(hevc, poc_tmp);

			if (idx >= 0)
				set_bit(idx, &new_held);
		}
	}

	hevc->ref_held = new_held;

	/* Snapshot intermediate RPS classification for mpred_trace debugfs.
	 * Done before L0/L1 construction so the trace shows the firmware-
	 * reported past/future split independent of list-building bugs.
	 */
	memcpy(hevc->st_curr0_snap, st_curr0, sizeof(st_curr0));
	memcpy(hevc->st_curr1_snap, st_curr1, sizeof(st_curr1));
	hevc->num_neg_snap = num_neg;
	hevc->num_pos_snap = num_pos;

	/* Step 2: Build L0 and L1 reference picture lists (HEVC spec 8.3.4).
	 * I-slices have no references.  P-slices use L0 only.  B-slices use
	 * both L0 and L1.
	 */
	hevc->ref_num_l0 = 0;
	hevc->ref_num_l1 = 0;
	memset(hevc->ref_poc_list0, 0, sizeof(hevc->ref_poc_list0));
	memset(hevc->ref_poc_list1, 0, sizeof(hevc->ref_poc_list1));

	if (slice_type == 2)  /* I-slice */
		return;

	total_num = num_neg + num_pos;
	if (total_num == 0)
		return;

	num_l0_active = param->p.num_ref_idx_l0_active;
	num_l1_active = (slice_type == 0) ? param->p.num_ref_idx_l1_active : 0;

	if (num_l0_active > 16)
		num_l0_active = 16;
	if (num_l1_active > 16)
		num_l1_active = 16;

	/* L0: StCurr0 (past) first, then StCurr1 (future), cycling */
	if (mod_flag & 1) {
		/* Reference picture list modification for L0 */
		for (i = 0; i < num_l0_active; i++) {
			int mod_idx = param->p.modification_list[i];
			int combined_poc;

			/* modification_list index into {StCurr0, StCurr1} */
			if (mod_idx < num_neg)
				combined_poc = st_curr0[mod_idx];
			else if (mod_idx < total_num)
				combined_poc = st_curr1[mod_idx - num_neg];
			else
				combined_poc = st_curr0[0]; /* fallback */
			hevc->ref_poc_list0[i] = combined_poc;
		}
	} else {
		for (i = 0; i < num_l0_active; i++) {
			int c_idx = i % total_num;

			if (c_idx < num_neg)
				hevc->ref_poc_list0[i] = st_curr0[c_idx];
			else
				hevc->ref_poc_list0[i] = st_curr1[c_idx - num_neg];
		}
	}
	hevc->ref_num_l0 = num_l0_active;

	/* L1 (B-slice only): StCurr1 (future) first, then StCurr0 (past) */
	if (slice_type == 0 && num_l1_active > 0) {
		if (mod_flag & 2) {
			/* Reference picture list modification for L1 */
			int mod_off = (mod_flag & 1) ? num_l0_active : 0;

			for (i = 0; i < num_l1_active; i++) {
				int mod_idx = param->p.modification_list[mod_off + i];
				int combined_poc;

				/* For L1, combined = {StCurr1, StCurr0} */
				if (mod_idx < num_pos)
					combined_poc = st_curr1[mod_idx];
				else if (mod_idx < total_num)
					combined_poc = st_curr0[mod_idx - num_pos];
				else
					combined_poc = st_curr1[0]; /* fallback */
				hevc->ref_poc_list1[i] = combined_poc;
			}
		} else {
			for (i = 0; i < num_l1_active; i++) {
				int c_idx = i % total_num;

				if (c_idx < num_pos)
					hevc->ref_poc_list1[i] = st_curr1[c_idx];
				else
					hevc->ref_poc_list1[i] = st_curr0[c_idx - num_pos];
			}
		}
		hevc->ref_num_l1 = num_l1_active;
	}
}

static void codec_hevc_output_frame(struct amvdec_session *sess,
				    struct hevc_frame *frame);
static void codec_hevc_bump_frames(struct amvdec_session *sess, bool flush);
static bool codec_hevc_stage_output(struct amvdec_session *sess,
				    struct hevc_frame *frame);
static void codec_hevc_dst_buf_done(struct amvdec_session *sess,
				    struct hevc_frame *frame);

/*
 * KI-061 N15: SAO settle before recovering picture N.  The 0x22 EOS park
 * suppresses picture N's DECPIC_DATA_DONE (0x0a) -- the only "SAO fully
 * written" signal -- so the drain can deliver picture N while the firmware
 * is still on SAO's final write pass, giving a +-1 LSB (~48 dB) frame in a
 * fraction of runs.  Poll HEVC_SAO_Y_WPTR until it stops advancing (SAO
 * done) so the full frame is written before the M2 copy.
 */
#define HEVC_N15_SAO_SETTLE_LOOPS	2000u	/* ~100-200 ms ceiling */
#define HEVC_N15_SAO_SETTLE_STABLE	16	/* WPTR flat this many polls */

static void codec_hevc_flush_output(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_hevc *hevc = sess->priv;
	struct hevc_frame *tmp, *n;

	if (!hevc)	/* N16b: drain callback on a torn-down session */
		return;

	/*
	 * Only halt firmware during stream teardown (STREAMOFF), not during
	 * normal EOS drain.  Writing 0 to HEVC_DECODE_SIZE stops the firmware
	 * from processing queued frames.  During normal drain, we need the
	 * firmware to keep decoding so all buffered frames are delivered.
	 * During teardown, halt to prevent use-after-free via prev_frame.
	 */
	if (!sess->streamon_out)
		amvdec_write_dos(core, HEVC_DECODE_SIZE, 0);

	/*
	 * KI-061 / N13: keep the drain alive while the EOS trailer recovery is
	 * still in flight, and kick the proven feed path so it fires within the
	 * drain window -- do NOT feed the trailer HERE at drain time.
	 *
	 * The trailer that delimits + completes the real last picture N is fed
	 * by codec_hevc_resume_after_feed() off the esparser feed path while the
	 * firmware is still actively decoding: it appends the slice AND acks
	 * DEC_CONT (0x20 case) in the firmware's own BUFEMPTY data-wait
	 * handshake.  Feeding raw bytes at drain time instead misses that
	 * handshake (the audit's inferior drain-inject variant: recovers small
	 * pictures but not the 0x20/large ones), so we must let the esparser
	 * path do the feed, not do it here.
	 *
	 * Under gst-native that feed reliably precedes drain_work, so by the
	 * time flush_output polls, trailer_injected is already set and this is a
	 * no-op.  Under the libva-v4l2-m2m shim the two work items race the
	 * other way: DEC_CMD_STOP -> DRAIN_START is ~25 us, so drain_work
	 * wedge-exits at pics=N-1 and halts the firmware below ~8 ms BEFORE the
	 * trailer feed fires -- losing the recovered picture (N13: shim settled
	 * pics=N-1 why=wedge vs gst pics=N on the identical stream).
	 *
	 * Fix: (1) schedule esparser_queue_work so codec_hevc_resume_after_feed
	 * runs and feeds+acks the trailer during the drain window (the firmware
	 * is still decoding -- streamon_out stays 1 until the shim STREAMOFFs
	 * OUTPUT after V4L2_BUF_FLAG_LAST, ~15 s later), and (2) the poll loop
	 * below does NOT wedge-/idle-exit while the recovery is pending
	 * (trailer_data && !trailer_injected), so the firmware is not halted
	 * before the feed lands and completes picture N.
	 *
	 * Gated on should_stop AND streamon_out so it is inert on the mid-stream
	 * gst SOURCE_CHANGE drain (should_stop==0) and the STREAMOFF(OUT)
	 * teardown drain (streamon_out==0, DECODE_SIZE=0 above has halted decode
	 * and the DEC_CMD_STOP drain already recovered the trailer).
	 */
	if (!READ_ONCE(sess->aborted) &&
	    READ_ONCE(sess->should_stop) && sess->streamon_out &&
	    hevc->trailer_data && !hevc->trailer_injected)
		schedule_work(&sess->esparser_queue_work);	/* N16b: not once aborting */

	/*
	 * Wait for the SAO/deblock double-write pipeline to finish.
	 * The HEVC firmware decodes into FBC compressed buffers, and the
	 * SAO post-processor writes the final NV12/YUV420 pixels via DMA.
	 * If we deliver buffers before SAO completes, userspace gets
	 * partial/stale pixel data (KI-061: last frame ~41 dB PSNR).
	 *
	 * Poll HEVC_SAO_IF_STATUS (idle when all bits clear) and verify
	 * both Y and C write pointers have reached their target addresses.
	 * Same approach as codec_vp9_flush_output().  Timeout after 100ms.
	 */
	{
		int i, stable = 0, no_progress = 0;
		u32 sao_if = 0, pics, prev_pics;
		const char *why = "abs-ceiling";
		bool sao_idle;
		u32 recov_wait = 0;		/* KI-061/N13 keep-alive polls used */
		/*
		 * Per-picture no-progress budget, scaled to THIS session's
		 * picture area (~8x one picture's RE'd decode-poll count),
		 * clamped to [MIN,MAX].  Reset on every pics_decoded advance
		 * below, so it bounds the wait for ONE picture, never the whole
		 * drain.  Sized this way because after the last picture the SAO
		 * IF status settles at a busy residual (a resolution-scaled
		 * value, NOT the idle baseline), so the wedge guard -- not the
		 * idle-completion path -- ends the drain; a fixed budget large
		 * enough for 8K would then add ~1.2 s to every small-stream EOS.
		 * hevc->width/height are 0 before the first SOURCE_CHANGE, in
		 * which case clamp() pins the budget to MIN.
		 */
		u32 noprog_budget =
			clamp((u32)hevc->width * hevc->height /
				      HEVC_DRAIN_NOPROGRESS_DIV,
			      HEVC_DRAIN_NOPROGRESS_MIN,
			      HEVC_DRAIN_NOPROGRESS_MAX);

		pics = prev_pics = READ_ONCE(hevc->pics_decoded);

		/*
		 * Let the EOS decode tail fully drain (KI-131).
		 *
		 * At EOS the firmware is still several pictures behind the input
		 * feed; the drain must let it catch up, because cutting it short
		 * truncates the trailing frames.  The catch is that a single
		 * large picture takes a long time to decode (a 4K picture
		 * >60 ms, an 8K picture several times that), during which
		 * pics_decoded is flat -- so any bound expressed as a fixed
		 * wall-clock / loop count cuts the engine off mid-picture.  The
		 * bounds below are therefore PROGRESS-gated (reset on every
		 * pics_decoded advance), which makes the drain wait exactly as
		 * long as the engine keeps producing pictures, independent of
		 * resolution, and never re-breaks at the next resolution up.
		 *
		 * Two hardware signals are read each poll:
		 *   - HEVC_SAO_IF_STATUS: the SAO/decode engine is idle iff no
		 *     bit is set outside the RE'd baseline HEVC_SAO_IF_IDLE_VALUE.
		 *     The busy field stays set continuously through the backlog
		 *     and clears only on the last picture's SAO write -- the only
		 *     reliable between-picture keep-alive, since pics_decoded is
		 *     flat between frames.  While it reports BUSY the engine is
		 *     actively writing a picture, so we keep waiting no matter how
		 *     long that one picture takes (the KI-131 fix: the old code
		 *     counted these busy polls toward its stall cap and bailed
		 *     mid-4K-picture with SAO still busy at if=0x100000).
		 *   - pics_decoded: advances frame-by-frame as the firmware
		 *     drains its queue.
		 *
		 * Exit conditions:
		 *   1. Done / KI-061 idle-phantom: SAO idle AND pics_decoded flat
		 *      for HEVC_DRAIN_STABLE consecutive polls.  This is the fast
		 *      normal exit (a finished drain settles in a few polls) and
		 *      also escapes the KI-061 phantom cur_frame, where the engine
		 *      goes idle but pics_decoded never advances -- SAO-idle +
		 *      flat is indistinguishable from "done", and bailing is the
		 *      correct action for both.
		 *   2. No-progress guard ("noprogress"): pics_decoded flat while
		 *      the engine is NOT idle for noprog_budget polls.  On the
		 *      KI-061 EOS path this is the EXPECTED terminal state, not a
		 *      fault -- the firmware parks with SAO_IF at a busy residual,
		 *      so exit 1 never fires; see exit 2 in the loop below.
		 *      Progress-gated, so it bounds the per-picture wait, never
		 *      the whole drain.
		 *   3. Absolute ceiling: a defensive hard bound so the drain can
		 *      never hang unbounded even under pathological progress
		 *      oscillation.
		 */
		for (i = 0; i < HEVC_DRAIN_ABS_LOOPS; i++) {
			sao_if = amvdec_read_dos(core, HEVC_SAO_IF_STATUS);
			pics = READ_ONCE(hevc->pics_decoded);
			sao_idle = (sao_if & ~HEVC_SAO_IF_IDLE_VALUE) == 0;

			/*
			 * KI-061/N13: hold the drain OPEN while the EOS trailer
			 * recovery is still in flight -- the feed scheduled above
			 * has not yet injected the trailer + completed the real
			 * last picture N.  Neither exit 1 (idle+quiescent) nor
			 * exit 2 (wedge) may end the drain here, or the firmware
			 * would be halted below at pics=N-1 before the racing
			 * esparser feed (codec_hevc_resume_after_feed) lands -- the
			 * N13 shim failure.  Reset the stall counters (the drain is
			 * legitimately waiting, not stuck) and keep polling.  Clears
			 * the instant resume_after_feed sets trailer_injected (on
			 * another CPU), or after HEVC_DRAIN_RECOVERY_LOOPS so a
			 * stream that genuinely cannot recover still terminates
			 * (falls back to the pre-fix N-1, never hangs).
			 *
			 * Not on an ABORTED session.  The schedule_work() above is
			 * gated on !sess->aborted (N16b), so on the abort path the
			 * esparser feed that would set trailer_injected is never
			 * queued -- and amvdec_abort() has already halted the
			 * firmware (HEVC_MPSR=0, MBOX1_MASK=0), so nothing can ever
			 * inject the trailer.  Without this term the keep-alive
			 * therefore waits on a recovery that was deliberately
			 * suppressed and burns the FULL HEVC_DRAIN_RECOVERY_LOOPS
			 * budget every single time: measured recov=40000 with
			 * 40000+ total loops (~4.6-4.8 s) on every gated abort,
			 * versus recov=0-123 / ~2-3.3 k loops on a normal EOS drain.
			 * Mirror the scheduling gate so an aborted drain skips the
			 * keep-alive entirely and terminates on the ordinary exits.
			 */
			if (!READ_ONCE(sess->aborted) &&
			    READ_ONCE(sess->should_stop) && sess->streamon_out &&
			    hevc->trailer_data &&
			    !READ_ONCE(hevc->trailer_injected) &&
			    recov_wait < HEVC_DRAIN_RECOVERY_LOOPS) {
				recov_wait++;
				no_progress = 0;
				stable = 0;
				prev_pics = pics;
				usleep_range(50, 100);
				continue;
			}

			/*
			 * Real decode progress resets the wedge guard: total drain
			 * time is bounded by queue depth, not by any fixed cap.
			 */
			if (pics != prev_pics)
				no_progress = 0;
			else
				no_progress++;

			/* Exit 1: SAO idle + pics flat, debounced. */
			if (sao_idle && pics == prev_pics) {
				if (++stable >= HEVC_DRAIN_STABLE) {
					why = "idle+quiescent";
					break;
				}
			} else {
				stable = 0;
			}

			/*
			 * Exit 2: no-progress guard.  Only counts polls where the
			 * engine is NOT idle -- an idle engine is handled by exit 1
			 * above, never here, so this can never fire on the SAO-idle
			 * path.
			 *
			 * This is NOT an anomaly indicator, despite the "wedge"
			 * label it used to carry.  On the KI-061 EOS path the
			 * firmware ends parked on the injected trailer with
			 * HEVC_SAO_IF_STATUS at a resolution-scaled BUSY residual
			 * rather than HEVC_SAO_IF_IDLE_VALUE (see the noprog_budget
			 * comment above), so a completed, all-frames-delivered drain
			 * terminates HERE by design; exit 1 is reached only when the
			 * engine happens to settle back to the idle baseline (seen on
			 * long multi-slice streams, e.g. the KI-144 500-picture runs
			 * logging why=idle+quiescent).  A genuine wedge and a normal
			 * EOS park are indistinguishable from the two hardware
			 * signals available here, which is exactly why the exit is
			 * progress-gated and bounded rather than diagnostic.
			 *
			 * The printed loop count i is therefore NOT bounded by
			 * noprog_budget and routinely exceeds it: the trailer
			 * keep-alive polls (printed as recov) do not advance
			 * no_progress, and every pics_decoded advance resets it to 0.
			 * i = recov + (polls spent while pictures were still coming
			 * out) + noprog_budget.  A 320x240 30-picture EOS drain
			 * settling at ~2900 loops against budget=2000 recov=63 is the
			 * healthy shape, not an overrun.
			 */
			if (!sao_idle && no_progress >= noprog_budget) {
				why = "noprogress";
				break;
			}

			prev_pics = pics;
			usleep_range(50, 100);
		}
		dev_info(core->dev_dec,
			 "HEVC drain: settled in %d loops (if=0x%x pics=%u why=%s budget=%u recov=%u)\n",
			 i, sao_if, pics, why, noprog_budget, recov_wait);
	}

	/*
	 * KI-061 EOS last-frame recovery is handled in the FEED path
	 * (codec_hevc_resume_after_feed), not here: the stashed trailer slice
	 * is appended to the VIFIFO right after the last real frame, while the
	 * firmware is still actively decoding, so the production ISR delimits
	 * picture N through the normal 0x20/0x22 BUFEMPTY handling.  Injecting
	 * at drain time instead -- into the firmware's fragile post-stall
	 * state -- is both crash-prone and resolution-dependent (small pictures
	 * stall at 0x22 and recover, large pictures stall at 0x20 and do not),
	 * so the recovery is done during active decode where the firmware
	 * handles both uniformly.  By the time we reach here the recovered
	 * picture N is already in ref_frames_list and the trailer is the new
	 * stuck cur_frame (phantom, output_ready==0), dropped by the recycle
	 * path below -- contract stays N-in/N-out.
	 */

	/*
	 * KI-061 N15: about to recover picture N (0x22 EOS park suppressed its
	 * 0x0a)?  Wait for SAO to finish writing it before the M2 copy below,
	 * else the recovered frame is +-1 LSB short (~48 dB) in a fraction of
	 * runs.  Same gate as the delivery decision so this is inert on every
	 * other path.
	 */
	if (READ_ONCE(sess->should_stop) && hevc->trailer_injected &&
	    READ_ONCE(hevc->eos_trailer_parked) &&
	    READ_ONCE(hevc->pics_decoded) == hevc->pics_at_inject &&
	    hevc->cur_frame && !hevc->cur_frame->done &&
	    !hevc->cur_frame->output_ready) {
		u32 wptr, prev = ~0u;
		int settle = 0, w;

		for (w = 0; w < HEVC_N15_SAO_SETTLE_LOOPS; w++) {
			wptr = amvdec_read_dos(core, HEVC_SAO_Y_WPTR);
			if (wptr == prev) {
				if (++settle >= HEVC_N15_SAO_SETTLE_STABLE)
					break;
			} else {
				settle = 0;
			}
			prev = wptr;
			usleep_range(50, 100);
		}
		dev_info(core->dev_dec,
			 "HEVC N15 SAO settle: %d loops Y_WPTR=0x%x Y_LEN=0x%x C_WPTR=0x%x\n",
			 w, wptr,
			 amvdec_read_dos(core, HEVC_SAO_Y_LENGTH),
			 amvdec_read_dos(core, HEVC_SAO_C_WPTR));
	}

	mutex_lock(&hevc->lock);

	/*
	 * Handle the still-pending cur_frame.  During normal decoding,
	 * frame output is deferred: SLICE_SEGMENT_DONE marks the PREVIOUS
	 * frame output_ready when it sees slice_segment_address==0.
	 *
	 * KI-061: at EOS in single-decode mode the firmware often
	 * allocates a phantom cur_frame for would-be picture N+1 by
	 * parsing trailing bitstream data as a slice header, then stalls
	 * mid-decode (no real picture follows).  done remains 0 because
	 * NAL_DECODE_DONE never fires.  We must distinguish:
	 *
	 *   - cur_frame received SLICE_SEGMENT_DONE (decoded body, just
	 *     missing the closing NAL_DECODE_DONE): output_ready was
	 *     bumped already.  Emit normally below.
	 *   - cur_frame is a phantom (never decoded): output_ready==0.
	 *     Recycle the capture buffer back to the m2m pool so the
	 *     framework can use it as the LAST sentinel in
	 *     vdec_drain_work; do NOT emit it via dst_buf_done with
	 *     bytesused=0.  Setting show=0 routes the drain loop below
	 *     through v4l2_m2m_buf_queue() instead of dst_buf_done().
	 */
	if (hevc->cur_frame && !hevc->cur_frame->done) {
		if (!hevc->cur_frame->output_ready) {
			/*
			 * KI-061 N14: distinguish the REAL undelivered last
			 * picture from a phantom.  A tiny last picture (a
			 * few-hundred-byte P) fully decodes -- DECPIC_DATA_DONE
			 * fired, SAO ran, cur_frame->decoded is set -- but the
			 * firmware then parks at 0x22 SEARCH_BUFEMPTY before
			 * parsing the injected EOS trailer, so the trailer's
			 * new-picture SLICE_SEGMENT_DONE that would delimit and
			 * deliver this picture never fires.  cur_frame is that
			 * real, decoded picture: SHOW it (keep show=1).
			 *
			 * The phantoms this must still DROP have decoded==0: the
			 * injected trailer's own cur_frame (a larger last picture
			 * lets the firmware parse the trailer, which delivers the
			 * real picture N via its delimiter and leaves the trailer
			 * as a never-completed cur_frame), and any mid-decode
			 * frame at a non-EOS SOURCE_CHANGE drain.  Gate on
			 * should_stop so only the true-EOS drain can show a
			 * pending frame, never a SOURCE_CHANGE drain.
			 */
			/*
			 * KI-061 N15: on the ~big last-picture shim path the
			 * 0x22 EOS park suppressed picture N's 0x0a, so
			 * cur_frame->decoded is 0 even though picture N fully
			 * decoded (SAO ran) and is the real, undelivered last
			 * frame.  Recover it iff BOTH:
			 *   - eos_trailer_parked: a 0x22/0x24 search-park fired,
			 *     i.e. the firmware reached the end of picture N's
			 *     data and (after the trailer landed) SAO ran -- so
			 *     the pixels are real, not a mid-decode garbage buffer.
			 *   - pics_decoded == pics_at_inject: no picture completed
			 *     after the trailer was injected, so cur_frame is
			 *     picture N itself, NOT the trailer phantom.  On the
			 *     0x20 pass path / normal large last picture / KD,
			 *     picture N completes (pics advances past pics_at_inject)
			 *     and is delivered via its delimiter, so this is false
			 *     -> cur_frame (the phantom) is dropped as before and
			 *     no phantom +1 frame is emitted.
			 * decoded==1 still shows (N14 tiny-last-P).  Gated on
			 * should_stop so a SOURCE_CHANGE drain never shows it.
			 */
			if (!(READ_ONCE(sess->should_stop) &&
			      (hevc->cur_frame->decoded ||
			       (READ_ONCE(hevc->eos_trailer_parked) &&
				hevc->trailer_injected &&
				READ_ONCE(hevc->pics_decoded) ==
					hevc->pics_at_inject))))
				hevc->cur_frame->show = 0;
		}
		hevc->cur_frame->output_ready = 1;
		hevc->cur_frame = NULL;
	}

	/*
	 * Drain the reorder buffer in POC order.  Pick the lowest-POC
	 * not-yet-done frame and output it; repeat until empty.  This
	 * mirrors BSP vh265.c:output_pic_display() with flush_flag=1.
	 * The pre-existing list-iteration-order drain released B-frame
	 * streams in decode order, producing the same wrong output as
	 * the decode-order ISR path (KI-001).
	 */
	for (;;) {
		struct hevc_frame *best = NULL;

		list_for_each_entry(tmp, &hevc->ref_frames_list, list) {
			if (tmp->done)
				continue;
			if (!best || hevc_frame_before(tmp, best))
				best = tmp;
		}
		if (!best)
			break;
		if (best->show) {
			/* M2: stage the slot into a vb2 first.  If none free at
			 * drain the frame's pixels are dropped, but the drain
			 * still completes and the slot is freed below. */
			if (codec_hevc_stage_output(sess, best))
				codec_hevc_dst_buf_done(sess, best);
		} else if (best->vbuf) {
			v4l2_m2m_buf_queue(sess->m2m_ctx, best->vbuf);
		}
		best->done = 1;
		hevc->frames_num--;
	}

	/* Now release the list entries themselves. */
	list_for_each_entry_safe(tmp, n, &hevc->ref_frames_list, list) {
		list_del(&tmp->list);
		kfree(tmp);
	}
	hevc->cur_frame = NULL;
	hevc->prev_frame = NULL;
	hevc->ref_held = 0;
	mutex_unlock(&hevc->lock);

	/*
	 * The HEVC firmware in SINGLE decode mode has no bitstream-level
	 * EOS mechanism (unlike MPEG1/2 sequence_end_code or H.264 EOS
	 * NAL).  After the last frame, the firmware sits idle polling the
	 * VIFIFO for more data.  esparser_queued_bufs may still be
	 * positive because the firmware consumed the data (BUFEMPTY fired)
	 * but no NAL_DECODE_DONE decremented the counter.
	 *
	 * Force the counter to zero so the post-drain wait in
	 * vdec_drain_work() exits immediately instead of stalling for 5s
	 * (KI-033).
	 */
	if (atomic_read(&sess->esparser_queued_bufs) > 0) {
		dev_dbg(core->dev,
			"HEVC drain: forcing queued count from %d to 0\n",
			atomic_read(&sess->esparser_queued_bufs));
		atomic_set(&sess->esparser_queued_bufs, 0);
	}

	/*
	 * Halt firmware ONLY on the true EOS path -- when DEC_CMD_STOP
	 * (or the stop-teardown at vdec.c:619) has set sess->should_stop.
	 *
	 * Background: HEVC in single-decode mode has no bitstream-level
	 * EOS, so after the last frame the firmware sits in the
	 * phantom-cur_frame stall (allocated a new cur_frame from
	 * trailing bytes, spins on BUFEMPTY-style status codes, ISR ACKs
	 * each one).  vdec_wait_inactive() then waits a full 2 s because
	 * the 50 ms IRQ-quiescence window never opens.  Halting here --
	 * at the end of flush_output, after SAO settle and reorder drain
	 * have flushed every real frame -- stops the IRQ spam and lets
	 * vdec_wait_inactive() exit promptly, dropping end-to-end drain
	 * latency by ~2 s per session.
	 *
	 * MUST gate on sess->should_stop, NOT on sess->streamon_out:
	 * codec_ops->drain is also called from vdec_stop_streaming() on
	 * the CAPTURE queue (vdec.c:1341) every time gst-launch processes
	 * a SOURCE_CHANGE event -- mid-decode, with streamon_out still 1
	 * and should_stop still 0.  Halting firmware there corrupts state
	 * mid-stream and reboots the board on subsequent decode (the same
	 * regression that reverted the KI-061 option-A inject in commit
	 * 8f481f2b2ea3).
	 *
	 * Double-halt at drain_finalize is harmless: MPSR=0 is idempotent.
	 */
	if (sess->should_stop && !sess->firmware_halted) {
		amvdec_hevc_halt_firmware(core);
		sess->firmware_halted = 1;
	}
}

static u32 codec_hevc_num_pending_bufs(struct amvdec_session *sess)
{
	struct codec_hevc *hevc = sess->priv;

	if (!hevc)
		return 0;

	return hevc->frames_num;
}

/*
 * Bytes of MPRED MV storage the hardware touches for one decode slot at
 * WxH, as an exact upper bound over every legal LCU size.
 *
 * codec_hevc_set_mpred() programs the read window as
 * lcu_width * lcu_height * MV_MEM_UNIT, with MV_MEM_UNIT scaling with LCU
 * area (0x200 / 0x80 / 0x20 for 64 / 32 / 16).  For W,H that are
 * multiples of 64 all three LCU sizes give the identical product (W*H/8);
 * for other W,H the 64-LCU grid pads the most, so evaluating with LCU 64
 * bounds the 32- and 16-LCU cases too.  64K-aligned like BSP
 * get_mv_buf().
 */
static u32 hevc_mv_buf_size(u32 w, u32 h)
{
	u32 lcu_w = DIV_ROUND_UP(ALIGN(w, LCU_SIZE), LCU_SIZE);
	u32 lcu_h = DIV_ROUND_UP(ALIGN(h, LCU_SIZE), LCU_SIZE);

	return ALIGN(lcu_w * lcu_h * MPRED_MV_MEM_UNIT_LCU64, 64 * SZ_1K);
}

/* Per-slot MV size for a resolution: BSP formula, floored at the sub-4K
 * tier and clamped to what the platform can decode at all.
 */
static u32 codec_hevc_mv_size_for(struct amvdec_session *sess, u32 w, u32 h)
{
	u32 max_w = sess->fmt_out->max_width;
	u32 max_h = sess->fmt_out->max_height;

	/* sess->width/height default to 1280x720 at session init and are set
	 * by S_FMT before poweron, so they are always non-zero here; clamp
	 * defensively anyway.
	 */
	w = clamp(w ? w : max_w, 64u, max_w);
	h = clamp(h ? h : max_h, 64u, max_h);

	return max(hevc_mv_buf_size(w, h), (u32)MPRED_MV_BUF_SIZE_MIN);
}

/*
 * Lay the workspace out for a given per-slot MPRED MV size.  Everything
 * below MPRED_MV_OFFSET is fixed; RPM and LMEM float above the slot array.
 */
static void codec_hevc_layout_workspace(struct codec_hevc *hevc, u32 mv_buf_size)
{
	hevc->mpred_mv_buf_size = mv_buf_size;
	hevc->mpred_mv_total = mv_buf_size * MAX_REF_PIC_NUM;
	hevc->rpm_offset = MPRED_MV_OFFSET + hevc->mpred_mv_total;
	hevc->lmem_offset = hevc->rpm_offset + RPM_BUF_SIZE;
	hevc->workspace_size = ALIGN(hevc->lmem_offset + LMEM_SIZE,
				     64 * SZ_1K);
}

static int codec_hevc_alloc_workspace(struct amvdec_session *sess,
				      struct codec_hevc *hevc)
{
	struct amvdec_core *core = sess->core;

	codec_hevc_layout_workspace(hevc,
				    codec_hevc_mv_size_for(sess, sess->width,
							   sess->height));

	dev_info(core->dev,
		 "HEVC workspace: mpred_mv=%#x/slot (%ux%u coded, %ux%u max), total=%#x bytes\n",
		 hevc->mpred_mv_buf_size, sess->width, sess->height,
		 sess->fmt_out->max_width, sess->fmt_out->max_height,
		 hevc->workspace_size);

	hevc->workspace_vaddr = dma_alloc_coherent(core->dev,
						   hevc->workspace_size,
						   &hevc->workspace_paddr,
						   GFP_KERNEL);
	if (!hevc->workspace_vaddr) {
		dev_err(core->dev, "Failed to allocate HEVC Workspace\n");
		return -ENOMEM;
	}

	return 0;
}

static void codec_hevc_setup_workspace(struct amvdec_session *sess,
				       struct codec_hevc *hevc);

/*
 * Re-allocate the workspace when the bitstream's coded resolution needs
 * larger MPRED MV slots than the workspace was sized for -- S_FMT
 * under-declared the resolution, or the stream resized mid-decode.
 *
 * Called from resume(), the one point where a resolution change has
 * already been signalled to userspace and no decode is in flight.  MV
 * slot contents are per-reference prediction state, which a resolution
 * change invalidates anyway.  On the fast-resume path (resolution
 * unchanged, and always for anything up to the 1080p floor) this is a
 * no-op: it returns before touching the allocation.
 *
 * Growing moves workspace_paddr, so every base+offset register has to be
 * re-programmed -- hence the codec_hevc_setup_workspace() call, which is
 * a pure sequence of idempotent register writes.  It runs ONLY on the
 * grow path.
 */
static int codec_hevc_grow_workspace(struct amvdec_session *sess,
				     struct codec_hevc *hevc)
{
	struct amvdec_core *core = sess->core;
	u32 need = codec_hevc_mv_size_for(sess, hevc->width ? hevc->width :
						sess->width,
					  hevc->height ? hevc->height :
						sess->height);
	u32 old_mv = hevc->mpred_mv_buf_size;
	u32 old_size = hevc->workspace_size;
	dma_addr_t paddr;
	void *vaddr;

	if (need <= old_mv)
		return 0;

	codec_hevc_layout_workspace(hevc, need);

	vaddr = dma_alloc_coherent(core->dev, hevc->workspace_size, &paddr,
				   GFP_KERNEL);
	if (!vaddr) {
		dev_err(core->dev,
			"Failed to grow HEVC Workspace to %#x bytes for %ux%u\n",
			hevc->workspace_size, hevc->width, hevc->height);
		codec_hevc_layout_workspace(hevc, old_mv);
		return -ENOMEM;
	}

	dma_free_coherent(core->dev, old_size, hevc->workspace_vaddr,
			  hevc->workspace_paddr);
	hevc->workspace_vaddr = vaddr;
	hevc->workspace_paddr = paddr;

	dev_info(core->dev,
		 "HEVC workspace grown for %ux%u: mpred_mv=%#x/slot, total=%#x bytes\n",
		 hevc->width, hevc->height, hevc->mpred_mv_buf_size,
		 hevc->workspace_size);

	codec_hevc_setup_workspace(sess, hevc);

	return 0;
}

static void codec_hevc_setup_workspace(struct amvdec_session *sess,
				       struct codec_hevc *hevc)
{
	struct amvdec_core *core = sess->core;
	dma_addr_t wkaddr = hevc->workspace_paddr;

	amvdec_write_dos(core, HEVCD_IPP_LINEBUFF_BASE, wkaddr + IPP_OFFSET);
	amvdec_write_dos(core, HEVC_RPM_BUFFER, wkaddr + hevc->rpm_offset);
	amvdec_write_dos(core, HEVC_SHORT_TERM_RPS, wkaddr + SH_TM_RPS_OFFSET);
	amvdec_write_dos(core, HEVC_VPS_BUFFER, wkaddr + VPS_OFFSET);
	amvdec_write_dos(core, HEVC_SPS_BUFFER, wkaddr + SPS_OFFSET);
	amvdec_write_dos(core, HEVC_PPS_BUFFER, wkaddr + PPS_OFFSET);
	amvdec_write_dos(core, HEVC_SAO_UP, wkaddr + SAO_UP_OFFSET);

	/* SCRATCH_7 (HEVC_STREAM_SWAP_BUFFER):
	 * On pre-G12A with MMU firmware, SCRATCH_7 is repurposed as the
	 * MMU page table map address (H265_MMU_MAP_BUFFER in BSP vh265.c:113).
	 * The MMU firmware reads this at startup, so we must write the
	 * pre-allocated MMU map address here before firmware starts.
	 * On G12A+, a dedicated HEVC_ASSIST_MMU_MAP_ADDR register is used
	 * and SCRATCH_7 holds the swap buffer workspace address.
	 */
	if (codec_hevc_use_mmu(core->platform->revision, sess->pixfmt_cap,
			       hevc->is_10bit) &&
	    core->platform->revision < VDEC_REVISION_G12A)
		amvdec_write_dos(core, HEVC_STREAM_SWAP_BUFFER,
				 hevc->common.mmu_map_paddr);
	else
		amvdec_write_dos(core, HEVC_STREAM_SWAP_BUFFER,
				 wkaddr + SWAP_BUF_OFFSET);
	amvdec_write_dos(core, HEVC_STREAM_SWAP_BUFFER2,
			 wkaddr + SWAP_BUF2_OFFSET);
	amvdec_write_dos(core, HEVC_sCALELUT, wkaddr + SCALELUT_OFFSET);

	if (core->platform->revision >= VDEC_REVISION_G12A)
		amvdec_write_dos(core, HEVC_DBLK_CFGE,
				 wkaddr + DBLK_DATA2_OFFSET);

	amvdec_write_dos(core, HEVC_DBLK_CFG4, wkaddr + DBLK_PARA_OFFSET);
	amvdec_write_dos(core, HEVC_DBLK_CFG5, wkaddr + DBLK_DATA_OFFSET);
	amvdec_write_dos(core, LMEM_DUMP_ADR, wkaddr + hevc->lmem_offset);

	/* BSP-protocol: program HEVC_RPM_BUFFER (= HEVC_ASSIST_SCRATCH_1)
	 * with the per-session RPM DMA buffer addr.  Only fires under
	 * BSP_PROTO; upstream firmware doesn't read this register.  This
	 * MUST happen before MPSR=1 so the firmware's first slice handler
	 * has a valid DMA target.  See codec_hevc_bsp.c.
	 */
	/* Dispatch to hevc_bsp setup_workspace hook under BSP_PROTO.
	 * Programs RPM / AUX / MPRED-init / SCRATCH_H registers.
	 * All BSP-protocol workspace programming lives in
	 * codec_hevc_bsp.c; this is the only gate at the
	 * setup_workspace level.
	 */
	if (sess->fw_match &&
	    (sess->fw_match->quirks & AMVDEC_FW_QUIRK_HEVC_BSP_PROTO))
		codec_hevc_bsp_session_setup_workspace(sess, &hevc->common,
				wkaddr + MPRED_MV_OFFSET,
				hevc->mpred_mv_buf_size,
				wkaddr + MPRED_ABV_OFFSET);

	/* SAO buffer configuration for firmware (BSP config_title_hw) */
	amvdec_write_dos(core, HEVC_ASSIST_SCRATCH_9,
			 SAO_ABV_SIZE / LCU_SIZE);   /* sao_mem_unit */
	amvdec_write_dos(core, HEVC_ASSIST_SCRATCH_A,
			 wkaddr + SAO_ABV_OFFSET);   /* SAO ABV start */
	amvdec_write_dos(core, HEVC_ASSIST_SCRATCH_B,
			 SAO_VB_SIZE / LCU_SIZE);    /* sao_vb_size */
	amvdec_write_dos(core, HEVC_ASSIST_SCRATCH_C,
			 wkaddr + SAO_VB_OFFSET);    /* SAO VB start */

	if (codec_hevc_use_mmu(core->platform->revision, sess->pixfmt_cap,
			       hevc->is_10bit)) {
		amvdec_write_dos(core, HEVC_SAO_MMU_VH0_ADDR,
				 wkaddr + MMU_VBH_OFFSET);
		amvdec_write_dos(core, HEVC_SAO_MMU_VH1_ADDR,
				 wkaddr + MMU_VBH_OFFSET + (MMU_VBH_SIZE / 2));

		/* Write MMU map address.  The MMU map is pre-allocated
		 * in codec_hevc_start() so this address is always valid.
		 * On GXL this overwrites HEVC_ASSIST_SCRATCH_7 (stream
		 * swap buffer) -- the MMU firmware uses SCRATCH_7 for
		 * the MMU map, not the swap buffer.
		 */
		codec_hevc_write_mmu_map_addr(core,
					      hevc->common.mmu_map_paddr);
	}
}

static void codec_hevc_init_decoder(struct amvdec_core *core,
				    struct codec_hevc *hevc)
{
	/* Force clocks on for ALL HEVC submodules (BSP: CONFIG_HEVC_CLK_FORCED_ON).
	 * Without these, the SAO DMA output path is clock-gated off and
	 * produces zero pixel output.
	 */
	amvdec_write_dos_bits(core, HEVC_IQIT_CLK_RST_CTRL, BIT(2));
	amvdec_write_dos_bits(core, HEVC_DBLK_CFG0, BIT(2));
	amvdec_write_dos_bits(core, HEVC_SAO_CTRL1, BIT(2));
	amvdec_write_dos_bits(core, HEVC_MPRED_CTRL1, BIT(24));
	/* Parser/stream/CABAC clock force-on */
	amvdec_write_dos_bits(core, HEVC_STREAM_CONTROL, BIT(15));
	amvdec_write_dos_bits(core, HEVC_SHIFT_CONTROL, BIT(15));
	amvdec_write_dos_bits(core, HEVC_CABAC_CONTROL, BIT(13));
	amvdec_write_dos_bits(core, HEVC_PARSER_CORE_CONTROL, BIT(15));
	amvdec_write_dos_bits(core, HEVC_PARSER_INT_CONTROL, BIT(15));
	/* Parser IF control: force-on clocks for all parser sub-blocks
	 * BIT(5..0) = individual sub-block clock enables
	 */
	amvdec_write_dos_bits(core, HEVC_PARSER_IF_CONTROL,
			      BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0));
	/* IPP dynamic clock gating -- enable ALL gates */
	amvdec_write_dos(core, HEVCD_IPP_DYNCLKGATE_CONFIG, 0xffffffff);

	/* Initialize deblocking filter.
	 * DBLK_CFGB encodes double-write FIFO thresholds (SM1), pipeline
	 * config (G12A), or legacy pipeline mode (GXL/GXBB).
	 * BIT(0) = VP9 format flag -- NOT set here (HEVC mode).
	 */
	if (core->platform->revision >= VDEC_REVISION_SM1) {
		amvdec_write_dos(core, HEVC_DBLK_CFGB,
				 DBLK_CFGB_SM1_DW_FIFO_THRES);
		amvdec_write_dos(core, HEVC_DBLK_CFG3,
				 DBLK_CFG3_SM1_PIPELINE);
	} else if (core->platform->revision >= VDEC_REVISION_G12A) {
		amvdec_write_dos(core, HEVC_DBLK_CFGB,
				 DBLK_CFGB_G12A_INIT);
	} else {
		amvdec_write_dos(core, HEVC_DBLK_CFGB,
				 DBLK_CFGB_PRE_G12A_INIT);
	}
}

/* Deferred ACTION_DONE: signal firmware to process next NAL after a short
 * delay.  This breaks the tight threaded-ISR loop that otherwise processes
 * all frames at RT priority without yielding to userspace.
 */
static void codec_hevc_action_done_work(struct work_struct *work)
{
	struct codec_hevc *hevc = container_of(work, struct codec_hevc,
					       action_done_work.work);
	struct amvdec_session *sess = hevc->sess;
	struct amvdec_core *core = sess->core;

	/* M2: this fires after get_new_frame() deferred on internal-slot
	 * exhaustion.  A buffer-holding downstream consumer may have QBUF'd a
	 * capture buffer back since we deferred, so DRAIN OUTPUT here -- stage
	 * decode-complete frames into the now-free vb2s and recycle done +
	 * unreferenced frames -- to free internal decode slots BEFORE we re-ack
	 * the firmware to retry the deferred decode.  Without this, decode and
	 * output mutually stall once the output-lag headroom fills under an
	 * unpaced fast consumer (slots only freed on decode events, which are
	 * themselves stalled).  Draining on each retry tick paces the decode to
	 * the consumer's drain rate instead of deadlocking.  stop() cancels this
	 * work BEFORE taking hevc->lock, so taking it here is deadlock-safe. */
	mutex_lock(&hevc->lock);
	if (hevc->common.nv12_internal_vaddr[0]) {
		struct hevc_frame *tmp, *n;

		codec_hevc_bump_frames(sess, false);
		list_for_each_entry_safe(tmp, n, &hevc->ref_frames_list, list) {
			if (tmp->done &&
			    !test_bit(tmp->index, &hevc->ref_held)) {
				/* KI-143: never recycle the frame the decoder
				 * is still pointing at.  A multi-slice picture
				 * keeps cur_frame live across its continuation
				 * slices (which do NOT reallocate it), and
				 * prev_frame is still dereferenced by
				 * setup_decode() for the collocated-MV
				 * fallback.  Freeing either here leaves the
				 * threaded ISR reading a reused slab object,
				 * whose garbage ->index scales x8 into
				 * nv12_internal_vaddr[] and faults. */
				if (tmp == hevc->cur_frame ||
				    tmp == hevc->prev_frame)
					continue;
				list_del(&tmp->list);
				kfree(tmp);
			}
		}
	}
	mutex_unlock(&hevc->lock);

	amvdec_write_dos(core, HEVC_DEC_STATUS_REG, HEVC_ACTION_DONE);
	amvdec_write_dos(core, HEVC_MCPU_INTR_REQ, AMRISC_MAIN_REQ);
}

static int codec_hevc_start(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_hevc *hevc;
	u32 val;
	int i;
	int ret;

	hevc = kzalloc(sizeof(*hevc), GFP_KERNEL);
	if (!hevc)
		return -ENOMEM;

	/* KI-143 build marker (in-band proof a deployed kernel carries this) */
	dev_info(core->dev, "KI143-FIX: recycle-loop live-frame guard active\n");

	ret = codec_hevc_alloc_workspace(sess, hevc);
	if (ret)
		goto free_hevc;

	/* Pre-allocate MMU map before workspace setup.  On GXL, the MMU
	 * map address is written to HEVC_ASSIST_SCRATCH_7 during workspace
	 * setup, replacing the stream swap buffer.  The firmware reads
	 * this register during init, so a valid DMA address must be
	 * present before the firmware starts running.
	 */
	if (codec_hevc_use_mmu(core->platform->revision, sess->pixfmt_cap,
			       0 /* is_10bit not known yet, assume 8-bit */)) {
		/* Must match MMU_MAP_SIZE in codec_hevc_common.c -- this is
		 * the same buffer.  codec_hevc_alloc_mmu_headers() skips its
		 * own allocation when mmu_map_vaddr is already non-NULL, and
		 * codec_hevc_free_mmu_headers() frees with MMU_MAP_SIZE.
		 * Allocating with a smaller size (0x4800) caused KI-107: free
		 * walked extra pages outside the allocation, calling put_page
		 * on adjacent CMA pages and underflowing their refcounts.
		 */
		hevc->common.mmu_map_vaddr = dma_alloc_coherent(core->dev,
				MMU_MAP_SIZE, &hevc->common.mmu_map_paddr,
				GFP_KERNEL);
		if (!hevc->common.mmu_map_vaddr) {
			ret = -ENOMEM;
			goto free_workspace;
		}
	}

	/* Dispatch to hevc_bsp session start (Bucket 3) for BSP-PROTO
	 * firmware -- power VDEC_1 + alloc RPM + alloc AUX.  Upstream
	 * firmware sessions skip this entirely.  All BSP-protocol
	 * orchestration lives in codec_hevc_bsp.c; this is the only
	 * gate at the session_start level.
	 */
	if (sess->fw_match &&
	    (sess->fw_match->quirks & AMVDEC_FW_QUIRK_HEVC_BSP_PROTO)) {
		ret = codec_hevc_bsp_session_start(sess, &hevc->common);
		if (ret)
			goto free_workspace;
	}

	codec_hevc_setup_workspace(sess, hevc);

	/* Reset IQIT to reinit memory */
	amvdec_write_dos(core, DOS_SW_RESET3, BIT(14));
	amvdec_clear_dos_bits(core, HEVC_CABAC_CONTROL, BIT(0));
	amvdec_clear_dos_bits(core, HEVC_PARSER_CORE_CONTROL, BIT(0));

	val = amvdec_read_dos(core, HEVC_STREAM_CONTROL);
	val |= BIT(0); /* stream_fetch_enable */
	if (core->platform->revision >= VDEC_REVISION_G12A)
		val |= (0xf << 25); /* arwlen_axi_max: 16-beat AXI bursts (G12A+) */
	amvdec_write_dos(core, HEVC_STREAM_CONTROL, val);

	if (core->platform->revision >= VDEC_REVISION_G12A)
		amvdec_write_dos_bits(core, HEVC_STREAM_FIFO_CTL, BIT(29));

	/* PARSER_INT_CONTROL: configure parser interrupt sources.
	 * BSP: (3<<29) = parser_int_disable_cnt, (2<<26) = parser_int_src_sel,
	 * BIT(24) = startcode_int_en, BIT(22) = nal_search_int_en,
	 * BIT(15) = clk_forceon, BIT(7) = cmd_fifo_int_en,
	 * BIT(4) = cabac_int_en, BIT(0) = parser_int_enable.
	 */
	val = amvdec_read_dos(core, HEVC_PARSER_INT_CONTROL) & 0x03ffffff;
	val |= (3 << 29) | (2 << 26) | BIT(24) | BIT(22) |
	       BIT(15) | BIT(7) | BIT(4) | BIT(0);
	amvdec_write_dos(core, HEVC_PARSER_INT_CONTROL, val);
	amvdec_write_dos_bits(core, HEVC_SHIFT_STATUS, BIT(1) | BIT(0));
	/* SHIFT_CONTROL: BIT(15)=clk_forceon, (3<<6)=sft_valid_wr_position,
	 * (2<<4)=emulate_code_length_minus1, (2<<1)=start_code_length_minus1,
	 * BIT(0)=stream_shift_enable.
	 */
	amvdec_write_dos(core, HEVC_SHIFT_CONTROL,
			 BIT(15) | (3 << 6) | (2 << 4) | (2 << 1) | BIT(0));
	amvdec_write_dos(core, HEVC_CABAC_CONTROL, BIT(13) | BIT(0));
	/* CABAC: BIT(13)=clk_forceon, BIT(0)=cabac_enable */
	amvdec_write_dos(core, HEVC_PARSER_CORE_CONTROL, BIT(15) | BIT(0));
	/* PARSER_CORE: BIT(15)=clk_forceon, BIT(0)=parser_core_enable */
	amvdec_write_dos(core, HEVC_SHIFT_STARTCODE, 0x00000100);
	amvdec_write_dos(core, HEVC_SHIFT_EMULATECODE, 0x00000300);

	amvdec_write_dos(core, HEVC_DEC_STATUS_REG, 0);

	/* Initialize IQIT_SCALELUT memory */
	amvdec_write_dos(core, HEVC_IQIT_SCALELUT_WR_ADDR, 0);
	for (i = 0; i < 1024; i++)
		amvdec_write_dos(core, HEVC_IQIT_SCALELUT_DATA, 0);

	amvdec_write_dos(core, HEVC_DECODE_SIZE, 0);

	amvdec_write_dos(core, HEVC_PARSER_CMD_WRITE, BIT(16));
	for (i = 0; i < ARRAY_SIZE(vdec_hevc_parser_cmd); ++i)
		amvdec_write_dos(core, HEVC_PARSER_CMD_WRITE,
				 vdec_hevc_parser_cmd[i]);

	amvdec_write_dos(core, HEVC_PARSER_CMD_SKIP_0, PARSER_CMD_SKIP_CFG_0);
	amvdec_write_dos(core, HEVC_PARSER_CMD_SKIP_1, PARSER_CMD_SKIP_CFG_1);
	amvdec_write_dos(core, HEVC_PARSER_CMD_SKIP_2, PARSER_CMD_SKIP_CFG_2);
	amvdec_write_dos(core, HEVC_PARSER_IF_CONTROL,
			 BIT(5) | BIT(2) | BIT(0));

	amvdec_write_dos(core, HEVCD_IPP_TOP_CNTL, BIT(0));
	amvdec_write_dos(core, HEVCD_IPP_TOP_CNTL, BIT(1));

	amvdec_write_dos(core, HEVC_WAIT_FLAG, 1);

	/* Clear and enable mailbox interrupt */
	amvdec_write_dos(core, HEVC_ASSIST_MBOX1_CLR_REG, 1);
	amvdec_write_dos(core, HEVC_ASSIST_MBOX1_MASK, 1);
	/* Disable PSCALE */
	amvdec_write_dos(core, HEVC_PSCALE_CTRL, 0);
	/* Let firmware auto-search NAL units.
	 * 0xc = BIT(3)|BIT(2): auto_search_mode + nal_skip_check_enable (BSP).
	 */
	amvdec_write_dos(core, NAL_SEARCH_CTL, 0xc);

	amvdec_write_dos(core, DECODE_STOP_POS, 0);
	amvdec_write_dos(core, HEVC_DECODE_MODE, DECODE_MODE_SINGLE);

	codec_hevc_init_decoder(core, hevc);

	INIT_LIST_HEAD(&hevc->ref_frames_list);
	mutex_init(&hevc->lock);
	hevc->max_ra = INT_MAX;
	hevc->first_output_done = false;
	hevc->sess = sess;
	INIT_DELAYED_WORK(&hevc->action_done_work, codec_hevc_action_done_work);
	sess->priv = hevc;

	/* Both firmware versions populate bit_depth from SPS RPM 0x56,
	 * but at different LMEM offsets due to different struct layouts:
	 *   36864-byte firmware: LMEM 0x148 (struct offset 0x48)
	 *   16384-byte firmware: LMEM 0x144 (struct offset 0x44)
	 * The BSP firmware expanded tile_width[] from 8 to 12 entries,
	 * shifting all subsequent fields by 4 u16 slots.  The original
	 * upstream struct (matching 16KB firmware) has bit_depth at 0x44:
	 *   tile_width[8]+tile_height[8] then misc..pps_cr_qp_offset,
	 *   first_slice..vui_time_scale_lo, then bit_depth at 0x44.
	 * The BSP struct has tile_width[12], pushing bit_depth to 0x48.
	 */
	if (sess->fw_size == 36864 || sess->fw_size == 40960)
		hevc->bitdepth_offset = 0x48;  /* LMEM 0x148 -- BSP layout (36KB + 40KB swap) */
	else
		hevc->bitdepth_offset = 0x44;  /* LMEM 0x144 -- upstream 16KB layout.
						* The production 16K firmware DOES populate
						* bit_depth here -- a board probe on a Main10 decode
						* read data[0x44]=0x22 (=>10-bit) vs 0x00 (=>8-bit).
						* The previous "garbage toggles is_10bit -> infinite
						* src_change" was NOT a bad offset: bit_depth reads 0
						* on P/B frames (no SPS) and process_rpm updated
						* is_10bit every frame.  Fixed below by gating the
						* update on pic_width != 0 (SPS-carrying frames). */

	dev_info(core->dev, "HEVC: firmware %u bytes, bit_depth at RPM offset %#x\n",
		 sess->fw_size, hevc->bitdepth_offset);

	if (hevc->bitdepth_offset < 0)
		dev_warn(core->dev,
			 "HEVC: unknown firmware size %u. "
			 "bit_depth detection disabled -- 10-bit content "
			 "may decode incorrectly.\n", sess->fw_size);

	/* Do NOT inherit sess->width/height here. Starting with width=0
	 * ensures the first frame with valid RPM dimensions triggers
	 * amvdec_src_change() to set up capture buffers. Inheriting
	 * the session default (1280x720) would skip src_change when the
	 * stream matches, causing decode to stall (no capture buffers).
	 *
	 * For KI-023 (firmware restart mid-stream): P-frames with 0x0
	 * RPM dimensions are dropped by the !hevc->width guard in the
	 * ISR until the next I-frame with valid SPS arrives.
	 */

	dev_dbg(core->dev, "HEVC codec started, workspace=%pad size=%x\n",
		 &hevc->workspace_paddr, hevc->workspace_size);

	return 0;

free_workspace:
	/* Undo hevc_bsp session_start if it was done.  session_stop is
	 * a no-op for non-BSP_PROTO sessions.
	 */
	codec_hevc_bsp_session_stop(sess, &hevc->common);
	if (hevc->common.mmu_map_vaddr) {
		/* Must match the alloc size above (MMU_MAP_SIZE).  KI-107
		 * note above explains why size mismatch is a refcount bug.
		 */
		dma_free_coherent(core->dev, MMU_MAP_SIZE,
				  hevc->common.mmu_map_vaddr,
				  hevc->common.mmu_map_paddr);
	}
	dma_free_coherent(core->dev, hevc->workspace_size,
			  hevc->workspace_vaddr,
			  hevc->workspace_paddr);
free_hevc:
	kfree(hevc);
	return ret;
}

static int codec_hevc_stop(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_hevc *hevc = sess->priv;

	cancel_delayed_work_sync(&hevc->action_done_work);

	/*
	 * By the time we reach here, __vdec_hevc_stop() has already halted
	 * the firmware AND synchronously powered off the HEVC engine
	 * (DOS_SW_RESET3=0xffffffff toggle inside vdec_hevc_power_off).
	 * The IP block holds no DMA refs on our workspace / MMU buffers,
	 * so dma_free_coherent() below is safe.
	 */

	mutex_lock(&hevc->lock);
	if (hevc->workspace_vaddr) {
		dma_free_coherent(core->dev, hevc->workspace_size,
				  hevc->workspace_vaddr,
				  hevc->workspace_paddr);
		hevc->workspace_vaddr = NULL;
	}

	/* KI-061 EOS trailer copy (kmemdup'd in pre_queue). */
	kfree(hevc->trailer_data);
	hevc->trailer_data = NULL;
	hevc->trailer_len = 0;
	hevc->trailer_injected = false;
	hevc->eos_trailer_parked = false;	/* KI-061 N15 */
	hevc->pics_at_inject = 0;		/* KI-061 N15 */

	/* hevc_bsp session teardown: free AUX+RPM, release VDEC_1 power.
	 * No-op for upstream-firmware sessions.
	 */
	codec_hevc_bsp_session_stop(sess, &hevc->common);

	codec_hevc_free_fbc_buffers(sess, &hevc->common);
	codec_hevc_free_mmu_headers(sess, &hevc->common);
	mutex_unlock(&hevc->lock);

	return 0;
}

static void codec_hevc_fetch_rpm(struct amvdec_session *sess)
{
	struct codec_hevc *hevc = sess->priv;
	u16 *rpm_vaddr = hevc->workspace_vaddr + hevc->rpm_offset;
	int i, j;
	int count = RPM_END - RPM_BEGIN;

	for (i = 0; i < count; i += 4)
		for (j = 0; j < 4; j++)
			hevc->rpm_param.l.data[i + j] = rpm_vaddr[i + 3 - j];

	/* Debug: dump first 32 raw RPM words to see width/height */
	pr_debug("RPM raw[000]: %04x %04x %04x %04x %04x %04x %04x %04x "
		"%04x %04x %04x %04x %04x %04x %04x %04x\n",
		rpm_vaddr[0], rpm_vaddr[1], rpm_vaddr[2], rpm_vaddr[3],
		rpm_vaddr[4], rpm_vaddr[5], rpm_vaddr[6], rpm_vaddr[7],
		rpm_vaddr[8], rpm_vaddr[9], rpm_vaddr[10], rpm_vaddr[11],
		rpm_vaddr[12], rpm_vaddr[13], rpm_vaddr[14], rpm_vaddr[15]);
	pr_debug("RPM raw[016]: %04x %04x %04x %04x %04x %04x %04x %04x "
		"%04x %04x %04x %04x %04x %04x %04x %04x\n",
		rpm_vaddr[16], rpm_vaddr[17], rpm_vaddr[18], rpm_vaddr[19],
		rpm_vaddr[20], rpm_vaddr[21], rpm_vaddr[22], rpm_vaddr[23],
		rpm_vaddr[24], rpm_vaddr[25], rpm_vaddr[26], rpm_vaddr[27],
		rpm_vaddr[28], rpm_vaddr[29], rpm_vaddr[30], rpm_vaddr[31]);
	pr_debug("RPM parsed[16..31]: l.data = %04x %04x %04x %04x %04x %04x %04x %04x "
		"%04x %04x %04x %04x %04x %04x %04x %04x\n",
		hevc->rpm_param.l.data[16], hevc->rpm_param.l.data[17],
		hevc->rpm_param.l.data[18], hevc->rpm_param.l.data[19],
		hevc->rpm_param.l.data[20], hevc->rpm_param.l.data[21],
		hevc->rpm_param.l.data[22], hevc->rpm_param.l.data[23],
		hevc->rpm_param.l.data[24], hevc->rpm_param.l.data[25],
		hevc->rpm_param.l.data[26], hevc->rpm_param.l.data[27],
		hevc->rpm_param.l.data[28], hevc->rpm_param.l.data[29],
		hevc->rpm_param.l.data[30], hevc->rpm_param.l.data[31]);
}

static int codec_hevc_process_rpm(struct codec_hevc *hevc)
{
	union hevc_rpm_param *param = &hevc->rpm_param;
	u32 pic_width = param->p.pic_width_in_luma_samples;
	u32 pic_height = param->p.pic_height_in_luma_samples;
	int new_is_10bit = hevc->is_10bit;
	int reconfig = 0;

	/*
	 * bit_depth (data[bitdepth_offset], low nibble = bit_depth_luma_minus8) is
	 * valid ONLY on SPS-carrying frames; P/B frames report 0.  Update is_10bit
	 * ONLY when a valid SPS is present (pic_width != 0) -- the old code updated
	 * it every frame, so the per-frame 0 toggled is_10bit 10->8->10 and
	 * triggered infinite amvdec_src_change() (why detection had been disabled).
	 *
	 * Engage the 10-bit decode/FBC path ONLY for an FBC10 (AMSA) capture -- the
	 * only output format that can carry 10 bits.  For NV12/AMS8 (8-bit outputs)
	 * a 10-bit stream stays is_10bit=0 and the firmware SAO double-write
	 * downsamples to correct 8-bit (the long-proven ~58 dB path); this avoids
	 * the codec_hevc_use_downsample (10-bit -> 8-bit NV12) path entirely.
	 * A real mid-stream 8->10 change on an SPS frame triggers reconfig to
	 * reallocate the FBC body (3200 vs 4096 bytes/block) + SAO_CTRL5 BIT(9).
	 */
	if (hevc->bitdepth_offset >= 0 && pic_width) {
		u16 raw_bitdepth = param->l.data[hevc->bitdepth_offset];
		int stream_is_10bit = (((raw_bitdepth & 0xf) + 8) > 8) ? 1 : 0;

		new_is_10bit = (stream_is_10bit &&
				hevc->sess->pixfmt_cap ==
					V4L2_PIX_FMT_MESON_FBC10_SCATTER) ? 1 : 0;
		if (new_is_10bit != hevc->is_10bit && hevc->width)
			reconfig = 1;
		hevc->is_10bit = new_is_10bit;
	}

	/* P/B-frames don't carry SPS -- firmware reports 0x0 after a
	 * restart caused by V4L2 dynamic resolution change (STREAMOFF/
	 * STREAMON CAPTURE reloads firmware, losing SPS context).
	 * Keep existing dimensions; BSP also skips 0x0 (KI-023).
	 */
	if (!pic_width || !pic_height)
		return reconfig;

	if (pic_width != hevc->width || pic_height != hevc->height) {
		hevc->width = pic_width;
		hevc->height = pic_height;
		return 1;
	}

	return reconfig;
}

/* Compute Picture Order Count from LMEM parameters (BSP vh265.c logic).
 * Must be called after codec_hevc_fetch_rpm() and before setup_decode().
 */
static void codec_hevc_compute_poc(struct codec_hevc *hevc)
{
	union hevc_rpm_param *param = &hevc->rpm_param;
	u16 nal_type = param->p.m_nalUnitType;
	u16 temporal_id = param->p.m_temporalId;

	/* IDR pictures reset POC to 0 (BSP NAL_UNIT_CODED_SLICE_IDR = 19,
	 * NAL_UNIT_CODED_SLICE_IDR_N_LP = 20) */
	if (nal_type == 19 || nal_type == 20) {
		hevc->curr_poc = 0;
		if (temporal_id <= 1)
			hevc->iPrevTid0POC = 0;
	} else {
		int iMaxPOClsb = 1 << (param->p.log2_max_pic_order_cnt_lsb_minus4 + 4);
		int iPOClsb = param->p.POClsb;
		int iPrevPOClsb, iPrevPOCmsb, iPOCmsb;

		if (iMaxPOClsb == 0) {
			hevc->curr_poc = 0;
			return;
		}

		iPrevPOClsb = hevc->iPrevTid0POC % iMaxPOClsb;
		iPrevPOCmsb = hevc->iPrevTid0POC - iPrevPOClsb;

		if ((iPOClsb < iPrevPOClsb) &&
		    ((iPrevPOClsb - iPOClsb) >= (iMaxPOClsb / 2)))
			iPOCmsb = iPrevPOCmsb + iMaxPOClsb;
		else if ((iPOClsb > iPrevPOClsb) &&
			 ((iPOClsb - iPrevPOClsb) > (iMaxPOClsb / 2)))
			iPOCmsb = iPrevPOCmsb - iMaxPOClsb;
		else
			iPOCmsb = iPrevPOCmsb;

		/* BLA pictures reset POCmsb (nal_type 16, 17, 18) */
		if (nal_type >= 16 && nal_type <= 18)
			iPOCmsb = 0;

		hevc->curr_poc = iPOCmsb + iPOClsb;
		if (temporal_id <= 1)
			hevc->iPrevTid0POC = hevc->curr_poc;
	}
}

/*
 * Claim the oldest queued timestamp for a frame that is about to start
 * decoding.  esparser.c adds one ts per input buffer in decode order and
 * always before the firmware can parse that picture's slice header, so the
 * head of the list is exactly this picture's ts.  The frame carries it until
 * it is bumped for display.
 *
 * Called only on codec_hevc_get_new_frame()'s success paths: a failed
 * allocation is retried by the caller and must not consume a ts.
 *
 * Caller must hold hevc->lock.
 */
static void codec_hevc_take_ts(struct amvdec_session *sess,
			       struct hevc_frame *frame)
{
	struct amvdec_timestamp *tmp;
	unsigned long flags;

	spin_lock_irqsave(&sess->ts_spinlock, flags);
	if (list_empty(&sess->timestamps)) {
		spin_unlock_irqrestore(&sess->ts_spinlock, flags);
		/*
		 * No ts queued: a phantom picture parsed out of trailing
		 * bitstream bytes at EOS (KI-061).  Deliver ts=0 rather than
		 * erroring the buffer -- such a frame is normally show=0 and
		 * recycled without ever reaching dst_buf_done anyway.
		 */
		frame->ts_valid = false;
		dev_dbg(sess->core->dev,
			"HEVC: no ts queued for new frame (phantom at EOS?)\n");
		return;
	}

	tmp = list_first_entry(&sess->timestamps, struct amvdec_timestamp, list);
	frame->ts = tmp->ts;
	frame->tc = tmp->tc;
	frame->flags = tmp->flags;
	frame->cvs_start = tmp->cvs_start;
	frame->ts_valid = true;
	list_del(&tmp->list);
	spin_unlock_irqrestore(&sess->ts_spinlock, flags);
	kfree(tmp);
}

/*
 * Deliver a frame's CAPTURE buffer with the timestamp it claimed at decode
 * time.  Replaces amvdec_dst_buf_done() on every HEVC output path; that FIFO
 * variant would re-pop the (now display-order-mismatched) list head.  The
 * esparser_queued_bufs decrement + wake that amvdec_dst_buf_done() performs
 * are replicated here, since amvdec_dst_buf_done_ts() does not do them.
 */
static void codec_hevc_dst_buf_done(struct amvdec_session *sess,
				    struct hevc_frame *frame)
{
	amvdec_dst_buf_done_ts(sess, frame->vbuf, V4L2_FIELD_NONE,
			       frame->ts, frame->tc, frame->flags);
	atomic_dec(&sess->esparser_queued_bufs);
	wake_up(&sess->esparser_queue_done_wq);
}

static struct hevc_frame *
codec_hevc_get_new_frame(struct amvdec_session *sess)
{
	struct codec_hevc *hevc = sess->priv;
	struct v4l2_m2m_buffer *m2m_buf, *m2m_tmp;
	struct vb2_v4l2_buffer *vbuf = NULL;
	struct hevc_frame *new_frame;

	new_frame = kzalloc(sizeof(*new_frame), GFP_KERNEL);
	if (!new_frame)
		return NULL;

	if (hevc->common.nv12_internal_vaddr[0]) {
		/* M2 shadow-copy: allocate a free INTERNAL decode slot,
		 * decoupled from the userspace vb2 buffer.  Free = not held as
		 * a reference (ref_held) and not currently owned by a live
		 * frame.  The slot's nv12_internal[] buffer is the decode
		 * target (set_sao); a vb2 is grabbed only at output, where the
		 * slot is memcpy'd into it.  This keeps the decode pool from
		 * being starved by a buffer-holding downstream consumer. */
		struct hevc_frame *f;
		unsigned long used = hevc->ref_held;
		int slot = -1, i;

		list_for_each_entry(f, &hevc->ref_frames_list, list)
			used |= BIT(f->index);
		for (i = 0; i < hevc->dpb_size; i++) {
			if (!(used & BIT(i))) {
				slot = i;
				break;
			}
		}
		if (slot < 0) {
			dev_warn(sess->core->dev,
				 "HEVC: no free decode slot (dpb=%u)\n",
				 hevc->dpb_size);
			kfree(new_frame);
			return NULL;
		}
		new_frame->vbuf = NULL;
		new_frame->index = slot;
		new_frame->show = 1;
		new_frame->width = hevc->width;
		new_frame->height = hevc->height;
		codec_hevc_take_ts(sess, new_frame);
		list_add_tail(&new_frame->list, &hevc->ref_frames_list);
		hevc->frames_num++;
		dev_dbg(sess->core->dev, "HEVC new_frame(M2): slot=%d ts=%llu\n",
			slot, new_frame->ts);
		return new_frame;
	}

	v4l2_m2m_for_each_dst_buf_safe(sess->m2m_ctx, m2m_buf, m2m_tmp) {
		if (!test_bit(m2m_buf->vb.vb2_buf.index, &hevc->ref_held)) {
			vbuf = &m2m_buf->vb;
			v4l2_m2m_dst_buf_remove_by_buf(sess->m2m_ctx, vbuf);
			break;
		}
	}
	if (!vbuf) {
		dev_warn(sess->core->dev,
			 "HEVC: no free capture buffer (all ref_held)\n");
		kfree(new_frame);
		return NULL;
	}

	new_frame->vbuf = vbuf;
	new_frame->index = vbuf->vb2_buf.index;
	new_frame->show = 1;
	new_frame->width = hevc->width;
	new_frame->height = hevc->height;

	codec_hevc_take_ts(sess, new_frame);
	list_add_tail(&new_frame->list, &hevc->ref_frames_list);
	hevc->frames_num++;

	dev_dbg(sess->core->dev,
		"HEVC new_frame: idx=%d ts=%llu Y=0x%llx UV=0x%llx\n",
		new_frame->index, new_frame->ts,
		(u64)vb2_dma_contig_plane_dma_addr(&vbuf->vb2_buf, 0),
		vbuf->vb2_buf.num_planes > 1 ?
		(u64)vb2_dma_contig_plane_dma_addr(&vbuf->vb2_buf, 1) : 0ULL);

	return new_frame;
}

/*
 * Bytes the CPU may write through vb2_plane_vaddr(vb, plane).  For an
 * imported dmabuf that is the dmabuf's own size -- planes[].length is only
 * what the client declared (v4l2_codec2 declares the per-plane sizeimage while
 * backing both planes with one gralloc buffer).  Otherwise it is the plane.
 */
static size_t hevc_dst_capacity(struct vb2_buffer *vb, unsigned int plane)
{
	if (vb->memory == VB2_MEMORY_DMABUF && vb->planes[plane].dbuf)
		return vb->planes[plane].dbuf->size;
	return vb2_plane_size(vb, plane);
}

static void hevc_warn_short(struct amvdec_session *sess,
			    struct vb2_v4l2_buffer *vbuf, size_t cap,
			    size_t need)
{
	dev_warn_ratelimited(sess->core->dev_dec,
			     "HEVC stage: CAPTURE buf %u holds %zu B, frame %ux%u needs %zu B -- copy truncated\n",
			     vbuf->vb2_buf.index, cap, sess->width,
			     sess->height, need);
}

/*
 * M2 shadow-copy: stage a decoded frame's NV12 into a userspace vb2 buffer.
 * Legacy (non-M2) frames already hold their vb2 (frame->vbuf set) -- nothing to
 * do.  For M2 (frame->vbuf == NULL) grab a free vb2 and memcpy the slot's
 * nv12_internal[] into it.  Returns false (defer) when no vb2 is free: the
 * decoder keeps running on internal slots and codec_hevc_bump_frames retries on
 * the next picture, so a buffer-holding consumer delays OUTPUT without stalling
 * DECODE (the KI-061/KI-115 capture-exhaustion deadlock).
 *
 * Caller must hold hevc->lock.
 */
static bool codec_hevc_stage_output(struct amvdec_session *sess,
				    struct hevc_frame *frame)
{
	struct codec_hevc *hevc = sess->priv;
	struct vb2_v4l2_buffer *vbuf;
	void *src;
	u32 ysz;

	if (frame->vbuf)
		return true;
	if (!hevc->common.nv12_internal_vaddr[frame->index])
		return false;

	vbuf = v4l2_m2m_dst_buf_remove(sess->m2m_ctx);
	if (!vbuf)
		return false;

	src = hevc->common.nv12_internal_vaddr[frame->index];
	ysz = amvdec_get_output_size(sess);
	if (vbuf->vb2_buf.num_planes > 1) {
		void *y = vb2_plane_vaddr(&vbuf->vb2_buf, 0);
		void *uv = vb2_plane_vaddr(&vbuf->vb2_buf, 1);
		bool aliased = y && uv &&
			vb2_dma_contig_plane_dma_addr(&vbuf->vb2_buf, 1) ==
			vb2_dma_contig_plane_dma_addr(&vbuf->vb2_buf, 0);

		/*
		 * Single-buffer NV12M import (Android CCodec / v4l2_codec2):
		 * the gralloc output pool allocates ONE contiguous NV12 dmabuf
		 * and imports it for BOTH MPLANE planes with data_offset=0, so
		 * plane 1 (U/V) resolves to the SAME dma address as plane 0
		 * (Y) -- confirmed here via the same DMA-address comparison as
		 * the H.264 aliasing guard in vdec_helpers.c set_canvas_nv12m().
		 * Copying chroma to vb2_plane_vaddr(plane 1) in that case
		 * overwrites the luma bytes just written by the memcpy above
		 * (symptom: HEVC through Android's codec layer delivers every
		 * frame but scores ~9.2 dB).
		 *
		 * When aliased, the destination is ONE packed NV12 image in
		 * the client's layout: luma at the CODED size G_FMT reports
		 * (ALIGN(width, 64) x ALIGN(height, 32), KI-170), chroma
		 * straight after it -- where the client allocated it
		 * (minigbm: stride x allocation height) and where
		 * set_canvas_nv12m() puts it for H.264.  Chroma at the
		 * visible height lands 16 rows early at 720p (N26: every
		 * ByteBuffer frame ~21 dB).  It is NOT the internal slot
		 * layout (luma padded to ysz, rounded to 64 KiB): chroma at
		 * y + ysz writes past the end of the client's buffer
		 * (kce17: a write fault one page past it).  Genuine
		 * 2-plane imports (MMAP, or a HAL that allocates separate
		 * planes / sets a proper data_offset) have distinct dma
		 * addresses and keep the plane-sized copies below.
		 */
		if (aliased) {
			size_t lsz = (size_t)ALIGN(sess->width, 64) *
				     ALIGN(sess->height, 32);
			size_t cap = hevc_dst_capacity(&vbuf->vb2_buf, 0);

			memcpy(y, src, min(lsz, cap));
			if (cap > lsz)
				memcpy(y + lsz, src + ysz,
				       min(lsz / 2, cap - lsz));
			if (cap < lsz + lsz / 2)
				hevc_warn_short(sess, vbuf, cap, lsz + lsz / 2);
		} else {
			size_t cap = hevc_dst_capacity(&vbuf->vb2_buf, 0);

			if (y)
				memcpy(y, src, min((size_t)ysz, cap));
			if (y && cap < ysz)
				hevc_warn_short(sess, vbuf, cap, ysz);
			cap = hevc_dst_capacity(&vbuf->vb2_buf, 1);
			if (uv)
				memcpy(uv, src + ysz, min((size_t)ysz / 2, cap));
			if (uv && cap < ysz / 2)
				hevc_warn_short(sess, vbuf, cap, ysz / 2);
		}
	} else {
		void *y = vb2_plane_vaddr(&vbuf->vb2_buf, 0);
		size_t cap = hevc_dst_capacity(&vbuf->vb2_buf, 0);

		if (y)
			memcpy(y, src, min((size_t)ysz + ysz / 2, cap));
		if (y && cap < ysz + ysz / 2)
			hevc_warn_short(sess, vbuf, cap, ysz + ysz / 2);
	}
	frame->vbuf = vbuf;
	return true;
}

static void codec_hevc_output_frame(struct amvdec_session *sess,
				    struct hevc_frame *frame)
{
	struct codec_hevc *hevc = sess->priv;

	if (!hevc || !frame->show)
		return;

	if (frame->done) {
		/* Already output -- avoid double v4l2_m2m_buf_done which
		 * triggers a vb2 WARN and can crash.  This can happen when
		 * SLICE_SEGMENT_DONE outputs prev_frame and the following
		 * DECPIC_DATA_DONE outputs cur_frame == prev_frame after
		 * session teardown races.
		 */
		return;
	}

	/* M2: copy the slot into a vb2; defer if none free (decode continues). */
	if (!codec_hevc_stage_output(sess, frame))
		return;

	dev_dbg(sess->core->dev,
		"HEVC output: idx=%d poc=%d ts=%llu Y=0x%llx\n",
		frame->index, frame->poc, frame->ts,
		(u64)vb2_dma_contig_plane_dma_addr(&frame->vbuf->vb2_buf, 0));
	codec_hevc_dst_buf_done(sess, frame);
	frame->done = 1;
	hevc->frames_num--;
}

/*
 * Display-order bumping (HEVC Annex C DPB behavior, mirrors BSP
 * vh265.c:output_pic()).  Releases frames whose decode is complete in
 * POC order.  For each iteration, picks the lowest-POC frame with
 * output_ready=1 && done=0 and emits it if we have more such frames
 * buffered than sps_num_reorder_pics_0 (or if flushing).
 *
 * Caller must hold hevc->lock.
 *
 * For IPPP streams (sps_num_reorder_pics_0 == 0) this releases every
 * finished frame immediately, preserving the decode-order behavior.
 */
static void codec_hevc_bump_frames(struct amvdec_session *sess, bool flush)
{
	struct codec_hevc *hevc = sess->priv;
	struct hevc_frame *frame, *best;
	int num_pending;

	for (;;) {
		num_pending = 0;
		best = NULL;
		list_for_each_entry(frame, &hevc->ref_frames_list, list) {
			if (!frame->output_ready || frame->done)
				continue;
			num_pending++;
			if (!best || hevc_frame_before(frame, best))
				best = frame;
		}

		if (!best)
			break;

		/* Bumping: stop when we have just enough buffered to
		 * guarantee the next-to-emit POC is really the minimum.
		 * Force-drain on flush (end of stream / error) and when
		 * the DPB is near-full (mirror BSP vh265.c:4540).
		 *
		 * First-frame fast output (BSP vh265.c fast_output analog):
		 * the very first displayable frame is delivered WITHOUT the
		 * reorder-window hold (the `first_output_done` guard below
		 * suppresses this break only for it).  Otherwise the first
		 * output is deferred until sps_num_reorder_pics_0 + 1 pictures
		 * decode (3 for t265), and the c2/CCodec client's first-output
		 * wait (~400 ms) times out -- it cancels the CAPTURE dequeue
		 * and STREAMOFFs about a millisecond before the first frame
		 * lands, wedging the whole session (measured: client cancel at
		 * t and HEVC output at t+1ms).  The first output-ready frame at
		 * stream start is the IDR (lowest POC -- nothing lower can
		 * precede it), so immediate delivery preserves display order.
		 * The reorder hold resumes from the second output on.
		 */
		if (!flush && hevc->first_output_done &&
		    num_pending <= hevc->sps_num_reorder_pics_0 &&
		    num_pending < MAX_REF_PIC_NUM - 1)
			break;

		/* codec_hevc_output_frame() already sets frame->done = 1
		 * and decrements frames_num.
		 */
		codec_hevc_output_frame(sess, best);
		/* Not delivered (M2 with no free CAPTURE buffer, or a frame
		 * that is not to be shown): the same frame would be picked
		 * again forever, spinning here with hevc->lock held.  It is
		 * retried on the next picture or buffer return.
		 */
		if (!best->done)
			break;
		hevc->first_output_done = true;
	}
}

static void codec_hevc_set_sao(struct amvdec_session *sess,
			       int slot, struct vb2_buffer *vb,
			       int new_pic)
{
	struct amvdec_core *core = sess->core;
	struct codec_hevc *hevc = sess->priv;
	/* M2 shadow-copy: when the internal NV12 pool is active (NV12+MMU),
	 * the SAO double-write target is the driver-owned nv12_internal[slot]
	 * and vb is NULL (no vb2 grabbed until output).  Otherwise vb is the
	 * decode target as before.
	 */
	bool m2 = hevc->common.nv12_internal_vaddr[slot] != NULL;
	dma_addr_t buf_y_paddr;
	dma_addr_t buf_u_v_paddr;
	u32 val;
	int use_fbc = codec_hevc_use_fbc(sess->pixfmt_cap, hevc->is_10bit);
	int use_mmu = codec_hevc_use_mmu(core->platform->revision,
					 sess->pixfmt_cap, hevc->is_10bit);

	/* BSP writes sentinel 0xffffffff to SAO_Y_START_ADDR on new_pic.
	 * This may be required to reset SAO DMA internal state for the
	 * new picture before the actual address is written below.
	 */
	if (new_pic)
		amvdec_write_dos(core, HEVC_SAO_Y_START_ADDR, 0xffffffff);

	/* Determine body buffer address (same logic as VP9) */
	if (codec_hevc_use_downsample(sess->pixfmt_cap, hevc->is_10bit))
		buf_y_paddr = hevc->common.fbc_buffer_paddr[slot];
	else if (m2)
		buf_y_paddr = hevc->common.nv12_internal_paddr[slot];
	else
		buf_y_paddr = vb2_dma_contig_plane_dma_addr(vb, 0);

	/* FBC compressed body setup.
	 * On G12A+ even 8-bit decode uses FBC internally (dw_mode=1).
	 * The CM_BODY_START_ADDR must point to the FBC buffer.
	 * On GXL with compressed refs, point to per-frame FBC buffer.
	 */
	if (use_fbc) {
		/* Clear FBC_body_uncouple(17) + mem_saving(9) for FBC setup */
		val = amvdec_read_dos(core, HEVC_SAO_CTRL5) & ~0xff0200;
		amvdec_write_dos(core, HEVC_SAO_CTRL5, val);
		/* In MMU paged-mem mode (G12A+: HEVCD_MPP_DECOMP_CTL1=BIT(4)) the
		 * compressed body is written THROUGH the MMU page table, not to a
		 * contiguous CM_BODY_START_ADDR -- the NV12 path does exactly this
		 * via the else-if below.  For an FBC-output capture this use_fbc
		 * branch is taken, so without the !use_mmu guard we program a
		 * contiguous CM_BODY while paged mode is active -> conflict.  Only
		 * program CM_BODY on non-MMU (GXL compressed-ref). */
		if (!use_mmu)
			amvdec_write_dos(core, HEVC_CM_BODY_START_ADDR, buf_y_paddr);
	} else if (use_mmu) {
		/* MMU paged_mem_mode (HEVCD_MPP_DECOMP_CTL1=BIT(4), set in
		 * codec_hevc_setup_decode_head) is active: the HEVC engine
		 * writes the current frame's compressed body THROUGH the MMU
		 * page table (mmu_map, filled per-frame by codec_hevc_fill_mmu_map
		 * for cur_frame), not to a contiguous CM_BODY_START_ADDR.
		 *
		 * BSP confirms it does NOT program HEVC_CM_BODY_START_ADDR in
		 * mmu_enable mode (h266/vh266.c:4003 `if (!hevc->mmu_enable)`;
		 * avs2_fb `#ifndef AVS2_10B_MMU`).  The previous write of a
		 * contiguous fbc_buffer_paddr here is what forced the large
		 * physically-contiguous CMA allocation that fails on a
		 * fragmented pool (KI-115).  Skip it; the page table maps the
		 * (scatter-gather) body pages.
		 */
	} else if (hevc_needs_compress_ref(core, hevc->is_10bit)) {
		/* GXL compressed reference mode: body buffer per frame */
		amvdec_write_dos(core, HEVC_CM_BODY_START_ADDR,
				 hevc->common.fbc_buffer_paddr[slot]);
	}

	/* NV12 output addresses.
	 * BSP with dw_mode=0x10 (double-write only, 8-bit NV12):
	 *   SAO_Y_START_ADDR = 0xffffffff sentinel (kept, NOT overwritten)
	 *   SAO_C_START_ADDR = dw_u_v_adr
	 *   SAO_Y_WPTR = dw_y_adr
	 *   SAO_C_WPTR = dw_u_v_adr
	 * The sentinel tells SAO hardware to use WPTR as the write target.
	 */
	if (codec_hevc_is_nv12(sess->pixfmt_cap)) {
		if (m2) {
			/* SAO double-writes into the driver-owned internal NV12
			 * buffer; output_frame later memcpy's it to a vb2.  The
			 * internal alloc is one contiguous Y+UV block, so UV is
			 * at Y + output_size. */
			buf_y_paddr = hevc->common.nv12_internal_paddr[slot];
			buf_u_v_paddr =
				buf_y_paddr + amvdec_get_output_size(sess);
		} else {
			buf_y_paddr = vb2_dma_contig_plane_dma_addr(vb, 0);
			if (vb->num_planes > 1)
				buf_u_v_paddr =
					vb2_dma_contig_plane_dma_addr(vb, 1);
			else
				buf_u_v_paddr =
					buf_y_paddr + amvdec_get_output_size(sess);
		}
		/* BSP: SAO_Y_START = 0xffffffff sentinel for compress+dw modes,
		 * but for dw_mode=0x10 (direct NV12 only), the START address
		 * stays as sentinel. For non-compress mode (#else path in BSP),
		 * START = mc_y_adr. Use sentinel for MMU, direct addr otherwise.
		 */
		amvdec_write_dos(core, HEVC_SAO_Y_START_ADDR,
				 buf_y_paddr);
		amvdec_write_dos(core, HEVC_SAO_C_START_ADDR, buf_u_v_paddr);
		amvdec_write_dos(core, HEVC_SAO_Y_WPTR, buf_y_paddr);
		amvdec_write_dos(core, HEVC_SAO_C_WPTR, buf_u_v_paddr);
		dev_dbg(core->dev,
			"SAO addrs: Y=0x%08llx UV=0x%08llx outsize=0x%x 10bit=%d\n",
			(u64)buf_y_paddr, (u64)buf_u_v_paddr,
			amvdec_get_output_size(sess), hevc->is_10bit);
		dev_dbg(core->dev,
			"SAO CTRL5=0x%x Y_LEN=0x%x C_LEN=0x%x CTRL1=0x%x\n",
			 amvdec_read_dos(core, HEVC_SAO_CTRL5),
			 amvdec_read_dos(core, HEVC_SAO_Y_LENGTH),
			 amvdec_read_dos(core, HEVC_SAO_C_LENGTH),
			 amvdec_read_dos(core, HEVC_SAO_CTRL1));
	}

	/* MMU header address (for address translation via ANC2AXI) */
	if (use_mmu) {
		amvdec_write_dos(core, HEVC_CM_HEADER_START_ADDR,
				 hevc->common.mmu_header_paddr[slot]);

		/* FBC-output display fix (fbc-output-display-impl-plan.md S1/S2,
		 * PROVEN by the S0 PoC: 7.7 dB garbage -> 23.4 dB picture).  For
		 * an FBC-output (am21c) capture the HW just wrote the compress
		 * HEADER to the SEPARATE per-slot buffer mmu_header_paddr[slot]
		 * (HEVC_CM_HEADER_START_ADDR above), NOT into the vb2 capture
		 * buffer.  The contiguous am21c BODY is at the vb2 capture-buffer
		 * base (== gem->dma_addr / AFBC_BODY_BADDR on the display side).
		 * Register {body base -> header} with meson-DRM so the VD1 AFBC
		 * scatter HEAD reads the real header instead of the vb2 base.
		 * Strict no-op for non-FBC-output decode (NV12 / downsample):
		 * for FBC output, use_downsample and the M2 NV12-internal path
		 * are both false, so vb is the real capture buffer and
		 * vb2_dma_contig_plane_dma_addr(vb, 0) is exactly what the
		 * display sees as gem->dma_addr.
		 */
		if (codec_hevc_is_fbc_output(sess->pixfmt_cap))
			meson_overlay_fbc_set_header(
				vb2_dma_contig_plane_dma_addr(vb, 0),
				hevc->common.mmu_header_paddr[slot]);
		/* BIT(10): use HEVC_CM_HEADER_START_ADDR for MMU.
		 * BIT(9): mem_saving_mode -- 8-bit FBC uses 3200 bytes per
		 * 64x32 block; 10-bit uses full 4096.  BSP only sets BIT(9)
		 * when mem_saving_mode=1 (8-bit).  Setting it for 10-bit
		 * causes body format mismatch: SAO reads compressed refs
		 * as 3200-byte blocks but they were written as 4096-byte,
		 * corrupting the chroma plane (KI-060).
		 */
		amvdec_write_dos_bits(core, HEVC_SAO_CTRL5,
				      SAO_CTRL5_USE_CM_HDR);
		if (!hevc->is_10bit)
			amvdec_write_dos_bits(core, HEVC_SAO_CTRL5,
					      SAO_CTRL5_MEM_SAVING);
		else
			amvdec_clear_dos_bits(core, HEVC_SAO_CTRL5,
					      SAO_CTRL5_MEM_SAVING);
	}

	amvdec_write_dos(core, HEVC_SAO_Y_LENGTH,
			 amvdec_get_output_size(sess));
	amvdec_write_dos(core, HEVC_SAO_C_LENGTH,
			 amvdec_get_output_size(sess) / 2);

	/* DBLK_CFGB (G12A+): output mode control.
	 * BSP dw_mode=1 (FBC+NV12): BIT(8) (compress write) | BIT(9) (double write).
	 * Both are needed on G12A+ even for 8-bit -- the hardware always does
	 * compressed decode internally and double-writes to NV12.
	 */
	if (core->platform->revision >= VDEC_REVISION_G12A) {
		amvdec_clear_dos_bits(core, HEVC_DBLK_CFGB,
				      DBLK_CFGB_PIPELINE_MODE | BIT(5) |
				      DBLK_CFGB_COMPRESS_WR |
				      DBLK_CFGB_DOUBLE_WR);
		/* For an FBC-output capture (AMS8/AMSA) the vb2 buffer receives
		 * the compressed body only; the is_nv12 block that programs
		 * SAO_C_START/Y_WPTR/C_WPTR is skipped, so DOUBLE_WR here would
		 * make SAO write NV12 to stale addresses.  Compress-write-only
		 * for FBC output; compress+double-write (dw_mode=1) for NV12.
		 */
		if (codec_hevc_is_fbc_output(sess->pixfmt_cap)) {
			dev_dbg(core->dev, "FBC-output: DBLK compress-write-only (no NV12 double-write)\n");
			amvdec_write_dos_bits(core, HEVC_DBLK_CFGB,
					      DBLK_CFGB_COMPRESS_WR);
		} else {
			amvdec_write_dos_bits(core, HEVC_DBLK_CFGB,
					      DBLK_CFGB_COMPRESS_WR |
					      DBLK_CFGB_DOUBLE_WR);
		}
		/* dblk pipeline mode for large frames */
		if (sess->width >= 1280)
			amvdec_write_dos_bits(core, HEVC_DBLK_CFGB,
					      DBLK_CFGB_PIPELINE_MODE);

		/* KI-001 Strike 9: SAO_CTRL5 BIT(7) Cb chroma rounding workaround
		 * was historically set unconditionally for G12+.  Comment
		 * claimed "Y and Cr bit-perfect without it; Cb has ~56 dB
		 * error".  But 2026-04-26 retest shows the OPPOSITE pattern
		 * is true on G12B+SM1 with current mainline: V (Cr) is the
		 * worst (~13 dB), Y is bad (~18 dB), U (Cb) is at silicon
		 * floor (~54 dB).  VP9 (works bit-exact on G12+) does NOT
		 * set this bit.  Hypothesis: the workaround was originally
		 * tested against a different test source / firmware era and
		 * actually REGRESSES the V plane on current upstream firmware.
		 * Strike 9: skip the bit on G12+ to test.
		 */
	}

	/* SAO_CTRL1: endianness and mode control.
	 * BSP dw_mode=1: bits[1:0]=0 (both compress and double-write enabled).
	 * BSP dw_mode=0x10: bit[0]=1 (cm_disable, double-write only).
	 *
	 * GXL HEVC 8-bit uses dw_mode=1 (compressed refs + SAO double-write).
	 * Both compress and DW must be enabled (bits[1:0]=0).
	 */
	val = amvdec_read_dos(core, HEVC_SAO_CTRL1) & ~0xffff;
	val |= SAO_CTRL1_CLK_FORCEON;
	val |= SAO_CTRL1_AXI_ALIGN_64;
	val |= SAO_CTRL1_ENDIAN_MASK; /* Little-endian per 2-byte swap */
	/* bits[1:0] = 0 for compressed + double-write mode (dw_mode=1) */
	amvdec_write_dos(core, HEVC_SAO_CTRL1, val);

	/* No downscale for full-res NV12 */
	val = amvdec_read_dos(core, HEVC_SAO_CTRL5) & ~SAO_CTRL5_DW_SCALE_MASK;
	amvdec_write_dos(core, HEVC_SAO_CTRL5, val);

	val = amvdec_read_dos(core, HEVCD_IPP_AXIIF_CONFIG);
	val &= ~(IPP_AXIIF_ENDIAN_MASK | (3 << 8) | IPP_AXIIF_UV_SWAP);
	val |= IPP_AXIIF_ENDIAN_MASK; /* Little-endian per 2-byte swap */
	val |= IPP_AXIIF_ALIGN_64;    /* Linear_LineAlignment = 64 bytes */
	if (codec_hevc_is_nv21(sess->pixfmt_cap))
		val |= IPP_AXIIF_UV_SWAP;
	amvdec_write_dos(core, HEVCD_IPP_AXIIF_CONFIG, val);

	dev_dbg(core->dev,
		"SAO set: idx=%d Y_LEN=0x%x C_LEN=0x%x Y_START=0x%x C_START=0x%x CTRL1=0x%x CTRL5=0x%x BODY=0x%x HDR=0x%x\n",
		slot,
		amvdec_read_dos(core, HEVC_SAO_Y_LENGTH),
		amvdec_read_dos(core, HEVC_SAO_C_LENGTH),
		amvdec_read_dos(core, HEVC_SAO_Y_START_ADDR),
		amvdec_read_dos(core, HEVC_SAO_C_START_ADDR),
		amvdec_read_dos(core, HEVC_SAO_CTRL1),
		amvdec_read_dos(core, HEVC_SAO_CTRL5),
		amvdec_read_dos(core, HEVC_CM_BODY_START_ADDR),
		amvdec_read_dos(core, HEVC_CM_HEADER_START_ADDR));
}

/* Configure MPRED (Motion Prediction) hardware for HEVC decode.
 * This is critical for inter-frame prediction (P/B frames).
 * Based on BSP config_mpred_hw() in vh265.c.
 *
 * MPRED CTRL0 register layout (from BSP):
 *   [1:0]   slice_type (0=B, 1=P, 2=I)
 *   [2]     new_pic (first slice of new picture)
 *   [3]     new_tile
 *   [4]     isNextSliceSegment
 *   [5]     TMVPFlag (temporal MV prediction enable)
 *   [6]     LDCFlag (low-delay check)
 *   [7]     ColFromL0Flag
 *   [8]     above_ptr_ctrl
 *   [9]     above_en
 *   [10]    mv_wr_en (MV write enable -- CRITICAL)
 *   [11]    mv_rd_en (MV read enable -- CRITICAL)
 *   [12]    col_isIntra (collocated frame is intra)
 *   [13]    buffer_linear (=1)
 *   [14]    LongTerm_Curr
 *   [15]    LongTerm_Col
 *   [19:16] lcu_size_log2
 *   [23:20] cu_size_log2 (=3, 8x8 min CU)
 *   [26:24] plevel (log2_parallel_merge_level)
 */
static void codec_hevc_set_mpred(struct amvdec_core *core,
				 struct codec_hevc *hevc)
{
	union hevc_rpm_param *param = &hevc->rpm_param;
	u32 log2_min_cb = param->p.log2_min_coding_block_size_minus3 + 3;
	u32 log2_ctb = log2_min_cb + param->p.log2_diff_max_min_coding_block_size;
	u32 lcu_size_log2 = log2_ctb;
	u32 lcu_size = 1 << log2_ctb;
	u32 lcu_width = (hevc->width + lcu_size - 1) / lcu_size;
	u32 lcu_height = (hevc->height + lcu_size - 1) / lcu_size;
	u32 slice_type = param->p.slice_type;
	u32 tmvp_flag = param->p.slice_temporal_mvp_enable_flag;
	u32 col_from_l0 = param->p.collocated_from_l0_flag;
	u32 plevel = param->p.log2_parallel_merge_level;
	u32 max_merge_cand = 5 - param->p.five_minus_max_num_merge_cand;
	u32 mv_wr_start, mv_rd_start, mv_rd_end;
	u32 above_en, mv_wr_en, mv_rd_en, col_isIntra;
	u32 ldc_flag;		/* CTRL0[6] LDCFlag -- see below */
	u32 data32;
	int col_poc_val = 0;   /* collocated reference POC */
	int col_buf_idx_snap = -1;  /* snapshot for mpred_trace debugfs */
	bool has_col_ref = false;  /* true if valid collocated ref found */
	int MV_MEM_UNIT;
	int i;

	/* Clamp merge candidates (BSP sanity check) */
	if (max_merge_cand > 5 || max_merge_cand < 1)
		max_merge_cand = 5;

	dev_dbg(core->dev,
		"MPRED enter: slice=%u lcu=%u tmvp=%u merge=%u prev=%p\n",
		slice_type, lcu_size, tmvp_flag, max_merge_cand,
		hevc->prev_frame);

	/* MV write/read enable based on slice type (BSP logic) */
	if (slice_type != 2) {
		/* P or B slice */
		above_en = 1;
		mv_wr_en = 1;
		mv_rd_en = 1;
		col_isIntra = 0;
	} else {
		/* I slice -- still write MVs but don't read */
		above_en = 1;
		mv_wr_en = 1;
		mv_rd_en = 0;
		col_isIntra = 0;
	}

	/* MV buffer addresses -- per-frame MV buffer (BSP: pic->mpred_mv_wr_start_addr) */
	mv_wr_start = hevc->workspace_paddr + MPRED_MV_OFFSET +
		      (hevc->cur_frame->index * hevc->mpred_mv_buf_size);

	/* Collocated reference: BSP uses collocated_ref_idx to select from
	 * L0 or L1 based on collocated_from_l0_flag and slice_type.
	 * For B-slices: col_from_l0=1 -> use L0, col_from_l0=0 -> use L1
	 * For P-slices: always use L0
	 */
	{
		int col_ref = param->p.collocated_ref_idx;
		int col_buf_idx = -1;
		bool col_found = false;  /* did we resolve col_ref to a POC? */

		if (slice_type == 0) {
			/* B-slice: select L0 or L1 based on col_from_l0 */
			if (col_from_l0) {
				if (col_ref < hevc->ref_num_l0) {
					col_poc_val = hevc->ref_poc_list0[col_ref];
					col_found = true;
				}
			} else {
				if (col_ref < hevc->ref_num_l1) {
					col_poc_val = hevc->ref_poc_list1[col_ref];
					col_found = true;
				}
			}
		} else if (slice_type == 1) {
			/* P-slice: always from L0 */
			if (col_ref < hevc->ref_num_l0) {
				col_poc_val = hevc->ref_poc_list0[col_ref];
				col_found = true;
			}
		}

		if (col_found)
			col_buf_idx = codec_hevc_find_buf_index(hevc, col_poc_val);
		col_buf_idx_snap = col_buf_idx;

		if (col_buf_idx >= 0) {
			mv_rd_start = hevc->workspace_paddr + MPRED_MV_OFFSET +
				      (col_buf_idx * hevc->mpred_mv_buf_size);
			has_col_ref = true;
		} else if (hevc->prev_frame) {
			/* Fallback to prev_frame if collocated ref not found */
			mv_rd_start = hevc->workspace_paddr + MPRED_MV_OFFSET +
				      (hevc->prev_frame->index * hevc->mpred_mv_buf_size);
			col_poc_val = hevc->prev_frame->poc;
			has_col_ref = true;
		} else {
			mv_rd_start = mv_wr_start;
			col_isIntra = 1;
		}

		/* Check if collocated frame has same dimensions */
		if (has_col_ref && col_buf_idx >= 0) {
			struct hevc_frame *col_frame;
			bool size_mismatch = false;

			list_for_each_entry(col_frame,
					    &hevc->ref_frames_list, list) {
				if (col_frame->index == col_buf_idx) {
					if (col_frame->width !=
					    hevc->cur_frame->width ||
					    col_frame->height !=
					    hevc->cur_frame->height)
						size_mismatch = true;
					break;
				}
			}
			if (size_mismatch) {
				col_isIntra = 1;
				mv_rd_en = 0;
			}
		}
	}

	/* MV_MEM_UNIT: bytes per LCU for motion vector storage (BSP).
	 * Scales with LCU area: 64x64=0x200(512), 32x32=0x80(128),
	 * 16x16=0x20(32) -- 2 bytes per 4x4 sub-block for MV+refIdx.
	 */
	MV_MEM_UNIT = lcu_size_log2 == 6 ? 0x200 :
		      lcu_size_log2 == 5 ? 0x80 : 0x20;

	mv_rd_end = mv_rd_start + (lcu_width * lcu_height * MV_MEM_UNIT);

	/*
	 * Backstop for the resolution-derived slot sizing: the slot array was
	 * sized for hevc->width/height at start()/resume(), so this product
	 * must fit.  If a path ever reaches here with an SPS bigger than what
	 * the workspace was laid out for, clamp the read window to the slot
	 * rather than let the engine walk into the next slot, and say so.
	 */
	if (WARN_ONCE(lcu_width * lcu_height * MV_MEM_UNIT >
		      hevc->mpred_mv_buf_size,
		      "HEVC: MPRED MV window %#x > slot %#x at %ux%u (lcu %u)\n",
		      lcu_width * lcu_height * MV_MEM_UNIT,
		      hevc->mpred_mv_buf_size, hevc->width, hevc->height,
		      lcu_size))
		mv_rd_end = mv_rd_start + hevc->mpred_mv_buf_size;

	/* CTRL0[6] LDCFlag == the HEVC spec's NoBackwardPredFlag (H.265
	 * 8.5.3.2.9, "Derivation process for temporal luma motion vector
	 * prediction"): it is 1 iff EVERY active reference picture of the
	 * current slice has POC <= the current POC, i.e. no reference lies in
	 * the future.  The HW uses it exactly the way the spec uses
	 * NoBackwardPredFlag -- to pick WHICH collocated MV list supplies the
	 * temporal MV predictor for a colPb: when it is 1 the predictor is
	 * taken from the collocated picture's list X (X == the list being
	 * derived), when it is 0 from list collocated_from_l0_flag.  Getting
	 * it wrong yields subtly wrong TMVP/merge MVs on the minority of PUs
	 * whose colPb is inter-coded -- a small, per-PU, reference-propagating
	 * drift that an IRAP clears.
	 *
	 * Computed per-slice exactly as BSP config_mpred_hw() does
	 * (vh265.c:5879 writes (hevc->LDCFlag & 1) << 6, value derived at
	 * vh265.c:8042-8058).  Historically hard-coded to 0 here; the value
	 * was computed for the debugfs trace only, on the strength of a
	 * 2026-04-19 A/B that was later found to have run on a stream where
	 * both arms compute the SAME LDCFlag (a no-op A/B).  It is now written
	 * to the register, which is what the BSP does.
	 *
	 * The lists are 16 entries and ref_num_lX is already clamped to that.
	 */
	ldc_flag = 0;
	if (slice_type != 2) {	/* P or B */
		int n_l0 = min_t(int, hevc->ref_num_l0,
				 (int)ARRAY_SIZE(hevc->ref_poc_list0));
		int n_l1 = min_t(int, hevc->ref_num_l1,
				 (int)ARRAY_SIZE(hevc->ref_poc_list1));
		int n;

		ldc_flag = 1;
		for (n = 0; n < n_l0 && ldc_flag; n++)
			if (hevc->ref_poc_list0[n] > hevc->curr_poc)
				ldc_flag = 0;
		if (slice_type == 0)	/* B: L1 must also be all-past */
			for (n = 0; n < n_l1 && ldc_flag; n++)
				if (hevc->ref_poc_list1[n] > hevc->curr_poc)
					ldc_flag = 0;
	}

	/* CTRL3: MV precision/search range constant (BSP) */
	amvdec_write_dos(core, HEVC_MPRED_CTRL3, MPRED_CTRL3_DEFAULT);

	/* CTRL0: master control register -- THIS WAS MISSING */
	data32 = (slice_type & 3) |
		 ((hevc->new_pic & 1) << 2) | /* new_pic from slice header */
		 (0 << 3) |		/* new_tile = 0 (no tiles) */
		 (0 << 4) |		/* isNextSliceSegment = 0 */
		 ((tmvp_flag & 1) << 5) |
		 ((ldc_flag & 1) << 6) |	/* LDCFlag (BSP-parity) */
		 ((col_from_l0 & 1) << 7) |
		 (0 << 8) |		/* above_ptr_ctrl = 0 */
		 ((above_en & 1) << 9) |
		 ((mv_wr_en & 1) << 10) |
		 ((mv_rd_en & 1) << 11) |
		 ((col_isIntra & 1) << 12) |
		 (1 << 13) |		/* buffer_linear = 1 */
		 (0 << 14) |		/* LongTerm_Curr = 0 */
		 (0 << 15) |		/* LongTerm_Col = 0 */
		 ((lcu_size_log2 & 0xf) << 16) |
		 (3 << 20) |		/* cu_size_log2 = 3 (8x8) */
		 ((plevel & 0x7) << 24);
	data32 &= ~BIT(28);
	amvdec_write_dos(core, HEVC_MPRED_CTRL0, data32);

	/* Emit per-slice snapshot to debugfs mpred_trace (read via
	 * /sys/kernel/debug/meson-vdec/mpred_trace).  Informational only --
	 * does NOT affect decode.  Used by TODO-HEVC-GXL-BFRAME-CONTENT
	 * and TASK-004 investigations.
	 */
	{
		struct vdec_dbg_mpred_entry e = {
			.timestamp_ns = ktime_get_ns(),
			.codec = 0, /* hevc */
			.slice_type = slice_type,
			.new_pic = hevc->new_pic ? 1 : 0,
			.col_from_l0 = col_from_l0 ? 1 : 0,
			.curr_poc = hevc->curr_poc,
			.ref_num_l0 = min_t(u32, hevc->ref_num_l0,
					    VDEC_DBG_MPRED_MAX_REFS),
			.ref_num_l1 = min_t(u32, hevc->ref_num_l1,
					    VDEC_DBG_MPRED_MAX_REFS),
			.tmvp_flag = tmvp_flag ? 1 : 0,
			.num_neg = min_t(int, hevc->num_neg_snap,
					 VDEC_DBG_MPRED_MAX_REFS),
			.num_pos = min_t(int, hevc->num_pos_snap,
					 VDEC_DBG_MPRED_MAX_REFS),
			.mod_flag = param->p.modification_flag & 0xff,
			.num_ref_idx_l0_active = param->p.num_ref_idx_l0_active,
			.num_ref_idx_l1_active = param->p.num_ref_idx_l1_active,
			.col_poc = has_col_ref ? col_poc_val : -1,
			.col_buf_idx = col_buf_idx_snap,
			.collocated_ref_idx = param->p.collocated_ref_idx,
			.ctrl0 = data32,
			.mv_wr_start = mv_wr_start,
			.mv_rd_start = mv_rd_start,
			.mv_rd_end = mv_rd_end,
		};
		u8 n;

		/* LDCFlag is now computed above and WRITTEN to CTRL0[6];
		 * the trace just records the value that went to the HW.
		 */
		e.ldc_flag = ldc_flag & 1;

		for (n = 0; n < e.ref_num_l0; n++)
			e.ref_poc_list0[n] = hevc->ref_poc_list0[n];
		for (n = 0; n < e.ref_num_l1; n++)
			e.ref_poc_list1[n] = hevc->ref_poc_list1[n];
		for (n = 0; n < e.num_neg; n++)
			e.st_curr0[n] = hevc->st_curr0_snap[n];
		for (n = 0; n < e.num_pos; n++)
			e.st_curr1[n] = hevc->st_curr1_snap[n];
		for (n = 0; n < VDEC_DBG_MPRED_MAX_REFS; n++)
			e.cur_rps[n] = param->p.CUR_RPS[n];

		vdec_dbg_log_mpred(core, &e);
	}

	/* CTRL1: merge candidates and chroma mode (BSP) */
	data32 = max_merge_cand |
		 MPRED_CTRL1_AMVP_MAX |
		 MPRED_CTRL1_AMVP_MAX_MEM |
		 MPRED_CTRL1_NUM_CHROMA |
		 MPRED_CTRL1_DM_CHROMA_IDX |
		 MPRED_CTRL1_CLK_FORCEON;
	amvdec_write_dos(core, HEVC_MPRED_CTRL1, data32);

	/* PIC_SIZE and PIC_SIZE_LCU (BSP) */
	amvdec_write_dos(core, HEVC_MPRED_PIC_SIZE,
			 hevc->width | (hevc->height << 16));
	amvdec_write_dos(core, HEVC_MPRED_PIC_SIZE_LCU,
			 (lcu_width - 1) | ((lcu_height - 1) << 16));

	/* TILE_START and TILE_SIZE_LCU (no tiles = full picture) */
	amvdec_write_dos(core, HEVC_MPRED_TILE_START, 0);
	amvdec_write_dos(core, HEVC_MPRED_TILE_SIZE_LCU,
			 lcu_width | (lcu_height << 16));

	/* REF_NUM: L0 and L1 reference counts (BSP) */
	if (slice_type == 2) {
		/* I-slice: no references */
		amvdec_write_dos(core, HEVC_MPRED_REF_NUM, 0);
		amvdec_write_dos(core, HEVC_MPRED_REF_EN_L0, 0);
		amvdec_write_dos(core, HEVC_MPRED_REF_EN_L1, 0);
	} else if (slice_type == 1) {
		/* P-slice: 1 L0 reference */
		amvdec_write_dos(core, HEVC_MPRED_REF_NUM,
				 param->p.num_ref_idx_l0_active);
		data32 = 0;
		for (i = 0; i < param->p.num_ref_idx_l0_active && i < 16; i++)
			data32 |= BIT(i);
		amvdec_write_dos(core, HEVC_MPRED_REF_EN_L0, data32);
		amvdec_write_dos(core, HEVC_MPRED_REF_EN_L1, 0);
	} else {
		/* B-slice: L0 + L1 references */
		amvdec_write_dos(core, HEVC_MPRED_REF_NUM,
				 param->p.num_ref_idx_l0_active |
				 (param->p.num_ref_idx_l1_active << 8));
		data32 = 0;
		for (i = 0; i < param->p.num_ref_idx_l0_active && i < 16; i++)
			data32 |= BIT(i);
		amvdec_write_dos(core, HEVC_MPRED_REF_EN_L0, data32);
		data32 = 0;
		for (i = 0; i < param->p.num_ref_idx_l1_active && i < 16; i++)
			data32 |= BIT(i);
		amvdec_write_dos(core, HEVC_MPRED_REF_EN_L1, data32);
	}

	/* HEVC_MPRED_LT_REF (0xc828): long-term reference bitmask.  BSP
	 * config_mpred_hw() (vh265.c:5916) writes this register UNCONDITIONALLY
	 * every slice (bit i for a long-term L0[i], bit i+16 for a long-term
	 * L1[i]); upstream never wrote it, leaving whatever value the HW
	 * register held from power-on or a prior decode.  A stale nonzero
	 * value would mis-mark a short-term reference as long-term, excluding
	 * it from TMVP/AMVP MV scaling.  The upstream driver + firmware do NOT
	 * support long-term RPS (SUPPORT_LONG_TERM_RPS is undefined here; see
	 * comment ~line 414), so no reference is ever long-term -> the
	 * BSP-faithful mask is 0, which also defensively clears any stale
	 * value.  If long-term RPS support is added, build the mask from the
	 * per-ref long-term flags like BSP does.
	 */
	amvdec_write_dos(core, HEVC_MPRED_LT_REF, 0);

	/* CUR_POC and COL_POC (BSP) */
	amvdec_write_dos(core, HEVC_MPRED_CUR_POC, hevc->curr_poc);
	if (has_col_ref && !col_isIntra)
		amvdec_write_dos(core, HEVC_MPRED_COL_POC, col_poc_val);

	/* Reference POC lists (BSP config_mpred_hw at vh265.c:5792).
	 * BSP comment: "must follow Ref_POC_xx_L0 -> Ref_POC_xx_L1 in pair
	 * write order!!!"  Write L0[i] then L1[i] for each index.
	 */
	for (i = 0; i < 16; i++) {
		int l0_poc = (i < hevc->ref_num_l0) ?
			     hevc->ref_poc_list0[i] : 0;
		int l1_poc = (i < hevc->ref_num_l1) ?
			     hevc->ref_poc_list1[i] : 0;

		amvdec_write_dos(core, HEVC_MPRED_L0_REF00_POC + i * 4,
				 l0_poc);
		amvdec_write_dos(core, HEVC_MPRED_L1_REF00_POC + i * 4,
				 l1_poc);
	}

	/* COLREF_EN_L0/L1: BSP parity -- BSP vh265.c never writes these
	 * registers anywhere.  The "comment-cited BSP" behavior was wrong.
	 * Previous upstream wrote bit 0 unconditionally for the col-ref
	 * case; this could have been gating or enabling collocated-MV
	 * prediction incorrectly.  Strike 4 (2026-04-19): match BSP by
	 * not writing these at all.  If the HW default doesn't work for
	 * our decode path, a follow-up can revisit with a trace-backed
	 * value (currently unknown what the BSP-supported hardware
	 * expects here).
	 */

	/* MV buffer addresses (BSP) */
	amvdec_write_dos(core, HEVC_MPRED_MV_WR_START_ADDR, mv_wr_start);
	amvdec_write_dos(core, HEVC_MPRED_MV_RD_START_ADDR, mv_rd_start);
	amvdec_write_dos(core, HEVC_MPRED_MV_RD_END_ADDR, mv_rd_end);

	/* Row jump for tiled MV access (no tiles = 0) */
	amvdec_write_dos(core, HEVC_MPRED_MV_WR_ROW_JUMP, 0);
	amvdec_write_dos(core, HEVC_MPRED_MV_RD_ROW_JUMP, 0);

	/* ABV (above) buffer and MV write/read pointers:
	 * Only reset on new picture (BSP: config_mpred_hw new_pic branch).
	 * For continuation slices, pointers continue from where the
	 * previous slice left off.
	 */
	if (hevc->new_pic) {
		amvdec_write_dos(core, HEVC_MPRED_ABV_START_ADDR,
				 hevc->workspace_paddr + MPRED_ABV_OFFSET);
		amvdec_write_dos(core, HEVC_MPRED_MV_WPTR, mv_wr_start);
		amvdec_write_dos(core, HEVC_MPRED_MV_RPTR, mv_rd_start);
	}

	/* CTRL4 BIT(6): enable MPRED use for inter prediction.
	 * Must be set AFTER CTRL0 configures the MPRED engine.
	 */
	amvdec_clear_dos_bits(core, HEVC_MPRED_CTRL4, BIT(6));
	if (slice_type != 2 && has_col_ref && !col_isIntra)
		amvdec_write_dos_bits(core, HEVC_MPRED_CTRL4, BIT(6));

	dev_dbg(core->dev,
		"MPRED: type=%u POC=%d col_POC=%d CTRL0=0x%x mv_wr=0x%x mv_rd=0x%x CTRL4=0x%x\n",
		slice_type, hevc->curr_poc, col_poc_val,
		amvdec_read_dos(core, HEVC_MPRED_CTRL0),
		mv_wr_start, mv_rd_start,
		amvdec_read_dos(core, HEVC_MPRED_CTRL4));
}

static void codec_hevc_set_mcrcc(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	u32 val;

	/* Reset mcrcc */
	amvdec_write_dos(core, HEVCD_MCRCC_CTL1, MCRCC_CTL1_RESET);

	amvdec_write_dos(core, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, BIT(1));
	val = amvdec_read_dos(core, HEVCD_MPP_ANC_CANVAS_DATA_ADDR) & 0xffff;
	val |= (val << 16);
	amvdec_write_dos(core, HEVCD_MCRCC_CTL2, val);
	val = amvdec_read_dos(core, HEVCD_MPP_ANC_CANVAS_DATA_ADDR) & 0xffff;
	val |= (val << 16);
	amvdec_write_dos(core, HEVCD_MCRCC_CTL3, val);

	/* Enable mcrcc progressive-mode */
	amvdec_write_dos(core, HEVCD_MCRCC_CTL1, MCRCC_CTL1_ENABLE);
}

/* Allocate compressed reference buffers for GXL HEVC 8-bit.
 * The HEVC firmware on GXL uses FBC internally even for 8-bit content.
 * These buffers hold compressed reference frame data (body+header).
 * SAO double-writes the decoded NV12 output to V4L2 capture buffers.
 */
static int codec_hevc_alloc_compress_bufs(struct amvdec_session *sess,
					  struct codec_hevc_common *comm)
{
	struct device *dev = sess->core->dev;
	struct v4l2_m2m_buffer *buf;
	u32 am21_size = amvdec_am21c_size(sess->width, sess->height,
					  comm->is_10bit);

	if (comm->fbc_buffer_vaddr[0])
		return 0;

	v4l2_m2m_for_each_dst_buf(sess->m2m_ctx, buf) {
		u32 idx = buf->vb.vb2_buf.index;
		dma_addr_t paddr;
		void *vaddr = dma_alloc_coherent(dev, am21_size, &paddr,
						 GFP_KERNEL);
		if (!vaddr) {
			codec_hevc_free_fbc_buffers(sess, comm);
			return -ENOMEM;
		}

		comm->fbc_buffer_vaddr[idx] = vaddr;
		comm->fbc_buffer_paddr[idx] = paddr;
	}

	return 0;
}

/* Set up ANC2AXI with compressed body addresses for GXL HEVC 8-bit.
 * In compressed reference mode (BSP dw_mode=1), the ANC2AXI table
 * has ONE entry per buffer (body address only), unlike NV12 mode
 * which has TWO entries (Y+UV).
 */
static void codec_hevc_setup_compress_anc2axi(struct amvdec_session *sess,
					       struct codec_hevc_common *comm)
{
	struct amvdec_core *core = sess->core;
	struct vb2_queue *q = v4l2_m2m_get_dst_vq(sess->m2m_ctx);
	int i;

	amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_CONF_ADDR,
			 ANC2AXI_CONF_AUTO_INC);

	for (i = 0; i < vb2_get_num_buffers(q); ++i) {
		if (!vb2_get_buffer(q, i))
			continue;

		/* Write FBC body address (compressed reference frame) */
		amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_DATA,
				 comm->fbc_buffer_paddr[i] >> 5);
	}

	amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_CONF_ADDR, 1);
	amvdec_write_dos(core, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR,
			 (0 << 8) | (0 << 1) | 1);
	for (i = 0; i < 32; ++i)
		amvdec_write_dos(core, HEVCD_MPP_ANC_CANVAS_DATA_ADDR, 0);
}

/* Set up ANC2AXI with direct Y/UV buffer addresses for 8-bit NV12.
 * This bypasses the MMU path used for FBC/10-bit decode.
 */
static void codec_hevc_setup_buffers_nv12(struct amvdec_session *sess,
					  struct codec_hevc_common *comm)
{
	struct amvdec_core *core = sess->core;
	struct v4l2_m2m_buffer *buf;
	int i;

	amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_CONF_ADDR,
			 ANC2AXI_CONF_AUTO_INC);

	v4l2_m2m_for_each_dst_buf(sess->m2m_ctx, buf) {
		struct vb2_buffer *vb = &buf->vb.vb2_buf;
		dma_addr_t buf_y_paddr;
		dma_addr_t buf_uv_paddr;

		buf_y_paddr = vb2_dma_contig_plane_dma_addr(vb, 0);
		amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_DATA,
				 buf_y_paddr >> 5);

		if (vb->num_planes > 1)
			buf_uv_paddr =
				vb2_dma_contig_plane_dma_addr(vb, 1);
		else
			buf_uv_paddr =
				buf_y_paddr + amvdec_get_output_size(sess);
		amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_DATA,
				 buf_uv_paddr >> 5);
	}

	amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_CONF_ADDR, 1);
	amvdec_write_dos(core, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, 1);
	for (i = 0; i < 32; ++i)
		amvdec_write_dos(core, HEVCD_MPP_ANC_CANVAS_DATA_ADDR, 0);
}

static void codec_hevc_setup_decode(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_hevc *hevc = sess->priv;
	union hevc_rpm_param *param = &hevc->rpm_param;
	/* Compute actual CTB size from bitstream SPS parameters */
	u32 log2_min_cb = param->p.log2_min_coding_block_size_minus3 + 3;
	u32 log2_ctb = log2_min_cb + param->p.log2_diff_max_min_coding_block_size;
	u32 lcu_size = 1 << log2_ctb;
	u32 lcu_size_log2 = log2_ctb;
	u32 lcu_width = (hevc->width + lcu_size - 1) / lcu_size;
	u32 lcu_height = (hevc->height + lcu_size - 1) / lcu_size;
	u32 misc_flag0 = param->p.misc_flag0;
	u32 slice_deblocking_filter_disabled;
	u32 val, cfg9, cfg1, data32_2;

	hevc->lcu_total = lcu_width * lcu_height;

	/* HEVC decode pipeline configuration:
	 *
	 * G12A+: MMU mode (DECOMP_CTL1=BIT(4)), compressed decode with
	 * SAO double-write to NV12 output.
	 *
	 * GXL 8-bit: The HEVC firmware always uses compressed references
	 * internally. Unlike H.264/MPEG which support 2-plane NV12 refs
	 * via DECOMP_CTL1=BIT(31), the HEVC engine requires FBC even for
	 * 8-bit content. BSP uses dw_mode=1: DECOMP_CTL1=0 (compressed
	 * refs), DECOMP_CTL2=body_size/32, with SAO double-write to NV12.
	 *
	 * The common setup_decode_head sets BIT(31) for non-MMU non-FBC
	 * 8-bit, which is correct for VDEC_1 codecs but wrong for HEVC.
	 * Override it here with compressed mode for pre-G12A HEVC.
	 */
	codec_hevc_setup_decode_head(sess, hevc->is_10bit);

	/* Buffer setup and MMU map fill only needed for first slice of a
	 * new picture.  Continuation slices decode into the same frame.
	 */
	if (hevc->new_pic) {
		/* Allocate per-frame FBC body storage + MMU headers via
		 * codec_hevc_setup_buffers() below, which uses the scatter-
		 * gather allocator (codec_hevc_fbc_sg, default): per-page
		 * order-0 allocations the HEVC MMU page table maps.  No large
		 * physically-contiguous CMA run is required (KI-115).
		 *
		 * The previously redundant codec_hevc_alloc_compress_bufs()
		 * here allocated a contiguous ~3.2 MB (800-page @1080p) FBC body
		 * per DPB buffer.  It existed only to feed a contiguous
		 * fbc_buffer_paddr[] into HEVC_CM_BODY_START_ADDR (KI-081) -- but
		 * in MMU paged_mem_mode (DECOMP_CTL1=BIT(4)) the engine writes
		 * the body through the page table (codec_hevc_fill_mmu_map maps
		 * cur_frame's pages), and CM_BODY_START_ADDR is no longer
		 * programmed (codec_hevc_set_sao, matching the BSP
		 * `if (!mmu_enable)`).  With the body write paged, the contiguous
		 * alloc is dead weight that failed + leaked on a fragmented pool,
		 * so it is removed.  KI-081 stays fixed: the page table maps each
		 * frame's own (SG) pages, never page 0.
		 */
		if (hevc_needs_compress_ref(core, hevc->is_10bit)) {
			u32 width_64 = ALIGN(sess->width, 64) / 64;
			u32 height_32 = ALIGN(sess->height, 32) / 32;
			u32 body_size = (hevc->is_10bit ? 4096 : 3200) *
					width_64 * height_32;
			amvdec_write_dos(core, HEVCD_MPP_DECOMP_CTL1, 0);
			amvdec_write_dos(core, HEVCD_MPP_DECOMP_CTL2,
					 body_size / 32);
			amvdec_write_dos(core, HEVC_CM_BODY_LENGTH,
					 body_size);
			amvdec_write_dos(core, HEVC_CM_HEADER_OFFSET,
					 body_size);
			amvdec_write_dos(core, HEVC_CM_HEADER_LENGTH,
					 amvdec_am21c_head_size(
						 sess->width,
						 sess->height));
			codec_hevc_setup_compress_anc2axi(sess,
							  &hevc->common);
		} else if (codec_hevc_setup_buffers(sess, &hevc->common,
						    hevc->is_10bit)) {
			dev_err(core->dev,
				"HEVC: buffer setup failed, aborting decode (KI-113)\n");
			amvdec_abort(sess);
			return;
		}

		if (codec_hevc_use_mmu(core->platform->revision,
				       sess->pixfmt_cap, hevc->is_10bit)) {
			codec_hevc_write_mmu_map_addr(core,
						hevc->common.mmu_map_paddr);
			codec_hevc_fill_mmu_map(sess, &hevc->common,
					hevc->cur_frame->index,
					hevc->cur_frame->vbuf ?
					&hevc->cur_frame->vbuf->vb2_buf : NULL);
		}
	}

	/* config_mc_buffer: write per-reference buffer indices to the MC
	 * canvas table.  The hardware uses this to resolve L0[i]/L1[i]
	 * reference indices to ANC2AXI entries (physical buffer addresses).
	 *
	 * BSP: config_mc_buffer() at vh265.c:4619 -- writes all L0 and L1
	 * entries from m_aiRefPOCList0/1, looked up via get_pic_by_POC().
	 *
	 * In MMU mode, canvas_id = buffer_index (one ANC2AXI entry per buf).
	 */
	{
		u32 slice_type = param->p.slice_type;
		int i;

		if (slice_type != 2 && hevc->ref_num_l0 > 0) {
			/* L0 references: start at canvas index 0 */
			amvdec_write_dos(core,
					 HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR,
					 (0 << 8) | (0 << 1) | 1);
			for (i = 0; i < hevc->ref_num_l0; i++) {
				int buf_idx = codec_hevc_find_buf_index(
					hevc, hevc->ref_poc_list0[i]);
				u32 canvas_val;

				if (buf_idx < 0) {
					/* Fallback: use prev_frame if available */
					buf_idx = hevc->prev_frame ?
						  hevc->prev_frame->index : 0;
				}
				canvas_val = (buf_idx << 16) |
					     (buf_idx << 8) | buf_idx;
				amvdec_write_dos(core,
					HEVCD_MPP_ANC_CANVAS_DATA_ADDR,
					canvas_val);
			}
			dev_dbg(core->dev,
				"MC config: slice=%u L0=%d L1=%d POC=%d held=0x%lx\n",
				slice_type, hevc->ref_num_l0,
				hevc->ref_num_l1, hevc->curr_poc,
				hevc->ref_held);

			if (slice_type == 0 && hevc->ref_num_l1 > 0) {
				/* B-slice: L1 references at canvas index 16 */
				amvdec_write_dos(core,
					HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR,
					(16 << 8) | (0 << 1) | 1);
				for (i = 0; i < hevc->ref_num_l1; i++) {
					int buf_idx = codec_hevc_find_buf_index(
						hevc,
						hevc->ref_poc_list1[i]);
					u32 canvas_val;

					if (buf_idx < 0) {
						buf_idx = hevc->prev_frame ?
							hevc->prev_frame->index : 0;
					}
					canvas_val = (buf_idx << 16) |
						     (buf_idx << 8) | buf_idx;
					amvdec_write_dos(core,
						HEVCD_MPP_ANC_CANVAS_DATA_ADDR,
						canvas_val);
				}
			}
		}
	}

	codec_hevc_set_sao(sess, hevc->cur_frame->index,
			   hevc->cur_frame->vbuf ?
			   &hevc->cur_frame->vbuf->vb2_buf : NULL,
			   hevc->new_pic);
	codec_hevc_set_mpred(core, hevc);
	codec_hevc_set_mcrcc(sess);

	/* Update DBLK config */
	amvdec_write_dos(core, HEVC_DBLK_CFG4,
			 hevc->workspace_paddr + DBLK_PARA_OFFSET);
	amvdec_write_dos(core, HEVC_DBLK_CFG5,
			 hevc->workspace_paddr + DBLK_DATA_OFFSET);
	if (core->platform->revision >= VDEC_REVISION_G12A)
		amvdec_write_dos(core, HEVC_DBLK_CFGE,
				 hevc->workspace_paddr + DBLK_DATA2_OFFSET);

	/* SAO picture size and LCU count (BSP config_sao_hw) */
	amvdec_write_dos(core, HEVC_SAO_PIC_SIZE,
			 hevc->width | (hevc->height << 16));
	amvdec_write_dos(core, HEVC_SAO_PIC_SIZE_LCU,
			 (lcu_width - 1) | ((lcu_height - 1) << 16));

	/* SAO_CTRL0: read-modify-write, only set lcu_size_log2 in [3:0] */
	val = amvdec_read_dos(core, HEVC_SAO_CTRL0);
	val &= ~0xf;
	val |= lcu_size_log2;

	/* SAO_CTRL0 bits [9:8]: loop filter across tiles/slices (BSP) */
	data32_2 = val;
	data32_2 &= ~0x300;
	if (param->p.tiles_enabled_flag)
		data32_2 |= ((misc_flag0 >>
			      LOOP_FILER_ACROSS_TILES_ENABLED_FLAG_BIT) & 1)
			     << 8;

	slice_deblocking_filter_disabled = (misc_flag0 >>
		SLICE_DEBLOCKING_FILTER_DISABLED_FLAG_BIT) & 1;

	if ((misc_flag0 & BIT(PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED_FLAG_BIT))
	    && ((misc_flag0 & BIT(SLICE_SAO_LUMA_FLAG_BIT))
		|| (misc_flag0 & BIT(SLICE_SAO_CHROMA_FLAG_BIT))
		|| !slice_deblocking_filter_disabled)) {
		data32_2 |= ((misc_flag0 >>
			      SLICE_LOOP_FILTER_ACROSS_SLICES_ENABLED_FLAG_BIT)
			     & 1) << 9;
	} else {
		data32_2 |= ((misc_flag0 >>
			      PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED_FLAG_BIT)
			     & 1) << 9;
	}
	amvdec_write_dos(core, HEVC_SAO_CTRL0, data32_2);

	/* DBLK_CFG2: picture dimensions (BSP config_sao_hw) */
	amvdec_write_dos(core, HEVC_DBLK_CFG2,
			 hevc->width | (hevc->height << 16));

	/* DBLK_CFG1: LCU size + QP offsets + PCM filter (BSP config_sao_hw) */
	if ((misc_flag0 >> PCM_ENABLE_FLAG_BIT) & 1)
		cfg1 = ((misc_flag0 >> PCM_LOOP_FILTER_DISABLED_FLAG_BIT) & 1)
		       << 3;
	else
		cfg1 = 0;
	cfg1 |= (param->p.pps_cb_qp_offset & 0x1f) << 4;
	cfg1 |= (param->p.pps_cr_qp_offset & 0x1f) << 9;
	cfg1 |= (lcu_size == 64) ? 0 : ((lcu_size == 32) ? 1 : 2);
	if (hevc->width <= 64)
		cfg1 |= BIT(20);
	amvdec_write_dos(core, HEVC_DBLK_CFG1, cfg1);

	/* G12A+: select dblk_main pipeline (BSP config_sao_hw).
	 * STS1+4 (0xd434) = undocumented DBLK pipeline select register.
	 * BIT(28) enables the HEVC dblk_main decode path.
	 */
	if (core->platform->revision >= VDEC_REVISION_G12A)
		amvdec_write_dos(core, HEVC_DBLK_STS1 + 4, BIT(28));

	/* DBLK_CFG9: per-slice deblocking params (BSP config_sao_hw) */
	cfg9 = 0;
	if (param->p.tiles_enabled_flag)
		cfg9 |= ((misc_flag0 >>
			  LOOP_FILER_ACROSS_TILES_ENABLED_FLAG_BIT) & 1);

	if ((misc_flag0 & BIT(DEBLOCKING_FILTER_OVERRIDE_ENABLED_FLAG_BIT))
	    && (misc_flag0 & BIT(DEBLOCKING_FILTER_OVERRIDE_FLAG_BIT))) {
		cfg9 |= slice_deblocking_filter_disabled << 2;
		if (!slice_deblocking_filter_disabled) {
			cfg9 |= (param->p.slice_beta_offset_div2 & 0xf) << 3;
			cfg9 |= (param->p.slice_tc_offset_div2 & 0xf) << 7;
		}
	} else {
		cfg9 |= ((misc_flag0 >>
			  PPS_DEBLOCKING_FILTER_DISABLED_FLAG_BIT) & 1) << 2;
		if (!((misc_flag0 >> PPS_DEBLOCKING_FILTER_DISABLED_FLAG_BIT)
		      & 1)) {
			cfg9 |= (param->p.pps_beta_offset_div2 & 0xf) << 3;
			cfg9 |= (param->p.pps_tc_offset_div2 & 0xf) << 7;
		}
	}

	if ((misc_flag0 & BIT(PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED_FLAG_BIT))
	    && ((misc_flag0 & BIT(SLICE_SAO_LUMA_FLAG_BIT))
		|| (misc_flag0 & BIT(SLICE_SAO_CHROMA_FLAG_BIT))
		|| !slice_deblocking_filter_disabled)) {
		cfg9 |= ((misc_flag0 >>
			  SLICE_LOOP_FILTER_ACROSS_SLICES_ENABLED_FLAG_BIT)
			 & 1) << 1;
	} else {
		cfg9 |= ((misc_flag0 >>
			  PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED_FLAG_BIT)
			 & 1) << 1;
	}
	amvdec_write_dos(core, HEVC_DBLK_CFG9, cfg9);

	dev_dbg(core->dev,
		"HEVC setup: %ux%u CFGB=0x%x SAO_Y=0x%08x CTRL1=0x%x CTRL0=0x%x PC=0x%x\n",
		hevc->width, hevc->height,
		amvdec_read_dos(core, HEVC_DBLK_CFGB),
		amvdec_read_dos(core, HEVC_SAO_Y_START_ADDR),
		amvdec_read_dos(core, HEVC_SAO_CTRL1),
		amvdec_read_dos(core, HEVC_SAO_CTRL0),
		amvdec_read_dos(core, HEVC_MPC_E));

	/* Signal firmware to decode via INT2 handler (status 5 path).
	 * The INT2 handler calls sao_config which configures internal SAO
	 * hardware registers, then starts decode via CR 0x0114.
	 *
	 * The INT2 handler masks INT1 on entry (MCPU_INTR_MSK = 0xFFFFFFFE).
	 * We must re-enable INT1 from the host so the decode completion
	 * interrupt (INT1 at 0x02aa) can fire during or after decode.
	 */
	/* Signal firmware to decode via INT2 handler.
	 * The INT2 handler at 0x00f8 processes CODED_SLICE_SEGMENT_DAT:
	 *   - Calls sao_config (configures internal SAO/DBLK HW registers)
	 *   - Starts decode engine (CR 0x0114)
	 *   - Enters CTB processing loop
	 *   - When all CTBs done: INT1 fires, calls wait_decode_done
	 *     (flushes SAO/DBLK pipeline for last CTB row)
	 *   - Sends DECPIC_DATA_DONE or NAL_SEARCH_DONE
	 *
	 * We must unmask INT1 (bit 1) so the decode completion handler
	 * can fire during the CTB processing.
	 */
	{
		u32 msk = amvdec_read_dos(core, HEVC_MCPU_INTR_MSK);
		dev_dbg(core->dev,
			"Pre-signal: INTR_MSK=0x%x PC=0x%x\n",
			msk, amvdec_read_dos(core, HEVC_MPC_E));
	}
	amvdec_write_dos(core, HEVC_DEC_STATUS_REG,
			 HEVC_CODED_SLICE_SEGMENT_DAT);
	/* MCPU_INTR_MSK = ~(BIT(1)|BIT(2)): unmask INT1 (decode done)
	 * and INT2 (slice segment). BIT(1)=INT1 mask, BIT(2)=INT2 mask.
	 * Both must be unmasked for the firmware's CTB processing loop.
	 */
	amvdec_write_dos(core, HEVC_MCPU_INTR_MSK, 0xFFFFFFF9);
	amvdec_write_dos(core, HEVC_MCPU_INTR_REQ, AMRISC_MAIN_REQ);

}

static void codec_hevc_resume(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_hevc *hevc = sess->priv;
	int ret;

	if (!hevc)	/* N16b: resume callback on a torn-down session */
		return;

	dev_dbg(core->dev, "HEVC resume called, %ux%u\n",
		sess->width, sess->height);

	mutex_lock(&hevc->lock);

	/*
	 * The MPRED MV slot array was sized at start() from what S_FMT
	 * declared.  If the SPS turned out larger, grow it now -- before any
	 * decode at the new resolution programs an MV write window past the
	 * end of its slot.  No-op unless the stream genuinely exceeds the
	 * sized-for resolution.
	 */
	if (codec_hevc_grow_workspace(sess, hevc)) {
		mutex_unlock(&hevc->lock);
		amvdec_abort(sess);
		return;
	}

	/* Set up canvas/buffer infrastructure for new resolution */
	if (hevc_needs_compress_ref(core, hevc->is_10bit)) {
		/* codec_hevc_alloc_compress_bufs() does a per-DPB-buffer
		 * contiguous dma_alloc_coherent of amvdec_am21c_size() (e.g.
		 * ~1.5 MB / 368 pages at 720p).  On the small 256 MB GXL CMA
		 * this fails with -ENOMEM once the pool fragments after a few
		 * decodes.  If we ignore it, fbc_buffer_paddr[] stays 0 and
		 * codec_hevc_setup_compress_anc2axi() programs physical
		 * address 0 into the ANC2AXI table -- the HEVC engine then
		 * DMAs reference frames to address 0, bus-faults, and the SoC
		 * hard-resets (no CPU oops; netconsole-captured, KI-113).
		 * Fail the decode cleanly instead.
		 */
		ret = codec_hevc_alloc_compress_bufs(sess, &hevc->common);
		if (ret) {
			dev_err(core->dev,
				"HEVC resume: compress buf alloc failed (%d), aborting session (KI-113)\n",
				ret);
			mutex_unlock(&hevc->lock);
			/* amvdec_abort() halts the firmware (MPSR=0) and errors the
			 * vb2 queues.  Halting is essential: an ACK-and-continue
			 * leaves the firmware running against unallocated reference
			 * buffers, which bus-faults and resets the SoC a few frames
			 * later instead of here.
			 */
			amvdec_abort(sess);
			return;
		}
		codec_hevc_setup_compress_anc2axi(sess, &hevc->common);
	} else {
		ret = codec_hevc_setup_buffers(sess, &hevc->common, hevc->is_10bit);
		if (ret) {
			dev_err(core->dev,
				"HEVC resume: buffer setup failed (%d), aborting session\n",
				ret);
			mutex_unlock(&hevc->lock);
			amvdec_abort(sess);
			return;
		}
	}

	/*
	 * Clamp the M2 internal decode-slot count to the number of backing
	 * buffers codec_hevc_setup_buffers() actually allocated (one internal
	 * NV12/FBC/MMU-header slot per CAPTURE buffer, i.e. num_dst_bufs bounded
	 * by MAX_REF_PIC_NUM).  dpb_size was computed from the SPS
	 * (max_dec_pic_buffering + margin + M2 headroom) BEFORE REQBUFS, so a
	 * client that allocated fewer CAPTURE buffers than the firmware DPB would
	 * otherwise let codec_hevc_get_new_frame() hand out a slot with no backing
	 * internal buffer -> decode into an unprogrammed slot + a frame that can
	 * never be staged/recycled.  Mirrors codec_h264_resume()'s clamp of the
	 * firmware DPB to the canvased-buffer count (23eb61ae).  Identity when the
	 * client allocated >= dpb_size buffers (the normal case: min_buf_capture
	 * == dpb_size is advertised, and MMAP queues them all).
	 */
	if (hevc->common.nv12_internal_vaddr[0] && sess->num_dst_bufs) {
		u32 pool = min_t(u32, sess->num_dst_bufs, MAX_REF_PIC_NUM);

		if (hevc->dpb_size > pool) {
			dev_dbg(core->dev,
				"HEVC resume: clamp M2 slots dpb_size %u -> %u (num_dst=%u)\n",
				hevc->dpb_size, pool, sess->num_dst_bufs);
			hevc->dpb_size = pool;
		}
	}

	/* Write MMU map address after setup_buffers allocates it */
	if (codec_hevc_use_mmu(core->platform->revision, sess->pixfmt_cap,
			       hevc->is_10bit))
		codec_hevc_write_mmu_map_addr(core,
					      hevc->common.mmu_map_paddr);

	/* Restore DECODE_SIZE before signaling the firmware.
	 * The BUFEMPTY handler clears DECODE_SIZE to 0 to stop ISR
	 * floods. When DECODE_SIZE == 0, the firmware's completion_check
	 * always returns "not done" and DECPIC_DATA_DONE is blocked.
	 * The pre_queue callback normally restores this, but between
	 * src_change and resume, no buffers are queued.
	 */
	amvdec_write_dos(core, HEVC_DECODE_SIZE, 0x7FFFFFFF);

	/* Decode the pending frame that triggered the resolution change. */
	codec_hevc_compute_poc(hevc);
	dev_dbg(core->dev, "HEVC resume: POC=%d, getting frame\n",
		 hevc->curr_poc);

	hevc->new_pic = 1;
	hevc->slice_idx = 0;

	hevc->cur_frame = codec_hevc_get_new_frame(sess);
	if (!hevc->cur_frame) {
		dev_err(core->dev, "No frame available for HEVC resume\n");
		amvdec_write_dos(core, HEVC_DEC_STATUS_REG,
				 HEVC_ACTION_DONE);
		mutex_unlock(&hevc->lock);
		return;
	}
	codec_hevc_tag_frame(hevc, hevc->cur_frame);

	codec_hevc_setup_decode(sess);

	mutex_unlock(&hevc->lock);
}

static void codec_hevc_show_frame(struct amvdec_session *sess)
{
	struct codec_hevc *hevc = sess->priv;
	struct hevc_frame *tmp, *n;

	list_for_each_entry_safe(tmp, n, &hevc->ref_frames_list, list) {
		if (tmp->done || !tmp->show)
			continue;

		/* M2: stage the slot into a vb2 first (skip if none free). */
		if (!codec_hevc_stage_output(sess, tmp))
			continue;
		codec_hevc_dst_buf_done(sess, tmp);
		tmp->done = 1;
		hevc->frames_num--;
		list_del(&tmp->list);
		kfree(tmp);
	}
}

static irqreturn_t codec_hevc_threaded_isr(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_hevc *hevc = sess->priv;
	u32 dec_status;

	if (!hevc)
		return IRQ_HANDLED;

	mutex_lock(&hevc->lock);
	dec_status = hevc->dec_status;

	if (dec_status) {
		dev_dbg(core->dev,
			"HEVC ISR: st=0x%x DS=0x%x PC=0x%x\n",
			dec_status,
			amvdec_read_dos(core, HEVC_DECODE_SIZE),
			amvdec_read_dos(core, HEVC_MPC_E));
	}

	/*
	 * KI-061 N15: force the trailer into the VIFIFO EARLY -- while picture
	 * N is still being decoded -- so the firmware finds the trailer's start
	 * code as the delimiter WITHOUT a search-underrun, i.e. it completes
	 * picture N through its normal path (bit-exact), never the 0x22
	 * SEARCH_BUFEMPTY park whose KI-115 DECODE_SIZE=0 both suppresses the
	 * 0x0a completion IRQ and leaves the last picture +-1 LSB off.  gst is
	 * deterministically bit-exact precisely because its pipeline feeds the
	 * trailer mid-decode; the shim's DEC_CMD_STOP feeds it late (at drain),
	 * losing the race ~40% of runs.  Scheduling the esparser feed here, on
	 * the first ISR after should_stop, replicates gst's early feed:
	 * resume_after_feed injects the stashed slice as normal continuing
	 * input before the firmware exhausts picture N.  One-shot
	 * (!trailer_injected); the flush_output drain schedule remains as a
	 * backstop, and Fix B recovers the frame if a park still slips through.
	 */
	if (!READ_ONCE(sess->aborted) &&
	    READ_ONCE(sess->should_stop) && hevc->trailer_data &&
	    !hevc->trailer_injected &&
	    !v4l2_m2m_num_src_bufs_ready(sess->m2m_ctx))
		schedule_work(&sess->esparser_queue_work);	/* N16b: not once aborting */

	/*
	 * Bucket EVERY threaded-ISR status, so irq_counters RECONCILES against
	 * `total`.  The two hardirq-only statuses (0x22/0x24 and 0x20) are
	 * counted in codec_hevc_isr() and never reach here; everything else
	 * reaches here exactly once per interrupt, so
	 *
	 *   total = search_bufempty + bufempty + slice_done + decpic_done
	 *         + nal_decode_done + other_status
	 *
	 * and a leftover is a real event (an ISR that returned before this
	 * point), not an uncounted status.  hevc_slice_done is incremented in
	 * its own case body below, where it has been since N112; this switch
	 * only adds the arms that had no bucket at all.  A classifier rather
	 * than an increment per case body: one contiguous block is cheaper to
	 * read and cannot drift out of sync with the arithmetic above it.
	 */
	switch (dec_status) {
	case HEVC_SLICE_SEGMENT_DONE:
	case HEVC_HEAD_PARSER_DONE:
		break;			/* counted in the case body below */
	case HEVC_DECPIC_DATA_DONE:
		atomic_inc(&core->debugfs.irq.hevc_decpic_done);
		break;
	case HEVC_NAL_DECODE_DONE:
		atomic_inc(&core->debugfs.irq.hevc_nal_decode_done);
		break;
	default:
		atomic_inc(&core->debugfs.irq.hevc_other_status);
		break;
	}

	switch (dec_status) {
	case HEVC_SLICE_SEGMENT_DONE:
	case HEVC_HEAD_PARSER_DONE:
		if (dec_status == HEVC_HEAD_PARSER_DONE)
			atomic_inc(&core->debugfs.irq.hevc_head_parser_done);
		/*
		 * The usable progress signal.  0xf0 is a NAL-search-mode status
		 * that DECODE_MODE_SINGLE never produces, so the counter above
		 * reads 0 even on a decode that delivers every frame; this one
		 * counts the arm the firmware actually takes.
		 */
		atomic_inc(&core->debugfs.irq.hevc_slice_done);
		dev_dbg(core->dev, "HEVC SLICE/HEAD: st=0x%x\n", dec_status);
		sess->keyframe_found = 1;

		codec_hevc_fetch_rpm(sess);
		codec_hevc_compute_poc(hevc);

		dev_dbg(core->dev,
			"HEVC rpm: %ux%u slice_type=%u POClsb=%u POC=%d seg_addr=%u tmvp=%u bit_depth=0x%x is_10bit=%d\n",
			 hevc->rpm_param.p.pic_width_in_luma_samples,
			 hevc->rpm_param.p.pic_height_in_luma_samples,
			 hevc->rpm_param.p.slice_type,
			 hevc->rpm_param.p.POClsb,
			 hevc->curr_poc,
			 hevc->rpm_param.p.slice_segment_address,
			 hevc->rpm_param.p.slice_temporal_mvp_enable_flag,
			 hevc->bitdepth_offset >= 0 ?
				hevc->rpm_param.l.data[hevc->bitdepth_offset] : 0,
			 hevc->is_10bit);

		if (codec_hevc_process_rpm(hevc)) {
			/* DPB size from SPS: max_dec_pic_buffering + 2.
			 * The extra buffer ensures get_new_frame() always
			 * finds a non-referenced buffer when ref_held
			 * protects reference FBC bodies.
			 */
			u32 dpb = hevc->rpm_param.p.sps_max_dec_pic_buffering_minus1_0 + 2;
			if (dpb < 5)
				dpb = 5;  /* 1 decode + 1 ref + 1 output + 2 margin */
			/* M2 (NV12+MMU): add output-lag HEADROOM.  The internal
			 * decode slots and the vb2 output buffers are both sized to
			 * this count.  A buffer-holding consumer can hold several
			 * vb2s while the decoder keeps decoding ahead into spare
			 * internal slots; the extra buffers give that head start so
			 * stage_output always finds a free vb2 and the slots drain.
			 * (This is why M2 + headroom works where the falsified plain
			 * deeper-DPB did not: decode and output are now decoupled.) */
			if (codec_hevc_is_nv12(sess->pixfmt_cap) &&
			    codec_hevc_use_mmu(core->platform->revision,
					       sess->pixfmt_cap, hevc->is_10bit))
				dpb += 6;
			if (dpb > MAX_REF_PIC_NUM)
				dpb = MAX_REF_PIC_NUM;
			hevc->dpb_size = dpb;  /* M2 internal-slot count */
			hevc->cur_frame = NULL;
			/* Drop the lock before amvdec_src_change because its
			 * fast-resume path may call codec_hevc_resume() which
			 * also takes hevc->lock -- deadlock if we hold it.
			 */
			mutex_unlock(&hevc->lock);
			amvdec_src_change(sess, hevc->width, hevc->height,
					  dpb);
			/* Don't signal firmware -- leave it paused until
			 * resume() is called after capture buffers are set up.
			 * Lock is already released, return directly.
			 */
			return IRQ_HANDLED;
		}

		if (!hevc->width || !hevc->height) {
			/* Resolution not yet known, ack and wait */
			amvdec_write_dos(core, HEVC_DEC_STATUS_REG,
					 HEVC_ACTION_DONE);
			goto unlock;
		}

		/*
		 * KI-144 capability gate: this driver does not implement HEVC
		 * DEPENDENT slice segments.  KI-053 designed and validated the
		 * multi-slice path for INDEPENDENT slices only -- its own test
		 * plan deferred dependent segments as "rare in practice" and
		 * that item was never executed.  Three pieces are missing
		 * versus the BSP (vh265.c): the containing independent slice's
		 * address (BSP's slice_addr, updated only when
		 * !dependent_slice_segment_flag), per-slice_idx reference
		 * lists (BSP's m_aiRefPOCList0[slice_idx][]), and the
		 * three-way MPRED pointer policy (new_pic -> reset all;
		 * independent continuation -> update MV_RPTR; dependent ->
		 * leave alone).  We have none of them.
		 *
		 * Programming the hardware for a dependent segment anyway
		 * hangs the decode engine: the firmware stops responding, no
		 * further IRQ arrives, the kernel degrades over the next few
		 * seconds and the SoC watchdog resets the whole board ~30 s
		 * later with no oops, no panic and no printk.  Confirmed on
		 * SM1 and G12B with DSLICE_A_HHI_5, DSLICE_C_HHI_5, WPP_A and
		 * CAINIT_G; the matched negative control SLICES_A_Rovi_3 (20
		 * genuinely independent slices, zero dependent) is clean, and
		 * 6000 independent slices decode fine -- so this gate keys on
		 * the one thing that actually distinguishes them.
		 *
		 * Reject cleanly BEFORE codec_hevc_setup_decode() hands the
		 * segment to the engine, exactly as codec_vp9_threaded_isr()
		 * does for a non-8-aligned VP9 coded size (N18).  The board
		 * survives and userspace gets an error instead of a reboot.
		 * Full dependent-segment support is the larger BSP-parity
		 * project behind this gate.
		 *
		 * The warn is one-shot: a stream can carry many dependent
		 * segments per picture, and an unthrottled print on the HEVC
		 * threaded-ISR path is a synchronous netconsole UDP write that
		 * is itself destabilising (it once took a run from 486
		 * pictures to 125).
		 */
		if (hevc->rpm_param.p.dependent_slice_segment_flag) {
			dev_warn_once(core->dev,
				      "HEVC dependent slice segments are not supported by this driver; aborting session (KI-144)\n");
			amvdec_abort(sess);
			goto unlock;
		}

		if (hevc->rpm_param.p.slice_segment_address == 0) {
			/* === NEW PICTURE (first slice) === */
			hevc->new_pic = 1;
			hevc->slice_idx = 0;

			/* Cache sps_num_reorder_pics for the bumping decision.
			 * RPM is parsed per-slice; cache on first-slice so the
			 * value is stable for the rest of the picture.
			 */
			hevc->sps_num_reorder_pics_0 =
				hevc->rpm_param.p.sps_num_reorder_pics_0;
			dev_dbg(core->dev,
				"HEVC reorder: num_reorder_pics=%u max_dec_pic_buf=%u dpb_size=%u\n",
				hevc->sps_num_reorder_pics_0,
				hevc->rpm_param.p.sps_max_dec_pic_buffering_minus1_0 + 1,
				hevc->dpb_size);

			codec_hevc_apply_rps(hevc);

			/* Mark previous picture as decode-complete and bump:
			 * release the lowest-POC frame if we have more buffered
			 * than sps_num_reorder_pics_0 frames (HEVC Annex C DPB
			 * bumping, mirrors BSP vh265.c:output_pic()).
			 */
			if (hevc->cur_frame) {
				hevc->cur_frame->output_ready = 1;
				hevc->prev_frame = hevc->cur_frame;
				/* prev picture fully decoded -- bump progress */
				WRITE_ONCE(hevc->pics_decoded,
					   hevc->pics_decoded + 1);
				/* The new picture's RPM is already fetched: an
				 * IDR/BLA outputs every waiting picture first
				 * (C.5.2.2, KI-175).  Anything not delivered
				 * now still precedes the new sequence through
				 * the (cvs, poc) order.
				 */
				codec_hevc_bump_frames(sess,
						codec_hevc_next_starts_cvs(sess));
			}

			/* Recycle done frames no longer needed as refs */
			{
				struct hevc_frame *tmp, *n;

				list_for_each_entry_safe(tmp, n,
						&hevc->ref_frames_list,
						list) {
					if (!tmp->done)
						continue;
					if (test_bit(tmp->index,
						     &hevc->ref_held))
						continue;
					/* KI-143: see codec_hevc_action_done_work
					 * -- the frame is still live for the
					 * continuation slices of a multi-slice
					 * picture, and prev_frame is still read
					 * by setup_decode(). */
					if (tmp == hevc->cur_frame ||
					    tmp == hevc->prev_frame)
						continue;
					list_del(&tmp->list);
					kfree(tmp);
				}
			}

			hevc->cur_frame =
				codec_hevc_get_new_frame(sess);
			if (!hevc->cur_frame &&
			    hevc->common.nv12_internal_vaddr[0]) {
				/* M2: internal decode slots exhausted.  Drain
				 * output -- stage decode-complete frames into
				 * free vb2s + recycle done/unreferenced frames --
				 * to free slots, then retry once.  At exhaustion
				 * there are usually free vb2s (a holding consumer
				 * cannot hold all dpb+6 buffers), so this recovers
				 * immediately and paces decode to the consumer's
				 * drain rate instead of deadlocking (the unpaced
				 * fast-consumer case). */
				struct hevc_frame *tmp, *n;

				codec_hevc_bump_frames(sess, false);
				list_for_each_entry_safe(tmp, n,
						&hevc->ref_frames_list, list) {
					if (tmp->done &&
					    !test_bit(tmp->index,
						      &hevc->ref_held)) {
						/* KI-143: still live -- see
						 * codec_hevc_action_done_work. */
						if (tmp == hevc->cur_frame ||
						    tmp == hevc->prev_frame)
							continue;
						list_del(&tmp->list);
						kfree(tmp);
					}
				}
				hevc->cur_frame =
					codec_hevc_get_new_frame(sess);
			}
			if (!hevc->cur_frame) {
				dev_dbg(core->dev,
					"HEVC: no dst buf for POC=%d, deferring\n",
					hevc->curr_poc);
				schedule_delayed_work(
					&hevc->action_done_work,
					msecs_to_jiffies(10));
				goto unlock;
			}
			codec_hevc_tag_frame(hevc, hevc->cur_frame);
		} else {
			/* === CONTINUATION SLICE === */
			hevc->new_pic = 0;
			hevc->slice_idx++;

			if (!hevc->cur_frame) {
				dev_dbg(core->dev,
					"HEVC: continuation slice but no cur_frame, skipping\n");
				amvdec_write_dos(core,
						 HEVC_DEC_STATUS_REG,
						 HEVC_ACTION_DONE);
				goto unlock;
			}

			/* Multi-slice picture: the eager per-DECPIC output path
			 * (HEVC_DECPIC_DATA_DONE below) may have marked cur_frame
			 * output_ready after its first slice.  The picture is NOT
			 * complete until its last slice, so re-clear it here.
			 * Only the final slice's DECPIC_DATA_DONE -- with no
			 * continuation slice following -- leaves output_ready set
			 * for delivery.  (Guarded on !done: if the bump threshold
			 * was already exceeded and the frame was delivered, leave
			 * it; that rare case is a genuine multi-slice edge the
			 * fleet 1080p regression gate covers.)
			 */
			if (!hevc->cur_frame->done)
				hevc->cur_frame->output_ready = 0;
		}

		codec_hevc_setup_decode(sess);
		break;

	case HEVC_DECPIC_DATA_DONE:
	case HEVC_FIND_NEXT_PIC_NAL: {
		u32 y_wptr = amvdec_read_dos(core, HEVC_SAO_Y_WPTR);

		hevc->done_count = 0;
		hevc->last_sao_y_wptr = y_wptr;

		dev_dbg(core->dev,
			"HEVC DONE: st=0x%x Y_WPTR=0x%x Y_LEN=0x%x C_LEN=0x%x Y_START=0x%x C_START=0x%x CTRL1=0x%x\n",
			dec_status, y_wptr,
			amvdec_read_dos(core, HEVC_SAO_Y_LENGTH),
			amvdec_read_dos(core, HEVC_SAO_C_LENGTH),
			amvdec_read_dos(core, HEVC_SAO_Y_START_ADDR),
			amvdec_read_dos(core, HEVC_SAO_C_START_ADDR),
			amvdec_read_dos(core, HEVC_SAO_CTRL1));

		/*
		 * KI-061 N14: DECPIC_DATA_DONE means this picture's CTBs + SAO
		 * are fully written -- mark cur_frame decoded so an
		 * already-decoded but never-delimited last picture can be
		 * delivered at EOS (see codec_hevc_flush_output).
		 */
		if (hevc->cur_frame)
			hevc->cur_frame->decoded = 1;

		/* Dump MMU header and FBC body for first 3 frames */
		if (hevc->cur_frame && hevc->cur_frame->index < 3) {
			int idx = hevc->cur_frame->index;
			u32 *hdr = hevc->common.mmu_header_vaddr[idx];
			u32 *body = hevc->common.fbc_buffer_vaddr[idx];

			if (hdr)
				dev_dbg(core->dev,
					"MMU HDR[%d]: %08x %08x %08x %08x %08x %08x %08x %08x\n",
					idx, hdr[0], hdr[1], hdr[2], hdr[3],
					hdr[4], hdr[5], hdr[6], hdr[7]);
			if (body)
				dev_dbg(core->dev,
					"FBC BODY[%d]: %08x %08x %08x %08x %08x %08x %08x %08x\n",
					idx, body[0], body[1], body[2], body[3],
					body[4], body[5], body[6], body[7]);
		}

		/* Slice decode complete.  For multi-slice pictures the
		 * firmware sends DECPIC_DATA_DONE after each slice, not
		 * after the whole picture.
		 *
		 * Eager display bumping for reorder streams (B-frames): mark
		 * the just-decoded picture output_ready at its OWN
		 * DECPIC_DATA_DONE and bump now, instead of waiting for the
		 * next picture's first slice.  That +1-picture deferral is
		 * what starves the c2/CCodec incremental-feed backpressure
		 * loop: the client stops feeding after ~reorder+1 pictures
		 * because the outputs it waits for arrive one picture late, so
		 * the reorder DPB never drains and the last decoded frames are
		 * never delivered (c2 HEVC t265 capped at ~3-4 decoded / 1
		 * output).  Whole-stream MMAP does not hit this (it feeds the
		 * whole stream, decode runs ahead, the EOS flush-drain
		 * catches up); H.264 does not (firmware DPB, decode-order
		 * delivery, no reorder hold).
		 *
		 * Gated on sps_num_reorder_pics_0 > 0:
		 *  - IPPP (reorder == 0) is already delivered promptly by the
		 *    SLICE_SEGMENT_DONE(seg==0) path and does not starve;
		 *    marking a picture ready at its first-slice
		 *    DECPIC_DATA_DONE with no bump-threshold hold to protect
		 *    it would risk emitting a multi-slice picture early.
		 *  - For reorder > 0 the bump threshold holds a not-yet-lowest
		 *    (or stream-start) picture, so a multi-slice picture is
		 *    not delivered at its first slice; the continuation-slice
		 *    re-clear (SLICE_SEGMENT_DONE seg!=0, above) covers the
		 *    intermediate slices, and only the last slice leaves
		 *    output_ready set.
		 * Display order is preserved: codec_hevc_bump_frames() always
		 * emits the lowest-POC output_ready frame.  The next-picture
		 * SLICE_SEGMENT_DONE(seg==0) path still marks + bumps as the
		 * multi-slice-safe backstop (and the only path for
		 * reorder == 0); marking is idempotent.
		 *
		 * NOTE: Do NOT clear ref_held here. The RPS for the NEXT
		 * frame will clear/set bits appropriately in apply_rps().
		 */
		if (hevc->cur_frame && !hevc->cur_frame->output_ready &&
		    hevc->sps_num_reorder_pics_0 > 0) {
			hevc->cur_frame->output_ready = 1;
			codec_hevc_bump_frames(sess, false);
		}

		/* Defer ACTION_DONE by 1ms to break the tight IRQ loop.
		 * Without this delay, the firmware fires the next
		 * SLICE_SEGMENT_DONE immediately, and the threaded ISR
		 * processes frames back-to-back at RT priority, starving
		 * userspace from recycling capture buffers.
		 */
		schedule_delayed_work(&hevc->action_done_work,
				      msecs_to_jiffies(1));
		break;
	}

	case HEVC_NAL_DECODE_DONE: {
		/* NAL_DECODE_DONE (0xe): all bitstream data consumed.
		 * The firmware's completion_check saw consumed >=
		 * DECODE_SIZE and sent this status.
		 *
		 * Output any pending frame.  Then write ACTION_DONE
		 * to break the firmware's spin loop.  Do NOT halt the
		 * firmware -- it returns to its main loop and continues
		 * polling DEC_STATUS_REG.  DECODE_SIZE remains at the
		 * sentinel value (0x7FFFFFFF) set by pre_queue, so the
		 * next completion_check returns 0 (not done) and the
		 * firmware enters the multi-slice path harmlessly.
		 */
		if (hevc->cur_frame) {
			hevc->cur_frame->output_ready = 1;
			hevc->prev_frame = hevc->cur_frame;
			hevc->cur_frame = NULL;
			/* last picture fully decoded -- bump progress */
			WRITE_ONCE(hevc->pics_decoded,
				   hevc->pics_decoded + 1);
		}
		/* End of stream path: flush-drain the reorder buffer. */
		codec_hevc_bump_frames(sess, true);
		amvdec_write_dos(core, HEVC_DEC_STATUS_REG,
				 HEVC_ACTION_DONE);
		break;
	}

	case HEVC_OVER_DECODE:
		/* Firmware sends 0x0f (OVER_DECODE) and spins waiting for
		 * the driver to clear DEC_STATUS to 0. This happens when
		 * the firmware detects over-decode (decoded past expected
		 * boundary). ACK by writing 0 so firmware can continue.
		 */
		dev_dbg(core->dev, "HEVC OVER_DECODE\n");
		if (hevc->cur_frame) {
			hevc->cur_frame->output_ready = 1;
			hevc->prev_frame = hevc->cur_frame;
			hevc->cur_frame = NULL;
			/* last picture fully decoded -- bump progress */
			WRITE_ONCE(hevc->pics_decoded,
				   hevc->pics_decoded + 1);
		}
		/* End of stream path: flush-drain the reorder buffer. */
		codec_hevc_bump_frames(sess, true);
		amvdec_write_dos(core, HEVC_DEC_STATUS_REG, 0);
		break;

	case HEVC_DECPIC_DATA_ERROR:
		dev_dbg(core->dev, "HEVC decode error\n");
		amvdec_write_dos(core, HEVC_DEC_STATUS_REG, HEVC_ACTION_DONE);
		break;

	case HEVC_NAL_SEARCH_DONE:
		/* Firmware found next NAL header. ACK and let it continue.
		 * Do NOT fire AMRISC_MAIN_REQ -- firmware polls DEC_STATUS.
		 */
		dev_dbg(core->dev,
			"HEVC NAL_SEARCH: Y_WPTR=0x%08x Y_START=0x%08x SAO_CTRL3=0x%x\n",
			 amvdec_read_dos(core, HEVC_SAO_Y_WPTR),
			 amvdec_read_dos(core, HEVC_SAO_Y_START_ADDR),
			 amvdec_read_dos(core, HEVC_SAO_CTRL3));
		amvdec_write_dos(core, HEVC_DEC_STATUS_REG, HEVC_ACTION_DONE);
		break;

	case HEVC_DECODE_OVER_SIZE:
		/* Firmware sends 0x23 when decode exceeds expected size.
		 * This is a fatal error -- firmware enters infinite loop.
		 * Abort the session.
		 */
		dev_err(core->dev, "HEVC decode over size error\n");
		amvdec_abort(sess);
		break;

	case 0:
		break;

	default:
		dev_dbg(core->dev, "Unexpected HEVC status: %08X\n",
			dec_status);
		amvdec_write_dos(core, HEVC_DEC_STATUS_REG, HEVC_ACTION_DONE);
		break;
	}

unlock:
	mutex_unlock(&hevc->lock);
	return IRQ_HANDLED;
}

/*
 * KI-061 EOS trailer stash.  On the first source AU that carries a VCL slice,
 * copy that slice (from its start code to the end of the AU) so it can be
 * re-injected at true EOS to unstick the last picture (see the struct
 * codec_hevc trailer_data comment).  Parameter-set NALs (VPS/SPS/PPS/SEI,
 * nal_unit_type >= 32) are skipped so the re-injected trailer is slice-only
 * and cannot re-fire SEQ_READY / a spurious mid-drain SOURCE_CHANGE.  The
 * first AU of any decodable HEVC stream is an IRAP/IDR, so the stashed slice
 * is a real intra slice matching the active SPS/PPS -- exactly the trailer
 * shape the viability audit proved recovers the lost picture bit-exact.
 *
 * Best-effort: any failure (no VCL found yet, too large, alloc fail) simply
 * leaves trailer_data NULL, and the EOS recovery does not engage.
 */
static void codec_hevc_stash_trailer(struct codec_hevc *hevc,
				     const u8 *data, u32 size)
{
	u32 i;

	if (hevc->trailer_data || !data || size < 5)
		return;

	/* Scan for Annex-B start codes (00 00 01), classify each NAL. */
	for (i = 0; i + 3 < size; i++) {
		u32 nal_off, len;
		u8 nal_type;

		if (data[i] != 0x00 || data[i + 1] != 0x00 || data[i + 2] != 0x01)
			continue;

		nal_off = i + 3;			/* first NAL header byte */
		nal_type = (data[nal_off] >> 1) & 0x3f;
		if (nal_type >= 32)			/* non-VCL: VPS/SPS/PPS/SEI */
			continue;

		/* First VCL slice: stash from this start code to end of AU. */
		len = size - i;
		if (len > HEVC_TRAILER_MAX)
			return;			/* too large; leave disengaged */

		hevc->trailer_data = kmemdup(data + i, len, GFP_KERNEL);
		if (hevc->trailer_data)
			hevc->trailer_len = len;
		return;
	}
}

static int codec_hevc_pre_queue(struct amvdec_session *sess,
			       struct vb2_buffer *vb)
{
	struct amvdec_core *core = sess->core;
	struct codec_hevc *hevc = sess->priv;

	/*
	 * N16b: esparser_queue_all_src (this callback's caller) is
	 * re-scheduled from several codec paths (resume_after_feed, the
	 * BUFEMPTY ISR, the KI-061 wedge-drain recovery) and from the
	 * generic SOURCE_CHANGE handler, so it can fire AFTER
	 * vdec_stop_streaming() has freed and NULLed sess->priv.  Feature-
	 * stress streams driven through the m2m/VA-API shim (per-surface
	 * DEC_CMD_STOP) reliably hit this window: the wedge drain recovers
	 * the engine, then this callback runs with hevc==NULL.  Dereferencing
	 * it oopses *under sess->lock* (held by esparser_queue_all_src),
	 * leaking the mutex -> every waiter (the client ioctl + kworkers)
	 * goes uninterruptible D-state -> hung_task_panic reboots the SoC.
	 * That NULL-deref oops -- not a HW/firmware bus-hang -- is the N16
	 * "reset" (netconsole PC: codec_hevc_pre_queue+0x24, 2026-07-19).
	 * Bail cleanly: esparser_queue() errors this buffer (STATE_ERROR)
	 * and the feed loop stops.  Same guard codec_hevc_isr() and
	 * codec_hevc_num_pending_bufs() already carry.
	 */
	if (!hevc) {
		dev_warn_once(core->dev,
			      "N16b: HEVC pre_queue on torn-down session (priv=NULL); erroring buffer instead of oopsing\n");
		return -ENODEV;
	}

	/*
	 * KI-061: stash the first VCL slice for the EOS trailer inject.
	 * Runs under sess->lock (esparser_queue path); one-shot per session.
	 */
	if (!hevc->trailer_data)
		codec_hevc_stash_trailer(hevc, vb2_plane_vaddr(vb, 0),
					 vb2_get_plane_payload(vb, 0));

	/* Set DECODE_SIZE to a large sentinel value.  The firmware's
	 * completion_check compares consumed bytes against DECODE_SIZE
	 * and fires NAL_DECODE_DONE (0xe) when consumed >= DECODE_SIZE.
	 * Using a sentinel (0x7FFFFFFF) ensures completion_check never
	 * fires prematurely.
	 *
	 * DECODE_SIZE must be non-zero for two firmware code paths:
	 * 1. DECPIC_DATA_DONE (0xa): gated on DECODE_SIZE != 0
	 * 2. buf_empty_check: skips monitoring when DECODE_SIZE == 0
	 */
	amvdec_write_dos(core, HEVC_DECODE_SIZE, 0x7FFFFFFF);

	return 0;
}

static irqreturn_t codec_hevc_isr(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_hevc *hevc = sess->priv;
	u32 status;

	if (!hevc)
		return IRQ_HANDLED;

	status = amvdec_read_dos(core, HEVC_DEC_STATUS_REG);

	/* Handle BUFEMPTY in hardirq to avoid DEC_STATUS_REG race.
	 * If the threaded handler writes DEC_CONT for BUFEMPTY, it
	 * can overwrite a pending CODED_SLICE_SEGMENT_DAT before the
	 * firmware's INT2 handler processes it.  By responding here
	 * in the hardirq (before the threaded handler runs for any
	 * subsequent SLICE_SEGMENT_DONE), we ensure the firmware
	 * sees DEC_CONT while still in INT0, and then INT2 fires
	 * with the correct DEC_STATUS_REG value.
	 */
	/* KI-115 tail: the 0x22/0x24 SEARCH_BUFEMPTY/BUFEMPTY2 variant is NOT a
	 * source underrun -- bitstream data is present in the VIFIFO, but the
	 * firmware's empty-resume check (PC 0x0799) reads DECODE_SIZE as a live
	 * decodable-byte credit, and it spins for ACTION_DONE (0xff), not DEC_CONT.
	 * With DECODE_SIZE pinned 0x7FFFFFFF, acking 0xff routes the firmware to a
	 * FALSE NAL_DECODE_DONE (PC 0x07cb) -> premature frame + DPB corruption ->
	 * hard hang (the historical ack-only failure).  Writing DECODE_SIZE=0 first
	 * selects the re-arm branch (PC 0x07c9: DECODE_SIZE==0 -> 0x07d3): the
	 * firmware re-arms the shift engine, fetches the bytes already in the
	 * VIFIFO, and continues.  Two posted writes, one-shot -- the firmware then
	 * stops re-posting, so the cold-bus read-hammer that wedges the 0x20 spin
	 * (comment below) never starts.  Validated by the I1 sweep (full-clip
	 * decode, RD_PTR advances vs frozen baseline) -- see
	 * hevc/plan/ki115-tail-option1-completion-model.md "I1 RESULTS".
	 */
	if (status == HEVC_SEARCH_BUFEMPTY ||
	    status == HEVC_DECODE_BUFEMPTY2) {
		/*
		 * KI-061 N15: the DECODE_SIZE=0 write below suppresses picture
		 * N's DECPIC_DATA_DONE (0x0a) -- see hevc->eos_trailer_parked.
		 * Record that a 0x22/0x24 search-park happened so flush_output
		 * can still deliver the real, fully-decoded picture N (gated
		 * there on pics_at_inject, so setting this unconditionally here
		 * is race-free -- it does NOT depend on should_stop already
		 * being visible at this instant, which is the residual the
		 * earlier should_stop-gated version missed).  No new firmware
		 * poke; the KI-115 DECODE_SIZE=0 + 0xff re-arm below is unchanged.
		 */
		/*
		 * This arm returns IRQ_HANDLED without waking the threaded
		 * handler, so nothing downstream can record that it ran.  One
		 * atomic_inc beside two posted DOS writes is the cheapest
		 * always-on record there is, and without it a starved HEVC
		 * session is completely silent (N109: 30 such interrupts per
		 * decode with `dmesg | grep -i hevc` printing nothing at all).
		 */
		atomic_inc(&core->debugfs.irq.hevc_search_bufempty);
		WRITE_ONCE(hevc->eos_trailer_parked, true);
		amvdec_write_dos(core, HEVC_DECODE_SIZE, 0);
		amvdec_write_dos(core, HEVC_DEC_STATUS_REG, HEVC_ACTION_DONE);
		return IRQ_HANDLED;
	}

	if (status == HEVC_DECODE_BUFEMPTY) {
		/* Same invisibility as the 0x22/0x24 arm above: this one does not
		 * ack and does not wake the threaded handler either, so the bucket
		 * is the only trace a genuine VIFIFO underrun leaves.
		 */
		atomic_inc(&core->debugfs.irq.hevc_bufempty);
		/* KI-081: Do NOT clear DECODE_SIZE here.
		 * Firmware path 0x02b3 (g12a_hevc_mmu.asm): `beqi r8, 0, +11`
		 * skips DECPIC_DATA_DONE when DECODE_SIZE == 0.  P/B-frames
		 * trigger multiple BUFEMPTY events while the firmware processes
		 * NAL units; clearing DS during decode silently drops the frame.
		 * DECODE_SIZE=0 for teardown is handled in flush_output().
		 */
		/*
		 * KI-115 tail: 0x20 = genuine VIFIFO underrun (STREAM_LEVEL <= 512).
		 * Do NOT ack (DEC_CONT) here.  Acking re-arms the firmware, which
		 * re-checks the still-low VIFIFO (firmware path PC 0x0413) and
		 * re-posts BUFEMPTY -- the hardirq then read-hammers
		 * HEVC_DEC_STATUS_REG on the cold SM1 shared DOS bus until it
		 * hard-hangs (KI-115 tail wedge, ~38% on the first decode after a
		 * cold power-on).  Instead leave the firmware PAUSED (spinning
		 * internally at PC 0x0415 for 0xfd, posting nothing) and trigger an
		 * esparser feed.  codec_hevc_resume_after_feed() acks DEC_CONT (a
		 * posted write -- safe even on a fragile bus) only once the VIFIFO
		 * is re-filled above the underrun mark.  This bounds the hardirq to
		 * ONE read per underrun episode and lets the decode resume at the
		 * source-supply rate instead of stalling/wedging.  On true source
		 * starvation the firmware stays paused and the framework watchdog
		 * aborts cleanly (no IRQ -> abort), with the firmware idle so the
		 * abort hits a healthy bus.
		 */
		WRITE_ONCE(sess->bufempty_pending, true);
		if (!READ_ONCE(sess->aborted))	/* N16b: not once aborting */
			schedule_work(&sess->esparser_queue_work);
		return IRQ_HANDLED;
	}

	hevc->dec_status = status;
	return IRQ_WAKE_THREAD;
}

/*
 * Halt the HEVC firmware at end of drain, matching BSP amhevc_stop().
 * gxl_hevc_mmu.bin / g12a_hevc_mmu.bin sit "idle polling the VIFIFO
 * for more data" after the last NAL is consumed (see
 * codec_hevc_flush_output comment above the counter force-reset).
 * Leaving the firmware in that state wastes CPU cycles and leaves the
 * HEVC engine in an indeterminate idle vs. running state when the
 * next session starts.
 *
 * The counter force-reset remains in codec_hevc_flush_output (see
 * the atomic_set(&esparser_queued_bufs, 0) block there).  Moving it
 * here would add up to 5 s to every drain (vdec_drain_work's
 * wait_event_timeout on the counter) because HEVC firmware does not
 * fire late-EOS ISRs that would decrement it -- the counter stays
 * stuck at its post-BUFEMPTY value.  Leaving the force-reset where
 * it is keeps drain latency bounded.
 *
 * Called from vdec_drain_work() AFTER vdec_wait_inactive() and
 * flush_pending_frames().
 */
static void codec_hevc_drain_finalize(struct amvdec_session *sess)
{
	amvdec_hevc_halt_firmware(sess->core);
	sess->firmware_halted = 1;
}

/*
 * KI-115 tail: resume a firmware that codec_hevc_isr left PAUSED on a
 * VIFIFO-underrun BUFEMPTY, now that the esparser has (re-)fed source.  Called
 * from the esparser feed path.  Only acks once the VIFIFO data level is back
 * above the firmware's underrun mark (512 B) so the firmware takes its
 * "continue" branch instead of immediately re-posting BUFEMPTY.  The firmware
 * is idle (paused, not hammering), so the level read here is on a healthy bus;
 * the DEC_CONT ack is a posted write.  If still empty, stay paused -- a later
 * feed (or the watchdog) handles it.
 */
static void codec_hevc_resume_after_feed(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_hevc *hevc = sess->priv;

	/* N16b: also on the esparser_queue_all_src path -- tolerate a
	 * torn-down session (freed priv) that raced the feed work.
	 */
	if (!hevc)
		return;

	/*
	 * KI-061 EOS last-frame recovery.  This hook runs from the esparser
	 * feed path (esparser_queue_all_src) after every batch of source is
	 * pushed to the VIFIFO.  At true EOS -- DEC_CMD_STOP has set
	 * should_stop AND userspace has no more source buffers queued -- append
	 * the stashed first-AU VCL slice as a trailer, ONCE.  It lands in the
	 * VIFIFO right after the last real frame, so while the firmware is
	 * still actively decoding it reaches picture N, sees the trailer's
	 * start code as the next access unit, and delimits + completes picture
	 * N through the normal decode + BUFEMPTY (0x20/0x22) ISR path with real
	 * pixels (viability audit 2026-07-18: bit-exact recovery).  The trailer
	 * then becomes the new last-and-unterminated picture (a phantom
	 * cur_frame, output_ready==0) which codec_hevc_flush_output recycles --
	 * so the contract stays N-in/N-out.
	 *
	 * Feeding here (as normal continuing input, before the firmware
	 * stalls) rather than at drain avoids poking the firmware's fragile
	 * post-stall state -- the crash class that reverted option-A
	 * (8f481f2b2ea3) and drain-time inject variants.  Gated on should_stop
	 * so it never fires on the mid-stream gst SOURCE_CHANGE drain.
	 */
	if (READ_ONCE(sess->should_stop) && hevc->trailer_data &&
	    !hevc->trailer_injected &&
	    !v4l2_m2m_num_src_bufs_ready(sess->m2m_ctx)) {
		hevc->trailer_injected = true;
		/* KI-061 N15: snapshot pics_decoded (== N-1) so flush_output
		 * can tell whether picture N completed after this inject. */
		hevc->pics_at_inject = READ_ONCE(hevc->pics_decoded);
		dev_dbg(core->dev_dec,
			"HEVC KI-061: feeding %u-byte EOS trailer slice\n",
			hevc->trailer_len);
		esparser_queue_eos(core, hevc->trailer_data, hevc->trailer_len);
	}

	if (!READ_ONCE(sess->bufempty_pending))
		return;
	if (amvdec_read_dos(core, HEVC_STREAM_LEVEL) <= 512)
		return;
	WRITE_ONCE(sess->bufempty_pending, false);
	amvdec_write_dos(core, HEVC_DEC_STATUS_REG, HEVC_ACTION_DEC_CONT);
}

struct amvdec_codec_ops codec_hevc_ops = {
	.start = codec_hevc_start,
	.stop = codec_hevc_stop,
	.isr = codec_hevc_isr,
	.resume_after_feed = codec_hevc_resume_after_feed,
	.threaded_isr = codec_hevc_threaded_isr,
	.num_pending_bufs = codec_hevc_num_pending_bufs,
	.drain = codec_hevc_flush_output,
	.drain_finalize = codec_hevc_drain_finalize,
	.resume = codec_hevc_resume,
	.pre_queue = codec_hevc_pre_queue,
};

static int __init codec_hevc_mod_init(void)
{
	return amvdec_register_codec(V4L2_PIX_FMT_HEVC, &codec_hevc_ops);
}

static void __exit codec_hevc_mod_exit(void)
{
	amvdec_unregister_codec(V4L2_PIX_FMT_HEVC, &codec_hevc_ops);
}

module_init(codec_hevc_mod_init);
module_exit(codec_hevc_mod_exit);
MODULE_DESCRIPTION("Meson VDEC HEVC codec");
MODULE_LICENSE("GPL");
