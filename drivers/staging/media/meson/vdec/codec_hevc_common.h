/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2018 BayLibre, SAS
 * Author: Maxime Jourdan <mjourdan@baylibre.com>
 */

#ifndef __MESON_VDEC_HEVC_COMMON_H_
#define __MESON_VDEC_HEVC_COMMON_H_

#include "vdec.h"

/* Parser command skip configuration (BSP literal values).
 * These configure the parser's NAL unit skip/search behavior.
 * CFG_0: skip count thresholds, CFG_1/2: skip pattern masks.
 */
#define PARSER_CMD_SKIP_CFG_0 0x0000090b
#define PARSER_CMD_SKIP_CFG_1 0x1b14140f
#define PARSER_CMD_SKIP_CFG_2 0x001b1910

#define VDEC_HEVC_PARSER_CMD_LEN 37
extern const u16 vdec_hevc_parser_cmd[VDEC_HEVC_PARSER_CMD_LEN];

/* cma-opt-sg module param; set via /sys/module/meson_vdec_core/parameters/fbc_sg.
 * When true, FBC body allocations go through the scatter-gather path
 * (alloc_page + dma_map_page) instead of dma_alloc_coherent.  The HEVC
 * MMU translates via mmu_map so the contiguous-CMA budget no longer
 * constrains the probe-time max-resolution clamp.
 */
extern bool codec_hevc_fbc_sg;

/* Config-only-AU timestamp fix (see codec_hevc_common.c / esparser_queue). */
extern bool hevc_skip_config_ts;
bool codec_hevc_is_config_only(const u8 *data, u32 size);
bool codec_hevc_au_starts_cvs(const u8 *data, u32 size);

#define MAX_REF_PIC_NUM	24

/*
 * Shared SAO/decode EOS-drain completion signals (HEVC engine -- used by
 * both codec_hevc.c and codec_vp9.c, which share the SAO double-write
 * hardware).
 *
 * HEVC_SAO_IF_STATUS (0xc4c0) idle baseline -- reverse-engineered on
 * hardware (GXL/G12B/SM1 all idle at 0x00e00900; see
 * hevc/audit/sao-drain-2000-loop-timeout.md).  While the SAO/decode
 * engine is busy it sets additional bits OUTSIDE this baseline (G12B
 * observed 0x01300b00, i.e. bits 9/20/24); the value drops back to the
 * baseline exactly when the in-flight SAO write completes.  "SAO idle"
 * is therefore "no bit set outside the baseline" -- robust to
 * per-platform busy-bit variation: an unexpected busy bit keeps the
 * drain looping (safe) rather than exiting early into a half-flushed
 * pipeline (HEVC: frame truncation; VP9: bottom-LCU-row chroma).
 */
#define HEVC_SAO_IF_IDLE_VALUE			0x00e00900u
/* Consecutive idle polls required to declare the drain done (debounces
 * a brief between-picture engine-idle blip). */
#define HEVC_DRAIN_STABLE			4
/*
 * KI-131 progress-gated drain bounds (poll = usleep_range(50,100) ~= 75us).
 *
 * The EOS drain must let the SAO/decode engine finish its picture backlog no
 * matter how long a SINGLE picture takes: a 3840x2160 picture decodes in
 * >60 ms (~1000 polls, RE'd on SM1) and an 8K picture ~4x that.  Any fixed
 * wall-clock / loop-count cap that bounds the WHOLE drain therefore truncates
 * the tail while the engine is mid-decode -- HEVC_SAO_IF_STATUS still shows
 * busy bits at the bail (KI-131: the old 800-poll stall cap dropped 4K to
 * 2/30 bailing at if=0x100000; the old 8000-poll absolute cap dropped 1080p
 * to 25/30 bailing at if=0x1d01000).  Merely enlarging those caps re-breaks
 * at the next resolution up, so the bound is expressed as time WITHOUT
 * pics_decoded progress and reset on every advance: total drain then scales
 * with queue depth and is resolution-independent.
 *
 * The per-picture no-progress budget is the ONLY thing that must exceed a
 * single picture's decode time, and that time scales with picture AREA -- so
 * the budget is derived from width*height rather than fixed.  A fixed budget
 * is either too small (truncates large pictures) or, sized for 8K, absurdly
 * oversized for a 320x240 stream (would add ~1.2 s to every small-stream EOS,
 * since after the last picture the SAO IF status settles at a busy residual,
 * not the idle baseline, so the wedge guard -- not the idle-completion path --
 * is what ends the drain).  area/DIV approximates one picture's decode polls
 * (RE'd: 3840x2160 ~= 1000 polls => DIV 8192) and the budget is ~8x that
 * (area/1024), clamped:
 *   MIN  ~150 ms floor  -- below this a scheduler hiccup on a small picture
 *                          could be mistaken for a wedge.
 *   MAX  ~3.6 s ceiling -- bounds 8K+ (8K ~= 32000 polls unclamped) so the
 *                          per-picture wait stays bounded.
 * Worked budgets: 320x240 -> MIN(2000, ~150 ms); 1080p -> ~2000; 4K -> ~8100
 * (~600 ms); 8K -> MAX(48000, ~3.6 s).  Each is ~8x that resolution's measured
 * per-picture decode, so it never truncates a legitimate tail yet keeps the
 * common-case EOS drain short.
 *
 * ABS: absolute hard ceiling on TOTAL drain loops -- defensive bounded
 *   termination in case pics_decoded oscillates pathologically (a finite EOS
 *   queue cannot, but a firmware fault might, and an unbounded drain once hung
 *   a test harness for hours).  A deep decode-ahead backlog of large pictures
 *   drains in a few seconds; 131072 (~10 s) covers it with margin while
 *   guaranteeing the drain can never hang unbounded.
 */
#define HEVC_DRAIN_NOPROGRESS_DIV		1024u
#define HEVC_DRAIN_NOPROGRESS_MIN		2000u
#define HEVC_DRAIN_NOPROGRESS_MAX		48000u
#define HEVC_DRAIN_ABS_LOOPS			131072
/*
 * KI-061/N13 EOS trailer-recovery keep-alive.  Max polls the drain holds
 * open waiting for the racing esparser feed path (codec_hevc_resume_after_feed)
 * to inject the trailer + complete the real last picture N, before it gives up
 * and falls back to the pre-fix N-1 (never a hang).  The feed is scheduled from
 * the drain and normally lands in well under 100 ms; ~40000 polls (~3 s) is a
 * generous ceiling that tolerates workqueue/scheduler latency under load.
 */
#define HEVC_DRAIN_RECOVERY_LOOPS		40000u

/* Per-frame MMU header buffer size (one allocation per reference frame). */
#define MMU_COMPRESS_HEADER_SIZE 0x48000

/* Session-wide MMU map buffer size.  ALL allocators / freers of
 * codec_hevc_common::mmu_map_vaddr MUST use this size.  The pre-alloc
 * in codec_hevc_start (so the firmware can read SCRATCH_7 at init)
 * and the lazy alloc in codec_hevc_alloc_mmu_headers (after
 * SOURCE_CHANGE) write to the SAME pointer; mismatched sizes corrupt
 * adjacent CMA page refcounts (KI-107).
 */
#define MMU_MAP_SIZE 0x5000

/*
 * Scatter-gather FBC body descriptor.  When codec_hevc_fbc_sg module
 * param is enabled, FBC body pages are allocated individually via
 * alloc_page + dma_map_page instead of a single dma_alloc_coherent.
 * The HEVC MMU translates device-visible FBC virtual pages to physical
 * pages via the mmu_map table (codec_hevc_fill_mmu_map writes per-page
 * PFNs).  This removes the contiguous-CMA requirement and unlocks
 * higher resolutions on fragmented / small-CMA boards.
 *
 * Mutually exclusive with fbc_buffer_vaddr/paddr: sg mode leaves
 * those NULL/0, contig mode leaves pages[]/page_paddr[] NULL/0.
 * See cma-opt-sg-plan.md for the full design.
 */
struct codec_hevc_fbc_sg {
	struct page **pages;
	dma_addr_t  *page_paddr;
	u32	     nb_pages;
};

struct codec_hevc_common {
	void      *fbc_buffer_vaddr[MAX_REF_PIC_NUM];
	dma_addr_t fbc_buffer_paddr[MAX_REF_PIC_NUM];

	/* Driver-owned internal NV12 decode pool (M2 shadow-copy), indexed by
	 * decode slot.  The SAO double-write target + the ANC2AXI NV12 canvas
	 * point HERE, not at the vb2 capture buffer, so the decode slot is
	 * decoupled from the userspace vb2 buffer: get_new_frame allocates a
	 * slot (always available), and a vb2 is grabbed only at output, where
	 * the slot's NV12 is memcpy'd into it.  A buffer-holding downstream
	 * consumer then can't starve the decode pool (KI-061/KI-115 capture-
	 * exhaustion deadlock).  nv12_internal_size is the per-buffer byte size
	 * (output_size * 3/2) stored for the free path.  NULL on FBC-output /
	 * non-NV12 sessions (vb2 is the decode target there, unchanged).
	 */
	void      *nv12_internal_vaddr[MAX_REF_PIC_NUM];
	dma_addr_t nv12_internal_paddr[MAX_REF_PIC_NUM];
	u32        nv12_internal_size;

	/* Scatter-gather FBC body allocation (codec_hevc_fbc_sg param
	 * enabled).  Mutually exclusive with fbc_buffer_vaddr[] above.
	 */
	struct codec_hevc_fbc_sg fbc_sg[MAX_REF_PIC_NUM];

	void      *mmu_header_vaddr[MAX_REF_PIC_NUM];
	dma_addr_t mmu_header_paddr[MAX_REF_PIC_NUM];

	void      *mmu_map_vaddr;
	dma_addr_t mmu_map_paddr;

	/*
	 * Mirror of the is_10bit argument to codec_hevc_setup_buffers.
	 * Stored so free_fbc_buffers and fill_mmu_map can compute the
	 * correct FBC body size (3200 B/block 8-bit, 4096 B/block 10-bit)
	 * for dma_free_coherent and MMU page-count without plumbing
	 * is_10bit through every call site.  Set on alloc.
	 */
	bool is_10bit;

	/* BSP-protocol DMA buffers (Bucket 3 / hevc_bsp path).
	 * Allocated only when sess->fw_match->quirks &
	 * AMVDEC_FW_QUIRK_HEVC_BSP_PROTO.  Upstream-firmware sessions
	 * leave these fields NULL/0.  Accessed by codec_hevc_bsp.c via
	 * struct codec_hevc_common * passed in the session hooks.
	 */
	void      *bsp_rpm_vaddr;
	dma_addr_t bsp_rpm_paddr;
	void      *bsp_aux_vaddr;
	dma_addr_t bsp_aux_paddr;
};

/* Returns true if the capture fourcc is one of the AMLOGIC FBC
 * compressed-output formats.  In that mode the vb2 capture buffer
 * holds the FBC body directly (no SAO double-write to NV12).
 */
static inline bool codec_hevc_is_fbc_output(u32 pixfmt)
{
	return pixfmt == V4L2_PIX_FMT_MESON_FBC08_SCATTER ||
	       pixfmt == V4L2_PIX_FMT_MESON_FBC10_SCATTER;
}

/* Returns 1 if the decode pipeline uses framebuffer compression.
 * On G12A+ the HEVC engine always uses the compressed decode path
 * internally; the SAO double-write produces NV12 output alongside.
 * On GXBB/GXL, FBC is only used for 10-bit content.
 * When capture is one of the FBC-output fourccs, FBC is always used
 * regardless of bit depth (that's the whole point of the format).
 */
static inline int codec_hevc_use_fbc(u32 pixfmt, int is_10bit)
{
	return is_10bit || codec_hevc_is_fbc_output(pixfmt);
}

/* Returns 1 if we are decoding 10-bit but outputting 8-bit NV12.
 * FBC-output capture passes the compressed data through without
 * downsampling; the consumer (VPU VD1) or a separate downsample
 * block handles 10->8 if needed.
 */
static inline int codec_hevc_use_downsample(u32 pixfmt, int is_10bit)
{
	return is_10bit && !codec_hevc_is_fbc_output(pixfmt);
}

/* Returns 1 if we are decoding using the IOMMU.
 * The HEVC engine always requires MMU for its compressed decode
 * pipeline on all platforms (GXL through SM1+).  The BSP always
 * sets mmu_enable=1 for HEVC, even on GXL (vh265.c line 16456).
 * VP9 always uses FBC/MMU for reference frames.
 */
static inline int codec_hevc_use_mmu(u32 revision, u32 pixfmt, int is_10bit)
{
	return revision >= VDEC_REVISION_GXL;
}

/* Returns true if pixfmt uses NV12/NV21 layout (Y + interleaved UV/VU) */
static inline bool codec_hevc_is_nv12(u32 pixfmt)
{
	return pixfmt == V4L2_PIX_FMT_NV12M || pixfmt == V4L2_PIX_FMT_NV12 ||
	       pixfmt == V4L2_PIX_FMT_NV21M || pixfmt == V4L2_PIX_FMT_NV21;
}

/* Returns true if pixfmt needs UV swap (NV21 = VU byte order) */
static inline bool codec_hevc_is_nv21(u32 pixfmt)
{
	return pixfmt == V4L2_PIX_FMT_NV21M || pixfmt == V4L2_PIX_FMT_NV21;
}

/* Configure decode head read mode */
void codec_hevc_setup_decode_head(struct amvdec_session *sess, int is_10bit);

void codec_hevc_free_fbc_buffers(struct amvdec_session *sess,
				 struct codec_hevc_common *comm);

void codec_hevc_free_mmu_headers(struct amvdec_session *sess,
				 struct codec_hevc_common *comm);

int codec_hevc_setup_buffers(struct amvdec_session *sess,
			     struct codec_hevc_common *comm,
			     int is_10bit);

void codec_hevc_fill_mmu_map(struct amvdec_session *sess,
			     struct codec_hevc_common *comm,
			     int index, struct vb2_buffer *vb);

#endif
