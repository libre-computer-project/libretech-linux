// SPDX-License-Identifier: GPL-2.0
/*
 * MT8365 (Genio 350) stateless VP9 decode backend -- DIRECT REGISTER, no SCP,
 * no codec firmware, no vpud.
 *
 * Third codec on the mt8365 in-kernel direct-register path, after
 * vdec_h264_mt8365_if.c and vdec_vp8_mt8365_if.c. MT8365 has no codec
 * coprocessor: the VDEC is a stateless accelerator historically driven by the
 * closed userspace libvpud_vcodec.so. This backend replaces "send to VPU/SCP +
 * wait IPI" with "program the VDEC registers + kick + poll".
 *
 * SCOPE (this revision): KEY, INTRA-ONLY and INTER frames, profile 0, 8-bit
 * 4:2:0, one tile, WITH segmentation.
 *
 * THE SEGMENTATION INTERFACE (2026-07-26) -- recovered from the vendor's
 * VP9_Set_Segmentation / VP9_Set_UnCompressed / VP9_Set_SQT_Q_Table in
 * libvpud_vcodec.so, then CONFIRMED on a matched TRIPLE of 176x144 clips that
 * differ only in segmentation (libvpx -aq-mode 0 / 1 / 3):
 *
 *   VP9_VLD+0xa8 bit8/9/10   segmentation_enabled / _update_map /
 *                            _temporal_update
 *   VP9_VLD+0x1f0            tree_probs[0..3], little-endian
 *   VP9_VLD+0x1f4            tree_probs[4..6]
 *   VP9_VLD+0x1f8            pred_probs[0..2]
 *   VDEC_TOP+0x118 / +0x11c  the segment-id map surface, RAW physical address,
 *                            the SAME address in both -- one buffer serving as
 *                            both the previous-map source and the current-map
 *                            destination, which is why it must survive frame to
 *                            frame and be cleared by setup_past_independence()
 *   VDEC_TOP+0x124 low 16    = 0x10, the map prefetch size (already written)
 *   VDEC_PP+0xe4/0xe8/0xec   SEG_LVL_ALT_L: enable mask | abs_delta<<8, then
 *                            eight signed data bytes  [DERIVED, NOT MEASURED]
 *   VP9_VLD+0xac             SEG_LVL_SKIP enable mask (bits 7:0), the 2-bit
 *                            SEG_LVL_REF_FRAME datum per segment (bits 23:8)
 *                            and its enable mask (bits 31:24)
 *                            [DERIVED; only the SKIP mask is measurable in the
 *                            conformance corpus -- no VP9-TEST-VECTORS clip
 *                            codes ALT_L or REF_FRAME at all]
 *   VLD+0x260 / +0x264       the SIXTEEN dequantiser entries are eight
 *                            segments x {luma, chroma}: entry 2s carries
 *                            segment s's luma pair and 2s+1 its chroma pair,
 *                            each built from get_qindex(s) rather than from
 *                            base_q_idx
 *
 * The vendor's entry point returns immediately when segmentation is off, which
 * is why no earlier trace showed any of this; the whole interface is guarded on
 * segmentation_enabled here for the same reason, and a segmentation-off frame
 * therefore reproduces the previous revision's write set exactly.
 *
 * THE INTER INTERFACE (2026-07-26) -- derived by tracing multi-frame vendor
 * decodes and diffing frame 1's write set against frame 0's on the SAME clip,
 * over four clips (176x144, 192x192, 320x256 and a 176x144 error-resilient
 * encode). The whole difference is 8 MC writes, one MV write, three changed
 * register VALUES and a longer compressed-header sub-parse chain:
 *
 *   MC+0xd28/0xd2c/0xd30   ref[0..2] Y   (already written on a keyframe, where
 *   MC+0xd34/0xd38/0xd3c   ref[0..2] C    all three point at the recon itself)
 *   MC+0xd4c/0xd50/0xd54   (ref_w << 16) | ref_h      -- MEASURED 3/3 sizes
 *   MC+0xd70/0xd74/0xd78   ALIGN(ref_w, 64) / 16      -- MEASURED 3/3 (0xc at
 *                          176 and 192 px, 0x14 at 320 px, so it is the
 *                          64-aligned MB stride, not the display width)
 *   MC+0xd7c = 0x07770777  constant in every inter trace
 *   MC+0xd80 = 0          constant in every inter trace
 *   MV+0x3c4 = use_prev_frame_mvs (1 on a plain inter frame, 0 on an
 *                          error-resilient one, 0 on an intra frame)
 *
 * The ref-SLOT mapping is pinned by frame 2 of the 176x144 trace: only
 * MC+0xd28/+0xd34 changed between frame 1 and frame 2 while +0xd2c/+0xd30 kept
 * the keyframe's recon address -- exactly refresh_frame_flags == 0x01, i.e.
 * +0xd28/+0xd2c/+0xd30 are ref_frame_idx[0]/[1]/[2] in that order.
 *
 * VP9_VLD+0xa8 is a flag word, decomposed across eight measured shapes:
 *      0x00000008  frame_parallel_decoding_mode
 *                                         0x00000004  lossless
 *      0x00000001  inter (!FrameIsIntra)  0x00000010  error_resilient_mode
 *      0x80000000  inter                  0x04000000  interp filter SWITCHABLE
 *   key/lossy 0x8 | key/lossless 0xc | key/error-res 0x18 |
 *   inter/switchable 0x84000009 | inter/EIGHTTAP 0x80000009 |
 *   inter/error-res/switchable 0x84000019.
 *   bit3 was transcribed as "always" for as long as every traced clip carried
 *   libvpx's default frame-parallel 1. MEASURED 2026-07-26 on a matched
 *   frame-parallel 0 / 1 PAIR of the same 176x144 source (identical byte-for-byte
 *   keyframes, identical register write COUNTS, identical every other field):
 *   the ONLY non-address difference in the whole trace is this bit --
 *   fp1 KEY 0x00000008 / INTER 0x84000009 against fp0 KEY 0x00000000 /
 *   INTER 0x84000001. So bit3 == frame_parallel_decoding_mode, and clearing it
 *   is what puts the hardware in the counts-collecting (backward-adaptation)
 *   mode that spec 8.4 requires.
 *   bit31 is confounded with bit0 across every trace (both are "inter"); the
 *   non-SWITCHABLE filter TYPE has no observed encoding yet (the only
 *   non-switchable trace codes literal 1 = EIGHTTAP and writes zero in
 *   bits 25:24).
 *
 * VP9_VLD+0xc8 sub-parse masks, one pulse per compressed_header() sub-step:
 *      0x0001 tx_mode probs (only when tx_mode == TX_MODE_SELECT)
 *      0x0002 coefficient probs      0x0004 skip prob
 *   ...and, on an inter frame only:
 *      0x0008 inter_mode probs
 *      0x0010 interp-filter probs (ONLY when the filter is SWITCHABLE --
 *             MEASURED: the 192x192 clip codes literal 1 and its mask list
 *             is exactly this one shorter)
 *      0x0020 is_inter probs
 *      0x0040 comp_inter probs       0x0080 single_ref probs
 *      0x0100 comp_ref probs
 *      0x0200 y_mode probs           0x0400 partition probs
 *      0x0800 / 0x1000 / 0x2000 motion-vector probs
 *   0x0040 / 0x0080 / 0x0100 are MEASURED (2026-07-27) -- not from a trace but
 *   from libvpud_vcodec.so's own setters, each of which writes its literal mask
 *   to +0xc8 and then polls +0xcc for the matching bit:
 *      VP9_UPDATE_COMP_INTER_PROBS  @0x7bc90  mask 0x40,  polls bit 6
 *      VP9_UPDATE_SINGLE_REF_PROBS  @0x7bd00  mask 0x80,  polls bit 7
 *      VP9_UPDATE_COMP_REF_PROBS    @0x7bd70  mask 0x100, polls bit 8
 *   and their gating is VP9_Read_Frame_Ref_Mode_Probs @0x76d50, which is
 *   libvpx's read_frame_reference_mode_probs() verbatim: comp_inter iff
 *   reference_mode == REFERENCE_MODE_SELECT, single_ref iff reference_mode !=
 *   COMPOUND_REFERENCE, comp_ref iff reference_mode != SINGLE_REFERENCE. The
 *   earlier note that 0x40/0x100 fire together "when compoundReferenceAllowed
 *   holds" was wrong on both counts, and 0x80 is single_ref, not "reference
 *   mode".
 *   NOTE the reference_mode bits themselves have NO sub-step: the host reads
 *   them from the bool decoder (VP9_Read_Frame_Reference_Mode @0x76c80 ->
 *   VP9_Read_Bit), between the 0x20 and the 0x40/0x80/0x100 pulses.
 *
 * PROBABILITY CONTEXT. The 630-access probability upload is NOT a keyframe
 * constant in general: it is the frame context, and its ADDRESS stream is
 * identical on every frame of every trace (615 data words -- 488 into SRAM bank
 * A at word 0x000..0x1f7 with five small gaps, 127 into bank B at 0x00..0x7e)
 * while the DATA changes. The decisive measurement is the error-resilient clip:
 * error_resilient_mode forces setup_past_independence() every frame, and there
 * all four frames upload a BYTE-IDENTICAL table -- the same one the keyframe
 * uploads -- whereas the plain clips differ by 53 and 59 words on frames 1
 * and 2. So the constant transcribed below is the VP9 DEFAULT context, and a
 * frame uploads frame_contexts[frame_context_idx].
 *
 * The updated context comes back from the hardware: the post-decode
 * VP9_VLD+0x1dc MODE-2 WDMA run writes it to DRAM as 640 words, in the two
 * SRAMs' own serialization -- template order for the coefficient records,
 * ADDRESS order for the tail (see vp9_build_prob_map()). MEASURED: with that
 * map our frame 0's dump is word-for-word EQUAL to the vendor's frame 1 UPLOAD
 * for the same clip, ndiff = 0 over all 615 words, so save_probs() needs no
 * software probability model. The mode-1 run produces nothing usable (all zero
 * in every dump), so the vendor's two runs are prob-then-counts, not
 * counts-then-prob as the allocator's symbol naming suggested.
 * NOTE this covers refresh_frame_context with frame_parallel_decoding_mode == 1
 * (the spec stores the FORWARD-UPDATED probabilities). Whether the hardware
 * also performs the backward ADAPTATION that frame_parallel == 0 requires is
 * not established; see the audit.
 *
 * SOURCE: seven reads-inclusive live vendor register traces of a VP9 keyframe
 * decode on med-mt8365-pumpkin (2026-07-25). Six were captured this session
 * from six deliberately-varied clips so no field could stay confounded:
 *
 *   clip           w x h     base_q_idx   lf_level  hdr_sz  tile_start
 *   k176x144-c32   176x144   37           3         150     168
 *   k176x144-c55   176x144   145          18         93     111
 *   k192x192-c32   192x192   37           3         124     142
 *   k320x240-c32   320x240   37           3         149     167
 *   k160x128-c40   160x128   70           6         123     141
 *   k96x64-c20      96x64    15           2          92     110
 *   k176x144-ll    176x144   0 (lossless) 0         115     133
 *
 * All seven traces have an IDENTICAL 138-write structure (139 for the lossless
 * one, which adds VDEC_PP+0x9c0) and a BYTE-IDENTICAL 630-word probability
 * upload, which is what makes the keyframe path a closed problem: a VP9
 * keyframe resets every probability context to the spec defaults, so the upload
 * is a constant (vp9_kf_prob_seq[] below, transcribed verbatim).
 *
 * Sub-block -> reg_base[] index (enum mtk_hw_reg_idx), mt8365.dtsi codec@16000000:
 *   VDEC_SYS 0x16000000  VDEC_MISC 0x16025000  VDEC_LD  0x16020000 (VLD)
 *   VDEC_TOP 0x16020800  VDEC_CM   0x16021000 (MC)     VDEC_AV 0x16023000 (MV)
 *   VDEC_PP  0x16024000  VDEC_HWB  0x16026800 (BS2)    VDEC_HWV 0x16028400 (VP9_VLD)
 *
 * VDEC_HWV is NEW: index 12, added to mt8365.dtsi in the same series together
 * with NUM_MAX_VDEC_REG_BASE 12 -> 13. VP8 needed no DT change; VP9 does.
 *
 * KEY RE FINDINGS
 *
 * 1. TWO bit-read banks on VP9_VLD, with DIFFERENT return conventions:
 *
 *      VP9_VLD + (nbits << 2)         raw bits, uncompressed header
 *      VP9_VLD + 0x80 + (nbits << 2)  bool-decoder literal, compressed header
 *
 *    The RAW bank returns the barrel shifter's 32-bit window AFTER the shift,
 *    so the VALUE of the field is the TOP nbits of the window BEFORE the read
 *    (offset 0 peeks without consuming). Decoding the traced read stream that
 *    way reproduces the VP9 spec's uncompressed_header() exactly, field for
 *    field: frame_marker 2, profile 0, show_existing 0, frame_type KEY,
 *    show_frame 1, error_resilient 0, sync 0x49 0x83 0x42, colour config,
 *    frame_size 175+1 x 143+1, render_size, refresh_frame_context,
 *    frame_parallel, frame_context_idx, loop_filter_params (level 3, sharpness
 *    0, delta_enabled, delta_update, ref deltas {1,0,-1,-1} = the libvpx
 *    setup_past_independence() defaults, mode deltas {0,0}), base_q_idx 33,
 *    three absent delta_q, segmentation_enabled 0, tile_rows_log2 0,
 *    header_size_in_bytes -- 140 bits, then f(4) of trailing_bits() to the
 *    18-byte boundary. VP9_VLD+0x138 is the running BIT POSITION and reads
 *    exactly 140 (0x8c) at that point in every trace, which pins the parse.
 *    Because the read CONSUMES, this parse is mandatory, not observational.
 *
 * 2. The COMPRESSED header is decoded by the HARDWARE, not by software. The
 *    only software reads of it are read_tx_mode(): f(2) through +0x88 and, when
 *    that is ALLOW_32X32, f(1) through +0x84. The result is handed to the HW at
 *    VP9_VLD+0xb4 and the HW then bool-decodes the tx / coefficient probability
 *    updates itself when +0xc8 is pulsed. MEASURED: the lossless clip (which
 *    has no tx_mode syntax at all) makes both reads disappear and writes
 *    +0xb4 = 0; the six lossy clips read tx_mode 3 (ALLOW_32X32) and write 3;
 *    the pre-existing 2026-07-25 trace read 4 (TX_MODE_SELECT), wrote 4, and is
 *    the ONLY trace that also pulses +0xc8 = 1 -- so that pulse is the tx-prob
 *    sub-parse and is gated on TX_MODE_SELECT. After the pulses VP9_VLD+0x138
 *    reads 8 bits short of header_size_in_bytes * 8 in every trace (1192 of
 *    1200 for the 150-byte header, 728 of 736 for the 92-byte one) -- i.e. the
 *    HW really did consume the whole compressed header.
 *
 * 3. The tile descriptors (VP9_VLD+0x100 index / +0x104 data). +0x100 is written
 *    0 once and +0x104 then auto-increments through FOUR WORDS PER TILE, in
 *    raster (row-major) order, with VP9_VLD+0xc4 carrying the tile counts:
 *
 *      +0xc4  = ((tile_rows - 1) << 16) | (tile_cols - 1)
 *      word0  = (tile_end - base) * 8         bits from base to the tile's end
 *      word1  = bs + base,  base = tile_data_start & ~15
 *      word2  = (tile_data_start & 15) * 8    bit offset inside the granule
 *      word3  = ((tile_sb64_rows - 1) << 6) | (tile_sb64_cols - 1)
 *
 *    where tile_data_start / tile_end come from the VP9 spec 6.4 tile-size
 *    partitioning (every tile but the last is prefixed by a 4-byte big-endian
 *    size) and the per-tile SB64 extents from spec 6.2.14's get_tile_offset().
 *
 *    MEASURED on a matched set of 1280x768 clips from ONE source encoded with
 *    libvpx --tile-columns 0/1/2 and --tile-rows 1: the vendor's VLD9 write
 *    count goes 663 / 667 / 675 / 667, i.e. exactly +4 words per extra tile,
 *    and +0xc4 reads 0x0 / 0x1 / 0x3 / 0x10000. All 4 words of all 9 tiles in
 *    those four traces are reproduced by the formulas above. The write-set
 *    difference between the 1-tile and the N-tile trace is EXACTLY {+0xc4,
 *    the extra +0x104 words} -- nothing else: no per-tile-column above-context
 *    reset register, no extra work buffer. The HW resets above-context at a
 *    tile-column boundary itself (spec 8.2.4).
 *
 *    The 16-byte granule is a CORRECTION: the earlier "base = tile_start & ~31"
 *    fit only because every clip measured then had bit 4 of tile_start clear.
 *    Both forms address the same absolute bit (base*8 + word2), so the old form
 *    was harmless, but the vendor's granule is 16.
 *
 *    MORE THAN 8 TILES USE A DIFFERENT TRANSPORT (2026-07-28). The earlier
 *    "VLDTOP+0xb8 is unreachable, it arms only for tile_cols > 8" note was
 *    WRONG on the predicate: transcribed from VP9_Set_Tile_Info @0x7ca20 in
 *    libvpud_vcodec.so, the branch is on the TOTAL tile count
 *    (tile_rows * tile_cols), not the column count --
 *
 *       w3 = n_tile_rows << tile_cols_log2;   [sp,#8]
 *       ...
 *       cmp w0, #0x8 ; b.hi  ->  stp w2,w8,[x24] / stp w7,w5,[x24,#8]
 *                                add x24, x24, #0x10
 *
 *    so with > 8 tiles the four descriptor words per tile are written into a
 *    DMA DESCRIPTOR BUFFER (16 bytes per tile, raster order, same four words
 *    in the same order) instead of being pushed through the +0x104 MMIO port,
 *    the buffer's DMA address goes to VLDTOP+0xb8, and V9_VLD+0xbc is written
 *    0x00000111 -- WITHOUT the bit-16 that the <= 8 case sets (the vendor's
 *    `movk w2,#0x1,lsl#16` is skipped on the >8 branch, @0x7cde8). +0xc4 and
 *    the +0x100 = 0 write happen on both paths.
 *
 *    That predicate IS reachable here: 4 tile rows x 4 tile columns = 16 tiles
 *    at 1920x1080 (vp90-2-08-tile-4x4). Pushing 64 words through the MMIO port
 *    and leaving bit-16 set made the hardware time out (V9_STATUS15C = 1) on
 *    every frame of that vector. A matched 9-point 1920x1080 sweep (one source,
 *    tile_rows_log2 x tile_cols_log2 in {0,1,2}^2) is bit-exact on all eight
 *    shapes up to 8 tiles and failed only at 4x4 -- so the discriminator is the
 *    tile COUNT crossing 8, not "both dimensions > 1".
 *
 * 4. Geometry, all MEASURED against the five distinct resolutions:
 *      VDEC_TOP+0x70 = (h << 16) | w                  display size
 *      VDEC_TOP+0x68 = (sb64_rows-1) << 16 | (sb64_cols-1)
 *      MC+0xd88      = (mi_cols << 16) | mi_rows      8-px units
 *      MC+0x22c      = ALIGN(w, 64) / 16              recon stride, MB units
 *      MC+0x260/0x264 = ALIGN(w,64)/16 - 1 / ALIGN(h,64)/16 - 1
 *      MC+0x208/0x20c = w / h                         DISPLAY size, not aligned
 *      PP+0xf0       = (h << 16) | w
 *    The recon buffer is 64-ALIGNED in BOTH dimensions (the vendor CAPTURE
 *    geometry is 192x192 for 176x144, 320x256 for 320x240, 128x64 for 96x64) --
 *    unlike VP8/H.264, which are ALIGN(w,16) x ALIGN(h,32).
 *
 * 5. The VLD fetch window end is ALIGN(bs_size, 64) + 448, 6/6 across clips
 *    (VP8 uses bs_size + 0x400).
 */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/moduleparam.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>
#include <uapi/linux/v4l2-controls.h>

#include "../mtk_vcodec_dec.h"
#include "../../common/mtk_vcodec_intr.h"
#include "../vdec_drv_base.h"
#include "../vdec_drv_if.h"

/* ---- VDEC_SYS ---- */
#define SYS_CKEN_SET		0x00
#define SYS_DCM_CON		0x18
/* ---- VDEC_MISC ---- */
#define MISC_CRC_BASE		0x08	/* 0x08..0x24 per-picture recon CRC */
#define MISC_STATE		0x04
#define MISC_SYS_SEL		0x84	/* 2 for VP9 (H.264 and VP8 use 1) */
#define MISC_SPEC_CON		0xc8
#define MISC_FUNC_CON		0xcc
#define MISC_DCM_CON		0xec
#define MISC_SRST		0xf4
#define MISC_CFG178		0x178
/* ---- VDEC_LD (VLD) / VDEC_HWB (BS2): the shared bitstream-fetch arm ---- */
#define VLD_FETCH_BASE		0xb0
#define VLD_WIN_START		0xb4	/* window start PA >> 6 */
#define VLD_WIN_END		0xb8	/* window end   PA >> 6 */
#define VLD_KICK		0x8c
#define VLD_FETCH_DONE		0xe8
#define VLD_ENABLE_EC		0xec	/* RMW |= 0x10000000 on BOTH VLD and BS2 */
#define VLD_IDLE		0xf4
#define VLD_SRST		0x108
#define VLD_WIN_END_RAW		0x110
#define VLD_CFG140		0x140	/* 0x00100014 on both VLD and BS2 */
#define VLD_DEQ_ADDR		0x260	/* dequantiser SRAM port pair */
#define VLD_DEQ_DATA		0x264
/* ---- VDEC_TOP ---- */
#define TOP_PICSIZE_SB		0x68	/* (sb64_rows-1)<<16 | (sb64_cols-1) */
#define TOP_PICSIZE_PX		0x70	/* (h<<16) | w */
#define TOP_SEGMAP_A		0x118	/* segment-map surface, raw phys */
#define TOP_SEGMAP_B		0x11c	/* ...written twice, same address */
#define TOP_CFG124		0x124	/* low 16 = segment-map prefetch size */
#define TOP_TILE_DESC		0x0b8	/* > 8 tiles: descriptor-buffer DMA addr */
/* ---- VDEC_CM (MC) ---- */
#define MC_PIC_W_PX		0x208
#define MC_PIC_H_PX		0x20c
#define MC_ENABLE220		0x220
#define MC_RECON_Y		0x224	/* recon Y >> 9 */
#define MC_RECON_C		0x228	/* recon C >> 8 */
#define MC_RECON_STRIDE_MB	0x22c	/* ALIGN(w,64)/16 */
#define MC_INTER_EN		0x238
#define MC_ENABLE250		0x250
#define MC_MB_W_M1		0x260
#define MC_MB_H_M1		0x264
#define MC_RECON_LIMIT		0x850	/* VP8 recon-window limit (quirk only) */
#define MC_RECON_BASE		0x854	/* VP8 recon-window base  (quirk only) */
#define MC_CFGA60		0xa60
#define MC_CFGB5C		0xb5c
#define MC_REF_Y		0xd28	/* ref[0..2] Y, full phys */
#define MC_REF_C		0xd34	/* ref[0..2] C, full phys */
#define MC_REF_SCALE_STEP	0xd40	/* ref[0..2] (x_step_q4<<16) | y_step_q4 */
#define MC_REF_DIM		0xd4c	/* ref[0..2] (w << 16) | h, inter only */
#define MC_REF_SCALE_FP		0xd58	/* ref[i]: +8i = x_scale_fp, +8i+4 = y */
#define MC_REF_STRIDE_MB	0xd70	/* ref[0..2] ALIGN(w,64)/16, inter only */
#define MC_REF_RRF_BLK		0xd7c	/* per-ref max RRF block codes, packed */
#define MC_REF_SCALE_EN		0xd80	/* per-ref is_scaled, packed 8/4/0 */
#define MC_MI_DIM		0xd88	/* (mi_cols<<16) | mi_rows */
/* ---- VDEC_AV (MV) ---- */
#define MV_CFG3C4		0x3c4
#define MV_COLMV_A		0x3c0	/* colocated-MV buffer >> 4 */
#define MV_COLMV_B		0x20c
/* ---- VDEC_PP ---- */
#define PP_LF_CTRL		0xd8	/* (loop_filter_level << 16) | 1 */
#define PP_REF_LF_DELTA		0xdc	/* four signed bytes */
#define PP_MODE_LF_DELTA	0xe0	/* two signed bytes */
#define PP_PICSIZE		0xf0
#define PP_DEQ_GATE		0x504	/* 1 around the dequantiser upload */
#define PP_SEG_LF_EN		0xe4	/* ALT_L enable mask | abs_delta << 8 */
#define PP_SEG_LF_D03		0xe8	/* ALT_L data, segments 0..3, s8 each */
#define PP_SEG_LF_D47		0xec	/* ALT_L data, segments 4..7, s8 each */
#define PP_LOSSLESS		0x9c0	/* 0xe00, lossless frames only */
/* ---- VDEC_HWV (VP9_VLD) ---- */
#define V9_BITS_RAW		0x000	/* + (nbits<<2): raw bits, returns window */
#define V9_BITS_BOOL		0x080	/* + (nbits<<2): bool literal, returns value */
#define V9_CTRL			0x0a4
#define V9_FLAGS		0x0a8	/* 0x8 | (lossless ? 0x4 : 0) */
#define V9_CFGAC		0x0ac
/*
 * V9_REF_MODE (0x0b0) -- the frame_reference_mode word. Transcribed from
 * VP9_Set_Pic_Info @0x7c494 in libvpud_vcodec.so (mt8365, aarch64), which
 * builds it as
 *      w0 = sum_{i<4} ref_frame_sign_bias[i] << i        (@ctx+0x4980)
 *      w2 = (w0 << 16)
 *         + ((comp_var_ref[0] + (comp_var_ref[1] << 2)) << 24)
 *         + reference_mode + (comp_fixed_ref << 8)
 * i.e. bits [1:0] reference_mode, [9:8] comp_fixed_ref,
 *      [19:16] ref_frame_sign_bias[0..3] (index 0 = INTRA, always 0),
 *      [25:24] comp_var_ref[0], [27:26] comp_var_ref[1].
 * Every previously captured trace is single-reference with all sign biases 0,
 * where this collapses to 0x0 -- which is why it was long transcribed as a
 * constant and mis-named "lossless".
 */
#define V9_REF_MODE		0x0b0
#define V9_TX_MODE		0x0b4
#define V9_KICK			0x0b8	/* = 1 : DECODE GO */
#define V9_CFGBC		0x0bc	/* 0x00010111 */
#define V9_CFGC4		0x0c4	/* ((tile_rows-1) << 16) | (tile_cols-1) */
#define V9_HDR_PARSE		0x0c8	/* pulse: run a compressed-header sub-parse */
#define V9_HDR_STATUS		0x0cc
#define V9_TILE_ADDR		0x100	/* tile descriptor SRAM port pair */
#define V9_TILE_DATA		0x104
#define V9_STATE110		0x110
#define V9_BITPOS		0x138	/* running bit position of the parse */
#define V9_BSHIFT_INIT		0x144
#define V9_STATUS15C		0x15c
#define V9_DONE			0x174	/* bit0 = picture done (VP8 convention) */
#define V9_SEG_TREE0		0x1f0	/* segmentation tree_probs[0..3], LE */
#define V9_SEG_TREE1		0x1f4	/* segmentation tree_probs[4..6] */
#define V9_SEG_PRED		0x1f8	/* segmentation pred_probs[0..2] */
#define V9_PROB_PTR_A		0x198
#define V9_PROB_DATA_A		0x19c
#define V9_PROB_PTR_B		0x1a0
#define V9_PROB_DATA_B		0x1a4
#define V9_SYNC			0x1a8	/* pulse 1, poll to 0 */
#define V9_WDMA_MODE		0x1dc
#define V9_WDMA_ADDR		0x1e0
#define V9_WDMA_GO		0x1e8
#define V9_WDMA_DONE		0x1ec

#define VP9_DPB_SIZE		9
#define MT8365_VP9_TIMEOUT	400000
#define VP9_COLMV_BUF_SZ	(512 * 1024)
#define VP9_WDMA_BUF_SZ		(64 * 1024)
/*
 * The segment-id map surface. The vendor allocator sizes it
 * (sb64_cols * sb64_rows + 8) * 32 bytes -- 32 B per superblock, i.e. one
 * NIBBLE per 8x8 mode-info unit (64 per SB64), plus an 8-SB guard. 128 KiB
 * covers 4096 superblocks, more than VP9's 4096x2304 maximum (64 x 36 = 2304).
 */
#define VP9_SEGMAP_BUF_SZ	(128 * 1024)

/*
 * Tile-descriptor buffer, used INSTEAD of the +0x104 MMIO port whenever the
 * frame carries more than 8 tiles (see header note 3). 16 bytes per tile;
 * VP9's maximum is 64 tile columns x 4 tile rows = 256 tiles = 4 KiB, which is
 * exactly the size the vendor allocator reserves.
 */
#define VP9_TILEDESC_BUF_SZ	4096
#define VP9_TILEDESC_MMIO_MAX	8	/* <= this many tiles go through +0x104 */

/* Probability-context geometry -- see the header comment. */
#define VP9_PROB_WORDS		615	/* data words in the upload template */
#define VP9_NUM_FC		4	/* VP9 keeps four frame contexts */
#define VP9_PROB_COEF_WORDS	480	/* template order below this, addr order above */
#define VP9_PROB_BANK_B_BASE	608	/* bank B word 0x60 lands here in the dump */

/* VP9 interpolation_filter, in the spec's enum order (SWITCHABLE == 4). */
#define VP9_SWITCHABLE_FILTER	4

/* VP9 spec 6.3.12 frame_reference_mode() -- libvpx's REFERENCE_MODE enum */
#define VP9_SINGLE_REFERENCE	0
#define VP9_COMPOUND_REFERENCE	1
#define VP9_REFERENCE_MODE_SELECT 2

/*
 * VP9 spec 6.2.7 literal_to_type[]: the coded interpolation-filter literal maps
 * to libvpx's filter TYPE enum, which is what the hardware's +0xa8 bits 26:24
 * carry (EIGHTTAP 0, SMOOTH 1, SHARP 2, BILINEAR 3, SWITCHABLE 4).
 */
static const u8 vp9_lit2type[4] = { 1, 0, 2, 3 };

/* VP9 segmentation features, spec 6.2.11 order. */
#define VP9_SEG_LVL_ALT_Q	0
#define VP9_SEG_LVL_ALT_L	1
#define VP9_SEG_LVL_REF_FRAME	2
#define VP9_SEG_LVL_SKIP	3

/*
 * Bring-up knob, mirroring mt8365_vp8_quirk:
 *   bit0 (0x1)   also program the VP8-style recon write WINDOW MC+0x850/0x854
 *                to the union of both CAPTURE planes. The vendor NEVER writes
 *                it on a VP9 decode, but the MC block is shared with the VP8 /
 *                H.264 backends, so a stale window from an earlier decode is a
 *                candidate for an all-zero frame WITH a correct HW CRC (the
 *                exact VP8 signature -- see the VP8 recon-window audit).
 *   bit8 (0x100) dump every register write (WSET log to diff against the
 *                vendor trace); the 630-word probability run is excluded.
 *   bit10 (0x400) DISABLE backward adaptation (VP9 spec 8.4) and go back to
 *                pinning VLD9+0xa8 bit3, i.e. tell the hardware every stream is
 *                frame-parallel and never collect counts. 8.4 is ON by default
 *                since it was measured to take fp0-176.ivf from 1/6 to 6/6
 *                bit-exact and VP9-TEST-VECTORS from 3/305 to 138/305 with no
 *                board reset. (The bit spent one round OFF by default because
 *                the counts-collecting mode had been seen to reset the board
 *                around VP9-TEST-VECTORS' vp90-2-14-resize-fp-tiles-* block;
 *                that block was then isolated and does not even reach this
 *                backend -- every one of its 22 vectors is refused upstream on
 *                resolution -- and the full suite now runs clean.)
 *   bit11 (0x800) dump the 8.4 inputs and outputs for OFF-BOARD validation:
 *                the PRE frame context (615 words), the live part of the counts
 *                surface (3592 words), the POST context and the hardware's
 *                forward-updated context. libvpx computes the identical
 *                adaptation, so a host-side replay must reproduce POST exactly.
 *   bit12 (0x1000) apply the MEASURED VLD9+0xa8 inter-frame flag corrections:
 *                bit31 = allow_high_precision_mv (not "always on an inter
 *                frame") and bits 26:24 = the interpolation filter's libvpx
 *                TYPE (not "0x04000000 iff SWITCHABLE"). Both were confounded
 *                until vp90-2-19-skip broke them; kept behind a knob so the
 *                segmentation conformance delta can be attributed on its own.
 *   bit13 (0x2000) force every reference unscaled (pre-scaled-reference
 *                behaviour).
 *   bit14 (0x4000) force VLD9+0xb0 to 0 -- the pre-compound-prediction
 *                behaviour, for A/B.
 *   bit15 (0x8000) write reference_mode = SINGLE_REFERENCE in VLD9+0xb0 even on
 *                a compound-allowed frame (the sign-bias and comp_fixed_ref /
 *                comp_var_ref fields are still programmed). The hardware
 *                decodes reference_mode itself, so which value belongs in the
 *                register is not determined by the bitstream parse here; this
 *                knob is what discriminates the two readings.
 *   bit19 (0x80000) write the PP+0xd8 low nibble as the constant 1 instead of
 *                the frame's loop_filter_delta_enabled (pre-2026-07-28
 *                behaviour, for A/B on vp90-2-09-lf_deltas -- the only
 *                VP9-TEST-VECTORS clip that toggles the flag).
 */
static int mt8365_vp9_quirk;
module_param(mt8365_vp9_quirk, int, 0644);
MODULE_PARM_DESC(mt8365_vp9_quirk, "mt8365 VP9 bring-up quirk mask");

static int mt8365_vp9_rrf_up = -1;
module_param(mt8365_vp9_rrf_up, int, 0644);
MODULE_PARM_DESC(mt8365_vp9_rrf_up,
		 "mt8365 VP9: override VP9_Max_RRF_BLK for an UPSCALING reference (scale_fp < 1<<14); -1 = transcribed");

static int mt8365_vp9_dumpn = 32;
module_param(mt8365_vp9_dumpn, int, 0644);
MODULE_PARM_DESC(mt8365_vp9_dumpn, "mt8365 VP9 quirk-0x200 dump length in words");

static int mt8365_vp9_cntn;
module_param(mt8365_vp9_cntn, int, 0644);
MODULE_PARM_DESC(mt8365_vp9_cntn,
		 "mt8365 VP9 quirk-0x200 COUNTS dump length in words (cnt_buf is 4096)");

static bool mt8365_vp9_diag;
module_param(mt8365_vp9_diag, bool, 0644);
MODULE_PARM_DESC(mt8365_vp9_diag, "mt8365 VP9 per-frame diagnostic dump");

/*
 * EXPERIMENT ARM, DEFAULT OFF -- raise the VP9 picture-size ceiling above the
 * built-in MTK_VDEC_MAX_W/H (2048x1088) to ask the silicon directly whether
 * that constant is a hardware limit or a conservative driver value.
 *
 * Packed (max_w << 16) | max_h; 0 = built-in, unchanged behaviour.
 *
 * NOTE -- raising the gate alone is NOT sufficient and would measure the wrong
 * thing.  The colMV store is a FIXED VP9_COLMV_BUF_SZ allocation, while the
 * vendor allocator (Vdec_Drv_VP9_AllocBuf @0x79780, transcribed in N4) sizes it
 * sb64_cols * sb64_rows * 576 + 1152 -- 307 KiB at 2048x1088 but 1.12 MiB at
 * 3840x2160 and 1.27 MiB at 4096x2304.  So the work buffers are sized from this
 * ceiling too; otherwise the hardware DMAs ~800 KiB past the end of the
 * allocation and the result is an IOMMU fault, not a decoder verdict.
 */
static unsigned int mt8365_vp9_maxdim;
module_param(mt8365_vp9_maxdim, uint, 0644);
MODULE_PARM_DESC(mt8365_vp9_maxdim,
		 "mt8365 VP9: raise the picture-size ceiling, packed (w << 16) | h; 0 = built-in 2048x1088");

static u32 vp9_max_w(void)
{
	u32 w = mt8365_vp9_maxdim >> 16;

	return w > MTK_VDEC_MAX_W ? w : MTK_VDEC_MAX_W;
}

static u32 vp9_max_h(void)
{
	u32 h = mt8365_vp9_maxdim & 0xffff;

	return h > MTK_VDEC_MAX_H ? h : MTK_VDEC_MAX_H;
}

/*
 * VP9 spec 8-bit dequantiser lookups (VP9 bitstream spec section 8.6.1).
 * VERIFIED against five measured (base_q_idx -> VLD+0x264) trace points:
 * q=0 -> 4/4, 15 -> 19/22, 37 -> 38/44, 70 -> 66/77, 145 -> 180/235.
 *
 * The ac table's LAST SIXTEEN entries (index 240..255) were wrong until
 * 2026-07-28 -- they had been extrapolated rather than transcribed, and every
 * one of them was too large (1370 vs 1369 at 240, rising to 1842 vs 1828 at
 * 255).  Now transcribed byte-for-byte out of libvpx 1.15.0's own
 * ac_qlookup[] (the vp9dec we gate against) and cross-checked against the
 * vendor's dequantiser-SRAM writes on this silicon: base_q_idx 236 ->
 * VLD+0x264 = 0x04f302cd (ac 1267 / dc 717), 240 -> 0x0559031c (ac 1369 /
 * dc 796), 255 -> 0x07240538 (ac 1828 / dc 1336).  The dc table needed no
 * change.
 */
static const u16 vp9_dc_qlookup[256] = {
	   4,    8,    8,    9,   10,   11,   12,   12,   13,   14,   15,   16,
	  17,   18,   19,   19,   20,   21,   22,   23,   24,   25,   26,   26,
	  27,   28,   29,   30,   31,   32,   32,   33,   34,   35,   36,   37,
	  38,   38,   39,   40,   41,   42,   43,   43,   44,   45,   46,   47,
	  48,   48,   49,   50,   51,   52,   53,   53,   54,   55,   56,   57,
	  57,   58,   59,   60,   61,   62,   62,   63,   64,   65,   66,   66,
	  67,   68,   69,   70,   70,   71,   72,   73,   74,   74,   75,   76,
	  77,   78,   78,   79,   80,   81,   81,   82,   83,   84,   85,   85,
	  87,   88,   90,   92,   93,   95,   96,   98,   99,  101,  102,  104,
	 105,  107,  108,  110,  111,  113,  114,  116,  117,  118,  120,  121,
	 123,  125,  127,  129,  131,  134,  136,  138,  140,  142,  144,  146,
	 148,  150,  152,  154,  156,  158,  161,  164,  166,  169,  172,  174,
	 177,  180,  182,  185,  187,  190,  192,  195,  199,  202,  205,  208,
	 211,  214,  217,  220,  223,  226,  230,  233,  237,  240,  243,  247,
	 250,  253,  257,  261,  265,  269,  272,  276,  280,  284,  288,  292,
	 296,  300,  304,  309,  313,  317,  322,  326,  330,  335,  340,  344,
	 349,  354,  359,  364,  369,  374,  379,  384,  389,  395,  400,  406,
	 411,  417,  423,  429,  435,  441,  447,  454,  461,  467,  475,  482,
	 489,  497,  505,  513,  522,  530,  539,  549,  559,  569,  579,  590,
	 602,  614,  626,  640,  654,  668,  684,  700,  717,  736,  755,  775,
	 796,  819,  843,  869,  896,  925,  955,  988, 1022, 1058, 1098, 1139,
	1184, 1232, 1282, 1336,
};

static const u16 vp9_ac_qlookup[256] = {
	   4,    8,    9,   10,   11,   12,   13,   14,   15,   16,   17,   18,
	  19,   20,   21,   22,   23,   24,   25,   26,   27,   28,   29,   30,
	  31,   32,   33,   34,   35,   36,   37,   38,   39,   40,   41,   42,
	  43,   44,   45,   46,   47,   48,   49,   50,   51,   52,   53,   54,
	  55,   56,   57,   58,   59,   60,   61,   62,   63,   64,   65,   66,
	  67,   68,   69,   70,   71,   72,   73,   74,   75,   76,   77,   78,
	  79,   80,   81,   82,   83,   84,   85,   86,   87,   88,   89,   90,
	  91,   92,   93,   94,   95,   96,   97,   98,   99,  100,  101,  102,
	 104,  106,  108,  110,  112,  114,  116,  118,  120,  122,  124,  126,
	 128,  130,  132,  134,  136,  138,  140,  142,  144,  146,  148,  150,
	 152,  155,  158,  161,  164,  167,  170,  173,  176,  179,  182,  185,
	 188,  191,  194,  197,  200,  203,  207,  211,  215,  219,  223,  227,
	 231,  235,  239,  243,  247,  251,  255,  260,  265,  270,  275,  280,
	 285,  290,  295,  300,  305,  311,  317,  323,  329,  335,  341,  347,
	 353,  359,  366,  373,  380,  387,  394,  401,  408,  416,  424,  432,
	 440,  448,  456,  465,  474,  483,  492,  501,  510,  520,  530,  540,
	 550,  560,  571,  582,  593,  604,  615,  627,  639,  651,  663,  676,
	 689,  702,  715,  729,  743,  757,  771,  786,  801,  816,  832,  848,
	 864,  881,  898,  915,  933,  951,  969,  988, 1007, 1026, 1046, 1066,
	1087, 1108, 1129, 1151, 1173, 1196, 1219, 1243, 1267, 1292, 1317, 1343,
	1369, 1396, 1423, 1451, 1479, 1508, 1537, 1567, 1597, 1628, 1660, 1692,
	1725, 1759, 1793, 1828,
};

/*
 * The VP9 KEYFRAME probability upload, transcribed VERBATIM from the vendor
 * trace as an ordered (register offset, value) stream on the VP9_VLD SRAM
 * ports: +0x198 / +0x1a0 set the two auto-incrementing write pointers and
 * +0x19c / +0x1a4 are their data ports. Each probability record is 20 bytes --
 * four words through +0x19c then one through +0x1a4 -- and the records are the
 * VP9 default probability tables (record 0 decodes as the tx4x4 / Y / intra
 * band-0 coefficient probabilities {195,29,183},{84,49,136},{8,42,71} and
 * record 1 as band 1's six triples, which is what pins the layout).
 *
 * This is a CONSTANT for every keyframe: a VP9 keyframe runs
 * setup_past_independence() and resets all frame contexts to the defaults. That
 * is not an assumption -- the 630-word stream is BYTE-IDENTICAL across all
 * seven vendor traces, which span five resolutions, base_q_idx 0..145, and two
 * different encoders. The run is also contiguous: not one non-probability
 * register access is interleaved into it in any trace.
 */
static const struct { u16 off; u32 val; } vp9_kf_prob_seq[] = {
	{ 0x198, 0x00010000 }, { 0x1a0, 0x00010000 }, { 0x19c, 0x54b71dc3 }, { 0x19c, 0x2a088831 },
	{ 0x19c, 0x00000047 }, { 0x19c, 0x00000000 }, { 0x1a4, 0x00000000 }, { 0x19c, 0x23a96b1f },
	{ 0x19c, 0x52119f63 }, { 0x19c, 0x7242088c }, { 0x19c, 0x014c2c02 }, { 0x1a4, 0x00002013 },
	{ 0x19c, 0x1dc98428 }, { 0x19c, 0x5b0dbb72 }, { 0x19c, 0x7f4b079d }, { 0x19c, 0x015f3a03 },
	{ 0x1a4, 0x00002f1c }, { 0x19c, 0x2add8e45 }, { 0x19c, 0x5b0fc97a }, { 0x19c, 0x7943069f },
	{ 0x19c, 0x014d2a01 }, { 0x1a4, 0x00001f11 }, { 0x19c, 0x43e49466 }, { 0x19c, 0x5211cc75 },
	{ 0x19c, 0x723b069a }, { 0x19c, 0x014b2702 }, { 0x1a4, 0x00001d0f }, { 0x19c, 0x77e9399c },
	{ 0x19c, 0x303ad439 }, { 0x19c, 0x7c281da3 }, { 0x19c, 0x03511e0c }, { 0x1a4, 0x00001f0c },
	{ 0x19c, 0x7ce26bbf }, { 0x19c, 0x6319cc75 }, { 0x19c, 0x0000009b }, { 0x19c, 0x00000000 },
	{ 0x1a4, 0x00000000 }, { 0x19c, 0x25d2941d }, { 0x19c, 0x5d08c27e }, { 0x19c, 0x7644029d },
	{ 0x19c, 0x01452701 }, { 0x1a4, 0x00002111 }, { 0x19c, 0x1bd59729 }, { 0x19c, 0x5203c17b },
	{ 0x19c, 0x693a0190 }, { 0x19c, 0x013c2001 }, { 0x1a4, 0x00001a0d }, { 0x19c, 0x17dc9f3b },
	{ 0x19c, 0x5804c67e }, { 0x19c, 0x72420197 }, { 0x19c, 0x01472601 }, { 0x1a4, 0x00002212 },
	{ 0x19c, 0x33e88872 }, { 0x19c, 0x530bcf72 }, { 0x19c, 0x6938039b }, { 0x19c, 0x01412101 },
	{ 0x1a4, 0x00002211 }, { 0x19c, 0x79ea4195 }, { 0x19c, 0x313dd739 }, { 0x19c, 0x72241ca6 },
	{ 0x19c, 0x034c190c }, { 0x1a4, 0x00002a10 }, { 0x19c, 0x84dc31d6 }, { 0x19c, 0x412abc3f },
	{ 0x19c, 0x00000089 }, { 0x19c, 0x00000000 }, { 0x1a4, 0x00000000 }, { 0x19c, 0x68dd8955 },
	{ 0x19c, 0x6f31d883 }, { 0x19c, 0x9b5715c0 }, { 0x19c, 0x01573102 }, { 0x1a4, 0x00001c10 },
	{ 0x19c, 0x5ae6a359 }, { 0x19c, 0x641ddc89 }, { 0x19c, 0x87460ab7 }, { 0x19c, 0x01512a02 },
	{ 0x1a4, 0x00002111 }, { 0x19c, 0x37eda76c }, { 0x19c, 0x610fde85 }, { 0x19c, 0x874804b3 },
	{ 0x19c, 0x01552d01 }, { 0x1a4, 0x00002613 }, { 0x19c, 0x42f0927c }, { 0x19c, 0x5811e07c },
	{ 0x19c, 0x7a3a04af }, { 0x19c, 0x014b2401 }, { 0x1a4, 0x00002512 }, { 0x19c, 0x7ef14f8d },
	{ 0x19c, 0x3a42e346 }, { 0x19c, 0x882c1eb6 }, { 0x19c, 0x0260220c }, { 0x1a4, 0x00002f14 },
	{ 0x19c, 0x8ff963e5 }, { 0x19c, 0x6d2eeb6f }, { 0x19c, 0x000000c0 }, { 0x19c, 0x00000000 },
	{ 0x1a4, 0x00000000 }, { 0x19c, 0x5eec9e52 }, { 0x19c, 0x7519e092 }, { 0x19c, 0x955709bf },
	{ 0x19c, 0x01633803 }, { 0x1a4, 0x00003921 }, { 0x19c, 0x44eda753 }, { 0x19c, 0x670ade91 },
	{ 0x19c, 0x834802b1 }, { 0x19c, 0x014f2901 }, { 0x1a4, 0x00002714 }, { 0x19c, 0x2fefa763 },
	{ 0x19c, 0x680ae08d }, { 0x19c, 0x854902b2 }, { 0x19c, 0x01552c01 }, { 0x1a4, 0x00002f16 },
	{ 0x19c, 0x47f3917f }, { 0x19c, 0x5d11e481 }, { 0x19c, 0x7c3d03b1 }, { 0x19c, 0x01542901 },
	{ 0x1a4, 0x00003415 }, { 0x19c, 0x8cf44e9d }, { 0x19c, 0x3a45e748 }, { 0x19c, 0x892c1fb8 },
	{ 0x19c, 0x0869260e }, { 0x1a4, 0x00003d17 }, { 0x19c, 0x34bb227d }, { 0x19c, 0x1f068529 },
	{ 0x19c, 0x00000038 }, { 0x19c, 0x00000000 }, { 0x1a4, 0x00000000 }, { 0x19c, 0x33996d25 },
	{ 0x19c, 0x57179366 }, { 0x19c, 0x65430880 }, { 0x19c, 0x013f2901 }, { 0x1a4, 0x00001d13 },
	{ 0x19c, 0x11b99a1f }, { 0x19c, 0x6006af7f }, { 0x19c, 0x72490291 }, { 0x19c, 0x01523301 },
	{ 0x1a4, 0x00002d1c }, { 0x19c, 0x0ac8a317 }, { 0x19c, 0x5d02b983 }, { 0x19c, 0x6f430194 },
	{ 0x19c, 0x01452901 }, { 0x1a4, 0x0000180e }, { 0x19c, 0x0cd9b01d }, { 0x19c, 0x6503c991 },
	{ 0x19c, 0x6f45019c }, { 0x19c, 0x013f2701 }, { 0x1a4, 0x0000170e }, { 0x19c, 0x19e9c039 },
	{ 0x19c, 0x6d06d79a }, { 0x19c, 0x764e03a7 }, { 0x19c, 0x01453001 }, { 0x1a4, 0x00001d15 },
	{ 0x19c, 0x6cf569ca }, { 0x19c, 0x5a12d86a }, { 0x19c, 0x00000090 }, { 0x19c, 0x00000000 },
	{ 0x1a4, 0x00000000 }, { 0x19c, 0x40dbac21 }, { 0x19c, 0x750ece95 }, { 0x19c, 0x8d5a05b1 },
	{ 0x19c, 0x015f3d02 }, { 0x1a4, 0x00003925 }, { 0x19c, 0x0bdcb321 }, { 0x19c, 0x5901c68c },
	{ 0x19c, 0x683c0194 }, { 0x19c, 0x01392101 }, { 0x1a4, 0x0000150c }, { 0x19c, 0x08ddb51e },
	{ 0x19c, 0x5701c68d }, { 0x19c, 0x643a0191 }, { 0x19c, 0x01371f01 }, { 0x1a4, 0x0000140c },
	{ 0x19c, 0x07e0ba20 }, { 0x19c, 0x5601c68e }, { 0x19c, 0x643a018f }, { 0x19c, 0x01371f01 },
	{ 0x1a4, 0x0000160c }, { 0x19c, 0x14e3c039 }, { 0x19c, 0x6003cc8f }, { 0x19c, 0x7044019a },
	{ 0x19c, 0x01452a01 }, { 0x1a4, 0x00002013 }, { 0x19c, 0x71d723d4 }, { 0x19c, 0x301da92f },
	{ 0x19c, 0x00000069 }, { 0x19c, 0x00000000 }, { 0x1a4, 0x00000000 }, { 0x19c, 0x6acb814a },
	{ 0x19c, 0x6b31cb78 }, { 0x19c, 0x905413b2 }, { 0x19c, 0x01543204 }, { 0x1a4, 0x0000190f },
	{ 0x19c, 0x2cd9ac47 }, { 0x19c, 0x660fd18d }, { 0x19c, 0x854c06ad }, { 0x19c, 0x01593302 },
	{ 0x1a4, 0x00002a18 }, { 0x19c, 0x1fe7b940 }, { 0x19c, 0x6708d894 }, { 0x19c, 0x834a03af },
	{ 0x19c, 0x01512e01 }, { 0x1a4, 0x00001e12 }, { 0x19c, 0x19ebc441 }, { 0x19c, 0x6905dd9d },
	{ 0x19c, 0x784301ae }, { 0x19c, 0x01452601 }, { 0x1a4, 0x00001e0f }, { 0x19c, 0x1eeecc41 },
	{ 0x19c, 0x6b07e09c }, { 0x19c, 0x7c4602b1 }, { 0x19c, 0x01492a01 }, { 0x1a4, 0x00002212 },
	{ 0x19c, 0x90fb56e1 }, { 0x19c, 0x632aeb68 }, { 0x19c, 0x000000b5 }, { 0x19c, 0x00000000 },
	{ 0x1a4, 0x00000000 }, { 0x19c, 0x70efaf55 }, { 0x19c, 0x881de5a5 }, { 0x19c, 0xa2670cc8 },
	{ 0x19c, 0x027b4d06 }, { 0x1a4, 0x00005435 }, { 0x19c, 0x1eefb74b }, { 0x19c, 0x6a03dd9b },
	{ 0x19c, 0x804a01ab }, { 0x19c, 0x014c2c01 }, { 0x1a4, 0x00001c11 }, { 0x19c, 0x1bf0b949 },
	{ 0x19c, 0x6b02de9f }, { 0x19c, 0x7f4b01ac }, { 0x19c, 0x01492a01 }, { 0x1a4, 0x00001d11 },
	{ 0x19c, 0x15eebe3e }, { 0x19c, 0x6b02de9f }, { 0x19c, 0x7a4801ac }, { 0x19c, 0x01472801 },
	{ 0x1a4, 0x00002012 }, { 0x19c, 0x1bf0c73d }, { 0x19c, 0x7104e2a1 }, { 0x19c, 0x814c01b4 },
	{ 0x19c, 0x01502e01 }, { 0x1a4, 0x00002917 }, { 0x19c, 0x05991b07 }, { 0x19c, 0x10015f1e },
	{ 0x19c, 0x0000001e }, { 0x19c, 0x00000000 }, { 0x1a4, 0x00000000 }, { 0x19c, 0x397f4b32 },
	{ 0x19c, 0x431b7c4b }, { 0x19c, 0x56360a6c }, { 0x19c, 0x01342101 }, { 0x1a4, 0x0000120c },
	{ 0x19c, 0x1a977d2b }, { 0x19c, 0x5307946c }, { 0x19c, 0x593b027a }, { 0x19c, 0x013c2601 },
	{ 0x1a4, 0x00001b11 }, { 0x19c, 0x0da39017 }, { 0x19c, 0x4b029a70 }, { 0x19c, 0x51320175 },
	{ 0x19c, 0x01331f01 }, { 0x1a4, 0x0000170e }, { 0x19c, 0x06b9a212 }, { 0x19c, 0x4e01ab7b },
	{ 0x19c, 0x5633017d }, { 0x19c, 0x01361f01 }, { 0x1a4, 0x0000170e }, { 0x19c, 0x03e3c70f },
	{ 0x19c, 0x5b01cc96 }, { 0x19c, 0x5f370192 }, { 0x19c, 0x01351e01 }, { 0x1a4, 0x0000140b },
	{ 0x19c, 0x13f03713 }, { 0x19c, 0x3403c43b }, { 0x19c, 0x00000069 }, { 0x19c, 0x00000000 },
	{ 0x1a4, 0x00000000 }, { 0x19c, 0x68cfa629 }, { 0x19c, 0x7b1fc799 }, { 0x19c, 0x98650eb5 },
	{ 0x19c, 0x016a4805 }, { 0x1a4, 0x00003424 }, { 0x19c, 0x0cd3b023 }, { 0x19c, 0x5802be83 },
	{ 0x19c, 0x653c0190 }, { 0x19c, 0x013c2401 }, { 0x1a4, 0x00001c10 }, { 0x19c, 0x08d5b71c },
	{ 0x19c, 0x5601bf86 }, { 0x19c, 0x6038018e }, { 0x19c, 0x01351e01 }, { 0x1a4, 0x0000140c },
	{ 0x19c, 0x04d7be14 }, { 0x19c, 0x5401c087 }, { 0x19c, 0x5b35018b }, { 0x19c, 0x01311c01 },
	{ 0x1a4, 0x0000140b }, { 0x19c, 0x02d8c40d }, { 0x19c, 0x5601c089 }, { 0x19c, 0x6339018f },
	{ 0x19c, 0x01382001 }, { 0x1a4, 0x0000180d }, { 0x19c, 0x60d91dd3 }, { 0x19c, 0x2b169c2f },
	{ 0x19c, 0x00000057 }, { 0x19c, 0x00000000 }, { 0x1a4, 0x00000000 }, { 0x19c, 0x6fc1784e },
	{ 0x19c, 0x662eba74 }, { 0x19c, 0x80500fa4 }, { 0x19c, 0x014c3102 }, { 0x1a4, 0x00001c12 },
	{ 0x19c, 0x2acba147 }, { 0x19c, 0x620ac084 }, { 0x19c, 0x6d450396 }, { 0x19c, 0x01462c01 },
	{ 0x1a4, 0x00001d12 }, { 0x19c, 0x1ed3ba39 }, { 0x19c, 0x5d04c48c }, { 0x19c, 0x663e0192 },
	{ 0x19c, 0x01412601 }, { 0x1a4, 0x00001b10 }, { 0x19c, 0x0ed9c72f }, { 0x19c, 0x5801c491 },
	{ 0x19c, 0x6239018e }, { 0x19c, 0x013e2401 }, { 0x1a4, 0x00001a0f }, { 0x19c, 0x05e5db1a },
	{ 0x19c, 0x5e01cf9b }, { 0x19c, 0x683c0197 }, { 0x19c, 0x013e2401 }, { 0x1a4, 0x00001c10 },
	{ 0x19c, 0x92f81de9 }, { 0x19c, 0x342bdc2f }, { 0x19c, 0x0000008c }, { 0x19c, 0x00000000 },
	{ 0x1a4, 0x00000000 }, { 0x19c, 0xb3e8a364 }, { 0x19c, 0x8e3fdea1 }, { 0x19c, 0xae7125cc },
	{ 0x19c, 0x1289591a }, { 0x1a4, 0x00006144 }, { 0x19c, 0x20e6b555 }, { 0x19c, 0x6407d192 },
	{ 0x19c, 0x794703a4 }, { 0x19c, 0x014d2d01 }, { 0x1a4, 0x00001e12 }, { 0x19c, 0x14e6bb41 },
	{ 0x19c, 0x6102cf94 }, { 0x19c, 0x7444019f }, { 0x19c, 0x01462801 }, { 0x1a4, 0x00001d0e },
	{ 0x19c, 0x08e3c228 }, { 0x19c, 0x5e01cc93 }, { 0x19c, 0x7041019b }, { 0x19c, 0x01422701 },
	{ 0x1a4, 0x00001a0e }, { 0x19c, 0x03e4d010 }, { 0x19c, 0x6201cf97 }, { 0x19c, 0x754301a0 },
	{ 0x19c, 0x014a2901 }, { 0x1a4, 0x00001f11 }, { 0x19c, 0x078c2611 }, { 0x19c, 0x11015022 },
	{ 0x19c, 0x0000001d }, { 0x19c, 0x00000000 }, { 0x1a4, 0x00000000 }, { 0x19c, 0x29804b25 },
	{ 0x19c, 0x421a804c }, { 0x19c, 0x5e340c74 }, { 0x19c, 0x01372002 }, { 0x1a4, 0x0000100a },
	{ 0x19c, 0x259a7f32 }, { 0x19c, 0x5210986d }, { 0x19c, 0x553b0579 }, { 0x19c, 0x01362301 },
	{ 0x1a4, 0x0000140d }, { 0x19c, 0x11a78e28 }, { 0x19c, 0x47029d6e }, { 0x19c, 0x482c0170 },
	{ 0x19c, 0x012d1b01 }, { 0x1a4, 0x0000110b }, { 0x19c, 0x09bcaf1e }, { 0x19c, 0x4a01a97c },
	{ 0x19c, 0x4e300174 }, { 0x19c, 0x01311e01 }, { 0x1a4, 0x0000120b }, { 0x19c, 0x02dfde0a },
	{ 0x19c, 0x5301c296 }, { 0x19c, 0x4f300180 }, { 0x19c, 0x012d1b01 }, { 0x1a4, 0x0000110b },
	{ 0x19c, 0x1deb2924 }, { 0x19c, 0x1b0ac124 }, { 0x19c, 0x0000006f }, { 0x19c, 0x00000000 },
	{ 0x1a4, 0x00000000 }, { 0x19c, 0xb1dea555 }, { 0x19c, 0x876ed7a2 }, { 0x19c, 0xa87139c3 },
	{ 0x19c, 0x0a785317 }, { 0x1a4, 0x00003d31 }, { 0x19c, 0x24dfbe55 }, { 0x19c, 0x5a05c88b },
	{ 0x19c, 0x673c0192 }, { 0x19c, 0x01412601 }, { 0x1a4, 0x00001e12 }, { 0x19c, 0x17dfca48 },
	{ 0x19c, 0x5602c78d }, { 0x19c, 0x6138018c }, { 0x19c, 0x013d2401 }, { 0x1a4, 0x00001b10 },
	{ 0x19c, 0x0de1da37 }, { 0x19c, 0x5601c891 }, { 0x19c, 0x6339018d }, { 0x19c, 0x013d2301 },
	{ 0x1a4, 0x0000160d }, { 0x19c, 0x01d4eb0f }, { 0x19c, 0x5401b884 }, { 0x19c, 0x6139018b },
	{ 0x19c, 0x01382201 }, { 0x1a4, 0x0000170e }, { 0x19c, 0x3dc915b5 }, { 0x19c, 0x260a7b25 },
	{ 0x19c, 0x00000047 }, { 0x19c, 0x00000000 }, { 0x1a4, 0x00000000 }, { 0x19c, 0x5fac6a2f },
	{ 0x19c, 0x5d2aad68 }, { 0x19c, 0x834d129f }, { 0x19c, 0x01513204 }, { 0x1a4, 0x00001711 },
	{ 0x19c, 0x2cc7933e }, { 0x19c, 0x661cbd82 }, { 0x19c, 0x734b129a }, { 0x19c, 0x01412c02 },
	{ 0x1a4, 0x0000130c }, { 0x19c, 0x18d29937 }, { 0x19c, 0x5d03c282 }, { 0x19c, 0x613d0192 },
	{ 0x19c, 0x01321f01 }, { 0x1a4, 0x0000100a }, { 0x19c, 0x11dfba31 }, { 0x19c, 0x6001cc94 },
	{ 0x19c, 0x5335018e }, { 0x19c, 0x012c1a01 }, { 0x1a4, 0x0000110b }, { 0x19c, 0x02d4d90d },
	{ 0x19c, 0x4e01b488 }, { 0x19c, 0x5332017c }, { 0x19c, 0x01311d01 }, { 0x1a4, 0x0000170e },
	{ 0x19c, 0x52f70dc5 }, { 0x19c, 0x1119de11 }, { 0x19c, 0x000000a2 }, { 0x19c, 0x00000000 },
	{ 0x1a4, 0x00000000 }, { 0x19c, 0xeaf7ba7e }, { 0x19c, 0xb1b0f3bf }, { 0x19c, 0xdc9e68ea },
	{ 0x19c, 0x37ba8042 }, { 0x1a4, 0x0000895a }, { 0x19c, 0x2ef2c56f }, { 0x19c, 0x6809db9e },
	{ 0x19c, 0x7d4102ab }, { 0x19c, 0x01502c01 }, { 0x1a4, 0x00005b11 }, { 0x19c, 0x27f5d068 },
	{ 0x19c, 0x6d03e0a8 }, { 0x19c, 0x7c4f01a2 }, { 0x19c, 0x01663201 }, { 0x1a4, 0x0000662b },
	{ 0x19c, 0x1ff6dc54 }, { 0x19c, 0x7302e7b1 }, { 0x19c, 0x864f01b4 }, { 0x19c, 0x014d3701 },
	{ 0x1a4, 0x00004f3c }, { 0x19c, 0x08f0f32b }, { 0x19c, 0x7301d9b4 }, { 0x19c, 0x795401a6 },
	{ 0x19c, 0x01433301 }, { 0x1a4, 0x00000610 }, { 0x198, 0x000101d0 }, { 0x19c, 0xb04c0778 },
	{ 0x19c, 0x361c7ed0 }, { 0x19c, 0x00000067 }, { 0x19c, 0x00000000 }, { 0x19c, 0x9b9a0c30 },
	{ 0x19c, 0x75225a8b }, { 0x19c, 0x00000077 }, { 0x19c, 0x00000000 }, { 0x19c, 0xcc190643 },
	{ 0x19c, 0x150d9ef3 }, { 0x19c, 0x00000060 }, { 0x19c, 0x00000000 }, { 0x19c, 0x832c0561 },
	{ 0x19c, 0x44308bb0 }, { 0x19c, 0x00000061 }, { 0x19c, 0x00000000 }, { 0x19c, 0x9c2a0553 },
	{ 0x19c, 0x311a986f }, { 0x19c, 0x00000098 }, { 0x19c, 0x00000000 }, { 0x19c, 0xb23a0550 },
	{ 0x19c, 0x3e21534a }, { 0x19c, 0x00000091 }, { 0x19c, 0x00000000 }, { 0x19c, 0x9a200556 },
	{ 0x19c, 0x160ea8c0 }, { 0x19c, 0x000000a3 }, { 0x19c, 0x00000000 }, { 0x19c, 0x9c200555 },
	{ 0x19c, 0x1d1394d8 }, { 0x19c, 0x00000049 }, { 0x19c, 0x00000000 }, { 0x19c, 0x7440074d },
	{ 0x19c, 0x7e257a84 }, { 0x19c, 0x00000078 }, { 0x19c, 0x00000000 }, { 0x19c, 0xb56b1565 },
	{ 0x19c, 0x431367c0 }, { 0x19c, 0x0000007d }, { 0x19c, 0x00000000 }, { 0x198, 0x00010180 },
	{ 0x19c, 0x90122041 }, { 0x19c, 0x3329c2a2 }, { 0x19c, 0x00000062 }, { 0x19c, 0x00000000 },
	{ 0x19c, 0xa5124484 }, { 0x19c, 0x282dc4d9 }, { 0x19c, 0x0000004e }, { 0x19c, 0x00000000 },
	{ 0x19c, 0xb01350ad }, { 0x19c, 0x2340c1f0 }, { 0x19c, 0x0000002e }, { 0x19c, 0x00000000 },
	{ 0x19c, 0xc22687dd }, { 0x19c, 0x556079f8 }, { 0x19c, 0x0000001d }, { 0x19c, 0x00000000 },
	{ 0x198, 0x00010190 }, { 0x19c, 0x0000a2eb }, { 0x19c, 0x00000000 }, { 0x19c, 0x00000000 },
	{ 0x19c, 0x00000000 }, { 0x19c, 0x0000ff24 }, { 0x19c, 0x00000000 }, { 0x19c, 0x00000000 },
	{ 0x19c, 0x00000000 }, { 0x19c, 0x00000322 }, { 0x19c, 0x00000000 }, { 0x19c, 0x00000000 },
	{ 0x19c, 0x00000000 }, { 0x19c, 0x00009095 }, { 0x19c, 0x00000000 }, { 0x19c, 0x00000000 },
	{ 0x19c, 0x00000000 }, { 0x1a0, 0x00010060 }, { 0x1a4, 0x008d7ac7 }, { 0x1a4, 0x009f3f93 },
	{ 0x1a4, 0x00768594 }, { 0x1a4, 0x00726879 }, { 0x1a4, 0x005749ae }, { 0x1a4, 0x0053295c },
	{ 0x1a4, 0x00326352 }, { 0x1a4, 0x00272735 }, { 0x1a4, 0x003b3ab1 }, { 0x1a4, 0x003f1a44 },
	{ 0x1a4, 0x00194f34 }, { 0x1a4, 0x000c0e11 }, { 0x1a4, 0x001e22de }, { 0x1a4, 0x002c1048 },
	{ 0x1a4, 0x000c203a }, { 0x1a4, 0x0006070a }, { 0x1a0, 0x0001007e }, { 0x1a4, 0xe1bb6609 },
	{ 0x198, 0x000101a8 }, { 0x19c, 0x6077b7ef }, { 0x19c, 0x00000029 }, { 0x198, 0x000101ac },
	{ 0x19c, 0xdd7b7e32 }, { 0x19c, 0x000000e2 }, { 0x198, 0x000101b0 }, { 0x19c, 0x4a4d1021 },
	{ 0x19c, 0xaaac8e8e }, { 0x19c, 0x0000f7ee }, { 0x1a0, 0x00010078 }, { 0x1a4, 0x00000064 },
	{ 0x1a4, 0x00000042 }, { 0x1a4, 0x00009814 }, { 0x1a4, 0x0000650f }, { 0x1a4, 0x00258803 },
	{ 0x1a4, 0x000d3405 }, { 0x1a0, 0x00010077 }, { 0x1a4, 0x004080c0 }, { 0x1a0, 0x00010070 },
	{ 0x1a4, 0x0022ad02 }, { 0x1a4, 0x00559107 }, { 0x1a4, 0x003fa607 }, { 0x1a4, 0x00425e07 },
	{ 0x1a4, 0x002e4008 }, { 0x1a4, 0x001f5111 }, { 0x1a4, 0x001e1d19 }, { 0x198, 0x000101b4 },
	{ 0x19c, 0x00604020 }, { 0x198, 0x000101b8 }, { 0x19c, 0xc090e080 }, { 0x19c, 0xc0b0c0a8 },
	{ 0x19c, 0x00f5c6c6 }, { 0x19c, 0x00000000 }, { 0x19c, 0xb080d880 }, { 0x19c, 0xc0b0b0a0 },
	{ 0x19c, 0x00d0c6c6 }, { 0x19c, 0x00000000 }, { 0x19c, 0x948c88d8 }, { 0x19c, 0xe0c0b0a0 },
	{ 0x19c, 0x00f0eaea }, { 0x19c, 0x00000000 }, { 0x19c, 0x948c88d0 }, { 0x19c, 0xe0c0b0a0 },
	{ 0x19c, 0x00f0eaea }, { 0x19c, 0x00000000 }, { 0x19c, 0x60408080 }, { 0x19c, 0x60404070 },
	{ 0x19c, 0x0080a040 }, { 0x19c, 0x00000000 }, { 0x19c, 0x60408080 }, { 0x19c, 0x60404070 },
	{ 0x19c, 0x0080a040 }, { 0x19c, 0x00000000 },
};

struct vdec_vp9_mt8365_inst {
	struct mtk_vcodec_dec_ctx *ctx;
	struct mtk_vcodec_mem colmv_buf;	/* colocated MV scratch */
	struct mtk_vcodec_mem cnt_buf;		/* backward-adaptation counts */
	struct mtk_vcodec_mem prob_buf;		/* backward-adaptation probs */
	struct mtk_vcodec_mem seg_buf;		/* per-SB64 segment-id map */
	struct mtk_vcodec_mem tile_buf;		/* > 8 tiles: descriptor surface */
	bool work_buf_ready;
	unsigned int frm_cnt;
	u32 pic_w, pic_h;

	/* The four VP9 frame contexts, in upload-template order. */
	u32 fc[VP9_NUM_FC][VP9_PROB_WORDS];

	/*
	 * Loop-filter deltas PERSIST across frames (VP9 spec 6.2.8: they are
	 * only rewritten by a loop_filter_delta_update, and only reset by
	 * setup_past_independence()). Keeping them as locals is the exact bug
	 * that cost the VP8 arc a round on segment feature data.
	 */
	s8 ref_delta[4];
	s8 mode_delta[2];

	/*
	 * Segmentation state (VP9 spec 6.2.11 / 7.2.10). Like the loop-filter
	 * deltas, ALL OF IT PERSISTS: the feature table is rewritten only by a
	 * segmentation_update_data, the tree / pred probabilities only by a
	 * segmentation_update_map, and setup_past_independence() is the only
	 * thing that clears the feature table. MEASURED: the two inter frames
	 * of segon-176.ivf code update_map = 0 and update_data = 0 and the
	 * vendor still writes the KEYFRAME's tree probabilities and the
	 * keyframe's per-segment dequantisers on both of them.
	 */
	u8 seg_tree_probs[7];
	u8 seg_pred_probs[3];
	/*
	 * temporal_update PERSISTS too, and unlike the rest that is MEASURED:
	 * vp90-2-19-skip's frame 1 codes update_map = 1 + temporal_update = 1
	 * and the vendor writes +0xa8 = 0x84000701 (bits 8|9|10); its frames 2
	 * and 3 code update_map = 0 -- so no temporal_update bit is read at all
	 * -- and the vendor still writes bit10, 0x00000501 / 0x82000501. The
	 * vendor's parser clears only its update_map / update_data bytes on
	 * entry, never the temporal one.
	 */
	bool seg_temporal;
	u8 seg_feat_en[8];		/* bit j = FeatureEnabled[i][j] */
	s16 seg_feat_data[8][4];
	bool seg_abs_delta;

	/* Previous decoded frame, for UsePrevFrameMvs. */
	u32 prev_w, prev_h;
	bool prev_show, prev_valid;
	/* LastFrameType, for the coefficient merge's update factor (8.4). */
	bool prev_key;

	/* The forward-updated context the hardware hands back (WDMA mode 2). */
	u32 hwfc[VP9_PROB_WORDS];
	/* Counts-layout self-check violations on the last adapted frame. */
	u32 cnt_bad;

	/* Per-CAPTURE-buffer picture size, for the MC ref-dimension registers. */
	u32 fb_w[VB2_MAX_FRAME];
	u32 fb_h[VB2_MAX_FRAME];
};

/*
 * The post-decode probability read-back -- MEASURED end to end.
 *
 * The VP9_VLD+0x1dc MODE-2 WDMA run (the one the vendor points at the
 * allocation its symbol names call the counts table) writes the new frame
 * context to DRAM as 640 words. Mode 1 writes nothing usable (all zero in
 * every dump).
 *
 * The 640-word surface is the two probability SRAMs in the hardware's own
 * serialization, which is the upload template's order for the coefficient
 * records and ADDRESS order for the tail:
 *
 *   dump[k]                        = template word k          for k < 480
 *                                    (the coefficient records: four bank-A
 *                                     words then one bank-B word, 20 B each)
 *   dump[480 + (addrA - 0x180)]    = bank A word addrA        (0x180..0x1f7)
 *   dump[608 + (addrB - 0x60)]     = bank B word addrB        (0x60..0x7f)
 *
 * PROOF: with that map, our frame 0's mode-2 dump is word-for-word EQUAL to
 * the VENDOR's frame 1 UPLOAD for the same clip -- ndiff = 0 over all 615
 * words. The hardware hands back exactly the context the next frame loads, so
 * save_probs() needs no software probability model.
 *
 * vp9_prob_wdma_word[] is that map, built once by replaying the template.
 */
static u16 vp9_prob_wdma_word[VP9_PROB_WORDS];
static bool vp9_prob_map_ready;

static void vp9_build_prob_map(void)
{
	u32 pa = 0, pb = 0;
	int i, k = 0;

	for (i = 0; i < (int)ARRAY_SIZE(vp9_kf_prob_seq); i++) {
		u16 off = vp9_kf_prob_seq[i].off;
		u32 val = vp9_kf_prob_seq[i].val;

		if (off == V9_PROB_PTR_A) {
			pa = val & 0xffff;
		} else if (off == V9_PROB_PTR_B) {
			pb = val & 0xffff;
		} else if (off == V9_PROB_DATA_A) {
			vp9_prob_wdma_word[k] = (k < VP9_PROB_COEF_WORDS) ? k :
				(VP9_PROB_COEF_WORDS + (u16)(pa - 0x180));
			k++;
			pa++;
		} else if (off == V9_PROB_DATA_B) {
			vp9_prob_wdma_word[k] = (k < VP9_PROB_COEF_WORDS) ? k :
				(VP9_PROB_BANK_B_BASE + (u16)(pb - 0x60));
			k++;
			pb++;
		}
	}
	vp9_prob_map_ready = true;
}

/* Reset one frame context to the VP9 defaults (setup_past_independence). */
static void vp9_fc_default(struct vdec_vp9_mt8365_inst *inst, int idx)
{
	int i, k = 0;

	for (i = 0; i < (int)ARRAY_SIZE(vp9_kf_prob_seq); i++)
		if (vp9_kf_prob_seq[i].off == V9_PROB_DATA_A ||
		    vp9_kf_prob_seq[i].off == V9_PROB_DATA_B)
			inst->fc[idx][k++] = vp9_kf_prob_seq[i].val;
}

/* Replay the template's pointer writes, with @data supplying every data word. */
static void vp9_upload_probs(struct mtk_vcodec_dec_ctx *ctx, const u32 *data)
{
	void __iomem *base = ctx->dev->reg_base[VDEC_HWV];
	int i, k = 0;

	for (i = 0; i < (int)ARRAY_SIZE(vp9_kf_prob_seq); i++) {
		u16 off = vp9_kf_prob_seq[i].off;

		if (off == V9_PROB_PTR_A || off == V9_PROB_PTR_B)
			writel(vp9_kf_prob_seq[i].val, base + off);
		else
			writel(data[k++], base + off);
	}
}

/* Pull the hardware's post-decode (forward-updated) context out of the dump. */
static void vp9_read_probs(struct vdec_vp9_mt8365_inst *inst, u32 *out)
{
	const u32 *w = inst->prob_buf.va;
	int k;

	if (!w)
		return;
	for (k = 0; k < VP9_PROB_WORDS; k++)
		out[k] = w[vp9_prob_wdma_word[k]];
}

/* save_probs(): pull the hardware's post-decode context out of the WDMA dump. */
static void vp9_save_probs(struct vdec_vp9_mt8365_inst *inst, int idx)
{
	vp9_read_probs(inst, inst->fc[idx]);
}

/*
 * ======================= VP9 spec 8.4 backward adaptation =================
 *
 * TWO maps are needed and both are now MEASURED, not assumed.
 *
 * (a) THE FRAME-CONTEXT BYTE MAP. Our frame context is the 615 packed hardware
 *     SRAM words the upload template writes (2460 bytes), NOT a software
 *     probability model -- deliberately, because that is what let save_probs()
 *     need no model at all. Section 8.4 however is defined on NAMED probability
 *     fields, so the merge needs a byte map.
 *
 *     It was recovered by matching the VP9 spec / libvpx DEFAULT probability
 *     tables against the template's bytes, which IS the defaults (a keyframe
 *     runs setup_past_independence()). VALIDATION, and it is total: rebuilding
 *     every named field through the map below reproduces
 *     vp9_default_coef_probs (all 1584 values) and every field of
 *     vp9_default_*_probs (311 values) EXACTLY -- ndiff = 0 over 1895 bytes --
 *     and every one of the 565 bytes the map does NOT claim is ZERO in the
 *     template. So the map accounts for the whole context with nothing left
 *     over and nothing overlapping.
 *
 *     Coefficients: 96 records of 20 bytes, record = (tx, plane, ref, band)
 *     nested in that order, band 0 carrying 3 contexts (9 B used) and bands
 *     1..5 six (18 B used). That is words 0..479 -- exactly the boundary
 *     VP9_PROB_COEF_WORDS already marked for a different reason (the WDMA
 *     serialization changes there), which is an independent corroboration.
 *     Everything else is a small strided block in the 540-byte tail.
 *
 *     All indices are in LIBVPX order, which is what both the template and the
 *     counts surface use: intra modes DC,V,H,D45,D135,D117,D153,D207,D63,TM;
 *     partition contexts with the 8x8 group first.
 *
 * (b) THE COUNTS-SURFACE WORD MAP -- transcribed from the vendor's own
 *     VP9_Get_Counts_Table (aarch64 disassembly of libvpud_vcodec.so), whose
 *     destination offsets are libvpx's FRAME_COUNTS verbatim. See the audit.
 *
 * The arithmetic is stock libvpx (merge_probs / mode_mv_merge_probs), which the
 * vendor's VP9_Adapt_Mode_Probs corroborates by passing COUNT_SAT = 20 and
 * MAX_UPDATE_FACTOR = 128 for the non-coefficient merges.
 */

/* --- (a) frame-context byte offsets --- */
#define FCB_COEF_REC		20	/* bytes per (tx,plane,ref,band) record */
#define FCB_UV_MODE		1920	/* [10][9]   stride 16 */
#define FCB_Y_MODE		2080	/* [4][9]    stride 16 */
#define FCB_FILTER		2144	/* [4][2]    stride 16 */
#define FCB_PARTITION		2208	/* [16][3]   stride 4 */
#define FCB_INTRA_INTER		2272	/* [4] */
#define FCB_COMP_INTER		2276	/* [5] */
#define FCB_COMP_REF		2284	/* [5] */
#define FCB_SINGLE_REF		2292	/* [5][2]    packed */
#define FCB_TX8P		2304	/* [2]       stride 4 */
#define FCB_TX16P		2312	/* [2][2]    stride 4 */
#define FCB_TX32P		2320	/* [2][3]    stride 4 */
#define FCB_SKIP		2328	/* [3] */
#define FCB_INTER_MODE		2332	/* [7][3]    stride 4 */
#define FCB_MV_JOINT		2360	/* [3] */
#define FCB_MV_SIGN		2364	/* [2]       stride FCB_MV_STRIDE */
#define FCB_MV_CLASSES		2365	/* [2][10] */
#define FCB_MV_CLASS0		2396	/* [2] */
#define FCB_MV_BITS		2397	/* [2][10] */
#define FCB_MV_CLASS0_FP	2428	/* [2][2][3] */
#define FCB_MV_FP		2434	/* [2][3] */
#define FCB_MV_CLASS0_HP	2437	/* [2] */
#define FCB_MV_HP		2438	/* [2] */
#define FCB_MV_STRIDE		16	/* between the two MV components */
#define FCB_MODE_STRIDE		16	/* between y_mode / uv_mode / filter rows */

/* The intra-frame carve-out: tx8p..skip, which 8.4 does NOT adapt on an intra
 * frame -- those keep the FORWARD-updated values instead. One contiguous run. */
#define FCB_INTRA_KEEP_OFF	FCB_TX8P
#define FCB_INTRA_KEEP_LEN	(FCB_SKIP + 3 - FCB_TX8P)

/* --- (b) counts-surface word offsets --- */
#define CNT_EOB			0	/* 16 groups of 36 words */
#define CNT_COEF		1024	/* 16 groups of 132 words */
#define CNT_INTRA_INTER		3136	/* [4][2] */
#define CNT_COMP_INTER		3144	/* [5][2] */
#define CNT_COMP_REF		3156	/* [5][2] */
#define CNT_SINGLE_REF		3168	/* [5][2][2] */
#define CNT_INTER_MODE		3188	/* [7][4] */
#define CNT_Y_MODE		3216	/* [4][10]  stride 12 */
#define CNT_UV_MODE		3264	/* [10][10] packed */
#define CNT_PARTITION		3364	/* [16][4]  packed */
#define CNT_INTERP		3428	/* [4][3]   stride 4 */
#define CNT_TX8X8		3444	/* [2][2]   packed */
#define CNT_TX16X16		3448	/* [2][3]   stride 4 */
#define CNT_TX32X32		3456	/* [2][4]   packed */
#define CNT_SKIP		3464	/* [3][2]   stride 4 */
#define CNT_MV_JOINT		3476	/* [4] */
#define CNT_MV_COMP		3480	/* per component, stride CNT_MV_STRIDE */
#define CNT_MV_STRIDE		56
#define CNT_MVC_SIGN		0	/* [2] */
#define CNT_MVC_CLASS0		2	/* [2] */
#define CNT_MVC_CLASSES		4	/* [11] */
#define CNT_MVC_BITS		16	/* [10][2] */
#define CNT_MVC_CLASS0_FP	40	/* [2][4] */
#define CNT_MVC_FP		48	/* [4] */
#define CNT_MVC_CLASS0_HP	52	/* [2] */
#define CNT_MVC_HP		54	/* [2] */

/* Coefficient merge constants (libvpx COEF_COUNT_SAT / *_MAX_UPDATE_FACTOR). */
#define VP9_COEF_COUNT_SAT	24
#define VP9_COEF_UF		112
#define VP9_COEF_UF_AFTER_KEY	128
/* Non-coefficient constants -- transcribed from VP9_Adapt_Mode_Probs. */
#define VP9_MODE_COUNT_SAT	20
#define VP9_MODE_UF		128

/*
 * merge_probs() of the VP9 spec / libvpx:
 *   den = ct0 + ct1; prob = clip((ct0*256 + den/2) / den, 1, 255)
 *   factor = max_uf * min(den, count_sat) / count_sat
 *   out = ROUND_POWER_OF_TWO(pre*(256-factor) + prob*factor, 8)
 * The last line is written in the algebraically identical incremental form so
 * the multiply stays inside an int.
 */
static void vp9_merge(u8 *p, u32 ct0, u32 ct1, u32 count_sat, u32 max_uf)
{
	u32 den = ct0 + ct1, factor;
	int p1, p2;

	if (!den)
		return;
	factor = max_uf * min(den, count_sat) / count_sat;
	p2 = (int)div_u64(((u64)ct0 << 8) + (den >> 1), den);
	p2 = clamp(p2, 1, 255);
	p1 = *p;
	*p = (u8)(p1 + (((p2 - p1) * (int)factor + 128) >> 8));
}

static void vp9_merge_mode(u8 *p, u32 ct0, u32 ct1)
{
	vp9_merge(p, ct0, ct1, VP9_MODE_COUNT_SAT, VP9_MODE_UF);
}

/* One 4-leaf tree {a,b,c,d} over three probability nodes. */
static void vp9_merge_tree4(u8 *pp, u32 a, u32 b, u32 c, u32 d)
{
	vp9_merge_mode(&pp[0], a, b + c + d);
	vp9_merge_mode(&pp[1], b, c + d);
	vp9_merge_mode(&pp[2], c, d);
}

/* vp9_intra_mode_tree, counts in LIBVPX mode order. */
static void vp9_merge_intra_mode(u8 *pp, const u32 *c)
{
	u32 sum, s2;
	int i;

	for (sum = 0, i = 1; i < 10; i++)
		sum += c[i];
	vp9_merge_mode(&pp[0], c[0], sum);		/* DC   vs rest */
	sum -= c[9];
	vp9_merge_mode(&pp[1], c[9], sum);		/* TM   vs rest */
	sum -= c[1];
	vp9_merge_mode(&pp[2], c[1], sum);		/* V    vs rest */
	s2 = c[2] + c[4] + c[5];			/* H, D135, D117 */
	sum -= s2;
	vp9_merge_mode(&pp[3], s2, sum);
	s2 -= c[2];
	vp9_merge_mode(&pp[4], c[2], s2);		/* H    vs D135/D117 */
	vp9_merge_mode(&pp[5], c[4], c[5]);		/* D135 vs D117 */
	sum -= c[3];
	vp9_merge_mode(&pp[6], c[3], sum);		/* D45  vs rest */
	sum -= c[8];
	vp9_merge_mode(&pp[7], c[8], sum);		/* D63  vs rest */
	vp9_merge_mode(&pp[8], c[6], c[7]);		/* D153 vs D207 */
}

/*
 * Run the whole of VP9 spec 8.4 on frame_contexts[@idx], IN PLACE.
 *
 * 8.4 merges from the PRE context -- frame_contexts[frame_context_idx] as it
 * stood BEFORE this frame's forward updates -- which is exactly what @idx still
 * holds (it is what we uploaded). The forward-updated context the hardware
 * hands back is needed only for the fields 8.4 leaves alone on an intra frame.
 *
 * Fields whose syntax was not coded this frame have all-zero counts, and
 * merge_probs of an empty count pair is the identity, so the "adapt only when
 * reference_mode/tx_mode/interp_filter says so" gates of libvpx are
 * unnecessary for correctness. The two gates we CAN evaluate cheaply
 * (tx_mode, interp_filter) are kept anyway; reference_mode is decoded inside
 * the hardware and is not visible to us.
 */
static void vp9_adapt_probs(struct vdec_vp9_mt8365_inst *inst, int idx,
			    const u32 *hwfc, bool intra_frame, bool after_key,
			    u32 tx_mode, u32 interp, bool allow_hp)
{
	u8 *p = (u8 *)inst->fc[idx];
	const u32 *cnt = inst->cnt_buf.va;
	u32 uf = after_key ? VP9_COEF_UF_AFTER_KEY : VP9_COEF_UF;
	int tx, pl, rf, band, ctx, i, j, g;

	if (!cnt)
		return;
	inst->cnt_bad = 0;

	/* ---- coefficient probabilities (every frame type) ---- */
	for (tx = 0; tx < 4; tx++) {
		for (pl = 0; pl < 2; pl++) {
			for (rf = 0; rf < 2; rf++) {
				g = (tx * 2 + pl) * 2 + rf;
				for (band = 0; band < 6; band++) {
					int nctx = band ? 6 : 3;

					for (ctx = 0; ctx < nctx; ctx++) {
						const u32 *cc;
						u32 eo, neob;
						u8 *pp;

						eo = cnt[CNT_EOB + g * 36 +
							 (band ? 4 + (band - 1) * 6 + ctx
							       : ctx)];
						cc = &cnt[CNT_COEF + g * 132 +
							  (band ? 12 + (band - 1) * 24
								: 0) + ctx * 4];
						pp = &p[(g * 6 + band) * FCB_COEF_REC
							+ ctx * 3];
						neob = cc[3];
						/*
						 * SELF-CHECK of the layout. The
						 * EOB-token count is a subset of
						 * the EOB-node evaluations, so
						 * neob <= eob_branch must hold at
						 * every one of the 1584 entries;
						 * a mis-indexed group or stride
						 * breaks it immediately. MEASURED
						 * 0 violations over 6 frames.
						 * (The token counts are NOT a
						 * second such identity: the
						 * zero-run loop increments
						 * ZERO_TOKEN at band/contexts
						 * whose EOB node was never
						 * evaluated.)
						 */
						if (neob > eo)
							inst->cnt_bad++;
						vp9_merge(&pp[0], neob,
							  eo > neob ? eo - neob : 0,
							  VP9_COEF_COUNT_SAT, uf);
						vp9_merge(&pp[1], cc[0], cc[1] + cc[2],
							  VP9_COEF_COUNT_SAT, uf);
						vp9_merge(&pp[2], cc[1], cc[2],
							  VP9_COEF_COUNT_SAT, uf);
					}
				}
			}
		}
	}

	if (intra_frame) {
		/*
		 * 8.4 on an intra frame adapts the coefficient probabilities
		 * only; skip and the tx probabilities keep their FORWARD-updated
		 * values, which is what the hardware handed back.
		 */
		memcpy(p + FCB_INTRA_KEEP_OFF,
		       (const u8 *)hwfc + FCB_INTRA_KEEP_OFF, FCB_INTRA_KEEP_LEN);
		return;
	}

	/* ---- single-node fields ---- */
	for (i = 0; i < 3; i++)
		vp9_merge_mode(&p[FCB_SKIP + i],
			       cnt[CNT_SKIP + i * 4], cnt[CNT_SKIP + i * 4 + 1]);
	for (i = 0; i < 4; i++)
		vp9_merge_mode(&p[FCB_INTRA_INTER + i],
			       cnt[CNT_INTRA_INTER + i * 2],
			       cnt[CNT_INTRA_INTER + i * 2 + 1]);
	for (i = 0; i < 5; i++) {
		vp9_merge_mode(&p[FCB_COMP_INTER + i],
			       cnt[CNT_COMP_INTER + i * 2],
			       cnt[CNT_COMP_INTER + i * 2 + 1]);
		vp9_merge_mode(&p[FCB_COMP_REF + i],
			       cnt[CNT_COMP_REF + i * 2],
			       cnt[CNT_COMP_REF + i * 2 + 1]);
		for (j = 0; j < 2; j++)
			vp9_merge_mode(&p[FCB_SINGLE_REF + i * 2 + j],
				       cnt[CNT_SINGLE_REF + i * 4 + j * 2],
				       cnt[CNT_SINGLE_REF + i * 4 + j * 2 + 1]);
	}

	/* ---- partition ---- */
	for (i = 0; i < 16; i++) {
		const u32 *c = &cnt[CNT_PARTITION + i * 4];

		vp9_merge_tree4(&p[FCB_PARTITION + i * 4], c[0], c[1], c[2], c[3]);
	}

	/* ---- transform size ---- */
	if (tx_mode == 4) {			/* TX_MODE_SELECT */
		for (i = 0; i < 2; i++) {
			const u32 *c8 = &cnt[CNT_TX8X8 + i * 2];
			const u32 *c16 = &cnt[CNT_TX16X16 + i * 4];
			const u32 *c32 = &cnt[CNT_TX32X32 + i * 4];
			u8 *pp;

			vp9_merge_mode(&p[FCB_TX8P + i * 4], c8[0], c8[1]);
			pp = &p[FCB_TX16P + i * 4];
			vp9_merge_mode(&pp[0], c16[0], c16[1] + c16[2]);
			vp9_merge_mode(&pp[1], c16[1], c16[2]);
			pp = &p[FCB_TX32P + i * 4];
			vp9_merge_tree4(pp, c32[0], c32[1], c32[2], c32[3]);
		}
	}

	/* ---- interpolation filter ---- */
	if (interp == VP9_SWITCHABLE_FILTER) {
		for (i = 0; i < 4; i++) {
			const u32 *c = &cnt[CNT_INTERP + i * 4];
			u8 *pp = &p[FCB_FILTER + i * FCB_MODE_STRIDE];

			vp9_merge_mode(&pp[0], c[0], c[1] + c[2]);
			vp9_merge_mode(&pp[1], c[1], c[2]);
		}
	}

	/* ---- inter modes: the tree is {ZERO, NEAREST, NEAR, NEW} and the
	 * counts are in INTER_OFFSET order {NEAREST, NEAR, ZERO, NEW} ---- */
	for (i = 0; i < 7; i++) {
		const u32 *c = &cnt[CNT_INTER_MODE + i * 4];
		u8 *pp = &p[FCB_INTER_MODE + i * 4];

		vp9_merge_mode(&pp[0], c[2], c[0] + c[1] + c[3]);
		vp9_merge_mode(&pp[1], c[0], c[1] + c[3]);
		vp9_merge_mode(&pp[2], c[1], c[3]);
	}

	/* ---- motion vectors ---- */
	{
		const u32 *c = &cnt[CNT_MV_JOINT];

		vp9_merge_tree4(&p[FCB_MV_JOINT], c[0], c[1], c[2], c[3]);
	}
	for (i = 0; i < 2; i++) {
		const u32 *cb = &cnt[CNT_MV_COMP + i * CNT_MV_STRIDE];
		int pb = i * FCB_MV_STRIDE;
		const u32 *cc;
		u8 *pp;
		u32 sum;

		vp9_merge_mode(&p[FCB_MV_SIGN + pb],
			       cb[CNT_MVC_SIGN], cb[CNT_MVC_SIGN + 1]);

		cc = &cb[CNT_MVC_CLASSES];
		pp = &p[FCB_MV_CLASSES + pb];
		for (sum = 0, j = 1; j < 11; j++)
			sum += cc[j];
		vp9_merge_mode(&pp[0], cc[0], sum);
		sum -= cc[1];
		vp9_merge_mode(&pp[1], cc[1], sum);
		sum -= cc[2] + cc[3];
		vp9_merge_mode(&pp[2], cc[2] + cc[3], sum);
		vp9_merge_mode(&pp[3], cc[2], cc[3]);
		sum -= cc[4] + cc[5];
		vp9_merge_mode(&pp[4], cc[4] + cc[5], sum);
		vp9_merge_mode(&pp[5], cc[4], cc[5]);
		sum -= cc[6];
		vp9_merge_mode(&pp[6], cc[6], sum);
		vp9_merge_mode(&pp[7], cc[7] + cc[8], cc[9] + cc[10]);
		vp9_merge_mode(&pp[8], cc[7], cc[8]);
		vp9_merge_mode(&pp[9], cc[9], cc[10]);

		vp9_merge_mode(&p[FCB_MV_CLASS0 + pb],
			       cb[CNT_MVC_CLASS0], cb[CNT_MVC_CLASS0 + 1]);
		for (j = 0; j < 10; j++)
			vp9_merge_mode(&p[FCB_MV_BITS + pb + j],
				       cb[CNT_MVC_BITS + j * 2],
				       cb[CNT_MVC_BITS + j * 2 + 1]);
		for (j = 0; j < 2; j++) {
			cc = &cb[CNT_MVC_CLASS0_FP + j * 4];
			vp9_merge_tree4(&p[FCB_MV_CLASS0_FP + pb + j * 3],
					cc[0], cc[1], cc[2], cc[3]);
		}
		cc = &cb[CNT_MVC_FP];
		vp9_merge_tree4(&p[FCB_MV_FP + pb], cc[0], cc[1], cc[2], cc[3]);
		if (allow_hp) {
			vp9_merge_mode(&p[FCB_MV_CLASS0_HP + pb],
				       cb[CNT_MVC_CLASS0_HP],
				       cb[CNT_MVC_CLASS0_HP + 1]);
			vp9_merge_mode(&p[FCB_MV_HP + pb],
				       cb[CNT_MVC_HP], cb[CNT_MVC_HP + 1]);
		}
	}

	/* ---- intra modes ---- */
	for (i = 0; i < 4; i++)
		vp9_merge_intra_mode(&p[FCB_Y_MODE + i * FCB_MODE_STRIDE],
				     &cnt[CNT_Y_MODE + i * 12]);
	for (i = 0; i < 10; i++)
		vp9_merge_intra_mode(&p[FCB_UV_MODE + i * FCB_MODE_STRIDE],
				     &cnt[CNT_UV_MODE + i * 10]);
}

/* Software mirror of the HW barrel shifter's raw-bit bank. */
struct vp9_bits {
	struct mtk_vcodec_dec_ctx *ctx;
	u32 win;
	u32 pos;
};

static inline void wr(struct mtk_vcodec_dec_ctx *ctx, int blk, u32 off, u32 val)
{
	if (unlikely(mt8365_vp9_quirk & 0x100))
		pr_info("vp9w b%d +%03x = %08x\n", blk, off, val);
	writel(val, ctx->dev->reg_base[blk] + off);
}

static inline u32 rd(struct mtk_vcodec_dec_ctx *ctx, int blk, u32 off)
{
	u32 v = readl(ctx->dev->reg_base[blk] + off);

	/*
	 * quirk 0x100 logs READS as well as writes. A read is a first-class
	 * programming step on this engine -- the bool-literal bank at
	 * VLD9+0x84 CONSUMES bitstream on read (N26) -- so a write-only trace
	 * is structurally blind to a whole class of defect. Paired with
	 * tests/mt8365-vp9-tracediff.py's read-aware modes.
	 */
	if (unlikely(mt8365_vp9_quirk & 0x100))
		pr_info("vp9r b%d +%03x = %08x\n", blk, off, v);
	return v;
}

/*
 * Read @n raw bits of the uncompressed header. The port returns the barrel
 * shifter's window AFTER consuming, so the field value is the top @n bits of
 * the window we already hold. The extra peek at +0x00 mirrors the vendor's
 * access pattern exactly (it re-peeks after every consume).
 */
static u32 vp9_f(struct vp9_bits *b, int n)
{
	u32 v = n ? (b->win >> (32 - n)) : 0;

	b->win = rd(b->ctx, VDEC_HWV, V9_BITS_RAW + (n << 2));
	(void)rd(b->ctx, VDEC_HWV, V9_BITS_RAW);
	b->pos += n;
	return v;
}

/* VP9 su(n): magnitude f(n) then sign f(1). */
static int vp9_su(struct vp9_bits *b, int n)
{
	int v = (int)vp9_f(b, n);

	return vp9_f(b, 1) ? -v : v;
}

/* Bool-decoder literal on the compressed header (value-returning bank). */
static inline u32 vp9_lit(struct mtk_vcodec_dec_ctx *ctx, int n)
{
	return rd(ctx, VDEC_HWV, V9_BITS_BOOL + (n << 2));
}

/*
 * ---------------------------------------------------------------------------
 * A HOST-SIDE software bool decoder for the compressed header.
 *
 * reference_mode (spec 6.3.12) decides which of the +0xc8 sub-steps
 * comp_inter / single_ref / comp_ref the hardware must be pulsed for.
 *
 * IT IS READ THROUGH THE HARDWARE BOOL BANK (a +0x84 = vp9_lit(1) read, and a
 * second one when the first is 1), between the 0x20 and the 0x80 sub-steps --
 * MEASURED in the vendor trace, 2026-07-28 (see vp9_dec_decode()). The earlier
 * "NOT readable / DOUBLE-CONSUMES" reading was an artefact of reading the bits
 * at the wrong point in the sub-step chain.
 *
 * The vendor host ALSO runs its own software bool decoder over the compressed
 * header in parallel with the hardware (VP9_InitBool at the head of
 * VP9_Compressed_Header_Parse @0x76e10). This is that decoder; it is kept as a
 * cross-check on the hardware read (and as the quirk-0x10000 fallback).
 *
 * Everything on the path to reference_mode -- read_tx_mode, read_tx_mode_probs,
 * read_coef_probs, read_skip_prob, read_inter_mode_probs,
 * read_switchable_interp_probs, read_intra_inter_probs -- is decoded with
 * vp9_diff_update_prob(), which reads at a FIXED probability (252) and whose
 * payload (decode_term_subexp) needs no probability context either. So the
 * prefix parse is entirely self-contained: no frame context, no defaults, no
 * state carried between frames. Only the bit COUNT matters, never the values,
 * so no probability is stored.
 * ---------------------------------------------------------------------------
 */
struct vp9_bool {
	const u8 *buf;
	u32 size;
	u32 pos;
	u32 value;
	u32 range;
	int bit_count;
};

static u8 vp9_bool_byte(struct vp9_bool *d)
{
	u8 b = d->pos < d->size ? d->buf[d->pos] : 0;

	d->pos++;
	return b;
}

static void vp9_bool_init(struct vp9_bool *d, const u8 *buf, u32 size)
{
	d->buf = buf;
	d->size = size;
	d->pos = 0;
	d->value = (u32)vp9_bool_byte(d) << 8;
	d->value |= vp9_bool_byte(d);
	d->bit_count = 0;
	d->range = 255;
}

/*
 * RFC 6386 §7.3. VP9's split formula ((range * prob + (256 - prob)) >> 8) is
 * algebraically identical to VP8's (1 + (((range - 1) * prob) >> 8)).
 */
static int vp9_bool_read(struct vp9_bool *d, int prob)
{
	u32 split = 1 + (((d->range - 1) * (u32)prob) >> 8);
	u32 big = split << 8;
	int bit;

	if (d->value >= big) {
		bit = 1;
		d->range -= split;
		d->value -= big;
	} else {
		bit = 0;
		d->range = split;
	}
	while (d->range < 128) {
		d->value <<= 1;
		d->range <<= 1;
		if (++d->bit_count == 8) {
			d->bit_count = 0;
			d->value |= vp9_bool_byte(d);
		}
	}
	return bit;
}

static inline int vp9_bool_bit(struct vp9_bool *d)
{
	return vp9_bool_read(d, 128);
}

static u32 vp9_bool_lit(struct vp9_bool *d, int n)
{
	u32 v = 0;

	while (n--)
		v = (v << 1) | (u32)vp9_bool_bit(d);
	return v;
}

/* decode_term_subexp(): the value is discarded, only the bit count matters. */
static void vp9_bool_term_subexp(struct vp9_bool *d)
{
	u32 v;

	if (!vp9_bool_bit(d)) {
		vp9_bool_lit(d, 4);
		return;
	}
	if (!vp9_bool_bit(d)) {
		vp9_bool_lit(d, 4);
		return;
	}
	if (!vp9_bool_bit(d)) {
		vp9_bool_lit(d, 5);
		return;
	}
	v = vp9_bool_lit(d, 7);
	if (v >= 65)
		vp9_bool_bit(d);
}

static void vp9_bool_diff_update(struct vp9_bool *d)
{
	if (vp9_bool_read(d, 252))	/* MODE_UPDATE_PROB */
		vp9_bool_term_subexp(d);
}

/* tx_mode_to_biggest_tx_size[]: ONLY_4X4 .. TX_MODE_SELECT */
static const u8 vp9_biggest_tx[5] = { 0, 1, 2, 3, 3 };

/*
 * Decode the compressed-header prefix of an INTER frame and return its
 * reference_mode (VP9_SINGLE_REFERENCE / _COMPOUND_REFERENCE /
 * _REFERENCE_MODE_SELECT), or a negative errno. *tx_mode_out receives the
 * software-decoded tx_mode, which the caller cross-checks against the
 * hardware-read one.
 */
static int vp9_sw_reference_mode(const u8 *hdr, u32 len, bool lossless,
				 bool interp_switchable, u32 *tx_mode_out)
{
	struct vp9_bool d;
	u32 tx_mode = 0;		/* ONLY_4X4 when lossless */
	int i, j, k, l, m, tx;

	if (!hdr || !len)
		return -EINVAL;
	vp9_bool_init(&d, hdr, len);
	if (vp9_bool_bit(&d))		/* the compressed header's marker bit */
		return -EINVAL;
	if (!lossless) {
		tx_mode = vp9_bool_lit(&d, 2);
		if (tx_mode == 3)
			tx_mode += vp9_bool_bit(&d);
	}
	*tx_mode_out = tx_mode;
	if (tx_mode == 4) {		/* TX_MODE_SELECT: read_tx_mode_probs() */
		for (i = 0; i < 2; i++)
			for (j = 0; j < 1; j++)
				vp9_bool_diff_update(&d);
		for (i = 0; i < 2; i++)
			for (j = 0; j < 2; j++)
				vp9_bool_diff_update(&d);
		for (i = 0; i < 2; i++)
			for (j = 0; j < 3; j++)
				vp9_bool_diff_update(&d);
	}
	for (tx = 0; tx <= vp9_biggest_tx[tx_mode]; tx++) {	/* coef probs */
		if (!vp9_bool_bit(&d))
			continue;
		for (i = 0; i < 2; i++)			/* PLANE_TYPES */
			for (j = 0; j < 2; j++)		/* REF_TYPES */
				for (k = 0; k < 6; k++)	/* COEF_BANDS */
					for (l = 0; l < (k == 0 ? 3 : 6); l++)
						for (m = 0; m < 3; m++)
							vp9_bool_diff_update(&d);
	}
	for (i = 0; i < 3; i++)			/* SKIP_CONTEXTS */
		vp9_bool_diff_update(&d);
	for (i = 0; i < 7; i++)			/* INTER_MODE_CONTEXTS */
		for (j = 0; j < 3; j++)		/* INTER_MODES - 1 */
			vp9_bool_diff_update(&d);
	if (interp_switchable)
		for (i = 0; i < 4; i++)		/* SWITCHABLE_FILTER_CONTEXTS */
			for (j = 0; j < 2; j++)	/* SWITCHABLE_FILTERS - 1 */
				vp9_bool_diff_update(&d);
	for (i = 0; i < 4; i++)			/* INTRA_INTER_CONTEXTS */
		vp9_bool_diff_update(&d);
	/* frame_reference_mode(), spec 6.3.12 */
	if (!vp9_bool_bit(&d))
		return VP9_SINGLE_REFERENCE;
	return vp9_bool_bit(&d) ? VP9_REFERENCE_MODE_SELECT
				: VP9_COMPOUND_REFERENCE;
}

static void free_work_bufs(struct vdec_vp9_mt8365_inst *inst)
{
	if (inst->colmv_buf.va)
		mtk_vcodec_mem_free(inst->ctx, &inst->colmv_buf);
	if (inst->cnt_buf.va)
		mtk_vcodec_mem_free(inst->ctx, &inst->cnt_buf);
	if (inst->prob_buf.va)
		mtk_vcodec_mem_free(inst->ctx, &inst->prob_buf);
	if (inst->seg_buf.va)
		mtk_vcodec_mem_free(inst->ctx, &inst->seg_buf);
	if (inst->tile_buf.va)
		mtk_vcodec_mem_free(inst->ctx, &inst->tile_buf);
	inst->work_buf_ready = false;
}

/*
 * colMV store size for a given picture ceiling, from the vendor allocator
 * Vdec_Drv_VP9_AllocBuf @0x79780 (transcribed in N4): 576 bytes per 64x64
 * superblock plus a 1152-byte header.  At the built-in 2048x1088 ceiling this
 * is 314496 B, inside VP9_COLMV_BUF_SZ; a raised ceiling needs more.
 */
static u32 vp9_colmv_sz(void)
{
	u32 sb = DIV_ROUND_UP(vp9_max_w(), 64) * DIV_ROUND_UP(vp9_max_h(), 64);
	u32 need = ALIGN(sb * 576 + 1152, PAGE_SIZE);

	return max_t(u32, need, VP9_COLMV_BUF_SZ);
}

/* Segment-id map: (sb64_cols * sb64_rows + 8) * 32, same allocator. */
static u32 vp9_segmap_sz(void)
{
	u32 sb = DIV_ROUND_UP(vp9_max_w(), 64) * DIV_ROUND_UP(vp9_max_h(), 64);
	u32 need = ALIGN((sb + 8) * 32, PAGE_SIZE);

	return max_t(u32, need, VP9_SEGMAP_BUF_SZ);
}

static int alloc_work_bufs(struct vdec_vp9_mt8365_inst *inst)
{
	inst->colmv_buf.size = vp9_colmv_sz();
	inst->cnt_buf.size = VP9_WDMA_BUF_SZ;
	inst->prob_buf.size = VP9_WDMA_BUF_SZ;
	inst->seg_buf.size = vp9_segmap_sz();
	inst->tile_buf.size = VP9_TILEDESC_BUF_SZ;
	if (mtk_vcodec_mem_alloc(inst->ctx, &inst->colmv_buf) ||
	    mtk_vcodec_mem_alloc(inst->ctx, &inst->cnt_buf) ||
	    mtk_vcodec_mem_alloc(inst->ctx, &inst->prob_buf) ||
	    mtk_vcodec_mem_alloc(inst->ctx, &inst->tile_buf) ||
	    mtk_vcodec_mem_alloc(inst->ctx, &inst->seg_buf)) {
		free_work_bufs(inst);
		return -ENOMEM;
	}
	memset(inst->seg_buf.va, 0, inst->seg_buf.size);
	inst->work_buf_ready = true;
	return 0;
}

/*
 * setup_past_independence(): reset the loop-filter deltas to the libvpx
 * defaults and (per the caller's rules) the frame contexts to the spec
 * defaults. VP9 spec 7.2 / 8.4.
 */
static void vp9_past_independence(struct vdec_vp9_mt8365_inst *inst)
{
	inst->ref_delta[0] = 1;
	inst->ref_delta[1] = 0;
	inst->ref_delta[2] = -1;
	inst->ref_delta[3] = -1;
	inst->mode_delta[0] = 0;
	inst->mode_delta[1] = 0;
	/*
	 * VP9 spec 7.2: setup_past_independence() also clears the segmentation
	 * feature table, resets SegmentationAbsOrDeltaUpdate, and zeroes
	 * PrevSegmentIds. The hardware keeps the previous frame's map in the
	 * +0x118 surface (the vendor points the read and the write port at the
	 * SAME buffer), so clearing PrevSegmentIds means clearing that buffer.
	 */
	memset(inst->seg_feat_en, 0, sizeof(inst->seg_feat_en));
	memset(inst->seg_feat_data, 0, sizeof(inst->seg_feat_data));
	inst->seg_abs_delta = false;
	inst->seg_temporal = false;
	if (inst->seg_buf.va)
		memset(inst->seg_buf.va, 0, inst->seg_buf.size);
}

/*
 * VP9 spec 8.6.1 get_qindex(): the per-segment quantiser index, ALT_Q being
 * either an absolute value or a delta on base_q_idx. Transcribed from the
 * vendor's VP9_Get_Qindex (libvpud_vcodec.so @0x761b0), which is libvpx's.
 */
static u32 vp9_seg_qindex(const struct vdec_vp9_mt8365_inst *inst,
			  bool seg_enabled, int seg, u32 base_q)
{
	int data;

	if (!seg_enabled || !(inst->seg_feat_en[seg] & BIT(VP9_SEG_LVL_ALT_Q)))
		return base_q;
	data = inst->seg_feat_data[seg][VP9_SEG_LVL_ALT_Q];
	if (!inst->seg_abs_delta)
		data += (int)base_q;
	return clamp_t(int, data, 0, 255);
}

/*
 * VP9 spec 6.2.14 get_tile_offset(): the MI-unit start of tile @idx along an
 * axis of @mis MI units split into 1 << @log2 tiles. NOT an even split -- the
 * division is done in SB64 units and then scaled back, so the tiles differ in
 * width whenever Sb64Cols is not a multiple of the tile count.
 */
static u32 vp9_tile_offset(u32 idx, u32 mis, u32 log2)
{
	u32 sbs = (mis + 7) >> 3;
	u32 off = ((idx * sbs) >> log2) << 3;

	return min(off, mis);
}

static int vdec_vp9_mt8365_init(struct mtk_vcodec_dec_ctx *ctx)
{
	struct vdec_vp9_mt8365_inst *inst;
	int i;

	inst = kzalloc(sizeof(*inst), GFP_KERNEL);
	if (!inst)
		return -ENOMEM;
	inst->ctx = ctx;
	if (!vp9_prob_map_ready)
		vp9_build_prob_map();
	for (i = 0; i < VP9_NUM_FC; i++)
		vp9_fc_default(inst, i);
	vp9_past_independence(inst);
	ctx->drv_handle = inst;
	return 0;
}

/*
 * HW enable + soft reset, trace seq 0-10. Structurally the VP8/H.264 sequence
 * with VP9's own MISC magic values (MISC_SPEC_CON 0xfdff vs VP8's 0xff7f,
 * MISC_FUNC_CON 0x71e31182 vs 0x71201102, MISC_CFG178 0xfffff041 vs
 * 0xfffffffb, MISC_SYS_SEL 2 vs 1, MISC_STATE 3 written once up front).
 */
static void vp9_hw_enable(struct mtk_vcodec_dec_ctx *ctx)
{
	wr(ctx, VDEC_SYS, SYS_CKEN_SET, 0x1);
	wr(ctx, VDEC_MISC, MISC_SRST, 0x0);
	wr(ctx, VDEC_MISC, MISC_DCM_CON, 0x1);
	wr(ctx, VDEC_SYS, SYS_DCM_CON, 0x1);
	wr(ctx, VDEC_LD, VLD_SRST, 0x101);
	wr(ctx, VDEC_MISC, MISC_SPEC_CON, 0x0000fdff);
	wr(ctx, VDEC_MISC, MISC_FUNC_CON, 0x71e31182);
	wr(ctx, VDEC_MISC, MISC_CFG178, 0xfffff041);
	wr(ctx, VDEC_MISC, MISC_SYS_SEL, 0x2);
	wr(ctx, VDEC_LD, VLD_SRST, 0x0);
	wr(ctx, VDEC_MISC, MISC_STATE, 0x3);
}

/*
 * Run one compressed-header sub-parse. The vendor trace shows
 * { W +0xc8 = mask; W +0xc8 = 0; R +0xcc } with no visible poll -- but every
 * traced access goes through the LD_PRELOAD interposer, which fprintf()s a line
 * to a log file between accesses, so the vendor's "no poll" is milliseconds of
 * accidental delay. Back-to-back writel()s do not give the HW time to
 * bool-decode a ~1200-bit probability update. Wait for the command register to
 * self-clear (the same shape as VP8's HWD_COMMIT 0x148 / HWD_MODE_COMMIT 0x14c
 * polls) before writing the explicit 0 the vendor writes.
 */
static void vp9_hdr_parse(struct mtk_vcodec_dec_ctx *ctx, u32 mask)
{
	u32 prev = ~0u, pos;
	int i;

	wr(ctx, VDEC_HWV, V9_HDR_PARSE, mask);
	/*
	 * Wait for the sub-parse to RUN TO COMPLETION: the bit position must
	 * have stopped advancing AND the status register must read the idle
	 * pattern 0x3fff. Neither condition alone is sufficient -- 0x3fff is
	 * also the value the status holds before a pulse starts, and the
	 * position is momentarily stable while the engine refills.
	 */
	for (i = 0; i < 20000; i++) {
		udelay(1);
		pos = rd(ctx, VDEC_HWV, V9_BITPOS);
		if (pos == prev && rd(ctx, VDEC_HWV, V9_HDR_STATUS) == 0x3fff)
			break;
		prev = pos;
	}
	wr(ctx, VDEC_HWV, V9_HDR_PARSE, 0x0);
	if (mt8365_vp9_diag)
		pr_info("vp9 hdrparse mask=%x spins=%d cc=%08x bitpos=%u\n",
			mask, i, rd(ctx, VDEC_HWV, V9_HDR_STATUS),
			rd(ctx, VDEC_HWV, V9_BITPOS));
	else
		(void)rd(ctx, VDEC_HWV, V9_HDR_STATUS);
}

/* Pulse VP9_VLD+0x1a8 and wait for the HW to clear it (trace seq 12-13). */
static void vp9_sync(struct mtk_vcodec_dec_ctx *ctx)
{
	int i;

	wr(ctx, VDEC_HWV, V9_SYNC, 0x1);
	for (i = 0; i < 100000; i++) {
		if (!rd(ctx, VDEC_HWV, V9_SYNC))
			return;
		cpu_relax();
	}
}

/*
 * Arm BOTH bitstream fetch blocks over the AU and run one fetch. VP9 is
 * dual-VLD: the common VLD (VDEC_LD) carries the header streams and BS2
 * (VDEC_HWB) the tile data, and the vendor programs the same window into both
 * (trace seq 14-34).
 */
static void vp9_arm_fetch(struct mtk_vcodec_dec_ctx *ctx, dma_addr_t start,
			  dma_addr_t end)
{
	int i;

	wr(ctx, VDEC_LD, VLD_WIN_START, (u32)(start >> 6));
	wr(ctx, VDEC_LD, VLD_WIN_END, (u32)(end >> 6));
	wr(ctx, VDEC_HWB, VLD_WIN_START, (u32)(start >> 6));
	wr(ctx, VDEC_HWB, VLD_WIN_END, (u32)(end >> 6));
	wr(ctx, VDEC_LD, VLD_ENABLE_EC,
	   rd(ctx, VDEC_LD, VLD_ENABLE_EC) | 0x10000000);
	wr(ctx, VDEC_HWB, VLD_ENABLE_EC,
	   rd(ctx, VDEC_HWB, VLD_ENABLE_EC) | 0x10000000);
	(void)rd(ctx, VDEC_LD, VLD_IDLE);
	wr(ctx, VDEC_LD, VLD_FETCH_BASE, (u32)start);
	wr(ctx, VDEC_LD, VLD_WIN_END_RAW, (u32)end);
	/* The vendor asserts the VLD soft reset FIVE times before releasing it. */
	for (i = 0; i < 5; i++)
		wr(ctx, VDEC_LD, VLD_SRST, 0x100);
	wr(ctx, VDEC_LD, VLD_SRST, 0x0);
	wr(ctx, VDEC_LD, VLD_KICK, 0x00100000);
	for (i = 0; i < 8192; i++) {
		if (rd(ctx, VDEC_LD, VLD_FETCH_DONE) & 0x1)
			break;
		cpu_relax();
	}
	wr(ctx, VDEC_LD, VLD_KICK, 0x00800000);
}

/*
 * Run one post-decode WDMA (VP9_VLD+0x1dc mode / +0x1e0 DRAM address /
 * +0x1e8 go, +0x1ec done).
 *
 * +0x1ec bit0 is ALREADY SET when the run is kicked (measured: pre=00000001 on
 * every frame), so a bare "poll until bit0" returns immediately and the caller
 * reads a buffer the DMA has not finished writing -- the same class of
 * asynchronous-handshake trap as the +0xc8 compressed-header sub-parse. Give
 * the engine a settle window before believing the flag, and another after it.
 */
static u32 vp9_wdma(struct mtk_vcodec_dec_ctx *ctx, u32 mode, u32 addr)
{
	int i;

	wr(ctx, VDEC_HWV, V9_WDMA_MODE, mode);
	wr(ctx, VDEC_HWV, V9_WDMA_ADDR, addr);
	wr(ctx, VDEC_HWV, V9_WDMA_GO, mode);
	udelay(200);
	for (i = 0; i < 20000; i++) {
		if (rd(ctx, VDEC_HWV, V9_WDMA_DONE) & 0x1)
			break;
		udelay(1);
	}
	udelay(50);
	return i;
}

/*
 * Resolve the three active reference surfaces + their picture sizes.
 *
 * The stateless API contract does the 8-slot pool bookkeeping for us:
 * V4L2_CID_STATELESS_VP9_FRAME carries last_frame_ts / golden_frame_ts /
 * alt_frame_ts as the TIMESTAMPS of the buffers holding LAST / GOLDEN / ALTREF
 * -- already selected by ref_frame_idx[] -- and both queues are
 * TIMESTAMP_COPY, so vb2_find_buffer()
 * on the CAPTURE queue returns the surface. This is what mainline
 * vdec_vp9_req_lat_if.c does, and it is why refresh_frame_flags needs no
 * driver-side slot array. The reference SIZE has no home in the control, so the
 * backend records the decoded size per CAPTURE buffer index.
 *
 * Falls back to the current recon for all three when the control is absent.
 */
static void vp9_resolve_refs(struct vdec_vp9_mt8365_inst *inst,
			     u32 rec_y, u32 rec_c,
			     u32 ref_y[3], u32 ref_c[3],
			     u32 ref_w[3], u32 ref_h[3])
{
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	const struct v4l2_ctrl_vp9_frame *hdr = NULL;
	struct v4l2_ctrl *ctrl;
	struct vb2_queue *vq;
	u64 ts[3];
	int i;

	for (i = 0; i < 3; i++) {
		ref_y[i] = rec_y;
		ref_c[i] = rec_c;
		ref_w[i] = inst->prev_valid ? inst->prev_w : 0;
		ref_h[i] = inst->prev_valid ? inst->prev_h : 0;
	}

	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, V4L2_CID_STATELESS_VP9_FRAME);
	if (ctrl)
		hdr = ctrl->p_cur.p;
	if (!hdr)
		return;

	ts[0] = hdr->last_frame_ts;
	ts[1] = hdr->golden_frame_ts;
	ts[2] = hdr->alt_frame_ts;
	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	if (!vq)
		return;
	for (i = 0; i < 3; i++) {
		struct vb2_buffer *vb = vb2_find_buffer(vq, ts[i]);
		const dma_addr_t *y, *c;

		if (!vb || vb->num_planes < 2)
			continue;
		/*
		 * vb2_dma_contig_plane_dma_addr() dereferences the plane cookie
		 * WITHOUT a NULL check, and vb2_find_buffer() will happily
		 * return a buffer whose planes are not (or no longer) backed --
		 * which is exactly what an SVC stream does when a reference
		 * timestamp resolves to a buffer the queue has already torn
		 * down across a spatial-layer change. Oopsed on
		 * vp90-2-22-svc_1280x720_1 (2026-07-28,
		 * `Unable to handle kernel read from unreadable memory at
		 * virtual address 0` at vp9_resolve_refs+0x118, i.e. the
		 * `ldr x2, [x2]` right after vb2_plane_cookie). Fall back to
		 * the recon buffer, as for a missing reference.
		 */
		y = vb2_plane_cookie(vb, 0);
		c = vb2_plane_cookie(vb, 1);
		if (!y || !c)
			continue;
		ref_y[i] = (u32)*y;
		ref_c[i] = (u32)*c;
		if (vb->index < VB2_MAX_FRAME && inst->fb_w[vb->index]) {
			ref_w[i] = inst->fb_w[vb->index];
			ref_h[i] = inst->fb_h[vb->index];
		}
	}
}

/*
 * ---- Scaled references (VP9 spec 8.5.2.3) ----
 *
 * An inter frame may reference a picture of a DIFFERENT size; the MC then
 * scales on the fly. The hardware does NOT derive the scale from the
 * per-reference geometry it is already given (MC+0xd4c / +0xd70) -- it has to
 * be told, through nine registers this backend did not previously write.
 * Transcribed from libvpud_vcodec.so (mt8365, aarch64):
 *
 *   VP9_Setup_Scale_Factors_For_Frame @0x75890  x_scale_fp = (ref_w << 14) / cur_w
 *   VP9_Get_Fixed_Point_Scale_Factor  @0x75820  y_scale_fp = (ref_h << 14) / cur_h
 *   VP9_Scaled_X / _Y                 @0x75830  step_q4    = (16 * fp) >> 14
 *   VP9_Is_Scaled                     @0x75860  fp != 1<<14 on either axis
 *   VP9_Check_Scale_Factors           @0x757f0  ref <= 2x and >= 1/16 the frame
 *   VP9_Max_RRF_BLK_Cal               @0x76f54  the per-ref prefetch-block code
 *   VP9_Set_MC_Ref_Scaling_Enable     @0x7c170  MC+0xd80
 *   VP9_Set_MC_Ref_Scaling_Step       @0x7c200  MC+0xd40 / 0xd44 / 0xd48
 *   VP9_Set_MC_Set_Scaling_Factor     @0x7c250  MC+0xd58/5c, 0xd60/64, 0xd68/6c
 *   VP9_Set_MC_MAX_RRF_BLK_Size       @0x7c3b0  MC+0xd7c
 *   the caller loop                   @0x7c930  VP9_Set_Pic_Info
 *
 * Both packed registers put reference 0 in the HIGH field:
 *   MC+0xd7c = (rrfx0<<24)|(rrfx1<<20)|(rrfx2<<16)|(rrfy0<<8)|(rrfy1<<4)|rrfy2
 *   MC+0xd80 = (scaled0<<8)|(scaled1<<4)|scaled2
 * which is why the all-unscaled case is the long-standing 0x07770777 / 0x0.
 * The vendor writes the step/factor registers ONLY for a scaled reference.
 */
#define VP9_REF_SCALE_SHIFT	14
#define VP9_REF_NO_SCALE	(1 << VP9_REF_SCALE_SHIFT)

/* VP9_Check_Scale_Factors(): libvpx valid_ref_frame_size(). */
static bool vp9_valid_ref_size(u32 ref_w, u32 ref_h, u32 cur_w, u32 cur_h)
{
	return ref_w <= 2 * cur_w && ref_h <= 2 * cur_h &&
	       cur_w <= 16 * ref_w && cur_h <= 16 * ref_h;
}

static u32 vp9_scale_fp(u32 other, u32 cur)
{
	return cur ? (other << VP9_REF_SCALE_SHIFT) / cur : VP9_REF_NO_SCALE;
}

/*
 * VP9_Max_RRF_BLK_Cal(): 4 while the reference is at most ~1.001x the frame,
 * otherwise 3 -- except for a razor-thin window right at a 1.8x reference,
 * which gives 0. The unscaled path never calls it (VP9_Set_Pic_Info hardcodes
 * 7 there), and the first return guarantees the subtraction cannot wrap.
 *
 * The middle test is a `ccmp ..., #0x0, hi` + `csel ..., gt`: when the `hi`
 * condition is FALSE the flags are FORCED to 0, and NZCV=0 satisfies `gt`, so
 * a SHORT (v>>14) yields 3, not 0. Getting that backwards is invisible on a
 * 1/2-scale resize (ratio 2.0 answers 3 either way) and shows up only on a
 * 3/4-scale one (ratio 4/3), where it wedged the MC into a decode timeout.
 */
static u32 vp9_max_rrf_blk(u32 scale_fp)
{
	u32 v = scale_fp * 1000;

	/*
	 * Bring-up sweep knob for the UPSCALING-reference case (the reference
	 * SMALLER than the current frame, scale_fp < 1<<14). Every scaled
	 * reference validated so far is a DOWNSCALE -- the whole vp90-2-21
	 * family (24/24) only ever shrinks and never returns to full size --
	 * so the `return 4` branch below has never been exercised by a passing
	 * vector. -1 keeps the transcribed value.
	 */
	if (mt8365_vp9_rrf_up >= 0 && scale_fp < VP9_REF_NO_SCALE)
		return (u32)mt8365_vp9_rrf_up & 0xf;
	if (v <= 0xfa3fffu)
		return 4;
	if ((v >> VP9_REF_SCALE_SHIFT) - 1001u <= 0x31eu)
		return 3;
	return v > 0x1c1ffffu ? 3 : 0;
}

static int vdec_vp9_mt8365_decode(void *h_vdec, struct mtk_vcodec_mem *bs,
				  struct vdec_fb *unused, bool *res_chg)
{
	struct vdec_vp9_mt8365_inst *inst = h_vdec;
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	struct vp9_bits b = { .ctx = ctx };
	struct vdec_fb *fb;
	dma_addr_t bs_end;
	u32 pic_w, pic_h, aw, ah, sb_cols, sb_rows, mi_cols, mi_rows;
	u32 rec_y, rec_c, colmv;
	u32 base_q, lf_level, lf_sharp, tile_start, base_off, tx_mode = 0;
	u32 cfgbc;
	u32 hdr_size, uh_bytes, crc0;
	u32 pre174, pre15c, prea4, spins, hdrpos;
	u32 min_log2_tile_cols, max_log2_tile_cols, tile_cols_log2, tile_rows_log2;
	u32 ref_y[3], ref_c[3], ref_w[3], ref_h[3];
	u32 refresh_flags = 0xff, reset_fc = 0, fc_idx = 0, flags9, scale_en = 0;
	u32 interp = VP9_SWITCHABLE_FILTER, interp_type = VP9_SWITCHABLE_FILTER;
	u32 masks[16];
	u32 ref_mode = VP9_SINGLE_REFERENCE, ref_mode_word = 0;
	u32 comp_fixed_ref = 0, comp_var_ref[2] = { 0, 0 };
	u8 ref_idx[3] = { 0, 0, 0 }, sign_bias[3] = { 0, 0, 0 };
	int dq_ydc, dq_uvdc, dq_uvac;
	bool lossless, lf_delta_en, keyframe, intra_only, err_res, show_frame;
	bool refresh_fc = false, frame_parallel = true, use_prev_mvs = false;
	bool compound_allowed = false, found_ref = false, allow_hp = false;
	bool seg_en = false, seg_upd_map = false;
	u32 seg_ac = 0, seg_lf_en = 0, seg_lf03 = 0, seg_lf47 = 0;
	bool adapt_en = !(mt8365_vp9_quirk & 0x400);
	u32 wdma_pre[2], wdma_post[2], wdma_spin[2];
	int i, ret, nmask = 0, npre = 0;

	*res_chg = false;
	if (!bs)			/* flush / EOS */
		return 0;

	fb = ctx->dev->vdec_pdata->get_cap_buffer(ctx);
	if (!fb)
		return -EBUSY;

	{
		struct mtk_video_dec_buf *src_buf_info =
			container_of(bs, struct mtk_video_dec_buf, bs_buffer);
		struct mtk_video_dec_buf *dst_buf_info =
			container_of(fb, struct mtk_video_dec_buf, frame_buffer);

		v4l2_m2m_buf_copy_metadata(&src_buf_info->m2m_buf.vb,
					   &dst_buf_info->m2m_buf.vb, true);
	}

	if (!bs->va || bs->size < 16)
		return -EINVAL;

	/*
	 * Cheap software pre-check of the frame tag before touching the HW: the
	 * HW parse below is authoritative (and is what positions the barrel
	 * shifter), but a non-profile-0 stream or a show_existing_frame must be
	 * rejected before any register is programmed.
	 */
	{
		const u8 *p = bs->va;
		u32 w0 = ((u32)p[0] << 24) | ((u32)p[1] << 16) |
			 ((u32)p[2] << 8) | p[3];

		if ((w0 >> 30) != 0x2) {
			mtk_v4l2_vdec_err(ctx, "mt8365 vp9: bad frame marker");
			return -EINVAL;
		}
		if ((w0 >> 29) & 1 || (w0 >> 28) & 1) {
			mtk_v4l2_vdec_err(ctx, "mt8365 vp9: profile != 0");
			return -EINVAL;
		}
		if ((w0 >> 27) & 1) {
			mtk_v4l2_vdec_err(ctx, "mt8365 vp9: show_existing_frame");
			return -EINVAL;
		}
	}

	if (!inst->work_buf_ready) {
		ret = alloc_work_bufs(inst);
		if (ret)
			return ret;
	}

	/* N45: audit + barrier -- see mtk_vcodec_mt8365_bs_audit(). */
	mtk_vcodec_mt8365_bs_audit(ctx, bs->dma_addr, bs->size);

	rec_y = (u32)fb->base_y.dma_addr;
	rec_c = (u32)fb->base_c.dma_addr;
	colmv = (u32)inst->colmv_buf.dma_addr;
	/* MEASURED 6/6: the fetch window ends ALIGN(size, 64) + 448 past base. */
	bs_end = bs->dma_addr + ALIGN(bs->size, 64) + 448;

	/* ---- trace seq 0-34: HW enable, sync pulse, dual-VLD fetch ---- */
	vp9_hw_enable(ctx);
	wr(ctx, VDEC_HWV, V9_CTRL, 0x1);
	vp9_sync(ctx);
	vp9_arm_fetch(ctx, bs->dma_addr, bs_end);

	/* ---- seq 35-37: barrel-shifter init, then the uncompressed header ---- */
	wr(ctx, VDEC_HWV, V9_BSHIFT_INIT, 0x1);
	wr(ctx, VDEC_HWV, V9_BSHIFT_INIT, 0x0);
	(void)rd(ctx, VDEC_HWV, V9_BITPOS);

	b.win = rd(ctx, VDEC_HWV, V9_BITS_RAW);
	vp9_f(&b, 2);				/* frame_marker */
	vp9_f(&b, 1);				/* profile_low_bit */
	vp9_f(&b, 1);				/* profile_high_bit */
	vp9_f(&b, 1);				/* show_existing_frame */
	keyframe = !vp9_f(&b, 1);		/* frame_type */
	show_frame = vp9_f(&b, 1);
	err_res = vp9_f(&b, 1);			/* error_resilient_mode */
	intra_only = false;
	if (keyframe) {
		if (vp9_f(&b, 8) != 0x49 || vp9_f(&b, 8) != 0x83 ||
		    vp9_f(&b, 8) != 0x42) {
			mtk_v4l2_vdec_err(ctx, "mt8365 vp9: bad frame sync code");
			return -EINVAL;
		}
		if (vp9_f(&b, 3) != 7)		/* color_space; 7 == CS_SRGB */
			vp9_f(&b, 1);		/* color_range */
		pic_w = vp9_f(&b, 16) + 1;
		pic_h = vp9_f(&b, 16) + 1;
		if (vp9_f(&b, 1)) {		/* render_and_frame_size_different */
			vp9_f(&b, 16);
			vp9_f(&b, 16);
		}
		refresh_flags = 0xff;
	} else {
		if (!show_frame)
			intra_only = vp9_f(&b, 1);
		if (!err_res)
			reset_fc = vp9_f(&b, 2);
		if (intra_only) {
			if (vp9_f(&b, 8) != 0x49 || vp9_f(&b, 8) != 0x83 ||
			    vp9_f(&b, 8) != 0x42) {
				mtk_v4l2_vdec_err(ctx,
					"mt8365 vp9: bad intra-only sync code");
				return -EINVAL;
			}
			/* profile 0 => the 4:2:0 8-bit colour defaults */
			refresh_flags = vp9_f(&b, 8);
			pic_w = vp9_f(&b, 16) + 1;
			pic_h = vp9_f(&b, 16) + 1;
			if (vp9_f(&b, 1)) {
				vp9_f(&b, 16);
				vp9_f(&b, 16);
			}
		} else {
			refresh_flags = vp9_f(&b, 8);
			for (i = 0; i < 3; i++) {
				ref_idx[i] = vp9_f(&b, 3);
				sign_bias[i] = vp9_f(&b, 1);
			}
			/*
			 * frame_size_with_refs(): the size may be INHERITED
			 * from a reference rather than coded, so the reference
			 * surfaces must be resolved before the parse can
			 * continue.
			 */
			vp9_resolve_refs(inst, rec_y, rec_c, ref_y, ref_c,
					 ref_w, ref_h);
			pic_w = 0;
			pic_h = 0;
			for (i = 0; i < 3; i++) {
				if (vp9_f(&b, 1)) {
					found_ref = true;
					pic_w = ref_w[i];
					pic_h = ref_h[i];
					break;
				}
			}
			if (!found_ref) {
				pic_w = vp9_f(&b, 16) + 1;
				pic_h = vp9_f(&b, 16) + 1;
			}
			if (!pic_w || !pic_h) {
				mtk_v4l2_vdec_err(ctx,
					"mt8365 vp9: inter frame with no usable reference size");
				return -EINVAL;
			}
			if (vp9_f(&b, 1)) {	/* render_size_different */
				vp9_f(&b, 16);
				vp9_f(&b, 16);
			}
			allow_hp = vp9_f(&b, 1);	/* allow_high_precision_mv */
			/* read_interpolation_filter() */
			interp = vp9_f(&b, 1) ? VP9_SWITCHABLE_FILTER
					      : vp9_f(&b, 2);
			interp_type = interp == VP9_SWITCHABLE_FILTER ?
				VP9_SWITCHABLE_FILTER : vp9_lit2type[interp];
		}
	}
	if (!err_res) {
		refresh_fc = vp9_f(&b, 1);
		frame_parallel = vp9_f(&b, 1);
	} else {
		refresh_fc = false;
		frame_parallel = true;
	}
	fc_idx = vp9_f(&b, 2);
	/*
	 * VP9 spec 6.2 / 7.2: an intra or error-resilient frame runs
	 * setup_past_independence(), resets the saved contexts per
	 * reset_frame_context, and then decodes from context 0.
	 */
	if (keyframe || intra_only || err_res) {
		vp9_past_independence(inst);
		if (keyframe || err_res || reset_fc == 3) {
			for (i = 0; i < VP9_NUM_FC; i++)
				vp9_fc_default(inst, i);
		} else if (reset_fc == 2) {
			vp9_fc_default(inst, fc_idx);
		}
		fc_idx = 0;
	}
	/* loop_filter_params() -- the deltas PERSIST in the instance */
	lf_level = vp9_f(&b, 6);
	lf_sharp = vp9_f(&b, 3);
	lf_delta_en = vp9_f(&b, 1);
	if (lf_delta_en && vp9_f(&b, 1)) {	/* loop_filter_delta_update */
		for (i = 0; i < 4; i++)
			if (vp9_f(&b, 1))
				inst->ref_delta[i] = (s8)vp9_su(&b, 6);
		for (i = 0; i < 2; i++)
			if (vp9_f(&b, 1))
				inst->mode_delta[i] = (s8)vp9_su(&b, 6);
	}
	/* quantization_params() */
	base_q = vp9_f(&b, 8);
	dq_ydc  = vp9_f(&b, 1) ? vp9_su(&b, 4) : 0;
	dq_uvdc = vp9_f(&b, 1) ? vp9_su(&b, 4) : 0;
	dq_uvac = vp9_f(&b, 1) ? vp9_su(&b, 4) : 0;
	lossless = (base_q == 0 && dq_ydc == 0 && dq_uvdc == 0 && dq_uvac == 0);
	/*
	 * segmentation_params() -- VP9 spec 6.2.11. Everything it does NOT
	 * update persists in the instance (see the struct comment); the local
	 * seg_* variables below are only the per-frame gates.
	 */
	seg_en = vp9_f(&b, 1);
	if (seg_en) {
		seg_upd_map = vp9_f(&b, 1);
		if (seg_upd_map) {
			for (i = 0; i < 7; i++)
				inst->seg_tree_probs[i] =
					vp9_f(&b, 1) ? vp9_f(&b, 8) : 255;
			inst->seg_temporal = vp9_f(&b, 1);
			for (i = 0; i < 3; i++)
				inst->seg_pred_probs[i] = inst->seg_temporal &&
					vp9_f(&b, 1) ? vp9_f(&b, 8) : 255;
		}
		if (vp9_f(&b, 1)) {		/* segmentation_update_data */
			static const u8 feat_bits[4] = { 8, 6, 2, 0 };
			static const u8 feat_signed[4] = { 1, 1, 0, 0 };
			int seg, j;

			inst->seg_abs_delta = vp9_f(&b, 1);
			memset(inst->seg_feat_en, 0, sizeof(inst->seg_feat_en));
			memset(inst->seg_feat_data, 0,
			       sizeof(inst->seg_feat_data));
			for (seg = 0; seg < 8; seg++) {
				for (j = 0; j < 4; j++) {
					int val = 0;

					if (!vp9_f(&b, 1))
						continue;
					inst->seg_feat_en[seg] |= BIT(j);
					if (feat_bits[j])
						val = vp9_f(&b, feat_bits[j]);
					if (feat_signed[j] && vp9_f(&b, 1))
						val = -val;
					inst->seg_feat_data[seg][j] = (s16)val;
				}
			}
		}
	}
	/*
	 * MTK_VDEC_MAX_W/H is enforced in try_fmt against the INITIAL negotiated
	 * format only, so a stream that opens small and resizes past the cap is
	 * admitted and then dies in hardware -- a 500 ms decode timeout per
	 * frame followed by larb=3 IOMMU read faults (measured on
	 * vp90-2-14-resize-fp-tiles-1-8, which resizes 432x240 -> 3840x2160).
	 * Refuse it here instead.
	 */
	if (pic_w > vp9_max_w() || pic_h > vp9_max_h()) {
		mtk_v4l2_vdec_err(ctx,
			"mt8365 vp9: mid-stream resize to %ux%u exceeds %ux%u",
			pic_w, pic_h, vp9_max_w(), vp9_max_h());
		return -EINVAL;
	}
	/*
	 * tile_info() -- VP9 spec 6.2.14. The increment_tile_cols_log2 loop
	 * codes ZERO bits only while minLog2TileCols == maxLog2TileCols, which
	 * is true for Sb64Cols < 8 (<= 448 px wide) and NOT above it. The
	 * earlier "reject anything wider" gate therefore refused every clip
	 * over 448 px REGARDLESS of how many tile columns it actually uses --
	 * a parse gap, not a hardware limit. Parse the field properly; the
	 * multi-tile-column DECODE is still unimplemented, so reject only when
	 * the stream really asks for more than one column.
	 *
	 *   MAX_TILE_WIDTH_B64 = 64, MIN_TILE_WIDTH_B64 = 4
	 */
	sb_cols = DIV_ROUND_UP(pic_w, 64);
	sb_rows = DIV_ROUND_UP(pic_h, 64);
	for (min_log2_tile_cols = 0;
	     (64 << min_log2_tile_cols) < sb_cols; min_log2_tile_cols++)
		;
	for (max_log2_tile_cols = 1;
	     (sb_cols >> max_log2_tile_cols) >= 4; max_log2_tile_cols++)
		;
	max_log2_tile_cols--;
	tile_cols_log2 = min_log2_tile_cols;
	while (tile_cols_log2 < max_log2_tile_cols) {
		if (!vp9_f(&b, 1))		/* increment_tile_cols_log2 */
			break;
		tile_cols_log2++;
	}
	tile_rows_log2 = vp9_f(&b, 1);
	if (tile_rows_log2)
		tile_rows_log2 += vp9_f(&b, 1);
	hdr_size = vp9_f(&b, 16);		/* header_size_in_bytes */
	/* trailing_bits(): pad the uncompressed header out to a byte boundary */
	vp9_f(&b, (8 - (b.pos & 7)) & 7);
	uh_bytes = b.pos / 8;
	tile_start = uh_bytes + hdr_size;
	if (!hdr_size || tile_start >= bs->size) {
		mtk_v4l2_vdec_err(ctx, "mt8365 vp9: bad header size %u (au %zx)",
				  hdr_size, bs->size);
		return -EINVAL;
	}

	/* The HW's own bit-position counter must agree with the software parse
	 * -- the single cheapest proof that the header was read correctly. */
	if (rd(ctx, VDEC_HWV, V9_BITPOS) != b.pos)
		mtk_v4l2_vdec_err(ctx,
			"mt8365 vp9: header bitpos mismatch hw=%u sw=%u",
			rd(ctx, VDEC_HWV, V9_BITPOS), b.pos);

	inst->pic_w = pic_w;
	inst->pic_h = pic_h;
	aw = ALIGN(pic_w, 64);
	ah = ALIGN(pic_h, 64);
	mi_cols = DIV_ROUND_UP(pic_w, 8);
	mi_rows = DIV_ROUND_UP(pic_h, 8);

	/* ---- seq 123-130: geometry, VP9 core flags, loop-filter params ---- */
	wr(ctx, VDEC_TOP, TOP_PICSIZE_PX, (pic_h << 16) | pic_w);
	wr(ctx, VDEC_TOP, TOP_PICSIZE_SB, ((sb_rows - 1) << 16) | (sb_cols - 1));
	wr(ctx, VDEC_HWV, V9_CTRL, 0x1);
	/*
	 * MEASURED across six shapes -- see the header comment's 0xa8 table.
	 * bit31 is set on every inter trace and is confounded with bit0 there;
	 * both are written together here.
	 */
	flags9 = ((frame_parallel || !adapt_en) ? 0x8 : 0x0) |
		 (lossless ? 0x4 : 0x0) | (err_res ? 0x10 : 0x0);
	/*
	 * bits 10:8 = segmentation_enabled / _update_map / _temporal_update.
	 * Transcribed from VP9_Set_UnCompressed (libvpud_vcodec.so @0x7ad20),
	 * which ORs the three segmentation bytes in at shifts 8, 9 and 10, and
	 * MEASURED on the matched triple: segon-176's keyframe (enabled +
	 * update_map, temporal 0) writes 0x00000300, its update_map = 0 inter
	 * frames 0x84000101, and segcr-176's temporal_update inter frames
	 * 0x84000701.
	 */
	if (seg_en)
		flags9 |= 0x100 | (seg_upd_map ? 0x200 : 0) |
			  (inst->seg_temporal ? 0x400 : 0);
	if (!keyframe && !intra_only) {
		/*
		 * bit31 and bits 26:24 were CONFOUNDED until the segmentation
		 * round: every earlier clip was allow_high_precision_mv = 1 AND
		 * either SWITCHABLE or literal 1. vp90-2-19-skip breaks both --
		 * its frames 1/2/3 are (hp 1, SWITCHABLE), (hp 0, literal 1)
		 * and (hp 1, literal 2), and the vendor writes 0x84000701,
		 * 0x00000501 and 0x82000501. So bit31 is
		 * allow_high_precision_mv, and bits 26:24 are the interpolation
		 * filter in libvpx's TYPE enum (literal_to_type[] =
		 * {SMOOTH, EIGHTTAP, SHARP, BILINEAR}, SWITCHABLE = 4), of
		 * which the long-standing 0x04000000 is just the type-4 case.
		 *
		 * ON by default since it was measured to take
		 * vp90-2-19-skip from a frame-3 divergence to 60/60 bit-exact
		 * (md5-identical to the ffmpeg reference); quirk 0x1000
		 * restores the old encoding for a same-build A/B.
		 */
		if (mt8365_vp9_quirk & 0x1000) {
			flags9 |= 0x80000001;
			if (interp == VP9_SWITCHABLE_FILTER)
				flags9 |= 0x04000000;
		} else {
			flags9 |= 0x1 | (allow_hp ? 0x80000000 : 0) |
				  (interp_type << 24);
		}
	}
	wr(ctx, VDEC_HWV, V9_FLAGS, flags9);
	/*
	 * +0xac carries the two features the hardware needs as a whole-frame
	 * table rather than through a SRAM: SEG_LVL_SKIP's enable mask in bits
	 * 7:0, SEG_LVL_REF_FRAME's 2-bit datum per segment in bits 23:8, and
	 * its enable mask in bits 31:24. Transcribed from VP9_Set_UnCompressed;
	 * DERIVED, NOT MEASURED -- no clip in the matched triple codes either
	 * feature, so every traced frame writes 0 here (which this reproduces).
	 */
	if (seg_en) {
		for (i = 0; i < 8; i++) {
			u32 en = inst->seg_feat_en[i];

			if (en & BIT(VP9_SEG_LVL_SKIP))
				seg_ac |= 1u << i;
			if (en & BIT(VP9_SEG_LVL_REF_FRAME))
				seg_ac |= 1u << (24 + i);
			seg_ac |= ((u32)inst->seg_feat_data[i]
					[VP9_SEG_LVL_REF_FRAME] & 0x3)
				  << (8 + 2 * i);
		}
	}
	wr(ctx, VDEC_HWV, V9_CFGAC, seg_ac);
	/*
	 * PP+0xd8 = (loop_filter_level << 16) | (sharpness_level << 8) |
	 * (mode_ref_delta_enabled & 0xf).
	 *
	 * The level field was MEASURED across lf_level 0/2/3/6/18. The
	 * SHARPNESS field is MEASURED 2026-07-28 on a matched pair of vendor
	 * traces of vp90-2-01-sharpness-1 and -7 (one source, differing only in
	 * sharpness_level): both keyframes write 0x001b0001 (level 27,
	 * sharpness 0), and frame 1 writes 0x001e0101 (level 30, sharpness 1)
	 * vs 0x00120701 (level 18, sharpness 7). VP9_Set_UnCompressed
	 * @0x7ae94 packs it the same way -- `ubfiz w3, w3, #8, #8` of the
	 * struct word at +284 -- with the low nibble coming from the byte at
	 * +292 that also gates the +0xdc / +0xe0 delta writes.
	 *
	 * sharpness_level changes the spec 8.8.1 limit/blimit/thresh
	 * derivation, so leaving it at 0 deblocked every non-zero-sharpness
	 * frame too hard. quirk 0x40000 restores the old always-0 encoding.
	 *
	 * The LOW NIBBLE is loop_filter_delta_enabled (spec 6.2.8
	 * `loop_filter_delta_enabled`, libvpx `lf->mode_ref_delta_enabled`) --
	 * it gates whether the hardware applies the +0xdc / +0xe0 ref/mode
	 * deltas at all. Every vendor trace we hold codes it 1, so it used to
	 * be written as the constant 1; vp90-2-09-lf_deltas is the one
	 * VP9-TEST-VECTORS clip that toggles it frame to frame (its diagnostic
	 * shows lf=<level>/<sharp>/0 on roughly every other inter frame), and
	 * on those frames a constant 1 makes the hardware deblock with deltas
	 * the bitstream disabled. quirk 0x80000 restores the constant 1.
	 */
	wr(ctx, VDEC_PP, PP_LF_CTRL,
	   (lf_level << 16) |
	   (((mt8365_vp9_quirk & 0x40000) ? 0 : (lf_sharp & 0xff)) << 8) |
	   (((mt8365_vp9_quirk & 0x80000) || lf_delta_en) ? 0x1 : 0x0));
	wr(ctx, VDEC_PP, PP_REF_LF_DELTA,
	   (u32)(u8)inst->ref_delta[0] | ((u32)(u8)inst->ref_delta[1] << 8) |
	   ((u32)(u8)inst->ref_delta[2] << 16) |
	   ((u32)(u8)inst->ref_delta[3] << 24));
	wr(ctx, VDEC_PP, PP_MODE_LF_DELTA,
	   (u32)(u8)inst->mode_delta[0] | ((u32)(u8)inst->mode_delta[1] << 8));
	/*
	 * Per-segment ALT_L (loop-filter level) -- PP+0xe4 is the enable mask
	 * with abs_or_delta_update at bit 8, +0xe8 / +0xec are the eight signed
	 * data bytes. Transcribed from VP9_Set_UnCompressed, which reads the
	 * three words the vendor's segmentation parser packs at seg+8/12/16;
	 * DERIVED, NOT MEASURED (no traced clip codes ALT_L, and all three
	 * write 0 -- reproduced).
	 */
	if (seg_en) {
		for (i = 0; i < 8; i++) {
			u32 d = (u32)(u8)(s8)inst->seg_feat_data[i]
					[VP9_SEG_LVL_ALT_L];

			if (inst->seg_feat_en[i] & BIT(VP9_SEG_LVL_ALT_L))
				seg_lf_en |= 1u << i;
			if (i < 4)
				seg_lf03 |= d << (8 * i);
			else
				seg_lf47 |= d << (8 * (i - 4));
		}
		if (inst->seg_abs_delta)
			seg_lf_en |= 0x100;
		wr(ctx, VDEC_PP, PP_SEG_LF_EN, seg_lf_en);
		wr(ctx, VDEC_PP, PP_SEG_LF_D03, seg_lf03);
		wr(ctx, VDEC_PP, PP_SEG_LF_D47, seg_lf47);
		/*
		 * The tree / pred probabilities and the segment-map surface.
		 * MEASURED on the matched triple, value for value (segon-176's
		 * tree probs {54,133,255,217,128,255,128} -> 0xd9ff8536 /
		 * 0x0080ff80, its all-255 pred probs -> 0x00ffffff; segcr-176's
		 * {245,1,1} -> 0x000101f5). The map address is written RAW
		 * (not shifted) to BOTH ports -- the vendor allocates ONE
		 * surface and points the previous-map read and the current-map
		 * write at it, which is what makes temporal prediction an
		 * in-place read-then-write and what makes the buffer's
		 * frame-to-frame survival load-bearing.
		 */
		wr(ctx, VDEC_HWV, V9_SEG_TREE0,
		   (u32)inst->seg_tree_probs[0] |
		   ((u32)inst->seg_tree_probs[1] << 8) |
		   ((u32)inst->seg_tree_probs[2] << 16) |
		   ((u32)inst->seg_tree_probs[3] << 24));
		wr(ctx, VDEC_HWV, V9_SEG_TREE1,
		   (u32)inst->seg_tree_probs[4] |
		   ((u32)inst->seg_tree_probs[5] << 8) |
		   ((u32)inst->seg_tree_probs[6] << 16));
		wr(ctx, VDEC_HWV, V9_SEG_PRED,
		   (u32)inst->seg_pred_probs[0] |
		   ((u32)inst->seg_pred_probs[1] << 8) |
		   ((u32)inst->seg_pred_probs[2] << 16));
		wr(ctx, VDEC_TOP, TOP_SEGMAP_A, (u32)inst->seg_buf.dma_addr);
		wr(ctx, VDEC_TOP, TOP_SEGMAP_B, (u32)inst->seg_buf.dma_addr);
	}
	if (lossless)
		wr(ctx, VDEC_PP, PP_LOSSLESS, 0xe00);

	/* ---- seq 131-760: upload frame_contexts[frame_context_idx] ---- */
	vp9_upload_probs(ctx, inst->fc[fc_idx]);

	/* ---- seq 761-776: re-init the barrel shifter onto the COMPRESSED
	 * header, read tx_mode, then let the HW bool-decode the rest of it ---- */
	wr(ctx, VDEC_HWV, V9_BSHIFT_INIT, 0x1);
	wr(ctx, VDEC_HWV, V9_BSHIFT_INIT, 0x0);
	(void)rd(ctx, VDEC_HWV, V9_BITPOS);
	wr(ctx, VDEC_HWV, V9_STATE110, 0x1);
	(void)rd(ctx, VDEC_HWV, V9_STATE110);
	if (!lossless) {
		tx_mode = vp9_lit(ctx, 2);
		if (tx_mode == 3)		/* ALLOW_32X32 */
			tx_mode += vp9_lit(ctx, 1);
	}
	wr(ctx, VDEC_HWV, V9_TX_MODE, tx_mode);
	/*
	 * One +0xc8 pulse per compressed_header() sub-step, in bitstream order.
	 * All MEASURED -- see the header comment.
	 */
	if (tx_mode == 4)
		masks[nmask++] = 0x1;		/* TX_MODE_SELECT: tx probs */
	masks[nmask++] = 0x2;			/* coefficient probs */
	masks[nmask++] = 0x4;			/* skip probs */
	if (!keyframe && !intra_only) {
		compound_allowed = (sign_bias[0] != sign_bias[1]) ||
				   (sign_bias[0] != sign_bias[2]);
		masks[nmask++] = 0x8;		/* inter_mode probs */
		if (interp == VP9_SWITCHABLE_FILTER)
			masks[nmask++] = 0x10;	/* interp-filter probs */
		masks[nmask++] = 0x20;		/* is_inter probs */
	}
	/*
	 * frame_reference_mode() -- VP9 spec 6.3.12.
	 *
	 * It sits in the compressed header BETWEEN the intra_inter (+0xc8 =
	 * 0x20) sub-step and the reference-mode-prob sub-steps, and the vendor
	 * host reads it THROUGH THE HARDWARE BOOL BANK, one literal at a time:
	 *
	 *   tr-n22f5.log seq 2537..2543, vp90-2-14-resize-10frames-fp-tiles-1-2
	 *   frame 2 (compoundReferenceAllowed == 1):
	 *      2537  W V9:VLD9 +0xc8 = 0x20      <- intra_inter probs
	 *      2538  W V9:VLD9 +0xc8 = 0x0
	 *      2539  R V9:VLD9 +0xcc = 0x3fff
	 *      2540  R V9:VLD9 +0x84 = 0x0       <- non_single_reference, L(1)
	 *      2541  W V9:VLD9 +0xc8 = 0x80      <- single_ref probs
	 *   the same window on frame 1 (compoundReferenceAllowed == 0), seq
	 *   1635..1638, has NO +0x84 read: 0x20 pulse then straight to 0x80.
	 *
	 * The bank CONSUMES on read, so decoding reference_mode in software and
	 * not reading it here leaves the hardware's bool decoder 1-2 bits
	 * BEHIND for the whole rest of the compressed header. Every later
	 * probability update then lands on the wrong bits, the forward-updated
	 * context is garbage, and the tile decode runs off the end of the frame
	 * (V9_STATUS15C = 1, bitpos > the tile's bit budget) and reconstructs
	 * the picture as a verbatim copy of the LAST reference.
	 *
	 * That is why EVERY compoundReferenceAllowed frame in the corpus failed
	 * and no passing vector contains one -- and why it was invisible to the
	 * register cross-diff: mt8365-vp9-tracediff.py compares WRITES only.
	 *
	 * The 2026-07-28 "a host vp9_lit() DOUBLE-CONSUMES" reading was wrong:
	 * that experiment read the bits at the wrong point in the sub-step
	 * chain. Fire the sub-steps up to 0x20 FIRST, then read.
	 *
	 * quirk 0x10000 restores the software-only decode (no hardware read),
	 * i.e. the pre-fix behaviour, for A/B.
	 */
	npre = nmask;
	for (i = 0; i < npre; i++)
		vp9_hdr_parse(ctx, masks[i]);
	if (compound_allowed) {
		u32 sw_tx = tx_mode;
		int m = -EINVAL;

		if (bs->va && uh_bytes + hdr_size <= bs->size)
			m = vp9_sw_reference_mode((const u8 *)bs->va + uh_bytes,
						  hdr_size, lossless,
						  interp == VP9_SWITCHABLE_FILTER,
						  &sw_tx);
		if (!(mt8365_vp9_quirk & 0x10000)) {
			/* read_frame_reference_mode(), through the hardware's
			 * bool bank -- exactly the vendor's +0x84 reads.
			 */
			if (vp9_lit(ctx, 1))
				ref_mode = vp9_lit(ctx, 1) ?
					   VP9_REFERENCE_MODE_SELECT :
					   VP9_COMPOUND_REFERENCE;
			else
				ref_mode = VP9_SINGLE_REFERENCE;
			if (m >= 0 && sw_tx == tx_mode && m != ref_mode)
				mtk_v4l2_vdec_err(ctx,
					"mt8365 vp9: sw ref_mode %d != hw %d",
					m, ref_mode);
		} else if (m >= 0 && sw_tx == tx_mode) {
			ref_mode = m;
		} else {
			ref_mode = (mt8365_vp9_quirk & 0x8000)
				 ? VP9_SINGLE_REFERENCE
				 : VP9_REFERENCE_MODE_SELECT;
		}
		/* setup_compound_reference_mode(), spec 6.3.12 -- libvpx runs it
		 * only when reference_mode != SINGLE_REFERENCE, and the vendor
		 * trace confirms the register's compound fields stay 0 otherwise.
		 */
		if (ref_mode != VP9_SINGLE_REFERENCE) {
			if (sign_bias[0] == sign_bias[1]) {
				comp_fixed_ref = 3;	/* ALTREF */
				comp_var_ref[0] = 1;	/* LAST   */
				comp_var_ref[1] = 2;	/* GOLDEN */
			} else if (sign_bias[0] == sign_bias[2]) {
				comp_fixed_ref = 2;	/* GOLDEN */
				comp_var_ref[0] = 1;	/* LAST   */
				comp_var_ref[1] = 3;	/* ALTREF */
			} else {
				comp_fixed_ref = 1;	/* LAST   */
				comp_var_ref[0] = 2;	/* GOLDEN */
				comp_var_ref[1] = 3;	/* ALTREF */
			}
		}
	}
	if (!keyframe && !intra_only) {
		/*
		 * read_frame_reference_mode_probs(), transcribed verbatim from
		 * VP9_Read_Frame_Ref_Mode_Probs @0x76d50. The three masks are
		 * MEASURED: VP9_UPDATE_COMP_INTER_PROBS @0x7bc90 writes
		 * +0xc8 = 0x40, VP9_UPDATE_SINGLE_REF_PROBS @0x7bd00 writes 0x80,
		 * VP9_UPDATE_COMP_REF_PROBS @0x7bd70 writes 0x100 -- and libvpx's
		 * gating on reference_mode, which we now know, is applied.
		 */
		if (ref_mode == VP9_REFERENCE_MODE_SELECT)
			masks[nmask++] = 0x40;	/* comp_inter probs */
		if (ref_mode != VP9_COMPOUND_REFERENCE)
			masks[nmask++] = 0x80;	/* single_ref probs */
		if (ref_mode != VP9_SINGLE_REFERENCE)
			masks[nmask++] = 0x100;	/* comp_ref probs */
		masks[nmask++] = 0x200;		/* y_mode probs */
		masks[nmask++] = 0x400;		/* partition probs */
		masks[nmask++] = 0x800;		/* motion-vector probs */
		masks[nmask++] = 0x1000;
		masks[nmask++] = 0x2000;
	}
	for (i = npre; i < nmask; i++)
		vp9_hdr_parse(ctx, masks[i]);
	hdrpos = rd(ctx, VDEC_HWV, V9_BITPOS);
	/*
	 * The frame_reference_mode word (see V9_REF_MODE above). Bit 16 is
	 * ref_frame_sign_bias[INTRA_FRAME], which the spec pins at 0; our
	 * sign_bias[0..2] are LAST/GOLDEN/ALTREF = the vendor's indices 1..3.
	 * quirk 0x4000 restores the pre-2026-07-27 behaviour (always 0, single
	 * reference), for A/B.
	 */
	ref_mode_word = ref_mode |
			(comp_fixed_ref << 8) |
			((u32)sign_bias[0] << 17) |
			((u32)sign_bias[1] << 18) |
			((u32)sign_bias[2] << 19) |
			(comp_var_ref[0] << 24) |
			(comp_var_ref[1] << 26);
	if (mt8365_vp9_quirk & 0x4000)
		ref_mode_word = 0;	/* pre-compound behaviour, for A/B */
	wr(ctx, VDEC_HWV, V9_REF_MODE, ref_mode_word);

	/* ---- seq 777-795: colocated MV, references, recon, geometry ---- */
	/*
	 * MV+0x3c4 is use_prev_frame_mvs: 1 on a plain inter frame (including
	 * the one right after a keyframe -- MEASURED), 0 on an error-resilient
	 * inter frame and 0 on an intra frame. The size / show-frame clauses
	 * follow VP9 spec 6.2 and are INFERRED (no trace varies them).
	 */
	use_prev_mvs = !keyframe && !intra_only && !err_res &&
		       inst->prev_valid && inst->prev_show &&
		       inst->prev_w == pic_w && inst->prev_h == pic_h;
	wr(ctx, VDEC_AV, MV_CFG3C4, use_prev_mvs ? 0x1 : 0x0);
	wr(ctx, VDEC_AV, MV_COLMV_A, colmv >> 4);
	wr(ctx, VDEC_AV, MV_COLMV_B, colmv >> 4);
	/*
	 * An intra frame references only itself: all three slots point at the
	 * current recon (trace, 7/7). An inter frame's three slots are
	 * ref_frame_idx[0..2], resolved through the stateless control.
	 */
	if (keyframe || intra_only) {
		for (i = 0; i < 3; i++) {
			ref_y[i] = rec_y;
			ref_c[i] = rec_c;
			ref_w[i] = pic_w;
			ref_h[i] = pic_h;
		}
	}
	for (i = 0; i < 3; i++) {
		wr(ctx, VDEC_CM, MC_REF_Y + 4 * i, ref_y[i]);
		wr(ctx, VDEC_CM, MC_REF_C + 4 * i, ref_c[i]);
	}
	if (!keyframe && !intra_only) {
		u32 xfp[3], yfp[3], rrf_x[3], rrf_y[3];
		bool scaled[3];

		for (i = 0; i < 3; i++) {
			xfp[i] = vp9_scale_fp(ref_w[i], pic_w);
			yfp[i] = vp9_scale_fp(ref_h[i], pic_h);
			scaled[i] = !(mt8365_vp9_quirk & 0x2000) &&
				    vp9_valid_ref_size(ref_w[i], ref_h[i],
						       pic_w, pic_h) &&
				    (xfp[i] != VP9_REF_NO_SCALE ||
				     yfp[i] != VP9_REF_NO_SCALE);
			rrf_x[i] = scaled[i] ? vp9_max_rrf_blk(xfp[i]) : 7;
			rrf_y[i] = scaled[i] ? vp9_max_rrf_blk(yfp[i]) : 7;
			scale_en |= (u32)scaled[i] << (8 - 4 * i);
		}
		wr(ctx, VDEC_CM, MC_REF_SCALE_EN, scale_en);
		for (i = 0; i < 3; i++) {
			wr(ctx, VDEC_CM, MC_REF_DIM + 4 * i,
			   (ref_w[i] << 16) | ref_h[i]);
			if (scaled[i]) {
				u32 xs = (16 * xfp[i]) >> VP9_REF_SCALE_SHIFT;
				u32 ys = (16 * yfp[i]) >> VP9_REF_SCALE_SHIFT;

				wr(ctx, VDEC_CM, MC_REF_SCALE_STEP + 4 * i,
				   (xs << 16) | (ys & 0xffff));
				wr(ctx, VDEC_CM, MC_REF_SCALE_FP + 8 * i,
				   xfp[i]);
				wr(ctx, VDEC_CM, MC_REF_SCALE_FP + 8 * i + 4,
				   yfp[i]);
			}
			wr(ctx, VDEC_CM, MC_REF_STRIDE_MB + 4 * i,
			   ALIGN(ref_w[i], 64) / 16);
		}
		wr(ctx, VDEC_CM, MC_REF_RRF_BLK,
		   (rrf_x[0] << 24) | (rrf_x[1] << 20) | (rrf_x[2] << 16) |
		   (rrf_y[0] << 8) | (rrf_y[1] << 4) | rrf_y[2]);
	}
	wr(ctx, VDEC_CM, MC_MI_DIM, (mi_cols << 16) | mi_rows);
	wr(ctx, VDEC_CM, MC_ENABLE220, 0x1);
	wr(ctx, VDEC_CM, MC_RECON_Y, rec_y >> 9);
	wr(ctx, VDEC_CM, MC_RECON_C, rec_c >> 8);
	wr(ctx, VDEC_CM, MC_RECON_STRIDE_MB, aw / 16);
	wr(ctx, VDEC_CM, MC_INTER_EN, 0x3);
	wr(ctx, VDEC_CM, MC_ENABLE250, 0x1);
	wr(ctx, VDEC_CM, MC_MB_W_M1, aw / 16 - 1);
	wr(ctx, VDEC_CM, MC_MB_H_M1, ah / 16 - 1);
	if (mt8365_vp9_quirk & 0x1) {
		u32 wlo = min(rec_y, rec_c);
		u32 whi = max(rec_y + aw * ah, rec_c + aw * ah / 2);

		wr(ctx, VDEC_CM, MC_RECON_LIMIT, whi >> 4);
		wr(ctx, VDEC_CM, MC_RECON_BASE, wlo >> 4);
	}

	/* ---- seq 796-830: dequantisers. Sixteen SRAM entries = eight segments
	 * x {luma, chroma}; with segmentation off only segment 0 is non-zero,
	 * and the word is (ac << 16) | dc. ---- */
	wr(ctx, VDEC_PP, PP_PICSIZE, (pic_h << 16) | pic_w);
	wr(ctx, VDEC_PP, PP_DEQ_GATE, 0x1);
	/*
	 * With segmentation OFF the vendor fills entry 0 / 1 and leaves 2..15
	 * zero (MEASURED, segoff-176). With it ON every segment's qindex comes
	 * from get_qindex(), and entry 2s / 2s+1 are that segment's luma /
	 * chroma pair (MEASURED, segon-176: base_q 33 with ALT_Q deltas
	 * -24/-20/-13/+14 on segments 0/1/2/4 gives 0x0010000e / 0x00140012 /
	 * 0x001b0018 / 0x0036002f, the unfeatured segments keeping the base
	 * 0x00280023).
	 */
	for (i = 0; i < 16; i++) {
		u32 v = 0;

		if (seg_en || i < 2) {
			u32 q = vp9_seg_qindex(inst, seg_en, i / 2, base_q);

			if (i & 1)
				v = ((u32)vp9_ac_qlookup[clamp_t(int, (int)q +
						dq_uvac, 0, 255)] << 16) |
				    vp9_dc_qlookup[clamp_t(int, (int)q +
						dq_uvdc, 0, 255)];
			else
				v = ((u32)vp9_ac_qlookup[q] << 16) |
				    vp9_dc_qlookup[clamp_t(int, (int)q +
						dq_ydc, 0, 255)];
		}
		wr(ctx, VDEC_LD, VLD_DEQ_ADDR, i);
		wr(ctx, VDEC_LD, VLD_DEQ_DATA, v);
	}
	wr(ctx, VDEC_PP, PP_DEQ_GATE, 0x0);

	/* ---- seq 831-834 ---- */
	wr(ctx, VDEC_CM, MC_CFGA60, 0x0);
	wr(ctx, VDEC_CM, MC_CFGB5C, 0x1);
	wr(ctx, VDEC_CM, MC_PIC_W_PX, pic_w);
	wr(ctx, VDEC_CM, MC_PIC_H_PX, pic_h);

	/* ---- seq 835-843: the per-tile descriptors (see header note 3) ---- */
	cfgbc = 0x00010111;
	{
		u32 n_cols = 1u << tile_cols_log2;
		u32 n_rows = 1u << tile_rows_log2;
		u32 pos = tile_start, r, c;
		/*
		 * > 8 tiles go through the DMA descriptor buffer, not the +0x104
		 * MMIO port (header note 3, VP9_Set_Tile_Info @0x7ca20). quirk
		 * 0x20000 forces the old MMIO-only behaviour back for an A/B.
		 */
		bool desc_dma = (n_rows * n_cols) > VP9_TILEDESC_MMIO_MAX &&
				!(mt8365_vp9_quirk & 0x20000);
		u32 *desc = inst->tile_buf.va;

		if (desc_dma &&
		    (size_t)n_rows * n_cols * 16 > inst->tile_buf.size) {
			mtk_v4l2_vdec_err(ctx,
				"mt8365 vp9: %u tiles exceed the descriptor buffer",
				n_rows * n_cols);
			return -EINVAL;
		}

		wr(ctx, VDEC_HWV, V9_CFGC4,
		   ((n_rows - 1) << 16) | (n_cols - 1));
		wr(ctx, VDEC_HWV, V9_TILE_ADDR, 0x0);

		for (r = 0; r < n_rows; r++) {
			u32 mr0 = vp9_tile_offset(r, mi_rows, tile_rows_log2);
			u32 mr1 = vp9_tile_offset(r + 1, mi_rows,
						  tile_rows_log2);

			for (c = 0; c < n_cols; c++) {
				u32 mc0 = vp9_tile_offset(c, mi_cols,
							  tile_cols_log2);
				u32 mc1 = vp9_tile_offset(c + 1, mi_cols,
							  tile_cols_log2);
				bool last = (r == n_rows - 1) &&
					    (c == n_cols - 1);
				u32 tsz, tend, w[4];

				if (last) {
					tsz = (u32)bs->size - pos;
				} else {
					const u8 *p = (const u8 *)bs->va + pos;

					if (pos + 4 > bs->size) {
						mtk_v4l2_vdec_err(ctx,
							"mt8365 vp9: truncated tile size at %u/%zu",
							pos, bs->size);
						return -EINVAL;
					}
					tsz = ((u32)p[0] << 24) |
					      ((u32)p[1] << 16) |
					      ((u32)p[2] << 8) | p[3];
					pos += 4;
				}
				tend = pos + tsz;
				if (tend > bs->size || tend < pos) {
					mtk_v4l2_vdec_err(ctx,
						"mt8365 vp9: tile %u,%u overruns AU (%u+%u > %zu)",
						r, c, pos, tsz, bs->size);
					return -EINVAL;
				}
				base_off = pos & ~15u;
				w[0] = (tend - base_off) * 8;
				w[1] = (u32)bs->dma_addr + base_off;
				w[2] = (pos & 15u) * 8;
				w[3] = ((DIV_ROUND_UP(mr1 - mr0, 8) - 1) << 6) |
				       (DIV_ROUND_UP(mc1 - mc0, 8) - 1);
				if (desc_dma) {
					*desc++ = w[0];
					*desc++ = w[1];
					*desc++ = w[2];
					*desc++ = w[3];
				} else {
					wr(ctx, VDEC_HWV, V9_TILE_DATA, w[0]);
					wr(ctx, VDEC_HWV, V9_TILE_DATA, w[1]);
					wr(ctx, VDEC_HWV, V9_TILE_DATA, w[2]);
					wr(ctx, VDEC_HWV, V9_TILE_DATA, w[3]);
				}
				pos = tend;
			}
		}
		if (desc_dma) {
			/* The buffer is coherent; the barrier orders it before
			 * the address write the hardware then follows.
			 */
			wmb();
			wr(ctx, VDEC_TOP, TOP_TILE_DESC,
			   (u32)inst->tile_buf.dma_addr);
		}
		cfgbc = desc_dma ? 0x00000111 : 0x00010111;
	}
	wr(ctx, VDEC_LD, VLD_CFG140, 0x00100014);
	wr(ctx, VDEC_HWB, VLD_CFG140, 0x00100014);
	wr(ctx, VDEC_HWV, V9_CFGBC, cfgbc);

	/* ---- seq 844-849: final sync, TOP gate, DECODE KICK ---- */
	vp9_sync(ctx);
	wr(ctx, VDEC_TOP, TOP_CFG124,
	   (rd(ctx, VDEC_TOP, TOP_CFG124) & ~0xfffu) | 0x10);
	/*
	 * The vendor's decode-completion signal is NOT in the trace (it waits on
	 * the VDEC IRQ inside vpud, which the MMIO interposer cannot see), so the
	 * done source is a bring-up question. Sample the candidates BEFORE the
	 * kick so a "done bit" that is already set is visible rather than
	 * silently turning the poll into a no-op.
	 */
	pre174 = rd(ctx, VDEC_HWV, V9_DONE);
	pre15c = rd(ctx, VDEC_HWV, V9_STATUS15C);
	prea4 = rd(ctx, VDEC_MISC, 0xa4);
	wr(ctx, VDEC_HWV, V9_KICK, 0x1);

	if (mt8365_vp9_quirk & 0x4) {
		/* Use the core dec-done IRQ, exactly as the H.264 backend does. */
		ret = mtk_vcodec_wait_for_done_ctx(ctx, MTK_INST_IRQ_RECEIVED,
						   1000, MTK_VDEC_CORE);
		i = ret ? MT8365_VP9_TIMEOUT : 0;
	} else {
		for (i = 0; i < MT8365_VP9_TIMEOUT; i++) {
			if (rd(ctx, VDEC_HWV, V9_DONE) & 0x1)
				break;
			cpu_relax();
		}
	}
	spins = i;
	/* quirk 0x2: crude settle delay before the post-decode tail, to separate
	 * "the done source is wrong" from "the tail races the recon DMA". */
	if (mt8365_vp9_quirk & 0x2)
		mdelay(50);
	if (i == MT8365_VP9_TIMEOUT) {
		mtk_v4l2_vdec_err(ctx,
			"mt8365 vp9 TIMEOUT f%u %ux%u q=%u lf=%u tx=%u p2=%u bs=%pad sz=%zx | V9[0x174]=%08x [0x15c]=%08x [0x138]=%08x [0x1a8]=%08x [0xcc]=%08x | MISC[0xa4]=%08x [0x04]=%08x | LD[0xe8]=%08x [0xf4]=%08x HWB[0xe8]=%08x",
			inst->frm_cnt, pic_w, pic_h, base_q, lf_level, tx_mode,
			tile_start, &bs->dma_addr, bs->size,
			rd(ctx, VDEC_HWV, V9_DONE),
			rd(ctx, VDEC_HWV, V9_STATUS15C),
			rd(ctx, VDEC_HWV, V9_BITPOS),
			rd(ctx, VDEC_HWV, V9_SYNC),
			rd(ctx, VDEC_HWV, V9_HDR_STATUS),
			rd(ctx, VDEC_MISC, 0xa4), rd(ctx, VDEC_MISC, MISC_STATE),
			rd(ctx, VDEC_LD, VLD_FETCH_DONE),
			rd(ctx, VDEC_LD, VLD_IDLE),
			rd(ctx, VDEC_HWB, VLD_FETCH_DONE));
		return -ETIMEDOUT;
	}

	(void)rd(ctx, VDEC_HWV, V9_STATUS15C);
	crc0 = rd(ctx, VDEC_MISC, MISC_CRC_BASE);

	/*
	 * ---- seq 851-861: the vendor's post-decode tail -- two backward-
	 * adaptation WDMA runs (mode 2 = symbol counts, mode 1 = probabilities)
	 * writing the HW's updated context out to DRAM. A keyframe-only backend
	 * has no use for the result, but the tail is replayed anyway because on
	 * VP8 omitting the equivalent tail let the core cut the VDEC clocks with
	 * the recon write DMA still in flight and HARD-RESET the board.
	 */
	if (mt8365_vp9_quirk & 0x200) {
		memset(inst->cnt_buf.va, 0xee, inst->cnt_buf.size);
		memset(inst->prob_buf.va, 0xee, inst->prob_buf.size);
	}
	wdma_pre[0] = rd(ctx, VDEC_HWV, V9_WDMA_DONE);
	wdma_spin[0] = vp9_wdma(ctx, 0x2, (u32)inst->prob_buf.dma_addr);
	wdma_post[0] = rd(ctx, VDEC_HWV, V9_WDMA_DONE);
	wdma_pre[1] = wdma_post[0];
	wdma_spin[1] = vp9_wdma(ctx, 0x1, (u32)inst->cnt_buf.dma_addr);
	wdma_post[1] = rd(ctx, VDEC_HWV, V9_WDMA_DONE);
	if (mt8365_vp9_quirk & 0x200)
		pr_info("vp9 wdma pre=%08x/%08x spins=%u/%u post=%08x/%08x cnt=%pad prob=%pad\n",
			wdma_pre[0], wdma_pre[1], wdma_spin[0], wdma_spin[1],
			wdma_post[0], wdma_post[1], &inst->cnt_buf.dma_addr,
			&inst->prob_buf.dma_addr);

	/*
	 * refresh_probs(): save the hardware's post-decode context. The WDMA
	 * surface is bank A then bank B, so the upload template's own address
	 * map indexes it directly -- no software probability model.
	 */
	/*
	 * quirk 0x200: dump the raw prob WDMA surface (640 words) and the 615
	 * words this frame UPLOADED. Comparing the upload against a vendor
	 * trace of the same clip, and solving the upload<->WDMA permutation
	 * against the previous frame's dump, is how the bank layout is pinned.
	 */
	if (mt8365_vp9_quirk & 0x200) {
		const u32 *w = inst->prob_buf.va;
		const u32 *c = inst->cnt_buf.va;

		for (i = 0; i < min(mt8365_vp9_dumpn, 640); i += 8)
			pr_info("vp9wd %03x: %08x %08x %08x %08x %08x %08x %08x %08x\n",
				i, w[i], w[i + 1], w[i + 2], w[i + 3],
				w[i + 4], w[i + 5], w[i + 6], w[i + 7]);
		{
			int nz = 0, nt = 0, last = -1;
			int nw = inst->cnt_buf.size / 4;

			for (i = 0; i < nw; i++) {
				if (c[i])
					nz++;
				if (c[i] != 0xeeeeeeeeu) {	/* DMA touched it */
					nt++;
					last = i;
				}
			}
			pr_info("vp9cs f%u nonzero=%d touched=%d/%d last=%d\n",
				inst->frm_cnt, nz, nt, nw, last);
		}
		for (i = 0; i < min(mt8365_vp9_cntn,
				    (int)(inst->cnt_buf.size / 4)); i += 8)
			pr_info("vp9cd %03x: %08x %08x %08x %08x %08x %08x %08x %08x\n",
				i, c[i], c[i + 1], c[i + 2], c[i + 3],
				c[i + 4], c[i + 5], c[i + 6], c[i + 7]);
		for (i = 0; i < min(mt8365_vp9_dumpn, VP9_PROB_WORDS); i += 8)
			pr_info("vp9up %03x: %08x %08x %08x %08x %08x %08x %08x %08x\n",
				i, inst->fc[fc_idx][i],
				inst->fc[fc_idx][min(i + 1, VP9_PROB_WORDS - 1)],
				inst->fc[fc_idx][min(i + 2, VP9_PROB_WORDS - 1)],
				inst->fc[fc_idx][min(i + 3, VP9_PROB_WORDS - 1)],
				inst->fc[fc_idx][min(i + 4, VP9_PROB_WORDS - 1)],
				inst->fc[fc_idx][min(i + 5, VP9_PROB_WORDS - 1)],
				inst->fc[fc_idx][min(i + 6, VP9_PROB_WORDS - 1)],
				inst->fc[fc_idx][min(i + 7, VP9_PROB_WORDS - 1)]);
	}

	/*
	 * quirk 0x800: the OFF-BOARD VALIDATION dump for 8.4 -- the PRE context
	 * (615 words), the whole live part of the counts surface (3592 words)
	 * and the POST context. libvpx computes the identical adaptation, so a
	 * host-side replay of merge(pre, counts) must reproduce post exactly.
	 */
	if (mt8365_vp9_quirk & 0x800) {
		const u32 *c = inst->cnt_buf.va;

		pr_info("vp9v f%u %s fp=%d er=%d rfc=%d fc=%u tx=%u interp=%u hp=%d prevkey=%d\n",
			inst->frm_cnt,
			keyframe ? "KEY" : (intra_only ? "INTRA" : "INTER"),
			frame_parallel, err_res, refresh_fc, fc_idx, tx_mode,
			interp, allow_hp, inst->prev_key);
		for (i = 0; i < VP9_PROB_WORDS; i += 8)
			pr_info("vp9pre %03x: %08x %08x %08x %08x %08x %08x %08x %08x\n",
				i, inst->fc[fc_idx][i],
				inst->fc[fc_idx][min(i + 1, VP9_PROB_WORDS - 1)],
				inst->fc[fc_idx][min(i + 2, VP9_PROB_WORDS - 1)],
				inst->fc[fc_idx][min(i + 3, VP9_PROB_WORDS - 1)],
				inst->fc[fc_idx][min(i + 4, VP9_PROB_WORDS - 1)],
				inst->fc[fc_idx][min(i + 5, VP9_PROB_WORDS - 1)],
				inst->fc[fc_idx][min(i + 6, VP9_PROB_WORDS - 1)],
				inst->fc[fc_idx][min(i + 7, VP9_PROB_WORDS - 1)]);
		if (c)
			for (i = 0; i < 3592; i += 8)
				pr_info("vp9cn %04x: %u %u %u %u %u %u %u %u\n",
					i, c[i], c[i + 1], c[i + 2], c[i + 3],
					c[i + 4], c[i + 5], c[i + 6], c[i + 7]);
	}

	/*
	 * refresh_probs() -- VP9 spec 8.4.
	 *
	 * With frame_parallel_decoding_mode == 1 (or error_resilient_mode) the
	 * spec stores the FORWARD-updated probabilities, which is exactly the
	 * context the hardware hands back on the mode-2 WDMA run.
	 *
	 * Otherwise the stored context is the BACKWARD-ADAPTED one: merge the
	 * frame's symbol counts into the context this frame STARTED from
	 * (frame_contexts[frame_context_idx], still intact in inst->fc[]) --
	 * NOT into the forward-updated one. The forward updates are discarded
	 * for every adapted field, which is what libvpx does and is easy to get
	 * wrong.
	 */
	if (refresh_fc) {
		vp9_read_probs(inst, inst->hwfc);
		if (adapt_en && !err_res && !frame_parallel)
			vp9_adapt_probs(inst, fc_idx, inst->hwfc,
					keyframe || intra_only,
					!keyframe && !intra_only && inst->prev_key,
					tx_mode, interp, allow_hp);
		else
			memcpy(inst->fc[fc_idx], inst->hwfc,
			       sizeof(inst->fc[fc_idx]));
	}

	if (mt8365_vp9_quirk & 0x800) {
		pr_info("vp9v f%u post rfc=%d cnt_bad=%u\n", inst->frm_cnt,
			refresh_fc, inst->cnt_bad);
		for (i = 0; i < VP9_PROB_WORDS; i += 8)
			pr_info("vp9post %03x: %08x %08x %08x %08x %08x %08x %08x %08x\n",
				i, inst->fc[fc_idx][i],
				inst->fc[fc_idx][min(i + 1, VP9_PROB_WORDS - 1)],
				inst->fc[fc_idx][min(i + 2, VP9_PROB_WORDS - 1)],
				inst->fc[fc_idx][min(i + 3, VP9_PROB_WORDS - 1)],
				inst->fc[fc_idx][min(i + 4, VP9_PROB_WORDS - 1)],
				inst->fc[fc_idx][min(i + 5, VP9_PROB_WORDS - 1)],
				inst->fc[fc_idx][min(i + 6, VP9_PROB_WORDS - 1)],
				inst->fc[fc_idx][min(i + 7, VP9_PROB_WORDS - 1)]);
		for (i = 0; i < VP9_PROB_WORDS; i += 8)
			pr_info("vp9hw %03x: %08x %08x %08x %08x %08x %08x %08x %08x\n",
				i, inst->hwfc[i],
				inst->hwfc[min(i + 1, VP9_PROB_WORDS - 1)],
				inst->hwfc[min(i + 2, VP9_PROB_WORDS - 1)],
				inst->hwfc[min(i + 3, VP9_PROB_WORDS - 1)],
				inst->hwfc[min(i + 4, VP9_PROB_WORDS - 1)],
				inst->hwfc[min(i + 5, VP9_PROB_WORDS - 1)],
				inst->hwfc[min(i + 6, VP9_PROB_WORDS - 1)],
				inst->hwfc[min(i + 7, VP9_PROB_WORDS - 1)]);
	}

	/* Record this picture's size against its CAPTURE buffer, so a later
	 * frame that references it can program MC+0xd4c / +0xd70. */
	{
		struct mtk_video_dec_buf *dst_buf_info =
			container_of(fb, struct mtk_video_dec_buf, frame_buffer);
		unsigned int idx = dst_buf_info->m2m_buf.vb.vb2_buf.index;

		if (idx < VB2_MAX_FRAME) {
			inst->fb_w[idx] = pic_w;
			inst->fb_h[idx] = pic_h;
		}
	}
	inst->prev_w = pic_w;
	inst->prev_h = pic_h;
	inst->prev_show = show_frame;
	inst->prev_key = keyframe;
	inst->prev_valid = true;

	if (mt8365_vp9_diag)
		mtk_v4l2_vdec_err(ctx,
		"mt8365 vp9 f%u %s %ux%u (buf %ux%u) q=%u dq=%d/%d/%d lf=%u/%u/%d ll=%d tx=%u sb=%ux%u uh=%u hdr=%u tile=%u tcr=%ux%u bssz=%zu rec=%08x/%08x colmv=%08x hdrpos=%u/%u | fl=%08x fc=%u rfc=%d par=%d er=%d prev=%d interp=%u refresh=%02x ridx=%u/%u/%u ref=%08x/%08x/%08x refdim=%ux%u/%ux%u/%ux%u scen=%03x show=%d rm=%u rmw=%08x | pre[0x174]=%08x [0x15c]=%08x MISC[0xa4]=%08x spins=%u post[0x174]=%08x [0x15c]=%08x MISC[0xa4]=%08x bitpos=%u | CRC=%08x %08x %08x %08x",
		inst->frm_cnt,
		keyframe ? "KEY" : (intra_only ? "INTRA" : "INTER"),
		pic_w, pic_h, aw, ah, base_q,
		dq_ydc, dq_uvdc, dq_uvac, lf_level, lf_sharp, lf_delta_en,
		lossless, tx_mode, sb_cols, sb_rows, uh_bytes, hdr_size,
		tile_start, 1u << tile_cols_log2, 1u << tile_rows_log2,
		bs->size, rec_y, rec_c, colmv, hdrpos, hdr_size * 8,
		flags9, fc_idx, refresh_fc, frame_parallel, err_res,
		use_prev_mvs, interp, refresh_flags,
		ref_idx[0], ref_idx[1], ref_idx[2],
		ref_y[0], ref_y[1], ref_y[2],
		ref_w[0], ref_h[0], ref_w[1], ref_h[1], ref_w[2], ref_h[2],
		scale_en, show_frame, ref_mode, ref_mode_word,
		pre174, pre15c, prea4, spins,
		rd(ctx, VDEC_HWV, V9_DONE), rd(ctx, VDEC_HWV, V9_STATUS15C),
		rd(ctx, VDEC_MISC, 0xa4), rd(ctx, VDEC_HWV, V9_BITPOS), crc0,
		rd(ctx, VDEC_MISC, MISC_CRC_BASE + 4),
		rd(ctx, VDEC_MISC, MISC_CRC_BASE + 8),
		rd(ctx, VDEC_MISC, MISC_CRC_BASE + 12));

	inst->frm_cnt++;
	return 0;
}

static int vdec_vp9_mt8365_get_param(void *h_vdec,
				     enum vdec_get_param_type type, void *out)
{
	struct vdec_vp9_mt8365_inst *inst = h_vdec;
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;

	switch (type) {
	case GET_PARAM_PIC_INFO:
		/*
		 * MM21 at the CODED SUPERBLOCK size. MEASURED against the
		 * vendor CAPTURE geometry on five resolutions: 176x144 ->
		 * 192x192, 192x192 -> 192x192, 320x240 -> 320x256, 160x128 ->
		 * 192x128, 96x64 -> 128x64, i.e. ALIGN(.,64) in BOTH axes.
		 * This differs from the VP8 / H.264 backends' 16x32.
		 */
		ctx->picinfo.buf_w = ALIGN(ctx->picinfo.pic_w, 64);
		ctx->picinfo.buf_h = ALIGN(ctx->picinfo.pic_h, 64);
		ctx->picinfo.fb_sz[0] = ctx->picinfo.buf_w * ctx->picinfo.buf_h;
		ctx->picinfo.fb_sz[1] = ctx->picinfo.fb_sz[0] >> 1;
		*(struct vdec_pic_info *)out = ctx->picinfo;
		break;
	case GET_PARAM_DPB_SIZE:
		*(unsigned int *)out = VP9_DPB_SIZE;
		break;
	case GET_PARAM_CROP_INFO: {
		struct v4l2_rect *cr = out;

		cr->left = 0;
		cr->top = 0;
		cr->width = ctx->picinfo.pic_w;
		cr->height = ctx->picinfo.pic_h;
		break;
	}
	default:
		return -EINVAL;
	}
	return 0;
}

static void vdec_vp9_mt8365_deinit(void *h_vdec)
{
	struct vdec_vp9_mt8365_inst *inst = h_vdec;

	if (inst)
		free_work_bufs(inst);
	kfree(h_vdec);
}

const struct vdec_common_if vdec_vp9_mt8365_if = {
	.init		= vdec_vp9_mt8365_init,
	.decode		= vdec_vp9_mt8365_decode,
	.get_param	= vdec_vp9_mt8365_get_param,
	.deinit		= vdec_vp9_mt8365_deinit,
};
