// SPDX-License-Identifier: GPL-2.0
/*
 * MT8365 (Genio 350) stateless HEVC/H.265 decode backend -- DIRECT REGISTER,
 * no SCP, no codec firmware, no vpud.
 *
 * Sibling of vdec_h264_mt8365_if.c / vdec_vp8_mt8365_if.c /
 * vdec_vp9_mt8365_if.c. MT8365 has no codec coprocessor: the VDEC is a
 * stateless accelerator historically driven by the closed userspace
 * libvpud_vcodec.so. This backend replaces "send to VPU/SCP + wait IPI" with
 * "program the VDEC registers + kick + poll".
 *
 * STATUS: FIRST CUT. What is PROVEN here is the CAPTURE-plane geometry (see
 * below) and the enumeration path. The register program in
 * hevc_program_picture() is a first, UNVALIDATED transcription of one proven
 * vendor trace and is gated behind the default-OFF module parameter
 * @mt8365_hevc_arm, so that merely advertising S265 cannot wedge the decoder
 * for the rest of a boot. With the arm OFF the decode path returns -EIO
 * without touching a register.
 *
 * SOURCE -- proven vendor mmiotraces captured on med-mt8365-pumpkin and
 * proven before use (bit-exact decode, md5 == the JCT-VC reference):
 *
 *   ~/bftrace/mmio-MERGE_A_TI_3.log   416x240 Main 8-bit 4:2:0, 8 coded
 *                                     pictures, 1 slice each, no tiles/WPP,
 *                                     CtbSizeY 64, 2734 CPU MMIO accesses
 *                                     (2026-08-08, md5 3a15886a..., 3 runs).
 *   ~/bftrace/mmio-MAXBINS_A_TI_5.log 416x240 Main 8-bit, CtbSizeY *16*,
 *                                     captured 2026-08-09 as the alignment
 *                                     discriminator (md5 1386fb14... == the
 *                                     JCT reference).
 *
 * The blind-instrument warning applies: regtrace.so (the *_HAL_WriteReg PLT
 * interposer) sees ZERO events on an HEVC decode because the H265 HAL stores
 * directly through the mapped window. Only mmiotrace.so can observe this path.
 *
 * Sub-block -> reg_base[] index (enum mtk_hw_reg_idx), mt8365.dtsi
 * codec@16000000, all already mapped -- M3b needs NO .dtsi change:
 *   VDEC_SYS 0x16000000  VDEC_MISC 0x16025000  VDEC_LD  0x16020000 (VLD)
 *   VDEC_TOP 0x16020800  VDEC_CM   0x16021000 (MC)
 *   VDEC_AV  0x16023000 (AVC_MV -- REUSED by the HEVC path for MV storage)
 *   VDEC_PP  0x16024000  VDEC_HWG 0x16028000 (HEVC_VLD, DT index 11)
 * The HEVC path never touches VDEC_AD (AVC_VLD) or any VP6/VP8/VP9 window.
 *
 * ============================ CAPTURE GEOMETRY ============================
 *
 * PROVEN, no free parameter, from the two traces above plus the H.264 control
 * (~/bftrace/mmio-FM2_SVA_C.log, 176x144):
 *
 *   The recon plane is written at the CODED BLOCK geometry
 *       ALIGN(pic_w, B) x ALIGN(pic_h, B)
 *   where B is the codec's largest coding block: 16 for an H.264 macroblock,
 *   64 for a VP9 superblock, and *CtbSizeY* for HEVC -- which is per-SPS and
 *   is NOT always 64.
 *
 *   Five register values across three streams and three block sizes, every one
 *   predicted with no fitted constant:
 *
 *     clip                B    MC+0x22c   MC+0x260  MC+0x264  TOP+0x094
 *                              (aw/16)    (aw/16-1) (ah/16-1) (rows<<16|cols)
 *     FM2_SVA_C  176x144  16   0x0b (176) 0x0a      0x08      --
 *     MERGE_A    416x240  64   0x1c (448) 0x1b      0x0f      0x00040007
 *     MAXBINS_A  416x240  16   0x1a (416) 0x19      0x0e      0x000f001a
 *
 *   MERGE and MAXBINS are the SAME resolution and differ ONLY in CtbSizeY, so
 *   the 448-vs-416 stride is attributable to nothing else. A fixed
 *   ALIGN(w,64) -- the VP9 rule -- would over-declare the stride by 32 px on
 *   every CtbSizeY<64 stream whose width is not already 64-aligned, which is
 *   exactly the per-tile-row diagonal shift that collapsed the H.264 path to
 *   ~8.6 dB before its geometry was corrected.
 *
 * ==== CORRECTION 2026-08-09 (step 6): the REGISTER stride is NOT the MEMORY
 * stride. ====
 *
 * Everything above is about the values the vendor writes into MC+0x22c /
 * MC+0x260 / MC+0x264 / TOP+0x094, and those remain exactly as derived --
 * ALIGN(w, CtbSizeY) -- and are still programmed that way below. What was WRONG
 * was carrying that rule over to the CAPTURE buffer we allocate and read.
 *
 * The vendor's own CAPTURE plane was measured directly (all three proven clips
 * decoded through the vendor driver in its NATIVE format, then each true luma
 * row LOCATED inside the tiled plane by search, assuming no tiling):
 *
 *   MERGE_A_TI_3   416x240 CtbSizeY 64 -> 26 tiles/row = stride 416, 256 rows
 *   MAXBINS_A_TI_5 416x240 CtbSizeY 16 -> 26 tiles/row = stride 416, 256 rows
 *   DBLK_A_SONY_3  832x480 CtbSizeY 64 -> 52 tiles/row = stride 832, 480 rows
 *
 * i.e. the MM21 memory geometry is ALIGN(w,16) x ALIGN(h,32), independent of
 * CtbSizeY -- MERGE and MAXBINS differ ONLY in CtbSizeY and share a stride, and
 * DBLK's 480 rows are not ALIGN(ALIGN(480,64),32) = 512. Our CPU de-tiler
 * reproduces the JCT-VC reference md5 of all three clips bit-exactly from the
 * vendor's own output under that geometry (the reader positive control), so it
 * is measured on both sides. @mt8365_hevc_geom keeps the old rule available.
 *
 * NOT proven by the traces: the in-memory ROW COUNT. MC+0x264 reports the
 * CODED height (ALIGN(h,B)); the MM21 luma tile is 32 rows tall, so a plane
 * must contain a whole number of tile rows and the declared height is taken
 * as ALIGN(ALIGN(h,B),32) -- consistent with the H.264 backend, where
 * MC+0x264 likewise says 144 while the declared CAPTURE height is 160. For
 * the two HEVC clips measured this is not separable (both give 256). What
 * would disambiguate it: a CtbSizeY<=32 clip whose ALIGN(h,B) is not a
 * multiple of 32, decoded through the vendor with its CAPTURE G_FMT read out,
 * or a pixel-level detile of such a clip against both hypotheses.
 */

#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "../mtk_vcodec_dec.h"
#include "../../common/mtk_vcodec_intr.h"
#include "../vdec_drv_base.h"
#include "../vdec_drv_if.h"

#define MT8365_HEVC_IRQ_TIMEOUT_MS	1000
#define HEVC_DPB_SIZE		9
#define HEVC_COLMV_BUF_SZ	(512 * 1024)
#define HEVC_NUM_REF_SLOTS	16
#define MT8365_HEVC_TIMEOUT	400000

/* ---- VDEC_SYS ---- */
#define SYS_CTRL		0x000
#define SYS_ENABLE		0x018
#define SYS_GATE80C		0x80c
/* ---- VDEC_MISC ---- */
#define MISC_CFG004		0x004
#define MISC_CFG084		0x084
#define MISC_CFG0C8		0x0c8
#define MISC_CFG0CC		0x0cc
#define MISC_CFG0EC		0x0ec
#define MISC_CFG0F4		0x0f4
#define MISC_CFG178		0x178
/* ---- VDEC_LD (VLD) ---- */
#define VLD_CFG08C		0x08c
#define VLD_BS_PAGE		0x0b4
#define VLD_BS_PAGE_LEN		0x0b8
#define VLD_BS_START		0x0b0
#define VLD_BS_END		0x110
#define VLD_CFG0EC		0x0ec
#define VLD_RESET		0x108
/* ---- VDEC_TOP ---- */
#define TOP_CTB_M1		0x068	/* (ctb_rows-1) << 16 | (ctb_cols-1) */
#define TOP_PICSIZE_PX		0x070	/* (h << 16) | w */
#define TOP_CTB_DIM		0x094	/* (ctb_rows << 16) | ctb_cols */
#define TOP_CFG100		0x100
/* ---- VDEC_CM (MC) ---- */
#define MC_PIC_W_PX		0x208
#define MC_PIC_H_PX		0x20c
#define MC_ENABLE220		0x220
#define MC_RECON_Y		0x224	/* recon Y >> 9 */
#define MC_RECON_C		0x228	/* recon C >> 8 */
#define MC_RECON_STRIDE_MB	0x22c	/* ALIGN(w, CtbSizeY) / 16 */
#define MC_INTER_EN		0x238
#define MC_ENABLE250		0x250
#define MC_MB_W_M1		0x260
#define MC_MB_H_M1		0x264
#define MC_REF_Y_A		0x45c	/* 16 slots, +4i */
#define MC_REF_Y_B		0x4dc	/* 16 slots, +4i (mirror) */
#define MC_REF_C_A		0xbe0	/* 16 slots, +4i */
#define MC_REF_C_B		0xc60	/* 16 slots, +4i (mirror) */
#define MC_RECON_LIMIT		0x850	/* recon write-DMA window LIMIT >> 4 (VP8/H.264) */
#define MC_RECON_BASE		0x854	/* recon write-DMA window BASE  >> 4 (VP8/H.264) */
#define MC_RECON_EN		0x858	/* recon write-DMA window enable (VP8/H.264) */
#define MC_STRIDE64		0x980	/* ALIGN(w,64) / 16 -- allocation stride */
#define MC_CFGAF8		0xaf8
#define MC_CFGA60		0xa60
#define MC_CFGB5C		0xb5c
/* ---- VDEC_AV (AVC_MV, reused by the HEVC path) ---- */
#define MV_COLMV_A		0x000	/* 16 slots, +4i, colmv >> 4 */
#define MV_COLMV_B		0x040	/* 16 slots, +4i (mirror) */
#define MV_CUR_COLMV		0x20c	/* current picture colmv >> 4 */
#define MV_CFG210		0x210
#define MV_CFG320		0x320
/* ---- VDEC_PP ---- */
#define PP_CFG040		0x040
#define PP_CFG630		0x630
#define PP_CFGB80		0xb80
/* ---- VDEC_HWG (HEVC_VLD) ---- */
#define H265_CFG084		0x084
#define H265_CFG088		0x088
#define H265_KICK		0x090	/* = 1 : DECODE GO */
#define H265_CFG094		0x094
/*
 * N37 -- the start-code-search strobe (H265_HAL_GetStartCode). +0x084 bit 4 is
 * the only bit the vendor ever writes there (mask 0x10), +0x094 = 0x100 starts
 * the search and +0x094 bit 8 reads back set while it runs.
 */
#define H265_SC_GO		BIT(4)		/* +0x084, masked write */
#define H265_SC_START		0x100		/* +0x094, full write */
#define H265_SC_BUSY		BIT(8)		/* +0x094, polled */
#define H265_SC_SPINS		0x100000	/* the vendor's own bound */
#define H265_SPS_A		0x0a0
#define H265_SPS_B		0x0a4
#define H265_SPS_C		0x0a8
#define H265_SPS_D		0x0ac
#define H265_SPS_E		0x0b0
#define H265_SPS_F		0x0b4
#define H265_SPS_G		0x0b8
#define H265_CFG0BC		0x0bc
#define H265_SHDR_A		0x0c0	/* QP + SAO + cabac_init */
#define H265_SHDR_B		0x0c4	/* ref counts, merge, deblock offsets */
#define H265_SHDR_C		0x0c8	/* RPS delta-poc counts */
#define H265_SHDR_D		0x0cc	/* (idx_width << 4) | NumPicTotalCurr */
#define H265_SHDR_E		0x0f4	/* 0xffff0015 | nal_unit_type << 8 */
#define H265_CFG0D0		0x0d0	/* per-picture slice-data extent */
#define H265_CFG0D4		0x0d4
#define H265_CFG0E0		0x0e0
#define H265_REFTAB		0x140	/* 16 words */
#define H265_REFTAB2		0x180	/* 16 words */
#define H265_CFG1C0		0x1c0
#define H265_CFG1D0		0x1d0
#define H265_LCU_XY		0x12c	/* (lcu_y << 16) | lcu_x, live position */
#define H265_EOBS_ERR		0x0e4	/* bit0 : end-of-bitstream error */
#define H265_DONE		0x278	/* bit31 : PICTURE COMPLETE (vendor) */
#define H265_ERR_CODE		0x284
#define H265_ERR_MASK		0x280
#define H265_PIC_START		0x2ac	/* = 0x80000000 : new-picture latch */

/*
 * How the vendor decides a picture is DONE -- read out of libvpud_vcodec.so,
 * not guessed from the trace (the mmiotrace interposer records stores only, so
 * every vendor READ is structurally invisible to it):
 *
 *   H265_HAL_WaitISR      @0x69ca0 -- does NOT poll. It builds a wait
 *                                     descriptor and blocks in eHalCmdProc(0),
 *                                     i.e. it sleeps on the kernel VDEC IRQ.
 *   H265_HAL_ReadFinishFlag @0x69020 -- reads VDEC_MISC +0xa4 and returns
 *                                     bit 16 (ldrh [sp,#6] then AND #1).
 *   H265_HAL_DecPicComplete @0x64ed0 -- reads HEVC_VLD +0x278, returns bit 31.
 *   bH265_HAL_IsVDecComplete@0x68ef0 -- reads HEVC_VLD +0x12c as
 *                                     (lcu_y << 16) | lcu_x and declares the
 *                                     picture complete when both reach the
 *                                     last CTB.
 *
 * VDEC_MISC +0xa4 bit16 is independently confirmed by this very driver:
 * mtk_vcodec_dec_hw.h has VDEC_IRQ_CFG_REG 0xa4 and mtk_vcodec_dec.h has
 * MTK_VDEC_IRQ_STATUS_DEC_SUCCESS 0x10000. The core ISR reads it, ACKS it and
 * wakes the waiter -- which is why polling it (or anything else) from the
 * decode thread races the ISR and can never be relied on.
 *
 * +0x2ac, which this backend polled through step 4, is the NEW-PICTURE LATCH
 * (established in step 2 when it was wrongly used to segment the trace). It
 * never clears on completion, so the wait could only ever time out.
 */

/*
 * Default OFF. 0 = advertise S265 and fail the decode cleanly without touching
 * a register (safe: cannot wedge the decoder, so a conformance battery in the
 * same boot stays valid). 1 = run the trace-derived program and kick the HW.
 */
static int mt8365_hevc_arm;
module_param(mt8365_hevc_arm, int, 0644);
MODULE_PARM_DESC(mt8365_hevc_arm,
		 "mt8365 HEVC M3b: 0 = enumerate only, decode returns -EIO without touching the HW (default); 1 = run the UNVALIDATED trace-derived register program and kick");

/*
 * Execution instrument. @hevc_cand counts every decode() call that reached the
 * backend with a bitstream buffer -- counted UNCONDITIONALLY, so the
 * denominator is observable with the arm off. @hevc_appl counts the pictures
 * that actually reached the hardware kick.
 */
static int hevc_cand;
module_param(hevc_cand, int, 0644);
MODULE_PARM_DESC(hevc_cand,
		 "mt8365 HEVC M3b instrument: decode() calls that reached the backend with an AU (candidates; counted even when mt8365_hevc_arm is 0)");

static int hevc_appl;
module_param(hevc_appl, int, 0644);
MODULE_PARM_DESC(hevc_appl,
		 "mt8365 HEVC M3b instrument: pictures whose register program actually reached the HW kick");

/*
 * Done-source selector, so the step-4 behaviour stays reproducible:
 *   0 = the vendor path -- sleep on the VDEC dec-done IRQ (default)
 *   1 = busy-poll HEVC_VLD +0x278 bit31 (H265_HAL_DecPicComplete)
 *   2 = the OLD step-4 behaviour: busy-poll the +0x2ac latch clearing
 */
static int mt8365_hevc_done_src;
module_param(mt8365_hevc_done_src, int, 0644);
MODULE_PARM_DESC(mt8365_hevc_done_src,
		 "mt8365 HEVC completion source: 0 = VDEC dec-done IRQ (vendor, default), 1 = poll HEVC_VLD+0x278 bit31, 2 = poll the +0x2ac latch (the step-4 behaviour, always times out)");

/*
 * POSITIVE CONTROL. 1 = program every register for the picture but do NOT write
 * the kick, and return success immediately. The CAPTURE buffer is then dequeued
 * having been touched by nothing but the allocator, which is the only way to
 * tell decoder-written pixels from an allocation pattern.
 */
static int mt8365_hevc_nokick;
module_param(mt8365_hevc_nokick, int, 0644);
MODULE_PARM_DESC(mt8365_hevc_nokick,
		 "mt8365 HEVC positive control: 1 = program the picture but skip the kick and return success (unkicked recon buffer)");

/* 1 = dump the status-probe set pre-kick and at completion/timeout. */
static int mt8365_hevc_probe;
module_param(mt8365_hevc_probe, int, 0644);
MODULE_PARM_DESC(mt8365_hevc_probe,
		 "mt8365 HEVC: 1 = log the status-register probe set pre-kick and at completion/timeout");

/*
 * N44 -- the IN-LOOP FILTER arm.
 *
 * N42 measured the recon write-out's exact footprint with the CAPTURE planes
 * pre-filled with a poison byte, re-plotted in the geometry the engine is
 * actually programmed with (MC +0x22c = 28 tiles = 448 px on MERGE_A_TI_3):
 * the frame-level alignment padding is filled for the WHOLE surface, real
 * pixels stop inside CTB 0, and EVERY touched MM21 tile is short by exactly
 * FOUR rows -- 28 of 32 luma rows, 12 of 16 chroma rows.
 *
 * Four rows is not a fraction (28/32 = 7/8, 12/16 = 3/4); it is an ABSOLUTE
 * row constant that is the SAME in both planes even though they have different
 * vertical subsampling. A row-granular constant that does not scale with
 * subsampling is the fingerprint of a per-plane line buffer, and HEVC has
 * exactly one thing of that shape: the in-loop filters. Deblocking across a
 * horizontal CTB edge rewrites the 4 rows above it, so those rows cannot be
 * committed until the CTB row BELOW has been reconstructed; SAO adds one more
 * row of neighbour dependence. If the last rows of every band are held pending
 * a filter flush that only fires at picture completion -- and GetErrMsg says
 * the picture is never completed -- then "4 rows short" and "0x440c0040" have
 * ONE cause.
 *
 * This arm removes the dependence instead of chasing the flush: with the loop
 * filters disabled there is nothing to hold back, so a held-rows hypothesis
 * predicts the write-out reaches the bottom of each band. The output is NOT
 * expected to be conformant -- the oracle is the poison scan's byte count and
 * footprint, not the md5.
 *
 * Bitmask, default 0 (nothing touched):
 *   bit0  deblocking OFF -- slice_deblocking_filter_disabled (+0x0c4 bit30)
 *         forced SET, and the PPS deblock-enable (+0x0b8 bit0) forced CLEAR.
 *   bit1  SAO OFF -- slice_sao_luma/chroma (+0x0c0 bits 5,6) and the SPS
 *         sample_adaptive_offset_enabled (+0x0a0 bit5) forced CLEAR.
 *   bit2  loop-filter-across-slices OFF -- slice (+0x0c4 bit29) and PPS
 *         (+0x0b4 bit24) forced CLEAR.
 *
 * hevc_nolf_cand counts every picture that reached the packing point with the
 * instrument compiled in (the arm-OFF denominator); hevc_nolf_appl counts the
 * pictures whose words were actually rewritten.
 */
static int mt8365_hevc_nolf;
module_param(mt8365_hevc_nolf, int, 0644);
MODULE_PARM_DESC(mt8365_hevc_nolf,
		 "mt8365 HEVC N44: bitmask disabling the in-loop filters (0 = off, bit0 deblock, bit1 SAO, bit2 across-slices)");

static int hevc_nolf_cand;
module_param(hevc_nolf_cand, int, 0444);
MODULE_PARM_DESC(hevc_nolf_cand, "mt8365 HEVC N44: pictures packed with the loop-filter arm compiled in (arm-OFF denominator)");

static int hevc_nolf_appl;
module_param(hevc_nolf_appl, int, 0444);
MODULE_PARM_DESC(hevc_nolf_appl, "mt8365 HEVC N44: pictures whose loop-filter fields were actually rewritten");

/*
 * N15 -- the recon write-DMA address WINDOW.
 *
 * MEASURED on this SoC by the VP8 backend (vdec_vp8_mt8365_if.c): MC+0x854 is
 * the window BASE and MC+0x850 the window LIMIT, both >> 4, and MC+0x858 is the
 * enable; every byte the recon write DMA emits -- luma AND chroma -- must fall
 * inside [base, limit). With an inverted/short window the DMA writes NOTHING
 * while the reconstruction itself still completes and the picture CRC stays
 * correct, which is exactly the shape of the residual here (8/8 pictures
 * complete, the LCU counter reaches the last CTB, and only the first ~14 kB of
 * the recon plane is ever touched).
 *
 * The three registers do NOT appear in ANY of the three proven vendor HEVC
 * mmiotraces -- but that instrument records STORES only and the traces begin
 * after vpud has already initialised the block, so "absent from the trace" is
 * not evidence of "not required": it is exactly the blind-channel situation
 * that cost five rounds on the completion signal. This arm settles it by
 * experiment instead, in the VP8-measured union form.
 *
 *   0 = do not touch them (the step-5 behaviour, default)
 *   1 = program base = min(rec_y, rec_c) >> 4,
 *               limit = max(rec_y + luma_sz, rec_c + luma_sz / 2) >> 4,
 *               enable = 1
 */
static int mt8365_hevc_reconwin;
module_param(mt8365_hevc_reconwin, int, 0644);
MODULE_PARM_DESC(mt8365_hevc_reconwin,
		 "mt8365 HEVC N15: 0 = leave MC+0x850/0x854/0x858 alone (default); 1 = program the VP8-measured recon write-DMA union window + enable");

/*
 * N8/N15 -- the CAPTURE (MM21) in-memory geometry.
 *
 * MEASURED 2026-08-09 by decoding all three proven clips through the VENDOR
 * driver in its NATIVE CAPTURE format and locating each true luma row inside
 * the tiled plane (no assumption about the tiling was made -- the row map was
 * recovered by search):
 *
 *   clip            display   tiles/row   => tiled stride   rows
 *   MERGE_A_TI_3    416x240   26          416              256   CtbSizeY 64
 *   MAXBINS_A_TI_5  416x240   26          416              256   CtbSizeY 16
 *   DBLK_A_SONY_3   832x480   52          832              480   CtbSizeY 64
 *
 * i.e. stride = ALIGN(w, 16) and rows = ALIGN(h, 32) -- NOT ALIGN(w, CtbSizeY),
 * and NOT ALIGN(ALIGN(h, CtbSizeY), 32). MERGE and MAXBINS are the same
 * resolution with different CtbSizeY and land on the SAME stride, which is what
 * separates the two rules; DBLK's 480 rows separate ALIGN(h,32) from
 * ALIGN(ALIGN(h,64),32) = 512. With that geometry our own CPU de-tiler
 * reproduces the JCT-VC reference md5 of all three clips BIT-EXACTLY from the
 * vendor's own output (3a15886a.. / 1386fb14.. / 42486a48..), which is the
 * positive control that makes the geometry a measurement rather than a guess.
 *
 *   0 = the measured vendor geometry: ALIGN(w,16) x ALIGN(h,32)  (default)
 *   1 = the step-2 coded-CTB geometry: ALIGN(w,CtbSizeY) x ALIGN(ALIGN(h,CtbSizeY),32)
 */
static int mt8365_hevc_geom;
module_param(mt8365_hevc_geom, int, 0644);
MODULE_PARM_DESC(mt8365_hevc_geom,
		 "mt8365 HEVC CAPTURE geometry: 0 = measured vendor ALIGN(w,16) x ALIGN(h,32) (default), 1 = the step-2 coded-CTB rule");

/*
 * N35 -- THREE registers that were transcribed as MERGE_A_TI_3 LITERALS and are
 * in fact functions of the CTB grid. They are correct on MERGE only because
 * that is the clip they were read from; they are wrong on every stream with a
 * different CTB geometry.
 *
 * DERIVED off-board from all three proven, MD5_MATCH vendor mmiotraces --
 * MERGE_A_TI_3 (416x240 CtbSizeY 64 -> 7x4 CTBs), MAXBINS_A_TI_5 (416x240
 * CtbSizeY 16 -> 26x15) and DBLK_A_SONY_3 (832x480 CtbSizeY 64 -> 13x8) -- and
 * exact on all three with no free parameter:
 *
 *   HEVCVLD +0x140 (REFTAB  slot 0) = BIT(ctb_cols - 1)
 *                    MERGE 0x00000040 = 1<<6   (cols 7)
 *                    MAXB  0x02000000 = 1<<25  (cols 26)
 *                    DBLK  0x00001000 = 1<<12  (cols 13)
 *   HEVCVLD +0x180 (REFTAB2 slot 0) = BIT(ctb_rows - 1)
 *                    MERGE 0x00000008 = 1<<3   (rows 4)
 *                    MAXB  0x00004000 = 1<<14  (rows 15)
 *                    DBLK  0x00000080 = 1<<7   (rows 8)
 *   HEVCVLD +0x0bc [23:20]           = ceil_log2(ctb_cols * ctb_rows), i.e. the
 *                  number of bits needed to address a CTB in the picture
 *                    MERGE 5 (28 CTBs)  MAXB 9 (390)  DBLK 7 (104)
 *
 * The +0x0bc [23:20] reading is AMBIGUOUS on this evidence: 2*floor(log2(
 * ctb_cols)) + 1 fits all three equally well, because rows ~= cols/2 in every
 * clip we have. A clip with a power-of-two CTB count, or a square CTB grid,
 * would separate them. Reported as ambiguous rather than settled.
 *
 * The other 15 slots of both tables are 0 on all three clips, and +0x0bc's top
 * nibble varies per picture in a way we have not derived; only the picture-0
 * value is programmed here, which is what the picture-0 value-diff scores.
 *
 *   0 = the MERGE literals (default -- unchanged behaviour)
 *   1 = the derived CTB-grid values
 */
static int mt8365_hevc_ctbtab;
module_param(mt8365_hevc_ctbtab, int, 0644);
MODULE_PARM_DESC(mt8365_hevc_ctbtab,
		 "mt8365 HEVC CTB-grid registers: 0 = MERGE_A_TI_3 literals (default), 1 = derived BIT(cols-1)/BIT(rows-1)/ceil_log2(cols*rows)");

/* Unconditional candidate counter for the arm above (read via sysfs). */
static int hevc_ctbtab_cand;
module_param(hevc_ctbtab_cand, int, 0444);
MODULE_PARM_DESC(hevc_ctbtab_cand,
		 "mt8365 HEVC: times the derived CTB-grid values were COMPUTED (armed or not)");

static int hevc_ctbtab_appl;
module_param(hevc_ctbtab_appl, int, 0444);
MODULE_PARM_DESC(hevc_ctbtab_appl,
		 "mt8365 HEVC: times the derived CTB-grid values were APPLIED");

/*
 * ============ N37 -- the start-code-search STROBE, DERIVED ==================
 *
 * The reset prologue's `HEVCVLD +0x084 / +0x094` pair was transcribed as two
 * MERGE_A_TI_3 LITERALS (0x1011, 0x0011, each followed by +0x094 = 0x100).
 * It is neither a literal nor a fixed-length sequence.
 *
 * The vendor emitter is H265_HAL_GetStartCode@0x655d0 / _PicStart@0x653c0,
 * verbatim from objdump (the _PicStart copy at 0x654a8):
 *
 *   654a8: mov w3, #0x10       ; MASK  = 0x10
 *   654b0: mov w2, w3          ; VALUE = 0x10
 *   654b4: mov w1, #0x84
 *   654bc: bl  H265_HAL_WriteHEVCVLD
 *   654c8: mov w3, #0xffffffff ; MASK  = all
 *   654cc: mov w2, #0x100      ; VALUE = 0x100
 *   654d0: mov w1, #0x94
 *   654d4: bl  H265_HAL_WriteHEVCVLD
 *   654fc: mov w19, #0x100000  ; then poll ReadHEVCVLD(0x94) bit 8 until clear
 *
 * H265_HAL_WriteHEVCVLD(hal, off, val, mask, fp)@0x63ea0 forwards (val, mask)
 * to H265_HAL_WriteReg, i.e. it is a READ-MODIFY-WRITE. So every +0x084 value
 * in the archived traces is an ECHO of hardware state with bit 4 forced -- the
 * vendor never programs bits [12:11] at all:
 *
 *   MERGE_A_TI_3    0x1011, 0x0011                 (2 searches)
 *   MAXBINS_A_TI_5  0x1811                         (1 search)
 *   DBLK_A_SONY_3   0x0011, 0x0011, 0x1811, 0x1011 (4 searches)
 *
 * Two consequences, and NEITHER is visible to the picture-0 write-set diff
 * that has driven this bring-up: the offset SET is identical on both sides,
 * and the FINAL picture-0 value at +0x084 is 0x1 on both sides, so a
 * presence-or-final-value comparison scores zero. Only a per-offset
 * MULTIPLICITY comparison sees it (MAXBINS: ours +0x084 x4 / +0x094 x2, vendor
 * x3 / x1).
 *
 *   0 = the two MERGE literals, no wait (default -- unchanged behaviour)
 *   N = N vendor-shaped searches: +0x084 = rd|0x10 (read-modify-write),
 *       +0x094 = 0x100, then poll +0x094 bit 8 clear (bounded)
 *
 * Vendor-exact settings: MERGE_A_TI_3 = 2, MAXBINS_A_TI_5 = 1, DBLK = 4.
 */
static int mt8365_hevc_scstrobe;
module_param(mt8365_hevc_scstrobe, int, 0644);
MODULE_PARM_DESC(mt8365_hevc_scstrobe,
		 "mt8365 HEVC start-code-search strobe: 0 = MERGE_A_TI_3 absolute literals x2 (default), N = N read-modify-write searches with the +0x094 bit8 wait");

static int hevc_sc_cand;
module_param(hevc_sc_cand, int, 0444);
MODULE_PARM_DESC(hevc_sc_cand,
		 "mt8365 HEVC: times the start-code-strobe block was reached (armed or not)");

static int hevc_sc_appl;
module_param(hevc_sc_appl, int, 0444);
MODULE_PARM_DESC(hevc_sc_appl,
		 "mt8365 HEVC: times the vendor-shaped start-code strobe was APPLIED");

static int hevc_sc_spins;
module_param(hevc_sc_spins, int, 0444);
MODULE_PARM_DESC(hevc_sc_spins,
		 "mt8365 HEVC: total +0x094 bit8 poll iterations across all applied strobes");

static int hevc_sc_timeouts;
module_param(hevc_sc_timeouts, int, 0444);
MODULE_PARM_DESC(hevc_sc_timeouts,
		 "mt8365 HEVC: applied strobes whose +0x094 bit8 never cleared");

/* ceil(log2(n)) for n >= 1; ceil_log2(1) = 0. */
static u32 hevc_ceil_log2(u32 n)
{
	u32 w = 0;

	while ((1u << w) < n)
		w++;
	return w;
}

/*
 * N19 -- the END-OF-BITSTREAM SENTINEL the vendor writes into DRAM.
 *
 * MEASURED 2026-08-09 by widening mmiotrace.so to trap EVERY /dev/vcu mapping
 * vpud takes (44 of them; the previous capture trapped exactly one, the SFR
 * aperture, which is why every "the write set is complete" verdict up to step 6
 * was scoped to MMIO alone).
 *
 * Two 0x480000-byte /dev/vcu bitstream buffers are used ALTERNATELY, one per
 * picture, and the ONLY CPU writes the vendor makes into either are FIVE bytes
 * per picture:
 *
 *   00 00 00 01 ff
 *
 * i.e. a 4-byte Annex-B start code followed by 0xff, placed at the END of that
 * picture's copied bitstream chunk. Derived offset, with no free parameter:
 *
 *   sentinel_off = chunk_len + (number of 3-byte start codes in the chunk)
 *
 * where the chunk runs from this AU's first VCL NAL up to (not including) the
 * next AU's first VCL NAL (the first chunk starts at byte 0 so the VPS/SPS/PPS
 * travel with the IDR), and every 3-byte start code is normalised to 4 bytes.
 * 8/8 on MERGE_A_TI_3 and 2/2 on MAXBINS_A_TI_5 -- 10/10, both captures proven
 * by vendor output md5 == the JCT-VC reference in the same run.
 *
 * Our OUTPUT buffer carries one slice NAL and nothing else, so the equivalent
 * position is simply bs->size (one past the fed data). The sentinel lies PAST
 * the extent programmed into HEVCVLD +0x0d0, so this is a prefetch/terminator
 * hypothesis, not a length correction: nothing else in the write set changes.
 *
 *   0 = do not write it (default -- the step-6 behaviour)
 *   1 = write 00 00 00 01 ff at bs->va + bs->size when the plane has room
 */
static int mt8365_hevc_eosmark;
module_param(mt8365_hevc_eosmark, int, 0644);
MODULE_PARM_DESC(mt8365_hevc_eosmark,
		 "mt8365 HEVC N19: 0 = no end-of-bitstream sentinel (default); 1 = append the vendor's 00 00 00 01 ff at bs->size");

/*
 * Unconditional instrument: hevc_eos_cand counts every picture that REACHED the
 * sentinel site, hevc_eos_appl counts every picture where the five bytes were
 * actually stored. cand advances with the arm OFF too, so the denominator is
 * observable and execution is never inferred from pixels.
 */
static int hevc_eos_cand;
module_param(hevc_eos_cand, int, 0644);
MODULE_PARM_DESC(hevc_eos_cand, "mt8365 HEVC N19: pictures that reached the sentinel site (arm-independent)");

static int hevc_eos_appl;
module_param(hevc_eos_appl, int, 0644);
MODULE_PARM_DESC(hevc_eos_appl, "mt8365 HEVC N19: pictures where the sentinel was actually written");

static int hevc_eos_incoh;
module_param(hevc_eos_incoh, int, 0444);
MODULE_PARM_DESC(hevc_eos_incoh,
		 "mt8365 HEVC N45: sentinel writes SKIPPED because the source plane is cached (DMABUF/USERPTR or V4L2_MEMORY_FLAG_NON_COHERENT) and cannot be flushed legally from here");

static int hevc_wrlog;
module_param(hevc_wrlog, int, 0644);
MODULE_PARM_DESC(hevc_wrlog,
		 "mt8365 HEVC M3b: 1 = log every register write as H265WR <blk> +0xoff = 0xval");

/*
 * N21 -- the READ instrument.
 *
 * Every write-side channel is closed (MMIO write set 225 vs 225 with 0
 * vendor-only under a full 44-mapping trap; the vendor's whole CPU DRAM write
 * set is a pre-kick pool init plus a 5-byte sentinel that was falsified). The
 * named residual is the ABSOLUTE register STATE the block is in when the kick
 * lands -- what InitVDecHW left behind that our probe/reset did not. A write
 * trace cannot see that by construction, so this dumps the state instead.
 *
 * mt8365_hevc_regdump is a BITMASK over mtk_hw_reg_idx (bit N = block N), 0 =
 * off (default), -1 = every block. mt8365_hevc_regdump_pt selects the dump
 * point: bit0 = PREPROG (after our reset/window, before the picture program),
 * bit1 = PREKICK (immediately before the H265_KICK store -- the point the
 * userspace vendor instrument dumps at). Only the FIRST picture is dumped.
 *
 * Read-only, and read at a QUIESCENT point (nothing kicked, no fetch
 * outstanding) -- the H.264 backend's warning about sweeping VLD low offsets
 * applies while a fetch is in flight, which is not this point. The ranges are
 * exactly the 13 reg windows the mt8365 codec@16000000 DT node declares, so no
 * read leaves declared register space.
 */
static int mt8365_hevc_regdump;
module_param(mt8365_hevc_regdump, int, 0644);
MODULE_PARM_DESC(mt8365_hevc_regdump,
		 "mt8365 HEVC N21: bitmask of VDEC reg blocks to sweep-read (0 = off, -1 = all)");

static int mt8365_hevc_regdump_pt = 2;
module_param(mt8365_hevc_regdump_pt, int, 0644);
MODULE_PARM_DESC(mt8365_hevc_regdump_pt,
		 "mt8365 HEVC N21: dump points bitmask -- bit0 PREPROG, bit1 PREKICK, bit2 POSTKICK (default 2)");

/*
 * N24 -- the BISECT window inside the selected block(s).
 *
 * Reading the whole VDEC_LD window (0x20000..0x207ff, 512 words) at PREKICK
 * hard-resets the SoC, measured 2 of 2, against a clean negative control (the
 * same script with the mask at 0 completes the decode, and the step-8
 * SYS/TOP/MC/PP/MISC/HEVCVLD mask sweeps completed repeatedly). So VDEC_LD
 * cannot be swept whole; it has to be walked a few offsets at a time to find
 * which offsets are readable and where the boundary is.
 *
 * regdump_off/regdump_len are BLOCK-RELATIVE (both a multiple of 32 for the
 * 8-word line), applied to every block the mask selects. len 0 = to the end of
 * the block, which is the previous behaviour, so the defaults are a no-op.
 */
static int mt8365_hevc_regdump_off;
module_param(mt8365_hevc_regdump_off, int, 0644);
MODULE_PARM_DESC(mt8365_hevc_regdump_off,
		 "mt8365 HEVC N24: block-relative start offset of the sweep (default 0)");

static int mt8365_hevc_regdump_len;
module_param(mt8365_hevc_regdump_len, int, 0644);
MODULE_PARM_DESC(mt8365_hevc_regdump_len,
		 "mt8365 HEVC N24: sweep length in bytes (0 = to the end of the block)");

static int hevc_rd_appl;
module_param(hevc_rd_appl, int, 0644);
MODULE_PARM_DESC(hevc_rd_appl, "mt8365 HEVC N21: register sweeps actually performed");

static int hevc_rd_lines;
module_param(hevc_rd_lines, int, 0644);
MODULE_PARM_DESC(hevc_rd_lines, "mt8365 HEVC N24: 8-word lines actually read (survival counter)");

/*
 * N21 fix-arm -- force the vendor's pre-kick STATE on the registers the read
 * instrument found to differ and that NEITHER side writes this picture.
 *
 * Measured on MERGE_A_TI_3 picture 0, vendor (userspace sweep at its own kick)
 * vs ours (kernel sweep at ours), over VDEC_SYS / TOP / MC / PP / MISC /
 * HEVCVLD -- 4864 words, 0 one-sided:
 *
 *   TOP     +0x144   vendor 00000105   ours 00000005   (bit8)
 *   TOP     +0x224   vendor 00000005   ours 00000000
 *   HEVCVLD +0x084   vendor 00001801   ours 00001001   (bit11)
 *   HEVCVLD +0x098   vendor 00000000   ours 00000006
 *
 * SYS and PP are word-identical, so block enables and the post-processor are
 * NOT the divergence. The remaining differences are the bitstream shift-register
 * window (HEVCVLD +0x000..+0x080, the same bytes at a different fill point) and
 * MC/MISC internal line-buffer RAM, neither of which is a programmable input.
 *
 * These four are SINGLE-CLIP observations, not a derived model -- hence a
 * default-OFF bitmask arm scored by A/B, not a change to the program path:
 *   bit0 HEVCVLD+0x084 |= 0x800 · bit1 TOP+0x144 = 0x105
 *   bit2 TOP+0x224 = 0x005      · bit3 HEVCVLD+0x098 = 0
 */
static int mt8365_hevc_state;
module_param(mt8365_hevc_state, int, 0644);
MODULE_PARM_DESC(mt8365_hevc_state,
		 "mt8365 HEVC N21: bitmask forcing the vendor's pre-kick register state (0 = off)");

static int hevc_st_cand;
module_param(hevc_st_cand, int, 0644);
MODULE_PARM_DESC(hevc_st_cand, "mt8365 HEVC N21: pictures that reached the state-force site (arm-independent)");

static int hevc_st_appl;
module_param(hevc_st_appl, int, 0644);
MODULE_PARM_DESC(hevc_st_appl, "mt8365 HEVC N21: pictures where the state force was applied");

/*
 * N31 -- the post-decode Y/Cb/Cr digest (the one READ-side surface the twenty
 * write-set/pre-kick rounds are structurally blind to).
 *
 * H265_HAL_GetYCbCrCRC @0x65170 in libvpud_vcodec.so reads EIGHT words at
 * VDEC_MISC +0x08..+0x24 (0x16025008..0x16025024), or, when the HAL's
 * compressed/UFO flag is set, the alternate bank at UFO +0xa0..+0xbc
 * (UFO = 0x16024f00 = VDEC_PP +0xf00, so PP +0xfa0..+0xfbc). Its enable is
 * H265_HAL_EnableCRC -> WriteMISC(0x04, bEnable | (bMode ? 2 : 0)) -- i.e.
 * VDEC_MISC +0x04 bits[1:0].
 *
 * The vendor reads the digest at every picture completion on the DIRECT
 * (non-GCE) path, which is MT8365's path -- but H265_HAL_EnableCRC has ZERO
 * call sites and the reset prologue leaves MISC+0x04 = 0x100 (bits[1:0]
 * clear), so the vendor is reading a DISABLED block. This is untested silicon
 * and the arm is default OFF.
 *
 * mt8365_hevc_crc is the literal value written into MISC+0x04 bits[1:0]
 * (1 = enable, 3 = enable|mode); 0 = off. The prologue rewrites MISC+0x04
 * every decode, so the arm is applied AFTER that write and BEFORE the kick.
 * hevc_crc_cand advances unconditionally so the arm-off denominator is
 * observable.
 */
static int mt8365_hevc_crc;
module_param(mt8365_hevc_crc, int, 0644);
MODULE_PARM_DESC(mt8365_hevc_crc,
		 "mt8365 HEVC N31: value for VDEC_MISC+0x04 bits[1:0] (0 = off, 1 = CRC enable, 3 = enable|mode)");

static int hevc_crc_cand;
module_param(hevc_crc_cand, int, 0644);
MODULE_PARM_DESC(hevc_crc_cand, "mt8365 HEVC N31: kicks reached with the CRC instrument compiled in (arm-OFF denominator)");

static int hevc_crc_appl;
module_param(hevc_crc_appl, int, 0644);
MODULE_PARM_DESC(hevc_crc_appl, "mt8365 HEVC N31: kicks where the CRC enable was actually written");

static int hevc_crc_lines = 8;
module_param(hevc_crc_lines, int, 0644);
MODULE_PARM_DESC(hevc_crc_lines, "mt8365 HEVC N31: how many completed pictures report the digest");

/*
 * N39 -- the BITSTREAM BYTES the engine actually fetches.
 *
 * Every register that points at the bitstream has been proven identical to the
 * vendor's (N34/N35/N38), but a register diff cannot see the CONTENT at the
 * address it names. The engine reads those bytes from DRAM; if our OUTPUT
 * buffer holds something other than the clip's VCL NAL, every register can
 * match while the decode is fed different data. This is the channel that hid
 * the H.264 mvpair defect -- a CPU write to memory, invisible to an MMIO diff.
 *
 * READ-ONLY instrument: it prints, it never modifies the buffer or any
 * register, so the armed and unarmed decodes are bit-identical by construction.
 *
 * Reports, per candidate picture, from bs->va/bs->size:
 *   - bytesused, plane capacity, dma_addr and its 16-byte alignment (the
 *     engine's aligned fetch base is ALIGN_DOWN(read_ptr,16); when the read
 *     pointer is already 16-aligned the pre-roll is EMPTY, and that is a fact
 *     about our buffer, not an assumption),
 *   - crc32 over the whole bytesused extent -- the byte-for-byte comparator
 *     against the clip file's own VCL NAL, computed off-board,
 *   - the leading 48 and trailing 32 bytes (start-code inclusion and the tail),
 *   - the number of emulation-prevention 00 00 03 sequences present.
 */
static int mt8365_hevc_bsdump;
module_param(mt8365_hevc_bsdump, int, 0644);
MODULE_PARM_DESC(mt8365_hevc_bsdump,
		 "mt8365 HEVC N39: 0 = off (default); N = dump the OUTPUT-buffer bitstream content for the first N candidate pictures");

static int hevc_bs_cand;
module_param(hevc_bs_cand, int, 0644);
MODULE_PARM_DESC(hevc_bs_cand, "mt8365 HEVC N39: pictures that reached the bitstream-dump site (arm-independent denominator)");

static int hevc_bs_appl;
module_param(hevc_bs_appl, int, 0644);
MODULE_PARM_DESC(hevc_bs_appl, "mt8365 HEVC N39: pictures whose bitstream content was actually dumped");

/*
 * N41 -- the RECON WRITE-OUT probe.
 *
 * N40 established that the digest (VDEC_MISC +0x08..+0x24) is a validated
 * reconstructed-pixel oracle and that on MAXBINS_A_TI_5 it reports a
 * NON-UNIFORM surface, while the CAPTURE buffer that comes out of the same
 * decode is bit-for-bit ZERO (md5(319488 x 0x00) computed on-board equals the
 * output md5). Two mutually exclusive explanations fit that pair and no
 * register comparison can separate them:
 *
 *   (a) the recon write-out DMA never wrote the buffer at all, and the zeros
 *       are simply the allocator's, or
 *   (b) the DMA wrote, and what it wrote was zeros.
 *
 * The discriminator is to make the pre-decode contents distinguishable from
 * zero. @mt8365_hevc_poison, when non-zero, fills the CAPTURE luma and chroma
 * planes with that byte immediately before the picture is programmed, and the
 * post-completion scan reports, per plane, how many bytes still hold the
 * poison, how many are zero and how many are anything else, plus the first
 * offset that is no longer the poison byte.
 *
 *   poison survives   -> nothing was written: the write-out client is dead
 *   poison -> zeros    -> the DMA ran and delivered zeros
 *   poison -> content  -> pixels exist and are lost downstream
 *
 * This is the ONE arm in this driver that deliberately modifies the CAPTURE
 * buffer, so it is default OFF and its effect on the output md5 is expected by
 * construction; the digest, GetErrMsg and every register are untouched.
 * @hevc_poison_cand advances unconditionally as the arm-off denominator.
 */
static int mt8365_hevc_poison;
module_param(mt8365_hevc_poison, int, 0644);
MODULE_PARM_DESC(mt8365_hevc_poison,
		 "mt8365 HEVC N41: 0 = off (default); 1..255 = pre-fill the CAPTURE planes with this byte and scan them after dec-done");

static int hevc_poison_cand;
module_param(hevc_poison_cand, int, 0644);
MODULE_PARM_DESC(hevc_poison_cand, "mt8365 HEVC N41: pictures that reached the poison site (arm-independent denominator)");

static int hevc_poison_appl;
module_param(hevc_poison_appl, int, 0644);
MODULE_PARM_DESC(hevc_poison_appl, "mt8365 HEVC N41: pictures actually pre-filled");

/*
 * N41 -- the UFO window base, CORRECTED.
 *
 * The digest instrument has been reading its "UFO" bank at VDEC_PP +0xfa0 on
 * the strength of H265_HAL_WriteUFO's DEBUG path, which biases the traced PP
 * pointer by +0x800. That is not the runtime path: the shipped
 * H265_HAL_WriteUFO/ReadUFO take their window from an INDEPENDENT hal-struct
 * slot ([hal+120], vs [hal+88] for PP), and the archived vendor trace pins it
 * -- H265_HAL_SetAUOReg's two writes appear back to back as
 *
 *   49  W +0x21b5c = 0x00000000     (MC +0xb5c, RMW clearing bit 4)
 *   50  W +0x0080c = 0x00000000     (UFO +0x0c, RMW clearing bits[5:4])
 *
 * so the UFO window is at APERTURE offset 0x800, i.e. inside the VDEC_SYS
 * window, and UFO +0xa0..+0xbc is SYS +0x8a0..+0x8bc. Read both banks: the old
 * PP +0xfa0 lane is kept so the N32..N40 record stays comparable, and the
 * corrected SYS +0x8a0 lane is what the vendor's alternate digest branch would
 * actually read.
 */
#define HEVC_UFO_IN_SYS		0x800

/*
 * Aperture offset (from 0x16000000) and length of each VDEC register window,
 * transcribed from /proc/device-tree/soc/codec@16000000/reg on
 * med-mt8365-pumpkin. Index order is enum mtk_hw_reg_idx, which is the DT reg
 * order. Keeping the APERTURE offset (not the block-relative one) is what makes
 * this directly comparable to the userspace vendor dump, which only ever sees
 * one 0x40000 mapping based at 0x16000000.
 */
struct hevc_regwin {
	u32 aperture_off;
	u32 len;
};

static const struct hevc_regwin hevc_regwins[NUM_MAX_VDEC_REG_BASE] = {
	[VDEC_SYS]  = { 0x00000, 0x1000 },
	[VDEC_MISC] = { 0x25000, 0x1000 },
	[VDEC_LD]   = { 0x20000, 0x0800 },
	[VDEC_TOP]  = { 0x20800, 0x0800 },
	[VDEC_CM]   = { 0x21000, 0x1000 },
	[VDEC_AD]   = { 0x22000, 0x1000 },
	[VDEC_AV]   = { 0x23000, 0x1000 },
	[VDEC_PP]   = { 0x24000, 0x1000 },
	[VDEC_HWD]  = { 0x27800, 0x0800 },
	[VDEC_HWQ]  = { 0x26000, 0x0800 },
	[VDEC_HWB]  = { 0x26800, 0x0800 },
	[VDEC_HWG]  = { 0x28000, 0x0400 },
	[VDEC_HWV]  = { 0x28400, 0x0400 },
};

static const char * const mt8365_hevc_blkname[NUM_MAX_VDEC_REG_BASE] = {
	[VDEC_SYS] = "SYS",  [VDEC_MISC] = "MISC", [VDEC_LD] = "VLD",
	[VDEC_TOP] = "TOP",  [VDEC_CM] = "MC",     [VDEC_AD] = "AVCVLD",
	[VDEC_AV] = "AVCMV", [VDEC_PP] = "PP",     [VDEC_HWG] = "HEVCVLD",
};

static inline void wr(struct mtk_vcodec_dec_ctx *ctx, int blk, u32 off, u32 val)
{
	if (unlikely(hevc_wrlog))
		pr_info("H265WR %-7s +0x%03x = 0x%08x\n",
			mt8365_hevc_blkname[blk] ? mt8365_hevc_blkname[blk] : "?",
			off, val);
	writel(val, ctx->dev->reg_base[blk] + off);
}

static inline u32 rd(struct mtk_vcodec_dec_ctx *ctx, int blk, u32 off)
{
	return readl(ctx->dev->reg_base[blk] + off);
}

/*
 * N21 read instrument -- sweep every word of the selected reg windows and emit
 * them keyed by APERTURE offset, eight per line, so the output diffs directly
 * against the userspace vendor dump. See the module-param block above.
 */
static void hevc_regdump(struct mtk_vcodec_dec_ctx *ctx, const char *tag)
{
	int blk;

	for (blk = 0; blk < NUM_MAX_VDEC_REG_BASE; blk++) {
		u32 off, start, end;

		if (!(mt8365_hevc_regdump & (1 << blk)))
			continue;
		if (!hevc_regwins[blk].len || !ctx->dev->reg_base[blk])
			continue;
		/* N24: bisect window, clamped to the declared block. */
		start = min_t(u32, (u32)mt8365_hevc_regdump_off & ~31u,
			      hevc_regwins[blk].len);
		end = mt8365_hevc_regdump_len > 0
			? min_t(u32, start + (u32)mt8365_hevc_regdump_len,
				hevc_regwins[blk].len)
			: hevc_regwins[blk].len;
		for (off = start; off < end; off += 32) {
			u32 v[8];
			int i;

			for (i = 0; i < 8; i++)
				v[i] = rd(ctx, blk, off + 4 * i);
			pr_info("H265RD %s +0x%05x %08x %08x %08x %08x %08x %08x %08x %08x\n",
				tag, hevc_regwins[blk].aperture_off + off,
				v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]);
			hevc_rd_lines++;
		}
	}
	hevc_rd_appl++;
}

/*
 * Status-probe set. Read-only, so it cannot perturb the decode. Deliberately
 * NOT swept across a block: the H.264 backend established that a broad CPU read
 * of the VLD block's low offsets can hang the AXI bus while a fetch is
 * outstanding, so only named status offsets are read.
 */
struct hevc_probe_reg {
	u8 blk;
	u16 off;
	const char *name;
};

static const struct hevc_probe_reg hevc_probes[] = {
	{ VDEC_HWG,  H265_KICK,      "HEVCVLD+090 kick"  },
	{ VDEC_HWG,  H265_EOBS_ERR,  "HEVCVLD+0e4 eobs"  },
	{ VDEC_HWG,  H265_LCU_XY,    "HEVCVLD+12c lcuxy" },
	{ VDEC_HWG,  H265_DONE,      "HEVCVLD+278 done"  },
	{ VDEC_HWG,  H265_ERR_MASK,  "HEVCVLD+280 mask"  },
	{ VDEC_HWG,  H265_ERR_CODE,  "HEVCVLD+284 err"   },
	{ VDEC_HWG,  H265_PIC_START, "HEVCVLD+2ac latch" },
	{ VDEC_MISC, 0x0a4,          "MISC+0a4 irqsts"   },
	{ VDEC_MISC, 0x000,          "MISC+000"          },
	{ VDEC_MISC, 0x084,          "MISC+084"          },
	{ VDEC_MISC, 0x0ec,          "MISC+0ec"          },
	{ VDEC_CM,   MC_RECON_LIMIT, "MC+850 win limit"  },
	{ VDEC_CM,   MC_RECON_BASE,  "MC+854 win base"   },
	{ VDEC_CM,   MC_RECON_EN,    "MC+858 win enable" },
	{ VDEC_LD,   0x0e8,          "VLD+0e8 fetchok"   },
	{ VDEC_LD,   0x0f4,          "VLD+0f4 idle"      },
	{ VDEC_SYS,  0x000,          "SYS+000 hwactive"  },
};

static void hevc_probe_snap(struct mtk_vcodec_dec_ctx *ctx, u32 *v)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(hevc_probes); i++)
		v[i] = rd(ctx, hevc_probes[i].blk, hevc_probes[i].off);
}

/* Absolute values of the whole probe set -- needed to read a register whose
 * pre-kick value is the question (MC+0x850/0x854/0x858), which a MOVED-only
 * report cannot show.
 */
static void hevc_probe_dump(const char *tag, const u32 *v)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(hevc_probes); i++)
		pr_info("mt8365 hevc: %s %-18s %08x\n", tag, hevc_probes[i].name, v[i]);
}

/* Report every probe that MOVED -- a register that transitions across the wait
 * is evidence the engine ran; nothing moving anywhere is evidence it did not. */
static void hevc_probe_report(struct mtk_vcodec_dec_ctx *ctx, const char *tag,
			      const u32 *pre, const u32 *post)
{
	unsigned int i, moved = 0;

	for (i = 0; i < ARRAY_SIZE(hevc_probes); i++) {
		if (pre[i] == post[i])
			continue;
		moved++;
		pr_info("mt8365 hevc: %s MOVED %-18s %08x -> %08x\n",
			tag, hevc_probes[i].name, pre[i], post[i]);
	}
	if (!moved)
		pr_info("mt8365 hevc: %s NO probe register moved (%zu probed)\n",
			tag, ARRAY_SIZE(hevc_probes));
}

struct vdec_hevc_mt8365_inst {
	struct mtk_vcodec_dec_ctx *ctx;
	struct mtk_vcodec_mem colmv_buf;
	bool work_buf_ready;
	unsigned int frm_cnt;
	u32 ctb_size;			/* CtbSizeY of the active SPS */
};

/*
 * CtbSizeY from the stateless SPS control. Defaults to 64 -- the value 67 of
 * the 77 JCT-VC-HEVC_V1 vectors that parse actually use -- when the control is
 * absent, which is the only safe fallback because a too-large alignment
 * over-declares rather than truncates.
 */
static u32 hevc_ctb_size(struct mtk_vcodec_dec_ctx *ctx)
{
	const struct v4l2_ctrl_hevc_sps *sps;
	struct v4l2_ctrl *ctrl;
	u32 log2sz;

	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, V4L2_CID_STATELESS_HEVC_SPS);
	if (!ctrl)
		return 64;
	sps = ctrl->p_cur.p;
	if (!sps)
		return 64;
	log2sz = sps->log2_min_luma_coding_block_size_minus3 + 3 +
		 sps->log2_diff_max_min_luma_coding_block_size;
	if (log2sz < 4 || log2sz > 6)
		return 64;
	return 1u << log2sz;
}

static void free_work_bufs(struct vdec_hevc_mt8365_inst *inst)
{
	if (inst->colmv_buf.va)
		mtk_vcodec_mem_free(inst->ctx, &inst->colmv_buf);
	inst->work_buf_ready = false;
}

static int alloc_work_bufs(struct vdec_hevc_mt8365_inst *inst)
{
	inst->colmv_buf.size = HEVC_COLMV_BUF_SZ;
	if (mtk_vcodec_mem_alloc(inst->ctx, &inst->colmv_buf))
		return -ENOMEM;
	inst->work_buf_ready = true;
	return 0;
}

static int vdec_hevc_mt8365_init(struct mtk_vcodec_dec_ctx *ctx)
{
	struct vdec_hevc_mt8365_inst *inst;

	inst = kzalloc(sizeof(*inst), GFP_KERNEL);
	if (!inst)
		return -ENOMEM;
	inst->ctx = ctx;
	inst->ctb_size = 64;
	ctx->drv_handle = inst;
	return 0;
}

/*
 * The VDEC reset / bitstream-window prologue. Transcribed from the
 * MERGE_A_TI_3 trace rows 0..50; the vendor issues the sequence twice and both
 * passes are reproduced because their tails differ.
 */
/*
 * N37 -- the vendor's post-strobe wait: poll HEVCVLD +0x094 until bit 8 is
 * clear, bounded exactly as the blob bounds it. Returns the spin count.
 */
static void hevc_sc_wait(struct mtk_vcodec_dec_ctx *ctx)
{
	u32 n;

	for (n = 0; n < H265_SC_SPINS; n++) {
		if (!(rd(ctx, VDEC_HWG, H265_CFG094) & H265_SC_BUSY))
			break;
		cpu_relax();
	}
	hevc_sc_spins += n;
	if (n == H265_SC_SPINS)
		hevc_sc_timeouts++;
}

static void hevc_reset_and_window(struct mtk_vcodec_dec_ctx *ctx,
				  u32 bs_start, u32 bs_end, u32 cfg0bc)
{
	int pass;

	for (pass = 0; pass < 2; pass++) {
		wr(ctx, VDEC_SYS, SYS_CTRL, 0x1);
		wr(ctx, VDEC_LD, VLD_RESET, 0x101);
		wr(ctx, VDEC_MISC, MISC_CFG0F4, 0x0);
		wr(ctx, VDEC_LD, VLD_RESET, 0x101);
		wr(ctx, VDEC_MISC, MISC_CFG0C8, 0xfeff);
		wr(ctx, VDEC_MISC, MISC_CFG0CC, 0x77e30180);
		wr(ctx, VDEC_MISC, MISC_CFG178, 0xfffdfa60);
		wr(ctx, VDEC_MISC, MISC_CFG084, 0x2);
		wr(ctx, VDEC_LD, VLD_RESET, 0x0);
		wr(ctx, VDEC_SYS, SYS_ENABLE, 0x1);
		wr(ctx, VDEC_MISC, MISC_CFG0EC, 0x1);
		wr(ctx, VDEC_MISC, MISC_CFG004, 0x100);
		wr(ctx, VDEC_HWG, H265_CFG084, 0x1);
		wr(ctx, VDEC_LD, VLD_CFG0EC, 0x10000000);
		/*
		 * N7 -- DERIVED (was a fabricated "page" encoding that pointed
		 * the fetch engine at an unrelated physical address; that is
		 * why every kick timed out). H265_HAL_SetVLDVFIFO@0x69490 in
		 * libvpud_vcodec.so is two writes and nothing else:
		 *   WriteVLD(0xb4, arg_start >> 6);
		 *   WriteVLD(0xb8, arg_end   >> 6);
		 * i.e. both are plain 64-BYTE-GRANULE ADDRESSES, not a page
		 * base OR'd with a length. Confirmed against both proven
		 * traces: MERGE_A_TI_3 pic0 buffer base 0xfe800000 ->
		 * +0xb4 = 0x03fa0000, end 0xfe802600 -> +0xb8 = 0x03fa0098.
		 */
		wr(ctx, VDEC_LD, VLD_BS_PAGE, bs_start >> 6);
		wr(ctx, VDEC_LD, VLD_BS_PAGE_LEN, bs_end >> 6);
		wr(ctx, VDEC_LD, VLD_BS_START, bs_start);
		wr(ctx, VDEC_LD, VLD_BS_END, bs_end);
		wr(ctx, VDEC_LD, VLD_RESET, 0x100);
		wr(ctx, VDEC_LD, VLD_RESET, 0x0);
		wr(ctx, VDEC_LD, VLD_CFG08C, 0x00100000);
		wr(ctx, VDEC_LD, VLD_CFG08C, 0x00800000);
		if (pass == 0) {
			/* N37 -- see the mt8365_hevc_scstrobe block. */
			hevc_sc_cand++;
			if (mt8365_hevc_scstrobe > 0) {
				int k;

				for (k = 0; k < mt8365_hevc_scstrobe; k++) {
					wr(ctx, VDEC_HWG, H265_CFG084,
					   rd(ctx, VDEC_HWG, H265_CFG084) |
					   H265_SC_GO);
					wr(ctx, VDEC_HWG, H265_CFG094,
					   H265_SC_START);
					hevc_sc_wait(ctx);
				}
				hevc_sc_appl++;
			} else {
				wr(ctx, VDEC_HWG, H265_CFG084, 0x1011);
				wr(ctx, VDEC_HWG, H265_CFG094, 0x100);
				wr(ctx, VDEC_HWG, H265_CFG084, 0x0011);
				wr(ctx, VDEC_HWG, H265_CFG094, 0x100);
			}
		} else {
			wr(ctx, VDEC_HWG, H265_CFG088, 0x100);
			wr(ctx, VDEC_HWG, H265_CFG088, 0x0);
			wr(ctx, VDEC_HWG, H265_CFG0BC, cfg0bc);
		}
	}
}

/*
 * The 16-slot reference-surface table (trace rows 51..151). For an IDR every
 * slot carries the CURRENT recon -- the same self-reference convention the VP8
 * keyframe path uses, and what the trace's pre-first-kick window shows.
 */
static void hevc_program_reftab(struct mtk_vcodec_dec_ctx *ctx, u32 rec_y,
				u32 rec_c, u32 colmv)
{
	int i;

	wr(ctx, VDEC_CM, MC_CFGB5C, 0x0);
	wr(ctx, VDEC_SYS, SYS_GATE80C, 0x0);
	wr(ctx, VDEC_PP, PP_CFGB80, 0x00808010);
	wr(ctx, VDEC_SYS, SYS_GATE80C, 0x0);
	wr(ctx, VDEC_CM, MC_CFGA60, 0x0);
	for (i = 0; i < HEVC_NUM_REF_SLOTS; i++) {
		wr(ctx, VDEC_CM, MC_REF_Y_B + 4 * i, rec_y);
		wr(ctx, VDEC_CM, MC_REF_Y_A + 4 * i, rec_y);
		wr(ctx, VDEC_CM, MC_REF_C_A + 4 * i, rec_c);
		wr(ctx, VDEC_CM, MC_REF_C_B + 4 * i, rec_c);
		wr(ctx, VDEC_AV, MV_COLMV_A + 4 * i, colmv >> 4);
		wr(ctx, VDEC_AV, MV_COLMV_B + 4 * i, colmv >> 4);
	}
}

/*
 * ===================== SPS/PPS PACKING -- DERIVED (N6) =====================
 *
 * H265_SPS_A..G (HEVCVLD +0x0a0 .. +0x0b8) are no longer clip literals. Each
 * bit field below was read out of libvpud_vcodec.so's H265_HAL_SetSPSHEVLD
 * (@0x67500) and H265_HAL_SetPPSHEVLD (@0x67860) -- the two functions are a
 * straight-line sequence of ubfiz/orr into one word per register, so the
 * layout is a transcription of the code, not a fit to the data. The vendor
 * SPS/PPS struct offsets fall in exact syntax-element order, which is what
 * pins each field to its meaning.
 *
 * VERIFIED off-board against BOTH proven vendor traces -- 20 of 20 words
 * (7 HEVCVLD + 3 VLDTOP per clip) reproduced exactly:
 *
 *   MERGE_A_TI_3   CtbSizeY 64  a0=0x0b0077b1 a4=0x00011408 a8=0x22575263
 *                               ac=0x0211000b b0=0x00001a60 b4=0x01000000
 *                               b8=0x00000001
 *   MAXBINS_A_TI_5 CtbSizeY 16  a0=0x010077b5 a4=0x00011108 a8=0x00432243
 *                               ac=0x0233000b b0=0x00001a40 b4=0x01000000
 *                               b8=0x00000001
 *
 * The two clips differ in CtbSizeY, MaxTb, transform-hierarchy depth, PCM,
 * num_short_term_ref_pic_sets and num_ref_idx defaults, so those separations
 * are measured rather than assumed.
 *
 * ONE non-derived constant, named: when pcm_enabled_flag is 0 the vendor still
 * emits Log2MinIpcmCbSizeY = 7 / Log2MaxIpcmCbSizeY = 5 in +0x0a8 bits
 * [18:16]/[22:20] (MERGE's values -- 7 is not even a legal PCM size, so this is
 * vpud's uninitialised SPS default, a don't-care while +0x0a0 bit2 is clear).
 * They are reproduced verbatim so a pcm-disabled stream matches the vendor
 * bit-for-bit; @mt8365_hevc_pcmdef=0 switches to the V4L2-derived values for
 * an A/B if the PCM path ever matters.
 *
 * All fields come from V4L2_CID_STATELESS_HEVC_SPS / _PPS. Nothing needed by
 * this packing is absent from the uAPI.
 */
static int mt8365_hevc_pcmdef = 1;
module_param(mt8365_hevc_pcmdef, int, 0644);
MODULE_PARM_DESC(mt8365_hevc_pcmdef,
		 "mt8365 HEVC: 1 = reproduce the vendor's pcm-disabled don't-care defaults (7,5) in HEVCVLD+0x0a8 (default); 0 = use the V4L2-derived PCM sizes");

struct hevc_spspps_words {
	u32 a0, a4, a8, ac, b0, b4, b8;
	u32 top68, top70, top94, top100;
	bool valid;
};

static u32 hevc_clog2(u32 v)
{
	u32 n = 1;

	if (v <= 2)
		return 1;
	while ((1u << (n + 1)) < v)
		n++;
	return n + 1;
}

static void hevc_pack_spspps(struct mtk_vcodec_dec_ctx *ctx,
			     struct hevc_spspps_words *w)
{
	const struct v4l2_ctrl_hevc_sps *s;
	const struct v4l2_ctrl_hevc_pps *p;
	struct v4l2_ctrl *cs, *cp;
	u32 ctb, minTb, maxTb, W, H, cw, ch, pcm_min, pcm_max, mincuqp;
	u32 pcm_bd_l, pcm_bd_c;

	memset(w, 0, sizeof(*w));
	cs = v4l2_ctrl_find(&ctx->ctrl_hdl, V4L2_CID_STATELESS_HEVC_SPS);
	cp = v4l2_ctrl_find(&ctx->ctrl_hdl, V4L2_CID_STATELESS_HEVC_PPS);
	if (!cs || !cp || !cs->p_cur.p || !cp->p_cur.p)
		return;
	s = cs->p_cur.p;
	p = cp->p_cur.p;

	ctb = s->log2_min_luma_coding_block_size_minus3 + 3 +
	      s->log2_diff_max_min_luma_coding_block_size;
	minTb = s->log2_min_luma_transform_block_size_minus2 + 2;
	maxTb = minTb + s->log2_diff_max_min_luma_transform_block_size;
	W = s->pic_width_in_luma_samples;
	H = s->pic_height_in_luma_samples;
	if (!W || !H || ctb < 4 || ctb > 6)
		return;
	cw = DIV_ROUND_UP(W, 1u << ctb);
	ch = DIV_ROUND_UP(H, 1u << ctb);

	if (s->flags & V4L2_HEVC_SPS_FLAG_PCM_ENABLED) {
		pcm_min = s->log2_min_pcm_luma_coding_block_size_minus3 + 3;
		pcm_max = pcm_min + s->log2_diff_max_min_pcm_luma_coding_block_size;
		pcm_bd_l = s->pcm_sample_bit_depth_luma_minus1;
		pcm_bd_c = s->pcm_sample_bit_depth_chroma_minus1;
	} else if (mt8365_hevc_pcmdef) {
		/*
		 * MEASURED on silicon 2026-08-09: with pcm_enabled_flag 0 the
		 * V4L2 SPS control carries pcm_sample_bit_depth_*_minus1 = 0,
		 * so +0x0a0 came out 0x0b0000b1 against the vendor's
		 * 0x0b0077b1 -- the only one of the seven SPS/PPS words that
		 * did not match. vpud's uninitialised default is 7/7 (i.e.
		 * 8-bit PCM), and 7 is also what the pcm-ENABLED clip
		 * (MAXBINS) carries, so 7/7 is the don't-care value in both
		 * directions.
		 */
		pcm_min = 7;
		pcm_max = 5;
		pcm_bd_l = 7;
		pcm_bd_c = 7;
	} else {
		pcm_min = 3;
		pcm_max = 3;
		pcm_bd_l = s->pcm_sample_bit_depth_luma_minus1;
		pcm_bd_c = s->pcm_sample_bit_depth_chroma_minus1;
	}

	/* VLDTOP -- H265_HAL_SetSPSHEVLD writes these four first. */
	w->top68  = ((ch - 1) & 0xfff) << 16 | ((cw - 1) & 0xfff);
	w->top70  = (H << 16) | (W & 0xffff);
	w->top94  = ((ch & 0xfff) << 16) | (cw & 0xfff);
	w->top100 = ((u32)s->bit_depth_chroma_minus8 & 0xf) << 4 |
		    ((u32)s->bit_depth_luma_minus8 & 0xf);

	/* HEVCVLD +0x0a0 */
	w->a0  = (u32)s->chroma_format_idc & 3;
	w->a0 |= !!(s->flags & V4L2_HEVC_SPS_FLAG_PCM_ENABLED) << 2;
	w->a0 |= !!(s->flags & V4L2_HEVC_SPS_FLAG_AMP_ENABLED) << 4;
	w->a0 |= !!(s->flags & V4L2_HEVC_SPS_FLAG_SAMPLE_ADAPTIVE_OFFSET) << 5;
	w->a0 |= !!(s->flags & V4L2_HEVC_SPS_FLAG_PCM_LOOP_FILTER_DISABLED) << 6;
	w->a0 |= !!(s->flags & V4L2_HEVC_SPS_FLAG_SPS_TEMPORAL_MVP_ENABLED) << 7;
	w->a0 |= (pcm_bd_l & 0xf) << 8;
	w->a0 |= (pcm_bd_c & 0xf) << 12;
	w->a0 |= ((u32)s->num_long_term_ref_pics_sps & 0x3f) << 16;
	w->a0 |= !!(s->flags & V4L2_HEVC_SPS_FLAG_LONG_TERM_REF_PICS_PRESENT) << 22;
	w->a0 |= ((u32)s->num_short_term_ref_pic_sets & 0x7f) << 24;

	/* HEVCVLD +0x0a4 */
	w->a4  = ((u32)s->log2_max_pic_order_cnt_lsb_minus4 + 4) & 0x1f;
	w->a4 |= (hevc_clog2(s->num_short_term_ref_pic_sets & 0x7f) & 7) << 8;
	w->a4 |= (hevc_clog2(s->num_long_term_ref_pics_sps & 0x3f) & 7) << 12;
	w->a4 |= !!(s->flags & V4L2_HEVC_SPS_FLAG_STRONG_INTRA_SMOOTHING_ENABLED) << 16;

	/* HEVCVLD +0x0a8 */
	w->a8  = ((u32)s->log2_min_luma_coding_block_size_minus3 + 3) & 7;
	w->a8 |= (ctb & 7) << 4;
	w->a8 |= (minTb & 7) << 8;
	w->a8 |= (maxTb & 7) << 12;
	w->a8 |= (pcm_min & 7) << 16;
	w->a8 |= (pcm_max & 7) << 20;
	w->a8 |= ((u32)s->max_transform_hierarchy_depth_inter & 7) << 24;
	w->a8 |= ((u32)s->max_transform_hierarchy_depth_intra & 7) << 28;

	/* HEVCVLD +0x0ac */
	w->ac  = !!(p->flags & V4L2_HEVC_PPS_FLAG_SIGN_DATA_HIDING_ENABLED);
	w->ac |= !!(p->flags & V4L2_HEVC_PPS_FLAG_CABAC_INIT_PRESENT) << 1;
	w->ac |= !!(p->flags & V4L2_HEVC_PPS_FLAG_CONSTRAINED_INTRA_PRED) << 2;
	w->ac |= !!(p->flags & V4L2_HEVC_PPS_FLAG_TRANSFORM_SKIP_ENABLED) << 3;
	w->ac |= !!(p->flags & V4L2_HEVC_PPS_FLAG_WEIGHTED_PRED) << 4;
	w->ac |= !!(p->flags & V4L2_HEVC_PPS_FLAG_WEIGHTED_BIPRED) << 5;
	w->ac |= !!(p->flags & V4L2_HEVC_PPS_FLAG_OUTPUT_FLAG_PRESENT) << 6;
	w->ac |= !!(p->flags & V4L2_HEVC_PPS_FLAG_TRANSQUANT_BYPASS_ENABLED) << 7;
	w->ac |= !!(p->flags & V4L2_HEVC_PPS_FLAG_DEPENDENT_SLICE_SEGMENT_ENABLED) << 8;
	w->ac |= !!(p->flags & V4L2_HEVC_PPS_FLAG_ENTROPY_CODING_SYNC_ENABLED) << 9;
	w->ac |= !!(p->flags & V4L2_HEVC_PPS_FLAG_SLICE_SEGMENT_HEADER_EXTENSION_PRESENT) << 11;
	w->ac |= ((u32)p->num_ref_idx_l0_default_active_minus1 & 0xf) << 16;
	w->ac |= ((u32)p->num_ref_idx_l1_default_active_minus1 & 0xf) << 20;
	w->ac |= (((u32)p->log2_parallel_merge_level_minus2 + 2) & 7) << 24;
	w->ac |= ((u32)p->num_extra_slice_header_bits & 7) << 28;

	/* HEVCVLD +0x0b0 */
	mincuqp = ctb - ((p->flags & V4L2_HEVC_PPS_FLAG_CU_QP_DELTA_ENABLED) ?
			 p->diff_cu_qp_delta_depth : 0);
	w->b0  = !!(p->flags & V4L2_HEVC_PPS_FLAG_CU_QP_DELTA_ENABLED);
	w->b0 |= !!(p->flags & V4L2_HEVC_PPS_FLAG_PPS_SLICE_CHROMA_QP_OFFSETS_PRESENT) << 1;
	w->b0 |= (mincuqp & 7) << 4;
	w->b0 |= (((u32)((int)p->init_qp_minus26 + 26)) & 0x7f) << 8;
	w->b0 |= ((u32)p->pps_cb_qp_offset & 0x1f) << 16;
	w->b0 |= ((u32)p->pps_cr_qp_offset & 0x1f) << 24;

	/* HEVCVLD +0x0b4 */
	w->b4  = !!(p->flags & V4L2_HEVC_PPS_FLAG_TILES_ENABLED);
	w->b4 |= ((u32)p->pps_beta_offset_div2 & 0x1f) << 8;
	w->b4 |= ((u32)p->pps_tc_offset_div2 & 0x1f) << 16;
	w->b4 |= !!(p->flags & V4L2_HEVC_PPS_FLAG_PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED) << 24;

	/* HEVCVLD +0x0b8 -- bit0 is DEBLOCKING ENABLED (the inverse of the
	 * PPS disable flag), which is why both traces read 1.
	 */
	w->b8  = !(p->flags & V4L2_HEVC_PPS_FLAG_PPS_DISABLE_DEBLOCKING_FILTER);
	w->b8 |= !!(p->flags & V4L2_HEVC_PPS_FLAG_DEBLOCKING_FILTER_OVERRIDE_ENABLED) << 1;
	/*
	 * +0x0b8 bit2 is pps_scaling_list_data_present_flag, and that is the ONE
	 * field this packing needs that the V4L2 HEVC uAPI does NOT expose --
	 * there is no V4L2_HEVC_PPS_FLAG_* for it (the scaling data itself
	 * arrives out-of-band via V4L2_CID_STATELESS_HEVC_SCALING_MATRIX, with
	 * no per-PPS present bit). Both proven traces read 0 here. Approximate
	 * it from the SPS enable, which is the only recoverable signal, and note
	 * that a stream with an SPS scaling list but no PPS override will
	 * over-set this bit.
	 */
	w->b8 |= !!(s->flags & V4L2_HEVC_SPS_FLAG_SCALING_LIST_ENABLED) << 2;
	w->b8 |= !!(p->flags & V4L2_HEVC_PPS_FLAG_LISTS_MODIFICATION_PRESENT) << 3;

	w->valid = true;
}

/*
 * ================== SLICE-HEADER PACKING -- DERIVED (N9) ==================
 *
 * HEVCVLD +0x0c0/+0x0c4/+0x0c8/+0x0cc/+0x0f4, transcribed field-for-field from
 * H265_HAL_SetSHDRHEVLD (libvpud_vcodec.so @0x67ad4) -- a straight-line
 * ubfiz/orr sequence with one ceil-log2 loop, exactly like the SPS/PPS setters.
 * Every struct offset it reads is NAMED by the blob's own debug fprintf format
 * strings inside H265_parseSliceHeader @0x72d10, which is what pins each
 * bitfield to its syntax element rather than to a trace coincidence:
 *
 *   [x20,#769] slice_temporal_mvp_enable_flag  [x20,#796] SliceQpY
 *   [x20,#770] slice_sao_luma_flag             [x20,#800] slice_cb_qp_offset
 *   [x20,#771] slice_sao_chroma_flag           [x20,#804] slice_cr_qp_offset
 *   [x20,#772] num_ref_idx_l0_active           [x20,#809] slice_deblocking_..disabled
 *   [x20,#776] num_ref_idx_l1_active           [x20,#812] slice_beta_offset_div2
 *   [x20,#785] mvd_l1_zero_flag                [x20,#816] slice_tc_offset_div2
 *   [x20,#786] cabac_init_flag                 [x20,#820] slice_loop_filter_across_slices
 *   [x20,#787] collocated_from_l0_flag         [x20,#20]  nal_unit_type
 *   [x20,#788] collocated_ref_idx              [x20,#792] five_minus_max_num_merge_cand
 *
 * Layout:
 *   +0x0c0  5 sao_luma & SPS_sao   6 sao_chroma & SPS_sao   7 cabac_init
 *           [14:8] SliceQpY   [20:16] pps_cb+slice_cb   [28:24] pps_cr+slice_cr
 *   +0x0c4  [3:0] l0_active-1  [7:4] l1_active-1  [11:8] collocated_ref_idx
 *           [14:12] 5 - five_minus_max_num_merge_cand
 *           [20:16] slice_beta_offset_div2   [25:21] slice_tc_offset_div2
 *           26 temporal_mvp  27 collocated_from_l0  28 mvd_l1_zero
 *           29 loop_filter_across_slices      30 slice_deblocking_disabled
 *   +0x0c8  [4:0] num_delta_pocs_of_ref_rps_idx
 *           [13:8] NumDeltaPocs of the LAST EXPLICITLY-CODED SPS st_ref_pic_set
 *           [20:16] / [28:24] long-term counts (0 in all three proven clips)
 *   +0x0cc  (idx_width << 4) | NumPicTotalCurr, idx_width per the blob's own
 *           ceil-log2 loop -- note count 1 yields width 0, not 1
 *   +0x0f4  0xffff0015 | (nal_unit_type << 8)
 *
 * VALIDATED off-board against THREE proven vendor mmiotraces, every picture,
 * every one of the five words -- 200 of 200 predicted, no free parameter:
 *
 *   MERGE_A_TI_3   416x240 CtbSizeY 64   8 pics  B slices, MaxNumMergeCand 0/1,
 *                  collocated_from_l0 both ways, mvd_l1_zero both ways
 *   MAXBINS_A_TI_5 416x240 CtbSizeY 16   2 pics  I-only, QP 4, pcm enabled
 *   DBLK_A_SONY_3  832x480 CtbSizeY 64  30 pics  P+B, slice_beta/tc_offset_div2
 *                  swept the full signed -6..+6 range, sao off, loop-filter-
 *                  across-slices off, MaxNumMergeCand 5 -- i.e. it flips every
 *                  field the other two hold constant
 *
 * All three decodes are bit-exact vs their JCT references
 * (3a15886a.. / 1386fb14.. / 42486a48..), so the traces are usable.
 *
 * TWO fields are derived but NOT obtainable from the V4L2 HEVC uAPI:
 *   * +0x0c8 [13:8]. The SPS control does not carry the st_ref_pic_set
 *     contents. Emitted as 0. This is exact for MAXBINS-class streams, and it
 *     is a MEASURED don't-care for an IDR: MAXBINS picture 0 writes 0 while
 *     MERGE and DBLK_A picture 0 write 1, on the same picture class (I slice,
 *     NumPicTotalCurr 0), and all three decodes are bit-exact.
 *
 *     N35 -- it is a GENUINE uAPI GAP, not a derivation we skipped. The value
 *     is a property of the LAST explicitly-coded SPS st_ref_pic_set, and the
 *     driver never sees the SPS RBSP: for stateless HEVC the OUTPUT buffer
 *     carries slice NALs only and the SPS arrives as
 *     V4L2_CID_STATELESS_HEVC_SPS, which has no st_ref_pic_set member. The one
 *     case it IS derivable is num_short_term_ref_pic_sets == 0, which forces 0
 *     -- exactly what we already emit. MEASURED across all 40 pictures of the
 *     three proven traces: [13:8] is CONSTANT per clip and independent of
 *     picture type (MERGE 1 on all 8, DBLK 1 on all 30, MAXBINS 0 on both),
 *     while [4:0] -- the field the uAPI DOES supply, via
 *     num_delta_pocs_of_ref_rps_idx -- is the part that varies per picture
 *     (MERGE 0,1,2,4,4,4,4,4). So the gap is one constant bit-field per
 *     SEQUENCE, and it is NOT specific to non-IDR pictures.
 *   * +0x0b8 bit2, pps_scaling_list_data_present_flag (see above).
 *
 * For an I slice vpud leaves num_ref_idx_l0/l1_active, five_minus_max_num_-
 * merge_cand, cabac_init and mvd_l1_zero at their struct defaults rather than
 * at whatever V4L2 reports, which is why they are forced here: that is what
 * produces the 0x280000ff seen on all three IDRs.
 */
struct hevc_shdr_words {
	u32 c0, c4, c8, cc, f4;
	bool valid;
};

static u32 hevc_totalcurr_width(u32 n)
{
	/* 0x67cc4..0x67cf0: the w2<=1 branch enters the loop with w1 = 0 */
	u32 w, t;

	if (n <= 1) {
		w = 0;
		t = n >> 1;
	} else {
		w = 1;
		t = (n - 1) >> 1;
	}
	while (t) {
		w++;
		t >>= 1;
	}
	return w;
}

static void hevc_pack_shdr(struct mtk_vcodec_dec_ctx *ctx,
			   struct hevc_shdr_words *w)
{
	const struct v4l2_ctrl_hevc_slice_params *sl;
	const struct v4l2_ctrl_hevc_decode_params *dp;
	const struct v4l2_ctrl_hevc_sps *s;
	const struct v4l2_ctrl_hevc_pps *p;
	struct v4l2_ctrl *cs, *cp, *csl, *cdp;
	u32 l0, l1, merge, ntc, sao;
	bool intra;

	memset(w, 0, sizeof(*w));
	cs = v4l2_ctrl_find(&ctx->ctrl_hdl, V4L2_CID_STATELESS_HEVC_SPS);
	cp = v4l2_ctrl_find(&ctx->ctrl_hdl, V4L2_CID_STATELESS_HEVC_PPS);
	csl = v4l2_ctrl_find(&ctx->ctrl_hdl, V4L2_CID_STATELESS_HEVC_SLICE_PARAMS);
	cdp = v4l2_ctrl_find(&ctx->ctrl_hdl, V4L2_CID_STATELESS_HEVC_DECODE_PARAMS);
	if (!cs || !cp || !csl || !cdp)
		return;
	if (!cs->p_cur.p || !cp->p_cur.p || !csl->p_cur.p || !cdp->p_cur.p)
		return;
	s = cs->p_cur.p;
	p = cp->p_cur.p;
	sl = csl->p_cur.p;		/* first slice segment of the picture */
	dp = cdp->p_cur.p;

	intra = (sl->slice_type == V4L2_HEVC_SLICE_TYPE_I);
	sao = !!(s->flags & V4L2_HEVC_SPS_FLAG_SAMPLE_ADAPTIVE_OFFSET);

	/* vpud's struct defaults for a slice type that does not code these */
	l0 = intra ? 0 : (u32)sl->num_ref_idx_l0_active_minus1 + 1;
	l1 = (sl->slice_type == V4L2_HEVC_SLICE_TYPE_B) ?
	     (u32)sl->num_ref_idx_l1_active_minus1 + 1 : 0;
	merge = intra ? 0 : (5 - (sl->five_minus_max_num_merge_cand & 7));
	ntc = (u32)dp->num_poc_st_curr_before + dp->num_poc_st_curr_after +
	      dp->num_poc_lt_curr;

	w->c0  = (sao && (sl->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_SAO_LUMA)) << 5;
	w->c0 |= (sao && (sl->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_SAO_CHROMA)) << 6;
	if (!intra)
		w->c0 |= !!(sl->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_CABAC_INIT) << 7;
	w->c0 |= (((u32)((int)p->init_qp_minus26 + 26 + sl->slice_qp_delta)) & 0x7f) << 8;
	w->c0 |= (((u32)(p->pps_cb_qp_offset + sl->slice_cb_qp_offset)) & 0x1f) << 16;
	w->c0 |= (((u32)(p->pps_cr_qp_offset + sl->slice_cr_qp_offset)) & 0x1f) << 24;

	w->c4  = (l0 - 1) & 0xf;
	w->c4 |= ((l1 - 1) & 0xf) << 4;
	w->c4 |= ((u32)sl->collocated_ref_idx & 0xf) << 8;
	w->c4 |= (merge & 7) << 12;
	w->c4 |= ((u32)sl->slice_beta_offset_div2 & 0x1f) << 16;
	w->c4 |= ((u32)sl->slice_tc_offset_div2 & 0x1f) << 21;
	w->c4 |= !!(sl->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_TEMPORAL_MVP_ENABLED) << 26;
	/* not coded outside a B slice; vpud's default is 1 */
	w->c4 |= (sl->slice_type == V4L2_HEVC_SLICE_TYPE_B ?
		  !!(sl->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_COLLOCATED_FROM_L0) : 1) << 27;
	if (sl->slice_type == V4L2_HEVC_SLICE_TYPE_B)
		w->c4 |= !!(sl->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_MVD_L1_ZERO) << 28;
	w->c4 |= !!(sl->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_LOOP_FILTER_ACROSS_SLICES_ENABLED) << 29;
	w->c4 |= !!(sl->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_DEBLOCKING_FILTER_DISABLED) << 30;

	/* [13:8] is the one field the uAPI cannot supply -- see the block above */
	w->c8 = (u32)dp->num_delta_pocs_of_ref_rps_idx & 0x1f;

	w->cc = ((hevc_totalcurr_width(ntc) & 7) << 4) | (ntc & 0xf);

	w->f4 = 0xffff0015u | (((u32)sl->nal_unit_type & 0xff) << 8);

	w->valid = true;
}

/*
 * The per-picture program (trace rows 152..224). The geometry words and the
 * H265_SPS_A..G words are both DERIVED (see the CAPTURE GEOMETRY block at the
 * top of this file and the SPS/PPS PACKING block above).
 */
static void hevc_program_picture(struct vdec_hevc_mt8365_inst *inst,
				 struct mtk_vcodec_mem *bs, u32 rec_y,
				 u32 rec_c, u32 colmv, u32 pic_w, u32 pic_h,
				 u32 bs_len)
{
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	u32 b = inst->ctb_size;
	u32 aw = ALIGN(pic_w, b);
	u32 ah = ALIGN(pic_h, b);
	u32 ctb_cols = aw / b;
	u32 ctb_rows = ah / b;
	struct hevc_spspps_words w;
	struct hevc_shdr_words sh;
	u32 tab0, tab20;
	int i;

	hevc_pack_spspps(ctx, &w);

	wr(ctx, VDEC_HWG, H265_CFG0E0, 0xfffef7fb);
	wr(ctx, VDEC_HWG, H265_CFG0D0, bs_len);
	wr(ctx, VDEC_HWG, H265_PIC_START, 0x80000000);
	wr(ctx, VDEC_HWG, H265_CFG0D4, 0x04011d01);
	wr(ctx, VDEC_PP, PP_CFG040, 0x0000f011);
	wr(ctx, VDEC_AV, MV_CFG210, 0x0);
	wr(ctx, VDEC_AV, MV_CFG320, 0x0);
	wr(ctx, VDEC_AV, MV_CUR_COLMV, colmv >> 4);
	wr(ctx, VDEC_CM, MC_CFGB5C, 0x1);
	wr(ctx, VDEC_CM, MC_CFGAF8, 0x1100);
	wr(ctx, VDEC_CM, MC_PIC_W_PX, pic_w);
	wr(ctx, VDEC_CM, MC_PIC_H_PX, pic_h);
	wr(ctx, VDEC_CM, MC_STRIDE64, ALIGN(pic_w, 64) / 16);
	wr(ctx, VDEC_CM, MC_ENABLE220, 0x1);
	wr(ctx, VDEC_CM, MC_RECON_Y, rec_y >> 9);
	wr(ctx, VDEC_CM, MC_RECON_C, rec_c >> 8);
	wr(ctx, VDEC_CM, MC_RECON_STRIDE_MB, aw / 16);
	wr(ctx, VDEC_CM, MC_INTER_EN, 0x3);
	wr(ctx, VDEC_CM, MC_ENABLE250, 0x1);
	wr(ctx, VDEC_CM, MC_MB_W_M1, aw / 16 - 1);
	wr(ctx, VDEC_CM, MC_MB_H_M1, ah / 16 - 1);
	wr(ctx, VDEC_PP, PP_CFG630, 0x0);
	/*
	 * The slice-header block, at the vendor's own position in the sequence:
	 * writes 174..178 of picture 0, immediately after PP +0x630 and
	 * immediately before TOP +0x068. With these five landed, a
	 * picture-0 write-set diff against the vendor trace (segmented on the
	 * KICK) has NO vendor-only and NO ours-only offsets.
	 */
	hevc_pack_shdr(ctx, &sh);

	/* N44 -- the in-loop-filter arm (see the module-param block). */
	hevc_nolf_cand++;
	if (unlikely(mt8365_hevc_nolf)) {
		u32 c0_in = sh.c0, c4_in = sh.c4;
		u32 a0_in = w.a0, b4_in = w.b4, b8_in = w.b8;

		if (mt8365_hevc_nolf & 1) {
			sh.c4 |= BIT(30);	/* slice_deblocking_disabled */
			w.b8 &= ~(u32)BIT(0);	/* PPS deblock enable */
		}
		if (mt8365_hevc_nolf & 2) {
			sh.c0 &= ~(u32)(BIT(5) | BIT(6));
			w.a0 &= ~(u32)BIT(5);	/* SPS SAO enable */
		}
		if (mt8365_hevc_nolf & 4) {
			sh.c4 &= ~(u32)BIT(29);
			w.b4 &= ~(u32)BIT(24);
		}
		hevc_nolf_appl++;
		if (hevc_nolf_appl <= 2)
			pr_info("mt8365 hevc: N44 NOLF=%d c0 %08x->%08x c4 %08x->%08x a0 %08x->%08x b4 %08x->%08x b8 %08x->%08x\n",
				mt8365_hevc_nolf, c0_in, sh.c0, c4_in, sh.c4,
				a0_in, w.a0, b4_in, w.b4, b8_in, w.b8);
	}

	if (sh.valid) {
		wr(ctx, VDEC_HWG, H265_SHDR_A, sh.c0);
		wr(ctx, VDEC_HWG, H265_SHDR_B, sh.c4);
		wr(ctx, VDEC_HWG, H265_SHDR_C, sh.c8);
		wr(ctx, VDEC_HWG, H265_SHDR_D, sh.cc);
		wr(ctx, VDEC_HWG, H265_SHDR_E, sh.f4);
	} else {
		pr_warn("mt8365 hevc: slice-header controls absent -- block not programmed\n");
	}
	wr(ctx, VDEC_TOP, TOP_CTB_M1, ((ctb_rows - 1) << 16) | (ctb_cols - 1));
	wr(ctx, VDEC_TOP, TOP_PICSIZE_PX, (pic_h << 16) | pic_w);
	wr(ctx, VDEC_TOP, TOP_CTB_DIM, (ctb_rows << 16) | ctb_cols);
	wr(ctx, VDEC_TOP, TOP_CFG100, 0x0);
	if (w.valid) {
		wr(ctx, VDEC_HWG, H265_SPS_A, w.a0);
		wr(ctx, VDEC_HWG, H265_SPS_B, w.a4);
		wr(ctx, VDEC_HWG, H265_SPS_C, w.a8);
		wr(ctx, VDEC_HWG, H265_SPS_D, w.ac);
		wr(ctx, VDEC_HWG, H265_SPS_E, w.b0);
		wr(ctx, VDEC_HWG, H265_SPS_F, w.b4);
		wr(ctx, VDEC_HWG, H265_SPS_G, w.b8);
	} else {
		/* controls absent -- MERGE_A_TI_3 literals, so the path is
		 * still exercisable, but flagged loudly.
		 */
		pr_warn("mt8365 hevc: SPS/PPS controls absent -- falling back to clip literals\n");
		wr(ctx, VDEC_HWG, H265_SPS_A, 0x0b0077b1);
		wr(ctx, VDEC_HWG, H265_SPS_B, 0x00011408);
		wr(ctx, VDEC_HWG, H265_SPS_C, 0x22575263);
		wr(ctx, VDEC_HWG, H265_SPS_D, 0x0211000b);
		wr(ctx, VDEC_HWG, H265_SPS_E, 0x00001a60);
		wr(ctx, VDEC_HWG, H265_SPS_F, 0x01000000);
		wr(ctx, VDEC_HWG, H265_SPS_G, 0x00000001);
	}
	/* N35 -- see the mt8365_hevc_ctbtab block. */
	tab0 = mt8365_hevc_ctbtab ? (1u << (ctb_cols - 1)) : 0x40;
	tab20 = mt8365_hevc_ctbtab ? (1u << (ctb_rows - 1)) : 0x8;
	for (i = 0; i < 16; i++)
		wr(ctx, VDEC_HWG, H265_REFTAB + 4 * i, i == 0 ? tab0 : 0x0);
	wr(ctx, VDEC_HWG, H265_CFG1C0, 0x0);
	for (i = 0; i < 16; i++)
		wr(ctx, VDEC_HWG, H265_REFTAB2 + 4 * i, i == 0 ? tab20 : 0x0);
	wr(ctx, VDEC_HWG, H265_CFG1D0, 0x0);
	/*
	 * N15 recon write-DMA window (see the module-param block). Programmed
	 * LAST, immediately before the kick, which is where the VP8 backend
	 * writes it. The luma extent is the CAPTURE plane the driver actually
	 * declared, not the coded ALIGN(w,CtbSizeY) x ALIGN(h,CtbSizeY) box:
	 * the window has to cover what the DMA may touch inside the buffer.
	 */
	if (mt8365_hevc_reconwin) {
		u32 luma_sz = ctx->picinfo.buf_w * ctx->picinfo.buf_h;
		u32 wlo = min(rec_y, rec_c);
		u32 whi = max(rec_y + luma_sz, rec_c + luma_sz / 2);

		wr(ctx, VDEC_CM, MC_RECON_BASE, wlo >> 4);
		wr(ctx, VDEC_CM, MC_RECON_LIMIT, whi >> 4);
		wr(ctx, VDEC_CM, MC_RECON_EN, 0x1);
	}
	/*
	 * N21 -- force the vendor's pre-kick state (see the module-param block).
	 * Counted UNCONDITIONALLY so the denominator is readable with the arm
	 * off; the applied count is the execution proof.
	 */
	hevc_st_cand++;
	if (mt8365_hevc_state) {
		if (mt8365_hevc_state & 0x1)
			wr(ctx, VDEC_HWG, H265_CFG084,
			   rd(ctx, VDEC_HWG, H265_CFG084) | 0x800);
		if (mt8365_hevc_state & 0x2)
			wr(ctx, VDEC_TOP, 0x144, 0x105);
		if (mt8365_hevc_state & 0x4)
			wr(ctx, VDEC_TOP, 0x224, 0x005);
		if (mt8365_hevc_state & 0x8)
			wr(ctx, VDEC_HWG, 0x098, 0x0);
		hevc_st_appl++;
	}

	/* N21: the whole-block state the kick is about to land on. */
	if (unlikely(mt8365_hevc_regdump) && (mt8365_hevc_regdump_pt & 2) &&
	    hevc_cand == 1)
		hevc_regdump(ctx, "PREKICK");
	/*
	 * N31 CRC arm. Deliberately the LAST write before the kick: the reset
	 * prologue sets MISC+0x04 = 0x100 on every decode, so an earlier arm
	 * would be overwritten.
	 */
	hevc_crc_cand++;
	if (unlikely(mt8365_hevc_crc)) {
		u32 v = rd(ctx, VDEC_MISC, MISC_CFG004);

		wr(ctx, VDEC_MISC, MISC_CFG004,
		   (v & ~3u) | ((u32)mt8365_hevc_crc & 3u));
		hevc_crc_appl++;
		if (hevc_crc_appl <= 2)
			pr_info("mt8365 hevc CRCARM MISC+04 %08x -> %08x (SYS+00=%08x)\n",
				v, rd(ctx, VDEC_MISC, MISC_CFG004),
				rd(ctx, VDEC_SYS, SYS_CTRL));
	}

	if (!mt8365_hevc_nokick)
		wr(ctx, VDEC_HWG, H265_KICK, 0x1);
}

static int vdec_hevc_mt8365_decode(void *h_vdec, struct mtk_vcodec_mem *bs,
				   struct vdec_fb *unused, bool *res_chg)
{
	struct vdec_hevc_mt8365_inst *inst = h_vdec;
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	struct vdec_fb *fb;
	u32 rec_y, rec_c, colmv, pic_w, pic_h, bs_start, bs_end, bs_len;
	u32 cfg0bc = 0xa0500000;	/* N35: MERGE literal unless armed */
	u32 spins = 0, done = 0;
	u32 pre[ARRAY_SIZE(hevc_probes)], post[ARRAY_SIZE(hevc_probes)];
	int ret;

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
	 * Counted UNCONDITIONALLY so the denominator is observable with the arm
	 * off -- an execution instrument, not an inference from pixels.
	 */
	hevc_cand++;

	inst->ctb_size = hevc_ctb_size(ctx);
	pic_w = ctx->picinfo.pic_w;
	pic_h = ctx->picinfo.pic_h;
	if (!pic_w || !pic_h)
		return -EINVAL;

	if (!mt8365_hevc_arm) {
		mtk_v4l2_vdec_err(ctx,
			"mt8365 hevc: backend is a stub (mt8365_hevc_arm=0); %ux%u CtbSizeY=%u, no register touched",
			pic_w, pic_h, inst->ctb_size);
		return -EIO;
	}

	if (!inst->work_buf_ready) {
		ret = alloc_work_bufs(inst);
		if (ret)
			return ret;
	}

	/*
	 * N19 -- the vendor's end-of-bitstream sentinel (see the module-param
	 * block). Written BEFORE the DMA sync so the five bytes are visible to
	 * the engine, and the sync is widened to cover them.
	 */
	{
		struct mtk_video_dec_buf *sbi =
			container_of(bs, struct mtk_video_dec_buf, bs_buffer);
		struct vb2_buffer *svb = &sbi->m2m_buf.vb.vb2_buf;
		size_t cap = vb2_plane_size(svb, 0);
		size_t sync = bs->size;
		bool uncached_plane = svb->memory == V4L2_MEMORY_MMAP &&
				      !svb->vb2_queue->non_coherent_mem;

		hevc_eos_cand++;
		if (mt8365_hevc_eosmark && cap >= bs->size + 5) {
			static const u8 eos[5] = { 0x00, 0x00, 0x00, 0x01, 0xff };

			/*
			 * N45: this write lands AFTER vb2's own buf_prepare
			 * cache handling, so it is only safe on a plane whose
			 * kernel mapping needs no cache maintenance -- i.e. a
			 * coherent MMAP plane, which dma_pgprot() maps uncached
			 * on this non-coherent device. On a cached plane there
			 * is no legal way to flush five bytes from here (the
			 * scatter-aware sgtable belongs to vb2, not to us), so
			 * the sentinel is skipped and COUNTED rather than left
			 * dirty in cache where the engine would never see it.
			 */
			if (uncached_plane) {
				memcpy((u8 *)bs->va + bs->size, eos, sizeof(eos));
				sync = bs->size + sizeof(eos);
				hevc_eos_appl++;
				if (hevc_eos_appl <= 2)
					pr_info("mt8365 hevc: N19 sentinel written at bs+0x%zx (plane cap %zu, bytesused %zu)\n",
						bs->size, cap, bs->size);
			} else {
				hevc_eos_incoh++;
			}
		}
		/*
		 * N45: audit + write barrier. The
		 * dma_sync_single_for_device(bs->dma_addr, sync) that used to
		 * stand here was an illegal streaming sync on an
		 * IOMMU-scattered vb2 handle: it translated only the first page
		 * and then ran cache maintenance across `sync` bytes of
		 * physically contiguous memory that, past that page, was not
		 * the buffer -- and once in 415 armed decodes walked into the
		 * board's no-map secmon carve-out and Oopsed in
		 * dcache_clean_poc(). See mtk_vcodec_mt8365_bs_audit().
		 */
		mtk_vcodec_mt8365_bs_audit(ctx, bs->dma_addr, sync);

		/*
		 * N39 -- the bitstream CONTENT instrument (read-only; see the
		 * module-param block). Runs after the sync so it reports exactly
		 * the bytes the engine will fetch.
		 */
		hevc_bs_cand++;
		if (mt8365_hevc_bsdump && hevc_bs_appl < mt8365_hevc_bsdump) {
			const u8 *p = bs->va;
			size_t i, n = bs->size;
			u32 epb = 0;
			u32 crc = crc32_le(~0U, p, n) ^ ~0U;

			for (i = 2; i < n; i++)
				if (p[i] == 3 && p[i - 1] == 0 && p[i - 2] == 0)
					epb++;

			hevc_bs_appl++;
			pr_info("mt8365 hevc: N39 bs#%d bytesused=%zu cap=%zu dma=0x%08llx align16=%u crc32=0x%08x epb=%u\n",
				hevc_bs_appl, n, cap,
				(unsigned long long)bs->dma_addr,
				(unsigned)(bs->dma_addr & 0xf), crc, epb);
			print_hex_dump(KERN_INFO, "mt8365 hevc: N39 head ",
				       DUMP_PREFIX_OFFSET, 16, 1, p,
				       min_t(size_t, n, 48), false);
			if (n > 32)
				print_hex_dump(KERN_INFO, "mt8365 hevc: N39 tail ",
					       DUMP_PREFIX_OFFSET, 16, 1,
					       p + n - 32, 32, false);
		}
	}

	rec_y = (u32)fb->base_y.dma_addr;
	rec_c = (u32)fb->base_c.dma_addr;
	colmv = (u32)inst->colmv_buf.dma_addr;

	/*
	 * N41 -- pre-fill the CAPTURE planes so "still zero afterwards" and
	 * "never written" stop being the same observation. See the module-param
	 * block. Default off; the counter advances either way.
	 */
	hevc_poison_cand++;
	if (unlikely(mt8365_hevc_poison > 0 && mt8365_hevc_poison < 256)) {
		u8 pb = (u8)mt8365_hevc_poison;

		if (fb->base_y.va && fb->base_y.size)
			memset(fb->base_y.va, pb, fb->base_y.size);
		if (fb->base_c.va && fb->base_c.size)
			memset(fb->base_c.va, pb, fb->base_c.size);
		wmb(); /* poison visible to the engine before the kick */
		hevc_poison_appl++;
		if (hevc_poison_appl <= 4)
			pr_info("mt8365 hevc: N41 POISON pic %u byte=0x%02x y_va=%p y_sz=%zu c_va=%p c_sz=%zu rec_y=0x%08x rec_c=0x%08x\n",
				inst->frm_cnt, pb, fb->base_y.va,
				fb->base_y.size, fb->base_c.va,
				fb->base_c.size, rec_y, rec_c);
	}
	bs_start = (u32)bs->dma_addr;

	/*
	 * N7 -- the bitstream extent, DERIVED from both proven traces.
	 *
	 * VLD+0x0b0 is the READ POINTER and points at the VCL NAL's START CODE
	 * (vendor pic0: buffer base 0xfe800000 + 0x5c, exactly the IDR start
	 * code offset in the AU; pics 1..7 start at offset 0). HEVCVLD+0x0d0 is
	 * the extent measured from the NAL PAYLOAD (i.e. past the start code)
	 * plus a fixed 10-byte tail:
	 *
	 *   +0x0d0 = (au_end - vcl_payload_start) + 9 + slices_in_au
	 *
	 * N35: settled over ALL 40 captured pictures of ALL THREE proven clips,
	 * no free parameter -- MERGE_A_TI_3 8/8 (1 slice/AU, +10), MAXBINS_A_TI_5
	 * 2/2 (1 slice/AU, +10), DBLK_A_SONY_3 30/30 (4 slices/AU, +13).
	 * "au_end" is the first byte of the NEXT access unit (or EOF), so it
	 * INCLUDES every trailing suffix NAL of this AU.
	 *
	 * MERGE carries a 57-byte suffix SEI after every slice and MAXBINS
	 * carries none, which is what separates the constant from the SEI: on
	 * MERGE the vendor's value is exactly ours + 57 (0x23c5 vs 0x238c on
	 * picture 0), and on MAXBINS the two are IDENTICAL (0x34ca1 / 0x37354,
	 * MEASURED both sides). The residual on MERGE is therefore NOT a driver
	 * defect -- the suffix SEI is not in the V4L2 OUTPUT buffer at all, so
	 * the extent we can legally program stops where our data stops.
	 *
	 * The window END is then
	 *   end = ALIGN(rp_off + len, 64) + 448
	 * (7 x 64B of prefetch margin past the data) -- 10/10 exact.
	 *
	 * Our OUTPUT buffer holds one slice NAL with its start code and no
	 * suffix NALs, so rp_off = 0 and len = bytesused - sc_len + 10.
	 */
	{
		const u8 *p = bs->va;
		u32 sc = 0;

		if (bs->size >= 4 && p[0] == 0 && p[1] == 0) {
			if (p[2] == 1)
				sc = 3;
			else if (p[2] == 0 && p[3] == 1)
				sc = 4;
		}
		bs_len = (u32)bs->size - sc + 10;
		bs_end = bs_start + ALIGN(bs_len, 64) + 448;
	}

	/*
	 * Keyed on the CANDIDATE counter, not on frm_cnt: frm_cnt only advances
	 * on a completed picture, so while the kick times out frm_cnt is stuck
	 * at 0 and the instrument would re-print picture 0 forever.
	 */
	if (hevc_cand <= 2) {
		struct hevc_spspps_words dbg;
		struct hevc_shdr_words dsh;

		hevc_pack_spspps(ctx, &dbg);
		hevc_pack_shdr(ctx, &dsh);
		pr_info("mt8365 hevc: call%d pic%u win bs=0x%08x..0x%08x b4=0x%08x b8=0x%08x len=%u(0x%x) sz=%zu ctb=%u | valid=%d a0=%08x a4=%08x a8=%08x ac=%08x b0=%08x b4w=%08x b8w=%08x\n",
			hevc_cand, inst->frm_cnt, bs_start, bs_end, bs_start >> 6,
			bs_end >> 6, bs_len, bs_len, bs->size, inst->ctb_size,
			dbg.valid, dbg.a0, dbg.a4, dbg.a8, dbg.ac, dbg.b0,
			dbg.b4, dbg.b8);
		pr_info("mt8365 hevc: call%d pic%u shdr valid=%d c0=%08x c4=%08x c8=%08x cc=%08x f4=%08x\n",
			hevc_cand, inst->frm_cnt, dsh.valid, dsh.c0, dsh.c4, dsh.c8,
			dsh.cc, dsh.f4);
		/*
		 * The CAPTURE geometry the driver actually declared, beside the
		 * addresses the register program will carry -- so a recon
		 * write-out question is answered from the log, not inferred.
		 */
		pr_info("mt8365 hevc: call%d GEOM pic=%ux%u ctb=%u buf=%ux%u fb_sz=%u/%u | rec_y=0x%08x rec_c=0x%08x colmv=0x%08x delta_yc=0x%x reconwin=%d\n",
			hevc_cand, pic_w, pic_h, inst->ctb_size,
			ctx->picinfo.buf_w, ctx->picinfo.buf_h,
			ctx->picinfo.fb_sz[0], ctx->picinfo.fb_sz[1],
			rec_y, rec_c, colmv, rec_y - rec_c, mt8365_hevc_reconwin);
	}

	/* N21: the state BEFORE our own reset/window touches anything -- i.e.
	 * whatever the previous owner of the block (our probe, or a vendor
	 * window) left behind.
	 */
	if (unlikely(mt8365_hevc_regdump) && (mt8365_hevc_regdump_pt & 1) &&
	    hevc_cand == 1)
		hevc_regdump(ctx, "PREPROG");

	/*
	 * N35 -- HEVCVLD +0x0bc [23:20]. Computed unconditionally so the
	 * candidate counter is a real instrument; applied only when armed.
	 */
	{
		u32 b = inst->ctb_size;
		u32 cols = ALIGN(pic_w, b) / b;
		u32 rows = ALIGN(pic_h, b) / b;
		u32 derived = 0xa0000000u |
			      ((hevc_ceil_log2(cols * rows) & 0xf) << 20);

		hevc_ctbtab_cand++;
		if (mt8365_hevc_ctbtab) {
			cfg0bc = derived;
			hevc_ctbtab_appl++;
		}
		if (hevc_cand <= 2)
			pr_info("mt8365 hevc: call%d CTBTAB ctb=%ux%u tab0=0x%08x tab20=0x%08x cfg0bc=0x%08x (derived 0x%08x) arm=%d\n",
				hevc_cand, cols, rows, 1u << (cols - 1),
				1u << (rows - 1), cfg0bc, derived,
				mt8365_hevc_ctbtab);
	}
	hevc_reset_and_window(ctx, bs_start, bs_end, cfg0bc);
	hevc_program_reftab(ctx, rec_y, rec_c, colmv);

	if (unlikely(mt8365_hevc_probe)) {
		hevc_probe_snap(ctx, pre);
		if (hevc_appl == 0)
			hevc_probe_dump("PRE", pre);
	}

	hevc_program_picture(inst, bs, rec_y, rec_c, colmv, pic_w, pic_h, bs_len);
	hevc_appl++;

	/*
	 * POSITIVE CONTROL: everything programmed, nothing kicked. Returning 0
	 * hands the untouched CAPTURE buffer down the pipeline so its contents
	 * can be histogrammed against a real decode.
	 */
	if (unlikely(mt8365_hevc_nokick)) {
		mtk_v4l2_vdec_err(ctx,
			"mt8365 hevc: picture %u NOT KICKED (positive control)",
			inst->frm_cnt);
		inst->frm_cnt++;
		return 0;
	}

	/*
	 * Completion. The vendor sleeps on the VDEC dec-done IRQ (WaitISR ->
	 * eHalCmdProc(0)); the core ISR here reads VDEC_MISC +0xa4, requires
	 * bit16 (MTK_VDEC_IRQ_STATUS_DEC_SUCCESS), ACKs and wakes us. That is
	 * the same plumbing vdec_h264_mt8365_if.c already uses, and the reason
	 * a poll can never work: the ISR clears the status before the decode
	 * thread can observe it.
	 */
	if (mt8365_hevc_done_src == 0) {
		ret = mtk_vcodec_wait_for_done_ctx(ctx, MTK_INST_IRQ_RECEIVED,
						   MT8365_HEVC_IRQ_TIMEOUT_MS,
						   MTK_VDEC_CORE);
	} else {
		u32 mask = (mt8365_hevc_done_src == 1) ? 0 : 0x80000000;
		u32 reg = (mt8365_hevc_done_src == 1) ? H265_DONE
						      : H265_PIC_START;

		ret = -ETIMEDOUT;
		while (spins < MT8365_HEVC_TIMEOUT) {
			done = rd(ctx, VDEC_HWG, reg);
			if ((done & 0x80000000) != mask) {
				ret = 0;
				break;
			}
			spins++;
			udelay(1);
		}
	}

	if (unlikely(mt8365_hevc_probe)) {
		hevc_probe_snap(ctx, post);
		hevc_probe_report(ctx, ret ? "TIMEOUT" : "DONE", pre, post);
	}

	if (ret) {
		mtk_v4l2_vdec_err(ctx,
			"mt8365 hevc: decode TIMEOUT (src=%d) spins=%u irq_status=%08x | +0x278=%08x +0x2ac=%08x +0x12c=%08x +0x090=%08x MISC+0xa4=%08x +0x284=%08x",
			mt8365_hevc_done_src, spins, ctx->irq_status,
			rd(ctx, VDEC_HWG, H265_DONE),
			rd(ctx, VDEC_HWG, H265_PIC_START),
			rd(ctx, VDEC_HWG, H265_LCU_XY),
			rd(ctx, VDEC_HWG, H265_KICK),
			rd(ctx, VDEC_MISC, 0xa4),
			rd(ctx, VDEC_HWG, H265_ERR_CODE));
		return ret;
	}

	if (mt8365_hevc_done_src == 0 &&
	    (ctx->irq_status & MTK_VDEC_IRQ_STATUS_DEC_SUCCESS) !=
	    MTK_VDEC_IRQ_STATUS_DEC_SUCCESS) {
		mtk_v4l2_vdec_err(ctx,
			"mt8365 hevc: picture %u IRQ without DEC_SUCCESS, irq_status=%08x",
			inst->frm_cnt, ctx->irq_status);
		return -EIO;
	}

	/*
	 * Step 10 -- POSTKICK dump point. The same window read at PREPROG and
	 * at PREKICK gave different verdicts on whether the read perturbs the
	 * decode, so the third point (after the picture has completed and the
	 * engine is idle again) is needed to say whether the perturbation is a
	 * property of the read or of WHEN the read happens.
	 */
	if (unlikely(mt8365_hevc_regdump) && (mt8365_hevc_regdump_pt & 4) &&
	    hevc_cand == 1)
		hevc_regdump(ctx, "POSTKICK");

	/*
	 * N31 -- the vendor's post-decode read block, replayed in its own
	 * order: GetErrMsg (HEVCVLD +0xe0 mask / +0xe4,+0xe8 status), the four
	 * status words at +0x120..+0x12c, then the 8-word digest in both banks.
	 * Reading it unconditionally when the instrument is armed, including
	 * when the arm value is 0, is what makes a null result interpretable.
	 */
	if (unlikely(mt8365_hevc_crc >= 0) && inst->frm_cnt < (u32)hevc_crc_lines) {
		u32 c[8], u[8], s[8];
		int i;

		for (i = 0; i < 8; i++) {
			c[i] = rd(ctx, VDEC_MISC, 0x08 + 4 * i);
			u[i] = rd(ctx, VDEC_PP, 0xfa0 + 4 * i);
			/* N41: the corrected UFO window -- SYS +0x8a0.. */
			s[i] = rd(ctx, VDEC_SYS, HEVC_UFO_IN_SYS + 0xa0 + 4 * i);
		}
		pr_info("mt8365 hevc N41 UFO@SYS+8a0..8bc pic %u %08x %08x %08x %08x %08x %08x %08x %08x | SYS+80c=%08x MC+b5c=%08x PP+b80=%08x\n",
			inst->frm_cnt, s[0], s[1], s[2], s[3], s[4], s[5],
			s[6], s[7], rd(ctx, VDEC_SYS, SYS_GATE80C),
			rd(ctx, VDEC_CM, MC_CFGB5C), rd(ctx, VDEC_PP, PP_CFGB80));
		pr_info("mt8365 hevc CRC pic %u arm=%d MISC+04=%08x SYS+00=%08x MISC+00=%08x | MISC+08..24 %08x %08x %08x %08x %08x %08x %08x %08x | UFO+a0..bc %08x %08x %08x %08x %08x %08x %08x %08x | HEVCVLD e0/e4/e8 %08x %08x %08x | 120/124/128/12c %08x %08x %08x %08x\n",
			inst->frm_cnt, mt8365_hevc_crc,
			rd(ctx, VDEC_MISC, MISC_CFG004),
			rd(ctx, VDEC_SYS, SYS_CTRL),
			rd(ctx, VDEC_MISC, 0x00),
			c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7],
			u[0], u[1], u[2], u[3], u[4], u[5], u[6], u[7],
			rd(ctx, VDEC_HWG, 0x0e0), rd(ctx, VDEC_HWG, 0x0e4),
			rd(ctx, VDEC_HWG, 0x0e8),
			rd(ctx, VDEC_HWG, 0x120), rd(ctx, VDEC_HWG, 0x124),
			rd(ctx, VDEC_HWG, 0x128), rd(ctx, VDEC_HWG, 0x12c));
	}

	/*
	 * N41 -- the recon write-out verdict. Scans the two CAPTURE planes the
	 * engine was pointed at and classifies every byte against the poison.
	 */
	if (unlikely(mt8365_hevc_poison > 0 && mt8365_hevc_poison < 256) &&
	    inst->frm_cnt < (u32)hevc_crc_lines) {
		u8 pb = (u8)mt8365_hevc_poison;
		int p;

		rmb(); /* observe whatever the engine wrote */
		for (p = 0; p < 2; p++) {
			const u8 *va = p ? fb->base_c.va : fb->base_y.va;
			size_t sz = p ? fb->base_c.size : fb->base_y.size;
			size_t i, npo = 0, nz = 0, noth = 0;
			long first = -1;
			u8 h[16];

			if (!va || !sz)
				continue;
			for (i = 0; i < sz; i++) {
				u8 v = va[i];

				if (v == pb) {
					npo++;
				} else {
					if (first < 0)
						first = (long)i;
					if (v == 0)
						nz++;
					else
						noth++;
				}
			}
			if (first >= 0)
				memcpy(h, va + first,
				       min_t(size_t, 16, sz - (size_t)first));
			else
				memset(h, 0, sizeof(h));
			pr_info("mt8365 hevc N41 SCAN pic %u plane=%c sz=%zu poison=%zu zero=%zu other=%zu first_diff=%ld head=%*ph\n",
				inst->frm_cnt, p ? 'C' : 'Y', sz, npo, nz, noth,
				first, 16, h);
		}
	}

	if (hevc_appl <= 3)
		pr_info("mt8365 hevc: picture %u COMPLETED (src=%d) irq_status=%08x +0x278=%08x +0x12c=%08x +0x284=%08x\n",
			inst->frm_cnt, mt8365_hevc_done_src, ctx->irq_status,
			rd(ctx, VDEC_HWG, H265_DONE),
			rd(ctx, VDEC_HWG, H265_LCU_XY),
			rd(ctx, VDEC_HWG, H265_ERR_CODE));

	inst->frm_cnt++;
	return 0;
}

static int vdec_hevc_mt8365_get_param(void *h_vdec,
				      enum vdec_get_param_type type, void *out)
{
	struct vdec_hevc_mt8365_inst *inst = h_vdec;
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	u32 b;

	switch (type) {
	case GET_PARAM_PIC_INFO:
		/*
		 * MM21 at the CODED CTB geometry -- see the CAPTURE GEOMETRY
		 * block at the top of this file. The MM21 luma tile is 32 rows
		 * tall, so the declared height carries the extra 32-round.
		 */
		b = hevc_ctb_size(ctx);
		if (mt8365_hevc_geom) {
			ctx->picinfo.buf_w = ALIGN(ctx->picinfo.pic_w, b);
			ctx->picinfo.buf_h = ALIGN(ALIGN(ctx->picinfo.pic_h, b), 32);
		} else {
			ctx->picinfo.buf_w = ALIGN(ctx->picinfo.pic_w, 16);
			ctx->picinfo.buf_h = ALIGN(ctx->picinfo.pic_h, 32);
		}
		ctx->picinfo.fb_sz[0] = ctx->picinfo.buf_w * ctx->picinfo.buf_h;
		ctx->picinfo.fb_sz[1] = ctx->picinfo.fb_sz[0] >> 1;
		pr_info("mt8365 hevc: PIC_INFO pic=%ux%u ctb=%u geom=%d buf=%ux%u fb_sz=%u/%u\n",
			ctx->picinfo.pic_w, ctx->picinfo.pic_h, b, mt8365_hevc_geom,
			ctx->picinfo.buf_w, ctx->picinfo.buf_h,
			ctx->picinfo.fb_sz[0], ctx->picinfo.fb_sz[1]);
		*(struct vdec_pic_info *)out = ctx->picinfo;
		break;
	case GET_PARAM_DPB_SIZE:
		*(unsigned int *)out = HEVC_DPB_SIZE;
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

static void vdec_hevc_mt8365_deinit(void *h_vdec)
{
	struct vdec_hevc_mt8365_inst *inst = h_vdec;

	if (inst)
		free_work_bufs(inst);
	kfree(h_vdec);
}

const struct vdec_common_if vdec_hevc_mt8365_if = {
	.init		= vdec_hevc_mt8365_init,
	.decode		= vdec_hevc_mt8365_decode,
	.get_param	= vdec_hevc_mt8365_get_param,
	.deinit		= vdec_hevc_mt8365_deinit,
};
