// SPDX-License-Identifier: GPL-2.0
/*
 * MT8365 (Genio 350) MPEG-2 stateless decode backend -- direct register
 * programming, no SCP and no codec firmware.
 *
 * WHERE THIS PROGRAM COMES FROM.  Every register, every value and the ORDER
 * they are written in is transcribed from the vendor user-space HAL
 * (libvpud_vcodec.so sha256 da6979b2...ddb53) and from a RUNTIME capture of
 * the command lists that HAL submits.  The vendor MPEG-2 path never touches
 * MMIO directly: mpeg2_risc_settings() builds 40-byte command entries
 * {type, pad, bank_base, reg_offset, value, mask} and submits them through
 * eHalCmdProc(), so the command list IS the complete hardware-programming
 * surface.  An LD_PRELOAD shim on eHalCmdProc captured 12 867 entries over 62
 * pictures of two clips (mcp10ccett 720x576 frame-coded I/P/B, sony-ct1
 * 352x224 field-coded), and this file replays that ordered list as plain
 * writel()s -- the entry opcodes map one-for-one:
 *
 *   type 2  -> wr(bank, off, value)
 *   type 6  -> wr(bank, off, (rd(bank, off) & mask) | value)
 *   type 3  -> rd(bank, off)                (a read-back; value is a pointer)
 *   type 13 -> poll rd(bank, off) until (v & mask)
 *   type 10 -> wait the decode-done IRQ     (mtk_vcodec_wait_for_done_ctx)
 *   type 0/1/14 -> HW lock / release / terminator; the kernel owns those
 *
 * The RE record is the internal MPEG-2 RE set (N138/N140/N141/
 * N143/N144/N145/N156/N180) and the executable model is mediacodek
 * families/no-scp/mt8365/vdec.
 *
 * DEFAULT OFF.  Like vdec_hevc_mt8365_if.c, the register program is behind the
 * module parameter mt8365_mpeg2_arm.  With the arm clear the format still
 * enumerates and negotiates and the decode path returns -EIO WITHOUT touching
 * a single register, so nothing changes for anyone who does not ask for it.
 *
 * Copyright (c) 2026 Libre Computer
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/delay.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>
#include <uapi/linux/v4l2-controls.h>

#include "../mtk_vcodec_dec.h"
#include "../../common/mtk_vcodec_intr.h"
#include "../vdec_drv_base.h"
#include "../vdec_drv_if.h"

/* ---- VDEC_SYS (0x16000000) ---- */
#define SYS_CKEN_SET		0x00
#define SYS_DCM_CON		0x18

/* ---- VDEC_MISC (0x16025000) ---- */
#define MISC_STATE		0x04
#define MISC_CRC_BASE		0x08	/* 0x08..0x24 read-back run */
#define MISC_SYS_SEL		0x84
#define MISC_SPEC_CON		0xc8
#define MISC_FUNC_CON		0xcc
#define MISC_DCM_CON		0xec
#define MISC_SRST		0xf4
#define MISC_IRQ_CFG		0xa4
#define MISC_CFG178		0x178

/* ---- VDEC_LD / MT_VDEC_VLD (0x16020000) ---- */
#define VLD_PIC_PARAM		0x88	/* the whole of v4l2_ctrl_mpeg2_picture */
#define VLD_KICK		0x8c	/* bit20 fetch, bit23 release, 0x4000ff = GO */
#define VLD_PIC_GEOM		0x90	/* ALIGN(v,16)<<16 | version | 0x1ff */
#define VLD_CFG94		0x94
#define VLD_CFG98		0x98
#define VLD_CFG9C		0x9c
#define VLD_FETCH_BASE		0xb0	/* raw PA of the first byte to fetch */
#define VLD_WIN_START		0xb4	/* window start PA >> 6 */
#define VLD_WIN_END		0xb8	/* window end   PA >> 6 (ceil 64) */
#define VLD_DRAIN20		0x20
#define VLD_DRAIN40		0x40
#define VLD_DRAIN60		0x60
#define VLD_FETCH_DONE		0xe8	/* bit0 */
#define VLD_CFGEC		0xec
#define VLD_STATF0		0xf0
#define VLD_IDLE		0xf4
#define VLD_QM_LATCH		0x104
#define VLD_SRST		0x108
#define VLD_WIN_END_RAW		0x110
#define VLD_QM_ADDR		0x260	/* quantiser-matrix index port */
#define VLD_QM_DATA		0x264	/* quantiser-matrix data port */
#define VLD_STREAM_CFG		0x1d0	/* bit8 = auto-prefetch the fetch window */

/* ---- VDEC_TOP / MT_VDEC_VLD_TOP (0x16020800) ---- */
#define TOP_PIC_DIM		0x70

/* ---- VDEC_CM / MT_VDEC_MC (0x16021000) ---- */
#define MC_FWD_Y		0x00	/* forward reference luma >> 9 */
#define MC_FWD_C		0x04	/* forward reference chroma >> 8 */
#define MC_BWD_Y		0x08	/* backward reference luma >> 9 */
#define MC_BWD_C		0x0c	/* backward reference chroma >> 8 */
#define MC_REF_VALID		0x18	/* P/B only; vendor submits 0 */
#define MC_CFG20		0x20
#define MC_PIC_TYPE		0x24	/* I -> 0, P -> 1, B -> 4 */
#define MC_CUR_Y		0x9c	/* destination luma >> 9   (B only) */
#define MC_CUR_C		0xa0	/* destination chroma >> 8 (B only) */
#define MC_PIC_W_PX		0x208
#define MC_PIC_H_PX		0x20c
#define MC_ENABLE220		0x220
#define MC_RECON_LO		0x850	/* vendor: current chroma base >> 4 */
#define MC_RECON_HI		0x854	/* vendor: current luma   base >> 4 */
#define MC_RECON_EN		0x858
#define MC_CFG920		0x920
#define MC_MB_W			0x980
#define MC_CFGA60		0xa60
#define MC_STOP_MBX		0x28
#define MC_STOP_MBY		0x2c

/* ---- VDEC_PP (0x16024000) ---- */
#define PP_CFG3C		0x3c

#define MT8365_MPEG2_TIMEOUT_MS		1000
#define MPEG2_DPB_SIZE			4
#define MPEG2_FETCH_POLL		4096

/*
 * DEFAULT OFF.  Setting this to 1 enables the register program below.  With it
 * clear, MG2S enumerates and negotiates but decode returns -EIO without
 * touching hardware -- the vdec_hevc_mt8365_if.c landing precedent.
 */
static int mt8365_mpeg2_arm;
module_param(mt8365_mpeg2_arm, int, 0644);
MODULE_PARM_DESC(mt8365_mpeg2_arm,
		 "mt8365 MPEG-2: 1 = run the register program (default 0 = OFF, decode returns -EIO)");

/*
 * The vendor programs the VLD fetch window at the OUTPUT buffer's base.  The
 * V4L2 stateless MPEG-2 contract hands us one picture per OUTPUT buffer but
 * does not pin where inside it the slice data begins.  arm 1 = start the fetch
 * at the first slice start code (0x00000101..0x000001af) instead of the buffer
 * base.  Default 0 = transcribe the vendor.
 */
static int mt8365_mpeg2_slicestart = 1;
module_param(mt8365_mpeg2_slicestart, int, 0644);
MODULE_PARM_DESC(mt8365_mpeg2_slicestart,
		 "mt8365 MPEG-2: 1 = point the VLD at the first slice start code (default), 0 = buffer base");

/*
 * The vendor's post-fetch run of reads at VDEC_LD+0x60 is NOT a status poll: on
 * this VLD a read at +(n * 0x20) CONSUMES n bytes from the barrel shifter and
 * returns the new window (the H.264 backend documents the same mechanism for
 * AVCVLD, "advanced only by CONSUMING bits"), so +0x60 takes THREE bytes each
 * time.  vpud's count is data-dependent (16 on mcp10ccett, 13..116 across
 * sony-ct1's pictures) because it is scanning the picture header in software to
 * reach the first slice.  A FIXED count is therefore wrong by construction: it
 * desynchronises the barrel and the HW resynchronises only at a later slice
 * start code, losing whole macroblock rows off the TOP of the picture.
 * mt8365_mpeg2_slicestart=1 makes the scan unnecessary by aiming the fetch at
 * the slice start code directly, so the default here is 0 reads.
 */
static int mt8365_mpeg2_drain;
module_param(mt8365_mpeg2_drain, int, 0644);
MODULE_PARM_DESC(mt8365_mpeg2_drain,
		 "mt8365 MPEG-2: number of 3-byte VDEC_LD+0x60 barrel consumes after the fetch (default 0)");

/*
 * 0 = never load the quantiser matrices (byte-for-byte the vendor on a stream
 * that signals none), 1 = load on the first picture and on any change,
 * 2 = load on every picture (default).
 *
 * "On every picture" is not belt-and-braces, it is the measured requirement:
 * the block does not retain the matrices across the per-picture
 * mpeg2_reset_hw(), so a picture is decoded correctly if and only if a load
 * happened on that picture. Under the old on-change default, `att` loads once
 * at picture 0 and every one of its 131 later pictures is wrong, and `ibm-bw`
 * picture 12 -- whose picture header is byte-identical to picture 1's -- has
 * all 334 of its intra macroblocks wrong at maxabs 126 while every one of its
 * 1016 inter macroblocks is within 1 LSB, because an intra macroblock is
 * dequantised with the intra matrix and an inter macroblock with the non-intra
 * one. Measured on all 43 MPEG2_VIDEO-MAIN vectors: ISO/IEC 13818-4 2.4.4.1
 * goes 12/43 -> 20/43 against the vendor stack's 21/43 with no regression on
 * any row, and the vectors that signal no matrix at all are byte-identical
 * between the two policies (the internal N188 quantiser-matrix audit).
 *
 * The whole MPEG-2 register program is still behind the default-OFF
 * mt8365_mpeg2_arm, so this changes nothing for anyone who does not set it.
 */
static int mt8365_mpeg2_qm = 2;
module_param(mt8365_mpeg2_qm, int, 0644);
MODULE_PARM_DESC(mt8365_mpeg2_qm,
		 "mt8365 MPEG-2: quantiser-matrix load policy 0=never 1=on-change 2=always (default)");

/*
 * The vendor writes MC+0x850 = current CHROMA base >> 4 and MC+0x854 = current
 * LUMA base >> 4.  In vpud's allocation chroma sits BELOW luma; in ours luma
 * sits below chroma, so if the pair is really a [low, high] recon window guard
 * (which is how the VP8 backend had to program the same two offsets) the roles
 * swap with the layout.  arm 1 = program min at 0x850 / max at 0x854 instead.
 * Default 0 = transcribe the vendor's per-plane assignment.
 */
static int mt8365_mpeg2_reconwin;
module_param(mt8365_mpeg2_reconwin, int, 0644);
MODULE_PARM_DESC(mt8365_mpeg2_reconwin,
		 "mt8365 MPEG-2: 1 = treat MC+0x850/0x854 as a [low,high] window (default 0 = C/Y as the vendor writes them)");

/*
 * N182 ARM -- auto-prefetch enable, VDEC_LD+0x1d0 bit8.
 *
 * The eHalCmdProc shim N156 captured the command list with is structurally
 * blind to any register the vendor HAL writes with DIRECT MMIO rather than as
 * a command entry, and eBufGetBitStream (@0x2d820) is exactly that: it sets
 * VLD+0x1d0 |= 0x100 before kicking the fetch.  So this write can never have
 * appeared in the 12 867 captured entries, and its absence here is a property
 * of the instrument, not evidence that the vendor omits it.
 *
 * The same register is the H.264 backend's own fix for the same signature: a
 * single primed fetch UNDERRUNS because the MB decode STREAMS the bitstream,
 * and the engine then runs the picture out without ever asserting dec-done
 * (MISC+0xa4 stalls at 0x1000, never 0x10000 = DEC_SUCCESS, SYS+0x00 stays
 * 0x111 HW-active).  N182 measured that signature on 36 of 36 MPEG-2 timeout
 * events across 30 vectors, with MC+0x2c (mby) == mb_h - 1 on EVERY one --
 * i.e. the engine reaches the LAST macroblock row of the picture, at every
 * resolution from 2x2 MBs to 45x36.  It is not stopping early on a syntax
 * element; it is finishing on starved data and never signalling done.
 *
 * Default 0 so this kernel's baseline reproduces the pre-N182 score exactly on
 * the same binary; 1 is the arm.  Live lever, no rebuild to A/B it.
 *
 * FALSIFIER, registered before the run: if with =1 the timeouts persist with
 * the SAME MISC+0xa4 = 0x1000/0x1001 and the same mby == mb_h - 1, the
 * bitstream-underrun mechanism is dead for MPEG-2 and must not be re-mined.
 *
 * ===== RESULT: the arm is ELIMINATED. Do NOT mine variants of it. =====
 *
 * Measured on this kernel, interleaved on one boot so nothing is attributed to
 * a wedge -- control decode of mcp10ccett before, during and after, each
 * compared to its PINNED md5 0706d9e882915d83a34c28308cb4a4c0:
 *
 *   CTL[A] prefetch=0 rc=0 md5=0706d9e882915d83a34c28308cb4a4c0   (bit-exact)
 *   CTL[B] prefetch=1 rc=0 md5=d41d8cd98f00b204e9800998ecf8427e   (0 bytes)
 *   CTL[C] prefetch=0 rc=0 md5=0706d9e882915d83a34c28308cb4a4c0   (bit-exact)
 *   CTL[D] prefetch=0 rc=0 md5=0706d9e882915d83a34c28308cb4a4c0   (bit-exact)
 *
 * So the block was never wedged and the effect is the lever's alone. With =1
 * the arm makes things WORSE in both directions: it breaks the two vectors
 * that decode, and it fixes none of the timeouts. MISC+0xa4 is unchanged at
 * 0x1000 (still no DEC_SUCCESS), so the completion story does not move.
 *
 * It is NOT inert, which is the informative part -- the stop position
 * COLLAPSES to the origin:
 *
 *   prefetch=0  conf2  mbx=43/44 mby=35/35   LDe8=00000811 LDf4=00018008
 *   prefetch=1  conf2  mbx= 0/44 mby= 0/35   LDe8=00001811 LDf4=00048007
 *
 * i.e. bit8 of VDEC_LD+0x1d0 on this VLD is not the H.264 auto-prefetch
 * semantic; setting it stops the decoder from starting at all. Kept in the
 * tree at default 0 for exactly one reason: it is the ONLY manipulation found
 * that moves MC+0x28/0x2c, which is what proves those two registers are a LIVE
 * stop position rather than a latched constant or an echo of the geometry --
 * the positive control the "mby == mb_h - 1 on every event" finding needs.
 */
static int mt8365_mpeg2_prefetch;
module_param(mt8365_mpeg2_prefetch, int, 0644);
MODULE_PARM_DESC(mt8365_mpeg2_prefetch,
		 "mt8365 MPEG-2: 1 = set VDEC_LD+0x1d0 bit8 (auto-prefetch the fetch window) before the kick (default 0)");

/*
 * N185 -- the VLD fetch-window arm, pre-registered with its falsifier in N184
 * section 6 and run here unmodified.
 *
 * MECHANISM.  vdec_mpeg2_mt8365_decode() computes bs_end = bs->dma_addr +
 * bs->size and programs it into VLD_WIN_END_RAW (VDEC_LD+0x110) and, ceil-64,
 * into VLD_WIN_END (+0xb8), so the fetch window closes on exactly the last
 * coded byte.  A barrel-shifter front end needs bytes PAST the last coded bit
 * to decode the final symbols, so a window closed on the last byte starves the
 * fetch while the final macroblock is still in flight.  That predicts N184's
 * measured law exactly: the engine completes every macroblock and stops ONE
 * short (MC+0x28 = mb_w - 2), at every geometry from 2x2 to 45x36 and in both
 * picture structures, then sits busy with no interrupt.
 *
 * THE LEVER.  A count of bytes by which the PROGRAMMED window end is extended
 * past the last coded byte.  Nothing else moves: bs->size, VLD_FETCH_BASE, the
 * DMA mapping and every other register are untouched, and the extension is
 * CLAMPED to the source plane's own allocation (vb2_plane_size), so the engine
 * is never pointed at a byte outside the buffer vb2 already gave us.  Where
 * the allocation has no slack the clamp yields 0 and the leg degenerates to
 * the control -- which is why the differ-proof below is mandatory, not
 * optional (N183's arm was a null manipulation for exactly this reason).
 *
 * FALSIFIER (banked before the run): the timeouts persist with MC+0x28 still
 * exactly one short, SYS+0x00 = 0x111 and no interrupt.
 */
static int mt8365_mpeg2_lookahead;
module_param(mt8365_mpeg2_lookahead, int, 0644);
MODULE_PARM_DESC(mt8365_mpeg2_lookahead,
		 "mt8365 MPEG-2: extend the programmed VLD fetch window this many bytes past the last coded byte, clamped to the source plane's allocation (default 0 = the window ends on the last coded byte)");

static int mt8365_mpeg2_diag;
module_param(mt8365_mpeg2_diag, int, 0644);
MODULE_PARM_DESC(mt8365_mpeg2_diag,
		 "mt8365 MPEG-2: 1 = one summary line per picture; 2 = also the PREKICK + DONE register lines, byte-for-byte diffable against the TIMEOUT line (default 0, zero MMIO)");

/*
 * ISO/IEC 13818-2 default quantiser matrices, in ZIGZAG order -- the order
 * v4l2_ctrl_mpeg2_quantisation uses and the order the vendor streams into the
 * VLD_QM_DATA port.  Derived from the spec's raster-order tables by applying
 * the Figure 7-2 zigzag scan; the first two packed words are 0x08101013 and
 * 0x10131616, which is exactly what the runtime capture shows the vendor
 * writing for a stream that loads the default intra matrix.
 */
static const u8 mpeg2_default_intra_zz[64] = {
	8, 16, 16, 19, 16, 19, 22, 22,
	22, 22, 22, 22, 26, 24, 26, 27,
	27, 27, 26, 26, 26, 26, 27, 27,
	27, 29, 29, 29, 34, 34, 34, 29,
	29, 29, 27, 27, 29, 29, 32, 32,
	34, 34, 37, 38, 37, 35, 35, 34,
	35, 38, 38, 40, 40, 40, 48, 48,
	46, 46, 56, 56, 58, 69, 69, 83,
};

struct vdec_mpeg2_mt8365_inst {
	struct mtk_vcodec_dec_ctx *ctx;
	unsigned int frm_cnt;

	/* last programmed matrices, so a picture that changes nothing does not
	 * re-stream 128 port writes -- and so the first picture always does. */
	bool qm_valid;
	u8 qm_intra[64];
	u8 qm_non_intra[64];

	/* fallback references when the picture control is absent */
	u32 last_y, last_c;
	bool have_last;
};

static inline void wr(struct mtk_vcodec_dec_ctx *ctx, int blk, u32 off, u32 val)
{
	writel(val, ctx->dev->reg_base[blk] + off);
}

static inline u32 rd(struct mtk_vcodec_dec_ctx *ctx, int blk, u32 off)
{
	return readl(ctx->dev->reg_base[blk] + off);
}

/*
 * Command-list entries 0..9: the inlined ResetHW block.  The vendor cannot call
 * GetPDNSetting from a command list (the list is built before it is submitted),
 * so mpeg2_get_platform_data() hard-codes the three PDN words; the values below
 * are those literals, confirmed on every one of the 62 captured pictures.
 */
static void mpeg2_reset_hw(struct mtk_vcodec_dec_ctx *ctx)
{
	wr(ctx, VDEC_LD,   VLD_SRST,      0x101);
	wr(ctx, VDEC_MISC, MISC_SPEC_CON, 0x0000fffe);
	wr(ctx, VDEC_MISC, MISC_FUNC_CON, 0x7f6a11df);
	wr(ctx, VDEC_MISC, MISC_CFG178,   0xfffffffb);
	wr(ctx, VDEC_MISC, MISC_SRST,     0x0);
	wr(ctx, VDEC_MISC, MISC_SYS_SEL,  0x1);
	wr(ctx, VDEC_LD,   VLD_SRST,      0x0);
	wr(ctx, VDEC_SYS,  SYS_DCM_CON,   0x1);
	wr(ctx, VDEC_MISC, MISC_DCM_CON,  0x1);
	wr(ctx, VDEC_MISC, MISC_STATE,    0x1);
}

/*
 * Entries 10..40: arm the VLD over [start, end) and run one fetch, then drain.
 * The drain reads (LD+0xf0, a variable run of LD+0x60, LD+0x40, LD+0xf0) are
 * reproduced because the vendor issues them between the fetch and the picture
 * parameters; the count of LD+0x60 reads varies per picture in the capture
 * (13..116), so this polls a bounded run rather than replaying a fixed count.
 */
static void mpeg2_arm_fetch(struct mtk_vcodec_dec_ctx *ctx, dma_addr_t start,
			    dma_addr_t end)
{
	int i;

	(void)rd(ctx, VDEC_LD, VLD_IDLE);
	wr(ctx, VDEC_LD, VLD_WIN_START,  (u32)(start >> 6));
	wr(ctx, VDEC_LD, VLD_WIN_END,    (u32)(ALIGN(end, 64) >> 6));
	wr(ctx, VDEC_LD, VLD_FETCH_BASE, (u32)start);
	wr(ctx, VDEC_LD, VLD_WIN_END_RAW, (u32)end);
	wr(ctx, VDEC_LD, VLD_SRST, 0x100);
	wr(ctx, VDEC_LD, VLD_SRST, 0x0);
	wr(ctx, VDEC_LD, VLD_CFG9C, 0x00030fff);
	wr(ctx, VDEC_LD, VLD_CFGEC, rd(ctx, VDEC_LD, VLD_CFGEC) | 0x10000000);
	if (mt8365_mpeg2_prefetch)
		wr(ctx, VDEC_LD, VLD_STREAM_CFG,
		   rd(ctx, VDEC_LD, VLD_STREAM_CFG) | 0x100);
	/*
	 * N185 -- the MANDATORY differ-proof for the lookahead arm.  Read AFTER
	 * the SRST pulse, so it reports what the engine is actually about to
	 * fetch with, not merely that the write was accepted.  A null result
	 * whose two legs print the same WINRAW is UNEVALUATED, never an
	 * elimination.  diag >= 2 only: zero added MMIO at the default.
	 */
	if (mt8365_mpeg2_diag >= 2)
		mtk_v4l2_vdec_err(ctx,
			"mt8365 mpeg2 WIN start=%pad end=%pad look=%d | LD[0x110]=%08x LD[0xb8]=%08x LD[0xb4]=%08x",
			&start, &end, mt8365_mpeg2_lookahead,
			rd(ctx, VDEC_LD, VLD_WIN_END_RAW),
			rd(ctx, VDEC_LD, VLD_WIN_END),
			rd(ctx, VDEC_LD, VLD_WIN_START));
	wr(ctx, VDEC_LD, VLD_KICK, 0x00100000);
	for (i = 0; i < MPEG2_FETCH_POLL; i++) {
		if (rd(ctx, VDEC_LD, VLD_FETCH_DONE) & 0x1)
			break;
		cpu_relax();
	}
	wr(ctx, VDEC_LD, VLD_KICK, 0x00800000);

	(void)rd(ctx, VDEC_LD, VLD_STATF0);
	for (i = 0; i < mt8365_mpeg2_drain; i++)
		(void)rd(ctx, VDEC_LD, VLD_DRAIN60);
	if (mt8365_mpeg2_drain)
		(void)rd(ctx, VDEC_LD, VLD_DRAIN40);
	(void)rd(ctx, VDEC_LD, VLD_STATF0);
}

/*
 * The command list's LAST opcode before the terminator is type 1, RELEASE_HW,
 * whose handler (@0x2f8dc) waits for SRAM idle, writes VDEC_SYS+0x4 = 1 and
 * unlocks the engine.  The kernel owns the lock, but the +0x4 write is a real
 * hardware release and omitting it is what leaves SYS+0x0 stuck at 0x111
 * (HW-active) after a picture, so the NEXT decode -- in any process -- times
 * out.  Emitted on the success path AND on both failure paths, which is the
 * point: a wedge that survives the wedging decode poisons every vector after
 * it in a sweep.
 */
static void mpeg2_release_hw(struct mtk_vcodec_dec_ctx *ctx)
{
	int i;

	for (i = 0; i < 4096; i++) {
		if (!(rd(ctx, VDEC_SYS, SYS_CKEN_SET) & 0x10))
			break;
		cpu_relax();
	}
	wr(ctx, VDEC_SYS, 0x4, 0x1);
}

/*
 * FAILURE PATH -- stop the VLD before releasing, which the success path does
 * not need.  MEASURED, not defensive: a decode that times out leaves the VLD
 * FETCHING, and it walks off the end of the bitstream mapping.  On 2026-09-03 a
 * 43-vector sweep on this backend produced an mtk-iommu fault storm from the
 * VDEC master (larb=3 port=7, ~2.07 M suppressed callbacks per 5 s at
 * iova 0xff78xc01 .. 0xff79xc01, pa=0), which starved the board into hung-task
 * reports and took it off the network; it needed a hard reset.  The
 * timeout itself is a per-vector result; the runaway fetch is what turned one
 * bad vector into a dead board and every later verdict in the sweep into
 * garbage.  Drop the fetch enable, then hold the VLD and the MISC block in
 * reset, then release.  Only VDEC_LD / VDEC_MISC / VDEC_SYS are touched -- the
 * H.264 backend records that a broad read of the wedged VLD's own window hangs
 * the AXI bus, so nothing here reads back.
 */
static void mpeg2_abort_hw(struct mtk_vcodec_dec_ctx *ctx)
{
	wr(ctx, VDEC_LD,   VLD_KICK,   0x0);
	wr(ctx, VDEC_LD,   VLD_SRST,   0x101);
	wr(ctx, VDEC_LD,   VLD_SRST,   0x0);
	wr(ctx, VDEC_MISC, MISC_SRST,  0x0);
	wr(ctx, VDEC_MISC, MISC_STATE, 0x0);
	wr(ctx, VDEC_MISC, MISC_IRQ_CFG, 0x0);
	mpeg2_release_hw(ctx);
}

/*
 * LoadQuantizerMatrix().  Per 32-bit word i the vendor emits three entries in
 * buffer order: the 0x2xx index, the data word, then the 0x1xx index.  One word
 * carries four zigzag coefficients, most significant coefficient first.
 */
static void mpeg2_load_matrix(struct mtk_vcodec_dec_ctx *ctx, const u8 *m,
			      u32 hi_base, u32 lo_base)
{
	int i;

	for (i = 0; i < 16; i++) {
		u32 w = ((u32)m[4 * i] << 24) | ((u32)m[4 * i + 1] << 16) |
			((u32)m[4 * i + 2] << 8) | (u32)m[4 * i + 3];

		wr(ctx, VDEC_LD, VLD_QM_ADDR, hi_base + 4 * i);
		wr(ctx, VDEC_LD, VLD_QM_DATA, w);
		wr(ctx, VDEC_LD, VLD_QM_ADDR, lo_base + 4 * i);
	}
}

/* v4l2_ctrl_mpeg2_picture -> VLD+0x88, the one register the whole control maps
 * to.  Bit assignment transcribed from the shifts that build it at 0x8c2c4.
 * f_code is passed through VERBATIM: the capture shows the vendor copying the
 * coded value even on an I picture, where five of ten captured pictures carry
 * 4/4 rather than the 15/15 an "unused" reading would synthesise. */
static u32 mpeg2_pic_param_word(const struct v4l2_ctrl_mpeg2_picture *pic)
{
	u32 f = pic->flags;
	u32 v;

	v  = ((u32)pic->f_code[1][1] & 0xf);
	v |= ((u32)pic->f_code[1][0] & 0xf) << 4;
	v |= ((u32)pic->f_code[0][1] & 0xf) << 8;
	v |= ((u32)pic->f_code[0][0] & 0xf) << 12;
	v |= (f & V4L2_MPEG2_PIC_FLAG_ALT_SCAN)       ? BIT(17) : 0;
	v |= ((u32)pic->intra_dc_precision & 0x3) << 18;
	v |= (f & V4L2_MPEG2_PIC_FLAG_INTRA_VLC)      ? BIT(20) : 0;
	v |= ((u32)pic->picture_coding_type & 0x7) << 21;
	/* bits 24/25 are full_pel_{backward,forward}_vector, MPEG-1-only
	 * syntax that v4l2_ctrl_mpeg2_picture does not carry. */
	v |= (f & V4L2_MPEG2_PIC_FLAG_TOP_FIELD_FIRST) ? BIT(26) : 0;
	v |= (f & V4L2_MPEG2_PIC_FLAG_Q_SCALE_TYPE)    ? BIT(27) : 0;
	v |= (f & V4L2_MPEG2_PIC_FLAG_CONCEALMENT_MV)  ? BIT(28) : 0;
	v |= (f & V4L2_MPEG2_PIC_FLAG_FRAME_PRED_DCT)  ? BIT(29) : 0;
	v |= ((u32)pic->picture_structure & 0x3) << 30;
	return v;
}

/* MC+0x24, measured over all 62 captured pictures: I -> 0, P -> 1, B -> 4,
 * independent of picture_structure. */
static u32 mpeg2_pic_type_code(u8 pct)
{
	switch (pct) {
	case V4L2_MPEG2_PIC_CODING_TYPE_P:
		return 1;
	case V4L2_MPEG2_PIC_CODING_TYPE_B:
		return 4;
	default:
		return 0;
	}
}

static u32 mpeg2_find_slice_start(const u8 *p, size_t n)
{
	size_t i;

	if (!p)
		return 0;
	for (i = 0; i + 3 < n; i++) {
		if (p[i] == 0 && p[i + 1] == 0 && p[i + 2] == 1 &&
		    p[i + 3] >= 0x01 && p[i + 3] <= 0xaf)
			return (u32)i;
	}
	return 0;
}

static int vdec_mpeg2_mt8365_init(struct mtk_vcodec_dec_ctx *ctx)
{
	struct vdec_mpeg2_mt8365_inst *inst;

	inst = kzalloc(sizeof(*inst), GFP_KERNEL);
	if (!inst)
		return -ENOMEM;
	inst->ctx = ctx;
	ctx->drv_handle = inst;
	return 0;
}

static void mpeg2_resolve_refs(struct vdec_mpeg2_mt8365_inst *inst,
			       const struct v4l2_ctrl_mpeg2_picture *pic,
			       u32 cur_y, u32 cur_c,
			       u32 *fwd_y, u32 *fwd_c, u32 *bwd_y, u32 *bwd_c)
{
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	struct vb2_queue *vq;
	struct vb2_buffer *vb;

	/* Vendor default, from the capture: an I picture's forward slot is its
	 * own pair, and a P picture's backward slot is its own pair. */
	*fwd_y = inst->have_last ? inst->last_y : cur_y;
	*fwd_c = inst->have_last ? inst->last_c : cur_c;
	*bwd_y = cur_y;
	*bwd_c = cur_c;
	if (!pic)
		return;
	if (pic->picture_coding_type == V4L2_MPEG2_PIC_CODING_TYPE_I) {
		*fwd_y = cur_y;
		*fwd_c = cur_c;
		return;
	}

	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	if (!vq)
		return;

	vb = vb2_find_buffer(vq, pic->forward_ref_ts);
	if (vb && vb->num_planes >= 2) {
		*fwd_y = (u32)vb2_dma_contig_plane_dma_addr(vb, 0);
		*fwd_c = (u32)vb2_dma_contig_plane_dma_addr(vb, 1);
	}
	if (pic->picture_coding_type == V4L2_MPEG2_PIC_CODING_TYPE_B) {
		vb = vb2_find_buffer(vq, pic->backward_ref_ts);
		if (vb && vb->num_planes >= 2) {
			*bwd_y = (u32)vb2_dma_contig_plane_dma_addr(vb, 0);
			*bwd_c = (u32)vb2_dma_contig_plane_dma_addr(vb, 1);
		}
	}
}

static int vdec_mpeg2_mt8365_decode(void *h_vdec, struct mtk_vcodec_mem *bs,
				    struct vdec_fb *unused, bool *res_chg)
{
	struct vdec_mpeg2_mt8365_inst *inst = h_vdec;
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	const struct v4l2_ctrl_mpeg2_sequence *seq = NULL;
	const struct v4l2_ctrl_mpeg2_picture *pic = NULL;
	const struct v4l2_ctrl_mpeg2_quantisation *qm = NULL;
	struct v4l2_ctrl *ctrl;
	struct vdec_fb *fb;
	dma_addr_t bs_start, bs_end;
	u32 pic_w, pic_h, aw, ah, mb_w;
	u32 cur_y, cur_c, fwd_y, fwd_c, bwd_y, bwd_c;
	u32 r_88, r_90, r_top70, pstruct, pct, off;
	size_t look;
	const u8 *qi, *qn;
	bool is_b, is_i, load_qm;
	int ret;

	*res_chg = false;
	if (!bs)			/* flush / EOS */
		return 0;

	if (!mt8365_mpeg2_arm) {
		/*
		 * DEFAULT-OFF ARM.  Nothing below this point has run; no
		 * register of any block has been touched by this backend.
		 */
		if (!inst->frm_cnt++)
			mtk_v4l2_vdec_err(ctx,
				"mt8365 mpeg2: register program is OFF (set mt8365_mpeg2_arm=1)");
		return -EIO;
	}

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

	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, V4L2_CID_STATELESS_MPEG2_SEQUENCE);
	if (ctrl)
		seq = ctrl->p_cur.p;
	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, V4L2_CID_STATELESS_MPEG2_PICTURE);
	if (ctrl)
		pic = ctrl->p_cur.p;
	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl,
			      V4L2_CID_STATELESS_MPEG2_QUANTISATION);
	if (ctrl)
		qm = ctrl->p_cur.p;
	if (!pic) {
		mtk_v4l2_vdec_err(ctx, "mt8365 mpeg2: no PICTURE control");
		return -EINVAL;
	}

	/*
	 * horizontal_size / vertical_size already carry the
	 * *_size_extension fold (ISO/IEC 13818-2 section 6.3.3) -- the V4L2
	 * control is defined as the combination, which is exactly what the
	 * vendor's ctx+0 / ctx+4 hold after MPEG2_Parse_Sequence_Header.
	 */
	pic_w = seq ? seq->horizontal_size : ctx->picinfo.pic_w;
	pic_h = seq ? seq->vertical_size   : ctx->picinfo.pic_h;
	if (!pic_w || !pic_h) {
		mtk_v4l2_vdec_err(ctx, "mt8365 mpeg2: no picture size");
		return -EINVAL;
	}
	aw = ALIGN(pic_w, 16);
	ah = ALIGN(pic_h, 16);
	mb_w = aw >> 4;

	pct = pic->picture_coding_type;
	pstruct = pic->picture_structure;
	is_i = pct == V4L2_MPEG2_PIC_CODING_TYPE_I;
	is_b = pct == V4L2_MPEG2_PIC_CODING_TYPE_B;

	cur_y = (u32)fb->base_y.dma_addr;
	cur_c = (u32)fb->base_c.dma_addr;
	mpeg2_resolve_refs(inst, pic, cur_y, cur_c,
			   &fwd_y, &fwd_c, &bwd_y, &bwd_c);

	off = mt8365_mpeg2_slicestart ?
		mpeg2_find_slice_start(bs->va, bs->size) : 0;
	if (off >= bs->size)
		off = 0;
	bs_start = bs->dma_addr + off;
	bs_end = bs->dma_addr + bs->size;

	/*
	 * N185 -- extend the programmed fetch window past the last coded byte,
	 * never past the allocation vb2 handed us.  See the module-param block.
	 */
	{
		struct mtk_video_dec_buf *sbi =
			container_of(bs, struct mtk_video_dec_buf, bs_buffer);
		size_t cap = vb2_plane_size(&sbi->m2m_buf.vb.vb2_buf, 0);

		look = 0;
		if (mt8365_mpeg2_lookahead > 0 && cap > bs->size)
			look = min_t(size_t, (size_t)mt8365_mpeg2_lookahead,
				     cap - bs->size);
		bs_end += look;
		if (mt8365_mpeg2_diag >= 2)
			mtk_v4l2_vdec_err(ctx,
				"mt8365 mpeg2 WINCALC f%u bytesused=%zx plane_cap=%zx lookahead=%d applied=%zx",
				inst->frm_cnt, bs->size, cap,
				mt8365_mpeg2_lookahead, look);
	}

	mtk_vcodec_mt8365_bs_audit(ctx, bs->dma_addr, bs->size);

	r_88 = mpeg2_pic_param_word(pic);
	/*
	 * VLD+0x90.  [31:16] = ALIGN(vertical_size,16); bit30 = the MPEG-2
	 * stream-version select (bit31 is the MPEG-1 one, which this control
	 * path never sees -- an MPEG-1 stream is carried by the MPEG-2
	 * controls and the vendor's own tag comes from sequence_extension
	 * presence).  [8:0] are forced to 1 by the vendor's final orr #0x1ff,
	 * which is why the macroblock-width term the disassembly also adds is
	 * DEAD in the emitted word and is not reproduced here.
	 */
	r_90 = (ah << 16) | 0x40000000 | 0x1ff;
	/*
	 * TOP+0x70.  The FIELD arm HALVES the height -- this is the one place
	 * the H.264 backend's identical-looking expression must NOT be copied
	 * (it keeps the full frame height for a field picture).
	 */
	if (pstruct == V4L2_MPEG2_PIC_FRAME)
		r_top70 = (ah << 16) | aw;
	else
		r_top70 = aw | (ah << 15);

	qi = (qm && qm->intra_quantiser_matrix[0]) ?
		qm->intra_quantiser_matrix : mpeg2_default_intra_zz;
	qn = (qm && qm->non_intra_quantiser_matrix[0]) ?
		qm->non_intra_quantiser_matrix : NULL;
	load_qm = mt8365_mpeg2_qm == 2 ||
		  (mt8365_mpeg2_qm == 1 &&
		   (!inst->qm_valid || memcmp(inst->qm_intra, qi, 64) ||
		    (qn && memcmp(inst->qm_non_intra, qn, 64))));

	/* ================= the ordered command list ================= */

	mpeg2_reset_hw(ctx);
	mpeg2_arm_fetch(ctx, bs_start, bs_end);

	wr(ctx, VDEC_LD,  VLD_PIC_PARAM, r_88);
	wr(ctx, VDEC_LD,  VLD_PIC_GEOM,  r_90);
	wr(ctx, VDEC_TOP, TOP_PIC_DIM,   r_top70);
	wr(ctx, VDEC_CM,  MC_MB_W,       mb_w);

	wr(ctx, VDEC_LD, VLD_QM_LATCH, 0x0);
	wr(ctx, VDEC_LD, VLD_QM_LATCH, 0x400);
	if (load_qm) {
		u8 nonintra[64];

		if (!qn) {
			memset(nonintra, 16, sizeof(nonintra));
			qn = nonintra;
		}
		mpeg2_load_matrix(ctx, qi, 0x200, 0x100);
		wr(ctx, VDEC_LD, VLD_QM_LATCH, 0x100);
		mpeg2_load_matrix(ctx, qn, 0x240, 0x140);
		wr(ctx, VDEC_LD, VLD_QM_LATCH, 0x200);
		memcpy(inst->qm_intra, qi, 64);
		memcpy(inst->qm_non_intra, qn, 64);
		inst->qm_valid = true;
	}

	wr(ctx, VDEC_LD, VLD_CFG94, 0x1ff);
	wr(ctx, VDEC_LD, VLD_CFG98, 0x3ff);
	wr(ctx, VDEC_CM, MC_ENABLE220, 0x0);
	wr(ctx, VDEC_CM, MC_CFGA60, 0x0);

	/* PASS 1 -- prime every slot with the CURRENT picture's own pair. */
	wr(ctx, VDEC_CM, MC_FWD_Y, cur_y >> 9);
	wr(ctx, VDEC_CM, MC_FWD_C, cur_c >> 8);
	wr(ctx, VDEC_CM, MC_BWD_Y, cur_y >> 9);
	wr(ctx, VDEC_CM, MC_BWD_C, cur_c >> 8);
	if (is_b) {
		wr(ctx, VDEC_CM, MC_CUR_Y, cur_y >> 9);
		wr(ctx, VDEC_CM, MC_CUR_C, cur_c >> 8);
	}

	/* PASS 2 -- assign. */
	wr(ctx, VDEC_CM, MC_FWD_Y, fwd_y >> 9);
	wr(ctx, VDEC_CM, MC_FWD_C, fwd_c >> 8);
	if (!is_i) {
		wr(ctx, VDEC_CM, MC_BWD_Y, bwd_y >> 9);
		wr(ctx, VDEC_CM, MC_BWD_C, bwd_c >> 8);
		wr(ctx, VDEC_CM, MC_REF_VALID, 0x0);
	}
	if (is_b) {
		wr(ctx, VDEC_CM, MC_CUR_Y, cur_y >> 9);
		wr(ctx, VDEC_CM, MC_CUR_C, cur_c >> 8);
	}

	wr(ctx, VDEC_CM, MC_CFG920, 0x01000000);
	wr(ctx, VDEC_PP, PP_CFG3C,  0x10000000);
	if (mt8365_mpeg2_reconwin) {
		wr(ctx, VDEC_CM, MC_RECON_LO, min(cur_y, cur_c) >> 4);
		wr(ctx, VDEC_CM, MC_RECON_HI, max(cur_y, cur_c) >> 4);
	} else {
		wr(ctx, VDEC_CM, MC_RECON_LO, cur_c >> 4);
		wr(ctx, VDEC_CM, MC_RECON_HI, cur_y >> 4);
	}
	wr(ctx, VDEC_CM, MC_RECON_EN, 0x1);
	wr(ctx, VDEC_CM, MC_CFG20, 0x0);
	wr(ctx, VDEC_CM, MC_PIC_TYPE, mpeg2_pic_type_code(pct));
	wr(ctx, VDEC_CM, MC_PIC_W_PX, aw);
	wr(ctx, VDEC_CM, MC_PIC_H_PX, ah);

	/*
	 * N184 -- the SUCCESS-path control for the timeout register set.
	 *
	 * Every end-of-picture register reading this backend has produced came
	 * from the failure path below, so MISC[0xa4]=0x1000 / LD[0xe8]=…811 /
	 * LD[0xf4] were absolutes with nothing to subtract.  diag >= 2 emits the
	 * SAME seven registers, in the SAME order and the SAME format, at two
	 * points on EVERY picture: immediately BEFORE the kick (so a difference
	 * that already exists at t=0 is separable from one the decode creates)
	 * and immediately AFTER a successful wait, which is exactly where the
	 * TIMEOUT line reads them.  Costs zero MMIO at the default diag = 0.
	 */
	if (mt8365_mpeg2_diag >= 2)
		mtk_v4l2_vdec_err(ctx,
			"mt8365 mpeg2 PREKICK f%u %ux%u pct=%u ps=%u bs=%pad+%x sz=%zx | SYS[0x00]=%08x MISC[0xa4]=%08x [0x04]=%08x | LD[0xe8]=%08x [0xf4]=%08x | MC[0x28]=%08x [0x2c]=%08x",
			inst->frm_cnt, pic_w, pic_h, pct, pstruct,
			&bs->dma_addr, off, bs->size,
			rd(ctx, VDEC_SYS, SYS_CKEN_SET),
			rd(ctx, VDEC_MISC, MISC_IRQ_CFG),
			rd(ctx, VDEC_MISC, MISC_STATE),
			rd(ctx, VDEC_LD, VLD_FETCH_DONE),
			rd(ctx, VDEC_LD, VLD_IDLE),
			rd(ctx, VDEC_CM, MC_STOP_MBX),
			rd(ctx, VDEC_CM, MC_STOP_MBY));

	/* KICK -- the last entry before the vendor's WAIT_ISR. */
	wr(ctx, VDEC_LD, VLD_KICK, 0x004000ff);

	ret = mtk_vcodec_wait_for_done_ctx(ctx, MTK_INST_IRQ_RECEIVED,
					   MT8365_MPEG2_TIMEOUT_MS,
					   MTK_VDEC_CORE);
	if (ret) {
		mtk_v4l2_vdec_err(ctx,
			"mt8365 mpeg2 TIMEOUT f%u %ux%u pct=%u ps=%u bs=%pad+%x sz=%zx | SYS[0x00]=%08x MISC[0xa4]=%08x [0x04]=%08x | LD[0xe8]=%08x [0xf4]=%08x | MC[0x28]=%08x [0x2c]=%08x",
			inst->frm_cnt, pic_w, pic_h, pct, pstruct,
			&bs->dma_addr, off, bs->size,
			rd(ctx, VDEC_SYS, SYS_CKEN_SET),
			rd(ctx, VDEC_MISC, MISC_IRQ_CFG),
			rd(ctx, VDEC_MISC, MISC_STATE),
			rd(ctx, VDEC_LD, VLD_FETCH_DONE),
			rd(ctx, VDEC_LD, VLD_IDLE),
			rd(ctx, VDEC_CM, MC_STOP_MBX),
			rd(ctx, VDEC_CM, MC_STOP_MBY));
		mpeg2_abort_hw(ctx);
		return ret;
	}
	if (mt8365_mpeg2_diag >= 2)
		mtk_v4l2_vdec_err(ctx,
			"mt8365 mpeg2 DONE    f%u %ux%u pct=%u ps=%u bs=%pad+%x sz=%zx | SYS[0x00]=%08x MISC[0xa4]=%08x [0x04]=%08x | LD[0xe8]=%08x [0xf4]=%08x | MC[0x28]=%08x [0x2c]=%08x",
			inst->frm_cnt, pic_w, pic_h, pct, pstruct,
			&bs->dma_addr, off, bs->size,
			rd(ctx, VDEC_SYS, SYS_CKEN_SET),
			rd(ctx, VDEC_MISC, MISC_IRQ_CFG),
			rd(ctx, VDEC_MISC, MISC_STATE),
			rd(ctx, VDEC_LD, VLD_FETCH_DONE),
			rd(ctx, VDEC_LD, VLD_IDLE),
			rd(ctx, VDEC_CM, MC_STOP_MBX),
			rd(ctx, VDEC_CM, MC_STOP_MBY));
	if ((ctx->irq_status & MTK_VDEC_IRQ_STATUS_DEC_SUCCESS) !=
	    MTK_VDEC_IRQ_STATUS_DEC_SUCCESS) {
		mpeg2_abort_hw(ctx);
		return -EIO;
	}

	/*
	 * The vendor's post-IRQ tail: re-apply the PDN words, pulse SYS+0x0 and
	 * read the MISC 0x08..0x24 run + MC+0x28/0x2c.  Reproduced because the
	 * H.264 backend's own experience is that the vendor's tail is what lets
	 * the recon write DMA retire before the core gates the VDEC clocks.
	 */
	wr(ctx, VDEC_MISC, MISC_SPEC_CON, 0x0000fffe);
	wr(ctx, VDEC_MISC, MISC_FUNC_CON, 0x7f6a11df);
	wr(ctx, VDEC_MISC, MISC_CFG178,   0xfffffffb);
	wr(ctx, VDEC_SYS,  SYS_CKEN_SET,  0x1);
	{
		int i;

		for (i = 0; i < 8; i++)
			(void)rd(ctx, VDEC_MISC, MISC_CRC_BASE + 4 * i);
	}
	(void)rd(ctx, VDEC_CM, MC_STOP_MBX);
	(void)rd(ctx, VDEC_CM, MC_STOP_MBY);
	mpeg2_release_hw(ctx);

	if (mt8365_mpeg2_diag)
		mtk_v4l2_vdec_err(ctx,
			"mt8365 mpeg2 f%u %ux%u pct=%u ps=%u 88=%08x 90=%08x t70=%08x mbw=%u cur=%08x/%08x fwd=%08x/%08x bwd=%08x/%08x qm=%d off=%u sz=%zx drain=%d CRC=%08x %08x",
			inst->frm_cnt, pic_w, pic_h, pct, pstruct,
			r_88, r_90, r_top70, mb_w, cur_y, cur_c,
			fwd_y, fwd_c, bwd_y, bwd_c, load_qm, off,
			bs->size, mt8365_mpeg2_drain,
			rd(ctx, VDEC_MISC, MISC_CRC_BASE),
			rd(ctx, VDEC_MISC, MISC_CRC_BASE + 4));

	if (!is_b) {
		inst->last_y = cur_y;
		inst->last_c = cur_c;
		inst->have_last = true;
	}
	inst->frm_cnt++;
	return 0;
}

static int vdec_mpeg2_mt8365_get_param(void *h_vdec,
				       enum vdec_get_param_type type, void *out)
{
	struct vdec_mpeg2_mt8365_inst *inst = h_vdec;
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;

	switch (type) {
	case GET_PARAM_PIC_INFO:
		/* MM21 at the CODED TILE size, the same rule the mt8365 H.264
		 * and VP8 backends use: the HW writes the recon planes densely
		 * at ALIGN(pic_w,16) x ALIGN(pic_h,32) with UV packed
		 * immediately after Y.  Declaring ALIGN(.,64) lies to userspace
		 * and collapses an otherwise exact decode. */
		ctx->picinfo.buf_w = ALIGN(ctx->picinfo.pic_w, 16);
		ctx->picinfo.buf_h = ALIGN(ctx->picinfo.pic_h, 32);
		ctx->picinfo.fb_sz[0] = ctx->picinfo.buf_w * ctx->picinfo.buf_h;
		ctx->picinfo.fb_sz[1] = ctx->picinfo.fb_sz[0] >> 1;
		*(struct vdec_pic_info *)out = ctx->picinfo;
		break;
	case GET_PARAM_DPB_SIZE:
		*(unsigned int *)out = MPEG2_DPB_SIZE;
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

static void vdec_mpeg2_mt8365_deinit(void *h_vdec)
{
	kfree(h_vdec);
}

const struct vdec_common_if vdec_mpeg2_mt8365_if = {
	.init		= vdec_mpeg2_mt8365_init,
	.decode		= vdec_mpeg2_mt8365_decode,
	.get_param	= vdec_mpeg2_mt8365_get_param,
	.deinit		= vdec_mpeg2_mt8365_deinit,
};
