// SPDX-License-Identifier: GPL-2.0
/*
 * MT8365 (Genio 350) stateless VP8 decode backend -- DIRECT REGISTER, no SCP,
 * no codec firmware, no vpud.
 *
 * Sibling of vdec_h264_mt8365_if.c. MT8365 has no codec coprocessor: the VDEC is
 * a stateless accelerator historically driven by the closed userspace
 * libvpud_vcodec.so. This backend replaces "send to VPU/SCP + wait IPI" with
 * "program the VDEC registers + kick + poll".
 *
 * TWO independent sources, both open / reproducible:
 *
 *  1. The mainline GPL MT8173 backend drivers/.../vdec/vdec_vp8_if.c programs the
 *     SAME blocks at the SAME offsets (VP8_HW_VLD_ADDR 0x93C / VP8_HW_VLD_VALUE
 *     0x940 segment SRAM, VP8_BSASET 0x100 / VP8_BSDSET 0x104 dec-table with
 *     VP8_DEC_TABLE_PROC_LOOP 96, VP8_RW_VP8_CTRL 0xA4, MISC 0x84/0xC8/0xCC/0xEC/
 *     0xF4, VLD soft-reset 0x108, vdecsys 0x0/0x18). Only the magic values differ.
 *     enable_hw_rw_function() / write_hw_segmentation_data() / store_dec_table()
 *     below are ports of those functions.
 *  2. A reads-inclusive live vendor register trace of a 176x144 VP8 keyframe
 *     decode on med-mt8365-pumpkin (2026-07-25, ~/bftrace/trace-vp8.log, 5389
 *     lines) supplies the mt8365-specific values and the half mainline delegates
 *     to VPU firmware: the frame-header parse and the decode kick.
 *
 * Sub-block -> reg_base[] index (enum mtk_hw_reg_idx), mt8365.dtsi codec@16000000:
 *   VDEC_SYS 0x16000000  VDEC_MISC 0x16025000  VDEC_LD  0x16020000
 *   VDEC_TOP 0x16020800  VDEC_CM   0x16021000  VDEC_HWD 0x16027800 (VP8_VLD)
 *   VDEC_HWB 0x16026800 (VP8_VLD2)
 * No DT change is needed -- VDEC_HWD/VDEC_HWB are already mapped, and the larb3
 * VLD_EXT / VLD2_EXT IOMMU ports are already listed on the node.
 *
 * KEY RE FINDING (trace seq 2301-2390): the VP8 frame header is parsed by
 * READING the HW barrel shifter through width-indexed ports
 *
 *	VDEC_HWD + 0x80 + (nbits << 2)   -> read nbits literal bits (prob 128)
 *
 * exactly like the H.264 AVCVLD "AVCVLD[nbits<<2]" convention this project
 * already relies on. Decoding the traced read stream against the VP8 spec
 * reproduced the bitstream byte-for-byte (4 x L(1) = colour_space / clamping /
 * segmentation_enabled / filter_type all 0, L(6) loop_filter_level = 0, L(3)
 * sharpness = 0, then loop-filter deltas {2,0,-2,-2} / {4,-2,2,4} -- the libvpx
 * defaults -- then L(2) partitions = 0, L(7) q_index = 4, five absent quant
 * deltas, refresh_entropy = 1). Because the read CONSUMES the bits, the parse is
 * simultaneously what leaves the HW's internal state at slice/MB data, so it is
 * mandatory, not observational: this backend must drive the same reads.
 *
 * INTER (P) FRAMES -- added from a 10-frame reads-inclusive vendor trace of the
 * same 176x144 clip (2026-07-25, ~/bftrace/trace-vp8-10f.log, 54032 lines). The
 * three things an inter frame needs beyond the keyframe path, all trace-derived:
 *
 *  a) Header parse: after the 3-byte frame tag (vs the keyframe's 10) the RFC
 *     6386 section 9.7/9.8 interframe fields are read through the SAME barrel
 *     shifter -- refresh_golden / refresh_alternate / copy_buffer_to_{golden,
 *     alternate} L(2) / sign_bias_{golden,alternate} / refresh_entropy_probs /
 *     refresh_last -- and later prob_intra / prob_last / prob_gf L(8) plus the
 *     y/uv mode prob update flags. The COEFFICIENT and MV prob updates are
 *     bool-coded at non-128 probabilities and are NOT read by software at all:
 *     the HW applies them itself when the driver pulses HWD+0x148 (coefficients)
 *     and HWD+0x14c (mode/MV). That is why neither appears in the vendor trace.
 *
 *  b) Entropy-context persistence: MEASURED -- every frame's dec-table UPLOAD is
 *     byte-identical to the PREVIOUS frame's post-decode READBACK (all 288 words,
 *     9/9 inter frames). The same holds for the y-mode / uv-mode / MV-prob SRAM,
 *     which the vendor reads back in the inter post-decode tail (HWD+0x108 index
 *     + 0x10c data, plus the 0x120 short/sign word: frame 1 uploads the default
 *     0x8080a2a4 and frames 2..9 upload 0x8080a216, the value the HW left after
 *     frame 1). A keyframe resets both contexts to the spec defaults.
 *
 *  c) Reference wiring: MC+0x3e0/0x3e4/0x3e8 (last/golden/altref Y) and
 *     MC+0xb64/0xb68/0xb6c (C) take full physical addresses. The buffers are
 *     resolved from the V4L2_CID_STATELESS_VP8_FRAME control's last/golden/
 *     alt_frame_ts through vb2_find_buffer(), exactly as the mainline stateless
 *     backend vdec_vp8_req_if.c does -- so refresh_/copy_buffer_to_ rotation is
 *     handled by the stateless API contract rather than re-derived here. A
 *     driver-side fallback (last decoded recon) covers a missing control.
 *
 * Inter-only registers, all absent from a keyframe (trace set-diff f0 vs f1):
 *   MC+0x238 = 3, MV(VDEC_AV)+0x2d4 = sign-bias word, HWD+0x14c mode/MV commit,
 *   HWD+0xa8 bit0 = 1 (non-key), and the PP loop-filter block +0x20/0x24/0x28/
 *   0x2c (a keyframe with loop_filter_level == 0 writes no PP register at all).
 *
 * SCOPE (this revision): 1, 2, 4 or 8 DCT token partitions -- the full RFC 6386
 * section 9.5 range.
 */

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

/* ---- VDEC_SYS (GCON) ---- */
#define SYS_CKEN_SET		0x00
#define SYS_DCM_CON		0x18
/* ---- VDEC_MISC ---- */
#define MISC_CRC_BASE		0x08	/* 0x08..0x24 CRC readback */
#define MISC_STATE		0x04	/* 1 = configured, 3 = armed */
#define MISC_SYS_SEL		0x84
#define MISC_SPEC_CON		0xc8
#define MISC_FUNC_CON		0xcc
#define MISC_DCM_CON		0xec
#define MISC_SRST		0xf4
#define MISC_MISC_F0		0xf0
#define MISC_CFG178		0x178
#define MISC_INPUT_BASE		0x160
/* ---- VDEC_LD (common VLD) / VDEC_HWB (VP8_VLD2) share this arm layout ---- */
#define VLD_FETCH_BASE_HI	0xb0	/* raw PA of the first byte to fetch */
#define VLD_WIN_START		0xb4	/* window start PA >> 6 */
#define VLD_WIN_END		0xb8	/* window end   PA >> 6 */
#define VLD_KICK		0x8c	/* bit20 start fetch, bit23 release */
#define VLD_FETCH_DONE		0xe8	/* bit0 */
#define VLD_IDLE		0xf4
#define VLD_SRST		0x108
#define VLD_WIN_END_RAW		0x110
#define VLD_DEQ_ADDR		0x260	/* dequant SRAM port pair (VDEC_LD) */
#define VLD_DEQ_DATA		0x264
/* ---- VDEC_TOP ---- */
#define TOP_SEGID_DRAM_ADDR	0x3c	/* mainline VP8_SEGID_DRAM_ADDR */
#define TOP_CFG5C		0x5c
#define TOP_PICSIZE_MB		0x68	/* ((mb_h-1)<<16)|(mb_w-1) */
/* ---- VDEC_CM (MC) ---- */
#define MC_SEG_ADDR		0x93c	/* mainline VP8_HW_VLD_ADDR */
#define MC_SEG_VALUE		0x940	/* mainline VP8_HW_VLD_VALUE */
#define MC_PIC_W_PX		0x208
#define MC_PIC_H_PX		0x20c
#define MC_ENABLE220		0x220
#define MC_RECON_Y		0x224	/* recon Y >> 9 */
#define MC_RECON_C		0x228	/* recon C >> 8 */
#define MC_MB_W			0x22c
#define MC_ENABLE250		0x250
#define MC_MB_W_M1		0x260
#define MC_MB_H_M1		0x264
#define MC_CFG57C		0x57c
#define MC_REF_Y		0x3e0	/* last / golden / altref Y (full phys) */
#define MC_REF_C		0xb64	/* last / golden / altref C (full phys) */
#define MC_RECON_Y4		0x850	/* recon Y >> 4 */
#define MC_RECON_C4		0x854	/* recon C >> 4 */
#define MC_RECON_EN		0x858
#define MC_LUMA_SZ		0x934
#define MC_MB_W2		0x980
#define MC_MC_FILTER		0x97c	/* bit4 six-tap luma, bit8 full-pixel MV */
#define MC_CFG920		0x920
#define MC_WORKBUF_A		0xa20
#define MC_WORKBUF_B		0xa24
#define MC_CFGB5C		0xb5c
#define MC_INTER_EN		0x238	/* = 3 on every inter frame, absent on a keyframe */
/* ---- VDEC_AV (VP8MV) ---- */
#define MV_SIGN_BIAS		0x2d4	/* golden/altref sign bias, inter only */
/* ---- VDEC_PP (VP8PP) -- loop filter; written only when lf level > 0 ---- */
#define PP_LF_CTRL		0x20
#define PP_LF_LEVEL		0x24	/* per-segment level, one byte each */
#define PP_REF_LF_DELTA		0x28	/* 4 signed bytes */
#define PP_MODE_LF_DELTA	0x2c	/* 4 signed bytes */
/* ---- VDEC_HWD (VP8_VLD) ---- */
#define HWD_PEEK		0x000	/* +0x20*n : peek/consume n bytes */
#define HWD_BITS		0x080	/* +0x4*n  : read n-bit literal */
#define HWD_CTRL		0x0a4	/* mainline VP8_RW_VP8_CTRL */
#define HWD_PART1_END		0x0a8	/* (part2_off<<8)|(version<<1)|!keyframe */
#define HWD_PROBS		0x0b0	/* skip | intra<<8 | last<<16 | gf<<24 */
#define HWD_KICK		0x0b8	/* pulse 1 -> 0 : DECODE GO */
/*
 * skip<<16 | log2_nbr_of_DCT_partitions<<12 | seg_upd_map<<8 | seg_en<<4.
 * The partition-count field is MEASURED: three matched 176x144 q=4 keyframes
 * encoded from the same frame differing ONLY in token-partition count wrote
 * 0x00010000 (1 partition), 0x00011000 (2) and 0x00013000 (8) here, with the
 * rest of the 169-write frame identical modulo bitstream/DMA addresses.
 */
#define HWD_SKIP_EN		0x0ac
/*
 * Mode/MV probability SRAM index 0 holds the three RFC 6386 section 9.3
 * mb_segment_tree_probs, packed little-endian (byte0 = probs[0]). MEASURED:
 * the vendor writes it -- and ONLY it, plus the two HWD_SKIP_EN bits above --
 * for a segmented stream, at the exact point in the header parse where the
 * three probabilities have just been read (vp80-03-segmentation-01 keyframe
 * 240/190/255 -> 0x00ffbef0; -1401/-1403 all-default -> 0x00ffffff). It is
 * absent from every frame of the non-segmented 10-frame trace.
 */
#define HWD_SEG_TREE_IDX	0x00
/*
 * Token-partition window table: EIGHT base registers at 0x0c0 + 4*i and eight
 * size registers at 0x0e0 + 4*i, one pair per DCT partition (RFC 6386 allows
 * 1/2/4/8). MEASURED on the 8-partition matched clip: the vendor writes
 * 0xc0..0xdc and 0xe0..0xfc, and the values are exactly the per-partition
 * absolute base and byte count derived from the 3-byte-per-entry size table.
 */
#define HWD_PART2_BASE		0x0c0
#define HWD_PART2_SIZE		0x0e0
#define HWD_BSASET		0x100	/* mainline VP8_BSASET  (dec-table addr) */
#define HWD_BSDSET		0x104	/* mainline VP8_BSDSET  (dec-table data) */
#define HWD_MVP_ADDR		0x108	/* MV-probability SRAM port pair */
#define HWD_MVP_DATA		0x10c
#define HWD_STATE110		0x110
#define HWD_MVP_SHORT		0x120
#define HWD_BSHIFT_INIT		0x144	/* 1 = stream0, 0x100 = stream1 */
#define HWD_COMMIT		0x148	/* apply the in-header coefficient updates */
#define HWD_MODE_COMMIT		0x14c	/* apply the in-header mode/MV updates (inter) */
#define HWD_STATUS15C		0x15c
#define HWD_DONE		0x174	/* bit0 = picture done */
#define HWD_PEEK2		0x200	/* +0x20*n : second stream peek/consume */
#define HWD_PEEK2_TAKE1		0x220	/* consume 1 byte, return the new window */
#define HWD_PEEK2_TAKE3		0x260	/* consume 3 bytes, return the new window */

/*
 * Temporary bring-up knob for the inter-frame path (see the audit): a bitmask
 * so a hypothesis can be A/B'd on the board without a rebuild.
 *   bit0 skip the PP loop-filter block   bit1 mask the dec-table readback to 24
 *   bit2 force all three refs to "last"  bit3 gate MC+0x238 on !keyframe only
 *   bit9 (0x200) restore the OLD PP+0x20 encoding (bit4 = filter_type,
 *        bit8 = loop_filter_adj_enable) instead of the one-hot filter-mode
 *        field measured on vp80-00-comprehensive-003 -- see pp_lf_ctrl()
 */
static int mt8365_vp8_quirk;
module_param(mt8365_vp8_quirk, int, 0644);
MODULE_PARM_DESC(mt8365_vp8_quirk, "mt8365 VP8 inter-frame bring-up quirk mask");

/*
 * Per-frame decode-parameter dump (register write-set summary + HW CRC), the
 * oracle used to diff a decode against the vendor trace. It is emitted at _err
 * level so it survives without dynamic-debug, which means it must be OFF by
 * default -- a normal VP8 decode has to be quiet.
 */
static bool mt8365_vp8_diag;
module_param(mt8365_vp8_diag, bool, 0644);
MODULE_PARM_DESC(mt8365_vp8_diag, "mt8365 VP8 per-frame diagnostic dump");

/*
 * Bit position of the 3-bit RFC 6386 sharpness_level inside the VDEC_PP+0x20
 * loop-filter control word. The vendor trace cannot place the field (all its
 * frames have sharpness_level 0), so it was MEASURED by sweeping this
 * parameter over 0..13,16,20 against the first two frames of the fluster
 * vector vp80-05-sharpness-1428 (f0 KEY sharpness 0, f1 INTER sharpness 3):
 * every shift except 3, 4 and 16 produced one identical output, and only
 * shift 16 made f1 bit-exact (psnr inf; shift 4 = the old inferred value gave
 * 50.22 dB, shift 5 gave 56.40 dB). 16 is the default; the parameter is kept
 * so the field WIDTH (assumed 3 bits, 16..18, per RFC 6386) can be re-swept
 * once a sharpness_level > 3 vector is characterised.
 */
static int mt8365_vp8_sharp_shift = 16;
module_param(mt8365_vp8_sharp_shift, int, 0644);
MODULE_PARM_DESC(mt8365_vp8_sharp_shift,
		 "mt8365 VP8 PP+0x20 sharpness_level bit position (default 4)");

/*
 * Compose the VDEC_PP+0x20 loop-filter control word.
 *
 * MEASURED. The three vendor traces that were used to place bit4 and bit8
 * confound filter_type with loop_filter_adj_enable -- every traced frame is
 * either (normal, adj) or (simple, no-adj), never (simple, adj):
 *
 *   clip / frame                    filter_type lf_adj key  PP+0x20
 *   vp8-176x144 + -1403 inter                 0      1   no  0x1101
 *   vp80-03-segmentation-1401 KEY             0      1  yes  0x0101
 *   vp80-03-segmentation-01   KEY             1      0  yes  0x0011
 *
 * The old reading (bit4 = filter_type, bit8 = loop_filter_adj_enable) fits all
 * three, but so does a ONE-HOT filter-mode field: bit4 = run the SIMPLE filter,
 * bit8 = run the NORMAL filter. vp80-00-comprehensive-003 (KEY, simple,
 * lf_level 6, loop_filter_adj_enable 1) separates them -- the old reading emits
 * 0x0111, i.e. BOTH filter-mode bits, and the decode comes out Y bit-exact with
 * chroma-only error (U 43.39 / V 46.43 dB), exactly what running the normal
 * filter's chroma pass under a simple-filter frame produces (RFC 6386 section
 * 15.2: the simple filter is LUMA-ONLY). With the whole PP block gated off
 * (quirk bit0) the same frame's chroma is bit-exact (U/V inf) and Y drops to
 * 40.84 dB, proving the chroma error is filter-applied-to-chroma and not a
 * recon defect.
 *
 * loop_filter_adj_enable has its own measured home in MC+0x238 bit0
 * (= 0x2 | lf_adj), so it does not need a second copy here.
 *
 * quirk bit9 selects the one-hot encoding for A/B on the board.
 *
 * The one-hot reading is itself FALSIFIED by measurement: emitting bit4 alone
 * for that frame costs luma its bit-exactness (Y inf -> 41.93 dB) while chroma
 * stays wrong (U 47.29 / V 48.59), so bit8 is load-bearing for the filter
 * itself and bit4 alone is not "run the simple filter". The default therefore
 * stays the old (filter_type, lf_adj) encoding; quirk bit9 selects the one-hot
 * variant, and lfctrl_or / lfctrl_clr make the whole word sweepable on the
 * board so a chroma-suppress bit can be hunted without a rebuild.
 */
static int mt8365_vp8_lfctrl_or;
module_param(mt8365_vp8_lfctrl_or, int, 0644);
MODULE_PARM_DESC(mt8365_vp8_lfctrl_or,
		 "mt8365 VP8 bits to OR into VDEC_PP+0x20");

static int mt8365_vp8_lfctrl_clr;
module_param(mt8365_vp8_lfctrl_clr, int, 0644);
MODULE_PARM_DESC(mt8365_vp8_lfctrl_clr,
		 "mt8365 VP8 bits to clear from VDEC_PP+0x20");

/*
 * MC+0x238 (= 0x2 | loop_filter_adj_enable) has the SAME confound as
 * PP+0x20 bit8: every vendor-traced frame is either (normal filter,
 * lf_adj) -> 0x3 or (simple filter, no lf_adj) -> 0x2, so bit0 could be
 * loop_filter_adj_enable or "run the chroma loop-filter pass" (which the
 * normal filter needs and the simple filter must not have). Sweepable so the
 * two readings can be separated on vp80-00-comprehensive-003, the only
 * available (simple, lf_adj) frame.
 */
static int mt8365_vp8_mc238_or;
module_param(mt8365_vp8_mc238_or, int, 0644);
MODULE_PARM_DESC(mt8365_vp8_mc238_or, "mt8365 VP8 bits to OR into MC+0x238");

static int mt8365_vp8_mc238_clr;
module_param(mt8365_vp8_mc238_clr, int, 0644);
MODULE_PARM_DESC(mt8365_vp8_mc238_clr, "mt8365 VP8 bits to clear from MC+0x238");

static u32 pp_lf_ctrl(bool keyframe, bool lf_type, bool lf_adj, u32 lf_sharp)
{
	u32 w = 0x1 | (keyframe ? 0u : 0x1000u) |
		((lf_sharp & 0x7) << (mt8365_vp8_sharp_shift & 31));

	if (mt8365_vp8_quirk & 0x200)
		w |= lf_type ? 0x10u : 0x100u;
	else
		w |= (lf_type ? 0x10u : 0u) | (lf_adj ? 0x100u : 0u);
	return (w | (u32)mt8365_vp8_lfctrl_or) & ~(u32)mt8365_vp8_lfctrl_clr;
}

#define VP8_DEC_TABLE_PROC_LOOP	96
#define VP8_DEC_TABLE_UNIT	3
#define VP8_HW_SEGMENT_DATA_SZ	272
#define VP8_HW_SEGMENT_UINT	4
#define VP8_WORK_BUF_SZ		(45 * 4096)
#define VP8_DPB_SIZE		4
#define MT8365_VP8_TIMEOUT	200000

/*
 * VP8 default coefficient probabilities, in the exact 96 x 3-word SRAM layout
 * the HW dec-table port expects (11 bytes per (block-type, band, ctx) group,
 * 4*8*3 = 96 groups, one zero pad byte per group). Transcribed verbatim from the
 * vendor keyframe trace (trace-vp8.log seq 2391-2774, VDEC_HWD+0x104 write
 * stream) and spot-verified against the VP8 spec defaults: group 0-2 are all
 * 128, group 3 is {253,136,254,255,228,219,128,128,128,128,128} ==
 * vp8_default_coef_probs[0][1][0]. A keyframe always resets the coefficient
 * probabilities to these defaults; any per-frame update in the header is applied
 * by the HW itself (proven: the post-decode readback at seq 2818-3201 differs
 * from this upload).
 */
static const u32 vp8_default_dec_table[VP8_DEC_TABLE_PROC_LOOP * VP8_DEC_TABLE_UNIT] = {
	0x80808080, 0x80808080, 0x00808080, 0x80808080, 0x80808080, 0x00808080,
	0x80808080, 0x80808080, 0x00808080, 0xfffe88fd, 0x8080dbe4, 0x00808080,
	0xfff281bd, 0xdbffd5e3, 0x00808080, 0xfce37e6a, 0xffffd1d6, 0x00808080,
	0xfff86201, 0xffffe2ec, 0x00808080, 0xfeee85b5, 0x9affeadd, 0x00808080,
	0xf7ca864e, 0xdbffb4c6, 0x00808080, 0xfff9b901, 0x8080fff3, 0x00808080,
	0xfff796b8, 0x8080e0ec, 0x00808080, 0xffd86e4d, 0x8080e6ec, 0x00808080,
	0xfffb6501, 0x8080fff1, 0x00808080, 0xfcf18baa, 0xffffd1ec, 0x00808080,
	0xf3c47425, 0xffffffe4, 0x00808080, 0xfffecc01, 0x8080fff5, 0x00808080,
	0xfffaa0cf, 0x808080ee, 0x00808080, 0xffe76766, 0x8080abd3, 0x00808080,
	0xfffc9801, 0x8080fff0, 0x00808080, 0xfff387b1, 0x8080e1ea, 0x00808080,
	0xffd38150, 0x8080e0c2, 0x00808080, 0x80ff0101, 0x80808080, 0x00808080,
	0x80ff01f6, 0x80808080, 0x00808080, 0x808080ff, 0x80808080, 0x00808080,
	0xdfed23c6, 0xa0a2bbc1, 0x003e9b91, 0xddc62d83, 0x9ddcb0ac, 0x0001ddfc,
	0xd0922f44, 0xa2dda795, 0x0080dfff, 0xfff19501, 0xffffe0dd, 0x00808080,
	0xfdea8db8, 0xc7ffdcde, 0x00808080, 0xf2b56351, 0xcaf9beb0, 0x0080ffff,
	0xfde88101, 0xc4f2c5d6, 0x0080ffff, 0xfad27963, 0xcaffc6c9, 0x00808080,
	0xf2a35b17, 0xd2f7bbaa, 0x0080ffff, 0xfff6c801, 0x8080ffea, 0x00808080,
	0xfff1b26d, 0xfffff5e7, 0x00808080, 0xfdc9822c, 0xffffc0cd, 0x00808080,
	0xfbef8401, 0xa5ffd1db, 0x00808080, 0xfbe1885e, 0xffffbeda, 0x00808080,
	0xf5ae6416, 0xc7ffa1ba, 0x00808080, 0xfff9b601, 0x8080ebe8, 0x00808080,
	0xfff18f7c, 0x8080eae3, 0x00808080, 0xfbb54d23, 0xcdffd3c1, 0x00808080,
	0xfff79d01, 0xffffe7ec, 0x00808080, 0xffeb8d79, 0xffffe3e1, 0x00808080,
	0xfbbc632d, 0xe0ffd9c3, 0x00808080, 0xfffb0101, 0x8080ffd5, 0x00808080,
	0xfff801cb, 0x808080ff, 0x00808080, 0xffb10189, 0x8080ffe0, 0x00808080,
	0xfbf809fd, 0xc0ffd0cf, 0x00808080, 0xf3e00daf, 0xc6f9b9c1, 0x0080ffff,
	0xddab1149, 0xa7ecb3a1, 0x0080eaff, 0xfdf75f01, 0xffffb7d4, 0x00808080,
	0xfaf45aef, 0xffffd1d3, 0x00808080, 0xf8c34d9b, 0xffffc3bc, 0x00808080,
	0xfbef1801, 0xcdffdbda, 0x00808080, 0xffdb33c9, 0x8080bac4, 0x00808080,
	0xefbe2e45, 0xe4ffdac9, 0x00808080, 0xfffbbf01, 0x808080ff, 0x00808080,
	0xfff9a5df, 0x8080ffd5, 0x00808080, 0xfff87c8d, 0x808080ff, 0x00808080,
	0xfff81001, 0x808080ff, 0x00808080, 0xffe624be, 0x8080ffec, 0x00808080,
	0x80ff0195, 0x80808080, 0x00808080, 0x80ffe201, 0x80808080, 0x00808080,
	0x80ffc0f7, 0x80808080, 0x00808080, 0x80ff80f0, 0x80808080, 0x00808080,
	0xfffc8601, 0x808080ff, 0x00808080, 0xfffa3ed5, 0x808080ff, 0x00808080,
	0x80ff5d37, 0x80808080, 0x00808080, 0x80808080, 0x80808080, 0x00808080,
	0x80808080, 0x80808080, 0x00808080, 0x80808080, 0x80808080, 0x00808080,
	0xebd518ca, 0xa0dcbfba, 0x00ffaff0, 0xe8b6267e, 0xaee4b8a9, 0x0080bbff,
	0xdb8a2e3d, 0xaaf0b297, 0x0080d8ff, 0xfae67001, 0x9ff7bfc7, 0x0080ffff,
	0xfce46da6, 0xaeffd7d3, 0x00808080, 0xe8a24d27, 0xb2f5b4ac, 0x0080ffff,
	0xf6dc3401, 0xdcf9c7c6, 0x0080ffff, 0xf3bf4a7c, 0xddfac1b7, 0x0080ffff,
	0xdb824718, 0xb6f3aa9a, 0x0080ffff, 0xf9e1b601, 0xe0fff0db, 0x00808080,
	0xfce29695, 0xabffcdd8, 0x00808080, 0xf2aa6c1c, 0xdffec2b7, 0x0080ffff,
	0xfce65101, 0xc0ffcbcc, 0x00808080, 0xf7d1667b, 0xe9ffc4bc, 0x00808080,
	0xf3995f14, 0xcbffada4, 0x00808080, 0xfff8de01, 0x8080d5d8, 0x00808080,
	0xfcf6afa8, 0xffffcdeb, 0x00808080, 0xffd7742f, 0xffffd4d3, 0x00808080,
	0xfdec7901, 0xffffd6d4, 0x00808080, 0xfcd5548d, 0xdbffcac9, 0x00808080,
	0xf0a0502a, 0xcdffb9a2, 0x00808080, 0x80ff0101, 0x80808080, 0x00808080,
	0x80ff01f4, 0x80808080, 0x00808080, 0x80ff01ee, 0x80808080, 0x00808080,
};

/*
 * VP8 default motion-vector probabilities in the HW's SRAM layout, transcribed
 * from the vendor keyframe trace (seq 2284-2299) and verified against the spec
 * vp8_default_mv_context: HWD+0x120 = {row is_short, row sign, col is_short,
 * col sign} = {162,128,164,128}; SRAM word 0x0c = row MVPshort probs
 * {225,146,172,147,214,39,156}; 0x10 = row MVPbits {128,129,132,75,145,178,206,
 * 239,254,254}; 0x14 = col MVPshort {204,170,119,235,140,230,228}; 0x18 = col
 * MVPbits {128,130,130,74,148,180,203,236,254,254}. A keyframe always uses the
 * defaults.
 */
#define VP8_MVP_SHORT_WORD	0x8080a2a4
#define VP8_MVP_NGROUP		4
static const struct { u32 addr; u32 n; } vp8_mv_prob_layout[VP8_MVP_NGROUP] = {
	{ 0x0c, 2 }, { 0x10, 3 }, { 0x14, 2 }, { 0x18, 3 },
};
static const u32 vp8_default_mv_probs[VP8_MVP_NGROUP][3] = {
	{ 0x93ac92e1, 0x009c27d6, 0 },
	{ 0x4b848180, 0xefceb291, 0x0000fefe },
	{ 0xeb77aacc, 0x00e4e68c, 0 },
	{ 0x4a828280, 0xeccbb494, 0x0000fefe },
};

/*
 * Intra-mode probabilities used on an INTER frame, in the HW SRAM word layout
 * (little-endian byte packing), at HWD MV/mode SRAM index 0x04 (y) and 0x08 (uv).
 * These are the spec defaults vp8_ymode_prob {112,86,140,37} and
 * vp8_uv_mode_prob {162,101,204}, transcribed from the vendor trace's frame-1
 * upload (seq 8152-8155). A keyframe uses the fixed kf_* tables inside the HW and
 * uploads neither.
 */
#define VP8_DEFAULT_YMODE_WORD	0x258c5670
#define VP8_DEFAULT_UVMODE_WORD	0x00cc65a2

/* VP8 spec dequantiser lookup tables (RFC 6386 section 14.1). */
static const u16 vp8_dc_qlookup[128] = {
	  4,   5,   6,   7,   8,   9,  10,  10,  11,  12,  13,  14,  15,  16,  17,  17,
	 18,  19,  20,  20,  21,  21,  22,  22,  23,  23,  24,  25,  25,  26,  27,  28,
	 29,  30,  31,  32,  33,  34,  35,  36,  37,  37,  38,  39,  40,  41,  42,  43,
	 44,  45,  46,  46,  47,  48,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,
	 59,  60,  61,  62,  63,  64,  65,  66,  67,  68,  69,  70,  71,  72,  73,  74,
	 75,  76,  76,  77,  78,  79,  80,  81,  82,  83,  84,  85,  86,  87,  88,  89,
	 91,  93,  95,  96,  98, 100, 101, 102, 104, 106, 108, 110, 112, 114, 116, 118,
	122, 124, 126, 128, 130, 132, 134, 136, 138, 140, 143, 145, 148, 151, 154, 157,
};

static const u16 vp8_ac_qlookup[128] = {
	  4,   5,   6,   7,   8,   9,  10,  11,  12,  13,  14,  15,  16,  17,  18,  19,
	 20,  21,  22,  23,  24,  25,  26,  27,  28,  29,  30,  31,  32,  33,  34,  35,
	 36,  37,  38,  39,  40,  41,  42,  43,  44,  45,  46,  47,  48,  49,  50,  51,
	 52,  53,  54,  55,  56,  57,  58,  60,  62,  64,  66,  68,  70,  72,  74,  76,
	 78,  80,  82,  84,  86,  88,  90,  92,  94,  96,  98, 100, 102, 104, 106, 108,
	110, 112, 114, 116, 119, 122, 125, 128, 131, 134, 137, 140, 143, 146, 149, 152,
	155, 158, 161, 164, 167, 170, 173, 177, 181, 185, 189, 193, 197, 201, 205, 209,
	213, 217, 221, 225, 229, 234, 239, 245, 249, 254, 259, 264, 269, 274, 279, 284,
};

/*
 * Persistent VP8 entropy + loop-filter context. VP8 carries these across frames
 * (RFC 6386 section 9.9/9.10); a keyframe resets them to the spec defaults.
 * MEASURED against the vendor: frame N's dec-table upload == frame N-1's
 * post-decode readback, for all 288 words and all 9 inter frames of the 10-frame
 * trace, and the same for the mode/MV SRAM.
 */
struct vp8_entropy_ctx {
	u32 coef[VP8_DEC_TABLE_PROC_LOOP * VP8_DEC_TABLE_UNIT];
	u32 ymode;
	u32 uvmode;
	u32 mv_short;
	u32 mv[VP8_MVP_NGROUP][3];
};

struct vdec_vp8_mt8365_inst {
	struct mtk_vcodec_dec_ctx *ctx;
	struct mtk_vcodec_mem work_buf[2];
	struct mtk_vcodec_mem pair_buf;		/* quirk 0x20 vendor-strided pair */
	bool work_buf_ready;
	unsigned int frm_cnt;

	/* entropy context: live, plus the pre-frame snapshot that a frame with
	 * refresh_entropy_probs == 0 must restore. */
	struct vp8_entropy_ctx ent;
	struct vp8_entropy_ctx ent_saved;

	/* loop-filter deltas persist until a mode_ref_lf_delta_update */
	s8 ref_lf_delta[4];
	s8 mode_lf_delta[4];

	/* mb_segment_tree_probs persist until an update_mb_segmentation_map */
	u8 seg_tree_prob[3];

	/* geometry, taken from the last keyframe (an inter frame carries none) */
	u32 pic_w, pic_h;

	/*
	 * RFC 6386 section 9.3 segment feature data -- the per-segment
	 * quantizer / loop-filter values and the absolute-vs-delta mode. These
	 * PERSIST: they are re-read from the bitstream only when
	 * update_mb_segmentation_data is set, and a key frame resets them
	 * (libvpx init_frame(): memset(segment_feature_data) +
	 * mb_segement_abs_delta = SEGMENT_DELTADATA).
	 */
	int seg_qd[4];
	int seg_lfd[4];
	int seg_abs;

	/* fallback reference when the stateless control is unavailable */
	u32 last_y, last_c;
	bool have_last;
};

/*
 * Replicate the last valid row of a decoded MM21 plane into its padding rows.
 *
 * The recon plane is allocated at ALIGN(pic_h, 32) but the HW only writes the
 * picture rows (measured on the keyframe: exactly 176*16 = 2816 zero bytes, the
 * 16 padding rows of ALIGN(144,32) = 160). A VP8 motion vector may point below
 * the last picture row, and libvpx extends the reference border by replicating
 * the edge row -- so a reference whose padding is zero decodes those macroblocks
 * differently. Done in the TILED domain: MM21 tiles are 16 wide, @th tall, in
 * raster tile order, so byte offset(tile_row, in_tile_row, tile_col) =
 * tile_row*aw*th + tile_col*th*16 + in_tile_row*16.
 */
static void vp8_extend_bottom_border(u8 *va, u32 aw, u32 ah, u32 valid_h, u32 th)
{
	u32 tile_row = valid_h / th;
	u32 last = valid_h % th;
	u32 tiles = aw / 16;
	u32 c, r;

	if (!va || valid_h >= ah || !last)
		return;
	for (c = 0; c < tiles; c++) {
		u8 *base = va + tile_row * aw * th + c * th * 16;

		for (r = last; r < th; r++)
			memcpy(base + r * 16, base + (last - 1) * 16, 16);
	}
}

static u32 vp8_ctx_sum(const u32 *p)
{
	u32 s = 0;
	int i;

	for (i = 0; i < VP8_DEC_TABLE_PROC_LOOP * VP8_DEC_TABLE_UNIT; i++)
		s += p[i];
	return s;
}

static u32 vp8_ctx_xor(const u32 *p)
{
	u32 s = 0;
	int i;

	for (i = 0; i < VP8_DEC_TABLE_PROC_LOOP * VP8_DEC_TABLE_UNIT; i++)
		s ^= p[i];
	return s;
}

static void vp8_reset_entropy(struct vdec_vp8_mt8365_inst *inst)
{
	int i;

	memcpy(inst->ent.coef, vp8_default_dec_table, sizeof(inst->ent.coef));
	inst->ent.ymode = VP8_DEFAULT_YMODE_WORD;
	inst->ent.uvmode = VP8_DEFAULT_UVMODE_WORD;
	inst->ent.mv_short = VP8_MVP_SHORT_WORD;
	for (i = 0; i < VP8_MVP_NGROUP; i++)
		memcpy(inst->ent.mv[i], vp8_default_mv_probs[i],
		       sizeof(inst->ent.mv[i]));
}

/*
 * quirk 0x100: dump every register write, so the ORDERED write sequence can be
 * diffed against the vendor's complete MMIO capture (mmiotrace.so, which
 * fault-traces the whole 256 KiB SFR aperture and is proven to see every store).
 * The two bulk SRAM streams (MC segment-ID port, HWD dec-table port) are
 * excluded -- thousands of writes per frame, already proven identical by the
 * entropy-context checksum.
 */
static inline void wr(struct mtk_vcodec_dec_ctx *ctx, int blk, u32 off, u32 val)
{
	if (unlikely(mt8365_vp8_quirk & 0x100) &&
	    !(blk == VDEC_CM && (off == 0x93c || off == 0x940)) &&
	    !(blk == VDEC_HWD && (off == 0x100 || off == 0x104)))
		pr_info("vp8w b%d +%03x = %08x\n", blk, off, val);
	writel(val, ctx->dev->reg_base[blk] + off);
}

static inline u32 rd(struct mtk_vcodec_dec_ctx *ctx, int blk, u32 off)
{
	return readl(ctx->dev->reg_base[blk] + off);
}

/* Read @n literal bits (prob 128) through the VP8_VLD barrel shifter. The read
 * CONSUMES them -- this is how the frame header is parsed. */
static inline u32 hbits(struct mtk_vcodec_dec_ctx *ctx, int n)
{
	return rd(ctx, VDEC_HWD, HWD_BITS + (n << 2));
}

/* signed value with no presence flag: magnitude L(nbits), sign L(1) */
static int hdelta_val(struct mtk_vcodec_dec_ctx *ctx, int nbits)
{
	int v = (int)hbits(ctx, nbits);

	return hbits(ctx, 1) ? -v : v;
}

/* signed value: flag L(1), magnitude L(nbits), sign L(1) */
static int hdelta(struct mtk_vcodec_dec_ctx *ctx, int nbits)
{
	if (!hbits(ctx, 1))
		return 0;
	return hdelta_val(ctx, nbits);
}

static int clampq(int q)
{
	if (q < 0)
		return 0;
	if (q > 127)
		return 127;
	return q;
}

/*
 * Port of mainline vdec_vp8_if.c enable_hw_rw_function(), split per VLD block --
 * the mt8365 vendor stack runs this sequence TWICE per decode, once resetting
 * the common VLD (VDEC_LD) and once resetting VP8_VLD2 (VDEC_HWB); trace seq
 * 0-13 and 30-43. mt8365 values: MISC_SPEC_CON 0xff7f (mt8173 0x17F),
 * MISC_FUNC_CON 0x71201102 (mt8173 0x71201100), plus MISC+0x178 and the MC+0x920
 * bit24 set which mt8173 does not do.
 */
static void enable_hw_rw_function(struct mtk_vcodec_dec_ctx *ctx, int vld_blk)
{
	wr(ctx, VDEC_SYS, SYS_CKEN_SET, 0x1);
	wr(ctx, vld_blk, VLD_SRST, 0x101);
	wr(ctx, VDEC_SYS, SYS_CKEN_SET, 0x1);
	wr(ctx, VDEC_MISC, MISC_SRST, rd(ctx, VDEC_MISC, MISC_SRST) & ~0x1u);
	wr(ctx, VDEC_MISC, MISC_SYS_SEL, 0x1);
	wr(ctx, VDEC_MISC, MISC_SPEC_CON, 0x0000ff7f);
	wr(ctx, VDEC_MISC, MISC_FUNC_CON, 0x71201102);
	wr(ctx, VDEC_MISC, MISC_CFG178, 0xfffffffb);
	wr(ctx, VDEC_CM, MC_CFG920, rd(ctx, VDEC_CM, MC_CFG920) | 0x01000000);
	wr(ctx, vld_blk, VLD_SRST, 0x0);
	wr(ctx, VDEC_SYS, SYS_DCM_CON, 0x1);
	wr(ctx, VDEC_MISC, MISC_DCM_CON, 0x1);
	wr(ctx, VDEC_HWD, HWD_CTRL, 0x1);
	/* Vendor writes MISC+0x160 with this constant on every pass; it is NOT
	 * the bitstream address (the H.264 backend proves the register is not
	 * load-bearing for the fetch, which is driven by VLD+0xb0/0xb4/0xb8). */
	wr(ctx, VDEC_MISC, MISC_INPUT_BASE, 0x0065ff10);
}

/* Arm a VLD block's fetch window over [start, end) and run one fetch. */
static void vld_arm_fetch(struct mtk_vcodec_dec_ctx *ctx, int blk,
			  dma_addr_t win_start, dma_addr_t start, dma_addr_t end,
			  bool do_fetch)
{
	int i;

	wr(ctx, blk, VLD_WIN_START, (u32)(win_start >> 6));
	wr(ctx, blk, VLD_WIN_END, (u32)(end >> 6));
	if (!do_fetch)
		return;
	wr(ctx, blk, VLD_FETCH_BASE_HI, (u32)start);
	wr(ctx, blk, VLD_WIN_END_RAW, (u32)end);
	wr(ctx, blk, VLD_SRST, 0x100);
	wr(ctx, blk, VLD_SRST, 0x0);
	wr(ctx, blk, VLD_KICK, 0x00100000);
	for (i = 0; i < 4096; i++) {
		if (rd(ctx, blk, VLD_FETCH_DONE) & 0x1)
			break;
		cpu_relax();
	}
	wr(ctx, blk, VLD_KICK, 0x00800000);
}

/* Port of mainline write_hw_segmentation_data() -- the segment-id SRAM is
 * cleared through the MC address/value port pair. */
static void write_hw_segmentation_data(struct mtk_vcodec_dec_ctx *ctx)
{
	u32 seg_id_addr = rd(ctx, VDEC_TOP, TOP_SEGID_DRAM_ADDR) >> 4;
	int i, j;

	for (i = 0; i < VP8_HW_SEGMENT_DATA_SZ; i++)
		for (j = VP8_HW_SEGMENT_UINT - 1; j >= 0; j--) {
			wr(ctx, VDEC_CM, MC_SEG_ADDR,
			   (1 << 16) + ((seg_id_addr + i) << 2) + j);
			wr(ctx, VDEC_CM, MC_SEG_VALUE, 0);
		}
}

/* Port of mainline read_hw_segmentation_data() -- drain the segment-id SRAM
 * back out after the decode. Also part of the vendor's post-decode tail (trace
 * seq 3211+), which the HW needs: see the load_dec_table() note below. */
static void read_hw_segmentation_data(struct mtk_vcodec_dec_ctx *ctx)
{
	u32 seg_id_addr = rd(ctx, VDEC_TOP, TOP_SEGID_DRAM_ADDR) >> 4;
	int i, j;

	for (i = 0; i < VP8_HW_SEGMENT_DATA_SZ; i++)
		for (j = VP8_HW_SEGMENT_UINT - 1; j >= 0; j--) {
			wr(ctx, VDEC_CM, MC_SEG_ADDR, ((seg_id_addr + i) << 2) + j);
			(void)rd(ctx, VDEC_CM, MC_SEG_VALUE);
		}
}

/*
 * Port of mainline load_dec_table() -- read the (HW-updated) coefficient
 * probabilities back out. Two reasons this is not optional:
 *  - it is the entropy context an inter frame will need;
 *  - MEASURED: the vendor's post-decode tail (this + the CRC reads + the
 *    segment-SRAM readback, ~2500 register accesses) is also what lets the
 *    recon write DMA drain before the caller returns and the core cuts the
 *    VDEC clocks/power domain. Returning straight from the done-poll HARD-RESET
 *    the board (silent watchdog reset, nothing in the kernel log); replaying the
 *    vendor tail fixes it.
 */
static void load_dec_table(struct mtk_vcodec_dec_ctx *ctx, u32 *out)
{
	u32 addr = 0;
	int i, j;

	for (i = 0; i < VP8_DEC_TABLE_PROC_LOOP; i++) {
		wr(ctx, VDEC_HWD, HWD_BSASET, addr);
		for (j = 0; j < VP8_DEC_TABLE_UNIT; j++) {
			u32 v = rd(ctx, VDEC_HWD, HWD_BSDSET);

			/* mainline vdec_vp8_if.c masks the third word to 24 bits */
			if ((mt8365_vp8_quirk & 0x2) && j == VP8_DEC_TABLE_UNIT - 1)
				v &= 0xffffff;
			*out++ = v;
		}
		addr += 4;
	}
}

/*
 * Mode / MV probability SRAM (HWD+0x108 index, +0x10c data, +0x120 the
 * is_short/sign word). On an INTER frame the vendor uploads the persisted
 * context before pulsing HWD_MODE_COMMIT and reads the HW-updated context back
 * in the post-decode tail (trace seq 8152-8170 / 8599-8617). A keyframe uploads
 * only the MV half and reads nothing back.
 */
static void store_mode_mv_probs(struct mtk_vcodec_dec_ctx *ctx,
				const struct vp8_entropy_ctx *e, bool with_modes)
{
	int i, s;

	if (with_modes) {
		wr(ctx, VDEC_HWD, HWD_MVP_ADDR, 0x04);
		wr(ctx, VDEC_HWD, HWD_MVP_DATA, e->ymode);
		wr(ctx, VDEC_HWD, HWD_MVP_ADDR, 0x08);
		wr(ctx, VDEC_HWD, HWD_MVP_DATA, e->uvmode);
	}
	wr(ctx, VDEC_HWD, HWD_MVP_SHORT, e->mv_short);
	for (i = 0; i < VP8_MVP_NGROUP; i++) {
		wr(ctx, VDEC_HWD, HWD_MVP_ADDR, vp8_mv_prob_layout[i].addr);
		for (s = 0; s < vp8_mv_prob_layout[i].n; s++)
			wr(ctx, VDEC_HWD, HWD_MVP_DATA, e->mv[i][s]);
	}
}

static void load_mode_mv_probs(struct mtk_vcodec_dec_ctx *ctx,
			       struct vp8_entropy_ctx *e)
{
	int i, s;

	wr(ctx, VDEC_HWD, HWD_MVP_ADDR, 0x04);
	e->ymode = rd(ctx, VDEC_HWD, HWD_MVP_DATA);
	wr(ctx, VDEC_HWD, HWD_MVP_ADDR, 0x08);
	e->uvmode = rd(ctx, VDEC_HWD, HWD_MVP_DATA);
	e->mv_short = rd(ctx, VDEC_HWD, HWD_MVP_SHORT);
	for (i = 0; i < VP8_MVP_NGROUP; i++) {
		wr(ctx, VDEC_HWD, HWD_MVP_ADDR, vp8_mv_prob_layout[i].addr);
		for (s = 0; s < vp8_mv_prob_layout[i].n; s++)
			e->mv[i][s] = rd(ctx, VDEC_HWD, HWD_MVP_DATA);
	}
}

/* Port of mainline store_dec_table() -- upload the coefficient probabilities. */
static void store_dec_table(struct mtk_vcodec_dec_ctx *ctx, const u32 *p)
{
	u32 addr = 0;
	int i, j;

	for (i = 0; i < VP8_DEC_TABLE_PROC_LOOP; i++) {
		wr(ctx, VDEC_HWD, HWD_BSASET, addr);
		for (j = 0; j < VP8_DEC_TABLE_UNIT; j++)
			wr(ctx, VDEC_HWD, HWD_BSDSET, *p++);
		addr += 4;
	}
}

static void free_work_bufs(struct vdec_vp8_mt8365_inst *inst)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(inst->work_buf); i++)
		if (inst->work_buf[i].va)
			mtk_vcodec_mem_free(inst->ctx, &inst->work_buf[i]);
	if (inst->pair_buf.va)
		mtk_vcodec_mem_free(inst->ctx, &inst->pair_buf);
	inst->work_buf_ready = false;
}

static int alloc_work_bufs(struct vdec_vp8_mt8365_inst *inst)
{
	int i, err;

	for (i = 0; i < ARRAY_SIZE(inst->work_buf); i++) {
		inst->work_buf[i].size = VP8_WORK_BUF_SZ;
		err = mtk_vcodec_mem_alloc(inst->ctx, &inst->work_buf[i]);
		if (err) {
			free_work_bufs(inst);
			return err;
		}
	}
	inst->pair_buf.size = 2 * VP8_WORK_BUF_SZ;
	if (mtk_vcodec_mem_alloc(inst->ctx, &inst->pair_buf))
		inst->pair_buf.va = NULL;
	inst->work_buf_ready = true;
	return 0;
}

static int vdec_vp8_mt8365_init(struct mtk_vcodec_dec_ctx *ctx)
{
	struct vdec_vp8_mt8365_inst *inst;

	inst = kzalloc(sizeof(*inst), GFP_KERNEL);
	if (!inst)
		return -ENOMEM;
	inst->ctx = ctx;
	memset(inst->seg_tree_prob, 255, sizeof(inst->seg_tree_prob));
	vp8_reset_entropy(inst);
	ctx->drv_handle = inst;
	return 0;
}

/*
 * Resolve the last / golden / altref surfaces. Primary source is the stateless
 * API contract: V4L2_CID_STATELESS_VP8_FRAME carries last_frame_ts /
 * golden_frame_ts / alt_frame_ts, and both queues are TIMESTAMP_COPY, so
 * vb2_find_buffer() on the CAPTURE queue returns the buffer that held that
 * reference. Identical to mainline vdec_vp8_req_if.c. Userspace therefore owns
 * the refresh_/copy_buffer_to_ rotation; the driver only programs the result.
 * Falls back to "the previously decoded recon" for all three when the control is
 * absent (e.g. a client that feeds frames without the request API).
 */
static void vp8_resolve_refs(struct vdec_vp8_mt8365_inst *inst, bool keyframe,
			     u32 rec_y, u32 rec_c, u32 ref_y[3], u32 ref_c[3])
{
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	const struct v4l2_ctrl_vp8_frame *hdr = NULL;
	struct v4l2_ctrl *ctrl;
	struct vb2_queue *vq;
	u64 ts[3];
	int i;

	/* A keyframe references only itself (vendor trace f0: all six ref
	 * registers point at the current recon). */
	for (i = 0; i < 3; i++) {
		ref_y[i] = keyframe ? rec_y :
			   (inst->have_last ? inst->last_y : rec_y);
		ref_c[i] = keyframe ? rec_c :
			   (inst->have_last ? inst->last_c : rec_c);
	}
	if (keyframe)
		return;

	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, V4L2_CID_STATELESS_VP8_FRAME);
	if (ctrl)
		hdr = ctrl->p_cur.p;
	if (!hdr)
		return;

	ts[0] = hdr->last_frame_ts;
	ts[1] = hdr->golden_frame_ts;
	ts[2] = hdr->alt_frame_ts;
	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	for (i = 0; i < 3; i++) {
		struct vb2_buffer *vb = vb2_find_buffer(vq, ts[i]);

		if (!vb || vb->num_planes < 2)
			continue;
		ref_y[i] = (u32)vb2_dma_contig_plane_dma_addr(vb, 0);
		ref_c[i] = (u32)vb2_dma_contig_plane_dma_addr(vb, 1);
	}
}

static int vdec_vp8_mt8365_decode(void *h_vdec, struct mtk_vcodec_mem *bs,
				  struct vdec_fb *unused, bool *res_chg)
{
	struct vdec_vp8_mt8365_inst *inst = h_vdec;
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	struct vdec_fb *fb;
	const u8 *p;
	u32 tag, first_part_size, part2_off, pic_w, pic_h, mb_w, mb_h, luma_sz;
	u32 npeek, nparts, tbl, part_left;
	u32 rec_y, rec_c, q_index, seg_q[4] = { 0, 0, 0, 0 };
	u32 ref_y[3], ref_c[3];
	u32 lf_level, lf_sharp, lf_type, seg_lf[4], lf_word;
	u32 prob_skip = 0, prob_intra = 0, prob_last = 0, prob_gf = 0;
	dma_addr_t bs_end;
	bool keyframe, lf_adj = false, mb_no_coeff_skip;
	int refresh_entropy = 1;
	u32 version, mc_filter, seg_tree_word = 0;
	int seg_upd_map = 0, upd_data = 0;
	int i, s, seg_enabled = 0, seg_abs, log2_parts;
	int ydc_d, y2dc_d, y2ac_d, uvdc_d, uvac_d;
	int seg_qd[4], seg_lfd[4];
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

	if (!bs->va || bs->size < 3)
		return -EINVAL;
	p = bs->va;
	tag = p[0] | (p[1] << 8) | (p[2] << 16);
	keyframe = !(tag & 1);
	/* RFC 6386 section 9.1 version: selects the reconstruction + loop
	 * filter variant. MEASURED in HWD_PART1_END bits 3:1 -- the
	 * vp80-03-segmentation-01 keyframe is version 1 and the vendor writes
	 * (part2_off << 8) | 0x02, where every version-0 frame of every other
	 * trace writes | 0x00 (keyframe) or | 0x01 (inter). */
	version = (tag >> 1) & 0x7;
	first_part_size = (tag >> 5) & 0x7ffff;
	if (unlikely(mt8365_vp8_quirk & 0x100))
		pr_info("vp8w FRAME %u %s\n", inst->frm_cnt,
			keyframe ? "KEY" : "INTER");
	if (keyframe) {
		if (bs->size < 10) {
			mtk_v4l2_vdec_err(ctx, "mt8365 vp8: short keyframe");
			return -EINVAL;
		}
		if (p[3] != 0x9d || p[4] != 0x01 || p[5] != 0x2a) {
			mtk_v4l2_vdec_err(ctx, "mt8365 vp8: bad keyframe start code");
			return -EINVAL;
		}
		pic_w = (p[6] | (p[7] << 8)) & 0x3fff;
		pic_h = (p[8] | (p[9] << 8)) & 0x3fff;
		part2_off = 10 + first_part_size;
		inst->pic_w = pic_w;
		inst->pic_h = pic_h;
		/* A keyframe resets both entropy contexts (RFC 6386 section 9.9). */
		vp8_reset_entropy(inst);
		memset(inst->ref_lf_delta, 0, sizeof(inst->ref_lf_delta));
		memset(inst->mode_lf_delta, 0, sizeof(inst->mode_lf_delta));
		/* ... and the segment feature data, exactly as libvpx
		 * init_frame() does on a key frame. */
		memset(inst->seg_qd, 0, sizeof(inst->seg_qd));
		memset(inst->seg_lfd, 0, sizeof(inst->seg_lfd));
		inst->seg_abs = 0;
	} else {
		/* An inter frame carries no dimensions -- reuse the last keyframe's. */
		pic_w = inst->pic_w;
		pic_h = inst->pic_h;
		part2_off = 3 + first_part_size;
		if (!pic_w || !pic_h) {
			mtk_v4l2_vdec_err(ctx,
				"mt8365 vp8: inter frame before any keyframe");
			return -EINVAL;
		}
	}
	if (!pic_w || !pic_h || part2_off >= bs->size) {
		mtk_v4l2_vdec_err(ctx, "mt8365 vp8: bad header %ux%u p2=%u sz=%zx",
				  pic_w, pic_h, part2_off, bs->size);
		return -EINVAL;
	}
	mb_w = DIV_ROUND_UP(pic_w, 16);
	mb_h = DIV_ROUND_UP(pic_h, 16);
	luma_sz = ALIGN(pic_w, 16) * ALIGN(pic_h, 32);

	if (!inst->work_buf_ready) {
		ret = alloc_work_bufs(inst);
		if (ret)
			return ret;
	}

	/* N45: audit + barrier -- see mtk_vcodec_mt8365_bs_audit(). */
	mtk_vcodec_mt8365_bs_audit(ctx, bs->dma_addr, bs->size);

	rec_y = (u32)fb->base_y.dma_addr;
	rec_c = (u32)fb->base_c.dma_addr;
	/* Vendor fetch window ends 0x400 past the AU (trace: end - base - size
	 * == 0x400 exactly for the 0xe45-byte keyframe). */
	bs_end = bs->dma_addr + bs->size + 0x400;

	/* ---- trace seq 0-49: two HW-enable passes, bitstream fetch, seg SRAM ---- */
	enable_hw_rw_function(ctx, VDEC_LD);
	(void)rd(ctx, VDEC_LD, VLD_IDLE);
	vld_arm_fetch(ctx, VDEC_LD, bs->dma_addr, bs->dma_addr, bs_end, true);
	wr(ctx, VDEC_HWD, HWD_BSHIFT_INIT, 0x1);
	wr(ctx, VDEC_HWD, HWD_BSHIFT_INIT, 0x0);

	enable_hw_rw_function(ctx, VDEC_HWB);
	vld_arm_fetch(ctx, VDEC_HWB, bs->dma_addr, 0, bs_end, false);
	wr(ctx, VDEC_MISC, MISC_STATE, 0x1);
	write_hw_segmentation_data(ctx);

	/* ---- trace seq 2226-2274: repeat both enable passes ---- */
	enable_hw_rw_function(ctx, VDEC_LD);
	(void)rd(ctx, VDEC_LD, VLD_IDLE);
	vld_arm_fetch(ctx, VDEC_LD, bs->dma_addr, bs->dma_addr, bs_end, true);
	wr(ctx, VDEC_HWD, HWD_BSHIFT_INIT, 0x1);
	wr(ctx, VDEC_HWD, HWD_BSHIFT_INIT, 0x0);

	enable_hw_rw_function(ctx, VDEC_HWB);
	vld_arm_fetch(ctx, VDEC_HWB, bs->dma_addr, 0, bs_end, false);
	wr(ctx, VDEC_MISC, MISC_STATE, 0x1);

	/* ---- consume the uncompressed header through the byte-indexed
	 * peek/consume ports (+0x20 per byte): 10 bytes on a keyframe (trace seq
	 * 2275-2282), 3 bytes -- the frame tag alone -- on an inter frame (trace
	 * seq 7663-7664). ---- */
	(void)rd(ctx, VDEC_HWD, HWD_PEEK);
	(void)rd(ctx, VDEC_HWD, HWD_PEEK + 3 * 0x20);
	if (keyframe) {
		(void)rd(ctx, VDEC_HWD, HWD_PEEK);
		(void)rd(ctx, VDEC_HWD, HWD_PEEK + 3 * 0x20);
		(void)rd(ctx, VDEC_HWD, HWD_PEEK);
		(void)rd(ctx, VDEC_HWD, HWD_PEEK + 2 * 0x20);
		(void)rd(ctx, VDEC_HWD, HWD_PEEK);
		(void)rd(ctx, VDEC_HWD, HWD_PEEK + 2 * 0x20);
	}

	/* ---- seq 2283-2299 / 7665-7666: picture size, partition-1 end. The
	 * PART1_END low bit is the non-key flag (keyframe 0x00025500, inter
	 * 0x00006c01). A keyframe also uploads the default MV probabilities here;
	 * an inter frame uploads its persisted mode+MV context later, after the
	 * mode-prob update flags have been parsed. ---- */
	wr(ctx, VDEC_TOP, TOP_PICSIZE_MB, (mb_w - 1) | ((mb_h - 1) << 16));
	wr(ctx, VDEC_HWD, HWD_PART1_END,
	   (part2_off << 8) | (version << 1) | (keyframe ? 0 : 1));
	if (keyframe)
		store_mode_mv_probs(ctx, &inst->ent, false);
	(void)rd(ctx, VDEC_HWD, HWD_STATE110);

	/* ---- seq 2301-2330 / 7668-7673: frame header parse, driven through the
	 * barrel shifter (each read CONSUMES its bits). RFC 6386 section 9.2-9.4.
	 * colour_space/clamping_type exist on a keyframe only. ---- */
	if (keyframe) {
		hbits(ctx, 1);			/* colour_space */
		hbits(ctx, 1);			/* clamping_type */
	}
	/*
	 * Seed from the PERSISTENT copy: RFC 6386 section 9.3 re-reads the
	 * segment feature data only when update_mb_segmentation_data is set,
	 * so a frame with segmentation_enabled == 1 and
	 * update_mb_segmentation_data == 0 keeps the previous frame's
	 * per-segment quantizer / loop-filter values and abs-delta mode. These
	 * used to be stack locals re-zeroed every frame, which silently forced
	 * every such frame to "no per-segment delta" -- wrong quantizer AND
	 * wrong loop-filter level for every macroblock outside segment 0.
	 */
	memcpy(seg_qd, inst->seg_qd, sizeof(seg_qd));
	memcpy(seg_lfd, inst->seg_lfd, sizeof(seg_lfd));
	seg_abs = inst->seg_abs;
	seg_enabled = hbits(ctx, 1);		/* segmentation_enabled */
	if (seg_enabled) {
		seg_upd_map = hbits(ctx, 1);
		upd_data = hbits(ctx, 1);

		if (upd_data) {
			seg_abs = hbits(ctx, 1);	/* segment_feature_mode */
			for (i = 0; i < 4; i++)
				seg_qd[i] = hdelta(ctx, 7);
			for (i = 0; i < 4; i++)
				seg_lfd[i] = hdelta(ctx, 6);
			memcpy(inst->seg_qd, seg_qd, sizeof(seg_qd));
			memcpy(inst->seg_lfd, seg_lfd, sizeof(seg_lfd));
			inst->seg_abs = seg_abs;
		}
		if (seg_upd_map) {
			/* RFC 6386 section 9.3: a map update resets the three
			 * tree probabilities, each optionally re-sent. */
			for (i = 0; i < 3; i++)
				inst->seg_tree_prob[i] =
					hbits(ctx, 1) ? hbits(ctx, 8) : 255;
		}
		/* Hand the tree probabilities to the HW right here -- this is
		 * exactly where the vendor writes them (trace order: the write
		 * sits between the last tree-prob read and the filter_type
		 * read, on BOTH a keyframe and an inter frame). */
		seg_tree_word = inst->seg_tree_prob[0] |
				(inst->seg_tree_prob[1] << 8) |
				(inst->seg_tree_prob[2] << 16);
		wr(ctx, VDEC_HWD, HWD_MVP_ADDR, HWD_SEG_TREE_IDX);
		wr(ctx, VDEC_HWD, HWD_MVP_DATA, seg_tree_word);
	}
	lf_type = hbits(ctx, 1);		/* filter_type */
	lf_level = hbits(ctx, 6);		/* loop_filter_level */
	lf_sharp = hbits(ctx, 3);		/* sharpness_level */
	if (hbits(ctx, 1)) {			/* loop_filter_adj_enable */
		lf_adj = true;
		if (hbits(ctx, 1)) {		/* mode_ref_lf_delta_update */
			for (i = 0; i < 4; i++)
				if (hbits(ctx, 1))
					inst->ref_lf_delta[i] =
						hdelta_val(ctx, 6);
			for (i = 0; i < 4; i++)
				if (hbits(ctx, 1))
					inst->mode_lf_delta[i] =
						hdelta_val(ctx, 6);
		}
	}

	/* ---- seq 2331-2343: arm VP8_VLD2 over the token partition ---- */
	(void)rd(ctx, VDEC_LD, VLD_IDLE);
	vld_arm_fetch(ctx, VDEC_HWB, bs->dma_addr, bs->dma_addr + part2_off,
		      bs_end, true);
	/*
	 * seq 2344-2353: skip the second stream forward to the true token-partition
	 * start. The VLD2 fetch lands on the 16-BYTE GRANULE CONTAINING
	 * bs + part2_off, not on part2_off itself, so the leading
	 * (part2_off mod 16) bytes must be consumed through the stream-1
	 * peek/consume port pair (+0x200 peek, +0x220 consume-one-byte) before the
	 * barrel shifter is initialised.
	 *
	 * MEASURED, not inferred: across the 10-frame vendor trace the peek count
	 * is exactly (part2_off mod 16) for every frame --
	 *   part2_off 549 108  81  70  71  53  72  70  70  72
	 *   peeks       5  12   1   6   7   5   8   6   6   8
	 * (10/10). This backend previously hard-coded 5, which is right ONLY when
	 * part2_off mod 16 == 5; that is why the canonical 176x144 clip's keyframe
	 * (part2_off 549, 549 mod 16 = 5) decoded bit-exact while every other
	 * keyframe we encoded decoded to ~6 dB garbage.
	 *
	 * The count was FIRST written as ((part2_off - 1) mod 16) + 1, which fits
	 * the same 10 trace points exactly -- none of them is 16-aligned, so the
	 * two expressions are indistinguishable there. They differ on ONE input:
	 * part2_off mod 16 == 0, where the "-1/+1" form consumes a full extra
	 * 16-byte granule and lands the barrel shifter one granule PAST the token
	 * partition. That single case is the dominant residual defect in the
	 * corpus: across VP8-TEST-VECTORS the FIRST frame whose part2_off is
	 * 16-aligned is exactly the first frame we decode wrong on 33 of the 38
	 * failing vectors, and every one of the 23 passing vectors has NO
	 * 16-aligned frame at all. part2_off mod 16 is the alignment of the fetch
	 * inside its granule, so 16-aligned means nothing to skip.
	 */
	npeek = part2_off & 15;
	for (i = 0; i < npeek; i++) {
		(void)rd(ctx, VDEC_HWD, HWD_PEEK2);
		(void)rd(ctx, VDEC_HWD, HWD_PEEK2_TAKE1);
	}

	log2_parts = hbits(ctx, 2);		/* log2_nbr_of_DCT_partitions */
	nparts = 1 << (log2_parts & 3);

	/*
	 * ---- seq 2352-2391: the token-partition window table.
	 *
	 * RFC 6386 section 9.5: a frame with N DCT partitions carries an
	 * (N-1) x 3-byte little-endian size table immediately after the first
	 * partition; partition 0 therefore starts at part2_off + 3*(N-1) and
	 * the LAST partition's size is implied by the bytes left in the frame.
	 *
	 * MEASURED against matched 1-/2-/8-partition vendor traces: the vendor
	 * writes base[0], then per table entry peeks the three size bytes
	 * (HWD_PEEK2), CONSUMES them from the second stream (HWD_PEEK2_TAKE3 --
	 * the exact generalisation of the single-partition TAKE1 skip loop
	 * above), writes size[i], reads base[i] back and writes
	 * base[i+1] = base[i] + size[i]. The trailing size register is written
	 * with the remainder and no peek/consume. Verified value-for-value on
	 * the 8-partition clip (bases 457/1184/1194/1195/1196/1197/1198/1650,
	 * sizes 727/10/1/1/1/1/452/594).
	 */
	tbl = 3 * (nparts - 1);
	part_left = bs->size > part2_off + tbl ? bs->size - part2_off - tbl : 0;
	wr(ctx, VDEC_HWD, HWD_PART2_BASE,
	   (u32)(bs->dma_addr + part2_off) + tbl);
	for (i = 0; i < nparts - 1; i++) {
		u32 peek = rd(ctx, VDEC_HWD, HWD_PEEK2);
		u32 psz = ((peek >> 24) & 0xff) | ((peek >> 8) & 0xff00) |
			  ((peek << 8) & 0xff0000);

		(void)rd(ctx, VDEC_HWD, HWD_PEEK2_TAKE3);
		if (psz > part_left)
			psz = part_left;	/* corrupt size table */
		wr(ctx, VDEC_HWD, HWD_PART2_SIZE + 4 * i, psz);
		wr(ctx, VDEC_HWD, HWD_PART2_BASE + 4 * (i + 1),
		   rd(ctx, VDEC_HWD, HWD_PART2_BASE + 4 * i) + psz);
		part_left -= psz;
	}
	wr(ctx, VDEC_HWD, HWD_PART2_SIZE + 4 * (nparts - 1), part_left);

	/* ---- seq 2392-2393: stream-1 barrel init ---- */
	wr(ctx, VDEC_HWD, HWD_BSHIFT_INIT, 0x100);
	wr(ctx, VDEC_HWD, HWD_BSHIFT_INIT, 0x0);
	(void)rd(ctx, VDEC_HWD, HWD_STATE110 + 4);

	/* ---- seq 2360-2365: quantiser header ---- */
	q_index = hbits(ctx, 7);
	ydc_d  = hdelta(ctx, 4);
	y2dc_d = hdelta(ctx, 4);
	y2ac_d = hdelta(ctx, 4);
	uvdc_d = hdelta(ctx, 4);
	uvac_d = hdelta(ctx, 4);

	/* ---- seq 2366-2389: per-segment dequantiser SRAM (VDEC_LD 0x260/0x264).
	 * Word layout per segment (trace, q_index=4 -> {16,12},{8,8},{8,8}):
	 *   +0x0 = (y2_dc << 16) | y2_ac
	 *   +0x4 = (y1_dc << 16) | y1_ac
	 *   +0x8 = (uv_dc << 16) | uv_ac
	 * repeated at segment stride 0x10. ---- */
	for (s = 0; s < 4; s++) {
		int q = seg_enabled ? (seg_abs ? seg_qd[s] : (int)q_index + seg_qd[s])
				    : (int)q_index;
		u32 y1dc, y1ac, y2dc, y2ac, uvdc, uvac;

		seg_q[s] = clampq(q);
		y1dc = vp8_dc_qlookup[clampq(q + ydc_d)];
		y1ac = vp8_ac_qlookup[clampq(q)];
		y2dc = vp8_dc_qlookup[clampq(q + y2dc_d)] * 2;
		y2ac = vp8_ac_qlookup[clampq(q + y2ac_d)] * 155 / 100;
		if (y2ac < 8)
			y2ac = 8;
		uvdc = vp8_dc_qlookup[clampq(q + uvdc_d)];
		if (uvdc > 132)
			uvdc = 132;
		uvac = vp8_ac_qlookup[clampq(q + uvac_d)];

		wr(ctx, VDEC_LD, VLD_DEQ_ADDR, s * 0x10 + 0x0);
		wr(ctx, VDEC_LD, VLD_DEQ_DATA, (y2dc << 16) | y2ac);
		wr(ctx, VDEC_LD, VLD_DEQ_ADDR, s * 0x10 + 0x4);
		wr(ctx, VDEC_LD, VLD_DEQ_DATA, (y1dc << 16) | y1ac);
		wr(ctx, VDEC_LD, VLD_DEQ_ADDR, s * 0x10 + 0x8);
		wr(ctx, VDEC_LD, VLD_DEQ_DATA, (uvdc << 16) | uvac);
	}

	if (keyframe) {
		refresh_entropy = hbits(ctx, 1);	/* refresh_entropy_probs */
	} else {
		/* ---- seq 7747-7755: RFC 6386 section 9.7/9.8 reference flags ---- */
		int refresh_golden = hbits(ctx, 1);
		int refresh_alt = hbits(ctx, 1);
		int sign_bias_golden, sign_bias_alt;

		if (!refresh_golden)
			hbits(ctx, 2);		/* copy_buffer_to_golden */
		if (!refresh_alt)
			hbits(ctx, 2);		/* copy_buffer_to_alternate */
		sign_bias_golden = hbits(ctx, 1);
		sign_bias_alt = hbits(ctx, 1);
		/* Sign-bias word. Both bits are 0 across the whole vendor trace, so
		 * the bit POSITIONS are inferred (golden = bit0, altref = bit1),
		 * matching the libvpx sign_bias[] ref-frame ordering; the written
		 * value 0 is trace-exact. */
		wr(ctx, VDEC_AV, MV_SIGN_BIAS,
		   (u32)(sign_bias_golden | (sign_bias_alt << 1)));
		refresh_entropy = hbits(ctx, 1);	/* refresh_entropy_probs */
		hbits(ctx, 1);				/* refresh_last */
	}

	/* A frame with refresh_entropy_probs == 0 uses its updated probabilities
	 * for itself only; the saved context is restored afterwards. */
	if (!refresh_entropy)
		inst->ent_saved = inst->ent;

	/* ---- seq 2391-2781 / 7756-8142: coefficient probabilities + commit. The
	 * upload is the PERSISTED context (defaults after a keyframe reset); the
	 * HW itself bool-decodes the in-header coefficient updates on top of it
	 * when HWD_COMMIT is pulsed. ---- */
	store_dec_table(ctx, inst->ent.coef);
	wr(ctx, VDEC_HWD, HWD_COMMIT, 0x1);
	for (i = 0; i < 10000; i++) {
		if (!(rd(ctx, VDEC_HWD, HWD_COMMIT) & 0x1))
			break;
		cpu_relax();
	}
	mb_no_coeff_skip = hbits(ctx, 1);
	/*
	 * HWD_SKIP_EN carries the SEGMENTATION ENABLE. MEASURED by diffing a
	 * matched pair of 176x144 q=4 lf=0 keyframes -- one segmented
	 * (vp80-03-segmentation-1401 f0), one not (vp8-176x144 f0): the
	 * ENTIRE difference in the vendor's 170-write frame, modulo bitstream
	 * and DMA addresses, is this register going 0x00010000 -> 0x00010110
	 * plus the tree-prob SRAM word. Bit 16 is mb_no_coeff_skip (already
	 * known). The 0x110 splits as bit4 = segmentation_enabled and bit8 =
	 * update_mb_segmentation_map -- that split is INFERRED, because all 61
	 * VP8-TEST-VECTORS frames that enable segmentation also update the map,
	 * so the two are confounded in the whole corpus. Both observed states
	 * (0x000 and 0x110) are reproduced exactly.
	 */
	wr(ctx, VDEC_HWD, HWD_SKIP_EN,
	   (mb_no_coeff_skip ? 0x00010000 : 0x0) | ((log2_parts & 3) << 12) |
	   (seg_enabled ? 0x10 : 0) | (seg_upd_map ? 0x100 : 0));
	if (mb_no_coeff_skip)
		prob_skip = hbits(ctx, 8);	/* prob_skip_false */
	if (!keyframe) {
		/* ---- seq 8145-8151 ---- */
		prob_intra = hbits(ctx, 8);
		prob_last = hbits(ctx, 8);
		prob_gf = hbits(ctx, 8);
	}
	wr(ctx, VDEC_HWD, HWD_PROBS,
	   prob_skip | (prob_intra << 8) | (prob_last << 16) | (prob_gf << 24));

	if (!keyframe) {
		/* ---- seq 8150-8173: intra-mode prob updates (literal, software) then
		 * the mode/MV SRAM upload; the MV prob updates themselves are
		 * bool-coded and applied by the HW on the HWD_MODE_COMMIT pulse. ---- */
		if (hbits(ctx, 1)) {		/* intra_16x16_prob_update_flag */
			u32 w = 0;

			for (i = 0; i < 4; i++)
				w |= hbits(ctx, 8) << (8 * i);
			inst->ent.ymode = w;
		}
		if (hbits(ctx, 1)) {		/* intra_chroma_prob_update_flag */
			u32 w = 0;

			for (i = 0; i < 3; i++)
				w |= hbits(ctx, 8) << (8 * i);
			inst->ent.uvmode = w;
		}
		store_mode_mv_probs(ctx, &inst->ent, true);
		wr(ctx, VDEC_HWD, HWD_MODE_COMMIT, 0x1);
		for (i = 0; i < 10000; i++) {
			if (!(rd(ctx, VDEC_HWD, HWD_MODE_COMMIT) & 0x1))
				break;
			cpu_relax();
		}
	}

	/* ---- seq 2782-2814 / 8174-8211: MC (recon / reference / geometry) ---- */
	wr(ctx, VDEC_CM, MC_CFGB5C, 0x1);
	/*
	 * MC+0x208 / MC+0x20c take the MB-ALIGNED picture size, NOT the display
	 * size from the frame tag. They bound the motion-compensation engine's
	 * reference read: a VP8 reference frame is the whole reconstructed
	 * MB grid (16*mb_w x 16*mb_h), INCLUDING the part of an edge macroblock
	 * that lies outside the display rectangle, and the border replication
	 * that RFC 6386 inter prediction sees starts only beyond THAT edge
	 * (libvpx vp8_yv12_extend_frame_borders() extends from the 16-aligned
	 * y_width/y_height; ffmpeg vp8.c passes width = 16*s->mb_width to
	 * ff_emulated_edge_mc). Feeding the display size made the engine clamp
	 * at the display edge instead, so every macroblock whose MC read reached
	 * into the padding of a partial edge MB got replicated pixels where the
	 * spec wants reconstructed ones.
	 *
	 * MEASURED, and it is invisible on an MB-ALIGNED clip -- which is every
	 * clip the vendor trace ever covered (all 176x144), so `pic_w` and
	 * `ALIGN(pic_w, 16)` were indistinguishable in the trace. Separated by
	 * rewriting the KEYFRAME-declared dimensions of three non-aligned fluster
	 * vectors UP to their 16-aligned value (mb_w/mb_h, and hence every other
	 * register this backend programs, are unchanged by that rewrite, and an
	 * ffmpeg decode of the rewritten clip is bit-identical to the original
	 * over the display rectangle on all 108 frames x 3 planes of
	 * vp80-02-inter-1418). Decoded through this backend the rewrite is NOT
	 * a no-op -- it is the whole defect:
	 *
	 *   vector                     declared    as-coded      16-aligned
	 *   vp80-02-inter-1418         200x200     1/108 exact   108/108 exact
	 *   vp80-00-comprehensive-006  175x143     2/48  exact    48/48  exact
	 *   vp80-00-comprehensive-008  1432x888    1/2   exact     2/2   exact
	 *
	 * and the two runs' CAPTURE dumps are BYTE-IDENTICAL on the keyframe and
	 * diverge from the first inter frame -- exactly a reference-read bound.
	 */
	wr(ctx, VDEC_CM, MC_PIC_W_PX, mb_w * 16);
	wr(ctx, VDEC_CM, MC_PIC_H_PX, mb_h * 16);
	wr(ctx, VDEC_CM, MC_MB_W2, mb_w);
	/*
	 * MC+0x97c selects the RECONSTRUCTION (motion-compensation) filter, and
	 * it is a pure function of the RFC 6386 section 9.1 frame-tag version --
	 * bit4 = six-tap (bicubic) luma interpolation, bit8 = full-pixel motion
	 * vectors (no sub-pel at all). It was a read-modify-write-back of
	 * whatever the block already held, which happens to be correct for
	 * version 0 only, so every version 1/2/3 clip decoded its keyframes
	 * bit-exact and drifted on every inter frame.
	 *
	 * MEASURED by a matched-quad vendor trace: one 3-frame 176x144 clip
	 * encoded four times from the same source with libvpx -profile:v 0/1/2/3
	 * (identical 3612-byte keyframe), traced through vpud on the 13-range
	 * vendor DTB. MC+0x97c is written once per frame and is the ONLY MC
	 * control that differs between version 0 and version 2:
	 *
	 *   version 0 -> 0x010   version 1 -> 0x000
	 *   version 2 -> 0x000   version 3 -> 0x100
	 *
	 * which is exactly libvpx vp8_setup_version(): versions 1 and 2 use the
	 * bilinear MC filter, version 3 additionally uses full-pixel MVs, and
	 * the reserved versions 4..7 fall back to version 0's behaviour.
	 */
	switch (version) {
	case 1:
	case 2:
		mc_filter = 0x000;
		break;
	case 3:
		mc_filter = 0x100;
		break;
	default:
		mc_filter = 0x010;
		break;
	}
	wr(ctx, VDEC_CM, MC_MC_FILTER, mc_filter);
	wr(ctx, VDEC_CM, MC_LUMA_SZ, luma_sz);
	/* last / golden / altref: a keyframe points all three at the current
	 * recon; an inter frame takes them from the stateless control. */
	vp8_resolve_refs(inst, keyframe, rec_y, rec_c, ref_y, ref_c);
	if (mt8365_vp8_quirk & 0x4)
		for (i = 1; i < 3; i++) {
			ref_y[i] = ref_y[0];
			ref_c[i] = ref_c[0];
		}
	for (i = 0; i < 3; i++) {
		wr(ctx, VDEC_CM, MC_REF_Y + 4 * i, ref_y[i]);
		wr(ctx, VDEC_CM, MC_REF_C + 4 * i, ref_c[i]);
	}
	wr(ctx, VDEC_CM, MC_ENABLE220, 0x1);
	wr(ctx, VDEC_CM, MC_ENABLE250, 0x1);
	wr(ctx, VDEC_CM, MC_RECON_Y, rec_y >> 9);
	wr(ctx, VDEC_CM, MC_RECON_C, rec_c >> 8);
	wr(ctx, VDEC_CM, MC_MB_W, mb_w);
	wr(ctx, VDEC_CM, MC_MB_W_M1, mb_w - 1);
	wr(ctx, VDEC_CM, MC_MB_H_M1, mb_h - 1);
	wr(ctx, VDEC_CM, MC_CFG57C, 0x0);
	/*
	 * MC+0xa20 / MC+0xa24 scratch. The vendor's pair is one 0x20000-strided
	 * region (0xff3e0000 / 0xff3c0000); ours are two independent allocations.
	 * quirk 0x20 reproduces the vendor's exact relationship out of a single
	 * 256 KiB buffer; quirk 0x40 additionally zeroes the scratch each frame.
	 */
	if ((mt8365_vp8_quirk & 0x20) && inst->pair_buf.va) {
		if (mt8365_vp8_quirk & 0x40)
			memset(inst->pair_buf.va, 0, inst->pair_buf.size);
		wr(ctx, VDEC_CM, MC_WORKBUF_A,
		   (u32)inst->pair_buf.dma_addr + 0x20000);
		wr(ctx, VDEC_CM, MC_WORKBUF_B, (u32)inst->pair_buf.dma_addr);
	} else {
		if (mt8365_vp8_quirk & 0x40) {
			memset(inst->work_buf[0].va, 0, inst->work_buf[0].size);
			memset(inst->work_buf[1].va, 0, inst->work_buf[1].size);
		}
		wr(ctx, VDEC_CM, MC_WORKBUF_A, (u32)inst->work_buf[0].dma_addr);
		wr(ctx, VDEC_CM, MC_WORKBUF_B, (u32)inst->work_buf[1].dma_addr);
	}
	wr(ctx, VDEC_MISC, MISC_MISC_F0, rd(ctx, VDEC_MISC, MISC_MISC_F0) | 0x1);

	/* ---- seq 8199-8203: inter-only MC enable + the PP loop filter. Both are
	 * absent from a keyframe decode (set-diff f0 vs f1). A frame with
	 * loop_filter_level == 0 (the trace's keyframe) writes no PP register at
	 * all, so the whole block is gated on the level. ---- */
	/*
	 * MC+0x238 = 3. In the 10-frame vendor trace this register is present on
	 * every inter frame and absent on the keyframe -- but that trace's
	 * keyframe is also the only frame with loop_filter_level == 0, so
	 * "inter" and "lf > 0" are perfectly CONFOUNDED in it and the original
	 * !keyframe gate was an inference from a single aliased point.
	 * quirk 0x8 restores that old gate so the two can be A/B'd on the board.
	 */
	/*
	 * SUPERSEDED reading, kept for the trace provenance it records: the
	 * value was read as 0x2 | loop_filter_adj_enable, from these points --
	 * the 10-frame trace could not show it (all nine of its inter frames
	 * have lf_delta_enable == 1). vp80-03-segmentation-01 is a KEYFRAME
	 * with lf_level 19 and lf_delta_enable 0 and the vendor writes 0x2;
	 * vp80-03-segmentation-1401's keyframes (lf > 0, lf_delta_enable 1)
	 * write 0x3, as do every traced inter frame. Those points are all still
	 * reproduced by the per-plane reading below, which additionally explains
	 * vp80-00-comprehensive-003. The WRITE GATE is left as-is (lf_level, or any inter
	 * frame): across all 16 traced frames the vendor's gate is exactly
	 * lf_level > 0, but no traced inter frame has lf_level == 0, and the
	 * currently bit-exact lf0.ivf does, so narrowing the gate is a
	 * separate, separately-measurable change.
	 */
	/*
	 * MEASURED: MC+0x238 is a per-PLANE loop-filter enable -- bit1 = filter
	 * LUMA, bit0 = filter CHROMA -- not 0x2 | loop_filter_adj_enable. The
	 * two readings are indistinguishable in every vendor trace because each
	 * traced frame is either (normal filter, lf_adj) -> 0x3 or (simple
	 * filter, no lf_adj) -> 0x2. vp80-00-comprehensive-003's keyframe is the
	 * one available (simple, lf_adj) frame and it separates them: with 0x3
	 * it decodes Y bit-exact and chroma wrong, with 0x2 it is bit-exact on
	 * ALL THREE planes. Clearing bit1 instead leaves chroma exact and breaks
	 * luma, which is the per-plane reading's other prediction.
	 *
	 * So the value follows filter_type, per RFC 6386 section 15.2: the
	 * simple filter is LUMA-ONLY, the normal filter covers both planes.
	 * loop_filter_adj_enable is carried by PP+0x20 bit8 alone (also
	 * confirmed on this frame: clearing bit8 costs luma its bit-exactness).
	 */
	if (!(mt8365_vp8_quirk & 0x8) ? (!keyframe || lf_level) : !keyframe)
		wr(ctx, VDEC_CM, MC_INTER_EN,
		   ((0x2u | (lf_type ? 0x0u : 0x1u)) | (u32)mt8365_vp8_mc238_or) &
		   ~(u32)mt8365_vp8_mc238_clr);
	for (s = 0; s < 4; s++) {
		int l = seg_enabled ? (seg_abs ? seg_lfd[s]
					       : (int)lf_level + seg_lfd[s])
				    : (int)lf_level;

		seg_lf[s] = clamp(l, 0, 63);
	}
	if (lf_level && !(mt8365_vp8_quirk & 0x1)) {
		/*
		 * PP+0x20 control word, now fully MEASURED except sharpness'
		 * width. The 10-frame trace aliased three fields at once (all
		 * its PP frames are inter, filter_type 0, lf_adj 1 -> a constant
		 * 0x00001101); the segmentation traces break every one of them:
		 *
		 *   clip / frame                    filter_type lf_adj key  value
		 *   vp8-176x144 + -1403 inter                 0      1   no  0x1101
		 *   vp80-03-segmentation-1401 KEY             0      1  yes  0x0101
		 *   vp80-03-segmentation-01   KEY             1      0  yes  0x0011
		 *
		 * -> bit0 = enable, bit4 = filter_type (SIMPLE), bit8 =
		 * loop_filter_adj_enable, bit12 = !keyframe. The old code had
		 * filter_type INFERRED at bit1 and bit12 hard-wired on, which
		 * is wrong for every keyframe with lf_level > 0. sharpness at
		 * bit16 is unchanged (measured earlier); its WIDTH is still
		 * assumed to be 3.
		 */
		wr(ctx, VDEC_PP, PP_LF_CTRL,
		   pp_lf_ctrl(keyframe, lf_type, lf_adj, lf_sharp));
		wr(ctx, VDEC_PP, PP_LF_LEVEL,
		   seg_lf[0] | (seg_lf[1] << 8) | (seg_lf[2] << 16) |
		   (seg_lf[3] << 24));
		lf_word = 0;
		for (i = 0; i < 4; i++)
			lf_word |= (u32)(u8)inst->ref_lf_delta[i] << (8 * i);
		wr(ctx, VDEC_PP, PP_REF_LF_DELTA, lf_word);
		lf_word = 0;
		for (i = 0; i < 4; i++)
			lf_word |= (u32)(u8)inst->mode_lf_delta[i] << (8 * i);
		wr(ctx, VDEC_PP, PP_MODE_LF_DELTA, lf_word);
	}
	/*
	 * MC+0x850 / MC+0x854 are the recon write DMA's address WINDOW, not a
	 * pair of per-plane pointers: 0x854 is the window BASE and 0x850 the
	 * window LIMIT, both >> 4. Every byte the recon DMA emits -- luma AND
	 * chroma -- must fall inside [base, limit); the per-plane bases the HW
	 * actually writes to are 0x224 (Y >> 9) and 0x228 (C >> 8).
	 *
	 * MEASURED, and it is why this used to be intermittent. The vb2 CAPTURE
	 * planes are two independent dma-contig allocations, so their relative
	 * order is whatever the allocator picks. The old code wrote
	 * base = rec_c, limit = rec_y + luma_sz, which spans both planes ONLY
	 * when rec_c < rec_y. When the allocator handed back a buffer with C
	 * ABOVE Y the window was inverted, the recon DMA wrote NOTHING, and the
	 * frame came out all zero -- while the HW picture CRC stayed correct,
	 * because the reconstruction itself had happened. Correlation over 20
	 * back-to-back decodes of one clip: 18 runs with C = Y - 0x4000 all
	 * bit-exact, the 2 runs with C > Y both all-zero, 20/20. The 128x96
	 * clip hit it deterministically (its size class always allocated C above
	 * Y), which is the whole of its "6.59 dB geometry failure".
	 *
	 * The union below is identical to the old expression in the C-below-Y
	 * case (rec_c + luma_sz/2 < rec_y <= rec_y + luma_sz), so the vendor-
	 * bit-exact trace case is reproduced unchanged.
	 */
	{
		u32 wlo = min(rec_y, rec_c);
		u32 whi = max(rec_y + luma_sz, rec_c + luma_sz / 2);

		wr(ctx, VDEC_CM, MC_RECON_Y4, whi >> 4);
		wr(ctx, VDEC_CM, MC_RECON_C4, wlo >> 4);
	}
	wr(ctx, VDEC_CM, MC_RECON_EN, 0x1);
	wr(ctx, VDEC_TOP, TOP_CFG5C, rd(ctx, VDEC_TOP, TOP_CFG5C) | 0x20000000);
	wr(ctx, VDEC_MISC, MISC_STATE, 0x3);

	/* ---- seq 2815-2817: DECODE KICK (VP8_VLD+0xb8 pulse), then poll done.
	 * NOTE: VP8 does NOT use the H.264 MISC+0x28 trigger. ---- */
	wr(ctx, VDEC_HWD, HWD_KICK, 0x1);
	wr(ctx, VDEC_HWD, HWD_KICK, 0x0);
	for (i = 0; i < MT8365_VP8_TIMEOUT; i++) {
		if (rd(ctx, VDEC_HWD, HWD_DONE) & 0x1)
			break;
		cpu_relax();
	}
	if (i == MT8365_VP8_TIMEOUT) {
		mtk_v4l2_vdec_err(ctx,
			"mt8365 vp8 TIMEOUT f%u %ux%u q=%u p2=%u bs=%pad sz=%zx | HWD[0x174]=%08x [0x15c]=%08x [0x25c]=%08x MISC[0xa4]=%08x [0x04]=%08x | LD[0xe8]=%08x [0xf4]=%08x HWB[0xe8]=%08x",
			inst->frm_cnt, pic_w, pic_h, q_index, part2_off,
			&bs->dma_addr, bs->size,
			rd(ctx, VDEC_HWD, HWD_DONE), rd(ctx, VDEC_HWD, HWD_STATUS15C),
			rd(ctx, VDEC_HWD, 0x25c), rd(ctx, VDEC_MISC, 0xa4),
			rd(ctx, VDEC_MISC, MISC_STATE),
			rd(ctx, VDEC_LD, VLD_FETCH_DONE), rd(ctx, VDEC_LD, VLD_IDLE),
			rd(ctx, VDEC_HWB, VLD_FETCH_DONE));
		return -ETIMEDOUT;
	}

	/* ---- seq 2818-5387: the vendor's post-decode tail. MANDATORY -- see the
	 * load_dec_table() comment: without it the recon write DMA is still in
	 * flight when the core cuts the VDEC clocks, and the board hard-resets. ---- */
	load_dec_table(ctx, inst->ent.coef);
	if (!keyframe)
		load_mode_mv_probs(ctx, &inst->ent);
	(void)rd(ctx, VDEC_HWD, HWD_STATUS15C);

	/*
	 * The CRC window MISC+0x08..0x24 is a per-picture reconstruction hash. It
	 * is the cheapest oracle we have against the vendor: for the canonical
	 * 176x144 clip this backend must reproduce the vendor trace's per-frame
	 * CRC[0] c9a7d031 0806ec98 c81a70c5 2d87ade0 fe7a212e 32e7f9f2 cda810ab
	 * b0f4bcab d979a07e 376ad33e (frames 0..9).
	 */
	/* DIAG (mt8365_vp8_diag=1, _err so it survives without dynamic-debug):
	 * the per-frame write-set summary to diff against the vendor trace. */
	if (mt8365_vp8_diag)
		mtk_v4l2_vdec_err(ctx,
		"mt8365 vp8 f%u %s %ux%u q=%u lf=%u/%u/%u seg=%d p2=%u np=%u rec=%08x/%08x ref=%08x,%08x,%08x refc=%08x,%08x,%08x probs=%08x pp=%08x/%08x/%08x/%08x ctx=%08x^%08x,%08x ym=%08x uv=%08x mvs=%08x re=%d wb=%08x/%08x sid=%08x qk=%d CRC=%08x %08x %08x %08x",
		inst->frm_cnt, keyframe ? "KEY" : "INTER", pic_w, pic_h,
		q_index, lf_level, lf_sharp, lf_type,
	seg_enabled | (seg_upd_map << 1) | (seg_abs << 2) | (upd_data << 3),
	part2_off, nparts,
		rec_y, rec_c, ref_y[0], ref_y[1], ref_y[2],
		ref_c[0], ref_c[1], ref_c[2],
		prob_skip | (prob_intra << 8) | (prob_last << 16) | (prob_gf << 24),
		lf_level ? pp_lf_ctrl(keyframe, lf_type, lf_adj, lf_sharp) : 0u,
		seg_lf[0] | (seg_lf[1] << 8) | (seg_lf[2] << 16) | (seg_lf[3] << 24),
		(u32)((u8)inst->ref_lf_delta[0] | ((u8)inst->ref_lf_delta[1] << 8) |
		      ((u8)inst->ref_lf_delta[2] << 16) | ((u8)inst->ref_lf_delta[3] << 24)),
		(u32)((u8)inst->mode_lf_delta[0] | ((u8)inst->mode_lf_delta[1] << 8) |
		      ((u8)inst->mode_lf_delta[2] << 16) | ((u8)inst->mode_lf_delta[3] << 24)),
		vp8_ctx_sum(inst->ent.coef), vp8_ctx_xor(inst->ent.coef),
		inst->ent.coef[9],
		inst->ent.ymode, inst->ent.uvmode, inst->ent.mv_short,
		refresh_entropy,
		(u32)inst->work_buf[0].dma_addr, (u32)inst->work_buf[1].dma_addr,
		rd(ctx, VDEC_TOP, TOP_SEGID_DRAM_ADDR), mt8365_vp8_quirk,
		rd(ctx, VDEC_MISC, MISC_CRC_BASE),
		rd(ctx, VDEC_MISC, MISC_CRC_BASE + 4),
		rd(ctx, VDEC_MISC, MISC_CRC_BASE + 8),
		rd(ctx, VDEC_MISC, MISC_CRC_BASE + 12));
	read_hw_segmentation_data(ctx);

	if (mt8365_vp8_quirk & 0x10) {
		u32 awb = ALIGN(pic_w, 16), ahb = ALIGN(pic_h, 32);

		vp8_extend_bottom_border(fb->base_y.va, awb, ahb, pic_h, 32);
		vp8_extend_bottom_border(fb->base_c.va, awb, ahb / 2,
					 pic_h / 2, 16);
	}

	if (!refresh_entropy)
		inst->ent = inst->ent_saved;
	inst->last_y = rec_y;
	inst->last_c = rec_c;
	inst->have_last = true;
	(void)seg_q[0];
	inst->frm_cnt++;
	return 0;
}

static int vdec_vp8_mt8365_get_param(void *h_vdec,
				     enum vdec_get_param_type type, void *out)
{
	struct vdec_vp8_mt8365_inst *inst = h_vdec;
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;

	switch (type) {
	case GET_PARAM_PIC_INFO:
		/* MM21 at the CODED TILE size -- identical rule to the mt8365
		 * H.264 backend: the HW writes the recon planes densely at the
		 * MB stride ALIGN(pic_w,16) x ALIGN(pic_h,32), with the UV plane
		 * immediately after the packed Y. Declaring ALIGN(.,64) lies to
		 * userspace and collapses a bit-exact decode to ~8.6 dB. */
		ctx->picinfo.buf_w = ALIGN(ctx->picinfo.pic_w, 16);
		ctx->picinfo.buf_h = ALIGN(ctx->picinfo.pic_h, 32);
		ctx->picinfo.fb_sz[0] = ctx->picinfo.buf_w * ctx->picinfo.buf_h;
		ctx->picinfo.fb_sz[1] = ctx->picinfo.fb_sz[0] >> 1;
		*(struct vdec_pic_info *)out = ctx->picinfo;
		break;
	case GET_PARAM_DPB_SIZE:
		*(unsigned int *)out = VP8_DPB_SIZE;
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

static void vdec_vp8_mt8365_deinit(void *h_vdec)
{
	struct vdec_vp8_mt8365_inst *inst = h_vdec;

	if (inst)
		free_work_bufs(inst);
	kfree(h_vdec);
}

const struct vdec_common_if vdec_vp8_mt8365_if = {
	.init		= vdec_vp8_mt8365_init,
	.decode		= vdec_vp8_mt8365_decode,
	.get_param	= vdec_vp8_mt8365_get_param,
	.deinit		= vdec_vp8_mt8365_deinit,
};
