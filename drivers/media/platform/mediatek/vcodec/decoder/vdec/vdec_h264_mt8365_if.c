// SPDX-License-Identifier: GPL-2.0
/*
 * MT8365 (Genio 350) stateless H.264 decode backend -- DIRECT REGISTER, no SCP,
 * no codec firmware, no vpud.
 *
 * Unlike the mt8183+ stateless backends (which marshal the V4L2 H.264 controls
 * into a VSI and IPI them to the SCP), the mt8167-family MT8365 has no codec
 * coprocessor firmware -- the HW VDEC is a stateless slice accelerator driven by
 * direct register programming (historically done by the closed userspace
 * libvpud_vcodec.so/vpud). This backend reuses the mainline framework's V4L2
 * Request plumbing + the shared mtk_vdec_h264_* marshalling helpers + the
 * VDEC_MISC IRQ, and replaces "send to SCP + wait IPI" with "program the VDEC
 * registers + trigger + wait the HW IRQ".
 *
 * Register map (sub-block bases from mt8365.dtsi codec@16000000, offsets RE'd
 * from libvpud_vcodec.so and TRACE-VERIFIED against a real 10-frame decode;
 * the full map is in the internal mt8365 VDEC register-map record):
 *
 * Sub-block -> reg_base[] index (enum mtk_hw_reg_idx):
 *   VDEC_SYS  0x16000000   VDEC_MISC 0x16025000   VDEC_TOP 0x16020800
 *   VDEC_CM   0x16021000 (MC)        VDEC_AD  0x16022000 (AVC_VLD)
 *
 * STATUS: WIP. The full trace-derived register set is programmed -- picture
 * size, input base, recon Y/C + mirrors, ref-list arrays (full phys), SPS/PPS +
 * config words, frame_num/IDR, trigger, IRQ wait. The MV/colocated/deblock
 * WORKING BUFFERS are driver-allocated in decode() (alloc_work_bufs), and
 * get_param() answers PIC_INFO/DPB_SIZE/CROP_INFO so the core can size the
 * CAPTURE queue (stateless has no SOURCE_CHANGE). Remaining board-iteration
 * TODO: validate the exact MISC+0x160 input-base encoding and the SPS/PPS field
 * bit-packing against a live decode (first end-to-end frame).
 */

#include <linux/dma-mapping.h>
#include <linux/moduleparam.h>

#include <media/v4l2-mem2mem.h>
#include <media/v4l2-h264.h>
#include <media/videobuf2-dma-contig.h>

#include "../mtk_vcodec_dec.h"
#include "../../common/mtk_vcodec_intr.h"
#include "../vdec_drv_base.h"
#include "../vdec_drv_if.h"
#include "vdec_h264_req_common.h"

/* ---- trace-verified register offsets (within the sub-block reg_base) ----
 * All offsets + the address shifts below are from the full 10-frame live trace
 * (mt8365-vdec-register-map.md). Recon Y = phys>>9, recon C = phys>>8, the
 * ref-list arrays store FULL phys (no shift). Test clip was 176x144 QCIF.
 */
/* VDEC_SYS */
#define SYS_ENABLE		0x00	/* = 0x1 per frame */
/* VDEC_MISC */
#define MISC_CRC		0x04	/* = 0x10 */
#define MISC_TRIGGER		0x28	/* = 0x1 : DECODE GO (one per frame) */
#define  MISC_TRIGGER_GO	0x1
#define MISC_RECON_Y_FULL	0x3c	/* recon Y full phys (display/CRC mirror) */
#define MISC_RECON_C_FULL	0x40	/* recon C full phys */
#define MISC_WORKBUF_A		0x54	/* MV/deblock working buffer (full phys) */
#define MISC_WORKBUF_B		0x58	/* MV/deblock working buffer (byte-gran) */
#define MISC_INPUT_BASE		0x160	/* INPUT bitstream buffer base (see note) */
/* VDEC_MISC + 0xa4 == VDEC_IRQ_CFG_REG : dec-done status (handled by the core
 * IRQ handler; success = MTK_VDEC_IRQ_STATUS_DEC_SUCCESS 0x10000) */
/* VDEC_TOP */
#define TOP_CTRL		0x28	/* = 0xff1e0000 */
#define TOP_ENABLE		0x54	/* = 0x1 */
#define TOP_PICSIZE_MB		0x68	/* = ((mb_h-1)<<16)|(mb_w-1) */
#define TOP_PICSIZE_PX		0x70	/* = (height_px<<16)|width_px */
/* VDEC_AD (AVC_VLD) */
#define AVCVLD_SPS_A		0x84	/* SPS word A; bit23 (0x800000) = non-IDR */
#define  AVCVLD_SPS_A_BASE	0xae00e007
#define  AVCVLD_SPS_A_NONIDR	0x00800000
#define AVCVLD_SPS_B		0x88	/* SPS word B (dims/POC) = 0x00346081 */
#define AVCVLD_SPS_C		0x90	/* SPS/PPS word = 0x00f1f7a0 */
#define AVCVLD_REFCTL		0x98	/* per-frame ref/POC control */
#define AVCVLD_FRAME_CNT	0x9c	/* low byte = frame_num; [19:16] IDR/P */
#define AVCVLD_CFG0		0x26c	/* = 0x7fffbcb7 */
#define AVCVLD_CFG1		0x280	/* = 0x7fffbcb7 */
#define AVCVLD_CFG2		0x2c8	/* = 0xf900a03b */
#define AVCVLD_CFG3		0x2d0	/* = 0x08009101 */
#define AVCVLD_CFG4		0x2d4	/* = 0x00204008 */
#define AVCVLD_MASK0		0x2b4	/* = 0xffffffff */
#define AVCVLD_MASK1		0x2b8	/* = 0xffffffff */
#define AVCVLD_SLICE_STROBE	0x28c
/*
 * N88 -- H.264 High scaling-matrix (quant-matrix) RAM port. RE'd from the
 * vendor blob's straight-line setter H264_HAL_WriteScalingList
 * (libvpud_vcodec.so @ 0x4b750).
 *
 * N90 CORRECTION -- the block was WRONG. `hal+0x7d8` is not a block base, it is
 * the base of the HAL's mapped-register-pointer array, and each H264_HAL_Write*
 * helper indexes a DIFFERENT slot of it:
 *   +0x08 SYS  +0x10 MISC/CRC  +0x18 VLD  +0x20 VLDTOP  +0x28 MC
 *   +0x30 AVCVLD  +0x38 AVCMV  +0x40 PP  +0x60 UFO
 * which is exactly `enum mtk_hw_reg_idx` shifted by one slot. WriteScalingList
 * calls H264_HAL_WriteVLD (+0x18 = VDEC_LD, 0x16020000); SetPPSAVLD calls
 * H264_HAL_WriteAVCVLD (+0x30 = VDEC_AD, 0x16022000). So the quant-matrix RAM
 * lives on VDEC_LD and the 0x94 enable bit lives on VDEC_AD -- see h264_scaling
 * bit10.
 *
 * The RAM is written 4 bytes at a time through an address/data pair:
 *	[QM_ADDR] = 0x200 | byte_off	(select + write-enable phase)
 *	[QM_DATA] = b0<<24 | b1<<16 | b2<<8 | b3
 *	[QM_ADDR] = 0x100 | byte_off	(latch phase)
 *
 * Byte map of the RAM (contiguous, 0xe0 bytes total):
 *	0x00 + 16*i	4x4 list i, i = 0..5	(Intra Y/Cb/Cr, Inter Y/Cb/Cr)
 *	0x60		8x8 list 0		(Intra Y)
 *	0xa0		8x8 list 1		(Inter Y)
 * Both are in RASTER (inverse-scanned) order -- the blob's own
 * QUANT_INTRA_DEFAULT / QUANT_INTER_DEFAULT / QUANT8_{INTRA,INTER}_DEFAULT
 * symbols hold the spec's Default_*_* tables already de-zigzagged (the blob
 * carries ZZ_SCAN / ZZ_SCAN8 separately for the parser), and V4L2's
 * scaling_list_4x4/8x8 are specified "after applying the inverse scanning
 * process" -- i.e. the same raster order. No re-scan is needed.
 */
#define AVCVLD_QM_ADDR		0x260
#define AVCVLD_QM_DATA		0x264
#define  AVCVLD_QM_ADDR_SEL	0x200	/* select + write-enable phase */
#define  AVCVLD_QM_ADDR_LATCH	0x100	/* latch phase */
#define AVCVLD_QM_4X4_BASE(i)	(16u * (i))
#define AVCVLD_QM_8X8_BASE(i)	((i) ? 0xa0u : 0x60u)
/* AVCVLD+0x94: the vendor's SetPPSAVLD ORs its bScalingMatrix argument in at
 * bit18 and the 8 per-list "list is in use" bytes at [17:10]. */
#define  AVCVLD_PPS94_QM_EN	BIT(18)
#define  AVCVLD_PPS94_QM_LISTS	GENMASK(17, 10)
/* VDEC_CM (MC) */
#define MC_RECON_Y		0x224	/* recon/output Y dma >> 9 */
#define MC_RECON_C		0x228	/* recon/output C dma >> 8 */
#define MC_RECON_C4		0x850	/* recon C dma >> 4 (mirror) */
#define MC_RECON_Y4		0x854	/* recon Y dma >> 4 (mirror) */
#define MC_DPB_BASE		0x560	/* recon/DPB Y full phys */
#define MC_REF_Y0		0x3dc	/* ref-list Y array #0, +4*n (full phys) */
#define MC_REF_Y1		0x45c	/* ref-list Y array #1 */
#define MC_REF_Y2		0x4dc	/* ref-list Y array #2 */
#define MC_REF_C0		0xb60	/* ref-list C array #0, +4*n (full phys) */
#define MC_REF_C1		0xbe0	/* ref-list C array #1 */
#define MC_REF_C2		0xc60	/* ref-list C array #2 */
#define MC_WORKBUF_A		0xa20	/* colocated-MV working buffer (full phys) */
#define MC_WORKBUF_B		0xa24
#define MC_CFG0			0x768	/* = 0x01000004 */
#define MC_CFG1			0x920	/* = 0x01000111 */

#define MT8365_VDEC_TIMEOUT_MS	1000

/*
 * A/B levers for the 2026-07-25 progressive-gap round. Each defaults to the NEW
 * behaviour; setting it to 0 restores the pre-round programming byte-for-byte so
 * both arms are reachable on ONE kernel (no rebuild, no DTB change between arms).
 */
static bool h264_longterm = true;
module_param(h264_longterm, bool, 0644);
MODULE_PARM_DESC(h264_longterm,
		 "mt8365 H.264 N27: keep LONG_TERM DPB entries in the ref lists (default 1)");

/*
 * N28 DEFAULT OFF -- MEASURED, NOT SPECULATIVE. The mechanism is real (the
 * AVCVLD+0x230 strobe completes on silicon: the "0x230 stuck" diagnostic never
 * fired across a CVWP1_TOSHIBA_E decode), but as wired here it flips NO vector
 * and DEGRADES one: on kernel 6.18.35-01064-g438dca9ad609, with h264_longterm=1,
 *   h264_predwt=1 -> CAWP5_TOSHIBA_E = Error, CVWP1/2/3/5 + CAWP1 = Fail
 *   h264_predwt=0 -> CAWP5_TOSHIBA_E = Fail,  CVWP1/2/3/5 + CAWP1 = Fail
 * so the HW-parse handoff is not equivalent to the vendor's in-parse call in
 * this driver's software-parse + bulk-barrel-advance model. Kept behind the
 * param for the next round (needs a vendor read-trace of a CVWP clip to place
 * the strobe), default 0 so no vector regresses.
 */
/*
 * N48 RE-OPENED. The N28 verdict above was measured on the FRAME-BASED parse
 * path; under gstreamer's v4l2slh264dec (which always supplies
 * header_bit_size) the pred_weight_table() bit window was never computed at
 * all, so the gate below could not fire and h264_predwt=1 was a byte-for-byte
 * no-op -- re-measured this session on 6.18.35-01233-g82a03b316951, all four
 * WP vectors byte-identical between h264_predwt=0 and =1.
 *
 * The per-MB instrument (tests/h264-mb-errmap.py) then named the mechanism:
 * this hardware applies NO explicit weighted prediction. Our whole-clip output
 * for CVWP1_TOSHIBA_E and CAWP1_TOSHIBA_E is BIT-IDENTICAL to the ffmpeg
 * reference decoded with weighted prediction suppressed. So the weights have
 * never reached the HW, and the AVCVLD+0x230 in-parse handshake -- the only
 * weight path that exists in the vendor binary, and the vendor writes no
 * weight REGISTER anywhere -- is the mechanism that must be made to fire.
 *
 * Bitmask (0 = pre-N48 behaviour byte-for-byte):
 *   1 = fire the AVCVLD+0x230 handshake, with the pwt window now derived on
 *       the slice-params path too
 *   2 = write the 0xf0 restore to VDEC_MISC (the block the value was READ
 *       from) instead of VDEC_AD, which is what the vendor transcription
 *       literally reads and is the one ambiguity in it
 */
static int h264_predwt;
module_param(h264_predwt, int, 0644);
MODULE_PARM_DESC(h264_predwt,
		 "mt8365 H.264 N28/N48 pred_weight_table HW parse (AVCVLD+0x230) BITMASK: 1=fire the handshake (pwt window now derived on the slice-params path), 2=restore 0xf0 to VDEC_MISC not VDEC_AD (default 0 = pre-N48 byte-for-byte)");

/*
 * N30 recon-window A/B. 0 = the historical assignment (0x854 <- recon Y >> 4,
 * 0x850 <- recon C >> 4). 1 = the VP8-measured window form (0x854 = window BASE
 * = min(Y,C) >> 4, 0x850 = window LIMIT = max(plane end) >> 4). Pixel-visible?
 * -> settled by hw_md5 on one kernel.
 */
static int h264_recwin;
module_param(h264_recwin, int, 0644);
MODULE_PARM_DESC(h264_recwin,
		 "mt8365 H.264 N30: 0=as-is (0x854=Y,0x850=C), 1=VP8 union window form");

#define MBAFF_T68	0x1
#define MBAFF_A0	0x2
#define MBAFF_MV220	0x4
#define MBAFF_VLD2D8	0x8

/*
 * N38 MBAFF (mb_adaptive_frame_field_flag) programming. Derived from a MATCHED
 * WRITE-VALUE differential of two vendor vpud read-traces captured on THIS
 * silicon at the SAME resolution (720x480, 45x30 MB):
 *   ~/bftrace/trace-mbaff-cvma1.log (CVMA1_Sony_D, MBAFF I) vs
 *   ~/bftrace/trace-frm.log         (progressive)
 * A write-set PRESENCE diff sees only two MBAFF-only registers (AVCMV+0x220,
 * AVCVLD+0x2d8); the VALUE diff additionally exposes two registers BOTH traces
 * write with a DIFFERENT constant, which presence can never see:
 *   TOP+0x68     prog 0x001d002c   mbaff 0x000e002c   (MB-PAIR rows, not MB rows)
 *   AVCVLD+0xa0  prog 0x00008000   mbaff 0x00010000   (= the FIELD ref-list mode)
 * Our driver was programming the PROGRESSIVE form of both on MBAFF clips (the
 * in-driver stop-MB dump printed t68=001d002c verbatim) and omitting both
 * MBAFF-only writes -- the HW walked every MB and then never raised completion.
 * Param so both arms are reachable on ONE kernel (0 = pre-round byte-for-byte).
 *
 * MEASURED, kernel 6.18.35-01160-g7e2c1a42e21f, CVMA1_Sony_D, one 16-way
 * bitmask sweep on ONE kernel (mt8365-h264-mbaff-masksweep.sh). Transcribed:
 *   MASK=0  GST_RC=1 hw_bytes=0        t68=001d002c err]=00000080 mbx]=2c mby]=1d
 *   MASK=1  GST_RC=0 hw_bytes=2592000  t68=000e002c  hw_md5 == sw_md5  BIT-EXACT
 *   MASK=2  GST_RC=1 hw_bytes=0        t68=001d002c err]=00000080 mbx]=2c mby]=1d
 *   MASK=4  GST_RC=1 hw_bytes=0        t68=001d002c err]=00000080 mbx]=2c mby]=1d
 *   MASK=8  GST_RC=1 hw_bytes=0        t68=001d002c err]=00000000 mbx]=00 mby]=00
 *   MASK=3  GST_RC=1 hw_bytes=0        t68=000e002c err]=00000000 mbx]=2c mby]=1d
 *   MASK=7  GST_RC=1 hw_bytes=0        t68=000e002c err]=00000000 mbx]=2c mby]=1d
 * So the ENTIRE lever is bit 0 (TOP+0x68). The two MBAFF-ONLY registers the
 * presence differential found are not it: AVCMV+0x220 is inert (mask 4 == mask
 * 0), and AVCVLD+0x2d8 is HARMFUL -- it stops the MB walk before MB0 (mask 8
 * mbx/mby = 0/0), consistent with a GetStartCode strobe repositioning the VLD
 * under our bulk-barrel-advance model. AVCVLD+0xa0's field value also BREAKS
 * MBAFF (mask 3 and 7 regress mask 1 back to no output). Default is therefore
 * 1, not 0xf; bits 2/4/8 are retained only so the negative result stays
 * reproducible on one kernel.
 */
/*
 * N39 FIELD PicNum write-VALUE bitmask (2026-08-03). See the AVCVLD+0x0a4
 * comment in mt8365_decode(): the vendor writes the FIELD PicNum
 * (2*frame_num + 1) for a field picture where this driver wrote the raw
 * frame_num. Default 0 = pre-round byte-for-byte so the cherry-pick is a no-op
 * until a measurement says otherwise.
 *   1 = AVCVLD+0x0a4 current-picture PicNum in field form (2*frame_num + 1)
 * The REFERENCE PicNums (AVCVLD+0x0a8/0x128/0x1a8) come from the V4L2 uAPI
 * dpb[].pic_num, which is documented to already be the PicNum assigned by the
 * ref-list construction process (field form for a field picture) -- the FLDPICNUM
 * dmesg line below prints them so that is verified rather than assumed.
 */
#define FLDPICNUM_CUR	0x1
static int h264_fldpicnum;
module_param(h264_fldpicnum, int, 0644);
MODULE_PARM_DESC(h264_fldpicnum,
		 "mt8365 H.264 N39 FIELD PicNum BITMASK: 1=AVCVLD+0x0a4 current PicNum = 2*frame_num+1 (default 0 = pre-round byte-for-byte)");

/*
 * N51 FIELD RefPicList0 bitmask (2026-08-04). A FIELD reference list holds TWO
 * entries per DPB frame (the top and the bottom field of a frame are separate
 * reference pictures, H.264 Sec 8.2.4.2.2 / 8.2.4.2.5), while this driver's
 * mt8365_build_init_p_list() enumerates DPB FRAMES. On a field picture that
 * makes the programmed descriptor set exactly HALF the list the bitstream
 * activates -- e.g. CVFI1_SVA_C sends num_ref_idx_l0_active = 1,2,3,4,5,... and
 * the hardware's own ue(v) reader (h264_hwsh bit 3) reads the same, while we
 * populate 1,1,2,2,3,... descriptor slots. The COUNT register AVCVLD 0x98[24:20]
 * is already correct in field units (the N45 raw-count rule reports the raw
 * bitstream value whenever it exceeds the slot count), so the HW is told "N
 * active refs" and reads N/2 populated + N/2 stale descriptors.
 *   1 = build + program the spec FIELD RefPicList0: per-field entries with the
 *       Sec 8.2.4.2.5 alternating-parity order, field PicNum
 *       (2*FrameNumWrap + 1 same parity / 2*FrameNumWrap opposite), per-parity
 *       POC, and the vendor's FIELD descriptor encoding (bit19 + bit22 parity,
 *       4-byte AVCMV stride) instead of the frame encoding (bit21, 8-byte).
 *   2 = FLDRL0 per-field dmesg dump of the built list (diagnostic only).
 * eff_field_pic-gated, so every frame-coded picture is byte-for-byte unchanged.
 */
#define FLDLIST_LIST	0x1
#define FLDLIST_DIAG	0x2
/*
 * N104 FIELD ref_pic_list_modification (2026-08-20). The N51 field path above
 * was gated on `nops == 0`: a field slice carrying a ref_pic_list_modification
 * fell all the way back to the FRAME-unit list, because Sec 8.2.4.3.1 operates
 * on FIELD PicNums when field_pic_flag is set and the frame-unit reorder cannot
 * express that. Measured on MR6_BT_B (field-coded, no B slice, 120 field
 * pictures): only 27 pictures reached the field builder -- every other P field
 * was programmed with a HALF-LENGTH list in the wrong number space. Verbatim,
 * fn=1 bot=1, against the software reference model
 * (the internal reflist reference script):
 *   driver   nl0=2 rp=0,1                (frame PicNums, 2 entries)
 *   spec     nl0=3 pn=2,1,0              (field PicNums, 3 entries)
 * 4 = run Sec 8.2.4.3.1 over the FIELD list in field units -- MaxPicNum =
 *     2*MaxFrameNum, CurrPicNum = 2*frame_num+1, short-term matched on the
 *     field PicNum the Sec 8.2.4.2.5 interleave assigned, long-term (idc 2) on
 *     LongTermPicNum -- so a field slice with a modification list keeps the
 *     field path instead of falling back.
 * Field-picture-gated (a PAFF FRAME picture on the N58 path keeps the frame-unit
 * reorder, where its CurrPicNum really is frame_num), so no frame-coded output
 * moves.
 */
#define FLDLIST_RPLM	0x4
/*
 * Default 1 as of the N51 gate: on-silicon A/B on one kernel
 * (6.18.35-01240-g505f67558118) moved every field P picture up 8-13 dB with a
 * byte-identical 78-vector JVT-AVC_V1 passer set --
 *   CVFI1_SVA_C      n:2 7.81 -> 20.83 dB   (md5 f9edc37a... -> bc73de81...)
 *   CVMAPAQP3_Sony_E n:2 15.10 -> 21.59 dB  (md5 56bd0295... -> c6a2af81...)
 * No field vector reaches bit-exact yet, so nothing flips; the value of the
 * default is that the field path now starts from the SPEC list.
 */
static int h264_fldlist = FLDLIST_LIST | FLDLIST_RPLM;
module_param(h264_fldlist, int, 0644);
MODULE_PARM_DESC(h264_fldlist,
		 "mt8365 H.264 N51/N104 FIELD RefPicList0 BITMASK: 1=build+program the per-FIELD Sec 8.2.4.2.5 list (parity descriptors, field PicNum), 2=FLDRL0 dump, 4=Sec 8.2.4.3.1 reorder in FIELD units (default 5)");

/*
 * N52 (2026-08-04): the per-group REFERENCE-PICTURE-STRUCTURE registers
 * MC+0x564 / MC+0x568 / MC+0x56c.
 *
 * Found by the first decode-ordered write-VALUE differential of an INTER FIELD
 * picture (the earlier N41 field differential covered field picture 0 -- an
 * I field -- and could not see an inter-only write). Method: h264_wrlog on
 * ~/bftrace/fld.h264 picture 2 vs ~/bftrace/trace-fld.log window seq
 * 1404..2212 (the P-TOP field), ordered per (block, offset).
 *
 * Vendor, verbatim from the trace:
 *
 *   P-TOP  (curr par=top;  group A = [pn1 top, pn0 bot])
 *       seq 1754  MC+0x564 = 2      (right after the group-A entries)
 *       seq 2102  MC+0x568 = 2      (right after the group-B entries)
 *       seq 2113  MC+0x56c = 2 ; 2139 -> 3 ; 2141 -> 1   (final 1)
 *   P-BOT  (curr par=bot;  group A = [pn1 bot, pn2 top, pn0 top])
 *       seq 2559  MC+0x564 = 1
 *       seq 2898  MC+0x568 = 1
 *       seq 2909  MC+0x56c = 1 ; 2969 -> 0 ; 2971 -> 2   (final 2)
 *
 * CONTROL -- this is INTER-FIELD-ONLY, which is why the progressive path is
 * bit-exact on 67 vectors without it:
 *   - the two I fields of the SAME trace write all three = 0 (seq 62/260/261
 *     and 794/996/997);
 *   - the whole PROGRESSIVE trace (~/bftrace/trace-frm.log) writes all three
 *     = 0 on EVERY picture, P frames included.
 * This driver writes 0 unconditionally, so it is correct on progressive and
 * on an I field, and wrong on every inter field.
 *
 * Derived meaning: each register carries the PICTURE STRUCTURE of the FIRST
 * entry of its ref-list group -- 2 = top field, 1 = bottom field, 0 = frame
 * (hence 0 everywhere on the progressive path). Group C is group A with the
 * first two entries swapped, which is exactly why 0x56c settles on the
 * complementary parity of 0x564/0x568. Checked against both P fields above:
 *   P-TOP  A[0]=top -> 2, C[0]=A[1]=bot -> 1
 *   P-BOT  A[0]=bot -> 1, C[0]=A[1]=top -> 2
 *
 * eff_field_pic + fld_list gated, so a progressive picture and an I field stay
 * byte-for-byte identical to the pre-N52 driver.
 *
 * Measured on silicon, kernel 6.18.35-01242-gbf7de7a19613, same-build A/B:
 *   CVFI1_SVA_C       n:2 20.83 dB -> inf (BIT-EXACT), n:3..7 ~17-20 -> ~29-30 dB
 *   CVMAPAQP3_Sony_E  n:4/n:7/n:8 -> inf (BIT-EXACT), the rest +1..+4 dB
 *   FI1_Sony_E        bit-exact in BOTH arms (the field passer most at risk)
 * These are the FIRST bit-exact field P pictures on this decoder, so the
 * default is ON.
 */
static int h264_fldstruct = 1;
module_param(h264_fldstruct, int, 0644);
MODULE_PARM_DESC(h264_fldstruct,
		 "mt8365 H.264 N52: 1 = program the per-group reference picture-structure registers MC+0x564/0x568/0x56c on an inter FIELD picture (default 1)");

/*
 * N53 (2026-08-04): MC+0x564/0x568/0x56c are a PACKED 2-BIT-PER-PAIR ARRAY, not
 * a scalar. N52 read them off a vendor trace whose field ref-lists never exceed
 * THREE entries; at <=3 entries the vendor performs exactly ONE accumulate, so
 * the register settles on the bare structure code and the scalar reading fits.
 * The vendor trace of CVFI1_SVA_C itself (~/bftrace/trace-cvfi.log, 12610 events,
 * 14 field pictures, vendor decode bit-exact) reaches 4..10-entry field lists and
 * shows the accumulation directly: one read-modify-write after every SECOND list
 * entry, e.g. the P-TOP fn=2 picture (4-entry list A=[3,2,1,0])
 *
 *     seq 3410  R MC+0x564 = 0x00000000     (after entries 0,1)
 *     seq 3411  W MC+0x564 = 0x00000002
 *     seq 3420  R MC+0x564 = 0x00000002     (after entries 2,3)
 *     seq 3421  W MC+0x564 = 0x0000000a
 *
 * so the value is
 *
 *     reg = OR over k in [0, n/2) of  struct(list[2k]) << (2k)
 *
 * with struct = 2 (top field), 1 (bottom field), 0 (frame), n = the FIELD list
 * length and k indexing the top/bottom PAIR contributed by one DPB frame. The
 * group-C swap of the first two entries rewrites only the LOW 2-bit field
 * (0x0a -> 0x0b -> 0x09 in the same picture), which is what pins pair 0 to
 * bits[1:0] rather than to the high end.
 *
 * Verified against ALL 13 inter-field windows of that trace (26 register values,
 * lists of 1,2,3,4,5,6,7,8,9,10,9,10,9 entries): every predicted 0x564/0x568 and
 * 0x56c matches the vendor byte-for-byte -- 2,10,42,170,682 on the top fields and
 * 1,5,21,85 on the bottom ones, i.e. exactly the repeated-2-bit-field pattern.
 *
 * n<=3 reproduces the N52 scalar exactly (one pair), so this only changes an
 * inter field carrying FOUR OR MORE references -- which is precisely where the
 * post-N52 residual starts (CVFI1_SVA_C is bit-exact through n:2 and first
 * diverges at n:3, the frame whose two fields carry 4- and 5-entry lists).
 * n==1 now writes 0, matching the vendor's 1-entry P field (N52 wrote the bare
 * structure code there; the register was already 0 from the per-picture reset,
 * and that field is bit-exact in both arms).
 */
static int h264_fldpack = 1;
module_param(h264_fldpack, int, 0644);
MODULE_PARM_DESC(h264_fldpack,
		 "mt8365 H.264 N53: 1 = MC+0x564/0x568/0x56c carry the PACKED 2-bit-per-pair reference picture-structure array (default 1)");

/*
 * N106 (2026-08-20): MC+0x564/0x568/0x56c is a ONE-BIT-PER-SLOT bottom-field
 * BITMAP, not a packed 2-bit-per-PAIR array.
 *
 * Ground truth is the identified vendor PAFF trace (~/bftrace/trace-paff-n63.log,
 * CVPA1_TOSHIBA_B, 138 trigger-anchored picture windows): with bit s taken as
 * bit22 of that slot's group descriptor (AVCMV+{0x000,0x080,0x100}+4s -- the
 * same parity bit the N51 field descriptors already carry), the three region
 * masks reproduce the vendor value on 288 of 288 FIELD windows with zero
 * divergences, and the mask is written 0 on every FRAME window (0 non-zero
 * frame writes over the whole trace). The vendor accumulates it one bit per
 * entry inside its descriptor loop (0x1 -> 0x5 -> 0x15 -> 0x55 ...), which is
 * what N60's FLDRMW_WALK models.
 *
 * The N53 packed form agrees numerically whenever the field list's parity
 * alternates in pairs AND its length is EVEN, which covers every list N53/N55
 * were fitted on. It diverges in exactly two ways, both reachable today:
 *   - ODD list length: its loop runs while 2k+1 < n, so the LAST entry never
 *     contributes a bit;
 *   - non-alternating parity (a Sec 8.2.4.3.1 field reorder, default ON since
 *     N104): it derives the odd-slot bit from ref_bot[2k], not ref_bot[2k+1].
 *
 * Default ON. Clearing it restores the N53 packed form byte-for-byte, so the
 * arm is A/B-able on one kernel with no rebuild.
 */
static int h264_fldmask = 1;
module_param(h264_fldmask, int, 0644);
MODULE_PARM_DESC(h264_fldmask,
		 "mt8365 H.264 N106: 1 = MC+0x564/0x568/0x56c carry the exact per-SLOT bottom-field bitmap (default 1; 0 = the N53 packed 2-bit-per-pair form)");

/*
 * N107 (2026-08-20): the SAME encoding fix, for the N54 B-FIELD path.
 *
 * N106 fixed MC+0x564/0x568/0x56c only on the P field path. The B field path
 * (MT8365_PROG_FGROUP, below) computes its own sA/sB/sC and was deliberately
 * left on N53's packed 2-bit-per-PAIR form, so it carries the identical defect:
 * its loop runs while 2*k+1 < n (the final entry of an ODD list never
 * contributes a bit) and it derives the odd-slot bit from *_bot[2k] instead of
 * *_bot[2k+1] (wrong after a Sec 8.2.4.3.1 field reorder).
 *
 * The correction is the same one-bit-per-SLOT bottom-field bitmap, applied per
 * GROUP over that group's OWN list -- A over the field PicNum-desc list, B over
 * field RefPicList0, C over field RefPicList1. Unlike the P path there is NO
 * swap-first-two on group C here: for B pictures the vendor builds L1
 * independently (Sec 8.2.4.2.4) and group C carries that separate list, so the
 * bitmap is taken straight over fb1_bot.
 *
 * Reachable only inside `if (is_inter && fld_blist)`, i.e. on a field-coded B
 * picture with all three field lists built; every frame-coded path and the P
 * field path are byte-identical by construction. Default OFF until measured.
 */
static int h264_fldmaskb;
module_param(h264_fldmaskb, int, 0644);
MODULE_PARM_DESC(h264_fldmaskb,
		 "mt8365 H.264 N107: 1 = the B-FIELD path emits MC+0x564/0x568/0x56c as the exact per-SLOT bottom-field bitmap over each group's own list (default 0 = the N53 packed 2-bit-per-pair form)");

/*
 * N60 (2026-08-05): the MC+0x564/0x568/0x56c READ-MODIFY-WRITE WALK.
 *
 * N53 resolved the VALUE of these three registers -- a packed 2-bit-per-PAIR
 * array, reg = OR_k struct(list[2k]) << 2k -- and that model is out-of-sample
 * exact (60/60 windows, 120 values, no free parameter; N55 re-verified all
 * three groups on 36/36 B-field windows). What it does NOT reproduce is the
 * vendor's write SEQUENCE. The vendor accumulates one read-modify-write per
 * PAIR, e.g. on the 4-entry P-TOP field of trace-cvfi.log:
 *
 *     seq 3410  R MC+0x564 = 0x00000000     (after list entries 0,1)
 *     seq 3411  W MC+0x564 = 0x00000002
 *     seq 3420  R MC+0x564 = 0x00000002     (after list entries 2,3)
 *     seq 3421  W MC+0x564 = 0x0000000a
 *
 * while this driver computes the final value and writes it ONCE. Both reach
 * 0x0a. The open question this bit answers is whether the hardware latches
 * anything on the intermediate writes -- i.e. whether the SEQUENCE is
 * load-bearing or only the final register state is.
 *
 * Note what the trace itself already says: the read at seq 3420 returns
 * exactly the value written at seq 3411, so on the vendor's own bus these are
 * plain holding registers with no write side effect. This bit exists to turn
 * that reading into a pixel measurement rather than an inference.
 *
 * Default 0 = the single final write, byte-for-byte the pre-N60 driver.
 */
#define FLDRMW_WALK	0x1
static int h264_fldrmw;
module_param(h264_fldrmw, int, 0644);
MODULE_PARM_DESC(h264_fldrmw,
		 "mt8365 H.264 N60: 1 = reach MC+0x564/0x568/0x56c by the vendor's accumulating per-PAIR read-modify-write walk instead of one final write (default 0 = one write)");


/*
 * N54 (2026-08-05): B FIELD reference lists, in FIELD units.
 *
 * N51 converted the FIELD RefPicList0 to field units, but routed the conversion
 * through mt8365_build_field_p_list(), which is a P-ordering function by
 * construction (Sec 8.2.4.2.5: reference FRAMES in descending FrameNumWrap,
 * then the alternating-parity field interleave). A B field needs the SAME
 * interleave seeded by the POC-based B ordering of Sec 8.2.4.2.4, for TWO
 * lists -- and because the B path never reached the field builder at all,
 * n_avail stayed at the frame count, so the N53 packed picture-structure
 * accumulate never ran on a B field either (MC+0x564/8/c stayed 0).
 *
 * Measured, on ~/bftrace/trace-fld.log (cavlc_mot_fld0_full_B), first B field
 * (vendor win5 <-> our pic4), vendor vs the pre-N54 driver:
 *
 *   group A @AVCVLD+0x0a8   vendor [3,2,1,0]   ours [3,1]      (half length)
 *   group B @AVCVLD+0x128   vendor [1,0,3,2]   ours [1,1,3,3]  (opposite-parity
 *                                                               PicNum never
 *                                                               decremented)
 *   group C @AVCVLD+0x1a8   vendor [3,2,1,0]   ours [3,3,1,1]
 *   MC+0x564/0x568/0x56c    vendor 0,2,0a      ours 0          (no accumulate)
 *
 * This bit builds, for a B FIELD:
 *   group A = the field-unit PicNum-descending list (the P builder, unchanged),
 *   group B = the field-unit RefPicList0, group C = the field-unit RefPicList1,
 * each as Sec 8.2.4.2.4's POC ordering (L0: POC <= curr descending, then POC >
 * curr ascending; L1: the reverse) fed through the SAME Sec 8.2.4.2.5
 * parity-interleave stage the P path already uses, plus the "swap the first two
 * of L1 when L1 == L0" rule. Long-term refs follow, ascending LongTermFrameIdx,
 * interleaved the same way.
 *
 * eff_field_pic + B-slice gated, so every frame-coded picture and every I/P
 * field is byte-for-byte unchanged.
 */
static int h264_fldblist = 1;
module_param(h264_fldblist, int, 0644);
MODULE_PARM_DESC(h264_fldblist,
		 "mt8365 H.264 N54: 1 = build a B FIELD's RefPicList0/1 in FIELD units (Sec 8.2.4.2.4 POC order + 8.2.4.2.5 parity interleave) (default 1)");

/*
 * N40 PROGRESSIVE write-VALUE bitmask (2026-08-03). Candidates come from a
 * matched write-VALUE differential of the vendor progressive trace
 * (~/bftrace/trace-frm.log, cvmp_mot_frm0_full_B 720x480) against this
 * driver's own on-silicon write stream (h264_wrlog) on the SAME clip.
 * All bits are !eff_field_pic-gated, so the field path is untouched.
 *   1 = AVCVLD+0x0a0 = 0x00008000 (the vendor progressive value; this driver
 *       has always written 0x00000010, and the in-code comment claimed 0x10
 *       was "trace-verified" -- the matched trace says otherwise, see the
 *       write site in mt8365_decode()).
 *   2 = AVCVLD+0x084 POST-trigger write always sets bit23.
 *   4 = AVCVLD+0x084 PRE-trigger write always CLEARS bit23.
 *
 * Bits 2|4 come from the same N40 differential and together make the
 * progressive AVCVLD+0x084 pair vendor-exact. The vendor writes this register
 * TWICE per picture and the pair is 0xae00e007 (pre-trigger) then 0xae80e007
 * (post-trigger, immediately before the AVCVLD+0x238 commit strobe) for EVERY
 * picture of BOTH the progressive and the field trace -- IDR and non-IDR
 * alike (trace-frm.log seq 21/646, 750/1396, 1500/2166; trace-fld.log seq
 * 20/649, 753/1400, 1504/2209). So bit23 is a pre/post-trigger STROBE, not
 * the "non-IDR" flag nor the "field" flag the two in-code comments claimed.
 * This driver derives it as (non-IDR || field_pic) and writes the same value
 * to both, which diverges from the vendor on a progressive IDR (post) and on
 * a progressive non-IDR (pre).
 *
 * Default 0 = pre-round byte-for-byte until a measurement says otherwise
 * (N38: replaying the vendor's AVCVLD+0xa0 value BROKE working MBAFF).
 */
#define PROGCFG_A0	0x1
#define PROGCFG_84POST	0x2
#define PROGCFG_84PRE	0x4
static int h264_progcfg;
module_param(h264_progcfg, int, 0644);
MODULE_PARM_DESC(h264_progcfg,
		 "mt8365 H.264 N40 PROGRESSIVE write-VALUE BITMASK: 1=AVCVLD+0x0a0 vendor progressive 0x00008000, 2=AVCVLD+0x084 post-trigger bit23 always SET, 4=AVCVLD+0x084 pre-trigger bit23 always CLEAR (default 0 = pre-round byte-for-byte)");

/*
 * N41 FIELD AVCVLD+0x084 bit23 pre/post SPLIT (2026-08-03). The field-path
 * counterpart of the N40 progressive bits, and the one arm the whole bit23
 * history never tested.
 *
 * The vendor writes AVCVLD+0x084 TWICE per picture -- 0xae00e007 (bit23 CLEAR)
 * pre-trigger, 0xae80e007 (bit23 SET) post-trigger, right before the +0x238
 * commit strobe -- on every picture of the progressive trace AND every field of
 * the field trace (trace-fld.log seq 20/649, 753/1400, 1504/2209). This driver
 * derives bit23 as (non-IDR || field_pic) and writes THAT to both, so on a
 * field picture it writes bit23 = 1 / 1. The historical F1.6 experiment ("bit23
 * = 0 stalls the I-field") set bit23 = 0 in BOTH writes. So the vendor's actual
 * 0-pre / 1-post pattern has never been on silicon on the field path.
 *
 * This driver additionally emits a THIRD AVCVLD+0x084 write, a hardcoded
 * 0xae00e007 (bit23 clear) inside the AVC-VLD barrel-shifter arm, between the
 * pre and post writes. So the as-shipped field sequence is 1 -> 0 -> 1 and the
 * vendor's is 0 -> 1: the last write before the commit strobe already matches,
 * and a bit23-clear write already occurs before it. The arms below make the
 * remaining permutations reachable.
 *
 *   1 = FIELD PRE-trigger write CLEARS bit23   (with 0 -> the vendor 0-pre/1-post)
 *   2 = FIELD POST-trigger write CLEARS bit23  (1|2 = the F1.6 0/0 control)
 *   4 = the mid barrel-arm write uses the derived r_84 (bit23 = 1 on a field)
 *       instead of the hardcoded bit23-clear constant -- lets an all-ones
 *       sequence (mask 4) and a strict 0->1->1 (mask 5) be tested too.
 *   8 = emit the vendor's VLD+0x110 = 0xffffff7f sentinel pre-write. This is
 *       the ONLY write the full field write-stream VALUE differential (our
 *       h264_wrlog picture 0 on ~/bftrace/fld.h264 vs trace-fld.log, both cut
 *       at the AVCVLD+0x238 commit strobe) found the vendor emitting and us
 *       not: 1 vendor-only register, 3 ours-only, 3 constant-valued
 *       divergences (all explained), the rest allocation-dependent addresses.
 *
 * All bits are eff_field_pic-gated, so the progressive path is untouched and
 * default 0 is byte-for-byte the pre-round driver.
 */
#define FLDCFG_84PRE_CLR	0x1
#define FLDCFG_84POST_CLR	0x2
#define FLDCFG_84MID_R84	0x4
#define FLDCFG_VLD110_PRE	0x8
static int h264_fldcfg;
module_param(h264_fldcfg, int, 0644);
MODULE_PARM_DESC(h264_fldcfg,
		 "mt8365 H.264 N41 FIELD write BITMASK: 1=AVCVLD+0x084 pre-trigger write clears bit23, 2=post-trigger write clears bit23, 4=mid barrel-arm write uses derived r_84, 8=emit the vendor's VLD+0x110=0xffffff7f sentinel pre-write (default 0 = pre-round byte-for-byte)");

static int h264_mbaff = MBAFF_T68;
module_param(h264_mbaff, int, 0644);
MODULE_PARM_DESC(h264_mbaff,
		 "mt8365 H.264 N38 MBAFF write BITMASK: 1=TOP+0x68 MB-pair rows (the lever), 2=AVCVLD+0xa0 field mode (measured NEGATIVE), 4=AVCMV+0x220 bit5 (measured INERT), 8=AVCVLD+0x2d8 strobe (measured NEGATIVE) (default 1; 0 = pre-round byte-for-byte)");

/*
 * N34 mixed-slice-type A/B (2026-07-26). The HW auto-continues across every VCL
 * NAL in the access unit on ONE kick -- measured: BASQP1_Sony_C (20 slices),
 * BA1_FT_C (12), and MR1_BT_A's 9-slice I / 7-slice P pictures all decode
 * BIT-EXACT while this driver parses only the FIRST slice header and always
 * programs first_mb_in_slice = 0. So per-slice first_mb programming is NOT
 * required. What the HW cannot re-derive per slice is the reference-list
 * DESCRIPTOR set (AVCVLD+0xa8/0x128/0x1a8 + AVCMV), which this driver programs
 * once per picture from the FIRST slice.
 *
 * CABAST3_Sony_E / CABASTBR3_Sony_B carry pictures whose slices have DIFFERENT
 * slice_types (I,P,I,P and I,P,B,I in one picture). With slice 0 = I this driver
 * sets is_inter = false and programs NO reference list at all, so the picture's
 * P/B slices decode against an unprogrammed list.
 *
 *   0 = as-is (ref-list state comes from VCL NAL #0 only)
 *   1 = if VCL NAL #0 is I/SI, derive is_inter + num_ref_idx + the rplm ops from
 *       the first INTER VCL NAL in the same access unit; the slice-header
 *       registers (hbs, first_mb, qp, deblock, AVCVLD 0x98 slice_type) stay at
 *       NAL #0's values.
 *   2 = as 1, and ALSO drive AVCVLD 0x98's slice_type field from that inter
 *       slice (tests whether the HW uses the register or re-parses slice_type).
 *       FALSIFIED at N34: the decode dies outright (0 bytes out).
 *
 * N111 adds the two arms that follow, both gated on the access unit REALLY
 * being mixed (any VCL NAL whose slice_type differs from NAL #0's).  Measured
 * corpus-wide: 2 of 137 JVT-AVC_V1 vectors and 0 of JVT-FR-EXT contain such a
 * picture, so neither arm can move any other vector by construction.
 *
 *   3 = as 1, and write the RAW H.264 slice_type (the 0..4 form) into AVCVLD
 *       0x98[16:13] instead of the +5 "all slices of this picture share one
 *       slice_type" form, and take the ref-COUNT field 0x98[24:20] from the
 *       inter slice that owns the programmed lists.  This is what the vendor
 *       writes on every multi-slice picture (see the r_shdr_p comment).
 *   4 = as 3, but when the AU contains a B slice, adopt the FIRST B slice's
 *       ref-list state rather than the first inter slice's, so the B list
 *       groups (AVCVLD +0xa8/+0x128/+0x1a8) are programmed for a picture that
 *       mixes P and B slices.
 */
/*
 * DEFAULT 4 since N111. Measured on med-mt8365-pumpkin, kernel
 * 6.18.35-01355-gc2a31417d0a7, taint 0, one boot, arms A/B'd back to back:
 *
 *   arm 0  CABAST3_Sony_E hw md5 1a1b71c3401d594fd59e0de8d211a602  Fail
 *   arm 1  CABAST3_Sony_E hw md5 e7248c1ad602ba14229d169f0a562ef6  Fail
 *   arm 3  CABAST3_Sony_E hw md5 e7248c1ad602ba14229d169f0a562ef6  Fail
 *   arm 4  CABAST3_Sony_E hw md5 e95f27c3779af519c1a317adb2f699a5  Success
 *                                 == the suite gold md5, 25/25 frames
 *
 * arm 3 == arm 1 exactly, so the raw-vs-+5 slice_type ENCODING is pixel-inert
 * on this hardware; what fires is arm 4's ref-list state. The three published
 * controls (MR7_BT_B, Sharp_MP_Field_1_B, Sharp_MP_Field_2_B) reproduce their
 * gold md5 on every arm, and only 2 of 137 JVT-AVC_V1 vectors (and 0 of 69
 * JVT-FR-EXT) contain a mixed access unit at all, so nothing else can move.
 */
static int h264_mixedslice = 4;
module_param(h264_mixedslice, int, 0644);
MODULE_PARM_DESC(h264_mixedslice,
		 "mt8365 H.264 N34/N111: 0=off, 1=ref-list from first inter slice, 2=+slice_type register (falsified), 3=+raw slice_type/ref-count encoding on a mixed AU, 4=+prefer the first B slice's lists");

/*
 * N50 -- the two DISAGREEMENTS the HW read-back channel (h264_hwsh bit 3)
 * found between what the hardware's own bit reader decodes out of the slice
 * header and what this driver programs. Both default OFF: at 0 the register
 * writes are byte-for-byte the pre-N50 driver.
 *
 * h264_cabacinit: program cabac_init_idc into AVCVLD 0x9c[21:20]. This is N13
 *   re-landed. N13 was reverted as a "corpus-wide no-op" because the two CABAC
 *   vectors it was scoped on (CABA2_Sony_E, CABA3_TOSHIBA_E) carry 0 on every
 *   slice -- but the HW channel reads 2/0/1/2... on CABACI3_Sony_B (8x0, 7x1,
 *   8x2 over its 23 slices, exactly matching ffmpeg's parse), so the write is a
 *   no-op only where the value is 0, which is not corpus-wide.
 *
 * h264_l1mod: apply ref_pic_list_modification to L1. The driver has always
 *   parsed the L1 modification list and thrown it away, applying rplm to L0
 *   only; the HW channel reads a non-empty L1 modification on 14 slices of
 *   HCMP1_HHI_A.
 */
static int h264_cabacinit = 1;
module_param(h264_cabacinit, int, 0644);
MODULE_PARM_DESC(h264_cabacinit,
		 "mt8365 H.264 N50: 1=program cabac_init_idc into AVCVLD 0x9c[21:20] (DEFAULT; 0 = the pre-N50 behaviour, in which the element is decoded by the HW's own reader but never programmed)");

static int h264_l1mod = 1;
module_param(h264_l1mod, int, 0644);
MODULE_PARM_DESC(h264_l1mod,
		 "mt8365 H.264 N50: 1=apply ref_pic_list_modification to RefPicList1 (DEFAULT; 0 = the pre-N50 behaviour, L1 modification parsed and discarded)");

/*
 * N44 WEIGHTED-PREDICTION value differential (bitmask, default 0 = off).
 *
 * The 2026-07-26 WP investigation ran a PRESENCE differential and concluded the
 * vendor programs no weighted-prediction register anywhere. True, and blind to a
 * register BOTH sides write with a different VALUE. Running the ordered VALUE
 * differential of ~/bftrace/trace-cvwp1.log (vendor vpud, CVWP1_TOSHIBA_E)
 * against this driver's own h264_wrlog stream on the SAME clip, cut at the
 * AVCVLD+0x238 commit strobe, and contrasting picture 1 (the clip's only
 * SINGLE-reference weighted P slice, which we already decode BIT-EXACT) against
 * picture 2 (the first TWO-reference weighted P slice, where we diverge at
 * 6.40 dB), leaves exactly four constant-valued mismatches:
 *
 *   reg            vendor p1   vendor p2   ours p1     ours p2
 *   AVCVLD+0x090   000007e0    000007e0    000007e4    000007e4   (bit2 = the
 *                                                                  PPS
 *                                                                  weighted_pred
 *                                                                  flag)
 *   AVCVLD+0x094   00000000    00000000    00000000    00000001   <- p2 ONLY
 *   AVCVLD+0x098   0000a000    0000a000    0000a000    0010a000   <- p2 ONLY
 *                                                                  ([24:20] the
 *                                                                  L0-active
 *                                                                  count)
 *   AVCVLD+0x09c   0002007f    0002007f    00020000    00020000   ([6:0] =
 *                                                                  slice_qp_delta)
 *
 * The two p2-only rows are the discriminator: they are IDENTICAL on the picture
 * we get bit-exact and DIFFER on the first picture we get wrong. The vendor
 * holds AVCVLD+0x094 at 0 and AVCVLD+0x098[24:20] at 0 for EVERY P picture of
 * this clip (pictures 1..6 of the trace), while this driver ramps both with the
 * per-slice active-reference count. CVWP1 re-transmits a PPS before every slice,
 * so our ramp is faithful to the bitstream and the vendor's constant is not --
 * which only works because the MT8365 VDEC parses the slice header in hardware
 * (parseSliceHeaderHW), so the weights and the active-ref count reach the HW
 * from the bitstream, not from these registers. The hypothesis under test is
 * that on a weighted-prediction slice the driver-programmed L0-active count
 * FIGHTS the hardware's own parse and mis-associates weight[i] with
 * RefPicList0[i].
 *
 * Bits (all gated on the PPS weighted_pred_flag / weighted_bipred_idc, so any
 * non-weighted slice -- i.e. every currently-passing vector -- is byte-identical
 * with any bitmask value):
 *   bit0 (1) = AVCVLD+0x098[24:20] (reffld, L0-active-minus-1) forced to 0
 *   bit1 (2) = AVCVLD+0x094 forced to 0
 *   bit2 (4) = AVCVLD+0x090 bit2 RESTORED to the pre-N44 (wrong) value. The
 *              driver now follows the vendor and leaves bit2 clear by default;
 *              this bit is the A/B control that puts the old behaviour back.
 *   bit3 (8) = AVCVLD+0x098[24:20] forced to the PPS default rather than the
 *              per-slice active count (the softer form of bit0)
 *
 * N130 (2026-08-24) re-measured bits 0/1/3 (mask values 1/2/8) and the
 * combinations 3 and 10 PER PLANE, because N44 scored them on a luma-only
 * harness. Every arm was proven to FIRE by the h264_wrlog register stream
 * (CVWP1 picture 2: mask 1 moves 0x098 0x0010a000 -> 0x0000a000; mask 2 moves
 * 0x094 0x1 -> 0x0; mask 8 is inert on this clip because its PPS default
 * equals the per-slice count). None of them helps on ANY plane:
 *   mask 1  -- CAWP1/CAWP5 fail outright (0 bytes); CVWP1 luma 3 735 610 ->
 *              7 775 483, CVWP5 7 011 497 -> 7 872 815. HARMFUL.
 *   mask 2  -- byte-identical output on all five vectors. INERT.
 *   mask 8  -- inert on CAWP1/CVWP1/CVWP2; CAWP5/CVWP5 luma 7 011 497 ->
 *              7 747 236 / 7 747 920. HARMFUL where it fires.
 *   mask 3  == mask 1 byte-for-byte; mask 10 == mask 8 byte-for-byte.
 * All bits therefore stay 0 by default (unchanged). Chroma tracks luma on
 * every arm -- there is no plane-selective lever here.
 */
static int h264_wp;
module_param(h264_wp, int, 0644);
MODULE_PARM_DESC(h264_wp,
		 "mt8365 H.264 N44 weighted-pred value differential (bitmask, WP-slices only): 1=AVCVLD 0x98[24:20]=0, 2=AVCVLD 0x94=0, 4=set the 0x90 bit2 weighted-pred ENABLE (SUBSUMED by h264_wpflag, which now does this by default -- kept so the historical mask still means what it meant), 8=0x98[24:20]=PPS default; N45 (any slice): 16=RESTORE the pre-N45 clamped AVCVLD 0x98[24:20] (default now reports the SLICE's num_ref_idx_l0_active-1 even when it exceeds the available DPB refs)");

/*
 * N134 (2026-08-24): AVCVLD+0x90 bit2 is the weighted-prediction ENABLE, and it
 * is the second half of a two-part handshake whose first half only landed at
 * N132.
 *
 * The static chain, end to end, from libvpud_vcodec.so sha256
 * da6979b2d8a83cd8900e6556eb146ec68eaa273e69e8930e066de633a54ddb53:
 *   parseSliceHeaderHW@0x54860+0x350 gates its ONE pred_weight_table() call on
 *   `[x23+1404] && slice_type in {P,SP}`, where x23 is PPS[pic_parameter_set_id];
 *   parsePPS@0x51aa0+0x318 fills that byte from a RBSPGetBits(bs,1) sitting
 *   between num_ref_idx_l1_default_active_minus1 (+1400) and
 *   weighted_bipred_idc (+1408) -- i.e. it is `weighted_pred_flag` by H.264
 *   7.3.2.2 element order; and H264_HAL_SetPPSAVLD@0x4bd80+0x54 loads the same
 *   byte, shifts it left by 2, and folds it into the word it writes to
 *   AVCVLD+0x90. So the gate quantity is BOTH a host-side branch AND a register
 *   bit.
 *
 * Why N44 measured this bit HARMFUL and N134 measures it a FIX: N44 ran with
 * h264_hwsh=0, so the AVCVLD+0x230 strobe never fired and the silicon never
 * consumed pred_weight_table(). Setting the enable with no weights latched is
 * garbage in, garbage out -- which is exactly what N44 saw. N132 made the
 * strobe the built-in default. With the table latched, the enable is the piece
 * that was missing, and the two together are bit-exact:
 *
 *   vector             h264_wp=0 (control)    h264_wp=4 (bit2 set)
 *   CVWP1_TOSHIBA_E    f908d7b2.. 8/90 exact  e8868ca5.. == ffmpeg ref, 90/90
 *   CAWP1_TOSHIBA_E    8138e965.. 9/90 exact  87946607.. == ffmpeg ref, 90/90
 *   CVWP5_TOSHIBA_E    b31def3c.. 7/90 exact  6161f301.. 7/90 (moves, not fixed)
 *   CAWP5_TOSHIBA_E    b31def3c.. 7/90 exact  6161f301.. 7/90 (moves, not fixed)
 *
 * fluster JVT-AVC_V1 on those four: Ran 0/4 -> Ran 2/4, CVWP1 and CAWP1 both
 * Fail -> Success, same kernel and same boot, ten standing controls bit-exact
 * on both legs.
 *
 * The arm is proven to fire AT THE REGISTER, not at a call site: h264_wrlog on
 * CVWP1 gives `AVCVLD +0x090 = 0x000007e4` on each weighted P picture against
 * `0x000007e0` on the control, with the I picture staying 0x000007e0 on both
 * legs, and the H264ADV consuming-read stream byte-identical between legs (8
 * accesses before the strobe, strobe = 1, 2 after) -- so this lever moves no
 * bit reader and cannot disturb CABAC alignment.
 *
 * Honest residue: the vendor's own capture (~/bftrace/trace-cvwp1.log) writes
 * 0x000007e0 -- bit2 CLEAR -- on all seven pictures of this same clip, which it
 * decodes bit-exact, and it never fires the strobe either. So the vendor
 * reaches the right answer by a route we still do not have; ours needs both
 * halves. That is a difference in route, not a contradiction in the RE: the
 * packing above is what the disassembly says the field is, and setting it is
 * what makes our decode match the reference.
 */
/*
 * N135: build RefPicList0 to its FULL declared length on a P/SP slice whose
 * num_ref_idx_l0_active EXCEEDS the available DPB references, per §8.2.4.3.1 --
 * duplicates included. See mt8365_build_l0_overlong(). Only reachable on a
 * slice the driver used to truncate, so every other stream is byte-identical at
 * either setting.
 *
 * DEFAULT 1 since N135, on a paired-control A/B on one boot of
 * 6.18.35-01366-gcc5fec57f245-dirty: CVWP5_TOSHIBA_E and CAWP5_TOSHIBA_E both
 * go 6161f3013d5c5b27c91bb4994651b459 (exact_all3 7/90) ->
 * 9663da55e5ef516c8bf0ca0bcc0abbb8 = the board-local ffmpeg reference,
 * BIT-EXACT on all three planes over 90 frames; CVWP1/CAWP1 unchanged and
 * bit-exact on both legs; fluster on the four capped WP-P vectors
 * Ran 2/4 -> Ran 4/4; twelve standing controls "12 checked, 0 MISMATCH".
 */
static int h264_l0dup = 1;
module_param(h264_l0dup, int, 0644);
MODULE_PARM_DESC(h264_l0dup,
		 "mt8365 H.264 N135: on a P/SP slice whose num_ref_idx_l0_active EXCEEDS the available DPB references, build the full-length Sec 8.2.4.3.1 RefPicList0 (the surplus indices hold a SECOND copy of an already-listed reference, each with its own pred_weight_table() entry) and program all of it, instead of clamping the list to the available references (default 1 = ON, which makes CVWP5/CAWP5 bit-exact; 0 = the pre-N135 clamp)");

/* N135 instruments: candidates seen, and lists actually rebuilt. */
static int h264_l0dupcand;
module_param(h264_l0dupcand, int, 0444);
MODULE_PARM_DESC(h264_l0dupcand,
		 "mt8365 H.264 N135 instrument: P/SP slices whose num_ref_idx_l0_active exceeds the available DPB references (counted even when the arm is off -- the denominator)");

static int h264_l0duphit;
module_param(h264_l0duphit, int, 0444);
MODULE_PARM_DESC(h264_l0duphit,
		 "mt8365 H.264 N135 instrument: slices whose RefPicList0 was actually rebuilt to its full declared length");

static int h264_wpflag = 1;
module_param(h264_wpflag, int, 0644);
MODULE_PARM_DESC(h264_wpflag,
		 "mt8365 H.264 N134: pack the PPS weighted_pred_flag into AVCVLD+0x90 bit2 (the weighted-prediction ENABLE) on explicitly weighted slices (default 1 = ON, which with the N132 h264_hwsh strobe makes CVWP1/CAWP1 bit-exact; 0 = the pre-N134 behaviour, bit2 always clear)");

/* Per-frame-store colocated-MV buffer pool depth. Up to 16 DPB refs + the
 * current frame-store = 17 distinct frame-stores can be LIVE at once, but the
 * decode-order ring counter (mv_ring_next) advances on EVERY decoded frame
 * (including the non-ref B frames), so with a large max_num_ref_frames and a
 * deep B GOP the ring index wraps long before a still-live reference leaves the
 * DPB. Size the pool comfortably above the 17-live ceiling so the liveness-aware
 * assignment in mt8365_mv_assign() can always re-home the current frame to a
 * slot NOT owned by a live reference (never stranding a live ref's colMV). 32
 * gives ample headroom over any DPB (and over the gst CAPTURE-buffer count) and
 * stays well within the 9-bit descriptor slot field ([31:23], max 511). N16/B5:
 * HCMP1_HHI_A / HCBP1/2_HHI_A carry max_num_ref_frames=15 with 8 non-ref B per
 * GOP, so the old 17-slot mod-ring stole live refs' slots -> temporal-direct
 * mis-derivation (fallback slot 0). */
#define MT8365_MV_POOL_NUM	32

struct vdec_h264_mt8365_inst {
	struct mtk_vcodec_dec_ctx *ctx;
	struct mtk_h264_sps_param sps;
	struct mtk_h264_pps_param pps;
	struct slice_api_h264_scaling_matrix scaling;
	struct slice_api_h264_decode_param decode_params;
	struct mtk_h264_dpb_info dpb[V4L2_H264_NUM_DPB_ENTRIES];
	/* HW working buffers -- driver-allocated scratch DMA the VDEC needs for
	 * motion-vector / deblock / colocated-MV storage. The trace points
	 * MISC+0x54/0x58 (MV/deblock) + MC+0xa20/0xa24 (colocated) at vpud-allocated
	 * DMA (full phys). Sized from the picture dims on first decode, reallocated
	 * on a resolution change. [0]=MV [1]=deblock [2],[3]=colocated. */
	struct mtk_vcodec_mem work_buf[5];
	bool work_buf_ready;
	u32 work_buf_w, work_buf_h;
	/*
	 * PER-FRAME-STORE colocated-MV buffer pool. The vendor vpud keeps a
	 * distinct colMV buffer for every DPB frame-store: when a frame decodes
	 * it stores its colocated MVs to its own buffer (AVCMV+0x20c), and when
	 * that frame is later used as a reference the ref-list descriptor
	 * (AVCMV+0x180..) points back at THAT buffer. The MT8365 H.264 HW
	 * consults a reference's colMV buffer even for P frames (empirically:
	 * a nonzero AVCMV descriptor with the wrong +0x180 address corrupts even
	 * a single-ref P frame). Our old single-buffer model (work_buf[4] for
	 * every +0x180 slot) could not supply a ref's own colMV, so activating
	 * the descriptors regressed f1. Key each pool slot by the owning
	 * frame-store's recon-Y dma_addr; the slot INDEX is what the descriptor
	 * high field encodes (AVCMV +0x000 bits[31:23], = the vendor's colMV
	 * ring slot). Sized like work_buf; realloc'd on a res change.
	 */
	struct mtk_vcodec_mem mv_pool[MT8365_MV_POOL_NUM];
	u32 mv_pool_key[MT8365_MV_POOL_NUM];	/* recon-Y dma_addr of owner, 0 = free */
	/*
	 * N59: was the picture that WROTE this frame-store coded as a FIELD
	 * (field_pic_flag = 1)? The V4L2 DPB entry does not carry it, and the
	 * vendor's FIELD-unit reference descriptor needs it (AVCMV bit19, see
	 * PAFF_B19FLD). Recorded per colMV pool slot because that slot is
	 * already keyed by the frame-store's recon-Y dma_addr, which is exactly
	 * the identity a reference is looked up by (mt8365_mv_lookup). A
	 * complementary field pair shares one recon-Y buffer, hence one slot,
	 * and both of its fields agree on the flag.
	 */
	u8 mv_pool_fld[MT8365_MV_POOL_NUM];
	/*
	 * Global DECODE-ORDER ring counter. The vendor keys each frame-store's
	 * colMV ring slot by the decode-order index (mod pool), NOT by first-free
	 * compaction: the CVBS3 read-trace shows I0->slot0, P3->1, P6->2, P9->5,
	 * P12->8, P15->11 (the gaps are the interleaved non-ref B frames, which
	 * also consume a slot). The descriptor high field the HW consults for
	 * temporal-direct co-located derivation encodes THIS slot, so it must be
	 * globally stable across the whole stream -- address-keyed first-free
	 * assignment diverges the moment CAPTURE buffers start recycling (~n:20+,
	 * exactly the DPB-full 4-ref failure onset). N16/B4.
	 */
	u32 mv_ring_next;
	/*
	 * N68: identity of the IMMEDIATELY PRECEDING decoded picture's colMV
	 * slot. In PAFF the two fields of one frame-store are always adjacent
	 * in decode order, so "the previous picture was field-coded, had this
	 * recon-Y, and owns slot i" is the exact test for "i belongs to my
	 * complementary field" -- as opposed to a stale key left on a RECYCLED
	 * recon buffer by a long-evicted frame-store, which matches on address
	 * alone and must not be shared with.
	 */
	u32 mv_last_y;
	int mv_last_slot;
	u8 mv_last_fld;
};

/*
 * N40 write-VALUE log. The systematic form of the accident that produced the
 * AVCVLD+0x0a0 lever: dump EVERY register write of the first N pictures in the
 * same (block, +offset, = value) shape as the vendor regtrace logs, so our
 * on-silicon write stream can be value-diffed against ~/bftrace/trace-frm.log
 * register-for-register. A write-set PRESENCE diff cannot see a wrong constant;
 * only this can. Default 0 = no logging, zero cost (one predicted branch).
 */
static int h264_wrlog;
module_param(h264_wrlog, int, 0644);
MODULE_PARM_DESC(h264_wrlog,
		 "mt8365 H.264 N40: log the FULL register write stream of the first N decoded pictures as 'H264WR <blk> +0xoff = 0xval' (0 = off)");
/*
 * N45: log WINDOW start. h264_wrlog alone can only capture a PREFIX of the
 * clip, and the printk cost is high enough that >2 logged pictures per decode
 * makes gstreamer's bitstream allocator fail the whole pipeline
 * ("Not enough memory to decode H264 stream", measured 2026-08-04 at
 * h264_wrlog=14). To diff a LATE boundary -- CAWP1_TOSHIBA_E is bit-exact
 * through output frame n:13 (picture 12) and wrong from n:14 (picture 13) --
 * the window must start where the boundary is, not at picture 0.
 * Logged pictures are those with index in [h264_wrlog_from,
 * h264_wrlog_from + h264_wrlog).
 */
static int h264_wrlog_from;
module_param(h264_wrlog_from, int, 0644);
MODULE_PARM_DESC(h264_wrlog_from,
		 "mt8365 H.264 N45: first picture index of the h264_wrlog window (default 0)");

/*
 * N137 -- the PER-PICTURE DIAGNOSTIC LOG, and why it is now OFF by default.
 *
 * Four bring-up diagnostics were emitted at mtk_v4l2_vdec_err() level on EVERY
 * picture of a completely healthy, bit-exact decode:
 *
 *   "mt8365 BS va=... dma=... sz=... first=<24 bytes>"   (~190 chars)
 *   "mt8365 VLD fetch-done OK after N; AVCVLD+0x25c=..."
 *   "mt8365 CABAC align hbs N -> M"      (fires on every CABAC clip)
 *   "mt8365 hwsh pos=N != hbs=M (pad K)" (fires on every padded slice header)
 *
 * None of them is an error.  All four are on the SUCCESS path.
 *
 * printk to a console is SYNCHRONOUS, and this board boots
 * console=ttyS0,115200n8, so each line costs its own length in serial bits:
 * ~10 ms for a short line and ~26 ms for the 190-char BS dump.  Measured on
 * 6.18.35-01367-g665a77cbe31c with ftrace function_graph: __dev_printk 18.2 ms
 * per call, vdec_h264_mt8365_decode 54.5 ms per picture, and
 * mtk_vcodec_wait_for_done_ctx -- the actual wait on the silicon -- only
 * 0.115 ms.  The hardware was never the slow part; the console was, at
 * 74-76 % of the whole decode wall time:
 *
 *   HPCA_BRCM_C  (352x288, 300 pictures)   20.235 s -> 5.226 s
 *   LS_SVA_D     (176x144, 1700 pictures)  64.386 s -> 15.295 s
 *
 * measured as a paired A/B on ONE boot with `dmesg -n 1` as the only variable,
 * output md5 IDENTICAL on both legs.  That cost is what put LS_SVA_D,
 * HPCAFL_BRCM_C and HPCAFLNL_BRCM_C over fluster's series-standard `-t 30`
 * while decoding bit-exactly (N136).
 *
 * So the default is 0 = silent.  Set h264_diaglog=1 to get the bring-up log
 * back; it changes no register write and no decoded sample either way, which
 * is why it needs no A/B arm.  Genuine failures -- the VLD fetch-done timeout,
 * the decode timeout dump, every "stuck" gate -- are NOT gated and still print
 * unconditionally.
 */
static int h264_diaglog;
module_param(h264_diaglog, int, 0644);
MODULE_PARM_DESC(h264_diaglog,
		 "mt8365 H.264 N137: emit the four PER-PICTURE bring-up diagnostics (BS dump, VLD fetch-done OK, CABAC align, hwsh pad) on the SUCCESS path. Default 0 = off; they cost 74-76% of the decode wall time on a 115200 serial console. Real errors print regardless");

/*
 * Step-10 CONTROL instrument (HEVC M3b): a CPU read of VDEC_LD at the HEVC
 * pre-kick perturbs OUR HEVC decode while the same read is inert for the
 * vendor's. That asymmetry only indicts the HEVC backend if the read is inert
 * on a decode path that WORKS -- so this reads the same VDEC_LD window at the
 * H.264 pre-kick, where the backend produces bit-exact pictures, and the
 * output md5 answers whether the perturbation is HEVC-specific or a property
 * of reading VDEC_LD from our driver at all.
 *
 * Default 0 = off. Offsets are BLOCK-RELATIVE into VDEC_LD and are clamped to
 * [0x100, 0x800): +0x000..+0x0ff hard-resets the SoC (step 9 §1.2) and is
 * refused here rather than left to the caller.
 */
static int mt8365_h264_ldread;
module_param(mt8365_h264_ldread, int, 0644);
MODULE_PARM_DESC(mt8365_h264_ldread,
		 "mt8365 H.264 step10: read VDEC_LD at the pre-kick of the first picture (0 = off)");

static int mt8365_h264_ldread_off = 0x100;
module_param(mt8365_h264_ldread_off, int, 0644);
MODULE_PARM_DESC(mt8365_h264_ldread_off,
		 "mt8365 H.264 step10: block-relative start of the VDEC_LD read (clamped to >= 0x100)");

static int mt8365_h264_ldread_len = 0x100;
module_param(mt8365_h264_ldread_len, int, 0644);
MODULE_PARM_DESC(mt8365_h264_ldread_len,
		 "mt8365 H.264 step10: length in bytes of the VDEC_LD read");

static int h264_ld_cand;
module_param(h264_ld_cand, int, 0644);
MODULE_PARM_DESC(h264_ld_cand,
		 "mt8365 H.264 step10: pre-kicks reached with the instrument compiled in (arm-OFF denominator)");

static int h264_ld_lines;
module_param(h264_ld_lines, int, 0644);
MODULE_PARM_DESC(h264_ld_lines,
		 "mt8365 H.264 step10: 8-word VDEC_LD lines actually read (survival counter)");

/*
 * N100 -- the AVC_VLD per-stage state registers.
 *
 * The mt8366 / mt8391 / mt8188 / mt8395 blobs ship vdec_hal_dump_h264_reg with
 * a NAMED format-string table for the AVC_VLD register file; MT8365's blob has
 * none. Transcribed verbatim from mt8366's libvpud_vcodec.so:
 *
 *   244(145) -            248(146) slice hdr state   24C(147) weighted pred state
 *   250(148) -            254(149) slice data state  258(150) mb state
 *   25C(151) mb pred st   260(152) residual state    264(153) residual blk state
 *   26C(155) -            270(156) Error msg         274(157) Dec complete
 *   278(158) Dec complete 280(160) Error mask        284(161) Error acc + MB ErrCnt
 *   29C(167) Residual checksum                       2A0(168) Bs-in checksum
 *
 * The index in parentheses is offset/4, which is how the file is confirmed to
 * be this same AVC_VLD = VDEC_AD @0x16022000 (NOT VDEC_LD @0x16020000, whose
 * +0x000..+0x0ff is the hard-reset trap). Seven of those offsets -- 0x244
 * 0x248 0x24c 0x250 0x254 0x258 0x260 -- had never been read here.
 *
 * These are STAGE state, so they are read POST-decode. Default OFF; the
 * candidate counter advances unconditionally so the arm-off denominator is
 * observable.
 */
static int mt8365_h264_stage;
module_param(mt8365_h264_stage, int, 0644);
MODULE_PARM_DESC(mt8365_h264_stage,
		 "mt8365 H.264 N100: read the AVC_VLD (VDEC_AD) per-stage state registers after each completed picture (0 = off)");

static int h264_stage_cand;
module_param(h264_stage_cand, int, 0644);
MODULE_PARM_DESC(h264_stage_cand, "mt8365 H.264 N100: completed pictures reached with the instrument compiled in (arm-OFF denominator)");

static int h264_stage_appl;
module_param(h264_stage_appl, int, 0644);
MODULE_PARM_DESC(h264_stage_appl, "mt8365 H.264 N100: pictures where the stage-state read was performed");

static int h264_stage_lines = 8;
module_param(h264_stage_lines, int, 0644);
MODULE_PARM_DESC(h264_stage_lines, "mt8365 H.264 N100: how many pictures report the stage state");

/*
 * N31 positive control -- the SAME post-decode Y/Cb/Cr digest the HEVC backend
 * arms (VDEC_MISC +0x04 bits[1:0] enable, 8 words at VDEC_MISC +0x08..+0x24).
 * VDEC_MISC is one block shared by every codec on this SoC, so arming it on a
 * path that is known to produce bit-exact pixels is what makes a null HEVC
 * reading interpretable: a digest that does not move HERE is not wired.
 * Default OFF -- this writes two bits the vendor never writes.
 */
static int mt8365_h264_crc;
module_param(mt8365_h264_crc, int, 0644);
MODULE_PARM_DESC(mt8365_h264_crc,
		 "mt8365 H.264 N31 control: value for VDEC_MISC+0x04 bits[1:0] (0 = off, 1 = CRC enable, 3 = enable|mode)");

static int h264_crc_cand;
module_param(h264_crc_cand, int, 0644);
MODULE_PARM_DESC(h264_crc_cand, "mt8365 H.264 N31 control: triggers reached with the CRC instrument compiled in (arm-OFF denominator)");

static int h264_crc_appl;
module_param(h264_crc_appl, int, 0644);
MODULE_PARM_DESC(h264_crc_appl, "mt8365 H.264 N31 control: triggers where the CRC enable was actually written");

static int wrlog_idx;		/* decoded-picture index (always advances) */
static int wrlog_on;		/* logging armed for the picture in flight */

/*
 * N47 PicNum-wrap bitmask (2026-08-04). Two independent defects, both about
 * the H.264 PicNum number space, both measured this session:
 *
 *   bit0 PICNUM_SIGNED -- PicNum is FrameNumWrap and is NEGATIVE for every
 *      short-term reference whose FrameNum exceeds the current frame_num
 *      (H.264 Sec 8.2.4.1). This driver read it out of
 *      `struct slice_h264_dpb_entry.pic_num`, which is an `unsigned short`
 *      (vdec_h264_req_common.h) holding the 16-bit TRUNCATION of that negative
 *      -- measured on silicon: MR3_TANDBERG_B picture 176 programs
 *      AVCVLD+0x0a8.. = [0, 65535, 65534, 65533, 65532, 65529, 65528, 65527,
 *      65526, 65525, 65523, 65522, 65517], i.e. -1,-2,-3,-4,-7..-11,-13,-14,-19
 *      truncated to 16 bits, where the vendor regtrace writes the 19-bit two's
 *      complement (0x7fffd = -3, 0x7fff0 = -16 in ~/bftrace/trace-mr1.log).
 *      Consequences, all three fixed here by deriving PicNum from the untruncated
 *      `frame_num` syntax element instead: the initial-list sort compares
 *      unsigned and inverts Sec 8.2.4.2.1; the Sec 8.2.4.3.1 short-term reorder
 *      search compares its (signed) target against the truncated value and
 *      silently finds nothing; and the descriptor carries 0x0fff2 where the HW
 *      wants 0x7fff2.
 *      MR3_TANDBERG_B (log2_max_frame_num = 8) restarts frame_num 242 -> 0 at
 *      picture 175: it is bit-exact for all 176 pictures BEFORE that and wrong
 *      from picture 176, the first picture whose list mixes wrapped and
 *      un-wrapped references.
 *
 *   bit1 PICNUM_MAXPIC -- AVCVLD+0x0a0 is MaxPicNum, not the 0x10 bring-up
 *      constant this driver has always written. Transcribed from four vendor
 *      `vpud` regtraces against each clip's SPS log2_max_frame_num:
 *        MR1_BT_A     log2_max_frame_num 5 -> 0x00000020 = 32
 *        MR3_TANDBERG_B                  8 -> 0x00000100 = 256
 *        HCBP1_HHI_A                     9 -> 0x00000200 = 512
 *        CVWP1_TOSHIBA_E                 4 -> 0x00000010 = 16   (= our constant)
 *      and the FIELD/MBAFF traces write 2*MaxFrameNum (trace-frm progressive
 *      0x00008000 vs trace-fld field 0x00010000 on the same SPS), i.e. MaxPicNum
 *      per Sec 7.4.3. The existing 0x00010000 field special-case is that same
 *      law evaluated for one clip family. 22 of the 135 JVT vectors have
 *      log2_max_frame_num == 4, so the constant is right for those and wrong for
 *      the rest.
 *
 * Measured, kernel 6.18.35-01233-g82a03b316951 at these defaults, fluster
 * JVT-AVC_V1 "Ran 71/135 tests successfully, in 606.465 secs" against the same
 * kernel at h264_picnum=0 (= the pre-N47 driver) "Ran 67/135 ... in 606.228
 * secs": gained CABA3_TOSHIBA_E, CVWP3_TOSHIBA_E, MR1_BT_A, MR3_TANDBERG_B and
 * lost nothing (passer-set comm empty in the other direction). VP8 61/61, VP9
 * 272/305, taint 0. The two unscoped gains are both log2_max_frame_num > 4
 * clips, i.e. the MaxPicNum constant had been costing conformance beyond the
 * two vectors under investigation.
 */
#define PICNUM_SIGNED	0x1
#define PICNUM_MAXPIC	0x2
static int h264_picnum = PICNUM_SIGNED | PICNUM_MAXPIC;
module_param(h264_picnum, int, 0644);
MODULE_PARM_DESC(h264_picnum,
		 "mt8365 H.264 N47 PicNum bitmask: 1=PicNum derived as signed FrameNumWrap from frame_num (list order + rplm search + descriptor), 2=AVCVLD+0x0a0 = MaxPicNum from the SPS (default 3 = both; 0 reproduces the pre-N47 driver byte-for-byte)");

/*
 * N56 PAFF frame<->field transition (2026-08-05). The six PAFF vectors emit ZERO
 * frames: the leading FIELD pictures decode, then the first FRAME-coded picture
 * HW-times-out at MB(8..9,0) with AVCVLD+0x284 = 0x00800000 (bit 23, inside the
 * vendor's own error mask 0x7fffbcb7). The vendor `vpud` decodes the same
 * vectors BIT-EXACT vs the JVT md5 on this silicon (CVPA1_TOSHIBA_B,
 * Sharp_MP_PAFF_1r2, 2026-08-05), so the class is reachable.
 *
 * A vendor regtrace of CVPA1_TOSHIBA_B (~/bftrace/trace-paff.log) value-diffed
 * against this driver's own on-silicon write log for the SAME clip and the SAME
 * picture (h264_wrlog) leaves exactly TWO divergent words on the first frame
 * picture; both are armed here, both default OFF, and both are structurally
 * inert on every non-PAFF-frame picture:
 *
 *   bit0 PAFF_T68 -- TOP+0x68 rows: ours 0x00110015, vendor 0x00080015.
 *   bit1 PAFF_A0  -- AVCVLD+0x0a0: ours 0x00000010, vendor 0x00000020 (this
 *                    clip's MaxFrameNum is 16, so the vendor emits MaxFrameNum
 *                    on a FIELD picture and 2*MaxFrameNum on a FRAME picture --
 *                    the exact INVERSE of the N47 MaxPicNum model, which emits
 *                    0x20 on our fields and 0x10 on our frame). Only the FRAME
 *                    side is flipped here; the field side is a separate arm
 *                    (bit2) because the seven passing field vectors ride it.
 *   bit2 PAFF_A0F -- also emit the vendor FIELD value (MaxFrameNum) on a PAFF
 *                    field picture. Touches the field passers; measure alone.
 */
#define PAFF_T68	0x1
#define PAFF_A0		0x2
#define PAFF_A0F	0x4
/*
 * N58 PAFF FRAME-picture reference arrays in FIELD units (bit3, PAFF_FLDREF).
 *
 * Re-derived from ~/bftrace/trace-paff.log with the entry COUNT read off the
 * AVCVLD PicNum array (+0xa8/+0x128/+0x1a8, one word per reference) rather than
 * off the AVCMV descriptor array (which the vendor zero-fills to 32 words first,
 * so a "non-zero scan" of it overruns into the next group). Per window:
 *
 *   win0 FIELD  gA n=1   AVCMV A = 00080000                       (1 word/ref)
 *   win2 FRAME  gA n=3   AVCMV A = 00c00000 00800000 00080000     (1 word/ref)
 *   win3 FIELD  gA n=3   AVCMV A = 00c00000 00080000 00800000     (1 word/ref)
 *   win4 FIELD  gA n=1   AVCMV A = 00a00000 00e00000              (2 words/ref)
 *   win5 FRAME  gA n=3   AVCMV A = 00c00000 00080000 00800000     (1 word/ref)
 *   win6 FIELD  gA n=4   AVCMV A = 00080000 02480000 00800000 00c00000
 *
 * So the AVCMV stride is per-REFERENCE, not per-picture-type: a reference that
 * occupies ONE word (bit21 clear, 4-byte stride) is a FIELD reference, and one
 * that occupies TWO (bit21 set, 8-byte stride, a single PicNum entry) is a FRAME
 * reference. Both vendor FRAME windows (win2, win5) program THREE field-unit
 * references; ours programs a single frame-unit reference (two words,
 * 00a00000/00e00000, zero payload in groups B and C).
 *
 * This arm makes a PAFF FRAME-coded picture take the SAME separable field
 * stages the N51 (P) and N54 (B) field paths already use -- the Sec 8.2.4.2.5
 * parity interleave and the Sec 8.2.4.2.4 POC ordering -- with the current
 * parity taken as TOP (eff_bottom_field is 0 on a frame picture) and the frame
 * POC of Sec 8.2.1 (Min(Top,Bottom)) as the B-list pivot. Everything downstream
 * (the FIELD descriptor encoding, the N53 packed picture-structure words, the
 * per-field colMV parity bit and the region A/B/C recon fill) is reached through
 * the existing fld_list / fld_blist flags, so nothing is forked.
 *
 * Gated on !FRAME_MBS_ONLY && !MB_ADAPTIVE && !field_pic, i.e. ONLY a
 * frame-coded picture of a PAFF stream: a progressive stream, an MBAFF stream
 * and every field picture are byte-for-byte untouched by construction.
 *
 * NOT claimed: a same-picture value diff. trace-paff.log does not start at the
 * clip's first picture (its first TOP+0x68 is preceded by 582 events, i.e. the
 * tail of a picture whose setup is off the front of the capture), so this arm is
 * verified on FORM -- entry count, flag form, payload shape, picstruct walk --
 * not on value equality against a named vendor window.
 */
#define PAFF_FLDREF	0x8
/*
 * N59 FIELD-unit descriptor bit19 = "the REFERENCE was FIELD-coded" (bit4,
 * PAFF_B19FLD).
 *
 * The N51/N54 field paths set AVCMV bit19 (0x00080000) on EVERY field-unit
 * reference descriptor, transcribed from trace-fld.log where it is indeed set on
 * every entry. trace-paff.log shows that is a special case, not the rule: within
 * a SINGLE window the vendor sets bit19 on some field-unit entries and clears it
 * on others. Read against the colMV slot each entry carries (bits 31:23), the
 * split is exact and has no free parameter -- all 24 non-zero descriptor words of
 * the seven complete windows obey it:
 *
 *   win2 FRAME  00c00000 (sl1.bot)  00800000 (sl1.top)  00080000 (sl0.top)
 *   win3 FIELD  00c00000 (sl1.bot)  00080000 (sl0.top)  00800000 (sl1.top)
 *   win4 FIELD  00a00000 00e00000   (sl1, the TWO-word FRAME form)
 *   win5 FRAME  00c00000 (sl1.bot)  00080000 (sl0.top)  00800000 (sl1.top)
 *   win6 FIELD  00080000 (sl0.top)  02480000 (sl4.bot)  00800000 (sl1.top)
 *                                                       00c00000 (sl1.bot)
 *
 * Slot 1 -- the only frame-store the vendor ever addresses with the two-word
 * FRAME form (win4) -- NEVER carries bit19, in either parity, in any window.
 * Slots 0 and 4 always do. So bit19 is a property of the REFERENCE, and the
 * property is "that picture was coded as a field". The competing reading, "only
 * one parity of this reference is in the list", fits trace-paff but is FALSIFIED
 * by trace-fld.log, where both parities of the same slot appear and BOTH carry
 * bit19 (36/36 windows).
 *
 * This arm therefore emits bit19 only when the referenced frame-store was
 * written by a field-coded picture (inst->mv_pool_fld[]). On a pure-field stream
 * every reference is field-coded, so the arm is INERT there by construction --
 * the seven field passers cannot move. It bites only on a PAFF stream, which is
 * where the residual is.
 */
#define PAFF_B19FLD	0x10
/*
 * N61 FRAME-form descriptor bit19 = "this reference was FIELD-coded" (bit5
 * PAFF_FRMB19 on the frame-coded B path, bit6 PAFF_FRMP19 on the frame-coded
 * P path).
 *
 * N61 named the B defect: a B picture is broken iff its co-located picture's
 * colMV must be consumed across a FIELD/FRAME coding-form boundary. On
 * CVPA1_TOSHIBA_B's frame-coded B pictures the split is exact and has no free
 * parameter -- 17/17 bit-exact have a FRAME-coded co-located picture, 7/7
 * broken have a FIELD-coded one, with nl0/nl1 constant at 5/5 across the
 * split. An INTRA co-located picture masks the defect entirely (the colMV is
 * never read), which is what identifies the colMV path rather than
 * bi-prediction reconstruction.
 *
 * On a frame-coded picture the driver takes the frame descriptor path
 * unconditionally: hi = 0x00200000 | (lt << 20) | (slot << 23), i.e. bit21 =
 * FRAME form, two consecutive AVCMV words (top then bottom parity). A
 * reference that was itself coded as a FIELD is therefore described to the HW
 * with no indication that its colMV surface is stored in field form -- exactly
 * the population measured broken, and the vendor decodes it bit-exact.
 *
 * This arm ORs bit19 (0x00080000) INTO the existing frame descriptor when
 * inst->mv_pool_fld[slot] says the frame-store was written by a field-coded
 * picture. It does NOT change the array form and does NOT change the two-word
 * FRAME stride -- that distinguishes it from bit3 PAFF_FLDREF (which rebuilds
 * the arrays in field units, measured 52 -> 7) and from bit4 PAFF_B19FLD
 * (which CLEARS bit19 on the FIELD path, measured 52 -> 51).
 *
 * Inert by construction on progressive streams (no field-coded frame-store can
 * exist, so mv_pool_fld[] is 0 everywhere) and on pure-field streams (the frame
 * descriptor path is not taken at all). It can only bite on a mixed PAFF
 * stream, which is where the frame-B half of the residual lives.
 *
 * bit5 covers the B path (is_inter && use_l1) -- the colMV consumer, and the
 * arm the mechanism predicts. bit6 additionally covers the frame P path, whose
 * pictures are all already bit-exact; it exists only so the two can be
 * separated by measurement rather than by assumption.
 *
 * Instrumented: h264_b19frm counts frame-path descriptors whose reference was
 * field-coded (the CANDIDATES -- incremented even when the arm is off, so the
 * predicate's reachability is observable at built-in defaults), h264_b19hit
 * counts the descriptors that actually received the extra bit19. Both are
 * readable at /sys/module/mtk_vcodec_dec/parameters/.
 */
#define PAFF_FRMB19	0x20
#define PAFF_FRMP19	0x40
/*
 * N57: bit0 (PAFF_T68) is DEFAULT-ON. Measured on-silicon, whole battery:
 * it takes all six PAFF vectors from ZERO output + a HW timeout to their FULL
 * frame count with zero timeouts, while fluster JVT-AVC_V1 stays Ran 85/135
 * with a byte-identical passer set (comm empty both ways), VP8 61/61 and the
 * VP9 control unchanged. bit1 measured INERT (paff=3 byte-identical to paff=1)
 * and bit2 touches the seven field passers, so both stay default-OFF.
 *
 * N70: bit4 (PAFF_B19FLD) and bit5 (PAFF_FRMB19) are now DEFAULT-ON as well,
 * as the READ half of the vendor-shaped colMV path (with h264_mvpair=3,
 * h264_mvstore=1, h264_refpoc=1, h264_fldpoc=1 -- all five are ONE mechanism
 * and only make sense together). Measured on-silicon as one battery with all
 * five in force, kernel 6.18.35-01266-gad5b273d59c4: fluster JVT-AVC_V1
 * Ran 85/135 in 662.740 secs with a comm-identical 85-name passer set (both
 * directions empty), VP8 Ran 61/61, VP9 Ran 272/305 with a name-identical
 * 33-vector non-passer set, the two VP9 bit-exact inter clips md5-unchanged,
 * CVMP_MOT_FLD_L30_B BIT-EXACT (7a2ff88bf2a85190b690db602323124f, 30 frames),
 * taint 0, 0 HW timeouts -- and on CVPA1_TOSHIBA_B 38 of 90 frames improve by
 * +0.04 .. +13.14 dB with ZERO frames worse (inf_count 52/90 either way).
 */
static int h264_paff = PAFF_T68 | PAFF_B19FLD | PAFF_FRMB19;
module_param(h264_paff, int, 0644);
MODULE_PARM_DESC(h264_paff,
		 "mt8365 H.264 N56 PAFF frame<->field bitmask: 1=TOP+0x68 rows from the SPS map-unit count (vendor-exact on a PAFF frame picture; DEFAULT ON since N57), 2=AVCVLD+0x0a0 = 2*MaxFrameNum on a PAFF frame picture, 4=AVCVLD+0x0a0 = MaxFrameNum on a PAFF field picture, 8=N58 build a PAFF FRAME picture's ref arrays in FIELD units, 16=N59 FIELD-unit descriptor bit19 only when the REFERENCE was field-coded, 32=N61 FRAME-form descriptor bit19 in addition to bit21 when the reference was field-coded, B path, 64=same on the frame P path (default 1 = PAFF_T68 only)");

/*
 * N61 instruments for PAFF_FRMB19/PAFF_FRMP19. h264_b19frm = frame-path
 * reference descriptors whose frame-store was written by a field-coded picture
 * (counted unconditionally, so the predicate's reachability is measurable at
 * built-in defaults); h264_b19hit = descriptors that actually received the
 * extra bit19. Read them at /sys/module/mtk_vcodec_dec/parameters/. Writable so
 * a test can zero them between arms.
 */
/*
 * N61 FIELD POC anchor (AVCMV+0x200/0x204/0x208). bit0 FLDPOC_CUR: on a
 * field-coded picture emit the CURRENT FIELD's own POC instead of the frame's
 * top-field POC. Default OFF. Rationale + the vendor value-compare that
 * established it: see the write site.
 */
#define FLDPOC_CUR	0x1
/* N70: DEFAULT-ON -- the field half of the same one mechanism as h264_paff
 * bit4/bit5 + h264_refpoc + h264_mvpair + h264_mvstore; see the h264_paff
 * comment for the whole-battery evidence.
 */
static int h264_fldpoc = FLDPOC_CUR;
module_param(h264_fldpoc, int, 0644);
MODULE_PARM_DESC(h264_fldpoc,
		 "mt8365 H.264 N61 field POC anchor bitmask: 1=AVCMV+0x200/4/8 = the CURRENT FIELD's own POC on a field-coded picture (vendor-exact 60/60 on cavlc_mot_fld0_full_B; default 0 = the pre-N61 top-field POC)");

/*
 * N63: AVCMV+0x220 bit5 is the ONLY register the vendor writes on a PAFF picture
 * that this driver never writes AT ALL.
 *
 * Established by a decode-ordered, value-compared write-set diff on the FIRST
 * identified, picture-0-proven PAFF vendor trace (CVPA1_TOSHIBA_B, vendor decode
 * BIT-EXACT vs the JVT md5 6c1d64160a6a4d81a3c8176e7ba304c4, 127 125 register
 * events, 138 `W AVCVLD+0x98` == the clip's 138 coded pictures, +0x200 == the
 * spec POC of that coded picture on 138/138). On the first broken frame-B
 * picture (decode 15, POC 10) the vendor and this driver write the SAME 415
 * distinct registers except for exactly one:
 *
 *     W AVCMV +0x220 = 0x00000c20   n=4   (seq 12164 / 12901 / 12917 / 12933)
 *
 * and the same single offset on the bit-exact frame-B control (decode 17, n=5).
 * Corpus shape: 2051 writes across the clip, ALWAYS the constant 0x00000c20,
 * one per picture plus one per reference frame; resolution-invariant (identical
 * value in the 720x480 trace-fld.log); and completely ABSENT from the
 * progressive traces (trace-frm.log, trace-cvwp1.log write it zero times). So
 * the vendor's condition is `frame_mbs_only_flag == 0` -- the exact bucket the
 * residual lives in -- not "MBAFF".
 *
 * The read side shows bit5 is cleared by the HW once per picture (R = 0x00000c00
 * at each picture's first access, 0x00000c20 on every later access within the
 * same picture), so ONE read-modify-write per picture reproduces the vendor's
 * register STATE exactly; the vendor's per-reference repeats are idempotent.
 *
 * The driver already had this RMW (N38) but gated behind `eff_mbaff &&
 * (h264_mbaff & MBAFF_MV220)`, which is FALSE on every PAFF and every pure-field
 * picture -- so it has never fired on the failing bucket. Default OFF; bit0
 * widens the gate to the vendor's own condition. Instruments h264_mv220cand
 * (candidates, counted with the arm OFF) and h264_mv220hit (actual writes).
 */
#define MV220_NOTFMO	0x1
static int h264_mv220;
module_param(h264_mv220, int, 0644);
MODULE_PARM_DESC(h264_mv220,
		 "mt8365 H.264 N63 bitmask: 1=fire the AVCMV+0x220 bit5 RMW on every picture of a !frame_mbs_only stream (the vendor's own condition; default 0 = MBAFF-only per N38)");

static int h264_mv220cand;
module_param(h264_mv220cand, int, 0644);
MODULE_PARM_DESC(h264_mv220cand,
		 "mt8365 H.264 N63 instrument: pictures of a !frame_mbs_only stream (candidates; counted even when the arm is off)");

static int h264_mv220hit;
module_param(h264_mv220hit, int, 0644);
MODULE_PARM_DESC(h264_mv220hit,
		 "mt8365 H.264 N63 instrument: pictures on which the AVCMV+0x220 bit5 RMW actually fired");

/*
 * N72: MC+0x920 -- the LAST config-word divergence left on the failing picture.
 *
 * A pipeline-aligned ABSOLUTE (not matched-pair) value diff of THIS driver's own
 * on-silicon write stream (h264_wrlog) for decode 15 of CVPA1_TOSHIBA_B -- the
 * broken frame-B whose colPic is a field pair -- against the vendor's logical
 * window for the same picture (`window[15][42:] ++ window[16][:42]`,
 * ~/bftrace/trace-paff-n63.log, the identified capture of a run scored BIT-EXACT
 * vs JVT) leaves, after the address-domain registers are set aside, exactly
 * these divergences:
 *
 *   AVCMV +0x220  vendor-only x4 0x00000c20   (h264_mv220, MEASURED N64: fires
 *                                              138/138, the 7 frame-B pictures
 *                                              byte-identical -> inert here)
 *   VLD   +0x1d0  ours-only 0x00000100        (also present on the BIT-EXACT
 *                                              frame-B picture 17)
 *   AVCVLD+0x084 / MC+0x024 / TOP+0x054       (duplicate idempotent writes)
 *   MC    +0x920  vendor 0x01000000           <-- ours 0x01000111
 *
 * Everything else -- the six L1 reference descriptors (0x0328000c / 0x0368000d /
 * 0x01a80006 / 0x01e80007 / 0x00280000 / 0x00680001, slot field included), the
 * POC anchor AVCMV+0x200/4/8 = 0x0a, the AVCVLD ref-list/POC arrays -- is now
 * BYTE-IDENTICAL to the vendor under the N70 default-ON stack.
 *
 * MC+0x920 is a hardcoded constant in this driver and it sits in the MOTION-
 * COMPENSATION block, i.e. exactly on the path "between a correct co-located MV
 * and a pixel" that the remaining defect is bounded to. Provenance is stated
 * honestly: the vendor writes 0x01000000 on 138/138 pictures of the ONE trace
 * that is confirmed bit-exact against the JVT reference for this clip, and
 * 0x01000111 in eleven OLDER captures (including the superseded, unusable
 * trace-paff.log of this same clip). So the value is not uniformly load-bearing
 * -- which is why this is an A/B arm, DEFAULT OFF, and not a fix.
 *
 * bit0 = write the vendor bit-exact-run value 0x01000000 instead of 0x01000111.
 * h264_mc920cand counts UNCONDITIONALLY (every picture that programs the word),
 * so the denominator is observable with the arm OFF; h264_mc920hit counts the
 * writes that actually carried the vendor value.
 */
#define MC920_VENDOR	0x1	/* bit0 */
static int h264_mc920;
module_param(h264_mc920, int, 0644);
MODULE_PARM_DESC(h264_mc920,
		 "mt8365 H.264 N72 A/B: bit0 = program MC+0x920 with the vendor bit-exact-run value 0x01000000 instead of 0x01000111 (default 0 = pre-N72 byte-for-byte)");

static int h264_mc920cand;
module_param(h264_mc920cand, int, 0644);
MODULE_PARM_DESC(h264_mc920cand,
		 "mt8365 H.264 N72 instrument: pictures that program MC+0x920 (counted with the arm OFF -- the observable denominator)");

static int h264_mc920hit;
module_param(h264_mc920hit, int, 0644);
MODULE_PARM_DESC(h264_mc920hit,
		 "mt8365 H.264 N72 instrument: pictures on which MC+0x920 actually carried the vendor value 0x01000000");

/*
 * N65: the FRAME-form reference descriptor's BOTTOM-parity word must carry the
 * reference's OWN BottomFieldOrderCnt, not a repeat of its TopFieldOrderCnt.
 *
 * Derived off-board from the identified PAFF vendor trace (trace-paff-n63.log,
 * CVPA1_TOSHIBA_B, vendor decode bit-exact vs the JVT md5) with NO free
 * parameter.  Every non-zero group-B/group-C descriptor word of every window
 * decodes exactly as
 *
 *   [31:23] slot   = allocation-order index of the referenced frame-store's
 *                    colMV-pool buffer (pool = order of first appearance of the
 *                    +0x180 fill value; 13 buffers on this clip)
 *   [22]    parity = the referenced FIELD's parity (frame form: word 2i = top,
 *                    word 2i+1 = bottom of the same reference frame)
 *   [21]           = the CURRENT picture is frame-coded (FRAME-form descriptor)
 *   [20]           = long-term
 *   [19]           = the referenced frame-store was written by a FIELD-coded pic
 *   [17:0]         = that FIELD's own POC, signed 18-bit
 *
 * Verified 2510/2510 descriptor words over all 138 windows of the PAFF trace and
 * 644/644 over all 60 windows of the identified pure-field trace (trace-fld.log,
 * 720x480, a 7-buffer pool) -- 3154/3154 with no fitted constant.
 *
 * Two consequences:
 *
 *  1. The slot field is a colMV-pool ALLOCATION index, i.e. a frame-store
 *     identity tag, NOT a field-vs-frame unit.  This driver's own slots are the
 *     same kind of index in its own pool (verified a bijection against the
 *     +0x180 addresses we program: 0x0fe88000/0x0fe98000/0x0feb0000/0x0fec4000
 *     <-> 16/12/6/1, no collisions).  The N63 note that our slots are "exactly
 *     2x the vendor's" is an artifact of one window: 12=2*6 and 6=2*3 there, but
 *     the third entry is ours 1 vs vendor 0, and the next picture has ours 16
 *     where a 2x rule needs a vendor 8.  Halving the slot would be wrong; there
 *     is nothing to fix on that half.
 *
 *  2. The POC half IS a real divergence, and only on the FRAME-form path.  The
 *     field-form builder (MT8365_PROG_FGROUP) already emits each field's own
 *     POC.  The frame-form builders emit `w1 = w0 | bit22`, so a reference frame
 *     whose two fields have different POCs (any field-coded reference in a PAFF
 *     stream) gets its TOP POC on both words: e.g. decode picture 15 of
 *     CVPA1_TOSHIBA_B, ours 0x0620000c/0x0660000c vs vendor 0x0328000c/
 *     0x0368000d (POC 12/12 vs 12/13), 0x03200006/0x03600006 vs 0x01a80006/
 *     0x01e80007, 0x00a00000/0x00e00000 vs 0x00280000/0x00680001.  A reference
 *     that really is frame-coded with delta_pic_order_cnt_bottom == 0 has
 *     top == bottom and both sides agree (POC 18 on picture 17).
 *
 * bit0 REFPOC_FRMBOT switches the frame-form bottom word to the reference's
 * bottom_field_order_cnt on both frame-form builders (the B path and the P/SP
 * path).  Default OFF.  h264_refpoccand counts, UNCONDITIONALLY, the frame-form
 * bottom-parity descriptor words whose reference has bottom != top POC (the
 * words the arm would change, so the denominator is observable with the arm
 * off); h264_refpochit counts the words actually changed.
 */
#define REFPOC_FRMBOT	0x1
/* N70: DEFAULT-ON -- see the h264_paff comment (one mechanism, five values). */
static int h264_refpoc = REFPOC_FRMBOT;
module_param(h264_refpoc, int, 0644);
MODULE_PARM_DESC(h264_refpoc,
		 "mt8365 H.264 N65 bitmask: 1=FRAME-form reference descriptor bottom-parity word carries the reference's own bottom_field_order_cnt instead of repeating its top POC (vendor-exact 3154/3154 descriptor words across two identified traces; default 0 = the pre-N65 repeat)");

static int h264_refpoccand;
module_param(h264_refpoccand, int, 0644);
MODULE_PARM_DESC(h264_refpoccand,
		 "mt8365 H.264 N65 instrument: FRAME-form bottom-parity descriptor words whose reference has bottom_field_order_cnt != top_field_order_cnt (candidates; counted even when the arm is off)");

static int h264_refpochit;
module_param(h264_refpochit, int, 0644);
MODULE_PARM_DESC(h264_refpochit,
		 "mt8365 H.264 N65 instrument: FRAME-form bottom-parity descriptor words that actually received the reference's own bottom POC");

/*
 * N66: the colMV-pool REUSE PATTERN.  The descriptor slot ([31:23]) is a
 * per-decoder colMV-pool allocation-order tag (N65, 3154/3154 words modelled
 * with no free parameter), so the vendor's numbers and ours legitimately
 * differ -- both are valid indices in their own pool.  What is NOT the same is
 * the pool's SHAPE: derived off-board from the identified traces, the vendor's
 * pool holds 13 buffers on CVPA1_TOSHIBA_B (7 on the 720x480 field clip), it
 * recycles a buffer every 11-16 frame-stores, and the largest slot it EVER
 * emits in any trace on disk is 12 (5 on trace-fld.log / trace-cvwp1.log, 3 on
 * trace-mbaff-cvma1.log).  Ours is a 32-deep decode-order ring, so it recycles
 * roughly twice as slowly AND emits tags the vendor never emits (slot 16 is
 * already in our CVPA1 descriptor stream).  If the hardware keys an internal
 * colMV cache by this tag, a tag outside the range the vendor ever uses -- or a
 * recycle cadence twice the vendor's -- is the one descriptor-adjacent thing
 * still untested.  h264_mvpool caps the EFFECTIVE pool (allocation is unchanged
 * at MT8365_MV_POOL_NUM); 16 puts our slot range at 0..15 and our recycle
 * cadence inside the vendor's 11-16 frame-store envelope.  Must be a power of
 * two (the descriptor builder masks with pool-1) and >= 2.  Default 0 = OFF =
 * the pre-N66 32-slot ring, byte-for-byte.
 */
static int h264_mvpool;
module_param(h264_mvpool, int, 0644);
MODULE_PARM_DESC(h264_mvpool,
		 "mt8365 H.264 N66: effective colMV-pool size (power of two, 2..32; 0 = default 32 = pre-N66 behaviour). 16 reproduces the vendor's slot range (max 12 observed) and its 11-16 frame-store recycle cadence");

static int h264_mvpoolcand;
module_param(h264_mvpoolcand, int, 0644);
MODULE_PARM_DESC(h264_mvpoolcand,
		 "mt8365 H.264 N66 instrument: reference-descriptor words emitted with a slot >= 16, i.e. a tag the vendor is never observed to write (counted even when the arm is off -- the denominator)");

static int h264_mvpoolhit;
module_param(h264_mvpoolhit, int, 0644);
MODULE_PARM_DESC(h264_mvpoolhit,
		 "mt8365 H.264 N66 instrument: current-picture colMV assignments that landed on a DIFFERENT slot than the uncapped 32-deep ring would have given (execution of the h264_mvpool arm)");

/*
 * N67: the colMV STORE base carries the CURRENT PICTURE's FIELD PARITY in bit1.
 *
 * AVCMV+0x20c is the address the H.264 AVC_MV write port stores this picture's
 * colocated MVs through (N22). The vendor writes it once per coded picture --
 * 138 writes for the 138 coded pictures of trace-paff-n63.log -- and its value
 * is NOT just `colmv_base >> 4`: bit1 is set exactly when the picture being
 * decoded is the BOTTOM FIELD of a field-coded frame-store.
 *
 *   AVCMV+0x20c = (colmv_base >> 4) | ((field_pic_flag && bottom_field_flag) ? 2 : 0)
 *
 * Scored against the coding form decoded from the same trace's AVCVLD+0x98
 * config word (bit17 = field_pic_flag, bit18 = bottom_field_flag, the same
 * encoding this driver writes at ~line 3512), with no free parameter:
 *
 *   trace-paff-n63.log  (CVPA1_TOSHIBA_B, PAFF)   138/138  mismatch 0
 *   trace-fld.log       (cavlc_mot_fld0_full_B)     60/60   mismatch 0
 *   trace-paff.log                                   8/8    mismatch 0
 *   trace-mbaff-cvma1.log                            5/5    mismatch 0
 *   trace-cvwp1.log                                  7/7    mismatch 0
 *                                            total 218/218  mismatch 0
 *
 * On the PAFF trace the pairing is visible directly: a complementary field pair
 * shares ONE colMV buffer and differs only in this bit (pic 0 `0x0f9ec800` /
 * pic 1 `0x0f9ec802`; pic 5 `0x0f9fa800` / pic 6 `0x0f9fa802`), while every
 * frame-coded picture writes bit1 = 0 (pic 4 `0x0fa02800`, pic 15 `0x0f9f6800`).
 *
 * We have never emitted it: the write below was `cur_mv` unconditionally, so
 * BOTH fields of a field-coded frame-store store their colMVs through the
 * top-field surface. The READ side already carries the same flag -- the
 * frame-form ref array writes `+0x180+8s = rmv` and `+0x184+8s = rmv | 2`, and
 * the field-form paths write `rmv | (ref_bot ? 2 : 0)` -- so on a field-coded
 * reference we read a bottom-field colMV surface that was never written.
 *
 * That asymmetry is invisible to a whole-window register diff of the BROKEN
 * picture (N66): decode 15 and decode 17 are both FRAME pictures, and a frame
 * picture writes bit1 = 0 on both sides. The divergence is in the window of the
 * field-coded colPic (decode 14), whose PIXELS are bit-exact -- which is why
 * reference-error propagation scored TP=0 FP=0 FN=7 TN=17 while the defect is
 * still inherited from that reference: what is wrong is its colMV BUFFER, not
 * its reconstruction.
 *
 * Default OFF. Inert by construction on progressive streams (no field-coded
 * picture exists) and on every top field.
 */
#define MVSTORE_FLDPARITY	0x1

/* N70: DEFAULT-ON -- see the h264_paff comment (one mechanism, five values). */
static int h264_mvstore = MVSTORE_FLDPARITY;
module_param(h264_mvstore, int, 0644);
MODULE_PARM_DESC(h264_mvstore,
		 "mt8365 H.264 N67 (bitmask, default 0=OFF): 1 = emit the vendor's FIELD PARITY bit (bit1) in the colMV STORE base AVCMV+0x20c on a BOTTOM field, so the two fields of a field-coded frame-store no longer store their colocated MVs through the same surface (vendor model 218/218 across five traces)");

static int h264_mvstorecand;
module_param(h264_mvstorecand, int, 0644);
MODULE_PARM_DESC(h264_mvstorecand,
		 "mt8365 H.264 N67 instrument: coded pictures that are the BOTTOM FIELD of a field-coded frame-store, counted UNCONDITIONALLY (the denominator, readable with the arm off)");

static int h264_mvstorehit;
module_param(h264_mvstorehit, int, 0644);
MODULE_PARM_DESC(h264_mvstorehit,
		 "mt8365 H.264 N67 instrument: colMV STORE bases actually written with bit1 set (execution of the h264_mvstore arm)");

/*
 * N68: colMV BUFFER byte dump. Six mechanism-matched register arms have now
 * fired on exactly the right pictures and flipped nothing, and the AVCMV
 * descriptor word + store base are fully modelled -- the register stream is
 * exhausted. The one surface nobody has read is what the hardware actually
 * WROTE into the colMV buffer: our pool buffers are CPU-mapped
 * (mtk_vcodec_mem_alloc -> dma_alloc_coherent), so a memcpy after the dec-done
 * IRQ captures the exact bytes the AVC_MV write port stored, with no new
 * allocation and no extra DMA.
 *
 * The question this answers, which no register can: a complementary field pair
 * shares ONE frame-store and therefore ONE colMV buffer. Where does the SECOND
 * field's data live inside it -- a second region at an offset, interleaved by
 * MB row, or nowhere because both fields stored through the same surface and
 * the bottom field's MVs overwrote the top field's? Snapshotting the SAME
 * buffer after the top field's decode and again after the bottom field's
 * decode answers it directly: if the surfaces are separate, the region the top
 * field wrote survives the bottom field's decode byte-for-byte.
 *
 * h264_mvdump   = number of consecutive decoded pictures to snapshot (0 = off)
 * h264_mvdump_from = first decode index of the snapshot window
 * h264_mvdump_taken = instrument, snapshots actually captured
 * h264_mvdump_flush = write a snapshot index to emit it to the kernel log as
 *                     'H264MVD <off> <64 hex bytes>' -- one line per 64-byte
 *                     HW_MB_STORE_SZ macroblock unit. The emission is a
 *                     separate, post-decode event on purpose: the capture path
 *                     is a memcpy, so it cannot perturb the decode it
 *                     instruments the way h264_wrlog's in-line printk does.
 */
#define MT8365_MVDUMP_SNAPS	8
#define MT8365_MVDUMP_CAP	(128 * 1024)
/*
 * N83: the one snapshot slot that is taken on the READ side (see
 * mt8365_mvdump_snap_read below). Kept above the write-side window (which the
 * scripts drive as from=11 count=6 -> slots 0..5) so a single run can hold both
 * the write-time and the read-time view of the same buffer.
 */
#define MT8365_MVDUMP_READ_SNAP	6

static int h264_mvdump;
module_param(h264_mvdump, int, 0644);
MODULE_PARM_DESC(h264_mvdump,
		 "mt8365 H.264 N68: snapshot the colMV BUFFER bytes of this many consecutive decoded pictures, starting at h264_mvdump_from (0 = off)");

static int h264_mvdump_from;
module_param(h264_mvdump_from, int, 0644);
MODULE_PARM_DESC(h264_mvdump_from,
		 "mt8365 H.264 N68: first decode index of the h264_mvdump window (default 0)");

static int h264_mvdump_taken;
module_param(h264_mvdump_taken, int, 0444);
MODULE_PARM_DESC(h264_mvdump_taken,
		 "mt8365 H.264 N68 instrument: colMV buffer snapshots actually captured");

/*
 * Bytes of a snapshot the flush emits as hex. The buffer is 256 KiB
 * (BUF_PREDICTION_SZ floor) while the colMV content of a 352x288 frame is
 * 25 856 B, so the default window covers the content with room to spare and
 * keeps a flush small enough that journald's 10 000-messages/30 s rate limit
 * cannot silently drop lines (a drop is what makes a byte dump lie). The
 * NONZERO MAP printed with every flush covers the WHOLE captured region, so
 * anything living outside the hex window is still visible.
 */
static int h264_mvdump_len = 32768;
module_param(h264_mvdump_len, int, 0644);
MODULE_PARM_DESC(h264_mvdump_len,
		 "mt8365 H.264 N68: bytes of a snapshot emitted as hex by h264_mvdump_flush (default 32768)");

static int h264_mvdump_off;
module_param(h264_mvdump_off, int, 0644);
MODULE_PARM_DESC(h264_mvdump_off,
		 "mt8365 H.264 N68: byte offset the h264_mvdump_flush hex window starts at (default 0)");

/*
 * N83 -- the colMV buffer AS THE READING PICTURE SEES IT.
 *
 * Every colMV dump this programme has taken (N68, N70) was captured on the
 * picture that WROTE the buffer, at dec-done. Nothing has ever inspected the
 * buffer at the moment a LATER picture consumes it as colPic. If anything
 * disturbs it in between -- ring recycling, a stale mv_pool_key stamp, the
 * frame-store being freed/reallocated, or the reading picture resolving to the
 * WRONG slot entirely (mt8365_mv_lookup() falls back to slot 0 on a miss,
 * SILENTLY) -- no write-side dump could ever have shown it.
 *
 * h264_mvread      = decode index of the READING picture (0 = off). At that
 *                    picture's ref-list programming -- i.e. BEFORE the trigger,
 *                    with the buffer in exactly the state the HW is about to
 *                    fetch -- snapshot the colPic (RefPicList1[0]) slot.
 * h264_mvread_ref  = which write-side snapshot slot to byte-compare against
 *                    (default 1 = the second picture of the h264_mvdump window;
 *                    with from=11 that is decode 12, the bottom field that
 *                    completes the P field pair).
 * h264_mvread_cand = candidates, counted UNCONDITIONALLY so the denominator is
 *                    observable with the arm OFF.
 * h264_mvread_taken= snapshots actually captured (also the once-per-run guard;
 *                    writable so a script can re-arm it).
 *
 * Like the write-side instrument the capture is a memcpy from a coherent
 * mapping and emits NOTHING in line: the findings are formatted into a static
 * line buffer and printed only when the snapshot is flushed, so the decode it
 * instruments cannot be perturbed by console I/O.
 */
static int h264_mvread;
module_param(h264_mvread, int, 0644);
MODULE_PARM_DESC(h264_mvread,
		 "mt8365 H.264 N83: decode index of the READING picture whose colPic colMV buffer is snapshotted at ref-list-programming time (0 = off)");

static int h264_mvread_ref = 1;
module_param(h264_mvread_ref, int, 0644);
MODULE_PARM_DESC(h264_mvread_ref,
		 "mt8365 H.264 N83: write-side snapshot index the read-time capture is byte-compared against (default 1)");

static int h264_mvread_cand;
module_param(h264_mvread_cand, int, 0644);
MODULE_PARM_DESC(h264_mvread_cand,
		 "mt8365 H.264 N83 instrument: colPic read-side resolutions seen (counted unconditionally)");

static int h264_mvread_taken;
module_param(h264_mvread_taken, int, 0644);
MODULE_PARM_DESC(h264_mvread_taken,
		 "mt8365 H.264 N83 instrument: read-time colMV snapshots captured");

#define MT8365_MVREAD_LINES	48
#define MT8365_MVREAD_COLS	160
static char mvread_note[MT8365_MVREAD_LINES][MT8365_MVREAD_COLS];
static int mvread_nlines;

static int mvdump_idx;			/* decode index, ALWAYS advances */
static void *mvdump_buf[MT8365_MVDUMP_SNAPS];
static u32 mvdump_len[MT8365_MVDUMP_SNAPS];
static u32 mvdump_meta[MT8365_MVDUMP_SNAPS][7];

static void mt8365_mvdump_snap(struct vdec_h264_mt8365_inst *inst, int idx,
			       int slot, int fld, int bot, u32 stype)
{
	u32 len;
	int s;

	if (!h264_mvdump || idx < h264_mvdump_from ||
	    idx >= h264_mvdump_from + h264_mvdump)
		return;
	s = idx - h264_mvdump_from;
	if (s < 0 || s >= MT8365_MVDUMP_SNAPS)
		return;
	if (slot < 0 || slot >= MT8365_MV_POOL_NUM || !inst->mv_pool[slot].va)
		return;

	len = min_t(u32, (u32)inst->mv_pool[slot].size, MT8365_MVDUMP_CAP);
	if (!mvdump_buf[s]) {
		mvdump_buf[s] = kvmalloc(MT8365_MVDUMP_CAP, GFP_KERNEL);
		if (!mvdump_buf[s])
			return;
	}
	memcpy(mvdump_buf[s], inst->mv_pool[slot].va, len);
	mvdump_len[s] = len;
	mvdump_meta[s][0] = idx;
	mvdump_meta[s][1] = slot;
	mvdump_meta[s][2] = fld;
	mvdump_meta[s][3] = bot;
	mvdump_meta[s][4] = (u32)(inst->mv_pool[slot].dma_addr >> 4);
	mvdump_meta[s][5] = stype;
	mvdump_meta[s][6] = (u32)inst->mv_pool[slot].size;
	h264_mvdump_taken++;
}

#define MVREAD_LOG(fmt, ...)						\
	do {								\
		if (mvread_nlines < MT8365_MVREAD_LINES)		\
			scnprintf(mvread_note[mvread_nlines++],		\
				  MT8365_MVREAD_COLS, fmt, ##__VA_ARGS__); \
	} while (0)

/*
 * N83: capture the colPic's colMV buffer at the instant the READING picture
 * resolves it, and record the bookkeeping that says whether the resolution is
 * even the right one.
 *
 * Three questions, each answered from data rather than inference:
 *  (a) CONTENT   -- byte-compare the buffer now against the write-time snapshot
 *                   h264_mvread_ref (kernel-side memcmp over the whole captured
 *                   region, so the answer cannot be lost to a dropped log line;
 *                   the hex dump of both snapshots is available for independent
 *                   off-board verification).
 *  (b) SLOT      -- mt8365_mv_lookup() returns 0 on a MISS, so the slot number
 *                   alone is not evidence. Re-scan mv_pool_key[] here and record
 *                   whether the colPic's recon-Y is actually present, and where.
 *  (c) OWNERSHIP -- dump the whole pool (key, field flag, dma_addr, size, va) so
 *                   a ring recycle to a later picture is VISIBLE, not deduced.
 */
static void mt8365_mvdump_snap_read(struct vdec_h264_mt8365_inst *inst,
				    int idx, u32 ry, int sl)
{
	const int s = MT8365_MVDUMP_READ_SNAP;
	int r = h264_mvread_ref;
	int first = -1, found = -1, j;
	u32 len = 0, cmplen, ndiff = 0, i;
	const u8 *a, *b;

	h264_mvread_cand++;
	if (!h264_mvread || idx != h264_mvread || h264_mvread_taken)
		return;

	/* (b) independent slot resolution */
	for (j = 0; j < MT8365_MV_POOL_NUM; j++)
		if (inst->mv_pool_key[j] == ry) {
			found = j;
			break;
		}
	MVREAD_LOG("H264MVR pic=%d colpic_ry=0x%08x lookup_slot=%d key_found_slot=%d resolved_by_key=%d rd_fld=%u",
		   idx, ry, sl, found, found == sl ? 1 : 0,
		   inst->mv_pool_fld[sl & (MT8365_MV_POOL_NUM - 1)]);

	/* (c) + the capture itself */
	if (sl >= 0 && sl < MT8365_MV_POOL_NUM && inst->mv_pool[sl].va) {
		len = min_t(u32, (u32)inst->mv_pool[sl].size,
			    MT8365_MVDUMP_CAP);
		if (!mvdump_buf[s])
			mvdump_buf[s] = kvmalloc(MT8365_MVDUMP_CAP, GFP_KERNEL);
		if (mvdump_buf[s]) {
			memcpy(mvdump_buf[s], inst->mv_pool[sl].va, len);
			mvdump_len[s] = len;
			mvdump_meta[s][0] = idx;
			mvdump_meta[s][1] = sl;
			mvdump_meta[s][2] = 0;
			mvdump_meta[s][3] = 0;
			mvdump_meta[s][4] =
				(u32)(inst->mv_pool[sl].dma_addr >> 4);
			mvdump_meta[s][5] = 0xead;
			mvdump_meta[s][6] = (u32)inst->mv_pool[sl].size;
		}
	}
	MVREAD_LOG("H264MVR rd_slot=%d va=%d dma>>4=0x%08x size=%u captured=%u",
		   sl,
		   (sl >= 0 && sl < MT8365_MV_POOL_NUM && inst->mv_pool[sl].va)
			   ? 1 : 0,
		   (sl >= 0 && sl < MT8365_MV_POOL_NUM)
			   ? (u32)(inst->mv_pool[sl].dma_addr >> 4) : 0,
		   (sl >= 0 && sl < MT8365_MV_POOL_NUM)
			   ? (u32)inst->mv_pool[sl].size : 0,
		   len);

	for (j = 0; j < MT8365_MV_POOL_NUM; j++)
		if (inst->mv_pool_key[j] || inst->mv_pool[j].va)
			MVREAD_LOG("H264MVR pool[%02d] key=0x%08x fld=%u dma>>4=0x%08x size=%u va=%d",
				   j, inst->mv_pool_key[j],
				   inst->mv_pool_fld[j],
				   (u32)(inst->mv_pool[j].dma_addr >> 4),
				   (u32)inst->mv_pool[j].size,
				   inst->mv_pool[j].va ? 1 : 0);

	/* (a) content, write time vs read time */
	if (r >= 0 && r < MT8365_MVDUMP_SNAPS && mvdump_buf[r] &&
	    mvdump_len[r] && mvdump_buf[s] && mvdump_len[s]) {
		cmplen = min_t(u32, mvdump_len[r], mvdump_len[s]);
		a = mvdump_buf[r];
		b = mvdump_buf[s];
		for (i = 0; i < cmplen; i++)
			if (a[i] != b[i]) {
				ndiff++;
				if (first < 0)
					first = (int)i;
			}
		MVREAD_LOG("H264MVR CMP wr_snap=%d wr_pic=%u wr_slot=%u wr_base>>4=0x%08x rd_base>>4=0x%08x cmplen=%u ndiff=%u first_diff=%d",
			   r, mvdump_meta[r][0], mvdump_meta[r][1],
			   mvdump_meta[r][4], mvdump_meta[s][4], cmplen, ndiff,
			   first);
	} else {
		MVREAD_LOG("H264MVR CMP UNAVAILABLE wr_snap=%d wr_len=%u rd_len=%u",
			   r,
			   (r >= 0 && r < MT8365_MVDUMP_SNAPS) ? mvdump_len[r]
							       : 0,
			   mvdump_len[s]);
	}
	h264_mvread_taken++;
}

static int mt8365_mvdump_flush_set(const char *val,
				   const struct kernel_param *kp)
{
	u32 off, first = U32_MAX, last = 0, i, blk;
	char map[MT8365_MVDUMP_CAP / 1024 + 1];
	const u8 *p;
	int s, rc;

	rc = kstrtoint(val, 0, &s);
	if (rc)
		return rc;
	if (s < 0 || s >= MT8365_MVDUMP_SNAPS || !mvdump_buf[s] ||
	    !mvdump_len[s])
		return -EINVAL;

	p = mvdump_buf[s];
	/*
	 * N83: the read-side snapshot carries its bookkeeping findings with it.
	 * Emitted here, post-decode, for the same reason the byte dump is.
	 */
	if (s == MT8365_MVDUMP_READ_SNAP) {
		int k;

		for (k = 0; k < mvread_nlines; k++)
			pr_info("%s\n", mvread_note[k]);
		pr_info("H264MVR notes=%d\n", mvread_nlines);
	}
	pr_info("H264MVD ==== snap %d pic=%u slot=%u fld=%u bot=%u styp=%u base>>4=0x%08x size=%u captured=%u\n",
		s, mvdump_meta[s][0], mvdump_meta[s][1], mvdump_meta[s][2],
		mvdump_meta[s][3], mvdump_meta[s][5], mvdump_meta[s][4],
		mvdump_meta[s][6], mvdump_len[s]);

	/*
	 * NONZERO MAP over the WHOLE captured region, one character per KiB.
	 * This is what says where the data actually lives -- and, because it
	 * costs two lines, it is never the thing a rate limiter drops.
	 */
	for (blk = 0; blk * 1024 < mvdump_len[s]; blk++) {
		bool nz = false;

		for (i = blk * 1024; i < (blk + 1) * 1024 && i < mvdump_len[s];
		     i++)
			if (p[i]) {
				nz = true;
				if (first == U32_MAX)
					first = i;
				last = i;
			}
		map[blk] = nz ? '1' : '.';
	}
	map[blk] = '\0';
	pr_info("H264MVD MAP  snap %d 1KiB-blocks %s\n", s, map);
	pr_info("H264MVD SPAN snap %d first_nonzero=%d last_nonzero=%d\n",
		s, first == U32_MAX ? -1 : (int)first,
		first == U32_MAX ? -1 : (int)last);

	/* 256 bytes per line: 58 lines cover a 352x288 colMV content region. */
	for (off = clamp_t(u32, h264_mvdump_off, 0, mvdump_len[s]);
	     off + 256 <= mvdump_len[s] &&
	     off < (u32)h264_mvdump_off + (u32)h264_mvdump_len;
	     off += 256)
		pr_info("H264MVD %05x %*phN%*phN%*phN%*phN\n", off,
			64, p + off, 64, p + off + 64,
			64, p + off + 128, 64, p + off + 192);
	pr_info("H264MVD ==== snap %d end\n", s);
	return 0;
}

static const struct kernel_param_ops mt8365_mvdump_flush_ops = {
	.set = mt8365_mvdump_flush_set,
};
module_param_cb(h264_mvdump_flush, &mt8365_mvdump_flush_ops, NULL, 0200);
MODULE_PARM_DESC(h264_mvdump_flush,
		 "mt8365 H.264 N68: write a snapshot index (0..5) to emit that colMV buffer to the kernel log, 64 bytes (one HW_MB_STORE_SZ MB unit) per line");

/*
 * N68 arm: a complementary FIELD PAIR must share ONE colMV buffer.
 *
 * Derived from the first colMV buffer dump. mt8365_mv_assign() consumes a
 * decode-order ring slot for every DECODED PICTURE, so the top field and the
 * bottom field of one frame-store are handed two DIFFERENT pool slots and two
 * DIFFERENT buffers -- and the assign then CLEARS the older slot's key
 * ("i != slot && mv_pool_key[i] == y"), so the frame-store resolves to the
 * BOTTOM field's buffer only and the TOP field's stored colMVs become
 * unreachable: nothing ever reads them.
 *
 * The vendor does the opposite. A complementary field pair shares ONE colMV
 * base and the two members differ only in bit1 of AVCMV+0x20c (0x0f9ec800 /
 * 0x0f9ec802, N67, 218/218 across five traces) -- one buffer per FRAME-STORE,
 * two field surfaces inside it, selected by the parity bit.
 *
 * MVPAIR_SHARE makes the second field of a frame-store reuse the slot its
 * complementary field already owns instead of consuming a new ring slot, which
 * is the vendor's one-slot-per-frame-store arrangement. It is the write-side
 * half of the pair; h264_mvstore bit0 (N67) is the other half, and the two are
 * plausibly co-dependent -- sharing a buffer without the parity bit would have
 * the bottom field overwrite the top field's surface, which is precisely the
 * arrangement N67 measured as inert on a pure-field clip.
 *
 * Default OFF. Inert by construction on progressive streams (no field-coded
 * picture) and on the first field of every pair.
 */
#define MVPAIR_SHARE	0x1
#define MVPAIR_ADJACENT	0x2

/* N70: DEFAULT-ON (strict form, bit0|bit1) -- see the h264_paff comment. */
static int h264_mvpair = MVPAIR_SHARE | MVPAIR_ADJACENT;
module_param(h264_mvpair, int, 0644);
MODULE_PARM_DESC(h264_mvpair,
		 "mt8365 H.264 N68 (bitmask, default 0=OFF): 1 = a FIELD picture whose complementary field already owns a colMV pool slot REUSES that slot, so a complementary field pair shares ONE colMV buffer (the vendor arrangement) instead of the second field orphaning the first field's buffer; 2 = restrict that reuse to a TRUE complementary pair (the immediately preceding decoded picture), excluding a stale key left on a recycled recon buffer -- use 3 for the strict form");

static int h264_mvpaircand;
module_param(h264_mvpaircand, int, 0644);
MODULE_PARM_DESC(h264_mvpaircand,
		 "mt8365 H.264 N68 instrument: FIELD pictures whose frame-store already owns a colMV pool slot, counted UNCONDITIONALLY (the denominator, readable with the arm off)");

static int h264_mvpairadj;
module_param(h264_mvpairadj, int, 0644);
MODULE_PARM_DESC(h264_mvpairadj,
		 "mt8365 H.264 N68 instrument: of the h264_mvpaircand matches, how many are a TRUE complementary pair (the previous decoded picture was field-coded, same recon-Y, owns that slot) rather than a stale key on a recycled recon buffer");

static int h264_mvpairhit;
module_param(h264_mvpairhit, int, 0644);
MODULE_PARM_DESC(h264_mvpairhit,
		 "mt8365 H.264 N68 instrument: field pictures that actually reused their complementary field's colMV slot (execution of the h264_mvpair arm)");

/*
 * N120 -- the colMV parity addend K, and a POISON probe for the records it
 * makes the two parities share.
 *
 * N117 decoded the vendor's parity addend as a single chip-flag quantity
 *   K = ctx[+2516] ? 2 : 4                (mov w22,#4 @0x4c218, #2 @0x4c9b4)
 * used by BOTH the store (AVCMV+0x20c) and the read (AVCMV+0x180) side, in
 * units of 16 bytes. This driver hard-codes K = 2 (the value the vendor is
 * measured to program on MT8365, 218/218 stores across five traces).
 *
 * With K = 2 a 128-byte per-field-MB colMV unit is laid out
 *      top parity    -> bytes [ 0.. 63]  (4 x 16B records: TL TR BL BR)
 *      bottom parity -> bytes [32.. 95]
 *      [96..127]     -> never written
 * so the bottom field's records 0-1 land on the top field's records 2-3.
 * N117 measured that overwrite (top rec2/rec3 score 12/12 -> 0/12 against
 * ffmpeg once the bottom field decodes) and argued from vendor identity that
 * the hardware must therefore not CONSULT a top-parity colPic's records 2-3.
 * N119 then measured, at 8x8-quadrant granularity, that 740 of 740 root-wrong
 * direct-mode top-field macroblocks have a bit-exact TOP half with all damage
 * in BL+BR -- which are exactly records 2 and 3.
 *
 * N120 settled it by POISON rather than by inference, and N117's reading is
 * REFUTED. Overwriting a selected 16-byte record range of the co-located
 * picture's colMV buffer between the store and the read, on
 * cavlc_mot_fld0_full_B:
 *   bytes [ 0.. 31] (top rec0,1)              -> TOP output changes, BOTTOM does not
 *   bytes [32.. 63] (top rec2,3 = bot rec0,1) -> BOTH change
 *   bytes [64.. 95] (bot rec2,3)              -> BOTTOM changes, TOP does not
 *   bytes [96..127] (never written)           -> output BYTE-IDENTICAL
 * so the TOP-parity read window is exactly [0..63] -- records 2 and 3 included
 * -- and the last line is the negative control that makes the other three mean
 * something. The engine DOES consult the records the bottom field's store
 * destroys, and that overwrite is the mechanism.
 *
 * Programming K = 4 on the store AND the field read side then makes all four
 * failing 720x480 field vectors BIT-EXACT against ffmpeg
 * (cavlc_mot_fld0_full_B, Sharp_MP_Field_3_B, CVFI2_SVA_C, CVNLFI2_Sony_H),
 * with the four non-regression controls byte-identical and the arm proven to
 * execute on them. Either side alone produces a THIRD, still-wrong output --
 * the two windows have to agree -- which is why the default is 3 and not 1
 * or 2.
 *
 * Why the vendor gets away with K = 2 and we cannot: K is an addend, not a
 * footprint. Our per-parity colMV record set is 4 x 16 B = 64 B (measured by
 * the poison map above), so a 32-byte separation necessarily overlaps; the
 * vendor's K = 2 can only be self-consistent if its per-parity footprint is
 * 32 B. The buffer is the driver's own allocation, so the non-overlapping
 * layout is ours to choose even though the vendor chose differently on theirs.
 *
 * The two arms:
 *
 * h264_mvpoison (bitmask over the eight 16-byte records of the 128-byte unit)
 *   overwrites the selected records of the CO-LOCATED picture's colMV buffer
 *   with h264_mvpoison_val, at the reading picture's ref-list programming --
 *   i.e. after every store into that buffer and before the DECODE GO. Poison
 *   records 2|3 (mask 0x0c) and watch the B fields:
 *     - the BOTTOM B field reads that byte range as ITS records 0-1 (TL/TR),
 *       so it MUST change: that is the built-in positive control proving the
 *       poison landed in the buffer the hardware fetches;
 *     - the TOP B field changes ONLY if the engine consults records 2-3 of a
 *       top-parity colPic. If it does not, its output is byte-identical.
 *
 * h264_mvk (bitmask) programs K = 4 instead of K = 2, which separates the two
 *   parities by 64 bytes -- top [0..63], bottom [64..127], the 128-byte unit
 *   exactly filled, no overlap, and the [96..127] region N117 measured as
 *   never written becomes the bottom field's BL/BR. K drives store AND read,
 *   so the bits are separable to tell which side (if either) matters.
 */
#define MVK_STORE	1	/* AVCMV+0x20c bottom-field addend */
#define MVK_FLDREAD	2	/* AVCMV+0x180 parity addend, FIELD ref lists */
#define MVK_FRMREAD	4	/* AVCMV+0x184 bottom-of-frame addend */

static int h264_mvk = MVK_STORE | MVK_FLDREAD;
module_param(h264_mvk, int, 0644);
MODULE_PARM_DESC(h264_mvk,
		 "mt8365 H.264 N120 (bitmask, DEFAULT 3 = the field store+read sides at K=4): program the colMV parity addend K=4 (64-byte parity separation, no record overlap) instead of the vendor's K=2. 1 = STORE side (AVCMV+0x20c), 2 = FIELD READ side (AVCMV+0x180 field ref lists), 4 = FRAME READ side (AVCMV+0x184). 0 = the pre-N120 behaviour (K=2 everywhere), which overlaps the two parities and destroys the top field's records 2-3");

static int h264_mvkhit;
module_param(h264_mvkhit, int, 0644);
MODULE_PARM_DESC(h264_mvkhit,
		 "mt8365 H.264 N120 instrument: parity addends actually emitted as 4 rather than 2 (execution of the h264_mvk arm)");

/*
 * N125 MBAFFREF -- bit 18 of the AVCMV reference DESCRIPTOR is "this reference
 * picture was coded MBAFF", and this driver has never emitted it.
 *
 * Transcribed from libvpud_vcodec.so (mt8365 aarch64, sha256 da6979b2...):
 * Vdec_Drv_setDecPicInfo builds the per-parity descriptor addend it stores at
 * picdesc+132 (top) / picdesc+136 (bottom) as
 *
 *     w4 = pic_struct                       ; 1 top, 2 bottom, 3 frame
 *     w0 = (w4 != 3) << 19                  ; 0x3f080-0x3f084
 *     if (w4 == 3)                          ; 0x3f088 -> 0x3fc80
 *             w0 += sps[1357] ? 0x40000 : 0 ; 0x3fc84-0x3fc90
 *     picdesc[132] = w0                     ; 0x3fc94   (bottom twin: 0x3fcb0)
 *
 * and sps+1357 is mb_adaptive_frame_field_flag (parseSPS 0x50a40-0x50a60:
 * [1356] = frame_mbs_only_flag; if it is 0 the next bit goes to [1357], and
 * only then is direct_8x8_inference_flag read into [1358]).  The same word is
 * later consumed by H264_HAL_SetBRefPicListReg as the addend x21[+36]/[+40]
 * (frame form) / x21[+40] (field form) that rides on top of the 18-bit
 * descriptor field written to AVCMV+0x000/+0x080/+0x100 -- picdesc is
 * slot_base+2480, verified through the colMV pointer (slot+2592 == picdesc+112
 * is the buffer H264_HAL_DecStart reads at 0x4c4d8).
 *
 * So descriptor bits [19:18] are a two-bit reference CODING-MODE field:
 *   00 = progressive frame reference
 *   01 = MBAFF frame reference          <- never emitted by this driver
 *   10 = field reference                 (we do emit this, 0x00080000)
 *
 * N124 measured that on an MBAFF picture the engine consults only records 0-3
 * (bytes [0..63]) of each 128-byte colMV unit while a progressive picture
 * consults all eight, i.e. the colocated motion of every second FRAME
 * macroblock is stored and never read.  This is the only MBAFF-conditioned
 * quantity in the vendor's whole colMV store/read programming, and it is
 * absent from ours -- so it is the arm.
 *
 * Default OFF: nothing is claimed until it is measured.
 */
/*
 * N128 -- the FIELD-form half of the same descriptor field.
 *
 * N125's MBAFFREF_DESC above rides the FRAME-form reference descriptor and is
 * gated `!fld_list`, i.e. it fires only when the CURRENT picture is a frame.
 * N127 localised the surviving field-B defect to exactly the complementary
 * cell: a FIELD-coded B picture whose colocated picture is a FRAME (MBAFF)
 * picture. There the driver walks the FIELD-form builders instead
 * (MT8365_PROG_FGROUP for the B field lists, the `fld_list ? ...` ternary for
 * the P field list), and both of them emit descriptor bits [19:18] = 00
 * ("progressive frame reference") for a frame-coded reference -- the vendor's
 * own encoding says an MBAFF frame reference is 01.
 *
 * The vendor does NOT recompute the addend per consuming picture: it is stored
 * per REFERENCE at that picture's own decode (Vdec_Drv_setDecPicInfo,
 * picdesc+132/+136) and H264_HAL_SetBRefPicListReg consumes the SAME word in
 * the field form (x21[+40]) as in the frame form (x21[+36]/[+40]). So a frame
 * reference of an mb_adaptive_frame_field_flag sequence carries bit18 no matter
 * which form the current picture is -- which is precisely what this driver
 * never emitted on the field side.
 *
 * `mb_adaptive_frame_field_flag` is a SEQUENCE parameter, so "this frame
 * reference was coded MBAFF" and "this SPS says MBAFF" are the same predicate
 * for every frame reference (N125 §2.2). `!inst->mv_pool_fld[slot]` is what
 * makes it a frame reference: a frame-store written by two FIELD pictures is
 * not MBAFF.
 *
 * Bits 2 and 4 are separate so the B field list can be armed without the P
 * field list -- the defect surface is the B one.
 */
#define MBAFFREF_DESC	1	/* AVCMV descriptor bit18 on a frame reference */
#define MBAFFREF_BFLD	2	/* N128: same bit on the B FIELD list (MT8365_PROG_FGROUP) */
#define MBAFFREF_PFLD	4	/* N128: same bit on the P FIELD list */

/*
 * N128 A/B, default-ON justification (same kernel, same boot, one decode at a
 * time, STOCK GStreamer, scored against a board-local ffmpeg -threads 1
 * reference per file):
 *
 *   CVMAPAQP3_Sony_E    9 988 wrong luma samples -> 0     BIT-EXACT
 *   FRExt2_Panasonic_C  145 260 -> 33 294, 4 bad frames -> 1
 *   FRExt4_Panasonic_B  2 245 -> 2 131, 2 bad frames -> 1
 *   CAPAMA3_Sand_F      296 105 -> 167 069, 10 bad frames -> 4
 *
 * Every frame the arm changes it repairs COMPLETELY; the frames left over are
 * the MBAFF frame pictures of the separate, closed MBAFF-AND-B defect (N126),
 * which this arm neither helps nor harms.  No vector anywhere got worse.
 *
 * MBAFFREF_BFLD is therefore DEFAULT-ON.  MBAFFREF_DESC (N125's frame-form
 * half) stays off -- measured inert on 5 vectors at N125 -- and MBAFFREF_PFLD
 * stays off: armed together with BFLD it produced byte-identical output on all
 * three primary vectors, i.e. it is inert, and an inert bit does not ship on.
 */
static int h264_mbaffref = MBAFFREF_BFLD;
module_param(h264_mbaffref, int, 0644);
MODULE_PARM_DESC(h264_mbaffref,
		 "mt8365 H.264 N125/N128 (bitmask, DEFAULT 2 = MBAFFREF_BFLD ON): 1 = emit the vendor's AVCMV reference-descriptor bit18 (0x00040000, \"this reference was coded MBAFF\") on every FRAME-form reference of an mb_adaptive_frame_field_flag sequence, as libvpud_vcodec.so Vdec_Drv_setDecPicInfo@0x3fc84 does; 2 = N128, the same bit on the B FIELD ref lists where the reference is a frame-store of such a sequence (the N127 surface); 4 = N128, the same on the P FIELD ref list");

static int h264_mbaffrefcand;
module_param(h264_mbaffrefcand, int, 0644);
MODULE_PARM_DESC(h264_mbaffrefcand,
		 "mt8365 H.264 N125 instrument: frame-form reference slots reached on an MBAFF sequence (arm-OFF denominator)");

static int h264_mbaffrefhit;
module_param(h264_mbaffrefhit, int, 0644);
MODULE_PARM_DESC(h264_mbaffrefhit,
		 "mt8365 H.264 N125 instrument: reference descriptors that actually carried bit18 (execution of the h264_mbaffref arm)");

static int h264_mbaffreffcand;
module_param(h264_mbaffreffcand, int, 0644);
MODULE_PARM_DESC(h264_mbaffreffcand,
		 "mt8365 H.264 N128 instrument: FIELD-form reference-list entries whose reference is a FRAME frame-store of an MBAFF sequence (arm-OFF denominator, B and P field lists together)");

static int h264_mbaffreffhit;
module_param(h264_mbaffreffhit, int, 0644);
MODULE_PARM_DESC(h264_mbaffreffhit,
		 "mt8365 H.264 N128 instrument: FIELD-form reference descriptors that actually carried bit18 (execution of the MBAFFREF_BFLD / MBAFFREF_PFLD arms)");

/* The parity addend, in AVCMV register units (base >> 4). */
static u32 mt8365_mv_k(int bit)
{
	if (h264_mvk & bit) {
		h264_mvkhit++;
		return 0x4u;
	}
	return 0x2u;
}

static int h264_mvpoison;
module_param(h264_mvpoison, int, 0644);
MODULE_PARM_DESC(h264_mvpoison,
		 "mt8365 H.264 N120 (bitmask over the eight 16-byte records of a 128-byte field-MB colMV unit, default 0 = OFF): overwrite these records of the CO-LOCATED picture's colMV buffer with h264_mvpoison_val at the reading picture's ref-list programming. 0x0c = the top parity's records 2 and 3 (its BL/BR quadrants), which are also the bottom parity's records 0 and 1");

static unsigned int h264_mvpoison_val = 0xdeadbeef;
module_param(h264_mvpoison_val, uint, 0644);
MODULE_PARM_DESC(h264_mvpoison_val,
		 "mt8365 H.264 N120: the 32-bit word h264_mvpoison fills the selected colMV records with (default 0xdeadbeef)");

static int h264_mvpoison_cand;
module_param(h264_mvpoison_cand, int, 0644);
MODULE_PARM_DESC(h264_mvpoison_cand,
		 "mt8365 H.264 N120 instrument: co-located colMV buffers resolved at a reading picture's ref-list programming, counted UNCONDITIONALLY (the denominator, readable with the arm off)");

static int h264_mvpoison_hit;
module_param(h264_mvpoison_hit, int, 0644);
MODULE_PARM_DESC(h264_mvpoison_hit,
		 "mt8365 H.264 N120 instrument: co-located colMV buffers actually poisoned (execution of the h264_mvpoison arm)");

static int h264_mvpoison_units;
module_param(h264_mvpoison_units, int, 0644);
MODULE_PARM_DESC(h264_mvpoison_units,
		 "mt8365 H.264 N120 instrument: 128-byte field-MB colMV units written by the last h264_mvpoison execution");

/*
 * Poison the selected 16-byte records of every 128-byte field-MB unit of the
 * co-located picture's colMV buffer. Called at the READING picture's ref-list
 * programming (before the DECODE GO), so it lands after every store into that
 * buffer. The buffer is a coherent DMA mapping (the same one the N68/N83
 * snapshot instruments memcpy from), so a plain CPU store plus a barrier is
 * what the engine will fetch.
 */
static void mt8365_mv_poison(struct vdec_h264_mt8365_inst *inst, int sl)
{
	u32 *p, off, unit, rec;
	int units = 0;

	h264_mvpoison_cand++;
	if (!h264_mvpoison)
		return;
	if (sl < 0 || sl >= MT8365_MV_POOL_NUM || !inst->mv_pool[sl].va)
		return;
	p = (u32 *)inst->mv_pool[sl].va;
	for (off = 0; off + 128 <= (u32)inst->mv_pool[sl].size; off += 128) {
		unit = off / 4;
		for (rec = 0; rec < 8; rec++) {
			u32 w;

			if (!(h264_mvpoison & (1 << rec)))
				continue;
			for (w = 0; w < 4; w++)
				p[unit + rec * 4 + w] = h264_mvpoison_val;
		}
		units++;
	}
	wmb();	/* the engine fetches this buffer after the DECODE GO */
	h264_mvpoison_units = units;
	h264_mvpoison_hit++;
}

/* Effective colMV pool size: the arm's value when it is a sane power of two,
 * else the built-in MT8365_MV_POOL_NUM. */
static int mt8365_mv_pool_n(void)
{
	int n = h264_mvpool;

	if (n >= 2 && n <= MT8365_MV_POOL_NUM && !(n & (n - 1)))
		return n;
	return MT8365_MV_POOL_NUM;
}

static int h264_b19frm;
module_param(h264_b19frm, int, 0644);
MODULE_PARM_DESC(h264_b19frm,
		 "mt8365 H.264 N61 instrument: count of FRAME-form reference descriptors whose reference was FIELD-coded (candidates; counted even when the arm is off)");

static int h264_b19hit;
module_param(h264_b19hit, int, 0644);
MODULE_PARM_DESC(h264_b19hit,
		 "mt8365 H.264 N61 instrument: count of FRAME-form reference descriptors that actually received the extra bit19 (PAFF_FRMB19 / PAFF_FRMP19)");

/*
 * N88 -- H.264 High scaling matrices. DEFAULT ON (N93) at 1027 =
 * 1024|2|1: program the quant-matrix RAM on VDEC_LD (0x16020260/0x264),
 * raise the AVCVLD+0x94 bit18 global enable, and set the per-list
 * RAM-over-default select 0x94[17:10]. Streams whose effective scaling
 * matrix is flat (V4L2_H264_PPS_FLAG_SCALING_MATRIX_PRESENT clear --
 * every JVT-AVC_V1 vector and the 6 JVT-FR-EXT passers with
 * seq_scaling_matrix_present_flag=0) never enter this path at all, whatever
 * the bitmask is, so they stay byte-identical by construction.
 */
static int h264_scaling = 1027;
module_param(h264_scaling, int, 0644);
MODULE_PARM_DESC(h264_scaling,
		 "mt8365 H.264 N88 High scaling matrices (bitmask, non-flat-matrix pictures only; DEFAULT 1027 = 1024|2|1, N93): 1=program the AVCVLD+0x260/0x264 quant-matrix RAM and set 0x94 bit18, 2=also set the per-list in-use bits 0x94[17:10], 4=pack the RAM data word little-endian instead of the vendor's big-endian, 8=re-apply the H.264 zigzag scan before writing (the vendor writes raster), 16=log the delivered matrix for the first picture, 32=transpose each list before writing (the vendor's four default tables are symmetric, so a raster-vs-column-major reading of the RAM is invisible on them), 64=N89 SKIP the RAM writes and raise only the 0x94 bit18 enable (negative control: every vector the arm flipped programs the Table 7-2 defaults, so if the silicon holds those itself the RAM writes are inert), 128=N89 POISON the RAM with 0x10 everywhere (distinguishes inert from consumed), 256=N89 dump the words actually pushed through the port for the first programmed picture, 512=N89 push the RAM AFTER 0x94 instead of before (ordering control), 1024=N90 write the quant-matrix RAM to VDEC_LD (0x16020000) instead of VDEC_AD -- the blob routes 0x260/0x264 through H264_HAL_WriteVLD (hal_regs+0x18 = VDEC_LD) while 0x90/0x94 go through H264_HAL_WriteAVCVLD (hal_regs+0x30 = VDEC_AD), 2048=N90 read the port back on BOTH windows for the first picture");

static int h264_scalcand;
module_param(h264_scalcand, int, 0644);
MODULE_PARM_DESC(h264_scalcand,
		 "mt8365 H.264 N88 instrument: pictures whose PPS reports a non-flat scaling matrix (candidates; counted even when the arm is OFF)");

static int h264_scalhit;
module_param(h264_scalhit, int, 0644);
MODULE_PARM_DESC(h264_scalhit,
		 "mt8365 H.264 N88 instrument: pictures that actually had the quant-matrix RAM programmed (execution of the h264_scaling arm)");

/*
 * N94: monochrome (4:0:0) chroma-plane fill.
 *
 * A chroma_format_idc=0 bitstream codes NO chroma samples at all, so the HW
 * leaves the CAPTURE buffer's second (UV) plane at whatever it was -- which
 * this driver observes as 0. A conforming 4:0:0 presentation carries the
 * NEUTRAL value 128 (0x80) in both chroma components; the reference decoder's
 * output does. Measured N93: on HPCAMOLQ_BRCM_B and HPCVMOLQ_BRCM_B the LUMA is
 * bit-exact on 100/100 frames (mse_y 0.00) and the whole error is a constant
 * mse_u = mse_v = 16384 = 128^2 -- i.e. exactly a UV plane of 0 where it should
 * be 0x80. So the fix is a plane fill, not a decode change.
 *
 * WHERE: the CAPTURE buffer's chroma plane, fb->base_c, sized from
 * fb->base_c.size -- which is ctx->q_data[MTK_Q_DATA_DST].sizeimage[1], i.e.
 * this driver's OWN GET_PARAM_PIC_INFO geometry
 * (fb_sz[1] = ALIGN(pic_w,16) * ALIGN(pic_h,32) / 2). No second geometry is
 * hand-rolled here, and the fill therefore covers the full PADDED extent an
 * MM21 detiler reads, not just the visible region.
 *
 * TILING: MM21 is a tiled layout, but a CONSTANT-byte fill is tiling-INVARIANT
 * -- every byte of the plane takes the same value, so the tile permutation
 * (and the UV interleave) cannot change the result. That is why this is a
 * plain memset and not a tile-aware writer.
 *
 * WHEN: after the dec-done IRQ, per decoded picture. Post-IRQ so nothing the
 * HW writes can clobber it; per-picture because CAPTURE buffers are recycled
 * through the DPB and this backend has no buffer-allocation hook to fill once
 * (buffers arrive already-allocated via ctx->fb_ops->get_cap_buffer). The cost
 * is one memset of pic_w*pic_h/2 bytes on a monochrome stream only.
 *
 * GATED on SPS chroma_format_idc == 0, so no 4:2:0 stream is touched: the arm's
 * body is unreachable for every 4:2:0 vector by construction, exactly like the
 * h264_scaling non-flat-matrix gate.
 */
static int h264_mono = 1;
module_param(h264_mono, int, 0644);
MODULE_PARM_DESC(h264_mono,
		 "mt8365 H.264 N94 monochrome (chroma_format_idc=0) chroma-plane fill (bitmask, DEFAULT 1): 1=fill the CAPTURE chroma plane with h264_monoval after dec-done");

static int h264_monoval = 0x80;
module_param(h264_monoval, int, 0644);
MODULE_PARM_DESC(h264_monoval,
		 "mt8365 H.264 N94: the byte written by the h264_mono fill (default 0x80 = neutral chroma; set 0x40 as a POISON control -- a wrong constant must NOT pass)");

static int h264_monocand;
module_param(h264_monocand, int, 0644);
MODULE_PARM_DESC(h264_monocand,
		 "mt8365 H.264 N94 instrument: decoded pictures whose SPS says chroma_format_idc=0 (candidates; counted even when the arm is OFF)");

static int h264_monohit;
module_param(h264_monohit, int, 0644);
MODULE_PARM_DESC(h264_monohit,
		 "mt8365 H.264 N94 instrument: pictures whose chroma plane was actually filled (execution of the h264_mono arm)");

static const char * const mt8365_blkname[NUM_MAX_VDEC_REG_BASE] = {
	[VDEC_SYS] = "SYS", [VDEC_MISC] = "MISC", [VDEC_LD] = "VLD",
	[VDEC_TOP] = "TOP", [VDEC_CM] = "MC",     [VDEC_AD] = "AVCVLD",
	[VDEC_AV] = "AV",   [VDEC_PP] = "PP",
};

static inline void wr(struct mtk_vcodec_dec_ctx *ctx, int blk, u32 off, u32 val)
{
	if (unlikely(wrlog_on))
		pr_info("H264WR %-6s +0x%03x = 0x%08x\n",
			mt8365_blkname[blk] ? mt8365_blkname[blk] : "?",
			off, val);
	writel(val, ctx->dev->reg_base[blk] + off);
}

static inline u32 rd(struct mtk_vcodec_dec_ctx *ctx, int blk, u32 off)
{
	return readl(ctx->dev->reg_base[blk] + off);
}

/*
 * N133: the READ-side companion to h264_wrlog's H264WR write log.
 *
 * Consuming slice-header bits is a READ of AVCVLD (+0x23c ue, +0x240 se,
 * +(n<<2) consume-n), so a lever that changes only WHICH BITS WE CONSUME is
 * invisible in a write-only log -- and h264_pwtafter is exactly such a lever
 * (its write stream is predicted identical to the control's). This logs each
 * such consuming access, so "the header tail is no longer consumed" is
 * observable at the register rather than at a call-site counter.
 *
 * Gated on the SAME wrlog_on window as H264WR: off by default, symmetric
 * across both legs of an A/B, and it perturbs no leg when off.
 */
static inline void adv_log(u32 off, u32 pos)
{
	if (unlikely(wrlog_on))
		pr_info("H264ADV AVCVLD +0x%03x pos=%u\n", off, pos);
}

/* N88: the H.264 zigzag scans, only needed for the h264_scaling bit3 control
 * arm (the vendor writes the RAM in raster order; these are the blob's
 * ZZ_SCAN / ZZ_SCAN8 tables, kept so a wrong-order reading is testable). */
static const u8 mt8365_zz4[16] = {
	0, 1, 4, 8, 5, 2, 3, 6, 9, 12, 13, 10, 7, 11, 14, 15
};

static const u8 mt8365_zz8[64] = {
	 0,  1,  8, 16,  9,  2,  3, 10, 17, 24, 32, 25, 18, 11,  4,  5,
	12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13,  6,  7, 14, 21, 28,
	35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
	58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63
};

/*
 * N88: push one H.264 scaling list into the VDEC quant-matrix RAM through the
 * AVCVLD+0x260/0x264 address/data port, byte-for-byte in the vendor's
 * H264_HAL_WriteScalingList order (select -> data -> latch, 4 bytes a time).
 */
static void mt8365_h264_write_qm_list(struct mtk_vcodec_dec_ctx *ctx, u32 base,
				      const u8 *list, u32 n, bool dump)
{
	const u8 *scan = (n == 64) ? mt8365_zz8 : mt8365_zz4;
	u32 dim = (n == 64) ? 8 : 4;
	u32 words[16];
	u32 k;
	int blk;

	for (k = 0; k < n; k += 4) {
		u8 b[4];
		u32 d, j;

		for (j = 0; j < 4; j++) {
			u32 idx = k + j;

			/*
			 * bit5: transpose the list. The blob's four default
			 * tables are all SYMMETRIC about the diagonal, so a
			 * raster-vs-column-major reading of the RAM is
			 * invisible on them and only shows up on an
			 * explicitly-coded (generally asymmetric) list.
			 */
			if (h264_scaling & 32)
				idx = (idx % dim) * dim + (idx / dim);
			b[j] = (h264_scaling & 8) ? list[scan[idx]] : list[idx];
			/*
			 * N89 bit7 -- POISON. Write a value the stream never
			 * asked for. If a picture whose effective matrix is the
			 * Table 7-2 default still decodes bit-exact with the
			 * RAM poisoned, the RAM writes are INERT and the whole
			 * port model is wrong; if it breaks, the RAM really is
			 * the consumed surface. Either answer is decisive, and
			 * neither is obtainable from a default-valued vector
			 * with the honest values in place.
			 */
			if (h264_scaling & 128)
				b[j] = 0x10;
		}

		if (h264_scaling & 4)
			d = ((u32)b[3] << 24) | ((u32)b[2] << 16) |
			    ((u32)b[1] << 8) | b[0];
		else
			d = ((u32)b[0] << 24) | ((u32)b[1] << 16) |
			    ((u32)b[2] << 8) | b[3];

		words[k >> 2] = d;

		/*
		 * N90 bit10 -- the BLOCK. The blob's H264_HAL_WriteScalingList
		 * pushes 0x260/0x264 through H264_HAL_WriteVLD, whose mapped
		 * base is hal_regs[+0x18]; H264_HAL_SetPPSAVLD pushes 0x90/0x94
		 * through H264_HAL_WriteAVCVLD, whose base is hal_regs[+0x30].
		 * Those are two DIFFERENT windows -- +0x18 is VDEC_LD
		 * (0x16020000) and +0x30 is VDEC_AD (0x16022000). N88/N89 wrote
		 * the quant matrices to VDEC_AD, which is why they were inert
		 * while 0x94 bit18 (genuinely VDEC_AD) worked.
		 */
		blk = (h264_scaling & 1024) ? VDEC_LD : VDEC_AD;

		wr(ctx, blk, AVCVLD_QM_ADDR, AVCVLD_QM_ADDR_SEL | (base + k));
		wr(ctx, blk, AVCVLD_QM_DATA, d);
		wr(ctx, blk, AVCVLD_QM_ADDR, AVCVLD_QM_ADDR_LATCH | (base + k));

		/*
		 * N90 bit11 -- READBACK. Read the address/data pair straight
		 * back on BOTH windows for the first four words of the first
		 * programmed picture. A port that does not hold what was
		 * written to it is not the RAM.
		 */
		if ((h264_scaling & 2048) && dump && k < 16)
			mtk_v4l2_vdec_err(ctx,
				"mt8365 N90 QMRB base=0x%02x+%u wrote=%08x AD(a=%08x d=%08x) LD(a=%08x d=%08x)",
				base, k, d,
				rd(ctx, VDEC_AD, AVCVLD_QM_ADDR),
				rd(ctx, VDEC_AD, AVCVLD_QM_DATA),
				rd(ctx, VDEC_LD, AVCVLD_QM_ADDR),
				rd(ctx, VDEC_LD, AVCVLD_QM_DATA));
	}

	/*
	 * N89 bit8 -- dump the words ACTUALLY pushed through the port, not the
	 * bytes the control delivered. N88 verified delivery (the control's
	 * bytes matched an independent spec parse on freh1_b); it never
	 * verified that what leaves the driver equals what arrived.
	 */
	if (dump) {
		if (n == 16)
			mtk_v4l2_vdec_err(ctx,
				"mt8365 N89 QMPORT base=0x%02x %08x %08x %08x %08x",
				base, words[0], words[1], words[2], words[3]);
		else
			mtk_v4l2_vdec_err(ctx,
				"mt8365 N89 QMPORT base=0x%02x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x",
				base, words[0], words[1], words[2], words[3],
				words[4], words[5], words[6], words[7],
				words[8], words[9], words[10], words[11],
				words[12], words[13], words[14], words[15]);
	}
}

/* Step-10 control: the HEVC pre-kick VDEC_LD read, run on the H.264 path. */
static noinline void h264_ldread(struct mtk_vcodec_dec_ctx *ctx)
{
	u32 off, start, end;

	start = clamp_t(u32, (u32)mt8365_h264_ldread_off & ~31u, 0x100, 0x7e0);
	end = mt8365_h264_ldread_len > 0
		? min_t(u32, start + (u32)mt8365_h264_ldread_len, 0x800)
		: 0x800;
	for (off = start; off < end; off += 32) {
		u32 v[8];
		int i;

		for (i = 0; i < 8; i++)
			v[i] = rd(ctx, VDEC_LD, off + 4 * i);
		pr_info("H264RD PREKICK +0x%05x %08x %08x %08x %08x %08x %08x %08x %08x\n",
			0x20000 + off,
			v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]);
		h264_ld_lines++;
	}
}

/*
 * One-shot broad status dump to localize a decode-timeout stall. Reads each
 * sub-block's status window (8 regs/line). Only ever fires on the hang path,
 * and only touches offsets <= what the replay already writes, so it stays
 * inside the mapped bank. The point: compare the read-back values against the
 * values the replay wrote -- any register the HW *changed* is an activity
 * indicator (e.g. a VLD read-pointer/byte-count that advanced from 0 tells us
 * the bitstream parser actually consumed input vs. never started).
 */
static void mt8365_dump_blk(struct mtk_vcodec_dec_ctx *ctx, int blk,
			    const char *name, u32 start, u32 end)
{
	u32 off;

	for (off = start; off < end; off += 32)
		mtk_v4l2_vdec_err(ctx,
			"mt8365 dump %s+%03x: %08x %08x %08x %08x %08x %08x %08x %08x",
			name, off,
			rd(ctx, blk, off),      rd(ctx, blk, off + 4),
			rd(ctx, blk, off + 8),  rd(ctx, blk, off + 12),
			rd(ctx, blk, off + 16), rd(ctx, blk, off + 20),
			rd(ctx, blk, off + 24), rd(ctx, blk, off + 28));
}

/*
 * N60: emit one picture-structure register (MC+0x564/0x568/0x56c) either as a
 * single write of the final value (default, the pre-N60 form) or as the
 * vendor's accumulating per-PAIR read-modify-write walk. @steps holds the
 * successive accumulated values; @final is the value the register must end on.
 * See the h264_fldrmw comment.
 */
static void mt8365_fldstruct_emit(struct mtk_vcodec_dec_ctx *ctx, u32 off,
				  const u32 *steps, int nsteps, u32 final)
{
	int i;

	if (!(h264_fldrmw & FLDRMW_WALK) || nsteps <= 0) {
		wr(ctx, VDEC_CM, off, final);
		return;
	}
	for (i = 0; i < nsteps; i++) {
		rd(ctx, VDEC_CM, off);
		wr(ctx, VDEC_CM, off, steps[i]);
	}
	if (steps[nsteps - 1] != final) {
		rd(ctx, VDEC_CM, off);
		wr(ctx, VDEC_CM, off, final);
	}
}

#define MT8365_WORK_BUF_NUM	5	/* [0..3] MC/MV/deblock scratch; [4] AVC_MV colocated-MV write buf */

static void free_work_bufs(struct vdec_h264_mt8365_inst *inst)
{
	int i;

	for (i = 0; i < MT8365_WORK_BUF_NUM; i++)
		if (inst->work_buf[i].va)
			mtk_vcodec_mem_free(inst->ctx, &inst->work_buf[i]);
	for (i = 0; i < MT8365_MV_POOL_NUM; i++) {
		if (inst->mv_pool[i].va)
			mtk_vcodec_mem_free(inst->ctx, &inst->mv_pool[i]);
		inst->mv_pool_key[i] = 0;
	}
	inst->work_buf_ready = false;
}

/*
 * Resolve the per-frame-store colMV pool slot that owns recon-Y address @y.
 * Returns an existing slot if @y is already mapped (the frame is still a live
 * ref), else assigns a free slot (or, if the pool is full, evicts slot 0). The
 * returned index IS the value the AVCMV descriptor high field encodes, and
 * mv_pool[idx] is the colMV buffer that frame stores/reads its MVs through --
 * keeping the write (decode-time AVCMV+0x20c) and the read (ref-time
 * AVCMV+0x180) pointed at the SAME physical buffer per frame-store.
 */
/*
 * CURRENT frame: consume the next DECODE-ORDER ring slot (vendor scheme --
 * mv_ring_next++ mod pool), and stamp this frame-store's recon-Y as the slot's
 * owner. Every decoded frame (I, P, and non-ref B) advances the counter, so the
 * slot a frame-store is assigned == the vendor's decode-order index and stays
 * fixed for the frame-store's lifetime (write at decode == read when referenced).
 * A recycled recon buffer may still be keyed to its PREVIOUS frame-store in an
 * older ring slot; clear that stale key so a later reference lookup of this
 * address resolves to the NEW slot, not the evicted frame's. N16/B4.
 */
/*
 * Is colMV-pool key @y (a recon-Y dma_addr) currently owned by a frame-store
 * that is STILL in the DPB (a live reference)? inst->dpb[] is filled by
 * mtk_vdec_h264_fill_dpb_info() before this runs: reference_flag != 0 marks an
 * active DPB entry with a valid y_dma_addr. Stealing such a slot would strand
 * that reference's colMV -- its later mt8365_mv_lookup() would miss and fall
 * back to slot 0, mis-directing the temporal-direct co-located derivation.
 */
static bool mt8365_key_live(struct vdec_h264_mt8365_inst *inst, u32 y)
{
	int j;

	if (!y)
		return false;
	for (j = 0; j < V4L2_H264_NUM_DPB_ENTRIES; j++)
		if (inst->dpb[j].reference_flag &&
		    (u32)inst->dpb[j].y_dma_addr == y)
			return true;
	return false;
}

static int mt8365_mv_assign(struct vdec_h264_mt8365_inst *inst, u32 y,
			    bool fld)
{
	int pool = mt8365_mv_pool_n();		/* N66: capped effective pool */
	int i, slot, ring0;

	/*
	 * N68: the complementary field of this frame-store may already own a
	 * slot. Counted unconditionally so the denominator is observable with
	 * the arm off; the ring is deliberately NOT advanced on a reuse, so a
	 * frame-store consumes exactly ONE slot, as the vendor's does.
	 */
	if (y && fld) {
		for (i = 0; i < MT8365_MV_POOL_NUM; i++)
			if (inst->mv_pool_key[i] == y) {
				bool adj = (inst->mv_last_fld &&
					    inst->mv_last_y == y &&
					    inst->mv_last_slot == i);

				h264_mvpaircand++;
				if (adj)
					h264_mvpairadj++;
				if ((h264_mvpair & MVPAIR_SHARE) &&
				    (!(h264_mvpair & MVPAIR_ADJACENT) || adj)) {
					h264_mvpairhit++;
					inst->mv_last_y = y;
					inst->mv_last_slot = i;
					inst->mv_last_fld = 1;
					return i;
				}
				break;
			}
	}

	slot = inst->mv_ring_next % pool;
	ring0 = inst->mv_ring_next % MT8365_MV_POOL_NUM;
	inst->mv_ring_next++;
	if (!y)
		return 0;
	/*
	 * The decode-order candidate slot (ring_next % pool) is the vendor's
	 * numbering; keep it whenever it is free or owned by an already-evicted
	 * frame-store -- so shallow-DPB streams (CVBS3 et al., whose wrapped-slot
	 * owner has long left the DPB) get byte-identical behaviour to the pure
	 * decode-order ring. But for a deep DPB (max_num_ref_frames up to 16 with a
	 * long B GOP) the candidate frequently lands on a slot still owned by a LIVE
	 * reference; taking it would strand that ref's colMV. Re-home the current
	 * frame to a slot that is free or owned by an evicted frame-store, preserving
	 * every live ref's slot. Only if the whole pool is live (>= MT8365_MV_POOL_NUM
	 * live frame-stores, impossible for a <=17-deep DPB now the pool is 32) do we
	 * fall back to the candidate. The HW consults the +0x180 colMV ADDRESS keyed
	 * by this slot, and self-consistency (write slot == read slot per frame-store)
	 * is all it requires -- the exact slot NUMBER need not match the vendor
	 * (proven by the canonical clip decoding bit-exact with non-vendor numbers).
	 */
	if (mt8365_key_live(inst, inst->mv_pool_key[slot]) &&
	    inst->mv_pool_key[slot] != y) {
		int cand = -1;

		for (i = 0; i < pool; i++) {
			u32 k = inst->mv_pool_key[i];

			if (k == y) {		/* already mine -- reuse */
				cand = i;
				break;
			}
			if (cand < 0 && (k == 0 || !mt8365_key_live(inst, k)))
				cand = i;	/* first free / evicted slot */
		}
		if (cand >= 0)
			slot = cand;
	}
	for (i = 0; i < MT8365_MV_POOL_NUM; i++)
		if (i != slot && inst->mv_pool_key[i] == y)
			inst->mv_pool_key[i] = 0;
	inst->mv_pool_key[slot] = y;
	/*
	 * N66 instrument: did the cap move this frame-store off the slot the
	 * uncapped 32-deep ring would have handed it?  Counted only when the arm
	 * is active, so it is a true execution proof (arm off => 0 by
	 * construction; the DENOMINATOR is h264_mvpoolcand, counted below on
	 * every emitted descriptor regardless of the arm).
	 */
	if (pool != MT8365_MV_POOL_NUM && slot != ring0)
		h264_mvpoolhit++;
	inst->mv_last_y = y;
	inst->mv_last_slot = slot;
	inst->mv_last_fld = fld ? 1 : 0;
	return slot;
}

/*
 * REFERENCE: find the decode-order ring slot the ref frame-store was assigned
 * when IT decoded (lookup only -- never mutates the ring). A ref that is still
 * in the DPB always has a live key; slot 0 is the harmless fallback for a
 * not-found (evicted) address.
 */
static int mt8365_mv_lookup(struct vdec_h264_mt8365_inst *inst, u32 y)
{
	int i;

	if (!y)
		return 0;
	for (i = 0; i < MT8365_MV_POOL_NUM; i++)
		if (inst->mv_pool_key[i] == y) {
			/*
			 * N66 instrument, counted UNCONDITIONALLY so the
			 * denominator is readable with the arm off: reference
			 * resolutions that hand the HW a colMV tag >= 16, i.e.
			 * outside the range the vendor is ever observed to
			 * emit (max 12 across every trace on disk).
			 */
			if (i >= 16)
				h264_mvpoolcand++;
			return i;
		}
	return 0;
}

/* Allocate the VDEC's MV/deblock/colocated scratch buffers for a w*h picture.
 * The exact per-buffer sizes/semantics are still refined against the live
 * decode; for the first bring-up each is sized with the shared H.264 MV-buffer
 * formula (a generous per-frame size that scales with the picture) floored at
 * BUF_PREDICTION_SZ, so the HW always has valid, large-enough scratch DMA and
 * cannot fault on a NULL/undersized work buffer. Reallocated on a res change. */
static int alloc_work_bufs(struct vdec_h264_mt8365_inst *inst, u32 w, u32 h)
{
	unsigned int sz = mtk_vdec_h264_get_mv_buf_size(w, h);
	int i, err;

	if (sz < BUF_PREDICTION_SZ)
		sz = BUF_PREDICTION_SZ;
	free_work_bufs(inst);
	for (i = 0; i < MT8365_WORK_BUF_NUM; i++) {
		inst->work_buf[i].size = sz;
		err = mtk_vcodec_mem_alloc(inst->ctx, &inst->work_buf[i]);
		if (err) {
			free_work_bufs(inst);
			return err;
		}
	}
	/* per-frame-store colMV pool: one mv-sized buffer per DPB frame-store */
	for (i = 0; i < MT8365_MV_POOL_NUM; i++) {
		inst->mv_pool[i].size = sz;
		err = mtk_vcodec_mem_alloc(inst->ctx, &inst->mv_pool[i]);
		if (err) {
			free_work_bufs(inst);
			return err;
		}
		inst->mv_pool_key[i] = 0;
	}
	inst->mv_ring_next = 0;
	inst->work_buf_w = w;
	inst->work_buf_h = h;
	inst->work_buf_ready = true;
	return 0;
}

static int vdec_h264_mt8365_init(struct mtk_vcodec_dec_ctx *ctx)
{
	struct vdec_h264_mt8365_inst *inst;

	inst = kzalloc(sizeof(*inst), GFP_KERNEL);
	if (!inst)
		return -ENOMEM;
	inst->ctx = ctx;
	ctx->drv_handle = inst;
	wrlog_idx = 0;		/* N45: restart the h264_wrlog window per decode */
	mvdump_idx = 0;		/* N68: restart the h264_mvdump window per decode */
	h264_mvdump_taken = 0;
	return 0;
}

/* ------------------------------------------------------------------------- *
 * Minimal in-kernel H.264 slice-header parser (Exp-Golomb).
 *
 * Used ONLY on the FRAME-BASED path (GStreamer v4l2slh264dec), where a
 * spec-compliant userspace supplies SPS/PPS/DECODE_PARAMS as controls but leaves
 * the slice-based-only H264_SLICE_PARAMS fields zero (a frame-based HW is
 * expected to derive them from the bitstream). This no-SCP backend has no
 * firmware to do that, so it parses the coded slice NAL itself. When userspace
 * DOES populate SLICE_PARAMS (the direct harness / any slice-based-aware client)
 * the parser is NOT invoked -- the supplied controls are used verbatim (see the
 * adaptive block in decode()). Mirrors the proven userspace parser
 * (the internal v4l2 decode harness) + H.264 spec Sec 7.3.3.
 * Scope: Baseline/Main/High, P + I (B header bits parsed, L0-only ref derive).
 * CAVLC or CABAC, frame coding (no field/MBAFF).
 * ------------------------------------------------------------------------- */

/* one ref_pic_list_modification entry (modification_of_pic_nums_idc + value) */
struct mt8365_rplm_op {
	u8 idc;
	u32 val;
};

struct mt8365_br {		/* MSB-first bit reader over an RBSP buffer */
	const u8 *d;
	u32 nbits;
	u32 p;
};

static u32 mt8365_br_u1(struct mt8365_br *b)
{
	u32 v;

	if (b->p >= b->nbits)
		return 0;
	v = (b->d[b->p >> 3] >> (7 - (b->p & 7))) & 1;
	b->p++;
	return v;
}

static u32 mt8365_br_u(struct mt8365_br *b, int n)
{
	u32 v = 0;

	while (n-- > 0)
		v = (v << 1) | mt8365_br_u1(b);
	return v;
}

static u32 mt8365_br_ue(struct mt8365_br *b)
{
	int z = 0;

	while (mt8365_br_u1(b) == 0 && z < 32 && b->p < b->nbits)
		z++;
	return (u32)((1u << z) - 1) + (z ? mt8365_br_u(b, z) : 0);
}

static int mt8365_br_se(struct mt8365_br *b)
{
	u32 k = mt8365_br_ue(b);

	return (k & 1) ? (int)((k + 1) / 2) : -(int)(k / 2);
}

/* strip H.264 emulation-prevention 0x03 bytes (00 00 03 -> 00 00) */
static u32 mt8365_unescape(const u8 *in, u32 n, u8 *out, u32 outmax)
{
	u32 i = 0, o = 0;

	while (i < n && o < outmax) {
		if (i + 2 < n && in[i] == 0 && in[i + 1] == 0 && in[i + 2] == 3) {
			out[o++] = in[i];
			if (o < outmax)
				out[o++] = in[i + 1];
			i += 3;
		} else {
			out[o++] = in[i++];
		}
	}
	return o;
}

/*
 * Parse the first VCL slice NAL in @buf. On success fills:
 *   *out_hbs        header_bit_size -- bit offset from OUTPUT-buffer byte 0 to
 *                   slice_data() (start code + NAL header + slice header). This
 *                   is exactly what the VLD barrel-shifter consumes, and matches
 *                   the userspace parser's convention (bytes*8 up to the NAL
 *                   header + the RBSP bits of the slice header).
 *   *out_first_mb   first_mb_in_slice
 *   *out_slice_type slice_type mod 5 (0=P 1=B 2=I 3=SP 4=SI)
 *   *out_num_ref_l0 num_ref_idx_l0 active COUNT (slice override else PPS default)
 *   *out_num_ref_l1 num_ref_idx_l1 active COUNT for B (slice override else PPS
 *                   default); 0 for non-B -- feeds AVCVLD 0x98[29:25]
 *   ops[]/*out_nops ref_pic_list_modification l0 ops (idc 0/1/2), capped at 34
 * Returns 0 or -EINVAL.
 */
static int mt8365_parse_slice_header(const struct v4l2_ctrl_h264_sps *sps,
				     const struct v4l2_ctrl_h264_pps *pps,
				     const u8 *buf, u32 buf_sz, u32 nal_skip,
				     u32 *out_hbs, u32 *out_first_mb,
				     u32 *out_slice_type, int *out_num_ref_l0,
				     int *out_num_ref_l1,
				     int *out_qp_delta, int *out_dbf_idc,
				     int *out_alpha, int *out_beta,
				     int *out_direct_spatial,
				     int *out_field_pic, int *out_bottom_field,
				     u32 *out_pwt_start, u32 *out_pwt_end,
				     struct mt8365_rplm_op *ops, int *out_nops,
				     int *out_cabac_init,
				     struct mt8365_rplm_op *ops_l1,
				     int *out_nops_l1)
{
	u8 rbsp[512];
	struct mt8365_br br;
	u32 i, nal_off = 0, rlen;
	int nal_type = -1, nal_ref_idc = 0, st, num_l0, num_l1, nops = 0;
	int nops_l1 = 0;
	bool is_idr, cabac;

	*out_cabac_init = 0;
	*out_nops_l1 = 0;

	if (!buf || buf_sz < 5)
		return -EINVAL;

	/*
	 * Find VCL slice NAL #@nal_skip (type 1 or 5) + its NAL-header byte
	 * offset. nal_skip == 0 is the historical "first VCL NAL" behaviour and
	 * is byte-identical to the pre-N34 scan.
	 */
	for (i = 0; i + 3 <= buf_sz;) {
		if (buf[i] == 0 && buf[i + 1] == 0 && buf[i + 2] == 1) {
			u32 h = i + 3;

			if (h < buf_sz) {
				int t = buf[h] & 0x1f;

				if (t == NAL_NON_IDR_SLICE || t == NAL_IDR_SLICE) {
					if (nal_skip) {
						nal_skip--;
					} else {
						nal_off = h;
						nal_type = t;
						nal_ref_idc = (buf[h] >> 5) & 3;
						break;
					}
				}
			}
			i += 3;
		} else {
			i++;
		}
	}
	if (nal_type < 0)
		return -EINVAL;
	is_idr = (nal_type == NAL_IDR_SLICE);
	cabac = !!(pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE);

	rlen = mt8365_unescape(buf + nal_off + 1, buf_sz - nal_off - 1,
			       rbsp, sizeof(rbsp));
	br.d = rbsp;
	br.nbits = rlen * 8;
	br.p = 0;

	*out_first_mb = mt8365_br_ue(&br);		/* first_mb_in_slice */
	st = mt8365_br_ue(&br) % 5;			/* slice_type mod 5 */
	mt8365_br_ue(&br);				/* pic_parameter_set_id */
	if (sps->flags & V4L2_H264_SPS_FLAG_SEPARATE_COLOUR_PLANE)
		mt8365_br_u(&br, 2);			/* colour_plane_id */
	mt8365_br_u(&br, sps->log2_max_frame_num_minus4 + 4);	/* frame_num */
	if (!(sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY)) {
		if (mt8365_br_u1(&br)) {			/* field_pic_flag */
			*out_field_pic = 1;
			*out_bottom_field = mt8365_br_u1(&br);	/* bottom_field_flag */
		}
	}
	if (is_idr)
		mt8365_br_ue(&br);			/* idr_pic_id */
	if (sps->pic_order_cnt_type == 0) {
		mt8365_br_u(&br, sps->log2_max_pic_order_cnt_lsb_minus4 + 4);
		/*
		 * delta_pic_order_cnt_bottom is present iff
		 * (bottom_field_pic_order_in_frame_present_flag && !field_pic_flag)
		 * -- H.264 7.3.3. For a FIELD picture it is ABSENT; reading it
		 * anyway (the pre-fix bug) consumes a spurious se(v), inflating
		 * header_bit_size and advancing the VLD barrel-shifter PAST
		 * slice_data() -> CAVLC desync -> flat-DC/blank field (AVCVLD err
		 * 0x20000000). Gate on !field_pic_flag. Frame-coded pictures have
		 * *out_field_pic==0 so this is byte-identical for them.
		 */
		if ((pps->flags & V4L2_H264_PPS_FLAG_BOTTOM_FIELD_PIC_ORDER_IN_FRAME_PRESENT) &&
		    !*out_field_pic)
			mt8365_br_se(&br);		/* delta_pic_order_cnt_bottom */
	} else if (sps->pic_order_cnt_type == 1 &&
		   !(sps->flags & V4L2_H264_SPS_FLAG_DELTA_PIC_ORDER_ALWAYS_ZERO)) {
		mt8365_br_se(&br);			/* delta_pic_order_cnt[0] */
		/* delta_pic_order_cnt[1]: same !field_pic_flag gate (7.3.3). */
		if ((pps->flags & V4L2_H264_PPS_FLAG_BOTTOM_FIELD_PIC_ORDER_IN_FRAME_PRESENT) &&
		    !*out_field_pic)
			mt8365_br_se(&br);		/* delta_pic_order_cnt[1] */
	}
	if (pps->flags & V4L2_H264_PPS_FLAG_REDUNDANT_PIC_CNT_PRESENT)
		mt8365_br_ue(&br);			/* redundant_pic_cnt */
	if (st == 1)					/* B */
		*out_direct_spatial = mt8365_br_u1(&br); /* direct_spatial_mv_pred_flag */
	num_l0 = pps->num_ref_idx_l0_default_active_minus1 + 1;
	num_l1 = pps->num_ref_idx_l1_default_active_minus1 + 1;
	if (st == 0 || st == 1 || st == 3) {		/* P, B, SP */
		if (mt8365_br_u1(&br)) {		/* num_ref_idx_active_override */
			num_l0 = mt8365_br_ue(&br) + 1;
			if (st == 1)
				num_l1 = mt8365_br_ue(&br) + 1;
		}
	}
	/* ref_pic_list_modification (l0) -- non-I/SI */
	if (st != 2 && st != 4) {
		if (mt8365_br_u1(&br)) {		/* ..._flag_l0 */
			for (;;) {
				u32 idc;

				/*
				 * Bound on bitstream exhaustion. mt8365_br_ue()
				 * returns 0 once br.p >= br.nbits, and idc==0
				 * matches neither break below -- so a truncated
				 * or degenerate slice header (e.g. a tiny
				 * disposable B slice in MR8_BT_B) spins this
				 * decode worker forever (CPU-bound, job_finish
				 * never called), wedging a later STREAMOFF in
				 * v4l2_m2m_cancel_job() as an unkillable D-state.
				 */
				if (br.p >= br.nbits)
					break;
				idc = mt8365_br_ue(&br);

				if (idc == 3)
					break;
				if (idc > 2)
					break;		/* malformed */
				{
					u32 v = mt8365_br_ue(&br);

					if (nops < 34) {
						ops[nops].idc = (u8)idc;
						ops[nops].val = v;
						nops++;
					}
				}
			}
		}
	}
	if (st == 1) {					/* B: ref_pic_list_mod l1 */
		if (mt8365_br_u1(&br)) {
			for (;;) {
				u32 idc;

				/* Same EOF bound as the l0 loop above. */
				if (br.p >= br.nbits)
					break;
				idc = mt8365_br_ue(&br);

				if (idc == 3 || idc > 2)
					break;
				{
					/*
					 * N50: CAPTURE the L1 modification ops.
					 * They used to be parsed and thrown away
					 * -- the driver applied ref_pic_list_
					 * modification to L0 only. The N50 HW
					 * read-back channel showed the hardware's
					 * own reader decoding a NON-EMPTY L1
					 * modification list on HCMP1_HHI_A (14
					 * slices), so "the L1 we program equals
					 * the L1 in the bitstream" was false.
					 */
					u32 v = mt8365_br_ue(&br);

					if (nops_l1 < 34) {
						ops_l1[nops_l1].idc = (u8)idc;
						ops_l1[nops_l1].val = v;
						nops_l1++;
					}
				}
			}
		}
	}
	*out_nops_l1 = nops_l1;
	/* pred_weight_table (present iff weighted_pred for P/SP, or wbipred==1 for B) */
	*out_pwt_start = 0;
	*out_pwt_end = 0;
	if (((pps->flags & V4L2_H264_PPS_FLAG_WEIGHTED_PRED) && (st == 0 || st == 3)) ||
	    (pps->weighted_bipred_idc == 1 && st == 1)) {
		bool chroma = sps->chroma_format_idc != 0;
		int cnts[2] = { num_l0, (st == 1) ? num_l1 : 0 };
		int l, k;

		/*
		 * N28: record the BIT WINDOW this syntax structure occupies, in the
		 * same OUTPUT-buffer bit space as *out_hbs. The weights themselves are
		 * still skipped here (nothing in the driver consumes them) -- the HW
		 * re-parses them from its own barrel shifter, see the AVCVLD+0x230
		 * handshake in decode().
		 */
		*out_pwt_start = nal_off * 8 + 8 + br.p;
		mt8365_br_ue(&br);			/* luma_log2_weight_denom */
		if (chroma)
			mt8365_br_ue(&br);		/* chroma_log2_weight_denom */
		for (l = 0; l < 2; l++) {
			for (k = 0; k < cnts[l] && k < 32; k++) {
				if (mt8365_br_u1(&br)) {	/* luma_weight_lX_flag */
					mt8365_br_se(&br);	/* weight */
					mt8365_br_se(&br);	/* offset */
				}
				if (chroma && mt8365_br_u1(&br)) { /* chroma_weight_lX_flag */
					mt8365_br_se(&br);
					mt8365_br_se(&br);
					mt8365_br_se(&br);
					mt8365_br_se(&br);
				}
			}
		}
		*out_pwt_end = nal_off * 8 + 8 + br.p;
	}
	/* dec_ref_pic_marking */
	if (nal_ref_idc != 0) {
		if (is_idr) {
			mt8365_br_u1(&br);	/* no_output_of_prior_pics_flag */
			mt8365_br_u1(&br);	/* long_term_reference_flag */
		} else if (mt8365_br_u1(&br)) {	/* adaptive_ref_pic_marking_mode */
			for (;;) {
				u32 op = mt8365_br_ue(&br);	/* mmco */

				if (op == 0 || op > 6)
					break;
				if (op == 1 || op == 3)
					mt8365_br_ue(&br);
				if (op == 2)
					mt8365_br_ue(&br);
				if (op == 3 || op == 6)
					mt8365_br_ue(&br);
				if (op == 4)
					mt8365_br_ue(&br);
			}
		}
	}
	if (cabac && st != 2 && st != 4)
		*out_cabac_init = (int)mt8365_br_ue(&br);	/* cabac_init_idc */
	*out_qp_delta = mt8365_br_se(&br);		/* slice_qp_delta */
	/*
	 * Deblocking-filter control. disable_deblocking_filter_idc (0=on,
	 * 1=off, 2=on-except-slice-boundary) and the alpha/beta offsets are
	 * per-slice and MUST reach the HW: the vendor packs them into
	 * AVCVLD+0x9c ([8:7]=idc, [12:9]=alpha_div2, [16:13]=beta_div2 --
	 * vpud SetSHDR/DecStart RE). Parsing them only to advance the reader
	 * left the HW with idc=0 (filter always ON), so any clip that signals
	 * the loop filter OFF (idc=1) decoded a spuriously-deblocked frame
	 * (~42-45 dB, errors clustered on the 4-px block edges). Capture them.
	 */
	*out_dbf_idc = 0;
	*out_alpha = 0;
	*out_beta = 0;
	if (pps->flags & V4L2_H264_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT) {
		*out_dbf_idc = mt8365_br_ue(&br);	/* disable_deblocking_filter_idc */
		if (*out_dbf_idc != 1) {
			*out_alpha = mt8365_br_se(&br);	/* slice_alpha_c0_offset_div2 */
			*out_beta = mt8365_br_se(&br);	/* slice_beta_offset_div2 */
		}
	}
	/* slice_group_change_cycle (FMO) is not supported by this HW -> skipped. */

	if (br.p > br.nbits)		/* overran the RBSP -> header longer than parsed */
		return -EINVAL;
	*out_hbs = nal_off * 8 + 8 + br.p;
	*out_slice_type = (u32)st;
	*out_num_ref_l0 = num_l0;
	*out_num_ref_l1 = (st == 1) ? num_l1 : 0;	/* B: active L1 count -> 0x98[29:25] */
	*out_nops = nops;
	return 0;
}

/*
 * ===================== N49: the VENDOR slice-header model =====================
 *
 * Transcribed from libvpud_vcodec.so (mt8365, aarch64). The vendor's
 * parseSliceHeaderHW @0x54860 is NOT a software bit reader: it programs
 * NOTHING, and every syntax element it "parses" is an MMIO READ of the AVCVLD
 * block, i.e. the HARDWARE's own bit reader decodes and CONSUMES it:
 *
 *   H264_HAL_ReadAVCVLD(hal,off,&v)   @0x489b0  v = readl(hal[48] + off)
 *   H264_HAL_AVCVLDGetBitS(hal,n)     @0x490c0  ReadAVCVLD(off = n << 2)
 *   H264_HAL_GetRealBitStream(hal,n)  @0x49194  peek = GetBitS(0);
 *                                               GetBitS(n);
 *                                               return peek >> (32 - n)
 *   H264_HAL_GetBitStreamFlg(hal)     @0x491f0  GetBitStreamShift(1) >> 31
 *   H264_HAL_UeCodeNum(hal)           @0x49220  ReadAVCVLD(off = 0x23c)
 *   H264_HAL_SeCodeNum(hal)           @0x49290  ReadAVCVLD(off = 0x240)
 *
 * So AVCVLD+0x00 is a 32-bit PEEK window, AVCVLD+(n<<2) is a consume-n-bits
 * strobe (this is the bulk advance our driver already uses), and AVCVLD+0x23c /
 * +0x240 are the HARDWARE Exp-Golomb ue(v) / se(v) decoders -- one register read
 * returns the decoded value AND advances the barrel shifter by the codeword's
 * length. parseSliceHeaderHW walks the whole slice header through those, in
 * spec order (the fprintf format strings name every element: u4FirstMbInSlice,
 * u4SliceType, u4PPSID, u4FrameNum, bFieldPicFlag, i4PicOrderCntLsb,
 * i4DeltaPicOrderCntBottom, bDirectSpatialMvPredFlag,
 * bNumRefIdxActiveOverrideFlag, u4NumRefIdxL0/L1ActiveMinus1, then
 * "pred_weight_table();", then "dec_ref_pic_marking();", u4CabacInitIdc,
 * i4SliceQpDelta, u4DisableDeblockingFilterIdc, i4SliceAlphaC0OffsetDiv2,
 * i4SliceBetaOffsetDiv2), and calls H264_HAL_PredWeightTable at +0x390 exactly
 * where pred_weight_table() sits -- the HW then parses that sub-structure out
 * of its own reader.
 *
 * N48 fired that handshake with the reader advanced to the same BIT COUNT by
 * bulk GetBitS reads, and it was pixel-inert. The surviving hypothesis was that
 * bit-count equality is not state equality -- that the weight latch only means
 * something when the HW's own parse state machine has walked the header. This
 * block implements the vendor model so that hypothesis can be MEASURED.
 *
 * Everything here is gated behind h264_hwsh. At 0 not one extra register
 * access is issued and the driver is byte-for-byte the N48 driver.
 *
 * N131/N132 (2026-08-24): bit0 is now DEFAULT ON (h264_hwsh = HWSH_WALK).
 *
 * N49 parked this at 0 because the AVCVLD+0x230 weight strobe moved zero
 * pixels -- but its A/B covered only the four weighted_pred_flag P-slice
 * vectors (CVWP1/CVWP5/CAWP1/CAWP5). It never ran CVWP2_TOSHIBA_E, the
 * weighted_bipred_idc == 1 explicit-bipred B vector, and the vendor binary has
 * no B call site for the strobe at all -- so this is a path our driver can
 * drive that the vendor's software does not. N131 established WHY it works:
 * the SILICON parses pred_weight_table() for itself, and the strobe is the
 * latch. Fired from our own HW walk, CVWP2_TOSHIBA_E becomes bit-exact against
 * the ffmpeg reference on Y, Cb AND Cr (md5 7616ccc1e4cc89805cf5ae11e4437533,
 * 90/90 frames), and it also removes that vector's run-to-run
 * nondeterminism -- at 0 the same clip produced five different md5s across two
 * boots; at 1 it produces the reference md5, every time.
 *
 * Measured non-regression for this default change (N132, kernel
 * 6.18.35-01361-gc39229356453, clean boot, paired same-boot control legs, per
 * VECTOR NAME in both comm directions):
 *   JVT-AVC_V1     103 -> 104/135; lost direction EMPTY, gained = exactly
 *                  CVWP2_TOSHIBA_E; two extraction rules agree at 104.
 *   JVT-FR-EXT     37/69, all twelve chunks incl. the four wedge-prone ones on
 *                  their own boots; the passer NAME SET is byte-identical to
 *                  the h264_hwsh=0 leg and to the committed 37-name reference.
 *   VP8            61/61.  VP9 272/305, non-passer name set unchanged, both
 *                  bit-exact clips at 9a1cd425ad286aeacf18dadc2824d808 and
 *                  5dac7fa0fa4accc7cfed87c1c96ad6ae.
 *   9 standing bit-exact controls OK at every chunk boundary, taint 0.
 *
 * Bits 1-3 remain diagnostics and stay OFF; bit1 in particular does NOT
 * reproduce the gain (reading the table with our own ue/se consumes the bits
 * without latching anything), which is what proves the strobe does the work
 * rather than the walk.
 */
#define HWSH_WALK	0x1	/* element-by-element HW parse instead of bulk advance */
#define HWSH_PWT_READ	0x2	/* read pred_weight_table() with HW ue/se (no 0x230 strobe) */
#define HWSH_LOG	0x4	/* per-element trace */
#define HWSH_SYN	0x8	/* N50: HW-view vs our-view syntax comparison line */

#define AVCVLD_PEEK	0x000
#define AVCVLD_UE	0x23c
#define AVCVLD_SE	0x240

static int h264_hwsh = HWSH_WALK;
module_param(h264_hwsh, int, 0644);
MODULE_PARM_DESC(h264_hwsh,
		 "mt8365 H.264 N49 vendor-model HW slice-header parse BITMASK: 1=walk the slice header element-by-element through the AVCVLD HW bit reader (ue=+0x23c, se=+0x240, u(n)=+(n<<2)) instead of bulk-advancing, firing AVCVLD+0x230 at the vendor's pred_weight_table position, 2=read pred_weight_table() with our own HW ue/se reads instead of the 0x230 strobe (diagnostic: proves the HW reader is positioned), 4=per-element trace, 8=N50 HW-view-vs-our-view syntax comparison (field_pic/bottom_field/direct_spatial/active counts/cabac_init_idc/ref-list modification, one 'hw' + one 'our' line per slice) (default 1 = N131/N132 the silicon parses pred_weight_table() and +0x230 is its latch; set 0 for the pre-N49 byte-for-byte behaviour)");

/*
 * N132 -- `h264_pwtstop`: PARK the hardware reader at pred_weight_table(),
 * instead of advancing it past the table.
 *
 * N131 §9 named this and did not build it. The architecture it rests on is
 * N131's result: the SILICON parses pred_weight_table() out of its own barrel
 * shifter. The vendor's software reader is provably parked on the FIRST BIT of
 * the table when DecStart fires -- it consumes 3 ue + 1 u(4) + 2 u(1) + 1 se
 * per weighted P picture, never the table, and never writes AVCVLD+0x230 on
 * these clips -- and the vendor is bit-exact anyway.
 *
 * Our driver does the opposite: it advances the shifter all the way to
 * header_bit_size (h264_hwsh=0 in bulk, h264_hwsh=1 element by element), so at
 * DecStart the engine has been DENIED the bits it would parse for itself. That
 * is a difference in WHERE THE ADVANCE STOPS, and this arm stops it at
 * pwt_start.
 *
 *   bit0 (1) -- park on P/SP (weighted_pred_flag) weighted slices. This is the
 *               bucket that is still capped: CVWP1, CAWP1, CVWP5, CAWP5.
 *   bit1 (2) -- extend the parking to B (weighted_bipred_idc == 1) slices.
 *               DELIBERATELY SEPARATE from bit0 so the landed h264_hwsh win on
 *               CVWP2_TOSHIBA_E (a B vector, which the +0x230 strobe fixes) is
 *               untouched by bit0 and the two levers score separately.
 *   bit2 (4) -- with parking on, also re-program AVCVLD+0x9c[6:0] with the
 *               DESYNCED slice_qp_delta a reader standing at pwt_start decodes
 *               (u(1) then se(v)), read through the SAME hardware Exp-Golomb
 *               decoders the vendor uses. This reproduces the vendor's captured
 *               0x0002007f rather than the correct value our driver takes from
 *               the V4L2 controls. Diagnostic: it is what makes the registered
 *               "0x9c[6:0] should read 0x7f" prediction testable.
 *
 * Default 0. Predictions + falsifiers were registered BEFORE this code existed,
 * in decoder/h264/design/mt8365-n132-pwtstop-prediction.md (claude tree).
 */
#define PWTSTOP_P	0x1	/* park at pwt_start on P/SP weighted slices */
#define PWTSTOP_B	0x2	/* ... and on weighted_bipred_idc==1 B slices */
#define PWTSTOP_QPD	0x4	/* mimic the vendor's desynced slice_qp_delta */

static int h264_pwtstop;
module_param(h264_pwtstop, int, 0644);
MODULE_PARM_DESC(h264_pwtstop,
		 "mt8365 H.264 N132 pred_weight_table PARK bitmask (requires h264_hwsh bit0): 1=stop the slice-header advance at pwt_start on P/SP weighted slices and leave the HW reader parked there, as the vendor's reader is, instead of consuming the table and the header tail, 2=park on weighted_bipred_idc==1 B slices too, 4=also re-program AVCVLD+0x9c[6:0] with the DESYNCED slice_qp_delta a reader at pwt_start decodes (the vendor's own garbage value) (default 0 = advance to header_bit_size, unchanged)");

/*
 * N133 -- `h264_pwtafter`: park the hardware reader AFTER the
 * pred_weight_table() strobe fires, instead of INSTEAD OF it.
 *
 * h264_pwtstop (N132) did two things at once and its falsification cannot
 * separate them: it parked the reader BEFORE the table, and as a consequence
 * it took the AVCVLD+0x230 strobe count from 1 to 0. So "the table bits are
 * denied to the engine" and "the header-TAIL bits are denied to the engine"
 * were varied together.
 *
 * h264_pwtafter varies only the second. At the table it does exactly what the
 * landed h264_hwsh walk does -- fire the +0x230 strobe, let the silicon consume
 * pred_weight_table() for itself, advance the reader to pwt_end -- and THEN
 * stops, consuming none of dec_ref_pic_marking(), slice_qp_delta or the
 * deblocking offsets, and issuing no bulk tail advance.
 *
 * There is deliberately NO slice_qp_delta desync-mimic bit here (pwtstop bit2):
 * at pwt_end the reader is spec-CORRECT, so we keep programming the correct
 * value from the V4L2 controls. A 0x9c[6:0] of 0x7f on a pwtafter leg would
 * mean the arm had turned into pwtstop.
 *
 * This is the last untested cell of the strobe x park 2x2. Predictions and
 * falsifiers were registered BEFORE this code existed, in
 * decoder/h264/design/mt8365-n133-pwtafter-prediction.md (claude tree).
 */
#define PWTAFTER_P	0x1	/* park at pwt_end on P/SP weighted slices */
#define PWTAFTER_B	0x2	/* ... and on weighted_bipred_idc==1 B slices */

static int h264_pwtafter;
module_param(h264_pwtafter, int, 0644);
MODULE_PARM_DESC(h264_pwtafter,
		 "mt8365 H.264 N133 pred_weight_table park-AFTER-the-strobe bitmask (requires h264_hwsh bit0): 1=on P/SP weighted slices fire the AVCVLD+0x230 strobe as usual, advance to pwt_end, then stop -- consume no header tail (dec_ref_pic_marking/slice_qp_delta/deblocking) and issue no bulk advance, 2=same on weighted_bipred_idc==1 B slices (default 0 = advance to header_bit_size, unchanged). h264_pwtstop takes precedence where both are armed");

struct mt8365_hwbr {
	struct mtk_vcodec_dec_ctx *ctx;
	u32 pos;		/* bits consumed since OUTPUT-buffer byte 0 */
};

/*
 * N50: the HARDWARE'S OWN VIEW of the slice header.
 *
 * N49 proved the AVCVLD block is a complete Exp-Golomb bit reader (+0x23c =
 * ue(v), +0x240 = se(v), +(n<<2) = consume-n) and that walking the whole slice
 * header through it lands bit-exactly on header_bit_size, every slice, with the
 * pred_weight_table() values matching ffmpeg's parser character for character.
 * That makes the walk the FIRST channel that can read back what THIS HARDWARE
 * decodes for a syntax element, independently of what our driver programs.
 *
 * Several large exclusions rest on the premise "the value is correct in OUR
 * programming model" -- a premise that channel can now check directly. This
 * struct is the readout: every element the walk consumes that some excluded
 * bucket depends on is CAPTURED (it was previously consumed and discarded), so
 * the caller can print it beside the driver's own effective value and the V4L2
 * uAPI's. Nothing here changes a register write; the capture is inert.
 */
struct mt8365_hwsh_syn {
	u32 first_mb, stype, pps_id, frame_num;
	u32 field_pic, bottom_field;	/* vs DECODE_PARAMS FIELD_PIC/BOTTOM_FIELD */
	u32 direct_spatial;		/* vs SLICE_FLAG_DIRECT_SPATIAL_MV_PRED */
	u32 override;			/* num_ref_idx_active_override_flag */
	u32 num_l0, num_l1;		/* ACTIVE counts (override else PPS default) */
	u32 cabac_init_idc, has_cabac_init;
	u32 l0mod_flag, l1mod_flag, l0mod_n, l1mod_n;
	u32 l0mod[8][2], l1mod[8][2];	/* {idc, value} */
};

/* Bits of the ue(v) codeword that encodes codeNum v (H.264 9.1). */
static u32 mt8365_ue_len(u32 v)
{
	u64 cw = (u64)v + 1;
	u32 lz = 0;

	while (cw > 1) {
		cw >>= 1;
		lz++;
	}
	return 2 * lz + 1;
}

/* u(n), n <= 16 (offset stays clear of the 0x80/0x84 status/kick words). */
static u32 mt8365_hw_u(struct mt8365_hwbr *h, u32 n)
{
	u32 peek;

	if (!n)
		return 0;
	peek = rd(h->ctx, VDEC_AD, AVCVLD_PEEK);
	(void)rd(h->ctx, VDEC_AD, n << 2);
	adv_log(n << 2, h->pos);
	h->pos += n;
	return peek >> (32 - n);
}

static void mt8365_hw_skip(struct mt8365_hwbr *h, u32 n)
{
	while (n) {
		u32 chunk = n < 16 ? n : 16;

		(void)rd(h->ctx, VDEC_AD, chunk << 2);
		adv_log(chunk << 2, h->pos);
		h->pos += chunk;
		n -= chunk;
	}
}

static u32 mt8365_hw_ue(struct mt8365_hwbr *h)
{
	u32 v = rd(h->ctx, VDEC_AD, AVCVLD_UE);

	adv_log(AVCVLD_UE, h->pos);
	h->pos += mt8365_ue_len(v);
	return v;
}

static int mt8365_hw_se(struct mt8365_hwbr *h)
{
	int v = (int)rd(h->ctx, VDEC_AD, AVCVLD_SE);
	u32 cn = (v > 0) ? (u32)(2 * v - 1) : (u32)(-2 * v);

	adv_log(AVCVLD_SE, h->pos);
	h->pos += mt8365_ue_len(cn);
	return v;
}

/* The vendor's H264_HAL_PredWeightTable @0x49d80, verbatim. */
static void mt8365_hw_pwt_strobe(struct mtk_vcodec_dec_ctx *ctx)
{
	u32 saved = rd(ctx, VDEC_MISC, 0xf0);
	u32 j;

	wr(ctx, VDEC_AD, 0x230, 0x00000001);
	for (j = 0; j < 100000; j++) {
		if (rd(ctx, VDEC_AD, 0x230))
			break;
		cpu_relax();
	}
	if (j == 100000)
		mtk_v4l2_vdec_err(ctx, "mt8365 hwsh: AVCVLD+0x230 stuck");
	wr(ctx, VDEC_AD, 0xf0, saved & ~1u);
}

/*
 * Walk the slice header through the HW bit reader, in parseSliceHeaderHW's
 * order, leaving the barrel shifter exactly at slice_data(). Returns 0 and
 * *out_pos = the bit position reached; the caller cross-checks it against
 * header_bit_size (the accounting check -- a desync of the HW parse against our
 * software parse shows up there immediately, and the software parse is already
 * verified slice-for-slice against ffmpeg's, 0 mismatches on 1114 slices).
 */
static int mt8365_hwsh_walk(struct mtk_vcodec_dec_ctx *ctx,
			    const struct v4l2_ctrl_h264_sps *sps,
			    const struct v4l2_ctrl_h264_pps *pps,
			    const u8 *buf, u32 buf_sz,
			    u32 sw_pwt_start, u32 sw_pwt_end,
			    u32 *out_pos, bool *out_fired,
			    bool *out_parked, int *out_desync_qpd,
			    struct mt8365_hwsh_syn *syn)
{
	struct mt8365_hwbr h = { .ctx = ctx, .pos = 0 };
	u32 i, nal_off = 0, first_mb, pps_id, st, num_l0, num_l1;
	u32 field_pic = 0;
	int nal_type = -1, nal_ref_idc = 0;
	bool is_idr, cabac, log = !!(h264_hwsh & HWSH_LOG);

	*out_fired = false;
	*out_parked = false;
	*out_desync_qpd = 0;
	memset(syn, 0, sizeof(*syn));
	if (!buf || buf_sz < 5)
		return -EINVAL;
	for (i = 0; i + 3 <= buf_sz;) {
		if (buf[i] == 0 && buf[i + 1] == 0 && buf[i + 2] == 1) {
			u32 hh = i + 3;

			if (hh < buf_sz) {
				int t = buf[hh] & 0x1f;

				if (t == NAL_NON_IDR_SLICE || t == NAL_IDR_SLICE) {
					nal_off = hh;
					nal_type = t;
					nal_ref_idc = (buf[hh] >> 5) & 3;
					break;
				}
			}
			i += 3;
		} else {
			i++;
		}
	}
	if (nal_type < 0)
		return -EINVAL;
	is_idr = (nal_type == NAL_IDR_SLICE);
	cabac = !!(pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE);

	/* start code + NAL header: not syntax, just advance the reader. */
	mt8365_hw_skip(&h, nal_off * 8 + 8);

	first_mb = mt8365_hw_ue(&h);
	st = mt8365_hw_ue(&h) % 5;
	pps_id = mt8365_hw_ue(&h);
	syn->first_mb = first_mb;
	syn->stype = st;
	syn->pps_id = pps_id;
	if (log)
		pr_info("H264HWSH first_mb=%u stype=%u ppsid=%u pos=%u\n",
			first_mb, st, pps_id, h.pos);
	if (sps->flags & V4L2_H264_SPS_FLAG_SEPARATE_COLOUR_PLANE)
		mt8365_hw_u(&h, 2);
	syn->frame_num = mt8365_hw_u(&h, sps->log2_max_frame_num_minus4 + 4);
	if (!(sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY)) {
		field_pic = mt8365_hw_u(&h, 1);
		syn->field_pic = field_pic;
		if (field_pic)
			syn->bottom_field = mt8365_hw_u(&h, 1);	/* bottom_field_flag */
	}
	if (is_idr)
		mt8365_hw_ue(&h);			/* idr_pic_id */
	if (sps->pic_order_cnt_type == 0) {
		mt8365_hw_u(&h, sps->log2_max_pic_order_cnt_lsb_minus4 + 4);
		if ((pps->flags & V4L2_H264_PPS_FLAG_BOTTOM_FIELD_PIC_ORDER_IN_FRAME_PRESENT) &&
		    !field_pic)
			mt8365_hw_se(&h);
	} else if (sps->pic_order_cnt_type == 1 &&
		   !(sps->flags & V4L2_H264_SPS_FLAG_DELTA_PIC_ORDER_ALWAYS_ZERO)) {
		mt8365_hw_se(&h);
		if ((pps->flags & V4L2_H264_PPS_FLAG_BOTTOM_FIELD_PIC_ORDER_IN_FRAME_PRESENT) &&
		    !field_pic)
			mt8365_hw_se(&h);
	}
	if (pps->flags & V4L2_H264_PPS_FLAG_REDUNDANT_PIC_CNT_PRESENT)
		mt8365_hw_ue(&h);
	if (st == 1)
		syn->direct_spatial = mt8365_hw_u(&h, 1); /* direct_spatial_mv_pred_flag */
	num_l0 = pps->num_ref_idx_l0_default_active_minus1 + 1;
	num_l1 = pps->num_ref_idx_l1_default_active_minus1 + 1;
	if (st == 0 || st == 1 || st == 3) {
		if (mt8365_hw_u(&h, 1)) {
			syn->override = 1;
			num_l0 = mt8365_hw_ue(&h) + 1;
			if (st == 1)
				num_l1 = mt8365_hw_ue(&h) + 1;
		}
	}
	syn->num_l0 = num_l0;
	syn->num_l1 = (st == 1) ? num_l1 : 0;
	if (st != 2 && st != 4) {			/* ref_pic_list_modification l0 */
		if (mt8365_hw_u(&h, 1)) {
			syn->l0mod_flag = 1;
			for (i = 0; i < 64; i++) {
				u32 idc = mt8365_hw_ue(&h), val;

				if (idc > 2)
					break;
				val = mt8365_hw_ue(&h);
				if (syn->l0mod_n < ARRAY_SIZE(syn->l0mod)) {
					syn->l0mod[syn->l0mod_n][0] = idc;
					syn->l0mod[syn->l0mod_n][1] = val;
				}
				syn->l0mod_n++;
			}
		}
	}
	if (st == 1) {					/* l1 */
		if (mt8365_hw_u(&h, 1)) {
			syn->l1mod_flag = 1;
			for (i = 0; i < 64; i++) {
				u32 idc = mt8365_hw_ue(&h), val;

				if (idc > 2)
					break;
				val = mt8365_hw_ue(&h);
				if (syn->l1mod_n < ARRAY_SIZE(syn->l1mod)) {
					syn->l1mod[syn->l1mod_n][0] = idc;
					syn->l1mod[syn->l1mod_n][1] = val;
				}
				syn->l1mod_n++;
			}
		}
	}
	/* pred_weight_table() -- the vendor's H264_HAL_PredWeightTable call site. */
	if (((pps->flags & V4L2_H264_PPS_FLAG_WEIGHTED_PRED) && (st == 0 || st == 3)) ||
	    (pps->weighted_bipred_idc == 1 && st == 1)) {
		bool chroma = sps->chroma_format_idc != 0;

		if (sw_pwt_end > sw_pwt_start && h.pos != sw_pwt_start) {
			mtk_v4l2_vdec_err(ctx,
				"mt8365 hwsh: DESYNC at pred_weight_table, HW pos=%u sw=%u",
				h.pos, sw_pwt_start);
			return -EIO;
		}
		/*
		 * N132 PWTSTOP -- reproduce the VENDOR's parked reader.
		 *
		 * Everything below this point in the header (the table itself,
		 * dec_ref_pic_marking(), slice_qp_delta, the deblocking offsets)
		 * is what the vendor's reader NEVER consumes. Return here and the
		 * caller consumes nothing more, so the shifter is left exactly
		 * where the vendor's is when DecStart fires. No +0x230 strobe is
		 * fired on the parked path -- the vendor does not fire it either.
		 */
		if (((h264_pwtstop & PWTSTOP_P) && (st == 0 || st == 3)) ||
		    ((h264_pwtstop & PWTSTOP_B) && st == 1)) {
			if (h264_pwtstop & PWTSTOP_QPD) {
				/*
				 * The vendor's own desynced continuation, read
				 * through the SAME hardware Exp-Golomb decoders
				 * it uses: the u(1) it believes is
				 * adaptive_ref_pic_marking_flag and the se(v) it
				 * believes is slice_qp_delta, both landing inside
				 * pred_weight_table(). N131 §2.3 reproduced the
				 * vendor's captured AVCVLD+0x9c writes from
				 * exactly this pair, 7 of 7.
				 */
				if (nal_ref_idc != 0 && !is_idr)
					mt8365_hw_u(&h, 1);
				*out_desync_qpd = mt8365_hw_se(&h);
			}
			*out_parked = true;
			*out_pos = h.pos;
			if (log)
				pr_info("H264PWTSTOP parked st=%u pos=%u pwt=%u..%u qpd=%d\n",
					st, h.pos, sw_pwt_start, sw_pwt_end,
					*out_desync_qpd);
			return 0;
		}
		if (h264_hwsh & HWSH_PWT_READ) {
			u32 denom_l = mt8365_hw_ue(&h), denom_c = 0;
			u32 cnts[2] = { num_l0, (st == 1) ? num_l1 : 0 };
			int l, k;

			if (chroma)
				denom_c = mt8365_hw_ue(&h);
			if (log)
				pr_info("H264HWSH pwt denom l=%u c=%u nl0=%u\n",
					denom_l, denom_c, num_l0);
			for (l = 0; l < 2; l++) {
				for (k = 0; k < (int)cnts[l] && k < 32; k++) {
					int w = 0, o = 0;

					if (mt8365_hw_u(&h, 1)) {
						w = mt8365_hw_se(&h);
						o = mt8365_hw_se(&h);
					}
					if (log)
						pr_info("H264HWSH pwt L%d[%d] luma w=%d o=%d\n",
							l, k, w, o);
					if (chroma && mt8365_hw_u(&h, 1)) {
						int cw0 = mt8365_hw_se(&h);
						int co0 = mt8365_hw_se(&h);
						int cw1 = mt8365_hw_se(&h);
						int co1 = mt8365_hw_se(&h);

						if (log)
							pr_info("H264HWSH pwt L%d[%d] cb w=%d o=%d cr w=%d o=%d\n",
								l, k, cw0, co0, cw1, co1);
					}
				}
			}
		} else {
			mt8365_hw_pwt_strobe(ctx);
			*out_fired = true;
			if (sw_pwt_end > sw_pwt_start)
				h.pos = sw_pwt_end;
			/*
			 * N133 PWTAFTER -- park HERE, one step later than
			 * pwtstop does. The strobe has fired and the silicon
			 * has consumed pred_weight_table() for itself; what we
			 * now decline to consume is only the header TAIL. The
			 * shifter is left at pwt_end, which is a spec-correct
			 * position, so slice_qp_delta stays the correct value
			 * we already programmed from the V4L2 controls.
			 */
			if (((h264_pwtafter & PWTAFTER_P) && (st == 0 || st == 3)) ||
			    ((h264_pwtafter & PWTAFTER_B) && st == 1)) {
				*out_parked = true;
				*out_pos = h.pos;
				if (log)
					pr_info("H264PWTAFTER parked st=%u pos=%u pwt=%u..%u\n",
						st, h.pos, sw_pwt_start, sw_pwt_end);
				return 0;
			}
		}
		if (sw_pwt_end > sw_pwt_start && h.pos != sw_pwt_end) {
			mtk_v4l2_vdec_err(ctx,
				"mt8365 hwsh: pwt LENGTH mismatch, HW pos=%u sw_end=%u",
				h.pos, sw_pwt_end);
			return -EIO;
		}
	}
	/* dec_ref_pic_marking() */
	if (nal_ref_idc != 0) {
		if (is_idr) {
			mt8365_hw_u(&h, 1);
			mt8365_hw_u(&h, 1);
		} else if (mt8365_hw_u(&h, 1)) {
			for (i = 0; i < 64; i++) {
				u32 op = mt8365_hw_ue(&h);

				if (op == 0 || op > 6)
					break;
				if (op == 1 || op == 3)
					mt8365_hw_ue(&h);
				if (op == 2)
					mt8365_hw_ue(&h);
				if (op == 3 || op == 6)
					mt8365_hw_ue(&h);
				if (op == 4)
					mt8365_hw_ue(&h);
			}
		}
	}
	if (cabac && st != 2 && st != 4) {
		syn->cabac_init_idc = mt8365_hw_ue(&h);	/* cabac_init_idc */
		syn->has_cabac_init = 1;
	}
	mt8365_hw_se(&h);				/* slice_qp_delta */
	if (pps->flags & V4L2_H264_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT) {
		u32 idc = mt8365_hw_ue(&h);

		if (idc != 1) {
			mt8365_hw_se(&h);
			mt8365_hw_se(&h);
		}
	}
	*out_pos = h.pos;
	return 0;
}

/*
 * N47: the SHORT-TERM PicNum of a DPB entry, derived per H.264 Sec 8.2.4.1 from
 * the untruncated `frame_num` syntax element rather than read out of the
 * 16-bit-truncated `pic_num` field (see the h264_picnum comment):
 *   FrameNumWrap = (FrameNum > frame_num) ? FrameNum - MaxFrameNum : FrameNum
 * and PicNum == FrameNumWrap for a frame-coded picture. Identical to the old
 * `pic_num` read for every entry that has not wrapped, so a stream whose
 * frame_num never exceeds the current one is byte-for-byte unchanged.
 * Long-term entries are NOT routed through this -- their pic_num carries
 * LongTermPicNum, which is small and non-negative.
 */
static int mt8365_st_picnum(const struct slice_h264_dpb_entry *e,
			    u32 cur_frame_num, u32 max_frame_num)
{
	int fn = (int)e->frame_num;

	return (fn > (int)cur_frame_num) ? fn - (int)max_frame_num : fn;
}

/*
 * Apply ref_pic_list_modification (l0, short-term idc 0/1) to the default L0
 * list @refidx (DPB indices, already in default P-List0 order). Reorders in
 * place per H.264 Sec 8.2.4.3.1. Long-term ops (idc 2) are out of scope (kept in
 * default position). @dpb supplies each entry's PicNum.
 */
static void mt8365_reorder_l0(int *refidx, int n_avail,
			      const struct slice_h264_dpb_entry *dpb,
			      u32 cur_frame_num, u32 max_frame_num,
			      const struct mt8365_rplm_op *ops, int nops)
{
	int cur_pic_num = (int)cur_frame_num;
	int max_pic_num = (int)max_frame_num;
	int pic_num_pred = cur_pic_num;
	int refpos = 0, k;

	if (max_pic_num <= 0)
		return;
	for (k = 0; k < nops && refpos < n_avail; k++) {
		int pic_num, target, found = -1, i;

		/*
		 * N27: modification_of_pic_nums_idc == 2 is the LONG-TERM reorder op
		 * (§8.2.4.3.2): move the long-term entry whose LongTermPicNum equals
		 * long_term_pic_num to refpos. It does NOT update picNumLXPred (the
		 * short-term predictor), so it is handled entirely here.
		 */
		if (ops[k].idc == 2) {
			for (i = refpos; i < n_avail; i++)
				if ((dpb[refidx[i]].flags &
				     V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM) &&
				    dpb[refidx[i]].pic_num == ops[k].val) {
					found = i;
					break;
				}
			if (found < 0)
				continue;
			{
				int save = refidx[found];

				for (i = found; i > refpos; i--)
					refidx[i] = refidx[i - 1];
				refidx[refpos] = save;
			}
			refpos++;
			continue;
		}
		if (ops[k].idc != 0 && ops[k].idc != 1)
			continue;		/* malformed */
		if (ops[k].idc == 0) {
			pic_num = pic_num_pred - (int)(ops[k].val + 1);
			if (pic_num < 0)
				pic_num += max_pic_num;
		} else {
			pic_num = pic_num_pred + (int)(ops[k].val + 1);
			if (pic_num >= max_pic_num)
				pic_num -= max_pic_num;
		}
		pic_num_pred = pic_num;
		target = pic_num > cur_pic_num ? pic_num - max_pic_num : pic_num;
		for (i = refpos; i < n_avail; i++)
			if (!(dpb[refidx[i]].flags &
			      V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM) &&
			    ((h264_picnum & PICNUM_SIGNED)
				? mt8365_st_picnum(&dpb[refidx[i]],
						   cur_frame_num, max_frame_num)
				: (int)dpb[refidx[i]].pic_num) == target) {
				found = i;
				break;
			}
		if (found < 0)
			continue;
		{
			int save = refidx[found];

			for (i = found; i > refpos; i--)
				refidx[i] = refidx[i - 1];
			refidx[refpos] = save;
		}
		refpos++;
	}
}

/*
 * N135 (2026-08-24): the OVER-LENGTH RefPicList0.
 *
 * mt8365_reorder_l0() above is a PERMUTATION of the n_avail available
 * references -- it moves entries inside refidx[0..n_avail) and can never emit a
 * duplicate. That is correct only while num_ref_idx_l0_active <= n_avail. When
 * a slice sends num_ref_idx_active_override_flag with a count that EXCEEDS the
 * available references, H.264 §8.2.4.3.1 fills the surplus indices by
 * re-inserting pictures that are already in the list: its dedup loop scans only
 * `cIdx >= refIdxL0`, so an earlier copy survives, and the same reference ends
 * up at two indices -- each with its OWN pred_weight_table() entry.
 *
 * CVWP5_TOSHIBA_E / CAWP5_TOSHIBA_E are built on exactly that. Their
 * sps.max_num_ref_frames is 5 and their num_ref_idx_l0_active reaches 10, so
 * picture 5's list is [pn4,pn3,pn2,pn1,pn0,pn4,pn3,pn2,pn1,pn0] and 377 of the
 * clip's index pairs carry DIFFERENT weights at the two copies. The now
 * bit-exact CVWP1/CAWP1 pair has 0 such pairs, and CVWP2/CVWP3 have duplicates
 * but never differently-weighted ones -- so this is the single syntax
 * discriminator that separates pass from fail across all five WP vectors
 * (per the internal N135 WP-duplication probe).
 *
 * The N45 comment below states the premise this refutes: "The clamp is right
 * for the ref-list SLOTS (there is no reference picture to put in the extra
 * ones)". There is one, and AVCVLD+0x98[24:20] already tells the hardware to
 * expect it -- measured, h264_wrlog picture 5 of CVWP5_TOSHIBA_E:
 * 0x098 = 0x0090a000 (refcnt = 9, i.e. ten active references) against a group A
 * holding only ten words = FIVE entries x two parity words.
 *
 * Builds RefPicList0 to its full declared length @nl0 in @idx, by the letter of
 * §8.2.4.3.1 (temporary list of nl0 + 1 entries; shift up, insert, then
 * compact from refIdxL0). Returns the number of entries produced, or 0 if the
 * process could not fill every index (a non-conforming stream), in which case
 * the caller keeps the pre-N135 clamp.
 *
 * Gated by h264_l0dup and only reached when nl0 > n_avail, so every slice the
 * driver already handles is byte-identical by construction.
 */
static int mt8365_build_l0_overlong(int *idx, int n_avail,
				    const struct slice_h264_dpb_entry *dpb,
				    u32 cur_frame_num, u32 max_frame_num,
				    const struct mt8365_rplm_op *ops, int nops,
				    int nl0)
{
	int tmp[V4L2_H264_NUM_DPB_ENTRIES + 1];
	int cur_pic_num = (int)cur_frame_num;
	int max_pic_num = (int)max_frame_num;
	int pic_num_pred = cur_pic_num;
	int refpos = 0, k, c, i;

	if (max_pic_num <= 0 || nl0 <= 0 || nops <= 0 ||
	    nl0 + 1 > (int)ARRAY_SIZE(tmp))
		return 0;

	for (c = 0; c <= nl0; c++)
		tmp[c] = (c < n_avail) ? idx[c] : -1;

	for (k = 0; k < nops && refpos < nl0; k++) {
		int pic_num, target = -1, nidx;

		if (ops[k].idc == 2) {
			for (i = 0; i < V4L2_H264_NUM_DPB_ENTRIES; i++) {
				u32 fl = dpb[i].flags;

				if ((fl & V4L2_H264_DPB_ENTRY_FLAG_ACTIVE) &&
				    (fl & V4L2_H264_DPB_ENTRY_FLAG_VALID) &&
				    (fl & V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM) &&
				    dpb[i].pic_num == ops[k].val) {
					target = i;
					break;
				}
			}
		} else if (ops[k].idc == 0 || ops[k].idc == 1) {
			int want;

			if (ops[k].idc == 0) {
				pic_num = pic_num_pred - (int)(ops[k].val + 1);
				if (pic_num < 0)
					pic_num += max_pic_num;
			} else {
				pic_num = pic_num_pred + (int)(ops[k].val + 1);
				if (pic_num >= max_pic_num)
					pic_num -= max_pic_num;
			}
			pic_num_pred = pic_num;
			want = pic_num > cur_pic_num
				? pic_num - max_pic_num : pic_num;
			for (i = 0; i < V4L2_H264_NUM_DPB_ENTRIES; i++) {
				u32 fl = dpb[i].flags;

				if (!(fl & V4L2_H264_DPB_ENTRY_FLAG_ACTIVE) ||
				    !(fl & V4L2_H264_DPB_ENTRY_FLAG_VALID) ||
				    (fl & V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM))
					continue;
				if (((h264_picnum & PICNUM_SIGNED)
				     ? mt8365_st_picnum(&dpb[i], cur_frame_num,
							max_frame_num)
				     : (int)dpb[i].pic_num) == want) {
					target = i;
					break;
				}
			}
		} else {
			continue;		/* malformed */
		}
		if (target < 0)
			continue;
		for (c = nl0; c > refpos; c--)
			tmp[c] = tmp[c - 1];
		tmp[refpos++] = target;
		nidx = refpos;
		for (c = refpos; c <= nl0; c++)
			if (tmp[c] != target && tmp[c] >= 0)
				tmp[nidx++] = tmp[c];
		while (nidx <= nl0)
			tmp[nidx++] = -1;
	}

	for (c = 0; c < nl0; c++)
		if (tmp[c] < 0)
			return 0;		/* could not fill every index */
	for (c = 0; c < nl0; c++)
		idx[c] = tmp[c];
	return nl0;
}

/*
 * Build the INITIAL RefPicList0 for a P/SP slice from the marshalled DPB, per
 * H.264 §8.2.4.2.1: short-term references FIRST in DESCENDING PicNum order,
 * then long-term references in ASCENDING LongTermPicNum order. The V4L2
 * stateless uAPI puts LongTermPicNum (8-29) in dpb[].pic_num for entries
 * flagged LONG_TERM (ext-ctrls-codec-stateless.rst), so one field serves both.
 *
 * N27: @with_lt == false reproduces the pre-round list byte-for-byte (long-term
 * entries dropped entirely), which is the A arm of the module-param A/B.
 * Returns the entry count.
 */
static int mt8365_build_init_p_list(const struct slice_h264_dpb_entry *dpb,
				    int *idx, bool with_lt,
				    u32 cur_frame_num, u32 max_frame_num)
{
	int n = 0, a, b;

	for (a = 0; a < V4L2_H264_NUM_DPB_ENTRIES; a++) {
		u32 fl = dpb[a].flags;

		if (!(fl & V4L2_H264_DPB_ENTRY_FLAG_ACTIVE) ||
		    !(fl & V4L2_H264_DPB_ENTRY_FLAG_VALID))
			continue;
		if ((fl & V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM) && !with_lt)
			continue;
		idx[n++] = a;
	}
	for (a = 0; a < n; a++)
		for (b = a + 1; b < n; b++) {
			bool lta = !!(dpb[idx[a]].flags &
				      V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM);
			bool ltb = !!(dpb[idx[b]].flags &
				      V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM);
			bool swap;

			if (lta != ltb)
				swap = lta;	/* short-term entries first */
			else if (lta)		/* long-term: ascending LongTermPicNum */
				swap = dpb[idx[b]].pic_num < dpb[idx[a]].pic_num;
			else if (h264_picnum & PICNUM_SIGNED)
				/*
				 * short-term: descending PicNum = FrameNumWrap,
				 * signed and derived from frame_num (N47).
				 */
				swap = mt8365_st_picnum(&dpb[idx[b]],
							cur_frame_num,
							max_frame_num) >
				       mt8365_st_picnum(&dpb[idx[a]],
							cur_frame_num,
							max_frame_num);
			else			/* short-term: descending PicNum */
				swap = dpb[idx[b]].pic_num > dpb[idx[a]].pic_num;
			if (swap) {
				int t = idx[a];

				idx[a] = idx[b];
				idx[b] = t;
			}
		}
	return n;
}

/*
 * N51: build the initial FIELD RefPicList0 for a P/SP FIELD picture, per
 * H.264 Sec 8.2.4.2.2 + Sec 8.2.4.2.5.
 *
 *  - Sec 8.2.4.2.2 orders the short-term reference FRAMES by descending
 *    FrameNumWrap (refFrameList0ShortTerm), then the long-term reference frames
 *    by ascending LongTermFrameIdx.
 *  - Sec 8.2.4.2.5 then derives the FIELD list from that frame list: fields are
 *    taken alternately, starting with a field of the SAME parity as the current
 *    field, each parity walking the ordered frame list independently and
 *    skipping frames that have no reference-marked field of that parity. When
 *    one parity runs out, the remaining fields of the other parity follow in
 *    order.
 *
 * The result is up to 2 entries per DPB frame, which is exactly the factor-2
 * the hardware's own num_ref_idx_l0_active reading shows against this driver's
 * frame-based list.
 *
 * @out_di/@out_bot receive the DPB index and the field parity of each list
 * entry; @out_pn receives its FIELD PicNum (Sec 8.2.4.1: 2*FrameNumWrap + 1 for
 * a field of the same parity as the current field, 2*FrameNumWrap otherwise;
 * long-term entries keep the uAPI-supplied LongTermPicNum). Returns the entry
 * count.
 */
/*
 * N51/N54: the Sec 8.2.4.2.5 frame->field stage on its own, so that BOTH the P
 * ordering (Sec 8.2.4.2.2 / 8.2.4.2.5, descending FrameNumWrap) and the B
 * ordering (Sec 8.2.4.2.4, POC-based, two lists) can feed the SAME interleave.
 *
 * @fr / @fnw are an ALREADY-ORDERED reference-FRAME list and, per frame, its
 * FrameNumWrap (short-term) or LongTermFrameIdx (long-term). Fields are taken
 * alternately starting with the parity of the current field, each parity
 * walking @fr independently and skipping frames with no reference-marked field
 * of that parity; when one parity runs out the rest of the other parity
 * follows. Entries are APPENDED starting at @n; the new count is returned.
 *
 * @out_pn is the FIELD PicNum of Sec 8.2.4.1: 2*fnw + 1 for a field of the same
 * parity as the current field, 2*fnw for the opposite parity.
 */
/* the P path's callers pass 32-entry arrays; a B field programs at most 16 */
#define MT8365_FIELD_LIST_MAX	32
#define MT8365_BFIELD_MAX	16

static int mt8365_field_interleave(const struct v4l2_h264_dpb_entry *dpb,
				   const int *fr, const int *fnw, int nf,
				   int cur_bottom, u8 *out_di, u8 *out_bot,
				   int *out_pn, int n, int max)
{
	int sd[V4L2_H264_NUM_DPB_ENTRIES], od[V4L2_H264_NUM_DPB_ENTRIES];
	int ns = 0, no = 0, si = 0, oi = 0, a;
	u8 same_mask = cur_bottom ? V4L2_H264_BOTTOM_FIELD_REF
				  : V4L2_H264_TOP_FIELD_REF;
	u8 opp_mask = cur_bottom ? V4L2_H264_TOP_FIELD_REF
				 : V4L2_H264_BOTTOM_FIELD_REF;

	for (a = 0; a < nf; a++) {
		if (dpb[fr[a]].fields & same_mask)
			sd[ns++] = a;
		if (dpb[fr[a]].fields & opp_mask)
			od[no++] = a;
	}
	while ((si < ns || oi < no) && n < max) {
		if (si < ns) {
			int f = sd[si++];

			out_di[n] = (u8)fr[f];
			out_bot[n] = (u8)cur_bottom;
			out_pn[n] = 2 * fnw[f] + 1;
			n++;
		}
		if (oi < no && n < max) {
			int f = od[oi++];

			out_di[n] = (u8)fr[f];
			out_bot[n] = (u8)!cur_bottom;
			out_pn[n] = 2 * fnw[f];
			n++;
		}
	}
	return n;
}

static int mt8365_build_field_p_list(const struct v4l2_h264_dpb_entry *dpb,
				     u32 cur_frame_num, u32 max_frame_num,
				     int cur_bottom, bool with_lt,
				     u8 *out_di, u8 *out_bot, int *out_pn,
				     int max)
{
	int fr[V4L2_H264_NUM_DPB_ENTRIES];	/* ordered reference FRAMES */
	int fnw[V4L2_H264_NUM_DPB_ENTRIES];	/* their FrameNumWrap / LTidx */
	int nf = 0, a, b, n = 0;
	u8 same_mask = cur_bottom ? V4L2_H264_BOTTOM_FIELD_REF
				  : V4L2_H264_TOP_FIELD_REF;
	u8 opp_mask = cur_bottom ? V4L2_H264_TOP_FIELD_REF
				 : V4L2_H264_BOTTOM_FIELD_REF;

	/* short-term frames, descending FrameNumWrap */
	for (a = 0; a < V4L2_H264_NUM_DPB_ENTRIES; a++) {
		u32 fl = dpb[a].flags;
		int w;

		if (!(fl & V4L2_H264_DPB_ENTRY_FLAG_ACTIVE) ||
		    !(fl & V4L2_H264_DPB_ENTRY_FLAG_VALID))
			continue;
		if (fl & V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM)
			continue;
		if (!(dpb[a].fields & (same_mask | opp_mask)))
			continue;
		w = (int)dpb[a].frame_num;
		if (w > (int)cur_frame_num)
			w -= (int)max_frame_num;
		fr[nf] = a;
		fnw[nf] = w;
		nf++;
	}
	for (a = 0; a < nf; a++)
		for (b = a + 1; b < nf; b++)
			if (fnw[b] > fnw[a]) {
				int t = fr[a];

				fr[a] = fr[b]; fr[b] = t;
				t = fnw[a]; fnw[a] = fnw[b]; fnw[b] = t;
			}
	/* the two per-parity walks over the ordered frame list */
	n = mt8365_field_interleave(dpb, fr, fnw, nf, cur_bottom,
				    out_di, out_bot, out_pn, n, max);
	if (!with_lt)
		return n;

	/* long-term frames, ascending LongTermFrameIdx (== LongTermPicNum >> 1) */
	nf = 0;
	for (a = 0; a < V4L2_H264_NUM_DPB_ENTRIES; a++) {
		u32 fl = dpb[a].flags;

		if (!(fl & V4L2_H264_DPB_ENTRY_FLAG_ACTIVE) ||
		    !(fl & V4L2_H264_DPB_ENTRY_FLAG_VALID))
			continue;
		if (!(fl & V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM))
			continue;
		if (!(dpb[a].fields & (same_mask | opp_mask)))
			continue;
		fr[nf] = a;
		fnw[nf] = (int)dpb[a].pic_num >> 1;
		nf++;
	}
	for (a = 0; a < nf; a++)
		for (b = a + 1; b < nf; b++)
			if (fnw[b] < fnw[a]) {
				int t = fr[a];

				fr[a] = fr[b]; fr[b] = t;
				t = fnw[a]; fnw[a] = fnw[b]; fnw[b] = t;
			}
	return mt8365_field_interleave(dpb, fr, fnw, nf, cur_bottom,
				       out_di, out_bot, out_pn, n, max);
}

/*
 * N104: apply ref_pic_list_modification (Sec 8.2.4.3.1) to a FIELD RefPicList0,
 * IN FIELD UNITS.
 *
 * The frame-unit mt8365_reorder_l0() above cannot serve a field picture: when
 * field_pic_flag is 1 the modification process runs on FIELD PicNums, with
 * MaxPicNum = 2*MaxFrameNum and CurrPicNum = 2*frame_num + 1 (Sec 8.2.4.1), and
 * a short-term list entry's PicNum is the one the Sec 8.2.4.2.5 interleave
 * already assigned (2*FrameNumWrap + 1 same parity, 2*FrameNumWrap opposite).
 * @pn therefore IS the match key -- no re-derivation from the DPB entry, which
 * carries no parity.
 *
 * Long-term ops (idc 2) match on LongTermPicNum, which for a field is
 * 2*LongTermFrameIdx + 1 / 2*LongTermFrameIdx -- exactly what the interleave put
 * in @pn for a long-term entry. Per Sec 8.2.4.3.1 a long-term op does NOT update
 * picNumLXPred.
 *
 * @di/@bot/@pn are reordered in place; @n is the field-list length.
 */
static void mt8365_reorder_field_l0(u8 *di, u8 *bot, int *pn, int n,
				    const struct v4l2_h264_dpb_entry *dpb,
				    u32 cur_frame_num, u32 max_frame_num,
				    const struct mt8365_rplm_op *ops, int nops)
{
	int max_pic_num = 2 * (int)max_frame_num;
	int cur_pic_num = 2 * (int)cur_frame_num + 1;
	int pic_num_pred = cur_pic_num;
	int refpos = 0, k, i;

	if (max_pic_num <= 0)
		return;
	for (k = 0; k < nops && refpos < n; k++) {
		int pic_num, target, found = -1;
		bool want_lt = (ops[k].idc == 2);

		if (want_lt) {
			target = (int)ops[k].val;
		} else {
			if (ops[k].idc != 0 && ops[k].idc != 1)
				continue;	/* malformed */
			if (ops[k].idc == 0) {
				pic_num = pic_num_pred - (int)(ops[k].val + 1);
				if (pic_num < 0)
					pic_num += max_pic_num;
			} else {
				pic_num = pic_num_pred + (int)(ops[k].val + 1);
				if (pic_num >= max_pic_num)
					pic_num -= max_pic_num;
			}
			pic_num_pred = pic_num;
			target = pic_num > cur_pic_num
				? pic_num - max_pic_num : pic_num;
		}
		for (i = refpos; i < n; i++) {
			bool lt = !!(dpb[di[i]].flags &
				     V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM);

			if (lt == want_lt && pn[i] == target) {
				found = i;
				break;
			}
		}
		if (found < 0)
			continue;
		{
			u8 sd = di[found], sb = bot[found];
			int sp = pn[found];

			for (i = found; i > refpos; i--) {
				di[i] = di[i - 1];
				bot[i] = bot[i - 1];
				pn[i] = pn[i - 1];
			}
			di[refpos] = sd;
			bot[refpos] = sb;
			pn[refpos] = sp;
		}
		refpos++;
	}
}

/*
 * N54: the FIELD RefPicList0 + RefPicList1 of a B FIELD picture, per H.264
 * Sec 8.2.4.2.4 (initial ordering) + Sec 8.2.4.2.5 (frame -> field).
 *
 * Sec 8.2.4.2.4, for a field, orders the short-term reference FRAMES by
 * PicOrderCnt relative to the current field:
 *   refFrameList0ShortTerm = { POC <= curPOC, descending } then
 *                            { POC >  curPOC, ascending  }
 *   refFrameList1ShortTerm = { POC >  curPOC, ascending  } then
 *                            { POC <= curPOC, descending }
 * (the "<=" rather than "<" is the field form: the complementary field of the
 * current frame can already be a reference and carries the same frame POC).
 * refFrameListLongTerm is ascending LongTermFrameIdx and is the SAME for both
 * lists. Each ordered frame list is then run through the Sec 8.2.4.2.5 parity
 * interleave (mt8365_field_interleave, shared with the P path). Finally, when
 * RefPicList1 has more than one entry and is identical to RefPicList0, its
 * first two entries are swapped.
 *
 * A frame's PicOrderCnt is Min(Top,Bottom) when both fields are reference-
 * marked, else the POC of the field that is.
 *
 * Returns the L0 entry count and stores the L1 count in @n1.
 */
static int mt8365_build_field_b_lists(const struct v4l2_h264_dpb_entry *dpb,
				      u32 cur_frame_num, u32 max_frame_num,
				      int cur_bottom, int cur_poc, bool with_lt,
				      u8 *l0_di, u8 *l0_bot, int *l0_pn,
				      u8 *l1_di, u8 *l1_bot, int *l1_pn, int *n1)
{
	int fr0[V4L2_H264_NUM_DPB_ENTRIES], fr1[V4L2_H264_NUM_DPB_ENTRIES];
	int fw0[V4L2_H264_NUM_DPB_ENTRIES], fw1[V4L2_H264_NUM_DPB_ENTRIES];
	int idx[V4L2_H264_NUM_DPB_ENTRIES], poc[V4L2_H264_NUM_DPB_ENTRIES];
	int fnw[V4L2_H264_NUM_DPB_ENTRIES];
	int nf = 0, a, b, n0 = 0, nl1 = 0, npast = 0;
	u8 same_mask = cur_bottom ? V4L2_H264_BOTTOM_FIELD_REF
				  : V4L2_H264_TOP_FIELD_REF;
	u8 opp_mask = cur_bottom ? V4L2_H264_TOP_FIELD_REF
				 : V4L2_H264_BOTTOM_FIELD_REF;

	/* the short-term reference frames, with their frame POC + FrameNumWrap */
	for (a = 0; a < V4L2_H264_NUM_DPB_ENTRIES; a++) {
		u32 fl = dpb[a].flags;
		int w, p;

		if (!(fl & V4L2_H264_DPB_ENTRY_FLAG_ACTIVE) ||
		    !(fl & V4L2_H264_DPB_ENTRY_FLAG_VALID))
			continue;
		if (fl & V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM)
			continue;
		if (!(dpb[a].fields & (same_mask | opp_mask)))
			continue;
		w = (int)dpb[a].frame_num;
		if (w > (int)cur_frame_num)
			w -= (int)max_frame_num;
		if ((dpb[a].fields & (V4L2_H264_TOP_FIELD_REF |
				      V4L2_H264_BOTTOM_FIELD_REF)) ==
		    (V4L2_H264_TOP_FIELD_REF | V4L2_H264_BOTTOM_FIELD_REF))
			p = min_t(int, (int)dpb[a].top_field_order_cnt,
				  (int)dpb[a].bottom_field_order_cnt);
		else if (dpb[a].fields & V4L2_H264_TOP_FIELD_REF)
			p = (int)dpb[a].top_field_order_cnt;
		else
			p = (int)dpb[a].bottom_field_order_cnt;
		idx[nf] = a;
		poc[nf] = p;
		fnw[nf] = w;
		nf++;
	}
	/* sort ascending POC once; both orderings are slices of it */
	for (a = 0; a < nf; a++)
		for (b = a + 1; b < nf; b++)
			if (poc[b] < poc[a]) {
				int t = idx[a];

				idx[a] = idx[b]; idx[b] = t;
				t = poc[a]; poc[a] = poc[b]; poc[b] = t;
				t = fnw[a]; fnw[a] = fnw[b]; fnw[b] = t;
			}
	for (a = 0; a < nf; a++)
		if (poc[a] <= cur_poc)
			npast++;
	/* L0 frames: past descending, then future ascending */
	for (a = npast - 1; a >= 0; a--) {
		fr0[n0] = idx[a];
		fw0[n0] = fnw[a];
		n0++;
	}
	for (a = npast; a < nf; a++) {
		fr0[n0] = idx[a];
		fw0[n0] = fnw[a];
		n0++;
	}
	/* L1 frames: future ascending, then past descending */
	for (a = npast; a < nf; a++) {
		fr1[nl1] = idx[a];
		fw1[nl1] = fnw[a];
		nl1++;
	}
	for (a = npast - 1; a >= 0; a--) {
		fr1[nl1] = idx[a];
		fw1[nl1] = fnw[a];
		nl1++;
	}

	n0 = mt8365_field_interleave(dpb, fr0, fw0, n0, cur_bottom,
				     l0_di, l0_bot, l0_pn, 0, MT8365_BFIELD_MAX);
	nl1 = mt8365_field_interleave(dpb, fr1, fw1, nl1, cur_bottom,
				      l1_di, l1_bot, l1_pn, 0, MT8365_BFIELD_MAX);

	if (with_lt) {
		/* long-term frames, ascending LongTermFrameIdx; same for L0/L1 */
		nf = 0;
		for (a = 0; a < V4L2_H264_NUM_DPB_ENTRIES; a++) {
			u32 fl = dpb[a].flags;

			if (!(fl & V4L2_H264_DPB_ENTRY_FLAG_ACTIVE) ||
			    !(fl & V4L2_H264_DPB_ENTRY_FLAG_VALID))
				continue;
			if (!(fl & V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM))
				continue;
			if (!(dpb[a].fields & (same_mask | opp_mask)))
				continue;
			idx[nf] = a;
			fnw[nf] = (int)dpb[a].pic_num >> 1;
			nf++;
		}
		for (a = 0; a < nf; a++)
			for (b = a + 1; b < nf; b++)
				if (fnw[b] < fnw[a]) {
					int t = idx[a];

					idx[a] = idx[b]; idx[b] = t;
					t = fnw[a]; fnw[a] = fnw[b]; fnw[b] = t;
				}
		n0 = mt8365_field_interleave(dpb, idx, fnw, nf, cur_bottom,
					     l0_di, l0_bot, l0_pn, n0,
					     MT8365_BFIELD_MAX);
		nl1 = mt8365_field_interleave(dpb, idx, fnw, nf, cur_bottom,
					      l1_di, l1_bot, l1_pn, nl1,
					      MT8365_BFIELD_MAX);
	}

	/* Sec 8.2.4.2.4: swap L1[0]/L1[1] when L1 is identical to L0 */
	if (nl1 > 1 && nl1 == n0) {
		bool same = true;

		for (a = 0; a < nl1; a++)
			if (l1_di[a] != l0_di[a] || l1_bot[a] != l0_bot[a]) {
				same = false;
				break;
			}
		if (same) {
			u8 td = l1_di[0], tb = l1_bot[0];
			int tp = l1_pn[0];

			l1_di[0] = l1_di[1]; l1_di[1] = td;
			l1_bot[0] = l1_bot[1]; l1_bot[1] = tb;
			l1_pn[0] = l1_pn[1]; l1_pn[1] = tp;
		}
	}
	*n1 = nl1;
	return n0;
}

/*
 * Build the spec RefPicList0 (b0) + RefPicList1 (b1) for a B-slice via the
 * mainline v4l2_h264 reflist builder (spec 8.2.4.2, POC-based, incl. the
 * swap-first-two rule), and return them as DPB-index arrays. Split into its own
 * function so the large reflist_builder + reference arrays live on THIS stack
 * frame, not the decode()'s (which is already near CONFIG_FRAME_WARN). N15.
 */
static noinline void
mt8365_build_b_reflists(const struct v4l2_ctrl_h264_decode_params *dph,
			const struct v4l2_ctrl_h264_sps *sps,
			const struct v4l2_h264_dpb_entry v4l2_dpb[V4L2_H264_NUM_DPB_ENTRIES],
			int l0idx[V4L2_H264_NUM_DPB_ENTRIES],
			int l1idx[V4L2_H264_NUM_DPB_ENTRIES],
			int *num_b0, int *num_b1)
{
	struct v4l2_h264_reflist_builder bld;
	struct v4l2_h264_reference b0[V4L2_H264_REF_LIST_LEN];
	struct v4l2_h264_reference b1[V4L2_H264_REF_LIST_LEN];
	int k, nv;

	*num_b0 = 0;
	*num_b1 = 0;
	v4l2_h264_init_reflist_builder(&bld, dph, sps, v4l2_dpb);
	v4l2_h264_build_b_ref_lists(&bld, b0, b1);
	nv = bld.num_valid;
	if (nv > V4L2_H264_NUM_DPB_ENTRIES)
		nv = V4L2_H264_NUM_DPB_ENTRIES;
	for (k = 0; k < nv; k++) {
		l0idx[(*num_b0)++] = b0[k].index & 0xf;
		l1idx[(*num_b1)++] = b1[k].index & 0xf;
	}
}

static int vdec_h264_mt8365_decode(void *h_vdec, struct mtk_vcodec_mem *bs,
				   struct vdec_fb *unused, bool *res_chg)
{
	struct vdec_h264_mt8365_inst *inst = h_vdec;
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	const struct v4l2_ctrl_h264_sps *sps;
	const struct v4l2_ctrl_h264_pps *pps;
	const struct v4l2_ctrl_h264_slice_params *slp;
	const struct v4l2_ctrl_h264_decode_params *dph;
	struct vdec_fb *fb;
	u32 rec_y, rec_c;
	/* per-frame packed register values, computed from the V4L2 controls
	 * (replaces the IDR trace constants; see the packer block below) */
	u32 r_sps, r_pps90, r_pps94, r_shdr_p, r_9c, r_84, r_top68, r_top70, r_top100;
	/* N89 bit9 ordering control: the scaling matrix to push AFTER 0x94. */
	const struct v4l2_ctrl_h264_scaling_matrix *scaling_deferred = NULL;
	bool scaling_dump = false;
	u32 r_a4;			/* AVCVLD+0x0a4 current-picture PicNum (N39) */
	bool wp_slice = false;		/* N44: explicitly weighted slice (h264_wp gate) */
	u32 mb_w, mb_h, mb_h_pic, mb_h_map;
	int ret, i;
	bool is_inter = false;
	bool sh_from_ctrl;
	int num_l0 = 0;
	/*
	 * N45: the RAW bitstream num_ref_idx_l0_active, BEFORE the clamp to the
	 * number of reference pictures actually in the DPB. The WP5 vectors
	 * (CAWP5/CVWP5) set num_ref_idx_active_override_flag and then activate
	 * MORE L0 entries than the DPB holds (4 entries over 2 refs, 6 over 3,
	 * 8 over 4) -- and send a pred_weight_table() entry for every one of
	 * them. num_l0 is min(raw, available), which is right for the ref-list
	 * slots but may be wrong for whatever consumes AVCVLD 0x98[24:20].
	 */
	int raw_num_l0 = 0;
	bool l0_overlong = false;	/* N135: full-length RefPicList0 built */
	int num_l1 = 0;	/* B: active L1 ref count (slice/PPS) -> AVCVLD 0x98[29:25] */
	u32 eff_hbs = 0, eff_first_mb = 0, eff_stype = 2;	/* 2 = I */
	/* N28: bit window of pred_weight_table() inside the slice header, in the
	 * same OUTPUT-buffer bit space as eff_hbs. 0/0 = not present. */
	u32 pwt_start = 0, pwt_end = 0;
	bool pwt_parked = false;	/* N132: the hwsh walk stopped at pwt_start */
	int pwt_desync_qpd = 0;		/* N132: the vendor's garbage slice_qp_delta */
	int eff_qpd = 0;	/* slice_qp_delta -> AVCVLD+0x9c[6:0] */
	int eff_dbf_idc = 0;	/* disable_deblocking_filter_idc -> 0x9c[8:7] */
	int eff_alpha = 0;	/* slice_alpha_c0_offset_div2  -> 0x9c[12:9] */
	int eff_beta = 0;	/* slice_beta_offset_div2      -> 0x9c[16:13] */
	int eff_direct_spatial = 0;	/* direct_spatial_mv_pred_flag -> 0x98[19] */
	int eff_field_pic = 0;		/* field_pic_flag     -> 0x98[17] (F1, PAFF field geometry) */
	int eff_mbaff = 0;		/* N38: this picture is MBAFF-coded (frame pic in an
					 * mb_adaptive_frame_field_flag stream) */
	int eff_bottom_field = 0;	/* bottom_field_flag  -> 0x98[18] (F1) */
	/*
	 * N50, from the HW read-back channel. Both of these are elements the
	 * hardware's own reader decodes out of the slice header and this driver
	 * did NOT program: cabac_init_idc (N13 was reverted as a corpus-wide
	 * no-op on the two all-zero CABAC vectors, but CABACI3_Sony_B carries
	 * 0/1/2 per slice) and the L1 ref_pic_list_modification list (applied to
	 * L0 only; HCMP1_HHI_A carries 14 non-empty L1 modifications).
	 */
	int eff_cabac_init = 0;			/* -> AVCVLD 0x9c[21:20] */
	struct mt8365_rplm_op ops_l1[34];
	int nops_l1 = 0;
	int refidx[V4L2_H264_NUM_DPB_ENTRIES];
	u32 ref_picnum[16] = { 0 }, ref_poc[16] = { 0 };
	/*
	 * N51: per-list-slot FIELD parity, and the flag that says the L0 slots
	 * below are FIELD entries (Sec 8.2.4.2.5) rather than DPB frames. Zero /
	 * false on every frame-coded picture -> byte-for-byte unchanged.
	 */
	u8 ref_bot[16] = { 0 };
	bool fld_list = false;
	/*
	 * N58: this is a FRAME-coded picture of a PAFF stream and the PAFF_FLDREF
	 * arm is on, so its reference arrays are built in FIELD units through the
	 * same stages a field picture uses. False on every progressive picture,
	 * every MBAFF picture and every field picture.
	 */
	bool paff_frm = false;
	bool fld_rplm = false;		/* N104: field-unit Sec 8.2.4.3.1 */
	/*
	 * N54: the three FIELD-unit reference lists of a B FIELD -- group A
	 * (PicNum-descending, the P builder), group B (RefPicList0) and group C
	 * (RefPicList1). fld_blist is false on every frame-coded picture and on
	 * every I/P field, so those paths are byte-for-byte unchanged.
	 */
	bool fld_blist = false;
	u8 fa_di[MT8365_BFIELD_MAX], fa_bot[MT8365_BFIELD_MAX];
	u8 fb0_di[MT8365_BFIELD_MAX], fb0_bot[MT8365_BFIELD_MAX];
	u8 fb1_di[MT8365_BFIELD_MAX], fb1_bot[MT8365_BFIELD_MAX];
	int fa_pn[MT8365_BFIELD_MAX], fb0_pn[MT8365_BFIELD_MAX];
	int fb1_pn[MT8365_BFIELD_MAX];
	int n_fa = 0, n_fb0 = 0, n_fb1 = 0;
	int cur_slot = 0;
	u32 cur_mv = 0;
	/*
	 * B-slice (bi-prediction) L1 ref-list -- N15 (2026-07-17), from a live
	 * vendor CAVLC-B vs CAVLC-P vpud read-trace on the 13-range DTB. The HW
	 * has THREE parallel ref-list index groups (AVCVLD PicNum + AVCMV
	 * descriptor): A=+0xa8/+0x000, B=+0x128/+0x080, C=+0x1a8/+0x100. The trace
	 * (descriptor low16 = ref POC, decoded per-frame) proved the roles:
	 *   group A = raw PicNum-descending DPB list (refidx),
	 *   group B = spec RefPicList0 (POC-based L0),
	 *   group C = spec RefPicList1 (L1).
	 * For P/SP slices RefPicList0 == PicNum-desc and RefPicList1 ==
	 * swap-first-two(RefPicList0), so groups B/C collapse to the pre-N15
	 * behaviour -- hence P/I output is byte-identical (use_l1 stays false).
	 * For B slices l0idx[]/l1idx[] carry the POC-based lists from the mainline
	 * v4l2_h264_build_b_ref_lists() helper (spec 8.2.4.2).
	 */
	bool use_l1 = false;
	/*
	 * N111 -- MIXED-SLICE-TYPE access unit (h264_mixedslice >= 3).
	 *
	 * `mixed_au` is set when the access unit's VCL NALs do NOT all carry the
	 * same slice_type; `list_stype` is the slice_type whose reference lists
	 * the picture-level register set is programmed for (0xff = not a mixed
	 * AU, i.e. every gate below falls back to eff_stype and the register
	 * writes are byte-for-byte the pre-N111 driver).
	 *
	 * Measured corpus-wide: exactly 2 of 137 JVT-AVC_V1 vectors contain a
	 * mixed-slice-type picture (CABAST3_Sony_E, CABASTBR3_Sony_B), so an arm
	 * gated on mixed_au cannot move any other vector by construction.
	 */
	bool mixed_au = false;
	u32 list_stype = 0xff;
	int l0idx[V4L2_H264_NUM_DPB_ENTRIES], l1idx[V4L2_H264_NUM_DPB_ENTRIES];
	int num_b0 = 0, num_b1 = 0;
	struct v4l2_h264_dpb_entry v4l2_dpb[V4L2_H264_NUM_DPB_ENTRIES];
	/*
	 * B-slice fill-count fix (N15/B2, 2026-07-17). The within-stream vendor
	 * read-trace (trace-b.log) proved the SOLE B-vs-P delta is the ref-list
	 * groups: the vendor fills ALL THREE picnum groups AND the reference-recon
	 * array (MC+0x3dc/0xb60) + colMV table to num_valid (every DPB ref), while
	 * the pre-fix driver filled them to num_l0 (num_ref_idx_l0_active). For a B
	 * frame with active-L0=1 but 2 valid refs, only refidx[0] (the highest
	 * PicNum = the BACKWARD ref) got its recon loaded; the FORWARD ref (L0[0])
	 * recon was never written -> forward bi-pred read garbage (~6 dB). aidx[] is
	 * ALL valid short-term refs in raw PicNum-desc order (= vendor group A);
	 * n_valid is its count. Filled independently of the slice/gst path from
	 * decode_params.dpb so it is correct in both. */
	int aidx[V4L2_H264_NUM_DPB_ENTRIES];
	int n_valid = 0;

	*res_chg = false;

	if (!bs)		/* flush / EOS: nothing to decode */
		return 0;

	/*
	 * Stateless: the core calls vdec_if_decode() with fb==NULL and expects
	 * the backend to pull the destination frame from the CAPTURE queue
	 * itself (as vdec_h264_req_if.c does). Relying on the passed-in fb bailed
	 * here every frame, so the HW was never programmed or triggered.
	 */
	fb = ctx->dev->vdec_pdata->get_cap_buffer(ctx);
	if (!fb)
		return -EBUSY;

	/*
	 * Propagate the OUTPUT (bitstream) buffer's timestamp to this CAPTURE
	 * (recon) buffer -- both queues are V4L2_BUF_FLAG_TIMESTAMP_COPY. This is
	 * what lets a LATER inter frame resolve its references: fill_dpb_info()
	 * does vb2_find_buffer(cap_q, dpb->reference_ts), which matches on the
	 * CAPTURE buffer's timestamp. Without this the reference lookup fails and
	 * every P/B slice decodes with no reference. The sibling H.264 backends
	 * (vdec_h264_req_if.c etc.) do the same copy.
	 */
	{
		struct mtk_video_dec_buf *src_buf_info =
			container_of(bs, struct mtk_video_dec_buf, bs_buffer);
		struct mtk_video_dec_buf *dst_buf_info =
			container_of(fb, struct mtk_video_dec_buf, frame_buffer);

		v4l2_m2m_buf_copy_metadata(&src_buf_info->m2m_buf.vb,
					   &dst_buf_info->m2m_buf.vb, true);
	}

	/* DIAG: does the kernel see our bitstream in the input buffer? (if this
	 * shows zeros, the client write never reached the buffer; if it shows the
	 * slice, the data is present and the HW's input DMA is the suspect). */
	if (unlikely(h264_diaglog))
		mtk_v4l2_vdec_err(ctx, "mt8365 BS va=%p dma=%pad sz=%zx first=%*ph",
				  bs->va, &bs->dma_addr, bs->size,
				  (int)min_t(size_t, bs->size, 24), bs->va);

	/*
	 * N45: the bitstream is already in DRAM -- vb2 owns this plane's cache
	 * state and the MMAP mapping is uncached on this non-coherent device.
	 * The dma_sync_single_for_device() that used to stand here was an
	 * illegal streaming sync on an IOMMU-scattered handle and cleaned
	 * foreign physical memory past the first page. Audit + barrier only.
	 */
	mtk_vcodec_mt8365_bs_audit(ctx, bs->dma_addr, bs->size);

	/* Pull the SPS (picture dimensions for the working-buffer sizing). The
	 * other H.264 controls are not consumed yet -- the IDR replay below uses
	 * the captured trace constants; mapping the per-frame SPS/PPS/frame_num
	 * fields off those constants is the next step. */
	sps = mtk_vdec_h264_get_ctrl_ptr(ctx, V4L2_CID_STATELESS_H264_SPS);
	if (IS_ERR(sps))
		return -EINVAL;

	/* allocate/realloc the HW working buffers for this resolution */
	{
		u32 w = ((u32)sps->pic_width_in_mbs_minus1 + 1) * 16;
		u32 h = ((u32)sps->pic_height_in_map_units_minus1 + 1) * 16;

		if (!(sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY))
			h *= 2;
		if (!inst->work_buf_ready ||
		    w != inst->work_buf_w || h != inst->work_buf_h) {
			ret = alloc_work_bufs(inst, w, h);
			if (ret)
				return ret;
		}
	}

	rec_y = (u32)fb->base_y.dma_addr;
	rec_c = (u32)fb->base_c.dma_addr;

	/*
	 * Compute the SPS/PPS/slice-header/picsize AVCVLD+VLDTOP register words
	 * from the V4L2 controls (RE'd bit-packing of the vendor SetSPSAVLD /
	 * SetPPSAVLD / SetSHDRAVLD1 / DecStart packers) instead of the captured
	 * IDR trace constants. A self-check below asserts they reproduce the
	 * known-good IDR values bit-for-bit. Signed fields (qp deltas, chroma qp
	 * offsets) are masked to their field width after two's-complement.
	 */
	pps = mtk_vdec_h264_get_ctrl_ptr(ctx, V4L2_CID_STATELESS_H264_PPS);
	slp = mtk_vdec_h264_get_ctrl_ptr(ctx, V4L2_CID_STATELESS_H264_SLICE_PARAMS);
	dph = mtk_vdec_h264_get_ctrl_ptr(ctx, V4L2_CID_STATELESS_H264_DECODE_PARAMS);
	if (IS_ERR_OR_NULL(pps) || IS_ERR_OR_NULL(slp) || IS_ERR_OR_NULL(dph)) {
		mtk_v4l2_vdec_err(ctx, "mt8365 missing PPS/SLICE_PARAMS/DECODE_PARAMS control");
		return -EINVAL;
	}
	mb_w = (u32)sps->pic_width_in_mbs_minus1 + 1;
	/* N56: the raw SPS map-unit row count, BEFORE the !frame_mbs_only
	 * doubling -- the sequence constant TOP+0x68 carries (see r_top68).
	 */
	mb_h_map = (u32)sps->pic_height_in_map_units_minus1 + 1;
	mb_h = mb_h_map;
	if (!(sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY))
		mb_h *= 2;

	/*
	 * ADAPTIVE per-slice field source (2026-07-10). A slice-based / direct-
	 * harness userspace populates H264_SLICE_PARAMS (header_bit_size,
	 * slice_type, num_ref_idx, ref_pic_list0); a spec-compliant FRAME_BASED
	 * userspace (GStreamer v4l2slh264dec) leaves them ZERO and expects the
	 * frame-based HW to derive them. This no-SCP backend must derive them
	 * itself in that case. DECISIVE detection signal: header_bit_size != 0.
	 *
	 *   present -> use the controls VERBATIM (byte-identical to the proven
	 *              direct-harness decode path -- zero regression), and
	 *   absent  -> parse the coded slice NAL in the OUTPUT buffer in-kernel for
	 *              header_bit_size + slice_type + num_ref_idx, and derive the
	 *              L0 ref list from DECODE_PARAMS (the DPB), reordered per
	 *              ref_pic_list_modification.
	 *
	 * Both paths feed the SAME downstream register programming through
	 * refidx[]/ref_picnum[]/ref_poc[] + is_inter/num_l0/eff_stype, so the HW
	 * sequence below is identical; only the field source differs.
	 *
	 * The DPB marshalling (update_dpb -> copy_decode_params -> fill_dpb_info)
	 * runs for BOTH paths: it resolves each ref's recon Y/C DMA address into
	 * inst->dpb[] (from the CAPTURE-buffer timestamps) and is a pure helper
	 * with no register side effects.
	 */
	sh_from_ctrl = (slp->header_bit_size != 0);
	/* v4l2_dpb is hoisted to the top of the function so it survives to the
	 * B-slice L1 ref-list build below (v4l2_h264_init_reflist_builder). */
	mtk_vdec_h264_update_dpb(dph, v4l2_dpb);
	mtk_vdec_h264_copy_decode_params(&inst->decode_params, dph, v4l2_dpb, false);
	mtk_vdec_h264_fill_dpb_info(ctx, &inst->decode_params, inst->dpb);

	if (sh_from_ctrl) {
		/* slice-based-aware userspace: use the supplied controls verbatim */
		eff_hbs = slp->header_bit_size;
		eff_first_mb = slp->first_mb_in_slice;
		eff_stype = slp->slice_type % 5;
		eff_qpd = slp->slice_qp_delta;
		eff_dbf_idc = slp->disable_deblocking_filter_idc;
		eff_alpha = slp->slice_alpha_c0_offset_div2;
		eff_beta = slp->slice_beta_offset_div2;
		eff_direct_spatial = (eff_stype == 1) &&
			!!(slp->flags & V4L2_H264_SLICE_FLAG_DIRECT_SPATIAL_MV_PRED);
		/*
		 * N48: the pred_weight_table() BIT WINDOW is not carried by any
		 * V4L2 control, so on this path (the one gstreamer's
		 * v4l2slh264dec actually takes -- it always supplies
		 * header_bit_size) pwt_start/pwt_end stayed 0/0 and the N28
		 * AVCVLD+0x230 HW-parse handshake below could never fire. That is
		 * why h264_predwt=1 was a byte-for-byte no-op under gst, and why
		 * the N28 "falsified as wired" verdict -- measured on the
		 * frame-based parse path, on a kernel that still had the N44
		 * 0x90-bit2 corruption, the N45 0x98 clamp and the N47 MaxPicNum
		 * constant -- does not apply here.
		 *
		 * Derive the window from the bitstream (the controls stay
		 * authoritative for every other field). N43 verified this parse's
		 * pwt window against ffmpeg's own H.264 parser slice-for-slice on
		 * all six WP vectors: 0 mismatches.
		 */
		/* N49: the hwsh walk needs this window too (it is the accounting
		 * check, and the source of pwt_end after the HW consumes the
		 * table), so derive it for that arm as well -- including the B /
		 * weighted_bipred_idc==1 shape, which the walk can also reach. */
		if (((h264_predwt & 1) || (h264_hwsh & HWSH_WALK)) &&
		    ((((pps->flags & V4L2_H264_PPS_FLAG_WEIGHTED_PRED)) &&
		      (eff_stype == 0 || eff_stype == 3)) ||
		     (pps->weighted_bipred_idc == 1 && eff_stype == 1))) {
			struct mt8365_rplm_op d_ops[34], d_ops1[34];
			u32 d_hbs = 0, d_fmb = 0, d_st = 2;
			int d_l0 = 0, d_l1 = 0, d_qpd = 0, d_dbf = 0, d_al = 0;
			int d_be = 0, d_ds = 0, d_fp = 0, d_bf = 0, d_nops = 0;
			int d_ci = 0, d_nops1 = 0;

			if (mt8365_parse_slice_header(sps, pps, bs->va, bs->size,
						      0, &d_hbs, &d_fmb, &d_st,
						      &d_l0, &d_l1, &d_qpd,
						      &d_dbf, &d_al, &d_be,
						      &d_ds, &d_fp, &d_bf,
						      &pwt_start, &pwt_end,
						      d_ops, &d_nops,
						      &d_ci, d_ops1, &d_nops1))
				pwt_start = pwt_end = 0;
		}
		/* field_pic_flag / bottom_field_flag are DECODE_PARAMS (picture-level)
		 * flags in the V4L2 H.264 stateless uAPI, not slice flags. */
		eff_field_pic = !!(dph->flags & V4L2_H264_DECODE_PARAM_FLAG_FIELD_PIC);
		eff_bottom_field = !!(dph->flags & V4L2_H264_DECODE_PARAM_FLAG_BOTTOM_FIELD);
		/* N50: the uAPI DOES carry cabac_init_idc on this path (it does
		 * NOT carry the L1 modification list -- nops_l1 stays 0 here). */
		eff_cabac_init = slp->cabac_init_idc;
		is_inter = (eff_stype != 2 && eff_stype != 4);
		num_l0 = is_inter ? (slp->num_ref_idx_l0_active_minus1 + 1) : 0;
		raw_num_l0 = num_l0;		/* N45 */
		if (num_l0 > 16)
			num_l0 = 16;
		num_l1 = (eff_stype == 1) ? (slp->num_ref_idx_l1_active_minus1 + 1) : 0;
		for (i = 0; i < num_l0; i++) {
			int di = slp->ref_pic_list0[i].index & 0xf;

			refidx[i] = di;
			ref_picnum[i] = dph->dpb[di].pic_num;
			ref_poc[i] = (u32)dph->dpb[di].top_field_order_cnt;
		}
	} else {
		/* frame-based (gst): parse the slice header + derive the ref list */
		struct mt8365_rplm_op ops[34];
		u32 p_hbs = 0, p_first_mb = 0, p_stype = 2;
		int p_num_l0 = 0, p_num_l1 = 0, nops = 0, n_avail = 0, p_qpd = 0;
		int p_dbf_idc = 0, p_alpha = 0, p_beta = 0, p_direct_spatial = 0;
		int p_field_pic = 0, p_bottom_field = 0;
		u32 ref_stype;		/* N34: slice_type that owns the ref list */

		ret = mt8365_parse_slice_header(sps, pps, bs->va, bs->size, 0,
						&p_hbs, &p_first_mb, &p_stype,
						&p_num_l0, &p_num_l1, &p_qpd,
						&p_dbf_idc, &p_alpha, &p_beta,
						&p_direct_spatial,
						&p_field_pic, &p_bottom_field,
						&pwt_start, &pwt_end,
						ops, &nops,
						&eff_cabac_init, ops_l1, &nops_l1);
		if (ret) {
			mtk_v4l2_vdec_err(ctx,
				"mt8365 in-kernel slice-header parse failed: %d (frame-based, no SLICE_PARAMS)",
				ret);
			return ret;
		}
		eff_hbs = p_hbs;
		eff_first_mb = p_first_mb;
		eff_stype = p_stype;
		eff_qpd = p_qpd;
		eff_dbf_idc = p_dbf_idc;
		eff_alpha = p_alpha;
		eff_beta = p_beta;
		eff_direct_spatial = p_direct_spatial;
		eff_field_pic = p_field_pic;
		eff_bottom_field = p_bottom_field;
		num_l1 = p_num_l1;	/* B: active L1 count -> 0x98[29:25] */
		/*
		 * N34: mixed-slice-type access unit. If VCL NAL #0 is I/SI but a
		 * LATER slice of the same picture is P/B, the reference-list
		 * descriptors must still be programmed -- they are picture-global
		 * registers the HW cannot re-derive per slice. Walk the remaining
		 * VCL NALs for the first inter slice and take its ref-list state.
		 */
		ref_stype = p_stype;
		if (h264_mixedslice && (p_stype == 2 || p_stype == 4)) {
			u32 k;
			bool got_inter = false;

			for (k = 1; k < 8; k++) {
				u32 m_hbs = 0, m_fmb = 0, m_st = 2;
				u32 m_pws = 0, m_pwe = 0;
				int m_l0 = 0, m_l1 = 0, m_nops = 0, m_qpd = 0;
				int m_dbf = 0, m_a = 0, m_b = 0, m_ds = 0;
				int m_fp = 0, m_bf = 0, m_ci = 0, m_nops1 = 0;
				struct mt8365_rplm_op m_ops[34], m_ops1[34];

				if (mt8365_parse_slice_header(sps, pps, bs->va,
							      bs->size, k,
							      &m_hbs, &m_fmb, &m_st,
							      &m_l0, &m_l1, &m_qpd,
							      &m_dbf, &m_a, &m_b,
							      &m_ds, &m_fp, &m_bf,
							      &m_pws, &m_pwe,
							      m_ops, &m_nops,
							      &m_ci, m_ops1, &m_nops1))
					break;		/* no more VCL NALs */
				if (m_fmb == 0)
					break;		/* next access unit */
				/*
				 * N111: any slice whose type differs from VCL
				 * NAL #0's makes this a MIXED access unit --
				 * recorded whether or not it is the slice whose
				 * ref lists we adopt.
				 */
				if (m_st != p_stype)
					mixed_au = true;
				if (m_st == 2 || m_st == 4)
					continue;	/* still intra */
				/*
				 * N111 arm 4: prefer a B slice's ref-list state
				 * over the first inter slice's, so the B list
				 * groups (A/B/C) are programmed for a picture
				 * that contains both P and B slices. Arms 1-3
				 * keep N34's "first inter slice wins".
				 */
				if (got_inter &&
				    !(h264_mixedslice >= 4 && m_st == 1 &&
				      ref_stype != 1))
					continue;
				got_inter = true;
				ref_stype = m_st;
				p_num_l0 = m_l0;
				num_l1 = m_l1;
				p_direct_spatial = m_ds;
				eff_direct_spatial = m_ds;
				nops = m_nops;
				memcpy(ops, m_ops, sizeof(ops));
				eff_cabac_init = m_ci;		/* N50 */
				nops_l1 = m_nops1;
				memcpy(ops_l1, m_ops1, sizeof(ops_l1));
				if (h264_mixedslice == 2)
					eff_stype = m_st;
				if (h264_mixedslice < 3)
					break;
			}
			/*
			 * N111: arm the mixed-AU register model only when the
			 * AU really is mixed. list_stype stays 0xff otherwise,
			 * so arms 0-2 and every uniform-slice-type picture are
			 * byte-for-byte the pre-N111 driver.
			 */
			if (h264_mixedslice >= 3 && mixed_au)
				list_stype = ref_stype;
		}
		/* initial P List0 (§8.2.4.2.1): short-term desc PicNum, then
		 * long-term asc LongTermPicNum (N27). */
		n_avail = mt8365_build_init_p_list(inst->decode_params.dpb,
						   refidx, h264_longterm,
						   dph->frame_num,
						   1u << (sps->log2_max_frame_num_minus4 + 4));
		is_inter = (ref_stype == 0 || ref_stype == 1 || ref_stype == 3) &&
			   n_avail > 0;
		num_l0 = is_inter ? min_t(int, p_num_l0, n_avail) : 0;
		raw_num_l0 = is_inter ? p_num_l0 : 0;		/* N45 */
		if (num_l0 > 16)
			num_l0 = 16;
		/*
		 * N135: the surplus indices of an over-length RefPicList0 hold a
		 * SECOND copy of an already-listed reference (§8.2.4.3.1), each
		 * with its own pred_weight_table() entry. This consumes the
		 * §8.2.4.2.1 INITIAL list, so it runs INSTEAD of (not after)
		 * mt8365_reorder_l0(), which is a permutation of the same array.
		 * Candidate counter runs unconditionally, so the denominator
		 * exists at either setting.
		 */
		l0_overlong = false;
		if (is_inter && nops > 0 && !eff_field_pic &&
		    p_num_l0 > n_avail &&
		    p_num_l0 <= V4L2_H264_NUM_DPB_ENTRIES) {
			h264_l0dupcand++;
			if (h264_l0dup) {
				int nfull = mt8365_build_l0_overlong(
					refidx, n_avail,
					inst->decode_params.dpb,
					dph->frame_num,
					1u << (sps->log2_max_frame_num_minus4 + 4),
					ops, nops, p_num_l0);

				if (nfull > 0) {
					num_l0 = min_t(int, nfull, 16);
					h264_l0duphit++;
					l0_overlong = true;
				}
			}
		}
		if (is_inter && nops > 0 && !l0_overlong)
			mt8365_reorder_l0(refidx, n_avail,
					  inst->decode_params.dpb,
					  dph->frame_num,
					  1u << (sps->log2_max_frame_num_minus4 + 4),
					  ops, nops);
		for (i = 0; i < num_l0; i++) {
			const struct slice_h264_dpb_entry *re =
				&inst->decode_params.dpb[refidx[i]];

			/*
			 * N47: a SHORT-TERM descriptor carries PicNum =
			 * FrameNumWrap, which is negative after a frame_num wrap;
			 * the register field is 19 bits, so the value must be the
			 * 19-bit two's complement (the vendor writes 0x7fffd /
			 * 0x7fff0), not the 16-bit truncation the uAPI marshalling
			 * leaves in pic_num. LONG_TERM entries keep pic_num
			 * (= LongTermPicNum).
			 */
			ref_picnum[i] = ((h264_picnum & PICNUM_SIGNED) &&
					 !(re->flags & V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM))
				? (u32)mt8365_st_picnum(re, dph->frame_num,
							1u << (sps->log2_max_frame_num_minus4 + 4))
				: re->pic_num;
			ref_poc[i] =
				(u32)re->top_field_order_cnt;
		}

		/*
		 * N51: a FIELD picture's RefPicList0 holds TWO entries per DPB
		 * frame (Sec 8.2.4.2.5) -- the frame-based list built above is
		 * exactly half of it. Rebuild it in field units: per-field
		 * entries in alternating-parity order, field PicNum, per-parity
		 * POC. eff_field_pic-gated, so frame-coded output is untouched.
		 *
		 * N104: ref_pic_list_modification IS now reinterpreted in field
		 * units (mt8365_reorder_field_l0(), Sec 8.2.4.3.1 on field
		 * PicNums), so a field slice carrying a modification list stays
		 * on the field path. Before N104 it fell back to the frame-unit
		 * list -- half the length, in the wrong number space -- which on
		 * MR6_BT_B was 93 of 120 field pictures.
		 */
		/*
		 * N58: a FRAME-coded picture of a PAFF stream takes the same
		 * field-unit path, with the current parity taken as TOP
		 * (eff_bottom_field is 0 on a frame picture). See PAFF_FLDREF.
		 */
		paff_frm = (h264_paff & PAFF_FLDREF) && !eff_field_pic &&
			   !(sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY) &&
			   !(sps->flags & V4L2_H264_SPS_FLAG_MB_ADAPTIVE_FRAME_FIELD);
		fld_rplm = eff_field_pic && nops > 0 &&
			   (h264_fldlist & FLDLIST_RPLM);
		if ((h264_fldlist & FLDLIST_LIST) &&
		    (eff_field_pic || paff_frm) && is_inter &&
		    (nops == 0 || fld_rplm)) {
			u8 f_di[32], f_bot[32];
			int f_pn[32];
			/*
			 * N58: on a PAFF FRAME picture the vendor's Sec
			 * 8.2.4.2.5 walk takes BOTTOM as the "same parity".
			 * Derived from its two FRAME windows with no free
			 * parameter -- it predicts all six PicNum values and
			 * all six parity bits, plus MC+0x564 = 1 (bot ? 1 : 2):
			 *   win2 gA [-1,-2,-16] A [sl1.bot, sl1.top, sl0.top]
			 *          = [2*-1+1, 2*-1, 2*-8]
			 *   win5 gA [-1,16,-2]  A [sl1.bot, sl0.top, sl1.top]
			 *          = [2*-1+1, 2*8,  2*-1]
			 * Taking TOP instead inverts every entry's parity and
			 * was measured a large regression (52 -> 7 inf frames).
			 */
			int cur_bot = paff_frm ? 1 : eff_bottom_field;
			int nfld = mt8365_build_field_p_list(v4l2_dpb,
							     dph->frame_num,
							     1u << (sps->log2_max_frame_num_minus4 + 4),
							     cur_bot,
							     h264_longterm,
							     f_di, f_bot, f_pn,
							     MT8365_FIELD_LIST_MAX);

			/*
			 * N104: Sec 8.2.4.3.1 in FIELD units. Only for a real
			 * field picture -- a PAFF FRAME picture on the N58 path
			 * has CurrPicNum = frame_num and keeps the frame-unit
			 * reorder already applied to refidx[] above.
			 */
			if (fld_rplm && nfld > 0)
				mt8365_reorder_field_l0(f_di, f_bot, f_pn, nfld,
							v4l2_dpb,
							dph->frame_num,
							1u << (sps->log2_max_frame_num_minus4 + 4),
							ops, nops);

			if (nfld > 0) {
				fld_list = true;
				n_avail = nfld;
				/*
				 * N58: on a FRAME picture num_ref_idx_l0_active
				 * counts FRAMES, so it is half the field-unit
				 * length; the vendor fills every group to the
				 * number of available reference FIELDS (three on
				 * both of its FRAME windows, four on win6), the
				 * same fill-to-num_valid rule N15 established.
				 */
				num_l0 = paff_frm ? nfld
						  : min_t(int, p_num_l0, nfld);
				if (num_l0 > 16)
					num_l0 = 16;
				for (i = 0; i < num_l0; i++) {
					int di = f_di[i];

					refidx[i] = di;
					ref_bot[i] = f_bot[i];
					ref_picnum[i] =
						(v4l2_dpb[di].flags & V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM)
						? v4l2_dpb[di].pic_num
						: (u32)f_pn[i];
					ref_poc[i] = f_bot[i]
						? (u32)v4l2_dpb[di].bottom_field_order_cnt
						: (u32)v4l2_dpb[di].top_field_order_cnt;
				}
			}
			if (h264_fldlist & FLDLIST_DIAG)
				pr_info("mt8365 FLDRL0 bot=%d fn=%u raw=%d nfld=%d nl0=%d di=%d,%d,%d,%d,%d,%d par=%d,%d,%d,%d,%d,%d pn=%d,%d,%d,%d,%d,%d poc=%d,%d,%d,%d,%d,%d\n",
					eff_bottom_field, dph->frame_num,
					p_num_l0, nfld, num_l0,
					nfld > 0 ? f_di[0] : -1, nfld > 1 ? f_di[1] : -1,
					nfld > 2 ? f_di[2] : -1, nfld > 3 ? f_di[3] : -1,
					nfld > 4 ? f_di[4] : -1, nfld > 5 ? f_di[5] : -1,
					nfld > 0 ? f_bot[0] : -1, nfld > 1 ? f_bot[1] : -1,
					nfld > 2 ? f_bot[2] : -1, nfld > 3 ? f_bot[3] : -1,
					nfld > 4 ? f_bot[4] : -1, nfld > 5 ? f_bot[5] : -1,
					nfld > 0 ? f_pn[0] : -1, nfld > 1 ? f_pn[1] : -1,
					nfld > 2 ? f_pn[2] : -1, nfld > 3 ? f_pn[3] : -1,
					nfld > 4 ? f_pn[4] : -1, nfld > 5 ? f_pn[5] : -1,
					nfld > 0 ? (int)ref_poc[0] : -1,
					nfld > 1 ? (int)ref_poc[1] : -1,
					nfld > 2 ? (int)ref_poc[2] : -1,
					nfld > 3 ? (int)ref_poc[3] : -1,
					nfld > 4 ? (int)ref_poc[4] : -1,
					nfld > 5 ? (int)ref_poc[5] : -1);
		}
	}

	/*
	 * B-slice (bi-prediction) L1 ref-list derivation -- N15 (2026-07-17).
	 * Build the spec RefPicList0 (b0) + RefPicList1 (b1) via the mainline
	 * v4l2_h264_build_b_ref_lists() helper (spec 8.2.4.2, POC-based, incl. the
	 * "swap first two when L1==L0" rule). Group A stays the raw PicNum-desc
	 * list (refidx, above); the trace showed the HW ref-list groups B/C carry
	 * b0/b1 respectively. Gated on the B slice_type so P/I are untouched.
	 * eff_stype comes from SLICE_PARAMS (sh_from_ctrl) or the in-kernel parse.
	 */
	if (is_inter && (list_stype == 0xff ? eff_stype : list_stype) == 1) {
		mt8365_build_b_reflists(dph, sps, v4l2_dpb, l0idx, l1idx,
					&num_b0, &num_b1);
		/* only activate if the helper produced a usable L1 (>=1 ref) */
		use_l1 = (num_b0 > 0 && num_b1 > 0);

		/*
		 * N50: apply ref_pic_list_modification to RefPicList1. The
		 * modification process of Sec 8.2.4.3 is the SAME for L0 and L1
		 * (only the list it operates on differs), so the L0 reorder helper
		 * is reused verbatim on l1idx. Default off; nops_l1 is also always
		 * 0 on the slice-based (ctrl) path, which carries no L1
		 * modification list in the uAPI.
		 */
		if (h264_l1mod && nops_l1 > 0 && num_b1 > 0)
			mt8365_reorder_l0(l1idx, num_b1,
					  inst->decode_params.dpb,
					  dph->frame_num,
					  1u << (sps->log2_max_frame_num_minus4 + 4),
					  ops_l1, nops_l1);

		/*
		 * Build aidx[] = ALL valid short-term refs in raw PicNum-descending
		 * order (the vendor group-A list), from the shared decode_params.dpb
		 * (populated by copy_decode_params in BOTH the slice-based and gst
		 * paths). Filled to num_valid so the B recon-fill / group A / colMV
		 * load every reference (forward AND backward), matching the vendor.
		 * Independent of refidx[] (which the gst path may reorder per L0
		 * ref_pic_list_modification -- group A must stay RAW PicNum-desc).
		 */
		n_valid = mt8365_build_init_p_list(inst->decode_params.dpb,
						   aidx, h264_longterm,
						   dph->frame_num,
						   1u << (sps->log2_max_frame_num_minus4 + 4));
		/* if no valid ref survived, fall back to the L0-only path */
		if (n_valid == 0)
			use_l1 = false;

		/*
		 * N54: a B FIELD's three ref-list groups, in FIELD units. The
		 * frame-unit lists above are exactly half the length the HW
		 * expects and carry the same-parity PicNum twice; see the
		 * h264_fldblist comment for the vendor-vs-ours transcription.
		 * Group A is the field-unit PicNum-desc list (the N51 P builder,
		 * reused verbatim); groups B/C are the Sec 8.2.4.2.4 POC-ordered
		 * L0/L1 fed through the SAME Sec 8.2.4.2.5 parity interleave.
		 *
		 * Gated on fld_list -- i.e. only when the N51 L0 field
		 * conversion itself ran, which already carries eff_field_pic,
		 * is_inter, a non-empty field list and (crucially) no L0
		 * ref_pic_list_modification. Sec 8.2.4.3.1 operates on field
		 * PicNums when field_pic_flag is set and this driver does not
		 * reinterpret it in field units, so a B field carrying a
		 * modification list keeps the frame path in both directions
		 * (nops_l1 == 0 is the L1 half of the same condition).
		 */
		if (h264_fldblist && fld_list && use_l1 && nops_l1 == 0) {
			int cur_poc = eff_bottom_field
				? (int)dph->bottom_field_order_cnt
				: (int)dph->top_field_order_cnt;
			u32 maxfn = 1u << (sps->log2_max_frame_num_minus4 + 4);
			/* N58: BOTTOM is the "same parity" on a PAFF frame pic */
			int cur_bot = paff_frm ? 1 : eff_bottom_field;

			/*
			 * N58: a FRAME picture's POC is Min(Top,Bottom)
			 * (Sec 8.2.1), which is what the Sec 8.2.4.2.4 POC
			 * ordering pivots on.
			 */
			if (paff_frm)
				cur_poc = min_t(int,
						(int)dph->top_field_order_cnt,
						(int)dph->bottom_field_order_cnt);

			n_fa = mt8365_build_field_p_list(v4l2_dpb,
							 dph->frame_num, maxfn,
							 cur_bot,
							 h264_longterm,
							 fa_di, fa_bot, fa_pn,
							 MT8365_BFIELD_MAX);
			n_fb0 = mt8365_build_field_b_lists(v4l2_dpb,
							   dph->frame_num, maxfn,
							   cur_bot,
							   cur_poc,
							   h264_longterm,
							   fb0_di, fb0_bot, fb0_pn,
							   fb1_di, fb1_bot, fb1_pn,
							   &n_fb1);
			fld_blist = (n_fa > 0 && n_fb0 > 0 && n_fb1 > 0);
			if (fld_blist && (h264_fldlist & FLDLIST_DIAG))
				pr_info("mt8365 FLDB bot=%d fn=%u poc=%d nA=%d nL0=%d nL1=%d A=%d,%d,%d,%d,%d,%d L0=%d,%d,%d,%d,%d,%d L1=%d,%d,%d,%d,%d,%d\n",
					eff_bottom_field, dph->frame_num, cur_poc,
					n_fa, n_fb0, n_fb1,
					n_fa > 0 ? fa_pn[0] : -1, n_fa > 1 ? fa_pn[1] : -1,
					n_fa > 2 ? fa_pn[2] : -1, n_fa > 3 ? fa_pn[3] : -1,
					n_fa > 4 ? fa_pn[4] : -1, n_fa > 5 ? fa_pn[5] : -1,
					n_fb0 > 0 ? fb0_pn[0] : -1, n_fb0 > 1 ? fb0_pn[1] : -1,
					n_fb0 > 2 ? fb0_pn[2] : -1, n_fb0 > 3 ? fb0_pn[3] : -1,
					n_fb0 > 4 ? fb0_pn[4] : -1, n_fb0 > 5 ? fb0_pn[5] : -1,
					n_fb1 > 0 ? fb1_pn[0] : -1, n_fb1 > 1 ? fb1_pn[1] : -1,
					n_fb1 > 2 ? fb1_pn[2] : -1, n_fb1 > 3 ? fb1_pn[3] : -1,
					n_fb1 > 4 ? fb1_pn[4] : -1, n_fb1 > 5 ? fb1_pn[5] : -1);
		}
	}

	r_sps = (sps->chroma_format_idc & 3)
	      | ((sps->log2_max_frame_num_minus4 & 0xf) << 2)
	      | ((sps->pic_order_cnt_type & 3) << 6)
	      | ((sps->log2_max_pic_order_cnt_lsb_minus4 & 0xf) << 8)
	      /*
	       * bit12 = delta_pic_order_always_zero_flag (N33, 2026-07-26 vendor
	       * read-trace). Derived, not guessed: across four vendor `vpud`
	       * traces captured this session on the 13-range vendor DTB, the SPS
	       * word AVCVLD+0x88 is 0x0034f045 for MR1_BT_A and 0x0024a081 /
	       * 0x0035e315 / 0x0035e091 for CVWP1_TOSHIBA_E / HCBP1_HHI_A /
	       * MR3_TANDBERG_B. Unpacked against the four SPSes, every other
	       * field matches this packing exactly; the ONLY residual bit is
	       * bit12, set in MR1 and clear in the other three -- and MR1 is the
	       * only one of the four with pic_order_cnt_type == 1, i.e. the only
	       * one whose SPS even CARRIES delta_pic_order_always_zero_flag
	       * (=1, ffmpeg trace_headers). Gated on poc_type==1 so that a
	       * userspace that leaves the flag set on a poc_type 0/2 stream
	       * cannot perturb any currently-passing vector: for every clip with
	       * poc_type != 1 the word is byte-identical to before.
	       */
	      | ((sps->pic_order_cnt_type == 1 &&
		  (sps->flags & V4L2_H264_SPS_FLAG_DELTA_PIC_ORDER_ALWAYS_ZERO))
			? 1u : 0u) << 12
	      | ((sps->max_num_ref_frames & 0x1f) << 13)
	      | ((sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY ? 1u : 0u) << 18)
	      | ((sps->flags & V4L2_H264_SPS_FLAG_MB_ADAPTIVE_FRAME_FIELD ? 1u : 0u) << 19)
	      | ((sps->flags & V4L2_H264_SPS_FLAG_DIRECT_8X8_INFERENCE ? 1u : 0u) << 20)
	      | 0x200000u;	/* fixed HW-enable bit */

	/*
	 * N44: bit2 was packed as the PPS weighted_pred_flag. That RE assignment is
	 * WRONG and actively corrupts multi-reference weighted P prediction. The
	 * vendor writes AVCVLD+0x090 = 0x000007e0 -- bit2 CLEAR -- on every picture
	 * of ~/bftrace/trace-cvwp1.log, a clip whose PPS weighted_pred_flag is 1 and
	 * which vpud decodes BIT-EXACT (hw_md5 == the official JVT md5). Setting
	 * bit2 is measured strictly harmful and never helpful (h264_wp bit2 restores
	 * it for A/B):
	 *
	 *   clip                 bit2 set (old)                    bit2 clear (vendor)
	 *   CVWP1_TOSHIBA_E      n:3 6.40 dB,  8 exact frames       n:3 29.73 dB,  8 exact
	 *   CVWP5_TOSHIBA_E      n:3 4.52 dB,  7 exact frames       n:3 20.48 dB,  7 exact
	 *   CAWP1_TOSHIBA_E      n:3 4.60 dB,  8 exact frames       n:4 first LUMA-bad
	 *                                                           frame (chroma bad
	 *                                                           from n:4 too)
	 *   CAWP5_TOSHIBA_E      n:4 4.82 dB,  7 exact frames       n:4 23.68 dB,  7 exact
	 *
	 * N130 CORRECTION (2026-08-24) -- this table's CAWP1 cell used to read
	 * "n:1..n:13 ALL BIT-EXACT, 19 exact", and the paragraph below used to call
	 * that "the decisive one". BOTH WERE FALSE, and the error was the
	 * INSTRUMENT, not the decode: the WP harness (ours-gen.sh, now in our
	 * internal tests) counted
	 * `psnr_y:inf` ONLY, so a frame whose chroma was wholly wrong scored as
	 * "exact". Re-measured per plane on the SAME decode (md5
	 * 8138e965b9b071bfade978c5578e8f28, kernel 6.18.35-01361-gc39229356453):
	 * CAWP1 with bit2 CLEAR carries 1 298 049 differing LUMA samples over 70 of
	 * 90 frames (first bad frame 13, 0-based) and 3 541 411 differing CHROMA
	 * samples from frame 3 (0-based) onward; only 9 of its 90 frames are exact
	 * on all three planes. N129 refuted the "luma-clean" reading; N130 re-ran
	 * the h264_wp bits 1/2/8 A/B chroma-aware.
	 *
	 * N134 REVERSAL (2026-08-24) -- the bit2 VERDICT above is now RETRACTED,
	 * and the retraction is not a correction of the measurement but of its
	 * SCOPE. Every number in the table was taken with h264_hwsh=0, i.e. with
	 * the AVCVLD+0x230 strobe never fired and pred_weight_table() never
	 * consumed by the silicon. bit2 is the weighted-prediction ENABLE; N44
	 * enabled a weight set that had never been latched. Since N132 made the
	 * strobe the built-in default, the same bit makes CVWP1_TOSHIBA_E and
	 * CAWP1_TOSHIBA_E BIT-EXACT on all three planes (fluster Fail -> Success
	 * on both). See the h264_wpflag block near the top of this file for the
	 * full chain, the paired-control table and the register witness. bit2 is
	 * therefore packed by DEFAULT below; h264_wpflag=0 restores the pre-N134
	 * behaviour this comment describes.
	 *
	 * What SURVIVES from N130 is its instrument correction (the CAWP1 cell's
	 * "n:1..n:13 ALL BIT-EXACT, 19 exact" was a luma-only artefact). What does
	 * NOT survive is "clearing bit2 still beats setting it on every vector" --
	 * that held only for the strobe-less configuration N44 measured.
	 */
	r_pps90 = ((pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE ? 1u : 0u) << 0)
		| ((pps->weighted_bipred_idc & 3) << 3)
		| (((u32)pps->pic_init_qp_minus26 & 0x3f) << 5)		/* signed6 */
		| (((u32)pps->chroma_qp_index_offset & 0x1f) << 11)	/* signed5 */
		| ((pps->flags & V4L2_H264_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT ? 1u : 0u) << 16)
		| ((pps->flags & V4L2_H264_PPS_FLAG_CONSTRAINED_INTRA_PRED ? 1u : 0u) << 17)
		| ((pps->flags & V4L2_H264_PPS_FLAG_TRANSFORM_8X8_MODE ? 1u : 0u) << 18)
		| (((u32)pps->second_chroma_qp_index_offset & 0x1f) << 19);	/* signed5 */

	/*
	 * AVCVLD+0x90 bit1 (0x2) -- FIELD-picture config (F1.6, vendor read-trace
	 * cvmp_mot_fld0 720x480). The vendor sets 0x90 bit1 on EVERY field
	 * (trace-fld.log seq 585: 0x90=0x00000002 while r_pps90's PPS bits are all
	 * 0 for this CAVLC clip). bit1 is UNUSED by the PPS packing above, so it is
	 * a pure field-mode flag. Without it (progressive 0x90) the intra I-field
	 * stalls at MB5. Gated on eff_field_pic -> frame-coded byte-identical.
	 */
	if (eff_field_pic)
		r_pps90 |= 0x2u;

	r_pps94 = (pps->num_ref_idx_l0_default_active_minus1 & 0x1f)
		| ((pps->num_ref_idx_l1_default_active_minus1 & 0x1f) << 5);

	/*
	 * N44: is this an EXPLICITLY weighted slice? Only such slices are touched
	 * by h264_wp, so every non-weighted vector stays byte-identical whatever
	 * the bitmask is.
	 */
	wp_slice = h264_wp &&
		(((pps->flags & V4L2_H264_PPS_FLAG_WEIGHTED_PRED) && eff_stype == 0) ||
		 (pps->weighted_bipred_idc == 1 && eff_stype == 1));
	if (wp_slice && (h264_wp & 2))
		r_pps94 = 0;
	if (wp_slice && (h264_wp & 4))
		r_pps90 |= 0x4u;	/* subsumed by h264_wpflag; kept for A/B */

	/*
	 * N134: AVCVLD+0x90 bit2 = the PPS weighted_pred_flag, on explicitly
	 * weighted slices -- the weighted-prediction ENABLE that pairs with the
	 * N132 AVCVLD+0x230 table-latch strobe. DEFAULT ON. The predicate is
	 * EXACTLY the one the A/B was measured with (`wp_slice` evaluated at
	 * h264_wp == 4), so the landed default is the measured configuration and
	 * not a generalisation of it: a P/SP-only or unconditional-PPS variant
	 * would be a DIFFERENT arm and is not what was scored.
	 */
	if (h264_wpflag &&
	    (((pps->flags & V4L2_H264_PPS_FLAG_WEIGHTED_PRED) && eff_stype == 0) ||
	     (pps->weighted_bipred_idc == 1 && eff_stype == 1)))
		r_pps90 |= 0x4u;

	/*
	 * Slice-header packed word P, RMW-split into 0x8c[12:0] / 0x98[31:13]:
	 *   [12:0]  first_mb_in_slice
	 *   [16:13] raw H.264 slice_type -- V4L2 slice_type is the value mod 5
	 *           (I=2); single-slice pictures use the "all slices same type"
	 *           5..9 range, so the HW wants v4l2_slice_type + 5 (I -> 7).
	 *   [24:20] REF-LIST-ACTIVE count. THE SLICE-BASED MULTI-REF ENABLE
	 *           (2026-07-10, vendor read-trace). The vendor writes
	 *           0x98[21:20] = num_ref_idx_l0_active_minus1 (capped at 2 by
	 *           this clip's PPS default): f1=0 (0x0000a000), f2=1 (0x0010a000),
	 *           f3+=2 (0x0020a000). This is the field that TELLS THE HW HOW
	 *           MANY ACTIVE REFERENCES the ref-list tables hold -- with it 0
	 *           the HW treats the slice as <=1 ref and IGNORES the AVCVLD
	 *           PicNum groups + AVCMV descriptors (why kernel 00824 was inert).
	 *           EVERY prior fn>=2 attempt wrote slice_type (=0 for P) here, so
	 *           the ref-list was never enabled. For an I/IDR slice_type=2 gives
	 *           the vendor's IDR value 0x0020e000 (num_ref_idx is N/A), so keep
	 *           slice_type on the I path (self-check below still holds).
	 * (2<<20)|(7<<13) = the trace constant 0x98=0x0020e000.
	 */
	{
		/* reffld = 0x98[24:20] ref-count enable, from the effective per-slice
		 * fields (adaptive block above): for P it is num_l0-1 (active L0 ref
		 * count minus one), clamped only by the 5-bit field width [24:20].
		 * The old code capped this at 2 (=3 refs) -- a leftover from the
		 * canonical clip whose PPS default was 3 refs -- which lied to the HW
		 * for any P-frame needing >=4 active refs (the 4th+ ref went unused ->
		 * catastrophic corruption, e.g. BA_MW_D dropping to ~18 dB at frame 5).
		 * num_l0 and the ref-list group/descriptor arrays already handle up to
		 * 16 refs, so uncap to min(num_l0-1, 15). For I it carries slice_type. */
		/*
		 * N111: on a MIXED access unit the picture-level ref-count field
		 * must describe the picture's INTER slices, not VCL NAL #0's I
		 * slice -- so it is taken from list_stype, whose ref lists are
		 * what the group registers actually hold.
		 */
		u32 cnt_stype = (list_stype == 0xff) ? eff_stype : list_stype;
		u32 reffld = (cnt_stype != 2 && cnt_stype != 4)
			? (u32)(num_l0 > 0 ? min_t(int, num_l0 - 1, 15) : 0)
			: (cnt_stype & 0x1f);
		/*
		 * N45 (landed default): on a slice whose num_ref_idx_l0_active
		 * EXCEEDS the available DPB references (num_ref_idx_active_override
		 * with an over-length list -- the CAWP5/CVWP5 shape), report the RAW
		 * bitstream count here instead of the clamped ref-list slot count.
		 * The clamp is right for the ref-list SLOTS (there is no reference
		 * picture to put in the extra ones) but wrong for this field, which
		 * the HW's own slice-header/pred_weight_table parse consumes: the
		 * bitstream sends a pred_weight_table() entry for every ACTIVE
		 * entry, so a short count desynchronises it. Byte-identical on every
		 * slice where the two agree -- i.e. every clip in the 67-vector
		 * passer set. h264_wp bit 16 restores the old clamped value for A/B.
		 */
		if (!(h264_wp & 16) && (eff_stype != 2 && eff_stype != 4) &&
		    raw_num_l0 > num_l0)
			reffld = (u32)min_t(int, raw_num_l0 - 1, 15);
		/* N44: the WP value differential -- see h264_wp. */
		if (wp_slice && (h264_wp & 1))
			reffld = 0;
		else if (wp_slice && (h264_wp & 8))
			reffld = pps->num_ref_idx_l0_default_active_minus1 & 0x1f;
		/*
		 * L1-active field 0x98[29:25] = num_ref_idx_l1_active_minus1 -- the
		 * MULTI-REF TEMPORAL-DIRECT enable, symmetric to the L0 field [24:20].
		 * Vendor trace (CVBS3_Sony_C, 13-range DTB, this session): a B slice with
		 * 2 active L1 refs writes 0x98=0x02102000 -> [24:20]=1 (L0 active=2) AND
		 * [29:25]=1 (L1 active=2). Our driver programmed only [24:20], leaving the
		 * HW to run L1 as a single ref -- so any temporal-direct B whose L1 has 2+
		 * future refs (co-located P + a farther P) mis-derived the co-located field
		 * (CVBS3 34/300, worst 13.45 dB). The synthetic 4-ref clip passes because
		 * its L1 is 1-active (bit25=0). Gated on B; L1-active==1 leaves it 0 so
		 * every currently-passing clip (2-ref temporal/spatial/explicit-B, the
		 * fluster passers, the L1=1 synthetic 4-ref) is byte-identical. Clamp to
		 * the available L1 refs (num_b1) and the 5-bit field width. */
		int eff_num_l1 = ((list_stype == 0xff ? eff_stype : list_stype) == 1
				  && use_l1)
			? min_t(int, num_l1, num_b1) : 0;
		u32 l1fld = (eff_num_l1 > 1)
			? (u32)min_t(int, eff_num_l1 - 1, 15) : 0;

		/*
		 * N111 -- the slice_type field [16:13] carries the RAW H.264
		 * slice_type, i.e. the 5..9 "all slices of this picture have
		 * the same slice_type" form ONLY when that is true.
		 *
		 * Vendor evidence (~/bftrace, AVCVLD+0x98 at the MISC+0x28
		 * DECODE-GO of every kick): the SINGLE-slice-per-picture traces
		 * write only 5/6/7 (trace-frm 0x0000a000/0x0018c000/0x0020e000
		 * = P5/B6/I7), while the MULTI-slice traces write the 0..4 form
		 * (trace-mr1 = MR1_BT_A: 0x00604000 -> 2 = I on its 4-slice IDR,
		 * 0x00600000 -> 0 = P on its 2-slice P pictures; trace-mr3 the
		 * same). That is exactly what those bitstreams code.
		 *
		 * This driver added 5 unconditionally, so on a picture whose
		 * slices do NOT all share a type it told the hardware "every
		 * slice of this picture is an I slice" -- the same
		 * programmed-once-for-a-case-with-two-answers shape as the
		 * ref-list, descriptor and mask defects before it.  Gated on
		 * mixed_au, so every uniform picture (and therefore 135 of the
		 * 137 JVT-AVC_V1 vectors, measured) is byte-identical.
		 */
		r_shdr_p = (eff_first_mb & 0x1fff)
			 | (((eff_stype + (list_stype == 0xff ? 5u : 0u)) & 0xf) << 13)
			 | ((eff_field_pic ? 1u : 0u) << 17)		/* field_pic_flag (F1) */
			 | ((eff_bottom_field ? 1u : 0u) << 18)		/* bottom_field_flag (F1) */
			 | ((eff_direct_spatial ? 1u : 0u) << 19)	/* B: direct_spatial_mv_pred_flag */
			 | ((reffld & 0x1f) << 20)
			 | ((l1fld & 0x1f) << 25);			/* B: num_ref_idx_l1_active_minus1 */
	}

	/*
	 * AVCVLD+0x9c[6:0] = slice_qp_delta (7-bit two's-complement). Vendor
	 * fresh-vs-canonical read-trace, 2026-07-12: this low byte is NOT a
	 * constant -- it is the per-slice slice_qp_delta, and the HW dequantizes
	 * with it rather than re-parsing QP from the bitstream. The prior "constant
	 * 0x71" reading was a single-clip-trace artifact: the canonical regtrace
	 * clip's slice_qp_delta is -15 (== 0x71 in 7 bits), so a hardcoded 0x71
	 * matched it by coincidence and every other clip decoded with the WRONG
	 * dequant scale (mean ~correct, AC scrambled, ~9-11 dB). 20/20 correlation
	 * across two 176x144 clips proves it: canonical f0..f7 -15 -> 0x71, f8 -14
	 * -> 0x72, f9 -13 -> 0x73 (the "0x72/73 DPB-fullness counter" was really the
	 * QP stepping); fresh clip +1/+2/.../+7 -> 0x01/0x02/.../0x07. bits[18:17]
	 * = nal_ref_idc, bit19 = IDR (both trace-verified). Sourced from the V4L2
	 * SLICE_PARAMS (slice-based) or the in-kernel slice-header parse (gst).
	 *
	 * The same word also carries the per-slice deblocking-filter control
	 * (vpud H264_HAL_DecStart @0x4c50c RE, mt8365 libvpud_vcodec.so):
	 *   [8:7]   = disable_deblocking_filter_idc (0=on 1=off 2=on-except-bnd)
	 *   [12:9]  = slice_alpha_c0_offset_div2 (signed 4-bit)
	 *   [16:13] = slice_beta_offset_div2     (signed 4-bit)
	 * Leaving these 0 forced idc=0 (loop filter always ON), so any clip that
	 * signals the filter OFF (NL1_Sony_D / SVA_NL1_B, idc=1) decoded a
	 * spuriously-deblocked frame -- ~42-45 dB, errors on the 4-px block edges.
	 * For the common deblock-ON default (idc=0, offsets 0) all three fields
	 * are 0, so r_9c is byte-identical to before -> guaranteed no regression.
	 */
	r_9c = ((u32)eff_qpd & 0x7fu)
	     | (((u32)eff_dbf_idc & 3u) << 7)		/* disable_deblocking_filter_idc */
	     | (((u32)eff_alpha & 0xfu) << 9)		/* slice_alpha_c0_offset_div2 (s4) */
	     | (((u32)eff_beta & 0xfu) << 13)		/* slice_beta_offset_div2 (s4) */
	     | ((dph->nal_ref_idc & 3) << 17)
	     | ((dph->flags & V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC ? 1u : 0u) << 19)
	     /* N50: cabac_init_idc, gated (default 0 -> byte-identical). */
	     | ((h264_cabacinit &&
		 (pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE) &&
		 (eff_stype == 0 || eff_stype == 1 || eff_stype == 3))
			? (((u32)eff_cabac_init & 3u) << 20) : 0u);

	/*
	 * 0x84 bit23. Frame semantics = non-IDR (progressive IDR=0, P=1 -- correct,
	 * canonical bit-exact). But the vendor field read-trace (cvmp_mot_fld0
	 * 720x480) sets bit23=1 at decode time on EVERY field -- INCLUDING the IDR
	 * top field (0x84=0xae80e007 for the 0x98=0x0022e000 IDR field), and again
	 * for the bottom field + P-fields -- so for a field picture bit23 is a FIELD
	 * flag, not non-IDR. With it 0 the intra I-field HW-stalls at MB5 (F1.6:
	 * 0x0a0/0x90 alone did not clear the stall; this is the remaining diff).
	 * Set bit23 for (non-IDR) OR (field_pic); frame-coded byte-identical.
	 *
	 * N40 CORRECTION to the two claims above ("bit23 = non-IDR" for frames,
	 * "bit23 = a FIELD flag" for fields): BOTH are artefacts of reading only
	 * one of the vendor's two writes. The vendor writes AVCVLD+0x84 TWICE per
	 * picture and the pair is always 0xae00e007 (pre-trigger) then 0xae80e007
	 * (post-trigger, right before the +0x238 commit strobe) -- for every
	 * picture of the progressive trace AND every field of the field trace,
	 * IDR and non-IDR alike (trace-frm.log seq 21/646, 750/1396, 1500/2166;
	 * trace-fld.log seq 20/649, 753/1400, 1504/2209). bit23 is a pre/post
	 * STROBE. The F1.6 measurement that bit23=0 stalls the I-field remains
	 * valid -- it set bit23=0 in BOTH writes -- but it does not establish the
	 * FIELD reading. The vendor-exact progressive pair is reachable behind
	 * h264_progcfg bits 2|4; the field path is deliberately left as-is here
	 * (it is a separately measured arm and out of this round's scope).
	 */
	r_84 = 0xae00e007u
	     | (((!(dph->flags & V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC)) || eff_field_pic)
			? 1u : 0u) << 23;

	/*
	 * PAFF field-picture geometry (F1, 2026-07-18 vendor read-trace). A field
	 * picture codes half the frame's MB rows (PicHeightInMbs =
	 * FrameHeightInMbs / 2); mb_h above is the full FRAME height in MBs
	 * (doubled for !frame_mbs_only). Split the geometry:
	 *   - MB-GRID regs (TOP+0x68, MC+0x264, TOP+0x6c) use mb_h_pic (halved for
	 *     a field pic) = the MB rows the HW decodes this pass;
	 *   - PIXEL/SURFACE regs (TOP+0x70, MC+0x20c, MC+0x55c) stay full-frame
	 *     height (the recon surface holds both interleaved fields).
	 * Trace-verified byte-for-byte on cvmp_mot_fld0 (720x480, field=720x240):
	 * field TOP+0x68=0x000e002c (mb_h_pic=15) & TOP+0x70=0x01e002d0 (full 480)
	 * vs the matched frame clip TOP+0x68=0x001d002c (mb_h=30). For a frame
	 * picture (progressive / PAFF-frame / MBAFF) mb_h_pic==mb_h so every
	 * frame-coded clip is byte-identical (non-regression by construction).
	 */
	mb_h_pic = eff_field_pic ? (mb_h / 2) : mb_h;

	/*
	 * N38 MBAFF. A picture is MBAFF-coded when the SPS says
	 * mb_adaptive_frame_field_flag (which is only present/meaningful when
	 * frame_mbs_only_flag == 0) AND this picture is frame-coded -- a FIELD
	 * picture in an MBAFF stream is a plain field, not MB-pair coded (7.4.2.1.1).
	 */
	eff_mbaff = !eff_field_pic &&
		    !(sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY) &&
		    !!(sps->flags & V4L2_H264_SPS_FLAG_MB_ADAPTIVE_FRAME_FIELD);

	/*
	 * TOP+0x68 = (MB-grid rows - 1) << 16 | (mb_w - 1). For MBAFF the HW walks
	 * MB *PAIRS*, so the row count is FrameHeightInMbs/2 -- vendor MBAFF
	 * 0x000e002c (14 = 30/2-1) vs the matched progressive 0x001d002c (29).
	 * Same halving as a field picture, different reason. MC+0x264 and TOP+0x6c
	 * stay FULL-frame for MBAFF (vendor writes 0x1d and 0x546=45*30), so only
	 * this word changes.
	 */
	/*
	 * N56 PAFF. The vendor's own PAFF trace (CVPA1_TOSHIBA_B 352x288,
	 * ~/bftrace/trace-paff.log, 8 picture windows) writes TOP+0x68 =
	 * 0x00080015 on EVERY picture -- the FRAME-coded pictures included, where
	 * this driver writes 0x00110015. In the same windows MC+0x264 (8 -> 0x11)
	 * and TOP+0x6c (0xc6 -> 0x18c) DO switch to the frame values, so only this
	 * word is wrong.
	 *
	 * The unifying reading, with no free parameter: TOP+0x68's row field is
	 * the SPS pic_height_in_map_units_minus1 -- a SEQUENCE constant, never a
	 * per-picture value. It reproduces every value measured so far:
	 *   progressive (frame_mbs_only=1): map units == FrameHeightInMbs   -> 29 (720x480)
	 *   MBAFF frame:                    map units == MB PAIR rows       -> 14 (N38)
	 *   PAFF field:                     map units == FieldHeightInMbs   ->  8 (352x288)
	 *   PAFF frame:                     map units == FieldHeightInMbs   ->  8  <-- the fix
	 * mb_h_pic already equals the map-unit count in the first three cases, so
	 * this changes the emitted value on a PAFF FRAME picture and nowhere else.
	 */
	r_top68 = (mb_w - 1) |
		  ((((h264_paff & PAFF_T68) ? (mb_h_map)
		     : (eff_mbaff && (h264_mbaff & MBAFF_T68)) ? (mb_h / 2)
							       : mb_h_pic) - 1) << 16);
	r_top70 = (mb_w * 16) | ((mb_h * 16) << 16);
	r_top100 = (sps->bit_depth_luma_minus8 & 0xf)
		 | ((sps->bit_depth_chroma_minus8 & 0xf) << 4);

	/* Regression self-check: for the trace-verified IDR of the canonical test
	 * clip these must reproduce the captured constants. A mismatch means a
	 * packer field is wrong or an unexpected SPS/PPS value. Gated to IDR frames
	 * only -- an inter frame legitimately differs here (r_84 bit23, slice-hdr
	 * type). r_9c is EXCLUDED: its low byte is slice_qp_delta (per-slice,
	 * clip-dependent), so it is 0x000e0071 only for the canonical clip's -15 QP
	 * delta and legitimately differs on any other clip. r_top68/r_top70 are
	 * ALSO EXCLUDED: they encode the picture MB/pixel dimensions and so
	 * legitimately differ at any resolution other than 176x144 (a
	 * fresh-vs-canonical vendor read-trace confirmed the picture-geometry words
	 * are the only writes that scale with resolution). Comparing them against
	 * the canonical 176x144 constants falsely fired "PACKER MISMATCH" on every
	 * >176x144 clip. The remaining fields (SPS/PPS/r_84/r_top100/slice-hdr) are
	 * resolution-independent and stay checked. */
	if ((dph->flags & V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC) &&
	    (r_sps != 0x00346081u || r_pps90 != 0x00f1f7a0u || r_pps94 != 0x00000002u ||
	     r_84 != 0xae00e007u || r_top100 != 0u ||
	     (r_shdr_p & 0xffffe000u) != 0x0020e000u))
		mtk_v4l2_vdec_err(ctx,
			"mt8365 PACKER MISMATCH vs IDR: 88=%08x 90=%08x 94=%08x 9c=%08x 84=%08x t68=%08x t70=%08x t100=%08x Phi=%08x mbaff=%d fld=%d",
			r_sps, r_pps90, r_pps94, r_9c, r_84, r_top68, r_top70,
			r_top100, r_shdr_p & 0xffffe000u,
			eff_mbaff ? h264_mbaff : 0, eff_field_pic);

	/*
	 * Faithful replay of the reverse-engineered frame-0 IDR register sequence
	 * (decode-trace-full.log seq 0..456, IN ORDER), substituting the live DMA
	 * addresses for the trace's captured constants:
	 *   input MISC+0x160 = bs->dma_addr >> 8               (trace 0x0065ff10)
	 *   recon Y full 0xfe728000 / C full 0xff6bc000  ->  fb->base_y/c.dma_addr
	 *         (with the >>9 / >>8 / >>4 recon mirrors)
	 *   MV    MC+0xa20/0xa24 (trace 0xff150000/0xff160000) -> work_buf[0/1]
	 * Everything else is the captured constant for this 176x144 IDR clip. The
	 * prior partial / reordered programming never raised the dec-done IRQ; the
	 * per-field SPS/PPS/frame_num mapping off these constants is the next step.
	 */

	/*
	 * recon-mirror + MV/deblock work-buffer set (MISC+0x3c/0x40/0x54/0x58).
	 * The trace writes these at the START of frame 1+ (recon-mirrors-first
	 * order) but OMITS them in frame 0, which relies on the device's prior
	 * per-decode state. A fresh single-frame decode must set them itself, or
	 * the HW DMAs the deblock/MV scratch to an unset/zero address and hangs.
	 */
	/* N40: arm the write-VALUE log for this picture (see h264_wrlog). */
	wrlog_on = (h264_wrlog && wrlog_idx >= h264_wrlog_from &&
		    wrlog_idx < h264_wrlog_from + h264_wrlog);
	if (wrlog_on)
		pr_info("H264WR ---- picture %d begin (fld=%d bot=%d mbaff=%d styp=%u fn=%u %ux%u MB)\n",
			wrlog_idx, eff_field_pic, eff_bottom_field, eff_mbaff,
			eff_stype, dph->frame_num, mb_w, mb_h_pic);

	wr(ctx, VDEC_MISC, 0x3c, rec_y);
	wr(ctx, VDEC_MISC, 0x40, rec_c);
	wr(ctx, VDEC_MISC, 0x54, (u32)inst->work_buf[2].dma_addr);
	wr(ctx, VDEC_MISC, 0x58, (u32)inst->work_buf[3].dma_addr);

	/*
	 * seq 0-11: init / IRQ-config block + the VLD SOFT-RESET the trace never
	 * captured. regtrace hooked WriteSYS/MISC/VLDTOP/MC/AVCVLD/PP but NOT
	 * H264_HAL_WriteVLD, so every write to the common-VLD block (0x16020000 =
	 * VDEC_LD) was invisible -- including the vendor H264_HAL_ResetHW's VLD
	 * reset (poll VLD+0xf4 idle, assert VLD+0x108=0x101 around the MISC init,
	 * deassert VLD+0x108=0). Without it a VLD that powers up busy never clears
	 * AVCVLD+0x25c bit25, so its bitstream fetch wedges. reg_base[VDEC_LD] maps
	 * 0x16020000 (mt8365 DT reg index 2), so we can drive it directly.
	 */
	wr(ctx, VDEC_SYS,  0x00,  0x00000001);
	{
		int i;

		for (i = 0; i < 50000; i++) {
			u32 v = rd(ctx, VDEC_LD, 0xf4);

			if (!(v & 0x8000) && !(v & 0x1))
				break;
			cpu_relax();
		}
	}
	wr(ctx, VDEC_LD,   0x108, 0x00000101);	/* assert VLD soft-reset */
	wr(ctx, VDEC_MISC, 0x84,  0x00000002);
	wr(ctx, VDEC_MISC, 0x00,  0x00801ff0);
	wr(ctx, VDEC_MISC, 0xc8,  0x0000fff7);
	wr(ctx, VDEC_MISC, 0xcc,  0x51e30182);
	wr(ctx, VDEC_MISC, 0x178, 0xfffffe60);
	wr(ctx, VDEC_MISC, 0xf4,  0x00000000);
	wr(ctx, VDEC_MISC, 0x84,  0x00000002);
	wr(ctx, VDEC_LD,   0x108, 0x00000000);	/* deassert VLD soft-reset */
	wr(ctx, VDEC_SYS,  0x18,  0x00000001);
	wr(ctx, VDEC_MISC, 0xec,  0x00000001);
	wr(ctx, VDEC_MISC, 0x04,  0x00000010);
	wr(ctx, VDEC_CM,   0xb5c, 0x00000001);

	/* seq 12-16: input base, SPS-A (IDR), TOP enable, DPB cnt, TOP ctrl */
	wr(ctx, VDEC_MISC, 0x160, (u32)(bs->dma_addr >> 8));
	/* N40 bit 4: on a progressive picture the vendor's PRE-trigger write
	 * always has bit23 clear (see the r_84 comment). */
	wr(ctx, VDEC_AD,   0x84,
	   ((!eff_field_pic && (h264_progcfg & PROGCFG_84PRE)) ||
	    (eff_field_pic && (h264_fldcfg & FLDCFG_84PRE_CLR)))
		? (r_84 & ~(1u << 23)) : r_84);
	wr(ctx, VDEC_TOP,  0x54,  0x00000001);
	wr(ctx, VDEC_CM,   0x24,  0x00000006);
	wr(ctx, VDEC_TOP,  0x28,  0xff1e0000);

	/*
	 * seq 17-24: MC scalar + VLD status clear. The AVCVLD+0x28c strobes and
	 * the AVCVLD+0x228 arm from the trace belong to vpud's SOFTWARE
	 * barrel-shifter header parse (paired with VLD+0x8c fetch-start + VLD+0xe8
	 * poll -- which the stateless HW-decode path does NOT use); firing 0x28c /
	 * 0x228 in isolation starts a phantom VLD fetch that never completes, so
	 * AVCVLD+0x25c bit25 wedges busy and a VLD read hangs the AXI bus. Dropped.
	 */
	wr(ctx, VDEC_CM, 0x564, 0x00000000);
	/*
	 * Ref-list LOAD ACK (vendor InitPRefList @0x4a550): issue the load
	 * command AVCVLD+0x228=1, then poll bit0 until the HW has transferred the
	 * ref-list into its internal SRAM. Even an IDR needs this handshake --
	 * without it the ref/DPB SRAM is uninitialised and MB0 faults
	 * (AVCVLD+0x284 bit17, die-at-MB0). We dropped 0x228 in 00713 as a
	 * "phantom fetch", but that firing was pre-arm; in the ref-list context
	 * 0x228 is the load ack, so re-add it HERE (after the arm, before the
	 * fill) with the poll.
	 */
	{
		int j;

		wr(ctx, VDEC_AD, 0x228, 0x00000001);
		for (j = 0; j < 10000; j++) {
			if (!(rd(ctx, VDEC_AD, 0x228) & 0x1))
				break;
			cpu_relax();
		}
	}
	/*
	 * AVCVLD+0x0a0 InitPRefList config.
	 *
	 * PROGRESSIVE: this driver writes 0x10. The comment here used to call
	 * that "trace-verified (canonical + all working frame clips)" -- it is
	 * NOT. The matched 720x480 vendor progressive trace (trace-frm.log,
	 * cvmp_mot_frm0_full_B) writes 0x00008000 once per picture, and the N38
	 * MBAFF paragraph below states the same value; the two halves of this
	 * comment contradicted each other for several rounds. 0x10 is a
	 * bring-up constant that happens to decode 67 vectors bit-exactly, not a
	 * traced value. The vendor value is reachable behind h264_progcfg bit 0
	 * (N40) so the difference is measurable rather than assumed.
	 *
	 * FIELD (F1.6): the vendor writes
	 * 0x00010000 (bit16 = field ref-list mode) for EVERY field of the
	 * pure-field cvmp_mot_fld0 720x480 trace (trace-fld.log seq 65/797/1553...,
	 * one per field); with the progressive 0x10 the intra I-field stalls at
	 * MB5. Gated on eff_field_pic so frame-coded is byte-identical.
	 *
	 * N38 MBAFF: the vendor writes the SAME 0x00010000 for an MBAFF frame
	 * picture (trace-mbaff-cvma1 seq 64/802/1559/2316, one per picture) where
	 * the matched progressive trace writes 0x00008000 -- i.e. the MB-pair
	 * ref-list mode is the field mode. Both traces write this register, so the
	 * write-set presence diff could not see it; only the VALUE diff can.
	 */
	if (h264_picnum & PICNUM_MAXPIC) {
		/*
		 * N47: MaxPicNum per Sec 7.4.3 -- MaxFrameNum for a frame-coded
		 * picture, 2*MaxFrameNum for a field. Reproduces every traced
		 * vendor value (see the h264_picnum comment) including the
		 * 0x00010000 the field special-case below hardcodes.
		 */
		u32 max_fn = 1u << (sps->log2_max_frame_num_minus4 + 4);
		bool paff = !(sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY) &&
			    !eff_mbaff;
		u32 v = (eff_field_pic || (eff_mbaff && (h264_mbaff & MBAFF_A0)))
			? max_fn * 2u : max_fn;

		/* N56: the vendor's PAFF trace inverts this (see h264_paff). */
		if (paff && !eff_field_pic && (h264_paff & PAFF_A0))
			v = max_fn * 2u;
		if (paff && eff_field_pic && (h264_paff & PAFF_A0F))
			v = max_fn;
		wr(ctx, VDEC_AD, 0x0a0, v);
	} else {
		wr(ctx, VDEC_AD, 0x0a0,
		   (eff_field_pic || (eff_mbaff && (h264_mbaff & MBAFF_A0)))
			? 0x00010000
			: ((h264_progcfg & PROGCFG_A0) ? 0x00008000 : 0x00000010));
	}
	/*
	 * AVCVLD+0x0a4 = the CURRENT picture's PicNum.
	 *
	 * Progressive (vendor `trace-frm.log`, one write per picture): the raw
	 * frame_num -- 0,1,2,2,2,3,3,3,4,4,4,...
	 *
	 * FIELD (N39 write-VALUE diff of the matched vendor pair, 2026-08-03).
	 * `trace-fld.log` writes 1,1,3,3,5,5,5,5,5,5,7,7,... one per FIELD, i.e.
	 * **2*frame_num + 1** -- the H.264 field PicNum of the same-parity field
	 * (Sec. 8.2.4.1: PicNum = 2*FrameNumWrap + 1 for a same-parity reference,
	 * MaxPicNum = 2*MaxFrameNum). Both traces WRITE this register, so the
	 * presence-based write-set differential that produced the "field write-set
	 * identical" verdict structurally could not see it -- the same blind spot
	 * that hid the MBAFF TOP+0x68 MB-pair constant (N38).
	 *
	 * Gated on eff_field_pic, so every frame-coded picture (progressive /
	 * PAFF-frame / MBAFF) is byte-identical by construction.
	 */
	r_a4 = (eff_field_pic && (h264_fldpicnum & FLDPICNUM_CUR))
		? ((2u * dph->frame_num + 1u) & 0x7f)
		: (dph->frame_num & 0x7f);
	wr(ctx, VDEC_AD, 0x0a4, r_a4);
	/* N137: per-FIELD diagnostic, not an error -- gated with the rest. */
	if (eff_field_pic && unlikely(h264_diaglog))
		mtk_v4l2_vdec_err(ctx,
			"mt8365 FLDPICNUM fld=1 bot=%d styp=%d fn=%u a4=%02x nl0=%d rp=%u,%u,%u,%u",
			eff_bottom_field, eff_stype, dph->frame_num, r_a4, num_l0,
			ref_picnum[0], ref_picnum[1], ref_picnum[2], ref_picnum[3]);

	/* seq 25-410: ref-list Y/C arrays. An IDR has no references, so every slot
	 * ends pointing at the current recon frame: Y MC+0x3dc..0x558 (96 slots),
	 * C MC+0xb60..0xcdc (96 slots). Replicate the trace's TWO passes: first
	 * ZERO-FILL every slot, then recon-fill -- the HW may latch on the zero-init
	 * write stream even though the final state is identical either way. */
	for (i = 0; i < 96; i++) {
		wr(ctx, VDEC_CM, 0x3dc + 4 * i, 0x00000000);
		wr(ctx, VDEC_CM, 0xb60 + 4 * i, 0x00000000);
	}
	wr(ctx, VDEC_CM, 0x568, 0x00000000);
	wr(ctx, VDEC_CM, 0x56c, 0x00000000);
	/*
	 * Recon-fill. I/IDR: every slot = current recon (no references). P/B:
	 * the first num_l0 slots = the L0 reference recon (resolved from the V4L2
	 * DPB via the shared marshalling helpers -- update_dpb -> copy_decode_
	 * params -> fill_dpb_info gives inst->dpb[idx].y/c_dma_addr, full phys),
	 * the rest stay current recon (harmless, matches the vendor _Extra fill).
	 */
	/* is_inter / num_l0 / inst->dpb + the ordered refidx[] list are all derived
	 * up top in the adaptive block (from SLICE_PARAMS or the in-kernel parse). */
	{
		/*
		 * B (use_l1): load recon for ALL num_valid refs in PicNum-desc order
		 * (aidx) -- the forward AND backward references both must be resident,
		 * else bi-pred reads garbage (N15/B2 fill-count fix). P/I: unchanged,
		 * first num_l0 slots = L0 recon (refidx). Both fill the same MC+0x3dc/
		 * 0xb60 array in PicNum-desc order, consistent with group A.
		 */
		int n_rec = use_l1 ? n_valid : num_l0;
		const int *ridx = use_l1 ? aidx : refidx;

		/* N54: a B FIELD's region A is the FIELD-unit group-A list */
		if (fld_blist)
			n_rec = n_fa;

		for (i = 0; i < 96; i++) {
			u32 ry = rec_y, rc = rec_c;

			if (i < n_rec) {
				int di = fld_blist ? fa_di[i] : ridx[i];

				ry = (u32)inst->dpb[di].y_dma_addr;
				rc = (u32)inst->dpb[di].c_dma_addr;
			}
			wr(ctx, VDEC_CM, 0x3dc + 4 * i, ry);
			wr(ctx, VDEC_CM, 0xb60 + 4 * i, rc);
		}
	}

	/*
	 * B-slice L1 recon-array fill (N15/B3, 2026-07-18). DECISIVE trace finding:
	 * the HW reference-pixel recon array MC+0x3dc (Y) / 0xb60 (C) is NOT a single
	 * flat list -- it is partitioned into THREE 32-slot group REGIONS that mirror
	 * the AVCVLD/AVCMV descriptor groups: A=[slot 0..31], B=[32..63], C=[64..95].
	 * trace-b.log proves the vendor fills region B with spec RefPicList0 (l0idx)
	 * and region C with spec RefPicList1 (l1idx), each in list order, so a B
	 * frame's group-B/C descriptor resolves to region-B/C recon slots that hold
	 * the L0/L1 pixels. The recon-fill loop ABOVE fills ONLY region A (aidx,
	 * n_valid slots); it leaves regions B/C = current recon, so bi-pred L0/L1
	 * fetched the CURRENT frame as garbage. This is the confound behind the
	 * "byte-identical descriptors yet B fails" terminal digs: they compared
	 * descriptor slot-INDICES, never the region-B/C array CONTENTS (the dynamic
	 * recon addresses at slots 32+/64+ that the loop above never overwrote).
	 * Verified byte-exact on trace-b.log frame F9 (bi-pred B): descriptor picnum
	 * lists vldB=[5,3,4]->[f8,f5,f6]==reconY region B, vldC=[3,5,4]->[f5,f8,f6]==
	 * reconY region C. The HW reads the recon array by descriptor LIST-POSITION
	 * (dense), not by picnum value -- proven because P works today with region A
	 * filled dense-from-0. Gated on use_l1 so P/I stay byte-identical (P reads
	 * only region A -- proven by P decoding bit-exact with regions B/C left as
	 * current recon). Fill count + order mirror the group-B/C descriptor writes
	 * (l0idx/num_b0, l1idx/num_b1) programmed below.
	 */
	if (use_l1) {
		int s, nb0 = num_b0 > 16 ? 16 : num_b0;
		int nb1 = num_b1 > 16 ? 16 : num_b1;

		/* N54: field-unit L0/L1 for a B FIELD (same regions, more slots) */
		if (fld_blist) {
			nb0 = n_fb0;
			nb1 = n_fb1;
		}

		for (s = 0; s < nb0; s++) {
			int di = fld_blist ? fb0_di[s] : l0idx[s];

			wr(ctx, VDEC_CM, 0x3dc + 4 * (32 + s),
			   (u32)inst->dpb[di].y_dma_addr);
			wr(ctx, VDEC_CM, 0xb60 + 4 * (32 + s),
			   (u32)inst->dpb[di].c_dma_addr);
		}
		for (s = 0; s < nb1; s++) {
			int di = fld_blist ? fb1_di[s] : l1idx[s];

			wr(ctx, VDEC_CM, 0x3dc + 4 * (64 + s),
			   (u32)inst->dpb[di].y_dma_addr);
			wr(ctx, VDEC_CM, 0xb60 + 4 * (64 + s),
			   (u32)inst->dpb[di].c_dma_addr);
		}
	}

	/*
	 * AVC-MV colocated-MV base/POC table -- the other un-hooked block
	 * (VDEC_AV 0x16023000). The vendor InitPRefList/InitBRefList zero the
	 * ref-MV slots AVC_MV[0x000..0x17c] (96 words), SetPRefPicListReg_Extra
	 * sets the current frame's colMV base in AVC_MV[0x180..0x1fc] (32 words,
	 * >>4), and SetPOC writes AVC_MV+0x200 = POC (0 for an IDR). Missing this
	 * whole 0x000..0x200 region leaves a garbage MV-base/POC table -> MB0
	 * hard error (AVCVLD+0x284 bit17). For an IDR the per-ref loops don't run
	 * (empty DPB), so the table is just zeros + the current-frame colMV base.
	 */
	/*
	 * Resolve THIS frame-store's colMV pool slot (keyed by its recon-Y
	 * address). cur_mv is the buffer this frame writes its colocated MVs
	 * to (AVCMV+0x20c, below); when this frame is later a reference, the
	 * ref-list descriptor points +0x180 back at cur_mv and encodes cur_slot
	 * as the descriptor high field. Done for EVERY frame (I and P) so an IDR
	 * registers its slot/colMV before any P references it.
	 */
	cur_slot = mt8365_mv_assign(inst, rec_y, eff_field_pic);
	cur_mv = (u32)(inst->mv_pool[cur_slot].dma_addr >> 4);
	/*
	 * N59: record whether THIS picture is field-coded, against the slot that
	 * identifies its frame-store, so a later picture referencing it can emit
	 * the vendor's bit19 correctly (PAFF_B19FLD). Written unconditionally --
	 * the flag must be fresh whenever a slot is (re)assigned -- and read only
	 * under the arm, so this is inert at built-in defaults.
	 */
	inst->mv_pool_fld[cur_slot] = eff_field_pic ? 1 : 0;

	for (i = 0; i < 96; i++)
		wr(ctx, VDEC_AV, 0x000 + 4 * i, 0x00000000);
	/* init-fill the colMV base array with THIS frame's buffer (vendor
	 * SetPRefPicListReg_Extra fills 32 words with the current colMV base);
	 * active ref slots are overwritten per-reference below. */
	for (i = 0; i < 32; i++)
		wr(ctx, VDEC_AV, 0x180 + 4 * i, cur_mv);
	/*
	 * current-frame POC descriptor (vendor writes +0x200/+0x204/+0x208).
	 *
	 * N61 FLDPOC: on a FIELD-coded picture this register is the CURRENT
	 * FIELD's own POC, not the frame's top-field POC. Established by a
	 * value-compare of our own h264_wrlog stream against the aligned vendor
	 * trace ~/bftrace/trace-fld.log on the clip it was captured from
	 * (cavlc_mot_fld0_full_B, md5 8d3f81c79f5b722c1015d3e880e2304d): the
	 * vendor emits exactly 60 W AVCMV+0x200 for 60 coded pictures, each one
	 * immediately BEFORE that picture's W AVCVLD+0x98 config word, and the
	 * value equals that field's spec POC on 60/60 pictures -- including every
	 * bottom field (pic1 -> 1, pic3 -> 7, pic5 -> 3, pic7 -> 5). Ours writes
	 * dph->top_field_order_cnt unconditionally, so every BOTTOM field carries
	 * its complementary TOP field's POC (pic5 ours 0x2 vs vendor 0x3, pic7
	 * ours 0x4 vs vendor 0x5) -- the first divergence being the bottom field
	 * of the clip's FIRST B field pair, which is also the first broken
	 * output frame.
	 *
	 * This anchor is the current picture's POC for the temporal-direct
	 * distance derivation, so a wrong value can only bite where colMV
	 * scaling is consumed: B pictures, masked when the co-located picture is
	 * intra -- exactly the N61 predicate's shape. Frame-coded pictures are
	 * untouched (eff_field_pic == 0), so this is inert on progressive
	 * streams by construction.
	 */
	if ((h264_fldpoc & FLDPOC_CUR) && eff_field_pic) {
		u32 fpoc = (u32)(eff_bottom_field ? dph->bottom_field_order_cnt
						  : dph->top_field_order_cnt);

		wr(ctx, VDEC_AV, 0x200, fpoc & 0x3ffff);
		wr(ctx, VDEC_AV, 0x204, fpoc & 0x3ffff);
		wr(ctx, VDEC_AV, 0x208, fpoc & 0x3ffff);
	} else {
		wr(ctx, VDEC_AV, 0x200, (u32)dph->top_field_order_cnt & 0x3ffff);
		wr(ctx, VDEC_AV, 0x204, (u32)dph->top_field_order_cnt & 0x3ffff);
		wr(ctx, VDEC_AV, 0x208, (u32)dph->top_field_order_cnt & 0x3ffff);
	}

	/*
	 * SLICE-BASED multi-reference activation (2026-07-10, from the vendor
	 * vpud read-trace vendor-readtrace-fn-ge2-20260710.log). For every P
	 * frame the vendor programs THREE parallel ref-list index groups in BOTH
	 * the AVCVLD block (ref PicNum) AND the AVCMV block (per-ref colMV/POC
	 * descriptor), plus AVCVLD+0xa4=frame_num (above) and a commit strobe
	 * AVCVLD+0x22c=1 (fired below, after the SPS/config writes, in vendor
	 * order). The prior frame-based attempts wrote ONLY the AVCVLD PicNum
	 * groups (kernel 00824 -- INERT) and left the AVCMV descriptor tables
	 * zeroed (the audit had dismissed them as "B-slice-only"); the trace
	 * shows the vendor writes them for P too, so they are the untested half
	 * of the ref-list activation.
	 *
	 * CORRECTED descriptor VALUE encoding (2026-07-10 re-derivation from the
	 * fn=1/2/3/4/9 trace, verified against the per-frame recon+colMV buffer
	 * rotation). The prior encoding used ref_frame_num directly in the high
	 * field; that only coincides with the real field for fn<5 and corrupted
	 * even single-ref f1. The high field is the reference's colMV-buffer
	 * RING-SLOT index (its physical frame-store slot), NOT frame_num --
	 * decisive evidence: fn=9 references f8/f7/f6 (pic_num 8/7/6) yet the
	 * vendor descriptor encodes slots 3/2/1 (= their colMV ring slots
	 * 8%5/7%5/6%5), exactly matching the +0x180 colMV addresses those refs
	 * were decoded into. Our pool assigns a stable slot per frame-store
	 * (mt8365_mv_slot), so cur_slot at each frame's decode == the slot we
	 * later encode when it is a reference.
	 *   AVCVLD group{A=+0xa8,B=+0x128,C=+0x1a8}[slot,+4] = ref pic_num
	 *   AVCMV  group{A=+0x000,B=+0x080,C=+0x100}[slot,+8]:
	 *       high  = 0x00200000 | (ref_colmv_slot << 23)
	 *       word0 = high | (group==A ? 0 : (ref_poc & 0xffff))  (A low16=0)
	 *       word1 = word0 | 0x00400000
	 *   AVCMV colMV address array +0x180[slot,+8]:
	 *       +0x180+8s = ref colMV buffer (phys>>4); +0x184+8s = that | 2
	 *   group C: first two slots swapped (AVCVLD picnum + AVCMV descriptor +
	 *   the +0x180 colMV addresses), vendor SetPRefPicListReg @seq 2125/2137.
	 *   Slots nearest-ref first (ref_pic_list0 order).
	 */
	if (is_inter && fld_blist) {
		/*
		 * N54 B FIELD path. Same three groups, same FIELD descriptor
		 * encoding as the N51 P field path (one 32-bit AVCMV word per
		 * FIELD entry, bit19 = field, bit22 = the reference field's
		 * parity, slot<<23; group A low16 = 0, groups B/C low16 = the
		 * reference FIELD's POC), but the three groups now carry THREE
		 * different lists: A = field PicNum-desc, B = field RefPicList0,
		 * C = field RefPicList1. Group C is NOT the swap-first-two of
		 * group A here -- that identity only holds for P, where
		 * RefPicList1 is defined as swap-first-two(RefPicList0); for B
		 * the spec builds L1 independently (Sec 8.2.4.2.4), which is
		 * exactly what the vendor trace shows (group C = [3,2,1,0] while
		 * group B = [1,0,3,2] on the first B field of
		 * cavlc_mot_fld0_full_B).
		 */
		int s;

#define MT8365_PROG_FGROUP(_vld, _mv, _di, _bot, _pn, _n, _apoc) do {		\
		int _s;								\
		for (_s = 0; _s < (_n); _s++) {					\
			int _d = (_di)[_s];					\
			u32 _ry = (u32)inst->dpb[_d].y_dma_addr;		\
			int _sl = mt8365_mv_lookup(inst, _ry);			\
			u32 _lt = (v4l2_dpb[_d].flags &				\
				   V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM) ? 1u : 0u; \
			u32 _pnv = _lt ? v4l2_dpb[_d].pic_num			\
				       : (u32)(_pn)[_s];			\
			u32 _pc = (_bot)[_s]					\
				? (u32)v4l2_dpb[_d].bottom_field_order_cnt	\
				: (u32)v4l2_dpb[_d].top_field_order_cnt;	\
			u32 _fc = inst->mv_pool_fld[_sl &			\
					(MT8365_MV_POOL_NUM - 1)] ? 1u : 0u;	\
			u32 _b19 = ((h264_paff & PAFF_B19FLD) && !_fc)		\
				   ? 0u : 0x00080000u;			\
			u32 _hi;						\
										\
			/*							\
			 * N128: descriptor bits[19:18] on the FIELD form.	\
			 * A frame-coded reference of an MBAFF sequence is	\
			 * the vendor's `01`, not `00`. Candidate counter	\
			 * unconditional; ASSIGN rather than OR so the two	\
			 * bits can never both be set (`11` is not a mode).	\
			 */							\
			if (!_fc &&						\
			    !(sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY) && \
			    (sps->flags &					\
			     V4L2_H264_SPS_FLAG_MB_ADAPTIVE_FRAME_FIELD)) {	\
				h264_mbaffreffcand++;				\
				if (h264_mbaffref & MBAFFREF_BFLD) {		\
					_b19 = 0x00040000u;			\
					h264_mbaffreffhit++;			\
				}						\
			}							\
			_hi = _b19 | (_lt << 20) |				\
				  ((_bot)[_s] ? 0x00400000u : 0u) |		\
				  ((u32)(_sl & 0x1ff) << 23);			\
										\
			wr(ctx, VDEC_AD, (_vld) + 4 * _s,			\
			   (_pnv & 0x7ffff) | (_lt << 19));			\
			wr(ctx, VDEC_AV, (_mv) + 4 * _s,			\
			   (_apoc) ? (_hi | (_pc & 0xffff)) : _hi);		\
		}								\
	} while (0)

		MT8365_PROG_FGROUP(0x0a8, 0x000, fa_di, fa_bot, fa_pn, n_fa, false);
		MT8365_PROG_FGROUP(0x128, 0x080, fb0_di, fb0_bot, fb0_pn, n_fb0, true);
		MT8365_PROG_FGROUP(0x1a8, 0x100, fb1_di, fb1_bot, fb1_pn, n_fb1, true);
#undef MT8365_PROG_FGROUP

		/*
		 * colMV read-address array, in RefPicList1 order (the temporal-
		 * direct co-located picture is RefPicList1[0], Sec 8.4.1.2.1) --
		 * the field-stride form of the frame B path's +0x180 fill, with
		 * the parity bit the N51 field path carries.
		 */
		for (s = 0; s < n_fb1; s++) {
			int di = fb1_di[s];
			u32 ry = (u32)inst->dpb[di].y_dma_addr;
			int sl = mt8365_mv_lookup(inst, ry);
			u32 rmv = (u32)(inst->mv_pool[sl].dma_addr >> 4);

			/*
			 * N120: slot 0 IS the co-located picture (spec
			 * 8.4.1.2.1). Poison its colMV records HERE -- the
			 * read side, after every store, before the trigger.
			 */
			if (s == 0)
				mt8365_mv_poison(inst, sl);
			wr(ctx, VDEC_AV, 0x180 + 4 * s,
			   rmv | (fb1_bot[s] ? mt8365_mv_k(MVK_FLDREAD) : 0u));
		}

		if (h264_fldstruct) {
			/*
			 * N52/N53 picture-structure words, one per GROUP: the
			 * packed 2-bit-per-PAIR array over that group's own list
			 * (2 = top field, 1 = bottom field, bit position 2*k).
			 * With the interleave putting a same-parity field at
			 * every even index all three collapse to the same value
			 * -- which is what the vendor writes (0x0a on the first
			 * B field of cavlc_mot_fld0_full_B) -- but they are
			 * computed per group so a list that ever diverges stays
			 * correct.
			 */
			u32 sA = 0, sB = 0, sC = 0;
			u32 stA[8], stB[8], stC[8];
			int nA = 0, nB = 0, nC = 0;
			int k;

			if (h264_fldmaskb) {
				/*
				 * N107: the exact per-SLOT bottom-field bitmap,
				 * one bit per list entry (bit s == bit22 of that
				 * slot's group descriptor), taken per GROUP over
				 * that group's own list. No group-C swap here --
				 * for B pictures group C is an independently
				 * built RefPicList1, not swap-first-two(L0).
				 */
				for (k = 0; k < n_fa && k < 32; k++) {
					if (fa_bot[k])
						sA |= 1u << k;
					if (nA < (int)ARRAY_SIZE(stA))
						stA[nA++] = sA;
				}
				for (k = 0; k < n_fb0 && k < 32; k++) {
					if (fb0_bot[k])
						sB |= 1u << k;
					if (nB < (int)ARRAY_SIZE(stB))
						stB[nB++] = sB;
				}
				for (k = 0; k < n_fb1 && k < 32; k++) {
					if (fb1_bot[k])
						sC |= 1u << k;
					if (nC < (int)ARRAY_SIZE(stC))
						stC[nC++] = sC;
				}
			} else {
				for (k = 0; 2 * k + 1 < n_fa; k++) {
					sA |= (fa_bot[2 * k] ? 1u : 2u) << (2 * k);
					if (nA < (int)ARRAY_SIZE(stA))
						stA[nA++] = sA;
				}
				for (k = 0; 2 * k + 1 < n_fb0; k++) {
					sB |= (fb0_bot[2 * k] ? 1u : 2u) << (2 * k);
					if (nB < (int)ARRAY_SIZE(stB))
						stB[nB++] = sB;
				}
				for (k = 0; 2 * k + 1 < n_fb1; k++) {
					sC |= (fb1_bot[2 * k] ? 1u : 2u) << (2 * k);
					if (nC < (int)ARRAY_SIZE(stC))
						stC[nC++] = sC;
				}
			}

			/* N60: one write (default) or the vendor's RMW walk. */
			mt8365_fldstruct_emit(ctx, 0x564, stA, nA, sA);
			mt8365_fldstruct_emit(ctx, 0x568, stB, nB, sB);
			mt8365_fldstruct_emit(ctx, 0x56c, stC, nC, sC);
		}
	} else if (is_inter && num_l0 > 0 && !use_l1) {
		/*
		 * P/SP path -- UNCHANGED from the pre-N15 driver (byte-identical).
		 * Groups A and B both carry the L0 (PicNum-desc) list; group C is the
		 * L0 list with the first two slots swapped. Descriptor low16 = 0 for
		 * group A, = ref POC for groups B/C.
		 */
		u32 gpv[16], hi[16], mdA[16], mdBC[16], m1BC[16], rmv[16];
		int s, n = num_l0 > 16 ? 16 : num_l0;

		for (s = 0; s < n; s++) {
			int di = refidx[s];
			u32 rpicnum = ref_picnum[s];
			u32 rpoc = ref_poc[s];
			u32 ry = (u32)inst->dpb[di].y_dma_addr;
			int rslot = mt8365_mv_lookup(inst, ry);
			/*
			 * N27 long-term marking, transcribed from
			 * H264_HAL_SetPRefPicListReg @libvpud_vcodec.so+0x4a714
			 * (mt8365 aarch64). The descriptor struct field [8] is the
			 * ref's marking class; the vendor computes
			 *   w24 = (marking > 3)          <- long-term
			 *   w27 = w24 << 19
			 *   AVCVLD +0xa8+4*slot = (picnum & 0x7ffff) + w27
			 *   AVCMV  word        |= w24 << 20
			 * and selects a DIFFERENT picnum field for marking > 3 (the
			 * LongTermPicNum). So: AVCVLD word bit19 and AVCMV descriptor
			 * bit20 are the long-term flags, and the AVCVLD value is
			 * LongTermPicNum -- which the V4L2 uAPI already delivers in
			 * dpb[].pic_num for LONG_TERM entries.
			 */
			u32 lt = (inst->decode_params.dpb[di].flags &
				  V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM) ? 1u : 0u;
			u32 fld_ref = inst->mv_pool_fld[rslot &
					(MT8365_MV_POOL_NUM - 1)] ? 1u : 0u;
			u32 p19 = 0u;
			u32 mbaff18 = 0u;	/* N125 descriptor bit18 */
			u32 fb19 = 0u;		/* N128 field-form bits[19:18] */
			bool fb19_set = false;

			gpv[s] = (rpicnum & 0x7ffff) | (lt << 19);
			/*
			 * N51: the FIELD descriptor encoding, transcribed from
			 * the vendor field trace (~/bftrace/trace-fld.log, the
			 * P-bottom session): a FIELD reference is one 32-bit word
			 * with bit19 set (where a FRAME reference sets bit21) and
			 * bit22 = the reference field's PARITY, e.g. group B =
			 * [0x00480001, 0x00880006, 0x00080000] for the field list
			 * [i0.bot pn1 poc1 slot0, i1.top pn2 poc6 slot1,
			 *  i0.top pn0 poc0 slot0]. A FRAME reference occupies TWO
			 * consecutive words (bit22=0 then bit22=1, i.e. its top
			 * and bottom field), which is the pre-existing 8-byte
			 * stride; a FIELD reference occupies ONE, so the AVCMV
			 * stride is 4 bytes on the field path.
			 */
			/*
			 * N61 PAFF_FRMP19: the frame-form analogue of the arm
			 * built on the B path -- bit19 IN ADDITION TO bit21,
			 * same two-word stride, when the reference's frame-store
			 * was written by a field-coded picture. Candidate count
			 * runs unconditionally; the frame P pictures of a PAFF
			 * stream are already bit-exact, so this is a separable
			 * arm, not the predicted one.
			 */
			if (!fld_list && fld_ref) {
				h264_b19frm++;
				if (h264_paff & PAFF_FRMP19) {
					p19 = 0x00080000u;
					h264_b19hit++;
				}
			}

			/*
			 * N125: the vendor's descriptor bit18 -- "this reference
			 * was coded MBAFF". It rides the FRAME form only: a field
			 * picture takes the (struct != frame) << 19 branch and
			 * never gets the addend (vendor 0x3f080 vs 0x3fc84). A
			 * frame-store written by two FIELD pictures is not MBAFF
			 * either, hence !fld_ref. Candidate counter unconditional.
			 */
			if (!fld_list && !fld_ref &&
			    !(sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY) &&
			    (sps->flags & V4L2_H264_SPS_FLAG_MB_ADAPTIVE_FRAME_FIELD)) {
				h264_mbaffrefcand++;
				if (h264_mbaffref & MBAFFREF_DESC) {
					mbaff18 = 0x00040000u;
					h264_mbaffrefhit++;
				}
			}

			/*
			 * N128: the P-FIELD-list twin of the arm above. Same
			 * predicate, same ASSIGN discipline; separate bit so
			 * the B field list can be armed on its own.
			 */
			if (fld_list && !fld_ref &&
			    !(sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY) &&
			    (sps->flags & V4L2_H264_SPS_FLAG_MB_ADAPTIVE_FRAME_FIELD)) {
				h264_mbaffreffcand++;
				if (h264_mbaffref & MBAFFREF_PFLD) {
					fb19 = 0x00040000u;
					fb19_set = true;
					h264_mbaffreffhit++;
				}
			}

			hi[s] = fld_list
				? ((fb19_set ? fb19
				    : (((h264_paff & PAFF_B19FLD) &&
				     !inst->mv_pool_fld[rslot & (MT8365_MV_POOL_NUM - 1)])
				    ? 0u : 0x00080000u)) | (lt << 20) |
				   (ref_bot[s] ? 0x00400000u : 0u) |
				   ((u32)(rslot & 0x1ff) << 23))
				: (0x00200000u | p19 | mbaff18 | (lt << 20) |
				   ((u32)(rslot & 0x1ff) << 23));
			mdA[s] = hi[s];			/* group A low16 = 0 */
			mdBC[s] = hi[s] | (rpoc & 0xffff);	/* B/C low16 = POC */
			m1BC[s] = mdBC[s] | 0x00400000u;
			/*
			 * N65: on the FRAME-form path the bottom-parity word
			 * carries the reference's OWN bottom POC (the vendor's
			 * encoding, 3154/3154 descriptor words across two
			 * identified traces). Candidate counter unconditional.
			 */
			if (!fld_list) {
				u32 rpocb = (u32)inst->decode_params.dpb[di]
						.bottom_field_order_cnt;

				if (rpocb != rpoc) {
					h264_refpoccand++;
					if (h264_refpoc & REFPOC_FRMBOT) {
						m1BC[s] = hi[s] | 0x00400000u |
							  (rpocb & 0xffff);
						h264_refpochit++;
					}
				}
			}
			rmv[s] = (u32)(inst->mv_pool[rslot].dma_addr >> 4);
		}
		for (s = 0; s < n; s++) {
			wr(ctx, VDEC_AD, 0x0a8 + 4 * s, gpv[s]);
			wr(ctx, VDEC_AD, 0x128 + 4 * s, gpv[s]);
			wr(ctx, VDEC_AD, 0x1a8 + 4 * s, gpv[s]);
			if (fld_list) {
				/* N51: one AVCMV word per FIELD entry */
				u32 pt = ref_bot[s]
					 ? mt8365_mv_k(MVK_FLDREAD) : 0u;

				wr(ctx, VDEC_AV, 0x000 + 4 * s, mdA[s]);
				wr(ctx, VDEC_AV, 0x080 + 4 * s, mdBC[s]);
				wr(ctx, VDEC_AV, 0x100 + 4 * s, mdBC[s]);
				wr(ctx, VDEC_AV, 0x180 + 4 * s, rmv[s] | pt);
				continue;
			}
			wr(ctx, VDEC_AV, 0x000 + 8 * s, mdA[s]);
			wr(ctx, VDEC_AV, 0x004 + 8 * s, mdA[s] | 0x00400000u);
			wr(ctx, VDEC_AV, 0x080 + 8 * s, mdBC[s]);
			wr(ctx, VDEC_AV, 0x084 + 8 * s, m1BC[s]);
			wr(ctx, VDEC_AV, 0x100 + 8 * s, mdBC[s]);
			wr(ctx, VDEC_AV, 0x104 + 8 * s, m1BC[s]);
			wr(ctx, VDEC_AV, 0x180 + 8 * s, rmv[s]);
			wr(ctx, VDEC_AV, 0x184 + 8 * s,
			   rmv[s] | mt8365_mv_k(MVK_FRMREAD));
		}
		if (n >= 2 && fld_list) {
			/* group C + colMV array: swap the first two FIELD slots */
			u32 p0 = ref_bot[0] ? mt8365_mv_k(MVK_FLDREAD) : 0u;
			u32 p1 = ref_bot[1] ? mt8365_mv_k(MVK_FLDREAD) : 0u;

			wr(ctx, VDEC_AD, 0x1a8, gpv[1]);
			wr(ctx, VDEC_AD, 0x1ac, gpv[0]);
			wr(ctx, VDEC_AV, 0x100, mdBC[1]);
			wr(ctx, VDEC_AV, 0x104, mdBC[0]);
			wr(ctx, VDEC_AV, 0x180, rmv[1] | p1);
			wr(ctx, VDEC_AV, 0x184, rmv[0] | p0);
		}
		if (fld_list && h264_fldstruct) {
			/*
			 * N52: per-group reference picture-structure registers
			 * (2 = top field, 1 = bottom field). Group A and B lead
			 * with list entry 0; group C leads with entry 1 because
			 * of the swap above. See the h264_fldstruct comment.
			 */
			u32 sA = ref_bot[0] ? 1u : 2u;
			u32 sC = (n >= 2) ? (ref_bot[1] ? 1u : 2u) : sA;
			u32 stA[8];
			int nst = 0;

			if (h264_fldmask) {
				/*
				 * N106: the register is a plain ONE-BIT-PER-SLOT
				 * bottom-field bitmap over the 32-slot recon
				 * region, bit s == bit22 of that slot's group
				 * descriptor. Measured on the identified vendor
				 * PAFF trace: 288 of 288 field-picture region
				 * masks reproduce exactly, and the mask is
				 * written 0 on every frame picture (0 non-zero
				 * frame writes). Group C carries the same bitmap
				 * with slots 0/1 swapped, matching the group-C
				 * descriptor swap below -- the vendor's own
				 * +0x56c re-write (0x2ab -> 0x2a9) is that swap.
				 *
				 * The N53 packed-per-PAIR form this replaces is
				 * numerically identical whenever the list parity
				 * alternates in pairs AND n is EVEN, which is why
				 * it survived until now. It is WRONG when n is
				 * ODD -- its loop runs while 2k+1 < n, so the
				 * final entry never contributes a bit -- and when
				 * a Sec 8.2.4.3.1 reorder (N104) breaks the
				 * alternation, because it derives the odd-slot
				 * bit from ref_bot[2k] instead of ref_bot[2k+1].
				 */
				int b;

				sA = 0;
				for (b = 0; b < n && b < 32; b++) {
					if (ref_bot[b])
						sA |= 1u << b;
					if (nst < (int)ARRAY_SIZE(stA))
						stA[nst++] = sA;
				}
				sC = sA;
				if (n >= 2)
					sC = (sA & ~3u) |
					     (ref_bot[1] ? 1u : 0u) |
					     (ref_bot[0] ? 2u : 0u);
			} else if (h264_fldpack) {
				/*
				 * N53: the registers are a packed 2-bit-per-PAIR
				 * array, one field per top/bottom pair, bit
				 * position 2*k. Identical to the scalar above
				 * for n <= 3 (a single pair). See h264_fldpack.
				 */
				int k;

				sA = 0;
				for (k = 0; 2 * k + 1 < n; k++) {
					sA |= (ref_bot[2 * k] ? 1u : 2u) << (2 * k);
					if (nst < (int)ARRAY_SIZE(stA))
						stA[nst++] = sA;
				}
				sC = (n >= 2)
					? ((sA & ~3u) | (ref_bot[1] ? 1u : 2u))
					: sA;
			}

			/* N60: one write (default) or the vendor's RMW walk. */
			mt8365_fldstruct_emit(ctx, 0x564, stA, nst, sA);
			mt8365_fldstruct_emit(ctx, 0x568, stA, nst, sA);
			mt8365_fldstruct_emit(ctx, 0x56c, stA, nst, sC);
		}
		if (n >= 2 && !fld_list) {	/* group C + colMV array: swap the first two slots */
			wr(ctx, VDEC_AD, 0x1a8, gpv[1]);
			wr(ctx, VDEC_AD, 0x1ac, gpv[0]);
			wr(ctx, VDEC_AV, 0x100, mdBC[1]);
			wr(ctx, VDEC_AV, 0x104, m1BC[1]);
			wr(ctx, VDEC_AV, 0x108, mdBC[0]);
			wr(ctx, VDEC_AV, 0x10c, m1BC[0]);
			wr(ctx, VDEC_AV, 0x180, rmv[1]);
			wr(ctx, VDEC_AV, 0x184, rmv[1] | mt8365_mv_k(MVK_FRMREAD));
			wr(ctx, VDEC_AV, 0x188, rmv[0]);
			wr(ctx, VDEC_AV, 0x18c, rmv[0] | mt8365_mv_k(MVK_FRMREAD));
		}
	} else if (is_inter && use_l1) {
		/*
		 * B-slice path (N15/B2). Group A = raw PicNum-desc over ALL num_valid
		 * refs (aidx). Group B = spec RefPicList0 (l0idx). Group C = spec
		 * RefPicList1 (l1idx). All are filled to num_valid (the fill-count fix:
		 * the vendor trace fills every group to num_valid, not num_l0). All
		 * three index the reshuffled DPB (inst->decode_params.dpb / inst->dpb /
		 * mv_pool), the SAME index space the v4l2_h264 reflist builder emits,
		 * so pic_num / POC / recon-Y / colMV all resolve consistently. The
		 * descriptor VALUE encoding is the trace-verified L0 format reused for
		 * L1 (0x00200000 | colMV_slot<<23 | (POC in low16 for B/C, 0 for A)).
		 */
		int s;

#define MT8365_PROG_GROUP(_vld, _mv, _idx, _n, _apoc) do {			\
		int _s;								\
		for (_s = 0; _s < (_n); _s++) {					\
			int _di = (_idx)[_s];					\
			u32 _ry = (u32)inst->dpb[_di].y_dma_addr;		\
			int _sl = mt8365_mv_lookup(inst, _ry);			\
			u32 _pn = inst->decode_params.dpb[_di].pic_num;		\
			u32 _pc = (u32)inst->decode_params.dpb[_di]		\
					.top_field_order_cnt;			\
			u32 _lt = (inst->decode_params.dpb[_di].flags &		\
				   V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM) ? 1u : 0u; \
			u32 _fc = inst->mv_pool_fld[_sl &			\
					(MT8365_MV_POOL_NUM - 1)] ? 1u : 0u;	\
			u32 _pcb = (u32)inst->decode_params.dpb[_di]		\
					.bottom_field_order_cnt;		\
			u32 _b19 = 0u;						\
			u32 _hi;						\
			u32 _w0;						\
			u32 _w1;						\
										\
			/* N61 PAFF_FRMB19: keep bit21 + the two-word FRAME	\
			 * stride, add bit19 when the reference was FIELD-coded.	\
			 * The candidate counter runs unconditionally. */	\
			if (_fc) {						\
				h264_b19frm++;					\
				if (h264_paff & PAFF_FRMB19) {			\
					_b19 = 0x00080000u;			\
					h264_b19hit++;				\
				}						\
			}							\
			_hi = 0x00200000u | _b19 | (_lt << 20) |		\
				  ((u32)(_sl & 0x1ff) << 23);			\
			_w0 = (_apoc) ? (_hi | (_pc & 0xffff)) : _hi;		\
			_w1 = _w0 | 0x00400000u;				\
			/* N65: the bottom-parity word carries the reference's	\
			 * OWN bottom POC. Candidate counter runs		\
			 * unconditionally. */					\
			if ((_apoc) && _pcb != _pc) {				\
				h264_refpoccand++;				\
				if (h264_refpoc & REFPOC_FRMBOT) {		\
					_w1 = _hi | 0x00400000u |		\
					      (_pcb & 0xffff);			\
					h264_refpochit++;			\
				}						\
			}							\
			wr(ctx, VDEC_AD, (_vld) + 4 * _s,			\
			   (_pn & 0x7ffff) | (_lt << 19));			\
			wr(ctx, VDEC_AV, (_mv) + 8 * _s, _w0);			\
			wr(ctx, VDEC_AV, (_mv) + 8 * _s + 4, _w1);		\
		}								\
	} while (0)

		MT8365_PROG_GROUP(0x0a8, 0x000, aidx,  n_valid > 16 ? 16 : n_valid, false);
		MT8365_PROG_GROUP(0x128, 0x080, l0idx, num_b0 > 16 ? 16 : num_b0, true);
		MT8365_PROG_GROUP(0x1a8, 0x100, l1idx, num_b1 > 16 ? 16 : num_b1, true);
#undef MT8365_PROG_GROUP

		/*
		 * +0x180 colMV read-address array -- in spec RefPicList1 (group-C /
		 * l1idx) order, NOT group-A (PicNum-desc). This is the TEMPORAL-DIRECT
		 * co-located colMV read path: for a temporal-direct B slice the HW reads
		 * the co-located field from +0x180 slot 0, which MUST be RefPicList1[0]
		 * (the co-located picture, spec 8.4.1.2.1). A live vendor `vpud`
		 * read-trace of the failing `dtemporal` clip (trace-dtemp.log) proves the
		 * vendor fills +0x180 in group-C order: e.g. B@POC4 refs {I0,P6,B2} ->
		 * vendor +0x180 = [P6, B2, I0] = RefPicList1, with the co-located P6 at
		 * slot 0. The prior group-A (PicNum-desc) fill only coincided with
		 * RefPicList1 when the co-located ref had the highest PicNum (the 1st B
		 * of a pair, 2 refs) -- so those decoded bit-exact -- but the 2nd B of a
		 * `bframes=2` pair has a 3rd ref (the just-decoded 1st B) with a HIGHER
		 * PicNum than the co-located P, so PicNum-desc wrongly put THAT at slot 0
		 * and the HW read the wrong frame's colMV. Invisible under uniform motion
		 * (every MB's colMV identical) but fatal for a spatially-varied field at
		 * scale -- exactly the observed `dtemporal` failure set (the 2nd B of
		 * each pair: n:3,6,9,12,15,18,21,24,28). Fill in l1idx order to num_b1;
		 * the earlier init-fill (all 32 = cur_mv) covers unused slots.
		 * direct=none / explicit-bi-pred / spatial-direct B do NOT consume this
		 * read path (they use L0/L1 recon, not co-located colMV), so this is
		 * correctness-neutral for them (non-regression verified on-silicon).
		 */
		for (s = 0; s < (num_b1 > 16 ? 16 : num_b1); s++) {
			int di = l1idx[s];
			u32 ry = (u32)inst->dpb[di].y_dma_addr;
			int sl = mt8365_mv_lookup(inst, ry);
			u32 rmv = (u32)(inst->mv_pool[sl].dma_addr >> 4);

			/*
			 * N83: slot 0 of this array IS the co-located picture
			 * (spec 8.4.1.2.1). Inspect its colMV buffer HERE --
			 * the read side, before the trigger.
			 */
			if (s == 0)
				mt8365_mvdump_snap_read(inst, mvdump_idx, ry,
							sl);
			wr(ctx, VDEC_AV, 0x180 + 8 * s, rmv);
			wr(ctx, VDEC_AV, 0x184 + 8 * s,
			   rmv | mt8365_mv_k(MVK_FRMREAD));
		}

		/*
		 * BDIAG (throwaway, N16/B4): dump our decode-order ring-slot
		 * assignment + the L1 (RefPicList1) order for every B frame, so it
		 * can be diffed against the vendor trace-cvbs3 FRAME n. Compare the
		 * per-ref pic_num/POC (rule out reflist-order = hypothesis B) and the
		 * slot (hypothesis A: does our monotonic ring reproduce the vendor's
		 * I0=0/P3=1/P6=2/P9=5/P12=8/... slots?).
		 */
		{
			int di0 = num_b1 > 0 ? l1idx[0] : -1;
			int di1 = num_b1 > 1 ? l1idx[1] : -1;
			int di2 = num_b1 > 2 ? l1idx[2] : -1;
			int di3 = num_b1 > 3 ? l1idx[3] : -1;
#define B4_SL(di) ((di) < 0 ? -1 : mt8365_mv_lookup(inst, (u32)inst->dpb[(di)].y_dma_addr))
#define B4_PC(di) ((di) < 0 ? -1 : (int)(inst->decode_params.dpb[(di)].top_field_order_cnt))
#define B4_PN(di) ((di) < 0 ? 0u : (u32)inst->decode_params.dpb[(di)].pic_num)
			/*
			 * N137: the four slot lookups stay UNCONDITIONAL --
			 * mt8365_mv_lookup() bumps the h264_mvpoolcand
			 * instrument denominator, and an instrument whose
			 * denominator moves when a LOG is silenced is not an
			 * instrument.  Only the printk is gated.
			 */
			int sl0 = B4_SL(di0), sl1 = B4_SL(di1);
			int sl2 = B4_SL(di2), sl3 = B4_SL(di3);

			if (unlikely(h264_diaglog))
				mtk_v4l2_vdec_err(ctx,
					"B4REF poc=%d nb0=%d nb1=%d curslot=%d L1: [0]pn%u/poc%d/sl%d [1]pn%u/poc%d/sl%d [2]pn%u/poc%d/sl%d [3]pn%u/poc%d/sl%d",
					(int)(dph->top_field_order_cnt & 0x3ffff), num_b0, num_b1, cur_slot,
					B4_PN(di0), B4_PC(di0), sl0,
					B4_PN(di1), B4_PC(di1), sl1,
					B4_PN(di2), B4_PC(di2), sl2,
					B4_PN(di3), B4_PC(di3), sl3);
#undef B4_SL
#undef B4_PC
#undef B4_PN
		}
	}

	/* seq 411-455: SPS/PPS words, picture size, recon mirrors (shifted),
	 * MV buffers, profile/config words -- trace constants + live addrs. */
	/*
	 * N88: H.264 High scaling matrices. The V4L2 PPS flag is defined by the
	 * uAPI as "a NON-FLAT scaling matrix applies to this picture" (not the
	 * literal pic_scaling_matrix_present_flag), and
	 * V4L2_CID_STATELESS_H264_SCALING_MATRIX carries the EFFECTIVE lists --
	 * userspace has already applied the Table 7-2 fall-back rules A and B
	 * and UseDefaultScalingMatrixFlag. So the driver's whole job is to push
	 * the 8 lists the HW consumes into the quant-matrix RAM and raise the
	 * enable bit, exactly as the vendor's Vdec_Drv_setDecPicInfo does after
	 * running those same fall-backs in software.
	 */
	if (pps->flags & V4L2_H264_PPS_FLAG_SCALING_MATRIX_PRESENT) {
		h264_scalcand++;
		if (h264_scaling & 1) {
			const struct v4l2_ctrl_h264_scaling_matrix *scm;

			scm = mtk_vdec_h264_get_ctrl_ptr(ctx,
					V4L2_CID_STATELESS_H264_SCALING_MATRIX);
			if (!IS_ERR_OR_NULL(scm)) {
				int i;
				bool dump;

				h264_scalhit++;
				dump = (h264_scaling & 256) && h264_scalhit == 1;
				scaling_dump = dump;
				/*
				 * N89 bit6 -- SKIP the RAM writes and raise the
				 * enable bit alone. The NEGATIVE CONTROL N88
				 * never ran: all 12 vectors the arm flipped
				 * carry effective matrices that are exactly the
				 * Table 7-2 defaults, so if this silicon holds
				 * those defaults itself, bit18 alone explains
				 * every flip and the RAM writes do nothing --
				 * which would also explain, completely, why the
				 * 9 vectors carrying real transmitted values
				 * still fail.
				 *
				 * N89 bit9 -- defer the RAM writes until AFTER
				 * AVCVLD+0x94 is programmed (ordering control;
				 * we currently write the RAM first).
				 */
				scaling_deferred = scm;
				if (!(h264_scaling & (64 | 512))) {
					for (i = 0; i < 6; i++)
						mt8365_h264_write_qm_list(ctx,
							AVCVLD_QM_4X4_BASE(i),
							scm->scaling_list_4x4[i], 16,
							dump);
					for (i = 0; i < 2; i++)
						mt8365_h264_write_qm_list(ctx,
							AVCVLD_QM_8X8_BASE(i),
							scm->scaling_list_8x8[i], 64,
							dump);
				}

				r_pps94 |= AVCVLD_PPS94_QM_EN;
				if (h264_scaling & 2)
					r_pps94 |= AVCVLD_PPS94_QM_LISTS;

				if ((h264_scaling & 16) && h264_scalhit == 1)
					mtk_v4l2_vdec_err(ctx,
						"mt8365 N88 QM 4x4[0]=%16ph 4x4[3]=%16ph 8x8[0]=%16ph 8x8[1]=%16ph pps94=0x%08x",
						scm->scaling_list_4x4[0],
						scm->scaling_list_4x4[3],
						scm->scaling_list_8x8[0],
						scm->scaling_list_8x8[1],
						r_pps94);
			} else if (unlikely(h264_diaglog)) {
				/* N137: per-picture, and absent is the NORMAL
				 * case for a stream with no explicit lists. */
				mtk_v4l2_vdec_err(ctx,
					"mt8365 N88 SCALING_MATRIX control absent");
			}
		}
	}

	wr(ctx, VDEC_AD,  0x88,  r_sps);
	wr(ctx, VDEC_TOP, 0x68,  r_top68);
	wr(ctx, VDEC_TOP, 0x100, r_top100);
	wr(ctx, VDEC_AD,  0x90,  r_pps90);
	wr(ctx, VDEC_AD,  0x94,  r_pps94);

	/*
	 * N89 bit9 -- the ordering control: push the quant-matrix RAM AFTER the
	 * enable bit rather than before it. bit6 (skip) takes precedence, so
	 * 64|512 is still "no RAM writes at all".
	 */
	if ((h264_scaling & 512) && !(h264_scaling & 64) && scaling_deferred) {
		int qi;

		for (qi = 0; qi < 6; qi++)
			mt8365_h264_write_qm_list(ctx, AVCVLD_QM_4X4_BASE(qi),
						  scaling_deferred->scaling_list_4x4[qi],
						  16, scaling_dump);
		for (qi = 0; qi < 2; qi++)
			mt8365_h264_write_qm_list(ctx, AVCVLD_QM_8X8_BASE(qi),
						  scaling_deferred->scaling_list_8x8[qi],
						  64, scaling_dump);
	}
	/*
	 * Slice-header AVCVLD+0x8c/0x98 are a READ-MODIFY-WRITE in the vendor
	 * SetSHDRAVLD1 (@0x4bec4), not absolute writes: 0x8c keeps [31:17] and
	 * loads the packed slice word's [12:0]; 0x98 keeps [12:0] (earlier
	 * ref/POC state) and loads [31:13]. The packed word's bit17 lands in
	 * 0x98[17] -- exactly our error bit (AVCVLD+0x284 bit17), and the decode
	 * dies at MB0, so a plain write that clobbers the preserved bits is the
	 * prime suspect. Our trace low-bits are 0 (0x8c const=0, 0x98 const &
	 * 0x1fff = 0), so RMW = preserve HW's current low/high halves.
	 */
	wr(ctx, VDEC_AD,  0x8c,  (rd(ctx, VDEC_AD, 0x8c) & 0xfffe0000) | (r_shdr_p & 0x1fff));
	wr(ctx, VDEC_AD,  0x98,  (rd(ctx, VDEC_AD, 0x98) & 0x00001fff) | (r_shdr_p & 0xffffe000));
	/*
	 * Ref-list COMMIT STROBE in VENDOR ORDER (AVCVLD+0x22c=1 immediately after
	 * the 0x98 slice-header word -- vendor seq 2151->2152, every P frame). This
	 * latches the slice-header ref-count (0x98[21:20]) + the AVCVLD PicNum
	 * groups + AVCMV descriptor tables programmed above. The prior WIP fired
	 * 0x22c LATE (after the VLD barrel-arm re-wrote 0x84) and WITHOUT the 0x98
	 * ref-count enable -- so it committed a "0 active refs" state and the
	 * ref-list was inert/corrupt. Fire it here, in the vendor's position, so
	 * the committed state matches the vendor's. IDR path has no references.
	 */
	if (is_inter)
		wr(ctx, VDEC_AD, 0x22c, 0x00000001);
	/*
	 * N38 MBAFF-only colocated-MV block enable. Vendor MBAFF trace seq 600/601
	 * (and 1326/1327, once per picture), immediately BEFORE the MC+0x90 write
	 * below -- a read-modify-write that SETS bit 5:
	 *     R AVCMV +0x220 = 0x00000c00 ; W AVCMV +0x220 = 0x00000c20
	 * `grep -c` over the matched progressive trace returns 0. Preserve the
	 * other bits (RMW, exactly as the vendor does).
	 */
	/*
	 * N63: widen the gate to the vendor's OWN condition. The identified,
	 * picture-0-proven PAFF trace writes this RMW on every picture of a
	 * !frame_mbs_only stream and never on a progressive one -- see the
	 * h264_mv220 comment above. Candidates counted unconditionally so the
	 * predicate's reachability is observable with the arm OFF.
	 */
	{
		bool notfmo = !(sps->flags &
				V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY);

		if (notfmo)
			h264_mv220cand++;
		if ((eff_mbaff && (h264_mbaff & MBAFF_MV220)) ||
		    (notfmo && (h264_mv220 & MV220_NOTFMO))) {
			wr(ctx, VDEC_AV, 0x220,
			   rd(ctx, VDEC_AV, 0x220) | 0x00000020);
			h264_mv220hit++;
		}
	}
	wr(ctx, VDEC_CM,  0x90,  0x00000004);
	wr(ctx, VDEC_TOP, 0x58,  0xffffffff);
	/*
	 * Picture-geometry words -- DERIVED from the live SPS MB dims (mb_w/mb_h),
	 * NOT hardcoded to the 176x144 canonical clip. Confirmed by a fresh-vs-
	 * canonical vendor vpud read-trace diff (c176 vs c320, IDR phase): every
	 * one of these scales with resolution and nothing else does. Canonical
	 * (176x144: mb_w=11 mb_h=9) reproduces the old constants exactly, so this
	 * is a pure generalization (no canonical regression).
	 *   MC+0x208 = pic width px   (c176 0xb0=176  -> c320 0x140=320)
	 *   MC+0x20c = pic height px  (c176 0x90=144  -> c320 0xf0=240)
	 */
	wr(ctx, VDEC_CM,  0x208, mb_w * 16);
	wr(ctx, VDEC_CM,  0x20c, mb_h * 16);
	wr(ctx, VDEC_MISC, 0x04, 0x00000010);
	wr(ctx, VDEC_TOP, 0x70,  r_top70);
	wr(ctx, VDEC_AD,  0x280, 0x7fffbcb7);
	wr(ctx, VDEC_AD,  0x26c, 0x7fffbcb7);
	wr(ctx, VDEC_CM,  0x220, 0x00000001);
	wr(ctx, VDEC_CM,  0x224, (u32)(fb->base_y.dma_addr >> 9));
	wr(ctx, VDEC_CM,  0x228, (u32)(fb->base_c.dma_addr >> 8));
	wr(ctx, VDEC_CM,  0x250, 0x00000001);
	wr(ctx, VDEC_CM,  0x238, 0x00000003);
	wr(ctx, VDEC_CM,  0x260, mb_w - 1);	/* mb_w-1 (c176 0xa -> c320 0x13) */
	wr(ctx, VDEC_CM,  0x264, mb_h_pic - 1);	/* MB-grid rows-1 (field: halved) */
	wr(ctx, VDEC_CM,  0x22c, mb_w);		/* mb_w   (c176 0xb -> c320 0x14) */
	/*
	 * N30 recon-window A/B. arm 0 (default, historical): 0x854 <- recon Y >> 4,
	 * 0x850 <- recon C >> 4. arm 1: the VP8-measured window semantics
	 * (vdec_vp8_mt8365_if.c L1544-1572, 20/20 allocator-order correlation) --
	 * 0x854 = window BASE = min(planes) >> 4, 0x850 = window LIMIT =
	 * max(plane end) >> 4, spanning the union of BOTH CAPTURE planes. Settle
	 * by hw_md5 on one kernel; no rebuild between arms.
	 */
	if (h264_recwin == 1) {
		u32 ysz = (mb_w * 16) * ALIGN(mb_h * 16, 32);
		u32 y0 = (u32)fb->base_y.dma_addr, c0 = (u32)fb->base_c.dma_addr;
		u32 y1 = y0 + ysz, c1 = c0 + ysz / 2;
		u32 base = min(y0, c0), limit = max(y1, c1);

		wr(ctx, VDEC_CM,  0x854, base >> 4);
		wr(ctx, VDEC_CM,  0x850, limit >> 4);
	} else {
		wr(ctx, VDEC_CM,  0x854, (u32)(fb->base_y.dma_addr >> 4));
		wr(ctx, VDEC_CM,  0x850, (u32)(fb->base_c.dma_addr >> 4));
	}
	wr(ctx, VDEC_CM,  0x858, 0x00000001);
	wr(ctx, VDEC_CM,  0x5e4, 0x00020000);
	wr(ctx, VDEC_CM,  0x560, rec_y);
	/*
	 * MC+0x55c = luma plane byte size = tile-stride(ALIGN(w,16)=mb_w*16) *
	 * ALIGN(h,32). It doubles as the C-plane offset within the recon buffer,
	 * so a wrong (hardcoded-176x144) value scatters the reconstruction and
	 * the HW stalls. c176 0x6e00=28160=176*160 -> c320 0x14000=81920=320*256
	 * (matches the CAPTURE bytesused the vendor trace reported for each clip).
	 */
	wr(ctx, VDEC_CM,  0x55c, (mb_w * 16) * ALIGN(mb_h * 16, 32));
	wr(ctx, VDEC_CM,  0x980, mb_w);		/* mb_w (c176 0xb -> c320 0x14) */
	wr(ctx, VDEC_AD,  0x2c8, 0xf900a03b);
	wr(ctx, VDEC_AD,  0x2d0, 0x08009101);
	wr(ctx, VDEC_AD,  0x2b4, 0xffffffff);
	wr(ctx, VDEC_AD,  0x2b8, 0xffffffff);
	wr(ctx, VDEC_AD,  0x9c,  r_9c);
	wr(ctx, VDEC_CM,  0x5e4, 0x00021000);
	/* N72: see the h264_mc920 comment -- default 0 keeps 0x01000111. */
	{
		u32 mc920 = 0x01000111u;

		h264_mc920cand++;
		if (h264_mc920 & MC920_VENDOR) {
			mc920 = 0x01000000u;
			h264_mc920hit++;
		}
		wr(ctx, VDEC_CM, 0x920, mc920);
	}
	wr(ctx, VDEC_CM,  0x768, 0x01000004);
	wr(ctx, VDEC_TOP, 0x6c,  mb_w * mb_h_pic);	/* MB-grid MB count (field: halved) */
	wr(ctx, VDEC_TOP, 0x50,  0xffffffff);
	wr(ctx, VDEC_CM,  0xa20, (u32)inst->work_buf[0].dma_addr);
	wr(ctx, VDEC_CM,  0xa24, (u32)inst->work_buf[1].dma_addr);
	/*
	 * AVC_MV colocated-MV WRITE base -- the register the H.264 AVC_MV_EXT
	 * write port (larb3 port9) stores this frame's colocated MVs through.
	 * Vendor H264_HAL_DecStart @0x4c4d8 writes AVC_MV(0x16023000)+0x20c =
	 * curr_fb->mv_buf_phys >> 4. Invisible in the trace (regtrace never
	 * hooked the AVC_MV block), so it defaulted to 0 -> the HW wrote MB0's
	 * MVs to iova~0 -> M4U fault -> the generic AVCVLD+0x284 bit17 error at
	 * MB0. Point it at THIS frame-store's own colMV pool buffer (cur_mv) so
	 * that when this frame is later referenced its stored colocated MVs are
	 * fetched from the same buffer the ref-list descriptor (+0x180) names.
	 */
	/*
	 * N67: bit1 = the CURRENT picture's field parity (see the h264_mvstore
	 * comment). Candidates counted unconditionally so the denominator is
	 * observable with the arm OFF.
	 */
	{
		u32 store = cur_mv;

		if (eff_field_pic && eff_bottom_field) {
			h264_mvstorecand++;
			if (h264_mvstore & MVSTORE_FLDPARITY) {
				store |= mt8365_mv_k(MVK_STORE);
				h264_mvstorehit++;
			}
		}
		wr(ctx, VDEC_AV, 0x20c, store);
	}
	wr(ctx, VDEC_AD,  0x2d0, 0x08009101);
	wr(ctx, VDEC_AD,  0x2d4, 0x00204008);
	wr(ctx, VDEC_TOP, 0x9c,  (rd(ctx, VDEC_TOP, 0x9c) & ~0x1fffu) | 0x400);	/* RMW (vendor 0x4c774) */

	/*
	 * AVC-VLD bitstream-engine ARM -- vendor H264_HAL_InitH264BarrelShift
	 * (@vpud+0x4d970) + ...Align (@0x4d410). The writes-only trace missed this
	 * WHOLE step: the common-VLD (VDEC_LD, 0x16020000) writes were invisible
	 * (regtrace never hooked WriteVLD), and the AVCVLD+0x28c arm strobe was
	 * mis-labeled software-parse-only (it is the AVC-VLD arm; removing it in
	 * 00713 deleted the arm). Without this the AVC-VLD never fetches the
	 * bitstream, so AVCVLD+0x25c bit25 (VLD-busy) never clears and the decode
	 * hangs. Program the VLD fetch window over our AU
	 * [dma_addr, dma_addr+size+0x100) in 64-byte units, kick the fetch
	 * (VLD+0x8c bit20), wait fetch-done (VLD+0xe8 bit0). Encodings resolved
	 * from disasm: VLDTOP+0x28 = raw PA; VLD+0xb4 = PA>>6; VLD+0xb8/0x110 =
	 * (PA+size+0x100) in >>6 / raw.
	 */
	{
		int i;
		u32 v;

		wr(ctx, VDEC_LD,  0xc0,  0x00040000);
		/* N41 bit 4: this mid barrel-arm write is ours-only (the vendor
		 * writes AVCVLD+0x84 exactly twice per picture) and hardcodes
		 * bit23 clear; the arm lets a field picture keep the derived
		 * r_84 here instead. */
		wr(ctx, VDEC_AD,  0x84,
		   (eff_field_pic && (h264_fldcfg & FLDCFG_84MID_R84))
			? r_84 : 0xae00e007u);
		wr(ctx, VDEC_TOP, 0x54,  rd(ctx, VDEC_TOP, 0x54) | 0x1);
		wr(ctx, VDEC_CM,  0x24,  0x00000006);
		wr(ctx, VDEC_LD,  0xb4,  (u32)(bs->dma_addr >> 6));
		wr(ctx, VDEC_LD,  0xb8,  (u32)((bs->dma_addr + bs->size + 0x100) >> 6));
		wr(ctx, VDEC_LD,  0xec,  rd(ctx, VDEC_LD, 0xec) | 0x10000000);
		wr(ctx, VDEC_TOP, 0x28,  (u32)bs->dma_addr);
		wr(ctx, VDEC_AD,  0x28c, 0x00000001);
		wr(ctx, VDEC_AD,  0x28c, 0x00000000);

		/* ...Align: wait VLD idle (0xf4), program byte window, reset, fetch */
		for (i = 0; i < 50000; i++) {
			v = rd(ctx, VDEC_LD, 0xf4);
			if (!(v & 0x8000) || (v & 0x1))
				break;
			cpu_relax();
		}
		/* N41 bit 8: the ONLY vendor write the field write-stream value
		 * differential found missing from ours -- a VLD+0x110 = 0xffffff7f
		 * sentinel emitted immediately before the two VLD+0xb0 base writes
		 * and re-armed by the real end-of-window value below (trace-fld.log
		 * seq 33/36 and 766/…; the progressive trace does the same at seq
		 * 34/37, so it is not field-specific -- but the progressive path is
		 * bit-exact on 67 vectors without it, which is why it is only wired
		 * for a field picture here). */
		if (eff_field_pic && (h264_fldcfg & FLDCFG_VLD110_PRE))
			wr(ctx, VDEC_LD, 0x110, 0xffffff7fu);
		wr(ctx, VDEC_LD,  0xb0,  (u32)bs->dma_addr);
		wr(ctx, VDEC_LD,  0xb0,  (u32)bs->dma_addr);
		wr(ctx, VDEC_LD,  0x110, (u32)(bs->dma_addr + bs->size + 0x100));
		wr(ctx, VDEC_AD,  0x28c, 0x00000001);	/* strobe asserted around reset+fetch */
		wr(ctx, VDEC_LD,  0x108, 0x00000100);
		wr(ctx, VDEC_LD,  0x108, 0x00000000);
		wr(ctx, VDEC_LD,  0xcc,  0x00000000);
		/*
		 * Pre-fetch/streaming enable (vendor eBufGetBitStream @0x2d820:
		 * VLD+0x1d0 |= 0x100 before the fetch). The MB decode STREAMS the
		 * bitstream -- a single primed fetch underruns (MISC+0xa4 stalls at
		 * 0x1000, HW stays active). Setting the auto-prefetch enable lets the
		 * HW keep fetching the programmed window [dma_addr, +size+0x100) as
		 * the decoder consumes it, instead of one 64B prime.
		 */
		wr(ctx, VDEC_LD,  0x1d0, rd(ctx, VDEC_LD, 0x1d0) | 0x100);
		wr(ctx, VDEC_LD,  0x8c,  0x00100000);	/* kick VLD fetch (bit20) */
		for (i = 0; i < 4096; i++) {
			if (rd(ctx, VDEC_LD, 0xe8) & 0x1)
				break;
			cpu_relax();
		}
		if (i == 4096)
			mtk_v4l2_vdec_err(ctx,
				"mt8365 VLD fetch-done VLD+0xe8 timeout; AVCVLD+0x25c=%08x VLD+0xf4=%08x",
				rd(ctx, VDEC_AD, 0x25c), rd(ctx, VDEC_LD, 0xf4));
		else {
			/*
			 * N137: keep the AVCVLD+0x25c READ unconditional even
			 * when the message is gated.  Gating a printk cannot
			 * change the decode; gating a REGISTER ACCESS could,
			 * and this driver's AVCVLD reads are not all
			 * side-effect-free (the +0x23c/+0x240/+(n<<2) family
			 * CONSUMES bitstream bits).  0x25c is a status word, but
			 * the register stream stays byte-identical to the
			 * pre-N137 driver by construction rather than by
			 * argument.
			 */
			u32 vld25c = rd(ctx, VDEC_AD, 0x25c);

			if (unlikely(h264_diaglog))
				mtk_v4l2_vdec_err(ctx,
					"mt8365 VLD fetch-done OK after %d; AVCVLD+0x25c=%08x",
					i, vld25c);
		}
		wr(ctx, VDEC_LD,  0x8c,  0x00800000);
		wr(ctx, VDEC_AD,  0x28c, 0x00000000);
		(void)rd(ctx, VDEC_AD, 0x00);
		/*
		 * N38 MBAFF-only VLD strobe. Vendor MBAFF trace seq 785 (and 1542,
		 * once per picture), exactly here -- after the barrel-shift arm ends
		 * (0x28c cleared, AVCVLD+0x0 / +0x84 read back) and before the
		 * header bit advance. It self-clears (the vendor's very next line is
		 * `R AVCVLD +0x2d8 = 0x00000000`), so read it back the same way.
		 * Absent from the matched progressive trace.
		 */
		if (eff_mbaff && (h264_mbaff & MBAFF_VLD2D8)) {
			wr(ctx, VDEC_AD, 0x2d8, 0x00000001);
			(void)rd(ctx, VDEC_AD, 0x2d8);
		}
	}

	/*
	 * Advance the VLD barrel-shifter to the slice-DATA bit position. There is
	 * NO MB-start-offset register: the HW MB entropy decoder resumes from the
	 * barrel-shifter's internal read pointer, which is advanced only by
	 * CONSUMING bits -- each read of AVCVLD[nbits<<2] extracts nbits bits and
	 * auto-advances (H264_HAL_AVCVLDGetBitS @0x490c0). vpud reaches slice-data
	 * bit 0 by parsing the header in software; we consume the equivalent bit
	 * count directly.
	 *
	 * The arm left the VLD at bs->dma_addr (OUTPUT buffer byte 0). The V4L2
	 * H264_SLICE_PARAMS header_bit_size is the bit offset from that byte 0 to
	 * slice_data() -- the annexb start code + NAL header + slice header (the
	 * same quantity cedrus consumes from its VLD byte 0; START_CODE is forced
	 * to ANNEX_B by this driver). Consume exactly header_bit_size bits, chunked
	 * <=16 to stay clear of the 0x80/0x84 status/kick words.
	 */
	{
		/* eff_hbs = header_bit_size, from SLICE_PARAMS (slice-based client) or
		 * the in-kernel slice-header parse (frame-based gst) -- adaptive block
		 * above. It is the bit offset from OUTPUT-buffer byte 0 to slice_data();
		 * consume exactly that many bits to advance the barrel-shifter, chunked
		 * <=16 to stay clear of the 0x80/0x84 status/kick words. */
		u32 remaining;

		if (eff_hbs == 0 || eff_hbs > bs->size * 8) {
			mtk_v4l2_vdec_err(ctx,
				"mt8365 bad header_bit_size=%u (bs.size=%zx, src=%s)",
				eff_hbs, bs->size, sh_from_ctrl ? "ctrl" : "parse");
			return -EINVAL;
		}
		/*
		 * CABAC byte-alignment (H.264 7.3.4 cabac_alignment_one_bit). For a
		 * CABAC picture slice_data() begins with a run of cabac_alignment_one_bits
		 * that pads to a BYTE boundary BEFORE the arithmetic-decoding engine
		 * initialises (it reads its first 9-bit codIOffset from the byte-aligned
		 * position). header_bit_size points at the end of the slice header, which
		 * is generally mid-byte (e.g. 68 for the QCIF IDR = byte 8.5), so for
		 * CABAC advance the barrel-shifter up to the next byte boundary; for
		 * CAVLC slice_data() starts exactly at header_bit_size (unchanged, so
		 * every CAVLC clip is bit-identical to before). Trace-verified fix: with
		 * the mid-byte start the CABAC engine inits from the wrong bit position
		 * and the decode faults part-way (AVCVLD+0x284=0x4008 at MB(3,3)).
		 */
		if (pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE) {
			u32 aligned = ALIGN(eff_hbs, 8);

			if (aligned != eff_hbs && unlikely(h264_diaglog))
				mtk_v4l2_vdec_err(ctx,
					"mt8365 CABAC align hbs %u -> %u", eff_hbs, aligned);
			eff_hbs = aligned;
		}
		/*
		 * N28 WEIGHTED PREDICTION. The MT8365 H.264 HW parses
		 * pred_weight_table() ITSELF out of its own barrel shifter -- there
		 * are no per-weight registers. Transcribed from
		 * H264_HAL_PredWeightTable @libvpud_vcodec.so+0x49d80 (mt8365
		 * aarch64), whose entire body is:
		 *     ReadMISC (hal, 0xf0, &saved)            ; MISC base = hal[16]
		 *     WriteAVCVLD(hal, 0x230, 1)              ; AVCVLD base = hal[48]
		 *     do { ReadAVCVLD(hal, 0x230, &v) } while (!v && elapsed <= 1500us)
		 *     WriteAVCVLD(hal, 0xf0, saved & ~1)
		 * and its ONE call site is parseSliceHeaderHW+0x390 (@0x54bf0), fired
		 * exactly where pred_weight_table() sits in the slice header, gated on
		 * PPS weighted_pred_flag && slice_type in {0,3,5} (P/SP) -- the vendor
		 * has no B/weighted_bipred_idc==1 call site at all.
		 *
		 * Our driver software-parses the header and then bulk-advances the
		 * barrel shifter by eff_hbs bits, so to reach the same HW state we
		 * split the advance: consume up to pred_weight_table(), let the HW
		 * consume the table (which is what latches the weights), then consume
		 * the remainder. The shifter position after the handshake is
		 * pwt_end by construction (the HW parse is spec-exact), so the
		 * remaining count is eff_hbs - pwt_end.
		 */
		/*
		 * N49 (h264_hwsh & 1): the VENDOR MODEL -- let the HARDWARE parse
		 * the slice header, element by element, through its own bit reader
		 * (AVCVLD ue=+0x23c / se=+0x240 / u(n)=+(n<<2)), exactly as
		 * parseSliceHeaderHW does, and fire the AVCVLD+0x230
		 * pred_weight_table handshake from inside that walk. The bulk
		 * advance below reaches the same BIT COUNT; this reaches the same
		 * PARSE STATE. The walk's final position is cross-checked against
		 * header_bit_size, so a HW/SW parse desync is loud, not silent.
		 */
		if (h264_hwsh & HWSH_WALK) {
			u32 hw_pos = 0;
			bool fired = false;
			struct mt8365_hwsh_syn syn;
			int wret = mt8365_hwsh_walk(ctx, sps, pps, bs->va,
						    bs->size, pwt_start, pwt_end,
						    &hw_pos, &fired,
						    &pwt_parked, &pwt_desync_qpd,
						    &syn);

			if (wret) {
				mtk_v4l2_vdec_err(ctx,
					"mt8365 hwsh walk failed %d (pos would be %u, hbs %u)",
					wret, hw_pos, eff_hbs);
				return wret;
			}
			if (hw_pos > eff_hbs) {
				mtk_v4l2_vdec_err(ctx,
					"mt8365 hwsh OVERRUN pos=%u > hbs=%u", hw_pos, eff_hbs);
				return -EIO;
			}
			/*
			 * N132 PWTSTOP: a parked walk is SUPPOSED to stop short
			 * of header_bit_size -- that is the whole arm -- so the
			 * accounting check below must not fire for it, and the
			 * caller must consume nothing more.
			 */
			if (pwt_parked) {
				if (h264_hwsh & HWSH_LOG)
					pr_info("H264PWTSTOP advance stops at %u (hbs %u, pwt %u..%u)\n",
						hw_pos, eff_hbs, pwt_start, pwt_end);
				remaining = 0;
				goto pwtstop_advance_done;
			}
			if (hw_pos != eff_hbs && unlikely(h264_diaglog))
				mtk_v4l2_vdec_err(ctx,
					"mt8365 hwsh pos=%u != hbs=%u (pad %u)",
					hw_pos, eff_hbs, eff_hbs - hw_pos);
			if (h264_hwsh & HWSH_LOG)
				pr_info("H264HWSH done pos=%u hbs=%u pwt=%u..%u fired=%d\n",
					hw_pos, eff_hbs, pwt_start, pwt_end, fired);
			/*
			 * N50: the HARDWARE'S OWN VIEW vs OURS, one line each, for
			 * exactly the elements the excluded buckets rest on. "hw" is
			 * decoded by the AVCVLD ue/se decoders out of the bitstream;
			 * "our" is what this driver actually programs, which on the
			 * gst path (sh_from_ctrl) comes from the V4L2 uAPI --
			 * field_pic/bottom_field from DECODE_PARAMS, direct_spatial
			 * and the active counts from SLICE_PARAMS -- i.e. a
			 * genuinely INDEPENDENT source. A disagreement reopens the
			 * bucket; agreement moves the exclusion off our model and
			 * onto the hardware's own reading.
			 */
			if (h264_hwsh & HWSH_SYN) {
				pr_info("H264HWSYN hw  st=%u fn=%u fp=%u bf=%u ds=%u ovr=%u l0=%u l1=%u cii=%u/%u l0mod=%u/%u l1mod=%u/%u\n",
					syn.stype, syn.frame_num, syn.field_pic,
					syn.bottom_field, syn.direct_spatial,
					syn.override, syn.num_l0, syn.num_l1,
					syn.cabac_init_idc, syn.has_cabac_init,
					syn.l0mod_flag, syn.l0mod_n,
					syn.l1mod_flag, syn.l1mod_n);
				pr_info("H264HWSYN our st=%u fn=%u fp=%d bf=%d ds=%d l0=%d l1=%d cii=%u/%u src=%s rawl0=%d\n",
					eff_stype, dph->frame_num, eff_field_pic,
					eff_bottom_field, eff_direct_spatial,
					num_l0, num_l1,
					eff_cabac_init,
					!!(pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE),
					sh_from_ctrl ? "ctrl" : "parse", raw_num_l0);
				/*
				 * The V4L2 uAPI's OWN picture-level view, always
				 * printed regardless of which path the driver took
				 * -- DECODE_PARAMS is a genuinely independent
				 * source from the bitstream, and it is what the
				 * driver programs on the slice-based (ctrl) path.
				 * This is the cross-check the field bucket needs.
				 */
				pr_info("H264HWSYN uapi dpfp=%d dpbf=%d ctrl_hbs=%u ctrl_cii=%u ctrl_l0=%u ctrl_l1=%u ctrl_ds=%d nopsl1=%d\n",
					!!(dph->flags & V4L2_H264_DECODE_PARAM_FLAG_FIELD_PIC),
					!!(dph->flags & V4L2_H264_DECODE_PARAM_FLAG_BOTTOM_FIELD),
					slp->header_bit_size, slp->cabac_init_idc,
					slp->num_ref_idx_l0_active_minus1 + 1u,
					slp->num_ref_idx_l1_active_minus1 + 1u,
					!!(slp->flags & V4L2_H264_SLICE_FLAG_DIRECT_SPATIAL_MV_PRED),
					nops_l1);
				if (syn.l0mod_n)
					pr_info("H264HWSYN hw  l0mod [%u,%u] [%u,%u] [%u,%u] [%u,%u]\n",
						syn.l0mod[0][0], syn.l0mod[0][1],
						syn.l0mod[1][0], syn.l0mod[1][1],
						syn.l0mod[2][0], syn.l0mod[2][1],
						syn.l0mod[3][0], syn.l0mod[3][1]);
				if (syn.l1mod_n)
					pr_info("H264HWSYN hw  l1mod [%u,%u] [%u,%u] [%u,%u] [%u,%u]\n",
						syn.l1mod[0][0], syn.l1mod[0][1],
						syn.l1mod[1][0], syn.l1mod[1][1],
						syn.l1mod[2][0], syn.l1mod[2][1],
						syn.l1mod[3][0], syn.l1mod[3][1]);
			}
			remaining = eff_hbs - hw_pos;
		} else if ((h264_predwt & 1) && pwt_end > pwt_start &&
		    pwt_end <= eff_hbs &&
		    (eff_stype == 0 || eff_stype == 3)) {
			u32 saved, j;

			remaining = pwt_start;
			while (remaining > 0) {
				u32 chunk = remaining < 16 ? remaining : 16;

				(void)rd(ctx, VDEC_AD, chunk << 2);
				remaining -= chunk;
			}
			saved = rd(ctx, VDEC_MISC, 0xf0);
			wr(ctx, VDEC_AD, 0x230, 0x00000001);
			for (j = 0; j < 100000; j++) {
				if (rd(ctx, VDEC_AD, 0x230))
					break;
				cpu_relax();
			}
			if (j == 100000)
				mtk_v4l2_vdec_err(ctx,
					"mt8365 pred_weight_table HW parse AVCVLD+0x230 stuck");
			if (h264_predwt & 2)
				wr(ctx, VDEC_MISC, 0xf0, saved & ~1u);
			else
				wr(ctx, VDEC_AD, 0xf0, saved & ~1u);
			remaining = eff_hbs - pwt_end;
		} else {
			remaining = eff_hbs;
		}
pwtstop_advance_done:
		while (remaining > 0) {
			u32 chunk = remaining < 16 ? remaining : 16;

			(void)rd(ctx, VDEC_AD, chunk << 2);
			adv_log(chunk << 2, 0);
			remaining -= chunk;
		}
		/*
		 * N132 PWTSTOP bit2: re-program AVCVLD+0x9c[6:0] with the vendor's
		 * DESYNCED slice_qp_delta. 0x9c was written earlier in DecStart
		 * order with the CORRECT value from the V4L2 controls / our own
		 * parse; the vendor writes garbage there because its reader is
		 * standing inside the weight table, and it can afford to because
		 * the silicon has already latched the real QP from its own parse.
		 * This is the half that makes the registered "0x9c[6:0] = 0x7f"
		 * prediction testable -- parking alone does not change what we
		 * program.
		 */
		if (pwt_parked && (h264_pwtstop & PWTSTOP_QPD))
			wr(ctx, VDEC_AD, 0x9c,
			   (r_9c & ~0x7fu) | ((u32)pwt_desync_qpd & 0x7fu));
	}

	/*
	 * The arm used VLDTOP+0x28 as the fetch buffer base (=dma_addr), but the
	 * MB-decode path wants the trace's control value there (0xff1e0000). The
	 * fetch is done (bit25 cleared), so restore it before the trigger. With
	 * the arm's dma_addr left in place the decode reached MISC+0xa4=0x1000
	 * (engine ran) but never SUCCESS (0x10000); restoring 0xff1e0000 gives the
	 * MB decode its expected VLDTOP+0x28.
	 */
	wr(ctx, VDEC_TOP, 0x28, 0xff1e0000);

	/* (Ref-list COMMIT STROBE AVCVLD+0x22c=1 is now fired earlier, in vendor
	 * order immediately after the 0x98 slice-header word -- not here.) */

	/*
	 * CABAC arithmetic-engine ARM -- AVCVLD+0x234 = 1, fired once per frame
	 * for CABAC pictures ONLY (entropy_coding_mode == 1), in vendor DecStart
	 * order: after the 0x9c config word, immediately before the 0x25c
	 * VLD-ready poll below. The 2026-07-17 live vpud CABAC-vs-CAVLC read-trace
	 * (regtrace LD_PRELOAD into the vendor mtk-vcodec-mt8167 stack via the
	 * 13-range DTB; matched Main/CABAC vs Main/CAVLC I-slice clips) proved
	 * this is the ONLY runtime control-register write the vendor emits for
	 * CABAC and NOT for CAVLC -- the entropy select bit AVCVLD+0x90[0] is
	 * already set (since N8). Profile confound ruled out: a Main-profile CAVLC
	 * clip (profile_idc=77, entropy=0) does NOT get this write. Without the arm
	 * the CABAC engine is configured (0x90[0]=1) but never started -> ~8 dB
	 * garbage (the exact N8/N11 residual). CAVLC path is byte-identical
	 * (strobe skipped), so no non-CABAC regression is possible by construction.
	 */
	if (pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE)
		wr(ctx, VDEC_AD, 0x234, 0x00000001);

	/*
	 * Pre-trigger VLD-ready gate. The writes-only trace ended at the
	 * MISC+0x28 trigger, but the vendor H264_HAL_DecStart (@vpud+0x4c180)
	 * does MORE around it -- via DIRECT stores that the HAL-helper-hooked
	 * trace never captured. First it polls AVCVLD+0x25c bit25 until clear
	 * (VLD idle) before arming (disasm @0x4c570). Reading AVCVLD before the
	 * trigger is bus-safe (the VLD is not yet fetching).
	 */
	{
		int i;

		for (i = 0; i < 100000; i++) {
			if (!(rd(ctx, VDEC_AD, 0x25c) & 0x02000000))
				break;
			cpu_relax();
		}
		if (i == 100000)
			mtk_v4l2_vdec_err(ctx,
				"mt8365 VLD-ready gate AVCVLD+0x25c bit25 stuck=%08x",
				rd(ctx, VDEC_AD, 0x25c));
	}

	/*
	 * Step-10 control instrument: the denominator counter advances whether
	 * or not the arm is set, so a negative result is provable rather than
	 * inferred. Only the first pre-kick of a boot is read.
	 */
	h264_ld_cand++;
	if (unlikely(mt8365_h264_ldread) && h264_ld_cand == 1)
		h264_ldread(ctx);

	/*
	 * N31 control: arm the MISC CRC last, after the picture prologue's
	 * MISC+0x04 = 0x10 write, so the enable survives to the trigger.
	 */
	h264_crc_cand++;
	if (unlikely(mt8365_h264_crc)) {
		u32 v = rd(ctx, VDEC_MISC, 0x04);

		wr(ctx, VDEC_MISC, 0x04,
		   (v & ~3u) | ((u32)mt8365_h264_crc & 3u));
		h264_crc_appl++;
		if (h264_crc_appl <= 2)
			pr_info("mt8365 h264 CRCARM MISC+04 %08x -> %08x (SYS+00=%08x)\n",
				v, rd(ctx, VDEC_MISC, 0x04),
				rd(ctx, VDEC_SYS, 0x00));
	}

	/* seq 456: trigger ("go") -- RMW |=1 (vendor 0x4c7a0), not absolute */
	wr(ctx, VDEC_MISC, 0x28, rd(ctx, VDEC_MISC, 0x28) | 0x00000001);

	/*
	 * Phase-2 -- DecStart's post-trigger writes (direct stores in
	 * vpud+0x4c7d0..0x4c874, absent from the writes-only trace). The engine
	 * arms on MISC+0x28 but does not truly begin until the recon/workbuf
	 * addresses are re-latched, the SPS word is (re)written, and the final
	 * commit strobe AVCVLD+0x238=1 fires -- the last write before DecStart
	 * returns. Without them the VLD sits with the go-bit latched and hangs
	 * at the bitstream fetch (SYS=0x111, dec-done=0, MISC+0x28 stuck=1).
	 */
	wr(ctx, VDEC_MISC, 0x3c,  rec_y);
	wr(ctx, VDEC_MISC, 0x40,  rec_c);
	wr(ctx, VDEC_MISC, 0x54,  (u32)inst->work_buf[2].dma_addr);
	wr(ctx, VDEC_MISC, 0x58,  (u32)inst->work_buf[3].dma_addr);
	/* SPS word A. N40 bit 2: on a progressive picture the vendor's
	 * POST-trigger write always has bit23 SET (see the r_84 comment; bit23 is
	 * a pre/post-trigger strobe, not the non-IDR flag this used to claim). */
	wr(ctx, VDEC_AD,   0x84,
	   (!eff_field_pic && (h264_progcfg & PROGCFG_84POST))
		? (r_84 | (1u << 23))
		: ((eff_field_pic && (h264_fldcfg & FLDCFG_84POST_CLR))
			? (r_84 & ~(1u << 23)) : r_84));
	wr(ctx, VDEC_AD,   0x238, 0x00000001);	/* final commit strobe */

	/* N40: the picture's write stream ends here. */
	if (wrlog_on) {
		pr_info("H264WR ---- picture %d end\n", wrlog_idx);
		wrlog_on = 0;
	}
	if (h264_wrlog)
		wrlog_idx++;

	/* 5. wait the HW dec-done IRQ (core handler reads VDEC_MISC+0xa4, success
	 * bit 0x10000, then wakes us) -- same plumbing the SCP backends use. */
	ret = mtk_vcodec_wait_for_done_ctx(ctx, MTK_INST_IRQ_RECEIVED,
					   MT8365_VDEC_TIMEOUT_MS, MTK_VDEC_CORE);
	if (ret) {
		/* HW triggered but no dec-done IRQ: dump the input base we
		 * programmed (raw bs->dma_addr) vs the trace's 0x0065ff10, and
		 * key status regs to localize the stall. */
		mtk_v4l2_vdec_err(ctx,
			"mt8365 TIMEOUT bs.dma=%pad sz=%zx fb.y=%pad fb.c=%pad | SYS[0x00 hw_active]=%08x MISC:sts[0xa4]=%08x [0x00]=%08x [0x84]=%08x [0xec]=%08x inbase[0x160]=%08x | TOP[0x28]=%08x AVCVLD[0xa4]=%08x MC[0x24]=%08x",
			&bs->dma_addr, bs->size, &fb->base_y.dma_addr,
			&fb->base_c.dma_addr,
			rd(ctx, VDEC_SYS, 0x00),
			rd(ctx, VDEC_MISC, 0xa4), rd(ctx, VDEC_MISC, 0x00),
			rd(ctx, VDEC_MISC, 0x84), rd(ctx, VDEC_MISC, 0xec),
			rd(ctx, VDEC_MISC, MISC_INPUT_BASE),
			rd(ctx, VDEC_TOP, TOP_CTRL), rd(ctx, VDEC_AD, 0xa4),
			rd(ctx, VDEC_CM, 0x24));
		/*
		 * The vendor's real done/error signals are AVCVLD-side, not
		 * MISC+0xa4: DecPicComplete reads AVCVLD+0x278 bit31 (picture
		 * done); GetErrMsg returns AVCVLD+0x284 & +0x280 (error code).
		 * Also read VLD+0xe8 (fetch-done) / +0xf4 (idle) / +0x1d0
		 * (prefetch-en) to tell a bitstream underrun from a decode error.
		 */
		mtk_v4l2_vdec_err(ctx,
			"mt8365 vendor-status AVCVLD[0x278 done]=%08x [0x284 err]=%08x [0x280 mask]=%08x [0x25c]=%08x | VLD[0xe8]=%08x [0xf4]=%08x [0x1d0]=%08x",
			rd(ctx, VDEC_AD, 0x278), rd(ctx, VDEC_AD, 0x284),
			rd(ctx, VDEC_AD, 0x280), rd(ctx, VDEC_AD, 0x25c),
			rd(ctx, VDEC_LD, 0xe8), rd(ctx, VDEC_LD, 0xf4),
			rd(ctx, VDEC_LD, 0x1d0));
		/*
		 * The authoritative MB-stop position is MC+0x28 (mbx) / MC+0x2c
		 * (mby) -- what H264_HAL_GetMbxMby (@0x4cb74) reads and the vendor
		 * prints as the first-error MB. The MB grid is per-clip and is
		 * printed live below (mb_w x mb_h_pic) -- the old "frame is
		 * 11x9 MBs" here was a 176x144 bring-up literal that stayed
		 * wrong for every other resolution.
		 * mbx=mby=0 => dies at MB0 (config/start/slice-header); part-way
		 * => a slice-field / bitstream / ref issue.
		 */
		mtk_v4l2_vdec_err(ctx,
			"mt8365 stop-MB MC[0x28 mbx]=%08x MC[0x2c mby]=%08x (MB grid %ux%u, fld=%d)",
			rd(ctx, VDEC_CM, 0x28), rd(ctx, VDEC_CM, 0x2c),
			mb_w, mb_h_pic, eff_field_pic);
		/*
		 * Localized: the VLD (VDEC_AD @ 0x16022000) block is wedged at
		 * the bitstream fetch -- a broad CPU read of its low offsets
		 * HANGS THE AXI BUS (watchdog reset), the signature of a
		 * started-but-never-completing VLD fetch transaction (the
		 * trigger MISC+0x28 stays 1, dec-done MISC+0xa4 stays 0). So we
		 * must NOT sweep VDEC_AD / VDEC_CM here -- only the summary
		 * line's single AD+0xa4 / CM+0x24 reads are safe. Sweep the SYS
		 * and MISC status windows only (proven bus-safe). The real fix
		 * is a VLD-fetch-OK poll (vendor bVDecWaitVLDFetchOk) that the
		 * writes-only trace never captured.
		 */
		mt8365_dump_blk(ctx, VDEC_SYS,  "SYS",  0x00,  0x20);
		mt8365_dump_blk(ctx, VDEC_MISC, "MISC", 0x00,  0x60);
		mt8365_dump_blk(ctx, VDEC_MISC, "MISC", 0xa0,  0xe0);
		mt8365_dump_blk(ctx, VDEC_MISC, "MISC", 0x160, 0x180);
		return ret;
	}
	if ((ctx->irq_status & MTK_VDEC_IRQ_STATUS_DEC_SUCCESS) !=
	    MTK_VDEC_IRQ_STATUS_DEC_SUCCESS)
		return -EIO;

	/*
	 * N100 / N31 -- post-decode read side. The picture is complete and the
	 * engine is idle, which is the only point at which a STAGE state word
	 * or a computed digest means anything.
	 */
	h264_stage_cand++;
	if (unlikely(mt8365_h264_stage) && h264_stage_appl < h264_stage_lines) {
		h264_stage_appl++;
		pr_info("mt8365 h264 STAGE pic %d AVCVLD(0x16022000) 244=%08x 248=%08x 24c=%08x 250=%08x 254=%08x 258=%08x 25c=%08x 260=%08x 264=%08x 26c=%08x 270=%08x 274=%08x 278=%08x 280=%08x 284=%08x 29c=%08x 2a0=%08x | fld=%d btm=%d stype=%d\n",
			h264_stage_cand,
			rd(ctx, VDEC_AD, 0x244), rd(ctx, VDEC_AD, 0x248),
			rd(ctx, VDEC_AD, 0x24c), rd(ctx, VDEC_AD, 0x250),
			rd(ctx, VDEC_AD, 0x254), rd(ctx, VDEC_AD, 0x258),
			rd(ctx, VDEC_AD, 0x25c), rd(ctx, VDEC_AD, 0x260),
			rd(ctx, VDEC_AD, 0x264), rd(ctx, VDEC_AD, 0x26c),
			rd(ctx, VDEC_AD, 0x270), rd(ctx, VDEC_AD, 0x274),
			rd(ctx, VDEC_AD, 0x278), rd(ctx, VDEC_AD, 0x280),
			rd(ctx, VDEC_AD, 0x284), rd(ctx, VDEC_AD, 0x29c),
			rd(ctx, VDEC_AD, 0x2a0),
			eff_field_pic, eff_bottom_field, eff_stype);
	}
	if (unlikely(mt8365_h264_crc) && h264_crc_appl <= h264_stage_lines) {
		u32 c[8];
		int i;

		for (i = 0; i < 8; i++)
			c[i] = rd(ctx, VDEC_MISC, 0x08 + 4 * i);
		pr_info("mt8365 h264 CRC pic %d arm=%d MISC+04=%08x SYS+00=%08x MISC+00=%08x | MISC+08..24 %08x %08x %08x %08x %08x %08x %08x %08x\n",
			h264_stage_cand, mt8365_h264_crc,
			rd(ctx, VDEC_MISC, 0x04), rd(ctx, VDEC_SYS, 0x00),
			rd(ctx, VDEC_MISC, 0x00),
			c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7]);
	}

	/*
	 * N68: the HW has finished storing this picture's colocated MVs through
	 * AVCMV+0x20c, so the pool buffer now holds exactly what it wrote.
	 * Snapshot it here (a memcpy from a coherent mapping; zero cost when
	 * h264_mvdump is 0). mvdump_idx advances on every picture so the index
	 * matches the h264_wrlog picture numbering the audits cite.
	 */
	mt8365_mvdump_snap(inst, mvdump_idx, cur_slot, eff_field_pic,
			   eff_bottom_field, eff_stype);
	mvdump_idx++;

	/*
	 * N94: monochrome (4:0:0) chroma-plane fill. See the h264_mono block for
	 * the geometry / tiling-invariance / timing argument. The candidate
	 * counter is UNCONDITIONAL so the denominator is observable with the arm
	 * OFF; only the fill itself is behind the bitmask.
	 */
	if (sps->chroma_format_idc == 0) {
		h264_monocand++;
		if ((h264_mono & 1) && fb->base_c.va && fb->base_c.size) {
			memset(fb->base_c.va, h264_monoval & 0xff,
			       fb->base_c.size);
			h264_monohit++;
		}
	}

	return 0;
}

static int vdec_h264_mt8365_get_param(void *h_vdec,
				      enum vdec_get_param_type type, void *out)
{
	struct vdec_h264_mt8365_inst *inst = h_vdec;
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;

	switch (type) {
	case GET_PARAM_PIC_INFO:
		/*
		 * Stateless mode has no SOURCE_CHANGE: pic_w/pic_h are set from
		 * S_FMT(CAPTURE) before this call. Fill the CAPTURE (MM21) buffer
		 * geometry + per-plane sizes so the core can size the CAPTURE vb2
		 * queue -- an unset fb_sz[] leaves sizeimage[]=0, which trips the
		 * WARN_ON(!plane_sizes[i]) in vb2_core_reqbufs() and fails
		 * REQBUFS(CAPTURE) with -EINVAL.
		 *
		 * MM21 (== GStreamer NV12_16L32S) geometry MUST be declared at the
		 * CODED TILE size, NOT ALIGN(.,64). This mt8365 HW writes the recon
		 * planes DENSELY at the coded tile stride: Y as 16x32 tiles over
		 * ALIGN(pic_w,16) x ALIGN(pic_h,32), UV as 16x16 tiles over
		 * ALIGN(pic_w,16) x ALIGN(pic_h,32)/2. The luma tile-row stride the
		 * HW uses is mb_w*16 == ALIGN(pic_w,16) (it is derived from the MB
		 * picture size, NOT from buf_w), and the UV plane immediately
		 * follows the packed Y. The old ALIGN(.,64) declaration lied to
		 * userspace twice: it reported bytesperline = ALIGN(pic_w,64) (e.g.
		 * 192 for a 176-wide dense plane, so a standard MM21/NV12_16L32S
		 * detiler read 12 tile-columns from an 11-tile-column plane -> a
		 * per-tile-row diagonal shift) AND it placed the UV plane at
		 * ALIGN(pic_w,64)*ALIGN(pic_h,64) instead of the packed Y size. The
		 * net effect: GStreamer videoconvert and vdec-sweep's mm21_detile
		 * both collapsed to ~8.58 dB on an otherwise bit-exact decode.
		 * Declaring the geometry at the coded tile size makes the CAPTURE a
		 * standard, detileable MM21 (bytesperline == the HW's dense stride,
		 * UV plane at the packed offset).
		 */
		ctx->picinfo.buf_w = ALIGN(ctx->picinfo.pic_w, 16);
		ctx->picinfo.buf_h = ALIGN(ctx->picinfo.pic_h, 32);
		ctx->picinfo.fb_sz[0] =
			ctx->picinfo.buf_w * ctx->picinfo.buf_h;
		ctx->picinfo.fb_sz[1] = ctx->picinfo.fb_sz[0] >> 1;
		*(struct vdec_pic_info *)out = ctx->picinfo;
		break;
	case GET_PARAM_DPB_SIZE:
		/* H.264 max frame-store DPB; a per-clip SPS may need fewer. */
		*(unsigned int *)out = 16;
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

static void vdec_h264_mt8365_deinit(void *h_vdec)
{
	struct vdec_h264_mt8365_inst *inst = h_vdec;

	if (inst)
		free_work_bufs(inst);
	kfree(h_vdec);
}

const struct vdec_common_if vdec_h264_mt8365_if = {
	.init		= vdec_h264_mt8365_init,
	.decode		= vdec_h264_mt8365_decode,
	.get_param	= vdec_h264_mt8365_get_param,
	.deinit		= vdec_h264_mt8365_deinit,
};
