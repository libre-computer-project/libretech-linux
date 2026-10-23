// SPDX-License-Identifier: GPL-2.0+
/*
 * codec_h264_mvc.c -- Amlogic meson-vdec H.264 MVC (Multi-View Coding) decoder
 *
 * Implements stereo 3D H.264 MVC decode for the meson-vdec upstream driver.
 * Uses dedicated MVC firmware with bidirectional mailbox protocol.
 * Outputs both views frame-packed (default) or frame-sequential.
 * Frame-packed: view 0 + 45-line gap + view 1 in one NV12 buffer.
 * Frame-sequential: view 0 and view 1 as separate capture buffers.
 *
 * BSP reference: vh264_mvc.c (A311D_Linux2603V2.5, 1911 lines), analysed in
 * our internal H.264 MVC BSP analysis
 * Firmware:     meson/vdec/gxl_h264_mvc.bin, gxm_h264_mvc.bin, sm1_h264_mvc.bin
 *
 * MVC scratch register protocol differs from regular H.264:
 *   AV_SCRATCH_0 (MAILBOX_CMD):    bidirectional command (0=done, 1=alloc_view0,
 *                                   2=alloc_view1, 3=display, 4=fatal)
 *   AV_SCRATCH_1 (MAILBOX_DATA_0): alloc: level_idc<<24|max_ref<<16|mb_w<<8|mb_h
 *                                  display: view_id[7:6]|buf_id[5:0]
 *   AV_SCRATCH_2 (MAILBOX_DATA_1): display POC or ref_size response
 *   AV_SCRATCH_3 (MAILBOX_DATA_2): stream offset
 *   AV_SCRATCH_6 (CANVAS_START):   canvas base index (0x78 for legacy, 0 for vdec_canvas)
 *   AV_SCRATCH_7 (BUFFER_RECYCLE): (view_id<<8 | buf_id+1), 0=idle
 *   AV_SCRATCH_A (DECODE_STATUS):  1=DECODE_START_HEADER at init
 *   AV_SCRATCH_B (SPS_STATUS):     SPS availability bitmap (0 at init)
 *   AV_SCRATCH_C (PPS_STATUS):     PPS availability bitmap (0 at init)
 *   AV_SCRATCH_E (WORKSPACE_START): physical address of workspace buffer
 *   AV_SCRATCH_H (CURRENT_UCODE):  0xff at init (no ucode cached)
 *   AV_SCRATCH_I (CURRENT_SPS_PPS): 0xffff at init
 *   AV_SCRATCH_M (REF_START_VIEW_0): view 0 reference buffer physical addr
 *   AV_SCRATCH_N (REF_START_VIEW_1): view 1 reference buffer physical addr
 *
 * Buffer model:
 *   The MVC firmware manages two independent canvas-based decode buffer pools,
 *   one per stereo view.  The driver allocates CMA buffers for both views and
 *   writes canvas addresses to ANC0_CANVAS_ADDR.  View 0 canvases start at
 *   CANVAS_INDEX_START (0x78); view 1 starts at 0x78 + (num_bufs_view0 * 3).
 *   When CMD_FRAME_DISPLAY fires for view 0, the decoded frame is copied to
 *   the next available V4L2 capture buffer and signalled done.
 *   View 1 frames are immediately recycled (MVP: left-eye output only).
 *
 * BSP source: vh264_mvc.c:H264_DECODE_INIT(), vh264mvc_prot_init(),
 *             do_alloc_work(), vh264mvc_isr(), vh264mvc_put_timer_func()
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>
#include <linux/soc/amlogic/meson-canvas.h>

#include "vdec_helpers.h"
#include "dos_regs.h"
#include "vdec.h"

/* ---- End-of-stream synthetic NAL sequence ------------------------------ */

/*
 * Buffer to send to the ESPARSER to signal End Of Stream for H.264 MVC.
 * This is a 16x16 encoded picture that will trigger drain firmware-side.
 * There is no known alternative.
 */
static const u8 eos_sequence_mvc[SZ_4K] = {
	0x00, 0x00, 0x00, 0x01, 0x06, 0x05, 0xff, 0xe4, 0xdc, 0x45, 0xe9, 0xbd,
	0xe6, 0xd9, 0x48, 0xb7,	0x96, 0x2c, 0xd8, 0x20, 0xd9, 0x23, 0xee, 0xef,
	0x78, 0x32, 0x36, 0x34, 0x20, 0x2d, 0x20, 0x63,	0x6f, 0x72, 0x65, 0x20,
	0x36, 0x37, 0x20, 0x72, 0x31, 0x31, 0x33, 0x30, 0x20, 0x38, 0x34, 0x37,
	0x35, 0x39, 0x37, 0x37, 0x20, 0x2d, 0x20, 0x48, 0x2e, 0x32, 0x36, 0x34,
	0x2f, 0x4d, 0x50, 0x45,	0x47, 0x2d, 0x34, 0x20, 0x41, 0x56, 0x43, 0x20,
	0x63, 0x6f, 0x64, 0x65, 0x63, 0x20, 0x2d, 0x20,	0x43, 0x6f, 0x70, 0x79,
	0x6c, 0x65, 0x66, 0x74, 0x20, 0x32, 0x30, 0x30, 0x33, 0x2d, 0x32, 0x30,
	0x30, 0x39, 0x20, 0x2d, 0x20, 0x68, 0x74, 0x74, 0x70, 0x3a, 0x2f, 0x2f,
	0x77, 0x77, 0x77, 0x2e,	0x76, 0x69, 0x64, 0x65, 0x6f, 0x6c, 0x61, 0x6e,
	0x2e, 0x6f, 0x72, 0x67, 0x2f, 0x78, 0x32, 0x36,	0x34, 0x2e, 0x68, 0x74,
	0x6d, 0x6c, 0x20, 0x2d, 0x20, 0x6f, 0x70, 0x74, 0x69, 0x6f, 0x6e, 0x73,
	0x3a, 0x20, 0x63, 0x61, 0x62, 0x61, 0x63, 0x3d, 0x31, 0x20, 0x72, 0x65,
	0x66, 0x3d, 0x31, 0x20,	0x64, 0x65, 0x62, 0x6c, 0x6f, 0x63, 0x6b, 0x3d,
	0x31, 0x3a, 0x30, 0x3a, 0x30, 0x20, 0x61, 0x6e,	0x61, 0x6c, 0x79, 0x73,
	0x65, 0x3d, 0x30, 0x78, 0x31, 0x3a, 0x30, 0x78, 0x31, 0x31, 0x31, 0x20,
	0x6d, 0x65, 0x3d, 0x68, 0x65, 0x78, 0x20, 0x73, 0x75, 0x62, 0x6d, 0x65,
	0x3d, 0x36, 0x20, 0x70,	0x73, 0x79, 0x5f, 0x72, 0x64, 0x3d, 0x31, 0x2e,
	0x30, 0x3a, 0x30, 0x2e, 0x30, 0x20, 0x6d, 0x69,	0x78, 0x65, 0x64, 0x5f,
	0x72, 0x65, 0x66, 0x3d, 0x30, 0x20, 0x6d, 0x65, 0x5f, 0x72, 0x61, 0x6e,
	0x67, 0x65, 0x3d, 0x31, 0x36, 0x20, 0x63, 0x68, 0x72, 0x6f, 0x6d, 0x61,
	0x5f, 0x6d, 0x65, 0x3d,	0x31, 0x20, 0x74, 0x72, 0x65, 0x6c, 0x6c, 0x69,
	0x73, 0x3d, 0x30, 0x20, 0x38, 0x78, 0x38, 0x64,	0x63, 0x74, 0x3d, 0x30,
	0x20, 0x63, 0x71, 0x6d, 0x3d, 0x30, 0x20, 0x64, 0x65, 0x61, 0x64, 0x7a,
	0x6f, 0x6e, 0x65, 0x3d, 0x32, 0x31, 0x2c, 0x31, 0x31, 0x20, 0x63, 0x68,
	0x72, 0x6f, 0x6d, 0x61,	0x5f, 0x71, 0x70, 0x5f, 0x6f, 0x66, 0x66, 0x73,
	0x65, 0x74, 0x3d, 0x2d, 0x32, 0x20, 0x74, 0x68,	0x72, 0x65, 0x61, 0x64,
	0x73, 0x3d, 0x31, 0x20, 0x6e, 0x72, 0x3d, 0x30, 0x20, 0x64, 0x65, 0x63,
	0x69, 0x6d, 0x61, 0x74, 0x65, 0x3d, 0x31, 0x20, 0x6d, 0x62, 0x61, 0x66,
	0x66, 0x3d, 0x30, 0x20,	0x62, 0x66, 0x72, 0x61, 0x6d, 0x65, 0x73, 0x3d,
	0x30, 0x20, 0x6b, 0x65, 0x79, 0x69, 0x6e, 0x74,	0x3d, 0x32, 0x35, 0x30,
	0x20, 0x6b, 0x65, 0x79, 0x69, 0x6e, 0x74, 0x5f, 0x6d, 0x69, 0x6e, 0x3d,
	0x32, 0x35, 0x20, 0x73, 0x63, 0x65, 0x6e, 0x65, 0x63, 0x75, 0x74, 0x3d,
	0x34, 0x30, 0x20, 0x72,	0x63, 0x3d, 0x61, 0x62, 0x72, 0x20, 0x62, 0x69,
	0x74, 0x72, 0x61, 0x74, 0x65, 0x3d, 0x31, 0x30,	0x20, 0x72, 0x61, 0x74,
	0x65, 0x74, 0x6f, 0x6c, 0x3d, 0x31, 0x2e, 0x30, 0x20, 0x71, 0x63, 0x6f,
	0x6d, 0x70, 0x3d, 0x30, 0x2e, 0x36, 0x30, 0x20, 0x71, 0x70, 0x6d, 0x69,
	0x6e, 0x3d, 0x31, 0x30,	0x20, 0x71, 0x70, 0x6d, 0x61, 0x78, 0x3d, 0x35,
	0x31, 0x20, 0x71, 0x70, 0x73, 0x74, 0x65, 0x70,	0x3d, 0x34, 0x20, 0x69,
	0x70, 0x5f, 0x72, 0x61, 0x74, 0x69, 0x6f, 0x3d, 0x31, 0x2e, 0x34, 0x30,
	0x20, 0x61, 0x71, 0x3d, 0x31, 0x3a, 0x31, 0x2e, 0x30, 0x30, 0x00, 0x80,
	0x00, 0x00, 0x00, 0x01,	0x67, 0x4d, 0x40, 0x0a, 0x9a, 0x74, 0xf4, 0x20,
	0x00, 0x00, 0x03, 0x00, 0x20, 0x00, 0x00, 0x06,	0x51, 0xe2, 0x44, 0xd4,
	0x00, 0x00, 0x00, 0x01, 0x68, 0xee, 0x32, 0xc8, 0x00, 0x00, 0x00, 0x01,
	0x65, 0x88, 0x80, 0x20, 0x00, 0x08, 0x7f, 0xea, 0x6a, 0xe2, 0x99, 0xb6,
	0x57, 0xae, 0x49, 0x30,	0xf5, 0xfe, 0x5e, 0x46, 0x0b, 0x72, 0x44, 0xc4,
	0xe1, 0xfc, 0x62, 0xda, 0xf1, 0xfb, 0xa2, 0xdb,	0xd6, 0xbe, 0x5c, 0xd7,
	0x24, 0xa3, 0xf5, 0xb9, 0x2f, 0x57, 0x16, 0x49, 0x75, 0x47, 0x77, 0x09,
	0x5c, 0xa1, 0xb4, 0xc3, 0x4f, 0x60, 0x2b, 0xb0, 0x0c, 0xc8, 0xd6, 0x66,
	0xba, 0x9b, 0x82, 0x29,	0x33, 0x92, 0x26, 0x99, 0x31, 0x1c, 0x7f, 0x9b,
	0x00, 0x00, 0x01, 0x0ff,
};

static const u8 *codec_h264_mvc_eos_sequence(u32 *len)
{
	*len = ARRAY_SIZE(eos_sequence_mvc);
	return eos_sequence_mvc;
}

/* ---- Module parameters --------------------------------------------------- */

/*
 * mvc_output_mode: 0=frame-packed (default), 1=frame-sequential
 *
 * Frame-packed mode composites view 0 and view 1 into one NV12 buffer:
 * view 0 on top, MVC_FRAME_PACK_GAP blank lines, view 1 on bottom.
 * Total height = H*2+45 for 1080p gives 2205 lines per frame.
 *
 * Frame-sequential mode delivers view 0 and view 1 as separate V4L2
 * capture buffers (original MVP behaviour).
 */
static int mvc_output_mode;
module_param(mvc_output_mode, int, 0644);
MODULE_PARM_DESC(mvc_output_mode, "MVC output: 0=frame-packed (default), 1=frame-sequential");

/*
 * KI-108 P1 (D159) diagnostic: dump the firmware's DPB descriptor + output-list
 * LMEM tables at EOS (drain), to determine whether the ~13 un-announced pictures
 * (D157: fw emits only 2-3 CMD_FRAME_DISPLAY of 16) are still present in the
 * host-readable AMRISC LMEM tables with recoverable output-needed flags
 * (=> a firmware EOS-flush is AUTHORABLE) or are gone/DDR-only (=> terminal).
 * Read-only: no firmware byte is touched, no decode state altered.
 * Default off.  RE map (gxm_h264_mvc.asm):
 *   [0x1e]/[0x1f] = display-ring rp/wp (ring is LMEM[14..29])
 *   [0x6d] = DPB fill count, [0x6f] = output-list end addr, [0x7] = list start idx
 *   [0x2a],[0x47],[0x56] = per-picture/view control
 *   per-view DPB flag tables: base 256 (view0) / 384 (view1); entry i flag word
 *     at base+(i<<3)+1; bit0 = needed-for-output.  Output lists at 512 / 576.
 */
static int mvc_dump_dpb;
module_param(mvc_dump_dpb, int, 0644);
MODULE_PARM_DESC(mvc_dump_dpb, "KI-108 P1 D159: dump DPB/output-list LMEM tables at drain (0=off,1=on)");

/*
 * KI-108 D138 composite source map -- the per-view display index (sub-system 2d).
 *
 * D27's "TERMINAL FINDING" (mono, both-in-v1) is now SUPERSEDED.  With the
 * D134/D135/D136 firmware-authored second-A=5 per-MB pass installed, the firmware
 * DOES reconstruct a DISTINCT, CORRECT dependent view into the view-1 buffer pool
 * (bufs1) and its CMD_FRAME_DISPLAY DOES report a distinct view_id (0 for view0,
 * 1 for view1) -- confirmed on HW (D138, G12B .148): view0 buf=0 view_id=0, view1
 * buf=0 view_id=1.  The firmware's buf_id is a PER-VIEW-RELATIVE DPB slot index
 * (fw display-select 0x0d59/0x0d06: r16 = slot from the shared output list; the
 * per-view descriptor table base 257/385 is used only for min-POC selection), so
 * view0 and view1 legitimately share the same small slot index (v0b==v1b) but the
 * pixels live in DISTINCT pools (view0 -> bufs0, view1 -> bufs1, allocated with
 * distinct contiguous canvas ranges in mvc_do_alloc()).
 *
 * The correct composite therefore reads view0 from bufs0[v0b] and view1 from
 * bufs1[v1b] -- the PER-VIEW SPLIT (map 0), NOT the mono both-in-v1 (map 1).
 * D138 HW proof (jmvc_cavlc, JMVC SW reference, per-view PSNR at the correct
 * blk32-detiled offset): HW view0 == SW view0 BIT-EXACT (inf dB), HW view1 ==
 * SW view1 BIT-EXACT (inf dB), view0 != view1 (duplicate gate passes).  So 2d is
 * a DRIVER-ONLY fix -- default map 0 surfaces the distinctly-reconstructed view1.
 *   0 = per-view split : s0=bufs0[v0b], s1=bufs1[v1b]     (distinct stereo) [default]
 *   1 = both-in-v1     : s0=bufs1[v0b], s1=bufs1[v1b]     (mono duplicate; pre-D138)
 *   2 = swap           : s0=bufs1[v0b], s1=bufs0[v1b]     (diagnostic)
 */
static int mvc_src_map;		/* 0 = per-view split (distinct stereo), D138 default */
module_param(mvc_src_map, int, 0644);
MODULE_PARM_DESC(mvc_src_map, "KI-108 D138 composite source map (0=per-view split[default],1=both-in-v1 mono,2=swap)");

/*
 * KI-108 P1 D159 (approach A): DRIVER-SIDE end-of-stream DPB flush.
 *
 * The MVC firmware decodes all 16 access units of jmvc_cavlc into its per-view
 * DPB slots (bufs0[i]/bufs1[i]) but emits only 4 CMD_FRAME_DISPLAY (POC 0,1 both
 * views) = 2 composite frames delivered; the remaining ~13 access units are
 * decoded-but-un-announced (D159 HW dump: all 16 LMEM DPB slots occupied at EOS,
 * output-list emptied, need_out bit0 cleared for 15/16 -- the firmware discarded
 * the output-pending state without announcing it).  Zero firmware bytes are
 * touched, so there is NO recycle-corruption risk (the case-7 firmware-flush
 * failure mode is entirely avoided): at drain (firmware idle, all pixels already
 * in the DMA capture pools) the driver reads the LMEM DPB descriptor tables,
 * recovers each occupied slot's POC, and delivers the not-yet-delivered slots in
 * POC order through the EXISTING mvc_composite_frame_packed() path.  Default ON
 * (KI-108 P1 D166: full-frame MVC delivery is the fleet default; the D162
 * mvc_flush_guard, also default-on, protects the MVCDS1 wedge-reset path).
 *   mvc_drain_flush : 0=off, 1=on (D166 fleet default)
 *   mvc_poc_off     : LMEM word offset of the POC key within each 8-word DPB
 *                     entry (flag word is at offset 1; POC key derived by the fw
 *                     bump routine from the word at flag+1 -> offset 2).  Runtime-
 *                     tunable so the exact offset/encoding can be corrected from
 *                     the raw-entry dump without a rebuild.
 *   mvc_flush_dump  : 1=print all 8 raw LMEM words per occupied slot at drain
 *                     (diagnostic, to validate mvc_poc_off).
 */
static int mvc_drain_flush = 1;
module_param(mvc_drain_flush, int, 0644);
MODULE_PARM_DESC(mvc_drain_flush, "KI-108 P1 D159 driver-side EOS DPB flush (0=off,1=on[default,D166 fleet default])");
static int mvc_poc_off = 2;
module_param(mvc_poc_off, int, 0644);
MODULE_PARM_DESC(mvc_poc_off, "KI-108 P1 D159 LMEM word offset of POC within the 8-word DPB entry (default 2)");
static int mvc_flush_dump;
module_param(mvc_flush_dump, int, 0644);
MODULE_PARM_DESC(mvc_flush_dump, "KI-108 P1 D159 print raw 8-word DPB entries at drain (0=off[default],1=on)");
/*
 * KI-108 P1 D162: wedge-state guard for the drain-flush.  When on (default),
 * the drain-flush SKIPS its LMEM_DMA drain-read on a wedged / empty decode
 * (a session that delivered 0 frames normally -- e.g. the MVCDS1 large-slice
 * VLD-wedge path), which is what makes flush-on RESET the board (D161): the
 * drain-time LMEM_DMA poll against the wedged VLD hard-hangs the HW.  A decode
 * that delivered >=1 frame made real progress and is safe to flush.  Set to 0
 * ONLY to reproduce the D161 reset for mechanism bisection (see D162).
 */
static int mvc_flush_guard = 1;
module_param(mvc_flush_guard, int, 0644);
MODULE_PARM_DESC(mvc_flush_guard, "KI-108 P1 D162 wedge-state guard on the drain-flush (1=on[default,safe],0=off[repro reset])");
/*
 * D159 approach-A HW RESULT (G12B AML-A311D-CC 8abde1aa5c2c, gxm_h264_mvc.bin
 * f49818ab, kernel 6.12.80-01173-g3b762096d777): with mvc_drain_flush=1 the
 * driver recovers the FULL 16-frame stereo sequence that the firmware decoded
 * but never announced.  jmvc_cavlc 2->16 composite frames, EVERY frame per-view
 * BIT-EXACT vs the JMVC-SW reference (view0=inf, view1=inf dB, POC-ordered
 * HWframe_i==SWf_i, distinct stereo); jmvc_cavlc_iv likewise 16/16 bit-exact.
 * flush=0 leaves production unchanged (jmvc_cavlc 2f md5 26ab68fc, jmvc_cavlc_iv
 * 2f md5 8ba3cf70).  flush=1 x5 back-to-back: no reset, taint held 1026.  The
 * DPB entry layout was read live: flag word at (slot<<3)+1, POC at (slot<<3)+2
 * (mvc_poc_off=2); occupied slots (nonzero flag) held POCs {1..15}, POC 0
 * delivered normally.  So the P1 wall is DRIVER-RECOVERABLE, not firmware-
 * terminal.  Left default OFF: promotion to default-on should first confirm the
 * drain flush emits nothing spurious on the firmware-terminal CABAC-MVC paths.
 */
#define MVC_FRAME_PACK_GAP 45

/* ---- Additional DOS registers not in dos_regs.h -------------------------- */

/*
 * AV_SCRATCH_M and AV_SCRATCH_N: per-view reference buffer addresses.
 * Not defined in dos_regs.h (used only by MVC).
 * BSP source: vh264_mvc.c:169-170.
 */
#define AV_SCRATCH_M		0x2758
#define AV_SCRATCH_N		0x275c

/*
 * ANC0_CANVAS_ADDR: base address of the array of canvas indices
 * written to by init_canvas() for reference frame DPB management.
 * BSP source: vh264_mvc.c:906 (ANC_CANVAS_ADDR = ANC0_CANVAS_ADDR).
 */
#ifndef ANC0_CANVAS_ADDR
#define ANC0_CANVAS_ADDR	0x2640
#endif

/*
 * MC_MPORT_CTRL / MC_MPORT_DAT: motion compensation weight prediction port.
 * Used in H264_DECODE_INIT() to fill the weight prediction table.
 * BSP source: vh264_mvc.c:1384-1387.
 */
#define MC_MPORT_CTRL		0x2590
#define MC_MPORT_DAT		0x2594

/*
 * MBY_MBX (MB_MOTION_MODE) at 0x301c is also written with 0xff at init
 * to signal invalid mb_width to firmware.
 * BSP source: vh264_mvc.c:1389 (WRITE_VREG(MB_WIDTH, 0xff)).
 * Upstream name: MBY_MBX (defined in dos_regs.h at 0x301c -- same register).
 */

/*
 * SLICE_START_BYTE_01 / SLICE_START_BYTE_23: slice start code detection.
 * BSP source: vh264_mvc.c:1391-1392.
 */
#define SLICE_START_BYTE_01	0x3064
#define SLICE_START_BYTE_23	0x3068

/*
 * VLD_ERROR_MASK: disable specific VLD error conditions at init.
 * Value 0x1011: disable COEF_GT_64, error_m4_table, voff_rw_err.
 * BSP source: vh264_mvc.c:1396.
 */
#define VLD_ERROR_MASK		0x3030

/*
 * M4_CONTROL_REG is defined in dos_regs.h (0x30a4).
 * Bit 13 enables H.264 decode mode (same bit in upstream).
 * BSP source: vh264_mvc.c:1424.
 */
#define M4_CONTROL_H264_EN	BIT(13)

/*
 * AMRISC interrupt routing registers for VDEC_1.
 * BSP defines ASSIST_AMR1_INT0 = VDEC_ASSIST_AMR1_INT0 = 0x0025 (VREG word).
 * Upstream byte address = VREG * 4.  dos_regs.h only defines INT8 (0x00b4).
 *
 * BSP source: dos_registers_v1.h:1717-1724 (VDEC_ASSIST_AMR1_INT0..7).
 *             dos_registers.h:23-30 (alias ASSIST_AMR1_INT0..7).
 *             vh264_mvc.c:1400-1409 (H264_DECODE_INIT interrupt setup).
 */
#define VDEC_ASSIST_AMR1_INT0	0x0094	/* 0x0025 * 4 */
#define VDEC_ASSIST_AMR1_INT1	0x0098	/* 0x0026 * 4 */
#define VDEC_ASSIST_AMR1_INT2	0x009c	/* 0x0027 * 4 */
#define VDEC_ASSIST_AMR1_INT3	0x00a0	/* 0x0028 * 4 */
#define VDEC_ASSIST_AMR1_INT4	0x00a4	/* 0x0029 * 4 */
#define VDEC_ASSIST_AMR1_INT5	0x00a8	/* 0x002a * 4: MCPU */
#define VDEC_ASSIST_AMR1_INT6	0x00ac	/* 0x002b * 4: CCPU */
#define VDEC_ASSIST_AMR1_INT7	0x00b0	/* 0x002c * 4 */
/* INT8 = 0x00b4 already in dos_regs.h as VDEC_ASSIST_AMR1_INT8 */

/*
 * CPC_P: CCPU (content CPU) program counter / start address register.
 * BSP source: dos_registers_v1.h:21 (CPC_P = 0x0326, VREG word address).
 * Upstream byte address: 0x0326 * 4 = 0x0c98.
 *
 * CINT_VEC_BASE: CCPU interrupt vector base address register.
 * BSP source: dos_registers_v1.h:20 (CINT_VEC_BASE = 0x0322, VREG word).
 * Upstream byte address: 0x0322 * 4 = 0x0c88.
 */
#define CPC_P			0x0c98	/* 0x0326 * 4 */
#define CINT_VEC_BASE		0x0c88	/* 0x0322 * 4 */
#define VC1_CONTROL_REG		0x3078	/* 0x0c1e * 4; bit 23 needed for GXL MVC */

/*
 * PSCALE_CTRL is defined in dos_regs.h (0x2444).
 * DOS_SW_RESET0 is defined in dos_regs.h (0xfc00).
 * Both are used without redefinition here.
 */

/* ---- Mailbox command values (AV_SCRATCH_0) -------------------------------- */

/*
 * MVC firmware mailbox commands. Firmware writes to MAILBOX_CMD (AV_SCRATCH_0);
 * host responds by writing CMD_DONE (0) to acknowledge.
 * BSP source: vh264_mvc.c:176-180.
 */
#define CMD_DONE		0	/* host ack: processing complete */
#define CMD_ALLOC_VIEW_0	1	/* firmware requests view 0 buffer alloc */
#define CMD_ALLOC_VIEW_1	2	/* firmware requests view 1 buffer alloc */
#define CMD_FRAME_DISPLAY	3	/* firmware signals frame ready for output */
#define CMD_FATAL_ERROR		4	/* firmware hit unrecoverable error */

/* ---- MAILBOX_DATA_0 (AV_SCRATCH_1) field extractors ---------------------- */

/*
 * CMD_ALLOC_VIEW_0/1 data fields in MAILBOX_DATA_0.
 * BSP source: vh264_mvc.c:872-876 (alloc_view_0), 947-951 (alloc_view_1).
 */
#define MBOX_DATA0_LEVEL_IDC(v)		(((v) >> 24) & 0xff)
#define MBOX_DATA0_MAX_REF(v)		(((v) >> 16) & 0xff)
#define MBOX_DATA0_MB_WIDTH(v)		(((v) >> 8) & 0xff)
#define MBOX_DATA0_MB_HEIGHT(v)		((v) & 0xff)

/*
 * CMD_FRAME_DISPLAY data fields in MAILBOX_DATA_0.
 * BSP source: vh264_mvc.c:1057-1059.
 */
#define MBOX_DATA0_VIEW_ID(v)		(((v) >> 6) & 0x3)
#define MBOX_DATA0_BUFF_ID(v)		((v) & 0x3f)
#define MBOX_DATA0_DROP_STATUS(v)	(((v) >> 8) & 0x1)

/* ---- Buffer recycle register (AV_SCRATCH_7) encoding --------------------- */

/*
 * Recycle encoding: (view_id << 8 | buf_id + 1).
 * Register must read 0 (idle) before writing next recycle command.
 * BSP source: vh264_mvc.c:1263-1264, 1275-1276.
 */
#define RECYCLE_IDLE			0
#define RECYCLE_ENCODE(view, buf)	(((view) << 8) | ((buf) + 1))

/*
 * mvc_wait_recycle_idle - wait for firmware to clear AV_SCRATCH_7.
 *
 * The firmware's recycle handler (0x010f) clears AV_SCRATCH_7 after
 * processing a recycle token.  If the firmware is halted (MPSR=0, e.g.
 * during drain or stop_streaming), AV_SCRATCH_7 is never cleared and
 * an unbounded wait would spin forever in the threaded ISR, blocking
 * synchronize_irq() and causing a D-state hang.
 *
 * Returns true if idle, false if timed out (firmware not running).
 */
static bool mvc_wait_recycle_idle(struct amvdec_core *core)
{
	int timeout = 10000;	/* ~10ms at cpu_relax() cadence */

	while (amvdec_read_dos(core, AV_SCRATCH_7) != RECYCLE_IDLE) {
		if (--timeout <= 0)
			return false;
		cpu_relax();
	}
	return true;
}

/* ---- Initialization constants -------------------------------------------- */

/*
 * DECODE_STATUS initial value: 1 = DECODE_START_HEADER.
 * Firmware begins with SPS/PPS header parsing.
 * BSP source: vh264_mvc.c:1447 (WRITE_VREG(DECODE_STATUS, 1)).
 */
#define DECODE_START_HEADER	1
#define DECODE_STATUS_HEADER_DONE 2	/* firmware idle, waiting for VLD data */

/*
 * CURRENT_UCODE initial value: 0xff = no ucode currently loaded.
 * BSP source: vh264_mvc.c:1443.
 */
#define CURRENT_UCODE_NONE	0xff

/*
 * CURRENT_SPS_PPS initial value: 0xffff = no SPS/PPS cached yet.
 * BSP source: vh264_mvc.c:1445.
 */
#define CURRENT_SPS_PPS_NONE	0xffff

/*
 * CANVAS_INDEX_START: canvas base index for legacy VDEC canvas (non-vdec_canvas).
 * BSP source: vh264_mvc.c:182 (#define CANVAS_INDEX_START 0x78).
 */
#define CANVAS_INDEX_START	0x78

/*
 * POWER_CTL_VLD bits (same as regular H.264).
 * BSP source: vh264_mvc.c:1418-1422 (H264_DECODE_INIT).
 */
#define POWER_CTL_VLD_MEM_GATE		BIT(9)	/* VLD memory clock gate */
#define POWER_CTL_VLD_PSCALE_CLK	BIT(6)	/* PSCALE clock enable */
#define POWER_CTL_VLD_CABAC_DIS		BIT(10)	/* disable cabac_step_2 */

/*
 * Workspace size for MVC decoder: 640 KB.
 * BSP source: vh264_mvc.c:199 (DECODER_WORK_SPACE_SIZE = 0xa0000).
 */
#define WORKSPACE_SIZE		0xa0000

/*
 * MVC firmware segment layout within the 28 KB binary.
 * BSP source: vh264_mvc.c:1642-1647.
 *   mc_cpu_addr[0x0000..0x0fff] = header  (from buf[0x1000..0x1fff])
 *   mc_cpu_addr[0x1000..0x2fff] = mmco    (from buf[0x2000..0x3fff])
 *   mc_cpu_addr[0x3000..0x5fff] = slice   (from buf[0x4000..0x6fff])
 *
 * The BSP allocates MC_TOTAL_SIZE=28KB for mc_cpu_addr and points AV_SCRATCH_K at it.
 * The firmware DMA-loads ucode segments from this buffer at runtime.
 * MC_SIZE in upstream vdec_1.c = 16KB, so only buf[0..0x3FFF] is loaded to IMEM.
 * The swap buffer (pointed to by AV_SCRATCH_K) must contain the BSP segment copies.
 *
 * BSP source: vh264_mvc.c:1609-1647.
 */
#define MC_TOTAL_SIZE		(28 * SZ_1K)	/* swap buffer size = full firmware size */
#define MC_SWAP_HEADER_SRC	0x1000		/* header src offset in firmware binary */
#define MC_SWAP_HEADER_DST	0x0000		/* header dst offset in swap buffer */
#define MC_SWAP_HEADER_LEN	0x1000		/* 4KB */
#define MC_SWAP_MMCO_SRC	0x2000		/* MMCO src offset */
#define MC_SWAP_MMCO_DST	0x1000		/* MMCO dst offset */
#define MC_SWAP_MMCO_LEN	0x2000		/* 8KB */
#define MC_SWAP_SLICE_SRC	0x4000		/* slice src offset */
#define MC_SWAP_SLICE_DST	0x3000		/* slice dst offset */
#define MC_SWAP_SLICE_LEN	0x3000		/* 12KB */

/*
 * KI-108 D113 CAVLC-MVC decode-enable load-time firmware patches.
 *
 * The GXL win (D113, board-validated on GXL aml-s905x): the BSP MVC firmware
 * needs TWO load-time byte patches to decode CAVLC MVC for real (per-MB engine
 * runs, MBY_MBX advances, both views reconstructed):
 *
 *   1. M4_CONTROL bit13 (VLD "h264_en").  At AMRISC PC 0x0737 the firmware
 *      executes "clr r8,r8,11" (LE c8 82 c0 09); its lone M4_CONTROL write at
 *      0x073d then commits 0x38 with bit13 CLEAR, so the VLD never enters H.264
 *      slice mode (VLD_STATUS=0x2b wedge).  Replacing 0x0737 with "set r8,r8,13"
 *      (LE 48 83 80 09, only first 3 bytes change) makes the write commit 0x2038
 *      (bit13 set, per-slice bits preserved).  Flat-.bin byte offset 0x1cdc
 *      (PC 0x0737 << 2) is in the HEADER segment (fw[0x1000..0x1fff] ->
 *      swap[0x0000..0x0fff]); swap offset 0x1cdc-0x1000+0x0000 = 0xcdc.
 *
 *   2. 0x1062 slice-skip-gate.  At PC 0x1062 the slice ucode does
 *      "mfcr r8,0x09d3" (read AV_SCRATCH_J, LE 08 d3 49 06); the J-check at
 *      0x1063 diverts EVERY picture to the skip-handler 0x132c when J!=0, so the
 *      per-MB reconstruction loop at 0x1100+ (sole MBY_MBX writer) never runs ->
 *      MBY=0, ~4% luma band.  But J MUST be non-zero to pass the 0x05dd
 *      proceed-gate.  Replacing 0x1062 with "li r8,0" (LE 08 00 80 06, bytes
 *      [1] and [2] change) makes the 0x1063 check always take the DECODE path
 *      while the genuine skip condition ([0x84] gate at 0x106d) and the real
 *      AV_SCRATCH_J register are untouched.  Flat-.bin byte offset 0x4188
 *      (PC 0x1062 << 2) is in the SLICE segment (fw[0x4000..0x6fff] ->
 *      swap[0x3000..0x5fff]); swap offset 0x4188-0x4000+0x3000 = 0x3188.
 *
 * GXM PORT (D113->G12B, this commit): G12A/G12B advertise M264/MVC but load a
 * DIFFERENT firmware (gxm_h264_mvc.bin, sha f49818ab..., NOT gxl_h264_mvc.bin).
 * vici-dasm + asm-diff (ref/vdec/gx[lm]_h264_mvc.asm) confirm gxm shares the
 * ENTIRE CAVLC/MVC decode core with GXL byte-for-byte: PC 0x0737 = "clr r8,r8,11"
 * and PC 0x1062 = "mfcr r8,0x09d3" are IDENTICAL bytes at IDENTICAL flat offsets
 * (the 426 asm-diff lines are confined to the early dispatch/init region, not the
 * M4_CONTROL write, proceed-gate 0x05dd, skip-gate 0x1063/0x106d, skip-handler
 * 0x132c, or MB loop 0x1100, which all match exactly).  So both gxm equivalents
 * live at the SAME offsets -- the guards below (size + exact original bytes)
 * therefore fire identically for the unpatched gxl_h264_mvc.bin AND
 * gxm_h264_mvc.bin, and skip+warn on any patched/different variant (Firmware
 * Binary Integrity policy Rule 0/1 -- source .bin stays byte-identical, the
 * EXECUTED swap-buffer copy is patched, never fail hard).
 *
 * SM1 PORT (D113->SM1, S905D3): SM1 loads yet a THIRD firmware
 * (sm1_h264_mvc.bin, sha 0ef2a060..., size 28672, NOT gxl/gxm).  vici-dasm +
 * asm-diff (ref/vdec/sm1_h264_mvc.asm vs gxl) show sm1 diverges MORE than gxm
 * (4034 asm-diff lines, mostly in the early dispatch/init region) BUT the two
 * patch sites and their load-bearing decode-core context are byte-identical to
 * GXL/GXM:
 *   bit13:     PC 0x0737 "clr r8,r8,11" (c8 82 c0 09) @ flat 0x1cdc, and the lone
 *              M4_CONTROL write at 0x073d, all identical.
 *   skip-gate: PC 0x1062 "mfcr r8,0x09d3" (08 d3 49 06) @ flat 0x4188, with the
 *              0x1063 J-check and 0x106d [0x84] gate identical; the proceed-gate
 *              at 0x05dd is identical too.
 * The ONLY sm1 difference near the patch is the skip-HANDLER destination
 * (sm1: 0x106a branches to 0x1332; gxl: to 0x132c) -- irrelevant because the
 * patched 0x1062->li-r8,0 forces the 0x1063 check onto the DECODE path, so the
 * 0x106a skip branch is never taken on any firmware.  So all three sm1
 * equivalents live at the SAME flat offsets, and the size+exact-bytes guards
 * below fire identically for gxl AND gxm AND sm1.
 */
#define MC_KI108_BIT13_FW_OFFSET   0x1cdc	/* flat-.bin byte offset (PC 0x0737) */
#define MC_KI108_BIT13_SWAP_OFFSET (MC_KI108_BIT13_FW_OFFSET - MC_SWAP_HEADER_SRC + \
				    MC_SWAP_HEADER_DST)	  /* 0xcdc in swap header */
#define MC_KI108_SKIPGATE_FW_OFFSET   0x4188	/* flat-.bin byte offset (PC 0x1062) */
#define MC_KI108_SKIPGATE_SWAP_OFFSET (MC_KI108_SKIPGATE_FW_OFFSET - MC_SWAP_SLICE_SRC + \
				       MC_SWAP_SLICE_DST) /* 0x3188 in swap slice */

/*
 * KI-108 D134: MVC 2c dispatch trampoline over the dependent-slice PARK at
 * flat PC 0x05f3.  PC 0x05f3 is IMEM 0x5f3 (> 0x400 overlay base), so its
 * EXECUTED copy is the HEADER-segment SWAP re-DMA'd from swap[0x0000..0x0fff]
 * (data[0x1000..0x1fff]); patch the swap copy (source .bin byte-identical, Rule
 * 0).  Original 0x05f3 = `b -485 -> 0x040e` (PARK, bytes c0 86 7f 0c).  Replace
 * with `b -1161 -> 0x016a` (bytes c0 dd 7e 0c) -> the guarded resident dispatch
 * block installed at PC 0x016a in mc_addr (vdec_1_load_firmware, D134).  Guard on
 * the exact original 4 bytes (identical across gxl/gxm/sm1, verified D134).
 */
#define MC_KI108_DISPATCH_FW_OFFSET   0x17cc	/* flat-.bin byte offset (PC 0x05f3) */
#define MC_KI108_DISPATCH_SWAP_OFFSET (MC_KI108_DISPATCH_FW_OFFSET - MC_SWAP_HEADER_SRC + \
				       MC_SWAP_HEADER_DST)	/* 0x7cc in swap header */

/*
 * KI-108 D135: MVC 2c dispatch ROUTING trampoline over the dependent-slice DROP
 * at flat PC 0x043c.  The type-20 dependent slice is dropped at the dead view-id
 * filter (`beqi [0x4f],255 -> 0x0442 -> 0x040e`, [0x4f]=0xFF sentinel, D22) and
 * never reaches recon; HW (D134) proved D134's 0x05f3 unpark is downstream of
 * this drop.  PC 0x043c is IMEM 0x43c (> 0x400 overlay base), so its EXECUTED
 * copy is the HEADER-segment SWAP re-DMA'd from swap[0x0000..0x0fff]; patch the
 * swap copy (source .bin byte-identical, Rule 0).  Original 0x043c =
 * `beqi r8,255,+6 -> 0x0442` (bytes c8 bf a1 0a).  Replace with
 * `b -714 -> 0x0172` (bytes 80 4d 7f 0c) -> the guarded resident route block
 * installed at PC 0x0172 in mc_addr (vdec_1_load_firmware, D135), which sets
 * [0x56]=1 and routes the type-20 slice into recon (0x0446 -> 0x044d -> 0x0456
 * -> 0x05d0).  Guard on the exact original 4 bytes (identical across gxl/gxm/sm1,
 * verified D135 -- routing region diff gxm==gxl byte-identical).
 */
#define MC_KI108_ROUTE_FW_OFFSET      0x10f0	/* flat-.bin byte offset (PC 0x043c) */
#define MC_KI108_ROUTE_SWAP_OFFSET    (MC_KI108_ROUTE_FW_OFFSET - MC_SWAP_HEADER_SRC + \
				       MC_SWAP_HEADER_DST)	/* 0xf0 in swap header */

/*
 * KI-108 D136: MVC second-A=5 re-entry trampoline over the A=3 issue at flat PC
 * 0x0690 (`itcr 0x03,AV_SCRATCH_A`).  After D135 routes the dependent slice into
 * recon-setup it reaches 0x0690 but the resulting per-MB grid is skipped because
 * [0x5b]==0 (dependent slice carries the base view's frame_num/POC -> the
 * change-detector at fw 0x0611-0x0646 computes 0), so the C-core grid-counter
 * re-arm at fw 0x107e (gated 0x102d `beqi r17,0`) never runs and MBY stays frozen.
 * PC 0x0690 is IMEM 0x690 (> 0x400 overlay base), so its EXECUTED copy is the
 * HEADER-segment SWAP re-DMA'd from swap[0x0000..0x0fff]; patch the swap copy
 * (source .bin byte-identical, Rule 0).  Original 0x0690 = `itcr 0x03,0x09ca`
 * (bytes 03 ca 09 08, byte-identical gxl/gxm/sm1).  Replace with `b -1302 ->
 * 0x017a` (bytes 80 ba 7e 0c) -> the guarded resident re-entry block installed at
 * PC 0x017a in mc_addr (vdec_1_load_firmware, D136), which for the dependent view
 * ([0x56]==1) forces [0x5b]=2 so the second per-MB grid runs, re-issues A=3 (the
 * original 0x0690 instruction), and returns to 0x0691.  fw 0x068f (sta r17,[0x5b])
 * is the DELAY SLOT of the 0x068e call and is NOT patched -- 0x0690 runs after it.
 */
#define MC_KI108_REENTRY_FW_OFFSET    0x1a40	/* flat-.bin byte offset (PC 0x0690) */
#define MC_KI108_REENTRY_SWAP_OFFSET  (MC_KI108_REENTRY_FW_OFFSET - MC_SWAP_HEADER_SRC + \
				       MC_SWAP_HEADER_DST)	/* 0xa40 in swap header */

/*
 * Dynamic buffer margin: extra display buffers beyond DPB max.
 * BSP source: vh264_mvc.c:194 (dynamic_buf_num_margin = 8).
 */
#define DYNAMIC_BUF_MARGIN	8

/*
 * Maximum decode buffers per view.
 * BSP source: vh264_mvc.c:195 (DECODE_BUFFER_NUM_MAX = 16).
 */
#define MVC_MAX_BUFS		16

/*
 * Timer interval: 10 ms, matching the BSP put-timer (PUT_INTERVAL = HZ/100,
 * vh264_mvc.c:61).  This timer drives BOTH the (neutered) progress watchdog
 * AND the VIFIFO read-pointer coupling (mvc_set_rp, KI-108 D3) -- the BSP runs
 * the RP coupling at this same 10 ms cadence, which is load-bearing for
 * advancing past view-0 MB 0 on large multi-AU streams.
 */
#define CHECK_INTERVAL		msecs_to_jiffies(10)

/* ---- Per-buffer descriptor ----------------------------------------------- */

/**
 * struct mvc_buf - per-buffer DMA descriptor for one view's decode buffer pool
 * @y_paddr:  physical address of Y (luma) plane
 * @u_paddr:  physical address of U (Cb) plane
 * @v_paddr:  physical address of V (Cr) plane
 * @paddr:    physical address of the contiguous Y+U+V block
 * @vaddr:    virtual address of the contiguous Y+U+V block
 * @size:     total allocation size in bytes
 * @canvas_y: canvas ID for Y plane
 * @canvas_u: canvas ID for U plane
 * @canvas_v: canvas ID for V plane
 * @used:     true if this slot is currently in use by firmware
 */
struct mvc_buf {
	dma_addr_t	paddr;
	void		*vaddr;
	size_t		size;
	dma_addr_t	y_paddr;
	dma_addr_t	u_paddr;
	dma_addr_t	v_paddr;
	u8		canvas_y;
	u8		canvas_u;
	u8		canvas_v;
	bool		used;
};

/* ---- POC pairing table --------------------------------------------------- */

#define PAIR_POOL_SIZE	20

/**
 * struct mvc_pair - tracks one POC's left/right buffer IDs for pairing
 * @display_poc: picture order count used to match view 0 and view 1
 * @view0_buf:   view 0 buffer index (-1 = not yet received)
 * @view1_buf:   view 1 buffer index (-1 = not yet received)
 * @used:        true if this slot is occupied
 */
struct mvc_pair {
	int	display_poc;
	int	view0_buf;
	int	view1_buf;
	bool	used;
};

/* ---- Private codec state ------------------------------------------------- */

/**
 * struct codec_h264_mvc - private state for one MVC decode session
 *
 * @workspace_vaddr:  kernel VA of the 640 KB firmware workspace
 * @workspace_paddr:  physical address of workspace (written to AV_SCRATCH_E)
 * @swap_vaddr:       kernel VA of the swap segment (MC_SWAP_SIZE bytes)
 * @swap_paddr:       physical address (written to AV_SCRATCH_K = UCODE_START_ADDR)
 * @ref_paddr:        physical addresses of per-view reference buffers
 * @ref_vaddr:        kernel VAs of per-view reference buffers
 * @ref_size:         size in bytes of each view's reference buffer
 * @bufs0:            display buffer pool for view 0 (left eye)
 * @bufs1:            display buffer pool for view 1 (right eye; recycled in MVP)
 * @num_bufs0:        number of allocated view 0 buffers
 * @num_bufs1:        number of allocated view 1 buffers
 * @mb_width:         macroblock columns (filled on CMD_ALLOC_VIEW_0)
 * @mb_height:        macroblock rows (filled on CMD_ALLOC_VIEW_0)
 * @max_ref:          max reference frames from firmware
 * @dpb0:             DPB size for view 0 (filled by alloc handler)
 * @dpb1:             DPB size for view 1 (filled by alloc handler)
 * @pairs:            POC pairing table for matching view 0 and view 1 frames
 * @check_timer:      watchdog timer for decode stall detection
 * @sess:             back-pointer to session for timer callback
 * @last_pic_count:   previous value of AV_SCRATCH_9 (frame counter)
 * @alloc_view:       view being allocated in threaded ISR (0 or 1)
 * @views_allocated:  number of views fully allocated (0, 1, or 2)
 * @initialized:      true after BOTH views are fully allocated
 */
struct codec_h264_mvc {
	void		*workspace_vaddr;
	dma_addr_t	 workspace_paddr;

	void		*swap_vaddr;
	dma_addr_t	 swap_paddr;

	dma_addr_t	 ref_paddr[2];
	void		*ref_vaddr[2];
	size_t		 ref_size[2];

	struct mvc_buf	 bufs0[MVC_MAX_BUFS];
	struct mvc_buf	 bufs1[MVC_MAX_BUFS];
	int		 num_bufs0;
	int		 num_bufs1;

	/* Stream geometry -- filled on CMD_ALLOC_VIEW_0 */
	u32		 mb_width;
	u32		 mb_height;
	u32		 max_ref;
	int		 dpb0;
	int		 dpb1;

	/* POC pairing table */
	struct mvc_pair	 pairs[PAIR_POOL_SIZE];

	/* Watchdog */
	struct timer_list check_timer;
	struct amvdec_session *sess;
	u32		 last_pic_count;

	/* Last timestamp from view 0 delivery -- reused for view 1 */
	u64		 last_ts;
	struct v4l2_timecode last_tc;
	u32		 last_flags;

	/* Which view is being allocated in the threaded ISR */
	int		 alloc_view;
	int		 views_allocated;	/* 0, 1, or 2 -- only stall-abort after both */
	bool		 initialized;
	bool		 dpb_primed;	/* KI-108 D1q: pool handed to firmware once */
	bool		 d138_logged;	/* KI-108 D138: per-view display-index announce once */
	bool		 alloc1_deferred; /* KI-108 D17: view-1 CMD_DONE held until CAPTURE STREAMON */

	/*
	 * KI-108 P1 D159 (approach A): POCs already delivered via the normal
	 * CMD_FRAME_DISPLAY path, so the EOS drain-flush does not re-deliver them.
	 */
	s16		 delivered_poc[MVC_MAX_BUFS * 2];
	int		 n_delivered_poc;
};

/* ---- DPB size calculation ------------------------------------------------ */

/*
 * Compute the maximum DPB (Decoded Picture Buffer) size for a given H.264
 * level and reference frame count.  Used to size the per-view decode buffer
 * pools.
 *
 * BSP source: vh264_mvc.c:772-841 (get_max_dec_frame_buf_size).
 */
static int mvc_max_dpb_size(int level_idc, int max_ref, int mb_width, int mb_height)
{
	int pic_size = mb_width * mb_height * 384;
	int size;

	switch (level_idc) {
	case 9:  case 10: size = 152064;   break;
	case 11: size = 345600;   break;
	case 12: case 13: case 20: size = 912384;   break;
	case 21: size = 1824768;  break;
	case 22: size = 3110400;  break;
	case 30: size = 3110400;  break;
	case 31: size = 6912000;  break;
	case 32: size = 7864320;  break;
	case 40: case 41: size = 12582912; break;
	case 42: size = 13369344; break;
	case 50: size = 42393600; break;
	case 51: size = 70778880; break;
	default: size = 12582912; break;	/* default to level 4.0 budget */
	}

	if (pic_size == 0)
		return max_ref;

	size = size / pic_size + 1;	/* BSP: +1 for MVC inter-view ref */
	if (max_ref > size)
		size = max_ref;
	if (size > MVC_MAX_BUFS)
		size = MVC_MAX_BUFS;

	return size;
}

/* ---- Canvas helpers ------------------------------------------------------ */

/*
 * canvas_config_blk32 - configure a canvas with 32x32 block mode and
 * optionally write the G12A+ block-mode shadow register.
 *
 * MVC uses CANVAS_BLKMODE_32x32 (not linear) for all planes.
 * BSP source: vh264_mvc.c:748-765 (config_cav_lut_ex calls in init_canvas).
 */
static void canvas_config_blk32(struct amvdec_core *core, u8 canvas_id,
				 dma_addr_t paddr, u32 width, u32 height)
{
	meson_canvas_config(core->canvas, canvas_id, paddr,
			    width, height,
			    MESON_CANVAS_WRAP_NONE,
			    MESON_CANVAS_BLKMODE_32x32,
			    MESON_CANVAS_ENDIAN_SWAP64);

	/*
	 * On G12A+, write block-mode shadow register after each canvas config.
	 * BSP uses config_cav_lut_ex which handles this internally.
	 * Bit 11: write-enable, bit 10: blkmode, bit 8: index write-enable, [7:0]: index.
	 */
	if (core->platform->revision >= VDEC_REVISION_G12A)
		amvdec_write_dos(core, VDEC_ASSIST_CANVAS_BLK32,
				 CANVAS_BLK32_WR |
				 (MESON_CANVAS_BLKMODE_32x32 << CANVAS_BLK32_BIT) |
				 CANVAS_INDEX_WR | canvas_id);
}

/*
 * mvc_alloc_view_buffers - allocate display buffer pool for one view.
 *
 * Allocates num_bufs contiguous DMA buffers (Y+U+V in one alloc),
 * assigns canvas IDs starting at canvas_base, configures the canvases,
 * and writes ANC_CANVAS_ADDR entries.
 *
 * @core:        amvdec core
 * @bufs:        output array of mvc_buf descriptors (size >= num_bufs)
 * @num_bufs:    number of display buffers to allocate
 * @canvas_base: first canvas ID to use (successive IDs used for Y, U, V)
 * @anc_base:    base register for ANC canvas array entries
 * @mb_width:    macroblock columns (already 4-aligned)
 * @mb_height:   macroblock rows (already 4-aligned)
 *
 * BSP source: vh264_mvc.c:684-770 (init_canvas, inner loop over j).
 */
static int mvc_alloc_view_buffers(struct amvdec_core *core,
				  struct mvc_buf *bufs, int num_bufs,
				  int canvas_base, u32 anc_base,
				  u32 mb_width, u32 mb_height)
{
	u32 y_size  = mb_width * mb_height * 256;	/* mb_total << 8 */
	u32 uv_size = mb_width * mb_height * 64;	/* mb_total << 6 */
	u32 frame_size = y_size + uv_size * 2;
	int canvas_idx = canvas_base;
	int i;

	for (i = 0; i < num_bufs; i++) {
		struct mvc_buf *b = &bufs[i];
		int ret;

		b->size = PAGE_ALIGN(frame_size);
		b->vaddr = dma_alloc_coherent(core->dev, b->size,
					      &b->paddr, GFP_KERNEL);
		if (!b->vaddr) {
			dev_err(core->dev,
				"mvc: view buf %d alloc failed (%zu bytes)\n",
				i, b->size);
			return -ENOMEM;
		}

		b->y_paddr  = b->paddr;
		b->u_paddr  = b->paddr + y_size;
		b->v_paddr  = b->paddr + y_size + uv_size;

		/* Allocate canvas IDs */
		ret = meson_canvas_alloc(core->canvas, &b->canvas_y);
		if (ret) {
			dev_err(core->dev, "mvc: canvas_y alloc failed: %d\n", ret);
			dma_free_coherent(core->dev, b->size, b->vaddr, b->paddr);
			b->vaddr = NULL;
			return ret;
		}
		ret = meson_canvas_alloc(core->canvas, &b->canvas_u);
		if (ret) {
			dev_err(core->dev, "mvc: canvas_u alloc failed: %d\n", ret);
			meson_canvas_free(core->canvas, b->canvas_y);
			dma_free_coherent(core->dev, b->size, b->vaddr, b->paddr);
			b->vaddr = NULL;
			return ret;
		}
		ret = meson_canvas_alloc(core->canvas, &b->canvas_v);
		if (ret) {
			dev_err(core->dev, "mvc: canvas_v alloc failed: %d\n", ret);
			meson_canvas_free(core->canvas, b->canvas_u);
			meson_canvas_free(core->canvas, b->canvas_y);
			dma_free_coherent(core->dev, b->size, b->vaddr, b->paddr);
			b->vaddr = NULL;
			return ret;
		}

		/*
		 * KI-108 D19: the firmware addresses output canvases as
		 * CANVAS_START + buf*3 (+1/+2), so the dynamically-allocated IDs
		 * MUST be contiguous (Y=base+i*3, U=Y+1, V=Y+2).  meson_canvas
		 * hands out sequential free IDs, so this holds in practice; warn
		 * loudly if a fragmented pool ever breaks it (decoded pixels would
		 * land on the wrong canvas again).
		 */
		if (b->canvas_u != b->canvas_y + 1 ||
		    b->canvas_v != b->canvas_y + 2 ||
		    b->canvas_y != bufs[0].canvas_y + i * 3)
			dev_warn(core->dev,
				 "mvc: D19 NON-CONTIGUOUS canvas at buf %d (y=0x%02x u=0x%02x v=0x%02x base=0x%02x) -- output will be wrong\n",
				 i, b->canvas_y, b->canvas_u, b->canvas_v,
				 bufs[0].canvas_y);

		canvas_config_blk32(core, b->canvas_y, b->y_paddr,
				    mb_width * 16, mb_height * 16);
		canvas_config_blk32(core, b->canvas_u, b->u_paddr,
				    mb_width * 8, mb_height * 8);
		canvas_config_blk32(core, b->canvas_v, b->v_paddr,
				    mb_width * 8, mb_height * 8);

		/*
		 * Write ANC (anchor) canvas address array entry.
		 * Packs Y, U, V canvas IDs into one 32-bit register.
		 * BSP source: vh264_mvc.c:716-718.
		 */
		amvdec_write_dos(core, anc_base + i * 4,
				 b->canvas_y |
				 (b->canvas_u << 8) |
				 (b->canvas_v << 16));

		canvas_idx += 3;
		b->used = false;

		dev_dbg(core->dev,
			"mvc: view buf %d: paddr=%pad size=%zu canvas=%02x/%02x/%02x\n",
			i, &b->paddr, b->size,
			b->canvas_y, b->canvas_u, b->canvas_v);
	}

	return 0;
}

/*
 * mvc_free_view_buffers - free all buffers allocated by mvc_alloc_view_buffers.
 */
static void mvc_free_view_buffers(struct amvdec_core *core,
				  struct mvc_buf *bufs, int num_bufs)
{
	int i;

	for (i = 0; i < num_bufs; i++) {
		struct mvc_buf *b = &bufs[i];

		if (!b->vaddr)
			continue;

		meson_canvas_free(core->canvas, b->canvas_v);
		meson_canvas_free(core->canvas, b->canvas_u);
		meson_canvas_free(core->canvas, b->canvas_y);
		dma_free_coherent(core->dev, b->size, b->vaddr, b->paddr);
		b->vaddr = NULL;
	}
}

/* ---- Hardware init ------------------------------------------------------- */

/*
 * mvc_h264_decode_init - VDEC_1 hardware decoder reset and init.
 *
 * Performs the DOS_SW_RESET0 pulse sequence, fills weight prediction table,
 * and configures AMRISC interrupt assignments.  Mirrors BSP H264_DECODE_INIT().
 *
 * BSP source: vh264_mvc.c:1344-1448.
 */
static void mvc_h264_decode_init(struct amvdec_core *core,
				 struct codec_h264_mvc *mvc)
{
	int i;

	/*
	 * KI-108 D71: save AV_SCRATCH_J (CR 0x09d3) across the DOS reset and
	 * RESTORE it -- as the BSP does.  It is NOT just a skip count: the slice
	 * ucode uses it as a PROCEED-GATE at firmware 0x05dd, reached before the
	 * per-MB decode:
	 *     05dd: mfcr r8, 0x09d3            ; r8 = AV_SCRATCH_J
	 *     05de: bneqi r8,0,+4  -> 05e2     ; J != 0 -> PROCEED to decode setup
	 *     05df: lda  r8,[0x84]             ; J == 0:
	 *     05e0: bneqi r8,0,+2  -> 05e2     ;   lmem[0x84] != 0 -> proceed
	 *     05e1: b    -452      -> 041d     ;   lmem[0x84] == 0 -> BACK TO SCAN LOOP
	 * The prior "D1z" change force-zeroed J on the theory it was only a skip
	 * count (the 0x1062 path), missing this gate.  Forcing it to 0 sent every
	 * stream whose lmem[0x84]==0 here (MVCDS1, MVCSPS-1) back into the VLD scan
	 * loop forever -- the wedge (RP stuck at 512, bit5 clear, MBY=0).  MVCDS-4
	 * has [0x84]!=0 so it proceeded either way, which masked the bug.
	 */
	i = amvdec_read_dos(core, AV_SCRATCH_J);

	/* DOS reset pulse: bits 7, 6, 4 (IQIDCT, MC, VLD_PART) */
	amvdec_write_dos(core, DOS_SW_RESET0, BIT(7) | BIT(6) | BIT(4));
	amvdec_write_dos(core, DOS_SW_RESET0, 0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);

	amvdec_write_dos(core, DOS_SW_RESET0, BIT(7) | BIT(6) | BIT(4));
	amvdec_write_dos(core, DOS_SW_RESET0, 0);

	/* Second pulse: bits 9, 8 (clear more internal state) */
	amvdec_write_dos(core, DOS_SW_RESET0, BIT(9) | BIT(8));
	amvdec_write_dos(core, DOS_SW_RESET0, 0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);

	/* KI-108 D71: restore the saved value (proceed-gate, see above) -- NOT 0. */
	amvdec_write_dos(core, AV_SCRATCH_J, i);

	/*
	 * Fill weight prediction table (192 entries of 0x100).
	 * BSP source: vh264_mvc.c:1384-1387.
	 */
	amvdec_write_dos(core, MC_MPORT_CTRL, 0x0300);
	for (i = 0; i < 192; i++)
		amvdec_write_dos(core, MC_MPORT_DAT, 0x100);
	amvdec_write_dos(core, MC_MPORT_CTRL, 0);

	/*
	 * Signal invalid mb_width to firmware (BSP H264_DECODE_INIT:1389).
	 * D1s fix: this is MB_WIDTH (CR 0x0c09 = byte 0x3024), NOT MBY_MBX
	 * (CR 0x0c07/MB_MOTION_MODE = byte 0x301c). The prior code wrote the
	 * wrong register. Benign for this stream (the header ucode at 0x0752
	 * overwrites MB_WIDTH from the parsed SPS [0x2c] unconditionally), but
	 * the write must target the correct register.
	 */
	amvdec_write_dos(core, MB_WIDTH, 0xff);

	/* Slice start code registers */
	amvdec_write_dos(core, SLICE_START_BYTE_01, 0x00000000);
	amvdec_write_dos(core, SLICE_START_BYTE_23, 0x01010000);

	/* Set to MPEG-2 mode to enable mismatch logic */
	amvdec_write_dos(core, MPEG1_2_REG, 1);

	/* Disable COEF_GT_64, error_m4_table, voff_rw_err */
	amvdec_write_dos(core, VLD_ERROR_MASK, 0x1011);

	/*
	 * AMRISC interrupt routing: assign hardware interrupt sources to AMRISC INT lines.
	 * BSP source: vh264_mvc.c:1400-1409 (H264_DECODE_INIT).
	 * Register addresses: VREG word addr * 4 = upstream byte addr.
	 *   VDEC_ASSIST_AMR1_INT0 = 0x0025 * 4 = 0x0094
	 * NOTE: The previous code used 0x00a0..0x00b8 (VREG 0x0028..0x002e = INT3..INT8),
	 *       shifted by 3 VREGs. Corrected to 0x0094..0x00b0 (INT0..INT7).
	 */
	amvdec_write_dos(core, VDEC_ASSIST_AMR1_INT0, 0x1);	/* viu_vsync_int */
	amvdec_write_dos(core, VDEC_ASSIST_AMR1_INT1, 0x5);	/* mbox_isr */
	amvdec_write_dos(core, VDEC_ASSIST_AMR1_INT2, 0x8);	/* vld_isr */
	amvdec_write_dos(core, VDEC_ASSIST_AMR1_INT3, 0x15);	/* vififo_empty */
	amvdec_write_dos(core, VDEC_ASSIST_AMR1_INT4, 0xd);	/* rv_ai_mb_finished_int */
	amvdec_write_dos(core, VDEC_ASSIST_AMR1_INT7, 0x14);	/* dcac_dma_done */
	amvdec_write_dos(core, VDEC_ASSIST_AMR1_INT5, 0x9);	/* MCPU interrupt */
	amvdec_write_dos(core, VDEC_ASSIST_AMR1_INT6, 0x17);	/* CCPU interrupt */

	/*
	 * CCPU start address and interrupt vector base.
	 * BSP: WRITE_VREG(CPC_P, 0xc00) -> CPC_P = 0x0326 -> byte = 0x0c98.
	 * BSP: WRITE_VREG(CINT_VEC_BASE, 0xc20>>5) -> CINT_VEC_BASE = 0x0322 -> byte = 0x0c88.
	 * Previous code used 0x0c14/0x0c18 (VREG 0x0305/0x0306), which are wrong.
	 * BSP source: vh264_mvc.c:1411-1412.
	 */
	amvdec_write_dos(core, CPC_P, 0xc00);		/* CCPU starts at 0xc00 */
	amvdec_write_dos(core, CINT_VEC_BASE, (0xc20 >> 5));	/* CCPU int vector base */

	/*
	 * POWER_CTL_VLD: enable VLD memory and PSCALE clocks, disable cabac_step_2.
	 * BSP source: vh264_mvc.c:1418-1422.
	 */
	amvdec_write_dos(core, POWER_CTL_VLD,
			 POWER_CTL_VLD_CABAC_DIS |
			 POWER_CTL_VLD_MEM_GATE  |
			 POWER_CTL_VLD_PSCALE_CLK);

	/* Enable H.264 decode mode in M4_CONTROL */
	amvdec_write_dos(core, M4_CONTROL_REG, M4_CONTROL_H264_EN);

	/*
	 * CANVAS_START = 0 (vdec_canvas mode; upstream always uses meson_canvas).
	 * BSP sets 0x78 for legacy canvas, 0 for vdec_canvas.
	 * BSP source: vh264_mvc.c:1426-1429.
	 */
	amvdec_write_dos(core, AV_SCRATCH_6, 0);

	/* Workspace address */
	amvdec_write_dos(core, AV_SCRATCH_E, mvc->workspace_paddr);

	/* SPS / PPS / ucode / SPS-PPS init */
	amvdec_write_dos(core, AV_SCRATCH_B, 0);		/* SPS bitmap */
	amvdec_write_dos(core, AV_SCRATCH_C, 0);		/* PPS bitmap */
	amvdec_write_dos(core, AV_SCRATCH_H, CURRENT_UCODE_NONE);
	amvdec_write_dos(core, AV_SCRATCH_I, CURRENT_SPS_PPS_NONE);
	amvdec_write_dos(core, AV_SCRATCH_A, DECODE_START_HEADER);
}

/*
 * mvc_prot_init - enable firmware and clear mailbox.
 *
 * Called after H264_DECODE_INIT. Waits for DMA idle, clears and enables
 * mailbox interrupt, disables post-scaler, runs RESET0 bit 11, clears
 * mailbox registers and drop/recycle controls.
 *
 * BSP source: vh264_mvc.c:1450-1488.
 */
static void mvc_prot_init(struct amvdec_core *core)
{
	/*
	 * Wait for DCAC and LMEM DMA engines to go idle before init.
	 * BSP source: vh264_mvc.c:1452-1455.  Bit 15 = DMA busy.
	 * Without this wait, firmware init can race with a stale DMA
	 * from a previous session.
	 */
	while (amvdec_read_dos(core, DCAC_DMA_CTRL) & 0x8000)
		;
	while (amvdec_read_dos(core, LMEM_DMA_CTRL) & 0x8000)
		;

	/* Disable post-scaler (PSCALE not used for MVC) */
	amvdec_write_dos(core, PSCALE_CTRL, 0);

	/* Clear, then enable mailbox interrupt */
	amvdec_write_dos(core, ASSIST_MBOX1_CLR_REG, 1);
	amvdec_write_dos(core, ASSIST_MBOX1_MASK, 1);

	/* Reset MCPU (bit 11) -- BSP source: vh264_mvc.c:1467-1473 */
	amvdec_write_dos(core, DOS_SW_RESET0, BIT(11));
	amvdec_write_dos(core, DOS_SW_RESET0, 0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);

	/* Clear mailbox command and recycle/drop registers */
	amvdec_write_dos(core, AV_SCRATCH_0, CMD_DONE);
	amvdec_write_dos(core, AV_SCRATCH_7, RECYCLE_IDLE);
	amvdec_write_dos(core, AV_SCRATCH_8, 0);

	/* Clear MDEC_PIC_DC_CTRL bits 17 and 16 */
	amvdec_clear_dos_bits(core, MDEC_PIC_DC_CTRL, BIT(17) | BIT(16));

	/* Set DC threshold -- BSP source: vh264_mvc.c:1487 */
	amvdec_write_dos(core, MDEC_PIC_DC_THRESH, 0x404038aa);

	/*
	 * On G12A+, clear MMC_CTRL1 bit 3 for non-compressed memory mode.
	 * BSP doesn't do this explicitly but upstream H.264 does.
	 */
	if (core->platform->revision >= VDEC_REVISION_G12A)
		amvdec_clear_dos_bits(core, VDEC_ASSIST_MMC_CTRL1, BIT(3));
}

/* ---- VIFIFO read-pointer coupling (KI-108 D3) ---------------------------- */

/*
 * PARSER_VIDEO_RP: parser-block video read pointer, one word above
 * PARSER_VIDEO_WP (0x88).  BSP dos_registers_v1.h: WP=0x2982, RP=0x2983.
 */
#define PARSER_VIDEO_RP		0x8c

/*
 * mvc_set_rp - couple the VDEC VIFIFO read pointer into the parser's
 * PARSER_VIDEO_RP, mirroring the BSP mvc_set_rp() (vh264_mvc.c:1026 ->
 * parser_set_rp -> WRITE_PARSER_REG(PARSER_VIDEO_RP, READ_VREG(
 * VLD_MEM_VIFIFO_RP))).  The BSP runs this on every ISR (vh264mvc_isr) AND
 * every 10ms put-timer tick (vh264mvc_put_timer_func), so the esparser /
 * stream-buffer always tracks how far the firmware has consumed.
 *
 * Our V4L2 feed path had NO such coupling (vdec_1_stbuf_power_up auto-fill
 * only), so a large multi-AU stream (MVCDS1.264, 6.8 MB) dumped whole into
 * the VIFIFO wedged slice/MB parse at view-0 MB 0: the VIFIFO stayed full
 * (LEVEL=0x691f80) and the firmware never advanced past MB 0, while a tiny
 * single-AU clip (MVCDS-4.264) decoded fine.  See KI-108 D3 clean-baseline
 * capture (ki108-fix-approach.md).
 */
static void mvc_set_rp(struct amvdec_core *core)
{
	amvdec_write_parser(core, PARSER_VIDEO_RP,
			    amvdec_read_dos(core, VLD_MEM_VIFIFO_RP));
}

/* ---- Watchdog timer ------------------------------------------------------ */

static void codec_h264_mvc_check_timer(struct timer_list *timer)
{
	struct codec_h264_mvc *mvc = timer_container_of(mvc, timer, check_timer);
	struct amvdec_session *sess = mvc->sess;
	struct amvdec_core *core = sess->core;
	u32 pic_count;

	/* KI-108 D3: mirror the BSP put-timer, which couples the VIFIFO RP into
	 * the parser every 10ms so the firmware can advance past view-0 MB 0. */
	mvc_set_rp(core);

	/*
	 * KI-108 D17: release the held view-1 CMD_DONE once the CAPTURE queue is
	 * streaming, so the firmware only starts decoding/displaying when V4L2
	 * capture buffers exist.  Single serialization point -- race-free vs the
	 * threaded-ISR alloc path and the CAPTURE-STREAMON resume.
	 */
	if (mvc->alloc1_deferred && sess->streamon_cap) {
		mvc->alloc1_deferred = false;
		amvdec_write_dos(core, AV_SCRATCH_0, CMD_DONE);
		dev_warn(core->dev,
			 "mvc: D17 CAPTURE streaming -- released view-1 CMD_DONE, firmware may decode\n");
	}

	pic_count = amvdec_read_dos(core, AV_SCRATCH_9);

	/*
	 * KI-108 D1v: buffer recycle is fully INLINE and ISR-driven, NOT timer-
	 * driven (the watchdog runs at 100ms, the parser at ~1 frame/ms, so a
	 * timer-based recycle can never keep pace). Two inline stages, both in the
	 * threaded ISR:
	 *   1. Bootstrap: mvc_do_alloc() hands the firmware its WHOLE free-buffer
	 *      pool the instant view-1 alloc completes (before the parse races).
	 *   2. Sustain:   mvc_handle_display() recycles each displayed buffer the
	 *      moment CMD_FRAME_DISPLAY fires (single-slot, gated on
	 *      AV_SCRATCH_7==0, mirroring the BSP put-timer recycle loop).
	 * This check_timer is now watchdog-only; it holds NO recycle logic. (D1q's
	 * one-time bootstrap lived here and fired ~300ms too late -- D1o/D1q.)
	 */

	/*
	 * Debug: dump key firmware state registers every watchdog tick.
	 * Helps diagnose firmware stalls during development.
	 * BSP: MPSR=MCPU PC, CPSR=CCPU PC, AV_SCRATCH_A=DECODE_STATUS,
	 *      AV_SCRATCH_H=CURRENT_UCODE, AV_SCRATCH_0=MAILBOX_CMD.
	 */
	dev_dbg(core->dev,
		"mvc: watchdog: MPSR=0x%04x CPSR=0x%04x STATUS=%u UCODE=%u CMD=%u pic=%u init=%d\n",
		 amvdec_read_dos(core, MPSR),
		 amvdec_read_dos(core, CPSR),
		 amvdec_read_dos(core, AV_SCRATCH_A),
		 amvdec_read_dos(core, AV_SCRATCH_H),
		 amvdec_read_dos(core, AV_SCRATCH_0),
		 pic_count,
		 mvc->initialized);

	/*
	 * Only abort on stall after BOTH views are allocated.  During
	 * CMD_ALLOC_VIEW_1 processing the firmware is blocked in the MCPU
	 * waiting for the host ack, so pic_count naturally stays zero.
	 * Aborting during this window breaks the bootstrap.
	 */
	if (pic_count == mvc->last_pic_count && mvc->views_allocated >= 2) {
		u32 status = amvdec_read_dos(core, AV_SCRATCH_A);

		/*
		 * Firmware idle -- not a stall.  Two idle states:
		 * STATUS=2: waiting for VLD data (input exhausted)
		 * STATUS=7: recycle wait after last frame (no more input)
		 *
		 * In both cases, if esparser has no queued buffers, the
		 * decode is complete.  Let the drain path handle teardown.
		 */
		if (status == DECODE_STATUS_HEADER_DONE ||
		    (status == 7 &&
		     atomic_read(&sess->esparser_queued_bufs) <= 0)) {
			mod_timer(&mvc->check_timer, jiffies + CHECK_INTERVAL);
			return;
		}

		/* KI-108 D1k: do NOT abort during the recycle-feed experiment --
		 * keep the firmware running so the PC-trace can observe A 7->4->5. */
		dev_warn(core->dev,
			 "mvc: decode stall (D1k, not aborting): pic_count=%u status=%u esparser=%d\n",
			 pic_count, status,
			 atomic_read(&sess->esparser_queued_bufs));
		mod_timer(&mvc->check_timer, jiffies + CHECK_INTERVAL);
		return;
	}

	mvc->last_pic_count = pic_count;
	mod_timer(&mvc->check_timer, jiffies + CHECK_INTERVAL);
}

/* ---- View 0 frame output ------------------------------------------------- */

/*
 * mvc_pair_lookup - find or create a POC pairing slot.
 *
 * Returns the slot index, or -1 if the pool is full.
 */
static int mvc_pair_lookup(struct codec_h264_mvc *mvc, int display_poc)
{
	int i, free = -1;

	for (i = 0; i < PAIR_POOL_SIZE; i++) {
		if (mvc->pairs[i].used &&
		    mvc->pairs[i].display_poc == display_poc)
			return i;
		if (!mvc->pairs[i].used && free < 0)
			free = i;
	}

	if (free >= 0) {
		mvc->pairs[free].display_poc = display_poc;
		mvc->pairs[free].view0_buf = -1;
		mvc->pairs[free].view1_buf = -1;
		mvc->pairs[free].used = true;
	}

	return free;
}

/*
 * mvc_copy_frame - copy one decoded view from internal DMA buffer to V4L2 buffer.
 *
 * The firmware writes YUV420 3-plane (Y, U, V separately) into the internal
 * DMA buffer.  V4L2 capture format is NV12/NV12M.  Copy Y plane, then
 * interleave U and V into the NV12 UV plane.
 */
static void mvc_copy_frame(struct amvdec_session *sess,
			   struct codec_h264_mvc *mvc,
			   struct vb2_v4l2_buffer *vbuf,
			   struct mvc_buf *src)
{
	u32 y_size  = mvc->mb_width * 16 * mvc->mb_height * 16;
	u32 uv_size = y_size / 4;
	void *dst_y, *dst_uv;

	if (sess->pixfmt_cap == V4L2_PIX_FMT_NV12M ||
	    sess->pixfmt_cap == V4L2_PIX_FMT_NV12) {
		dst_y = vb2_plane_vaddr(&vbuf->vb2_buf, 0);
		if (sess->pixfmt_cap == V4L2_PIX_FMT_NV12M)
			dst_uv = vb2_plane_vaddr(&vbuf->vb2_buf, 1);
		else
			dst_uv = dst_y + y_size;

		if (dst_y && dst_uv) {
			const u8 *src_y = (const u8 *)src->vaddr;
			const u8 *src_u = src_y + y_size;
			const u8 *src_v = src_u + uv_size;
			u8 *uv = (u8 *)dst_uv;
			u32 j;

			memcpy(dst_y, src_y, y_size);
			for (j = 0; j < uv_size; j++) {
				*uv++ = src_u[j];
				*uv++ = src_v[j];
			}
		}
	}
}

/*
 * mvc_composite_frame_packed - composite view 0 and view 1 into one NV12 buffer.
 *
 * Layout:
 *   Y plane:  [view0 Y: W*H] [gap: W*45 of 0x10] [view1 Y: W*H]
 *   UV plane: [view0 UV: W*H/2 interleaved] [gap: W*22 of 0x80] [view1 UV: W*H/2 interleaved]
 *
 * W = mvc->mb_width * 16, H = mvc->mb_height * 16.
 * Internal buffers store Y + U + V separately (planar YUV420).
 * U size = W/2 * H/2 = W*H/4.
 *
 * For NV12M: dst plane 0 = Y composite, dst plane 1 = UV composite.
 * For NV12:  dst = Y composite followed immediately by UV composite.
 */
static void mvc_composite_frame_packed(struct amvdec_session *sess,
				       struct codec_h264_mvc *mvc,
				       struct vb2_v4l2_buffer *vbuf,
				       struct mvc_buf *src0,
				       struct mvc_buf *src1)
{
	u32 w       = mvc->mb_width  * 16;
	u32 h       = mvc->mb_height * 16;
	u32 y_size  = w * h;
	u32 uv_size = y_size / 4;	/* U size = V size = W/2 * H/2 */
	u32 gap_y   = (u32)w * MVC_FRAME_PACK_GAP;
	u32 gap_uv  = (u32)w * (MVC_FRAME_PACK_GAP / 2);
	void *dst_y, *dst_uv;

	if (sess->pixfmt_cap != V4L2_PIX_FMT_NV12M &&
	    sess->pixfmt_cap != V4L2_PIX_FMT_NV12)
		return;

	dst_y = vb2_plane_vaddr(&vbuf->vb2_buf, 0);
	if (sess->pixfmt_cap == V4L2_PIX_FMT_NV12M)
		dst_uv = vb2_plane_vaddr(&vbuf->vb2_buf, 1);
	else
		dst_uv = dst_y + y_size * 2 + gap_y;

	if (!dst_y || !dst_uv)
		return;

	{
		const u8 *s0_y = (const u8 *)src0->vaddr;
		const u8 *s0_u = s0_y + y_size;
		const u8 *s0_v = s0_u + uv_size;
		const u8 *s1_y = (const u8 *)src1->vaddr;
		const u8 *s1_u = s1_y + y_size;
		const u8 *s1_v = s1_u + uv_size;
		u8 *dy  = (u8 *)dst_y;
		u8 *duv = (u8 *)dst_uv;
		u32 j;

		/* Y: view0 */
		memcpy(dy, s0_y, y_size);
		dy += y_size;
		/* Y: gap */
		memset(dy, 0x10, gap_y);
		dy += gap_y;
		/* Y: view1 */
		memcpy(dy, s1_y, y_size);

		/* UV: view0 interleaved */
		for (j = 0; j < uv_size; j++) {
			*duv++ = s0_u[j];
			*duv++ = s0_v[j];
		}
		/* UV: gap */
		memset(duv, 0x80, gap_uv * 2);
		duv += gap_uv * 2;
		/* UV: view1 interleaved */
		for (j = 0; j < uv_size; j++) {
			*duv++ = s1_u[j];
			*duv++ = s1_v[j];
		}
	}
}

/*
 * mvc_handle_display - process CMD_FRAME_DISPLAY from firmware.
 *
 * Outputs both view 0 (left eye) and view 1 (right eye) as separate V4L2
 * capture buffers with frame-sequential delivery.  Both views of the same
 * POC share the same timestamp.  View 0 is delivered with V4L2_FIELD_NONE,
 * view 1 with V4L2_FIELD_TOP (stereo view indicator for applications).
 *
 * BSP source: vh264_mvc.c:1057-1210 (ISR CMD_FRAME_DISPLAY handler).
 */
static void mvc_handle_display(struct amvdec_session *sess,
			       struct codec_h264_mvc *mvc,
			       u32 data0, u32 data1)
{
	struct amvdec_core *core = sess->core;
	u8 view_id = MBOX_DATA0_VIEW_ID(data0);
	u8 buf_id  = MBOX_DATA0_BUFF_ID(data0);
	int display_poc = (s32)data1;
	int slot;

	if (buf_id >= MVC_MAX_BUFS) {
		dev_err(core->dev,
			"mvc: CMD_FRAME_DISPLAY invalid buf_id=%u view=%u\n",
			buf_id, view_id);
		amvdec_write_dos(core, AV_SCRATCH_0, CMD_DONE);
		return;
	}

	dev_dbg(core->dev,
		"mvc: CMD_FRAME_DISPLAY view=%u buf=%u poc=%d\n",
		view_id, buf_id, display_poc);

	/* Ack the firmware immediately to unblock decode */
	amvdec_write_dos(core, AV_SCRATCH_0, CMD_DONE);

	/* ---- Frame-packed output mode ---- */
	if (mvc_output_mode == 0) {
		/*
		 * Frame-packed: accumulate both views in the pairing slot,
		 * then composite into one NV12 capture buffer.
		 */
		slot = mvc_pair_lookup(mvc, display_poc);
		if (slot < 0) {
			dev_warn(core->dev,
				 "mvc: pair pool full (poc=%d), dropping view=%u buf=%u\n",
				 display_poc, view_id, buf_id);
			/* Recycle immediately */
			if (mvc_wait_recycle_idle(core))
				amvdec_write_dos(core, AV_SCRATCH_7,
						 RECYCLE_ENCODE(view_id, buf_id));
			return;
		}

		if (view_id == 0)
			mvc->pairs[slot].view0_buf = buf_id;
		else
			mvc->pairs[slot].view1_buf = buf_id;

		/*
		 * KI-108 D18: do NOT recycle the internal view buffer here.
		 * The composite below READS both view buffers; recycling now
		 * hands them back to the firmware, which reuses/overwrites them
		 * before mvc_composite_frame_packed() copies the pixels -> the
		 * delivered capture buffer came out empty (only the frame-pack
		 * gap band had data).  Recycle BOTH buffers AFTER the composite.
		 */
		if (mvc->pairs[slot].view0_buf < 0 || mvc->pairs[slot].view1_buf < 0)
			return;	/* partner not yet arrived */

		/* Both views present: composite into one capture buffer */
		{
			struct vb2_v4l2_buffer *vbuf;
			int v0b = mvc->pairs[slot].view0_buf;
			int v1b = mvc->pairs[slot].view1_buf;
			/*
			 * KI-108 D138: per-view split is the default -- view0 from
			 * its own pool (bufs0), view1 from its own pool (bufs1).  The
			 * D134/D135/D136 firmware writes the distinct dependent view
			 * into bufs1 and reports a distinct view_id, so map 0 surfaces
			 * genuine stereo (HW-proven bit-exact vs JMVC SW ref, D138).
			 */
			struct mvc_buf *s0, *s1;

			switch (mvc_src_map) {
			case 1:	/* both-in-v1 (mono duplicate; pre-D138 default) */
				s0 = &mvc->bufs1[v0b]; s1 = &mvc->bufs1[v1b]; break;
			case 2:	/* swap (diagnostic) */
				s0 = &mvc->bufs1[v0b]; s1 = &mvc->bufs0[v1b]; break;
			default: /* per-view split (distinct stereo) [D138 default] */
				s0 = &mvc->bufs0[v0b]; s1 = &mvc->bufs1[v1b]; break;
			}
			dev_dbg(core->dev,
				 "mvc: D27 src_map=%d poc=%d v0b=%d v1b=%d s0_nz=%d s1_nz=%d\n",
				 mvc_src_map, display_poc, v0b, v1b,
				 s0->vaddr && ((u8 *)s0->vaddr)[100*640+200] != 0,
				 s1->vaddr && ((u8 *)s1->vaddr)[100*640+200] != 0);

			/*
			 * KI-108 D138: announce the per-view display index (2d) once
			 * so field logs show which composite mode is active.  With the
			 * D134/D135/D136 firmware second-A=5 pass, map 0 (per-view
			 * split) delivers distinct stereo (HW-proven bit-exact vs the
			 * JMVC SW reference for both views, D138); map 1 is the pre-D138
			 * mono duplicate.
			 */
			if (!mvc->d138_logged) {
				mvc->d138_logged = true;
				dev_warn(core->dev,
					 "mvc: D138 per-view display index -- src_map=%d (%s); view0<-bufs0[%d] view1<-bufs1[%d], distinct=%d\n",
					 mvc_src_map,
					 mvc_src_map == 0 ? "per-view split: distinct stereo" :
					 mvc_src_map == 1 ? "both-in-v1: mono duplicate (pre-D138)" :
					 "swap (diagnostic)",
					 v0b, v1b, s0 != s1);
			}

			dev_dbg(core->dev,
				 "mvc: DELIVERY-DIAG poc=%d dst_ready=%u streamon_cap=%d streamon_out=%d status=%d\n",
				 display_poc,
				 v4l2_m2m_num_dst_bufs_ready(sess->m2m_ctx),
				 sess->streamon_cap, sess->streamon_out,
				 sess->status);
			vbuf = v4l2_m2m_dst_buf_remove(sess->m2m_ctx);
			if (!vbuf) {
				dev_warn(core->dev,
					"mvc: no capture buf for poc=%d (dst_ready=%u)\n",
					display_poc,
					v4l2_m2m_num_dst_bufs_ready(sess->m2m_ctx));
				mvc->pairs[slot].used = false;
				return;
			}

			mvc_composite_frame_packed(sess, mvc, vbuf, s0, s1);

			/* Peek at timestamp before delivery */
			{
				struct amvdec_timestamp *peek;
				unsigned long ts_flags;

				spin_lock_irqsave(&sess->ts_spinlock, ts_flags);
				if (!list_empty(&sess->timestamps)) {
					peek = list_first_entry(&sess->timestamps,
								struct amvdec_timestamp,
								list);
					mvc->last_ts = peek->ts;
					mvc->last_tc = peek->tc;
					mvc->last_flags = peek->flags;
				}
				spin_unlock_irqrestore(&sess->ts_spinlock, ts_flags);
			}

			amvdec_dst_buf_done(sess, vbuf, V4L2_FIELD_NONE);

			/*
			 * KI-108 P1 D159: record this POC as delivered so the
			 * EOS drain-flush (approach A) does not re-announce it.
			 */
			if (mvc->n_delivered_poc <
			    (int)ARRAY_SIZE(mvc->delivered_poc))
				mvc->delivered_poc[mvc->n_delivered_poc++] =
					(s16)display_poc;

			/*
			 * KI-108 D18: now that the composite has copied both
			 * views into the capture buffer, recycle the two
			 * internal view buffers back to the firmware (single-
			 * slot AV_SCRATCH_7 handshake, one at a time).
			 */
			if (mvc_wait_recycle_idle(core))
				amvdec_write_dos(core, AV_SCRATCH_7,
					RECYCLE_ENCODE(0, mvc->pairs[slot].view0_buf));
			if (mvc_wait_recycle_idle(core))
				amvdec_write_dos(core, AV_SCRATCH_7,
					RECYCLE_ENCODE(1, mvc->pairs[slot].view1_buf));

			mvc->pairs[slot].used = false;
		}
		return;
	}
	/* else: frame-sequential (existing code below) */

	slot = mvc_pair_lookup(mvc, display_poc);
	if (slot < 0) {
		dev_warn(core->dev,
			 "mvc: pairing pool full (poc=%d), dropping view=%u buf=%u\n",
			 display_poc, view_id, buf_id);
		/* Recycle the buffer immediately */
		if (mvc_wait_recycle_idle(core))
			amvdec_write_dos(core, AV_SCRATCH_7,
					 RECYCLE_ENCODE(view_id, buf_id));
		return;
	}

	if (view_id == 0) {
		struct vb2_v4l2_buffer *vbuf;
		struct mvc_buf *src = &mvc->bufs0[buf_id];

		mvc->pairs[slot].view0_buf = buf_id;

		if (!src->vaddr) {
			dev_err(core->dev,
				"mvc: view0 buf %u has no DMA buffer\n", buf_id);
			goto recycle_v0;
		}

		vbuf = v4l2_m2m_dst_buf_remove(sess->m2m_ctx);
		if (!vbuf) {
			dev_dbg(core->dev,
				"mvc: no capture buf for view0 poc=%d (pre-resume)\n",
				display_poc);
			goto recycle_v0;
		}

		mvc_copy_frame(sess, mvc, vbuf, src);

		/*
		 * Peek at the timestamp that amvdec_dst_buf_done will consume,
		 * save it for view 1 reuse.  Must be done BEFORE delivery --
		 * after v4l2_m2m_buf_done the buffer is owned by userspace.
		 */
		{
			struct amvdec_timestamp *peek;
			unsigned long ts_flags;

			spin_lock_irqsave(&sess->ts_spinlock, ts_flags);
			if (!list_empty(&sess->timestamps)) {
				peek = list_first_entry(&sess->timestamps,
							struct amvdec_timestamp, list);
				mvc->last_ts = peek->ts;
				mvc->last_tc = peek->tc;
				mvc->last_flags = peek->flags;
			}
			spin_unlock_irqrestore(&sess->ts_spinlock, ts_flags);
		}

		amvdec_dst_buf_done(sess, vbuf, V4L2_FIELD_NONE);

recycle_v0:
		if (!mvc_wait_recycle_idle(core))
			goto recycle_done;
		amvdec_write_dos(core, AV_SCRATCH_7,
				 RECYCLE_ENCODE(0, buf_id));

		if (mvc->pairs[slot].view1_buf >= 0)
			mvc->pairs[slot].used = false;

	} else {
		struct vb2_v4l2_buffer *vbuf;
		struct mvc_buf *src = &mvc->bufs1[buf_id];

		mvc->pairs[slot].view1_buf = buf_id;

		/*
		 * View 1 (right eye): output as a separate capture buffer.
		 * Use V4L2_FIELD_TOP as stereo view indicator so applications
		 * can distinguish view 0 (FIELD_NONE) from view 1 (FIELD_TOP).
		 *
		 * View 1 reuses the timestamp from view 0 (same instant) via
		 * amvdec_dst_buf_done_ts() which doesn't consume from the
		 * timestamp list or decrement esparser_queued_bufs.
		 */
		if (src->vaddr) {
			vbuf = v4l2_m2m_dst_buf_remove(sess->m2m_ctx);
			if (vbuf) {
				mvc_copy_frame(sess, mvc, vbuf, src);
				amvdec_dst_buf_done_ts(sess, vbuf,
						       V4L2_FIELD_TOP,
						       mvc->last_ts,
						       mvc->last_tc,
						       mvc->last_flags);
			}
		}

		if (!mvc_wait_recycle_idle(core))
			goto recycle_done;
		amvdec_write_dos(core, AV_SCRATCH_7,
				 RECYCLE_ENCODE(1, buf_id));

		if (mvc->pairs[slot].view0_buf >= 0)
			mvc->pairs[slot].used = false;
	}

recycle_done:
	return;
}

/* ---- Threaded ISR -- buffer allocation ----------------------------------- */

/*
 * mvc_do_alloc - allocate buffers for one view (called in threaded ISR).
 *
 * Reads stream parameters from MAILBOX_DATA_0, allocates reference buffer
 * and display buffer pool, configures canvases, writes response to firmware,
 * and acks the command.
 *
 * BSP source: vh264_mvc.c:859-1024 (do_alloc_work).
 */
static void mvc_do_alloc(struct amvdec_session *sess,
			  struct codec_h264_mvc *mvc, int view_id)
{
	struct amvdec_core *core = sess->core;
	u32 data0 = amvdec_read_dos(core, AV_SCRATCH_1);
	int level_idc = MBOX_DATA0_LEVEL_IDC(data0);
	int max_ref   = MBOX_DATA0_MAX_REF(data0);
	int mb_w      = MBOX_DATA0_MB_WIDTH(data0);
	int mb_h      = MBOX_DATA0_MB_HEIGHT(data0);
	int dpb, total_bufs;
	u32 ref_buf_size, ref_size;
	int canvas_start, anc_base;
	struct mvc_buf *bufs;
	int *num_bufs;
	dma_addr_t *ref_paddr;
	void **ref_vaddr;
	size_t *ref_sz;
	int ret;

	/* Align mb dimensions to 4 macroblocks */
	mb_w = ALIGN(mb_w, 4);
	mb_h = ALIGN(mb_h, 4);

	dpb = mvc_max_dpb_size(level_idc, max_ref, mb_w, mb_h);
	total_bufs = dpb + DYNAMIC_BUF_MARGIN;
	if (total_bufs > MVC_MAX_BUFS)
		total_bufs = MVC_MAX_BUFS;

	/* Per-MV ref buffer: mb_total * 96 bytes per frame, *2 for field/frame */
	ref_size     = mb_w * mb_h * 96;
	ref_buf_size = ref_size * (max_ref + 1) * 2;

	dev_dbg(core->dev,
		"mvc: alloc view%d: mb=%dx%d level=%d max_ref=%d dpb=%d total=%d ref_buf=%u\n",
		view_id, mb_w, mb_h, level_idc, max_ref, dpb, total_bufs, ref_buf_size);

	if (view_id == 0) {
		/* Store geometry (view 0 is authoritative) */
		mvc->mb_width  = mb_w;
		mvc->mb_height = mb_h;
		mvc->max_ref   = max_ref;
		mvc->dpb0      = dpb;

		canvas_start = 0;	/* view 0 canvases start at index 0 */
		anc_base     = ANC0_CANVAS_ADDR;
		bufs         = mvc->bufs0;
		num_bufs     = &mvc->num_bufs0;
		ref_paddr    = &mvc->ref_paddr[0];
		ref_vaddr    = &mvc->ref_vaddr[0];
		ref_sz       = &mvc->ref_size[0];
	} else {
		/*
		 * View 1 DPB must match view 0 (BSP enforces this).
		 * BSP source: vh264_mvc.c:955-961.
		 */
		if (dpb != mvc->dpb0) {
			dev_warn(core->dev,
				 "mvc: view0/1 dpb mismatch (%d/%d), using view0\n",
				 mvc->dpb0, dpb);
			dpb = mvc->dpb0;
			total_bufs = dpb + DYNAMIC_BUF_MARGIN;
			if (total_bufs > MVC_MAX_BUFS)
				total_bufs = MVC_MAX_BUFS;
		}
		mvc->dpb1 = dpb;

		/* View 1 canvases follow view 0: start at num_bufs0 * 3 */
		canvas_start = mvc->num_bufs0 * 3;
		anc_base     = ANC0_CANVAS_ADDR + mvc->num_bufs0 * 4;
		bufs         = mvc->bufs1;
		num_bufs     = &mvc->num_bufs1;
		ref_paddr    = &mvc->ref_paddr[1];
		ref_vaddr    = &mvc->ref_vaddr[1];
		ref_sz       = &mvc->ref_size[1];
	}

	/* Allocate reference (motion vector) buffer */
	*ref_sz = PAGE_ALIGN(ref_buf_size);
	*ref_vaddr = dma_alloc_coherent(core->dev, *ref_sz,
					ref_paddr, GFP_KERNEL);
	if (!*ref_vaddr) {
		dev_err(core->dev,
			"mvc: view%d ref buf alloc failed (%zu bytes)\n",
			view_id, *ref_sz);
		amvdec_abort(sess);
		return;
	}

	/* Allocate display buffer pool and configure canvases */
	ret = mvc_alloc_view_buffers(core, bufs, total_bufs,
				     canvas_start, anc_base, mb_w, mb_h);
	if (ret < 0) {
		dev_err(core->dev,
			"mvc: view%d display buf alloc failed: %d\n",
			view_id, ret);
		dma_free_coherent(core->dev, *ref_sz, *ref_vaddr, *ref_paddr);
		*ref_vaddr = NULL;
		amvdec_abort(sess);
		return;
	}
	*num_bufs = total_bufs;

	/* Write reference buffer address to firmware */
	if (view_id == 0) {
		amvdec_write_dos(core, AV_SCRATCH_M, (u32)*ref_paddr);

		/*
		 * KI-108 D19 (canvas-scheme fix): the MVC firmware computes its
		 * per-frame OUTPUT canvas as CANVAS_START + buf*3 (+1/+2 for U/V),
		 * NOT from the ANC0_CANVAS_ADDR ID list (that drives reference
		 * management only).  We allocate the view canvases dynamically via
		 * meson_canvas (contiguous IDs, e.g. 0x0b,0x0c,0x0d,0x0e,...), so
		 * CANVAS_START must be the FIRST allocated canvas ID -- not 0.
		 * With AV_SCRATCH_6=0 the firmware wrote decoded pixels to canvas
		 * 0/1/2 (stale DRAM outside our pool) and the composite read empty
		 * view buffers.  View-1 canvases are contiguous after view-0, so
		 * the firmware's CANVAS_START + num_view0*3 + buf*3 lands correctly
		 * too.  Contiguity is verified in mvc_alloc_view_buffers().
		 */
		amvdec_write_dos(core, AV_SCRATCH_6, mvc->bufs0[0].canvas_y);
		dev_warn(core->dev,
			 "mvc: D19 CANVAS_START set to view-0 base canvas 0x%02x\n",
			 mvc->bufs0[0].canvas_y);

		/*
		 * Notify V4L2 framework of stream dimensions.
		 *
		 * Frame-packed mode: capture height = H*2+45 blank lines;
		 *   one capture buffer per stereo pair (not two).
		 * Frame-sequential mode: capture height = H;
		 *   two capture buffers per decoded picture (view 0 + view 1).
		 */
		if (mvc_output_mode == 0) {
			amvdec_src_change(sess, mb_w * 16,
					  mb_h * 16 * 2 + MVC_FRAME_PACK_GAP,
					  total_bufs);
		} else {
			amvdec_src_change(sess, mb_w * 16, mb_h * 16,
					  total_bufs * 2);
		}
		mvc->views_allocated = 1;
	} else {
		amvdec_write_dos(core, AV_SCRATCH_N, (u32)*ref_paddr);
		mvc->views_allocated = 2;

		/*
		 * The firmware enters STATUS=7 after CMD_ALLOC_VIEW_1 and spins
		 * at 0x00f4 waiting for a recycle token (a free buffer) before it
		 * loads slice ucode and decodes. The DPB bootstrap that hands the
		 * firmware its whole free-buffer pool is done ONCE in check_timer
		 * (D1q, guarded by mvc->dpb_primed) -- the recycle handshake needs
		 * the firmware running, which only happens after the CMD_DONE write
		 * below resumes it from the CMD_ALLOC_VIEW_1 wait.
		 */
		mvc->initialized = true;
	}

	/*
	 * Response to firmware:
	 *   MAILBOX_DATA_0: (max_dec_frame_buffering << 8) | total_bufs
	 *   MAILBOX_DATA_1: ref_size (per-frame MV size)
	 *   MAILBOX_CMD: CMD_DONE
	 * BSP source: vh264_mvc.c:918-923, 999-1004.
	 */
	amvdec_write_dos(core, AV_SCRATCH_1, (dpb << 8) | total_bufs);
	amvdec_write_dos(core, AV_SCRATCH_2, ref_size);

	/*
	 * KI-108 D17: view-1 alloc is the last firmware handshake before it
	 * decodes + emits CMD_FRAME_DISPLAY.  MVC powers on at OUTPUT STREAMON
	 * (DYN_RESOLUTION) and races ahead of userspace, so if we ACK here the
	 * firmware decodes and displays frame 0 BEFORE CAPTURE STREAMON --
	 * v4l2_m2m_dst_buf_remove() then returns NULL (dst_ready=0, streamon_cap=0)
	 * and every early frame is dropped.  Hold the view-1 CMD_DONE until the
	 * CAPTURE queue is streaming; the check_timer releases it once
	 * streamon_cap is set (race-free single point).  view-0 ACKs normally
	 * (firmware only requests view-1 next, no decode yet).
	 */
	if (view_id == 1 && !sess->streamon_cap) {
		mvc->alloc1_deferred = true;
		dev_warn(core->dev,
			 "mvc: D17 holding view-1 CMD_DONE until CAPTURE STREAMON\n");
	} else {
		amvdec_write_dos(core, AV_SCRATCH_0, CMD_DONE);
	}

	/*
	 * KI-108 F3 (root-cause fix): NO recycle-token priming here. The BSP
	 * (vh264_mvc.c do_alloc_work :918-923 / :999-1004) ends the per-view alloc
	 * handshake at CMD_DONE (above) and hands the firmware ZERO recycle tokens
	 * before frame 0 -- the firmware decodes into its freshly-alloc'd DPB with
	 * no AV_SCRATCH_7 priming. The earlier D1q/D1v/D2d code flooded AV_SCRATCH_7
	 * with the whole pool right after alloc, on the "buffer-supply is the gate"
	 * premise that D1w already FALSIFIED. That fabricated step (absent from the
	 * BSP, which runs the same firmware fine) desynced the firmware's
	 * buffer-ownership / free-list state before it could advance MB 0 -- the
	 * KI-108 stall. AV_SCRATCH_7 is now written ONLY by mvc_handle_display(),
	 * one token per buffer the consumer actually returns, mirroring the BSP
	 * put-timer recycle (vh264_mvc.c:1252-1283).
	 */

	dev_dbg(core->dev,
		"mvc: view%d alloc done: %d bufs, ref=%zu bytes at %pad\n",
		view_id, total_bufs, *ref_sz, ref_paddr);
}

/* ---- codec_ops implementations ------------------------------------------- */

/*
 * load_extended_firmware - allocate and populate swap buffer from firmware.
 *
 * Called by vdec_1_start() with fw->data + MC_SIZE (= firmware bytes after 16KB).
 * The MVC firmware is 28KB total:
 *   - First 16KB: loaded to IMEM by vdec_1_start() DMA
 *   - Extended 12KB: passed here, used to populate the swap buffer
 *
 * The swap buffer (MC_TOTAL_SIZE = 28KB at AV_SCRATCH_K) must contain
 * specific firmware segment copies for runtime ucode swap:
 *   [0x0000..0x0fff]: header ucode   (from fw[0x1000..0x1fff])
 *   [0x1000..0x2fff]: MMCO ucode     (from fw[0x2000..0x3fff])
 *   [0x3000..0x5fff]: slice ucode    (from fw[0x4000..0x6fff])
 *
 * Since we only receive fw+0x4000 here (12KB), we also need fw[0x1000..0x3fff]
 * from the full 28KB binary.  vdec_1.c passes the raw fw->data bytes starting
 * at MC_SIZE offset, so we cannot reach back into the first 16KB.
 *
 * Solution: store the struct here, workspace alloc in start().  The swap buffer
 * segments that lie within [0..MC_SIZE) must be sourced from the extended data
 * pointer -- but since we cannot reach back, we call load_extended_firmware
 * with the FULL firmware to handle the BSP segment layout correctly.
 *
 * NOTE: vdec_1.c calls load_extended_firmware with (fw->data + MC_SIZE, fw->size - MC_SIZE).
 * We allocate the struct here (matching codec_h264.c pattern) but defer the
 * swap buffer segment copy to start() where we can re-request the firmware.
 * For now: allocate the full swap buffer and copy the extended (last 12KB) part.
 * The first two segments (header and MMCO) overlap with the first 16KB of IMEM
 * so the firmware can also load them from IMEM directly -- BSP copies them to
 * mc_cpu_addr as an optimization for fast swap-in.
 *
 * BSP source: vh264_mvc.c:1609-1647.
 */
static int codec_h264_mvc_load_extended_firmware(struct amvdec_session *sess,
						 const u8 *data, u32 len,
						 u32 mc_size)
{
	struct codec_h264_mvc *mvc;
	struct amvdec_core *core = sess->core;

	/*
	 * data points to the FULL firmware binary (from fw->data[0]).
	 * len = fw->size (total firmware size = 28KB = MC_TOTAL_SIZE).
	 * mc_size = MC_SIZE = 16KB (first portion already loaded to IMEM).
	 *
	 * Swap buffer layout (BSP vh264_mvc.c:1609-1647):
	 *   mc_cpu_addr[0x0000..0x0fff] <- fw[0x1000..0x1fff] header   (4KB)
	 *   mc_cpu_addr[0x1000..0x2fff] <- fw[0x2000..0x3fff] mmco     (8KB)
	 *   mc_cpu_addr[0x3000..0x5fff] <- fw[0x4000..0x6fff] slice    (12KB)
	 *
	 * All 3 segments must be populated.  If header/MMCO slots are zeros,
	 * the firmware DMA-loads zeros into IMEM and overwrites the dispatch
	 * loop at 0x0000-0x03E9, silently killing decode.
	 */
	if (len < MC_TOTAL_SIZE) {
		dev_err(core->dev,
			"mvc: firmware too small: %u < %u\n",
			len, MC_TOTAL_SIZE);
		return -EINVAL;
	}

	mvc = kzalloc(sizeof(*mvc), GFP_KERNEL);
	if (!mvc)
		return -ENOMEM;

	/*
	 * Allocate swap buffer (MC_TOTAL_SIZE = 28KB).
	 * AV_SCRATCH_K points here; firmware DMA-loads ucode segments from it.
	 * BSP: mc_cpu_addr, MC_TOTAL_SIZE = 28*SZ_1K.
	 * BSP source: vh264_mvc.c:1609-1618.
	 */
	mvc->swap_vaddr = dma_alloc_coherent(core->dev, MC_TOTAL_SIZE,
					     &mvc->swap_paddr, GFP_KERNEL);
	if (!mvc->swap_vaddr) {
		dev_err(core->dev, "mvc: swap buffer alloc failed\n");
		kfree(mvc);
		return -ENOMEM;
	}

	/*
	 * Copy all three ucode segments from the full firmware binary.
	 * BSP source: vh264_mvc.c:1637-1647 (three memcpy blocks).
	 *
	 * Header segment: fw[0x1000..0x1fff] -> swap[0x0000..0x0fff]
	 */
	memcpy((u8 *)mvc->swap_vaddr + MC_SWAP_HEADER_DST,
	       data + MC_SWAP_HEADER_SRC, MC_SWAP_HEADER_LEN);

	/*
	 * MMCO segment: fw[0x2000..0x3fff] -> swap[0x1000..0x2fff]
	 */
	memcpy((u8 *)mvc->swap_vaddr + MC_SWAP_MMCO_DST,
	       data + MC_SWAP_MMCO_SRC, MC_SWAP_MMCO_LEN);

	/*
	 * Slice segment: fw[0x4000..0x6fff] -> swap[0x3000..0x5fff]
	 * mc_size = 0x4000, so data + mc_size = fw[0x4000..].
	 */
	memcpy((u8 *)mvc->swap_vaddr + MC_SWAP_SLICE_DST,
	       data + MC_SWAP_SLICE_SRC, MC_SWAP_SLICE_LEN);

	/*
	 * KI-108 D113 patch 1/2: M4_CONTROL bit13 (VLD h264_en).  Patch the
	 * EXECUTED copy of PC 0x0737 in the swap-buffer HEADER segment (source
	 * .bin stays byte-identical -- Firmware Binary Integrity Rule 0).  Guard
	 * on size (MC_TOTAL_SIZE=28672) AND the exact 4 unpatched bytes
	 * (c8 82 c0 09 = "clr r8,r8,11") at flat offset 0x1cdc, which match
	 * gxl_h264_mvc.bin AND gxm_h264_mvc.bin AND sm1_h264_mvc.bin (byte-identical
	 * decode core at the patch site, see the define block above) -- so this fires
	 * on GXL, G12A/G12B, and SM1 alike, and
	 * skips+warns on a patched/different variant (Rule 1, never fail hard).
	 * On match write 48 83 80 ("set r8,r8,13"; 4th byte 0x09 unchanged) into
	 * swap[0xcdc].
	 */
	if (len == MC_TOTAL_SIZE &&
	    data[MC_KI108_BIT13_FW_OFFSET + 0] == 0xc8 &&
	    data[MC_KI108_BIT13_FW_OFFSET + 1] == 0x82 &&
	    data[MC_KI108_BIT13_FW_OFFSET + 2] == 0xc0 &&
	    data[MC_KI108_BIT13_FW_OFFSET + 3] == 0x09) {
		u8 *swap = (u8 *)mvc->swap_vaddr + MC_KI108_BIT13_SWAP_OFFSET;

		/* clr r8,r8,11 (c8 82 c0 09) -> set r8,r8,13 (48 83 80 09) */
		swap[0] = 0x48;
		swap[1] = 0x83;
		swap[2] = 0x80;
		/* swap[3] stays 0x09 (opcode byte unchanged) */
		dev_warn(core->dev,
			 "KI-108 D113: patched MVC firmware M4_CONTROL bit13 (VLD h264_en) at 0x%x (PC 0x0737, swap[0x%x]) -- gxl/gxm/sm1 CAVLC-MVC decode-enable\n",
			 MC_KI108_BIT13_FW_OFFSET, MC_KI108_BIT13_SWAP_OFFSET);
	} else {
		dev_warn(core->dev,
			 "KI-108 D113: M4_CONTROL bit13 patch SKIPPED (firmware size=%u or bytes at 0x%x not the BSP gxl/gxm/sm1 c8 82 c0 09; running unmodified)\n",
			 len, MC_KI108_BIT13_FW_OFFSET);
	}

	/*
	 * KI-108 D113 patch 2/2: slice skip-gate 0x1062.  Patch the EXECUTED copy
	 * of PC 0x1062 in the swap-buffer SLICE segment (source .bin stays
	 * byte-identical -- Rule 0).  Guard on size AND the exact 4 unpatched
	 * bytes (08 d3 49 06 = "mfcr r8,0x09d3") at flat offset 0x4188, which
	 * match gxl AND gxm AND sm1 (byte-identical decode core at the patch site;
	 * sm1's only nearby divergence is the skip-HANDLER target 0x1332 vs 0x132c,
	 * never taken after this patch).  On match write
	 * 08 00 80 06 ("li r8,0") into swap[0x3188] so the 0x1063 J-check always
	 * takes the DECODE path (per-MB engine runs, MBY_MBX advances); the real
	 * AV_SCRATCH_J register and the [0x84] genuine-skip gate at 0x106d are
	 * untouched, so the 0x05dd proceed-gate still sees J!=0.
	 */
	if (len == MC_TOTAL_SIZE &&
	    data[MC_KI108_SKIPGATE_FW_OFFSET + 0] == 0x08 &&
	    data[MC_KI108_SKIPGATE_FW_OFFSET + 1] == 0xd3 &&
	    data[MC_KI108_SKIPGATE_FW_OFFSET + 2] == 0x49 &&
	    data[MC_KI108_SKIPGATE_FW_OFFSET + 3] == 0x06) {
		u8 *swap = (u8 *)mvc->swap_vaddr + MC_KI108_SKIPGATE_SWAP_OFFSET;

		/* mfcr r8,0x09d3 (08 d3 49 06) -> li r8,0 (08 00 80 06) */
		swap[1] = 0x00;
		swap[2] = 0x80;
		/* swap[0]=0x08 and swap[3]=0x06 unchanged */
		dev_warn(core->dev,
			 "KI-108 D113: patched MVC firmware slice skip-gate 0x1062 (mfcr->li r8,0) at 0x%x (PC 0x1062, swap[0x%x]) -- per-MB engine will run\n",
			 MC_KI108_SKIPGATE_FW_OFFSET, MC_KI108_SKIPGATE_SWAP_OFFSET);
	} else {
		dev_warn(core->dev,
			 "KI-108 D113: slice skip-gate patch SKIPPED (firmware size=%u or bytes at 0x%x not the BSP gxl/gxm/sm1 08 d3 49 06; running unmodified)\n",
			 len, MC_KI108_SKIPGATE_FW_OFFSET);
	}

	/*
	 * KI-108 D134: MVC 2c dispatch trampoline over the dependent-slice PARK at
	 * PC 0x05f3.  Patch the EXECUTED HEADER-segment swap copy (swap[0x7cc]);
	 * source .bin stays byte-identical (Rule 0).  Guard on the exact original 4
	 * bytes (c0 86 7f 0c = "b -485 -> 0x040e", the PARK), identical across
	 * gxl/gxm/sm1.  On match write c0 dd 7e 0c ("b -1161 -> 0x016a") to divert
	 * the dependent-slice park into the guarded resident dispatch block installed
	 * at PC 0x016a by vdec_1_load_firmware (D134).  The resident block seeds
	 * [0xa]=0 for the dependent view (unpark -> A=5 fires -> per-MB recon into
	 * canvas1) and leaves the base view untouched (guarded on [0x56]).
	 */
	if (len == MC_TOTAL_SIZE &&
	    data[MC_KI108_DISPATCH_FW_OFFSET + 0] == 0xc0 &&
	    data[MC_KI108_DISPATCH_FW_OFFSET + 1] == 0x86 &&
	    data[MC_KI108_DISPATCH_FW_OFFSET + 2] == 0x7f &&
	    data[MC_KI108_DISPATCH_FW_OFFSET + 3] == 0x0c) {
		u8 *swap = (u8 *)mvc->swap_vaddr + MC_KI108_DISPATCH_SWAP_OFFSET;

		/* b -485 ->0x040e (c0 86 7f 0c) -> b -1161 ->0x016a (c0 dd 7e 0c) */
		swap[0] = 0xc0;
		swap[1] = 0xdd;
		swap[2] = 0x7e;
		swap[3] = 0x0c;
		dev_warn(core->dev,
			 "KI-108 D134: patched MVC firmware dependent-slice park 0x05f3 -> trampoline to 0x016a dispatch block at 0x%x (PC 0x05f3, swap[0x%x]) -- 2c recon dispatch for view-1\n",
			 MC_KI108_DISPATCH_FW_OFFSET, MC_KI108_DISPATCH_SWAP_OFFSET);
	} else {
		dev_warn(core->dev,
			 "KI-108 D134: dispatch trampoline SKIPPED (firmware size=%u or bytes at 0x%x not the BSP gxl/gxm/sm1 c0 86 7f 0c park; running unmodified)\n",
			 len, MC_KI108_DISPATCH_FW_OFFSET);
	}

	/*
	 * KI-108 D135: MVC 2c dispatch ROUTING trampoline over the dependent-slice
	 * DROP at PC 0x043c.  Patch the EXECUTED HEADER-segment swap copy
	 * (swap[0xf0]); source .bin stays byte-identical (Rule 0).  Guard on the
	 * exact original 4 bytes (c8 bf a1 0a = "beqi r8,255,+6 -> 0x0442", the drop),
	 * identical across gxl/gxm/sm1.  On match write 80 4d 7f 0c ("b -714 ->
	 * 0x0172") to divert the type-20 slice into the guarded resident route block
	 * installed at PC 0x0172 by vdec_1_load_firmware (D135).  The route block sets
	 * [0x56]=1 and routes the slice to 0x0446 -> 0x044d -> 0x0456 -> 0x05d0 recon
	 * (mirroring the base route), then the D134 block unparks the 0x05f3 [0xa]
	 * gate.  Type-14 prefix NALs ([0x48]==14) fall through the route block to the
	 * original drop, so they are unaffected.
	 */
	if (len == MC_TOTAL_SIZE &&
	    data[MC_KI108_ROUTE_FW_OFFSET + 0] == 0xc8 &&
	    data[MC_KI108_ROUTE_FW_OFFSET + 1] == 0xbf &&
	    data[MC_KI108_ROUTE_FW_OFFSET + 2] == 0xa1 &&
	    data[MC_KI108_ROUTE_FW_OFFSET + 3] == 0x0a) {
		u8 *swap = (u8 *)mvc->swap_vaddr + MC_KI108_ROUTE_SWAP_OFFSET;

		/* beqi r8,255,+6 ->0x0442 (c8 bf a1 0a) -> b -714 ->0x0172 (80 4d 7f 0c) */
		swap[0] = 0x80;
		swap[1] = 0x4d;
		swap[2] = 0x7f;
		swap[3] = 0x0c;
		dev_warn(core->dev,
			 "KI-108 D135: patched MVC firmware dependent-slice drop 0x043c -> trampoline to 0x0172 route block at 0x%x (PC 0x043c, swap[0x%x]) -- 2c recon ROUTING for view-1\n",
			 MC_KI108_ROUTE_FW_OFFSET, MC_KI108_ROUTE_SWAP_OFFSET);
	} else {
		dev_warn(core->dev,
			 "KI-108 D135: route trampoline SKIPPED (firmware size=%u or bytes at 0x%x not the BSP gxl/gxm/sm1 c8 bf a1 0a drop; running unmodified)\n",
			 len, MC_KI108_ROUTE_FW_OFFSET);
	}

	/*
	 * KI-108 D136: MVC second-A=5 re-entry trampoline over the A=3 issue at PC
	 * 0x0690.  Patch the EXECUTED HEADER-segment swap copy (swap[0xa40]); source
	 * .bin stays byte-identical (Rule 0).  Guard on the exact original 4 bytes
	 * (03 ca 09 08 = "itcr 0x03,0x09ca", the A=3 issue), byte-identical across
	 * gxl/gxm/sm1.  On match write 80 ba 7e 0c ("b -1302 -> 0x017a") to divert
	 * into the guarded resident re-entry block installed at PC 0x017a by
	 * vdec_1_load_firmware (D136), which forces [0x5b]!=0 for the dependent view
	 * (-> the C-core grid re-arm fw 0x107e runs -> a SECOND per-MB grid into
	 * canvas1), re-issues A=3, and returns to 0x0691.  The base view ([0x56]!=1)
	 * is untouched.
	 */
	if (len == MC_TOTAL_SIZE &&
	    data[MC_KI108_REENTRY_FW_OFFSET + 0] == 0x03 &&
	    data[MC_KI108_REENTRY_FW_OFFSET + 1] == 0xca &&
	    data[MC_KI108_REENTRY_FW_OFFSET + 2] == 0x09 &&
	    data[MC_KI108_REENTRY_FW_OFFSET + 3] == 0x08) {
		u8 *swap = (u8 *)mvc->swap_vaddr + MC_KI108_REENTRY_SWAP_OFFSET;

		/* itcr 0x03,0x09ca (03 ca 09 08) -> b -1302 ->0x017a (80 ba 7e 0c) */
		swap[0] = 0x80;
		swap[1] = 0xba;
		swap[2] = 0x7e;
		swap[3] = 0x0c;
		dev_warn(core->dev,
			 "KI-108 D136: patched MVC firmware A=3 issue 0x0690 -> trampoline to 0x017a re-entry block at 0x%x (PC 0x0690, swap[0x%x]) -- SECOND A=5 per-MB pass for view-1\n",
			 MC_KI108_REENTRY_FW_OFFSET, MC_KI108_REENTRY_SWAP_OFFSET);
	} else {
		dev_warn(core->dev,
			 "KI-108 D136: re-entry trampoline SKIPPED (firmware size=%u or bytes at 0x%x not the BSP gxl/gxm/sm1 03 ca 09 08 A=3 issue; running unmodified)\n",
			 len, MC_KI108_REENTRY_FW_OFFSET);
	}

	sess->priv = mvc;

	dev_info(core->dev,
		 "mvc: swap buffer allocated (%pad): header@%x mmco@%x slice@%x\n",
		 &mvc->swap_paddr,
		 MC_SWAP_HEADER_DST, MC_SWAP_MMCO_DST, MC_SWAP_SLICE_DST);

	return 0;
}

static int codec_h264_mvc_start(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_h264_mvc *mvc = sess->priv;

	/* sess->priv is set by load_extended_firmware -- it must be called first */
	if (!mvc) {
		dev_err(core->dev, "mvc: priv not set (load_extended_firmware missing?)\n");
		return -EINVAL;
	}

	/* Allocate workspace (640 KB, physically contiguous) */
	mvc->workspace_vaddr = dma_alloc_coherent(core->dev, WORKSPACE_SIZE,
						  &mvc->workspace_paddr,
						  GFP_KERNEL);
	if (!mvc->workspace_vaddr) {
		dev_err(core->dev, "mvc: workspace alloc failed\n");
		return -ENOMEM;
	}

	/* Hardware init */
	mvc_h264_decode_init(core, mvc);

	/*
	 * Write swap buffer address BEFORE releasing MCPU.  The firmware reads
	 * AV_SCRATCH_K (UCODE_START_ADDR) during its init path to DMA-load
	 * ucode segments.  If we write it after mvc_prot_init() the MCPU may
	 * already be running and see AV_SCRATCH_K=0, causing a DMA from address
	 * 0 which hangs on G12B (MMU/security).
	 * BSP order: WRITE_VREG(UCODE_START_ADDR, ...) then vh264mvc_prot_init().
	 * BSP source: vh264_mvc.c:1622 vs 1650.
	 */
	amvdec_write_dos(core, AV_SCRATCH_K, mvc->swap_paddr);

	mvc_prot_init(core);

	/*
	 * VC1_CONTROL_REG bit 23: MUST be set AFTER mvc_prot_init() because
	 * the MCPU reset (DOS_SW_RESET0 bit 11) inside prot_init clears it.
	 * GXM firmware sets this from inside the running AMRISC after reset,
	 * but GXL firmware omits it, causing hard lockups on large streams.
	 * Setting it here (post-reset, pre-decode) is equivalent to the GXM
	 * firmware's own write at instruction 0x004b-0x004d.
	 */
	amvdec_write_dos_bits(core, VC1_CONTROL_REG, BIT(23));

	/* Start watchdog */
	mvc->sess = sess;
	timer_setup(&mvc->check_timer, codec_h264_mvc_check_timer, 0);
	mod_timer(&mvc->check_timer, jiffies + CHECK_INTERVAL * 3);

	dev_dbg(core->dev, "mvc: start ok (workspace=%pad, swap=%pad)\n",
		&mvc->workspace_paddr, &mvc->swap_paddr);

	return 0;
}

/*
 * codec_h264_mvc_resume - called on CAPTURE STREAMON after SOURCE_CHANGE.
 *
 * MVC allocates its own internal buffer pools and configures canvases in
 * mvc_do_alloc() (threaded ISR), unlike regular H.264 which defers canvas
 * setup to resume.  The firmware is already running and may have decoded
 * frames by this point.  No additional hardware setup is needed here.
 *
 * This callback exists because amvdec_src_change() in mvc_do_alloc()
 * puts the session into STATUS_NEEDS_RESUME, and vdec_start_streaming()
 * calls codec_ops->resume() unconditionally in that state.
 */
static void codec_h264_mvc_resume(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;

	dev_dbg(core->dev, "mvc: resume (views_allocated=%d)\n",
		((struct codec_h264_mvc *)sess->priv)->views_allocated);
}

/*
 * codec_h264_mvc_drain - drain callback to unblock end-of-stream path.
 *
 * Root cause: after all frames are decoded the MVC firmware enters STATUS=7
 * (recycle wait loop) and spins indefinitely.  vdec_drain_work() calls
 * vdec_wait_inactive() which waits for IRQ silence; since the firmware keeps
 * the AMRISC running, the drain either times out or stalls the session.
 *
 * Fix: detect STATUS=7 with no more input queued and halt the AMRISC by
 * writing MPSR=0.  The firmware is done -- it is just spinning in a software
 * idle loop -- so halting is safe and equivalent to what vdec_1_stop() does
 * during teardown.
 *
 * BSP reference: vh264mvc_put_timer_func() detects STATUS=7 and calls
 *   vh264_mvc_stop_decoding() which writes MPSR=0 to exit the loop.
 */
/*
 * KI-108 P1 D159: read `count` (<=128) 16-bit LMEM words starting at LMEM word
 * offset `off` (4-word aligned) into `dst`, de-swapping the LMEM_DMA 64-bit
 * XOR-6 byte permutation (within each 4-word group the word order is reversed).
 * LMEM->mem: CTRL bit15=start, bit14=0 (read dir), bits11:0 = LMEM word offset.
 * Returns 0 on success (bit15 cleared), -1 on DMA hang.
 */
static int mvc_lmem_read(struct amvdec_core *core, dma_addr_t paddr, u16 *scratch,
			 u16 *dst, u32 off, u32 count)
{
	int spin = 10000, i;

	while (--spin && (amvdec_read_dos(core, LMEM_DMA_CTRL) & BIT(15)))
		udelay(1);
	amvdec_write_dos(core, LMEM_DMA_ADR, (u32)paddr);
	amvdec_write_dos(core, LMEM_DMA_COUNT, count);
	amvdec_write_dos(core, LMEM_DMA_CTRL, 0x8000 | (off & 0xfff));
	spin = 10000;
	while (--spin && (amvdec_read_dos(core, LMEM_DMA_CTRL) & BIT(15)))
		udelay(1);
	if (spin == 0)
		return -1;
	/* de-swap XOR-6: within each 4-word group reverse word order */
	for (i = 0; i < (int)count; i++)
		dst[i] = scratch[(i & ~3) | (3 - (i & 3))];
	return 0;
}

static void codec_h264_mvc_dump_dpb(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	dma_addr_t paddr;
	u16 *scratch, *low, *v0, *v1, *ol;
	int i;

	scratch = dma_alloc_coherent(core->dev, 512, &paddr, GFP_KERNEL);
	if (!scratch)
		return;
	low = kzalloc(128 * sizeof(u16), GFP_KERNEL);
	v0  = kzalloc(128 * sizeof(u16), GFP_KERNEL);
	v1  = kzalloc(128 * sizeof(u16), GFP_KERNEL);
	ol  = kzalloc(128 * sizeof(u16), GFP_KERNEL);
	if (!low || !v0 || !v1 || !ol)
		goto out;

	if (mvc_lmem_read(core, paddr, scratch, low, 0x000, 128) ||
	    mvc_lmem_read(core, paddr, scratch, v0,  0x100, 128) ||
	    mvc_lmem_read(core, paddr, scratch, v1,  0x180, 128) ||
	    mvc_lmem_read(core, paddr, scratch, ol,  0x200, 128)) {
		dev_warn(core->dev, "KI-108 P1 D159 DPB-dump: LMEM_DMA hang, aborted\n");
		goto out;
	}

	dev_warn(core->dev,
		 "KI-108 P1 D159 DPB-dump control: rp[0x1e]=%u wp[0x1f]=%u dpbcnt[0x6d]=%u olist_end[0x6f]=0x%x lstart[0x7]=%u [0x2a]=%u [0x47]=%u [0x56]=%u\n",
		 low[0x1e], low[0x1f], low[0x6d], low[0x6f], low[0x7],
		 low[0x2a], low[0x47], low[0x56]);
	/*
	 * Per-view DPB entry flag words: entry i flag at table[(i<<3)+1].
	 * base 256 -> v0[], base 384 -> v1 window starts at 0x180-0x180=0.
	 * Print flag word + POC word (flag+? maskl 16,16) for entries 0..15.
	 */
	for (i = 0; i < 16; i++) {
		u16 f0 = v0[(i << 3) + 1];   /* view0 entry i, offset 0x100 */
		u16 f1 = v1[(i << 3) + 1];   /* view1 entry i, offset 0x180 */

		if (f0 || f1)
			dev_warn(core->dev,
				 "KI-108 P1 D159 DPB[%2d]: v0_flag=0x%04x(need_out=%d) v1_flag=0x%04x(need_out=%d)\n",
				 i, f0, f0 & 1, f1, f1 & 1);
		/*
		 * KI-108 P1 D159 (approach A): raw 8-word entries so the exact
		 * POC word offset/encoding can be validated for mvc_poc_off.
		 */
		if (mvc_flush_dump && (f0 || f1)) {
			int b = i << 3;

			dev_warn(core->dev,
				 "KI-108 P1 D159 RAW v0[%2d]: %04x %04x %04x %04x %04x %04x %04x %04x | v1: %04x %04x %04x %04x %04x %04x %04x %04x\n",
				 i,
				 v0[b], v0[b+1], v0[b+2], v0[b+3],
				 v0[b+4], v0[b+5], v0[b+6], v0[b+7],
				 v1[b], v1[b+1], v1[b+2], v1[b+3],
				 v1[b+4], v1[b+5], v1[b+6], v1[b+7]);
		}
	}
	/* output-list contents at 512(=ol[0..]) / 576(=ol[0x40..]) */
	dev_warn(core->dev,
		 "KI-108 P1 D159 olist@512: %u %u %u %u %u %u %u %u | @576: %u %u %u %u %u %u %u %u\n",
		 ol[0], ol[1], ol[2], ol[3], ol[4], ol[5], ol[6], ol[7],
		 ol[0x40], ol[0x41], ol[0x42], ol[0x43], ol[0x44], ol[0x45], ol[0x46], ol[0x47]);
out:
	kfree(low); kfree(v0); kfree(v1); kfree(ol);
	dma_free_coherent(core->dev, 512, scratch, paddr);
}

/*
 * KI-108 P1 D159 (approach A): driver-side end-of-stream DPB flush.
 *
 * At drain the firmware is idle and every access unit is already decoded into
 * the per-view DMA capture pools (bufs0[]/bufs1[]).  Read the LMEM DPB
 * descriptor tables, recover each occupied slot's POC, and deliver the
 * not-yet-delivered slots in POC order via the existing composite path.  NO
 * firmware byte is touched and NO recycle handshake is issued (the firmware may
 * be halted), so this cannot corrupt the normal-decode recycle (the case-7
 * firmware-flush failure mode is structurally avoided).
 */
static void codec_h264_mvc_drain_flush(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_h264_mvc *mvc = sess->priv;
	dma_addr_t paddr;
	u16 *scratch, *v0, *v1;
	int nbufs = mvc->num_bufs0;
	int order[MVC_MAX_BUFS];
	s16 poc[MVC_MAX_BUFS];
	u16 f0v[MVC_MAX_BUFS];
	int nslot = 0, i, j, flushed = 0;

	if (nbufs <= 0 || nbufs > MVC_MAX_BUFS)
		return;
	if (mvc_poc_off < 0 || mvc_poc_off > 7)
		return;

	/*
	 * KI-108 P1 D162 wedge-state guard.  The MVCDS1 large-slice VLD-wedge
	 * path delivers 0 frames and leaves the VLD scan engine wedged
	 * (VLD_DECODE_STATUS ~ 0x502b); running the drain-time LMEM_DMA read
	 * below against the wedged HW hard-hangs and RESETS the board (D161).
	 * A decode that delivered 0 frames normally made no real progress, so
	 * skip the flush entirely -- with ZERO MMIO on this path, so the guard
	 * itself cannot trip the wedge.  The four other zero-frame paths
	 * (transform-4x4 / CABAC-MVC) delivered 0 too and their flush found no
	 * new frames anyway, so skipping them is behaviorally identical.
	 */
	if (mvc_flush_guard && mvc->n_delivered_poc == 0) {
		dev_warn(core->dev,
			 "mvc: D162: drain-flush skipped -- decode delivered 0 frames (wedged/empty)\n");
		return;
	}
	/*
	 * Belt-and-suspenders: a decode that DID deliver frames but then wedged
	 * the VLD mid-stream is also unsafe to LMEM-read.  Only reached once the
	 * decode is known to have progressed, so this single-shot read is safe
	 * (the wedge window is poll-fatal, not single-read-fatal).
	 */
	if (mvc_flush_guard) {
		u32 vld = amvdec_read_dos(core, VLD_DECODE_STATUS);

		if ((vld & 0xff) == 0x2b) {
			dev_warn(core->dev,
				 "mvc: D162: drain-flush skipped -- VLD wedged 0x%04x\n",
				 vld);
			return;
		}
	}

	scratch = dma_alloc_coherent(core->dev, 512, &paddr, GFP_KERNEL);
	if (!scratch)
		return;
	v0 = kzalloc(128 * sizeof(u16), GFP_KERNEL);
	v1 = kzalloc(128 * sizeof(u16), GFP_KERNEL);
	if (!v0 || !v1)
		goto out;

	dev_warn(core->dev,
		 "mvc: D162 pre-LMEM drain-read (n_delivered=%d)\n",
		 mvc->n_delivered_poc);
	if (mvc_lmem_read(core, paddr, scratch, v0, 0x100, 128) ||
	    mvc_lmem_read(core, paddr, scratch, v1, 0x180, 128)) {
		dev_warn(core->dev, "mvc: D159 flush: LMEM_DMA hang, aborted\n");
		goto out;
	}
	dev_warn(core->dev, "mvc: D162 post-LMEM drain-read OK\n");

	/* Collect occupied slots (nonzero flag word) and their POC key. */
	for (i = 0; i < nbufs; i++) {
		int b = i << 3;
		u16 f0 = v0[b + 1];

		if (!f0)
			continue;
		f0v[nslot] = f0;
		poc[nslot] = (s16)v0[b + mvc_poc_off];
		order[nslot] = i;
		nslot++;
	}

	/* Insertion sort by POC ascending (nslot <= 16). */
	for (i = 1; i < nslot; i++) {
		int oi = order[i];
		s16 pi = poc[i];
		u16 fi = f0v[i];

		for (j = i - 1; j >= 0 && poc[j] > pi; j--) {
			poc[j + 1] = poc[j];
			order[j + 1] = order[j];
			f0v[j + 1] = f0v[j];
		}
		poc[j + 1] = pi;
		order[j + 1] = oi;
		f0v[j + 1] = fi;
	}

	for (i = 0; i < nslot; i++) {
		int slot = order[i];
		struct vb2_v4l2_buffer *vbuf;
		struct mvc_buf *s0, *s1;
		bool dup = false;

		/* Skip POCs already announced by the normal path. */
		for (j = 0; j < mvc->n_delivered_poc; j++)
			if (mvc->delivered_poc[j] == poc[i]) {
				dup = true;
				break;
			}
		if (dup)
			continue;
		/* De-dup within the flush (a stale + fresh copy of one POC). */
		for (j = 0; j < i; j++)
			if (poc[j] == poc[i]) {
				dup = true;
				break;
			}
		if (dup)
			continue;

		switch (mvc_src_map) {
		case 1:
			s0 = &mvc->bufs1[slot]; s1 = &mvc->bufs1[slot]; break;
		case 2:
			s0 = &mvc->bufs1[slot]; s1 = &mvc->bufs0[slot]; break;
		default:
			s0 = &mvc->bufs0[slot]; s1 = &mvc->bufs1[slot]; break;
		}
		if (!s0->vaddr || !s1->vaddr)
			continue;

		vbuf = v4l2_m2m_dst_buf_remove(sess->m2m_ctx);
		if (!vbuf) {
			dev_warn(core->dev,
				 "mvc: D159 flush: out of capture bufs after %d frame(s) (poc=%d slot=%d)\n",
				 flushed, poc[i], slot);
			break;
		}

		mvc_composite_frame_packed(sess, mvc, vbuf, s0, s1);
		amvdec_dst_buf_done_ts(sess, vbuf, V4L2_FIELD_NONE,
				       mvc->last_ts, mvc->last_tc,
				       mvc->last_flags);
		if (mvc->n_delivered_poc < (int)ARRAY_SIZE(mvc->delivered_poc))
			mvc->delivered_poc[mvc->n_delivered_poc++] = poc[i];
		flushed++;
	}

	dev_warn(core->dev,
		 "mvc: D159 drain-flush (approach A): %d occupied slot(s), %d already-delivered, %d NEW frame(s) delivered (poc_off=%d)\n",
		 nslot, mvc->n_delivered_poc - flushed, flushed, mvc_poc_off);
out:
	kfree(v0);
	kfree(v1);
	dma_free_coherent(core->dev, 512, scratch, paddr);
}

static void codec_h264_mvc_drain(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_h264_mvc *mvc = sess->priv;
	u32 status = amvdec_read_dos(core, AV_SCRATCH_A);

	if (mvc_dump_dpb)
		codec_h264_mvc_dump_dpb(sess);

	if (mvc_drain_flush) {
		/*
		 * KI-108 P1 D162 drain discriminator (single-shot, SAFE): the
		 * wedge state the flush-guard keys on.  Healthy jmvc_cavlc drains
		 * with the VLD idle (0), firmware STATUS clean, and n_delivered>0;
		 * the MVCDS1 wedge drains with VLD_DECODE_STATUS ~ 0x502b and
		 * n_delivered==0.  One MMIO read only -- never a poll of the wedge
		 * window.  Gated behind mvc_drain_flush so the default-OFF
		 * production path is untouched.  Runs BEFORE the guard so the
		 * discriminator is captured even when the guard then skips.
		 */
		dev_warn(core->dev,
			 "mvc: D162 drain-entry: VLD_DECODE_STATUS=0x%04x fw_STATUS=%u n_delivered=%d guard=%d\n",
			 amvdec_read_dos(core, VLD_DECODE_STATUS), status,
			 mvc ? mvc->n_delivered_poc : -1, mvc_flush_guard);
		codec_h264_mvc_drain_flush(sess);
	}

	if (status == 7 && atomic_read(&sess->esparser_queued_bufs) <= 0) {
		dev_dbg(core->dev,
			"mvc: drain: firmware idle at STATUS=7, halting AMRISC\n");
		amvdec_write_dos(core, MPSR, 0);
		/*
		 * Clear any pending recycle token so the ISR's recycle-wait
		 * loop does not spin forever on the halted firmware.
		 */
		amvdec_write_dos(core, AV_SCRATCH_7, RECYCLE_IDLE);
	}
}

static int codec_h264_mvc_stop(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_h264_mvc *mvc = sess->priv;
	int v;

	if (!mvc)
		return 0;

	/* Stop watchdog */
	timer_delete_sync(&mvc->check_timer);

	/* Clear mailbox to prevent spurious commands */
	amvdec_write_dos(core, AV_SCRATCH_0, CMD_DONE);
	amvdec_write_dos(core, ASSIST_MBOX1_MASK, 0);

	/* Free display buffer pools */
	mvc_free_view_buffers(core, mvc->bufs0, mvc->num_bufs0);
	mvc_free_view_buffers(core, mvc->bufs1, mvc->num_bufs1);

	/* Free reference buffers */
	for (v = 0; v < 2; v++) {
		if (mvc->ref_vaddr[v]) {
			dma_free_coherent(core->dev, mvc->ref_size[v],
					  mvc->ref_vaddr[v], mvc->ref_paddr[v]);
			mvc->ref_vaddr[v] = NULL;
		}
	}

	/* Free swap buffer (MC_TOTAL_SIZE = 28KB, allocated in load_extended_firmware) */
	if (mvc->swap_vaddr) {
		dma_free_coherent(core->dev, MC_TOTAL_SIZE,
				  mvc->swap_vaddr, mvc->swap_paddr);
		mvc->swap_vaddr = NULL;
	}

	/* Free workspace */
	if (mvc->workspace_vaddr) {
		dma_free_coherent(core->dev, WORKSPACE_SIZE,
				  mvc->workspace_vaddr, mvc->workspace_paddr);
		mvc->workspace_vaddr = NULL;
	}

	kfree(mvc);
	sess->priv = NULL;

	dev_dbg(core->dev, "mvc: stopped\n");
	return 0;
}

/*
 * Hard ISR: clear mailbox interrupt, save command, wake threaded ISR.
 *
 * We wake the thread for all commands so the hard ISR stays minimal.
 * CMD_FATAL_ERROR is handled in threaded ISR too (amvdec_abort is safe
 * from threaded context).
 *
 * BSP source: vh264_mvc.c:1036 (vh264mvc_isr).
 */
static irqreturn_t codec_h264_mvc_isr(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;

	amvdec_write_dos(core, ASSIST_MBOX1_CLR_REG, 1);
	dev_info(core->dev, "mvc: ISR fired, cmd=0x%x\n",
		 amvdec_read_dos(core, AV_SCRATCH_0));

	return IRQ_WAKE_THREAD;
}

/*
 * Threaded ISR: process mailbox commands from MVC firmware.
 *
 * CMD_ALLOC_VIEW_0/1: allocate DMA buffers and configure canvases.
 * CMD_FRAME_DISPLAY:  deliver view 0 frame to V4L2 capture queue.
 * CMD_FATAL_ERROR:    abort session.
 *
 * BSP source: vh264_mvc.c:1036-1222.
 */
static irqreturn_t codec_h264_mvc_threaded_isr(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_h264_mvc *mvc = sess->priv;
	u32 cmd, data0, data1;

	if (!mvc)
		return IRQ_HANDLED;

	/* KI-108 D3: BSP couples the VIFIFO RP at every ISR entry (vh264mvc_isr
	 * calls mvc_set_rp() before reading MAILBOX_COMMAND) -- mirror it. */
	mvc_set_rp(core);

	cmd   = amvdec_read_dos(core, AV_SCRATCH_0) & 0xff;
	data0 = amvdec_read_dos(core, AV_SCRATCH_1);
	data1 = amvdec_read_dos(core, AV_SCRATCH_2);

	switch (cmd) {
	case CMD_DONE:
		/* No pending command */
		break;

	case CMD_ALLOC_VIEW_0:
		/*
		 * Firmware requests view 0 buffer allocation.
		 * data0: level_idc[31:24] | max_ref[23:16] | mb_w[15:8] | mb_h[7:0]
		 * BSP source: vh264_mvc.c:866-940.
		 */
		dev_dbg(core->dev, "mvc: CMD_ALLOC_VIEW_0 data0=0x%08x\n", data0);
		mvc_do_alloc(sess, mvc, 0);
		break;

	case CMD_ALLOC_VIEW_1:
		/*
		 * Firmware requests view 1 buffer allocation.
		 * BSP source: vh264_mvc.c:941-1024.
		 */
		dev_dbg(core->dev, "mvc: CMD_ALLOC_VIEW_1 data0=0x%08x\n", data0);
		mvc_do_alloc(sess, mvc, 1);
		break;

	case CMD_FRAME_DISPLAY:
		/*
		 * Firmware signals a decoded frame is ready.
		 * data0[7:6]=view_id, data0[5:0]=buf_id, data1=display_POC.
		 * BSP source: vh264_mvc.c:1057-1210.
		 */
		mvc_handle_display(sess, mvc, data0, data1);
		break;

	case CMD_FATAL_ERROR:
		/*
		 * Unrecoverable firmware error.
		 * BSP response: full stop + reinit (error_do_work).
		 * BSP source: vh264_mvc.c:1210-1213.
		 */
		dev_err(core->dev, "mvc: firmware fatal error\n");
		amvdec_abort(sess);
		break;

	default:
		dev_warn(core->dev, "mvc: unknown command 0x%02x (data0=0x%08x)\n",
			 cmd, data0);
		amvdec_write_dos(core, AV_SCRATCH_0, CMD_DONE);
		break;
	}

	return IRQ_HANDLED;
}

/* ---- input pre-processing ------------------------------------------------ */

/*
 * codec_h264_mvc_pre_queue - strip MVC prefix NAL units (nal_type 14) from the
 * input before it reaches the firmware (KI-108 D2c).
 *
 * Root cause chain: after DECODE_SKIP_PICTURE was fixed (D1z) the firmware
 * reaches the slice state on frame 0, but D2b showed it processes the MVC
 * PREFIX NAL (nal_type 14) -- [0x2f]=14 -> the slice ucode returns at
 * 0x1104/0x110c WITHOUT decoding a macroblock (a prefix NAL carries no slice
 * data, only nal_unit_header_mvc_extension for the FOLLOWING base-view slice)
 * -- then parks in the case-7 recycle-wait, never advancing to the base-view
 * coded slice (nal_type 1/5) that holds the macroblocks.
 *
 * Removing the prefix NAL from the elementary stream makes the firmware see the
 * base-view slice directly as a plain H.264 slice (base view == implicit view
 * 0). The dependent view (coded-slice-extension, nal_type 20) is unaffected.
 * Done in-place: output length <= input length, so a forward memmove is safe.
 */
static int codec_h264_mvc_pre_queue(struct amvdec_session *sess,
				    struct vb2_buffer *vb)
{
	struct amvdec_core *core = sess->core;
	u8 *d = vb2_plane_vaddr(vb, 0);
	u32 n = vb2_get_plane_payload(vb, 0);
	u32 r = 0, w = 0;
	int dropped = 0;

	if (!d || n < 5)
		return 0;

	while (r < n) {
		u32 sc_len = 0, nal_hdr, nal_type, seg_start = r, e;

		/* Annex-B start code at r? (00 00 01 or 00 00 00 01) */
		if (r + 3 <= n && !d[r] && !d[r + 1] && d[r + 2] == 1)
			sc_len = 3;
		else if (r + 4 <= n && !d[r] && !d[r + 1] && !d[r + 2] &&
			 d[r + 3] == 1)
			sc_len = 4;

		if (!sc_len) {
			/* bytes before the first start code: pass through */
			if (w != r)
				d[w] = d[r];
			w++;
			r++;
			continue;
		}

		nal_hdr = r + sc_len;
		nal_type = (nal_hdr < n) ? (d[nal_hdr] & 0x1f) : 0;

		/* This NAL ends at the next start code (or buffer end) */
		e = nal_hdr + 1;
		while (e + 3 <= n && !(!d[e] && !d[e + 1] && d[e + 2] == 1))
			e++;
		if (e + 3 > n)
			e = n;

		if (nal_type == 14) {
			r = e;		/* drop [seg_start .. e) */
			dropped++;
			continue;
		}

		if (w != seg_start)
			memmove(d + w, d + seg_start, e - seg_start);
		w += e - seg_start;
		r = e;
	}

	if (dropped) {
		vb2_set_plane_payload(vb, 0, w);
		dev_dbg(core->dev,
			"mvc: stripped %d MVC prefix NAL(s), %u -> %u bytes\n",
			dropped, n, w);
	}

	return 0;
}

/* ---- codec_ops table ----------------------------------------------------- */

struct amvdec_codec_ops codec_h264_mvc_ops = {
	.pre_queue	= codec_h264_mvc_pre_queue,
	.load_extended_firmware = codec_h264_mvc_load_extended_firmware,
	.eos_sequence	= codec_h264_mvc_eos_sequence,
	.start		= codec_h264_mvc_start,
	.stop		= codec_h264_mvc_stop,
	.drain		= codec_h264_mvc_drain,
	.resume		= codec_h264_mvc_resume,
	.isr		= codec_h264_mvc_isr,
	.threaded_isr	= codec_h264_mvc_threaded_isr,
};

/* ---- Module glue --------------------------------------------------------- */

static int __init codec_h264_mvc_mod_init(void)
{
	return amvdec_register_codec(V4L2_PIX_FMT_H264_MVC, &codec_h264_mvc_ops);
}

static void __exit codec_h264_mvc_mod_exit(void)
{
	amvdec_unregister_codec(V4L2_PIX_FMT_H264_MVC, &codec_h264_mvc_ops);
}

module_init(codec_h264_mvc_mod_init);
module_exit(codec_h264_mvc_mod_exit);
MODULE_DESCRIPTION("Amlogic meson-vdec H.264 MVC decoder (frame-packed + frame-sequential)");
MODULE_LICENSE("GPL v2");
