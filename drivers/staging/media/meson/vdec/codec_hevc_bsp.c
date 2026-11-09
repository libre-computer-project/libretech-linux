// SPDX-License-Identifier: GPL-2.0+
/*
 * BSP-protocol HEVC firmware support (Bucket 3 of TASK-004).
 *
 * Code in this file runs ONLY when sess->fw_match->quirks &
 * AMVDEC_FW_QUIRK_HEVC_BSP_PROTO is set, i.e. when a BSP-derived HEVC
 * firmware (36KB g12a/sm1/gxl variant or 40KB swap variant) is loaded.
 * Upstream firmware sessions skip these paths entirely; the dispatch
 * happens at the call sites in codec_hevc.c via quirks bit checks.
 *
 * BUCKET DISCIPLINE: every function in this file asserts BSP_PROTO is
 * active via WARN_ON_ONCE on entry.  Calling a Bucket 3 function from
 * an UPSTREAM_PROTO session is a programming error.
 *
 * Phase A.2 (this file's first phase) covers ONLY the RPM buffer
 * allocation + register programming.  Without this, BSP firmware
 * DMA-writes parsed slice-header parameters to physical address 0,
 * causing an AXI bus fault and immediate hard lockup -- KI-036
 * (re-confirmed Phase A.1, see task004-phase-a1-bsp-fw-test-20260420.md).
 *
 * See the internal HEVC BSP architecture plan.
 */

#include <linux/dma-mapping.h>

#include "codec_hevc_bsp.h"
#include "codec_hevc_fw.h"
#include "hevc_regs.h"
#include "vdec.h"
#include "vdec_1.h"		/* vdec_1_power_get/put */
#include "vdec_helpers.h"


static bool hevc_bsp_proto_active(struct amvdec_session *sess)
{
	if (!sess || !sess->fw_match)
		return false;
	return !!(sess->fw_match->quirks & AMVDEC_FW_QUIRK_HEVC_BSP_PROTO);
}

int codec_hevc_bsp_alloc_rpm(struct amvdec_session *sess,
			     void **rpm_vaddr, dma_addr_t *rpm_paddr)
{
	struct amvdec_core *core = sess->core;
	struct device *dev = core->dev;

	if (WARN_ON_ONCE(!hevc_bsp_proto_active(sess)))
		return -EINVAL;

	*rpm_vaddr = dma_alloc_coherent(dev, HEVC_BSP_RPM_BUF_SIZE,
					rpm_paddr, GFP_KERNEL);
	if (!*rpm_vaddr) {
		dev_err(dev, "BSP HEVC: failed to alloc RPM buffer\n");
		return -ENOMEM;
	}

	dev_dbg(dev, "BSP HEVC: allocated RPM buffer phys=0x%pad size=%u\n",
		rpm_paddr, HEVC_BSP_RPM_BUF_SIZE);
	return 0;
}

void codec_hevc_bsp_free_rpm(struct amvdec_session *sess,
			     void *rpm_vaddr, dma_addr_t rpm_paddr)
{
	struct amvdec_core *core = sess->core;

	if (rpm_vaddr)
		dma_free_coherent(core->dev, HEVC_BSP_RPM_BUF_SIZE,
				  rpm_vaddr, rpm_paddr);
}

void codec_hevc_bsp_program_rpm(struct amvdec_session *sess,
				dma_addr_t rpm_paddr)
{
	struct amvdec_core *core = sess->core;

	if (WARN_ON_ONCE(!hevc_bsp_proto_active(sess)))
		return;

	/* HEVC_RPM_BUFFER = HEVC_ASSIST_SCRATCH_1 (per BSP vh265.c:798).
	 * Write the RPM DMA buffer's physical address before MPSR=1 so
	 * the firmware's first SLICE_SEGMENT_DONE handler has a valid
	 * target for its parsed-parameter DMA write.
	 *
	 * Without this, BSP firmware writes to phys addr 0 -> AXI bus
	 * fault -> hard lockup (KI-036, re-confirmed Phase A.1).
	 */
	amvdec_write_dos(core, HEVC_ASSIST_SCRATCH_1, (u32)rpm_paddr);
	dev_info(core->dev,
		 "BSP HEVC: programmed RPM buffer addr=0x%pad to HEVC_RPM_BUFFER\n",
		 &rpm_paddr);
}

int codec_hevc_bsp_alloc_aux(struct amvdec_session *sess,
			     void **aux_vaddr, dma_addr_t *aux_paddr)
{
	struct amvdec_core *core = sess->core;
	struct device *dev = core->dev;

	if (WARN_ON_ONCE(!hevc_bsp_proto_active(sess)))
		return -EINVAL;

	*aux_vaddr = dma_alloc_coherent(dev, HEVC_BSP_AUX_BUF_SIZE,
					aux_paddr, GFP_KERNEL);
	if (!*aux_vaddr) {
		dev_err(dev, "BSP HEVC: failed to alloc AUX buffer\n");
		return -ENOMEM;
	}

	dev_dbg(dev, "BSP HEVC: allocated AUX buffer phys=0x%pad size=%u\n",
		aux_paddr, HEVC_BSP_AUX_BUF_SIZE);
	return 0;
}

void codec_hevc_bsp_free_aux(struct amvdec_session *sess,
			     void *aux_vaddr, dma_addr_t aux_paddr)
{
	struct amvdec_core *core = sess->core;

	if (aux_vaddr)
		dma_free_coherent(core->dev, HEVC_BSP_AUX_BUF_SIZE,
				  aux_vaddr, aux_paddr);
}

void codec_hevc_bsp_program_aux(struct amvdec_session *sess,
				dma_addr_t aux_paddr)
{
	struct amvdec_core *core = sess->core;
	u32 size_word;

	if (WARN_ON_ONCE(!hevc_bsp_proto_active(sess)))
		return;

	/* HEVC_AUX_ADR = HEVC_ASSIST_SCRATCH_L (per BSP vh265.c:5786):
	 * physical address of the prefix+suffix AUX buffer.  BSP firmware
	 * DMA-writes parsed prefix-SEI NAL data here on every access unit.
	 *
	 * HEVC_AUX_DATA_SIZE = HEVC_ASSIST_SCRATCH_M (per BSP vh265.c:5789):
	 * encodes prefix and suffix sizes in 16-byte units.  Layout:
	 *   bits [31:16] = prefix_size >> 4
	 *   bits [15:0]  = suffix_size >> 4
	 * Firmware uses these to bound-check its DMA writes.
	 *
	 * BSP_PROTO-ONLY: SCRATCH_M is HEVC_DECODE_COUNT under upstream
	 * firmware -- programming it here under upstream would corrupt
	 * the upstream decode-count protocol.  WARN_ON_ONCE above
	 * enforces.
	 */
	amvdec_write_dos(core, HEVC_ASSIST_SCRATCH_L, (u32)aux_paddr);

	size_word = ((HEVC_BSP_AUX_PREFIX_SIZE >> 4) << 16) |
		    (HEVC_BSP_AUX_SUFFIX_SIZE >> 4);
	amvdec_write_dos(core, HEVC_ASSIST_SCRATCH_M, size_word);

	dev_info(core->dev,
		 "BSP HEVC: programmed AUX buffer addr=0x%pad size_word=0x%08x (prefix=%uKB suffix=%uKB)\n",
		 &aux_paddr, size_word,
		 HEVC_BSP_AUX_PREFIX_SIZE / 1024,
		 HEVC_BSP_AUX_SUFFIX_SIZE / 1024);
}

void codec_hevc_bsp_program_mpred_init(struct amvdec_session *sess,
				       dma_addr_t mv_paddr,
				       u32 mv_buf_size,
				       dma_addr_t abv_paddr)
{
	struct amvdec_core *core = sess->core;

	if (WARN_ON_ONCE(!hevc_bsp_proto_active(sess)))
		return;

	/* MPRED MV write/read addresses for first-picture bootstrap.
	 * BSP firmware runs MPRED hardware engine immediately after MPSR=1;
	 * with these registers at hardware-default 0 the engine AXI-writes
	 * colocated MVs to physical address 0 -> bus fault -> hard lockup.
	 *
	 * Per-slice handler (codec_hevc_set_mpred() in codec_hevc.c) will
	 * overwrite these with per-frame slot addresses on every
	 * SLICE_SEGMENT_DONE event.  This call just establishes safe
	 * initial state covering the window between MPSR=1 and the
	 * first per-slice ISR.
	 *
	 * RD_END_ADDR is set to (mv_paddr + mv_buf_size - 4) so the
	 * MPRED range check passes on the first MV read attempt.
	 */
	amvdec_write_dos(core, HEVC_MPRED_MV_WR_START_ADDR, (u32)mv_paddr);
	amvdec_write_dos(core, HEVC_MPRED_MV_RD_START_ADDR, (u32)mv_paddr);
	amvdec_write_dos(core, HEVC_MPRED_MV_RD_END_ADDR,
			 (u32)mv_paddr + mv_buf_size - 4);

	/* MPRED above-row buffer base. Same fault class as the MV addrs:
	 * defaults to 0, hardware AXI-reads this address on the first
	 * non-trivial CTU row.
	 */
	amvdec_write_dos(core, HEVC_MPRED_ABV_START_ADDR, (u32)abv_paddr);

	/* SCRATCH_H = codec type tag.  BSP firmware reads this at PC 0x0056
	 * (per reset-vector trace) and bit-extracts into LMEM[0x55] which
	 * gates coefficient/transform initialisation.  Value 0x01 = 8-bit
	 * HEVC single-instance (matches BSP vh265.c codec-type value for
	 * the configuration upstream uses).
	 */
	amvdec_write_dos(core, HEVC_ASSIST_SCRATCH_H, 0x01);

	dev_info(core->dev,
		 "BSP HEVC: programmed MPRED init mv=0x%pad+0x%x abv=0x%pad SCRATCH_H=0x01\n",
		 &mv_paddr, mv_buf_size, &abv_paddr);
}

/* ==========================================================
 * hevc_bsp session hooks -- primary BSP_PROTO entry points.
 * ==========================================================
 *
 * codec_hevc.c calls these three functions under a single
 * BSP_PROTO quirk gate at session_start / setup_workspace /
 * session_stop, replacing the scattered inline gates that
 * accumulated during Phases A.2 / A.3.1 / A.3.2 / VDEC_1-power.
 *
 * Design rationale: keep hevc_bsp as a separate code path
 * inside the same compilation unit.  Callers remain simple
 * dispatchers; all BSP-protocol orchestration lives here.
 */

int codec_hevc_bsp_session_start(struct amvdec_session *sess,
				 struct codec_hevc_common *comm)
{
	struct amvdec_core *core = sess->core;
	int ret;

	if (WARN_ON_ONCE(!hevc_bsp_proto_active(sess)))
		return -EINVAL;

	/* Power VDEC_1 alongside HEVC.  BSP driver does this via
	 * vdec_core_request(CORE_MASK_VDEC_1 | CORE_MASK_HEVC) --
	 * BSP firmware's AXI accesses to shared DOS bus fault
	 * hard if VDEC_1 is unpowered (TASK-004 breakthrough).
	 *
	 * Side effect (TASK-004 stream-FIFO root cause): vdec_1_poweron
	 * does DOS_SW_RESET3 = 0xffffffff which resets ALL DOS blocks
	 * including HEVC, clearing HEVC_STREAM_START_ADDR / END_ADDR /
	 * RD_PTR / WR_PTR that __vdec_hevc_start's vdec_hevc_stbuf_init
	 * wrote earlier.  Upstream-firmware sessions don't call
	 * vdec_1_power_get so their stream regs survive; BSP sessions
	 * end up with START_ADDR=0 -> esparser sees STREAM_LEVEL=
	 * WR-RD=base-0=~3GB -> "VIFIFO usage > size" -> refuses to queue
	 * -> parser starves forever (HEVC_MPRED_CUR_POC=0, IRQ_total=0).
	 *
	 * Re-write the stream FIFO base/end here to restore them.  Driver
	 * has the values (sess->vififo_paddr, sess->vififo_size) so we
	 * don't need to touch vdec_hevc.c.  RD_PTR/WR_PTR are left at
	 * their post-reset state (=0) -- esparser will fix WR_PTR when it
	 * queues data, and the parser hardware initialises RD_PTR from
	 * START_ADDR when HEVC_STREAM_CTRL_ENABLE is set later.
	 */
	ret = vdec_1_power_get(core, false);	/* decode-side HEVC-BSP path */
	if (ret)
		return ret;

	amvdec_write_dos(core, HEVC_STREAM_START_ADDR, sess->vififo_paddr);
	amvdec_write_dos(core, HEVC_STREAM_END_ADDR,
			 sess->vififo_paddr + sess->vififo_size);
	dev_info(core->dev,
		 "BSP HEVC: restored HEVC_STREAM_START_ADDR=0x%pad END_ADDR=+0x%x after vdec_1_poweron full DOS reset\n",
		 &sess->vififo_paddr, sess->vififo_size);

	ret = codec_hevc_bsp_alloc_rpm(sess,
				       &comm->bsp_rpm_vaddr,
				       &comm->bsp_rpm_paddr);
	if (ret)
		goto put_vdec1;

	ret = codec_hevc_bsp_alloc_aux(sess,
				       &comm->bsp_aux_vaddr,
				       &comm->bsp_aux_paddr);
	if (ret)
		goto free_rpm;

	return 0;

free_rpm:
	codec_hevc_bsp_free_rpm(sess, comm->bsp_rpm_vaddr, comm->bsp_rpm_paddr);
	comm->bsp_rpm_vaddr = NULL;
	comm->bsp_rpm_paddr = 0;
put_vdec1:
	vdec_1_power_put(core);
	return ret;
}

void codec_hevc_bsp_session_setup_workspace(struct amvdec_session *sess,
					    struct codec_hevc_common *comm,
					    dma_addr_t mpred_mv_paddr,
					    u32 mpred_mv_buf_size,
					    dma_addr_t mpred_abv_paddr)
{
	struct amvdec_core *core = sess->core;

	if (WARN_ON_ONCE(!hevc_bsp_proto_active(sess)))
		return;

	codec_hevc_bsp_program_rpm(sess, comm->bsp_rpm_paddr);
	codec_hevc_bsp_program_aux(sess, comm->bsp_aux_paddr);
	codec_hevc_bsp_program_mpred_init(sess,
					  mpred_mv_paddr,
					  mpred_mv_buf_size,
					  mpred_abv_paddr);

	/* Enable HEVC MBOX0 (in addition to upstream's MBOX1).
	 *
	 * BSP firmware signals slice-done events via MBOX0
	 * (vh265.c:13272-13275 writes HEVC_ASSIST_MBOX0_CLR_REG=1 then
	 * MBOX0_MASK=1).  Upstream's codec_hevc_start enables only MBOX1.
	 *
	 * The DT exposes a single shared 'vdec' GIC IRQ; both mailboxes
	 * raise it.  Enabling MBOX0_MASK lets BSP firmware's MBOX0
	 * writes propagate to the GIC line and wake the ARM ISR.  The
	 * existing per-codec hard ISR (codec_hevc_isr) reads
	 * HEVC_DEC_STATUS_REG agnostic of which mailbox triggered.
	 *
	 * Without this, BSP firmware runs init then waits forever for
	 * ARM to ack via MBOX0; ARM never wakes; STREAM_RD_PTR stays 0,
	 * IRQ counters stay 0, decode never advances.  Symptoms documented
	 * in task004-bsp-fw-full-snapshot-20260421.md (top-1 hypothesis).
	 */
	amvdec_write_dos(core, HEVC_ASSIST_MBOX0_CLR_REG, 1);
	amvdec_write_dos(core, HEVC_ASSIST_MBOX0_MASK, 1);
	dev_info(core->dev, "BSP HEVC: enabled HEVC_ASSIST_MBOX0 (in addition to MBOX1)\n");
}

void codec_hevc_bsp_session_stop(struct amvdec_session *sess,
				 struct codec_hevc_common *comm)
{
	struct amvdec_core *core = sess->core;

	/* session_stop is called on every session teardown; only act
	 * if this was actually a BSP_PROTO session.  Safe to call
	 * unconditionally -- the caller does the quirk check once.
	 */
	if (!hevc_bsp_proto_active(sess))
		return;

	codec_hevc_bsp_free_aux(sess, comm->bsp_aux_vaddr, comm->bsp_aux_paddr);
	comm->bsp_aux_vaddr = NULL;
	comm->bsp_aux_paddr = 0;

	codec_hevc_bsp_free_rpm(sess, comm->bsp_rpm_vaddr, comm->bsp_rpm_paddr);
	comm->bsp_rpm_vaddr = NULL;
	comm->bsp_rpm_paddr = 0;

	vdec_1_power_put(core);
}
