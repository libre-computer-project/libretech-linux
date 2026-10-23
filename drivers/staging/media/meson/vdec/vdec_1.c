// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018 BayLibre, SAS
 * Author: Maxime Jourdan <mjourdan@baylibre.com>
 *
 * VDEC_1 is a video decoding block that allows decoding of
 * MPEG 1/2/4, H.263, H.264, MJPEG, VC1
 */

#include <linux/firmware.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/reset.h>	/* hevcf_dmc_pipl_reset (KI-121 SM1 RESET7) */
#include <linux/delay.h>

#include "vdec_1.h"
#include "vdec_hevc.h"
#include "vdec_helpers.h"
#include "codec_avs.h"
#include "dos_regs.h"
#include "hevc_regs.h"		/* HEVC_CORE_ENABLE (KI-121 CE re-assert) */

/* AO Registers */
#define AO_RTI_GEN_PWR_SLEEP0	0xe8
	/* SLEEP0 bit definitions (power on/off) */
	#define GEN_PWR_VDEC_1 (BIT(3) | BIT(2))
	#define GEN_PWR_VDEC_1_SM1 (BIT(1))
	#define GEN_PWR_VDEC_HEVC (BIT(7) | BIT(6))
	#define GEN_PWR_VDEC_HEVC_SM1 (BIT(2))
#define AO_RTI_GEN_PWR_ISO0	0xec
	/*
	 * ISO0 bit definitions (isolation control).
	 * NOTE: ISO0 uses DIFFERENT bit positions from SLEEP0 on non-SM1!
	 * HEVC isolation is at bits 10-11 in ISO0 (0xc00) but bits 6-7 in
	 * SLEEP0 (0xc0).  VDEC_1 isolation is at bits 6-7 in ISO0 (0xc0)
	 * but bits 2-3 in SLEEP0 (0x0c).  SM1 uses the same bits for both.
	 */
	#define GEN_PWR_ISO0_HEVC (BIT(11) | BIT(10))       /* 0xc00 */
	#define GEN_PWR_ISO0_VDEC_1 (BIT(7) | BIT(6))       /* 0xc0 */
	#define GEN_PWR_ISO0_HEVC_SM1 (BIT(2))
	#define GEN_PWR_ISO0_VDEC_1_SM1 (BIT(1))

#define MC_SIZE			(4096 * 4)

/*
 * VDEC_1 decoder turbo clock -- DISABLED.
 *
 * The VDEC_1 hardware block cannot operate above 800 MHz.  At 1 GHz
 * via gp0_pll, the H.264 decoder stalls at MBY_MBX=0 on every frame
 * (tested on G12B A311D, 2026-04-20).  The HCODEC encoder block on
 * the same SoC runs fine at 1 GHz, so this is a per-IP timing limit.
 *
 * The module param is retained (read-only) so users see the parameter
 * exists but is locked at 0.  Removing it would break scripts that
 * set all three turbo params in one tee command.
 */
static unsigned int vdec1_turbo;
module_param(vdec1_turbo, uint, 0444);
MODULE_PARM_DESC(vdec1_turbo,
		 "VDEC_1 decoder turbo: DISABLED (HW limit, 800MHz max)");

static const unsigned long vdec1_turbo_rates[] = {
	800000000,	/* 0: 800 MHz max for VDEC_1 decode HW */
};

#define VDEC1_TURBO_MAX (ARRAY_SIZE(vdec1_turbo_rates) - 1)

/*
 * vdec1_target_rate - effective VDEC_1 clock rate
 *
 * Returns turbo_rate when not thermally throttled (state 0), otherwise
 * returns the thermal rate.  Thermal ALWAYS wins over turbo for safety.
 */
static unsigned long vdec1_target_rate(struct amvdec_core *core)
{
	unsigned long thermal_rate = amvdec_clock_rate(core);
	unsigned int level = min_t(unsigned int, vdec1_turbo, VDEC1_TURBO_MAX);
	unsigned long turbo_rate = vdec1_turbo_rates[level];

	if (thermal_rate < 800000000)
		return thermal_rate;
	return turbo_rate;
}

/*
 * VDEC_1 idle autosuspend.
 *
 * Instead of powering off immediately at STREAMOFF, schedule a delayed
 * poweroff.  If another STREAMON arrives before the delay expires, cancel
 * the work and skip the power cycle entirely.
 *
 * Default idle timeout: 100ms. Settable via module param for tuning.
 */
static unsigned int vdec1_idle_ms = 100;
module_param(vdec1_idle_ms, uint, 0644);
MODULE_PARM_DESC(vdec1_idle_ms,
		 "VDEC_1 idle poweroff delay in ms (default=100, 0=immediate)");

/* KI-118 block manager (defined in hcodec.c, same meson-vdec-core module):
 * gates the decode-side wait for a clean power-cycle when VDEC_1 is currently
 * owned by the HCODEC pin (encode-first). */
extern bool ki118_block_manager;

static int
vdec_1_load_firmware(struct amvdec_session *sess, const char *fwname)
{
	const struct firmware *fw;
	struct amvdec_core *core = sess->core;
	struct device *dev = core->dev_dec;
	struct amvdec_codec_ops *codec_ops = sess->fmt_out->codec_ops;
	const struct amvdec_fw_version *match;
	static void *mc_addr;
	static dma_addr_t mc_addr_map;
	int ret;
	u32 i = 1000;

	ret = request_firmware(&fw, fwname, dev);
	if (ret < 0)
		return -EINVAL;

	mc_addr = dma_alloc_coherent(core->dev, MC_SIZE,
				     &mc_addr_map, GFP_KERNEL);
	if (!mc_addr) {
		ret = -ENOMEM;
		goto release_firmware;
	}

	/*
	 * Copy the IMEM portion of the firmware (up to MC_SIZE bytes) and
	 * zero-fill any remainder.  Firmware sizes vary:
	 *   - SM1 AVS: 12 KiB (smaller than MC_SIZE) -- zero-padded
	 *   - MPEG12/MPEG4: 16 KiB (exactly MC_SIZE)
	 *   - H.264: 36 KiB (larger than MC_SIZE) -- only first 16 KiB is
	 *     IMEM; the rest is extended firmware handled by
	 *     load_extended_firmware() below
	 */
	memset(mc_addr, 0, MC_SIZE);
	memcpy(mc_addr, fw->data, min_t(size_t, fw->size, MC_SIZE));

	/*
	 * Identify the firmware binary by SHA256 and store the match on the
	 * session.  Must be done AFTER memcpy (we hash fw->data) and BEFORE
	 * any firmware-specific byte patches so that quirk-gated patches can
	 * inspect sess->fw_match.
	 *
	 * amvdec_fw_identify() returns ERR_PTR(-ENOENT) when a non-empty
	 * table was provided but no entry matched -- propagate as load error.
	 * NULL means no table (unwired codec); session continues normally.
	 */
	match = amvdec_fw_identify(dev, fwname,
				   fw->data, fw->size,
				   sess->fmt_out->fw_versions,
				   sess->fmt_out->fw_versions_count);
	if (IS_ERR(match)) {
		ret = PTR_ERR(match);
		goto free_mc;
	}
	sess->fw_match = match;  /* NULL if codec is unwired, or valid entry */

	/*
	 * Patch gxl_mpeg12.bin firmware bug: the sequence_end handler at
	 * address 0x02ef uses "bgti r9, 3" to check if the current canvas
	 * slot needs flushing.  This skips slots 4-7, causing the last
	 * decoded frame to be silently dropped when it lands in those slots.
	 * Fix: change to "bgti r9, 7" (byte 0xBBD: 0x80 -> 0x81).
	 *
	 * Guard: check firmware name, size, and the 4-byte instruction
	 * at the bug site (c9 80 e3 0a at offset 0xBBC) to avoid patching
	 * a fixed or different firmware version. A patched firmware would
	 * have 0x81 at 0xBBD instead of 0x80 and would not match.
	 */
	if (strstr(fwname, "mpeg12") &&
	    fw->size == MC_SIZE &&
	    ((u8 *)mc_addr)[0xBBC] == 0xc9 &&
	    ((u8 *)mc_addr)[0xBBD] == 0x80 &&
	    ((u8 *)mc_addr)[0xBBE] == 0xe3 &&
	    ((u8 *)mc_addr)[0xBBF] == 0x0a) {
		((u8 *)mc_addr)[0xBBD] = 0x81;
		dev_info(dev, "Patched MPEG12 firmware canvas flush bug\n");
	}

	/*
	 * Patch gxl_mpeg12.bin BUFFEROUT race on G12A+: the firmware's
	 * picture-done handler (address 0x038f) reads BUFFEROUT and, if
	 * non-zero, gives up and returns to the main loop without
	 * signaling the current decoded frame.  On GXL (28nm), the ARM
	 * core's IRQ handler clears BUFFEROUT fast enough that the
	 * firmware rarely sees it non-zero.  On G12A/SM1 (12nm), the
	 * AMRISC decodes faster than the ARM can service IRQs, causing
	 * 30-70% frame drops.
	 *
	 * Fix: replace the single-check-and-give-up at 0x0390-0x0393
	 * with a spin-wait loop, matching the firmware's own EOS flush
	 * path at 0x0300-0x0301 which already spin-waits for BUFFEROUT=0.
	 *
	 * Original (0x0390-0x0393):
	 *   beqi  r9, 0, +4     # if BUFFEROUT==0, skip ahead
	 *   li    r7, 2          # set state = 2
	 *   b     -864           # goto main loop (give up)
	 *   sta   r7, [0x7]      # (delay slot) store state
	 *
	 * Patched:
	 *   bneqi r9, 0, -1     # if BUFFEROUT!=0, goto 0x038f (re-read)
	 *   nop                  # (delay slot: safe)
	 *   nop
	 *   nop
	 *
	 * Guard: verify the 4 instruction words at 0x0390-0x0393 match
	 * the known unpatched firmware.  Apply only on G12A+ where the
	 * race is observed.
	 */
	if (strstr(fwname, "mpeg12") &&
	    fw->size == MC_SIZE &&
	    core->platform->revision >= VDEC_REVISION_G12A &&
	    ((u8 *)mc_addr)[0xE40] == 0x09 &&
	    ((u8 *)mc_addr)[0xE41] == 0x00 &&
	    ((u8 *)mc_addr)[0xE42] == 0xa1 &&
	    ((u8 *)mc_addr)[0xE43] == 0x0a &&
	    ((u8 *)mc_addr)[0xE44] == 0x87 &&
	    ((u8 *)mc_addr)[0xE45] == 0x00 &&
	    ((u8 *)mc_addr)[0xE46] == 0x80 &&
	    ((u8 *)mc_addr)[0xE47] == 0x06) {
		/* 0x0390: beqi r9,0,+4 -> bneqi r9,0,-1 */
		((u8 *)mc_addr)[0xE41] = 0xC0;
		((u8 *)mc_addr)[0xE42] = 0x6F;
		/* 0x0391-0x0393: NOP out the give-up path */
		memset((u8 *)mc_addr + 0xE44, 0, 12);
		dev_info(dev,
			 "Patched MPEG12 firmware BUFFEROUT race (G12A+)\n");
	}

	amvdec_write_dos(core, MPSR, 0);
	amvdec_write_dos(core, CPSR, 0);

	amvdec_clear_dos_bits(core, MDEC_PIC_DC_CTRL, MDEC_PIC_DC_CTRL_OLD_MODE);

	amvdec_write_dos(core, IMEM_DMA_ADR, mc_addr_map);
	amvdec_write_dos(core, IMEM_DMA_COUNT, MC_SIZE / 4);
	amvdec_write_dos(core, IMEM_DMA_CTRL, IMEM_DMA_CTRL_RUN);

	while (--i && amvdec_read_dos(core, IMEM_DMA_CTRL) & IMEM_DMA_CTRL_START);

	if (i == 0) {
		dev_err(dev, "Firmware load fail (DMA hang?)\n");
		ret = -EINVAL;
		goto free_mc;
	}

	/*
	 * Patch AVS firmware: NOP the VLD_STATUS_CTRL busy check.
	 *
	 * The AVS firmware main loop at address 0x07dc checks
	 * VLD_STATUS_CTRL bits[1:0] and exits if non-zero (VLD busy).
	 * When the esparser feeds data before firmware start (V4L2 M2M
	 * flow), the VLD has already processed VIFIFO data and bit 0 is
	 * permanently set.  The firmware sees VLD busy and never decodes.
	 *
	 * Fix: NOP the 3-instruction check-and-exit sequence at 0x07dc-07de
	 * so the firmware always falls through to the VIFIFO data check.
	 *
	 * Original (byte 0x1F70): bnandi r3,3,+4; li r2,0; b +36
	 * Patched:                nop; nop; nop
	 */
	if (sess->fw_match &&
	    (sess->fw_match->quirks & AMVDEC_FW_QUIRK_AVS_VLD_BUSY_PATCH)) {
		/*
		 * gxl_avs.bin main loop at AMRISC 0x07dc polls VLD_STATUS_CTRL
		 * bits[1:0] and exits if non-zero.  In the V4L2 M2M flow the
		 * esparser feeds data before firmware start so bit 0 is latched.
		 * NOP the 3-instruction check so the loop falls through.
		 *
		 * The guard is now the firmware hash -- only gxl_avs.bin has the
		 * bug, identified by SHA256.  Preserved byte assertion left for
		 * extra safety in case a hash collision ever lines up.
		 */
		if (((u8 *)mc_addr)[0x1F70] == 0xc3 &&
		    ((u8 *)mc_addr)[0x1F71] == 0x00 &&
		    ((u8 *)mc_addr)[0x1F72] == 0x21 &&
		    ((u8 *)mc_addr)[0x1F73] == 0x0a) {
			memset((u8 *)mc_addr + 0x1F70, 0, 12);
			dev_info(dev,
				 "firmware workaround: patched AVS VLD busy check at 0x07dc (hash-matched gxl_avs.bin)\n");
		} else {
			dev_warn(dev,
				 "AVS VLD busy patch: quirk set but bytes don't match; firmware may already be patched or is a different variant\n");
		}
	}

	/*
	 * Patch MPEG4 firmware: NOP the VLD_STATUS_CTRL busy check in the
	 * main polling loop at AMRISC address 0x06b1.
	 *
	 *   06b0: mfcr r2, VLD_STATUS_CTRL
	 *   06b1: bandi r2, 1, +5        <- branch if bit 0 set -> 06b6
	 *   06b2: blti  r37, 64, +7      <- fall-through to frame process
	 *
	 * On GXL (28nm), the VLD hardware latches "data available" bit 0
	 * when the esparser feeds data before MPSR=1.  The firmware poll
	 * loop sees bit 0 set, branches to 0x06b6 (reset counter, return),
	 * and never enters frame processing.  NOP the check so the loop
	 * always falls through to the counter test at 0x06b2.
	 *
	 * On G12A+ (12nm), this patch is NOT needed and breaks decode:
	 * the VLD busy check at 0x06b1 is part of the firmware's normal
	 * flow control.  G12B MPEG4 was 28/30 frames without this patch
	 * (baseline 98e927c0126d).  Applying it unconditionally caused
	 * 0/0/0 IRQs on G12B (verified 2026-04-20 on 7a31c9b5cc0b).
	 *
	 * Guard: GXL-only + firmware name contains "mpeg4" + 4 guard
	 * bytes at byte offset 0x1AC4 match the known unpatched
	 * instruction 0x0be14042 (LE: 42 40 e1 0b).
	 */
	if (core->platform->revision < VDEC_REVISION_G12A &&
	    strstr(fwname, "mpeg4") && fw->size >= MC_SIZE &&
	    ((u8 *)mc_addr)[0x1AC4] == 0x42 &&
	    ((u8 *)mc_addr)[0x1AC5] == 0x40 &&
	    ((u8 *)mc_addr)[0x1AC6] == 0xe1 &&
	    ((u8 *)mc_addr)[0x1AC7] == 0x0b) {
		memset((u8 *)mc_addr + 0x1AC4, 0, 4);
		dev_info(dev,
			 "Patched MPEG4 firmware VLD busy check (NOP at 0x06b1, GXL only)\n");
	}

	/*
	 * KI-108 D134: MVC two-view CAVLC 2c dependent-slice recon DISPATCH.
	 *
	 * The dependent (type-20) slice PARKS at flat PC 0x05f3 (the [0xa]
	 * inter-view-ref sentinel gate in the shared recon-setup 0x05d0): for the
	 * dependent view, [0xa]==32767 (built sentinel, no base canvas registered
	 * as the view-1 inter-view ref), so 0x05f2 falls to 0x05f3 `b -> 0x040e`
	 * (PARK) and the A=5 slice-ucode load never fires for view-1 -> MBY_MBX
	 * frozen (D133).  Author: divert the park into a GUARDED resident block that,
	 * for the dependent view only, seeds [0xa]=0 (= DPB slot 0 = the base view's
	 * frame = the correct minimal inter-view reference; the MC ref ADDRESS is
	 * already driver-supplied via AV_SCRATCH_M/N, consumed at fw 0x0a0c) and
	 * proceeds to recon.  The [0x56] guard leaves the BASE view untouched.  The
	 * view-1 canvas is already [0x56]-parameterized (fw 0x0a72/0x0a74 = base-idx
	 * + [0x8]) so the dependent recon writes canvas1, DISTINCT from canvas0.
	 * Full RE + vici-asm round-trip verification: amlogic .../h264/mvc/
	 * ki108-fix-approach.md D134, source of truth mvc-2b2c-dispatch.asm.
	 *
	 * Deployment split (per the header re-DMA-from-swap architecture):
	 *  - The RESIDENT block goes at flat PC 0x016a (IMEM 0x16a, BELOW the
	 *    0x400 overlay base -> never overlay-swapped), patched HERE in mc_addr
	 *    (data byte 0x5a8), guarded on 32 all-zero (nop) bytes.
	 *  - The TRAMPOLINE over the 0x05f3 park is in the HEADER-segment SWAP copy
	 *    (swap[0x7cc], PC 0x05f3 is IMEM 0x5f3 > 0x400 = overlay-swapped from
	 *    swap) and is applied in codec_h264_mvc.c load_extended_firmware.
	 *
	 * The 8-word resident block (LE), PC 0x016a..0x0171 at flat 0x5a8:
	 *   016a lda r9,[0x56]      016b beqi r9,0,+5 (->0170 base proceed)
	 *   016c li r8,0            016d sta r8,[0xa]  (dependent: [0xa]=0 unpark)
	 *   016e b +1159 (->05f5)   016f nop
	 *   0170 b +1157 (->05f5)   0171 nop
	 */
	if (strstr(fwname, "h264") && strstr(fwname, "mvc") &&
	    fw->size == 28 * 1024 &&
	    /* flat 0x016a-0x0171 is a free NOP run (32 bytes all zero) */
	    ((u8 *)mc_addr)[0x5a8] == 0x00 && ((u8 *)mc_addr)[0x5a9] == 0x00 &&
	    ((u8 *)mc_addr)[0x5aa] == 0x00 && ((u8 *)mc_addr)[0x5ab] == 0x00 &&
	    ((u8 *)mc_addr)[0x5c4] == 0x00 && ((u8 *)mc_addr)[0x5c5] == 0x00 &&
	    ((u8 *)mc_addr)[0x5c6] == 0x00 && ((u8 *)mc_addr)[0x5c7] == 0x00) {
		/* 8-word (32-byte) block, vici-asm output (finalblock.bin, D134) */
		static const u8 d134_dispatch[8][4] = {
			{ 0x89, 0x15, 0x80, 0x07 }, /* 016a lda   r9,[0x56]      */
			{ 0x09, 0x40, 0xa1, 0x0a }, /* 016b beqi  r9,0,+5 ->0170 */
			{ 0x08, 0x00, 0x80, 0x06 }, /* 016c li    r8,0 (delay)   */
			{ 0x88, 0x02, 0xc0, 0x07 }, /* 016d sta   r8,[0xa] unpark*/
			{ 0xc0, 0x21, 0x79, 0x0c }, /* 016e b +1159 ->05f5       */
			{ 0x00, 0x00, 0x00, 0x00 }, /* 016f  nop (delay slot)    */
			{ 0x40, 0x21, 0x79, 0x0c }, /* 0170 b +1157 ->05f5       */
			{ 0x00, 0x00, 0x00, 0x00 }, /* 0171  nop (delay slot)    */
		};
		int w;

		for (w = 0; w < 8; w++)
			memcpy((u8 *)mc_addr + 0x5a8 + w * 4, d134_dispatch[w], 4);

		dev_warn(dev,
			 "KI-108 D134: installed MVC 2c dispatch block at PC 0x016a (fw=%s) -- dependent-slice recon unpark ([0xa]=0 for view-1), guarded on [0x56]; trampoline over 0x05f3 park applied in codec_h264_mvc\n",
			 fwname);
	} else if (strstr(fwname, "h264") && strstr(fwname, "mvc")) {
		dev_warn(dev,
			 "KI-108 D134: MVC 2c dispatch block SKIPPED (fw=%s size=%zu or 0x5a8 not a free NOP run; running unmodified)\n",
			 fwname, fw->size);
	}

	/*
	 * KI-108 D135: MVC two-view CAVLC 2c dependent-slice recon ROUTING.
	 *
	 * D134's 0x05f3 unpark was placed DOWNSTREAM of the real wall: HW (G12B
	 * .148, MPC_E samples) showed the dependent (type-20) slice NEVER reaches
	 * 0x05f3 -- it parks EARLIER, dropped at the NAL-dispatch view-id filter and
	 * recycled in the 0x0704-0x0725 VLD loop, so it is never ROUTED into recon
	 * (0x0456 -> 0x05d0) at all.  The drop is at flat PC 0x043c
	 * (`beqi [0x4f],255 -> 0x0442 -> 0x040e`): [0x4f] is a dead 0xFF sentinel
	 * (never written to a real view_id, D22), so EVERY type-20 slice fails the
	 * view-id compare and is dropped -- while the base slice (type 1/5) bypasses
	 * this filter entirely (0x042f -> 0x044d directly).  THAT is the base-vs-
	 * dependent routing divergence (ki108-fix-approach.md D135).
	 *
	 * Author: divert 0x043c into a GUARDED resident route block that, for the
	 * remapped type-20 slice ([0x48]=1/5, set at 0x0437), sets [0x56]=1
	 * (dependent view flag) and proceeds to 0x0446 (the poc/frame_num vrd reads
	 * -> 0x044c reloads [0x48]=1/5 -> 0x044d beqi 5/1 -> 0x0456 -> 0x05d0 recon,
	 * MIRRORING the base route).  A [0x48]==14 guard preserves the ORIGINAL drop
	 * for the type-14 prefix NAL (the driver strips those pre-queue, but belt-
	 * and-suspenders).  Once the dependent slice reaches 0x05d0 it hits the
	 * D134 [0xa] park at 0x05f3, which the D134 block unparks -- so D134 + D135
	 * together carry the dependent slice from NAL-dispatch through to A=5/recon.
	 *
	 * Deployment split (same architecture as D134):
	 *  - The RESIDENT route block goes at flat PC 0x0172 (IMEM 0x172, below the
	 *    0x400 overlay base -> never overlay-swapped), patched HERE in mc_addr
	 *    (data byte 0x5c8, immediately after the D134 block), guarded on 32
	 *    all-zero (nop) bytes.
	 *  - The TRAMPOLINE over the 0x043c drop is in the HEADER-segment SWAP copy
	 *    (swap[0xf0], PC 0x043c is IMEM 0x43c > 0x400 = overlay-swapped from
	 *    swap) and is applied in codec_h264_mvc.c load_extended_firmware.
	 *
	 * The 8-word route block (LE), PC 0x0172..0x0179 at flat 0x5c8:
	 *   0172 lda r8,[0x48]      0173 beqi r8,14,+4 (->0177 orig drop)
	 *   0174 li r8,1            0175 sta r8,[0x56] (dependent view flag)
	 *   0176 b +720 (->0446)    0177 nop
	 *   0178 b +714 (->0442)    0179 nop  (type-14: original drop path)
	 */
	if (strstr(fwname, "h264") && strstr(fwname, "mvc") &&
	    fw->size == 28 * 1024 &&
	    /* flat 0x0172-0x0179 is a free NOP run (32 bytes all zero) */
	    ((u8 *)mc_addr)[0x5c8] == 0x00 && ((u8 *)mc_addr)[0x5c9] == 0x00 &&
	    ((u8 *)mc_addr)[0x5ca] == 0x00 && ((u8 *)mc_addr)[0x5cb] == 0x00 &&
	    ((u8 *)mc_addr)[0x5e4] == 0x00 && ((u8 *)mc_addr)[0x5e5] == 0x00 &&
	    ((u8 *)mc_addr)[0x5e6] == 0x00 && ((u8 *)mc_addr)[0x5e7] == 0x00) {
		/* 8-word (32-byte) block, vici-asm output (routeblock.bin, D135) */
		static const u8 d135_route[8][4] = {
			{ 0x08, 0x12, 0x80, 0x07 }, /* 0172 lda   r8,[0x48]       */
			{ 0x88, 0x03, 0xa1, 0x0a }, /* 0173 beqi  r8,14,+4 ->0177 */
			{ 0x48, 0x00, 0x80, 0x06 }, /* 0174 li    r8,1 (delay)    */
			{ 0x88, 0x15, 0xc0, 0x07 }, /* 0175 sta   r8,[0x56] dep   */
			{ 0x00, 0xb4, 0x78, 0x0c }, /* 0176 b +720 ->0446         */
			{ 0x00, 0x00, 0x00, 0x00 }, /* 0177  nop (orig-drop tgt)  */
			{ 0x80, 0xb2, 0x78, 0x0c }, /* 0178 b +714 ->0442 (drop)  */
			{ 0x00, 0x00, 0x00, 0x00 }, /* 0179  nop (delay slot)     */
		};
		int w;

		for (w = 0; w < 8; w++)
			memcpy((u8 *)mc_addr + 0x5c8 + w * 4, d135_route[w], 4);

		dev_warn(dev,
			 "KI-108 D135: installed MVC 2c ROUTE block at PC 0x0172 (fw=%s) -- routes type-20 dependent slice to recon ([0x56]=1 -> 0x0446 -> 0x044d -> 0x0456 -> 0x05d0), guarded on [0x48]==14 (type-14 drop preserved); trampoline over 0x043c drop applied in codec_h264_mvc\n",
			 fwname);
	} else if (strstr(fwname, "h264") && strstr(fwname, "mvc")) {
		dev_warn(dev,
			 "KI-108 D135: MVC 2c ROUTE block SKIPPED (fw=%s size=%zu or 0x5c8 not a free NOP run; running unmodified)\n",
			 fwname, fw->size);
	}

	/*
	 * KI-108 D136: MVC two-view CAVLC -- SECOND A=5 per-MB pass for the
	 * dependent slice (slice-pipeline re-entry, the CORE of sub-system 2c).
	 *
	 * D135 ROUTED the type-20 dependent slice into the shared recon-setup
	 * (HW reached fw 0x0594 PMV2_Y) but MBY_MBX stayed FROZEN at 0x271d and
	 * no A=5 fired -- D50's "the firmware runs ONE per-MB pass per AU."  RE
	 * (this session) localized the exact one-pass gate: the C-core per-MB
	 * grid's MB-COUNTER init at fw 0x107e (AV_SCRATCH_9++, [0x85]/[0x86]=0,
	 * [0x87]=MB-address=0 = the MBY_MBX reset) is gated in the slice ucode at
	 * fw 0x102d `beqi r17,0,+3 -> 0x1030` (r17 = [0x5b]).  [0x5b] is the
	 * recon-setup "new-picture change-detect" bitfield (built fw 0x0611-0x0646
	 * from frame_num/POC deltas, stored fw 0x068f).  The DEPENDENT slice
	 * carries the BASE view's frame_num/POC (same AU) -> all deltas 0 ->
	 * [0x5b]==0 -> 0x102d SKIPS 0x107e -> [0x87] never re-zeroed, AV_SCRATCH_9
	 * never bumped -> MBY frozen, no second grid.  THAT is the CAVLC form of
	 * "one pass per AU."
	 *
	 * Author: force [0x5b] non-zero for the dependent slice ([0x56]==1) so the
	 * C-core's 0x102d sees r17!=0 and runs the grid re-arm 0x107e -> a SECOND
	 * MBY-counted per-MB grid into canvas1 (already [0x56]-parameterized, D134).
	 * The base view ([0x56]!=1) is UNTOUCHED.  No structural blocker: the C->M
	 * completion interrupt re-fires (MCPU_INTR_REQ 0x80 at fw 0x12a6/0x12b8),
	 * so a second 0x0800->0x0860->0x0887 (A=5) entry is reachable once [0x5b]!=0.
	 *
	 * Deployment split (same architecture as D134/D135):
	 *  - The RESIDENT re-entry block goes at flat PC 0x017a (IMEM 0x17a, below
	 *    the 0x400 overlay base -> never overlay-swapped), patched HERE in
	 *    mc_addr (data byte 0x5e8, immediately after the D135 block), guarded on
	 *    28 all-zero (nop) bytes.
	 *  - The TRAMPOLINE over the A=3 issue at fw 0x0690 (itcr 0x03,AV_SCRATCH_A;
	 *    IMEM 0x690 > 0x400 -> overlay-swapped from swap) is applied in
	 *    codec_h264_mvc.c load_extended_firmware (swap[0xa40]).  fw 0x068f
	 *    (sta r17,[0x5b]) is the DELAY SLOT of the 0x068e call and MUST NOT be
	 *    trampolined -- 0x0690 runs after [0x5b] is stored + 0x00ae returns.
	 *
	 * The 7-word re-entry block (LE), PC 0x017a..0x0180 at flat 0x5e8:
	 *   017a lda r9,[0x56]      017b bneqi r9,1,+3 (->017e do_a3)
	 *   017c li r8,2            017d sta r8,[0x5b] (dependent: [0x5b]=2 force !=0)
	 *   017e itcr 0x03,0x09ca (A=3, the original 0x0690)   017f b +1298 (->0691)
	 *   0180 nop (delay slot)
	 * Source of truth + RE + vici-asm round-trip: amlogic .../h264/mvc/
	 * mvc-2b2c-dispatch.asm (D136 section) + ki108-fix-approach.md D136.
	 */
	if (strstr(fwname, "h264") && strstr(fwname, "mvc") &&
	    fw->size == 28 * 1024 &&
	    /* flat 0x017a-0x0180 is a free NOP run (28 bytes all zero) */
	    ((u8 *)mc_addr)[0x5e8] == 0x00 && ((u8 *)mc_addr)[0x5e9] == 0x00 &&
	    ((u8 *)mc_addr)[0x5ea] == 0x00 && ((u8 *)mc_addr)[0x5eb] == 0x00 &&
	    ((u8 *)mc_addr)[0x600] == 0x00 && ((u8 *)mc_addr)[0x601] == 0x00 &&
	    ((u8 *)mc_addr)[0x602] == 0x00 && ((u8 *)mc_addr)[0x603] == 0x00) {
		/* 7-word (28-byte) block, vici-asm output (t136b.bin, D136) */
		static const u8 d136_reentry[7][4] = {
			{ 0x89, 0x15, 0x80, 0x07 }, /* 017a lda   r9,[0x56]      */
			{ 0x49, 0xc0, 0x60, 0x0a }, /* 017b bneqi r9,1,+3 ->017e */
			{ 0x88, 0x00, 0x80, 0x06 }, /* 017c li    r8,2 (delay)   */
			{ 0xc8, 0x16, 0xc0, 0x07 }, /* 017d sta   r8,[0x5b] !=0  */
			{ 0x03, 0xca, 0x09, 0x08 }, /* 017e itcr  0x03,0x09ca A=3*/
			{ 0x80, 0x44, 0x79, 0x0c }, /* 017f b +1298 ->0691       */
			{ 0x00, 0x00, 0x00, 0x00 }, /* 0180  nop (delay slot)    */
		};
		int w;

		for (w = 0; w < 7; w++)
			memcpy((u8 *)mc_addr + 0x5e8 + w * 4, d136_reentry[w], 4);

		dev_warn(dev,
			 "KI-108 D136: installed MVC second-A=5 re-entry block at PC 0x017a (fw=%s) -- forces [0x5b]!=0 for the dependent slice so the C-core grid-counter re-arm (fw 0x107e, gated at 0x102d) runs -> a SECOND per-MB pass (MBY reset + AV_SCRATCH_9 bump) into canvas1, guarded on [0x56]==1; trampoline over the A=3 issue 0x0690 applied in codec_h264_mvc\n",
			 fwname);
	} else if (strstr(fwname, "h264") && strstr(fwname, "mvc")) {
		dev_warn(dev,
			 "KI-108 D136: MVC second-A=5 re-entry block SKIPPED (fw=%s size=%zu or 0x5e8 not a free NOP run; running unmodified)\n",
			 fwname, fw->size);
	}

	/*
	 * CRITICAL: re-load the patched firmware into IMEM.
	 *
	 * The IMEM_DMA above ran BEFORE the AVS/MPEG4 patch block, so every
	 * patch applied to mc_addr after the first DMA (AVS VLD-busy 0x07dc,
	 * MPEG4 0x06b1) never reached the AMRISC IMEM -- mc_addr is freed at
	 * function end without being re-DMA'd.  Those patches were dead code.
	 * (MPEG12/H264 patches above the first DMA are unaffected.)
	 *
	 * Re-run the IMEM DMA now that mc_addr is fully patched.  MPSR is still
	 * 0 (codec start() raises it later), so this is the same load.
	 * Re-loading the already-applied pre-DMA patches is idempotent.
	 */
	i = 1000;
	amvdec_write_dos(core, IMEM_DMA_ADR, mc_addr_map);
	amvdec_write_dos(core, IMEM_DMA_COUNT, MC_SIZE / 4);
	amvdec_write_dos(core, IMEM_DMA_CTRL, IMEM_DMA_CTRL_RUN);
	while (--i && amvdec_read_dos(core, IMEM_DMA_CTRL) & IMEM_DMA_CTRL_START)
		;
	if (i == 0)
		dev_err(dev, "Firmware re-load (post-patch) IMEM DMA hang\n");

	if (codec_ops->load_extended_firmware)
		ret = codec_ops->load_extended_firmware(sess,
							fw->data, fw->size,
							MC_SIZE);

free_mc:
	dma_free_coherent(core->dev, MC_SIZE, mc_addr, mc_addr_map);
release_firmware:
	release_firmware(fw);
	return ret;
}

static int vdec_1_stbuf_power_up(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;

	amvdec_write_dos(core, VLD_MEM_VIFIFO_CONTROL, 0);
	amvdec_write_dos(core, VLD_MEM_VIFIFO_WRAP_COUNT, 0);
	amvdec_write_dos(core, POWER_CTL_VLD, POWER_CTL_VLD_ENABLE);

	amvdec_write_dos(core, VLD_MEM_VIFIFO_START_PTR, sess->vififo_paddr);
	amvdec_write_dos(core, VLD_MEM_VIFIFO_CURR_PTR, sess->vififo_paddr);
	amvdec_write_dos(core, VLD_MEM_VIFIFO_END_PTR,
			 sess->vififo_paddr + sess->vififo_size - 8);

	amvdec_write_dos_bits(core, VLD_MEM_VIFIFO_CONTROL, 1);
	amvdec_clear_dos_bits(core, VLD_MEM_VIFIFO_CONTROL, 1);

	amvdec_write_dos(core, VLD_MEM_VIFIFO_BUF_CNTL, MEM_BUFCTRL_MANUAL);
	amvdec_write_dos(core, VLD_MEM_VIFIFO_WP, sess->vififo_paddr);

	amvdec_write_dos_bits(core, VLD_MEM_VIFIFO_BUF_CNTL, 1);
	amvdec_clear_dos_bits(core, VLD_MEM_VIFIFO_BUF_CNTL, 1);

	/* FIFO count thresholds: bits 23:16=fill, bits 19:16=empty (0x11)
	 * These are hardware FIFO level thresholds for DMA refill logic.
	 */
	amvdec_write_dos_bits(core, VLD_MEM_VIFIFO_CONTROL,
			      (0x11 << MEM_FIFO_CNT_BIT) | MEM_FILL_ON_LEVEL |
			      MEM_CTRL_FILL_EN | MEM_CTRL_EMPTY_EN);

	return 0;
}

static void vdec_1_conf_esparser(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;

	/* VDEC_1 specific ESPARSER stuff */
	amvdec_write_dos(core, DOS_GEN_CTRL0, DOS_GEN_CTRL0_VDEC1);
	amvdec_write_dos(core, VLD_MEM_VIFIFO_BUF_CNTL, 1);
	amvdec_clear_dos_bits(core, VLD_MEM_VIFIFO_BUF_CNTL, 1);
}

static u32 vdec_1_vififo_level(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;

	return amvdec_read_dos(core, VLD_MEM_VIFIFO_LEVEL);
}

static void vdec1_clk_enable(struct amvdec_core *core)
{
	unsigned long target = vdec1_target_rate(core);

	/*
	 * If gp0_pll is available and target > 800 MHz, switch the mux
	 * parent to gp0_pll.  GXL has no gp0_pll in its VDEC_1 mux
	 * (only fclk_div3/4/5/7), so this path is skipped on GXL.
	 * The clk_set_parent call targets the MUX (vdec_sel), not the
	 * gate.  Walk: gate -> div -> mux.
	 */
	if (core->gp0_pll && target > 800000000) {
		clk_set_rate(core->gp0_pll, target);
		clk_prepare_enable(core->gp0_pll);

		struct clk *div = clk_get_parent(core->vdec_1_clk);
		struct clk *mux = div ? clk_get_parent(div) : NULL;

		if (mux)
			clk_set_parent(mux, core->gp0_pll);
	}

	clk_set_rate(core->vdec_1_clk, target);
	clk_prepare_enable(core->vdec_1_clk);
}

static int vdec_1_poweron(struct amvdec_core *core)
{
	int ret;
	bool g12a_plus = core->platform->revision >= VDEC_REVISION_G12A;
	bool pwr_hevc;
	bool hevc_warm_idle;
	u32 pwr_bits, iso_bits;

	vdec1_clk_enable(core);

	/*
	 * On G12A+, the MDEC DC cache hardware is inside the HEVC power
	 * domain. VDEC_1 needs HEVC clocks active for correct decode.
	 */
	if (core->platform->revision >= VDEC_REVISION_G12A) {
		ret = vdec_hevc_clk_get(core);
		if (ret) {
			clk_disable_unprepare(core->vdec_1_clk);
			return ret;
		}
	}

	/*
	 * KI-118 coupled unit: on G12A+ the HEVC power domain is shared (VDEC_1
	 * uses it for the MDEC DC cache).  Power it + DOS_SW_RESET3 ONLY if it is
	 * not already up.  If the HEVC engine is already powered (a HEVC decode
	 * running, or the HCODEC block-manager pin holds it), re-firing
	 * DOS_SW_RESET3 here would reset the live HEVC engine and clobber its
	 * decode -- so VDEC_1 reuses the already-powered shared domain instead.
	 * (hevc_powered is guarded by hw_lock, which the caller holds here.)
	 */
	pwr_hevc = g12a_plus && !core->hevc_powered;
	/* KI-121 pin: domain kept warm at a prior HEVC EOS, now reused by VDEC_1
	 * with the HEVC engine IDLE (no live decode / HCODEC pin). */
	hevc_warm_idle = g12a_plus && core->hevc_powered &&
			 core->hevc_power_users == 0;

	if (core->platform->revision == VDEC_REVISION_SM1) {
		pwr_bits = GEN_PWR_VDEC_1_SM1;
		iso_bits = GEN_PWR_ISO0_VDEC_1_SM1;
		if (pwr_hevc) {
			pwr_bits |= GEN_PWR_VDEC_HEVC_SM1;
			iso_bits |= GEN_PWR_ISO0_HEVC_SM1;
		}
	} else if (g12a_plus) {
		pwr_bits = GEN_PWR_VDEC_1;
		iso_bits = GEN_PWR_ISO0_VDEC_1;
		if (pwr_hevc) {
			pwr_bits |= GEN_PWR_VDEC_HEVC;
			iso_bits |= GEN_PWR_ISO0_HEVC;
		}
	} else {
		pwr_bits = GEN_PWR_VDEC_1;
		iso_bits = GEN_PWR_ISO0_VDEC_1;
	}

	/* Power on VDEC_1 (and the shared HEVC domain only when pwr_hevc) */
	regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_SLEEP0, pwr_bits, 0);
	usleep_range(10, 20);

	/*
	 * KI-121 (C2): a prior HEVC decode left HEVC_CORE_ENABLE (CE) cleared
	 * (its per-frame ANC2AXI commit clears it, KI-118 #16).  With the shared
	 * HEVC domain kept warm by the pin, CE still gates ARM access to the HEVC
	 * DOS range [0xc000,0xfd57] which holds the DOS_SW_RESET0 (0xfc00) below;
	 * re-assert CE=1 (0xfd58 is above the gated range, writable with CE=0)
	 * before the gated reset.  Idempotent for the normal case.
	 */
	if (g12a_plus)
		amvdec_write_dos(core, HEVC_CORE_ENABLE, 1);

	/* Reset VDEC1 */
	amvdec_write_dos(core, DOS_SW_RESET0, 0xfffffffc);
	amvdec_write_dos(core, DOS_SW_RESET0, 0x00000000);

	amvdec_write_dos(core, DOS_GCLK_EN0, DOS_GCLK_EN0_VDEC1);

	/* Enable VDEC Memories */
	amvdec_write_dos(core, DOS_MEM_PD_VDEC, 0);

	/*
	 * Reset the shared HEVC domain so VDEC_1's MDEC DC cache sees clean state.
	 * Two cases need it:
	 *   - pwr_hevc: we just cold-powered the domain.
	 *   - KI-121 pin: the domain was kept WARM at the prior HEVC decode's EOS
	 *     (hevc_powered, but IDLE -- hevc_power_users==0).  vdec_1_poweron used
	 *     to skip RESET3 whenever hevc_powered, assuming a LIVE HEVC engine that
	 *     RESET3 would clobber.  But a pinned-idle domain has no live decode, and
	 *     SKIPPING the reset leaves the prior HEVC decode's state in the shared
	 *     DOS block -> it reaches VDEC_1's MDEC DMA and stochastically wedges /
	 *     corrupts the bottom MB rows on SM1 (the same stale-HEVC-state-vs-MDEC
	 *     mechanism as the H.264 "rows 12-15" fix; SM1 is most sensitive because
	 *     its MDEC DC cache physically lives in the HEVC domain).  This was the
	 *     KI-121 SM1 residual the G12B pin never showed.
	 * SKIP only when a LIVE HEVC decode or HCODEC pin still holds the engine
	 * (hevc_power_users > 0) -- there RESET3 *would* clobber it, and that path
	 * (e.g. H.264-MMU) re-establishes the HEVC block itself.  CE was re-asserted
	 * above so DOS_SW_RESET3 (gated range) lands.
	 */
	if (pwr_hevc || hevc_warm_idle) {
		amvdec_write_dos(core, DOS_SW_RESET3, 0xffffffff);
		amvdec_write_dos(core, DOS_SW_RESET3, 0x00000000);
		amvdec_write_dos(core, DOS_GCLK_EN3, 0xffffffff);
		amvdec_write_dos(core, DOS_MEM_PD_HEVC, 0);
	}

	/*
	 * KI-121 SM1: on the warm-idle pin reuse, additionally clean the HEVC
	 * front-end DMC pipeline that the warm pin left dirty -- drain pending
	 * HEVC LMEM write-responses, then toggle the CHIP-LEVEL RESET7
	 * (hevcf_dmc_pipl_reset, RESET7 bit 13), mirroring vdec_hevc_power_on().
	 * The DOS_SW_RESET3 above is a DOS-block reset that does NOT reach the
	 * chip-level front-end; on SM1 (whose MDEC DC cache + HEVC front-end share
	 * the DMC path) the leftover HEVC front-end state stochastically hard-locks
	 * VDEC_1's MDEC DMA -- the same "soft-lock the kernel ... on SM1" the
	 * power_on RESET7 comment warns about.  ISO is already removed here (domain
	 * warm).  Cold pwr_hevc gets this via the normal vdec_hevc_power_on path.
	 */
	if (hevc_warm_idle) {
		int j = 1000;

		while ((amvdec_read_dos(core, HEVC_WRRSP_LMEM) &
			HEVC_WRRSP_LMEM_PENDING_MASK) && --j)
			udelay(1);
		if (core->hevcf_dmc_pipl_reset) {
			reset_control_assert(core->hevcf_dmc_pipl_reset);
			usleep_range(10, 20);
			reset_control_deassert(core->hevcf_dmc_pipl_reset);
			usleep_range(10, 20);
		}
	}

	/* Remove isolation (VDEC_1 always; HEVC only when we powered it) */
	regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_ISO0, iso_bits, 0);
	usleep_range(100, 200);

	/*
	 * N102: on G12A+ this function has just brought the SHARED HEVC domain
	 * up for VDEC_1's MDEC DC cache -- clocks via vdec_hevc_clk_get() above,
	 * and power+isolation either here (pwr_hevc) or already open because
	 * another holder had it.  Either way the HEVC DOS block is now
	 * MMIO-safe, and this is the path core->hevc_powered does NOT record:
	 * it is the KI-118 ownership flag and stays false for a VDEC_1 session.
	 * The caller (vdec_1_power_get / the poweroff work) holds hw_lock.
	 */
	if (g12a_plus)
		core->hevc_domain_on = true;

	return 0;
}

void vdec_1_poweroff(struct amvdec_core *core)
{
	/*
	 * Power-off order: clock off BEFORE isolation.
	 */
	if (core->platform->revision >= VDEC_REVISION_G12A) {
		vdec_hevc_clk_put(core);
		/*
		 * N102: this function never touches the HEVC domain's AO power or
		 * isolation -- it only drops the clock.  So the HEVC block stays
		 * MMIO-safe iff SOMEONE ELSE still holds a clock reference (a live
		 * HEVC decode, the HCODEC pin, or the diag bench); when the count
		 * reaches zero the block is unclocked and reading it would reset
		 * the SoC (KI-161).
		 */
		if (atomic_read(&core->hevc_clk_ref) == 0)
			core->hevc_domain_on = false;
	}
	clk_disable_unprepare(core->vdec_1_clk);

	/* Enable vdec1 isolation */
	if (core->platform->revision == VDEC_REVISION_SM1)
		regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_ISO0,
				   GEN_PWR_ISO0_VDEC_1_SM1,
				   GEN_PWR_ISO0_VDEC_1_SM1);
	else
		regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_ISO0,
				   GEN_PWR_ISO0_VDEC_1, GEN_PWR_ISO0_VDEC_1);

	/* Power off vdec1 memories */
	amvdec_write_dos(core, DOS_MEM_PD_VDEC, 0xffffffff);

	/* Power off vdec1 */
	if (core->platform->revision == VDEC_REVISION_SM1)
		regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_SLEEP0,
				   GEN_PWR_VDEC_1_SM1, GEN_PWR_VDEC_1_SM1);
	else
		regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_SLEEP0,
				   GEN_PWR_VDEC_1, GEN_PWR_VDEC_1);
}

static void vdec1_poweroff_work_fn(struct work_struct *work)
{
	struct amvdec_core *core = container_of(work, struct amvdec_core,
						vdec1_poweroff_work.work);

	bool woke = false;

	mutex_lock(&core->hw_lock);
	/* KI-118: only power off if no holder re-acquired during the idle delay */
	if (core->vdec1_power_users == 0 && core->vdec1_powered) {
		vdec_1_poweroff(core);
		core->vdec1_powered = false;
		core->vdec1_enc_owned = false;
		woke = true;
	}
	mutex_unlock(&core->hw_lock);
	if (woke)
		wake_up(&core->vdec1_dec_wq);
}

void vdec_1_power_init(struct amvdec_core *core)
{
	INIT_DELAYED_WORK(&core->vdec1_poweroff_work, vdec1_poweroff_work_fn);
	core->vdec1_powered = false;
	core->vdec1_power_users = 0;
	core->vdec1_enc_owned = false;
	init_waitqueue_head(&core->vdec1_dec_wq);
}

/*
 * vdec_1_power_get - ensure VDEC_1 is powered on.
 * Cancels any pending idle poweroff and powers on if needed.
 * Self-locking: caller must NOT hold hw_lock.
 *
 * @hcodec_pin: true when called by the HCODEC block-manager pin (KI-118),
 * false for a normal decode session.  On the cold power-on (vdec_1_poweron's
 * DOS-wide reset), this records WHO initialised the domain: if the encoder pin
 * powered it on (encode-first), the domain lacks the decode-appropriate reset
 * state, so a later decode must wait for a clean power-cycle (see vdec_1_start).
 */
int vdec_1_power_get(struct amvdec_core *core, bool hcodec_pin)
{
	cancel_delayed_work_sync(&core->vdec1_poweroff_work);
	mutex_lock(&core->hw_lock);
	/*
	 * N118: refuse to power a domain the device is on its way out of.
	 *
	 * The test is HERE, under hw_lock, and not only at the entry points,
	 * because the entry points' test is a TOCTOU: vdec_remove() can set
	 * core->unbound and reach its power-off block between a caller reading
	 * the flag and this function issuing vdec_1_poweron()'s DOS writes.
	 * With vdec_remove()'s power-off block now taking the same lock, the
	 * two orderings are the only ones: either this poweron completes first
	 * and remove() powers it back off, or remove() gets the lock first and
	 * this call sees unbound and issues no MMIO.  Same argument as
	 * 143e435ca9ae3 made for the debugfs readers -- the lock has to span
	 * the decision AND the access, not just sample the flag.
	 */
	if (READ_ONCE(core->unbound)) {
		mutex_unlock(&core->hw_lock);
		return -ENODEV;
	}
	if (!core->vdec1_powered) {
		int ret = vdec_1_poweron(core);

		if (ret) {
			mutex_unlock(&core->hw_lock);
			return ret;
		}
		core->vdec1_powered = true;
		/* KI-118: the cold power-on's owner determines whether a decode
		 * can run on this domain or must wait for a power-cycle. */
		core->vdec1_enc_owned = hcodec_pin;
	}
	core->vdec1_power_users++;	/* KI-118: refcount holders */
	mutex_unlock(&core->hw_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(vdec_1_power_get);

/*
 * vdec_1_power_put - schedule delayed VDEC_1 poweroff.
 * If vdec1_idle_ms is 0, powers off immediately under hw_lock.
 * Otherwise schedules delayed_work (which self-locks).
 * Caller must NOT hold hw_lock.
 */
void vdec_1_power_put(struct amvdec_core *core)
{
	bool sched_off = false;
	bool woke = false;

	/* KI-118: drop a refcount; only release power when the LAST holder puts
	 * (decode sessions + the HCODEC cross-engine pin share the domain).
	 */
	mutex_lock(&core->hw_lock);
	if (core->vdec1_power_users > 0)
		core->vdec1_power_users--;
	if (core->vdec1_power_users == 0) {
		/*
		 * N123: if the domain is already OFF, issue no MMIO.  This arm is
		 * reachable only after vdec_remove()'s power-off block has cleared
		 * vdec1_powered while a refcount was still outstanding -- every
		 * other holder's increment is paired with vdec1_powered being set.
		 * There, vdec_1_poweroff()'s DOS writes would land on a block whose
		 * clock remove() has just dropped, which is the N118/N107 external
		 * abort (Oops 96000047) this driver's unbind work exists to close.
		 * vdec1_poweroff_work_fn() already makes exactly this test; the two
		 * release paths now agree.  Still clear the ownership and wake, so
		 * nothing stays parked in vdec_1_start()'s wait.
		 */
		if (!core->vdec1_powered) {
			core->vdec1_enc_owned = false;
			woke = true;
		} else if (vdec1_idle_ms == 0 || core->vdec1_enc_owned) {
			vdec_1_poweroff(core);
			core->vdec1_powered = false;
			core->vdec1_enc_owned = false;
			woke = true;
		} else {
			sched_off = true;
		}
	}
	mutex_unlock(&core->hw_lock);
	if (woke)
		wake_up(&core->vdec1_dec_wq);
	if (sched_off)
		schedule_delayed_work(&core->vdec1_poweroff_work,
				      msecs_to_jiffies(vdec1_idle_ms));
}
EXPORT_SYMBOL_GPL(vdec_1_power_put);

/*
 * Halt the VDEC_1 AMRISC firmware and wait for the MDEC/DBLK/MC engines to go
 * idle (force-resetting an engine that stays busy past 10 ms), without the
 * DOS_SW_RESET0 or power-off that end a session.  The VDEC_1 counterpart of
 * amvdec_hevc_halt_firmware(): used by __vdec_1_stop at STREAMOFF and by codec
 * drain_finalize hooks, so no engine DMA is left running after the drain
 * while the session stays open.
 */
void amvdec_vdec1_halt_firmware(struct amvdec_core *core)
{
	unsigned long timeout;

	amvdec_write_dos(core, MPSR, 0);
	amvdec_write_dos(core, CPSR, 0);
	amvdec_write_dos(core, ASSIST_MBOX1_MASK, 0);

	/*
	 * Wait for MDEC, DBLK, and MC engines to idle before resetting.
	 * BSP (amvdec_pg_enable) polls each engine's status register and
	 * manually resets on timeout.  Without this, cutting power while
	 * engines are active can latch bad state that persists into the
	 * next power-on, causing ~30% of back-to-back decodes to fail
	 * with 0 output frames.
	 */

	/* Wait for MDEC PIC DC to idle */
	timeout = jiffies + msecs_to_jiffies(10);
	while (amvdec_read_dos(core, MDEC_PIC_DC_STATUS) != 0) {
		if (time_after(jiffies, timeout)) {
			/* Force reset MDEC PIC DC */
			amvdec_write_dos_bits(core, MDEC_PIC_DC_CTRL, BIT(0));
			amvdec_clear_dos_bits(core, MDEC_PIC_DC_CTRL, BIT(0));
			amvdec_read_dos(core, MDEC_PIC_DC_STATUS);
			break;
		}
		cpu_relax();
	}

	/* Wait for DBLK to idle */
	timeout = jiffies + msecs_to_jiffies(10);
	while (amvdec_read_dos(core, DBLK_STATUS) & DBLK_STATUS_BUSY) {
		if (time_after(jiffies, timeout)) {
			/* Force reset DBLK */
			amvdec_write_dos(core, DBLK_CTRL, DBLK_CTRL_RESET);
			amvdec_write_dos(core, DBLK_CTRL, 0);
			amvdec_read_dos(core, DBLK_STATUS);
			break;
		}
		cpu_relax();
	}

	/* Wait for MC to idle */
	timeout = jiffies + msecs_to_jiffies(10);
	while (amvdec_read_dos(core, MC_STATUS0) & MC_STATUS0_BUSY) {
		if (time_after(jiffies, timeout)) {
			/* Force reset MC */
			amvdec_write_dos_bits(core, MC_CTRL1, MC_CTRL1_RESET);
			amvdec_clear_dos_bits(core, MC_CTRL1, MC_CTRL1_RESET);
			amvdec_read_dos(core, MC_STATUS0);
			break;
		}
		cpu_relax();
	}
}
EXPORT_SYMBOL_GPL(amvdec_vdec1_halt_firmware);

/*
 * Resume the VDEC_1 AMRISC firmware from its halted PC after a drain_finalize
 * halt, for a session the client resumes (V4L2_DEC_CMD_START + new input)
 * without STREAMOFF.  Mirrors the tail of vdec_1_start(): MPSR=1, let the
 * firmware settle, clear any pending MBOX1 interrupt, then unmask it.  Does
 * not reload the firmware or re-run codec_ops->start (that would wipe the
 * session state).
 */
void amvdec_vdec1_restart_firmware(struct amvdec_core *core)
{
	amvdec_write_dos(core, MPSR, 1);
	usleep_range(10, 20);
	amvdec_write_dos(core, ASSIST_MBOX1_CLR_REG, 1);
	amvdec_write_dos(core, ASSIST_MBOX1_MASK, 1);
}
EXPORT_SYMBOL_GPL(amvdec_vdec1_restart_firmware);

static void __vdec_1_stop(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct amvdec_codec_ops *codec_ops = sess->fmt_out->codec_ops;

	amvdec_vdec1_halt_firmware(core);

	amvdec_write_dos(core, DOS_SW_RESET0, DOS_SW_RESET0_VDEC1);
	amvdec_write_dos(core, DOS_SW_RESET0, 0);
	amvdec_read_dos(core, DOS_SW_RESET0);

	if (sess->priv)
		codec_ops->stop(sess);

	/* Schedule deferred power-off via autosuspend */
	vdec_1_power_put(core);
}

static int vdec_1_stop(struct amvdec_session *sess)
{
	__vdec_1_stop(sess);
	return 0;
}

static int vdec_1_start(struct amvdec_session *sess)
{
	int ret;
	struct amvdec_core *core = sess->core;
	struct amvdec_codec_ops *codec_ops = sess->fmt_out->codec_ops;

	/*
	 * KI-118 queued-reset: if VDEC_1 is currently owned by the HCODEC pin
	 * (encode-first), it was cold-powered by the encoder and lacks the
	 * decode-appropriate power-on reset state -- and we cannot give it one
	 * now, because the full VDEC_1 power-on does a DOS-wide reset that would
	 * wedge the running HCODEC.  So block this decode until the encoder
	 * releases its pin and the domain power-cycles (vdec_1_power_put forces
	 * an immediate power-off for an encoder-owned domain, then wakes us).
	 * The decode then cold-powers VDEC_1 itself and is decode-owned.
	 *
	 * This serializes a decode that ARRIVES mid-encode behind the encode
	 * session -- matching d1388cfe's safety for that specific case -- while
	 * leaving decode-first transcodes, decode restarts on a decode-owned
	 * domain, and steady-state concurrency fully parallel.
	 */
	if (ki118_block_manager) {
		ret = wait_event_interruptible(core->vdec1_dec_wq,
					       !READ_ONCE(core->vdec1_enc_owned));
		if (ret)
			return ret;	/* -ERESTARTSYS: let userspace retry */
	}

	/*
	 * Power on VDEC_1 domain via autosuspend-aware API.
	 * vdec_1_power_get() cancels any pending idle poweroff and
	 * powers on if needed.  Uses vdec1_target_rate() for the clock
	 * so turbo level is applied at poweron time.
	 */
	ret = vdec_1_power_get(core, false);
	if (ret)
		return ret;

	/*
	 * KI-121 (C2): re-assert HEVC_CORE_ENABLE (CE) on G12A+ before the gated
	 * DOS resets below.  A prior HEVC decode left CE=0; vdec_1_poweron sets it
	 * on the COLD path, but power_get skips poweron when VDEC_1 is already
	 * powered (warm) -- so a warm MPEG-2/VDEC_1 decode after a HEVC decode
	 * would stall here without this.  0xfd58 is above the gated range so it is
	 * writable with CE=0; idempotent otherwise.
	 */
	if (core->platform->revision >= VDEC_REVISION_G12A)
		amvdec_write_dos(core, HEVC_CORE_ENABLE, 1);

	/* Reset DOS top registers */
	amvdec_write_dos(core, DOS_VDEC_MCRCC_STALL_CTRL, 0);

	amvdec_write_dos(core, GCLK_EN, DOS_GCLK_EN0_VDEC1);
	amvdec_clear_dos_bits(core, MDEC_PIC_DC_CTRL, MDEC_PIC_DC_CTRL_OLD_MODE);

	/*
	 * Clear IMEM and MDEC state before firmware load.
	 *
	 * BIT(4) = VLD reset, which clears IMEM contents.  Without this,
	 * firmware from a previous codec session persists in IMEM through
	 * the power cycle (DOS_MEM_PD_VDEC off/on above does not reliably
	 * zero SRAM on all SoCs).  Stale firmware causes codec cross-
	 * contamination: e.g., MPEG12 firmware executing during H.264
	 * decode when sessions run back-to-back without reboot.
	 *
	 * On G12A+, also reset MDEC subsystem (BIT(7)|BIT(6)) to clear
	 * DC prediction pipeline stale state (reduces +42 Y-plane offset).
	 *
	 * This must happen BEFORE firmware load -- the DMA load that
	 * follows immediately overwrites the cleared IMEM.
	 */
	if (core->platform->revision >= VDEC_REVISION_G12A) {
		amvdec_write_dos(core, DOS_SW_RESET0,
				 DOS_SW_RESET0_MDEC | DOS_SW_RESET0_VLD);
		amvdec_write_dos(core, DOS_SW_RESET0, 0);
		amvdec_read_dos(core, DOS_SW_RESET0);
		amvdec_read_dos(core, DOS_SW_RESET0);
		amvdec_read_dos(core, DOS_SW_RESET0);
		amvdec_write_dos(core, MDEC_SW_RESET, MDEC_SW_RESET_MDEC_CORE);
		amvdec_write_dos(core, MDEC_SW_RESET, 0);
	} else {
		/* GXL/GXM: clear VLD/IMEM only (no MDEC reset needed) */
		amvdec_write_dos(core, DOS_SW_RESET0, DOS_SW_RESET0_VLD);
		amvdec_write_dos(core, DOS_SW_RESET0, 0);
		amvdec_read_dos(core, DOS_SW_RESET0);
	}

	/* Disable PSCALE -- MJPEG enables it (PSCALE_CTRL=0xc000) and
	 * leaves it active after stop.  Other codecs (MPEG1/2, MPEG4)
	 * disable it in their start(), but H.264 does not.  Clear it
	 * here so every codec starts with PSCALE disabled regardless
	 * of what the previous codec left behind.
	 */
	amvdec_write_dos(core, PSCALE_CTRL, 0);

	/*
	 * Fill canvas registers with scratch buffer before firmware starts.
	 * Stateless (libre) sessions read canvas IDs from the LMEM parameter
	 * block instead of the AV_SCRATCH_0..7 pre-fill, so skip this for them
	 * (architecture.md §8d1). amvdec_set_canvases (called from the codec's
	 * start) still programs the canvas hardware + the scratch values the
	 * driver reads back to pack into LMEM.
	 */
	if (!amvdec_is_stateless(sess))
		amvdec_fill_scratch_canvases(core);

	vdec_1_stbuf_power_up(sess);

	ret = vdec_1_load_firmware(sess, sess->fmt_out->firmware_path);
	if (ret)
		goto stop;

	/*
	 * Defense in depth: per-codec modules (meson-vdec-h264, -mpeg12, ...)
	 * register codec_ops via amvdec_register_codec().  If userspace
	 * opens /dev/video0 before the per-codec module loaded -- e.g.
	 * MODULE_SOFTDEP failed, initramfs missing it, manual rmmod --
	 * codec_ops is NULL and dereferencing it would NULL-deref the
	 * kernel.  Fail clean instead.
	 */
	if (!codec_ops || !codec_ops->start) {
		dev_err(core->dev,
			"codec_ops not registered for fmt %p4cc -- per-codec module not loaded?\n",
			&sess->fmt_out->pixfmt);
		ret = -ENODEV;
		goto stop;
	}

	ret = codec_ops->start(sess);
	if (ret)
		goto stop;

	/* Enable 2-plane output and configure chroma order.
	 * Bit 17: 2-plane enable (NV12/NV21 mode)
	 * Bit 16: cbcr_merge_swap_en (1=NV21/VU, 0=NV12/UV)
	 */
	switch (sess->pixfmt_cap) {
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV12M:
		amvdec_write_dos_bits(core, MDEC_PIC_DC_CTRL,
				      MDEC_PIC_DC_CTRL_NV12);
		amvdec_clear_dos_bits(core, MDEC_PIC_DC_CTRL,
				      MDEC_PIC_DC_CTRL_CBCR_SWAP);
		break;
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_NV21M:
		amvdec_write_dos_bits(core, MDEC_PIC_DC_CTRL,
				      MDEC_PIC_DC_CTRL_NV12 |
				      MDEC_PIC_DC_CTRL_CBCR_SWAP);
		break;
	default:
		amvdec_clear_dos_bits(core, MDEC_PIC_DC_CTRL,
				      MDEC_PIC_DC_CTRL_NV12 |
				      MDEC_PIC_DC_CTRL_CBCR_SWAP);
		break;
	}

	if (codec_ops->early_irq_enable) {
		/*
		 * Enable MBOX1 BEFORE starting firmware for codecs that decode
		 * fast enough to fire their first-frame IRQ before the host can
		 * enable interrupts via the normal MPSR->CLR->MASK sequence.
		 *
		 * MJPEG firmware: after MPSR=1 the firmware immediately enters
		 * its main loop and can decode frame 0 within ~15-60 us.  With
		 * the normal sequence (MPSR -> usleep -> CLR -> MASK) the first
		 * MBOX1 interrupt is reliably lost, dropping 1 frame per session.
		 *
		 * Safety: codec_mjpeg_start() clears MREG_FROM_AMRISC = 0.
		 * Any stale MBOX1 pending from a prior session fires harmlessly
		 * (ISR sees reg=0 and returns without delivering a frame).
		 * MJPEG firmware does NOT fire spurious init IRQs; it busy-waits
		 * for MREG_TO_AMRISC != 0 before doing any decode.
		 *
		 * MPEG12 must NOT use this path: it fires MCPU_INTR_REQ=4 at
		 * firmware address 0x052f during init when VIFIFO has data,
		 * causing spurious MBOX1 delivery with stale AV_SCRATCH_9.
		 */
		amvdec_write_dos(core, ASSIST_MBOX1_MASK, 1);
		amvdec_write_dos(core, MPSR, 1);
		usleep_range(10, 20);
	} else {
		/* Enable firmware processor */
		amvdec_write_dos(core, MPSR, 1);
		/* Let the firmware settle */
		usleep_range(10, 20);

		/*
		 * Enable IRQ after firmware has initialized.  The firmware's
		 * init sequence (setting up interrupt routing, canvas addresses,
		 * etc.) can generate a spurious MBOX1 interrupt.  If the IRQ is
		 * enabled before the firmware settles, the ISR reads stale
		 * BUFFEROUT/FRAME_OFFSET values and delivers a bogus frame,
		 * consuming a timestamp entry.  This shifts all subsequent
		 * frame-to-timestamp matching, causing ~22dB PSNR drops on
		 * ~9% of MPEG1 decodes (and potentially other VDEC_1 codecs).
		 *
		 * Clear any pending interrupt first, then enable.
		 */
		amvdec_write_dos(core, ASSIST_MBOX1_CLR_REG, 1);
		amvdec_write_dos(core, ASSIST_MBOX1_MASK, 1);
	}

	/* Call codec post_start after firmware is running and IRQ is enabled.
	 * Used by MJPEG: the firmware waits for MREG_TO_AMRISC to be
	 * non-zero before decoding the first frame.  This must happen
	 * after MPSR=1 because the firmware clears MREG_TO_AMRISC during
	 * its init sequence (address 0x0009 in gxl_mjpeg.bin).
	 */
	if (codec_ops->post_start)
		codec_ops->post_start(sess);

	return 0;

stop:
	__vdec_1_stop(sess);
	return ret;
}

void vdec_1_thermal_update(struct amvdec_core *core, unsigned long rate)
{
	/*
	 * Only set VDEC_1's own clock. The HEVC clocks that VDEC_1
	 * uses on G12A+ (for MDEC DC cache) are managed by
	 * vdec_hevc_thermal_update() -- no double-set needed.
	 */
	clk_set_rate(core->vdec_1_clk, rate);
}

struct amvdec_ops vdec_1_ops = {
	.start = vdec_1_start,
	.stop = vdec_1_stop,
	.conf_esparser = vdec_1_conf_esparser,
	.vififo_level = vdec_1_vififo_level,
};
