// SPDX-License-Identifier: GPL-2.0+
/*
 * Amlogic Meson HCODEC H.264 Encoder - Hardware Operations
 *
 * Based on Amlogic BSP encoder driver.
 * Copyright (C) 2024 LibreTech
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/math.h>
#include <linux/module.h>
#include <linux/sizes.h>
#include <linux/regmap.h>
#include "vdec.h"
#include "vdec_helpers.h"
#include "vdec_1.h"		/* KI-118 block manager: vdec_1_power_get/put */
#include "vdec_hevc.h"		/* KI-118 block manager: vdec_hevc_power_get/put */
#include "dos_regs.h"
#include "hcodec.h"
#include "hcodec_regs.h"
#include "codec_hcodec_fw.h"

/* CBR firmware constants */
#define CBR_START_TBL_ID	8
#define CBR_SHORT_SHIFT		12	/* disabled (shift too large) */
#define CBR_LONG_MB_NUM		2
#define CBR_LONG_TH		4
#define CBR_TARGET_BITS_TBL_SZ	256
#define CBR_TABLE_ENTRY_SIZE	128	/* bytes per QP table entry */
#define CBR_MAGIC_0		0x55aa
#define CBR_MAGIC_1		0xaa55

/*
 * GXL (CAVLC) even-MB-width minimum QP.  The gxl_h264_enc.bin firmware
 * hard-wedges the VLC/QDCT/MC pipeline mid-frame for EVEN frame-width-in-MBs
 * when the per-MB bit density is high (i.e. low QP): A/B-proven on
 * aml-s905x-cc-v2 that even mb_w encodes cleanly at high QP (>= ~26) and
 * wedges deterministically at low QP (<= ~16); odd mb_w and G12A+ CABAC are
 * far more tolerant.  Clamp the QP floor for this case so the encoder stays in
 * its working range (rate control still adapts above the floor).  Companion to
 * the per-block CBR-target zeroing in hcodec_fill_cbr_table().  See
 * encoder/hcodec/todo.md "GXL H.264 encode status=0x5".
 */
#define HCODEC_GXL_EVEN_QP_FLOOR	26

/* True on GXL (CAVLC firmware, revision < G12A) with an even MB width. */
static inline bool hcodec_gxl_even_mbw(struct amvenc_ctx *ctx)
{
	return ctx->core->platform->revision < VDEC_REVISION_G12A &&
	       !(((ctx->width + 15) / 16) & 1);
}

/* AO power bits */
#define AO_RTI_PWR_CNTL_REG0	0x10
#define AO_RTI_GEN_PWR_SLEEP0	0xe8
#define AO_RTI_GEN_PWR_ISO0	0xec
#define GEN_PWR_HCODEC		(BIT(1) | BIT(0))
#define GEN_PWR_HCODEC_SM1	BIT(0)
#define GEN_PWR_HCODEC_ISO	0x30
#define GEN_PWR_HCODEC_ISO_SM1	BIT(0)

/* HHI clock register for direct programming fallback */
#define HHI_VDEC_CLK_CNTL	0x1E0
/* HCODEC clock in upper half: mux[27:25], gate[24], div[22:16] */
#define HCODEC_CLK_MUX_FCLK_DIV3	1
#define HCODEC_CLK_DIV_1		0
#define HCODEC_CLK_GATE			BIT(24)
#define HCODEC_CLK_VAL		(HCODEC_CLK_GATE | \
				 (HCODEC_CLK_MUX_FCLK_DIV3 << 25) | \
				 (HCODEC_CLK_DIV_1 << 16))

/* Reset bits for DOS_SW_RESET1 */
#define HCODEC_RESET_FULL	(BIT(2) | BIT(6) | BIT(7) | BIT(8) | \
				 BIT(14) | BIT(16) | BIT(17))
#define HCODEC_RESET_MC		(BIT(12) | BIT(11))

/* Firmware names */
static const char * const fw_h264_names[] = {
	[VDEC_REVISION_GXL]  = "meson/vdec/gxl_h264_enc.bin",
	[VDEC_REVISION_G12A] = "meson/vdec/ga_h264_enc_cabac.bin",
	[VDEC_REVISION_G12B] = "meson/vdec/ga_h264_enc_cabac.bin",
	[VDEC_REVISION_SM1]  = "meson/vdec/ga_h264_enc_cabac.bin",
};

/* ME MV SAD table (V3 improvements) */
static const u32 v3_mv_sad[64] = {
	0x00000004, 0x00010008, 0x00020010, 0x00030018,
	0x00040020, 0x00050028, 0x00060038, 0x00070048,
	0x00080058, 0x00090068, 0x000a0080, 0x000b0098,
	0x000c00b0, 0x000d00c8, 0x000e00e8, 0x000f0110,
	0x00100002, 0x00110004, 0x00120008, 0x0013000c,
	0x00140010, 0x00150014, 0x0016001c, 0x00170024,
	0x0018002c, 0x00190034, 0x001a0044, 0x001b0054,
	0x001c0064, 0x001d0074, 0x001e0094, 0x001f00b4,
	0x00200006, 0x0021000c, 0x0022000c, 0x00230018,
	0x00240018, 0x00250018, 0x00260018, 0x00270030,
	0x00280030, 0x00290030, 0x002a0030, 0x002b0030,
	0x002c0030, 0x002d0030, 0x002e0030, 0x002f0050,
	0x00300001, 0x00310002, 0x00320002, 0x00330004,
	0x00340004, 0x00350004, 0x00360004, 0x00370006,
	0x00380006, 0x00390006, 0x003a0006, 0x003b0006,
	0x003c0006, 0x003d0006, 0x003e0006, 0x003f0006,
};

/* QP to chroma QP mapping table (H.264 spec Table 8-15) */
static const u8 qp_to_qpc[] = {
	 0,  1,  2,  3,  4,  5,  6,  7,  8,  9,
	10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
	20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
	29, 30, 31, 32, 32, 33, 34, 34, 35, 35,
	36, 36, 37, 37, 37, 38, 38, 38, 39, 39,
	39,
};

/*
 * HCODEC encoder turbo clock level (0-2).
 *
 *   0: 800 MHz -- default, matches VDEC_1/HEVC thermal state 0
 *   1: 1000 MHz -- +4% fps gain (requires gp0_pll on G12B+)
 *   2: 1200 MHz -- +7% fps gain, max stable (requires gp0_pll on G12B+)
 *
 * Levels 1-2 are silently ignored on GXL (no gp0_pll); the encoder falls
 * back to the thermal-state rate.  On G12A/G12B/SM1 gp0_pll is available
 * and the target rate is applied on the next hcodec_poweron().
 *
 * Note: turbo only affects HCODEC (encoder).  VDEC_1 and VDEC_HEVC
 * (decoders) remain at the thermal cooling rate.
 */
static unsigned int hcodec_turbo;
module_param(hcodec_turbo, uint, 0644);
MODULE_PARM_DESC(hcodec_turbo,
		 "HCODEC encoder turbo: 0=800MHz 1=1000MHz 2=1200MHz (default=0)");

/*
 * Auto-clock gate settle delay in microseconds.
 * BSP uses 10000 (10ms). Theoretical minimum is ~50us.
 * Settable via module param for runtime probing without rebuild.
 */
/*
 * Auto-clock gate settle delay (microseconds).
 * BSP uses mdelay(10) = 10,000us. Empirically tested: 0us works on
 * both G12B 12nm and GXL 28nm (probed via this param at runtime).
 * Keep at 0; expose param for field debugging if a future SoC needs it.
 */
static unsigned int hcodec_settle_us;
module_param(hcodec_settle_us, uint, 0644);
MODULE_PARM_DESC(hcodec_settle_us,
		 "HCODEC auto-clock gate settle delay in us (default=0, BSP=10000)");

/*
 * Per-level clock targets for hcodec_turbo.
 * Index 0 must match vdec_cooling_rates[0] (800 MHz) so that turbo=0
 * leaves the encoder at the same default rate as the decoders.
 */
static const unsigned long hcodec_turbo_rates[] = {
	800000000,	/* 0: default */
	1000000000,	/* 1: +4% fps */
	1200000000,	/* 2: +7% fps, max stable */
};

#define HCODEC_TURBO_MAX (ARRAY_SIZE(hcodec_turbo_rates) - 1)

/*
 * hcodec_target_rate - effective HCODEC clock rate
 *
 * Returns turbo_rate when not thermally throttled (state 0), otherwise
 * returns the thermal rate.  Thermal ALWAYS wins over turbo for safety.
 */
static unsigned long hcodec_target_rate(struct amvdec_core *core)
{
	unsigned long thermal_rate = amvdec_clock_rate(core);
	unsigned int level = min_t(unsigned int, hcodec_turbo, HCODEC_TURBO_MAX);
	unsigned long turbo_rate = hcodec_turbo_rates[level];

	/*
	 * Thermal always wins: if throttled (thermal < 800 MHz),
	 * use thermal rate regardless of turbo setting.
	 * Turbo only applies when thermal is at state 0 (max).
	 */
	if (thermal_rate < 800000000)
		return thermal_rate;
	return turbo_rate;
}

/*
 * hcodec_thermal_update - apply a new thermal cooling rate to HCODEC
 *
 * Called from vdec_cooling_set_cur_state() when the thermal framework
 * changes the cooling state.  Caps the encoder clock to @rate if it
 * is currently above @rate; otherwise the turbo rate is preserved.
 *
 * Using max() here means a turbo > thermal_rate session keeps running
 * at turbo until the next poweron -- consistent with how the decoders
 * handle in-session clock transitions (no IP pause needed, confirmed
 * 2026-04-10).
 */
void hcodec_thermal_update(struct amvdec_core *core, unsigned long rate)
{
	if (core->hcodec_clk)
		clk_set_rate(core->hcodec_clk, rate);
}

static void hcodec_clk_enable(struct amvdec_core *core)
{
	if (core->hcodec_clk) {
		unsigned long target = hcodec_target_rate(core);

		/*
		 * If gp0_pll is available and target > fclk_div2p5 (800 MHz),
		 * switch the mux parent to gp0_pll so the PLL can be
		 * programmed to the exact rate. Without clk_set_parent the
		 * mux stays on fclk_div2p5 and clk_set_rate silently caps
		 * at 800 MHz (the mux lacks CLK_SET_RATE_PARENT).
		 */
		/*
		 * clk_set_parent must target the MUX (vdec_hcodec_sel),
		 * not the gate (vdec_hcodec). The chain is:
		 *   gate -> div -> mux -> {fclk_div2p5, gp0_pll, ...}
		 * clk_get_parent twice to reach the mux.
		 */
		if (core->gp0_pll && target > 800000000) {
			/*
			 * gp0_pll must be enabled at the target rate BEFORE
			 * switching the mux. Otherwise CLK_MUX_ROUND_CLOSEST
			 * sees gp0_pll at 0 Hz and picks fclk_div2p5 instead.
			 */
			clk_set_rate(core->gp0_pll, target);
			clk_prepare_enable(core->gp0_pll);

			struct clk *div = clk_get_parent(core->hcodec_clk);
			struct clk *mux = div ? clk_get_parent(div) : NULL;

			if (mux)
				clk_set_parent(mux, core->gp0_pll);
		}

		clk_set_rate(core->hcodec_clk, target);
		clk_prepare_enable(core->hcodec_clk);
		dev_info(core->dev, "HCODEC: clk=%lu target=%lu gp0=%lu\n",
			 clk_get_rate(core->hcodec_clk), target,
			 core->gp0_pll ? clk_get_rate(core->gp0_pll) : 0);
	} else if (core->regmap_hhi) {
		/* Direct HHI programming: fclk_div3 (~667MHz), div=1, gate=on */
		regmap_update_bits(core->regmap_hhi, HHI_VDEC_CLK_CNTL,
				   HHI_VDEC_CLK_HCODEC_MASK, HCODEC_CLK_VAL);
	}
}

static void hcodec_clk_disable(struct amvdec_core *core)
{
	if (core->hcodec_clk) {
		clk_disable_unprepare(core->hcodec_clk);
	} else if (core->regmap_hhi) {
		regmap_update_bits(core->regmap_hhi, HHI_VDEC_CLK_CNTL,
				   HCODEC_CLK_GATE, 0);
	}
}

int hcodec_poweron(struct amvdec_core *core)
{
	u32 pwr_bits, iso_bits;

	/* DOS bus clock must be enabled first */
	clk_prepare_enable(core->dos_clk);

	if (core->platform->revision >= VDEC_REVISION_SM1) {
		pwr_bits = GEN_PWR_HCODEC_SM1;
		iso_bits = GEN_PWR_HCODEC_ISO_SM1;
	} else {
		pwr_bits = GEN_PWR_HCODEC;
		iso_bits = GEN_PWR_HCODEC_ISO;
	}

	/* Clear power control bits (BSP: AO_RTI_PWR_CNTL_REG0 &= ~0x18) */
	regmap_update_bits(core->regmap_ao, AO_RTI_PWR_CNTL_REG0,
			   AO_PWR_CNTL_HCODEC_BITS, 0);
	usleep_range(10, 20);

	/* Power on HCODEC domain */
	regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_SLEEP0,
			   pwr_bits, 0);
	usleep_range(10, 20);

	/* Reset HCODEC */
	amvdec_write_dos(core, DOS_SW_RESET1, 0xffffffff);
	amvdec_write_dos(core, DOS_SW_RESET1, 0);

	/* Enable HCODEC clock (BSP: after power domain + reset) */
	hcodec_clk_enable(core);

	/* Enable internal clock gates */
	amvdec_write_dos(core, DOS_GCLK_EN0,
			 amvdec_read_dos(core, DOS_GCLK_EN0) | GCLK_EN0_HCODEC_MASK);

	/* Power on HCODEC memories */
	amvdec_write_dos(core, DOS_MEM_PD_HCODEC, 0);

	/* Remove isolation */
	regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_ISO0,
			   iso_bits, 0);
	usleep_range(10, 20);

	/*
	 * Toggle auto-clock gate off then on.  The BSP uses mdelay(10)
	 * after this toggle, but the actual settle time is the clock
	 * gating FSM re-sync (~10 clock cycles at 200 MHz = 50ns) plus
	 * SRAM power stabilization (~10us for 12nm, ~20us for 28nm).
	 * Use 50us (measured minimum on both GXL 28nm and G12B 12nm)
	 * with 100% safety buffer = 100us.
	 */
	amvdec_write_dos(core, DOS_GEN_CTRL0,
			 amvdec_read_dos(core, DOS_GEN_CTRL0) | 1);
	amvdec_write_dos(core, DOS_GEN_CTRL0,
			 amvdec_read_dos(core, DOS_GEN_CTRL0) & ~1);

	if (hcodec_settle_us)
		usleep_range(hcodec_settle_us, hcodec_settle_us * 2);

	return 0;
}

void hcodec_poweroff(struct amvdec_core *core)
{
	u32 pwr_bits, iso_bits;

	if (core->platform->revision >= VDEC_REVISION_SM1) {
		pwr_bits = GEN_PWR_HCODEC_SM1;
		iso_bits = GEN_PWR_HCODEC_ISO_SM1;
	} else {
		pwr_bits = GEN_PWR_HCODEC;
		iso_bits = GEN_PWR_HCODEC_ISO;
	}

	/*
	 * Power-off order (matches BSP avc_poweroff):
	 *   1. Disable HCODEC clock (stop toggling before isolation)
	 *   2. Assert isolation (prevent glitches on power domain boundary)
	 *   3. Power down memories
	 *   4. Put domain to sleep
	 *   5. Disable DOS bus clock
	 */
	hcodec_clk_disable(core);
	regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_ISO0,
			   iso_bits, iso_bits);
	amvdec_write_dos(core, DOS_MEM_PD_HCODEC, 0xffffffff);
	regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_SLEEP0,
			   pwr_bits, pwr_bits);
	clk_disable_unprepare(core->dos_clk);
}

/*
 * HCODEC idle autosuspend.
 *
 * Instead of powering off immediately at STREAMOFF, schedule a delayed
 * poweroff. If another STREAMON arrives before the delay expires, cancel
 * the work and skip the power cycle entirely. This eliminates the full
 * poweron/poweroff overhead (~100us + firmware load) for back-to-back
 * encode sessions (e.g., camera burst capture: many 1-frame JPEG sessions).
 *
 * Default idle timeout: 100ms. Settable via module param for tuning.
 */
static unsigned int hcodec_idle_ms = 100;
module_param(hcodec_idle_ms, uint, 0644);
MODULE_PARM_DESC(hcodec_idle_ms,
		 "HCODEC idle poweroff delay in ms (default=100, 0=immediate)");

/*
 * KI-118 block manager: while HCODEC is powered, PIN the VDEC_1 decode domain
 * powered too.  The decode engine's power-on does a DOS-wide reset that wedges a
 * running HCODEC, so VDEC_1 must power on BEFORE HCODEC (while it is off) and must
 * never power-cycle while HCODEC stays up.  Pinning ties VDEC_1's power to HCODEC's
 * physical-power episode (refcounted in vdec_1_power_get/put).  Default on; set 0
 * to A/B (off = the bare revert that wedges a concurrent transcode).
 */
bool ki118_block_manager = true;	/* non-static: vdec_1.c reads it for the decode-side wait (same meson-vdec-core module) */
module_param(ki118_block_manager, bool, 0644);
MODULE_PARM_DESC(ki118_block_manager,
		 "KI-118: pin VDEC_1 powered while HCODEC is active, decode-engine first (default on)");

/*
 * KI-118 block manager: HCODEC pins the decode-engine power domains powered for
 * its whole physical-power episode, so a decode starting mid-encode never does
 * the destructive cold power-on (DOS-wide reset) that would wedge HCODEC.  The
 * engines are powered decode-first (BEFORE hcodec_poweron, while HCODEC is OFF)
 * and released in the poweroff paths.  Caller must NOT hold hw_lock
 * (vdec_*_power_get self-lock).
 */
/*
 * N123: release the decode-engine pins WITHOUT touching HCODEC power.
 *
 * hcodec_poweroff_and_unpin() cannot do this job: its whole body is gated on
 * core->hcodec_powered, and every path that needs an unpin here is one where
 * HCODEC never powered on, so that flag is still false and the pins would be
 * held forever.  Same context rules as hcodec_pin_decode_engines(): the caller
 * must NOT hold hw_lock (the two puts self-lock), and hcodec_power_get()'s
 * leading cancel_delayed_work_sync() means the poweroff work is not running,
 * so the hcodec_holds_* flags are stable outside the lock.
 *
 * HEVC first, VDEC_1 second -- the reverse of the pin order, and the same
 * order hcodec_poweroff_and_unpin() releases them in.
 */
static void hcodec_unpin_decode_engines(struct amvdec_core *core)
{
	if (core->hcodec_holds_hevc) {
		core->hcodec_holds_hevc = false;
		vdec_hevc_power_put(core);
	}
	if (core->hcodec_holds_vdec1) {
		core->hcodec_holds_vdec1 = false;
		vdec_1_power_put(core);
	}
}

static int hcodec_pin_decode_engines(struct amvdec_core *core)
{
	int ret;

	if (!core->hcodec_holds_vdec1) {
		ret = vdec_1_power_get(core, true);	/* HCODEC still off -> safe */
		if (ret)
			return ret;
		core->hcodec_holds_vdec1 = true;
	}
	/*
	 * The HEVC pin is G12A+ ONLY: the VDEC_1<->HEVC shared-power /
	 * DOS_SW_RESET3 coupling that makes it necessary is G12A+.  On GXL the
	 * engines are separate; pinning HEVC there does a spurious cold
	 * vdec_hevc_power_on whose RESET3 wedges a concurrent VDEC_1 decode +
	 * HCODEC encode (and KI-118 was never a GXL hazard) -- GXL instead
	 * serializes an HEVC decode on hcodec_powered (see __vdec_hevc_start).
	 * Wave420L needs no pin (separate IP, verified DOS-independent).
	 */
	if (core->platform->revision >= VDEC_REVISION_G12A &&
	    !core->hcodec_holds_hevc) {
		ret = vdec_hevc_power_get(core, true);	/* HCODEC still off -> safe */
		if (ret) {
			/*
			 * N123: unwind the VDEC_1 pin this call just took.  Without
			 * it a partial pin leaks a vdec1_power_users refcount on a
			 * LIVE device -- vdec_hevc_power_get() fails here for two
			 * reasons, and only one of them is the unbind: the other is
			 * vdec_hevc_power_on() -> vdec_hevc_clk_get() returning a
			 * clock error, which leaves the device bound and running.
			 * A leaked pin took the domain with hcodec_pin=true, so
			 * vdec1_enc_owned stays set and vdec_1_start()'s
			 * wait_event_interruptible(!vdec1_enc_owned) -- which has no
			 * timeout -- never completes for ANY later VDEC_1 decode.
			 */
			hcodec_unpin_decode_engines(core);
			return ret;
		}
		core->hcodec_holds_hevc = true;
	}
	return 0;
}

/*
 * KI-118: power HCODEC off and release the decode-engine pins.  Shared by the
 * idle-autosuspend work and the immediate (hcodec_idle_ms==0) put.  Clears the
 * pin flags + powers off under hw_lock, drops the engine refcounts outside it
 * (they self-lock), and wakes any HEVC decode serialized on the encoder (the
 * GXL no-pin path; harmless on G12A+ where the pin release already woke it).
 */
static void hcodec_poweroff_and_unpin(struct amvdec_core *core)
{
	bool unpin = false, unpin_hevc = false;

	mutex_lock(&core->hw_lock);
	if (core->hcodec_powered) {
		hcodec_poweroff(core);
		core->hcodec_powered = false;
		if (core->hcodec_holds_vdec1) {
			core->hcodec_holds_vdec1 = false;
			unpin = true;
		}
		if (core->hcodec_holds_hevc) {
			core->hcodec_holds_hevc = false;
			unpin_hevc = true;
		}
	}
	mutex_unlock(&core->hw_lock);
	if (unpin)
		vdec_1_power_put(core);
	if (unpin_hevc)
		vdec_hevc_power_put(core);
	wake_up(&core->hevc_dec_wq);
}

static void hcodec_poweroff_work_fn(struct work_struct *work)
{
	struct amvdec_core *core = container_of(work, struct amvdec_core,
						hcodec_poweroff_work.work);

	hcodec_poweroff_and_unpin(core);
}

void hcodec_power_init(struct amvdec_core *core)
{
	INIT_DELAYED_WORK(&core->hcodec_poweroff_work, hcodec_poweroff_work_fn);
	core->hcodec_powered = false;
	core->hcodec_holds_vdec1 = false;
	core->hcodec_holds_hevc = false;
}

/*
 * hcodec_power_get - ensure HCODEC is powered on.
 * Cancels any pending idle poweroff and powers on if needed.
 *
 * cancel_delayed_work_sync is called BEFORE acquiring hw_lock to
 * avoid ABBA deadlock: the work function acquires hw_lock, so
 * cancel_delayed_work_sync under hw_lock would deadlock if the
 * work is running.  The caller must NOT hold hw_lock when calling.
 */
int hcodec_power_get(struct amvdec_core *core)
{
	cancel_delayed_work_sync(&core->hcodec_poweroff_work);

	/*
	 * KI-118: pin the decode engines powered BEFORE HCODEC powers on, so their
	 * DOS-wide power-on reset happens while HCODEC is OFF and never again while
	 * HCODEC stays up.  The preceding cancel_delayed_work_sync ensures the
	 * poweroff work (which clears the flags) is not running, so they are stable.
	 */
	if (ki118_block_manager) {
		int ret = hcodec_pin_decode_engines(core);

		if (ret)
			return ret;
	}

	mutex_lock(&core->hw_lock);
	/* N118: same refusal, same reason, as vdec_1_power_get(). */
	if (READ_ONCE(core->unbound)) {
		mutex_unlock(&core->hw_lock);
		/*
		 * N123: the pins were taken above, BEFORE this test -- the order
		 * is load-bearing (the engines must power on while HCODEC is
		 * off), so the test cannot simply move up, and moving it would
		 * be a TOCTOU besides.  Release them on the way out instead;
		 * nothing else will, because hcodec_poweroff_and_unpin() is
		 * gated on hcodec_powered, which this refusal leaves false.
		 * Outside hw_lock: the puts self-lock.
		 */
		hcodec_unpin_decode_engines(core);
		return -ENODEV;
	}
	if (!core->hcodec_powered) {
		int ret = hcodec_poweron(core);

		if (ret) {
			mutex_unlock(&core->hw_lock);
			hcodec_unpin_decode_engines(core);	/* N123 */
			return ret;
		}
		core->hcodec_powered = true;
	}
	mutex_unlock(&core->hw_lock);
	return 0;
}

/*
 * hcodec_power_put - schedule delayed HCODEC poweroff.
 * If hcodec_idle_ms is 0, powers off immediately under hw_lock.
 * Otherwise schedules delayed_work (which self-locks).
 * Caller must NOT hold hw_lock (same as power_get).
 */
void hcodec_power_put(struct amvdec_core *core)
{
	if (hcodec_idle_ms == 0) {
		hcodec_poweroff_and_unpin(core);
	} else {
		schedule_delayed_work(&core->hcodec_poweroff_work,
				      msecs_to_jiffies(hcodec_idle_ms));
	}
}

/**
 * hcodec_compute_buf_layout - compute the DMA buffer layout for a given resolution
 * @width: encode width in pixels
 * @height: encode height in pixels
 * @jpeg: true for the JPEG (intra-only) layout
 * @layout: output layout struct (all fields are set)
 *
 * Each sub-buffer (see &enum hcodec_sub_buf) is sized independently and
 * gets its OWN dma_alloc_coherent at stream-on (KI-116 CMA-fragmentation
 * fix), so the largest contiguous allocation is one sub-buffer, not the
 * whole working set.  All variable buffers are SZ_1M-aligned; fixed-size
 * firmware buffers (ASSIST, DUMP_INFO, CBR) use their defined sizes
 * rounded up to SZ_1M.
 *
 * For JPEG (intra-only: no ME/QDCT-MB ring, no reference frames) the
 * DCT, DEC0 and DEC1 sub-buffers are omitted (sub_size == 0), dropping
 * the largest contiguous need from ~16 MB (H.264 @1080p) to ~1 MB.
 */
void hcodec_compute_buf_layout(u32 width, u32 height, bool jpeg,
				struct hcodec_buf_layout *layout)
{
	u32 mb_w = DIV_ROUND_UP(width, 16);
	u32 mb_h = DIV_ROUND_UP(height, 16);
	u32 canvas_w = ALIGN(width, 64);
	u32 canvas_h = ALIGN(height, 32);
	unsigned int i;

	memset(layout, 0, sizeof(*layout));
	layout->jpeg = jpeg;

	layout->dct_size = ALIGN((u64)mb_w * mb_h * 1024, SZ_1M);
	layout->dec_size = ALIGN((u64)canvas_w * canvas_h * 3 / 2, SZ_1M);
	layout->bs_size = max_t(u32, SZ_1M,
				ALIGN((u32)((u64)canvas_w * canvas_h * 4 / 10),
				      SZ_1M));

	/*
	 * Per-sub-buffer sizes.  H.264-only buffers (DCT, DEC0, DEC1) are
	 * left at 0 for the JPEG layout so they are neither allocated nor
	 * canvas-configured.
	 */
	if (!jpeg) {
		layout->sub_size[HCODEC_SUB_DCT]  = layout->dct_size;
		layout->sub_size[HCODEC_SUB_DEC0] = layout->dec_size;
		layout->sub_size[HCODEC_SUB_DEC1] = layout->dec_size;
	}
	layout->sub_size[HCODEC_SUB_BS]        = layout->bs_size;
	layout->sub_size[HCODEC_SUB_ASSIST]    = ALIGN(HCODEC_ASSIST_SIZE,
						       SZ_1M);
	layout->sub_size[HCODEC_SUB_DUMP_INFO] = ALIGN(HCODEC_DUMP_INFO_SIZE,
						       SZ_1M);
	layout->sub_size[HCODEC_SUB_CBR]       = ALIGN(HCODEC_CBR_INFO_SIZE +
						       HCODEC_CBR_TABLE_SIZE,
						       SZ_1M);

	layout->total_size = 0;
	for (i = 0; i < HCODEC_SUB_COUNT; i++)
		layout->total_size += layout->sub_size[i];
}
EXPORT_SYMBOL_GPL(hcodec_compute_buf_layout);

int hcodec_load_firmware(struct amvdec_core *core)
{
	const struct firmware *fw;
	const struct amvdec_fw_version *match;
	const char *fw_name;
	void *mc_addr;
	dma_addr_t mc_paddr;
	u32 i;
	int ret;

	fw_name = fw_h264_names[core->platform->revision];

	ret = request_firmware(&fw, fw_name, core->dev);
	if (ret) {
		dev_err(core->dev, "Failed to load firmware %s: %d\n",
			fw_name, ret);
		return ret;
	}

	match = amvdec_fw_identify(core->dev, fw_name,
				   fw->data, fw->size,
				   hcodec_fw_versions, HCODEC_FW_VERSIONS_COUNT);
	if (IS_ERR(match)) {
		ret = PTR_ERR(match);
		release_firmware(fw);
		return ret;
	}
	/* match may be NULL only if table is empty -- won't happen here.
	 * Encoders have no quirk bits yet; don't persist match. */

	if (fw->size < 4096 || fw->size % 4) {
		dev_err(core->dev, "Invalid firmware size: %zu\n", fw->size);
		ret = -EINVAL;
		goto out;
	}

	mc_addr = dma_alloc_coherent(core->dev, fw->size,
				     &mc_paddr, GFP_KERNEL);
	if (!mc_addr) {
		ret = -ENOMEM;
		goto out;
	}

	memcpy(mc_addr, fw->data, fw->size);

	amvdec_write_dos(core, HCODEC_MPSR, 0);
	amvdec_write_dos(core, HCODEC_CPSR, 0);

	/* Configure MMC controller - required before firmware DMA */
	amvdec_write_dos(core, HCODEC_ASSIST_MMC_CTRL1, ASSIST_MMC_CTRL1_INIT);

	/* DMA firmware to IMEM - BSP loads exactly 0x1000 words (16KB) */
	amvdec_write_dos(core, HCODEC_IMEM_DMA_ADR, mc_paddr);
	amvdec_write_dos(core, HCODEC_IMEM_DMA_COUNT, HCODEC_MC_SIZE / 4);
	amvdec_write_dos(core, HCODEC_IMEM_DMA_CTRL,
			 IMEM_DMA_CTRL_FILL | IMEM_DMA_CTRL_CTRL_BITS);

	for (i = 0; i < 1000; i++) {
		if (!(amvdec_read_dos(core, HCODEC_IMEM_DMA_CTRL) & IMEM_DMA_CTRL_FILL))
			break;
		usleep_range(100, 200);
	}

	if (i >= 1000) {
		dev_err(core->dev, "HCODEC firmware DMA timeout\n");
		ret = -ETIMEDOUT;
	}

	dma_free_coherent(core->dev, fw->size, mc_addr, mc_paddr);
out:
	release_firmware(fw);
	return ret;
}

/*
 * Cache firmware in a persistent DMA buffer for fast per-frame reload.
 * Called once at session start. The cached buffer stays allocated until
 * the session ends (hcodec_free_cached_firmware).
 */
int hcodec_cache_firmware(struct amvdec_core *core,
			  struct amvenc_ctx *ctx)
{
	const struct firmware *fw;
	const struct amvdec_fw_version *match;
	const char *fw_name;
	int ret;

	fw_name = fw_h264_names[core->platform->revision];

	ret = request_firmware(&fw, fw_name, core->dev);
	if (ret) {
		dev_err(core->dev, "Failed to load firmware %s: %d\n",
			fw_name, ret);
		return ret;
	}

	match = amvdec_fw_identify(core->dev, fw_name,
				   fw->data, fw->size,
				   hcodec_fw_versions, HCODEC_FW_VERSIONS_COUNT);
	if (IS_ERR(match)) {
		ret = PTR_ERR(match);
		release_firmware(fw);
		return ret;
	}
	/* Encoders have no quirk bits yet; don't persist match. */

	if (fw->size < 4096 || fw->size % 4) {
		dev_err(core->dev, "Invalid firmware size: %zu\n", fw->size);
		release_firmware(fw);
		return -EINVAL;
	}

	ctx->fw_vaddr = dma_alloc_coherent(core->dev, fw->size,
					    &ctx->fw_paddr, GFP_KERNEL);
	if (!ctx->fw_vaddr) {
		release_firmware(fw);
		return -ENOMEM;
	}

	memcpy(ctx->fw_vaddr, fw->data, fw->size);
	ctx->fw_size = fw->size;

	release_firmware(fw);
	return 0;
}

void hcodec_free_cached_firmware(struct amvenc_ctx *ctx)
{
	if (ctx->fw_vaddr) {
		dma_free_coherent(ctx->core->dev, ctx->fw_size,
				  ctx->fw_vaddr, ctx->fw_paddr);
		ctx->fw_vaddr = NULL;
		ctx->fw_size = 0;
	}
}

/*
 * Fast firmware reload from cached DMA buffer.
 * Only does the IMEM DMA transfer -- no file I/O.
 * Takes ~100us for 16KB DMA vs ~10ms for request_firmware().
 */
int hcodec_reload_firmware(struct amvdec_core *core,
			   struct amvenc_ctx *ctx)
{
	u32 i;

	if (!ctx->fw_vaddr)
		return -EINVAL;

	amvdec_write_dos(core, HCODEC_MPSR, 0);
	amvdec_write_dos(core, HCODEC_CPSR, 0);

	amvdec_write_dos(core, HCODEC_ASSIST_MMC_CTRL1, ASSIST_MMC_CTRL1_INIT);

	amvdec_write_dos(core, HCODEC_IMEM_DMA_ADR, ctx->fw_paddr);
	amvdec_write_dos(core, HCODEC_IMEM_DMA_COUNT, HCODEC_MC_SIZE / 4);
	amvdec_write_dos(core, HCODEC_IMEM_DMA_CTRL,
			 IMEM_DMA_CTRL_FILL | IMEM_DMA_CTRL_CTRL_BITS);

	for (i = 0; i < 1000; i++) {
		if (!(amvdec_read_dos(core, HCODEC_IMEM_DMA_CTRL) & IMEM_DMA_CTRL_FILL))
			break;
		usleep_range(10, 20);
	}

	if (i >= 1000) {
		dev_err(core->dev, "HCODEC firmware reload DMA timeout\n");
		return -ETIMEDOUT;
	}

	return 0;
}

void hcodec_reset(struct amvdec_core *core)
{
	amvdec_read_dos(core, DOS_SW_RESET1);
	amvdec_read_dos(core, DOS_SW_RESET1);
	amvdec_read_dos(core, DOS_SW_RESET1);
	amvdec_write_dos(core, DOS_SW_RESET1, HCODEC_RESET_FULL);
	amvdec_write_dos(core, DOS_SW_RESET1, 0);
	amvdec_read_dos(core, DOS_SW_RESET1);
	amvdec_read_dos(core, DOS_SW_RESET1);
	amvdec_read_dos(core, DOS_SW_RESET1);
}

void hcodec_start(struct amvdec_core *core)
{
	amvdec_read_dos(core, DOS_SW_RESET1);
	amvdec_read_dos(core, DOS_SW_RESET1);
	amvdec_read_dos(core, DOS_SW_RESET1);
	amvdec_write_dos(core, DOS_SW_RESET1, HCODEC_RESET_MC);
	amvdec_write_dos(core, DOS_SW_RESET1, 0);
	amvdec_read_dos(core, DOS_SW_RESET1);
	amvdec_read_dos(core, DOS_SW_RESET1);
	amvdec_read_dos(core, DOS_SW_RESET1);

	amvdec_write_dos(core, HCODEC_MPSR, 1);
}

void hcodec_stop(struct amvdec_core *core)
{
	ulong timeout = jiffies + HZ;

	amvdec_write_dos(core, HCODEC_MPSR, 0);
	amvdec_write_dos(core, HCODEC_CPSR, 0);

	while (amvdec_read_dos(core, HCODEC_IMEM_DMA_CTRL) & IMEM_DMA_CTRL_FILL) {
		if (time_after(jiffies, timeout))
			break;
		usleep_range(100, 200);
	}

	amvdec_write_dos(core, DOS_SW_RESET1,
			 HCODEC_RESET_MC | HCODEC_RESET_FULL);
	amvdec_write_dos(core, DOS_SW_RESET1, 0);
	amvdec_read_dos(core, DOS_SW_RESET1);
}

int hcodec_alloc_canvases(struct amvdec_core *core,
			  struct amvenc_ctx *ctx)
{
	int ret, i;

	for (i = 0; i < 9; i++) {
		ret = meson_canvas_alloc(core->canvas, &ctx->canvas_ids[i]);
		if (ret) {
			dev_err(core->dev,
				"Failed to alloc canvas %d: %d\n", i, ret);
			while (i--)
				meson_canvas_free(core->canvas,
						  ctx->canvas_ids[i]);
			ctx->canvas_num = 0;
			return ret;
		}
	}
	ctx->canvas_num = 9;
	return 0;
}

void hcodec_free_canvases(struct amvdec_core *core,
			  struct amvenc_ctx *ctx)
{
	u32 i;

	for (i = 0; i < ctx->canvas_num; i++)
		meson_canvas_free(core->canvas, ctx->canvas_ids[i]);
	ctx->canvas_num = 0;
}

void hcodec_setup_canvases(struct amvdec_core *core,
			   struct amvenc_ctx *ctx)
{
	u32 canvas_w = ALIGN(ctx->width, 32);
	u32 canvas_h = ALIGN(ctx->height, 16);
	dma_addr_t dec0 = ctx->sub_paddr[HCODEC_SUB_DEC0];
	dma_addr_t dec1 = ctx->sub_paddr[HCODEC_SUB_DEC1];

	/*
	 * JPEG is intra-only: no reconstruction / reference frames, so the
	 * DEC0/DEC1 sub-buffers are not allocated (sub_paddr == 0) and their
	 * canvases must NOT be configured -- programming a canvas with base 0
	 * would point the engine at physical address 0 (KI-116 open question
	 * #2: skip the DEC canvas CONFIG for JPEG, not just the alloc).
	 */
	if (ctx->buf_layout.jpeg)
		goto pack;

	/* dec0: Y, UV, UV (global DMC canvas via meson_canvas_config) */
	meson_canvas_config(core->canvas, ctx->canvas_ids[0],
			    dec0,
			    canvas_w, canvas_h,
			    MESON_CANVAS_WRAP_NONE,
			    MESON_CANVAS_BLKMODE_LINEAR,
			    0);
	meson_canvas_config(core->canvas, ctx->canvas_ids[1],
			    dec0 + canvas_w * canvas_h,
			    canvas_w, canvas_h / 2,
			    MESON_CANVAS_WRAP_NONE,
			    MESON_CANVAS_BLKMODE_LINEAR,
			    0);
	meson_canvas_config(core->canvas, ctx->canvas_ids[2],
			    dec0 + canvas_w * canvas_h,
			    canvas_w, canvas_h / 2,
			    MESON_CANVAS_WRAP_NONE,
			    MESON_CANVAS_BLKMODE_LINEAR,
			    0);

	/* dec1: Y, UV, UV */
	meson_canvas_config(core->canvas, ctx->canvas_ids[3],
			    dec1,
			    canvas_w, canvas_h,
			    MESON_CANVAS_WRAP_NONE,
			    MESON_CANVAS_BLKMODE_LINEAR,
			    0);
	meson_canvas_config(core->canvas, ctx->canvas_ids[4],
			    dec1 + canvas_w * canvas_h,
			    canvas_w, canvas_h / 2,
			    MESON_CANVAS_WRAP_NONE,
			    MESON_CANVAS_BLKMODE_LINEAR,
			    0);
	meson_canvas_config(core->canvas, ctx->canvas_ids[5],
			    dec1 + canvas_w * canvas_h,
			    canvas_w, canvas_h / 2,
			    MESON_CANVAS_WRAP_NONE,
			    MESON_CANVAS_BLKMODE_LINEAR,
			    0);

pack:
	/*
	 * Pack canvas IDs: [23:16]=V [15:8]=U [7:0]=Y.
	 * Only set on first call.  venc.c swaps dblk_canvas/ref_canvas
	 * after each encoded frame for motion estimation continuity.
	 * Re-assigning here every frame would undo the swap and cause
	 * P-frame corruption (KI-074).
	 */
	if (!ctx->dblk_canvas && !ctx->ref_canvas) {
		ctx->dblk_canvas = ((u32)ctx->canvas_ids[2] << 16) |
				   ((u32)ctx->canvas_ids[1] << 8) |
				   ctx->canvas_ids[0];
		ctx->ref_canvas = ((u32)ctx->canvas_ids[5] << 16) |
				  ((u32)ctx->canvas_ids[4] << 8) |
				  ctx->canvas_ids[3];
	}
}

void hcodec_setup_buffers(struct amvdec_core *core,
			  struct amvenc_ctx *ctx,
			  dma_addr_t vlc_dst, u32 vlc_dst_size)
{
	dma_addr_t dct = ctx->sub_paddr[HCODEC_SUB_DCT];

	/*
	 * DCT (QDCT MB ring): H.264 only.  JPEG is intra-only and never
	 * programs the QDCT MB ring (matches BSP set_jpeg_input_format,
	 * which omits HCODEC_QDCT_MB_*); the DCT sub-buffer is not
	 * allocated for JPEG (dct == 0 / dct_size == 0), so skip it.
	 */
	if (!ctx->buf_layout.jpeg) {
		amvdec_write_dos(core, HCODEC_QDCT_MB_START_PTR,
				 dct);
		amvdec_write_dos(core, HCODEC_QDCT_MB_END_PTR,
				 dct + ctx->buf_layout.dct_size - 1);
		amvdec_write_dos(core, HCODEC_QDCT_MB_WR_PTR,
				 dct);
		amvdec_write_dos(core, HCODEC_QDCT_MB_RD_PTR,
				 dct);
		amvdec_write_dos(core, HCODEC_QDCT_MB_BUFF, 0);
	}

	/* Bitstream output buffer: zero-copy into caller-provided destination */
	amvdec_write_dos(core, HCODEC_VLC_VB_MEM_CTL, VLC_VB_MEM_CTL_INIT);
	amvdec_write_dos(core, HCODEC_VLC_VB_START_PTR,  vlc_dst);
	amvdec_write_dos(core, HCODEC_VLC_VB_WR_PTR,     vlc_dst);
	amvdec_write_dos(core, HCODEC_VLC_VB_SW_RD_PTR,  vlc_dst);
	amvdec_write_dos(core, HCODEC_VLC_VB_END_PTR,    vlc_dst + vlc_dst_size - 1);
	amvdec_write_dos(core, HCODEC_VLC_VB_CONTROL, 1);
	amvdec_write_dos(core, HCODEC_VLC_VB_CONTROL, VLC_VB_CTRL_INIT);

	/* Assist buffer */
	amvdec_write_dos(core, MEM_OFFSET_REG,
			 ctx->sub_paddr[HCODEC_SUB_ASSIST]);
}

void hcodec_setup_encoder(struct amvdec_core *core,
			  struct amvenc_ctx *ctx, bool idr)
{
	amvdec_write_dos(core, HCODEC_VLC_TOTAL_BYTES, 0);
	amvdec_write_dos(core, HCODEC_VLC_CONFIG, VLC_CONFIG_INIT);
	amvdec_write_dos(core, HCODEC_VLC_INT_CONTROL, 0);

	amvdec_write_dos(core, HCODEC_ASSIST_AMR1_INT0, AMR1_INT0_HCODEC);
	amvdec_write_dos(core, HCODEC_ASSIST_AMR1_INT1, AMR1_INT1_HCODEC);
	amvdec_write_dos(core, HCODEC_ASSIST_AMR1_INT3, AMR1_INT3_HCODEC);

	amvdec_write_dos(core, IDR_PIC_ID, ctx->idr_pic_id);
	amvdec_write_dos(core, FRAME_NUMBER, idr ? 0 : ctx->frame_number);
	amvdec_write_dos(core, PIC_ORDER_CNT_LSB,
			 idr ? 0 : ctx->pic_order_cnt_lsb);
	amvdec_write_dos(core, LOG2_MAX_PIC_ORDER_CNT_LSB, 4);
	amvdec_write_dos(core, LOG2_MAX_FRAME_NUM, 4);
	amvdec_write_dos(core, ANC0_BUFFER_ID, 0);
	/*
	 * QPPICTURE must always be HCODEC_DEFAULT_QP (26), matching the
	 * BSP (encoder.c line 4460: init_qppicture = 26).  The firmware
	 * uses QPPICTURE as the reference QP for pic_init_qp in its
	 * internally-generated SPS/PPS and for slice_qp_delta in slice
	 * headers (slice_qp_delta = actual_qp - QPPICTURE).  The actual
	 * per-block QP is controlled by the quant tables and CBR table,
	 * not by QPPICTURE.
	 *
	 * Setting QPPICTURE to ctx->qp (which varies per-frame with RC)
	 * causes pic_init_qp to change between PPS regenerations, making
	 * the decoder compute wrong actual QP from stale PPS values.
	 */
	amvdec_write_dos(core, QPPICTURE, HCODEC_DEFAULT_QP);
}

static void hcodec_prog_qtbl(struct amvdec_core *core,
			     struct amvenc_ctx *ctx)
{
	int i;

	amvdec_write_dos(core, HCODEC_Q_QUANT_CONTROL,
			 (0 << Q_QUANT_CTRL_ADDR_SHIFT) | Q_QUANT_CTRL_WRITE_EN);
	for (i = 0; i < 8; i++)
		amvdec_write_dos(core, HCODEC_QUANT_TABLE_DATA,
				 ctx->quant_tbl_i4[i]);

	amvdec_write_dos(core, HCODEC_Q_QUANT_CONTROL,
			 (8 << Q_QUANT_CTRL_ADDR_SHIFT) | Q_QUANT_CTRL_WRITE_EN);
	for (i = 0; i < 8; i++)
		amvdec_write_dos(core, HCODEC_QUANT_TABLE_DATA,
				 ctx->quant_tbl_i16[i]);

	amvdec_write_dos(core, HCODEC_Q_QUANT_CONTROL,
			 (16 << Q_QUANT_CTRL_ADDR_SHIFT) | Q_QUANT_CTRL_WRITE_EN);
	for (i = 0; i < 8; i++)
		amvdec_write_dos(core, HCODEC_QUANT_TABLE_DATA,
				 ctx->quant_tbl_me[i]);
}

void hcodec_setup_ie_me(struct amvdec_core *core,
			struct amvenc_ctx *ctx, u32 quant, bool idr)
{
	u32 pic_width_in_mb = (ctx->width + 15) / 16;
	u32 mb_height = (ctx->height + 15) / 16;
	u32 i_pic_qp = quant;
	u32 p_pic_qp = quant;
	u32 i_pic_qp_c, p_pic_qp_c;
	u32 me_w = ctx->me_weight;
	u32 i4_w = ctx->i4mb_weight;
	u32 i16_w = ctx->i16mb_weight;
	int i;

	amvdec_write_dos(core, IE_ME_MODE,
			 (12 & IE_PIPELINE_BLOCK_MASK) <<
			 IE_PIPELINE_BLOCK_SHIFT);
	amvdec_write_dos(core, HCODEC_HENC_SCRATCH_F, 0);

	if (idr)
		amvdec_write_dos(core, IE_ME_MB_TYPE, HENC_MB_Type_I4MB);
	else
		amvdec_write_dos(core, IE_ME_MB_TYPE,
				 (HENC_SKIP_RUN_AUTO << 16) |
				 (HENC_MB_Type_AUTO << 4) |
				 (HENC_MB_Type_AUTO << 0));

	/*
	 * Multi-slice: FIXED_SLICE_CFG [31:16]=rows_per_slice_P,
	 * [15:0]=rows_per_slice_I.  0 = single slice (full frame).
	 * BSP computes rows from slice_max_mb / pic_width_in_mb.
	 */
	if (ctx->slice_max_mb && ctx->slice_max_mb < pic_width_in_mb * mb_height) {
		u32 rows = ctx->slice_max_mb / pic_width_in_mb;

		if (rows < 1)
			rows = 1;
		amvdec_write_dos(core, FIXED_SLICE_CFG,
				 (rows << 16) | rows);
	} else {
		amvdec_write_dos(core, FIXED_SLICE_CFG, 0);
	}

	amvdec_write_dos(core, HCODEC_HDEC_MC_OMEM_AUTO,
			 MC_OMEM_AUTO_FILL_EN |
			 ((pic_width_in_mb - 1) << MC_OMEM_AUTO_WIDTH_SHIFT));

	amvdec_write_dos(core, HCODEC_VLC_ADV_CONFIG, VLC_ADV_CONFIG_INIT);

	amvdec_write_dos(core, HCODEC_QDCT_ADV_CONFIG, QDCT_ADV_CONFIG_INIT);

	/*
	 * QP-adaptive lambda weights (BSP avc_prot_init ted_lambda).
	 * IE_WEIGHT + ME_WEIGHT + ADV_MV use lambda-derived values.
	 * SAD_CONTROL keeps static per-request weights (BSP parity).
	 * V3_IPRED_TYPE_WEIGHT biases intra mode selection by cost.
	 */
	{
		static const u16 lambda_tbl[52] = {
			4, 4, 5, 5, 6, 7, 7, 8, 9, 10,
			12, 13, 15, 17, 19, 21, 23, 26, 30, 33,
			37, 42, 47, 53, 59, 66, 74, 83, 94, 105,
			118, 132, 149, 167, 187, 210, 236, 265, 297, 334,
			375, 421, 472, 530, 595, 668, 749, 841, 944, 1060,
			1189, 1335
		};
		u32 lam = lambda_tbl[clamp_t(u32, quant, 0, 51)];
		u32 ipred_i4m = min_t(u32, (1 * lam) >> 4, 255);
		u32 ipred_i4e = min_t(u32, (4 * lam) >> 4, 255);
		u32 ipred_16h = min_t(u32, (6 * lam) >> 4, 255);
		u32 ipred_16v = min_t(u32, (6 * lam) >> 4, 255);
		u32 ipred_16d = min_t(u32, (7 * lam) >> 4, 255);
		u32 ipred_ch = min_t(u32, (3 * lam) >> 4, 255);
		u32 ipred_cv = min_t(u32, (5 * lam) >> 4, 255);
		u32 ipred_cd = min_t(u32, (1 * lam) >> 4, 255);

		amvdec_write_dos(core, HCODEC_IE_WEIGHT,
				 (0 << 16) | (((1 * lam) >> 4) << 0));
		amvdec_write_dos(core, HCODEC_ME_WEIGHT,
				 (2 * lam) >> 4);

		/* SAD_CONTROL: static weights (BSP request->*_weight) */
		amvdec_write_dos(core, HCODEC_SAD_CONTROL_0,
				 (i16_w << 16) | (i4_w << 0));
		amvdec_write_dos(core, HCODEC_SAD_CONTROL_1,
				 (1 << 24) | (1 << 20) | (1 << 16) |
				 (me_w << 0));

		amvdec_write_dos(core, HCODEC_ADV_MV_CTL0,
				 (1 << 31) | (1 << 30) |
				 (min_t(u32, (1 * lam) >> 4, 0xffff) << 16) |
				 (min_t(u32, (5 * lam) >> 4, 0xffff) << 0));
		amvdec_write_dos(core, HCODEC_ADV_MV_CTL1,
				 (min_t(u32, (1 * lam) >> 4, 0xffff) << 16) |
				 (1 << 15) |
				 (min_t(u32, (5 * lam) >> 4, 0xffff) << 0));

		amvdec_write_dos(core, HCODEC_V3_IPRED_TYPE_WEIGHT_0,
				 (ipred_ch << 24) | (ipred_cv << 16) |
				 (ipred_i4e << 8) | (ipred_i4m << 0));
		amvdec_write_dos(core, HCODEC_V3_IPRED_TYPE_WEIGHT_1,
				 (ipred_16d << 24) | (ipred_16h << 16) |
				 (ipred_16v << 8) | (ipred_cd << 0));
	}

	hcodec_prog_qtbl(core, ctx);

	i_pic_qp_c = (i_pic_qp < 51) ? qp_to_qpc[i_pic_qp] : 39;
	p_pic_qp_c = (p_pic_qp < 51) ? qp_to_qpc[p_pic_qp] : 39;

	amvdec_write_dos(core, HCODEC_QDCT_Q_QUANT_I,
			 (i_pic_qp_c << Q_QUANT_CHROMA_QP_SHIFT) |
			 (i_pic_qp << Q_QUANT_LUMA_QP_SHIFT) |
			 ((i_pic_qp_c % 6) << Q_QUANT_CHROMA_MOD6_SHIFT) |
			 ((i_pic_qp_c / 6) << Q_QUANT_CHROMA_DIV6_SHIFT) |
			 ((i_pic_qp % 6) << Q_QUANT_LUMA_MOD6_SHIFT) |
			 ((i_pic_qp / 6) << Q_QUANT_LUMA_DIV6_SHIFT));
	amvdec_write_dos(core, HCODEC_QDCT_Q_QUANT_P,
			 (p_pic_qp_c << Q_QUANT_CHROMA_QP_SHIFT) |
			 (p_pic_qp << Q_QUANT_LUMA_QP_SHIFT) |
			 ((p_pic_qp_c % 6) << Q_QUANT_CHROMA_MOD6_SHIFT) |
			 ((p_pic_qp_c / 6) << Q_QUANT_CHROMA_DIV6_SHIFT) |
			 ((p_pic_qp % 6) << Q_QUANT_LUMA_MOD6_SHIFT) |
			 ((p_pic_qp / 6) << Q_QUANT_LUMA_DIV6_SHIFT));

	amvdec_write_dos(core, HCODEC_QDCT_VLC_QUANT_CTL_0,
			 (i_pic_qp << VLC_QUANT_CTL0_QP_HI_SHIFT) |
			 (i_pic_qp << VLC_QUANT_CTL0_QP_LO_SHIFT));
	amvdec_write_dos(core, HCODEC_QDCT_VLC_QUANT_CTL_1,
			 (14 << VLC_QUANT_CTL1_QP1_SHIFT) |
			 (13 << VLC_QUANT_CTL1_QP0_SHIFT));

	amvdec_write_dos(core, HCODEC_VLC_PIC_SIZE,
			 ctx->width | (ctx->height << 16));
	amvdec_write_dos(core, HCODEC_VLC_PIC_POSITION, 0);

	amvdec_write_dos(core, HCODEC_IGNORE_CONFIG, 0);
	amvdec_write_dos(core, HCODEC_IGNORE_CONFIG_2, 0);

	amvdec_write_dos(core, HCODEC_QDCT_MB_CONTROL, QDCT_MB_CTRL_RESET);
	amvdec_write_dos(core, HCODEC_QDCT_MB_CONTROL, QDCT_MB_CTRL_RUN);

	/* SAD pipeline: IE shift select + switch */
	amvdec_write_dos(core, HCODEC_SAD_CONTROL,
			 SAD_CTRL_IE_SAD_SHIFT | SAD_CTRL_SWITCH);
	amvdec_write_dos(core, HCODEC_IE_RESULT_BUFFER, 0);
	/* SAD pipeline: IE result read + ME shift select */
	amvdec_write_dos(core, HCODEC_SAD_CONTROL,
			 SAD_CTRL_IE_RESULT_RD | SAD_CTRL_ME_SAD_SHIFT);

	/* IE: start then stop (pulse IE pipeline reset) */
	amvdec_write_dos(core, HCODEC_IE_CONTROL,
			 IE_CTRL_ACTIVE | IE_CTRL_IE_START);
	amvdec_write_dos(core, HCODEC_IE_CONTROL,
			 IE_CTRL_ACTIVE);

	amvdec_write_dos(core, HCODEC_ME_SKIP_LINE,
			 (8 << ME_SKIP_Y3_SHIFT) |
			 (8 << ME_SKIP_X3_SHIFT) |
			 (2 << ME_SKIP_Y2_SHIFT));
	amvdec_write_dos(core, HCODEC_ME_MV_MERGE_CTL, MV_MERGE_CTL_INIT);
	amvdec_write_dos(core, HCODEC_ME_STEP0_CLOSE_MV, STEP0_CLOSE_MV_INIT);
	amvdec_write_dos(core, HCODEC_ME_SAD_ENOUGH_01, ME_SAD_ENOUGH_01_INIT);
	amvdec_write_dos(core, HCODEC_ME_SAD_ENOUGH_23, ME_SAD_ENOUGH_23_INIT);
	amvdec_write_dos(core, HCODEC_ME_F_SKIP_SAD, 0);
	amvdec_write_dos(core, HCODEC_ME_F_SKIP_WEIGHT, 0);
	amvdec_write_dos(core, HCODEC_ME_MV_WEIGHT_01, 0);
	amvdec_write_dos(core, HCODEC_ME_MV_WEIGHT_23, 0);
	amvdec_write_dos(core, HCODEC_ME_SAD_RANGE_INC, SAD_RANGE_INC_INIT);

	for (i = 0; i < 64; i++)
		amvdec_write_dos(core, HCODEC_V3_MV_SAD_TABLE,
				 v3_mv_sad[i]);

	amvdec_write_dos(core, HCODEC_V3_IPRED_TYPE_WEIGHT_0,
			 (V3_IPRED_W0_CH << 24) | (V3_IPRED_W0_CV << 16) |
			 (V3_IPRED_W0_DDL << 8) | (V3_IPRED_W0_DDR << 0));
	amvdec_write_dos(core, HCODEC_V3_IPRED_TYPE_WEIGHT_1,
			 (V3_IPRED_W1_VL << 24) | (V3_IPRED_W1_VR << 16) |
			 (V3_IPRED_W1_HD << 8) | (V3_IPRED_W1_HU << 0));
	amvdec_write_dos(core, HCODEC_V3_LEFT_SMALL_MAX_SAD,
			 (V3_LEFT_SMALL_MAX_SAD_HI << 16) |
			 (V3_LEFT_SMALL_MAX_SAD_LO << 0));

	amvdec_write_dos(core, HCODEC_V3_SKIP_CONTROL, V3_SKIP_CONTROL_INIT);
	amvdec_write_dos(core, HCODEC_V3_SKIP_WEIGHT,
			 (V3_SKIP_W1_DEFAULT << V3_SKIP_W1_SHIFT) |
			 (V3_SKIP_W0_DEFAULT << V3_SKIP_W0_SHIFT));
	amvdec_write_dos(core, HCODEC_V3_L1_SKIP_MAX_SAD,
			 (V3_L1_FMAX_SAD_DEFAULT << V3_L1_FMAX_SAD_SHIFT) |
			 (V3_L1_MAX_SAD_DEFAULT << V3_L1_MAX_SAD_SHIFT));
	amvdec_write_dos(core, HCODEC_V3_L2_SKIP_WEIGHT,
			 (V3_L2_FSAD2_DEFAULT << V3_L2_FSAD2_SHIFT) |
			 (V3_L2_W2_DEFAULT << V3_L2_W2_SHIFT));
	amvdec_write_dos(core, HCODEC_V3_F_ZERO_CTL_0,
			 ((i16_w + V3_I16_WEIGHT_OFFSET) << 16) |
			 ((i4_w + V3_I4_WEIGHT_OFFSET) << 0));
	amvdec_write_dos(core, HCODEC_V3_F_ZERO_CTL_1,
			 (V3_F_ZERO_CTL1_HI << 16) |
			 ((me_w + V3_ME_WEIGHT_OFFSET) << 0));

	amvdec_write_dos(core, HCODEC_V4_FORCE_SKIP_CFG,
			 (i_pic_qp << V4_QP_THRESH_HI_SHIFT) |
			 (i_pic_qp << V4_QP_THRESH_LO_SHIFT) |
			 (V4_SKIP_SAD_MULT_DEFAULT << V4_SKIP_SAD_MULT_SHIFT) |
			 (V4_SKIP_COEFF_MULT_DEFAULT << V4_SKIP_COEFF_MULT_SHIFT));

	amvdec_write_dos(core, HCODEC_V5_SIMPLE_MB_CTL, 0);
	amvdec_write_dos(core, HCODEC_V5_SIMPLE_MB_CTL, V5_SIMPLE_MB_CTL_INIT);
	amvdec_write_dos(core, HCODEC_V5_MB_DIFF_SUM, 0);
	amvdec_write_dos(core, HCODEC_V5_SMALL_DIFF_CNT,
			 (V5_SMALL_DIFF_THR_DEFAULT << V5_SMALL_DIFF_THR_SHIFT) |
			 (V5_DIFF_COUNT_DEFAULT << V5_DIFF_COUNT_SHIFT));
	amvdec_write_dos(core, HCODEC_V5_SIMPLE_MB_DQUANT, V5_DQUANT_IDR);
	amvdec_write_dos(core, HCODEC_V5_SIMPLE_MB_ME_WEIGHT, 0);
	amvdec_write_dos(core, HCODEC_QDCT_CONFIG, 1);

	amvdec_write_dos(core, HCODEC_IE_DATA_FEED_BUFF_INFO, 0);
	amvdec_write_dos(core, HCODEC_CURR_CANVAS_CTRL, 0);

	amvdec_write_dos(core, HCODEC_VLC_CONFIG,
			 amvdec_read_dos(core, HCODEC_VLC_CONFIG) | 1);

	/* Firmware writes per-MB info here; must point to valid DMA memory */
	amvdec_write_dos(core, INFO_DUMP_START_ADDR,
			 ctx->sub_paddr[HCODEC_SUB_DUMP_INFO]);

	/* Clear and enable mailbox IRQ */
	amvdec_write_dos(core, HCODEC_ASSIST_MBOX2_CLR_REG, 1);
	amvdec_write_dos(core, HCODEC_ASSIST_MBOX2_MASK, 1);
}

/*
 * Program MFDIN noise reduction registers (SNR/TNR).
 * BSP default values used — these are proven tuning parameters.
 */
static void hcodec_setup_nr(struct amvdec_core *core, u32 nr_mode)
{
	u32 y_snr_en = (nr_mode == 1 || nr_mode == 3) ? 1 : 0;
	u32 y_tnr_en = (nr_mode == 2 || nr_mode == 3) ? 1 : 0;

	/* Y SNR: err_norm=1, gau_bld_core=1, ofst=-1, rate=48, alpha 0-63 */
	amvdec_write_dos(core, HCODEC_MFDIN_REG0D,
			 (y_snr_en << 0) | (1 << 1) | (1 << 2) |
			 (((-1) & 0xff) << 6) | (48 << 14) |
			 (0 << 20) | (63 << 26));

	/* Y TNR: mc_en=1, txt_mode=0, sad_margin=1, alpha 32-63, deghost=0 */
	amvdec_write_dos(core, HCODEC_MFDIN_REG0E,
			 (y_tnr_en << 0) | (1 << 1) | (0 << 2) |
			 (1 << 3) | (32 << 7) | (63 << 13) | (0 << 19));

	/* Y TNR motion: cortxt_rate=1, distxt_ofst=5, distxt_rate=4,
	 * dismot_ofst=4, frcsad_lock=8 */
	amvdec_write_dos(core, HCODEC_MFDIN_REG0F,
			 (1 << 0) | (5 << 8) | (4 << 4) |
			 (4 << 16) | (8 << 24));

	/* Y TNR alpha: frc_gain=10, nrm_gain=216, dis_gain=128, dis_ofst=32 */
	amvdec_write_dos(core, HCODEC_MFDIN_REG10,
			 (10 << 0) | (216 << 8) | (128 << 16) | (32 << 24));

	/* Y blend: beta2alp_rate=16, beta_min=0, beta_max=63 */
	amvdec_write_dos(core, HCODEC_MFDIN_REG11,
			 (16 << 0) | (0 << 8) | (63 << 14));

	/* C SNR: same as Y SNR enable, err_norm=1, gau_bld_core=1 */
	amvdec_write_dos(core, HCODEC_MFDIN_REG12,
			 (y_snr_en << 0) | (1 << 1) | (1 << 2) |
			 (((-1) & 0xff) << 6) | (48 << 14) |
			 (0 << 20) | (63 << 26));

	/* C TNR: always disabled per BSP */
	amvdec_write_dos(core, HCODEC_MFDIN_REG13,
			 (0 << 0) | (1 << 1) | (0 << 2) |
			 (1 << 3) | (32 << 7) | (63 << 13) | (0 << 19));

	/* C TNR motion (same defaults as Y) */
	amvdec_write_dos(core, HCODEC_MFDIN_REG14,
			 (1 << 0) | (5 << 8) | (4 << 4) |
			 (4 << 16) | (8 << 24));

	amvdec_write_dos(core, HCODEC_MFDIN_REG15,
			 (10 << 0) | (216 << 8) | (128 << 16) | (32 << 24));

	/* C blend */
	amvdec_write_dos(core, HCODEC_MFDIN_REG16,
			 (16 << 0) | (0 << 8) | (63 << 14));
}

void hcodec_setup_mfdin(struct amvdec_core *core,
			struct amvenc_ctx *ctx,
			dma_addr_t y_addr, dma_addr_t uv_addr,
			dma_addr_t v_addr, u32 canvas_w)
{
	u32 picsize_x = ALIGN(ctx->width, 16);
	u32 picsize_y = ctx->height;
	u32 src_height = ctx->height;
	u32 canvas;
	u32 iformat, interp_en, canv_bpp;
	/*
	 * BSP disables NR on IDR frames (encoder.c line 3337:
	 * "if (cmd == ENCODER_IDR) wq->request.nr_mode = 0").
	 * IDR frames are reference-free, so TNR (temporal NR) has no
	 * reference to blend with, and SNR on IDR wastes bits on
	 * spatial smoothing that the decoder doesn't benefit from.
	 */
	u32 nr_enable = (ctx->nr_mode && !ctx->current_idr) ? 1 : 0;

	/*
	 * Determine MFDIN iformat and canvas BPP from pixel format.
	 * Canvas indices [6]=Y, [7]=UV (or U for planar).
	 * canv_bpp encodes [31:24] = idx1_bppy, idx0_bppy, idx1_bppx, idx0_bppx
	 */
	switch (ctx->src_fmt.pixelformat) {
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_NV21M:
		iformat = MFDIN_FMT_NV21;
		interp_en = 0;
		canv_bpp = (0 << 6) | (1 << 4) | (1 << 2) | (1 << 0);
		meson_canvas_config(core->canvas, ctx->canvas_ids[6],
				    y_addr, canvas_w,
				    src_height,
				    MESON_CANVAS_WRAP_NONE,
				    MESON_CANVAS_BLKMODE_LINEAR, 0);
		meson_canvas_config(core->canvas, ctx->canvas_ids[7],
				    uv_addr, canvas_w,
				    src_height / 2,
				    MESON_CANVAS_WRAP_NONE,
				    MESON_CANVAS_BLKMODE_LINEAR, 0);
		canvas = ((u32)ctx->canvas_ids[7] << 8) | ctx->canvas_ids[6];
		break;

	case V4L2_PIX_FMT_YUV420M:
		iformat = MFDIN_FMT_YUV420;
		interp_en = 0;
		canv_bpp = (0 << 6) | (1 << 4) | (0 << 2) | (1 << 0);
		meson_canvas_config(core->canvas, ctx->canvas_ids[6],
				    y_addr, canvas_w,
				    src_height,
				    MESON_CANVAS_WRAP_NONE,
				    MESON_CANVAS_BLKMODE_LINEAR, 0);
		/* U plane */
		meson_canvas_config(core->canvas, ctx->canvas_ids[7],
				    uv_addr, canvas_w / 2,
				    src_height / 2,
				    MESON_CANVAS_WRAP_NONE,
				    MESON_CANVAS_BLKMODE_LINEAR, 0);
		/* V plane */
		meson_canvas_config(core->canvas, ctx->canvas_ids[8],
				    v_addr, canvas_w / 2,
				    src_height / 2,
				    MESON_CANVAS_WRAP_NONE,
				    MESON_CANVAS_BLKMODE_LINEAR, 0);
		canvas = ((u32)ctx->canvas_ids[8] << 16) |
			 ((u32)ctx->canvas_ids[7] << 8) |
			 ctx->canvas_ids[6];
		break;

	case V4L2_PIX_FMT_YUYV:
		iformat = MFDIN_FMT_YUV422_SINGLE;
		interp_en = 0;
		canv_bpp = (0 << 6) | (1 << 4) | (1 << 2) | (2 << 0);
		meson_canvas_config(core->canvas, ctx->canvas_ids[6],
				    y_addr, canvas_w * 2,
				    src_height,
				    MESON_CANVAS_WRAP_NONE,
				    MESON_CANVAS_BLKMODE_LINEAR, 0);
		canvas = ctx->canvas_ids[6];
		break;

	case V4L2_PIX_FMT_RGB24:
		/*
		 * RGB888: iformat=1, 3 bytes/pixel, single canvas.
		 * MFDIN has built-in R2Y converter (r2y_en bit in REG1_CTRL).
		 */
		iformat = MFDIN_FMT_RGB_SINGLE;
		interp_en = 0;
		canv_bpp = (0 << 6) | (1 << 4) | (2 << 2) | (3 << 0);
		meson_canvas_config(core->canvas, ctx->canvas_ids[6],
				    y_addr, canvas_w * 3,
				    src_height,
				    MESON_CANVAS_WRAP_NONE,
				    MESON_CANVAS_BLKMODE_LINEAR, 0);
		canvas = ctx->canvas_ids[6];
		break;

	case V4L2_PIX_FMT_ABGR32:
		/*
		 * RGBA8888: iformat=12, linear DMA mode (not canvas).
		 * BSP: no canvas config, passes raw phys addr to mfdin_basic.
		 * MFDIN reads via REG5_LNR1 directly. canv_bpp uses default
		 * values (bppx=1 for all non-special formats).
		 */
		iformat = MFDIN_FMT_LINEAR;
		interp_en = 0;
		canv_bpp = (0 << 6) | (1 << 4) | (1 << 2) | (1 << 0);
		canvas = 0; /* no canvas -- linear mode uses REG5 directly */
		/* Do NOT call meson_canvas_config -- linear path bypasses canvas */
		break;

	default: /* NV12, NV12M */
		iformat = MFDIN_FMT_NV12;
		interp_en = 0;
		canv_bpp = (0 << 6) | (1 << 4) | (1 << 2) | (1 << 0);
		meson_canvas_config(core->canvas, ctx->canvas_ids[6],
				    y_addr, canvas_w,
				    src_height,
				    MESON_CANVAS_WRAP_NONE,
				    MESON_CANVAS_BLKMODE_LINEAR, 0);
		meson_canvas_config(core->canvas, ctx->canvas_ids[7],
				    uv_addr, canvas_w,
				    src_height / 2,
				    MESON_CANVAS_WRAP_NONE,
				    MESON_CANVAS_BLKMODE_LINEAR, 0);
		canvas = ((u32)ctx->canvas_ids[7] << 8) | ctx->canvas_ids[6];
		break;
	}

	/* Program NR registers before MFDIN_REG1_CTRL (BSP order) */
	if (nr_enable)
		hcodec_setup_nr(core, ctx->nr_mode);

	/*
	 * MFDIN configuration (GXL+ path)
	 * bits[3:0]: iformat, bits[7:4]: oformat (0=420)
	 * bits[9]: interp_en, bit[12]: r2y_en, bit[13]: r2y_mode
	 * bits[19]: nr_enable, bits[30:29]: mfdin_mode
	 */
	{
		/*
		 * BSP mfdin_basic() register setup, matching exactly:
		 *
		 * ifmt444 = 1 for iformat 1,5,8,9,12 (RGB24=1, RGBA=12)
		 * dsample_en = 1 when ifmt444 and oformat=420
		 * y_size = 0 for iformat=12, else (oformat != 0) ? 1 : 0
		 * r2y_en = 1 for RGB formats
		 * r2y_mode = 1 when r2y_en=1
		 * linear_enable = true for iformat >= 8
		 */
		bool is_rgba = (iformat == 12);
		bool is_rgb = (iformat == 1 &&
			       ctx->src_fmt.pixelformat == V4L2_PIX_FMT_RGB24);
		u32 r2y_en = (is_rgb || is_rgba) ? 1 : 0;
		u32 ifmt444 = (iformat == 1 || iformat == 5 ||
			       iformat == 8 || iformat == 9 ||
			       iformat == 12) ? 1 : 0;
		/*
		 * Determine output chroma from source pixel format
		 * (same logic as effective_chroma in venc.c):
		 *   YUYV   -> 422 (oformat=2)
		 *   ABGR32 -> 444 (oformat=0 + dsample_en=0)
		 *   other  -> 420 (oformat=0 + dsample_en=1)
		 */
		bool is_jpeg_422 = (ctx->dst_fmt.pixelformat == V4L2_PIX_FMT_JPEG &&
				    ctx->src_fmt.pixelformat == V4L2_PIX_FMT_YUYV);
		bool is_jpeg_444 = (ctx->dst_fmt.pixelformat == V4L2_PIX_FMT_JPEG &&
				    ctx->src_fmt.pixelformat == V4L2_PIX_FMT_ABGR32);
		/*
		 * MFDIN output format (REG1_CTRL bits[7:4]):
		 *   0 = YUV420  -- default for NV12/planar input
		 *   1 = YUV422  -- for YUYV packed input (produces 422 blocks)
		 *   2 = YUV444  -- for RGBA input (produces full-chroma blocks)
		 *
		 * Mapping verified from BSP H264 mfdin_basic() dsample_en logic:
		 *   dsample_en = ((ifmt444 && oformat!=2) || ...) -> oformat=2 means 444
		 * And BSP JPEG T3 path: oformat=2 = JPEGENC_FMT_YUV444_SINGLE.
		 * Test 2 (YUYV->422) uses oformat=1 (422) -- originally set to 2
		 * which also worked but was semantically wrong.
		 */
		/*
		 * MFDIN output format (REG1_CTRL bits[7:4]):
		 *   0 = YUV420  -- default for NV12/planar input
		 *   2 = YUV422  -- for YUYV packed input (confirmed working)
		 *   ? = YUV444  -- TBD; oformat=1 and oformat=0+dsample=0 both
		 *                  hang; oformat=2 conflicts with 422.
		 *
		 * NOTE: BSP for G12B (older SoC path) only uses oformat=0
		 * and oformat=1 for 420/422. YUV444 output from RGBA is not
		 * tested in BSP for this SoC family. The T3/newer BSP uses
		 * oformat=2 for YUV444. We use the same here for symmetry.
		 *
		 * dsample_en: from BSP H264 mfdin_basic():
		 *   dsample_en = ((ifmt444 && oformat!=2) || ...) ? 1 : 0
		 *   -> enabled when 444 input NOT going to 444 output
		 */
		/*
		 * MFDIN output format (REG1_CTRL bits[7:4]).
		 * oformat=2 confirmed for both 422 (YUYV input) and 444 (RGBA
		 * input) -- the MFDIN produces YUV444 samples for iformat=12
		 * with oformat=2, which is correct for JPEG 4:4:4. For 422,
		 * oformat=2 with YUYV produces 444-rate Cb/Cr; the VLC
		 * firmware (pic_format=2, 422 grid) uses every other block.
		 * oformat=1 caused a ~16x slowdown (stall on G12B); using 2
		 * for 444 is consistent with BSP H264 mfdin_basic semantics
		 * (oformat=2 disables dsample_en for ifmt444 input).
		 */
		u32 oformat = 0; /* YUV420 default */

		if (is_jpeg_422 || is_jpeg_444)
			oformat = 2;  /* 422 or 444: produce 444-rate samples */
		/* dsample_en: from BSP H264 mfdin_basic logic */
		u32 dsample_en = (ifmt444 && oformat != 2) ? 1 : 0;
		/*
		 * y_size (bit[8]): BSP forces 0 for iformat=12 (RGBA) in the
		 * JPEG path. However, YUYV (iformat=10) gets y_size=1 when
		 * oformat!=0 and that path works. Test: remove the RGBA
		 * special-case -- use the same y_size logic as all other formats.
		 * If RGBA+oformat=2 still stalls, the issue is elsewhere.
		 */
		u32 y_size = (oformat != 0) ? 1 : 0;

		amvdec_write_dos(core, HCODEC_MFDIN_REG1_CTRL,
				 (iformat << 0) | (oformat << 4) |
				 (dsample_en << 6) | (y_size << 8) |
				 (interp_en << 9) | (r2y_en << 12) |
				 (r2y_en << 13) | /* r2y_mode = r2y_en */
				 (nr_enable << 19) |
				 (ctx->mfdin_mode << 29));

		amvdec_write_dos(core, HCODEC_MFDIN_REG8_DMBL,
				 (picsize_x << 14) | (picsize_y << 0));

		if (iformat >= 8) {
			/* Linear DMA path (iformat 8-12) */
			u32 linear_bytes4p = is_rgba ? 3 : 0;
			u32 linear_bpl = is_rgba ? picsize_x * 4 :
					 picsize_x * linear_bytes4p;

			amvdec_write_dos(core, HCODEC_MFDIN_REG3_CANV,
					 (canv_bpp << 24));
			amvdec_write_dos(core, HCODEC_MFDIN_REG4_LNR0,
					 (linear_bytes4p << 16) | linear_bpl);
			amvdec_write_dos(core, HCODEC_MFDIN_REG5_LNR1,
					 (u32)y_addr);
		} else {
			/* Canvas DMA path (iformat 0-7) */
			amvdec_write_dos(core, HCODEC_MFDIN_REG3_CANV,
					 (canvas & 0xffffff) |
					 (canv_bpp << 24));
			amvdec_write_dos(core, HCODEC_MFDIN_REG4_LNR0, 0);
			amvdec_write_dos(core, HCODEC_MFDIN_REG5_LNR1, 0);
		}

		/*
		 * Endian config (REG9_ENDN): 8-byte swizzle pattern.
		 *
		 * BSP uses special RGBA pattern for iformat=12 because BSP's
		 * FMT_RGBA8888 is R-G-B-A byte order. V4L2's ABGR32 is
		 * B-G-R-A byte order -- the R/B are already swapped relative
		 * to BSP. Using the BSP's RGBA swizzle on BGRA input causes
		 * R/B channel swap (red appears blue). Use the default
		 * big-endian swizzle for BGRA input instead.
		 */
		if (is_rgba)
			amvdec_write_dos(core, HCODEC_MFDIN_REG9_ENDN,
					 (0 << 0) | (1 << 3) | (2 << 6) |
					 (3 << 9) | (4 << 12) | (5 << 15) |
					 (6 << 18) | (7 << 21));
		else
			amvdec_write_dos(core, HCODEC_MFDIN_REG9_ENDN,
					 (7 << 0) | (6 << 3) | (5 << 6) |
					 (4 << 9) | (3 << 12) | (2 << 15) |
					 (1 << 18) | (0 << 21));
	}
}

/*
 * Exp-Golomb / RBSP bitstream writer for SPS/PPS generation
 */
struct bitstream {
	u8 *buf;
	u32 size;
	u32 bit_pos;
};

static void bs_init(struct bitstream *bs, u8 *buf, u32 size)
{
	bs->buf = buf;
	bs->size = size;
	bs->bit_pos = 0;
	memset(buf, 0, size);
}

static void bs_write_bits(struct bitstream *bs, u32 val, u32 nbits)
{
	u32 i;

	for (i = 0; i < nbits; i++) {
		u32 byte_pos = bs->bit_pos / 8;
		u32 bit_off = 7 - (bs->bit_pos % 8);

		if (byte_pos >= bs->size)
			return;
		if (val & (1 << (nbits - 1 - i)))
			bs->buf[byte_pos] |= (1 << bit_off);
		bs->bit_pos++;
	}
}

static void bs_write_ue(struct bitstream *bs, u32 val)
{
	u32 v = val + 1;
	u32 nbits = 0;
	u32 tmp = v;

	while (tmp > 0) {
		nbits++;
		tmp >>= 1;
	}
	/* leading zeros */
	bs_write_bits(bs, 0, nbits - 1);
	bs_write_bits(bs, v, nbits);
}

static void bs_write_se(struct bitstream *bs, s32 val)
{
	if (val > 0)
		bs_write_ue(bs, val * 2 - 1);
	else
		bs_write_ue(bs, -val * 2);
}

static void bs_rbsp_trailing(struct bitstream *bs)
{
	bs_write_bits(bs, 1, 1);
	while (bs->bit_pos % 8)
		bs_write_bits(bs, 0, 1);
}

static u32 bs_byte_count(struct bitstream *bs)
{
	return (bs->bit_pos + 7) / 8;
}

/*
 * Write NAL unit with emulation prevention (0x00 0x00 0x03 stuffing).
 * Returns total bytes written including start code.
 */
static u32 write_nal(u8 *dst, u32 dst_size, u8 nal_header,
		     const u8 *rbsp, u32 rbsp_len)
{
	u32 pos = 0;
	u32 i;
	u32 zero_count = 0;

	if (pos + 4 > dst_size)
		return 0;

	/* Start code */
	dst[pos++] = 0x00;
	dst[pos++] = 0x00;
	dst[pos++] = 0x00;
	dst[pos++] = 0x01;
	dst[pos++] = nal_header;

	for (i = 0; i < rbsp_len && pos < dst_size; i++) {
		if (zero_count == 2 && rbsp[i] <= 0x03) {
			dst[pos++] = 0x03;
			if (pos >= dst_size)
				break;
			zero_count = 0;
		}
		dst[pos++] = rbsp[i];
		if (rbsp[i] == 0x00)
			zero_count++;
		else
			zero_count = 0;
	}

	return pos;
}

/*
 * Generate SPS + PPS NAL units. Returns total bytes written.
 *
 * GXL firmware (gxl_h264_enc.bin) uses CAVLC entropy coding:
 *   profile_idc=66 (Baseline), entropy_coding_mode_flag=0.
 *
 * G12A+ firmware (ga_h264_enc_cabac.bin) uses CABAC entropy coding:
 *   profile_idc=77 (Main), entropy_coding_mode_flag=1.
 *
 * The SW SPS/PPS must match the firmware's entropy coding mode --
 * mismatch causes decoders to misparse slice data (KI-009).
 */
u32 hcodec_build_sps_pps(struct amvdec_core *core,
			  struct amvenc_ctx *ctx,
			  u8 *buf, u32 buf_size)
{
	struct bitstream bs;
	u8 rbsp[64];
	u32 pos = 0;
	u32 len;
	u32 mb_w = (ctx->width + 15) / 16;
	u32 mb_h = (ctx->height + 15) / 16;
	bool cabac = (core->platform->revision >= VDEC_REVISION_G12A);
	u8 profile_idc = cabac ? 77 : 66; /* Main/CABAC on G12A+, Baseline/CAVLC on GXL */
	/* Map V4L2 level enum to H.264 level_idc (spec Table A-1) */
	static const u8 v4l2_to_level_idc[] = {
		10, 9, 11, 12, 13, 20, 21, 22, 30, 31, 32, 40, 41, 42, 50, 51,
	};
	u8 level_idc = (ctx->h264_level < ARRAY_SIZE(v4l2_to_level_idc))
		? v4l2_to_level_idc[ctx->h264_level] : 40;
	u32 crop_bottom = mb_h * 16 - ctx->height;

	/* SPS */
	bs_init(&bs, rbsp, sizeof(rbsp));
	bs_write_bits(&bs, profile_idc, 8);	/* profile_idc */
	bs_write_bits(&bs, 0, 1);		/* constraint_set0 */
	bs_write_bits(&bs, 0, 1);		/* constraint_set1 */
	bs_write_bits(&bs, 0, 1);		/* constraint_set2 */
	bs_write_bits(&bs, 0, 1);		/* constraint_set3 */
	bs_write_bits(&bs, 0, 4);		/* reserved_zero_4bits */
	bs_write_bits(&bs, level_idc, 8);	/* level_idc */
	bs_write_ue(&bs, 0);			/* seq_parameter_set_id */
	bs_write_ue(&bs, 0);			/* log2_max_frame_num_minus4 */
	bs_write_ue(&bs, 0);			/* pic_order_cnt_type */
	/*
	 * log2_max_pic_order_cnt_lsb_minus4 must remain 0 here.  The SW SPS
	 * path does not control slice-header emission -- that comes from the
	 * encoder firmware which hardcodes `pic_order_cnt_lsb` as 4 bits.
	 * Setting any other value in the SPS makes the SPS and the slice
	 * headers inconsistent: ffmpeg's SW H.264 decoder then misaligns
	 * the slice header parse and fails to decode any MB.  VDEC HW
	 * H.264 firmware-parser appears more tolerant but the bitstream is
	 * non-conformant against its own SPS, which is not a defensible
	 * state to ship.  Keep 0 unless the firmware emission is also
	 * patched to match.
	 */
	bs_write_ue(&bs, 0);			/* log2_max_pic_order_cnt_lsb_minus4 */
	/*
	 * max_num_ref_frames = 4.  Was 1 (encoder operates with a single ref
	 * regardless), but VDEC HW H.264 DPB-management refuses 13-byte SPS
	 * advertising only 1 ref slot.  Advertising 4 (max allowed at L4.0 for
	 * 1920x1080) does not constrain encoder behavior but satisfies the
	 * decoder-side DPB sanity gate.
	 */
	bs_write_ue(&bs, 4);			/* max_num_ref_frames */
	bs_write_bits(&bs, 0, 1);		/* gaps_in_frame_num */
	bs_write_ue(&bs, mb_w - 1);		/* pic_width_in_mbs_minus1 */
	bs_write_ue(&bs, mb_h - 1);		/* pic_height_in_map_units_minus1 */
	bs_write_bits(&bs, 1, 1);		/* frame_mbs_only_flag */
	/* no mb_adaptive_frame_field since frame_mbs_only=1 */
	/*
	 * direct_8x8_inference_flag = 1.  H.264 spec mandates this is 1 when
	 * level_idc >= 30 (i.e., 3.0 and above) and frame_mbs_only_flag=1.
	 * Our level_idc is mapped from V4L2_CID_MPEG_VIDEO_H264_LEVEL above
	 * which defaults to L4.0 (40) for the 1280x720+ resolutions we target,
	 * so the previous 0 was invalid per spec.  ffmpeg's SW H.264 decoder
	 * accepts the invalid 0; VDEC HW H.264 firmware-parser rejects it
	 * silently (no SOURCE_CHANGE event delivered to userspace despite the
	 * h264_src_change IRQ firing kernel-side).
	 */
	bs_write_bits(&bs, 1, 1);		/* direct_8x8_inference_flag */

	if (crop_bottom) {
		bs_write_bits(&bs, 1, 1);	/* frame_cropping_flag */
		bs_write_ue(&bs, 0);		/* crop_left */
		bs_write_ue(&bs, 0);		/* crop_right */
		bs_write_ue(&bs, 0);		/* crop_top */
		bs_write_ue(&bs, crop_bottom / 2); /* crop_bottom (in frame units) */
	} else {
		bs_write_bits(&bs, 0, 1);	/* frame_cropping_flag */
	}

	bs_write_bits(&bs, 0, 1);		/* vui_parameters_present */
	bs_rbsp_trailing(&bs);

	/* nal_ref_idc=3, nal_unit_type=7 (SPS) */
	len = write_nal(buf + pos, buf_size - pos, 0x67,
			rbsp, bs_byte_count(&bs));
	pos += len;

	/* PPS */
	bs_init(&bs, rbsp, sizeof(rbsp));
	bs_write_ue(&bs, 0);			/* pic_parameter_set_id */
	bs_write_ue(&bs, 0);			/* seq_parameter_set_id */
	bs_write_bits(&bs, cabac ? 1 : 0, 1);	/* entropy_coding_mode_flag */
	bs_write_bits(&bs, 0, 1);		/* bottom_field_pic_order */
	bs_write_ue(&bs, 0);			/* num_slice_groups_minus1 */
	bs_write_ue(&bs, 0);			/* num_ref_idx_l0_minus1 */
	bs_write_ue(&bs, 0);			/* num_ref_idx_l1_minus1 */
	bs_write_bits(&bs, 0, 1);		/* weighted_pred_flag */
	bs_write_bits(&bs, 0, 2);		/* weighted_bipred_idc */
	/*
	 * pic_init_qp_minus26 must be 0 (pic_init_qp = 26) to match
	 * QPPICTURE which is always 26.  The firmware computes
	 * slice_qp_delta = actual_qp - QPPICTURE, so the decoder needs
	 * pic_init_qp = QPPICTURE = 26 to reconstruct the correct QP.
	 */
	bs_write_se(&bs, 0);			/* pic_init_qp_minus26 */
	bs_write_se(&bs, 0);			/* pic_init_qs_minus26 */
	bs_write_se(&bs, 0);			/* chroma_qp_index_offset */
	bs_write_bits(&bs, 0, 1);		/* deblocking_filter_control */
	bs_write_bits(&bs, 0, 1);		/* constrained_intra_pred */
	bs_write_bits(&bs, 0, 1);		/* redundant_pic_cnt */
	bs_rbsp_trailing(&bs);

	/* nal_ref_idc=3, nal_unit_type=8 (PPS) */
	len = write_nal(buf + pos, buf_size - pos, 0x68,
			rbsp, bs_byte_count(&bs));
	pos += len;

	return pos;
}

/*
 * Generate SPS/PPS via firmware and cache in ctx.
 * The firmware generates bitstream-compatible SPS/PPS that match
 * the slice headers it produces. This avoids parameter mismatches
 * that cause decode errors.
 *
 * Protocol: send ENCODER_SEQUENCE (1) → firmware generates SPS,
 * signals SEQUENCE_DONE (7). Then send ENCODER_PICTURE (2) →
 * firmware generates PPS, signals PICTURE_DONE (8).
 */
static int hcodec_fw_poll_status(struct amvdec_core *core, u32 expected,
				 int timeout_ms)
{
	int i;

	for (i = 0; i < timeout_ms; i++) {
		if (amvdec_read_dos(core, ENCODER_STATUS) == expected)
			return 0;
		usleep_range(1000, 1100);
	}
	return -ETIMEDOUT;
}

int hcodec_fw_generate_sps_pps(struct amvdec_core *core,
			       struct amvenc_ctx *ctx)
{
	u32 sps_size, total_size;
	u8 *bs_vaddr = ctx->sub_vaddr[HCODEC_SUB_BS];

	/* Set up encoder for SPS/PPS generation (use workspace bs buffer) */
	hcodec_stop(core);
	hcodec_reset(core);
	hcodec_setup_canvases(core, ctx);
	hcodec_setup_encoder(core, ctx, true);
	hcodec_setup_buffers(core, ctx,
			     ctx->sub_paddr[HCODEC_SUB_BS],
			     ctx->buf_layout.bs_size);
	hcodec_setup_ie_me(core, ctx, ctx->qp, true);

	/*
	 * Send ENCODER_SEQUENCE (1) → firmware generates SPS.
	 * Keep firmware running for the subsequent PICTURE command.
	 * BSP sends both commands without stopping the AMRISC.
	 */
	amvdec_write_dos(core, ENCODER_STATUS, ENCODER_SEQUENCE);
	hcodec_start(core);

	if (hcodec_fw_poll_status(core, ENCODER_SEQUENCE_DONE, 500)) {
		dev_warn(core->dev, "HCODEC: SPS generation timeout\n");
		hcodec_stop(core);
		return -ETIMEDOUT;
	}

	sps_size = amvdec_read_dos(core, HCODEC_VLC_TOTAL_BYTES);

	if (sps_size == 0 || sps_size > 64) {
		dev_warn(core->dev, "HCODEC: bad SPS size %u\n", sps_size);
		hcodec_stop(core);
		return -EIO;
	}

	/*
	 * Send ENCODER_PICTURE (2) → firmware generates PPS.
	 * Firmware is still running; its main loop will pick up the new
	 * command. VLC_TOTAL_BYTES accumulates (SPS + PPS).
	 */
	amvdec_write_dos(core, ENCODER_STATUS, ENCODER_PICTURE);

	if (hcodec_fw_poll_status(core, ENCODER_PICTURE_DONE, 500)) {
		dev_warn(core->dev, "HCODEC: PPS generation timeout\n");
		hcodec_stop(core);
		return -ETIMEDOUT;
	}

	total_size = amvdec_read_dos(core, HCODEC_VLC_TOTAL_BYTES);
	hcodec_stop(core);

	if (total_size <= sps_size || total_size > sizeof(ctx->sps_pps)) {
		dev_warn(core->dev, "HCODEC: bad PPS size %u (sps=%u)\n",
			 total_size - sps_size, sps_size);
		return -EIO;
	}

	/* Copy SPS+PPS from bitstream buffer */
	memcpy(ctx->sps_pps, bs_vaddr, total_size);
	ctx->sps_pps_len = total_size;

	dev_dbg(core->dev, "HCODEC: firmware SPS/PPS: %u bytes (SPS=%u PPS=%u)\n",
		total_size, sps_size, total_size - sps_size);

	return 0;
}

/*
 * Update quantization tables to match current QP.
 * The QDCT hardware uses these tables for the actual quantization step,
 * and the BSP derives i_pic_qp from quant_tbl_i4[0] & 0xff to set the
 * VLC quant registers.  All three tables (i4, i16, me) must contain the
 * same byte-packed QP value as used in QPPICTURE and the CBR table.
 *
 * Without this, changing ctx->qp (e.g. via rate control) creates a
 * mismatch: QDCT quantizes at the old table QP but the VLC encodes
 * slice headers with the new QP, causing the decoder to apply wrong
 * inverse quantization and producing corrupted output (~11 dB PSNR).
 *
 * BSP encoder.c lines 3384-3389: memset(quant_tbl_*, quant, sizeof(...))
 */
static void hcodec_update_quant_tables(struct amvenc_ctx *ctx, u32 qp)
{
	u32 qp_packed;

	qp = min_t(u32, qp, 51);
	qp_packed = qp | (qp << 8) | (qp << 16) | (qp << 24);

	memset(ctx->quant_tbl_i4, qp, sizeof(ctx->quant_tbl_i4));
	memset(ctx->quant_tbl_i16, qp, sizeof(ctx->quant_tbl_i16));
	memset(ctx->quant_tbl_me, qp, sizeof(ctx->quant_tbl_me));
}

/*
 * Rate control: compute CBR block dimensions.
 * Aims for ~256 total blocks (matching firmware's target bits table size).
 */
void hcodec_compute_block_dims(struct amvenc_ctx *ctx)
{
	u32 mb_w = (ctx->width + 15) / 16;
	u32 mb_h = (ctx->height + 15) / 16;
	u32 bw_n, bh_n;

	if (!mb_w || !mb_h) {
		ctx->block_w = 1;
		ctx->block_h = 1;
		ctx->block_w_n = mb_w;
		ctx->block_h_n = mb_h;
		return;
	}

	bw_n = int_sqrt(CBR_TARGET_BITS_TBL_SZ * mb_w / mb_h);
	bh_n = int_sqrt(CBR_TARGET_BITS_TBL_SZ * mb_h / mb_w);
	if (!bw_n)
		bw_n = 1;
	if (!bh_n)
		bh_n = 1;

	ctx->block_w = DIV_ROUND_UP(mb_w, bw_n);
	ctx->block_h = DIV_ROUND_UP(mb_h, bh_n);
	ctx->block_w_n = DIV_ROUND_UP(mb_w, ctx->block_w);
	ctx->block_h_n = DIV_ROUND_UP(mb_h, ctx->block_h);
}

/*
 * Fill the CBR QP lookup table and per-block target sizes in DMA buffer.
 * Layout: 16 entries × 128 bytes (QP tables) at offset 0,
 *         per-block uint16_t targets at offset 0x800.
 */
static void hcodec_fill_cbr_table(struct amvenc_ctx *ctx)
{
	void *cbr_base = ctx->sub_vaddr[HCODEC_SUB_CBR];
	u8 *tbl;
	u16 *block_tbl;
	u32 qp_base, qp_val;
	u32 total_blocks, target_per_block;
	int i, j;

	memset(cbr_base, 0, HCODEC_CBR_INFO_SIZE);

	/*
	 * Build 16-entry QP ladder centered on ctx->qp at entry
	 * CBR_START_TBL_ID (8).  Each entry ±1 from center = ±1 QP.
	 * Firmware walks the table: lower IDs = lower QP (more bits),
	 * higher IDs = higher QP (fewer bits).
	 */
	for (i = 0; i < 16; i++) {
		int qp_signed = (int)ctx->qp + (i - CBR_START_TBL_ID);

		tbl = (u8 *)cbr_base + i * CBR_TABLE_ENTRY_SIZE;

		qp_val = (u32)clamp(qp_signed, 0, 51);
		/* Pack QP into all 4 bytes of each u32 */
		qp_val = qp_val | (qp_val << 8) |
			 (qp_val << 16) | (qp_val << 24);

		/* i4 QP table (bytes 0-31) */
		for (j = 0; j < 8; j++)
			((u32 *)tbl)[j] = qp_val;
		/* i16 QP table (bytes 32-63) */
		for (j = 0; j < 8; j++)
			((u32 *)(tbl + 32))[j] = qp_val;
		/* ME QP table (bytes 64-95) */
		for (j = 0; j < 8; j++)
			((u32 *)(tbl + 64))[j] = qp_val;

		/* SAD weights at bytes 96-107 */
		((u16 *)(tbl + 96))[0] = ctx->i4mb_weight;
		((u16 *)(tbl + 96))[1] = ctx->i16mb_weight;
		((u16 *)(tbl + 96))[2] = ctx->me_weight;
		((u16 *)(tbl + 96))[3] = ctx->i4mb_weight + 0x20;
		((u16 *)(tbl + 96))[4] = ctx->i16mb_weight + 0x10;
		((u16 *)(tbl + 96))[5] = ctx->me_weight + 0x10;
		/* Magic markers at bytes 108-111 */
		((u16 *)(tbl + 108))[0] = CBR_MAGIC_0;
		((u16 *)(tbl + 108))[1] = CBR_MAGIC_1;
	}

	/* Per-block target sizes at offset 0x800 (uniform distribution) */
	block_tbl = (u16 *)((u8 *)cbr_base + HCODEC_CBR_TABLE_SIZE);
	total_blocks = (u32)ctx->block_w_n * ctx->block_h_n;
	if (total_blocks && ctx->target_bits)
		target_per_block = (ctx->target_bits * 32) / total_blocks;
	else
		target_per_block = 0;

	/*
	 * GXL (CAVLC gxl_h264_enc.bin) even-MB-width workaround.  When the
	 * frame width in macroblocks is EVEN and the per-block CBR targets
	 * are non-zero, the GXL firmware's per-block long-term rate
	 * adjustment hard-wedges the VLC/QDCT/MC reconstruction pipeline
	 * mid-frame: VLC_MB_INFO freezes ~2/3 down (e.g. mb=(62,28) of
	 * (79,44) at 1280x720), ENCODER_STATUS stays at the "5" preamble
	 * marker, and every frame hits "encode timeout status=0x5".
	 * A/B-proven on aml-s905x-cc-v2: zeroing the per-block targets
	 * (identical to frame_level_rate_control_enable=0) lets even mb_w
	 * complete; odd mb_w and the G12A+ CABAC firmware are unaffected
	 * (they never wedge).  The AMRISC firmware is a black box, so fall
	 * back to frame-level rate control (hcodec_rc_update adjusts the
	 * picture QP per frame) for this case -- GXL per-block CBR is a
	 * coarse, marginal feature anyway (KI-111).  See
	 * encoder/hcodec/todo.md "GXL H.264 encode status=0x5".
	 */
	if (hcodec_gxl_even_mbw(ctx))
		target_per_block = 0;

	for (i = 0; i < (int)total_blocks && i < CBR_TARGET_BITS_TBL_SZ; i++)
		block_tbl[i] = min_t(u32, target_per_block, 0xffff);
}

/*
 * Initialize rate control state for a new stream.
 */
void hcodec_rc_init(struct amvenc_ctx *ctx, u32 fps)
{
	u32 pixels, bpp_x1000, qp;

	if (!fps)
		fps = 30;

	if (ctx->rc_enable && ctx->bitrate) {
		ctx->target_bits = ctx->bitrate / fps;
		ctx->last_frame_bits = 0;

		/*
		 * Estimate initial QP from target bits-per-pixel.
		 * Empirical mapping: ~0.10 bpp -> QP 26 for H.264 CAVLC.
		 * Each +6 QP roughly halves bitrate.
		 */
		pixels = ctx->width * ctx->height;
		if (pixels && ctx->target_bits) {
			/*
			 * Estimate initial QP from target bpp.  The CBR
			 * table spans ±8 QP around this center.  Firmware
			 * walks tbl_id 0-15 to adjust.  Start low (high
			 * quality) to give firmware room to increase QP.
			 */
			bpp_x1000 = (u32)div_u64(
				(u64)ctx->target_bits * 1000, pixels);
			if (bpp_x1000 >= 2000)
				qp = 2;
			else if (bpp_x1000 >= 1000)
				qp = 6;
			else if (bpp_x1000 >= 400)
				qp = 10;
			else if (bpp_x1000 >= 200)
				qp = 16;
			else if (bpp_x1000 >= 100)
				qp = 22;
			else if (bpp_x1000 >= 50)
				qp = 30;
			else if (bpp_x1000 >= 25)
				qp = 38;
			else
				qp = 44;
			ctx->qp = clamp_t(u32, qp, 0,
					  ctx->max_qp ? ctx->max_qp : 51);
		}
	} else {
		ctx->target_bits = 0;
	}

	/* GXL even-mb_w: keep QP out of the pipeline-wedge range (see above). */
	if (hcodec_gxl_even_mbw(ctx) && ctx->qp < HCODEC_GXL_EVEN_QP_FLOOR)
		ctx->qp = HCODEC_GXL_EVEN_QP_FLOOR;

	/*
	 * Sync quant tables to current QP.  The QDCT hardware reads these
	 * tables for quantization, and the VLC engine reads QP from
	 * HCODEC_QDCT_VLC_QUANT_CTL_0 (derived from quant_tbl values in
	 * BSP).  Both must agree or the decoder sees a QP mismatch.
	 */
	hcodec_update_quant_tables(ctx, ctx->qp);

	/*
	 * Always fill the CBR table, even without rate control.
	 * The firmware reads this table for per-block QP values.
	 * In constant-QP mode, all entries contain the same QP.
	 */
	hcodec_fill_cbr_table(ctx);
}

/*
 * Frame-level QP adjustment based on actual vs target bits.
 * Called before each frame encode. Updates ctx->qp in place and
 * re-syncs all QP-dependent state (quant tables + CBR table) so
 * that QPPICTURE, QDCT quant registers, and CBR table all agree.
 *
 * Without the CBR table re-sync, the firmware reads stale per-block
 * QP values from the CBR table while QPPICTURE and the QDCT quant
 * registers use the new QP. This mismatch causes the VLC engine to
 * encode slice headers with a slice_qp_delta inconsistent with the
 * actual quantization, producing ~11 dB PSNR output.
 */
void hcodec_rc_get_qp(struct amvenc_ctx *ctx, bool idr)
{
	if (!ctx->rc_enable || !ctx->target_bits)
		return;

	/*
	 * Firmware RE (2026-04-06): the AMRISC firmware manages CBR
	 * autonomously.  It overwrites the driver's quant table SRAM
	 * from the CBR DMA buffer (hcodec_fill_cbr_table entry selected
	 * by tbl_id) and adjusts tbl_id based on actual vs target bytes.
	 *
	 * The driver must NOT rebuild the CBR table per-frame or adjust
	 * ctx->qp -- doing so resets the firmware's tbl_id progress and
	 * causes oscillation (QP: 14→10→6→2→51→47... repeating).
	 *
	 * The driver's role is:
	 * 1. Fill the CBR table ONCE at stream start (hcodec_rc_init)
	 * 2. Set per-block target sizes in the MB size table
	 * 3. Let the firmware walk tbl_id 0-15 autonomously
	 *
	 * IDR QP adjustment: lower QP for IDR (4x budget per BSP).
	 * This only affects QDCT_Q_QUANT_I (slice header QP) and
	 * quant tables for the initial hardware setup before firmware
	 * takes over via CBR table.
	 */
	if (idr && ctx->qp > 4) {
		u32 idr_qp = ctx->qp - 4;

		/*
		 * GXL even-mb_w: the -4 IDR budget pushes the QP into the
		 * pipeline-wedge range; hold it at the working-range floor.
		 */
		if (hcodec_gxl_even_mbw(ctx) && idr_qp < HCODEC_GXL_EVEN_QP_FLOOR)
			idr_qp = HCODEC_GXL_EVEN_QP_FLOOR;
		ctx->qp = clamp_t(u32, idr_qp, 0, 51);
		hcodec_update_quant_tables(ctx, ctx->qp);
	}
}

void hcodec_rc_update(struct amvenc_ctx *ctx, u32 frame_bits)
{
	if (!ctx->rc_enable || !ctx->target_bits)
		return;

	ctx->last_frame_bits = frame_bits;

	/*
	 * Firmware RE (2026-04-06): do NOT adjust ctx->qp or rebuild
	 * the CBR table here.  The firmware's CBR mechanism handles
	 * rate control by walking tbl_id within the 16-entry CBR table.
	 * Driver-side QP adjustment conflicts with firmware CBR and
	 * causes oscillation.  See KI-006 in status.md.
	 */
}

void hcodec_encode(struct amvdec_core *core,
		   struct amvenc_ctx *ctx, bool idr,
		   dma_addr_t y_addr, dma_addr_t uv_addr,
		   dma_addr_t v_addr, u32 canvas_w,
		   dma_addr_t vlc_dst, u32 vlc_dst_size)
{
	/*
	 * Per-frame reset sequence matching BSP amvenc_avc_start_cmd():
	 *   1. Stop AMRISC (MPSR=0, CPSR=0, wait DMA idle)
	 *   2. Targeted HCODEC reset (HCODEC_RESET_FULL bits only)
	 *   3. Set up registers
	 *   4. MC reset + MPSR=1 to start
	 *
	 * The BSP reload_mc() (DOS_SW_RESET1=0xffffffff + firmware reload) is
	 * only called when switching firmware variants (ucode_mode change), NOT
	 * on every frame.  Per-frame full DOS reset destroys the SRAM state
	 * that the GXL CAVLC firmware needs for P-frame motion estimation and
	 * entropy coding continuity (KI-074).  Firmware is loaded once at
	 * session init via hcodec_load_firmware() and preserved across frames.
	 *
	 * BSP reference: encoder.c amvenc_avc_start_cmd(), amvenc_reset().
	 * The per-frame path uses amvenc_reset() (HCODEC_RESET_FULL bits only),
	 * then amvenc_start() (HCODEC_RESET_MC bits) after register setup.
	 */
	hcodec_stop(core);
	hcodec_reset(core);

	/* H.264 uses 4x4 block mode for MFDIN */
	ctx->mfdin_mode = 0;

	hcodec_setup_canvases(core, ctx);
	hcodec_setup_encoder(core, ctx, idr);
	hcodec_setup_buffers(core, ctx, vlc_dst, vlc_dst_size);

	hcodec_setup_ie_me(core, ctx, ctx->qp, idr);

	/* SVC pic type - firmware requires this on GA (G12A/G12B/SM1) */
	amvdec_write_dos(core, H264_ENC_SVC_PIC_TYPE, ENC_SLC_REF);

	/* Set dblk/ref canvases */
	dev_dbg(core->dev, "HCODEC canvas: idr=%d ref=0x%06x dblk=0x%06x\n",
		idr, ctx->ref_canvas, ctx->dblk_canvas);
	amvdec_write_dos(core, HCODEC_ANC0_CANVAS_ADDR, ctx->ref_canvas);
	amvdec_write_dos(core, HCODEC_VLC_HCMD_CONFIG, 0);
	amvdec_write_dos(core, HCODEC_REC_CANVAS_ADDR, ctx->dblk_canvas);
	amvdec_write_dos(core, HCODEC_DBKR_CANVAS_ADDR, ctx->dblk_canvas);
	amvdec_write_dos(core, HCODEC_DBKW_CANVAS_ADDR, ctx->dblk_canvas);

	/* Set up input */
	hcodec_setup_mfdin(core, ctx, y_addr, uv_addr, v_addr, canvas_w);

	/*
	 * Always program CBR registers -- the firmware reads the CBR table
	 * for per-block QP values even in constant-QP mode.  Without a
	 * valid table address, the firmware uses whatever stale value is in
	 * the register, causing non-deterministic quantization.
	 *
	 * When rate control is off, the table contains the constant QP for
	 * all blocks and CBR_SHORT_SHIFT=12 disables dynamic adjustment.
	 *
	 * Also clear the SAD shift bits in HCODEC_SAD_CONTROL_1 per BSP.
	 */
	{
		dma_addr_t cbr_base = ctx->sub_paddr[HCODEC_SUB_CBR];
		u32 sad_ctl;

		sad_ctl = amvdec_read_dos(core, HCODEC_SAD_CONTROL_1);
		sad_ctl &= 0xffff;
		amvdec_write_dos(core, HCODEC_SAD_CONTROL_1, sad_ctl);

		amvdec_write_dos(core, H264_ENC_CBR_TABLE_ADDR, cbr_base);
		amvdec_write_dos(core, H264_ENC_CBR_MB_SIZE_ADDR,
				 cbr_base + HCODEC_CBR_TABLE_SIZE);
		amvdec_write_dos(core, H264_ENC_CBR_CTL,
				 (CBR_START_TBL_ID << 28) |
				 (CBR_SHORT_SHIFT << 24) |
				 (CBR_LONG_MB_NUM << 16) |
				 (CBR_LONG_TH << 0));
		amvdec_write_dos(core, H264_ENC_CBR_REGION_SIZE,
				 ((u32)ctx->block_w << 16) |
				 ((u32)ctx->block_h << 0));
	}

	/* Issue encode command and start AMRISC */
	amvdec_write_dos(core, ENCODER_STATUS,
			 idr ? ENCODER_IDR : ENCODER_NON_IDR);

	hcodec_start(core);
}
