// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018 Maxime Jourdan <maxi.jourdan@wanadoo.fr>
 *
 * VDEC_HEVC is a video decoding block that allows decoding of
 * HEVC, VP9
 */

#include <linux/firmware.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/reset.h>
#include <linux/workqueue.h>
#include <linux/seq_file.h>

#include "vdec_1.h"
#include "vdec_helpers.h"
#include "vdec_hevc.h"
#include "hevc_regs.h"
#include "dos_regs.h"

/* AO Registers */
#define AO_RTI_GEN_PWR_SLEEP0	0xe8
	/* SLEEP0 bit definitions (power on/off) */
	#define GEN_PWR_VDEC_1 (BIT(3) | BIT(2))
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

/* IMEM size: maximum firmware code that fits in the HEVC IMEM via DMA */
#define MC_SIZE		(4096 * 4)

/* Firmware swap code support (BSP: SWAP_HEVC_UCODE).
 * The BSP HEVC firmware (40KB) exceeds the 16KB IMEM DMA limit.
 * The first 16KB is DMA'd to IMEM.  The full firmware blob is kept in
 * a persistent DMA buffer.  HEVC_STREAM_SWAP_BUFFER2 must point to
 * the swap code portion at offset SWAP_HEVC_OFFSET (12288) within
 * the buffer -- the firmware reads swap segments directly from this
 * address without adding any internal offset.
 * BSP: vh265.c copies fw->data+SWAP_HEVC_OFFSET into mc_cpu_addr,
 * then writes mc_dma_handle (pointing at the copy) to SCRATCH_8.
 */
#define SWAP_HEVC_OFFSET	(3 * 0x1000)	/* 12288 bytes */
#define HEVC_STREAM_SWAP_BUFFER2 HEVC_ASSIST_SCRATCH_8

/* Per-session firmware DMA buffer (persists while firmware is running).
 * BSP keeps the full firmware in DMA memory for swap code access.
 * The upstream 16KB firmware needs no swap, but BSP firmware (36KB) does.
 */
static void *fw_vaddr;
static dma_addr_t fw_paddr;
static size_t fw_dma_size;

/*
 * HEVC decoder turbo clock -- DISABLED.
 *
 * Same hardware timing limit as VDEC_1: the decoder IP blocks cannot
 * operate above 800 MHz.  VDEC_1 at 1 GHz stalls at MBY_MBX=0 on
 * G12B (tested 2026-04-20).  HEVC is assumed to have the same limit
 * since both are decoder blocks on the same die, sharing the same
 * power/clock domain architecture.  The HCODEC encoder block CAN run
 * at 1 GHz (different IP, simpler pipeline).
 *
 * Param retained read-only for script compatibility.
 */
static unsigned int hevc_turbo;
module_param(hevc_turbo, uint, 0444);
MODULE_PARM_DESC(hevc_turbo,
		 "HEVC decoder turbo: DISABLED (HW limit, 800MHz max)");

static const unsigned long hevc_turbo_rates[] = {
	800000000,	/* 0: 800 MHz max for HEVC decode HW */
};

#define HEVC_TURBO_MAX (ARRAY_SIZE(hevc_turbo_rates) - 1)

/*
 * hevc_target_rate - effective HEVC clock rate
 *
 * Returns turbo_rate when not thermally throttled (state 0), otherwise
 * returns the thermal rate.  Thermal ALWAYS wins over turbo for safety.
 */
static unsigned long hevc_target_rate(struct amvdec_core *core)
{
	unsigned long thermal_rate = amvdec_clock_rate(core);
	unsigned int level = min_t(unsigned int, hevc_turbo, HEVC_TURBO_MAX);
	unsigned long turbo_rate = hevc_turbo_rates[level];

	if (thermal_rate < 800000000)
		return thermal_rate;
	return turbo_rate;
}

/*
 * HEVC idle autosuspend.
 *
 * Instead of powering off immediately at STREAMOFF, schedule a delayed
 * poweroff.  If another STREAMON arrives before the delay expires, cancel
 * the work and skip the power cycle entirely.
 *
 * Default idle timeout: 100ms. Settable via module param for tuning.
 */
static unsigned int hevc_idle_ms = 100;
module_param(hevc_idle_ms, uint, 0644);
MODULE_PARM_DESC(hevc_idle_ms,
		 "HEVC idle poweroff delay in ms (default=100, 0=immediate)");

/* KI-118 block manager (defined in hcodec.c, same meson-vdec-core module):
 * gates the HEVC-decode-side wait for a clean power-cycle when the HEVC engine
 * is currently owned by the HCODEC pin (encode-first). */
extern bool ki118_block_manager;

/*
 * KI-118a: the DOS_SW_RESET3 mask for an HEVC-engine reset.
 *
 * G12A+: narrow to the HEVC sub-blocks only (DOS_SW_RESET3_HEVC_ALL = bits[19:2]
 * per the BSP).  0xffffffff also resets bits that belong to other DOS blocks --
 * including the VDEC_1/MDEC-shared state that lives in the HEVC power domain on
 * G12A+ -- so a full reset clobbers a concurrent VDEC_1 decode (the KI-118a
 * 3-way wedge).
 *
 * GXL/GXM: the HEVC and VDEC_1 power domains are SEPARATE (no MDEC-cache
 * coupling), so the full reset does not clobber VDEC_1; and the GXL HEVC engine
 * needs the broader reset -- the narrow G12A+ mask wedges VP9 decode on GXL.
 */
static inline u32 hevc_reset3_mask(struct amvdec_core *core)
{
	return core->platform->revision >= VDEC_REVISION_G12A ?
		DOS_SW_RESET3_HEVC_ALL : 0xffffffff;
}

static int vdec_hevc_load_firmware(struct amvdec_session *sess,
				   const char *fwname)
{
	struct amvdec_core *core = sess->core;
	struct device *dev = core->dev_dec;
	const struct firmware *fw;
	const struct amvdec_fw_version *match;
	int ret;
	u32 i = 100;

	ret = request_firmware(&fw, fwname, dev);
	if (ret < 0)  {
		dev_err(dev, "Unable to request firmware %s\n", fwname);
		return ret;
	}

	if (fw->size < MC_SIZE) {
		dev_err(dev, "Firmware size %zu is too small. Expected %u.\n",
			fw->size, MC_SIZE);
		ret = -EINVAL;
		goto release_firmware;
	}

	/* Free previous firmware DMA buffer if any */
	if (fw_vaddr) {
		dma_free_coherent(core->dev, fw_dma_size,
				  fw_vaddr, fw_paddr);
		fw_vaddr = NULL;
	}

	/* Allocate DMA buffer for the full firmware.
	 * BSP: mc_cpu_addr = dma_alloc_coherent(fw->size) and keeps it
	 * alive for the entire decode session.  The base address is
	 * written to HEVC_STREAM_SWAP_BUFFER2 so the firmware can read
	 * swap code segments from offset 12288 within this buffer.
	 */
	fw_dma_size = fw->size;
	fw_vaddr = dma_alloc_coherent(core->dev, fw_dma_size,
				      &fw_paddr, GFP_KERNEL);
	if (!fw_vaddr) {
		ret = -ENOMEM;
		goto release_firmware;
	}

	memcpy(fw_vaddr, fw->data, fw->size);

	/* Identify firmware by SHA256 and propagate match/quirks to session.
	 * Refuses to load unrecognized firmware when a codec is wired (has
	 * a fw_versions[] table).  Unwired codecs (VP9 until wired) log
	 * identity and continue.  Must be done BEFORE any DOS register writes
	 * so quirk-gated setup can inspect sess->fw_match.
	 */
	match = amvdec_fw_identify(dev, fwname, fw->data, fw->size,
				   sess->fmt_out->fw_versions,
				   sess->fmt_out->fw_versions_count);
	if (IS_ERR(match)) {
		ret = PTR_ERR(match);
		goto free_fw_dma;
	}
	sess->fw_match = match;

	amvdec_write_dos(core, HEVC_MPSR, 0);
	amvdec_write_dos(core, HEVC_CPSR, 0);

	/* DMA the first 16KB (MC_SIZE) to IMEM */
	amvdec_write_dos(core, HEVC_IMEM_DMA_ADR, fw_paddr);
	amvdec_write_dos(core, HEVC_IMEM_DMA_COUNT, MC_SIZE / 4);
	amvdec_write_dos(core, HEVC_IMEM_DMA_CTRL, IMEM_DMA_CTRL_RUN);

	while (i && (readl(core->dos_base + HEVC_IMEM_DMA_CTRL) &
		     IMEM_DMA_CTRL_START))
		i--;

	if (i == 0) {
		dev_err(dev, "Firmware load fail (DMA hang?)\n");
		ret = -ENODEV;
		goto free_fw_dma;
	}

	sess->fw_size = fw->size;
	dev_info(dev, "Firmware %s loaded (%zu bytes, swap=%s)\n",
		 fwname, fw->size,
		 fw->size > MC_SIZE ? "yes" : "no");

	release_firmware(fw);
	return 0;

free_fw_dma:
	dma_free_coherent(core->dev, fw_dma_size, fw_vaddr, fw_paddr);
	fw_vaddr = NULL;
release_firmware:
	release_firmware(fw);
	return ret;
}

static void vdec_hevc_stbuf_init(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;

	amvdec_write_dos(core, HEVC_STREAM_CONTROL,
			 amvdec_read_dos(core, HEVC_STREAM_CONTROL) &
			 ~HEVC_STREAM_CTRL_ENABLE);
	amvdec_write_dos(core, HEVC_STREAM_START_ADDR, sess->vififo_paddr);
	amvdec_write_dos(core, HEVC_STREAM_END_ADDR,
			 sess->vififo_paddr + sess->vififo_size);
	amvdec_write_dos(core, HEVC_STREAM_RD_PTR, sess->vififo_paddr);
	amvdec_write_dos(core, HEVC_STREAM_WR_PTR, sess->vififo_paddr);
}

/* VDEC_HEVC specific ESPARSER configuration */
static void vdec_hevc_conf_esparser(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;

	/* set vififo_vbuf_rp_sel=>vdec_hevc */
	amvdec_write_dos(core, DOS_GEN_CTRL0, DOS_GEN_CTRL0_HEVC);
	amvdec_write_dos(core, HEVC_STREAM_CONTROL,
			 amvdec_read_dos(core, HEVC_STREAM_CONTROL) |
			 HEVC_STREAM_CTRL_USE_PARSER);
	amvdec_write_dos(core, HEVC_STREAM_CONTROL,
			 amvdec_read_dos(core, HEVC_STREAM_CONTROL) |
			 HEVC_STREAM_CTRL_ENABLE);
	amvdec_write_dos(core, HEVC_STREAM_FIFO_CTL,
			 amvdec_read_dos(core, HEVC_STREAM_FIFO_CTL) |
			 HEVC_STREAM_FIFO_CTL_FILL_EN);
}

static u32 vdec_hevc_vififo_level(struct amvdec_session *sess)
{
	return readl_relaxed(sess->core->dos_base + HEVC_STREAM_LEVEL);
}

/*
 * Halt the HEVC-engine AMRISC firmware: disable MBOX1 IRQ, clear MPSR/CPSR,
 * wait for IMEM/LMEM DMA + pending LMEM writes to drain.  Mirrors BSP
 * amhevc_stop() (bsp/.../amvdec.c:1204).  Used by __vdec_hevc_stop at
 * STREAMOFF and by codec drain_finalize hooks at drain so the firmware
 * does not keep spinning in its VIFIFO-search loop post-EOS.  Cutting power
 * or starting the next engine while DMA is in flight latches bad state on
 * the shared DOS bus that persists until reboot.
 */
void amvdec_hevc_halt_firmware(struct amvdec_core *core)
{
	int i;

	/* Disable interrupt */
	amvdec_write_dos(core, HEVC_ASSIST_MBOX1_MASK, 0);
	/* Disable firmware processor */
	amvdec_write_dos(core, HEVC_MPSR, 0);
	amvdec_write_dos(core, HEVC_CPSR, 0);

	/* Wait for IMEM DMA to complete (BSP: amhevc_stop) */
	i = 1000;
	while ((amvdec_read_dos(core, HEVC_IMEM_DMA_CTRL) &
		IMEM_DMA_CTRL_START) && --i)
		udelay(1);

	/* Wait for LMEM DMA to complete (BSP: amhevc_stop) */
	i = 1000;
	while ((amvdec_read_dos(core, HEVC_LMEM_DMA_CTRL) &
		IMEM_DMA_CTRL_START) && --i)
		udelay(1);

	/* Wait for pending LMEM write responses (BSP: amhevc_stop) */
	i = 1000;
	while ((amvdec_read_dos(core, HEVC_WRRSP_LMEM) &
		HEVC_WRRSP_LMEM_PENDING_MASK) && --i)
		udelay(1);
}
EXPORT_SYMBOL_GPL(amvdec_hevc_halt_firmware);

/*
 * Restart the HEVC-engine AMRISC firmware after a drain_finalize halt.
 * Re-enables the MBOX1 IRQ mask and sets MPSR=1 to resume the firmware
 * from its paused PC (typically inside the AMLV-search / NAL-start-code
 * scan loop).  Does NOT reload the firmware binary or re-run the full
 * codec_ops->start init -- those are fresh-session entry points and
 * would wipe the existing decoder session state.
 *
 * Used by the V4L2_DEC_CMD_START ioctl when drain_finalize previously
 * halted the firmware but userspace is resuming the same session
 * without an intervening STREAMOFF.  This is the narrow case called out
 * as a known limitation in the original KI-VDEC-ARCH-001 merge; common
 * V4L2 flows go DEC_CMD_STOP -> STREAMOFF and never hit this path.
 *
 * Caveat: the VIFIFO may still contain residual data from the prior
 * drain (e.g. an injected EOS sentinel on G12A+).  When the firmware
 * resumes it may fire spurious VP9_EOS / parser-ready ISRs on the
 * residual data before it picks up the newly queued input.  The
 * per-codec ISR handlers are designed to handle these safely
 * (cur_frame NULL guards, atomic_dec_if_positive on counter).
 */
void amvdec_hevc_restart_firmware(struct amvdec_core *core)
{
	/* Re-enable MBOX1 IRQ; matches codec_hevc_start / codec_vp9_start */
	amvdec_write_dos(core, HEVC_ASSIST_MBOX1_MASK, 1);
	/* Resume firmware processor from halted PC */
	amvdec_write_dos(core, HEVC_MPSR, 1);
	/* CPSR stays at 0 -- matches BSP amhevc_start_f semantics */
}
EXPORT_SYMBOL_GPL(amvdec_hevc_restart_firmware);

static void __vdec_hevc_stop(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct amvdec_codec_ops *codec_ops = sess->fmt_out->codec_ops;
	bool woke = false;

	amvdec_hevc_halt_firmware(core);

	/*
	 * Synchronously power off the HEVC engine BEFORE codec_ops->stop frees
	 * its DMA-coherent buffers (workspace, MMU compress headers, MMU map).
	 *
	 * vdec_hevc_power_off() does the DOS_SW_RESET3=0xffffffff toggle that
	 * resets every HEVC IP sub-block (MMU, MPRED, MPP/HEVCD_MPP_DECOMP,
	 * SAO).  After this reset the IP block holds no internal references
	 * to the per-session DMA buffers, so dma_free_coherent() can release
	 * the CMA pages cleanly.
	 *
	 * Previously this happened via vdec_hevc_power_put() AFTER
	 * codec_ops->stop, with autosuspend (hevc_idle_ms=100ms default)
	 * deferring the actual power-off.  The IP block stayed DMA-active on
	 * the freed pages for ~100ms, producing
	 * "free_contig_range: N pages still in use!" WARN, leaked pages with
	 * stale refs, and intermittent LRU corruption / panic-reboot on
	 * small-CMA boards (KI-107 root cause).
	 *
	 * EOS is a definitive session end -- there is no fast-resume case to
	 * preserve, so synchronous power-off is the correct lifecycle.
	 * Autosuspend stays available for transient idle (start-time error
	 * unwind below); only the EOS path is forced synchronous.
	 *
	 * hw_lock guards core->hevc_powered against concurrent power-on from
	 * another session start.
	 *
	 * KI-118: this decode releases its hevc_power_users refcount here (the EOS
	 * path bypasses vdec_hevc_power_put -- see that comment).  If the HCODEC
	 * block-manager pin still holds the engine (users > 0 after the decrement),
	 * a full power-off would toggle the HEVC power domain and drop the pin --
	 * and re-establishing it later does the DOS-wide power-on that clobbers the
	 * running HCODEC.  So when pinned, do ONLY the DOS_SW_RESET3 buffer-clear
	 * (HCODEC lives in DOS_SW_RESET1, a different register, so RESET3 alone does
	 * not clobber the encoder) and leave the engine powered.  When NOT pinned,
	 * the original full power-off (RESET3 + power domain) runs as before.
	 */
	mutex_lock(&core->hw_lock);
	if (core->hevc_power_users > 0)
		core->hevc_power_users--;
	if (core->hevc_powered) {
		if (core->hevc_power_users == 0 &&
		    core->platform->revision >= VDEC_REVISION_G12A &&
		    !core->hevc_enc_owned) {
			/*
			 * KI-121: on G12A+ the HEVC power domain is shared with VDEC_1
			 * (the MDEC DC cache lives in it).  Do NOT power the domain off
			 * here at HEVC-decode EOS: a follow-on VDEC_1 decode (e.g.
			 * MPEG-2) would then have to cold-RE-POWER this just-used domain,
			 * and that GEN_PWR power-on of a residual domain silently wedges
			 * the SoC (the HEVC->MPEG-2 wedge).  The re-power hazard is a real
			 * HW limit (confirmed: a clean refcounted power-cycle still wedges
			 * -- see shared-domain-power-architecture.md), so keep-up is
			 * mandatory.  PIN the shared domain (RESET3 per-session
			 * buffer-clear only, same as the HCODEC-pin branch below) so a
			 * follow-on VDEC_1 poweron finds it already up (no re-power).  The
			 * RESET3 still releases the IP's DMA refs for KI-107.
			 */
			amvdec_write_dos(core, DOS_SW_RESET3, hevc_reset3_mask(core));
			usleep_range(10, 20);
			amvdec_write_dos(core, DOS_SW_RESET3, 0x00000000);
			usleep_range(10, 20);
			/* core->hevc_powered stays true (pinned) */
		} else if (core->hevc_power_users == 0) {
			vdec_hevc_power_off(core);
			core->hevc_powered = false;
			if (core->hevc_enc_owned) {
				core->hevc_enc_owned = false;
				woke = true;
			}
		} else {
			/* Pinned (HCODEC): clear the IP's per-session DMA buffer refs via
			 * RESET3 only; keep the engine powered for the pin. */
			amvdec_write_dos(core, DOS_SW_RESET3, hevc_reset3_mask(core));
			usleep_range(10, 20);
			amvdec_write_dos(core, DOS_SW_RESET3, 0x00000000);
			usleep_range(10, 20);
		}
	}
	mutex_unlock(&core->hw_lock);
	if (woke)
		wake_up(&core->hevc_dec_wq);

	if (sess->priv)
		codec_ops->stop(sess);

	/* Free firmware DMA buffer (swap code lives here) */
	if (fw_vaddr) {
		dma_free_coherent(core->dev, fw_dma_size,
				  fw_vaddr, fw_paddr);
		fw_vaddr = NULL;
	}
}

static int vdec_hevc_stop(struct amvdec_session *sess)
{
	__vdec_hevc_stop(sess);
	/*
	 * HEVC engine is already powered off synchronously above; cancel any
	 * pending deferred power-off work so it doesn't fire later as a no-op
	 * (idempotent, but tidy).
	 */
	cancel_delayed_work_sync(&sess->core->hevc_poweroff_work);

	return 0;
}

/**
 * vdec_hevc_power_on() - Power on HEVC engine (clocks + power domain).
 *
 * Performs the hardware initialization portion of the HEVC start sequence:
 * clock enable, power domain ON, reset, isolation OFF, memory ON, clock
 * gate enable.  Does NOT load firmware, does NOT start the AMRISC (no MPSR=1).
 *
 * Used by HEVC/VP9 decode (via __vdec_hevc_start) and by H.264 MMU mode
 * (via codec_h264.c) which needs the HEVC SAO pipeline without HEVC firmware.
 *
 * Caller must call vdec_hevc_power_off() to undo.
 */
int vdec_hevc_power_on(struct amvdec_core *core)
{
	int ret, i;

	/* Enable HEVC clocks via refcounted API */
	ret = vdec_hevc_clk_get(core);
	if (ret)
		return ret;

	if (core->platform->revision == VDEC_REVISION_SM1)
		regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_SLEEP0,
				   GEN_PWR_VDEC_HEVC_SM1, 0);
	else
		regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_SLEEP0,
				   GEN_PWR_VDEC_HEVC, 0);
	usleep_range(10, 20);

	/* SM1 has a combined VDEC_1/HEVC power domain.  The BSP powers
	 * on VDEC_1 memory and resets it before touching HEVC registers.
	 * Without this, the shared DOS bus may be partially powered and
	 * register accesses can hang the system (observed: soft-lock on
	 * first HEVC decode after clean boot, ICMP works but scheduler
	 * dead).  See BSP vdec_power_ctrl.c:219-226.
	 *
	 * Skip the VDEC_1 reset if VDEC_1 is already powered on
	 * (DOS_MEM_PD_VDEC == 0).  When called from H.264 MMU mode,
	 * vdec_1_start() has already powered on VDEC_1 and loaded
	 * firmware; resetting it destroys the firmware and stalls
	 * decode (MBY_MBX=0).
	 */
	if (core->platform->revision == VDEC_REVISION_SM1) {
		u32 mem_pd = amvdec_read_dos(core, DOS_MEM_PD_VDEC);

		if (mem_pd != 0) {
			amvdec_write_dos(core, DOS_MEM_PD_VDEC, 0x00000000);
			amvdec_write_dos(core, DOS_SW_RESET0, 0xfffffffc);
			usleep_range(10, 20);
			amvdec_write_dos(core, DOS_SW_RESET0, 0x00000000);
			usleep_range(10, 20);
			amvdec_write_dos(core, DOS_MEM_PD_VDEC, 0x00000000);
		}
	}

	/* Reset VDEC_HEVC */
	amvdec_write_dos(core, DOS_SW_RESET3, hevc_reset3_mask(core));
	amvdec_write_dos(core, DOS_SW_RESET3, 0x00000000);

	amvdec_write_dos(core, DOS_GCLK_EN3, 0xffffffff);

	/* VDEC_HEVC Memories */
	amvdec_write_dos(core, DOS_MEM_PD_HEVC, 0x00000000);

	/* Remove VDEC_HEVC Isolation.
	 * On G12A+, clear both HEVC and VDEC1 isolation since they
	 * share power domain hardware (MDEC DC cache).
	 *
	 * On GXL/GXBB/GXM, only clear HEVC isolation (GEN_PWR_VDEC_HEVC).
	 * These SoCs have separate HEVC and VDEC1 power domains.  Clearing
	 * VDEC1 isolation here while VDEC1 is powered off exposes
	 * unpowered registers on the shared DOS bus, risking bus hangs
	 * on engine transitions.
	 */
	if (core->platform->revision == VDEC_REVISION_SM1)
		regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_ISO0,
				   GEN_PWR_ISO0_HEVC_SM1, 0);
	else if (core->platform->revision >= VDEC_REVISION_G12A)
		regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_ISO0,
				   GEN_PWR_ISO0_HEVC | GEN_PWR_ISO0_VDEC_1, 0);
	else
		regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_ISO0,
				   GEN_PWR_ISO0_HEVC, 0);
	usleep_range(10, 20);

	amvdec_write_dos(core, DOS_SW_RESET3, hevc_reset3_mask(core));
	amvdec_write_dos(core, DOS_SW_RESET3, 0x00000000);

	/* Wait for pending HEVC LMEM write responses to drain.
	 * BSP hevc_reset_core() waits for HEVC_WRRSP_LMEM bits [11:0]
	 * to become 0 after DOS_SW_RESET3 clear.  Pending writes that
	 * complete after reset can corrupt register state.
	 */
	i = 1000;
	while ((amvdec_read_dos(core, HEVC_WRRSP_LMEM) &
		HEVC_WRRSP_LMEM_PENDING_MASK) && --i)
		udelay(1);

	/* Toggle HEVC front-end DMC pipeline reset (RESET7 bit 13).
	 * BSP hevc_reset_core() does this for G12A/G12B/SM1 after
	 * DOS_SW_RESET3.  Without it, HEVC decode on SM1 can soft-lock
	 * the kernel on first use after boot.
	 */
	if (core->hevcf_dmc_pipl_reset) {
		reset_control_assert(core->hevcf_dmc_pipl_reset);
		usleep_range(10, 20);
		reset_control_deassert(core->hevcf_dmc_pipl_reset);
		usleep_range(10, 20);
	}

	/* SM1 combined power domain: enable HEVC core after reset.
	 * BSP vh265.c:16180-16181 writes HEVC_CORE_ENABLE=1 after
	 * hevc_reset_core() when is_vdec_hevc_combine().  VDEC_1
	 * decoders (MJPEG, VC1) write 0 here instead.
	 * Note: BSP only does this for is_vdec_hevc_combine SoCs
	 * (not SM1), but it may help with bus initialization.
	 */
	if (core->platform->revision == VDEC_REVISION_SM1)
		amvdec_write_dos(core, HEVC_CORE_ENABLE, 1);

	/*
	 * N102: the HEVC DOS block is now powered, un-isolated and clocked --
	 * an MMIO read of an HEVC-block register is safe from here until a
	 * matching power-off.  Callers hold hw_lock (vdec_hevc_power_get, the
	 * poweroff work, and codec_h264's MMU gate), which is what lets the
	 * debugfs readers keep this true across their reads.
	 */
	core->hevc_domain_on = true;

	return 0;
}
EXPORT_SYMBOL_GPL(vdec_hevc_power_on);

/**
 * vdec_hevc_power_off() - Power off the HEVC engine.
 *
 * Reset, enable isolation, power off memories, assert SLEEP0, disable clocks.
 * Must match a prior vdec_hevc_power_on() call.
 *
 * Safe to call while VDEC_1 is running (independent power domains on G12A+).
 */
void vdec_hevc_power_off(struct amvdec_core *core)
{
	/* Full HEVC engine reset before power-down.
	 * BSP dos_local_config(off, VDEC_HEVC) does
	 * DOS_SW_RESET3=0xffffffff before powering off memories.
	 * Without this reset, stale HEVC engine state survives the
	 * power cycle and corrupts subsequent VDEC_1 sessions
	 * (H.264, MPEG) on the shared DOS bus -- observed as 0
	 * output frames from ALL codecs after a failed HEVC decode.
	 */
	amvdec_write_dos(core, DOS_SW_RESET3, hevc_reset3_mask(core));
	usleep_range(10, 20);
	amvdec_write_dos(core, DOS_SW_RESET3, 0x00000000);
	usleep_range(10, 20);

	/*
	 * Power-off order: clock off BEFORE isolation.
	 * Stop clocks before asserting isolation to prevent glitches.
	 */
	vdec_hevc_clk_put(core);

	/* Enable VDEC_HEVC Isolation */
	if (core->platform->revision == VDEC_REVISION_SM1)
		regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_ISO0,
				   GEN_PWR_ISO0_HEVC_SM1,
				   GEN_PWR_ISO0_HEVC_SM1);
	else
		regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_ISO0,
				   GEN_PWR_ISO0_HEVC, GEN_PWR_ISO0_HEVC);

	/* VDEC_HEVC Memories */
	amvdec_write_dos(core, DOS_MEM_PD_HEVC, 0xffffffffUL);

	if (core->platform->revision == VDEC_REVISION_SM1)
		regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_SLEEP0,
				   GEN_PWR_VDEC_HEVC_SM1,
				   GEN_PWR_VDEC_HEVC_SM1);
	else
		regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_SLEEP0,
				   GEN_PWR_VDEC_HEVC, GEN_PWR_VDEC_HEVC);

	/*
	 * N102: the domain is isolated and powered off -- UNCONDITIONALLY, even
	 * though the vdec_hevc_clk_put() above is refcounted and may have left
	 * the clocks running for a VDEC_1 holder.  So "clocks on" is NOT the
	 * predicate; this is, and it fails to the safe side.
	 */
	core->hevc_domain_on = false;
}
EXPORT_SYMBOL_GPL(vdec_hevc_power_off);

/*
 * hevc_clk_apply_turbo - apply turbo rate to HEVC clocks with gp0_pll switch.
 *
 * Called after vdec_hevc_power_on() when HEVC engine is starting its own
 * decode session.  Not called from VDEC_1 clock-only path (vdec_hevc_clk_get).
 * GXL has no gp0_pll in its HEVC mux, so turbo silently falls back on GXL.
 */
static void hevc_clk_apply_turbo(struct amvdec_core *core)
{
	unsigned long target = hevc_target_rate(core);

	/*
	 * If gp0_pll is available and target > 800 MHz, switch the mux
	 * parent to gp0_pll so the PLL can be programmed to the exact rate.
	 * Walk: gate -> div -> mux.
	 */
	if (core->gp0_pll && target > 800000000) {
		clk_set_rate(core->gp0_pll, target);
		clk_prepare_enable(core->gp0_pll);

		struct clk *div = clk_get_parent(core->vdec_hevc_clk);
		struct clk *mux = div ? clk_get_parent(div) : NULL;

		if (mux)
			clk_set_parent(mux, core->gp0_pll);
	}

	clk_set_rate(core->vdec_hevc_clk, target);
	if (core->vdec_hevcf_clk)
		clk_set_rate(core->vdec_hevcf_clk, target);
}

static void hevc_poweroff_work_fn(struct work_struct *work)
{
	struct amvdec_core *core = container_of(work, struct amvdec_core,
						hevc_poweroff_work.work);

	bool woke = false;

	mutex_lock(&core->hw_lock);
	/* KI-118: only power off if no holder re-acquired during the idle delay */
	if (core->hevc_power_users == 0 && core->hevc_powered) {
		vdec_hevc_power_off(core);
		core->hevc_powered = false;
		core->hevc_enc_owned = false;
		woke = true;
	}
	mutex_unlock(&core->hw_lock);
	if (woke)
		wake_up(&core->hevc_dec_wq);
}

void vdec_hevc_power_init(struct amvdec_core *core)
{
	INIT_DELAYED_WORK(&core->hevc_poweroff_work, hevc_poweroff_work_fn);
	core->hevc_powered = false;
	core->hevc_domain_on = false;	/* N102 */
	core->hevc_power_users = 0;
	core->hevc_enc_owned = false;
	init_waitqueue_head(&core->hevc_dec_wq);
}

/*
 * vdec_hevc_power_get - ensure HEVC engine is powered on.
 * Cancels any pending idle poweroff and powers on if needed.
 * Self-locking: caller must NOT hold hw_lock.
 *
 * @hcodec_pin: true when called by the HCODEC block-manager pin (KI-118),
 * false for a normal HEVC decode session.  On the cold power-on
 * (vdec_hevc_power_on's DOS_SW_RESET3 hazard), records WHO initialised the
 * engine: if the encoder pin powered it on (encode-first), the engine lacks the
 * decode-appropriate reset state, so a later HEVC decode must wait for a clean
 * power-cycle (see vdec_hevc_start) -- mirrors the VDEC_1 block manager.
 */
int vdec_hevc_power_get(struct amvdec_core *core, bool hcodec_pin)
{
	cancel_delayed_work_sync(&core->hevc_poweroff_work);
	mutex_lock(&core->hw_lock);
	/* N118: same refusal, same reason, as vdec_1_power_get(). */
	if (READ_ONCE(core->unbound)) {
		mutex_unlock(&core->hw_lock);
		return -ENODEV;
	}
	if (!core->hevc_powered) {
		int ret = vdec_hevc_power_on(core);

		if (ret) {
			mutex_unlock(&core->hw_lock);
			return ret;
		}
		hevc_clk_apply_turbo(core);
		core->hevc_powered = true;
		core->hevc_enc_owned = hcodec_pin;
	}
	core->hevc_power_users++;	/* KI-118: refcount holders */
	mutex_unlock(&core->hw_lock);
	return 0;
}

/*
 * vdec_hevc_power_put - schedule delayed HEVC poweroff.
 * If hevc_idle_ms is 0, powers off immediately under hw_lock.
 * Otherwise schedules delayed_work (which self-locks).
 * Caller must NOT hold hw_lock.
 */
void vdec_hevc_power_put(struct amvdec_core *core)
{
	bool sched_off = false;
	bool woke = false;

	/* KI-118: drop a refcount; only release power when the LAST holder puts
	 * (HEVC decode sessions + the HCODEC cross-engine pin share the engine).
	 */
	mutex_lock(&core->hw_lock);
	if (core->hevc_power_users > 0)
		core->hevc_power_users--;
	if (core->hevc_power_users == 0) {
		/* N123: same as vdec_1_power_put() -- an engine already powered off
		 * by vdec_remove() takes no MMIO here; hevc_poweroff_work_fn()
		 * already makes this test.  Clear ownership and wake regardless.
		 */
		if (!core->hevc_powered) {
			core->hevc_enc_owned = false;
			woke = true;
		} else if (hevc_idle_ms == 0 || core->hevc_enc_owned) {
			vdec_hevc_power_off(core);
			core->hevc_powered = false;
			core->hevc_enc_owned = false;
			woke = true;
		} else {
			sched_off = true;
		}
	}
	mutex_unlock(&core->hw_lock);
	if (woke)
		wake_up(&core->hevc_dec_wq);
	if (sched_off)
		schedule_delayed_work(&core->hevc_poweroff_work,
				      msecs_to_jiffies(hevc_idle_ms));
}

void vdec_hevc_thermal_update(struct amvdec_core *core, unsigned long rate)
{
	clk_set_rate(core->vdec_hevc_clk, rate);
	if (core->vdec_hevcf_clk)
		clk_set_rate(core->vdec_hevcf_clk, rate);
}

/*
 * Refcounted HEVC clock enable/disable.
 *
 * On G12A+, the MDEC DC cache hardware is inside the HEVC power domain,
 * so VDEC_1 must enable HEVC clocks for correct H.264/MPEG decode.
 * Both VDEC_1 and HEVC can independently request the HEVC clocks.
 * The clocks are disabled only when the last user releases them.
 */
int vdec_hevc_clk_get(struct amvdec_core *core)
{
	int ret;

	if (atomic_inc_return(&core->hevc_clk_ref) > 1)
		return 0; /* already enabled by another user */

	clk_set_rate(core->vdec_hevc_clk, amvdec_clock_rate(core));
	ret = clk_prepare_enable(core->vdec_hevc_clk);
	if (ret) {
		atomic_dec(&core->hevc_clk_ref);
		return ret;
	}

	if (core->vdec_hevcf_clk) {
		clk_set_rate(core->vdec_hevcf_clk, amvdec_clock_rate(core));
		ret = clk_prepare_enable(core->vdec_hevcf_clk);
		if (ret) {
			clk_disable_unprepare(core->vdec_hevc_clk);
			atomic_dec(&core->hevc_clk_ref);
			return ret;
		}
	}

	return 0;
}

void vdec_hevc_clk_put(struct amvdec_core *core)
{
	if (atomic_dec_return(&core->hevc_clk_ref) > 0)
		return; /* still in use by another user */

	clk_disable_unprepare(core->vdec_hevc_clk);
	if (core->vdec_hevcf_clk)
		clk_disable_unprepare(core->vdec_hevcf_clk);
}

/*
 * N95 instrument -- the VDEC_HEVC sub-block control registers.
 *
 * WHY: codec_hevc_init_decoder() force-clocks and re-writes all five of
 * HEVC_STREAM_CONTROL / _SHIFT_CONTROL / _CABAC_CONTROL / _PARSER_CORE_CONTROL
 * / _PARSER_INT_CONTROL before it configures them, and additionally forces the
 * IQIT / DBLK / SAO / MPRED / IPP clocks on.  codec_vp9_start() issues none of
 * those force-ons and read-modify-writes three of the same registers, so any
 * residue the BOOT CHAIN leaves in this block is masked for HEVC and inherited
 * by VP9 -- exactly the shape of a fault that spares one codec and kills the
 * other on the same engine (N94 F6).  No debugfs file read these registers, so
 * the hypothesis was untestable rather than unlikely.
 *
 * SAFE BY CONSTRUCTION (N72): every offset below is a VDEC_HEVC-block or
 * DOS-block register, and both dump sites run only when the VDEC_HEVC block is
 * clocked, un-isolated and out of reset -- the __vdec_hevc_start() site is
 * after vdec_hevc_power_get(), the debugfs site is behind core->cur_sess AND
 * the session's engine actually being vdec_hevc_ops.  NOT ONE VDEC_1 register
 * appears here; that is what reset the SoC when hevc_sao_status_show() read
 * MDEC_PIC_DC_MUX_CTRL with no VDEC_1 session in the boot (N94 F7).
 *
 * One register per line, short and uniquely tagged, because this rig's UART
 * adapter drops characters in bursts and a long packed line is unrecoverable.
 */
static const struct {
	u32 off;
	const char *name;
} hevc_subblock_regs[] = {
	/* the five N94 F6 names */
	{ HEVC_STREAM_CONTROL,		"STREAM_CONTROL" },
	{ HEVC_SHIFT_CONTROL,		"SHIFT_CONTROL" },
	{ HEVC_CABAC_CONTROL,		"CABAC_CONTROL" },
	{ HEVC_PARSER_CORE_CONTROL,	"PARSER_CORE_CONTROL" },
	{ HEVC_PARSER_INT_CONTROL,	"PARSER_INT_CONTROL" },
	/* the rest of the stream/parser front end the two codecs program */
	{ HEVC_PARSER_IF_CONTROL,	"PARSER_IF_CONTROL" },
	{ HEVC_STREAM_FIFO_CTL,		"STREAM_FIFO_CTL" },
	{ HEVC_SHIFT_STATUS,		"SHIFT_STATUS" },
	{ HEVC_SHIFT_STARTCODE,		"SHIFT_STARTCODE" },
	/* the clock force-ons HEVC issues and VP9 does not */
	{ HEVCD_IPP_DYNCLKGATE_CONFIG,	"IPP_DYNCLKGATE_CONFIG" },
	{ HEVC_IQIT_CLK_RST_CTRL,	"IQIT_CLK_RST_CTRL" },
	{ HEVC_DBLK_CFG0,		"DBLK_CFG0" },
	{ HEVC_SAO_CTRL1,		"SAO_CTRL1" },
	{ HEVC_MPRED_CTRL1,		"MPRED_CTRL1" },
	/* DOS-level context for the same instant */
	{ DOS_GEN_CTRL0,		"DOS_GEN_CTRL0" },
	{ DOS_GCLK_EN3,			"DOS_GCLK_EN3" },
	{ DOS_MEM_PD_HEVC,		"DOS_MEM_PD_HEVC" },
	/*
	 * N95c.  Not a VDEC_HEVC register at all -- a BOOT-CHAIN-owned DOS
	 * scratch (dos_regs.h already records it as "written by U-Boot").  It
	 * is here because `gxl_vp9_mmu.bin` SPINS on it: disassembled with
	 * vici-dasm, word 0x92f is
	 *     092f: mfcr   r43, 0x3f2f   # DOS_SCRATCH31
	 *     0930: bnandi r43, 4, -1    # -> 092f, i.e. loop until BIT(2) is set
	 * and `gxl_hevc_mmu.bin` word 0x7c8 is the same read with
	 *     07c9: bandi  r35, 1, -1    # -> 07c8, i.e. loop until BIT(0) CLEARS
	 * -- so an all-zero word releases HEVC immediately and holds VP9 for
	 * ever, on the same engine, with every other register identical.  That
	 * is the exact shape of the fault, and the value is what decides it.
	 */
	{ DOS_SCRATCH_31,		"DOS_SCRATCH_31" },
};

void amvdec_hevc_dump_subblock_regs(struct amvdec_core *core, const char *tag)
{
	int i;

	/*
	 * dev_dbg, not dev_info: this fires TWICE per VDEC_HEVC session and
	 * prints one line per register, which is far too much for a shipping
	 * kernel's log.  Turn it on when it is wanted with dynamic debug --
	 *   echo 'file vdec_hevc.c +p' > /sys/kernel/debug/dynamic_debug/control
	 * -- or read the same registers, live and on demand, from
	 * /sys/kernel/debug/meson-vdec/firmware_status.  The N95 boots that
	 * produced the DOS_SCRATCH_31 finding were run with dev_info, before
	 * this commit; nothing else about the reads changed.
	 */
	for (i = 0; i < ARRAY_SIZE(hevc_subblock_regs); i++)
		dev_dbg(core->dev, "N95 %s %s=%08x\n", tag,
			hevc_subblock_regs[i].name,
			amvdec_read_dos(core, hevc_subblock_regs[i].off));
}
EXPORT_SYMBOL_GPL(amvdec_hevc_dump_subblock_regs);

void amvdec_hevc_seq_subblock_regs(struct seq_file *s,
				   struct amvdec_core *core)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(hevc_subblock_regs); i++)
		seq_printf(s, "  %s: 0x%08x\n", hevc_subblock_regs[i].name,
			   amvdec_read_dos(core, hevc_subblock_regs[i].off));
}
EXPORT_SYMBOL_GPL(amvdec_hevc_seq_subblock_regs);

/*
 * N95b instrument -- WHERE the AMRISC is, not just that it is running.
 *
 * `HEVC_MPSR` = 5 says the processor is enabled; it does not say the firmware
 * is making progress.  `HEVC_MPC_E` is the M-core program counter, so sampling
 * it several times inside ONE debugfs read separates a firmware spinning in a
 * loop (several distinct values) from one parked at a single instruction (all
 * equal) -- e.g. a swap-code fetch from `HEVC_STREAM_SWAP_BUFFER_2` (=
 * SCRATCH_8) that never returns, which is the only VP9-specific step in the
 * whole start path (VP9's firmware is 28 672 B against MC_SIZE = 16 384, so it
 * is the only GXL codec that fetches code from DRAM at run time).
 *
 * The rest localise the stall along the stream front end: SHIFT_BYTE_COUNT is
 * how many bytes the shifter has pulled out of the VIFIFO, so a zero there
 * with a full `HEVC_STREAM_LEVEL` puts the fault upstream of the parser; the
 * IF_WAIT counters say which downstream sub-block the pipeline is waiting on.
 *
 * Same safety envelope as the table above: VDEC_HEVC-block offsets only,
 * reached only behind core->cur_sess with an HEVC-engine session (N72).
 */
static const struct {
	u32 off;
	const char *name;
} hevc_progress_regs[] = {
	{ HEVC_CPSR,			"CPSR" },
	{ HEVC_SHIFT_BYTE_COUNT,	"SHIFT_BYTE_COUNT" },
	{ HEVC_SHIFT_COMMAND,		"SHIFT_COMMAND" },
	{ HEVC_SHIFTED_DATA,		"SHIFTED_DATA" },
	{ HEVC_SHIFT_EMULATECODE,	"SHIFT_EMULATECODE" },
	{ HEVC_PARSER_INT_STATUS,	"PARSER_INT_STATUS" },
	{ HEVC_PARSER_CMD_STATUS,	"PARSER_CMD_STATUS" },
	{ HEVC_PARSER_LCU_START,	"PARSER_LCU_START" },
	{ HEVC_PARSER_LCU_INFO,		"PARSER_LCU_INFO" },
	{ HEVC_STREAM_SWAP_ADDR,	"STREAM_SWAP_ADDR" },
	{ HEVC_STREAM_SWAP_CTRL,	"STREAM_SWAP_CTRL" },
	{ HEVC_IMEM_DMA_CTRL,		"IMEM_DMA_CTRL" },
	{ HEVC_IMEM_DMA_ADR,		"IMEM_DMA_ADR" },
	{ HEVC_IMEM_DMA_COUNT,		"IMEM_DMA_COUNT" },
	{ HEVC_SAO_IF_STATUS,		"SAO_IF_STATUS" },
	{ HEVC_IQIT_IF_WAIT_CNT,	"IQIT_IF_WAIT_CNT" },
	{ HEVC_MPRED_IF_WAIT_CNT,	"MPRED_IF_WAIT_CNT" },
	{ HEVC_SAO_IF_WAIT_CNT,		"SAO_IF_WAIT_CNT" },
	{ HEVC_MPRED_INT_STATUS,	"MPRED_INT_STATUS" },
	{ HEVC_MPRED_CURR_LCU,		"MPRED_CURR_LCU" },
};

void amvdec_hevc_seq_progress_regs(struct seq_file *s,
				   struct amvdec_core *core)
{
	int i;

	/* Eight back-to-back PC samples: all-equal means parked, not looping. */
	seq_puts(s, "  MPC_E_x8:");
	for (i = 0; i < 8; i++)
		seq_printf(s, " %08x", amvdec_read_dos(core, HEVC_MPC_E));
	seq_puts(s, "\n");

	for (i = 0; i < ARRAY_SIZE(hevc_progress_regs); i++)
		seq_printf(s, "  %s: 0x%08x\n", hevc_progress_regs[i].name,
			   amvdec_read_dos(core, hevc_progress_regs[i].off));
}
EXPORT_SYMBOL_GPL(amvdec_hevc_seq_progress_regs);

static int __vdec_hevc_start(struct amvdec_session *sess)
{
	int ret;
	struct amvdec_core *core = sess->core;
	struct amvdec_codec_ops *codec_ops = sess->fmt_out->codec_ops;

	/*
	 * KI-118 queued-reset for an HEVC decode arriving mid-encode:
	 *
	 * G12A+: the HCODEC pin owns the HEVC engine (encode-first cold-powered it,
	 * so it lacks the decode-appropriate reset and we cannot give it one now --
	 * vdec_hevc_power_on's DOS_SW_RESET3 would wedge the running HCODEC).  Block
	 * until the encoder releases its pin and the engine power-cycles
	 * (vdec_hevc_power_put forces an immediate power-off for an encoder-owned
	 * engine, then wakes us).  Mirrors the VDEC_1 block manager.
	 *
	 * GXL: the HEVC pin is gated off (it wedges -- no VDEC_1<->HEVC coupling),
	 * so there is no enc-owned state to wait on.  Instead serialize directly on
	 * the encoder: an HEVC cold power-on conflicts with a running HCODEC on GXL,
	 * so block while HCODEC is powered (the hcodec poweroff paths wake us).
	 */
	if (ki118_block_manager) {
		if (core->platform->revision >= VDEC_REVISION_G12A)
			ret = wait_event_interruptible(core->hevc_dec_wq,
						       !READ_ONCE(core->hevc_enc_owned));
		else
			ret = wait_event_interruptible(core->hevc_dec_wq,
						       !READ_ONCE(core->hcodec_powered));
		if (ret)
			return ret;	/* -ERESTARTSYS: let userspace retry */
	}

	/*
	 * Power on HEVC engine via autosuspend-aware API.
	 * vdec_hevc_power_get() cancels any pending idle poweroff and
	 * powers on with turbo clock rate applied if not already powered.
	 */
	ret = vdec_hevc_power_get(core, false);
	if (ret)
		return ret;

	/* Fill HEVC AXI table with scratch buffer address */
	amvdec_fill_scratch_hevc(core);

	vdec_hevc_stbuf_init(sess);

	ret = vdec_hevc_load_firmware(sess, sess->fmt_out->firmware_path);
	if (ret)
		goto stop;

	/*
	 * Defense in depth: per-codec modules (meson-vdec-hevc, -vp9, ...)
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

	/*
	 * N95: the ONE point that is common to every VDEC_HEVC codec and runs
	 * BEFORE any codec-specific register write -- power/clock/isolation and
	 * the firmware load are done, codec_ops->start() has not run.  Dumping
	 * here makes the HEVC arm and the VP9 arm byte-comparable, and makes the
	 * open chain and the vendor chain byte-comparable, with ONE instrument.
	 */
	amvdec_hevc_dump_subblock_regs(core, "pre ");

	ret = codec_ops->start(sess);
	if (ret)
		goto stop;

	amvdec_hevc_dump_subblock_regs(core, "post");

	/* Write swap code address to SCRATCH_8 before starting AMRISC.
	 * BSP: copies fw[SWAP_HEVC_OFFSET:] to a separate DMA buffer and
	 * writes that buffer's address to HEVC_STREAM_SWAP_BUFFER2.
	 * The firmware reads swap code directly from this address without
	 * adding any internal offset.  Since we keep the full firmware in
	 * one DMA buffer, we write fw_paddr + SWAP_HEVC_OFFSET so the
	 * firmware sees the swap code at address offset 0.
	 */
	if (fw_vaddr && fw_dma_size > MC_SIZE)
		amvdec_write_dos(core, HEVC_STREAM_SWAP_BUFFER2,
				 fw_paddr + SWAP_HEVC_OFFSET);

	amvdec_write_dos(core, DOS_SW_RESET3, DOS_SW_RESET3_HEVC_PARTIAL);
	amvdec_write_dos(core, DOS_SW_RESET3, 0);
	amvdec_read_dos(core, DOS_SW_RESET3);

	amvdec_write_dos(core, HEVC_MPSR, 1);
	/* Let the firmware settle */
	usleep_range(10, 20);

	return 0;

stop:
	/* KI-118: __vdec_hevc_stop now releases the hevc_power_users refcount
	 * (and powers off / RESET3s as appropriate), so do NOT also power_put
	 * here -- that would double-decrement. */
	__vdec_hevc_stop(sess);
	return ret;
}

static int vdec_hevc_start(struct amvdec_session *sess)
{
	return __vdec_hevc_start(sess);
}

struct amvdec_ops vdec_hevc_ops = {
	.start = vdec_hevc_start,
	.stop = vdec_hevc_stop,
	.conf_esparser = vdec_hevc_conf_esparser,
	.vififo_level = vdec_hevc_vififo_level,
};
