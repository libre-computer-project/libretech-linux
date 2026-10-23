// SPDX-License-Identifier: GPL-2.0
/*
 * vdec_diag.c -- AMRISC diagnostic hot-reload interface (Tier 1 test bench)
 *
 * Exposes a debugfs directory /sys/kernel/debug/meson-vdec/diag/vdec1/
 * with the following files:
 *
 *   firmware  (write-only, binary blob up to 16 KiB)
 *       Hot-upload a raw AMRISC firmware image into the VDEC_1 engine's
 *       IMEM, bypassing request_firmware() and the module reload path.
 *       Sequence: MPSR=0, CPSR=0, IMEM_DMA to mc_addr_map, MPSR=1.
 *       No byte-patches are applied (the uploaded blob is consumed as-is).
 *
 *   reset     (write-only, "1")
 *       Pulse DOS_SW_RESET0_MDEC | DOS_SW_RESET0_VLD, re-arm MPSR.
 *       Does NOT reload firmware; leaves whatever is in IMEM running.
 *
 *   state     (read-only, text key=value lines)
 *       Single snapshot of the interesting scratch and hardware registers:
 *       AV_SCRATCH_0..F, VLD_STATUS_CTRL, VLD_MEM_VIFIFO_LEVEL,
 *       VLD_MEM_VIFIFO_BYTES_AVAIL, MDEC_PIC_DC_STATUS, MPSR, MINT_VEC_BASE,
 *       MCPU_INTR_MSK.
 *
 *   peek      (read-write, "<offset_hex>")
 *       Write a DOS MMIO byte offset; read back the 32-bit value at that
 *       offset.  The offset is range-checked against 0x0000-0x3fff.
 *
 *   poke      (write-only, "<offset_hex> <value_hex>")
 *       Write a 32-bit value to a DOS MMIO byte offset.  Range-checked.
 *
 *   mode      (write-only, "<decimal>")
 *       Shortcut for writing AV_SCRATCH_8.  The diagnostic firmware reads
 *       this at entry to select which test to run.
 *
 *   cmd       (write-only, "<decimal>")
 *       Shortcut for writing AV_SCRATCH_9.  The diagnostic firmware reads
 *       this as the command register in the command-poll test.
 *
 *   lmem_dma_test     (write-only, "<ctrl_hex> <count_dec> [<adr_hex>]")
 *       Fire one LMEM_DMA pulse with the parsed CTRL/COUNT/ADR.  Polls
 *       LMEM_DMA_CTRL bit 15 for clear (10000-iter cap) and snapshots
 *       elapsed time + WRRSP_LMEM into the engine's last_result struct.
 *       Hangs are recorded as data, not rejected -- the harness is for
 *       characterizing CTRL bit fields against engine response.
 *
 *   lmem_dma_result   (read-only, key=value lines)
 *       Outputs the most recent lmem_dma_test outcome (result, ctrl,
 *       count, adr, iters, wrrsp, ms).  Returns "result=noinit" if no
 *       lmem_dma_test write has happened since boot.
 *
 *   workspace_paddr   (read-only, "0x????????")
 *       Physical address of the pre-allocated 4 KiB LMEM workspace.
 *       Use as the implicit ADR target for lmem_dma_test, or for
 *       arranging fixture memory the firmware can DMA from.
 *
 *   workspace_pattern (read-write, "<seed_hex>")
 *       Write: fill workspace[i] = seed + i*4 for i in [0, 1024).
 *       Read: returns the most recent seed (or 0 on init).
 *
 * Safety:
 *   - Module param gate (`diag=1` on meson_vdec_core) -- directory is
 *     not created when disabled.  Default: disabled.
 *   - Operations require a valid VDEC_1 session to own the engine so
 *     power/clock/reset state is established.  Firmware hot-upload
 *     while no session exists returns -ENODEV.
 *   - Firmware blob is size-checked against MC_SIZE (16 KiB).
 *   - peek/poke offsets are range-checked against 0x0000-0x3fff.
 *   - Every diag operation is logged with dev_info (audit trail in the
 *     kernel ring buffer).
 *
 * Design and rationale:
 *   the internal stateless test-bench plan.
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>

#include <linux/soc/amlogic/meson-canvas.h>

#include "vdec.h"
#include "vdec_diag.h"
#include "vdec_helpers.h"
#include "dos_regs.h"

#define VDEC_DIAG_MC_SIZE		(4096 * 4)	/* 16 KiB, VDEC_1 IMEM budget */
#define VDEC_DIAG_DOS_OFFSET_MAX	0x3fff
#define VDEC_DIAG_PEEK_BUF_LEN		32

/*
 * LMEM_DMA characterization workspace -- 4 KiB pre-allocated DMA region
 * reused across lmem_dma_test fires.  4 KiB / 4 = 1024 32-bit words is
 * the maximum LMEM_DMA_COUNT the workspace can support; lmem_dma_test
 * range-checks count against this bound.
 */
#define VDEC_DIAG_WORKSPACE_SIZE	4096
#define VDEC_DIAG_WORKSPACE_WORDS	(VDEC_DIAG_WORKSPACE_SIZE / 4)

/* VIFIFO push ring: DMA-coherent buffer a raw bitstream is pre-filled into
 * by the vififo_push node, then exposed to the VLD VIFIFO (1 MiB is ample
 * for the single-frame elementary streams the stateless co-validation
 * fixtures feed). */
#define VDEC_DIAG_VIFIFO_SIZE		(1 << 20)

/* lmem_dma_test parser limit: "<ctrl_hex> <count_dec> <adr_hex>" + NUL. */
#define VDEC_DIAG_LMEM_TEST_BUF_LEN	64

/* Bound for the DMA-clear poll loop (usleep(1) per iter -> ~10 ms cap). */
#define VDEC_DIAG_LMEM_POLL_MAX		10000

/* LMEM_DMA_CTRL bit 15 -- engine busy flag (auto-clears on completion). */
#define LMEM_DMA_CTRL_START		BIT(15)

bool vdec_diag_enabled;
EXPORT_SYMBOL_GPL(vdec_diag_enabled);

/* Per-engine diagnostic state.  Tier 1 supports VDEC_1 only; HEVC and
 * HCODEC are stubs added in a later session. */
struct vdec_diag_engine {
	struct amvdec_core *core;
	const char *name;
	struct dentry *dir;
	struct mutex lock;
	/* DMA buffer for IMEM hot-uploads; allocated at init, reused. */
	void *fw_buf;
	dma_addr_t fw_buf_dma;
	size_t fw_buf_size;
	/* Remembered state for peek (user writes offset, then reads value) */
	u32 peek_offset;
	bool peek_armed;
	/* Set by diag-manual poweron (echo 1 > poweron). When true, peek /
	 * poke / firmware / mode / cmd operations are permitted WITHOUT
	 * a V4L2 session (the original gate). Clear by echo 0 > poweron. */
	bool manual_powered;

	/*
	 * LMEM_DMA characterization workspace.  4 KiB DMA-coherent buffer
	 * allocated at engine init, freed at exit.  Reused across every
	 * lmem_dma_test fire.  workspace_seed records the most recent value
	 * passed to workspace_pattern (read-back exposes it via debugfs).
	 */
	void *workspace_buf;
	dma_addr_t workspace_dma;
	u32 workspace_seed;

	/*
	 * Last lmem_dma_test result.  Populated on every lmem_dma_test
	 * write; read by lmem_dma_result.  result_valid is false until
	 * the first fire (read returns "result=noinit").
	 */
	bool last_result_valid;
	bool last_result_hang;	/* true = bit 15 never cleared within poll budget */
	u32 last_ctrl;
	u32 last_count;
	u32 last_adr;
	u32 last_iters;		/* iterations consumed before bit 15 cleared */
	u32 last_wrrsp;		/* WRRSP_LMEM after the pulse */
	u32 last_ms;		/* elapsed time in ms */

	/*
	 * VIFIFO push ring (vififo_push node).  DMA-coherent buffer a raw
	 * bitstream is copied into; the VLD VIFIFO pointers are then
	 * programmed to expose `vififo_len` bytes to the firmware.  Allocated
	 * at engine init, freed at exit.
	 */
	void *vififo_buf;
	dma_addr_t vififo_dma;
	u32 vififo_len;		/* bytes currently exposed (WP - ring base) */

	/*
	 * Output NV12 canvas (output_canvas node).  A DMA-coherent buffer the
	 * MC writes reconstructed pixels into, wired to two meson-canvas
	 * descriptors (Y + shared UV) so the firmware's LMEM-supplied packed
	 * canvas value (LMEM 0x104/0x105) resolves to a real AXI target.  The
	 * buffer is pre-painted with the 0x55 sentinel so a "MC never wrote"
	 * stall is distinguishable from a real decode (frame-content rule).
	 * Reallocated on each output_canvas write; freed at exit.
	 */
	void *out_buf;
	dma_addr_t out_dma;
	size_t out_size;
	u32 out_aligned_w;	/* ALIGN(w, 64) -- canvas stride + UV offset base */
	u32 out_aligned_h;	/* ALIGN(h, 32) */
	u32 out_canvas_val;	/* packed UV<<16 | UV<<8 | Y for LMEM 0x104/0x105 */
	u8 out_canvas_id[2];	/* [0]=Y, [1]=UV (shared) */
	bool out_configured;

	/*
	 * Last decode-orchestration result (decode node -> decode_result).
	 * All observation registers captured immediately after the trigger
	 * poll returns, so a wedge's frozen state is preserved even if a
	 * later peek races the engine.
	 */
	bool decode_valid;
	bool decode_timed_out;	/* AV_SCRATCH_9 never reached 2 (DONE) */
	u32 dec_iters;		/* trigger-poll iterations consumed */
	u32 dec_scratch9;	/* AV_SCRATCH_9 (2 = DONE) */
	u32 dec_scratchf;	/* AV_SCRATCH_F (0 = ok, 0x21 = MC hang) */
	u32 dec_wrrsp;		/* MCW_DBLK_WRRSP_CNT (bit31+count = MC write stall) */
	u32 dec_mby_mbx;	/* MBY_MBX (MB decode position; advances off 0 = MC ran) */
	u32 dec_mc_status0;	/* MC_STATUS0 */
	u32 dec_dcac;		/* DCAC_MB_COUNT (IQIDCT residual MB count) */
	u32 dec_vififo_level;	/* VLD_MEM_VIFIFO_LEVEL after decode */
};

struct vdec_diag {
	struct dentry *root;		/* /sys/kernel/debug/meson-vdec */
	struct dentry *diag_dir;	/* /sys/kernel/debug/meson-vdec/diag */
	struct vdec_diag_engine vdec1;
};

/*
 * Engine must be "powered" (power rails up, AMRISC reachable) for any
 * diag op except `poweron` itself. Two sources:
 *  - an active V4L2 session owns the engine (core->cur_sess non-NULL)
 *  - the user ran `echo 1 > diag/vdec1/poweron` manually
 * Either gate is sufficient. `poweron` maintains its own additional
 * check that a V4L2 session isn't already holding the engine, so the
 * two paths don't collide.
 */
static bool engine_powered(struct vdec_diag_engine *eng)
{
	return eng->core->cur_sess || eng->manual_powered;
}

static struct vdec_diag_engine *engine_from_file(struct file *f)
{
	return f->f_inode->i_private;
}

/* ---- firmware: write binary blob, load to IMEM, re-arm MPSR ------------- */

static ssize_t vdec_diag_firmware_write(struct file *f, const char __user *ubuf,
					size_t count, loff_t *ppos)
{
	struct vdec_diag_engine *eng = engine_from_file(f);
	struct amvdec_core *core = eng->core;
	struct device *dev = core->dev_dec;
	u32 waited = 1000;
	int ret;

	if (!engine_powered(eng)) {
		dev_warn(dev, "diag: firmware upload rejected -- no active session (power/clock not up)\n");
		return -ENODEV;
	}
	if (count == 0 || count > VDEC_DIAG_MC_SIZE)
		return -E2BIG;

	mutex_lock(&eng->lock);

	memset(eng->fw_buf, 0, eng->fw_buf_size);
	if (copy_from_user(eng->fw_buf, ubuf, count)) {
		ret = -EFAULT;
		goto out;
	}

	dev_info(dev, "diag: firmware hot-upload %zu bytes by pid=%d\n",
		 count, current->pid);

	/* Halt AMRISC, then full VDEC_1 reset (0xfffffffc matches the
	 * vdec_1_start reset mask -- resets the AMRISC CPU itself via
	 * bits 11-12, which is what makes PC restart at 0 after the new
	 * firmware is loaded.  Without the full reset, PC resumes at
	 * wherever the prior firmware was running when MPSR=0 latched
	 * and the new IMEM contents are never executed from PC 0. */
	amvdec_write_dos(core, MPSR, 0);
	amvdec_write_dos(core, CPSR, 0);
	amvdec_write_dos(core, DOS_SW_RESET0, 0xfffffffc);
	amvdec_write_dos(core, DOS_SW_RESET0, 0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);

	amvdec_write_dos(core, IMEM_DMA_ADR, eng->fw_buf_dma);
	amvdec_write_dos(core, IMEM_DMA_COUNT, VDEC_DIAG_MC_SIZE / 4);
	amvdec_write_dos(core, IMEM_DMA_CTRL, IMEM_DMA_CTRL_RUN);

	while (--waited &&
	       (amvdec_read_dos(core, IMEM_DMA_CTRL) & IMEM_DMA_CTRL_START))
		;

	if (waited == 0) {
		dev_err(dev, "diag: firmware hot-upload DMA hang\n");
		ret = -EIO;
		goto out;
	}

	amvdec_write_dos(core, MPSR, 1);

	dev_info(dev, "diag: firmware hot-upload OK -- MPSR=1\n");
	ret = count;

out:
	mutex_unlock(&eng->lock);
	return ret;
}

static const struct file_operations vdec_diag_firmware_fops = {
	.open = simple_open,
	.write = vdec_diag_firmware_write,
	.llseek = noop_llseek,
};

/* ---- reset: pulse MDEC|VLD reset, re-arm MPSR --------------------------- */

static ssize_t vdec_diag_reset_write(struct file *f, const char __user *ubuf,
				     size_t count, loff_t *ppos)
{
	struct vdec_diag_engine *eng = engine_from_file(f);
	struct amvdec_core *core = eng->core;
	struct device *dev = core->dev_dec;
	u32 mode_saved;

	if (!engine_powered(eng)) {
		dev_warn(dev, "diag: reset rejected -- no active session\n");
		return -ENODEV;
	}

	mutex_lock(&eng->lock);

	dev_info(dev, "diag: engine reset by pid=%d\n", current->pid);

	/* Full reset matching vdec_1_start: 0xfffffffc resets the entire
	 * VDEC_1 block including the AMRISC CPU (bits 11-12), which is
	 * required to restart PC at 0.  IMEM contents are NOT cleared
	 * by this reset -- only the CPU state -- so the previously-
	 * uploaded firmware still executes, just from the beginning.
	 *
	 * Block reset DOES clear AV_SCRATCH_8.  The diag firmware reads
	 * mode from AV_SCRATCH_8 at boot (PC 0x0008), so any value
	 * written via the `mode` debugfs file before reset would be
	 * lost across the block reset and the firmware would always
	 * dispatch to mode 0 (alive test).  Snapshot AV_SCRATCH_8
	 * before the block reset and restore it before MPSR=1 so the
	 * mode value survives.  Discovered 2026-04-25 during M4.1.7
	 * probe-1 work; without this, all standalone probe firmwares
	 * have to hardcode their entry point.
	 */
	mode_saved = amvdec_read_dos(core, AV_SCRATCH_8);

	amvdec_write_dos(core, MPSR, 0);
	amvdec_write_dos(core, CPSR, 0);
	amvdec_write_dos(core, DOS_SW_RESET0, 0xfffffffc);
	amvdec_write_dos(core, DOS_SW_RESET0, 0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);

	amvdec_write_dos(core, AV_SCRATCH_8, mode_saved);

	amvdec_write_dos(core, MPSR, 1);

	mutex_unlock(&eng->lock);
	return count;
}

static const struct file_operations vdec_diag_reset_fops = {
	.open = simple_open,
	.write = vdec_diag_reset_write,
	.llseek = noop_llseek,
};

/* ---- state: dump all interesting registers as key=value ----------------- */

static int vdec_diag_state_show(struct seq_file *s, void *unused)
{
	struct vdec_diag_engine *eng = s->private;
	struct amvdec_core *core = eng->core;

	if (!engine_powered(eng)) {
		seq_puts(s, "error=no_active_session\n");
		return 0;
	}

	seq_printf(s, "scratch_0=0x%08x\n", amvdec_read_dos(core, AV_SCRATCH_0));
	seq_printf(s, "scratch_1=0x%08x\n", amvdec_read_dos(core, AV_SCRATCH_1));
	seq_printf(s, "scratch_2=0x%08x\n", amvdec_read_dos(core, AV_SCRATCH_2));
	seq_printf(s, "scratch_3=0x%08x\n", amvdec_read_dos(core, AV_SCRATCH_3));
	seq_printf(s, "scratch_4=0x%08x\n", amvdec_read_dos(core, AV_SCRATCH_4));
	seq_printf(s, "scratch_5=0x%08x\n", amvdec_read_dos(core, AV_SCRATCH_5));
	seq_printf(s, "scratch_6=0x%08x\n", amvdec_read_dos(core, AV_SCRATCH_6));
	seq_printf(s, "scratch_7=0x%08x\n", amvdec_read_dos(core, AV_SCRATCH_7));
	seq_printf(s, "scratch_8=0x%08x\n", amvdec_read_dos(core, AV_SCRATCH_8));
	seq_printf(s, "scratch_9=0x%08x\n", amvdec_read_dos(core, AV_SCRATCH_9));
	seq_printf(s, "scratch_a=0x%08x\n", amvdec_read_dos(core, AV_SCRATCH_A));
	seq_printf(s, "scratch_b=0x%08x\n", amvdec_read_dos(core, AV_SCRATCH_B));
	seq_printf(s, "scratch_c=0x%08x\n", amvdec_read_dos(core, AV_SCRATCH_C));
	seq_printf(s, "scratch_d=0x%08x\n", amvdec_read_dos(core, AV_SCRATCH_D));
	seq_printf(s, "scratch_e=0x%08x\n", amvdec_read_dos(core, AV_SCRATCH_E));
	seq_printf(s, "scratch_f=0x%08x\n", amvdec_read_dos(core, AV_SCRATCH_F));
	seq_printf(s, "vld_status_ctrl=0x%08x\n",
		   amvdec_read_dos(core, VLD_STATUS_CTRL));
	seq_printf(s, "vld_vififo_level=0x%08x\n",
		   amvdec_read_dos(core, VLD_MEM_VIFIFO_LEVEL));
	seq_printf(s, "vld_vififo_wp=0x%08x\n",
		   amvdec_read_dos(core, VLD_MEM_VIFIFO_WP));
	seq_printf(s, "mdec_pic_dc_status=0x%08x\n",
		   amvdec_read_dos(core, MDEC_PIC_DC_STATUS));
	seq_printf(s, "mpsr=0x%08x\n", amvdec_read_dos(core, MPSR));
	/*
	 * MINT_VEC_BASE IS host-readable: it is an AMRISC CR (0x0302) mapped
	 * into DOS MMIO at 0x0c08, exactly like MPC_E (CR 0x0308 -> 0x0c20) and
	 * MPSR above.  (This used to say "cannot read from host side" and was
	 * therefore not dumped, contradicting this file's own header comment
	 * which lists MINT_VEC_BASE among the state registers.  Only the WRITE
	 * side is AMRISC-only, via mtcr.)  Worth dumping: it cold-boots to 0x01
	 * instead of 0x08 on GXL/G12B/SM1, and firmware that does not program it
	 * installs its interrupt vectors at the wrong base -- see the amlogic
	 * corpus ip-characterization.md §2.2 / KI-005.
	 */
	seq_printf(s, "mint_vec_base=0x%08x\n",
		   amvdec_read_dos(core, MINT_VEC_BASE));
	seq_printf(s, "mcpu_intr_msk=0x%08x\n",
		   amvdec_read_dos(core, MCPU_INTR_MSK));
	return 0;
}

static int vdec_diag_state_open(struct inode *i, struct file *f)
{
	return single_open(f, vdec_diag_state_show, i->i_private);
}

static const struct file_operations vdec_diag_state_fops = {
	.open = vdec_diag_state_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/* ---- peek: write offset, read value ------------------------------------- */

static ssize_t vdec_diag_peek_write(struct file *f, const char __user *ubuf,
				    size_t count, loff_t *ppos)
{
	struct vdec_diag_engine *eng = engine_from_file(f);
	char buf[VDEC_DIAG_PEEK_BUF_LEN];
	u32 offset;
	int ret;

	if (count >= sizeof(buf))
		return -E2BIG;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';

	ret = kstrtou32(strim(buf), 0, &offset);
	if (ret)
		return ret;
	if (offset > VDEC_DIAG_DOS_OFFSET_MAX || (offset & 3))
		return -EINVAL;

	mutex_lock(&eng->lock);
	eng->peek_offset = offset;
	eng->peek_armed = true;
	mutex_unlock(&eng->lock);

	return count;
}

static ssize_t vdec_diag_peek_read(struct file *f, char __user *ubuf,
				   size_t count, loff_t *ppos)
{
	struct vdec_diag_engine *eng = engine_from_file(f);
	struct amvdec_core *core = eng->core;
	char buf[VDEC_DIAG_PEEK_BUF_LEN];
	u32 val;
	int len;

	if (*ppos > 0)
		return 0;

	mutex_lock(&eng->lock);
	if (!eng->peek_armed) {
		mutex_unlock(&eng->lock);
		return -EINVAL;
	}
	if (!engine_powered(eng)) {
		mutex_unlock(&eng->lock);
		return -ENODEV;
	}
	val = amvdec_read_dos(core, eng->peek_offset);
	mutex_unlock(&eng->lock);

	len = scnprintf(buf, sizeof(buf), "0x%08x\n", val);
	if (copy_to_user(ubuf, buf, len))
		return -EFAULT;
	*ppos = len;
	return len;
}

static const struct file_operations vdec_diag_peek_fops = {
	.open = simple_open,
	.read = vdec_diag_peek_read,
	.write = vdec_diag_peek_write,
	.llseek = noop_llseek,
};

/* ---- poke: write "offset value" ----------------------------------------- */

static ssize_t vdec_diag_poke_write(struct file *f, const char __user *ubuf,
				    size_t count, loff_t *ppos)
{
	struct vdec_diag_engine *eng = engine_from_file(f);
	struct amvdec_core *core = eng->core;
	char buf[VDEC_DIAG_PEEK_BUF_LEN];
	u32 offset, value;
	char *sep;
	int ret;

	if (count >= sizeof(buf))
		return -E2BIG;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';

	sep = strchr(buf, ' ');
	if (!sep)
		return -EINVAL;
	*sep = '\0';

	ret = kstrtou32(strim(buf), 0, &offset);
	if (ret)
		return ret;
	ret = kstrtou32(strim(sep + 1), 0, &value);
	if (ret)
		return ret;
	if (offset > VDEC_DIAG_DOS_OFFSET_MAX || (offset & 3))
		return -EINVAL;

	if (!engine_powered(eng)) {
		dev_warn(core->dev_dec, "diag: poke rejected -- no active session\n");
		return -ENODEV;
	}

	mutex_lock(&eng->lock);
	dev_info(core->dev_dec, "diag: poke 0x%04x = 0x%08x by pid=%d\n",
		 offset, value, current->pid);
	amvdec_write_dos(core, offset, value);
	mutex_unlock(&eng->lock);

	return count;
}

static const struct file_operations vdec_diag_poke_fops = {
	.open = simple_open,
	.write = vdec_diag_poke_write,
	.llseek = noop_llseek,
};

/* ---- poweron: bring the VDEC_1 engine up without a V4L2 session -------- */
/*
 * Minimal VDEC_1 power-on for characterization sweeps.  Mirrors the
 * power/clock/reset path of vdec_1_start but does NOT load firmware or
 * initialize any codec state -- just brings the engine to a state
 * where DOS MMIO reads return meaningful values.
 *
 * Usage:
 *   echo 1 > /sys/kernel/debug/meson-vdec/diag/vdec1/poweron   # bring up
 *   echo 0 > /sys/kernel/debug/meson-vdec/diag/vdec1/poweron   # tear down
 *
 * The minimal sequence is:
 *   1. enable vdec_1_clk at max rate
 *   2. clear AO_RTI_GEN_PWR_SLEEP0 VDEC_1 power bits
 *   3. DOS_SW_RESET0 = 0xfffffffc pulse
 *   4. clear AO_RTI_GEN_PWR_ISO0 VDEC_1 isolation
 *   5. DOS_GCLK_EN0 = DOS_GCLK_EN0_VDEC1
 *
 * The SM1 path (used on the 42b6a1870bb4 bringup board) uses the
 * AO_RTI_GEN_PWR_*_SM1 register bit positions.  Other platforms need
 * their respective bit masks.  For the first sweep pass we only
 * support SM1; other platforms can be added on demand.
 */
#include <linux/clk.h>
#include <linux/regmap.h>

#include "vdec_platform.h"	/* VDEC_REVISION_* */
#include "vdec_hevc.h"		/* vdec_hevc_clk_get (G12A+ MDEC DC cache) */

/* AO_RTI_GEN_PWR_* byte offsets and per-platform VDEC_1 + HEVC power /
 * isolation bit masks.  Copied from vdec_1.c:18-39 to keep this file
 * self-contained.  The bit positions DIFFER per SoC family, and on
 * non-SM1 the ISO0 positions differ from SLEEP0 (see vdec_1.c).
 * vdec_diag_poweron selects the right set via core->platform->revision
 * (mirroring vdec_1_poweron); using the SM1 masks on GXL/G12A clears
 * the wrong bits, leaves VDEC_1 gated, and hangs the DOS bus on the
 * next write -- the 2026-06-01 GXL diag-poweron hang.
 *
 * On G12A+ the MDEC DC cache shares the HEVC power domain
 * (vdec_1.c:480-484), so HEVC must be powered + its clock enabled
 * alongside VDEC_1; on GXL/GXLX the rails are independent (VDEC_1
 * only). */
#define AO_RTI_GEN_PWR_SLEEP0	0xe8
#define AO_RTI_GEN_PWR_ISO0	0xec
#define GEN_PWR_VDEC_1		(BIT(3) | BIT(2))
#define GEN_PWR_VDEC_HEVC	(BIT(7) | BIT(6))
#define GEN_PWR_VDEC_1_SM1	BIT(1)
#define GEN_PWR_VDEC_HEVC_SM1	BIT(2)
#define GEN_PWR_ISO0_VDEC_1	(BIT(7) | BIT(6))
#define GEN_PWR_ISO0_HEVC	(BIT(11) | BIT(10))
#define GEN_PWR_ISO0_VDEC_1_SM1	BIT(1)
#define GEN_PWR_ISO0_HEVC_SM1	BIT(2)
#define DOS_GCLK_EN0_VDEC1	0x3ff
#define MDEC_SW_RESET_MDEC_CORE	BIT(7)

/*
 * Diag VDEC_1 poweron -- mirrors vdec_1.c::vdec_1_start's power/clock/
 * reset sequence up to line 569, minus the firmware-load and codec
 * init.  An earlier version was a "minimal" subset of this sequence
 * and failed to produce a running AMRISC -- the missing steps
 * (HEVC power rail on SM1, 100us isolation-settle delay, secondary
 * MDEC|VLD reset that actually clears IMEM+pipeline, PSCALE disable)
 * were all load-bearing.  Match the full sequence.
 */
static int vdec_diag_poweron(struct amvdec_core *core)
{
	int ret;

	ret = clk_prepare_enable(core->vdec_1_clk);
	if (ret)
		return ret;
	clk_set_rate(core->vdec_1_clk, 667000000);

	/*
	 * Enable the DOS register-bus + parser clocks (mirrors vdec_poweron,
	 * vdec.c:392/396). Without dos_clk the ff620000 DOS MMIO block is
	 * UNCLOCKED: every DOS register read returns 0 and every write is
	 * dropped. A session-less diag run then silently produces the all-zero
	 * decode_result + all-0x55 output -- indistinguishable from a real
	 * MC-output wedge -- with no firmware actually running. The diag path
	 * must clock the bus itself; relying on a concurrent V4L2 session to
	 * hold dos_clk on made every session-less capture nondeterministically
	 * confounded (2026-07-18).
	 */
	ret = clk_prepare_enable(core->dos_parser_clk);
	if (ret) {
		clk_disable_unprepare(core->vdec_1_clk);
		return ret;
	}
	ret = clk_prepare_enable(core->dos_clk);
	if (ret) {
		clk_disable_unprepare(core->dos_parser_clk);
		clk_disable_unprepare(core->vdec_1_clk);
		return ret;
	}

	/* On G12A+, the MDEC DC cache hardware is inside the HEVC power
	 * domain; VDEC_1 needs HEVC clocks active (mirrors vdec_1.c). */
	if (core->platform->revision >= VDEC_REVISION_G12A) {
		ret = vdec_hevc_clk_get(core);
		if (ret) {
			clk_disable_unprepare(core->dos_clk);
			clk_disable_unprepare(core->dos_parser_clk);
			clk_disable_unprepare(core->vdec_1_clk);
			return ret;
		}
	}

	/* Clear sleep bits: SM1 / G12A+ / GXL each use different masks
	 * (mirrors vdec_1.c).  SM1 must be checked first since SM1 also
	 * satisfies >= VDEC_REVISION_G12A. */
	if (core->platform->revision == VDEC_REVISION_SM1)
		regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_SLEEP0,
				   GEN_PWR_VDEC_1_SM1 | GEN_PWR_VDEC_HEVC_SM1, 0);
	else if (core->platform->revision >= VDEC_REVISION_G12A)
		regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_SLEEP0,
				   GEN_PWR_VDEC_1 | GEN_PWR_VDEC_HEVC, 0);
	else
		regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_SLEEP0,
				   GEN_PWR_VDEC_1, 0);
	usleep_range(10, 20);

	/* First reset */
	amvdec_write_dos(core, DOS_SW_RESET0, 0xfffffffc);
	amvdec_write_dos(core, DOS_SW_RESET0, 0);

	amvdec_write_dos(core, DOS_GCLK_EN0, DOS_GCLK_EN0_VDEC1);
	amvdec_write_dos(core, DOS_MEM_PD_VDEC, 0);

	/* On G12A+, also power on HEVC memories + enable HEVC clock
	 * gating (the MDEC DC cache lives here). */
	if (core->platform->revision >= VDEC_REVISION_G12A) {
		amvdec_write_dos(core, DOS_SW_RESET3, 0xffffffff);
		amvdec_write_dos(core, DOS_SW_RESET3, 0);
		amvdec_write_dos(core, DOS_GCLK_EN3, 0xffffffff);
		amvdec_write_dos(core, DOS_MEM_PD_HEVC, 0);
	}

	/* Remove isolation: SM1 / G12A+ / GXL distinct masks; ISO0 bit
	 * positions differ from SLEEP0 on non-SM1 (mirrors vdec_1.c). */
	if (core->platform->revision == VDEC_REVISION_SM1)
		regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_ISO0,
				   GEN_PWR_ISO0_VDEC_1_SM1 | GEN_PWR_ISO0_HEVC_SM1, 0);
	else if (core->platform->revision >= VDEC_REVISION_G12A)
		regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_ISO0,
				   GEN_PWR_ISO0_HEVC | GEN_PWR_ISO0_VDEC_1, 0);
	else
		regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_ISO0,
				   GEN_PWR_ISO0_VDEC_1, 0);
	usleep_range(100, 200);

	/* DOS top-level register clears (matches vdec_1_start:526-529) */
	amvdec_write_dos(core, DOS_VDEC_MCRCC_STALL_CTRL, 0);
	amvdec_clear_dos_bits(core, MDEC_PIC_DC_CTRL, MDEC_PIC_DC_CTRL_OLD_MODE);

	/* Second reset (MDEC + VLD) after isolation removal.  This is
	 * what actually clears IMEM contents and the MDEC pipeline --
	 * the first reset before isolation removal doesn't propagate
	 * fully. */
	amvdec_write_dos(core, DOS_SW_RESET0,
			 DOS_SW_RESET0_MDEC | DOS_SW_RESET0_VLD);
	amvdec_write_dos(core, DOS_SW_RESET0, 0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	/*
	 * MC-block reset (mirrors codec_mpeg12_start, codec_mpeg12.c:158-162).
	 * The reset above clears only MDEC|VLD, not the MC block, so a stale MC
	 * write-response FSM can persist and never complete -- the exact
	 * MC-output-write wedge the libre firmware hits. The stateful
	 * driver-context does this reset; provisioning it here runs the
	 * libre-firmware decode against the same MC-block state (firmware-vs-
	 * driver-context isolation for the MPEG-2 wedge -- frontier §9.18.7).
	 */
	amvdec_write_dos(core, DOS_SW_RESET0, DOS_SW_RESET0_MC);
	amvdec_write_dos(core, DOS_SW_RESET0, 0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_write_dos(core, MDEC_SW_RESET, MDEC_SW_RESET_MDEC_CORE);
	amvdec_write_dos(core, MDEC_SW_RESET, 0);

	/* Disable PSCALE (matches vdec_1_start:569) */
	amvdec_write_dos(core, PSCALE_CTRL, 0);

	return 0;
}

static void vdec_diag_poweroff(struct amvdec_core *core)
{
	amvdec_write_dos(core, DOS_SW_RESET0, DOS_SW_RESET0_VDEC1);
	amvdec_write_dos(core, DOS_SW_RESET0, 0);

	/* Re-isolate + power down VDEC_1 with the platform's masks (only
	 * VDEC_1 -- HEVC is left as-is; a reboot fully resets either way). */
	if (core->platform->revision == VDEC_REVISION_SM1)
		regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_ISO0,
				   GEN_PWR_ISO0_VDEC_1_SM1, GEN_PWR_ISO0_VDEC_1_SM1);
	else
		regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_ISO0,
				   GEN_PWR_ISO0_VDEC_1, GEN_PWR_ISO0_VDEC_1);
	amvdec_write_dos(core, DOS_MEM_PD_VDEC, 0xffffffff);
	if (core->platform->revision == VDEC_REVISION_SM1)
		regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_SLEEP0,
				   GEN_PWR_VDEC_1_SM1, GEN_PWR_VDEC_1_SM1);
	else
		regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_SLEEP0,
				   GEN_PWR_VDEC_1, GEN_PWR_VDEC_1);

	clk_disable_unprepare(core->dos_clk);
	clk_disable_unprepare(core->dos_parser_clk);
	clk_disable_unprepare(core->vdec_1_clk);
}

static ssize_t vdec_diag_poweron_write(struct file *f, const char __user *ubuf,
				       size_t count, loff_t *ppos)
{
	struct vdec_diag_engine *eng = engine_from_file(f);
	struct amvdec_core *core = eng->core;
	char buf[VDEC_DIAG_PEEK_BUF_LEN];
	u32 value;
	int ret;

	if (count >= sizeof(buf))
		return -E2BIG;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';
	ret = kstrtou32(strim(buf), 0, &value);
	if (ret)
		return ret;

	if (core->cur_sess) {
		dev_warn(core->dev_dec,
			 "diag: poweron/poweroff refused -- V4L2 session active\n");
		return -EBUSY;
	}

	mutex_lock(&eng->lock);
	if (value) {
		dev_info(core->dev_dec, "diag: manual poweron by pid=%d\n",
			 current->pid);
		ret = vdec_diag_poweron(core);
		if (ret) {
			mutex_unlock(&eng->lock);
			return ret;
		}
		eng->manual_powered = true;
	} else {
		eng->manual_powered = false;
		dev_info(core->dev_dec, "diag: manual poweroff by pid=%d\n",
			 current->pid);
		vdec_diag_poweroff(core);
	}
	mutex_unlock(&eng->lock);
	return count;
}

static const struct file_operations vdec_diag_poweron_fops = {
	.open = simple_open,
	.write = vdec_diag_poweron_write,
	.llseek = noop_llseek,
};

/* ---- mode/cmd: shortcuts for AV_SCRATCH_8 / AV_SCRATCH_9 ---------------- */

static ssize_t vdec_diag_scratch_write(struct file *f, const char __user *ubuf,
				       size_t count, loff_t *ppos, u32 reg,
				       const char *name)
{
	struct vdec_diag_engine *eng = engine_from_file(f);
	struct amvdec_core *core = eng->core;
	char buf[VDEC_DIAG_PEEK_BUF_LEN];
	u32 value;
	int ret;

	if (count >= sizeof(buf))
		return -E2BIG;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';

	ret = kstrtou32(strim(buf), 0, &value);
	if (ret)
		return ret;

	if (!engine_powered(eng)) {
		dev_warn(core->dev_dec, "diag: %s write rejected -- no active session\n",
			 name);
		return -ENODEV;
	}

	mutex_lock(&eng->lock);
	dev_info(core->dev_dec, "diag: %s = 0x%08x by pid=%d\n",
		 name, value, current->pid);
	amvdec_write_dos(core, reg, value);
	mutex_unlock(&eng->lock);

	return count;
}

static ssize_t vdec_diag_mode_write(struct file *f, const char __user *ubuf,
				    size_t count, loff_t *ppos)
{
	return vdec_diag_scratch_write(f, ubuf, count, ppos, AV_SCRATCH_8, "mode");
}

static ssize_t vdec_diag_cmd_write(struct file *f, const char __user *ubuf,
				   size_t count, loff_t *ppos)
{
	return vdec_diag_scratch_write(f, ubuf, count, ppos, AV_SCRATCH_9, "cmd");
}

static const struct file_operations vdec_diag_mode_fops = {
	.open = simple_open,
	.write = vdec_diag_mode_write,
	.llseek = noop_llseek,
};

static const struct file_operations vdec_diag_cmd_fops = {
	.open = simple_open,
	.write = vdec_diag_cmd_write,
	.llseek = noop_llseek,
};

/* ---- workspace_paddr: read-only physical address of the LMEM workspace -- */
/*
 * The LMEM_DMA characterization workspace is a 4 KiB DMA-coherent buffer
 * allocated at engine init.  Userspace reads its physical address here
 * so it can pass that exact address to lmem_dma_test (or omit the adr
 * arg, which uses the same workspace).  The address is stable for the
 * lifetime of the diag engine.
 */
static ssize_t vdec_diag_workspace_paddr_read(struct file *f, char __user *ubuf,
					      size_t count, loff_t *ppos)
{
	struct vdec_diag_engine *eng = engine_from_file(f);
	char buf[VDEC_DIAG_PEEK_BUF_LEN];
	int len;

	if (*ppos > 0)
		return 0;

	len = scnprintf(buf, sizeof(buf), "0x%08x\n",
			(u32)eng->workspace_dma);
	if (len > count)
		len = count;
	if (copy_to_user(ubuf, buf, len))
		return -EFAULT;
	*ppos = len;
	return len;
}

static const struct file_operations vdec_diag_workspace_paddr_fops = {
	.open = simple_open,
	.read = vdec_diag_workspace_paddr_read,
	.llseek = noop_llseek,
};

/* ---- workspace_pattern: write seed -> fill 1024 words; read returns seed - */
/*
 * Userspace writes a 32-bit hex seed; the workspace is filled with
 * (seed + i*4) for i in [0, 1024).  This is the pre-DMA fixture pattern
 * the LMEM characterization sweep loads before firing lmem_dma_test, so
 * post-DMA inspection of LMEM contents (via lda probes from a follow-up
 * firmware run) can verify the DMA actually transferred data.
 *
 * Reading returns the most recent seed (or "0" if no write has occurred).
 */
static ssize_t vdec_diag_workspace_pattern_write(struct file *f,
						 const char __user *ubuf,
						 size_t count, loff_t *ppos)
{
	struct vdec_diag_engine *eng = engine_from_file(f);
	struct amvdec_core *core = eng->core;
	char buf[VDEC_DIAG_PEEK_BUF_LEN];
	u32 seed, *words;
	int ret, i;

	if (count >= sizeof(buf))
		return -E2BIG;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';

	ret = kstrtou32(strim(buf), 0, &seed);
	if (ret)
		return ret;

	mutex_lock(&eng->lock);
	words = eng->workspace_buf;
	for (i = 0; i < VDEC_DIAG_WORKSPACE_WORDS; i++)
		words[i] = seed + (u32)(i * 4);
	eng->workspace_seed = seed;
	dev_info(core->dev_dec,
		 "diag: workspace pattern seed=0x%08x (%u words) by pid=%d\n",
		 seed, VDEC_DIAG_WORKSPACE_WORDS, current->pid);
	mutex_unlock(&eng->lock);

	return count;
}

static ssize_t vdec_diag_workspace_pattern_read(struct file *f, char __user *ubuf,
						size_t count, loff_t *ppos)
{
	struct vdec_diag_engine *eng = engine_from_file(f);
	char buf[VDEC_DIAG_PEEK_BUF_LEN];
	u32 seed;
	int len;

	if (*ppos > 0)
		return 0;

	mutex_lock(&eng->lock);
	seed = eng->workspace_seed;
	mutex_unlock(&eng->lock);

	len = scnprintf(buf, sizeof(buf), "0x%08x\n", seed);
	if (len > count)
		len = count;
	if (copy_to_user(ubuf, buf, len))
		return -EFAULT;
	*ppos = len;
	return len;
}

static const struct file_operations vdec_diag_workspace_pattern_fops = {
	.open = simple_open,
	.read = vdec_diag_workspace_pattern_read,
	.write = vdec_diag_workspace_pattern_write,
	.llseek = noop_llseek,
};

/* ---- workspace_write / workspace_read: RAW access to the LMEM-DMA staging
 *      buffer.  workspace_pattern only writes a seed+i*4 pattern; these give
 *      arbitrary-byte access so libre-fw-probe upload-lmem can seed an
 *      arbitrary LMEM parameter block (workspace_write the bytes, then
 *      lmem_dma_test DDR->LMEM), and so a DDR->LMEM->DDR round-trip can be
 *      validated (workspace_read back).  No engine power needed -- pure DDR. */
static ssize_t vdec_diag_workspace_write(struct file *f, const char __user *ubuf,
					 size_t count, loff_t *ppos)
{
	struct vdec_diag_engine *eng = engine_from_file(f);
	struct amvdec_core *core = eng->core;

	if (*ppos + count > VDEC_DIAG_WORKSPACE_SIZE)
		return -ENOSPC;

	mutex_lock(&eng->lock);
	if (copy_from_user((u8 *)eng->workspace_buf + *ppos, ubuf, count)) {
		mutex_unlock(&eng->lock);
		return -EFAULT;
	}
	dev_info(core->dev_dec,
		 "diag: workspace_write +%zu bytes at off %lld by pid=%d\n",
		 count, (long long)*ppos, current->pid);
	mutex_unlock(&eng->lock);

	*ppos += count;
	return count;
}

static ssize_t vdec_diag_workspace_read(struct file *f, char __user *ubuf,
					size_t count, loff_t *ppos)
{
	struct vdec_diag_engine *eng = engine_from_file(f);
	ssize_t ret;

	mutex_lock(&eng->lock);
	ret = simple_read_from_buffer(ubuf, count, ppos, eng->workspace_buf,
				      VDEC_DIAG_WORKSPACE_SIZE);
	mutex_unlock(&eng->lock);
	return ret;
}

static const struct file_operations vdec_diag_workspace_write_fops = {
	.open = simple_open,
	.write = vdec_diag_workspace_write,
	.llseek = noop_llseek,
};

static const struct file_operations vdec_diag_workspace_read_fops = {
	.open = simple_open,
	.read = vdec_diag_workspace_read,
	.llseek = noop_llseek,
};

/* ---- lmem_dma_test: fire one LMEM_DMA pulse, capture timing + WRRSP ----- */
/*
 * Format: "<ctrl_hex> <count_dec> [<adr_hex>]"
 *   ctrl  -- LMEM_DMA_CTRL value to write (full 32-bit, hardware uses low 16)
 *   count -- LMEM_DMA_COUNT value (number of 32-bit words; <=1024 with default
 *            workspace, but accepted up to 4096 for explicit-adr fires)
 *   adr   -- optional physical address for LMEM_DMA_ADR; omitting uses the
 *            pre-allocated workspace_dma
 *
 * Behavior:
 *   1. Write ADR, COUNT, CTRL (in that order; CTRL write triggers the engine).
 *   2. Poll CTRL & 0x8000 for clear, with a 10000-iter / ~10 ms cap.
 *   3. After bit 15 clears (or budget expires), read WRRSP_LMEM.
 *   4. Stash {ctrl, count, adr, hang_flag, iters, wrrsp, ms} for lmem_dma_result.
 *
 * Returns count from write even on engine hang -- the hang itself is data
 * the characterization sweep needs to record.  Returns -ENODEV / -EINVAL /
 * -EFAULT for parse / power-state / range-check failures (no result stash).
 */
static ssize_t vdec_diag_lmem_dma_test_write(struct file *f,
					     const char __user *ubuf,
					     size_t count, loff_t *ppos)
{
	struct vdec_diag_engine *eng = engine_from_file(f);
	struct amvdec_core *core = eng->core;
	struct device *dev = core->dev_dec;
	char buf[VDEC_DIAG_LMEM_TEST_BUF_LEN];
	char *p, *tok;
	u32 ctrl, dma_count, adr;
	bool adr_explicit = false;
	ktime_t t_start, t_end;
	u32 iters;
	int ret;

	if (count >= sizeof(buf))
		return -E2BIG;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';
	p = strim(buf);

	tok = strsep(&p, " \t");
	if (!tok || !*tok)
		return -EINVAL;
	ret = kstrtou32(tok, 0, &ctrl);
	if (ret)
		return ret;

	tok = strsep(&p, " \t");
	if (!tok || !*tok)
		return -EINVAL;
	ret = kstrtou32(tok, 0, &dma_count);
	if (ret)
		return ret;

	tok = strsep(&p, " \t");
	if (tok && *tok) {
		ret = kstrtou32(tok, 0, &adr);
		if (ret)
			return ret;
		adr_explicit = true;
	} else {
		adr = (u32)eng->workspace_dma;
	}

	/*
	 * count is bounded by the workspace size when using the default
	 * adr (workspace is 4 KiB = 1024 words).  For an explicit user-
	 * provided adr the user owns lifetime of that buffer, so we still
	 * accept up to 4096 words but never larger -- nothing about the
	 * engine has been characterized for >16 KiB transfers and the
	 * harness is for characterization, not bulk DMA.
	 */
	if (dma_count == 0)
		return -EINVAL;
	if (!adr_explicit && dma_count > VDEC_DIAG_WORKSPACE_WORDS)
		return -EINVAL;
	if (dma_count > 4096)
		return -EINVAL;

	if (!engine_powered(eng)) {
		dev_warn(dev, "diag: lmem_dma_test rejected -- engine not powered\n");
		return -ENODEV;
	}

	mutex_lock(&eng->lock);

	dev_info(dev,
		 "diag: lmem_dma_test ctrl=0x%08x count=%u adr=0x%08x%s by pid=%d\n",
		 ctrl, dma_count, adr,
		 adr_explicit ? " (explicit)" : " (workspace)",
		 current->pid);

	amvdec_write_dos(core, LMEM_DMA_ADR, adr);
	amvdec_write_dos(core, LMEM_DMA_COUNT, dma_count);

	t_start = ktime_get();
	amvdec_write_dos(core, LMEM_DMA_CTRL, ctrl);

	for (iters = 0; iters < VDEC_DIAG_LMEM_POLL_MAX; iters++) {
		if (!(amvdec_read_dos(core, LMEM_DMA_CTRL) & LMEM_DMA_CTRL_START))
			break;
		usleep_range(1, 2);
	}
	t_end = ktime_get();

	eng->last_result_valid = true;
	eng->last_result_hang = (iters >= VDEC_DIAG_LMEM_POLL_MAX);
	eng->last_ctrl = ctrl;
	eng->last_count = dma_count;
	eng->last_adr = adr;
	eng->last_iters = iters;
	eng->last_wrrsp = amvdec_read_dos(core, WRRSP_LMEM);
	eng->last_ms = (u32)ktime_to_ms(ktime_sub(t_end, t_start));

	dev_info(dev,
		 "diag: lmem_dma_test result=%s iters=%u wrrsp=0x%08x ms=%u\n",
		 eng->last_result_hang ? "hang" : "ok",
		 eng->last_iters, eng->last_wrrsp, eng->last_ms);

	mutex_unlock(&eng->lock);

	return count;
}

static const struct file_operations vdec_diag_lmem_dma_test_fops = {
	.open = simple_open,
	.write = vdec_diag_lmem_dma_test_write,
	.llseek = noop_llseek,
};

/* ---- lmem_dma_result: read back the most recent lmem_dma_test result ---- */
/*
 * Output is one key=value line per attribute.  If no lmem_dma_test write
 * has happened since boot, returns "result=noinit\n" only.
 */
static int vdec_diag_lmem_dma_result_show(struct seq_file *s, void *unused)
{
	struct vdec_diag_engine *eng = s->private;

	mutex_lock(&eng->lock);
	if (!eng->last_result_valid) {
		seq_puts(s, "result=noinit\n");
		mutex_unlock(&eng->lock);
		return 0;
	}

	seq_printf(s, "result=%s\n", eng->last_result_hang ? "hang" : "ok");
	seq_printf(s, "ctrl=0x%08x\n", eng->last_ctrl);
	seq_printf(s, "count=%u\n", eng->last_count);
	seq_printf(s, "adr=0x%08x\n", eng->last_adr);
	seq_printf(s, "iters=%u\n", eng->last_iters);
	seq_printf(s, "wrrsp=0x%08x\n", eng->last_wrrsp);
	seq_printf(s, "ms=%u\n", eng->last_ms);
	mutex_unlock(&eng->lock);
	return 0;
}

static int vdec_diag_lmem_dma_result_open(struct inode *i, struct file *f)
{
	return single_open(f, vdec_diag_lmem_dma_result_show, i->i_private);
}

static const struct file_operations vdec_diag_lmem_dma_result_fops = {
	.open = vdec_diag_lmem_dma_result_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/* ---- vififo_push: pre-fill the VLD VIFIFO ring with a raw bitstream ----- */
/*
 * Feed a bitstream into the VDEC_1 VLD VIFIFO WITHOUT the esparser or a
 * V4L2 session -- the board half of the stateless co-validation loop
 * (libre-fw-probe push-vififo):  cat bitstream.es > diag/vdec1/vififo_push
 *
 * The bytes are copied into a diag-owned DMA-coherent ring; the VLD VIFIFO
 * pointers are programmed in MANUAL buffer-control mode mirroring
 * vdec_1_stbuf_power_up() (vdec_1.c) -- START/CURR = ring base, WP =
 * base + total pushed.  Once fill_en is on, the VLD immediately prefetches
 * ~512 bytes from the ring into its internal FIFO (advancing RP), so the
 * `state` node's vld_vififo_level reads back about (total - 512); the full
 * payload is available to the firmware (internal FIFO + ring) and WP =
 * base + total confirms the whole push landed.  Verified on SM1 2026-06-11:
 * push 4096 -> level 0xe00, wp base+0x1000; push 1000 -> level 0x1e8.
 * Chunked writes (a multi-write `cat`) append at *ppos; the ring is
 * (re)initialised on the first chunk of a stream (*ppos == 0).
 */
static ssize_t vdec_diag_vififo_push_write(struct file *f, const char __user *ubuf,
					   size_t count, loff_t *ppos)
{
	struct vdec_diag_engine *eng = engine_from_file(f);
	struct amvdec_core *core = eng->core;
	u32 base = (u32)eng->vififo_dma;
	u32 wp;

	if (!engine_powered(eng)) {
		dev_warn(core->dev_dec,
			 "diag: vififo_push rejected -- engine not powered\n");
		return -ENODEV;
	}
	/* END_PTR = base + size - 8, so cap the fill 8 bytes short of the ring. */
	if (*ppos + count > VDEC_DIAG_VIFIFO_SIZE - 8)
		return -ENOSPC;

	mutex_lock(&eng->lock);

	if (copy_from_user((u8 *)eng->vififo_buf + *ppos, ubuf, count)) {
		mutex_unlock(&eng->lock);
		return -EFAULT;
	}

	if (*ppos == 0) {
		/* Ring init -- mirror vdec_1_stbuf_power_up() with the diag ring. */
		amvdec_write_dos(core, VLD_MEM_VIFIFO_CONTROL, 0);
		amvdec_write_dos(core, VLD_MEM_VIFIFO_WRAP_COUNT, 0);
		amvdec_write_dos(core, POWER_CTL_VLD, POWER_CTL_VLD_ENABLE);
		amvdec_write_dos(core, VLD_MEM_VIFIFO_START_PTR, base);
		amvdec_write_dos(core, VLD_MEM_VIFIFO_CURR_PTR, base);
		amvdec_write_dos(core, VLD_MEM_VIFIFO_END_PTR,
				 base + VDEC_DIAG_VIFIFO_SIZE - 8);
		amvdec_write_dos_bits(core, VLD_MEM_VIFIFO_CONTROL, 1);
		amvdec_clear_dos_bits(core, VLD_MEM_VIFIFO_CONTROL, 1);
		amvdec_write_dos(core, VLD_MEM_VIFIFO_BUF_CNTL, MEM_BUFCTRL_MANUAL);
		amvdec_write_dos(core, VLD_MEM_VIFIFO_WP, base);
		amvdec_write_dos_bits(core, VLD_MEM_VIFIFO_BUF_CNTL, 1);
		amvdec_clear_dos_bits(core, VLD_MEM_VIFIFO_BUF_CNTL, 1);
		amvdec_write_dos_bits(core, VLD_MEM_VIFIFO_CONTROL,
				      (0x11 << MEM_FIFO_CNT_BIT) | MEM_FILL_ON_LEVEL |
				      MEM_CTRL_FILL_EN | MEM_CTRL_EMPTY_EN);
		eng->vififo_len = 0;
	}

	eng->vififo_len = (u32)*ppos + count;

	/* Advance WP past the pushed data and commit it (MANUAL mode pulse). */
	wp = base + eng->vififo_len;
	amvdec_write_dos(core, VLD_MEM_VIFIFO_WP, wp);
	amvdec_write_dos_bits(core, VLD_MEM_VIFIFO_BUF_CNTL, 1);
	amvdec_clear_dos_bits(core, VLD_MEM_VIFIFO_BUF_CNTL, 1);

	dev_info(core->dev_dec,
		 "diag: vififo_push +%zu bytes (total=%u wp=0x%08x level=0x%08x) by pid=%d\n",
		 count, eng->vififo_len, wp,
		 amvdec_read_dos(core, VLD_MEM_VIFIFO_LEVEL), current->pid);

	mutex_unlock(&eng->lock);
	*ppos += count;
	return count;
}

static const struct file_operations vdec_diag_vififo_push_fops = {
	.open = simple_open,
	.write = vdec_diag_vififo_push_write,
	.llseek = noop_llseek,
};

/* ======================================================================== *
 *  MPEG-2 decode-fixture harness (mpeg12-mc-output-frontier.md §7.2)
 *
 *  Three nodes drive a full stateless MPEG-2 I-frame decode through the
 *  running libre firmware WITHOUT a V4L2 session, so the MCRCC null-canvas
 *  fix (§5a) can be A/B'd on-board:
 *
 *    output_canvas  (write "<W> <H>")   configure an NV12 output canvas
 *                                       (0x55 sentinel-prefilled), expose it
 *    output_canvas  (read)              y/uv paddr + packed canvas value
 *    output_read    (read, raw bytes)   pull the decoded buffer for MD5/PSNR
 *    decode         (write param line)  pack the LMEM block, DMA it to LMEM
 *                                       0x100, trigger AV_SCRATCH_9=1, poll,
 *                                       snapshot the MC observation regs
 *    decode_result  (read)              the snapshot (WRRSP/MBY_MBX/... )
 *
 *  The firmware's LMEM param protocol + trigger contract (all offsets are
 *  16-bit LMEM WORD indices) are transcribed from libre_sm1_vdec1.S and
 *  packed byte-correct here so a failed decode localizes to the firmware,
 *  not a garbled param block (co-dev rule 13 + localize-the-layer).
 * ======================================================================== */

/* LMEM param window: firmware reads codec_id..mpeg1_2_reg from word 0x100.
 * 0x100 is 8-byte (4-word) chunk aligned, so the LMEM_DMA 64-bit endian
 * swap (key-findings XOR-6) maps our 0-based workspace image onto LMEM
 * words 0x100.. cleanly via lmem_put16 with relative indices. */
#define LMEM_PARAM_START	0x100	/* LMEM word offset of the param block */
#define LMEM_PARAM_WORDS	0xA0	/* words DMA'd (0x100..0x19F, 4-word mult) */
#define LMEM_DMA_CTRL_MEM2LMEM	BIT(14)	/* direction: memory -> LMEM */

/* MPEG-2 default quantiser matrices (ISO 13818-2 Tables, raster order).
 * The firmware uploads intra_quant[64] @ LMEM 0x118 and non_intra_quant[64]
 * @ LMEM 0x158 to INTRA_QUANT_MATRIX / NON_I_QUANT_MATRIX.  Supplying the
 * spec defaults covers streams that do not carry custom matrices.  Scan
 * order affects PSNR only, never MC retire -- flagged tunable in §7.2. */
static const u8 mpeg2_default_intra_quant[64] = {
	 8, 16, 19, 22, 26, 27, 29, 34,
	16, 16, 22, 24, 27, 29, 34, 37,
	19, 22, 26, 27, 29, 34, 34, 38,
	22, 22, 26, 27, 29, 34, 37, 40,
	22, 26, 27, 29, 32, 35, 40, 48,
	26, 27, 29, 32, 35, 40, 48, 58,
	26, 27, 29, 34, 38, 46, 56, 69,
	27, 29, 35, 38, 46, 56, 69, 83,
};

/*
 * lmem_put16 -- place a 16-bit value at LMEM word index `idx` (relative to
 * the DMA'd block base) inside a u32 memory workspace, PRE-compensating the
 * LMEM_DMA 64-bit (8-byte) endian reversal so the value lands at the intended
 * LMEM word.  Byte-identical to the production codec_h264_libre.c helper.
 * Never write ws[i] directly -- the swap would scramble it (key-findings §4a).
 */
static inline void lmem_put16(u32 *ws, int idx, u16 val)
{
	int chunk = idx / 4;
	int pos = idx % 4;
	int ws_idx = chunk * 2 + (pos < 2 ? 1 : 0);
	bool hi = (pos == 0 || pos == 2);

	if (hi)
		ws[ws_idx] = (ws[ws_idx] & 0xffffU) | ((u32)val << 16);
	else
		ws[ws_idx] = (ws[ws_idx] & 0xffff0000U) | val;
}

/* Configure one meson-canvas descriptor (mirrors vdec_helpers set_canvas_nv12:
 * config + the G12A+ per-canvas blk32 shadow write). */
static void diag_canvas_config(struct amvdec_core *core, u8 id, u32 addr,
			       u32 stride, u32 height)
{
	meson_canvas_config(core->canvas, id, addr, stride, height,
			    MESON_CANVAS_WRAP_NONE, MESON_CANVAS_BLKMODE_LINEAR,
			    MESON_CANVAS_ENDIAN_SWAP64);
	if (core->platform->revision >= VDEC_REVISION_G12A)
		amvdec_write_dos(core, VDEC_ASSIST_CANVAS_BLK32,
				 CANVAS_BLK32_WR |
				 (MESON_CANVAS_BLKMODE_LINEAR << CANVAS_BLK32_BIT) |
				 CANVAS_INDEX_WR | id);
}

/* Release the output canvas + buffer if configured (reconfigure / exit). */
static void diag_output_free(struct vdec_diag_engine *eng)
{
	struct amvdec_core *core = eng->core;

	if (!eng->out_configured)
		return;
	meson_canvas_free(core->canvas, eng->out_canvas_id[0]);
	meson_canvas_free(core->canvas, eng->out_canvas_id[1]);
	if (eng->out_buf)
		dma_free_coherent(core->dev, eng->out_size, eng->out_buf,
				  eng->out_dma);
	eng->out_buf = NULL;
	eng->out_configured = false;
}

/* ---- output_canvas: configure an NV12 output canvas (0x55-sentinel) ----- */
static ssize_t vdec_diag_output_canvas_write(struct file *f,
					     const char __user *ubuf,
					     size_t count, loff_t *ppos)
{
	struct vdec_diag_engine *eng = engine_from_file(f);
	struct amvdec_core *core = eng->core;
	char buf[VDEC_DIAG_PEEK_BUF_LEN];
	u32 w, h, aw, ah;
	size_t sz;
	char *sep;
	int ret;

	if (count >= sizeof(buf))
		return -E2BIG;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';

	sep = strchr(buf, ' ');
	if (!sep)
		return -EINVAL;
	*sep = '\0';
	ret = kstrtou32(strim(buf), 0, &w);
	if (ret)
		return ret;
	ret = kstrtou32(strim(sep + 1), 0, &h);
	if (ret)
		return ret;
	if (w == 0 || h == 0 || w > 4096 || h > 4096)
		return -EINVAL;

	if (!engine_powered(eng)) {
		dev_warn(core->dev_dec,
			 "diag: output_canvas rejected -- engine not powered\n");
		return -ENODEV;
	}

	/* Canvas geometry matches amvdec_set_canvases: stride = ALIGN(w,64),
	 * canvas height = ALIGN(h,32); UV plane at Y_paddr + aw*ah. */
	aw = ALIGN(w, 64);
	ah = ALIGN(h, 32);
	sz = (size_t)aw * ah * 3 / 2;

	mutex_lock(&eng->lock);

	diag_output_free(eng);

	eng->out_buf = dma_alloc_coherent(core->dev, sz, &eng->out_dma,
					  GFP_KERNEL);
	if (!eng->out_buf) {
		mutex_unlock(&eng->lock);
		return -ENOMEM;
	}
	memset(eng->out_buf, 0x55, sz);		/* sentinel: MC-never-wrote */

	ret = meson_canvas_alloc(core->canvas, &eng->out_canvas_id[0]);
	if (ret)
		goto err_free;
	ret = meson_canvas_alloc(core->canvas, &eng->out_canvas_id[1]);
	if (ret) {
		meson_canvas_free(core->canvas, eng->out_canvas_id[0]);
		goto err_free;
	}

	/* Y plane: stride aw, height ah.  UV (shared): stride aw, height ah/2. */
	diag_canvas_config(core, eng->out_canvas_id[0], (u32)eng->out_dma,
			   aw, ah);
	diag_canvas_config(core, eng->out_canvas_id[1],
			   (u32)eng->out_dma + aw * ah, aw, ah / 2);

	/* NV12 packs the shared UV id in BOTH [23:16] and [15:8], Y in [7:0]. */
	eng->out_canvas_val = ((u32)eng->out_canvas_id[1] << 16) |
			      ((u32)eng->out_canvas_id[1] << 8) |
			      (u32)eng->out_canvas_id[0];
	eng->out_aligned_w = aw;
	eng->out_aligned_h = ah;
	eng->out_size = sz;
	eng->out_configured = true;

	dev_info(core->dev_dec,
		 "diag: output_canvas %ux%u (aligned %ux%u, %zu B) canvas=0x%06x y=0x%08x uv=0x%08x by pid=%d\n",
		 w, h, aw, ah, sz, eng->out_canvas_val, (u32)eng->out_dma,
		 (u32)eng->out_dma + aw * ah, current->pid);

	mutex_unlock(&eng->lock);
	return count;

err_free:
	dma_free_coherent(core->dev, sz, eng->out_buf, eng->out_dma);
	eng->out_buf = NULL;
	mutex_unlock(&eng->lock);
	return ret;
}

static int vdec_diag_output_canvas_show(struct seq_file *s, void *unused)
{
	struct vdec_diag_engine *eng = s->private;

	mutex_lock(&eng->lock);
	if (!eng->out_configured) {
		seq_puts(s, "configured=0\n");
		mutex_unlock(&eng->lock);
		return 0;
	}
	seq_printf(s, "configured=1\n");
	seq_printf(s, "canvas=0x%06x\n", eng->out_canvas_val);
	seq_printf(s, "canvas_id_y=%u\n", eng->out_canvas_id[0]);
	seq_printf(s, "canvas_id_uv=%u\n", eng->out_canvas_id[1]);
	seq_printf(s, "y_paddr=0x%08x\n", (u32)eng->out_dma);
	seq_printf(s, "uv_paddr=0x%08x\n",
		   (u32)eng->out_dma + eng->out_aligned_w * eng->out_aligned_h);
	seq_printf(s, "aligned_w=%u\n", eng->out_aligned_w);
	seq_printf(s, "aligned_h=%u\n", eng->out_aligned_h);
	seq_printf(s, "size=%zu\n", eng->out_size);
	mutex_unlock(&eng->lock);
	return 0;
}

static int vdec_diag_output_canvas_open(struct inode *i, struct file *f)
{
	return single_open(f, vdec_diag_output_canvas_show, i->i_private);
}

static const struct file_operations vdec_diag_output_canvas_fops = {
	.open = vdec_diag_output_canvas_open,
	.read = seq_read,
	.write = vdec_diag_output_canvas_write,
	.llseek = seq_lseek,
	.release = single_release,
};

/* ---- output_read: raw read-back of the decoded output buffer ------------ */
/* Pull the whole NV12 buffer for host-side sentinel / MD5 / PSNR checks.
 * A buffer still full of 0x55 means the MC never wrote (stall);  real pixels
 * (MD5 != all-0x55 and != all-zero) mean the decode retired. */
static ssize_t vdec_diag_output_read(struct file *f, char __user *ubuf,
				     size_t count, loff_t *ppos)
{
	struct vdec_diag_engine *eng = engine_from_file(f);
	ssize_t ret;

	mutex_lock(&eng->lock);
	if (!eng->out_configured) {
		mutex_unlock(&eng->lock);
		return -ENODEV;
	}
	ret = simple_read_from_buffer(ubuf, count, ppos, eng->out_buf,
				      eng->out_size);
	mutex_unlock(&eng->lock);
	return ret;
}

static const struct file_operations vdec_diag_output_read_fops = {
	.open = simple_open,
	.read = vdec_diag_output_read,
	.llseek = noop_llseek,
};

/* ---- decode: pack LMEM, DMA to 0x100, trigger, poll, snapshot regs ------ */
/*
 * Write format (all base-0 ints; hex ok):
 *   "<mb_w> <mb_h> <w_px> <h_px> <coding> <struct> <f_code> <pic_head> <vcop> <mpeg12_reg>"
 * e.g. a 320x240 MPEG-2 I-frame (whole picture):
 *   "20 15 320 240 1 3 0xffff 0x380 0x18 0x1000"
 *   mb_w=20 (320/16), mb_h=15 (240/16), coding=1 (I), struct=3 (frame),
 *   mpeg12_reg=0x1000 (MPEG-2 mode).  f_code / pic_head are stream-derived
 *   tunables (I-frame decode retires regardless; they refine PSNR).
 *
 * Preconditions: engine powered, firmware running its cmd_loop (a `firmware`
 * hot-upload happened), an output_canvas configured, and a bitstream pushed
 * via vififo_push (the firmware re-programs the VLD VIFIFO pointers from the
 * LMEM block, which this node fills with the diag ring base/end so they match
 * what vififo_push staged).
 */
#define DIAG_DECODE_NPARAM	10
#define DIAG_DECODE_POLL_MAX	60000		/* usleep(2) -> ~120 ms cap */
#define DIAG_DECODE_BUF_LEN	160

static ssize_t vdec_diag_decode_write(struct file *f, const char __user *ubuf,
				      size_t count, loff_t *ppos)
{
	struct vdec_diag_engine *eng = engine_from_file(f);
	struct amvdec_core *core = eng->core;
	struct device *dev = core->dev_dec;
	char buf[DIAG_DECODE_BUF_LEN];
	u32 p[DIAG_DECODE_NPARAM];
	u32 vf_start, vf_end, s9;
	u32 *ws;
	char *cur, *tok;
	int i, ret, waited;

	if (count >= sizeof(buf))
		return -E2BIG;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';
	cur = strim(buf);

	for (i = 0; i < DIAG_DECODE_NPARAM; i++) {
		tok = strsep(&cur, " \t");
		if (!tok || !*tok)
			return -EINVAL;
		ret = kstrtou32(tok, 0, &p[i]);
		if (ret)
			return ret;
	}

	if (!engine_powered(eng)) {
		dev_warn(dev, "diag: decode rejected -- engine not powered\n");
		return -ENODEV;
	}
	if (!eng->out_configured) {
		dev_warn(dev, "diag: decode rejected -- no output_canvas\n");
		return -ENODEV;
	}
	if (eng->vififo_len == 0) {
		dev_warn(dev, "diag: decode rejected -- no vififo_push bitstream\n");
		return -ENODEV;
	}

	mutex_lock(&eng->lock);

	/* VLD VIFIFO ring base/end (matches vififo_push START/END programming);
	 * the firmware writes these to VLD_MEM_VIFIFO_START/CURR/END from LMEM. */
	vf_start = (u32)eng->vififo_dma;
	vf_end = (u32)eng->vififo_dma + VDEC_DIAG_VIFIFO_SIZE - 8;

	/* Build the LMEM param block into the DMA workspace (0-based, relative
	 * to LMEM word 0x100).  Zero the whole DMA'd window first. */
	ws = eng->workspace_buf;
	memset(ws, 0, LMEM_PARAM_WORDS * 2);
#define PUT(abs, val) lmem_put16(ws, (abs) - LMEM_PARAM_START, (u16)(val))
	PUT(0x100, 1);				/* codec_id = MPEG-1/2 */
	PUT(0x101, 0);				/* flags */
	PUT(0x102, p[0]);			/* mb_width */
	PUT(0x103, p[1]);			/* mb_height */
	PUT(0x104, eng->out_canvas_val & 0xffff);	/* out canvas lo */
	PUT(0x105, (eng->out_canvas_val >> 16) & 0xff);	/* out canvas hi */
	PUT(0x106, 0);				/* fwd_ref lo (intra -> §5a subst) */
	PUT(0x107, 0);				/* fwd_ref hi */
	PUT(0x108, 0);				/* bwd_ref lo */
	PUT(0x109, 0);				/* bwd_ref hi */
	PUT(0x10B, p[2]);			/* width_pixels */
	PUT(0x10C, p[3]);			/* height_pixels */
	PUT(0x10D, vf_start & 0xffff);		/* vififo_start lo */
	PUT(0x10E, (vf_start >> 16) & 0xffff);	/* vififo_start hi */
	PUT(0x10F, vf_end & 0xffff);		/* vififo_end lo */
	PUT(0x110, (vf_end >> 16) & 0xffff);	/* vififo_end hi */
	for (i = 0; i < 64; i++)
		PUT(0x118 + i, mpeg2_default_intra_quant[i]);	/* intra quant */
	for (i = 0; i < 64; i++)
		PUT(0x158 + i, 16);		/* non-intra quant (flat 16) */
	PUT(0x198, p[4]);			/* pic_coding_type */
	PUT(0x199, p[5]);			/* pic_structure */
	PUT(0x19A, p[6]);			/* f_code */
	PUT(0x19B, p[7]);			/* pic_head_info */
	PUT(0x19C, p[8]);			/* vcop_ctrl */
	PUT(0x19D, p[9]);			/* mpeg1_2_reg */
#undef PUT

	dev_info(dev,
		 "diag: decode mbw=%u mbh=%u %ux%u coding=%u struct=%u fcode=0x%x pichead=0x%x vcop=0x%x mp12=0x%x canvas=0x%06x vf=[0x%08x,0x%08x] by pid=%d\n",
		 p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9],
		 eng->out_canvas_val, vf_start, vf_end, current->pid);

	/* DMA the param block DDR -> LMEM word 0x100 (CTRL bit14 mem->LMEM,
	 * bits11:0 = start word, COUNT in 16-bit words). */
	amvdec_write_dos(core, LMEM_DMA_ADR, (u32)eng->workspace_dma);
	amvdec_write_dos(core, LMEM_DMA_COUNT, LMEM_PARAM_WORDS);
	amvdec_write_dos(core, LMEM_DMA_CTRL,
			 LMEM_DMA_CTRL_START | LMEM_DMA_CTRL_MEM2LMEM |
			 LMEM_PARAM_START);
	for (waited = 0; waited < VDEC_DIAG_LMEM_POLL_MAX; waited++) {
		if (!(amvdec_read_dos(core, LMEM_DMA_CTRL) & LMEM_DMA_CTRL_START))
			break;
		usleep_range(1, 2);
	}
	if (waited >= VDEC_DIAG_LMEM_POLL_MAX)
		dev_warn(dev, "diag: decode LMEM param DMA did not drain (bit15 stuck)\n");

	/* Trigger: AV_SCRATCH_9 = 1 (LIBRE_CMD_DECODE).  Firmware's cmd_loop
	 * spins on ==1; on completion it writes AV_SCRATCH_9=2, AV_SCRATCH_F=err,
	 * fires MBOX1 (not routed here -- we poll the scratch). */
	amvdec_write_dos(core, AV_SCRATCH_F, 0);
	amvdec_write_dos(core, AV_SCRATCH_9, 1);

	for (waited = 0; waited < DIAG_DECODE_POLL_MAX; waited++) {
		s9 = amvdec_read_dos(core, AV_SCRATCH_9);
		if (s9 == 2)
			break;
		usleep_range(2, 4);
	}

	/* Snapshot the observation registers immediately (a wedge freezes them;
	 * a retire clears WRRSP bit31 and advances MBY_MBX / DCAC). */
	eng->decode_timed_out = (waited >= DIAG_DECODE_POLL_MAX);
	eng->dec_iters = waited;
	eng->dec_scratch9 = amvdec_read_dos(core, AV_SCRATCH_9);
	eng->dec_scratchf = amvdec_read_dos(core, AV_SCRATCH_F);
	eng->dec_wrrsp = amvdec_read_dos(core, MCW_DBLK_WRRSP_CNT);
	eng->dec_mby_mbx = amvdec_read_dos(core, MBY_MBX);
	eng->dec_mc_status0 = amvdec_read_dos(core, MC_STATUS0);
	eng->dec_dcac = amvdec_read_dos(core, DCAC_MB_COUNT);
	eng->dec_vififo_level = amvdec_read_dos(core, VLD_MEM_VIFIFO_LEVEL);
	eng->decode_valid = true;

	dev_info(dev,
		 "diag: decode %s iters=%u s9=0x%x sf=0x%x wrrsp=0x%08x mby_mbx=0x%08x mc_status0=0x%08x dcac=0x%08x\n",
		 eng->decode_timed_out ? "TIMEOUT" : "DONE", eng->dec_iters,
		 eng->dec_scratch9, eng->dec_scratchf, eng->dec_wrrsp,
		 eng->dec_mby_mbx, eng->dec_mc_status0, eng->dec_dcac);

	mutex_unlock(&eng->lock);
	return count;
}

static const struct file_operations vdec_diag_decode_fops = {
	.open = simple_open,
	.write = vdec_diag_decode_write,
	.llseek = noop_llseek,
};

/* ---- decode_result: read back the last decode's register snapshot ------- */
static int vdec_diag_decode_result_show(struct seq_file *s, void *unused)
{
	struct vdec_diag_engine *eng = s->private;

	mutex_lock(&eng->lock);
	if (!eng->decode_valid) {
		seq_puts(s, "result=noinit\n");
		mutex_unlock(&eng->lock);
		return 0;
	}
	seq_printf(s, "result=%s\n", eng->decode_timed_out ? "timeout" : "done");
	seq_printf(s, "iters=%u\n", eng->dec_iters);
	seq_printf(s, "scratch9=0x%08x\n", eng->dec_scratch9);
	seq_printf(s, "scratchf=0x%08x\n", eng->dec_scratchf);
	seq_printf(s, "wrrsp=0x%08x\n", eng->dec_wrrsp);
	seq_printf(s, "mby_mbx=0x%08x\n", eng->dec_mby_mbx);
	seq_printf(s, "mc_status0=0x%08x\n", eng->dec_mc_status0);
	seq_printf(s, "dcac_mb_count=0x%08x\n", eng->dec_dcac);
	seq_printf(s, "vififo_level=0x%08x\n", eng->dec_vififo_level);
	mutex_unlock(&eng->lock);
	return 0;
}

static int vdec_diag_decode_result_open(struct inode *i, struct file *f)
{
	return single_open(f, vdec_diag_decode_result_show, i->i_private);
}

static const struct file_operations vdec_diag_decode_result_fops = {
	.open = vdec_diag_decode_result_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/* ---- init / exit -------------------------------------------------------- */

int vdec_diag_init(struct amvdec_core *core)
{
	struct vdec_diag *diag;
	struct vdec_diag_engine *eng;

	if (!vdec_diag_enabled)
		return 0;

	diag = devm_kzalloc(core->dev, sizeof(*diag), GFP_KERNEL);
	if (!diag)
		return -ENOMEM;

	/* Reuse vdec_debugfs's meson-vdec root; don't create a second
	 * copy (kernel would silently succeed-but-orphan the second dir).
	 * vdec_debugfs_init runs earlier in vdec_probe so core->debugfs.root
	 * is already set unless debugfs failed entirely. */
	if (!core->debugfs.root) {
		dev_warn(core->dev, "diag: vdec_debugfs root not available\n");
		return -ENODEV;
	}
	diag->root = core->debugfs.root;

	diag->diag_dir = debugfs_create_dir("diag", diag->root);
	if (IS_ERR(diag->diag_dir)) {
		debugfs_remove_recursive(diag->diag_dir);
		return PTR_ERR(diag->diag_dir);
	}

	eng = &diag->vdec1;
	eng->core = core;
	eng->name = "vdec1";
	mutex_init(&eng->lock);

	eng->fw_buf_size = VDEC_DIAG_MC_SIZE;
	eng->fw_buf = dma_alloc_coherent(core->dev, eng->fw_buf_size,
					 &eng->fw_buf_dma, GFP_KERNEL);
	if (!eng->fw_buf) {
		debugfs_remove_recursive(diag->diag_dir);
		return -ENOMEM;
	}

	/*
	 * LMEM_DMA characterization workspace.  4 KiB DMA-coherent buffer
	 * reused across every lmem_dma_test fire; freed in vdec_diag_exit.
	 * Allocation failure here unwinds fw_buf and the diag dir; the
	 * driver continues without diag.
	 */
	eng->workspace_buf = dma_alloc_coherent(core->dev,
						VDEC_DIAG_WORKSPACE_SIZE,
						&eng->workspace_dma,
						GFP_KERNEL);
	if (!eng->workspace_buf) {
		dma_free_coherent(core->dev, eng->fw_buf_size,
				  eng->fw_buf, eng->fw_buf_dma);
		eng->fw_buf = NULL;
		debugfs_remove_recursive(diag->diag_dir);
		return -ENOMEM;
	}

	/*
	 * VIFIFO push ring (vififo_push node).  1 MiB DMA-coherent buffer,
	 * reused across pushes; freed in vdec_diag_exit.  Unwinds workspace +
	 * fw_buf on failure.
	 */
	eng->vififo_buf = dma_alloc_coherent(core->dev, VDEC_DIAG_VIFIFO_SIZE,
					     &eng->vififo_dma, GFP_KERNEL);
	if (!eng->vififo_buf) {
		dma_free_coherent(core->dev, VDEC_DIAG_WORKSPACE_SIZE,
				  eng->workspace_buf, eng->workspace_dma);
		eng->workspace_buf = NULL;
		dma_free_coherent(core->dev, eng->fw_buf_size,
				  eng->fw_buf, eng->fw_buf_dma);
		eng->fw_buf = NULL;
		debugfs_remove_recursive(diag->diag_dir);
		return -ENOMEM;
	}

	eng->dir = debugfs_create_dir("vdec1", diag->diag_dir);

	debugfs_create_file("firmware", 0200, eng->dir, eng, &vdec_diag_firmware_fops);
	debugfs_create_file("reset", 0200, eng->dir, eng, &vdec_diag_reset_fops);
	debugfs_create_file("state", 0400, eng->dir, eng, &vdec_diag_state_fops);
	debugfs_create_file("peek", 0600, eng->dir, eng, &vdec_diag_peek_fops);
	debugfs_create_file("poke", 0200, eng->dir, eng, &vdec_diag_poke_fops);
	debugfs_create_file("mode", 0200, eng->dir, eng, &vdec_diag_mode_fops);
	debugfs_create_file("cmd", 0200, eng->dir, eng, &vdec_diag_cmd_fops);
	/* poweron: bring engine up without a V4L2 session (added with the
	 * poweron_write function in commit 3aba09efd706; the debugfs_create
	 * line was accidentally omitted from that commit). */
	debugfs_create_file("poweron", 0200, eng->dir, eng, &vdec_diag_poweron_fops);
	/*
	 * LMEM_DMA characterization quartet (M4.1.7 ROOT CAUSE workstream):
	 *   lmem_dma_test       -- fire one LMEM_DMA pulse with parsed args
	 *   lmem_dma_result     -- read the most recent fire's outcome
	 *   workspace_paddr     -- physical address of the DMA workspace
	 *   workspace_pattern   -- prefill the workspace with seed + i*4
	 * Used to empirically characterize LMEM_DMA_CTRL bit fields against
	 * the BSP firmware's observed encodings (see
	 * the internal stateless ANALYSIS record
	 * "2026-04-24 -- M4.1.7 ROOT CAUSE").
	 */
	debugfs_create_file("lmem_dma_test", 0200, eng->dir, eng,
			    &vdec_diag_lmem_dma_test_fops);
	debugfs_create_file("lmem_dma_result", 0400, eng->dir, eng,
			    &vdec_diag_lmem_dma_result_fops);
	debugfs_create_file("workspace_paddr", 0400, eng->dir, eng,
			    &vdec_diag_workspace_paddr_fops);
	debugfs_create_file("workspace_pattern", 0600, eng->dir, eng,
			    &vdec_diag_workspace_pattern_fops);
	/* workspace_write/read: raw byte access to the LMEM-DMA staging buffer
	 * (arbitrary LMEM preload for libre-fw-probe upload-lmem). */
	debugfs_create_file("workspace_write", 0200, eng->dir, eng,
			    &vdec_diag_workspace_write_fops);
	debugfs_create_file("workspace_read", 0400, eng->dir, eng,
			    &vdec_diag_workspace_read_fops);
	/* vififo_push: feed a raw bitstream into the VLD VIFIFO (stateless
	 * co-validation board half; libre-fw-probe push-vififo). */
	debugfs_create_file("vififo_push", 0200, eng->dir, eng,
			    &vdec_diag_vififo_push_fops);
	/*
	 * MPEG-2 decode-fixture harness (mpeg12-mc-output-frontier.md §7.2):
	 *   output_canvas -- configure a 0x55-sentinel NV12 output canvas
	 *   output_read   -- pull the decoded buffer (sentinel/MD5/PSNR)
	 *   decode        -- pack LMEM + trigger + snapshot MC observ regs
	 *   decode_result -- read the last decode's register snapshot
	 * Drives a full I-frame decode through the running libre firmware to
	 * A/B the MCRCC null-canvas fix (§5a) without a V4L2 session.
	 */
	debugfs_create_file("output_canvas", 0600, eng->dir, eng,
			    &vdec_diag_output_canvas_fops);
	debugfs_create_file("output_read", 0400, eng->dir, eng,
			    &vdec_diag_output_read_fops);
	debugfs_create_file("decode", 0200, eng->dir, eng,
			    &vdec_diag_decode_fops);
	debugfs_create_file("decode_result", 0400, eng->dir, eng,
			    &vdec_diag_decode_result_fops);

	core->diag = diag;
	dev_info(core->dev, "diag: debugfs interface ready at /sys/kernel/debug/meson-vdec/diag/vdec1/\n");
	return 0;
}

void vdec_diag_exit(struct amvdec_core *core)
{
	struct vdec_diag *diag = core->diag;
	struct vdec_diag_engine *eng;

	if (!diag)
		return;

	eng = &diag->vdec1;

	/* Release the decode-harness output canvas + buffer. */
	diag_output_free(eng);

	/* If diag manual-powered the engine, power it off before teardown
	 * so clocks/rails are released cleanly. */
	if (eng->manual_powered) {
		vdec_diag_poweroff(core);
		eng->manual_powered = false;
	}

	if (eng->fw_buf)
		dma_free_coherent(core->dev, eng->fw_buf_size,
				  eng->fw_buf, eng->fw_buf_dma);
	if (eng->workspace_buf)
		dma_free_coherent(core->dev, VDEC_DIAG_WORKSPACE_SIZE,
				  eng->workspace_buf, eng->workspace_dma);
	if (eng->vififo_buf)
		dma_free_coherent(core->dev, VDEC_DIAG_VIFIFO_SIZE,
				  eng->vififo_buf, eng->vififo_dma);
	/* diag->root is shared with vdec_debugfs; only remove the diag_dir
	 * subtree we own, not the shared root. vdec_debugfs_remove()
	 * handles the rest. */
	debugfs_remove_recursive(diag->diag_dir);
	core->diag = NULL;
}
