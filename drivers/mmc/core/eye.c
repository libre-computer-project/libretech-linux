// SPDX-License-Identifier: GPL-2.0-only
/*
 * Platform-agnostic MMC eye-scan (signal-integrity) facility.
 *
 * Measures the RX (read/sampling) and TX (write/drive) timing "eyes" of an MMC
 * link on ANY controller that supplies a small set of timing-point primitives
 * (struct mmc_eye_ops), for SD, SDIO and eMMC. The core owns the sweep loop and
 * the pass/fail oracle; the host driver only knows how to move its own RX/TX
 * timing point. See software/mmc-eye-scan.md for the design.
 *
 * The two eyes use different oracles, because the hardware is asymmetric:
 *
 *   read eye  -- the card drives, the host samples. Oracle = CMD19 tuning
 *                (mmc_send_tuning): the card sends a known block and the core
 *                checks it. Non-destructive. Works for SD/SDIO/eMMC.
 *
 *   write eye -- the host drives, the card samples internally. There is no
 *                write-tuning command, so the oracle is: write a known burst of
 *                write_eye_blocks blocks (default a CMD25 multi-block transfer)
 *                to a caller-designated scratch LBA, read it back (at the tuned
 *                RX point) and compare. DESTRUCTIVE -- it clobbers write_eye_blocks
 *                blocks starting at scratch_lba -- so it is gated behind an
 *                explicit scratch_lba plus a one-shot arm. Works for SD/eMMC;
 *                SDIO (no block read-back) is left to a future function-loopback
 *                oracle.
 *
 * Exposed via debugfs at /sys/kernel/debug/mmcN/eye/ when the host sets
 * ->eye_ops. All sweeps run with the host claimed and re-tuning held (done by
 * the debugfs entry points below), so they never race live I/O; the backend
 * snapshots/restores every register it touches, leaving the tuned state intact.
 */

#include <linux/debugfs.h>
#include <linux/random.h>
#include <linux/scatterlist.h>
#include <linux/seq_file.h>
#include <linux/slab.h>

#include <linux/mmc/core.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>

#include "core.h"
#include "card.h"
#include "host.h"
#include <linux/module.h>

#if IS_ENABLED(CONFIG_MMC_BLOCK)
/* From block.c — not a static link when MMC_BLOCK=m (see eye_card_*). */
bool mmc_blk_card_in_use(struct mmc_card *card);
sector_t mmc_blk_card_user_sectors(struct mmc_card *card);
#endif

/*
 * Optional reverse dependency on mmc_block. Both MMC and MMC_BLOCK are often
 * =m on x86 host kernels; a static call from mmc_core.ko to mmc_block.ko is a
 * circular modpost error (mmc_block already depends on mmc_core). Use a
 * direct call when block is built-in, else symbol_get at runtime so the write
 * eye still refuses an open gendisk when mmc_block is loaded.
 */
static bool eye_card_in_use(struct mmc_card *card)
{
#if IS_BUILTIN(CONFIG_MMC_BLOCK)
	return mmc_blk_card_in_use(card);
#elif IS_MODULE(CONFIG_MMC_BLOCK)
	bool (*fn)(struct mmc_card *) = symbol_get(mmc_blk_card_in_use);
	bool ret = false;

	if (fn) {
		ret = fn(card);
		symbol_put(mmc_blk_card_in_use);
	}
	return ret;
#else
	return false;
#endif
}

static sector_t eye_card_user_sectors(struct mmc_card *card)
{
#if IS_BUILTIN(CONFIG_MMC_BLOCK)
	return mmc_blk_card_user_sectors(card);
#elif IS_MODULE(CONFIG_MMC_BLOCK)
	sector_t (*fn)(struct mmc_card *) = symbol_get(mmc_blk_card_user_sectors);
	sector_t ret = 0;

	if (fn) {
		ret = fn(card);
		symbol_put(mmc_blk_card_user_sectors);
	}
	return ret;
#else
	return 0;
#endif
}


/* Cap the sweep so a bogus ->num_points can't run away or over-allocate. */
#define MMC_EYE_MAX_POINTS	256
#define MMC_EYE_BLK		512
/* Cap the multi-block write-eye burst (blocks) so a bogus count can't over-allocate. */
/*
 * Cap the multi-block write-eye burst. The buffer is a single physically-
 * contiguous DMA allocation (kmalloc, sg_init_one), so keep the cap at an order
 * the page allocator serves reliably: 256 blocks = 128 KiB (order-5) vs a fragile
 * order-7 at 512 KiB. Still a large CMD25 burst -- ample to expose sustained-write
 * behaviour.
 */
#define MMC_EYE_MAX_BLOCKS	256	/* 128 KiB */
/*
 * Default write-eye burst = 8 blocks (a CMD25 multi-block transfer), NOT 1.
 * A single-block CMD24 write can SUCCEED on a link whose CMD25 multi-block
 * writes COLLAPSE at every TX delay -- so a single-block oracle over-reports a
 * clean write eye that no real (block-layer, always multi-block) I/O can use.
 * Measured on an S905D3 board's eMMC at 8-bit HS400: single-block eye =
 * a 32-tap window, but >=2-block (CMD25) eye = fully collapsed, and every
 * block-layer write failed with a card write-CRC error. Default to multi-block
 * so the write eye is honest; set write_eye_blocks=1 for the old single-block
 * probe.
 */
#define MMC_EYE_WR_DEFAULT_BLOCKS	8
/* CMD19 reads per read-eye point -- a marginal point should fail at least one. */
#define MMC_EYE_READ_TRIES	3

static u32 mmc_eye_tuning_opcode(struct mmc_host *host)
{
	/* eMMC HS200/HS400 tune with CMD21; SD/SDIO (and slower eMMC) use CMD19 */
	if (host->ios.timing == MMC_TIMING_MMC_HS200 ||
	    host->ios.timing == MMC_TIMING_MMC_HS400)
		return MMC_SEND_TUNING_BLOCK_HS200;
	return MMC_SEND_TUNING_BLOCK;
}

/* Read-eye oracle: the sampling point passes if every tuning read succeeds. */
static bool mmc_eye_read_ok(struct mmc_host *host)
{
	u32 opcode = mmc_eye_tuning_opcode(host);
	int i;

	for (i = 0; i < MMC_EYE_READ_TRIES; i++)
		if (mmc_send_tuning(host, opcode, NULL))
			return false;
	return true;
}

/*
 * Raw single- or multi-block write/read to the scratch LBA (@nblk blocks). A
 * single block uses CMD24/CMD17; @nblk > 1 uses the open-ended multi-block
 * CMD25/CMD18 + a CMD12 STOP -- the SAME command path the block layer takes for
 * bulk I/O, so the multi-block write eye reflects real sustained-write behaviour
 * (a HS400 write path can pass single-block CMD24 yet fail the CMD25 burst).
 */
static int mmc_eye_xfer(struct mmc_host *host, struct mmc_card *card,
			u64 lba, void *buf, unsigned int nblk, bool write)
{
	struct mmc_request mrq = {};
	struct mmc_command cmd = {};
	struct mmc_command stop = {};
	struct mmc_data data = {};
	struct scatterlist sg;
	bool multi = nblk > 1;

	if (multi)
		cmd.opcode = write ? MMC_WRITE_MULTIPLE_BLOCK : MMC_READ_MULTIPLE_BLOCK;
	else
		cmd.opcode = write ? MMC_WRITE_BLOCK : MMC_READ_SINGLE_BLOCK;
	/* high-capacity cards address by block, standard-capacity by byte */
	cmd.arg = mmc_card_blockaddr(card) ? lba : lba * MMC_EYE_BLK;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;

	data.blksz = MMC_EYE_BLK;
	data.blocks = nblk;
	data.flags = write ? MMC_DATA_WRITE : MMC_DATA_READ;
	data.sg = &sg;
	data.sg_len = 1;
	sg_init_one(&sg, buf, (size_t)nblk * MMC_EYE_BLK);

	mrq.cmd = &cmd;
	mrq.data = &data;
	if (multi) {
		stop.opcode = MMC_STOP_TRANSMISSION;
		stop.arg = 0;
		stop.flags = MMC_RSP_R1B | MMC_CMD_AC;
		mrq.stop = &stop;
	}

	mmc_set_data_timeout(&data, card);
	mmc_wait_for_req(host, &mrq);

	if (cmd.error)
		return cmd.error;
	if (data.error)
		return data.error;
	if (multi && stop.error)
		return stop.error;
	return 0;
}

/*
 * Write-eye oracle: drive a known block at the TX point under test, then read
 * it back and compare. The read-back is done at the driver's tuned timing (the
 * backend's ->restore() re-applies it) so a mismatch reflects the WRITE at this
 * point, not a mistuned read command. A silent write corruption -- the card
 * committing wrong data with no CRC error -- shows up here as a compare failure.
 */
static bool mmc_eye_write_ok(struct mmc_host *host, const struct mmc_eye_ops *ops,
			     void *wbuf, void *rbuf, unsigned int nblk)
{
	size_t len = (size_t)nblk * MMC_EYE_BLK;
	int werr, rerr;

	memset(rbuf, 0, len);
	werr = mmc_eye_xfer(host, host->card, host->eye_scratch_lba,
			    wbuf, nblk, true);
	ops->restore(host, MMC_EYE_WRITE);	/* tuned TX for a clean read-back */
	rerr = mmc_eye_xfer(host, host->card, host->eye_scratch_lba,
			    rbuf, nblk, false);

	return !werr && !rerr && !memcmp(wbuf, rbuf, len);
}

static int mmc_eye_bus_lanes(struct mmc_host *host)
{
	if (host->ios.bus_width == MMC_BUS_WIDTH_8)
		return 8;
	if (host->ios.bus_width == MMC_BUS_WIDTH_4)
		return 4;
	return 1;
}

/*
 * Run one eye sweep and emit the pass/fail map to @s. Caller holds the host
 * claimed and re-tune held. Returns with the tuned timing restored.
 */
static void mmc_eye_run(struct mmc_host *host, enum mmc_eye_mode mode,
			struct seq_file *s)
{
	const struct mmc_eye_ops *ops = host->eye_ops;
	int n, npts, cur_start = -1, cur_len = 0, best_start = -1, best_len = 0;
	void *wbuf = NULL, *rbuf = NULL;
	unsigned int wblk = 1;
	char *map, label[48];

	if (!host->card) {
		seq_puts(s, "no card\n");
		return;
	}

	npts = ops->num_points ? ops->num_points(host, mode) : 0;
	if (npts <= 0) {
		seq_printf(s, "%s eye unsupported on this controller\n",
			   mode == MMC_EYE_WRITE ? "write" : "read");
		return;
	}
	if (npts > MMC_EYE_MAX_POINTS)
		npts = MMC_EYE_MAX_POINTS;

	map = kmalloc(npts + 1, GFP_KERNEL);
	if (!map)
		return;

	if (mode == MMC_EYE_WRITE) {
		sector_t cap = host->card ? eye_card_user_sectors(host->card) : 0;
		size_t len;

		wblk = host->eye_write_blocks ?: MMC_EYE_WR_DEFAULT_BLOCKS;
		if (wblk > MMC_EYE_MAX_BLOCKS)
			wblk = MMC_EYE_MAX_BLOCKS;

		/*
		 * Bounds-check the scratch region against the real device size
		 * (in 512-byte sectors == MMC_EYE_BLK blocks) so it can't run off
		 * the end of the card and just return EIO. cap==0 = no user-area
		 * gendisk to size against; skip the check (nothing to bound).
		 */
		if (cap && host->eye_scratch_lba + wblk > cap) {
			seq_printf(s, "refused: scratch_lba %llu + %u block(s) exceeds card capacity (%llu blocks)\n",
				   host->eye_scratch_lba, wblk,
				   (unsigned long long)cap);
			goto out;
		}

		len = (size_t)wblk * MMC_EYE_BLK;
		/*
		 * kmalloc, NOT kvmalloc: the buffer is DMA'd (sg_init_one ->
		 * virt_to_page), so it must be physically contiguous / lowmem;
		 * kvmalloc can fall back to vmalloc, which sg_init_one mishandles.
		 * MMC_EYE_MAX_BLOCKS caps @len at a reliable low-order allocation so
		 * a large write_eye_blocks can't demand a fragile high-order alloc.
		 */
		wbuf = kmalloc(len, GFP_KERNEL);
		rbuf = kmalloc(len, GFP_KERNEL);
		if (!wbuf || !rbuf) {
			seq_puts(s, "out of memory\n");
			goto out;
		}
		/* a transition-rich pattern stresses the write eye hardest */
		get_random_bytes(wbuf, len);
	}

	ops->begin(host, mode);
	for (n = 0; n < npts; n++) {
		bool ok;

		ops->set_point(host, mode, n);
		if (mode == MMC_EYE_READ)
			ok = mmc_eye_read_ok(host);
		else
			ok = mmc_eye_write_ok(host, ops, wbuf, rbuf, wblk);

		map[n] = ok ? '#' : '.';
		if (ok) {
			if (cur_start < 0)
				cur_start = n;
			cur_len++;
			if (cur_len > best_len) {
				best_len = cur_len;
				best_start = cur_start;
			}
		} else {
			cur_start = -1;
			cur_len = 0;
		}
	}
	map[npts] = '\0';
	ops->restore(host, mode);

	seq_printf(s, "%s eye: timing=%d lanes=%d sd_clk=%u  (%d points)\n",
		   mode == MMC_EYE_WRITE ? "write" : "read",
		   host->ios.timing, mmc_eye_bus_lanes(host),
		   host->actual_clock, npts);
	if (mode == MMC_EYE_WRITE)
		seq_printf(s, "write burst: %u block(s) (%zu bytes) per TX point\n",
			   wblk, (size_t)wblk * MMC_EYE_BLK);
	seq_printf(s, "map: %s\n", map);
	if (best_len > 0) {
		int center = best_start + best_len / 2;

		label[0] = '\0';
		if (ops->describe)
			ops->describe(host, mode, center, label, sizeof(label));
		seq_printf(s, "widest window: start=%d width=%d center=%d%s%s\n",
			   best_start, best_len, center,
			   label[0] ? "  " : "", label);
	} else {
		seq_puts(s, "no passing window\n");
	}
out:
	kfree(rbuf);
	kfree(wbuf);
	kfree(map);
}

static int mmc_read_eye_show(struct seq_file *s, void *data)
{
	struct mmc_host *host = s->private;

	mmc_claim_host(host);
	mmc_retune_hold(host);
	mmc_eye_run(host, MMC_EYE_READ, s);
	mmc_retune_release(host);
	mmc_release_host(host);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mmc_read_eye);

static int mmc_write_eye_show(struct seq_file *s, void *data)
{
	struct mmc_host *host = s->private;

	/*
	 * The write eye OVERWRITES the scratch block repeatedly. Refuse unless
	 * the operator has designated a scratch LBA and armed the one-shot gate
	 * (auto-disarmed here), so a stray read of this file cannot destroy data.
	 */
	if (!host->eye_scratch_valid) {
		seq_puts(s, "refused: write 'scratch_lba' first -- the write eye is DESTRUCTIVE and clobbers write_eye_blocks blocks from there\n");
		return 0;
	}
	if (!host->eye_write_armed) {
		seq_puts(s, "refused: 'echo 1 > write_eye_arm' first (one-shot); the write eye repeatedly overwrites scratch_lba\n");
		return 0;
	}
	/*
	 * Refuse to clobber a live device: if the card's user-area block device
	 * (or any partition) is open -- mounted, or held by a raw opener -- the
	 * write eye would destroy that data. Checked BEFORE consuming the arm so
	 * the operator can unmount and retry without re-arming.
	 */
	if (host->card && eye_card_in_use(host->card)) {
		seq_puts(s, "refused: the eMMC/SD block device is in use (mounted or open) -- unmount it first; the write eye is DESTRUCTIVE\n");
		return 0;
	}
	host->eye_write_armed = false;

	mmc_claim_host(host);
	mmc_retune_hold(host);
	mmc_eye_run(host, MMC_EYE_WRITE, s);
	mmc_retune_release(host);
	mmc_release_host(host);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mmc_write_eye);

static int mmc_eye_scratch_get(void *data, u64 *val)
{
	struct mmc_host *host = data;

	*val = host->eye_scratch_lba;
	return 0;
}

static int mmc_eye_scratch_set(void *data, u64 val)
{
	struct mmc_host *host = data;

	host->eye_scratch_lba = val;
	host->eye_scratch_valid = true;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(mmc_eye_scratch_fops, mmc_eye_scratch_get,
			 mmc_eye_scratch_set, "%llu\n");

static int mmc_eye_arm_get(void *data, u64 *val)
{
	struct mmc_host *host = data;

	*val = host->eye_write_armed;
	return 0;
}

static int mmc_eye_arm_set(void *data, u64 val)
{
	struct mmc_host *host = data;

	host->eye_write_armed = !!val;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(mmc_eye_arm_fops, mmc_eye_arm_get, mmc_eye_arm_set,
			 "%llu\n");

/* write_eye_blocks: blocks per write-eye TX point. 1 = single-block CMD24;
 * >1 (default MMC_EYE_WR_DEFAULT_BLOCKS) = a sustained CMD25 multi-block burst
 * (the real bulk-write path -- the honest write eye; see the default note above). */
static int mmc_eye_wblocks_get(void *data, u64 *val)
{
	*val = ((struct mmc_host *)data)->eye_write_blocks ?: MMC_EYE_WR_DEFAULT_BLOCKS;
	return 0;
}
static int mmc_eye_wblocks_set(void *data, u64 val)
{
	struct mmc_host *host = data;

	if (val < 1)
		val = 1;
	if (val > MMC_EYE_MAX_BLOCKS)
		val = MMC_EYE_MAX_BLOCKS;
	host->eye_write_blocks = val;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(mmc_eye_wblocks_fops, mmc_eye_wblocks_get,
			 mmc_eye_wblocks_set, "%llu\n");

/*
 * Create the per-host "eye/" debugfs directory. Present only when the host
 * supplies ->eye_ops. read_eye is non-destructive; write_eye is gated by
 * scratch_lba + write_eye_arm.
 */
void mmc_add_eye_debugfs(struct mmc_host *host, struct dentry *root)
{
	struct dentry *dir;

	if (!host->eye_ops)
		return;

	dir = debugfs_create_dir("eye", root);

	debugfs_create_file("read_eye", 0400, dir, host, &mmc_read_eye_fops);
	debugfs_create_file("write_eye", 0400, dir, host, &mmc_write_eye_fops);
	debugfs_create_file_unsafe("scratch_lba", 0600, dir, host,
				   &mmc_eye_scratch_fops);
	debugfs_create_file_unsafe("write_eye_arm", 0600, dir, host,
				   &mmc_eye_arm_fops);
	debugfs_create_file_unsafe("write_eye_blocks", 0600, dir, host,
				   &mmc_eye_wblocks_fops);
}
