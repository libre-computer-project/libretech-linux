// SPDX-License-Identifier: GPL-2.0-only
/*
 * Crypto acceleration support for Rockchip RK3288
 *
 * Copyright (c) 2015, Fuzhou Rockchip Electronics Co., Ltd
 *
 * Author: Zain Wang <zain.wang@rock-chips.com>
 *
 * Some ideas are from marvell-cesa.c and s5p-sss.c driver.
 */

#include "rk3288_crypto.h"
#include <crypto/engine.h>
#include <crypto/internal/hash.h>
#include <crypto/internal/skcipher.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/reset.h>
#include <linux/spinlock.h>

static struct rockchip_ip rocklist = {
	.dev_list = LIST_HEAD_INIT(rocklist.dev_list),
	.lock = __SPIN_LOCK_UNLOCKED(rocklist.lock),
};

struct rk_crypto_info *get_rk_crypto(void)
{
	struct rk_crypto_info *first;

	spin_lock(&rocklist.lock);
	first = list_first_entry_or_null(&rocklist.dev_list,
					 struct rk_crypto_info, list);
	list_rotate_left(&rocklist.dev_list);
	spin_unlock(&rocklist.lock);
	return first;
}

/*
 * A device to hang a diagnostic message off, for code that runs in an alg
 * callback and has no rk_crypto_info of its own -- the tfm_init() paths, which
 * only ever need it to print. Reads the live rocklist rather than a pointer
 * cached in rk_cipher_algs[], because those algs outlive any single instance
 * (see the comment on struct rk_crypto_tmp::registered).
 *
 * Does not rotate, so a diagnostic cannot perturb get_rk_crypto()'s
 * round-robin. Returns NULL if no instance is left, which dev_err() renders as
 * "(NULL device *)". The pointer is only guaranteed live for as long as the
 * caller's alg registration is, which is the same window get_rk_crypto()'s
 * result already has; this does not narrow that race and does not claim to.
 */
struct device *rk_crypto_diag_dev(void)
{
	struct rk_crypto_info *first;

	spin_lock(&rocklist.lock);
	first = list_first_entry_or_null(&rocklist.dev_list,
					 struct rk_crypto_info, list);
	spin_unlock(&rocklist.lock);
	return first ? first->dev : NULL;
}

static const struct rk_variant rk3288_variant = {
	.num_clks = 4,
	.rkclks = {
		{ "sclk", 150000000},
	}
};

static const struct rk_variant rk3328_variant = {
	.num_clks = 3,
};

static const struct rk_variant rk3399_variant = {
	.num_clks = 3,
};

static int rk_crypto_get_clks(struct rk_crypto_info *dev)
{
	int i, j, err;
	unsigned long cr;

	dev->num_clks = devm_clk_bulk_get_all(dev->dev, &dev->clks);
	if (dev->num_clks < dev->variant->num_clks) {
		dev_err(dev->dev, "Missing clocks, got %d instead of %d\n",
			dev->num_clks, dev->variant->num_clks);
		return -EINVAL;
	}

	for (i = 0; i < dev->num_clks; i++) {
		cr = clk_get_rate(dev->clks[i].clk);
		for (j = 0; j < ARRAY_SIZE(dev->variant->rkclks); j++) {
			if (dev->variant->rkclks[j].max == 0)
				continue;
			if (strcmp(dev->variant->rkclks[j].name, dev->clks[i].id))
				continue;
			if (cr > dev->variant->rkclks[j].max) {
				err = clk_set_rate(dev->clks[i].clk,
						   dev->variant->rkclks[j].max);
				if (err)
					dev_err(dev->dev, "Fail downclocking %s from %lu to %lu\n",
						dev->variant->rkclks[j].name, cr,
						dev->variant->rkclks[j].max);
				else
					dev_info(dev->dev, "Downclocking %s from %lu to %lu\n",
						 dev->variant->rkclks[j].name, cr,
						 dev->variant->rkclks[j].max);
			}
		}
	}
	return 0;
}

static int rk_crypto_enable_clk(struct rk_crypto_info *dev)
{
	int err;

	err = clk_bulk_prepare_enable(dev->num_clks, dev->clks);
	if (err)
		dev_err(dev->dev, "Could not enable clock clks\n");

	return err;
}

static void rk_crypto_disable_clk(struct rk_crypto_info *dev)
{
	clk_bulk_disable_unprepare(dev->num_clks, dev->clks);
}

/*
 * Power management strategy: The device is suspended until a request
 * is handled. For avoiding suspend/resume yoyo, the autosuspend is set to 2s.
 */
static int rk_crypto_pm_suspend(struct device *dev)
{
	struct rk_crypto_info *rkdev = dev_get_drvdata(dev);

	rk_crypto_disable_clk(rkdev);
	reset_control_assert(rkdev->rst);

	return 0;
}

static int rk_crypto_pm_resume(struct device *dev)
{
	struct rk_crypto_info *rkdev = dev_get_drvdata(dev);
	int ret;

	ret = rk_crypto_enable_clk(rkdev);
	if (ret)
		return ret;

	reset_control_deassert(rkdev->rst);
	return 0;

}

static const struct dev_pm_ops rk_crypto_pm_ops = {
	SET_RUNTIME_PM_OPS(rk_crypto_pm_suspend, rk_crypto_pm_resume, NULL)
};

static int rk_crypto_pm_init(struct rk_crypto_info *rkdev)
{
	int err;

	pm_runtime_use_autosuspend(rkdev->dev);
	pm_runtime_set_autosuspend_delay(rkdev->dev, 2000);

	err = pm_runtime_set_suspended(rkdev->dev);
	if (err)
		return err;
	pm_runtime_enable(rkdev->dev);
	return err;
}

static void rk_crypto_pm_exit(struct rk_crypto_info *rkdev)
{
	pm_runtime_disable(rkdev->dev);
}

static irqreturn_t rk_crypto_irq_handle(int irq, void *dev_id)
{
	struct rk_crypto_info *dev  = platform_get_drvdata(dev_id);
	u32 interrupt_status;

	interrupt_status = CRYPTO_READ(dev, RK_CRYPTO_INTSTS);
	CRYPTO_WRITE(dev, RK_CRYPTO_INTSTS, interrupt_status);

	dev->status = 1;
	if (interrupt_status & 0x0a) {
		dev_warn(dev->dev, "DMA Error\n");
		dev->status = 0;
	}
	complete(&dev->complete);

	return IRQ_HANDLED;
}

static struct rk_crypto_tmp *rk_cipher_algs[] = {
	&rk_ecb_aes_alg,
	&rk_cbc_aes_alg,
	&rk_ecb_des_alg,
	&rk_cbc_des_alg,
	&rk_ecb_des3_ede_alg,
	&rk_cbc_des3_ede_alg,
	&rk_ahash_sha1,
	&rk_ahash_sha256,
	&rk_ahash_md5,
};

static int rk_crypto_debugfs_show(struct seq_file *seq, void *v)
{
	struct rk_crypto_info *dd;
	unsigned int i;

	spin_lock(&rocklist.lock);
	list_for_each_entry(dd, &rocklist.dev_list, list) {
		seq_printf(seq, "%s %s requests: %lu\n",
			   dev_driver_string(dd->dev), dev_name(dd->dev),
			   dd->nreq);
	}
	spin_unlock(&rocklist.lock);

	for (i = 0; i < ARRAY_SIZE(rk_cipher_algs); i++) {
		if (!rk_cipher_algs[i]->registered)
			continue;
		switch (rk_cipher_algs[i]->type) {
		case CRYPTO_ALG_TYPE_SKCIPHER:
			seq_printf(seq, "%s %s reqs=%lu fallback=%lu\n",
				   rk_cipher_algs[i]->alg.skcipher.base.base.cra_driver_name,
				   rk_cipher_algs[i]->alg.skcipher.base.base.cra_name,
				   rk_cipher_algs[i]->stat_req, rk_cipher_algs[i]->stat_fb);
			seq_printf(seq, "\tfallback due to length: %lu\n",
				   rk_cipher_algs[i]->stat_fb_len);
			seq_printf(seq, "\tfallback due to alignment: %lu\n",
				   rk_cipher_algs[i]->stat_fb_align);
			seq_printf(seq, "\tfallback due to SGs: %lu\n",
				   rk_cipher_algs[i]->stat_fb_sgdiff);
			break;
		case CRYPTO_ALG_TYPE_AHASH:
			seq_printf(seq, "%s %s reqs=%lu fallback=%lu\n",
				   rk_cipher_algs[i]->alg.hash.base.halg.base.cra_driver_name,
				   rk_cipher_algs[i]->alg.hash.base.halg.base.cra_name,
				   rk_cipher_algs[i]->stat_req, rk_cipher_algs[i]->stat_fb);
			break;
		}
	}
	return 0;
}

DEFINE_SHOW_ATTRIBUTE(rk_crypto_debugfs);

static void register_debugfs(struct rk_crypto_info *crypto_info)
{
	struct dentry *dbgfs_dir __maybe_unused;
	struct dentry *dbgfs_stats __maybe_unused;

	/* Ignore error of debugfs */
	dbgfs_dir = debugfs_create_dir("rk3288_crypto", NULL);
	dbgfs_stats = debugfs_create_file("stats", 0444, dbgfs_dir, &rocklist,
					  &rk_crypto_debugfs_fops);

#ifdef CONFIG_CRYPTO_DEV_ROCKCHIP_DEBUG
	rocklist.dbgfs_dir = dbgfs_dir;
	rocklist.dbgfs_stats = dbgfs_stats;
#endif
}

static int rk_crypto_register(struct rk_crypto_info *crypto_info)
{
	unsigned int i, k;
	int err = 0;

	for (i = 0; i < ARRAY_SIZE(rk_cipher_algs); i++) {
		switch (rk_cipher_algs[i]->type) {
		case CRYPTO_ALG_TYPE_SKCIPHER:
			dev_info(crypto_info->dev, "Register %s as %s\n",
				 rk_cipher_algs[i]->alg.skcipher.base.base.cra_name,
				 rk_cipher_algs[i]->alg.skcipher.base.base.cra_driver_name);
			err = crypto_engine_register_skcipher(&rk_cipher_algs[i]->alg.skcipher);
			break;
		case CRYPTO_ALG_TYPE_AHASH:
			dev_info(crypto_info->dev, "Register %s as %s\n",
				 rk_cipher_algs[i]->alg.hash.base.halg.base.cra_name,
				 rk_cipher_algs[i]->alg.hash.base.halg.base.cra_driver_name);
			err = crypto_engine_register_ahash(&rk_cipher_algs[i]->alg.hash);
			break;
		default:
			dev_err(crypto_info->dev, "unknown algorithm\n");
		}
		if (err)
			goto err_cipher_algs;
		rk_cipher_algs[i]->registered = true;
	}
	return 0;

err_cipher_algs:
	/*
	 * Every index here is k, the algorithm being torn down -- not i, the one
	 * that just failed to register and therefore has nothing to tear down.
	 * Two of the three used to read i: the TYPE test and the ahash unregister.
	 * With a mixed array (rk_cipher_algs holds both skciphers and ahashes)
	 * that mis-selected the branch from the WRONG algorithm's type, so a
	 * failing ahash with any registered skcipher below it ran the ahash arm k
	 * times against rk_cipher_algs[i]->alg.hash -- an alg that was never
	 * registered -- while every skcipher that WAS registered stayed
	 * registered, outliving the rk_crypto_info that probe's unwind then
	 * devm-frees. Pre-existing upstream; found while checking the claim
	 * corrected in the probe unwind comment below.
	 */
	for (k = 0; k < i; k++) {
		if (rk_cipher_algs[k]->type == CRYPTO_ALG_TYPE_SKCIPHER)
			crypto_engine_unregister_skcipher(&rk_cipher_algs[k]->alg.skcipher);
		else
			crypto_engine_unregister_ahash(&rk_cipher_algs[k]->alg.hash);
		rk_cipher_algs[k]->registered = false;
	}
	return err;
}

static void rk_crypto_unregister(void)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(rk_cipher_algs); i++) {
		if (rk_cipher_algs[i]->type == CRYPTO_ALG_TYPE_SKCIPHER)
			crypto_engine_unregister_skcipher(&rk_cipher_algs[i]->alg.skcipher);
		else
			crypto_engine_unregister_ahash(&rk_cipher_algs[i]->alg.hash);
		rk_cipher_algs[i]->registered = false;
	}
}

/*
 * The TRNG produces 256 bits per trigger.  The sample period is the number of
 * cycles between two samples of the ring oscillator, one output bit per
 * sample.
 *
 * 100 is the value Rockchip's own U-Boot driver programs for this IP
 * (drivers/rng/rockchip_rng.c, rk_cryptov1_rng_read), and it is TOO SHORT:
 * consecutive samples of the ring are correlated, so the output carries run
 * patterns (0x00/0xff/0x03/0xc0 ... at 2-4x their expected frequency, with
 * WHICH bytes drifting boot to boot as the ring frequency drifts).  Measured
 * on an RK3399 (ROC-RK3399-PC, both crypto instances), 256 KiB per arm,
 * byte chi^2 over 256 bins (df 255, 3-sigma band 255 +- 68) and a
 * Wald-Wolfowitz runs test on the bit stream (|z| < 3 expected):
 *
 *   sample period    50     100    200    400    1000
 *   byte chi^2   307061  221337   7937   2187   1914   (100: 1 MiB)
 *   runs-test z  -144.5  -116.9   -1.0   -2.0   -0.9
 *
 * From 400 on the samples are independent within measurement; 1000 leaves
 * 2.5x of margin for the ring slowing with temperature and voltage, at
 * 21.7 kB/s (100 gave 141 kB/s) -- far more than the hwrng core draws.
 * Enabling the oscillator earlier (200 us / 2 ms before TRNG_START) or
 * leaving it running between reads changed nothing, so this is the sample
 * spacing, not oscillator warm-up.
 *
 * What the period does NOT fix is a DC bias of the source: every one of the
 * 32 output bit positions reads a one with p = 0.4855 +- 0.002 at every
 * period tried (monobit z = -41 on 2 Mbit), which is the ring's duty cycle,
 * not a driver artefact.  It caps the min-entropy at about 0.96 bit per
 * output bit, and it is what the residual chi^2 of ~1900 per 256 KiB is:
 * with independent bits at that bias the expected byte chi^2 is ~1760 +
 * 255.  RK_CRYPTO_TRNG_QUALITY (256 per 1024 bits) is four times below
 * that cap, so it stays as it is.
 *
 * The 256-bit trigger then takes ~1.5 ms, well inside the poll below.
 */
#define RK_CRYPTO_TRNG_SAMPLE_CYCLES	1000
#define RK_CRYPTO_TRNG_POLL_PERIOD_US	10
#define RK_CRYPTO_TRNG_POLL_TIMEOUT_US	50000

/*
 * Entropy credited per 1024 output bits.  The neighbouring Rockchip TRNGs use
 * 900, and rockchip-rng.c uses 999 for rk3576/rk3588 "as determined by actual
 * testing", but neither number is available to us here: this block has no
 * public TRM chapter, no vendor min-entropy claim, and no way to read its raw
 * noise, so there is nothing to base an SP 800-90B estimate on.  What the
 * driver can assert is only what rk_crypto_trng_health_check() actually tests
 * -- that the source has not died in one of the few ways the gate can name.
 * Read its DETECTION LIMITS paragraph: a counter or an LFSR passes every
 * read, so "not dead" is a weaker statement than it looks.  256 credits one
 * bit of entropy per four output bits, which is deliberately far below any
 * plausible true rate: the pool fills a little more slowly and a subtle
 * shortfall this driver cannot detect cannot dominate it.  It is left at 256
 * rather than lowered further because it was never derived from the gate's
 * discriminating power in the first place -- there is no min-entropy estimate
 * to move it against, and swapping one unmeasured number for another buys
 * nothing.  Raising it needs a measured min-entropy estimate, not a
 * comparison to a sibling driver.
 */
#define RK_CRYPTO_TRNG_QUALITY		256

static const char * const rk_crypto_trng_health_str[] = {
	[RK_TRNG_HEALTH_OK]		= "ok",
	[RK_TRNG_HEALTH_ALL_ZERO]	= "all-zero block",
	[RK_TRNG_HEALTH_STUCK_WORD]	= "every word identical",
	[RK_TRNG_HEALTH_REPEAT_BLOCK]	= "block repeats a recent one",
	[RK_TRNG_HEALTH_BAD_LENGTH]	= "block length out of range",
};

/*
 * Consecutive FAILED READS after which the source is declared dead -- either
 * arm: a health-test rejection, or a START poll that timed out.  Both are a
 * read that produced no usable entropy, and counting only the first left the
 * likelier hardware death (a block that never completes a trigger) unable to
 * reach the latch at all.
 *
 * A working 256-bit source fails a single HEALTH TEST with probability around
 * 2^-250, so on that arm any threshold above 1 is unreachable by chance and
 * this is set well clear of it.  That argument does not transfer to the
 * timeout arm, which has no per-read probability; there the threshold is
 * justified by duration instead -- see the comment at the poll.
 */
#define RK_CRYPTO_TRNG_MAX_FAILS	16

/*
 * How long a source declared dead is refused for before it is given ONE real
 * read to prove itself with.  The latch used to be permanent, and a permanent
 * latch has no way out: nothing above ->read() ever clears it, and a latched
 * read returns before touching the block, so a source that was only
 * transiently wedged stayed off until module unload.
 *
 * Upstream's rockchip-rng.c has no latch at all -- every failed read is a
 * full attempt, retried by hwrng_fillfn() every 10 s forever.  This keeps
 * the latch's saving (a dead block costs one 50 ms poll per minute instead of
 * one per 10 s, and the ratelimited failure line is printed a sixth as often)
 * and bounds what it can cost a live one: a source that wedges and recovers
 * is back within a minute, not never.
 *
 * Measured on RK3399 (ROC-RK3399-PC) on the production
 * driver: 133,991 consecutive 32-byte /dev/hwrng reads under four and then
 * eight AF_ALG loads holding both engines at their ceiling (40-42 MB/s of
 * md5/sha256/AES-CBC DMA, flat from 4 to 8 loads) returned 133,991 full
 * blocks -- zero -EIO, zero -ETIMEDOUT, zero short reads, zero TRNG log
 * lines -- and 87,738 no-load reads did the same.  No load this board can
 * drive reaches the latch, so the only way to it is a block that stops
 * answering, for which one probe a minute is cheap and a permanent refusal
 * was an overreach.  The rate under load (24.7 kB/s) equals the rate
 * without, so the engines and the TRNG do not even slow each other.
 */
#define RK_CRYPTO_TRNG_DEAD_RETRY_MS	60000U

/*
 * Force one of the two ways this source can die, neither of which good silicon
 * will produce on demand, so both are debug-build knobs rather than dead code.
 *
 *   1 = ZEROS.  Skip the oscillator enable, so the block still completes a
 *       trigger and TRNG_DOUT reads back zeros.  This is the failure the
 *       health test exists to catch.
 *   2 = STALL.  Make the RK_CRYPTO_TRNG_START poll time out, which is the
 *       OTHER dead mode: a block that never completes a trigger at all.
 *       Injected as a term of the poll's success CONDITION -- not after the
 *       poll, which is where it was first written and which left the poll
 *       succeeding and the message lying about it (see the poll).  No host
 *       write can stop real hardware from clearing START, so what this cannot
 *       force is the hardware bit's state at out:; everything on the software
 *       side of the poll, the poll's own exit included, is real.
 *
 * Mode 2 exists because the bench plan for the dead-source latch was
 * self-confirming without it: trng_fault_inject=1 drives only the zeros path,
 * so the arm that would have exposed the timeout exit not being counted at all
 * was absent from the plan that was supposed to test the counting.
 */
#define RK_TRNG_INJECT_OFF	0
#define RK_TRNG_INJECT_ZEROS	1
#define RK_TRNG_INJECT_STALL	2

#ifdef CONFIG_CRYPTO_DEV_ROCKCHIP_DEBUG
static unsigned int trng_fault_inject;
module_param(trng_fault_inject, uint, 0644);
MODULE_PARM_DESC(trng_fault_inject,
		 "debug: 0=off, 1=leave the ring oscillator off so reads return zeros, 2=force the TRNG_START poll to time out");
#else
#define trng_fault_inject RK_TRNG_INJECT_OFF
#endif

/*
 * Record one failed read and latch the source dead once they stop being
 * isolated.  Shared by both failure exits so that neither can quietly stop
 * counting: the health-test rejection and the START-poll timeout are different
 * evidence for the same conclusion, and RK_CRYPTO_TRNG_MAX_FAILS is a count of
 * consecutive FAILED READS, not of health verdicts.
 *
 * A latched source is re-probed once per RK_CRYPTO_TRNG_DEAD_RETRY_MS (see
 * the top of rk_crypto_trng_read()).  The count is saturated at the threshold
 * so that a failed re-probe re-arms the cooldown at once, and only the FIRST
 * latch is loud: a block that stays dead re-arms silently every cooldown,
 * with the ratelimited line at the failing exit as its only trace, rather
 * than printing "dead" at the same cadence it is re-probed.
 */
static void rk_crypto_trng_note_failure(struct rk_crypto_info *dev,
					const char *why)
{
	if (++dev->trng_fails < RK_CRYPTO_TRNG_MAX_FAILS)
		return;

	dev->trng_fails = RK_CRYPTO_TRNG_MAX_FAILS;
	dev->trng_retry_at = jiffies +
			     msecs_to_jiffies(RK_CRYPTO_TRNG_DEAD_RETRY_MS);
	if (dev->trng_dead) {
		dev_dbg(dev->dev, "TRNG re-probe failed (%s); next in %u ms\n",
			why, RK_CRYPTO_TRNG_DEAD_RETRY_MS);
		return;
	}

	dev->trng_dead = true;
	dev_err(dev->dev,
		"TRNG failed %u consecutive reads (last: %s); treating the source as dead, re-probing it every %u ms\n",
		dev->trng_fails, why, RK_CRYPTO_TRNG_DEAD_RETRY_MS);
}

static int rk_crypto_trng_read(struct hwrng *rng, void *buf, size_t max,
			       bool wait)
{
	struct rk_crypto_info *dev = container_of(rng, struct rk_crypto_info,
						  hwrng);
	size_t to_read = min_t(size_t, max, RK_CRYPTO_TRNG_MAX_BYTE);
	size_t words = DIV_ROUND_UP(to_read, sizeof(u32));
	u32 data[RK_CRYPTO_TRNG_MAX_WORD];
	u32 reg;
	int ret, health;
	size_t i;

	/*
	 * There is no "data ready" status to sample without triggering, so a
	 * non-blocking caller gets nothing rather than a half-started TRNG.
	 */
	if (!wait)
		return 0;

	/*
	 * A source that has failed RK_CRYPTO_TRNG_MAX_FAILS reads in a row is
	 * dead, and nothing above us will stop asking: hwrng_fillfn() treats
	 * every rc <= 0 the same way -- hwrng_msleep(rng, 10000) and continue
	 * -- so it re-triggers the block and re-logs the failure every 10 s
	 * forever.
	 *
	 * This latch does not stop the asking; it makes answering free.  The
	 * kthread still comes back every 10 s and still gets -EIO, but from
	 * here, before any PM resume, any register write to a block that is
	 * not answering, or any log line.  -EIO rather than 0 so that
	 * rng_dev_read(), which unlike hwrng_fillfn() does distinguish them,
	 * tells userspace the truth.
	 *
	 * Stopping the loop outright means unregistering the hwrng, and
	 * hwrng_unregister() cannot be called from ->read(): the core holds
	 * reading_mutex across the call and takes rng_mutex inside unregister.
	 * Doing it properly needs a work item, which is a larger change than
	 * this fix.
	 *
	 * The way out: once per RK_CRYPTO_TRNG_DEAD_RETRY_MS the latch lets
	 * exactly one read through as a probe.  trng_dead stays set across
	 * it and trng_fails sits at the threshold, so if the probe fails
	 * rk_crypto_trng_note_failure() re-arms the cooldown at once (and
	 * quietly), and if it succeeds the success path below clears both.
	 * A transiently wedged source is therefore back within one cooldown;
	 * a dead one costs one real attempt per cooldown instead of one per
	 * hwrng_fillfn() retry.  Nothing else clears the latch: an AES or
	 * hash completion says nothing about the ring oscillator, and a
	 * reset-triggered clear would need the reset to be ours to trigger.
	 */
	if (dev->trng_dead && time_before(jiffies, dev->trng_retry_at))
		return -EIO;

	ret = pm_runtime_resume_and_get(dev->dev);
	if (ret < 0)
		return ret;

	/*
	 * Enable the ring oscillator and set its sample period.  This is the
	 * entropy source: with RK_CRYPTO_TRNG_CTRL left at reset the block
	 * still completes a trigger, but every output word reads back zero.
	 */
	CRYPTO_WRITE(dev, RK_CRYPTO_TRNG_CTRL,
		     (trng_fault_inject == RK_TRNG_INJECT_ZEROS ?
		      0 : RK_CRYPTO_TRNG_OSC_ENABLE) |
		     RK_CRYPTO_TRNG_SAMPLE_PERIOD(RK_CRYPTO_TRNG_SAMPLE_CYCLES));

	/*
	 * RK_CRYPTO_CTRL is a hiword-masked register.  Touch only TRNG_START
	 * so that an AES or hash request running on the crypto engine is not
	 * cancelled by this read -- writing RK_CRYPTO_WRITE_MASK would clear
	 * every other start bit in the same word.
	 */
	CRYPTO_WRITE(dev, RK_CRYPTO_CTRL,
		     (RK_CRYPTO_TRNG_START << 16) | RK_CRYPTO_TRNG_START);

	/*
	 * START self-clears once the 256-bit value is ready.
	 *
	 * The stall injection is a term of the poll's own success CONDITION, and
	 * has to be: it used to be an `if (trng_fault_inject == ...) ret =
	 * -ETIMEDOUT;` AFTER this call, where the poll had already SUCCEEDED. So
	 * the mode advertised as "make the START poll time out" left the poll
	 * exit -- the only part of it that is not shared with the healthy path --
	 * the one thing never executed, skipped the RK_CRYPTO_TRNG_POLL_TIMEOUT_US
	 * of real polling it is supposed to cost, and then printed "TRNG START did
	 * not clear" about a START that had just cleared. A debug mode whose
	 * purpose is validating what this arm logs must not itself make the log
	 * lie. Written this way the timeout is produced BY readl_poll_timeout(),
	 * so the spin, the -ETIMEDOUT and the message are all the real thing.
	 *
	 * What it still cannot force is the STATE OF THE HARDWARE BIT at out:.
	 * No host write stops real silicon from clearing START, so out: runs its
	 * stop sequence against a block that has in fact finished. Everything on
	 * the software side of the poll -- the counted failure, the latch, the
	 * ratelimited message, the PM put -- is exercised for real.
	 *
	 * With CONFIG_CRYPTO_DEV_ROCKCHIP_DEBUG off, trng_fault_inject is a
	 * literal RK_TRNG_INJECT_OFF and the added term folds away.
	 */
	ret = readl_poll_timeout(dev->reg + RK_CRYPTO_CTRL, reg,
				 !(reg & RK_CRYPTO_TRNG_START) &&
				 trng_fault_inject != RK_TRNG_INJECT_STALL,
				 RK_CRYPTO_TRNG_POLL_PERIOD_US,
				 RK_CRYPTO_TRNG_POLL_TIMEOUT_US);
	if (ret < 0) {
		/*
		 * COUNTED, and it was not before.  A block whose START never
		 * self-clears is dead in the plainer sense than the one the
		 * health test catches -- the health test only ever sees a
		 * source that answers with bad data, while this exit is a
		 * source that does not answer at all -- and it is at least as
		 * likely.  Left uncounted, this path returned -ETIMEDOUT on
		 * every read forever, paying a full PM resume, an oscillator
		 * enable and a 50 ms poll each time, and could never reach the
		 * latch that exists to make exactly that cheap.
		 *
		 * The 2^-250 argument under RK_CRYPTO_TRNG_MAX_FAILS does NOT
		 * cover this arm -- there is no per-read probability to appeal
		 * to for a timeout.  What justifies sharing the threshold is
		 * duration: hwrng_fillfn() retries every 10 s, so 16 in a row
		 * is a block that has not completed a single trigger in about
		 * 160 s, having been given 50 ms each time.
		 */
		dev_err_ratelimited(dev->dev,
				    "TRNG START did not clear within %d us\n",
				    RK_CRYPTO_TRNG_POLL_TIMEOUT_US);
		rk_crypto_trng_note_failure(dev, "START never cleared");
		goto out;
	}

	/*
	 * Read into a local u32 array rather than casting the caller's void *
	 * buffer: the ->read() contract promises no u32 alignment (the core's
	 * own buffers happen to be aligned today, which is not the same
	 * thing), and the block has to be judged before any of it is handed
	 * up.  A short read still fetches whole words -- the odd tail lands at
	 * most on TRNG_DOUT_7 (0x0220), inside the eight-word window.
	 */
	for (i = 0; i < words; i++)
		data[i] = CRYPTO_READ(dev, RK_CRYPTO_TRNG_DOUT_0 + i * 4);

	/*
	 * Nothing in the block reports a dead entropy source: START self-
	 * clears with the oscillator off and TRNG_DOUT reads back zeros, so
	 * the only evidence available is the data itself.  Serialised by the
	 * hwrng core's reading_mutex, which is held across every ->read().
	 */
	health = rk_crypto_trng_health_check(&dev->trng_health, data, words);
	if (health != RK_TRNG_HEALTH_OK) {
		dev_err_ratelimited(dev->dev,
				    "TRNG health test failed (%s), discarding %zu bytes\n",
				    rk_crypto_trng_health_str[health], to_read);
		/*
		 * -EIO rather than 0, but only rng_dev_read() can tell the
		 * difference: it propagates a negative rc to userspace, where
		 * a zero-length read would leave it looping on a dead source.
		 *
		 * hwrng_fillfn() cannot.  Its test is `if (rc <= 0)
		 * hwrng_msleep(rng, 10000)` followed by `if (rc <= 0)
		 * continue`, so -EIO and 0 take the identical path and the
		 * kthread retries forever either way.
		 *
		 * The trng_dead latch below does NOT stop that, and this
		 * comment used to say it did.  The latch also returns -EIO, so
		 * by the very logic quoted above hwrng_fillfn() keeps looping
		 * at its 10 s cadence for as long as the module is loaded.
		 * What the latch changes is the COST of each retry, not the
		 * fact of it: a latched read returns at the top of this
		 * function, so it costs no PM resume, no register traffic to a
		 * block that is not answering, and no log line -- instead of a
		 * full resume, an oscillator enable, a trigger, a poll, eight
		 * MMIO reads and a ratelimited error every 10 s.  Stopping the
		 * loop outright means unregistering the hwrng, which cannot be
		 * done from ->read() (see the latch comment at the top).
		 */
		rk_crypto_trng_note_failure(dev,
					    rk_crypto_trng_health_str[health]);
		ret = -EIO;
		goto out;
	}

	if (dev->trng_dead) {
		dev->trng_dead = false;
		dev_warn(dev->dev,
			 "TRNG source answered a re-probe after %u failed reads; back in service\n",
			 dev->trng_fails);
	}
	dev->trng_fails = 0;
	memcpy(buf, data, to_read);

out:
	memzero_explicit(data, sizeof(data));
	/* Stop the TRNG and switch the ring oscillator back off. */
	CRYPTO_WRITE(dev, RK_CRYPTO_CTRL, RK_CRYPTO_TRNG_START << 16);
	CRYPTO_WRITE(dev, RK_CRYPTO_TRNG_CTRL, 0);
	pm_runtime_mark_last_busy(dev->dev);
	pm_runtime_put_autosuspend(dev->dev);

	return ret < 0 ? ret : (int)to_read;
}

static const struct of_device_id crypto_of_id_table[] = {
	{ .compatible = "rockchip,rk3288-crypto",
	  .data = &rk3288_variant,
	},
	{ .compatible = "rockchip,rk3328-crypto",
	  .data = &rk3328_variant,
	},
	{ .compatible = "rockchip,rk3399-crypto",
	  .data = &rk3399_variant,
	},
	{}
};
MODULE_DEVICE_TABLE(of, crypto_of_id_table);

static int rk_crypto_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rk_crypto_info *crypto_info, *first;
	int err = 0;

	crypto_info = devm_kzalloc(&pdev->dev,
				   sizeof(*crypto_info), GFP_KERNEL);
	if (!crypto_info) {
		err = -ENOMEM;
		goto err_crypto;
	}

	crypto_info->dev = &pdev->dev;
	platform_set_drvdata(pdev, crypto_info);

	crypto_info->variant = of_device_get_match_data(&pdev->dev);
	if (!crypto_info->variant) {
		dev_err(&pdev->dev, "Missing variant\n");
		return -EINVAL;
	}

	crypto_info->rst = devm_reset_control_array_get_exclusive(dev);
	if (IS_ERR(crypto_info->rst)) {
		err = PTR_ERR(crypto_info->rst);
		goto err_crypto;
	}

	reset_control_assert(crypto_info->rst);
	usleep_range(10, 20);
	reset_control_deassert(crypto_info->rst);

	crypto_info->reg = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(crypto_info->reg)) {
		err = PTR_ERR(crypto_info->reg);
		goto err_crypto;
	}

	err = rk_crypto_get_clks(crypto_info);
	if (err)
		goto err_crypto;

	crypto_info->irq = platform_get_irq(pdev, 0);
	if (crypto_info->irq < 0) {
		err = crypto_info->irq;
		goto err_crypto;
	}

	err = devm_request_irq(&pdev->dev, crypto_info->irq,
			       rk_crypto_irq_handle, IRQF_SHARED,
			       "rk-crypto", pdev);

	if (err) {
		dev_err(&pdev->dev, "irq request failed.\n");
		goto err_crypto;
	}

	crypto_info->engine = crypto_engine_alloc_init(&pdev->dev, true);
	if (!crypto_info->engine) {
		err = -ENOMEM;
		goto err_crypto;
	}

	crypto_engine_start(crypto_info->engine);
	init_completion(&crypto_info->complete);

	err = rk_crypto_pm_init(crypto_info);
	if (err)
		goto err_pm;

	spin_lock(&rocklist.lock);
	first = list_first_entry_or_null(&rocklist.dev_list,
					 struct rk_crypto_info, list);
	list_add_tail(&crypto_info->list, &rocklist.dev_list);
	spin_unlock(&rocklist.lock);

	if (!first) {
		err = rk_crypto_register(crypto_info);
		if (err) {
			dev_err(dev, "Fail to register crypto algorithms");
			goto err_register_alg;
		}

		register_debugfs(crypto_info);
	}

	crypto_info->hwrng.name = devm_kasprintf(dev, GFP_KERNEL,
						 "rk3288-crypto-trng-%s",
						 dev_name(dev));
	if (!crypto_info->hwrng.name) {
		err = -ENOMEM;
		goto err_hwrng;
	}
	crypto_info->hwrng.read = rk_crypto_trng_read;
	crypto_info->hwrng.quality = RK_CRYPTO_TRNG_QUALITY;

	err = devm_hwrng_register(dev, &crypto_info->hwrng);
	if (err) {
		dev_err(dev, "Failed to register hwrng: %d\n", err);
		goto err_hwrng;
	}

	return 0;

	/*
	 * The unwind is split exactly where the list membership is, because
	 * crypto_info is devm_kzalloc()'d against pdev: returning an error from
	 * probe frees it, so any path that returns while the entry is still on
	 * rocklist.dev_list leaves that list pointing into freed memory.
	 *
	 * err_hwrng and err_register_alg are the only two labels reachable
	 * AFTER the list_add_tail() above, and both now flow through the
	 * list_del() at err_register_alg -- once each, since err_hwrng falls
	 * into it rather than repeating it.  err_pm and err_crypto are only
	 * ever reached from gotos that run BEFORE the list_add_tail(), so they
	 * must not unwind a membership that was never taken; that is why the
	 * list_del() sits at err_register_alg and not lower down.
	 *
	 * rk_crypto_remove() also list_del()s, but it never runs for a device
	 * whose probe failed, so there is no double-del from either path.
	 *
	 * rk_crypto_unregister() is safe to call before the list_del() because
	 * it walks the static rk_cipher_algs[] array, not rocklist, so the
	 * entry still being linked at that point does not matter.
	 *
	 * The `if (!first)` mirrors the registration above: only the first
	 * probed instance registered the algorithms and debugfs, so only it
	 * tears them down.
	 *
	 * ⚠️ WITHDRAWN, 2026-09-03: this used to add "err_register_alg is reached
	 * only when that same first instance's rk_crypto_register() failed, so
	 * nothing it would have torn down was ever installed."  That is FALSE.
	 * rk_crypto_register() registers the algorithms one at a time and jumps
	 * to its own err_cipher_algs on the i-th failure, so algorithms 0..i-1
	 * ARE installed when it returns an error.  What makes err_register_alg
	 * correct is not that nothing was installed -- it is that
	 * rk_crypto_register() unwinds its own partial success before returning,
	 * which is a property of THAT function and had to be fixed to be true
	 * (see the index note at its err_cipher_algs label).  debugfs is the one
	 * thing genuinely never installed on this path: register_debugfs() runs
	 * only after rk_crypto_register() has returned 0.
	 */
err_hwrng:
	if (!first) {
#ifdef CONFIG_CRYPTO_DEV_ROCKCHIP_DEBUG
		debugfs_remove_recursive(rocklist.dbgfs_dir);
#endif
		rk_crypto_unregister();
	}
err_register_alg:
	spin_lock(&rocklist.lock);
	list_del(&crypto_info->list);
	spin_unlock(&rocklist.lock);
	rk_crypto_pm_exit(crypto_info);
err_pm:
	crypto_engine_exit(crypto_info->engine);
err_crypto:
	dev_err(dev, "Crypto Accelerator not successfully registered\n");
	return err;
}

static void rk_crypto_remove(struct platform_device *pdev)
{
	struct rk_crypto_info *crypto_tmp = platform_get_drvdata(pdev);
	struct rk_crypto_info *first;

	spin_lock_bh(&rocklist.lock);
	list_del(&crypto_tmp->list);
	first = list_first_entry_or_null(&rocklist.dev_list,
					 struct rk_crypto_info, list);
	spin_unlock_bh(&rocklist.lock);

	if (!first) {
#ifdef CONFIG_CRYPTO_DEV_ROCKCHIP_DEBUG
		debugfs_remove_recursive(rocklist.dbgfs_dir);
#endif
		rk_crypto_unregister();
	}
	rk_crypto_pm_exit(crypto_tmp);
	crypto_engine_exit(crypto_tmp->engine);
}

static struct platform_driver crypto_driver = {
	.probe		= rk_crypto_probe,
	.remove		= rk_crypto_remove,
	.driver		= {
		.name	= "rk3288-crypto",
		.pm		= &rk_crypto_pm_ops,
		.of_match_table	= crypto_of_id_table,
	},
};

module_platform_driver(crypto_driver);

MODULE_AUTHOR("Zain Wang <zain.wang@rock-chips.com>");
MODULE_DESCRIPTION("Support for Rockchip's cryptographic engine");
MODULE_LICENSE("GPL");
