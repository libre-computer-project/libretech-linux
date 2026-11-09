// SPDX-License-Identifier: GPL-2.0-only
/*
 * Amlogic GXL DDR PHY (PUB) VT telemetry monitor
 *
 * The GXL DDR PHY ("PUB", 0xc8836000) rescales its delay lines from a live
 * measurement of the clock period (VT drift tracking). The measurement and its
 * outputs are plain PUB registers and the block keeps no event counter, so the
 * statistics here are the driver's own: a delayed-work timer samples every
 * tracked word once per period and keeps base/cur/min/max/mean/changes per
 * field, so the delay-line behaviour can be read on a running board and a
 * memory fault lined up against what the PHY was doing.
 *
 * Word index << 2 = byte offset in the block. Base = u-boot
 * arch/arm/include/asm/arch-meson/dram-gxl.h DDR0_PUB_REG_BASE.
 *
 * word          name        fields        source
 * 0x1e8+0x40*l  DXlMDLR0    [8:0] [24:16] lane l period at init / live period;
 *                                       named in the interface spec, NOT in
 *                                       dram-gxl.h
 * 0x168         ACMDLR0     [8:0] [24:16] dram-gxl.h:25
 * 0x1e0+0x40*l  DXlLCDLR0   [8:0] [24:16] dram-gxl.h:138,172,182,192
 * 0x1e1+0x40*l  DXlLCDLR1   [8:0] [24:16] follows the LCDLR0/2 pattern, NOT
 *                                       named in dram-gxl.h
 * 0x1e2+0x40*l  DXlLCDLR2   [8:0] [24:16] dram-gxl.h:139,173,183,193
 * 0x160         ACLCDLR     [8:0] [24:16] dram-gxl.h:24
 * 0x0a          PGCR6       bit 0         tracking inhibit (INHVT);
 *                                       dram-gxl.h:21, read only
 * 0x0d          PGSR0       whole word    PHY status; dram-gxl.h:22
 * 0x0e          (PGSR1?)    whole word    VT stop candidate; name unconfirmed,
 *                                       reported by address (w0e)
 *
 * The driver never writes a PUB register: readl() only. The inhibit is a
 * measurement lever for userspace tools, not a driver control.
 *
 * System sleep: suspend stops the sampler and keeps a pre-suspend copy of
 * the sampled words. Resume records them against the fresh reads
 * (before/after in the resume debugfs file), counts itself, and continues
 * without resetting the statistics: a delay-line jump across the suspend is
 * not counted as a change.
 *
 * The debugfs table under /sys/kernel/debug/meson-ddr-pub/ is the whole
 * user interface; the device tree node is documented in
 * Documentation/devicetree/bindings/soc/amlogic/amlogic,gxl-ddr-pub-mon.yaml.
 *
 * Two sample sources select at probe:
 * SC mode is selected by an available `amlogic,sc-shmem` phandle naming an
 * SC-written SRAM block whose magic and version validate -- the AO Cortex-M3
 * (the SC) runs the sampler and this driver only reads the block under its
 * seqcount, writes the requested period and reset generation into the control
 * words, and replays the block's change ring through the `ring` debugfs file.
 * With no such node, a disabled one, or an invalid block (the vendor chain),
 * the in-kernel delayed-work sampler below runs exactly as before; the stats
 * header says which source is live (`source=sc|kernel`).
 */

#include <linux/bitfield.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/sched/signal.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/suspend.h>
#include <linux/timekeeping.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#define CREATE_TRACE_POINTS
#include <trace/events/meson_ddr_pub.h>

#define PUB_WORD(w)			((w) << 2)

/* DX*MDLR0: word 0x1e8 + 0x40 per data lane (spec names; see file header). */
#define PUB_DX0_MDLR0			PUB_WORD(0x1e8)
#define PUB_DX1_MDLR0			PUB_WORD(0x228)
#define PUB_DX2_MDLR0			PUB_WORD(0x268)
#define PUB_DX3_MDLR0			PUB_WORD(0x2a8)
#define PUB_ACMDLR0			PUB_WORD(0x168)

/* DX*LCDLR0/1/2: word 0x1e0/0x1e1/0x1e2 + 0x40 per data lane. */
#define PUB_DX0_LCDLR0			PUB_WORD(0x1e0)
#define PUB_DX0_LCDLR1			PUB_WORD(0x1e1)
#define PUB_DX0_LCDLR2			PUB_WORD(0x1e2)
#define PUB_DX1_LCDLR0			PUB_WORD(0x220)
#define PUB_DX1_LCDLR1			PUB_WORD(0x221)
#define PUB_DX1_LCDLR2			PUB_WORD(0x222)
#define PUB_DX2_LCDLR0			PUB_WORD(0x260)
#define PUB_DX2_LCDLR1			PUB_WORD(0x261)
#define PUB_DX2_LCDLR2			PUB_WORD(0x262)
#define PUB_DX3_LCDLR0			PUB_WORD(0x2a0)
#define PUB_DX3_LCDLR1			PUB_WORD(0x2a1)
#define PUB_DX3_LCDLR2			PUB_WORD(0x2a2)
#define PUB_ACLCDLR			PUB_WORD(0x160)

/* Status words, snapshot only. */
#define PUB_PGCR6			PUB_WORD(0x0a)
#define  PUB_PGCR6_INHVT		BIT(0)	/* VT tracking inhibit (N137) */
#define PUB_PGSR0			PUB_WORD(0x0d)
#define PUB_W0E				PUB_WORD(0x0e)	/* PGSR1 candidate */

#define PUB_FIELD_MASK			GENMASK(8, 0)	/* both tracked fields */

#define MESON_DDR_PUB_DEFAULT_PERIOD_MS	10
#define MESON_DDR_PUB_MIN_PERIOD_MS	1
#define MESON_DDR_PUB_MAX_PERIOD_MS	10000

enum {
	PUB_FIELD_LO,				/* [8:0]  */
	PUB_FIELD_HI,				/* [24:16] */
	PUB_FIELD_COUNT
};

struct pub_field {
	const char *name;
	unsigned int shift;
	u32 base;
	u32 cur;
	u32 min;
	u32 max;
	u32 prev;
	u64 sum;
	u64 changes;
};

struct pub_word {
	const char *name;			/* e.g. "dx0.MDLR0" */
	u32 off;
	u32 before;				/* whole word kept at the last suspend */
	u32 after;				/* ... and read again at its resume */
	struct pub_field fields[PUB_FIELD_COUNT];
};

#define PUB_W(_name, _off)							\
{										\
	.name = _name,								\
	.off = _off,								\
	.fields = {								\
		[PUB_FIELD_LO] = { .name = "lo", .shift = 0 },			\
		[PUB_FIELD_HI] = { .name = "hi", .shift = 16 },			\
	},									\
}

static const struct pub_word meson_ddr_pub_words[] = {
	PUB_W("dx0.MDLR0", PUB_DX0_MDLR0),
	PUB_W("dx1.MDLR0", PUB_DX1_MDLR0),
	PUB_W("dx2.MDLR0", PUB_DX2_MDLR0),
	PUB_W("dx3.MDLR0", PUB_DX3_MDLR0),
	PUB_W("ac.MDLR0", PUB_ACMDLR0),
	PUB_W("dx0.LCDLR0", PUB_DX0_LCDLR0),
	PUB_W("dx0.LCDLR1", PUB_DX0_LCDLR1),
	PUB_W("dx0.LCDLR2", PUB_DX0_LCDLR2),
	PUB_W("dx1.LCDLR0", PUB_DX1_LCDLR0),
	PUB_W("dx1.LCDLR1", PUB_DX1_LCDLR1),
	PUB_W("dx1.LCDLR2", PUB_DX1_LCDLR2),
	PUB_W("dx2.LCDLR0", PUB_DX2_LCDLR0),
	PUB_W("dx2.LCDLR1", PUB_DX2_LCDLR1),
	PUB_W("dx2.LCDLR2", PUB_DX2_LCDLR2),
	PUB_W("dx3.LCDLR0", PUB_DX3_LCDLR0),
	PUB_W("dx3.LCDLR1", PUB_DX3_LCDLR1),
	PUB_W("dx3.LCDLR2", PUB_DX3_LCDLR2),
	PUB_W("ac.LCDLR", PUB_ACLCDLR),
};

/*
 * SC mode shared block, written by the AO Cortex-M3 every period_us,
 * little-endian. The SC is the only writer
 * except for the two control words at 0x20/0x24. Field records follow the
 * order of meson_ddr_pub_words[] above, `lo` before `hi` per word.
 */
#define MESON_DDR_PUB_SC_MAGIC			0x53425550	/* "PUBS" */
#define MESON_DDR_PUB_SC_VERSION		1
#define MESON_DDR_PUB_SC_HDR_LEN		0x40
#define MESON_DDR_PUB_SC_SIZE			0x400
#define MESON_DDR_PUB_SC_NFIELDS		36
#define MESON_DDR_PUB_SC_WAIT_MS		500	/* every wait on the SC */
#define MESON_DDR_PUB_SC_MIN_PERIOD_US		1000	/* clamp the block's
							 * rate word before it
							 * drives any window
							 */
#define MESON_DDR_PUB_SC_MAX_PERIOD_US		10000000

#define SC_OFF_MAGIC				0x000
#define SC_OFF_VERSION				0x004	/* u16 version, u16 hdr_len */
#define SC_OFF_SEQ				0x008	/* odd while the SC writes */
#define SC_OFF_SAMPLES				0x00c
#define SC_OFF_PERIOD_US			0x010	/* rate the SC is running */
#define SC_OFF_SLEEP_COUNT			0x014	/* sleep-path entries seen */
#define SC_OFF_STATE				0x018	/* 0 sampling, 1 in sleep path */
#define SC_OFF_RING_HEAD			0x01c	/* next change-ring slot */
#define SC_OFF_CTRL_PERIOD_US			0x020	/* Linux -> SC, 0 = keep */
#define SC_OFF_CTRL_RESET_GEN			0x024	/* Linux -> SC */
#define SC_OFF_RESET_ACK			0x028	/* gen the SC last acted on */
#define SC_OFF_ISR_MAX_CYCLES			0x02c	/* SC sampling ISR peak */
#define SC_OFF_SAMPLES_HI			0x030	/* samples, high half */
#define SC_OFF_FIELDS				0x040
#define  SC_FIELD_SIZE				24
#define SC_OFF_RING				0x3a0
#define  SC_RING_SIZE				12
#define  SC_RING_SLOTS				8

struct pubmon_sc_field {				/* SC_OFF_FIELDS + n * 24 */
	u32 base;
	u32 cur;
	u32 min;
	u32 max;
	u32 changes;
	u64 sum;				/* changes u32 + pad u32 precede */
};

struct pubmon_sc_change {			/* SC_OFF_RING + n * 12 */
	u32 word;
	u32 field;
	u32 old;
	u32 new;
	u32 samples;
};

struct pubmon_sc_data {
	u32 seq;		/* the even seq the copy validated: the gate
				 * compares it with the previous copy's
				 */
	ktime_t t_obs;		/* taken before the first seq read of the
				 * successful pass: where that copy's move is
				 * timestamped from
				 */
	u64 samples;		/* 0x00c low, 0x030 high: passes the u32 range
				 * within days at fast rates
				 */
	u32 period_us;
	u32 sleeps;
	u32 state;
	u32 isr_max_cycles;
	u32 ring_head;
	struct pubmon_sc_field fields[MESON_DDR_PUB_SC_NFIELDS];
	struct pubmon_sc_change ring[SC_RING_SLOTS];
};

struct meson_ddr_pub {
	struct device *dev;
	void __iomem *regs;
	struct pub_word *words;
	unsigned int nwords;
	spinlock_t lock;	/* the in-kernel sampler: stats, samples,
				 * resumes, start, period, next, and the
				 * per-word before/after
				 */
	struct mutex sc_lock;	/* SC mode: the block cache and the control
				 * writes. SC mode has no atomic-context user,
				 * and a seqcount copy is ~170 device-SRAM
				 * reads, so it never runs under the spinlock
				 * with IRQs disabled.
				 */
	u64 samples;
	u64 resumes;		/* system-sleep resumes since probe (not reset by `reset`) */
	ktime_t start;
	ktime_t next;			/* absolute deadline of the next sample */
	unsigned long period_ms;
	struct delayed_work dwork;
	struct dentry *dir;
	bool sc_mode;		/* the SC samples; this driver only reads */
	void __iomem *sc;		/* SC block, SC_OFF_* offsets */
	struct pubmon_sc_data *sc_live;	/* the last consistent seqcount copy,
					 * only ever a copy that validated
					 */
	struct pubmon_sc_data *sc_scratch;	/* where a copy fills before
						 * committing to sc_live
						 */
	bool have_copy;			/* sc_live holds a committed copy */
	struct notifier_block hiber_nb;	/* SC mode: refuse image creation */
	u32 reset_gen;		/* generations written to SC_OFF_CTRL_RESET_GEN */
	bool reset_pending;	/* latched by a timed-out reset: stats are not
				 * current until a later `reset` write is
				 * acknowledged
				 */
	u32 last_seq;		/* the block's seq at the previous copy */
	ktime_t last_obs;	/* that previous copy's time: bounds where a
				 * move between two copies can have happened
				 */
	ktime_t live_at;	/* earliest time the SC is known to have
				 * written a round: 0 until first proved
				 */
};

/*
 * Zero the statistics, re-take base from a fresh read, and re-base the
 * sample deadline. Caller locks.
 */
static void meson_ddr_pub_take_base(struct meson_ddr_pub *pub)
{
	unsigned int i, f;

	for (i = 0; i < pub->nwords; i++) {
		struct pub_word *w = &pub->words[i];
		u32 v = readl(pub->regs + w->off);

		for (f = 0; f < PUB_FIELD_COUNT; f++) {
			struct pub_field *pf = &w->fields[f];
			u32 val = (v >> pf->shift) & PUB_FIELD_MASK;

			pf->base = val;
			pf->cur = val;
			pf->min = val;
			pf->max = val;
			pf->prev = val;
			pf->sum = 0;
			pf->changes = 0;
		}
	}
	pub->samples = 0;
	pub->start = ktime_get();
	pub->next = ktime_add_ms(pub->start, pub->period_ms);
}

/*
 * The rate word made safe before it drives a window or a render: a block
 * that has not stated one yet means the default, and garbage stays inside a
 * sane range.
 */
static u32 meson_ddr_pub_sc_rate_us(u32 v)
{
	if (!v)
		v = MESON_DDR_PUB_DEFAULT_PERIOD_MS * USEC_PER_MSEC;

	return clamp_t(u32, v, MESON_DDR_PUB_SC_MIN_PERIOD_US,
		       MESON_DDR_PUB_SC_MAX_PERIOD_US);
}

/*
 * Copy the SC block tear-free under its seqcount: odd seq = mid-update; the
 * data is valid between two equal even seq reads; seq == 0 means the SC has
 * not completed a single update. The retry is bounded in TIME -- a fixed
 * wait window, sleeping -- so a wedged SC cannot hang a debugfs read: the
 * read then fails -EBUSY (-ERESTARTSYS to a signal, where the copy is
 * interruptible) rather than showing a torn copy as current (that would
 * pair broken statistics with a fresh elapsed_ms). One pass is ~170
 * device-SRAM reads, so callers hold the sleeping sc_lock, never the
 * spinlock (probe's first validate runs with no reader yet attached). The
 * copy fills a scratch buffer and commits it to sc_live only when it
 * validates, so a failed copy never overwrites the kept copy; the gate
 * owns the liveness bookkeeping.
 */
static int meson_ddr_pub_sc_copy(struct meson_ddr_pub *pub, bool interruptible)
{
	struct pubmon_sc_data *d = pub->sc_scratch;
	void __iomem *sc = pub->sc;
	ktime_t deadline = ktime_add_ms(ktime_get(),
					MESON_DDR_PUB_SC_WAIT_MS);
	unsigned int i;
	u32 seq, seq2;

	for (;;) {
		/*
		 * Timestamp the pass before its first read: the successful
		 * pass's t_obs is when its view of the block was taken.
		 */
		d->t_obs = ktime_get_boottime();
		seq = readl(sc + SC_OFF_SEQ);
		if (!seq || (seq & 1))
			goto busy;
		d->samples = readl(sc + SC_OFF_SAMPLES);
		d->samples |= (u64)readl(sc + SC_OFF_SAMPLES_HI) << 32;
		d->period_us = meson_ddr_pub_sc_rate_us(readl(sc +
							      SC_OFF_PERIOD_US));
		d->sleeps = readl(sc + SC_OFF_SLEEP_COUNT);
		d->state = readl(sc + SC_OFF_STATE);
		d->isr_max_cycles = readl(sc + SC_OFF_ISR_MAX_CYCLES);
		d->ring_head = readl(sc + SC_OFF_RING_HEAD) % SC_RING_SLOTS;
		for (i = 0; i < MESON_DDR_PUB_SC_NFIELDS; i++) {
			unsigned long off = SC_OFF_FIELDS + i * SC_FIELD_SIZE;
			struct pubmon_sc_field *f = &d->fields[i];
			u32 a = readl(sc + off);
			u32 b = readl(sc + off + 4);

			f->base = FIELD_GET(GENMASK(15, 0), a);	/* base:cur */
			f->cur = FIELD_GET(GENMASK(31, 16), a);
			f->min = FIELD_GET(GENMASK(15, 0), b);	/* min:max */
			f->max = FIELD_GET(GENMASK(31, 16), b);
			f->changes = readl(sc + off + 8);
			f->sum = readl(sc + off + 16);
			f->sum |= (u64)readl(sc + off + 20) << 32;
		}
		for (i = 0; i < SC_RING_SLOTS; i++) {
			unsigned long off = SC_OFF_RING + i * SC_RING_SIZE;
			struct pubmon_sc_change *c = &d->ring[i];
			u32 a = readl(sc + off);
			u32 b = readl(sc + off + 4);

			c->word = FIELD_GET(GENMASK(7, 0), a);	/* word, field, pad */
			c->field = FIELD_GET(GENMASK(15, 8), a);
			c->old = FIELD_GET(GENMASK(15, 0), b);	/* old:new */
			c->new = FIELD_GET(GENMASK(31, 16), b);
			c->samples = readl(sc + off + 8);
		}
		/*
		 * The SC orders its data before the even seq (DMB); each
		 * readl() carries the reader-side barrier, so no rmb() is
		 * needed before the seq re-read below.
		 */
		if (readl(sc + SC_OFF_MAGIC) != MESON_DDR_PUB_SC_MAGIC)
			goto busy;	/* block lost/rewritten mid-pass */
		seq2 = readl(sc + SC_OFF_SEQ);
		if (seq == seq2) {
			d->seq = seq;
			*pub->sc_live = *d;
			pub->have_copy = true;
			return 0;
		}
busy:
		if (interruptible && signal_pending(current))
			return -ERESTARTSYS;
		if (!ktime_before(ktime_get(), deadline))
			return -EBUSY;
		usleep_range_state(50, 100,
				   interruptible ? TASK_INTERRUPTIBLE :
						   TASK_UNINTERRUPTIBLE);
	}
}

/*
 * Wait (bounded, sleeping) for the SC to act on a reset generation;
 * signal-interruptible where interruptible.
 */
static int meson_ddr_pub_sc_wait_gen(struct meson_ddr_pub *pub, u32 gen,
				     bool interruptible)
{
	ktime_t deadline = ktime_add_ms(ktime_get(),
					MESON_DDR_PUB_SC_WAIT_MS);

	while (readl(pub->sc + SC_OFF_RESET_ACK) != gen) {
		if (interruptible && signal_pending(current))
			return -ERESTARTSYS;
		if (!ktime_before(ktime_get(), deadline))
			return -ETIMEDOUT;
		usleep_range_state(50, 100,
				   interruptible ? TASK_INTERRUPTIBLE :
						   TASK_UNINTERRUPTIBLE);
	}

	return 0;
}

/*
 * Read-path gate, called under sc_lock after a fresh copy of the block. Two
 * reasons a consistent copy is still not current: a reset whose ack timed
 * out (-EBUSY until a later `reset` write sees its ack land and re-bases
 * `start` beside the zeroed samples -- a late ack of the old generation
 * never resumes reads on its own), and a dead SC (-ESTALE, never its last
 * data as current).
 *
 * Liveness follows observed movement ONLY -- no round counting, no
 * timestamp the driver set itself. live_at is the earliest time the SC is
 * known to have written a round: a copy whose seq differs from the
 * previous copy's, taken no more than W after that previous copy, proves a
 * write somewhere in between, so live_at moves up to the previous copy's
 * timestamp -- the time the previous pass took before its first read, not
 * the moment the gate noticed. W = 2 * published period_us + 100 ms +
 * 100 ms (the extra 100 ms covers one SC sub-tick: the block advances its
 * seq in pieces of the period, so movement can lag a full period boundary
 * without a wedge). The read passes while live_at is within W of now;
 * otherwise the wait is on nothing but the seq -- re-read about every
 * millisecond for at most W, one full copy once it moves: an SC that
 * resumes is caught by the pass itself, a silent block ends -ESTALE, a
 * signal ends -ERESTARTSYS. All timestamps are boot time: the SC samples
 * through suspend, CLOCK_MONOTONIC does not.
 */
/*
 * The gate's dead-SC exit. Without a log line a dead SC was silent: every
 * SC-mode read failed -ESTALE until reboot and nothing said why (landed
 * review S11). Rate-limited, since a reader may poll a dead block.
 */
static int meson_ddr_pub_sc_stale(struct meson_ddr_pub *pub)
{
	dev_warn_ratelimited(pub->dev,
			     "SC sampler block not seen moving (seq %u): SC stopped? SC-mode reads fail -ESTALE until it resumes\n",
			     pub->last_seq);
	return -ESTALE;
}

static int meson_ddr_pub_sc_gate(struct meson_ddr_pub *pub)
{
	const struct pubmon_sc_data *d;
	s64 window_us;
	bool bounded = false;
	ktime_t deadline = ktime_set(0, 0);
	ktime_t now;
	int ret;

	/*
	 * Latched by a timed-out `reset` write and cleared only by a later
	 * one that sees its ack: a late ack of the old generation must not
	 * silently resume reads whose `start` predates the zeroing.
	 */
	if (pub->reset_pending)
		return -EBUSY;

	for (;;) {
		/* d follows every commit the re-copies below make */
		d = pub->sc_live;
		now = ktime_get_boottime();
		window_us = 2LL * d->period_us +
			    200LL * USEC_PER_MSEC;

		if (d->seq != pub->last_seq &&
		    ktime_us_delta(now, pub->last_obs) <= window_us)
			pub->live_at = pub->last_obs;
		pub->last_seq = d->seq;
		pub->last_obs = d->t_obs;

		if (ktime_us_delta(now, pub->live_at) <= window_us)
			return 0;

		if (signal_pending(current))
			return -ERESTARTSYS;
		if (bounded && !ktime_after(deadline, now))
			return meson_ddr_pub_sc_stale(pub);
		if (!bounded) {
			deadline = ktime_add_us(now, window_us);
			bounded = true;
		}

		/*
		 * The quiet-gap wait: re-read ONLY the seq about every
		 * millisecond until it moves, then take one full copy so
		 * the movement check above runs on real data.
		 */
		while (readl(pub->sc + SC_OFF_SEQ) == d->seq) {
			if (signal_pending(current))
				return -ERESTARTSYS;
			if (!ktime_after(deadline, ktime_get_boottime()))
				return meson_ddr_pub_sc_stale(pub);
			usleep_range_state(1000, 1100, TASK_INTERRUPTIBLE);
		}

		ret = meson_ddr_pub_sc_copy(pub, true);
		if (ret)
			return ret;
	}
}

/* Validate magic/version and take the first consistent copy. Caller mapped. */
static int meson_ddr_pub_sc_validate(struct meson_ddr_pub *pub)
{
	u32 v = readl(pub->sc + SC_OFF_VERSION);

	if (readl(pub->sc + SC_OFF_MAGIC) != MESON_DDR_PUB_SC_MAGIC)
		return -ENODEV;
	if (FIELD_GET(GENMASK(15, 0), v) != MESON_DDR_PUB_SC_VERSION ||
	    FIELD_GET(GENMASK(31, 16), v) != MESON_DDR_PUB_SC_HDR_LEN)
		return -ENODEV;

	/*
	 * Probe context: uninterruptible on purpose -- a pending signal at
	 * modprobe must not steer the mode choice to the kernel sampler.
	 */
	return meson_ddr_pub_sc_copy(pub, false);
}

/* Drop the SC mapping and run the in-kernel sampler instead. */
static void meson_ddr_pub_sc_disable(struct meson_ddr_pub *pub,
				     struct device *dev)
{
	devm_kfree(dev, pub->sc_live);
	devm_kfree(dev, pub->sc_scratch);
	devm_iounmap(dev, pub->sc);
	pub->sc_live = NULL;
	pub->sc_scratch = NULL;
	pub->have_copy = false;
	pub->sc = NULL;
	pub->sc_mode = false;
}

/*
 * SC mode selection: an available `amlogic,sc-shmem` phandle on our node,
 * naming the SC's block with a valid magic and version, and a probe-time
 * reset the SC acknowledges -- the ack is also the liveness proof, so a
 * wedged SC never opens a mode whose reads would only ever fail. Anything
 * else (absent or disabled node -- e.g. the vendor chain --, bad block,
 * silent SC) leaves the in-kernel sampler. The node must be enabled only
 * where the SC has opened this KiB to the non-secure world: an NS read
 * before that is unsafe, so status is the gate, flipped by the U-Boot SCMI
 * fixup (open chain) or a test DTB.
 */
/*
 * Hibernation image creation is the point to refuse, not .freeze:
 * .freeze also runs for PMSG_QUIESCE (the boot kernel before restoring
 * an image), where a refusal would abort a restore and lose a
 * kernel-mode image. Refusing PM_HIBERNATION_PREPARE in SC mode stops
 * the image before anything is at stake: a restore may boot an SC
 * without the sampler, and the restored kernel would then read a KiB
 * nobody opened.
 */
static int meson_ddr_pub_hiber_notify(struct notifier_block *nb,
				      unsigned long action, void *unused)
{
	struct meson_ddr_pub *pub =
		container_of(nb, struct meson_ddr_pub, hiber_nb);

	if (action == PM_HIBERNATION_PREPARE) {
		dev_warn(pub->dev, "SC mode: hibernation refused, a restore may not have the SC sampler\n");
		return notifier_from_errno(-EBUSY);
	}

	return NOTIFY_DONE;
}

static void meson_ddr_pub_sc_select(struct meson_ddr_pub *pub, struct device *dev)
{
	struct device_node *np;
	struct resource res;
	void __iomem *sc;
	int ret;

	np = of_parse_phandle(dev->of_node, "amlogic,sc-shmem", 0);
	if (!np)
		return;

	if (!of_device_is_available(np) ||
	    !of_device_is_compatible(np, "amlogic,gxl-pubmon-sc-shmem"))
		goto out_put;

	if (of_address_to_resource(np, 0, &res))
		goto out_put;
	if (resource_size(&res) < MESON_DDR_PUB_SC_SIZE) {
		dev_info(dev, "SC shmem too small (%llx), using the in-kernel sampler\n",
			 (u64)resource_size(&res));
		goto out_put;
	}
	/*
	 * Map WITHOUT requesting: the whole SRAM is already claimed by the
	 * mmio-sram parent sram@c8000000, so devm_of_iomap()'s request would
	 * fail -EBUSY. Bare devm_ioremap is the rpm_master_stats.c way.
	 */
	sc = devm_ioremap(dev, res.start, resource_size(&res));
	if (!sc) {
		dev_info(dev, "SC shmem iomap failed, using the in-kernel sampler\n");
		goto out_put;
	}
	pub->sc = sc;

	pub->sc_live = devm_kzalloc(dev, sizeof(*pub->sc_live), GFP_KERNEL);
	pub->sc_scratch = devm_kzalloc(dev, sizeof(*pub->sc_scratch),
				       GFP_KERNEL);
	if (!pub->sc_live || !pub->sc_scratch) {
		meson_ddr_pub_sc_disable(pub, dev);
		goto out_put;
	}

	ret = meson_ddr_pub_sc_validate(pub);
	if (ret == -EBUSY)
		dev_info(dev, "SC block busy (seq never settled), using the in-kernel sampler\n");
	else if (ret)
		dev_info(dev, "no valid SC block (magic/version), using the in-kernel sampler\n");
	if (ret) {
		meson_ddr_pub_sc_disable(pub, dev);
		goto out_put;
	}
	/*
	 * Seed the liveness bookkeeping from this validated copy: the first
	 * gated read then measures against a real observation, not a zero.
	 */
	pub->last_seq = pub->sc_live->seq;
	pub->last_obs = pub->sc_live->t_obs;
	/*
	 * The SC's counters survive a module reload: start the generation
	 * above the one the block last ACKNOWLEDGED, so it can never equal
	 * an ack the SC has already given.
	 */
	pub->reset_gen = readl(sc + SC_OFF_RESET_ACK);
	/*
	 * Claim the default rate BEFORE the reset generation is written: a
	 * reloaded driver must never run even one interval on a previous
	 * instance's period.
	 */
	writel(MESON_DDR_PUB_DEFAULT_PERIOD_MS * USEC_PER_MSEC,
	       sc + SC_OFF_CTRL_PERIOD_US);
	/*
	 * Prove liveness now rather than at first read: one zeroing reset,
	 * accepted only with its ack, which also re-bases `start` beside the
	 * zeroed samples. No ack in the wait window means the SC is not
	 * running this block: fall back. Nothing needs undoing -- the
	 * generation was ack+1 and simply stands unacked.
	 */
	writel(++pub->reset_gen, sc + SC_OFF_CTRL_RESET_GEN);
	if (meson_ddr_pub_sc_wait_gen(pub, pub->reset_gen, false)) {
		dev_info(dev, "SC did not acknowledge its block, using the in-kernel sampler\n");
		meson_ddr_pub_sc_disable(pub, dev);
		goto out_put;
	}
	/*
	 * Liveness needs no anchor past the seed above: the block's seq
	 * survives the reset, and the gate proves movement from the first
	 * pair of copies.
	 */
	pub->start = ktime_get_boottime();
	pub->sc_mode = true;
	pub->hiber_nb.notifier_call = meson_ddr_pub_hiber_notify;
	register_pm_notifier(&pub->hiber_nb);

out_put:
	of_node_put(np);
}

static void meson_ddr_pub_sample(struct work_struct *work)
{
	struct meson_ddr_pub *pub =
		container_of(to_delayed_work(work), struct meson_ddr_pub, dwork);
	unsigned long flags, period, delay;
	unsigned int i, f;
	ktime_t now, next;

	spin_lock_irqsave(&pub->lock, flags);
	now = ktime_get();
	for (i = 0; i < pub->nwords; i++) {
		struct pub_word *w = &pub->words[i];
		u32 v = readl(pub->regs + w->off);

		for (f = 0; f < PUB_FIELD_COUNT; f++) {
			struct pub_field *pf = &w->fields[f];
			u32 val = (v >> pf->shift) & PUB_FIELD_MASK;

			if (val != pf->prev) {
				trace_field_change(w->name, pf->name, pf->prev,
						   val);
				pf->changes++;
				pf->prev = val;
			}
			pf->cur = val;
			if (val < pf->min)
				pf->min = val;
			if (val > pf->max)
				pf->max = val;
			pf->sum += val;
		}
	}
	pub->samples++;
	period = pub->period_ms;

	/*
	 * Schedule against the absolute deadline start + n*period, not
	 * relative to this sample: a relative re-arm plus the
	 * msecs-to-jiffies round-up loses a measurable slice of the sample
	 * rate. If the deadline is already past (the board was busy), re-base
	 * to now + period rather than burst-catch-up.
	 */
	next = ktime_add_ms(pub->next, period);
	if (!ktime_after(next, now))
		next = ktime_add_ms(now, period);
	pub->next = next;
	delay = usecs_to_jiffies(ktime_us_delta(next, now));
	spin_unlock_irqrestore(&pub->lock, flags);

	if (delay < 1)
		delay = 1;
	mod_delayed_work(system_power_efficient_wq, &pub->dwork, delay);
}

/*
 * SC mode: same line format, rendered from one consistent block copy -- only
 * when the gate calls that copy current (pending reset ack, dead-SC window).
 */
static int meson_ddr_pub_sc_stats_show(struct seq_file *m,
				       struct meson_ddr_pub *pub)
{
	struct pubmon_sc_data *d;
	unsigned long period_ms, flags;
	u64 elapsed_ms, samples, resumes;
	unsigned int i, f;
	int ret;

	ret = mutex_lock_interruptible(&pub->sc_lock);
	if (ret)
		return -ERESTARTSYS;
	ret = meson_ddr_pub_sc_copy(pub, true);
	if (!ret)
		ret = meson_ddr_pub_sc_gate(pub);
	if (ret) {
		mutex_unlock(&pub->sc_lock);
		return ret;
	}
	d = pub->sc_live;
	samples = d->samples;
	/*
	 * Boot time, like `start`: the SC samples through suspend, so its
	 * sample count keeps pace with CLOCK_BOOTTIME, not CLOCK_MONOTONIC.
	 */
	elapsed_ms = ktime_ms_delta(ktime_get_boottime(), pub->start);
	period_ms = DIV_ROUND_CLOSEST(d->period_us, USEC_PER_MSEC);
	/*
	 * resumes= is the kernel's count in both modes, so it means the same
	 * thing in both: every system resume, s2idle included.  The SC's
	 * sleep-path count is a different quantity and is not substituted for
	 * it (it is sleeps= in the resume file).
	 */
	spin_lock_irqsave(&pub->lock, flags);
	resumes = pub->resumes;
	spin_unlock_irqrestore(&pub->lock, flags);
	seq_printf(m, "samples=%llu elapsed_ms=%llu period_ms=%lu resumes=%llu source=sc\n",
		   samples, elapsed_ms, period_ms, resumes);
	for (i = 0; i < pub->nwords; i++) {
		struct pub_word *w = &pub->words[i];

		for (f = 0; f < PUB_FIELD_COUNT; f++) {
			const struct pubmon_sc_field *sf =
				&d->fields[i * PUB_FIELD_COUNT + f];
			u64 mean = 0, frac = 0, rem = 0;

			if (samples) {
				mean = div64_u64_rem(sf->sum, samples, &rem);
				frac = div64_u64(rem * 100, samples);
			}
			seq_printf(m,
				   "%s %s base=0x%03x cur=0x%03x min=0x%03x max=0x%03x mean=%llu.%02llu changes=%llu\n",
				   w->name, w->fields[f].name, sf->base,
				   sf->cur, sf->min, sf->max, mean, frac,
				   (u64)sf->changes);
		}
	}
	mutex_unlock(&pub->sc_lock);

	return 0;
}

/*
 * The block's change ring: the flight recorder of the last SC_RING_SLOTS
 * changes (the kernel-mode fallback traces every change instead).
 * Oldest first; slots no change has reached render as nothing.
 */
static int meson_ddr_pub_ring_show(struct seq_file *m, void *unused)
{
	struct meson_ddr_pub *pub = m->private;
	const struct pubmon_sc_data *d;
	unsigned int i, slot;
	bool stale = false;
	int ret;

	ret = mutex_lock_interruptible(&pub->sc_lock);
	if (ret)
		return -ERESTARTSYS;
	/*
	 * The ring is the exception to the stale rule: it exists for the
	 * last changes before the SC stopped, so nothing about the SC
	 * stopping fails the read. stale=1 -- rendering the kept copy --
	 * covers a block whose seq stayed odd (copy -EBUSY, a wedge
	 * mid-round), an SC not seen moving (gate -ESTALE), and a latched
	 * reset; only having never held a consistent copy fails.
	 */
	if (pub->reset_pending) {
		stale = true;
	} else {
		ret = meson_ddr_pub_sc_copy(pub, true);
		if (ret == -ERESTARTSYS) {
			mutex_unlock(&pub->sc_lock);
			return ret;
		}
		if (ret) {
			stale = true;	/* -EBUSY: seq stayed odd */
		} else {
			ret = meson_ddr_pub_sc_gate(pub);
			if (ret == -ERESTARTSYS) {
				mutex_unlock(&pub->sc_lock);
				return ret;
			}
			if (ret)
				stale = true;	/* -ESTALE: not seen moving */
		}
	}
	if (stale && !pub->have_copy) {
		mutex_unlock(&pub->sc_lock);
		return -EBUSY;
	}
	seq_printf(m, "stale=%d\n", stale);
	d = pub->sc_live;
	for (i = 0; i < SC_RING_SLOTS; i++) {
		const struct pubmon_sc_change *c;
		const struct pub_word *w;

		slot = (d->ring_head + i) % SC_RING_SLOTS;	/* oldest first */
		c = &d->ring[slot];
		if (c->old == c->new ||
		    c->word >= pub->nwords || c->field >= PUB_FIELD_COUNT)
			continue;
		w = &pub->words[c->word];
		seq_printf(m, "%s %s old=0x%03x new=0x%03x samples=%u\n",
			   w->name, w->fields[c->field].name, c->old, c->new,
			   c->samples);
	}
	mutex_unlock(&pub->sc_lock);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(meson_ddr_pub_ring);

static int meson_ddr_pub_stats_show(struct seq_file *m, void *unused)
{
	struct meson_ddr_pub *pub = m->private;
	unsigned long flags;
	u64 elapsed_ms;
	unsigned int i, f;

	if (pub->sc_mode)
		return meson_ddr_pub_sc_stats_show(m, pub);

	spin_lock_irqsave(&pub->lock, flags);
	elapsed_ms = ktime_ms_delta(ktime_get(), pub->start);
	seq_printf(m, "samples=%llu elapsed_ms=%llu period_ms=%lu resumes=%llu source=kernel\n",
		   pub->samples, elapsed_ms, pub->period_ms, pub->resumes);
	for (i = 0; i < pub->nwords; i++) {
		struct pub_word *w = &pub->words[i];

		for (f = 0; f < PUB_FIELD_COUNT; f++) {
			struct pub_field *pf = &w->fields[f];
			u64 mean = 0, frac = 0, rem = 0;

			if (pub->samples) {
				mean = div64_u64_rem(pf->sum, pub->samples,
						     &rem);
				frac = div64_u64(rem * 100, pub->samples);
			}
			seq_printf(m,
				   "%s %s base=0x%03x cur=0x%03x min=0x%03x max=0x%03x mean=%llu.%02llu changes=%llu\n",
				   w->name, pf->name, pf->base, pf->cur,
				   pf->min, pf->max, mean, frac, pf->changes);
		}
	}
	spin_unlock_irqrestore(&pub->lock, flags);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(meson_ddr_pub_stats);

static int meson_ddr_pub_resume_show(struct seq_file *m, void *unused)
{
	struct meson_ddr_pub *pub = m->private;
	unsigned long flags;
	unsigned int i;
	int ret;

	if (pub->sc_mode) {
		u32 sleeps, state, isr;

		/*
		 * The SC counts its own sleep-path entries, records which
		 * side of one it is on, and bounds its sampling ISR in
		 * cycles; all three from one consistent copy, printed after
		 * the lock is dropped.
		 */
		ret = mutex_lock_interruptible(&pub->sc_lock);
		if (ret)
			return -ERESTARTSYS;
		ret = meson_ddr_pub_sc_copy(pub, true);
		if (!ret)
			ret = meson_ddr_pub_sc_gate(pub);
		if (ret) {
			mutex_unlock(&pub->sc_lock);
			return ret;
		}
		sleeps = pub->sc_live->sleeps;
		state = pub->sc_live->state;
		isr = pub->sc_live->isr_max_cycles;
		mutex_unlock(&pub->sc_lock);
		if (isr == U32_MAX)
			/*
			 * No cycle counter on this SC build: the bound was
			 * never measured, which is not the same as zero.
			 */
			seq_printf(m, "sleeps=%u state=%u isr_max_cycles=n/a\n",
				   sleeps, state);
		else
			seq_printf(m, "sleeps=%u state=%u isr_max_cycles=%u\n",
				   sleeps, state, isr);

		return 0;
	}

	spin_lock_irqsave(&pub->lock, flags);
	if (!pub->resumes) {
		seq_puts(m, "none\n");
	} else {
		for (i = 0; i < pub->nwords; i++)
			seq_printf(m, "%s before=0x%08x after=0x%08x\n",
				   pub->words[i].name, pub->words[i].before,
				   pub->words[i].after);
	}
	spin_unlock_irqrestore(&pub->lock, flags);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(meson_ddr_pub_resume);

static int meson_ddr_pub_snapshot_show(struct seq_file *m, void *unused)
{
	struct meson_ddr_pub *pub = m->private;
	unsigned int i;

	for (i = 0; i < pub->nwords; i++)
		seq_printf(m, "%s 0x%08x\n", pub->words[i].name,
			   readl(pub->regs + pub->words[i].off));
	seq_printf(m, "PGCR6 0x%08x\n", readl(pub->regs + PUB_PGCR6));
	seq_printf(m, "PGSR0 0x%08x\n", readl(pub->regs + PUB_PGSR0));
	seq_printf(m, "w0e 0x%08x\n", readl(pub->regs + PUB_W0E));

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(meson_ddr_pub_snapshot);

/*
 * period_ms renders through seq_file on every read: in SC mode under the
 * same gate as every other SC read, so no read can serve a value older
 * than its own gate; in kernel mode the live field, un-gated.
 */
static int meson_ddr_pub_period_show(struct seq_file *m, void *unused)
{
	struct meson_ddr_pub *pub = m->private;
	unsigned long flags, period;
	int ret;

	if (pub->sc_mode) {
		ret = mutex_lock_interruptible(&pub->sc_lock);
		if (ret)
			return -ERESTARTSYS;
		ret = meson_ddr_pub_sc_copy(pub, true);
		if (!ret)
			ret = meson_ddr_pub_sc_gate(pub);
		if (!ret) {
			period = DIV_ROUND_CLOSEST(pub->sc_live->period_us,
						   USEC_PER_MSEC);
			seq_printf(m, "%lu\n", period);
		}
		mutex_unlock(&pub->sc_lock);

		return ret;
	}

	spin_lock_irqsave(&pub->lock, flags);
	period = pub->period_ms;
	spin_unlock_irqrestore(&pub->lock, flags);
	seq_printf(m, "%lu\n", period);

	return 0;
}

static int meson_ddr_pub_period_open(struct inode *inode, struct file *file)
{
	return single_open(file, meson_ddr_pub_period_show, inode->i_private);
}

static ssize_t period_ms_write(struct file *file, const char __user *ubuf,
			       size_t count, loff_t *ppos)
{
	struct seq_file *m = file->private_data;
	struct meson_ddr_pub *pub = m->private;
	unsigned long period, delay;
	unsigned long flags;
	ktime_t now;
	int ret;

	ret = kstrtoul_from_user(ubuf, count, 10, &period);
	if (ret)
		return ret;
	if (period < MESON_DDR_PUB_MIN_PERIOD_MS ||
	    period > MESON_DDR_PUB_MAX_PERIOD_MS)
		return -EINVAL;

	if (pub->sc_mode) {
		/*
		 * Requested rate for the SC; it picks the control word up on
		 * its next tick. Nothing to re-base here.
		 */
		ret = mutex_lock_interruptible(&pub->sc_lock);
		if (ret)
			return -ERESTARTSYS;
		writel(period * USEC_PER_MSEC, pub->sc + SC_OFF_CTRL_PERIOD_US);
		mutex_unlock(&pub->sc_lock);

		return count;
	}

	/*
	 * Changing the period re-bases the deadline: no burst from the
	 * old schedule.
	 */
	spin_lock_irqsave(&pub->lock, flags);
	pub->period_ms = period;
	now = ktime_get();
	pub->next = ktime_add_ms(now, period);
	delay = usecs_to_jiffies(ktime_us_delta(pub->next, now));
	spin_unlock_irqrestore(&pub->lock, flags);

	if (delay < 1)
		delay = 1;
	mod_delayed_work(system_power_efficient_wq, &pub->dwork, delay);

	return count;
}

static const struct file_operations meson_ddr_pub_period_fops = {
	.owner = THIS_MODULE,
	.open = meson_ddr_pub_period_open,
	.release = single_release,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = period_ms_write,
};

static ssize_t reset_write(struct file *file, const char __user *ubuf,
			   size_t count, loff_t *ppos)
{
	struct meson_ddr_pub *pub = file->private_data;
	unsigned long flags;
	int ret;

	if (pub->sc_mode) {
		/*
		 * The SC re-takes its base and zeroes the statistics and the
		 * change ring when this generation differs from reset_ack.
		 * Re-base `start` only once the SC has acted on it (ack ==
		 * gen): the stats elapsed_ms must restart beside the zeroed
		 * samples, not before them, so the samples /
		 * (elapsed_ms / period_ms) fidelity ratio stays meaningful
		 * across a per-cell reset. On -ETIMEDOUT the generation stays
		 * latched and `start` is untouched: reads fail -EBUSY until
		 * a later `reset` write is acknowledged -- the only place
		 * `start` ever re-bases.
		 */
		ret = mutex_lock_interruptible(&pub->sc_lock);
		if (ret)
			return -ERESTARTSYS;
		/* Above the ack the SC has given, never equal to it. */
		pub->reset_gen = readl(pub->sc + SC_OFF_RESET_ACK) + 1;
		writel(pub->reset_gen, pub->sc + SC_OFF_CTRL_RESET_GEN);
		ret = meson_ddr_pub_sc_wait_gen(pub, pub->reset_gen, true);
		if (ret) {
			/*
			 * Latch either way; only the timeout warns -- a
			 * signalled caller restarts and will see the latch.
			 */
			pub->reset_pending = true;
			if (ret == -ETIMEDOUT)
				dev_warn(pub->dev,
					 "reset gen %u not acknowledged: reads fail -EBUSY until a later acknowledged reset\n",
					 pub->reset_gen);
			mutex_unlock(&pub->sc_lock);
			return ret;
		}
		pub->reset_pending = false;
		pub->start = ktime_get_boottime();
		mutex_unlock(&pub->sc_lock);

		return count;
	}

	/* Any write: zero the statistics and re-take base. */
	spin_lock_irqsave(&pub->lock, flags);
	meson_ddr_pub_take_base(pub);
	spin_unlock_irqrestore(&pub->lock, flags);

	return count;
}

static const struct file_operations meson_ddr_pub_reset_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = reset_write,
};

static void meson_ddr_pub_debugfs_add(struct device *dev,
				      struct meson_ddr_pub *pub,
				      const char *name, umode_t mode,
				      const struct file_operations *fops)
{
	struct dentry *d;

	d = debugfs_create_file(name, mode, pub->dir, pub, fops);
	if (IS_ERR(d))
		dev_warn(dev, "debugfs %s unavailable (%pe), continuing without it\n",
			 name, d);
}

static int meson_ddr_pub_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct meson_ddr_pub *pub;

	pub = devm_kzalloc(dev, sizeof(*pub), GFP_KERNEL);
	if (!pub)
		return -ENOMEM;
	pub->dev = dev;

	pub->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pub->regs))
		return PTR_ERR(pub->regs);

	pub->words = devm_kmemdup(dev, meson_ddr_pub_words,
				  sizeof(meson_ddr_pub_words), GFP_KERNEL);
	if (!pub->words)
		return -ENOMEM;
	pub->nwords = ARRAY_SIZE(meson_ddr_pub_words);

	BUILD_BUG_ON(ARRAY_SIZE(meson_ddr_pub_words) * PUB_FIELD_COUNT !=
		     MESON_DDR_PUB_SC_NFIELDS);

	spin_lock_init(&pub->lock);
	mutex_init(&pub->sc_lock);
	pub->period_ms = MESON_DDR_PUB_DEFAULT_PERIOD_MS;
	INIT_DELAYED_WORK(&pub->dwork, meson_ddr_pub_sample);

	platform_set_drvdata(pdev, pub);

	meson_ddr_pub_sc_select(pub, dev);
	dev_info(dev, "pubmon source=%s\n", pub->sc_mode ? "sc" : "kernel");

	/*
	 * Take the base values and arm the sampler BEFORE any debugfs file
	 * exists: a reset or period_ms write on a freshly created file could
	 * otherwise race the unlocked base read below. SC mode arms nothing:
	 * the SC samples, and `start` was already set on its probe ack.
	 */
	if (!pub->sc_mode) {
		meson_ddr_pub_take_base(pub);
		mod_delayed_work(system_power_efficient_wq, &pub->dwork,
				 msecs_to_jiffies(pub->period_ms));
	}

	/*
	 * debugfs is optional: the sampler and the trace event must survive
	 * CONFIG_DEBUG_FS=n / debugfs=off, so failures here warn and let
	 * probe succeed.
	 */
	pub->dir = debugfs_create_dir("meson-ddr-pub", NULL);
	if (IS_ERR(pub->dir)) {
		dev_warn(dev, "debugfs dir unavailable (%pe), sampling continues without the stats files\n",
			 pub->dir);
		pub->dir = NULL;
	} else {
		meson_ddr_pub_debugfs_add(dev, pub, "stats", 0444,
					  &meson_ddr_pub_stats_fops);
		meson_ddr_pub_debugfs_add(dev, pub, "snapshot", 0444,
					  &meson_ddr_pub_snapshot_fops);
		meson_ddr_pub_debugfs_add(dev, pub, "resume", 0444,
					  &meson_ddr_pub_resume_fops);
		meson_ddr_pub_debugfs_add(dev, pub, "period_ms", 0644,
					  &meson_ddr_pub_period_fops);
		meson_ddr_pub_debugfs_add(dev, pub, "reset", 0222,
					  &meson_ddr_pub_reset_fops);
		if (pub->sc_mode)
			meson_ddr_pub_debugfs_add(dev, pub, "ring", 0444,
						  &meson_ddr_pub_ring_fops);
	}

	return 0;
}

static void meson_ddr_pub_remove(struct platform_device *pdev)
{
	struct meson_ddr_pub *pub = platform_get_drvdata(pdev);

	/*
	 * Tear the files down first: a period_ms write here would re-arm
	 * the work after the cancel.
	 */
	if (pub->sc_mode)
		unregister_pm_notifier(&pub->hiber_nb);
	debugfs_remove_recursive(pub->dir);
	cancel_delayed_work_sync(&pub->dwork);
	mutex_destroy(&pub->sc_lock);
}

/*
 * suspend: stop the sampler, then keep a pre-suspend copy of every sampled
 * word. resume: record those against the fresh reads, set each field's prev
 * to the fresh value so the jump across the suspend is not counted as a
 * change, count and trace the resume, and re-base the deadline from now.
 * The statistics are not reset.
 */
static int meson_ddr_pub_suspend(struct device *dev)
{
	struct meson_ddr_pub *pub = dev_get_drvdata(dev);
	unsigned long flags;
	unsigned int i;

	/*
	 * SC mode: the SC samples through the shallow sleep path and counts
	 * the sleep entries itself; nothing to stop here.
	 */
	if (pub->sc_mode)
		return 0;

	cancel_delayed_work_sync(&pub->dwork);

	spin_lock_irqsave(&pub->lock, flags);
	for (i = 0; i < pub->nwords; i++)
		pub->words[i].before = readl(pub->regs + pub->words[i].off);
	spin_unlock_irqrestore(&pub->lock, flags);

	return 0;
}

static int meson_ddr_pub_resume(struct device *dev)
{
	struct meson_ddr_pub *pub = dev_get_drvdata(dev);
	unsigned long flags, delay;
	unsigned int i, f;
	ktime_t now;

	/*
	 * SC mode: the SC samples through the sleep itself, so there is no
	 * sampler to restart -- but the resume is still counted HERE.  This
	 * callback runs for every system sleep, s2idle included; the SC's own
	 * count sees only the sleeps routed through it (PSCI SYSTEM_SUSPEND),
	 * so resumes= taken from it read 0 across s2idle where kernel mode read
	 * 1 (landed review L3).  The SC's count stays its own field, sleeps=
	 * in the resume file.
	 */
	if (pub->sc_mode) {
		u64 n;

		spin_lock_irqsave(&pub->lock, flags);
		n = ++pub->resumes;
		spin_unlock_irqrestore(&pub->lock, flags);
		trace_pub_resume(n);
		return 0;
	}

	spin_lock_irqsave(&pub->lock, flags);
	now = ktime_get();
	pub->resumes++;
	for (i = 0; i < pub->nwords; i++) {
		struct pub_word *w = &pub->words[i];
		u32 v = readl(pub->regs + w->off);

		w->after = v;
		for (f = 0; f < PUB_FIELD_COUNT; f++) {
			struct pub_field *pf = &w->fields[f];

			/*
			 * prev := the fresh value: no change counted across
			 * the suspend; the rest of the statistics stay.
			 */
			pf->prev = (v >> pf->shift) & PUB_FIELD_MASK;
		}
	}
	trace_pub_resume(pub->resumes);

	/*
	 * Re-base from now: the samples missed while asleep are not made up
	 * in a burst.
	 */
	pub->next = ktime_add_ms(now, pub->period_ms);
	delay = usecs_to_jiffies(ktime_us_delta(pub->next, now));
	spin_unlock_irqrestore(&pub->lock, flags);

	if (delay < 1)
		delay = 1;
	mod_delayed_work(system_power_efficient_wq, &pub->dwork, delay);

	return 0;
}

/*
 * System sleep and the hibernation transition callbacks: the sampler
 * stops for suspend, freeze and poweroff alike (a hibernation power-off
 * that kept sampling would sample an image's last pages), and the resume
 * path serves thaw and restore too. Refusing hibernation itself is not
 * this struct's job -- see meson_ddr_pub_hiber_notify.
 */
static const struct dev_pm_ops meson_ddr_pub_pm = {
	.suspend = pm_sleep_ptr(meson_ddr_pub_suspend),
	.resume = pm_sleep_ptr(meson_ddr_pub_resume),
	.freeze = pm_sleep_ptr(meson_ddr_pub_suspend),
	.thaw = pm_sleep_ptr(meson_ddr_pub_resume),
	.poweroff = pm_sleep_ptr(meson_ddr_pub_suspend),
	.restore = pm_sleep_ptr(meson_ddr_pub_resume),
};

static const struct of_device_id meson_ddr_pub_of_match[] = {
	{ .compatible = "amlogic,gxl-ddr-pub-mon" },
	{ }
};
MODULE_DEVICE_TABLE(of, meson_ddr_pub_of_match);

static struct platform_driver meson_ddr_pub_driver = {
	.probe = meson_ddr_pub_probe,
	.remove = meson_ddr_pub_remove,
	.driver = {
		.name = "meson-gxl-ddr-pub-mon",
		.of_match_table = meson_ddr_pub_of_match,
		.pm = pm_sleep_ptr(&meson_ddr_pub_pm),
		/*
		 * Probe validates the SC block with waits of up to
		 * MESON_DDR_PUB_SC_WAIT_MS; with the node okay and the SC
		 * unresponsive that blocked a built-in boot for about a
		 * second (landed review L4). Nothing depends on this driver.
		 */
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
};
module_platform_driver(meson_ddr_pub_driver);

MODULE_DESCRIPTION("Amlogic GXL DDR PHY (PUB) VT telemetry monitor");
MODULE_LICENSE("GPL");

#if IS_ENABLED(CONFIG_MESON_GXL_DDR_PUB_MON_KUNIT_TEST)
#include "meson-gxl-ddr-pub-mon-test.c"
#endif
