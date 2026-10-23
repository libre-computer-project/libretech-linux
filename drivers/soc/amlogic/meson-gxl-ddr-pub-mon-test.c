// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for the SC-mode reader of meson-gxl-ddr-pub-mon.c.
 *
 * Included at the end of that file (the functions under test are static)
 * when CONFIG_MESON_GXL_DDR_PUB_MON_KUNIT_TEST is set. The SC block -- the
 * KiB the AO Cortex-M3 publishes -- is built in kernel memory with the
 * layout the SC writes (SC_OFF_*, little-endian), and the reader runs
 * against it through pub->sc.
 *
 * Run: tools/testing/kunit/kunit.py run --kunitconfig=drivers/soc/amlogic \
 *          --arch=x86_64
 */

#include <kunit/test.h>

static u32 *pubmon_blk(struct meson_ddr_pub *pub)
{
	return (u32 __force *)pub->sc;
}

static void pubmon_w(struct meson_ddr_pub *pub, unsigned int off, u32 v)
{
	pubmon_blk(pub)[off / 4] = v;
}

/* A pub in SC mode over a valid, consistent block (seq 2, 10 ms rate). */
static struct meson_ddr_pub *pubmon_test_pub(struct kunit *test)
{
	struct meson_ddr_pub *pub;
	u32 *blk;

	pub = kunit_kzalloc(test, sizeof(*pub), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, pub);
	blk = kunit_kzalloc(test, MESON_DDR_PUB_SC_SIZE, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, blk);
	pub->sc_live = kunit_kzalloc(test, sizeof(*pub->sc_live), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, pub->sc_live);
	pub->sc_scratch = kunit_kzalloc(test, sizeof(*pub->sc_scratch),
					GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, pub->sc_scratch);

	pub->sc = (void __iomem __force *)blk;
	pub->sc_mode = true;
	mutex_init(&pub->sc_lock);

	pubmon_w(pub, SC_OFF_MAGIC, MESON_DDR_PUB_SC_MAGIC);
	pubmon_w(pub, SC_OFF_VERSION, (MESON_DDR_PUB_SC_HDR_LEN << 16) |
				      MESON_DDR_PUB_SC_VERSION);
	pubmon_w(pub, SC_OFF_SEQ, 2);
	pubmon_w(pub, SC_OFF_PERIOD_US, 10000);
	return pub;
}

static void pubmon_rate_clamp(struct kunit *test)
{
	/* 0 = not stated yet: the default, never a zero-width window */
	KUNIT_EXPECT_EQ(test, meson_ddr_pub_sc_rate_us(0),
			MESON_DDR_PUB_DEFAULT_PERIOD_MS * USEC_PER_MSEC);
	KUNIT_EXPECT_EQ(test, meson_ddr_pub_sc_rate_us(500),
			MESON_DDR_PUB_SC_MIN_PERIOD_US);
	KUNIT_EXPECT_EQ(test, meson_ddr_pub_sc_rate_us(20000000),
			MESON_DDR_PUB_SC_MAX_PERIOD_US);
	KUNIT_EXPECT_EQ(test, meson_ddr_pub_sc_rate_us(20000), 20000);
}

static void pubmon_copy_parses_the_block(struct kunit *test)
{
	struct meson_ddr_pub *pub = pubmon_test_pub(test);
	unsigned int f = SC_OFF_FIELDS + 3 * SC_FIELD_SIZE;
	unsigned int r = SC_OFF_RING + 2 * SC_RING_SIZE;
	const struct pubmon_sc_data *d;

	pubmon_w(pub, SC_OFF_SAMPLES, 5);
	pubmon_w(pub, SC_OFF_SAMPLES_HI, 1);
	pubmon_w(pub, SC_OFF_PERIOD_US, 0);	/* clamped to the default */
	pubmon_w(pub, SC_OFF_SLEEP_COUNT, 2);
	pubmon_w(pub, SC_OFF_STATE, 1);
	pubmon_w(pub, SC_OFF_ISR_MAX_CYCLES, 1234);
	pubmon_w(pub, SC_OFF_RING_HEAD, 11);	/* taken modulo the slots */
	pubmon_w(pub, f, (12 << 16) | 10);	/* cur:base */
	pubmon_w(pub, f + 4, (14 << 16) | 9);	/* max:min */
	pubmon_w(pub, f + 8, 3);		/* changes */
	pubmon_w(pub, f + 16, 1000);		/* sum low */
	pubmon_w(pub, f + 20, 2);		/* sum high */
	pubmon_w(pub, r, (1 << 8) | 4);		/* field 1, word 4 */
	pubmon_w(pub, r + 4, (9 << 16) | 7);	/* new:old */
	pubmon_w(pub, r + 8, 77);		/* samples at the change */

	KUNIT_ASSERT_EQ(test, meson_ddr_pub_sc_copy(pub, false), 0);
	d = pub->sc_live;

	KUNIT_EXPECT_TRUE(test, pub->have_copy);
	KUNIT_EXPECT_EQ(test, d->seq, 2);
	KUNIT_EXPECT_EQ(test, d->samples, 0x100000005ULL);
	KUNIT_EXPECT_EQ(test, d->period_us,
			MESON_DDR_PUB_DEFAULT_PERIOD_MS * USEC_PER_MSEC);
	KUNIT_EXPECT_EQ(test, d->sleeps, 2);
	KUNIT_EXPECT_EQ(test, d->state, 1);
	KUNIT_EXPECT_EQ(test, d->isr_max_cycles, 1234);
	KUNIT_EXPECT_EQ(test, d->ring_head, 3);
	KUNIT_EXPECT_EQ(test, d->fields[3].base, 10);
	KUNIT_EXPECT_EQ(test, d->fields[3].cur, 12);
	KUNIT_EXPECT_EQ(test, d->fields[3].min, 9);
	KUNIT_EXPECT_EQ(test, d->fields[3].max, 14);
	KUNIT_EXPECT_EQ(test, d->fields[3].changes, 3);
	KUNIT_EXPECT_EQ(test, d->fields[3].sum, (2ULL << 32) | 1000);
	KUNIT_EXPECT_EQ(test, d->ring[2].word, 4);
	KUNIT_EXPECT_EQ(test, d->ring[2].field, 1);
	KUNIT_EXPECT_EQ(test, d->ring[2].old, 7);
	KUNIT_EXPECT_EQ(test, d->ring[2].new, 9);
	KUNIT_EXPECT_EQ(test, d->ring[2].samples, 77);
}

/* A copy that cannot validate is -EBUSY and never replaces the kept copy. */
static void pubmon_copy_refusal_keeps_the_kept_copy(struct kunit *test,
						    unsigned int off, u32 v)
{
	struct meson_ddr_pub *pub = pubmon_test_pub(test);

	pubmon_w(pub, SC_OFF_SAMPLES, 41);
	KUNIT_ASSERT_EQ(test, meson_ddr_pub_sc_copy(pub, false), 0);

	pubmon_w(pub, SC_OFF_SAMPLES, 99);
	pubmon_w(pub, off, v);
	KUNIT_EXPECT_EQ(test, meson_ddr_pub_sc_copy(pub, false), -EBUSY);
	KUNIT_EXPECT_EQ(test, pub->sc_live->seq, 2);
	KUNIT_EXPECT_EQ(test, pub->sc_live->samples, 41);
}

static void pubmon_copy_odd_seq_is_busy(struct kunit *test)
{
	pubmon_copy_refusal_keeps_the_kept_copy(test, SC_OFF_SEQ, 3);
}

static void pubmon_copy_zero_seq_is_busy(struct kunit *test)
{
	pubmon_copy_refusal_keeps_the_kept_copy(test, SC_OFF_SEQ, 0);
}

static void pubmon_copy_lost_magic_is_busy(struct kunit *test)
{
	pubmon_copy_refusal_keeps_the_kept_copy(test, SC_OFF_MAGIC, 0);
}

static void pubmon_validate(struct kunit *test)
{
	struct meson_ddr_pub *pub = pubmon_test_pub(test);

	pubmon_w(pub, SC_OFF_MAGIC, 0);
	KUNIT_EXPECT_EQ(test, meson_ddr_pub_sc_validate(pub), -ENODEV);
	pubmon_w(pub, SC_OFF_MAGIC, MESON_DDR_PUB_SC_MAGIC);

	pubmon_w(pub, SC_OFF_VERSION, (MESON_DDR_PUB_SC_HDR_LEN << 16) | 2);
	KUNIT_EXPECT_EQ(test, meson_ddr_pub_sc_validate(pub), -ENODEV);
	pubmon_w(pub, SC_OFF_VERSION, (0x80 << 16) | MESON_DDR_PUB_SC_VERSION);
	KUNIT_EXPECT_EQ(test, meson_ddr_pub_sc_validate(pub), -ENODEV);
	KUNIT_EXPECT_FALSE(test, pub->have_copy);

	pubmon_w(pub, SC_OFF_VERSION, (MESON_DDR_PUB_SC_HDR_LEN << 16) |
				      MESON_DDR_PUB_SC_VERSION);
	KUNIT_EXPECT_EQ(test, meson_ddr_pub_sc_validate(pub), 0);
	KUNIT_EXPECT_TRUE(test, pub->have_copy);
}

static void pubmon_wait_gen(struct kunit *test)
{
	struct meson_ddr_pub *pub = pubmon_test_pub(test);

	pubmon_w(pub, SC_OFF_RESET_ACK, 7);
	KUNIT_EXPECT_EQ(test, meson_ddr_pub_sc_wait_gen(pub, 7, false), 0);
	/* an ack that never lands times out, bounded */
	KUNIT_EXPECT_EQ(test, meson_ddr_pub_sc_wait_gen(pub, 8, false),
			-ETIMEDOUT);
}

static void pubmon_gate_reset_pending_is_busy(struct kunit *test)
{
	struct meson_ddr_pub *pub = pubmon_test_pub(test);

	KUNIT_ASSERT_EQ(test, meson_ddr_pub_sc_copy(pub, false), 0);
	pub->reset_pending = true;
	pub->live_at = ktime_get_boottime();
	KUNIT_EXPECT_EQ(test, meson_ddr_pub_sc_gate(pub), -EBUSY);
}

/*
 * L3: every system resume -- s2idle included, which never reaches the SC --
 * is counted by the kernel in SC mode too, and the SC's own sleep count is
 * left alone.
 */
static void pubmon_sc_resume_is_counted(struct kunit *test)
{
	struct meson_ddr_pub *pub = pubmon_test_pub(test);
	struct device *dev;

	dev = kunit_kzalloc(test, sizeof(*dev), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dev);
	spin_lock_init(&pub->lock);
	dev_set_drvdata(dev, pub);

	KUNIT_EXPECT_EQ(test, meson_ddr_pub_resume(dev), 0);
	KUNIT_EXPECT_EQ(test, meson_ddr_pub_resume(dev), 0);
	KUNIT_EXPECT_EQ(test, pub->resumes, 2ULL);
	KUNIT_EXPECT_EQ(test, pubmon_blk(pub)[SC_OFF_SLEEP_COUNT / 4], 0U);
}

/*
 * L3: the SC-mode stats header carries the kernel's resumes=, not the SC's
 * sleep-path count -- here the two disagree (3 vs 7), as they do after an
 * s2idle cycle.
 */
static void pubmon_sc_stats_resumes_is_the_kernel_count(struct kunit *test)
{
	struct meson_ddr_pub *pub = pubmon_test_pub(test);
	struct seq_file *m;

	m = kunit_kzalloc(test, sizeof(*m), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, m);
	m->size = 512;
	m->buf = kunit_kzalloc(test, m->size, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, m->buf);
	spin_lock_init(&pub->lock);
	pub->resumes = 3;
	pubmon_w(pub, SC_OFF_SLEEP_COUNT, 7);
	pub->start = ktime_get_boottime();

	/* make the gate pass: the block was seen live just now */
	KUNIT_ASSERT_EQ(test, meson_ddr_pub_sc_copy(pub, false), 0);
	pub->last_seq = pub->sc_live->seq;
	pub->last_obs = pub->sc_live->t_obs;
	pub->live_at = ktime_get_boottime();

	KUNIT_ASSERT_EQ(test, meson_ddr_pub_sc_stats_show(m, pub), 0);
	m->buf[min_t(size_t, m->count, m->size - 1)] = '\0';
	KUNIT_EXPECT_NOT_NULL(test, strstr(m->buf, " resumes=3 source=sc"));
	KUNIT_EXPECT_NULL(test, strstr(m->buf, " resumes=7 "));
}

static void pubmon_gate_recently_live_passes(struct kunit *test)
{
	struct meson_ddr_pub *pub = pubmon_test_pub(test);

	KUNIT_ASSERT_EQ(test, meson_ddr_pub_sc_copy(pub, false), 0);
	pub->last_seq = pub->sc_live->seq;
	pub->last_obs = pub->sc_live->t_obs;
	pub->live_at = ktime_get_boottime();
	KUNIT_EXPECT_EQ(test, meson_ddr_pub_sc_gate(pub), 0);
}

/* A seq that moved since the previous copy, inside the window, proves a
 * write and dates liveness to the previous copy's timestamp. */
static void pubmon_gate_movement_proves_liveness(struct kunit *test)
{
	struct meson_ddr_pub *pub = pubmon_test_pub(test);
	ktime_t prev = ktime_sub_ms(ktime_get_boottime(), 10);

	pub->last_seq = 0;		/* the previous copy saw seq 0 */
	pub->last_obs = prev;
	pub->live_at = 0;		/* never proved live */
	KUNIT_ASSERT_EQ(test, meson_ddr_pub_sc_copy(pub, false), 0); /* seq 2 */

	KUNIT_EXPECT_EQ(test, meson_ddr_pub_sc_gate(pub), 0);
	KUNIT_EXPECT_EQ(test, ktime_to_ns(pub->live_at), ktime_to_ns(prev));
	KUNIT_EXPECT_EQ(test, pub->last_seq, 2);
}

/* A block whose seq never moves is a dead SC: -ESTALE, bounded by the
 * window (2 * period + 200 ms), never its last data as current. */
static void pubmon_gate_silent_block_is_stale(struct kunit *test)
{
	struct meson_ddr_pub *pub = pubmon_test_pub(test);
	ktime_t t0;

	pubmon_w(pub, SC_OFF_PERIOD_US, 1000);	/* window 202 ms */
	KUNIT_ASSERT_EQ(test, meson_ddr_pub_sc_copy(pub, false), 0);
	pub->last_seq = pub->sc_live->seq;
	pub->last_obs = pub->sc_live->t_obs;
	pub->live_at = 0;

	t0 = ktime_get_boottime();
	KUNIT_EXPECT_EQ(test, meson_ddr_pub_sc_gate(pub), -ESTALE);
	KUNIT_EXPECT_LT(test, ktime_ms_delta(ktime_get_boottime(), t0), 1000);
}

static struct kunit_case meson_ddr_pub_sc_cases[] = {
	KUNIT_CASE(pubmon_rate_clamp),
	KUNIT_CASE(pubmon_copy_parses_the_block),
	KUNIT_CASE(pubmon_copy_odd_seq_is_busy),
	KUNIT_CASE(pubmon_copy_zero_seq_is_busy),
	KUNIT_CASE(pubmon_copy_lost_magic_is_busy),
	KUNIT_CASE(pubmon_validate),
	KUNIT_CASE(pubmon_wait_gen),
	KUNIT_CASE(pubmon_gate_reset_pending_is_busy),
	KUNIT_CASE(pubmon_gate_recently_live_passes),
	KUNIT_CASE(pubmon_gate_movement_proves_liveness),
	KUNIT_CASE(pubmon_gate_silent_block_is_stale),
	KUNIT_CASE(pubmon_sc_resume_is_counted),
	KUNIT_CASE(pubmon_sc_stats_resumes_is_the_kernel_count),
	{}
};

static struct kunit_suite meson_ddr_pub_sc_suite = {
	.name = "meson-gxl-ddr-pub-mon-sc",
	.test_cases = meson_ddr_pub_sc_cases,
};
kunit_test_suite(meson_ddr_pub_sc_suite);
