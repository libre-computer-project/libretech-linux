/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Continuous health test for the RK3288/RK3328/RK3399 crypto-block TRNG.
 *
 * The block has no "entropy source healthy" status bit.  RK_CRYPTO_TRNG_START
 * self-clears even with the ring oscillator switched off, in which case the
 * output window reads back all zeros -- a dead source is indistinguishable
 * from a good one at the register level.  That is exactly how e1dc9f0687ec0
 * (reverted in a566a21d81827) credited 32 constant zero bytes per read into
 * the kernel entropy pool.  So the driver must judge the *data*.
 *
 * This is deliberately a catastrophic-failure detector, not an entropy
 * estimator: it is the SP 800-90B repetition-count idea applied at 32-bit word
 * and 256-bit block granularity, which is the granularity the hardware hands
 * us.  It cannot see a source that is merely biased or short of full entropy,
 * and it cannot see a stuck source whose pattern repeats with a period longer
 * than RK_CRYPTO_TRNG_HISTORY -- read the DETECTION LIMITS paragraph above
 * rk_crypto_trng_health_check() before relying on it for anything.  See the
 * RK_CRYPTO_TRNG_QUALITY comment in rk3288_crypto.c for why the entropy credit
 * is set accordingly.
 *
 * The file is intentionally free-standing -- no linux/ includes, and nothing
 * but u32, size_t, bool and memcmp -- so that the predicate the driver ships
 * is the same text a host-compiled unit test exercises, rather than a copy of
 * it that can drift.  The includer provides those four names.
 * Host test: rk-trng-health.c, in our internal test suite.
 */

#ifndef RK3288_CRYPTO_TRNG_HEALTH_H
#define RK3288_CRYPTO_TRNG_HEALTH_H

/* One trigger produces 256 bits. */
#define RK_CRYPTO_TRNG_MAX_WORD		8
#define RK_CRYPTO_TRNG_MAX_BYTE		(RK_CRYPTO_TRNG_MAX_WORD * 4)

/*
 * How many previous blocks the repetition test remembers.  One block only
 * detects a source stuck on a single value: a period-2 alternation
 * A, B, A, B, ... passes every read forever, because the immediately
 * previous block is always the OTHER pattern.  A ring of N detects every
 * repetition of period <= N.
 *
 * 4 costs 168 bytes of per-device state on arm64, up from 40 for the
 * single-block version: prev[4][8] is 128 bytes, prev_words[4] is 32, next is
 * 4, and the struct pads to 168.  (65c8b55e339fa said 144, which was the size
 * of nothing -- it counted neither the old struct nor the new one.  The figure
 * is now MEASURED by case 12 of the host test rather than asserted here, so it
 * cannot drift again.)  It is not a bound anyone should lean on -- see the
 * DETECTION LIMITS note above rk_crypto_trng_health_check() for what still
 * gets through.
 */
#define RK_CRYPTO_TRNG_HISTORY		4

/* rk_crypto_trng_health_check() verdicts. */
#define RK_TRNG_HEALTH_OK		0
#define RK_TRNG_HEALTH_ALL_ZERO		1	/* the e1dc9f0687ec0 signature */
#define RK_TRNG_HEALTH_STUCK_WORD	2	/* every word identical */
#define RK_TRNG_HEALTH_REPEAT_BLOCK	3	/* repeats a recent block */
#define RK_TRNG_HEALTH_BAD_LENGTH	4	/* @words outside 1..MAX_WORD */

/**
 * struct rk_crypto_trng_health - continuous health-test state
 * @prev:	ring of the last RK_CRYPTO_TRNG_HISTORY blocks handed to
 *		rk_crypto_trng_health_check()
 * @prev_words:	how many words of each @prev entry are valid; 0 = unused slot
 * @next:	ring slot the next block will be written to
 *
 * Serialised by the hwrng core, which takes reading_mutex around every
 * ->read() from both hwrng_fillfn() and rng_dev_read().
 */
struct rk_crypto_trng_health {
	u32 prev[RK_CRYPTO_TRNG_HISTORY][RK_CRYPTO_TRNG_MAX_WORD];
	size_t prev_words[RK_CRYPTO_TRNG_HISTORY];
	unsigned int next;
};

/**
 * rk_crypto_trng_health_check() - judge one TRNG block, and remember it
 * @h:		per-device health state, zero-initialised
 * @data:	the block just read out of TRNG_DOUT_0..7
 * @words:	number of valid words in @data, 1..RK_CRYPTO_TRNG_MAX_WORD
 *
 * Returns RK_TRNG_HEALTH_OK, or the reason the block was rejected.
 *
 * @data is recorded in the ring whether it passed or failed, so a repeating
 * source keeps failing rather than alternating pass/fail.
 *
 * DETECTION LIMITS -- what this does NOT catch.  It rejects, exactly:
 * an all-zero block, a block whose words are all equal, and a block equal to
 * one of the last RK_CRYPTO_TRNG_HISTORY blocks.  That is every repetition of
 * period <= RK_CRYPTO_TRNG_HISTORY and nothing else.  A stuck-but-varying
 * source with a longer period, a free-running counter, and an LFSR all pass
 * 100% of reads forever -- an LFSR of any useful width never repeats a
 * 256-bit block at all, so no finite ring can see it.  Detecting those needs
 * a structural test this predicate does not attempt.  Nor can it see a source
 * that is merely biased or short of full entropy.  It is a catastrophic-
 * failure detector, and the RK_CRYPTO_TRNG_QUALITY comment in rk3288_crypto.c
 * is written to assert no more than that.
 *
 * A @words outside 1..RK_CRYPTO_TRNG_MAX_WORD is RK_TRNG_HEALTH_BAD_LENGTH,
 * not OK: a health test asked a question it cannot answer must fail closed.
 * Nothing in the driver can produce one today -- @words is
 * DIV_ROUND_UP(min(max, MAX_BYTE), 4) -- so this is the polarity of a latent
 * case, not a live bug.  The block is not recorded in the ring: there is no
 * well-defined block to record.
 */
static inline int rk_crypto_trng_health_check(struct rk_crypto_trng_health *h,
					      const u32 *data, size_t words)
{
	size_t i;
	unsigned int j;
	int verdict = RK_TRNG_HEALTH_OK;
	bool all_zero = true, stuck = true;

	if (words == 0 || words > RK_CRYPTO_TRNG_MAX_WORD)
		return RK_TRNG_HEALTH_BAD_LENGTH;

	for (i = 0; i < words; i++) {
		if (data[i])
			all_zero = false;
		if (data[i] != data[0])
			stuck = false;
	}

	if (all_zero) {
		verdict = RK_TRNG_HEALTH_ALL_ZERO;
	} else if (stuck && words > 1) {
		/*
		 * A single word cannot be "stuck" on the evidence of itself,
		 * so this only means anything for a real 2..8 word block.
		 */
		verdict = RK_TRNG_HEALTH_STUCK_WORD;
	} else {
		for (j = 0; j < RK_CRYPTO_TRNG_HISTORY; j++) {
			if (h->prev_words[j] == words &&
			    !memcmp(h->prev[j], data, words * sizeof(u32))) {
				verdict = RK_TRNG_HEALTH_REPEAT_BLOCK;
				break;
			}
		}
	}

	for (i = 0; i < words; i++)
		h->prev[h->next][i] = data[i];
	h->prev_words[h->next] = words;
	h->next = (h->next + 1) % RK_CRYPTO_TRNG_HISTORY;

	return verdict;
}

#endif /* RK3288_CRYPTO_TRNG_HEALTH_H */
