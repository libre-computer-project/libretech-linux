/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __RK3288_CRYPTO_H__
#define __RK3288_CRYPTO_H__

#include <crypto/aes.h>
#include <crypto/engine.h>
#include <crypto/internal/des.h>
#include <crypto/internal/hash.h>
#include <crypto/internal/skcipher.h>
#include <crypto/md5.h>
#include <crypto/sha1.h>
#include <crypto/sha2.h>
#include <linux/dma-mapping.h>
#include <linux/hw_random.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/scatterlist.h>
#include <linux/string.h>
#include <linux/types.h>

#include "rk3288_crypto_trng_health.h"

#define _SBF(v, f)			((v) << (f))

/* Crypto control registers*/
#define RK_CRYPTO_INTSTS		0x0000
#define RK_CRYPTO_PKA_DONE_INT		BIT(5)
#define RK_CRYPTO_HASH_DONE_INT		BIT(4)
#define RK_CRYPTO_HRDMA_ERR_INT		BIT(3)
#define RK_CRYPTO_HRDMA_DONE_INT	BIT(2)
#define RK_CRYPTO_BCDMA_ERR_INT		BIT(1)
#define RK_CRYPTO_BCDMA_DONE_INT	BIT(0)

#define RK_CRYPTO_INTENA		0x0004
#define RK_CRYPTO_PKA_DONE_ENA		BIT(5)
#define RK_CRYPTO_HASH_DONE_ENA		BIT(4)
#define RK_CRYPTO_HRDMA_ERR_ENA		BIT(3)
#define RK_CRYPTO_HRDMA_DONE_ENA	BIT(2)
#define RK_CRYPTO_BCDMA_ERR_ENA		BIT(1)
#define RK_CRYPTO_BCDMA_DONE_ENA	BIT(0)

#define RK_CRYPTO_CTRL			0x0008
/*
 * 0xFFFF is a plain int, so _SBF()'s `(v) << (f)` shifts a set bit into the
 * sign position -- undefined behaviour, and in practice an int whose value is
 * negative.  That stays invisible while the macro has no users, which is again
 * the case here, but OR it with a BIT() (unsigned long) and the negative int is
 * sign-extended to 0xFFFF_FFFF_FFFF_0000 before CRYPTO_WRITE() truncates it
 * back to the intended 32-bit word -- the register is written CORRECTLY and the
 * only symptom is a -Woverflow diagnostic.  Keep the literal unsigned: same
 * 0xFFFF0000 bit pattern, no UB, no sign extension, no warning, and the next
 * user of the macro does not have to rediscover this.
 */
#define RK_CRYPTO_WRITE_MASK		_SBF(0xFFFFU, 16)
#define RK_CRYPTO_TRNG_FLUSH		BIT(9)
#define RK_CRYPTO_TRNG_START		BIT(8)
#define RK_CRYPTO_PKA_FLUSH		BIT(7)
#define RK_CRYPTO_HASH_FLUSH		BIT(6)
#define RK_CRYPTO_BLOCK_FLUSH		BIT(5)
#define RK_CRYPTO_PKA_START		BIT(4)
#define RK_CRYPTO_HASH_START		BIT(3)
#define RK_CRYPTO_BLOCK_START		BIT(2)
#define RK_CRYPTO_TDES_START		BIT(1)
#define RK_CRYPTO_AES_START		BIT(0)

#define RK_CRYPTO_CONF			0x000c
/* HASH Receive DMA Address Mode:   fix | increment */
#define RK_CRYPTO_HR_ADDR_MODE		BIT(8)
/* Block Transmit DMA Address Mode: fix | increment */
#define RK_CRYPTO_BT_ADDR_MODE		BIT(7)
/* Block Receive DMA Address Mode:  fix | increment */
#define RK_CRYPTO_BR_ADDR_MODE		BIT(6)
#define RK_CRYPTO_BYTESWAP_HRFIFO	BIT(5)
#define RK_CRYPTO_BYTESWAP_BTFIFO	BIT(4)
#define RK_CRYPTO_BYTESWAP_BRFIFO	BIT(3)
/* AES = 0 OR DES = 1 */
#define RK_CRYPTO_DESSEL				BIT(2)
#define RK_CYYPTO_HASHINSEL_INDEPENDENT_SOURCE		_SBF(0x00, 0)
#define RK_CYYPTO_HASHINSEL_BLOCK_CIPHER_INPUT		_SBF(0x01, 0)
#define RK_CYYPTO_HASHINSEL_BLOCK_CIPHER_OUTPUT		_SBF(0x02, 0)

/* Block Receiving DMA Start Address Register */
#define RK_CRYPTO_BRDMAS		0x0010
/* Block Transmitting DMA Start Address Register */
#define RK_CRYPTO_BTDMAS		0x0014
/* Block Receiving DMA Length Register */
#define RK_CRYPTO_BRDMAL		0x0018
/* Hash Receiving DMA Start Address Register */
#define RK_CRYPTO_HRDMAS		0x001c
/* Hash Receiving DMA Length Register */
#define RK_CRYPTO_HRDMAL		0x0020

/* AES registers */
#define RK_CRYPTO_AES_CTRL			  0x0080
#define RK_CRYPTO_AES_BYTESWAP_CNT	BIT(11)
#define RK_CRYPTO_AES_BYTESWAP_KEY	BIT(10)
#define RK_CRYPTO_AES_BYTESWAP_IV	BIT(9)
#define RK_CRYPTO_AES_BYTESWAP_DO	BIT(8)
#define RK_CRYPTO_AES_BYTESWAP_DI	BIT(7)
#define RK_CRYPTO_AES_KEY_CHANGE	BIT(6)
#define RK_CRYPTO_AES_ECB_MODE		_SBF(0x00, 4)
#define RK_CRYPTO_AES_CBC_MODE		_SBF(0x01, 4)
#define RK_CRYPTO_AES_CTR_MODE		_SBF(0x02, 4)
#define RK_CRYPTO_AES_128BIT_key	_SBF(0x00, 2)
#define RK_CRYPTO_AES_192BIT_key	_SBF(0x01, 2)
#define RK_CRYPTO_AES_256BIT_key	_SBF(0x02, 2)
/* Slave = 0 / fifo = 1 */
#define RK_CRYPTO_AES_FIFO_MODE		BIT(1)
/* Encryption = 0 , Decryption = 1 */
#define RK_CRYPTO_AES_DEC		BIT(0)

#define RK_CRYPTO_AES_STS		0x0084
#define RK_CRYPTO_AES_DONE		BIT(0)

/* AES Input Data 0-3 Register */
#define RK_CRYPTO_AES_DIN_0		0x0088
#define RK_CRYPTO_AES_DIN_1		0x008c
#define RK_CRYPTO_AES_DIN_2		0x0090
#define RK_CRYPTO_AES_DIN_3		0x0094

/* AES output Data 0-3 Register */
#define RK_CRYPTO_AES_DOUT_0		0x0098
#define RK_CRYPTO_AES_DOUT_1		0x009c
#define RK_CRYPTO_AES_DOUT_2		0x00a0
#define RK_CRYPTO_AES_DOUT_3		0x00a4

/* AES IV Data 0-3 Register */
#define RK_CRYPTO_AES_IV_0		0x00a8
#define RK_CRYPTO_AES_IV_1		0x00ac
#define RK_CRYPTO_AES_IV_2		0x00b0
#define RK_CRYPTO_AES_IV_3		0x00b4

/* AES Key Data 0-3 Register */
#define RK_CRYPTO_AES_KEY_0		0x00b8
#define RK_CRYPTO_AES_KEY_1		0x00bc
#define RK_CRYPTO_AES_KEY_2		0x00c0
#define RK_CRYPTO_AES_KEY_3		0x00c4
#define RK_CRYPTO_AES_KEY_4		0x00c8
#define RK_CRYPTO_AES_KEY_5		0x00cc
#define RK_CRYPTO_AES_KEY_6		0x00d0
#define RK_CRYPTO_AES_KEY_7		0x00d4

/* des/tdes */
#define RK_CRYPTO_TDES_CTRL		0x0100
#define RK_CRYPTO_TDES_BYTESWAP_KEY	BIT(8)
#define RK_CRYPTO_TDES_BYTESWAP_IV	BIT(7)
#define RK_CRYPTO_TDES_BYTESWAP_DO	BIT(6)
#define RK_CRYPTO_TDES_BYTESWAP_DI	BIT(5)
/* 0: ECB, 1: CBC */
#define RK_CRYPTO_TDES_CHAINMODE_CBC	BIT(4)
/* TDES Key Mode, 0 : EDE, 1 : EEE */
#define RK_CRYPTO_TDES_EEE		BIT(3)
/* 0: DES, 1:TDES */
#define RK_CRYPTO_TDES_SELECT		BIT(2)
/* 0: Slave, 1:Fifo */
#define RK_CRYPTO_TDES_FIFO_MODE	BIT(1)
/* Encryption = 0 , Decryption = 1 */
#define RK_CRYPTO_TDES_DEC		BIT(0)

#define RK_CRYPTO_TDES_STS		0x0104
#define RK_CRYPTO_TDES_DONE		BIT(0)

#define RK_CRYPTO_TDES_DIN_0		0x0108
#define RK_CRYPTO_TDES_DIN_1		0x010c
#define RK_CRYPTO_TDES_DOUT_0		0x0110
#define RK_CRYPTO_TDES_DOUT_1		0x0114
#define RK_CRYPTO_TDES_IV_0		0x0118
#define RK_CRYPTO_TDES_IV_1		0x011c
#define RK_CRYPTO_TDES_KEY1_0		0x0120
#define RK_CRYPTO_TDES_KEY1_1		0x0124
#define RK_CRYPTO_TDES_KEY2_0		0x0128
#define RK_CRYPTO_TDES_KEY2_1		0x012c
#define RK_CRYPTO_TDES_KEY3_0		0x0130
#define RK_CRYPTO_TDES_KEY3_1		0x0134

/* HASH */
#define RK_CRYPTO_HASH_CTRL		0x0180
#define RK_CRYPTO_HASH_SWAP_DO		BIT(3)
#define RK_CRYPTO_HASH_SWAP_DI		BIT(2)
#define RK_CRYPTO_HASH_SHA1		_SBF(0x00, 0)
#define RK_CRYPTO_HASH_MD5		_SBF(0x01, 0)
#define RK_CRYPTO_HASH_SHA256		_SBF(0x02, 0)
#define RK_CRYPTO_HASH_PRNG		_SBF(0x03, 0)

#define RK_CRYPTO_HASH_STS		0x0184
#define RK_CRYPTO_HASH_DONE		BIT(0)

#define RK_CRYPTO_HASH_MSG_LEN		0x0188
#define RK_CRYPTO_HASH_DOUT_0		0x018c
#define RK_CRYPTO_HASH_DOUT_1		0x0190
#define RK_CRYPTO_HASH_DOUT_2		0x0194
#define RK_CRYPTO_HASH_DOUT_3		0x0198
#define RK_CRYPTO_HASH_DOUT_4		0x019c
#define RK_CRYPTO_HASH_DOUT_5		0x01a0
#define RK_CRYPTO_HASH_DOUT_6		0x01a4
#define RK_CRYPTO_HASH_DOUT_7		0x01a8

/*
 * TRNG registers.
 *
 * These are NOT the 0x01b0..0x01cc block that e1dc9f0687ec0 ("crypto:
 * rockchip: add TRNG hardware random number generator", reverted in
 * a566a21d81827) inferred by adjacency to RK_CRYPTO_HASH_DOUT_7 and that
 * measured 32 zero bytes on every trigger on a ROC-RK3399-PC.  0x01b0 is
 * simply not the TRNG.
 *
 * The real map comes from Rockchip's own U-Boot driver drivers/rng/
 * rockchip_rng.c (Copyright (c) 2020 Fuzhou Rockchip Electronics), whose
 * of_match_table binds "rockchip,rk3399-crypto" -- the exact compatible of
 * crypto0/crypto1 in rk3399-base.dtsi -- to rk_cryptov1_soc_data:
 *
 *	#define CRYPTO_V1_TRNG_CTRL		0x0200
 *	#define CRYPTO_V1_OSC_ENABLE		BIT(16)
 *	#define CRYPTO_V1_TRNG_SAMPLE_PERIOD(x)	(x)
 *	#define CRYPTO_V1_TRNG_DOUT_0		0x0204
 *
 * Two independent defects therefore each explain the all-zero reads: the
 * output window was 0x54 bytes low, and the ring oscillator was never
 * enabled.  Both are fixed together because either alone still yields zeros.
 * The DT reg window is 0x4000, so 0x0200..0x0220 is inside the ioremap.
 */
#define RK_CRYPTO_TRNG_CTRL		0x0200
#define RK_CRYPTO_TRNG_OSC_ENABLE	BIT(16)
#define RK_CRYPTO_TRNG_SAMPLE_PERIOD(x)	(x)

#define RK_CRYPTO_TRNG_DOUT_0		0x0204

#define CRYPTO_READ(dev, offset)		  \
		readl_relaxed(((dev)->reg + (offset)))
#define CRYPTO_WRITE(dev, offset, val)	  \
		writel_relaxed((val), ((dev)->reg + (offset)))

#define RK_MAX_CLKS 4

/*
 * struct rockchip_ip - struct for managing a list of RK crypto instance
 * @dev_list:		Used for doing a list of rk_crypto_info
 * @lock:		Control access to dev_list
 * @dbgfs_dir:		Debugfs dentry for statistic directory
 * @dbgfs_stats:	Debugfs dentry for statistic counters
 */
struct rockchip_ip {
	struct list_head	dev_list;
	spinlock_t		lock; /* Control access to dev_list */
	struct dentry		*dbgfs_dir;
	struct dentry		*dbgfs_stats;
};

struct rk_clks {
	const char *name;
	unsigned long max;
};

struct rk_variant {
	int num_clks;
	struct rk_clks rkclks[RK_MAX_CLKS];
};

struct rk_crypto_info {
	struct list_head		list;
	struct device			*dev;
	struct clk_bulk_data		*clks;
	int				num_clks;
	struct reset_control		*rst;
	void __iomem			*reg;
	int				irq;
	const struct rk_variant *variant;
	unsigned long nreq;
	struct crypto_engine *engine;
	struct completion complete;
	int status;
	struct hwrng hwrng;
	struct rk_crypto_trng_health trng_health;
	/*
	 * Consecutive failed reads, the latch they set, and the time (jiffies)
	 * at which a latched source is next given one real read to prove
	 * itself with.  All three are touched only from rk_crypto_trng_read(),
	 * which the hwrng core serialises with reading_mutex.
	 */
	unsigned int trng_fails;
	bool trng_dead;
	unsigned long trng_retry_at;
};

/* the private variable of hash */
struct rk_ahash_ctx {
	/* for fallback */
	struct crypto_ahash		*fallback_tfm;
};

/* the private variable of hash for fallback */
struct rk_ahash_rctx {
	struct rk_crypto_info		*dev;
	struct ahash_request		fallback_req;
	u32				mode;
	int nrsg;
};

/* the private variable of cipher */
struct rk_cipher_ctx {
	unsigned int			keylen;
	u8				key[AES_MAX_KEY_SIZE];
	u8				iv[AES_BLOCK_SIZE];
	struct crypto_skcipher *fallback_tfm;
};

struct rk_cipher_rctx {
	struct rk_crypto_info		*dev;
	u8 backup_iv[AES_BLOCK_SIZE];
	u32				mode;
	struct skcipher_request fallback_req;   // keep at the end
};

struct rk_crypto_tmp {
	u32 type;
	/*
	 * Whether this alg is currently registered with the crypto framework.
	 * A plain bool, NOT a pointer to the rk_crypto_info that registered it:
	 * rk_cipher_algs[] is a single static array shared by every instance of
	 * the IP (RK3399 has two, crypto@ff8b0000 and crypto@ff8b8000), the
	 * algs are registered once by whichever instance probes first, and they
	 * stay registered until the LAST instance is removed. A pointer stored
	 * here therefore outlives the instance it names on the ordinary
	 * teardown path -- remove crypto0 while crypto1 is still up and every
	 * registered alg holds a pointer to crypto0's devm_kzalloc()'d
	 * rk_crypto_info, which is freed the moment crypto0's remove() returns.
	 * Per-request device selection already goes through get_rk_crypto(),
	 * which reads the live rocklist, so nothing needs the pointer.
	 */
	bool registered;
	union {
		struct skcipher_engine_alg skcipher;
		struct ahash_engine_alg hash;
	} alg;
	unsigned long stat_req;
	unsigned long stat_fb;
	unsigned long stat_fb_len;
	unsigned long stat_fb_sglen;
	unsigned long stat_fb_align;
	unsigned long stat_fb_sgdiff;
};

extern struct rk_crypto_tmp rk_ecb_aes_alg;
extern struct rk_crypto_tmp rk_cbc_aes_alg;
extern struct rk_crypto_tmp rk_ecb_des_alg;
extern struct rk_crypto_tmp rk_cbc_des_alg;
extern struct rk_crypto_tmp rk_ecb_des3_ede_alg;
extern struct rk_crypto_tmp rk_cbc_des3_ede_alg;

extern struct rk_crypto_tmp rk_ahash_sha1;
extern struct rk_crypto_tmp rk_ahash_sha256;
extern struct rk_crypto_tmp rk_ahash_md5;

struct rk_crypto_info *get_rk_crypto(void);
struct device *rk_crypto_diag_dev(void);
#endif
