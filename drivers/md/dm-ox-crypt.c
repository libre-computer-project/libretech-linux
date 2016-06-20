/* linux/drivers/md/dm-ox-crypt.c
 *
 * OX800 DPE core compatable device encryption 
 */

/*
 * Copyright (C) 2003 Christophe Saout <christophe@saout.de>
 * Copyright (C) 2004 Clemens Fruhwirth <clemens@endorphin.org>
 *
 * This file is released under the GPL.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/bio.h>
#include <linux/blkdev.h>
#include <linux/mempool.h>
#include <linux/slab.h>
#include <linux/crypto.h>
#include <linux/workqueue.h>
#include <asm/atomic.h>
#include <asm/scatterlist.h>
#include <asm/page.h>
#include <asm/arch/cipher.h>

#include "dm.h"

#define DM_MSG_PREFIX "ox-crypt: "

/*
 * per bio private data
 */
struct oxcrypt_io {
	struct dm_target *target;
	struct bio *bio;
	struct bio *first_clone;
	struct work_struct work;
	atomic_t pending;
	int error;
};

/*
 * context holding the current state of a multi-part conversion
 */
struct convert_context {
	struct bio *bio_in;
	struct bio *bio_out;
	unsigned int offset_in;
	unsigned int offset_out;
	unsigned int idx_in;
	unsigned int idx_out;
	sector_t sector;
	int write;
};

struct oxcrypt_config;

struct oxcrypt_iv_operations {
	int (*ctr)(struct oxcrypt_config *cc, struct dm_target *ti,
	           const char *opts);
	void (*dtr)(struct oxcrypt_config *cc);
	const char *(*status)(struct oxcrypt_config *cc);
	int (*generator)(struct oxcrypt_config *cc, u8 *iv, sector_t sector);
};

/*
 * Crypt: maps a linear range of a block device
 * and encrypts / decrypts at the same time.
 */

struct oxcrypt_config {
	struct dm_dev *dev;
	sector_t start;

	/*
	 * pool for per bio private data and
	 * for encryption buffer pages
	 */
	mempool_t *io_pool;
	mempool_t *page_pool;

	/*
	 * crypto related data
	 */
	struct oxcrypt_iv_operations *iv_gen_ops;
	void *iv_gen_private;
	sector_t iv_offset;
	unsigned int iv_size;

	struct crypto_tfm *tfm;
	u8 key[OX800DPE_KEYSIZE]; /* size of key is fixed by hardware */
    u8 iv_key[OX800DPE_KEYSIZE];
};

#define MIN_IOS        256
#define MIN_POOL_PAGES 32
#define MIN_BIO_PAGES  8

static struct kmem_cache *_oxcrypt_io_pool;

/*
 * Mempool alloc and free functions for the page
 */
static void *mempool_alloc_page(gfp_t gfp_mask, void *data)
{
	return alloc_page(gfp_mask);
}

static void mempool_free_page(void *page, void *data)
{
	__free_page(page);
}


/*
 * Different IV generation algorithms:
 *
 * oxsemi:
 *        Uses the 32 sector number and a reproducable hash of target device
 *        properties to generate bits 35-32
 *
 */

static int oxcrypt_iv_oxsemi_gen(struct oxcrypt_config *cc, u8 *iv, sector_t sector)
{
	*((u32* )iv) = cpu_to_le32(sector & 0xffffffff);
    *( ((u32* )iv) + 1) = 0; /** @todo bits 35 - 32 */

	return 0;
}

static struct oxcrypt_iv_operations oxcrypt_iv_oxsemi_ops = {
	.generator = oxcrypt_iv_oxsemi_gen
};


/*static inline*/ int
oxcrypt_convert_scatterlist(struct oxcrypt_config *cc, struct scatterlist *out,
                          struct scatterlist *in, unsigned int length,
                          int write, sector_t sector)
{
	u8 iv[OX800DPE_KEYSIZE];
	int r = 0;

	if (cc->iv_gen_ops) { /* probably no need to check this */
        u8* pri = cc->key;
        u8* twe = cc->iv_key;

		r = cc->iv_gen_ops->generator(cc, iv, sector);
		if (r < 0)
			return r;

		if (write)
			r = ox800_aeslrw_encrypt(in, out, 1, iv, pri, twe);
		else
			r = ox800_aeslrw_decrypt(in, out, 1, iv, pri, twe);
	} else {
        BUG();
	}

    //printk("back\n");
    if (r < 0) {
        printk(KERN_ERR"oxcrypt_convert_scatterlist: core driver returned error %d\n",r);
    }
    
	return r;
}

static void
oxcrypt_convert_init(struct oxcrypt_config *cc, struct convert_context *ctx,
                   struct bio *bio_out, struct bio *bio_in,
                   sector_t sector, int write)
{
	ctx->bio_in = bio_in;
	ctx->bio_out = bio_out;
	ctx->offset_in = 0;
	ctx->offset_out = 0;
	ctx->idx_in = bio_in ? bio_in->bi_idx : 0;
	ctx->idx_out = bio_out ? bio_out->bi_idx : 0;
	ctx->sector = sector + cc->iv_offset;
	ctx->write = write;
}

/**
 * Encrypt / decrypt data from one bio to another one (can be the same one)
 *
 * @todo This only goes atr one sector at a time, could it be made to this in
 * a scatter gather list of multiple sectors? 
 */
static int oxcrypt_convert(struct oxcrypt_config *cc,
                         struct convert_context *ctx)
{
	int r = 0;
    struct bio_vec *bv_in ;
    struct bio_vec *bv_out ;
    struct scatterlist sg_in;
    struct scatterlist sg_out;
    
    //printk("oxcrypt_convert config %p context %p \n", cc, ctx);

	while(ctx->idx_in < ctx->bio_in->bi_vcnt &&
	      ctx->idx_out < ctx->bio_out->bi_vcnt) {

        bv_in = bio_iovec_idx(ctx->bio_in, ctx->idx_in);
        bv_out = bio_iovec_idx(ctx->bio_out, ctx->idx_out);
        
        sg_in.page = bv_in->bv_page;
        sg_in.offset = bv_in->bv_offset + ctx->offset_in;
        sg_in.length = 1 << SECTOR_SHIFT;
        
        sg_out.page = bv_out->bv_page;
        sg_out.offset = bv_out->bv_offset + ctx->offset_out;
        sg_out.length = 1 << SECTOR_SHIFT;
        
		ctx->offset_in += sg_in.length;
		if (ctx->offset_in >= bv_in->bv_len) {
			ctx->offset_in = 0;
			ctx->idx_in++;
		}

		ctx->offset_out += sg_out.length;
		if (ctx->offset_out >= bv_out->bv_len) {
			ctx->offset_out = 0;
			ctx->idx_out++;
		}

		r = oxcrypt_convert_scatterlist(cc, &sg_out, &sg_in, sg_in.length,
		                              ctx->write, ctx->sector);
		if (r < 0)
			break;

		ctx->sector++;
	}

	return r;
}

/*
 * Generate a new unfragmented bio with the given size
 * This should never violate the device limitations
 * May return a smaller bio when running out of pages
 */
static struct bio *
oxcrypt_alloc_buffer(struct oxcrypt_config *cc, unsigned int size,
                   struct bio *base_bio, unsigned int *bio_vec_idx)
{
	struct bio *bio;
	unsigned int nr_iovecs = dm_div_up(size, PAGE_SIZE);
	int gfp_mask = GFP_NOIO | __GFP_HIGHMEM;
	unsigned long flags = current->flags;
	unsigned int i;

	/*
	 * Tell VM to act less aggressively and fail earlier.
	 * This is not necessary but increases throughput.
	 * FIXME: Is this really intelligent?
	 */
	current->flags &= ~PF_MEMALLOC;

	if (base_bio)
		bio = bio_clone(base_bio, GFP_NOIO);
	else
		bio = bio_alloc(GFP_NOIO, nr_iovecs);
	if (!bio) {
		if (flags & PF_MEMALLOC)
			current->flags |= PF_MEMALLOC;
		return NULL;
	}

	/* if the last bio was not complete, continue where that one ended */
	bio->bi_idx = *bio_vec_idx;
	bio->bi_vcnt = *bio_vec_idx;
	bio->bi_size = 0;
	bio->bi_flags &= ~(1 << BIO_SEG_VALID);

	/* bio->bi_idx pages have already been allocated */
	size -= bio->bi_idx * PAGE_SIZE;

	for(i = bio->bi_idx; i < nr_iovecs; i++) {
		struct bio_vec *bv = bio_iovec_idx(bio, i);

		bv->bv_page = mempool_alloc(cc->page_pool, gfp_mask);
		if (!bv->bv_page)
			break;

		/*
		 * if additional pages cannot be allocated without waiting,
		 * return a partially allocated bio, the caller will then try
		 * to allocate additional bios while submitting this partial bio
		 */
		if ((i - bio->bi_idx) == (MIN_BIO_PAGES - 1))
			gfp_mask = (gfp_mask | __GFP_NOWARN) & ~__GFP_WAIT;

		bv->bv_offset = 0;
		if (size > PAGE_SIZE)
			bv->bv_len = PAGE_SIZE;
		else
			bv->bv_len = size;

		bio->bi_size += bv->bv_len;
		bio->bi_vcnt++;
		size -= bv->bv_len;
	}

	if (flags & PF_MEMALLOC)
		current->flags |= PF_MEMALLOC;

	if (!bio->bi_size) {
		bio_put(bio);
		return NULL;
	}

	/*
	 * Remember the last bio_vec allocated to be able
	 * to correctly continue after the splitting.
	 */
	*bio_vec_idx = bio->bi_vcnt;

	return bio;
}

static void oxcrypt_free_buffer_pages(struct oxcrypt_config *cc,
                                    struct bio *bio, unsigned int bytes)
{
	unsigned int i, start, end;
	struct bio_vec *bv;

	/*
	 * This is ugly, but Jens Axboe thinks that using bi_idx in the
	 * endio function is too dangerous at the moment, so I calculate the
	 * correct position using bi_vcnt and bi_size.
	 * The bv_offset and bv_len fields might already be modified but we
	 * know that we always allocated whole pages.
	 * A fix to the bi_idx issue in the kernel is in the works, so
	 * we will hopefully be able to revert to the cleaner solution soon.
	 */
	i = bio->bi_vcnt - 1;
	bv = bio_iovec_idx(bio, i);
	end = (i << PAGE_SHIFT) + (bv->bv_offset + bv->bv_len) - bio->bi_size;
	start = end - bytes;

	start >>= PAGE_SHIFT;
	if (!bio->bi_size)
		end = bio->bi_vcnt;
	else
		end >>= PAGE_SHIFT;

	for(i = start; i < end; i++) {
		bv = bio_iovec_idx(bio, i);
		BUG_ON(!bv->bv_page);
		mempool_free(bv->bv_page, cc->page_pool);
		bv->bv_page = NULL;
	}
}

/*
 * One of the bios was finished. Check for completion of
 * the whole request and correctly clean up the buffer.
 */
static void dec_pending(struct oxcrypt_io *io, int error)
{
	struct oxcrypt_config *cc = (struct oxcrypt_config *) io->target->private;

	if (error < 0)
		io->error = error;

	if (!atomic_dec_and_test(&io->pending))
		return;

	if (io->first_clone)
		bio_put(io->first_clone);

	bio_endio(io->bio, io->bio->bi_size, io->error);

	mempool_free(io, cc->io_pool);
}

/*
 * kcryptd:
 *
 * Needed because it would be very unwise to do decryption in an
 * interrupt context, so bios returning from read requests get
 * queued here.
 */
static struct workqueue_struct *_kcryptd_workqueue;

static void kcryptd_do_work(struct work_struct *work)
{
	struct oxcrypt_io *io = container_of(work, struct oxcrypt_io, work);
	struct oxcrypt_config *cc = (struct oxcrypt_config *) io->target->private;
	struct convert_context ctx;
	int r;

	oxcrypt_convert_init(cc, &ctx, io->bio, io->bio,
	                   io->bio->bi_sector - io->target->begin, 0);

    /* printk("kcryptd_do_work %d sectors\n", ctx.bio_in->bi_vcnt ); */
	r = oxcrypt_convert(cc, &ctx);

	dec_pending(io, r);
}

static void kcryptd_queue_io(struct oxcrypt_io *io)
{
	INIT_WORK(&io->work, kcryptd_do_work);
	queue_work(_kcryptd_workqueue, &io->work);
}

/*
 * Decode key from its hex representation
 */
static int oxcrypt_decode_key(u8 *key, char *hex, unsigned int size)
{
	char buffer[3];
	char *endp;
	unsigned int i;

	buffer[2] = '\0';

	for(i = 0; i < size; i++) {
		buffer[0] = *hex++;
		buffer[1] = *hex++;
        key[i] = (u8)simple_strtoul(buffer, &endp, 16);

		if (endp != &buffer[2])
			return -EINVAL;
	}

	if (*hex != '\0')
		return -EINVAL;

    /*
    printk(KERN_INFO"key ="); 
    for (i = 0; i < OX800DPE_KEYSIZE; ++i)
        printk("%02x", key[i]);
    printk("\n");
    */
    
	return 0;
}

/*
 * Encode key into its hex representation
 */
static void oxcrypt_encode_key(char *hex, u8 *key, unsigned int size)
{
	unsigned int i;

	for(i = 0; i < size; i++) {
		sprintf(hex, "%02x", *key);
		hex += 2;
		key++;
	}
}

/*
 * Construct an encryption mapping, much simpler:
 * <key> <iv-key> <iv_offset> <dev_path> <start>
 */
static int oxcrypt_ctr(struct dm_target *ti, unsigned int argc, char **argv)
{
	struct oxcrypt_config *cc;
	unsigned long long tmpll;

	if (argc != 5) {
		ti->error = DM_MSG_PREFIX "Not enough arguments";
		return -EINVAL;
	}

	cc = kmalloc(sizeof(*cc) , GFP_KERNEL);
	if (cc == NULL) {
		ti->error =
			DM_MSG_PREFIX "Cannot allocate transparent encryption context";
		return -ENOMEM;
	}

    memset( cc, 0, sizeof(*cc) );

	if (oxcrypt_decode_key(cc->key, argv[0], OX800DPE_KEYSIZE) < 0) {
		ti->error = DM_MSG_PREFIX "Error decoding key";
		goto bad1;
	}

	if (oxcrypt_decode_key(cc->iv_key, argv[1], OX800DPE_KEYSIZE) < 0) {
		ti->error = DM_MSG_PREFIX "Error decoding iv key";
		goto bad1;
	}
    
    /*
     * Force the ivmode to the ox-semi version
     */
	cc->iv_gen_ops = &oxcrypt_iv_oxsemi_ops;

	cc->io_pool = mempool_create(MIN_IOS, mempool_alloc_slab,
				     mempool_free_slab, _oxcrypt_io_pool);
	if (!cc->io_pool) {
		ti->error = DM_MSG_PREFIX "Cannot allocate crypt io mempool";
		goto bad3;
	}

	cc->page_pool = mempool_create(MIN_POOL_PAGES, mempool_alloc_page,
				       mempool_free_page, NULL);
	if (!cc->page_pool) {
		ti->error = DM_MSG_PREFIX "Cannot allocate page mempool";
		goto bad4;
	}

	if (sscanf(argv[2], "%llu", &tmpll) != 1) {
		ti->error = DM_MSG_PREFIX "Invalid iv_offset sector";
		goto bad5;
	}
	cc->iv_offset = tmpll;

	if (sscanf(argv[4], "%llu", &tmpll) != 1) {
		ti->error = DM_MSG_PREFIX "Invalid device sector";
		goto bad5;
	}
	cc->start = tmpll;

	if (dm_get_device(ti, argv[3], cc->start, ti->len,
	                  dm_table_get_mode(ti->table), &cc->dev)) {
		ti->error = DM_MSG_PREFIX "Device lookup failed";
		goto bad5;
	}


	ti->private = cc;

	return 0;

bad5:
	mempool_destroy(cc->page_pool);
bad4:
	mempool_destroy(cc->io_pool);
bad3:
	if (cc->iv_gen_ops && cc->iv_gen_ops->dtr)
		cc->iv_gen_ops->dtr(cc);
bad1:
	kfree(cc);
	return -EINVAL;
}

static void oxcrypt_dtr(struct dm_target *ti)
{
	struct oxcrypt_config *cc = (struct oxcrypt_config *) ti->private;

	mempool_destroy(cc->page_pool);
	mempool_destroy(cc->io_pool);

	if (cc->iv_gen_ops && cc->iv_gen_ops->dtr)
		cc->iv_gen_ops->dtr(cc);
	dm_put_device(ti, cc->dev);
    
	kfree(cc);
}

static int oxcrypt_endio(struct bio *bio, unsigned int done, int error)
{
	struct oxcrypt_io *io = (struct oxcrypt_io *) bio->bi_private;
	struct oxcrypt_config *cc = (struct oxcrypt_config *) io->target->private;

	if (bio_data_dir(bio) == WRITE) {
		/*
		 * free the processed pages, even if
		 * it's only a partially completed write
		 */
		oxcrypt_free_buffer_pages(cc, bio, done);
	}

	if (bio->bi_size)
		return 1;

	bio_put(bio);

	/*
	 * successful reads are decrypted by the worker thread
	 */
	if ((bio_data_dir(bio) == READ)
	    && bio_flagged(bio, BIO_UPTODATE)) {
		kcryptd_queue_io(io);
		return 0;
	}

	dec_pending(io, error);
	return error;
}

static struct bio *
oxcrypt_clone(struct oxcrypt_config *cc, struct oxcrypt_io *io, struct bio *bio,
            sector_t sector, unsigned int *bvec_idx,
            struct convert_context *ctx)
{
	struct bio *clone;

	if (bio_data_dir(bio) == WRITE) {
		clone = oxcrypt_alloc_buffer(cc, bio->bi_size,
                                 io->first_clone, bvec_idx);
		if (clone) {
			ctx->bio_out = clone;
			if (oxcrypt_convert(cc, ctx) < 0) {
				oxcrypt_free_buffer_pages(cc, clone,
				                        clone->bi_size);
				bio_put(clone);
				return NULL;
			}
		}
	} else {
		/*
		 * The block layer might modify the bvec array, so always
		 * copy the required bvecs because we need the original
		 * one in order to decrypt the whole bio data *afterwards*.
		 */
		clone = bio_alloc(GFP_NOIO, bio_segments(bio));
		if (clone) {
			clone->bi_idx = 0;
			clone->bi_vcnt = bio_segments(bio);
			clone->bi_size = bio->bi_size;
			memcpy(clone->bi_io_vec, bio_iovec(bio),
			       sizeof(struct bio_vec) * clone->bi_vcnt);
		}
	}

	if (!clone)
		return NULL;

	clone->bi_private = io;
	clone->bi_end_io = oxcrypt_endio;
	clone->bi_bdev = cc->dev->bdev;
	clone->bi_sector = cc->start + sector;
	clone->bi_rw = bio->bi_rw;

	return clone;
}

static int oxcrypt_map(struct dm_target *ti, struct bio *bio,
		     union map_info *map_context)
{
	struct oxcrypt_config *cc = (struct oxcrypt_config *) ti->private;
	struct oxcrypt_io *io = mempool_alloc(cc->io_pool, GFP_NOIO);
	struct convert_context ctx;
	struct bio *clone;
	unsigned int remaining = bio->bi_size;
	sector_t sector = bio->bi_sector - ti->begin;
	unsigned int bvec_idx = 0;

	io->target = ti;
	io->bio = bio;
	io->first_clone = NULL;
	io->error = 0;
	atomic_set(&io->pending, 1); /* hold a reference */

	if (bio_data_dir(bio) == WRITE)
		oxcrypt_convert_init(cc, &ctx, NULL, bio, sector, 1);

	/*
	 * The allocated buffers can be smaller than the whole bio,
	 * so repeat the whole process until all the data can be handled.
	 */
	while (remaining) {
		clone = oxcrypt_clone(cc, io, bio, sector, &bvec_idx, &ctx);
		if (!clone)
			goto cleanup;

		if (!io->first_clone) {
			/*
			 * hold a reference to the first clone, because it
			 * holds the bio_vec array and that can't be freed
			 * before all other clones are released
			 */
			bio_get(clone);
			io->first_clone = clone;
		}
		atomic_inc(&io->pending);

		remaining -= clone->bi_size;
		sector += bio_sectors(clone);

		generic_make_request(clone);

		/* out of memory -> run queues */
		if (remaining)
			congestion_wait(bio_data_dir(clone), HZ/100);
	}

	/* drop reference, clones could have returned before we reach this */
	dec_pending(io, 0);
	return 0;

cleanup:
	if (io->first_clone) {
		dec_pending(io, -ENOMEM);
		return 0;
	}

	/* if no bio has been dispatched yet, we can directly return the error */
	mempool_free(io, cc->io_pool);
	return -ENOMEM;
}

static int oxcrypt_status(struct dm_target *ti, status_type_t type,
			char *result, unsigned int maxlen)
{
	struct oxcrypt_config *cc = (struct oxcrypt_config *) ti->private;
	char buffer[32];
	const char *cipher;
	const char *chainmode = NULL;
	unsigned int sz = 0;

	switch (type) {
	case STATUSTYPE_INFO:
		result[0] = '\0';
		break;

	case STATUSTYPE_TABLE:
		cipher = "AES";

		chainmode = "ecb";

        DMEMIT("%s-%s ", cipher, chainmode);

        oxcrypt_encode_key(result + sz, cc->key, OX800DPE_KEYSIZE);
        sz += OX800DPE_KEYSIZE << 1;

		format_dev_t(buffer, cc->dev->bdev->bd_dev);
		DMEMIT(" %llu %s %llu", (unsigned long long)cc->iv_offset,
            buffer, (unsigned long long)cc->start);
		break;
	}
	return 0;
}

static struct target_type oxcrypt_target = {
	.name   = "ox-crypt",
	.version= {1, 1, 0},
	.module = THIS_MODULE,
	.ctr    = oxcrypt_ctr,
	.dtr    = oxcrypt_dtr,
	.map    = oxcrypt_map,
	.status = oxcrypt_status,
};

static int __init dm_oxcrypt_init(void)
{
	int r;

	_oxcrypt_io_pool = kmem_cache_create("dm-ox-oxcrypt_io",
	                                   sizeof(struct oxcrypt_io),
	                                   0, 0, NULL);
	if (!_oxcrypt_io_pool)
		return -ENOMEM;

	_kcryptd_workqueue = create_workqueue("kcryptd");
	if (!_kcryptd_workqueue) {
		r = -ENOMEM;
		DMERR("couldn't create kcryptd");
		goto bad1;
	}

	r = dm_register_target(&oxcrypt_target);
	if (r < 0) {
		DMERR("register failed %d", r);
		goto bad2;
	}

	return 0;

bad2:
	destroy_workqueue(_kcryptd_workqueue);
bad1:
	kmem_cache_destroy(_oxcrypt_io_pool);
	return r;
}

static void __exit dm_oxcrypt_exit(void)
{
	int r = dm_unregister_target(&oxcrypt_target);

	if (r < 0)
		DMERR("unregister failed %d", r);

	destroy_workqueue(_kcryptd_workqueue);
	kmem_cache_destroy(_oxcrypt_io_pool);
}

module_init(dm_oxcrypt_init);
module_exit(dm_oxcrypt_exit);

MODULE_AUTHOR("Oxford Semiconductor based on work of Christophe Saout");
MODULE_DESCRIPTION(DM_NAME " target for hardware encryption / decryption");
MODULE_LICENSE("GPL");
