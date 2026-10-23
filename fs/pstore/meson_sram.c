// SPDX-License-Identifier: GPL-2.0+
/*
 * Amlogic on-chip SRAM pstore backend
 * Copyright (C) 2026 Libre Computer
 * Author: Da Xue <da@libre.computer>
 *
 * Keeps the pstore console ring and the last kmsg_dump record in on-chip
 * SRAM, which retains its contents across a warm reset (watchdog bite, soft
 * reboot) without depending on the bootloader leaving a DRAM carve-out
 * alone.  A watchdog bite resets instantly and never runs kmsg_dump(), so
 * the continuously-written CONSOLE ring -- not the dmesg slot -- is the
 * record that survives such a reset; CONFIG_PSTORE_CONSOLE is therefore the
 * load-bearing option for this backend.
 *
 * Binding -- standard mmio-sram CONSUMER, not an mmio-sram child.
 * drivers/misc/sram.c never calls of_platform_populate() and "mmio-sram" is
 * not in of_default_bus_match_table, so a platform_driver bound to a child
 * of the SRAM node can never probe.  Instead this driver is its own device
 * node holding a phandle to a "pool" region of the SRAM node, and takes its
 * memory the way mmio-sram partitions are meant to be consumed:
 * of_gen_pool_get() + gen_pool_alloc().
 *
 *   sram@c8000000 {
 *       compatible = "mmio-sram";
 *       ...
 *       pstore_sram_pool: pstore@1000 {
 *           reg = <0x1000 0x5000>;
 *           pool;
 *           label = "pstore-sram-pool";
 *       };
 *   };
 *
 *   pstore {
 *       compatible = "amlogic,meson-sram-pstore";
 *       sram = <&pstore_sram_pool>;
 *   };
 *
 * The "label" property is not optional in practice: sram.c names the
 * gen_pool after the region's "label" (falling back to the node name) and
 * of_gen_pool_get() looks the pool up by that same string, so an explicit
 * label keeps both sides in agreement.
 *
 * SRAM layout (all offsets relative to the allocated region base):
 *   [0x000] global header (PS_HDR_SIZE bytes)
 *   [PS_HDR_SIZE] console zone (con_size bytes) -- ring buffer
 *   [PS_HDR_SIZE + con_size] dmesg zone (dmg_size bytes) -- single slot
 *
 * Zone sizes are derived at probe from the pool size (60% console,
 * 40% dmesg, 4-byte aligned).
 *
 * NOTE on SoC applicability: the region handed to this driver must be
 * readable and writable by the CPU.  On G12A/G12B/SM1 the only on-chip SRAM
 * exposed in DT is the NPU's AXI SRAM at 0xff000000, which sits behind the
 * NPU's AXI master and external-aborts on CPU access (and is claimed in full
 * by the etnaviv NPU pool) -- it is NOT a valid region for this driver.
 */

#include <linux/err.h>
#include <linux/genalloc.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/kmsg_dump.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pstore.h>
#include <linux/slab.h>

#define MESON_PS_MAGIC    0x4D535053U  /* "MSPS" */
#define MESON_CON_SIG     0x434F4E53U  /* "CONS" */
#define MESON_DMG_SIG     0x444D5347U  /* "DMSG" */
#define MESON_PS_VERSION  1

/*
 * Global header (PS_HDR_SIZE = 32 bytes):
 *   +0  u32  magic
 *   +4  u8   version
 *   +5  u8   flags  (PS_FLAG_*)
 *   +6  u16  oops_count
 *   +8  u32  con_size (console zone total bytes, incl. header)
 *   +12 u32  dmg_size (dmesg zone total bytes, incl. header)
 *   +16 [16 bytes reserved]
 */
#define PS_HDR_MAGIC     0
#define PS_HDR_VERSION   4
#define PS_HDR_FLAGS     5
#define PS_HDR_OOPS_CNT  6
#define PS_HDR_CON_SIZE  8
#define PS_HDR_DMG_SIZE  12
#define PS_HDR_SIZE      32

#define PS_FLAG_CON_VALID  BIT(0)
#define PS_FLAG_DMG_VALID  BIT(1)

/*
 * Console zone header (CON_HDR_LEN = 12 bytes, followed by ring data):
 *   +0  u32  sig
 *   +4  u32  start  (read pointer -- offset into data, 0..data_size-1)
 *   +8  u32  size   (valid bytes -- 0..data_size)
 */
#define CON_HDR_SIG    0
#define CON_HDR_START  4
#define CON_HDR_SIZE   8
#define CON_HDR_LEN    12

/*
 * Dmesg zone header (DMG_HDR_LEN = 32 bytes, followed by crash data):
 *   +0  u32  sig
 *   +4  u32  size    (valid data bytes)
 *   +8  u32  reason  (enum kmsg_dump_reason)
 *   +12 u32  count   (oops count)
 *   +16 u32  tsec_lo
 *   +20 u32  tsec_hi
 *   +24 u32  tnsec_lo
 *   +28 u32  tnsec_hi
 */
#define DMG_HDR_SIG       0
#define DMG_HDR_SIZE      4
#define DMG_HDR_REASON    8
#define DMG_HDR_COUNT     12
#define DMG_HDR_TSEC_LO   16
#define DMG_HDR_TSEC_HI   20
#define DMG_HDR_TNSEC_LO  24
#define DMG_HDR_TNSEC_HI  28
#define DMG_HDR_LEN       32

struct meson_sram_ps {
	void __iomem      *base;
	struct gen_pool   *pool;
	unsigned long      pool_va;   /* gen_pool_alloc() cookie, for free */
	size_t             total_size;
	size_t             con_size;   /* console zone total (incl. CON_HDR_LEN) */
	size_t             dmg_size;   /* dmesg zone total (incl. DMG_HDR_LEN) */
	int                read_step;
	struct pstore_info pstore;
};

#define to_meson_ps(p)  container_of((p), struct meson_sram_ps, pstore)

static inline void __iomem *con_zone(struct meson_sram_ps *ctx)
{
	return ctx->base + PS_HDR_SIZE;
}

static inline void __iomem *dmg_zone(struct meson_sram_ps *ctx)
{
	return ctx->base + PS_HDR_SIZE + ctx->con_size;
}

static inline size_t con_data_size(struct meson_sram_ps *ctx)
{
	return ctx->con_size - CON_HDR_LEN;
}

static inline size_t dmg_data_size(struct meson_sram_ps *ctx)
{
	return ctx->dmg_size - DMG_HDR_LEN;
}

/* ioread/iowrite handle LE byte order internally -- no cpu_to_le needed */

static void meson_sram_ps_init(struct meson_sram_ps *ctx)
{
	memset_io(ctx->base, 0, ctx->total_size);
	iowrite32(MESON_PS_MAGIC,      ctx->base + PS_HDR_MAGIC);
	iowrite8(MESON_PS_VERSION,     ctx->base + PS_HDR_VERSION);
	iowrite32(ctx->con_size,       ctx->base + PS_HDR_CON_SIZE);
	iowrite32(ctx->dmg_size,       ctx->base + PS_HDR_DMG_SIZE);
	iowrite32(MESON_CON_SIG,       con_zone(ctx) + CON_HDR_SIG);
	iowrite32(MESON_DMG_SIG,       dmg_zone(ctx) + DMG_HDR_SIG);
}

static bool meson_sram_ps_valid(struct meson_sram_ps *ctx)
{
	return ioread32(ctx->base + PS_HDR_MAGIC)    == MESON_PS_MAGIC  &&
	       ioread8(ctx->base + PS_HDR_VERSION)   == MESON_PS_VERSION &&
	       ioread32(ctx->base + PS_HDR_CON_SIZE) == ctx->con_size   &&
	       ioread32(ctx->base + PS_HDR_DMG_SIZE) == ctx->dmg_size   &&
	       ioread32(con_zone(ctx) + CON_HDR_SIG) == MESON_CON_SIG   &&
	       ioread32(dmg_zone(ctx) + DMG_HDR_SIG) == MESON_DMG_SIG;
}

static int meson_ps_open(struct pstore_info *psi)
{
	to_meson_ps(psi)->read_step = 0;
	return 0;
}

static int meson_ps_close(struct pstore_info *psi)
{
	return 0;
}

static ssize_t meson_ps_read(struct pstore_record *record)
{
	struct meson_sram_ps *ctx = to_meson_ps(record->psi);
	u8 flags = ioread8(ctx->base + PS_HDR_FLAGS);

	while (ctx->read_step < 2) {
		int step = ctx->read_step++;

		if (step == 0) {
			/* Console record */
			void __iomem *zone = con_zone(ctx);
			size_t dsz = con_data_size(ctx);
			u32 start, size;
			char *buf;

			if (!(flags & PS_FLAG_CON_VALID))
				continue;

			size  = ioread32(zone + CON_HDR_SIZE);
			start = ioread32(zone + CON_HDR_START);
			if (!size || size > dsz)
				continue;

			buf = kmalloc(size, GFP_KERNEL);
			if (!buf)
				return -ENOMEM;

			/*
			 * Ring buffer may wrap: copy in at most two segments.
			 * [start .. dsz-1] then [0 .. remainder-1]
			 */
			if (start + size <= dsz) {
				memcpy_fromio(buf,
					      zone + CON_HDR_LEN + start,
					      size);
			} else {
				size_t tail = dsz - start;
				memcpy_fromio(buf,
					      zone + CON_HDR_LEN + start,
					      tail);
				memcpy_fromio(buf + tail,
					      zone + CON_HDR_LEN,
					      size - tail);
			}

			record->type = PSTORE_TYPE_CONSOLE;
			record->id   = 0;
			record->buf  = buf;
			record->size = size;
			return size;

		} else {
			/* Dmesg record */
			void __iomem *zone = dmg_zone(ctx);
			size_t dsz = dmg_data_size(ctx);
			u32 size, reason, count;
			u64 tsec, tnsec;
			char *buf;

			if (!(flags & PS_FLAG_DMG_VALID))
				continue;

			size   = ioread32(zone + DMG_HDR_SIZE);
			reason = ioread32(zone + DMG_HDR_REASON);
			count  = ioread32(zone + DMG_HDR_COUNT);
			tsec   = (u64)ioread32(zone + DMG_HDR_TSEC_HI) << 32 |
				  ioread32(zone + DMG_HDR_TSEC_LO);
			tnsec  = (u64)ioread32(zone + DMG_HDR_TNSEC_HI) << 32 |
				  ioread32(zone + DMG_HDR_TNSEC_LO);

			if (!size || size > dsz)
				continue;

			buf = kmalloc(size, GFP_KERNEL);
			if (!buf)
				return -ENOMEM;

			memcpy_fromio(buf, zone + DMG_HDR_LEN, size);

			record->type         = PSTORE_TYPE_DMESG;
			record->id           = 1;
			record->reason       = (enum kmsg_dump_reason)reason;
			record->count        = count;
			record->time.tv_sec  = (time64_t)tsec;
			record->time.tv_nsec = (long)tnsec;
			record->buf          = buf;
			record->size         = size;
			return size;
		}
	}

	return 0; /* no more records */
}

static int meson_ps_write(struct pstore_record *record)
{
	struct meson_sram_ps *ctx = to_meson_ps(record->psi);

	if (record->type == PSTORE_TYPE_CONSOLE) {
		void __iomem *zone = con_zone(ctx);
		size_t dsz = con_data_size(ctx);
		size_t len = min_t(size_t, record->size, dsz);
		u32 start = ioread32(zone + CON_HDR_START);
		u32 cur   = ioread32(zone + CON_HDR_SIZE);
		u32 write_off = (start + cur) % dsz;

		if (write_off + len <= dsz) {
			memcpy_toio(zone + CON_HDR_LEN + write_off,
				    record->buf, len);
		} else {
			size_t tail = dsz - write_off;
			memcpy_toio(zone + CON_HDR_LEN + write_off,
				    record->buf, tail);
			memcpy_toio(zone + CON_HDR_LEN,
				    record->buf + tail, len - tail);
		}

		if (cur + len <= dsz) {
			cur += len;
		} else {
			/* Buffer full: advance start past overwritten bytes */
			start = (write_off + len) % dsz;
			cur = dsz;
		}

		iowrite32(start, zone + CON_HDR_START);
		iowrite32(cur,   zone + CON_HDR_SIZE);
		iowrite8(ioread8(ctx->base + PS_HDR_FLAGS) | PS_FLAG_CON_VALID,
			 ctx->base + PS_HDR_FLAGS);

	} else if (record->type == PSTORE_TYPE_DMESG) {
		void __iomem *zone = dmg_zone(ctx);
		size_t dsz = dmg_data_size(ctx);
		size_t len = min_t(size_t, record->size, dsz);
		u64 tsec  = (u64)record->time.tv_sec;
		u64 tnsec = (u64)record->time.tv_nsec;
		u16 oops;

		oops = ioread16(ctx->base + PS_HDR_OOPS_CNT);
		iowrite16(oops + 1, ctx->base + PS_HDR_OOPS_CNT);

		iowrite32((u32)len,              zone + DMG_HDR_SIZE);
		iowrite32((u32)record->reason,   zone + DMG_HDR_REASON);
		iowrite32((u32)record->count,    zone + DMG_HDR_COUNT);
		iowrite32((u32)(tsec  & 0xffffffffU), zone + DMG_HDR_TSEC_LO);
		iowrite32((u32)(tsec  >> 32),         zone + DMG_HDR_TSEC_HI);
		iowrite32((u32)(tnsec & 0xffffffffU), zone + DMG_HDR_TNSEC_LO);
		iowrite32((u32)(tnsec >> 32),         zone + DMG_HDR_TNSEC_HI);
		memcpy_toio(zone + DMG_HDR_LEN, record->buf, len);

		iowrite8(ioread8(ctx->base + PS_HDR_FLAGS) | PS_FLAG_DMG_VALID,
			 ctx->base + PS_HDR_FLAGS);
	}

	return 0;
}

static int meson_ps_erase(struct pstore_record *record)
{
	struct meson_sram_ps *ctx = to_meson_ps(record->psi);
	u8 flags = ioread8(ctx->base + PS_HDR_FLAGS);

	if (record->type == PSTORE_TYPE_CONSOLE) {
		memset_io(con_zone(ctx), 0, ctx->con_size);
		iowrite32(MESON_CON_SIG, con_zone(ctx) + CON_HDR_SIG);
		iowrite8(flags & ~PS_FLAG_CON_VALID, ctx->base + PS_HDR_FLAGS);
	} else if (record->type == PSTORE_TYPE_DMESG) {
		memset_io(dmg_zone(ctx), 0, ctx->dmg_size);
		iowrite32(MESON_DMG_SIG, dmg_zone(ctx) + DMG_HDR_SIG);
		iowrite8(flags & ~PS_FLAG_DMG_VALID, ctx->base + PS_HDR_FLAGS);
	}

	return 0;
}

static int meson_sram_ps_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct meson_sram_ps *ctx;
	phys_addr_t phys;
	size_t usable;
	char *buf;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	if (!of_property_present(dev->of_node, "sram")) {
		dev_err(dev, "missing 'sram' phandle to an mmio-sram pool\n");
		return -EINVAL;
	}

	/*
	 * The mmio-sram device may not have probed yet -- the pool only
	 * exists once it has.  Defer rather than fail permanently.
	 */
	ctx->pool = of_gen_pool_get(dev->of_node, "sram", 0);
	if (!ctx->pool)
		return dev_err_probe(dev, -EPROBE_DEFER,
				     "SRAM pool not available yet\n");

	ctx->total_size = gen_pool_avail(ctx->pool);
	if (ctx->total_size < PS_HDR_SIZE + CON_HDR_LEN + DMG_HDR_LEN + 128) {
		dev_err(dev, "SRAM pool too small: %zu bytes\n",
			ctx->total_size);
		return -EINVAL;
	}
	usable = ctx->total_size - PS_HDR_SIZE;

	/* Split usable space: 60% console, 40% dmesg, 4-byte aligned */
	ctx->con_size = ALIGN_DOWN((usable * 3) / 5, 4);
	ctx->dmg_size = usable - ctx->con_size;

	/*
	 * Take the whole pool.  gen_pool_alloc() hands back the virtual
	 * address mmio-sram already established for the region (ioremap or
	 * ioremap_wc), so no second mapping is needed -- and must not be
	 * created, since ioremap of an already-mapped resource conflicts.
	 */
	ctx->pool_va = gen_pool_alloc(ctx->pool, ctx->total_size);
	if (!ctx->pool_va) {
		dev_err(dev, "gen_pool_alloc(%zu) failed\n", ctx->total_size);
		return -ENOMEM;
	}
	ctx->base = (void __iomem *)(uintptr_t)ctx->pool_va;
	phys = gen_pool_virt_to_phys(ctx->pool, ctx->pool_va);

	if (!meson_sram_ps_valid(ctx)) {
		dev_info(dev, "SRAM header invalid or zone size mismatch -- reinitializing\n");
		meson_sram_ps_init(ctx);
	}

	/*
	 * pstore core writes crash data into psi->buf (DRAM) before calling
	 * write(); our write() copies it to SRAM.  Size to fit one dmesg slot.
	 */
	buf = devm_kzalloc(dev, dmg_data_size(ctx), GFP_KERNEL);
	if (!buf) {
		ret = -ENOMEM;
		goto err_free_pool;
	}

	ctx->pstore.owner      = THIS_MODULE;
	ctx->pstore.name       = "meson-sram";
	ctx->pstore.flags      = PSTORE_FLAGS_DMESG | PSTORE_FLAGS_CONSOLE;
	ctx->pstore.buf        = buf;
	ctx->pstore.bufsize    = dmg_data_size(ctx);
	ctx->pstore.max_reason = KMSG_DUMP_OOPS;
	ctx->pstore.open       = meson_ps_open;
	ctx->pstore.close      = meson_ps_close;
	ctx->pstore.read       = meson_ps_read;
	ctx->pstore.write      = meson_ps_write;
	ctx->pstore.erase      = meson_ps_erase;
	raw_spin_lock_init(&ctx->pstore.buf_lock);

	platform_set_drvdata(pdev, ctx);

	ret = pstore_register(&ctx->pstore);
	if (ret) {
		/*
		 * pstore takes a single backend: -EBUSY here means another
		 * one (typically ramoops) already registered.
		 */
		dev_err(dev, "pstore_register failed: %d\n", ret);
		goto err_free_pool;
	}

	dev_info(dev, "registered: phys=%pa total=%zuKB con=%zuKB dmg=%zuKB\n",
		 &phys,
		 ctx->total_size / 1024,
		 con_data_size(ctx) / 1024,
		 dmg_data_size(ctx) / 1024);
	return 0;

err_free_pool:
	gen_pool_free(ctx->pool, ctx->pool_va, ctx->total_size);
	return ret;
}

static void meson_sram_ps_remove(struct platform_device *pdev)
{
	struct meson_sram_ps *ctx = platform_get_drvdata(pdev);

	pstore_unregister(&ctx->pstore);
	gen_pool_free(ctx->pool, ctx->pool_va, ctx->total_size);
}

static const struct of_device_id meson_sram_ps_of_ids[] = {
	{ .compatible = "amlogic,meson-sram-pstore" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, meson_sram_ps_of_ids);

static struct platform_driver meson_sram_ps_driver = {
	.probe  = meson_sram_ps_probe,
	.remove = meson_sram_ps_remove,
	.driver = {
		.name           = "meson-sram-pstore",
		.of_match_table = meson_sram_ps_of_ids,
	},
};
module_platform_driver(meson_sram_ps_driver);

MODULE_AUTHOR("Da Xue <da@libre.computer>");
MODULE_DESCRIPTION("Amlogic on-chip SRAM pstore backend");
MODULE_LICENSE("GPL v2");
