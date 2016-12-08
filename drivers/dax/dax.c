/*
 * Copyright(c) 2016 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */
#include <linux/pagemap.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/mount.h>
#include <linux/pfn_t.h>
#include <linux/async.h>
#include <linux/hash.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/dax.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include "dax.h"

static dev_t dax_devt;
static struct class *dax_class;
static DEFINE_IDA(dax_minor_ida);
static int nr_dax = CONFIG_NR_DEV_DAX;
module_param(nr_dax, int, S_IRUGO);
static struct vfsmount *dax_mnt;
static struct kmem_cache *dax_cache __read_mostly;
static struct super_block *dax_superblock __read_mostly;
MODULE_PARM_DESC(nr_dax, "max number of device-dax instances");
static ASYNC_DOMAIN_EXCLUSIVE(dax_dev_async);

/**
 * struct dax_region - mapping infrastructure for dax devices
 * @id: kernel-wide unique region for a memory range
 * @base: linear address corresponding to @res
 * @kref: to pin while other agents have a need to do lookups
 * @lock: synchronize changes / consistent-access to the resource tree (@res)
 * @dev: parent device backing this region
 * @seed: next device for dynamic allocation / configuration
 * @align: allocation and mapping alignment for child dax devices
 * @res: physical address range of the region
 * @child_count: number of registered dax device instances
 * @pfn_flags: identify whether the pfns are paged back or not
 */
struct dax_region {
	int id;
	struct ida ida;
	void *base;
	struct kref kref;
	struct mutex lock;
	struct device *dev;
	struct device *seed;
	unsigned int align;
	struct resource res;
	atomic_t child_count;
	unsigned long pfn_flags;
};

/**
 * struct dax_dev - subdivision of a dax region
 * @region - parent region
 * @resize_lock - for resource size reductions
 * @dev - device backing the character device
 * @cdev - core chardev data
 * @alive - !alive + rcu grace period == no new mappings can be established
 * @id - child id in the region
 * @num_resources - number of physical address extents in this device
 * @res - array of physical address ranges
 */
struct dax_dev {
	struct dax_region *region;
	rwlock_t resize_lock;
	struct inode *inode;
	struct device dev;
	struct cdev cdev;
	bool alive;
	int id;
	int num_resources;
	struct resource **res;
};

#define for_each_dax_region_resource(dax_region, res) \
	for (res = (dax_region)->res.child; res; res = res->sibling)

static unsigned long long dax_region_avail_size(
		struct dax_region *dax_region)
{
	unsigned long long size;
	struct resource *res;

	WARN_ON_ONCE(!mutex_is_locked(&dax_region->lock));

	size = resource_size(&dax_region->res);
	for_each_dax_region_resource(dax_region, res) {
		dev_dbg(dax_region->dev, "%s: %pr offset: %lx\n",
				res->name, res, res->desc);
		size -= resource_size(res);
	}

	return size;
}

static ssize_t available_size_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dax_region *dax_region;
	ssize_t rc = -ENXIO;

	device_lock(dev);
	dax_region = dev_get_drvdata(dev);
	if (dax_region) {
		mutex_lock(&dax_region->lock);
		rc = sprintf(buf, "%llu\n", dax_region_avail_size(dax_region));
		mutex_unlock(&dax_region->lock);
	}
	device_unlock(dev);

	return rc;
}
static DEVICE_ATTR_RO(available_size);

static ssize_t id_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dax_region *dax_region;
	ssize_t rc = -ENXIO;

	device_lock(dev);
	dax_region = dev_get_drvdata(dev);
	if (dax_region)
		rc = sprintf(buf, "%d\n", dax_region->id);
	device_unlock(dev);

	return rc;
}
static DEVICE_ATTR_RO(id);

static ssize_t region_size_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dax_region *dax_region;
	ssize_t rc = -ENXIO;

	device_lock(dev);
	dax_region = dev_get_drvdata(dev);
	if (dax_region)
		rc = sprintf(buf, "%llu\n", (unsigned long long)
				resource_size(&dax_region->res));
	device_unlock(dev);

	return rc;
}
static struct device_attribute dev_attr_region_size = __ATTR(size, 0444,
		region_size_show, NULL);

static ssize_t align_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dax_region *dax_region;
	ssize_t rc = -ENXIO;

	device_lock(dev);
	dax_region = dev_get_drvdata(dev);
	if (dax_region)
		rc = sprintf(buf, "%u\n", dax_region->align);
	device_unlock(dev);

	return rc;
}
static DEVICE_ATTR_RO(align);

static ssize_t seed_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dax_region *dax_region;
	ssize_t rc = -ENXIO;

	device_lock(dev);
	dax_region = dev_get_drvdata(dev);
	if (dax_region) {
		mutex_lock(&dax_region->lock);
		if (dax_region->seed)
			rc = sprintf(buf, "%s\n", dev_name(dax_region->seed));
		mutex_unlock(&dax_region->lock);
	}
	device_unlock(dev);

	return rc;
}
static DEVICE_ATTR_RO(seed);

static struct attribute *dax_region_attributes[] = {
	&dev_attr_available_size.attr,
	&dev_attr_region_size.attr,
	&dev_attr_align.attr,
	&dev_attr_seed.attr,
	&dev_attr_id.attr,
	NULL,
};

static const struct attribute_group dax_region_attribute_group = {
	.name = "dax_region",
	.attrs = dax_region_attributes,
};

static const struct attribute_group *dax_region_attribute_groups[] = {
	&dax_region_attribute_group,
	NULL,
};

static struct inode *dax_alloc_inode(struct super_block *sb)
{
	return kmem_cache_alloc(dax_cache, GFP_KERNEL);
}

static void dax_i_callback(struct rcu_head *head)
{
	struct inode *inode = container_of(head, struct inode, i_rcu);

	kmem_cache_free(dax_cache, inode);
}

static void dax_destroy_inode(struct inode *inode)
{
	call_rcu(&inode->i_rcu, dax_i_callback);
}

static const struct super_operations dax_sops = {
	.statfs = simple_statfs,
	.alloc_inode = dax_alloc_inode,
	.destroy_inode = dax_destroy_inode,
	.drop_inode = generic_delete_inode,
};

static struct dentry *dax_mount(struct file_system_type *fs_type,
		int flags, const char *dev_name, void *data)
{
	return mount_pseudo(fs_type, "dax:", &dax_sops, NULL, DAXFS_MAGIC);
}

static struct file_system_type dax_type = {
	.name = "dax",
	.mount = dax_mount,
	.kill_sb = kill_anon_super,
};

static int dax_test(struct inode *inode, void *data)
{
	return inode->i_cdev == data;
}

static int dax_set(struct inode *inode, void *data)
{
	inode->i_cdev = data;
	return 0;
}

static struct inode *dax_inode_get(struct cdev *cdev, dev_t devt)
{
	struct inode *inode;

	inode = iget5_locked(dax_superblock, hash_32(devt + DAXFS_MAGIC, 31),
			dax_test, dax_set, cdev);

	if (!inode)
		return NULL;

	if (inode->i_state & I_NEW) {
		inode->i_mode = S_IFCHR;
		inode->i_flags = S_DAX;
		inode->i_rdev = devt;
		mapping_set_gfp_mask(&inode->i_data, GFP_USER);
		unlock_new_inode(inode);
	}
	return inode;
}

static void init_once(void *inode)
{
	inode_init_once(inode);
}

static int dax_inode_init(void)
{
	int rc;

	dax_cache = kmem_cache_create("dax_cache", sizeof(struct inode), 0,
			(SLAB_HWCACHE_ALIGN|SLAB_RECLAIM_ACCOUNT|
			 SLAB_MEM_SPREAD|SLAB_ACCOUNT),
			init_once);
	if (!dax_cache)
		return -ENOMEM;

	rc = register_filesystem(&dax_type);
	if (rc)
		goto err_register_fs;

	dax_mnt = kern_mount(&dax_type);
	if (IS_ERR(dax_mnt)) {
		rc = PTR_ERR(dax_mnt);
		goto err_mount;
	}
	dax_superblock = dax_mnt->mnt_sb;

	return 0;

 err_mount:
	unregister_filesystem(&dax_type);
 err_register_fs:
	kmem_cache_destroy(dax_cache);

	return rc;
}

static void dax_inode_exit(void)
{
	kern_unmount(dax_mnt);
	unregister_filesystem(&dax_type);
	kmem_cache_destroy(dax_cache);
}

static void dax_region_free(struct kref *kref)
{
	struct dax_region *dax_region;

	dax_region = container_of(kref, struct dax_region, kref);
	WARN(atomic_read(&dax_region->child_count),
			"%s: child count not zero\n",
			dev_name(dax_region->dev));
	kfree(dax_region);
	module_put(THIS_MODULE);
}

void dax_region_put(struct dax_region *dax_region)
{
	kref_put(&dax_region->kref, dax_region_free);
}
EXPORT_SYMBOL_GPL(dax_region_put);


static void dax_region_unregister(void *region)
{
	struct dax_region *dax_region = region;

	sysfs_remove_groups(&dax_region->dev->kobj,
			dax_region_attribute_groups);
	dax_region_put(dax_region);
}

struct dax_region *alloc_dax_region(struct device *parent, int region_id,
		struct resource *res, unsigned int align, void *addr,
		unsigned long pfn_flags)
{
	struct dax_region *dax_region;

	if (dev_get_drvdata(parent)) {
		dev_WARN(parent, "dax core found drvdata already in use\n");
		return NULL;
	}

	if (!IS_ALIGNED(res->start, align)
			|| !IS_ALIGNED(resource_size(res), align))
		return NULL;

	dax_region = kzalloc(sizeof(*dax_region), GFP_KERNEL);
	if (!dax_region)
		return NULL;
	dev_set_drvdata(parent, dax_region);
	dax_region->res.name = dev_name(parent);
	dax_region->res.start = res->start;
	dax_region->res.end = res->end;
	dax_region->res.flags = IORESOURCE_MEM;
	dax_region->pfn_flags = pfn_flags;
	mutex_init(&dax_region->lock);
	kref_init(&dax_region->kref);
	dax_region->id = region_id;
	ida_init(&dax_region->ida);
	dax_region->align = align;
	dax_region->dev = parent;
	dax_region->base = addr;
	if (!try_module_get(THIS_MODULE))
		goto err_module;

	if (sysfs_create_groups(&parent->kobj, dax_region_attribute_groups))
		goto err_groups;

	kref_get(&dax_region->kref);
	if (devm_add_action_or_reset(parent, dax_region_unregister, dax_region))
		return NULL;
	return dax_region;

err_groups:
	module_put(THIS_MODULE);
err_module:
	kfree(dax_region);
	return NULL;
}
EXPORT_SYMBOL_GPL(alloc_dax_region);

static struct dax_dev *to_dax_dev(struct device *dev)
{
	return container_of(dev, struct dax_dev, dev);
}

static unsigned long long dax_dev_size(struct dax_dev *dax_dev)
{
	struct dax_region *dax_region = dax_dev->region;
	unsigned long long size = 0;
	int i;

	WARN_ON_ONCE(!mutex_is_locked(&dax_region->lock));

	if (!dax_dev->alive)
		return 0;

	for (i = 0; i < dax_dev->num_resources; i++)
		size += resource_size(dax_dev->res[i]);

	return size;
}

static ssize_t size_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned long long size;
	struct dax_dev *dax_dev = to_dax_dev(dev);
	struct dax_region *dax_region = dax_dev->region;

	/* flush previous size operations */
	async_synchronize_full_domain(&dax_dev_async);

	mutex_lock(&dax_region->lock);
	size = dax_dev_size(dax_dev);
	mutex_unlock(&dax_region->lock);

	return sprintf(buf, "%llu\n", size);
}

/*
 * Reuse the unused ->desc attribute of a dax_dev resource to store the
 * relative pgoff of the resource within the device.
 */
static unsigned long to_dev_pgoff(struct resource *res)
{
	return res->desc;
}

static void set_dev_pgoff(struct resource *res, unsigned long dev_pgoff)
{
	res->desc = dev_pgoff;
}

static unsigned order_at(struct resource *res, unsigned long pgoff)
{
	unsigned long dev_pgoff = to_dev_pgoff(res) + pgoff;
	unsigned long nr_pages = PHYS_PFN(resource_size(res));
	unsigned order_max, order_pgoff;

	if (nr_pages == pgoff)
		return UINT_MAX;

	/*
	 * What is the largest power-of-2 range available from this
	 * resource pgoff to the end of the resource range, considering
	 * the alignment of the current dev_pgoff?
	 */
	order_pgoff = ilog2(nr_pages | dev_pgoff);
	order_max = ilog2(nr_pages - pgoff);
	return min(order_max, order_pgoff);
}

#define foreach_order_pgoff(res, order, pgoff) \
	for (pgoff = 0, order = order_at((res), pgoff); order < UINT_MAX; \
		pgoff += 1UL << order, order = order_at(res, pgoff))

static int dax_dev_adjust_resource(struct dax_dev *dax_dev,
		struct resource *res, resource_size_t size)
{
	struct address_space *mapping = dax_dev->inode->i_mapping;
	unsigned long pgoff;
	int rc = 0, order;

	/*
	 * Take the lock to prevent false negative lookups while we
	 * adjust both the resource and radix entries. Note that the
	 * false *positive* lookups that are allowed by not locking when
	 * deleting full resources are permissible because we will end
	 * up invalidating those mappings before completing the resize.
	 */
	write_lock(&dax_dev->resize_lock);
	foreach_order_pgoff(res, order, pgoff)
		radix_tree_delete(&mapping->page_tree,
				to_dev_pgoff(res) + pgoff);

	adjust_resource(res, res->start, size);

	foreach_order_pgoff(res, order, pgoff) {
		rc = __radix_tree_insert(&mapping->page_tree,
				to_dev_pgoff(res) + pgoff, order, res);
		if (rc) {
			dev_WARN(&dax_dev->dev,
					"error: %d adjusting size\n", rc);
			break;
		}
	}
	write_unlock(&dax_dev->resize_lock);

	return rc;
}

static void clear_dax_dev_radix(struct dax_dev *dax_dev)
{
	struct address_space *mapping = dax_dev->inode->i_mapping;
	struct radix_tree_iter iter;
	void **slot;

	rcu_read_lock();
	radix_tree_for_each_slot(slot, &mapping->page_tree, &iter, 0) {
		struct resource *res;
		unsigned long pgoff;
		unsigned order;

		res = radix_tree_deref_slot(slot);
		if (unlikely(!res))
			continue;
		if (radix_tree_deref_retry(res)) {
			slot = radix_tree_iter_retry(&iter);
			continue;
		}

		foreach_order_pgoff(res, order, pgoff)
			radix_tree_delete(&mapping->page_tree,
					to_dev_pgoff(res) + pgoff);
	}
	rcu_read_unlock();

	synchronize_rcu();
}

static void unregister_dax_dev(void *dev)
{
	struct dax_dev *dax_dev = to_dax_dev(dev);
	struct dax_region *dax_region = dax_dev->region;
	struct cdev *cdev = &dax_dev->cdev;
	int i;

	dev_dbg(dev, "%s\n", __func__);

	/*
	 * Note, rcu is not protecting the liveness of dax_dev, rcu is
	 * ensuring that any fault handlers that might have seen
	 * dax_dev->alive == true, have completed.  Any fault handlers
	 * that start after synchronize_rcu() has started will abort
	 * upon seeing dax_dev->alive == false.
	 */
	dax_dev->alive = false;
	synchronize_rcu();
	unmap_mapping_range(dax_dev->inode->i_mapping, 0, 0, 1);

	mutex_lock(&dax_region->lock);
	clear_dax_dev_radix(dax_dev);
	for (i = 0; i < dax_dev->num_resources; i++)
		__release_region(&dax_region->res, dax_dev->res[i]->start,
				resource_size(dax_dev->res[i]));
	if (dax_region->seed == dev)
		dax_region->seed = NULL;
	mutex_unlock(&dax_region->lock);
	atomic_dec(&dax_region->child_count);

	cdev_del(cdev);
	device_unregister(dev);
}

static void dax_dev_async_unregister(void *d, async_cookie_t cookie)
{
	struct device *dev = d;
	struct dax_dev *dax_dev = to_dax_dev(dev);
	struct dax_region *dax_region = dax_dev->region;

	/*
	 * Check that we still have an enabled region, if not then this
	 * device was unregistered when the region was disabled.
	 */
	device_lock(dax_region->dev);
	if (dev_get_drvdata(dax_region->dev)) {
		devm_remove_action(dax_region->dev, unregister_dax_dev, dev);
		unregister_dax_dev(dev);
	}
	device_unlock(dax_region->dev);

	put_device(dev);
}

static int dax_dev_shrink(struct dax_region *dax_region,
		struct dax_dev *dax_dev, unsigned long long size)
{
	struct address_space *mapping = dax_dev->inode->i_mapping;
	resource_size_t dev_size = dax_dev_size(dax_dev);
	resource_size_t res_size, to_free;
	struct resource *max_res, *res;
	unsigned long pgoff;
	int i, order, rc = 0;

	to_free = dev_size - size;

retry:
	max_res = NULL;
	/* delete from the highest pgoff resource */
	for (i = 0; i < dax_dev->num_resources; i++) {
		res = dax_dev->res[i];
		if (!max_res || to_dev_pgoff(res) > to_dev_pgoff(max_res))
			max_res = res;
	}

	res = max_res;
	if (!res)
		return -ENXIO;
	res_size = resource_size(res);

	if (to_free >= res_size) {
		foreach_order_pgoff(res, order, pgoff)
			radix_tree_delete(&mapping->page_tree,
					to_dev_pgoff(res) + pgoff);
		synchronize_rcu();
		__release_region(&dax_region->res, res->start, res_size);
		for (i = 0; i < dax_dev->num_resources; i++)
			if (res == dax_dev->res[i])
				break;
		for (i = i + 1; i < dax_dev->num_resources; i++)
			dax_dev->res[i - 1] = dax_dev->res[i];
		dax_dev->num_resources--;
		to_free -= res_size;

		/*
		 * Once we've deleted a resource we need to search the
		 * next resource at the highest remaining dev_pgoff.
		 */
		if (to_free)
			goto retry;
	} else {
		rc = dax_dev_adjust_resource(dax_dev, res, res_size - to_free);
		synchronize_rcu();
	}

	/*
	 * Now that the lookup radix and resource tree has been cleaned
	 * up we can invalidate any remaining mappings in the deleted
	 * range.
	 */
	unmap_mapping_range(mapping, size, dev_size - size, 1);

	if (size == 0 && &dax_dev->dev != dax_region->seed) {
		get_device(&dax_dev->dev);
		dax_dev->alive = false;
		synchronize_rcu();
		async_schedule_domain(dax_dev_async_unregister, &dax_dev->dev,
				&dax_dev_async);
	}

	return rc;
}

static int dax_dev_add_resource(struct dax_region *dax_region,
		struct dax_dev *dax_dev, resource_size_t start,
		resource_size_t size, unsigned long dev_pgoff)
{
	struct address_space *mapping = dax_dev->inode->i_mapping;
	struct resource *res, **resources;
	int order, rc = -ENOMEM;
	unsigned long pgoff;

	res = __request_region(&dax_region->res, start, size,
			dev_name(&dax_dev->dev), 0);
	if (!res)
		return -EBUSY;
	set_dev_pgoff(res, dev_pgoff);
	resources = krealloc(dax_dev->res, sizeof(struct resource *)
			* (dax_dev->num_resources + 1), GFP_KERNEL);
	if (!resources)
		goto err_resources;
	dax_dev->res = resources;
	dax_dev->res[dax_dev->num_resources++] = res;

	foreach_order_pgoff(res, order, pgoff) {
		rc = __radix_tree_insert(&mapping->page_tree,
				to_dev_pgoff(res) + pgoff, order, res);
		if (rc)
			goto err_radix;
	}

	return 0;

err_radix:
	foreach_order_pgoff(res, order, pgoff)
		radix_tree_delete(&mapping->page_tree,
				to_dev_pgoff(res) + pgoff);
	dax_dev->res[--dax_dev->num_resources] = NULL;
err_resources:
	__release_region(&dax_region->res, start, size);
	return -ENOMEM;

}

static ssize_t dax_dev_resize(struct dax_region *dax_region,
		struct dax_dev *dax_dev, resource_size_t size)
{
	resource_size_t avail = dax_region_avail_size(dax_region), to_alloc;
	resource_size_t dev_size = dax_dev_size(dax_dev);
	struct resource *max_res = NULL, *res, *first;
	unsigned long dev_pgoff = PHYS_PFN(dev_size);
	const char *name = dev_name(&dax_dev->dev);
	resource_size_t region_end;
	int i, rc;

	if (!dax_dev->alive)
		return -ENXIO;

	if (size == dev_size)
		return 0;
	if (size > dev_size && size - dev_size > avail)
		return -ENOSPC;

	if (size < dev_size)
		return dax_dev_shrink(dax_region, dax_dev, size);

	to_alloc = size - dev_size;
	if (!IS_ALIGNED(to_alloc, dax_region->align)) {
		WARN_ON(1);
		return -ENXIO;
	}

	for (i = 0; i < dax_dev->num_resources; i++) {
		res = dax_dev->res[i];
		if (!max_res || to_dev_pgoff(res) > to_dev_pgoff(max_res))
			max_res = res;
	}

	/*
	 * Expand the device into the unused portion of the region. This
	 * may involve adjusting the end of an existing resource, or
	 * allocating a new disjoint resource.
	 */
	region_end = dax_region->res.start + resource_size(&dax_region->res);
	first = dax_region->res.child;
	for (res = first; to_alloc && res; res = res->sibling) {
		struct resource *next = res->sibling;
		resource_size_t alloc, res_end;

		res_end = res->start + resource_size(res);

		/* space at the beginning of the region */
		if (res == first && res->start > dax_region->res.start) {
			alloc = res->start - dax_region->res.start;
			alloc = min(alloc, to_alloc);
			rc = dax_dev_add_resource(dax_region, dax_dev,
					dax_region->res.start, alloc,
					dev_pgoff);
			if (rc)
				return rc;
			to_alloc -= alloc;
			dev_pgoff += PHYS_PFN(alloc);
		}

		/* space between allocations */
		if (to_alloc && next && next->start > res_end) {
			alloc = next->start - res_end;
			alloc = min(alloc, to_alloc);
			if (res == max_res && strcmp(name, res->name) == 0)
				rc = dax_dev_adjust_resource(dax_dev, res,
						resource_size(res) + alloc);
			else
				rc = dax_dev_add_resource(dax_region, dax_dev,
						res_end, alloc, dev_pgoff);
			if (rc)
				return rc;
			to_alloc -= alloc;
			dev_pgoff += PHYS_PFN(alloc);
		}

		/* space at the end of the region */
		if (to_alloc && !next && res_end < region_end) {
			alloc = region_end - res_end;
			alloc = min(alloc, to_alloc);
			if (res == max_res && strcmp(name, res->name) == 0)
				rc = dax_dev_adjust_resource(dax_dev, res,
						resource_size(res) + alloc);
			else
				rc = dax_dev_add_resource(dax_region, dax_dev,
						res_end, alloc, dev_pgoff);
			if (rc)
				return rc;
			to_alloc -= alloc;
			dev_pgoff += PHYS_PFN(alloc);
		}
	}

	device_lock(dax_region->dev);
	if (dev_get_drvdata(dax_region->dev) && dev_size == 0
			&& &dax_dev->dev == dax_region->seed) {
		struct dax_dev *seed;

		seed = devm_create_dax_dev(dax_region, NULL, 0);
		if (IS_ERR(seed))
			dev_warn(dax_region->dev,
					"failed to create new region seed\n");
		else
			dax_region->seed = &seed->dev;
	}
	device_unlock(dax_region->dev);

	return 0;
}

static ssize_t size_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	ssize_t rc;
	unsigned long long val;
	struct dax_dev *dax_dev = to_dax_dev(dev);
	struct dax_region *dax_region = dax_dev->region;

	rc = kstrtoull(buf, 0, &val);
	if (rc)
		return rc;

	if (!IS_ALIGNED(val, dax_region->align)) {
		dev_dbg(&dax_dev->dev, "%s: size: %lld misaligned\n",
				__func__, val);
		return -EINVAL;
	}

	mutex_lock(&dax_region->lock);
	rc = dax_dev_resize(dax_region, dax_dev, val);
	mutex_unlock(&dax_region->lock);

	if (rc == 0)
		return len;

	return rc;
}
static DEVICE_ATTR_RW(size);

static struct attribute *dax_device_attributes[] = {
	&dev_attr_size.attr,
	NULL,
};

static const struct attribute_group dax_device_attribute_group = {
	.attrs = dax_device_attributes,
};

static const struct attribute_group *dax_attribute_groups[] = {
	&dax_device_attribute_group,
	NULL,
};

static int check_vma(struct dax_dev *dax_dev, struct vm_area_struct *vma,
		const char *func)
{
	struct dax_region *dax_region = dax_dev->region;
	struct device *dev = &dax_dev->dev;
	unsigned long mask;

	if (!dax_dev->alive)
		return -ENXIO;

	/* prevent private mappings from being established */
	if ((vma->vm_flags & VM_SHARED) != VM_SHARED) {
		dev_info(dev, "%s: %s: fail, attempted private mapping\n",
				current->comm, func);
		return -EINVAL;
	}

	mask = dax_region->align - 1;
	if (vma->vm_start & mask || vma->vm_end & mask) {
		dev_info(dev, "%s: %s: fail, unaligned vma (%#lx - %#lx, %#lx)\n",
				current->comm, func, vma->vm_start, vma->vm_end,
				mask);
		return -EINVAL;
	}

	if ((dax_region->pfn_flags & (PFN_DEV|PFN_MAP)) == PFN_DEV
			&& (vma->vm_flags & VM_DONTCOPY) == 0) {
		dev_info(dev, "%s: %s: fail, dax range requires MADV_DONTFORK\n",
				current->comm, func);
		return -EINVAL;
	}

	if (!vma_is_dax(vma)) {
		dev_info(dev, "%s: %s: fail, vma is not DAX capable\n",
				current->comm, func);
		return -EINVAL;
	}

	return 0;
}

static phys_addr_t __pgoff_to_phys(struct dax_dev *dax_dev, pgoff_t pgoff,
		unsigned long size)
{
	struct address_space *mapping = dax_dev->inode->i_mapping;
	phys_addr_t res_offset;
	struct resource *res;

	res = radix_tree_lookup(&mapping->page_tree, pgoff);
	if (!res)
		return -1;
	res_offset = PFN_PHYS(pgoff - to_dev_pgoff(res));
	if (res_offset + size >= resource_size(res))
		return -1;
	return res->start + res_offset;
}

static phys_addr_t pgoff_to_phys(struct dax_dev *dax_dev, pgoff_t pgoff,
                unsigned long size)
{
	phys_addr_t phys;

	read_lock(&dax_dev->resize_lock);
	phys = __pgoff_to_phys(dax_dev, pgoff, size);
	read_unlock(&dax_dev->resize_lock);

	return phys;
}

static int __dax_dev_fault(struct dax_dev *dax_dev, struct vm_area_struct *vma,
		struct vm_fault *vmf)
{
	unsigned long vaddr = (unsigned long) vmf->virtual_address;
	struct device *dev = &dax_dev->dev;
	struct dax_region *dax_region;
	int rc = VM_FAULT_SIGBUS;
	phys_addr_t phys;
	pfn_t pfn;

	if (check_vma(dax_dev, vma, __func__))
		return VM_FAULT_SIGBUS;

	dax_region = dax_dev->region;
	if (dax_region->align > PAGE_SIZE) {
		dev_dbg(dev, "%s: alignment > fault size\n", __func__);
		return VM_FAULT_SIGBUS;
	}

	phys = pgoff_to_phys(dax_dev, vmf->pgoff, PAGE_SIZE);
	if (phys == -1) {
		dev_dbg(dev, "%s: phys_to_pgoff(%#lx) failed\n", __func__,
				vmf->pgoff);
		return VM_FAULT_SIGBUS;
	}

	pfn = phys_to_pfn_t(phys, dax_region->pfn_flags);

	rc = vm_insert_mixed(vma, vaddr, pfn);

	if (rc == -ENOMEM)
		return VM_FAULT_OOM;
	if (rc < 0 && rc != -EBUSY)
		return VM_FAULT_SIGBUS;

	return VM_FAULT_NOPAGE;
}

static int dax_dev_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	int rc;
	struct file *filp = vma->vm_file;
	struct dax_dev *dax_dev = filp->private_data;

	dev_dbg(&dax_dev->dev, "%s: %s: %s (%#lx - %#lx)\n", __func__,
			current->comm, (vmf->flags & FAULT_FLAG_WRITE)
			? "write" : "read", vma->vm_start, vma->vm_end);
	rcu_read_lock();
	rc = __dax_dev_fault(dax_dev, vma, vmf);
	rcu_read_unlock();

	return rc;
}

static int __dax_dev_pmd_fault(struct dax_dev *dax_dev,
		struct vm_area_struct *vma, unsigned long addr, pmd_t *pmd,
		unsigned int flags)
{
	unsigned long pmd_addr = addr & PMD_MASK;
	struct device *dev = &dax_dev->dev;
	struct dax_region *dax_region;
	phys_addr_t phys;
	pgoff_t pgoff;
	pfn_t pfn;

	if (check_vma(dax_dev, vma, __func__))
		return VM_FAULT_SIGBUS;

	dax_region = dax_dev->region;
	if (dax_region->align > PMD_SIZE) {
		dev_dbg(dev, "%s: alignment > fault size\n", __func__);
		return VM_FAULT_SIGBUS;
	}

	/* dax pmd mappings require pfn_t_devmap() */
	if ((dax_region->pfn_flags & (PFN_DEV|PFN_MAP)) != (PFN_DEV|PFN_MAP)) {
		dev_dbg(dev, "%s: alignment > fault size\n", __func__);
		return VM_FAULT_SIGBUS;
	}

	pgoff = linear_page_index(vma, pmd_addr);
	phys = pgoff_to_phys(dax_dev, pgoff, PMD_SIZE);
	if (phys == -1) {
		dev_dbg(dev, "%s: phys_to_pgoff(%#lx) failed\n", __func__,
				pgoff);
		return VM_FAULT_SIGBUS;
	}

	pfn = phys_to_pfn_t(phys, dax_region->pfn_flags);

	return vmf_insert_pfn_pmd(vma, addr, pmd, pfn,
			flags & FAULT_FLAG_WRITE);
}

static int dax_dev_pmd_fault(struct vm_area_struct *vma, unsigned long addr,
		pmd_t *pmd, unsigned int flags)
{
	int rc;
	struct file *filp = vma->vm_file;
	struct dax_dev *dax_dev = filp->private_data;

	dev_dbg(&dax_dev->dev, "%s: %s: %s (%#lx - %#lx)\n", __func__,
			current->comm, (flags & FAULT_FLAG_WRITE)
			? "write" : "read", vma->vm_start, vma->vm_end);

	rcu_read_lock();
	rc = __dax_dev_pmd_fault(dax_dev, vma, addr, pmd, flags);
	rcu_read_unlock();

	return rc;
}

static const struct vm_operations_struct dax_dev_vm_ops = {
	.fault = dax_dev_fault,
	.pmd_fault = dax_dev_pmd_fault,
};

static int dax_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct dax_dev *dax_dev = filp->private_data;
	int rc;

	dev_dbg(&dax_dev->dev, "%s\n", __func__);

	rc = check_vma(dax_dev, vma, __func__);
	if (rc)
		return rc;

	vma->vm_ops = &dax_dev_vm_ops;
	vma->vm_flags |= VM_MIXEDMAP | VM_HUGEPAGE;
	return 0;
}

/* return an unmapped area aligned to the dax region specified alignment */
static unsigned long dax_get_unmapped_area(struct file *filp,
		unsigned long addr, unsigned long len, unsigned long pgoff,
		unsigned long flags)
{
	unsigned long off, off_end, off_align, len_align, addr_align, align;
	struct dax_dev *dax_dev = filp ? filp->private_data : NULL;
	struct dax_region *dax_region;

	if (!dax_dev || addr)
		goto out;

	dax_region = dax_dev->region;
	align = dax_region->align;
	off = pgoff << PAGE_SHIFT;
	off_end = off + len;
	off_align = round_up(off, align);

	if ((off_end <= off_align) || ((off_end - off_align) < align))
		goto out;

	len_align = len + align;
	if ((off + len_align) < off)
		goto out;

	addr_align = current->mm->get_unmapped_area(filp, addr, len_align,
			pgoff, flags);
	if (!IS_ERR_VALUE(addr_align)) {
		addr_align += (off - addr_align) & (align - 1);
		return addr_align;
	}
 out:
	return current->mm->get_unmapped_area(filp, addr, len, pgoff, flags);
}

static int dax_open(struct inode *inode, struct file *filp)
{
	struct dax_dev *dax_dev;

	dax_dev = container_of(inode->i_cdev, struct dax_dev, cdev);
	dev_dbg(&dax_dev->dev, "%s\n", __func__);
	inode->i_mapping = dax_dev->inode->i_mapping;
	inode->i_mapping->host = dax_dev->inode;
	filp->f_mapping = inode->i_mapping;
	filp->private_data = dax_dev;
	inode->i_flags = S_DAX;

	return 0;
}

static int dax_release(struct inode *inode, struct file *filp)
{
	struct dax_dev *dax_dev = filp->private_data;

	dev_dbg(&dax_dev->dev, "%s\n", __func__);
	return 0;
}

static const struct file_operations dax_fops = {
	.llseek = noop_llseek,
	.owner = THIS_MODULE,
	.open = dax_open,
	.release = dax_release,
	.get_unmapped_area = dax_get_unmapped_area,
	.mmap = dax_mmap,
};

static void dax_dev_release(struct device *dev)
{
	struct dax_dev *dax_dev = to_dax_dev(dev);
	struct dax_region *dax_region = dax_dev->region;

	ida_simple_remove(&dax_region->ida, dax_dev->id);
	ida_simple_remove(&dax_minor_ida, MINOR(dev->devt));
	dax_region_put(dax_region);
	iput(dax_dev->inode);
	kfree(dax_dev->res);
	kfree(dax_dev);
}

struct dax_dev *devm_create_dax_dev(struct dax_region *dax_region,
		struct resource *res, int count)
{
	struct device *parent = dax_region->dev;
	struct dax_dev *dax_dev;
	int rc = 0, minor, i;
	unsigned long pgoff;
	struct device *dev;
	struct cdev *cdev;
	dev_t dev_t;

	dax_dev = kzalloc(sizeof(*dax_dev), GFP_KERNEL);
	if (!dax_dev)
		return ERR_PTR(-ENOMEM);

	dax_dev->res = kzalloc(sizeof(res) * count, GFP_KERNEL);
	if (!dax_dev->res)
		goto err_res;

	for (i = 0; i < count; i++) {
		struct resource *dax_res;

		if (!IS_ALIGNED(res[i].start, dax_region->align)
				|| !IS_ALIGNED(resource_size(&res[i]),
					dax_region->align)) {
			rc = -EINVAL;
			break;
		}

		mutex_lock(&dax_region->lock);
		dax_res = __request_region(&dax_region->res, res[i].start,
				resource_size(&res[i]), NULL, 0);
		mutex_unlock(&dax_region->lock);
		if (!dax_res) {
			rc = -EBUSY;
			break;
		}
		dax_dev->res[i] = dax_res;
	}

	if (i < count)
		goto err_request_region;

	dax_dev->id = ida_simple_get(&dax_region->ida, 0, 0, GFP_KERNEL);
	if (dax_dev->id < 0) {
		rc = dax_dev->id;
		goto err_request_region;
	}

	minor = ida_simple_get(&dax_minor_ida, 0, 0, GFP_KERNEL);
	if (minor < 0) {
		rc = minor;
		goto err_minor;
	}

	dev_t = MKDEV(MAJOR(dax_devt), minor);
	dev = &dax_dev->dev;
	dax_dev->inode = dax_inode_get(&dax_dev->cdev, dev_t);
	if (!dax_dev->inode) {
		rc = -ENOMEM;
		goto err_inode;
	}

	for (i = 0, pgoff = 0; i < count; i++) {
		struct address_space *mapping = dax_dev->inode->i_mapping;
		struct resource *dax_res;
		int order;

		dax_res = dax_dev->res[i];
		set_dev_pgoff(dax_res, pgoff);
		mutex_lock(&dax_region->lock);
		foreach_order_pgoff(dax_res, order, pgoff) {
			rc = __radix_tree_insert(&mapping->page_tree,
					to_dev_pgoff(dax_res) + pgoff, order,
					dax_res);
			if (rc)
				break;
		}
		mutex_unlock(&dax_region->lock);
		pgoff = to_dev_pgoff(dax_res) + PHYS_PFN(resource_size(dax_res));

		if (rc)
			goto err_radix_insert;
	}

	/* device_initialize() so cdev can reference kobj parent */
	device_initialize(dev);

	cdev = &dax_dev->cdev;
	cdev_init(cdev, &dax_fops);
	cdev->owner = parent->driver->owner;
	cdev->kobj.parent = &dev->kobj;
	rc = cdev_add(&dax_dev->cdev, dev_t, 1);
	if (rc)
		goto err_cdev;

	/* from here on we're committed to teardown via dax_dev_release() */
	dax_dev->num_resources = count;
	dax_dev->alive = true;
	dax_dev->region = dax_region;
	rwlock_init(&dax_dev->resize_lock);
	kref_get(&dax_region->kref);

	dev->devt = dev_t;
	dev->class = dax_class;
	dev->parent = parent;
	dev->groups = dax_attribute_groups;
	dev->release = dax_dev_release;
	dev_set_name(dev, "dax%d.%d", dax_region->id, dax_dev->id);
	/* update resource names now that the owner device is named */
	for (i = 0; i < dax_dev->num_resources; i++)
		dax_dev->res[i]->name = dev_name(dev);

	rc = device_add(dev);
	if (rc) {
		put_device(dev);
		return ERR_PTR(rc);
	}

	rc = devm_add_action_or_reset(dax_region->dev, unregister_dax_dev, dev);
	if (rc)
		return ERR_PTR(rc);

	if (atomic_inc_return(&dax_region->child_count) == 1) {
		struct dax_dev *seed;

		seed = devm_create_dax_dev(dax_region, NULL, 0);
		if (IS_ERR(seed))
			dev_warn(parent, "failed to create region seed\n");
		else
			dax_region->seed = &seed->dev;
	}

	return dax_dev;

 err_cdev:
	mutex_lock(&dax_region->lock);
	clear_dax_dev_radix(dax_dev);
	mutex_unlock(&dax_region->lock);
 err_radix_insert:
	iput(dax_dev->inode);
 err_inode:
	ida_simple_remove(&dax_minor_ida, minor);
 err_minor:
	ida_simple_remove(&dax_region->ida, dax_dev->id);
 err_request_region:
	mutex_lock(&dax_region->lock);
	for (i--; i >= 0; i--)
		__release_region(&dax_region->res, dax_dev->res[i]->start,
				resource_size(dax_dev->res[i]));
	mutex_unlock(&dax_region->lock);
	kfree(dax_dev->res);
 err_res:
	kfree(dax_dev);

	return ERR_PTR(rc);
}
EXPORT_SYMBOL_GPL(devm_create_dax_dev);

static int __init dax_init(void)
{
	int rc;

	rc = dax_inode_init();
	if (rc)
		return rc;

	nr_dax = max(nr_dax, 256);
	rc = alloc_chrdev_region(&dax_devt, 0, nr_dax, "dax");
	if (rc)
		goto err_chrdev;

	dax_class = class_create(THIS_MODULE, "dax");
	if (IS_ERR(dax_class)) {
		rc = PTR_ERR(dax_class);
		goto err_class;
	}

	return 0;

 err_class:
	unregister_chrdev_region(dax_devt, nr_dax);
 err_chrdev:
	dax_inode_exit();
	return rc;
}

static void __exit dax_exit(void)
{
	class_destroy(dax_class);
	unregister_chrdev_region(dax_devt, nr_dax);
	ida_destroy(&dax_minor_ida);
	dax_inode_exit();
}

MODULE_AUTHOR("Intel Corporation");
MODULE_LICENSE("GPL v2");
subsys_initcall(dax_init);
module_exit(dax_exit);
