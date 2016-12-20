#include <linux/bitmap.h>
#include <linux/export.h>
#include <linux/idr.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

static DEFINE_SPINLOCK(simple_ida_lock);

/**
 * idr_alloc - allocate an id
 * @idr: idr handle
 * @ptr: pointer to be associated with the new id
 * @start: the minimum id (inclusive)
 * @end: the maximum id (exclusive)
 * @gfp: memory allocation flags
 *
 * Allocates an unused ID in the range [start, end).  Returns -ENOSPC
 * if there are no unused IDs in that range.
 *
 * Note that @end is treated as max when <= 0.  This is to always allow
 * using @start + N as @end as long as N is inside integer range.
 *
 * Simultaneous modifications to the @idr are not allowed and should be
 * prevented by the user, usually with a lock.  idr_alloc() may be called
 * concurrently with read-only accesses to the @idr, such as idr_find() and
 * idr_for_each_entry().
 */
int idr_alloc(struct idr *idr, void *ptr, int start, int end, gfp_t gfp)
{
	void **slot;
	struct radix_tree_iter iter;

	if (WARN_ON_ONCE(start < 0))
		return -EINVAL;
	BUG_ON(radix_tree_is_internal_node(ptr));

	radix_tree_iter_init(&iter, start);
	slot = idr_get_empty(&idr->idr_rt, &iter, gfp, end);
	if (IS_ERR(slot))
		return PTR_ERR(slot);

	radix_tree_iter_replace(&idr->idr_rt, &iter, slot, ptr);
	radix_tree_iter_tag_clear(&idr->idr_rt, &iter, IDR_FREE);
	return iter.index;
}
EXPORT_SYMBOL_GPL(idr_alloc);

/**
 * idr_alloc_cyclic - allocate new idr entry in a cyclical fashion
 * @idr: idr handle
 * @ptr: pointer to be associated with the new id
 * @start: the minimum id (inclusive)
 * @end: the maximum id (exclusive)
 * @gfp: memory allocation flags
 *
 * Allocates an ID larger than the last ID allocated if one is available.
 * If not, it will attempt to allocate the smallest ID that is larger or
 * equal to @start.
 */
int idr_alloc_cyclic(struct idr *idr, void *ptr, int start, int end, gfp_t gfp)
{
	int id, curr = idr->idr_next;

	if (curr < start)
		curr = start;

	id = idr_alloc(idr, ptr, curr, end, gfp);
	if ((id == -ENOSPC) && (curr > start))
		id = idr_alloc(idr, ptr, start, curr, gfp);

	if (id >= 0)
		idr->idr_next = id + 1U;

	return id;
}
EXPORT_SYMBOL(idr_alloc_cyclic);

/**
 * idr_for_each - iterate through all stored pointers
 * @idr: idr handle
 * @fn: function to be called for each pointer
 * @data: data passed to callback function
 *
 * The callback function will be called for each entry in @idr, passing
 * the id, the pointer and the data pointer passed to this function.
 *
 * If @fn returns anything other than %0, the iteration stops and that
 * value is returned from this function.
 *
 * idr_for_each() can be called concurrently with idr_alloc() and
 * idr_remove() if protected by RCU.  Newly added entries may not be
 * seen and deleted entries may be seen, but adding and removing entries
 * will not cause other entries to be skipped, nor spurious ones to be seen.
 */
int idr_for_each(struct idr *idr,
		int (*fn)(int id, void *p, void *data), void *data)
{
	struct radix_tree_iter iter;
	void **slot;

	radix_tree_for_each_slot(slot, &idr->idr_rt, &iter, 0) {
		int ret = fn(iter.index, *slot, data);
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL(idr_for_each);

/**
 * idr_get_next - Find next populated entry
 * @idr: idr handle
 * @nextid: Pointer to lowest possible ID to return
 *
 * Returns the next populated entry in the tree with an ID greater than
 * or equal to the value pointed to by @nextid.  On exit, @nextid is updated
 * to the ID of the found value.  To use in a loop, the value pointed to by
 * nextid must be incremented by the user.
 */
void *idr_get_next(struct idr *idr, int *nextid)
{
	struct radix_tree_iter iter;
	void **slot;

	slot = radix_tree_iter_find(&idr->idr_rt, &iter, *nextid);
	if (!slot)
		return NULL;

	*nextid = iter.index;
	return *slot;
}
EXPORT_SYMBOL(idr_get_next);

/**
 * idr_replace - replace pointer for given id
 * @idr: idr handle
 * @ptr: New pointer to associate with the ID
 * @id: Lookup key
 *
 * Replace the pointer registered with an id and return the old value.
 * A %-ENOENT return indicates that @id was not found.
 * A %-EINVAL return indicates that @id was not within valid constraints.
 *
 * This function can be called under the RCU read lock concurrently with
 * idr_remove().
 */
void *idr_replace(struct idr *idr, void *ptr, int id)
{
	struct radix_tree_node *node;
	void **slot;
	void *entry;

	if (id < 0)
		return ERR_PTR(-EINVAL);
	if (!ptr || radix_tree_is_internal_node(ptr))
		return ERR_PTR(-EINVAL);

	entry = __radix_tree_lookup(&idr->idr_rt, id, &node, &slot);

	if (!entry)
		return ERR_PTR(-ENOENT);

	__radix_tree_replace(&idr->idr_rt, node, slot, ptr, NULL, NULL);

	return entry;
}
EXPORT_SYMBOL(idr_replace);

/**
 * ida_pre_get - reserve resources for ida allocation
 * @ida: ida handle
 * @gfp: memory allocation flags
 *
 * This function should be called before calling ida_get_new_above().  If it
 * is unable to allocate memory, it will return %0.  On success, it returns %1.
 */
int ida_pre_get(struct ida *ida, gfp_t gfp)
{
	struct ida_bitmap *bitmap;

	/*
	 * This looks weird, but the IDA API has no preload_end() equivalent.
	 * Instead, ida_get_new() can return -EAGAIN, prompting the caller
	 * to return to the ida_pre_get() step.
	 */
	idr_preload(gfp);
	idr_preload_end();

	if (!ida->free_bitmap) {
		bitmap = kmalloc(sizeof(struct ida_bitmap), gfp);
		if (!bitmap)
			return 0;
		bitmap = xchg(&ida->free_bitmap, bitmap);
		kfree(bitmap);
	}

	return 1;
}
EXPORT_SYMBOL(ida_pre_get);

#define IDA_MAX (0x80000000U / IDA_BITMAP_BITS)

/**
 * ida_get_new_above - allocate new ID above or equal to a start id
 * @ida: ida handle
 * @starting_id: id to start search at
 * @p_id: pointer to the allocated handle
 *
 * Allocate new ID above or equal to @starting_id.  It should be called
 * with any required locks to ensure that concurrent calls to
 * ida_get_new_above() / ida_get_new() / ida_remove() are not allowed.
 * Consider using ida_simple_get() if you do not have complex locking
 * requirements.
 *
 * If memory is required, it will return %-EAGAIN, you should unlock
 * and go back to the ida_pre_get() call.  If the ida is full, it will
 * return %-ENOSPC.
 *
 * @p_id returns a value in the range @starting_id ... %0x7fffffff.
 */
int ida_get_new_above(struct ida *ida, int start, int *id)
{
	struct radix_tree_root *root = &ida->ida_rt;
	void **slot;
	struct radix_tree_iter iter;
	struct ida_bitmap *bitmap;
	unsigned long index;
	unsigned bit;
	int new;

	index = start / IDA_BITMAP_BITS;
	bit = start % IDA_BITMAP_BITS;

	slot = radix_tree_iter_init(&iter, index);
	for (;;) {
		if (slot)
			slot = radix_tree_next_slot(slot, &iter,
						RADIX_TREE_ITER_TAGGED);
		if (!slot) {
			slot = idr_get_empty(root, &iter, GFP_NOWAIT, IDA_MAX);
			if (IS_ERR(slot)) {
				if (slot == ERR_PTR(-ENOMEM))
					return -EAGAIN;
				return PTR_ERR(slot);
			}
		}
		if (iter.index > index)
			bit = 0;
		new = iter.index * IDA_BITMAP_BITS;
		bitmap = *slot;
		if (bitmap) {
			bit = find_next_zero_bit(bitmap->bitmap,
							IDA_BITMAP_BITS, bit);
			new += bit;
			if (new < 0)
				return -ENOSPC;
			if (bit == IDA_BITMAP_BITS)
				continue;

			__set_bit(bit, bitmap->bitmap);
			if (bitmap_full(bitmap->bitmap, IDA_BITMAP_BITS))
				radix_tree_iter_tag_clear(root, &iter,
								IDR_FREE);
		} else {
			new += bit;
			if (new < 0)
				return -ENOSPC;
			bitmap = ida->free_bitmap;
			if (!bitmap)
				return -EAGAIN;
			ida->free_bitmap = NULL;
			memset(bitmap, 0, sizeof(*bitmap));
			__set_bit(bit, bitmap->bitmap);
			radix_tree_iter_replace(root, &iter, slot, bitmap);
		}

		*id = new;
		return 0;
	}
}
EXPORT_SYMBOL(ida_get_new_above);

/**
 * ida_remove - Free the given ID
 * @ida: ida handle
 * @id: ID to free
 *
 * This function should not be called at the same time as ida_get_new_above().
 */
void ida_remove(struct ida *ida, int id)
{
	unsigned long index = id / IDA_BITMAP_BITS;
	unsigned offset = id % IDA_BITMAP_BITS;
	struct ida_bitmap *bitmap;
	struct radix_tree_iter iter;
	void **slot;

	slot = radix_tree_iter_lookup(&ida->ida_rt, &iter, index);
	if (!slot)
		goto err;

	bitmap = *slot;
	if (!test_bit(offset, bitmap->bitmap))
		goto err;

	__clear_bit(offset, bitmap->bitmap);
	radix_tree_iter_tag_set(&ida->ida_rt, &iter, IDR_FREE);
	if (bitmap_empty(bitmap->bitmap, IDA_BITMAP_BITS)) {
		kfree(bitmap);
		radix_tree_iter_delete(&ida->ida_rt, &iter);
	}
	return;
 err:
	WARN(1, "ida_remove called for id=%d which is not allocated.\n", id);
}
EXPORT_SYMBOL(ida_remove);

/**
 * ida_destroy - Free the contents of an ida
 * @ida: ida handle
 *
 * Calling this function releases all resources associated with an IDA.  When
 * this call returns, the IDA is empty and can be reused or freed.  The caller
 * should not allow ida_remove() or ida_get_new_above() to be called at the
 * same time.
 */
void ida_destroy(struct ida *ida)
{
	struct radix_tree_iter iter;
	void **slot;

	radix_tree_for_each_slot(slot, &ida->ida_rt, &iter, 0) {
		struct ida_bitmap *bitmap = *slot;
		kfree(bitmap);
		radix_tree_iter_delete(&ida->ida_rt, &iter);
	}

	kfree(ida->free_bitmap);
	ida->free_bitmap = NULL;
}
EXPORT_SYMBOL(ida_destroy);

/**
 * ida_simple_get - get a new id.
 * @ida: the (initialized) ida.
 * @start: the minimum id (inclusive, < 0x8000000)
 * @end: the maximum id (exclusive, < 0x8000000 or 0)
 * @gfp_mask: memory allocation flags
 *
 * Allocates an id in the range start <= id < end, or returns -ENOSPC.
 * On memory allocation failure, returns -ENOMEM.
 *
 * Compared to ida_get_new_above() this function does its own locking, and
 * should be used unless there are special requirements.
 *
 * Use ida_simple_remove() to get rid of an id.
 */
int ida_simple_get(struct ida *ida, unsigned int start, unsigned int end,
		   gfp_t gfp_mask)
{
	int ret, id;
	unsigned int max;
	unsigned long flags;

	BUG_ON((int)start < 0);
	BUG_ON((int)end < 0);

	if (end == 0)
		max = 0x80000000;
	else {
		BUG_ON(end < start);
		max = end - 1;
	}

again:
	if (!ida_pre_get(ida, gfp_mask))
		return -ENOMEM;

	spin_lock_irqsave(&simple_ida_lock, flags);
	ret = ida_get_new_above(ida, start, &id);
	if (!ret) {
		if (id > max) {
			ida_remove(ida, id);
			ret = -ENOSPC;
		} else {
			ret = id;
		}
	}
	spin_unlock_irqrestore(&simple_ida_lock, flags);

	if (unlikely(ret == -EAGAIN))
		goto again;

	return ret;
}
EXPORT_SYMBOL(ida_simple_get);

/**
 * ida_simple_remove - remove an allocated id.
 * @ida: the (initialized) ida.
 * @id: the id returned by ida_simple_get.
 *
 * Use to release an id allocated with ida_simple_get().
 *
 * Compared to ida_remove() this function does its own locking, and should be
 * used unless there are special requirements.
 */
void ida_simple_remove(struct ida *ida, unsigned int id)
{
	unsigned long flags;

	BUG_ON((int)id < 0);
	spin_lock_irqsave(&simple_ida_lock, flags);
	ida_remove(ida, id);
	spin_unlock_irqrestore(&simple_ida_lock, flags);
}
EXPORT_SYMBOL(ida_simple_remove);
