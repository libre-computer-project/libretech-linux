#include <linux/idr.h>
#include <linux/spinlock.h>

static DEFINE_SPINLOCK(simple_ida_lock);

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
