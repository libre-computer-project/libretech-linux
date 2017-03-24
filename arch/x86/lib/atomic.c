#include <linux/module.h>
#include <asm/atomic.h>

/**
 * __arch_atomic_add_unless - add unless the number is already a given value
 * @v: pointer of type atomic_t
 * @a: the amount to add to v...
 * @u: ...unless v is equal to u.
 *
 * Atomically adds @a to @v, so long as @v was not already @u.
 * Returns the old value of @v.
 */
int __arch_atomic_add_unless(atomic_t *v, int a, int u)
{
	int c = arch_atomic_read(v);
	do {
		if (unlikely(c == u))
			break;
	} while (!atomic_try_cmpxchg(v, &c, c + a));
	return c;
}
EXPORT_SYMBOL(__arch_atomic_add_unless);
