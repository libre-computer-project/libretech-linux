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
	int c, old;
	c = arch_atomic_read(v);
	for (;;) {
		if (unlikely(c == (u)))
			break;
		old = arch_atomic_cmpxchg((v), c, c + (a));
		if (likely(old == c))
			break;
		c = old;
	}
	return c;
}
EXPORT_SYMBOL(__arch_atomic_add_unless);
