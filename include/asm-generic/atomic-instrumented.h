/*
 * This file provides wrappers with KASAN instrumentation for atomic operations.
 * To use this functionality an arch's atomic.h file needs to define all
 * atomic operations with arch_ prefix (e.g. arch_atomic_read()) and include
 * this file at the end. This file provides atomic_read() that forwards to
 * arch_atomic_read() for actual atomic operation.
 * Note: if an arch atomic operation is implemented by means of other atomic
 * operations (e.g. atomic_read()/atomic_cmpxchg() loop), then it needs to use
 * arch_ variants (i.e. arch_atomic_read()/arch_atomic_cmpxchg()) to avoid
 * double instrumentation.
 */
#ifndef _LINUX_ATOMIC_INSTRUMENTED_H
#define _LINUX_ATOMIC_INSTRUMENTED_H

#include <linux/kasan-checks.h>

static __always_inline int atomic_read(const atomic_t *v)
{
	kasan_check_read(v, sizeof(*v));
	return arch_atomic_read(v);
}

static __always_inline long long atomic64_read(const atomic64_t *v)
{
	kasan_check_read(v, sizeof(*v));
	return arch_atomic64_read(v);
}


static __always_inline void atomic_set(atomic_t *v, int i)
{
	kasan_check_write(v, sizeof(*v));
	arch_atomic_set(v, i);
}

static __always_inline void atomic64_set(atomic64_t *v, long long i)
{
	kasan_check_write(v, sizeof(*v));
	arch_atomic64_set(v, i);
}

static __always_inline int atomic_xchg(atomic_t *v, int i)
{
	kasan_check_write(v, sizeof(*v));
	return arch_atomic_xchg(v, i);
}

static __always_inline long long atomic64_xchg(atomic64_t *v, long long i)
{
	kasan_check_write(v, sizeof(*v));
	return arch_atomic64_xchg(v, i);
}

static __always_inline int atomic_cmpxchg(atomic_t *v, int old, int new)
{
	kasan_check_write(v, sizeof(*v));
	return arch_atomic_cmpxchg(v, old, new);
}

static __always_inline long long atomic64_cmpxchg(atomic64_t *v, long long old,
						  long long new)
{
	kasan_check_write(v, sizeof(*v));
	return arch_atomic64_cmpxchg(v, old, new);
}

static __always_inline int __atomic_add_unless(atomic_t *v, int a, int u)
{
	kasan_check_write(v, sizeof(*v));
	return __arch_atomic_add_unless(v, a, u);
}


static __always_inline bool atomic64_add_unless(atomic64_t *v, long long a,
						long long u)
{
	kasan_check_write(v, sizeof(*v));
	return arch_atomic64_add_unless(v, a, u);
}

static __always_inline short int atomic_inc_short(short int *v)
{
	kasan_check_write(v, sizeof(*v));
	return arch_atomic_inc_short(v);
}

#define __INSTR_VOID1(op, sz)						\
static __always_inline void atomic##sz##_##op(atomic##sz##_t *v)	\
{									\
	kasan_check_write(v, sizeof(*v));				\
	arch_atomic##sz##_##op(v);					\
}

#define INSTR_VOID1(op)	\
__INSTR_VOID1(op,);	\
__INSTR_VOID1(op, 64)

INSTR_VOID1(inc);
INSTR_VOID1(dec);

#undef __INSTR_VOID1
#undef INSTR_VOID1

#define __INSTR_VOID2(op, sz, type)					\
static __always_inline void atomic##sz##_##op(type i, atomic##sz##_t *v)\
{									\
	kasan_check_write(v, sizeof(*v));				\
	arch_atomic##sz##_##op(i, v);					\
}

#define INSTR_VOID2(op)		\
__INSTR_VOID2(op, , int);	\
__INSTR_VOID2(op, 64, long long)

INSTR_VOID2(add);
INSTR_VOID2(sub);
INSTR_VOID2(and);
INSTR_VOID2(or);
INSTR_VOID2(xor);

#undef __INSTR_VOID2
#undef INSTR_VOID2

#define __INSTR_RET1(op, sz, type, rtype)				\
static __always_inline rtype atomic##sz##_##op(atomic##sz##_t *v)	\
{									\
	kasan_check_write(v, sizeof(*v));				\
	return arch_atomic##sz##_##op(v);				\
}

#define INSTR_RET1(op)		\
__INSTR_RET1(op, , int, int);	\
__INSTR_RET1(op, 64, long long, long long)

INSTR_RET1(inc_return);
INSTR_RET1(dec_return);
__INSTR_RET1(inc_not_zero, 64, long long, long long);
__INSTR_RET1(dec_if_positive, 64, long long, long long);

#define INSTR_RET_BOOL1(op)	\
__INSTR_RET1(op, , int, bool);	\
__INSTR_RET1(op, 64, long long, bool)

INSTR_RET_BOOL1(dec_and_test);
INSTR_RET_BOOL1(inc_and_test);

#undef __INSTR_RET1
#undef INSTR_RET1
#undef INSTR_RET_BOOL1

#define __INSTR_RET2(op, sz, type, rtype)				\
static __always_inline rtype atomic##sz##_##op(type i, atomic##sz##_t *v) \
{									\
	kasan_check_write(v, sizeof(*v));				\
	return arch_atomic##sz##_##op(i, v);				\
}

#define INSTR_RET2(op)		\
__INSTR_RET2(op, , int, int);	\
__INSTR_RET2(op, 64, long long, long long)

INSTR_RET2(add_return);
INSTR_RET2(sub_return);
INSTR_RET2(fetch_add);
INSTR_RET2(fetch_sub);
INSTR_RET2(fetch_and);
INSTR_RET2(fetch_or);
INSTR_RET2(fetch_xor);

#define INSTR_RET_BOOL2(op)		\
__INSTR_RET2(op, , int, bool);		\
__INSTR_RET2(op, 64, long long, bool)

INSTR_RET_BOOL2(sub_and_test);
INSTR_RET_BOOL2(add_negative);

#undef __INSTR_RET2
#undef INSTR_RET2
#undef INSTR_RET_BOOL2

/*
 * In the following macros we need to be careful to not clash with arch_ macros.
 * arch_xchg() can be defined as an extended statement expression as well,
 * if we define a __ptr variable, and arch_xchg() also defines __ptr variable,
 * and we pass __ptr as an argument to arch_xchg(), it will use own __ptr
 * instead of ours. This leads to unpleasant crashes. To avoid the problem
 * the following macros declare variables with lots of underscores.
 */

#define xchg(ptr, v)					\
({							\
	__typeof__(ptr) ____ptr = (ptr);		\
	kasan_check_write(____ptr, sizeof(*____ptr));	\
	arch_xchg(____ptr, (v));			\
})

#define cmpxchg(ptr, old, new)				\
({							\
	__typeof__(ptr) ___ptr = (ptr);			\
	kasan_check_write(___ptr, sizeof(*___ptr));	\
	arch_cmpxchg(___ptr, (old), (new));		\
})

#define sync_cmpxchg(ptr, old, new)			\
({							\
	__typeof__(ptr) ___ptr = (ptr);			\
	kasan_check_write(___ptr, sizeof(*___ptr));	\
	arch_sync_cmpxchg(___ptr, (old), (new));	\
})

#define cmpxchg_local(ptr, old, new)			\
({							\
	__typeof__(ptr) ____ptr = (ptr);		\
	kasan_check_write(____ptr, sizeof(*____ptr));	\
	arch_cmpxchg_local(____ptr, (old), (new));	\
})

#define cmpxchg64(ptr, old, new)			\
({							\
	__typeof__(ptr) ____ptr = (ptr);		\
	kasan_check_write(____ptr, sizeof(*____ptr));	\
	arch_cmpxchg64(____ptr, (old), (new));		\
})

#define cmpxchg64_local(ptr, old, new)			\
({							\
	__typeof__(ptr) ____ptr = (ptr);		\
	kasan_check_write(____ptr, sizeof(*____ptr));	\
	arch_cmpxchg64_local(____ptr, (old), (new));	\
})

#define cmpxchg_double(p1, p2, o1, o2, n1, n2)				\
({									\
	__typeof__(p1) ____p1 = (p1);					\
	kasan_check_write(____p1, 2 * sizeof(*____p1));			\
	arch_cmpxchg_double(____p1, (p2), (o1), (o2), (n1), (n2));	\
})

#define cmpxchg_double_local(p1, p2, o1, o2, n1, n2)			\
({									\
	__typeof__(p1) ____p1 = (p1);					\
	kasan_check_write(____p1, 2 * sizeof(*____p1));			\
	arch_cmpxchg_double_local(____p1, (p2), (o1), (o2), (n1), (n2));\
})

#endif /* _LINUX_ATOMIC_INSTRUMENTED_H */
