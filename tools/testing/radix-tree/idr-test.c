/*
 * idr.c: Test the IDR API
 * Copyright (c) 2016 Matthew Wilcox <willy@infradead.org>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */
#include <linux/idr.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/errno.h>

#include "test.h"

#define DUMMY_PTR	((void *)0x12)

int item_idr_free(int id, void *p, void *data)
{
	struct item *item = p;
	assert(item->index == id);
	idr_remove(data, id);
	free(p);

	return 0;
}

void item_idr_remove(struct idr *idr, int id)
{
	struct item *item = idr_find(idr, id);
	assert(item->index == id);
	idr_remove(idr, id);
	free(item);
}

void idr_alloc_test(void)
{
	unsigned long i;
	DEFINE_IDR(idr);

	assert(idr_alloc_cyclic(&idr, DUMMY_PTR, 0, 0x4000, GFP_KERNEL) == 0);
	assert(idr_alloc_cyclic(&idr, DUMMY_PTR, 0x3ffd, 0x4000, GFP_KERNEL) == 0x3ffd);
	idr_remove(&idr, 0x3ffd);
	idr_remove(&idr, 0);

	for (i = 0x3ffe; i < 0x4003; i++) {
		int id;
		struct item *item;

		if (i < 0x4000)
			item = item_create(i, 0);
		else
			item = item_create(i - 0x3fff, 0);

		id = idr_alloc_cyclic(&idr, item, 1, 0x4000, GFP_KERNEL);
		assert(id == item->index);
	}

	idr_for_each(&idr, item_idr_free, &idr);
}

void idr_replace_test(void)
{
	DEFINE_IDR(idr);

	idr_alloc(&idr, (void *)-1, 10, 11, GFP_KERNEL);
	idr_replace(&idr, &idr, 10);

	idr_destroy(&idr);
}

void idr_checks(void)
{
	unsigned long i;
	DEFINE_IDR(idr);

	for (i = 0; i < 10000; i++) {
		struct item *item = item_create(i, 0);
		assert(idr_alloc(&idr, item, 0, 20000, GFP_KERNEL) == i);
	}

	assert(idr_alloc(&idr, DUMMY_PTR, 5, 30, GFP_KERNEL) < 0);

	for (i = 0; i < 5000; i++)
		item_idr_remove(&idr, i);

	idr_for_each(&idr, item_idr_free, &idr);

	assert(idr_is_empty(&idr));

	for (i = INT_MAX - 3UL; i < INT_MAX + 1UL; i++) {
		struct item *item = item_create(i, 0);
		assert(idr_alloc(&idr, item, i, i + 10, GFP_KERNEL) == i);
	}
	assert(idr_alloc(&idr, DUMMY_PTR, i - 2, i, GFP_KERNEL) == -ENOSPC);

	idr_destroy(&idr);
	idr_destroy(&idr);

	assert(idr_is_empty(&idr));

	for (i = 1; i < 10000; i++) {
		struct item *item = item_create(i, 0);
		assert(idr_alloc(&idr, item, 1, 20000, GFP_KERNEL) == i);
	}

	idr_destroy(&idr);

	idr_replace_test();

	idr_alloc_test();
}

/*
 * Check that we get the correct error when we run out of memory doing
 * allocations.  To ensure we run out of memory, just "forget" to preload.
 * The first test is for not having a bitmap available, and the second test
 * is for not being able to allocate a level of the radix tree.
 */
void ida_check_nomem(void)
{
	DEFINE_IDA(ida);
	int id, err;

	err = ida_get_new(&ida, &id);
	assert(err == -EAGAIN);
	err = ida_get_new_above(&ida, 1UL << 30, &id);
	assert(err == -EAGAIN);
}

void ida_checks(void)
{
	DEFINE_IDA(ida);

	unsigned long i, j;
	int id;
	int err;

	ida_check_nomem();

	for (i = 0; i < 10000; i++) {
		ida_pre_get(&ida, GFP_KERNEL);
		ida_get_new(&ida, &id);
		assert(id == i);
	}

	ida_remove(&ida, 20);
	ida_remove(&ida, 21);
	for (i = 0; i < 3; i++) {
		ida_pre_get(&ida, GFP_KERNEL);
		ida_get_new(&ida, &id);
		if (i == 2)
			assert(id == 10000);
	}

	for (i = 0; i < 5000; i++)
		ida_remove(&ida, i);

	ida_pre_get(&ida, GFP_KERNEL);
	ida_get_new_above(&ida, 5000, &id);
	assert(id == 10001);

	ida_destroy(&ida);

	assert(ida_is_empty(&ida));

	ida_pre_get(&ida, GFP_KERNEL);
	ida_get_new_above(&ida, 1, &id);
	assert(id == 1);

	ida_remove(&ida, id);
	assert(ida_is_empty(&ida));
	ida_destroy(&ida);

	ida_pre_get(&ida, GFP_KERNEL);
	ida_get_new_above(&ida, 1, &id);
	ida_destroy(&ida);

	for (j = 1; j < 65537; j *= 2) {
		for (i = 0; i < j; i++) {
			ida_pre_get(&ida, GFP_KERNEL);
			err = ida_get_new_above(&ida, (1UL << 31) - j, &id);
			assert(err == 0);
			assert(id == (1UL << 31) - j + i);
		}
		ida_pre_get(&ida, GFP_KERNEL);
		err = ida_get_new_above(&ida, 0x7fffffff, &id);
		assert(err == -ENOSPC);
		ida_destroy(&ida);
		assert(ida_is_empty(&ida));
		rcu_barrier();
	}

	radix_tree_cpu_dead(1);
}
