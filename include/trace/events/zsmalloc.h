#undef TRACE_SYSTEM
#define TRACE_SYSTEM zsmalloc

#if !defined(_TRACE_ZSMALLOC_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_ZSMALLOC_H

#include <linux/types.h>
#include <linux/tracepoint.h>

TRACE_EVENT(zs_compact_start,

	TP_PROTO(const char *pool_name),

	TP_ARGS(pool_name),

	TP_STRUCT__entry(
		__field(const char *, pool_name)
	),

	TP_fast_assign(
		__entry->pool_name = pool_name;
	),

	TP_printk("pool %s",
		  __entry->pool_name)
);

TRACE_EVENT(zs_compact_end,

	TP_PROTO(const char *pool_name, unsigned long pages_compacted),

	TP_ARGS(pool_name, pages_compacted),

	TP_STRUCT__entry(
		__field(const char *, pool_name)
		__field(unsigned long, pages_compacted)
	),

	TP_fast_assign(
		__entry->pool_name = pool_name;
		__entry->pages_compacted = pages_compacted;
	),

	TP_printk("pool %s: %ld pages compacted",
		  __entry->pool_name,
		  __entry->pages_compacted)
);

TRACE_EVENT(zs_compact,

	TP_PROTO(int class, unsigned long nr_migrated_obj, unsigned long nr_freed_pages),

	TP_ARGS(class, nr_migrated_obj, nr_freed_pages),

	TP_STRUCT__entry(
		__field(int, class)
		__field(unsigned long, nr_migrated_obj)
		__field(unsigned long, nr_freed_pages)
	),

	TP_fast_assign(
		__entry->class = class;
		__entry->nr_migrated_obj = nr_migrated_obj;
		__entry->nr_freed_pages = nr_freed_pages;
	),

	TP_printk("class %3d: %ld objects migrated, %ld pages freed",
		  __entry->class,
		  __entry->nr_migrated_obj,
		  __entry->nr_freed_pages)
);

#endif /* _TRACE_ZSMALLOC_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
