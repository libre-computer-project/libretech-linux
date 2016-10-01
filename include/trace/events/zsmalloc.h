#undef TRACE_SYSTEM
#define TRACE_SYSTEM zsmalloc

#if !defined(_TRACE_ZSMALLOC_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_ZSMALLOC_H

#include <linux/types.h>
#include <linux/tracepoint.h>

TRACE_EVENT(zsmalloc_compact_start,

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

TRACE_EVENT(zsmalloc_compact_end,

	TP_PROTO(const char *pool_name, unsigned long pages_compacted,
			unsigned long pages_total_compacted),

	TP_ARGS(pool_name, pages_compacted, pages_total_compacted),

	TP_STRUCT__entry(
		__field(const char *, pool_name)
		__field(unsigned long, pages_compacted)
		__field(unsigned long, pages_total_compacted)
	),

	TP_fast_assign(
		__entry->pool_name = pool_name;
		__entry->pages_compacted = pages_compacted;
		__entry->pages_total_compacted = pages_total_compacted;
	),

	TP_printk("pool %s: %ld pages compacted(total %ld)",
		  __entry->pool_name,
		  __entry->pages_compacted,
		  __entry->pages_total_compacted)
);

#endif /* _TRACE_ZSMALLOC_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
