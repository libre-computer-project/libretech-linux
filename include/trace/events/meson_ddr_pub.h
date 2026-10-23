/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM meson_ddr_pub

#if !defined(_TRACE_MESON_DDR_PUB_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_MESON_DDR_PUB_H

#include <linux/tracepoint.h>

TRACE_EVENT(field_change,
	TP_PROTO(const char *word, const char *field, u32 old, u32 new),

	TP_ARGS(word, field, old, new),

	TP_STRUCT__entry(
		__field(u32, old)
		__field(u32, new)
		__string(word, word)
		__string(field, field)
	),

	TP_fast_assign(
		__entry->old = old;
		__entry->new = new;
		__assign_str(word);
		__assign_str(field);
	),

	/*
	 * One tracked delay-line field moved between two samples, e.g.
	 * "dx2.LCDLR1 lo 0x0000004f -> 0x00000050". Lets a memory fault be
	 * lined up against the PHY's VT activity in the ftrace stream.
	 */
	TP_printk("%s %s 0x%08x -> 0x%08x",
		  __get_str(word), __get_str(field), __entry->old,
		  __entry->new)
);

TRACE_EVENT(pub_resume,
	TP_PROTO(u64 count),

	TP_ARGS(count),

	TP_STRUCT__entry(
		__field(u64, count)
	),

	TP_fast_assign(
		__entry->count = count;
	),

	/*
	 * System sleep ended; count is the resumes since probe. With the
	 * resume debugfs file it lets a fault after a wake be told apart
	 * from one mid-boot.
	 */
	TP_printk("count=%llu", __entry->count)
);

#endif /* _TRACE_MESON_DDR_PUB_H */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH trace/events
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE meson_ddr_pub
#include <trace/define_trace.h>
