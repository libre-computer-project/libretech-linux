// SPDX-License-Identifier: GPL-2.0 OR MIT
/* Copyright 2026 Libre Computer
 *
 * Mali-400/450 hardware performance counter exposure via debugfs.
 *
 * The GP (geometry processor) and PP (pixel processor) each have two
 * hardware performance counters (counter 0 and counter 1).  Each counter
 * has an enable register, a source-select register, a 32-bit value
 * register, and a limit register.  The source register selects which
 * hardware event the counter tracks.
 *
 * Event source IDs come from ARM DDI0415A (Mali-400 TRM) Tables 3-57
 * (GP) and 3-60 (PP).  The Mali-450 uses the same event encoding.
 *
 * Counters are reset by the driver's soft-reset between jobs, so we
 * hook into task_run (to enable counters) and task_fini (to read and
 * accumulate before reset).  The accumulated values are exposed through
 * debugfs.  Writing a source resets that counter's accumulator to 0.
 *
 * debugfs at /sys/kernel/debug/dri/<card>/perf/:
 *   gp_src0, gp_src1     -- read/write GP counter source (decimal)
 *   gp_val0, gp_val1     -- read accumulated GP counter value
 *   pp<N>_src0/1          -- read/write PP counter source per core
 *   pp<N>_val0/1          -- read accumulated PP counter value
 *   events               -- list known event source IDs
 *   snapshot              -- one-shot read of all enabled accumulators
 */

#include <linux/atomic.h>
#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>
#include <linux/seq_file.h>

#include "lima_device.h"
#include "lima_perfcnt.h"
#include "lima_regs.h"

/*
 * Mali-400/450 GP performance counter events (ARM DDI0415A Table 3-57).
 * Source register: LIMA_GP_PERF_CNT_x_SRC (0x44, 0x48).
 */
static const struct {
	u32 id;
	const char *name;
} lima_gp_events[] = {
	{ 0x00, "active_cycles_mgmt" },
	{ 0x01, "active_cycles_vertex_shader" },
	{ 0x02, "active_cycles_vertex_storer" },
	{ 0x03, "active_cycles_vertex_loader" },
	{ 0x04, "cycles_vertex_loader_waiting" },
	{ 0x05, "number_of_words_read" },
	{ 0x06, "number_of_words_written" },
	{ 0x07, "number_of_read_bursts" },
	{ 0x08, "number_of_write_bursts" },
	{ 0x09, "number_of_vertices_loader" },
	{ 0x0a, "number_of_vertices_processed" },
	{ 0x0b, "number_of_vertices_cached_hit" },
	{ 0x0c, "number_of_backface_cullings" },
	{ 0x0d, "number_of_commands" },
	{ 0x0e, "active_cycles_plbu" },
	{ 0x0f, "active_cycles_plbu_array" },
	{ 0x10, "cycles_plbu_resource_starvation" },
	{ 0x11, "number_of_output_quads" },
};

/*
 * Mali-400/450 PP performance counter events (ARM DDI0415A Table 3-60).
 * Source register: LIMA_PP_PERF_CNT_x_SRC (0x1084, 0x10a4).
 */
static const struct {
	u32 id;
	const char *name;
} lima_pp_events[] = {
	{ 0x00, "active_cycles_mgmt" },
	{ 0x01, "active_cycles_vertex_shader" },
	{ 0x02, "active_cycles_fragment_shader" },
	{ 0x03, "cycles_vertex_shader_waiting" },
	{ 0x04, "cycles_fragment_shader_waiting" },
	{ 0x05, "number_of_fragment_cycles" },
	{ 0x06, "number_of_fragment_dummy_cycles" },
	{ 0x07, "number_of_quad_rasterized" },
	{ 0x08, "number_of_tile_rasterized" },
	{ 0x09, "active_cycles_tile_rasterizer" },
	{ 0x0a, "active_cycles_load_store" },
	{ 0x0b, "number_of_read_words" },
	{ 0x0c, "number_of_write_words" },
	{ 0x0d, "number_of_read_bursts" },
	{ 0x0e, "number_of_write_bursts" },
	{ 0x0f, "number_of_read_stall_cycles" },
	{ 0x10, "number_of_write_stall_cycles" },
	{ 0x11, "number_of_l2_read_hits" },
	{ 0x12, "number_of_l2_read_misses" },
	{ 0x13, "number_of_l2_write_hits" },
	{ 0x14, "number_of_l2_write_misses" },
	{ 0x15, "number_of_compressed_texture_requests" },
	{ 0x16, "number_of_input_primitives" },
	{ 0x17, "number_of_culled_primitives" },
	{ 0x18, "number_of_visible_primitives" },
	{ 0x19, "number_of_tilebuffer_read_words" },
	{ 0x1a, "number_of_tilebuffer_write_words" },
	{ 0x1b, "number_of_tilebuffer_read_hits" },
	{ 0x1c, "number_of_tilebuffer_read_misses" },
	{ 0x1d, "number_of_tilebuffer_write_hits" },
	{ 0x1e, "number_of_tilebuffer_write_misses" },
	{ 0x1f, "active_cycles_renderer" },
	{ 0x20, "active_cycles_loading_descriptors" },
	{ 0x21, "number_of_palette_cache_hits" },
	{ 0x22, "total_cycles" },
};

/*
 * Per-counter state with 64-bit accumulators.
 * The hardware counters are 32-bit and reset each job, so we accumulate
 * across jobs into 64-bit values for meaningful profiling.
 */
struct lima_perfcnt_state {
	u32 gp_src[2];
	u32 pp_src[LIMA_SCHED_PIPE_MAX_PROCESSOR][2];
	bool gp_enabled[2];
	bool pp_enabled[LIMA_SCHED_PIPE_MAX_PROCESSOR][2];
	atomic64_t gp_accum[2];
	atomic64_t pp_accum[LIMA_SCHED_PIPE_MAX_PROCESSOR][2];
	atomic_t gp_jobs;
	atomic_t pp_jobs[LIMA_SCHED_PIPE_MAX_PROCESSOR];
};

struct lima_perfcnt_accessor {
	struct lima_device *ldev;
	int pp_index;  /* -1 = GP, 0..7 = PP core */
	int counter;   /* 0 or 1 */
};

static struct lima_perfcnt_state perfcnt_state;
static struct dentry *perfcnt_root;

#define MAX_ACCESSORS 40
static struct lima_perfcnt_accessor accessors[MAX_ACCESSORS];
static int accessor_count;

static struct lima_perfcnt_accessor *alloc_accessor(struct lima_device *ldev,
						    int pp_index, int counter)
{
	struct lima_perfcnt_accessor *a;

	if (accessor_count >= MAX_ACCESSORS)
		return NULL;
	a = &accessors[accessor_count++];
	a->ldev = ldev;
	a->pp_index = pp_index;
	a->counter = counter;
	return a;
}

/*
 * Job hooks -- called from lima_gp.c and lima_pp.c.
 * These run in IRQ or scheduler context, so they must be fast.
 */

void lima_perfcnt_gp_job_start(struct lima_ip *ip)
{
	int i;

	for (i = 0; i < 2; i++) {
		if (!perfcnt_state.gp_enabled[i])
			continue;
		if (i == 0) {
			writel(perfcnt_state.gp_src[0],
			       ip->iomem + LIMA_GP_PERF_CNT_0_SRC);
			writel(1, ip->iomem + LIMA_GP_PERF_CNT_0_ENABLE);
		} else {
			writel(perfcnt_state.gp_src[1],
			       ip->iomem + LIMA_GP_PERF_CNT_1_SRC);
			writel(1, ip->iomem + LIMA_GP_PERF_CNT_1_ENABLE);
		}
	}
}

void lima_perfcnt_gp_job_done(struct lima_ip *ip)
{
	int i;

	for (i = 0; i < 2; i++) {
		u32 val;

		if (!perfcnt_state.gp_enabled[i])
			continue;
		if (i == 0)
			val = readl(ip->iomem + LIMA_GP_PERF_CNT_0_VALUE);
		else
			val = readl(ip->iomem + LIMA_GP_PERF_CNT_1_VALUE);
		atomic64_add(val, &perfcnt_state.gp_accum[i]);
	}
	if (perfcnt_state.gp_enabled[0] || perfcnt_state.gp_enabled[1])
		atomic_inc(&perfcnt_state.gp_jobs);
}

void lima_perfcnt_pp_job_start(struct lima_ip *ip)
{
	int pp = ip->id - lima_ip_pp0;
	int i;

	if (pp < 0 || pp >= LIMA_SCHED_PIPE_MAX_PROCESSOR)
		return;

	for (i = 0; i < 2; i++) {
		if (!perfcnt_state.pp_enabled[pp][i])
			continue;
		if (i == 0) {
			writel(perfcnt_state.pp_src[pp][0],
			       ip->iomem + LIMA_PP_PERF_CNT_0_SRC);
			writel(1, ip->iomem + LIMA_PP_PERF_CNT_0_ENABLE);
		} else {
			writel(perfcnt_state.pp_src[pp][1],
			       ip->iomem + LIMA_PP_PERF_CNT_1_SRC);
			writel(1, ip->iomem + LIMA_PP_PERF_CNT_1_ENABLE);
		}
	}
}

void lima_perfcnt_pp_job_done(struct lima_ip *ip)
{
	int pp = ip->id - lima_ip_pp0;
	int i;

	if (pp < 0 || pp >= LIMA_SCHED_PIPE_MAX_PROCESSOR)
		return;

	for (i = 0; i < 2; i++) {
		u32 val;

		if (!perfcnt_state.pp_enabled[pp][i])
			continue;
		if (i == 0)
			val = readl(ip->iomem + LIMA_PP_PERF_CNT_0_VALUE);
		else
			val = readl(ip->iomem + LIMA_PP_PERF_CNT_1_VALUE);
		atomic64_add(val, &perfcnt_state.pp_accum[pp][i]);
	}
	if (perfcnt_state.pp_enabled[pp][0] || perfcnt_state.pp_enabled[pp][1])
		atomic_inc(&perfcnt_state.pp_jobs[pp]);
}

/* --- debugfs source read/write --- */

static int lima_perfcnt_src_show(struct seq_file *m, void *data)
{
	struct lima_perfcnt_accessor *a = m->private;

	if (a->pp_index < 0)
		seq_printf(m, "%u\n", perfcnt_state.gp_src[a->counter]);
	else
		seq_printf(m, "%u\n",
			   perfcnt_state.pp_src[a->pp_index][a->counter]);
	return 0;
}

static int lima_perfcnt_src_open(struct inode *inode, struct file *file)
{
	return single_open(file, lima_perfcnt_src_show, inode->i_private);
}

static ssize_t lima_perfcnt_src_write(struct file *file,
				      const char __user *ubuf,
				      size_t count, loff_t *ppos)
{
	struct seq_file *m = file->private_data;
	struct lima_perfcnt_accessor *a = m->private;
	u32 val;
	int ret;

	ret = kstrtou32_from_user(ubuf, count, 0, &val);
	if (ret)
		return ret;

	if (a->pp_index < 0) {
		perfcnt_state.gp_src[a->counter] = val;
		perfcnt_state.gp_enabled[a->counter] = true;
		atomic64_set(&perfcnt_state.gp_accum[a->counter], 0);
		atomic_set(&perfcnt_state.gp_jobs, 0);
	} else {
		perfcnt_state.pp_src[a->pp_index][a->counter] = val;
		perfcnt_state.pp_enabled[a->pp_index][a->counter] = true;
		atomic64_set(&perfcnt_state.pp_accum[a->pp_index][a->counter], 0);
		atomic_set(&perfcnt_state.pp_jobs[a->pp_index], 0);
	}

	return count;
}

static const struct file_operations lima_perfcnt_src_fops = {
	.owner = THIS_MODULE,
	.open = lima_perfcnt_src_open,
	.read = seq_read,
	.write = lima_perfcnt_src_write,
	.llseek = seq_lseek,
	.release = single_release,
};

/* --- debugfs value read (accumulated) --- */

static int lima_perfcnt_val_show(struct seq_file *m, void *data)
{
	struct lima_perfcnt_accessor *a = m->private;
	u64 val;

	if (a->pp_index < 0)
		val = atomic64_read(&perfcnt_state.gp_accum[a->counter]);
	else
		val = atomic64_read(&perfcnt_state.pp_accum[a->pp_index][a->counter]);

	seq_printf(m, "%llu\n", val);
	return 0;
}

static int lima_perfcnt_val_open(struct inode *inode, struct file *file)
{
	return single_open(file, lima_perfcnt_val_show, inode->i_private);
}

static const struct file_operations lima_perfcnt_val_fops = {
	.owner = THIS_MODULE,
	.open = lima_perfcnt_val_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/* --- events list --- */

static int lima_perfcnt_events_show(struct seq_file *m, void *data)
{
	int i;

	seq_puts(m, "GP events (write decimal ID to gp_src0/gp_src1):\n");
	for (i = 0; i < ARRAY_SIZE(lima_gp_events); i++)
		seq_printf(m, "  %2u  %s\n",
			   lima_gp_events[i].id, lima_gp_events[i].name);

	seq_puts(m, "\nPP events (write decimal ID to pp<N>_src0/pp<N>_src1):\n");
	for (i = 0; i < ARRAY_SIZE(lima_pp_events); i++)
		seq_printf(m, "  %2u  %s\n",
			   lima_pp_events[i].id, lima_pp_events[i].name);

	return 0;
}

static int lima_perfcnt_events_open(struct inode *inode, struct file *file)
{
	return single_open(file, lima_perfcnt_events_show, inode->i_private);
}

static const struct file_operations lima_perfcnt_events_fops = {
	.owner = THIS_MODULE,
	.open = lima_perfcnt_events_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/* --- snapshot: read all enabled accumulated counters --- */

static int lima_perfcnt_snapshot_show(struct seq_file *m, void *data)
{
	struct lima_device *ldev = m->private;
	int i;

	/* GP counters */
	if (perfcnt_state.gp_enabled[0] || perfcnt_state.gp_enabled[1]) {
		int jobs = atomic_read(&perfcnt_state.gp_jobs);

		seq_printf(m, "gp  jobs=%d\n", jobs);
		for (i = 0; i < 2; i++) {
			if (perfcnt_state.gp_enabled[i]) {
				u64 val = atomic64_read(
					&perfcnt_state.gp_accum[i]);
				seq_printf(m,
					   "  cnt%d  src=%u  accum=%llu\n",
					   i, perfcnt_state.gp_src[i], val);
			}
		}
	}

	/* PP counters */
	for (i = 0; i < LIMA_SCHED_PIPE_MAX_PROCESSOR; i++) {
		struct lima_ip *ip = ldev->ip + lima_ip_pp0 + i;
		int c;
		bool any = false;

		if (!ip->present)
			continue;
		for (c = 0; c < 2; c++)
			if (perfcnt_state.pp_enabled[i][c])
				any = true;
		if (!any)
			continue;

		seq_printf(m, "pp%d  jobs=%d\n", i,
			   atomic_read(&perfcnt_state.pp_jobs[i]));
		for (c = 0; c < 2; c++) {
			if (perfcnt_state.pp_enabled[i][c]) {
				u64 val = atomic64_read(
					&perfcnt_state.pp_accum[i][c]);
				seq_printf(m,
					   "  cnt%d  src=%u  accum=%llu\n",
					   c, perfcnt_state.pp_src[i][c], val);
			}
		}
	}

	return 0;
}

static int lima_perfcnt_snapshot_open(struct inode *inode, struct file *file)
{
	return single_open(file, lima_perfcnt_snapshot_show, inode->i_private);
}

static const struct file_operations lima_perfcnt_snapshot_fops = {
	.owner = THIS_MODULE,
	.open = lima_perfcnt_snapshot_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/* --- init / fini --- */

void lima_perfcnt_debugfs_init(struct lima_device *ldev)
{
	struct dentry *root;
	struct lima_perfcnt_accessor *a;
	char name[32];
	int i;

	memset(&perfcnt_state, 0, sizeof(perfcnt_state));
	accessor_count = 0;

	root = debugfs_create_dir("perf", ldev->ddev->debugfs_root);
	if (IS_ERR_OR_NULL(root))
		return;
	perfcnt_root = root;

	/* GP counter files */
	for (i = 0; i < 2; i++) {
		a = alloc_accessor(ldev, -1, i);
		if (!a)
			return;
		snprintf(name, sizeof(name), "gp_src%d", i);
		debugfs_create_file(name, 0644, root, a,
				    &lima_perfcnt_src_fops);

		a = alloc_accessor(ldev, -1, i);
		if (!a)
			return;
		snprintf(name, sizeof(name), "gp_val%d", i);
		debugfs_create_file(name, 0444, root, a,
				    &lima_perfcnt_val_fops);
	}

	/* PP counter files for each present core */
	for (i = 0; i < LIMA_SCHED_PIPE_MAX_PROCESSOR; i++) {
		struct lima_ip *ip = ldev->ip + lima_ip_pp0 + i;

		if (!ip->present)
			continue;

		for (int c = 0; c < 2; c++) {
			a = alloc_accessor(ldev, i, c);
			if (!a)
				return;
			snprintf(name, sizeof(name), "pp%d_src%d", i, c);
			debugfs_create_file(name, 0644, root, a,
					    &lima_perfcnt_src_fops);

			a = alloc_accessor(ldev, i, c);
			if (!a)
				return;
			snprintf(name, sizeof(name), "pp%d_val%d", i, c);
			debugfs_create_file(name, 0444, root, a,
					    &lima_perfcnt_val_fops);
		}
	}

	/* Event list */
	debugfs_create_file("events", 0444, root, ldev,
			    &lima_perfcnt_events_fops);

	/* Snapshot */
	debugfs_create_file("snapshot", 0444, root, ldev,
			    &lima_perfcnt_snapshot_fops);

	dev_info(ldev->dev, "perf counters exposed via debugfs\n");
}

void lima_perfcnt_debugfs_fini(struct lima_device *ldev)
{
	debugfs_remove_recursive(perfcnt_root);
	perfcnt_root = NULL;
}
