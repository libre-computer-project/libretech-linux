/*
 * Specific stress-testing of wakeup logic in the presence of hotplug
 * operations.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 * Copyright (C) IBM Corporation, 2016
 *
 * Author: Paul E. McKenney <paulmck@us.ibm.com>
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/smp.h>
#include <linux/rcupdate.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/atomic.h>
#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/moduleparam.h>
#include <linux/percpu.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/freezer.h>
#include <linux/cpu.h>
#include <linux/delay.h>
#include <linux/stat.h>
#include <linux/srcu.h>
#include <linux/slab.h>
#include <linux/trace_clock.h>
#include <asm/byteorder.h>
#include <linux/torture.h>
#include <linux/vmalloc.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Paul E. McKenney <paulmck@us.ibm.com>");


torture_param(int, nwaiters, -1, "Number of timed-wait threads");
torture_param(int, onoff_holdoff, 0, "Time after boot before CPU hotplugs (s)");
torture_param(int, onoff_interval, 0,
	     "Time between CPU hotplugs (jiffies), 0=disable");
torture_param(int, shutdown_secs, 0, "Shutdown time (s), <= zero to disable.");
torture_param(int, stat_interval, 60,
	     "Number of seconds between stats printk()s");
torture_param(bool, verbose, true,
	     "Enable verbose debugging printk()s");
torture_param(int, wait_duration, 127,
	     "Number of microseconds to wait each iteration");
torture_param(int, wait_grace, 20,
	     "Number of jiffies before complaining about long wait");

static char *torture_type = "sh";
module_param(torture_type, charp, 0444);
MODULE_PARM_DESC(torture_type, "Type of wait to torture (sti, stui, ...)");

static int nrealwaiters;
static struct task_struct **waiter_tasks;
static struct task_struct *stats_task;
static struct task_struct *onoff_task;

/* Yes, these cache-thrash, and it is inherent to the concurrency design. */
static bool *waiter_done;		/* Waiter is done, don't wake. */
static unsigned long *waiter_iter;	/* Number of wait iterations. */
static bool *waiter_cts;		/* Waiter already checked. */
static unsigned long *waiter_kicks;	/* Number of waiter starvations. */
static unsigned long *waiter_ts;	/* Jiffies last run. */
static DEFINE_MUTEX(waiter_mutex);
static DEFINE_PER_CPU(u64, waiter_cputime); /* Nanoseconds. */
static u64 starttime;

static int onoff_cpu = -1;
static long n_offline_attempts;
static long n_offline_successes;
static unsigned long sum_offline;
static int min_offline = -1;
static int max_offline;
static long n_online_attempts;
static long n_online_successes;
static unsigned long sum_online;
static int min_online = -1;
static int max_online;

static int torture_runnable = IS_ENABLED(MODULE);
module_param(torture_runnable, int, 0444);
MODULE_PARM_DESC(torture_runnable, "Start waketorture at boot");

/*
 * Operations vector for selecting different types of tests.
 */

struct wake_torture_ops {
	void (*wait)(void);
	const char *name;
};

static struct wake_torture_ops *cur_ops;

/*
 * Definitions for schedule_hrtimeout() torture testing.
 */

static void wait_schedule_hrtimeout(void)
{
	ktime_t wait = ns_to_ktime(wait_duration * 1000);

	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule_hrtimeout(&wait, HRTIMER_MODE_REL);
}

static struct wake_torture_ops sh_ops = {
	.wait		= wait_schedule_hrtimeout,
	.name		= "sh"
};

/*
 * Definitions for schedule_timeout_interruptible() torture testing.
 */

static void wait_schedule_timeout_interruptible(void)
{
	schedule_timeout_interruptible((wait_duration + 999) / 1000);
}

static struct wake_torture_ops sti_ops = {
	.wait		= wait_schedule_timeout_interruptible,
	.name		= "sti"
};

/*
 * Definitions for schedule_timeout_uninterruptible() torture testing.
 */

static void wait_schedule_timeout_uninterruptible(void)
{
	schedule_timeout_uninterruptible((wait_duration + 999) / 1000);
}

static struct wake_torture_ops stui_ops = {
	.wait		= wait_schedule_timeout_uninterruptible,
	.name		= "stui"
};

/*
 * Has the specified waiter thread run recently?
 */
static bool kthread_ran_recently(int tnum)
{
	smp_mb(); /* Ensure waiter_cts[] read before waiter_ts[].  [A] */
	return time_before(READ_ONCE(waiter_ts[tnum]), jiffies + wait_grace);
}

/*
 * Wakeup torture fake writer kthread.  Repeatedly calls sync, with a random
 * delay between calls.
 */
static int wake_torture_waiter(void *arg)
{
	int i;
	long me = (long)arg;
	u64 ts;

	VERBOSE_TOROUT_STRING("wake_torture_waiter task started");
	set_user_nice(current, MAX_NICE);

	preempt_disable();
	ts = trace_clock_local();
	do {
		waiter_ts[me] = jiffies;
		smp_mb(); /* Ensure waiter_ts[] written before waiter_cts[]. */
			  /* Pairs with [A]. */
		waiter_cts[me] = false;
		__this_cpu_add(waiter_cputime, trace_clock_local() - ts);
		preempt_enable();
		cur_ops->wait();
		preempt_disable();
		ts = trace_clock_local();
		waiter_iter[me]++;
		for (i = 0; i < nrealwaiters; i++) {
			if (waiter_done[i] ||
			    waiter_cts[i] ||
			    kthread_ran_recently(i))
				continue;
			if (!mutex_trylock(&waiter_mutex)) {
				break; /* Keep lock contention to dull roar. */
			} else if (waiter_done[i] ||
				   waiter_cts[i] ||
				   kthread_ran_recently(i)) {
				mutex_unlock(&waiter_mutex);
			} else {
				waiter_cts[i] = true;
				waiter_kicks[i]++;
				pr_alert("%s%s wake_torture_waiter(): P%d (%#lx) failing to awaken!\n",
					 torture_type, TORTURE_FLAG,
					 waiter_tasks[i]->pid,
					 waiter_tasks[i]->state);
				rcu_ftrace_dump(DUMP_ALL);
				wake_up_process(waiter_tasks[i]);
				mutex_unlock(&waiter_mutex);
			}
		}
		__this_cpu_add(waiter_cputime, trace_clock_local() - ts);
		preempt_enable();
		torture_shutdown_absorb("wake_torture_waiter");
		preempt_disable();
		ts = trace_clock_local();
	} while (!torture_must_stop());
	__this_cpu_add(waiter_cputime, trace_clock_local() - ts);
	preempt_enable();
	mutex_lock(&waiter_mutex);
	waiter_done[me] = true;
	mutex_unlock(&waiter_mutex);
	torture_kthread_stopping("wake_torture_waiter");
	return 0;
}

/*
 * Find a hotpluggable CPU and repeatedly take it online and offline.
 */
static int wake_torture_onoff(void *args)
{
	cpumask_var_t cm;
	int cpu;
	int i;

	VERBOSE_TOROUT_STRING("wake_torture_onoff task started");
	if (onoff_holdoff > 0) {
		VERBOSE_TOROUT_STRING("wake_torture_onoff begin holdoff");
		schedule_timeout_interruptible(onoff_holdoff * HZ);
		VERBOSE_TOROUT_STRING("wake_torture_onoff end holdoff");
	}

	/*
	 * Find the last hotpluggable CPU, and affinity the waiter
	 * tasks elsewhere.
	 */
	for_each_online_cpu(cpu) {
		if (cpu_is_hotpluggable(cpu))
			onoff_cpu = cpu;
	}
	if (onoff_cpu < 0) {
		VERBOSE_TOROUT_STRING("wake_torture_onoff: no hotpluggable CPUs!");
		if (shutdown_secs > 0) {
			VERBOSE_TOROUT_STRING("wake_torture_onoff: Shutting down");
			kernel_power_off();
			VERBOSE_TOROUT_STRING("wake_torture_onoff: Survived kernel_power_off()?");
		}
	}
	pr_alert("%s" TORTURE_FLAG " wake_torture_onoff: onoff_cpu: %d\n", torture_type, onoff_cpu);
	if (!zalloc_cpumask_var(&cm, GFP_KERNEL)) {
		VERBOSE_TOROUT_STRING("wake_torture_onoff: Out of memory, no affinity");
	} else {
		cpumask_copy(cm, cpu_online_mask);
		cpumask_clear_cpu(onoff_cpu, cm);
		if (cpumask_weight(cm) == 0)
			cpumask_setall(cm);
		for (i = 0; i < nrealwaiters; i++)
			set_cpus_allowed_ptr(waiter_tasks[i], cm);
	}

	/* Cycle the victim CPU online and offline! */
	while (!torture_must_stop() && onoff_cpu >= 0) {
		if (!torture_offline(onoff_cpu,
				    &n_offline_attempts, &n_offline_successes,
				    &sum_offline, &min_offline, &max_offline))
			torture_online(onoff_cpu,
				       &n_online_attempts, &n_online_successes,
				       &sum_online, &min_online, &max_online);
		schedule_timeout_interruptible(onoff_interval);
	}
	torture_kthread_stopping("wake_torture_onoff");
	return 0;
}

/*
 * Initiate waketorture-specific online-offline handling, which
 * focuses on a single CPU.
 */
static int wake_torture_onoff_init(void)
{
	int ret = 0;

	if (!IS_ENABLED(CONFIG_HOTPLUG_CPU))
		return ret;
	if (onoff_interval <= 0)
		return 0;
	ret = torture_create_kthread(wake_torture_onoff, NULL, onoff_task);
	return ret;
}

/*
 * Clean up after waketorture-specific online-offline handling.
 */
static void wake_torture_onoff_cleanup(void)
{
	if (!IS_ENABLED(CONFIG_HOTPLUG_CPU))
		return;
	VERBOSE_TOROUT_STRING("Stopping wake_torture_onoff task");
	kthread_stop(onoff_task);
	onoff_task = NULL;
}

/*
 * Print torture statistics.  Caller must ensure that there is only one
 * call to this function at a given time!!!  This is normally accomplished
 * by relying on the module system to only have one copy of the module
 * loaded, and then by giving the wake_torture_stats kthread full control
 * (or the init/cleanup functions when wake_torture_stats thread is not
 * running).
 *
 * Note that some care is required because this can be called once during
 * cleanup processing after a failed startup attempt.
 */
static void
wake_torture_stats_print(void)
{
	int i;
	bool tardy = false;
	u64 timediff;
	u64 timetot;

	if (!waiter_done || !waiter_iter || !waiter_cts ||
	    !waiter_kicks || !waiter_ts) {
		TOROUT_STRING("Partial initialization, no stats print.\n");
		return;
	}
	for (i = 0; i < nrealwaiters; i++)
		if (waiter_kicks[i]) {
			if (!tardy)
				pr_alert("%s" TORTURE_FLAG " Tardy kthreads:",
					 torture_type);
			tardy = true;
			pr_cont("  P%d%c: %lud/%lu",
				waiter_tasks && waiter_tasks[i]
					? waiter_tasks[i]->pid
					: -1,
				"!."[kthread_ran_recently(i)],
				waiter_kicks[i], waiter_iter[i]);
		}
	if (tardy)
		pr_cont("\n");
	else
		TOROUT_STRING(" No tardy kthreads");
	timediff = (trace_clock_global() - starttime) / 1000;
	timetot = 0;
	for_each_possible_cpu(i)
		timetot += READ_ONCE(per_cpu(waiter_cputime, i));
	timetot /= nr_cpu_ids;
	timetot /= timediff;
	pr_alert("%s" TORTURE_FLAG " timediff: %llu utilization: %llu.%llu nr_cpu_ids: %d onoff: %ld/%ld:%ld/%ld %d,%d:%d,%d %lu:%lu (HZ=%d)\n",
		 torture_type, timediff,
		 timetot / 1000ULL, timetot % 1000ULL, nr_cpu_ids,
		 n_online_successes, n_online_attempts,
		 n_offline_successes, n_offline_attempts,
		 min_online, max_online,
		 min_offline, max_offline,
		 sum_online, sum_offline, HZ);
}

/*
 * Periodically prints torture statistics, if periodic statistics printing
 * was specified via the stat_interval module parameter.
 */
static int
wake_torture_stats(void *arg)
{
	VERBOSE_TOROUT_STRING("wake_torture_stats task started");
	do {
		schedule_timeout_interruptible(stat_interval * HZ);
		wake_torture_stats_print();
		torture_shutdown_absorb("wake_torture_stats");
	} while (!torture_must_stop());
	torture_kthread_stopping("wake_torture_stats");
	return 0;
}

static inline void
wake_torture_print_module_parms(struct wake_torture_ops *cur_ops,
				const char *tag)
{
	pr_alert("%s" TORTURE_FLAG
		 "--- %s: nwaiters=%d onoff_holdoff=%d onoff_interval=%d shutdown_secs=%d stat_interval=%d verbose=%d wait_duration=%d wait_grace=%d\n",
		 torture_type, tag,
		 nrealwaiters, onoff_holdoff, onoff_interval,
		 shutdown_secs, stat_interval, verbose,
		 wait_duration, wait_grace);
}

static void
wake_torture_cleanup(void)
{
	int i;
	bool success;

	(void)torture_cleanup_begin();

	if (onoff_task)
		wake_torture_onoff_cleanup();

	if (waiter_tasks) {
		for (i = 0; i < nrealwaiters; i++)
			torture_stop_kthread(wake_torture_waiter,
					     waiter_tasks[i]);
		kfree(waiter_tasks);
	}

	torture_stop_kthread(wake_torture_stats, stats_task);

	wake_torture_stats_print();  /* -After- the stats thread is stopped! */

	success = !!waiter_kicks;
	for (i = 0; i < nrealwaiters; i++)
		if (!success || waiter_kicks[i]) {
			success = false;
			break;
		}

	kfree(waiter_done);
	kfree(waiter_iter);
	kfree(waiter_cts);
	kfree(waiter_kicks);
	kfree(waiter_ts);

	wake_torture_print_module_parms(cur_ops,
					success ? "End of test: SUCCESS"
						: "End of test: FAILURE");
	torture_cleanup_end();
}

static int __init
wake_torture_init(void)
{
	int i;
	int firsterr = 0;
	static struct wake_torture_ops *torture_ops[] = {
		&sh_ops, &sti_ops, &stui_ops
	};

	if (!torture_init_begin(torture_type, verbose, &torture_runnable))
		return -EBUSY;
	starttime = trace_clock_global();

	/* Process args and tell the world that the torturer is on the job. */
	for (i = 0; i < ARRAY_SIZE(torture_ops); i++) {
		cur_ops = torture_ops[i];
		if (strcmp(torture_type, cur_ops->name) == 0)
			break;
	}
	if (i == ARRAY_SIZE(torture_ops)) {
		pr_alert("wake-torture: invalid torture type: \"%s\"\n",
			 torture_type);
		pr_alert("wake-torture types:");
		for (i = 0; i < ARRAY_SIZE(torture_ops); i++)
			pr_alert(" %s", torture_ops[i]->name);
		pr_alert("\n");
		firsterr = -EINVAL;
		goto unwind;
	}

	if (nwaiters >= 0) {
		nrealwaiters = nwaiters;
	} else {
		nrealwaiters = num_online_cpus() - 2 - nwaiters;
		if (nrealwaiters <= 0)
			nrealwaiters = 1;
	}
	wake_torture_print_module_parms(cur_ops, "Start of test");

	/* Initialize the statistics so that each run gets its own numbers. */

	waiter_done = kcalloc(nrealwaiters, sizeof(*waiter_done), GFP_KERNEL);
	waiter_iter = kcalloc(nrealwaiters, sizeof(*waiter_iter), GFP_KERNEL);
	waiter_cts = kcalloc(nrealwaiters, sizeof(*waiter_cts), GFP_KERNEL);
	waiter_kicks = kcalloc(nrealwaiters, sizeof(*waiter_kicks), GFP_KERNEL);
	waiter_ts = kcalloc(nrealwaiters, sizeof(*waiter_ts), GFP_KERNEL);
	if (!waiter_done || !waiter_iter || !waiter_cts || !waiter_kicks ||
	    !waiter_ts) {
		VERBOSE_TOROUT_ERRSTRING("out of memory");
		firsterr = -ENOMEM;
		goto unwind;
	}

	/* Start up the kthreads. */

	waiter_tasks = kcalloc(nrealwaiters, sizeof(waiter_tasks[0]),
			       GFP_KERNEL);
	if (!waiter_tasks) {
		VERBOSE_TOROUT_ERRSTRING("out of memory");
		firsterr = -ENOMEM;
		goto unwind;
	}
	for (i = 0; i < nrealwaiters; i++) {
		firsterr = torture_create_kthread(wake_torture_waiter,
						  NULL, waiter_tasks[i]);
		if (firsterr)
			goto unwind;
	}
	if (stat_interval > 0) {
		firsterr = torture_create_kthread(wake_torture_stats, NULL,
						  stats_task);
		if (firsterr)
			goto unwind;
	}
	firsterr = torture_shutdown_init(shutdown_secs, wake_torture_cleanup);
	if (firsterr)
		goto unwind;
	firsterr = wake_torture_onoff_init();
	if (firsterr)
		goto unwind;
	torture_init_end();
	return 0;

unwind:
	torture_init_end();
	wake_torture_cleanup();
	return firsterr;
}

module_init(wake_torture_init);
module_exit(wake_torture_cleanup);
