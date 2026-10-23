/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/* Copyright 2026 Libre Computer */

#ifndef __LIMA_PERFCNT_H__
#define __LIMA_PERFCNT_H__

#include <linux/types.h>

struct lima_device;
struct lima_ip;

#if IS_ENABLED(CONFIG_DEBUG_FS)
void lima_perfcnt_debugfs_init(struct lima_device *ldev);
void lima_perfcnt_debugfs_fini(struct lima_device *ldev);

/* Called from task_run to enable counters before a job starts */
void lima_perfcnt_gp_job_start(struct lima_ip *ip);
void lima_perfcnt_pp_job_start(struct lima_ip *ip);

/* Called from task_fini to read and accumulate before soft reset */
void lima_perfcnt_gp_job_done(struct lima_ip *ip);
void lima_perfcnt_pp_job_done(struct lima_ip *ip);
#else
static inline void lima_perfcnt_debugfs_init(struct lima_device *ldev) {}
static inline void lima_perfcnt_debugfs_fini(struct lima_device *ldev) {}
static inline void lima_perfcnt_gp_job_start(struct lima_ip *ip) {}
static inline void lima_perfcnt_pp_job_start(struct lima_ip *ip) {}
static inline void lima_perfcnt_gp_job_done(struct lima_ip *ip) {}
static inline void lima_perfcnt_pp_job_done(struct lima_ip *ip) {}
#endif

#endif
