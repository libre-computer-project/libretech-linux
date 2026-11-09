/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2018 MediaTek Inc.
 * Author: Cheng-Jung Ho <cheng-jung.ho@mediatek.com>
 */

#ifndef __VCODEC_DVFS_H__
#define __VCODEC_DVFS_H__

#define MAX_HISTORY 10
#define MIN_SUBMIT_GAP 2000		/* 2ms */
#define MAX_SUBMIT_GAP (1000*1000)	/* 1 second */
#define FREE_HIST_DELAY (5000*1000)	/* Free history delay */
#define DEFAULT_MHZ 99999
#define MAX_VDEC_HW 8
#define DEC_MODULE MAX_VDEC_HW

struct codec_history {
	void *handle;
	int kcy[MAX_HISTORY];
	long long submit[MAX_HISTORY];
	long long start[MAX_HISTORY];
	long long end[MAX_HISTORY];
	long long sw_time[MAX_HISTORY];
	long long submit_interval;
	int cur_idx;
	int cur_cnt;
	int tot_kcy;
	long long tot_time;
	struct codec_history *next;
};

struct codec_job {
	void *handle;
	long long submit;
	long long start;
	long long end;
	int hw_kcy;
	int mhz;
	struct codec_job *next;
};

struct codec_freq {
	unsigned long freq[DEC_MODULE];
	unsigned long active_freq;
};


/* first scenario based version, will change to loading based */
struct temp_job {
	int ctx_id;
	int format;
	int type;
	int module;
	int visible_width; /* temp usage only, will use kcy */
	int visible_height; /* temp usage only, will use kcy */
	int operation_rate;
	long long submit;
	int kcy;
	struct temp_job *next;
};

/* dvfs policies  */
struct dvfs_params {
	u8 codec_type;
	u8 allow_oc;		/* allow oc freq */
	u8 per_frame_adjust;	/* do per frame adjust dvfs / pmqos */
	u8 per_frame_adjust_op_rate; /* per frame adjust threshold */
	u32 min_freq;		/* min freq */
	u32 normal_max_freq;	/* normal max freq (no oc) */
	u32 freq_sum;		/* summation of all instances */
	u32 target_freq;	/* target freq */
	u8 lock_cnt[DEC_MODULE]; /* lock cnt */
	u8 frame_need_update;	/* this frame begin / end needs update */
};
long long get_time_us(void);

/* Add a new job to job queue */
struct codec_job *add_job(void *handle, struct codec_job **head);

/* Move target job to queue head for processing */
struct codec_job *move_job_to_head(void *handle, struct codec_job **head);

/* Update history with completed job */
int update_hist(struct codec_job *job, struct codec_history **head,
		long long submit_interval);

/* Estimate required freq from job queue and previous history */
int est_freq(void *handle, struct codec_job **job, struct codec_history *head);
unsigned long match_freq(
	unsigned long target_hz, unsigned long *freq_list, u32 freq_cnt, unsigned long max_freq_hz);
u64 match_freq_v2(int target_mhz, u64 *freq_list, u32 freq_cnt);

/* Free unused/all history */
int free_hist(struct codec_history **head, int only_unused);
int free_hist_by_handle(void *handle, struct codec_history **head);
#endif
