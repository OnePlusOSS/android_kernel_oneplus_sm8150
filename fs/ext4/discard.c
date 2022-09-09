/***********************************************************
** Copyright (C), 2008-2018, OPLUS Mobile Comm Corp., Ltd.
** File: - discard.c
** Description: Source file for ext4 async discard suppot.
** To support ext4 async discard.
** Version: 1.0
** Date : 2018/11/26
**
** ------------------------------- Revision History:-------------------------------
** <author> <data> <version > <desc>
** yh 2018/11/26 1.0 build this module
****************************************************************/

#include <linux/fs.h>
#include <linux/bio.h>
#include <linux/blkdev.h>
#include <linux/prefetch.h>
#include <linux/kthread.h>
#include <linux/swap.h>
#include <linux/timer.h>
#include <linux/freezer.h>
#include "discard.h"
#include "ext4.h"

static void __init_discard_policy(struct ext4_sb_info *sbi,
				struct discard_policy *dpolicy,
				int discard_type, unsigned int granularity)
{
	/* common policy */
	dpolicy->type = discard_type;
	dpolicy->sync = true;
	dpolicy->granularity = granularity;

	dpolicy->max_requests = DEF_MAX_DISCARD_REQUEST;
	dpolicy->io_aware_gran = MAX_DISCARD_GRANLULARITY;

	if (discard_type == DPOLICY_BG) {
		dpolicy->min_interval = DEF_MIN_DISCARD_ISSUE_TIME;
		dpolicy->mid_interval = DEF_MID_DISCARD_ISSUE_TIME;
		dpolicy->max_interval = DEF_MAX_DISCARD_ISSUE_TIME;
		dpolicy->io_aware = true;
		dpolicy->sync = false;

		if (ext4_utilization(sbi) < DEF_DISCARD_URGENT_UTIL) {
			dpolicy->granularity = 1;
			dpolicy->max_interval = DEF_MIN_DISCARD_ISSUE_TIME;
		}

	} else if (discard_type == DPOLICY_FORCE) {
		dpolicy->min_interval = DEF_MIN_DISCARD_ISSUE_TIME;
		dpolicy->mid_interval = DEF_MID_DISCARD_ISSUE_TIME;
		dpolicy->max_interval = DEF_MAX_DISCARD_ISSUE_TIME;
		dpolicy->io_aware = false;
	} else if (discard_type == DPOLICY_FSTRIM) {
		dpolicy->io_aware = false;
	} else if (discard_type == DPOLICY_UMOUNT) {
		dpolicy->io_aware = false;
	}
}

void ext4_stop_discard_thread(struct ext4_sb_info *sbi)
{
	struct discard_cmd_control *dcc = sbi->dcc_info;

	if (dcc && dcc->ext4_issue_discard_thread) {
		struct task_struct *discard_thread = dcc->ext4_issue_discard_thread;

		dcc->ext4_issue_discard_thread = NULL;
		kthread_stop(discard_thread);
	}
}

static int issue_discard_thread(void *data)
{
	struct ext4_sb_info *sbi = data;
	struct discard_cmd_control *dcc = sbi->dcc_info;
	wait_queue_head_t *q = &dcc->discard_wait_queue;
	unsigned int wait_ms = DEF_MIN_DISCARD_ISSUE_TIME;
	int issued = 0;
	unsigned long interval = sbi->interval_time * HZ;
	long delta;

	set_freezable();

	do {
		if (!dcc->dpolicy_param_tune) {
			__init_discard_policy(sbi, &dcc->dpolicy, DPOLICY_BG,
						dcc->discard_granularity);
		}
		wait_event_interruptible_timeout(*q,
				kthread_should_stop() || freezing(current) ||
				dcc->discard_wake,
				msecs_to_jiffies(wait_ms));
		if (try_to_freeze())
			continue;
		if (ext4_readonly(sbi->s_sb))
			continue;
		if (kthread_should_stop())
			return 0;

		if (dcc->discard_wake)
			dcc->discard_wake = 0;

		sb_start_intwrite(sbi->s_sb);

		issued = ext4_trim_groups(sbi->s_sb, dcc);

		if (issued > 0) { /* issued >0 */
			wait_ms = dcc->dpolicy.min_interval;
		} else if ((issued == 0) && dcc->io_interrupted) { /* io_interrupted */
			dcc->io_interrupted = false;
			delta = (sbi->last_time + interval) - jiffies;
			if (delta > 0)
				wait_ms = jiffies_to_msecs(delta);
			else
				wait_ms = dcc->dpolicy.mid_interval;
		} else { /* issued <= 0, for idle or error */
			wait_ms = dcc->dpolicy.max_interval;
		}

		sb_end_intwrite(sbi->s_sb);
	} while (!kthread_should_stop());
	return 0;
}

int create_discard_cmd_control(struct ext4_sb_info *sbi)
{
	dev_t dev = sbi->s_sb->s_bdev->bd_dev;
	struct discard_cmd_control *dcc;
	int err = 0;

	if (sbi->dcc_info) {
		dcc = sbi->dcc_info;
		goto init_thread;
	}

	dcc = kzalloc(sizeof(struct discard_cmd_control), GFP_KERNEL);
	if (!dcc)
		return -ENOMEM;

	dcc->groups_block_bitmap = vmalloc((sbi->s_clusters_per_group/8) * sbi->s_groups_count); /* for backup groups block bitmap */
	if (!dcc->groups_block_bitmap)
		goto free_dcc;
	memset(dcc->groups_block_bitmap, 0xFF, (sbi->s_clusters_per_group/8) * sbi->s_groups_count);

	dcc->discard_granularity = DEFAULT_DISCARD_GRANULARITY;
	dcc->dpolicy_param_tune = 0;
	mutex_init(&dcc->cmd_lock);
	dcc->issued_discard = 0;
	dcc->max_discards = ext4_free_blocks_count(sbi->s_es);
	dcc->group_start = 0;	/* trim start group */
	dcc->blk_offset = 0;	/* group internal blk offset */

	init_waitqueue_head(&dcc->discard_wait_queue);
	sbi->dcc_info = dcc;
init_thread:
	dcc->ext4_issue_discard_thread = kthread_run(issue_discard_thread, sbi,
				"ext4_discard-%u:%u", MAJOR(dev), MINOR(dev));
	if (IS_ERR(dcc->ext4_issue_discard_thread)) {
		err = PTR_ERR(dcc->ext4_issue_discard_thread);
		vfree(dcc->groups_block_bitmap);
		kfree(dcc);
		sbi->dcc_info = NULL;
		return err;
	}

	return err;

free_dcc:
	kfree(dcc);
	return -ENOMEM;
}

void destroy_discard_cmd_control(struct ext4_sb_info *sbi)
{
	struct discard_cmd_control *dcc = sbi->dcc_info;

	if (!dcc)
		return;

	ext4_stop_discard_thread(sbi);

	if (dcc->groups_block_bitmap)
		vfree(dcc->groups_block_bitmap);
	kfree(dcc);
	sbi->dcc_info = NULL;
}

int ext4_seq_discard_info_show(struct seq_file *seq, void *v)
{
	struct super_block *sb = (struct super_block *) seq->private;
	struct ext4_sb_info *sbi = EXT4_SB(sb);

	if (v != SEQ_START_TOKEN)
		return 0;

	if (!test_opt(sb, ASYNC_DISCARD)) {
		seq_printf(seq, "async_discard option is closed !\n");
        return 0;
	}

	seq_printf(seq, "DCC info:\n  DCC granularity:%d\n  total issued discard:%lld\n  total trimed groups:%d\n",
		   sbi->dcc_info->discard_granularity,
		   sbi->dcc_info->issued_discard,
		   sbi->dcc_info->total_trimed_groups);
	seq_printf(seq, "  current triming group:%d\n  triming group internal offset:%d\n  cnt_io_interrupted:%lld\n",
		   sbi->dcc_info->group_start,
		   sbi->dcc_info->blk_offset,
		   sbi->dcc_info->cnt_io_interrupted);

	seq_printf(seq, "Dpolicy info:\n  discard type:%d\n  min_interval:%d\n  mid_interval:%d\n  max_interval:%d\n",
		   sbi->dcc_info->dpolicy.type, sbi->dcc_info->dpolicy.min_interval,
		   sbi->dcc_info->dpolicy.mid_interval, sbi->dcc_info->dpolicy.max_interval);
	seq_printf(seq, "  max_request per round:%d\n  io_aware:%s\n  io_aware_gran:%d\n  discard granularity:%d\n",
		   sbi->dcc_info->dpolicy.max_requests, sbi->dcc_info->dpolicy.io_aware ? "TRUE" : "FALSE",
		   sbi->dcc_info->dpolicy.io_aware_gran, sbi->dcc_info->dpolicy.granularity);

	seq_printf(seq, "EXT4 status:\n  ext4_utilization(free):%d\n", ext4_utilization(sbi));
	seq_printf(seq, "  is_ext4_idle:%s\n", is_ext4_idle(sbi) ? "idle" : "active");

	return 0;
}


