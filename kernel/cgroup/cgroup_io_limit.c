// SPDX-License-Identifier: GPL-2.1
/*
 * cgroup_iolimit.c -  control group iolimit subsystem
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2.1 of the GNU Lesser General Public License
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it would be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include "../../include/linux/iolimit_cgroup.h"
#include <linux/jiffies.h>

bool iolimit_enable;
bool iolimit_enable_registered = false;

static int is_need_iolimit(struct iolimit_cgroup *iolimitcg)
{
	int ret = 0;

	if (current_is_fg()) {
	    return 0;
	}


	ret = signal_pending_state(TASK_INTERRUPTIBLE, current);
	if (ret == 1) {
		return 0;
    }
	return atomic64_read(&iolimitcg->switching);
}

static bool is_write_need_wakeup(struct iolimit_cgroup *iolimitcg)
{
	int ret = false;

	if (atomic64_read(&iolimitcg->switching) == 0)
		ret = true;

	if (iolimitcg->write_part_nbyte > iolimitcg->write_already_used)
		ret = true;

	rcu_read_lock();
	if (iolimitcg != task_iolimitcg(current))
		ret = true;

	if (current_is_fg())
		ret = true;

	rcu_read_unlock();
	return ret;
}

static bool is_read_need_wakeup(struct iolimit_cgroup *iolimitcg)
{
	int ret = false;

	if (atomic64_read(&iolimitcg->switching) == 0)
		ret = true;

	if (iolimitcg->read_part_nbyte > iolimitcg->read_already_used)
		ret = true;

	rcu_read_lock();
	if (iolimitcg != task_iolimitcg(current))
		ret = true;

	if (current_is_fg())
		ret = true;

	rcu_read_unlock();
	pr_info("[iolimit] current: %s, uid: %d, ret: %d", current -> comm, current_uid().val, current_is_fg());
	return ret;
}

void do_io_write_bandwidth_control(size_t count)
{
	size_t may_io_cnt;
	struct iolimit_cgroup *iolimitcg;

repeat:
	rcu_read_lock();
	iolimitcg = task_iolimitcg(current);
	if (!is_need_iolimit(iolimitcg)) {
		rcu_read_unlock();
		return;
	}
	pr_info("[iolimit] need limit: %s, is_fg: %d", current ->comm, current_is_fg());

	spin_lock_bh(&iolimitcg->write_lock);
	may_io_cnt = iolimitcg->write_part_nbyte - iolimitcg->write_already_used;
	if (may_io_cnt < count) {
		spin_unlock_bh(&iolimitcg->write_lock);
		if (css_tryget_online(&iolimitcg->css)) {
			rcu_read_unlock();
			wait_event_interruptible_timeout(iolimitcg->write_wait,
				is_write_need_wakeup(iolimitcg), msecs_to_jiffies(125));
			css_put(&iolimitcg->css);
		} else {
			rcu_read_unlock();
		}
		goto repeat;
	} else if (may_io_cnt >= count) {
		may_io_cnt = count;
		iolimitcg->write_already_used += may_io_cnt;
	}

	spin_unlock_bh(&iolimitcg->write_lock);
	rcu_read_unlock();
}

void do_io_read_bandwidth_control(size_t count)
{
	size_t may_io_cnt;
	struct iolimit_cgroup *iolimitcg;

repeat:
	rcu_read_lock();
	iolimitcg = task_iolimitcg(current);
	if (!is_need_iolimit(iolimitcg)) {
		rcu_read_unlock();
		return;
	}

	spin_lock_bh(&iolimitcg->read_lock);
	may_io_cnt = iolimitcg->read_part_nbyte - iolimitcg->read_already_used;
	if (may_io_cnt < count) {
		spin_unlock_bh(&iolimitcg->read_lock);
		if (css_tryget_online(&iolimitcg->css)) {
			rcu_read_unlock();
			wait_event_interruptible_timeout(iolimitcg->read_wait,
				is_read_need_wakeup(iolimitcg), msecs_to_jiffies(125));
			css_put(&iolimitcg->css);
		} else {
			rcu_read_unlock();
		}

		if (task_in_pagefault(current))
			return;

		goto repeat;
	} else if (may_io_cnt >= count) {
		may_io_cnt = count;
		iolimitcg->read_already_used += may_io_cnt;
	}

	spin_unlock_bh(&iolimitcg->read_lock);
	rcu_read_unlock();
}

//static void write_timer_handler(unsigned long data)
static void write_timer_handler(struct timer_list *t)
{
//	struct iolimit_cgroup *iolimitcg = (struct iolimit_cgroup *)data;
	struct iolimit_cgroup *iolimitcg = from_timer(iolimitcg, t, write_timer);

	spin_lock_bh(&iolimitcg->write_lock);
	iolimitcg->write_already_used = 0;
	spin_unlock_bh(&iolimitcg->write_lock);
	wake_up_all(&iolimitcg->write_wait);
	mod_timer(&iolimitcg->write_timer, jiffies + HZ / 8);
}

//static void read_timer_handler(unsigned long data)
static void read_timer_handler(struct timer_list *t)
{
//	struct iolimit_cgroup *iolimitcg = (struct iolimit_cgroup *)data;
	struct iolimit_cgroup *iolimitcg = from_timer(iolimitcg, t, read_timer);

	spin_lock_bh(&iolimitcg->read_lock);
	iolimitcg->read_already_used = 0;
	spin_unlock_bh(&iolimitcg->read_lock);
	wake_up_all(&iolimitcg->read_wait);
	mod_timer(&iolimitcg->read_timer, jiffies + HZ / 8);
}

static int iolimit_enable_seq_show(struct seq_file *seq, void *p)
{
	seq_printf(seq, "%d\n", iolimit_enable?1:0);
	return 0;
}

static ssize_t iolimit_enable_write(struct file *filp, const char __user *ubuf,
	size_t cnt, loff_t *ppos)
{
	char buf[64] = { 0 };
	int user_set_value = 0;
	int ret = -1;

	if (cnt > sizeof(buf) - 1)
		cnt = sizeof(buf) - 1;

	if (copy_from_user(&buf[0], ubuf, cnt))
		return -EFAULT;

	ret = kstrtoint(strstrip(&buf[0]), 0, &user_set_value);

	if (ret < 0)
		return ret;

	iolimit_enable = !!user_set_value;
	return cnt;
}

static int iolimit_enable_open(struct inode *inode, struct file *file)
{
	return single_open(file, iolimit_enable_seq_show, NULL);
}

static const struct file_operations iolimit_enable_ops = {
	.open = iolimit_enable_open,
	.read = seq_read,
	.write = iolimit_enable_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static struct cgroup_subsys_state *iolimit_css_alloc(struct cgroup_subsys_state *parent)
{
	struct iolimit_cgroup *iolimitcg;

	iolimitcg = kzalloc(sizeof(struct iolimit_cgroup), GFP_KERNEL);
	if (!iolimitcg)
		return ERR_PTR(-ENOMEM);

	atomic64_set(&iolimitcg->switching, 0);

	atomic64_set(&iolimitcg->write_limit, 0);
	iolimitcg->write_part_nbyte = 0;
	iolimitcg->write_already_used = 0;
//	init_timer(&iolimitcg->write_timer);
//	iolimitcg->write_timer.data = (unsigned long)iolimitcg;
//	iolimitcg->write_timer.function = write_timer_handler;
	timer_setup(&iolimitcg->write_timer, write_timer_handler, TIMER_DEFERRABLE);
	spin_lock_init(&iolimitcg->write_lock);
	init_waitqueue_head(&iolimitcg->write_wait);

	atomic64_set(&iolimitcg->read_limit, 0);
	iolimitcg->read_part_nbyte = 0;
	iolimitcg->read_already_used = 0;
//	init_timer(&iolimitcg->read_timer);
//	iolimitcg->read_timer.data = (unsigned long)iolimitcg;
//	iolimitcg->read_timer.function = read_timer_handler;
	timer_setup(&iolimitcg->read_timer, read_timer_handler, TIMER_DEFERRABLE);
	spin_lock_init(&iolimitcg->read_lock);
	init_waitqueue_head(&iolimitcg->read_wait);
	if (!iolimit_enable_registered && !proc_create("iolimit_enable", 0644, NULL,
			 &iolimit_enable_ops)){
		pr_err("%s : Failed to register proc interface\n", __func__);
	} else {
		iolimit_enable_registered = true;
	}
	pr_info("IOLimit Debug:%s\n", __func__);
	return &iolimitcg->css;
}

static void iolimit_css_free(struct cgroup_subsys_state *css)
{
	struct iolimit_cgroup *iolimitcg = css_iolimit(css);

	del_timer_sync(&iolimitcg->write_timer);
	del_timer_sync(&iolimitcg->read_timer);
	kfree(css_iolimit(css));
}

static s64 iolimit_switching_read(struct cgroup_subsys_state *css, struct cftype *cft)
{
	struct iolimit_cgroup *iolimitcg = css_iolimit(css);

	return atomic64_read(&iolimitcg->switching);
}

static int iolimit_switching_write(struct cgroup_subsys_state *css, struct cftype *cft, s64 switching)
{
	struct iolimit_cgroup *iolimitcg = css_iolimit(css);
	int err = 0;

	if (switching != 0 && switching != 1) {
		err = -EINVAL;
		goto out;
	}
	atomic64_set(&iolimitcg->switching, switching);
	if (switching == 0) {
		wake_up_all(&iolimitcg->write_wait);
		del_timer_sync(&iolimitcg->write_timer);

		wake_up_all(&iolimitcg->read_wait);
		del_timer_sync(&iolimitcg->read_timer);
	} else if (switching == 1) {
		mod_timer(&iolimitcg->write_timer, jiffies + HZ / 8);
		iolimitcg->write_already_used = iolimitcg->write_part_nbyte;

		mod_timer(&iolimitcg->read_timer, jiffies + HZ / 8);
		iolimitcg->read_already_used = iolimitcg->read_part_nbyte;
	}
out:
	return err;
}

static s64 writeiolimit_read(struct cgroup_subsys_state *css, struct cftype *cft)
{
	struct iolimit_cgroup *iolimitcg = css_iolimit(css);

	return atomic64_read(&iolimitcg->write_limit);
}

static int writeiolimit_write(struct cgroup_subsys_state *css, struct cftype *cft, s64 limit)
{
	struct iolimit_cgroup *iolimitcg = css_iolimit(css);
	int err = 0;

	if (limit <= 0) {
		err = -EINVAL;
		goto out;
	}

	atomic64_set(&iolimitcg->write_limit, limit);
	spin_lock_bh(&iolimitcg->write_lock);
	iolimitcg->write_part_nbyte = limit / 8;
	spin_unlock_bh(&iolimitcg->write_lock);
out:
	return err;
}

static s64 readiolimit_read(struct cgroup_subsys_state *css, struct cftype *cft)
{
	struct iolimit_cgroup *iolimitcg = css_iolimit(css);

	return atomic64_read(&iolimitcg->read_limit);
}

static int readiolimit_write(struct cgroup_subsys_state *css, struct cftype *cft,
				s64 limit)
{
	struct iolimit_cgroup *iolimitcg = css_iolimit(css);
	int err = 0;

	if (limit <= 0) {
		err = -EINVAL;
		goto out;
	}

	atomic64_set(&iolimitcg->read_limit, limit);
	spin_lock_bh(&iolimitcg->read_lock);
	iolimitcg->read_part_nbyte = limit / 8;
	spin_unlock_bh(&iolimitcg->read_lock);
out:
	return err;
}

static void iolimit_attach(struct cgroup_taskset *tset)
{
	struct task_struct *task;
	struct cgroup_subsys_state *dst_css;

	cgroup_taskset_for_each(task, dst_css, tset)
		wake_up_process(task);
}

static struct cftype iolimit_files[] = {
	{
		.name = "switching",
		.flags = CFTYPE_NOT_ON_ROOT,
		.read_s64 = iolimit_switching_read,
		.write_s64 = iolimit_switching_write,
	},
	{
		.name = "write_limit",
		.flags = CFTYPE_NOT_ON_ROOT,
		.read_s64 = writeiolimit_read,
		.write_s64 = writeiolimit_write,
	},
	{
		.name = "read_limit",
		.flags = CFTYPE_NOT_ON_ROOT,
		.read_s64 = readiolimit_read,
		.write_s64 = readiolimit_write,
	},
	{}
};

struct cgroup_subsys iolimit_cgrp_subsys = {
	.css_alloc      = iolimit_css_alloc,
	.css_free       = iolimit_css_free,
	.attach         = iolimit_attach,
	.legacy_cftypes = iolimit_files,
};
