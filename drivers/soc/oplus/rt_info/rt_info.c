// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */

#include <linux/list.h>
#include <linux/jiffies.h>
#include <trace/events/sched.h>
#include <../kernel/sched/sched.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/mm.h>  
#include <linux/sched.h>  
#include <linux/types.h>  
#include <linux/errno.h>  
#include <linux/slab.h>  
#include <linux/gfp.h>  
#include <linux/spinlock.h>
#include <linux/pid_namespace.h>
#include <linux/module.h>

#ifdef CONFIG_OPLUS_FEATURE_RT_INFO
#include "rt_info.h"

static int rt_num = 0;
struct kmem_cache *rt_waker_cache = NULL;
struct proc_dir_entry *game_dir = NULL;
DEFINE_SPINLOCK(rt_info_lock);

/* render thread information
 */
struct render_thread_info_t {
	pid_t rt_tid;
	struct task_struct *rt_task;
	struct list_head waker_list;
} render_thread_info[MAX_RT_NUM];

/* render thread waker information
 */
struct rt_waker_info_t {
	struct list_head list;
	pid_t waker_tid;
	int count;
};

static void add_rt_waker_stat(pid_t waker_tid, struct render_thread_info_t *info) {
	struct list_head *pos, *n;
	struct rt_waker_info_t *new_waker;
	struct rt_waker_info_t *waker;

	list_for_each_safe(pos, n, &info->waker_list) {
		waker = list_entry(pos, struct rt_waker_info_t, list);
		if (waker->waker_tid == waker_tid) {
			if (waker->count % 30 == 0)
				rt_err("%d wake up %d %d times\n", waker_tid, info->rt_tid, waker->count);
			waker->count++;
			return;
		}
	}

	// add a new waker
	new_waker = kmem_cache_zalloc(rt_waker_cache, GFP_ATOMIC);
	if (new_waker) {
		rt_err("Add a new waker %d for %d\n", waker_tid, info->rt_tid);
		new_waker->waker_tid = waker_tid;
		new_waker->count = 1;
		list_add_tail(&new_waker->list, &info->waker_list);
	}
	// rt_err("Done\n");

	return;
}

/*
 * render_thread_info_handler is callback funtion called in core.c->try_to_wake_up
 */
static void render_thread_info_handler(void *taskp) {
	int i = 0;
	struct task_struct *task = (struct task_struct *)taskp;
	pid_t wakee_tid = 0, waker_tid = 0;
	// static int count = 0;

	if (rt_num <= 0)
		return;

	if (current != NULL) {
		waker_tid = current->pid;
	} else {
		rt_err("current is NULL\n");
		return;
	}

	if (taskp == NULL)
		return;

	wakee_tid = task->pid;

	//if (count % 10000 == 0)
	//    rt_err("%d:%s wake up %d:%s\n", waker_tid, current->comm, (int)task->pid, task->comm);

	// only update waker stat when lock is available,
	// if not available, skip these information
	if (spin_trylock(&rt_info_lock)) {
		for (i = 0; i < rt_num; i++) {
			if (wakee_tid == render_thread_info[i].rt_tid) {
				// found one wakeup
				// rt_err("waker: %d:%s, tgid: %d\n", current->pid, current->comm, current->tgid);
				// rt_err("wakee: %d:%s, tgid: %d\n", render_thread_info[i].rt_task->pid,
				//    render_thread_info[i].rt_task->comm,
				//    render_thread_info[i].rt_task->tgid);

				// waker and wakee belongs to same pid
				if (current->tgid == render_thread_info[i].rt_task->tgid) {
					add_rt_waker_stat(waker_tid, &render_thread_info[i]);
				}
			}
		}
		spin_unlock(&rt_info_lock);
	} else {
		rt_err("fail to get lock\n");
	}
}

static int proc_rt_num_show(struct seq_file *m, void *v) {
	int i;
	seq_printf(m, "%d: ", rt_num);
	for (i = 0; i < rt_num; i++) {
		seq_printf(m, "%d ", render_thread_info[i].rt_tid);
	}
	seq_printf(m, "\n");

	return 0;
}
static int proc_rt_sched_info_show(struct seq_file *m, void *v) {
	int i;
	struct list_head *pos, *n;
	struct rt_waker_info_t *waker;

	rt_err("%d render thread\n", rt_num);

	spin_lock(&rt_info_lock);
	for (i = 0; i < rt_num; i++) {
		list_for_each_safe(pos, n, &render_thread_info[i].waker_list) {
			waker = list_entry(pos, struct rt_waker_info_t, list);
			if (waker != NULL) {
				rt_err("waker: %d\n", waker->waker_tid);
				seq_printf(m, "%d %d %d\n", waker->waker_tid, 
						render_thread_info[i].rt_tid, waker->count);
			}
		}
	}
	spin_unlock(&rt_info_lock);

	return 0;
}

static int proc_rt_info_open(struct inode* inode, struct file *filp) {
	return single_open(filp, proc_rt_sched_info_show, inode);
}

static int proc_rt_num_open(struct inode* inode, struct file *filp) {
	return single_open(filp, proc_rt_num_show, inode);
}

static void free_waker_list(struct list_head *waker_list) {
	struct list_head *pos, *n;
	struct rt_waker_info_t *waker;

	list_for_each_safe(pos, n, waker_list) {
		waker = list_entry(pos, struct rt_waker_info_t, list);
		list_del(&waker->list);
		kmem_cache_free(rt_waker_cache, waker);
	}
	return;
}

static ssize_t proc_rt_info_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos) {
	char buffer[128] = {0, };
	char *iter = buffer;
	int i;
	struct task_struct *task;
	pid_t tid;

	memset(buffer, 0, sizeof(buffer));
	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;
	if (copy_from_user(buffer, buf, count)) {
		return -EFAULT;
	}

	rt_err("Input string %s\n", buffer);

	// input should be "123 234 345"

	// TODO: clear pervious rt stat, need to free rt_waker_info_t
    spin_lock(&rt_info_lock);
	for (i = 0; i < rt_num; i++) {
		put_task_stack(render_thread_info[i].rt_task);

		free_waker_list(&render_thread_info[i].waker_list);
	}
	memset(render_thread_info, 0, sizeof(render_thread_info));

    rt_num = 0;

	while (iter != NULL) {
		sscanf(iter, "%d", &tid);

		rt_err("read pid %d\n", tid);
		iter = strchr(iter + 1, ' ');

		rcu_read_lock();
		task = pid_task(find_pid_ns(tid, task_active_pid_ns(current)), PIDTYPE_PID);
		if (task)
			get_task_struct(task);
		rcu_read_unlock();

		if (task) {
			rt_err("find task %d:%s\n", task->pid, task->comm);
			render_thread_info[rt_num].rt_tid = tid;
			render_thread_info[rt_num].rt_task = task;
			INIT_LIST_HEAD(&render_thread_info[rt_num].waker_list);
		} else {
			rt_err("failed to find task_struct for %d\n", tid);
			continue;
		}

		rt_num++;
		if (rt_num > MAX_RT_NUM) {
			break;
		}
	}
	spin_unlock(&rt_info_lock);

	rt_err("total %d tid to track\n", rt_num);

	return count;
}

const struct file_operations proc_rt_info_operations = {
	.open		= proc_rt_info_open,
	.write		= proc_rt_info_write,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

const struct file_operations rt_num_operations = {
	.open		= proc_rt_num_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init rt_info_init(void) {
	struct proc_dir_entry *rt_sched_info;

	game_dir = proc_mkdir("game_opt", NULL);
	if (NULL == game_dir) {
		rt_err("fail to mkdir /proc/game_opt\n");
		return -ENOMEM;
	}

	rt_sched_info = proc_create_data("render_thread_info", 0770, game_dir, &proc_rt_info_operations, NULL);
	if (NULL == rt_sched_info) {
		rt_err("fail to create /proc/game_opt/render_thread_info\n");
		return -ENOMEM;        
	}

	proc_create_data("rt_num", 0770, game_dir, &rt_num_operations, NULL);

	rt_waker_cache = kmem_cache_create("rt_waker_cache", sizeof(struct render_thread_info_t), 0, 0, NULL);
	if (NULL == rt_waker_cache) {
		rt_err("fail to create rt_waker_cache\n");
		return -ENOMEM;        
	}

	register_rt_info_handler(render_thread_info_handler);

	return 0;
}

static void __exit rt_info_exit(void) {
	if (rt_waker_cache) {
		kmem_cache_destroy(rt_waker_cache);
	}

	remove_proc_entry("render_thread_info", game_dir);
	remove_proc_entry("game_opt", NULL);
}


module_init(rt_info_init);
module_exit(rt_info_exit);

MODULE_DESCRIPTION("RT info for game optimization");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("JiangFei@GameOpt");
#endif
