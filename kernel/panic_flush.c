/***********************************************************
** Copyright (C), 2008-2019, OPLUS Mobile Comm Corp., Ltd.
** VENDOR_EDIT
** File: - panic_flush.c
** Description:  code to flush device cache in panic
**
** Version: 1.0
** Date : 2019/08/27
**
** ------------------ Revision History:------------------------
** <author> <data> <version > <desc>
** yanwu 2019/08/27 1.0  add the file
****************************************************************/

#define DEBUG
#include <linux/wait.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/blkdev.h>
#include <linux/kthread.h>
#include <linux/workqueue.h>

#define PANIC_FLUSH_POLL_MS (10)


struct panic_flush_control {
	struct task_struct *flush_thread;
	wait_queue_head_t flush_wq;
	atomic_t flush_issuing;
	atomic_t flush_issued;
};

static struct panic_flush_control *pfc;
static void panic_issue_flush(struct super_block *sb , void *arg)
{
	int ret = -1;
	int *flush_count = (int *)arg;
	if (!(sb->s_flags & MS_RDONLY) && NULL != sb->s_bdev) {
		ret = blkdev_issue_flush(sb->s_bdev, GFP_KERNEL, NULL);
	}
	if (!ret) {
		(*flush_count)++;
		pr_emerg("blkdev_issue_flush before panic return %d\n", *flush_count);
	}
}

static int panic_flush_thread(void *data)
{
	int flush_count = 0;
repeat:
	if (kthread_should_stop())
		return 0;
	wait_event(pfc->flush_wq, kthread_should_stop() ||
			atomic_read(&pfc->flush_issuing) > 0);
	if (atomic_read(&pfc->flush_issuing) > 0) {
		iterate_supers(panic_issue_flush, &flush_count);
		pr_emerg("Up to now, total %d panic_issue_flush_count\n", flush_count);
		atomic_inc(&pfc->flush_issued);
		atomic_dec(&pfc->flush_issuing);
	}
	goto repeat;
}

extern bool is_fulldump_enable(void);

static inline bool need_flush_device_cache(void)
{
	if (is_fulldump_enable())
		return false;
	return true;
}

int panic_flush_device_cache(int timeout)
{
	pr_emerg("%s\n", __func__);
	if (!need_flush_device_cache()) {
		pr_emerg("%s: skip flush device cache\n", __func__);
		return timeout;
	}

	if (atomic_inc_return(&pfc->flush_issuing) == 1 &&
		waitqueue_active(&pfc->flush_wq)) {
		pr_emerg("%s: flush device cache\n", __func__);
		atomic_set(&pfc->flush_issued, 0);
		wake_up(&pfc->flush_wq);
		while (timeout > 0 && atomic_read(&pfc->flush_issued) == 0) {
			mdelay(PANIC_FLUSH_POLL_MS);
			timeout -= PANIC_FLUSH_POLL_MS;
		}
		pr_emerg("%s: remaining timeout = %d\n", __func__, timeout);
	}
	return timeout;
}

static int __init create_panic_flush_control(void)
{
	int err = 0;
	pr_debug("%s\n", __func__);
	pfc = kzalloc(sizeof(*pfc), GFP_KERNEL);
	if (!pfc) {
		pr_err("%s: fail to allocate memory\n", __func__);
		return -ENOMEM;
	}

	init_waitqueue_head(&pfc->flush_wq);
	atomic_set(&pfc->flush_issuing, 0);
	atomic_set(&pfc->flush_issued, 0);
	pfc->flush_thread = kthread_run(panic_flush_thread, pfc, "panic_flush");
	if (IS_ERR(pfc->flush_thread)) {
		err = PTR_ERR(pfc->flush_thread);
		kfree(pfc);
		pfc = NULL;
	}
	return err;
}

static void __exit destroy_panic_flush_control(void)
{
	pr_debug("%s\n", __func__);
	if (pfc && pfc->flush_thread) {
		pr_debug("%s: stop panic_flush thread\n", __func__);
		kthread_stop(pfc->flush_thread);
		kfree(pfc);
		pfc = NULL;
	}
}
module_init(create_panic_flush_control);
module_exit(destroy_panic_flush_control);
MODULE_DESCRIPTION("OPLUS panic flush control");
MODULE_LICENSE("GPL v2");
