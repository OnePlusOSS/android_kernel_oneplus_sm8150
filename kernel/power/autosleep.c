// SPDX-License-Identifier: GPL-2.0
/*
 * kernel/power/autosleep.c
 *
 * Opportunistic sleep support.
 *
 * Copyright (C) 2012 Rafael J. Wysocki <rjw@sisk.pl>
 */

#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/pm_wakeup.h>

#include "power.h"

static suspend_state_t autosleep_state;
static struct workqueue_struct *autosleep_wq;
/*
 * Note: it is only safe to mutex_lock(&autosleep_lock) if a wakeup_source
 * is active, otherwise a deadlock with try_to_suspend() is possible.
 * Alternatively mutex_lock_interruptible() can be used.  This will then fail
 * if an auto_sleep cycle tries to freeze processes.
 */
static DEFINE_MUTEX(autosleep_lock);
static struct wakeup_source *autosleep_ws;

#ifdef OPLUS_FEATURE_POWERINFO_STANDBY
static void wakelock_printk(struct work_struct *work);
static struct workqueue_struct *wakelock_printk_work_queue = NULL;
static DECLARE_DELAYED_WORK(wakelock_printk_work, wakelock_printk);
static void wakelock_printk(struct work_struct *work)
{
	pr_info("%s for debug\n", __func__);
	pm_print_active_wakeup_sources();
	queue_delayed_work(wakelock_printk_work_queue, &wakelock_printk_work, msecs_to_jiffies(50*1000));
}

void wakelock_printk_control(int on) 
{
	if (wakelock_printk_work_queue == NULL) {
		printk(KERN_INFO"%s: wakelock_printk_work_queue is NULL, do nothing\n", __func__);
		return;
	}
	if (on) {
		queue_delayed_work(wakelock_printk_work_queue, &wakelock_printk_work, msecs_to_jiffies(50*1000));
	} else {
		cancel_delayed_work(&wakelock_printk_work);
	}
}
#endif /* VENDOR_EDIT */

static void try_to_suspend(struct work_struct *work)
{
	unsigned int initial_count, final_count;

	if (!pm_get_wakeup_count(&initial_count, true))
		goto out;

	mutex_lock(&autosleep_lock);

	if (!pm_save_wakeup_count(initial_count) ||
		system_state != SYSTEM_RUNNING) {
		mutex_unlock(&autosleep_lock);
		goto out;
	}

	if (autosleep_state == PM_SUSPEND_ON) {
		mutex_unlock(&autosleep_lock);
		return;
	}
	if (autosleep_state >= PM_SUSPEND_MAX)
		hibernate();
	else
		pm_suspend(autosleep_state);

	mutex_unlock(&autosleep_lock);

	if (!pm_get_wakeup_count(&final_count, false))
		goto out;

	/*
	 * If the wakeup occured for an unknown reason, wait to prevent the
	 * system from trying to suspend and waking up in a tight loop.
	 */
	if (final_count == initial_count)
		schedule_timeout_uninterruptible(HZ / 2);

 out:
	queue_up_suspend_work();
}

static DECLARE_WORK(suspend_work, try_to_suspend);

void queue_up_suspend_work(void)
{
	if (autosleep_state > PM_SUSPEND_ON)
		queue_work(autosleep_wq, &suspend_work);
}

suspend_state_t pm_autosleep_state(void)
{
	return autosleep_state;
}

int pm_autosleep_lock(void)
{
	return mutex_lock_interruptible(&autosleep_lock);
}

void pm_autosleep_unlock(void)
{
	mutex_unlock(&autosleep_lock);
}

int pm_autosleep_set_state(suspend_state_t state)
{

#ifndef CONFIG_HIBERNATION
	if (state >= PM_SUSPEND_MAX)
		return -EINVAL;
#endif

#ifdef OPLUS_FEATURE_POWERINFO_STANDBY
		wakelock_printk_control(0);
#endif /* VENDOR_EDIT */


	__pm_stay_awake(autosleep_ws);

	mutex_lock(&autosleep_lock);

	autosleep_state = state;

	__pm_relax(autosleep_ws);

	if (state > PM_SUSPEND_ON) {
		pm_wakep_autosleep_enabled(true);
		queue_up_suspend_work();
	} else {
		pm_wakep_autosleep_enabled(false);
	}

	mutex_unlock(&autosleep_lock);

#ifdef OPLUS_FEATURE_POWERINFO_STANDBY
	wakelock_printk_control(1); 
#endif /* VENDOR_EDIT */

	return 0;
}

int __init pm_autosleep_init(void)
{
#ifdef OPLUS_FEATURE_POWERINFO_STANDBY
	wakelock_printk_work_queue = create_singlethread_workqueue("wakelock_printk");
	if (wakelock_printk_work_queue == NULL)
		printk(KERN_INFO "%s: failed to create work queue\n", __func__);
	wakelock_printk_control(1);
#endif /* VENDOR_EDIT */

	autosleep_ws = wakeup_source_register(NULL, "autosleep");
	if (!autosleep_ws)
		return -ENOMEM;

	autosleep_wq = alloc_ordered_workqueue("autosleep", 0);
	if (autosleep_wq)
		return 0;

	wakeup_source_unregister(autosleep_ws);
	return -ENOMEM;
}
