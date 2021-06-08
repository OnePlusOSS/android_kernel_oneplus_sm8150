#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched/signal.h>
#include <linux/oem/tpd.h>
#include <linux/cred.h>
#include <linux/oem/im.h>
#include <linux/list.h>
#include <linux/slab.h>

/*
 * Task Placement Decision
 *
 * two main rules as below:
 *  1. trigger tpd with tgid and thread's name which will be limited
 *     through online config, framework can tag threads and limit cpu placement
 *     of tagged threads that are also foreground task.
 *     tpd_cmds
 *  2. trigger tpd with tgid and thread_id
 *     control the placement limitation by itself, set tpd_ctl = 1 will limit
 *     cpu placement of tagged threads, but need release by itself to set tpd_enable = 0,
 *     tpd_ctl = 0 and task->tpd = 0
 *     tpd_id
 *  3. control the placement limitation by itself, set tpd_ctl = 1 will limit
 *     cpu placement of tagged threads, but need release by itself to set tpd_enable = 0,
 *     tpd_ctl = 0 and task->tpd = 0
 *     tpd_dynamic
 */

struct tgid_list_entry {
	int pid;
	struct list_head node;
};

struct monitor_gp {
	int decision; /* cpu affinity  */
	spinlock_t tgid_list_lock; /* used to check dynamic tpd task */
	struct list_head tgid_head;/* used to check dynamic tpd task */
	spinlock_t miss_list_lock; /* used to check if need re-tag or not */
	unsigned int miss_list_tgid[MAX_MISS_LIST];/* used to check if need re-tag or not */
	char miss_list[MAX_MISS_LIST][TASK_COMM_LEN];/* used to check if need re-tag or not */
	int not_yet;
	int cur_idx;
};

/* monitor group for dynamic tpd threads */
static struct monitor_gp mgp[TPD_GROUP_MAX];

static atomic_t tpd_enable_rc = ATOMIC_INIT(0);
static int tpd_enable_rc_show(char *buf, const struct kernel_param *kp)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&tpd_enable_rc));
}

static struct kernel_param_ops tpd_enable_rc_ops = {
	.get = tpd_enable_rc_show,
};

module_param_cb(tpd_en_rc, &tpd_enable_rc_ops, NULL, 0664);

static int tpd_log_lv = 2;
module_param_named(log_lv, tpd_log_lv, int, 0664);

static atomic_t tpd_ctl = ATOMIC_INIT(0); /* used to ignore fg task checking */

static bool should_update_tpd_enable(int enable) {

	bool ret = true;
	int refcnt = 0;

	if (enable) {
		if (atomic_inc_return(&tpd_enable_rc) > 1)
			ret = false;
	} else {
		refcnt = atomic_read(&tpd_enable_rc);
		if (refcnt < 1) {
			ret = false;
			goto out;
		}

		/* tpd_enable ref count must greater or equal tpd_ctl */
		if (refcnt - 1 < atomic_read(&tpd_ctl)) {
			ret = false;
			goto out;
		}

		if (atomic_dec_return(&tpd_enable_rc) > 0) {
			tpd_loge("tpd cannot disable");
			ret = false;
		}
	}

out:
	tpd_logv("should_update_tpd_enable? %d", ret);

	return ret;
}

static int tpd_enable = 0;
static int tpd_enable_store(const char *buf, const struct kernel_param *kp)
{
	int val;

	if (sscanf(buf, "%d\n", &val) <= 0)
		return 0;

	if (should_update_tpd_enable(val))
		tpd_enable = val;

	return 0;
}

static int tpd_enable_show(char *buf, const struct kernel_param *kp)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", tpd_enable);
}

static struct kernel_param_ops tpd_enable_ops = {
	.set = tpd_enable_store,
	.get = tpd_enable_show,
};

module_param_cb(tpd_enable, &tpd_enable_ops, NULL, 0664);

bool is_tpd_enable(void)
{
	return tpd_enable;
}

static int miss_list_show(char *buf, const struct kernel_param *kp)
{
	int i, j;
	int cnt = 0;

	for (j = TPD_GROUP_MEDIAPROVIDER; j < TPD_GROUP_MAX; ++j) {
		spin_lock(&mgp[j].miss_list_lock);
		if (mgp[j].not_yet > 0) {
			for (i = 0; i < MAX_MISS_LIST; ++i) {
				if(strlen(mgp[j].miss_list[i]) > 0) {
					cnt +=	snprintf(buf + cnt, PAGE_SIZE - cnt, "%s %d\n", mgp[j].miss_list[i], mgp[j].miss_list_tgid[i]);
				}
			}
		}
		spin_unlock(&mgp[j].miss_list_lock);
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "\n");
	}
	return cnt;
}

static struct kernel_param_ops miss_list_ops = {
	.get = miss_list_show,
};

module_param_cb(miss_list, &miss_list_ops, NULL, 0664);

bool is_dynamic_tpd_task(struct task_struct *tsk)
{
	int gid, len = 0, i;
	bool ret = false;
	struct monitor_gp *group;
	struct tgid_list_entry *p;
	struct task_struct *leader;

	/* this is for all tpd tasks reset(dynamic/not dynamic tpd tasks) */
	if (!tpd_enable) {
		tsk->tpd = 0;
		tsk->dtpdg = -1;
		tsk->dtpd = 0;
		return ret;
	}

	leader = tsk ? tsk->group_leader : NULL;
	if (leader == NULL)
		return ret;

	gid = leader->dtpdg;
	/* not dynamic tpd task will return */
	if (gid < TPD_GROUP_MEDIAPROVIDER || gid > TPD_GROUP_MAX)
		return ret;

	group = &mgp[gid];

	switch (gid) {
	case TPD_GROUP_MEDIAPROVIDER:

		spin_lock(&group->tgid_list_lock);

		/* no dynamic tpd task enable of this dynamic tpd group id */
		if (list_empty(&group->tgid_head)) {
			if (tsk->dtpd) {
				tsk->dtpd = 0;
				tsk->tpd = 0;
			}
			spin_unlock(&group->tgid_list_lock);
			return ret;
		}

		list_for_each_entry(p, &group->tgid_head, node) {
			if (leader->pid != p->pid)
				continue;

			/* parent of thread was dynamic tpd task */
			/* dynamic tpd task has already tagged */
			if (tsk->dtpd && tsk->tpd) {
				ret = true;
				break;
			}

			/* start tagging process */
#ifdef CONFIG_IM
			/*binder thread of media provider */
			if (im_binder(tsk)) {
				tsk->dtpd = 1; /* dynamic tpd */
				tsk->tpd = group->decision;
				ret = true;
				break;
			}
#endif

#ifdef CONFIG_ONEPLUS_FG_OPT
			#if 0
			/* fuse related thread of media provider */
			if (tsk->fuse_boost) {
				tsk->dtpd = 1; /* dynamic tpd */
				tsk->tpd = group->decision;
				ret = true;
				break;
			}
			#endif
#endif
			/* re-tag missed thread, only one name with  one thread */
			spin_lock(&group->miss_list_lock);
			if (group->not_yet > 0) {
				for (i = 0; i < MAX_MISS_LIST; ++i) {
					len = strlen(group->miss_list[i]);
					if (len == 0)
						continue;
					if (group->miss_list_tgid[i] != p->pid)
						continue;
					if (!strncmp(tsk->comm, group->miss_list[i], len)) {
						strcpy(group->miss_list[i], "");
						group->miss_list_tgid[i] = 0;
						tsk->dtpd = 1;
						tsk->tpd = group->decision;
						group->not_yet--;
						ret = true;
						break;
					}
				}

				if (ret) {
					spin_unlock(&group->miss_list_lock);
					break;
				}
			}
			spin_unlock(&group->miss_list_lock);
			/* end tagging process */
		}
		spin_unlock(&group->tgid_list_lock);

		/* reset flag if dynamic tpd task removed (tgid was removed) */
		if (!ret) {
			if (tsk->dtpd) {
				tsk->dtpd = 0;
				tsk->tpd = 0;
			}
		}
		break;
	default:
		break;
	}

	return ret;
}

static void set_tpd_ctl(int force)
{
	if (force)
		atomic_inc(&tpd_ctl);
	else {
		if (atomic_read(&tpd_ctl) > 0)
			atomic_dec(&tpd_ctl);
	}
}

static int tpd_ctl_store(const char *buf, const struct kernel_param *kp)
{
	int val;

	if (sscanf(buf, "%d\n", &val) <= 0)
		return 0;

	set_tpd_ctl(val);

	return 0;
}

static int tpd_ctl_show(char *buf, const struct kernel_param *kp)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&tpd_ctl));
}

static struct kernel_param_ops tpd_ctl_ops = {
	.set = tpd_ctl_store,
	.get = tpd_ctl_show,
};

module_param_cb(tpd_ctl, &tpd_ctl_ops, NULL, 0664);

static inline void tagging(struct task_struct *tsk, int decision)
{
	if (tsk == NULL) {
		tpd_loge("task cannot set");
		return;
	}

	tpd_logv("%s task: %s pid:%d decision:%d\n", __func__, tsk->comm, tsk->pid, decision);

	tsk->tpd = decision;
}

static inline void tagging_by_name(struct task_struct *tsk, char* name, int decision, int *cnt)
{
	size_t tlen = 0, len = 0;

	tlen = strlen(name);
	if (tlen == 0)
		return;

	len = strlen(tsk->comm);

	if (len != tlen)
		return;

	if (!strncmp(tsk->comm, name, tlen)) {
		tpd_logv("%s task: %s pid:%d decision:%d name=%s\n", __func__, tsk->comm, tsk->pid, decision, name);
		tsk->tpd = decision;
		*cnt = *cnt + 1;
	}
}

static void tag_from_tgid(unsigned int tgid, int decision, char* thread_name, int *cnt)
{
	struct task_struct *p, *t;

	rcu_read_lock();
	p = find_task_by_vpid(tgid);
	if (p) {
		for_each_thread(p, t)
			tagging_by_name(t, thread_name, decision, cnt);
	}
	rcu_read_unlock();

}

static inline void dy_tagging_by_name(struct task_struct *tsk, const char* name, int decision, int dtpd, int *cnt)
{
	size_t tlen = 0, len = 0;

	tlen = strlen(name);
	if (tlen == 0)
		return;

	len = strlen(tsk->comm);

	if (len != tlen)
		return;

	if (!strncmp(tsk->comm, name, tlen)) {
		tpd_logv("%s task: %s pid:%d decision:%d name=%s, dtpd=%d\n", __func__, tsk->comm, tsk->pid, decision, name, dtpd);
		tsk->tpd = decision;
		tsk->dtpd = dtpd;
		*cnt = *cnt + 1;
	}
}

static void dy_tag_from_tgid(unsigned int tgid, int decision, const char* thread_name, int dtpd, int *cnt)
{
	struct task_struct *p, *t;

	rcu_read_lock();
	p = find_task_by_vpid(tgid);
	if (p) {
		for_each_thread(p, t)
			dy_tagging_by_name(t, thread_name, decision, dtpd, cnt);
	}
	rcu_read_unlock();

}

/* set dtpd group id in the main thread */
static void tag_dtpdg(unsigned int tgid, unsigned int dtpdg)
{
	struct task_struct *p;

	rcu_read_lock();
	p = find_task_by_vpid(tgid);
	if (p) {
		p->dtpdg = dtpdg;
	}
	rcu_read_unlock();

}

static int tpd_cmd_store(const char *buf, const struct kernel_param *kp)
{
	unsigned int tgid = 0;
	int tp_decision = -1;
	char threads[MAX_THREAD_INPUT][TASK_COMM_LEN] = {{0}, {0}, {0}, {0}, {0}, {0}};
	int ret, i, cnt = 0;

	ret = sscanf(buf, "%u %d %s %s %s %s %s %s\n",
		&tgid, &tp_decision,
		threads[0], threads[1], threads[2], threads[3], threads[4], threads[5]);

        tpd_logi("tpd params: %u %d %s %s %s %s %s %s, from %s %d, total=%d\n",
              tgid, tp_decision, threads[0], threads[1], threads[2], threads[3], threads[4], threads[5],
              current->comm, current->pid, ret);

	for (i = 0; i < MAX_THREAD_INPUT; i++) {
		if (strlen(threads[i]) > 0)
			tag_from_tgid(tgid, tp_decision, threads[i], &cnt);
	}

	tpd_logv("tpd tagging count = %d\n", cnt);

	return 0;
}

static struct kernel_param_ops tpd_cmd_ops = {
	.set = tpd_cmd_store,
};
module_param_cb(tpd_cmds, &tpd_cmd_ops, NULL, 0664);

static void tag_from_tid(unsigned int pid, unsigned int tid, int decision)
{
	struct task_struct *p;

	rcu_read_lock();
	p = find_task_by_vpid(tid);
	if (p) {
		if (p->group_leader && (p->group_leader->pid == pid)) {
			tpd_logv("tpd tagging task pid= %d\n", pid);
			tagging(p, decision);
		}
	} else {
		tpd_loge("cannot find task!!! pid = %d", tid);
	}
	rcu_read_unlock();
}

static int tpd_store(const char *buf, const struct kernel_param *kp)
{
	unsigned int pid = 0;
	unsigned int tid = 0;
	int tpdenable = 0;
	int tp_decision = -1;
	int ret;

	ret = sscanf(buf, "%u,%u,%d,%d\n",
		&pid, &tid, &tpdenable, &tp_decision);

	tpd_logi("tpd param pid:%u tid:%u, tpd_enable:%d decision:%d from %s %d\n",
		pid, tid, tpdenable, tp_decision, current->comm, current->pid);

	if (ret != 4) {
		tpd_loge("Invalid params!!!!!!");
		return 0;
	}

	tag_from_tid(pid, tid, tpdenable ? tp_decision : 0);

	set_tpd_ctl(tpdenable);

	/* update tpd_enable ref cnt*/
	if (should_update_tpd_enable(tpdenable))
		tpd_enable = tpdenable;

	return 0;
}

static struct kernel_param_ops tpd_ops = {
	.set = tpd_store,
};
module_param_cb(tpd_id, &tpd_ops, NULL, 0664);

static void tgid_list_add(struct monitor_gp *_mgp, int pid)
{
	struct tgid_list_entry *p;

	p = kmalloc(sizeof(struct tgid_list_entry), GFP_KERNEL);
	if (p == NULL)
		return;
	p->pid = pid;
	INIT_LIST_HEAD(&p->node);

	spin_lock(&_mgp->tgid_list_lock);
	list_add_tail(&p->node, &_mgp->tgid_head);
	tpd_logv("add main thread: %d", pid);
	spin_unlock(&_mgp->tgid_list_lock);
}

void tpd_tglist_del(struct task_struct *tsk)
{
	struct tgid_list_entry *p, *next;
	struct monitor_gp *group;

	if (!tsk->pid)
		return;

	if (tsk->dtpdg < 0 || tsk->dtpdg >= TPD_GROUP_MAX)
		return;

	group = &mgp[tsk->dtpdg];

	spin_lock(&group->tgid_list_lock);

	if (list_empty(&group->tgid_head))
		goto unlock;

	list_for_each_entry_safe(p, next, &group->tgid_head, node) {
		if (p != NULL && (p->pid == tsk->pid)) {
			list_del_init(&p->node);
			tpd_logv("remove task: %d", tsk->pid);
			kfree(p);
			break;
		}
	}

	/* if no tgid in list, disable tpd_ctl && try to disable tpd */
	if (list_empty(&group->tgid_head)) {
		set_tpd_ctl(0);
		if (should_update_tpd_enable(0))
			tpd_enable = 0;
	}

unlock:
	spin_unlock(&group->tgid_list_lock);
}

/* return true if list size is from one to empty */
static bool tgid_list_del(struct monitor_gp *_mgp, int pid)
{
	struct tgid_list_entry *p, *next;
	bool ret = false;

	spin_lock(&_mgp->tgid_list_lock);
	/* do nothing if list empty */
	if (list_empty(&_mgp->tgid_head))
		goto unlock;

	list_for_each_entry_safe(p, next, &_mgp->tgid_head, node) {
		if (p != NULL && (p->pid == pid)) {
			list_del_init(&p->node);
			tpd_logv("remove main thread: %d", pid);
			kfree(p);
			break;
		}
	}
	/* re-check if list is empty after list deletion */
	if (list_empty(&_mgp->tgid_head))
		ret = true;

unlock:
	spin_unlock(&_mgp->tgid_list_lock);
	return ret;
}

static int list_show(char *buf, const struct kernel_param *kp)
{
	struct tgid_list_entry *p;
	int cnt = 0;
	int i;

	for (i = 0; i < TPD_GROUP_MAX; ++i) {
		spin_lock(&mgp[i].tgid_list_lock);

		if (list_empty(&mgp[i].tgid_head))
			goto unlock;

		list_for_each_entry(p, &mgp[i].tgid_head, node) {
			cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "%d %d\n", i, p->pid);
		}
unlock:
		spin_unlock(&mgp[i].tgid_list_lock);
	}

	return cnt;
}

static struct kernel_param_ops tgid_list_ops = {
	.get = list_show,
};

module_param_cb(tgid_list, &tgid_list_ops, NULL, 0664);

#define MONITOR_THREAD_NUM 1
static int tpd_process_trigger_store(const char *buf, const struct kernel_param *kp)
{
	unsigned int tgid = 0;
	int tpdenable = 0;
	int tp_decision = -1;
	int tpd_group = -1;
	int ret, i, cnt = 0, j;
	const char *threads[MONITOR_THREAD_NUM] = {"bg"};
	struct monitor_gp *group;

	ret = sscanf(buf, "%d,%u,%d,%d\n",
		&tpd_group, &tgid, &tpdenable, &tp_decision);

	tpd_logi("tpd param group:%d pid:%u tpd_enable:%d decision:%d from %s %d\n",
		tpd_group, tgid, tpdenable, tp_decision, current->comm, current->pid);

	if (ret != 4) {
		tpd_loge("Invalid params!!!!!!");
		return 0;
	}

	if (tpd_group >= TPD_GROUP_MAX || tpd_group < 0) {
		tpd_loge("Invalid group!!!!!!");
		return 0;
	}

	group = &mgp[tpd_group];

	if (!tpdenable) {
		if (tgid_list_del(group, tgid))
			group->decision = 0;

		tag_dtpdg(tgid, -1);
	} else {
		group->decision = tp_decision;
		tgid_list_add(group, tgid);
		tag_dtpdg(tgid, tpd_group);
	}

	set_tpd_ctl(tpdenable);


	/* tagged by name from group: media provider */
	if (tpd_group == TPD_GROUP_MEDIAPROVIDER) {
		for(i = 0; i < MONITOR_THREAD_NUM; i++) {
			cnt = 0;
			dy_tag_from_tgid(tgid, tpdenable ? tp_decision : 0, threads[i], tpdenable, &cnt);
			/* can't find thread to tag/un-tag */
			if (cnt == 0) {
				spin_lock(&group->miss_list_lock);
				if (tpdenable) {
					/* need re-tag, add thread name into miss_list */
					strncpy(group->miss_list[group->cur_idx], threads[i], TASK_COMM_LEN);
					group->miss_list_tgid[group->cur_idx] = tgid;
					group->cur_idx = (++group->cur_idx) % MAX_MISS_LIST;
					group->not_yet++;
				} else {
					/* dynmic tpd disabled when miss_list still have thread need to tag, we clear the miss list */
					for (j = 0; j < MAX_MISS_LIST; ++j) {
						if (group->miss_list_tgid[j] == tgid) {
							strcpy(group->miss_list[j], "");
							group->miss_list_tgid[j] = 0;
							group->not_yet--;
						}
					}
				}
				spin_unlock(&group->miss_list_lock);
			}
		}
	}

	tpd_logv("tagging count = %d, tpd enable set:%d", cnt, tpdenable);

	/* update tpd_enable by ref cnt */
	if (should_update_tpd_enable(tpdenable))
		tpd_enable = tpdenable;

	return 0;
}

static struct kernel_param_ops tpd_pt_ops = {
	.set = tpd_process_trigger_store,
};
module_param_cb(tpd_dynamic, &tpd_pt_ops, NULL, 0664);

int tpd_suggested(struct task_struct* tsk, int min_idx, int mid_idx, int max_idx, int request_cpu)
{
	int suggest_cpu = request_cpu;

	if (!(task_is_fg(tsk) || atomic_read(&tpd_ctl)))
		goto out;

	switch (tsk->tpd) {
	case TPD_TYPE_S:
	case TPD_TYPE_GS:
	case TPD_TYPE_PS:
	case TPD_TYPE_PGS:
		suggest_cpu = min_idx;
		break;
	case TPD_TYPE_G:
	case TPD_TYPE_PG:
		suggest_cpu = mid_idx;
		break;
	case TPD_TYPE_P:
		suggest_cpu = max_idx;
		break;
	default:
		break;
	}
out:
	tpd_logi("pid = %d: comm = %s, tpd = %d, suggest_cpu = %d, task is fg? %d, tpd_ctl = %d\n", tsk->pid, tsk->comm,
		tsk->tpd, suggest_cpu, task_is_fg(tsk), atomic_read(&tpd_ctl));
	return suggest_cpu;
}

void tpd_mask(struct task_struct* tsk, int min_idx, int mid_idx, int max_idx, cpumask_t *request, int nrcpu)
{
	int start_idx = nrcpu, end_idx = -1, i, next_start_idx = nrcpu;
	bool second_round = false;

	if (!(task_is_fg(tsk) || atomic_read(&tpd_ctl))) {
		if (!task_is_fg(tsk))
			tpd_logv("task is not fg!!!\n");
		return;
	}

	switch (tsk->tpd) {
	case TPD_TYPE_S:
		start_idx = mid_idx;
		break;
	case TPD_TYPE_G:
		start_idx = min_idx;
		end_idx = mid_idx;
		second_round = true;
		next_start_idx = max_idx;
		break;
	case TPD_TYPE_GS:
		start_idx = max_idx;
		break;
	case TPD_TYPE_P:
		start_idx = min_idx;
		end_idx = max_idx;
		break;
	case TPD_TYPE_PS:
		start_idx = mid_idx;
		end_idx = max_idx;
		break;
	case TPD_TYPE_PG:
		start_idx = min_idx;
		end_idx = mid_idx;
		break;
	default:
		break;
	}

redo:
	for (i = start_idx; i < nrcpu; ++i) {

		if (i == end_idx)
			break;

		tpd_logv("task: %d, cpu clear bit = %d\n", (tsk) ? tsk->pid : -1, i);

		cpumask_clear_cpu(i, request);
	}

	if (second_round) {
		start_idx = next_start_idx;
		second_round = false;
		goto redo;
	}

	tpd_logi("pid = %d: comm = %s, tpd = %d, min_idx = %d, mid_idx = %d, max_idx = %d, task is fg? %d, tpd_ctl = %d\n", tsk->pid, tsk->comm,
		tsk->tpd, min_idx, mid_idx, max_idx, task_is_fg(tsk), atomic_read(&tpd_ctl));
}

bool tpd_check(struct task_struct *tsk, int dest_cpu, int min_idx, int mid_idx, int max_idx)
{
	bool mismatch = false;

	if (!(task_is_fg(tsk) || atomic_read(&tpd_ctl)))
		goto out;

	switch (tsk->tpd) {
	case TPD_TYPE_S:
		if (dest_cpu >= mid_idx)
			mismatch = true;
		break;
	case TPD_TYPE_G:
		if ((mid_idx != max_idx) &&
				(dest_cpu < mid_idx || dest_cpu >= max_idx))
			mismatch = true;
		break;
	case TPD_TYPE_GS:
		/* if no gold plus cores, mid = max*/
		if (dest_cpu >= max_idx)
			mismatch = true;
		break;
	case TPD_TYPE_P:
		if (dest_cpu < max_idx)
			mismatch = true;
		break;
	case TPD_TYPE_PS:
		if (dest_cpu < max_idx && dest_cpu >= mid_idx)
			mismatch = true;
		break;
	case TPD_TYPE_PG:
		if (dest_cpu < mid_idx)
			mismatch = true;
		break;
	default:
		break;
	}

out:
	tpd_logi("task:%d comm:%s dst: %d should migrate = %d, task is fg? %d\n", tsk->pid, tsk->comm, dest_cpu, !mismatch, task_is_fg(tsk));

	return mismatch;
}

static void tpd_mgp_init()
{
	int i, j;

	for (i = 0; i < TPD_GROUP_MAX; ++i) {
		mgp[i].decision = 0;
		spin_lock_init(&mgp[i].tgid_list_lock);
		INIT_LIST_HEAD(&mgp[i].tgid_head);
		spin_lock_init(&mgp[i].miss_list_lock);
		for (j = 0; j < MAX_MISS_LIST; j++) {
			mgp[i].miss_list_tgid[j] = 0;
			strcpy(mgp[i].miss_list[j], "");
		}
		mgp[i].not_yet = 0;
		mgp[i].cur_idx = 0;
	}
}

static int tpd_init(void)
{
        tpd_mgp_init();
        tpd_logv("tpd init\n");
        return 0;
}

pure_initcall(tpd_init);
