#ifdef VENDOR_EDIT
#include <linux/kernel_stat.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/sched/energy.h>
#include <linux/sched/topology.h>
#include <linux/cpufreq.h>
#include <linux/vmalloc.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/task_cpustats.h>

/* FIXME get max_pid on the runtime.*/
#define MAX_PID (32768)
#define CTP_WINDOW_SZ (5)
unsigned int sysctl_task_cpustats_enable = 0;
DEFINE_PER_CPU(struct kernel_task_cpustat, ktask_cpustat);
struct acct_cpustat {
	pid_t tgid;
	unsigned int pwr;
	char comm[TASK_COMM_LEN];
};

static struct acct_cpustat cpustats[MAX_PID];

static int get_power(int cpu, int freq) {
	int i;
	struct sched_group_energy *sge = sge_array[cpu][SD_LEVEL0];
	if (!sge)
		goto err_found;
	for (i = sge->nr_cap_states - 1; i > -1; i--) {
		struct capacity_state* cs = sge->cap_states + i;
		if (cs->frequency == freq)
			return cs->power;
	}
err_found:
	pr_err("not found %d %d in sge.\n", cpu, freq);
	return 0;
}

static int task_cpustats_show(struct seq_file *m, void *v)
{
	int *idx = (int *) v;
	struct acct_cpustat *s = cpustats + *idx;
	seq_printf(m, "%d\t%d\t%d\t%s\n", *idx, s->tgid,
			s->pwr, s->comm);
	return 0;
}

static void *task_cpustats_next(struct seq_file *m, void *v, loff_t *ppos)
{
	int *idx = (int *)v;
	(*idx)++;
	(*ppos)++;
	for (; *idx < MAX_PID; (*idx)++, (*ppos)++) {
		struct acct_cpustat *s = cpustats + *idx;
		if (s->pwr)
			return idx;
	}
	return NULL;
}

static void *task_cpustats_start(struct seq_file *m, loff_t *ppos)
{
	int i, j;
	unsigned long begin = jiffies - CTP_WINDOW_SZ * HZ, end = jiffies;
	int *idx;
	if (!sysctl_task_cpustats_enable)
		return NULL;
	idx = (int *) kmalloc(sizeof(int), GFP_KERNEL);
	if (!idx)
		return NULL;
	*idx = *ppos;
	if (*idx >= MAX_PID)
		goto start_error;
	memset(cpustats, 0, sizeof(cpustats));
	for_each_possible_cpu(i) {
		struct kernel_task_cpustat* kstat = &per_cpu(ktask_cpustat, i);
		for (j = 0; j < MAX_CTP_WINDOW; j++) {
			struct task_cpustat *ts = kstat->cpustat + j;
			unsigned long r_time = ts->end - ts->begin;
			if (ts->pid >= MAX_PID)
				continue;
			if (ts->begin >= begin && ts->end <= end) {
				struct acct_cpustat *as = cpustats + ts->pid;
				if (as->pwr == 0)
					memcpy(as->comm, ts->comm, TASK_COMM_LEN);
				/* 4 ms each tick */
				as->pwr += get_power(i, ts->freq) * jiffies_to_msecs(r_time);
				as->tgid = ts->tgid;
			}
		}
	}
	for (; *idx < MAX_PID; (*idx)++, (*ppos)++) {
		struct acct_cpustat *as = cpustats + *idx;
		if (as->pwr)
			return idx;
	}
start_error:
	kfree(idx);
	return NULL;
}

static void task_cpustats_stop(struct seq_file *m, void *v)
{
	if (v)
		kfree(v);
}

static const struct seq_operations seq_ops = {
	.start	= task_cpustats_start,
	.next	= task_cpustats_next,
	.stop	= task_cpustats_stop,
	.show	= task_cpustats_show
};

static int sge_show(struct seq_file *m, void *v)
{
	int i, cpu;
	for_each_possible_cpu(cpu) {
		struct sched_group_energy *sge = sge_array[cpu][SD_LEVEL0];
		struct cpufreq_policy *p = cpufreq_cpu_get_raw(cpu);
		int max_freq;
		int min_freq;
		if (!sge || !p)
			continue;
		max_freq = p->cpuinfo.max_freq;
		min_freq = p->cpuinfo.min_freq;
		seq_printf(m, "cpu %d\n", cpu);
		for (i = sge->nr_cap_states - 1; i > -1; i--) {
			struct capacity_state* cs = sge->cap_states + i;
			if (cs->frequency >= min_freq && cs->frequency <= max_freq)
				seq_printf(m, "freq %lu pwr %lu\n", cs->frequency, cs->power);
		}
	}
	return 0;
}

static int sge_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, sge_show, NULL);
}

static int task_cpustats_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &seq_ops);
}

static const struct file_operations sge_proc_fops = {
	.open		= sge_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations task_cpustats_proc_fops = {
	.open		= task_cpustats_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

static int __init proc_task_cpustat_init(void)
{
	proc_create("sgeinfo", 0, NULL, &sge_proc_fops);
	proc_create("task_cpustats", 0, NULL, &task_cpustats_proc_fops);
	return 0;
}
fs_initcall(proc_task_cpustat_init);
#endif
