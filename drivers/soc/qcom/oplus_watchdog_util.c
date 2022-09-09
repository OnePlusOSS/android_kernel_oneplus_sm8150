/**********************************************************************************
* Copyright (c)  2008-2016  Guangdong OPLUS Mobile Comm Corp., Ltd
* VENDOR_EDIT
* Description: Provide some utils for watchdog to enhance log and action after wdt
* Version   : 1.0
* Date      : 2016-06-22
* ------------------------------ Revision History: --------------------------------
* <version>       <date>        <author>          	<desc>
***********************************************************************************/

#include <linux/irq.h>
#include <linux/percpu.h>
#include <linux/sched.h>
#include <linux/kernel_stat.h>
#include <linux/slab.h>
#include <linux/sched/debug.h>
#include <linux/sched/signal.h>

#include "oplus_watchdog_util.h"

#define MASK_SIZE	32
#define MAX_IRQ_NO	1200

extern struct task_struct *oplus_get_cpu_task(int cpu);
extern int oplus_get_work_cpu(struct work_struct *work);
extern int cpu_idle_pc_state[NR_CPUS];

unsigned int smp_call_any_cpu;
unsigned long smp_call_many_cpumask;
int recovery_tried;

static const char *recoverable_procs[] = {"SearchDaemon", "libsu.so", "NotificationObs"};

struct oplus_irq_counter {
	unsigned int all_irqs_last;
	unsigned int all_irqs_delta;
	unsigned int *irqs_last;
	unsigned int *irqs_delta;
};

static struct oplus_irq_counter o_irq_counter;
static int irqno_sort[10];

int init_oplus_watchlog(void)
{
	if (!o_irq_counter.irqs_last) {
		o_irq_counter.irqs_last = (unsigned int *)kzalloc(
						sizeof(unsigned int)*MAX_IRQ_NO,
						GFP_KERNEL);
		if (!o_irq_counter.irqs_last) {
			return -ENOMEM;
		}
	}

	if (!o_irq_counter.irqs_delta) {
		o_irq_counter.irqs_delta = (unsigned int *)kzalloc(
						sizeof(unsigned int)*MAX_IRQ_NO,
						GFP_KERNEL);
		if (!o_irq_counter.irqs_delta) {
			kfree(o_irq_counter.irqs_last);
			return -ENOMEM;
		}
	}

	return 0;
}

static void update_irq_counter(void)
{
	int n;
	struct irq_desc *desc;
	unsigned int all_count = 0;
	unsigned int irq_count = 0;

	BUG_ON(nr_irqs > MAX_IRQ_NO);

	if (!o_irq_counter.irqs_delta || !o_irq_counter.irqs_last)
		return;

	for_each_irq_desc(n, desc) {
		irq_count = kstat_irqs(n);
		if (!desc->action && !irq_count)
			continue;

		if (irq_count <= o_irq_counter.irqs_last[n])
			o_irq_counter.irqs_delta[n] = 0;
		else
			o_irq_counter.irqs_delta[n] = irq_count - o_irq_counter.irqs_last[n];

		o_irq_counter.irqs_last[n] = irq_count;
		all_count += irq_count;
	}
	o_irq_counter.all_irqs_delta = all_count - o_irq_counter.all_irqs_last;
	o_irq_counter.all_irqs_last = all_count;
}

static void insert_irqno(int no, int i, int size)
{
	int n;
	for (n = size-1; n > i; n--) {
		irqno_sort[n] = irqno_sort[n-1];
	}
	irqno_sort[i] = no;
}

static void sort_irqs_delta(void)
{
	int irq, i;

	for (i = 0; i < 10; i++) {
		irqno_sort[i] = -1;
	}

	for_each_irq_nr(irq) {
		for (i = 0; i < 10; i++) {
			if (irqno_sort[i] == -1) {
				irqno_sort[i] = irq;
				break;
			}

			if (o_irq_counter.irqs_delta[irq] > o_irq_counter.irqs_delta[irqno_sort[i]]) {
				insert_irqno(irq, i, 10);
				break;
			}
		}
	}
}

static void print_top10_irqs(void)
{
	sort_irqs_delta();
	printk(KERN_INFO "Top10 irqs since last: %d:%u; %d:%u; %d:%u; %d:%u; %d:%u; %d:%u; %d:%u; %d:%u; %d:%u; %d:%u; Total: %u\n",
		irqno_sort[0], o_irq_counter.irqs_delta[irqno_sort[0]], irqno_sort[1], o_irq_counter.irqs_delta[irqno_sort[1]],
		irqno_sort[2], o_irq_counter.irqs_delta[irqno_sort[2]], irqno_sort[3], o_irq_counter.irqs_delta[irqno_sort[3]],
		irqno_sort[4], o_irq_counter.irqs_delta[irqno_sort[4]], irqno_sort[5], o_irq_counter.irqs_delta[irqno_sort[5]],
		irqno_sort[6], o_irq_counter.irqs_delta[irqno_sort[6]], irqno_sort[7], o_irq_counter.irqs_delta[irqno_sort[7]],
		irqno_sort[8], o_irq_counter.irqs_delta[irqno_sort[8]], irqno_sort[9], o_irq_counter.irqs_delta[irqno_sort[9]], o_irq_counter.all_irqs_delta);
}

void dump_cpu_online_mask()
{
	static char alive_mask_buf[MASK_SIZE];
	struct cpumask avail_mask;
	cpumask_andnot(&avail_mask, cpu_online_mask, cpu_isolated_mask);
	scnprintf(alive_mask_buf, MASK_SIZE, "%*pb1", cpumask_pr_args(&avail_mask));
	printk(KERN_INFO "cpu avail mask %s\n", alive_mask_buf);
}

void get_cpu_ping_mask(cpumask_t *pmask)
{
	int cpu;
	struct cpumask avail_mask;
	update_irq_counter();
	cpumask_copy(pmask, cpu_online_mask);
	cpumask_andnot(&avail_mask, cpu_online_mask, cpu_isolated_mask);

	for_each_cpu(cpu, cpu_online_mask) {
		if (cpu_idle_pc_state[cpu] || cpu_isolated(cpu))
			cpumask_clear_cpu(cpu, pmask);
	}
	printk(KERN_INFO "[wdog_util]cpu avail mask: 0x%lx; ping mask: 0x%lx; irqs since last: %u\n",
		*cpumask_bits(&avail_mask), *cpumask_bits(pmask), o_irq_counter.all_irqs_delta);
}

void print_smp_call_cpu()
{
	printk(KERN_INFO "cpu of last smp_call_function_any: %d\n",
		smp_call_any_cpu);
	printk(KERN_INFO "cpumask of last smp_call_function_many: 0x%lx\n",
		smp_call_many_cpumask);
}

void dump_wdog_cpu(struct task_struct *w_task)
{
	int work_cpu = 0;
	int wdog_busy = 0;
	struct pt_regs *regs = get_irq_regs();

	update_irq_counter();
	print_top10_irqs();
	work_cpu = task_cpu(w_task);
	wdog_busy = task_curr(w_task);
	if (wdog_busy)
		printk(KERN_EMERG "Watchdog work is running at CPU(%d)\n", work_cpu);
	else
		printk(KERN_EMERG "Watchdog work is pending at CPU(%d)\n", work_cpu);

	if (regs)
		show_regs(regs);
}

static int match_recoverable_procs(char *comm)
{
	const char *p;
	int count = sizeof(recoverable_procs)/sizeof(char *);
	int i = 0;
	while(i < count) {
		p = recoverable_procs[i];
		if(!strncmp(comm, p, TASK_COMM_LEN))
			return 1;
		i++;
	}
	return 0;
}

int try_to_recover_pending(struct task_struct *w_task)
{
	int work_cpu = 0;
	struct task_struct *p;

	if (recovery_tried)
		return 0;

	if (task_curr(w_task))
		return 0;

	work_cpu = task_cpu(w_task);
	p = oplus_get_cpu_task(work_cpu);
	if (match_recoverable_procs(p->comm)) {
		printk(KERN_EMERG "[wdog_util]Try to kill [%s] to recover WDT\n", p->comm);
		do_send_sig_info(SIGKILL, SEND_SIG_FORCED, p, true);
		wake_up_process(p);
		recovery_tried = 1;
		return 1;
	}
	return 0;
}

void reset_recovery_tried()
{
	recovery_tried = 0;
}
