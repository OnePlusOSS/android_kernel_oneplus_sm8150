/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#ifndef __OPLUS_MULTI_KSWAPD__
#define __OPLUS_MULTI_KSWAPD__
#include <linux/sched.h>

#define MAX_KSWAPD_THREADS 16

extern int kswapd_threads;
extern int max_kswapd_threads;

extern int kswapd_threads_sysctl_handler(struct ctl_table *, int,
					void __user *, size_t *, loff_t *);
extern void update_kswapd_threads(void);
extern int kswapd_cpu_online_ext(unsigned int cpu);
extern int cpu_callback_ext(struct notifier_block *nfb, unsigned long action,
			void *hcpu);
extern int kswapd_run_ext(int nid);
extern void kswapd_stop_ext(int nid);
extern int kswapd(void *p);

#ifdef CONFIG_KSWAPD_UNBIND_MAX_CPU
extern int kswapd_unbind_cpu;
extern void upate_kswapd_unbind_cpu(void);

static inline int kswapd_affinity_check(struct task_struct *p, const struct cpumask *mask)
{
	if (kswapd_unbind_cpu == -1)
		return 0;
	return (p->flags & PF_KSWAPD) && cpumask_test_cpu(kswapd_unbind_cpu, mask);
}
#endif

#endif /*__OPLUS_MULTI_KSWAPD__*/

