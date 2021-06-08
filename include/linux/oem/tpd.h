#ifndef __TPD_H__
#define __TPD_H__

#include <linux/cpumask.h>
#include <linux/sched.h>

#define TPD_CLUSTER_0 (1 << 0)
#define TPD_CLUSTER_1 (1 << 1)
#define TPD_CLUSTER_2 (1 << 2)

#define TPD_TYPE_S TPD_CLUSTER_0					/* Silver only */
#define TPD_TYPE_G TPD_CLUSTER_1					/* gold only */
#define TPD_TYPE_GS (TPD_CLUSTER_1 | TPD_CLUSTER_0)			/* sliver + gold */
#define TPD_TYPE_P TPD_CLUSTER_2					/* gold+ only */
#define TPD_TYPE_PS (TPD_CLUSTER_2 | TPD_CLUSTER_0)			/* gold+ + silver */
#define TPD_TYPE_PG (TPD_CLUSTER_2 | TPD_CLUSTER_1)			/* gold+ + gold */
#define TPD_TYPE_PGS (TPD_CLUSTER_2 | TPD_CLUSTER_1 | TPD_CLUSTER_0)	/* all */

#define MAX_THREAD_INPUT 6
#define MAX_MISS_LIST 20

#define TPD_TAG "TPD_DEBUG: "

#define tpd_logv(fmt...) \
	do { \
		if (tpd_log_lv < 1) \
			pr_info(TPD_TAG fmt); \
	} while (0)

#define tpd_logi(fmt...) \
	do { \
		if (tpd_log_lv < 2) \
			pr_info(TPD_TAG fmt); \
	} while (0)

#define tpd_logw(fmt...) \
	do { \
		if (tpd_log_lv < 3) \
			pr_warn(TPD_TAG fmt); \
	} while (0)

#define tpd_loge(fmt...) pr_err(TPD_TAG fmt)
#define tpd_logd(fmt...) pr_debug(TPD_TAG fmt)

enum dynamic_tpd_type {
        TPD_GROUP_MEDIAPROVIDER = 0,

        TPD_GROUP_MAX
};

#ifdef CONFIG_TPD
extern bool is_tpd_enable(void);
extern int tpd_suggested(struct task_struct* tsk, int min_idx, int mid_idx,
		int max_idx, int request_cpu);
extern void tpd_mask(struct task_struct* tsk, int min_idx, int mid_idx, int max_idx,
		cpumask_t *request, int nrcpu);
extern bool tpd_check(struct task_struct *tsk, int dest_cpu, int min_idx, int mid_idx, int max_idx);
extern bool is_dynamic_tpd_task(struct task_struct *tsk);
static inline bool is_tpd_task(struct task_struct *tsk) { return tsk ? (tsk->tpd != 0) : false; }
extern void tpd_tglist_del(struct task_struct *tsk);
#else
static inline bool is_tpd_enable(void) { return false; }
static inline int tpd_suggested(struct task_struct* tsk, int min_idx, int mid_idx,
			int max_idx, int request_cpu) { return request_cpu; }
static inline void tpd_mask(struct task_struct* tsk, int min_idx, int mid_idx, int max_idx,
			cpumask_t *request, int nrcpu) {}
static inline bool tpd_check(struct task_struct *tsk, int dest_cpu, int min_idx,
			int mid_idx, int max_idx) { return false; }
static inline bool is_dynamic_tpd_task(struct task_struct *tsk) { return false; }
static inline bool is_tpd_task(struct task_struct *tsk) { return false; }
static inline void tpd_tglist_del(struct task_struct *tsk) {};
#endif

#endif
