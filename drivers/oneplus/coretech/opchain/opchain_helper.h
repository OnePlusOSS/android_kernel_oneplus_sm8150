#ifndef _LINUX_OPCHAIN_HELPER_H
#define _LINUX_OPCHAIN_HELPER_H
#include "opchain_define.h"

extern unsigned int *opc_boost;
extern unsigned int *opc_boost_tl;
extern unsigned int *opc_boost_min_sample_time;

extern bool is_opc_task(struct task_struct *t, int type);
extern void is_opc_task_cb(bool (*cb)(struct task_struct *t, int type));
extern void opc_task_switch(bool enqueue, int cpu, struct task_struct *p, u64 clock);
extern void opc_task_switch_cb(void (*cb)(bool enqueue, int cpu, struct task_struct *p, u64 clock));
extern int opc_get_claim_on_cpu(int cpu);
extern void opc_get_claim_on_cpu_cb(int (*cb)(int cpu));
extern unsigned int opc_get_claims(void);
extern void opc_get_claims_cb(unsigned int (*cb)(void));
extern int opc_select_path(struct task_struct *cur, struct task_struct *t, int prev_cpu);
extern void opc_select_path_cb(int (*cb)(struct task_struct *cur, struct task_struct *t, int prev_cpu));
extern long opc_cpu_util(long util, int cpu, struct task_struct *t, int op_path);
extern void opc_cpu_util_cb(long (*cb)(long util, int cpu, struct task_struct *t, int op_path));
extern bool opc_fps_check(int lvl);
extern void opc_fps_check_cb(bool (*cb)(int i));
extern void opc_add_to_chain_cb(void (*cb)(struct task_struct *t));
extern void *opc_task_rq(void *t);
extern struct rq *opc_cpu_rq(int cpu);
extern unsigned int opc_task_load(struct task_struct *p);
extern int opc_cpu_active(int cpu);
extern int opc_cpu_isolated(int cpu);
#endif
