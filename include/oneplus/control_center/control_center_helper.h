/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __CONTROL_CENTER_HELPER_INC__
#define __CONTROL_CENTER_HELPER_INC__

#ifdef CONFIG_CONTROL_CENTER
/* define for boost ts information */
#define CC_BOOST_TS_SIZE (8)
struct cc_boost_ts {
	pid_t pid;
	u32 type;
	u64 ts_us;
	u64 min;
	u64 max;
};

#define CC_CTL_PARAM_SIZE 4
struct cc_command {
	pid_t pid;
	pid_t leader;
	u32 period_us;
	u32 prio;
	u32 group;
	u32 category;
	u32 type;
	u64 params[CC_CTL_PARAM_SIZE];
	u64 response;
	bool bind_leader;
	int status;
};

/* define for task embedded data */
struct cc_tsk_data {
	struct cc_command cc;
	struct list_head node;
	struct delayed_work dwork;
};

extern void cc_tsk_init(void* task);
extern void cc_tsk_free(void* task);

/* ddr related control */
extern atomic_t cc_expect_ddrfreq;
extern bool cc_ddr_boost_enable;
//extern bool cc_ddr_set_enable;
//extern bool cc_ddr_lower_bound_enable;
//extern bool cc_ddr_lock_enable;

//extern u64 cc_cpu_find_ddr(int cpu);
extern bool cc_is_ddrfreq_related(const char* name);

/* ddr lock api */
extern void aop_lock_ddr_freq(int lv);

extern unsigned long cc_get_expect_ddrfreq(void);

#define CC_DDR_LOWER_BOUND 0x0001
#define CC_DDR_VOTING      0x0002
#define CC_DDR_CCDM        0x0004

extern bool cc_ddr_config_check(int config);
#else
static inline void cc_tsk_init(void* task) {};
static inline void cc_tsk_free(void* task) {};

//extern u64 cc_cpu_find_ddr(int cpu) { return 0; }
extern bool cc_is_ddrfreq_related(const char* name) { return false; }
extern bool cc_ddr_config_check(int config) { return false; }
#endif

#ifdef CONFIG_CONTROL_CENTER
/* expected to public */
enum {
	CCDM_DEFAULT = 0,
	CCDM_CLUS_0_CPUFREQ,
	CCDM_CLUS_1_CPUFREQ,
	CCDM_CLUS_2_CPUFREQ,
	CCDM_FPS_BOOST,
	CCDM_VOTING_DDRFREQ,
	CCDM_FPS_BOOST_HINT,

	/* Turbo Rendering */
	CCDM_TB_CLUS_0_FREQ_BOOST,
	CCDM_TB_CLUS_1_FREQ_BOOST,
	CCDM_TB_CLUS_2_FREQ_BOOST,
	CCDM_TB_FREQ_BOOST,
	CCDM_TB_PLACE_BOOST,

	CCDM_TB_CPU_0_IDLE_BLOCK,
	CCDM_TB_CPU_1_IDLE_BLOCK,
	CCDM_TB_CPU_2_IDLE_BLOCK,
	CCDM_TB_CPU_3_IDLE_BLOCK,
	CCDM_TB_CPU_4_IDLE_BLOCK,
	CCDM_TB_CPU_5_IDLE_BLOCK,
	CCDM_TB_CPU_6_IDLE_BLOCK,
	CCDM_TB_CPU_7_IDLE_BLOCK,
	CCDM_TB_IDLE_BLOCK,
};

/* status check */
extern bool ccdm_enabled(void);

/* update hint */
extern void ccdm_update_hint_1(
	int type,
	long long arg1
);
extern void ccdm_update_hint_2(
	int type,
	long long arg1,
	long long arg2
);
extern void ccdm_update_hint_3(
	int type,
	long long arg1,
	long long arg2,
	long long arg3
);
extern void ccdm_update_hint_4(
	int type,
	long long arg1,
	long long arg2,
	long long arg3,
	long long arg4
);

/* get hint */
extern long long ccdm_get_hint(int type);
extern int ccdm_any_hint(void);

/* make decision */
extern long long ccdm_decision_1(
	int type,
	long long arg1
);
extern long long ccdm_decision_2(
	int type,
	long long arg1,
	long long arg2
);
extern long long ccdm_decision_3(
	int type,
	long long arg1,
	long long arg2,
	long long arg3
);
extern long long ccdm_decision_4(
	int type,
	long long arg1,
	long long arg2,
	long long arg3,
	long long arg4
);
extern long long ccdm_decision(
	int type,
	long long arg1,
	long long arg2,
	long long arg3,
	long long arg4
);

/* get current status */
extern void ccdm_get_status(void *ptr);

/* reset current status */
extern void ccdm_reset(void);
extern unsigned int ccdm_get_min_util_threshold(void);

#else
static inline int ccdm_enabled(void) { return 0; }
static inline void ccdm_update_hint_1(
	int type,
	long long arg1)
{}
static inline void ccdm_update_hint_2(
	int type,
	long long arg1,
	long long arg2)
{}
static inline void ccdm_update_hint_3(
	int type,
	long long arg1,
	long long arg2,
	long long arg3)
{}
static inline void ccdm_update_hint_4(
	int type,
	long long arg1,
	long long arg2,
	long long arg3,
	long long arg4)
{}

static inline long long ccdm_get_hint(int type) { return 0; }
static inline int ccdm_any_hint(void) { return 0; }

static inline long long ccdm_decision_1(
	int type,
	long long arg1)
{ return 0; }
static inline long long ccdm_decision_2(
	int type,
	long long arg1,
	long long arg2)
{ return 0; }
static inline long long ccdm_decision_3(
	int type,
	long long arg1,
	long long arg2,
	long long arg3)
{ return 0; }
static inline long long ccdm_decision_4(
	int type,
	long long arg1,
	long long arg2,
	long long arg3,
	long long arg4)
{ return 0; }
static inline long long ccdm_decision(
	int type,
	long long arg1,
	long long arg2,
	long long arg3,
	long long arg4)
{ return 0; }
static inline void ccdm_get_status(void *ptr) { return; }
static inline void ccdm_reset(void) {}
static inline unsigned int ccdm_get_min_util_threshold(void) { return 0; }
#endif

#endif
