#ifndef __INCLUDE_PCCORE_HELPER__
#define __INCLUDE_PCCORE_HELPER__

#ifdef CONFIG_PCCORE
extern unsigned int get_op_mode(void);
extern int cross_pd(int cpu, int prefer_idx,
		int target_idx, bool ascending);
extern bool get_op_select_freq_enable(void);
extern unsigned int get_op_limit(void);
extern unsigned int get_op_level(void);
extern unsigned int get_op_fd_mode(void);
extern int find_prefer_pd(int cpu,
		int target_idx, bool ascending, int lv_cnt);
#else
static inline unsigned int get_op_mode(void) { return 0; };
static inline int cross_pd(int cpu, int prefer_idx,
		int target_idx, bool ascending) { return target_idx; };
static inline bool get_op_select_freq_enable(void) { return false; };
static inline unsigned int get_op_limit(void) { return 0; };
static inline unsigned int get_op_level(void) { return 0; };
static inline unsigned int get_op_fd_mode(void) { return 0; };
static inline int find_prefer_pd(int cpu, int target_idx, bool ascending,
			int lv_cnt) { return target_idx; };
#endif
#endif // __INCLUDE_PCCORE_HELPER__
