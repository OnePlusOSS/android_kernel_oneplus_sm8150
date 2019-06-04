#ifndef _MEMORY_PLUS_H
#define _MEMORY_PLUS_H

#ifdef CONFIG_MEMPLUS
#define AID_APP	10000  /* first app user */
extern int vm_memory_plus;
extern int vm_memory_plus_test_worstcase;
void memplus_state_check(int cur_adj, int prev_adj, struct task_struct* task);
enum {
	RECLAIM_STANDBY,
	RECLAIM_QUEUE,
	RECLAIM_DONE,
	SWAPIN_QUEUE,
};

enum {
	TYPE_NORMAL,
	TYPE_FREQUENT,
	TYPE_SYS_IGNORE,
	TYPE_WILL_NEED,
	TYPE_END
};
#define MEMPLUS_TYPE_MASK	0x7
bool memplus_enabled(void);
bool current_is_reclaimd(void);
bool current_is_swapind(void);
extern void memplus_move_swapcache_to_anon_lru(struct page *page);
extern void memplus_move_anon_to_swapcache_lru(struct page *page);
#else
static __always_inline void memplus_move_swapcache_to_anon_lru(struct page *page)
{
     clear_bit(PG_swapcache, &(PF_NO_TAIL(page, 1))->flags);
}
static __always_inline void memplus_move_anon_to_swapcache_lru(struct page *page)
{
     set_bit(PG_swapcache, &(PF_NO_TAIL(page, 1))->flags);
}

#endif
#endif /* _MEMORY_PLUS_H */
