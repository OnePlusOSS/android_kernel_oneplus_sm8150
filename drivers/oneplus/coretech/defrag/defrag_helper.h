/*
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _LINUX_DEFRAG_HELPER_H
#define _LINUX_DEFRAG_HELPER_H
#include <linux/seq_file.h>

struct page *ctech_defrag_alloc(struct zone *zone, unsigned long flags,
			int migratetype, int order);
long ctech_defrag_calc(struct zone *zone, int order, int alloc_flag);

bool ctech_check_alloc_flag(int alloc_flag, int order);

void ctech_check_alloc_flag_reg(bool (*fn)(int alloc_flag, int order));

void ctech_defrag_alloc_reg(struct page *(*fn)(struct zone *zone,
			unsigned long flags, int migratetype, int order));
void ctech_defrag_calc_reg(long (*fn)(struct zone *zone, int order,
			int alloc_flag));
struct pglist_data *ctech_first_online_pgdat(void);
struct zone *ctech_first_zone(void);
struct zone *ctech_next_zone(struct zone *zone);
void ctech_set_pageblock_migratetype(struct page *page,
			int migratetype);
int ctech_move_freepages_block(struct zone *zone, struct page *page,
			int migratetype, int old_mt);
unsigned long ctech_get_pageblock_migratetype(struct page *page);
struct page *ctech___rmqueue(struct zone *zone, unsigned int order,
			int migratetype);
unsigned long ctech_zone_page_state(struct zone *zone,
			enum zone_stat_item item);
unsigned long ctech_zone_end_pfn(const struct zone *zone);
int ctech_pfn_valid(unsigned long pfn);
struct page *ctech_pfn_to_page(unsigned long pfn);


extern atomic64_t fp_order_usage[MAX_ORDER];
extern atomic64_t fp_order_fail[MAX_ORDER];
extern unsigned int alloc_status;

static inline void fp_usage_add(int order)
{
	if (alloc_status)
		atomic64_add(1, &fp_order_usage[order]);
}
static inline void fp_fail_add(int order)
{
	if (alloc_status)
		atomic64_add(1, &fp_order_fail[order]);
}


static inline void print_fp_statistics(struct seq_file *m)
{
	int order;

	if (!alloc_status)
		return;
	seq_printf(m, "fp_usage\t\t");
	for (order = 0; order < MAX_ORDER; ++order)
		seq_printf(m, "%6lu ", atomic64_read(&fp_order_usage[order]));
	seq_putc(m, '\n');
	seq_printf(m, "fp_fail \t\t");
	for (order = 0; order < MAX_ORDER; ++order)
		seq_printf(m, "%6lu ", atomic64_read(&fp_order_fail[order]));
	seq_putc(m, '\n');
}

#endif

