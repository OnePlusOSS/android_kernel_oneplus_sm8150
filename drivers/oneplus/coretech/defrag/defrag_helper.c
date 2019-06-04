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

#include <linux/types.h>
#include <asm/pgtable.h>
#include <linux/mmzone.h>
#include <linux/page-isolation.h>
#include <linux/atomic.h>
#include <linux/moduleparam.h>
#include <linux/module.h>
#include "defrag_helper.h"

atomic64_t fp_order_usage[MAX_ORDER] = {ATOMIC64_INIT(0)};
atomic64_t fp_order_fail[MAX_ORDER] = {ATOMIC64_INIT(0)};
struct page *(*ctech_defrag_alloc_t)(struct zone *zone, unsigned long flags,
			int migratetype, int order) = NULL;
long (*ctech_defrag_calc_t)(struct zone *zone, int order,
			int alloc_flag) = NULL;
bool (*ctech_check_alloc_flag_t)(int alloc_flag, int order) = NULL;
void __mod_zone_page_state(struct zone *, enum zone_stat_item item, long);


/* calling functions */
bool ctech_check_alloc_flag(int alloc_flag, int order)
{
	if (ctech_check_alloc_flag_t)
		return ctech_check_alloc_flag_t(alloc_flag, order);
	return false;
}
struct page *ctech_defrag_alloc(struct zone *zone, unsigned long flags,
			int migratetype, int order)
{
	if (ctech_defrag_alloc_t)
		return ctech_defrag_alloc_t(zone, flags, migratetype, order);
	return 0;
}

long ctech_defrag_calc(struct zone *zone, int order, int alloc_flag)
{
	if (likely(ctech_defrag_calc_t))
		return ctech_defrag_calc_t(zone, order, alloc_flag);
	else
		return ctech_zone_page_state(zone, NR_FREE_DEFRAG_POOL);
}

/* register functions */
void ctech_check_alloc_flag_reg(bool (*fn)(int alloc_flag, int order))
{
	ctech_check_alloc_flag_t = fn;
}
EXPORT_SYMBOL(ctech_check_alloc_flag_reg);

void ctech_defrag_alloc_reg(struct page *(*fn)(struct zone *zone,
			unsigned long flags, int migratetype, int order))
{
	ctech_defrag_alloc_t = fn;
}
EXPORT_SYMBOL(ctech_defrag_alloc_reg);

void ctech_defrag_calc_reg(long (*fn)(struct zone *zone,
			int order, int alloc_flag))
{
	ctech_defrag_calc_t = fn;
}
EXPORT_SYMBOL(ctech_defrag_calc_reg);

struct zone *ctech_first_zone(void)
{
	return NODE_DATA(first_online_node)->node_zones;
}
EXPORT_SYMBOL(ctech_first_zone);

struct zone *ctech_next_zone(struct zone *zone)
{
	return next_zone(zone);
}
EXPORT_SYMBOL(ctech_next_zone);

void ctech_set_pageblock_migratetype(struct page *page, int migratetype)
{
	set_pageblock_migratetype(page, migratetype);
}
EXPORT_SYMBOL(ctech_set_pageblock_migratetype);

unsigned long ctech_get_pageblock_migratetype(struct page *page)
{
	return get_pageblock_migratetype(page);
}
EXPORT_SYMBOL(ctech_get_pageblock_migratetype);

int ctech_move_freepages_block(struct zone *zone, struct page *page,
		int migratetype, int old_mt)
{
	return move_freepages_block(zone, page, migratetype, NULL, old_mt);
}
EXPORT_SYMBOL(ctech_move_freepages_block);

unsigned long ctech_zone_page_state(struct zone *zone, enum zone_stat_item item)
{
	long x = atomic_long_read(&zone->vm_stat[item]);
#ifdef CONFIG_SMP
	if (x < 0)
		x = 0;
#endif
	return x;
}
EXPORT_SYMBOL(ctech_zone_page_state);

unsigned long ctech_zone_end_pfn(const struct zone *zone)
{
	return zone_end_pfn(zone);
}
EXPORT_SYMBOL(ctech_zone_end_pfn);

int ctech_pfn_valid(unsigned long pfn)
{
	return pfn_valid(pfn);
}
EXPORT_SYMBOL(ctech_pfn_valid);

struct page *ctech_pfn_to_page(unsigned long pfn)
{
	return pfn_to_page(pfn);
}
EXPORT_SYMBOL(ctech_pfn_to_page);

static void release_migratetype(void)
{
	struct zone *zone;
	unsigned long start_pfn, pfn, end_pfn;
	unsigned long block_migratetype;
	unsigned long flags;
	struct page *page;
	int counter, pages_moved;

	for_each_zone(zone) {
		spin_lock_irqsave(&zone->lock, flags);
		start_pfn = zone->zone_start_pfn;
		end_pfn = zone_end_pfn(zone);
		start_pfn = roundup(start_pfn, pageblock_nr_pages);
		counter = 0;

		for (pfn = start_pfn; pfn < end_pfn;
					pfn += pageblock_nr_pages) {
			if (!pfn_valid(pfn))
				continue;
			page = pfn_to_page(pfn);
			block_migratetype = get_pageblock_migratetype(page);
			if (block_migratetype ==
					MIGRATE_UNMOVABLE_DEFRAG_POOL){
				set_pageblock_migratetype(page,
						MIGRATE_MOVABLE);
				pages_moved = move_freepages_block(zone, page,
							MIGRATE_MOVABLE, NULL,
					MIGRATE_UNMOVABLE_DEFRAG_POOL);
				__mod_zone_page_state(zone,
				NR_FREE_DEFRAG_POOL, -pages_moved);
			}
		}
		spin_unlock_irqrestore(&zone->lock, flags);
	}
}

unsigned int __read_mostly alloc_status = 1;

static int alloc_status_show(char *buf, const struct kernel_param *kp)
{
	return snprintf(buf, PAGE_SIZE, "%u", alloc_status);
}

static int alloc_status_store(const char *buf, const struct kernel_param *kp)
{
	unsigned int val;

	if (sscanf(buf, "%u\n", &val) <= 0)
		return -EINVAL;
	if (!strncmp(buf, "1", 1))
		alloc_status = val;
	else
		alloc_status = 0;
	return 0;
}

static const struct kernel_param_ops param_ops_alloc_status = {
	.get = alloc_status_show,
	.set = alloc_status_store,
};
module_param_cb(alloc_status, &param_ops_alloc_status, NULL, 0644);

unsigned int __read_mostly disable;

static int disable_show(char *buf, const struct kernel_param *kp)
{
	return snprintf(buf, PAGE_SIZE, "%u", disable);
}

static int disable_store(const char *buf, const struct kernel_param *kp)
{
	unsigned int val;

	if (sscanf(buf, "%u\n", &val) <= 0)
		return -EINVAL;
	if (!strncmp(buf, "1", 1)) {
		disable = val;
		release_migratetype();
	}
	return 0;
}

static const struct kernel_param_ops param_ops_disable = {
	.get = disable_show,
	.set = disable_store,
};
module_param_cb(disable, &param_ops_disable, NULL, 0644);
