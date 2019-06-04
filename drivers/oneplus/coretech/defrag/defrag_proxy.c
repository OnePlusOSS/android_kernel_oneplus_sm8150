/*
 * Copyright (c) 2015-2016, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "defrag.h"
static bool defrag_check_flag(int alloc_flag, int order)
{
	return !check_alloc_flag(alloc_flag, order);
}
static struct page *defrag_pool_alloc(struct zone *zone, unsigned long flags,
			int migratetype, int order)
{
	struct page *page = NULL;
	static bool prev_empty;

	/* if target zone has no space left for defrag, just skip it */
	if (unlikely(prev_empty && ctech_zone_page_state(zone,
					NR_FREE_DEFRAG_POOL) == 0))
		return NULL;

	if (check_alloc_type(migratetype, order)) {
		spin_lock_irqsave(&zone->lock, flags);
		page = ctech___rmqueue(zone, order,
					MIGRATE_UNMOVABLE_DEFRAG_POOL);

		if (page) {
			spin_unlock(&zone->lock);
			__mod_zone_page_state(zone, NR_FREE_PAGES,
							-(1 << order));
			__mod_zone_page_state(zone, NR_FREE_DEFRAG_POOL,
							-(1 << order));
			local_irq_restore(flags);
		} else
			spin_unlock_irqrestore(&zone->lock, flags);
	}

	if (unlikely(page == NULL))
		prev_empty = true;

	return page;
}

/* return pool size, if this allocation cannot use our pool */
static long calculate_reserved_pool(struct zone *z, int order, int alloc_flag)
{
	if (check_alloc_flag(alloc_flag, order))
		return ctech_zone_page_state(z, NR_FREE_DEFRAG_POOL);
	else
		return 0;
}

static void release_unused_area(int request)
{
	struct zone *zone;
	unsigned long start_pfn, pfn, end_pfn;
	unsigned long block_mt;
	unsigned long flags;
	struct page *page;
	int counter, pages_moved;

	if (request)
		request = request_reserved_block();

	/* for_each_zone(zone) { */
	for (zone = ctech_first_zone(); zone; zone = ctech_next_zone(zone)) {
		if (strstr(zone->name, "Movable") != NULL)
			continue;
		spin_lock_irqsave(&zone->lock, flags);
		start_pfn = ZONE_ZSP_R(zone);
		end_pfn = ctech_zone_end_pfn(zone);
		start_pfn = roundup(start_pfn, pageblock_nr_pages);
		counter = 0;

		for (pfn = start_pfn; pfn < end_pfn;
					pfn += pageblock_nr_pages) {
			if (!ctech_pfn_valid(pfn))
				continue;
			page = ctech_pfn_to_page(pfn);
			block_mt = ctech_get_pageblock_migratetype(page);
			if (block_mt == MIGRATE_UNMOVABLE_DEFRAG_POOL) {
				if (++counter <= request)
					continue;
				else {
					ctech_set_pageblock_migratetype(page,
							MIGRATE_MOVABLE);
					pages_moved = ctech_move_freepages_block(zone,
							page, MIGRATE_MOVABLE,
						MIGRATE_UNMOVABLE_DEFRAG_POOL);
					__mod_zone_page_state(zone,
						NR_FREE_DEFRAG_POOL,
							-pages_moved);
				}
			}
		}
		spin_unlock_irqrestore(&zone->lock, flags);
	}
}


static int __init defrag_pool_init(void)
{
	release_unused_area(1);
	ctech_defrag_alloc_reg(defrag_pool_alloc);
	ctech_defrag_calc_reg(calculate_reserved_pool);
	ctech_check_alloc_flag_reg(defrag_check_flag);
	return 0;
}

static void __exit defrag_pool_exit(void)
{
	ctech_defrag_alloc_reg(NULL);
	ctech_defrag_calc_reg(NULL);
	ctech_check_alloc_flag_reg(NULL);
	release_unused_area(0);
}

module_init(defrag_pool_init);
module_exit(defrag_pool_exit);

MODULE_DESCRIPTION("OnePlus Defragger");
MODULE_LICENSE("GPL");
