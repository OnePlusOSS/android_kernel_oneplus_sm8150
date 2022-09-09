/* Copyright (c) 2017-2018, The Linux Foundation. All rights reserved.
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
#include <soc/qcom/secure_buffer.h>
#include "ion.h"

#ifndef _ION_SYSTEM_HEAP_H
#define _ION_SYSTEM_HEAP_H

#ifndef CONFIG_ALLOC_BUFFERS_IN_4K_CHUNKS
#if defined(CONFIG_IOMMU_IO_PGTABLE_ARMV7S)
static const unsigned int orders[] = {8, 4, 0};
#else
static const unsigned int orders[] = {9, 4, 0};
#endif
#else
static const unsigned int orders[] = {0};
#endif

#define NUM_ORDERS ARRAY_SIZE(orders)

struct ion_system_heap {
	struct ion_heap heap;
	struct ion_page_pool *uncached_pools[MAX_ORDER];
	struct ion_page_pool *cached_pools[MAX_ORDER];
	struct ion_page_pool *secure_pools[VMID_LAST][MAX_ORDER];
	/* Prevents unnecessary page splitting */
	struct mutex split_page_mutex;
#ifdef CONFIG_OPLUS_ION_BOOSTPOOL
	struct ion_boost_pool *gr_pool, *cam_pool, *uncached_boost_pool;
#endif /* CONFIG_OPLUS_ION_BOOSTPOOL */
};

struct page_info {
	struct page *page;
	bool from_pool;
	unsigned int order;
	bool from_boost_kmem_cache;
	struct list_head list;
};

int order_to_index(unsigned int order);

void free_buffer_page(struct ion_system_heap *heap,
		      struct ion_buffer *buffer, struct page *page,
		      unsigned int order);

int ion_system_heap_create_pools(struct ion_page_pool **pools,
				 bool cached, bool boost_flag);

void ion_system_heap_destroy_pools(struct ion_page_pool **pools);
#endif /* _ION_SYSTEM_HEAP_H */
