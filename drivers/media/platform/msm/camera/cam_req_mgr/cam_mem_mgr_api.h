/* Copyright (c) 2016-2018, The Linux Foundation. All rights reserved.
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

#ifndef _CAM_MEM_MGR_API_H_
#define _CAM_MEM_MGR_API_H_

#include <media/cam_req_mgr.h>
#include "cam_smmu_api.h"

/**
 * struct cam_mem_mgr_request_desc
 *
 * @size    : Size of memory requested for allocation
 * @align   : Alignment of requested memory
 * @smmu_hdl: SMMU handle to identify context bank where memory will be mapped
 * @flags   : Flags to indicate cached/uncached property
 * @region  : Region where memory should be allocated
 */
struct cam_mem_mgr_request_desc {
	uint64_t size;
	uint64_t align;
	int32_t smmu_hdl;
	uint32_t flags;
};

/**
 * struct cam_mem_mgr_memory_desc
 *
 * @kva        : Kernel virtual address of allocated memory
 * @iova       : IOVA of allocated memory
 * @smmu_hdl   : SMMU handle of allocated memory
 * @mem_handle : Mem handle identifying allocated memory
 * @len        : Length of allocated memory
 * @region     : Region to which allocated memory belongs
 */
struct cam_mem_mgr_memory_desc {
	uintptr_t kva;
	uint32_t iova;
	int32_t smmu_hdl;
	uint32_t mem_handle;
	uint64_t len;
	enum cam_smmu_region_id region;
};

/**
 * @brief: Requests a memory buffer
 *
 * @inp:   Information specifying requested buffer properties
 * @out:   Information about allocated buffer
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_mem_mgr_request_mem(struct cam_mem_mgr_request_desc *inp,
	struct cam_mem_mgr_memory_desc *out);

/**
 * @brief: Releases a memory buffer
 *
 * @inp:   Information specifying buffer to be released
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_mem_mgr_release_mem(struct cam_mem_mgr_memory_desc *inp);

/**
 * @brief: Returns IOVA information about buffer
 *
 * @buf_handle: Handle of the buffer
 * @mmu_handle: SMMU handle where buffer is mapped
 * @iova_ptr  : Pointer to mmu's iova
 * @len_ptr   : Length of the buffer
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_mem_get_io_buf(int32_t buf_handle, int32_t mmu_handle,
	uint64_t *iova_ptr, size_t *len_ptr);

/**
 * @brief: This indicates begin of CPU access.
 *         Also returns CPU address information about DMA buffer
 *
 * @buf_handle: Handle for the buffer
 * @vaddr_ptr : pointer to kernel virtual address
 * @len_ptr   : Length of the buffer
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_mem_get_cpu_buf(int32_t buf_handle, uintptr_t *vaddr_ptr,
	size_t *len);

/**
 * @brief: This indicates end of CPU access
 *
 * @buf_handle: Handle for the buffer
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_mem_put_cpu_buf(int32_t buf_handle);

static inline bool cam_mem_is_secure_buf(int32_t buf_handle)
{
	return CAM_MEM_MGR_IS_SECURE_HDL(buf_handle);
}

/**
 * @brief: Reserves a memory region
 *
 * @inp:  Information specifying requested region properties
 * @region : Region which is to be reserved
 * @out   : Information about reserved region
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_mem_mgr_reserve_memory_region(struct cam_mem_mgr_request_desc *inp,
		enum cam_smmu_region_id region,
		struct cam_mem_mgr_memory_desc *out);

/**
 * @brief: Frees a memory region
 *
 * @inp   : Information about region which is to be freed
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_mem_mgr_free_memory_region(struct cam_mem_mgr_memory_desc *inp);

#endif /* _CAM_MEM_MGR_API_H_ */
