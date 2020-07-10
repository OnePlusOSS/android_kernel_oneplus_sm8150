/* Copyright (c) 2014-2018, The Linux Foundation. All rights reserved.
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

#ifndef _CAM_SMMU_API_H_
#define _CAM_SMMU_API_H_

#include <linux/dma-direction.h>
#include <linux/module.h>
#include <linux/dma-buf.h>
#include <asm/dma-iommu.h>
#include <linux/dma-direction.h>
#include <linux/of_platform.h>
#include <linux/iommu.h>
#include <linux/random.h>
#include <linux/spinlock_types.h>
#include <linux/mutex.h>
#include <linux/msm_ion.h>

/*Enum for possible CAM SMMU operations */
enum cam_smmu_ops_param {
	CAM_SMMU_ATTACH,
	CAM_SMMU_DETACH,
	CAM_SMMU_VOTE,
	CAM_SMMU_DEVOTE,
	CAM_SMMU_OPS_INVALID
};

enum cam_smmu_map_dir {
	CAM_SMMU_MAP_READ,
	CAM_SMMU_MAP_WRITE,
	CAM_SMMU_MAP_RW,
	CAM_SMMU_MAP_INVALID
};

enum cam_smmu_region_id {
	CAM_SMMU_REGION_FIRMWARE,
	CAM_SMMU_REGION_SHARED,
	CAM_SMMU_REGION_SCRATCH,
	CAM_SMMU_REGION_IO,
	CAM_SMMU_REGION_SECHEAP,
	CAM_SMMU_REGION_QDSS
};

/**
 * @brief        : Callback function type that gets called back on cam
 *                     smmu page fault.
 *
 * @param domain   : Iommu domain received in iommu page fault handler
 * @param dev      : Device received in iommu page fault handler
 * @param iova     : IOVA where page fault occurred
 * @param flags    : Flags received in iommu page fault handler
 * @param token    : Userdata given during callback registration
 * @param buf_info : Closest mapped buffer info
 */
typedef void (*cam_smmu_client_page_fault_handler)(struct iommu_domain *domain,
	struct device *dev, unsigned long iova, int flags, void *token,
	uint32_t buf_info);

/**
 * @brief            : Structure to store region information
 *
 * @param iova_start : Start address of region
 * @param iova_len   : length of region
 */
struct cam_smmu_region_info {
	dma_addr_t iova_start;
	size_t iova_len;
};

/**
 * @brief           : Gets an smmu handle
 *
 * @param identifier: Unique identifier to be used by clients which they
 *                    should get from device tree. CAM SMMU driver will
 *                    not enforce how this string is obtained and will
 *                    only validate this against the list of permitted
 *                    identifiers
 * @param handle_ptr: Based on the indentifier, CAM SMMU drivier will
 *                    fill the handle pointed by handle_ptr
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_smmu_get_handle(char *identifier, int *handle_ptr);

/**
 * @brief       : Performs IOMMU operations
 *
 * @param handle: Handle to identify the CAM SMMU client (VFE, CPP, FD etc.)
 * @param op    : Operation to be performed. Can be either CAM_SMMU_ATTACH
 *                or CAM_SMMU_DETACH
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_smmu_ops(int handle, enum cam_smmu_ops_param op);

/**
 * @brief       : Maps user space IOVA for calling driver
 *
 * @param handle: Handle to identify the CAM SMMU client (VFE, CPP, FD etc.)
 * @param ion_fd: ION handle identifying the memory buffer.
 * @dir         : Mapping direction: which will traslate toDMA_BIDIRECTIONAL,
 *                DMA_TO_DEVICE or DMA_FROM_DEVICE
 * @dma_addr    : Pointer to physical address where mapped address will be
 *                returned if region_id is CAM_SMMU_REGION_IO. If region_id is
 *                CAM_SMMU_REGION_SHARED, dma_addr is used as an input parameter
 *                which specifies the cpu virtual address to map.
 * @len_ptr     : Length of buffer mapped returned by CAM SMMU driver.
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_smmu_map_user_iova(int handle,
	int ion_fd, enum cam_smmu_map_dir dir,
	dma_addr_t *dma_addr, size_t *len_ptr,
	enum cam_smmu_region_id region_id);

/**
 * @brief        : Maps kernel space IOVA for calling driver
 *
 * @param handle : Handle to identify the CAM SMMU client (VFE, CPP, FD etc.)
 * @param buf    : dma_buf allocated for kernel usage in mem_mgr
 * @dir          : Mapping direction: which will traslate toDMA_BIDIRECTIONAL,
 *                 DMA_TO_DEVICE or DMA_FROM_DEVICE
 * @dma_addr     : Pointer to physical address where mapped address will be
 *                 returned if region_id is CAM_SMMU_REGION_IO. If region_id is
 *                 CAM_SMMU_REGION_SHARED, dma_addr is used as an input
 *                 parameter which specifies the cpu virtual address to map.
 * @len_ptr      : Length of buffer mapped returned by CAM SMMU driver.
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_smmu_map_kernel_iova(int handle,
	struct dma_buf *buf, enum cam_smmu_map_dir dir,
	dma_addr_t *dma_addr, size_t *len_ptr,
	enum cam_smmu_region_id region_id);

/**
 * @brief       : Unmaps user space IOVA for calling driver
 *
 * @param handle: Handle to identify the CAMSMMU client (VFE, CPP, FD etc.)
 * @param ion_fd: ION handle identifying the memory buffer.
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_smmu_unmap_user_iova(int handle,
	int ion_fd, enum cam_smmu_region_id region_id);

/**
 * @brief       : Unmaps kernel IOVA for calling driver
 *
 * @param handle: Handle to identify the CAMSMMU client (VFE, CPP, FD etc.)
 * @param buf   : dma_buf allocated for the kernel
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_smmu_unmap_kernel_iova(int handle,
	struct dma_buf *buf, enum cam_smmu_region_id region_id);

/**
 * @brief          : Allocates a scratch buffer
 *
 * This function allocates a scratch virtual buffer of length virt_len in the
 * device virtual address space mapped to phys_len physically contiguous bytes
 * in that device's SMMU.
 *
 * virt_len and phys_len are expected to be aligned to PAGE_SIZE and with each
 * other, otherwise -EINVAL is returned.
 *
 * -EINVAL will be returned if virt_len is less than phys_len.
 *
 * Passing a too large phys_len might also cause failure if that much size is
 * not available for allocation in a physically contiguous way.
 *
 * @param handle   : Handle to identify the CAMSMMU client (VFE, CPP, FD etc.)
 * @param dir      : Direction of mapping which will translate to IOMMU_READ
 *                   IOMMU_WRITE or a bit mask of both.
 * @param paddr_ptr: Device virtual address that the client device will be
 *                   able to read from/write to
 * @param virt_len : Virtual length of the scratch buffer
 * @param phys_len : Physical length of the scratch buffer
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */

int cam_smmu_get_scratch_iova(int handle,
	enum cam_smmu_map_dir dir,
	dma_addr_t *paddr_ptr,
	size_t virt_len,
	size_t phys_len);

/**
 * @brief          : Frees a scratch buffer
 *
 * This function frees a scratch buffer and releases the corresponding SMMU
 * mappings.
 *
 * @param handle   : Handle to identify the CAMSMMU client (IFE, ICP, etc.)
 * @param paddr    : Device virtual address of client's scratch buffer that
 *                   will be freed.
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */

int cam_smmu_put_scratch_iova(int handle,
	dma_addr_t paddr);

/**
 *@brief        : Destroys an smmu handle
 *
 * @param handle: Handle to identify the CAM SMMU client (VFE, CPP, FD etc.)
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_smmu_destroy_handle(int handle);

/**
 * @brief       : Finds index by handle in the smmu client table
 *
 * @param handle: Handle to identify the CAM SMMU client (VFE, CPP, FD etc.)
 * @return Index of SMMU client. Nagative in case of error.
 */
int cam_smmu_find_index_by_handle(int hdl);

/**
 * @brief       : Registers smmu fault handler for client
 *
 * @param handle: Handle to identify the CAM SMMU client (VFE, CPP, FD etc.)
 * @param handler_cb: It is triggered in IOMMU page fault
 * @param token: It is input param when trigger page fault handler
 */
void cam_smmu_set_client_page_fault_handler(int handle,
	cam_smmu_client_page_fault_handler handler_cb, void *token);

/**
 * @brief       : Unregisters smmu fault handler for client
 *
 * @param handle: Handle to identify the CAM SMMU client (VFE, CPP, FD etc.)
 * @param token: It is input param when trigger page fault handler
 */
void cam_smmu_unset_client_page_fault_handler(int handle, void *token);

/**
 * @brief Maps memory from an ION fd into IOVA space
 *
 * @param handle: SMMU handle identifying the context bank to map to
 * @param ion_fd: ION fd of memory to map to
 * @param paddr_ptr: Pointer IOVA address that will be returned
 * @param len_ptr: Length of memory mapped
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_smmu_get_iova(int handle, int ion_fd,
	dma_addr_t *paddr_ptr, size_t *len_ptr);

/**
 * @brief Maps memory from an ION fd into IOVA space
 *
 * @param handle: SMMU handle identifying the secure context bank to map to
 * @param ion_fd: ION fd of memory to map to
 * @param paddr_ptr: Pointer IOVA address that will be returned
 * @param len_ptr: Length of memory mapped
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_smmu_get_stage2_iova(int handle, int ion_fd,
	dma_addr_t *paddr_ptr, size_t *len_ptr);

/**
 * @brief Unmaps memory from context bank
 *
 * @param handle: SMMU handle identifying the context bank
 * @param ion_fd: ION fd of memory to unmap
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_smmu_put_iova(int handle, int ion_fd);

/**
 * @brief Maps secure memory for SMMU handle
 *
 * @param handle: SMMU handle identifying secure context bank
 * @param ion_fd: ION fd to map securely
 * @param dir: DMA Direction for the mapping
 * @param dma_addr: Returned IOVA address after mapping
 * @param len_ptr: Length of memory mapped
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_smmu_map_stage2_iova(int handle,
	int ion_fd, enum cam_smmu_map_dir dir, dma_addr_t *dma_addr,
	size_t *len_ptr);

/**
 * @brief Unmaps secure memopry for SMMU handle
 *
 * @param handle: SMMU handle identifying secure context bank
 * @param ion_fd: ION fd to unmap
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_smmu_unmap_stage2_iova(int handle, int ion_fd);

/**
 * @brief Allocates firmware for context bank
 *
 * @param smmu_hdl: SMMU handle identifying context bank
 * @param iova: IOVA address of allocated firmware
 * @param kvaddr: CPU mapped address of allocated firmware
 * @param len: Length of allocated firmware memory
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_smmu_alloc_firmware(int32_t smmu_hdl,
	dma_addr_t *iova,
	uintptr_t *kvaddr,
	size_t *len);

/**
 * @brief Deallocates firmware memory for context bank
 *
 * @param smmu_hdl: SMMU handle identifying the context bank
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_smmu_dealloc_firmware(int32_t smmu_hdl);

/**
 * @brief Gets region information specified by smmu handle and region id
 *
 * @param smmu_hdl: SMMU handle identifying the context bank
 * @param region_id: Region id for which information is desired
 * @param region_info: Struct populated with region information
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_smmu_get_region_info(int32_t smmu_hdl,
	enum cam_smmu_region_id region_id,
	struct cam_smmu_region_info *region_info);

/**
 * @brief Reserves secondary heap
 *
 * @param smmu_hdl: SMMU handle identifying the context bank
 * @param iova: IOVA of secondary heap after reservation has completed
 * @param buf: Allocated dma_buf for secondary heap
 * @param request_len: Length of secondary heap after reservation has completed
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_smmu_reserve_sec_heap(int32_t smmu_hdl,
	struct dma_buf *buf,
	dma_addr_t *iova,
	size_t *request_len);

/**
 * @brief Releases secondary heap
 *
 * @param smmu_hdl: SMMU handle identifying the context bank
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_smmu_release_sec_heap(int32_t smmu_hdl);

/**
 * @brief Allocates qdss for context bank
 *
 * @param smmu_hdl: SMMU handle identifying context bank
 * @param iova: IOVA address of allocated qdss
 * @param len: Length of allocated qdss memory
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_smmu_alloc_qdss(int32_t smmu_hdl,
	dma_addr_t *iova,
	size_t *len);

/**
 * @brief Deallocates qdss memory for context bank
 *
 * @param smmu_hdl: SMMU handle identifying the context bank
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_smmu_dealloc_qdss(int32_t smmu_hdl);

#endif /* _CAM_SMMU_API_H_ */
