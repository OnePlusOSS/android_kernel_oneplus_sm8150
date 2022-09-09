/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __QCOM_SMEM_H__
#define __QCOM_SMEM_H__

#include <linux/types.h>

#define QCOM_SMEM_HOST_ANY -1

int qcom_smem_alloc(unsigned host, unsigned item, size_t size);
void *qcom_smem_get(unsigned host, unsigned item, size_t *size);

int qcom_smem_get_free_space(unsigned host);
phys_addr_t qcom_smem_virt_to_phys(void *addr);

#ifdef OPLUS_FEATURE_AGINGTEST
#define DUMP_REASON_SIZE 256

struct dump_info{
    char dump_reason[DUMP_REASON_SIZE];  //dump reason
};

char *parse_function_builtin_return_address(unsigned long function_address);
void save_dump_reason_to_smem(char *reason, char *function_name);
#endif /*OPLUS_FEATURE_AGINGTEST*/
#endif
