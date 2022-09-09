# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_ION) +=	ion.o ion-ioctl.o ion_heap.o \
			ion_page_pool.o ion_system_heap.o \
			ion_carveout_heap.o ion_chunk_heap.o \
			ion_system_secure_heap.o ion_cma_heap.o \
			ion_secure_util.o ion_cma_secure_heap.o msm/
#ifdef OPLUS_FEATURE_HEALTHINFO
obj-y	+= oplus_ion/
#endif /* OPLUS_FEATURE_HEALTHINFO */

obj-$(CONFIG_OPLUS_ION_BOOSTPOOL) += oplus_ion_boost_pool.o
