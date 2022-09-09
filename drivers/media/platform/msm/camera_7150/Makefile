obj-$(CONFIG_SPECTRA_CAMERA) += cam_req_mgr/
obj-$(CONFIG_SPECTRA_CAMERA) += cam_hyp_intf/
obj-$(CONFIG_SPECTRA_CAMERA) += cam_utils/
obj-$(CONFIG_SPECTRA_CAMERA) += cam_core/
obj-$(CONFIG_SPECTRA_CAMERA) += cam_sync/
obj-$(CONFIG_SPECTRA_CAMERA) += cam_smmu/
obj-$(CONFIG_SPECTRA_CAMERA) += cam_cpas/
obj-$(CONFIG_SPECTRA_CAMERA) += cam_cdm/
obj-$(CONFIG_SPECTRA_CAMERA) += cam_isp/
obj-$(CONFIG_SPECTRA_CAMERA) += cam_sensor_module/
obj-$(CONFIG_SPECTRA_CAMERA) += cam_icp/
obj-$(CONFIG_SPECTRA_CAMERA) += cam_jpeg/
obj-$(CONFIG_SPECTRA_CAMERA) += cam_fd/
obj-$(CONFIG_SPECTRA_CAMERA) += cam_lrme/

#VENDOR_EDIT_START
#bsp not added this flag yet, for early stage, added it for porting
#ifndef VENDOR_EDIT
KBUILD_CFLAGS += -DVENDOR_EDIT
KBUILD_CPPFLAGS += -DVENDOR_EDIT
CFLAGS_KERNEL += -DVENDOR_EDIT
CFLAGS_MODULE += -DVENDOR_EDIT
#endif
#VENDOR_EDIT_END