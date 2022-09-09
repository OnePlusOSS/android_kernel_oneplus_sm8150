#
# Makefile for the qti specific video device drivers
# based on V4L2.
#
$(warning BRAND_SHOW_FLAG:$(BRAND_SHOW_FLAG))
obj-y += sde/
obj-$(CONFIG_MSM_NPU) += npu/
obj-$(CONFIG_MSM_NPU_V2) += npu_v2/
obj-$(CONFIG_MSM_VIDC_V4L2) += vidc/
obj-$(CONFIG_MSM_VIDC_3X_V4L2) += vidc_3x/
ifeq ($(TARGET_PRODUCT),msmnile)
obj-$(CONFIG_SPECTRA_CAMERA) += camera_8150_op/
else ifeq ($(TARGET_PRODUCT),sm6150)
obj-$(CONFIG_SPECTRA_CAMERA) += camera_7150/
else ifeq ($(strip $(TARGET_PRODUCT)),$(filter $(TARGET_PRODUCT),atoll))
obj-$(CONFIG_SPECTRA_CAMERA) += camera/
endif
obj-$(CONFIG_MSM_AIS) += ais/
obj-$(CONFIG_TSPP) += broadcast/
obj-$(CONFIG_DVB_MPQ) += dvb/
obj-$(CONFIG_MSMB_CAMERA) += camera_v2/
