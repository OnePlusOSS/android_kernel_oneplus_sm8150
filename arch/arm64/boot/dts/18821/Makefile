ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
    dtbo-$(CONFIG_ARCH_SM8150) += sm8150-mtp-overlay.dtbo


    sm8150-mtp-overlay.dtbo-base := sm8150.dtb sm8150-v2.dtb sm8150p.dtb sm8150p-v2.dtb

else
dtb-$(CONFIG_ARCH_SM8150) += 	sm8150-v2-mtp.dtb
endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb *.dtbo