# SPDX-License-Identifier: GPL-2.0


dtbo-$(CONFIG_ARCH_ATOLL) += \
	atollp-idp-overlay.dtbo\
	atoll-ab-idp-overlay.dtbo

atoll-idp-overlay.dtbo-base := atoll.dtb
atoll-ab-idp-overlay.dtbo-base := atoll-ab.dtb

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb *.dtbo
