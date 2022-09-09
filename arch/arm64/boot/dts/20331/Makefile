# SPDX-License-Identifier: GPL-2.0

dtbo-$(CONFIG_ARCH_ATOLL) += \
        atoll-idp-overlay.dtbo\
        atoll-atp-overlay.dtbo\
        atoll-qrd-overlay.dtbo\
        atoll-wcd937x-idp-overlay.dtbo\
        atoll-usbc-idp-overlay.dtbo\
        atoll-rumi-overlay.dtbo\
        atollp-idp-overlay.dtbo\
        atollp-atp-overlay.dtbo\
        atollp-qrd-overlay.dtbo\
        atoll-ab-idp-overlay.dtbo\
        atoll-ab-atp-overlay.dtbo\
        atoll-ab-qrd-overlay.dtbo

atoll-idp-overlay.dtbo-base := atoll.dtb
atoll-atp-overlay.dtbo-base := atoll.dtb
atoll-qrd-overlay.dtbo-base := atoll.dtb
atoll-rumi-overlay.dtbo-base := atoll.dtb
atoll-wcd937x-idp-overlay.dtbo-base := atoll.dtb
atoll-usbc-idp-overlay.dtbo-base := atoll.dtb
atollp-idp-overlay.dtbo-base := atollp.dtb
atollp-atp-overlay.dtbo-base := atollp.dtb
atollp-qrd-overlay.dtbo-base := atollp.dtb
atoll-ab-idp-overlay.dtbo-base := atoll-ab.dtb
atoll-ab-atp-overlay.dtbo-base := atoll-ab.dtb
atoll-ab-qrd-overlay.dtbo-base := atoll-ab.dtb

always          := $(dtb-y)
subdir-y        := $(dts-dirs)
clean-files     := *.dtb *.dtbo

