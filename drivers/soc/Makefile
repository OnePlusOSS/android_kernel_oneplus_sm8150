# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the Linux Kernel SOC specific device drivers.
#

obj-$(CONFIG_ARCH_ACTIONS)	+= actions/
obj-$(CONFIG_ARCH_AT91)		+= atmel/
obj-y				+= bcm/
obj-$(CONFIG_ARCH_DOVE)		+= dove/
obj-$(CONFIG_MACH_DOVE)		+= dove/
obj-y				+= fsl/
obj-$(CONFIG_ARCH_MXC)		+= imx/
obj-$(CONFIG_SOC_XWAY)		+= lantiq/
obj-$(CONFIG_ARCH_MEDIATEK)	+= mediatek/
obj-$(CONFIG_ARCH_MESON)	+= amlogic/
obj-$(CONFIG_ARCH_QCOM)		+= qcom/
obj-y				+= renesas/
obj-$(CONFIG_ARCH_ROCKCHIP)	+= rockchip/
obj-$(CONFIG_SOC_SAMSUNG)	+= samsung/
obj-$(CONFIG_ARCH_SUNXI)	+= sunxi/
obj-$(CONFIG_ARCH_TEGRA)	+= tegra/
obj-$(CONFIG_SOC_TI)		+= ti/
obj-$(CONFIG_ARCH_U8500)	+= ux500/
obj-$(CONFIG_PLAT_VERSATILE)	+= versatile/
obj-$(CONFIG_ARCH_ZX)		+= zte/
obj-y                           += oplus/

#ifdef OPLUS_FEATURE_IOMONITOR
obj-$(CONFIG_IOMONITOR)    +=  oplus/iomonitor/
#endif /*OPLUS_FEATURE_IOMONITOR*/

#ifdef VENDOR_EDIT
obj-y				+= oplus/
#endif
#ifdef OPLUS_FEATURE_POWERINFO_STANDBY
obj-$(CONFIG_OPLUS_WAKELOCK_PROFILER) += oplus/owakelock/
#endif /* OPLUS_FEATURE_POWERINFO_STANDBY */
#ifdef OPLUS_FEATURE_HEALTHINFO
obj-y     +=  oplus/oplus_healthinfo/
#endif /* OPLUS_FEATURE_HEALTHINFO */

#ifdef OPLUS_FEATURE_SENSOR_SMEM
obj-y                     += oplus/sensor/
#endif

#ifdef OPLUS_FEATURE_SENSOR_SMEM
obj-$(CONFIG_HORAE_THERMAL_SHELL) += oplus/thermal/
#endif /* OPLUS_FEATURE_SENSOR_SMEM */

#ifdef OPLUS_FEATURE_RT_INFO
obj-$(CONFIG_OPLUS_FEATURE_RT_INFO) += oplus/rt_info/
#endif

#Add for NandSwap
obj-$(CONFIG_NANDSWAP)		+= oplus/oplus_nandswap/
