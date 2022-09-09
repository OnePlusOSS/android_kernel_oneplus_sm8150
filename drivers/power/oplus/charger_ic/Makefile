ifeq ($(CONFIG_OPLUS_CHARGER_MTK),y)

	ifeq ($(CONFIG_MTK_HAFG_20),y)
		obj-y	+= oplus_battery_mtk_fg_20.o
	else ifeq ($(CONFIG_OPLUS_CHARGER_MTK6763),y)
		obj-y	+= oplus_battery_mtk6763.o
	else ifeq ($(CONFIG_OPLUS_CHARGER_MTK6771),y)
		obj-y	+= oplus_bq24190.o
		obj-y	+= oplus_bq25890h.o
		obj-y   += oplus_bq25601d.o
		obj-y   += smb1351-charger.o
	else ifeq ($(CONFIG_OPLUS_CHARGER_MTK6779),y)
		MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
		subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
		subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic/include
		subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/typec/tcpc/inc
		subdir-ccflags-y += -I$(srctree)/drivers/power/supply/mediatek/misc/
		obj-y	+= oplus_battery_mtk6779.o
	else ifeq ($(CONFIG_OPLUS_CHARGER_MTK6779Q),y)
		MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
		subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
		subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic/include
		subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/typec/tcpc/inc
		subdir-ccflags-y += -I$(srctree)/drivers/power/supply/mediatek/misc/
		obj-y	+= oplus_battery_mtk6779Q.o
	else ifeq ($(CONFIG_OPLUS_CHARGER_MTK6885),y)
                MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
                subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
                subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic/include
                subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/typec/tcpc/inc
                subdir-ccflags-y += -I$(srctree)/drivers/power/supply/mediatek/misc/
                obj-y   += oplus_battery_mtk6885.o
	else ifeq ($(CONFIG_OPLUS_CHARGER_MTK6873),y)
                MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
                subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
                subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic/include
                subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/typec/tcpc/inc
                subdir-ccflags-y += -I$(srctree)/drivers/power/supply/mediatek/misc/
                obj-y   += oplus_battery_mtk6873.o
	else
		obj-y	+= oplus_battery_mtk.o
	endif
	//obj-y	+= oplus_bq24196.o

else ifeq ($(CONFIG_OPLUS_MSM8953N_CHARGER),y)
	obj-y	+= oplus_battery_msm8953_N.o
else ifeq ($(CONFIG_OPLUS_MSM8953_CHARGER),y)
	obj-y	+= oplus_battery_msm8953.o
else ifeq ($(CONFIG_OPLUS_MSM8998_CHARGER),y)
	obj-y	+= oplus_battery_msm8998.o
else ifeq ($(CONFIG_OPLUS_MSM8998O_CHARGER),y)
	obj-y	+= oplus_battery_msm8998_O.o
else ifeq ($(CONFIG_OPLUS_SDM845_CHARGER),y)
	obj-y	+= oplus_battery_sdm845.o
	obj-y	+= oplus_bq25882.o oplus_da9313.o
else ifeq ($(CONFIG_OPLUS_SDM670_CHARGER),y)
	obj-y	+= oplus_battery_sdm670.o
	obj-y	+= oplus_bq25882.o oplus_da9313.o
else ifeq ($(CONFIG_OPLUS_SDM670P_CHARGER),y)
	obj-y	+= oplus_battery_sdm670P.o
	obj-y	+= oplus_bq25882.o oplus_da9313.o
else ifeq ($(CONFIG_OPLUS_SDM670Q_CHARGER),y)
	obj-y	+= oplus_battery_sdm670Q.o
	obj-y	+= oplus_bq25882.o oplus_da9313.o
else ifeq ($(CONFIG_OPLUS_SM8150_CHARGER),y)
	obj-y	+= oplus_bq25882.o
	obj-y	+= oplus_da9313.o
	obj-y	+= oplus_battery_msm8150.o
else ifeq ($(CONFIG_OPLUS_SM8150_PRO_CHARGER),y)
	obj-y	+= oplus_mp2650.o
	obj-y	+= oplus_da9313.o
	obj-y	+= oplus_battery_msm8150_pro.o
else ifeq ($(CONFIG_OPLUS_SM6125_CHARGER),y)
	obj-y   += oplus_battery_sm6125P.o
else ifeq ($(CONFIG_OPLUS_SM7150R_CHARGER),y)
	obj-y   += oplus_battery_sm7150_R.o
else ifeq ($(CONFIG_OPLUS_SM7250_CHARGER),y)
	obj-y	+= oplus_bq25882.o
	obj-y	+= oplus_da9313.o
	obj-y   += oplus_battery_msm7250_Q.o
else ifeq ($(CONFIG_OPLUS_SM7250R_CHARGER),y)
	obj-y	+= oplus_da9313.o
	obj-y   += oplus_battery_msm7250_R.o
	obj-y	+= oplus_mp2650.o
else ifeq ($(CONFIG_OPLUS_SM8250_CHARGER),y)
	obj-y	+= oplus_da9313.o
	obj-y	+= oplus_battery_msm8250.o
	obj-y	+= oplus_mp2650.o

else ifeq ($(CONFIG_OPLUS_SM7125R_CHARGER),y)
	obj-y	+= oplus_mp2650.o
	obj-y	+= oplus_da9313.o
	obj-y	+= oplus_battery_msm7125_R.o
	obj-y	+= oplus_usbtemp.o
	obj-y	+= oplus_bq2597x.o
else ifeq ($(CONFIG_OPLUS_SM8150R_CHARGER),y)
	obj-y	+= oplus_da9313.o
	obj-y	+= oplus_mp2650.o
	obj-y	+= oplus_battery_msm8150Q.o
else
	obj-y	+= oplus_battery_msm8976.o
endif 

	obj-y   += oplus_short_ic.o
