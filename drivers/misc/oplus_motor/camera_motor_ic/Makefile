ifneq ($(CONFIG_MTK_PLATFORM),)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(CONFIG_MTK_PLATFORM)/include
endif
ifeq ($(strip $(CONFIG_ONEPLUS_MOTOR_OP7)), OP7)
obj-y		+= oneplus_drv8834.o
else
obj-y		+= oplus_drv8834.o
obj-y		+= oplus_stspin220.o
obj-y		+= oplus_lt3572.o
endif
