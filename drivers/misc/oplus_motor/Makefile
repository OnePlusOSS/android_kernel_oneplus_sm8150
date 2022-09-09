subdir-ccflags-y += -Werror
subdir-ccflags-y += -Wno-unused-variable

obj-$(CONFIG_OPLUS_MOTOR) += camera_motor_ic/
obj-$(CONFIG_OPLUS_MOTOR) += digital_hall_ic/
ifeq ($(strip $(CONFIG_ONEPLUS_MOTOR_OP7)), OP7)
obj-y += oneplus_motor.o
obj-y += oneplus_motor_notifier.o
else
obj-$(CONFIG_OPLUS_MOTOR) += oplus_motor.o
obj-$(CONFIG_OPLUS_MOTOR) += oplus_motor_notifier.o
endif
