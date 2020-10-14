obj-$(CONFIG_POWER_AVS)		+= avs/
obj-$(CONFIG_POWER_RESET)	+= reset/
obj-$(CONFIG_POWER_SUPPLY)	+= supply/
obj-$(CONFIG_ARCH_QCOM)		+= qcom/
obj-$(CONFIG_ARCH_QCOM)		+= qcom/
obj-$(CONFIG_POWER_SUPPLY)	+= supply/

obj-y += smart_charge.o

subdir-ccflags-y += -Os
subdir-ccflags-y += $(call cc-option, -g0)
subdir-ccflags-y += $(call cc-option, -Os)