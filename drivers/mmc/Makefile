#
# Makefile for the kernel mmc device drivers.
#

subdir-ccflags-$(CONFIG_MMC_DEBUG) := -DDEBUG

obj-$(CONFIG_MMC)		+= core/
obj-$(CONFIG_MMC)		+= card/
obj-$(subst m,y,$(CONFIG_MMC))	+= host/

# optimise kernel size
subdir-ccflags-y += $(call cc-option, -g0)
subdir-ccflags-y += $(call cc-option, -Os)
