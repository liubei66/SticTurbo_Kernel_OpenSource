ifneq ($(CONFIG_DEBUG_FS),y)
gsidbg-$(CONFIG_GSI) += gsi_dbg.o

CFLAGS_gsi.o += -DCONFIG_DEBUG_FS
CFLAGS_gsi_dbg.o += -DCONFIG_DEBUG_FS
CFLAGS_gsidbg.o += -DCONFIG_DEBUG_FS
else
obj-$(CONFIG_GSI) += gsi.o gsi_dbg.o
endif

ifdef CONFIG_X86
ccflags-y += -DIPA_EMULATION_COMPILE=1
obj-$(CONFIG_GSI) += gsi_emulation.o
else
ccflags-y += -DIPA_EMULATION_COMPILE=0
endif
