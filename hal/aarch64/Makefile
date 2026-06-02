#
# Makefile for Phoenix-RTOS loader (AArch64 HAL)
#
# Copyright 2024 Phoenix Systems
#
# %LICENSE%
#

PLATFORM_MAKEFILE := hal/aarch64/$(TARGET_SUBFAMILY)/Makefile

ifneq ($(wildcard $(PLATFORM_MAKEFILE)),)
  include $(PLATFORM_MAKEFILE)
  CFLAGS += -I$(dir $(PLATFORM_MAKEFILE))
endif

OBJS += $(addprefix $(PREFIX_O)hal/$(TARGET_SUFF)/, exceptions.o cache.o cpu.o string.o mmu.o)
