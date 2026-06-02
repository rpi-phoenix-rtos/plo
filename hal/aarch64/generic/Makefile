#
# Makefile for Phoenix-RTOS loader (AArch64 HAL generic)
#
# Copyright 2026 Phoenix Systems
#
# %LICENSE%
#

CFLAGS += -DVADDR_KERNEL_INIT=$(VADDR_KERNEL_INIT)

PLO_COMMANDS ?= alias app blob call console copy devices dump echo go help kernel \
  map mem phfs reboot script stop wait

PLO_ALLDEVICES := ram-storage

OBJS += $(addprefix $(PREFIX_O)hal/$(TARGET_SUFF)/$(TARGET_SUBFAMILY)/, _init.o hal.o console.o interrupts.o timer.o video.o)
