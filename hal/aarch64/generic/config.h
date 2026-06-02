/*
 * Phoenix-RTOS
 *
 * Operating system loader
 *
 * Platform configuration file for generic AArch64
 *
 * Copyright 2026 Phoenix Systems
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <board_config.h>

#ifndef PLO_GICD_BASE_ADDRESS
#define PLO_GICD_BASE_ADDRESS 0x08000000u
#endif

#ifndef PLO_GICC_BASE_ADDRESS
#define PLO_GICC_BASE_ADDRESS 0x08010000u
#endif

#ifndef PLO_UART0_BASE_ADDRESS
#define PLO_UART0_BASE_ADDRESS 0x09000000u
#endif

#define GICD_BASE_ADDRESS ((void *)PLO_GICD_BASE_ADDRESS)
#define GICC_BASE_ADDRESS ((void *)PLO_GICC_BASE_ADDRESS)

#define UART0_BASE_ADDRESS ((void *)PLO_UART0_BASE_ADDRESS)

#ifndef RAM_ADDR
#define RAM_ADDR      0x48000000
#endif

#ifndef RAM_BANK_SIZE
#define RAM_BANK_SIZE 0x08000000
#endif

#ifndef __ASSEMBLY__

#include "types.h"

typedef struct {
	long long int resetReason;
#if defined(HAS_GRAPHICS) && (HAS_GRAPHICS != 0)
	struct {
		unsigned short width;
		unsigned short height;
		unsigned short bpp;
		unsigned short pitch;
		unsigned long framebuffer; /* addr_t */
	} __attribute__((packed)) graphmode;
#endif
	unsigned long firmwareDtb;     /* addr_t */
	unsigned long firmwareDtbSize; /* size_t */
} __attribute__((packed)) hal_syspage_t;

#include <phoenix/syspage.h>

#include "../cpu.h"

#if defined(__TARGET_AARCH64A72)
#define CPU_INFO "Cortex-A72 Generic"
#else
#define CPU_INFO "Cortex-A53 Generic"
#endif

#if defined(__TARGET_AARCH64A72)
#define PATH_KERNEL "phoenix-aarch64a72-generic.elf"
#else
#define PATH_KERNEL "phoenix-aarch64a53-generic.elf"
#endif

#endif

#if defined(__TARGET_AARCH64A72)
#include "ld/aarch64a72-generic.ldt"
#else
#include "ld/aarch64a53-generic.ldt"
#endif

#define ADDR_KERNEL (ADDR_PLO + SIZE_PLO)

#endif
