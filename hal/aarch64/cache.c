/*
 * Phoenix-RTOS
 *
 * Operating system loader
 *
 * ARMv8-A cache management
 *
 * Copyright 2021 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <hal/hal.h>

#include "cache.h"


static u64 getL1DcacheID(void)
{
	sysreg_write(csselr_el1, 0); /* Select L1 Dcache */
	return sysreg_read(ccsidr_el1);
}


void hal_dcacheInvalAll(void)
{
	u64 cacheSizeID = getL1DcacheID();
	u64 lineSizeLog = (cacheSizeID & 0x7) + 4;
	u64 assoc = (cacheSizeID >> 3) & 0x3ff;
	u64 sets = (cacheSizeID >> 13) & 0x7fff;
	u32 assocLog = __builtin_clz((u32)assoc);
	u64 wayCtr, setCtr, setway;

	hal_cpuDataSyncBarrier();
	for (wayCtr = 0; wayCtr <= assoc; wayCtr++) {
		for (setCtr = 0; setCtr <= sets; setCtr++) {
			setway = (wayCtr << assocLog) | (setCtr << lineSizeLog);
			asm volatile("dc cisw, %0" : : "r"(setway) : "memory");
		}
	}

	hal_cpuDataSyncBarrier();
	hal_cpuInstrBarrier();
}


void hal_icacheInval(void)
{
	asm volatile(
			"dsb ish\n"
			"ic iallu\n"
			"dsb ish\n"
			"isb\n");
}


static u64 hal_getCacheLineSize(void)
{
	u64 ctr = sysreg_read(ctr_el0);
	ctr = (ctr >> 16) & 0xf;
	return 4 << ctr;
}


void hal_dcacheClean(addr_t start, addr_t end)
{
	u64 ctr;
	hal_cpuDataSyncBarrier();
	ctr = hal_getCacheLineSize();
	start &= ~(ctr - 1);
	while (start < end) {
		asm volatile("dc cvac, %0" : : "r"(start) : "memory");
		start += ctr;
	}

	hal_cpuDataSyncBarrier();
	hal_cpuInstrBarrier();
}


void hal_dcacheInval(addr_t start, addr_t end)
{
	u64 ctr;
	hal_cpuDataSyncBarrier();
	ctr = hal_getCacheLineSize();
	start &= ~(ctr - 1);
	while (start < end) {
		asm volatile("dc ivac, %0" : : "r"(start) : "memory");
		start += ctr;
	}

	hal_cpuDataSyncBarrier();
	hal_cpuInstrBarrier();
}


void hal_dcacheFlush(addr_t start, addr_t end)
{
	u64 ctr;
	hal_cpuDataSyncBarrier();
	ctr = hal_getCacheLineSize();
	start &= ~(ctr - 1);
	while (start < end) {
		asm volatile("dc civac, %0" : : "r"(start) : "memory");
		start += ctr;
	}

	hal_cpuDataSyncBarrier();
	hal_cpuInstrBarrier();
}


/* Read SCTLR for the current EL (rpi4b plo runs at EL2; zynqmp at EL3).
 * Path A of docs/plans/plo-el2-mmu-fix.md. */
static inline u64 cache_readSctlr(void)
{
	switch ((unsigned)(sysreg_read(currentEL) & 0xcU)) {
		case 0xcU: return sysreg_read(sctlr_el3);
		case 0x8U: return sysreg_read(sctlr_el2);
		default:   return sysreg_read(sctlr_el1);
	}
}


static inline void cache_writeSctlr(u64 val)
{
	switch ((unsigned)(sysreg_read(currentEL) & 0xcU)) {
		case 0xcU: sysreg_write(sctlr_el3, val); break;
		case 0x8U: sysreg_write(sctlr_el2, val); break;
		default:   sysreg_write(sctlr_el1, val); break;
	}
}


static void cacheToggle(unsigned int mode, u64 sctlr_bit)
{
	u64 val;

	hal_cpuDataSyncBarrier();
	val = cache_readSctlr();
	val &= ~sctlr_bit;
	if (mode != 0u) {
		val |= sctlr_bit;
	}
	cache_writeSctlr(val);
	hal_cpuDataSyncBarrier();
	hal_cpuInstrBarrier();
}


void hal_dcacheEnable(unsigned int mode)
{
	return cacheToggle(mode, 1 << 2);
}


void hal_icacheEnable(unsigned int mode)
{
	return cacheToggle(mode, 1 << 12);
}
