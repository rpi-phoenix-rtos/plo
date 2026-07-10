/*
 * Phoenix-RTOS
 *
 * Operating system loader
 *
 * Hardware Abstraction Layer
 *
 * Copyright 2026 Phoenix Systems
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <hal/hal.h>

#include "../cpu.h"
#include "../mmu.h"
#include "../cache.h"


struct {
	/* These fields are used in assembly code in _init.S, don't reorder them */
	hal_syspage_t *hs;
	addr_t entry;
} hal_common;

volatile u64 hal_coreJumpFlag;
volatile addr_t hal_firmwareDtb;


/* Linker symbols */
extern char __init_start[], __init_end[];
extern char __text_start[], __etext[];
extern char __rodata_start[], __rodata_end[];
extern char __cmd_start[], __cmd_end[];
extern char __init_array_start[], __init_array_end[];
extern char __fini_array_start[], __fini_array_end[];
extern char __ramtext_start[], __ramtext_end[];
extern char __data_start[], __data_end[];
extern char __bss_start[], __bss_end[];
extern char __heap_base[], __heap_limit[];
extern char __stack_top[], __stack_limit[];


extern void console_init(void);
extern void interrupts_init(void);
extern void timer_init(void);
extern void timer_done(void);
extern void video_init(void);
extern void video_publishGraphmode(void);
extern void video_markHalReady(void);
extern void video_markKernelJump(void);
extern void hal_exitToEL1(void) __attribute__((noreturn));


static void hal_printCurrentEl(void)
{
	switch (sysreg_read(currentEL)) {
		case 0xc:
			hal_consolePrint("hal: entry EL3\n");
			break;

		case 0x8:
			hal_consolePrint("hal: entry EL2\n");
			break;

		case 0x4:
			hal_consolePrint("hal: entry EL1\n");
			break;

		default:
			hal_consolePrint("hal: entry EL?\n");
			break;
	}
}


static u32 hal_readBe32(addr_t addr)
{
	volatile const u8 *ptr = (const void *)addr;

	return ((u32)ptr[0] << 24) | ((u32)ptr[1] << 16) | ((u32)ptr[2] << 8) | (u32)ptr[3];
}


/* Map all ARM-accessible DRAM as Normal WB Cacheable and enable the MMU
 * (SCTLR.M only — caches disabled; full M|C|I is enabled by the kernel).
 * console_init must run before this function because PL011 MMIO writes
 * work without the MMU but the remapping step cannot produce output.
 */
static void hal_memoryInit(void)
{
	size_t sz;
	addr_t addr;

	mmu_init();

	/* Pi 4 4GB unlock: remap all ARM-accessible DRAM as Normal WB
	 * Cacheable, EXCEPT the 76 MB GPU reserve (which VC4 owns and which
	 * must stay Device so ARM-side cache writes don't alias VC4's
	 * incoherent view of the framebuffer/GPU heap). Everything from
	 * ADDR_DDR through SIZE_DDR is identity-remapped; the GPU hole stays
	 * at the default DEVICE mapping installed by mmu_init.
	 *   0x00000000 - 0x3b3fffff   CACHED  (chunk 1, 948 MB)
	 *   0x3b400000 - 0x3fffffff   DEVICE  (GPU reserve, untouched)
	 *   0x40000000 - 0xfbffffff   CACHED  (chunk 2, 3008 MB)
	 *   0xfc000000 - 0xffffffff   DEVICE  (BCM2711 peripherals)
	 */
	for (sz = 0; sz < (size_t)SIZE_DDR; sz += SIZE_MMU_SECTION_REGION) {
		addr = (addr_t)ADDR_DDR + sz;
		if ((addr >= (addr_t)ADDR_GPU_RSV) &&
				(addr < ((addr_t)ADDR_GPU_RSV + (addr_t)SIZE_GPU_RSV))) {
			continue; /* leave Device — VC4 owns this range */
		}
		mmu_mapAddr(addr, addr, MMU_FLAG_CACHED);
	}

	/* plo runs SCTLR.M=1 only (caches off). The single-shot M|C|I
	 * variant hangs at the MSR on A72 r0p3 + BCM2711 silicon. The
	 * kernel's _init.S enables full M|C|I once it takes over. */
	{
		u64 val;
		asm volatile (
			"ic   ialluis\n"
			"dsb  ish\n"
			"tlbi vmalle1is\n"
			"dsb  ish\n"
			"isb\n"
			::: "memory");
		asm volatile ("mrs %0, sctlr_el1" : "=r"(val));
		val |= (1uL << 0);  /* SCTLR.M only (cache-off plo) */
		asm volatile (
			"msr sctlr_el1, %0\n"
			"isb\n"
			:: "r"(val) : "memory");
	}
}


/* secondary_smoke_entry (defined in _init.S): prints "cN: alive" and
 * parks in WFE. Not called in the current boot path; secondaries go
 * directly from armstub's WFE to kernel entry via hal_cpuJump release-2.
 * PLO_SMP_ENABLE (not set in generic/config.h) gates the full SMP path.
 */
extern void secondary_smoke_entry(void);


void hal_init(void)
{
	interrupts_init();
	console_init();           /* must precede hal_memoryInit (caches-off PL011 writes work without MMU) */
	hal_consolePrint("hal: console_init done\n");
	hal_memoryInit();
	hal_consolePrint("hal: hal_memoryInit done\n");
	timer_init();
	hal_consolePrint("hal: timer_init done\n");
	video_init();
	hal_consolePrint("hal: video_init done\n");
	hal_printCurrentEl();
	video_markHalReady();
	/* Secondaries stay parked in armstub's spin-table WFE; hal_cpuJump
	 * releases them directly to kernel entry (release-2 path). */
	hal_consolePrint("hal: init complete\n");

	hal_common.entry = (addr_t)-1;
}


void hal_done(void)
{
	timer_done();
}


void hal_graphicsInit(void)
{
	video_publishGraphmode();
}


void hal_syspageSet(hal_syspage_t *hs)
{
	addr_t dtbAddr;
	u32 armstubDtb32;

	hal_common.hs = hs;
	hs->resetReason = 0;
	hs->firmwareDtb = 0;
	hs->firmwareDtbSize = 0;

	/* TD-06 (Goal 3: 4 GB RAM) fix 2026-05-17.
	 *
	 * The Pi 4 firmware patches the armstub's `dtb_ptr32` field
	 * (PA 0xf8 — physical address of the firmware-moved runtime
	 * DTB) AND `kernel_entry32` (PA 0xfc — kernel start). The
	 * runtime DTB has both the low memory bank (`memory@0`) and
	 * the high memory bank (`memory@40000000`) on 4 GB Pi 4. The
	 * static `system.dtb` on disk has only the low bank, so we
	 * MUST use the firmware-patched DTB to see all RAM.
	 *
	 * Empirically (2026-05-17 diagnostics): even when the
	 * firmware patches `dtb_ptr32` correctly with the moved-DTB
	 * physical address (e.g. `0x2eff1e00`), the `x0` register
	 * value the armstub passes to plo arrives as 0. The chain
	 * (armstub `mov x0, x5` -> kernel8-reloc stub `mov x0, x19`
	 * -> plo `mov x19, x0`) loses the value. Rather than chase
	 * that fragile propagation, read the armstub's `dtb_ptr32`
	 * field directly from PA 0xf8 in DRAM — the armstub lives at
	 * PA 0x0 by Pi 4 firmware convention and the field stays
	 * valid until plo overwrites that page (nothing in plo's
	 * boot path does).
	 *
	 * Inline asm avoids GCC's null-pointer-deref warning on a
	 * direct C-pointer read of PA 0xf8 in hosted-mode compile. */
	asm volatile (
		"mov x9, #0xf8\n"
		"ldr %w0, [x9]\n"
		: "=r"(armstubDtb32)
		:
		: "x9", "memory");

	dtbAddr = hal_firmwareDtb;
	if (dtbAddr == 0u) {
		dtbAddr = (addr_t)armstubDtb32;
	}

	if ((dtbAddr != 0u) && (hal_readBe32(dtbAddr) == 0xd00dfeedu)) {
		hs->firmwareDtb = dtbAddr;
		hs->firmwareDtbSize = hal_readBe32(dtbAddr + 4u);
		hal_consolePrint("plo: firmware DTB accepted\n");
	}
	else {
		hal_consolePrint("plo: firmware DTB rejected\n");
	}
}


const char *hal_cpuInfo(void)
{
	return CPU_INFO;
}


addr_t hal_kernelGetAddress(addr_t addr)
{
	addr_t offs;

	if ((addr_t)VADDR_KERNEL_INIT != (addr_t)ADDR_KERNEL) {
		offs = addr - VADDR_KERNEL_INIT;
		addr = ADDR_KERNEL + offs;
	}

	return addr;
}


void hal_kernelGetEntryPointOffset(addr_t *off, int *indirect)
{
	*off = 0;
	*indirect = 1;
}


void hal_kernelEntryPoint(addr_t addr)
{
	hal_common.entry = addr;
}


int hal_memoryAddMap(addr_t start, addr_t end, u32 attr, u32 mapId)
{
	(void)start;
	(void)end;
	(void)attr;
	(void)mapId;

	return 0;
}


static void hal_getMinOverlappedRange(addr_t start, addr_t end, mapent_t *entry, mapent_t *minEntry)
{
	if ((start < entry->end) && (end > entry->start)) {
		if (start > entry->start) {
			entry->start = start;
		}

		if (end < entry->end) {
			entry->end = end;
		}

		if (entry->start < minEntry->start) {
			minEntry->start = entry->start;
			minEntry->end = entry->end;
			minEntry->type = entry->type;
		}
	}
}


int hal_memoryGetNextEntry(addr_t start, addr_t end, mapent_t *entry)
{
	size_t i;
	mapent_t tempEntry, minEntry;

	static const mapent_t entries[] = {
		{ .start = (addr_t)__init_start, .end = (addr_t)__init_end, .type = hal_entryTemp },
		{ .start = (addr_t)__text_start, .end = (addr_t)__etext, .type = hal_entryTemp },
		{ .start = (addr_t)__rodata_start, .end = (addr_t)__rodata_end, .type = hal_entryTemp },
		{ .start = (addr_t)__cmd_start, .end = (addr_t)__cmd_end, .type = hal_entryTemp },
		{ .start = (addr_t)__init_array_start, .end = (addr_t)__init_array_end, .type = hal_entryTemp },
		{ .start = (addr_t)__fini_array_start, .end = (addr_t)__fini_array_end, .type = hal_entryTemp },
		{ .start = (addr_t)__ramtext_start, .end = (addr_t)__ramtext_end, .type = hal_entryTemp },
		{ .start = (addr_t)__data_start, .end = (addr_t)__data_end, .type = hal_entryTemp },
		{ .start = (addr_t)__bss_start, .end = (addr_t)__bss_end, .type = hal_entryTemp },
		{ .start = (addr_t)__heap_base, .end = (addr_t)__heap_limit, .type = hal_entryTemp },
		{ .start = (addr_t)__stack_limit, .end = (addr_t)__stack_top, .type = hal_entryTemp },
	};

	if (start == end) {
		return -1;
	}

	minEntry.start = (addr_t)-1;
	minEntry.end = 0;
	minEntry.type = 0;

	tempEntry.start = (addr_t)hal_common.hs;
	tempEntry.end = (addr_t)__heap_limit;
	tempEntry.type = hal_entryReserved;
	hal_getMinOverlappedRange(start, end, &tempEntry, &minEntry);

	for (i = 0; i < sizeof(entries) / sizeof(entries[0]); ++i) {
		if (entries[i].start >= entries[i].end) {
			continue;
		}

		tempEntry.start = entries[i].start;
		tempEntry.end = entries[i].end;
		tempEntry.type = entries[i].type;
		hal_getMinOverlappedRange(start, end, &tempEntry, &minEntry);
	}

	if (minEntry.start != (addr_t)-1) {
		entry->start = minEntry.start;
		entry->end = minEntry.end;
		entry->type = minEntry.type;

		return 0;
	}

	return -1;
}


void hal_cpuReboot(void)
{
	for (;;) {
		hal_cpuHalt();
	}
}


int hal_cpuJump(void)
{
	if (hal_common.entry == (addr_t)-1) {
		return -1;
	}

	video_markKernelJump();
	hal_interruptsDisableAll();
	hal_coreJumpFlag = 1;

	/* Phase Z1 reverted (2026-05-17): plo runs M-only (mmu_enable
	 * single-shot M|C|I hangs at MSR on A72 r0p3 + BCM2711). With
	 * caches off, plo's writes go direct to DDR; cache lines that
	 * exist at teardown are stale firmware-era residue. Use
	 * dc ivac (invalidate-only) — clean would write those stale
	 * lines back over the correct DDR data plo just placed. */
#if defined(PLO_SMP_ENABLE) && (PLO_SMP_ENABLE != 0)
	/* SMP Phase D fix: publish the syspage PA at PA 0xD8 (= armstub
	 * spin_cpu0, never read by any secondary's armstub WFE loop —
	 * cpu0 doesn't spin) so the kernel-side secondary trampoline can
	 * pick it up. The kernel's shared _start path assumes
	 * `x9 = syspage PA` (set by hal_exitToEL1 for primary, but
	 * armstub's `br x4` to spin_cpuN delivers secondaries with x9
	 * clobbered). The kernel-side fix is in
	 * phoenix-rtos-kernel/hal/aarch64/_init.S: secondaries reload x9
	 * from PA 0xD8 immediately after el1_entry. */
	asm volatile (
		"mov x10, #0xd8\n"
		"str %0, [x10]\n"
		"dc cvac, x10\n"
		:: "r"(hal_common.hs) : "x10", "memory");

	/* SMP Phase A second-stage release: cores 1-3 are busy-polling
	 * their spin_cpuN slot from secondary_handoff (plo/_init.S).
	 * Writing the kernel entry PA into those slots wakes them on
	 * their next poll iteration; they then branch into kernel `_start`.
	 *
	 * dc cvac after each store pushes plo's cached write down to the
	 * Point of Coherency so the secondaries' caches-off `ldr` reads
	 * actually see it. */
	asm volatile (
		"mov x10, #0xe0\n"
		"str %0, [x10]\n"
		"dc cvac, x10\n"
		"mov x10, #0xe8\n"
		"str %0, [x10]\n"
		"dc cvac, x10\n"
		"mov x10, #0xf0\n"
		"str %0, [x10]\n"
		"dc cvac, x10\n"
		"dsb sy\n"
		"sev\n"
		:: "r"(hal_common.entry) : "x10", "memory");
	hal_consolePrint("hal: smp release-2 → kernel entry\n");
#endif

	/* Cold-boot reliability fix (2026-05-23): the prior
	 * `hal_dcacheInval(0, 0xfc000000)` walked 4 GB of DRAM with
	 * `dc ivac` (~65 M iterations) using the EL2 MMU. ARM ARM says
	 * `dc ivac` on Device memory is CONSTRAINED UNPREDICTABLE; the
	 * 76 MB GPU reserve in [0x3b400000, 0x40000000) is mapped Device,
	 * making the sweep occasionally fault silently on A72 (matches
	 * the observed intermittent "hang after release-2, no kernel
	 * banner" symptom). Switch to set/way invalidation via the
	 * existing `hal_dcacheInvalAll` helper — same effect (full L1
	 * D-cache invalidated) but doesn't go through the MMU and can't
	 * fault on Device-memory mappings. Matches U-Boot's default
	 * `__asm_invalidate_dcache_all()` path. */
	hal_dcacheEnable(0);
	hal_dcacheInvalAll();
	hal_icacheEnable(0);
	hal_icacheInval();
	mmu_disable();

	hal_exitToEL1();

	/* Never reached */
	return 0;
}
