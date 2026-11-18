/*
 * Phoenix-RTOS
 *
 * Operating system loader
 *
 * Fill free RAM with a seeded pseudo-random pattern (debug only)
 *
 * Copyright 2026 Phoenix Systems
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

/*
 * A provocation tool for reads of uninitialised memory in the kernel and the
 * programs plo starts. What DRAM holds at power-on is undefined and usually
 * benign (often zeros), so a missing initialisation can go unnoticed for many
 * boots and then fail on an unlucky one. Filling every free page with
 * pseudo-random data before anything is loaded makes such a read misbehave on
 * every boot.
 *
 * The pattern is ONE stream: a 64-bit xorshift generator seeded from <seed>,
 * written over the free pages in a fixed order (the order of plo's memory maps,
 * then ascending addresses). Page N of the stream always holds the same values,
 * whatever else is zeroed: a zeroed page still advances the generator. So when
 * a boot fails with the fill, the failure can be bisected by zeroing halves of
 * the same stream (`poison <seed> <first page> <page count>`) until one page is
 * left. A fresh fill with another seed restarts the sequence and may not fail
 * at all, so never compare runs with different seeds.
 *
 * A page is filled only if it is
 *   - inside one of plo's memory maps (the `map` commands; this keeps the fill
 *     out of the boot stub and plo below the first map), and
 *   - not covered by any map entry (plo itself, anything loaded so far), and
 *   - inside a /memory bank of the firmware device tree (DRAM that exists and
 *     is not kept by the firmware, e.g. the GPU memory at the top of the low
 *     1 GiB), and
 *   - outside every exclusion below.
 * Excluded: plo's image, .bss, heap (the syspage) and stack, from the linker
 * symbols; the firmware device tree blob, its memory reservation block and
 * every /reserved-memory child; the whole RAM-storage bank (the boot image plo
 * reads its scripts and programs from); every program already in the syspage;
 * the GPU reserve window plo maps as Device memory.
 *
 * Without a valid firmware device tree the command refuses to run.
 *
 * Run it before `kernel`, `app` and `blob` in the boot script, after the `map`
 * commands. plo runs with the data cache off, so the stores go straight to
 * DRAM: there is no cache maintenance to do before the kernel starts.
 */

#include "cmd.h"

#include <hal/hal.h>
#include <syspage.h>


#define POISON_MAX_BANKS 8
#define POISON_MAX_EXCL  32
#define POISON_MAX_GUARD 8

#define POISON_PAGE_WORDS (SIZE_PAGE / sizeof(u64))

/* Flattened device tree format (Devicetree Specification, chapter 5) */
#define FDT_MAGIC      0xd00dfeedu
#define FDT_BEGIN_NODE 1u
#define FDT_END_NODE   2u
#define FDT_PROP       3u
#define FDT_NOP        4u
#define FDT_END        9u
#define FDT_MAX_DEPTH  16
#define FDT_MAX_SIZE   (2u * 1024u * 1024u)

#define POISON_PAGE_DOWN(a) ((a) & ~((addr_t)SIZE_PAGE - 1u))


typedef struct {
	addr_t start;
	addr_t end;
} poison_range_t;


typedef struct {
	int fill;          /* 0: list and check the ranges, 1: write the stream */
	u64 state;         /* generator state */
	size_t page;       /* stream index of the next free page */
	size_t zeroStart;  /* stream pages [zeroStart, zeroEnd) are zeroed */
	size_t zeroEnd;
	int unsafe;        /* a range covers an address plo still needs */
} poison_walk_t;


enum { node_other = 0, node_memory, node_reservedMemory, node_reservedChild };


static struct {
	poison_range_t banks[POISON_MAX_BANKS];
	size_t nBanks;
	poison_range_t excl[POISON_MAX_EXCL];
	size_t nExcl;
	addr_t guard[POISON_MAX_GUARD];
	size_t nGuard;
	int full;
} poison_common;


/* Linker symbols */
extern char __init_start[], _end[];
extern char __heap_base[], __heap_limit[];
extern char __stack_limit[], __stack_top[];


static void cmd_poisonInfo(void)
{
	lib_printf("fill free RAM with a seeded pattern (debug), usage: poison <seed> [<zero first page> <zero page count>]");
}


static addr_t poison_pageUp(addr_t a)
{
	addr_t up = POISON_PAGE_DOWN(a + SIZE_PAGE - 1u);

	return (up < a) ? POISON_PAGE_DOWN((addr_t)-1) : up;
}


static addr_t poison_end(addr_t start, u64 size)
{
	addr_t end = start + (addr_t)size;

	return (end < start) ? (addr_t)-1 : end;
}


/* Excluded ranges grow outwards to whole pages and are kept sorted by start */
static void poison_exclude(addr_t start, addr_t end)
{
	size_t i;

	if (start >= end) {
		return;
	}

	if (poison_common.nExcl == POISON_MAX_EXCL) {
		poison_common.full = 1;
		return;
	}

	start = POISON_PAGE_DOWN(start);
	end = poison_pageUp(end);

	for (i = poison_common.nExcl; (i > 0) && (poison_common.excl[i - 1].start > start); --i) {
		poison_common.excl[i] = poison_common.excl[i - 1];
	}
	poison_common.excl[i].start = start;
	poison_common.excl[i].end = end;
	poison_common.nExcl++;
}


static void poison_addBank(addr_t start, addr_t end)
{
	size_t i;

	/* Ignoring a bank only leaves memory unfilled, which is safe */
	if ((start >= end) || (poison_common.nBanks == POISON_MAX_BANKS)) {
		return;
	}

	for (i = poison_common.nBanks; (i > 0) && (poison_common.banks[i - 1].start > start); --i) {
		poison_common.banks[i] = poison_common.banks[i - 1];
	}
	poison_common.banks[i].start = start;
	poison_common.banks[i].end = end;
	poison_common.nBanks++;
}


static void poison_addGuard(addr_t addr)
{
	if (poison_common.nGuard < POISON_MAX_GUARD) {
		poison_common.guard[poison_common.nGuard++] = addr;
	}
}


/* Device tree helpers: big-endian, byte-wise reads (the blob need not be aligned for us) */

static u32 poison_be32(const u8 *p)
{
	return ((u32)p[0] << 24) | ((u32)p[1] << 16) | ((u32)p[2] << 8) | (u32)p[3];
}


static u64 poison_be64(const u8 *p)
{
	return ((u64)poison_be32(p) << 32) | (u64)poison_be32(p + 4);
}


static u64 poison_cells(const u8 *p, u32 cells)
{
	return (cells == 2u) ? poison_be64(p) : (u64)poison_be32(p);
}


static int poison_nameIs(const char *name, const char *base)
{
	size_t len = hal_strlen(base);

	return (hal_strncmp(name, base, len) == 0) && ((name[len] == '\0') || (name[len] == '@'));
}


/* Adds the (address, size) pairs of a `reg` value as banks or exclusions */
static int poison_dtbReg(const u8 *val, u32 len, u32 addrCells, u32 sizeCells, int isBank)
{
	u32 entry = 4u * (addrCells + sizeCells);
	u32 off;
	addr_t start;
	u64 size;

	if ((addrCells < 1u) || (addrCells > 2u) || (sizeCells < 1u) || (sizeCells > 2u)) {
		return -EINVAL;
	}

	for (off = 0; (off + entry) <= len; off += entry) {
		start = (addr_t)poison_cells(val + off, addrCells);
		size = poison_cells(val + off + 4u * addrCells, sizeCells);
		if (isBank != 0) {
			poison_addBank(start, poison_end(start, size));
		}
		else {
			poison_exclude(start, poison_end(start, size));
		}
	}

	return EOK;
}


/* Collects the DRAM banks and the firmware's reserved ranges from the device tree at `dtb` */
static int poison_dtbParse(const u8 *dtb)
{
	u32 totalSize, offStruct, offStrings, offRsv, sizeStrings;
	u32 pos, token, len, nameOff, depth = 0, level;
	u32 addrCells[FDT_MAX_DEPTH], sizeCells[FDT_MAX_DEPTH];
	u8 kind[FDT_MAX_DEPTH];
	u64 rsvAddr, rsvSize;
	const char *name;
	const u8 *val;

	if (poison_be32(dtb) != FDT_MAGIC) {
		return -EINVAL;
	}

	totalSize = poison_be32(dtb + 4);
	offStruct = poison_be32(dtb + 8);
	offStrings = poison_be32(dtb + 12);
	offRsv = poison_be32(dtb + 16);
	sizeStrings = poison_be32(dtb + 32);

	if ((totalSize > FDT_MAX_SIZE) || (offStruct >= totalSize) || (offRsv >= totalSize) ||
			(offStrings > totalSize) || (sizeStrings > (totalSize - offStrings))) {
		return -EINVAL;
	}

	poison_exclude((addr_t)dtb, (addr_t)dtb + totalSize);

	/* Memory reservation block: (address, size) pairs up to a zero pair */
	for (pos = offRsv; (pos + 16u) <= totalSize; pos += 16u) {
		rsvAddr = poison_be64(dtb + pos);
		rsvSize = poison_be64(dtb + pos + 8u);
		if ((rsvAddr == 0u) && (rsvSize == 0u)) {
			break;
		}
		poison_exclude((addr_t)rsvAddr, poison_end((addr_t)rsvAddr, rsvSize));
	}

	/* Structure block */
	pos = offStruct;
	for (;;) {
		if ((pos + 4u) > totalSize) {
			return -EINVAL;
		}
		token = poison_be32(dtb + pos);
		pos += 4u;

		if (token == FDT_END) {
			break;
		}

		switch (token) {
			case FDT_BEGIN_NODE:
				if (depth == FDT_MAX_DEPTH) {
					return -EINVAL;
				}
				name = (const char *)dtb + pos;
				for (len = 0; ((pos + len) < totalSize) && (name[len] != '\0'); ++len) {
				}
				if ((pos + len) >= totalSize) {
					return -EINVAL;
				}
				pos += (len + 4u) & ~3u;

				kind[depth] = node_other;
				if (depth == 1u) {
					if (poison_nameIs(name, "memory") != 0) {
						kind[depth] = node_memory;
					}
					else if (poison_nameIs(name, "reserved-memory") != 0) {
						kind[depth] = node_reservedMemory;
					}
				}
				else if ((depth == 2u) && (kind[1] == node_reservedMemory)) {
					kind[depth] = node_reservedChild;
				}

				/* Defaults for this node's children, until its own properties say otherwise */
				addrCells[depth] = 2u;
				sizeCells[depth] = 1u;
				depth++;
				break;

			case FDT_END_NODE:
				if (depth == 0u) {
					return -EINVAL;
				}
				depth--;
				break;

			case FDT_PROP:
				if ((depth == 0u) || ((pos + 8u) > totalSize)) {
					return -EINVAL;
				}
				len = poison_be32(dtb + pos);
				nameOff = poison_be32(dtb + pos + 4u);
				pos += 8u;
				if ((len > (totalSize - pos)) || (nameOff >= sizeStrings)) {
					return -EINVAL;
				}
				val = dtb + pos;
				name = (const char *)dtb + offStrings + nameOff;
				pos += (len + 3u) & ~3u;

				level = depth - 1u; /* the open node */
				if ((len == 4u) && (hal_strcmp(name, "#address-cells") == 0)) {
					addrCells[level] = poison_be32(val);
				}
				else if ((len == 4u) && (hal_strcmp(name, "#size-cells") == 0)) {
					sizeCells[level] = poison_be32(val);
				}
				else if ((level > 0u) && (hal_strcmp(name, "reg") == 0) &&
						((kind[level] == node_memory) || (kind[level] == node_reservedChild))) {
					if (poison_dtbReg(val, len, addrCells[level - 1u], sizeCells[level - 1u], kind[level] == node_memory) < 0) {
						return -EINVAL;
					}
				}
				break;

			case FDT_NOP:
				break;

			default:
				return -EINVAL;
		}
	}

	return EOK;
}


static void poison_collect(const syspage_t *sp)
{
	const syspage_prog_t *prog;
	addr_t start, end;

	/* plo: image, .bss, heap (holds the syspage) and stack, as one hull */
	start = min(min((addr_t)__init_start, (addr_t)__heap_base), (addr_t)__stack_limit);
	end = max(max((addr_t)_end, (addr_t)__heap_limit), (addr_t)__stack_top);
	poison_exclude(start, end);

	prog = sp->progs;
	if (prog != NULL) {
		do {
			poison_exclude(prog->start, prog->end);
			prog = prog->next;
		} while (prog != sp->progs);
	}

#if defined(RAM_ADDR) && defined(RAM_BANK_SIZE)
	/* The RAM-storage device: the boot image the scripts and programs are read from */
	poison_exclude((addr_t)RAM_ADDR, (addr_t)RAM_ADDR + (addr_t)RAM_BANK_SIZE);
	poison_addGuard((addr_t)RAM_ADDR);
#endif

#if defined(ADDR_GPU_RSV) && defined(SIZE_GPU_RSV)
	poison_exclude((addr_t)ADDR_GPU_RSV, (addr_t)ADDR_GPU_RSV + (addr_t)SIZE_GPU_RSV);
#endif

	poison_addGuard((addr_t)cmd_poisonInfo);
	poison_addGuard((addr_t)__stack_top - 1u);
	poison_addGuard((addr_t)&poison_common);
	poison_addGuard((addr_t)sp);
	poison_addGuard((addr_t)sp->hs.firmwareDtb);
#if defined(HAS_GRAPHICS) && (HAS_GRAPHICS != 0)
	/* Tripwire for a /memory bank that wrongly includes the GPU memory */
	poison_addGuard((addr_t)sp->hs.graphmode.framebuffer);
#endif
}


static inline u64 poison_next(u64 x)
{
	x ^= x << 13;
	x ^= x >> 7;
	x ^= x << 17;

	return x;
}


/* Writes one page of the stream, masked by `keep` (0 zeroes it), and returns the advanced state */
static u64 poison_page(u64 *dst, u64 x, u64 keep)
{
	size_t i;
	u64 a, b;

	for (i = 0; i < POISON_PAGE_WORDS; i += 2) {
		a = poison_next(x);
		b = poison_next(a);
		dst[i] = a & keep;
		dst[i + 1] = b & keep;
		x = b;
	}

	return x;
}


static void poison_emit(poison_walk_t *w, addr_t start, addr_t end)
{
	size_t i, pages;
	addr_t a;

	start = poison_pageUp(start);
	end = POISON_PAGE_DOWN(end);
	if (start >= end) {
		return;
	}
	pages = (end - start) / SIZE_PAGE;

	if (w->fill == 0) {
		for (i = 0; i < poison_common.nGuard; ++i) {
			if ((poison_common.guard[i] >= start) && (poison_common.guard[i] < end)) {
				w->unsafe = 1;
			}
		}
		lib_printf("\npoison: 0x%010llx-0x%010llx %6llu MiB, stream pages %zu-%zu",
			(unsigned long long)start, (unsigned long long)end,
			(unsigned long long)((end - start) >> 20), w->page, w->page + pages - 1u);
		w->page += pages;
		return;
	}

	for (a = start; a < end; a += SIZE_PAGE, w->page++) {
		w->state = poison_page((u64 *)a, w->state,
			((w->page >= w->zeroStart) && (w->page < w->zeroEnd)) ? 0u : ~(u64)0);
	}
}


/* Free part of [start, end) after the exclusions (sorted, may overlap) */
static void poison_subtract(poison_walk_t *w, addr_t start, addr_t end)
{
	size_t i;

	for (i = 0; (i < poison_common.nExcl) && (start < end); ++i) {
		if (poison_common.excl[i].end <= start) {
			continue;
		}
		if (poison_common.excl[i].start >= end) {
			break;
		}
		if (poison_common.excl[i].start > start) {
			poison_emit(w, start, poison_common.excl[i].start);
		}
		start = poison_common.excl[i].end;
	}

	if (start < end) {
		poison_emit(w, start, end);
	}
}


/* Part of [start, end) that is DRAM according to the device tree */
static void poison_clip(poison_walk_t *w, addr_t start, addr_t end)
{
	size_t i;
	addr_t s, e, done = 0;

	for (i = 0; i < poison_common.nBanks; ++i) {
		s = max(max(start, poison_common.banks[i].start), done);
		e = min(end, poison_common.banks[i].end);
		if (s < e) {
			poison_subtract(w, s, e);
			done = e;
		}
	}
}


/* Visits the free pages of every map, in map order, then address order */
static void poison_walk(const syspage_t *sp, poison_walk_t *w)
{
	const syspage_map_t *map = sp->maps;
	const mapent_t *e;
	addr_t cur;

	do {
		cur = map->start;
		e = map->entries;
		if (e != NULL) {
			do {
				if (e->start > cur) {
					poison_clip(w, cur, min(e->start, map->end));
				}
				cur = max(cur, e->end);
				e = e->next;
			} while (e != map->entries);
		}
		if (cur < map->end) {
			poison_clip(w, cur, map->end);
		}
		map = map->next;
	} while (map != sp->maps);
}


static int poison_parseNumber(const char *arg, unsigned long *val)
{
	char *end;

	*val = lib_strtoul((char *)arg, &end, 0);

	return ((end == arg) || (*end != '\0')) ? -EINVAL : EOK;
}


static int cmd_poison(int argc, char *argv[])
{
	const syspage_t *sp = syspage_get();
	poison_walk_t w;
	unsigned long seed, zeroStart = 0, zeroCount = 0;
	size_t total;
	time_t t0, ms;
	u64 x;

	if (((argc != 2) && (argc != 4)) || (poison_parseNumber(argv[1], &seed) < 0) ||
			((argc == 4) && ((poison_parseNumber(argv[2], &zeroStart) < 0) || (poison_parseNumber(argv[3], &zeroCount) < 0)))) {
		log_error("\n%s: Wrong arguments", argv[0]);
		return CMD_EXIT_FAILURE;
	}

	if ((zeroStart + zeroCount) < zeroStart) {
		log_error("\n%s: Zeroed range overflows", argv[0]);
		return CMD_EXIT_FAILURE;
	}

	if (sp->maps == NULL) {
		log_error("\n%s: No memory maps defined, run it after the map commands", argv[0]);
		return CMD_EXIT_FAILURE;
	}

	hal_memset(&poison_common, 0, sizeof(poison_common));

	if ((sp->hs.firmwareDtb == 0u) || (poison_dtbParse((const u8 *)sp->hs.firmwareDtb) < 0)) {
		log_error("\n%s: No valid firmware device tree, refusing to guess which RAM is free", argv[0]);
		return CMD_EXIT_FAILURE;
	}

	if (poison_common.nBanks == 0u) {
		log_error("\n%s: The device tree has no /memory banks", argv[0]);
		return CMD_EXIT_FAILURE;
	}

	poison_collect(sp);
	poison_addGuard((addr_t)&w);

	if (poison_common.full != 0) {
		log_error("\n%s: Too many reserved ranges", argv[0]);
		return CMD_EXIT_FAILURE;
	}

	/* Pass 1: list the ranges and check them before touching anything */
	hal_memset(&w, 0, sizeof(w));
	lib_printf("\npoison: seed 0x%llx", (unsigned long long)seed);
	poison_walk(sp, &w);
	total = w.page;

	if (w.unsafe != 0) {
		log_error("\n%s: A range covers memory plo still needs, nothing written", argv[0]);
		return CMD_EXIT_FAILURE;
	}

	if ((zeroStart + zeroCount) > total) {
		log_error("\n%s: Zeroed stream pages [%llu, %llu) outside the stream of %zu pages", argv[0],
			(unsigned long long)zeroStart, (unsigned long long)(zeroStart + zeroCount), total);
		return CMD_EXIT_FAILURE;
	}

	/* Pass 2: write the stream. splitmix64 spreads small seeds and never yields a state of 0 for xorshift */
	x = (u64)seed + 0x9e3779b97f4a7c15uLL;
	x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9uLL;
	x = (x ^ (x >> 27)) * 0x94d049bb133111ebuLL;
	x ^= x >> 31;

	hal_memset(&w, 0, sizeof(w));
	w.fill = 1;
	w.state = (x != 0u) ? x : 0x9e3779b97f4a7c15uLL;
	w.zeroStart = zeroStart;
	w.zeroEnd = zeroStart + zeroCount;

	t0 = hal_timerGet();
	poison_walk(sp, &w);
	ms = hal_timerGet() - t0;

	lib_printf("\npoison: filled %llu bytes (%zu pages, %llu MiB), zeroed stream pages [%llu, %llu), %llu ms",
		(unsigned long long)total * SIZE_PAGE, total, ((unsigned long long)total * SIZE_PAGE) >> 20,
		(unsigned long long)zeroStart, (unsigned long long)(zeroStart + zeroCount), (unsigned long long)ms);

	return CMD_EXIT_SUCCESS;
}


static const cmd_t poison_cmd __attribute__((section("commands"), used)) = {
	.name = "poison", .run = cmd_poison, .info = cmd_poisonInfo
};
