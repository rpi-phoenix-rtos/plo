/*
 * Phoenix-RTOS
 *
 * Entrypoint
 *
 * Copyright 2021-2022 Phoenix Systems
 * Author: Hubert Buczynski, Gerard Swiderski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <hal/hal.h>

extern char __bss_start[], __bss_end[];
extern char __data_load[], __data_start[], __data_end[];
extern char __rodata_load[], __rodata_start[], __rodata_end[];
extern char __ramtext_load[], __ramtext_start[], __ramtext_end[];
extern char __heap_base[], __heap_limit[];

extern void (*__init_array_start[])(void);
extern void (*__init_array_end[])(void);

extern void (*__fini_array_start[])(void);
extern void (*__fini_array_end[])(void);


extern int main(void);


void _startc(int argc, char **argv, char **env)
{
	size_t i, size;

	/* Load .fastram.text, .data and .rodata sections */
	if (&__ramtext_start[0] != &__ramtext_load[0]) {
		/* hal_memcpy may reside in fastram. */
		for (i = 0; i <= (__ramtext_end - __ramtext_start); i++) {
			__ramtext_start[i] = __ramtext_load[i];
		}
	}

	if (&__data_start[0] != &__data_load[0]) {
		hal_memcpy(__data_start, __data_load, __data_end - __data_start);
	}
	if (&__rodata_start[0] != &__rodata_load[0]) {
		hal_memcpy(__rodata_start, __rodata_load, __rodata_end - __rodata_start);
	}
	/* Clear the .bss section */
	hal_memset(__bss_start, 0, __bss_end - __bss_start);

	/* TD-05 diagnostic: zero the heap so any byte the syspage/allocator
	 * does not explicitly write reads back as 0 rather than as
	 * uninitialised DRAM. Helps distinguish "cache didn't flush dirty
	 * data to DDR" (would still be deterministic) from "plo never
	 * wrote that byte" (was showing up as bit-level nondeterminism
	 * across boots on Pi 4). Safe to keep: cost is one memset of 16 KB
	 * at startup. Revisit once the root cause is understood. */
	hal_memset(__heap_base, 0, __heap_limit - __heap_base);

	size = __init_array_end - __init_array_start;
	for (i = 0; i < size; i++) {
		(*__init_array_start[i])();
	}

	main();

	size = __fini_array_end - __fini_array_start;
	for (i = size; i > 0; i--) {
		(*__fini_array_start[i - 1])();
	}
}
