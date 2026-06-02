/*
 * Phoenix-RTOS
 *
 * Operating system loader
 *
 * AArch64 architectural counter timer
 *
 * Copyright 2026 Phoenix Systems
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <hal/hal.h>


static struct {
	u64 freq;
	u64 start;
} timer_common;


time_t hal_timerGet(void)
{
	u64 now;

	now = sysreg_read(cntpct_el0);
	if (timer_common.freq == 0) {
		return 0;
	}

	return (now - timer_common.start) * 1000u / timer_common.freq;
}


void timer_done(void)
{
}


void timer_init(void)
{
	timer_common.freq = sysreg_read(cntfrq_el0);
	timer_common.start = sysreg_read(cntpct_el0);
}
