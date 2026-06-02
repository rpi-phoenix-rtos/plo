/*
 * Phoenix-RTOS
 *
 * Operating system loader
 *
 * GICv2 compatible interrupt controller driver
 *
 * Copyright 2026 Phoenix Systems
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <hal/hal.h>


#define SIZE_INTERRUPTS 256
#define SPI_FIRST_IRQID 32

#define DEFAULT_PRIORITY 0x80

enum {
	gicd_ctlr = 0x0 / sizeof(u32),
	gicd_typer = 0x4 / sizeof(u32),
	gicd_igroupr0 = 0x80 / sizeof(u32),
	gicd_isenabler0 = 0x100 / sizeof(u32),
	gicd_icenabler0 = 0x180 / sizeof(u32),
	gicd_icpendr0 = 0x280 / sizeof(u32),
	gicd_icactiver0 = 0x380 / sizeof(u32),
	gicd_ipriorityr0 = 0x400 / sizeof(u32),
	gicd_itargetsr0 = 0x800 / sizeof(u32),
	gicd_icfgr0 = 0xc00 / sizeof(u32)
};

enum {
	gicc_ctlr = 0x0 / sizeof(u32),
	gicc_pmr = 0x4 / sizeof(u32),
	gicc_bpr = 0x8 / sizeof(u32),
	gicc_iar = 0xc / sizeof(u32),
	gicc_eoir = 0x10 / sizeof(u32)
};

enum {
	gicv2_cfg_reserved = 0,
	gicv2_cfg_high_level = 1,
	gicv2_cfg_rising_edge = 3
};


typedef struct {
	int (*f)(unsigned int, void *);
	void *data;
} intr_handler_t;


static struct {
	volatile u32 *gicd;
	volatile u32 *gicc;
	intr_handler_t handlers[SIZE_INTERRUPTS];
} interrupts_common;


static int interrupts_isEl3(void)
{
	return (sysreg_read(currentEL) == 0xcU) ? 1 : 0;
}


void hal_interruptsEnable(unsigned int irqn)
{
	unsigned int irqReg;
	unsigned int irqOffs;

	if (irqn >= SIZE_INTERRUPTS) {
		return;
	}

	irqReg = irqn / 32;
	irqOffs = irqn % 32;

	*(interrupts_common.gicd + gicd_isenabler0 + irqReg) = 1u << irqOffs;
	hal_cpuDataMemoryBarrier();
}


void hal_interruptsDisable(unsigned int irqn)
{
	unsigned int irqReg;
	unsigned int irqOffs;

	if (irqn >= SIZE_INTERRUPTS) {
		return;
	}

	irqReg = irqn / 32;
	irqOffs = irqn % 32;

	*(interrupts_common.gicd + gicd_icenabler0 + irqReg) = 1u << irqOffs;
	hal_cpuDataMemoryBarrier();
}


static void interrupts_setConf(unsigned int irqn, u32 conf)
{
	unsigned int irqReg;
	unsigned int irqOffs;
	u32 mask;

	if ((irqn < 16) || (irqn >= SIZE_INTERRUPTS) || (conf == gicv2_cfg_reserved)) {
		return;
	}

	irqReg = irqn / 16;
	irqOffs = (irqn % 16) * 2;
	mask = *(interrupts_common.gicd + gicd_icfgr0 + irqReg) & ~(0x3 << irqOffs);

	*(interrupts_common.gicd + gicd_icfgr0 + irqReg) = mask | ((conf & 0x3) << irqOffs);
}


static void interrupts_setCPU(unsigned int irqn, u32 cpuId)
{
	unsigned int irqReg;
	unsigned int irqOffs;
	u32 mask;

	if ((irqn < SPI_FIRST_IRQID) || (irqn >= SIZE_INTERRUPTS)) {
		return;
	}

	irqReg = irqn / 4;
	irqOffs = (irqn % 4) * 8;
	mask = *(interrupts_common.gicd + gicd_itargetsr0 + irqReg) & ~(0xff << irqOffs);

	*(interrupts_common.gicd + gicd_itargetsr0 + irqReg) = mask | ((cpuId & 0xff) << irqOffs);
}


static void interrupts_setPriority(unsigned int irqn, u32 priority)
{
	unsigned int irqReg;
	unsigned int irqOffs;
	u32 mask;

	irqReg = irqn / 4;
	irqOffs = (irqn % 4) * 8;
	mask = *(interrupts_common.gicd + gicd_ipriorityr0 + irqReg) & ~(0xff << irqOffs);

	*(interrupts_common.gicd + gicd_ipriorityr0 + irqReg) = mask | ((priority & 0xff) << irqOffs);
}


void interrupts_dispatch(void)
{
	u32 irqn;

	irqn = *(interrupts_common.gicc + gicc_iar) & 0x3ff;

	if ((irqn < SIZE_INTERRUPTS) && (interrupts_common.handlers[irqn].f != NULL)) {
		interrupts_common.handlers[irqn].f(irqn, interrupts_common.handlers[irqn].data);
	}

	*(interrupts_common.gicc + gicc_eoir) = irqn;
}


int hal_interruptsSet(unsigned int irqn, int (*f)(unsigned int, void *), void *data)
{
	if (irqn >= SIZE_INTERRUPTS) {
		return -1;
	}

	hal_interruptsDisableAll();

	interrupts_common.handlers[irqn].data = data;
	interrupts_common.handlers[irqn].f = f;

	if (f == NULL) {
		hal_interruptsDisable(irqn);
	}
	else {
		interrupts_setPriority(irqn, DEFAULT_PRIORITY);
		interrupts_setCPU(irqn, 0x1);
		hal_interruptsEnable(irqn);
	}

	hal_interruptsEnableAll();

	return 0;
}


void interrupts_init(void)
{
	unsigned int i;
	unsigned int nRegs;
	int el3;

	interrupts_common.gicd = GICD_BASE_ADDRESS;
	interrupts_common.gicc = GICC_BASE_ADDRESS;

	for (i = 0; i < SIZE_INTERRUPTS; ++i) {
		interrupts_common.handlers[i].f = NULL;
		interrupts_common.handlers[i].data = NULL;
	}

	nRegs = ((*(interrupts_common.gicd + gicd_typer) & 0x1f) + 1);
	if ((nRegs * 32) > SIZE_INTERRUPTS) {
		nRegs = SIZE_INTERRUPTS / 32;
	}

	el3 = interrupts_isEl3();

	*(interrupts_common.gicd + gicd_ctlr) = 0;

	for (i = 0; i < nRegs; ++i) {
		*(interrupts_common.gicd + gicd_icenabler0 + i) = 0xffffffff;
		*(interrupts_common.gicd + gicd_icpendr0 + i) = 0xffffffff;
		*(interrupts_common.gicd + gicd_icactiver0 + i) = 0xffffffff;
	}

	if (el3 != 0) {
		for (i = 0; i < nRegs; ++i) {
			*(interrupts_common.gicd + gicd_igroupr0 + i) = 0xffffffff;
		}
	}

	for (i = 16; i < SIZE_INTERRUPTS; ++i) {
		interrupts_setConf(i, gicv2_cfg_high_level);
	}

	*(interrupts_common.gicc + gicc_pmr) = 0xff;
	if (el3 != 0) {
		*(interrupts_common.gicc + gicc_bpr) = 2;
		*(interrupts_common.gicc + gicc_ctlr) = 0x1b;
		*(interrupts_common.gicd + gicd_ctlr) = 0x3;
	}
	else {
		*(interrupts_common.gicc + gicc_bpr) = 0;
		*(interrupts_common.gicc + gicc_ctlr) = 1;
		*(interrupts_common.gicd + gicd_ctlr) = 1;
	}
}
