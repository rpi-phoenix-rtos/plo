/*
 * Phoenix-RTOS
 *
 * Operating system loader
 *
 * Low-level initialization for generic AArch64
 *
 * Copyright 2026 Phoenix Systems
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#define __ASSEMBLY__

#include "config.h"
#include "../cpu.h"

.section .init, "ax"

#define UARTFR_OFFSET 0x18
#define UARTFR_TXFF   (1 << 5)

.extern hal_common
.extern hal_coreJumpFlag
.extern hal_firmwareDtb


.macro uart_putc imm
991:
	ldr x3, =(PLO_UART0_BASE_ADDRESS + UARTFR_OFFSET)
	ldr w4, [x3]
	tst w4, #UARTFR_TXFF
	b.ne 991b
	ldr x3, =PLO_UART0_BASE_ADDRESS
	mov w4, #\imm
	str w4, [x3]
.endm


.globl _start
.type _start, %function
_start:
	b _start_after_signature


_start_signature:
	.word PLO_RPI_FIXED_ENTRY_SIGNATURE0
	.word PLO_RPI_FIXED_ENTRY_SIGNATURE1


_start_after_signature:
	mov x19, x0
	b _start_real


_start_real:
	mov x0, #0
	mov x1, #0
	mov x2, #0
	mov x3, #0
	mov x4, #0
	mov x5, #0
	mov x6, #0
	mov x7, #0

	mov x8, #0
	mov x9, #0
	mov x10, #0
	mov x11, #0
	mov x12, #0
	mov x13, #0
	mov x14, #0
	mov x15, #0

	mov x16, #0
	mov x17, #0
	mov x18, #0
	mov x20, #0
	mov x21, #0
	mov x22, #0
	mov x23, #0

	mov x24, #0
	mov x25, #0
	mov x26, #0
	mov x27, #0
	mov x28, #0
	mov x29, #0
	mov x30, #0

	dsb sy
	isb

	mrs x20, currentEL
	cmp x20, #0xc
	b.eq start_el3
	cmp x20, #0x8
	b.eq start_el2
	cmp x20, #0x4
	b.eq start_el1
	b start_el_unknown

start_el3:
	ldr x0, =0x30c50838
	msr sctlr_el3, x0

	msr cptr_el3, xzr

	mov x0, #0b1110011
	msr actlr_el3, x0

	msr cntvoff_el2, xzr
	mov x0, #0x3
	msr cnthctl_el2, x0
	msr cptr_el2, xzr
	ldr x0, =0x30c00838
	msr sctlr_el2, x0
	msr sctlr_el1, xzr
	msr vttbr_el2, xzr

	mov x0, #0b1110011
	msr actlr_el2, x0
	mov x0, #((1 << 31) | (1 << 29))
	msr hcr_el2, x0

	mov x0, #0x53e
	msr scr_el3, x0

	adr x0, _vector_table
	msr vbar_el3, x0

	ic iallu
	tlbi alle3

	b start_common


/* start_el2: minimal EL2 setup, then eret to EL1.
 *
 * Canonical AArch64 boot recipe followed by Circle, rust-raspberrypi-
 * OS-tutorials, NetBSD evbarm rpi4, U-Boot, Linux: ALL MMU + cache
 * enable work runs at EL1, not EL2. The EL2 stage is purely a
 * pass-through: set HCR_EL2.RW so EL1 is AArch64, set the EL1 timer
 * counter accessibility bits, ERET. Phoenix's earlier "run plo MMU
 * at EL2" approach exposed cache-aliasing pathologies that didn't
 * happen at EL1 in any of the three reference projects.
 *
 * What we configure at EL2 BEFORE the drop:
 *   - DAIF masked (defensive — armstub doesn't guarantee a clean state).
 *   - CNTVOFF_EL2 = 0, CNTHCTL_EL2 = 3 so EL1 can access the timer.
 *   - CPTR_EL2 = 0x33ff (RES1 + FP/SIMD trap disable), MDCR_EL2 = 0,
 *     HSTR_EL2 = 0, VTTBR_EL2 = 0 — all the trap-class registers
 *     zeroed (UNKNOWN at reset on A72, must be set explicitly).
 *   - VPIDR_EL2/VMPIDR_EL2 mirror MIDR/MPIDR so EL1 reads sane IDs.
 *   - HCR_EL2.RW = 1 (EL1 is AArch64), nothing else — we don't
 *     enable AMO/IMO/FMO because we're not staying at EL2 to handle
 *     them.
 *   - SCTLR_EL2 baseline keeps RES1 bits, M/C/I all OFF.
 *   - SCTLR_EL1 zero except RES1 (will be configured by start_el1).
 *   - VBAR_EL2 -> _vector_table as a safety net if a sync exception
 *     fires during the brief EL2 window before ERET.
 *   - SP_EL1 -> _stack so EL1 has a stack the instant it starts.
 *
 * Then SPSR_EL2 = EL1h with DAIF masked, ELR_EL2 -> start_el1, ERET.
 */
start_el2:
	msr daifSet, #0xf

	/* Timer accessibility for EL1 (CNTVOFF_EL2 must be 0, CNTHCTL_EL2
	 * bits[1:0] = EL1PCEN | EL1PCTEN). */
	msr cntvoff_el2, xzr
	mov x0, #0x3
	msr cnthctl_el2, x0

	/* CPTR_EL2 = 0x33ff: RES1 bits + FP/SIMD trap disable. Per Linux
	 * __init_el2_state and U-Boot armv8. */
	mov x0, #0x33ff
	msr cptr_el2, x0

	/* SCTLR_EL2 RES1 baseline (no M/C/I). 0x30c50838 sets bits 29, 28,
	 * 23, 22, 18, 16, 11, 5, 4, 3 — all RES1 on A72 r0p3 non-VHE. */
	ldr x0, =0x30c50838
	msr sctlr_el2, x0

	/* SCTLR_EL1 RES1 baseline (no M/C/I) — start_el1 will pick this
	 * up via mmu_enable() which RMWs the SCTLR. */
	ldr x0, =0x30c50838
	msr sctlr_el1, x0

	/* Zero MDCR_EL2 / HSTR_EL2 / VTTBR_EL2 — UNKNOWN at reset on A72. */
	msr mdcr_el2, xzr
	msr hstr_el2, xzr
	msr vttbr_el2, xzr

	/* Mirror MIDR_EL1 -> VPIDR_EL2, MPIDR_EL1 -> VMPIDR_EL2. */
	mrs x0, midr_el1
	msr vpidr_el2, x0
	mrs x0, mpidr_el1
	msr vmpidr_el2, x0

	/* ACTLR_EL2 = 0x73: grant EL1 access to CPUACTLR_EL1 / CPUECTLR_EL1
	 * / L2CTLR_EL1 (A72 IMP-DEF). */
	mov x0, #0b1110011
	msr actlr_el2, x0

	/* HCR_EL2.RW = 1 (EL1 is AArch64). No AMO/IMO/FMO — we don't run
	 * any code at EL2 after this, so SError/IRQ/FIQ routing is N/A. */
	mov x0, #(1 << 31)
	msr hcr_el2, x0

	/* VBAR_EL2 safety net for any sync exception in the brief EL2 window. */
	adr x0, _vector_table
	msr vbar_el2, x0

	/* Hand EL1 a usable stack and vector table. */
	adr x0, _stack
	msr sp_el1, x0

	isb
	ic iallu
	tlbi alle2
	dsb sy
	isb

	/* Prepare ERET to start_el1. SPSR_EL2 = EL1h | D|A|I|F masked = 0x3c5
	 * (bits[3:0] = 0b0101 = EL1h, bits[9:6] = 0b1111 = DAIF). */
	mov x0, #0x3c5
	msr spsr_el2, x0
	adr x0, start_el1
	msr elr_el2, x0
	eret


start_el1:
	adr x0, _vector_table
	msr vbar_el1, x0

	mrs x0, cpacr_el1
	orr x0, x0, #(0x3 << 20)
	msr cpacr_el1, x0

	ic iallu
	tlbi vmalle1


start_common:
	dsb sy
	isb

	/* Per ARM ARM B2.2.6 + DEN0024 + the convergent evidence of Circle,
	 * rust-raspberrypi-OS-tutorials, NetBSD, U-Boot, and Linux on Pi 4:
	 * set/way cache maintenance (dc isw) on the boot path is "for cache
	 * power-down ONLY, not for coherency". None of those projects do
	 * it; they trust the upstream stage (armstub/firmware) to leave
	 * caches off and the cluster L1 contents UNKNOWN-but-harmless,
	 * because plo's MMU-enable code runs with the cache off and the
	 * very first cacheable access happens only after SCTLR.M|C|I=1.
	 *
	 * (Historical context: the previous TD-05 set/way loop was
	 * motivated by a misdiagnosed VPU-side-L2 aliasing hypothesis.
	 * Multi-subagent research 2026-05-12 falsified that — there is no
	 * VPU L2 on the ARM->DRAM path on BCM2711; the system L2 is
	 * GPU-only. So this loop was removing dirty lines that the boot
	 * path didn't actually depend on, while making the boot
	 * non-deterministic by running an architecturally-deprecated
	 * primitive at an early phase.) */

	ldr x2, =hal_firmwareDtb
	str x19, [x2]

	adr x1, _stack
	mov sp, x1

	/* Trap all cores except core 0 until the kernel handoff. */
	adr x1, hal_coreJumpFlag
	str xzr, [x1]
	mrs x0, mpidr_el1
	and x0, x0, #0xff
	cbz x0, start_core_0
other_core_trap:
	dsb ish
	wfe
	/*
	 * NOTE: cores 1-3 do not actually reach this point in the current
	 * configuration — the rpi4 armstub parks them in its own
	 * `secondary_spin` loop at the standard spin-table addresses
	 * (PA 0xE0 / 0xE8 / 0xF0). Plo's hal_smpBringupSecondaries (called
	 * from C, see hal.c) writes the secondary entry address into those
	 * armstub slots and issues SEV, which jumps cores 1-3 into
	 * secondary_smoke_entry below (NOT here). This local trap is kept
	 * as defense-in-depth in case the armstub layout ever changes.
	 */
	b other_core_trap

start_core_0:
	adr x1, _startc
	br x1


/* SMP smoke trampoline. Reached by cores 1-3 from the armstub
 * spin-table after plo writes the entry address there and does SEV.
 * Cores arrive at EL2 with caches off (armstub state), MMU off, and
 * no usable stack. We just print 'c<digit>: alive\n' via direct PL011
 * MMIO (MMU off -> VA = PA, no translation) and park in WFE.
 *
 * UART writes from secondary cores can race with core 0's writes —
 * we accept the interleaving for this smoke test. Each character is
 * a single 32-bit store to PL011_DR which is atomic at word level.
 *
 * No stack use: only x3..x7 scratch registers, no `bl`.
 */
.globl secondary_smoke_entry
.type secondary_smoke_entry, %function
secondary_smoke_entry:
	/* SMP Phase D plo-side marker. 'H' for cpu0 (shouldn't happen),
	 * 'I'/'J'/'K' for cpu1/2/3. Confirms the secondary reached plo's
	 * smoke entry. If 'I'/'J'/'K' don't appear, secondaries jumped
	 * elsewhere (or crashed) after the armstub `br x4`. */
	mrs x6, mpidr_el1
	and x6, x6, #0x3        /* core ID 0..3 */

	mov w4, w6
	add w4, w4, #'H'
9800:
	ldr x3, =(PLO_UART0_BASE_ADDRESS + UARTFR_OFFSET)
	ldr w5, [x3]
	tst w5, #UARTFR_TXFF
	b.ne 9800b
	ldr x3, =PLO_UART0_BASE_ADDRESS
	str w4, [x3]

	/* spin until UART TX FIFO has room, then write a char (in w4) */
	/* 'c' */
	mov w4, #'c'
1:
	ldr x3, =(PLO_UART0_BASE_ADDRESS + UARTFR_OFFSET)
	ldr w5, [x3]
	tst w5, #UARTFR_TXFF
	b.ne 1b
	ldr x3, =PLO_UART0_BASE_ADDRESS
	str w4, [x3]

	/* digit = '0' + core ID */
	add w4, w6, #'0'
2:
	ldr x3, =(PLO_UART0_BASE_ADDRESS + UARTFR_OFFSET)
	ldr w5, [x3]
	tst w5, #UARTFR_TXFF
	b.ne 2b
	ldr x3, =PLO_UART0_BASE_ADDRESS
	str w4, [x3]

	/* ':' */
	mov w4, #':'
3:
	ldr x3, =(PLO_UART0_BASE_ADDRESS + UARTFR_OFFSET)
	ldr w5, [x3]
	tst w5, #UARTFR_TXFF
	b.ne 3b
	ldr x3, =PLO_UART0_BASE_ADDRESS
	str w4, [x3]

	/* ' ' */
	mov w4, #' '
4:
	ldr x3, =(PLO_UART0_BASE_ADDRESS + UARTFR_OFFSET)
	ldr w5, [x3]
	tst w5, #UARTFR_TXFF
	b.ne 4b
	ldr x3, =PLO_UART0_BASE_ADDRESS
	str w4, [x3]

	/* 'a' */
	mov w4, #'a'
5:
	ldr x3, =(PLO_UART0_BASE_ADDRESS + UARTFR_OFFSET)
	ldr w5, [x3]
	tst w5, #UARTFR_TXFF
	b.ne 5b
	ldr x3, =PLO_UART0_BASE_ADDRESS
	str w4, [x3]

	/* '\n' */
	mov w4, #10
6:
	ldr x3, =(PLO_UART0_BASE_ADDRESS + UARTFR_OFFSET)
	ldr w5, [x3]
	tst w5, #UARTFR_TXFF
	b.ne 6b
	ldr x3, =PLO_UART0_BASE_ADDRESS
	str w4, [x3]

#if defined(PLO_SMP_ENABLE) && (PLO_SMP_ENABLE != 0)
	/* SMP Phase A handoff: after the smoke marker, busy-poll
	 * spin_cpuN until plo's hal_cpuJump release-2 overwrites it with
	 * the kernel entry PA. Then set x9 = syspage PA and branch to
	 * the new value. Compiled out by default (see PLO_SMP_ENABLE gate
	 * in hal.c) until Phase B closes the secondary re-entry loop. */
secondary_handoff:
	mrs x6, mpidr_el1
	and x6, x6, #0x3              /* core ID (0..3) */
	mov x5, #0xd8                  /* armstub spin_cpu0 base PA */
	add x5, x5, x6, lsl #3         /* address of our spin_cpuN */
	dc ivac, x5
	dsb sy
	ldr x8, [x5]                  /* snapshot initial value */
1:
	dc ivac, x5
	dsb sy
	ldr x4, [x5]
	cmp x4, x8
	b.eq 1b                        /* same as snapshot, keep polling */

	/* Kernel _start expects x9 = syspage PA from plo (primary takes
	 * this via hal_exitToEL1's `ldp x9, x0, [x0]` from hal_common). */
	adrp x10, hal_common
	add x10, x10, #:lo12:hal_common
	ldr x9, [x10]
	mov x0, #0
	mov x1, #0
	mov x2, #0
	mov x3, #0
	br x4
#endif

secondary_park:
	/* Default destination after secondary_smoke_entry: WFE forever
	 * in plo memory. Pre-Phase-A behaviour. */
	dsb sy
	wfe
	b secondary_park
.size secondary_smoke_entry, .-secondary_smoke_entry


start_el_unknown:

error_loop:
	b .

.size _start, .-_start


.align 4
.globl hal_exitToEL1
.type hal_exitToEL1, %function
hal_exitToEL1:
	dsb sy
	isb
	uart_putc 10
	uart_putc 65
	adr x0, hal_common
	ldp x9, x0, [x0]
	mrs x1, currentEL
	cmp x1, #0xc
	b.eq exit_el3
	cmp x1, #0x8
	b.eq exit_el2
	cmp x1, #0x4
	b.eq exit_el1
	b error_loop

exit_el3:
	uart_putc 51
	uart_putc 10
	msr elr_el3, x0
	mov x0, #0b1111000101
	msr spsr_el3, x0
	mov x2, #0x5b1
	msr scr_el3, x2
	eret

exit_el2:
	uart_putc 50
	uart_putc 10
	msr elr_el2, x0
	mov x0, #0b1111000101
	msr spsr_el2, x0
	eret

exit_el1:
	uart_putc 49
	uart_putc 10
	br x0
.size hal_exitToEL1, .-hal_exitToEL1


#include "../_interrupts.S"
#include "../_exceptions.S"

/* TD-diag: every vector slot prints a unique tag char then halts, so
 * we can SEE which exception class fires instead of silently looping
 * on `b .` or trapping deeper on _exceptions_dispatch's EL3-only sysreg
 * accesses. Tag mapping:
 *   A 000  SP_EL0 sync     B 080  SP_EL0 irq     C 100  SP_EL0 fiq    D 180  SP_EL0 serror
 *   E 200  SPx    sync     F 280  SPx    irq     G 300  SPx    fiq    H 380  SPx    serror
 *   I 400  L-EL64 sync     J 480  L-EL64 irq     K 500  L-EL64 fiq    L 580  L-EL64 serror
 *   M 600  L-EL32 sync     N 680  L-EL32 irq     O 700  L-EL32 fiq    P 780  L-EL32 serror
 * Each slot prints "EXC" then the tag, then halts. After diagnosis we
 * either fix the offending exception cause or restore the original
 * vector table.
 */
.macro exc_tag tag
	uart_putc \tag
	uart_putc '\n'
	b .
.endm

/* Print one ASCII hex nibble for the low 4 bits of w4. */
.macro uart_put_hexnibble
	and w4, w4, #0xf
	cmp w4, #10
	b.lo 992f
	add w4, w4, #('a' - 10)
	b 993f
992:
	add w4, w4, #'0'
993:
	ldr x3, =(PLO_UART0_BASE_ADDRESS + UARTFR_OFFSET)
994:
	ldr w5, [x3]
	tst w5, #UARTFR_TXFF
	b.ne 994b
	ldr x3, =PLO_UART0_BASE_ADDRESS
	str w4, [x3]
.endm

/* Print a 64-bit reg in x6 as 16 hex chars (high nibble first), no newline. */
.globl uart_put_hex64
.type uart_put_hex64, %function
uart_put_hex64:
	mov x7, #60
995:
	lsr x4, x6, x7
	uart_put_hexnibble
	cbz x7, 996f
	sub x7, x7, #4
	b 995b
996:
	ret
.size uart_put_hex64, .-uart_put_hex64

/* Slot-E (Sync at Current EL with SPx) full diagnostic. Reads EL2
 * exception registers (ESR_EL2, ELR_EL2, FAR_EL2, SPSR_EL2) and prints
 * them as hex. ARM ARM D13.2.36: ESR.EC[31:26] decodes the fault class.
 * Most likely cases for our boot:
 *   0x26 = SP alignment fault    (matches the SCTLR_EL2.SA=1 + RES1
 *                                 missing-bits hypothesis)
 *   0x25 = Data abort same EL    (DFSC field then narrows it)
 *   0x2f = SError                 (latent from armstub/VC4)
 *   0x18 = MSR/MRS trap          (would mean dispatcher EL3-reg)
 */
.globl _slot_e_dump
.type _slot_e_dump, %function
_slot_e_dump:
	uart_putc 'E'
	uart_putc '\n'
	uart_putc 'E'
	uart_putc 'S'
	uart_putc 'R'
	uart_putc '='
	mrs x6, esr_el1
	bl uart_put_hex64
	uart_putc '\n'
	uart_putc 'E'
	uart_putc 'L'
	uart_putc 'R'
	uart_putc '='
	mrs x6, elr_el1
	bl uart_put_hex64
	uart_putc '\n'
	uart_putc 'F'
	uart_putc 'A'
	uart_putc 'R'
	uart_putc '='
	mrs x6, far_el1
	bl uart_put_hex64
	uart_putc '\n'
	uart_putc 'S'
	uart_putc 'P'
	uart_putc 'R'
	uart_putc '='
	mrs x6, spsr_el1
	bl uart_put_hex64
	uart_putc '\n'
	uart_putc 'S'
	uart_putc 'C'
	uart_putc 'T'
	uart_putc '='
	mrs x6, sctlr_el1
	bl uart_put_hex64
	uart_putc '\n'

	/* TTBR0_EL2 / TCR_EL2 / MAIR_EL2 — verify translation setup. */
	uart_putc 'T'
	uart_putc 'T'
	uart_putc 'B'
	uart_putc '='
	mrs x6, ttbr0_el1
	bl uart_put_hex64
	uart_putc '\n'
	uart_putc 'T'
	uart_putc 'C'
	uart_putc 'R'
	uart_putc '='
	mrs x6, tcr_el1
	bl uart_put_hex64
	uart_putc '\n'
	uart_putc 'M'
	uart_putc 'A'
	uart_putc 'R'
	uart_putc '='
	mrs x6, mair_el1
	bl uart_put_hex64
	uart_putc '\n'

	/* Read 4 instruction bytes at ELR_EL2. If these match the static
	 * disassembly (0x911c6000 for `add x0, x0, #0x718`) the I-cache is
	 * NOT the culprit and EC=0 must come from elsewhere. If they
	 * don't match, I-cache aliasing is proven. */
	uart_putc 'I'
	uart_putc 'N'
	uart_putc 'S'
	uart_putc '='
	mrs x8, elr_el1
	ldr w6, [x8]                /* writing w6 zero-extends to x6 */
	bl uart_put_hex64
	uart_putc '\n'
	b .
.size _slot_e_dump, .-_slot_e_dump

.align 11
.globl _vector_table
_vector_table:
.org(_vector_table)
	exc_tag 'A'

.org (_vector_table + 0x80)
	exc_tag 'B'

.org (_vector_table + 0x100)
	exc_tag 'C'

.org (_vector_table + 0x180)
	exc_tag 'D'

.org (_vector_table + 0x200)
	b _slot_e_dump

.org (_vector_table + 0x280)
	exc_tag 'F'

.org (_vector_table + 0x300)
	exc_tag 'G'

.org (_vector_table + 0x380)
	exc_tag 'H'

.org (_vector_table + 0x400)
	exc_tag 'I'

.org (_vector_table + 0x480)
	exc_tag 'J'

.org (_vector_table + 0x500)
	exc_tag 'K'

.org (_vector_table + 0x580)
	exc_tag 'L'

.org (_vector_table + 0x600)
	exc_tag 'M'

.org (_vector_table + 0x680)
	exc_tag 'N'

.org (_vector_table + 0x700)
	exc_tag 'O'

.org (_vector_table + 0x780)
	exc_tag 'P'

.org (_vector_table + 0x800)
