/*
 * Copyright (c) 2023 Arm Limited (or its affiliates). All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _MACRO_PRIV_INC_
#define _MACRO_PRIV_INC_

#include <zephyr/arch/arm/cortex_a_r/tpidruro.h>

/*
 * Get CPU id
 */

.macro get_cpu_id rreg0
	/* Read MPIDR register */
	mrc p15, 0, \rreg0, c0, c0, 5
	ubfx	\rreg0, \rreg0, #0, #24
.endm

/*
 * Get CPU logic id by looking up cpu_node_list
 * returns
 *   reg0: MPID
 *   reg1: logic id (0 ~ CONFIG_MP_MAX_NUM_CPUS - 1)
 * clobbers: reg0, reg1, reg2, reg3
 */
.macro get_cpu_logic_id reg0, reg1, reg2, reg3
	get_cpu_id \reg0
	ldr	\reg3, =cpu_node_list
	mov	\reg1, #0
1:	ldr	\reg2, [\reg3, \reg1, lsl #2]
	cmp	\reg2, \reg0
	beq	2f
	add	\reg1, \reg1, #1
	cmp	\reg1, #CONFIG_MP_MAX_NUM_CPUS
	bne	1b
	b	.
2:
.endm

.macro get_cpu rreg0
	/*
         * Get CPU pointer.
	 */
	mrc p15, 0, \rreg0, c13, c0, 3
        and \rreg0, #TPIDRURO_CURR_CPU
.endm

#if defined(CONFIG_USE_SWITCH) && defined(CONFIG_FPU_SHARING)
/*
 * CONFIG_USE_SWITCH uses an eager, complete VFP exception frame.  This is the
 * correctness baseline for hard-float C handlers: every nesting level owns the
 * live VFP state while the state it interrupted remains on that level's exact
 * exception-frame SP.
 *
 * All source GPRs have already been saved before these macros are used.
 */
.macro z_arm_cortex_ar_fpu_save_stack
	sub	sp, sp, #___fpu_t_SIZEOF

	vmrs	ip, fpexc
	orr	r1, ip, #FPEXC_EN
	vmsr	fpexc, r1

	mov	r2, sp
	vstmia	r2!, {d0-d15}
#ifdef CONFIG_VFP_FEATURE_REGS_S64_D32
	vstmia	r2!, {d16-d31}
#endif
	vmrs	r1, fpscr
	stm	r2, {r1, ip}
.endm

/*
 * Restore a complete VFP image without changing the frame pointer.
 *
 * frame: address of struct __fpu_sf
 * addr/control/saved: scratch GPRs; control may alias frame after addr is set
 */
.macro z_arm_cortex_ar_fpu_restore_ptr frame, addr, control, saved
	mov	\addr, \frame
	ldr	\saved, [\addr, #___fpu_t_SIZEOF - 4]
	orr	\control, \saved, #FPEXC_EN
	vmsr	fpexc, \control

	vldmia	\addr!, {d0-d15}
#ifdef CONFIG_VFP_FEATURE_REGS_S64_D32
	vldmia	\addr!, {d16-d31}
#endif
	ldm	\addr, {\control, \saved}
	vmsr	fpscr, \control
	vmsr	fpexc, \saved
.endm

.macro z_arm_cortex_ar_fpu_restore_stack
	z_arm_cortex_ar_fpu_restore_ptr sp, r2, r1, ip
	add	sp, sp, #___fpu_t_SIZEOF
.endm
#endif /* CONFIG_USE_SWITCH && CONFIG_FPU_SHARING */

/*
 * An exception can interrupt code while its stack is only word-aligned.
 * Preserve the exact exception-frame SP in an 8-byte private record and align
 * the active stack before calling a public C interface.
 *
 * The source GPRs are already part of the exception frame.  Keep the record
 * private to assembly: C receives the exact frame pointer in \frame.
 */
.macro z_arm_cortex_ar_align_exc_stack frame, pad
	mov	\frame, sp
	and	\pad, sp, #4
	sub	sp, sp, \pad
	push	{\frame, \pad}
.endm

.macro z_arm_cortex_ar_restore_exc_stack frame, pad
	pop	{\frame, \pad}
	mov	sp, \frame
.endm

.macro z_arm_cortex_ar_enter_exc
	/*
	 * Store r0-r3, r12, lr into the stack to construct an exception
	 * stack frame.
	 */
	srsdb sp!, #MODE_SYS
	cps #MODE_SYS
	push {r0-r3, r12, lr}

#if defined(CONFIG_USE_SWITCH) && defined(CONFIG_FPU_SHARING)
	z_arm_cortex_ar_fpu_save_stack
#endif

	/* TODO: EXTRA_EXCEPTION_INFO */
	mov r0, sp

	/* increment exception depth */
	get_cpu r2
	ldrb r1, [r2, #_cpu_offset_to_exc_depth]
	add r1, r1, #1
	strb r1, [r2, #_cpu_offset_to_exc_depth]
.endm

#endif /* _MACRO_PRIV_INC_ */
