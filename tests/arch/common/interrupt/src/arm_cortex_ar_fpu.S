/*
 * Copyright 2026 iCNC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/toolchain.h>
#include <zephyr/linker/sections.h>

_ASM_FILE_PROLOGUE

GTEXT(test_arm_cortex_ar_fpu_load)
GTEXT(test_arm_cortex_ar_fpu_load_trigger_store)

/*
 * void test_arm_cortex_ar_fpu_load(const uint64_t *input, uint32_t fpscr);
 *
 * Intentionally violates the AAPCS VFP callee-save rule.  The caller is an ISR
 * test whose purpose is to model live FP state at an arbitrary instruction.
 */
SECTION_FUNC(TEXT, test_arm_cortex_ar_fpu_load)
	vldmia r0!, {d0-d15}
#ifdef CONFIG_VFP_FEATURE_REGS_S64_D32
	vldmia r0!, {d16-d31}
#endif
	vmsr fpscr, r1
	bx lr

/*
 * void test_arm_cortex_ar_fpu_load_trigger_store(
 *	const uint64_t *input, uint32_t fpscr,
 *	uintptr_t trigger_reg, uint32_t trigger_value,
 *	const volatile uint32_t *done, uint32_t expected,
 *	uint64_t *output, uint32_t *fpscr_output);
 *
 * Load a complete live FP image, raise an SGI without crossing another C ABI
 * boundary, wait for the exception to return, then capture the resulting image.
 */
SECTION_FUNC(TEXT, test_arm_cortex_ar_fpu_load_trigger_store)
	push {r4-r10, lr}

	ldr r4, [sp, #32]	/* done */
	ldr r5, [sp, #36]	/* expected */
	ldr r6, [sp, #40]	/* output */
	ldr r7, [sp, #44]	/* fpscr_output */

	/*
	 * Deliberately interrupt a word-aligned stack.  The architecture entry
	 * path must retain this exact SP while presenting an aligned C stack and
	 * a safely word-aligned FP exception frame.
	 */
	sub sp, sp, #4

	vldmia r0!, {d0-d15}
#ifdef CONFIG_VFP_FEATURE_REGS_S64_D32
	vldmia r0!, {d16-d31}
#endif
	vmsr fpscr, r1

	str r3, [r2]
	dsb sy

1:
	ldr r8, [r4]
	cmp r8, r5
	bne 1b

	vstmia r6!, {d0-d15}
#ifdef CONFIG_VFP_FEATURE_REGS_S64_D32
	vstmia r6!, {d16-d31}
#endif
	vmrs r8, fpscr
	str r8, [r7]

	add sp, sp, #4
	pop {r4-r10, pc}
