/*
 * Copyright 2026 iCNC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/toolchain.h>
#include <zephyr/linker/sections.h>

_ASM_FILE_PROLOGUE

GTEXT(test_arm_cortex_ar_fpu_trap_unaligned)

/*
 * void test_arm_cortex_ar_fpu_trap_unaligned(void);
 *
 * Trigger a lazy VFP undefined-instruction exception with SP % 8 == 4.  The
 * exception path must align its C stack without losing the exact return SP.
 */
SECTION_FUNC(TEXT, test_arm_cortex_ar_fpu_trap_unaligned)
	push {lr}
	vmov s0, s0
	pop {pc}
