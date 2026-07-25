/*
 * Copyright (c) 2023 Arm Limited (or its affiliates). All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_ARCH_ARM_STRUCTS_H_
#define ZEPHYR_INCLUDE_ARCH_ARM_STRUCTS_H_

#include <zephyr/types.h>

#if defined(CONFIG_CPU_AARCH32_CORTEX_A) || defined(CONFIG_CPU_AARCH32_CORTEX_R)
/* Per CPU architecture specifics */
struct _cpu_arch {
	int8_t exc_depth;
#if defined(CONFIG_IRQ_OFFLOAD_NESTED)
	/*
	 * irq_offload may be entered recursively from an IRQ and concurrently
	 * on different CPUs.  Keep the active callback in the owning CPU's
	 * exception state instead of using one global slot.
	 */
	void (*irq_offload_routine)(const void *parameter);
	const void *irq_offload_param;
#endif
};

#else

/* Default definitions when no architecture specific definitions exist. */

/* Per CPU architecture specifics (empty) */
struct _cpu_arch {
#ifdef __cplusplus
	/* This struct will have a size 0 in C which is not allowed in C++ (it'll have a size 1). To
	 * prevent this, we add a 1 byte dummy variable.
	 */
	uint8_t dummy;
#endif
};

#endif

#endif /* ZEPHYR_INCLUDE_ARCH_ARM_STRUCTS_H_ */
