/*
 * Copyright (c) 2026 Opus One
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Level-2 (outer) cache maintenance hooks for Cortex-A/R AArch32.
 *
 * The Cortex-A/R cache API maintains the CP15 level-1 caches by MVA.  A SoC
 * that sits behind an external, physically addressed outer cache controller
 * (Zynq-7000: ARM PL310) selects CONFIG_ARM_OUTER_CACHE and implements these
 * six operations, so that one sys_cache_data_* call maintains both levels.
 *
 * Addresses are virtual; the implementation translates them.  Every operation
 * has completed and drained when it returns.
 */

#ifndef ZEPHYR_INCLUDE_ARCH_ARM_CORTEX_A_R_OUTER_CACHE_H_
#define ZEPHYR_INCLUDE_ARCH_ARM_CORTEX_A_R_OUTER_CACHE_H_

#include <stddef.h>
#include <zephyr/toolchain.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_ARM_OUTER_CACHE

void outer_cache_clean_range(void *addr, size_t size);
void outer_cache_invd_range(void *addr, size_t size);
void outer_cache_flush_and_invd_range(void *addr, size_t size);
void outer_cache_clean_all(void);
void outer_cache_invd_all(void);
void outer_cache_flush_and_invd_all(void);

#else

static inline void outer_cache_clean_range(void *addr, size_t size)
{
	ARG_UNUSED(addr);
	ARG_UNUSED(size);
}

static inline void outer_cache_invd_range(void *addr, size_t size)
{
	ARG_UNUSED(addr);
	ARG_UNUSED(size);
}

static inline void outer_cache_flush_and_invd_range(void *addr, size_t size)
{
	ARG_UNUSED(addr);
	ARG_UNUSED(size);
}

static inline void outer_cache_clean_all(void)
{
}

static inline void outer_cache_invd_all(void)
{
}

static inline void outer_cache_flush_and_invd_all(void)
{
}

#endif /* CONFIG_ARM_OUTER_CACHE */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_ARCH_ARM_CORTEX_A_R_OUTER_CACHE_H_ */
