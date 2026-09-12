/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

/* Exercise the actual L1/outer composition with delayed L1 writeback. */
#include <zephyr/ztest.h>
#include <zephyr/cache.h>
#include <zephyr/arch/arm/cortex_a_r/outer_cache.h>
#include <zephyr/sys/barrier.h>
#include <cmsis_core.h>

#define FIRST_LINE 0x1000U
#define LINE_SIZE  32U
#define NUM_LINES  4U

static bool inner_dirty[NUM_LINES];
static bool pending[NUM_LINES];
static bool invalidate_pending[NUM_LINES];
static bool outer_dirty[NUM_LINES];
static bool memory_saved[NUM_LINES];
static bool inner_valid[NUM_LINES];
static uint32_t inner_cleans[NUM_LINES];

static size_t line_index(void *address)
{
	size_t index = ((uintptr_t)address - FIRST_LINE) / LINE_SIZE;

	zassert_true(index < NUM_LINES);
	return index;
}

static void clean_inner(void *address)
{
	size_t index = line_index(address);

	inner_cleans[index]++;
	pending[index] |= inner_dirty[index];
	inner_dirty[index] = false;
}

static void invalidate_inner(void *address)
{
	size_t index = line_index(address);

	inner_dirty[index] = false;
	invalidate_pending[index] = true;
}

static void __unused clean_invalidate_inner(void *address)
{
	clean_inner(address);
	invalidate_inner(address);
}

static void complete_inner(void)
{
	for (size_t i = 0U; i < NUM_LINES; i++) {
		outer_dirty[i] |= pending[i];
		pending[i] = false;
		if (invalidate_pending[i]) {
			inner_valid[i] = false;
			invalidate_pending[i] = false;
		}
	}
}

static void invalidate_outer(void *address, size_t size)
{
	uintptr_t start = (uintptr_t)address;
	uintptr_t end = start + size;

	for (uintptr_t line = ROUND_DOWN(start, LINE_SIZE); line < end; line += LINE_SIZE) {
		size_t index = line_index((void *)line);

		/* PL310 preserves the partial edges, and drops aligned interior lines. */
		if (line < start || end < line + LINE_SIZE) {
			memory_saved[index] |= outer_dirty[index];
		}
		outer_dirty[index] = false;
	}
}

/* CMSIS range operations issue DMB; only a DSB completes the pending writeback. */
#define L1C_CleanDCacheMVA           clean_inner
#define L1C_InvalidateDCacheMVA      invalidate_inner
#define L1C_CleanInvalidateDCacheMVA clean_invalidate_inner
#define barrier_dsync_fence_full     complete_inner
#define outer_cache_invd_range       invalidate_outer
#undef read_sysreg
#define read_sysreg(reg)                 (3U << 16)
#define arch_dcache_line_size_get        edge_line_size_get
#define arch_dcache_enable               edge_enable
#define arch_dcache_disable              edge_disable
#define arch_dcache_flush_all            edge_flush_all
#define arch_dcache_invd_all             edge_invd_all
#define arch_dcache_flush_and_invd_all   edge_flush_and_invd_all
#define arch_dcache_flush_range          edge_flush_range
#define arch_dcache_invd_range           edge_invd_range
#define arch_dcache_flush_and_invd_range edge_flush_and_invd_range
#define arch_icache_enable               edge_icache_enable
#define arch_icache_disable              edge_icache_disable
#define arch_icache_flush_all            edge_icache_flush_all
#define arch_icache_invd_all             edge_icache_invd_all
#define arch_icache_flush_and_invd_all   edge_icache_flush_and_invd_all
#define arch_icache_flush_range          edge_icache_flush_range
#define arch_icache_invd_range           edge_icache_invd_range
#define arch_icache_flush_and_invd_range edge_icache_flush_and_invd_range
#define arch_cache_init                  edge_cache_init
int edge_icache_invd_all(void);
#include "../../../../../arch/arm/core/cortex_a_r/cache.c"

static void reset_edges(void)
{
	memset(inner_dirty, 0, sizeof(inner_dirty));
	memset(pending, 0, sizeof(pending));
	memset(invalidate_pending, 0, sizeof(invalidate_pending));
	memset(outer_dirty, 0, sizeof(outer_dirty));
	memset(memory_saved, 0, sizeof(memory_saved));
	memset(inner_cleans, 0, sizeof(inner_cleans));
	for (size_t i = 0U; i < NUM_LINES; i++) {
		inner_valid[i] = true;
	}
}

ZTEST(pl310_edges, test_partial_inner_lines_reach_memory_before_outer_invalidate)
{
	const struct {
		uint32_t offset;
		size_t size;
	} ranges[] = {
		{8U, 100U}, {0U, 100U}, {8U, 120U}, {3U, 2U}, {0U, 128U}, {3U, 0U},
	};

	for (size_t range = 0U; range < ARRAY_SIZE(ranges); range++) {
		uintptr_t start = FIRST_LINE + ranges[range].offset;
		uintptr_t end = start + ranges[range].size;

		reset_edges();
		for (size_t i = 0U; i < NUM_LINES; i++) {
			uintptr_t line = FIRST_LINE + i * LINE_SIZE;
			bool partial = ranges[range].size > 0U &&
				       ((line < start && start < line + LINE_SIZE) ||
					(line < end && end < line + LINE_SIZE));

			/* The dirty bytes represent CPU-owned data outside the requested range. */
			inner_dirty[i] = partial;
		}
		zassert_ok(edge_invd_range((void *)start, ranges[range].size));
		for (size_t i = 0U; i < NUM_LINES; i++) {
			uintptr_t line = FIRST_LINE + i * LINE_SIZE;
			bool covered =
				ranges[range].size > 0U && line < end && start < line + LINE_SIZE;
			bool partial = covered && (line < start || end < line + LINE_SIZE);

			zassert_equal(memory_saved[i], partial,
				      "adjacent CPU data never reached RAM");
			zassert_equal(inner_cleans[i], partial ? 1U : 0U,
				      "clean only the two edges, once per line");
			zassert_false(pending[i] || outer_dirty[i],
				      "writeback escaped outer maintenance");
			zassert_equal(inner_valid[i], !covered);
		}
	}
}

ZTEST_SUITE(pl310_edges, NULL, NULL, NULL, NULL, NULL);
