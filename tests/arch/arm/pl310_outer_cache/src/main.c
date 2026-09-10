/*
 * Copyright (c) 2026 Opus One
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * The PL310 outer cache against a fake controller: a 2 KiB buffer standing in
 * for the register window, so that the boot sequence, the by-way operations,
 * the line arithmetic and the virtual-to-physical translation can all be read
 * back and checked.  Every register this driver touches has its own offset, so
 * a plain buffer records the whole init sequence; a range operation writes one
 * register repeatedly, so what a buffer preserves there is the last line of
 * the range -- which is exactly the end of the arithmetic worth checking.
 */

#include <zephyr/ztest.h>
#include <zephyr/cache.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/util.h>
#include <zephyr/arch/arm/cortex_a_r/outer_cache.h>

#include "soc.h"

#define PL310_CTRL           0x100U
#define PL310_AUX_CTRL       0x104U
#define PL310_TAG_RAM_CTRL   0x108U
#define PL310_DATA_RAM_CTRL  0x10CU
#define PL310_CACHE_SYNC     0x730U
#define PL310_INV_PA         0x770U
#define PL310_INV_WAY        0x77CU
#define PL310_CLEAN_PA       0x7B0U
#define PL310_CLEAN_WAY      0x7BCU
#define PL310_CLEAN_INV_PA   0x7F0U
#define PL310_CLEAN_INV_WAY  0x7FCU

#define LINE 32U

static uint8_t fake[0x800] __aligned(4);
static uint8_t scratch[8192] __aligned(4096);

static uintptr_t saved_base;

static uint32_t reg(uint32_t off)
{
	uint32_t val;

	memcpy(&val, &fake[off], sizeof(val));
	return val;
}

static void use_fake(void)
{
	memset(fake, 0, sizeof(fake));
	zynq_pl310_base = (uintptr_t)fake;
}

static void *setup(void)
{
	saved_base = zynq_pl310_base;
	return NULL;
}

static void teardown(void *unused)
{
	ARG_UNUSED(unused);
	zynq_pl310_base = saved_base;
}

ZTEST(pl310_outer_cache, test_init_sequence)
{
	use_fake();

	zynq_pl310_init((uintptr_t)fake);

	zassert_equal(reg(PL310_AUX_CTRL), 0x72360000U,
		      "aux control: 8 ways of 64 KiB, prefetch, parity");
	zassert_equal(reg(PL310_TAG_RAM_CTRL), 0x00000111U, "tag RAM latency");
	zassert_equal(reg(PL310_DATA_RAM_CTRL), 0x00000121U, "data RAM latency");
	zassert_equal(reg(PL310_INV_WAY), 0x0000FFFFU, "all ways invalidated");
	zassert_equal(reg(PL310_CTRL), 1U, "controller enabled last");

	teardown(NULL);
}

ZTEST(pl310_outer_cache, test_way_operations)
{
	use_fake();
	zynq_pl310_base = (uintptr_t)fake;

	outer_cache_clean_all();
	zassert_equal(reg(PL310_CLEAN_WAY), 0x0000FFFFU, "clean by way");

	outer_cache_invd_all();
	zassert_equal(reg(PL310_INV_WAY), 0x0000FFFFU, "invalidate by way");

	outer_cache_flush_and_invd_all();
	zassert_equal(reg(PL310_CLEAN_INV_WAY), 0x0000FFFFU, "clean and invalidate by way");

	teardown(NULL);
}

/* An unaligned range must cover the line the first byte sits in and the line
 * the last byte sits in, and nothing past it.
 */
ZTEST(pl310_outer_cache, test_clean_range_line_arithmetic)
{
	uintptr_t start = (uintptr_t)scratch + 8U;
	size_t size = 100U;
	uintptr_t last_line = ROUND_DOWN(start + size - 1U, LINE);

	use_fake();

	outer_cache_clean_range((void *)start, size);

	zassert_equal(reg(PL310_CLEAN_PA), (uint32_t)last_line,
		      "last line of an unaligned clean range");

	/* A range that ends exactly on a line boundary must not reach into the
	 * next line.
	 */
	start = (uintptr_t)scratch;
	size = 2U * LINE;
	use_fake();
	outer_cache_clean_range((void *)start, size);
	zassert_equal(reg(PL310_CLEAN_PA), (uint32_t)(start + LINE),
		      "aligned range stops at its last line");

	teardown(NULL);
}

/* Invalidate keeps the partial lines at either end by cleaning them instead of
 * dropping them, and only the interior goes to the plain invalidate register.
 */
ZTEST(pl310_outer_cache, test_invd_range_partial_lines)
{
	uintptr_t start = (uintptr_t)scratch + 8U;
	size_t size = 100U;
	uintptr_t tail = ROUND_DOWN(start + size, LINE);
	uintptr_t head = ROUND_DOWN(start, LINE);

	use_fake();

	outer_cache_invd_range((void *)start, size);

	/* Head line first, tail line second: the tail is what survives. */
	zassert_equal(reg(PL310_CLEAN_INV_PA), (uint32_t)tail,
		      "unaligned tail line cleaned, not dropped");
	zassert_equal(reg(PL310_INV_PA), (uint32_t)(tail - LINE),
		      "interior runs up to, and not into, the tail line");
	zassert_not_equal(head, tail, "test range must straddle line boundaries");

	/* A line-aligned range uses the plain invalidate for every line. */
	use_fake();
	outer_cache_invd_range(scratch, 4U * LINE);
	zassert_equal(reg(PL310_CLEAN_INV_PA), 0U, "no partial lines to clean");
	zassert_equal(reg(PL310_INV_PA), (uint32_t)((uintptr_t)scratch + 3U * LINE),
		      "last line of an aligned invalidate range");

	teardown(NULL);
}

/* The range walk translates once per 4 KiB page; a range crossing a page
 * boundary must still end on its own last line.
 */
ZTEST(pl310_outer_cache, test_range_crosses_page)
{
	uintptr_t start = (uintptr_t)scratch + KB(4) - LINE;
	size_t size = 4U * LINE;

	use_fake();

	outer_cache_flush_and_invd_range((void *)start, size);

	zassert_equal(reg(PL310_CLEAN_INV_PA), (uint32_t)(start + size - LINE),
		      "last line of a range spanning two pages");

	teardown(NULL);
}

/*
 * The controller works on physical addresses.  Map a page of RAM a second
 * time, at a virtual address the MMU has to translate, and check that the
 * address handed to the controller is the physical one.
 */
ZTEST(pl310_outer_cache, test_virtual_to_physical)
{
	uintptr_t phys = (uintptr_t)scratch;
	mm_reg_t virt;

	device_map(&virt, phys, KB(4), K_MEM_CACHE_WB);
	zassert_not_equal((uintptr_t)virt, phys, "device_map must not return the identity VA");

	use_fake();

	outer_cache_clean_range((void *)(uintptr_t)virt, LINE);
	zassert_equal(reg(PL310_CLEAN_PA), (uint32_t)phys,
		      "aliased VA translated back to its physical address");

	outer_cache_clean_range((void *)((uintptr_t)virt + 3U * LINE), LINE);
	zassert_equal(reg(PL310_CLEAN_PA), (uint32_t)(phys + 3U * LINE),
		      "page offset preserved across the translation");

	teardown(NULL);
}

ZTEST_SUITE(pl310_outer_cache, NULL, setup, NULL, NULL, teardown);
