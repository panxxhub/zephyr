/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

/* Run the SoC implementation against a controller with dirty, busy ways. */
#include <zephyr/ztest.h>
#include <zephyr/cache.h>
#include <zephyr/arch/arm/cortex_a_r/outer_cache.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/sys_io.h>
#include <cmsis_core.h>
#include "soc.h"

static uint32_t control;
static uint32_t sctlr;
static uint32_t busy_reg;
static uint32_t busy_reads;
static bool stuck;
static bool l1_dirty;
static bool l2_dirty;
static bool drained;
static size_t reconfigurations;
static size_t debug_writes;

static uint32_t model_read(mem_addr_t address)
{
	uint32_t reg = address - ZYNQ_PL310_BASE;

	if (reg == 0x100U) {
		return control;
	}
	if (reg == busy_reg && busy_reads > 0U) {
		if (!stuck) {
			busy_reads--;
		}
		return 0xFFFFU;
	}
	return 0U;
}

static void model_write(uint32_t value, mem_addr_t address)
{
	uint32_t reg = address - ZYNQ_PL310_BASE;

	zassert_equal(busy_reads, 0U, "command issued while by-way operation is busy");
	if (reg == 0x100U) {
		zassert_false(l1_dirty || l2_dirty,
			      "controller disabled before dirty data drained");
		control = value;
	} else if (reg == 0x104U || reg == 0x108U || reg == 0x10CU) {
		zassert_equal(control, 0U, "configuration changed while L2 enabled");
		reconfigurations++;
	} else if (reg == 0x7FCU) {
		zassert_equal(sctlr & SCTLR_C_Msk, 0U, "L1 can refill L2 during warm init");
		zassert_false(l1_dirty, "dirty L1 never reached L2");
		busy_reg = reg;
		busy_reads = 2U;
	} else if (reg == 0x730U) {
		l2_dirty = false;
		drained = true;
	} else if (reg == 0x77CU) {
		zassert_false(l2_dirty, "invalidate discarded dirty bootloader data");
		busy_reg = reg;
		busy_reads = 2U;
	} else if (reg == 0xF40U) {
		debug_writes++;
	}
}

static void model_disable(void)
{
	l2_dirty |= l1_dirty;
	l1_dirty = false;
	sctlr &= ~SCTLR_C_Msk;
}

static void model_enable(void)
{
	zassert_equal(control, 1U, "L1 restored before L2 enabled");
	sctlr |= SCTLR_C_Msk;
}

/* Keep the real SoC instance, including ATS1CPR tests, linked alongside this one. */
#define sys_read32                       model_read
#define sys_write32                      model_write
#define __get_SCTLR()                    sctlr
#define arch_dcache_disable              model_disable
#define arch_dcache_enable               model_enable
#define zynq_pl310_base                  model_base
#define zynq_pl310_init                  model_init
#define outer_cache_clean_range          model_clean_range
#define outer_cache_invd_range           model_invd_range
#define outer_cache_flush_and_invd_range model_flush_and_invd_range
#define outer_cache_clean_all            model_clean_all
#define outer_cache_invd_all             model_invd_all
#define outer_cache_flush_and_invd_all   model_flush_and_invd_all
#include "../../../../../soc/xlnx/zynq7000/common/cache_pl310.c"

static void reset_model(bool l1_enabled)
{
	control = 1U;
	sctlr = l1_enabled ? SCTLR_C_Msk : 0U;
	l1_dirty = l1_enabled;
	l2_dirty = true;
	drained = false;
	busy_reg = 0U;
	busy_reads = 0U;
	reconfigurations = 0U;
	debug_writes = 0U;
	stuck = false;
}

ZTEST(pl310_sequence, test_warm_init_preserves_dirty_data_and_l1_state)
{
	for (uint32_t enabled = 0U; enabled < 2U; enabled++) {
		reset_model(enabled != 0U);
		model_init(ZYNQ_PL310_BASE);
		zassert_true(drained, "warm init did not drain dirty bootloader data");
		zassert_equal(control, 1U);
		zassert_equal(reconfigurations, 3U);
		zassert_equal(sctlr & SCTLR_C_Msk, enabled != 0U ? SCTLR_C_Msk : 0U);

		/* Re-entry must obey the same handoff, without a software-only guard. */
		l2_dirty = true;
		drained = false;
		model_init(ZYNQ_PL310_BASE);
		zassert_true(drained);
		zassert_equal(reconfigurations, 6U);
	}
}

ZTEST(pl310_sequence, test_warm_init_timeout_keeps_dirty_ways_enabled)
{
	reset_model(true);
	stuck = true;
	model_init(ZYNQ_PL310_BASE);
	zassert_false(drained);
	zassert_true(l2_dirty);
	zassert_equal(control, 1U);
	zassert_equal(sctlr & SCTLR_C_Msk, SCTLR_C_Msk);
	zassert_equal(reconfigurations, 0U);
}

ZTEST(pl310_sequence, test_r3p2_maintenance_does_not_write_debug_control)
{
	static uint8_t line[32] __aligned(32);

	reset_model(false);
	model_init(ZYNQ_PL310_BASE);
	drained = false;
	model_clean_range(line, sizeof(line));
	model_invd_range(line, sizeof(line));
	model_flush_and_invd_range(line, sizeof(line));
	model_clean_all();
	model_invd_all();
	model_flush_and_invd_all();
	zassert_true(drained, "maintenance still requires CACHE_SYNC completion");
	zassert_equal(debug_writes, 0U, "r3p2 needs no debug-register workaround");
}

ZTEST_SUITE(pl310_sequence, NULL, NULL, NULL, NULL, NULL);
