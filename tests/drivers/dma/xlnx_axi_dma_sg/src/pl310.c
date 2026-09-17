/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors */
/* SPDX-License-Identifier: Apache-2.0 */
#include <zephyr/ztest.h>
#include <zephyr/arch/cache.h>
#include <zephyr/drivers/cache.h>
#include <zephyr/drivers/cache/xlnx_zynq7000_pl310.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/sys_io.h>

static uint32_t outer_writes;

static uint32_t probe_read(mem_addr_t addr)
{
	ARG_UNUSED(addr);
	return 0U;
}

static void probe_write(uint32_t value, mem_addr_t addr)
{
	ARG_UNUSED(value);
	ARG_UNUSED(addr);
	outer_writes++;
}

static int probe_translate(uintptr_t va, uintptr_t *pa)
{
	*pa = va & UINT32_MAX;
	return 0;
}

static int probe_l1_range(void *addr, size_t size)
{
	ARG_UNUSED(addr);
	ARG_UNUSED(size);
	return 0;
}

static void probe_l1_enable(void)
{
}

static int probe_l1_all(void)
{
	return 0;
}

#define arch_dcache_disable            probe_l1_enable
#define arch_dcache_enable             probe_l1_enable
#define arch_dcache_flush_and_invd_all probe_l1_all
#define arch_dcache_flush_all          probe_l1_all
#define arch_dcache_invd_all           probe_l1_all
#undef sys_read32
#undef sys_write32
#define sys_read32                       probe_read
#define sys_write32                      probe_write
#define zynq_pl310_virt_to_phys          probe_translate
#define arch_dcache_invd_range           probe_l1_range
#define arch_dcache_flush_range          probe_l1_range
#define zynq_pl310_init                  probe_zynq_pl310_init
#define zynq_pl310_shutdown              probe_zynq_pl310_shutdown
#define cache_data_flush_range           probe_cache_data_flush_range
#define cache_data_invd_range            probe_cache_data_invd_range
#define cache_data_flush_and_invd_range  probe_cache_data_flush_and_invd_range
#define cache_data_flush_all             probe_cache_data_flush_all
#define cache_data_invd_all              probe_cache_data_invd_all
#define cache_data_flush_and_invd_all    probe_cache_data_flush_and_invd_all
#define cache_data_disable               probe_cache_data_disable
#define cache_data_enable                probe_cache_data_enable
#define cache_data_line_size_get         probe_cache_data_line_size_get
#define cache_instr_enable               probe_cache_instr_enable
#define cache_instr_disable              probe_cache_instr_disable
#define cache_instr_flush_all            probe_cache_instr_flush_all
#define cache_instr_invd_all             probe_cache_instr_invd_all
#define cache_instr_flush_and_invd_all   probe_cache_instr_flush_and_invd_all
#define cache_instr_flush_range          probe_cache_instr_flush_range
#define cache_instr_invd_range           probe_cache_instr_invd_range
#define cache_instr_flush_and_invd_range probe_cache_instr_flush_and_invd_range
#define cache_instr_line_size_get        probe_cache_instr_line_size_get
#include "../../../../../drivers/cache/cache_xlnx_zynq7000_pl310.c"

/* Route the DMA fixture's generic cache call through the real PL310 driver. */
int dma_test_outer_invalidate(void *addr, size_t size)
{
	pl310_enabled = true;
	return probe_cache_data_invd_range(addr, size);
}

uint32_t dma_test_outer_writes(void)
{
	uint32_t count = outer_writes;

	outer_writes = 0U;
	return count;
}
