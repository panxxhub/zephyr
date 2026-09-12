/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/arch/cache.h>
#include <zephyr/drivers/cache.h>
#include <zephyr/drivers/cache/xlnx_zynq7000_pl310.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/sys_io.h>

#define PL310_BASE      0xf8f02000U
#define PL310_CTRL      0x100U
#define PL310_AUX       0x104U
#define PL310_TAG       0x108U
#define PL310_DATA      0x10cU
#define PL310_INT_MASK  0x214U
#define PL310_INT_CLEAR 0x220U
#define PL310_SYNC      0x730U
#define PL310_INV_PA    0x770U
#define PL310_INV_WAY   0x77cU
#define PL310_CLEAN_PA  0x7b0U
#define PL310_CLEAN_WAY 0x7bcU
#define PL310_CINV_PA   0x7f0U
#define PL310_CINV_WAY  0x7fcU
#define PL310_ALL_WAYS  0xffffU
#define PL310_LINE      32U
#define PL310_PAGE      4096U
#define SLCR_UNLOCK    0xf8000008U
#define SLCR_L2C_RAM   0xf8000a1cU

static struct k_spinlock pl310_lock;
static bool pl310_enabled;

static void pl310_sync(void)
{
	sys_write32(0U, PL310_BASE + PL310_SYNC);
	while ((sys_read32(PL310_BASE + PL310_SYNC) & 1U) != 0U) {
	}
	barrier_dsync_fence_full();
}

static void pl310_way(uint32_t reg)
{
	sys_write32(PL310_ALL_WAYS, PL310_BASE + reg);
	while ((sys_read32(PL310_BASE + reg) & PL310_ALL_WAYS) != 0U) {
	}
	pl310_sync();
}

void zynq_pl310_init(uint32_t cpu)
{
	bool restore_l1;

	if (cpu != 0U || pl310_enabled) {
		return;
	}

	/* Also tolerate a bootloader which left dirty data in an enabled L2. */
	restore_l1 = (sys_read32(PL310_BASE + PL310_CTRL) & 1U) != 0U;
	if (restore_l1) {
		arch_dcache_disable();
		pl310_way(PL310_CINV_WAY);
	}
	sys_write32(0U, PL310_BASE + PL310_CTRL);
	barrier_dsync_fence_full();
	sys_write32(0xdf0dU, SLCR_UNLOCK);
	sys_write32(0x00020202U, SLCR_L2C_RAM);
	sys_write32(0x72360000U, PL310_BASE + PL310_AUX);
	sys_write32(0x0111U, PL310_BASE + PL310_TAG);
	sys_write32(0x0121U, PL310_BASE + PL310_DATA);
	pl310_way(PL310_INV_WAY);
	sys_write32(0U, PL310_BASE + PL310_INT_MASK);
	sys_write32(0x1ffU, PL310_BASE + PL310_INT_CLEAR);
	sys_write32(1U, PL310_BASE + PL310_CTRL);
	pl310_sync();
	pl310_enabled = true;
	barrier_dsync_fence_full();
	if (restore_l1) {
		arch_dcache_enable();
	}
}

/*
 * r3p2 fixes 727915, 753970 and 769419. PA operations are atomic; way
 * operations run in the background. Serialize both and drain with CACHE_SYNC.
 */
static int pl310_range(uintptr_t start, size_t size, uint32_t op)
{
	uintptr_t last = ROUND_DOWN(start + size - 1U, PL310_LINE);
	uintptr_t line = ROUND_DOWN(start, PL310_LINE);
	uintptr_t page = 0U;
	uintptr_t pa = 0U;
	bool translated = false;
	int ret;

	if (!pl310_enabled) {
		return 0;
	}

	for (;;) {
		uintptr_t next_page = ROUND_DOWN(line, PL310_PAGE);
		uint32_t command = op;

		if (!translated || page != next_page) {
			ret = zynq_pl310_virt_to_phys(next_page, &pa);
			if (ret != 0 || pa > UINT32_MAX) {
				pl310_sync();
				return ret != 0 ? ret : -EFAULT;
			}
			page = next_page;
			translated = true;
		}
		if (op == PL310_INV_PA &&
		    ((line < start) || (line == last && ((start + size) % PL310_LINE) != 0U))) {
			command = PL310_CINV_PA;
		}
		sys_write32((uint32_t)(pa + line - page), PL310_BASE + command);
		if (line == last) {
			break;
		}
		line += PL310_LINE;
	}
	pl310_sync();
	return 0;
}

static int cache_range(void *addr, size_t size, uint32_t op)
{
	uintptr_t start = (uintptr_t)addr;
	k_spinlock_key_t key;
	int ret = 0;

	if (size == 0U) {
		return 0;
	}
	if (size > UINTPTR_MAX - start) {
		return -EINVAL;
	}

	key = k_spin_lock(&pl310_lock);
	if (op != PL310_INV_PA) {
		ret = arch_dcache_flush_range(addr, size);
	} else {
		/* Preserve dirty bytes outside the range before invalidating the outer line. */
		uintptr_t first = ROUND_DOWN(start, PL310_LINE);
		uintptr_t last = ROUND_DOWN(start + size - 1U, PL310_LINE);

		if (first != start) {
			ret = arch_dcache_flush_range((void *)first, PL310_LINE);
		}
		if (ret == 0 && ((start + size) % PL310_LINE) != 0U &&
		    (last != first || first == start)) {
			ret = arch_dcache_flush_range((void *)last, PL310_LINE);
		}
	}
	if (ret == 0) {
		barrier_dsync_fence_full();
		ret = pl310_range(start, size, op);
	}
	if (ret == 0 && op != PL310_CLEAN_PA) {
		ret = arch_dcache_invd_range(addr, size);
		barrier_dsync_fence_full();
	}
	k_spin_unlock(&pl310_lock, key);
	return ret;
}

int cache_data_flush_range(void *addr, size_t size)
{
	return cache_range(addr, size, PL310_CLEAN_PA);
}

int cache_data_invd_range(void *addr, size_t size)
{
	return cache_range(addr, size, PL310_INV_PA);
}

int cache_data_flush_and_invd_range(void *addr, size_t size)
{
	return cache_range(addr, size, PL310_CINV_PA);
}

static int cache_all(uint32_t op)
{
	k_spinlock_key_t key = k_spin_lock(&pl310_lock);
	int ret = 0;

	if (op == PL310_CINV_WAY) {
		/* Later calls can dirty the stack; never discard L1 after the outer operation. */
		ret = arch_dcache_flush_and_invd_all();
	} else if (op == PL310_CLEAN_WAY) {
		ret = arch_dcache_flush_all();
	}
	if (ret == 0 && pl310_enabled) {
		barrier_dsync_fence_full();
		pl310_way(op);
	}
	if (ret == 0 && op == PL310_INV_WAY) {
		ret = arch_dcache_invd_all();
		barrier_dsync_fence_full();
	}
	k_spin_unlock(&pl310_lock, key);
	return ret;
}

int cache_data_flush_all(void)
{
	return cache_all(PL310_CLEAN_WAY);
}

int cache_data_invd_all(void)
{
	return cache_all(PL310_INV_WAY);
}

int cache_data_flush_and_invd_all(void)
{
	return cache_all(PL310_CINV_WAY);
}

void cache_data_disable(void)
{
	k_spinlock_key_t key = k_spin_lock(&pl310_lock);

	arch_dcache_disable();
	if (pl310_enabled) {
		pl310_way(PL310_CLEAN_WAY);
	}
	/* Runtime local disable keeps shared L2 enabled for other cores. */
	k_spin_unlock(&pl310_lock, key);
}

void zynq_pl310_shutdown(void)
{
	/* The caller has quiesced other cache users and will reset immediately. */
	cache_data_disable();
	sys_write32(0U, PL310_BASE + PL310_CTRL);
	barrier_dsync_fence_full();
	pl310_enabled = false;
}

void cache_data_enable(void)
{
	arch_dcache_enable();
}

size_t cache_data_line_size_get(void)
{
	return PL310_LINE;
}

#ifdef CONFIG_ICACHE
void cache_instr_enable(void)
{
	arch_icache_enable();
}
void cache_instr_disable(void)
{
	arch_icache_disable();
}
int cache_instr_flush_all(void)
{
	return arch_icache_flush_all();
}
int cache_instr_invd_all(void)
{
	return arch_icache_invd_all();
}
int cache_instr_flush_and_invd_all(void)
{
	return arch_icache_flush_and_invd_all();
}
int cache_instr_flush_range(void *addr, size_t size)
{
	return arch_icache_flush_range(addr, size);
}
int cache_instr_invd_range(void *addr, size_t size)
{
	return arch_icache_invd_range(addr, size);
}
int cache_instr_flush_and_invd_range(void *addr, size_t size)
{
	return arch_icache_flush_and_invd_range(addr, size);
}
size_t cache_instr_line_size_get(void)
{
	return PL310_LINE;
}
#endif
