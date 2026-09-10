/*
 * Copyright (c) 2026 Opus One
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ARM PL310 (L2C-310) outer cache for Zynq-7000.
 *
 * 512 KiB, 8 ways of 64 KiB, 32-byte lines, shared by both Cortex-A9 cores and
 * sitting between them and the DDR controller.  PL masters on the AXI HP ports
 * do not go through it, so a buffer shared with the PL needs the same explicit
 * maintenance the L1 always needed -- which is why this is wired into the
 * sys_cache_data_* path rather than exposed as its own API.
 *
 * Maintenance is by physical address; kernel RAM is identity mapped but a
 * device_map()'d region is not, so every page is translated with the CP15
 * address translation operation (ATS1CPR) before its lines are queued.
 *
 * Errata for the r3p2 cut used on Zynq-7000 (UG585):
 *  - 588369 (r1p0/r2p0) and 727915 (r2p0..r3p0) predate this cut.
 *  - 753970 (sync register at 0x740) is r3p0 only; the Xilinx BSP removed its
 *    workaround for Zynq under CR#989132, see vendor xil_errata.h.
 *  - 769419 (the store buffer is not drained automatically) applies: every
 *    maintenance operation below ends in an explicit cache sync.
 */

#include <zephyr/kernel.h>
#include <zephyr/arch/arm/cortex_a_r/outer_cache.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include "soc.h"

/* Register offsets, ARM DDI 0246 */
#define PL310_CTRL             0x100U
#define PL310_AUX_CTRL         0x104U
#define PL310_TAG_RAM_CTRL     0x108U
#define PL310_DATA_RAM_CTRL    0x10CU
#define PL310_INT_RAW_STATUS   0x21CU
#define PL310_INT_CLEAR        0x220U
#define PL310_CACHE_SYNC       0x730U
#define PL310_INV_PA           0x770U
#define PL310_INV_WAY          0x77CU
#define PL310_CLEAN_PA         0x7B0U
#define PL310_CLEAN_WAY        0x7BCU
#define PL310_CLEAN_INV_PA     0x7F0U
#define PL310_CLEAN_INV_WAY    0x7FCU

#define PL310_CTRL_ENABLE      BIT(0)

/*
 * Xilinx boot.S values.  Aux control 0x72360000 decodes as: early BRESP,
 * instruction and data prefetch, round-robin replacement, parity and event
 * monitor bus on, way size field 3 (64 KiB) and associativity 0 (8 ways),
 * i.e. the 512 KiB the Zynq-7000 carries.  Tag/data RAM latencies match the
 * arm,tag-latency = <2 2 2> / arm,data-latency = <3 2 2> of the Linux
 * zynq-7000 device tree.
 */
#define PL310_AUX_CTRL_INIT    0x72360000U
#define PL310_TAG_RAM_INIT     0x00000111U
#define PL310_DATA_RAM_INIT    0x00000121U
#define PL310_WAY_MASK         0x0000FFFFU

#define PL310_LINE_SIZE        32U

/*
 * Poll bound for a background by-way operation.  The longest such operation
 * is a clean of 512 KiB of dirty lines, roughly a millisecond of DDR write
 * back; every iteration below is a strongly ordered read of the controller,
 * tens of cycles each, so this bound is about two orders of magnitude of
 * headroom -- giving up early would silently drop dirty data, so the margin
 * leans that way.  It is a count and not a clock reading on purpose: the
 * reboot path runs with interrupts locked and the system timer already
 * stopped, and a reset must never be lost to a cache operation that will
 * not finish.
 */
#define PL310_WAY_OP_POLL_MAX  1000000U

/* Base of the controller's register window.  A test points this at a fake. */
uintptr_t zynq_pl310_base = ZYNQ_PL310_BASE;

static ALWAYS_INLINE uint32_t pl310_read(uint32_t off)
{
	return sys_read32(zynq_pl310_base + off);
}

static ALWAYS_INLINE void pl310_write(uint32_t off, uint32_t val)
{
	sys_write32(val, zynq_pl310_base + off);
}

/*
 * Drain the store buffer.  This is the explicit sync erratum 769419 asks for
 * after every maintenance operation.  It must not be issued while a
 * background by-way operation is still running -- the controller stalls the
 * write until that operation retires -- so every caller waits for the way
 * register first.
 */
static void pl310_sync(void)
{
	pl310_write(PL310_CACHE_SYNC, 0U);

	for (uint32_t i = 0U; i < PL310_WAY_OP_POLL_MAX; i++) {
		if (pl310_read(PL310_CACHE_SYNC) == 0U) {
			break;
		}
	}

	barrier_dsync_fence_full();
}

/* CP15 ATS1CPR: translate a virtual address as a PL1 read in the current
 * security state, then read the result out of PAR.
 */
static uintptr_t pl310_va_to_pa(uintptr_t va)
{
	unsigned int key = irq_lock();
	uint32_t par;

	__asm__ volatile("mcr p15, 0, %0, c7, c8, 0" : : "r"(va) : "memory");
	__asm__ volatile("isb");
	__asm__ volatile("mrc p15, 0, %0, c7, c4, 0" : "=r"(par));

	irq_unlock(key);

	return (uintptr_t)(par & 0xFFFFF000U) | (va & 0xFFFU);
}

/*
 * Queue one operation for every line of [start, end), translating once per
 * 4 KiB page.  Both bounds are line aligned by the caller.
 */
static void pl310_op_range(uint32_t reg, uintptr_t start, uintptr_t end)
{
	while (start < end) {
		uintptr_t chunk_end = MIN(end, ROUND_UP(start + 1U, KB(4)));
		uintptr_t pa = pl310_va_to_pa(start);

		while (start < chunk_end) {
			pl310_write(reg, (uint32_t)pa);
			start += PL310_LINE_SIZE;
			pa += PL310_LINE_SIZE;
		}
	}
}

/*
 * A by-way operation is a background operation: the controller clears the bit
 * of every way as that way retires, and the operation is complete when the
 * register itself reads zero.  The cache sync register is not that indicator
 * -- it can read zero while the background operation is still in flight, and
 * writing it in that window stalls -- so poll the way register, and give up
 * after a bounded number of tries rather than spin forever.
 */
static bool pl310_wait_way(uint32_t reg)
{
	for (uint32_t i = 0U; i < PL310_WAY_OP_POLL_MAX; i++) {
		if ((pl310_read(reg) & PL310_WAY_MASK) == 0U) {
			return true;
		}
	}

	return false;
}

static void pl310_op_way(uint32_t reg)
{
	pl310_write(reg, PL310_WAY_MASK);

	if (pl310_wait_way(reg)) {
		pl310_sync();
	}
}

void outer_cache_clean_range(void *addr, size_t size)
{
	uintptr_t start = ROUND_DOWN((uintptr_t)addr, PL310_LINE_SIZE);
	uintptr_t end = ROUND_UP((uintptr_t)addr + size, PL310_LINE_SIZE);

	pl310_op_range(PL310_CLEAN_PA, start, end);
	pl310_sync();
}

void outer_cache_invd_range(void *addr, size_t size)
{
	uintptr_t start = (uintptr_t)addr;
	uintptr_t end = start + size;

	/*
	 * A line the range only partly covers holds bytes that belong to
	 * somebody else, so clean and invalidate those two rather than
	 * dropping them.
	 */
	if (start & (PL310_LINE_SIZE - 1U)) {
		uintptr_t head = ROUND_DOWN(start, PL310_LINE_SIZE);

		pl310_op_range(PL310_CLEAN_INV_PA, head, head + PL310_LINE_SIZE);
		start = head + PL310_LINE_SIZE;
	}

	if ((end & (PL310_LINE_SIZE - 1U)) && start < end) {
		uintptr_t tail = ROUND_DOWN(end, PL310_LINE_SIZE);

		pl310_op_range(PL310_CLEAN_INV_PA, tail, tail + PL310_LINE_SIZE);
		end = tail;
	}

	if (start < end) {
		pl310_op_range(PL310_INV_PA, start, end);
	}

	pl310_sync();
}

void outer_cache_flush_and_invd_range(void *addr, size_t size)
{
	uintptr_t start = ROUND_DOWN((uintptr_t)addr, PL310_LINE_SIZE);
	uintptr_t end = ROUND_UP((uintptr_t)addr + size, PL310_LINE_SIZE);

	pl310_op_range(PL310_CLEAN_INV_PA, start, end);
	pl310_sync();
}

void outer_cache_clean_all(void)
{
	pl310_op_way(PL310_CLEAN_WAY);
}

void outer_cache_invd_all(void)
{
	pl310_op_way(PL310_INV_WAY);
}

void outer_cache_flush_and_invd_all(void)
{
	pl310_op_way(PL310_CLEAN_INV_WAY);
}

/*
 * Bring the controller up, in the order the Xilinx boot code uses.  The SLCR
 * L2C RAM configuration that goes with it is written in soc_reset_hook(),
 * while the SLCR is still reachable by physical address.
 */
void zynq_pl310_init(uintptr_t base)
{
	zynq_pl310_base = base;

	/* The FSBL hands over with the cache clean and off; a JTAG boot never
	 * turned it on.  Writing the configuration below needs it off either
	 * way.
	 */
	pl310_write(PL310_CTRL, 0U);

	pl310_write(PL310_AUX_CTRL, pl310_read(PL310_AUX_CTRL) | PL310_AUX_CTRL_INIT);
	pl310_write(PL310_TAG_RAM_CTRL, PL310_TAG_RAM_INIT);
	pl310_write(PL310_DATA_RAM_CTRL, PL310_DATA_RAM_INIT);

	pl310_write(PL310_INV_WAY, PL310_WAY_MASK);
	(void)pl310_wait_way(PL310_INV_WAY);

	pl310_write(PL310_INT_CLEAR, pl310_read(PL310_INT_RAW_STATUS));

	barrier_dsync_fence_full();
	pl310_write(PL310_CTRL, pl310_read(PL310_CTRL) | PL310_CTRL_ENABLE);
	barrier_dsync_fence_full();
	barrier_isync_fence_full();
}
