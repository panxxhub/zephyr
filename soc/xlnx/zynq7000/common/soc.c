/*
 * Copyright (c) 2021 Weidmueller Interface GmbH & Co. KG
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/cache.h>
#include <string.h>

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/irq.h>

#include <cmsis_core.h>
#include <zephyr/arch/arm/mmu/arm_mmu.h>
#include "soc.h"

/* System Level Control Registers (SLCR) */
#define SLCR_UNLOCK     0x0008
#define SLCR_UNLOCK_KEY 0xdf0d
#define SLCR_PSS_RST_CTRL 0x0200
#define SLCR_PSS_RST_CTRL_SOFT_RST BIT(0)
#define SLCR_L2C_RAM    0x0A1C
/* Xilinx boot.S value, CR #697094 */
#define SLCR_L2C_RAM_CONFIG 0x00020202

/* Zynq-7000 MPCore SCU */
#define ZYNQ_SCU_BASE		0xF8F00000U
#define ZYNQ_SCU_CTRL		(ZYNQ_SCU_BASE + 0x0000U)
#define ZYNQ_SCU_INV_ALL	(ZYNQ_SCU_BASE + 0x000CU)
#define ZYNQ_SCU_CTRL_ENABLE	BIT(0)
#define AXI_GPIO_MMU_ENTRY(id)\
	MMU_REGION_FLAT_ENTRY("axigpio",\
			      DT_REG_ADDR(id),\
			      DT_REG_SIZE(id),\
			      MT_DEVICE | MATTR_SHARED | MPERM_R | MPERM_W),

static const struct arm_mmu_region mmu_regions[] = {

	MMU_REGION_FLAT_ENTRY("vectors",
			      0x00000000,
			      0x1000,
			      MT_STRONGLY_ORDERED | MPERM_R | MPERM_X),
	MMU_REGION_FLAT_ENTRY("mpcore",
			      0xF8F00000,
			      0x2000,
			      MT_STRONGLY_ORDERED | MPERM_R | MPERM_W),
	/* The mpcore entry above ends exactly at the PL310 register window. */
	MMU_REGION_FLAT_ENTRY("l2cc",
			      ZYNQ_PL310_BASE,
			      0x1000,
			      MT_STRONGLY_ORDERED | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("ocm",
			      DT_REG_ADDR(DT_CHOSEN(zephyr_ocm)),
			      DT_REG_SIZE(DT_CHOSEN(zephyr_ocm)),
			      MT_NORMAL | MATTR_SHARED |
				      MATTR_CACHE_OUTER_WB_WA |
				      MATTR_CACHE_INNER_WB_WA |
				      MPERM_R | MPERM_W),
	/* ARM Arch timer, GIC are covered by the MPCore mapping */

DT_FOREACH_STATUS_OKAY(xlnx_xps_gpio_1_00_a, AXI_GPIO_MMU_ENTRY)

	/*
	 * AXI DMA buffers — identity-mapped as L1 section entries (1 MiB each).
	 * This avoids consuming one L2 page table per MiB, which would exhaust
	 * CONFIG_ARM_MMU_NUM_L2_TABLES for large (>25 MiB) buffers.
	 * Addresses and sizes come from the DTS axi_dma0 node reg-names.
	 *
	 * TX stays strongly ordered: the CPU writes it and the engine reads it,
	 * so the stores must reach DDR with no cache maintenance at all.
	 *
	 * RX is Normal, inner write-back: the consumer copies whole windows out
	 * of it, and a strongly ordered load is a single uncached DDR access
	 * per word (measured 130 ns per 4 bytes, 265 us for a 2000-byte
	 * window).  The driver invalidates each window before handing it to the
	 * consumer.  Only the inner (L1) level is cacheable, because the cache
	 * API on this SoC maintains L1 only; the outer level (PL310) is left
	 * out of the picture rather than depending on it staying disabled.
	 * Nothing in the system writes into the RX region, so no line in it is
	 * ever dirty — that is what makes an invalidate of a window whose end
	 * shares a cache line with the next window safe.
	 */
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(axi_dma0))
	MMU_REGION_FLAT_ENTRY("dma_tx_buf",
			      DT_REG_ADDR_BY_NAME(DT_NODELABEL(axi_dma0), tx_buf),
			      DT_REG_SIZE_BY_NAME(DT_NODELABEL(axi_dma0), tx_buf),
			      MT_STRONGLY_ORDERED | MPERM_R | MPERM_W |
				      MATTR_MAY_MAP_L1_SECTION),
	MMU_REGION_FLAT_ENTRY("dma_rx_buf",
			      DT_REG_ADDR_BY_NAME(DT_NODELABEL(axi_dma0), rx_buf),
			      DT_REG_SIZE_BY_NAME(DT_NODELABEL(axi_dma0), rx_buf),
			      MT_NORMAL | MATTR_SHARED | MATTR_CACHE_INNER_WB_nWA |
				      MPERM_R | MPERM_W | MATTR_MAY_MAP_L1_SECTION),
#endif

};

const struct arm_mmu_config mmu_config = {
	.num_regions = ARRAY_SIZE(mmu_regions),
	.mmu_regions = mmu_regions,
};

#ifdef CONFIG_SMP
static void zynq_scu_enable(void)
{
	uint32_t scu_ctrl;

	/*
	 * JTAG boots bypass the FSBL, so Zephyr must invalidate the duplicate
	 * tags and enable the SCU before SMP state becomes cacheable.
	 */
	sys_write32(0x0000FFFFU, ZYNQ_SCU_INV_ALL);
	barrier_dsync_fence_full();

	scu_ctrl = sys_read32(ZYNQ_SCU_CTRL);
	if ((scu_ctrl & ZYNQ_SCU_CTRL_ENABLE) == 0U) {
		sys_write32(scu_ctrl | ZYNQ_SCU_CTRL_ENABLE, ZYNQ_SCU_CTRL);
		barrier_dsync_fence_full();
		barrier_isync_fence_full();
	}
}

static void zynq_enable_smp_mode(void)
{
	uint32_t actlr = __get_ACTLR();

	if ((actlr & ACTLR_SMP_Msk) == 0U) {
		__set_ACTLR(actlr | ACTLR_SMP_Msk);
		barrier_dsync_fence_full();
		barrier_isync_fence_full();
	}
}
#endif /* CONFIG_SMP */

/*
 * Copy .ocm_data from its ROM load address (in DDR) to OCM.
 * On Zynq-7000 there is no XIP — the bootloader/JTAG loads the entire
 * image into DDR — so the xip.c copy path never runs.  We do the copy
 * here in soc_early_init_hook(), which fires after the MMU is up but
 * before any Zephyr subsystem init that might touch OCM data.
 */
void soc_early_init_hook(void)
{
#ifdef CONFIG_SOC_XLNX_ZYNQ7000_L2_CACHE
	/*
	 * The MMU and both L1 caches are on by now (z_arm_mmu_init) and the
	 * secondary core has not been released yet (z_smp_init runs much
	 * later), so this is the one place where the outer cache can be
	 * brought up with the system quiescent and every later access, on
	 * either core, already covered by it.
	 */
	zynq_pl310_init(ZYNQ_PL310_BASE);
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_CHOSEN(zephyr_ocm))
	memcpy(&__ocm_data_start, &__ocm_data_load_start,
	       __ocm_data_end - __ocm_data_start);
#endif
}

#ifdef CONFIG_SMP
/*
 * The secondary core reaches this hook with the MMU and L1 caches already on
 * (arch_secondary_cpu_init) but ACTLR.SMP still clear: soc_reset_hook only
 * runs on the boot path of the primary core. With SMP clear a Cortex-A9
 * treats every Shareable Normal access as non-cacheable, so the whole kernel
 * RAM is uncached for that core (silicon 2026-09-10: 25 CPU cycles per byte
 * for a memset on CPU 1). Switch the data cache off, set SMP, switch it on.
 */
void soc_per_core_init_hook(void)
{
	if ((__get_ACTLR() & ACTLR_SMP_Msk) == 0U) {
		sys_cache_data_disable();
		zynq_enable_smp_mode();
		sys_cache_data_enable();
	}

	/*
	 * Program-flow prediction (SCTLR.Z) resets to off and nothing in the
	 * boot path turns it on, so every taken branch flushes the pipeline:
	 * silicon 2026-09-10 measured 14 CPU cycles per load in a cache-hit
	 * loop and 68 us for a 2 KiB memset on both cores.
	 */
	uint32_t sctlr = __get_SCTLR();

	if ((sctlr & SCTLR_Z_Msk) == 0U) {
		__set_SCTLR(sctlr | SCTLR_Z_Msk);
		barrier_isync_fence_full();
	}
}
#endif /* CONFIG_SMP */

/* Platform-specific early initialization */

void soc_reset_hook(void)
{
	/*
	 * When coming out of u-boot rather than downloading the Zephyr binary
	 * via JTAG, a few things modified by u-boot have to be re-set to a
	 * suitable default value for Zephyr to run, namely:
	 *
	 * - u-boot places the exception vectors somewhere in RAM and then
	 *   lets the VBAR register point to them. Zephyr uses the default
	 *   vector table location at address zero (and maybe at some later
	 *   time alternatively the HIVECS position). If VBAR isn't reset
	 *   to zero, the system crashes during the first context switch when
	 *   SVC is invoked.
	 * - u-boot sets the following bits in the SCTLR register:
	 *   - [I] ICache enable
	 *   - [C] DCache enable
	 *   - [Z] Branch prediction enable
	 *   - [A] Enforce strict alignment enable
	 *   [I] and [C] will be enabled during the MMU init -> disable them
	 *   until then. [Z] is probably not harmful. [A] will cause a crash
	 *   as early as z_mem_manage_init when an unaligned access is performed
	 *   -> clear [A].
	 */

	uint32_t vbar = 0;

	__set_VBAR(vbar);

	uint32_t sctlr = __get_SCTLR();

	sctlr &= ~SCTLR_I_Msk;
	sctlr &= ~SCTLR_C_Msk;
	sctlr &= ~SCTLR_A_Msk;
	__set_SCTLR(sctlr);

#ifdef CONFIG_SMP
	if (MPIDR_TO_CORE(GET_MPIDR()) == 0U) {
		zynq_scu_enable();
	}

	zynq_enable_smp_mode();
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(slcr))
	mm_reg_t addr = DT_REG_ADDR(DT_NODELABEL(slcr));

	/* Unlock System Level Control Registers (SLCR) */
	sys_write32(SLCR_UNLOCK_KEY, addr + SLCR_UNLOCK);

#ifdef CONFIG_SOC_XLNX_ZYNQ7000_L2_CACHE
	/*
	 * L2 cache RAM timing.  Written here, where the SLCR is still
	 * reachable by physical address with the MMU off, and before the
	 * controller is enabled in soc_early_init_hook().
	 */
	sys_write32(SLCR_L2C_RAM_CONFIG, addr + SLCR_L2C_RAM);
#endif
#endif
}

#if defined(CONFIG_REBOOT) && DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(slcr))
/* PS-only software reset (UG585 PSS_RST_CTRL.SOFT_RST): reruns BootROM, so a
 * QSPI-booted image comes back through FSBL; a JTAG-loaded image is gone.
 * The MMU is on here, so map the SLCR page instead of using the physical
 * address the early init hook could still use.
 */
void sys_arch_reboot(int type)
{
	mm_reg_t slcr;

	ARG_UNUSED(type);

	device_map(&slcr, DT_REG_ADDR(DT_NODELABEL(slcr)), 0x1000, K_MEM_CACHE_NONE);
	(void)irq_lock();
	/* The reset discards whatever the caches still hold, so anything a
	 * caller wrote for the next boot to read has to be in memory first.
	 */
	sys_cache_data_flush_all();
	sys_write32(SLCR_UNLOCK_KEY, slcr + SLCR_UNLOCK);
	barrier_dsync_fence_full();
	sys_write32(SLCR_PSS_RST_CTRL_SOFT_RST, slcr + SLCR_PSS_RST_CTRL);
	barrier_dsync_fence_full();
	for (;;) {
		barrier_isync_fence_full();
	}
}
#endif
