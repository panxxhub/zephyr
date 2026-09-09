// SPDX-License-Identifier: Apache-2.0
#include <zephyr/ztest.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/interrupt_controller/gic.h>
#include <zephyr/drivers/dma/dma_xlnx_axi_dma_sg.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/cache.h>
#include <zephyr/sys/device_mmio.h>

#define CONFIG_DMA_XLNX_AXI_DMA_SG_NUM_RX_BD 8
#define CONFIG_DMA_XLNX_AXI_DMA_SG_IRQ_THRESHOLD 1
#define CONFIG_DMA_XLNX_AXI_DMA_SG_IRQ_TIMEOUT 16

static uint32_t regs[32];
static unsigned int resets;
static bool reset_stuck;

static int test_cache_range(void *addr, size_t size)
{
	ARG_UNUSED(addr);
	ARG_UNUSED(size);
	return 0;
}

static uint32_t test_read32(mem_addr_t addr)
{
	return regs[addr / 4U];
}

static void test_write32(uint32_t value, mem_addr_t addr)
{
	regs[addr / 4U] = value;
	if (addr == 0x30U && (value & BIT(2)) != 0U) {
		resets++;
		if (!reset_stuck) {
			regs[0x30U / 4U] = 0;
			regs[0x34U / 4U] = BIT(0) | BIT(3);
		}
	}
}

#undef sys_read32
#define sys_read32 test_read32
#undef sys_write32
#define sys_write32 test_write32
#undef DEVICE_MMIO_NAMED_GET
#define DEVICE_MMIO_NAMED_GET(dev, name) 0U
/* Exercise production ring/state logic without hardware or cache access. */
#define sys_cache_data_flush_range test_cache_range
#define sys_cache_data_invd_range test_cache_range
#include "../../../../../drivers/dma/dma_xlnx_axi_dma_sg.c"

static struct xlnx_sg_bd bds[8];
static struct dma_xlnx_sg_data data;
static const struct dma_xlnx_sg_cfg config = {
	.rx_buf_phys = 0x10000000,
	.rx_buf_size = 4096,
	.sg_len_mask = 0x3fff,
};
static const struct device dev = {.data = &data, .config = &config};

static int configure_rx(void)
{
	struct dma_block_config block = {
		.block_size = 512, .dest_scatter_count = 8,
	};
	struct dma_config cfg = {
		.channel_direction = PERIPHERAL_TO_MEMORY,
		.head_block = &block,
	};

	return dma_xlnx_sg_config(&dev, CH_RX, &cfg);
}

static void before(void *fixture)
{
	ARG_UNUSED(fixture);
	/* Normally referenced by DT device/IRQ instantiation, absent here. */
	(void)dma_xlnx_sg_init;
	(void)dma_xlnx_sg_rx_isr;
	(void)dma_xlnx_sg_tx_isr;
	memset(&data, 0, sizeof(data));
	memset(regs, 0, sizeof(regs));
	memset(bds, 0xff, sizeof(bds));
	data.ch[CH_RX].bds = bds;
	data.ch[CH_RX].num_bds = ARRAY_SIZE(bds);
	resets = 0;
	reset_stuck = false;
}

ZTEST(xlnx_finite_rx, test_halted_sg_error_is_reset_before_reusing_descriptors)
{
	regs[0x34U / 4U] = 0x14109;
	data.ch[CH_RX].error = true;
	zassert_ok(configure_rx());
	zassert_equal(resets, 1, "Halted SG error requires reset");
	zassert_equal(regs[0x34U / 4U] & DMASR_ALL_ERR, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(bds); i++) {
		zassert_equal(bds[i].status, 0,
			      "never submit a completed descriptor");
	}
}

ZTEST(xlnx_finite_rx, test_reset_timeout_does_not_rebuild_owned_descriptors)
{
	regs[0x34U / 4U] = BIT(3); /* Running, not halted. */
	reset_stuck = true;
	zassert_equal(configure_rx(), -EIO);
	zassert_equal(bds[0].status, UINT32_MAX,
		      "failed reset must not rewrite owned BDs");
}

ZTEST(xlnx_finite_rx, test_finite_ring_geometry_and_hardware_irq_threshold)
{
	regs[0x34U / 4U] = BIT(0) | BIT(3);
	zassert_ok(configure_rx());
	zassert_equal(data.ch[CH_RX].active_bds, 8);
	zassert_equal(data.ch[CH_RX].hw_irq_threshold, 8);
	for (unsigned int i = 0; i < ARRAY_SIZE(bds); i++) {
		zassert_equal((uintptr_t)&bds[i] & 63U, 0);
		zassert_equal(bds[i].next_desc,
			      (uint32_t)(uintptr_t)&bds[(i + 1) % 8]);
		zassert_equal(bds[i].buf_addr, 0x10000000U + 512U * i);
		zassert_equal(bds[i].control, 512);
		zassert_equal(bds[i].status, 0);
	}
	zassert_ok(dma_xlnx_sg_start(&dev, CH_RX));
	zassert_equal(regs[0x38U / 4U], (uint32_t)(uintptr_t)&bds[0]);
	zassert_equal(regs[0x40U / 4U], (uint32_t)(uintptr_t)&bds[7]);
	zassert_equal((regs[0x30U / 4U] & DMACR_IRQTHRESH_MASK) >>
		      DMACR_IRQTHRESH_SHIFT, 8);
}

ZTEST_SUITE(xlnx_finite_rx, NULL, NULL, before, NULL, NULL);
