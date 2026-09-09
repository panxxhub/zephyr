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
	} else if (addr == 0x30U && (value & BIT(0)) != 0U) {
		regs[0x34U / 4U] = BIT(3); /* Armed, waiting for stream data. */
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
static unsigned int callbacks;
static int callback_status;

static void completed(const struct device *device, void *user,
		      uint32_t channel, int status)
{
	ARG_UNUSED(device);
	ARG_UNUSED(user);
	ARG_UNUSED(channel);
	callbacks++;
	callback_status = status;
}

/* PG021 tail-pointer model: consume one packet per BD, including tail,
 * then pause without fetching its next pointer. This is not an AXI RTL
 * simulation; it checks the actual driver's submitted descriptor graph.
 */
static int feed_finite_packets(void)
{
	uintptr_t addr = regs[0x38U / 4U];
	uintptr_t tail = regs[0x40U / 4U];

	for (unsigned int i = 0; i <= ARRAY_SIZE(bds); i++) {
		if (addr < (uintptr_t)bds || addr >= (uintptr_t)(bds + 8) ||
		    (addr & 63U) != 0U) {
			return -EFAULT;
		}
		struct xlnx_sg_bd *bd = (struct xlnx_sg_bd *)addr;

		if ((bd->status & BIT(31)) != 0U) {
			return -ESTALE;
		}
		bd->status = BIT(31) | (bd->control & config.sg_len_mask);
		regs[0x38U / 4U] = addr;
		if (addr == tail) {
			regs[0x34U / 4U] = DMASR_IDLE | DMASR_IOC_IRQ;
			dma_xlnx_sg_rx_isr(&dev);
			return i + 1;
		}
		addr = bd->next_desc;
	}
	return -ELOOP;
}

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
	k_work_init(&data.ch[CH_RX].error_work, dma_xlnx_sg_error_work);
	data.ch[CH_RX].bds = bds;
	data.ch[CH_RX].num_bds = ARRAY_SIZE(bds);
	resets = 0;
	reset_stuck = false;
	callbacks = 0;
	callback_status = 0;
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

ZTEST(xlnx_finite_rx, test_first_arm_completes_at_tail)
{
	struct dma_status status;

	regs[0x34U / 4U] = BIT(0) | BIT(3);
	zassert_ok(configure_rx());
	data.ch[CH_RX].callback = completed;
	zassert_ok(dma_xlnx_sg_start(&dev, CH_RX));
	zassert_ok(dma_xlnx_sg_get_status(&dev, CH_RX, &status));
	zassert_true(status.busy);
	zassert_equal(dma_xlnx_sg_last_rx_bytes(&dev), 0);
	zassert_equal(callbacks, 0);
	zassert_equal(feed_finite_packets(), 8);
	zassert_equal(callbacks, 1);
	zassert_equal(callback_status, DMA_STATUS_COMPLETE);
	zassert_equal(dma_xlnx_sg_last_rx_bytes(&dev), 4096);
	/* A subsequent fetch of tail.next would encounter completed BD0;
	 * hardware must pause at tail before following that ring link.
	 */
	zassert_equal(bds[7].next_desc, (uint32_t)(uintptr_t)&bds[0]);
}

ZTEST(xlnx_finite_rx, test_bad_tail_model_detects_refetch_of_completed_head)
{
	regs[0x34U / 4U] = BIT(0) | BIT(3);
	zassert_ok(configure_rx());
	zassert_ok(dma_xlnx_sg_start(&dev, CH_RX));
	regs[0x40U / 4U] = (uint32_t)(uintptr_t)(bds + 8);
	zassert_equal(feed_finite_packets(), -ESTALE);
}

static void reset_ring_on_error(const struct device *device, void *user,
			       uint32_t channel, int status)
{
	completed(device, user, channel, status);
	memset(bds, 0, sizeof(bds));
	memset(regs, 0, sizeof(regs));
}

ZTEST(xlnx_finite_rx, test_error_snapshot_precedes_callback_teardown)
{
	struct k_work_sync sync;

	zassert_ok(configure_rx());
	data.ch[CH_RX].callback = reset_ring_on_error;
	zassert_ok(dma_xlnx_sg_start(&dev, CH_RX));
	bds[0].control = 0x8c000200U;
	regs[0x34U / 4U] = 0x00204109;
	k_sched_lock();
	dma_xlnx_sg_rx_isr(&dev);
	struct xlnx_sg_error_snapshot snap = data.ch[CH_RX].error_snapshot;

	k_sched_unlock();
	(void)k_work_flush(&data.ch[CH_RX].error_work, &sync);
	zassert_equal(callbacks, 1);
	zassert_equal(callback_status, -EIO);
	zassert_equal(snap.sr, 0x00204109);
	zassert_true(snap.arm_valid);
	zassert_equal(snap.arm_control, 512U);
	zassert_equal(snap.arm_status, 0U);
	zassert_equal(snap.cur, (uintptr_t)&bds[0]);
	zassert_equal(snap.tail, (uintptr_t)&bds[7]);
	zassert_equal(snap.head_written, snap.cur);
	zassert_equal(snap.tail_written, snap.tail);
	zassert_equal(snap.first.status, 0U);
	zassert_equal(snap.first.next_desc, (uintptr_t)&bds[1]);
	zassert_equal(snap.first.buf_addr, config.rx_buf_phys);
	zassert_equal(snap.first.control, 0x8c000200U);
	zassert_equal(bds[0].status, 0, "Callback erased the live descriptor");
}

ZTEST(xlnx_finite_rx, test_rx_rejects_status_bits_in_control_before_arm)
{
	struct k_work_sync sync;

	zassert_ok(configure_rx());
	/* Raw offsets are the hardware ABI, independent of C field names. */
	uint32_t *words = (uint32_t *)&bds[0];

	zassert_equal(words[0x18U / 4U], 512U);
	zassert_equal(words[0x1cU / 4U], 0U);
	words[0x18U / 4U] = 0x8c000200U;
	zassert_equal(dma_xlnx_sg_start(&dev, CH_RX), -EINVAL);
	(void)k_work_flush(&data.ch[CH_RX].error_work, &sync);
	zassert_false(data.ch[CH_RX].arm_valid);
	zassert_equal(regs[0x38U / 4U], 0U);
	zassert_equal(regs[0x40U / 4U], 0U);
	zassert_equal(data.ch[CH_RX].error_snapshot.arm_control, 0x8c000200U);
}
