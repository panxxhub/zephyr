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

#define CONFIG_DMA_XLNX_AXI_DMA_SG_NUM_RX_BD 64
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

/* The RX buffer region is Normal cacheable memory on the target, so the
 * ranges the driver invalidates are part of its contract.  native_sim has no
 * MMU memory types and no caches: record the calls and check the ranges.
 */
static struct {
	uintptr_t addr;
	size_t size;
} invd_log[16];
static unsigned int invd_count;

static int test_cache_invd(void *addr, size_t size)
{
	if (invd_count < ARRAY_SIZE(invd_log)) {
		invd_log[invd_count].addr = (uintptr_t)addr;
		invd_log[invd_count].size = size;
	}
	invd_count++;
	return 0;
}

static bool invd_logged(uintptr_t addr, size_t size)
{
	for (unsigned int i = 0; i < MIN(invd_count, ARRAY_SIZE(invd_log)); i++) {
		if (invd_log[i].addr == addr && invd_log[i].size == size) {
			return true;
		}
	}
	return false;
}

static uint32_t test_read32(mem_addr_t addr)
{
	return regs[addr / 4U];
}

static void engine_tail_write(uint32_t value);
static void engine_curdesc_write(uint32_t value);

static void test_write32(uint32_t value, mem_addr_t addr)
{
	if (addr == 0x04U || addr == 0x34U) {
		regs[addr / 4U] &= ~value; /* DMASR is write-1-to-clear */
		return;
	}
	regs[addr / 4U] = value;
	if (addr == 0x38U) {
		engine_curdesc_write(value);
	} else if (addr == 0x40U) {
		engine_tail_write(value);
	}
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
#define sys_cache_data_invd_range  test_cache_invd
#include "../../../../../drivers/dma/dma_xlnx_axi_dma_sg.c"

static struct xlnx_sg_bd bds[8];
static uint32_t rx_buffer[1024] __aligned(64);
static struct dma_xlnx_sg_data data;
static const struct dma_xlnx_sg_cfg config = {
	.rx_buf_phys = (uintptr_t)rx_buffer,
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
	memset(rx_buffer, 0, sizeof(rx_buffer));
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
		zassert_equal(bds[i].buf_addr, config.rx_buf_phys + 512U * i);
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
	memset(rx_buffer, 0, sizeof(rx_buffer));
}

ZTEST(xlnx_finite_rx, test_error_snapshot_precedes_callback_teardown)
{
	struct k_work_sync sync;

	zassert_ok(configure_rx());
	data.ch[CH_RX].callback = reset_ring_on_error;
	zassert_ok(dma_xlnx_sg_start(&dev, CH_RX));
	bds[0].control = 0x8c000200U;
	bds[1].status = 0x8c000200U;
	bds[2].app[4] = 0x12345678U;
	uint32_t expected[4][16];

	memcpy(expected, bds, sizeof(expected));
	for (uint32_t i = 0; i < 16U; i++) {
		rx_buffer[i] = 0x12340000U + i;
	}
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
	zassert_equal(snap.bd_count, 4U);
	zassert_mem_equal(snap.bd_words, expected, sizeof(expected));
	zassert_true(snap.payload_valid);
	for (uint32_t i = 0; i < 16U; i++) {
		zassert_equal(snap.payload[i], 0x12340000U + i);
		zassert_equal(rx_buffer[i], 0U);
	}
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

ZTEST(xlnx_finite_rx, test_error_dump_rejects_out_of_range_payload)
{
	struct k_work_sync sync;

	zassert_ok(configure_rx());
	zassert_ok(dma_xlnx_sg_start(&dev, CH_RX));
	bds[0].buf_addr = config.rx_buf_phys + config.rx_buf_size - 4U;
	regs[0x34U / 4U] = 0x00104109;
	dma_xlnx_sg_rx_isr(&dev);
	(void)k_work_flush(&data.ch[CH_RX].error_work, &sync);
	zassert_false(data.ch[CH_RX].error_snapshot.payload_valid);
	zassert_equal(data.ch[CH_RX].error_snapshot.bd_count, 4U);
}

/* ==========================================================================
 * Continuous RX stream: S2MM engine model
 *
 * PG021: the engine consumes descriptors from CURDESC up to and including
 * TAILDESC.  Running out of descriptors between packets leaves it idle, and
 * the stream back-pressures.  Running out inside a packet — before TLAST —
 * raises DMAIntErr and halts the channel.
 * ========================================================================== */
#define STREAM_BDS      64U
#define STREAM_BD_BYTES 2000U

static struct xlnx_sg_bd stream_bds[STREAM_BDS];
static uint8_t stream_buf[STREAM_BDS * STREAM_BD_BYTES];
static uint8_t placed_payload[2][2560] __aligned(32);
static const struct dma_xlnx_sg_cfg stream_config = {
	.rx_buf_phys = (uintptr_t)stream_buf,
	.rx_buf_size = sizeof(stream_buf),
	.sg_len_mask = 0x3fff,
};
static struct dma_xlnx_sg_data stream_data;
static const struct device stream_dev = {.data = &stream_data, .config = &stream_config};

static struct {
	uint32_t bds_written; /* BDs the engine has filled */
	uint32_t tail_abs;    /* BDs handed to the engine by TAILDESC */
	uint32_t completed;   /* BDs completed since the last IOC */
	bool overrun;         /* wrote a BD whose window was not consumed */
	bool halted;
	bool defer_ioc;
	bool placed_payload;
} eng;

static void engine_curdesc_write(uint32_t value)
{
	ARG_UNUSED(value);
	if (stream_data.ch[CH_RX].bds == stream_bds) {
		eng.bds_written = 0;
		eng.tail_abs = 0;
		eng.completed = 0;
	}
}

static void engine_tail_write(uint32_t value)
{
	struct dma_xlnx_sg_chan *ch = &stream_data.ch[CH_RX];
	uintptr_t base = (uintptr_t)stream_bds;

	if (ch->bds != stream_bds || value < base ||
	    value >= base + sizeof(stream_bds)) {
		return;
	}
	uint32_t idx = (uint32_t)((value - base) / sizeof(struct xlnx_sg_bd));
	uint32_t count = ch->active_bds;
	uint32_t last = (eng.tail_abs + count - 1U) % count;

	eng.tail_abs += (idx + count - last) % count;
}

static unsigned int stream_windows;
static unsigned int stream_windows_stale;
static uint32_t stream_window_size;
static uint32_t stream_first_stamp;
static unsigned int stream_errors;
static uint32_t stream_error_sr;

/* Every BD carries the absolute index of the BD the engine wrote it into. */
static void stream_stamp(uint32_t abs)
{
	uint32_t *p = (uint32_t *)&stream_buf[(abs % STREAM_BDS) * STREAM_BD_BYTES];

	if (eng.placed_payload) {
		uint32_t addr = stream_bds[abs % STREAM_BDS].buf_addr;

		zassert_true(addr == 0xffffe060U || addr == 0xffffea60U,
			     "descriptor escaped the two physical destinations");
		uint32_t slot = (addr - 0xffffe060U) / 2560U;

		p = (uint32_t *)&placed_payload[slot][96];
		for (uint32_t i = 0; i < STREAM_BD_BYTES / sizeof(*p); i++) {
			p[i] = 0xbd000000U + abs;
		}
	} else {
		*p = 0xbd000000U + abs;
	}
}

static void stream_window(const struct device *device, void *user, uint8_t *buf, uint32_t size,
			  uint32_t first_bd)
{
	ARG_UNUSED(first_bd);
	ARG_UNUSED(device);
	ARG_UNUSED(user);
	stream_windows++;
	stream_window_size = size;
	stream_first_stamp = *(uint32_t *)buf;
	if (!invd_logged((uintptr_t)buf, size)) {
		stream_windows_stale++;
	}
	invd_count = 0;
}

static void stream_error(const struct device *device, void *user, uint32_t dmasr)
{
	ARG_UNUSED(device);
	ARG_UNUSED(user);
	stream_errors++;
	stream_error_sr = dmasr;
}

static void engine_ioc(void)
{
	regs[0x34U / 4U] = DMASR_IOC_IRQ;
	dma_xlnx_sg_rx_isr(&stream_dev);
}

/* 0 on success, -EAGAIN when the engine is out of descriptors between
 * packets, -EIO for a packet that outran TAILDESC (DMAIntErr).
 */
static int feed_stream_packet(uint32_t bytes)
{
	struct dma_xlnx_sg_chan *ch = &stream_data.ch[CH_RX];
	uint32_t need = (bytes + STREAM_BD_BYTES - 1U) / STREAM_BD_BYTES;
	uint32_t runway = eng.tail_abs - eng.bds_written;

	if (runway == 0U) {
		return -EAGAIN;
	}
	if (need > runway) {
		eng.halted = true;
		regs[0x34U / 4U] = DMASR_HALTED | DMASR_INTERR | DMASR_ERR_IRQ;
		dma_xlnx_sg_rx_isr(&stream_dev);
		return -EIO;
	}

	for (uint32_t i = 0; i < need; i++) {
		uint32_t consumed = (uint32_t)atomic_get(&ch->rx_consumed_bds);

		if ((int32_t)(eng.bds_written - consumed - ch->active_bds) >= 0) {
			eng.overrun = true;
		}
		uint32_t idx = eng.bds_written % ch->active_bds;
		uint32_t len = (i + 1U == need) ? (bytes - i * STREAM_BD_BYTES)
						: STREAM_BD_BYTES;

		stream_stamp(eng.bds_written);
		stream_bds[idx].status = BIT(31) | len;
		eng.bds_written++;
		eng.completed++;
		if (!eng.defer_ioc && eng.completed >= ch->hw_irq_threshold) {
			eng.completed = 0;
			engine_ioc();
		}
	}
	return 0;
}

static int start_stream(uint16_t threshold)
{
	const struct dma_xlnx_sg_rx_stream_cfg cfg = {
		.bd_bytes = STREAM_BD_BYTES,
		.irq_threshold = threshold,
		.callback = stream_window,
		.error_callback = stream_error,
	};

	return dma_xlnx_sg_start_rx_stream(&stream_dev, &cfg);
}

static void stream_before(void *fixture)
{
	ARG_UNUSED(fixture);
	memset(&stream_data, 0, sizeof(stream_data));
	memset(regs, 0, sizeof(regs));
	memset(stream_bds, 0, sizeof(stream_bds));
	memset(stream_buf, 0, sizeof(stream_buf));
	memset(&eng, 0, sizeof(eng));
	stream_data.ch[CH_RX].dev = &stream_dev;
	stream_data.ch[CH_RX].bds = stream_bds;
	stream_data.ch[CH_RX].num_bds = STREAM_BDS;
	k_work_init(&stream_data.ch[CH_RX].error_work, dma_xlnx_sg_error_work);
	k_work_init(&stream_data.ch[CH_RX].rx_stream_work,
		    dma_xlnx_sg_rx_stream_work_handler);
	stream_windows = 0;
	stream_windows_stale = 0;
	stream_window_size = 0;
	invd_count = 0;
	stream_first_stamp = 0;
	stream_errors = 0;
	stream_error_sr = 0;
	regs[0x34U / 4U] = DMASR_HALTED | DMASR_SGINCL;
}

static void stream_after(void *fixture)
{
	struct k_work_sync sync;

	ARG_UNUSED(fixture);
	stream_data.ch[CH_RX].rx_stream_active = false;
	(void)k_work_cancel_sync(&stream_data.ch[CH_RX].rx_stream_work, &sync);
	(void)k_work_cancel_sync(&stream_data.ch[CH_RX].error_work, &sync);
}

ZTEST_SUITE(xlnx_rx_stream, NULL, NULL, stream_before, stream_after, NULL);

ZTEST(xlnx_rx_stream, test_runway_is_a_fixed_lead_not_the_irq_window)
{
	struct dma_xlnx_sg_chan *ch = &stream_data.ch[CH_RX];

	zassert_ok(start_stream(1));
	zassert_equal(ch->hw_irq_threshold, 1, "one BD per IOC");
	/* The defect: TAILDESC one hardware threshold ahead leaves a
	 * single-descriptor runway, so any two-BD packet halts the channel.
	 */
	zassert_equal(ch->rx_lead_bds, STREAM_BDS / 2U);
	zassert_equal(eng.tail_abs, STREAM_BDS / 2U);
	zassert_equal(regs[0x40U / 4U],
		      (uint32_t)(uintptr_t)&stream_bds[STREAM_BDS / 2U - 1U]);
}

ZTEST(xlnx_rx_stream, test_threshold_one_survives_a_reframed_packet)
{
	struct dma_xlnx_sg_rx_stream_stats stats;

	zassert_ok(start_stream(1));
	for (unsigned int i = 0; i < 200U; i++) {
		/* Packet 15 slips by one 16-byte frame and needs two BDs. */
		uint32_t bytes = (i == 14U) ? STREAM_BD_BYTES + 16U : STREAM_BD_BYTES;

		zassert_ok(feed_stream_packet(bytes), "stopped at packet %u", i);
		k_msleep(1);
	}
	zassert_false(eng.halted);
	zassert_false(eng.overrun, "engine wrote an unconsumed BD");
	zassert_equal(stream_errors, 0);
	/* 200 packets, one of which spans two BDs; one window per BD. */
	zassert_equal(stream_windows, 201U);
	zassert_equal(stream_windows_stale, 0U,
		      "every window must be invalidated before delivery");
	zassert_equal(stream_window_size, STREAM_BD_BYTES);
	zassert_equal(stream_first_stamp, 0xbd000000U + 200U,
		      "window must expose the BD the engine just filled");
	zassert_ok(dma_xlnx_sg_rx_stream_status(&stream_dev, &stats));
	zassert_true(stats.active);
	zassert_false(stats.halted);
	zassert_equal(stats.error_count, 0);
	zassert_equal(stats.bds_produced, 201U);
	zassert_equal(stats.bds_consumed, 201U);
}

ZTEST(xlnx_rx_stream, test_lagging_consumer_stalls_the_engine_without_overrun)
{
	struct dma_xlnx_sg_rx_stream_stats stats;
	int ret = 0;
	unsigned int fed = 0;

	zassert_ok(start_stream(1));
	/* Hold the system workqueue off so no window is ever consumed. */
	k_sched_lock();
	while (fed < 4U * STREAM_BDS) {
		ret = feed_stream_packet(STREAM_BD_BYTES);
		if (ret != 0) {
			break;
		}
		fed++;
	}
	zassert_ok(dma_xlnx_sg_rx_stream_status(&stream_dev, &stats));
	k_sched_unlock();
	zassert_equal(ret, -EAGAIN, "engine must idle, not halt");
	zassert_equal(fed, STREAM_BDS, "engine may fill the ring exactly once");
	zassert_false(eng.overrun);
	zassert_false(eng.halted);
	zassert_equal(stream_errors, 0);
	zassert_true(stats.overrun_count > 0U, "held-back tail must be counted");
	zassert_equal(stats.bds_consumed, 0);

	/* Let the consumer catch up: the engine gets its runway back. */
	k_msleep(10);
	zassert_ok(dma_xlnx_sg_rx_stream_status(&stream_dev, &stats));
	zassert_equal(stats.bds_consumed, STREAM_BDS);
	zassert_equal(stream_windows, STREAM_BDS);
	for (unsigned int i = 0; i < STREAM_BDS; i++) {
		zassert_ok(feed_stream_packet(STREAM_BD_BYTES));
		k_msleep(1);
	}
	zassert_false(eng.overrun);
	zassert_equal(stream_windows, 2U * STREAM_BDS);
}

ZTEST(xlnx_rx_stream, test_threshold_eight_window_delivery_is_unchanged)
{
	struct dma_xlnx_sg_rx_stream_stats stats;

	zassert_ok(start_stream(8));
	zassert_equal(stream_data.ch[CH_RX].hw_irq_threshold, 8);
	for (unsigned int i = 0; i < 200U; i++) {
		zassert_ok(feed_stream_packet(STREAM_BD_BYTES));
		k_msleep(1);
	}
	zassert_equal(stream_windows, 25U, "one window per 8 BDs");
	zassert_equal(stream_window_size, 8U * STREAM_BD_BYTES);
	zassert_equal(stream_windows_stale, 0U,
		      "the whole 16000-byte window is invalidated, not one BD");
	zassert_equal(stream_first_stamp, 0xbd000000U + 192U);
	zassert_false(eng.overrun);
	zassert_equal(stream_errors, 0);
	zassert_ok(dma_xlnx_sg_rx_stream_status(&stream_dev, &stats));
	zassert_equal(stats.bds_consumed, 200U);
	zassert_equal(stats.overrun_count, 0);
}

ZTEST(xlnx_rx_stream, test_dma_error_reaches_the_stream_consumer)
{
	struct dma_xlnx_sg_rx_stream_stats stats;

	zassert_ok(start_stream(1));
	/* Starve the runway down to a single descriptor with a consumer that
	 * never runs, then present a two-BD packet: DMAIntErr, the silent
	 * halt seen on silicon.
	 */
	k_sched_lock();
	for (unsigned int i = 0; i < STREAM_BDS - 1U; i++) {
		zassert_ok(feed_stream_packet(STREAM_BD_BYTES));
	}
	zassert_equal(feed_stream_packet(2U * STREAM_BD_BYTES), -EIO);
	k_sched_unlock();
	k_msleep(10);
	zassert_equal(stream_errors, 1, "halt must reach the stream consumer");
	zassert_true((stream_error_sr & DMASR_INTERR) != 0U);
	zassert_ok(dma_xlnx_sg_rx_stream_status(&stream_dev, &stats));
	zassert_true(stats.halted);
	zassert_equal(stats.error_count, 1);
	zassert_equal(stats.last_error, stream_error_sr);
}

static void placed_window(const struct device *device, void *user, uint8_t *buf, uint32_t size,
			  uint32_t first_bd)
{
	ARG_UNUSED(device);
	ARG_UNUSED(user);
	zassert_is_null(buf, "explicit destinations are CPU-invisible");
	zassert_equal(size, STREAM_BD_BYTES);
	zassert_equal(first_bd, stream_windows % 2U);
	for (unsigned int i = 0; i < invd_count; i++) {
		zassert_true(i < ARRAY_SIZE(invd_log));
		zassert_true(invd_log[i].addr >= (uintptr_t)stream_bds &&
				     invd_log[i].addr + invd_log[i].size <=
					     (uintptr_t)(stream_bds + 2),
			     "explicit placement must invalidate descriptors only");
	}
	stream_windows++;
	invd_count = 0;
}

ZTEST(xlnx_rx_stream, test_explicit_placement_and_cpu_invisible_completion)
{
	struct k_work_sync sync;
	struct dma_xlnx_sg_rx_stream_cfg cfg = {
		.bd_bytes = STREAM_BD_BYTES,
		.irq_threshold = 1,
		.callback = placed_window,
		.base_phys = 0xffffe060U,
		.stride = 2560U,
		.num_bds = 2,
	};

	zassert_equal(stream_data.ch[CH_RX].num_bds, STREAM_BDS);
	zassert_ok(dma_xlnx_sg_start_rx_stream(&stream_dev, &cfg));
	for (uint32_t i = 0; i < 2U; i++) {
		zassert_equal(stream_bds[i].buf_addr, cfg.base_phys + i * cfg.stride);
		zassert_equal(stream_bds[i].control, STREAM_BD_BYTES);
		zassert_equal(stream_bds[i].next_desc,
			      (uint32_t)(uintptr_t)&stream_bds[(i + 1U) % 2U]);
	}
	for (uint32_t i = 0; i < 6U; i++) {
		stream_bds[i % 2U].status = BD_STS_CMPLT | STREAM_BD_BYTES;
		engine_ioc();
		(void)k_work_flush(&stream_data.ch[CH_RX].rx_stream_work, &sync);
		zassert_equal(stream_windows, i + 1U, "NULL payload still delivers completion");
	}
	dma_xlnx_sg_stop_rx_stream(&stream_dev);
	/* Explicit memory belongs to the caller, even when larger than rx_buf. */
	cfg.num_bds = 0;
	cfg.base_phys = 0x02000000U;
	cfg.bd_bytes = 4096;
	cfg.stride = 4096;
	stream_data.ch[CH_RX].num_bds = STREAM_BDS;
	zassert_ok(dma_xlnx_sg_start_rx_stream(&stream_dev, &cfg));
}

ZTEST(xlnx_rx_stream, test_explicit_placement_alignment_and_default_reset)
{
	struct dma_xlnx_sg_rx_stream_cfg cfg = {
		.bd_bytes = STREAM_BD_BYTES,
		.irq_threshold = 1,
		.callback = placed_window,
		.base_phys = 0xffffe061U,
		.stride = 2560U,
	};

	zassert_equal(dma_xlnx_sg_start_rx_stream(&stream_dev, &cfg), -EINVAL);
	cfg.base_phys--;
	cfg.stride++;
	zassert_equal(dma_xlnx_sg_start_rx_stream(&stream_dev, &cfg), -EINVAL);
	cfg.stride = 1984;
	zassert_equal(dma_xlnx_sg_start_rx_stream(&stream_dev, &cfg), -EINVAL);
	cfg.stride = 2560;
	zassert_ok(dma_xlnx_sg_start_rx_stream(&stream_dev, &cfg));
	dma_xlnx_sg_stop_rx_stream(&stream_dev);
	zassert_ok(start_stream(1));
	zassert_equal(stream_bds[1].buf_addr, stream_config.rx_buf_phys + STREAM_BD_BYTES);
	zassert_ok(feed_stream_packet(STREAM_BD_BYTES));
	k_msleep(1);
	zassert_equal(stream_windows, 1U);
	zassert_equal(stream_windows_stale, 0U);
}

ZTEST(xlnx_rx_stream, test_explicit_payload_is_invisible_to_error_dump)
{
	struct k_work_sync sync;
	struct dma_xlnx_sg_rx_stream_stats stats;
	struct dma_xlnx_sg_rx_stream_cfg cfg = {
		.bd_bytes = STREAM_BD_BYTES,
		.irq_threshold = 1,
		.callback = placed_window,
		.error_callback = stream_error,
		.base_phys = ROUND_UP((uintptr_t)stream_buf, 32U),
		.stride = 2560U,
	};

	stream_data.ch[CH_RX].num_bds = 2;
	zassert_ok(dma_xlnx_sg_start_rx_stream(&stream_dev, &cfg));
	regs[0x34U / 4U] = DMASR_HALTED | DMASR_INTERR | DMASR_ERR_IRQ;
	dma_xlnx_sg_rx_isr(&stream_dev);
	(void)k_work_flush(&stream_data.ch[CH_RX].rx_stream_work, &sync);
	(void)k_work_flush(&stream_data.ch[CH_RX].error_work, &sync);
	zassert_equal(stream_errors, 1U);
	zassert_false(stream_data.ch[CH_RX].error_snapshot.payload_valid);
	zassert_false(invd_logged(cfg.base_phys, 64U));
	zassert_ok(dma_xlnx_sg_rx_stream_status(&stream_dev, &stats));
	zassert_true(stats.halted);
	zassert_equal(stats.error_count, 1U);
	zassert_equal(stats.last_error, stream_error_sr);
}

ZTEST(xlnx_rx_stream, test_stream_subset_preserves_finite_pool)
{
	struct dma_xlnx_sg_rx_stream_cfg cfg = {
		.bd_bytes = STREAM_BD_BYTES,
		.irq_threshold = 1,
		.callback = placed_window,
		.base_phys = 0xffffe060U,
		.stride = 2560U,
		.num_bds = 2,
	};
	struct dma_block_config block = {
		.block_size = 512,
		.dest_scatter_count = 8,
	};
	struct dma_config finite = {
		.channel_direction = PERIPHERAL_TO_MEMORY,
		.head_block = &block,
	};

	cfg.num_bds = STREAM_BDS + 1;
	zassert_equal(dma_xlnx_sg_start_rx_stream(&stream_dev, &cfg), -EINVAL);
	cfg.num_bds = 2;
	cfg.irq_threshold = 4;
	zassert_equal(dma_xlnx_sg_start_rx_stream(&stream_dev, &cfg), -EINVAL);
	cfg.irq_threshold = 1;
	zassert_ok(dma_xlnx_sg_start_rx_stream(&stream_dev, &cfg));
	zassert_equal(stream_data.ch[CH_RX].active_bds, 2);
	zassert_equal(stream_data.ch[CH_RX].rx_lead_bds, 1U);
	zassert_equal(stream_data.ch[CH_RX].tail_idx, 0U);
	zassert_equal(stream_bds[2].control, 0, "unused pool must not be armed");
	zassert_equal(dma_xlnx_sg_reserve_rx(&stream_dev), -EBUSY);
	dma_xlnx_sg_stop_rx_stream(&stream_dev);
	zassert_ok(dma_xlnx_sg_reserve_rx(&stream_dev));
	zassert_ok(dma_xlnx_sg_config(&stream_dev, CH_RX, &finite));
	zassert_equal(stream_data.ch[CH_RX].num_bds, STREAM_BDS);
	zassert_equal(stream_data.ch[CH_RX].active_bds, 8);
	zassert_equal(stream_bds[7].buf_addr, stream_config.rx_buf_phys + 7U * 512U);
	dma_xlnx_sg_release_rx(&stream_dev);
}

/* Payload slots may wrap while unconsumed descriptors still retain completion
 * metadata. Count every completed BD and preserve its actual ring index.
 */
static void repeated_slot_window(const struct device *device, void *user, uint8_t *buf,
				 uint32_t size, uint32_t first_bd)
{
	ARG_UNUSED(device);
	ARG_UNUSED(user);
	zassert_is_null(buf);
	zassert_equal(size, STREAM_BD_BYTES);
	zassert_equal(first_bd, stream_windows % STREAM_BDS);
	stream_windows++;
}

ZTEST(xlnx_rx_stream, test_repeated_slots_survive_delayed_harvest)
{
	struct dma_xlnx_sg_rx_stream_cfg cfg = {
		.bd_bytes = STREAM_BD_BYTES,
		.irq_threshold = 1,
		.callback = repeated_slot_window,
		.base_phys = 0xffffe060U,
		.stride = 2560U,
		.num_bds = STREAM_BDS,
		.num_slots = 2,
	};
	struct dma_xlnx_sg_rx_stream_stats stats;
	struct k_work_sync sync;

	zassert_ok(dma_xlnx_sg_start_rx_stream(&stream_dev, &cfg));
	eng.placed_payload = true;
	eng.defer_ioc = true;
	memset(placed_payload, 0xa5, sizeof(placed_payload));
	for (uint32_t i = 0; i < STREAM_BDS; i++) {
		zassert_equal(stream_bds[i].buf_addr, cfg.base_phys + (i % 2U) * cfg.stride);
		zassert_equal(stream_bds[i].next_desc,
			      (uint32_t)(uintptr_t)&stream_bds[(i + 1U) % STREAM_BDS]);
	}
	for (uint32_t batch = 0; batch < 128U; batch++) {
		int ret = 0;

		/* Eight 125-us bursts before IRQ harvest, sixteen before workqueue
		 * delivery. Feed attempts are not retried if TAILDESC runs out.
		 */
		k_sched_lock();
		for (uint32_t phase = 0; phase < 2U && ret == 0; phase++) {
			for (uint32_t burst = 0; burst < 8U; burst++) {
				ret = feed_stream_packet(STREAM_BD_BYTES);
				if (ret != 0) {
					break;
				}
			}
			engine_ioc();
			eng.completed = 0;
		}
		k_sched_unlock();
		(void)k_work_flush(&stream_data.ch[CH_RX].rx_stream_work, &sync);
		zassert_ok(ret, "lost a burst while harvest was delayed");
		zassert_ok(dma_xlnx_sg_rx_stream_status(&stream_dev, &stats));
		zassert_equal(eng.bds_written, (batch + 1U) * 16U);
		zassert_equal(stats.bds_produced, eng.bds_written);
		zassert_equal(stats.bds_consumed, eng.bds_written);
		zassert_equal(stream_windows, eng.bds_written);
		zassert_equal(stats.overrun_count, 0U);
		zassert_equal(stats.error_count, 0U);
		zassert_false(eng.halted || eng.overrun);
		for (uint32_t slot = 0; slot < 2U; slot++) {
			uint32_t *words = (uint32_t *)&placed_payload[slot][96];

			for (uint32_t word = 0; word < STREAM_BD_BYTES / 4U; word++) {
				zassert_equal(words[word],
					      0xbd000000U + eng.bds_written - 2U + slot);
			}
			for (uint32_t byte = 0; byte < 96U; byte++) {
				zassert_equal(placed_payload[slot][byte], 0xa5);
			}
		}
	}
	zassert_equal(stream_data.ch[CH_RX].rx_lead_bds, 32U);
	dma_xlnx_sg_stop_rx_stream(&stream_dev);
	/* A later explicit stream may use one destination per descriptor. */
	cfg.num_slots = 0;
	zassert_ok(dma_xlnx_sg_start_rx_stream(&stream_dev, &cfg));
	zassert_equal(stream_bds[2].buf_addr, cfg.base_phys + 2U * cfg.stride);
}

ZTEST(xlnx_rx_stream, test_repeated_slot_admission)
{
	struct dma_xlnx_sg_rx_stream_cfg cfg = {
		.bd_bytes = STREAM_BD_BYTES,
		.irq_threshold = 1,
		.callback = repeated_slot_window,
		.stride = 2560U,
		.num_bds = STREAM_BDS,
		.num_slots = 2,
	};

	zassert_equal(dma_xlnx_sg_start_rx_stream(&stream_dev, &cfg), -EINVAL);
	cfg.base_phys = 0xffffe060U;
	cfg.num_slots = STREAM_BDS + 1U;
	zassert_equal(dma_xlnx_sg_start_rx_stream(&stream_dev, &cfg), -EINVAL);
	cfg.num_slots = 3;
	zassert_equal(dma_xlnx_sg_start_rx_stream(&stream_dev, &cfg), -EINVAL);
	cfg.num_slots = 2;
	zassert_ok(dma_xlnx_sg_start_rx_stream(&stream_dev, &cfg));
}
