/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Compiles the actual GEM driver functions against a fake controller with the
 * opt-in options of panxxhub/zephyr#67 defined, and asserts the properties
 * those options promise. Building the same cases without the options is the
 * reverse control: the historical behaviour must fail the assertions.
 */
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* OPTIONS */

#define CONFIG_DCACHE               1
#define CONFIG_DCACHE_LINE_SIZE     32
#define BIT(n)                      (1U << (n))
#define K_FOREVER                   -1
#define K_NO_WAIT                   0
#define K_MSEC(n)                   (n)
#define K_SEM_MAX_LIMIT             UINT32_MAX
#define NET_AF_UNSPEC               0
#define ARG_UNUSED(x)               ((void)(x))
#define MIN(a, b)                   ((a) < (b) ? (a) : (b))
#define ARRAY_SIZE(a)               (sizeof(a) / sizeof((a)[0]))
#define ROUND_UP(x, align)          ((((x) + ((align) - 1)) / (align)) * (align))
#define ROUND_DOWN(x, align)        (((x) / (align)) * (align))
#define LOG_ERR_RATELIMIT_RATE(rate, ...) ((void)0)
#define LOG_WRN_RATELIMIT_RATE(...) ((void)0)
#define LOG_DBG(...)                ((void)0)
#define DT_INST_FOREACH_STATUS_OKAY(m)

#define RX_BD_COUNT  255U
#define TX_BD_COUNT  64U
#define BUFFER_SIZE  2048U

struct k_spinlock {
	bool held;
};
typedef uint32_t k_spinlock_key_t;
static k_spinlock_key_t k_spin_lock(struct k_spinlock *lock)
{
	assert(!lock->held);
	lock->held = true;
	return 0U;
}
static void k_spin_unlock(struct k_spinlock *lock, k_spinlock_key_t key)
{
	assert(lock->held);
	lock->held = false;
}
struct k_sem {
	int count;
};
struct k_mutex {
	int unused;
};
struct k_work {
	bool pending;
};
struct k_work_q {
	int unused;
};
struct net_pkt {
	uint32_t len;
	uint32_t cursor;
	uint8_t tag;
};
struct eth_xlnx_gem_bd {
	uint32_t addr, ctrl;
};
struct ring {
	struct eth_xlnx_gem_bd *first_bd;
	uint8_t next_to_process, next_to_use, free_bds;
	struct k_sem ring_sem;
};
struct eth_xlnx_gem_dev_cfg {
	uint16_t rx_buffer_size, tx_buffer_size;
	uint8_t rx_bd_count, tx_bd_count;
	bool defer_rxp_to_queue, defer_txd_to_queue;
};
struct eth_xlnx_gem_stats {
	uint32_t rx_overruns;
	uint32_t rx_buffer_not_available;
	uint32_t rx_queue_resets;
	uint32_t rx_reset_discards;
	uint32_t tx_send_timeouts;
	uint32_t tx_age_reclaims;
	uint32_t tx_ring_full;
};
struct eth_xlnx_gem_dev_data {
	struct k_spinlock nwcfg_lock;
	struct eth_xlnx_gem_stats diag;
	struct ring rx_bd_ring, tx_bd_ring;
	uint8_t *first_rx_buffer, *first_tx_buffer;
	void *iface;
	struct k_sem tx_done_sem;
	struct k_sem tx_space_sem;
	struct k_mutex tx_lock;
	struct k_work rx_pend_work, tx_done_work;
};
struct device {
	const char *name;
};
static struct device device = {"GEM0"};
static struct eth_xlnx_gem_dev_cfg cfg;
static struct eth_xlnx_gem_dev_data data;
static struct eth_xlnx_gem_bd rx[RX_BD_COUNT], tx[TX_BD_COUNT];
static _Alignas(4096) uint8_t rxbuf[RX_BD_COUNT * BUFFER_SIZE];
static _Alignas(4096) uint8_t txbuf[TX_BD_COUNT * BUFFER_SIZE];
static uint32_t mmio[512];
#define DEV_CFG(dev)                     (&cfg)
#define DEV_DATA(dev)                    (&data)
#define DEVICE_MMIO_NAMED_GET(dev, name) ((uintptr_t)mmio)
static struct k_work_q eth_xlnx_gem_workq;
/* MACROS */

/* Recorded cache maintenance, so the span of every operation can be checked. */
struct cache_op {
	uintptr_t address;
	size_t length;
	bool flush;
};
static struct cache_op cache_ops[512];
static uint32_t cache_op_count;

/* Fake controller state. */
static bool tx_confirm;
static uint32_t tx_started;
/* Order in which the controller picked frames off the ring, by payload tag. */
static uint8_t wire[4 * TX_BD_COUNT];
static uint32_t wire_count;
static uint8_t wire_next;
static uint32_t tx_tag;
static bool tx_lock_held;
static uint32_t tx_reentries_admitted;
static bool reentry_pending;
static int reentry_result;
static struct net_pkt reentry_pkt;
static uint32_t delivered, delivered_bytes;
static struct net_pkt rx_packet;
static uint8_t rx_scratch[16384];
static uint32_t rx_scratch_len;

static uint32_t sys_read32(uintptr_t addr)
{
	return *(uint32_t *)addr;
}
/*
 * The controller takes every armed transmission off the ring in ring order and
 * marks the first BD of each one used, which is how it reports that the frame
 * has been transmitted. An armed BD it has already taken must never be armed
 * again before the driver has reclaimed it.
 */
static void model_transmit(void)
{
	while ((tx[wire_next].ctrl & ETH_XLNX_GEM_TX_BD_USED_BIT) == 0U) {
		uint8_t first = wire_next;
		uint32_t bds = 0U;

		if (wire_count < ARRAY_SIZE(wire)) {
			wire[wire_count] = txbuf[(uint32_t)first * BUFFER_SIZE];
		}
		wire_count++;

		while ((tx[wire_next].ctrl & ETH_XLNX_GEM_TX_BD_LAST_BIT) == 0U) {
			wire_next = (uint8_t)((wire_next + 1U) % TX_BD_COUNT);
			assert(++bds < TX_BD_COUNT);
		}
		wire_next = (uint8_t)((wire_next + 1U) % TX_BD_COUNT);
		tx[first].ctrl |= ETH_XLNX_GEM_TX_BD_USED_BIT;
		tx_started++;
	}
}
static void sys_write32(uint32_t value, uintptr_t addr)
{
	if (addr == (uintptr_t)mmio + ETH_XLNX_GEM_NWCTRL_OFFSET &&
	    (value & ETH_XLNX_GEM_NWCTRL_STARTTX_BIT) != 0U) {
		*(uint32_t *)addr = value;
		model_transmit();
		return;
	}
	if (addr == (uintptr_t)mmio + ETH_XLNX_GEM_IDR_OFFSET) {
		mmio[ETH_XLNX_GEM_IMR_OFFSET / 4] |= value;
	} else if (addr == (uintptr_t)mmio + ETH_XLNX_GEM_IER_OFFSET) {
		mmio[ETH_XLNX_GEM_IMR_OFFSET / 4] &= ~value;
	} else if (addr == (uintptr_t)mmio + ETH_XLNX_GEM_ISR_OFFSET ||
		   addr == (uintptr_t)mmio + ETH_XLNX_GEM_RXSR_OFFSET ||
		   addr == (uintptr_t)mmio + ETH_XLNX_GEM_TXSR_OFFSET) {
		*(uint32_t *)addr &= ~value;
		return;
	}
	*(uint32_t *)addr = value;
}
static void barrier_dmem_fence_full(void)
{
}
static void record_cache_op(void *p, size_t n, bool flush)
{
	assert(cache_op_count < ARRAY_SIZE(cache_ops));
	cache_ops[cache_op_count].address = (uintptr_t)p;
	cache_ops[cache_op_count].length = n;
	cache_ops[cache_op_count].flush = flush;
	cache_op_count++;
}
static void sys_cache_data_flush_and_invd_range(void *p, size_t n)
{
	record_cache_op(p, n, true);
}
static void sys_cache_data_invd_range(void *p, size_t n)
{
	record_cache_op(p, n, false);
}
static void net_pkt_cursor_init(struct net_pkt *p)
{
	p->cursor = 0U;
}
static int net_pkt_read(struct net_pkt *p, void *b, size_t n)
{
	for (size_t i = 0U; i < n; i++) {
		((uint8_t *)b)[i] = (p->cursor + i == 0U)
					    ? p->tag
					    : (uint8_t)((p->cursor + i) & 0xFFU);
	}
	p->cursor += (uint32_t)n;
	return 0;
}
static size_t net_pkt_get_len(struct net_pkt *p)
{
	return p->len;
}
static void net_pkt_unref(struct net_pkt *p)
{
}
static struct net_pkt *net_pkt_rx_alloc_with_buffer(void *iface, uint32_t len, int f, int pr,
						    int t)
{
	rx_packet.len = len;
	rx_scratch_len = 0U;
	return &rx_packet;
}
static int net_pkt_write(struct net_pkt *p, const void *b, size_t n)
{
	assert(rx_scratch_len + n <= sizeof(rx_scratch));
	memcpy(&rx_scratch[rx_scratch_len], b, n);
	rx_scratch_len += (uint32_t)n;
	return 0;
}
static int net_recv_data(void *iface, struct net_pkt *p)
{
	delivered++;
	delivered_bytes = p->len;
	return 0;
}
static int k_work_submit(struct k_work *work)
{
	work->pending = true;
	return 0;
}
static int k_work_submit_to_queue(struct k_work_q *queue, struct k_work *work)
{
	assert(queue == &eth_xlnx_gem_workq);
	work->pending = true;
	return 0;
}
static void k_sem_give(struct k_sem *sem)
{
	sem->count++;
}
static void k_sem_init(struct k_sem *sem, unsigned int initial, unsigned int limit)
{
	sem->count = (int)initial;
}
static void k_sem_reset(struct k_sem *sem)
{
	sem->count = 0;
}
static void k_mutex_init(struct k_mutex *mutex)
{
	tx_lock_held = false;
}
static int k_mutex_lock(struct k_mutex *mutex, int timeout)
{
	/* A second thread would block here; the model never admits one. */
	assert(!tx_lock_held);
	tx_lock_held = true;
	return 0;
}
static int k_mutex_unlock(struct k_mutex *mutex)
{
	assert(tx_lock_held);
	tx_lock_held = false;
	return 0;
}
static void eth_xlnx_gem_isr(const struct device *dev);
static int eth_xlnx_gem_send(const struct device *dev, struct net_pkt *pkt);
static void eth_xlnx_gem_handle_tx_done(const struct device *dev);
static void eth_xlnx_gem_handle_rx_pending(const struct device *dev);
static int k_sem_take(struct k_sem *sem, int timeout)
{
	if (sem == &data.tx_space_sem && sem->count <= 0) {
		/*
		 * A sender waiting for room in the ring. The controller
		 * reports its completions while the sender waits, or reports
		 * nothing at all and the wait times out.
		 */
		if (!tx_confirm) {
			return -EAGAIN;
		}
		mmio[ETH_XLNX_GEM_ISR_OFFSET / 4] |= ETH_XLNX_GEM_IXR_TX_COMPLETE_BIT;
		eth_xlnx_gem_isr(&device);
		if (data.tx_done_work.pending) {
			data.tx_done_work.pending = false;
			eth_xlnx_gem_handle_tx_done(&device);
		}
		if (sem->count <= 0) {
			return -EAGAIN;
		}
	}
	if (sem == &data.tx_done_sem) {
		/*
		 * A second sender reaching eth_xlnx_gem_send() while this one
		 * waits for its confirmation. Serialization keeps it out; the
		 * model refuses to admit it exactly when the driver holds the
		 * transmit lock, which is where a real thread would block.
		 */
		if (reentry_pending && !tx_lock_held) {
			reentry_pending = false;
			tx_reentries_admitted++;
			reentry_result = eth_xlnx_gem_send(&device, &reentry_pkt);
		}
		if (tx_confirm && tx_started > 0U) {
			tx_started--;
			mmio[ETH_XLNX_GEM_ISR_OFFSET / 4] |= ETH_XLNX_GEM_IXR_TX_COMPLETE_BIT;
			eth_xlnx_gem_isr(&device);
			if (data.tx_done_work.pending) {
				data.tx_done_work.pending = false;
				eth_xlnx_gem_handle_tx_done(&device);
			}
		}
	}
	if (sem->count <= 0) {
		return -EAGAIN;
	}
	sem->count--;
	return 0;
}
/* FUNCTIONS */

static void init(bool defer_txd)
{
	cfg = (struct eth_xlnx_gem_dev_cfg){BUFFER_SIZE, BUFFER_SIZE, RX_BD_COUNT, TX_BD_COUNT,
					    true, defer_txd};
	memset(&data, 0, sizeof(data));
	memset(mmio, 0, sizeof(mmio));
	memset(rx, 0, sizeof(rx));
	memset(tx, 0, sizeof(tx));
	memset(rxbuf, 0xA5, sizeof(rxbuf));
	memset(txbuf, 0x5A, sizeof(txbuf));
	data.rx_bd_ring.first_bd = rx;
	data.tx_bd_ring.first_bd = tx;
	data.first_rx_buffer = rxbuf;
	data.first_tx_buffer = txbuf;
	data.tx_bd_ring.ring_sem.count = 1;
	wire_count = 0U;
	wire_next = 0U;
	tx_tag = 0U;
	tx_lock_held = false;
	k_mutex_init(&data.tx_lock);
	cache_op_count = 0U;
	tx_started = 0U;
	tx_confirm = true;
	tx_reentries_admitted = 0U;
	reentry_pending = false;
	delivered = delivered_bytes = 0U;
	eth_xlnx_gem_configure_buffers(&device);
	mmio[ETH_XLNX_GEM_NWCTRL_OFFSET / 4] = ETH_XLNX_GEM_NWCTRL_RXEN_BIT |
					       ETH_XLNX_GEM_NWCTRL_TXEN_BIT;
}

/* Every length class a 2048-byte buffer can produce, including its edges. */
static const uint32_t lengths[] = {1,    14,   32,   33,   46,   60,   64,   65,
				   128,  512,  1024, 1500, 1514, 1536, 2016, 2047,
				   2048, 2049, 3000, 4096, 4097, 6000};
#define LENGTH_COUNT ARRAY_SIZE(lengths)

static void check_span(const struct cache_op *op, const uint8_t *area, size_t area_size,
		       uint32_t bd_idx, uint32_t chunk, uint32_t buffer_size)
{
	/* The operation covers the buffer of the descriptor it belongs to. */
	assert(op->address == (uintptr_t)area + bd_idx * buffer_size);
	assert(op->address + op->length <= (uintptr_t)area + area_size);
	/* Both ends of a maintained range must be cache line aligned. */
	assert((op->address % CONFIG_DCACHE_LINE_SIZE) == 0U);
	assert((op->length % CONFIG_DCACHE_LINE_SIZE) == 0U);
	/* The payload the CPU reads or writes is covered in full. */
	assert(op->length >= chunk);
	/* The operation never reaches past the buffer it belongs to. */
	assert(op->length <= buffer_size);
}

/*
 * Total maintained bytes when every buffer is maintained to the length of the
 * frame fragment it holds: full buffers in full, the remainder rounded up to
 * the next cache line. Maintaining whole buffers exceeds this for any frame
 * that does not exactly fill its last buffer.
 */
static uint32_t frame_sized_total(uint32_t length, uint32_t bds, uint32_t buffer_size)
{
	uint32_t head = (bds - 1U) * buffer_size;

	return head + ROUND_UP(length - head, CONFIG_DCACHE_LINE_SIZE);
}

/* T30: every length class round trips, with the maintained span checked. */
static void frame_lengths_test(void)
{
	for (uint32_t n = 0U; n < LENGTH_COUNT; n++) {
		uint32_t length = lengths[n];
		uint32_t bds = (length + BUFFER_SIZE - 1U) / BUFFER_SIZE;
		uint32_t remaining = length;
		struct net_pkt pkt = {length, 0U, 0U};
		uint32_t maintained = 0U;

		init(false);

		/*
		 * Transmit: the flush must cover exactly the bytes written.
		 * The send function releases the descriptors in reverse order,
		 * so the last descriptor of the frame is flushed first.
		 */
		assert(eth_xlnx_gem_send(&device, &pkt) == 0);
		assert(cache_op_count == bds);
		for (uint32_t i = 0U; i < cache_op_count; i++) {
			uint32_t bd_idx = bds - 1U - i;
			uint32_t chunk = MIN(length - bd_idx * BUFFER_SIZE, BUFFER_SIZE);

			assert(cache_ops[i].flush);
			check_span(&cache_ops[i], txbuf, sizeof(txbuf), bd_idx, chunk,
				   BUFFER_SIZE);
			maintained += (uint32_t)cache_ops[i].length;
			remaining -= chunk;
		}
		assert(remaining == 0U);
		assert(data.tx_bd_ring.free_bds == TX_BD_COUNT);
		/* A short frame must not pay for the whole buffer. */
		assert(maintained == frame_sized_total(length, bds, BUFFER_SIZE));

		/* Receive the same length back through the RX ring. */
		cache_op_count = 0U;
		remaining = length;
		for (uint32_t i = 0U; i < bds; i++) {
			uint32_t chunk = MIN(remaining, BUFFER_SIZE);
			uint8_t *buffer = &rxbuf[i * BUFFER_SIZE];

			for (uint32_t b = 0U; b < chunk; b++) {
				buffer[b] = (uint8_t)((i * BUFFER_SIZE + b) & 0xFFU);
			}
			rx[i].addr |= ETH_XLNX_GEM_RX_BD_USED_BIT;
			rx[i].ctrl = (i == 0U) ? ETH_XLNX_GEM_RX_BD_START_OF_FRAME_BIT : 0U;
			if (i + 1U == bds) {
				rx[i].ctrl |= ETH_XLNX_GEM_RX_BD_END_OF_FRAME_BIT | length;
			}
			remaining -= chunk;
		}
		eth_xlnx_gem_handle_rx_pending(&device);

		assert(delivered == 1U && delivered_bytes == length);
		assert(rx_scratch_len == length);
		for (uint32_t b = 0U; b < length; b++) {
			assert(rx_scratch[b] == (uint8_t)(((b / BUFFER_SIZE) * BUFFER_SIZE +
							   (b % BUFFER_SIZE)) & 0xFFU));
		}
		assert(cache_op_count == bds);
		remaining = length;
		maintained = 0U;
		for (uint32_t i = 0U; i < cache_op_count; i++) {
			uint32_t chunk = MIN(remaining, BUFFER_SIZE);

			assert(!cache_ops[i].flush);
			check_span(&cache_ops[i], rxbuf, sizeof(rxbuf), i, chunk, BUFFER_SIZE);
			maintained += (uint32_t)cache_ops[i].length;
			remaining -= chunk;
		}
		assert(remaining == 0U);
		assert(maintained == frame_sized_total(length, bds, BUFFER_SIZE));
		assert(data.rx_bd_ring.next_to_process == bds % RX_BD_COUNT);
	}
}

/* T31: a transmission whose confirmation never arrives must not leak BDs. */
static void tx_timeout_test(bool defer_txd)
{
	struct net_pkt pkt = {1500U, 0U, 0U};

	init(defer_txd);
	tx_confirm = false;

	for (uint32_t n = 0U; n < 4U * TX_BD_COUNT; n++) {
		assert(eth_xlnx_gem_send(&device, &pkt) == -EIO);
		/*
		 * Historical behaviour books out the descriptors and never
		 * accounts for them again, so this count walks down to zero
		 * and the interface stops being able to transmit at all.
		 */
		assert(data.tx_bd_ring.free_bds == TX_BD_COUNT);
		assert(data.tx_bd_ring.next_to_process == data.tx_bd_ring.next_to_use);
		assert(data.tx_done_sem.count == 0);
	}

	/* The ring is intact, so a confirmed transmission still succeeds. */
	tx_confirm = true;
	assert(eth_xlnx_gem_send(&device, &pkt) == 0);
	assert(data.tx_bd_ring.free_bds == TX_BD_COUNT);
}

/* T32: a late confirmation must not release the next sender or the ring. */
static void tx_late_confirmation_test(void)
{
	struct net_pkt pkt = {1500U, 0U, 0U};

	init(false);
	tx_confirm = false;
	assert(eth_xlnx_gem_send(&device, &pkt) == -EIO);
	assert(data.tx_bd_ring.free_bds == TX_BD_COUNT);

	/* The controller finally reports the abandoned frame. */
	tx_started = 1U;
	mmio[ETH_XLNX_GEM_ISR_OFFSET / 4] |= ETH_XLNX_GEM_IXR_TX_COMPLETE_BIT;
	eth_xlnx_gem_isr(&device);
	assert(data.tx_done_sem.count == 0);
	assert(data.tx_bd_ring.free_bds == TX_BD_COUNT);
	assert(data.tx_bd_ring.next_to_process == data.tx_bd_ring.next_to_use);

	/* And the next transmission is unaffected by it. */
	tx_confirm = true;
	tx_started = 0U;
	assert(eth_xlnx_gem_send(&device, &pkt) == 0);
	assert(data.tx_bd_ring.free_bds == TX_BD_COUNT);
}

/* T33: a second sender must not consume the first sender's confirmation. */
static void tx_reentry_test(void)
{
	struct net_pkt pkt = {1500U, 0U, 0U};

	init(false);
	reentry_pkt.len = 1500U;
	reentry_pkt.cursor = 0U;
	reentry_pending = true;

	assert(eth_xlnx_gem_send(&device, &pkt) == 0);
	assert(tx_reentries_admitted == 0U);
	assert(data.tx_bd_ring.free_bds == TX_BD_COUNT);
}

/* T34: the deferred work of this driver must not reach the system work queue. */
static void workq_test(void)
{
	init(true);
	mmio[ETH_XLNX_GEM_ISR_OFFSET / 4] = ETH_XLNX_GEM_IXR_FRAME_RX_BIT |
					    ETH_XLNX_GEM_IXR_TX_COMPLETE_BIT;
	eth_xlnx_gem_isr(&device);
	assert(data.rx_pend_work.pending && data.tx_done_work.pending);
	/* The interrupt did no descriptor work of its own in either direction. */
	assert(cache_op_count == 0U);
	assert(data.tx_bd_ring.next_to_process == 0U);
}

/* Queue one frame, tagging its payload so its place on the wire is known. */
static int queue_frame(uint32_t length)
{
	struct net_pkt pkt = {length, 0U, (uint8_t)tx_tag};

	tx_tag++;
	return eth_xlnx_gem_send(&device, &pkt);
}

/* T35: more than one transmission is in flight, and they keep their order. */
static void tx_async_pipeline_test(bool defer_txd)
{
	const uint32_t frames = 8U;

	init(defer_txd);
	/* The controller reports completions only to a sender that waits. */
	tx_confirm = true;

	for (uint32_t n = 0U; n < frames; n++) {
		assert(queue_frame(1500U) == 0);
	}

	/*
	 * Historical behaviour waits for the completion of each frame before
	 * returning, so the ring is empty again after every send and only one
	 * transmission is ever outstanding.
	 */
	assert(data.tx_bd_ring.free_bds == TX_BD_COUNT - frames);
	assert(wire_count == frames);
	for (uint32_t n = 0U; n < frames; n++) {
		assert(wire[n] == (uint8_t)n);
	}

	/* One completion interrupt reclaims every finished transmission. */
	mmio[ETH_XLNX_GEM_ISR_OFFSET / 4] |= ETH_XLNX_GEM_IXR_TX_COMPLETE_BIT;
	eth_xlnx_gem_isr(&device);
	if (data.tx_done_work.pending) {
		data.tx_done_work.pending = false;
		eth_xlnx_gem_handle_tx_done(&device);
	}
	assert(data.tx_bd_ring.free_bds == TX_BD_COUNT);
	assert(data.tx_bd_ring.next_to_process == data.tx_bd_ring.next_to_use);
}

/* T36: a full ring holds the sender back instead of overrunning descriptors. */
static void tx_async_backpressure_test(void)
{
	init(false);
	tx_confirm = true;

	/* Fill the ring: no completion is reported while nothing waits. */
	for (uint32_t n = 0U; n < TX_BD_COUNT; n++) {
		assert(queue_frame(1500U) == 0);
		assert(data.tx_bd_ring.free_bds <= TX_BD_COUNT);
	}
	assert(data.tx_bd_ring.free_bds == 0U);

	/* Further frames wait for room; none of them may overrun the ring. */
	for (uint32_t n = 0U; n < 2U * TX_BD_COUNT; n++) {
		assert(queue_frame(1500U) == 0);
		assert(data.tx_bd_ring.free_bds <= TX_BD_COUNT);
		assert(data.tx_bd_ring.next_to_use < TX_BD_COUNT);
		assert(data.tx_bd_ring.next_to_process < TX_BD_COUNT);
	}
	assert(wire_count == 3U * TX_BD_COUNT);
	for (uint32_t n = 0U; n < wire_count; n++) {
		assert(wire[n] == (uint8_t)n);
	}
}

/* T37: a controller that reports nothing costs frames, never the ring. */
static void tx_async_age_reclaim_test(bool defer_txd)
{
	init(defer_txd);
	/* Not one completion, ever. */
	tx_confirm = false;

	for (uint32_t n = 0U; n < 3U * TX_BD_COUNT; n++) {
		assert(queue_frame(1500U) == 0);
		assert(data.tx_bd_ring.free_bds <= TX_BD_COUNT);
	}

	/* The ring still works once the controller reports again. */
	tx_confirm = true;
	mmio[ETH_XLNX_GEM_ISR_OFFSET / 4] |= ETH_XLNX_GEM_IXR_TX_COMPLETE_BIT;
	eth_xlnx_gem_isr(&device);
	if (data.tx_done_work.pending) {
		data.tx_done_work.pending = false;
		eth_xlnx_gem_handle_tx_done(&device);
	}
	assert(data.tx_bd_ring.free_bds == TX_BD_COUNT);
	assert(queue_frame(1500U) == 0);
}

/*
 * The counters of panxxhub/zephyr#70. Every case provokes one event and states
 * both halves of the claim: the counter of that event moves by exactly the
 * number of times the event happened, and no other counter moves at all.
 * Deleting an increment from the driver is the reverse control for its case.
 */
static void assert_counters(uint32_t overruns, uint32_t bna, uint32_t resets,
			    uint32_t discards, uint32_t timeouts, uint32_t aged,
			    uint32_t ring_full)
{
	assert(data.diag.rx_overruns == overruns);
	assert(data.diag.rx_buffer_not_available == bna);
	assert(data.diag.rx_queue_resets == resets);
	assert(data.diag.rx_reset_discards == discards);
	assert(data.diag.tx_send_timeouts == timeouts);
	assert(data.diag.tx_age_reclaims == aged);
	assert(data.diag.tx_ring_full == ring_full);
}

/* Raise one interrupt indication and run the interrupt handler over it. */
static void raise_isr(uint32_t bits)
{
	mmio[ETH_XLNX_GEM_ISR_OFFSET / 4] |= bits;
	eth_xlnx_gem_isr(&device);
	/* Receive handling is deferred here, so the ISR is all that ran. */
	data.rx_pend_work.pending = false;
}

/* T40: an overrun indication is counted; an ordinary receive is not. */
static void rx_overrun_counter_test(void)
{
	init(true);
	raise_isr(ETH_XLNX_GEM_IXR_RX_OVERRUN_BIT);
	assert_counters(1U, 0U, 0U, 0U, 0U, 0U, 0U);

	raise_isr(ETH_XLNX_GEM_IXR_RX_OVERRUN_BIT);
	assert_counters(2U, 0U, 0U, 0U, 0U, 0U, 0U);

	/* Reverse control: the event that is not an overrun moves nothing. */
	init(true);
	raise_isr(ETH_XLNX_GEM_IXR_FRAME_RX_BIT);
	assert_counters(0U, 0U, 0U, 0U, 0U, 0U, 0U);
}

/* T41: a buffer-not-available indication is counted, and only that one. */
static void rx_bna_counter_test(void)
{
	init(true);
	raise_isr(ETH_XLNX_GEM_IXR_RX_USED_BIT);
	assert_counters(0U, 1U, 0U, 0U, 0U, 0U, 0U);

	/* Both at once: the controller reports two distinct conditions. */
	raise_isr(ETH_XLNX_GEM_IXR_RX_USED_BIT | ETH_XLNX_GEM_IXR_RX_OVERRUN_BIT);
	assert_counters(1U, 2U, 0U, 0U, 0U, 0U, 0U);

	/* Reverse control: a transmit completion moves nothing. */
	init(true);
	mmio[ETH_XLNX_GEM_ISR_OFFSET / 4] |= ETH_XLNX_GEM_IXR_TX_COMPLETE_BIT;
	eth_xlnx_gem_isr(&device);
	data.tx_done_work.pending = false;
	assert_counters(0U, 0U, 0U, 0U, 0U, 0U, 0U);
}

/*
 * Mark the first @p frames descriptors as holding the start of a received
 * frame the driver has not delivered, which is what a queue reset throws away.
 */
static void arm_received_frames(uint32_t frames)
{
	for (uint32_t i = 0U; i < frames; i++) {
		rx[i].addr |= ETH_XLNX_GEM_RX_BD_USED_BIT;
		rx[i].ctrl |= ETH_XLNX_GEM_RX_BD_START_OF_FRAME_BIT;
	}
}

/* T42: a queue reset is counted, and an empty ring discards nothing. */
static void rx_queue_reset_counter_test(void)
{
	init(true);
	/* A halted queue: the receive handler resets it without scanning. */
	mmio[ETH_XLNX_GEM_RXSR_OFFSET / 4] |= ETH_XLNX_GEM_RXSR_BNA_BIT;
	eth_xlnx_gem_handle_rx_pending(&device);
	assert_counters(0U, 0U, 1U, 0U, 0U, 0U, 0U);

	/* Reverse control: a quiet ring is not reset and counts nothing. */
	init(true);
	eth_xlnx_gem_handle_rx_pending(&device);
	assert_counters(0U, 0U, 0U, 0U, 0U, 0U, 0U);
}

/* T43: the frames a reset throws away are counted, one per frame. */
static void rx_reset_discard_counter_test(void)
{
	const uint32_t frames = 7U;

	init(true);
	arm_received_frames(frames);
	mmio[ETH_XLNX_GEM_RXSR_OFFSET / 4] |= ETH_XLNX_GEM_RXSR_OVERRUN_BIT;
	eth_xlnx_gem_handle_rx_pending(&device);
	assert_counters(0U, 0U, 1U, frames, 0U, 0U, 0U);

	/* The rebuilt ring holds nothing, so a second reset discards nothing. */
	mmio[ETH_XLNX_GEM_RXSR_OFFSET / 4] |= ETH_XLNX_GEM_RXSR_OVERRUN_BIT;
	eth_xlnx_gem_handle_rx_pending(&device);
	assert_counters(0U, 0U, 2U, frames, 0U, 0U, 0U);
}

/* T47: a transmission whose confirmation never arrives is counted. */
static void tx_send_timeout_counter_test(bool defer_txd)
{
	struct net_pkt pkt = {1500U, 0U, 0U};

	init(defer_txd);
	/* A confirmed transmission moves nothing. */
	assert(eth_xlnx_gem_send(&device, &pkt) == 0);
	assert_counters(0U, 0U, 0U, 0U, 0U, 0U, 0U);

	/* Not one confirmation: the send function gives up on the frame. */
	tx_confirm = false;
	assert(eth_xlnx_gem_send(&device, &pkt) != 0);
	assert_counters(0U, 0U, 0U, 0U, 1U, 0U, 0U);
	assert(eth_xlnx_gem_send(&device, &pkt) != 0);
	assert_counters(0U, 0U, 0U, 0U, 2U, 0U, 0U);
}

/* Queue frames until the ring holds no more free descriptors. */
static void fill_tx_ring(void)
{
	while (data.tx_bd_ring.free_bds > 0U) {
		assert(queue_frame(1500U) == 0);
	}
}

/* T44: only a back-pressure wait that a confirmation never ends is a timeout. */
static void tx_async_timeout_counter_test(void)
{
	init(false);
	tx_confirm = true;

	/* The ring fills without any sender ever having to wait. */
	fill_tx_ring();
	assert_counters(0U, 0U, 0U, 0U, 0U, 0U, 0U);

	/*
	 * Reverse control: this sender waits, but the controller confirms
	 * while it waits, so its wait is not a timeout.
	 */
	assert(queue_frame(1500U) == 0);
	assert_counters(0U, 0U, 0U, 0U, 0U, 0U, 1U);

	/* Nothing is confirmed from here on, so the next wait does time out. */
	fill_tx_ring();
	tx_confirm = false;
	assert(queue_frame(1500U) == 0);
	assert_counters(0U, 0U, 0U, 0U, 1U, 1U, 2U);
}

/* T45: descriptors returned to the ring by age are counted, one per frame. */
static void tx_age_reclaim_counter_test(void)
{
	const uint32_t extra = 5U;

	init(false);
	tx_confirm = true;
	fill_tx_ring();
	assert_counters(0U, 0U, 0U, 0U, 0U, 0U, 0U);

	/*
	 * Not one confirmation: every further frame finds the ring full, waits
	 * a whole timeout and abandons the transmission at the head of the ring
	 * to make room for itself.
	 */
	tx_confirm = false;
	for (uint32_t n = 0U; n < extra; n++) {
		assert(queue_frame(1500U) == 0);
	}
	assert_counters(0U, 0U, 0U, 0U, extra, extra, extra);
}

/* T46: a sender held back by a full ring is counted, once per wait. */
static void tx_ring_full_counter_test(void)
{
	init(false);
	tx_confirm = true;

	for (uint32_t round = 0U; round < 3U; round++) {
		/* Reverse control: a ring with room is never back pressure. */
		fill_tx_ring();
		assert_counters(0U, 0U, 0U, 0U, 0U, 0U, round);

		/*
		 * The ring is full: this sender waits, and the confirmation
		 * that arrives while it waits returns the whole ring to it.
		 */
		assert(queue_frame(1500U) == 0);
		assert_counters(0U, 0U, 0U, 0U, 0U, 0U, round + 1U);
	}
}

int main(int argc, char **argv)
{
	int test = (argc > 1) ? atoi(argv[1]) : 0;

	switch (test) {
	case 30:
		frame_lengths_test();
		break;
	case 31:
		tx_timeout_test(false);
		tx_timeout_test(true);
		break;
	case 32:
		tx_late_confirmation_test();
		break;
	case 33:
		tx_reentry_test();
		break;
	case 34:
		workq_test();
		break;
	case 35:
		tx_async_pipeline_test(false);
		tx_async_pipeline_test(true);
		break;
	case 36:
		tx_async_backpressure_test();
		break;
	case 37:
		tx_async_age_reclaim_test(false);
		tx_async_age_reclaim_test(true);
		break;
	case 40:
		rx_overrun_counter_test();
		break;
	case 41:
		rx_bna_counter_test();
		break;
	case 42:
		rx_queue_reset_counter_test();
		break;
	case 43:
		rx_reset_discard_counter_test();
		break;
	case 44:
		tx_async_timeout_counter_test();
		break;
	case 45:
		tx_age_reclaim_counter_test();
		break;
	case 46:
		tx_ring_full_counter_test();
		break;
	case 47:
		tx_send_timeout_counter_test(false);
		tx_send_timeout_counter_test(true);
		break;
	default:
		return 1;
	}
	printf("T%d ok\n", test);
	return 0;
}
