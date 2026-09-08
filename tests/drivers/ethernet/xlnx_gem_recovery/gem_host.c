/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define BIT(n)                 (1U << (n))
#define MIN(a, b)              ((a) < (b) ? (a) : (b))
#define DIV_ROUND_UP(a, b)     (((a) + (b) - 1) / (b))
#define CONTAINER_OF(p, t, m)  ((t *)((char *)(p) - offsetof(t, m)))
#define K_MSEC(n)              (n)
#define K_NO_WAIT              0
#define K_FOREVER              -1
#define NET_ETH_MAX_FRAME_SIZE 1518
#define NET_AF_UNSPEC          0
#define CONFIG_DCACHE          1
#define LOG_ERR(...)           log_errors++
#define LOG_DBG(...)           ((void)0)
#define __unused
struct k_spinlock {
	bool held;
};
typedef int k_spinlock_key_t;
struct k_mutex {
	bool held;
};
struct k_sem {
	int count;
};
struct k_work {
	int pending;
};
struct k_work_delayable {
	struct k_work work;
	uint64_t due;
};
struct net_if;
struct device {
	const char *name;
};
struct net_pkt {
	size_t len;
};
struct eth_xlnx_gem_bd {
	uint32_t addr, ctrl;
};
struct ring {
	struct eth_xlnx_gem_bd *first_bd;
	uint8_t next_to_process, next_to_use, free_bds;
};
/* DIAGNOSTICS */
struct eth_xlnx_gem_dev_cfg {
	uint8_t rx_bd_count, tx_bd_count, hw_rx_buffer_offset;
	uint16_t rx_buffer_size, tx_buffer_size;
	bool defer_txd_to_queue, defer_rxp_to_queue;
};
struct eth_xlnx_gem_dev_data {
	struct k_spinlock ring_lock;
	struct k_mutex send_lock;
	struct k_work_delayable recovery_work, rx_pend_work;
	struct k_work tx_done_work;
	struct k_sem tx_done_sem;
	struct eth_xlnx_gem_diagnostics diagnostics;
	struct ring rx_bd_ring, tx_bd_ring;
	bool started, recovering, recovery_failed;
	int tx_result;
	uint8_t *first_rx_buffer, *first_tx_buffer;
	void *iface;
};
static struct device device = {"fake GEM"};
static struct eth_xlnx_gem_dev_cfg cfg = {32, 32, 0, 64, 64, false, true};
static struct eth_xlnx_gem_dev_data data;
static struct eth_xlnx_gem_bd rx[255], tx[64];
static _Alignas(4096) uint8_t rxbuf[255 * 2048];
static _Alignas(4096) uint8_t txbuf[64 * 2048];
static uint32_t mmio[512];
static uint32_t log_errors, copies, pool_pkts, busy_waits;
static bool flood_active, bench_active;
static uint32_t bench_cost, bench_tail, bench_head;
static struct net_pkt bench_queue[512];
static void advance_bench(uint32_t us);
static void advance_flood(uint32_t us);
static uint32_t reads, deliveries, unrefs, allocations, clean_tx, clean_rx, barriers;
static uint32_t publications, clocks, speeds;
static bool allocation_failure, copy_failure, refill, auto_complete, stuck_stop, carrier;
static struct net_pkt packet = {100};
static struct eth_xlnx_gem_dev_cfg *get_cfg(void)
{
	return &cfg;
}
static struct eth_xlnx_gem_dev_data *get_data(void)
{
	return &data;
}
#define DEV_CFG(d)                  get_cfg()
#define DEV_DATA(d)                 get_data()
#define DEVICE_MMIO_NAMED_GET(d, n) ((uintptr_t)mmio)
#define DT_INST_FOREACH_STATUS_OKAY(m)
#define ETH_XLNX_GEM_INIT_BD_RING
/* MACROS */
static k_spinlock_key_t k_spin_lock(struct k_spinlock *lock)
{
	assert(!lock->held);
	lock->held = true;
	return 0;
}
static void k_spin_unlock(struct k_spinlock *lock, k_spinlock_key_t key)
{
	assert(lock->held);
	lock->held = false;
}
static int k_mutex_lock(struct k_mutex *lock, int timeout)
{
	assert(!lock->held);
	lock->held = true;
	return 0;
}
static void k_mutex_unlock(struct k_mutex *lock)
{
	lock->held = false;
}
static void k_sem_reset(struct k_sem *sem)
{
	sem->count = 0;
}
static void k_sem_give(struct k_sem *sem)
{
	sem->count = 1;
}
static int k_work_submit(struct k_work *work)
{
	work->pending = 1;
	return 0;
}
static uint64_t now_us;
static uint32_t k_cycle_get_32(void)
{
	return now_us * 1000;
}
static uint32_t k_cyc_to_us_floor32(uint32_t cycles)
{
	return cycles / 1000;
}
static int k_work_schedule(struct k_work_delayable *work, int delay)
{
	if (!work->work.pending) {
		work->work.pending = 1;
		work->due = now_us + delay * 1000;
	}
	return 0;
}
static struct k_work_delayable *k_work_delayable_from_work(struct k_work *work)
{
	return CONTAINER_OF(work, struct k_work_delayable, work);
}
static void k_busy_wait(uint32_t us)
{
	busy_waits++;
}
static const struct device *net_if_get_device(void *iface)
{
	return &device;
}
static void barrier_dmem_fence_full(void)
{
	barriers++;
}
static uint32_t sys_read32(uintptr_t addr)
{
	/* Hard guard converts an unbounded production walk into an assertion. */
	assert(++reads < 4096);
	return *(uint32_t *)addr;
}
static void sys_write32(uint32_t value, uintptr_t addr)
{
	if (addr == (uintptr_t)mmio + ETH_XLNX_GEM_IER_OFFSET) {
		mmio[ETH_XLNX_GEM_IMR_OFFSET / 4] &= ~value;
	} else if (addr == (uintptr_t)mmio + ETH_XLNX_GEM_IDR_OFFSET) {
		mmio[ETH_XLNX_GEM_IMR_OFFSET / 4] |= value;
	}
	if (addr >= (uintptr_t)rx && addr < (uintptr_t)&rx[255] &&
	    (addr - (uintptr_t)rx) % sizeof(rx[0]) == 0 && (value & 1) == 0) {
		assert(clean_rx > 0 && barriers > 0);
	}
	if (addr >= (uintptr_t)&tx[0] && addr < (uintptr_t)&tx[64] &&
	    (addr - (uintptr_t)&tx[0]) % sizeof(tx[0]) == offsetof(struct eth_xlnx_gem_bd, ctrl) &&
	    (value & ETH_XLNX_GEM_TX_BD_USED_BIT) == 0U) {
		assert(clean_tx >= DIV_ROUND_UP(packet.len, cfg.tx_buffer_size));
		assert(barriers > 0);
		uint32_t idx = (addr - (uintptr_t)tx) / sizeof(tx[0]);
		uint32_t count = DIV_ROUND_UP(packet.len, cfg.tx_buffer_size);

		assert(idx == (data.tx_bd_ring.next_to_process + count - publications - 1) %
				      cfg.tx_bd_count);
		publications++;
	}
	if (addr == (uintptr_t)mmio + ETH_XLNX_GEM_ISR_OFFSET ||
	    addr == (uintptr_t)mmio + ETH_XLNX_GEM_TXSR_OFFSET ||
	    addr == (uintptr_t)mmio + ETH_XLNX_GEM_RXSR_OFFSET) {
		*(uint32_t *)addr &= ~value;
		return;
	}
	if (addr == (uintptr_t)mmio + ETH_XLNX_GEM_NWCTRL_OFFSET &&
	    (value & ETH_XLNX_GEM_NWCTRL_TXEN_BIT) == 0 && !stuck_stop) {
		mmio[ETH_XLNX_GEM_TXSR_OFFSET / 4] &= ~BIT(3);
	}
	*(uint32_t *)addr = value;
}
static void sys_cache_data_flush_and_invd_range(void *buf, size_t size)
{
	if ((uintptr_t)buf >= (uintptr_t)txbuf &&
	    (uintptr_t)buf < (uintptr_t)(txbuf + sizeof(txbuf))) {
		clean_tx++;
	} else {
		assert(buf == rxbuf && size == cfg.rx_bd_count * cfg.rx_buffer_size);
		clean_rx++;
	}
}
static void sys_cache_data_invd_range(void *buf, size_t size)
{
}
static size_t net_pkt_get_len(struct net_pkt *pkt)
{
	return pkt->len;
}
static void net_pkt_cursor_init(struct net_pkt *pkt)
{
}
static int net_pkt_read(struct net_pkt *pkt, void *buf, size_t size)
{
	memset(buf, 0x5a, size);
	return 0;
}
static struct net_pkt *net_pkt_rx_alloc_with_buffer(void *iface, uint32_t n, int f, int p, int t)
{
	allocations++;
	if (bench_active) {
		packet.len = n;
		advance_bench(bench_cost / 3);
		assert(bench_tail - bench_head < 512);
	}
	if (flood_active) {
		advance_flood(600);
		if (pool_pkts == 16) {
			return NULL;
		}
		if (!allocation_failure) {
			pool_pkts++;
		}
	}
	return allocation_failure ? NULL : &packet;
}
static int net_pkt_write(struct net_pkt *pkt, const void *buf, size_t size)
{
	assert(!allocation_failure);
	copies++;
	if (bench_active) {
		advance_bench(bench_cost / 3);
	}
	return copy_failure ? -ENOBUFS : 0;
}
static void net_pkt_unref(struct net_pkt *pkt)
{
	unrefs++;
}
static int net_recv_data(void *iface, struct net_pkt *pkt)
{
	assert(!data.ring_lock.held);
	deliveries++;
	if (bench_active) {
		bench_queue[bench_tail++ % 512] = *pkt;
		advance_bench(bench_cost - 2 * (bench_cost / 3));
	}
	if (refill) {
		for (uint32_t n = 0; n < 32; n++) {
			rx[n].addr |= 1;
			rx[n].ctrl = ETH_XLNX_GEM_RX_BD_START_OF_FRAME_BIT |
				     ETH_XLNX_GEM_RX_BD_END_OF_FRAME_BIT | 60;
		}
	}
	return 0;
}
static void eth_xlnx_gem_handle_tx_done(const struct device *dev);
static void eth_xlnx_gem_request_recovery(const struct device *dev, uint32_t reason);
static void eth_xlnx_gem_isr(const struct device *dev);
static int k_sem_take(struct k_sem *sem, int timeout)
{
	if (auto_complete) {
		tx[data.tx_bd_ring.next_to_process].ctrl |= ETH_XLNX_GEM_TX_BD_USED_BIT;
		if (bench_active) {
			mmio[ETH_XLNX_GEM_ISR_OFFSET / 4] |= ETH_XLNX_GEM_IXR_TX_COMPLETE_BIT;
			eth_xlnx_gem_isr(&device);
		} else {
			eth_xlnx_gem_handle_tx_done(&device);
		}
	}
	if (sem->count != 0) {
		sem->count = 0;
		return 0;
	}
	return -EAGAIN;
}
struct phy_link_state {
	bool is_up;
	int speed;
};
enum {
	LINK_HALF_10BASE = 1,
	LINK_FULL_10BASE = 2,
	LINK_HALF_100BASE = 4,
	LINK_FULL_100BASE = 8,
	LINK_HALF_1000BASE = 16,
	LINK_FULL_1000BASE = 32
};
static void eth_xlnx_gem_configure_clocks(const struct device *dev, struct phy_link_state *s)
{
	clocks++;
}
static void eth_xlnx_gem_set_nwcfg_link_speed(const struct device *dev, struct phy_link_state *s)
{
	speeds++;
}
static void net_eth_carrier_set(void *iface, bool up)
{
	carrier = up;
}
/* FUNCTIONS */
static void init(void)
{
	memset(&data, 0, sizeof(data));
	memset(mmio, 0, sizeof(mmio));
	memset(rx, 0, sizeof(rx));
	memset(tx, 0, sizeof(tx));
	data.rx_bd_ring.first_bd = rx;
	data.tx_bd_ring.first_bd = tx;
	data.first_rx_buffer = rxbuf;
	data.first_tx_buffer = txbuf;
	data.started = true;
	data.iface = &device;
	reads = deliveries = allocations = unrefs = clean_tx = clean_rx = barriers = publications =
		0;
	allocation_failure = copy_failure = refill = auto_complete = stuck_stop = false;
	packet.len = 100;
	eth_xlnx_gem_configure_buffers(&device);
	assert(clean_rx == 1 && barriers > 0);
	mmio[ETH_XLNX_GEM_NWCTRL_OFFSET / 4] = ETH_XLNX_GEM_NWCTRL_MDEN_BIT |
					       ETH_XLNX_GEM_NWCTRL_RXEN_BIT |
					       ETH_XLNX_GEM_NWCTRL_TXEN_BIT;
}
static void good_rx(uint32_t idx)
{
	rx[idx].addr |= 1;
	rx[idx].ctrl =
		ETH_XLNX_GEM_RX_BD_START_OF_FRAME_BIT | ETH_XLNX_GEM_RX_BD_END_OF_FRAME_BIT | 60;
}
static void recover(void)
{
	assert(data.recovery_work.work.pending);
	data.recovery_work.work.pending = 0;
	eth_xlnx_gem_recovery_work(&data.recovery_work.work);
	assert(!data.recovering && !data.recovery_failed);
	assert(data.tx_bd_ring.free_bds == cfg.tx_bd_count && data.tx_bd_ring.next_to_process == 0);
	assert(mmio[ETH_XLNX_GEM_RXQBASE_OFFSET / 4] == (uint32_t)(uintptr_t)rx);
	assert(mmio[ETH_XLNX_GEM_TXQBASE_OFFSET / 4] == (uint32_t)(uintptr_t)tx);
	assert(mmio[ETH_XLNX_GEM_NWCTRL_OFFSET / 4] & ETH_XLNX_GEM_NWCTRL_MDEN_BIT);
}
static void test_t1(void)
{
	init();
	data.rx_bd_ring.next_to_process = 31;
	rx[31].addr |= 1;
	rx[31].ctrl = ETH_XLNX_GEM_RX_BD_END_OF_FRAME_BIT | 60;
	good_rx(0);
	eth_xlnx_gem_handle_rx_pending(&device);
	assert(deliveries == 1 && !(rx[31].addr & 1) && (rx[31].addr & 2));
	assert(data.rx_bd_ring.next_to_process == 1);
	uint32_t faults[] = {ETH_XLNX_GEM_IXR_HRESP_NOT_OK_BIT, ETH_XLNX_GEM_IXR_TX_UNDERRUN_BIT};

	for (uint32_t n = 0; n < 2; n++) {
		init();
		mmio[ETH_XLNX_GEM_ISR_OFFSET / 4] = faults[n];
		mmio[ETH_XLNX_GEM_RXSR_OFFSET / 4] = 0x5;
		good_rx(0);
		eth_xlnx_gem_isr(&device);
		assert(data.recovering && data.diagnostics.isr == faults[n]);
		assert(data.diagnostics.rxsr == 5 && (data.diagnostics.rx_head_addr & 1));
		recover();
		good_rx(0);
		eth_xlnx_gem_handle_rx_pending(&device);
		assert(deliveries == 1);
	}
}
static void test_t3(void)
{
	init();
	for (uint32_t n = 0; n < 32; n++) {
		rx[n].addr |= 1;
	}
	rx[0].ctrl = ETH_XLNX_GEM_RX_BD_START_OF_FRAME_BIT;
	eth_xlnx_gem_handle_rx_pending(&device);
	assert(data.recovering && reads < 200);
	recover();
	init();
	rx[0].addr |= 1;
	rx[0].ctrl = ETH_XLNX_GEM_RX_BD_START_OF_FRAME_BIT;
	rx[1].ctrl = ETH_XLNX_GEM_RX_BD_END_OF_FRAME_BIT | 100;
	eth_xlnx_gem_handle_rx_pending(&device);
	assert(allocations == 0 && (rx[0].addr & 1) && !data.recovering);
	rx[1].addr |= 1;
	eth_xlnx_gem_handle_rx_pending(&device);
	assert(deliveries == 1 && !(rx[0].addr & 1) && !(rx[1].addr & 1));
	init();
	good_rx(0);
	rx[0].ctrl =
		ETH_XLNX_GEM_RX_BD_START_OF_FRAME_BIT | ETH_XLNX_GEM_RX_BD_END_OF_FRAME_BIT | 100;
	eth_xlnx_gem_handle_rx_pending(&device);
	assert(data.recovering && allocations == 0);
	init();
	good_rx(0);
	allocation_failure = true;
	eth_xlnx_gem_handle_rx_pending(&device);
	assert(!(rx[0].addr & 1) && deliveries == 0);
	init();
	good_rx(0);
	copy_failure = true;
	eth_xlnx_gem_handle_rx_pending(&device);
	assert(!(rx[0].addr & 1) && deliveries == 0 && unrefs == 1);
	init();
	good_rx(0);
	refill = true;
	eth_xlnx_gem_handle_rx_pending(&device);
	/* Returning permits the peer work item to run before the submitted continuation. */
	assert(deliveries == 8 && data.rx_pend_work.work.pending && reads < 600);
}
static void test_t2(void)
{
	init();
	for (uint32_t n = 0; n < 32; n++) {
		reads = clean_tx = publications = 0;
		assert(eth_xlnx_gem_send(&device, &packet) == -EIO);
		assert(data.recovering && data.tx_bd_ring.free_bds == 30);
		assert(eth_xlnx_gem_send(&device, &packet) == -EIO);
		recover();
		eth_xlnx_gem_handle_tx_done(&device);
		assert(data.tx_bd_ring.free_bds == 32 && data.tx_done_sem.count == 0);
	}
	reads = clean_tx = publications = 0;
	auto_complete = true;
	assert(eth_xlnx_gem_send(&device, &packet) == 0);
	assert(data.tx_bd_ring.free_bds == 32);
	eth_xlnx_gem_handle_tx_done(&device);
	assert(data.tx_done_sem.count == 0 && data.tx_bd_ring.free_bds == 32);
	/* Coalesced completions, then a DMA-owned first BD: only completed frames retire. */
	init();
	data.tx_bd_ring.free_bds = 29;
	tx[0].ctrl = tx[1].ctrl = ETH_XLNX_GEM_TX_BD_USED_BIT | ETH_XLNX_GEM_TX_BD_LAST_BIT;
	tx[2].ctrl = ETH_XLNX_GEM_TX_BD_LAST_BIT;
	eth_xlnx_gem_handle_tx_done(&device);
	assert(data.tx_bd_ring.free_bds == 31 && data.tx_done_sem.count == 0);
	tx[2].ctrl |= ETH_XLNX_GEM_TX_BD_USED_BIT;
	eth_xlnx_gem_handle_tx_done(&device);
	assert(data.tx_bd_ring.free_bds == 32 && data.tx_done_sem.count == 1);
	init();
	data.tx_bd_ring.free_bds = 31;
	tx[0].ctrl = ETH_XLNX_GEM_TX_BD_USED_BIT | ETH_XLNX_GEM_TX_BD_LAST_BIT |
		     ETH_XLNX_GEM_TX_BD_RETRY_BIT;
	eth_xlnx_gem_handle_tx_done(&device);
	assert(data.recovering && data.tx_bd_ring.free_bds == 31);
	recover();
	/* Refuse to reclaim a controller that fails to stop. */
	init();
	clean_tx = publications = 0;
	assert(eth_xlnx_gem_send(&device, &packet) == -EIO);
	stuck_stop = true;
	mmio[ETH_XLNX_GEM_TXSR_OFFSET / 4] |= BIT(3);
	eth_xlnx_gem_recovery_work(&data.recovery_work.work);
	assert(data.recovery_failed && data.tx_bd_ring.free_bds == 30);
}
static void test_t7(void)
{
	init();
	data.tx_bd_ring.next_to_use = data.tx_bd_ring.next_to_process = 31;
	auto_complete = true;
	assert(eth_xlnx_gem_send(&device, &packet) == 0);
	assert(publications == 2 && clean_tx == 2 && data.tx_bd_ring.free_bds == 32);
}
static void test_mac(void)
{
	init();
	struct phy_link_state state = {true, 0};

	clocks = speeds = 0;
	eth_xlnx_gem_phy_cb(NULL, &state, &device);
	assert(!carrier && clocks == 0 && speeds == 0);
	state.speed = LINK_FULL_100BASE;
	eth_xlnx_gem_phy_cb(NULL, &state, &device);
	assert(carrier && clocks == 1 && speeds == 1);
	state.speed = LINK_FULL_1000BASE;
	eth_xlnx_gem_phy_cb(NULL, &state, &device);
	assert(carrier && clocks == 2 && speeds == 2);
	state.is_up = false;
	eth_xlnx_gem_phy_cb(NULL, &state, &device);
	assert(!carrier && clocks == 2);
}
/* Simulated hardware runs even while CPU0 is executing a cooperative handler. */
static uint64_t next_arrival;
static uint32_t dma_next, arrivals, overruns, pause_us;
static void advance_flood(uint32_t us)
{
	now_us += us;
	while (flood_active && next_arrival <= now_us && next_arrival < 60000000) {
		next_arrival += 400; /* 500 ICMP + 2000 UDP frames/s. */
		arrivals++;
		if (rx[dma_next].addr & ETH_XLNX_GEM_RX_BD_USED_BIT) {
			overruns++;
		} else {
			rx[dma_next].addr |= ETH_XLNX_GEM_RX_BD_USED_BIT;
			rx[dma_next].ctrl = ETH_XLNX_GEM_RX_BD_START_OF_FRAME_BIT |
				ETH_XLNX_GEM_RX_BD_END_OF_FRAME_BIT |
				(arrivals % 5 == 0 ? 98 : 1242);
			dma_next = (dma_next + 1) % 32;
		}
	}
}
static void test_trace(void)
{
	uint32_t heads[] = {11, 14};

	for (uint32_t n = 0; n < 2; n++) {
		init();
		mmio[ETH_XLNX_GEM_ISR_OFFSET / 4] = 0x400;
		reads = 0;
		eth_xlnx_gem_isr(&device);
		assert(!data.recovering);
		data.rx_bd_ring.next_to_process = heads[n];
		rx[heads[n]].addr |= 1;
		rx[heads[n]].ctrl = ETH_XLNX_GEM_RX_BD_END_OF_FRAME_BIT | 1242;
		good_rx(heads[n] + 1);
		allocation_failure = true;
		eth_xlnx_gem_handle_rx_pending(&device);
		assert(data.diagnostics.rx_orphans == 1);
		assert(data.rx_bd_ring.next_to_process == heads[n] + 2);
		assert(!(rx[heads[n]].addr & 1) && !(rx[heads[n] + 1].addr & 1));
		assert(data.diagnostics.rx_dropped == 1);
		assert(mmio[ETH_XLNX_GEM_IER_OFFSET / 4] & ETH_XLNX_GEM_IXR_FRAME_RX_BIT);
		mmio[ETH_XLNX_GEM_ISR_OFFSET / 4] = 0x404;
		eth_xlnx_gem_isr(&device);
		assert(!data.recovering && !data.recovery_work.work.pending);
		assert(data.diagnostics.rx_used == 1 && data.diagnostics.rx_overruns == 2);
		eth_xlnx_gem_handle_rx_pending(&device);
		for (uint32_t i = 0; i < 32; i++) {
			assert(!(rx[i].addr & 1));
		}
		assert(mmio[ETH_XLNX_GEM_IER_OFFSET / 4] & ETH_XLNX_GEM_IXR_FRAME_RX_BIT);
		allocation_failure = false;
		good_rx(data.rx_bd_ring.next_to_process);
		eth_xlnx_gem_handle_rx_pending(&device);
		assert(deliveries == 1);
	}
}
static void test_flood(void)
{
	cfg.rx_buffer_size = cfg.tx_buffer_size = 1536;
	init();
	now_us = next_arrival = dma_next = arrivals = overruns = pause_us = 0;
	copies = log_errors = pool_pkts = busy_waits = 0;
	flood_active = true;
	uint64_t heartbeat_due = 0, last_heartbeat = 0;
	uint32_t heartbeats = 0, invocations = 0;

	/* CPU0: workqueue and CoAP are cooperative -1. A workqueue yield may
	 * dispatch a ready CoAP thread, but only blocking guarantees CPU slack.
	 * CoAP wakes every 10 ms; pool exhaustion alternates every 100 ms.
	 * 600 us/frame is injected CPU cost, deliberately slower than arrivals.
	 */
	while (now_us < 60000000 || data.rx_pend_work.work.pending) {
		if (now_us >= heartbeat_due) {
			assert(now_us - last_heartbeat < 18000);
			last_heartbeat = now_us;
			heartbeat_due = now_us + 10000;
			heartbeats++;
			pool_pkts = 0;
		}
		if (!data.rx_pend_work.work.pending && now_us < 60000000 &&
		    (rx[data.rx_bd_ring.next_to_process].addr & 1)) {
			mmio[ETH_XLNX_GEM_ISR_OFFSET / 4] = ETH_XLNX_GEM_IXR_FRAME_RX_BIT;
			reads = 0;
			eth_xlnx_gem_isr(&device);
		}
		if (data.rx_pend_work.work.pending && data.rx_pend_work.due <= now_us) {
			uint32_t before = allocations, copied = copies;
			uint32_t dropped = data.diagnostics.rx_dropped;

			allocation_failure = (now_us / 100000) % 2 == 0;
			data.rx_pend_work.work.pending = 0;
			reads = 0;
			eth_xlnx_gem_handle_rx_pending(&device);
			assert(allocations - before <= 8);
			if (allocation_failure) {
				assert(copies == copied);
			}
			assert(!data.recovering);
			invocations++;
			/* This is the queue's normal k_yield(), not an invented sleep. */
			if (data.rx_pend_work.work.pending) {
				assert(data.rx_pend_work.due ==
				       now_us + (data.diagnostics.rx_dropped > dropped ? 2000 : 0));
				assert(mmio[ETH_XLNX_GEM_IMR_OFFSET / 4] &
				       ETH_XLNX_GEM_IXR_FRAME_RX_BIT);
			}
		} else {
			pause_us += 100;
			advance_flood(100);
		}
		assert(invocations < 20000);
	}
	flood_active = false;
	assert(heartbeats > 3000 && arrivals == 150000 && overruns > 0);
	assert(pause_us > 0 && data.diagnostics.rx_dropped > 0);
	assert(copies > 0 && log_errors == 0 && busy_waits == 0);
	assert(mmio[ETH_XLNX_GEM_IER_OFFSET / 4] & ETH_XLNX_GEM_IXR_FRAME_RX_BIT);
	allocation_failure = false;
	reads = 0;
	uint32_t before = deliveries;

	good_rx(data.rx_bd_ring.next_to_process);
	eth_xlnx_gem_handle_rx_pending(&device);
	assert(deliveries == before + 1);
}
static void test_recovery_flood(void)
{
	cfg.rx_buffer_size = cfg.tx_buffer_size = 1536;
	init();
	now_us = 0;
	busy_waits = 0;
	uint32_t heartbeats = 0;
	uint64_t heartbeat_due = 0, last_heartbeat = 0;

	/* Genuine bus faults still use delayed recovery and never busy-wait. */
	while (now_us < 60000000) {
		if (now_us >= heartbeat_due) {
			assert(now_us - last_heartbeat <= 11000);
			last_heartbeat = now_us;
			heartbeat_due = now_us + 10000;
			heartbeats++;
		}
		reads = 0;
		mmio[ETH_XLNX_GEM_ISR_OFFSET / 4] = ETH_XLNX_GEM_IXR_HRESP_NOT_OK_BIT;
		eth_xlnx_gem_isr(&device);
		assert(data.recovery_work.due >= now_us + 2000);
		assert(mmio[ETH_XLNX_GEM_IMR_OFFSET / 4] == ETH_XLNX_GEM_IXR_ALL_MASK);
		now_us = data.recovery_work.due; /* Block, never spin until this deadline. */
		recover();
		now_us += 100; /* Inject finite reset/cache-maintenance cost. */
		assert(busy_waits == 0);
	}
	assert(heartbeats > 5000);
	reads = 0;
	good_rx(0);
	eth_xlnx_gem_handle_rx_pending(&device);
	assert(deliveries == 1);
}
/* Separate periodic senders: UDP 1800/s, ICMP 500/s, liveness ICMP 1/s.
 * CPU costs are injected, not measurements of Cortex-A9 execution. DMA sets
 * real ISR/RXSR pressure bits when it reaches a CPU-owned descriptor.
 */
static uint32_t bench_sent[3], bench_dma, bench_drops;
static const uint32_t bench_rates[] = {1800, 500, 1};
static const uint32_t bench_lengths[] = {1242, 98, 99};
static void advance_bench(uint32_t us)
{
	now_us += us;
	while (true) {
		uint32_t sender = 0;
		uint64_t next = UINT64_MAX;

		for (uint32_t n = 0; n < 3; n++) {
			uint64_t due = (uint64_t)bench_sent[n] * 1000000 / bench_rates[n];

			if (due < next) {
				next = due;
				sender = n;
			}
		}
		if (next > now_us || next >= 60000000) {
			break;
		}
		bench_sent[sender]++;
		if (!(mmio[ETH_XLNX_GEM_NWCTRL_OFFSET / 4] & ETH_XLNX_GEM_NWCTRL_RXEN_BIT) ||
		    (rx[bench_dma].addr & ETH_XLNX_GEM_RX_BD_USED_BIT)) {
			bench_drops++;
			mmio[ETH_XLNX_GEM_ISR_OFFSET / 4] |= 0x404;
			mmio[ETH_XLNX_GEM_RXSR_OFFSET / 4] |= 5;
		} else {
			rx[bench_dma].addr |= ETH_XLNX_GEM_RX_BD_USED_BIT;
			rx[bench_dma].ctrl = ETH_XLNX_GEM_RX_BD_START_OF_FRAME_BIT |
					     ETH_XLNX_GEM_RX_BD_END_OF_FRAME_BIT |
					     bench_lengths[sender];
			bench_dma = (bench_dma + 1) % cfg.rx_bd_count;
			mmio[ETH_XLNX_GEM_ISR_OFFSET / 4] |= ETH_XLNX_GEM_IXR_FRAME_RX_BIT;
			mmio[ETH_XLNX_GEM_RXSR_OFFSET / 4] |= BIT(1);
		}
	}
}
static void test_bench(void)
{
	/* Include allocator, copy and net_recv_data cost; ready RX/CoAP peers run
	 * at each workqueue yield. 250/350 us bracket the old pacing ceiling.
	 */
	for (bench_cost = 250; bench_cost <= 350; bench_cost += 100) {
		cfg.rx_bd_count = 255;
		cfg.tx_bd_count = 64;
		cfg.rx_buffer_size = cfg.tx_buffer_size = 2048;
		init();
		now_us = bench_head = bench_tail = bench_dma = bench_drops = 0;
		memset(bench_sent, 0, sizeof(bench_sent));
		bench_active = auto_complete = true;
		uint32_t replies = 0, probes = 0, heartbeats = 0;
		uint64_t last_heartbeat = 0, max_batch = 0;

		while (now_us < 60100000) {
			reads = 0;
			if (mmio[ETH_XLNX_GEM_ISR_OFFSET / 4] &
			    ~mmio[ETH_XLNX_GEM_IMR_OFFSET / 4]) {
				eth_xlnx_gem_isr(&device);
			}
			if (data.recovery_work.work.pending && data.recovery_work.due <= now_us) {
				recover();
				bench_dma = 0;
			}
			if (data.rx_pend_work.work.pending && data.rx_pend_work.due <= now_us) {
				uint64_t start = now_us;

				data.rx_pend_work.work.pending = 0;
				eth_xlnx_gem_handle_rx_pending(&device);
				if (now_us - start > max_batch) {
					max_batch = now_us - start;
				}
			}
			/* Normal workqueue yield lets equal-priority ready peers run. */
			assert(now_us - last_heartbeat < 10000);
			if (now_us - last_heartbeat >= 5000) {
				last_heartbeat = now_us;
				heartbeats++;
			}
			while (bench_head != bench_tail) {
				packet = bench_queue[bench_head++ % 512];
				/* Inject stack/service cost, including UDP discard. */
				advance_bench(25);
				if (packet.len < 100) {
					reads = clean_tx = publications = 0;
					if (eth_xlnx_gem_send(&device, &packet) == 0) {
						replies += packet.len == 98;
						probes += packet.len == 99;
					}
				}
			}
			advance_bench(10);
		}
		bench_active = false;
		printf("T13 cost=%u us: probes=%u/60 ICMP=%u/30000 drops=%u resets=%u "
		       "max_batch=%llu us heartbeat=%u\n",
		       bench_cost, probes, replies, bench_drops, data.diagnostics.recoveries,
		       (unsigned long long)max_batch, heartbeats);
		fflush(stdout);
		assert(bench_sent[0] == 108000 && bench_sent[1] == 30000 && bench_sent[2] == 60);
		assert(probes == 60 && replies == 30000 && bench_drops == 0);
		assert(data.diagnostics.recoveries == 0 && heartbeats > 6000);
		assert(max_batch <= 8 * bench_cost);
		assert(data.diagnostics.rx_work_max_us == max_batch);
		assert(!(mmio[ETH_XLNX_GEM_IMR_OFFSET / 4] & ETH_XLNX_GEM_IXR_FRAME_RX_BIT));
	}
	/* Coalesced RX pressure must not abort a successfully completed TX. */
	reads = clean_tx = publications = 0;
	packet.len = 98;
	bench_active = true;
	mmio[ETH_XLNX_GEM_ISR_OFFSET / 4] = 0x404;
	mmio[ETH_XLNX_GEM_RXSR_OFFSET / 4] = 5;
	assert(eth_xlnx_gem_send(&device, &packet) == 0);
	bench_active = false;
	assert(!data.recovering && data.tx_bd_ring.free_bds == 64);
	reads = 0;
	eth_xlnx_gem_handle_rx_pending(&device);
	assert(!(mmio[ETH_XLNX_GEM_IMR_OFFSET / 4] & 0x406));
	assert(!(mmio[ETH_XLNX_GEM_RXSR_OFFSET / 4] & 5));
	/* A TX IRQ may expose raw RX status while an RX batch is running.
	 * It must not queue immediate work that overrides the batch's backoff.
	 */
	data.rx_pend_work.work.pending = 0;
	sys_write32(GEM_RX_IRQS, (uintptr_t)mmio + ETH_XLNX_GEM_IDR_OFFSET);
	mmio[ETH_XLNX_GEM_ISR_OFFSET / 4] = ETH_XLNX_GEM_IXR_FRAME_RX_BIT |
		ETH_XLNX_GEM_IXR_TX_COMPLETE_BIT;
	reads = 0;
	eth_xlnx_gem_isr(&device);
	assert(!data.rx_pend_work.work.pending);
}
static void test_h2_ring(void)
{
	cfg.rx_buffer_size = cfg.tx_buffer_size = 1536;
	init();
	data.started = false;
	mmio[ETH_XLNX_GEM_NWCTRL_OFFSET / 4] = ETH_XLNX_GEM_NWCTRL_MDEN_BIT;
	for (uint32_t n = 0; n < 32; n++) {
		assert((rx[n].addr & ETH_XLNX_GEM_RX_BD_USED_BIT) == 0);
		assert((rx[n].addr & ETH_XLNX_GEM_RX_BD_BUFFER_ADDR_MASK) ==
		       (uintptr_t)rxbuf + n * 1536);
	}
	assert(eth_xlnx_gem_start_device(&device, NULL) == 0);
	eth_xlnx_gem_isr(&device);
	assert(!data.recovering && data.recovery_work.work.pending == 0);
	assert(mmio[ETH_XLNX_GEM_NWCTRL_OFFSET / 4] & ETH_XLNX_GEM_NWCTRL_RXEN_BIT);
	good_rx(0);
	eth_xlnx_gem_handle_rx_pending(&device);
	assert(deliveries == 1);
	auto_complete = true;
	assert(eth_xlnx_gem_send(&device, &packet) == 0);
}

int main(int argc, char **argv)
{
	assert(argc == 2);
	switch (atoi(argv[1])) {
	case 1:
		test_t1();
		break;
	case 2:
		test_t2();
		break;
	case 3:
		test_t3();
		break;
	case 6:
		test_mac();
		break;
	case 13:
		test_bench();
		break;
	case 11:
		test_recovery_flood();
		break;
	case 9:
		test_flood();
		break;
	case 10:
		test_trace();
		break;
	case 8:
		test_h2_ring();
		break;
	case 7:
		test_t7();
		break;
	default:
		abort();
	}
	return 0;
}
