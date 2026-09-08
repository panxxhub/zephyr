/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define BIT(n)                      (1U << (n))
#define K_FOREVER                   -1
#define K_NO_WAIT                   0
#define K_MSEC(n)                   (n)
#define CONFIG_DCACHE               1
#define NET_AF_UNSPEC               0
#define LOG_ERR_RATELIMIT_RATE(rate, ...) model_log(__VA_ARGS__)
#define LOG_WRN_RATELIMIT_RATE(...) ((void)0)
#define LOG_DBG(...)                ((void)0)
#define DT_INST_FOREACH_STATUS_OKAY(m)
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
struct k_work {
	bool pending;
};
struct net_pkt {
	uint32_t len;
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
struct eth_xlnx_gem_dev_data {
	struct k_spinlock nwcfg_lock;
	struct ring rx_bd_ring, tx_bd_ring;
	uint8_t *first_rx_buffer, *first_tx_buffer;
	void *iface;
	struct k_sem tx_done_sem;
	struct k_work rx_pend_work, tx_done_work;
};
struct device {
	const char *name;
};
static struct device device = {"GEM0"};
static struct eth_xlnx_gem_dev_cfg cfg;
static struct eth_xlnx_gem_dev_data data;
static struct eth_xlnx_gem_bd rx[255], tx[64];
static _Alignas(4096) uint8_t rxbuf[255 * 2048];
static _Alignas(4096) uint8_t txbuf[64 * 2048];
static uint32_t mmio[512];
#define DEV_CFG(dev)                     (&cfg)
#define DEV_DATA(dev)                    (&data)
#define DEVICE_MMIO_NAMED_GET(dev, name) ((uintptr_t)mmio)
/* MACROS */
static uint32_t resync_logs, resync_skipped, status_reads, rx_control_writes;
static bool dma_blocked, ring_dirty;
static uint32_t queue_base_writes;
static void model_log(const char *format, ...)
{
	if (strstr(format, "RX resync skipped") != NULL) {
		va_list args;

		va_start(args, format);
		(void)va_arg(args, const char *);
		resync_skipped += va_arg(args, unsigned int);
		va_end(args);
		resync_logs++;
	}
}
static bool burst, empty_pool;
static uint64_t now_us;
static uint32_t cpu_cost, sent[3], dma_next, drops, delivered, copied, reads;
static uint32_t queue_head, queue_tail;
static struct net_pkt packet, queue[512];
static const uint32_t rates[] = {1800, 500, 1};
static const uint32_t lengths[] = {1242, 98, 99};
static bool dma_receive_bd(uint32_t ctrl);
static void advance(uint32_t us)
{
	now_us += us;
	while (burst) {
		uint64_t next = UINT64_MAX;
		uint32_t sender = 0;

		for (uint32_t n = 0; n < 3; n++) {
			uint64_t due = (uint64_t)sent[n] * 1000000 / rates[n];

			if (due < next) {
				next = due;
				sender = n;
			}
		}
		if (next > now_us || next >= 60000000) {
			break;
		}
		sent[sender]++;
		(void)dma_receive_bd(ETH_XLNX_GEM_RX_BD_START_OF_FRAME_BIT |
				     ETH_XLNX_GEM_RX_BD_END_OF_FRAME_BIT | lengths[sender]);
	}
}
static uint32_t sys_read32(uintptr_t addr)
{
	assert(++reads < 100000);
	if (addr == (uintptr_t)mmio + ETH_XLNX_GEM_RXSR_OFFSET) {
		status_reads++;
	}
	return *(uint32_t *)addr;
}
static void sys_write32(uint32_t value, uintptr_t addr)
{
	if (addr == (uintptr_t)mmio + ETH_XLNX_GEM_NWCTRL_OFFSET) {
		rx_control_writes++;
	}
	if (addr >= (uintptr_t)rx && addr < (uintptr_t)(rx + cfg.rx_bd_count) &&
	    data.nwcfg_lock.held) {
		assert((mmio[ETH_XLNX_GEM_NWCTRL_OFFSET / 4] & ETH_XLNX_GEM_NWCTRL_RXEN_BIT) == 0U);
		ring_dirty = true;
	}
	if (addr == (uintptr_t)mmio + ETH_XLNX_GEM_RXQBASE_OFFSET) {
		/* Hardware ignores the queue-base write while RX is enabled. */
		if ((mmio[ETH_XLNX_GEM_NWCTRL_OFFSET / 4] & ETH_XLNX_GEM_NWCTRL_RXEN_BIT) != 0U) {
			return;
		}
		assert(value == (uint32_t)(uintptr_t)rx && !ring_dirty);
		for (uint32_t i = 0U; i < cfg.rx_bd_count; i++) {
			uint32_t expected = (uint32_t)(uintptr_t)rxbuf + i * cfg.rx_buffer_size;

			if (i + 1U == cfg.rx_bd_count) {
				expected |= ETH_XLNX_GEM_RX_BD_WRAP_BIT;
			}
			assert(rx[i].addr == expected && rx[i].ctrl == 0U);
		}
		assert(data.rx_bd_ring.next_to_process == 0U);
		queue_base_writes++;
		dma_next = 0U;
		dma_blocked = false;
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
	ring_dirty = false;
}
static void sys_cache_data_flush_and_invd_range(void *p, size_t n)
{
}
static void sys_cache_data_invd_range(void *p, size_t n)
{
}
static void net_pkt_cursor_init(struct net_pkt *p)
{
}
static int net_pkt_read(struct net_pkt *p, void *b, size_t n)
{
	return 0;
}
static size_t net_pkt_get_len(struct net_pkt *p)
{
	return p->len;
}
static void net_pkt_unref(struct net_pkt *p)
{
}
static struct net_pkt *net_pkt_rx_alloc_with_buffer(void *iface, uint32_t len, int f, int p, int t)
{
	advance(cpu_cost / 3);
	packet.len = len;
	return empty_pool ? NULL : &packet;
}
static int net_pkt_write(struct net_pkt *p, const void *b, size_t n)
{
	assert(!empty_pool);
	copied++;
	advance(cpu_cost / 3);
	return 0;
}
static int net_recv_data(void *iface, struct net_pkt *p)
{
	assert(queue_tail - queue_head < 512);
	queue[queue_tail++ % 512] = *p;
	delivered++;
	advance(cpu_cost - 2 * (cpu_cost / 3));
	return 0;
}
static int k_work_submit(struct k_work *work)
{
	work->pending = true;
	return 0;
}
static void k_sem_give(struct k_sem *sem)
{
	sem->count++;
}
static void eth_xlnx_gem_isr(const struct device *dev);
static int k_sem_take(struct k_sem *sem, int timeout)
{
	if (sem == &data.tx_done_sem) {
		tx[data.tx_bd_ring.next_to_process].ctrl |= ETH_XLNX_GEM_TX_BD_USED_BIT;
		mmio[ETH_XLNX_GEM_ISR_OFFSET / 4] |= ETH_XLNX_GEM_IXR_TX_COMPLETE_BIT;
		eth_xlnx_gem_isr(&device);
	}
	assert(sem->count > 0);
	sem->count--;
	return 0;
}
/* FUNCTIONS */
static void init(uint8_t count, uint16_t size)
{
	cfg = (struct eth_xlnx_gem_dev_cfg){size, 2048, count, 64, true, false};
	memset(&data, 0, sizeof(data));
	memset(mmio, 0, sizeof(mmio));
	memset(rx, 0, sizeof(rx));
	memset(tx, 0, sizeof(tx));
	data.rx_bd_ring.first_bd = rx;
	data.tx_bd_ring.first_bd = tx;
	data.first_rx_buffer = rxbuf;
	data.first_tx_buffer = txbuf;
	data.tx_bd_ring.ring_sem.count = 1;
	reads = queue_head = queue_tail = delivered = copied = 0;
	empty_pool = burst = dma_blocked = ring_dirty = false;
	queue_base_writes = 0U;
	dma_next = drops = 0;
	resync_logs = resync_skipped = status_reads = rx_control_writes = 0;
	eth_xlnx_gem_configure_buffers(&device);
	mmio[ETH_XLNX_GEM_NWCTRL_OFFSET / 4] = ETH_XLNX_GEM_NWCTRL_RXEN_BIT |
		ETH_XLNX_GEM_NWCTRL_TXEN_BIT;
}
static void orphan_test(void)
{
	const uint32_t heads[] = {11, 14, 31};

	for (uint32_t n = 0; n < 3; n++) {
		init(32, 512);
		uint32_t head = heads[n];
		uint32_t good = (head + 1) % 32;

		data.rx_bd_ring.next_to_process = head;
		rx[head].addr |= 1;
		rx[head].ctrl = ETH_XLNX_GEM_RX_BD_END_OF_FRAME_BIT | 98;
		/* Real 1242-byte frame consumes three 512-byte BDs after the orphan. */
		for (uint32_t i = 0; i < 3; i++) {
			rx[(good + i) % 32].addr |= 1;
			rx[(good + i) % 32].ctrl =
				i == 0 ? ETH_XLNX_GEM_RX_BD_START_OF_FRAME_BIT : 0;
		}
		rx[(good + 2) % 32].ctrl |= ETH_XLNX_GEM_RX_BD_END_OF_FRAME_BIT | 1242;
		mmio[ETH_XLNX_GEM_ISR_OFFSET / 4] = 0x406;
		eth_xlnx_gem_isr(&device);
		assert(data.rx_pend_work.pending);
		eth_xlnx_gem_handle_rx_pending(&device);
		assert(delivered == 1 && data.rx_bd_ring.next_to_process == 0U);
		assert(!(rx[head].addr & 1));
		assert((rx[31].addr & 2) != 0);
		assert(!(mmio[ETH_XLNX_GEM_IMR_OFFSET / 4] & ETH_XLNX_GEM_IXR_FRAME_RX_BIT));
		assert(mmio[ETH_XLNX_GEM_RXSR_OFFSET / 4] == 0);
		empty_pool = true;
		uint32_t next = data.rx_bd_ring.next_to_process;
		uint32_t before = copied;

		rx[next].addr |= 1;
		rx[next].ctrl = ETH_XLNX_GEM_RX_BD_START_OF_FRAME_BIT |
				ETH_XLNX_GEM_RX_BD_END_OF_FRAME_BIT | 98;
		eth_xlnx_gem_handle_rx_pending(&device);
		assert(!(rx[next].addr & 1) && copied == before);
	}
}
/* Bench fault: BNA latches a halt until an effective RXQBASE write. */
static bool dma_receive_bd(uint32_t ctrl)
{
	if (dma_blocked ||
	    (mmio[ETH_XLNX_GEM_NWCTRL_OFFSET / 4] & ETH_XLNX_GEM_NWCTRL_RXEN_BIT) == 0U) {
		drops++;
		return false;
	}
	if ((rx[dma_next].addr & ETH_XLNX_GEM_RX_BD_USED_BIT) != 0U) {
		dma_blocked = true;
		drops++;
		mmio[ETH_XLNX_GEM_RXSR_OFFSET / 4] |= 5U;
		mmio[ETH_XLNX_GEM_ISR_OFFSET / 4] |= 0x404U;
		return false;
	}

	rx[dma_next].ctrl = ctrl;
	rx[dma_next].addr |= ETH_XLNX_GEM_RX_BD_USED_BIT;
	dma_next = (dma_next + 1U) % cfg.rx_bd_count;
	if ((ctrl & ETH_XLNX_GEM_RX_BD_END_OF_FRAME_BIT) != 0U) {
		mmio[ETH_XLNX_GEM_RXSR_OFFSET / 4] |= 2U;
		mmio[ETH_XLNX_GEM_ISR_OFFSET / 4] |= ETH_XLNX_GEM_IXR_FRAME_RX_BIT;
	}
	return true;
}

static void service_rx(void)
{
	reads = 0U;
	eth_xlnx_gem_isr(&device);
	if (data.rx_pend_work.pending) {
		data.rx_pend_work.pending = false;
		eth_xlnx_gem_handle_rx_pending(&device);
	}
	queue_head = queue_tail;
}

static void overflow_resync_test(void)
{
	const uint32_t heads[] = {0U, 11U, 14U, 31U};
	const uint32_t statuses[] = {1U, 4U, 5U};
	const uint32_t frame = ETH_XLNX_GEM_RX_BD_START_OF_FRAME_BIT |
			       ETH_XLNX_GEM_RX_BD_END_OF_FRAME_BIT | 98U;

	for (uint32_t n = 0U; n < 4U; n++) {
		for (uint32_t status = 0U; status < 3U; status++) {
			init(32, 512);
			cfg.defer_rxp_to_queue = n % 2U == 0U;
			dma_next = heads[n];
			data.rx_bd_ring.next_to_process = heads[n];
			struct eth_xlnx_gem_bd saved_tx[64];

			memcpy(saved_tx, tx, sizeof(tx));
			for (uint32_t i = 0U; i < 32U; i++) {
				assert(dma_receive_bd(frame));
			}
			assert(!dma_receive_bd(frame));
			assert(dma_blocked && dma_next == heads[n]);
			/* Isolate BNA/overrun interrupts: no RX-complete interrupt remains. */
			mmio[ETH_XLNX_GEM_RXSR_OFFSET / 4] = statuses[status];
			mmio[ETH_XLNX_GEM_ISR_OFFSET / 4] =
				status == 0U ? ETH_XLNX_GEM_IXR_RX_USED_BIT :
					       ETH_XLNX_GEM_IXR_RX_OVERRUN_BIT;
			/* Include an incomplete chain: recovery must not spin looking for EOF. */
			if (status != 0U) {
				for (uint32_t i = 0U; i < 32U; i++) {
					rx[i].ctrl = i == heads[n] ?
						ETH_XLNX_GEM_RX_BD_START_OF_FRAME_BIT : 0U;
				}
			}
			service_rx();
			bool recovered = false;

			for (uint32_t attempt = 0U; attempt < 4U && !recovered; attempt++) {
				recovered = dma_receive_bd(frame);
				service_rx();
			}
			assert(recovered);
			assert(delivered == 1U && data.rx_bd_ring.next_to_process == 1U);
			assert(queue_base_writes == 2U && rx_control_writes == 2U);
			assert(memcmp(tx, saved_tx, sizeof(tx)) == 0);
			assert(data.tx_bd_ring.free_bds == 64U);
			assert((mmio[ETH_XLNX_GEM_NWCTRL_OFFSET / 4] &
				ETH_XLNX_GEM_NWCTRL_TXEN_BIT) != 0U);
			assert(status_reads >= 2U && mmio[ETH_XLNX_GEM_RXSR_OFFSET / 4] == 0U);
			for (uint32_t i = 0U; i < 4096U; i++) {
				assert(dma_receive_bd(frame));
				service_rx();
				assert(delivered == 2U + i);
				assert(data.rx_bd_ring.next_to_process == dma_next);
			}
			assert(queue_base_writes == 2U);
		}
	}
}

static void halt_model_test(void)
{
	init(32, 512);
	rx[0].addr |= ETH_XLNX_GEM_RX_BD_USED_BIT;
	assert(!dma_receive_bd(0U) && dma_blocked);
	rx[0].addr &= ~ETH_XLNX_GEM_RX_BD_USED_BIT;
	sys_write32(5U, (uintptr_t)mmio + ETH_XLNX_GEM_RXSR_OFFSET);
	assert(!dma_receive_bd(0U));
	/* Rewriting RXQBASE with RX enabled must be ignored. */
	sys_write32((uint32_t)(uintptr_t)rx, (uintptr_t)mmio + ETH_XLNX_GEM_RXQBASE_OFFSET);
	assert(!dma_receive_bd(0U));
	sys_write32(0U, (uintptr_t)mmio + ETH_XLNX_GEM_NWCTRL_OFFSET);
	sys_write32(ETH_XLNX_GEM_NWCTRL_RXEN_BIT, (uintptr_t)mmio + ETH_XLNX_GEM_NWCTRL_OFFSET);
	assert(!dma_receive_bd(0U));
	/* Only a queue-base write while disabled clears the halt. */
	sys_write32(0U, (uintptr_t)mmio + ETH_XLNX_GEM_NWCTRL_OFFSET);
	sys_write32((uint32_t)(uintptr_t)rx, (uintptr_t)mmio + ETH_XLNX_GEM_RXQBASE_OFFSET);
	sys_write32(ETH_XLNX_GEM_NWCTRL_RXEN_BIT, (uintptr_t)mmio + ETH_XLNX_GEM_NWCTRL_OFFSET);
	assert(dma_receive_bd(ETH_XLNX_GEM_RX_BD_START_OF_FRAME_BIT |
			      ETH_XLNX_GEM_RX_BD_END_OF_FRAME_BIT | 98U));
	service_rx();
	assert(delivered == 1U);
}

static void missing_eof_test(void)
{
	init(32, 512);
	rx[0].addr |= ETH_XLNX_GEM_RX_BD_USED_BIT;
	rx[0].ctrl = ETH_XLNX_GEM_RX_BD_START_OF_FRAME_BIT;
	eth_xlnx_gem_handle_rx_pending(&device);
	assert(queue_base_writes == 2U && delivered == 0U);
}

static void stray_ring_test(void)
{
	init(32, 512);
	for (uint32_t i = 0U; i < 32U; i++) {
		assert(dma_receive_bd(ETH_XLNX_GEM_RX_BD_END_OF_FRAME_BIT | 98U));
	}
	service_rx();
	assert(delivered == 0U && copied == 0U);
	assert(resync_logs == 1U && resync_skipped == 32U);
	assert(data.rx_bd_ring.next_to_process == 0U);
	for (uint32_t i = 0U; i < 32U; i++) {
		assert((rx[i].addr & ETH_XLNX_GEM_RX_BD_USED_BIT) == 0U);
	}
	assert(dma_receive_bd(ETH_XLNX_GEM_RX_BD_START_OF_FRAME_BIT |
			      ETH_XLNX_GEM_RX_BD_END_OF_FRAME_BIT | 98U));
	service_rx();
	assert(delivered == 1U && data.rx_bd_ring.next_to_process == 1U);
}

void phy_burst_init(void);
uint32_t phy_burst_poll(uint64_t elapsed_us);
static void flood_test(void)
{
	for (cpu_cost = 250; cpu_cost <= 350; cpu_cost += 100) {
		init(255, 2048);
		phy_burst_init();
		memset(sent, 0, sizeof(sent));
		now_us = dma_next = drops = 0;
		burst = true;
		uint32_t probes = 0, replies = 0, heartbeats = 0;
		uint64_t last_heartbeat = 0, next_monitor = 0;

		while (now_us < 60100000) {
			if (now_us >= next_monitor) {
				advance(phy_burst_poll(now_us));
				next_monitor += 500000;
			}
			reads = 0;
			if (mmio[ETH_XLNX_GEM_ISR_OFFSET / 4] &
			    ~mmio[ETH_XLNX_GEM_IMR_OFFSET / 4]) {
				eth_xlnx_gem_isr(&device);
			}
			if (data.rx_pend_work.pending) {
				data.rx_pend_work.pending = false;
				eth_xlnx_gem_handle_rx_pending(&device);
			}
			/* Model ready cooperative peers after the driver returns. */
			assert(now_us - last_heartbeat < 25000);
			if (now_us - last_heartbeat >= 5000) {
				last_heartbeat = now_us;
				heartbeats++;
			}
			while (queue_head != queue_tail) {
				struct net_pkt out = queue[queue_head++ % 512];

				advance(25);
				if (out.len < 100) {
					reads = 0;
					assert(eth_xlnx_gem_send(&device, &out) == 0);
					probes += out.len == 99;
					replies += out.len == 98;
				}
			}
			advance(10);
		}
		burst = false;
		printf("cost=%u: probes=%u/60 ICMP=%u/30000 drops=%u heartbeat=%u\n", cpu_cost,
		       probes, replies, drops, heartbeats);
		assert(probes == 60 && replies == 30000 && drops == 0);
		assert(sent[0] == 108000 && heartbeats > 6000);
		assert(data.tx_bd_ring.free_bds == 64);
	}
}
int main(int argc, char **argv)
{
	assert(argc == 2);
	if (atoi(argv[1]) == 1) {
		orphan_test();
	} else if (atoi(argv[1]) == 14) {
		overflow_resync_test();
	} else if (atoi(argv[1]) == 15) {
		stray_ring_test();
	} else if (atoi(argv[1]) == 16) {
		halt_model_test();
	} else if (atoi(argv[1]) == 17) {
		missing_eof_test();
	} else {
		flood_test();
	}
	return 0;
}
