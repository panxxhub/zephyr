/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
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
#define LOG_ERR_RATELIMIT_RATE(...) ((void)0)
#define LOG_WRN_RATELIMIT_RATE(...) ((void)0)
#define LOG_DBG(...)                ((void)0)
#define DT_INST_FOREACH_STATUS_OKAY(m)
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
static bool burst, empty_pool;
static uint64_t now_us;
static uint32_t cpu_cost, sent[3], dma_next, drops, delivered, copied, reads;
static uint32_t queue_head, queue_tail;
static struct net_pkt packet, queue[512];
static const uint32_t rates[] = {1800, 500, 1};
static const uint32_t lengths[] = {1242, 98, 99};
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
		if (rx[dma_next].addr & 1) {
			drops++;
			mmio[ETH_XLNX_GEM_ISR_OFFSET / 4] |= 0x404;
			mmio[ETH_XLNX_GEM_RXSR_OFFSET / 4] |= 5;
		} else {
			rx[dma_next].addr |= 1;
			rx[dma_next].ctrl = ETH_XLNX_GEM_RX_BD_START_OF_FRAME_BIT |
					    ETH_XLNX_GEM_RX_BD_END_OF_FRAME_BIT | lengths[sender];
			dma_next = (dma_next + 1) % cfg.rx_bd_count;
			mmio[ETH_XLNX_GEM_ISR_OFFSET / 4] |= ETH_XLNX_GEM_IXR_FRAME_RX_BIT;
		}
	}
}
static uint32_t sys_read32(uintptr_t addr)
{
	assert(++reads < 100000);
	return *(uint32_t *)addr;
}
static void sys_write32(uint32_t value, uintptr_t addr)
{
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
	empty_pool = false;
	eth_xlnx_gem_configure_buffers(&device);
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
		assert(delivered == 1 && data.rx_bd_ring.next_to_process == (good + 3) % 32);
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
	} else {
		flood_test();
	}
	return 0;
}
