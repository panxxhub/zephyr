/*
 * Copyright (c) 2026 Moton Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Round trips frames of every length class through the actual Xilinx GEM
 * driver on the emulated Zynq controller, using the controller's local
 * loopback mode so that no host network interface is needed. The transmit
 * path, the buffer descriptor rings and the receive path - including the
 * cache maintenance the driver performs on both - are the production ones.
 *
 * What this cannot show: the emulated controller has no cache and no timing
 * of its own, so a maintained span that is too small still returns the right
 * bytes here. The spans themselves are checked arithmetically below and
 * against a fake controller in tests/drivers/ethernet/xlnx_gem_recovery;
 * their cost is a hardware measurement.
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/ethernet/eth_xlnx_gem.h>
#include <zephyr/kernel.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/promiscuous.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/ztest.h>

#include "eth_xlnx_gem_priv.h"

#define GEM_NODE DT_NODELABEL(gem0)
#define RX_BUFFER_SIZE DT_PROP(GEM_NODE, rx_buffer_size)
#define TX_BUFFER_SIZE DT_PROP(GEM_NODE, tx_buffer_size)

/*
 * Every length class an Ethernet frame can have, from the shortest legal
 * frame to the longest, plus the lengths on either side of a buffer boundary
 * and of a cache line boundary.
 */
static const uint16_t frame_lengths[] = {
	60, 61, 62, 63, 64, 65, 96, 127, 128, 129, 255, 256,
	511, 512, 513, 1023, 1024, 1500, 1513, 1514,
};

/* Offsets of the two checksum fields the controller substitutes, if enabled. */
#define IP_OFFSET	 14
#define UDP_OFFSET	 34
#define CHKSUM_FRAME_LEN 60

struct loopback_fixture {
	const struct device *dev;
	struct net_if *iface;
	const struct ethernet_api *api;
	uint32_t saved_nwctrl;
};

static struct loopback_fixture fixture;

static void fill(uint8_t *buffer, uint16_t length)
{
	/* The destination MAC must not match, so that nothing but the
	 * promiscuous hook consumes the looped back frame.
	 */
	static const uint8_t header[12] = {
		0x02, 0x00, 0x5e, 0x00, 0x00, 0x11,
		0x02, 0x00, 0x5e, 0x00, 0x00, 0x22,
	};

	memcpy(buffer, header, sizeof(header));
	buffer[12] = 0x88;
	buffer[13] = 0xb5;
	for (uint16_t i = 14U; i < length; i++) {
		buffer[i] = (uint8_t)(i * 7U + length);
	}
}

static void *setup(void)
{
	fixture.dev = DEVICE_DT_GET(GEM_NODE);
	zassert_true(device_is_ready(fixture.dev), "GEM not ready");

	fixture.iface = net_if_lookup_by_dev(fixture.dev);
	zassert_not_null(fixture.iface, "no interface for the GEM");
	fixture.api = fixture.dev->api;

	/* The emulated PHY negotiates a moment after boot. */
	for (int i = 0; i < 500 && !net_if_is_carrier_ok(fixture.iface); i++) {
		k_msleep(10);
	}
	zassert_true(net_if_is_carrier_ok(fixture.iface), "carrier stayed down");
	for (int i = 0; i < 500 && !net_if_is_up(fixture.iface); i++) {
		(void)net_if_up(fixture.iface);
		k_msleep(10);
	}
	zassert_true(net_if_is_up(fixture.iface), "interface stayed down");

	/*
	 * Local loopback: the controller returns every transmitted frame to
	 * its own receive path, so the whole data path is exercised without a
	 * peer on the host.
	 */
	fixture.saved_nwctrl = sys_read32(DEVICE_MMIO_NAMED_GET(fixture.dev, mac) +
					  ETH_XLNX_GEM_NWCTRL_OFFSET);
	sys_write32(fixture.saved_nwctrl | ETH_XLNX_GEM_NWCTRL_LOOPEN_BIT,
		    DEVICE_MMIO_NAMED_GET(fixture.dev, mac) + ETH_XLNX_GEM_NWCTRL_OFFSET);

	zassert_ok(net_promisc_mode_on(fixture.iface), "promiscuous mode refused");
	return &fixture;
}

static void teardown(void *unused)
{
	ARG_UNUSED(unused);

	sys_write32(fixture.saved_nwctrl,
		    DEVICE_MMIO_NAMED_GET(fixture.dev, mac) + ETH_XLNX_GEM_NWCTRL_OFFSET);
	(void)net_promisc_mode_off(fixture.iface);
}

/*
 * How much of a frame is guaranteed to come back unaltered. With the transmit
 * checksum engine enabled the controller substitutes the checksum fields of the
 * frames it recognises, so only the bytes ahead of any network header can be
 * compared. QEMU's model recognises more frames than the silicon does - it
 * rewrote the IPv4 header checksum position of frames whose EtherType is not
 * IPv4 at all - which is one more reason not to compare past that point.
 */
static uint16_t unaltered_span(uint16_t length)
{
	if (DEV_CFG(fixture.dev)->disable_tx_chksum_offload) {
		return length;
	}

	return MIN(length, IP_OFFSET);
}

static void drain(void)
{
	struct net_pkt *pkt;

	while ((pkt = net_promisc_mode_wait_data(K_MSEC(20))) != NULL) {
		net_pkt_unref(pkt);
	}
}

static void round_trip(uint16_t length)
{
	static uint8_t sent[1514];
	static uint8_t received[1514];
	struct net_pkt *pkt;

	fill(sent, length);

	pkt = net_pkt_alloc_with_buffer(fixture.iface, length, NET_AF_UNSPEC, 0, K_SECONDS(1));
	zassert_not_null(pkt, "no packet for a %u byte frame", length);
	zassert_ok(net_pkt_write(pkt, sent, length), "write of %u bytes failed", length);
	zassert_ok(fixture.api->send(fixture.dev, pkt), "send of %u bytes failed", length);
	net_pkt_unref(pkt);

	pkt = net_promisc_mode_wait_data(K_MSEC(500));
	zassert_not_null(pkt, "%u byte frame did not come back", length);
	zassert_equal(net_pkt_get_len(pkt), length, "%u byte frame came back as %zu bytes",
		      length, net_pkt_get_len(pkt));

	net_pkt_cursor_init(pkt);
	zassert_ok(net_pkt_read(pkt, received, length), "read back of %u bytes failed", length);
	net_pkt_unref(pkt);

	zassert_mem_equal(received, sent, unaltered_span(length),
			  "%u byte frame came back altered", length);
}

/* Every length class survives the round trip byte for byte. */
ZTEST(xlnx_gem_loopback, test_frame_length_classes)
{
	for (int i = 0; i < (int)ARRAY_SIZE(frame_lengths); i++) {
		drain();
		round_trip(frame_lengths[i]);
	}
}

/*
 * Frames queued back to back, without waiting for any of them, must reach the
 * wire in the order in which they were queued and with their contents intact.
 * This holds whether or not the send function waits for each transmission; the
 * pipelining that CONFIG_ETH_XLNX_GEM_TX_ASYNC adds on top is established in
 * the host harness, where the controller's reporting can be withheld.
 */
ZTEST(xlnx_gem_loopback, test_back_to_back_frames)
{
	static uint8_t sent[8][256];
	static uint8_t received[256];
	const uint16_t length = 256;
	struct net_pkt *pkt;

	drain();

	for (int i = 0; i < (int)ARRAY_SIZE(sent); i++) {
		fill(sent[i], length);
		/* A tag that survives the round trip identifies the frame. */
		sent[i][14] = (uint8_t)i;

		pkt = net_pkt_alloc_with_buffer(fixture.iface, length, NET_AF_UNSPEC, 0,
						K_SECONDS(1));
		zassert_not_null(pkt, "no packet for frame %d", i);
		zassert_ok(net_pkt_write(pkt, sent[i], length), "write of frame %d failed", i);
		zassert_ok(fixture.api->send(fixture.dev, pkt), "send of frame %d failed", i);
		net_pkt_unref(pkt);
	}

	for (int i = 0; i < (int)ARRAY_SIZE(sent); i++) {
		pkt = net_promisc_mode_wait_data(K_MSEC(500));
		zassert_not_null(pkt, "frame %d did not come back", i);
		zassert_equal(net_pkt_get_len(pkt), length, "frame %d came back as %zu bytes",
			      i, net_pkt_get_len(pkt));

		net_pkt_cursor_init(pkt);
		zassert_ok(net_pkt_read(pkt, received, length), "read back of frame %d failed",
			   i);
		net_pkt_unref(pkt);

		zassert_equal(received[14], (uint8_t)i, "frame %d came back in position %u",
			      received[14], i);
		zassert_mem_equal(received, sent[i], unaltered_span(length),
				  "frame %d came back altered", i);
	}

	zassert_is_null(net_promisc_mode_wait_data(K_MSEC(100)), "more frames came back");
}

/* Repeating the longest frame must not consume buffer descriptors. */
ZTEST(xlnx_gem_loopback, test_descriptors_are_returned)
{
	struct eth_xlnx_gem_dev_data *dev_data = DEV_DATA(fixture.dev);
	const struct eth_xlnx_gem_dev_cfg *dev_conf = DEV_CFG(fixture.dev);

	zassert_equal(dev_data->tx_bd_ring.free_bds, dev_conf->tx_bd_count,
		      "the TX ring did not start out empty");

	for (int i = 0; i < 4 * dev_conf->tx_bd_count; i++) {
		drain();
		round_trip(1514);
		zassert_equal(dev_data->tx_bd_ring.free_bds, dev_conf->tx_bd_count,
			      "TX descriptors leaked after %d frames: %u of %u free",
			      i + 1, dev_data->tx_bd_ring.free_bds, dev_conf->tx_bd_count);
	}
}

/*
 * The alignment rules the frame sized cache maintenance relies on, checked
 * against the addresses and the cache line size of this build.
 */
ZTEST(xlnx_gem_loopback, test_cache_maintenance_alignment)
{
	struct eth_xlnx_gem_dev_data *dev_data = DEV_DATA(fixture.dev);
	const struct eth_xlnx_gem_dev_cfg *dev_conf = DEV_CFG(fixture.dev);
	const size_t line = CONFIG_DCACHE_LINE_SIZE;

	zassert_true(IS_ENABLED(CONFIG_DCACHE), "this board is expected to have a data cache");

	/* Every buffer starts on a cache line boundary ... */
	zassert_equal((uintptr_t)dev_data->first_rx_buffer % line, 0U,
		      "the RX buffers are not cache line aligned");
	zassert_equal((uintptr_t)dev_data->first_tx_buffer % line, 0U,
		      "the TX buffers are not cache line aligned");
	zassert_equal(ETH_XLNX_GEM_DMA_AREA_ALIGNMENT % line, 0U,
		      "the DMA area alignment is not a multiple of the cache line size");
	/* ... and is a whole number of cache lines long, so rounding a length
	 * up to the next line can never leave the buffer it belongs to.
	 */
	zassert_equal(dev_conf->rx_buffer_size % line, 0U,
		      "the RX buffer size is not a multiple of the cache line size");
	zassert_equal(dev_conf->tx_buffer_size % line, 0U,
		      "the TX buffer size is not a multiple of the cache line size");

	for (int i = 0; i < (int)ARRAY_SIZE(frame_lengths); i++) {
		uint16_t remaining = frame_lengths[i];

		while (remaining > 0U) {
			uint16_t chunk = MIN(remaining, dev_conf->rx_buffer_size);
			size_t span = ETH_XLNX_GEM_CACHE_SPAN(chunk, dev_conf->rx_buffer_size);

			zassert_equal(span % line, 0U, "span of %u is not a whole cache line",
				      chunk);
			zassert_true(span >= chunk, "span of %u leaves %zu bytes stale", chunk,
				     chunk - span);
			zassert_true(span <= dev_conf->rx_buffer_size,
				     "span of %u reaches past its buffer", chunk);
			zassert_true(span - chunk < line, "span of %u is needlessly large",
				     chunk);
			remaining -= chunk;
		}
	}
}

/*
 * Checksum offload. The controller computes and checks the IPv4 header, TCP and
 * UDP checksums; it knows nothing about ICMP. The driver reports exactly that,
 * and the network stack skips in software only what the driver reports.
 *
 * What this cannot show: the emulated controller is a model of the offload, not
 * of the silicon, and the point of the offload is the time it saves the CPU,
 * which is a hardware measurement.
 */
static uint16_t ones_complement(const uint8_t *data, size_t length)
{
	uint32_t sum = 0;

	for (size_t i = 0; i < length; i += 2) {
		sum += ((uint32_t)data[i] << 8) | data[i + 1];
	}
	while ((sum >> 16) != 0U) {
		sum = (sum & 0xFFFFU) + (sum >> 16);
	}

	return (uint16_t)~sum;
}

/* An IPv4/UDP datagram whose two checksum fields are left at zero. */
static void build_datagram(uint8_t *buffer)
{
	uint8_t *ip = buffer + IP_OFFSET;
	uint8_t *udp = buffer + UDP_OFFSET;

	memset(buffer, 0, CHKSUM_FRAME_LEN);
	fill(buffer, IP_OFFSET);
	buffer[12] = 0x08;
	buffer[13] = 0x00;

	ip[0] = 0x45;
	ip[3] = (uint8_t)(CHKSUM_FRAME_LEN - IP_OFFSET);
	ip[8] = 64;
	ip[9] = 17;
	ip[12] = 10; ip[13] = 0; ip[14] = 0; ip[15] = 1;
	ip[16] = 10; ip[17] = 0; ip[18] = 0; ip[19] = 2;

	udp[0] = 0x13; udp[1] = 0x88;
	udp[2] = 0x13; udp[3] = 0x89;
	udp[5] = (uint8_t)(CHKSUM_FRAME_LEN - UDP_OFFSET);

	for (int i = UDP_OFFSET + 8; i < CHKSUM_FRAME_LEN; i++) {
		buffer[i] = (uint8_t)i;
	}
}

/* What the driver reports must follow the device tree and the Kconfig. */
ZTEST(xlnx_gem_loopback, test_checksum_offload_is_reported)
{
	const struct eth_xlnx_gem_dev_cfg *dev_conf = DEV_CFG(fixture.dev);
	const bool offload = !IS_ENABLED(CONFIG_ETH_XLNX_GEM_QEMU_NO_CHKSUM_OFFLOAD) &&
			     !DT_PROP(GEM_NODE, disable_rx_checksum_offload);
	enum ethernet_hw_caps caps = fixture.api->get_capabilities(fixture.dev, fixture.iface);
	struct ethernet_config config;
	uint32_t reg;

	zassert_equal(!dev_conf->disable_rx_chksum_offload, offload);
	zassert_equal(!dev_conf->disable_tx_chksum_offload, offload);
	zassert_equal((caps & ETHERNET_HW_RX_CHKSUM_OFFLOAD) != 0, offload);
	zassert_equal((caps & ETHERNET_HW_TX_CHKSUM_OFFLOAD) != 0, offload);

	/* gem.net_cfg [24] receive checksum offload, gem.dma_cfg [11] transmit */
	reg = sys_read32(DEVICE_MMIO_NAMED_GET(fixture.dev, mac) + ETH_XLNX_GEM_NWCFG_OFFSET);
	zassert_equal((reg & ETH_XLNX_GEM_NWCFG_RXCHKSUMEN_BIT) != 0, offload,
		      "gem.net_cfg 0x%08x does not match the configured RX offload", reg);
	reg = sys_read32(DEVICE_MMIO_NAMED_GET(fixture.dev, mac) + ETH_XLNX_GEM_DMACR_OFFSET);
	zassert_equal((reg & ETH_XLNX_GEM_DMACR_TCP_CHKSUM_BIT) != 0, offload,
		      "gem.dma_cfg 0x%08x does not match the configured TX offload", reg);

	zassert_ok(fixture.api->get_config(fixture.dev, fixture.iface,
					   ETHERNET_CONFIG_TYPE_TX_CHECKSUM_SUPPORT, &config));
	zassert_equal(config.chksum_support,
		      offload ? (ETHERNET_CHECKSUM_SUPPORT_IPV4_HEADER |
				 ETHERNET_CHECKSUM_SUPPORT_IPV6_HEADER |
				 ETHERNET_CHECKSUM_SUPPORT_TCP |
				 ETHERNET_CHECKSUM_SUPPORT_UDP)
			      : ETHERNET_CHECKSUM_SUPPORT_NONE);

	/* The stack skips exactly the checksums the controller computes. */
	zassert_equal(net_if_need_calc_tx_checksum(fixture.iface, NET_IF_CHECKSUM_IPV4_UDP),
		      !offload);
	zassert_equal(net_if_need_calc_tx_checksum(fixture.iface, NET_IF_CHECKSUM_IPV4_TCP),
		      !offload);
	zassert_equal(net_if_need_calc_rx_checksum(fixture.iface, NET_IF_CHECKSUM_IPV4_HEADER),
		      !offload);
	/* The controller knows nothing about ICMP, whatever else is offloaded. */
	zassert_true(net_if_need_calc_tx_checksum(fixture.iface, NET_IF_CHECKSUM_IPV4_ICMP),
		     "ICMP checksums must stay in software");
}

/* An offloading controller fills in the checksums the stack left at zero. */
ZTEST(xlnx_gem_loopback, test_checksum_offload_fills_in_the_frame)
{
	const struct eth_xlnx_gem_dev_cfg *dev_conf = DEV_CFG(fixture.dev);
	static uint8_t sent[CHKSUM_FRAME_LEN];
	static uint8_t received[CHKSUM_FRAME_LEN];
	struct net_pkt *pkt;
	uint16_t expected;

	drain();
	build_datagram(sent);

	pkt = net_pkt_alloc_with_buffer(fixture.iface, CHKSUM_FRAME_LEN, NET_AF_UNSPEC, 0,
					K_SECONDS(1));
	zassert_not_null(pkt);
	zassert_ok(net_pkt_write(pkt, sent, CHKSUM_FRAME_LEN));
	zassert_ok(fixture.api->send(fixture.dev, pkt));
	net_pkt_unref(pkt);

	pkt = net_promisc_mode_wait_data(K_MSEC(500));
	zassert_not_null(pkt, "the datagram did not come back");
	net_pkt_cursor_init(pkt);
	zassert_ok(net_pkt_read(pkt, received, CHKSUM_FRAME_LEN));
	net_pkt_unref(pkt);

	if (dev_conf->disable_tx_chksum_offload) {
		/* Nothing computes them, so they stay as the stack left them. */
		zassert_equal(received[IP_OFFSET + 10], 0);
		zassert_equal(received[IP_OFFSET + 11], 0);
		zassert_equal(received[UDP_OFFSET + 6], 0);
		zassert_equal(received[UDP_OFFSET + 7], 0);
		ztest_test_skip();
	}

	expected = ones_complement(&received[IP_OFFSET], 20);
	zassert_equal(expected, 0, "the IPv4 header checksum came back wrong");
	zassert_true(received[IP_OFFSET + 10] != 0 || received[IP_OFFSET + 11] != 0,
		     "the IPv4 header checksum was not filled in");
	zassert_true(received[UDP_OFFSET + 6] != 0 || received[UDP_OFFSET + 7] != 0,
		     "the UDP checksum was not filled in");

	/* Everything but the two checksum fields survives unaltered. */
	memcpy(&received[IP_OFFSET + 10], &sent[IP_OFFSET + 10], 2);
	memcpy(&received[UDP_OFFSET + 6], &sent[UDP_OFFSET + 6], 2);
	zassert_mem_equal(received, sent, CHKSUM_FRAME_LEN, "the datagram came back altered");
}

#ifdef CONFIG_ETH_XLNX_GEM_RX_THREAD
static void name_matches(const struct k_thread *thread, void *found)
{
	const char *name = k_thread_name_get((k_tid_t)thread);

	if (name != NULL && strcmp(name, "gem_rx") == 0) {
		*(bool *)found = true;
	}
}

/* The deferred work of this driver has a thread of its own to be seen on. */
ZTEST(xlnx_gem_loopback, test_driver_thread_exists)
{
	bool found = false;

	k_thread_foreach(name_matches, &found);
	zassert_true(found, "no thread named gem_rx");
	zassert_true(strcmp(k_thread_name_get(k_current_get()), "gem_rx") != 0,
		     "the test is running on the driver's own thread");
}
#endif /* CONFIG_ETH_XLNX_GEM_RX_THREAD */

/*
 * The diagnostic counters of panxxhub/zephyr#70, read through the driver API
 * that needs no network statistics subsystem. The counters are always compiled
 * in, so every scenario of this suite exercises them.
 */

static struct eth_xlnx_gem_stats read_counters(void)
{
	struct eth_xlnx_gem_stats stats;

	zassert_ok(eth_xlnx_gem_stats_get(fixture.dev, &stats), "the counters could not be read");
	return stats;
}

/* The API refuses everything that is not this driver's own device. */
ZTEST(xlnx_gem_loopback, test_stats_api_arguments)
{
	const struct device *phy = DEVICE_DT_GET(DT_PHANDLE(GEM_NODE, phy_handle));
	struct eth_xlnx_gem_stats stats;

	zassert_equal(eth_xlnx_gem_stats_get(NULL, &stats), -EINVAL, "a NULL device was accepted");
	zassert_equal(eth_xlnx_gem_stats_get(fixture.dev, NULL), -EINVAL,
		      "a NULL destination was accepted");
	zassert_equal(eth_xlnx_gem_stats_get(phy, &stats), -ENOTSUP,
		      "a device that is not a GEM was accepted");
	zassert_ok(eth_xlnx_gem_stats_get(fixture.dev, &stats), "the GEM was refused");
}

/*
 * The reverse control for every counter at once: frames that arrive and leave
 * the way they are supposed to move none of them.
 */
ZTEST(xlnx_gem_loopback, test_counters_ignore_healthy_traffic)
{
	struct eth_xlnx_gem_stats before = read_counters();
	struct eth_xlnx_gem_stats after;

	for (int i = 0; i < (int)ARRAY_SIZE(frame_lengths); i++) {
		drain();
		round_trip(frame_lengths[i]);
	}

	after = read_counters();
	zassert_mem_equal(&after, &before, sizeof(after),
			  "healthy traffic moved a diagnostic counter");
}

/*
 * Hand every receive descriptor back to the driver, so the controller finds no
 * buffer for the next frame it receives and reports buffer-not-available. The
 * driver's answer to that is a queue reset, which throws away whatever the ring
 * was holding - which is the whole point of the counters.
 */
static void starve_rx_ring(uint32_t frames)
{
	struct eth_xlnx_gem_dev_data *dev_data = DEV_DATA(fixture.dev);
	const struct eth_xlnx_gem_dev_cfg *dev_conf = DEV_CFG(fixture.dev);
	unsigned int key = irq_lock();

	for (uint32_t i = 0U; i < dev_conf->rx_bd_count; i++) {
		uintptr_t addr = (uintptr_t)&dev_data->rx_bd_ring.first_bd[i].addr;
		uintptr_t ctrl = (uintptr_t)&dev_data->rx_bd_ring.first_bd[i].ctrl;

		sys_write32(sys_read32(addr) | ETH_XLNX_GEM_RX_BD_USED_BIT, addr);
		/*
		 * A descriptor's control word still holds the flags of the
		 * frame it last carried, so the ones that are not to look like
		 * an undelivered frame have to be cleared explicitly.
		 */
		sys_write32((i < frames) ? ETH_XLNX_GEM_RX_BD_START_OF_FRAME_BIT : 0U, ctrl);
	}
	irq_unlock(key);
}

/* Wait until the driver has reset the queue, which it does from its handler. */
static struct eth_xlnx_gem_stats wait_for_reset(uint32_t resets)
{
	struct eth_xlnx_gem_stats stats = read_counters();

	for (int i = 0; i < 200 && stats.rx_queue_resets < resets; i++) {
		k_msleep(10);
		stats = read_counters();
	}
	return stats;
}

ZTEST(xlnx_gem_loopback, test_rx_starvation_counters)
{
	struct eth_xlnx_gem_stats before;
	struct eth_xlnx_gem_stats after;
	struct net_pkt *pkt;
	static uint8_t frame[256];

	drain();
	before = read_counters();

	starve_rx_ring(0U);
	fill(frame, sizeof(frame));
	pkt = net_pkt_alloc_with_buffer(fixture.iface, sizeof(frame), NET_AF_UNSPEC, 0,
					K_SECONDS(1));
	zassert_not_null(pkt, "no packet for the starving frame");
	zassert_ok(net_pkt_write(pkt, frame, sizeof(frame)), "write failed");
	zassert_ok(fixture.api->send(fixture.dev, pkt), "send failed");
	net_pkt_unref(pkt);

	after = wait_for_reset(before.rx_queue_resets + 1U);
	zassert_true(after.rx_buffer_not_available > before.rx_buffer_not_available,
		     "a starved ring did not count a buffer-not-available indication");
	zassert_equal(after.rx_queue_resets, before.rx_queue_resets + 1U,
		      "a starved ring did not cost exactly one queue reset");
	/* The ring held no frame, so the reset threw none away. */
	zassert_equal(after.rx_reset_discards, before.rx_reset_discards,
		      "an empty ring reset discarded %u frames",
		      after.rx_reset_discards - before.rx_reset_discards);
	zassert_equal(after.tx_send_timeouts, before.tx_send_timeouts, "a transmit counter moved");
	zassert_equal(after.tx_age_reclaims, before.tx_age_reclaims, "a transmit counter moved");
	zassert_equal(after.tx_ring_full, before.tx_ring_full, "a transmit counter moved");

	/* Again, this time with frames in the ring for the reset to discard. */
	before = after;
	starve_rx_ring(3U);
	pkt = net_pkt_alloc_with_buffer(fixture.iface, sizeof(frame), NET_AF_UNSPEC, 0,
					K_SECONDS(1));
	zassert_not_null(pkt, "no packet for the second starving frame");
	zassert_ok(net_pkt_write(pkt, frame, sizeof(frame)), "write failed");
	zassert_ok(fixture.api->send(fixture.dev, pkt), "send failed");
	net_pkt_unref(pkt);

	after = wait_for_reset(before.rx_queue_resets + 1U);
	zassert_equal(after.rx_queue_resets, before.rx_queue_resets + 1U,
		      "the second starved ring did not cost exactly one queue reset");
	zassert_equal(after.rx_reset_discards, before.rx_reset_discards + 3U,
		      "the reset discarded %u frames, not the 3 that were in the ring",
		      after.rx_reset_discards - before.rx_reset_discards);

	/* The interface still receives once the driver has rebuilt the ring. */
	drain();
	round_trip(256);
}

/*
 * A controller whose transmitter is switched off never reports a completion.
 * That is what the transmit counters are there to distinguish: a sender that
 * gives up on a frame, a sender held back by a full ring, and the descriptors
 * an abandoned transmission leaves behind.
 */
/*
 * Re-arm every transmit descriptor and restart the controller at descriptor
 * zero, which is the state eth_xlnx_gem_configure_buffers() leaves behind.
 * The transmitter must be off while the queue base is written.
 */
static void restore_tx_ring(uint32_t nwctrl)
{
	struct eth_xlnx_gem_dev_data *dev_data = DEV_DATA(fixture.dev);
	const struct eth_xlnx_gem_dev_cfg *dev_conf = DEV_CFG(fixture.dev);

	for (uint32_t i = 0U; i < dev_conf->tx_bd_count; i++) {
		struct eth_xlnx_gem_bd *bd = &dev_data->tx_bd_ring.first_bd[i];
		uint32_t ctrl = ETH_XLNX_GEM_TX_BD_USED_BIT;

		if (i == (uint32_t)(dev_conf->tx_bd_count - 1U)) {
			ctrl |= ETH_XLNX_GEM_TX_BD_WRAP_BIT;
		}
		sys_write32((uint32_t)dev_data->first_tx_buffer + (i * dev_conf->tx_buffer_size),
			    (uintptr_t)&bd->addr);
		sys_write32(ctrl, (uintptr_t)&bd->ctrl);
	}
	dev_data->tx_bd_ring.next_to_process = 0U;
	dev_data->tx_bd_ring.next_to_use = 0U;
	dev_data->tx_bd_ring.free_bds = dev_conf->tx_bd_count;

	barrier_dmem_fence_full();
	sys_write32((uint32_t)dev_data->tx_bd_ring.first_bd,
		    DEVICE_MMIO_NAMED_GET(fixture.dev, mac) + ETH_XLNX_GEM_TXQBASE_OFFSET);
	sys_write32(nwctrl, DEVICE_MMIO_NAMED_GET(fixture.dev, mac) + ETH_XLNX_GEM_NWCTRL_OFFSET);
}

ZTEST(xlnx_gem_loopback, test_tx_stall_counters)
{
	struct eth_xlnx_gem_dev_data *dev_data = DEV_DATA(fixture.dev);
	const struct eth_xlnx_gem_dev_cfg *dev_conf = DEV_CFG(fixture.dev);
	struct eth_xlnx_gem_stats before;
	struct eth_xlnx_gem_stats after;
	static uint8_t frame[256];
	uint32_t sent = 0U;
	uint32_t nwctrl;

	if (!IS_ENABLED(CONFIG_ETH_XLNX_GEM_TX_RECLAIM) &&
	    !IS_ENABLED(CONFIG_ETH_XLNX_GEM_TX_ASYNC)) {
		/* Without either option an abandoned frame costs the ring its
		 * descriptors for good, which would break what follows.
		 */
		ztest_test_skip();
	}

	drain();
	zassert_equal(dev_data->tx_bd_ring.free_bds, dev_conf->tx_bd_count,
		      "the TX ring did not start out empty");
	before = read_counters();

	/* Switch the transmitter off: nothing leaves, nothing is confirmed. */
	nwctrl = sys_read32(DEVICE_MMIO_NAMED_GET(fixture.dev, mac) + ETH_XLNX_GEM_NWCTRL_OFFSET);
	sys_write32(nwctrl & ~ETH_XLNX_GEM_NWCTRL_TXEN_BIT,
		    DEVICE_MMIO_NAMED_GET(fixture.dev, mac) + ETH_XLNX_GEM_NWCTRL_OFFSET);

	fill(frame, sizeof(frame));
	/*
	 * One frame more than the ring holds. The blocking send function gives
	 * up on each of them in turn; the asynchronous one queues them until
	 * the ring is full and only then has to wait.
	 */
	for (uint32_t i = 0U; i <= dev_conf->tx_bd_count; i++) {
		struct net_pkt *pkt = net_pkt_alloc_with_buffer(
			fixture.iface, sizeof(frame), NET_AF_UNSPEC, 0, K_SECONDS(1));

		zassert_not_null(pkt, "no packet for stalled frame %u", i);
		zassert_ok(net_pkt_write(pkt, frame, sizeof(frame)), "write failed");
		(void)fixture.api->send(fixture.dev, pkt);
		net_pkt_unref(pkt);
		sent++;
		if (!IS_ENABLED(CONFIG_ETH_XLNX_GEM_TX_ASYNC)) {
			/* Every blocking send has already timed out by now. */
			break;
		}
	}

	after = read_counters();
	zassert_true(after.tx_send_timeouts > before.tx_send_timeouts,
		     "a transmitter that never confirms cost no send timeout");
	if (IS_ENABLED(CONFIG_ETH_XLNX_GEM_TX_ASYNC)) {
		zassert_true(after.tx_ring_full > before.tx_ring_full,
			     "a full ring held nobody back");
		zassert_true(after.tx_age_reclaims > before.tx_age_reclaims,
			     "no transmission was abandoned by age");
	} else {
		zassert_equal(after.tx_ring_full, before.tx_ring_full,
			      "the ring was reported full although %u frames fit", sent);
		zassert_equal(after.tx_age_reclaims, before.tx_age_reclaims,
			      "the blocking send function reclaimed by age");
	}
	zassert_equal(after.rx_overruns, before.rx_overruns, "a receive counter moved");
	zassert_equal(after.rx_queue_resets, before.rx_queue_resets, "a receive counter moved");

	/*
	 * Whatever the controller did, the accounting stayed inside the ring:
	 * the reclaim paths of #68 return what an abandoned transmission left
	 * behind. The blocking send function gives up on its one frame and the
	 * ring is whole again; the asynchronous one keeps the ring full, since
	 * every frame takes the descriptor the previous abandonment freed.
	 */
	zassert_true(dev_data->tx_bd_ring.free_bds <= dev_conf->tx_bd_count,
		     "the TX ring overran: %u of %u descriptors free",
		     dev_data->tx_bd_ring.free_bds, dev_conf->tx_bd_count);
	if (!IS_ENABLED(CONFIG_ETH_XLNX_GEM_TX_ASYNC)) {
		zassert_equal(dev_data->tx_bd_ring.free_bds, dev_conf->tx_bd_count,
			      "the TX ring did not recover: %u of %u descriptors free",
			      dev_data->tx_bd_ring.free_bds, dev_conf->tx_bd_count);
	}

	/*
	 * The controller stopped part way through the ring, so put its cursor
	 * and the ring back where the driver's initialisation leaves them
	 * before switching the transmitter on again.
	 */
	restore_tx_ring(nwctrl);
	drain();
	round_trip(256);
}

ZTEST_SUITE(xlnx_gem_loopback, NULL, setup, NULL, NULL, teardown);
