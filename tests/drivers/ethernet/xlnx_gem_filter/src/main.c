/*
 * Copyright (c) 2026 Moton Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/igmp.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/ztest.h>

#include "eth_xlnx_gem_priv.h"

#define GEM_NODE DT_NODELABEL(gem0)
#define GEM_BASE DT_REG_ADDR(GEM_NODE)

#define BASELINE_HASHL BIT(31)
#define BASELINE_HASHH BIT(30)
#define IGMP_ALL_SYSTEMS_HASH BIT(6)
#define EXPECTED_HASHL BASELINE_HASHL
#define EXPECTED_HASHH (BASELINE_HASHH | IGMP_ALL_SYSTEMS_HASH)

static const struct device *gem_dev;
static const struct ethernet_api *gem_api;
static struct net_if *gem_iface;
static uint32_t baseline_nwcfg;
static struct net_if_mcast_monitor carrier_monitor;
static uint8_t carrier_join_count;
static uint8_t carrier_leave_count;

static const struct net_in_addr mdns_group = { { { 224, 0, 0, 251 } } };

static const struct net_eth_addr mdns_mac = {
	.addr = {0x01, 0x00, 0x5e, 0x00, 0x00, 0xfb},
};
static const struct net_eth_addr mdns_collision_mac = {
	.addr = {0x01, 0x00, 0x5e, 0x00, 0x00, 0x38},
};
static const struct net_eth_addr ipv4_all_systems_mac = {
	.addr = {0x01, 0x00, 0x5e, 0x00, 0x00, 0x01},
};
static const struct net_eth_addr ipv6_all_nodes_mac = {
	.addr = {0x33, 0x33, 0x00, 0x00, 0x00, 0x01},
};
static const struct net_eth_addr ipv6_solicited_node_mac = {
	.addr = {0x33, 0x33, 0xff, 0x00, 0x00, 0x42},
};
static const struct net_eth_addr baseline_hashl_collision_mac = {
	.addr = {0x01, 0x00, 0x5e, 0x00, 0x00, 0x26},
};
static const struct net_eth_addr baseline_hashh_collision_mac = {
	.addr = {0x01, 0x00, 0x5e, 0x00, 0x00, 0x20},
};

static void carrier_mcast_monitor(struct net_if *iface, const struct net_addr *addr,
				  bool is_joined)
{
	if (iface != gem_iface || addr->family != NET_AF_INET ||
	    !net_ipv4_addr_cmp(&addr->in_addr, &mdns_group)) {
		return;
	}

	if (is_joined) {
		carrier_join_count++;
	} else {
		carrier_leave_count++;
	}
}

static uint32_t read_reg(uint32_t offset)
{
	return sys_read32(GEM_BASE + offset);
}

static int configure_filter(const struct net_eth_addr *mac,
			    enum ethernet_filter_type type, bool set)
{
	struct ethernet_config config = {
		.filter = {
			.type = type,
			.set = set,
		},
	};

	memcpy(&config.filter.mac_address, mac, sizeof(*mac));
	return gem_api->set_config(gem_dev, gem_iface, ETHERNET_CONFIG_TYPE_FILTER, &config);
}

static int configure_promiscuous(bool enable)
{
	struct ethernet_config config = {
		.promisc_mode = enable,
	};

	return gem_api->set_config(gem_dev, gem_iface, ETHERNET_CONFIG_TYPE_PROMISC_MODE, &config);
}

/*
 * Model bootloader-owned/shared unicast hash entries. The GEM driver is a
 * POST_KERNEL device, so this runs before it captures the shared hash table.
 */
static int seed_hash_baseline(void)
{
	sys_write32(BASELINE_HASHL, GEM_BASE + ETH_XLNX_GEM_HASHL_OFFSET);
	sys_write32(BASELINE_HASHH, GEM_BASE + ETH_XLNX_GEM_HASHH_OFFSET);
	return 0;
}

SYS_INIT(seed_hash_baseline, PRE_KERNEL_1, 0);

static void *xlnx_gem_filter_setup(void)
{
	gem_dev = DEVICE_DT_GET(GEM_NODE);
	zassert_true(device_is_ready(gem_dev));

	gem_api = gem_dev->api;
	zassert_not_null(gem_api);
	zassert_not_null(gem_api->get_capabilities);
	zassert_not_null(gem_api->set_config);

	gem_iface = ((struct eth_xlnx_gem_dev_data *)gem_dev->data)->iface;
	zassert_not_null(gem_iface);

	baseline_nwcfg = read_reg(ETH_XLNX_GEM_NWCFG_OFFSET);
	zassert_equal(read_reg(ETH_XLNX_GEM_HASHL_OFFSET), EXPECTED_HASHL);
	zassert_equal(read_reg(ETH_XLNX_GEM_HASHH_OFFSET), EXPECTED_HASHH);
	zassert_true((baseline_nwcfg & ETH_XLNX_GEM_NWCFG_UCASTHASHEN_BIT) != 0U);
	zassert_true((baseline_nwcfg & ETH_XLNX_GEM_NWCFG_MCASTHASHEN_BIT) != 0U);
	zassert_equal(baseline_nwcfg & ETH_XLNX_GEM_NWCFG_COPYALLEN_BIT, 0U);

	return NULL;
}

static void xlnx_gem_filter_after(void *fixture)
{
	ARG_UNUSED(fixture);

	zassert_equal(read_reg(ETH_XLNX_GEM_HASHL_OFFSET), EXPECTED_HASHL);
	zassert_equal(read_reg(ETH_XLNX_GEM_HASHH_OFFSET), EXPECTED_HASHH);
	zassert_equal(read_reg(ETH_XLNX_GEM_NWCFG_OFFSET), baseline_nwcfg);
}

ZTEST(xlnx_gem_filter, test_capability)
{
	enum ethernet_hw_caps caps = gem_api->get_capabilities(gem_dev, gem_iface);

	zassert_true((caps & ETHERNET_HW_FILTERING) != 0U);
}

ZTEST(xlnx_gem_filter, test_known_hash_vectors_and_multiple_groups)
{
	zassert_ok(configure_filter(&mdns_mac, ETHERNET_FILTER_TYPE_DST_MAC_ADDRESS, true));
	zassert_equal(read_reg(ETH_XLNX_GEM_HASHH_OFFSET),
		      EXPECTED_HASHH | BIT(24));

	zassert_ok(configure_filter(&ipv4_all_systems_mac,
				    ETHERNET_FILTER_TYPE_DST_MAC_ADDRESS, true));
	zassert_equal(read_reg(ETH_XLNX_GEM_HASHH_OFFSET),
		      EXPECTED_HASHH | BIT(24));

	zassert_ok(configure_filter(&ipv6_all_nodes_mac,
				    ETHERNET_FILTER_TYPE_DST_MAC_ADDRESS, true));
	zassert_equal(read_reg(ETH_XLNX_GEM_HASHH_OFFSET),
		      EXPECTED_HASHH | BIT(24) | BIT(12));

	zassert_ok(configure_filter(&ipv6_solicited_node_mac,
				    ETHERNET_FILTER_TYPE_DST_MAC_ADDRESS, true));
	zassert_equal(read_reg(ETH_XLNX_GEM_HASHL_OFFSET), EXPECTED_HASHL | BIT(3));
	zassert_true((read_reg(ETH_XLNX_GEM_NWCFG_OFFSET) &
		      ETH_XLNX_GEM_NWCFG_MCASTHASHEN_BIT) != 0U);

	zassert_ok(configure_filter(&mdns_mac, ETHERNET_FILTER_TYPE_DST_MAC_ADDRESS, false));
	zassert_ok(configure_filter(&ipv4_all_systems_mac,
				    ETHERNET_FILTER_TYPE_DST_MAC_ADDRESS, false));
	zassert_ok(configure_filter(&ipv6_all_nodes_mac,
				    ETHERNET_FILTER_TYPE_DST_MAC_ADDRESS, false));
	zassert_true((read_reg(ETH_XLNX_GEM_NWCFG_OFFSET) &
		      ETH_XLNX_GEM_NWCFG_MCASTHASHEN_BIT) != 0U);
	zassert_ok(configure_filter(&ipv6_solicited_node_mac,
				    ETHERNET_FILTER_TYPE_DST_MAC_ADDRESS, false));
}

ZTEST(xlnx_gem_filter, test_last_membership_clears_multicast_gate)
{
	uint32_t nwcfg;

	zassert_ok(configure_filter(&ipv4_all_systems_mac,
				    ETHERNET_FILTER_TYPE_DST_MAC_ADDRESS, false));
	zassert_equal(read_reg(ETH_XLNX_GEM_HASHH_OFFSET), BASELINE_HASHH);

	nwcfg = read_reg(ETH_XLNX_GEM_NWCFG_OFFSET);
	zassert_equal(nwcfg, baseline_nwcfg & ~ETH_XLNX_GEM_NWCFG_MCASTHASHEN_BIT);

	zassert_ok(configure_filter(&ipv4_all_systems_mac,
				    ETHERNET_FILTER_TYPE_DST_MAC_ADDRESS, true));
}

ZTEST(xlnx_gem_filter, test_collision_repeat_and_underflow)
{
	zassert_ok(configure_filter(&mdns_mac, ETHERNET_FILTER_TYPE_DST_MAC_ADDRESS, true));
	zassert_ok(configure_filter(&mdns_mac, ETHERNET_FILTER_TYPE_DST_MAC_ADDRESS, true));
	zassert_ok(configure_filter(&mdns_collision_mac,
				    ETHERNET_FILTER_TYPE_DST_MAC_ADDRESS, true));
	zassert_equal(read_reg(ETH_XLNX_GEM_HASHH_OFFSET),
		      EXPECTED_HASHH | BIT(24));

	zassert_ok(configure_filter(&mdns_mac, ETHERNET_FILTER_TYPE_DST_MAC_ADDRESS, false));
	zassert_equal(read_reg(ETH_XLNX_GEM_HASHH_OFFSET),
		      EXPECTED_HASHH | BIT(24));
	zassert_ok(configure_filter(&mdns_collision_mac,
				    ETHERNET_FILTER_TYPE_DST_MAC_ADDRESS, false));
	zassert_equal(read_reg(ETH_XLNX_GEM_HASHH_OFFSET),
		      EXPECTED_HASHH | BIT(24));
	zassert_ok(configure_filter(&mdns_mac, ETHERNET_FILTER_TYPE_DST_MAC_ADDRESS, false));

	zassert_equal(configure_filter(&mdns_mac,
				       ETHERNET_FILTER_TYPE_DST_MAC_ADDRESS, false),
		      -ENOENT);
}

ZTEST(xlnx_gem_filter, test_never_added_collision_does_not_remove_live_entry)
{
	zassert_ok(configure_filter(&mdns_mac, ETHERNET_FILTER_TYPE_DST_MAC_ADDRESS, true));
	zassert_equal(configure_filter(&mdns_collision_mac,
				       ETHERNET_FILTER_TYPE_DST_MAC_ADDRESS, false),
		      -ENOENT);
	zassert_equal(read_reg(ETH_XLNX_GEM_HASHH_OFFSET),
		      EXPECTED_HASHH | BIT(24));
	zassert_true((read_reg(ETH_XLNX_GEM_NWCFG_OFFSET) &
		      ETH_XLNX_GEM_NWCFG_MCASTHASHEN_BIT) != 0U);
	zassert_ok(configure_filter(&mdns_mac, ETHERNET_FILTER_TYPE_DST_MAC_ADDRESS, false));
}

ZTEST(xlnx_gem_filter, test_membership_capacity)
{
	struct net_eth_addr mac = {
		.addr = {0x01, 0x00, 0x5e, 0x00, 0x00, 0x00},
	};

	for (uint8_t i = 0U; i < ETH_XLNX_GEM_MCAST_FILTER_SLOTS; i++) {
		mac.addr[5] = i;
		zassert_ok(configure_filter(&mac, ETHERNET_FILTER_TYPE_DST_MAC_ADDRESS, true));
	}

	mac.addr[4] = 0x01;
	zassert_equal(configure_filter(&mac, ETHERNET_FILTER_TYPE_DST_MAC_ADDRESS, true),
		      -ENOSPC);
	mac.addr[4] = 0x00;

	for (uint8_t i = 0U; i < ETH_XLNX_GEM_MCAST_FILTER_SLOTS; i++) {
		mac.addr[5] = i;
		zassert_ok(configure_filter(&mac, ETHERNET_FILTER_TYPE_DST_MAC_ADDRESS, false));
	}
}

ZTEST(xlnx_gem_filter, test_carrier_cycle_balances_managed_filter)
{
	zassert_ok(net_ipv4_igmp_join(gem_iface, &mdns_group, NULL));
	zassert_ok(configure_filter(&ipv6_solicited_node_mac,
				    ETHERNET_FILTER_TYPE_DST_MAC_ADDRESS, true));
	zassert_equal(read_reg(ETH_XLNX_GEM_HASHH_OFFSET),
		      EXPECTED_HASHH | BIT(24));
	zassert_equal(read_reg(ETH_XLNX_GEM_HASHL_OFFSET), EXPECTED_HASHL | BIT(3));

	carrier_join_count = 0U;
	carrier_leave_count = 0U;
	net_if_mcast_mon_register(&carrier_monitor, gem_iface, carrier_mcast_monitor);

	net_if_carrier_off(gem_iface);
	zassert_equal(carrier_leave_count, 1U);
	zassert_equal(carrier_join_count, 0U);
	zassert_equal(read_reg(ETH_XLNX_GEM_HASHL_OFFSET), EXPECTED_HASHL | BIT(3));
	zassert_equal(read_reg(ETH_XLNX_GEM_HASHH_OFFSET), BASELINE_HASHH);

	net_if_carrier_on(gem_iface);
	zassert_equal(carrier_leave_count, 1U);
	zassert_equal(carrier_join_count, 1U);
	zassert_equal(read_reg(ETH_XLNX_GEM_HASHL_OFFSET), EXPECTED_HASHL | BIT(3));
	zassert_equal(read_reg(ETH_XLNX_GEM_HASHH_OFFSET),
		      EXPECTED_HASHH | BIT(24));

	zassert_ok(net_ipv4_igmp_leave(gem_iface, &mdns_group));
	zassert_equal(carrier_leave_count, 2U);
	zassert_equal(carrier_join_count, 1U);
	zassert_equal(read_reg(ETH_XLNX_GEM_HASHH_OFFSET), EXPECTED_HASHH);
	zassert_equal(read_reg(ETH_XLNX_GEM_HASHL_OFFSET), EXPECTED_HASHL | BIT(3));

	net_if_mcast_mon_unregister(&carrier_monitor);
	zassert_ok(configure_filter(&ipv6_solicited_node_mac,
				    ETHERNET_FILTER_TYPE_DST_MAC_ADDRESS, false));
}

ZTEST(xlnx_gem_filter, test_preexisting_shared_hash_bits_are_preserved)
{
	zassert_ok(configure_filter(&baseline_hashl_collision_mac,
				    ETHERNET_FILTER_TYPE_DST_MAC_ADDRESS, true));
	zassert_ok(configure_filter(&baseline_hashh_collision_mac,
				    ETHERNET_FILTER_TYPE_DST_MAC_ADDRESS, true));
	zassert_equal(read_reg(ETH_XLNX_GEM_HASHL_OFFSET), EXPECTED_HASHL);
	zassert_equal(read_reg(ETH_XLNX_GEM_HASHH_OFFSET), EXPECTED_HASHH);

	zassert_ok(configure_filter(&baseline_hashl_collision_mac,
				    ETHERNET_FILTER_TYPE_DST_MAC_ADDRESS, false));
	zassert_ok(configure_filter(&baseline_hashh_collision_mac,
				    ETHERNET_FILTER_TYPE_DST_MAC_ADDRESS, false));
	zassert_equal(read_reg(ETH_XLNX_GEM_HASHL_OFFSET), EXPECTED_HASHL);
	zassert_equal(read_reg(ETH_XLNX_GEM_HASHH_OFFSET), EXPECTED_HASHH);
}

ZTEST(xlnx_gem_filter, test_unsupported_filters_do_not_change_hardware)
{
	static const struct net_eth_addr unicast_mac = {
		.addr = {0x02, 0x00, 0x00, 0x00, 0x00, 0x42},
	};
	static const struct net_eth_addr broadcast_mac = {
		.addr = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
	};

	zassert_equal(configure_filter(&mdns_mac,
				       ETHERNET_FILTER_TYPE_SRC_MAC_ADDRESS, true),
		      -ENOTSUP);
	zassert_equal(configure_filter(&unicast_mac,
				       ETHERNET_FILTER_TYPE_DST_MAC_ADDRESS, true),
		      -ENOTSUP);
	zassert_equal(configure_filter(&broadcast_mac,
				       ETHERNET_FILTER_TYPE_DST_MAC_ADDRESS, true),
		      -ENOTSUP);
}

ZTEST(xlnx_gem_filter, test_nwcfg_fields_are_preserved)
{
	uint32_t nwcfg;

	zassert_ok(configure_filter(&mdns_mac, ETHERNET_FILTER_TYPE_DST_MAC_ADDRESS, true));
	nwcfg = read_reg(ETH_XLNX_GEM_NWCFG_OFFSET);
	zassert_equal(nwcfg, baseline_nwcfg | ETH_XLNX_GEM_NWCFG_MCASTHASHEN_BIT);

	zassert_ok(configure_promiscuous(true));
	nwcfg = read_reg(ETH_XLNX_GEM_NWCFG_OFFSET);
	zassert_equal(nwcfg, baseline_nwcfg | ETH_XLNX_GEM_NWCFG_MCASTHASHEN_BIT |
				     ETH_XLNX_GEM_NWCFG_COPYALLEN_BIT);

	zassert_ok(configure_filter(&mdns_mac, ETHERNET_FILTER_TYPE_DST_MAC_ADDRESS, false));
	nwcfg = read_reg(ETH_XLNX_GEM_NWCFG_OFFSET);
	zassert_equal(nwcfg, baseline_nwcfg | ETH_XLNX_GEM_NWCFG_COPYALLEN_BIT);

	zassert_ok(configure_promiscuous(false));
}

ZTEST_SUITE(xlnx_gem_filter, NULL, xlnx_gem_filter_setup, NULL,
	    xlnx_gem_filter_after, NULL);
