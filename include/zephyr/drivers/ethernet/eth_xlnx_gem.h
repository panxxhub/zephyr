/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_ETHERNET_ETH_XLNX_GEM_H_
#define ZEPHYR_INCLUDE_DRIVERS_ETHERNET_ETH_XLNX_GEM_H_

#include <zephyr/device.h>
#include <stdint.h>

#define ETH_XLNX_GEM_RECOVERY_TX_TIMEOUT (1U << 31)
#define ETH_XLNX_GEM_RECOVERY_RX_CHAIN   (1U << 30)
#define ETH_XLNX_GEM_RECOVERY_TX_CHAIN   (1U << 29)

/** Packet-path counters and the last fault snapshot, saved before clearing status. */
struct eth_xlnx_gem_diagnostics {
	uint32_t recoveries;
	uint32_t recovery_failures;
	uint32_t rx_orphans;
	uint32_t rx_dropped;
	uint32_t rx_work_calls;
	uint32_t rx_work_max_us;
	uint32_t rx_budget_hits;
	uint32_t rx_backoffs;
	uint32_t rx_used;
	uint32_t rx_overruns;
	uint32_t hresp_errors;
	uint32_t tx_timeouts;
	uint32_t reasons;
	uint32_t isr;
	uint32_t imr;
	uint32_t rxsr;
	uint32_t txsr;
	uint32_t nwctrl;
	uint32_t nwcfg;
	uint32_t rxqbase;
	uint32_t txqbase;
	uint32_t rx_head_addr;
	uint32_t rx_head_ctrl;
	uint32_t tx_head_addr;
	uint32_t tx_head_ctrl;
	uint8_t rx_next;
	uint8_t tx_next;
	uint8_t tx_free;
};

/**
 * @brief Copy the GEM packet-path diagnostics under the driver lock.
 * @param dev Initialized Xilinx GEM Ethernet device.
 * @param diagnostics Destination snapshot.
 * @retval 0 Snapshot copied.
 * @retval -EINVAL Null destination.
 */
int eth_xlnx_gem_get_diagnostics(const struct device *dev,
			       struct eth_xlnx_gem_diagnostics *diagnostics);

#endif /* ZEPHYR_INCLUDE_DRIVERS_ETHERNET_ETH_XLNX_GEM_H_ */
