/*
 * Xilinx Processor System Gigabit Ethernet controller (GEM) driver
 *
 * Diagnostic counters
 *
 * Copyright (c) 2026, Moton Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_ETHERNET_ETH_XLNX_GEM_H_
#define ZEPHYR_INCLUDE_DRIVERS_ETHERNET_ETH_XLNX_GEM_H_

#include <stdint.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Events the Xilinx GEM driver counts.
 *
 * The counters are always compiled in: they cost one increment on paths the
 * driver only reaches when the event has already happened, and no counter is
 * read or written on the per-frame receive or transmit path. Each counter is
 * a free-running 32-bit value that wraps; a reader interested in a rate takes
 * two samples and subtracts.
 *
 * Every counter has a single writer in any one build. The receive counters are
 * written from the interrupt handler and the receive handler, which run either
 * both in the interrupt or both on the driver's work queue, never at the same
 * time on two CPUs. The transmit counters are written from the send function,
 * which CONFIG_ETH_XLNX_GEM_TX_RECLAIM serializes with the transmit lock;
 * without that option concurrent senders exist, and an increment lost between
 * two of them costs an event, never correctness.
 */
struct eth_xlnx_gem_stats {
	/** Receive overrun indications reported by the controller. */
	uint32_t rx_overruns;
	/** Receive buffer-not-available indications reported by the controller. */
	uint32_t rx_buffer_not_available;
	/** Times the receive queue was rebuilt and restarted at descriptor zero. */
	uint32_t rx_queue_resets;
	/** Frames thrown away by those rebuilds: descriptors the controller had
	 *  written and the driver had not yet delivered.
	 */
	uint32_t rx_reset_discards;
	/** Transmissions for which no completion arrived within the timeout. */
	uint32_t tx_send_timeouts;
	/** Transmissions whose descriptors were returned to the ring by age,
	 *  which only CONFIG_ETH_XLNX_GEM_TX_ASYNC does.
	 */
	uint32_t tx_age_reclaims;
	/** Times the send function found too few free descriptors. With
	 *  CONFIG_ETH_XLNX_GEM_TX_ASYNC this is a back-pressure wait, otherwise
	 *  the frame is dropped.
	 */
	uint32_t tx_ring_full;
};

/**
 * @brief Reads the diagnostic counters of a Xilinx GEM controller.
 *
 * Available without the network statistics subsystem, so that firmware can
 * report the counters over its own interface. The copy is not atomic as a
 * whole: each counter is read once, so two counters in the returned set may
 * belong to two different instants.
 *
 * @param dev   GEM device
 * @param stats Destination for the counters
 * @retval 0 on success
 * @retval -EINVAL if either argument is NULL
 * @retval -ENOTSUP if @p dev is not a Xilinx GEM controller
 */
int eth_xlnx_gem_stats_get(const struct device *dev, struct eth_xlnx_gem_stats *stats);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_ETHERNET_ETH_XLNX_GEM_H_ */
