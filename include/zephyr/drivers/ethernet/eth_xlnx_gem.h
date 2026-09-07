/*
 * Copyright (c) 2026 Xiang Pan
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file
 * @brief Xilinx GEM service-liveness interface.
 */

#ifndef ZEPHYR_DRIVERS_ETHERNET_ETH_XLNX_GEM_H_
#define ZEPHYR_DRIVERS_ETHERNET_ETH_XLNX_GEM_H_

#include <zephyr/device.h>
#include <stdint.h>

/**
 * @brief Read the GEM service heartbeat.
 *
 * Pending RX/TX must progress; idle rings are healthy.
 *
 * @param dev GEM device.
 * @return Monotonic service progress counter.
 */
uint32_t eth_xlnx_gem_heartbeat(const struct device *dev);

#endif
