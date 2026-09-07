/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_DRIVERS_ETHERNET_ETH_XLNX_GEM_H_
#define ZEPHYR_DRIVERS_ETHERNET_ETH_XLNX_GEM_H_
#include <zephyr/device.h>
#include <stdint.h>
/** Service-loop heartbeat; pending RX/TX must progress, idle rings are healthy. */
uint32_t eth_xlnx_gem_heartbeat(const struct device *dev);
#endif
