/*
 * Copyright (c) 2025 Moton Intelligent Equipment
 * SPDX-License-Identifier: Apache-2.0
 *
 * Public API for the Zynq-7000 PS Quad-SPI controller's LQSPI
 * (linear / memory-mapped) read mode.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SPI_XLNX_ZYNQ_QSPI_H_
#define ZEPHYR_INCLUDE_DRIVERS_SPI_XLNX_ZYNQ_QSPI_H_

#include <stddef.h>
#include <sys/types.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief LQSPI Quad Output Fast Read (0x6B), memory-mapped.
 *
 * Reconfigures the controller into LQSPI mode (read-only, hardware-issued
 * 0x6B with 1 dummy byte), memcpy's from the controller's linear aperture
 * at 0xFC000000, then restores manual I/O mode. The SPI context lock is
 * held across the operation so it is mutually exclusive with
 * spi_transceive() on the same controller.
 *
 * Constraints:
 *   - addr + len must fit within the lower 16 MiB of flash (Zynq-7000
 *     LQSPI aperture is 16 MiB; bank-select for the upper 16 MiB is not
 *     yet supported here).
 *   - Caller must ensure the flash chip's quad-enable bit is set (e.g.
 *     W25Q SR2.QE) before invoking — the driver does not configure the
 *     flash side.
 *   - Caller must hold the underlying flash device acquired/released as
 *     usual via the flash subsystem; this function does not interact
 *     with flash-level locks beyond the SPI context lock.
 *
 * @param dev xlnx,zynq-qspi controller device
 * @param addr Flash byte offset (0 .. 0x00FFFFFF)
 * @param dst Destination buffer
 * @param len Bytes to read
 *
 * @retval 0 success
 * @retval -EINVAL addr/len out of range or dev/dst NULL
 * @retval -EIO MMU mapping of the LQSPI aperture failed
 */
int xlnx_zynq_qspi_lqspi_read(const struct device *dev,
			      off_t addr, void *dst, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SPI_XLNX_ZYNQ_QSPI_H_ */
