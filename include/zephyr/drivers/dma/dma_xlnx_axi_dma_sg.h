/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Xilinx AXI DMA SG driver — device-specific extensions.
 *
 * These functions supplement the standard dma_*() API for features
 * not covered by struct dma_driver_api: APP field access, cyclic SG
 * reconfiguration, and RX frame length query.
 */

#ifndef ZEPHYR_DRIVERS_DMA_XLNX_AXI_DMA_SG_H_
#define ZEPHYR_DRIVERS_DMA_XLNX_AXI_DMA_SG_H_

#include <zephyr/device.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** APP field block — mirrors the 5 user application words in PG021 SG descriptors. */
struct dma_xlnx_sg_app_fields {
	uint32_t app[5];
};

/**
 * @brief Reconfigure the RX SG ring at runtime.
 *
 * Stops the S2MM channel, rebuilds the BD ring with the given per-BD
 * buffer size and IRQ threshold, then restarts.  The total ring capacity
 * (bd_buf_bytes * num_bds) must not exceed the RX buffer region size.
 *
 * @param dev       DMA device.
 * @param bd_bytes  Bytes per BD buffer.
 * @param threshold IRQ coalescing threshold (must divide num_bds evenly).
 * @return 0 on success, -EINVAL on bad parameters, -EIO on halt timeout.
 */
int dma_xlnx_sg_reconfigure_rx(const struct device *dev,
				uint32_t bd_bytes, uint8_t threshold);

/**
 * @brief Prepare RX channel for cyclic mode without starting.
 *
 * Sets ch->cyclic and ch->callback so that reconfigure_rx can build
 * and kick the channel with CYC_BD_EN and IRQ callback enabled.
 * Does NOT build BDs or write any hardware registers.
 */
void dma_xlnx_sg_prepare_rx_cyclic(const struct device *dev,
				    dma_callback_t callback, void *user_data);

/**
 * @brief Re-enable RX IOC/DLY IRQs after cyclic ISR disabled them.
 */
void dma_xlnx_sg_reenable_rx_irq(const struct device *dev);

/**
 * @brief Read APP fields from the most recently completed RX descriptor.
 *
 * @param dev  DMA device.
 * @param app  Output: APP0-APP4 from the completed descriptor.
 * @return 0 on success, -EAGAIN if no completed descriptor available.
 */
int dma_xlnx_sg_get_rx_app(const struct device *dev,
			    struct dma_xlnx_sg_app_fields *app);

/**
 * @brief Set APP fields for the next TX descriptor (control stream).
 *
 * @param dev  DMA device.
 * @param app  APP0-APP4 to write into the SOF descriptor.
 * @return 0 on success.
 */
int dma_xlnx_sg_set_tx_app(const struct device *dev,
			    const struct dma_xlnx_sg_app_fields *app);

/**
 * @brief Get the byte count of the last completed RX transfer.
 *
 * Reads the transferred byte count from the BD status field (bits [25:0]).
 *
 * @param dev  DMA device.
 * @return Byte count, or 0 if no transfer completed yet.
 */
uint32_t dma_xlnx_sg_last_rx_bytes(const struct device *dev);

/**
 * @brief Get buffer region addresses for a channel.
 *
 * Returns both the physical address (for hardware/BD programming) and
 * the MMU-mapped CPU-accessible address.
 *
 * @param dev      DMA device.
 * @param channel  0 = TX, 1 = RX.
 * @param phys     Output: physical base address.
 * @param virt     Output: CPU-accessible (MMU-mapped) base address.
 * @param size     Output: buffer region size in bytes.
 * @return 0 on success, -EINVAL if channel invalid.
 */
int dma_xlnx_sg_get_buffer(const struct device *dev, uint32_t channel,
			    uintptr_t *phys, uintptr_t *virt, size_t *size);

/** Completed BD window descriptor — one entry per BD in the window. */
struct dma_xlnx_sg_rx_bd_info {
	uintptr_t buf_virt;                     /* CPU-accessible buffer pointer */
	uint32_t byte_count;                    /* Bytes transferred (from status[25:0]) */
	struct dma_xlnx_sg_app_fields app;      /* APP0-APP4 from completed descriptor */
};

/**
 * @brief Get the completed RX BD window after an IRQ.
 *
 * In cyclic mode with IRQ coalescing, each IRQ fires after `threshold`
 * BDs complete.  This function returns info for each BD in the window.
 * The caller provides an array of at least `threshold` entries.
 *
 * After the caller has processed the window, it must call
 * dma_xlnx_sg_release_rx_window() to advance the consumer index.
 *
 * @param dev       DMA device.
 * @param info      Output array (caller-allocated, length >= threshold).
 * @param count     Input: array capacity.  Output: number of entries filled.
 * @return 0 on success, -EAGAIN if no complete window, -EINVAL on error.
 */
int dma_xlnx_sg_get_rx_window(const struct device *dev,
			       struct dma_xlnx_sg_rx_bd_info *info,
			       uint32_t *count);

/**
 * @brief Release an RX BD window after processing.
 *
 * Advances the consumer index so the DMA engine can reuse the descriptors.
 * In cyclic mode with CYC_BD_EN, the CMPLT bits are ignored by hardware,
 * but this function updates the driver's internal tracking.
 *
 * @param dev    DMA device.
 * @param count  Number of BDs to release (must match get_rx_window output).
 * @return 0 on success.
 */
int dma_xlnx_sg_release_rx_window(const struct device *dev, uint32_t count);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_DMA_XLNX_AXI_DMA_SG_H_ */
