/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Xilinx AXI DMA SG driver — device-specific extensions.
 *
 * These functions supplement the standard dma_*() API for features
 * not covered by struct dma_driver_api: APP field access, RX stream
 * start/stop, and RX frame length query.
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
 * @brief Callback invoked when an RX stream window is ready.
 *
 * Called from system workqueue context. The driver owns SG descriptor
 * lifecycle, IRQ masking/re-enable, and ring re-arm policy.
 */
typedef void (*dma_xlnx_sg_rx_stream_cb_t)(const struct device *dev, void *user_data, uint8_t *buf,
					   uint32_t size);

/**
 * @brief Continuous RX stream configuration.
 */
struct dma_xlnx_sg_rx_stream_cfg {
	uint32_t bd_bytes;
	uint8_t irq_threshold;
	dma_xlnx_sg_rx_stream_cb_t callback;
	void *user_data;
};

/**
 * @brief Start a continuous RX stream.
 *
 * The driver configures the RX SG ring, manages descriptor completion
 * windows internally, and invokes @p callback once per completed window.
 *
 * @param dev DMA device.
 * @param cfg Stream configuration and callback binding.
 * @return 0 on success, negative errno on failure.
 */
int dma_xlnx_sg_start_rx_stream(const struct device *dev,
				const struct dma_xlnx_sg_rx_stream_cfg *cfg);

/**
 * @brief Stop the active continuous RX stream.
 *
 * Safe to call when no stream is active.
 *
 * @param dev DMA device.
 */
void dma_xlnx_sg_stop_rx_stream(const struct device *dev);

#ifdef CONFIG_DMA_XLNX_AXI_DMA_SG_APP_FIELDS
/**
 * @brief Read APP fields from the most recently completed RX descriptor.
 *
 * @param dev  DMA device.
 * @param app  Output: APP0-APP4 from the completed descriptor.
 * @return 0 on success, -EAGAIN if no completed descriptor available.
 */
int dma_xlnx_sg_get_rx_app(const struct device *dev, struct dma_xlnx_sg_app_fields *app);

/**
 * @brief Set APP fields for the next TX descriptor (control stream).
 *
 * @param dev  DMA device.
 * @param app  APP0-APP4 to write into the SOF descriptor.
 * @return 0 on success.
 */
int dma_xlnx_sg_set_tx_app(const struct device *dev, const struct dma_xlnx_sg_app_fields *app);
#endif

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
int dma_xlnx_sg_get_buffer(const struct device *dev, uint32_t channel, uintptr_t *phys,
			   uintptr_t *virt, size_t *size);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_DMA_XLNX_AXI_DMA_SG_H_ */
