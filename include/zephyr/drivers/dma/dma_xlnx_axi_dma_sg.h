/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors */
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
 * lifecycle, IRQ masking/re-enable, and ring re-arm policy. @p buf points into
 * the driver's RX buffer region, has already been cache-invalidated, and is
 * valid only for the duration of the callback; copy it if the data must
 * outlive the call. Do not write into it - see dma_xlnx_sg_get_buffer(). @p size is the
 * contiguous completed window span in bytes (`bd_bytes * irq_threshold`).
 */
typedef void (*dma_xlnx_sg_rx_stream_cb_t)(const struct device *dev, void *user_data, uint8_t *buf,
					   uint32_t size);

/**
 * @brief Callback invoked when the RX stream hits a DMA error.
 *
 * Called from system workqueue context with the DMASR value latched by the
 * ISR. The S2MM engine is halted at this point: the stream delivers no
 * further windows until the consumer stops and restarts it.
 */
typedef void (*dma_xlnx_sg_rx_stream_err_cb_t)(const struct device *dev, void *user_data,
					       uint32_t dmasr);

/**
 * @brief Continuous RX stream configuration.
 */
struct dma_xlnx_sg_rx_stream_cfg {
	uint32_t bd_bytes;
	uint16_t irq_threshold;
	dma_xlnx_sg_rx_stream_cb_t callback;
	void *user_data;
	/** Optional; NULL leaves DMA errors visible only through the status query. */
	dma_xlnx_sg_rx_stream_err_cb_t error_callback;
};

/**
 * @brief Continuous RX stream health.
 *
 * Everything a CONFIG_LOG=n image needs to explain a stream that stopped
 * delivering windows.
 */
struct dma_xlnx_sg_rx_stream_stats {
	bool active;           /**< a stream is armed */
	bool halted;           /**< S2MM DMASR.Halted — engine stopped */
	uint32_t dmasr;        /**< live DMASR */
	uint32_t last_error;   /**< DMASR at the most recent error IRQ, 0 = none */
	uint32_t error_count;  /**< error IRQs since the stream started */
	uint32_t overrun_count;/**< times TAILDESC was held back by a lagging consumer */
	uint32_t bds_produced; /**< BDs harvested by the ISR */
	uint32_t bds_consumed; /**< BDs released by the window consumer */
};

/**
 * @brief Reserve RX descriptors and storage for a finite session.
 *
 * Reserve before configuring the sampler or DMA. Keep the reservation through
 * setup, transfer completion and retained-buffer download; stop the transfer
 * before releasing it. Continuous stream start uses the same atomic owner.
 *
 * @param dev DMA device.
 * @retval 0 RX reserved.
 * @retval -EBUSY RX is reserved by another finite session or stream.
 * @retval -EIO A stopped mid-burst transfer could not be reset.
 */
int dma_xlnx_sg_reserve_rx(const struct device *dev);

/**
 * @brief Release a finite RX reservation after transfer and buffer use end.
 * @param dev DMA device.
 */
void dma_xlnx_sg_release_rx(const struct device *dev);

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

/**
 * @brief Query continuous RX stream health.
 *
 * @param dev   DMA device.
 * @param stats Output: stream and S2MM engine state.
 * @return 0 on success, -EINVAL if @p stats is NULL.
 */
int dma_xlnx_sg_rx_stream_status(const struct device *dev,
				 struct dma_xlnx_sg_rx_stream_stats *stats);

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
 * The RX region is mapped Normal cacheable. The driver invalidates the data it
 * reports as received - a stream window before its callback, a finite transfer
 * before its completion callback - so a consumer that reads only that data
 * needs no cache maintenance. Reading elsewhere in the region, or writing into
 * it at all, is the caller's own problem: the driver relies on the RX region
 * never holding a dirty line.
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
