/*
 * Copyright (c) 2025 Moton Intelligent Equipment
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zynq-7000 PS Quad-SPI controller driver (polling mode).
 * Reference: UG585 Zynq-7000 TRM, Chapter 12.
 */

#define DT_DRV_COMPAT xlnx_zynq_qspi

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(spi_xlnx_zynq_qspi, CONFIG_SPI_LOG_LEVEL);

/* spi_context.h has inline functions with LOG calls — must come after
 * LOG_MODULE_REGISTER so the log module symbols are available.
 */
#include "spi_context.h"

/* ── Register offsets ───────────────────────────────────────────── */
#define QSPI_CR		0x00 /* Configuration */
#define QSPI_ISR	0x04 /* Interrupt Status */
#define QSPI_IER	0x08 /* Interrupt Enable */
#define QSPI_IDR	0x0C /* Interrupt Disable */
#define QSPI_IMR	0x10 /* Interrupt Mask */
#define QSPI_ER		0x14 /* Enable */
#define QSPI_DR		0x18 /* Delay */
#define QSPI_TXD0	0x1C /* TX Data (4 bytes) */
#define QSPI_RXD	0x20 /* RX Data */
#define QSPI_SICR	0x24 /* Slave Idle Count */
#define QSPI_TXWR	0x28 /* TX Threshold */
#define QSPI_RXWR	0x2C /* RX Threshold */
#define QSPI_TXD1	0x80 /* TX Data (1 byte) */
#define QSPI_TXD2	0x84 /* TX Data (2 bytes) */
#define QSPI_TXD3	0x88 /* TX Data (3 bytes) */
#define QSPI_LQSPI_CR	0xA0 /* Linear QSPI Config */
#define QSPI_LQSPI_SR	0xA4 /* Linear QSPI Status */

/* ── CR bit definitions ─────────────────────────────────────────── */
#define CR_MSTREN	BIT(0)	       /* Master mode */
#define CR_CPOL		BIT(1)	       /* Clock polarity */
#define CR_CPHA		BIT(2)	       /* Clock phase */
#define CR_BAUD_SHIFT	3
#define CR_BAUD_MASK	GENMASK(5, 3)  /* Baud rate divider */
#define CR_REF_CLK	BIT(8)	       /* Reference clock select */
#define CR_PCS		BIT(10)	       /* Peripheral chip select (0=assert) */
#define CR_MANCS	BIT(14)	       /* Manual chip select */
#define CR_MANSTARTEN	BIT(15)	       /* Manual start enable */
#define CR_MANSTARTCOM	BIT(16)	       /* Manual start command (trigger) */
#define CR_ENDIAN	BIT(26)	       /* 0=little, 1=big */
#define CR_IFMODE	BIT(31)	       /* 0=I/O mode, 1=linear mode */

/* ── ISR/IER bit definitions ────────────────────────────────────── */
#define ISR_RX_OVF	BIT(0) /* RX overflow */
#define ISR_TX_NFULL	BIT(2) /* TX FIFO not full */
#define ISR_TX_FULL	BIT(3) /* TX FIFO full */
#define ISR_RX_NEMPTY	BIT(4) /* RX FIFO not empty */
#define ISR_RX_FULL	BIT(5) /* RX FIFO full */
#define ISR_TX_UF	BIT(6) /* TX FIFO underflow */

/* ── ER bit definitions ─────────────────────────────────────────── */
#define ER_ENABLE	BIT(0)

/* ── Structures ─────────────────────────────────────────────────── */

struct zynq_qspi_config {
	DEVICE_MMIO_ROM;
	uint32_t clock_frequency;
	const struct pinctrl_dev_config *pincfg;
};

struct zynq_qspi_data {
	DEVICE_MMIO_RAM;
	struct spi_context ctx;
};

/* ── Register access ────────────────────────────────────────────── */

static inline uint32_t qspi_read(const struct device *dev, uint32_t reg)
{
	return sys_read32(DEVICE_MMIO_GET(dev) + reg);
}

static inline void qspi_write(const struct device *dev, uint32_t reg,
			       uint32_t val)
{
	sys_write32(val, DEVICE_MMIO_GET(dev) + reg);
}

/* ── Configuration ──────────────────────────────────────────────── */

static int zynq_qspi_configure(const struct device *dev,
				const struct spi_config *config)
{
	const struct zynq_qspi_config *cfg = dev->config;
	struct zynq_qspi_data *data = dev->data;
	uint32_t cr;

	if (spi_context_configured(&data->ctx, config)) {
		return 0;
	}

	if (config->operation & SPI_OP_MODE_SLAVE) {
		LOG_ERR("Slave mode not supported");
		return -ENOTSUP;
	}

	if (config->operation & SPI_HALF_DUPLEX) {
		LOG_ERR("Half-duplex not supported");
		return -ENOTSUP;
	}

	if (SPI_WORD_SIZE_GET(config->operation) != 8) {
		LOG_ERR("Only 8-bit word size supported");
		return -ENOTSUP;
	}

	/* Compute baud rate divider: freq = ref_clk / (2 << baud_div) */
	uint32_t baud_div = 0;

	while (baud_div < 7 &&
	       (cfg->clock_frequency / (2U << baud_div)) > config->frequency) {
		baud_div++;
	}

	/* Build CR: master, manual CS (deasserted), manual start enable */
	cr = CR_MSTREN | CR_MANCS | CR_MANSTARTEN | CR_PCS;

	if (config->operation & SPI_MODE_CPOL) {
		cr |= CR_CPOL;
	}
	if (config->operation & SPI_MODE_CPHA) {
		cr |= CR_CPHA;
	}

	cr |= (baud_div << CR_BAUD_SHIFT) & CR_BAUD_MASK;

	qspi_write(dev, QSPI_CR, cr);
	data->ctx.config = config;

	return 0;
}

/* ── CS control ─────────────────────────────────────────────────── */

static void zynq_qspi_cs_assert(const struct device *dev)
{
	uint32_t cr = qspi_read(dev, QSPI_CR);

	cr &= ~CR_PCS; /* PCS=0 -> CS asserted (active low) */
	qspi_write(dev, QSPI_CR, cr);
}

static void zynq_qspi_cs_deassert(const struct device *dev)
{
	uint32_t cr = qspi_read(dev, QSPI_CR);

	cr |= CR_PCS; /* PCS=1 -> CS deasserted */
	qspi_write(dev, QSPI_CR, cr);
}

/* ── Transfer (polling, byte-at-a-time) ─────────────────────────── */

static int zynq_qspi_transceive(const struct device *dev,
				 const struct spi_config *config,
				 const struct spi_buf_set *tx_bufs,
				 const struct spi_buf_set *rx_bufs)
{
	struct zynq_qspi_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	int rc;

	spi_context_lock(ctx, false, NULL, NULL, config);

	rc = zynq_qspi_configure(dev, config);
	if (rc != 0) {
		spi_context_release(ctx, rc);
		return rc;
	}

	spi_context_buffers_setup(ctx, tx_bufs, rx_bufs, 1);

	/* Assert CS */
	if (spi_cs_is_gpio(config)) {
		spi_context_cs_control(ctx, true);
	} else {
		zynq_qspi_cs_assert(dev);
	}

	/* Transfer bytes */
	while (spi_context_tx_on(ctx) || spi_context_rx_on(ctx)) {
		uint8_t tx_byte = 0;

		if (spi_context_tx_buf_on(ctx)) {
			tx_byte = *ctx->tx_buf;
		}

		/* Write to 1-byte TX register */
		qspi_write(dev, QSPI_TXD1, tx_byte);

		/* Trigger transfer (manual start) */
		uint32_t cr = qspi_read(dev, QSPI_CR);

		qspi_write(dev, QSPI_CR, cr | CR_MANSTARTCOM);

		/* Poll for RX data */
		uint32_t timeout = 10000;

		while (!(qspi_read(dev, QSPI_ISR) & ISR_RX_NEMPTY)) {
			if (--timeout == 0) {
				LOG_ERR("RX timeout");
				rc = -ETIMEDOUT;
				goto done;
			}
		}

		/* Read RX byte and clear status */
		uint8_t rx_byte = (uint8_t)qspi_read(dev, QSPI_RXD);

		qspi_write(dev, QSPI_ISR, ISR_RX_NEMPTY);

		if (spi_context_rx_buf_on(ctx)) {
			*ctx->rx_buf = rx_byte;
		}

		spi_context_update_tx(ctx, 1, 1);
		spi_context_update_rx(ctx, 1, 1);
	}

	rc = 0;

done:
	/* Deassert CS */
	if (!(config->operation & SPI_HOLD_ON_CS)) {
		if (spi_cs_is_gpio(config)) {
			spi_context_cs_control(ctx, false);
		} else {
			zynq_qspi_cs_deassert(dev);
		}
	}

	spi_context_complete(ctx, dev, rc);
	spi_context_release(ctx, rc);
	return rc;
}

static int zynq_qspi_release(const struct device *dev,
			      const struct spi_config *config)
{
	struct zynq_qspi_data *data = dev->data;

	spi_context_unlock_unconditionally(&data->ctx);
	return 0;
}

/* ── Init ───────────────────────────────────────────────────────── */

static int zynq_qspi_init(const struct device *dev)
{
	const struct zynq_qspi_config *cfg = dev->config;
	struct zynq_qspi_data *data = dev->data;
	int rc;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	rc = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (rc < 0 && rc != -ENOENT) {
		LOG_ERR("Pinctrl apply failed: %d", rc);
		return rc;
	}

	/* Disable linear QSPI mode (use I/O mode for programming) */
	qspi_write(dev, QSPI_LQSPI_CR, 0);

	/* Disable SPI, configure, then enable */
	qspi_write(dev, QSPI_ER, 0);
	qspi_write(dev, QSPI_CR,
		    CR_MSTREN | CR_MANCS | CR_MANSTARTEN | CR_PCS);

	/* Disable all interrupts (polling mode) */
	qspi_write(dev, QSPI_IDR, 0x7F);
	/* Clear any pending status */
	qspi_write(dev, QSPI_ISR, qspi_read(dev, QSPI_ISR));

	/* Enable SPI */
	qspi_write(dev, QSPI_ER, ER_ENABLE);

	spi_context_unlock_unconditionally(&data->ctx);

	LOG_INF("Zynq PS QSPI initialized (ref_clk=%u Hz)",
		cfg->clock_frequency);
	return 0;
}

/* ── API + DT instantiation ─────────────────────────────────────── */

static DEVICE_API(spi, zynq_qspi_api) = {
	.transceive = zynq_qspi_transceive,
	.release = zynq_qspi_release,
#ifdef CONFIG_SPI_RTIO
	.iodev_submit = spi_rtio_iodev_default_submit,
#endif
};

#define ZYNQ_QSPI_INIT(n)                                                     \
	PINCTRL_DT_INST_DEFINE(n);                                             \
	static struct zynq_qspi_config zynq_qspi_cfg_##n = {                  \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                         \
		.clock_frequency = DT_INST_PROP(n, clock_frequency),           \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                  \
	};                                                                     \
	static struct zynq_qspi_data zynq_qspi_data_##n = {                   \
		SPI_CONTEXT_INIT_LOCK(zynq_qspi_data_##n, ctx),               \
		SPI_CONTEXT_INIT_SYNC(zynq_qspi_data_##n, ctx),               \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx)          \
	};                                                                     \
	SPI_DEVICE_DT_INST_DEFINE(n, zynq_qspi_init, NULL,                    \
				  &zynq_qspi_data_##n, &zynq_qspi_cfg_##n,    \
				  POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,       \
				  &zynq_qspi_api);

DT_INST_FOREACH_STATUS_OKAY(ZYNQ_QSPI_INIT)
