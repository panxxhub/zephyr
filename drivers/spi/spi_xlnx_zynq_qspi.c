/*
 * Copyright (c) 2025 Moton Intelligent Equipment
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zynq-7000 PS Quad-SPI controller driver (polling mode, FIFO burst).
 * Reference: UG585 Zynq-7000 TRM, Chapter 12.
 *
 * Full-duplex SPI controller — RX bytes are produced for every TX byte.
 * Callers that send command/address in TX and expect payload-only in RX
 * MUST use NOP RX buffers (buf=NULL) for the command phase, as spi_nor.c
 * does. Callers that omit NOP segments will get command-phase junk in
 * the RX buffer.
 *
 * Throughput model: the controller has a 63-deep × 32-bit TX FIFO and a
 * matching RX FIFO. transceive() chunks each transaction into bursts of
 * up to 252 bytes (63 slots × 4 bytes via TXD0). Each burst writes all
 * slots into the TX FIFO, fires a single MANSTARTCOM, then drains the
 * RX FIFO. This amortises the per-byte register-write overhead across
 * the burst — the previous TXD1 byte-at-a-time path required one
 * MANSTARTCOM + one ISR poll per byte.
 */

#define DT_DRV_COMPAT xlnx_zynq_qspi

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/spi/xlnx_zynq_qspi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/util.h>
#include <string.h>

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
#define CR_FIFO_WIDTH	(0x3 << 6)     /* bits[7:6]: data word size. Must be
					* 0x3 (32-bit) for the controller to
					* function — reset state from BootROM
					* may leave it at 0 (reserved/invalid),
					* in which case TXD/RXD bytes don't
					* shift correctly and reads return 0. */
#define CR_REF_CLK	BIT(8)	       /* Reference clock select */
#define CR_PCS		BIT(10)	       /* Peripheral chip select (0=assert) */
#define CR_MANCS	BIT(14)	       /* Manual chip select */
#define CR_MANSTARTEN	BIT(15)	       /* Manual start enable */
#define CR_MANSTARTCOM	BIT(16)	       /* Manual start command (trigger) */
#define CR_HOLDB_DR	BIT(19)	       /* Drive HOLD_B pin high (must be set
					* whenever a SPI-NOR flash is the slave —
					* otherwise the chip's HOLD function pulls
					* MISO low and reads return 0x00). Matches
					* XQSPIPS_HOLD_B_DRIVE_OPTION in the BSP. */
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

/* ── LQSPI_CR bit definitions (Linear QSPI mode) ──────────────────
 * UG585 Table 12-7. With LQSPI_CR.LINEAR_MASK set, the controller
 * auto-issues the configured read instruction whenever the AXI
 * master accesses the linear aperture at 0xFC000000.
 */
#define LQSPI_CR_LINEAR_MASK	BIT(31) /* LQSPI mode enable */
#define LQSPI_CR_TWO_MEM_MASK	BIT(30) /* Both memories (stacked) */
#define LQSPI_CR_SEP_BUS_MASK	BIT(29) /* Separate memory bus */
#define LQSPI_CR_U_PAGE_MASK	BIT(28) /* Upper memory page */
#define LQSPI_CR_MODE_EN_MASK	BIT(25) /* Enable mode bits */
#define LQSPI_CR_MODE_ON_MASK	BIT(24) /* Mode on */
#define LQSPI_CR_DUMMY_SHIFT	8        /* bits[10:8]: dummy bytes */
#define LQSPI_CR_INST_MASK	0xFFU    /* Read instruction code */

/* Quad Output Fast Read (0x6B) with 1 dummy byte — Xilinx BSP default
 * for SINGLE_QSPI_CONFIG_FAST_QUAD_READ. On opus_one_75s + W25Q256FV
 * this currently produces correct data with a 4-byte 0xFF prefix and
 * possibly some byte-shift; full clean-read tuning needs a logic
 * analyzer and explicit QE bit setup (volatile-write 0x50/0x31/0x02
 * on the W25Q to enable quad output). See follow-up issue.
 */
#define LQSPI_CR_QUAD_FAST_READ							\
	(LQSPI_CR_LINEAR_MASK | (1U << LQSPI_CR_DUMMY_SHIFT) | 0x6BU)

/* LQSPI aperture (Zynq-7000): 16 MiB at 0xFC000000. */
#define QSPI_LQSPI_APERTURE_PA		0xFC000000UL
#define QSPI_LQSPI_APERTURE_SIZE	0x01000000UL

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
	cr = CR_MSTREN | CR_MANCS | CR_MANSTARTEN | CR_HOLDB_DR | CR_FIFO_WIDTH | CR_PCS;

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

/* ── FIFO burst transfer ─────────────────────────────────────────── */

#define QSPI_FIFO_DEPTH 63U                          /* TX/RX FIFO words */
#define QSPI_BURST_MAX  (QSPI_FIFO_DEPTH * 4U)        /* 252 bytes per burst */
#define QSPI_RX_TIMEOUT 100000U                       /* polling iterations */

/* Pack 1..4 source bytes (LE) into a TXDn write. Byte 0 is sent first on
 * the SPI bus. TXD0 takes 4 bytes, TXD1/2/3 take 1/2/3 bytes (the rest of
 * the 32-bit word is don't-care).
 */
static inline void zynq_qspi_txd_write(const struct device *dev,
				       const uint8_t *src, uint8_t width)
{
	uint32_t word = (uint32_t)src[0];

	if (width >= 2U) {
		word |= (uint32_t)src[1] << 8;
	}
	if (width >= 3U) {
		word |= (uint32_t)src[2] << 16;
	}
	if (width == 4U) {
		word |= (uint32_t)src[3] << 24;
	}

	switch (width) {
	case 1U: qspi_write(dev, QSPI_TXD1, word); break;
	case 2U: qspi_write(dev, QSPI_TXD2, word); break;
	case 3U: qspi_write(dev, QSPI_TXD3, word); break;
	case 4U: qspi_write(dev, QSPI_TXD0, word); break;
	}
}

/* Unpack one RX FIFO entry (32 bits) into 1..4 dst bytes.
 *
 * The RX shifter is right-shift-into-top: each received bit enters at
 * bit 31 and shifts existing bits right by 1, so an N-byte transfer
 * leaves the data top-aligned in the FIFO slot. After N bytes received
 * (in chronological order b0..b{N-1}):
 *
 *   width=4 (TXD0): RXD = (b3<<24)|(b2<<16)|(b1<<8)|b0          [LE]
 *   width=3 (TXD3): RXD = (b2<<24)|(b1<<16)|(b0<<8)|0
 *   width=2 (TXD2): RXD = (b1<<24)|(b0<<16)|0
 *   width=1 (TXD1): RXD = (b0<<24)|0
 *
 * General form: byte K (Kth received) sits at bit ((4-N+K)*8). Empirical
 * baseline: a JEDEC RDID via TXD1 accumulates 0x00000000 → 0xef000000 →
 * 0x40ef0000 → 0x1940ef00 across 4 separate 1-byte transfers — same shift
 * pattern as multi-byte TXDn used here.
 */
static inline void zynq_qspi_rxd_unpack(uint32_t rxd, uint8_t *dst, uint8_t width)
{
	uint32_t shift = (4U - width) * 8U;

	for (uint8_t k = 0; k < width; k++) {
		dst[k] = (uint8_t)(rxd >> (shift + (uint32_t)k * 8U));
	}
}

/* Drain one RX FIFO slot: poll RX_NEMPTY, read RXD, clear ISR, optionally
 * unpack into `dst` (NULL = discard).
 */
static inline int zynq_qspi_drain_slot(const struct device *dev,
				       uint8_t *dst, uint8_t width)
{
	uint32_t timeout = QSPI_RX_TIMEOUT;

	while (!(qspi_read(dev, QSPI_ISR) & ISR_RX_NEMPTY)) {
		if (--timeout == 0U) {
			LOG_ERR("RX timeout draining slot (width=%u)", width);
			return -ETIMEDOUT;
		}
	}
	uint32_t rxd = qspi_read(dev, QSPI_RXD);

	qspi_write(dev, QSPI_ISR, ISR_RX_NEMPTY);

	if (dst != NULL) {
		zynq_qspi_rxd_unpack(rxd, dst, width);
	}
	return 0;
}

/* Run a single burst of `chunk` bytes (1..QSPI_BURST_MAX): plan slot
 * widths (TXD0 for full 4-byte slots, plus a 1/2/3-byte tail if needed),
 * fill TX FIFO, fire one MANSTARTCOM, drain RX FIFO. Caller stages TX
 * bytes in `tx_stage` and (optionally) receives into `rx_stage`.
 */
static int zynq_qspi_burst_xfer(const struct device *dev,
				const uint8_t *tx_stage, uint8_t *rx_stage,
				size_t chunk)
{
	uint8_t widths[QSPI_FIFO_DEPTH];
	size_t slots = 0;
	size_t pos = 0;

	while (pos < chunk) {
		size_t w = (chunk - pos >= 4U) ? 4U : (chunk - pos);

		widths[slots++] = (uint8_t)w;
		pos += w;
	}

	/* Fill TX FIFO. With CR_MANSTARTEN=1 these queue without firing. */
	pos = 0;
	for (size_t s = 0; s < slots; s++) {
		zynq_qspi_txd_write(dev, &tx_stage[pos], widths[s]);
		pos += widths[s];
	}

	/* Single trigger drains the entire TX FIFO; CS stays asserted. */
	uint32_t cr = qspi_read(dev, QSPI_CR);

	qspi_write(dev, QSPI_CR, cr | CR_MANSTARTCOM);

	/* Drain RX FIFO — one entry per TXDn slot, same widths. */
	pos = 0;
	for (size_t s = 0; s < slots; s++) {
		int rc = zynq_qspi_drain_slot(dev,
					      rx_stage ? &rx_stage[pos] : NULL,
					      widths[s]);

		if (rc != 0) {
			return rc;
		}
		pos += widths[s];
	}
	return 0;
}

/* ── Transceive ──────────────────────────────────────────────────── */

static int zynq_qspi_transceive(const struct device *dev,
				 const struct spi_config *config,
				 const struct spi_buf_set *tx_bufs,
				 const struct spi_buf_set *rx_bufs)
{
	struct zynq_qspi_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	uint8_t tx_stage[QSPI_BURST_MAX];
	uint8_t rx_stage[QSPI_BURST_MAX];
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

	while (spi_context_tx_on(ctx) || spi_context_rx_on(ctx)) {
		size_t tx_avail = ctx->tx_len;
		size_t rx_avail = ctx->rx_len;
		size_t chunk;

		/* Don't cross segment boundaries inside a burst — keep the
		 * per-side spi_context_update_*() calls bounded by the
		 * current segment. Asymmetric tx/rx (one side exhausted but
		 * the other still has data) is handled by treating the
		 * exhausted side as NOP and bursting up to the other side's
		 * remaining bytes.
		 */
		if (tx_avail != 0U && rx_avail != 0U) {
			chunk = MIN(MIN(tx_avail, rx_avail), QSPI_BURST_MAX);
		} else if (tx_avail != 0U) {
			chunk = MIN(tx_avail, QSPI_BURST_MAX);
		} else if (rx_avail != 0U) {
			chunk = MIN(rx_avail, QSPI_BURST_MAX);
		} else {
			/* Defensive: outer condition said _on(), but both
			 * lens are 0. Step one byte to advance past a
			 * pending NOP segment boundary.
			 */
			chunk = 1U;
		}

		bool has_tx = spi_context_tx_buf_on(ctx);
		bool has_rx = spi_context_rx_buf_on(ctx);

		if (has_tx) {
			memcpy(tx_stage, ctx->tx_buf, chunk);
		} else {
			memset(tx_stage, 0, chunk);
		}

		rc = zynq_qspi_burst_xfer(dev, tx_stage,
					  has_rx ? rx_stage : NULL, chunk);
		if (rc != 0) {
			goto done;
		}

		if (has_rx) {
			memcpy(ctx->rx_buf, rx_stage, chunk);
		}

		spi_context_update_tx(ctx, 1, MIN(chunk, ctx->tx_len));
		spi_context_update_rx(ctx, 1, MIN(chunk, ctx->rx_len));
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

/* ── LQSPI memory-mapped fast read ──────────────────────────────── */

/* Aperture is mapped lazily on first call (single 16 MiB region for the
 * whole driver — there is only one Zynq-7000 PS QSPI controller). The
 * mapping is K_MEM_CACHE_NONE because flash content can change beneath us.
 */
static mm_reg_t lqspi_aperture_va;
static struct k_mutex lqspi_map_lock;
static bool lqspi_map_lock_inited;

static int zynq_qspi_lqspi_map(void)
{
	int rc;

	if (lqspi_aperture_va != 0) {
		return 0;
	}
	if (!lqspi_map_lock_inited) {
		k_mutex_init(&lqspi_map_lock);
		lqspi_map_lock_inited = true;
	}
	k_mutex_lock(&lqspi_map_lock, K_FOREVER);
	if (lqspi_aperture_va == 0) {
		mm_reg_t va;

		device_map(&va, QSPI_LQSPI_APERTURE_PA,
			   QSPI_LQSPI_APERTURE_SIZE, K_MEM_CACHE_NONE);
		lqspi_aperture_va = va;
		rc = (va != 0) ? 0 : -EIO;
	} else {
		rc = 0;
	}
	k_mutex_unlock(&lqspi_map_lock);
	return rc;
}

int xlnx_zynq_qspi_lqspi_read(const struct device *dev,
			      off_t addr, void *dst, size_t len)
{
	struct zynq_qspi_data *data;
	uint32_t saved_cr;
	uint32_t saved_lqspi_cr;
	int rc;

	if (dev == NULL || dst == NULL || len == 0U) {
		return -EINVAL;
	}
	if (addr < 0 ||
	    (uint64_t)(off_t)addr + (uint64_t)len > QSPI_LQSPI_APERTURE_SIZE) {
		return -EINVAL;
	}

	rc = zynq_qspi_lqspi_map();
	if (rc != 0) {
		return rc;
	}

	data = dev->data;

	/* Mutually exclusive with spi_transceive(). config arg is unused
	 * here — the lock just acquires the underlying mutex.
	 */
	spi_context_lock(&data->ctx, false, NULL, NULL, NULL);

	saved_cr = qspi_read(dev, QSPI_CR);
	saved_lqspi_cr = qspi_read(dev, QSPI_LQSPI_CR);

	/* Enter LQSPI mode:
	 *   1. Disable controller
	 *   2. CR: keep baud/master/hold/fifo_width, but CLEAR manual flags
	 *      (MANCS / MANSTARTEN — those wait for SW intervention which
	 *      never comes during AXI-driven linear reads, hanging the bus)
	 *      and SET IFMODE for linear mode
	 *   3. LQSPI_CR for 0x6B Quad Output Fast Read + 1 dummy byte +
	 *      LINEAR_MASK
	 *   4. Re-enable
	 */
	uint32_t lqspi_cr_val =
		(saved_cr & ~(CR_MANCS | CR_MANSTARTEN | CR_PCS)) | CR_IFMODE;

	qspi_write(dev, QSPI_ER, 0);
	qspi_write(dev, QSPI_CR, lqspi_cr_val);
	qspi_write(dev, QSPI_LQSPI_CR, LQSPI_CR_QUAD_FAST_READ);
	qspi_write(dev, QSPI_ER, ER_ENABLE);

	memcpy(dst, (const void *)(uintptr_t)(lqspi_aperture_va + addr), len);

	/* Restore manual I/O mode. */
	qspi_write(dev, QSPI_ER, 0);
	qspi_write(dev, QSPI_LQSPI_CR, saved_lqspi_cr);
	qspi_write(dev, QSPI_CR, saved_cr);
	qspi_write(dev, QSPI_ER, ER_ENABLE);

	spi_context_release(&data->ctx, 0);
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
		    CR_MSTREN | CR_MANCS | CR_MANSTARTEN | CR_HOLDB_DR | CR_FIFO_WIDTH | CR_PCS);

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
