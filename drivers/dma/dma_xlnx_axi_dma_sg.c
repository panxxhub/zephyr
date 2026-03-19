/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Xilinx AXI DMA scatter-gather driver for Zephyr RTOS.
 *
 * Supports cyclic BD mode (CYC_BD_EN) for continuous RX streaming,
 * non-cyclic SG for one-shot TX transfers, generic APP field passthrough,
 * dynamic RX ring reconfiguration, and D-cache coherency management.
 *
 * Reference: Xilinx PG021 — AXI DMA v7.1 Product Guide.
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/arch/cache.h>
#include <string.h>

#include "dma_xlnx_axi_dma_sg.h"

#define DT_DRV_COMPAT xlnx_axi_dma_sg

LOG_MODULE_REGISTER(dma_xlnx_sg, CONFIG_DMA_LOG_LEVEL);

/* --------------------------------------------------------------------------
 * Channel indices
 * -------------------------------------------------------------------------- */
#define CH_TX 0
#define CH_RX 1
#define NUM_CHANNELS 2

/* --------------------------------------------------------------------------
 * Register offsets — per-channel, relative to channel base
 * -------------------------------------------------------------------------- */
#define REG_DMACR   0x00
#define REG_DMASR   0x04
#define REG_CURDESC 0x08
#define REG_TAILDESC 0x10

/* Channel base offsets from DMA register region */
#define MM2S_BASE 0x00
#define S2MM_BASE 0x30

/* --------------------------------------------------------------------------
 * DMACR bits
 * -------------------------------------------------------------------------- */
#define DMACR_RS         BIT(0)
#define DMACR_RESET      BIT(2)
#define DMACR_KEYHOLE    BIT(3)
#define DMACR_CYC_BD_EN  BIT(4)
#define DMACR_IOC_IRQEN  BIT(12)
#define DMACR_DLY_IRQEN  BIT(13)
#define DMACR_ERR_IRQEN  BIT(14)
#define DMACR_ALL_IRQEN  (DMACR_IOC_IRQEN | DMACR_DLY_IRQEN | DMACR_ERR_IRQEN)
#define DMACR_IRQTHRESH_SHIFT 16
#define DMACR_IRQTHRESH_MASK  0x00FF0000U
#define DMACR_IRQDELAY_SHIFT  24
#define DMACR_IRQDELAY_MASK   0xFF000000U

/* --------------------------------------------------------------------------
 * DMASR bits
 * -------------------------------------------------------------------------- */
#define DMASR_HALTED    BIT(0)
#define DMASR_IDLE      BIT(1)
#define DMASR_SGINCL    BIT(3)
#define DMASR_INTERR    BIT(4)
#define DMASR_SLVERR    BIT(5)
#define DMASR_DMADECERR BIT(6)
#define DMASR_SGINTERR  BIT(8)
#define DMASR_SGSLVERR  BIT(9)
#define DMASR_SGDECERR  BIT(10)
#define DMASR_IOC_IRQ   BIT(12)
#define DMASR_DLY_IRQ   BIT(13)
#define DMASR_ERR_IRQ   BIT(14)

#define DMASR_ALL_ERR (DMASR_INTERR | DMASR_SLVERR | DMASR_DMADECERR | \
		       DMASR_SGINTERR | DMASR_SGSLVERR | DMASR_SGDECERR)

#define DMASR_IRQ_BITS (DMASR_IOC_IRQ | DMASR_DLY_IRQ | DMASR_ERR_IRQ)

/* --------------------------------------------------------------------------
 * BD control / status field masks
 * -------------------------------------------------------------------------- */
#define BD_CTRL_EOF       BIT(26)
#define BD_CTRL_SOF       BIT(27)
#define BD_STS_CMPLT      BIT(31)

/* BD_CTRL_LEN_MASK and BD_STS_LEN_MASK are computed at init time from
 * the DT property xlnx,sg-length-width (Vivado c_sg_length_width).
 * The macro below converts the width to a bitmask.
 */
#define SG_LEN_MASK(width) (BIT(width) - 1U)

/* --------------------------------------------------------------------------
 * Scatter-gather BD descriptor — PG021 layout, 64-byte aligned
 * -------------------------------------------------------------------------- */
struct xlnx_sg_bd {
	uint32_t next_desc;       /* 0x00 */
	uint32_t next_desc_msb;   /* 0x04 */
	uint32_t buf_addr;        /* 0x08 */
	uint32_t buf_addr_msb;    /* 0x0C */
	uint32_t reserved0;       /* 0x10 */
	uint32_t reserved1;       /* 0x14 */
	uint32_t control;         /* 0x18 */
	uint32_t status;          /* 0x1C */
	uint32_t app[5];          /* 0x20-0x30 */
} __aligned(64);

/* --------------------------------------------------------------------------
 * Reset timeout
 * -------------------------------------------------------------------------- */
#define RESET_TIMEOUT_US 10000
#define RESET_POLL_US    10

/* --------------------------------------------------------------------------
 * Per-channel runtime state
 * -------------------------------------------------------------------------- */
struct dma_xlnx_sg_chan {
	struct xlnx_sg_bd *bds;      /* pointer to static BD array */
	uint32_t num_bds;            /* number of BDs in the ring */
	uint32_t producer_idx;       /* next BD to be submitted */
	uint32_t consumer_idx;       /* next BD to check for completion */
	dma_callback_t callback;
	void *user_data;
	bool cyclic;                 /* true for cyclic RX */
	bool error;                  /* sticky error flag */
	uint8_t irq_threshold;
	uint8_t irq_timeout;
	struct dma_xlnx_sg_app_fields tx_app;  /* APP fields for next TX SOF */
	struct dma_xlnx_sg_app_fields rx_app;  /* APP fields from last completed RX */
	uint32_t last_rx_bytes;      /* bytes from last completed RX BD */
	uint32_t bd_buf_bytes;       /* buffer size per BD */
};

/* --------------------------------------------------------------------------
 * ISR config function type
 * -------------------------------------------------------------------------- */
typedef void (*dma_xlnx_sg_irq_cfg_t)(const struct device *dev);

/* --------------------------------------------------------------------------
 * Device config (ROM) — from devicetree
 * -------------------------------------------------------------------------- */
struct dma_xlnx_sg_cfg {
	DEVICE_MMIO_NAMED_ROM(regs);
	DEVICE_MMIO_NAMED_ROM(tx_buf);
	DEVICE_MMIO_NAMED_ROM(rx_buf);
	dma_xlnx_sg_irq_cfg_t irq_config;
	uintptr_t tx_buf_phys;
	uintptr_t rx_buf_phys;
	size_t tx_buf_size;
	size_t rx_buf_size;
	uint32_t sg_len_mask;       /* (1 << sg_length_width) - 1 */
};

/* --------------------------------------------------------------------------
 * Device data (RAM)
 * -------------------------------------------------------------------------- */
struct dma_xlnx_sg_data {
	DEVICE_MMIO_NAMED_RAM(regs);
	DEVICE_MMIO_NAMED_RAM(tx_buf);
	DEVICE_MMIO_NAMED_RAM(rx_buf);
	struct dma_xlnx_sg_chan ch[NUM_CHANNELS];
};

/* --------------------------------------------------------------------------
 * Required macros for DEVICE_MMIO_NAMED_* helpers
 * -------------------------------------------------------------------------- */
#define DEV_DATA(dev) ((struct dma_xlnx_sg_data *)(dev)->data)
#define DEV_CFG(dev)  ((const struct dma_xlnx_sg_cfg *)(dev)->config)

/* --------------------------------------------------------------------------
 * MMIO helpers
 * -------------------------------------------------------------------------- */
static inline uintptr_t chan_base(const struct device *dev, uint32_t channel)
{
	return DEVICE_MMIO_NAMED_GET(dev, regs) +
	       (channel == CH_TX ? MM2S_BASE : S2MM_BASE);
}

static inline void chan_write(const struct device *dev, uint32_t channel,
			      uint32_t reg, uint32_t val)
{
	sys_write32(val, chan_base(dev, channel) + reg);
}

static inline uint32_t chan_read(const struct device *dev, uint32_t channel,
				 uint32_t reg)
{
	return sys_read32(chan_base(dev, channel) + reg);
}

/* --------------------------------------------------------------------------
 * Cache coherency helpers
 * -------------------------------------------------------------------------- */
static inline void cache_flush(void *addr, size_t len)
{
#if !IS_ENABLED(CONFIG_DMA_XLNX_AXI_DMA_SG_CACHE_COHERENT)
	arch_dcache_flush_range(addr, len);
#endif
}

static inline void cache_invd(void *addr, size_t len)
{
#if !IS_ENABLED(CONFIG_DMA_XLNX_AXI_DMA_SG_CACHE_COHERENT)
	arch_dcache_invd_range(addr, len);
#endif
}

/* --------------------------------------------------------------------------
 * Buffer address / size helpers — use fields stored in config at compile time
 * -------------------------------------------------------------------------- */
static inline uintptr_t buf_phys(const struct device *dev, uint32_t channel)
{
	const struct dma_xlnx_sg_cfg *cfg = dev->config;

	return (channel == CH_TX) ? cfg->tx_buf_phys : cfg->rx_buf_phys;
}

static inline uintptr_t buf_virt(const struct device *dev, uint32_t channel)
{
	if (channel == CH_TX) {
		return DEVICE_MMIO_NAMED_GET(dev, tx_buf);
	}
	return DEVICE_MMIO_NAMED_GET(dev, rx_buf);
}

static inline size_t buf_size(const struct device *dev, uint32_t channel)
{
	const struct dma_xlnx_sg_cfg *cfg = dev->config;

	return (channel == CH_TX) ? cfg->tx_buf_size : cfg->rx_buf_size;
}

/* --------------------------------------------------------------------------
 * Soft reset — returns 0 on success, -EIO on timeout
 * -------------------------------------------------------------------------- */
static int do_soft_reset(const struct device *dev, uint32_t channel)
{
	chan_write(dev, channel, REG_DMACR, DMACR_RESET);

	uint32_t elapsed = 0;

	while (elapsed < RESET_TIMEOUT_US) {
		if (!(chan_read(dev, channel, REG_DMACR) & DMACR_RESET)) {
			return 0;
		}
		k_busy_wait(RESET_POLL_US);
		elapsed += RESET_POLL_US;
	}

	LOG_ERR("ch %u: soft reset timed out", channel);
	return -EIO;
}

/* --------------------------------------------------------------------------
 * BD ring build — sets up circular chain
 *
 * For TX (non-cyclic): each BD gets SOF+EOF, status cleared.
 * For RX (cyclic): each BD points to its buffer slice, no SOF/EOF needed.
 * -------------------------------------------------------------------------- */
static int build_bd_ring(const struct device *dev, uint32_t channel)
{
	struct dma_xlnx_sg_data *data = dev->data;
	struct dma_xlnx_sg_chan *ch = &data->ch[channel];
	const uint32_t len_mask = DEV_CFG(dev)->sg_len_mask;
	uintptr_t phys = buf_phys(dev, channel);

	if (ch->bd_buf_bytes > len_mask) {
		LOG_ERR("BD buffer size %u exceeds hardware max %u "
			"(sg-length-width too narrow)",
			ch->bd_buf_bytes, len_mask);
		return -EINVAL;
	}

	for (uint32_t i = 0; i < ch->num_bds; i++) {
		uint32_t next = (i + 1) % ch->num_bds;

		memset(&ch->bds[i], 0, sizeof(ch->bds[i]));
		ch->bds[i].next_desc = (uint32_t)(uintptr_t)&ch->bds[next];
		ch->bds[i].next_desc_msb = 0U;
		ch->bds[i].buf_addr = (uint32_t)(phys + (uintptr_t)i * ch->bd_buf_bytes);
		ch->bds[i].buf_addr_msb = 0U;
		ch->bds[i].control = ch->bd_buf_bytes & DEV_CFG(dev)->sg_len_mask;

		if (channel == CH_TX) {
			/* Each TX BD is a complete single-BD transfer */
			ch->bds[i].control |= BD_CTRL_SOF | BD_CTRL_EOF;
			/* Write APP fields into SOF descriptor */
			for (int a = 0; a < 5; a++) {
				ch->bds[i].app[a] = ch->tx_app.app[a];
			}
		}

		ch->bds[i].status = 0U;
	}

	/* Flush entire BD ring so DMA engine sees current values */
	cache_flush(ch->bds, sizeof(struct xlnx_sg_bd) * ch->num_bds);
	return 0;
}

/* --------------------------------------------------------------------------
 * Build DMACR value for a channel
 * -------------------------------------------------------------------------- */
static uint32_t build_dmacr(const struct dma_xlnx_sg_chan *ch)
{
	uint32_t dmacr = DMACR_RS | DMACR_ALL_IRQEN;

	if (ch->cyclic) {
		dmacr |= DMACR_CYC_BD_EN;
	}

	dmacr |= ((uint32_t)ch->irq_threshold << DMACR_IRQTHRESH_SHIFT) & DMACR_IRQTHRESH_MASK;
	dmacr |= ((uint32_t)ch->irq_timeout << DMACR_IRQDELAY_SHIFT) & DMACR_IRQDELAY_MASK;

	return dmacr;
}

/* --------------------------------------------------------------------------
 * Start a channel: write CURDESC, DMACR (with RS=1), then TAILDESC
 * -------------------------------------------------------------------------- */
static void kick_channel(const struct device *dev, uint32_t channel)
{
	struct dma_xlnx_sg_data *data = dev->data;
	struct dma_xlnx_sg_chan *ch = &data->ch[channel];

	ch->error = false;

	/* CURDESC must be written before RS=1 */
	barrier_dmem_fence_full();
	chan_write(dev, channel, REG_CURDESC, (uint32_t)(uintptr_t)&ch->bds[0]);

	/* Write DMACR with RS=1 */
	uint32_t dmacr = build_dmacr(ch);

	chan_write(dev, channel, REG_DMACR, dmacr);

	/* TAILDESC must be written after RS=1 */
	barrier_dmem_fence_full();

	if (ch->cyclic) {
		/*
		 * PG021: In cyclic mode, TAILDESC must be set to a value
		 * OUTSIDE the BD ring. We use an arbitrary invalid address.
		 * The engine ignores CMPLT bits and runs forever.
		 */
		chan_write(dev, channel, REG_TAILDESC, 0x50U);
	} else {
		/*
		 * Non-cyclic: TAILDESC = last BD. Engine pauses when it
		 * reaches this descriptor after processing it.
		 */
		chan_write(dev, channel, REG_TAILDESC,
			   (uint32_t)(uintptr_t)&ch->bds[ch->num_bds - 1]);
	}

	barrier_dmem_fence_full();
}

/* --------------------------------------------------------------------------
 * Error logging helper
 * -------------------------------------------------------------------------- */
static void log_dma_errors(uint32_t channel, uint32_t dmasr)
{
	if (dmasr & DMASR_INTERR) {
		LOG_ERR("ch %u: DMA internal error (DMASR=0x%08x)", channel, dmasr);
	}
	if (dmasr & DMASR_SLVERR) {
		LOG_ERR("ch %u: DMA slave error (DMASR=0x%08x)", channel, dmasr);
	}
	if (dmasr & DMASR_DMADECERR) {
		LOG_ERR("ch %u: DMA decode error (DMASR=0x%08x)", channel, dmasr);
	}
	if (dmasr & DMASR_SGINTERR) {
		LOG_ERR("ch %u: SG internal error (DMASR=0x%08x)", channel, dmasr);
	}
	if (dmasr & DMASR_SGSLVERR) {
		LOG_ERR("ch %u: SG slave error (DMASR=0x%08x)", channel, dmasr);
	}
	if (dmasr & DMASR_SGDECERR) {
		LOG_ERR("ch %u: SG decode error (DMASR=0x%08x)", channel, dmasr);
	}
}

/* --------------------------------------------------------------------------
 * TX ISR — non-cyclic completion
 * -------------------------------------------------------------------------- */
static void dma_xlnx_sg_tx_isr(const struct device *dev)
{
	struct dma_xlnx_sg_data *data = dev->data;
	struct dma_xlnx_sg_chan *ch = &data->ch[CH_TX];
	uint32_t dmasr = chan_read(dev, CH_TX, REG_DMASR);

	/* Check for errors */
	if (dmasr & DMASR_ALL_ERR) {
		log_dma_errors(CH_TX, dmasr);
		ch->error = true;
		/* Clear error IRQ flag */
		chan_write(dev, CH_TX, REG_DMASR, DMASR_ERR_IRQ);
	}

	/* Completion or delay IRQ */
	if (dmasr & (DMASR_IOC_IRQ | DMASR_DLY_IRQ)) {
		/* Clear the IRQ flags (write-1-to-clear) */
		chan_write(dev, CH_TX, REG_DMASR,
			   dmasr & (DMASR_IOC_IRQ | DMASR_DLY_IRQ));

		if (ch->callback) {
			int status = ch->error ? -EIO : DMA_STATUS_COMPLETE;

			ch->callback(dev, ch->user_data, CH_TX, status);
		}
	}

	/* Notify consumer of error even without IOC/DLY */
	if (ch->error && !(dmasr & (DMASR_IOC_IRQ | DMASR_DLY_IRQ))) {
		if (ch->callback) {
			ch->callback(dev, ch->user_data, CH_TX, -EIO);
		}
	}
}

/* --------------------------------------------------------------------------
 * RX ISR — cyclic streaming completion
 * -------------------------------------------------------------------------- */
static void dma_xlnx_sg_rx_isr(const struct device *dev)
{
	struct dma_xlnx_sg_data *data = dev->data;
	struct dma_xlnx_sg_chan *ch = &data->ch[CH_RX];
	uint32_t dmasr = chan_read(dev, CH_RX, REG_DMASR);

	/* Check for errors */
	if (dmasr & DMASR_ALL_ERR) {
		log_dma_errors(CH_RX, dmasr);
		ch->error = true;
		chan_write(dev, CH_RX, REG_DMASR, DMASR_ERR_IRQ);
	}

	/* Completion or delay IRQ */
	if (dmasr & (DMASR_IOC_IRQ | DMASR_DLY_IRQ)) {
		chan_write(dev, CH_RX, REG_DMASR,
			   dmasr & (DMASR_IOC_IRQ | DMASR_DLY_IRQ));

		if (ch->callback) {
			int status = ch->error ? -EIO : DMA_STATUS_COMPLETE;

			ch->callback(dev, ch->user_data, CH_RX, status);
		}
	}

	/* Notify consumer of error even without IOC/DLY */
	if (ch->error && !(dmasr & (DMASR_IOC_IRQ | DMASR_DLY_IRQ))) {
		if (ch->callback) {
			ch->callback(dev, ch->user_data, CH_RX, -EIO);
		}
	}
}

/* ==========================================================================
 * Zephyr DMA driver API implementation
 * ========================================================================== */

/* --------------------------------------------------------------------------
 * dma_config() — validate params, build BD ring, store callback
 * -------------------------------------------------------------------------- */
static int dma_xlnx_sg_config(const struct device *dev, uint32_t channel,
			       struct dma_config *dma_cfg)
{
	struct dma_xlnx_sg_data *data = dev->data;

	if (channel >= NUM_CHANNELS) {
		LOG_ERR("invalid channel %u", channel);
		return -EINVAL;
	}

	/* Validate direction */
	if (channel == CH_TX && dma_cfg->channel_direction != MEMORY_TO_PERIPHERAL) {
		LOG_ERR("TX channel requires MEMORY_TO_PERIPHERAL");
		return -ENOTSUP;
	}
	if (channel == CH_RX && dma_cfg->channel_direction != PERIPHERAL_TO_MEMORY) {
		LOG_ERR("RX channel requires PERIPHERAL_TO_MEMORY");
		return -ENOTSUP;
	}

	struct dma_xlnx_sg_chan *ch = &data->ch[channel];

	/* Store callback */
	ch->callback = dma_cfg->dma_callback;
	ch->user_data = dma_cfg->user_data;

	/* Cyclic mode from dma_config */
	ch->cyclic = (dma_cfg->cyclic != 0);

	/* Calculate BD buffer size from total buffer region */
	size_t total = buf_size(dev, channel);

	if (ch->num_bds == 0) {
		LOG_ERR("ch %u: no BDs allocated", channel);
		return -EINVAL;
	}

	ch->bd_buf_bytes = (uint32_t)(total / ch->num_bds);
	if (ch->bd_buf_bytes == 0) {
		LOG_ERR("ch %u: buffer region too small for %u BDs", channel, ch->num_bds);
		return -EINVAL;
	}

	/* Use Kconfig defaults for IRQ coalescing */
	ch->irq_threshold = (uint8_t)CONFIG_DMA_XLNX_AXI_DMA_SG_IRQ_THRESHOLD;
	ch->irq_timeout = (uint8_t)CONFIG_DMA_XLNX_AXI_DMA_SG_IRQ_TIMEOUT;

	/* Reset indices */
	ch->producer_idx = 0;
	ch->consumer_idx = 0;
	ch->error = false;
	ch->last_rx_bytes = 0;
	memset(&ch->rx_app, 0, sizeof(ch->rx_app));

	/* Build the BD ring */
	int ret = build_bd_ring(dev, channel);
	if (ret) {
		return ret;
	}

	LOG_DBG("ch %u configured: %u BDs x %u bytes, cyclic=%d, thresh=%u",
		channel, ch->num_bds, ch->bd_buf_bytes, ch->cyclic, ch->irq_threshold);

	return 0;
}

/* --------------------------------------------------------------------------
 * dma_start() — write CURDESC, DMACR, TAILDESC to kick the engine
 * -------------------------------------------------------------------------- */
static int dma_xlnx_sg_start(const struct device *dev, uint32_t channel)
{
	if (channel >= NUM_CHANNELS) {
		return -EINVAL;
	}

	uint32_t dmasr = chan_read(dev, channel, REG_DMASR);

	/* If halted due to error, reset and rebuild */
	if (dmasr & DMASR_HALTED) {
		LOG_WRN("ch %u halted (DMASR=0x%08x), performing reset", channel, dmasr);

		int ret = do_soft_reset(dev, channel);

		if (ret) {
			return ret;
		}

		/* Rebuild BD ring after reset */
		ret = build_bd_ring(dev, channel);
		if (ret) {
			return ret;
		}
	}

	kick_channel(dev, channel);

	LOG_DBG("ch %u started", channel);
	return 0;
}

/* --------------------------------------------------------------------------
 * dma_stop() — clear RS bit, engine will halt after current transfer
 * -------------------------------------------------------------------------- */
static int dma_xlnx_sg_stop(const struct device *dev, uint32_t channel)
{
	if (channel >= NUM_CHANNELS) {
		return -EINVAL;
	}

	uint32_t dmacr = chan_read(dev, channel, REG_DMACR);

	dmacr &= ~DMACR_RS;
	chan_write(dev, channel, REG_DMACR, dmacr);
	barrier_dmem_fence_full();

	uint32_t elapsed = 0;

	while (elapsed < RESET_TIMEOUT_US) {
		if (chan_read(dev, channel, REG_DMASR) & DMASR_HALTED) {
			break;
		}
		k_busy_wait(RESET_POLL_US);
		elapsed += RESET_POLL_US;
	}
	if (elapsed >= RESET_TIMEOUT_US) {
		LOG_WRN("Channel %u did not halt within timeout", channel);
	}

	LOG_DBG("ch %u stopped", channel);
	return 0;
}

/* --------------------------------------------------------------------------
 * dma_get_status() — read DMASR, return busy/idle
 * -------------------------------------------------------------------------- */
static int dma_xlnx_sg_get_status(const struct device *dev, uint32_t channel,
				   struct dma_status *stat)
{
	if (channel >= NUM_CHANNELS) {
		return -EINVAL;
	}

	memset(stat, 0, sizeof(*stat));

	uint32_t dmasr = chan_read(dev, channel, REG_DMASR);

	stat->busy = !(dmasr & DMASR_IDLE) && !(dmasr & DMASR_HALTED);
	stat->dir = (channel == CH_TX) ? MEMORY_TO_PERIPHERAL : PERIPHERAL_TO_MEMORY;

	return 0;
}

static DEVICE_API(dma, dma_xlnx_sg_api) = {
	.config = dma_xlnx_sg_config,
	.start = dma_xlnx_sg_start,
	.stop = dma_xlnx_sg_stop,
	.get_status = dma_xlnx_sg_get_status,
	.reload = NULL,
	.suspend = NULL,
	.resume = NULL,
	.get_attribute = NULL,
	.chan_filter = NULL,
};

/* ==========================================================================
 * Device-specific API (from dma_xlnx_axi_dma_sg.h)
 * ========================================================================== */

/* --------------------------------------------------------------------------
 * Reconfigure RX ring at runtime
 * -------------------------------------------------------------------------- */
int dma_xlnx_sg_reconfigure_rx(const struct device *dev,
				uint32_t bd_bytes, uint8_t threshold)
{
	struct dma_xlnx_sg_data *data = dev->data;
	struct dma_xlnx_sg_chan *ch = &data->ch[CH_RX];

	if (bd_bytes == 0 ||
	    (size_t)bd_bytes * (size_t)ch->num_bds > buf_size(dev, CH_RX)) {
		LOG_ERR("bd_bytes %u * %u BDs exceeds RX buf size %zu",
			bd_bytes, ch->num_bds, buf_size(dev, CH_RX));
		return -EINVAL;
	}

	if (threshold == 0 || threshold > ch->num_bds ||
	    (ch->num_bds % threshold) != 0) {
		LOG_ERR("threshold %u must divide RX BD count %u evenly",
			threshold, ch->num_bds);
		return -EINVAL;
	}

	/* Stop S2MM channel */
	uint32_t dmacr = chan_read(dev, CH_RX, REG_DMACR);

	dmacr &= ~DMACR_RS;
	chan_write(dev, CH_RX, REG_DMACR, dmacr);
	barrier_dmem_fence_full();

	/* Wait for halt */
	uint32_t elapsed = 0;

	while (elapsed < RESET_TIMEOUT_US) {
		if (chan_read(dev, CH_RX, REG_DMASR) & DMASR_HALTED) {
			break;
		}
		k_busy_wait(RESET_POLL_US);
		elapsed += RESET_POLL_US;
	}
	if (elapsed >= RESET_TIMEOUT_US) {
		LOG_ERR("S2MM channel did not halt for RX reconfigure");
		return -EIO;
	}

	/* Update ring parameters */
	ch->bd_buf_bytes = bd_bytes;
	ch->irq_threshold = threshold;
	ch->consumer_idx = 0;
	ch->producer_idx = 0;
	ch->error = false;

	/* Rebuild and restart */
	int build_ret = build_bd_ring(dev, CH_RX);
	if (build_ret) {
		return build_ret;
	}
	kick_channel(dev, CH_RX);

	LOG_INF("RX ring reconfigured: %u BDs x %u bytes, threshold=%u",
		ch->num_bds, bd_bytes, threshold);
	return 0;
}

/* --------------------------------------------------------------------------
 * Get APP fields from last completed RX descriptor
 * -------------------------------------------------------------------------- */
int dma_xlnx_sg_get_rx_app(const struct device *dev,
			     struct dma_xlnx_sg_app_fields *app)
{
	struct dma_xlnx_sg_data *data = dev->data;
	struct dma_xlnx_sg_chan *ch = &data->ch[CH_RX];

	if (ch->last_rx_bytes == 0) {
		return -EAGAIN;
	}

	*app = ch->rx_app;
	return 0;
}

/* --------------------------------------------------------------------------
 * Set APP fields for next TX SOF descriptor
 * -------------------------------------------------------------------------- */
int dma_xlnx_sg_set_tx_app(const struct device *dev,
			     const struct dma_xlnx_sg_app_fields *app)
{
	struct dma_xlnx_sg_data *data = dev->data;

	data->ch[CH_TX].tx_app = *app;
	return 0;
}

/* --------------------------------------------------------------------------
 * Get byte count from last completed RX transfer
 * -------------------------------------------------------------------------- */
uint32_t dma_xlnx_sg_last_rx_bytes(const struct device *dev)
{
	struct dma_xlnx_sg_data *data = dev->data;

	return data->ch[CH_RX].last_rx_bytes;
}

/* --------------------------------------------------------------------------
 * Get buffer region addresses
 * -------------------------------------------------------------------------- */
int dma_xlnx_sg_get_buffer(const struct device *dev, uint32_t channel,
			     uintptr_t *phys, uintptr_t *virt, size_t *size)
{
	if (channel >= NUM_CHANNELS) {
		return -EINVAL;
	}

	*phys = buf_phys(dev, channel);
	*virt = buf_virt(dev, channel);
	*size = buf_size(dev, channel);
	return 0;
}

/* --------------------------------------------------------------------------
 * Get completed RX BD window
 * -------------------------------------------------------------------------- */
int dma_xlnx_sg_get_rx_window(const struct device *dev,
				struct dma_xlnx_sg_rx_bd_info *info,
				uint32_t *count)
{
	struct dma_xlnx_sg_data *data = dev->data;
	struct dma_xlnx_sg_chan *ch = &data->ch[CH_RX];
	uint32_t capacity = *count;
	uint32_t filled = 0;
	uint32_t idx = ch->consumer_idx;
	uintptr_t virt_base = buf_virt(dev, CH_RX);

	/*
	 * In cyclic mode with CYC_BD_EN, hardware ignores CMPLT bits.
	 * We rely on the IRQ threshold: after each IRQ, exactly
	 * irq_threshold BDs have been completed since the last IRQ.
	 * Walk up to irq_threshold BDs from the consumer index.
	 */
	uint32_t window_size = ch->irq_threshold;

	if (window_size > capacity) {
		LOG_ERR("window size %u exceeds array capacity %u", window_size, capacity);
		return -EINVAL;
	}

	for (uint32_t i = 0; i < window_size; i++) {
		struct xlnx_sg_bd *bd = &ch->bds[idx];

		/* Invalidate BD to get hardware-written status and app fields */
		cache_invd(bd, sizeof(*bd));

		info[filled].buf_virt = virt_base + (uintptr_t)idx * ch->bd_buf_bytes;
		info[filled].byte_count = bd->status & DEV_CFG(dev)->sg_len_mask;
		for (int a = 0; a < 5; a++) {
			info[filled].app.app[a] = bd->app[a];
		}

		/* Invalidate the data buffer so CPU sees DMA-written data */
		cache_invd((void *)info[filled].buf_virt, ch->bd_buf_bytes);

		/* Update last-completed tracking */
		ch->last_rx_bytes = info[filled].byte_count;
		for (int a = 0; a < 5; a++) {
			ch->rx_app.app[a] = bd->app[a];
		}

		filled++;
		idx = (idx + 1) % ch->num_bds;
	}

	*count = filled;
	return (filled > 0) ? 0 : -EAGAIN;
}

/* --------------------------------------------------------------------------
 * Release RX BD window — advance consumer index
 * -------------------------------------------------------------------------- */
int dma_xlnx_sg_release_rx_window(const struct device *dev, uint32_t count)
{
	struct dma_xlnx_sg_data *data = dev->data;
	struct dma_xlnx_sg_chan *ch = &data->ch[CH_RX];

	ch->consumer_idx = (ch->consumer_idx + count) % ch->num_bds;
	return 0;
}

/* ==========================================================================
 * Init & instantiation
 * ========================================================================== */

static int dma_xlnx_sg_init(const struct device *dev)
{
	const struct dma_xlnx_sg_cfg *cfg = dev->config;
	struct dma_xlnx_sg_data *data = dev->data;

	/* Map MMIO regions.
	 * Register region is mapped uncached (device memory).
	 * Buffer regions use default (cacheable) mapping — the driver manages
	 * coherency explicitly via cache_flush() / cache_invd() calls.
	 */
	DEVICE_MMIO_NAMED_MAP(dev, regs, K_MEM_CACHE_NONE);
	DEVICE_MMIO_NAMED_MAP(dev, tx_buf, K_MEM_CACHE_WB);
	DEVICE_MMIO_NAMED_MAP(dev, rx_buf, K_MEM_CACHE_WB);

	/* Soft-reset both channels */
	int ret = do_soft_reset(dev, CH_TX);

	if (ret) {
		return ret;
	}

	ret = do_soft_reset(dev, CH_RX);
	if (ret) {
		return ret;
	}

	/* Configure IRQs */
	cfg->irq_config(dev);

	LOG_INF("initialized (TX=%u BDs, RX=%u BDs, max_xfer=%u bytes)",
		data->ch[CH_TX].num_bds, data->ch[CH_RX].num_bds,
		cfg->sg_len_mask);
	return 0;
}

/* --------------------------------------------------------------------------
 * DT instantiation macro
 * -------------------------------------------------------------------------- */
#define DMA_XLNX_SG_INIT(inst)                                                              \
                                                                                             \
	static struct xlnx_sg_bd                                                             \
		dma_xlnx_sg_tx_bds_##inst[CONFIG_DMA_XLNX_AXI_DMA_SG_NUM_TX_BD]             \
		__aligned(64);                                                               \
	static struct xlnx_sg_bd                                                             \
		dma_xlnx_sg_rx_bds_##inst[CONFIG_DMA_XLNX_AXI_DMA_SG_NUM_RX_BD]             \
		__aligned(64);                                                               \
                                                                                             \
	static void dma_xlnx_sg_irq_config_##inst(const struct device *dev)                  \
	{                                                                                    \
		IRQ_CONNECT(DT_INST_IRQN_BY_IDX(inst, 0),                                   \
			    DT_INST_IRQ_BY_IDX(inst, 0, priority),                           \
			    dma_xlnx_sg_tx_isr,                                              \
			    DEVICE_DT_INST_GET(inst), 0);                                    \
		irq_enable(DT_INST_IRQN_BY_IDX(inst, 0));                                   \
                                                                                             \
		IRQ_CONNECT(DT_INST_IRQN_BY_IDX(inst, 1),                                   \
			    DT_INST_IRQ_BY_IDX(inst, 1, priority),                           \
			    dma_xlnx_sg_rx_isr,                                              \
			    DEVICE_DT_INST_GET(inst), 0);                                    \
		irq_enable(DT_INST_IRQN_BY_IDX(inst, 1));                                   \
	}                                                                                    \
                                                                                             \
	static const struct dma_xlnx_sg_cfg dma_xlnx_sg_cfg_##inst = {                      \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(regs, DT_DRV_INST(inst)),                \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(tx_buf, DT_DRV_INST(inst)),              \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(rx_buf, DT_DRV_INST(inst)),              \
		.irq_config = dma_xlnx_sg_irq_config_##inst,                                \
		.tx_buf_phys = DT_INST_REG_ADDR_BY_NAME(inst, tx_buf),                      \
		.rx_buf_phys = DT_INST_REG_ADDR_BY_NAME(inst, rx_buf),                      \
		.tx_buf_size = DT_INST_REG_SIZE_BY_NAME(inst, tx_buf),                      \
		.rx_buf_size = DT_INST_REG_SIZE_BY_NAME(inst, rx_buf),                      \
		.sg_len_mask = SG_LEN_MASK(                                              \
			DT_INST_PROP_OR(inst, xlnx_sg_length_width, 14)),                \
	};                                                                                   \
                                                                                             \
	static struct dma_xlnx_sg_data dma_xlnx_sg_data_##inst = {                          \
		.ch = {                                                                      \
			[CH_TX] = {                                                          \
				.bds = dma_xlnx_sg_tx_bds_##inst,                            \
				.num_bds = CONFIG_DMA_XLNX_AXI_DMA_SG_NUM_TX_BD,            \
			},                                                                   \
			[CH_RX] = {                                                          \
				.bds = dma_xlnx_sg_rx_bds_##inst,                            \
				.num_bds = CONFIG_DMA_XLNX_AXI_DMA_SG_NUM_RX_BD,            \
			},                                                                   \
		},                                                                           \
	};                                                                                   \
                                                                                             \
	DEVICE_DT_INST_DEFINE(inst, dma_xlnx_sg_init, NULL,                                 \
			      &dma_xlnx_sg_data_##inst,                                      \
			      &dma_xlnx_sg_cfg_##inst,                                       \
			      POST_KERNEL, CONFIG_DMA_INIT_PRIORITY,                         \
			      &dma_xlnx_sg_api);

DT_INST_FOREACH_STATUS_OKAY(DMA_XLNX_SG_INIT)
