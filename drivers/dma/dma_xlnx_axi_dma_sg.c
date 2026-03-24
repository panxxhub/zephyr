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
#include <zephyr/drivers/interrupt_controller/gic.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/cache.h>
#include <zephyr/sys/atomic.h>
#include <string.h>

#include <zephyr/drivers/dma/dma_xlnx_axi_dma_sg.h>

#define DT_DRV_COMPAT xlnx_axi_dma_sg

LOG_MODULE_REGISTER(dma_xlnx_sg, CONFIG_DMA_LOG_LEVEL);

/* --------------------------------------------------------------------------
 * Channel indices
 * -------------------------------------------------------------------------- */
#define CH_TX        0
#define CH_RX        1
#define NUM_CHANNELS 2

/* --------------------------------------------------------------------------
 * Register offsets — per-channel, relative to channel base
 * -------------------------------------------------------------------------- */
#define REG_DMACR    0x00
#define REG_DMASR    0x04
#define REG_CURDESC  0x08
#define REG_TAILDESC 0x10

/* Channel base offsets from DMA register region */
#define MM2S_BASE 0x00
#define S2MM_BASE 0x30

/* --------------------------------------------------------------------------
 * SG control register — absolute offset from DMA base (shared, not per-channel)
 *
 * PG021 Table 2-7: SG_CTL at offset 0x2C controls AxCACHE and AxUSER for
 * all M_AXI_SG descriptor fetch and write-back transactions.
 *
 * After reset SG_CTL = 0x0, giving AxCACHE = 0b0000 (device non-bufferable).
 * On Zynq-7000, HP-port writes to DDR with AxCACHE[1]=0 (non-modifiable)
 * may be silently dropped by the interconnect, preventing BD write-back
 * (CMPLT bit never set, IOC_IRQ never fires).
 *
 * The fix: set SG_CACHE to at least 0x3 (normal non-cacheable bufferable)
 * so that M_AXI_SG write-back transactions reach DDR.
 * -------------------------------------------------------------------------- */
#define REG_SGCTL            0x2C
#define SGCTL_SG_CACHE_SHIFT 0
#define SGCTL_SG_CACHE_MASK  0x0000000FU
#define SGCTL_SG_USER_SHIFT  8
#define SGCTL_SG_USER_MASK   0x00000F00U

/* --------------------------------------------------------------------------
 * DMACR bits
 * -------------------------------------------------------------------------- */
#define DMACR_RS              BIT(0)
#define DMACR_RESET           BIT(2)
#define DMACR_KEYHOLE         BIT(3)
#define DMACR_CYC_BD_EN       BIT(4)
#define DMACR_IOC_IRQEN       BIT(12)
#define DMACR_DLY_IRQEN       BIT(13)
#define DMACR_ERR_IRQEN       BIT(14)
#define DMACR_ALL_IRQEN       (DMACR_IOC_IRQEN | DMACR_DLY_IRQEN | DMACR_ERR_IRQEN)
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

#define DMASR_ALL_ERR                                                                              \
	(DMASR_INTERR | DMASR_SLVERR | DMASR_DMADECERR | DMASR_SGINTERR | DMASR_SGSLVERR |         \
	 DMASR_SGDECERR)

#define DMASR_IRQ_BITS (DMASR_IOC_IRQ | DMASR_DLY_IRQ | DMASR_ERR_IRQ)

/* --------------------------------------------------------------------------
 * BD control / status field masks
 * -------------------------------------------------------------------------- */
#define BD_CTRL_EOF  BIT(26)
#define BD_CTRL_SOF  BIT(27)
#define BD_STS_CMPLT BIT(31)

/* BD_CTRL_LEN_MASK and BD_STS_LEN_MASK are computed at init time from
 * the DT property xlnx,sg-length-width (Vivado c_sg_length_width).
 * The macro below converts the width to a bitmask.
 */
#define SG_LEN_MASK(width) (BIT(width) - 1U)

/* --------------------------------------------------------------------------
 * Scatter-gather BD descriptor — PG021 layout, 64-byte aligned
 * -------------------------------------------------------------------------- */
struct xlnx_sg_bd {
	uint32_t next_desc;     /* 0x00 */
	uint32_t next_desc_msb; /* 0x04 */
	uint32_t buf_addr;      /* 0x08 */
	uint32_t buf_addr_msb;  /* 0x0C */
	uint32_t mcctl;         /* 0x10 */
	uint32_t stride_vsize;  /* 0x14 */
	uint32_t control;       /* 0x18 */
	uint32_t status;        /* 0x1C */
	uint32_t app[5];        /* 0x20-0x30 */
} __aligned(64);

/* --------------------------------------------------------------------------
 * Reset timeout
 * -------------------------------------------------------------------------- */
#define RESET_TIMEOUT_US 10000
#define RESET_POLL_US    10

/* 64-byte aligned address guaranteed outside any BD ring (PG021 cyclic mode). */
#define CYCLIC_DUMMY_TAILDESC 0x00000040U
#define RX_STREAM_WINDOWS_MAX CONFIG_DMA_XLNX_AXI_DMA_SG_NUM_RX_BD

/* --------------------------------------------------------------------------
 * Per-channel runtime state
 * -------------------------------------------------------------------------- */
struct dma_xlnx_sg_chan {
	const struct device *dev;
	struct xlnx_sg_bd *bds; /* pointer to static BD array */
	uint32_t num_bds;       /* number of BDs in the ring */
	uint32_t producer_idx;  /* next BD to be submitted */
	uint32_t consumer_idx;  /* next BD to check for completion */
	dma_callback_t callback;
	void *user_data;
	bool cyclic; /* true for cyclic RX */
	bool error;  /* sticky error flag */
	uint16_t irq_threshold;    /* software window size in BDs */
	uint8_t hw_irq_threshold;  /* hardware threshold (max 255) written to DMACR */
	uint16_t irq_coalesce_target; /* hw IRQs per software window */
	uint16_t irq_coalesce_count;  /* current hw IRQ accumulator */
	uint8_t irq_timeout;
	struct dma_xlnx_sg_app_fields tx_app; /* APP fields for next TX SOF */
	struct dma_xlnx_sg_app_fields rx_app; /* APP fields from last completed RX */
	uint32_t last_rx_bytes;               /* bytes from last completed RX BD */
	uint32_t bd_buf_bytes;                /* buffer size per BD */
	uint32_t tx_src_addr;      /* one-shot TX: caller's source address (0 = use DT region) */
	uint32_t tx_xfer_size;     /* one-shot TX: transfer size in bytes */
	atomic_t rx_windows_ready; /* completed stream windows waiting for the worker */
	bool rx_stream_active;
	dma_xlnx_sg_rx_stream_cb_t rx_stream_callback;
	void *rx_stream_user_data;
	struct k_work rx_stream_work;
	struct k_work_sync rx_stream_work_sync;
	uint32_t tail_idx; /* current TAILDESC BD index for ping-pong */
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
	uint32_t sg_len_mask; /* (1 << sg_length_width) - 1 */
	uint8_t sg_cache;     /* AxCACHE for M_AXI_SG transactions */
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
static inline uintptr_t dma_base(const struct device *dev)
{
	return DEVICE_MMIO_NAMED_GET(dev, regs);
}

static inline uintptr_t chan_base(const struct device *dev, uint32_t channel)
{
	return dma_base(dev) + (channel == CH_TX ? MM2S_BASE : S2MM_BASE);
}

static inline void chan_write(const struct device *dev, uint32_t channel, uint32_t reg,
			      uint32_t val)
{
	sys_write32(val, chan_base(dev, channel) + reg);
}

static inline uint32_t chan_read(const struct device *dev, uint32_t channel, uint32_t reg)
{
	return sys_read32(chan_base(dev, channel) + reg);
}

/* --------------------------------------------------------------------------
 * Cache coherency helpers
 * -------------------------------------------------------------------------- */
static inline void cache_flush(void *addr, size_t len)
{
#if !IS_ENABLED(CONFIG_DMA_XLNX_AXI_DMA_SG_CACHE_COHERENT)
	sys_cache_data_flush_range(addr, len);
#endif
}

static inline void cache_invd(void *addr, size_t len)
{
#if !IS_ENABLED(CONFIG_DMA_XLNX_AXI_DMA_SG_CACHE_COHERENT)
	sys_cache_data_invd_range(addr, len);
#endif
}

static void dma_xlnx_sg_program_sgctl(const struct device *dev)
{
	uintptr_t sgctl_addr = dma_base(dev) + REG_SGCTL;
	uint32_t sgctl = sys_read32(sgctl_addr);

	sgctl &= ~SGCTL_SG_CACHE_MASK;
	sgctl |= (((uint32_t)DEV_CFG(dev)->sg_cache << SGCTL_SG_CACHE_SHIFT) & SGCTL_SG_CACHE_MASK);

	sys_write32(sgctl, sgctl_addr);
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
			/* PG021 reset returns registers to reset state, so restore SG_CTL. */
			dma_xlnx_sg_program_sgctl(dev);
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

	/*
	 * NOTE: Using CPU virtual addresses for BD next_desc and CURDESC/TAILDESC
	 * register writes. This assumes VA==PA (identity mapping), which holds
	 * on Zynq-7000 with Zephyr's default MMU configuration. If the platform
	 * uses non-identity mappings, these must be converted to physical
	 * addresses via k_mem_phys_addr() or equivalent.
	 */

	/* One-shot TX: single BD with caller's buffer address and size */
	if (channel == CH_TX && ch->tx_src_addr != 0) {
		if (ch->tx_xfer_size > len_mask) {
			LOG_ERR("TX transfer size %u exceeds hardware max %u", ch->tx_xfer_size,
				len_mask);
			return -EINVAL;
		}

		memset(&ch->bds[0], 0, sizeof(ch->bds[0]));
		ch->bds[0].next_desc = (uint32_t)(uintptr_t)&ch->bds[0];
		ch->bds[0].next_desc_msb = 0U;
		ch->bds[0].buf_addr = ch->tx_src_addr;
		ch->bds[0].buf_addr_msb = 0U;
		ch->bds[0].control = (ch->tx_xfer_size & len_mask) | BD_CTRL_SOF | BD_CTRL_EOF;
		for (int a = 0; a < 5; a++) {
			ch->bds[0].app[a] = ch->tx_app.app[a];
		}
		ch->bds[0].status = 0U;
		cache_flush(ch->bds, sizeof(struct xlnx_sg_bd));
		return 0;
	}

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
	uint32_t dmacr = DMACR_RS | DMACR_IOC_IRQEN | DMACR_ERR_IRQEN;

	/*
	 * Do NOT set CYC_BD_EN — it disables IOC_IRQ generation.
	 * Continuous RX therefore uses a normal BD ring with software-managed
	 * re-arm after each completed window, while TAILDESC still points at
	 * the last BD in the ring. See: AMD forum "AXI DMA Cyclic BD not working".
	 */

	dmacr |= ((uint32_t)ch->hw_irq_threshold << DMACR_IRQTHRESH_SHIFT) & DMACR_IRQTHRESH_MASK;
	if (ch->irq_timeout != 0U) {
		dmacr |= DMACR_DLY_IRQEN;
		dmacr |= ((uint32_t)ch->irq_timeout << DMACR_IRQDELAY_SHIFT) & DMACR_IRQDELAY_MASK;
	}

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
		 * Ping-pong RX: non-cyclic in hardware (no CYC_BD_EN).
		 * TAILDESC = first HW threshold boundary.
		 * DMA processes those BDs, goes IDLE, IOC fires.
		 * ISR advances TAILDESC by hw_irq_threshold each time.
		 */
		ch->tail_idx = ch->hw_irq_threshold - 1;
		chan_write(dev, channel, REG_TAILDESC,
			   (uint32_t)(uintptr_t)&ch->bds[ch->tail_idx]);
	} else {
		/*
		 * Non-cyclic: TAILDESC triggers processing up to this BD.
		 * For one-shot TX (tx_src_addr != 0), only bds[0] is used.
		 * For multi-BD transfers, submit through the last BD.
		 */
		uint32_t tail_idx = (ch->tx_src_addr != 0) ? 0 : (ch->num_bds - 1);

		chan_write(dev, channel, REG_TAILDESC, (uint32_t)(uintptr_t)&ch->bds[tail_idx]);
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
		chan_write(dev, CH_TX, REG_DMASR, dmasr & (DMASR_IOC_IRQ | DMASR_DLY_IRQ));

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
		chan_write(dev, CH_RX, REG_DMASR, dmasr & (DMASR_IOC_IRQ | DMASR_DLY_IRQ));

		if (ch->rx_stream_active) {
			/*
			 * Ping-pong: clear CMPLT on the just-completed BDs,
			 * then advance TAILDESC to resume DMA immediately.
			 * Use hw_irq_threshold (what HW actually coalesced)
			 * for the BD-level operations each HW IRQ.
			 */
			uint32_t hw_thresh = ch->hw_irq_threshold;
			uint32_t idx = ch->producer_idx;

			for (uint32_t i = 0; i < hw_thresh; i++) {
				cache_invd(&ch->bds[idx], sizeof(ch->bds[idx]));
				ch->bds[idx].control &= ~BD_STS_CMPLT;
				ch->bds[idx].status &= ~BD_STS_CMPLT;
				cache_flush(&ch->bds[idx], sizeof(ch->bds[idx]));
				idx = (idx + 1) % ch->num_bds;
			}
			ch->producer_idx = idx;

			/* Advance TAILDESC by hw_thresh BDs */
			ch->tail_idx = (ch->tail_idx + hw_thresh) % ch->num_bds;
			barrier_dmem_fence_full();
			chan_write(dev, CH_RX, REG_TAILDESC,
				   (uint32_t)(uintptr_t)&ch->bds[ch->tail_idx]);

			/* Software coalescing: only notify work handler
			 * when enough HW IRQs have accumulated to fill
			 * one full software window (irq_threshold BDs).
			 */
			ch->irq_coalesce_count++;
			if (ch->irq_coalesce_count >= ch->irq_coalesce_target) {
				ch->irq_coalesce_count = 0;
				atomic_inc(&ch->rx_windows_ready);
				(void)k_work_submit(&ch->rx_stream_work);
			}
		} else {
			if (ch->callback) {
				int status = ch->error ? -EIO : DMA_STATUS_COMPLETE;

				ch->callback(dev, ch->user_data, CH_RX, status);
			}
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
/*
 * NOTE: RX uses fixed DT-defined buffer regions. For TX, if head_block is
 * provided, source_address and block_size are used for a one-shot single-BD
 * transfer. If head_block is NULL, the DT TX buffer region is split across
 * all BDs. Use dma_xlnx_sg_get_buffer() to query buffer addresses.
 */
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

	if (channel == CH_TX && dma_cfg->cyclic) {
		LOG_ERR("Cyclic mode not supported on TX channel");
		return -ENOTSUP;
	}

	struct dma_xlnx_sg_chan *ch = &data->ch[channel];

	/* Store callback */
	ch->callback = dma_cfg->dma_callback;
	ch->user_data = dma_cfg->user_data;

	/* Cyclic RX must use start_rx_stream() for proper ping-pong init */
	if (channel == CH_RX && dma_cfg->cyclic != 0) {
		return -ENOTSUP;
	}
	ch->cyclic = (dma_cfg->cyclic != 0);

	/* Calculate BD buffer size.
	 * If head_block provides a block_size, use it to derive per-BD size
	 * (total transfer / num_bds). Otherwise fall back to DT buffer region.
	 */
	size_t total;

	if (dma_cfg->head_block != NULL && dma_cfg->head_block->block_size > 0) {
		total = dma_cfg->head_block->block_size;
	} else {
		total = buf_size(dev, channel);
	}

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
	ch->irq_threshold = (uint16_t)CONFIG_DMA_XLNX_AXI_DMA_SG_IRQ_THRESHOLD;
	ch->irq_timeout = (uint8_t)CONFIG_DMA_XLNX_AXI_DMA_SG_IRQ_TIMEOUT;

	/* TX one-shot: use caller's buffer from head_block instead of DT region */
	if (channel == CH_TX && dma_cfg->head_block != NULL) {
		ch->tx_src_addr = (uint32_t)dma_cfg->head_block->source_address;
		ch->tx_xfer_size = dma_cfg->head_block->block_size;
	} else {
		ch->tx_src_addr = 0;
		ch->tx_xfer_size = 0;
	}

	/* Reset indices */
	ch->producer_idx = 0;
	ch->consumer_idx = 0;
	atomic_set(&ch->rx_windows_ready, 0);
	ch->error = false;
	ch->last_rx_bytes = 0;
	memset(&ch->rx_app, 0, sizeof(ch->rx_app));

	/* Build the BD ring */
	int ret = build_bd_ring(dev, channel);
	if (ret) {
		return ret;
	}

	LOG_DBG("ch %u configured: %u BDs x %u bytes, cyclic=%d, thresh=%u", channel, ch->num_bds,
		ch->bd_buf_bytes, ch->cyclic, ch->irq_threshold);

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

	struct dma_xlnx_sg_data *data = dev->data;
	struct dma_xlnx_sg_chan *ch = &data->ch[channel];
	uint32_t dmasr = chan_read(dev, channel, REG_DMASR);

	/* If halted due to error, reset and rebuild */
	if ((dmasr & DMASR_HALTED) && ch->error) {
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
 * Internal helpers for continuous RX streaming.
 * -------------------------------------------------------------------------- */
static void dma_xlnx_sg_prepare_rx_stream(const struct device *dev,
					  const struct dma_xlnx_sg_rx_stream_cfg *cfg)
{
	struct dma_xlnx_sg_data *data = dev->data;
	struct dma_xlnx_sg_chan *ch = &data->ch[CH_RX];

	ch->cyclic = true;
	ch->callback = NULL;
	ch->user_data = NULL;
	ch->irq_timeout = 0U;
	ch->rx_stream_callback = cfg->callback;
	ch->rx_stream_user_data = cfg->user_data;
}


/* --------------------------------------------------------------------------
 * Reconfigure RX ring at runtime.
 * -------------------------------------------------------------------------- */
static int dma_xlnx_sg_reconfigure_rx(const struct device *dev, uint32_t bd_bytes,
				      uint16_t threshold)
{
	struct dma_xlnx_sg_data *data = dev->data;
	struct dma_xlnx_sg_chan *ch = &data->ch[CH_RX];

	if (bd_bytes == 0 || (size_t)bd_bytes * (size_t)ch->num_bds > buf_size(dev, CH_RX)) {
		LOG_ERR("bd_bytes %u * %u BDs exceeds RX buf size %zu", bd_bytes, ch->num_bds,
			buf_size(dev, CH_RX));
		return -EINVAL;
	}

	if (threshold == 0 || threshold > ch->num_bds || (ch->num_bds % threshold) != 0) {
		LOG_ERR("threshold %u must divide RX BD count %u evenly", threshold, ch->num_bds);
		return -EINVAL;
	}

	/*
	 * Stop the channel if running.  Halted channels (first call)
	 * skip the reset to avoid disrupting the DMA engine state.
	 * Running channels get a soft-reset to flush the prefetch
	 * pipeline and clear stale CMPLT bits.
	 */
	uint32_t dmasr = chan_read(dev, CH_RX, REG_DMASR);

	if (!(dmasr & DMASR_HALTED)) {
		int reset_ret = do_soft_reset(dev, CH_RX);

		if (reset_ret) {
			return reset_ret;
		}
	}

	/* Update ring parameters */
	ch->bd_buf_bytes = bd_bytes;
	ch->irq_threshold = threshold;

	/*
	 * Hardware IOC threshold is limited to a small value because
	 * IOC_IRQ does not reliably fire with large thresholds on this
	 * DMA IP (confirmed: threshold=16 works, threshold=128 does not).
	 * Always use software coalescing for the full window size.
	 */
	{
		uint8_t hw = 32;

		while (hw > 1 && (threshold % hw) != 0) {
			hw >>= 1;
		}
		ch->hw_irq_threshold = hw;
		ch->irq_coalesce_target = threshold / hw;
	}
	ch->irq_coalesce_count = 0;

	ch->consumer_idx = 0;
	ch->producer_idx = 0;
	atomic_set(&ch->rx_windows_ready, 0);
	ch->error = false;

	/* Rebuild and restart */
	int build_ret = build_bd_ring(dev, CH_RX);
	if (build_ret) {
		return build_ret;
	}

	kick_channel(dev, CH_RX);

	LOG_DBG("RX ring reconfigured: %u BDs x %u bytes, threshold=%u", ch->num_bds, bd_bytes,
		threshold);
	return 0;
}

static int dma_xlnx_sg_consume_rx_window(const struct device *dev, uint8_t **buf, uint32_t *size)
{
	struct dma_xlnx_sg_data *data = dev->data;
	struct dma_xlnx_sg_chan *ch = &data->ch[CH_RX];

	if (buf == NULL || size == NULL) {
		return -EINVAL;
	}

	*buf = NULL;
	*size = 0U;

	if (atomic_get(&ch->rx_windows_ready) <= 0) {
		return -EAGAIN;
	}

	uint32_t idx = ch->consumer_idx;
	uint32_t window_size = ch->irq_threshold;
	uint32_t window_bytes = ch->bd_buf_bytes * window_size;
	uintptr_t virt_base = buf_virt(dev, CH_RX);
	uint8_t *window_buf = (uint8_t *)(virt_base + (uintptr_t)idx * ch->bd_buf_bytes);

	if (window_size == 0U || window_size > RX_STREAM_WINDOWS_MAX) {
		return -EINVAL;
	}

	for (uint32_t i = 0; i < window_size; i++) {
		struct xlnx_sg_bd *bd = &ch->bds[idx];
		uint8_t *data_buf = (uint8_t *)(virt_base + (uintptr_t)idx * ch->bd_buf_bytes);
		uint32_t byte_count;

		cache_invd(bd, sizeof(*bd));
		byte_count = bd->status & DEV_CFG(dev)->sg_len_mask;
		cache_invd(data_buf, ch->bd_buf_bytes);

		ch->last_rx_bytes = byte_count;
		for (int a = 0; a < 5; a++) {
			ch->rx_app.app[a] = bd->app[a];
		}

		idx = (idx + 1) % ch->num_bds;
	}

	ch->consumer_idx = idx;
	atomic_dec(&ch->rx_windows_ready);

	*buf = window_buf;
	*size = window_bytes;
	return 0;
}

static void dma_xlnx_sg_rx_stream_work_handler(struct k_work *work)
{
	struct dma_xlnx_sg_chan *ch = CONTAINER_OF(work, struct dma_xlnx_sg_chan, rx_stream_work);
	const struct device *dev = ch->dev;

	if (dev == NULL || !ch->rx_stream_active) {
		return;
	}

	/*
	 * Ping-pong: ISR already advanced TAILDESC and DMA is processing
	 * the next half. We just consume all ready windows and deliver
	 * them to the callback. No DMA reconfiguration needed.
	 */
	while (atomic_get(&ch->rx_windows_ready) > 0) {
		uint8_t *buf = NULL;
		uint32_t size = 0U;
		int ret;

		ret = dma_xlnx_sg_consume_rx_window(dev, &buf, &size);
		if (ret != 0) {
			LOG_ERR("failed to consume RX stream window: %d", ret);
			break;
		}

		if (buf != NULL && size > 0U && ch->rx_stream_callback != NULL) {
			ch->rx_stream_callback(dev, ch->rx_stream_user_data, buf, size);
		}

		if (!ch->rx_stream_active) {
			return;
		}
	}
}

int dma_xlnx_sg_start_rx_stream(const struct device *dev,
				const struct dma_xlnx_sg_rx_stream_cfg *cfg)
{
	struct dma_xlnx_sg_data *data = dev->data;
	struct dma_xlnx_sg_chan *ch = &data->ch[CH_RX];
	int ret;

	if (cfg == NULL || cfg->callback == NULL) {
		return -EINVAL;
	}

	if (cfg->irq_threshold == 0U || cfg->irq_threshold > ch->num_bds ||
	    (ch->num_bds % cfg->irq_threshold) != 0U) {
		return -EINVAL;
	}

	if (cfg->bd_bytes == 0U ||
	    (size_t)cfg->bd_bytes * (size_t)ch->num_bds > buf_size(dev, CH_RX)) {
		return -EINVAL;
	}

	if (ch->rx_stream_active) {
		return -EBUSY;
	}

	dma_xlnx_sg_prepare_rx_stream(dev, cfg);
	ch->rx_stream_active = true;
	ret = dma_xlnx_sg_reconfigure_rx(dev, cfg->bd_bytes, cfg->irq_threshold);
	if (ret != 0) {
		ch->rx_stream_active = false;
		ch->rx_stream_callback = NULL;
		ch->rx_stream_user_data = NULL;
		return ret;
	}

	return 0;
}

void dma_xlnx_sg_stop_rx_stream(const struct device *dev)
{
	struct dma_xlnx_sg_data *data = dev->data;
	struct dma_xlnx_sg_chan *ch = &data->ch[CH_RX];

	if (!ch->rx_stream_active && ch->rx_stream_callback == NULL) {
		return;
	}

	ch->rx_stream_active = false;
	ch->rx_stream_callback = NULL;
	ch->rx_stream_user_data = NULL;
	atomic_set(&ch->rx_windows_ready, 0);

	(void)dma_xlnx_sg_stop(dev, CH_RX);
	(void)k_work_cancel_sync(&ch->rx_stream_work, &ch->rx_stream_work_sync);
}

#ifdef CONFIG_DMA_XLNX_AXI_DMA_SG_APP_FIELDS
/* --------------------------------------------------------------------------
 * Get APP fields from last completed RX descriptor
 * -------------------------------------------------------------------------- */
int dma_xlnx_sg_get_rx_app(const struct device *dev, struct dma_xlnx_sg_app_fields *app)
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
int dma_xlnx_sg_set_tx_app(const struct device *dev, const struct dma_xlnx_sg_app_fields *app)
{
	struct dma_xlnx_sg_data *data = dev->data;
	struct dma_xlnx_sg_chan *ch = &data->ch[CH_TX];

	memcpy(&ch->tx_app, app, sizeof(*app));

	/* Patch live BD(s) — update APP fields in all SOF descriptors */
	for (uint32_t i = 0; i < ch->num_bds; i++) {
		if (ch->bds[i].control & BD_CTRL_SOF) {
			for (int a = 0; a < 5; a++) {
				ch->bds[i].app[a] = app->app[a];
			}
			cache_flush(&ch->bds[i], sizeof(ch->bds[i]));
		}
	}

	return 0;
}
#endif

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
int dma_xlnx_sg_get_buffer(const struct device *dev, uint32_t channel, uintptr_t *phys,
			   uintptr_t *virt, size_t *size)
{
	if (channel >= NUM_CHANNELS) {
		return -EINVAL;
	}

	*phys = buf_phys(dev, channel);
	*virt = buf_virt(dev, channel);
	*size = buf_size(dev, channel);
	return 0;
}

/* ==========================================================================
 * Init & instantiation
 * ========================================================================== */

static int dma_xlnx_sg_init(const struct device *dev)
{
	const struct dma_xlnx_sg_cfg *cfg = dev->config;
	struct dma_xlnx_sg_data *data = dev->data;

	for (uint32_t channel = 0; channel < NUM_CHANNELS; channel++) {
		data->ch[channel].dev = dev;
	}
	data->ch[CH_RX].rx_stream_active = false;
	data->ch[CH_RX].rx_stream_callback = NULL;
	data->ch[CH_RX].rx_stream_user_data = NULL;
	k_work_init(&data->ch[CH_RX].rx_stream_work, dma_xlnx_sg_rx_stream_work_handler);

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

	/*
	 * Program SG_CTL with the configured AxCACHE value for M_AXI_SG
	 * descriptor fetch and write-back transactions. do_soft_reset()
	 * also restores this because PG021 reset returns registers to reset state.
	 */
	dma_xlnx_sg_program_sgctl(dev);

	/* Configure IRQs */
	cfg->irq_config(dev);

	LOG_INF("initialized (TX=%u BDs, RX=%u BDs, max_xfer=%u, sg_cache=0x%x)",
		data->ch[CH_TX].num_bds, data->ch[CH_RX].num_bds, cfg->sg_len_mask, cfg->sg_cache);
	return 0;
}

/* --------------------------------------------------------------------------
 * DT instantiation macro
 * -------------------------------------------------------------------------- */
#define DMA_XLNX_SG_INIT(inst)                                                                     \
                                                                                                   \
	static struct xlnx_sg_bd dma_xlnx_sg_tx_bds_##inst[CONFIG_DMA_XLNX_AXI_DMA_SG_NUM_TX_BD]   \
		__aligned(64);                                                                     \
	static struct xlnx_sg_bd dma_xlnx_sg_rx_bds_##inst[CONFIG_DMA_XLNX_AXI_DMA_SG_NUM_RX_BD]   \
		__aligned(64);                                                                     \
                                                                                                   \
	static void dma_xlnx_sg_irq_config_##inst(const struct device *dev)                        \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN_BY_IDX(inst, 0), DT_INST_IRQ_BY_IDX(inst, 0, priority),   \
			    dma_xlnx_sg_tx_isr, DEVICE_DT_INST_GET(inst), 0);                      \
		IF_ENABLED(CONFIG_SMP, (                                                         \
			sys_write8(BIT(0), GICD_ITARGETSRn + DT_INST_IRQN_BY_IDX(inst, 0));     \
		));                                                                              \
		irq_enable(DT_INST_IRQN_BY_IDX(inst, 0));                                          \
                                                                                                   \
		IRQ_CONNECT(DT_INST_IRQN_BY_IDX(inst, 1), DT_INST_IRQ_BY_IDX(inst, 1, priority),   \
			    dma_xlnx_sg_rx_isr, DEVICE_DT_INST_GET(inst), 0);                      \
		IF_ENABLED(CONFIG_SMP, (                                                         \
			sys_write8(BIT(0), GICD_ITARGETSRn + DT_INST_IRQN_BY_IDX(inst, 1));     \
		));                                                                              \
		irq_enable(DT_INST_IRQN_BY_IDX(inst, 1));                                          \
	}                                                                                          \
                                                                                                   \
	static const struct dma_xlnx_sg_cfg dma_xlnx_sg_cfg_##inst = {                             \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(regs, DT_DRV_INST(inst)),                       \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(tx_buf, DT_DRV_INST(inst)),                     \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(rx_buf, DT_DRV_INST(inst)),                     \
		.irq_config = dma_xlnx_sg_irq_config_##inst,                                       \
		.tx_buf_phys = DT_INST_REG_ADDR_BY_NAME(inst, tx_buf),                             \
		.rx_buf_phys = DT_INST_REG_ADDR_BY_NAME(inst, rx_buf),                             \
		.tx_buf_size = DT_INST_REG_SIZE_BY_NAME(inst, tx_buf),                             \
		.rx_buf_size = DT_INST_REG_SIZE_BY_NAME(inst, rx_buf),                             \
		.sg_len_mask = SG_LEN_MASK(DT_INST_PROP_OR(inst, xlnx_sg_length_width, 14)),       \
		.sg_cache = (uint8_t)DT_INST_PROP_OR(inst, xlnx_sg_cache, 0x3),                    \
	};                                                                                         \
                                                                                                   \
	static struct dma_xlnx_sg_data dma_xlnx_sg_data_##inst = {                                 \
		.ch =                                                                              \
			{                                                                          \
				[CH_TX] =                                                          \
					{                                                          \
						.bds = dma_xlnx_sg_tx_bds_##inst,                  \
						.num_bds = CONFIG_DMA_XLNX_AXI_DMA_SG_NUM_TX_BD,   \
					},                                                         \
				[CH_RX] =                                                          \
					{                                                          \
						.bds = dma_xlnx_sg_rx_bds_##inst,                  \
						.num_bds = CONFIG_DMA_XLNX_AXI_DMA_SG_NUM_RX_BD,   \
					},                                                         \
			},                                                                         \
	};                                                                                         \
                                                                                                   \
	BUILD_ASSERT(DT_INST_PROP(inst, xlnx_addrwidth) == 32,                                     \
		     "xlnx,axi-dma-sg: only 32-bit addressing supported");                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, dma_xlnx_sg_init, NULL, &dma_xlnx_sg_data_##inst,              \
			      &dma_xlnx_sg_cfg_##inst, POST_KERNEL, CONFIG_DMA_INIT_PRIORITY,      \
			      &dma_xlnx_sg_api);

DT_INST_FOREACH_STATUS_OKAY(DMA_XLNX_SG_INIT)
