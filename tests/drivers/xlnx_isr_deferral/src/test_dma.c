/*
 * Copyright (c) 2026 Moton Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/irq_offload.h>
#include <zephyr/ztest.h>

/*
 * Include the driver so this test can exercise its private ISR/session
 * boundary. No xlnx,axi-dma-sg DT instance exists on native_sim, so this
 * does not instantiate a production device.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#include "../../../../drivers/dma/dma_xlnx_axi_dma_sg.c"
#pragma GCC diagnostic pop

#define TEST_BD_COUNT 4U
#define TEST_BD_BYTES 16U

struct callback_probe {
	struct k_sem sem;
	int status;
	uint32_t channel;
	uint32_t count;
	bool in_isr;
};

struct dma_irq_test_context {
	struct dma_xlnx_sg_data data;
	struct dma_xlnx_sg_cfg config;
	struct device dev;
	struct xlnx_sg_bd tx_bds[TEST_BD_COUNT];
	struct xlnx_sg_bd rx_bds[TEST_BD_COUNT];
	uint8_t rx_buffer[TEST_BD_COUNT * TEST_BD_BYTES];
	uint32_t regs[32];
	uint32_t channel;
	bool entered_isr;
};

struct workqueue_blocker {
	struct k_work work;
	struct k_sem entered;
	struct k_sem release;
};

static void dma_test_callback(const struct device *dev, void *user_data, uint32_t channel,
			      int status)
{
	struct callback_probe *probe = user_data;

	zassert_not_null(dev);
	probe->status = status;
	probe->channel = channel;
	probe->count++;
	probe->in_isr = k_is_in_isr();
	k_sem_give(&probe->sem);
}

static void stream_test_callback(const struct device *dev, void *user_data, uint8_t *buf,
				 uint32_t size)
{
	struct callback_probe *probe = user_data;

	zassert_not_null(dev);
	zassert_not_null(buf);
	zassert_equal(size, 2U * TEST_BD_BYTES);
	probe->count++;
	probe->in_isr = k_is_in_isr();
	k_sem_give(&probe->sem);
}

static void blocker_handler(struct k_work *work)
{
	struct workqueue_blocker *blocker = CONTAINER_OF(work, struct workqueue_blocker, work);

	k_sem_give(&blocker->entered);
	(void)k_sem_take(&blocker->release, K_FOREVER);
}

static void block_system_workqueue(struct workqueue_blocker *blocker)
{
	k_work_init(&blocker->work, blocker_handler);
	k_sem_init(&blocker->entered, 0, 1);
	k_sem_init(&blocker->release, 0, 1);
	zassert_true(k_work_submit(&blocker->work) > 0);
	zassert_ok(k_sem_take(&blocker->entered, K_SECONDS(1)));
}

static void unblock_system_workqueue(struct workqueue_blocker *blocker)
{
	struct k_work_sync sync;

	k_sem_give(&blocker->release);
	(void)k_work_flush(&blocker->work, &sync);
}

static void init_probe(struct callback_probe *probe)
{
	k_sem_init(&probe->sem, 0, 1);
}

static void init_dma_context(struct dma_irq_test_context *ctx)
{
	ctx->config.regs.addr = (mm_reg_t)ctx->regs;
	ctx->config.rx_buf_phys = (uintptr_t)ctx->rx_buffer;
	ctx->config.rx_buf_size = sizeof(ctx->rx_buffer);
	ctx->config.sg_len_mask = 0x3FFFU;
	ctx->dev.config = &ctx->config;
	ctx->dev.data = &ctx->data;

	for (uint32_t channel = 0; channel < NUM_CHANNELS; channel++) {
		struct dma_xlnx_sg_chan *ch = &ctx->data.ch[channel];

		ch->dev = &ctx->dev;
		ch->bds = channel == CH_TX ? ctx->tx_bds : ctx->rx_bds;
		ch->num_bds = TEST_BD_COUNT;
		atomic_set(&ch->irq_event, 0);
		atomic_set(&ch->session_generation, 0);
		atomic_set(&ch->rx_stream_active, 0);
		k_work_init(&ch->irq_work, channel == CH_TX ? dma_xlnx_sg_tx_irq_work_handler
							    : dma_xlnx_sg_rx_irq_work_handler);
	}
	k_work_init(&ctx->data.ch[CH_RX].rx_stream_work, dma_xlnx_sg_rx_stream_work_handler);
}

static void begin_channel_session(struct dma_xlnx_sg_chan *ch)
{
	k_spinlock_key_t key = k_spin_lock(&ch->state_lock);

	dma_xlnx_sg_begin_session_locked(ch);
	k_spin_unlock(&ch->state_lock, key);
}

static void quiesce_channel_session(struct dma_xlnx_sg_chan *ch)
{
	k_spinlock_key_t key = k_spin_lock(&ch->state_lock);

	dma_xlnx_sg_quiesce_channel_locked(ch);
	k_spin_unlock(&ch->state_lock, key);
}

static uint32_t status_index(uint32_t channel)
{
	return ((channel == CH_TX ? MM2S_BASE : S2MM_BASE) + REG_DMASR) / sizeof(uint32_t);
}

static uint32_t taildesc_index(uint32_t channel)
{
	return ((channel == CH_TX ? MM2S_BASE : S2MM_BASE) + REG_TAILDESC) / sizeof(uint32_t);
}

static void run_dma_isr(const void *arg)
{
	struct dma_irq_test_context *ctx = (struct dma_irq_test_context *)arg;

	ctx->entered_isr = k_is_in_isr();
	if (ctx->channel == CH_TX) {
		dma_xlnx_sg_tx_isr(&ctx->dev);
	} else {
		dma_xlnx_sg_rx_isr(&ctx->dev);
	}
}

static void inject_irq(struct dma_irq_test_context *ctx, uint32_t channel, uint32_t status)
{
	ctx->channel = channel;
	ctx->regs[status_index(channel)] = status;
	irq_offload(run_dma_isr, ctx);
}

static void run_deferred_irq(uint32_t channel, uint32_t injected_status, int expected_status)
{
	struct dma_irq_test_context ctx = {0};
	struct callback_probe probe = {0};
	struct dma_xlnx_sg_chan *ch;
	struct k_work_sync sync;

	init_dma_context(&ctx);
	init_probe(&probe);
	ch = &ctx.data.ch[channel];
	ch->callback = dma_test_callback;
	ch->user_data = &probe;
	if (channel == CH_RX) {
		ch->active_bds = 1U;
		ch->bds[0].status = BD_STS_CMPLT | TEST_BD_BYTES;
	}
	begin_channel_session(ch);

	inject_irq(&ctx, channel, injected_status);
	zassert_ok(k_sem_take(&probe.sem, K_SECONDS(1)));
	(void)k_work_flush(&ch->irq_work, &sync);

	zassert_true(ctx.entered_isr);
	zassert_equal(ctx.regs[status_index(channel)], injected_status & DMASR_IRQ_BITS);
	zassert_equal(atomic_get(&ch->irq_event), 0);
	zassert_false(probe.in_isr);
	zassert_equal(probe.channel, channel);
	zassert_equal(probe.status, expected_status);
	zassert_equal(probe.count, 1U);
}

ZTEST(xlnx_dma_isr_deferral, test_completion_callback_runs_in_thread)
{
	run_deferred_irq(CH_TX, DMASR_IOC_IRQ, DMA_STATUS_COMPLETE);
}

ZTEST(xlnx_dma_isr_deferral, test_tx_error_injection_is_reported_from_thread)
{
	run_deferred_irq(CH_TX, DMASR_SLVERR | DMASR_ERR_IRQ, -EIO);
}

ZTEST(xlnx_dma_isr_deferral, test_rx_error_injection_is_reported_from_thread)
{
	run_deferred_irq(CH_RX, DMASR_SGSLVERR | DMASR_ERR_IRQ, -EIO);
}

ZTEST(xlnx_dma_isr_deferral, test_combined_error_and_completion_reports_once)
{
	run_deferred_irq(CH_TX, DMASR_INTERR | DMASR_ERR_IRQ | DMASR_IOC_IRQ, -EIO);
}

ZTEST(xlnx_dma_isr_deferral, test_repeated_irqs_merge_while_worker_is_blocked)
{
	struct dma_irq_test_context ctx = {0};
	struct callback_probe probe = {0};
	struct workqueue_blocker blocker = {0};
	struct dma_xlnx_sg_chan *ch;
	struct k_work_sync sync;
	uint32_t queued_event;

	init_dma_context(&ctx);
	init_probe(&probe);
	ch = &ctx.data.ch[CH_TX];
	ch->callback = dma_test_callback;
	ch->user_data = &probe;
	begin_channel_session(ch);
	block_system_workqueue(&blocker);

	inject_irq(&ctx, CH_TX, DMASR_IOC_IRQ);
	inject_irq(&ctx, CH_TX, DMASR_SLVERR | DMASR_ERR_IRQ);
	queued_event = (uint32_t)atomic_get(&ch->irq_event);

	unblock_system_workqueue(&blocker);
	zassert_ok(k_sem_take(&probe.sem, K_SECONDS(1)));
	(void)k_work_flush(&ch->irq_work, &sync);

	zassert_equal(queued_event & IRQ_EVENT_STATUS_MASK,
		      DMASR_IOC_IRQ | DMASR_SLVERR | DMASR_ERR_IRQ);
	zassert_equal(probe.count, 1U);
	zassert_equal(probe.status, -EIO);
	zassert_false(probe.in_isr);
}

ZTEST(xlnx_dma_isr_deferral, test_stream_rearms_before_blocked_workqueue_runs)
{
	struct dma_irq_test_context ctx = {0};
	struct callback_probe probe = {0};
	struct workqueue_blocker blocker = {0};
	struct dma_xlnx_sg_chan *ch;
	struct k_work_sync sync;
	uint32_t producer_idx;
	uint32_t tail_idx;
	uint32_t taildesc;
	uint32_t ready;
	uint32_t bd0_status;
	uint32_t bd1_status;

	init_dma_context(&ctx);
	init_probe(&probe);
	ch = &ctx.data.ch[CH_RX];
	ch->bd_buf_bytes = TEST_BD_BYTES;
	ch->irq_threshold = 2U;
	ch->hw_irq_threshold = 1U;
	ch->irq_coalesce_target = 2U;
	ch->tail_idx = 0U;
	ch->rx_stream_callback = stream_test_callback;
	ch->rx_stream_user_data = &probe;
	ch->bds[0].status = BD_STS_CMPLT | TEST_BD_BYTES;
	ch->bds[1].status = BD_STS_CMPLT | TEST_BD_BYTES;
	begin_channel_session(ch);
	atomic_set(&ch->rx_stream_active, 1);
	block_system_workqueue(&blocker);

	inject_irq(&ctx, CH_RX, DMASR_IOC_IRQ);
	inject_irq(&ctx, CH_RX, DMASR_IOC_IRQ);

	/* Capture the hard-IRQ results while all system work is still blocked. */
	producer_idx = ch->producer_idx;
	tail_idx = ch->tail_idx;
	taildesc = ctx.regs[taildesc_index(CH_RX)];
	ready = (uint32_t)atomic_get(&ch->rx_windows_ready);
	bd0_status = ch->bds[0].status;
	bd1_status = ch->bds[1].status;

	unblock_system_workqueue(&blocker);
	zassert_ok(k_sem_take(&probe.sem, K_SECONDS(1)));
	(void)k_work_flush(&ch->rx_stream_work, &sync);

	zassert_equal(producer_idx, 2U);
	zassert_equal(tail_idx, 2U);
	zassert_equal(taildesc, (uint32_t)(uintptr_t)&ch->bds[2]);
	zassert_equal(ready, 1U);
	zassert_equal(bd0_status & BD_STS_CMPLT, 0U);
	zassert_equal(bd1_status & BD_STS_CMPLT, 0U);
	zassert_equal(probe.count, 1U);
	zassert_false(probe.in_isr);
}

ZTEST(xlnx_dma_isr_deferral, test_stop_restart_discards_stale_deferred_event)
{
	struct dma_irq_test_context ctx = {0};
	struct callback_probe old_probe = {0};
	struct callback_probe new_probe = {0};
	struct workqueue_blocker blocker = {0};
	struct dma_xlnx_sg_chan *ch;
	struct k_work_sync sync;

	init_dma_context(&ctx);
	init_probe(&old_probe);
	init_probe(&new_probe);
	ch = &ctx.data.ch[CH_TX];
	ch->callback = dma_test_callback;
	ch->user_data = &old_probe;
	begin_channel_session(ch);
	block_system_workqueue(&blocker);

	inject_irq(&ctx, CH_TX, DMASR_IOC_IRQ);

	/* These are the exact generation transitions used by stop and start. */
	quiesce_channel_session(ch);
	ch->callback = dma_test_callback;
	ch->user_data = &new_probe;
	begin_channel_session(ch);

	unblock_system_workqueue(&blocker);
	(void)k_work_flush(&ch->irq_work, &sync);
	zassert_equal(old_probe.count, 0U);
	zassert_equal(new_probe.count, 0U);

	inject_irq(&ctx, CH_TX, DMASR_IOC_IRQ);
	zassert_ok(k_sem_take(&new_probe.sem, K_SECONDS(1)));
	(void)k_work_flush(&ch->irq_work, &sync);
	zassert_equal(new_probe.count, 1U);
	zassert_equal(new_probe.status, DMA_STATUS_COMPLETE);
}

ZTEST(xlnx_dma_isr_deferral, test_stream_combined_error_does_not_rearm)
{
	struct dma_irq_test_context ctx = {0};
	struct workqueue_blocker blocker = {0};
	struct dma_xlnx_sg_chan *ch;
	struct k_work_sync sync;
	uint32_t queued_event;

	init_dma_context(&ctx);
	ch = &ctx.data.ch[CH_RX];
	ch->hw_irq_threshold = 1U;
	ch->irq_coalesce_target = 1U;
	ch->tail_idx = 0U;
	ch->bds[0].status = BD_STS_CMPLT | TEST_BD_BYTES;
	begin_channel_session(ch);
	atomic_set(&ch->rx_stream_active, 1);
	block_system_workqueue(&blocker);

	inject_irq(&ctx, CH_RX, DMASR_IOC_IRQ | DMASR_ERR_IRQ | DMASR_SLVERR);
	queued_event = (uint32_t)atomic_get(&ch->irq_event);

	unblock_system_workqueue(&blocker);
	(void)k_work_flush(&ch->irq_work, &sync);

	zassert_equal(queued_event & IRQ_EVENT_STATUS_MASK,
		      DMASR_ERR_IRQ | DMASR_SLVERR);
	zassert_equal(ch->producer_idx, 0U);
	zassert_equal(ch->tail_idx, 0U);
	zassert_equal(ch->bds[0].status & BD_STS_CMPLT, BD_STS_CMPLT);
	zassert_equal(atomic_get(&ch->rx_windows_ready), 0);
}

ZTEST(xlnx_dma_isr_deferral, test_stopped_stream_cannot_rearm)
{
	struct dma_irq_test_context ctx = {0};
	struct dma_xlnx_sg_chan *ch;

	init_dma_context(&ctx);
	ch = &ctx.data.ch[CH_RX];
	ch->hw_irq_threshold = 1U;
	ch->irq_coalesce_target = 1U;
	ch->tail_idx = 0U;
	ch->bds[0].status = BD_STS_CMPLT | TEST_BD_BYTES;
	begin_channel_session(ch);
	atomic_set(&ch->rx_stream_active, 1);
	quiesce_channel_session(ch);

	inject_irq(&ctx, CH_RX, DMASR_IOC_IRQ);

	zassert_equal(ch->producer_idx, 0U);
	zassert_equal(ch->tail_idx, 0U);
	zassert_equal(ch->bds[0].status & BD_STS_CMPLT, BD_STS_CMPLT);
	zassert_equal(atomic_get(&ch->rx_windows_ready), 0);
}

ZTEST_SUITE(xlnx_dma_isr_deferral, NULL, NULL, NULL, NULL, NULL);
