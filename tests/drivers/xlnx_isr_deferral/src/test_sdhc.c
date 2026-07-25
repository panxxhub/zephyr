/*
 * Copyright (c) 2026 Moton Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/irq_offload.h>
#include <zephyr/ztest.h>

/*
 * Include the driver so this test can inject register status into its private
 * ISR. No xlnx,zynq-sdhc DT instance exists on native_sim, so this does not
 * instantiate a production device.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-variable"
#include "../../../../drivers/sdhc/xlnx_zynq_sdhc.c"
#pragma GCC diagnostic pop

struct sdhc_irq_test_context {
	struct zynq_sdhc_reg regs;
	adma_desc_t descriptors[1];
	struct zynq_sdhc_data data;
	struct zynq_sdhc_config config;
	struct device dev;
	bool entered_isr;
};

static void run_sdhc_isr(const void *arg)
{
	struct sdhc_irq_test_context *ctx = (struct sdhc_irq_test_context *)arg;

	ctx->entered_isr = k_is_in_isr();
	zynq_sdhc_isr(&ctx->dev);
}

static void init_sdhc_context(struct sdhc_irq_test_context *ctx)
{
	ctx->config._mmio.addr = (mm_reg_t)&ctx->regs;
	ctx->dev.config = &ctx->config;
	ctx->dev.data = &ctx->data;
	k_event_init(&ctx->data.irq_event);
}

ZTEST(xlnx_sdhc_isr_deferral, test_command_error_is_latched_for_thread)
{
	struct sdhc_irq_test_context ctx = {
		.data = {
			.adma_desc_tbl = ctx.descriptors,
		},
	};
	uint16_t error = ZYNQ_SDHC_HOST_CMD_CRC_ERR;

	init_sdhc_context(&ctx);
	ctx.regs.err_int_stat = error;
	irq_offload(run_sdhc_isr, &ctx);

	zassert_true(ctx.entered_isr);
	zassert_equal(ctx.regs.err_int_stat, error);
	zassert_equal(wait_for_cmd_complete(&ctx.data, 1), -EIO);
}

ZTEST(xlnx_sdhc_isr_deferral, test_transfer_error_is_latched_for_thread)
{
	struct sdhc_irq_test_context ctx = {
		.data = {
			.adma_desc_tbl = ctx.descriptors,
		},
	};
	uint16_t error = ZYNQ_SDHC_HOST_DATA_CRC_ERR;

	init_sdhc_context(&ctx);
	ctx.regs.err_int_stat = error;
	irq_offload(run_sdhc_isr, &ctx);

	zassert_true(ctx.entered_isr);
	zassert_equal(ctx.regs.err_int_stat, error);
	zassert_equal(wait_xfr_intr_complete(&ctx.dev, 1), -EIO);
}

ZTEST(xlnx_sdhc_isr_deferral, test_normal_status_is_latched_for_thread)
{
	struct sdhc_irq_test_context ctx = {
		.data = {
			.adma_desc_tbl = ctx.descriptors,
		},
	};

	init_sdhc_context(&ctx);
	ctx.regs.normal_int_stat = ZYNQ_SDHC_HOST_CMD_COMPLETE;
	irq_offload(run_sdhc_isr, &ctx);

	zassert_true(ctx.entered_isr);
	zassert_equal(ctx.regs.normal_int_stat, ZYNQ_SDHC_HOST_CMD_COMPLETE);
	zassert_ok(wait_for_cmd_complete(&ctx.data, 1));
}

ZTEST_SUITE(xlnx_sdhc_isr_deferral, NULL, NULL, NULL, NULL, NULL);
