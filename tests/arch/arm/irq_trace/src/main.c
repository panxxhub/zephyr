/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/ztest.h>
#include <zephyr/interrupt_util.h>

static volatile uint32_t events[8];
static volatile uint32_t count;
static volatile bool recording;

static void record(uint32_t tag)
{
	uint32_t irq = arch_irq_get_active();

	if (recording && count < ARRAY_SIZE(events)) {
		events[count++] = tag | irq;
	}
}

void sys_trace_isr_enter_user(void)
{
	record(0x10000U);
}

void sys_trace_isr_exit_user(void)
{
	record(0x20000U);
}

static void inner(const void *arg)
{
	ARG_UNUSED(arg);
	record(0x30000U);
}

static void outer(const void *arg)
{
	ARG_UNUSED(arg);
	record(0x30000U);
	trigger_irq(7);
	record(0x40000U);
}

ZTEST(irq_trace, test_nested_identity)
{
	static const uint32_t expected[] = {
		0x10006U, 0x30006U, 0x10007U, 0x30007U,
		0x20007U, 0x40006U, 0x20006U,
	};

	IRQ_CONNECT(6, IRQ_DEFAULT_PRIORITY, outer, NULL, 0);
	IRQ_CONNECT(7, 0, inner, NULL, 0);
	irq_enable(6);
	irq_enable(7);
	count = 0U;
	recording = true;
	trigger_irq(6);
	recording = false;
	zassert_equal(count, ARRAY_SIZE(expected), "trace count %u", count);
	for (size_t i = 0; i < ARRAY_SIZE(expected); i++) {
		zassert_equal(events[i], expected[i], "event %u: %x", (uint32_t)i, events[i]);
	}
	zassert_equal(arch_irq_get_active(), K_IRQ_ACTIVE_NONE);
}

ZTEST_SUITE(irq_trace, NULL, NULL, NULL, NULL, NULL);
