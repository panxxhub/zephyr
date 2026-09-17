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

	if (recording && (irq == 6U || irq == 7U) && count < ARRAY_SIZE(events)) {
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

static void verify_nested_identity(void)
{
	static const uint32_t expected[] = {
		0x10006U, 0x30006U, 0x10007U, 0x30007U, 0x20007U, 0x40006U, 0x20006U,
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
	unsigned int key = irq_lock();

	zassert_equal(arch_irq_get_active(), K_IRQ_ACTIVE_NONE);
	irq_unlock(key);
}

#ifdef CONFIG_SMP
static struct k_thread remote_thread;
static K_THREAD_STACK_DEFINE(remote_stack, 2048);
static K_SEM_DEFINE(remote_done, 0, 1);

static void remote_test(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	zassert_equal(arch_curr_cpu()->id, 1U);
	verify_nested_identity();
	k_sem_give(&remote_done);
}
#endif

ZTEST(irq_trace, test_nested_identity)
{
	verify_nested_identity();
#ifdef CONFIG_SMP
	k_thread_create(&remote_thread, remote_stack, K_THREAD_STACK_SIZEOF(remote_stack),
			remote_test, NULL, NULL, NULL, 1, 0, K_FOREVER);
	zassert_ok(k_thread_cpu_pin(&remote_thread, 1));
	k_thread_start(&remote_thread);
	zassert_ok(k_sem_take(&remote_done, K_SECONDS(1)));
	zassert_ok(k_thread_join(&remote_thread, K_SECONDS(1)));
#endif
}

ZTEST_SUITE(irq_trace, NULL, NULL, NULL, NULL, NULL);
