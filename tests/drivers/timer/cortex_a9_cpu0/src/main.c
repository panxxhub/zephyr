/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/ztest.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/sys/atomic.h>

static atomic_t timer_irqs[2];
static atomic_t announces[2];
static atomic_t handlers[2];
static atomic_t started;
static atomic_t ipis[2];
static struct k_thread worker;
static struct k_thread dormant;
static K_THREAD_STACK_DEFINE(worker_stack, 2048);
static K_THREAD_STACK_DEFINE(dormant_stack, 1024);
static K_SEM_DEFINE(done, 0, 1);
static K_SEM_DEFINE(never, 0, 1);

void __real_sys_clock_announce_locked(uint32_t ticks, k_spinlock_key_t key);

void __wrap_sys_clock_announce_locked(uint32_t ticks, k_spinlock_key_t key)
{
	atomic_inc(&announces[arch_curr_cpu()->id]);
	__real_sys_clock_announce_locked(ticks, key);
}

#ifdef CONFIG_ARM_TRACING_IRQ
void sys_trace_isr_enter_user(void)
{
	if (arch_irq_get_active() == 0U) {
		atomic_inc(&ipis[arch_curr_cpu()->id]);
	}
	if (arch_irq_get_active() == DT_IRQN(DT_NODELABEL(private_timer))) {
		atomic_inc(&timer_irqs[arch_curr_cpu()->id]);
	}
}

#endif

static void expire(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	atomic_inc(&handlers[arch_curr_cpu()->id]);
}

K_TIMER_DEFINE(load, expire, NULL);
K_TIMER_DEFINE(remote, expire, NULL);

static void dormant_entry(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	atomic_set(&started, 1);
}

static void remote_waits(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	zassert_equal(arch_curr_cpu()->id, 1U);
	for (uint32_t i = 0U; i < 50U; i++) {
		int64_t before = k_uptime_get();

		k_timer_start(&remote, K_MSEC(1), K_NO_WAIT);
		k_sleep(K_MSEC(2));
		zassert_between_inclusive(k_uptime_get() - before, 2, 20);
		zassert_equal(k_timer_status_get(&remote), 1U);
		before = k_uptime_get();
		zassert_equal(k_sem_take(&never, K_MSEC(2)), -EAGAIN);
		zassert_between_inclusive(k_uptime_get() - before, 2, 20);
	}
	/* These APIs remain available without a local private timer. */
	uint32_t before = k_cycle_get_32();

	k_busy_wait(100);
	zassert_true(k_cycle_get_32() != before);
	k_sem_give(&done);
}

static void run_remote_waits(void)
{
	k_thread_create(&worker, worker_stack, K_THREAD_STACK_SIZEOF(worker_stack), remote_waits,
			NULL, NULL, NULL, 1, 0, K_FOREVER);
	zassert_ok(k_thread_cpu_pin(&worker, 1));
	k_thread_start(&worker);
	int32_t rc = k_sem_take(&done, K_SECONDS(3));

	printk("wait rc %d ipis %ld/%ld timer %ld/%ld handlers %ld/%ld\n", rc, atomic_get(&ipis[0]),
	       atomic_get(&ipis[1]), atomic_get(&timer_irqs[0]), atomic_get(&timer_irqs[1]),
	       atomic_get(&handlers[0]), atomic_get(&handlers[1]));
	zassert_ok(rc);
	zassert_ok(k_thread_join(&worker, K_SECONDS(1)));
}

ZTEST(cpu0_timer, test_remote_waits_and_dormant_boot)
{
	k_thread_create(&dormant, dormant_stack, K_THREAD_STACK_SIZEOF(dormant_stack),
			dormant_entry, NULL, NULL, NULL, 1, 0, K_FOREVER);
	zassert_ok(k_thread_cpu_pin(&dormant, 1));
	/* Freshly created, pinned, but never started: the not_ready analogue. */
	k_sleep(K_MSEC(20));
	zassert_equal(atomic_get(&started), 0);
	/* CPU 0 has only a distant wait: remote earlier deadlines must wake it. */
	run_remote_waits();
	k_timer_start(&load, K_MSEC(1), K_MSEC(1));
	run_remote_waits();
	k_timer_stop(&load);
	k_timer_stop(&remote);
	k_thread_abort(&dormant);
	printk("timer IRQs %ld/%ld, announces %ld/%ld, handlers %ld/%ld\n",
	       atomic_get(&timer_irqs[0]), atomic_get(&timer_irqs[1]), atomic_get(&announces[0]),
	       atomic_get(&announces[1]), atomic_get(&handlers[0]), atomic_get(&handlers[1]));
	zassert_true(atomic_get(&handlers[0]) > 0);
	if (IS_ENABLED(CONFIG_TIMEOUT_ANNOUNCE_CPU0) || IS_ENABLED(REQUIRE_CPU0)) {
		zassert_equal(atomic_get(&timer_irqs[1]), 0);
		zassert_equal(atomic_get(&announces[1]), 0);
		zassert_equal(atomic_get(&handlers[1]), 0);
	} else {
		zassert_true(atomic_get(&timer_irqs[1]) > 0);
		zassert_true(atomic_get(&announces[1]) > 0);
		zassert_true(atomic_get(&handlers[1]) > 0);
	}
}

ZTEST(cpu0_timer, test_tickless_idle)
{
	atomic_val_t before = atomic_get(&timer_irqs[0]);

	k_sleep(K_MSEC(100));
	if (IS_ENABLED(CONFIG_TIMEOUT_ANNOUNCE_CPU0) && IS_ENABLED(CONFIG_TICKLESS_KERNEL) &&
	    IS_ENABLED(CONFIG_ARM_TRACING_IRQ)) {
		zassert_between_inclusive(atomic_get(&timer_irqs[0]) - before, 1, 3);
		zassert_equal(atomic_get(&timer_irqs[1]), 0);
	}
}

#ifdef CONFIG_TIMESLICING
static struct k_thread peers[2];
static K_THREAD_STACK_ARRAY_DEFINE(peer_stacks, 2, 1024);
static atomic_t progress[2];
static atomic_t stop;

static void slice_peer(void *index, void *b, void *c)
{
	uintptr_t i = (uintptr_t)index;

	ARG_UNUSED(b);
	ARG_UNUSED(c);
	while (atomic_get(&stop) == 0) {
		atomic_inc(&progress[i]);
	}
}

ZTEST(cpu0_timer, test_timeslicing)
{
	k_sched_time_slice_set(2, 0);
	for (uintptr_t i = 0U; i < ARRAY_SIZE(peers); i++) {
		k_thread_create(&peers[i], peer_stacks[i], K_THREAD_STACK_SIZEOF(peer_stacks[i]),
				slice_peer, (void *)i, NULL, NULL, 1, 0, K_FOREVER);
		zassert_ok(k_thread_cpu_pin(&peers[i], 1));
		k_thread_start(&peers[i]);
	}
	k_sleep(K_MSEC(50));
	atomic_set(&stop, 1);
	for (size_t i = 0U; i < ARRAY_SIZE(peers); i++) {
		zassert_ok(k_thread_join(&peers[i], K_SECONDS(1)));
		zassert_true(atomic_get(&progress[i]) > 0, "CPU 1 peer %u starved", (uint32_t)i);
	}
	k_sched_time_slice_set(0, 0);
	if (IS_ENABLED(CONFIG_TIMEOUT_ANNOUNCE_CPU0) || IS_ENABLED(REQUIRE_CPU0)) {
		zassert_equal(atomic_get(&timer_irqs[1]), 0);
		zassert_equal(atomic_get(&announces[1]), 0);
	}
}
#endif

ZTEST_SUITE(cpu0_timer, NULL, NULL, NULL, NULL, NULL);
