/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/drivers/interrupt_controller/loapic.h>
#include <zephyr/kernel/smp.h>
#include <ksched.h>
#include <kthread.h>

#define BEFORE_DECISION 1
#define AFTER_DECISION  2
#define RACE_ROUNDS     32

#ifdef CONFIG_SCHED_IRQ_EXIT_FASTPATH
#ifdef CONFIG_TIMESLICING
static atomic_t force_slice;
#endif
static atomic_t fast_calls;
static atomic_t guard_errors;
static atomic_t gate_timeouts;
static struct k_ipi_work timer_work;
static atomic_t armed;
static atomic_t entered;
static atomic_t release_exit;
#endif
static atomic_t ran;
static atomic_t drop_ipi;
static atomic_t sent;
static atomic_t received;
static atomic_t slow_calls[2];
static struct k_thread worker;
K_THREAD_STACK_DEFINE(worker_stack, 2048);

void *__real_z_get_next_switch_handle(void *interrupted);
void __real_arch_sched_directed_ipi(uint32_t cpu_bitmap);
#ifdef CONFIG_SCHED_IRQ_EXIT_FASTPATH
void *__real_z_irq_exit_switch_handle(void *interrupted);
#endif

static bool wait_nonzero(atomic_t *value)
{
	uint32_t start = k_cycle_get_32();
	uint32_t limit = sys_clock_hw_cycles_per_sec() / 2U;

	while (atomic_get(value) == 0) {
		if ((uint32_t)(k_cycle_get_32() - start) > limit) {
			return false;
		}
		arch_nop();
	}
	return true;
}

void z_trace_sched_ipi(void)
{
	if (arch_curr_cpu()->id == 1U) {
		atomic_inc(&received);
	}
}

void __wrap_arch_sched_directed_ipi(uint32_t cpu_bitmap)
{
	if ((cpu_bitmap & BIT(1)) != 0U) {
		atomic_inc(&sent);
		if (atomic_get(&drop_ipi) != 0) {
			cpu_bitmap &= ~BIT(1);
		}
	}
	__real_arch_sched_directed_ipi(cpu_bitmap);
}

void *__wrap_z_get_next_switch_handle(void *interrupted)
{
	atomic_inc(&slow_calls[arch_curr_cpu()->id]);
	return __real_z_get_next_switch_handle(interrupted);
}

#ifdef CONFIG_SCHED_IRQ_EXIT_FASTPATH
#ifdef CONFIG_TIMESLICING
bool __real_z_time_slice_pending(void);

bool __wrap_z_time_slice_pending(void)
{
	return atomic_get(&force_slice) != 0 || __real_z_time_slice_pending();
}
#endif

/* Exercise the real predicate with restored state before exposing the race. */
static void check_guards(void)
{
	struct _cpu *cpu = arch_curr_cpu();
	struct k_thread *thread = cpu->current;
	uint8_t state = thread->base.thread_state;

	if (!z_sched_irq_exit_can_idle()) {
		atomic_inc(&guard_errors);
		return;
	}
	cpu->swap_ok = 1U;
	bool swap_rejected = !z_sched_irq_exit_can_idle();

	cpu->swap_ok = 0U;
	cpu->nested = 1U;
	bool nested_rejected = !z_sched_irq_exit_can_idle();

	cpu->nested = 0U;
	thread->base.thread_state = state | _THREAD_ABORTING;
	bool abort_rejected = !z_sched_irq_exit_can_idle();

	thread->base.thread_state = state;
	thread->switch_handle = thread;
	bool handle_rejected = !z_sched_irq_exit_can_idle();

	thread->switch_handle = NULL;
#ifdef CONFIG_TIMESLICING
	atomic_set(&force_slice, 1);
	bool slice_rejected = !z_sched_irq_exit_can_idle();

	atomic_clear(&force_slice);
	if (!slice_rejected) {
		atomic_inc(&guard_errors);
	}
#endif
	atomic_set_bit(&_kernel.pending_ipi, 1);
	bool ipi_rejected = !z_sched_irq_exit_can_idle();

	atomic_clear_bit(&_kernel.pending_ipi, 1);
	if (!ipi_rejected) {
		atomic_inc(&guard_errors);
	}
#if CONFIG_NUM_METAIRQ_PRIORITIES > 0
	cpu->metairq_preempted = thread;
	bool metairq_rejected = !z_sched_irq_exit_can_idle();

	cpu->metairq_preempted = NULL;
	if (!metairq_rejected) {
		atomic_inc(&guard_errors);
	}
#endif
	if (!swap_rejected || !nested_rejected || !abort_rejected || !handle_rejected) {
		atomic_inc(&guard_errors);
	}
}

static void hold_exit(void)
{
	atomic_set(&entered, 1);
	if (!wait_nonzero(&release_exit)) {
		atomic_inc(&gate_timeouts);
	}
}

void *__wrap_z_irq_exit_switch_handle(void *interrupted)
{
	bool target = arch_curr_cpu()->id == 1U && _current == arch_curr_cpu()->idle_thread;
	atomic_val_t mode = target ? atomic_clear(&armed) : 0;

	if (mode == BEFORE_DECISION) {
		hold_exit();
	}
	atomic_val_t before = atomic_get(&slow_calls[arch_curr_cpu()->id]);
	void *next = __real_z_irq_exit_switch_handle(interrupted);
	bool fast = before == atomic_get(&slow_calls[arch_curr_cpu()->id]);

	if (target && fast) {
		atomic_inc(&fast_calls);
	}
	if (mode == AFTER_DECISION) {
		if (!fast || next != interrupted) {
			atomic_inc(&guard_errors);
		}
		check_guards();
		hold_exit();
	} else if (mode == BEFORE_DECISION && fast) {
		atomic_inc(&guard_errors);
	}
	return next;
}
#endif

#ifdef CONFIG_SCHED_IRQ_EXIT_FASTPATH
static void mask_timer(struct k_ipi_work *work)
{
	ARG_UNUSED(work);
	/* CPU 1 must not get a periodic timer rescue in the dropped-IPI control. */
	x86_write_loapic(LOAPIC_TIMER, x86_read_loapic(LOAPIC_TIMER) | LOAPIC_LVT_MASKED);
}

#endif

static void worker_entry(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	atomic_set(&ran, 1);
}

static void create_worker(void)
{
	atomic_clear(&ran);
	k_thread_create(&worker, worker_stack, K_THREAD_STACK_SIZEOF(worker_stack), worker_entry,
			NULL, NULL, NULL, 1, 0, K_FOREVER);
	zassert_ok(k_thread_cpu_pin(&worker, 1));
}

#ifdef CONFIG_SCHED_IRQ_EXIT_FASTPATH
static void wait_idle(void)
{
	uint32_t start = k_cycle_get_32();

	while (_kernel.cpus[1].current != _kernel.cpus[1].idle_thread) {
		zassert_true((uint32_t)(k_cycle_get_32() - start) <
				     sys_clock_hw_cycles_per_sec() / 2U,
			     "CPU 1 did not idle");
		arch_nop();
	}
}

#endif

ZTEST(irq_exit, test_directed_enqueue)
{
	zassert_equal(arch_curr_cpu()->id, 0U);
	create_worker();
	k_thread_start(&worker);
	zassert_true(wait_nonzero(&ran), "remote thread did not run");
	zassert_ok(k_thread_join(&worker, K_FOREVER));
	zassert_true(atomic_get(&slow_calls[1]) > 0, "normal scheduler was not exercised");
}

ZTEST(irq_exit, test_enqueue_at_idle_exit)
{
#ifndef CONFIG_SCHED_IRQ_EXIT_FASTPATH
	ztest_test_skip();
#else
	k_ipi_work_init(&timer_work);
	zassert_ok(k_ipi_work_add(&timer_work, BIT(1), mask_timer));
	unsigned int key = irq_lock();

	k_ipi_work_signal();
	irq_unlock(key);
	zassert_ok(k_ipi_work_wait(&timer_work, K_FOREVER));
	/* Drain the work completion's CPU 0 wake before arming the exit gate. */
	k_busy_wait(1000);
	for (uint32_t round = 0; round < RACE_ROUNDS; round++) {
		for (int32_t mode = AFTER_DECISION; mode >= BEFORE_DECISION; mode--) {
			create_worker();
			wait_idle();
			atomic_clear(&entered);
			atomic_clear(&release_exit);
			atomic_set(&armed, mode);
			arch_sched_directed_ipi(BIT(1));
			zassert_true(wait_nonzero(&entered), "IRQ exit did not reach gate");
			atomic_val_t ipis = atomic_get(&received);
			atomic_val_t sends = atomic_get(&sent);
#ifdef IRQ_EXIT_DROP_IPI_REVERSE
			atomic_set(&drop_ipi, 1);
#endif
			k_thread_start(&worker);
			zassert_true(atomic_get(&sent) > sends, "enqueue did not request IPI");
			atomic_set(&release_exit, 1);
			bool woke = wait_nonzero(&ran);

			atomic_clear(&drop_ipi);
			if (!woke) {
				/* Record failure before the cleanup IPI. */
				arch_sched_directed_ipi(BIT(1));
			}
			zassert_ok(k_thread_join(&worker, K_FOREVER));
			zassert_true(woke, "racing enqueue lost: no directed IPI wake");
			zassert_true(atomic_get(&received) > ipis, "no post-enqueue directed IPI");
			zassert_equal(atomic_get(&guard_errors), 0, "incorrect idle guard");
			zassert_equal(atomic_get(&gate_timeouts), 0, "IRQ gate timed out");
		}
	}
	printk("Idle exit: %ld fast, %ld slow, %ld directed IPIs; %u races passed\n",
	       (long)atomic_get(&fast_calls), (long)atomic_get(&slow_calls[1]),
	       (long)atomic_get(&received), RACE_ROUNDS * 2U);
#endif
}

ZTEST_SUITE(irq_exit, NULL, NULL, NULL, NULL, NULL);
