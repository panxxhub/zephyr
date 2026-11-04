/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <kernel_arch_func.h>
#include <kernel_internal.h>
#include <ksched.h>
#include <kswap.h>
#include <kthread.h>
#include <timeslicing.h>
#include <usage.h>

bool z_sched_irq_exit_can_idle(void)
{
	__ASSERT_NO_MSG(!arch_cpu_irqs_are_enabled());

	struct _cpu *cpu = arch_curr_cpu();
	struct k_thread *thread = cpu->current;

	if (cpu->nested != 0U || thread != cpu->idle_thread || thread == NULL ||
	    cpu->swap_ok != 0U || !z_is_thread_ready(thread) || z_is_thread_halting(thread) ||
	    thread->switch_handle != NULL) {
		return false;
	}
#if CONFIG_NUM_METAIRQ_PRIORITIES > 0
	if (cpu->metairq_preempted != NULL) {
		return false;
	}
#endif
#ifdef CONFIG_TIMESLICING
	if (z_time_slice_pending()) {
		return false;
	}
#endif
	/* Do not skip the slow path's dispatch of IPIs to other CPUs either. */
	if (atomic_get(&_kernel.pending_ipi) != 0) {
		return false;
	}

	/* Never traverse concurrently modified run-queue links. This load is
	 * the idle decision's linearization point. A later remote enqueue
	 * requests an IPI after publishing the count, while local IRQs remain
	 * masked through exception return. The pending IPI will reenter the
	 * scheduler even if this exit has already decided to resume idle.
	 */
	return atomic_get(&cpu->ready_q.queued) == 0;
}

void *z_irq_exit_switch_handle(void *interrupted)
{
	if (!z_sched_irq_exit_can_idle()) {
		return z_get_next_switch_handle(interrupted);
	}

	z_check_stack_sentinel();
	z_sched_usage_switch(_current);
	return interrupted;
}
