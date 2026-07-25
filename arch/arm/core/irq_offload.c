/*
 * Copyright (c) 2015 Intel corporation
 * Copyright 2025 Arm Limited and/or its affiliates <open-source-office@arm.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Software interrupts utility code - ARM implementation
 */

#include <zephyr/kernel.h>
#include <zephyr/irq_offload.h>
#include <cmsis_core.h>

#if defined(CONFIG_IRQ_OFFLOAD_NESTED) && defined(CONFIG_USE_SWITCH) && \
	(defined(CONFIG_CPU_AARCH32_CORTEX_A) || defined(CONFIG_CPU_AARCH32_CORTEX_R))
#define ARM_CORTEX_AR_NESTED_IRQ_OFFLOAD
#else
volatile irq_offload_routine_t offload_routine;
static const void *offload_param;
#endif

/* Called by z_arm_svc */
void z_irq_do_offload(void)
{
#ifdef ARM_CORTEX_AR_NESTED_IRQ_OFFLOAD
	irq_offload_routine_t routine = _current_cpu->arch.irq_offload_routine;

	routine(_current_cpu->arch.irq_offload_param);
#else
	offload_routine(offload_param);
#endif
}

void arch_irq_offload(irq_offload_routine_t routine, const void *parameter)
{
#if defined(CONFIG_ARMV6_M_ARMV8_M_BASELINE) && !defined(CONFIG_ARMV8_M_BASELINE) \
	&& defined(CONFIG_ASSERT)
	/* ARMv6-M HardFault if you make a SVC call with interrupts locked.
	 */
	__ASSERT(__get_PRIMASK() == 0U, "irq_offload called with interrupts locked\n");
#endif /* CONFIG_ARMV6_M_ARMV8_M_BASELINE && CONFIG_ASSERT */

#ifdef ARM_CORTEX_AR_NESTED_IRQ_OFFLOAD
	bool from_isr = k_is_in_isr();
	struct _cpu *cpu;
	irq_offload_routine_t previous_routine;
	const void *previous_param;

	/*
	 * A thread caller must not be preempted between publishing its per-CPU
	 * callback and taking SVC.  An ISR caller is already non-schedulable;
	 * taking k_sched_lock() there is invalid.
	 */
	if (!from_isr) {
		k_sched_lock();
	}

	/*
	 * Pin the CPU pointer only after a thread caller has disabled
	 * preemption.  Otherwise a migration between reading and publishing
	 * the callback state could mix two CPUs' slots.
	 */
	cpu = _current_cpu;
	previous_routine = cpu->arch.irq_offload_routine;
	previous_param = cpu->arch.irq_offload_param;
	cpu->arch.irq_offload_routine = routine;
	cpu->arch.irq_offload_param = parameter;
#else
	k_sched_lock();
	offload_routine = routine;
	offload_param = parameter;
#endif

	__asm__ volatile ("svc %[id]\n"
			  IF_ENABLED(CONFIG_ARM_BTI, ("bti"))
			  :
			  : [id] "i" (_SVC_CALL_IRQ_OFFLOAD)
			  : "memory");

#ifdef ARM_CORTEX_AR_NESTED_IRQ_OFFLOAD
	cpu->arch.irq_offload_routine = previous_routine;
	cpu->arch.irq_offload_param = previous_param;

	if (!from_isr) {
		k_sched_unlock();
	}
#else
	offload_routine = NULL;
	k_sched_unlock();
#endif
}

void arch_irq_offload_init(void)
{
}
