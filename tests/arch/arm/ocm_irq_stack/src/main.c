/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/devicetree.h>
#include <zephyr/drivers/interrupt_controller/gic.h>
#include <zephyr/kernel_structs.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/ztest.h>

K_KERNEL_STACK_ARRAY_DECLARE(z_interrupt_stacks, CONFIG_MP_MAX_NUM_CPUS, CONFIG_ISR_STACK_SIZE);
static K_SEM_DEFINE(irq_done, 0, 1);

static void check_irq_stack(const void *arg)
{
	volatile uint32_t scratch[64];
	uint32_t seed = *(const uint32_t *)arg;
	uintptr_t address = (uintptr_t)scratch;
	uintptr_t bottom = (uintptr_t)K_KERNEL_STACK_BUFFER(z_interrupt_stacks[0]);
	uintptr_t top = bottom + K_KERNEL_STACK_SIZEOF(z_interrupt_stacks[0]);

	zassert_true(k_is_in_isr());
	zassert_true(address >= bottom && address + sizeof(scratch) <= top);
	for (size_t i = 0; i < ARRAY_SIZE(scratch); ++i) {
		scratch[i] = seed + i;
	}
	for (size_t i = 0; i < ARRAY_SIZE(scratch); ++i) {
		zassert_equal(scratch[i], seed + i);
	}
	k_sem_give(&irq_done);
}

ZTEST(ocm_irq_stack, test_placement_and_live_irq_stack)
{
	static uint32_t seed;
	uintptr_t start = (uintptr_t)z_interrupt_stacks;
	uintptr_t end = start + sizeof(z_interrupt_stacks);
	uintptr_t ocm = DT_REG_ADDR(DT_CHOSEN(zephyr_ocm));
	uintptr_t ocm_end = ocm + DT_REG_SIZE(DT_CHOSEN(zephyr_ocm));

	if (IS_ENABLED(CONFIG_ARM_ISR_STACKS_IN_OCM)) {
		zassert_true(start >= ocm && end <= ocm_end);
		zassert_true(end <= (uintptr_t)__ocm_bss_start ||
			     start >= (uintptr_t)__ocm_bss_end);
	} else {
		zassert_true(end <= ocm || start >= ocm_end);
	}
	zassert_equal(start % Z_KERNEL_STACK_OBJ_ALIGN, 0U);
	IRQ_CONNECT(8, 0xa0, check_irq_stack, &seed, 0);
	irq_enable(8);
	for (seed = 0; seed < 32U; ++seed) {
		gic_raise_sgi(8, 0, BIT(0));
		zassert_ok(k_sem_take(&irq_done, K_SECONDS(1)));
	}
	irq_disable(8);
}

ZTEST_SUITE(ocm_irq_stack, NULL, NULL, NULL, NULL, NULL);
