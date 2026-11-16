/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/devicetree.h>
#include <zephyr/irq.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/ztest.h>

static uint32_t __attribute__((section(".ocm_text"), noinline))
ocm_calculate(uint32_t input)
{
	return (input ^ 0x5a97b3c1U) + 17U;
}

static bool privileged_write_allowed(uintptr_t address)
{
	unsigned int key = irq_lock();
	uint32_t par;

	__asm__ volatile("mcr p15, 0, %1, c7, c8, 1\n\t"
			 "isb\n\t"
			 "mrc p15, 0, %0, c7, c4, 0"
			 : "=r" (par) : "r" (address) : "memory");
	irq_unlock(key);
	return (par & 1U) == 0U;
}

ZTEST(ocm_text, test_copy_execution_and_permissions)
{
	uintptr_t start = (uintptr_t)__ocm_text_start;
	uintptr_t end = (uintptr_t)__ocm_text_end;
	uintptr_t address = (uintptr_t)ocm_calculate;
	size_t size = end - start;
	uintptr_t ocm = DT_REG_ADDR(DT_CHOSEN(zephyr_ocm_text));
	uintptr_t ocm_end = ocm + DT_REG_SIZE(DT_CHOSEN(zephyr_ocm_text));

	zassert_true(size > 0U && start >= ocm && end <= ocm_end);
	zassert_true(address >= start && address < end);
	zassert_not_equal((uintptr_t)__ocm_text_load_start, start);
	zassert_mem_equal(__ocm_text_start, __ocm_text_load_start, size);
	zassert_false(privileged_write_allowed(start));
	zassert_true(privileged_write_allowed(DT_REG_ADDR(DT_CHOSEN(zephyr_ocm))));
	for (uint32_t i = 0; i < 128U; ++i) {
		zassert_equal(ocm_calculate(i), (i ^ 0x5a97b3c1U) + 17U);
	}
}

ZTEST_SUITE(ocm_text, NULL, NULL, NULL, NULL, NULL);
