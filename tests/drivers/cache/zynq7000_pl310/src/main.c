/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

/* Compile the actual composite driver with a fake inner cache and PL310. */
#define CONFIG_EXTERNAL_CACHE 1
#define CONFIG_DCACHE 1
#include <zephyr/ztest.h>
#include <zephyr/arch/cache.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/sys_io.h>

struct operation {
	char kind;
	uintptr_t address;
	size_t value;
};

static struct operation operations[128];
static size_t operation_count;
static uint32_t control;
static uintptr_t busy_register;
static uint32_t busy_value;
static uint32_t busy_reads;
static bool bad_translation;
static bool model_stack;
static bool model_shutdown;
static bool shutdown_lock_held;
static bool shutdown_l1_disabled;
static uint32_t stack_l1;
static uint32_t stack_memory;

static void check_reboot_sequence(void);

static void record(char kind, uintptr_t address, size_t value)
{
	zassert_true(operation_count < ARRAY_SIZE(operations));
	operations[operation_count++] = (struct operation){kind, address, value};
}

static uint32_t fake_read32(mem_addr_t address)
{
	if (address == 0xf8f02100U) {
		return control;
	}
	if (address == busy_register && busy_reads > 0U) {
		busy_reads--;
		return busy_value;
	}
	return 0U;
}

static void fake_write32(uint32_t value, mem_addr_t address)
{
	/* A second command before a background way/sync completes would cause SLVERR. */
	zassert_equal(busy_reads, 0U);
	record('W', address, value);
	if (model_stack && address == 0xf8f027fcU) {
		/* Model a callee saving registers on the stack during the L2 operation. */
		stack_l1 = 0xfeed1234U;
	}
	if (address == 0xf8000200U) {
		check_reboot_sequence();
		ztest_test_pass();
	}
	if (address == 0xf8f02100U) {
		control = value;
	}
	if (address == 0xf8f0277cU || address == 0xf8f027bcU || address == 0xf8f027fcU ||
	    address == 0xf8f02730U) {
		busy_register = address;
		busy_value = address == 0xf8f02730U ? 1U : value;
		busy_reads = 2U;
	}
}

int zynq_pl310_virt_to_phys(uintptr_t va, uintptr_t *pa)
{
	record('T', va, 0U);
	if (bad_translation) {
		return -EFAULT;
	}
	/* Adjacent virtual pages deliberately have non-adjacent physical backing. */
	*pa = va == 0x10000000U ? 0x20000000U : 0x30004000U;
	return 0;
}

static void fake_dsb(void)
{
	record('B', 0U, 0U);
}
static void fake_enable(void)
{
	record('E', 0U, 0U);
}
static void fake_disable(void)
{
	/* arch_dcache_disable() cleans L1 before clearing SCTLR.C. */
	record('C', 0U, 0U);
	record('D', 0U, 0U);
	shutdown_l1_disabled = model_shutdown;
}
static int fake_clean_all(void)
{
	stack_memory = stack_l1;
	record('C', 0U, 0U);
	return 0;
}
static int fake_invalidate_all(void)
{
	stack_l1 = stack_memory;
	record('I', 0U, 0U);
	return 0;
}
static int fake_clean_invalidate_all(void)
{
	stack_memory = stack_l1;
	record('F', 0U, 0U);
	return 0;
}

static int fake_clean(void *address, size_t size)
{
	record('C', (uintptr_t)address, size);
	return 0;
}
static int fake_invalidate(void *address, size_t size)
{
	record('I', (uintptr_t)address, size);
	return 0;
}

/* Terminal reset must not return to SMP lock operations with L1 disabled. */
static k_spinlock_key_t checked_spin_lock(struct k_spinlock *lock)
{
	if (model_shutdown) {
		zassert_false(shutdown_l1_disabled, "Lock acquired with L1 disabled");
		zassert_false(shutdown_lock_held);
		shutdown_lock_held = true;
		return (k_spinlock_key_t){0};
	}
	return k_spin_lock(lock);
}

static void checked_spin_unlock(struct k_spinlock *lock, k_spinlock_key_t key)
{
	if (model_shutdown) {
		zassert_false(shutdown_l1_disabled, "SMP lock release after L1 disable");
		shutdown_lock_held = false;
		return;
	}
	k_spin_unlock(lock, key);
}

#define k_spin_lock checked_spin_lock
#define k_spin_unlock checked_spin_unlock
#define sys_read32 fake_read32
#define sys_write32 fake_write32
#define barrier_dsync_fence_full fake_dsb
#define arch_dcache_enable fake_enable
#define arch_dcache_disable fake_disable
#define arch_dcache_flush_all fake_clean_all
#define arch_dcache_invd_all fake_invalidate_all
#define arch_dcache_flush_and_invd_all fake_clean_invalidate_all
#define arch_dcache_flush_range fake_clean
#define arch_dcache_invd_range fake_invalidate
#include "../../../../../drivers/cache/cache_xlnx_zynq7000_pl310.c"

#undef k_spin_lock
#undef k_spin_unlock

/* The real reset hook uses the fake controller and reset register. */
static unsigned int fake_irq_lock(void)
{
	return 0U;
}

#undef SLCR_UNLOCK
#define CONFIG_SOC_XLNX_ZYNQ7000_L2_CACHE 1
#undef irq_lock
#define irq_lock fake_irq_lock
#define sys_arch_reboot test_sys_arch_reboot
#include "../../../../../soc/xlnx/zynq7000/common/reboot.c"
#undef sys_arch_reboot
#undef irq_lock

static void before(void *fixture)
{
	ARG_UNUSED(fixture);
	operation_count = 0U;
	busy_register = 0U;
	busy_reads = 0U;
	control = 1U;
	bad_translation = false;
	model_stack = false;
	model_shutdown = false;
	shutdown_lock_held = false;
	shutdown_l1_disabled = false;
	stack_l1 = 0U;
	stack_memory = 0U;
	pl310_enabled = true;
}

static size_t find_operation(char kind, uintptr_t address, size_t value)
{
	for (size_t i = 0U; i < operation_count; i++) {
		if (operations[i].kind == kind && operations[i].address == address &&
		    operations[i].value == value) {
			return i;
		}
	}
	zassert_unreachable("Missing operation %c at %lx", kind, (unsigned long)address);
	return SIZE_MAX;
}

static size_t count_kind(char kind)
{
	size_t count = 0U;

	for (size_t i = 0U; i < operation_count; i++) {
		if (operations[i].kind == kind) {
			count++;
		}
	}
	return count;
}

ZTEST(pl310, test_primary_initialization_and_completion)
{
	const struct operation writes[] = {
		{'W', 0xf8f02100U, 0U},
		{'W', 0xf8000008U, 0xdf0dU},
		{'W', 0xf8000a1cU, 0x00020202U},
		{'W', 0xf8f02104U, 0x72360000U},
		{'W', 0xf8f02108U, 0x0111U},
		{'W', 0xf8f0210cU, 0x0121U},
		{'W', 0xf8f0277cU, 0xffffU},
		{'W', 0xf8f02730U, 0U},
		{'W', 0xf8f02214U, 0U},
		{'W', 0xf8f02220U, 0x1ffU},
		{'W', 0xf8f02100U, 1U},
		{'W', 0xf8f02730U, 0U},
	};
	size_t next = 0U;

	pl310_enabled = false;
	control = 0U;
	zynq_pl310_init(0U);
	for (size_t i = 0U; i < operation_count; i++) {
		if (operations[i].kind == 'W') {
			zassert_true(next < ARRAY_SIZE(writes));
			zassert_equal(operations[i].address, writes[next].address);
			zassert_equal(operations[i].value, writes[next++].value);
		}
	}
	zassert_equal(next, ARRAY_SIZE(writes));
	zassert_equal(busy_reads, 0U);
	zassert_true(pl310_enabled);
	operation_count = 0U;
	zynq_pl310_init(0U);
	zassert_equal(operation_count, 0U);
}

ZTEST(pl310, test_secondary_never_initializes_shared_cache)
{
	pl310_enabled = false;
	zynq_pl310_init(1U);
	zassert_equal(operation_count, 0U);
	zassert_false(pl310_enabled);
}

ZTEST(pl310, test_enabled_bootloader_cache_is_cleaned_before_disable)
{
	pl310_enabled = false;
	zynq_pl310_init(0U);
	zassert_equal(operations[0].kind, 'C');
	zassert_equal(operations[1].kind, 'D');
	zassert_equal(operations[operation_count - 1U].kind, 'E');
	zassert_true(find_operation('W', 0xf8f027fcU, 0xffffU) <
		     find_operation('W', 0xf8f02100U, 0U));
}

ZTEST(pl310, test_clean_translates_every_page_and_reaches_outer_after_inner)
{
	zassert_ok(cache_data_flush_range((void *)0x10000ff0U, 64U));
	zassert_equal(operations[0].kind, 'C');
	zassert_equal(count_kind('T'), 2U);
	zassert_true(find_operation('T', 0x10000000U, 0U) <
		     find_operation('W', 0xf8f027b0U, 0x20000fe0U));
	zassert_true(find_operation('T', 0x10001000U, 0U) <
		     find_operation('W', 0xf8f027b0U, 0x30004000U));
	find_operation('W', 0xf8f027b0U, 0x30004020U);
	zassert_equal(count_kind('I'), 0U);
	zassert_equal(operations[operation_count - 1U].kind, 'B');
}

ZTEST(pl310, test_aligned_invalidate_is_outer_then_inner)
{
	zassert_ok(cache_data_invd_range((void *)0x10000020U, 64U));
	zassert_equal(count_kind('C'), 0U);
	find_operation('W', 0xf8f02770U, 0x20000020U);
	find_operation('W', 0xf8f02770U, 0x20000040U);
	zassert_equal(operations[operation_count - 2U].kind, 'I');
	zassert_equal(operations[operation_count - 1U].kind, 'B');
}

ZTEST(pl310, test_partial_invalidate_preserves_both_inner_edges_before_outer)
{
	zassert_ok(cache_data_invd_range((void *)0x10000ff0U, 64U));
	zassert_equal(operations[0].kind, 'C');
	zassert_equal(operations[0].address, 0x10000fe0U);
	zassert_equal(operations[1].kind, 'C');
	zassert_equal(operations[1].address, 0x10001020U);
	find_operation('W', 0xf8f027f0U, 0x20000fe0U);
	find_operation('W', 0xf8f02770U, 0x30004000U);
	find_operation('W', 0xf8f027f0U, 0x30004020U);
	zassert_equal(operations[operation_count - 2U].kind, 'I');
	zassert_equal(operations[operation_count - 1U].kind, 'B');
}

ZTEST(pl310, test_single_partial_line_is_cleaned_once)
{
	zassert_ok(cache_data_invd_range((void *)0x10000003U, 2U));
	zassert_equal(count_kind('C'), 1U);
	zassert_equal(count_kind('T'), 1U);
	find_operation('W', 0xf8f027f0U, 0x20000000U);
}

ZTEST(pl310, test_flush_invalidate_orders_three_levels)
{
	zassert_ok(cache_data_flush_and_invd_range((void *)0x10000020U, 32U));
	zassert_equal(operations[0].kind, 'C');
	find_operation('W', 0xf8f027f0U, 0x20000020U);
	zassert_equal(operations[operation_count - 2U].kind, 'I');
	zassert_equal(operations[operation_count - 1U].kind, 'B');
}

ZTEST(pl310, test_empty_overflow_and_translation_fault)
{
	zassert_ok(cache_data_invd_range((void *)0x10000003U, 0U));
	zassert_equal(cache_data_flush_range((void *)(UINTPTR_MAX - 15U), 32U), -EINVAL);
	zassert_equal(operation_count, 0U);
	bad_translation = true;
	zassert_equal(cache_data_invd_range((void *)0x10000000U, 32U), -EFAULT);
	zassert_equal(count_kind('I'), 0U);
	zassert_equal(count_kind('W'), 1U); /* Completion fence only, never a bogus PA. */
}

ZTEST(pl310, test_all_operations_and_local_disable)
{
	zassert_ok(cache_data_flush_all());
	zassert_equal(operations[0].kind, 'C');
	find_operation('W', 0xf8f027bcU, 0xffffU);
	operation_count = 0U;
	zassert_ok(cache_data_invd_all());
	zassert_equal(count_kind('C'), 0U);
	find_operation('W', 0xf8f0277cU, 0xffffU);
	zassert_equal(operations[operation_count - 2U].kind, 'I');
	zassert_equal(operations[operation_count - 1U].kind, 'B');
	operation_count = 0U;
	zassert_ok(cache_data_flush_and_invd_all());
	zassert_equal(operations[0].kind, 'F');
	find_operation('W', 0xf8f027fcU, 0xffffU);
	zassert_equal(count_kind('I'), 0U);
	zassert_equal(operations[operation_count - 1U].kind, 'B');
	operation_count = 0U;
	cache_data_disable();
	zassert_equal(operations[0].kind, 'C');
	zassert_equal(operations[1].kind, 'D');
	find_operation('W', 0xf8f027bcU, 0xffffU);
	zassert_equal(control, 1U);
	operation_count = 0U;
	cache_data_enable();
	zassert_equal(operation_count, 1U);
	zassert_equal(operations[0].kind, 'E');
}

ZTEST(pl310, test_flush_invalidate_all_preserves_live_stack)
{
	model_stack = true;
	stack_l1 = 0x12345678U;
	stack_memory = 0U;
	zassert_ok(cache_data_flush_and_invd_all());
	zassert_equal(stack_l1, 0xfeed1234U, "L1 invalidate discarded a callee's stack write");
}

static void check_reboot_sequence(void)
{
	const struct operation expected[] = {
		{'C', 0U, 0U},
		{'D', 0U, 0U},
		{'W', 0xf8f027fcU, 0xffffU},
		{'W', 0xf8f02730U, 0U},
		{'B', 0U, 0U},
		{'W', 0xf8f02100U, 0U},
		{'B', 0U, 0U},
		{'W', 0xf8000008U, 0xdf0dU},
		{'B', 0U, 0U},
		{'W', 0xf8000200U, 1U},
	};

	zassert_equal(operation_count, ARRAY_SIZE(expected));
	for (size_t i = 0U; i < ARRAY_SIZE(expected); i++) {
		zassert_equal(operations[i].kind, expected[i].kind, "Operation %zu", i);
		zassert_equal(operations[i].address, expected[i].address, "Address %zu", i);
		zassert_equal(operations[i].value, expected[i].value, "Value %zu", i);
	}
	zassert_equal(busy_reads, 0U);
	zassert_equal(control, 0U);
	zassert_true(shutdown_lock_held);
}

ZTEST(pl310, test_reboot_cleans_and_disables_before_reset)
{
	model_shutdown = true;
	test_sys_arch_reboot(SYS_REBOOT_COLD);
	zassert_unreachable("Reset hook returned without issuing the reset write");
}

ZTEST_SUITE(pl310, NULL, NULL, before, NULL, NULL);
