/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/ztest.h>

extern int z_arm_mmu_init_secondary(void);

/* Small descriptor format, level 2 small page: bit [0] is execute never. */
#define L2_XN_BIT	     0x00000001U
#define L1_TYPE_MASK	     0x00000003U
#define L1_TYPE_L2_PT	     0x00000001U
#define L1_L2_PT_ADDR_MASK   0xFFFFFC00U
#define L1_TABLE_ADDR_MASK   0xFFFFC000U

static struct k_thread worker;
static K_THREAD_STACK_DEFINE(worker_stack, 2048);
static atomic_t running;
static atomic_t samples;
static atomic_t deviations;
static atomic_t first_deviation;
static uint32_t steady_entry;
static volatile uint32_t *text_pte;

/* A function whose page is the one being watched. */
__attribute__((noinline)) static uint32_t step(uint32_t value)
{
	return value + 1U;
}

/*
 * The level 2 descriptor this CPU translates the given address through, found
 * the way the MMU finds it: the level 1 table is where TTBR0 says it is.
 */
static volatile uint32_t *descriptor_of(uintptr_t va)
{
	uint32_t ttbr0;
	const volatile uint32_t *l1;
	uint32_t entry;

	__asm__ volatile("mrc p15, 0, %0, c2, c0, 0" : "=r"(ttbr0));
	l1 = (const volatile uint32_t *)(ttbr0 & L1_TABLE_ADDR_MASK);
	entry = l1[va >> 20];

	if ((entry & L1_TYPE_MASK) != L1_TYPE_L2_PT) {
		/* A 1 MB section: this build maps its text in 4 kB pages. */
		return NULL;
	}

	return &((volatile uint32_t *)(entry & L1_L2_PT_ADDR_MASK))[(va >> 12) & 0xFFU];
}

static void worker_entry(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	zassert_equal(arch_curr_cpu()->id, 1U, "the worker is not on the second CPU");

	while (atomic_get(&running) != 0) {
		uint32_t entry = *text_pte;

		atomic_inc(&samples);
		if (entry != steady_entry) {
			if (atomic_inc(&deviations) == 0) {
				atomic_set(&first_deviation, (atomic_val_t)entry);
			}
		}
	}
}

/*
 * A secondary CPU points itself at the page tables the primary CPU built; it
 * must not build them again. Rebuilding them walks every range of
 * mmu_zephyr_ranges in order, and the first of those covers the whole image
 * read/write and execute never - _image_ram_start to _image_ram_end spans the
 * text - so every text page is momentarily marked execute never before the
 * text range narrows it back down. A CPU that fetches an instruction from
 * that page in the meantime takes a prefetch abort, which is how
 * panxxhub/zephyr#69 showed up on qemu_cortex_a9.
 *
 * Watch the descriptor of a text page from the other CPU while the secondary
 * MMU initialisation runs, and require it never to change. Calling
 * z_arm_mmu_init() here instead is the reverse control: the watcher sees the
 * descriptor lose its executable permission.
 */
ZTEST(arm_mmu_smp, test_secondary_init_leaves_the_tables_alone)
{
	uint32_t seen;

	zassert_equal(arch_num_cpus(), 2, "this test needs both CPUs");
	zassert_equal(arch_curr_cpu()->id, 0U, "the test is not on the first CPU");

	text_pte = descriptor_of((uintptr_t)step);
	zassert_not_null(text_pte, "the text is not mapped in 4 kB pages");
	steady_entry = *text_pte;
	zassert_equal(steady_entry & L2_XN_BIT, 0U,
		      "the text page is already execute never: 0x%08x", steady_entry);

	atomic_set(&running, 1);
	atomic_set(&samples, 0);
	atomic_set(&deviations, 0);
	k_thread_create(&worker, worker_stack, K_THREAD_STACK_SIZEOF(worker_stack), worker_entry,
			NULL, NULL, NULL, K_PRIO_COOP(1), 0, K_FOREVER);
	zassert_ok(k_thread_cpu_pin(&worker, 1), "the worker could not be pinned to CPU 1");
	k_thread_start(&worker);

	for (int i = 0; i < 100 && atomic_get(&samples) == 0; i++) {
		k_msleep(10);
	}
	zassert_true(atomic_get(&samples) > 0, "the watcher never ran on the second CPU");

	for (int i = 0; i < 64; i++) {
		zassert_ok(z_arm_mmu_init_secondary(), "secondary MMU init failed");
	}

	atomic_set(&running, 0);
	zassert_ok(k_thread_join(&worker, K_SECONDS(5)), "the watcher did not finish");

	seen = (uint32_t)atomic_get(&first_deviation);
	zassert_equal(atomic_get(&deviations), 0,
		      "the descriptor of a text page changed %ld times while the secondary "
		      "MMU initialisation ran: 0x%08x became 0x%08x",
		      (long)atomic_get(&deviations), steady_entry, seen);
	zassert_equal(*text_pte, steady_entry, "the descriptor did not end up where it started");
}

ZTEST_SUITE(arm_mmu_smp, NULL, NULL, NULL, NULL, NULL);
