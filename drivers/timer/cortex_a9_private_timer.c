/*
 * Copyright (c) 2026 Moton Intelligent Technology
 * SPDX-License-Identifier: Apache-2.0
 *
 * Cortex-A9 MPCore Private Timer driver for Zephyr.
 *
 * Interrupt generation : per-CPU 32-bit Private Timer (no cross-CPU
 *                        contention on compare/reload registers).
 * Cycle counting       : 64-bit SCU Global Timer counter (read-only,
 *                        shared across CPUs — safe for concurrent reads).
 *
 * This combination solves the Global Timer compare-register contention
 * that causes k_sleep() 3–7× slowdown under SMP + tickless on Cortex-A9.
 */

#include <zephyr/init.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/irq.h>
#include <zephyr/sys_clock.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/device_mmio.h>

/* ------------------------------------------------------------------ */
/* DTS nodes                                                          */
/* ------------------------------------------------------------------ */

#define PT_NODE  DT_INST(0, arm_cortex_a9_twd_timer)

/*
 * The Global Timer node is the arm,armv8-timer already present in the
 * SoC DTSI.  We only read its counter registers — never touch the
 * compare registers that caused the original SMP contention bug.
 */
#if DT_HAS_COMPAT_STATUS_OKAY(arm_armv8_timer)
#define GT_NODE  DT_INST(0, arm_armv8_timer)
#elif DT_HAS_COMPAT_STATUS_OKAY(arm_armv7_timer)
#define GT_NODE  DT_INST(0, arm_armv7_timer)
#else
#error "Cortex-A9 Private Timer driver requires a Global Timer node for cycle counting"
#endif

/* ------------------------------------------------------------------ */
/* Private Timer registers (per-CPU, hardware-banked at same MMIO)    */
/* ------------------------------------------------------------------ */

#define PT_LOAD     0x00   /* Timer Load Register           */
#define PT_COUNTER  0x04   /* Timer Counter Register         */
#define PT_CONTROL  0x08   /* Timer Control Register         */
#define PT_ISR      0x0C   /* Timer Interrupt Status Register */

/* PT_CONTROL bits */
#define PT_CTRL_ENABLE      BIT(0)
#define PT_CTRL_AUTO_RELOAD BIT(1)
#define PT_CTRL_IRQ_ENABLE  BIT(2)
/* bits [15:8] = prescaler */

/* PT_ISR bits */
#define PT_ISR_EVENT_FLAG   BIT(0)

/* ------------------------------------------------------------------ */
/* Global Timer registers (read-only for cycle counting)              */
/* ------------------------------------------------------------------ */

#define GT_BASE  DT_REG_ADDR(GT_NODE)

#define GT_CNT_LOWER  0x00
#define GT_CNT_UPPER  0x04
#define GT_CTRL       0x08

/* GT_CTRL bit 0 is the global timer enable (shared across CPUs).
 * Bits 1-3 are per-CPU banked (compare, IRQ, auto-increment) — we
 * must NOT touch them when enabling the counter.
 */
#define GT_CTRL_TIMER_ENABLE  BIT(0)

/* ------------------------------------------------------------------ */
/* Private Timer PPI                                                  */
/*                                                                    */
/* The Cortex-A9 Private Timer uses PPI 29.  In the GIC PPI encoding  */
/* used by Zephyr DTS (GIC_PPI offset), PPI 29 = GIC_PPI 13.         */
/* ------------------------------------------------------------------ */

#define PT_IRQ       DT_IRQN(PT_NODE)
#define PT_IRQ_PRIO  DT_IRQ(PT_NODE, priority)
#define PT_IRQ_FLAGS DT_IRQ(PT_NODE, flags)

/* ------------------------------------------------------------------ */
/* MMIO mapping for Private Timer                                     */
/* ------------------------------------------------------------------ */

DEVICE_MMIO_TOPLEVEL_STATIC(pt_regs, PT_NODE);

#define PT_REG(off)  (DEVICE_MMIO_TOPLEVEL_GET(pt_regs) + (off))

/* ------------------------------------------------------------------ */
/* MMIO mapping for Global Timer (counter only)                       */
/* ------------------------------------------------------------------ */

DEVICE_MMIO_TOPLEVEL_STATIC(gt_regs, GT_NODE);

#define GT_REG(off)  (DEVICE_MMIO_TOPLEVEL_GET(gt_regs) + (off))

/* ------------------------------------------------------------------ */
/* Timing constants                                                   */
/* ------------------------------------------------------------------ */

#define CYC_PER_TICK  ((uint32_t)(sys_clock_hw_cycles_per_sec() \
			/ CONFIG_SYS_CLOCK_TICKS_PER_SEC))

/* Maximum cycles we can load into the 32-bit down-counter.
 * Use 3/4 of full range to leave headroom for ISR latency.
 */
#define PT_MAX_CYCLES  ((uint32_t)0xC0000000u)

/* Minimum delay to avoid setting a compare that's already in the past */
#define MIN_DELAY  (CYC_PER_TICK / 2)

/* ------------------------------------------------------------------ */
/* Driver state                                                       */
/* ------------------------------------------------------------------ */

static struct k_spinlock lock;
static uint64_t last_cycle;
static uint64_t last_tick;
static uint32_t last_elapsed;

#if defined(CONFIG_TEST)
const int32_t z_sys_timer_irq_for_test = PT_IRQ;
#endif

/* ------------------------------------------------------------------ */
/* Global Timer enable (counter only — no compare/IRQ bits)           */
/* ------------------------------------------------------------------ */

static void global_timer_enable(void)
{
	uint32_t ctrl = sys_read32(GT_REG(GT_CTRL));

	if (!(ctrl & GT_CTRL_TIMER_ENABLE)) {
		/*
		 * Only set the enable bit.  Preserve per-CPU banked bits
		 * (compare, IRQ, auto-increment) read as the calling
		 * CPU's state — writing them back unchanged is safe.
		 */
		ctrl |= GT_CTRL_TIMER_ENABLE;
		sys_write32(ctrl, GT_REG(GT_CTRL));
	}
}

/* ------------------------------------------------------------------ */
/* Global Timer 64-bit counter read (safe concurrent access)          */
/* ------------------------------------------------------------------ */

static ALWAYS_INLINE uint64_t global_timer_count(void)
{
	uint32_t lower, upper, upper_saved;

	upper = sys_read32(GT_REG(GT_CNT_UPPER));
	do {
		upper_saved = upper;
		lower = sys_read32(GT_REG(GT_CNT_LOWER));
		upper = sys_read32(GT_REG(GT_CNT_UPPER));
	} while (upper != upper_saved);

	return ((uint64_t)upper << 32) | lower;
}

/* ------------------------------------------------------------------ */
/* Private Timer helpers                                              */
/* ------------------------------------------------------------------ */

static ALWAYS_INLINE void pt_stop(void)
{
	sys_write32(0, PT_REG(PT_CONTROL));
}

static ALWAYS_INLINE void pt_clear_isr(void)
{
	sys_write32(PT_ISR_EVENT_FLAG, PT_REG(PT_ISR));
}

/*
 * Load the Private Timer with a one-shot countdown.
 * The timer starts immediately and fires an IRQ when it reaches zero.
 */
static ALWAYS_INLINE void pt_set_oneshot(uint32_t cycles)
{
	pt_stop();
	pt_clear_isr();
	sys_write32(cycles, PT_REG(PT_LOAD));
	sys_write32(PT_CTRL_ENABLE | PT_CTRL_IRQ_ENABLE,
		    PT_REG(PT_CONTROL));
}

/*
 * Load the Private Timer in auto-reload mode for periodic ticks.
 */
static ALWAYS_INLINE void pt_set_periodic(uint32_t cycles)
{
	pt_stop();
	pt_clear_isr();
	sys_write32(cycles, PT_REG(PT_LOAD));
	sys_write32(PT_CTRL_ENABLE | PT_CTRL_AUTO_RELOAD | PT_CTRL_IRQ_ENABLE,
		    PT_REG(PT_CONTROL));
}

/* ------------------------------------------------------------------ */
/* ISR                                                                */
/* ------------------------------------------------------------------ */

static void private_timer_isr(const void *arg)
{
	ARG_UNUSED(arg);

	k_spinlock_key_t key = k_spin_lock(&lock);

	pt_clear_isr();

	uint64_t now = global_timer_count();
	uint64_t delta = now - last_cycle;
	uint32_t dticks = (uint32_t)(delta / CYC_PER_TICK);

	last_cycle += (uint64_t)dticks * CYC_PER_TICK;
	last_tick += dticks;
	last_elapsed = 0;

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		/*
		 * Periodic mode: auto-reload keeps running, nothing to
		 * reprogram.  Just announce.
		 */
	} else {
		/*
		 * Tickless mode: stop the timer.  The kernel will call
		 * sys_clock_set_timeout() to schedule the next wakeup.
		 */
		pt_stop();
	}

	k_spin_unlock(&lock, key);

	sys_clock_announce(dticks);
}

/* ------------------------------------------------------------------ */
/* Kernel timer API                                                   */
/* ------------------------------------------------------------------ */

void sys_clock_set_timeout(int32_t ticks, bool idle)
{
	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return;
	}

	if (idle && ticks == K_TICKS_FOREVER) {
		pt_stop();
		return;
	}

	k_spinlock_key_t key = k_spin_lock(&lock);

	uint64_t now = global_timer_count();
	uint64_t elapsed = now - last_cycle;
	uint64_t target;

	if (ticks == K_TICKS_FOREVER) {
		target = (uint64_t)PT_MAX_CYCLES;
	} else {
		target = (uint64_t)(last_tick + last_elapsed + ticks)
			 * CYC_PER_TICK - last_cycle;
		if (target > PT_MAX_CYCLES) {
			target = PT_MAX_CYCLES;
		}
	}

	/* Ensure we don't set a delay that's already elapsed */
	if (target <= elapsed + MIN_DELAY) {
		target = elapsed + MIN_DELAY;
	}

	uint32_t delay = (uint32_t)(target - elapsed);

	pt_set_oneshot(delay);

	k_spin_unlock(&lock, key);
}

uint32_t sys_clock_elapsed(void)
{
	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return 0;
	}

	k_spinlock_key_t key = k_spin_lock(&lock);

	uint64_t now = global_timer_count();
	uint64_t delta = now - last_cycle;
	uint32_t dticks = (uint32_t)(delta / CYC_PER_TICK);

	last_elapsed = dticks;

	k_spin_unlock(&lock, key);
	return dticks;
}

uint32_t sys_clock_cycle_get_32(void)
{
	return (uint32_t)global_timer_count();
}

uint64_t sys_clock_cycle_get_64(void)
{
	return global_timer_count();
}

#ifdef CONFIG_ARCH_HAS_CUSTOM_BUSY_WAIT
void arch_busy_wait(uint32_t usec_to_wait)
{
	if (usec_to_wait == 0) {
		return;
	}

	uint64_t start = global_timer_count();
	uint64_t cycles_to_wait =
		(uint64_t)sys_clock_hw_cycles_per_sec() / USEC_PER_SEC
		* usec_to_wait;

	while ((global_timer_count() - start) < cycles_to_wait) {
	}
}
#endif

/* ------------------------------------------------------------------ */
/* SMP secondary CPU timer init                                       */
/* ------------------------------------------------------------------ */

#ifdef CONFIG_SMP
void smp_timer_init(void)
{
	/*
	 * Each CPU has its own Private Timer instance (hardware-banked).
	 * Map the same MMIO page — the hardware routes reads/writes to
	 * the calling CPU's registers.
	 */
	DEVICE_MMIO_TOPLEVEL_MAP(pt_regs, K_MEM_CACHE_NONE);

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		pt_set_periodic(CYC_PER_TICK);
	} else {
		pt_set_oneshot(CYC_PER_TICK);
	}

	irq_enable(PT_IRQ);
}
#endif

/* ------------------------------------------------------------------ */
/* Driver init (runs on CPU 0, PRE_KERNEL_2)                          */
/* ------------------------------------------------------------------ */

static int sys_clock_driver_init(void)
{
	DEVICE_MMIO_TOPLEVEL_MAP(pt_regs, K_MEM_CACHE_NONE);
	DEVICE_MMIO_TOPLEVEL_MAP(gt_regs, K_MEM_CACHE_NONE);

	/* Ensure the Global Timer counter is running.  The old
	 * ARM_ARCH_TIMER driver used to enable this; since we replaced
	 * it, we must do it ourselves.  Only the enable bit (bit 0) is
	 * touched — per-CPU banked bits are preserved.
	 */
	global_timer_enable();

	IRQ_CONNECT(PT_IRQ, PT_IRQ_PRIO, private_timer_isr, NULL,
		    PT_IRQ_FLAGS);

	last_tick = global_timer_count() / CYC_PER_TICK;
	last_cycle = last_tick * CYC_PER_TICK;

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		pt_set_periodic(CYC_PER_TICK);
	} else {
		pt_set_oneshot(CYC_PER_TICK);
	}

	irq_enable(PT_IRQ);

	return 0;
}

SYS_INIT(sys_clock_driver_init, PRE_KERNEL_2,
	 CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
