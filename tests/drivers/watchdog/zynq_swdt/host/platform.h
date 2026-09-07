/*
 * Copyright (c) 2026 pan
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef TEST_PLATFORM_H
#define TEST_PLATFORM_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <assert.h>
#define BIT(n)             (1U << (n))
#define ARRAY_SIZE(a)      (sizeof(a) / sizeof((a)[0]))
#define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))
#define ARG_UNUSED(x)      (void)(x)
typedef uintptr_t mm_reg_t;
struct device {
	const void *config;
	void *data;
};
struct k_spinlock {
	int held;
};
typedef int k_spinlock_key_t;
static int k_spin_lock(struct k_spinlock *l)
{
	assert(!l->held);
	l->held = 1;
	return 0;
}
static void k_spin_unlock(struct k_spinlock *l, int key)
{
	(void)key;
	assert(l->held);
	l->held = 0;
}
struct wdt_timeout_cfg {
	struct {
		uint32_t min, max;
	} window;
	void (*callback)(const struct device *dev, int channel);
	uint8_t flags;
};
#define WDT_FLAG_RESET_SOC 2
struct wdt_api {
	int (*install_timeout)(const struct device *dev, const struct wdt_timeout_cfg *cfg);
	int (*setup)(const struct device *dev, uint8_t options);
	int (*feed)(const struct device *dev, int channel);
	int (*disable)(const struct device *dev);
};
#define DEVICE_API(type, name) const struct wdt_api name
#define DEVICE_MMIO_ROM        uintptr_t address
#define DEVICE_MMIO_RAM        uintptr_t address
#define DEVICE_MMIO_GET(dev)   (((struct zynq_wdt_data *)(dev)->data)->address)
#define DEVICE_MMIO_MAP(dev, flags)                                                                \
	(DEVICE_MMIO_GET(dev) = ((const struct zynq_wdt_config *)(dev)->config)->address)
#define DT_INST_FOREACH_STATUS_OKAY(fn)
#define K_MEM_CACHE_NONE 0
static unsigned int writes;
static uint32_t values[16];
static uintptr_t addresses[16];
static void sys_write32(uint32_t value, uintptr_t address)
{
	assert(writes < 16);
	values[writes] = value;
	addresses[writes++] = address;
}
#endif
