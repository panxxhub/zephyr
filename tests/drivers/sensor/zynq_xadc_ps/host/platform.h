/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0 */
#ifndef TEST_PLATFORM_H
#define TEST_PLATFORM_H
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <assert.h>
#define BIT(n)        (1U << (n))
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
typedef uintptr_t mm_reg_t;
struct device {
	const void *config;
	void *data;
};
struct sensor_value {
	int32_t val1, val2;
};
enum sensor_channel {
	SENSOR_CHAN_DIE_TEMP,
	SENSOR_CHAN_ALL,
	SENSOR_CHAN_PRIV_START = 100
};
struct sensor_driver_api {
	int (*sample_fetch)(const struct device *, enum sensor_channel);
	int (*channel_get)(const struct device *, enum sensor_channel, struct sensor_value *);
};
struct k_mutex {
	bool held;
};
#define K_FOREVER 0
static void k_mutex_init(struct k_mutex *m)
{
	m->held = false;
}
static int k_mutex_lock(struct k_mutex *m, int t)
{
	(void)t;
	assert(!m->held);
	m->held = true;
	return 0;
}
static int k_mutex_unlock(struct k_mutex *m)
{
	assert(m->held);
	m->held = false;
	return 0;
}
static unsigned int waits;
static void k_busy_wait(unsigned int us)
{
	waits += us;
}
#define DEVICE_MMIO_ROM      uintptr_t address
#define DEVICE_MMIO_RAM      uintptr_t address
#define DEVICE_MMIO_GET(dev) (((struct zynq_xadc_data *)(dev)->data)->address)
#define DEVICE_MMIO_MAP(dev, flags)                                                                \
	(DEVICE_MMIO_GET(dev) = ((const struct zynq_xadc_config *)(dev)->config)->address)
#define K_MEM_CACHE_NONE       0
#define DEVICE_API(type, name) const struct sensor_driver_api name
#define DT_INST_FOREACH_STATUS_OKAY(fn)
static uint32_t sys_read32(uintptr_t address);
static void sys_write32(uint32_t value, uintptr_t address);
#endif
