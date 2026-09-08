/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */
#define _POSIX_C_SOURCE 200809L
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <pthread.h>
#include <sched.h>
#include <stdlib.h>
#define BIT(n)                               (1U << (n))
#define K_MSEC(n)                            (n)
#define CONFIG_MDIO_XLNX_GEM_IDLE_TIMEOUT_US 1000
#define LOG_ERR_RATELIMIT_RATE(rate, ...) ((void)0)
#define LOG_ERR(...)                         ((void)0)
#define DEVICE_MMIO_GET(d)                   0
struct k_mutex {
	pthread_mutex_t native;
};
struct device {
	const char *name;
	void *data;
	const void *config;
};
typedef uintptr_t mm_reg_t;
enum mdio_opcode {
	MDIO_OP_C45_ADDRESS = 0,
	MDIO_OP_C22_WRITE = 1,
	MDIO_OP_C22_READ = 2,
	MDIO_OP_C45_WRITE = 1,
	MDIO_OP_C45_READ = 3
};
static _Thread_local uint32_t locked, ticks, sleeps, generation, address_generation;
static _Thread_local uint16_t expected;
static uint32_t command;
static bool stuck;
static int k_mutex_lock(struct k_mutex *lock, int timeout)
{
	assert(timeout == 10);
	int ret = pthread_mutex_lock(&lock->native);

	locked++;
	generation++;
	return ret;
}
static void k_mutex_unlock(struct k_mutex *lock)
{
	assert(locked > 0);
	locked--;
	assert(pthread_mutex_unlock(&lock->native) == 0);
}
static int64_t k_uptime_ticks(void)
{
	return ticks;
}
static int64_t k_us_to_ticks_ceil64(int us)
{
	return us;
}
static void k_usleep(int us)
{
	ticks += us;
	sleeps++;
	sched_yield();
}
static uint32_t sys_read32(uintptr_t addr)
{
	assert(locked == 1);
	sched_yield();
	if (addr == 8) {
		return stuck ? 0 : BIT(2);
	}
	assert(addr == 0x34);
	assert(((command >> 23) & 31) == expected);
	return expected;
}
static void sys_write32(uint32_t value, uintptr_t addr)
{
	assert(locked == 1 && addr == 0x34);
	command = value;
	if ((value & BIT(30)) == 0) {
		if (((value >> 28) & 3) == 0) {
			address_generation = generation;
		} else {
			assert(address_generation == generation);
		}
	}
	sched_yield();
}
/* FUNCTIONS */
static struct xlnx_gem_mdio_data runtime = {{PTHREAD_MUTEX_INITIALIZER}};
static struct device device = {"fake MDIO", &runtime, NULL};
static void *client(void *arg)
{
	expected = (uintptr_t)arg;
	for (uint32_t n = 0; n < 100; n++) {
		uint16_t value = 0;

		assert(xlnx_gem_mdio_read(&device, expected, 1, &value) == 0);
		assert(value == expected);
		assert(xlnx_gem_mdio_write(&device, expected, 1, 42) == 0);
		assert(xlnx_gem_mdio_read_c45(&device, expected, 3, 42, &value) == 0);
		assert(value == expected);
		assert(xlnx_gem_mdio_write_c45(&device, expected, 3, 42, 43) == 0);
	}
	return NULL;
}
int main(int argc, char **argv)
{
	pthread_t a, b;

	assert(pthread_create(&a, NULL, client, (void *)1) == 0);
	assert(pthread_create(&b, NULL, client, (void *)2) == 0);
	assert(pthread_join(a, NULL) == 0 && pthread_join(b, NULL) == 0);
	stuck = true;
	expected = 1;
	uint16_t value;

	assert(xlnx_gem_mdio_read(&device, 1, 1, &value) == -ETIMEDOUT);
	assert(ticks <= 1000 && sleeps > 0 && locked == 0);
	stuck = false;
	assert(xlnx_gem_mdio_read(&device, 1, 1, &value) == 0 && value == 1);
	return 0;
}
