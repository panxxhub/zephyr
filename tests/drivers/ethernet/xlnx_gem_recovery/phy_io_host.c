/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */
#define _POSIX_C_SOURCE 200809L
#include <assert.h>
#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include <stdint.h>
#include <stddef.h>
#define K_MSEC(n)         (n)
#define YTPHY_PAGE_SELECT 0x1e
#define YTPHY_PAGE_DATA   0x1f
struct k_mutex {
	pthread_mutex_t native;
};
struct device {
	const void *config;
	void *data;
};
struct mc_yt8531_config {
	const struct device *mdio;
	uint8_t phy_addr;
};
struct mc_yt8531_data {
	struct k_mutex lock;
	uint32_t read_errors;
};
static _Thread_local uint32_t depth, group, page_group;
static _Thread_local uint16_t expected_page;
static uint16_t page;
static int k_mutex_lock(struct k_mutex *lock, int timeout)
{
	int ret = pthread_mutex_lock(&lock->native);

	if (depth++ == 0) {
		group++;
	}
	return ret;
}
static void k_mutex_unlock(struct k_mutex *lock)
{
	assert(depth > 0);
	depth--;
	assert(pthread_mutex_unlock(&lock->native) == 0);
}
static int mdio_read(const struct device *dev, uint8_t phy, uint16_t reg, uint16_t *value)
{
	assert(depth > 0);
	if (reg == YTPHY_PAGE_DATA) {
		assert(group == page_group && page == expected_page);
		*value = page;
	} else {
		*value = 42;
	}
	sched_yield();
	return 0;
}
static int mdio_write(const struct device *dev, uint8_t phy, uint16_t reg, uint16_t value)
{
	assert(depth > 0);
	if (reg == YTPHY_PAGE_SELECT) {
		page = value;
		page_group = group;
	} else if (reg == YTPHY_PAGE_DATA) {
		assert(group == page_group && page == expected_page);
	}
	sched_yield();
	return 0;
}
/* FUNCTIONS */
static struct mc_yt8531_data data;
static struct mc_yt8531_config config;
static struct device device = {&config, &data};
static void *client(void *arg)
{
	expected_page = (uintptr_t)arg;
	for (uint32_t n = 0; n < 100; n++) {
		uint32_t value;

		assert(mc_yt8531_read_ext(&device, expected_page, &value) == 0);
		assert(value == expected_page);
		assert(mc_yt8531_write_ext(&device, expected_page, 42) == 0);
		assert(mc_yt8531_modify_ext(&device, expected_page, 0xffff, 43) == 0);
		assert(mc_yt8531_read(&device, 0, &value) == 0 && value == 42);
	}
	return NULL;
}
int main(void)
{
	pthread_mutexattr_t attr;
	pthread_t a, b;

	assert(pthread_mutexattr_init(&attr) == 0);
	assert(pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE) == 0);
	assert(pthread_mutex_init(&data.lock.native, &attr) == 0);
	assert(pthread_create(&a, NULL, client, (void *)1) == 0);
	assert(pthread_create(&b, NULL, client, (void *)2) == 0);
	assert(pthread_join(a, NULL) == 0 && pthread_join(b, NULL) == 0);
	return 0;
}
