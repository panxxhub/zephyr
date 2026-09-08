/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#define BIT(n)                    (1U << (n))
#define K_MSEC(n)                 (n)
#define K_FOREVER                 -1
#define K_NO_WAIT                 0
#define CONFIG_PHY_MONITOR_PERIOD 500
#define CONTAINER_OF(p, t, m)     ((t *)((char *)(p) - offsetof(t, m)))
struct device {
	void *data;
	const void *config;
};
enum phy_link_speed {
	SPEED_NONE = 0
};
struct phy_link_state {
	bool is_up;
	int speed;
};
typedef void (*phy_callback_t)(const struct device *, struct phy_link_state *, void *);
struct k_mutex {
	uint32_t depth;
};
struct k_work {
	int unused;
};
struct k_work_delayable {
	struct k_work work;
};
struct mc_yt8531_data {
	const struct device *dev;
	phy_callback_t cb;
	void *cb_data;
	struct phy_link_state state, notified;
	struct k_mutex lock;
	struct k_work_delayable monitor_work;
	bool notified_valid;
	uint32_t read_errors;
	int64_t last_read_ms;
};
static struct mc_yt8531_data data;
static struct device device = {&data, NULL};
static uint32_t reads, fail_at, calls, reset_clear_at, sleeps;
static uint32_t status, bmcr;
static struct phy_link_state observed;
#define MII_BMCR                    0
#define MII_BMCR_RESET              0x8000
#define MII_BMCR_POWER_DOWN         0x800
#define MII_BMCR_AUTONEG_ENABLE     0x1000
#define YTPHY_SPECIFIC_STATUS_REG   0x11
#define YTPHY_SSR_LINK              BIT(10)
#define SPEC_STATUS_REG_SPEED_MASK  (3U << 14)
#define SPEC_STATUS_REG_DUPLEX_MASK (1U << 13)
#define PHY_SPEED_10M               0
#define PHY_SPEED_100M              (1U << 14)
#define PHY_SPEED_1000M             (2U << 14)
#define PHY_DUPLEX_FULL             (1U << 13)
#define PHY_DUPLEX_HALF             0
#define LINK_FULL_10BASE            1
#define LINK_HALF_10BASE            2
#define LINK_FULL_100BASE           4
#define LINK_HALF_100BASE           8
#define LINK_FULL_1000BASE          16
#define LINK_HALF_1000BASE          32
static int k_mutex_lock(struct k_mutex *lock, int timeout)
{
	lock->depth++;
	return 0;
}
static void k_mutex_unlock(struct k_mutex *lock)
{
	assert(lock->depth > 0);
	lock->depth--;
}
static int64_t k_uptime_get(void)
{
	return 123;
}
static void k_msleep(int ms)
{
	sleeps++;
}
static int k_work_reschedule(struct k_work_delayable *work, int timeout)
{
	return 0;
}
static struct k_work_delayable *k_work_delayable_from_work(struct k_work *work)
{
	return CONTAINER_OF(work, struct k_work_delayable, work);
}
static int mc_yt8531_read(const struct device *dev, uint16_t reg, uint32_t *value)
{
	reads++;
	if (reads == fail_at) {
		return -EIO;
	}
	*value = reg == MII_BMCR ? bmcr : status;
	if (reg == MII_BMCR && reset_clear_at != 0 && reads >= reset_clear_at) {
		*value &= ~MII_BMCR_RESET;
	}
	return 0;
}
static int mc_yt8531_modify(const struct device *dev, uint16_t r, uint16_t m, uint16_t set)
{
	bmcr |= set;
	return 0;
}
static int mc_yt8531_get_link_state(const struct device *dev, struct phy_link_state *state);
/* FUNCTIONS */
static void callback(const struct device *dev, struct phy_link_state *state, void *arg)
{
	assert(data.lock.depth != 0);
	assert(!state->is_up || state->speed != 0);
	observed = *state;
	calls++;
}
static void init(void)
{
	memset(&data, 0, sizeof(data));
	data.dev = &device;
	reads = fail_at = calls = reset_clear_at = sleeps = 0;
	status = YTPHY_SSR_LINK | PHY_SPEED_100M | PHY_DUPLEX_FULL;
	bmcr = 0;
}
static void monitor(void)
{
	monitor_work_handler(&data.monitor_work.work);
}
static void test_state(void)
{
	init();
	fail_at = 1;
	assert(mc_yt8531_link_cb_set(&device, callback, NULL) == 0);
	assert(calls == 0);
	monitor();
	assert(calls == 1 && observed.is_up && observed.speed == LINK_FULL_100BASE);
	monitor();
	assert(calls == 1);
	struct phy_link_state state;

	memset(&state, 0xa5, sizeof(state));
	fail_at = reads + 1;
	assert(mc_yt8531_get_link_state(&device, &state) == -EIO);
	assert(!state.is_up && state.speed == 0);
	status = YTPHY_SSR_LINK | PHY_SPEED_1000M | PHY_DUPLEX_FULL;
	bmcr = MII_BMCR_AUTONEG_ENABLE;
	fail_at = reads + 2;
	monitor();
	assert(calls == 1); /* second read failed: no partial publication */
	fail_at = 0;
	assert(mc_yt8531_get_link_state(&device, &state) == 0);
	assert(state.speed == LINK_FULL_1000BASE && data.notified.speed == LINK_FULL_100BASE);
	monitor();
	assert(calls == 2 && observed.speed == LINK_FULL_1000BASE);
	monitor();
	assert(calls == 2);
	status = YTPHY_SSR_LINK | SPEC_STATUS_REG_SPEED_MASK;
	monitor();
	assert(calls == 2);
	status = 0;
	monitor();
	assert(calls == 3 && !observed.is_up);
	monitor();
	assert(calls == 3);
	status = YTPHY_SSR_LINK | PHY_SPEED_100M | PHY_DUPLEX_FULL;
	monitor();
	assert(calls == 4 && observed.is_up);
	/* Successful snapshot must not be replaced by a failing second callback read. */
	init();
	fail_at = 3;
	assert(mc_yt8531_link_cb_set(&device, callback, NULL) == 0);
	assert(calls == 1 && reads == 2 && observed.is_up);
}
static void test_reset(void)
{
	init();
	assert(mc_yt8531_soft_reset(&device) == -ETIMEDOUT);
	assert(reads == 500 && sleeps == 500);
	init();
	reset_clear_at = 500;
	assert(mc_yt8531_soft_reset(&device) == 0 && reads == 500);
	init();
	fail_at = 1;
	assert(mc_yt8531_soft_reset(&device) == -EIO && reads == 1);
}
int main(int argc, char **argv)
{
	assert(argc == 2);
	if (atoi(argv[1]) == 6) {
		test_state();
	} else {
		test_reset();
	}
	return 0;
}
