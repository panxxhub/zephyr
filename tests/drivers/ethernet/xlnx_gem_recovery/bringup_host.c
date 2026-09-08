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
#define BIT(n)                               (1U << (n))
#define GENMASK(h, l)                        (((~0U) << (l)) & ((~0U) >> (31 - (h))))
#define FIELD_PREP(mask, value)              (((value) << __builtin_ctz(mask)) & (mask))
#define K_MSEC(n)                            (n)
#define K_NO_WAIT                            0
#define K_FOREVER                            -1
#define CONTAINER_OF(p, t, m)                ((t *)((char *)(p) - offsetof(t, m)))
#define LOG_ERR_RATELIMIT_RATE(rate, ...) ((void)0)
#define LOG_ERR(...)                         ((void)0)
#define LOG_DBG(...)                         ((void)0)
#define LOG_INF(...)                         ((void)0)
#define LOG_WRN(...)                         ((void)0)
#define LOG_ERR_DEVICE_NOT_READY(...)        ((void)0)
/* H2+ c791b426 board defconfig: MMU, SMP, 10 kHz ticks, CPU0 cooperative work. */
#define CONFIG_MMU                           1
#define CONFIG_SMP                           1
#define CONFIG_SYS_CLOCK_TICKS_PER_SEC       10000
#define CONFIG_PHY_MONITOR_PERIOD            500
#define CONFIG_MDIO_XLNX_GEM_IDLE_TIMEOUT_US 1000
#define DEVICE_MMIO_IS_IN_RAM
#define ETH_XLNX_GEM_MCAST_FILTER_SLOTS 64
struct device {
	const void *config;
	void *data;
	const char *name;
};
typedef uintptr_t mm_reg_t;
struct k_mutex {
	uint32_t depth;
};
struct k_spinlock {
	uint32_t locked;
};
struct k_sem {
	uint32_t count;
};
struct k_work {
	void (*handler)(struct k_work *work);
};
struct k_work_delayable {
	struct k_work work;
};
struct eth_xlnx_gem_mcast_filter {
	uint32_t unused;
};
struct eth_xlnx_gem_bd_ring {
	uint32_t unused;
};
struct net_if {
	uint32_t unused;
};
enum phy_link_speed {
	LINK_HALF_10BASE = BIT(0),
	LINK_FULL_10BASE = BIT(1),
	LINK_HALF_100BASE = BIT(2),
	LINK_FULL_100BASE = BIT(3),
	LINK_HALF_1000BASE = BIT(4),
	LINK_FULL_1000BASE = BIT(5)
};
enum phy_cfg_link_flag {
	PHY_FLAG_AUTO_NEGOTIATION_DISABLED = BIT(0)
};
struct phy_link_state {
	bool is_up;
	enum phy_link_speed speed;
};
typedef void (*phy_callback_t)(const struct device *dev, struct phy_link_state *state,
			       void *user_data);
enum mdio_opcode {
	MDIO_OP_C45_ADDRESS = 0,
	MDIO_OP_C22_WRITE = 1,
	MDIO_OP_C22_READ = 2,
	MDIO_OP_C45_WRITE = 1,
	MDIO_OP_C45_READ = 3
};
#define MDC_DIVIDER_224 7
/* DECLARATIONS: production MMIO macros, parent layout, MDIO and PHY structs/constants. */
static struct eth_xlnx_gem_dev_data mac_data;
static struct xlnx_gem_mdio_data mdio_data;
static struct mc_yt8531_data phy_data;
static const struct device mac_dev = {NULL, &mac_data, "GEM0"};
static const struct device mdio_dev = {&mac_dev, &mdio_data, "GEM0 MDIO"};
/* Exact management PHY properties from c791b426 opus_one_ctrl_gen2.dts. */
static const struct mc_yt8531_config phy_config = {
	.phy_addr = 0,
	.mdio = &mdio_dev,
	.rx_delay_sel = 1,
	.tx_delay_sel = 13,
	.rxc_dly_en = false,
	.default_speeds = LINK_FULL_1000BASE | LINK_FULL_100BASE,
};
static const struct device phy_dev = {&phy_config, &phy_data, "YT8531"};
static uint32_t regs[1024], phy_regs[32], extended[65536], page;
static uint64_t now_us, command_end, reset_end, reset_delay_us;
static uint32_t reset_reads, bad_addresses, callbacks, sleeps;
static bool reset_started, ready, carrier;
static struct phy_link_state observed;
static int k_mutex_init(struct k_mutex *m)
{
	m->depth = 0;
	return 0;
}
static int k_mutex_lock(struct k_mutex *m, int timeout)
{
	m->depth++;
	return 0;
}
static void k_mutex_unlock(struct k_mutex *m)
{
	assert(m->depth > 0);
	m->depth--;
}
static int64_t k_uptime_ticks(void)
{
	return now_us / 100;
}
static int64_t k_uptime_get(void)
{
	return now_us / 1000;
}
static int64_t k_us_to_ticks_ceil64(int us)
{
	return (us + 99) / 100;
}
static void k_usleep(int us)
{
	now_us += ((us + 99) / 100 + 1) * 100;
	sleeps++;
}
static void k_msleep(int ms)
{
	k_usleep(ms * 1000);
}
static bool device_is_ready(const struct device *d)
{
	return true;
}
static void k_work_init_delayable(struct k_work_delayable *w, void (*h)(struct k_work *work))
{
	w->work.handler = h;
}
static int k_work_schedule(struct k_work_delayable *w, int delay)
{
	return 0;
}
static int k_work_reschedule(struct k_work_delayable *w, int delay)
{
	return 0;
}
static struct k_work_delayable *k_work_delayable_from_work(struct k_work *w)
{
	return CONTAINER_OF(w, struct k_work_delayable, work);
}
static uint32_t phy_reg_read(uint32_t reg)
{
	if (reg == MII_BMCR && reset_started) {
		reset_reads++;
		if (now_us >= reset_end) {
			phy_regs[reg] &= ~MII_BMCR_RESET;
		}
	}
	return reg == YTPHY_PAGE_DATA ? extended[page] : phy_regs[reg];
}
static void phy_reg_write(uint32_t reg, uint16_t value)
{
	if (reg == YTPHY_PAGE_SELECT) {
		page = value;
	}
	if (reg == YTPHY_PAGE_DATA) {
		extended[page] = value;
	} else {
		phy_regs[reg] = value;
	}
	if (reg == MII_BMCR && (value & MII_BMCR_RESET) != 0) {
		reset_started = true;
		reset_end = now_us + reset_delay_us;
	}
}
static uint32_t sys_read32(uintptr_t addr)
{
	if (addr < 0xe000b000U || addr >= 0xe000c000U) {
		bad_addresses++;
		return 0;
	}
	uint32_t offset = addr - 0xe000b000U;

	if (offset == 8) {
		return now_us >= command_end ? BIT(2) : 0;
	}
	return regs[offset / 4];
}
static void sys_write32(uint32_t value, uintptr_t addr)
{
	if (addr < 0xe000b000U || addr >= 0xe000c000U) {
		bad_addresses++;
		return;
	}
	uint32_t offset = addr - 0xe000b000U;

	regs[offset / 4] = value;
	if (offset == 0x34) {
		assert(mdio_data.lock.depth == 1 && phy_data.lock.depth > 0);
		assert(regs[0] & BIT(4));
		assert(((value >> 23) & 31) == 0);
		uint32_t reg = (value >> 18) & 31;

		if (((value >> 28) & 3) == MDIO_OP_C22_READ) {
			regs[offset / 4] = phy_reg_read(reg);
		} else {
			phy_reg_write(reg, value);
		}
		/* 64 MDC cycles at 133.333 MHz / 224, rounded up. */
		command_end = now_us + 108;
	}
}
/* MDIO FUNCTIONS */
#define mdio_read  xlnx_gem_mdio_read
#define mdio_write xlnx_gem_mdio_write
static int mc_yt8531_read(const struct device *dev, uint16_t reg, uint32_t *value);
static int mc_yt8531_write(const struct device *dev, uint16_t reg, uint32_t value);
#define phy_read                   mc_yt8531_read
#define phy_write                  mc_yt8531_write
#define WRITE_BIT(var, bit, value) ((var) = ((var) & ~BIT(bit)) | ((value) ? BIT(bit) : 0))
/* AUTONEG FUNCTIONS */
static int phy_mii_set_bmcr_reg_autoneg_disabled(const struct device *d, enum phy_link_speed speed)
{
	return 0;
}
/* PHY FUNCTIONS */
static void link_callback(const struct device *dev, struct phy_link_state *state, void *arg)
{
	observed = *state;
	carrier = state->is_up;
	callbacks++;
}
static void boot(uint64_t reset_delay)
{
	memset(&mac_data, 0, sizeof(mac_data));
	memset(&phy_data, 0, sizeof(phy_data));
	memset(regs, 0, sizeof(regs));
	memset(phy_regs, 0, sizeof(phy_regs));
	memset(extended, 0, sizeof(extended));
	bad_addresses = callbacks = reset_reads = sleeps = 0;
	now_us = command_end = 0;
	reset_started = carrier = false;
	reset_delay_us = reset_delay;
	/* MAC init maps the named regions before MDIO device init. */
	mac_data.mac = 0xe000b000U;
	mac_data.clkc = 0xf8000140U;
	phy_regs[MII_PHYID2R] = PHY_ID_YT8531;
	assert(xlnx_gem_mdio_initialize(&mdio_dev) == 0);
	ready = mc_yt8531_initialize_dynamic_link(&phy_dev) == 0;
}
int main(int argc, char **argv)
{
	/* Reset clearing is elapsed hardware time, not a fake read-count threshold. */
	uint64_t delays[] = {1000, 10000, 100000, 499000};

	for (uint32_t i = 0; i < 4; i++) {
		boot(delays[i]);
		assert(ready && bad_addresses == 0 && now_us < 30000000);
		assert(reset_reads > 0 && sleeps > 0);
		assert(phy_regs[MII_ANAR] == MII_ADVERTISE_100_FULL);
		assert(phy_regs[MII_1KTCR] == MII_ADVERTISE_1000_FULL);
		assert(extended[YT8521_CHIP_CONFIG_REG] & YT8521_CCR_RXC_DLY_DIS);
		assert((extended[YT8521_RGMII_CONFIG1_REG] & 0x3c0f) == ((1U << 10) | 13));
		assert(mc_yt8531_link_cb_set(&phy_dev, link_callback, NULL) == 0);
		assert(!carrier && callbacks == 1);
		phy_regs[YTPHY_SPECIFIC_STATUS_REG] =
			YTPHY_SSR_LINK | PHY_SPEED_1000M | PHY_DUPLEX_FULL;
		phy_data.monitor_work.work.handler(&phy_data.monitor_work.work);
		assert(carrier && observed.speed == LINK_FULL_1000BASE && callbacks == 2);
		assert(phy_data.lock.depth == 0 && mdio_data.lock.depth == 0);
	}
	boot(UINT64_MAX / 2);
	assert(!ready && reset_reads == 500 && bad_addresses == 0);
	return 0;
}
