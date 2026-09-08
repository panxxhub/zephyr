/*
 * SPDX-FileCopyrightText: Copyright 2025-2026 NXP
 *
 * Inspiration from phy_mii.c, which is:
 * Copyright (c) 2021 IP-Logix Inc.
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Dedicated Motorcomm YT8531 (RGMII, UTP-only) driver, split out of
 * phy_motorcomm_yt8521.c so each compat owns its own translation unit
 * and instance symbols, and so YT8531-specific configuration (SyncE
 * clock output, LED, delay tuning) has a home.
 */

#define DT_DRV_COMPAT motorcomm_yt8531

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(phy_motorcomm_yt8531, CONFIG_PHY_LOG_LEVEL);

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/net/phy.h>
#include <zephyr/net/mdio.h>
#include <zephyr/net/mii.h>
#include <string.h>

#include "phy_mii.h"

#define PHY_ID_YT8531 (0x0000E91A)
/* IEEE PHYID2 bits [3:0] are the silicon revision: YT8531SH-CA reads 0xE91A
 * (rev A) while YT8531C-CA reads 0xE91B (rev B, B004 HIL 2026-08-31). Match
 * the model, not the revision.
 */
#define PHY_ID_YT8531_REV_MASK (0x0000FFF0)

/* Allow a slow PHY clock/power domain to settle after reset. */
#define YTPHY_PHY_ID_MAX_ATTEMPTS   20U
#define YTPHY_PHY_ID_RETRY_DELAY_MS 100

/* PHY Specific Status Register */
#define SPEC_STATUS_REG_DUPLEX_MASK (1U << 13)
#define PHY_DUPLEX_HALF             (0U << 13)
#define PHY_DUPLEX_FULL             (1U << 13)

#define SPEC_STATUS_REG_SPEED_MASK (0x3U << 14)
#define PHY_SPEED_10M              (0U << 14)
#define PHY_SPEED_100M             (1U << 14)
#define PHY_SPEED_1000M            (2U << 14)

/* Specific Status Register */
#define YTPHY_SPECIFIC_STATUS_REG 0x11
#define YTPHY_SSR_LINK            BIT(10)

/* Extended Register's Address Offset Register */
#define YTPHY_PAGE_SELECT 0x1E
/* Extended Register's Data Register */
#define YTPHY_PAGE_DATA   0x1F

#define YT8521_REG_SPACE_SELECT_REG 0xA000

#define YT8521_CHIP_CONFIG_REG 0xA001
/* 0xA001 bit 8 is ACTIVE-LOW: 0 = coarse ~1.9 ns RXC delay enabled (chip
 * default), 1 = delay bypassed. Mainline motorcomm.c documents it as
 * "1b0 enable 1.9ns rxc clock delay *default*"; B004 A/B on the ESC ports
 * (2026-09-02) confirmed the polarity on silicon.
 */
#define YT8521_CCR_RXC_DLY_DIS BIT(8)

#define YT8521_EXTREG_SLEEP_CONTROL1_REG 0x27
#define YT8521_ESC1R_SLEEP_SW            BIT(15)

#define YT8521_RSSR_UTP_SPACE (0x0 << 1)

#define YT8521_RGMII_CONFIG1_REG  0xA003
#define YT8521_RC1R_RX_DELAY_MASK GENMASK(13, 10)
#define YT8521_RC1R_TX_DELAY_MASK GENMASK(3, 0)

#define YTPHY_WOL_CONFIG_REG 0xA00A
#define YTPHY_WCR_ENABLE     BIT(3)

#define YTPHY_SYNCE_CFG_REG     0xA012
#define YT8521_SCR_SYNCE_ENABLE BIT(5)

static int mc_yt8531_get_link_state(const struct device *dev, struct phy_link_state *state);

struct mc_yt8531_config {
	uint8_t phy_addr;
	const struct device *mdio;
	uint8_t rx_delay_sel;
	uint8_t tx_delay_sel;
	bool rxc_dly_en;
	enum phy_link_speed default_speeds;
};

struct mc_yt8531_data {
	const struct device *dev;
	phy_callback_t cb;
	void *cb_data;
	struct phy_link_state state;
	struct k_mutex lock;
	struct phy_link_state notified;
	bool notified_valid;
	uint32_t read_errors;
	int64_t last_read_ms;
	struct k_work_delayable monitor_work;
};

/* How often to poll auto-negotiation status while waiting for it to complete */
#define MII_AUTONEG_POLL_INTERVAL_MS 100

static int mc_yt8531_read(const struct device *dev, uint16_t reg, uint32_t *value)
{
	const struct mc_yt8531_config *config = dev->config;
	struct mc_yt8531_data *data = dev->data;
	uint16_t result = 0;
	int ret;

	*value = 0U;
	ret = k_mutex_lock(&data->lock, K_MSEC(10));
	if (ret != 0) {
		return ret;
	}
	ret = mdio_read(config->mdio, config->phy_addr, reg, &result);
	if (ret == 0) {
		*value = result;
	} else {
		data->read_errors++;
	}
	k_mutex_unlock(&data->lock);
	return ret;
}

static int mc_yt8531_write(const struct device *dev, uint16_t reg, uint32_t value)
{
	const struct mc_yt8531_config *config = dev->config;
	struct mc_yt8531_data *data = dev->data;
	int ret = k_mutex_lock(&data->lock, K_MSEC(10));

	if (ret != 0) {
		return ret;
	}
	ret = mdio_write(config->mdio, config->phy_addr, reg, (uint16_t)value);
	k_mutex_unlock(&data->lock);
	return ret;
}

static int mc_yt8531_modify(const struct device *dev, uint16_t reg, uint16_t mask, uint16_t set)
{
	uint32_t data = 0;
	uint32_t new = 0;
	int ret;

	ret = mc_yt8531_read(dev, reg, &data);
	if (ret) {
		return ret;
	}

	new = (data & ~mask) | set;
	if (new == data) {
		return 0;
	}

	return mc_yt8531_write(dev, reg, new);
}

static int mc_yt8531_read_ext(const struct device *dev, uint16_t reg, uint32_t *data)
{
	struct mc_yt8531_data *runtime = dev->data;
	int ret = k_mutex_lock(&runtime->lock, K_MSEC(10));

	if (ret != 0) {
		return ret;
	}

	ret = mc_yt8531_write(dev, YTPHY_PAGE_SELECT, reg);
	if (ret) {
		k_mutex_unlock(&runtime->lock);
		return ret;
	}

	ret = mc_yt8531_read(dev, YTPHY_PAGE_DATA, data);
	k_mutex_unlock(&runtime->lock);
	return ret;
}

static int mc_yt8531_write_ext(const struct device *dev, uint16_t reg, uint32_t data)
{
	struct mc_yt8531_data *runtime = dev->data;
	int ret = k_mutex_lock(&runtime->lock, K_MSEC(10));

	if (ret != 0) {
		return ret;
	}

	ret = mc_yt8531_write(dev, YTPHY_PAGE_SELECT, reg);
	if (ret) {
		k_mutex_unlock(&runtime->lock);
		return ret;
	}

	ret = mc_yt8531_write(dev, YTPHY_PAGE_DATA, data);
	k_mutex_unlock(&runtime->lock);
	return ret;
}

static int mc_yt8531_modify_ext(const struct device *dev, uint16_t reg, uint16_t mask, uint16_t set)
{
	struct mc_yt8531_data *runtime = dev->data;
	int ret = k_mutex_lock(&runtime->lock, K_MSEC(10));

	if (ret != 0) {
		return ret;
	}

	ret = mc_yt8531_write(dev, YTPHY_PAGE_SELECT, reg);
	if (ret) {
		k_mutex_unlock(&runtime->lock);
		return ret;
	}

	ret = mc_yt8531_modify(dev, YTPHY_PAGE_DATA, mask, set);
	k_mutex_unlock(&runtime->lock);
	return ret;
}

static int mc_yt8531_soft_reset(const struct device *dev)
{
	int max_cnt = 500; /* max time of reset ~500ms */
	uint32_t data;
	int ret;

	ret = mc_yt8531_modify(dev, MII_BMCR, 0, MII_BMCR_RESET);
	if (ret) {
		return ret;
	}

	while (max_cnt--) {
		k_msleep(1);

		ret = mc_yt8531_read(dev, MII_BMCR, &data);
		if (ret) {
			return ret;
		}

		if (!(data & MII_BMCR_RESET)) {
			return 0;
		}
	}

	return -ETIMEDOUT;
}

static int mc_yt8531_cfg_clock_delay(const struct device *dev)
{
	const struct mc_yt8531_config *const cfg = dev->config;
	uint16_t mask, val = 0;
	int ret;

	/* rxc-dly-en in DT means "the coarse ~1.9 ns RXC delay is wanted":
	 * clear the active-low disable bit. Without the flag the delay is
	 * bypassed — the Zynq PS GEM path already carries ~2 ns of its own
	 * (B004 HIL 2026-08-31: adding the PHY delay there broke every frame,
	 * which the old inverted macro name mis-recorded as the opposite).
	 */
	ret = mc_yt8531_modify_ext(dev, YT8521_CHIP_CONFIG_REG, YT8521_CCR_RXC_DLY_DIS,
				   cfg->rxc_dly_en ? 0 : YT8521_CCR_RXC_DLY_DIS);
	if (ret) {
		return ret;
	}

	mask = YT8521_RC1R_RX_DELAY_MASK | YT8521_RC1R_TX_DELAY_MASK;

	val |= FIELD_PREP(YT8521_RC1R_RX_DELAY_MASK, cfg->rx_delay_sel);
	val |= FIELD_PREP(YT8521_RC1R_TX_DELAY_MASK, cfg->tx_delay_sel);

	return mc_yt8531_modify_ext(dev, YT8521_RGMII_CONFIG1_REG, mask, val);
}

static int mc_yt8531_resume(const struct device *dev)
{
	uint32_t wol_config;
	int ret;

	/* disable auto sleep */
	ret = mc_yt8531_modify_ext(dev, YT8521_EXTREG_SLEEP_CONTROL1_REG, YT8521_ESC1R_SLEEP_SW, 0);
	if (ret) {
		return ret;
	}

	ret = mc_yt8531_read_ext(dev, YTPHY_WOL_CONFIG_REG, &wol_config);
	if (ret) {
		return ret;
	}

	/* if wol enable, do nothing */
	if (wol_config & YTPHY_WCR_ENABLE) {
		return 0;
	}

	return mc_yt8531_modify(dev, MII_BMCR, MII_BMCR_POWER_DOWN, 0);
}

static void invoke_link_cb(const struct device *dev)
{
	struct mc_yt8531_data *data = dev->data;
	struct phy_link_state state = data->state;

	/* Caller holds the recursive PHY lock. Never perform a second MDIO read. */
	if (data->cb != NULL && (!data->notified_valid ||
	    state.is_up != data->notified.is_up || state.speed != data->notified.speed)) {
		data->notified = state;
		data->notified_valid = true;
		data->cb(dev, &state, data->cb_data);
	}
}

static inline enum phy_link_speed mc_yt8531_get_link_speed_stat_reg(const struct device *dev,
								   uint16_t stat_reg)
{
	enum phy_link_speed speed;

	switch (stat_reg & (SPEC_STATUS_REG_SPEED_MASK | SPEC_STATUS_REG_DUPLEX_MASK)) {
	case PHY_SPEED_10M | PHY_DUPLEX_FULL:
		speed = LINK_FULL_10BASE;
		break;
	case PHY_SPEED_10M | PHY_DUPLEX_HALF:
		speed = LINK_HALF_10BASE;
		break;
	case PHY_SPEED_100M | PHY_DUPLEX_FULL:
		speed = LINK_FULL_100BASE;
		break;
	case PHY_SPEED_100M | PHY_DUPLEX_HALF:
		speed = LINK_HALF_100BASE;
		break;
	case PHY_SPEED_1000M | PHY_DUPLEX_FULL:
		speed = LINK_FULL_1000BASE;
		break;
	case PHY_SPEED_1000M | PHY_DUPLEX_HALF:
		speed = LINK_HALF_1000BASE;
		break;
	default:
		speed = 0;
		break;
	}

	return speed;
}

static int mc_yt8531_read_live_link_state(const struct device *dev, struct phy_link_state *state)
{
	struct mc_yt8531_data *data = dev->data;
	uint32_t status;
	int ret;

	*state = (struct phy_link_state){0};
	ret = mc_yt8531_read(dev, YTPHY_SPECIFIC_STATUS_REG, &status);
	if (ret != 0) {
		return ret;
	}
	if ((status & YTPHY_SSR_LINK) != 0U) {
		state->speed = mc_yt8531_get_link_speed_stat_reg(dev, status);
		if (state->speed == 0) {
			return -EIO;
		}
		state->is_up = true;
	}
	data->last_read_ms = k_uptime_get();
	return 0;
}

static int update_link_state(const struct device *dev)
{
	struct mc_yt8531_data *data = dev->data;
	struct phy_link_state state = {0};
	uint32_t bmcr;
	int ret = mc_yt8531_read_live_link_state(dev, &state);

	if (ret != 0) {
		return ret;
	}
	if (state.is_up) {
		ret = mc_yt8531_read(dev, MII_BMCR, &bmcr);
		if (ret != 0) {
			return ret;
		}
		if ((bmcr & (MII_BMCR_RESET | MII_BMCR_POWER_DOWN)) != 0U) {
			return -EAGAIN;
		}
	}
	data->state = state;
	return 0;
}



static void monitor_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct mc_yt8531_data *data = CONTAINER_OF(dwork, struct mc_yt8531_data, monitor_work);

	if (k_mutex_lock(&data->lock, K_NO_WAIT) == 0) {
		if (update_link_state(data->dev) == 0) {
			invoke_link_cb(data->dev);
		}
		k_mutex_unlock(&data->lock);
	}
	k_work_reschedule(&data->monitor_work, K_MSEC(CONFIG_PHY_MONITOR_PERIOD));
}

static int mc_yt8531_cfg_link(const struct device *dev, enum phy_link_speed adv_speeds,
			     enum phy_cfg_link_flag flags)
{
	struct mc_yt8531_data *const data = dev->data;
	const struct mc_yt8531_config *const cfg = dev->config;
	int ret = 0;

	k_mutex_lock(&data->lock, K_FOREVER);

	if ((flags & PHY_FLAG_AUTO_NEGOTIATION_DISABLED) != 0U) {
		ret = phy_mii_set_bmcr_reg_autoneg_disabled(dev, adv_speeds);
		if (ret >= 0) {
			k_work_reschedule(&data->monitor_work, K_NO_WAIT);
		}
	} else {
		ret = phy_mii_cfg_link_autoneg(dev, adv_speeds, true);
		if (ret >= 0) {
			LOG_DBG("PHY (%d) Starting MII PHY auto-negotiate sequence", cfg->phy_addr);
			k_work_reschedule(&data->monitor_work,
					  K_MSEC(MII_AUTONEG_POLL_INTERVAL_MS));
		}
	}

	if (ret == -EALREADY) {
		LOG_DBG("PHY (%d) Link already configured", cfg->phy_addr);
	}

	k_mutex_unlock(&data->lock);

	return ret;
}

static int mc_yt8531_get_link_state(const struct device *dev, struct phy_link_state *state)
{
	struct mc_yt8531_data *const data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = mc_yt8531_read_live_link_state(dev, state);

	k_mutex_unlock(&data->lock);

	return ret;
}

static int mc_yt8531_link_cb_set(const struct device *dev, phy_callback_t cb, void *user_data)
{
	struct mc_yt8531_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	data->cb = cb;
	data->cb_data = user_data;
	data->notified_valid = false;
	if (update_link_state(dev) == 0) {
		invoke_link_cb(dev);
	}
	/* A failed initial read is retried by the monitor, without fabricating carrier. */
	k_mutex_unlock(&data->lock);
	return 0;
}

static int mc_yt8531_get_id(const struct device *dev, uint32_t *phy_id)
{
	const struct mc_yt8531_config *const config = dev->config;
	uint32_t val = 0;
	uint32_t attempt;
	int ret = 0;

	for (attempt = 1U; attempt <= YTPHY_PHY_ID_MAX_ATTEMPTS; attempt++) {
		ret = mc_yt8531_read(dev, MII_PHYID2R, &val);
		if (ret == 0 && ((val & PHY_ID_YT8531_REV_MASK) == (PHY_ID_YT8531 & PHY_ID_YT8531_REV_MASK))) {
			break;
		}

		if (attempt < YTPHY_PHY_ID_MAX_ATTEMPTS) {
			k_msleep(YTPHY_PHY_ID_RETRY_DELAY_MS);
		}
	}

	if (attempt > YTPHY_PHY_ID_MAX_ATTEMPTS) {
		if (ret < 0) {
			LOG_ERR("PHY (%d) failed to read ID after %u attempts (last error %d)",
				config->phy_addr, YTPHY_PHY_ID_MAX_ATTEMPTS, ret);
		} else {
			LOG_ERR("PHY (%d) unsupported ID after %u attempts: 0x%X", config->phy_addr,
				YTPHY_PHY_ID_MAX_ATTEMPTS, val);
		}

		return -EIO;
	}

	if (attempt > 1U) {
		LOG_WRN("PHY (%d) ID became valid after %u attempts", config->phy_addr, attempt);
	}

	if (phy_id) {
		*phy_id = val;
	} else {
		LOG_DBG("PHY (%d) ID:0x%X", config->phy_addr, val);
	}

	return 0;
}


static int mc_yt8531_init(const struct device *dev)
{
	struct mc_yt8531_data *const data = dev->data;
	const struct mc_yt8531_config *const cfg = dev->config;
	int ret;

	k_mutex_init(&data->lock);

	data->state.is_up = false;
	data->dev = dev;
	data->cb = NULL;

	if (!device_is_ready(cfg->mdio)) {
		LOG_ERR_DEVICE_NOT_READY(cfg->mdio);
		return -ENODEV;
	}

	if (mc_yt8531_get_id(dev, NULL)) {
		return -EIO;
	}

	/* Carried over from the shared YT8521 driver: the YT8531 is UTP-only,
	 * but the register-space select write is retained bit-for-bit until
	 * bench bring-up confirms it is a no-op on this part.
	 */
	ret = mc_yt8531_write_ext(dev, YT8521_REG_SPACE_SELECT_REG, YT8521_RSSR_UTP_SPACE);
	if (ret) {
		LOG_ERR("PHY (%d) failed to select UTP register space", cfg->phy_addr);
		return ret;
	}

	ret = mc_yt8531_modify_ext(dev, YTPHY_SYNCE_CFG_REG, YT8521_SCR_SYNCE_ENABLE, 0);
	if (ret) {
		LOG_ERR("PHY (%d) failed to disable SyncE", cfg->phy_addr);
		return ret;
	}

	/* Reset PHY */
	ret = mc_yt8531_soft_reset(dev);
	if (ret) {
		return ret;
	}

	/* Enable clock delay */
	ret = mc_yt8531_cfg_clock_delay(dev);
	if (ret) {
		LOG_ERR("PHY (%d) failed to configure RGMII delays", cfg->phy_addr);
		return ret;
	}

	ret = mc_yt8531_resume(dev);
	if (ret) {
		LOG_ERR("PHY (%d) failed to resume from power save", cfg->phy_addr);
		return ret;
	}

	LOG_INF("Motorcomm YT8531 PHY %d initialized", cfg->phy_addr);
	return 0;
}

static int mc_yt8531_initialize_dynamic_link(const struct device *dev)
{
	const struct mc_yt8531_config *const config = dev->config;
	struct mc_yt8531_data *const data = dev->data;
	int ret = 0;

	ret = mc_yt8531_init(dev);
	if (ret < 0) {
		return ret;
	}

	data->state.is_up = false;

	k_work_init_delayable(&data->monitor_work, monitor_work_handler);

	/* Advertise default speeds */
	ret = mc_yt8531_cfg_link(dev, config->default_speeds, 0);
	if ((ret < 0) && (ret != -EALREADY)) {
		LOG_ERR("PHY (%d) failed to configure advertised speeds (mask=0x%x, err=%d)",
			config->phy_addr, config->default_speeds, ret);
		return ret;
	}

	/* This will schedule the monitor work, if not already scheduled by mc_yt8531_cfg_link(). */
	k_work_schedule(&data->monitor_work, K_NO_WAIT);

	return 0;
}

static DEVICE_API(ethphy, mc_yt8531_driver_api) = {
	.get_link = mc_yt8531_get_link_state,
	.link_cb_set = mc_yt8531_link_cb_set,
	.cfg_link = mc_yt8531_cfg_link,
	.read = mc_yt8531_read,
	.write = mc_yt8531_write,
};

#define MC_YT8531_CONFIG(n)                                                                         \
	static const struct mc_yt8531_config mc_yt8531_config_##n = {                                \
		.phy_addr = DT_INST_REG_ADDR(n),                                                   \
		.mdio = DEVICE_DT_GET(DT_INST_BUS(n)),                                             \
		.rx_delay_sel = DT_INST_PROP_OR(n, motorcomm_rx_delay_sel, 0),                     \
		.tx_delay_sel = DT_INST_PROP_OR(n, motorcomm_tx_delay_sel, 0),                     \
		.rxc_dly_en = DT_INST_PROP(n, motorcomm_rxc_dly_en),                               \
		.default_speeds = PHY_INST_GENERATE_DEFAULT_SPEEDS(n),                             \
	};

#define MC_YT8531_DATA(n)                                                                           \
	static struct mc_yt8531_data mc_yt8531_data_##n = {                                          \
		.dev = DEVICE_DT_INST_GET(n),                                                      \
		.cb = NULL,                                                                        \
		.lock = Z_MUTEX_INITIALIZER(mc_yt8531_data_##n.lock),                             \
	};

#define MC_YT8531_INIT &mc_yt8531_initialize_dynamic_link

#define MC_YT8531_DEVICE(n)                                                                         \
	MC_YT8531_CONFIG(n)                                                                         \
	MC_YT8531_DATA(n)                                                                           \
	DEVICE_DT_INST_DEFINE(n, MC_YT8531_INIT, NULL, &mc_yt8531_data_##n, &mc_yt8531_config_##n,    \
			      POST_KERNEL, CONFIG_PHY_INIT_PRIORITY, &mc_yt8531_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MC_YT8531_DEVICE)
