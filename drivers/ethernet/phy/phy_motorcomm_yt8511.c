/*
 * Motorcomm YT8511 Ethernet PHY driver
 *
 * Copyright (c) 2025 Moton Intelligent Equipment
 * SPDX-License-Identifier: Apache-2.0
 *
 * Based on phy_motorcomm_yt8521.c (Copyright 2025 NXP) and register access
 * patterns from the legacy phy_xlnx_gem.c driver.
 */

#define DT_DRV_COMPAT motorcomm_yt8511

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(phy_motorcomm_yt8511, CONFIG_PHY_LOG_LEVEL);

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/net/phy.h>
#include <zephyr/net/mdio.h>
#include <zephyr/net/mii.h>

#include "phy_mii.h"

/* PHY ID for YT8511 (PHYID2 register value, mask off revision bits [3:0]) */
#define PHY_ID_YT8511		0x00000110
#define PHY_ID_YT8511_MASK	0xFFFFFFF0

/* Specific Status Register (vendor-specific) */
#define YT8511_SSR_REG		0x11
#define YT8511_SSR_LINK		BIT(10)
#define YT8511_SSR_SPEED_MASK	(0x3U << 14)
#define YT8511_SSR_SPEED_10M	(0U << 14)
#define YT8511_SSR_SPEED_100M	(1U << 14)
#define YT8511_SSR_SPEED_1000M	(2U << 14)
#define YT8511_SSR_DUPLEX_MASK	BIT(13)
#define YT8511_SSR_DUPLEX_FULL	BIT(13)

/* Extended register access via indirect registers 0x1E (page select) / 0x1F (data) */
#define YTPHY_PAGE_SELECT	0x1E
#define YTPHY_PAGE_DATA		0x1F

/* RGMII delay configuration register (extended register space) */
#define YT8511_RGMII_CFG1_REG	0xA003
#define YT8511_RC1R_RX_DELAY_MASK	GENMASK(13, 10)
#define YT8511_RC1R_TX_DELAY_MASK	GENMASK(3, 0)

/* Auto-negotiation polling interval */
#define MII_AUTONEG_POLL_INTERVAL_MS	100

struct yt8511_config {
	uint8_t phy_addr;
	const struct device *mdio;
	uint8_t rx_delay_sel;
	uint8_t tx_delay_sel;
	enum phy_link_speed default_speeds;
};

struct yt8511_data {
	const struct device *dev;
	phy_callback_t cb;
	void *cb_data;
	struct phy_link_state state;
	struct k_sem sem;
	struct k_work_delayable monitor_work;
	bool autoneg_in_progress;
	k_timepoint_t autoneg_timeout;
};

static int yt8511_read(const struct device *dev, uint16_t reg, uint32_t *data)
{
	const struct yt8511_config *cfg = dev->config;

	*data = 0U;
	return mdio_read(cfg->mdio, cfg->phy_addr, reg, (uint16_t *)data);
}

static int yt8511_write(const struct device *dev, uint16_t reg, uint32_t data)
{
	const struct yt8511_config *cfg = dev->config;

	return mdio_write(cfg->mdio, cfg->phy_addr, reg, (uint16_t)data);
}

static int yt8511_modify(const struct device *dev, uint16_t reg,
			  uint16_t mask, uint16_t set)
{
	uint32_t data;
	uint32_t new_val;
	int ret;

	ret = yt8511_read(dev, reg, &data);
	if (ret) {
		return ret;
	}

	new_val = (data & ~mask) | set;
	if (new_val == data) {
		return 0;
	}

	return yt8511_write(dev, reg, new_val);
}

static int yt8511_modify_ext(const struct device *dev, uint16_t reg,
			     uint16_t mask, uint16_t set)
{
	int ret;

	ret = yt8511_write(dev, YTPHY_PAGE_SELECT, reg);
	if (ret) {
		return ret;
	}

	return yt8511_modify(dev, YTPHY_PAGE_DATA, mask, set);
}

static int yt8511_soft_reset(const struct device *dev)
{
	int max_cnt = 500;
	uint32_t data;
	int ret;

	ret = yt8511_modify(dev, MII_BMCR, 0, MII_BMCR_RESET);
	if (ret) {
		return ret;
	}

	while (max_cnt--) {
		k_msleep(1);
		ret = yt8511_read(dev, MII_BMCR, &data);
		if (ret) {
			return ret;
		}
		if (!(data & MII_BMCR_RESET)) {
			return 0;
		}
	}

	LOG_ERR("PHY reset timed out");
	return -ETIMEDOUT;
}

static int yt8511_cfg_clock_delay(const struct device *dev)
{
	const struct yt8511_config *cfg = dev->config;
	uint16_t mask, val = 0;

	mask = YT8511_RC1R_RX_DELAY_MASK | YT8511_RC1R_TX_DELAY_MASK;
	val |= FIELD_PREP(YT8511_RC1R_RX_DELAY_MASK, cfg->rx_delay_sel);
	val |= FIELD_PREP(YT8511_RC1R_TX_DELAY_MASK, cfg->tx_delay_sel);

	return yt8511_modify_ext(dev, YT8511_RGMII_CFG1_REG, mask, val);
}

static int yt8511_get_link_state(const struct device *dev,
				 struct phy_link_state *state);

static void invoke_link_cb(const struct device *dev)
{
	struct yt8511_data *data = dev->data;
	struct phy_link_state state;

	if (data->cb == NULL) {
		return;
	}

	yt8511_get_link_state(dev, &state);
	data->cb(data->dev, &state, data->cb_data);
}

static inline enum phy_link_speed yt8511_decode_speed(uint16_t stat_reg)
{
	switch (stat_reg & (YT8511_SSR_SPEED_MASK | YT8511_SSR_DUPLEX_MASK)) {
	case YT8511_SSR_SPEED_10M | YT8511_SSR_DUPLEX_FULL:
		return LINK_FULL_10BASE;
	case YT8511_SSR_SPEED_10M:
		return LINK_HALF_10BASE;
	case YT8511_SSR_SPEED_100M | YT8511_SSR_DUPLEX_FULL:
		return LINK_FULL_100BASE;
	case YT8511_SSR_SPEED_100M:
		return LINK_HALF_100BASE;
	case YT8511_SSR_SPEED_1000M | YT8511_SSR_DUPLEX_FULL:
		return LINK_FULL_1000BASE;
	case YT8511_SSR_SPEED_1000M:
		return LINK_HALF_1000BASE;
	default:
		return 0;
	}
}

static int update_link_state(const struct device *dev)
{
	const struct yt8511_config *cfg = dev->config;
	struct yt8511_data *data = dev->data;
	uint32_t stat_reg;
	uint32_t bmcr_reg;
	bool link_up;

	if (yt8511_read(dev, YT8511_SSR_REG, &stat_reg) < 0) {
		return -EIO;
	}

	link_up = (uint16_t)stat_reg & YT8511_SSR_LINK;

	if (!link_up) {
		data->state.speed = 0;
		if (link_up != data->state.is_up) {
			data->state.is_up = false;
			LOG_INF("PHY (%d) is down", cfg->phy_addr);
			return 0;
		}
		return -EAGAIN;
	}

	if (yt8511_read(dev, MII_BMCR, &bmcr_reg) < 0) {
		return -EIO;
	}

	if ((bmcr_reg & MII_BMCR_AUTONEG_ENABLE) == 0U) {
		enum phy_link_speed new_speed = yt8511_decode_speed(stat_reg);

		if ((data->state.speed != new_speed) || !data->state.is_up) {
			data->state.is_up = true;
			data->state.speed = new_speed;
			LOG_INF("PHY (%d) Link speed %s Mb, %s duplex",
				cfg->phy_addr,
				PHY_LINK_IS_SPEED_1000M(data->state.speed) ? "1000" :
				(PHY_LINK_IS_SPEED_100M(data->state.speed) ? "100" : "10"),
				PHY_LINK_IS_FULL_DUPLEX(data->state.speed) ? "full" : "half");
			return 0;
		}
		return -EAGAIN;
	}

	if (data->state.is_up) {
		return -EAGAIN;
	}

	data->state.is_up = true;
	LOG_DBG("PHY (%d) Starting auto-negotiate sequence", cfg->phy_addr);
	data->autoneg_timeout = sys_timepoint_calc(K_MSEC(CONFIG_PHY_AUTONEG_TIMEOUT_MS));
	return -EINPROGRESS;
}

static int check_autoneg_completion(const struct device *dev)
{
	const struct yt8511_config *cfg = dev->config;
	struct yt8511_data *data = dev->data;
	uint32_t stat_reg;
	uint32_t bmsr_reg;

	/* BMSR bits may be latched; read twice for correct status */
	if (yt8511_read(dev, MII_BMSR, &bmsr_reg) < 0) {
		return -EIO;
	}
	if (yt8511_read(dev, MII_BMSR, &bmsr_reg) < 0) {
		return -EIO;
	}

	if ((bmsr_reg & MII_BMSR_AUTONEG_COMPLETE) == 0U) {
		if (sys_timepoint_expired(data->autoneg_timeout)) {
			LOG_DBG("PHY (%d) auto-negotiate timeout", cfg->phy_addr);
			return -ETIMEDOUT;
		}
		return -EINPROGRESS;
	}

	LOG_DBG("PHY (%d) auto-negotiate completed", cfg->phy_addr);

	if (yt8511_read(dev, YT8511_SSR_REG, &stat_reg) < 0) {
		return -EIO;
	}

	data->state.speed = yt8511_decode_speed(stat_reg);
	data->state.is_up = (bmsr_reg & MII_BMSR_LINK_STATUS) != 0U;

	LOG_INF("PHY (%d) Link speed %s Mb, %s duplex", cfg->phy_addr,
		PHY_LINK_IS_SPEED_1000M(data->state.speed) ? "1000" :
		(PHY_LINK_IS_SPEED_100M(data->state.speed) ? "100" : "10"),
		PHY_LINK_IS_FULL_DUPLEX(data->state.speed) ? "full" : "half");

	return 0;
}

static void monitor_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct yt8511_data *data = CONTAINER_OF(dwork, struct yt8511_data, monitor_work);
	const struct device *dev = data->dev;
	int rc;

	if (k_sem_take(&data->sem, K_NO_WAIT) == 0) {
		if (data->autoneg_in_progress) {
			rc = check_autoneg_completion(dev);
		} else {
			rc = update_link_state(dev);
		}

		data->autoneg_in_progress = (rc == -EINPROGRESS);
		k_sem_give(&data->sem);

		if (rc == 0) {
			invoke_link_cb(dev);
		}
	}

	k_work_reschedule(&data->monitor_work,
			  data->autoneg_in_progress
				  ? K_MSEC(MII_AUTONEG_POLL_INTERVAL_MS)
				  : K_MSEC(CONFIG_PHY_MONITOR_PERIOD));
}

static int yt8511_cfg_link(const struct device *dev, enum phy_link_speed adv_speeds,
			   enum phy_cfg_link_flag flags)
{
	struct yt8511_data *data = dev->data;
	const struct yt8511_config *cfg = dev->config;
	int ret = 0;

	k_sem_take(&data->sem, K_FOREVER);

	if ((flags & PHY_FLAG_AUTO_NEGOTIATION_DISABLED) != 0U) {
		ret = phy_mii_set_bmcr_reg_autoneg_disabled(dev, adv_speeds);
		if (ret >= 0) {
			data->autoneg_in_progress = false;
			k_work_reschedule(&data->monitor_work, K_NO_WAIT);
		}
	} else {
		ret = phy_mii_cfg_link_autoneg(dev, adv_speeds, true);
		if (ret >= 0) {
			LOG_DBG("PHY (%d) Starting auto-negotiate sequence", cfg->phy_addr);
			data->autoneg_in_progress = true;
			data->autoneg_timeout =
				sys_timepoint_calc(K_MSEC(CONFIG_PHY_AUTONEG_TIMEOUT_MS));
			k_work_reschedule(&data->monitor_work,
					  K_MSEC(MII_AUTONEG_POLL_INTERVAL_MS));
		}
	}

	if (ret == -EALREADY) {
		LOG_DBG("PHY (%d) Link already configured", cfg->phy_addr);
	}

	k_sem_give(&data->sem);
	return ret;
}

static int yt8511_get_link_state(const struct device *dev, struct phy_link_state *state)
{
	struct yt8511_data *data = dev->data;

	k_sem_take(&data->sem, K_FOREVER);
	memcpy(state, &data->state, sizeof(struct phy_link_state));
	if (state->speed == 0) {
		state->is_up = false;
	}
	k_sem_give(&data->sem);

	return 0;
}

static int yt8511_link_cb_set(const struct device *dev, phy_callback_t cb, void *user_data)
{
	struct yt8511_data *data = dev->data;

	data->cb = cb;
	data->cb_data = user_data;

	invoke_link_cb(dev);
	return 0;
}

static int yt8511_init(const struct device *dev)
{
	struct yt8511_data *data = dev->data;
	const struct yt8511_config *cfg = dev->config;
	uint32_t phy_id;
	int ret;

	k_sem_init(&data->sem, 1, 1);
	data->state.is_up = false;
	data->dev = dev;
	data->cb = NULL;

	/* Verify PHY identity */
	ret = yt8511_read(dev, MII_PHYID2R, &phy_id);
	if (ret) {
		LOG_ERR("PHY (%d) failed to read ID", cfg->phy_addr);
		return -EIO;
	}

	if ((phy_id & PHY_ID_YT8511_MASK) != PHY_ID_YT8511) {
		LOG_ERR("PHY (%d) unexpected ID: 0x%04X (expected 0x%04X)",
			cfg->phy_addr, phy_id, PHY_ID_YT8511);
		return -ENODEV;
	}

	LOG_DBG("PHY (%d) ID: 0x%04X", cfg->phy_addr, phy_id);

	/* Soft reset */
	ret = yt8511_soft_reset(dev);
	if (ret) {
		LOG_ERR("PHY (%d) soft reset failed", cfg->phy_addr);
		return -EIO;
	}

	/* Configure RGMII clock delays */
	ret = yt8511_cfg_clock_delay(dev);
	if (ret) {
		LOG_ERR("PHY (%d) clock delay config failed", cfg->phy_addr);
		return ret;
	}

	/* Start link monitoring */
	k_work_init_delayable(&data->monitor_work, monitor_work_handler);

	/* Advertise default speeds */
	yt8511_cfg_link(dev, cfg->default_speeds, 0);

	k_work_schedule(&data->monitor_work, K_NO_WAIT);

	LOG_INF("Motorcomm YT8511 PHY %d initialized", cfg->phy_addr);
	return 0;
}

static DEVICE_API(ethphy, yt8511_api) = {
	.get_link = yt8511_get_link_state,
	.link_cb_set = yt8511_link_cb_set,
	.cfg_link = yt8511_cfg_link,
	.read = yt8511_read,
	.write = yt8511_write,
};

#define YT8511_CONFIG(n)							\
	static const struct yt8511_config yt8511_config_##n = {			\
		.phy_addr = DT_INST_REG_ADDR(n),				\
		.mdio = DEVICE_DT_GET(DT_INST_BUS(n)),				\
		.rx_delay_sel = DT_INST_PROP_OR(n, motorcomm_rx_delay_sel, 0),	\
		.tx_delay_sel = DT_INST_PROP_OR(n, motorcomm_tx_delay_sel, 0),	\
		.default_speeds = PHY_INST_GENERATE_DEFAULT_SPEEDS(n),		\
	};

#define YT8511_DATA(n)								\
	static struct yt8511_data yt8511_data_##n = {				\
		.dev = DEVICE_DT_INST_GET(n),					\
		.cb = NULL,							\
		.sem = Z_SEM_INITIALIZER(yt8511_data_##n.sem, 1, 1),		\
	};

#define YT8511_DEVICE(n)							\
	YT8511_CONFIG(n)							\
	YT8511_DATA(n)								\
	DEVICE_DT_INST_DEFINE(n, yt8511_init, NULL,				\
			      &yt8511_data_##n, &yt8511_config_##n,		\
			      POST_KERNEL, CONFIG_PHY_INIT_PRIORITY,		\
			      &yt8511_api);

DT_INST_FOREACH_STATUS_OKAY(YT8511_DEVICE)
