/*
 * Xilinx GEM MDIO bus driver
 *
 * Copyright (c) 2025 Moton Intelligent Equipment
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT xlnx_gem_mdio

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mdio_xlnx_gem, CONFIG_MDIO_LOG_LEVEL);

/* Register offsets within the GEM register space */
#define GEM_NWCTRL_OFFSET		0x00000000
#define GEM_NWSR_OFFSET			0x00000008
#define GEM_PHY_MAINTENANCE_OFFSET	0x00000034

/* gem.net_ctrl bits */
#define GEM_NWCTRL_MDEN_BIT		0x00000010

/* gem.net_status bits */
#define GEM_MDIO_IDLE_BIT		0x00000004

/* gem.phy_maint bits and fields */
#define GEM_PHY_MAINT_CONST_BITS	0x40020000
#define GEM_PHY_MAINT_READ_OP_BIT	0x20000000
#define GEM_PHY_MAINT_WRITE_OP_BIT	0x10000000
#define GEM_PHY_MAINT_PHY_ADDR_MASK	0x0000001F
#define GEM_PHY_MAINT_PHY_ADDR_SHIFT	23
#define GEM_PHY_MAINT_REG_ADDR_MASK	0x0000001F
#define GEM_PHY_MAINT_REG_ADDR_SHIFT	18
#define GEM_PHY_MAINT_DATA_MASK		0x0000FFFF

#define MDIO_POLL_RETRIES		10
#define MDIO_POLL_INTERVAL_US		100

struct mdio_xlnx_gem_config {
	uint32_t base_addr;
};

struct mdio_xlnx_gem_data {
	struct k_mutex mutex;
};

static int mdio_xlnx_gem_wait_idle(const struct mdio_xlnx_gem_config *cfg)
{
	uint32_t reg_val;

	for (int i = 0; i < MDIO_POLL_RETRIES; i++) {
		reg_val = sys_read32(cfg->base_addr + GEM_NWSR_OFFSET);
		if (reg_val & GEM_MDIO_IDLE_BIT) {
			return 0;
		}
		k_busy_wait(MDIO_POLL_INTERVAL_US);
	}

	LOG_ERR("MDIO bus idle timeout (base=0x%08x)", cfg->base_addr);
	return -ETIMEDOUT;
}

static int mdio_xlnx_gem_read_c22(const struct device *dev, uint8_t prtad,
				   uint8_t regad, uint16_t *data)
{
	const struct mdio_xlnx_gem_config *cfg = dev->config;
	struct mdio_xlnx_gem_data *priv = dev->data;
	uint32_t reg_val;
	int ret;

	k_mutex_lock(&priv->mutex, K_FOREVER);

	ret = mdio_xlnx_gem_wait_idle(cfg);
	if (ret) {
		goto out;
	}

	reg_val = GEM_PHY_MAINT_CONST_BITS | GEM_PHY_MAINT_READ_OP_BIT;
	reg_val |= ((uint32_t)prtad & GEM_PHY_MAINT_PHY_ADDR_MASK) <<
		   GEM_PHY_MAINT_PHY_ADDR_SHIFT;
	reg_val |= ((uint32_t)regad & GEM_PHY_MAINT_REG_ADDR_MASK) <<
		   GEM_PHY_MAINT_REG_ADDR_SHIFT;

	sys_write32(reg_val, cfg->base_addr + GEM_PHY_MAINTENANCE_OFFSET);

	ret = mdio_xlnx_gem_wait_idle(cfg);
	if (ret) {
		goto out;
	}

	reg_val = sys_read32(cfg->base_addr + GEM_PHY_MAINTENANCE_OFFSET);
	*data = (uint16_t)(reg_val & GEM_PHY_MAINT_DATA_MASK);

out:
	k_mutex_unlock(&priv->mutex);
	return ret;
}

static int mdio_xlnx_gem_write_c22(const struct device *dev, uint8_t prtad,
				    uint8_t regad, uint16_t data)
{
	const struct mdio_xlnx_gem_config *cfg = dev->config;
	struct mdio_xlnx_gem_data *priv = dev->data;
	uint32_t reg_val;
	int ret;

	k_mutex_lock(&priv->mutex, K_FOREVER);

	ret = mdio_xlnx_gem_wait_idle(cfg);
	if (ret) {
		goto out;
	}

	reg_val = GEM_PHY_MAINT_CONST_BITS | GEM_PHY_MAINT_WRITE_OP_BIT;
	reg_val |= ((uint32_t)prtad & GEM_PHY_MAINT_PHY_ADDR_MASK) <<
		   GEM_PHY_MAINT_PHY_ADDR_SHIFT;
	reg_val |= ((uint32_t)regad & GEM_PHY_MAINT_REG_ADDR_MASK) <<
		   GEM_PHY_MAINT_REG_ADDR_SHIFT;
	reg_val |= (uint32_t)data & GEM_PHY_MAINT_DATA_MASK;

	sys_write32(reg_val, cfg->base_addr + GEM_PHY_MAINTENANCE_OFFSET);

	ret = mdio_xlnx_gem_wait_idle(cfg);

out:
	k_mutex_unlock(&priv->mutex);
	return ret;
}

static int mdio_xlnx_gem_init(const struct device *dev)
{
	const struct mdio_xlnx_gem_config *cfg = dev->config;
	struct mdio_xlnx_gem_data *priv = dev->data;
	uint32_t reg_val;

	k_mutex_init(&priv->mutex);

	/* Enable MDIO port: set gem.net_ctrl[mgmt_port_en] */
	reg_val = sys_read32(cfg->base_addr + GEM_NWCTRL_OFFSET);
	reg_val |= GEM_NWCTRL_MDEN_BIT;
	sys_write32(reg_val, cfg->base_addr + GEM_NWCTRL_OFFSET);

	LOG_DBG("MDIO bus initialized (base=0x%08x)", cfg->base_addr);
	return 0;
}

static DEVICE_API(mdio, mdio_xlnx_gem_api) = {
	.read = mdio_xlnx_gem_read_c22,
	.write = mdio_xlnx_gem_write_c22,
};

#define MDIO_XLNX_GEM_DEVICE(n)						\
	static struct mdio_xlnx_gem_data mdio_xlnx_gem_data_##n;		\
	static const struct mdio_xlnx_gem_config mdio_xlnx_gem_cfg_##n = {	\
		.base_addr = DT_INST_REG_ADDR(n),				\
	};									\
	DEVICE_DT_INST_DEFINE(n, mdio_xlnx_gem_init, NULL,			\
			      &mdio_xlnx_gem_data_##n,				\
			      &mdio_xlnx_gem_cfg_##n,				\
			      POST_KERNEL, CONFIG_MDIO_INIT_PRIORITY,		\
			      &mdio_xlnx_gem_api);

DT_INST_FOREACH_STATUS_OKAY(MDIO_XLNX_GEM_DEVICE)
