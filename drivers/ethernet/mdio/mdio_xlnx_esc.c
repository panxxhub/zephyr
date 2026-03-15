/*
 * Xilinx ESC (EtherCAT Slave Controller) MDIO bus driver
 *
 * Copyright (c) 2025 Moton Intelligent Equipment
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT xlnx_esc_mdio

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mdio_xlnx_esc, CONFIG_MDIO_LOG_LEVEL);

/*
 * ESC MII management registers (offsets from ESC base):
 *   0x510: MII Control/Status (phy_addr, cmd, busy, port, reg_addr)
 *   0x514: MII Data (16-bit read/write data)
 */
#define ESC_MII_CTRLSTAT_OFFSET	0x0510
#define ESC_MII_DATA_OFFSET	0x0514

/* MII Control/Status register bitfield layout (little-endian) */
#define MII_CTRL_BUSY_BIT	BIT(15)	 /* byte 0x511 bit 7 */
#define MII_CTRL_CMD_SHIFT	8	 /* byte 0x511 bits [1:0] */
#define MII_CTRL_CMD_MASK	(0x3 << MII_CTRL_CMD_SHIFT)
#define MII_CTRL_CMD_READ	(1 << MII_CTRL_CMD_SHIFT)
#define MII_CTRL_CMD_WRITE	(2 << MII_CTRL_CMD_SHIFT)
#define MII_CTRL_PHY_ADDR_SHIFT	3    /* byte 0x510 bits [7:3] */
#define MII_CTRL_PHY_ADDR_MASK	(0x1F << MII_CTRL_PHY_ADDR_SHIFT)
#define MII_CTRL_PORT_SHIFT	16   /* byte 0x512 bits [1:0] */
#define MII_CTRL_PORT_MASK	(0x3 << MII_CTRL_PORT_SHIFT)
#define MII_CTRL_REG_ADDR_SHIFT	24   /* byte 0x513 bits [4:0] */
#define MII_CTRL_REG_ADDR_MASK	(0x1F << MII_CTRL_REG_ADDR_SHIFT)

#define MDIO_POLL_RETRIES	10
#define MDIO_POLL_INTERVAL_US	100

struct mdio_xlnx_esc_config {
	DEVICE_MMIO_ROM;
	uint8_t port_id;
};

struct mdio_xlnx_esc_data {
	DEVICE_MMIO_RAM;
	struct k_mutex mutex;
};

static inline uint32_t esc_mdio_read32(const struct device *dev, uint32_t offset)
{
	return sys_read32(DEVICE_MMIO_GET(dev) + offset);
}

static inline void esc_mdio_write32(const struct device *dev, uint32_t offset, uint32_t val)
{
	sys_write32(val, DEVICE_MMIO_GET(dev) + offset);
}

static inline void esc_mdio_write16(const struct device *dev, uint32_t offset, uint16_t val)
{
	sys_write16(val, DEVICE_MMIO_GET(dev) + offset);
}

static inline uint16_t esc_mdio_read16(const struct device *dev, uint32_t offset)
{
	return sys_read16(DEVICE_MMIO_GET(dev) + offset);
}

static int mdio_xlnx_esc_wait_idle(const struct device *dev)
{
	uint32_t reg_val;

	for (int i = 0; i < MDIO_POLL_RETRIES; i++) {
		reg_val = esc_mdio_read32(dev, ESC_MII_CTRLSTAT_OFFSET);
		if (!(reg_val & MII_CTRL_BUSY_BIT)) {
			return 0;
		}
		k_busy_wait(MDIO_POLL_INTERVAL_US);
	}

	LOG_ERR("ESC MDIO bus idle timeout");
	return -ETIMEDOUT;
}

static int mdio_xlnx_esc_read_c22(const struct device *dev, uint8_t prtad,
				   uint8_t regad, uint16_t *data)
{
	const struct mdio_xlnx_esc_config *cfg = dev->config;
	struct mdio_xlnx_esc_data *priv = dev->data;
	uint32_t ctrl;
	int ret;

	k_mutex_lock(&priv->mutex, K_FOREVER);

	ret = mdio_xlnx_esc_wait_idle(dev);
	if (ret) {
		goto out;
	}

	ctrl = ((uint32_t)prtad << MII_CTRL_PHY_ADDR_SHIFT) & MII_CTRL_PHY_ADDR_MASK;
	ctrl |= ((uint32_t)cfg->port_id << MII_CTRL_PORT_SHIFT) & MII_CTRL_PORT_MASK;
	ctrl |= ((uint32_t)regad << MII_CTRL_REG_ADDR_SHIFT) & MII_CTRL_REG_ADDR_MASK;
	ctrl |= MII_CTRL_CMD_READ;

	esc_mdio_write32(dev, ESC_MII_CTRLSTAT_OFFSET, ctrl);

	ret = mdio_xlnx_esc_wait_idle(dev);
	if (ret) {
		goto out;
	}

	*data = esc_mdio_read16(dev, ESC_MII_DATA_OFFSET);

out:
	k_mutex_unlock(&priv->mutex);
	return ret;
}

static int mdio_xlnx_esc_write_c22(const struct device *dev, uint8_t prtad,
				    uint8_t regad, uint16_t data)
{
	const struct mdio_xlnx_esc_config *cfg = dev->config;
	struct mdio_xlnx_esc_data *priv = dev->data;
	uint32_t ctrl;
	int ret;

	k_mutex_lock(&priv->mutex, K_FOREVER);

	ret = mdio_xlnx_esc_wait_idle(dev);
	if (ret) {
		goto out;
	}

	ctrl = ((uint32_t)prtad << MII_CTRL_PHY_ADDR_SHIFT) & MII_CTRL_PHY_ADDR_MASK;
	ctrl |= ((uint32_t)cfg->port_id << MII_CTRL_PORT_SHIFT) & MII_CTRL_PORT_MASK;
	ctrl |= ((uint32_t)regad << MII_CTRL_REG_ADDR_SHIFT) & MII_CTRL_REG_ADDR_MASK;
	ctrl |= MII_CTRL_CMD_WRITE;

	esc_mdio_write16(dev, ESC_MII_DATA_OFFSET, data);
	esc_mdio_write32(dev, ESC_MII_CTRLSTAT_OFFSET, ctrl);

	ret = mdio_xlnx_esc_wait_idle(dev);

out:
	k_mutex_unlock(&priv->mutex);
	return ret;
}

static int mdio_xlnx_esc_init(const struct device *dev)
{
	struct mdio_xlnx_esc_data *priv = dev->data;

	DEVICE_MMIO_MAP(dev, K_MEM_PERM_RW);
	k_mutex_init(&priv->mutex);

	LOG_DBG("ESC MDIO bus initialized (port %u)",
		((const struct mdio_xlnx_esc_config *)dev->config)->port_id);
	return 0;
}

static DEVICE_API(mdio, mdio_xlnx_esc_api) = {
	.read = mdio_xlnx_esc_read_c22,
	.write = mdio_xlnx_esc_write_c22,
};

#define MDIO_XLNX_ESC_DEVICE(n)							\
	static struct mdio_xlnx_esc_data mdio_xlnx_esc_data_##n;		\
	static const struct mdio_xlnx_esc_config mdio_xlnx_esc_cfg_##n = {	\
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),				\
		.port_id = DT_INST_PROP(n, xlnx_port_id),			\
	};									\
	DEVICE_DT_INST_DEFINE(n, mdio_xlnx_esc_init, NULL,			\
			      &mdio_xlnx_esc_data_##n,				\
			      &mdio_xlnx_esc_cfg_##n,				\
			      POST_KERNEL, CONFIG_MDIO_INIT_PRIORITY,		\
			      &mdio_xlnx_esc_api);

DT_INST_FOREACH_STATUS_OKAY(MDIO_XLNX_ESC_DEVICE)
