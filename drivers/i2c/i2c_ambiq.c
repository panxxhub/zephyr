/*
 * Copyright (c) 2023 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ambiq_i2c

#include <errno.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/cache.h>

#ifdef CONFIG_I2C_AMBIQ_BUS_RECOVERY
#include <zephyr/drivers/gpio.h>
#include "i2c_bitbang.h"
#endif /* CONFIG_I2C_AMBIQ_BUS_RECOVERY */

#include <zephyr/logging/log.h>
#include <zephyr/drivers/pinctrl.h>

LOG_MODULE_REGISTER(ambiq_i2c, CONFIG_I2C_LOG_LEVEL);

#define I2C_TRANSFER_TIMEOUT_MSEC 500 /* Transfer timeout period */

#include "i2c-priv.h"

#include <soc.h>

struct i2c_ambiq_config {
#ifdef CONFIG_I2C_AMBIQ_BUS_RECOVERY
	struct gpio_dt_spec scl;
	struct gpio_dt_spec sda;
#endif /* CONFIG_I2C_AMBIQ_BUS_RECOVERY */
	uint32_t base;
	int size;
	int inst_idx;
	uint32_t bitrate;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config_func)(void);
};

typedef void (*i2c_ambiq_callback_t)(const struct device *dev, int result, void *data);

struct i2c_ambiq_data {
	am_hal_iom_config_t iom_cfg;
	void *iom_handler;
	struct k_sem bus_sem;
	struct k_sem transfer_sem;
	i2c_ambiq_callback_t callback;
	void *callback_data;
	uint32_t transfer_status;
	bool pm_policy_state_on;
	bool dma_mode;
};

static void i2c_ambiq_pm_policy_state_lock_get(const struct device *dev)
{
	if (IS_ENABLED(CONFIG_PM)) {
		struct i2c_ambiq_data *data = dev->data;

		if (!data->pm_policy_state_on) {
			data->pm_policy_state_on = true;
			pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_RAM, PM_ALL_SUBSTATES);
			pm_device_runtime_get(dev);
		}
	}
}

static void i2c_ambiq_pm_policy_state_lock_put(const struct device *dev)
{
	if (IS_ENABLED(CONFIG_PM)) {
		struct i2c_ambiq_data *data = dev->data;

		if (data->pm_policy_state_on) {
			data->pm_policy_state_on = false;
			pm_device_runtime_put(dev);
			pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_RAM, PM_ALL_SUBSTATES);
		}
	}
}

static void i2c_ambiq_callback(void *callback_ctxt, uint32_t status)
{
	const struct device *dev = callback_ctxt;
	struct i2c_ambiq_data *data = dev->data;

	if (data->callback) {
		data->callback(dev, status, data->callback_data);
	}
	data->transfer_status = status;
}

static void i2c_ambiq_isr(const struct device *dev)
{
	uint32_t ui32Status;
	struct i2c_ambiq_data *data = dev->data;

	am_hal_iom_interrupt_status_get(data->iom_handler, false, &ui32Status);
	am_hal_iom_interrupt_clear(data->iom_handler, ui32Status);
	am_hal_iom_interrupt_service(data->iom_handler, ui32Status);
	k_sem_give(&data->transfer_sem);
}

static int i2c_ambiq_read(const struct device *dev, struct i2c_msg *hdr_msg,
			   struct i2c_msg *data_msg, uint16_t addr)
{
	struct i2c_ambiq_data *data = dev->data;

	int ret = 0;

	am_hal_iom_transfer_t trans = {0};

	trans.ui8Priority = 1;
	trans.eDirection = AM_HAL_IOM_RX;
	trans.uPeerInfo.ui32I2CDevAddr = addr;
	trans.ui32NumBytes = data_msg->len;
	trans.pui32RxBuffer = (uint32_t *)data_msg->buf;
	if (hdr_msg) {
		if (hdr_msg->len > AM_HAL_IOM_MAX_OFFSETSIZE) {
			return -E2BIG;
		}
#if defined(CONFIG_SOC_SERIES_APOLLO3X)
		memcpy(&trans.ui32Instr, hdr_msg->buf, hdr_msg->len);
#else
		memcpy(&trans.ui64Instr, hdr_msg->buf, hdr_msg->len);
#endif
		trans.ui32InstrLen = hdr_msg->len;
	}

	if (data->dma_mode) {
		data->transfer_status = -EFAULT;
		ret = am_hal_iom_nonblocking_transfer(data->iom_handler, &trans, i2c_ambiq_callback,
						      (void *)dev);
		if (k_sem_take(&data->transfer_sem, K_MSEC(I2C_TRANSFER_TIMEOUT_MSEC))) {
			LOG_ERR("Timeout waiting for transfer complete");
			/* cancel timed out transaction */
			am_hal_iom_disable(data->iom_handler);
			/* clean up for next xfer */
			k_sem_reset(&data->transfer_sem);
			am_hal_iom_enable(data->iom_handler);
			return -ETIMEDOUT;
		}
#if CONFIG_I2C_AMBIQ_HANDLE_CACHE
		if (!buf_in_nocache((uintptr_t)trans.pui32RxBuffer, trans.ui32NumBytes)) {
			/* Invalidate Dcache after DMA read */
			sys_cache_data_invd_range((void *)trans.pui32RxBuffer, trans.ui32NumBytes);
		}
#endif /* CONFIG_I2C_AMBIQ_HANDLE_CACHE */
		ret = data->transfer_status;
	} else {
		ret = am_hal_iom_blocking_transfer(data->iom_handler, &trans);
	}

	return (ret != AM_HAL_STATUS_SUCCESS) ? -EIO : 0;
}

static int i2c_ambiq_write(const struct device *dev, struct i2c_msg *hdr_msg,
			   struct i2c_msg *data_msg, uint16_t addr)
{
	struct i2c_ambiq_data *data = dev->data;

	int ret = 0;

	am_hal_iom_transfer_t trans = {0};

	trans.ui8Priority = 1;
	trans.eDirection = AM_HAL_IOM_TX;
	trans.uPeerInfo.ui32I2CDevAddr = addr;
	trans.ui32NumBytes = data_msg->len;
	trans.pui32TxBuffer = (uint32_t *)data_msg->buf;
	if (hdr_msg) {
		if (hdr_msg->len > AM_HAL_IOM_MAX_OFFSETSIZE) {
			return -E2BIG;
		}
#if defined(CONFIG_SOC_SERIES_APOLLO3X)
		memcpy(&trans.ui32Instr, hdr_msg->buf, hdr_msg->len);
#else
		memcpy(&trans.ui64Instr, hdr_msg->buf, hdr_msg->len);
#endif
		trans.ui32InstrLen = hdr_msg->len;
	}

	if (data->dma_mode) {
		data->transfer_status = -EFAULT;
#if CONFIG_I2C_AMBIQ_HANDLE_CACHE
		if (!buf_in_nocache((uintptr_t)trans.pui32TxBuffer, trans.ui32NumBytes)) {
			/* Clean Dcache before DMA write */
			sys_cache_data_flush_range((void *)trans.pui32TxBuffer, trans.ui32NumBytes);
		}
#endif /* CONFIG_I2C_AMBIQ_HANDLE_CACHE */
		ret = am_hal_iom_nonblocking_transfer(data->iom_handler, &trans, i2c_ambiq_callback,
						      (void *)dev);

		if (k_sem_take(&data->transfer_sem, K_MSEC(I2C_TRANSFER_TIMEOUT_MSEC))) {
			LOG_ERR("Timeout waiting for transfer complete");
			/* cancel timed out transaction */
			am_hal_iom_disable(data->iom_handler);
			/* clean up for next xfer */
			k_sem_reset(&data->transfer_sem);
			am_hal_iom_enable(data->iom_handler);
			return -ETIMEDOUT;
		}
		ret = data->transfer_status;
	} else {
		ret = am_hal_iom_blocking_transfer(data->iom_handler, &trans);
	}

	return (ret != AM_HAL_STATUS_SUCCESS) ? -EIO : 0;
}

static int i2c_ambiq_configure(const struct device *dev, uint32_t dev_config)
{
	struct i2c_ambiq_data *data = dev->data;

	if (!(I2C_MODE_CONTROLLER & dev_config)) {
		return -EINVAL;
	}

	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:
		data->iom_cfg.ui32ClockFreq = AM_HAL_IOM_100KHZ;
		break;
	case I2C_SPEED_FAST:
		data->iom_cfg.ui32ClockFreq = AM_HAL_IOM_400KHZ;
		break;
	case I2C_SPEED_FAST_PLUS:
		data->iom_cfg.ui32ClockFreq = AM_HAL_IOM_1MHZ;
		break;
	default:
		return -EINVAL;
	}

	am_hal_iom_configure(data->iom_handler, &data->iom_cfg);

	return 0;
}

static int i2c_ambiq_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			      uint16_t addr)
{
	struct i2c_ambiq_data *data = dev->data;
	int ret = 0;

	if (!num_msgs) {
		return 0;
	}

	i2c_ambiq_pm_policy_state_lock_get(dev);

	/* Send out messages */
	k_sem_take(&data->bus_sem, K_FOREVER);

	for (int i = 0; i < num_msgs; i++) {
		if (msgs[i].flags & I2C_MSG_READ) {
			ret = i2c_ambiq_read(dev, NULL, &(msgs[i]), addr);
		} else if ((i + 1) < num_msgs) {
			if (msgs[i + 1].flags & I2C_MSG_READ) {
				ret = i2c_ambiq_read(dev, &(msgs[i]), &(msgs[i + 1]), addr);
			} else {
				ret = i2c_ambiq_write(dev, &(msgs[i]), &(msgs[i + 1]), addr);
			}
			i++;
		} else {
			ret = i2c_ambiq_write(dev, NULL, &(msgs[i]), addr);
		}

		if (ret != 0) {
			LOG_ERR("i2c transfer failed: %d", ret);
			break;
		}
	}

	k_sem_give(&data->bus_sem);

	i2c_ambiq_pm_policy_state_lock_put(dev);

	return ret;
}

#if CONFIG_I2C_AMBIQ_BUS_RECOVERY
static void i2c_ambiq_bitbang_set_scl(void *io_context, int state)
{
	const struct i2c_ambiq_config *config = io_context;

	gpio_pin_set_dt(&config->scl, state);
}

static void i2c_ambiq_bitbang_set_sda(void *io_context, int state)
{
	const struct i2c_ambiq_config *config = io_context;

	gpio_pin_set_dt(&config->sda, state);
}

static int i2c_ambiq_bitbang_get_sda(void *io_context)
{
	const struct i2c_ambiq_config *config = io_context;

	return gpio_pin_get_dt(&config->sda) == 0 ? 0 : 1;
}

static int i2c_ambiq_recover_bus(const struct device *dev)
{
	const struct i2c_ambiq_config *config = dev->config;
	struct i2c_ambiq_data *data = dev->data;
	struct i2c_bitbang bitbang_ctx;
	struct i2c_bitbang_io bitbang_io = {
		.set_scl = i2c_ambiq_bitbang_set_scl,
		.set_sda = i2c_ambiq_bitbang_set_sda,
		.get_sda = i2c_ambiq_bitbang_get_sda,
	};
	uint32_t bitrate_cfg;
	int error = 0;

	LOG_ERR("attempting to recover bus");

	if (!gpio_is_ready_dt(&config->scl)) {
		LOG_ERR("SCL GPIO device not ready");
		return -EIO;
	}

	if (!gpio_is_ready_dt(&config->sda)) {
		LOG_ERR("SDA GPIO device not ready");
		return -EIO;
	}

	k_sem_take(&data->bus_sem, K_FOREVER);

	error = gpio_pin_configure_dt(&config->scl, GPIO_OUTPUT_HIGH);
	if (error != 0) {
		LOG_ERR("failed to configure SCL GPIO (err %d)", error);
		goto restore;
	}

	error = gpio_pin_configure_dt(&config->sda, GPIO_OUTPUT_HIGH);
	if (error != 0) {
		LOG_ERR("failed to configure SDA GPIO (err %d)", error);
		goto restore;
	}

	i2c_bitbang_init(&bitbang_ctx, &bitbang_io, (void *)config);

	bitrate_cfg = i2c_map_dt_bitrate(config->bitrate) | I2C_MODE_CONTROLLER;
	error = i2c_bitbang_configure(&bitbang_ctx, bitrate_cfg);
	if (error != 0) {
		LOG_ERR("failed to configure I2C bitbang (err %d)", error);
		goto restore;
	}

	error = i2c_bitbang_recover_bus(&bitbang_ctx);
	if (error != 0) {
		LOG_ERR("failed to recover bus (err %d)", error);
	}

restore:
	(void)pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);

	k_sem_give(&data->bus_sem);

	return error;
}
#endif /* CONFIG_I2C_AMBIQ_BUS_RECOVERY */

static int i2c_ambiq_init(const struct device *dev)
{
	struct i2c_ambiq_data *data = dev->data;
	const struct i2c_ambiq_config *config = dev->config;
	uint32_t bitrate_cfg = i2c_map_dt_bitrate(config->bitrate);
	int ret = 0;

	if (AM_HAL_STATUS_SUCCESS != am_hal_iom_initialize(config->inst_idx, &data->iom_handler)) {
		LOG_ERR("Fail to initialize I2C\n");
		return -ENXIO;
	}

	ret = am_hal_iom_power_ctrl(data->iom_handler, AM_HAL_SYSCTRL_WAKE, false);

	ret |= i2c_ambiq_configure(dev, I2C_MODE_CONTROLLER | bitrate_cfg);
	if (ret < 0) {
		LOG_ERR("Fail to config I2C\n");
		goto end;
	}

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("Fail to config I2C pins\n");
		goto end;
	}

	if (data->dma_mode) {
		am_hal_iom_interrupt_clear(data->iom_handler,
					   AM_HAL_IOM_INT_DCMP | AM_HAL_IOM_INT_CMDCMP);
		am_hal_iom_interrupt_enable(data->iom_handler,
					    AM_HAL_IOM_INT_DCMP | AM_HAL_IOM_INT_CMDCMP);
		config->irq_config_func();
	}

	if (AM_HAL_STATUS_SUCCESS != am_hal_iom_enable(data->iom_handler)) {
		LOG_ERR("Fail to enable I2C\n");
		ret = -EIO;
	}
end:
	if (ret < 0) {
		am_hal_iom_uninitialize(data->iom_handler);
	}
	return ret;
}

static DEVICE_API(i2c, i2c_ambiq_driver_api) = {
	.configure = i2c_ambiq_configure,
	.transfer = i2c_ambiq_transfer,
#if CONFIG_I2C_AMBIQ_BUS_RECOVERY
	.recover_bus = i2c_ambiq_recover_bus,
#endif /* CONFIG_I2C_AMBIQ_BUS_RECOVERY */
#ifdef CONFIG_I2C_RTIO
	.iodev_submit = i2c_iodev_submit_fallback,
#endif
};

#ifdef CONFIG_PM_DEVICE
static int i2c_ambiq_pm_action(const struct device *dev, enum pm_device_action action)
{
	const struct i2c_ambiq_config *config = dev->config;
	struct i2c_ambiq_data *data = dev->data;
	int ret;
	am_hal_sysctrl_power_state_e status;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		/* Move pins to active/default state */
		ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret < 0) {
			LOG_ERR("I2C pinctrl setup failed (%d)", ret);
			return ret;
		}
		status = AM_HAL_SYSCTRL_WAKE;
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		/* Move pins to sleep state */
		ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_SLEEP);
		if (ret == -ENOENT) {
			/* Warn but don't block suspend */
			LOG_WRN("I2C pinctrl sleep state not available ");
		} else if (ret < 0) {
			return ret;
		}
		status = AM_HAL_SYSCTRL_DEEPSLEEP;
		break;
	default:
		return -ENOTSUP;
	}

	ret = am_hal_iom_power_ctrl(data->iom_handler, status, true);

	if (ret != AM_HAL_STATUS_SUCCESS) {
		return -EPERM;
	} else {
		return 0;
	}
}
#endif /* CONFIG_PM_DEVICE */

#define IOM_HAL_CFG(n, cmdq, cmdq_size)                                                            \
	{                                                                                          \
		.eInterfaceMode     = AM_HAL_IOM_I2C_MODE,                                         \
		.ui32ClockFreq      = AM_HAL_IOM_100KHZ,                                           \
		.pNBTxnBuf          = cmdq,                                                        \
		.ui32NBTxnBufLength = cmdq_size,                                                   \
	}

#define AMBIQ_I2C_DEFINE(n)                                                                        \
	BUILD_ASSERT(DT_CHILD_NUM_STATUS_OKAY(DT_INST_PARENT(n)) == 1,                             \
		     "Too many children for IOM, either SPI or I2C should be enabled!");           \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static void i2c_irq_config_func_##n(void)                                                  \
	{                                                                                          \
		IRQ_CONNECT(DT_IRQN(DT_INST_PARENT(n)), DT_IRQ(DT_INST_PARENT(n), priority),       \
			    i2c_ambiq_isr, DEVICE_DT_INST_GET(n), 0);                              \
		irq_enable(DT_IRQN(DT_INST_PARENT(n)));                                            \
	};                                                                                         \
	IF_ENABLED(DT_PROP(DT_INST_PARENT(n), dma_mode),                                           \
	(static uint32_t i2c_ambiq_cmdq##n[DT_PROP_OR(DT_INST_PARENT(n), cmdq_buffer_size, 1024)]  \
	 __attribute__((section(DT_PROP_OR(DT_INST_PARENT(n),                                      \
					  cmdq_buffer_location, ".nocache"))));)                   \
	)                                                                                          \
	static struct i2c_ambiq_data i2c_ambiq_data##n = {                                         \
		.iom_cfg = IOM_HAL_CFG(                                                            \
			n, COND_CODE_1(DT_PROP(DT_INST_PARENT(n), dma_mode), (i2c_ambiq_cmdq##n), \
									    (NULL)),               \
				COND_CODE_1(DT_PROP(DT_INST_PARENT(n), dma_mode),              \
					(DT_INST_PROP_OR(n, cmdq_buffer_size, 1024)), (0))),  \
		.dma_mode = DT_PROP(DT_INST_PARENT(n), dma_mode),     \
		.bus_sem = Z_SEM_INITIALIZER(i2c_ambiq_data##n.bus_sem, 1, 1),                     \
		.transfer_sem = Z_SEM_INITIALIZER(i2c_ambiq_data##n.transfer_sem, 0, 1),           \
	};                                                                                         \
	static const struct i2c_ambiq_config i2c_ambiq_config##n = {                               \
		.base = DT_REG_ADDR(DT_INST_PARENT(n)),                                            \
		.size = DT_REG_SIZE(DT_INST_PARENT(n)),                                            \
		.inst_idx =                                                                        \
			(DT_REG_ADDR(DT_INST_PARENT(n)) - IOM0_BASE) / (IOM1_BASE - IOM0_BASE),    \
		.bitrate = DT_INST_PROP(n, clock_frequency),                                       \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.irq_config_func = i2c_irq_config_func_##n,                                        \
		IF_ENABLED(CONFIG_I2C_AMBIQ_BUS_RECOVERY,                                          \
		(.scl = GPIO_DT_SPEC_INST_GET_OR(n, scl_gpios, {0}),                               \
		 .sda = GPIO_DT_SPEC_INST_GET_OR(n, sda_gpios, {0}),)) };                          \
	PM_DEVICE_DT_INST_DEFINE(n, i2c_ambiq_pm_action);                                          \
	I2C_DEVICE_DT_INST_DEFINE(n, i2c_ambiq_init, PM_DEVICE_DT_INST_GET(n), &i2c_ambiq_data##n, \
				  &i2c_ambiq_config##n, POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,     \
				  &i2c_ambiq_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AMBIQ_I2C_DEFINE)
