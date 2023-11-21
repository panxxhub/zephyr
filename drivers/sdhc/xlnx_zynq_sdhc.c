#include "zephyr/drivers/sdhc.h"
#define DT_DRV_COMPAT xlnx_zynq_sdhc

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/sd/sd_spec.h>
#include <zephyr/logging/log.h>
#include <soc.h>

LOG_MODULE_REGISTER(sdhc, CONFIG_SDHC_LOG_LEVEL);

/**
 * @brief RESET the SDHC controller
 *
 * @param dev
 * @return int
 */

/*
static int zynq_sdhc_reset(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static int zynq_sdhc_request(const struct device *dev, struct sdhc_command *cmd, struct sdhc_data *data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cmd);
	ARG_UNUSED(data);
	return 0;
}

static int zynq_sdhc_set_io(const struct device *dev, struct sdhc_io *ios)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(ios);
	return 0;
}

static int zynq_sdhc_get_card_present(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static int zynq_sdhc_execute_tuning(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static int zynq_sdhc_card_busy(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static int zynq_sdhc_get_host_props(const struct device *dev, struct sdhc_host_props *props)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(props);
	return 0;
}

static int zynq_sdhc_enable_interrupt(const struct device *dev, sdhc_interrupt_cb_t callback, int sources,
				      void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(callback);
	ARG_UNUSED(sources);
	ARG_UNUSED(user_data);
	return 0;
}

static int zynq_sdhc_disable_interrupt(const struct device *dev, int sources)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(sources);
	return 0;
}


static const struct sdhc_driver_api zynq_sdhc_api = {
	.reset = zynq_sdhc_reset,
	.request = zynq_sdhc_request,
	.set_io = zynq_sdhc_set_io,
	.get_card_present = zynq_sdhc_get_card_present,
	.execute_tuning = zynq_sdhc_execute_tuning,
	.card_busy = zynq_sdhc_card_busy,
	.get_host_props = zynq_sdhc_get_host_props,
	.enable_interrupt = zynq_sdhc_enable_interrupt,
	.disable_interrupt = zynq_sdhc_disable_interrupt,
};

#define ZYNQ_SDHC_INIT(n)                                                                                              \
	static int zynq_sdhc_##n##_init(const struct device *dev)                                                      \
	{                                                                                                              \
		ARG_UNUSED(dev);                                                                                       \
		return 0;                                                                                              \
	}                                                                                                              \
	DEVICE_DT_DEFINE(DT_NODELABEL(sdhc##n), zynq_sdhc_##n##_init, NULL, NULL, NULL, POST_KERNEL,                   \
			 CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &zynq_sdhc_api);

DT_INST_FOREACH_STATUS_OKAY(ZYNQ_SDHC_INIT)
*/