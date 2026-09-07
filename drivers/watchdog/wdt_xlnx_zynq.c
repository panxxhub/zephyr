/* SPDX-License-Identifier: Apache-2.0 */
#define DT_DRV_COMPAT xlnx_zynq_swdt

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>
#include <errno.h>

#define ZMR          0x00
#define CCR          0x04
#define RESTART      0x08
#define ZKEY         0xabc000U
#define CKEY         0x920000U
#define RESTART_KEY  0x1999U
#define ENABLE       BIT(0)
#define RESET_ENABLE BIT(1)
#define RESET_LENGTH (7U << 4)

struct zynq_wdt_config {
	DEVICE_MMIO_ROM;
	uint32_t clock_frequency;
};
struct zynq_wdt_data {
	DEVICE_MMIO_RAM;
	struct k_spinlock lock;
	uint32_t ccr;
	bool installed;
	bool setup;
};

static int zynq_wdt_install(const struct device *dev, const struct wdt_timeout_cfg *cfg)
{
	const struct zynq_wdt_config *config = dev->config;
	struct zynq_wdt_data *data = dev->data;
	static const uint32_t divisors[] = {8, 64, 512, 4096};
	uint64_t cycles = DIV_ROUND_UP((uint64_t)config->clock_frequency * cfg->window.max, 1000);
	uint64_t count = 0;
	unsigned int prescale;

	if (cfg->callback || cfg->flags != WDT_FLAG_RESET_SOC) {
		return -ENOTSUP;
	}
	if (cfg->window.min || !cfg->window.max) {
		return -EINVAL;
	}
	for (prescale = 0; prescale < ARRAY_SIZE(divisors); prescale++) {
		count = DIV_ROUND_UP(cycles, (uint64_t)divisors[prescale] * 4096);
		if (count <= 4096) {
			break;
		}
	}
	if (prescale == ARRAY_SIZE(divisors)) {
		return -EINVAL;
	}
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	int rc = data->setup ? -EBUSY : (data->installed ? -ENOMEM : 0);

	if (!rc) {
		data->ccr = CKEY | ((uint32_t)(count - 1) << 2) | prescale;
		data->installed = true;
	}
	k_spin_unlock(&data->lock, key);
	return rc;
}

static int zynq_wdt_setup(const struct device *dev, uint8_t options)
{
	struct zynq_wdt_data *data = dev->data;
	mm_reg_t base = DEVICE_MMIO_GET(dev);

	if (options) {
		return -ENOTSUP;
	}
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	int rc = data->setup ? -EBUSY : (!data->installed ? -EINVAL : 0);

	if (!rc) {
		/* Adopt an FSBL-armed timer without ever disabling reset coverage. */
		sys_write32(data->ccr, base + CCR);
		sys_write32(ZKEY | ENABLE | RESET_ENABLE | RESET_LENGTH, base + ZMR);
		sys_write32(RESTART_KEY, base + RESTART);
		data->setup = true;
	}
	k_spin_unlock(&data->lock, key);
	return rc;
}

static int zynq_wdt_feed(const struct device *dev, int channel)
{
	struct zynq_wdt_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	int rc = channel != 0 || !data->setup ? -EINVAL : 0;

	if (!rc) {
		sys_write32(RESTART_KEY, DEVICE_MMIO_GET(dev) + RESTART);
	}
	k_spin_unlock(&data->lock, key);
	return rc;
}

static int zynq_wdt_disable(const struct device *dev)
{
	ARG_UNUSED(dev);
	/* System recovery remains armed for the entire boot, including updates. */
	return -EPERM;
}

static int zynq_wdt_init(const struct device *dev)
{
	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	/* Initialization must neither feed nor stop the FSBL watchdog. */
	return 0;
}

static DEVICE_API(wdt, zynq_wdt_api) = {
	.install_timeout = zynq_wdt_install,
	.setup = zynq_wdt_setup,
	.feed = zynq_wdt_feed,
	.disable = zynq_wdt_disable,
};

#define ZYNQ_WDT_DEFINE(inst)                                                                      \
	static struct zynq_wdt_data zynq_wdt_data_##inst;                                          \
	static const struct zynq_wdt_config zynq_wdt_config_##inst = {                             \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(inst)),                                           \
		.clock_frequency = DT_INST_PROP(inst, clock_frequency),                            \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, zynq_wdt_init, NULL, &zynq_wdt_data_##inst,                    \
			      &zynq_wdt_config_##inst, PRE_KERNEL_1,                               \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &zynq_wdt_api);
DT_INST_FOREACH_STATUS_OKAY(ZYNQ_WDT_DEFINE)
