/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT xlnx_zynq_xadc_ps

#include <errno.h>
#include <string.h>
#include <zephyr/drivers/sensor/zynq_xadc_ps.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>

/* Offsets from devcfg: XADCIF itself starts at 0x100. */
#define UNLOCK     0x034U
#define CFG        0x100U
#define INT_MASK   0x108U
#define STATUS     0x10cU
#define CMD        0x110U
#define DATA       0x114U
#define CONTROL    0x118U
#define CMD_FULL   BIT(11)
#define DATA_EMPTY BIT(8)
#define DRP_READ   BIT(26)
#define DRP_WRITE  BIT(27)
#define POLL_US    1000U
#define CHANNELS   21U

struct zynq_xadc_config {
	DEVICE_MMIO_ROM;
};

struct zynq_xadc_data {
	DEVICE_MMIO_RAM;
	struct k_mutex lock;
	uint16_t raw[CHANNELS];
	int sample_error;
};

static const uint8_t drp_registers[CHANNELS] = {
	0x00, 0x01, 0x02, 0x06, 0x0d, 0x0e, 0x0f, 0x20, 0x21, 0x22, 0x23,
	0x28, 0x29, 0x2a, 0x24, 0x25, 0x26, 0x27, 0x2c, 0x2d, 0x2e,
};

static int wait_clear(mm_reg_t base, uint32_t mask)
{
	for (uint32_t i = 0; i < POLL_US; i++) {
		if ((sys_read32(base + STATUS) & mask) == 0U) {
			return 0;
		}
		k_busy_wait(1);
	}
	return -ETIMEDOUT;
}

/* Every command produces a FIFO response, including writes and NOPs. */
static int exchange(mm_reg_t base, uint32_t command, uint16_t *reply)
{
	int rc = wait_clear(base, CMD_FULL);

	if (rc != 0) {
		return rc;
	}
	sys_write32(command, base + CMD);
	rc = wait_clear(base, DATA_EMPTY);
	if (rc == 0) {
		*reply = sys_read32(base + DATA);
	}
	return rc;
}

static int drp_read(mm_reg_t base, uint8_t reg, uint16_t *value)
{
	uint16_t discarded;
	int rc = exchange(base, DRP_READ | ((uint32_t)reg << 16), &discarded);

	/* The read result is returned by the next command in the DRP pipeline. */
	return rc == 0 ? exchange(base, 0, value) : rc;
}

static int drp_write(mm_reg_t base, uint8_t reg, uint16_t value)
{
	uint16_t discarded;

	return exchange(base, DRP_WRITE | ((uint32_t)reg << 16) | value, &discarded);
}

static int channel_index(enum sensor_channel channel)
{
	if (channel == SENSOR_CHAN_DIE_TEMP) {
		return 0;
	}
	int index = (int)channel - SENSOR_CHAN_ZYNQ_XADC_TEMP;

	return index >= 0 && index < (int)CHANNELS ? index : -ENOTSUP;
}

static int zynq_xadc_fetch(const struct device *dev, enum sensor_channel channel)
{
	struct zynq_xadc_data *data = dev->data;
	uint16_t raw[CHANNELS];
	int rc = 0;

	if (channel != SENSOR_CHAN_ALL && channel_index(channel) < 0) {
		return -ENOTSUP;
	}
	k_mutex_lock(&data->lock, K_FOREVER);
	/* After a FIFO timeout its pipeline is indeterminate: require reinit. */
	if (data->sample_error == -ETIMEDOUT) {
		rc = data->sample_error;
	} else {
		for (size_t i = 0; i < CHANNELS; i++) {
			rc = drp_read(DEVICE_MMIO_GET(dev), drp_registers[i], &raw[i]);
			if (rc != 0) {
				break;
			}
		}
	}
	if (rc == 0) {
		memcpy(data->raw, raw, sizeof(raw));
	}
	data->sample_error = rc;
	k_mutex_unlock(&data->lock);
	return rc;
}

static int zynq_xadc_get(const struct device *dev, enum sensor_channel channel,
			 struct sensor_value *value)
{
	struct zynq_xadc_data *data = dev->data;
	int index = channel_index(channel);
	int64_t micro;
	int rc;

	if (index < 0) {
		return index;
	}
	k_mutex_lock(&data->lock, K_FOREVER);
	rc = data->sample_error;
	if (rc == 0) {
		uint16_t adc = data->raw[index] >> 4;

		micro = index % 7 == 0 ? (int64_t)adc * 503975000 / 4096 - 273150000
				       : (int64_t)adc * 3000000 / 4096;
		value->val1 = micro / 1000000;
		value->val2 = micro % 1000000;
	}
	k_mutex_unlock(&data->lock);
	return rc;
}

static int zynq_xadc_init(const struct device *dev)
{
	struct zynq_xadc_data *data = dev->data;
	mm_reg_t base;
	int rc;
	/* Safe mode while programming, calibrated internal sensors only.
	 * PCAP <= 200 MHz / 16 / 8 gives ADCCLK <= 1.5625 MHz (UG480).
	 */
	static const struct {
		uint8_t reg;
		uint16_t value;
	} setup[] = {
		{0x41, 0x0f0f}, {0x40, 0}, {0x42, 0x0800}, {0x48, 0x47e1},
		{0x49, 0},      {0x4a, 0}, {0x4b, 0},      {0x4c, 0},
		{0x4d, 0},      {0x4e, 0}, {0x4f, 0},      {0x41, 0x2faf},
	};

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	base = DEVICE_MMIO_GET(dev);
	k_mutex_init(&data->lock);
	data->sample_error = -ENODATA;
	sys_write32(0x757bdf0d, base + UNLOCK);
	sys_write32(BIT(4), base + CONTROL);
	sys_write32(0, base + CONTROL);
	sys_write32(0xffffffff, base + INT_MASK);
	/* PS enable, both edges, TCK /16, 20-cycle idle gap. */
	sys_write32(0x80003314, base + CFG);
	for (size_t i = 0; i < ARRAY_SIZE(setup); i++) {
		rc = drp_write(base, setup[i].reg, setup[i].value);
		if (rc != 0) {
			data->sample_error = rc;
			return rc;
		}
	}
	return 0;
}

static DEVICE_API(sensor, zynq_xadc_api) = {
	.sample_fetch = zynq_xadc_fetch,
	.channel_get = zynq_xadc_get,
};

#define ZYNQ_XADC_DEFINE(inst)                                                                     \
	static struct zynq_xadc_data zynq_xadc_data_##inst;                                        \
	static const struct zynq_xadc_config zynq_xadc_config_##inst = {                           \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(inst)),                                           \
	};                                                                                         \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, zynq_xadc_init, NULL, &zynq_xadc_data_##inst,           \
				     &zynq_xadc_config_##inst, POST_KERNEL,                        \
				     CONFIG_SENSOR_INIT_PRIORITY, &zynq_xadc_api);
DT_INST_FOREACH_STATUS_OKAY(ZYNQ_XADC_DEFINE)
