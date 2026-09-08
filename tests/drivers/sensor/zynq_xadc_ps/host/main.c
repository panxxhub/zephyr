/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0 */
#include "platform.h"
#include <stdio.h>
#include "../../../../../drivers/sensor/zynq_xadc_ps/zynq_xadc_ps.c"

static uint16_t regs[128], pending, response;
static bool response_ready, full, stalled;
static unsigned int commands, reads, stall_at;
static uint32_t cfg, mask, unlock;

static uint32_t sys_read32(uintptr_t address)
{
	assert(address >= 0xf8007000U && address <= 0xf8007118U);
	switch (address - 0xf8007000U) {
	case 0x10c:
		return (full ? BIT(11) : 0) | (response_ready ? 0 : BIT(8));
	case 0x114:
		assert(response_ready);
		response_ready = false;
		reads++;
		return response;
	default:
		assert(false);
		return 0;
	}
}
static void sys_write32(uint32_t value, uintptr_t address)
{
	unsigned int reg = (value >> 16) & 0x7f;
	switch (address - 0xf8007000U) {
	case 0x34:
		unlock = value;
		break;
	case 0x100:
		cfg = value;
		break;
	case 0x108:
		mask = value;
		break;
	case 0x118:
		if (value == BIT(4)) {
			pending = 0;
			response_ready = false;
		}
		break;
	case 0x110:
		assert(!full && !response_ready);
		commands++;
		response = pending;
		if ((value & (0xfU << 26)) == BIT(26)) {
			pending = regs[reg];
		} else if ((value & (0xfU << 26)) == BIT(27)) {
			regs[reg] = value;
			pending = 0;
		} else {
			assert(value == 0);
			pending = 0;
		}
		response_ready = !(stalled || (stall_at != 0 && commands == stall_at));
		break;
	default:
		assert(false);
	}
}
static int64_t micro(struct sensor_value v)
{
	return (int64_t)v.val1 * 1000000 + v.val2;
}
int main(void)
{
	struct zynq_xadc_config config = {.address = 0xf8007000U};
	struct zynq_xadc_data data = {0};
	struct device dev = {.config = &config, .data = &data};
	struct sensor_value v = {0};
	assert(zynq_xadc_init(&dev) == 0);
	assert(unlock == 0x757bdf0d && cfg == 0x80003314 && mask == 0xffffffff);
	assert(regs[0x41] == 0x2faf && regs[0x48] == 0x47e1 && regs[0x49] == 0);
	assert(regs[0x42] == 0x0800);
	assert(commands == reads); /* write acknowledgements cannot accumulate */
	assert(zynq_xadc_api.channel_get(&dev, SENSOR_CHAN_DIE_TEMP, &v) == -ENODATA);
	const uint8_t expected[] = {0,  1,  2,  6,  13, 14, 15, 32, 33, 34, 35,
				    40, 41, 42, 36, 37, 38, 39, 44, 45, 46};
	for (unsigned int i = 0; i < 128; i++) {
		regs[i] = (i * 29 + 100) << 4;
	}
	assert(zynq_xadc_api.sample_fetch(&dev, SENSOR_CHAN_ALL) == 0);
	for (unsigned int i = 0; i < 21; i++) {
		assert(zynq_xadc_get(&dev, (enum sensor_channel)(SENSOR_CHAN_PRIV_START + i), &v) ==
		       0);
		int64_t adc = expected[i] * 29 + 100;
		assert(micro(v) ==
		       (i % 7 == 0 ? adc * 503975000 / 4096 - 273150000 : adc * 3000000 / 4096));
	}
	/* 12-bit ADC boundaries and unused low nibble; negative temperatures. */
	const uint16_t codes[] = {0x000f, 0x800f, 0xffff};
	const int64_t temp[] = {-273150000, -21162500, 230701959};
	const int64_t supply[] = {0, 1500000, 2999267};
	for (unsigned int i = 0; i < 3; i++) {
		regs[0] = regs[1] = codes[i];
		assert(zynq_xadc_fetch(&dev, SENSOR_CHAN_DIE_TEMP) == 0);
		assert(zynq_xadc_get(&dev, SENSOR_CHAN_DIE_TEMP, &v) == 0 && micro(v) == temp[i]);
		assert(zynq_xadc_get(&dev, (enum sensor_channel)SENSOR_CHAN_ZYNQ_XADC_VCCINT, &v) ==
			       0 &&
		       micro(v) == supply[i]);
	}
	assert(zynq_xadc_get(&dev, (enum sensor_channel)99, &v) == -ENOTSUP);
	assert(zynq_xadc_fetch(&dev, (enum sensor_channel)121) == -ENOTSUP);
	unsigned int before = commands;
	full = true;
	waits = 0;
	assert(zynq_xadc_fetch(&dev, SENSOR_CHAN_ALL) == -ETIMEDOUT);
	assert(waits == 1000 && commands == before);
	assert(zynq_xadc_get(&dev, SENSOR_CHAN_DIE_TEMP, &v) == -ETIMEDOUT);
	full = false;
	assert(zynq_xadc_init(&dev) == 0);
	assert(zynq_xadc_fetch(&dev, SENSOR_CHAN_ALL) == 0);
	uint16_t old = data.raw[0];
	regs[0] = 123;
	stall_at = commands + 4; /* actual result of the second channel */
	assert(zynq_xadc_fetch(&dev, SENSOR_CHAN_ALL) == -ETIMEDOUT);
	assert(data.raw[0] == old); /* no partially updated sample */
	assert(zynq_xadc_get(&dev, SENSOR_CHAN_DIE_TEMP, &v) == -ETIMEDOUT);
	before = commands;
	assert(zynq_xadc_fetch(&dev, SENSOR_CHAN_ALL) == -ETIMEDOUT && commands == before);
	stall_at = 0;
	stalled = true;
	assert(zynq_xadc_init(&dev) == -ETIMEDOUT);
	puts("PS XADC: sequencer, DRP pipeline, all channels/extrema, conversions and failure "
	     "cases PASS");
	return 0;
}
