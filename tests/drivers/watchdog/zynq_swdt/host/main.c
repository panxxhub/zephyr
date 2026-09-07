/*
 * Copyright (c) 2026 pan
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <string.h>
#include "../../../../../drivers/watchdog/wdt_xlnx_zynq.c"

int main(void)
{
	struct zynq_wdt_config config = {.address = 0xf8005000, .clock_frequency = 111111114};
	struct zynq_wdt_data data = {0};
	struct device dev = {.config = &config, .data = &data};
	struct wdt_timeout_cfg timeout = {.window = {0, 30000}, .flags = WDT_FLAG_RESET_SOC};

	assert(!zynq_wdt_init(&dev));
	assert(!writes);
	assert(zynq_wdt_api.feed(&dev, 0) == -EINVAL);
	assert(zynq_wdt_api.setup(&dev, 0) == -EINVAL);
	assert(!writes);
	timeout.window.min = 1;
	assert(zynq_wdt_api.install_timeout(&dev, &timeout) == -EINVAL);
	timeout.window.min = 0;
	timeout.window.max = 0;
	assert(zynq_wdt_api.install_timeout(&dev, &timeout) == -EINVAL);
	timeout.window.max = UINT32_MAX;
	assert(zynq_wdt_api.install_timeout(&dev, &timeout) == -EINVAL);
	timeout.window.max = 30000;
	timeout.flags = 0;
	assert(zynq_wdt_api.install_timeout(&dev, &timeout) == -ENOTSUP);
	timeout.flags = WDT_FLAG_RESET_SOC;
	assert(!zynq_wdt_api.install_timeout(&dev, &timeout));
	assert(!writes);
	assert(zynq_wdt_api.install_timeout(&dev, &timeout) == -ENOMEM);
	assert(zynq_wdt_api.setup(&dev, 1) == -ENOTSUP);
	assert(!writes);
	assert(!zynq_wdt_api.setup(&dev, 0));
	assert(writes == 3);
	assert(addresses[0] == 0xf8005004 && values[0] == data.ccr);
	assert(addresses[1] == 0xf8005000 &&
	       values[1] == (ZKEY | ENABLE | RESET_ENABLE | RESET_LENGTH));
	assert(addresses[2] == 0xf8005008 && values[2] == RESTART_KEY);
	assert(zynq_wdt_api.setup(&dev, 0) == -EBUSY);
	assert(zynq_wdt_api.install_timeout(&dev, &timeout) == -EBUSY);
	assert(zynq_wdt_api.feed(&dev, -1) == -EINVAL);
	assert(zynq_wdt_api.feed(&dev, 1) == -EINVAL);
	assert(writes == 3);
	assert(!zynq_wdt_api.feed(&dev, 0));
	assert(writes == 4);
	assert(zynq_wdt_api.disable(&dev) == -EPERM);
	assert(writes == 4);
	/* Verify rounding and all prescalers against the hardware count equation. */
	const unsigned int divs[] = {8, 64, 512, 4096};
	const unsigned int times[] = {1, 2, 100, 1000, 30000, 600000};

	for (unsigned int i = 0; i < ARRAY_SIZE(times); i++) {
		memset(&data, 0, sizeof(data));
		timeout.window.max = times[i];
		assert(!zynq_wdt_api.install_timeout(&dev, &timeout));
		unsigned int ps = data.ccr & 3, crv = (data.ccr >> 2) & 0xfff;
		uint64_t actual = (uint64_t)(crv + 1) * 4096 * divs[ps];
		uint64_t requested =
			DIV_ROUND_UP((uint64_t)config.clock_frequency * times[i], 1000);

		assert(actual >= requested && actual - requested < (uint64_t)4096 * divs[ps]);
	}
	puts("Zynq SWDT adoption, MMIO sequencing, API reverse cases and timing passed");
}
