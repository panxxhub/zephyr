/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */
static uint64_t burst_epoch;
void phy_burst_init(void)
{
	boot(1000);
	assert(ready && bad_addresses == 0);
	assert(mc_yt8531_link_cb_set(&phy_dev, link_callback, NULL) == 0);
	phy_regs[YTPHY_SPECIFIC_STATUS_REG] = YTPHY_SSR_LINK | PHY_SPEED_1000M | PHY_DUPLEX_FULL;
	phy_data.monitor_work.work.handler(&phy_data.monitor_work.work);
	assert(carrier && observed.speed == LINK_FULL_1000BASE);
	burst_epoch = now_us;
}
uint32_t phy_burst_poll(uint64_t elapsed_us)
{
	now_us = burst_epoch + elapsed_us;
	uint64_t start = now_us;

	phy_data.monitor_work.work.handler(&phy_data.monitor_work.work);
	assert(carrier && observed.speed == LINK_FULL_1000BASE && callbacks == 2);
	assert(phy_data.lock.depth == 0 && mdio_data.lock.depth == 0);
	return now_us - start;
}
