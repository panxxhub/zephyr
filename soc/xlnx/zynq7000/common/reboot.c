/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/devicetree.h>
#include <zephyr/drivers/cache/xlnx_zynq7000_pl310.h>
#include <zephyr/irq.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/sys_io.h>

#define SLCR_UNLOCK               0x0008U
#define SLCR_UNLOCK_KEY           0xdf0dU
#define SLCR_PSS_RST_CTRL         0x0200U
#define SLCR_PSS_RST_CTRL_SOFT_RST BIT(0)

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(slcr))
/* PS-only software reset reruns BootROM and the QSPI FSBL. */
void sys_arch_reboot(int type)
{
	const mem_addr_t slcr = DT_REG_ADDR(DT_NODELABEL(slcr));

	ARG_UNUSED(type);
	(void)irq_lock();
	if (IS_ENABLED(CONFIG_SOC_XLNX_ZYNQ7000_L2_CACHE)) {
		/* FSBL startup disables and invalidates L2 without preserving dirty lines. */
		zynq_pl310_shutdown();
	}
	sys_write32(SLCR_UNLOCK_KEY, slcr + SLCR_UNLOCK);
	barrier_dsync_fence_full();
	sys_write32(SLCR_PSS_RST_CTRL_SOFT_RST, slcr + SLCR_PSS_RST_CTRL);
	barrier_dsync_fence_full();
	for (;;) {
		barrier_isync_fence_full();
	}
}
#endif
