/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CACHE_XLNX_ZYNQ7000_PL310_H_
#define ZEPHYR_INCLUDE_DRIVERS_CACHE_XLNX_ZYNQ7000_PL310_H_

#include <stdint.h>

/**
 * @brief Initialize the shared PL310 after primary MMU/L1 setup, before SMP startup.
 * @param cpu Core index; secondary cores leave the controller unchanged.
 */
void zynq_pl310_init(uint32_t cpu);

/**
 * @brief Drain local L1, clean and invalidate shared L2, then disable PL310 for reset.
 *
 * Other CPUs and DMA producers must be quiescent. The caller must keep local
 * interrupts locked and reset immediately; normal cache users must not resume.
 * Local L1 must be enabled on entry for the controller lock's exclusive accesses.
 */
void zynq_pl310_shutdown(void);

/**
 * @brief Translate a privileged read address using the current MMU tables.
 *
 * The caller must serialize ATS1CPR/PAR against local interrupts.
 *
 * @param va Virtual address.
 * @param pa Destination for the translated physical address.
 * @retval 0 Translation succeeded.
 * @retval -EFAULT The MMU reported a translation fault.
 */
int zynq_pl310_virt_to_phys(uintptr_t va, uintptr_t *pa);

#endif
