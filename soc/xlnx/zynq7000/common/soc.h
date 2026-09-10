/*
 * Copyright (c) 2021 Weidmueller Interface GmbH & Co. KG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SOC__H_
#define _SOC__H_

#ifndef _ASMLANGUAGE

/*
 * The following definitions are required for the inclusion of the CMSIS
 * Common Peripheral Access Layer for aarch32 Cortex-A CPUs:
 */

#define __CORTEX_A 9U

#include <stdint.h>

/* ARM PL310 (L2C-310) outer cache controller register window. */
#define ZYNQ_PL310_BASE 0xF8F02000U

#ifdef CONFIG_SOC_XLNX_ZYNQ7000_L2_CACHE
extern uintptr_t zynq_pl310_base;
void zynq_pl310_init(uintptr_t base);
#endif

#endif /* !_ASMLANGUAGE */

#endif /* _SOC__H_ */
