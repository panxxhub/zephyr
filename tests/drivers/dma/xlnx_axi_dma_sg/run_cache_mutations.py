# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0
"""Falsify descriptor, consumer coverage, and outer-cache bypass contracts."""

import run_placement_mutations as runner

runner.MUTATIONS = {
    "descriptor cache maintenance restored": ("if (!ch->bds_nocache)", "if (true)"),
    "cacheable descriptor maintenance omitted": ("if (!ch->bds_nocache)", "if (false)"),
    "payload invalidation restored in ISR": (
        "if (DEV_CFG(dev)->rx_invalidate_in_isr)",
        "if (true)",
    ),
    "legacy ISR invalidation omitted": ("if (DEV_CFG(dev)->rx_invalidate_in_isr)", "if (false)"),
    "consumer slice unbounded": (
        "MIN(len, RX_INVALIDATE_SLICE - addr % RX_INVALIDATE_SLICE)",
        "len",
    ),
    "consumer skips first byte": ("addr += offset;", "addr += offset + 1U;"),
    "consumer overlaps slices": ("addr += slice;", "addr += slice - 1U;"),
    "consumer skips final byte": ("while (len > 0U)", "while (len > 1U)"),
    "consumer bounds omitted": (
        "offset > capacity || len > capacity - offset || offset > UINTPTR_MAX - addr",
        "false",
    ),
    "consumer accepts ISR callers": ("if (k_is_in_isr())", "if (false)"),
    "consumer ignores cache errors": (
        "if (ret != 0) {\n\t\t\treturn ret;\n\t\t}\n\t\taddr += slice;",
        "if (false) {\n\t\t\treturn ret;\n\t\t}\n\t\taddr += slice;",
    ),
    "outer cache bypass omitted": ("if (DEV_CFG(dev)->rx_outer_nocache)", "if (false)"),
    "outer cache bypass unconditional": ("if (DEV_CFG(dev)->rx_outer_nocache)", "if (true)"),
}

if __name__ == "__main__":
    runner.main()
