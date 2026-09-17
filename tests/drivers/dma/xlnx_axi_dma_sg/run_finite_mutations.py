# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0
"""Run finite RX falsification gates against isolated copies of the driver."""

import argparse
import subprocess

import run_placement_mutations as runner
from test_rx_ownership import function

runner.MUTATIONS = {
    "small transfer callback changed": ("ring_count <= 255U || ch->error ||", "ch->error ||"),
    "rescan completed prefix": (
        "for (uint32_t i = ch->consumer_idx; i < ring_count; i++)",
        "for (uint32_t i = 0U; i < ring_count; i++)",
    ),
    "skip incomplete descriptor": (
        "if ((ch->bds[i].status & BD_STS_CMPLT) == 0U) {\n\t\t\tbreak;",
        "if ((ch->bds[i].status & BD_STS_CMPLT) == 0U) {\n\t\t\tcontinue;",
    ),
    "overwrite accumulated bytes": ("total_bytes += bytes;", "total_bytes = bytes;"),
    "premature completion": (
        "(ch->consumer_idx == ring_count && first < ring_count)",
        "(first < ring_count)",
    ),
    "completion repeated for pending IOC": (
        "ch->consumer_idx == ring_count && first < ring_count",
        "ch->consumer_idx == ring_count",
    ),
    "ignore complete ring on first IOC": (
        "ch->consumer_idx == ring_count && first < ring_count",
        "ch->consumer_idx == ring_count && first > 0U && first < ring_count",
    ),
    "one interrupt per BD": ("ch->hw_irq_threshold = 255U;", "ch->hw_irq_threshold = 1U;"),
    "park at intermediate tail": (
        "tail_idx = ch->active_bds - 1;",
        "tail_idx = MIN(ch->active_bds, 255U) - 1U;",
    ),
    "no remainder reload": (
        "while (remaining > 0U && remaining < ch->hw_irq_threshold)",
        "while (false)",
    ),
    "only one reload despite partial race": (
        "while (remaining > 0U && remaining < ch->hw_irq_threshold)",
        "if (remaining > 0U && remaining < ch->hw_irq_threshold)",
    ),
    "no harvest after reload": (
        "\t\t\t\t\trx_finite_harvest(dev, ring_count);",
        "\t\t\t\t\t/* Harvest omitted. */",
    ),
    "recount ring after callback stops RX": (
        "(chan_read(dev, CH_RX, REG_DMACR) & DMACR_RS) == 0U",
        "false",
    ),
    "small legacy delay suppressed": (
        "count > 255U || !DEV_CFG(dev)->rx_invalidate_in_isr",
        "true",
    ),
    "small delegated delay retained": (
        "count > 255U || !DEV_CFG(dev)->rx_invalidate_in_isr",
        "count > 255U",
    ),
    "large legacy delay restored": (
        "count > 255U || !DEV_CFG(dev)->rx_invalidate_in_isr",
        "!DEV_CFG(dev)->rx_invalidate_in_isr",
    ),
    "delay enabled": ("ch->irq_timeout = 0U;", "ch->irq_timeout = 16U;"),
}

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument(
        "--reverse-ref", action="append", default=[], help="also test a driver revert; repeatable"
    )
    args = parser.parse_args()
    source = runner.DRIVER.read_text()
    for name in ("do_soft_reset", "dma_xlnx_sg_config", "dma_xlnx_sg_start", "dma_xlnx_sg_stop"):
        body = function(source, name)
        changed = body.replace("consumer_idx = 0U;", "consumer_idx = 7U;")
        changed = changed.replace("consumer_idx = 0;", "consumer_idx = 7U;")
        assert changed != body, name
        runner.MUTATIONS[f"retain harvest index in {name}"] = (body, changed)
    start = function(source, "dma_xlnx_sg_start")
    runner.MUTATIONS["retain short threshold on restart"] = (
        start,
        start.replace("ch->hw_irq_threshold = 255U;", "ch->hw_irq_threshold = 7U;"),
    )
    for ref in args.reverse_ref:
        old = subprocess.check_output(
            ["git", "show", f"{ref}:drivers/dma/dma_xlnx_axi_dma_sg.c"],
            cwd=runner.ROOT,
            text=True,
        )
        runner.MUTATIONS[f"complete driver revert to {ref}"] = (source, old)
    runner.main()
