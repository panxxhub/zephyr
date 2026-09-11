#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0
"""Build the fake-MMIO tests with isolated driver mutations on native_sim."""

import os
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
TEST = Path(__file__).resolve().parent
DRIVER = ROOT / "drivers/dma/dma_xlnx_axi_dma_sg.c"
MUTATIONS = {
    "destination slots ignored": ("i % num_slots : i", "i : i"),
    "runway limited by destination slots": (
        "return MIN(lead, ch->active_bds);",
        "return ch->rx_num_slots != 0U ? ch->rx_num_slots / 2U : MIN(lead, ch->active_bds);",
    ),
    "slot admission ignored": ("if (cfg->num_slots != 0U &&", "if (false &&"),
    "slot count retained on restart": (
        "ch->rx_num_slots = cfg->num_slots;",
        "if (cfg->num_slots != 0U) { ch->rx_num_slots = cfg->num_slots; }",
    ),
    "stream ignores selected ring size": (
        "ch->active_bds = num_bds;",
        "ch->active_bds = ch->num_bds;",
    ),
    "stream shrinks finite pool": (
        "ch->active_bds = num_bds;",
        "ch->active_bds = num_bds; ch->num_bds = num_bds;",
    ),
    "stream wraps at pool end": (
        "idx = (idx + 1) % ch->active_bds;",
        "idx = (idx + 1) % ch->num_bds;",
    ),
    "stream uses pool runway": ("ch->active_bds / 2U", "ch->num_bds / 2U"),
    "error dump reads invisible payload": ("!(ch->cyclic && ch->rx_base_phys != 0U)", "true"),
    "fallback destination": ("phys = ch->rx_base_phys;", "phys = buf_phys(dev, channel);"),
    "packed stride": ("stride = ch->rx_stride;", "stride = ch->bd_buf_bytes;"),
    "payload invalidate": ("if (ch->rx_base_phys == 0U)", "if (true)"),
    "NULL callback suppressed": (
        "if (size > 0U && ch->rx_stream_callback != NULL)",
        "if (buf != NULL && size > 0U && ch->rx_stream_callback != NULL)",
    ),
    "wrong BD identity": ("buf, size, first_bd);", "buf, size, first_bd ^ 1U);"),
    "unaligned base accepted": ("!IS_ALIGNED(cfg->base_phys, 32U)", "false"),
    "unaligned stride accepted": ("!IS_ALIGNED(stride, 32U)", "false"),
    "overlapping destinations": ("stride < cfg->bd_bytes", "false"),
    "explicit base bounded by rx_buf": (
        "if (cfg->bd_bytes == 0U)",
        "if (cfg->bd_bytes == 0U || (size_t)cfg->bd_bytes * ch->num_bds > buf_size(dev, CH_RX))",
    ),
    "placement retained on restart": (
        "ch->rx_base_phys = cfg->base_phys;",
        "if (cfg->base_phys != 0U) { ch->rx_base_phys = cfg->base_phys; }",
    ),
}


def main():
    env = dict(os.environ, ZEPHYR_BASE=str(ROOT))
    original = DRIVER.read_text()
    with tempfile.TemporaryDirectory(prefix="xlnx-placement-") as directory:
        app = Path(directory)
        driver = app / DRIVER.name
        (app / "dma_xlnx_axi_dma_sg.h").write_text(DRIVER.with_suffix(".h").read_text())
        (app / "main.c").write_text(
            (TEST / "src/main.c")
            .read_text()
            .replace('../../../../../drivers/dma/dma_xlnx_axi_dma_sg.c', str(driver))
        )
        (app / "prj.conf").write_text((TEST / "prj.conf").read_text())
        (app / "CMakeLists.txt").write_text(
            (TEST / "CMakeLists.txt").read_text().replace("src/main.c", "main.c")
        )
        for name, change in [("baseline", None), *MUTATIONS.items()]:
            source = original
            if change is not None:
                old, new = change
                assert old in source, name
                source = source.replace(old, new)
            driver.write_text(source)
            build = subprocess.run(
                [
                    "west",
                    "-z",
                    str(ROOT),
                    "build",
                    "-b",
                    "native_sim",
                    "-d",
                    str(app / "build"),
                    str(app),
                ],
                env=env,
                capture_output=True,
                text=True,
            )
            assert build.returncode == 0, build.stdout + build.stderr
            result = subprocess.run(
                [str(app / "build/zephyr/zephyr.exe")],
                capture_output=True,
                text=True,
            )
            assert (result.returncode == 0) == (change is None), name + result.stdout
            print(f"{name}: {'PASS' if change is None else 'RED'}", flush=True)


if __name__ == "__main__":
    main()
