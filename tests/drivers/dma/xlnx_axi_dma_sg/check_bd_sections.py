# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0
"""Check actual ring placement in every DT-instantiated Twister build."""

import argparse
from pathlib import Path

from elftools.elf.elffile import ELFFile


def check(path, reverse=False):
    """Require both rings to agree with the selected memory policy."""
    with path.open("rb") as stream:
        elf = ELFFile(stream)
        symbols = {
            symbol.name: symbol for symbol in elf.get_section_by_name(".symtab").iter_symbols()
        }
        if "dma_xlnx_sg_tx_bds_0" not in symbols:
            return False
        config = (path.parent / ".config").read_text()
        nocache = "CONFIG_DMA_XLNX_AXI_DMA_SG_NOCACHE_BD=y" in config
        start = symbols.get("_nocache_ram_start")
        end = symbols.get("_nocache_ram_end")
        for channel in ("tx", "rx"):
            symbol = symbols[f"dma_xlnx_sg_{channel}_bds_0"]
            addr = symbol["st_value"]
            inside = bool(
                start is not None
                and end is not None
                and start["st_value"] <= addr
                and addr + symbol["st_size"] <= end["st_value"]
            )
            assert addr % 64 == 0 and symbol["st_size"] > 0, path
            assert inside == (nocache != reverse), (path, channel, nocache, inside)
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("twister_out", type=Path)
    args = parser.parse_args()
    count = 0
    for path in args.twister_out.rglob("zephyr.elf"):
        if check(path):
            count += 1
            try:
                check(path, reverse=True)
            except AssertionError:
                print(f"{path.parent.parent.name}: PASS; reversed placement policy: RED")
            else:
                raise AssertionError(f"reverse placement policy passed: {path}")
    assert count > 0, "no DT-instantiated driver builds found"


if __name__ == "__main__":
    main()
