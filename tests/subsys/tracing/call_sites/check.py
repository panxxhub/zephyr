#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0

"""Check actual linked switch/idle calls against tracing category settings."""

import argparse
import re
import subprocess
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
parser.add_argument("build", type=Path)
parser.add_argument("--objdump")
args = parser.parse_args()
if args.objdump is None:
    cache = (args.build / "CMakeCache.txt").read_text()
    match = re.search(r"^CMAKE_OBJDUMP:[^=]+=(.+)$", cache, re.MULTILINE)
    if match is None:
        raise SystemExit("CMAKE_OBJDUMP missing from build cache; pass --objdump")
    args.objdump = match[1]
config = (args.build / "zephyr/.config").read_text()
dump = subprocess.check_output(
    [args.objdump, "-d", str(args.build / "zephyr/zephyr.elf")], text=True
)
calls = re.findall(r"\b(?:blx?|bl?\.w|b\.n)\s+[0-9a-f]+\s+<([^>+]+)", dump)
for setting, targets in {
    "TRACING_THREAD": {
        "z_thread_mark_switched_in",
        "z_thread_mark_switched_out",
        "sys_trace_k_thread_switched_in",
        "sys_trace_k_thread_switched_out",
    },
    "TRACING_IDLE": {"sys_trace_idle"},
}.items():
    present = sorted(set(calls) & targets)
    expected = f"CONFIG_{setting}=y" in config
    if bool(present) != expected:
        raise SystemExit(f"{setting}: expected calls={expected}, found {present}")
    print(f"{setting}: {present}")
if "sys_trace_isr_enter" not in calls or "sys_trace_isr_exit" not in calls:
    raise SystemExit("ISR hooks missing: this is not an ISR tracing build")
print("ISR enter and exit calls present")
