#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0
"""Compile the production sensor driver against a pipelined DRP fake."""
import subprocess
import tempfile
from pathlib import Path

root = Path(__file__).resolve().parent
repo = root.parents[4]
scratch = repo / ".scratch"
scratch.mkdir(exist_ok=True)
with tempfile.TemporaryDirectory(dir=scratch) as directory:
    build = Path(directory)
    for name in ("kernel.h", "drivers/sensor.h", "sys/device_mmio.h", "sys/sys_io.h"):
        header = build / "zephyr" / name
        header.parent.mkdir(parents=True, exist_ok=True)
        header.write_text('#include "platform.h"\n')
    binary = build / "test"
    subprocess.run([
        "cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-g",
        "-fsanitize=address,undefined", "-fno-pie", "-no-pie",
        "-I" + str(root), "-I" + str(build), "-I" + str(repo / "include"),
        str(root / "main.c"), "-o", str(binary),
    ], check=True)
    subprocess.run([str(binary)], check=True)
