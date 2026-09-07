#!/usr/bin/env python3
# Copyright (c) 2026 pan
# SPDX-License-Identifier: Apache-2.0
"""Compile the production driver against an MMIO recorder; no board access."""

import subprocess
import tempfile
from pathlib import Path

root = Path(__file__).resolve().parent
with tempfile.TemporaryDirectory() as directory:
    build = Path(directory)
    for name in ('kernel.h', 'device.h', 'drivers/watchdog.h', 'sys/device_mmio.h', 'sys/sys_io.h'):
        header = build / 'zephyr' / name
        header.parent.mkdir(parents=True, exist_ok=True)
        header.write_text('#include "platform.h"\n')
    binary = build / 'test'
    subprocess.run(
        [
            'cc',
            '-std=c11',
            '-Wall',
            '-Wextra',
            '-Werror',
            '-fsanitize=address,undefined',
            '-g',
            '-I' + str(root),
            '-I' + str(build),
            str(root / 'main.c'),
            '-o',
            str(binary),
        ],
        check=True,
    )
    subprocess.run([str(binary)], check=True)
