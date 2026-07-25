#!/usr/bin/env python3
#
# Copyright (c) 2026 Moton Technology Inc.
#
# SPDX-License-Identifier: Apache-2.0

"""Reject logging and callback calls reachable from the Xilinx ISR front ends."""

import re
import sys
from pathlib import Path

FUNCTION_CALL = re.compile(r"\b([A-Za-z_]\w*)\s*\(")
FORBIDDEN_CALL = re.compile(
    r"""
    \b(?:
        LOG_[A-Z0-9_]+
        |printk|vprintk|snprintk|vsnprintk
        |printf|vprintf|fprintf|vfprintf|puts|putchar
        |log_[A-Za-z0-9_]+|z_log_[A-Za-z0-9_]+
        |console_[A-Za-z0-9_]+
        |uart_poll_out|uart_fifo_fill
        |[A-Za-z_]\w*callback
    )\s*\(
    """,
    re.X,
)
CONTROL_WORDS = {"for", "if", "sizeof", "switch", "while"}


def sanitized_source(path: Path) -> str:
    source = path.read_text(encoding="utf-8")
    source = re.sub(r"/\*.*?\*/", lambda match: " " * len(match.group()), source, flags=re.S)
    source = re.sub(r"//[^\n]*", lambda match: " " * len(match.group()), source)
    source = re.sub(
        r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'',
        lambda match: " " * len(match.group()),
        source,
    )
    return source


def function_body(source: str, name: str) -> str:
    signature = re.search(rf"\b{re.escape(name)}\s*\([^;{{}}]*\)\s*\{{", source)
    if signature is None:
        raise ValueError(f"function not found: {name}")

    start = source.find("{", signature.start())
    depth = 0
    for index in range(start, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[start + 1 : index]

    raise ValueError(f"unterminated function: {name}")


def local_function_names(source: str) -> set[str]:
    names = set()
    for match in re.finditer(r"\b([A-Za-z_]\w*)\s*\([^;{}]*\)\s*\{", source):
        name = match.group(1)
        if name not in CONTROL_WORDS:
            names.add(name)
    return names


def assert_clean_callgraph(path: Path, roots: tuple[str, ...]) -> None:
    source = sanitized_source(path)
    local_functions = local_function_names(source)
    pending = list(roots)
    visited = set()

    while pending:
        name = pending.pop()
        if name in visited:
            continue
        visited.add(name)

        body = function_body(source, name)
        forbidden = FORBIDDEN_CALL.search(body)
        if forbidden is not None:
            raise ValueError(
                f"{path}:{name}: forbidden ISR-reachable call: {forbidden.group().strip()}"
            )

        for called in FUNCTION_CALL.findall(body):
            if called in local_functions and called not in visited:
                pending.append(called)


def main() -> int:
    if len(sys.argv) != 3:
        print(f"usage: {sys.argv[0]} DMA_SOURCE SDHC_SOURCE", file=sys.stderr)
        return 2

    try:
        assert_clean_callgraph(Path(sys.argv[1]), ("dma_xlnx_sg_tx_isr", "dma_xlnx_sg_rx_isr"))
        assert_clean_callgraph(Path(sys.argv[2]), ("zynq_sdhc_isr",))
    except ValueError as error:
        print(error, file=sys.stderr)
        return 1

    print(
        "Xilinx DMA/SDHC ISR callgraphs contain no logging, formatting, "
        "console output, or client callbacks"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
