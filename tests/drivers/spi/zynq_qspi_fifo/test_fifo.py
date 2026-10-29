#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0
"""Execute the driver's FIFO code against the UG585 TXD width restriction."""

import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
source = (ROOT / "drivers/spi/spi_xlnx_zynq_qspi.c").read_text()
registers = source[source.index("#define QSPI_CR") : source.index("/* ── Structures")]
transfer = source[source.index("#define QSPI_FIFO_DEPTH") : source.index("/* ── Transceive")]
harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#define BIT(n) (1U << (n))
#define GENMASK(h, l) (((~0U) >> (31U - (h))) & ((~0U) << (l)))
#define LOG_ERR(...) (void)0
struct device { int unused; };
static uint32_t qspi_read(const struct device *, uint32_t);
static void qspi_write(const struct device *, uint32_t, uint32_t);
'''
model = r'''
static uint32_t fifo[63], received[63], cr;
static uint8_t widths[63], wire[252];
static unsigned int queued, available, next, on_wire, starts, stall_at;

static uint32_t qspi_read(const struct device *dev, uint32_t reg)
{
    (void)dev;
    if (reg == QSPI_CR) return cr;
    if (reg == QSPI_ISR) return next < available && starts != stall_at ? ISR_RX_NEMPTY : 0;
    assert(reg == QSPI_RXD && next < available && starts != stall_at);
    return received[next++];
}

static void qspi_write(const struct device *dev, uint32_t reg, uint32_t value)
{
    (void)dev;
    if (reg == QSPI_ISR) { assert(value == ISR_RX_NEMPTY); return; }
    if (reg == QSPI_CR) {
        cr = value & ~CR_MANSTARTCOM;
        if (!(value & CR_MANSTARTCOM)) return;
        assert(queued > 0 && next == available);
        starts++;
        available = queued;
        next = 0;
        for (unsigned int s = 0; s < queued; ++s) {
            received[s] = 0;
            for (unsigned int k = 0; k < widths[s]; ++k) {
                uint8_t byte = fifo[s] >> (8 * k);
                assert(on_wire < sizeof(wire));
                wire[on_wire++] = byte;
                received[s] |= (uint32_t)(byte ^ 0xa5) << (8 * (4 - widths[s] + k));
            }
        }
        queued = 0;
        return;
    }
    unsigned int width = reg == QSPI_TXD0 ? 4 : reg == QSPI_TXD1 ? 1 :
                         reg == QSPI_TXD2 ? 2 : reg == QSPI_TXD3 ? 3 : 0;
    assert(width && queued < 63);
    /* UG585: only TXD0 -> TXD0 may be queued before draining TxFIFO. */
    assert(queued == 0 || (width == 4 && widths[queued - 1] == 4));
    assert(next == available);
    fifo[queued] = value;
    widths[queued++] = width;
}

static void reset(unsigned int stall)
{
    queued = available = next = on_wire = starts = 0;
    stall_at = stall;
    cr = CR_PCS | CR_MANCS | CR_MANSTARTEN;
}

int main(void)
{
    const struct device dev = {0};
    uint8_t tx[252], rx[254];
    for (size_t i = 0; i < sizeof(tx); ++i) tx[i] = (i * 37 + 0x5a) & 255;
    for (size_t n = 1; n <= sizeof(tx); ++n) {
        for (unsigned int discard = 0; discard < 2; ++discard) {
            reset(0); memset(rx, 0x69, sizeof(rx));
            assert(zynq_qspi_burst_xfer(&dev, tx, discard ? NULL : rx + 1, n) == 0);
            assert(on_wire == n && memcmp(tx, wire, n) == 0);
            assert(queued == 0 && next == available);
            assert(rx[0] == 0x69 && rx[n + 1] == 0x69);
            for (size_t i = 0; i < n; ++i) assert(rx[i + 1] == (discard ? 0x69 : (tx[i] ^ 0xa5)));
            assert(cr == (CR_PCS | CR_MANCS | CR_MANSTARTEN));
            /* Keep aligned traffic batched; tails need at most one extra start. */
            assert(starts <= 2 && (n % 4 != 0 || starts == 1));
        }
    }
    reset(1);
    assert(zynq_qspi_burst_xfer(&dev, tx, rx, 5) == -ETIMEDOUT);
    assert(starts == 1 && on_wire == 4); /* failed word burst must not send tail */
    reset(2);
    assert(zynq_qspi_burst_xfer(&dev, tx, rx, 5) == -ETIMEDOUT);
    assert(starts == 2 && on_wire == 5);
    puts("QSPI FIFO: 504 length/discard cases and two timeout cases passed");
}
'''
with tempfile.TemporaryDirectory(prefix="zynq-qspi-fifo-") as tmp:
    path = Path(tmp)
    c_file = path / "test.c"
    c_file.write_text(harness + registers + transfer + model)
    binary = path / "test"
    subprocess.run(
        [
            "cc",
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-g",
            "-fsanitize=address,undefined",
            "-fno-pie",
            "-no-pie",
            str(c_file),
            "-o",
            str(binary),
        ],
        check=True,
    )
    subprocess.run([str(binary)], check=True)
