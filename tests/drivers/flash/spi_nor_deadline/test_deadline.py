#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Moton Intelligent Equipment
"""Run the actual driver polling function with a deterministic SPI/clock fake."""

import pathlib
import subprocess
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[4]
source = (ROOT / "drivers/flash/spi_nor.c").read_text()
start = source.index("static int spi_nor_wait_until_ready(")
function = source[start : source.index("\n#if defined(CONFIG_SPI_NOR_SFDP_RUNTIME)", start)]
harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
#include <stdio.h>
typedef int k_timeout_t;
struct spi_nor_config { bool has_flsr; };
struct device { const struct spi_nor_config *config; };
#define ARG_UNUSED(x) (void)(x)
#define IS_ENABLED(x) (x)
#define ANY_INST_HAS_FLSR 1
#define CONFIG_SPI_NOR_READY_TIMEOUT_MS 20
#define SPI_NOR_CMD_RDFLSR 1
#define SPI_NOR_CMD_RDSR 2
#define SPI_NOR_CMD_CLRFLSR 3
#define SPI_NOR_FLSR_READY 128
#define SPI_NOR_FLSR_ERASE_FAIL 32
#define SPI_NOR_FLSR_PROGRAM_FAIL 16
#define SPI_NOR_FLSR_PROT_ERROR 2
#define SPI_NOR_WIP_BIT 1
#define LOG_ERR(...) (void)0
static int64_t now;
static int polls, ready_at, bus_error, fail_bits, clears;
static int64_t k_uptime_get(void) { return now; }
#ifdef CONFIG_SPI_NOR_SLEEP_WHILE_WAITING_UNTIL_READY
static void k_sleep(k_timeout_t delay) { now += delay; }
#endif
static int spi_nor_cmd_read(const struct device *dev, int cmd, uint8_t *reg, unsigned len)
{
    (void)cmd; (void)len;
    ++polls; ++now;
    if (bus_error) return bus_error;
    bool ready = ready_at && polls >= ready_at;
    *reg = dev->config->has_flsr ? (ready ? 128 | fail_bits : 0) : !ready;
    return 0;
}
static int spi_nor_cmd_write(const struct device *dev, int cmd)
{ (void)dev; assert(cmd == SPI_NOR_CMD_CLRFLSR); ++clears; return 0; }
'''
cases = r'''
static void reset(void) { now = polls = ready_at = bus_error = fail_bits = clears = 0; }
int main(void)
{
    struct spi_nor_config cfg = {0};
    const struct device dev = {&cfg};
    for (int flsr = 0; flsr < 2; ++flsr) {
        cfg.has_flsr = flsr;
        reset(); ready_at = 1;
        assert(spi_nor_wait_until_ready(&dev, 2) == 0 && polls == 1);
        reset(); ready_at = 4;
        assert(spi_nor_wait_until_ready(&dev, 2) == 0 && polls == 4);
        reset();
        assert(spi_nor_wait_until_ready(&dev, 2) == -ETIMEDOUT);
        assert(now >= 20 && now <= 23);
        /* A delayed scheduler cannot prolong the deadline by a poll count. */
        reset();
        assert(spi_nor_wait_until_ready(&dev, 100) == -ETIMEDOUT);
        assert(polls <= 20);
        reset(); bus_error = -EIO;
        assert(spi_nor_wait_until_ready(&dev, 2) == -EIO && polls == 1);
        reset(); ready_at = 1;
        assert(spi_nor_wait_until_ready(&dev, 2) == 0);
    }
    for (int bit = 2; bit <= 32; bit *= 2) {
        if (!(bit & (2 | 16 | 32))) continue;
        reset(); ready_at = 1; fail_bits = bit;
        assert(spi_nor_wait_until_ready(&dev, 2) == -EIO && clears == 1);
    }
    puts("NOR deadline: ready, busy, bus error, FLSR errors, subsequent wait passed");
}
'''
with tempfile.TemporaryDirectory() as tmp:
    c = pathlib.Path(tmp) / "test.c"
    c.write_text(harness + function + cases)
    for sleeping in (False, True):
        exe = pathlib.Path(tmp) / ("sleep" if sleeping else "spin")
        subprocess.run(
            [
                "cc",
                "-std=c11",
                "-Wall",
                "-Wextra",
                "-Werror",
                *(["-DCONFIG_SPI_NOR_SLEEP_WHILE_WAITING_UNTIL_READY"] if sleeping else []),
                str(c),
                "-o",
                str(exe),
            ],
            check=True,
        )
        subprocess.run([str(exe)], check=True)
