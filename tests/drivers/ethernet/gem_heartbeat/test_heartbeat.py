#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Xiang Pan
"""Exercise the production GEM service probe with fake rings and MMIO."""

import subprocess
import tempfile
from pathlib import Path

root = Path(__file__).resolve().parents[4]
source = (root / "drivers/ethernet/eth_xlnx_gem.c").read_text()
start = source.index("static void eth_xlnx_gem_liveness_work(struct k_work *item)\n{")
function = source[start:]
harness = r"""
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#define ETH_XLNX_GEM_NWCTRL_OFFSET 0
#define ETH_XLNX_GEM_NWCTRL_RXEN_BIT 1
#define ETH_XLNX_GEM_NWCTRL_TXEN_BIT 2
#define ETH_XLNX_GEM_RX_BD_USED_BIT 1
#define CONTAINER_OF(p,t,m) ((t *)((char *)(p)-offsetof(t,m)))
#define K_SECONDS(n) (n)
struct k_work { int unused; };
struct device { const void *config; };
struct bd { uint32_t addr; };
struct eth_xlnx_gem_dev_cfg { uintptr_t base_addr; unsigned tx_bd_count; };
struct eth_xlnx_gem_dev_data {
    struct k_work liveness_work;
    struct device *iface;
    bool started;
    int rx_progress, tx_progress, liveness;
    uint32_t last_rx_progress, last_tx_progress;
    struct { struct bd *first_bd; unsigned next_to_process; } rx_bd_ring;
    struct { unsigned free_bds; } tx_bd_ring;
};
static unsigned scheduled;
static struct k_work *k_work_delayable_from_work(struct k_work *w) { return w; }
static struct device *net_if_get_device(struct device *d) { return d; }
static int atomic_get(int *v) { return *v; }
static void atomic_inc(int *v) { ++*v; }
static uint32_t sys_read32(uintptr_t addr) { return *(uint32_t *)addr; }
static void k_work_reschedule(struct k_work *w, int seconds) {
    (void)w; assert(seconds==1); ++scheduled;
}
"""
cases = r"""
int main(void) {
    uint32_t control=3;
    struct bd ring={0};
    struct eth_xlnx_gem_dev_cfg cfg={(uintptr_t)&control,8};
    struct device dev={&cfg};
    struct eth_xlnx_gem_dev_data d={.iface=&dev,.started=true,
        .rx_bd_ring={&ring,0},.tx_bd_ring={8}};
#define PROBE(expected) do { eth_xlnx_gem_liveness_work(&d.liveness_work); \
                             assert(d.liveness==(expected)); } while(0)
    PROBE(1); PROBE(2); /* Idle is healthy without external packets. */
    ring.addr=1; PROBE(2); /* Pending RX with no recycling is stalled. */
    ++d.rx_progress; PROBE(3); PROBE(3);
    ring.addr=0; d.tx_bd_ring.free_bds=7; PROBE(3);
    ++d.tx_progress; PROBE(4); PROBE(4); /* TX must also drain. */
    ring.addr=1; ++d.rx_progress; PROBE(4); /* RX cannot hide a TX stall. */
    ++d.tx_progress; PROBE(4); /* Nor TX a stalled RX. */
    ++d.rx_progress; ++d.tx_progress; PROBE(5);
    ring.addr=0; d.tx_bd_ring.free_bds=8;
    control=1; PROBE(5); control=2; PROBE(5); control=3; PROBE(6);
    unsigned before=scheduled; d.started=false; PROBE(6); assert(scheduled==before);
    puts("GEM heartbeat: idle, RX/TX stalls, independent progress and stop passed");
}
"""
with tempfile.TemporaryDirectory() as directory:
    tmp = Path(directory)
    (tmp / "test.c").write_text(harness + function + cases)
    subprocess.run(
        [
            "cc",
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-fsanitize=address,undefined",
            str(tmp / "test.c"),
            "-o",
            str(tmp / "test"),
        ],
        check=True,
    )
    subprocess.run([str(tmp / "test")], check=True)
