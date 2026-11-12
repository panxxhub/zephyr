# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0
"""Exercise the production RX affinity guard and its Zynq MMU mapping."""

import re
import subprocess
import tempfile
from pathlib import Path

from test_rx_ownership import ROOT, function

STUB = r'''
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define BIT(n) (1U << (n))
#define ARG_UNUSED(x) (void)(x)
#define IS_ENABLED(x) 0
#define CONFIG_DCACHE 1
struct dma_xlnx_sg_cfg { bool rx_outer_nocache, rx_cpu0_owned; };
struct device { const struct dma_xlnx_sg_cfg *config; };
#define DEV_CFG(d) ((d)->config)
static struct { unsigned id; } cpu;
static struct { struct { unsigned cpu_mask; } base; } thread;
#define arch_curr_cpu() (&cpu)
#define k_current_get() (&thread)
static bool in_isr;
#define k_is_in_isr() in_isr
static unsigned l1_calls, outer_calls, barriers;
static int cache_error;
static int arch_dcache_invd_range(void *addr, size_t len)
{
    (void)addr; assert(len == 128U); l1_calls++; return cache_error;
}
static int sys_cache_data_invd_range(void *addr, size_t len)
{
    (void)addr; assert(len == 128U); outer_calls++; return cache_error;
}
#define barrier_dsync_fence_full() (++barriers)
'''

CHECKS = r'''
int main(void)
{
    struct dma_xlnx_sg_cfg cfg = { .rx_outer_nocache = true, .rx_cpu0_owned = true };
    struct device dev = { .config = &cfg };
    thread.base.cpu_mask = 1U;
    assert(rx_payload_invd(&dev, (void *)0x2000, 128U) == 0);
    assert(l1_calls == 1U && outer_calls == 0U && barriers == 1U);
    cache_error = -EIO;
    assert(rx_payload_invd(&dev, (void *)0x2000, 128U) == -EIO);
    cache_error = 0;
#ifdef CONFIG_SMP
    unsigned before = l1_calls;
    thread.base.cpu_mask = 3U;
    assert(rx_payload_invd(&dev, (void *)0x2000, 128U) == -EXDEV);
    thread.base.cpu_mask = 2U;
    cpu.id = 1U;
    assert(rx_payload_invd(&dev, (void *)0x2000, 128U) == -EXDEV);
    in_isr = true;
    assert(rx_payload_invd(&dev, (void *)0x2000, 128U) == -EXDEV);
    assert(l1_calls == before && outer_calls == 0U);
    cpu.id = 0U;
    assert(rx_payload_invd(&dev, (void *)0x2000, 128U) == 0);
    cfg.rx_cpu0_owned = false;
    cpu.id = 1U;
    in_isr = false;
    assert(rx_payload_invd(&dev, (void *)0x2000, 128U) == 0);
#endif
    cfg.rx_cpu0_owned = false;
    cfg.rx_outer_nocache = false;
    assert(rx_payload_invd(&dev, (void *)0x2000, 128U) == 0);
    assert(outer_calls == 1U);
    return 0;
}
'''


def execute(directory, source, defines, should_pass=True):
    cfile = directory / 'test.c'
    exe = directory / 'test'
    cfile.write_text(source)
    subprocess.run(['cc', '-std=c11', '-O2', *defines, str(cfile), '-o', str(exe)], check=True)
    result = subprocess.run([str(exe)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    assert (result.returncode == 0) == should_pass, result.returncode


def main():
    driver = (ROOT / 'drivers/dma/dma_xlnx_axi_dma_sg.c').read_text()
    body = function(driver, 'rx_cpu_access_check') + function(driver, 'rx_payload_invd')
    smp = ['-DCONFIG_SMP=1', '-DCONFIG_SCHED_CPU_MASK=1']
    with tempfile.TemporaryDirectory() as path:
        directory = Path(path)
        execute(directory, STUB + body + CHECKS, [])
        execute(directory, STUB + body + CHECKS, smp)
        for original in (
            'arch_curr_cpu()->id != 0U',
            '!k_is_in_isr() && k_current_get()->base.cpu_mask != BIT(0)',
        ):
            assert original in body
            execute(
                directory, STUB + body.replace(original, 'false') + CHECKS, smp, should_pass=False
            )

        soc = (ROOT / 'soc/xlnx/zynq7000/common/soc.c').read_text()
        match = re.search(r'MMU_REGION_FLAT_ENTRY\("dma_rx_buf",', soc)
        assert match
        depth = 1
        end = match.end()
        while depth:
            depth += (soc[end] == '(') - (soc[end] == ')')
            end += 1
        mapping = soc[match.start() : end]
        source = r'''
#include <assert.h>
#define MT_NORMAL 1U
#define MATTR_SHARED 2U
#define MATTR_CACHE_INNER_WB_nWA 4U
#define MPERM_R 8U
#define MPERM_W 16U
#define MATTR_MAY_MAP_L1_SECTION 32U
#define DT_NODELABEL(x) 0
#define DT_REG_ADDR_BY_NAME(n, name) 0x05600000U
#define DT_REG_SIZE_BY_NAME(n, name) 0x04000000U
#define DT_PROP_OR(n, prop, default_value) OWNED
#define MMU_REGION_FLAT_ENTRY(name, addr, size, attrs) (attrs)
static const unsigned attrs = MAPPING;
int main(void) { assert(attrs == (OWNED ? 61U : 63U)); return 0; }
'''.replace('MAPPING', mapping)
        for owned in (0, 1):
            execute(directory, source, [f'-DOWNED={owned}'])
        execute(
            directory,
            source.replace('0U : MATTR_SHARED', 'MATTR_SHARED : MATTR_SHARED'),
            ['-DOWNED=1'],
            should_pass=False,
        )
        print(
            'PASS: CPU0/remote/unpinned/ISR RX access, cache errors, UP and MMU attributes; '
            '2 missing-guard and 1 wrong-mapping mutations rejected'
        )


if __name__ == '__main__':
    main()
