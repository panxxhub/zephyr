# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0
"""Run the SoC reset hook with dirty outer data and detect reset-path regressions."""

import re
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
SOURCE = ROOT / 'soc/xlnx/zynq7000/common/soc.c'

HARNESS = r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <setjmp.h>
#define BIT(n) (1U << (n))
#define ARG_UNUSED(x) (void)(x)
#define DT_NODELABEL(x) 0
#define DT_REG_ADDR(x) 0xF8000000U
#define DT_REG_SIZE(x) 0x1000U
#define K_MEM_CACHE_NONE 0U
typedef uintptr_t mm_reg_t;
static jmp_buf reset;
static bool outer_dirty = true;
static bool interrupts_locked;
static bool unlocked;
static unsigned fences;
static unsigned flushes;
static unsigned irq_lock(void)
{
    interrupts_locked = true;
    return 0;
}
static int sys_cache_data_flush_all(void)
{
    assert(interrupts_locked);
    outer_dirty = false;
    flushes++;
    return 0;
}
static void barrier_dsync_fence_full(void)
{
    fences++;
}
static void barrier_isync_fence_full(void) {}
static void __attribute__((unused)) device_map(mm_reg_t *virt, uintptr_t phys,
                                              size_t size, unsigned flags)
{
    (void)virt;
    (void)phys;
    (void)size;
    (void)flags;
    assert(false && "page table changed with L1 off and outer walker cacheable");
}
static void sys_write32(uint32_t value, uintptr_t address)
{
    assert(interrupts_locked);
    assert(!outer_dirty && "reset reached SLCR before dirty L2 drained");
    if (address == 0xF8000008U) {
        assert(value == 0xDF0DU);
        unlocked = true;
    } else {
        assert(address == 0xF8000200U && value == 1U);
        assert(unlocked && fences > 0);
        longjmp(reset, 1);
    }
}
'''

MAIN = r'''
int main(void)
{
    /* sys_reboot has already switched off the calling core's L1. */
    if (setjmp(reset) == 0) {
        sys_arch_reboot(0);
        assert(false);
    }
    assert(flushes == 1);
    return 0;
}
'''


def main():
    source = SOURCE.read_text()
    start = source.index('void sys_arch_reboot(int type)\n{')
    end = source.index('\n}', start) + 2
    hook = source[start:end]
    definitions = '\n'.join(re.findall(r'^#define SLCR_.*$', source, re.MULTILINE))
    variants = {
        'reboot_sequence': hook,
        'missing_outer_flush': hook.replace('sys_cache_data_flush_all();', '(void)0;'),
        'late_slcr_mapping': hook.replace(
            'const mm_reg_t slcr = (mm_reg_t)DT_REG_ADDR(DT_NODELABEL(slcr));',
            'mm_reg_t slcr; device_map(&slcr, 0xF8000000U, 0x1000U, K_MEM_CACHE_NONE);',
        ),
    }
    with tempfile.TemporaryDirectory(prefix='pl310-reboot-') as temporary:
        directory = Path(temporary)
        for name, variant in variants.items():
            path = directory / f'{name}.c'
            binary = directory / name
            path.write_text(HARNESS + definitions + '\n' + variant + MAIN)
            subprocess.run(['cc', '-std=c11', '-Wall', str(path), '-o', str(binary)], check=True)
            result = subprocess.run([str(binary)], capture_output=True, text=True, timeout=5)
            if name == 'reboot_sequence':
                assert result.returncode == 0, result.stderr
                print(f'{name}: PASS')
            else:
                assert variant != hook
                assert result.returncode != 0, f'{name} did not fail'
                assert 'Assertion' in result.stderr, result.stderr
                print(f'{name}: expected runtime failure: {result.stderr.strip()}')


if __name__ == '__main__':
    main()
