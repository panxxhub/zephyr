#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0
"""Exercise the real Cortex-A/R idle implementation with mocked wait instructions."""

import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
HARNESS = r'''
#include <assert.h>
#include <stdbool.h>
#include <string.h>
static char events[32];
static unsigned int count;
static bool enabled, allow_wait;
static void event(char value) { events[count++] = value; }
static void dsb(void) { event('B'); }
static void wfi(void) { assert(!enabled); event('W'); }
static void wfe(void) { assert(!enabled); event('V'); }
static void enable(void) { event('U'); enabled = true; }
static void disable(void) { event('D'); enabled = false; }
static void isb(void) { event('I'); }
static void soc_exit(void) { assert(!enabled); event('S'); }
void sys_trace_idle(void) { event('E'); }
void sys_trace_idle_exit(void) { assert(!enabled); event('X'); }
bool z_arm_on_enter_cpu_idle(void) { event('P'); return allow_wait; }
#define __DSB dsb
#define __WFI wfi
#define __WFE wfe
#define __enable_irq enable
#define __disable_irq disable
#define __ISB isb
#define SOC_ON_EXIT_CPU_IDLE soc_exit()
#include "cpu_idle.c"
static void reset(bool wait) {
 memset(events, 0, sizeof(events)); count = 0; enabled = false; allow_wait = wait;
}
static void check(bool atomic, unsigned int key, bool wait) {
 char expected[32] = "";
#ifdef CONFIG_SYS_IDLE_HOOKS
 strcat(expected, "E");
#endif
 if (atomic) strcat(expected, "D");
#ifdef CONFIG_ARM_ON_ENTER_CPU_IDLE_HOOK
 strcat(expected, "P");
#else
 wait = true;
#endif
 if (wait) {
  strcat(expected, atomic ? "BVS" : "BWS");
#ifdef CONFIG_SYS_IDLE_HOOKS
  strcat(expected, "X");
#endif
 }
 if (!atomic || !key) strcat(expected, "U");
 if (!atomic) strcat(expected, "I");
 assert(!strcmp(events, expected));
 assert(enabled == (!atomic || !key));
}
int main(void) {
 for (unsigned int wait = 0; wait < 2; ++wait) {
  reset(wait); arch_cpu_idle(); check(false, 0, wait);
  for (unsigned int key = 0; key < 2; ++key) {
   reset(wait); arch_cpu_atomic_idle(key); check(true, key, wait);
  }
 }
 return 0;
}
'''

with tempfile.TemporaryDirectory() as directory:
    path = Path(directory)
    (path / "zephyr/tracing").mkdir(parents=True)
    (path / "zephyr/kernel.h").write_text("")
    (path / "zephyr/tracing/tracing.h").write_text("")
    (path / "cpu_idle.c").write_text((ROOT / "arch/arm/core/cortex_a_r/cpu_idle.c").read_text())
    (path / "test.c").write_text(HARNESS)
    for tracing in (False, True):
        for conditional in (False, True):
            defines = ["-DCONFIG_ARM_ON_EXIT_CPU_IDLE=1"]
            if tracing:
                defines.append("-DCONFIG_SYS_IDLE_HOOKS=1")
            if conditional:
                defines.append("-DCONFIG_ARM_ON_ENTER_CPU_IDLE_HOOK=1")
            subprocess.run(
                [
                    "cc",
                    "-std=c11",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    f"-I{path}",
                    *defines,
                    str(path / "test.c"),
                    "-o",
                    str(path / "test"),
                ],
                check=True,
            )
            subprocess.run([str(path / "test")], check=True)
            print(f"PASS: idle tracing={tracing}, conditional wait={conditional}")
