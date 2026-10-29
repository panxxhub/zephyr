<!-- SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Issue #40: update the servo kernel to current upstream main

## Problem and audited baseline

The firmware pins `b49a9f54357946fa7ac1a82f388906ead578fbb5`, the head of
open fork PR #75. Fork main is `7d3d351c4fc134466c2497ba49d1106910a86308`.
Both report 4.4.99. Moving the firmware directly to fork main would remove
the QSPI FIFO-width fix already used by the factory installer.

The requested upstream-main snapshot is
`7ff9bcc20cf3ddbb8f7a18a23cd2dfd64f5529a6` (4.5.99). Zephyr 4.5.0 was
released on 2026-10-28 at `41d10b59730406bbab4ca9f1fbbe9e96aafb4966`.
The audited histories have 140 fork-only commits, including merge commits,
and 5,655 upstream-only commits. Merge base:
`e17d4c6c86af49eb5df739f8b991f9165990881c`.

## Required behavior

- Integrate the audited upstream main while retaining the shipping QSPI fix
  and downstream Zynq DMA, cache, Ethernet, watchdog, timer and IRQ behavior.
- Preserve CPU affinity, exclusive FPU ownership with FPU sharing disabled,
  CPU-0-only timeout announcement, and bounded ISR work without logging.
- Preserve primary-only shared MMU page-table initialization. Upstream now
  provides `z_arm_mmu_init(bool is_primary_core)`; reconcile the downstream
  secondary initializer and its regression tests with that API.
- Keep #31 (unverified PHY transition behavior), #52 (rejected GEM recovery
  experiment) and #66 (optional idle IRQ-exit optimization) separate.
- Update firmware compatibility only where required by the new kernel,
  including the removed SMP_BOOT_DELAY option where applicable. Keep the
  kernel manifest revision an immutable fork commit.

## Acceptance criteria

- Resolve all six text-conflicted files and review overlapping changes that
  merge automatically, especially MMU initialization and timeout scheduling.
- Run the fork Cortex-A/R integration suite and QSPI regression; preserve
  their negative controls. Review any test adaptation against its behavior.
- Build Gen2 production, debug and encoder images; run the relevant linked
  RT, logging, affinity, stack, SII, census and CoAP gates from firmware CI.
- Run firmware SMP isolation and control-loop QEMU suites with the new kernel.
- Record the exact kernel/module revisions and distinguish local results,
  CI results and hardware results. A build alone does not qualify hardware.
- Deliver reviewable fork and firmware PRs; the owner merges. PR #75 must
  remain a prerequisite or be retained explicitly in the integration history.

## Constraints and exclusions

Do not rewrite published fork history, fabricate DCO sign-offs, enable the
unmerged scheduler optimization, or flash hardware as part of this source
upgrade. #75 currently passes QSPI and integration checks but fails DCO and
compliance (the inherited L2-cache prompt and missing human sign-offs).
The firmware pin advances after the kernel dependency has been validated
and merged, as required by issue #40.

## Local validation environment

The configured `/home/xpan/zephyrproject/.venv/bin/west` is absent on this
host. No ARM compiler or QEMU was found on PATH. CMake 4.2.3 and .NET are
available. Recover or provision an isolated build environment and establish
baseline results before attributing failures to this upgrade.
