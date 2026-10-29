<!-- SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Issue #40: upstream main implementation plan

## Branch and scope

- Branch: `chore/40-sync-main-20261029`.
- Base: `b49a9f54357946fa7ac1a82f388906ead578fbb5` (PR #75).
- Upstream snapshot: `7ff9bcc20cf3ddbb8f7a18a23cd2dfd64f5529a6`.
- Spec: `docs/superpowers/specs/2026-10-29-issue-40-upstream-main-spec.md`.
- Existing #40 tracks the kernel sync. Product issue #754 describes an older
  DTS migration; assess its remaining scope before using it for the new repin.

## Steps

- [x] Audit live fork PRs, version tags, upstream head and history divergence.
- [x] Simulate the merge without modifying the firmware or kernel checkouts.
- [ ] Obtain owner review of this spec and plan, then commit them and open
  a draft fork PR with the #75 dependency explicit.
- [ ] Recover an isolated west/SDK/QEMU environment. Establish baseline
  builds and test results using the current firmware and pinned kernel.
- [ ] Integrate the audited upstream snapshot without rewriting published
  history, following the fork-sync strategy recorded in issue #40.
- [ ] Resolve `.github/workflows/doc-build.yml` and `.gitignore`, preserving
  the fork's CI adaptations and current upstream requirements.
- [ ] Reconcile `arch/arm/core/cortex_a_r/smp.c` and the auto-merged
  `arch/arm/core/mmu/arm_mmu.c`. Keep both necessary headers, the per-core
  boot hooks and the primary-only page-table construction contract. Review
  MMU test wrappers for the changed signature.
- [ ] Resolve `drivers/ethernet/Kconfig.xlnx_gem` and `eth_xlnx_gem.c`,
  preserving the driver thread, asynchronous TX, frame-sized cache work,
  checksum test behavior and MDIO setup while adopting current interfaces.
- [ ] Reconcile `dts/arm/xilinx/zynq7000.dtsi` by node identity; the automatic
  merge misplaces downstream GEM properties next to new upstream I2C nodes.
- [ ] Review all other overlapping changes and module revisions. Make only
  compatibility fixes needed for the shipping configuration and its tests.
- [ ] Prepare a separate firmware issue branch/PR for required compatibility
  edits, SMP deferred-start migration and the eventual immutable kernel pin.
- [ ] Run the validation below, update PR descriptions with results and
  limitations, and leave merging to the owner.

## Validation

Use isolated build directories and export ZEPHYR_BASE to the kernel worktree.

```sh
python3 tests/drivers/spi/zynq_qspi_fifo/test_fifo.py
.github/ci/run-fork-tests.sh
./scripts/ci/check_compliance.py --parallel -c <integration-base>..HEAD
git diff --check
```

In the firmware worktree, after resolving modules through its manifest:

```sh
west -z "$ZEPHYR_BASE" build -d build/default -p always -b opus_one_ctrl_gen2 app/
west -z "$ZEPHYR_BASE" build -d build/debug-faults -p always -b opus_one_ctrl_gen2 app/ -- -DEXTRA_CONF_FILE=debug.conf
west -z "$ZEPHYR_BASE" build -d build/encoder -p always -b opus_one_ctrl_gen2 encoder_app/
python3 qemu_smp_test/isolation/run.py --zephyr "$ZEPHYR_BASE"
./qemu_control_loop_test/run_tests.sh --build --full
```

Run the firmware CI checks against those actual build products, including
`check_thread_pins.py` (kernel/module thread coverage), `check_rt_no_log.py`,
`check_rt_locks.py`, `check_stack_budget.py`, `check_logging_config.py`,
`check_debug_fault_build.py`, `check_embedded_sii.py`, census and native-PDO
variants, and the relevant source gates. Use the arguments in CI rather
than substituting source-only checks for linked-image coverage.

## Risks and completion boundaries

- A conflict-free merge is insufficient: the MMU auto-merge already combines
  overlapping implementations and needs deliberate reconciliation.
- Kernel and network changes can alter timing, memory use and thread census;
  preserve the current invariants and report required hardware follow-up.
- #75's human DCO requirement remains with its author; do not synthesize it.
- Source upgrade and test preparation are authorized; production deployment
  and merging remain separate owner actions.
