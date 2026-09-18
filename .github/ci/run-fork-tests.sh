#!/usr/bin/env bash
# SPDX-FileCopyrightText: Copyright (c) 2026 Xiang Pan
# SPDX-License-Identifier: Apache-2.0
# Fixed downstream platform gate, independent of PR changed-file selection.
set -euo pipefail
export ZEPHYR_BASE="$PWD"
python3 tests/drivers/watchdog/zynq_swdt/host/run.py
python3 tests/drivers/flash/spi_nor_deadline/test_deadline.py
for platform in qemu_cortex_a9 qemu_cortex_r5; do
  args=()
  if [[ "$platform" == qemu_cortex_a9 ]]; then
    args+=(--extra-args "EXTRA_DTC_OVERLAY_FILE=$PWD/.github/ci/qemu_cortex_a9_no_slcr.overlay")
  fi
  python3 scripts/twister -p "$platform" \
    -T tests/arch/common/interrupt -T tests/kernel/common \
    -T tests/kernel/fpu_sharing/float_disable -T tests/kernel/fpu_sharing/generic \
    -s arch.interrupt -s kernel.common \
    -s kernel.fpu_sharing.float_disable -s kernel.fpu_sharing.generic.arm \
    --enable-slow \
    --inline-logs --post-build-checks --timeout-multiplier 2 \
    "${args[@]}" -O "twister-out/$platform"
done
python3 scripts/twister -p qemu_cortex_a9 \
  -T tests/drivers/ethernet/xlnx_gem_filter \
  --build-only --inline-logs --post-build-checks -O twister-out/gem

python3 scripts/twister -p qemu_cortex_a9 \
  -T tests/drivers/ethernet/xlnx_gem_loopback \
  --inline-logs --post-build-checks -O twister-out/gem-loopback

python3 scripts/twister -p qemu_cortex_a9 \
  -T tests/arch/arm/irq_trace --inline-logs --post-build-checks \
  -O twister-out/irq-trace
while IFS= read -r elf; do
  python3 tests/subsys/tracing/call_sites/check.py "${elf%/zephyr/zephyr.elf}"
done < <(find twister-out/irq-trace -name zephyr.elf)

# Every scenario here is a two-CPU SMP image making assertions about elapsed
# wall-clock time. Running all five at once puts ten vCPUs on a four-core
# runner, where they starve each other; bound the concurrency instead.
python3 scripts/twister -p qemu_cortex_a9 --jobs 2 \
  -T tests/drivers/timer/cortex_a9_cpu0 --inline-logs --post-build-checks \
  --timeout-multiplier 2 -O twister-out/cortex-a9-timer
