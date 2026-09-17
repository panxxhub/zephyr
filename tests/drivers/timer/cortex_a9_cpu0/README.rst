.. SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
.. SPDX-License-Identifier: Apache-2.0

Cortex-A9 asymmetric timeout tests
################################

Run from a west workspace with ``ZEPHYR_BASE`` pointing at this checkout::

   west twister -p qemu_cortex_a9 -T tests/drivers/timer/cortex_a9_cpu0 --inline-logs
   west twister -p qemu_cortex_a9 -T tests/arch/arm/irq_trace --inline-logs

The QEMU fixture adds CPU 1 to the emulator device tree and enters the
secondary at the ELF entry, replacing the FSBL stub absent from the model.
It does not change the board or production boot code. SMP timer tests disable
QEMU instruction counting: its round-robin CPU execution introduces large
remote-IPI delays when both CPUs enter idle. These are functional tests, not
measurements of hardware interrupt latency.

The tests count private-timer ISRs through the active-IRQ trace hook, clock
announces through a linker wrapper, and timer callbacks on each CPU. CPU 1
creates earlier deadlines while CPU 0 has only a distant timeout, then repeats
under a periodic CPU 0 timer load. Each sleep and semaphore timeout must take
2 through 20 ms; every one-shot callback must execute exactly once. Cycle reads,
busy waits, tickless idle, and two CPU 1 timesliced peers are exercised. Periodic
clock and timeslicing-disabled configurations are also included.

A pinned, never-started thread models the dormant servo in ``not_ready``. This
checks the kernel fixture only; there is no EtherCAT drive in QEMU. Actual
fresh-flash ``not_ready`` boot and OP/load acceptance remain hardware tests.
No runtime audit or production panic is introduced.

The policy-off scenario requires timer IRQs, announces, and callbacks on CPU 1.
To make the same absence assertions run against that control and fail::

   west build -b qemu_cortex_a9 tests/drivers/timer/cortex_a9_cpu0 \
     -d build/timer-reverse -- -DREQUIRE_CPU0=ON
   west build -d build/timer-reverse -t run

``REQUIRE_CPU0`` is a test-only expectation. Removing the remote reprogram IPI
also makes the earlier-deadline test fail or hit its 30-second harness timeout.
Removing the GIC INTID extraction makes the CPU 1 nested-SGI test fail with no
recorded events. Turning off ``ARM_TRACING_IRQ`` while retaining
``ARM_TRACK_ACTIVE_IRQ`` makes the nested trace test reject the old hook window.

Firmware configuration
**********************

Keep the existing SMP image and CPU pinning. Enable
``CONFIG_TIMEOUT_ANNOUNCE_CPU0=y`` with the Cortex-A9 private timer. CPU 0 must
remain online; secondary private timers stay disabled. Earlier remote deadlines
request a directed scheduler IPI to CPU 0, where the current queue is recomputed
under the timeout lock. Expiry wakes remote threads through normal scheduler
IPIs. Timeslice expiry likewise runs on CPU 0 and requests an IPI to the sliced
CPU; a per-thread timeslice callback still executes in that CPU's scheduler
context. No local timer is needed. Busy waits and cycle reads use the global
counter. Long CPU 0 IRQ masking or callbacks still delay timeout service, as
with any single announcing CPU. No hard real-time wake-latency claim is made.

For a per-INTID census, also enable ``CONFIG_TRACING=y``,
``CONFIG_TRACING_USER=y``, ``CONFIG_TRACING_ISR=y``, and
``CONFIG_ARM_TRACING_IRQ=y``. Read ``arch_irq_get_active()`` inside the enter
and exit hooks; never read GICC_IAR again. The ID remains valid through nested
exit and is restored to the preempted interrupt. The traced span excludes
acknowledge and EOI. Disable ``CONFIG_TRACING_THREAD`` and
``CONFIG_TRACING_IDLE`` to omit their trace calls. Other users of switch
instrumentation and idle hooks, such as runtime statistics and CPU load,
retain their required instrumentation. Keep ``CONFIG_IPI_OPTIMIZE=y`` to avoid
unrelated scheduler broadcasts; the timeout policy does not suppress them.

All new policy and IRQ-aware placement options default off. Existing tracing
categories retain their default-on behavior. Firmware repinning and census
consumption are separate changes.

Hardware acceptance
*******************

Boot a freshly flashed drive in ``not_ready`` first. With the servo stopped,
CPU 1 should approach zero interrupts per second. With it running, the census
should show the ``sync_ext`` rate alone, including during a 1 MiB capture and
BLOCK2 download on CPU 0. Record per-INTID counts on both CPUs and compare the
census-on/off wake-latency statistics. These checks require the commander;
the kernel tests do not claim bench acceptance.
