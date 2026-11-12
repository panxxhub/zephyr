.. Copyright The Zephyr Project Contributors
.. SPDX-License-Identifier: Apache-2.0

Idle IRQ-exit fast path
######################

:kconfig:option:`CONFIG_SCHED_IRQ_EXIT_FASTPATH` is an opt-in optimization for
Cortex-A AArch32 and x86-64 SMP interrupt return. It defaults to disabled and
requires single-CPU thread pinning, private per-CPU run queues, builtin atomics,
and optimized directed scheduling IPIs. It excludes non-atomic context switching
and idle-CPU IPI reservations.

An outermost interrupt can return directly to the interrupted idle thread when:

* The interrupted thread is this CPU's actual idle object, is runnable, and has
  no abort/suspend state or saved switch handle.
* There is no pending swap, time-slice expiration, or preempted MetaIRQ thread.
* There is no pending scheduler IPI bit, including requests for another CPU.
* The atomic count for this CPU's private run queue is zero.

The architecture still unwinds nesting, acknowledges the interrupt controller,
and restores the original exception frame. The fast path preserves stack-sentinel
checks and runtime usage accounting. In particular, usage accounting can still
acquire its own lock; this option removes only the unchanged-idle exit's scheduler
lock acquisition. Every rejected exit calls ``z_get_next_switch_handle()``.
No scheduling policy, thread affinity, or interrupt priority is changed.

Publication and directed-IPI ordering
************************************

Queue links are never read without the scheduler lock. Each private run queue
instead has one atomic count. All queue insertions and removals go through
``runq_add()`` and ``runq_remove()`` under the existing scheduler lock. Insertion
increments the count before touching the links; removal decrements it after
unlinking. Thus a zero observation excludes an insertion in progress at that
observation. The existing run-queue initialization starts with a zero count in
kernel BSS. Queue yield does not change membership or the count.

The final sequentially consistent count load is the idle decision's
linearization point. The count contains only queued threads; the current thread
and a MetaIRQ-preempted thread are not queued in SMP, which is why their identity
and state are checked separately. Single-CPU pinning is immutable after thread
start, so another CPU cannot take runnable work from this queue to execute it.

There are two enqueue cases:

#. An insertion has published a nonzero count before the observation. IRQ exit
   takes the normal scheduler path, which acquires the scheduler lock before
   inspecting links. It therefore waits for any unfinished insertion.
#. An insertion begins after the zero observation. The publishing CPU completes
   the insertion, calls ``flag_ipi(ipi_mask_create(thread))`` from
   ``ready_thread()``, and dispatches pending requests through
   ``signal_pending_ipi()`` while holding the scheduler lock. Since the target is
   idle and the new runnable thread is pinned to it, the optimized mask includes
   that target. Idle-CPU reservations cannot suppress this IPI because they are
   excluded by Kconfig.

The target does not clear or drain IPI bits on the fast path. Local interrupts
remain masked from the decision through exception return, and interrupt-controller
EOI has already occurred. A later IPI therefore remains pending and reenters the
scheduler when interrupts are restored. If an earlier IPI has already been
handled (including a nested IPI), its preceding enqueue is visible to the count
load and prevents the fast path. On Cortex-A, ``gic_raise_sgi()`` executes DSB
before writing GICD_SGIR; on x86, the atomic publication precedes APIC dispatch.
This preserves publication before interrupt delivery on both supported paths.

Locally requested swap work is stable with local interrupts masked. Remote halt
requests set thread state before their synchronous directed IPI. Remote slice
expiration publishes an atomic pending flag before flagging its IPI. A request
that races a guard is therefore handled on the subsequent IRQ; the fast path
does not consume that request. The pending-IPI guard also preserves dispatch of
requests made by the just-completed handler for other CPUs.

Validation
**********

``tests/kernel/multiprocessing/irq_exit_fastpath`` uses the actual x86-64 SMP
IRQ-exit dispatcher. Link wrappers count slow scheduler calls and hold CPU 1
before or after its idle decision while CPU 0 starts a pinned thread. Each
configuration executes 32 races on each side of the decision. CPU 1's local timer
is masked so it cannot rescue a lost IPI. Tests cover simple, scalable and multiq
run queues, MetaIRQ bookkeeping, timeslicing enabled/disabled, and the default
slow-path configuration. Guard checks restore injected swap, nested, abort,
switch-handle, pending-IPI and time-slice state before releasing the race gate.

Run the positive scenarios with::

   west twister -p qemu_x86_64 -T tests/kernel/multiprocessing/irq_exit_fastpath

The reverse control drops CPU 0's post-decision directed IPI while recording
that the enqueue requested it. The same wake assertion must fail::

   west twister -p qemu_x86_64 -T tests/kernel/multiprocessing/irq_exit_fastpath \
     -s kernel.smp.irq_exit_fastpath -x IRQ_EXIT_DROP_IPI_REVERSE=ON \
     -O twister-out-irq-exit-reverse

The reverse control records the failed wake before sending a cleanup IPI. Its
expected diagnostic is ``racing enqueue lost: no directed IPI wake``. No periodic
interrupt or unconditional wake is substituted for the acceptance signal.
