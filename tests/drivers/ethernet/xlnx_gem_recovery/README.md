<!--
SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
SPDX-License-Identifier: Apache-2.0
-->

# Minimal GEM fault handling

Run `python3 tests/drivers/ethernet/xlnx_gem_recovery/test_recovery.py` in the
checkout. A C compiler, pthreads and UBSan are required. Generated artifacts
stay in `.scratch/gem-host`.

The normal-path baseline is `d9c2bd0085212cb49a4b3c9de20563f718fb5a39`.
Source hashes permit only logging, SOF resync, RX queue recovery, the bounded
EOF scan, and dispatch on BNA/overrun interrupts. The private header differs
only by the two named RXSR status bits. A source-equality reverse rejects
unrelated changes to the receive loop.

The cheap missing-SOF path clears USED, preserves address/WRAP and advances
to the next BD. It drains complete frames before hard recovery when possible.
A skipped BD, RXSR BNA/overrun, or an EOF scan exhausting the ring triggers
hard recovery. BNA/overrun is checked before scanning so a halted partial
frame cannot trap the handler. Error interrupts also dispatch the handler
without needing an RX-complete interrupt.

Hard recovery holds the device spinlock, disables RX, initializes every RX
BD address/WRAP/USED/control word and resets the software indices. A memory
barrier publishes the rebuilt ring before rewriting `ETH_XLNX_GEM_RXQBASE_OFFSET`
(`gem.rx_qbar`, macb `RBQP`, offset `0x18`), then the saved network control
word is restored. Pending partial data is discarded. TX descriptors and TX
state are preserved. There is no worker, pacing or buffer-size change.

Native one-second rate-limited logs report skipped BDs and queue resets.
Complete frames still release their descriptors without copying when packet
allocation fails.

R6 retains YT8531 reset timeout/final-read correctness, validated live state,
and separate last-notified state so getters do not consume notifications.
R5 restores the per-controller MDIO mutex across C22/C45 transactions and
protects PHY page transactions with the PHY's recursive mutex. MDIO polls
sleep between retries with a nominal 1 ms deadline; there is no busy-wait.
CoAP parse and oversized-option errors use native
`LOG_ERR_RATELIMIT_RATE(1000, ...)`, retaining skipped-message counts.

| Test | Evidence |
| --- | --- |
| T1 | Orphan heads 11/14/31 followed by a valid three-BD frame: cheap skipping delivers the frame, then hard resync returns the cursor to zero. Empty-pool release remains copy-free. Restoring break fails. |
| T14 | Fill 32 BDs and latch DMA halted; isolate RXSR BNA/overrun/both and their error-only interrupts at heads 0/11/14/31, with direct and deferred handlers. Recover complete or incomplete rings; require a new arrival within four attempts and 4096 subsequent deliveries per case. Assert every BD is initialized before RXQBASE is written, RX is disabled, barriers publish writes, indices start at zero and TX is untouched. Removing only the queue-base rewrite fails the bounded recovery assertion. Reverses also omit RX disable, initialize only one BD or restore skip-only handling. |
| T15 | A full ring of stray BDs drains without copying, reports 32 skips once, hard-resets and accepts the next valid frame. |
| T16 | Model invariant: clearing USED, acknowledging RXSR, rewriting RXQBASE while RX is enabled, or toggling RXEN cannot clear the halt. Only an effective queue-base write resumes DMA. |
| T17 | SOF without any EOF terminates after one ring traversal and resets the queue. |
| T13 | Actual GEM RX/ISR/send/completion functions with 255 x 2048 RX and 64 x 2048 TX; 512 packet slots; separate 1800 pps UDP, 500 pps ICMP and independent 1 pps probes for 60 seconds. Hardware raises pressure IRQs when descriptors fill. The actual retained PHY monitor and MDIO functions execute every 500 ms in a linked fixture. |
| T5 | Competing pthread C22/C45 clients, page-transaction exclusion, stuck-bus yielding deadline and subsequent recovery. Reverses remove locks, split C45 transactions or restore spinning/long timeout. |
| T6 | Reset clearing at the last read, stuck reset, forced 100/full, negotiated 1000/full, invalid speed, failed state reads, getter-before-monitor and cable bounce. Reverses restore each defect. |
| T8 | Actual MMIO runtime layout/accessors and MDIO/PHY initialization with H2+ board parameters; reset clears after 1/10/100/499 ms. A misplaced MMIO slot fails the reverse. |
| T12 | Actual CoAP parser and native rate-limit macros, 1800 malformed datagrams/s, synchronous 6 ms/line backend, ERR and DEBUG variants. Both log sites preserve the 25 ms completion budget; plain ERROR reverses fail. |

T13 includes allocation/copy/receive costs of 250 and 350 us/frame plus 25 us
of modeled stack work. Both cases deliver 60/60 probes and 30000/30000 ICMP
replies with zero drops and a progressing cooperative heartbeat. Actual PHY
poll time is added to the arrival timeline. GEM diagnostics have zero modeled cost in
this fixture, matching the production logs-off control; T12 separately tests
the native slow-backend behavior. Tests compile restored defects and require
SIGABRT; build errors and timeouts are not accepted as reverse evidence.

The fixture uses H2+ settings from zephyr-servo c791b426: GEM0 at 0xe000b000,
YT8531 address 0, RGMII TX/RX selectors 13/1, coarse RX delay bypassed, MMU/SMP,
10 kHz ticks, and cooperative CPU0 work. It simulates the hardware and scheduler;
it does not run the real socket/ARP stack or establish hardware throughput.
Costs are injected, not Cortex-A9 measurements. The DTS default remains 1536
bytes; only test fixtures use 512 or 2048 bytes.

## Hardware evidence and scope

Owner control kupv7 (unmodified d9c2bd00, logging off, 255 x 2048 RX, pools
512/1024) survives the corrected flood with 118/118 probes and 29999/29999
ICMP replies, no reset. The rejected 16c28af8 head with identical firmware
answered 2/10 probes, then blacked out and hit SWDT around 40 seconds, falling
back to v6. Earlier 0cbae6ec blacked out for the burst but recovered afterward.
Those observations place the regression in the PR, not the v7 lineage, despite
its earlier synthetic tests passing. The pacing/recovery implementation is
parked separately for investigation and is absent from this minimal driver.

The original small ring with logs off can still wedge on missing SOF. Hardware
acceptance therefore requires both the zero-loss large-ring flood and a 32-BD
overrun/orphan run. Use independent ICMP liveness and post-flood boot nonce;
CoAP polling into the flooded socket is not a valid liveness measurement.
Only fork-owned gates are required. No hardware acceptance is claimed here.

## Overflow model and hardware boundary

The owner small-ring flood falsified skip-only head `00e98a94`: management
reception stayed dead while ESC remained OP and SWDT did not fire. The model
now latches RX DMA halted on BNA and requires an effective queue-base rewrite
to resume. The previous model incorrectly let new arrivals resume after
clearing USED; those passing tests did not cover the observed halt.

[UG585 RXQBASE](https://docs.amd.com/r/en-US/ug585-zynq-7000-SoC-TRM/Register-XEMACPS_RXQBASE_OFFSET-Details)
specifies that queue-base writes are ignored while RX is enabled. The model
enforces this. The reset sequence follows
[mainline macb RX queue recovery](https://github.com/torvalds/linux/blob/master/drivers/net/ethernet/cadence/macb_main.c):
disable reception, initialize the RX ring, rewrite RBQP and restore reception.
The halt latch is the bench failure contract, not a full internal FIFO model.
Commander hardware confirmation of this hard-resync implementation is pending.
