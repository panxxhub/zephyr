<!--
SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
SPDX-License-Identifier: Apache-2.0
-->

# Minimal GEM fault handling

Run `python3 tests/drivers/ethernet/xlnx_gem_recovery/test_recovery.py` in the
checkout. A C compiler, pthreads and UBSan are required. Generated artifacts
stay in `.scratch/gem-host`.

The baseline is `d9c2bd0085212cb49a4b3c9de20563f718fb5a39`. The GEM source must
be byte-identical to that baseline after replacing only log statements and
the missing-SOF error branch. The complete private header must be identical.
SHA-256 checks enforce this, including a reverse that changes the RX loop.
There is no new RX budget, delayable drain, masking policy, recovery worker,
TX reconciliation, cache-ownership change, runtime layout or diagnostic API.

The one descriptor change recycles a CPU-owned head without SOF: clear control,
issue a barrier, return ownership while preserving address/wrap, advance, and
continue through the baseline drain. Native one-second rate-limited ERROR
macros replace GEM errors; the TX-wrap warning is also rate-limited. The
baseline already releases complete chains without copying when allocation
fails. Normal send, completion and RX handling remain otherwise unchanged.

R6 retains YT8531 reset timeout/final-read correctness, validated live state,
and separate last-notified state so getters do not consume notifications.
R5 restores the per-controller MDIO mutex across C22/C45 transactions and
protects PHY page transactions with the PHY's recursive mutex. MDIO polls
sleep between retries with a nominal 1 ms deadline; there is no busy-wait.
CoAP parse and oversized-option errors use native
`LOG_ERR_RATELIMIT_RATE(1000, ...)`, retaining skipped-message counts.

| Test | Evidence |
| --- | --- |
| T1 | 32 x 512-byte ring, orphan heads 11/14/31 after RX-used/overrun, then a valid 1242-byte frame spanning three BDs; ownership, wrap, next index, RX status clearing and RX-complete re-enable. Empty-pool chains are released without copies. Restoring the old break fails. |
| T13 | Actual GEM RX/ISR/send/completion functions with 255 x 2048 RX and 64 x 2048 TX; 512 packet slots; separate 1800 pps UDP, 500 pps ICMP and independent 1 pps probes for 60 seconds. Hardware raises pressure IRQs when descriptors fill. The actual retained PHY monitor and MDIO functions execute every 500 ms in a linked fixture. |
| T5 | Competing pthread C22/C45 clients, page-transaction exclusion, stuck-bus yielding deadline and subsequent recovery. Reverses remove locks, split C45 transactions or restore spinning/long timeout. |
| T6 | Reset clearing at the last read, stuck reset, forced 100/full, negotiated 1000/full, invalid speed, failed state reads, getter-before-monitor and cable bounce. Reverses restore each defect. |
| T8 | Actual MMIO runtime layout/accessors and MDIO/PHY initialization with H2+ board parameters; reset clears after 1/10/100/499 ms. A misplaced MMIO slot fails the reverse. |
| T12 | Actual CoAP parser and native rate-limit macros, 1800 malformed datagrams/s, synchronous 6 ms/line backend, ERR and DEBUG variants. Both log sites preserve the 25 ms completion budget; plain ERROR reverses fail. |

T13 includes allocation/copy/receive costs of 250 and 350 us/frame plus 25 us
of modeled stack work. Both cases deliver 60/60 probes and 30000/30000 ICMP
replies with zero drops and a progressing cooperative heartbeat. Actual PHY
poll time is added to the arrival timeline. GEM log calls are compiled out in
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
