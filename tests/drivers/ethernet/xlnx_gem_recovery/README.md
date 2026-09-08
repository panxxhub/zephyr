<!--
SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
SPDX-License-Identifier: Apache-2.0
-->

# GEM network recovery host tests

Run `python3 tests/drivers/ethernet/xlnx_gem_recovery/test_recovery.py` from the
Zephyr checkout. A C compiler with pthreads and UBSan is required. Generated
sources and binaries stay in `.scratch/gem-host`. The CI job runs the same command.

The harness extracts production C functions, compiles them against fake MMIO,
DMA rings, cache traces, packet operations and scheduling primitives, and checks
behavior with assertions. Reverse tests mutate those functions to restore a
defect, require successful compilation, and require SIGABRT from the designated
test. Compiler errors, crashes with other signals, timeouts and diagnostic text
are not accepted as reverse-test evidence.

| Test | Coverage |
| --- | --- |
| T1 / R1 | Orphan then good frame across wrap; HRESP and TX-underrun alone; saved pre-clear evidence; stop/reconcile/restart and subsequent good RX. RX pressure is covered by T10/T13. |
| T3 / R3 | Missing EOF, stale EOF in a DMA-owned BD, chain capacity, allocation/copy failures, and continuous refill limited to one ring per invocation. Eight-frame batches bound CPU use; pool pressure delays continuation. |
| T2 / R2 | 32 suppressed transmissions with recovery between attempts; late and duplicate IRQs; no outstanding frame; coalesced completed frames followed by a DMA-owned frame; descriptor errors; refused reclaim if DMA cannot stop; successful fresh send. |
| T6 / R6 | First/second read failures, poisoned getter output, no callback reread, getter-before-monitor, forced 100/full, negotiated 1G/full, invalid speed, cable bounce, stuck reset and reset clearing on read 500. MAC clock/carrier acceptance is tested separately. |
| T5 / R5 | Two pthread clients through actual C22/C45 functions; address/data transaction identity; yielding 1 ms stuck-idle timeout and subsequent read; recursive PHY lock across page-select/data/modify sequences and raw PHY accesses. |
| T7 / R7 | Cache preparation and barriers before ownership, reverse publication through wrap, RX startup preparation, and send after timeout recovery. |

## H2+ bring-up regression (T8)

The parent GEM runtime structure must begin with its named MAC MMIO slot.
With `CONFIG_MMU=y`, the MDIO driver's generic `DEVICE_MMIO_GET(parent)` reads
that first word. Placing the recovery locks before the slot makes MDIO use a
zero-initialized lock word as its register base, before PHY identification or
reset. The bring-up harness extracts the actual runtime structure and MMIO
accessor macros, rather than replacing the accessor with a fake constant.
Its reverse restores the misplaced MMIO fields and fails initialization.

The fixture models management GEM0 and YT8531 from zephyr-servo firmware
`c791b426`: `boards/moton/opus_one_ctrl_gen2/opus_one_ctrl_gen2.dts`, its
board defconfig, `app/prj.conf`, and `app/boards/opus_one_ctrl_gen2.conf`.
Relevant settings are MMU, SMP, 10 kHz ticks, CPU pin-only scheduling,
cooperative system workqueue priority -1, MDIO idle timeout 1000 us, PHY
monitor interval 500 ms, and DHCP plus IPv4LL. GEM0 maps at 0xe000b000;
PHY address is 0, RGMII TX/RX selectors are 13/1, coarse RX delay is bypassed,
and advertisement is 1000/full plus 100/full. The GEM startup case uses the
DTS defaults of 32 RX/TX descriptors and 1536-byte buffers with RX offset 0.

T8 runs actual MDIO initialization/transfers, YT8531 initialization, generic
MII advertisement helpers, page operations, callback registration and monitor
work. Fake MDIO completion costs 108 us and sleep advances 100 us ticks.
Reset clears by elapsed time at 1, 10, 100 and 499 ms; these are test inputs,
not measured silicon timings. A stuck reset rejects initialization; T6 also
covers reset clearing on the final permitted read. The monitor decodes link
and 1000/full directly from YT8531 register 0x11. The separate GEM case checks
startup ownership of every RX BD, no unsolicited recovery, and first RX/TX.

Recursive lock nesting is checked here; competing page/MDIO clients are
covered by the pthread T5 cases. This fixture does not execute Zephyr's SMP
scheduler, DHCP or IPv4LL and cannot establish physical board boot latency.

## Flood scheduling (T9-T11)

RX consumes at most eight frames and one ring of descriptors per invocation.
The first RX interrupt schedules immediate work. Healthy continuations use the
workqueue's normal yield between batches; only pool/stack rejection adds a 2 ms
pause. RX-complete, RX-used and overrun interrupts remain masked while draining
and are restored together afterward. Scheduling preserves existing deadlines.
Returning buffers and clearing RXSR buffer-not-available/overrun status repairs
ordinary RX pressure without stopping either DMA engine or resetting TX.
Full delayed recovery remains for bus/TX errors and inconsistent descriptor
chains. Its stop check is a single readback, never a polling loop.

T9 simulates a 60 s, 2500-frame/s burst with one 98-byte ICMP frame per four
1242-byte UDP frames, 32 x 1536-byte descriptors, and 16 packet slots. It imposes
600 us of CPU cost per allocation attempt, making arrivals faster than drain,
and holds the pool exhausted on alternating 100 ms intervals. A fake CPU0 CoAP
thread at the same cooperative priority wakes every 10 ms. Assertions cover
heartbeat gaps below 18 ms, pauses on rejection, eight-frame batches, no copies
on allocation failure, no error logs, and RX resuming after the burst. T10
replays overrun then orphan heads at BD 11 and 14, checks advancement and
hardware ownership, then RX-used/overrun draining without global recovery and
RX-complete re-enabling. T11 repeats genuine HRESP recovery for 60 simulated
seconds and checks heartbeat progress, delayed recovery, and subsequent RX.

Reverse cases remove the frame cap, pressure/recovery delay, interrupt masking,
orphan advancement or empty-pool copy guard, and insert synchronous logging or
busy waiting. They must fail assertions after compiling successfully.

## Large-ring throughput regression (T13)

The earlier flood fixture counted overruns without raising their interrupts
and did not send ICMP replies. It therefore missed two regressions: treating
RX-used/overrun as full recovery skips a simultaneous TX-complete, aborts the
sender, and resets both rings; unconditional 2 ms pauses impose a throughput
ceiling that includes time spent allocating, copying and in `net_recv_data`.
Eight frames per 2 ms is not 4000 fps once execution time is included.

T13 uses 255 x 2048-byte RX descriptors, 64 x 2048-byte TX descriptors, a
512-packet queue, separate 1800 pps UDP and 500 pps ICMP senders, and independent
1 pps probes for 60 seconds. Arrival timestamps are rate-derived; DMA continues
while CPU0 executes, raises RXSR/ISR pressure on occupied BDs, and respects
interrupt masks. RX packets queue to a peer network task, which exercises the
actual driver send/completion functions for replies. The model grants ready
equal-priority peers their normal workqueue yield and a CoAP heartbeat.

Injected total allocation/copy/receive costs of 250 and 350 us per frame plus
25 us of stack processing both deliver 60/60 probes and 30000/30000 ICMP replies,
with zero RX drops or resets. Maximum batches take 1000/2800 us respectively.
Restoring the previous initial/continuation delay and pressure-reset policy
at 250 us loses 8 probes and 4095 ICMP replies, with 71 global recoveries.
Restoring only unconditional continuation delay at 350 us delivers just 1/60
probes and 21043/30000 ICMP replies. Separate reverses restore pressure-triggered
TX abort, omit RX status acknowledgment, leave RX pressure IRQs masked, or
let masked raw RX status on a TX interrupt bypass a running batch's backoff.

These timings are model inputs, not measured Cortex-A9 costs. This reproduces
the loss regression, not the exact hardware 0/60 blackout or ten-second recovery
tail. The model does not execute the real Zephyr scheduler, socket queues, ARP
or full protocol stack, and cannot establish physical zero-loss acceptance.
`rx_work_calls`, `rx_work_max_us`, `rx_budget_hits`, and `rx_backoffs` are silent
production counters available through `eth_xlnx_gem_get_diagnostics()` to measure
the real batch cost including `net_recv_data` on the next bench run. No DTS ring
or buffer defaults are changed; the v7 RX default remains 1536 bytes.

The GEM ring spinlock serializes parsing, submission, completion, recovery, and
start/stop. TX completion retains its configured ISR path; recovery uses system
work and never waits for the synchronous sender. The sender returns an error
while recovery is pending and cannot publish another transfer until reconciliation
finishes. A controller that fails the stop readback stays faulted with descriptors
unreclaimed. An unplugged cable does not request recovery or affect watchdog feeds.

`eth_xlnx_gem_get_diagnostics()` copies counters and the latest pre-clear snapshot.
Reasons contain GEM interrupt bits plus the named software timeout/chain bits in
`<zephyr/drivers/ethernet/eth_xlnx_gem.h>`. The snapshot records queue bases, software
indices, free TX count, head descriptors, ISR/IMR, RXSR/TXSR and NWCTRL/NWCFG.
YT8531 runtime data also retains MDIO read-error count and last successful live-read
time for debugger inspection. The PHY lock serializes accesses through its driver;
unrelated clients that bypass the PHY API and write its page selector must provide
their own transaction exclusion.

Hardware semantics follow UG585 sections 16.3.5/16.3.8 and the
[Xilinx GEM reset sequence](https://github.com/Xilinx/embeddedsw/blob/master/XilinxProcessorIPLib/drivers/emacps/src/xemacps_hw.c).
GEM returns TX completion ownership on the first BD of a frame; continuation BDs
need not acquire USED. Clearing RXEN/TXEN resets queue positions before rebuilding
both rings. MDEN and MAC speed/filter configuration survive recovery.

Host tests cannot prove physical DMA quiescence, cache coherence, real scheduler
latency, clock programming, or ICMP/CoAP recovery. Bench validation must record the
firmware/kernel SHAs and generated config/DTS, inspect ELF/MMU/cache attributes,
and run error injection and cache-thrash traffic on the two H2+ boards. No board
access is needed or performed by this suite.

## Malformed CoAP storm and synchronous UART (T12 / R8)

The CoAP server parse-failure and oversized-option diagnostics use native
`LOG_ERR_RATELIMIT_RATE(1000, ...)`, retaining ERROR visibility in release and
debug builds. The native macro also reports the skipped-message count. There
is no network-specific rate-limit helper in `net_core.h`; the parser uses its
existing log module directly. Repeated MDIO transaction timeout errors and the
GEM unexpected-link-speed error use the same native one-second rate limit.
PHY initialization errors remain ordinary errors. The driver pacing/orphan
fixes and 1536-byte RX buffer default remain in place.

T12 extracts the actual rate-limit macros from `logging/log.h`, using C atomics
and a virtual uptime, together with the CoAP parser, option decoder, structures
and server socket-receive/parse/rejection prefix. Two 60-second streams at 1800
datagrams/s exercise reserved token lengths and 69-byte options with H2+'s
68-byte capacity. The backend charges 6 ms synchronously per printed line,
including native skipped-message summaries. Each site reports about 60 errors
per stream, rather than 108000. Both ERR-level and DEBUG-enabled variants run.

A 25 ms completion-heartbeat and arrival-lag budget allows the occasional four
lines from two independent sites, but rejects accumulating backlog. Both
streams complete all 108000 iterations within 60 seconds; reversing either
site to a plain ERROR must compile and fail the service budget. A valid packet
still parses afterward. Successful server dispatch and the full hardware
watchdog/scheduler chain are outside this fixture.

Owner bench evidence: unmodified v6 with logging off, a 255 x 2048-byte RX ring
and 512/1024 packet/buffer pools survives the 1800 pps garbage-UDP plus 500 pps
ICMP load with zero probe loss for 60 and 120 seconds. With the original
32-descriptor ring and logging off it still wedges on missing SOF; orphan
recycling remains the safety net for that case. Bench acceptance uses independent
ICMP probes from the second host and `/debug/boot` after the flood. In-flood
CoAP polling of the flooded socket is not a valid liveness measurement.

The owner subsequently tested kupfinal (0cbae6ec, logging off, the same large
rings/pools): 0/60 probes during the burst, spontaneous recovery afterward,
unchanged boot nonce and reset_reason 0x480000. That result rejects unconditional
RX pacing despite the previous host heartbeat tests passing. The native
rate-limit change remains valid; the RX throughput change needs a new hardware
run against the zero-loss v6 control before merge.

## MDIO elapsed-wait instrumentation

The per-controller MDIO runtime records `idle_wait_last_us`, `idle_wait_max_us`,
`idle_waits`, and `idle_timeouts` under its bus mutex, for debugger inspection.
Every pre/post transaction poll contributes, including polls that succeed
immediately and the final deadline read. The elapsed value uses 64-bit uptime
ticks converted to microseconds, includes time descheduled while sleeping, and
is not capped at the configured deadline. Resolution is one system tick
(100 us on the H2+ configuration); it is elapsed time, not CPU busy time.
Counters need no logging. The existing GEM accessor exposes RX batch timing.

T5 verifies normal completion, a stuck bus, preservation of the maximum after
recovery, and a 30-second injected wakeup delay. The delayed poll records the
whole elapsed interval while still issuing a yielding sleep. A reverse omitting
the maximum update must fail. This distinguishes a long observed wait from
proof of a busy-wait in the MDIO driver.
