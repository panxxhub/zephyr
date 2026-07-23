# Issue #34: YT8521 PHY ID Settling Retry

## Context

`mc_ytphy_get_id()` in the Motorcomm YT8521/YT8531 PHY driver currently
performs up to 1000 PHYID2 reads. It sleeps for 1 ms only when an MDIO
transaction fails. A successful MDIO transaction that returns a transient
invalid value, such as `0xffff`, is retried immediately.

On an affected Opus One Ctrl Gen1 board, PHYID2 is unstable for about one
second after reset release. The current loop can therefore exhaust all
attempts before the PHY settles and device initialization fails with:

```text
E: PHY (0) timeout to get PHY ID
```

The YT8511 driver received a 20-attempt, 100 ms settling retry in #32/#33,
but the affected hardware selects the YT8521 driver.

## Goals

- Cover the measured post-reset settling interval with a bounded retry.
- Treat both MDIO read failures and successful invalid ID values as
  retryable.
- Preserve support for both YT8521 and YT8531 PHY IDs.
- Avoid per-attempt log noise.
- Preserve the driver's existing `-EIO` failure result.

## Non-goals

- Change PHY reset timing, RGMII delay configuration, autonegotiation, or
  link-state behavior.
- Change device-tree bindings or Kconfig.
- Mask a permanently absent or unsupported PHY.
- Add retry policy configuration to Kconfig or devicetree.

## Required behavior

1. Define a maximum of 20 PHY ID attempts and a 100 ms delay between
   unsuccessful attempts.
2. On each attempt, read `MII_PHYID2R`.
3. Return success immediately when the value is `PHY_ID_YT8521` or
   `PHY_ID_YT8531`.
4. If the read fails or returns any other value, sleep for 100 ms only when
   another attempt remains.
5. If all attempts fail:
   - emit one terminal error;
   - include the last MDIO error or invalid ID in that error;
   - return `-EIO`, matching the current public behavior.
6. If a valid ID is obtained after the first attempt, emit one warning with
   the successful attempt count.
7. Preserve the existing `phy_id` output and debug-log behavior after a
   successful probe.

The maximum added settling window is 1.9 seconds because there are 19 delays
between 20 attempts.

## Acceptance criteria

- A sequence of invalid IDs followed by `0x011a` initializes successfully.
- A sequence of MDIO errors followed by a supported ID initializes
  successfully.
- A permanently invalid or unreadable PHY still fails after the bounded
  interval.
- The YT8521/YT8531 ID match and all unrelated driver behavior remain
  unchanged.
- The affected application builds cleanly against the hotfix.
- On the affected board, JTAG boot no longer reports the PHY ID timeout,
  `169.254.0.42` responds to ping, and the CoAP endpoint is reachable.

