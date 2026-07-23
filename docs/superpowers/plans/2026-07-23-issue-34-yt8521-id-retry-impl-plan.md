# Issue #34: YT8521 PHY ID Settling Retry Implementation Plan

## Progress

- [x] Implement bounded retry for read errors and invalid IDs.
- [x] Pass `clang-format`, `checkpatch`, and diff validation.
- [x] Build the Ctrl Gen1 application against the hotfix worktree.
- [x] Confirm the build selects `CONFIG_PHY_MOTORCOMM_YT8521=y`.
- [x] JTAG-load the hotfix ELF without writing QSPI.
- [x] Verify stable ping and CoAP reachability.
- [ ] Inspect the UART retry warning (intentionally skipped per user request).
- [ ] Complete PR review/CI and merge.
- [ ] Tag the merged kernel and advance `zephyr-servo/west.yml`.

## 1. Establish the failing baseline

- Confirm the worktree is based on `panxxhub/zephyr` `main`.
- Record the existing `mc_ytphy_get_id()` behavior:
  - 1000 iterations;
  - 1 ms sleep only on MDIO read errors;
  - no sleep after successful invalid reads.
- Preserve the board evidence in issue #34 as the hardware regression
  baseline.

## 2. Implement the bounded settling retry

Modify only:

```text
drivers/ethernet/phy/phy_motorcomm_yt8521.c
```

- Add named constants for 20 attempts and a 100 ms retry delay.
- Track the last MDIO result, last PHY ID, and successful attempt number.
- Accept YT8521 or YT8531 immediately.
- Delay after both read failures and invalid IDs when another attempt
  remains.
- Emit one warning on delayed recovery.
- Emit one terminal error on exhaustion and keep returning `-EIO`.

## 3. Local validation

- Run formatting/checkpatch-style validation on the modified driver.
- Inspect the final diff for unrelated changes.
- Build the Opus One Ctrl Gen1 application with
  `/home/xpan/moton_ws/zephyr-issue-34` as `ZEPHYR_BASE`.
- Confirm the generated image still selects
  `CONFIG_PHY_MOTORCOMM_YT8521=y`.

Because this fork has no focused PHY-driver unit-test harness, the direct
regression test is the affected physical board. The generic Ethernet test
suite provides only build coverage and cannot model the observed post-reset
MDIO sequence.

## 4. Hardware validation

- JTAG-load the rebuilt non-encrypted application on the affected board.
- Confirm there is no terminal `PHY (0) timeout to get PHY ID`.
- Confirm the retry warning reports recovery after more than one attempt
  when the marginal startup behavior occurs.
- Verify ping to `169.254.0.42`.
- Verify the CoAP endpoint is reachable.

No QSPI write is required for the first validation pass.

## 5. Review and merge the Zephyr hotfix

- Commit the planning documents and implementation as intentional commits.
- Push `fix/34-yt8521-id-retry`.
- Open a draft PR that closes #34.
- Run the repository review/check loop and classify any fork-inapplicable
  upstream metadata failures separately from code/test failures.
- Mark ready and merge only after validation and explicit approval.

## 6. Advance zephyr-servo

After the Zephyr PR is merged:

- Tag the merge commit as the next immutable servo kernel tag
  (`servo-kernel-v2`).
- Create a separate zephyr-servo issue, branch, and worktree.
- Update `west.yml` from `servo-kernel-v1` to `servo-kernel-v2`, including
  the exact merge SHA and hotfix rationale in the adjacent comment.
- Run `west update`/manifest resolution checks and a clean Ctrl Gen1
  application build.
- Open, review, and merge the zephyr-servo PR.
