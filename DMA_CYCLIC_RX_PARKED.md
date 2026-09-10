<!--
SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
SPDX-License-Identifier: Apache-2.0
-->

# Caller-provided cyclic RX blocks: parked, not required

The encoder waveform application uses the existing RX stream API and CPU writes
into ESC SM3. It does not depend on caller-provided cyclic RX descriptors.
PdoDma CH1 transport is on hold after the low-word duplication observed in
panxxhub/esc#342. This branch contains no driver implementation or API change.

The parked extension would accept a linked `dma_block_config` list with a
`dest_address` for each block, `cyclic=1`, and a completion callback per block.
Descriptor ownership would stay in `drivers/dma/dma_xlnx_axi_dma_sg.c`.
It starts from `servo-kernel-v7.2`; no v7.3 tag is needed for the CPU waveform path.
Any future implementation needs fork-local driver tests and commander bench
acceptance before activation. No upstream-organization gates are part of this
fork-only parked draft.
