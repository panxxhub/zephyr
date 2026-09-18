.. _arm_mmu_smp_tests:

ARM AArch32 MMU bring-up on a second CPU
########################################

A secondary CPU has to point itself at the page tables the primary CPU built.
It must not build them again: they are shared, and the primary CPU is
translating through them while the secondary comes up.

Rebuilding them is not merely redundant. ``mmu_zephyr_ranges`` maps the whole
image read/write and execute-never first (``_image_ram_start`` to
``_image_ram_end`` covers the text), and only then narrows the text pages down
to read-only and executable. A CPU that walks the tables during that window
finds the text mapped execute-never and takes a prefetch abort.

This test keeps a thread running on the second CPU while the first CPU re-runs
the secondary MMU initialisation, and requires the thread to keep making
progress. See panxxhub/zephyr#69.
