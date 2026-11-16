# Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.28.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(irq_exit_fastpath)

target_sources(app PRIVATE src/main.c)
target_include_directories(app PRIVATE
  ${ZEPHYR_BASE}/kernel/include
  ${ZEPHYR_BASE}/arch/${ARCH}/include
)
zephyr_link_libraries(
  -Wl,--wrap=z_get_next_switch_handle
  -Wl,--wrap=arch_sched_directed_ipi
)
if(CONFIG_SCHED_IRQ_EXIT_FASTPATH)
  zephyr_link_libraries(-Wl,--wrap=z_irq_exit_switch_handle)
  if(CONFIG_TIMESLICING)
    zephyr_link_libraries(-Wl,--wrap=z_time_slice_pending)
  endif()
endif()
if(IRQ_EXIT_DROP_IPI_REVERSE)
  target_compile_definitions(app PRIVATE IRQ_EXIT_DROP_IPI_REVERSE)
endif()
