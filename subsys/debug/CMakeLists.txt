# SPDX-License-Identifier: Apache-2.0

zephyr_sources_ifdef(
  CONFIG_DEBUG_THREAD_INFO
  thread_info.c
  )

zephyr_sources_ifdef(
  CONFIG_ASAN
  asan_hacks.c
  )

add_subdirectory_ifdef(
  CONFIG_THREAD_ANALYZER
  thread_analyzer
  )

add_subdirectory_ifdef(
  CONFIG_DEBUG_COREDUMP
  coredump
  )

add_subdirectory_ifdef(
  CONFIG_GDBSTUB
  gdbstub
  )

zephyr_sources_ifdef(
  CONFIG_MIPI_STP_DECODER
  mipi_stp_decoder.c
  )

add_subdirectory_ifdef(
  CONFIG_SYMTAB
  symtab
  )

zephyr_sources_ifdef(
  CONFIG_CS_TRACE_DEFMT
  coresight/cs_trace_defmt.c
)

zephyr_sources_ifdef(
  CONFIG_CPU_LOAD
  cpu_load.c
  )
