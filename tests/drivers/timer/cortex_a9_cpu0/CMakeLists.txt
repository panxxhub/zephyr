# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
include(${CMAKE_CURRENT_SOURCE_DIR}/../../../arch/arm/irq_trace/qemu_smp.cmake)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(cortex_a9_cpu0)
target_sources(app PRIVATE src/main.c)
zephyr_ld_options(-Wl,--wrap=sys_clock_announce_locked)
option(REQUIRE_CPU0 "Require CPU 0 isolation even in the policy-off control" OFF)
if(REQUIRE_CPU0)
  target_compile_definitions(app PRIVATE REQUIRE_CPU0=1)
endif()
