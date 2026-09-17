# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
if(IRQ_TRACE_SMP)
  include(${CMAKE_CURRENT_SOURCE_DIR}/qemu_smp.cmake)
endif()
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
if(IRQ_TRACE_SMP)
  qemu_smp_generate_dtb()
endif()
project(irq_trace)
target_sources(app PRIVATE src/main.c)
