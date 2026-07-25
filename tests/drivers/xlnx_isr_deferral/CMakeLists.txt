# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(xlnx_isr_deferral)

execute_process(
  COMMAND
    ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/check_isr_callgraph.py
    ${ZEPHYR_BASE}/drivers/dma/dma_xlnx_axi_dma_sg.c
    ${ZEPHYR_BASE}/drivers/sdhc/xlnx_zynq_sdhc.c
  RESULT_VARIABLE isr_callgraph_result
  OUTPUT_VARIABLE isr_callgraph_output
  ERROR_VARIABLE isr_callgraph_output
)
if(NOT isr_callgraph_result EQUAL 0)
  message(FATAL_ERROR "${isr_callgraph_output}")
endif()

target_sources(app PRIVATE src/test_dma.c src/test_sdhc.c)

# The production drivers are included directly by the unit tests so their
# private ISR/worker boundary can be exercised without real MMIO devices.
target_compile_definitions(app PRIVATE
  CONFIG_DMA_XLNX_AXI_DMA_SG_NUM_TX_BD=4
  CONFIG_DMA_XLNX_AXI_DMA_SG_NUM_RX_BD=4
  CONFIG_DMA_XLNX_AXI_DMA_SG_IRQ_THRESHOLD=1
  CONFIG_DMA_XLNX_AXI_DMA_SG_IRQ_TIMEOUT=0
  CONFIG_XLNX_ZYNQ_SDHC_HOST_INTR=1
)
