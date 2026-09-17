# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0

# The board's emulator DT has only one CPU. Extend it without changing the board.
find_program(DTC dtc REQUIRED)
set(qemu_dts ${CMAKE_CURRENT_BINARY_DIR}/qemu-smp.dts)
set(qemu_dtb ${CMAKE_CURRENT_BINARY_DIR}/qemu-smp.dtb)
execute_process(
  COMMAND ${DTC} -q -I dtb -O dts
          ${CMAKE_CURRENT_LIST_DIR}/../../../../boards/qemu/cortex_a9/fdt-zynq7000s.dtb
          -o ${qemu_dts}
  COMMAND_ERROR_IS_FATAL ANY
)
file(APPEND ${qemu_dts} "
/ { cpus { cpu@1 { compatible = \"arm,cortex-a9\"; device_type = \"cpu\"; reg = <1>; }; }; };
")
execute_process(
  COMMAND ${DTC} -q -I dts -O dtb ${qemu_dts} -o ${qemu_dtb}
  COMMAND_ERROR_IS_FATAL ANY
)
# QEMU has no FSBL spin-table stub to enter Zephyr on the secondary CPU.
list(APPEND QEMU_EXTRA_FLAGS -dtb ${qemu_dtb}
     -device "loader,file=$<TARGET_FILE:zephyr_final>,cpu-num=1")
