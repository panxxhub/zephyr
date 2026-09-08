Zynq PS XADC sensor
###################

This servo platform driver uses the devcfg XADCIF at 0xf8007100, without PL
logic, interrupts, a shell or floating point. The binding's register window
starts at 0xf8007000 because initialization also unlocks devcfg at offset 0x34.
PCAP must already be clocked at no more than 200 MHz. The interface clock is
PCAP/16 and ADCCLK is a further /8. The continuous sequencer samples calibration,
temperature and all six internal supplies; external channels are disabled.

Enable SENSOR and provide this board node::

    xadc: sensor@f8007000 {
        compatible = "xlnx,zynq-xadc-ps";
        reg = <0xf8007000 0x120>;
        status = "okay";
    };

``sensor_sample_fetch`` reads all internal channels and their hardware extrema.
``sensor_channel_get`` returns Celsius or volts using integer micro-units in
``sensor_value``. Use ``sensor_channel_zynq_xadc`` in
``zephyr/drivers/sensor/zynq_xadc_ps.h`` to select each rail and its min/max.
``SENSOR_CHAN_DIE_TEMP`` is also supported. The extrema cover time since the
initialization reset, not just time since the previous request. Sampling runs
only in the calling management thread; do not call it from a cyclic RT thread.

A FIFO wait is bounded to 1 ms. Failed fetches invalidate the cached sample;
get returns the error instead of stale or partial data. A FIFO timeout requires
reinitialization because the command/response pipeline position is unknown.
The driver exclusively owns XADC configuration: concurrent JTAG/PL XADC access
is unsupported. The PS supply called VCCPDRO in Xilinx software is VCCO_DDR.

Wiring audit: zephyr-servo hardware/core_sch_ax7021.pdf page 2 routes VP/VN to
the core connector. The CTR B004 schematic at zephyr-servo 1937736c, exported
with kicad-cli, lists only CN1.51 on XADC_VP and CN1.53 on XADC_VN, both marked
no-connect. No external analog measurement path is established; neither these
inputs nor VAUX channels are exposed.

References:

* https://docs.amd.com/r/en-US/ug480_7Series_XADC
* https://github.com/Xilinx/embeddedsw/tree/master/XilinxProcessorIPLib/drivers/xadcps/src

Host validation (production driver with a pipelined DRP fake)::

    python3 tests/drivers/sensor/zynq_xadc_ps/host/run.py
