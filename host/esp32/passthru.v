// ESP32 pass-through for the ULX3S: the FT231X UART on US1 goes straight to the on-board ESP32,
// and DTR / RTS give esptool its reset and boot-mode sequence. For programming or reading the
// ESP32 only: loaded into SRAM (host/esp32/esp.sh), the sampler comes back at the next power-up
// or when its bitstream is loaded again. The SD card pins are left alone (inputs).
module esp32_passthru (
    input        ftdi_txd,   // FT231X TX -> FPGA
    output       ftdi_rxd,   // FPGA -> FT231X RX
    input        ftdi_ndtr,  // active low
    input        ftdi_nrts,  // active low
    input        wifi_txd,   // ESP32 TX -> FPGA
    output       wifi_rxd,   // FPGA -> ESP32 RX
    output       wifi_en,
    output       wifi_gpio0,
    output [7:0] led
);
    assign wifi_rxd = ftdi_txd;
    assign ftdi_rxd = wifi_txd;
    // RTS alone: reset (EN low); DTR alone: boot mode (GPIO0 low); both or neither: run
    assign wifi_en    = ~(~ftdi_nrts & ftdi_ndtr);
    assign wifi_gpio0 = ~(~ftdi_ndtr & ftdi_nrts);
    assign led = {~ftdi_txd, ~wifi_txd, 4'b0, ~wifi_gpio0, ~wifi_en};
endmodule
