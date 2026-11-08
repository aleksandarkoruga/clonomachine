// Out-of-context wrapper for rtl/synth_filt.v (see ooc_osc.v).
`default_nettype none
module ooc_top (
    input  wire clk,
    input  wire din,
    output reg  dout
);
    reg [127:0] sh;
    always @(posedge clk) sh <= {sh[126:0], din};
    wire busy;
    wire signed [31:0] y;
    wire [10:0] cmax;
    synth_filt #(.UCODE_HEX("../../../rtl/sy_ucode.hex"), .FFN_HEX("../../../rtl/sy_ffn.hex")) u (
        .clk(clk), .rst(sh[0]), .start(sh[1]), .busy(busy),
        .we(sh[2]), .waddr(sh[13:3]), .wdata(sh[45:14]),
        .in_we(sh[46]), .in_a(sh[49:47]), .in_d(sh[81:50]),
        .y_ra(sh[86:82]), .y_q(y), .cyc_max(cmax)
    );
    always @(posedge clk) dout <= ^{busy, y, cmax};
endmodule
