// Out-of-context wrapper for rtl/synth_osc.v: every input comes from a shift register loaded
// through one pin, every output is folded into one pin, so nothing is optimised away and the
// unit's own paths are what nextpnr times. Used by scripts/ooc/run.sh.
`default_nettype none
module ooc_top (
    input  wire clk,
    input  wire din,
    output reg  dout
);
    reg [127:0] sh;
    always @(posedge clk) sh <= {sh[126:0], din};
    wire busy, rqv;
    wire [24:0] rqa;
    wire signed [23:0] of, od;
    wire [31:0] lag;
    wire [10:0] cmax;
    synth_osc #(.SINC_HEX("../../../rtl/sy_sinc.hex"), .FN_HEX("../../../rtl/sy_fn.hex")) u (
        .clk(clk), .rst(sh[0]), .start(sh[1]), .busy(busy),
        .we(sh[2]), .waddr(sh[12:3]), .wdata(sh[44:13]),
        .req_valid(rqv), .req_ready(sh[45]), .req_addr(rqa),
        .fill_valid(sh[46]), .fill_data(sh[62:47]),
        .out_ra(sh[65:63]), .out_filt(of), .out_direct(od), .lag(lag), .cyc_max(cmax)
    );
    always @(posedge clk) dout <= ^{busy, rqv, rqa, of, od, lag, cmax};
endmodule
