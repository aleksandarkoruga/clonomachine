// Two clients on the SDRAM line queue (audio clock domain): client 0 (the sampler engine) has
// priority, client 1 (the synth) takes the free cycles. Lines come back in request order, 64 words
// each; a tag FIFO says whose line the next 64 fill words are.
`default_nettype none
module line_arb (
    input  wire        clk,
    input  wire        rst,
    input  wire        r0_valid,
    output wire        r0_ready,
    input  wire [24:0] r0_addr,
    input  wire        r1_valid,
    output wire        r1_ready,
    input  wire [24:0] r1_addr,
    output wire        req_valid,
    input  wire        req_ready,
    output wire [24:0] req_addr,
    input  wire        fill_valid,
    input  wire [15:0] fill_data,
    output wire        f0_valid,
    output wire        f1_valid,
    output wire [15:0] f_data
);
    reg        tq [0:63];
    reg  [5:0] wp, rp, wcnt;
    reg  [6:0] n;
    wire       room = n < 7'd60;
    assign req_valid = room && (r0_valid || r1_valid);
    assign req_addr  = r0_valid ? r0_addr : r1_addr;
    assign r0_ready  = room && req_ready;
    assign r1_ready  = room && req_ready && !r0_valid;
    wire       fire = req_valid && req_ready;
    wire       head = tq[rp];
    assign f0_valid = fill_valid && !head;
    assign f1_valid = fill_valid && head;
    assign f_data   = fill_data;
    always @(posedge clk) begin
        if (rst) begin
            wp <= 0;
            rp <= 0;
            wcnt <= 0;
            n <= 0;
        end else begin
            if (fire) begin
                tq[wp] <= !r0_valid;
                wp <= wp + 1'b1;
            end
            if (fill_valid) begin
                wcnt <= wcnt + 1'b1;
                if (wcnt == 6'd63)
                    rp <= rp + 1'b1;
            end
            n <= n + (fire ? 7'd1 : 7'd0) - ((fill_valid && wcnt == 6'd63) ? 7'd1 : 7'd0);
        end
    end
endmodule
`default_nettype wire
