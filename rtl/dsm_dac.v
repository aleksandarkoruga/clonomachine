// Delta-sigma DAC for the ULX3S 4-bit R-2R audio outputs.
//
// One s24 sample per `tick` (48 kHz), linear interpolation to OSR 256 (modulator clock
// = clk/4 = 12.288 MHz at clk 49.152 MHz). Two modulators, selected by `mode`:
//   0: 2nd-order error feedback, 4-bit quantizer, NTF (1-z^-1)^2, noise-shaped dither
//      of +-1/16 LSB. Uses codes ~1..14 (full scale = +-6 LSB around 7.5).
//   1: 2nd-order CIFB 1-bit (all four R-2R bits driven together: inherently linear,
//      immune to resistor mismatch). Full scale input should stay below ~-3 dBFS.
// Integer prototype with in-band SINAD figures: scripts/dsm_proto.py, RTL check:
// sim/tb_dsm.cpp + scripts/dsm_check.py.
//
// The input sample is latched on `tick`, so output timing does not depend on when the
// engine finished computing it (fixed latency, no jitter). Each modulator step is
// spread over the 4 clock cycles of its period (phase 0..3) for timing.
`default_nettype none

module dsm_dac (
    input  wire               clk,
    input  wire               rst,
    input  wire               tick,
    input  wire signed [23:0] sample,
    input  wire               mode,
    output reg  [3:0]         code
);
    reg  [1:0]          ph;                 // free running: 1024 = 256 steps * 4 phases
    reg                 tick_pend;
    reg  signed [23:0]  s_lat;
    reg  signed [23:0]  x_prev, x_cur;
    reg  signed [24:0]  d;
    reg  signed [33:0]  iacc;               // d * k, k = 0..255
    reg  signed [24:0]  xs;                 // interpolated input (phase 1..)
    reg  signed [23:0]  u, w;
    reg  signed [23:0]  e1, e2;             // error feedback state, Q16 LSB units
    reg  [15:0]         lfsr;
    reg  signed [27:0]  i1, i2;             // CIFB state
    reg  signed [27:0]  yfb;

    always @(posedge clk) begin
        ph <= ph + 1'b1;
        if (tick) begin
            tick_pend <= 1'b1;
            s_lat     <= sample;
        end
        case (ph)
        2'd0: begin : p0           // interpolate
            reg signed [24:0] dd;
            reg signed [33:0] ia;
            reg signed [33:0] t;
            reg signed [24:0] ts;
            reg signed [23:0] xp;
            xp = x_prev;
            dd = d;
            ia = iacc;
            if (tick_pend) begin   // new input sample: restart interpolation
                tick_pend <= 1'b0;
                xp = x_cur;
                dd = {s_lat[23], s_lat} - {x_cur[23], x_cur};
                ia = 0;
                x_prev <= x_cur;
                x_cur  <= s_lat;
                d      <= dd;
            end
            t    = ia >>> 8;
            ts   = t[24:0];
            xs   <= xp + ts;
            iacc <= ia + dd;
        end
        2'd1: begin                // scale (mode 0) / feedback level (mode 1)
            u   <= 24'sd491520 + ((xs * 3) >>> 6);   // 7.5 LSB + xs * 6 LSB / 2^23, Q16
            yfb <= i2[27] ? -28'sd8388608 : 28'sd8388608;
        end
        2'd2: begin
            w    <= u - (e1 <<< 1) + e2;
            i1   <= i1 + ((xs - yfb) >>> 1);
        end
        2'd3: begin : p3
            reg signed [23:0] wd;
            reg signed [23:0] qv;
            reg [3:0] q;
            if (!mode) begin
                wd = w + $signed({11'd0, lfsr[12:0]}) - 24'sd4096;
                qv = (wd + 24'sd32768) >>> 16;
                if (qv < 0)        q = 4'd0;
                else if (qv > 15)  q = 4'd15;
                else               q = qv[3:0];
                e2   <= e1;
                e1   <= $signed({4'd0, q, 16'd0}) - w;
                code <= q;
            end else begin
                code <= (yfb > 0) ? 4'd15 : 4'd0;
            end
            i2   <= i2 + ((i1 - yfb) >>> 1);
            lfsr <= {1'b0, lfsr[15:1]} ^ (lfsr[0] ? 16'hB400 : 16'h0000);
        end
        endcase
        if (rst) begin
            ph        <= 0;
            tick_pend <= 1'b0;
            s_lat     <= 0;
            x_prev    <= 0;
            x_cur     <= 0;
            d         <= 0;
            iacc      <= 0;
            xs        <= 0;
            u         <= 0;
            w         <= 0;
            e1        <= 0;
            e2        <= 0;
            i1        <= 0;
            i2        <= 0;
            yfb       <= 0;
            lfsr      <= 16'hACE1;
            code      <= 4'd8;
        end
    end
endmodule

`default_nettype wire
