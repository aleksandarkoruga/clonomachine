// Plate reverb: the PMWave UGen (github.com/aleksandarkoruga/pmwave) as a 2x2 plate, bit-exact with
// model/reverb_model.cpp (see there for the arithmetic). Runs beside the engine in the audio
// clock domain: start takes one input sample (the voices' send mix), out holds the plate's
// sample when busy drops, about 270 cycles later (of 1024 per sample).
//
//   16 rails, 2048 x s18 each, in one 32K x 18 block RAM (32 EBR) at {rail, slot}
//   rail r: length len (Q11.20) -> delay d, fraction k (10 bits) -> taps t0 t1 = T0/T1[k],
//           t2 t3 = T1/T0[1024-k] (0 for k = 0) -> x = taps . ring[w-d .. w-d-3] (s24 Q2.22)
//           -> y = gain x + pole y[n-1] -> wave = lp = y; len += inc
//   junctions p[J] = (4 arriving waves) / 2, p[0] += 0.4 in
//   ring writes at slot w-1: p - wave, 24 -> 18 bits with stochastic rounding (xorshift32)
//   out = (p[3] + p[1]) / 2
//
// Registers (from the soft core, fw/reverb_ctl.c), taken at a block start (every 256 samples)
// when start comes: page 0 (waddr[5] = 0): 0..15 next target length (Q11.20), 16..31 next gain
// (Q1.17); page 1: 0..15 next pole (Q1.17). Each rail then ramps to its target over the block.
//
// The one multiplier (25 x 18, split like the engine's) has 3 cycles of latency.
module reverb_unit #(
    parameter T0_HEX = "rv_t0.hex",
    parameter T1_HEX = "rv_t1.hex"
) (
    input  wire               clk,
    input  wire               rst,
    input  wire               start,
    input  wire signed [23:0] in,
    output reg  signed [23:0] out,
    output wire               busy,
    input  wire               we,
    input  wire [5:0]         waddr,
    input  wire [31:0]        wdata
);
    // ------------------------------------------------------------------ helpers
    function signed [23:0] sat24;
        input signed [47:0] v;
        begin
            if (v > 48'sd8388607)       sat24 = 24'sd8388607;
            else if (v < -48'sd8388608) sat24 = -24'sd8388608;
            else                        sat24 = v[23:0];
        end
    endfunction
    function signed [17:0] sat18;
        input signed [31:0] v;
        begin
            if (v > 32'sd131071)       sat18 = 18'sd131071;
            else if (v < -32'sd131072) sat18 = -18'sd131072;
            else                       sat18 = v[17:0];
        end
    endfunction

    // ------------------------------------------------------------------ registers
    reg [30:0] nxt_tgt  [0:15];
    reg [16:0] nxt_gain [0:15];
    reg [16:0] nxt_pole [0:15];
    reg [30:0] tgt      [0:15];   // taken at the block start
    reg [16:0] gain     [0:15];
    reg [16:0] pole     [0:15];
    reg [31:0] len      [0:15];
    reg signed [31:0] inc [0:15];
    reg signed [23:0] lp   [0:15];
    reg signed [23:0] wave [0:15];
    reg signed [23:0] p    [0:3];
    integer i;

    always @(posedge clk)
        if (we) begin
            if (!waddr[5] && !waddr[4]) nxt_tgt[waddr[3:0]]  <= wdata[30:0];
            if (!waddr[5] &&  waddr[4]) nxt_gain[waddr[3:0]] <= wdata[16:0];
            if ( waddr[5] && !waddr[4]) nxt_pole[waddr[3:0]] <= wdata[16:0];
        end

    // ------------------------------------------------------------------ memories
    reg [17:0] ram [0:32767];
    reg [14:0] ram_ra, ram_wa;
    reg [17:0] ram_q, ram_wd;
    reg        ram_we;
    always @(posedge clk) begin
        ram_q <= ram[ram_ra];
        if (ram_we) ram[ram_wa] <= ram_wd;
    end
    integer k0;
    initial for (k0 = 0; k0 < 32768; k0 = k0 + 1) ram[k0] = 18'd0;

    reg [17:0] t0rom [0:1023];
    reg [17:0] t1rom [0:1023];
    initial begin
        $readmemh(T0_HEX, t0rom);
        $readmemh(T1_HEX, t1rom);
    end
    reg [9:0]  ta, tb;
    reg [17:0] t0a, t1a, t0b, t1b;
    always @(posedge clk) begin
        t0a <= t0rom[ta];
        t1a <= t1rom[ta];
        t0b <= t0rom[tb];
        t1b <= t1rom[tb];
    end

    // ------------------------------------------------------------------ multiplier
    // mp = ma * mb, 3-cycle latency (operands registered by the FSM)
    reg  signed [24:0] ma;
    reg  signed [17:0] mb;
    reg  signed [25:0] m_ph;   // ma[24:17] * mb
    reg  signed [35:0] m_pl;   // ma[16:0]  * mb
    reg  signed [42:0] mp;
    always @(posedge clk) begin
        m_ph <= $signed(ma[24:17]) * mb;
        m_pl <= $signed({1'b0, ma[16:0]}) * mb;
        mp   <= ($signed(m_ph) <<< 17) + m_pl;
    end

    // ------------------------------------------------------------------ FSM
    localparam S_IDLE = 3'd0, S_PRE = 3'd1, S_RAIL = 3'd2, S_JUNC = 3'd3, S_JIN = 3'd4, S_SCAT = 3'd5;
    reg [2:0]  state;
    reg [3:0]  r;              // rail; in S_SCAT the write index
    reg [3:0]  st;             // cycle within a rail / the pre-roll
    reg [10:0] w;
    reg [7:0]  cnt;
    reg [31:0] rng;
    reg signed [23:0] in_r, inq;
    reg [10:0] dly;            // delay of the rail being read
    reg        kz;             // fraction 0: t2 = t3 = 0
    reg signed [17:0] tp0, tp1, tp2, tp3;
    reg signed [17:0] a0, a1, a2, a3;
    reg signed [42:0] acc;
    assign busy = (state != S_IDLE) || start;

    wire [3:0] wj = {r[3:2], 2'b00};   // scatter: the junction's first rail
    wire [1:0] jj = r[3:2];
    reg signed [24:0] sv;              // scatter value p - wave
    reg [31:0] rn;
    always @(*) begin
        case (r[1:0])
        2'd0: sv = $signed(p[jj ^ 2'd1]) - $signed(wave[wj + 4'd2]);
        2'd1: sv = $signed(p[jj ^ 2'd2]) - $signed(wave[wj + 4'd3]);
        2'd2: sv = $signed(p[jj])        - $signed(wave[wj + 4'd0]);
        default: sv = $signed(p[jj])     - $signed(wave[wj + 4'd1]);
        endcase
        rn = rng ^ (rng << 13);
        rn = rn ^ (rn >> 17);
        rn = rn ^ (rn << 5);
    end

    always @(posedge clk) begin
        ram_we <= 1'b0;
        if (rst) begin
            state <= S_IDLE;
            w     <= 11'd0;
            cnt   <= 8'd0;
            rng   <= 32'h2545F491;
            out   <= 24'sd0;
            for (i = 0; i < 16; i = i + 1) begin
                len[i]  <= 32'd1024 << 20;
                inc[i]  <= 32'sd0;
                gain[i] <= 17'd0;
                pole[i] <= 17'd0;
                lp[i]   <= 24'sd0;
                wave[i] <= 24'sd0;
                tgt[i]  <= 31'd1024 << 20;
                nxt_tgt[i] <= 31'd1024 << 20;
            end
        end else case (state)
        S_IDLE: if (start) begin
            in_r <= in;
            if (cnt == 8'd0)                   // block start: the soft core's registers
                for (i = 0; i < 16; i = i + 1) begin
                    tgt[i]  <= nxt_tgt[i];
                    gain[i] <= nxt_gain[i];
                    pole[i] <= nxt_pole[i];
                end
            ma    <= {in[23], in};             // 0.4 in
            mb    <= 18'sd52429;
            st    <= 4'd0;
            state <= S_PRE;
        end
        S_PRE: begin                           // the input product
            st <= st + 1'b1;
            if (st == 4'd2) begin
                inq   <= sat24((mp + 43'sd65536) >>> 17);
                r     <= 4'd0;
                st    <= 4'd0;
                state <= S_RAIL;
            end
        end
        S_RAIL: begin
            st <= st + 1'b1;
            case (st)
            4'd0: begin
                if (cnt == 8'd0)
                    inc[r] <= ($signed({1'b0, tgt[r]}) - $signed(len[r])) >>> 8;
                dly    <= len[r][30:20];
                kz     <= (len[r][19:10] == 10'd0);
                ta     <= len[r][19:10];
                tb     <= 10'd0 - len[r][19:10];   // 1024 - k (k = 0: unused)
                ram_ra <= {r, w - len[r][30:20]};
            end
            4'd1: ram_ra <= {r, w - dly - 11'd1};
            4'd2: begin
                ram_ra <= {r, w - dly - 11'd2};
                tp0 <= t0a;
                tp1 <= t1a;
                tp2 <= kz ? 18'sd0 : t1b;
                tp3 <= kz ? 18'sd0 : t0b;
                a0  <= ram_q;
            end
            4'd3: begin
                ram_ra <= {r, w - dly - 11'd3};
                a1 <= ram_q;
                ma <= {{7{a0[17]}}, a0};
                mb <= tp0;
            end
            4'd4: begin
                a2 <= ram_q;
                ma <= {{7{a1[17]}}, a1};
                mb <= tp1;
            end
            4'd5: begin
                a3 <= ram_q;
                ma <= {{7{a2[17]}}, a2};
                mb <= tp2;
            end
            4'd6: begin
                acc <= mp;
                ma  <= {{7{a3[17]}}, a3};
                mb  <= tp3;
            end
            4'd7: acc <= acc + mp;
            4'd8: acc <= acc + mp;
            4'd9: acc <= acc + mp;
            4'd10: begin : x_in                // x, then gain x
                reg signed [23:0] x;
                x  = sat24((acc + 43'sd256) >>> 9);
                ma <= {x[23], x};
                mb <= $signed({1'b0, gain[r]});
            end
            4'd11: begin
                ma <= {lp[r][23], lp[r]};
                mb <= $signed({1'b0, pole[r]});
            end
            4'd13: acc <= mp;
            4'd14: begin : y_out
                reg signed [23:0] y;
                y = sat24((acc + mp + 43'sd65536) >>> 17);
                lp[r]   <= y;
                wave[r] <= y;
                len[r]  <= len[r] + inc[r];
                st      <= 4'd0;
                if (r == 4'd15)
                    state <= S_JUNC;
                r <= r + 1'b1;
            end
            default: ;
            endcase
        end
        S_JUNC: begin                          // J = i*2 + j: E = J^1, S = J^2
            for (i = 0; i < 4; i = i + 1)
                p[i] <= sat24(($signed(wave[i * 4 + 0]) + $signed(wave[i * 4 + 1]) +
                               $signed(wave[(i ^ 1) * 4 + 2]) + $signed(wave[(i ^ 2) * 4 + 3]) + 48'sd1) >>> 1);
            state <= S_JIN;
        end
        S_JIN: begin
            p[0]  <= sat24($signed(p[0]) + $signed(inq));
            r     <= 4'd0;
            state <= S_SCAT;
        end
        S_SCAT: begin                          // ring writes, stochastic rounding 24 -> 18 bits
            rng    <= rn;
            ram_we <= 1'b1;
            ram_wa <= {r, w - 11'd1};
            ram_wd <= sat18(($signed({{7{sv[24]}}, sv}) + $signed({26'd0, rn[31:26]})) >>> 6);
            r      <= r + 1'b1;
            if (r == 4'd15) begin
                out   <= sat24(($signed(p[3]) + $signed(p[1]) + 48'sd1) >>> 1);
                w     <= w + 1'b1;
                cnt   <= cnt + 1'b1;
                state <= S_IDLE;
            end
        end
        default: state <= S_IDLE;
        endcase
    end
endmodule
