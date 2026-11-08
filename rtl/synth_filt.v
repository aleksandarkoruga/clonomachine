// Wavetable synth, filter engine (prototype): the quad morph filter's four slots for NL voices,
// as microcode (scripts/synth_asm.py -> sy_ucode.hex). Thread t runs slot t's program; the NL
// voices run it together (lanes: one datapath each, one instruction stream). Threads take turns
// every cycle (a barrel of 4), so an instruction's result is written before the same thread's next
// one reads it; a function (tanh, 1/x) takes the thread's next turn as well. Each thread runs its
// program once per sub-sample (two per output sample): at most 128 issue slots per run.
//
// Pipeline (edges counted from the thread's fetch F):
//   F address -> F+1 the ROM's word -> F+2 decode, operands (LUT RAM register files, 3 read ports)
//   -> F+3 partial products / ALU -> F+4 sum, saturate -> written at F+5 (functions: at F+9, the
//   thread's skipped turn); the thread's next fetch is F+4, its operands F+6.
// Registers: r0..r31 per instance {thread, lane} (128 per lane), r32..r63 constants shared by all,
// all signed 32-bit Q8.24. The soft core writes coefficients and constants while the engine is
// idle (we is honoured only then: 'busy' low).
//
// Control writes (waddr): {2'b00, thread[1:0], lane[1:0], reg[4:0]} an instance register,
// {2'b01, ..., reg[4:0]} a constant (r32 + reg), {2'b10, ..., thread[1:0]} the slot's program start.
// Inputs: x[sub][lane] (Q8.24) loaded before start through in_*; outputs y[sub][thread][lane]
// through y_ra, double-buffered by output sample (the half last finished is readable).
`default_nettype none
module synth_filt #(
    parameter NL         = 3,
    parameter UCODE_HEX  = "sy_ucode.hex",
    parameter FFN_HEX    = "sy_ffn.hex"
) (
    input  wire               clk,
    input  wire               rst,
    input  wire               start,
    output wire               busy,
    input  wire               we,
    input  wire [10:0]        waddr,
    input  wire [31:0]        wdata,
    input  wire               in_we,
    input  wire [2:0]         in_a,          // {sub, lane}
    input  wire [31:0]        in_d,
    input  wire [4:0]         y_ra,          // {sub, thread[1:0], lane[1:0]}
    output reg  signed [31:0] y_q,
    output reg  [10:0]        cyc_max
);
    localparam OP_NOP = 4'd0, OP_MAC = 4'd1, OP_MSUB = 4'd2, OP_ADD = 4'd3, OP_SUB = 4'd4, OP_FN = 4'd5,
               OP_MAX = 4'd6, OP_MIN = 4'd7, OP_LDX = 4'd8, OP_OUT = 4'd9, OP_END = 4'd10, OP_ABS = 4'd12;

    function signed [31:0] sat32;
        input signed [63:0] v;
        begin
            if (v > 64'sd2147483647)       sat32 = 32'sh7FFFFFFF;
            else if (v < -64'sd2147483648) sat32 = 32'sh80000000;
            else                           sat32 = v[31:0];
        end
    endfunction
    function [4:0] clz31;   // leading zeros of a positive 31-bit value, 0..31
        input [30:0] x;
        integer k;
        begin
            clz31 = 31;
            for (k = 0; k < 31; k = k + 1)
                if (x[k]) clz31 = 30 - k;
        end
    endfunction

    // ------------------------------------------------------------------ ROMs
    reg  [35:0] ucode [0:511];
    initial $readmemh(UCODE_HEX, ucode);
    reg  [8:0]  rom_a;
    reg  [35:0] rom_q;
    always @(posedge clk) rom_q <= ucode[rom_a];

    // ------------------------------------------------------------------ thread state
    reg  [1:0]  cnt;                // the thread whose turn it is
    reg  [8:0]  pc     [0:3];
    reg  [8:0]  pstart [0:3];
    reg         act    [0:3];
    reg         sub    [0:3];
    reg         bub    [0:3];       // skip the next turn (after a function)
    reg  [10:0] cyc;
    reg         run;
    reg         half;               // output buffer being written
    wire        any_act = act[0] || act[1] || act[2] || act[3];
    // pipeline valid bits / tags
    reg         f_v, r_v, d_v, e1_v, e2_v, w_v;
    reg  [1:0]  f_t, r_t, d_t, e1_t, e2_t, w_t;
    reg         d_sub, e1_sub, e2_sub;
    reg  [3:0]  e1_op, e2_op;
    reg  [5:0]  e1_d, e2_d;
    // function-unit pipeline (writes at F + 8)
    reg  [6:0]  fv;                 // stage valid bits
    reg  [1:0]  ft [0:6];
    reg  [5:0]  fd [0:6];

    integer i;
    always @(posedge clk) begin
        if (rst) begin
            run <= 0;
            cnt <= 0;
            f_v <= 0;
            r_v <= 0;
            half <= 0;
            cyc_max <= 0;
            for (i = 0; i < 4; i = i + 1) begin
                act[i] <= 0;
                bub[i] <= 0;
                pstart[i] <= 0;
            end
        end else begin
            if (we && !run && waddr[10:9] == 2'b10)
                pstart[waddr[1:0]] <= wdata[8:0];
            cnt <= cnt + 1'b1;
            if (start && !run) begin
                run <= 1;
                cyc <= 0;
                half <= ~half;
                for (i = 0; i < 4; i = i + 1) begin
                    act[i] <= 1;
                    sub[i] <= 0;
                    bub[i] <= 0;
                    pc[i] <= pstart[i];
                end
            end else if (run) begin
                cyc <= cyc + 1'b1;
                if (!any_act && !f_v && !r_v && !d_v && !e1_v && !e2_v && !w_v && fv == 0) begin
                    run <= 0;
                    if (cyc > cyc_max)
                        cyc_max <= cyc;
                end
            end
            // F: the thread whose turn it is fetches (unless it's done or in a function's bubble)
            f_v <= 1'b0;
            if (run && act[cnt]) begin
                if (bub[cnt])
                    bub[cnt] <= 1'b0;
                else begin
                    rom_a <= pc[cnt];
                    pc[cnt] <= pc[cnt] + 1'b1;
                    f_v <= 1'b1;
                    f_t <= cnt;
                end
            end
            // F+1: the word arrives
            r_v <= f_v;
            r_t <= f_t;
            // F+2: END restarts the program for the second sub-sample, then the thread is done;
            // a function takes the thread's next turn
            if (r_v && rom_q[35:32] == OP_END) begin
                if (sub[r_t])
                    act[r_t] <= 1'b0;
                else begin
                    sub[r_t] <= 1'b1;
                    pc[r_t] <= pstart[r_t];
                end
            end
            if (r_v && rom_q[35:32] == OP_FN)
                bub[r_t] <= 1'b1;
        end
    end

    // ------------------------------------------------------------------ register files
    // per lane 128 x 32 (3 read copies), constants 32 x 32 (3 read copies)
    wire [35:0] ins = rom_q;
    wire [3:0]  d_op = ins[35:32];
    wire [5:0]  rd = ins[31:26], ra = ins[25:20], rb = ins[19:14], rc = ins[13:8];
    wire [7:0]  imm = ins[7:0];
    // write port: engine results (W) or the soft core's writes while idle
    reg         wr_en;
    reg  [6:0]  wr_a;                     // {thread, reg}
    reg  [NL-1:0] wr_lanes;
    reg  signed [31:0] wr_d [0:NL-1];
    reg         cw_en;
    reg  [4:0]  cw_a;
    reg  [31:0] cw_d;
    reg  signed [31:0] ca_q, cb_q, cc_q;   // constants read
    reg  [31:0] cst_a [0:31];
    reg  [31:0] cst_b [0:31];
    reg  [31:0] cst_c [0:31];
    always @(posedge clk)
        if (cw_en) begin
            cst_a[cw_a] <= cw_d;
            cst_b[cw_a] <= cw_d;
            cst_c[cw_a] <= cw_d;
        end
    // operands for each lane, registered at D
    reg  signed [31:0] opa [0:NL-1];
    reg  signed [31:0] opb [0:NL-1];
    reg  signed [31:0] opc [0:NL-1];
    reg  signed [31:0] xin [0:7];          // {sub, lane[1:0]}
    always @(posedge clk)
        if (in_we)
            xin[in_a] <= in_d;
    wire f_sub_now = sub[r_t];
    genvar gl;
    generate for (gl = 0; gl < NL; gl = gl + 1) begin : lane
        localparam [1:0] LI = gl;
        reg [31:0] rfa [0:127];
        reg [31:0] rfb [0:127];
        reg [31:0] rfc [0:127];
        always @(posedge clk)
            if (wr_en && wr_lanes[gl]) begin
                rfa[wr_a] <= wr_d[gl];
                rfb[wr_a] <= wr_d[gl];
                rfc[wr_a] <= wr_d[gl];
            end
        always @(posedge clk) begin
            opa[gl] <= (d_op == OP_LDX) ? xin[{f_sub_now, LI}]
                     : ra[5] ? cst_a[ra[4:0]] : rfa[{r_t, ra[4:0]}];
            opb[gl] <= rb[5] ? cst_b[rb[4:0]] : rfb[{r_t, rb[4:0]}];
            opc[gl] <= rc[5] ? cst_c[rc[4:0]] : rfc[{r_t, rc[4:0]}];
        end
    end endgenerate

    // ------------------------------------------------------------------ D -> E1 -> E2 -> W
    // multiplier: a x b = (ah 2^16 + al)(bh 2^16 + bl), 4 partial products
    reg  signed [33:0] p_ll [0:NL-1];
    reg  signed [33:0] p_lh [0:NL-1];
    reg  signed [33:0] p_hl [0:NL-1];
    reg  signed [33:0] p_hh [0:NL-1];
    reg  signed [31:0] alu  [0:NL-1];
    reg  signed [31:0] e1c  [0:NL-1];
    reg  signed [31:0] res  [0:NL-1];
    reg  signed [31:0] outv [0:NL-1];
    reg         e1_isout, e2_isout;
    reg  [5:0]  res_d;
    always @(posedge clk) begin
        if (rst) begin
            d_v <= 0; e1_v <= 0; e2_v <= 0; w_v <= 0;
        end else begin
            // D: operands registered (above); carry the instruction
            d_v <= r_v && d_op != OP_END && d_op != OP_NOP && d_op != OP_FN;
            d_t <= r_t;
            d_sub <= sub[r_t];
            e1_op <= d_op;
            e1_d  <= rd;
            // E1
            e1_v <= d_v;
            e1_t <= d_t;
            e1_sub <= d_sub;
            e2_op <= e1_op;
            e2_d  <= e1_d;
            e1_isout <= e1_op == OP_OUT;
            for (i = 0; i < NL; i = i + 1) begin
                p_ll[i] <= $signed({1'b0, opa[i][15:0]}) * $signed({1'b0, opb[i][15:0]});
                p_lh[i] <= $signed({1'b0, opa[i][15:0]}) * $signed(opb[i][31:16]);
                p_hl[i] <= $signed(opa[i][31:16]) * $signed({1'b0, opb[i][15:0]});
                p_hh[i] <= $signed(opa[i][31:16]) * $signed(opb[i][31:16]);
                e1c[i] <= opc[i];
                case (e1_op)
                OP_ADD:  alu[i] <= sat32($signed(opa[i]) + $signed(opb[i]));
                OP_SUB:  alu[i] <= sat32($signed(opa[i]) - $signed(opb[i]));
                OP_MAX:  alu[i] <= (opa[i] > opb[i]) ? opa[i] : opb[i];
                OP_MIN:  alu[i] <= (opa[i] < opb[i]) ? opa[i] : opb[i];
                OP_ABS:  alu[i] <= (opa[i] < 0) ? sat32(-$signed(opa[i])) : opa[i];
                default: alu[i] <= opa[i];   // LDX, OUT
                endcase
            end
            // E2
            e2_v <= e1_v;
            e2_t <= e1_t;
            res_d <= e2_d;           // the destination of the result in res
            e2_sub <= e1_sub;
            e2_isout <= e1_isout;
            for (i = 0; i < NL; i = i + 1) begin
                if (e2_op == OP_MAC || e2_op == OP_MSUB) begin : mac
                    reg signed [63:0] prod;
                    prod = $signed({{30{p_ll[i][33]}}, p_ll[i]}) + ($signed({{30{p_lh[i][33]}}, p_lh[i]}) <<< 16)
                         + ($signed({{30{p_hl[i][33]}}, p_hl[i]}) <<< 16) + ($signed({{30{p_hh[i][33]}}, p_hh[i]}) <<< 32);
                    res[i] <= (e2_op == OP_MAC) ? sat32($signed(e1c[i]) + (prod >>> 24))
                                                : sat32($signed(e1c[i]) - (prod >>> 24));
                end else
                    res[i] <= alu[i];
            end
            // W
            w_v <= e2_v;
            w_t <= e2_t;
        end
    end
    // e2_op at E2 is the op whose partial products are in p_*: e1_op moved one stage
    // (the case above reads e2_op, which holds the op of the values in p_* / alu)

    // ------------------------------------------------------------------ function unit
    // F+1 (D): operand; F+2: range reduction, table address; F+3: table read; F+4: the table
    // step; F+5: interpolation product; F+6: value; F+7: scaled; written in the thread's own slot
    reg         fn_v1, fn_v2, fn_v3, fn_v4, fn_v5, fn_v6, fn_v7;
    reg  [1:0]  fn_t1, fn_t2, fn_t3, fn_t4, fn_t5, fn_t6, fn_t7;
    reg  [5:0]  fn_d1, fn_d2, fn_d3, fn_d4, fn_d5, fn_d6, fn_d7;
    reg         fn_k1, fn_k2, fn_k3, fn_k4, fn_k5, fn_k6;     // 0 tanh, 1 recip
    reg  [NL-1:0] fn_neg2, fn_neg3, fn_neg4, fn_neg5, fn_neg6;
    reg  [NL*5-1:0] fn_lz2, fn_lz3, fn_lz4, fn_lz5, fn_lz6;
    reg  [NL-1:0] fn_big2, fn_big3, fn_big4, fn_big5, fn_big6; // tanh: |x| >= 8; recip: x <= 0
    reg  [15:0] fn_fr2 [0:NL-1];
    reg  [15:0] fn_fr3 [0:NL-1];
    reg  [15:0] fn_fr4 [0:NL-1];
    reg  [9:0]  fn_ad  [0:NL-1];
    reg  [17:0] fn_q0  [0:NL-1];
    reg  [17:0] fn_q1  [0:NL-1];
    reg  signed [17:0] fn_df [0:NL-1];   // the table step (|step| < 2^12: one 18 x 18 multiply)
    reg  [17:0] fn_b4  [0:NL-1];
    reg  signed [34:0] fn_ip [0:NL-1];
    reg  [17:0] fn_b5  [0:NL-1];
    reg  [17:0] fn_v17 [0:NL-1];
    reg  signed [31:0] fn_r8 [0:NL-1];
    generate for (gl = 0; gl < NL; gl = gl + 1) begin : fnrom
        reg [17:0] t [0:1023];
        initial $readmemh(FFN_HEX, t);
        always @(posedge clk) begin
            fn_q0[gl] <= t[fn_ad[gl]];
            fn_q1[gl] <= t[fn_ad[gl] + 1'b1];
        end
    end endgenerate
    reg  [31:0] fa;
    reg  [4:0]  lzv;
    always @(posedge clk) begin
        if (rst) begin
            fn_v1 <= 0; fn_v2 <= 0; fn_v3 <= 0; fn_v4 <= 0; fn_v5 <= 0; fn_v6 <= 0; fn_v7 <= 0;
        end else begin
            fn_v1 <= r_v && d_op == OP_FN;
            fn_t1 <= r_t;
            fn_d1 <= rd;
            fn_k1 <= imm[0];
            // F+2: operands are in opa (registered at D)
            fn_v2 <= fn_v1; fn_t2 <= fn_t1; fn_d2 <= fn_d1; fn_k2 <= fn_k1;
            for (i = 0; i < NL; i = i + 1) begin
                if (!fn_k1) begin   // tanh: |x| x 32 -> index [26:19], fraction [18:3]
                    fa = opa[i][31] ? -opa[i] : opa[i];
                    fn_neg2[i] <= opa[i][31];
                    fn_big2[i] <= fa[31:27] != 0;
                    fn_ad[i] <= {2'b00, fa[26:19]};
                    fn_fr2[i] <= fa[18:3];
                    fn_lz2[i*5 +: 5] <= 0;
                end else begin      // 1/x: leading one moved to bit 30, index [29:22], fraction [21:6]
                    lzv = clz31(opa[i][30:0]);
                    fa = opa[i] << lzv;
                    fn_neg2[i] <= 1'b0;
                    fn_big2[i] <= opa[i][31] || opa[i] == 0;
                    fn_ad[i] <= {2'b10, fa[29:22]};
                    fn_fr2[i] <= fa[21:6];
                    fn_lz2[i*5 +: 5] <= lzv;
                end
            end
            // F+3: table read in flight
            fn_v3 <= fn_v2; fn_t3 <= fn_t2; fn_d3 <= fn_d2; fn_k3 <= fn_k2;
            fn_neg3 <= fn_neg2; fn_big3 <= fn_big2; fn_lz3 <= fn_lz2;
            for (i = 0; i < NL; i = i + 1) fn_fr3[i] <= fn_fr2[i];
            // The data below run one stage ahead of the tags' numbering: the table words arrive
            // with the *3 flags, and the result is ready with fn_v7 (written then: F+9, the
            // thread's skipped turn). One operation per cycle:
            // the table words -> their step registered (the block RAM's output goes to a subtract only)
            fn_v4 <= fn_v3; fn_t4 <= fn_t3; fn_d4 <= fn_d3; fn_k4 <= fn_k3;
            fn_neg4 <= fn_neg3; fn_big4 <= fn_big3; fn_lz4 <= fn_lz3;
            for (i = 0; i < NL; i = i + 1) begin
                fn_fr4[i] <= fn_fr3[i];
                fn_df[i] <= fn_q1[i] - fn_q0[i];
                fn_b4[i] <= fn_q0[i];
            end
            // the interpolation product
            fn_v5 <= fn_v4; fn_t5 <= fn_t4; fn_d5 <= fn_d4; fn_k5 <= fn_k4;
            fn_neg5 <= fn_neg4; fn_big5 <= fn_big4; fn_lz5 <= fn_lz4;
            for (i = 0; i < NL; i = i + 1) begin
                fn_ip[i] <= fn_df[i] * $signed({1'b0, fn_fr4[i]});
                fn_b5[i] <= fn_b4[i];
            end
            // the value, Q0.17
            fn_v6 <= fn_v5; fn_t6 <= fn_t5; fn_d6 <= fn_d5; fn_k6 <= fn_k5;
            fn_neg6 <= fn_neg5; fn_big6 <= fn_big5; fn_lz6 <= fn_lz5;
            for (i = 0; i < NL; i = i + 1)
                fn_v17[i] <= fn_b5[i] + (fn_ip[i] >>> 16);
            // scaled to Q8.24
            fn_v7 <= fn_v6; fn_t7 <= fn_t6; fn_d7 <= fn_d6;
            for (i = 0; i < NL; i = i + 1) begin : sc
                reg [47:0] r;
                if (!fn_k6) begin
                    // tanh: Q0.17 << 7, |x| >= 8 -> 1
                    r = fn_big6[i] ? 48'd16777216 : {23'd0, fn_v17[i], 7'd0};
                    fn_r8[i] <= fn_neg6[i] ? -$signed(r[31:0]) : $signed(r[31:0]);
                end else begin
                    // 1/x: x = m 2^(6 - lz), 1/x = (1/m) 2^(lz - 6) = T17 << (lz + 1) in Q8.24
                    r = {30'd0, fn_v17[i]} << (fn_lz6[i*5 +: 5] + 1);
                    fn_r8[i] <= (fn_big6[i] || r[47:31] != 0) ? 32'sh7FFFFFFF : $signed(r[31:0]);
                end
            end
        end
    end
    always @(*) fv = {fn_v7, fn_v6, fn_v5, fn_v4, fn_v3, fn_v2, fn_v1};

    // ------------------------------------------------------------------ writes
    // A MAC / ALU result is written at F+4 (from E2's registers), a function's at F+8: the function's
    // slot is its thread's skipped turn, so they never collide. The soft core writes while idle.
    reg  signed [31:0] yb [0:63];          // {half, sub, thread, lane}
    always @(*) begin
        wr_en = 1'b0;
        wr_a = 0;
        wr_lanes = {NL{1'b1}};
        cw_en = 1'b0;
        cw_a = waddr[4:0];
        cw_d = wdata;
        for (i = 0; i < NL; i = i + 1)
            wr_d[i] = res[i];
        if (e2_v && !e2_isout && res_d < 6'd32) begin
            wr_en = 1'b1;
            wr_a = {e2_t, res_d[4:0]};
        end else if (fn_v7) begin
            wr_en = fn_d7 < 6'd32;
            wr_a = {fn_t7, fn_d7[4:0]};
            for (i = 0; i < NL; i = i + 1)
                wr_d[i] = fn_r8[i];
        end else if (we && !run && waddr[10:9] == 2'b00) begin
            wr_en = 1'b1;
            wr_a = {waddr[8:7], waddr[4:0]};
            wr_lanes = {{(NL-1){1'b0}}, 1'b1} << waddr[6:5];
            for (i = 0; i < NL; i = i + 1)
                wr_d[i] = wdata;
        end
        if (we && !run && waddr[10:9] == 2'b01)
            cw_en = 1'b1;
    end
    always @(posedge clk) begin
        if (e2_v && e2_isout)
            for (i = 0; i < NL; i = i + 1)
                yb[{half, e2_sub, e2_t, i[1:0]}] <= res[i];
        y_q <= yb[{~half, y_ra}];
    end

    assign busy = run;
endmodule
`default_nettype wire
