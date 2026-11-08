// Wavetable synth, oscillator unit (prototype). Runs beside the sampler engine in the audio clock
// domain: on start it renders one output sample's worth of oscillator signal at 2x (two sub-
// samples) for NV voices x 2 oscillators x up to UMAX unison voices, plus a sub and noise per
// voice, and leaves per voice and sub-sample the sum routed to the filter and the sum that
// bypasses it (read through out_ra by the filter stage during the next sample period).
// Arithmetic after model/synth_engine.cpp (hardware configuration); not bit-exact with it yet.
//
// Oscillator o = 2v + {0 A, 1 B}. Tables live in SDRAM as octave mip levels (16-bit words,
// frames of N = 2^logN samples, contiguous); the soft core picks the level per oscillator and
// writes its base address. Each oscillator owns NBUF frame buffers (2^FLOG words each) in block
// RAM; a cache controller loads the frames the table position needs (64-word lines through the
// shared line queue) and plays the two around the position (smooth) or the nearest one (stepped).
// While a frame is loading the oscillator keeps playing what it has (counted in 'lag').
//
// Per unison voice ("job"): phase += inc (thru-zero FM scales inc), two warp slots (phase maps
// via log2 / exp2 tables, phase / amplitude cross modulation by the other oscillator, the sub or
// noise), then an 8-tap windowed-sinc read of both frames (256 phases, coefficients interpolated
// between phases), the frame crossfade, the warp gain and the unison gain. The next job's warps
// run while the current one reads its 8 taps.
//
// Registers (we, waddr, wdata; the soft core at its control rate):
//   osc o, waddr = {0, o[2:0], r[4:0]}:
//     0..3   inc[u]     phase increment per sub-sample, 2^32 = one cycle
//     4..7   ugain[u]   s18 Q1.16 (blend, 1/sqrt(N))
//     8      pos        [23:0] u8.16 frame position, [24] smooth
//     9      lvl        [24:0] SDRAM word address of frame 0 of the level, [28:25] logN (8..FLOG)
//     10     mode       [4:0] warp 1, [9:5] warp 2, [11:10] unison voices - 1, [12] to filter,
//                       [13] fresh (note on: play nothing until this note's frames are in), [14] on
//     11/12  wpar1/2    warp parameter (see below)
//     13/14  wparb1/2   second warp parameter
//     15     level      s18 Q1.16 into the voice mix
//     16     nframes-1  [7:0]
//     17/18  wparc1/2   third warp parameter
//   voice v, waddr = {1, 0, v[1:0], r[3:0]}:
//     0 sub inc, 1 sub gain s18 Q1.16 ([20] triangle instead of sine), 2 noise gain s18 Q1.16,
//     3 flags: [0] sub to filter, [1] noise to filter, [2] B first (A is modulated by B)
// Warp modes as model/synth_warp.h. Parameters (par / parb / parc): sync / sync window k, PWM
// 1/width: u5.12; asym / mirror knee d u32, 0.5/d and 0.5/(1-d) u6.12; bend a s1.16; flip
// threshold u32; quantize: low bits cleared; odd/even e s1.16, norm u1.16; FM (other / sub /
// noise) index s2.16 cycles; AM / RM depth u1.16; thru-zero FM depth Q3.15. The multiplier's
// result is there three micro-steps after its operands. Signals: oscillator / sub / noise s18
// Q2.15, voice sums s24 Q8.15.
`default_nettype none
module synth_osc #(
    parameter NV       = 3,
    parameter UMAX     = 4,
    parameter FLOG     = 10,
    parameter NBUF     = 2,
    parameter SINC_HEX = "sy_sinc.hex",
    parameter FN_HEX   = "sy_fn.hex"
) (
    input  wire               clk,
    input  wire               rst,
    input  wire               start,          // one per output sample
    output wire               busy,
    input  wire               we,
    input  wire [9:0]         waddr,
    input  wire [31:0]        wdata,
    output wire               req_valid,
    input  wire               req_ready,
    output wire [24:0]        req_addr,
    input  wire               fill_valid,
    input  wire [15:0]        fill_data,
    input  wire [2:0]         out_ra,         // {sub, voice}
    output reg  signed [23:0] out_filt,       // registered read
    output reg  signed [23:0] out_direct,
    output reg  [31:0]        lag,            // oscillator-samples played away from the wanted frames
    output reg  [10:0]        cyc_max         // most cycles a sample took
);
    localparam NO = 2 * NV;
    localparam OB = 3;                        // oscillator index bits
    localparam UB = 2;                        // unison index bits (UMAX <= 4)
    localparam BB = (NBUF > 2) ? 2 : 1;       // buffer index bits
    localparam W_OFF = 0, W_SYNC = 1, W_SYNCWIN = 2, W_BEND_P = 3, W_BEND_M = 4, W_BEND_PM = 5,
               W_PWM = 6, W_ASYM_P = 7, W_ASYM_M = 8, W_ASYM_PM = 9, W_FLIP = 10, W_MIRROR = 11,
               W_QUANT = 12, W_ODDEVEN = 13, W_FM = 14, W_FM_SUB = 15, W_FM_NOISE = 16, W_AM = 17,
               W_RM = 18, W_TZFM = 19;

    function signed [17:0] sat18;
        input signed [47:0] v;
        begin
            if (v > 48'sd131071)       sat18 = 18'sd131071;
            else if (v < -48'sd131072) sat18 = -18'sd131072;
            else                       sat18 = v[17:0];
        end
    endfunction
    function signed [23:0] sat24;
        input signed [47:0] v;
        begin
            if (v > 48'sd8388607)       sat24 = 24'sd8388607;
            else if (v < -48'sd8388608) sat24 = -24'sd8388608;
            else                        sat24 = v[23:0];
        end
    endfunction
    function [5:0] clz32;
        input [31:0] x;
        integer k;
        begin
            clz32 = 32;
            for (k = 0; k < 32; k = k + 1)
                if (x[k]) clz32 = 31 - k;
        end
    endfunction
    function is_xmod;  // needs the other oscillator's output of this sub-sample
        input [4:0] w;
        is_xmod = w == W_FM || w == W_AM || w == W_RM || w == W_TZFM;
    endfunction

    // ------------------------------------------------------------------ registers
    reg [31:0]        r_inc   [0:NO*UMAX-1];
    reg signed [17:0] r_ugain [0:NO*UMAX-1];
    reg [24:0]        r_pos   [0:NO-1];
    reg [24:0]        r_base  [0:NO-1];
    reg [3:0]         r_logn  [0:NO-1];
    reg [4:0]         r_w1    [0:NO-1];
    reg [4:0]         r_w2    [0:NO-1];
    reg [1:0]         r_uni   [0:NO-1];
    reg               r_tof   [0:NO-1];
    reg               r_on    [0:NO-1];
    reg [31:0]        r_wp1   [0:NO-1];
    reg [31:0]        r_wp2   [0:NO-1];
    reg [17:0]        r_wb1   [0:NO-1];
    reg [17:0]        r_wb2   [0:NO-1];
    reg [17:0]        r_wc1   [0:NO-1];
    reg [17:0]        r_wc2   [0:NO-1];
    reg signed [17:0] r_level [0:NO-1];
    reg [7:0]         r_nfm1  [0:NO-1];
    reg [31:0]        r_subinc [0:NV-1];
    reg signed [17:0] r_subg   [0:NV-1];
    reg               r_subtri [0:NV-1];
    reg signed [17:0] r_noiseg [0:NV-1];
    reg [2:0]         r_vflags [0:NV-1];
    wire [OB-1:0]     wo = waddr[7:5];
    wire [4:0]        wr = waddr[4:0];
    wire              fresh_set = we && !waddr[8] && wr == 5'd10 && wdata[13];
    always @(posedge clk) begin
        if (we && !waddr[8]) begin
            case (wr)
            5'd0, 5'd1, 5'd2, 5'd3: r_inc[{wo, wr[1:0]}] <= wdata;
            5'd4, 5'd5, 5'd6, 5'd7: r_ugain[{wo, wr[1:0]}] <= wdata[17:0];
            5'd8:  r_pos[wo] <= wdata[24:0];
            5'd9:  begin r_base[wo] <= wdata[24:0]; r_logn[wo] <= wdata[28:25]; end
            5'd10: begin
                       r_w1[wo] <= wdata[4:0];
                       r_w2[wo] <= wdata[9:5];
                       r_uni[wo] <= wdata[11:10];
                       r_tof[wo] <= wdata[12];
                       r_on[wo] <= wdata[14];
                   end
            5'd11: r_wp1[wo] <= wdata;
            5'd12: r_wp2[wo] <= wdata;
            5'd13: r_wb1[wo] <= wdata[17:0];
            5'd14: r_wb2[wo] <= wdata[17:0];
            5'd15: r_level[wo] <= wdata[17:0];
            5'd16: r_nfm1[wo] <= wdata[7:0];
            5'd17: r_wc1[wo] <= wdata[17:0];
            5'd18: r_wc2[wo] <= wdata[17:0];
            default: ;
            endcase
        end
        if (we && waddr[8] && !waddr[7]) begin
            case (waddr[3:0])
            4'd0: r_subinc[waddr[5:4]] <= wdata;
            4'd1: begin r_subg[waddr[5:4]] <= wdata[17:0]; r_subtri[waddr[5:4]] <= wdata[20]; end
            4'd2: r_noiseg[waddr[5:4]] <= wdata[17:0];
            4'd3: r_vflags[waddr[5:4]] <= wdata[2:0];
            default: ;
            endcase
        end
    end

    // ------------------------------------------------------------------ ROMs
    reg  [17:0] sinc [0:2047];
    reg  [17:0] fnr  [0:1023];
    initial begin
        $readmemh(SINC_HEX, sinc);
        $readmemh(FN_HEX, fnr);
    end
    reg  [10:0] sc_a0, sc_a1;
    reg  [17:0] sc_q0, sc_q1;
    always @(posedge clk) begin
        sc_q0 <= sinc[sc_a0];
        sc_q1 <= sinc[sc_a1];
    end
    reg  [9:0]  fn_a0;
    reg  [17:0] fn_q0;
    always @(posedge clk)
        fn_q0 <= fnr[fn_a0];

    // ------------------------------------------------------------------ frame cache RAMs
    // buffer b of oscillator o at {o, offset}: all buffers are read at one address (both frames of
    // a read use the same offset); the second port takes the line fills
    localparam CAW = OB + FLOG;
    reg  [CAW-1:0] cr_a;
    wire [15:0]    crq [0:NBUF-1];
    reg  [NBUF-1:0] cw_we;
    reg  [CAW-1:0] cw_a;
    reg  [15:0]    cw_d;
    genvar gb;
    generate for (gb = 0; gb < NBUF; gb = gb + 1) begin : cache
        reg [15:0] mem [0:NO * (1 << FLOG) - 1];   // {o, offset}, o < NO
        reg [15:0] q;
        always @(posedge clk) begin
            q <= mem[cr_a];
            if (cw_we[gb]) mem[cw_a] <= cw_d;
        end
        assign crq[gb] = q;
    end endgenerate

    // ------------------------------------------------------------------ cache controller
    // per oscillator and buffer: frame, logN, state (0 empty, 1 loading, 2 ready)
    reg [7:0]  tg_f  [0:NO*NBUF-1];
    reg [3:0]  tg_n  [0:NO*NBUF-1];
    reg [1:0]  tg_s  [0:NO*NBUF-1];
    // what each oscillator plays this sample
    reg [BB-1:0] pl_a  [0:NO-1];
    reg [BB-1:0] pl_b  [0:NO-1];
    reg [15:0]   pl_u  [0:NO-1];
    reg [3:0]    pl_n  [0:NO-1];
    reg          pl_ok [0:NO-1];
    reg          fresh [0:NO-1];
    // one load per oscillator at a time
    reg          ld_act  [0:NO-1];
    reg [BB-1:0] ld_buf  [0:NO-1];
    reg [24:0]   ld_addr [0:NO-1];   // next line
    reg [4:0]    ld_left [0:NO-1];   // lines still to request - 1
    reg [4:0]    ld_line [0:NO-1];   // index of the next line
    // tag FIFO of the requested lines: {last, osc, buf, line}
    localparam TW = 1 + OB + BB + 5;
    reg [TW-1:0] tq [0:31];
    reg [4:0]    tq_wp, tq_rp;
    reg [5:0]    tq_n;
    reg [5:0]    fw;                 // word within the line being filled
    wire [TW-1:0] tqh = tq[tq_rp];
    wire [OB-1:0] tq_o = tqh[OB + BB + 4:BB + 5];
    wire [BB-1:0] tq_b = tqh[BB + 4:5];

    // request issuer: round robin over the oscillators with an active load
    reg  [OB-1:0] rq_o;
    reg           rq_v;
    reg  [24:0]   rq_a;
    assign req_valid = rq_v;
    assign req_addr  = rq_a;
    wire          rq_fire = rq_v && req_ready;

    // resolve pass, once per sample before the jobs
    reg  [OB-1:0] cc_o;
    reg  [1:0]    cc_st;
    reg  [7:0]    cc_f0, cc_f1;
    reg  [15:0]   cc_u;
    reg           cc_need1;
    reg  [NBUF-1:0] cc_h0, cc_h1, cc_busy, cc_p0, cc_p1;
    reg           cc_run, cc_done;

    integer i;
    reg [BB-1:0]  b0, b1, vic;
    reg           h0, h1, vok;
    reg [OB-1:0]  nx;
    reg           nxv;
    always @(posedge clk) begin
        if (rst) begin
            cc_run <= 0;
            cc_done <= 0;
            rq_v <= 0;
            rq_o <= 0;
            tq_wp <= 0;
            tq_rp <= 0;
            tq_n <= 0;
            fw <= 0;
            lag <= 0;
            cw_we <= 0;
            for (i = 0; i < NO * NBUF; i = i + 1)
                tg_s[i] <= 0;
            for (i = 0; i < NO; i = i + 1) begin
                ld_act[i] <= 0;
                pl_ok[i] <= 0;
                fresh[i] <= 1;
            end
        end else begin
            cw_we <= 0;
            if (fresh_set)
                fresh[wo] <= 1'b1;
            // ---- resolve pass
            if (start) begin
                cc_run <= 1;
                cc_done <= 0;
                cc_o <= 0;
                cc_st <= 0;
            end else if (cc_run) begin
                case (cc_st)
                2'd0: begin   // the wanted frames
                    if (r_pos[cc_o][24]) begin
                        cc_f0 <= r_pos[cc_o][23:16];
                        cc_u  <= r_pos[cc_o][15:0];
                        cc_need1 <= r_pos[cc_o][15:0] != 0 && r_pos[cc_o][23:16] < r_nfm1[cc_o];
                        cc_f1 <= r_pos[cc_o][23:16] + 1'b1;
                    end else begin
                        cc_f0 <= ({1'b0, r_pos[cc_o][23:16]} + r_pos[cc_o][15] > {1'b0, r_nfm1[cc_o]})
                                 ? r_nfm1[cc_o] : r_pos[cc_o][23:16] + r_pos[cc_o][15];
                        cc_u  <= 0;
                        cc_need1 <= 0;
                        cc_f1 <= 0;
                    end
                    cc_st <= 2'd1;
                end
                2'd1: begin   // compare with the tags
                    for (i = 0; i < NBUF; i = i + 1) begin
                        cc_h0[i] <= tg_s[cc_o * NBUF + i] == 2'd2 && tg_f[cc_o * NBUF + i] == cc_f0
                                    && tg_n[cc_o * NBUF + i] == r_logn[cc_o];
                        cc_h1[i] <= tg_s[cc_o * NBUF + i] == 2'd2 && tg_f[cc_o * NBUF + i] == cc_f1
                                    && tg_n[cc_o * NBUF + i] == r_logn[cc_o];
                        // present: ready or loading
                        cc_p0[i] <= tg_s[cc_o * NBUF + i] != 2'd0 && tg_f[cc_o * NBUF + i] == cc_f0
                                    && tg_n[cc_o * NBUF + i] == r_logn[cc_o];
                        cc_p1[i] <= tg_s[cc_o * NBUF + i] != 2'd0 && tg_f[cc_o * NBUF + i] == cc_f1
                                    && tg_n[cc_o * NBUF + i] == r_logn[cc_o];
                        // not a victim: loading, or holding a wanted frame
                        cc_busy[i] <= tg_s[cc_o * NBUF + i] == 2'd1
                                      || (tg_s[cc_o * NBUF + i] != 2'd0 && tg_n[cc_o * NBUF + i] == r_logn[cc_o]
                                          && (tg_f[cc_o * NBUF + i] == cc_f0
                                              || (cc_need1 && tg_f[cc_o * NBUF + i] == cc_f1)));
                    end
                    cc_st <= 2'd2;
                end
                default: begin   // play selection, maybe start a load
                    h0 = 0; h1 = 0; vok = 0; b0 = 0; b1 = 0; vic = 0;
                    for (i = NBUF - 1; i >= 0; i = i - 1) begin
                        if (cc_h0[i]) begin h0 = 1; b0 = i; end
                        if (cc_h1[i]) begin h1 = 1; b1 = i; end
                        if (!cc_busy[i]) begin vok = 1; vic = i; end
                    end
`ifdef SYNTH_DEBUG
                    if (cc_o == 0)
                        $display("cc o%0d f0 %0d f1 %0d need1 %0d h0 %0d h1 %0d vok %0d vic %0d ld_act %0d tags s %0d/%0d f %0d/%0d",
                                 cc_o, cc_f0, cc_f1, cc_need1, h0, h1, vok, vic, ld_act[cc_o], tg_s[0], tg_s[1], tg_f[0], tg_f[1]);
`endif
                    if (h0 && (h1 || !cc_need1)) begin
                        pl_a[cc_o]  <= b0;
                        pl_b[cc_o]  <= cc_need1 ? b1 : b0;
                        pl_u[cc_o]  <= cc_need1 ? cc_u : 16'd0;
                        pl_n[cc_o]  <= r_logn[cc_o];
                        pl_ok[cc_o] <= 1'b1;
                        fresh[cc_o] <= 1'b0;
                    end else begin
                        if (r_on[cc_o])
                            lag <= lag + 1'b1;
                        if (fresh[cc_o])
                            pl_ok[cc_o] <= 1'b0;
                        else if (h0 || h1) begin   // one of the two: play it alone
                            pl_a[cc_o] <= h0 ? b0 : b1;
                            pl_b[cc_o] <= h0 ? b0 : b1;
                            pl_u[cc_o] <= 0;
                            pl_n[cc_o] <= r_logn[cc_o];
                            pl_ok[cc_o] <= 1'b1;
                        end else if (!ld_act[cc_o] && vok && (cc_p0 == 0 || (cc_need1 && cc_p1 == 0))
                                     && (pl_a[cc_o] == vic || pl_b[cc_o] == vic))
                            pl_ok[cc_o] <= 1'b0;   // what it played is about to be replaced
                        // load the first wanted frame that isn't there or on its way
                        if (!ld_act[cc_o] && vok && r_on[cc_o] && (cc_p0 == 0 || (cc_need1 && cc_p1 == 0))) begin
                            ld_act[cc_o]  <= 1'b1;
                            ld_buf[cc_o]  <= vic;
                            ld_addr[cc_o] <= r_base[cc_o] + ({17'd0, (cc_p0 != 0 ? cc_f1 : cc_f0)} << r_logn[cc_o]);
                            ld_left[cc_o] <= (5'd1 << (r_logn[cc_o] - 4'd6)) - 1'b1;
                            ld_line[cc_o] <= 0;
                            tg_f[cc_o * NBUF + vic] <= cc_p0 != 0 ? cc_f1 : cc_f0;
                            tg_n[cc_o * NBUF + vic] <= r_logn[cc_o];
                            tg_s[cc_o * NBUF + vic] <= 2'd1;
                        end
                    end
                    if (cc_o == NO - 1) begin
                        cc_run <= 0;
                        cc_done <= 1;
                    end else
                        cc_o <= cc_o + 1'b1;
                    cc_st <= 2'd0;
                end
                endcase
            end
            // ---- request issuer
            if (rq_fire) begin
`ifdef SYNTH_DEBUG
                $display("req o %0d addr %0d left %0d line %0d", rq_o, rq_a, ld_left[rq_o], ld_line[rq_o]);
`endif
                tq[tq_wp] <= {ld_left[rq_o] == 0, rq_o, ld_buf[rq_o], ld_line[rq_o]};
                tq_wp <= tq_wp + 1'b1;
                if (ld_left[rq_o] == 0)
                    ld_act[rq_o] <= 1'b0;
                ld_left[rq_o] <= ld_left[rq_o] - 1'b1;
                ld_line[rq_o] <= ld_line[rq_o] + 1'b1;
                ld_addr[rq_o] <= ld_addr[rq_o] + 25'd64;
                rq_v <= 1'b0;
            end else if (!rq_v && tq_n < 6'd30) begin
                nxv = 0;
                nx = 0;
                for (i = NO - 1; i >= 0; i = i - 1)
                    if (ld_act[(rq_o + 1 + i) % NO]) begin
                        nxv = 1;
                        nx = (rq_o + 1 + i) % NO;
                    end
                if (nxv) begin
                    rq_o <= nx;
                    rq_a <= ld_addr[nx];
                    rq_v <= 1'b1;
                end
            end
            // ---- fills
            if (fill_valid) begin
                cw_a <= {tq_o, tqh[FLOG - 7:0], fw};
                cw_d <= fill_data;
                cw_we[tq_b] <= 1'b1;
                fw <= fw + 1'b1;
                if (fw == 6'd63) begin
`ifdef SYNTH_DEBUG
                    $display("fill done tag %b o %0d b %0d line %0d last %0d n %0d", tqh, tq_o, tq_b, tqh[4:0], tqh[TW-1], tq_n);
`endif
                    tq_rp <= tq_rp + 1'b1;
                    if (tqh[TW - 1])
                        tg_s[tq_o * NBUF + tq_b] <= 2'd2;
                end
            end
            tq_n <= tq_n + (rq_fire ? 6'd1 : 6'd0) - ((fill_valid && fw == 6'd63) ? 6'd1 : 6'd0);
        end
    end

    // ------------------------------------------------------------------ scheduler + warp stage
    reg           sc_s;               // sub-sample
    reg  [1:0]    sc_v;               // voice
    reg           sc_second;          // the voice's second oscillator
    reg  [UB-1:0] sc_u;               // unison voice
    reg  [OB-1:0] sc_o;               // the oscillator: {sc_v, 0} + (sc_second ^ B first), registered
                                      // where sc_v / sc_second change (S_VOICE, S_JOB, S_HAND)
    reg  [31:0]   ph    [0:NO*UMAX-1]; // unison phases
    reg  [31:0]   subph [0:NV-1];
    reg  signed [17:0] modv [0:NO-1];  // each oscillator's latest output (the modulator)
    reg  signed [17:0] v_sub, v_noise; // this voice, this sub-sample
    reg  signed [23:0] sn_f, sn_d;     // sub + noise into the voice sums
    reg  [31:0]   nrng;
    wire signed [39:0] sub_x = v_sub * r_subg[sc_v];      // explicit widths: no truncated products
    wire signed [39:0] noi_x = v_noise * r_noiseg[sc_v];
    wire signed [39:0] win_x = wa_g * $signed({1'b0, fn_q0[17:1]});
    reg  [NO-1:0] odone;               // oscillator finished this sub-sample

    // warp ALU: one 33 x 18 multiplier (2 cycles), the function ROM
    reg  [31:0]   wa_p, wa_p0;         // phase being warped, the unwarped phase
    reg  signed [17:0] wa_g;           // warp gain Q1.16
    reg  signed [32:0] wa_inc;
    reg           wa_slot;
    reg  [4:0]    wa_k;                // micro-step
    reg           wa_oe;               // odd/even second read
    reg  signed [17:0] wa_oee, wa_oen;
    reg  [4:0]    wa_mode;
    reg  [31:0]   wa_par;
    reg  [17:0]   wa_parb, wa_parc;
    reg  signed [17:0] wa_m;
    reg  signed [32:0] wm_a;           // multiplier
    reg  signed [17:0] wm_b;
    reg  signed [50:0] wm_p, wm_q;
    always @(posedge clk) begin
        wm_p <= wm_a * wm_b;
        wm_q <= wm_p;
    end
    reg  [31:0]   nrm;                 // bend / mirror: the half's own phase
    reg           hlf, lo;

    // handoff to the tap stage
    reg           tj_v;
    reg  [OB-1:0] tj_o;
    reg  [FLOG-1:0] tj_idx;            // the phase as a frame index and a 16-bit fraction
    reg  [15:0]   tj_fr;
    reg  signed [17:0] tj_g, tj_ug, tj_oee, tj_oen;
    reg           tj_oe, tj_last;
    wire          tj_take;

    // post stage -> scheduler
    reg           pd_v;                // an oscillator finished
    reg  [OB-1:0] pd_o;
    reg  signed [17:0] pd_x;
    reg  signed [23:0] mixf, mixd;

    // voice output RAM {sub, voice}
    reg           vo_we;
    reg  [2:0]    vo_wa;
    reg  signed [23:0] vo_wf, vo_wd;
    reg  signed [23:0] vof [0:7];
    reg  signed [23:0] vod [0:7];
    always @(posedge clk) begin
        if (vo_we) begin
            vof[vo_wa] <= vo_wf;
            vod[vo_wa] <= vo_wd;
        end
        out_filt   <= vof[out_ra];
        out_direct <= vod[out_ra];
    end

    localparam S_IDLE = 4'd0, S_WAITC = 4'd1, S_VOICE = 4'd2, S_SUB1 = 4'd3, S_SUB2 = 4'd4,
               S_JOB = 4'd5, S_WARP = 4'd6, S_HAND = 4'd7, S_VEND = 4'd8, S_VOUT = 4'd9;
    reg  [3:0]    st;
    reg  [10:0]   cyc;
    wire [4:0]    cur_w = wa_slot ? r_w2[sc_o] : r_w1[sc_o];
    wire          dep = is_xmod(r_w1[sc_o]) || is_xmod(r_w2[sc_o]);
    reg  [31:0]   x1, x2, x3;
    wire [31:0]   x0 = nrng ^ (nrng << 13);
    always @(*) begin
        x1 = x0 ^ (x0 >> 17);
        x2 = x1 ^ (x1 << 5);
        x3 = x2;
    end
    // the slot's last micro-step
    wire [4:0]    wa_last = (cur_w == W_BEND_P || cur_w == W_BEND_M || cur_w == W_BEND_PM) ? 5'd7
                          : (cur_w == W_AM || cur_w == W_RM || cur_w == W_TZFM) ? 5'd6
                          : (cur_w == W_MIRROR || cur_w == W_ASYM_P || cur_w == W_ASYM_M || cur_w == W_ASYM_PM) ? 5'd4
                          : (cur_w == W_SYNC || cur_w == W_SYNCWIN || cur_w == W_PWM || cur_w == W_FM
                             || cur_w == W_FM_SUB || cur_w == W_FM_NOISE) ? 5'd3 : 5'd0;

    always @(posedge clk) begin
        if (rst) begin
            st <= S_IDLE;
            tj_v <= 0;
            vo_we <= 0;
            nrng <= 32'h2545F491;
            odone <= 0;
            cyc <= 0;
            cyc_max <= 0;
            for (i = 0; i < NO; i = i + 1)
                modv[i] <= 0;
            for (i = 0; i < NO * UMAX; i = i + 1)
                ph[i] <= 0;
            for (i = 0; i < NV; i = i + 1)
                subph[i] <= 0;
        end else begin
            vo_we <= 1'b0;
            if (pd_v) begin
                modv[pd_o] <= pd_x;
                odone[pd_o] <= 1'b1;
            end
            if (tj_take)
                tj_v <= 1'b0;
            if (st != S_IDLE)
                cyc <= cyc + 1'b1;
            case (st)
            S_IDLE: if (start) begin
                st <= S_WAITC;
                sc_s <= 0;
                sc_v <= 0;
                cyc <= 0;
            end
            S_WAITC: if (cc_done) st <= S_VOICE;
            S_VOICE: begin   // noise and the sub of voice sc_v
                nrng <= x3;
                v_noise <= {x3[31], x3[31], x3[31:16]};
                wa_p <= subph[sc_v];
                subph[sc_v] <= subph[sc_v] + r_subinc[sc_v];
                odone <= 0;
                sc_second <= 0;
                sc_o <= {sc_v, 1'b0} + r_vflags[sc_v][2];
                sc_u <= 0;
                st <= S_SUB1;
            end
            S_SUB1: begin
                // quarter-wave index: [31] sign, [30] mirror, [29:22] index
                fn_a0 <= 10'd512 + (wa_p[30] ? ~wa_p[29:22] : wa_p[29:22]);
                wa_k <= 0;
                st <= S_SUB2;
            end
            S_SUB2: begin
                wa_k <= wa_k + 1'b1;
                if (wa_k == 5'd1) begin
                    if (r_subtri[sc_v])
                        v_sub <= $signed({2'b00, (wa_p[31] ? ~wa_p[30:16] : wa_p[30:16]), 1'b0}) - 18'sd32768;
                    else
                        v_sub <= wa_p[31] ? -$signed({2'b00, fn_q0[17:2]}) : $signed({2'b00, fn_q0[17:2]});
                end
                if (wa_k == 5'd2) begin
                    sn_f <= (r_vflags[sc_v][0] ? sub_x[39:16] : 24'sd0) + (r_vflags[sc_v][1] ? noi_x[39:16] : 24'sd0);
                    sn_d <= (r_vflags[sc_v][0] ? 24'sd0 : sub_x[39:16]) + (r_vflags[sc_v][1] ? 24'sd0 : noi_x[39:16]);
                    st <= S_JOB;
                end
            end
            S_JOB: begin
                // the job's registers load in every S_JOB cycle (nothing reads them before S_WARP),
                // so only the state depends on the tests below
                wa_p   <= ph[{sc_o, sc_u}];
                wa_p0  <= ph[{sc_o, sc_u}];
                wa_inc <= {1'b0, r_inc[{sc_o, sc_u}]};
                wa_g   <= 18'sd65536;
                wa_oe  <= 1'b0;
                wa_slot <= 1'b0;
                wa_k   <= 0;
                if (!r_on[sc_o]) begin
                    // off: no jobs, a silent modulator
                    modv[sc_o] <= 0;
                    odone[sc_o] <= 1'b1;
                    sc_u <= 0;
                    if (sc_second)
                        st <= S_VEND;
                    else begin
                        sc_second <= 1'b1;
                        sc_o <= {sc_v, 1'b0} + !r_vflags[sc_v][2];
                    end
                end else if (!(sc_second && dep && !odone[sc_o ^ 1'b1]))
                    // (the second oscillator waits for the first one's output when modulated)
                    st <= S_WARP;
            end
            S_WARP: begin
                wa_mode = cur_w;
                wa_par  = wa_slot ? r_wp2[sc_o] : r_wp1[sc_o];
                wa_parb = wa_slot ? r_wb2[sc_o] : r_wb1[sc_o];
                wa_parc = wa_slot ? r_wc2[sc_o] : r_wc1[sc_o];
                wa_m    = (wa_mode == W_FM_SUB) ? v_sub : (wa_mode == W_FM_NOISE) ? v_noise : modv[sc_o ^ 1'b1];
                wa_k <= wa_k + 1'b1;
                case (wa_mode)
                W_SYNC, W_SYNCWIN, W_PWM: begin
                    if (wa_k == 0) begin
                        wm_a <= {1'b0, wa_p};
                        wm_b <= wa_par[17:0];
                        fn_a0 <= 10'd512 + (wa_p[31] ? ~wa_p[30:23] : wa_p[30:23]);   // sin(pi p)
                    end
                    if (wa_k == 3) begin
                        if (wa_mode == W_PWM)
                            wa_p <= (wm_q[50:44] != 0) ? 32'hFFFFFFFF : wm_q[43:12];
                        else
                            wa_p <= wm_q[43:12];   // frac(p k)
                        if (wa_mode == W_SYNCWIN)
                            wa_g <= win_x[33:16];
                    end
                end
                W_ASYM_P, W_ASYM_M, W_ASYM_PM, W_MIRROR: begin
                    // knee: p < d: p 0.5/d; else 0.5 + (p - d) 0.5/(1-d); mirror: on 2p / 2 - 2p
                    if (wa_k == 0)
                        nrm <= (wa_mode == W_MIRROR) ? (wa_p[31] ? (~wa_p << 1) : (wa_p << 1)) : wa_p;
                    if (wa_k == 1) begin
                        lo   <= nrm < wa_par;
                        wm_a <= {1'b0, (nrm < wa_par) ? nrm : nrm - wa_par};
                        wm_b <= (nrm < wa_par) ? wa_parb : wa_parc;
                    end
                    if (wa_k == 4)
                        wa_p <= (lo ? 32'd0 : 32'h80000000) + ((wm_q[50:44] != 0) ? 32'h7FFFFFFF : wm_q[43:12]);
                end
                W_BEND_P, W_BEND_M, W_BEND_PM: begin
                    // per half: u + a u (1 - u)
                    if (wa_k == 0) begin
                        hlf <= wa_p[31];
                        nrm <= wa_p[31] ? (~wa_p << 1) : (wa_p << 1);
                    end
                    if (wa_k == 1) begin
                        wm_a <= {1'b0, nrm};
                        wm_b <= {1'b0, ~nrm[31:15]};   // (1 - u) Q0.17
                    end
                    if (wa_k == 4) begin
                        wm_a <= {1'b0, wm_q[48:17]};   // u (1 - u)
                        wm_b <= wa_par[17:0];          // a s1.16
                    end
                    if (wa_k == 7) begin
                        nrm = nrm + wm_q[47:16];
                        wa_p <= hlf ? ~(nrm >> 1) : (nrm >> 1);
                    end
                end
                W_QUANT:   if (wa_k == 0) wa_p <= wa_p & ~((32'd1 << wa_par[4:0]) - 1'b1);
                W_FLIP:    if (wa_k == 0) wa_g <= (wa_p < wa_par) ? wa_g : -wa_g;
                W_ODDEVEN: if (wa_k == 0) begin wa_oe <= 1'b1; wa_oee <= wa_par[17:0]; wa_oen <= wa_parb; end
                W_FM, W_FM_SUB, W_FM_NOISE: begin
                    if (wa_k == 0) begin wm_a <= {{15{wa_par[17]}}, wa_par[17:0]}; wm_b <= wa_m; end
                    if (wa_k == 3) wa_p <= wa_p + {wm_q[30:0], 1'b0};   // index Q2.16 x m Q2.15 = Q4.31 cycles
                end
                W_AM, W_RM: begin
                    if (wa_k == 0) begin wm_a <= {15'd0, wa_par[17:0]}; wm_b <= wa_m; end
                    if (wa_k == 3) begin
                        // AM: 1 - d/2 + d m / 2; RM: 1 - d + d m   (d Q1.16, m Q2.15)
                        wm_a <= (wa_mode == W_AM) ? 33'sd65536 - $signed({16'd0, wa_par[17:1]}) + (wm_q >>> 16)
                                                  : 33'sd65536 - $signed({15'd0, wa_par[17:0]}) + (wm_q >>> 15);
                        wm_b <= wa_g;
                    end
                    if (wa_k == 6) wa_g <= wm_q[33:16];
                end
                W_TZFM: begin
                    // inc x (1 + depth m), depth Q3.15, m Q2.15
                    if (wa_k == 0) begin wm_a <= {{15{wa_par[17]}}, wa_par[17:0]}; wm_b <= wa_m; end
                    if (wa_k == 3) begin wm_a <= wa_inc; wm_b <= wm_q[32:15]; end
                    if (wa_k == 6) wa_inc <= wa_inc + (wm_q >>> 15);
                end
                default: ;
                endcase
                if (wa_k == wa_last) begin
                    wa_k <= 0;
                    if (wa_slot)
                        st <= S_HAND;
                    else
                        wa_slot <= 1'b1;
                end
            end
            S_HAND: if (!tj_v || tj_take) begin
                ph[{sc_o, sc_u}] <= wa_p0 + wa_inc[31:0];
                tj_v   <= 1'b1;
                tj_o   <= sc_o;
                tj_idx <= wa_p >> (6'd32 - pl_n[sc_o]);
                tj_fr  <= (wa_p << pl_n[sc_o]) >> 16;
                tj_g   <= wa_g;
                tj_ug  <= r_ugain[{sc_o, sc_u}];
                tj_oe  <= wa_oe;
                tj_oee <= wa_oee;
                tj_oen <= wa_oen;
                tj_last <= sc_u == r_uni[sc_o];
                if (sc_u == r_uni[sc_o]) begin
                    sc_u <= 0;
                    if (sc_second)
                        st <= S_VEND;
                    else begin
                        sc_second <= 1'b1;
                        sc_o <= {sc_v, 1'b0} + !r_vflags[sc_v][2];
                        st <= S_JOB;
                    end
                end else begin
                    sc_u <= sc_u + 1'b1;
                    st <= S_JOB;
                end
            end
            S_VEND: if (odone[{sc_v, 1'b0}] && odone[{sc_v, 1'b1}]) st <= S_VOUT;
            S_VOUT: begin   // the voice's sums (the post stage added the oscillators to mixf / mixd)
                vo_we <= 1'b1;
                vo_wa <= {sc_s, sc_v};
                vo_wf <= sat24(mixf + sn_f);
                vo_wd <= sat24(mixd + sn_d);
                if (sc_v == NV - 1) begin
                    sc_v <= 0;
                    if (sc_s) begin
                        st <= S_IDLE;
                        if (cyc > cyc_max)
                            cyc_max <= cyc;
                    end else begin
                        sc_s <= 1'b1;
                        st <= S_VOICE;
                    end
                end else begin
                    sc_v <= sc_v + 1'b1;
                    st <= S_VOICE;
                end
            end
            default: st <= S_IDLE;
            endcase
        end
    end

    // ------------------------------------------------------------------ tap stage
    // 8 taps per pass (2 passes for odd/even), both frames at once; a job's parameters ride in one
    // of two slots (one job in the taps, one in the post stage)
    reg           tp_busy;
    reg  [OB-1:0] tp_o;
    reg  [FLOG-1:0] tp_idx0, tp_half, tp_msk;
    reg  [15:0]   tp_fr;
    reg  [2:0]    tp_j;
    reg           tp_pass, tp_oe;
    reg  [1:0]    tp_js;
    reg  [BB-1:0] tp_ba, tp_bb;
    reg  [3:0]    tp_n;
    // job parameter slots (a job's taps, its sums and its post stage overlap the next jobs)
    reg  [OB-1:0] js_o   [0:3];
    reg  signed [17:0] js_g [0:3], js_ug [0:3], js_oee [0:3], js_oen [0:3];
    reg           js_oe  [0:3], js_last [0:3];
    reg  [15:0]   js_u   [0:3];
    wire [1:0]    js_n = tp_js + 1'b1;
    assign tj_take = tj_v && !tp_busy;
    // the second pass (odd/even) reads half a cycle later: index + N/2, the same fraction
    wire [FLOG-1:0] tp_idx = tp_pass ? tp_idx0 + tp_half : tp_idx0;
    // pipeline: t1 BRAM data, t2 select + coefficient step, t3 coefficient, t4 products, t5 sums
    reg           p1_v, p2_v, p3_v, p4_v, p5_v;
    reg           p1_f, p2_f, p3_f, p4_f, p5_f;   // first tap of a pass
    reg           p1_e, p2_e, p3_e, p4_e, p5_e;   // last tap of a pass
    reg           p1_ps, p2_ps, p3_ps, p4_ps, p5_ps; // pass
    reg  [1:0]    p1_s, p2_s, p3_s, p4_s, p5_s;   // job slot
    reg  [BB-1:0] p1_ba, p1_bb, pb_ba, pb_bb;
    reg           pb_v, pb_f, pb_e, pb_ps, pb_z;
    reg  [1:0]    pb_s;
    reg  [7:0]    pb_pf;
    reg  [7:0]    p1_pf, p2_pf;
    reg           p1_z, p2_z;                     // phase-256 tap 0: coefficient 0
    reg  signed [17:0] p2_c0, p2_c1, p3_c0;
    reg  signed [26:0] p3_d;
    reg  signed [15:0] p2_sa, p2_sb, p3_sa, p3_sb, p4_sa, p4_sb;
    reg  signed [17:0] p4_c;
    reg  signed [33:0] p5_ma, p5_mb;
    reg  signed [37:0] acc_a, acc_b;
    reg           ac_v, ac_ps;                    // the pass's sums are complete
    reg  [1:0]    ac_s;
    always @(posedge clk) begin
        if (rst) begin
            tp_busy <= 0;
            tp_js <= 0;
            p1_v <= 0; pb_v <= 0; p2_v <= 0; p3_v <= 0; p4_v <= 0; p5_v <= 0;
            ac_v <= 0;
        end else begin
            if (tj_take) begin
                tp_busy <= 1'b1;
                tp_o  <= tj_o;
                tp_idx0 <= tj_idx - 3'd3;
                tp_fr <= tj_fr;
                tp_half <= 1 << (pl_n[tj_o] - 1);
                tp_msk <= (1 << pl_n[tj_o]) - 1;
                tp_oe <= tj_oe;
                tp_j  <= 0;
                tp_pass <= 0;
                tp_ba <= pl_a[tj_o];
                tp_bb <= pl_b[tj_o];
                tp_n  <= pl_n[tj_o];
                tp_js <= js_n;
                js_o[js_n]    <= tj_o;
                js_g[js_n]    <= pl_ok[tj_o] ? tj_g : 18'sd0;   // nothing loaded yet: silent
                js_ug[js_n]   <= tj_ug;
                js_oe[js_n]   <= tj_oe;
                js_oee[js_n]  <= tj_oee;
                js_oen[js_n]  <= tj_oen;
                js_last[js_n] <= tj_last;
                js_u[js_n]    <= pl_u[tj_o];
            end
            // t0: addresses of tap tp_j
            p1_v <= tp_busy;
            if (tp_busy) begin
                cr_a  <= {tp_o, (tp_idx + tp_j) & tp_msk};
                sc_a0 <= {tp_fr[15:8], tp_j};
                // phase 256 = phase 0 one sample later: its tap j is phase 0's tap j - 1
                sc_a1 <= (tp_fr[15:8] == 8'hFF) ? {8'd0, tp_j - 3'd1} : {tp_fr[15:8] + 8'd1, tp_j};
                p1_z  <= tp_fr[15:8] == 8'hFF && tp_j == 0;
                p1_pf <= tp_fr[7:0];
                p1_f  <= tp_j == 0;
                p1_e  <= tp_j == 7;
                p1_ps <= tp_pass;
                p1_s  <= tp_js;
                p1_ba <= tp_ba;
                p1_bb <= tp_bb;
                if (tp_j == 7) begin
                    if (tp_oe && !tp_pass) begin
                        tp_pass <= 1'b1;
                        tp_j <= 0;
                    end else
                        tp_busy <= 1'b0;
                end else
                    tp_j <= tp_j + 1'b1;
            end
            // t1: the block RAMs register tap j; its info waits one cycle with it
            pb_v <= p1_v; pb_f <= p1_f; pb_e <= p1_e; pb_ps <= p1_ps; pb_s <= p1_s;
            pb_pf <= p1_pf; pb_z <= p1_z; pb_ba <= p1_ba; pb_bb <= p1_bb;
            // t2: select the two frames' samples, the coefficient step
            p2_v <= pb_v; p2_f <= pb_f; p2_e <= pb_e; p2_ps <= pb_ps; p2_s <= pb_s;
            p2_pf <= pb_pf;
            p2_z  <= pb_z;
            p2_sa <= crq[pb_ba];
            p2_sb <= crq[pb_bb];
            p2_c0 <= sc_q0;
            p2_c1 <= sc_q1;
            p3_v <= p2_v; p3_f <= p2_f; p3_e <= p2_e; p3_ps <= p2_ps; p3_s <= p2_s;
            p3_d  <= ((p2_z ? 18'sd0 : p2_c1) - p2_c0) * $signed({1'b0, p2_pf});
            p3_c0 <= p2_c0;
            p3_sa <= p2_sa;
            p3_sb <= p2_sb;
            p4_v <= p3_v; p4_f <= p3_f; p4_e <= p3_e; p4_ps <= p3_ps; p4_s <= p3_s;
            p4_c  <= p3_c0 + (p3_d >>> 8);
            p4_sa <= p3_sa;
            p4_sb <= p3_sb;
            p5_v <= p4_v; p5_f <= p4_f; p5_e <= p4_e; p5_ps <= p4_ps; p5_s <= p4_s;
            p5_ma <= p4_sa * p4_c;
            p5_mb <= p4_sb * p4_c;
            if (p5_v) begin
                acc_a <= (p5_f ? 38'sd0 : acc_a) + p5_ma;
                acc_b <= (p5_f ? 38'sd0 : acc_b) + p5_mb;
            end
            ac_v  <= p5_v && p5_e;
            ac_ps <= p5_ps;
            ac_s  <= p5_s;
        end
    end

    // ------------------------------------------------------------------ post stage (pipelined)
    // q1: x = A + (B - A) u   q2: saturate   q3: odd/even or x g   q4: (odd/even) x g
    // q5: x ug   q6: the oscillator's sum; on its last job -> modulator register, voice sums
    reg           q1_v, q2_v, q3_v, q4_v, q5_v, q6_v;
    reg           q1_ps, q2_ps;
    reg  [1:0]    q1_s, q2_s, q3_s, q4_s, q5_s, q6_s;
    reg  signed [19:0] q1_a;
    reg  signed [35:0] q1_m;
    reg  signed [17:0] q2_x, q2_x0, q4_x, q5_x;
    reg  signed [35:0] q3_m, q5_m, qa_e, qa_g;
    reg           qa_v;
    reg  [1:0]    qa_s;
    reg  signed [17:0] qa_x0;
    reg  signed [35:0] q4_m;
    reg  signed [23:0] osum [0:NO-1];
    reg  [NO-1:0] seen;
    reg  signed [17:0] q6_out;
    reg  [OB-1:0] q6_o;
    always @(posedge clk) begin
        if (rst) begin
            q1_v <= 0; q2_v <= 0; qa_v <= 0; q3_v <= 0; q4_v <= 0; q5_v <= 0; q6_v <= 0;
            pd_v <= 0;
            seen <= 0;
        end else begin
            q1_v  <= ac_v;
            q1_ps <= ac_ps;
            q1_s  <= ac_s;
            q1_a  <= acc_a >>> 17;
            q1_m  <= ((acc_b - acc_a) >>> 17) * $signed({1'b0, js_u[ac_s]});
            q2_v  <= q1_v;
            q2_ps <= q1_ps;
            q2_s  <= q1_s;
            q2_x  <= sat18(q1_a + (q1_m >>> 16));
            // odd/even: the first pass waits in q2_x0 for the second
            if (q2_v && js_oe[q2_s] && !q2_ps)
                q2_x0 <= q2_x;
            qa_v <= q2_v && !(js_oe[q2_s] && !q2_ps);
            qa_s <= q2_s;
            qa_e <= q2_x * js_oee[q2_s];
            qa_g <= q2_x * js_g[q2_s];
            qa_x0 <= q2_x0;
            q3_v <= qa_v;
            q3_s <= qa_s;
            q3_m <= js_oe[qa_s] ? (qa_x0 + (qa_e >>> 16)) * js_oen[qa_s] : qa_g;
            q4_v <= q3_v;
            q4_s <= q3_s;
            q4_x <= sat18(q3_m >>> 16);
            q4_m <= sat18(q3_m >>> 16) * js_g[q3_s];
            q5_v <= q4_v;
            q5_s <= q4_s;
            q5_x <= js_oe[q4_s] ? sat18(q4_m >>> 16) : q4_x;
            q6_v <= q5_v;
            q6_s <= q5_s;
            q5_m <= (js_oe[q4_s] ? sat18(q4_m >>> 16) : q4_x) * js_ug[q4_s];
            pd_v <= 1'b0;
            if (q6_v) begin
                if (js_last[q6_s]) begin
                    pd_v <= 1'b1;
                    pd_o <= js_o[q6_s];
                    pd_x <= sat18((seen[js_o[q6_s]] ? osum[js_o[q6_s]] : 24'sd0) + (q5_m >>> 16));
                    seen[js_o[q6_s]] <= 1'b0;
                end else begin
                    osum[js_o[q6_s]] <= (seen[js_o[q6_s]] ? osum[js_o[q6_s]] : 24'sd0) + (q5_m >>> 16);
                    seen[js_o[q6_s]] <= 1'b1;
                end
            end
        end
    end
    // the finished oscillator into the voice sums (cleared when a voice starts)
    wire signed [39:0] pd_lx = pd_x * r_level[pd_o];
    always @(posedge clk) begin
        if (st == S_VOICE) begin
            mixf <= 0;
            mixd <= 0;
        end else if (pd_v) begin
            if (r_tof[pd_o]) mixf <= mixf + pd_lx[39:16];
            else             mixd <= mixd + pd_lx[39:16];
        end
    end

    assign busy = (st != S_IDLE) || cc_run;
endmodule
`default_nettype wire
