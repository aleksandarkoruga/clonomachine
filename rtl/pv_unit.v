// SPDX-License-Identifier: MPL-2.0
// Derived from Bungee (https://github.com/bungee-audio-stretch/bungee), Copyright (C) 2020-2026
// Parabola Research Limited, and therefore under the Mozilla Public License 2.0
// (https://mozilla.org/MPL/2.0/); the rest of this repository is MIT (LICENSE).
//
// Phase-vocoder unit: one grain job at a time, shared by all slots (audio domain).
//
// Bit-exact implementation of model/pv_model.cpp (PvCore::run + accumulate) plus the
// restart fade/clear of model/sampler_model.cpp (Engine::mixSlot). Jobs are queued per
// slot by the engine and executed earliest-deadline-first between slots, in order
// within a slot. Each job:
//   SETUP  sequential divider: playhead position, analysis hop, Bungee multipliers
//   FRAME  input lines from SDRAM (4-line buffer), interpolation, Hann window
//   FFT    2048-point complex radix-2 DIF, in place, two parity banks (1 butterfly/cycle)
//   A0     split of the packed real transform (in place, pairs k / M-k)
//   A1     CORDIC vectoring, Mitchell log2, frequency rotation, partial enumeration
//   B1     transient suppression (partial ends)        B2  phase advance per partial
//   C1     final rotation per bin                      C2  CORDIC rotation + merge
//   IFFT   same butterfly, conjugate twiddles, no scaling
//   FADE   (restart grain R0) old content fades out / is cleared
//   OLA    synthesis window, overlap-add into the slot's ring
// A ring entry is read and cleared by the engine when its sample is played.
module pv_unit #(
    parameter NS      = 2,               // slots
    parameter SB      = 1,               // slot index bits
    parameter SIN_HEX = "pv_sin.hex",
    parameter ATAN_HEX = "pv_atan.hex",
    parameter WIN_HEX = "pv_win.hex"
) (
    input  wire              clk,
    input  wire              rst,
    input  wire [31:0]       now,         // engine sample counter (sample being produced)

    // job queue (engine)
    input  wire              job_valid,
    output wire              job_ready,   // slot job_slot can take a job
    input  wire [SB-1:0]     job_slot,
    input  wire              job_clear,   // 1: clear the slot's ring and history (deactivate)
    input  wire [1:0]        job_win,     // PV_WIN_*
    input  wire              job_reset,
    input  wire              job_notr,    // no transient suppression
    input  wire [31:0]       job_c,       // grain centre (output sample)
    input  wire [31:0]       job_dl,      // first sample the grain contributes to
    input  wire [55:0]       job_ph,      // playhead Q32
    input  wire signed [15:0] job_ahead,  // pos = ph + ahead * inc
    input  wire signed [31:0] job_inc,    // playhead increment s8.24
    input  wire [26:0]       job_rate,    // read rate u8.24, (0, 4]
    input  wire [23:0]       job_base,
    input  wire [23:0]       job_frames,

    // ring read (engine): hold em_req until em_ack; em_q is valid with em_ack. Played
    // entries are cleared here (lazily, before the ring is written again).
    input  wire              em_req,
    input  wire [SB-1:0]     em_slot,
    input  wire [11:0]       em_addr,
    output reg               em_ack,
    output wire signed [27:0] em_q,
    output wire [NS-1:0]     clr_busy,    // slot has a ring clear job queued / running

    // SDRAM lines
    output wire              req_valid,
    input  wire              req_ready,
    output wire [24:0]       req_addr,
    input  wire              fill_valid,
    input  wire [15:0]       fill_data,

    output reg  [31:0]       misses,
    output wire              busy
);
    localparam M   = 2048;               // complex FFT length
    localparam DW  = 27;                 // data width
    localparam CI  = 18;                 // CORDIC iterations
    localparam LT  = 128;                // SMP_LATENCY
    localparam FADE = 64;
    localparam CLEAR_END = 2560;         // restart clears [c0+FADE, e+CLEAR_END)

    // ================================================================== helpers
    function [0:0] par11;
        input [10:0] v;
        par11 = ^v;
    endfunction
    function [1:0] bsel;                 // FFT memory bank of element v
        input [10:0] v;
        bsel = {v[10], ^v};
    endfunction
    function [10:0] rev11;
        input [10:0] v;
        integer k;
        begin
            for (k = 0; k < 11; k = k + 1)
                rev11[k] = v[10 - k];
        end
    endfunction
    function signed [DW-1:0] sat27;
        input signed [63:0] v;
        begin
            if (v > 64'sd67108863)       sat27 = 27'sd67108863;
            else if (v < -64'sd67108864) sat27 = -27'sd67108864;
            else                         sat27 = v[DW-1:0];
        end
    endfunction
    function [15:0] mlog2;               // Mitchell log2, Q11
        input [26:0] m;
        integer k;
        reg [4:0] e;
        reg [26:0] sh;
        begin
            e = 0;
            for (k = 0; k < 27; k = k + 1)
                if (m[k]) e = k[4:0];
            sh = (e >= 11) ? (m >> (e - 11)) : (m << (11 - e));
            mlog2 = (m == 0) ? 16'd0 : {e, sh[10:0]};
        end
    endfunction

    // ================================================================== ROMs
    reg [16:0] sin_rom [0:1023];
    reg [23:0] atan_rom [0:CI-1];
    reg [16:0] win_rom [0:2624];
    initial begin
        $readmemh(SIN_HEX, sin_rom);   // entry 1024 (= 65536) is implied
        $readmemh(ATAN_HEX, atan_rom);
        $readmemh(WIN_HEX, win_rom);
    end
    // cs(): cos/sin of 2*pi*a/4096, Q16, 2-cycle latency
    reg  [11:0] cs_a;
    reg  [16:0] cs_sr, cs_crr;
    reg  [1:0]  cs_q1;
    reg         cs_r0;
    reg  signed [17:0] cs_c, cs_s;
    always @(posedge clk) begin
        cs_sr  <= sin_rom[cs_a[9:0]];
        cs_crr <= sin_rom[(11'd1024 - {1'b0, cs_a[9:0]}) & 10'h3FF];
        cs_q1  <= cs_a[11:10];
        cs_r0  <= cs_a[9:0] == 0;
    end
    wire signed [17:0] cs_sv = {1'b0, cs_sr};
    wire signed [17:0] cs_cv = cs_r0 ? 18'sd65536 : {1'b0, cs_crr};
    always @(posedge clk)
        case (cs_q1)
        2'd0: begin cs_s <= cs_sv;  cs_c <= cs_cv;  end
        2'd1: begin cs_s <= cs_cv;  cs_c <= -cs_sv; end
        2'd2: begin cs_s <= -cs_sv; cs_c <= -cs_cv; end
        default: begin cs_s <= -cs_cv; cs_c <= cs_sv; end
        endcase
    reg  [11:0] wr_a;
    reg  [16:0] wr_d;
    always @(posedge clk)
        wr_d <= win_rom[wr_a];

    // ================================================================== memories
    // FFT data: element i -> bank {i[10], ^i}, index i[9:1]; {re, im}. The four operands of
    // butterflies b and b+512 of any radix-2 DIF stage are always in four different banks.
    reg  [53:0] bank0 [0:511];
    reg  [53:0] bank1 [0:511];
    reg  [53:0] bank2 [0:511];
    reg  [53:0] bank3 [0:511];
    reg  [8:0]  bk_ra [0:3];
    reg  [8:0]  bk_wa [0:3];
    reg         bk_we [0:3];
    reg  [53:0] bk_wd [0:3];
    reg  [53:0] bk_q  [0:3];
    always @(posedge clk) begin
        if (bk_we[0]) bank0[bk_wa[0]] <= bk_wd[0];
        bk_q[0] <= bank0[bk_ra[0]];
        if (bk_we[1]) bank1[bk_wa[1]] <= bk_wd[1];
        bk_q[1] <= bank1[bk_ra[1]];
        if (bk_we[2]) bank2[bk_wa[2]] <= bk_wd[2];
        bk_q[2] <= bank2[bk_ra[2]];
        if (bk_we[3]) bank3[bk_wa[3]] <= bk_wd[3];
        bk_q[3] <= bank3[bk_ra[3]];
    end
    // current grain, bins 0..2047 {mag 27, L 16, phase 16, rot 16}; bin 2048 in registers
    reg  [74:0] cur [0:2047];
    reg  [10:0] cu_ra, cu_wa;
    reg         cu_we;
    reg  [74:0] cu_wd, cu_q;
    always @(posedge clk) begin
        if (cu_we) cur[cu_wa] <= cu_wd;
        cu_q <= cur[cu_ra];
    end
    reg  [15:0] lcp [0:2047];            // copy of L (second read port for B1)
    reg  [10:0] lc_ra;
    reg  [15:0] lc_q;
    always @(posedge clk) begin
        if (cu_we) lcp[cu_wa] <= cu_wd[47:32];
        lc_q <= lcp[lc_ra];
    end
    // partials
    reg  [11:0] pk_mem [0:1023];
    reg  [11:0] en_mem [0:1023];
    reg  [15:0] dl_mem [0:1023];
    reg  [9:0]  pk_ra, pk_wa, en_ra, en_wa, dm_ra, dm_wa;
    reg         pk_we, en_we, dm_we;
    reg  [11:0] pk_wd, en_wd, pk_q, en_q;
    reg  [15:0] dm_wd, dm_q;
    always @(posedge clk) begin
        if (pk_we) pk_mem[pk_wa] <= pk_wd;
        pk_q <= pk_mem[pk_ra];
        if (en_we) en_mem[en_wa] <= en_wd;
        en_q <= en_mem[en_ra];
        if (dm_we) dl_mem[dm_wa] <= dm_wd;
        dm_q <= dl_mem[dm_ra];
    end
    // per-slot history {pL 16, pPhase 16, pRot 16}, bins 0..2047; bin 2048 in registers
    reg  [47:0] hist [0:NS*2048-1];
    reg  [SB+10:0] hi_ra, hi_wa;
    reg         hi_we;
    reg  [47:0] hi_wd, hi_q;
    always @(posedge clk) begin
        if (hi_we) hist[hi_wa] <= hi_wd;
        hi_q <= hist[hi_ra];
    end
    // overlap-add rings (simple dual port). The read port serves the engine whenever this
    // unit does not read (rg_rv low): every pass reads at most every other cycle.
    reg  signed [27:0] ring [0:NS*4096-1];
    reg  [SB+11:0] rg_a, rg_wa;
    reg         rg_rv, rg_we;
    reg  signed [27:0] rg_wd, rg_q;
    wire [SB+11:0] rg_ra = rg_rv ? rg_a : {em_slot, em_addr};
    always @(posedge clk) begin
        if (rg_we) ring[rg_wa] <= rg_wd;
        rg_q <= ring[rg_ra];
    end
    assign em_q = rg_q;
    always @(posedge clk)
        em_ack <= !rst && em_req && !rg_rv && !em_ack;
    // input line buffer (4 lines), two copies for two read ports
    reg  [15:0] lb0 [0:255];
    reg  [15:0] lb1 [0:255];
    reg  [7:0]  lb_wa, lb0_ra, lb1_ra;
    reg         lb_we;
    reg  [15:0] lb_wd, lb0_q, lb1_q;
    always @(posedge clk) begin
        if (lb_we) begin
            lb0[lb_wa] <= lb_wd;
            lb1[lb_wa] <= lb_wd;
        end
        lb0_q <= lb0[lb0_ra];
        lb1_q <= lb1[lb1_ra];
    end

    // ================================================================== job queues
    localparam JW = 1 + 2 + 1 + 1 + 32 + 32 + 56 + 16 + 32 + 27 + 24 + 24;
    localparam QD = 8;
    reg  [JW-1:0] jq [0:NS*QD-1];
    reg  [2:0]  jq_wp [0:NS-1];
    reg  [2:0]  jq_rp [0:NS-1];
    reg  [3:0]  jq_n  [0:NS-1];
    assign job_ready = jq_n[job_slot] < QD;
    wire [JW-1:0] job_word = {job_clear, job_win, job_reset, job_notr, job_c, job_dl, job_ph,
                              job_ahead, job_inc, job_rate, job_base, job_frames};

    // earliest-deadline head (registered: a pick is at most one cycle old, and only P_IDLE
    // pops, so a registered pick is always still valid)
    reg  [SB-1:0] pick, pick_r;
    reg         pick_ok, pick_ok_r;
    always @(posedge clk) begin
        pick_r    <= pick;
        pick_ok_r <= pick_ok && !rst;
    end
    integer qi;
    always @* begin
        pick = 0;
        pick_ok = 0;
        for (qi = 0; qi < NS; qi = qi + 1)
            if (jq_n[qi] != 0) begin
                if (!pick_ok || $signed(jq[qi*QD + jq_rp[qi]][JW-38 -: 32] - jq[pick*QD + jq_rp[pick]][JW-38 -: 32]) < 0) begin
                    pick = qi[SB-1:0];
                    pick_ok = 1;
                end
            end
    end

    // ================================================================== job registers
    reg  [SB-1:0] js;
    reg         jclear, jreset, jnotr;
    reg  [1:0]  jwin;
    reg  [31:0] jc, jdl;
    reg  [55:0] jph;
    reg  signed [15:0] jahead;
    reg  signed [31:0] jinc;
    reg  [26:0] jrate;
    reg  [23:0] jbase, jframes;
    wire [55:0] F = {jframes, 32'd0};
    // Dedicated, always-active registered multipliers (yosys cannot share them, and no
    // block-RAM output feeds a multiplier directly).
    reg  signed [47:0] ahinc;
    reg  signed [27:0] fd_a;
    reg  [16:0] fd_b;
    reg  signed [45:0] fd_p;
    always @(posedge clk)
        fd_p <= fd_a * $signed({1'b0, fd_b});
    always @(posedge clk)
        ahinc <= jahead * jinc;            // valid two cycles after a job is taken (P_CATCH, P_TAKE)

    // lazy ring clearing
    reg  [31:0] clp   [0:NS-1];         // entries for samples < clp are cleared
    reg  [SB-1:0] clr_rr;
    reg  [2:0]  clr_n [0:NS-1];         // clear jobs queued or running
    genvar gs;
    generate for (gs = 0; gs < NS; gs = gs + 1) begin : clrb
        assign clr_busy[gs] = clr_n[gs] != 0;
    end endgenerate

    // per-slot history registers
    reg         h_valid [0:NS-1];
    reg  [55:0] h_pos   [0:NS-1];
    reg  signed [17:0] h_err [0:NS-1];
    reg         h_rev   [0:NS-1];
    reg  [15:0] h_nL    [0:NS-1];
    reg  [15:0] h_nph   [0:NS-1];
    reg  [15:0] h_nrot  [0:NS-1];

    // setup results
    reg         cont, rev, prev_rev;
    reg  [55:0] pos;
    reg  signed [41:0] hopi;             // hopIdeal Q16
    reg  signed [17:0] err;
    reg  signed [27:0] hop;
    reg  signed [16:0] fmul;             // frequency rotation multiplier
    reg  signed [25:0] tmul;             // Bungee Stretch::Time multiplier
    reg  [55:0] p0;

    // ================================================================== divider (unsigned 64/64)
    reg  [63:0] dv_n, dv_d, dv_q, dv_r;
    reg  [6:0]  dv_cnt;
    reg         dv_busy;
    reg         dv_go;
    always @(posedge clk) begin
        if (rst) begin
            dv_busy <= 0;
            dv_cnt <= 0;
        end else if (dv_go) begin
            dv_busy <= 1;
            dv_cnt <= 7'd64;
            dv_q <= dv_n;
            dv_r <= 0;
        end else if (dv_busy) begin
            if ({dv_r[62:0], dv_q[63]} >= dv_d) begin
                dv_r <= {dv_r[62:0], dv_q[63]} - dv_d;
                dv_q <= {dv_q[62:0], 1'b1};
            end else begin
                dv_r <= {dv_r[62:0], dv_q[63]};
                dv_q <= {dv_q[62:0], 1'b0};
            end
            dv_cnt <= dv_cnt - 1'b1;
            if (dv_cnt == 1)
                dv_busy <= 0;
        end
    end

    // ================================================================== shared complex multiplier
    // products of (dr, di) with (c, s): 2-cycle latency
    reg  signed [27:0] mu_dr, mu_di;
    reg  signed [17:0] mu_c, mu_s;
    reg  signed [45:0] mu_rc, mu_rs, mu_ic, mu_is;
    reg  signed [45:0] mu_rc1, mu_rs1, mu_ic1, mu_is1;
    reg  signed [27:0] mu2_dr, mu2_di;
    reg  signed [17:0] mu2_c, mu2_s;
    reg  signed [45:0] mu2_rc, mu2_rs, mu2_ic, mu2_is;
    reg  signed [45:0] mu2_rc1, mu2_rs1, mu2_ic1, mu2_is1;
    always @(posedge clk) begin
        mu2_rc1 <= mu2_dr * mu2_c;
        mu2_rs1 <= mu2_dr * mu2_s;
        mu2_ic1 <= mu2_di * mu2_c;
        mu2_is1 <= mu2_di * mu2_s;
        mu2_rc <= mu2_rc1;
        mu2_rs <= mu2_rs1;
        mu2_ic <= mu2_ic1;
        mu2_is <= mu2_is1;
    end
    always @(posedge clk) begin
        mu_rc1 <= mu_dr * mu_c;
        mu_rs1 <= mu_dr * mu_s;
        mu_ic1 <= mu_di * mu_c;
        mu_is1 <= mu_di * mu_s;
        mu_rc <= mu_rc1;
        mu_rs <= mu_rs1;
        mu_ic <= mu_ic1;
        mu_is <= mu_is1;
    end

    // ================================================================== CORDIC (pipelined)
    // stage input: mode (1 rotation), x, y (31 bit), z (25 bit), tag
    localparam CTW = 12;
    reg         co_in_v, co_in_rot;
    reg  signed [26:0] co_in_x, co_in_y;
    reg  signed [15:0] co_in_ph;
    reg  [CTW-1:0] co_in_tag;
    reg         cv   [0:CI];
    reg         crot [0:CI];
    reg  signed [30:0] cx [0:CI];
    reg  signed [30:0] cy [0:CI];
    reg  signed [24:0] cz [0:CI];
    reg  [CTW-1:0] ctag [0:CI];
    // pre-rotation
    always @(posedge clk) begin
        cv[0]   <= co_in_v;
        crot[0] <= co_in_rot;
        ctag[0] <= co_in_tag;
        if (!co_in_rot) begin
            if (co_in_x < 0) begin
                cx[0] <= -($signed({{4{co_in_x[26]}}, co_in_x}) <<< 2);
                cy[0] <= -($signed({{4{co_in_y[26]}}, co_in_y}) <<< 2);
                cz[0] <= 25'sd8388608;
            end else begin
                cx[0] <= $signed({{4{co_in_x[26]}}, co_in_x}) <<< 2;
                cy[0] <= $signed({{4{co_in_y[26]}}, co_in_y}) <<< 2;
                cz[0] <= 25'sd0;
            end
        end else begin
            if ($signed({co_in_ph, 8'd0}) > 24'sd4194304 || $signed({co_in_ph, 8'd0}) < -24'sd4194304) begin
                cx[0] <= -($signed({{4{co_in_x[26]}}, co_in_x}) <<< 2);
                cz[0] <= $signed({co_in_ph[15], co_in_ph, 8'd0}) + (co_in_ph[15] ? 25'sd8388608 : -25'sd8388608);
            end else begin
                cx[0] <= $signed({{4{co_in_x[26]}}, co_in_x}) <<< 2;
                cz[0] <= $signed({co_in_ph[15], co_in_ph, 8'd0});
            end
            cy[0] <= 31'sd0;
        end
    end
    genvar gi;
    generate for (gi = 0; gi < CI; gi = gi + 1) begin : cordic
        wire signed [30:0] xs = cx[gi] >>> gi;
        wire signed [30:0] ys = cy[gi] >>> gi;
        wire neg = crot[gi] ? (cz[gi] < 0) : (cy[gi] >= 0);   // sigma = -1
        wire signed [24:0] at = $signed({1'b0, atan_rom[gi]});
        always @(posedge clk) begin
            cv[gi+1]   <= cv[gi];
            crot[gi+1] <= crot[gi];
            ctag[gi+1] <= ctag[gi];
            if (neg) begin
                cx[gi+1] <= cx[gi] + ys;
                cy[gi+1] <= cy[gi] - xs;
                cz[gi+1] <= cz[gi] + at;
            end else begin
                cx[gi+1] <= cx[gi] - ys;
                cy[gi+1] <= cy[gi] + xs;
                cz[gi+1] <= cz[gi] - at;
            end
        end
    end endgenerate
    wire                co_v   = cv[CI];
    wire [CTW-1:0]      co_tag = ctag[CI];
    wire signed [30:0]  co_xr  = (cx[CI] + 31'sd2) >>> 2;
    wire signed [30:0]  co_yr  = (cy[CI] + 31'sd2) >>> 2;
    wire signed [24:0]  co_zr  = (cz[CI] + 25'sd128) >>> 8;
    wire [26:0]         co_mag = co_xr[26:0];
    wire [15:0]         co_ph  = co_zr[15:0];
    wire signed [26:0]  co_xs  = co_xr[26:0];   // part-selects are unsigned: keep signed copies
    wire signed [26:0]  co_ys  = co_yr[26:0];

    // ================================================================== main FSM
    localparam
        P_IDLE = 5'd0,  P_TAKE = 5'd1,  P_S0 = 5'd2,   P_S1 = 5'd3,   P_S2 = 5'd4,
        P_S3   = 5'd5,  P_S4   = 5'd6,  P_S5 = 5'd7,   P_S6 = 5'd8,   P_FRAME = 5'd9,
        P_FFT  = 5'd10, P_A0   = 5'd11, P_A1 = 5'd12,  P_B1 = 5'd13,  P_B2 = 5'd14,
        P_C1   = 5'd15, P_C2   = 5'd16, P_IFFT = 5'd17, P_FADE = 5'd18, P_CLR = 5'd19,
        P_OLA  = 5'd20, P_CLEAR = 5'd21, P_DONE = 5'd22, P_CATCH = 5'd23;
    reg  [4:0]  ph;
    reg  [3:0]  sub;
    assign busy = ph != P_IDLE;

    // ---------------------------------------------------------------- frame generators
    reg  [55:0] fP, cP;                  // fetch / consume position
    reg  [12:0] ft, ct;                  // samples walked (0..4096)
    reg  [18:0] fLast, cLast;            // last line (absolute line index)
    reg         fLastV, cLastV;
    reg  [15:0] fSeq, cSeq;              // next sequence number to assign
    reg         fHalf;                   // fetch: 0 handle line of w0, 1 line of w1
    reg  [15:0] fillSeq;
    reg  [5:0]  fillW;
    reg  [15:0] cOld;                    // consumer's oldest needed sequence number
    wire [55:0] step_q = {21'd0, jrate, 8'd0};
    // Both walkers step the same positions: S0 registers the word addresses, S1 assigns
    // line sequence numbers (line buffer slot = seq & 3). The fetch walker issues a line
    // request for every new line (registered request, at most 4 lines ahead of the
    // consumer); the consume walker hands samples to the read stage once filled.
    wire [56:0] fP_n = {1'b0, fP} + step_q;
    wire [56:0] cP_n = {1'b0, cP} + step_q;
    // fetch walker
    reg         fs0_v;
    reg  [18:0] fs0_l0, fs0_l1;
    reg         rq_v;
    reg  [18:0] rq_l;
    wire [18:0] f_line  = fHalf ? fs0_l1 : fs0_l0;
    wire        f_new   = !fLastV || f_line != fLast;
    wire        f_allow = (fSeq - cOld) < 16'd4;
    wire        f_free  = !rq_v || req_ready;
    wire        f_step  = fs0_v && (!f_new || (f_free && f_allow));   // half of the entry done
    wire        f_done  = f_step && fHalf;
    wire        f_load  = ft < 13'd4096 && (!fs0_v || f_done);
    assign req_valid = rq_v;
    assign req_addr  = {rq_l, 6'd0};
    // consume walker
    reg         cs0_v, ce_v;
    reg  [24:0] cs0_w0, cs0_w1;
    reg  [15:0] cs0_fr, ce_fr;
    reg  [11:0] cs0_j, ce_j;
    reg  [5:0]  ce_w0, ce_w1;
    reg  [15:0] ce_s0, ce_s1;
    reg  [12:0] cdone;
    wire        c_take = ce_v && (fillSeq - ce_s1) != 0 && $signed(fillSeq - ce_s1) > 0;
    wire        c_s1ld = cs0_v && (!ce_v || c_take);
    wire        c_s0ld = ct < 13'd4096 && (!cs0_v || c_s1ld);
    wire [18:0] c_l0 = cs0_w0[24:6];
    wire [18:0] c_l1 = cs0_w1[24:6];
    wire        c_n0 = !cLastV || c_l0 != cLast;
    wire        c_n1 = c_l1 != c_l0;
    wire [15:0] c_s0 = c_n0 ? cSeq : cSeq - 1'b1;
    wire [15:0] c_s1 = c_n1 ? c_s0 + 1'b1 : c_s0;
    // consume pipeline
    reg         fr_v1, fr_v2, fr_v3, fr_v4, fr_v5, fr_v6;
    reg  [15:0] fr_fr1, fr_fr2, fr_frq;
    reg  [11:0] fr_j1, fr_j2, fr_j3, fr_j4, fr_j5, fr_j6;
    reg  [15:0] fr_s0, fr_s1, fr_s0b;
    reg  [16:0] fr_w, fr_wb;
    reg  signed [16:0] fr_s;
    reg  signed [26:0] fr_even;
    reg  signed [33:0] fr_ip, fr_xw;
    wire signed [16:0] fr_d   = $signed({fr_s1[15], fr_s1} - {fr_s0[15], fr_s0});
    always @(posedge clk) begin
        fr_ip <= fr_d * $signed({1'b0, fr_frq});           // v3
        fr_xw <= fr_s * $signed({1'b0, fr_wb});            // v5
    end
    wire signed [33:0] fr_xwr = (fr_xw + 34'sd64) >>> 7;

    // ---------------------------------------------------------------- FFT control
    reg  [3:0]  st;                      // stage
    reg  [10:0] bf;                      // butterfly index
    reg         bf_run;
    reg         inv;
    reg  [7:0]  fv;                      // valid pipeline
    reg  [10:0] f_i [0:7];
    reg  [10:0] f_j [0:7];
    reg  signed [27:0] f_sr, f_si, f_dr, f_di;
    reg  signed [27:0] f_sr2, f_si2;
    reg  signed [27:0] f_sr3, f_si3;
    reg  signed [DW-1:0] f_ar, f_ai, f_br, f_bi;
    reg  signed [DW-1:0] g_ar, g_ai, g_br, g_bi;
    reg  signed [27:0] g_sr, g_si, g_sr2, g_si2, g_sr3, g_si3;
    reg  [10:0] f_ib [0:7];
    reg  [10:0] f_jb [0:7];
    reg  [7:0]  f_st0;

    // ---------------------------------------------------------------- A0 / A1 / B / C state
    reg  [11:0] k, kq;
    reg  [4:0]  pc;
    reg  signed [DW-1:0] a_ar, a_ai, a_br, a_bi;
    reg  signed [27:0] a_er, a_ei, a_dr, a_di;
    reg  signed [DW-1:0] nyq_r, nyq_i;   // X[2048]
    reg  [26:0] n_mag;                   // bin 2048 of the current grain
    reg  [15:0] n_L, n_ph, n_rotF, n_rot;
    // A1
    reg  [15:0] a1_prevph, a1_rot;
    reg  [15:0] a1_L [0:1];              // L[m], L[m+1] window for the enumeration
    reg         a1_desc;
    reg  [11:0] a1_peak;
    reg  [10:0] a1_cnt;                  // partials
    reg  [15:0] a1_best;
    reg  [10:0] a1_strong;
    reg  [11:0] a1_m;                    // enumeration position
    reg  [10:0] P;                       // partial count
    reg  [10:0] strongest;
    // B1
    reg  [11:0] b_endp;
    reg  [15:0] b_lendp;
    // C1
    reg  [10:0] c_i;
    reg  [11:0] c_end;
    reg  [15:0] c_dl;
    // C2 / OLA scratch
    reg  signed [26:0] x_ar, x_ai;

    integer ii;

    // ---------------------------------------------------------------- the FSM
    // (memory write enables default low every cycle)
    reg  [12:0] t13;
    reg  [12:0] ocnt;
    reg  signed [44:0] o_prod;
    reg  signed [26:0] o_y;
    reg  [4:0]  ov;  // OLA tokens
    reg  [11:0] o_addr [0:4];
    reg  [31:0] o_time [0:4];
    reg  [10:0] o_las [0:4];
    reg  [16:0] o_w;
    reg         o_wv;
    always @(posedge clk)
        o_prod <= o_y * $signed({1'b0, o_w});

    // pass-local scratch for the setup
    reg  signed [57:0] s_sum;
    reg  signed [56:0] s_d;
    reg  signed [37:0] s_d16;
    reg  signed [43:0] s_e;
    reg  signed [43:0] s_er;
    wire signed [33:0] fracneg = -$signed({2'b00, pos[31:0]});
    reg  signed [45:0] erate;
    always @(posedge clk)
        erate <= err * $signed({1'b0, jrate});
    wire signed [41:0] ahopi_s = hopi < 0 ? -hopi : hopi;   // |hopIdeal| (separate: keep it signed)
    wire [41:0] ahopi = ahopi_s;
    wire signed [27:0] ahop_s = hop < 0 ? -hop : hop;
    wire [27:0] ahop = ahop_s;

    // B2 / C2 pipelines
    reg  [7:0]  pv;                      // generic valid pipeline
    reg  [11:0] pp_pk [0:7];
    reg  [10:0] pp_i [0:7];
    reg  [15:0] b2_phc, b2_rfc, b2_pph, b2_prt;
    reg  signed [15:0] b2_off, b2_rotF, b2_prot;
    reg  signed [15:0] b2_off2, b2_rotF2, b2_prot2, b2_off3, b2_rotF3, b2_prot3;
    wire signed [16:0] b2_dsh  = $signed(b2_da) >>> 15;
    reg  signed [42:0] b2_prodr;
    reg  [23:0] b2_pkh;
    always @(posedge clk) begin
        b2_pkh   <= pp_pk[3] * hop[11:0];   // only the low 12 bits matter (a = hop << 20)
        b2_prodr <= b2_dsh * tmul;
    end
    reg  [31:0] b2_da;
    reg  [31:0] b2_s32;
    reg  [15:0] b2_dl;

    // A1 output pipeline
    reg  [2:0]  a1v;
    reg  [26:0] a1s_mag, a1t_mag, a1u_mag;
    reg  [15:0] a1s_ph, a1t_ph, a1u_ph, a1t_L, a1u_L, a1u_d;
    reg  [11:0] a1s_tag, a1t_tag, a1u_tag;
    reg  [15:0] a1_dm;
    reg  signed [32:0] a1_prod;
    always @(posedge clk)
        a1_prod <= $signed(a1_dm) * fmul;   // (d * mul), bits 23:8 = int16(x >> 8)

    // C2

    // pass C1 read pipeline
    reg  [2:0]  c1v;

    // OLA pipeline
    reg  signed [26:0] o_yr, o_yi;
    reg         ol_odd;
    reg         o_mk [0:4];
    reg         o_odd [0:4];
    reg  signed [27:0] o_rq, o_rq2;
    reg         o_wen, o_wen2;
    // pipelined passes
    reg  [11:0] q;
    reg  [6:0]  a0v, a0b;
    reg  [11:0] a0k [0:6];
    reg  signed [47:0] a0_sxr, a0_sxi, a0_syr, a0_syi, a0_snr, a0_sni;
    reg  signed [27:0] a0e_r [0:2];
    reg  signed [27:0] a0e_i [0:2];
    reg  signed [27:0] a0d_r [0:2];
    reg  signed [27:0] a0d_i [0:2];
    reg  [5:0]  b1v;
    reg  [10:0] b1i [0:5];
    reg  [11:0] b1pk [0:5];
    reg  [11:0] b1en [0:5];
    reg  [15:0] b1_Lpk, b1_pLpk, b1_Len;
    reg  [1:0]  c_ld;
    reg  [11:0] c1n [0:2];
    reg  [15:0] c1d [0:2];
    reg  [1:0]  c2v;
    reg         c2b [0:1];
    reg  [11:0] c2k [0:1];
    reg  signed [26:0] c2xa_r, c2xa_i;
    reg  [6:0]  mv;
    reg  signed [27:0] me_r [0:6];
    reg  signed [27:0] me_i [0:6];
    reg  signed [27:0] md_r [0:6];
    reg  signed [27:0] md_i [0:6];
    reg  [11:0] mk [0:6];
    reg  signed [47:0] c2_zr, c2_zi, c2_wr, c2_wi;

    always @(posedge clk) begin
        bk_we[0] <= 0; bk_we[1] <= 0; bk_we[2] <= 0; bk_we[3] <= 0; cu_we <= 0; pk_we <= 0; en_we <= 0; dm_we <= 0;
        hi_we <= 0; rg_we <= 0; rg_rv <= 0; lb_we <= 0; dv_go <= 0; co_in_v <= 0;
        if (rst) begin
            ph <= P_IDLE;
            misses <= 0;
            rq_v <= 0;
            fillSeq <= 0;
            fillW <= 0;
            for (ii = 0; ii < NS; ii = ii + 1) begin
                jq_wp[ii] <= 0;
                jq_rp[ii] <= 0;
                jq_n[ii]  <= 0;
                clp[ii]   <= 0;
                clr_n[ii] <= 0;
                h_valid[ii] <= 0;
                h_pos[ii] <= 0;
                h_err[ii] <= 0;
                h_rev[ii] <= 0;
                h_nL[ii] <= 0;
                h_nph[ii] <= 0;
                h_nrot[ii] <= 0;
            end
        end else begin
            // ---- job push
            if (job_valid && job_ready) begin
                jq[job_slot*QD + jq_wp[job_slot]] <= job_word;
                jq_wp[job_slot] <= jq_wp[job_slot] + 1'b1;
                if (job_clear)
                    clr_n[job_slot] <= clr_n[job_slot] + 1'b1;
            end
            // queue counts (push and pop may coincide)
            for (ii = 0; ii < NS; ii = ii + 1)
                jq_n[ii] <= jq_n[ii] + ((job_valid && job_ready && job_slot == ii) ? 4'd1 : 4'd0)
                                     - ((ph == P_IDLE && pick_ok_r && pick_r == ii) ? 4'd1 : 4'd0);

            // ---- fills (PV lines)
            if (fill_valid) begin
                lb_we <= 1;
                lb_wa <= {fillSeq[1:0], fillW};
                lb_wd <= fill_data;
                fillW <= fillW + 1'b1;
                if (fillW == 6'd63)
                    fillSeq <= fillSeq + 1'b1;
            end

            case (ph)
            // ============================================================ idle / take
            P_IDLE: if (pick_ok_r) begin
                {jclear, jwin, jreset, jnotr, jc, jdl, jph, jahead, jinc, jrate, jbase, jframes}
                    <= jq[pick_r*QD + jq_rp[pick_r]];
                js <= pick_r;
                jq_rp[pick_r] <= jq_rp[pick_r] + 1'b1;
                ph <= P_CATCH;
                t13 <= 0;
            end else begin
                // clear played ring entries in the background, one slot per cycle
                if ($signed(now - clp[clr_rr]) > 0) begin
                    rg_we <= 1;
                    rg_wa <= {clr_rr, clp[clr_rr][11:0]};
                    rg_wd <= 0;
                    clp[clr_rr] <= clp[clr_rr] + 1'b1;
                end
                clr_rr <= (clr_rr == NS - 1) ? {SB{1'b0}} : clr_rr + 1'b1;
            end
            P_CATCH: begin
                // all played entries of this slot are cleared before its ring is written
                if ($signed(now - clp[js]) > 0 && t13 != 13'd4096) begin
                    rg_we <= 1;
                    rg_wa <= {js, clp[js][11:0]};
                    rg_wd <= 0;
                    clp[js] <= clp[js] + 1'b1;
                    t13 <= t13 + 1'b1;
                end else begin
                    if ($signed(now - clp[js]) > 0)
                        clp[js] <= now;      // more than a whole ring behind: all cleared
                    ph <= P_TAKE;
                end
            end
            P_TAKE: begin
                if (jclear) begin
                    ph <= P_CLEAR;
                    t13 <= 0;
                    h_valid[js] <= 0;
                end else begin
                    // pos = (ph + ahead * inc_q32) mod F
                    s_sum <= $signed({2'b00, jph}) + (ahinc <<< 8);
                    ph <= P_S0;
                    sub <= 0;
                end
            end
            // ============================================================ setup
            P_S0: case (sub)
                0: begin
                    dv_n <= s_sum < 0 ? -s_sum : s_sum;
                    dv_d <= {8'd0, F};
                    dv_go <= 1;
                    sub <= 1;
                end
                1: sub <= 2;
                2: if (!dv_busy) begin
                    pos <= (s_sum >= 0) ? dv_r[55:0] : (dv_r == 0 ? 56'd0 : F - dv_r[55:0]);
                    cont <= !jreset && h_valid[js];
                    prev_rev <= h_rev[js];
                    ph <= P_S1;
                    sub <= 0;
                end
            endcase
            P_S1: case (sub)
                0: begin
                    s_d <= $signed({1'b0, pos}) - $signed({1'b0, h_pos[js]});
                    sub <= 1;
                end
                1: begin
                    if (cont) begin
                        if (s_d > $signed({2'b00, F[55:1]}))
                            s_d <= s_d - $signed({1'b0, F});
                        else if (s_d < -$signed({2'b00, F[55:1]}))
                            s_d <= s_d + $signed({1'b0, F});
                    end
                    sub <= 2;
                end
                2: begin
                    if (cont) begin
                        if ((s_d >>> 16) > 57'sd68719476736)       s_d16 <= 38'sd68719476736;
                        else if ((s_d >>> 16) < -57'sd68719476736) s_d16 <= -38'sd68719476736;
                        else                                       s_d16 <= s_d >>> 16;
                    end else
                        s_d16 <= $signed({{5{jinc[31]}}, jinc, 1'b0});   // (inc_q32 * 512) >> 16
                    sub <= 3;
                end
                3: begin
                    dv_n <= {(s_d16 < 0 ? -s_d16 : s_d16), 24'd0};
                    dv_d <= {37'd0, jrate};
                    dv_go <= 1;
                    sub <= 4;
                end
                4: sub <= 5;
                5: if (!dv_busy) begin
                    hopi <= s_d16 < 0 ? -$signed({1'b0, dv_q[40:0]}) : $signed({1'b0, dv_q[40:0]});
                    ph <= P_S2;
                    sub <= 0;
                end
            endcase
            P_S2: case (sub)
                0: begin
                    if (cont) begin
                        s_e <= $signed(h_err[js]) - hopi;
                    end else begin
                        s_e <= hopi + 44'sd32768;
                    end
                    sub <= 1;
                end
                1: begin
                    if (cont) begin
                        hop <= (-s_e + 44'sd32768) >>> 16;
                        s_er <= s_e;
                    end else begin
                        hop <= s_e >>> 16;
                        err <= (pos[31:0] >= 32'h80000000) ? ($signed({2'b00, 32'hFFFFFFFF - pos[31:0] + 1'b1}) >>> 16)
                                                          : (fracneg >>> 16);
                    end
                    sub <= 2;
                end
                2: begin
                    if (cont)
                        err <= s_er + ($signed(hop) <<< 16);
                    rev <= hop < 0;
                    // frequency multiplier: round(2^33 / |hopIdeal|), max 32768
                    dv_n <= 64'h200000000 + {23'd0, ahopi[41:1]};
                    dv_d <= {22'd0, ahopi};
                    dv_go <= hopi != 0;
                    sub <= 3;
                end
                3: sub <= 4;
                4: if (!dv_busy) begin
                    if (hopi == 0 || dv_q > 64'd32768)
                        fmul <= -17'sd32768;
                    else
                        fmul <= -$signed({1'b0, dv_q[15:0]});
                    // time multiplier: (2^25 + |hop|) / (2 hop)
                    dv_n <= 64'h2000000 + {36'd0, ahop};
                    dv_d <= {35'd0, ahop, 1'b0};
                    dv_go <= hop != 0;
                    sub <= 5;
                end
                5: sub <= 6;
                6: if (!dv_busy) begin
                    tmul <= (hop == 0) ? 26'sd0 : (hop < 0 ? -$signed({1'b0, dv_q[24:0]}) : $signed({1'b0, dv_q[24:0]}));
                    // frame start P0 = (pos + (err*rate >> 8) - (2048*rate << 8)) mod F
                    s_sum <= $signed({2'b00, pos}) + (erate >>> 8) - $signed({12'd0, jrate, 19'd0});
                    sub <= 7;
                end
                7: begin
                    dv_n <= s_sum < 0 ? -s_sum : s_sum;
                    dv_d <= {8'd0, F};
                    dv_go <= 1;
                    sub <= 8;
                end
                8: sub <= 9;
                9: if (!dv_busy) begin
                    p0 <= (s_sum >= 0) ? dv_r[55:0] : (dv_r == 0 ? 56'd0 : F - dv_r[55:0]);
                    ph <= P_S3;
                    sub <= 0;
                end
            endcase
            P_S3: begin
                // start the frame
                fP <= p0;
                cP <= p0;
                ft <= 0;
                ct <= 0;
                fLastV <= 0;
                cLastV <= 0;
                fSeq <= fillSeq;
                cSeq <= fillSeq;
                cOld <= fillSeq;
                fHalf <= 0;
                fs0_v <= 0;
                cs0_v <= 0;
                ce_v <= 0;
                cdone <= 0;
                fr_v1 <= 0; fr_v2 <= 0; fr_v3 <= 0; fr_v4 <= 0; fr_v5 <= 0; fr_v6 <= 0;
                ph <= P_FRAME;
            end
            // ============================================================ frame
            P_FRAME: begin
                // ---- fetch walker
                if (rq_v && req_ready)
                    rq_v <= 0;
                if (f_step) begin
                    if (f_new) begin
                        rq_v <= 1;
                        rq_l <= f_line;
                        fLast <= f_line;
                        fLastV <= 1;
                        fSeq <= fSeq + 1'b1;
                    end
                    fHalf <= !fHalf;
                end
                if (f_load) begin
                    fs0_l0 <= ({1'b0, jbase} + 25'd64 + {1'b0, fP[55:32]}) >> 6;
                    fs0_l1 <= ({1'b0, jbase} + 25'd65 + {1'b0, fP[55:32]}) >> 6;
                    fs0_v <= 1;
                    ft <= ft + 1'b1;
                    fP <= (fP_n >= {1'b0, F}) ? fP_n[55:0] - F : fP_n[55:0];
                end else if (f_done)
                    fs0_v <= 0;
                // ---- consume walker: read stage takes a filled sample per cycle
                fr_v1 <= 0;
                if (c_take) begin
                    lb0_ra <= {ce_s0[1:0], ce_w0};
                    lb1_ra <= {ce_s1[1:0], ce_w1};
                    cs_a <= ce_j;
                    fr_v1 <= 1;
                    fr_fr1 <= ce_fr;
                    fr_j1 <= ce_j;
                    cOld <= ce_s0;
                    cdone <= cdone + 1'b1;
                end
                if (c_s1ld) begin
                    ce_v <= 1;
                    ce_w0 <= cs0_w0[5:0];
                    ce_w1 <= cs0_w1[5:0];
                    ce_s0 <= c_s0;
                    ce_s1 <= c_s1;
                    ce_fr <= cs0_fr;
                    ce_j <= cs0_j;
                    cSeq <= c_s1 + 1'b1;
                    cLast <= c_l1;
                    cLastV <= 1;
                end else if (c_take)
                    ce_v <= 0;
                if (c_s0ld) begin
                    cs0_w0 <= {1'b0, jbase} + 25'd64 + {1'b0, cP[55:32]};
                    cs0_w1 <= {1'b0, jbase} + 25'd65 + {1'b0, cP[55:32]};
                    cs0_fr <= cP[31:16];
                    cs0_j <= ct[11:0] + 12'd2048;            // j = t mod 4096, t = ct - 2048
                    cs0_v <= 1;
                    ct <= ct + 1'b1;
                    cP <= (cP_n >= {1'b0, F}) ? cP_n[55:0] - F : cP_n[55:0];
                end else if (c_s1ld)
                    cs0_v <= 0;
                // pipeline: v1 RAM address -> v2 samples registered -> v3 (s1-s0)*fr, window
                // -> v4 interpolated sample -> v5 windowed product -> v6 round, pair, write
                fr_v2 <= fr_v1;
                fr_fr2 <= fr_fr1;
                fr_j2 <= fr_j1;
                fr_v3 <= fr_v2;
                fr_j3 <= fr_j2;
                fr_v4 <= fr_v3;
                fr_j4 <= fr_j3;
                fr_v5 <= fr_v4;
                fr_j5 <= fr_j4;
                fr_v6 <= fr_v5;
                fr_j6 <= fr_j5;
                if (fr_v2) begin
                    fr_s0 <= lb0_q;
                    fr_s1 <= lb1_q;
                    fr_frq <= fr_fr2;
                end
                if (fr_v3) begin
                    fr_s0b <= fr_s0;
                    fr_w <= (18'd65536 + cs_c) >> 1;
                end
                if (fr_v4) begin
                    fr_s <= $signed(fr_s0b) + (fr_ip >>> 16);
                    fr_wb <= fr_w;
                end
                if (fr_v6) begin
                    if (!fr_j6[0])
                        fr_even <= fr_xwr[26:0];
                    else begin
                        bk_we[bsel(fr_j6[11:1])] <= 1;
                        bk_wa[bsel(fr_j6[11:1])] <= fr_j6[10:2];
                        bk_wd[bsel(fr_j6[11:1])] <= {fr_even, fr_xwr[26:0]};
                    end
                end
                if (cdone == 13'd4096 && !fr_v1 && !fr_v2 && !fr_v3 && !fr_v4 && !fr_v5 && !fr_v6) begin
                    ph <= P_FFT;
                    inv <= 0;
                    st <= 0;
                    bf <= 0;
                    bf_run <= 1;
                    fv <= 0;
                end
            end
            // ============================================================ FFT / IFFT
            // two butterflies per cycle: b and b + 512 (elements ia, ja = ia + span,
            // ib = ia + d, jb = ja + d; d = 512 in stage 0, else 1024)
            P_FFT, P_IFFT: begin
                fv <= {fv[6:0], 1'b0};
                if (bf_run) begin : issue
                    reg [10:0] span, kk, ia, ja, ib, jb, dl;
                    span = 11'd1024 >> st;
                    kk = bf & (span - 1'b1);
                    ia = ((bf & ~(span - 1'b1)) << 1) | kk;
                    ja = ia | span;
                    dl = (st == 0) ? 11'd512 : 11'd1024;
                    ib = ia + dl;
                    jb = ja + dl;
                    bk_ra[bsel(ia)] <= ia[9:1];
                    bk_ra[bsel(ja)] <= ja[9:1];
                    bk_ra[bsel(ib)] <= ib[9:1];
                    bk_ra[bsel(jb)] <= jb[9:1];
                    cs_a <= {kk, 1'b0} << st;
                    f_i[0] <= ia;
                    f_j[0] <= ja;
                    f_ib[0] <= ib;
                    f_jb[0] <= jb;
                    fv[0] <= 1;
                    if (bf == 11'd511)
                        bf_run <= 0;
                    bf <= bf + 1'b1;
                end
                for (ii = 1; ii < 8; ii = ii + 1) begin
                    f_i[ii] <= f_i[ii-1];
                    f_j[ii] <= f_j[ii-1];
                    f_ib[ii] <= f_ib[ii-1];
                    f_jb[ii] <= f_jb[ii-1];
                end
                f_st0 <= {f_st0[6:0], st == 0};
                // fv[1]: RAM data
                if (fv[1]) begin
                    f_ar <= bk_q[bsel(f_i[1])][53:27];  f_ai <= bk_q[bsel(f_i[1])][26:0];
                    f_br <= bk_q[bsel(f_j[1])][53:27];  f_bi <= bk_q[bsel(f_j[1])][26:0];
                    g_ar <= bk_q[bsel(f_ib[1])][53:27]; g_ai <= bk_q[bsel(f_ib[1])][26:0];
                    g_br <= bk_q[bsel(f_jb[1])][53:27]; g_bi <= bk_q[bsel(f_jb[1])][26:0];
                end
                // fv[2]: sums, multiplier operands (cs valid now). Second butterfly's twiddle:
                // same in stages >= 1, rotated by -j (index + 1024) in stage 0.
                if (fv[2]) begin
                    f_sr <= f_ar + f_br;
                    f_si <= f_ai + f_bi;
                    mu_dr <= f_ar - f_br;
                    mu_di <= f_ai - f_bi;
                    mu_c <= cs_c;
                    mu_s <= inv ? -cs_s : cs_s;
                    g_sr <= g_ar + g_br;
                    g_si <= g_ai + g_bi;
                    mu2_dr <= g_ar - g_br;
                    mu2_di <= g_ai - g_bi;
                    if (f_st0[1]) begin
                        mu2_c <= -cs_s;
                        mu2_s <= inv ? -cs_c : cs_c;
                    end else begin
                        mu2_c <= cs_c;
                        mu2_s <= inv ? -cs_s : cs_s;
                    end
                end
                f_sr2 <= f_sr; f_si2 <= f_si;
                f_sr3 <= f_sr2; f_si3 <= f_si2;
                g_sr2 <= g_sr; g_si2 <= g_si;
                g_sr3 <= g_sr2; g_si3 <= g_si2;
                // fv[5]: products ready
                if (fv[5]) begin : wb
                    reg signed [47:0] pr, pi, qr, qim;
                    pr = mu_rc + mu_is;
                    pi = mu_ic - mu_rs;
                    qr = mu2_rc + mu2_is;
                    qim = mu2_ic - mu2_rs;
                    bk_we[bsel(f_i[5])] <= 1;  bk_wa[bsel(f_i[5])] <= f_i[5][9:1];
                    bk_we[bsel(f_j[5])] <= 1;  bk_wa[bsel(f_j[5])] <= f_j[5][9:1];
                    bk_we[bsel(f_ib[5])] <= 1; bk_wa[bsel(f_ib[5])] <= f_ib[5][9:1];
                    bk_we[bsel(f_jb[5])] <= 1; bk_wa[bsel(f_jb[5])] <= f_jb[5][9:1];
                    if (!inv) begin
                        bk_wd[bsel(f_i[5])]  <= {sat27((f_sr3 + 28'sd1) >>> 1), sat27((f_si3 + 28'sd1) >>> 1)};
                        bk_wd[bsel(f_j[5])]  <= {sat27((pr + 48'sd65536) >>> 17), sat27((pi + 48'sd65536) >>> 17)};
                        bk_wd[bsel(f_ib[5])] <= {sat27((g_sr3 + 28'sd1) >>> 1), sat27((g_si3 + 28'sd1) >>> 1)};
                        bk_wd[bsel(f_jb[5])] <= {sat27((qr + 48'sd65536) >>> 17), sat27((qim + 48'sd65536) >>> 17)};
                    end else begin
                        bk_wd[bsel(f_i[5])]  <= {sat27(f_sr3), sat27(f_si3)};
                        bk_wd[bsel(f_j[5])]  <= {sat27((pr + 48'sd32768) >>> 16), sat27((pi + 48'sd32768) >>> 16)};
                        bk_wd[bsel(f_ib[5])] <= {sat27(g_sr3), sat27(g_si3)};
                        bk_wd[bsel(f_jb[5])] <= {sat27((qr + 48'sd32768) >>> 16), sat27((qim + 48'sd32768) >>> 16)};
                    end
                end
                // next stage when drained
                if (!bf_run && fv == 0) begin
                    if (st == 4'd10) begin
                        if (ph == P_FFT) begin
                            ph <= P_A0;
                            q <= 0;
                            a0v <= 0;
                            a0b <= 0;
                        end else begin
                            ph <= jwin == 2'd1 ? P_FADE : P_OLA;
                            t13 <= 0;
                            pc <= 0;
                            ol_odd <= 0;
                            ov <= 0;
                            if ($signed(now - jdl) >= 0)
                                misses <= misses + 1'b1;
                        end
                    end else begin
                        st <= st + 1'b1;
                        bf <= 0;
                        bf_run <= 1;
                    end
                end
            end
            // ============================================================ A0: split
            // pairs k / M-k, 2 cycles per pair: q even reads A = Z[k] (rev(k)), q odd
            // reads B = Z[M-k] (rev(M-k)); X[k] and X[M-k] are written back in place.
            P_A0: begin
                a0v <= {a0v[5:0], 1'b0};
                a0b <= {a0b[5:0], 1'b0};
                for (ii = 1; ii < 7; ii = ii + 1)
                    a0k[ii] <= a0k[ii-1];
                if (q < 12'd2050) begin : a0iss
                    reg [10:0] kk, la;
                    kk = q[11:1];
                    la = q[0] ? rev11(11'd0 - kk) : rev11(kk);
                    bk_ra[0] <= la[9:1]; bk_ra[1] <= la[9:1]; bk_ra[2] <= la[9:1]; bk_ra[3] <= la[9:1];
                    if (!q[0])
                        cs_a <= {1'b0, kk};
                    a0v[0] <= 1;
                    a0b[0] <= q[0];
                    a0k[0] <= q[11:1];
                    q <= q + 1'b1;
                end
                if (a0v[1]) begin : a0d
                    reg [10:0] la;
                    reg signed [DW-1:0] zr_, zi_;
                    la = a0b[1] ? rev11(11'd0 - a0k[1][10:0]) : rev11(a0k[1][10:0]);
                    zr_ = bk_q[bsel(la)][53:27];
                    zi_ = bk_q[bsel(la)][26:0];
                    if (!a0b[1]) begin
                        a_ar <= zr_;
                        a_ai <= zi_;
                    end else begin
                        // B = conj(Z[M-k]): E = A + B, D = A - B
                        a0e_r[0] <= a_ar + zr_;
                        a0e_i[0] <= a_ai - zi_;
                        a0d_r[0] <= a_ar - zr_;
                        a0d_i[0] <= a_ai + zi_;
                        mu_dr <= a_ar - zr_;
                        mu_di <= a_ai + zi_;
                        mu_c <= cs_c;
                        mu_s <= cs_s;
                    end
                end
                for (ii = 1; ii < 3; ii = ii + 1) begin
                    a0e_r[ii] <= a0e_r[ii-1];
                    a0e_i[ii] <= a0e_i[ii-1];
                    a0d_r[ii] <= a0d_r[ii-1];
                    a0d_i[ii] <= a0d_i[ii-1];
                end
                if (a0v[4] && a0b[4]) begin
                    // X[k]   = (Er*65536 + c*Di - s*Dr,  Ei*65536 - c*Dr - s*Di) >> 17
                    // X[M-k] = (Er*65536 - c*Di + s*Dr, -Ei*65536 - c*Dr - s*Di) >> 17
                    a0_sxr <= ($signed(a0e_r[2]) <<< 16) + mu_ic - mu_rs;
                    a0_sxi <= ($signed(a0e_i[2]) <<< 16) - mu_rc - mu_is;
                    a0_syr <= ($signed(a0e_r[2]) <<< 16) - mu_ic + mu_rs;
                    a0_syi <= -($signed(a0e_i[2]) <<< 16) - mu_rc - mu_is;
                    // X[M] (k = 0): c = -1, s = 0
                    a0_snr <= ($signed(a0e_r[2]) <<< 16) - ($signed(a0d_i[2]) <<< 16);
                    a0_sni <= ($signed(a0e_i[2]) <<< 16) + ($signed(a0d_r[2]) <<< 16);
                end
                if (a0v[5] && a0b[5]) begin : a0w
                    reg [10:0] la;
                    la = rev11(a0k[5][10:0]);
                    bk_we[bsel(la)] <= 1;
                    bk_wa[bsel(la)] <= la[9:1];
                    bk_wd[bsel(la)] <= {sat27((a0_sxr + 48'sd65536) >>> 17), sat27((a0_sxi + 48'sd65536) >>> 17)};
                    x_ar <= sat27((a0_syr + 48'sd65536) >>> 17);
                    x_ai <= sat27((a0_syi + 48'sd65536) >>> 17);
                    if (a0k[5] == 0) begin
                        nyq_r <= sat27((a0_snr + 48'sd65536) >>> 17);
                        nyq_i <= sat27((a0_sni + 48'sd65536) >>> 17);
                    end
                end
                if (a0v[6] && a0b[6] && a0k[6] != 0 && a0k[6] != 12'd1024) begin : a0w2
                    reg [10:0] lb;
                    lb = rev11(11'd0 - a0k[6][10:0]);
                    bk_we[bsel(lb)] <= 1;
                    bk_wa[bsel(lb)] <= lb[9:1];
                    bk_wd[bsel(lb)] <= {x_ar, x_ai};
                end
                if (q == 12'd2050 && a0v == 0) begin
                    ph <= P_A1;
                    k <= 0;
                    pc <= 0;
                    a1_desc <= 0;
                    a1_cnt <= 0;
                    a1_best <= 0;
                    a1_strong <= 0;
                    a1_m <= 0;
                    pv <= 0;
                end
            end
            // ============================================================ A1
            P_A1: begin
                // issue: read X[k] (Nyquist from register) into the CORDIC
                pv <= {pv[6:0], 1'b0};
                if (k <= 12'd2048 && pc == 0) begin
                    begin : rdX
                        reg [10:0] la;
                        la = rev11(k[10:0]);
                        bk_ra[0] <= la[9:1]; bk_ra[1] <= la[9:1]; bk_ra[2] <= la[9:1]; bk_ra[3] <= la[9:1];
                        pp_pk[0] <= k;
                    end
                    pv[0] <= 1;
                    k <= k + 1'b1;
                end
                for (ii = 1; ii < 8; ii = ii + 1)
                    pp_pk[ii] <= pp_pk[ii-1];
                if (pv[1]) begin
                    co_in_v <= 1;
                    co_in_rot <= 0;
                    co_in_tag <= pp_pk[1];
                    if (pp_pk[1] == 12'd2048) begin
                        co_in_x <= nyq_r; co_in_y <= nyq_i;
                    end else begin
                        co_in_x <= bk_q[bsel(rev11(pp_pk[1][10:0]))][53:27];
                        co_in_y <= bk_q[bsel(rev11(pp_pk[1][10:0]))][26:0];
                    end
                end
                // CORDIC output, in bin order: s1 register, s2 log2 / phase difference,
                // s3 product (a1_prod), s4 frequency rotation, cur write, partial enumeration
                a1v <= {a1v[1:0], co_v};
                if (co_v) begin
                    a1s_mag <= co_mag;
                    a1s_ph <= co_ph;
                    a1s_tag <= co_tag;
                end
                if (a1v[0]) begin
                    a1t_mag <= a1s_mag;
                    a1t_ph <= a1s_ph;
                    a1t_tag <= a1s_tag;
                    a1t_L <= mlog2(a1s_mag);
                    a1_dm <= a1_prevph - a1s_ph;
                    a1_prevph <= a1s_ph;
                end
                if (a1v[1]) begin
                    a1u_mag <= a1t_mag;
                    a1u_ph <= a1t_ph;
                    a1u_tag <= a1t_tag;
                    a1u_L <= a1t_L;
                    a1u_d <= a1_dm;
                end
                if (a1v[2]) begin : a1out
                    reg [15:0] Lv, rf;
                    Lv = a1u_L;
                    rf = (a1u_tag == 0) ? 16'd0 : a1_rot + a1_prod[23:8] + a1u_d;
                    a1_rot <= rf;
                    if (a1u_tag == 12'd2048) begin
                        n_mag <= a1u_mag; n_L <= Lv; n_ph <= a1u_ph; n_rotF <= rf;
                    end else begin
                        cu_we <= 1;
                        cu_wa <= a1u_tag[10:0];
                        cu_wd <= {a1u_mag, Lv, a1u_ph, rf};
                    end
                    // enumeration window: evaluate m = tag - 1 with e(m) = L[tag-1], e(m+1) = L[tag]
                    a1_L[0] <= Lv;
                    if (a1u_tag >= 12'd2)
                        begin : enum1
                            reg lt;
                            lt = a1_L[0] < Lv;            // e(m) < e(m+1), m = tag-1 >= 1
                            if (!a1_desc) begin
                                if (!lt) begin
                                    a1_peak <= a1u_tag - 1'b1;
                                    a1_desc <= 1;
                                    if (a1_cnt == 0 || a1_L[0] > a1_best) begin
                                        a1_best <= a1_L[0];
                                        a1_strong <= a1_cnt;
                                    end
                                end
                            end else if (lt) begin
                                pk_we <= 1; pk_wa <= a1_cnt[9:0]; pk_wd <= a1_peak;
                                en_we <= 1; en_wa <= a1_cnt[9:0]; en_wd <= a1u_tag - 1'b1;
                                a1_cnt <= a1_cnt + 1'b1;
                                a1_desc <= 0;
                            end
                        end
                    if (a1u_tag == 12'd2048)
                        pc <= 1;   // tail: m = 2048 (e(2049) = -1) and m = 2049 (-1 < 0)
                end
                if (pc == 1) begin : enumtail
                    // m = 2048: e(2048) < e(2049) = -1 is false
                    reg [10:0] cnt;
                    reg desc;
                    reg [11:0] peak;
                    cnt = a1_cnt;
                    desc = a1_desc;
                    peak = a1_peak;
                    if (!desc) begin
                        peak = 12'd2048;
                        desc = 1;
                        if (cnt == 0 || a1_L[0] > a1_best) begin
                            a1_best <= a1_L[0];
                            a1_strong <= cnt;
                        end
                    end
                    // m = 2049: -1 < 0 is true -> in DESCEND: end = 2049
                    pk_we <= 1; pk_wa <= cnt[9:0]; pk_wd <= peak;
                    en_we <= 1; en_wa <= cnt[9:0]; en_wd <= 12'd2049;
                    P <= cnt + 1'b1;
                    pc <= 2;
                end
                if (pc == 2) begin
                    strongest <= a1_strong;
                    ph <= P_B1;
                    q <= 0;
                    b1v <= 0;
                end
            end
            // ============================================================ B1: transient suppression
            // one partial per cycle: peak/end -> L[peak], pL[peak], L[end] -> decision
            P_B1: begin
                b1v <= {b1v[4:0], 1'b0};
                for (ii = 1; ii < 6; ii = ii + 1) begin
                    b1i[ii] <= b1i[ii-1];
                    b1pk[ii] <= b1pk[ii-1];
                    b1en[ii] <= b1en[ii-1];
                end
                if (q < {1'b0, P}) begin
                    pk_ra <= q[9:0];
                    en_ra <= q[9:0];
                    b1v[0] <= 1;
                    b1i[0] <= q[10:0];
                    q <= q + 1'b1;
                end
                if (b1v[1]) begin
                    cu_ra <= pk_q[10:0];
                    lc_ra <= en_q[10:0];
                    hi_ra <= {js, pk_q[10:0]};
                    b1pk[2] <= pk_q;
                    b1en[2] <= en_q;
                end
                if (b1v[3]) begin
                    b1_Lpk  <= b1pk[3] == 12'd2048 ? n_L : cu_q[47:32];
                    b1_pLpk <= b1pk[3] == 12'd2048 ? h_nL[js] : hi_q[47:32];
                    b1_Len  <= b1en[3] == 12'd2048 ? n_L : (b1en[3] == 12'd2049 ? 16'd0 : lc_q);
                end
                if (b1v[4]) begin : b1dec
                    reg [10:0] i;
                    reg trans;
                    i = b1i[4];
                    trans = cont && !jnotr && i != 0 && i != P - 1'b1 && i != strongest
                         && $signed({1'b0, b1_Lpk}) > $signed({1'b0, b1_pLpk}) + 18'sd599;
                    if (i == 0) begin
                        b_endp <= b1en[4];
                        b_lendp <= b1_Len;
                    end else if (trans && b_lendp > b1_Len) begin
                        en_we <= 1; en_wa <= i[9:0] - 1'b1; en_wd <= b1en[4];   // end[i-1] = end[i]
                        b_endp <= b1en[4];
                        b_lendp <= b1_Len;
                    end else if (trans) begin
                        en_we <= 1; en_wa <= i[9:0]; en_wd <= b_endp;            // end[i] = end[i-1]
                    end else begin
                        b_endp <= b1en[4];
                        b_lendp <= b1_Len;
                    end
                end
                if (q == {1'b0, P} && b1v == 0) begin
                    ph <= P_B2;
                    k <= 0;
                    pc <= 0;
                    pv <= 0;
                end
            end
            // ============================================================ B2: phase advance per partial
            P_B2: begin
                pv <= {pv[6:0], 1'b0};
                if (k < {1'b0, P} && pc == 0) begin
                    pk_ra <= k[9:0];
                    pp_i[0] <= k[10:0];
                    pv[0] <= 1;
                    k <= k + 1'b1;
                end
                for (ii = 1; ii < 8; ii = ii + 1) begin
                    pp_pk[ii] <= pp_pk[ii-1];
                    pp_i[ii] <= pp_i[ii-1];
                end
                // pv[1]: peak known -> read cur / hist
                if (pv[1]) begin
                    cu_ra <= pk_q[10:0];
                    hi_ra <= {js, pk_q[10:0]};
                    pp_pk[2] <= pk_q;
                end
                // pv[3]: RAM data registered
                if (pv[3]) begin
                    b2_phc <= pp_pk[3] == 12'd2048 ? n_ph   : cu_q[31:16];
                    b2_rfc <= pp_pk[3] == 12'd2048 ? n_rotF : cu_q[15:0];
                    b2_pph <= pp_pk[3] == 12'd2048 ? h_nph[js]  : hi_q[31:16];
                    b2_prt <= pp_pk[3] == 12'd2048 ? h_nrot[js] : hi_q[15:0];
                end
                // pv[4]: phase terms (b2_pkh = peak * hop[11:0] registered meanwhile)
                if (pv[4]) begin
                    b2_off <= (rev ? -b2_phc : b2_phc) - (prev_rev ? -b2_pph : b2_pph);
                    b2_rotF <= b2_rfc;
                    b2_prot <= b2_prt;
                    b2_da <= ({b2_phc, 16'd0} - {b2_pph, 16'd0}) - {b2_pkh[11:0], 20'd0};
                end
                b2_off2 <= b2_off;   b2_rotF2 <= b2_rotF;   b2_prot2 <= b2_prot;
                b2_off3 <= b2_off2;  b2_rotF3 <= b2_rotF2;  b2_prot3 <= b2_prot2;
                // pv[6]: b2_prodr = (da >> 15) * mulT registered in between
                if (pv[6])
                    b2_s32 <= {pp_pk[6][2:0], 29'd0} + b2_prodr[31:0];
                if (pv[7]) begin : b2w
                    reg [15:0] st16;
                    st16 = b2_s32[31:16];
                    dm_we <= 1;
                    dm_wa <= pp_i[7][9:0];
                    dm_wd <= cont ? (b2_prot3 - b2_off3 + st16 - b2_rotF3) : -b2_rotF3;
                end
                if (k >= {1'b0, P} && pv == 0) begin
                    ph <= P_C1;
                    k <= 0;
                    c_i <= 0;
                    en_ra <= 0;
                    dm_ra <= 0;
                    c_ld <= 2'd2;
                    c1v <= 0;
                end
            end
            // ============================================================ C1: rot[n] = rotF[n] + delta
            // one bin per cycle; moving to the next partial costs two cycles
            P_C1: begin
                c1v <= {c1v[1:0], 1'b0};
                c1n[1] <= c1n[0];
                c1d[1] <= c1d[0];
                if (c_ld != 0) begin
                    c_ld <= c_ld - 1'b1;
                    if (c_ld == 2'd1) begin
                        c_end <= en_q;
                        c_dl <= dm_q;
                    end
                end else if (k <= 12'd2047) begin
                    if (c_i < P - 1'b1 && k >= c_end) begin
                        c_i <= c_i + 1'b1;
                        en_ra <= c_i[9:0] + 1'b1;
                        dm_ra <= c_i[9:0] + 1'b1;
                        c_ld <= 2'd2;
                    end else begin
                        cu_ra <= k[10:0];
                        c1v[0] <= 1;
                        c1n[0] <= k;
                        c1d[0] <= c_dl;
                        k <= k + 1'b1;
                    end
                end
                if (c1v[1]) begin : c1w
                    reg [15:0] r;
                    r = cu_q[15:0] + c1d[1];
                    cu_we <= 1;
                    cu_wa <= c1n[1][10:0];
                    cu_wd <= {cu_q[74:16], r};
                    if (c1n[1] == 12'd2047)
                        n_rot <= r;     // bin 2048: rot = rot[2047]
                end
                if (k == 12'd2048 && c_ld == 0 && c1v == 0) begin
                    ph <= P_C2;
                    q <= 0;
                    c2v <= 0;
                    mv <= 0;
                end
            end
            // ============================================================ C2: resynthesis + merge
            // pairs kA = k, kB = M - k (k = 0: kB = 2048), 2 cycles per pair:
            // cur -> CORDIC rotation -> E/D -> twiddle products -> Z'[kA], Z'[kB]
            P_C2: begin
                c2v <= {c2v[0], 1'b0};
                c2b[1] <= c2b[0];
                c2k[1] <= c2k[0];
                if (q < 12'd2050) begin : c2iss
                    reg [11:0] kk;
                    kk = {1'b0, q[11:1]};
                    cu_ra <= q[0] ? ((12'd2048 - kk) & 12'h7FF) : kk;
                    c2v[0] <= 1;
                    c2b[0] <= q[0];
                    c2k[0] <= kk;
                    q <= q + 1'b1;
                end
                if (c2v[1]) begin : c2in
                    reg [26:0] mg;
                    reg [15:0] Lx, phx, rtx;
                    reg nyq;
                    nyq = c2b[1] && c2k[1] == 0;
                    mg  = nyq ? n_mag : cu_q[74:48];
                    Lx  = nyq ? n_L   : cu_q[47:32];
                    phx = nyq ? n_ph  : cu_q[31:16];
                    rtx = nyq ? n_rot : cu_q[15:0];
                    co_in_v <= 1;
                    co_in_rot <= 1;
                    co_in_x <= mg;
                    co_in_y <= 0;
                    co_in_ph <= (rev ? -phx : phx) + rtx;
                    co_in_tag <= {c2b[1], c2k[1][10:0]};
                    if (!c2b[1]) begin
                        hi_we <= 1;
                        hi_wa <= {js, c2k[1][10:0]};
                        hi_wd <= {Lx, phx, rtx};
                    end else if (nyq) begin
                        h_nL[js] <= Lx;
                        h_nph[js] <= phx;
                        h_nrot[js] <= rtx;
                    end else if (c2k[1] != 12'd1024) begin
                        hi_we <= 1;
                        hi_wa <= {js, 11'd0 - c2k[1][10:0]};
                        hi_wd <= {Lx, phx, rtx};
                    end
                end
                // CORDIC out: A (tag[11] = 0) then B
                mv <= {mv[5:0], 1'b0};
                for (ii = 1; ii < 7; ii = ii + 1) begin
                    me_r[ii] <= me_r[ii-1];
                    me_i[ii] <= me_i[ii-1];
                    md_r[ii] <= md_r[ii-1];
                    md_i[ii] <= md_i[ii-1];
                    mk[ii] <= mk[ii-1];
                end
                if (co_v && !co_tag[11]) begin
                    c2xa_r <= co_xr[26:0];
                    c2xa_i <= co_yr[26:0];
                    cs_a <= {1'b0, co_tag[10:0]};
                end
                if (co_v && co_tag[11]) begin
                    // E = XA + conj(XB), D = XA - conj(XB)
                    me_r[0] <= c2xa_r + co_xs;
                    me_i[0] <= c2xa_i - co_ys;
                    md_r[0] <= c2xa_r - co_xs;
                    md_i[0] <= c2xa_i + co_ys;
                    mk[0] <= {1'b0, co_tag[10:0]};
                    mv[0] <= 1;
                end
                if (mv[1]) begin
                    mu_dr <= md_r[1];
                    mu_di <= md_i[1];
                    mu_c <= cs_c;
                    mu_s <= cs_s;
                end
                if (mv[4]) begin
                    // Z'[kA] = (Er*65536 - (Dr*s + Di*c),  Ei*65536 + (Dr*c - Di*s)) >> 18
                    // Z'[kB] = (Er*65536 + (Dr*s + Di*c), -Ei*65536 + (Dr*c - Di*s)) >> 18
                    c2_zr <= ($signed(me_r[4]) <<< 16) - (mu_rs + mu_ic);
                    c2_zi <= ($signed(me_i[4]) <<< 16) + (mu_rc - mu_is);
                    c2_wr <= ($signed(me_r[4]) <<< 16) + (mu_rs + mu_ic);
                    c2_wi <= -($signed(me_i[4]) <<< 16) + (mu_rc - mu_is);
                end
                if (mv[5]) begin : c2w
                    reg [10:0] ka;
                    ka = mk[5][10:0];
                    bk_we[bsel(ka)] <= 1;
                    bk_wa[bsel(ka)] <= ka[9:1];
                    bk_wd[bsel(ka)] <= {sat27((c2_zr + 48'sd131072) >>> 18), sat27((c2_zi + 48'sd131072) >>> 18)};
                    x_ar <= sat27((c2_wr + 48'sd131072) >>> 18);
                    x_ai <= sat27((c2_wi + 48'sd131072) >>> 18);
                end
                if (mv[6] && mk[6] != 0 && mk[6] != 12'd1024) begin : c2w2
                    reg [10:0] kb;
                    kb = 11'd0 - mk[6][10:0];
                    bk_we[bsel(kb)] <= 1;
                    bk_wa[bsel(kb)] <= kb[9:1];
                    bk_wd[bsel(kb)] <= {x_ar, x_ai};
                end
                if (mv[6] && mk[6] == 12'd1024) begin
                    // last pair written: keep the slot history scalars, inverse transform
                    h_valid[js] <= 1;
                    h_pos[js] <= pos;
                    h_err[js] <= err;
                    h_rev[js] <= rev;
                    ph <= P_IFFT;
                    inv <= 1;
                    st <= 0;
                    bf <= 0;
                    bf_run <= 1;
                    fv <= 0;
                end
            end
            // ============================================================ FADE / CLR (restart grain)
            P_FADE: case (pc)
                0: begin
                    rg_a <= {js, jc[11:0] + t13[11:0]};
                    rg_rv <= 1;
                    wr_a <= 12'd2561 + t13[11:0];
                    pc <= 1;
                end
                1: pc <= 2;
                2: begin
                    fd_a <= rg_q;
                    fd_b <= wr_d;
                    pc <= 3;
                end
                3: pc <= 4;            // fd_p = fd_a * fd_b
                4: begin
                    rg_we <= 1;
                    rg_wa <= {js, jc[11:0] + t13[11:0]};
                    rg_wd <= (fd_p + 46'sd32768) >>> 16;
                    if (t13 == FADE - 1) begin
                        ph <= P_CLR;
                        t13 <= FADE;
                    end else
                        t13 <= t13 + 1'b1;
                    pc <= 0;
                end
            endcase
            P_CLR: begin
                // clear [c0+FADE, e+CLEAR_END), c0 = e + LT
                rg_we <= 1;
                rg_wa <= {js, jc[11:0] + t13[11:0]};
                rg_wd <= 0;
                if (t13 == CLEAR_END - LT - 1) begin
                    ph <= P_OLA;
                    t13 <= 0;
                    pc <= 0;
                    ol_odd <= 0;
                    ov <= 0;
                end else
                    t13 <= t13 + 1'b1;
            end
            // ============================================================ OLA
            // sample s = t + 1024, one every 2 cycles: T read ring / window / element,
            // T+2 register them, T+3 product (o_prod), T+4 sum and write
            P_OLA: begin
                ol_odd <= !ol_odd;
                ov <= {ov[3:0], 1'b0};
                for (ii = 1; ii < 5; ii = ii + 1) begin
                    o_addr[ii] <= o_addr[ii-1];
                    o_time[ii] <= o_time[ii-1];
                    o_mk[ii] <= o_mk[ii-1];
                    o_odd[ii] <= o_odd[ii-1];
                    o_las[ii] <= o_las[ii-1];
                end
                if (!ol_odd && t13 < 13'd2048) begin : oiss
                    reg signed [12:0] t, u;
                    reg [12:0] at;
                    reg [11:0] wa, j;
                    reg [10:0] la;
                    t = $signed({1'b0, t13}) - 13'sd1024;
                    at = t < 0 ? -t : t;
                    case (jwin)
                    2'd1: begin u = t;        wa = t < 0 ? 12'd0 : (t < 512 ? 12'd1025 + t : at); end
                    2'd2: begin u = t + 512;  wa = u < 0 ? 12'd0 : (u < 512 ? 12'd1537 + u : at); end
                    2'd3: begin u = t + 1024; wa = u < 512 ? 12'd2049 + u : at; end
                    default: begin u = 0;     wa = at; end
                    endcase
                    wr_a <= wa;
                    rg_a <= {js, jc[11:0] + t[11:0]};
                    rg_rv <= 1;
                    j = t[11:0];
                    la = rev11(j[11:1]);
                    if (!t13[0]) begin
                        bk_ra[0] <= la[9:1]; bk_ra[1] <= la[9:1]; bk_ra[2] <= la[9:1]; bk_ra[3] <= la[9:1];
                    end
                    ov[0] <= 1;
                    o_addr[0] <= jc[11:0] + t[11:0];
                    o_time[0] <= jc + {{19{t[12]}}, t};
                    o_mk[0] <= (jwin == 2'd1 && t < 0) || (jwin == 2'd2 && u < 0);
                    o_odd[0] <= t13[0];
                    o_las[0] <= la;
                    t13 <= t13 + 1'b1;
                end
                if (ov[1]) begin
                    // RAM / ROM outputs of the sample read two cycles ago
                    if (!o_odd[1]) begin
                        o_y <= bk_q[bsel(o_las[1])][53:27];
                        o_yi <= bk_q[bsel(o_las[1])][26:0];
                    end else
                        o_y <= o_yi;
                    o_w <= wr_d;
                    o_rq <= rg_q;
                    o_wen <= !o_mk[1] && wr_d != 0 && $signed(o_time[1] - now) > 0;
                end
                o_rq2 <= o_rq;
                o_wen2 <= o_wen;
                if (ov[3]) begin
                    rg_wa <= {js, o_addr[3]};
                    rg_we <= o_wen2;
                    rg_wd <= o_rq2 + ((o_prod + 45'sd131072) >>> 18);
                end
                if (t13 == 13'd2048 && ov == 0)
                    ph <= P_DONE;
            end
            // ============================================================ clear job
            P_CLEAR: begin
                rg_we <= 1;
                rg_wa <= {js, t13[11:0]};
                rg_wd <= 0;
                t13 <= t13 + 1'b1;
                if (t13 == 13'd4095) begin
                    ph <= P_DONE;
                    clp[js] <= now;
                    clr_n[js] <= clr_n[js] - 1'b1;
                end
            end
            P_DONE: ph <= P_IDLE;
            default: ph <= P_IDLE;
            endcase
        end
    end
endmodule
