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
    input  wire [25:0]       job_rate,    // read rate u8.24
    input  wire [23:0]       job_base,
    input  wire [23:0]       job_frames,

    // ring access (engine): read entry, optionally clear it
    input  wire              em_re,
    input  wire              em_clr,
    input  wire [SB-1:0]     em_slot,
    input  wire [11:0]       em_addr,
    output reg  signed [27:0] em_q,

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
    // FFT data: element i -> bank par11(i), index i[10:1]; {re, im}
    reg  [53:0] bank0 [0:1023];
    reg  [53:0] bank1 [0:1023];
    reg  [9:0]  b0_ra, b1_ra, b0_wa, b1_wa;
    reg         b0_we, b1_we;
    reg  [53:0] b0_wd, b1_wd, b0_q, b1_q;
    always @(posedge clk) begin
        if (b0_we) bank0[b0_wa] <= b0_wd;
        b0_q <= bank0[b0_ra];
        if (b1_we) bank1[b1_wa] <= b1_wd;
        b1_q <= bank1[b1_ra];
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
    // overlap-add rings: port A here, port B engine
    reg  signed [27:0] ring [0:NS*4096-1];
    reg  [SB+11:0] rg_a;
    reg         rg_we;
    reg  signed [27:0] rg_wd, rg_q;
    always @(posedge clk) begin
        if (rg_we) ring[rg_a] <= rg_wd;
        rg_q <= ring[rg_a];
    end
    always @(posedge clk) begin
        if (em_clr) ring[{em_slot, em_addr}] <= 28'sd0;
        if (em_re) em_q <= ring[{em_slot, em_addr}];
    end
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
    localparam JW = 1 + 2 + 1 + 1 + 32 + 32 + 56 + 16 + 32 + 26 + 24 + 24;
    localparam QD = 4;
    reg  [JW-1:0] jq [0:NS*QD-1];
    reg  [1:0]  jq_wp [0:NS-1];
    reg  [1:0]  jq_rp [0:NS-1];
    reg  [2:0]  jq_n  [0:NS-1];
    assign job_ready = jq_n[job_slot] < QD;
    wire [JW-1:0] job_word = {job_clear, job_win, job_reset, job_notr, job_c, job_dl, job_ph,
                              job_ahead, job_inc, job_rate, job_base, job_frames};

    // earliest-deadline head
    reg  [SB-1:0] pick;
    reg         pick_ok;
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
    reg  [25:0] jrate;
    reg  [23:0] jbase, jframes;
    wire [55:0] F = {jframes, 32'd0};
    wire signed [47:0] ahinc = jahead * jinc;

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

    // ================================================================== main FSM
    localparam
        P_IDLE = 5'd0,  P_TAKE = 5'd1,  P_S0 = 5'd2,   P_S1 = 5'd3,   P_S2 = 5'd4,
        P_S3   = 5'd5,  P_S4   = 5'd6,  P_S5 = 5'd7,   P_S6 = 5'd8,   P_FRAME = 5'd9,
        P_FFT  = 5'd10, P_A0   = 5'd11, P_A1 = 5'd12,  P_B1 = 5'd13,  P_B2 = 5'd14,
        P_C1   = 5'd15, P_C2   = 5'd16, P_IFFT = 5'd17, P_FADE = 5'd18, P_CLR = 5'd19,
        P_OLA  = 5'd20, P_CLEAR = 5'd21, P_DONE = 5'd22;
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
    wire [55:0] step_q = {22'd0, jrate, 8'd0};
    wire [24:0] f_w0 = {1'b0, jbase} + 25'd64 + {1'b0, fP[55:32]};
    wire [24:0] f_w1 = f_w0 + 25'd1;
    wire [18:0] f_line = fHalf ? f_w1[24:6] : f_w0[24:6];
    wire        f_new  = !fLastV || f_line != fLast;
    wire        f_allow = (fSeq - cOld) < 16'd4;
    assign req_valid = (ph == P_FRAME) && ft < 13'd4096 && f_new && f_allow;
    assign req_addr  = {f_line, 6'd0};
    wire [24:0] c_w0 = {1'b0, jbase} + 25'd64 + {1'b0, cP[55:32]};
    wire [24:0] c_w1 = c_w0 + 25'd1;
    wire [18:0] c_l0 = c_w0[24:6];
    wire [18:0] c_l1 = c_w1[24:6];
    wire        c_n0 = !cLastV || c_l0 != cLast;
    wire        c_n1 = c_l1 != c_l0;
    wire [15:0] c_s0 = c_n0 ? cSeq : cSeq - 1'b1;
    wire [15:0] c_s1 = c_n1 ? c_s0 + 1'b1 : c_s0;
    wire        c_ok = ct < 13'd4096 && (fillSeq - c_s1) != 0 && $signed(fillSeq - c_s1) > 0;
    // consume pipeline
    reg         fr_v1, fr_v2, fr_v3, fr_v4;
    reg  [15:0] fr_fr1, fr_fr2;
    reg  [11:0] fr_j1, fr_j2, fr_j3, fr_j4;
    reg  signed [16:0] fr_s;
    reg  signed [33:0] fr_xw;
    reg  signed [26:0] fr_even;
    wire signed [16:0] fr_d   = $signed({lb1_q[15], lb1_q} - {lb0_q[15], lb0_q});
    wire signed [33:0] fr_ip  = fr_d * $signed({1'b0, fr_fr2});
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
    reg  [15:0] c2_L [0:1];
    reg  [15:0] c2_ph [0:1];
    reg  [15:0] c2_rot [0:1];

    integer ii;

    // ---------------------------------------------------------------- the FSM
    // (memory write enables default low every cycle)
    reg  [12:0] t13;
    reg  [12:0] ocnt;
    reg  signed [44:0] o_prod;
    reg  signed [27:0] o_y;
    reg  [3:0]  ov;
    reg  [11:0] o_addr [0:3];
    reg  [31:0] o_time [0:3];
    reg  [16:0] o_w;
    reg         o_wv;

    // pass-local scratch for the setup
    reg  signed [57:0] s_sum;
    reg  signed [56:0] s_d;
    reg  signed [37:0] s_d16;
    reg  signed [43:0] s_e;
    reg  signed [43:0] s_er;
    wire signed [33:0] fracneg = -$signed({2'b00, pos[31:0]});
    wire signed [41:0] ahopi_s = hopi < 0 ? -hopi : hopi;   // |hopIdeal| (separate: keep it signed)
    wire [41:0] ahopi = ahopi_s;
    wire signed [27:0] ahop_s = hop < 0 ? -hop : hop;
    wire [27:0] ahop = ahop_s;

    // B2 / C2 pipelines
    reg  [5:0]  pv;                      // generic valid pipeline
    reg  [11:0] pp_pk [0:5];
    reg  [10:0] pp_i [0:5];
    reg  signed [15:0] b2_off, b2_rotF, b2_prot;
    reg  signed [15:0] b2_off2, b2_rotF2, b2_prot2;
    wire signed [16:0] b2_dsh  = $signed(b2_da) >>> 15;
    wire signed [42:0] b2_prod = b2_dsh * tmul;
    reg  [31:0] b2_da;
    reg  [31:0] b2_s32;
    reg  [15:0] b2_dl;

    // A1 pipeline
    reg  [26:0] a1_mag;
    reg  [15:0] a1_ph, a1_Lv;

    // C2
    reg  [11:0] c2_ka;
    reg  signed [26:0] c2_xr [0:1];
    reg  signed [26:0] c2_xi [0:1];
    reg         c2_have;
    reg  [3:0]  c2v;
    reg  [11:0] c2_kk [0:3];
    reg  signed [27:0] c2_er, c2_ei;

    // pass C1 read pipeline
    reg  [2:0]  c1v;
    reg  [11:0] c1_n [0:2];

    // OLA element pipeline
    reg  signed [26:0] o_yr, o_yi;

    always @(posedge clk) begin
        b0_we <= 0; b1_we <= 0; cu_we <= 0; pk_we <= 0; en_we <= 0; dm_we <= 0;
        hi_we <= 0; rg_we <= 0; lb_we <= 0; dv_go <= 0; co_in_v <= 0;
        if (rst) begin
            ph <= P_IDLE;
            misses <= 0;
            fillSeq <= 0;
            fillW <= 0;
            for (ii = 0; ii < NS; ii = ii + 1) begin
                jq_wp[ii] <= 0;
                jq_rp[ii] <= 0;
                jq_n[ii]  <= 0;
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
            end
            // queue counts (push and pop may coincide)
            for (ii = 0; ii < NS; ii = ii + 1)
                jq_n[ii] <= jq_n[ii] + ((job_valid && job_ready && job_slot == ii) ? 3'd1 : 3'd0)
                                     - ((ph == P_IDLE && pick_ok && pick == ii) ? 3'd1 : 3'd0);

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
            P_IDLE: if (pick_ok) begin
                {jclear, jwin, jreset, jnotr, jc, jdl, jph, jahead, jinc, jrate, jbase, jframes}
                    <= jq[pick*QD + jq_rp[pick]];
                js <= pick;
                jq_rp[pick] <= jq_rp[pick] + 1'b1;
                ph <= P_TAKE;
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
                    dv_d <= {38'd0, jrate};
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
                    s_sum <= $signed({2'b00, pos}) + (($signed(err) * $signed({1'b0, jrate})) >>> 8)
                           - $signed({13'd0, jrate, 19'd0});
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
                fr_v1 <= 0; fr_v2 <= 0; fr_v3 <= 0; fr_v4 <= 0;
                ph <= P_FRAME;
            end
            // ============================================================ frame
            P_FRAME: begin
                // fetch generator: one line decision per cycle
                if (ft < 13'd4096) begin
                    if (!f_new || (f_allow && req_ready)) begin
                        if (f_new) begin
                            fLast <= f_line;
                            fLastV <= 1;
                            fSeq <= fSeq + 1'b1;
                        end
                        if (fHalf) begin
                            fHalf <= 0;
                            ft <= ft + 1'b1;
                            fP <= ({1'b0, fP} + step_q >= {1'b0, F}) ? fP + step_q - F : fP + step_q;
                        end else
                            fHalf <= 1;
                    end
                end
                // consumer: one sample per cycle when its lines are in
                fr_v1 <= 0;
                if (c_ok) begin
                    lb0_ra <= {c_s0[1:0], c_w0[5:0]};
                    lb1_ra <= {c_s1[1:0], c_w1[5:0]};
                    cs_a <= ct[11:0] + 12'd2048;           // j = t mod 4096, t = ct - 2048
                    fr_v1 <= 1;
                    fr_fr1 <= cP[31:16];
                    fr_j1 <= ct[11:0] + 12'd2048;
                    cSeq <= c_s1 + 1'b1;
                    cLast <= c_l1;
                    cLastV <= 1;
                    cOld <= c_s0;
                    ct <= ct + 1'b1;
                    cP <= ({1'b0, cP} + step_q >= {1'b0, F}) ? cP + step_q - F : cP + step_q;
                end
                // pipeline: v1 RAM addr -> v2 data -> v3 interpolated -> v4 windowed
                fr_v2 <= fr_v1;
                fr_fr2 <= fr_fr1;
                fr_j2 <= fr_j1;
                fr_v3 <= fr_v2;
                fr_j3 <= fr_j2;
                if (fr_v2)
                    fr_s <= $signed(lb0_q) + (fr_ip >>> 16);
                fr_v4 <= fr_v3;
                fr_j4 <= fr_j3;
                if (fr_v3)
                    fr_xw <= fr_s * $signed({1'b0, (18'd65536 + cs_c) >> 1});
                if (fr_v4) begin
                    if (!fr_j4[0])
                        fr_even <= fr_xwr[26:0];
                    else begin
                        if (par11(fr_j4[11:1])) begin
                            b1_we <= 1;
                            b1_wa <= fr_j4[11:2];
                            b1_wd <= {fr_even, fr_xwr[26:0]};
                        end else begin
                            b0_we <= 1;
                            b0_wa <= fr_j4[11:2];
                            b0_wd <= {fr_even, fr_xwr[26:0]};
                        end
                    end
                end
                if (ct == 13'd4096 && !fr_v1 && !fr_v2 && !fr_v3 && !fr_v4) begin
                    ph <= P_FFT;
                    inv <= 0;
                    st <= 0;
                    bf <= 0;
                    bf_run <= 1;
                    fv <= 0;
                end
            end
            // ============================================================ FFT / IFFT
            P_FFT, P_IFFT: begin
                // issue
                fv <= {fv[6:0], 1'b0};
                if (bf_run) begin : issue
                    reg [10:0] span, kk, ia, ja;
                    span = 11'd1024 >> st;
                    kk = bf & (span - 1'b1);
                    ia = ((bf & ~(span - 1'b1)) << 1) | kk;
                    ja = ia | span;
                    if (par11(ia)) begin
                        b1_ra <= ia[10:1];
                        b0_ra <= ja[10:1];
                    end else begin
                        b0_ra <= ia[10:1];
                        b1_ra <= ja[10:1];
                    end
                    cs_a <= {kk, 1'b0} << st;
                    f_i[0] <= ia;
                    f_j[0] <= ja;
                    fv[0] <= 1;
                    if (bf == 11'd1023)
                        bf_run <= 0;
                    bf <= bf + 1'b1;
                end
                for (ii = 1; ii < 8; ii = ii + 1) begin
                    f_i[ii] <= f_i[ii-1];
                    f_j[ii] <= f_j[ii-1];
                end
                // fv[1]: RAM data valid (and cs stage 1)
                if (fv[1]) begin
                    if (par11(f_i[1])) begin
                        f_ar <= b1_q[53:27]; f_ai <= b1_q[26:0];
                        f_br <= b0_q[53:27]; f_bi <= b0_q[26:0];
                    end else begin
                        f_ar <= b0_q[53:27]; f_ai <= b0_q[26:0];
                        f_br <= b1_q[53:27]; f_bi <= b1_q[26:0];
                    end
                end
                // fv[2]: sums, multiplier operands (cs valid now)
                if (fv[2]) begin
                    f_sr <= f_ar + f_br;
                    f_si <= f_ai + f_bi;
                    mu_dr <= f_ar - f_br;
                    mu_di <= f_ai - f_bi;
                    mu_c <= cs_c;
                    mu_s <= inv ? -cs_s : cs_s;
                end
                f_sr2 <= f_sr; f_si2 <= f_si;
                f_sr3 <= f_sr2; f_si3 <= f_si2;
                // fv[5]: products ready (mu_* registered twice after fv[3])
                if (fv[5]) begin : wb
                    reg signed [47:0] pr, pi;
                    reg signed [DW-1:0] ar, ai, br, bi;
                    // W = c - j s (forward) / c + j s (inverse, s negated above)
                    pr = mu_rc + mu_is;
                    pi = mu_ic - mu_rs;
                    if (!inv) begin
                        ar = sat27((f_sr3 + 28'sd1) >>> 1);
                        ai = sat27((f_si3 + 28'sd1) >>> 1);
                        br = sat27((pr + 48'sd65536) >>> 17);
                        bi = sat27((pi + 48'sd65536) >>> 17);
                    end else begin
                        ar = sat27(f_sr3);
                        ai = sat27(f_si3);
                        br = sat27((pr + 48'sd32768) >>> 16);
                        bi = sat27((pi + 48'sd32768) >>> 16);
                    end
                    if (par11(f_i[5])) begin
                        b1_we <= 1; b1_wa <= f_i[5][10:1]; b1_wd <= {ar, ai};
                        b0_we <= 1; b0_wa <= f_j[5][10:1]; b0_wd <= {br, bi};
                    end else begin
                        b0_we <= 1; b0_wa <= f_i[5][10:1]; b0_wd <= {ar, ai};
                        b1_we <= 1; b1_wa <= f_j[5][10:1]; b1_wd <= {br, bi};
                    end
                end
                // next stage when drained
                if (!bf_run && fv == 0) begin
                    if (st == 4'd10) begin
                        if (ph == P_FFT) begin
                            ph <= P_A0;
                            k <= 0;
                            pc <= 0;
                        end else begin
                            ph <= jwin == 2'd1 ? P_FADE : P_OLA;
                            t13 <= 0;
                            pc <= 0;
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
            P_A0: case (pc)
                0: begin   // read A = Z[k] (loc rev(k)), then B = Z[M-k]
                    begin : rdA
                        reg [10:0] la;
                        la = rev11(k[10:0]);
                        b0_ra <= la[10:1];
                        b1_ra <= la[10:1];
                    end
                    cs_a <= k;
                    pc <= 1;
                end
                1: begin
                    begin : rdB
                        reg [10:0] lb;
                        lb = rev11(11'd0 - k[10:0]);
                        b0_ra <= lb[10:1];
                        b1_ra <= lb[10:1];
                    end
                    pc <= 2;
                end
                2: begin
                    begin : gA
                        reg [10:0] la;
                        la = rev11(k[10:0]);
                        if (par11(la)) begin a_ar <= b1_q[53:27]; a_ai <= b1_q[26:0]; end
                        else           begin a_ar <= b0_q[53:27]; a_ai <= b0_q[26:0]; end
                    end
                    pc <= 3;
                end
                3: begin
                    begin : gB
                        reg [10:0] lb;
                        reg signed [DW-1:0] br, bi;
                        lb = rev11(11'd0 - k[10:0]);
                        if (par11(lb)) begin br = b1_q[53:27]; bi = b1_q[26:0]; end
                        else           begin br = b0_q[53:27]; bi = b0_q[26:0]; end
                        // B = conj(Z[M-k])
                        a_er <= a_ar + br;
                        a_ei <= a_ai - bi;
                        mu_dr <= a_ar - br;
                        mu_di <= a_ai + bi;
                        a_dr <= a_ar - br;
                        a_di <= a_ai + bi;
                        mu_c <= cs_c;
                        mu_s <= cs_s;
                    end
                    pc <= 4;
                end
                4: pc <= 5;
                5: pc <= 6;
                6: begin : wrX
                    // X[k] = ((E*65536 + (c*Di - s*Dr)) , (Ei*65536 - c*Dr - s*Di)) >> 17
                    // X[M-k] = (Er*65536 - c*Di + s*Dr, -Ei*65536 - c*Dr - s*Di) >> 17
                    reg signed [47:0] sxr, sxi, syr, syi;
                    reg signed [DW-1:0] Xr, Xi, Yr, Yi;
                    reg [10:0] la, lb;
                    sxr = ($signed(a_er) <<< 16) + mu_ic - mu_rs;
                    sxi = ($signed(a_ei) <<< 16) - mu_rc - mu_is;
                    syr = ($signed(a_er) <<< 16) - mu_ic + mu_rs;
                    syi = -($signed(a_ei) <<< 16) - mu_rc - mu_is;
                    Xr = sat27((sxr + 48'sd65536) >>> 17);
                    Xi = sat27((sxi + 48'sd65536) >>> 17);
                    Yr = sat27((syr + 48'sd65536) >>> 17);
                    Yi = sat27((syi + 48'sd65536) >>> 17);
                    la = rev11(k[10:0]);
                    lb = rev11(11'd0 - k[10:0]);
                    if (k == 0) begin
                        // X[0] in place, X[M] (c = -1, s = 0: Er*65536 + 65536*... ) to the Nyquist register
                        b0_we <= 1; b0_wa <= 0; b0_wd <= {Xr, Xi};
                        nyq_r <= sat27((($signed(a_er) <<< 16) - (($signed(a_di)) <<< 16) + 48'sd65536) >>> 17);
                        nyq_i <= sat27((($signed(a_ei) <<< 16) + (($signed(a_dr)) <<< 16) + 48'sd65536) >>> 17);
                    end else begin
                        if (par11(la)) begin b1_we <= 1; b1_wa <= la[10:1]; b1_wd <= {Xr, Xi}; end
                        else           begin b0_we <= 1; b0_wa <= la[10:1]; b0_wd <= {Xr, Xi}; end
                        x_ar <= Yr;
                        x_ai <= Yi;
                    end
                    pc <= (k == 0 || k == 12'd1024) ? 5'd8 : 5'd7;
                end
                7: begin
                    begin : wrY
                        reg [10:0] lb;
                        lb = rev11(11'd0 - k[10:0]);
                        if (par11(lb)) begin b1_we <= 1; b1_wa <= lb[10:1]; b1_wd <= {x_ar, x_ai}; end
                        else           begin b0_we <= 1; b0_wa <= lb[10:1]; b0_wd <= {x_ar, x_ai}; end
                    end
                    pc <= 8;
                end
                8: begin
                    if (k == 12'd1024) begin
                        ph <= P_A1;
                        k <= 0;
                        pc <= 0;
                        a1_desc <= 0;
                        a1_cnt <= 0;
                        a1_best <= 0;
                        a1_strong <= 0;
                        a1_m <= 0;
                        pv <= 0;
                    end else begin
                        k <= k + 1'b1;
                        pc <= 0;
                    end
                end
            endcase
            // ============================================================ A1
            P_A1: begin
                // issue: read X[k] (Nyquist from register) into the CORDIC
                pv <= {pv[4:0], 1'b0};
                if (k <= 12'd2048 && pc == 0) begin
                    begin : rdX
                        reg [10:0] la;
                        la = rev11(k[10:0]);
                        b0_ra <= la[10:1];
                        b1_ra <= la[10:1];
                        pp_pk[0] <= k;
                    end
                    pv[0] <= 1;
                    k <= k + 1'b1;
                end
                for (ii = 1; ii < 6; ii = ii + 1)
                    pp_pk[ii] <= pp_pk[ii-1];
                if (pv[1]) begin
                    co_in_v <= 1;
                    co_in_rot <= 0;
                    co_in_tag <= pp_pk[1];
                    if (pp_pk[1] == 12'd2048) begin
                        co_in_x <= nyq_r; co_in_y <= nyq_i;
                    end else if (par11(rev11(pp_pk[1][10:0]))) begin
                        co_in_x <= b1_q[53:27]; co_in_y <= b1_q[26:0];
                    end else begin
                        co_in_x <= b0_q[53:27]; co_in_y <= b0_q[26:0];
                    end
                end
                // CORDIC output, in bin order
                if (co_v) begin : a1out
                    reg [15:0] Lv, d, rf;
                    reg signed [32:0] x;
                    Lv = mlog2(co_mag);
                    if (co_tag == 0) begin
                        rf = 16'd0;
                    end else begin
                        d = a1_prevph - co_ph;
                        x = ($signed(d) * fmul) >>> 8;
                        rf = a1_rot + x[15:0] + d;
                    end
                    a1_prevph <= co_ph;
                    a1_rot <= rf;
                    if (co_tag == 12'd2048) begin
                        n_mag <= co_mag; n_L <= Lv; n_ph <= co_ph; n_rotF <= rf;
                    end else begin
                        cu_we <= 1;
                        cu_wa <= co_tag[10:0];
                        cu_wd <= {co_mag, Lv, co_ph, rf};
                    end
                    // enumeration window: evaluate m = tag - 1 with e(m) = L[tag-1], e(m+1) = L[tag]
                    a1_L[0] <= Lv;
                    if (co_tag >= 12'd2)
                        begin : enum1
                            reg lt;
                            lt = a1_L[0] < Lv;            // e(m) < e(m+1), m = tag-1 >= 1
                            if (!a1_desc) begin
                                if (!lt) begin
                                    a1_peak <= co_tag - 1'b1;
                                    a1_desc <= 1;
                                    if (a1_cnt == 0 || a1_L[0] > a1_best) begin
                                        a1_best <= a1_L[0];
                                        a1_strong <= a1_cnt;
                                    end
                                end
                            end else if (lt) begin
                                pk_we <= 1; pk_wa <= a1_cnt[9:0]; pk_wd <= a1_peak;
                                en_we <= 1; en_wa <= a1_cnt[9:0]; en_wd <= co_tag - 1'b1;
                                a1_cnt <= a1_cnt + 1'b1;
                                a1_desc <= 0;
                            end
                        end
                    if (co_tag == 12'd2048)
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
                    k <= 0;
                    pc <= 0;
                    pv <= 0;
                end
            end
            // ============================================================ B1: transient suppression
            P_B1: begin
                // i = k: read peak/end; then L[peak], pL[peak], L[end]; 1 partial per 3 cycles
                case (pc)
                0: begin
                    pk_ra <= k[9:0];
                    en_ra <= k[9:0];
                    pc <= 1;
                end
                1: pc <= 2;
                2: begin
                    cu_ra <= pk_q[10:0];
                    lc_ra <= en_q[10:0];
                    hi_ra <= {js, pk_q[10:0]};
                    pp_pk[0] <= pk_q;
                    pp_pk[1] <= en_q;
                    pc <= 3;
                end
                3: pc <= 4;
                4: begin : b1dec
                    reg [15:0] Lpk, pLpk, Len;
                    reg trans;
                    Lpk  = pp_pk[0] == 12'd2048 ? n_L : cu_q[47:32];
                    pLpk = pp_pk[0] == 12'd2048 ? h_nL[js] : hi_q[47:32];
                    Len  = pp_pk[1] == 12'd2048 ? n_L : (pp_pk[1] == 12'd2049 ? 16'd0 : lc_q);
                    trans = cont && !jnotr && k != 0 && k != P - 1'b1 && k[10:0] != strongest
                         && $signed({1'b0, Lpk}) > $signed({1'b0, pLpk}) + 18'sd599;
                    if (k == 0) begin
                        b_endp <= pp_pk[1];
                        b_lendp <= Len;
                    end else if (trans && b_lendp > Len) begin
                        // end[i-1] = end[i]
                        en_we <= 1; en_wa <= k[9:0] - 1'b1; en_wd <= pp_pk[1];
                        b_endp <= pp_pk[1];
                        b_lendp <= Len;
                    end else if (trans) begin
                        // end[i] = end[i-1]
                        en_we <= 1; en_wa <= k[9:0]; en_wd <= b_endp;
                    end else begin
                        b_endp <= pp_pk[1];
                        b_lendp <= Len;
                    end
                    if (k == P - 1'b1) begin
                        ph <= P_B2;
                        k <= 0;
                        pv <= 0;
                    end else
                        k <= k + 1'b1;
                    pc <= 0;
                end
                endcase
            end
            // ============================================================ B2: phase advance per partial
            P_B2: begin
                pv <= {pv[4:0], 1'b0};
                if (k < {1'b0, P} && pc == 0) begin
                    pk_ra <= k[9:0];
                    pp_i[0] <= k[10:0];
                    pv[0] <= 1;
                    k <= k + 1'b1;
                end
                for (ii = 1; ii < 6; ii = ii + 1) begin
                    pp_pk[ii] <= pp_pk[ii-1];
                    pp_i[ii] <= pp_i[ii-1];
                end
                // pv[1]: peak known -> read cur / hist
                if (pv[1]) begin
                    cu_ra <= pk_q[10:0];
                    hi_ra <= {js, pk_q[10:0]};
                    pp_pk[2] <= pk_q;
                end
                // pv[3]: data
                if (pv[3]) begin : b2a
                    reg [15:0] phc, rfc, pph, prt;
                    reg [31:0] aa;
                    phc = pp_pk[3] == 12'd2048 ? n_ph   : cu_q[31:16];
                    rfc = pp_pk[3] == 12'd2048 ? n_rotF : cu_q[15:0];
                    pph = pp_pk[3] == 12'd2048 ? h_nph[js]  : hi_q[31:16];
                    prt = pp_pk[3] == 12'd2048 ? h_nrot[js] : hi_q[15:0];
                    b2_off <= (rev ? -phc : phc) - (prev_rev ? -pph : pph);
                    b2_rotF <= rfc;
                    b2_prot <= prt;
                    aa = {hop[11:0], 20'd0};
                    b2_da <= ({phc, 16'd0} - {pph, 16'd0}) - pp_pk[3] * aa;
                end
                if (pv[4]) begin
                    b2_s32 <= {pp_pk[4][2:0], 29'd0} + b2_prod[31:0];
                    b2_off2 <= b2_off;
                    b2_rotF2 <= b2_rotF;
                    b2_prot2 <= b2_prot;
                end
                if (pv[5]) begin : b2w
                    reg [15:0] st16;
                    st16 = b2_s32[31:16];
                    dm_we <= 1;
                    dm_wa <= pp_i[5][9:0];
                    dm_wd <= cont ? (b2_prot2 - b2_off2 + st16 - b2_rotF2) : -b2_rotF2;
                end
                if (k >= {1'b0, P} && pv == 0 && !pv[0]) begin
                    ph <= P_C1;
                    k <= 0;
                    pc <= 0;
                    c_i <= 0;
                    c1v <= 0;
                end
            end
            // ============================================================ C1: rot[n] = rotF[n] + delta
            P_C1: case (pc)
                0: begin   // load partial c_i
                    en_ra <= c_i[9:0];
                    dm_ra <= c_i[9:0];
                    pc <= 1;
                end
                1: pc <= 2;
                2: begin
                    c_end <= en_q;
                    c_dl <= dm_q;
                    pc <= 3;
                end
                3: begin
                    // advance while n >= end and i < P-1
                    if (c_i < P - 1'b1 && k >= c_end) begin
                        c_i <= c_i + 1'b1;
                        pc <= 0;
                    end else begin
                        if (k == 12'd2048) begin
                            // bin 2048: rot = rot[2047] (set below)
                            ph <= P_C2;
                            k <= 0;
                            pc <= 0;
                            c2v <= 0;
                        end else begin
                            cu_ra <= k[10:0];
                            pc <= 4;
                        end
                    end
                end
                4: pc <= 5;
                5: begin : c1w
                    reg [15:0] r;
                    r = cu_q[15:0] + c_dl;
                    cu_we <= 1;
                    cu_wa <= k[10:0];
                    cu_wd <= {cu_q[74:16], r};
                    if (k == 12'd2047)
                        n_rot <= r;
                    k <= k + 1'b1;
                    pc <= 3;
                end
            endcase
            // ============================================================ C2: resynthesis + merge
            P_C2: case (pc)
                0: begin   // pair kA = k, kB = M - k (k = 0: kB = 2048)
                    cu_ra <= k[10:0];
                    pc <= 1;
                end
                1: begin
                    cu_ra <= (12'd2048 - k) & 12'h7FF;
                    pc <= 2;
                end
                2: begin : c2a
                    reg [15:0] phA, outA;
                    phA = cu_q[31:16];
                    outA = (rev ? -phA : phA) + cu_q[15:0];
                    co_in_v <= 1; co_in_rot <= 1; co_in_x <= cu_q[74:48]; co_in_y <= 0;
                    co_in_ph <= outA; co_in_tag <= 0;
                    c2_L[0] <= cu_q[47:32]; c2_ph[0] <= phA; c2_rot[0] <= cu_q[15:0];
                    pc <= 3;
                end
                3: begin : c2b
                    reg [26:0] mg;
                    reg [15:0] Lb, phB, rtB, outB;
                    if (k == 0) begin
                        mg = n_mag; Lb = n_L; phB = n_ph; rtB = n_rot;
                    end else begin
                        mg = cu_q[74:48]; Lb = cu_q[47:32]; phB = cu_q[31:16]; rtB = cu_q[15:0];
                    end
                    outB = (rev ? -phB : phB) + rtB;
                    co_in_v <= 1; co_in_rot <= 1; co_in_x <= mg; co_in_y <= 0;
                    co_in_ph <= outB; co_in_tag <= 1;
                    c2_L[1] <= Lb; c2_ph[1] <= phB; c2_rot[1] <= rtB;
                    pc <= 4;
                end
                4: begin
                    // history writes (bin kA now, kB next cycle)
                    hi_we <= 1;
                    hi_wa <= {js, k[10:0]};
                    hi_wd <= {c2_L[0], c2_ph[0], c2_rot[0]};
                    pc <= 5;
                end
                5: begin
                    if (k == 0) begin
                        h_nL[js] <= c2_L[1]; h_nph[js] <= c2_ph[1]; h_nrot[js] <= c2_rot[1];
                    end else if (k != 12'd1024) begin
                        hi_we <= 1;
                        hi_wa <= {js, 11'd2048 - k[10:0]};
                        hi_wd <= {c2_L[1], c2_ph[1], c2_rot[1]};
                    end
                    pc <= 6;
                end
                6: begin   // wait for the CORDIC
                    if (co_v && co_tag == 0) begin
                        c2_xr[0] <= co_xr[26:0];
                        c2_xi[0] <= co_yr[26:0];
                        pc <= 7;
                    end
                end
                7: begin : c2m
                    // E = XA + conj(XB), D = XA - conj(XB)
                    reg signed [26:0] br, bi;
                    br = co_xr[26:0];
                    bi = co_yr[26:0];
                    c2_er <= c2_xr[0] + br;
                    c2_ei <= c2_xi[0] - bi;
                    mu_dr <= c2_xr[0] - br;
                    mu_di <= c2_xi[0] + bi;
                    cs_a <= k;
                    pc <= 8;
                end
                8: pc <= 9;     // cs latency (cs_c valid two cycles after cs_a)
                9: pc <= 10;
                10: begin
                    mu_c <= cs_c;
                    mu_s <= cs_s;
                    pc <= 11;
                end
                11: pc <= 12;
                12: pc <= 13;
                13: begin : c2w
                    // Z'[kA] = ((Er*65536 - (Dr*s + Di*c)), (Ei*65536 + (Dr*c - Di*s))) >> 18
                    // Z'[kB] = ((Er*65536 + (Dr*s + Di*c)), (-Ei*65536 + (Dr*c - Di*s))) >> 18
                    reg signed [47:0] zr, zi, wr, wi;
                    reg [10:0] ka, kb;
                    zr = ($signed(c2_er) <<< 16) - (mu_rs + mu_ic);
                    zi = ($signed(c2_ei) <<< 16) + (mu_rc - mu_is);
                    wr = ($signed(c2_er) <<< 16) + (mu_rs + mu_ic);
                    wi = -($signed(c2_ei) <<< 16) + (mu_rc - mu_is);
                    ka = k[10:0];
                    kb = 11'd0 - k[10:0];
                    if (par11(ka)) begin b1_we <= 1; b1_wa <= ka[10:1]; b1_wd <= {sat27((zr + 48'sd131072) >>> 18), sat27((zi + 48'sd131072) >>> 18)}; end
                    else           begin b0_we <= 1; b0_wa <= ka[10:1]; b0_wd <= {sat27((zr + 48'sd131072) >>> 18), sat27((zi + 48'sd131072) >>> 18)}; end
                    x_ar <= sat27((wr + 48'sd131072) >>> 18);
                    x_ai <= sat27((wi + 48'sd131072) >>> 18);
                    pc <= (k == 0 || k == 12'd1024) ? 5'd15 : 5'd14;
                end
                14: begin : c2w2
                    reg [10:0] kb;
                    kb = 11'd0 - k[10:0];
                    if (par11(kb)) begin b1_we <= 1; b1_wa <= kb[10:1]; b1_wd <= {x_ar, x_ai}; end
                    else           begin b0_we <= 1; b0_wa <= kb[10:1]; b0_wd <= {x_ar, x_ai}; end
                    pc <= 15;
                end
                15: begin
                    if (k == 12'd1024) begin
                        // keep slot history scalars
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
                    end else begin
                        k <= k + 1'b1;
                        pc <= 0;
                    end
                end
            endcase
            // ============================================================ FADE / CLR (restart grain)
            P_FADE: case (pc)
                0: begin
                    rg_a <= {js, jc[11:0] + t13[11:0]};
                    wr_a <= 12'd2561 + t13[11:0];
                    pc <= 1;
                end
                1: pc <= 2;
                2: begin
                    rg_we <= 1;
                    rg_wd <= ($signed(rg_q) * $signed({1'b0, wr_d}) + 46'sd32768) >>> 16;
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
                rg_a <= {js, jc[11:0] + t13[11:0]};
                rg_wd <= 0;
                if (t13 == CLEAR_END - LT - 1) begin
                    ph <= P_OLA;
                    t13 <= 0;
                    pc <= 0;
                end else
                    t13 <= t13 + 1'b1;
            end
            // ============================================================ OLA
            P_OLA: case (pc)
                0: begin   // element n of t = t13 - 1024 (t even): read z[n] at rev(n)
                    begin : ro
                        reg [11:0] j;
                        reg [10:0] la;
                        j = t13[11:0] - 12'd1024;
                        la = rev11(j[11:1]);
                        b0_ra <= la[10:1];
                        b1_ra <= la[10:1];
                    end
                    pc <= 1;
                end
                1: pc <= 2;
                2: begin : lo
                    reg [11:0] j;
                    reg [10:0] la;
                    j = t13[11:0] - 12'd1024;
                    la = rev11(j[11:1]);
                    if (par11(la)) begin o_yr <= b1_q[53:27]; o_yi <= b1_q[26:0]; end
                    else           begin o_yr <= b0_q[53:27]; o_yi <= b0_q[26:0]; end
                    pc <= 3;
                    sub <= 0;
                end
                3: begin : sm   // sample t (sub 0: even, 1: odd): weight, ring read
                    reg signed [12:0] t;
                    reg [12:0] at;
                    reg signed [12:0] u;
                    reg [11:0] wa;
                    t = $signed({1'b0, t13}) - 13'sd1024 + sub;
                    at = t < 0 ? -t : t;
                    case (jwin)
                    2'd1: begin u = t;           wa = t < 0 ? 12'd0 : (t < 512 ? 12'd1025 + t : at); end
                    2'd2: begin u = t + 512;     wa = u < 0 ? 12'd0 : (u < 512 ? 12'd1537 + u : at); end
                    2'd3: begin u = t + 1024;    wa = u < 512 ? 12'd2049 + u : at; end
                    default: begin u = 0;        wa = at; end
                    endcase
                    o_wv <= !((jwin == 2'd1 && t < 0) || (jwin == 2'd2 && u < 0));
                    wr_a <= wa;
                    rg_a <= {js, jc[11:0] + t[11:0]};
                    o_time[0] <= jc + {{19{t[12]}}, t};
                    o_y <= sub ? o_yi : o_yr;
                    pc <= 4;
                end
                4: pc <= 5;
                5: begin
                    o_prod <= o_y * $signed({1'b0, wr_d});
                    o_w <= wr_d;
                    pc <= 6;
                end
                6: begin
                    rg_we <= o_wv && o_w != 0 && $signed(o_time[0] - now) > 0;
                    rg_wd <= rg_q + ((o_prod + 45'sd131072) >>> 18);
                    if (sub == 0) begin
                        sub <= 1;
                        pc <= 3;
                    end else if (t13 == 13'd2046) begin
                        ph <= P_DONE;
                    end else begin
                        t13 <= t13 + 2'd2;
                        pc <= 0;
                    end
                end
            endcase
            // ============================================================ clear job
            P_CLEAR: begin
                rg_we <= 1;
                rg_a <= {js, t13[11:0]};
                rg_wd <= 0;
                t13 <= t13 + 1'b1;
                if (t13 == 13'd4095)
                    ph <= P_DONE;
            end
            P_DONE: ph <= P_IDLE;
            default: ph <= P_IDLE;
            endcase
        end
    end
endmodule
