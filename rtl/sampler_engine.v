// Sampler engine: N_SLOTS BufGrainPlay slots (32 grains each) + the drum voice's
// gate/stutter, time-multiplexed on one sequential datapath.
//
// Arithmetic spec: model/sampler_model.cpp (bit exact). Register map:
// include/sampler_regs.h.
//
// Interfaces (all in the engine clock domain, nominally 49.152 MHz = 1024 x 48 kHz):
//   cmd_*   register writes, accepted only between samples (cmd_ready = idle)
//   req_*   cache-line read requests: 64 consecutive 16-bit words from req_addr
//   fill_*  line data, returned in request order, 64 words per request, never stalled
//   out_*   one mono s24 sample (Q8 sample units) per sample period
//   stat_*  status words, streamed out while idle (into a dual-clock RAM)
//
// Phase vocoder (rtl/pv_unit.v) per slot, selected by ALGO: every slot output is delayed
// by SMP_LATENCY samples and crossfaded between the grain path and the PV path (see
// Engine::mixSlot in the model). The PV unit shares the line request/fill path; grain
// requests have priority, fills are routed by a tag FIFO.
//
// Line cache: every grain owns two 64-sample lines of a cache RAM. Line L of a buffer
// covers SDRAM words BASE + 64L .. BASE + 64L + 63, i.e. buffer samples 64L-64 .. 64L-1
// (pre-pad of 64). A grain reading sample index i needs lines {B, B+1} with
// B = (i + 32) >> 6; line L lives in way L[0]. After every render the grain requests
// whichever of the lines for its *next* position are missing, so data has at least one
// sample period (~20 us) to arrive and ~32 samples in the steady state.
`default_nettype none

module sampler_engine #(
    parameter N_SLOTS           = 2,
    parameter SLOT_BITS         = 1,          // >= clog2(N_SLOTS), >= 1
    parameter CYCLES_PER_SAMPLE = 1024,
    parameter HANN_HEX          = "hann.hex",
    parameter EXP2_HEX          = "exp2.hex",
    parameter PV_SIN_HEX        = "pv_sin.hex",
    parameter PV_ATAN_HEX       = "pv_atan.hex",
    parameter PV_WIN_HEX        = "pv_win.hex"
) (
    input  wire               clk,
    input  wire               rst,

    input  wire               cmd_valid,
    output wire               cmd_ready,
    input  wire [7:0]         cmd_addr,
    input  wire [31:0]        cmd_data,

    output wire               req_valid,
    input  wire               req_ready,
    output wire [24:0]        req_addr,

    input  wire               fill_valid,
    input  wire [15:0]        fill_data,

    output reg                out_stb,
    output reg signed [23:0]  out_l,        // left / right mix (slot OUT bits)
    output reg signed [23:0]  out_r,
    output wire               tick_out,     // sample-period strobe (DACs latch out_l / out_r)

    output reg                stat_we,
    output reg  [5:0]         stat_addr,
    output reg  [31:0]        stat_data
);
    localparam GW  = SLOT_BITS + 5;          // global grain index width
    localparam NG  = N_SLOTS * 32;
    localparam CW  = GW + 7;                 // cache address width
    localparam PD  = 64;                     // pending-fill FIFO depth

    // ------------------------------------------------------------------ helpers
    function [31:0] xorshift;
        input [31:0] x;
        reg [31:0] a;
        begin
            a = x ^ (x << 13);
            a = a ^ (a >> 17);
            xorshift = a ^ (a << 5);
        end
    endfunction

    function signed [23:0] sat24;
        input signed [67:0] v;
        begin
            if (v > 68'sd8388607)       sat24 = 24'sd8388607;
            else if (v < -68'sd8388608) sat24 = -24'sd8388608;
            else                        sat24 = v[23:0];
        end
    endfunction

    function signed [17:0] sat18;
        input signed [23:0] v;
        begin
            if (v > 24'sd131071)        sat18 = 18'sd131071;
            else if (v < -24'sd131072)  sat18 = -18'sd131072;
            else                        sat18 = v[17:0];
        end
    endfunction

    function [26:0] pvrate;                  // PV read rate: |PITCH| clamped to [1/16, 4]
        input [31:0] p;
        reg [31:0] a;
        begin
            a = p[31] ? -p : p;
            if (a > 32'd67108864)     pvrate = 27'd67108864;
            else if (a < 32'd1048576) pvrate = 27'd1048576;
            else                      pvrate = a[26:0];
        end
    endfunction

    function [5:0] popcount32;
        input [31:0] x;
        integer k;
        begin
            popcount32 = 0;
            for (k = 0; k < 32; k = k + 1)
                popcount32 = popcount32 + x[k];
        end
    endfunction

    function [5:0] lowest_one;               // index of lowest set bit, 32 if none
        input [31:0] x;
        integer k;
        begin
            lowest_one = 6'd32;
            for (k = 31; k >= 0; k = k - 1)
                if (x[k]) lowest_one = k[5:0];
        end
    endfunction

    // ------------------------------------------------------------------ registers
    reg [3:0]  r_ctrl     [0:N_SLOTS-1];
    reg [23:0] r_base     [0:N_SLOTS-1];
    reg [23:0] r_frames   [0:N_SLOTS-1];
    reg [31:0] r_pos      [0:N_SLOTS-1];
    reg [31:0] r_phinc    [0:N_SLOTS-1];
    reg [31:0] r_bginc    [0:N_SLOTS-1];   // BG_INC: background phasor (no RATE)
    reg [31:0] r_pitch    [0:N_SLOTS-1];
    reg [31:0] r_wininc   [0:N_SLOTS-1];
    reg [31:0] r_overlap  [0:N_SLOTS-1];
    reg [16:0] r_amptgt   [0:N_SLOTS-1];
    reg [16:0] r_prob     [0:N_SLOTS-1];
    reg [15:0] r_sep      [0:N_SLOTS-1];
    reg [23:0] r_randpos  [0:N_SLOTS-1];
    reg [15:0] r_randpit  [0:N_SLOTS-1];
    reg [5:0]  r_maxg     [0:N_SLOTS-1];
    reg [23:0] r_envdur   [0:N_SLOTS-1];
    reg [6:0]  r_repeats  [0:N_SLOTS-1];
    reg [13:0] r_interval [0:N_SLOTS-1];
    reg [15:0] r_amp      [0:N_SLOTS-1];
    reg [15:0] r_algo     [0:N_SLOTS-1];   // ALGO_AMP
    reg [2:0]  r_algosel  [0:N_SLOTS-1];   // ALGO: [1:0] algorithm, [2] PV no transient
    reg [1:0]  r_out      [0:N_SLOTS-1];   // OUT: [0] left, [1] right
    reg [4:0]  pend_stb   [0:N_SLOTS-1];
    reg [15:0] g_master;
    reg [31:0] g_beatinc;

    // ------------------------------------------------------------------ slot state
    reg [55:0] s_phasor   [0:N_SLOTS-1];
    reg [31:0] s_trigph   [0:N_SLOTS-1];
    reg [16:0] s_hop      [0:N_SLOTS-1];
    reg [24:0] s_amp      [0:N_SLOTS-1];
    reg [31:0] s_rng      [0:N_SLOTS-1];
    reg [31:0] s_active   [0:N_SLOTS-1];
    reg        s_gate     [0:N_SLOTS-1];
    reg [23:0] s_envleft  [0:N_SLOTS-1];
    reg [16:0] s_level    [0:N_SLOTS-1];
    reg [55:0] s_poslat   [0:N_SLOTS-1];   // effective position latched on TRIG (Q32)
    reg [55:0] s_bgph     [0:N_SLOTS-1];   // background phasor (always advances)
    reg [6:0]  s_stutleft [0:N_SLOTS-1];   // stutter retrigger pulses left
    reg [13:0] s_stutph   [0:N_SLOTS-1];
    reg [2:0]  s_gst      [0:N_SLOTS-1];
    reg [23:0] s_out      [0:N_SLOTS-1];
    reg [15:0] s_miss     [0:N_SLOTS-1];

    // ------------------------------------------------------------------ memories
    // grain state: {winPhase 32, winInc 32, readPhase 56, readInc 48, base 24, frames 24} = 216
    // (each grain keeps the buffer it was started in: BASE/FRAMES switches crossfade)
    reg  [215:0] gmem [0:NG-1];
    reg  [GW-1:0] gm_ra;
    reg  [215:0] gm_rd;
    reg          gm_we;
    reg  [GW-1:0] gm_wa;
    reg  [215:0] gm_wd;
    always @(posedge clk) begin
        if (gm_we) gmem[gm_wa] <= gm_wd;
        gm_rd <= gmem[gm_ra];
    end

    // line tags, asynchronous read (LUT RAM): {way1 19, way0 19}
    reg  [37:0] tagmem [0:NG-1];
    reg         tag_we;
    reg  [GW-1:0] tag_wa;
    reg  [37:0] tag_wd;
    always @(posedge clk)
        if (tag_we) tagmem[tag_wa] <= tag_wd;

    // line cache
    reg  [15:0] cmem [0:(1<<CW)-1];
    reg  [CW-1:0] cm_ra;
    reg  [15:0] cm_rd;
    reg         cm_we;
    reg  [CW-1:0] cm_wa;
    reg  [15:0] cm_wd;
    always @(posedge clk) begin
        if (cm_we) cmem[cm_wa] <= cm_wd;
        cm_rd <= cmem[cm_ra];
    end

    // ROMs
    reg [16:0] hann_rom [0:1024];
    reg [17:0] exp2_rom [0:256];
    initial begin
        $readmemh(HANN_HEX, hann_rom);
        $readmemh(EXP2_HEX, exp2_rom);
    end
    reg  [10:0] hr_a;
    reg  [16:0] hr_d;
    reg  [8:0]  er_a;
    reg  [17:0] er_d;
    always @(posedge clk) begin
        hr_d <= hann_rom[hr_a];
        er_d <= exp2_rom[er_a];
    end

    // ------------------------------------------------------------------ wide multiplier
    // mp = ma * mb, 3-cycle latency (operands registered by the FSM).
    reg  signed [49:0] ma;
    reg  signed [17:0] mb;
    reg  signed [50:0] m_ph;   // ma[49:17] * mb
    reg  signed [35:0] m_pl;   // ma[16:0]  * mb
    reg  signed [67:0] mp;
    always @(posedge clk) begin
        m_ph <= $signed(ma[49:17]) * mb;
        m_pl <= $signed({1'b0, ma[16:0]}) * mb;
        mp   <= ($signed(m_ph) <<< 17) + m_pl;
    end

    // ------------------------------------------------------------------ fill path
    reg  [GW+2:0] pfifo [0:PD-1];            // {gid, way, seq[1:0]}
    reg  [5:0]  pf_wp, pf_rp;
    reg  [6:0]  pf_cnt;
    reg  [5:0]  f_cnt;
    reg  [1:0]  seqr [0:NG*2-1];
    reg         vld  [0:NG*2-1];
    wire [GW+2:0] pf_head = pfifo[pf_rp];
    wire        pend_ok = pf_cnt < PD - 2;

    // ------------------------------------------------------------------ FSM
    localparam
        ST_IDLE   = 6'd0,  ST_SLOT   = 6'd1,  ST_GNEXT  = 6'd2,  ST_G1     = 6'd3,
        ST_G2     = 6'd4,  ST_G3     = 6'd5,  ST_G4     = 6'd6,  ST_G5     = 6'd7,
        ST_G6     = 6'd8,  ST_ENS0   = 6'd9,  ST_ENS1   = 6'd10, ST_ENS2   = 6'd11,
        ST_ENS3   = 6'd12, ST_AMP0   = 6'd13, ST_AMP1   = 6'd14, ST_TI0    = 6'd15,
        ST_TI1    = 6'd16, ST_TI2    = 6'd17, ST_TI3    = 6'd18, ST_TI4    = 6'd19,
        ST_TI5    = 6'd20, ST_TRG    = 6'd21, ST_RNG    = 6'd22, ST_HOP0   = 6'd23,
        ST_HOP1   = 6'd24, ST_HOP2   = 6'd25, ST_DIV    = 6'd26, ST_E0     = 6'd27,
        ST_E1     = 6'd28, ST_E2     = 6'd29, ST_E3     = 6'd30, ST_E4     = 6'd31,
        ST_E5     = 6'd32, ST_O0     = 6'd33, ST_O1     = 6'd34, ST_O2     = 6'd35,
        ST_O3     = 6'd36, ST_O4     = 6'd37, ST_O5     = 6'd38, ST_O6     = 6'd39,
        ST_WRAP   = 6'd40, ST_ALLOC  = 6'd41, ST_PH     = 6'd42, ST_ENV    = 6'd48, ST_OUT0   = 6'd49, ST_OUT1   = 6'd50, ST_OUT2   = 6'd51,
        ST_OUT3   = 6'd52, ST_NEXT   = 6'd53, ST_MIX0   = 6'd54, ST_MIX1   = 6'd55,
        ST_MULW   = 6'd56, ST_E4B    = 6'd58, ST_G1B    = 6'd59, ST_G3B    = 6'd60, ST_SLOT2  = 6'd61,
        ST_PV0    = 6'd43, ST_PVJ    = 6'd44, ST_PVE    = 6'd45, ST_PVM0   = 6'd46, ST_PVM1   = 6'd47,
        ST_PVM2   = 6'd57, ST_PVM3   = 6'd62, ST_PVM4   = 6'd63;

    reg  [5:0]  state, mret;
    reg  [1:0]  mcnt;
    reg  [SLOT_BITS-1:0] cs;

    // sample timing
    reg  [10:0] tick_cnt;
    reg         tick_pend;
    reg  [15:0] cyc, cyc_last, cyc_max;
    reg  [31:0] samples;
    reg  [63:0] beat;
    reg  [31:0] req_drops;

    // per-slot scratch
    reg  [31:0] gmask;
    reg  [55:0] pos_bg;          // effective position of this sample (POS [+ background])
    reg  [4:0]  gidx;
    reg  signed [41:0] acc;
    reg  signed [23:0] y;
    reg  [4:0]  stb;
    reg         trig;
    reg  [31:0] trig_inc;
    reg  [39:0] tbase;
    reg  [67:0] p1;
    reg         onset;
    reg  [31:0] r1, r2, r3, r4;
    reg  [1:0]  rcnt;
    reg  [4:0]  vfree;
    reg  [15:0] frac;
    reg  [32:0] drem;
    reg  [4:0]  dcnt;
    reg  signed [16:0] e12;
    reg  [17:0] e0v;
    reg  signed [18:0] e_de;
    reg  signed [47:0] rinc;
    reg  signed [57:0] rp;
    reg  [67:0] q1;
    reg  [55:0] off;
    reg  [31:0] wp0;
    reg  signed [27:0] o1;
    reg  signed [26:0] mix_l, mix_r;

    // grain scratch
    reg  [31:0] g_wp, g_wi;
    reg  [55:0] g_rp;
    reg  signed [47:0] g_ri;
    reg         g_hit;
    reg  signed [15:0] g_s0, g_s1;
    reg  [16:0] g_h0, g_h1;
    reg  [15:0] g_fr, g_wf;
    reg  signed [33:0] g_pa;
    reg  signed [34:0] g_pb;
    reg  signed [35:0] g_pc;
    reg  [32:0] g_wpn;
    reg  [23:0] g_bs, g_frm;
    reg  signed [57:0] g_rpn;

    // ensure-lines scratch
    reg  [GW-1:0] e_gid;
    reg  [23:0] e_base;
    reg  [23:0] e_idx;
    reg         e_fresh;
    reg  [5:0]  e_ret;
    reg  [18:0] e_la;
    reg         e_na, e_nb;
    reg  [18:0] e_t0, e_t1;


    // ---- combinational views of the current slot
    wire [3:0]  c_ctrl   = r_ctrl[cs];
    wire [23:0] c_frames = r_frames[cs];
    wire signed [57:0] c_FQ = $signed({2'b00, r_frames[cs], 32'd0});
    wire [55:0] c_posq   = {r_pos[cs], 24'd0};
    wire [55:0] c_pinned = (c_posq < {r_frames[cs], 32'd0}) ? c_posq : 56'd0;

    // ---- grain-loop combinational datapath
    wire [215:0] gst     = gm_rd;
    wire [31:0]  gst_wp  = gst[215:184];
    wire [31:0]  gst_wi  = gst[183:152];
    wire [55:0]  gst_rp  = gst[151:96];
    wire [47:0]  gst_ri  = gst[95:48];
    wire [23:0]  gst_bs  = gst[47:24];
    wire [23:0]  gst_fr  = gst[23:0];
    // line/way/hit of the current grain, from the *registered* state (G1B)
    wire [24:0]  gv0     = {1'b0, g_rp[55:32]} + 25'd64;
    wire [24:0]  gv1     = gv0 + 25'd1;
    wire [37:0]  gtags   = tagmem[{cs, gidx}];
    wire [18:0]  gl0     = gv0[24:6];
    wire [18:0]  gl1     = gv1[24:6];
    wire         ghit0   = (gl0[0] ? gtags[37:19] : gtags[18:0]) == gl0 && vld[{cs, gidx, gl0[0]}];
    wire         ghit1   = (gl1[0] ? gtags[37:19] : gtags[18:0]) == gl1 && vld[{cs, gidx, gl1[0]}];
    wire [5:0]   gm_low  = lowest_one(gmask);
    reg  [5:0]   g_o1;
    reg          g_w1;

    // ---- ensure-lines combinational
    wire [24:0]  e_b      = ({1'b0, e_idx} + 25'd32) >> 6;
    wire [37:0]  e_trow   = tagmem[e_gid];
    wire [18:0]  e_tA     = e_fresh ? 19'h7FFFF : (e_b[0] ? e_trow[37:19] : e_trow[18:0]);
    wire [18:0]  e_tB     = e_fresh ? 19'h7FFFF : (e_b[0] ? e_trow[18:0]  : e_trow[37:19]);
    wire [18:0]  e_lb     = e_la + 19'd1;

    wire   g_req_valid = pend_ok && ((state == ST_ENS1 && e_na) || (state == ST_ENS2 && e_nb));
    wire [24:0] g_req_addr = {1'b0, e_base}
                     + {((state == ST_ENS1) ? e_la : e_lb), 6'd0};
    wire   req_fire  = g_req_valid && req_ready;

    // ---- request arbitration (grain engine first) and fill routing by tag
    wire        pv_req_valid;
    wire [24:0] pv_req_addr;
    assign req_valid = g_req_valid || pv_req_valid;
    assign req_addr  = g_req_valid ? g_req_addr : pv_req_addr;
    wire        pv_req_ready = req_ready && !g_req_valid;
    reg         tagq [0:127];            // 1: PV line
    reg  [6:0]  tq_wp, tq_rp;
    reg  [5:0]  tq_w;
    wire        tq_head = tagq[tq_rp];
    wire        fill_g  = fill_valid && !tq_head;
    wire        fill_p  = fill_valid && tq_head;
    always @(posedge clk) begin
        if (rst) begin
            tq_wp <= 0;
            tq_rp <= 0;
            tq_w  <= 0;
        end else begin
            if (req_valid && req_ready) begin
                tagq[tq_wp] <= !g_req_valid;
                tq_wp <= tq_wp + 1'b1;
            end
            if (fill_valid) begin
                tq_w <= tq_w + 1'b1;
                if (tq_w == 6'd63)
                    tq_rp <= tq_rp + 1'b1;
            end
        end
    end

    // ---- phase vocoder
    reg              pj_valid, pj_clear, pj_reset, pj_notr;
    reg  [SLOT_BITS-1:0] pj_slot;
    reg  [1:0]       pj_win;
    reg  [31:0]      pj_c, pj_dl;
    reg  [55:0]      pj_ph;
    reg  signed [15:0] pj_ahead;
    reg  signed [31:0] pj_inc;
    reg  [26:0]      pj_rate;
    reg  [23:0]      pj_base, pj_frames;
    wire             pj_ready;
    reg              em_req;
    wire             em_ack;
    wire signed [27:0] em_q;
    wire [N_SLOTS-1:0] pv_clr_busy;
    wire [31:0]      pv_misses;
    reg  [31:0]      pv_drops;
    pv_unit #(.NS(N_SLOTS), .SB(SLOT_BITS), .SIN_HEX(PV_SIN_HEX), .ATAN_HEX(PV_ATAN_HEX), .WIN_HEX(PV_WIN_HEX)) u_pv (
        .clk(clk), .rst(rst), .now(samples),
        .job_valid(pj_valid), .job_ready(pj_ready), .job_slot(pj_slot), .job_clear(pj_clear),
        .job_win(pj_win), .job_reset(pj_reset), .job_notr(pj_notr), .job_c(pj_c), .job_dl(pj_dl),
        .job_ph(pj_ph), .job_ahead(pj_ahead), .job_inc(pj_inc), .job_rate(pj_rate),
        .job_base(pj_base), .job_frames(pj_frames),
        .em_req(em_req), .em_slot(cs), .em_addr(samples[11:0]), .em_ack(em_ack), .em_q(em_q),
        .clr_busy(pv_clr_busy),
        .req_valid(pv_req_valid), .req_ready(pv_req_ready), .req_addr(pv_req_addr),
        .fill_valid(fill_p), .fill_data(fill_data),
        .misses(pv_misses), .busy()
    );
    // per-slot PV state
    reg              pv_act [0:N_SLOTS-1];
    reg  [1:0]       pv_alg [0:N_SLOTS-1];
    reg  [31:0]      pv_xfs [0:N_SLOTS-1];
    reg  [16:0]      pv_x   [0:N_SLOTS-1];
    reg  [31:0]      pv_nc  [0:N_SLOTS-1];
    // latency delay line {grain output 24, gate level 17}
    reg  [40:0]      dlm [0:N_SLOTS*128-1];
    reg  [SLOT_BITS+6:0] dl_ra, dl_wa;
    reg              dl_we;
    reg  [40:0]      dl_wd, dl_q;
    always @(posedge clk) begin
        if (dl_we) dlm[dl_wa] <= dl_wd;
        dl_q <= dlm[dl_ra];
    end
    // per-sample scratch of the slot being mixed
    reg              pvrst, x_en, x_act;
    reg  [55:0]      pvph;
    reg  signed [23:0] gq;
    reg  [1:0]       x_alg, x_nj, x_ji;
    reg              x_clr;
    reg  signed [23:0] x_pvs, x_op;
    reg  signed [23:0] x_gd;
    reg  [16:0]      x_lvd, x_xn;
    reg  signed [67:0] x_p1;
    reg              pj_rs, x_wait;
    reg  [31:0]      pj_c0, pj_stc;
    wire   req_way   = (state == ST_ENS1) ? e_la[0] : e_lb[0];

    assign cmd_ready = (state == ST_IDLE);
    assign tick_out  = (tick_cnt == CYCLES_PER_SAMPLE - 1);

    wire [13:0] c_int = r_interval[cs];

    integer i;

    always @(posedge clk) begin
        gm_we   <= 1'b0;
        tag_we  <= 1'b0;
        out_stb <= 1'b0;
        stat_we <= 1'b0;

        // sample tick
        if (tick_cnt == CYCLES_PER_SAMPLE - 1) begin
            tick_cnt  <= 0;
            tick_pend <= 1'b1;
        end else
            tick_cnt <= tick_cnt + 1'b1;
        if (cyc != 16'hFFFF) cyc <= cyc + 1'b1;

        case (state)
        // ============================================================== idle
        ST_IDLE: begin
            if (cmd_valid) begin : apply_cmd
                reg [2:0] sl;
                sl = cmd_addr[7:5];
                if (sl == 3'd7) begin
                    case (cmd_addr[4:0])
                    5'd0: g_master  <= cmd_data[15:0];
                    5'd1: g_beatinc <= cmd_data;
                    5'd2: if (cmd_data[0]) beat <= 64'd0;
                    5'd3: begin
                        cyc_max   <= 0;
                        req_drops <= 0;
                        for (i = 0; i < N_SLOTS; i = i + 1) s_miss[i] <= 0;
                    end
                    default: ;
                    endcase
                end else if (sl < N_SLOTS) begin
                    case (cmd_addr[4:0])
                    5'd0:  r_ctrl[sl]     <= cmd_data[3:0];
                    5'd1:  pend_stb[sl]   <= pend_stb[sl] | cmd_data[4:0];
                    5'd2:  r_base[sl]     <= cmd_data[23:0];
                    5'd3:  r_frames[sl]   <= cmd_data[23:0];
                    5'd4:  r_pos[sl]      <= cmd_data;
                    5'd5:  r_phinc[sl]    <= cmd_data;
                    5'd23: r_bginc[sl]    <= cmd_data;
                    5'd6:  r_pitch[sl]    <= cmd_data;
                    5'd7:  r_wininc[sl]   <= cmd_data;
                    5'd8:  r_overlap[sl]  <= cmd_data;
                    5'd9:  r_amptgt[sl]   <= cmd_data[16:0];
                    5'd10: r_prob[sl]     <= cmd_data[16:0];
                    5'd11: r_sep[sl]      <= cmd_data[15:0];
                    5'd12: r_randpos[sl]  <= cmd_data[23:0];
                    5'd13: r_randpit[sl]  <= cmd_data[15:0];
                    5'd14: r_maxg[sl]     <= cmd_data[5:0];
                    5'd15: s_rng[sl]      <= (cmd_data != 0) ? cmd_data : 32'h9E3779B9;
                    5'd16: r_envdur[sl]   <= cmd_data[23:0];
                    5'd17: r_repeats[sl]  <= cmd_data[6:0];
                    5'd18: r_interval[sl] <= cmd_data[13:0];
                    5'd19: r_algosel[sl]  <= cmd_data[2:0];
                    5'd20: r_amp[sl]      <= cmd_data[15:0];
                    5'd21: r_algo[sl]     <= cmd_data[15:0];
                    5'd22: r_out[sl]      <= cmd_data[1:0];
                    default: ;
                    endcase
                end
            end else if (tick_pend) begin
                tick_pend <= 1'b0;
                cyc       <= 16'd1;
                cs        <= 0;
                mix_l     <= 0;
                mix_r     <= 0;
                state     <= ST_SLOT;
            end
            // stream status words
            stat_we   <= 1'b1;
            stat_addr <= stat_addr + 1'b1;
            begin : stat_mux
                reg [5:0] a;
                reg [2:0] ss;
                a = stat_addr + 1'b1;
                ss = a[5:3];
                stat_data <= 32'd0;
                if (a < 56) begin
                    if (ss < N_SLOTS) begin
                        case (a[2:0])
                        3'd0: stat_data <= s_phasor[ss][55:24];
                        3'd1: stat_data <= {26'd0, popcount32(s_active[ss])};
                        3'd2: stat_data <= {29'd0, s_gst[ss]};
                        3'd3: stat_data <= {16'd0, s_miss[ss]};
                        3'd4: stat_data <= {8'd0, s_out[ss]};
                        3'd5: stat_data <= {(pv_x[ss] > 17'd65535) ? 16'hFFFF : pv_x[ss][15:0], 15'd0, pv_act[ss]};
                        default: ;
                        endcase
                    end
                end else begin
                    case (a)
                    6'd56: stat_data <= {16'd0, cyc_max};
                    6'd57: stat_data <= {16'd0, cyc_last};
                    6'd58: stat_data <= beat[63:32];
                    6'd59: stat_data <= beat[31:0];
                    6'd60: stat_data <= samples;
                    6'd61: stat_data <= req_drops;
                    6'd62: stat_data <= pv_misses + pv_drops;
                    6'd63: stat_data <= 32'h534D504C;
                    default: ;
                    endcase
                end
            end
        end

        // ============================================================== slot start
        ST_SLOT: begin : slot          // background phasor + effective position
            reg [55:0] bg;
            reg [56:0] pb;
            stb            <= pend_stb[cs];
            pend_stb[cs]   <= 5'd0;
            acc            <= 0;
            bg = s_bgph[cs];
            if (pend_stb[cs][4])                 // BG_RESET
                bg = 56'd0;
            if (bg >= {c_frames, 32'd0})         // buffer switched to a shorter one
                bg = 56'd0;
            pb = {1'b0, c_pinned};
            if (c_ctrl[3]) begin                 // POS_BG: (POS + background) mod FRAMES
                pb = pb + {1'b0, bg};
                if (pb >= {1'b0, c_frames, 32'd0})
                    pb = pb - {1'b0, c_frames, 32'd0};
            end
            if (!c_ctrl[0] || c_frames < 24'd128) begin
                s_gst[cs] <= 0;
                gq        <= 24'sd0;
                pvrst     <= 1'b0;
                state     <= ST_PV0;
            end else begin
                s_bgph[cs] <= bg;
                pos_bg     <= pb[55:0];
                state      <= ST_SLOT2;
            end
        end
        ST_SLOT2: begin : slot2        // stutter pulses, trig latch, playhead
            reg        rt;
            reg [55:0] latq;
            reg [55:0] ph;
            rt   = 1'b0;
            latq = s_poslat[cs];
            if (stb[0]) begin
                latq = pos_bg;
                s_poslat[cs]   <= pos_bg;
                s_stutleft[cs] <= r_repeats[cs];
                s_stutph[cs]   <= 14'd0;
            end else if (s_stutleft[cs] != 0) begin
                if (s_stutph[cs] + 1'b1 >= c_int) begin
                    s_stutph[cs]   <= 14'd0;
                    s_stutleft[cs] <= s_stutleft[cs] - 1'b1;
                    rt = 1'b1;
                end else
                    s_stutph[cs] <= s_stutph[cs] + 1'b1;
            end
            trig <= stb[0] | rt;
            pvrst <= stb[0] | rt | stb[2] | stb[3];        // PV restart: trig, retrig, KILL, SEEK
            if (stb[2]) begin
                s_active[cs] <= 32'd0;
                gmask        <= 32'd0;
            end else
                gmask <= s_active[cs];
            ph = s_phasor[cs];
            if (ph >= {c_frames, 32'd0})       // buffer switched to a shorter one
                ph = 56'd0;
            if (stb[3])                        // SEEK
                ph = c_pinned;
            if (stb[0] | rt)
                ph = (latq < {c_frames, 32'd0}) ? latq : 56'd0;
            s_phasor[cs] <= ph;
            state <= ST_GNEXT;
        end

        // ============================================================== grain loop
        ST_GNEXT: begin
            if (gmask == 32'd0)
                state <= ST_AMP0;
            else begin
                gidx  <= lowest_one(gmask);
                gmask <= gmask & (gmask - 1);
                state <= ST_G1;
            end
        end
        ST_G1: begin   // state word on gm_rd: register it before any arithmetic
            g_wp  <= gst_wp;
            g_wi  <= gst_wi;
            g_rp  <= gst_rp;
            g_ri  <= gst_ri;
            g_bs  <= gst_bs;
            g_frm <= gst_fr;
            g_fr  <= gst_rp[31:16];
            g_wf  <= gst_wp[21:6];
            state <= ST_G1B;
        end
        ST_G1B: begin  // lines/hit from registers; first cache + Hann addresses (below)
            g_hit <= ghit0 && ghit1;
            g_o1  <= gv1[5:0];
            g_w1  <= gl1[0];
            state <= ST_G2;
        end
        ST_G2: begin   // s0, h0 arriving
            g_s0  <= cm_rd;
            g_h0  <= hr_d;
            state <= ST_G3;
        end
        ST_G3: begin   // s1, h1 arriving: register them (RAM outputs are slow)
            g_s1  <= cm_rd;
            g_h1  <= hr_d;
            state <= ST_G3B;
        end
        ST_G3B: begin
            g_pa  <= $signed({g_s1[15], g_s1} - {g_s0[15], g_s0}) * $signed({1'b0, g_fr});
            g_pb  <= $signed({1'b0, g_h1} - {1'b0, g_h0}) * $signed({1'b0, g_wf});
            state <= ST_G4;
        end
        ST_G4: begin : g4
            reg signed [16:0] s;
            reg signed [17:0] w;
            s = g_s0 + (g_pa >>> 16);
            w = $signed({1'b0, g_h0}) + (g_pb >>> 16);
            g_pc  <= s * w;
            state <= ST_G5;
        end
        ST_G5: begin
            if (g_hit)
                acc <= acc + g_pc;
            else if (s_miss[cs] != 16'hFFFF)
                s_miss[cs] <= s_miss[cs] + 1'b1;
            g_wpn <= {1'b0, g_wp} + {1'b0, g_wi};
            g_rpn <= $signed({2'b00, g_rp}) + g_ri;
            state <= ST_G6;
        end
        ST_G6: begin : g6
            reg signed [57:0] rpw;
            reg signed [57:0] gfq;
            gfq = $signed({2'b00, g_frm, 32'd0});
            if (g_rpn >= gfq)        rpw = g_rpn - gfq;
            else if (g_rpn < 0)      rpw = g_rpn + gfq;
            else                     rpw = g_rpn;
            gm_we <= 1'b1;
            gm_wa <= {cs, gidx};
            gm_wd <= {g_wpn[31:0], g_wi, rpw[55:0], g_ri, g_bs, g_frm};
            if (g_wpn[32]) begin
                s_active[cs][gidx] <= 1'b0;
                state <= ST_GNEXT;
            end else begin
                e_gid   <= {cs, gidx};
                e_base  <= g_bs;
                e_idx   <= rpw[55:32];
                e_fresh <= 1'b0;
                e_ret   <= ST_GNEXT;
                state   <= ST_ENS0;
            end
        end

        // ============================================================== ensure lines
        ST_ENS0: begin
            e_la <= e_b[18:0];
            e_na <= e_tA != e_b[18:0];
            e_nb <= e_tB != (e_b[18:0] + 19'd1);
            if (e_b[0]) begin
                e_t1 <= e_tA;
                e_t0 <= e_tB;
            end else begin
                e_t0 <= e_tA;
                e_t1 <= e_tB;
            end
            if (e_tA == e_b[18:0] && e_tB == e_b[18:0] + 19'd1)
                state <= e_ret;
            else
                state <= ST_ENS1;
        end
        ST_ENS1, ST_ENS2: begin
            if ((state == ST_ENS1 && e_na) || (state == ST_ENS2 && e_nb)) begin
                if (req_fire) begin
                    if (req_way) e_t1 <= (state == ST_ENS1) ? e_la : e_lb;
                    else         e_t0 <= (state == ST_ENS1) ? e_la : e_lb;
                end else if (req_drops != 32'hFFFFFFFF)
                    req_drops <= req_drops + 1'b1;
            end
            state <= (state == ST_ENS1) ? ST_ENS2 : ST_ENS3;
        end
        ST_ENS3: begin
            tag_we <= 1'b1;
            tag_wa <= e_gid;
            tag_wd <= {e_t1, e_t0};
            state  <= e_ret;
        end

        // ============================================================== gain
        ST_AMP0: begin : amp0
            reg signed [26:0] d;
            reg signed [34:0] dk;
            reg signed [34:0] dq;
            reg [24:0] an;
            d  = $signed({2'b00, r_amptgt[cs], 8'd0}) - $signed({2'b00, s_amp[cs]});
            dk = ($signed(d) <<< 7) + ($signed(d) <<< 3);
            dq = dk >>> 16;
            an = s_amp[cs] + dq[24:0];
            s_amp[cs] <= an;
            ma    <= acc;
            mb    <= $signed({1'b0, an[24:8]});
            mret  <= ST_AMP1;
            state <= ST_MULW;
        end
        ST_AMP1: begin
            y <= sat24(mp >>> 24);
            if (c_ctrl[1])                     // POS_PIN (with POS_BG: POS + background)
                s_phasor[cs] <= pos_bg;
            pvph <= c_ctrl[1] ? pos_bg : s_phasor[cs];   // playhead of this sample's onsets
            ma    <= $signed({18'd0, r_wininc[cs]});
            mb    <= $signed({2'b00, r_overlap[cs][31:16]});
            state <= ST_TI0;
        end
        ST_TI0: begin
            ma    <= $signed({18'd0, r_wininc[cs]});
            mb    <= $signed({2'b00, r_overlap[cs][15:0]});
            state <= ST_TI1;
        end
        ST_TI1: state <= ST_TI2;
        ST_TI2: begin p1 <= mp; state <= ST_TI3; end
        ST_TI3: begin : ti3
            reg [67:0] full;
            full  = (p1 << 16) + mp;
            ma    <= $signed({10'd0, full[63:24]});
            mb    <= $signed({1'b0, s_hop[cs]});
            mret  <= ST_TI5;
            state <= ST_MULW;
        end
        ST_TI5: begin
            trig_inc <= (mp[67:48] != 0) ? 32'hFFFFFFFF : mp[47:16];
            state    <= ST_TRG;
        end
        ST_TRG: begin : trg
            reg [32:0] n;
            n = {1'b0, s_trigph[cs]} + {1'b0, trig_inc};
            frac <= 16'd0;
            rcnt <= 2'd0;
            if (trig) begin
                s_trigph[cs] <= 32'd0;
                state <= ST_RNG;
            end else begin
                s_trigph[cs] <= n[31:0];
                drem <= {1'b0, n[31:0]};
                if (trig_inc != 0 && n[32])
                    state <= ST_RNG;
                else
                    state <= ST_PH;
            end
        end

        // ============================================================== onset
        ST_RNG: begin : rng
            reg [31:0] x;
            x = xorshift(s_rng[cs]);
            s_rng[cs] <= x;
            case (rcnt)
            2'd0: r1 <= x;
            2'd1: r2 <= x;
            2'd2: r3 <= x;
            2'd3: r4 <= x;
            endcase
            rcnt <= rcnt + 1'b1;
            if (rcnt == 2'd3) begin
                ma    <= $signed({34'd0, r_sep[cs]});
                mb    <= 18'sd58982;
                mret  <= ST_HOP0;
                state <= ST_MULW;
            end
        end
        ST_HOP0: begin
            ma    <= $signed({34'd0, mp[31:16]});
            mb    <= $signed({{2{r1[31]}}, r1[31:16]});
            mret  <= ST_HOP1;
            state <= ST_MULW;
        end
        ST_HOP1: begin : hop1
            reg [31:0] fm;
            reg fire;
            reg signed [67:0] hd;
            hd = mp >>> 15;
            s_hop[cs] <= 17'd65536 + hd[16:0];
            fm    = ~s_active[cs];
            fire  = (r_prob[cs] > {1'b0, r2[31:16]}) && (fm != 0)
                    && (popcount32(s_active[cs]) < r_maxg[cs]);
            vfree <= lowest_one(fm);
            dcnt  <= 5'd0;
            if (!fire)
                state <= ST_PH;
            else if (trig)
                state <= ST_E0;
            else
                state <= ST_DIV;
        end
        ST_DIV: begin : div
            reg [33:0] r2x;
            r2x = {drem, 1'b0};
            if (r2x >= {2'b00, trig_inc}) begin
                drem <= r2x - {2'b00, trig_inc};
                frac <= {frac[14:0], 1'b1};
            end else begin
                drem <= r2x[32:0];
                frac <= {frac[14:0], 1'b0};
            end
            dcnt <= dcnt + 1'b1;
            if (dcnt == 5'd15)
                state <= ST_E0;
        end
        ST_E0: begin
            ma    <= $signed({{34{r_randpit[cs][15]}}, r_randpit[cs]});
            mb    <= $signed({{2{r3[31]}}, r3[31:16]});
            mret  <= ST_E1;
            state <= ST_MULW;
        end
        ST_E1: begin   // e12 ready; exp2 ROM address li driven below
            e12   <= mp >>> 15;
            state <= ST_E2;
        end
        ST_E2: state <= ST_E3;                 // er_d = E[li] next
        ST_E3: begin e0v <= er_d; state <= ST_E4; end
        ST_E4: begin           // E[li+1] arriving: register the slope
            e_de  <= $signed({1'b0, er_d}) - $signed({1'b0, e0v});
            state <= ST_E4B;
        end
        ST_E4B: begin : e4
            reg signed [23:0] dl;
            reg signed [23:0] dls;
            reg [17:0] m;
            dl  = e_de * $signed({1'b0, e12[3:0]});
            dls = dl >>> 4;
            m   = e0v + dls[17:0];
            ma    <= $signed({{18{r_pitch[cs][31]}}, r_pitch[cs]});
            mb    <= $signed(m);
            mret  <= ST_E5;
            state <= ST_MULW;
        end
        ST_E5: begin : e5
            reg signed [67:0] pe;
            reg signed [4:0] ei;
            pe = mp >>> 16;
            ei = e12 >>> 12;
            if (ei >= 0) pe = pe <<< ei;
            else         pe = pe >>> (-ei);
            rinc  <= {pe[39:0], 8'd0};
            ma    <= $signed({18'd0, r4});
            mb    <= $signed({10'd0, r_randpos[cs][23:16]});
            state <= ST_O0;
        end
        ST_O0: begin
            ma    <= $signed({18'd0, r4});
            mb    <= $signed({2'b00, r_randpos[cs][15:0]});
            state <= ST_O1;
        end
        ST_O1: begin
            ma    <= {{2{rinc[47]}}, rinc};
            mb    <= $signed({2'b00, frac});
            state <= ST_O2;
        end
        ST_O2: begin           // result of r4*rh
            q1    <= mp;
            ma    <= $signed({18'd0, r_wininc[cs]});
            mb    <= $signed({2'b00, frac});
            state <= ST_O3;
        end
        ST_O3: begin           // result of r4*rl
            off   <= (q1[55:0] << 16) + mp[55:0];
            state <= ST_O4;
        end
        ST_O4: begin : o4      // result of frac*readInc
            reg signed [67:0] fi;
            fi = mp >>> 16;
            rp    <= $signed({2'b00, s_phasor[cs]}) + $signed({2'b00, off}) + fi[57:0];
            state <= ST_O5;
        end
        ST_O5: begin           // result of frac*winInc
            wp0   <= mp[47:16];
            state <= ST_WRAP;
        end
        ST_WRAP: begin
            if (rp >= c_FQ)
                rp <= rp - c_FQ;
            else if (rp < 0)
                rp <= rp + c_FQ;
            else
                state <= ST_ALLOC;
        end
        ST_ALLOC: begin
            gm_we   <= 1'b1;
            gm_wa   <= {cs, vfree[4:0]};
            gm_wd   <= {wp0, r_wininc[cs], rp[55:0], rinc, r_base[cs], c_frames};
            s_active[cs][vfree[4:0]] <= 1'b1;
            e_gid   <= {cs, vfree[4:0]};
            e_base  <= r_base[cs];
            e_idx   <= rp[55:32];
            e_fresh <= 1'b1;
            e_ret   <= ST_PH;
            state   <= ST_ENS0;
        end

        // ============================================================== playhead
        ST_PH: begin : ph
            reg signed [57:0] pn;
            reg signed [57:0] bn;
            pn = $signed({2'b00, s_phasor[cs]}) + $signed({{18{r_phinc[cs][31]}}, r_phinc[cs], 8'd0});
            if (pn >= c_FQ)    pn = pn - c_FQ;
            else if (pn < 0)   pn = pn + c_FQ;
            s_phasor[cs] <= pn[55:0];
            bn = $signed({2'b00, s_bgph[cs]}) + $signed({{18{r_bginc[cs][31]}}, r_bginc[cs], 8'd0});
            if (bn >= c_FQ)    bn = bn - c_FQ;
            else if (bn < 0)   bn = bn + c_FQ;
            s_bgph[cs] <= bn[55:0];
            state <= ST_ENV;
        end

        // ============================================================== gate env
        ST_ENV: begin : env
            reg g;
            reg [23:0] el;
            reg [17:0] lv;
            g  = s_gate[cs];
            el = s_envleft[cs];
            if (stb[1]) g = 1'b0;
            if (stb[0]) begin
                g  = 1'b1;
                el = r_envdur[cs];
            end else if (g && !c_ctrl[2]) begin
                if (el != 0) el = el - 1'b1;
                if (el == 0) g = 1'b0;
            end
            if (g) begin
                lv = {1'b0, s_level[cs]} + 18'd1366;
                if (lv > 18'd65536) lv = 18'd65536;
            end else begin
                lv = (s_level[cs] > 17'd1366) ? {1'b0, s_level[cs]} - 18'd1366 : 18'd0;
            end
            s_gate[cs]    <= g;
            s_envleft[cs] <= el;
            s_level[cs]   <= lv[16:0];
            s_gst[cs]     <= (s_stutleft[cs] != 0) ? 3'd4 : g ? ((lv < 18'd65536) ? 3'd1 : 3'd2)
                                                  : ((lv != 0) ? 3'd3 : 3'd0);
            ma    <= {{26{y[23]}}, y};
            mb    <= $signed({2'b00, r_amp[cs]});
            mret  <= ST_OUT0;
            state <= ST_MULW;
        end
        ST_OUT0: begin
            o1    <= mp >>> 12;
            ma    <= $signed(mp >>> 12);
            mb    <= $signed({1'b0, s_level[cs]});
            mret  <= ST_OUT1;
            state <= ST_MULW;
        end
        ST_OUT1: begin
            ma    <= $signed(mp >>> 16);
            mb    <= $signed({2'b00, r_algo[cs]});
            mret  <= ST_OUT2;
            state <= ST_MULW;
        end
        ST_OUT2: begin : out2
            gq    <= sat24(mp >>> 15);
            state <= ST_PV0;
        end

        // ============================================================== phase vocoder / mix
        ST_PV0: begin : pv0
            reg en, act, rs;
            reg [1:0] alg;
            en  = c_ctrl[0] && c_frames >= 24'd128;
            alg = en ? r_algosel[cs][1:0] : 2'd0;
            act = pv_act[cs];
            rs  = 1'b0;
            x_clr <= 1'b0;
            if (!en && act) begin              // hard stop, like the grain path
                act = 1'b0;
                pv_x[cs] <= 17'd0;
                x_clr <= 1'b1;
            end
            if (alg != pv_alg[cs]) begin
                pv_alg[cs] <= alg;
                pv_xfs[cs] <= samples + 32'd128;
                if (alg == 2'd1 && !act) begin
                    act = 1'b1;
                    rs  = 1'b1;
                end
            end
            if (act && pvrst)
                rs = 1'b1;
            pv_act[cs] <= act;
            x_en  <= en;
            x_act <= act;
            x_alg <= alg;
            x_ji  <= 2'd0;
            if (rs) begin
                x_nj <= 2'd3;
                pv_nc[cs] <= samples + 32'd128 + 32'd1536;
            end else if (act && $signed(samples - (pv_nc[cs] - 32'd1024 - 32'd512)) >= 0) begin
                x_nj <= 2'd1;
                pv_nc[cs] <= pv_nc[cs] + 32'd512;
            end else
                x_nj <= 2'd0;
            // job fields common to all jobs of this sample
            pj_slot   <= cs;
            pj_notr   <= r_algosel[cs][2];
            pj_ph     <= pvph;
            // pinned: frozen, or following the background phasor (POS_BG)
            pj_inc    <= c_ctrl[1] ? (c_ctrl[3] ? $signed(r_bginc[cs]) : 32'sd0) : $signed(r_phinc[cs]);
            pj_rate   <= pvrate(r_pitch[cs]);
            pj_base   <= r_base[cs];
            pj_frames <= r_frames[cs];
            pj_rs     <= rs;
            pj_c0     <= samples + 32'd128;
            pj_stc    <= pv_nc[cs];
            state <= ST_PVJ;
        end
        ST_PVJ: begin
            // clear job first (deactivation), then up to three grain jobs
            pj_valid <= 1'b0;
            if (pj_valid && !pj_ready)
                pv_drops <= pv_drops + 1'b1;
            if (x_clr) begin
                pj_valid <= 1'b1;
                pj_clear <= 1'b1;
                x_clr <= 1'b0;
            end else if (x_ji != x_nj) begin
                pj_valid <= 1'b1;
                pj_clear <= 1'b0;
                if (pj_rs) begin
                    pj_win   <= 2'd1 + x_ji;
                    pj_reset <= x_ji == 2'd0;
                    pj_c     <= pj_c0 + {21'd0, x_ji, 9'd0};
                    pj_dl    <= (x_ji == 2'd0) ? pj_c0 : pj_c0 + 32'd256;
                    pj_ahead <= {5'd0, x_ji, 9'd0};
                end else begin
                    pj_win   <= 2'd0;
                    pj_reset <= 1'b0;
                    pj_c     <= pj_stc;
                    pj_dl    <= pj_stc - 32'd1024;
                    pj_ahead <= pj_stc - 32'd128 - samples;
                end
                x_ji <= x_ji + 1'b1;
            end else begin
                em_req <= x_act;
                dl_ra  <= {cs, samples[6:0]};
                x_wait <= 1'b1;            // delay-line read latency
                state  <= ST_PVE;
            end
        end
        ST_PVE: begin
            // ring entry of this sample (PV output), delay line read
            x_wait <= 1'b0;
            if (!x_wait && (!x_act || em_ack)) begin
                em_req <= 1'b0;
                x_pvs  <= !x_act ? 24'sd0
                        : (em_q > 28'sd8388607 ? 24'sd8388607 : (em_q < -28'sd8388608 ? -24'sd8388608 : em_q[23:0]));
                x_gd   <= dl_q[40:17];
                x_lvd  <= dl_q[16:0];
                dl_we  <= 1'b1;
                dl_wa  <= {cs, samples[6:0]};
                dl_wd  <= {gq, x_en ? s_level[cs] : 17'd0};
                // crossfade position (from SMP_LATENCY after the switch)
                if ($signed(samples - pv_xfs[cs]) >= 0) begin
                    if (x_alg == 2'd1)
                        x_xn <= (pv_x[cs] + 17'd256 > 17'd65536) ? 17'd65536 : pv_x[cs] + 17'd256;
                    else
                        x_xn <= (pv_x[cs] > 17'd256) ? pv_x[cs] - 17'd256 : 17'd0;
                end else
                    x_xn <= pv_x[cs];
                state <= ST_PVM0;
            end
        end
        ST_PVM0: begin
            dl_we <= 1'b0;
            pv_x[cs] <= x_xn;
            if (x_act && x_alg != 2'd1 && x_xn == 17'd0) begin   // faded out: stop the PV
                pv_act[cs] <= 1'b0;
                x_clr <= 1'b1;
            end
            ma    <= {{26{x_pvs[23]}}, x_pvs};
            mb    <= $signed({2'b00, r_amp[cs]});
            mret  <= ST_PVM1;
            state <= ST_MULW;
        end
        ST_PVM1: begin
            ma    <= $signed(mp >>> 12);
            mb    <= $signed({1'b0, x_lvd});
            mret  <= ST_PVM2;
            state <= ST_MULW;
        end
        ST_PVM2: begin
            ma    <= $signed(mp >>> 16);
            mb    <= $signed({2'b00, r_algo[cs]});
            mret  <= ST_PVM3;
            state <= ST_MULW;
        end
        ST_PVM3: begin : pvm3
            reg signed [23:0] op;
            op = sat24(mp >>> 15);
            x_op  <= op;
            ma    <= {{26{x_gd[23]}}, x_gd};
            mb    <= $signed({1'b0, 17'd65536 - x_xn});
            mret  <= ST_PVM4;
            state <= ST_MULW;
            x_ji  <= 2'd0;
        end
        ST_PVM4: begin : pvm4
            reg signed [23:0] so;
            if (x_ji == 2'd0) begin
                x_p1  <= mp;
                ma    <= {{26{x_op[23]}}, x_op};
                mb    <= $signed({1'b0, x_xn});
                mret  <= ST_PVM4;
                x_ji  <= 2'd1;
                state <= ST_MULW;
            end else begin
                so = sat24((x_p1 + mp) >>> 16);
                s_out[cs] <= so;
                if (r_out[cs][0]) mix_l <= mix_l + so;
                if (r_out[cs][1]) mix_r <= mix_r + so;
                // deactivation clear job (faded out)
                pj_valid <= x_clr;
                pj_clear <= 1'b1;
                pj_slot  <= cs;
                x_clr    <= 1'b0;
                state <= ST_NEXT;
            end
        end
        ST_NEXT: begin
            pj_valid <= 1'b0;
            if (pj_valid && !pj_ready)
                pv_drops <= pv_drops + 1'b1;
            if (cs == N_SLOTS - 1) begin
                ma    <= {{23{mix_l[26]}}, mix_l};
                mb    <= $signed({2'b00, g_master});
                mret  <= ST_MIX0;
                state <= ST_MULW;
            end else begin
                cs    <= cs + 1'b1;
                state <= ST_SLOT;
            end
        end
        ST_MIX0: begin
            out_l <= sat24(mp >>> 12);
            ma    <= {{23{mix_r[26]}}, mix_r};
            mb    <= $signed({2'b00, g_master});
            mret  <= ST_MIX1;
            state <= ST_MULW;
        end
        ST_MIX1: begin
            out_r      <= sat24(mp >>> 12);
            out_stb    <= 1'b1;
            beat       <= beat + {32'd0, g_beatinc};
            samples    <= samples + 1'b1;
            cyc_last   <= cyc;
            if (cyc > cyc_max) cyc_max <= cyc;
            state      <= ST_IDLE;
        end

        // ============================================================== multiplier wait
        ST_MULW: begin
            if (mcnt == 2'd1) begin
                mcnt  <= 2'd0;
                state <= mret;
            end else
                mcnt <= mcnt + 1'b1;
        end
        default: state <= ST_IDLE;
        endcase

        // ---------------------------------------------------------- fill engine
        cm_we <= 1'b0;
        if (fill_g) begin
            cm_we <= 1'b1;
            cm_wa <= {pf_head[GW+2:3], pf_head[2], f_cnt};
            cm_wd <= fill_data;
            f_cnt <= f_cnt + 1'b1;
        end
        // seq/valid bookkeeping: request (clears) has priority over completion (sets)
        if (fill_g && f_cnt == 6'd63 && seqr[pf_head[GW+2:2]] == pf_head[1:0])
            vld[pf_head[GW+2:2]] <= 1'b1;
        if (req_fire) begin
            seqr[{e_gid, req_way}] <= seqr[{e_gid, req_way}] + 1'b1;
            vld[{e_gid, req_way}]  <= 1'b0;
            pfifo[pf_wp] <= {e_gid, req_way, seqr[{e_gid, req_way}] + 2'd1};
            pf_wp <= pf_wp + 1'b1;
        end
        // a freshly allocated grain must not see stale lines
        if (state == ST_ALLOC) begin
            vld[{cs, vfree[4:0], 1'b0}] <= 1'b0;
            vld[{cs, vfree[4:0], 1'b1}] <= 1'b0;
        end
        if (fill_g && f_cnt == 6'd63)
            pf_rp <= pf_rp + 1'b1;
        pf_cnt <= pf_cnt + (req_fire ? 7'd1 : 7'd0) - ((fill_g && f_cnt == 6'd63) ? 7'd1 : 7'd0);

        if (rst) begin
            state     <= ST_IDLE;
            tick_cnt  <= 0;
            tick_pend <= 1'b0;
            cyc       <= 0;
            cyc_last  <= 0;
            cyc_max   <= 0;
            samples   <= 0;
            beat      <= 0;
            req_drops <= 0;
            pv_drops  <= 0;
            pj_valid  <= 1'b0;
            em_req    <= 1'b0;
            dl_we     <= 1'b0;
            mcnt      <= 0;
            stat_addr <= 0;
            g_master  <= 16'd4096;
            g_beatinc <= 0;
            pf_wp     <= 0;
            pf_rp     <= 0;
            pf_cnt    <= 0;
            f_cnt     <= 0;
            cs        <= 0;
            for (i = 0; i < NG * 2; i = i + 1) begin
                vld[i]  <= 1'b0;
                seqr[i] <= 2'd0;
            end
            for (i = 0; i < N_SLOTS; i = i + 1) begin
                r_ctrl[i]     <= 4'd0;
                r_base[i]     <= 24'd0;
                r_frames[i]   <= 24'd128;
                r_pos[i]      <= 32'd0;
                r_phinc[i]    <= 32'h01000000;
                r_bginc[i]    <= 32'h01000000;
                r_pitch[i]    <= 32'h01000000;
                r_wininc[i]   <= 32'd2236962;
                r_overlap[i]  <= 32'h04000000;
                r_amptgt[i]   <= 17'd16384;
                r_prob[i]     <= 17'd65536;
                r_sep[i]      <= 16'd0;
                r_randpos[i]  <= 24'd0;
                r_randpit[i]  <= 16'd0;
                r_maxg[i]     <= 6'd32;
                r_envdur[i]   <= 24'd96000;
                r_repeats[i]  <= 7'd0;
                r_interval[i] <= 14'd4800;
                r_amp[i]      <= 16'd4096;
                r_algo[i]     <= 16'd32768;
                r_algosel[i]  <= 3'd0;
                r_out[i]      <= 2'b11;
                pv_act[i]     <= 1'b0;
                pv_alg[i]     <= 2'd0;
                pv_xfs[i]     <= 32'd0;
                pv_x[i]       <= 17'd0;
                pv_nc[i]      <= 32'd0;
                pend_stb[i]   <= 5'd0;
                s_phasor[i]   <= 56'd0;
                s_trigph[i]   <= 32'd0;
                s_hop[i]      <= 17'd65536;
                s_amp[i]      <= 25'h1000000;
                s_rng[i]      <= 32'h9E3779B9 + i;
                s_active[i]   <= 32'd0;
                s_gate[i]     <= 1'b0;
                s_envleft[i]  <= 24'd0;
                s_level[i]    <= 17'd0;
                s_stutph[i]   <= 14'd0;
                s_stutleft[i] <= 7'd0;
                s_poslat[i]   <= 56'd0;
                s_bgph[i]     <= 56'd0;
                s_gst[i]      <= 3'd0;
                s_out[i]      <= 24'd0;
                s_miss[i]     <= 16'd0;
            end
        end
    end

    // ------------------------------------------------------------------ RAM address muxes
    always @(*) begin
        gm_ra = {cs, gm_low[4:0]};
        // cache: first word in G1B, second in G2 (both from registered grain state)
        if (state == ST_G2)
            cm_ra = {cs, gidx, g_w1, g_o1};
        else
            cm_ra = {cs, gidx, gl0[0], gv0[5:0]};
        hr_a  = (state == ST_G2) ? ({1'b0, g_wp[31:22]} + 11'd1) : {1'b0, g_wp[31:22]};
        er_a  = (state == ST_E3) ? ({1'b0, e12[11:4]} + 9'd1) : {1'b0, e12[11:4]};
    end

endmodule

`default_nettype wire
