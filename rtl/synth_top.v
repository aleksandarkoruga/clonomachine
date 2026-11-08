// Wavetable synth (prototype) beside the sampler engine, audio clock domain: the oscillator unit
// (rtl/synth_osc.v), the filter engine (rtl/synth_filt.v), the voice mixer and the 2x -> 1x
// half-band decimator. Per sample period (tick):
//   1. the oscillator unit's previous sums move into the filter engine (filter part) and a
//      buffer (the part that bypasses the filter)
//   2. both units start: the oscillators render this period's sub-samples, the filter engine
//      filters the ones just moved
//   3. the mixer takes the filter engine's previous results: per sub-sample
//      sum_v amp_v (sum_t w[v][t] y[t][v] + direct_v), two 96 kHz samples into the decimator,
//      which gives the output sample (mono, s24 like the engine's: full scale 2^23)
// Latency: three sample periods.
//
// Commands (cmd_addr): [12:11] 00 the oscillator unit (its waddr[9:0]), 01 the filter engine
// (its waddr[10:0], taken only while it's idle), 10 here: [3:0] = {v, t} slot weight w s18 Q1.16
// (v * 4 + t), 16 + v amp target s18 Q1.16 (the gain glides to it, ~0.7 ms).
`default_nettype none
module synth_top #(
    parameter NV        = 3,              // voices (2 or 3): oscillator pairs, filter lanes
    parameter FLOG      = 10,             // frame cache size log2 (the largest mip level played)
    parameter SINC_HEX  = "sy_sinc.hex",
    parameter FN_HEX    = "sy_fn.hex",
    parameter UCODE_HEX = "sy_ucode.hex",
    parameter FFN_HEX   = "sy_ffn.hex",
    parameter HB_HEX    = "sy_hb.hex"
) (
    input  wire               clk,
    input  wire               rst,
    input  wire               tick,
    input  wire               cmd_valid,
    output wire               cmd_ready,
    input  wire [12:0]        cmd_addr,
    input  wire [31:0]        cmd_data,
    output wire               req_valid,
    input  wire               req_ready,
    output wire [24:0]        req_addr,
    input  wire               fill_valid,
    input  wire [15:0]        fill_data,
    output reg  signed [23:0] out,
    output wire [31:0]        lag,
    output wire [10:0]        osc_cyc_max,
    output wire [10:0]        filt_cyc_max
);
    function signed [23:0] sat24;
        input signed [47:0] v;
        begin
            if (v > 48'sd8388607)       sat24 = 24'sd8388607;
            else if (v < -48'sd8388608) sat24 = -24'sd8388608;
            else                        sat24 = v[23:0];
        end
    endfunction
    function signed [31:0] sat32;
        input signed [63:0] v;
        begin
            if (v > 64'sd2147483647)       sat32 = 32'sh7FFFFFFF;
            else if (v < -64'sd2147483648) sat32 = 32'sh80000000;
            else                           sat32 = v[31:0];
        end
    endfunction

    // ------------------------------------------------------------------ commands
    wire filt_busy;
    wire to_osc  = cmd_addr[12:11] == 2'b00;
    wire to_filt = cmd_addr[12:11] == 2'b01;
    wire to_top  = cmd_addr[12:11] == 2'b10;
    assign cmd_ready = !(to_filt && filt_busy);
    wire cmd_fire = cmd_valid && cmd_ready;
    reg  signed [17:0] w   [0:15];   // {v, t}
    reg  signed [17:0] amt [0:3];
    reg  signed [17:0] amp [0:3];
    always @(posedge clk)
        if (cmd_fire && to_top) begin
            if (!cmd_addr[4]) w[cmd_addr[3:0]] <= cmd_data[17:0];
            else              amt[cmd_addr[1:0]] <= cmd_data[17:0];
        end

    // ------------------------------------------------------------------ units
    reg          osc_start, filt_start;
    wire         osc_busy;
    reg  [2:0]   osc_ra;
    wire signed [23:0] osc_f, osc_d;
    synth_osc #(.NV(NV), .FLOG(FLOG), .SINC_HEX(SINC_HEX), .FN_HEX(FN_HEX)) u_osc (
        .clk(clk), .rst(rst), .start(osc_start), .busy(osc_busy),
        .we(cmd_fire && to_osc), .waddr(cmd_addr[9:0]), .wdata(cmd_data),
        .req_valid(req_valid), .req_ready(req_ready), .req_addr(req_addr),
        .fill_valid(fill_valid), .fill_data(fill_data),
        .out_ra(osc_ra), .out_filt(osc_f), .out_direct(osc_d), .lag(lag), .cyc_max(osc_cyc_max)
    );
    reg          in_we;
    reg  [2:0]   in_a;
    reg  [31:0]  in_d;
    reg  [4:0]   y_ra;
    wire signed [31:0] y_q;
    synth_filt #(.NL(NV), .UCODE_HEX(UCODE_HEX), .FFN_HEX(FFN_HEX)) u_filt (
        .clk(clk), .rst(rst), .start(filt_start), .busy(filt_busy),
        .we(cmd_fire && to_filt), .waddr(cmd_addr[10:0]), .wdata(cmd_data),
        .in_we(in_we), .in_a(in_a), .in_d(in_d),
        .y_ra(y_ra), .y_q(y_q), .cyc_max(filt_cyc_max)
    );

    // ------------------------------------------------------------------ sequencer
    // the bypass sums: moved now (dirn), used by the mixer one period later with the filter
    // results of the same oscillator sums (dirp). Transfer: osc out_ra k at step k (k = 0..7,
    // {sub, voice}), its data two cycles later.
    reg  signed [23:0] dirn [0:7];
    reg  signed [23:0] dirp [0:7];
    reg  [3:0]   xs;
    reg          xfer;
    integer i;
    always @(posedge clk) begin
        if (rst) begin
            xfer <= 0;
            osc_start <= 0;
            filt_start <= 0;
            in_we <= 0;
            for (i = 0; i < 4; i = i + 1)
                amp[i] <= 0;
        end else begin
            osc_start <= 1'b0;
            filt_start <= 1'b0;
            in_we <= 1'b0;
            if (tick) begin
                xfer <= 1'b1;
                xs <= 0;
                for (i = 0; i < 8; i = i + 1)
                    dirp[i] <= dirn[i];
                for (i = 0; i < 4; i = i + 1)   // the voice gains glide to their targets
                    amp[i] <= amp[i] + ((amt[i] - amp[i]) >>> 5);
            end else if (xfer) begin
                xs <= xs + 1'b1;
                osc_ra <= xs[2:0];
                if (xs >= 2) begin
                    in_we <= 1'b1;
                    in_a  <= xs[2:0] - 3'd2;
                    in_d  <= {{8{osc_f[23]}}, osc_f} <<< 9;   // Q8.15 -> Q8.24
                    dirn[xs[2:0] - 3'd2] <= osc_d;
                end
                if (xs == 4'd9) begin
                    xfer <= 1'b0;
                    osc_start <= 1'b1;
                    filt_start <= 1'b1;
                end
            end
        end
    end

    // ------------------------------------------------------------------ mixer
    // counter mc = 0..NV*32-1: sub = mc / (NV*16), v = (mc % (NV*16)) / 16, step = mc % 16. Each operation in a
    // cycle of its own (stage 2 = the y read's data, stage 3 one later):
    //   steps 0..3  stage 2: y x w registered; stage 3: into the voice sum (step 0 starts it)
    //   step 4      stage 3: + the bypass
    //   step 6      stage 2: the voice sum saturated to 32 bits (step 4's add landed a cycle before)
    //   step 7      stage 2: x amp registered
    //   step 8      stage 2: into the sub-sample's sum (v = 0 starts it)
    //   step 9      stage 2: (the last voice) the sum to the decimator
    localparam [6:0] MV = NV * 16;
    reg  [6:0]   mc;
    reg          mrun;
    wire         msub = mc >= MV;
    wire [6:0]   mrem = msub ? mc - MV : mc;
    wire [1:0]   mv   = mrem[5:4];
    wire [3:0]   mstp = mrem[3:0];
    reg          m_v1, m_v2, m_v3;
    reg  [6:0]   mk1, mk2, mk3;     // {sub, v, step}
    reg  signed [47:0] mx_vs, mx_acc;
    reg  signed [49:0] yw_r, va_r;
    reg  signed [31:0] vs_r;
    reg  signed [31:0] hi [0:1];
    reg          dc_go;
    always @(posedge clk) begin
        if (rst) begin
            mrun <= 0;
            m_v1 <= 0;
            m_v2 <= 0;
            m_v3 <= 0;
            dc_go <= 0;
        end else begin
            dc_go <= 1'b0;
            if (xfer && xs == 4'd9) begin
                mrun <= 1'b1;
                mc <= 0;
            end else if (mrun) begin
                mc <= mc + 1'b1;
                if (mc == 2 * MV - 1)
                    mrun <= 1'b0;
            end
            y_ra <= {msub, mstp[1:0], mv};
            m_v1 <= mrun;
            mk1  <= {msub, mv, mstp};
            m_v2 <= m_v1;
            mk2  <= mk1;
            m_v3 <= m_v2;
            mk3  <= mk2;
            if (m_v2)
                case (mk2[3:0])
                4'd0, 4'd1, 4'd2, 4'd3: yw_r <= y_q * w[{mk2[5:4], mk2[1:0]}];
                4'd6: vs_r <= sat32(mx_vs);
                4'd7: va_r <= vs_r * amp[mk2[5:4]];
                4'd8: mx_acc <= ((mk2[5:4] == 0) ? 48'sd0 : mx_acc) + (va_r >>> 16);
                4'd9: if (mk2[5:4] == NV - 1) begin
                    hi[mk2[6]] <= sat32(mx_acc);
                    if (mk2[6])
                        dc_go <= 1'b1;
                end
                default: ;
                endcase
            if (m_v3)
                case (mk3[3:0])
                4'd0, 4'd1, 4'd2, 4'd3: mx_vs <= (mk3[3:0] == 0 ? 48'sd0 : mx_vs) + (yw_r >>> 16);
                4'd4: mx_vs <= mx_vs + ({{24{dirp[{mk3[6], mk3[5:4]}][23]}}, dirp[{mk3[6], mk3[5:4]}]} <<< 9);
                default: ;
                endcase
        end
    end

    // ------------------------------------------------------------------ 2x -> 1x half-band
    // 71 taps: the centre (0.5) and 18 symmetric pairs at +-(2k - 1); ring of the 96 kHz samples
    // (Q8.24), written one per cycle; output Q8.24 x Q1.17 >> 18 -> s24 at 2^23 full scale
    reg  [17:0]  hbc [0:18];
    initial $readmemh(HB_HEX, hbc);
    reg  signed [31:0] ring [0:127];
    reg  [6:0]   wp;
    reg  [1:0]   dw;               // writes left
    reg  [4:0]   dk;
    reg          drun, out_pending;
    reg  signed [31:0] ra_q, rb_q;
    reg  signed [32:0] pre;
    reg  signed [17:0] cq1, cq2;
    reg  signed [50:0] dacc, dprod;
    reg  [4:0]   dk1, dk2, dk3;
    reg          dv1, dv2, dv3;
    wire [6:0]   doff = (dk == 0) ? 7'd0 : {1'b0, dk, 1'b0} - 7'd1;
    always @(posedge clk) begin
        if (rst) begin
            wp <= 0;
            dw <= 0;
            drun <= 0;
            dv1 <= 0;
            dv2 <= 0;
            dv3 <= 0;
            out_pending <= 0;
        end else begin
            if (dc_go)
                dw <= 2'd2;
            else if (dw != 0) begin
                ring[wp] <= (dw == 2'd2) ? hi[0] : hi[1];
                wp <= wp + 1'b1;
                dw <= dw - 1'b1;
                if (dw == 2'd1) begin
                    drun <= 1'b1;
                    dk <= 0;
                end
            end else if (drun) begin
                dk <= dk + 1'b1;
                if (dk == 5'd18)
                    drun <= 1'b0;
            end
            // the centre is 35 samples back from the newest (wp - 1)
            ra_q <= ring[wp - 7'd36 - doff];
            rb_q <= ring[wp - 7'd36 + doff];
            cq1  <= hbc[dk];
            dv1  <= drun;
            dk1  <= dk;
            pre  <= (dk1 == 0) ? ra_q : ra_q + rb_q;
            cq2  <= cq1;
            dv2  <= dv1;
            dk2  <= dk1;
            dprod <= pre * cq2;            // the product registered, then accumulated
            dv3  <= dv2;
            dk3  <= dk2;
            if (dv3) begin
                dacc <= ((dk3 == 0) ? 51'sd0 : dacc) + dprod;
                if (dk3 == 5'd18)
                    out_pending <= 1'b1;
            end
            if (out_pending) begin
                out_pending <= 1'b0;
                out <= sat24(dacc >>> 18);
            end
        end
    end
endmodule
`default_nettype wire
