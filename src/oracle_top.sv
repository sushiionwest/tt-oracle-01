// ORACLE — CHIP 001
// oracle_top: full-chip integration. VGA timing + pong game core + frozen
// policy engine. Action pipeline latency is ONE FRAME and is normative:
//   at frame_tick(k): pong_core applies action_reg (= policy(state[k-1]))
//                     and produces state[k];
//   one pixel clock later the top latches features from state[k] and starts
//   an inference whose result lands in action_reg well before tick(k+1).
// (The reference-model closed-loop emitter implements identical semantics.)
//
// Weight loading: hold rst with mode_load=1, stream bytes to load_* ports.
// Buttons are consumed raw here; platform debounce lands with pin map.

`default_nettype none

module oracle_top (
    input  wire        clk,          // pixel clock
    input  wire        rst,
    input  wire        mode_load,    // sample with rst=1 to enter LOAD mode
    input  wire        btn_up,
    input  wire        btn_down,
    input  wire        btn_serve,
    input  wire        load_we,
    input  wire [7:0]  load_addr,
    input  wire [7:0]  load_data,
    output wire [9:0]  hcount,
    output wire [9:0]  vcount,
    output wire        hsync,
    output wire        vsync,
    output wire        active,
    output wire [2:0]  rgb,          // PRELIMINARY 3-bit digital color
    output wire [7:0]  dbg_cycles,   // cycles of last policy inference
    output wire        dbg_done,
    output wire [1:0]  dbg_action,
    output wire [3:0]  dbg_score_l,
    output wire [3:0]  dbg_score_r,
    output wire [2:0]  dbg_state,
    output wire [9:0]  dbg_pad_l,
    output wire [9:0]  dbg_pad_r,
    output wire [15:0] dbg_bx,
    output wire [15:0] dbg_by
);

    // ---- VGA ----
    wire [9:0] hc, vc;
    vga_timing u_vga (
        .clk_pix(clk), .rst, .hcount(hc), .vcount(vc),
        .hsync, .vsync, .active
    );
    assign hcount = hc;
    assign vcount = vc;

    // one-cycle pulse at each frame start
    wire frame_tick = (hc == 10'd0) && (vc == 10'd0);

    // ---- pong core ----
    // bx/by fractional bits [5:0] and vx/vy magnitudes are intentionally
    // unobserved at this level: the policy consumes quantized positions and
    // velocity signs only, video consumes integer positions only.
    /* verilator lint_off UNUSEDSIGNAL */
    wire [15:0] bx, by;
    wire signed [7:0] vx, vy;
    /* verilator lint_on UNUSEDSIGNAL */
    wire [9:0] pad_l_y, pad_r_y;
    wire [3:0] score_l, score_r;
    wire [2:0] game_state;
    wire [1:0] action_reg;

    pong_core u_game (
        .clk, .rst,
        .frame_tick,
        .btn_up, .btn_down, .btn_serve,
        .action(action_reg),
        .bx, .by, .vx, .vy,
        .pad_l_y, .pad_r_y,
        .score_l, .score_r,
        .state(game_state)
    );

    // ---- feature latch: one pixel clock after frame_tick ----
    reg tick_d;
    always @(posedge clk) begin
        if (rst) tick_d <= 1'b0;
        else     tick_d <= frame_tick;
    end
    wire latch_feat = tick_d & ~frame_tick;   // single-cycle pulse

    reg signed [7:0] ff0, ff1, ff2, ff3, ff4;
    always @(posedge clk) begin
        if (rst) begin
            ff0 <= 8'sd0; ff1 <= 8'sd0; ff2 <= 8'sd0;
            ff3 <= 8'sd0; ff4 <= 8'sd0;
        end else if (latch_feat) begin
            ff0 <= {3'b000, bx[15:11]};           // (bx>>6)>>5 = bx>>11
            ff1 <= {3'b000, by[15:11]};           // (by>>6)>>5 = by>>11
            ff2 <= {7'd0, ~vx[7]};                // sign flag: 1 = positive
            ff3 <= {7'd0, ~vy[7]};
            ff4 <= {3'b000, pad_r_y[9:5]};        // pixels >> 5
        end
    end

    // ---- policy engine ----
    // LOAD mode (mode_load=1) holds the GAME in reset but keeps the policy
    // engine OUT of reset so weight writes land (spec REQ-INTF-003).
    // CR-001: gate requests off during LOAD — a frame_tick arriving
    // mid-load would otherwise start a phantom inference that rotates the
    // weight ring and corrupts the stream being loaded.
    // CR-001 rev B: the v3 engine rotates unconditionally and therefore
    // accepts start only on aligned windows (~1 in 147 clocks); a raw
    // single-cycle latch_feat pulse would usually be missed and leave a
    // STALE action applied for the whole frame. Hold pe_req until done:
    // pickup lands within ~147 cycles of the aligned-window cadence plus
    // 147 inference cycles — far inside the 420,000-clock frame — so the
    // one-frame action pipeline is preserved. Mirrors the golden-vector
    // TB's hold-start-until-done protocol.
    reg pe_req;
    always @(posedge clk) begin
        if (rst | mode_load) pe_req <= 1'b0;
        else if (latch_feat) pe_req <= 1'b1;
        else if (dbg_done)   pe_req <= 1'b0;
    end
    wire pe_start = pe_req & ~mode_load;
    policy_engine u_policy (
        .clk,
        .rst(rst & ~mode_load),
        .start(pe_start),
        .load_en(mode_load),
        .load_data, .load_addr, .load_we,
        .f0(ff0), .f1(ff1), .f2(ff2), .f3(ff3), .f4(ff4),
        .action(action_reg),
        .done(dbg_done),
        .cycles_cnt(dbg_cycles)
    );

    assign dbg_action  = action_reg;

    // ---- video composition (PRELIMINARY solid blocks) ----
    localparam [9:0] PAD_L_X_V = 10'd16;
    localparam [9:0] PAD_R_X_V = 10'd624;

    assign dbg_score_l = score_l;
    assign dbg_score_r = score_r;
    assign dbg_state   = game_state;
    assign dbg_pad_l   = pad_l_y;
    assign dbg_pad_r   = pad_r_y;
    assign dbg_bx      = bx;
    assign dbg_by      = by;

    wire in_l = active && (hc >= PAD_L_X_V) && (hc < PAD_L_X_V + 10'd8) &&
                (vc >= pad_l_y) && (vc < pad_l_y + 10'd48);
    wire in_r = active && (hc >= PAD_R_X_V) && (hc < PAD_R_X_V + 10'd8) &&
                (vc >= pad_r_y) && (vc < pad_r_y + 10'd48);
    wire in_ball = active &&
                (hc >= bx[15:6]) && (hc < bx[15:6] + 10'd8) &&
                (vc >= by[15:6]) && (vc < by[15:6] + 10'd8);
    assign rgb = in_ball ? 3'b111 :
                 in_l    ? 3'b010 :
                 in_r    ? 3'b010 : 3'b000;

endmodule

`default_nettype wire
