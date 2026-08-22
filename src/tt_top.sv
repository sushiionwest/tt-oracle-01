// ORACLE — CHIP 001
// tt_top: Tiny Tapeout wrapper. Maps oracle_top onto the platform's fixed
// pin budget: ui_in[7:0] inputs, uo_out[7:0] outputs, uio[7:0] bidir.
//
// Pin plan (frozen for spec gate):
//   ui_in[0] btn_up        ui_in[1] btn_down     ui_in[2] btn_serve
//   ui_in[3] mode_load
//   (ui_in[7:4] reserved, tie low)
//   uo_out[2:0] rgb        uo_out[3] vsync       uo_out[4] hsync
//   uo_out[5] dbg_state[0] uo_out[6] dbg_state[1] uo_out[7] dbg_state[2]
//   uio[7:0] load_data byte (bidir; driven by host during LOAD only)
//   load_addr is SERIALIZED through uio during LOAD mode to save pins:
//     each byte = 16 uio clocks: 8 addr bits then 8 data bits, MSB first,
//     strobed by load_we as a level (see tt_load_shift below).
//   VGA hcount/vcount/active/scores are observable via the demo board's
//   existing VGA connector path; on-chip debug ports are dropped from the
//   physical pinout (still present in sim-only builds).
//
// The load shifter costs ~20 flops and replaces what would have been 16
// parallel pins we do not have.

`default_nettype none

module tt_top (
    input  wire [7:0] ui_in,      // buttons + mode
    output wire [7:0] uo_out,     // video + game state
    input  wire [7:0] uio_in,     // load data/addr serial (input direction)
    output wire [7:0] uio_out,    // unused drive (tied 0)
    output wire [7:0] uio_oe,     // uio output enables (all 0: input only)
    input  wire clk,              // platform clock (RP2040-derived)
    input  wire rst_n             // platform reset, active LOW
);

    // ---- platform clocking: pixel clock = clk directly (PRELIMINARY:
    // final frequency selection lands with the VGA DAC decision) ----
    wire rst = ~rst_n;
    wire mode_load = ui_in[3];

    // ---- serial load channel ----
    // Protocol: while mode_load=1, every 16 clk edges consume one weight:
    //   first 8 edges shift addr MSB-first, next 8 shift data MSB-first,
    //   with load_we asserted (uio_in used as single-bit? No: uio carries
    //   the serial bit on uio_in[0], we_en on ui_in... see REV note).
    // REV NOTE (spec gate item): single-bit serial needs a we indicator;
    // we reserve ui_in[4] as LOAD_WE_LEVEL for this purpose.
    wire ser_bit  = uio_in[0];
    wire load_we  = ui_in[4];

    reg [3:0]  bit_cnt;
    /* verilator lint_off UNUSEDSIGNAL */
    reg [7:0]  sh_addr, sh_data;   // top shift bit consumed via concat
    /* verilator lint_on UNUSEDSIGNAL */
    reg [7:0]  ld_addr_q, ld_data_q;
    reg        ld_we_pulse;

    // reserved pins: tied low at the board connector
    /* verilator lint_off UNUSEDSIGNAL */
    wire unused_pins = &{1'b1, ui_in[7:5], uio_in[7:1], 1'b1};
    /* verilator lint_on UNUSEDSIGNAL */

    always @(posedge clk) begin
        if (rst || !mode_load) begin
            bit_cnt <= 4'd0;
            sh_addr <= 8'd0;
            sh_data <= 8'd0;
            ld_addr_q <= 8'd0;
            ld_data_q <= 8'd0;
            ld_we_pulse <= 1'b0;
        end else begin
            // MSB-first shift: bits 0..7 build addr, bits 8..15 build data
            sh_addr <= {sh_addr[6:0], (bit_cnt < 4'd8) ? ser_bit : 1'b0};
            sh_data <= {sh_data[6:0], (bit_cnt >= 4'd8) ? ser_bit : 1'b0};
            if (bit_cnt == 4'd7)
                ld_addr_q <= {sh_addr[6:0], ser_bit};
            if (bit_cnt == 4'd15) begin
                ld_data_q <= {sh_data[6:0], ser_bit};
                ld_we_pulse <= load_we;   // write fires next cycle
            end else begin
                ld_we_pulse <= 1'b0;
            end
            bit_cnt <= (bit_cnt == 4'd15) ? 4'd0 : bit_cnt + 4'd1;
        end
    end

    wire [7:0] wdata = ld_data_q;
    wire [7:0] waddr = ld_addr_q;

    // ---- chip ----
    // Debug outputs and sim-only nets are intentionally unconnected at the
    // physical boundary (documented waiver, spec gate item).
    /* verilator lint_off PINCONNECTEMPTY */
    wire hsync, vsync;
    wire [2:0] rgb;
    wire [2:0] game_state;

    oracle_top u_chip (
        .clk,
        .rst(rst),
        .mode_load(mode_load),
        .btn_up(ui_in[0]),
        .btn_down(ui_in[1]),
        .btn_serve(ui_in[2]),
        .load_we(ld_we_pulse),
        .load_addr(waddr),
        .load_data(wdata),
        .hcount(),
        .vcount(),
        .hsync(hsync),
        .vsync(vsync),
        .active(),
        .rgb,
        .dbg_cycles(),
        .dbg_done(),
        .dbg_action(),
        .dbg_score_l(),
        .dbg_score_r(),
        .dbg_state(game_state),
        .dbg_pad_l(),
        .dbg_pad_r(),
        .dbg_bx(),
        .dbg_by()
    );
    /* verilator lint_on PINCONNECTEMPTY */

    assign uo_out = {game_state, hsync, vsync, rgb};
    assign uio_out = 8'd0;
    assign uio_oe  = 8'd0;

endmodule

`default_nettype wire
