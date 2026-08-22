// ORACLE — CHIP 001
// policy_engine v2 (CR-001): frozen int8 MLP, 5 u8 features -> 16 ReLU ->
// 3 sat8 scores -> argmax (ties to lowest index). Semantics normative from
// reference/oracle_model.py. Cycle-exact: 147 cycles per inference.
//
// AREA REWRITE (CR-001, 2026-08-22): v1 stored weights in randomly-indexed
// arrays; variable-index reads synthesized to ~4.5k mux cells (43% of chip
// combinational logic) and blew the 1x2 tile budget (GPL-0301, 304% util).
// v2 stores weights in a 147-byte ROTATING RING whose order equals the
// inference traversal order; w_cur is always the head byte (wring[0]).
// 147 consumptions per inference = exactly one full revolution, so the
// ring self-restores between inferences. Same flop count, near-zero read
// muxing. Hidden activations use the same trick (16-byte ring; 48 OUT
// reads = 3 revolutions -> self-restoring).
//
// Weight stream order == load order (CR-001; supersedes v1 map). Biases
// are interleaved immediately after their row because the FSM consumes
// them in that order:
//   0..95   : for h = 0..15  -> w_in[h][0..4], b_h[h]     (6 bytes/unit)
//   96..146 : for o = 0..2   -> w_out[o][0..15], b_out[o] (17 bytes/output)
// Loads must stream ascending 0..146 (all program loaders do). load_addr
// is retained for pin/interface compatibility but functionally unused.
// A new LOAD session (rising edge of load_en) resets the stream counter,
// enabling live weight swaps without global reset.

`default_nettype none

module policy_engine (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,
    input  wire        load_en,
    input  wire [7:0]  load_data,
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [7:0]  load_addr,   // compat only; unused (CR-001)
    input  wire        load_we,
    input  wire [7:0]  f0, f1, f2, f3, f4,
    output reg  [1:0]  action,
    output reg         done,
    output reg  [7:0]  cycles_cnt
);
    /* verilator lint_on UNUSEDSIGNAL */

    localparam [1:0] IDLE = 2'd0, HIDDEN = 2'd1, OUT = 2'd2, FINISH = 2'd3;

    // ---- FSM state (declared early: consuming depends on st) ----
    reg [1:0] st;
    /* verilator lint_off UNUSEDSIGNAL */
    reg [1:0] st_q;     // debug alias of state (sim observability)
    /* verilator lint_on UNUSEDSIGNAL */
    reg [3:0] h_idx;     // 0..15
    reg [2:0] k_idx;     // 0..5, 5 = bias cycle
    reg [1:0] o_idx;     // 0..2
    reg [4:0] oh_idx;    // 0..16, 16 = bias cycle
    reg signed [31:0] acc;

    wire consuming = (st == HIDDEN) || (st == OUT);

    // ---- weight ring: head = wring[0] ----
    reg [7:0] wring [0:146];
    /* verilator lint_off UNUSEDSIGNAL */
    reg [7:0] wring0;   // debug alias of head byte (sim observability)
    /* verilator lint_on UNUSEDSIGNAL */
    reg [7:0] ld_cnt;    // bytes accepted in current LOAD session
    reg       ld_prev;   // load_en edge detect for session restart

    integer iw;          // loop var: wring block ONLY
    always @(posedge clk) begin
        if (rst) begin
            ld_cnt  <= 8'd0;
            ld_prev <= 1'b0;
            for (iw = 0; iw < 147; iw = iw + 1) wring[iw] <= 8'd0;
        end else begin
            ld_prev <= load_en;
            if (load_en && !ld_prev) begin
                ld_cnt <= 8'd0;              // fresh LOAD session
            end else if (load_en && load_we && ld_cnt < 8'd147) begin
                wring[ld_cnt[7:0]] <= load_data;   // static index (unrolled)
                ld_cnt <= ld_cnt + 8'd1;
            end else if (consuming) begin
                // rotate one step toward head; 147 steps = full revolution
                for (iw = 0; iw < 146; iw = iw + 1)
                    wring[iw] <= wring[iw+1];
                wring[146] <= wring[0];
            end
        end
    end
    always @(posedge clk) wring0 <= wring[0];
    always @(posedge clk) st_q <= st;

    // ---- hidden activation ring: head = hring[0] ----
    reg [7:0] hring [0:15];
    reg [3:0] hw_ptr;    // write position during HIDDEN; 16 writes/inference

    integer ih;          // loop var: hring block ONLY
    always @(posedge clk) begin
        if (rst) begin
            hw_ptr <= 4'd0;
            for (ih = 0; ih < 16; ih = ih + 1) hring[ih] <= 8'd0;
        end else if (st == HIDDEN && k_idx == 3'd5) begin
            hring[hw_ptr] <= relu_q;     // static index (unrolled)
            hw_ptr <= hw_ptr + 4'd1;     // wraps to 0 after 16 units
        end else if (st == OUT && oh_idx != 5'd16) begin
            // rotate ONLY on MAC cycles: 48 rotations per inference,
            // 48 mod 16 == 0 -> ring self-restores. Rotating on bias
            // cycles too would misalign by 3 per inference (found by
            // golden-vector run, CR-001).
            for (ih = 0; ih < 15; ih = ih + 1)
                hring[ih] <= hring[ih+1];
            hring[15] <= hring[0];
        end
    end

    // ---- datapath heads (zero mux cost) ----
    wire signed [7:0] w_head = $signed(wring[0]);
    wire signed [7:0] h_head = $signed(hring[0]);

    // ---- combinational operand select ----
    reg signed [7:0] w_cur;
    reg signed [7:0] f_cur;
    always @* begin
        w_cur = 8'sd0;
        f_cur = 8'sd0;
        if (st == HIDDEN) begin
            w_cur = w_head;
            case (k_idx)
                3'd0: f_cur = $signed(f0);
                3'd1: f_cur = $signed(f1);
                3'd2: f_cur = $signed(f2);
                3'd3: f_cur = $signed(f3);
                default: f_cur = $signed(f4);
            endcase
            if (k_idx == 3'd5) begin
                w_cur = w_head;          // bias byte from stream
                f_cur = 8'sd1;           // x1 => true bias add (model-exact)
            end
        end else if (st == OUT) begin
            w_cur = w_head;
            f_cur = (oh_idx == 5'd16) ? 8'sd1 : h_head;
        end
    end

    // s8 * s8 -> s16 product, accumulated into s32
    wire signed [15:0] mac     = w_cur * f_cur;
    wire signed [31:0] mac_ext = {{16{mac[15]}}, mac};
    wire signed [31:0] acc_nxt = acc + mac_ext;

    // ReLU then int8 saturation (model rev C)
    wire signed [31:0] relu_v   = (acc_nxt > 0) ? acc_nxt : 32'sd0;
    wire signed [7:0]  relu_q   = sat8(relu_v);

    function automatic signed [7:0] sat8(input signed [31:0] x);
        if (x > 32'sd127)       sat8 = 8'sd127;
        else if (x < -32'sd128) sat8 = -8'sd128;
        else                    sat8 = x[7:0];
    endfunction

    reg signed [7:0] scores [0:2];

    integer if_;         // loop var: FSM block ONLY
    always @(posedge clk) begin
        if (rst) begin
            st <= IDLE;
            action <= 2'd1;
            done <= 1'b0;
            cycles_cnt <= 8'd0;
            acc <= 32'sd0;
            h_idx <= 4'd0; k_idx <= 3'd0; o_idx <= 2'd0; oh_idx <= 5'd0;
            for (if_ = 0; if_ < 3; if_ = if_ + 1) scores[if_] <= 8'sd0;
        end else begin
            done <= 1'b0;
            case (st)
                IDLE: if (start) begin
                    st <= HIDDEN;
                    h_idx <= 4'd0; k_idx <= 3'd0;
                    acc <= 32'sd0;
                    cycles_cnt <= 8'd0;
                end

                HIDDEN: begin
                    cycles_cnt <= cycles_cnt + 8'd1;
                    if (k_idx == 3'd5) begin
                        acc <= 32'sd0;
                        if (h_idx == 4'd15) begin
                            st <= OUT;
                            o_idx <= 2'd0; oh_idx <= 5'd0;
                        end else begin
                            h_idx <= h_idx + 4'd1;
                            k_idx <= 3'd0;
                        end
                    end else begin
                        acc <= acc_nxt;
                        k_idx <= k_idx + 3'd1;
                    end
                end

                OUT: begin
                    cycles_cnt <= cycles_cnt + 8'd1;
                    if (oh_idx == 5'd16) begin
                        scores[o_idx] <= sat8(acc_nxt);
                        acc <= 32'sd0;
                        if (o_idx == 2'd2) begin
                            st <= FINISH;
                        end else begin
                            o_idx <= o_idx + 2'd1;
                            oh_idx <= 5'd0;
                        end
                    end else begin
                        acc <= acc_nxt;
                        oh_idx <= oh_idx + 5'd1;
                    end
                end

                FINISH: begin
                    if (scores[0] >= scores[1] && scores[0] >= scores[2])
                        action <= 2'd0;
                    else if (scores[1] >= scores[2])
                        action <= 2'd1;
                    else
                        action <= 2'd2;
                    done <= 1'b1;
                    st <= IDLE;
                end

                default: st <= IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
