// ORACLE — CHIP 001
// policy_engine v3 (CR-001 rev B): frozen int8 MLP, 5 u8 features ->
// 16 ReLU -> 3 sat8 scores -> argmax (ties lowest). Normative semantics:
// reference/oracle_model.py. Cycle-exact: 147 cycles per inference.
//
// AREA REWRITE HISTORY (CR-001):
//  v1: randomly-indexed weight arrays -> ~4.5k mux cells -> GPL-0301 fail
//      (304% utilization on a 1x2 tile).
//  v2: rotating ring, rotate-only-during-inference. Muxes fell to ~3.1k
//      but still 281% — "rotate vs hold" is itself one mux per flop
//      (~1176x), plus the ld_cnt write decode.
//  v3 (THIS): UNCONDITIONAL rotation. The ring turns every clock except
//      inside an open LOAD session. Therefore:
//        - zero per-flop hold muxes for rotation
//        - LOAD writes land at ring[0] (static index). Each accepted byte
//          rotates once, so after 147 WE-cycles the stream sits aligned
//          and the phase is self-restored (147 mod 147 == 0)
//        - inference consumes exactly 147 rotations and self-restores too
//      Only variable-index write left: hring[hw_ptr] (16-entry decode).
//
// Weight stream order (normative, CR-001): biases interleaved after their
// row; see reference/oracle_model.py infer() docstring:
//   idx 0..95  : h=0..15 -> w_in[h][0..4], b_h[h]
//   idx 96..146: o=0..2  -> w_out[o][0..15], b_out[o]
// Loads must stream ascending 0..146 with load_we asserted per byte.

`default_nettype none

module policy_engine (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,
    input  wire        load_en,
    input  wire [7:0]  load_data,
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [7:0]  load_addr,   // compat only; unused since CR-001
    /* verilator lint_on UNUSEDSIGNAL */
    input  wire        load_we,
    input  wire [7:0]  f0, f1, f2, f3, f4,
    output reg  [1:0]  action,
    output reg         done,
    output reg  [7:0]  cycles_cnt
);

    localparam [1:0] IDLE = 2'd0, HIDDEN = 2'd1, OUT = 2'd2, FINISH = 2'd3;

    reg [1:0] st;
    reg [3:0] h_idx;     // 0..15
    reg [2:0] k_idx;     // 0..5, 5 = bias cycle
    reg [1:0] o_idx;     // 0..2
    reg [4:0] oh_idx;    // 0..16, 16 = bias cycle
    reg signed [31:0] acc;
    /* verilator lint_off UNUSEDSIGNAL */
    wire consuming = (st == HIDDEN) || (st == OUT);  // v2 leftover, unused
    /* verilator lint_on UNUSEDSIGNAL */

    // ---- weight ring ----
    // v3b: rotation is unconditional (zero per-flop hold muxes) EXCEPT
    // during open LOAD sessions and the session-open/close edges.
    // ALIGNMENT MATH (validated in mirror before RTL): per WE-cycle the
    // ring rotates once and the incoming byte lands in slot0, so an
    // ascending 147-byte load leaves the ring as
    //     [s146, s0, s1, ..., s145]
    // The FSM rotates on its ACCEPT edge too (rotation is unconditional
    // outside LOAD), so alignment is declared at align_cnt==0: the accept
    // rotation lands s0 at the head on the FIRST MAC cycle. The counter
    // free-runs mod 147 (same period as the ring), so alignment recurs
    // once per revolution and back-to-back inferences re-arm. Worst wait:
    // 146 clocks — trivial vs the one-frame action pipeline.
    reg [7:0] wring [0:146];
    reg [7:0] ld_cnt;
    reg       ld_prev;
    reg [7:0] align_cnt;   // rotations since session close, mod 147
    wire      aligned = (align_cnt == 8'd0);

    integer iw;          // loop var: wring block ONLY
    always @(posedge clk) begin
        if (rst) begin
            ld_cnt    <= 8'd0;
            ld_prev   <= 1'b0;
            align_cnt <= 8'd0;
            for (iw = 0; iw < 147; iw = iw + 1) wring[iw] <= 8'd0;
        end else begin
            ld_prev <= load_en;          // unconditional edge tracking
            if (load_en != ld_prev) begin
                // session-open OR session-close edge: hold ring this cycle.
                if (!load_en) begin
                    ld_cnt    <= 8'd0;   // closed: arm next session
                    align_cnt <= 8'd0;   // phase count starts at close
                end
            end else begin
                if (!load_en || load_we) begin
                    // rotate once per clock outside LOAD, and on each
                    // accepted LOAD byte (WE-cycles fill consecutive slots)
                    for (iw = 0; iw < 146; iw = iw + 1)
                        wring[iw] <= wring[iw+1];
                    wring[146] <= wring[0];
                    if (!load_en) begin
                        // free-runs mod 147 so aligned (cnt==1) recurs once
                        // per ring revolution -> back-to-back inferences arm
                        align_cnt <= (align_cnt == 8'd146) ? 8'd0
                                                           : align_cnt + 8'd1;
                    end
                end
                if (load_en && load_we && ld_cnt < 8'd147) begin
                    // static-index write; after the rotation so the byte
                    // lands at slot 0 of this cycle
                    wring[0] <= load_data;
                    ld_cnt   <= ld_cnt + 8'd1;
                end
            end
        end
    end

    // ---- hidden activation ring ----
    // Writes during HIDDEN bias cycles at hw_ptr; rotates on every OUT
    // MAC cycle (48 rotations = 3 full revolutions -> self-restores).
    // Rotation holds during IDLE/HIDDEN/FINISH to keep write timing simple.
    reg [7:0] hring [0:15];
    reg [3:0] hw_ptr;

    integer ih;          // loop var: hring block ONLY
    always @(posedge clk) begin
        if (rst) begin
            hw_ptr <= 4'd0;
            for (ih = 0; ih < 16; ih = ih + 1) hring[ih] <= 8'd0;
        end else if (st == HIDDEN && k_idx == 3'd5) begin
            hring[hw_ptr] <= relu_q;     // static-ish index (16-entry decode)
            hw_ptr <= hw_ptr + 4'd1;
        end else if (st == OUT && oh_idx != 5'd16) begin
            for (ih = 0; ih < 15; ih = ih + 1)
                hring[ih] <= hring[ih+1];
            hring[15] <= hring[0];
        end
    end

    // ---- datapath heads (zero read-mux cost) ----
    wire signed [7:0] w_head = $signed(wring[0]);
    wire signed [7:0] h_head = $signed(hring[0]);

    // ---- combinational operand select ----
    reg signed [7:0] w_cur;
    reg signed [7:0] f_cur;
    always @* begin
        w_cur = 8'sd0;
        f_cur = 8'sd0;
        if (st == HIDDEN) begin
            case (k_idx)
                3'd0: f_cur = $signed(f0);
                3'd1: f_cur = $signed(f1);
                3'd2: f_cur = $signed(f2);
                3'd3: f_cur = $signed(f3);
                default: f_cur = $signed(f4);
            endcase
            if (k_idx == 3'd5) begin
                f_cur = 8'sd1;           // x1 => true bias add (model-exact)
            end
        end else if (st == OUT) begin
            f_cur = (oh_idx == 5'd16) ? 8'sd1 : h_head;
        end
        w_cur = w_head;
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
                IDLE: if (start && aligned) begin
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
