// ORACLE — CHIP 001
// policy_engine: frozen int8 MLP, 5 u8 features -> 16 ReLU -> 3 sat8 scores
// -> argmax (ties to lowest index). Semantics normative from
// reference/oracle_model.py. Cycle-exact: 147 cycles per inference
// (16*(1+5) hidden + 3*(1+16) output), single MAC lane.
//
// Weight bytes: 147 flops total, loaded byte-by-address in LOAD mode.
// Address map: 0..79 w_in[h][k]=h*5+k · 80..95 b_h[h] ·
//              96..143 w_out[o][h]=o*16+h · 144..146 b_out[o].

`default_nettype none

module policy_engine (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,
    input  wire        load_en,
    input  wire [7:0]  load_data,
    input  wire [7:0]  load_addr,
    input  wire        load_we,
    input  wire [7:0]  f0, f1, f2, f3, f4,
    output reg  [1:0]  action,
    output reg         done,
    output reg  [7:0]  cycles_cnt
);

    localparam [1:0] IDLE = 2'd0, HIDDEN = 2'd1, OUT = 2'd2, FINISH = 2'd3;

    // ---- weight storage (2D arrays, small typed indices) ----
    reg signed [7:0] w_in  [0:15][0:4];
    reg signed [7:0] b_h   [0:15];
    reg signed [7:0] w_out [0:2][0:15];
    reg signed [7:0] b_out [0:2];

    // ---- load address decode (div/mod via multiply-subtract) ----
    wire        ld_go    = load_en && load_we;
    wire [7:0]  ld_q5    = load_addr / 8'd5;      // 0..15
    /* verilator lint_off UNUSEDSIGNAL */
    wire [7:0]  ld_r5    = load_addr - ld_q5 * 8'd5;  // exact: addr<=79
    /* verilator lint_on UNUSEDSIGNAL */
    wire [7:0]  ld_wo16  = load_addr - 8'd96;
    wire [7:0]  ld_q16   = ld_wo16 / 8'd16;       // 0..2
    /* verilator lint_off UNUSEDSIGNAL */
    wire [7:0]  ld_r16   = ld_wo16 - ld_q16 * 8'd16;
    /* verilator lint_on UNUSEDSIGNAL */

    wire [3:0]  ld_wi_h  = ld_q5[3:0];
    wire [2:0]  ld_wi_k  = ld_r5[2:0];            // 0..4 by construction
    wire [1:0]  ld_wo_o  = ld_q16[1:0];
    wire [3:0]  ld_wo_h  = ld_r16[3:0];           // 0..15 by construction

    integer iw;  // loop var: weight-load block ONLY (never FSM block)
    integer if_; // loop var: FSM reset block ONLY (never weight-load block)
    always @(posedge clk) begin
        if (rst) begin
            for (iw = 0; iw < 16; iw = iw + 1) begin
                w_in[iw][0] <= 8'sd0; w_in[iw][1] <= 8'sd0;
                w_in[iw][2] <= 8'sd0; w_in[iw][3] <= 8'sd0;
                w_in[iw][4] <= 8'sd0;
                b_h[iw] <= 8'sd0;
            end
            for (iw = 0; iw < 3; iw = iw + 1) begin
                b_out[iw] <= 8'sd0;
            end
        end else if (ld_go) begin
            if (load_addr < 8'd80)
                w_in[ld_wi_h][ld_wi_k] <= $signed(load_data);
            else if (load_addr < 8'd96)
                b_h[load_addr[3:0]] <= $signed(load_data);
            else if (load_addr < 8'd144)
                w_out[ld_wo_o][ld_wo_h] <= $signed(load_data);
            else if (load_addr < 8'd147)
                b_out[load_addr[1:0]] <= $signed(load_data);
        end
    end
    // ---- FSM state ----
    reg [1:0] st;
    reg [3:0] h_idx;     // 0..15
    reg [2:0] k_idx;     // 0..5, 5 = bias cycle
    reg [1:0] o_idx;     // 0..2
    reg [4:0] oh_idx;    // 0..16, 16 = bias cycle
    reg signed [31:0] acc;
    reg signed [7:0] hidden [0:15];
    reg signed [7:0] scores [0:2];

    // ---- combinational decode of current weight/feature ----
    reg signed [7:0] w_cur;
    reg signed [7:0] f_cur;

    always @* begin
        w_cur = 8'sd0;
        f_cur = 8'sd0;
        if (st == HIDDEN) begin
            if (k_idx == 3'd5) begin
                w_cur = b_h[h_idx];
            end else begin
                // k_idx is 0..4 here (5 = bias handled above)
                case (k_idx)
                    3'd0: w_cur = w_in[h_idx][0];
                    3'd1: w_cur = w_in[h_idx][1];
                    3'd2: w_cur = w_in[h_idx][2];
                    3'd3: w_cur = w_in[h_idx][3];
                    default: w_cur = w_in[h_idx][4];
                endcase
                case (k_idx)
                    3'd0: f_cur = $signed(f0);
                    3'd1: f_cur = $signed(f1);
                    3'd2: f_cur = $signed(f2);
                    3'd3: f_cur = $signed(f3);
                    default: f_cur = $signed(f4);
                endcase
            end
        end else if (st == OUT) begin
            if (oh_idx == 5'd16) begin
                w_cur = b_out[o_idx];
            end else begin
                w_cur = w_out[o_idx][oh_idx[3:0]];
                f_cur = hidden[oh_idx[3:0]];
            end
        end
    end

    // s8 * s8 -> s16 product, accumulated into s32
    wire signed [15:0] mac     = w_cur * f_cur;
    wire signed [31:0] mac_ext = {{16{mac[15]}}, mac};
    wire signed [31:0] acc_nxt = acc + mac_ext;

    function automatic signed [7:0] sat8(input signed [31:0] x);
        if (x > 32'sd127)       sat8 = 8'sd127;
        else if (x < -32'sd128) sat8 = -8'sd128;
        else                    sat8 = x[7:0];
    endfunction

    always @(posedge clk) begin
        if (rst) begin
            st <= IDLE;
            action <= 2'd1;
            done <= 1'b0;
            cycles_cnt <= 8'd0;
            acc <= 32'sd0;
            h_idx <= 4'd0; k_idx <= 3'd0; o_idx <= 2'd0; oh_idx <= 5'd0;
            for (if_ = 0; if_ < 16; if_ = if_ + 1) hidden[if_] <= 8'sd0;
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
                        hidden[h_idx] <= (acc_nxt > 0) ? sat8(acc_nxt)
                                                       : 8'sd0;
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
