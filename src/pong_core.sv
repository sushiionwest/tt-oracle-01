// ORACLE — CHIP 001
// pong_core: game rules per docs/chip_specification.md REQ-FUNC-001.
// Must match reference/game_model.py frame-for-frame (vplan class 3).
// Positions in 1/64 px (FRAC=6), 16-bit registers. Single clock domain,
// sync reset. Update order mirrors the model exactly:
// paddles -> ball integrate -> wall clamp -> paddle collision -> score.
// All constants explicitly sized; -Wall clean by construction.

`default_nettype none

module pong_core (
    input  wire        clk,
    input  wire        rst,
    input  wire        frame_tick,   // one-cycle pulse per rendered frame
    input  wire        btn_up,
    input  wire        btn_down,
    input  wire        btn_serve,
    input  wire [1:0]  action,       // 0=DOWN 1=HOLD 2=UP
    output reg  [15:0] bx, by,       // ball position, 1/64 px
    output reg  signed [7:0] vx, vy, // velocity, 1/64 px/frame
    output reg  [9:0]  pad_l_y,
    output reg  [9:0]  pad_r_y,
    output reg  [3:0]  score_l,
    output reg  [3:0]  score_r,
    output reg  [2:0]  state
);

    localparam [2:0] ATTRACT=3'd0, SERVE_WAIT=3'd1, RALLY=3'd2,
                     POINT_PAUSE=3'd3, WIN=3'd4;

    // Positions are px << 6 (1/64 px fixed point); shifts use literal 6
    // so the constant itself stays sized and used only in comments.

    // 1/64-px fixed-point constants (px value << 6)
    localparam [15:0] BX16    = 16'd20480;  // 320 px
    localparam [15:0] BY16    = 16'd15360;  // 240 px
    localparam [15:0] PLX16   = 16'd1024;   // 16 px
    localparam [15:0] PRX16   = 16'd39936;  // 624 px
    localparam [15:0] BXMAX16 = 16'd40896;  // 639 px
    localparam [15:0] BYMAX16 = 16'd30656;  // 479 px

    // 12-bit signed geometry for comparisons
    localparam signed [11:0] W12      = 12'sd640;
    localparam signed [11:0] H12      = 12'sd480;
    localparam signed [11:0] PLX12    = 12'sd16;
    localparam signed [11:0] PRX12    = 12'sd624;
    localparam signed [11:0] PADH12   = 12'sd48;

    localparam [9:0]  PAD_Y_MAX        = 10'd432;
    localparam [9:0]  PAD_CENTER_INIT  = 10'd216;
    localparam [9:0]  PAD_SPD          = 10'd4;
    localparam [3:0]  WIN_SCORE        = 4'd11;
    localparam [5:0]  WAIT_FRAMES      = 6'd32;
    localparam signed [7:0] SPD_1_0    = 8'sd64;
    localparam signed [7:0] SPD_1_25   = 8'sd80;

    reg [5:0] wait_cnt;
    reg       last_scorer;

    function automatic signed [7:0] zone_vy(input signed [11:0] off);
        if (off < -12'sd14)      zone_vy = -SPD_1_25;
        else if (off <= -12'sd5) zone_vy = -SPD_1_0;
        else if (off <= 12'sd4)  zone_vy = 8'sd0;
        else if (off <= 12'sd14) zone_vy = SPD_1_0;
        else                     zone_vy = SPD_1_25;
    endfunction

    // Signed next positions (velocity sign-extended to 18 bits)
    wire signed [17:0] vxs18 = {{10{vx[7]}}, vx};
    wire signed [17:0] vys18 = {{10{vy[7]}}, vy};
    wire signed [17:0] next_bx = $signed({1'b0, bx}) + vxs18;
    wire signed [17:0] next_by = $signed({1'b0, by}) + vys18;
    wire signed [11:0] nx_i    = next_bx[17:6];   // integer part after move
    wire signed [11:0] ny_i    = next_by[17:6];

    // Wall-clamped integer y before collision checks (mirrors model order)
    wire        y_under = (ny_i < 0);
    wire        y_over  = (ny_i >= H12);
    wire signed [11:0] ny_c_i = y_under ? 12'sd0 :
                          y_over  ? 12'sd479 : $signed(next_by[17:6]);

    wire signed [11:0] pad_l_c = $signed({2'b00, pad_l_y});
    wire signed [11:0] pad_r_c = $signed({2'b00, pad_r_y});
    wire signed [11:0] off_l   = ny_c_i - pad_l_c - 12'sd24;
    wire signed [11:0] off_r   = ny_c_i - pad_r_c - 12'sd24;

    // Next paddle positions (spec order: paddles move BEFORE collision check)
    wire signed [11:0] pad_l_nxt_c = $signed({2'b00, pad_l_y})
        + ((btn_up && !btn_down) ? -12'sd4
           : (btn_down && !btn_up) ? 12'sd4 : 12'sd0);
    wire signed [11:0] pad_r_nxt_c = $signed({2'b00, pad_r_y})
        + ((action == 2'd2) ? -12'sd4
           : (action == 2'd0) ? 12'sd4 : 12'sd0);
    // clamp to [0, PAD_Y_MAX]
    wire signed [11:0] pad_l_next = (pad_l_nxt_c < 0) ? 12'sd0 :
        (pad_l_nxt_c > $signed({2'b00, PAD_Y_MAX})) ?
            $signed({2'b00, PAD_Y_MAX}) : pad_l_nxt_c;
    wire signed [11:0] pad_r_next = (pad_r_nxt_c < 0) ? 12'sd0 :
        (pad_r_nxt_c > $signed({2'b00, PAD_Y_MAX})) ?
            $signed({2'b00, PAD_Y_MAX}) : pad_r_nxt_c;

    wire left_hit  = (vx < 0) && (nx_i <= PLX12) &&
                     (pad_l_next <= ny_c_i) && (ny_c_i < pad_l_next + PADH12);
    wire right_hit = (vx > 0) && (nx_i >= PRX12) &&
                     (pad_r_next <= ny_c_i) && (ny_c_i < pad_r_next + PADH12);

    // ---- paddles ----
    always @(posedge clk) begin
        if (rst) begin
            pad_l_y <= PAD_CENTER_INIT;
            pad_r_y <= PAD_CENTER_INIT;
        end else if (frame_tick && (state == RALLY || state == POINT_PAUSE)) begin
            if (btn_up && !btn_down && pad_l_y >= PAD_SPD)
                pad_l_y <= pad_l_y - PAD_SPD;
            else if (btn_down && !btn_up && pad_l_y <= PAD_Y_MAX - PAD_SPD)
                pad_l_y <= pad_l_y + PAD_SPD;
            if (action == 2'd2 && pad_r_y >= PAD_SPD)
                pad_r_y <= pad_r_y - PAD_SPD;
            else if (action == 2'd0 && pad_r_y <= PAD_Y_MAX - PAD_SPD)
                pad_r_y <= pad_r_y + PAD_SPD;
        end
    end

    // ---- ball + FSM ----
    always @(posedge clk) begin
        if (rst) begin
            bx <= BX16;       by <= BY16;
            vx <= SPD_1_0;    vy <= SPD_1_0;
            score_l <= 4'd0;  score_r <= 4'd0;
            state <= ATTRACT;
            wait_cnt <= 6'd0;
            last_scorer <= 1'b1;
        end else if (frame_tick) begin
            case (state)
                WIN: if (btn_serve) begin
                    score_l <= 4'd0;
                    score_r <= 4'd0;
                    state <= ATTRACT;
                    bx <= BX16;   by <= BY16;
                    vx <= SPD_1_0; vy <= SPD_1_0;
                end

                ATTRACT: begin
                    if (btn_serve) begin
                        state <= SERVE_WAIT;
                        wait_cnt <= WAIT_FRAMES;
                    end
                    if (nx_i >= W12) begin
                        vx <= -SPD_1_0;
                        bx <= BXMAX16;
                    end else if (nx_i < 0) begin
                        vx <= SPD_1_0;
                        bx <= 16'd0;
                    end else begin
                        bx <= next_bx[15:0];
                    end
                    if (ny_i >= H12) begin
                        vy <= -SPD_1_0;
                        by <= BYMAX16;
                    end else if (ny_i < 0) begin
                        vy <= SPD_1_0;
                        by <= 16'd0;
                    end else begin
                        by <= next_by[15:0];
                    end
                end

                SERVE_WAIT: begin
                    wait_cnt <= wait_cnt - 6'd1;
                    if (wait_cnt == 6'd1) begin
                        // documented: attract serve goes left
                        bx <= BX16;    by <= BY16;
                        vx <= -SPD_1_0; vy <= SPD_1_0;
                        state <= RALLY;
                    end
                end

                POINT_PAUSE: begin
                    wait_cnt <= wait_cnt - 6'd1;
                    if (wait_cnt == 6'd1) begin
                        bx <= BX16;   by <= BY16;
                        // serve toward the conceder
                        vx <= last_scorer ? -SPD_1_0 : SPD_1_0;
                        vy <= SPD_1_0;
                        state <= RALLY;
                    end
                end

                default: begin // RALLY
                    if (y_under) begin
                        by <= 16'd0;
                        vy <= -vy;              // vy<0 here, so -vy > 0
                    end else if (y_over) begin
                        by <= BYMAX16;
                        vy <= -vy;              // vy>0 here, so -vy < 0
                    end else begin
                        by <= next_by[15:0];
                    end

                    if (left_hit) begin
                        vx <= SPD_1_25;
                        bx <= PLX16;
                        vy <= zone_vy(off_l);
                    end else if (right_hit) begin
                        vx <= -SPD_1_25;
                        bx <= PRX16;
                        vy <= zone_vy(off_r);
                    end else begin
                        bx <= next_bx[15:0];
                        if (nx_i < 0) begin
                            score_r <= score_r + 4'd1;
                            last_scorer <= 1'b1;
                            if (score_r + 4'd1 >= WIN_SCORE) begin
                                state <= WIN;
                            end else begin
                                state <= POINT_PAUSE;
                                wait_cnt <= WAIT_FRAMES;
                            end
                        end else if (nx_i >= W12) begin
                            score_l <= score_l + 4'd1;
                            last_scorer <= 1'b0;
                            if (score_l + 4'd1 >= WIN_SCORE) begin
                                state <= WIN;
                            end else begin
                                state <= POINT_PAUSE;
                                wait_cnt <= WAIT_FRAMES;
                            end
                        end
                    end
                end
            endcase
        end
    end

endmodule

`default_nettype wire
