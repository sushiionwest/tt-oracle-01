// ORACLE — CHIP 001
// vga_timing: industry-standard 640x480 @ ~60Hz VGA timing generator.
// PRELIMINARY target: 25.175 MHz pixel clock. Charcoal §10 rules applied:
// explicit widths, explicit reset, no latches, no tri-states.
//
// Horizontal: 800 px total (96 sync, 40 back porch, 640 visible, 64 front porch)
// Vertical:   525 lines (2 sync, 33 back porch, 480 visible, 10 front porch)
// Active-low sync outputs, per VESA GTF defaults used by common monitors.

`default_nettype none

module vga_timing (
    input  wire       clk_pix,     // pixel clock (PRELIMINARY 25.175 MHz)
    input  wire       rst,         // synchronous, active-high reset
    output reg  [9:0] hcount,      // 0..799 within line
    output reg  [9:0] vcount,      // 0..524 within frame
    output wire       hsync,       // active-low horizontal sync
    output wire       vsync,       // active-low vertical sync
    output wire       active       // high when pixel is in visible region
);

    localparam H_TOTAL = 10'd800;
    localparam H_SYNC_END = 10'd96;
    localparam H_BACK_END = 10'd136;
    localparam H_VISIBLE_END = 10'd776;

    localparam V_TOTAL = 10'd525;
    localparam V_SYNC_END = 10'd2;
    localparam V_BACK_END = 10'd35;
    localparam V_VISIBLE_END = 10'd515;

    // Horizontal counter
    always @(posedge clk_pix) begin
        if (rst) begin
            hcount <= 10'd0;
        end else if (hcount == H_TOTAL - 10'd1) begin
            hcount <= 10'd0;
        end else begin
            hcount <= hcount + 10'd1;
        end
    end

    // Vertical counter advances at end of each line
    always @(posedge clk_pix) begin
        if (rst) begin
            vcount <= 10'd0;
        end else if (hcount == H_TOTAL - 10'd1) begin
            if (vcount == V_TOTAL - 10'd1) begin
                vcount <= 10'd0;
            end else begin
                vcount <= vcount + 10'd1;
            end
        end
    end

    assign hsync  = ~(hcount < H_SYNC_END);
    assign vsync  = ~(vcount < V_SYNC_END);
    assign active = (hcount >= H_BACK_END && hcount < H_VISIBLE_END)
                 && (vcount >= V_BACK_END && vcount < V_VISIBLE_END);

endmodule

`default_nettype wire
