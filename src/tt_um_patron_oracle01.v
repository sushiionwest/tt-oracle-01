// ORACLE — CHIP 001
// Tiny Tapeout physical wrapper (tt_um_patron_oracle01).
// Instantiates tt_top; adds the platform 'ena' handling only.
// Pin contract is documented in docs/pin_map.md.

`default_nettype none

module tt_um_patron_oracle01 (
    input  wire [7:0] ui_in,    // dedicated inputs
    output wire [7:0] uo_out,   // dedicated outputs
    input  wire [7:0] uio_in,   // IOs: input path
    output wire [7:0] uio_out,  // IOs: output path
    output wire [7:0] uio_oe,   // IOs: enable path (0=input, 1=output)
    input  wire       ena,      // always 1 when powered
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

    tt_top inner (
        .ui_in,
        .uo_out,
        .uio_in,
        .uio_out,
        .uio_oe,
        .clk,
        .rst_n
    );

    // ena is a power-good indicator on this shuttle family: design runs
    // whenever powered. Recorded as unused per platform guidance.
    wire _unused = &{ena, 1'b0};

endmodule

`default_nettype wire
