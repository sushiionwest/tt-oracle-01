# ORACLE — CHIP 001

## What is it

A single-tile-pair (1x2) Tiny Tapeout design that plays Pong **against itself**
with no processor of any kind:

- `vga_timing` — 640x480@60Hz VGA sync generator
- `pong_core` — the game: ball physics, two paddles, scoring, serve logic
- `policy_engine` — a frozen int8 MLP (16→8→2) that watches the ball and
  steers both paddles. Weights are resident in flops.
- `tt_top` — pin adapter + serial weight loader (address+data shifted in over
  2 pins, 128 weights, one per 16 clock edges)

Zero CPUs. Zero software on chip. The neural network *is* the silicon.

## How to play

1. Wire VGA DAC to uo[0..4], buttons to ui[0..3].
2. Power on → game serves itself and plays forever.
3. Optional: hold MODE_LOAD, stream 128×16-bit weight words in on
   SERIAL_BIT with LOAD_WE strobing writes — swap the resident policy live.

## Clock

25 MHz pixel clock; game steps once per frame.

## Verification

Bit-exact golden vectors vs. Python reference model (`reference/`),
closed-loop full-chip sim (4000 frames), formal BMC on core safety
properties. See `reports/E1..E4`.
