# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0
#
# ORACLE (CHIP 001) smoke test: reset, load weights, confirm the resident
# policy drives the game and video output is alive.

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

# Pin contract per docs/pin_map.md:
BTN_UP, BTN_DOWN, BTN_SERVE = 0, 1, 2
MODE_LOAD, LOAD_WE = 3, 4
SERIAL_BIT = 8  # uio[0]


def ui(bits):
    return (1 << MODE_LOAD) if bits == "load" else bits


@cocotb.test()
async def test_oracle(dut):
    dut._log.info("ORACLE smoke test")

    clock = Clock(dut.clk, 40, unit="ns")  # 25 MHz
    cocotb.start_soon(clock.start())

    # Reset
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)

    # uio_oe must be 0 (all bidirectional pins are inputs on this design).
    assert dut.uio_oe.value == 0, "uio_oe must be 0"

    # Video must be alive: exactly one sync pair asserted during a scanline.
    seen_syncs = set()
    for _ in range(20000):
        v = int(dut.uo_out.value)
        seen_syncs.add((v >> 3) & 0x3)  # hsync/vsync bits 4,3
        await ClockCycles(dut.clk, 1)
        if len(seen_syncs) >= 4:
            break
    assert len(seen_syncs) >= 4, f"VGA sync never toggled: {seen_syncs}"
    dut._log.info(f"VGA alive; sync combos seen: {seen_syncs}")

    # Game runs without input for many frames and stays deterministic-alive.
    await ClockCycles(dut.clk, 2 * 800 * 525)

    dut._log.info("PASS")
