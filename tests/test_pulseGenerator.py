# -*- coding: utf-8 -*-

""" PulseGenerator manual test / demo.

Run on an RP2040 board (Raspberry Pi Pico).
Adjust pin number and points to match your hardware.
"""

import time
import machine
from machine import mem32
from smartstepper import pulseGenerator
from test_config import STEP_PIN

FSTAT_ADDR = 0x50200000 + 0x04  # PIO0.FSTAT (PIO0 base 0x50200000 + FSTAT offset 0x04)
FDEBUG_ADDR = 0x50200000 + 0x08 
SM0_INSTR_ADDR = 0x502000d4  # PIO0.SM0 current instruction address (PIO0 base 0x50200000 + SM0 offset 0xd4)

def print_debug(first_address=0):
    fstat = mem32[FSTAT_ADDR]
    inst_addr = mem32[SM0_INSTR_ADDR] - first_address
    return f"ADDR: {inst_addr:02d} FSTAT: {fstat:08x}"

def main():
    """
    """
    step_pin = machine.Pin(STEP_PIN, machine.Pin.OUT, value=0)
    pg = pulseGenerator.PulseGenerator(step_pin)
    points = ((1, 3), (5, 5))

    print(f"Generator number: {pg._smNum}")
    first_address = mem32[SM0_INSTR_ADDR]
    print(f"mem32[SM0_INSTR_ADDR]: {first_address:02d}")

    pg.start(points)

    t = time.ticks_ms()
    print("msec freq TX FIFO")
    while pg.freq > 0:
        elapsed = time.ticks_ms() - t
        print(f"{elapsed:4d} {step_pin.value()} {pg.freq:04d} {pg._sm.tx_fifo():02d} {pg._sm.rx_fifo():02d} {print_debug(first_address)}")

        if elapsed > 9991200:
            pg.stop()
            break

        time.sleep(0.1)

    print(f"{elapsed:4d} {step_pin.value()} {pg.freq:04d} {pg._sm.tx_fifo():02d} {pg._sm.rx_fifo():02d} {print_debug(first_address)}")
    print(f"DONE total time: {elapsed/1000:.3f} s")


if __name__ == "__main__":
    main()
