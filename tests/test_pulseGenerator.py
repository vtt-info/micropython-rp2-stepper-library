# -*- coding: utf-8 -*-

""" PulseGenerator manual test / demo.

Run on an RP2040 board (Raspberry Pi Pico).
Adjust pin number and points to match your hardware.
"""

import time
import machine
from smartstepper import pulseGenerator
from test_config import STEP_PIN


def main():
    """
    """
    pg = pulseGenerator.PulseGenerator(machine.Pin(STEP_PIN, machine.Pin.OUT, value=0))
    points = ((1, 3), (5, 5))
    pg.start(points)

    t = time.ticks_ms()
    print("freq TX FIFO")
    while pg.freq:
        print("{:04d} {:02d}".format(pg.freq, pg._sm.tx_fifo()))

        if time.ticks_ms()-t > 9991200:
            pg.stop()
            break

        time.sleep(0.1)

    print("{:04d} {:02d}".format(pg.freq, pg._sm.tx_fifo()))
    print(f"DONE total time: {(time.ticks_ms()-t)/1000:.3f} s")


if __name__ == "__main__":
    main()
