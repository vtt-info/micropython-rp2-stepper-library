# -*- coding: utf-8 -*-

"""PulseCounter manual test / demo.

Run on an RP2040 board (Raspberry Pi Pico).
Adjust pin number to match your hardware.
"""

import machine
from smartstepper import pulseCounter
from test_config import STEP_PIN


def main():
    """ """
    stepPin = machine.Pin(STEP_PIN, machine.Pin.OUT)
    stepPin.low()

    counter = pulseCounter.PulseCounter(stepPin)
    print(counter.value)

    counter.value = 1000
    for i in range(10):
        stepPin.high()
        stepPin.low()
    print(counter.value)

    counter.direction = "down"
    for i in range(100):
        stepPin.high()
        stepPin.low()
    print(counter.value)


if __name__ == "__main__":
    main()
