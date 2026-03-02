# -*- coding: utf-8 -*-

""" SmartStepper manual tests / demo.

Run on an RP2040 board (Raspberry Pi Pico).
Adjust pin numbers in test_config.py to match your hardware.
"""

import time
from smartstepper import smartStepper
from test_config import STEP_PIN, DIR_PIN, ENABLE_PIN


def jog(stepper):
    """
    """
    print("\nINFO::Testing 'jog'...")

    stepper.position = 0
    stepper.jog(50)
    t0 = time.ticks_ms()

    t = time.ticks_ms()
    while (time.ticks_ms() - t < 1500):
#         print("time={:6.3f}s speed={:+6.1f}mm/s pos={:+7.1f}mm".format((time.ticks_ms()-t0)/1000, stepper.speed, stepper.position))
        time.sleep(0.1)

    stepper.stop()

    while stepper.moving:
        time.sleep(0.1)

#     print("\n  time speed")
#     for t, freq in stepper._pulseGenerator.data:
#         print("{:6.3f} {:5.1f}".format(t/1000, freq / stepper.stepsPerUnit))


def moveTo(stepper):
    """
    """
    print("\nINFO::Testing 'moveto'...")

    stepper.moveTo(150)
    t0 = time.ticks_ms()

#     while stepper.moving:
    t = time.ticks_ms()
    while (time.ticks_ms() - t < 1500):
        time.sleep(0.1)

    stepper.stop()

    while stepper.moving:
        time.sleep(0.1)

    stepper.moveTo(50)

    while stepper.moving:
        time.sleep(0.1)

#     print("\n  time speed")
#     for t, freq in stepper._pulseGenerator.data:
#         print("{:6.3f} {:5.1f}".format(t/1000, freq / stepper.stepsPerUnit))


def moveToRel(stepper):
    """
    """
    print("\nINFO::Testing relative 'moveto'...")

    t = time.ticks_ms()
    stepper.position = 0
    stepper.moveTo(80, relative=True)

    while stepper.moving:
        time.sleep(0.1)

    stepper.moveTo(-50, relative=True)

    while stepper.moving:
        time.sleep(0.1)

#     print("\n  time speed")
#     for t, freq in stepper._pulseGenerator.data:
#         print("{:6.3f} {:5.1f}".format(t/1000, freq / stepper.stepsPerUnit))


def debug(stepper):
    """
    """
    t = time.ticks_ms()
    stepper.moveTo(50)
    print("\nINFO::starting with target: {:.1f}mm / {:d}steps".format(stepper.target, round(stepper.target*stepper.stepsPerUnit)))

    while not stepper.moving:
        time.sleep_ms(1)
    print("INFO::Moving...")

#     print("  time  speed     pos")
    while stepper.moving:
#         print("{:6.3f} {:+6.1f} {:+7.1f}".format((time.ticks_ms()-t)/1000, stepper.speed, stepper.position))
#
#         if time.ticks_ms()-t >= 5000:
#             stepper.stop()
#             while stepper.moving:
#                 print("{:6.3f} {:+6.1f} {:+7.1f}".format((time.ticks_ms()-t)/1000, stepper.speed, stepper.position))

        time.sleep_ms(1)

    print("\nINFO::done: pulse counter: {:.1f}mm / {:d}steps".format(stepper.position, stepper._pulseCounter.value))

#     print("\n  time speed")
#     for t, freq in stepper._pulseGenerator.data:
#         print("{:6.3f} {:5.1f}".format(t/1000, freq / stepper.stepsPerUnit))


def main():
    """
    """
    print("TRACE::main()")
    stepper = smartStepper.SmartStepper(STEP_PIN, DIR_PIN, ENABLE_PIN, accelCurve='smooth2')
    stepper.stepsPerUnit = 96.
    stepper.minSpeed = 1
    stepper.maxSpeed = 50
    stepper.acceleration = 300

    jog(stepper)
    moveTo(stepper)
    moveToRel(stepper)
    debug(stepper)


if __name__ == "__main__":
    main()
