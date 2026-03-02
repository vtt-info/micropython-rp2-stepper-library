# -*- coding: utf-8 -*-

""" Async homing for SmartStepper.

Provides a three-phase homing routine:

  0. Initial backoff (if needed) — if the sensor is already asserted at the
     start, jog away from it at slowSpeed until it de-asserts, then stop.

  1. Fast approach — jog toward the sensor at fastSpeed. When the sensor
     asserts, decelerate smoothly to a stop.

  2. Slow backoff — jog away from the sensor at slowSpeed. The instant the
     sensor de-asserts, stop immediately (emergency stop) and define that
     location as position 0.

"""

import time
import asyncio

class HomingError(Exception):
    """ Raised when homing fails to complete (e.g. timeout). """
    pass


async def home(stepper, sensor,
               fastSpeed=30, slowSpeed=2,
               direction='down', activeState=1,
               timeout=None):
    """ Home a stepper motor using a two-phase approach.

    Args:
        stepper:     SmartStepper instance.
        sensor:      Any object with a value() method (e.g. machine.Pin).
        fastSpeed:   Approach speed in units/s.
        slowSpeed:   Backoff speed in units/s.
        direction:   Direction of travel toward the home sensor ('up' or 'down').
        activeState: sensor.value() when the sensor is triggered
                     (1 = active-high / normally-open,
                      0 = active-low  / normally-closed).
        timeout:     Seconds allowed for the entire homing sequence.
                     None means no limit. Raises HomingError on expiry.
    """
    reverseDir = 'up' if direction == 'down' else 'down'

    savedMinSpeed = stepper.minSpeed
    savedMaxSpeed = stepper.maxSpeed

    def triggered():
        return sensor.value() == activeState

    deadline = (None if timeout is None
                else time.ticks_add(time.ticks_ms(), round(timeout * 1000)))

    def timed_out():
        return (deadline is not None
                and time.ticks_diff(time.ticks_ms(), deadline) >= 0)

    def abort(msg):
        stepper.stop(emergency=True)
        stepper.minSpeed = savedMinSpeed
        stepper.maxSpeed = savedMaxSpeed
        raise HomingError(msg)

    # Adjust minSpeed now so it applies to all slow-speed phases below.
    if slowSpeed < stepper.minSpeed:
        stepper.minSpeed = slowSpeed

    # Phase 0: if sensor is already asserted, back off until it clears
    if triggered():
        stepper.maxSpeed = slowSpeed
        stepper.jog(direction=reverseDir)
        while triggered():
            if timed_out():
                abort("Homing timed out during initial backoff")
            await asyncio.sleep_ms(1)
        stepper.stop()
        while stepper.moving:
            await asyncio.sleep_ms(1)

    # Phase 1: fast approach until sensor asserts
    stepper.maxSpeed = fastSpeed
    stepper.jog(direction=direction)
    while not triggered():
        if timed_out():
            abort("Homing timed out during approach")
        await asyncio.sleep_ms(1)
    stepper.stop()
    while stepper.moving:
        await asyncio.sleep_ms(1)

    # Phase 2: slow backoff until sensor de-asserts
    stepper.maxSpeed = slowSpeed
    stepper.jog(direction=reverseDir)
    while triggered():
        if timed_out():
            abort("Homing timed out during backoff")
        await asyncio.sleep_ms(1)
    stepper.stop(emergency=True)   # cut immediately at de-assert edge

    stepper.minSpeed = savedMinSpeed
    stepper.maxSpeed = savedMaxSpeed
    stepper.position = 0
