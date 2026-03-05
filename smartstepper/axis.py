# -*- coding: utf-8 -*-

"""Axis — SmartStepper wrapper with hard speed/acceleration limits.

Used standalone or as a building block for MultiAxis synchronized moves.
"""

import asyncio
from .smartStepper import SmartStepperError


class AxisError(Exception):
    pass


class Axis:
    """Wraps a SmartStepper and enforces hard limits on speed and acceleration.

    hard_max_speed and hard_max_accel are immutable after construction. Any
    attempt to set the underlying stepper's maxSpeed above hard_max_speed (or
    acceleration above hard_max_accel) raises AxisError.

    prepare_move() sets up the motion profile and programs the DMA channel
    without starting it. Use start_move() for single-axis deferred start, or
    MultiAxis.move() to fire all axes simultaneously.
    """

    def __init__(self, stepper, hard_max_speed=None, hard_max_accel=None):
        self._stepper = stepper
        self._hard_max_speed = hard_max_speed if hard_max_speed is not None else stepper.maxSpeed
        self._hard_max_accel = hard_max_accel if hard_max_accel is not None else stepper.acceleration

    # ------------------------------------------------------------------ #
    # Read-only hard limits                                                #
    # ------------------------------------------------------------------ #

    @property
    def hard_max_speed(self):
        return self._hard_max_speed

    @property
    def hard_max_accel(self):
        return self._hard_max_accel

    # ------------------------------------------------------------------ #
    # Delegated stepper access                                             #
    # ------------------------------------------------------------------ #

    @property
    def stepper(self):
        return self._stepper

    @property
    def position(self):
        return self._stepper.position

    @position.setter
    def position(self, value):
        self._stepper.position = value

    @property
    def speed(self):
        return self._stepper.speed

    @property
    def moving(self):
        return self._stepper.moving

    @property
    def target(self):
        return self._stepper.target

    @property
    def direction(self):
        return self._stepper.direction

    @property
    def minSpeed(self):
        return self._stepper.minSpeed

    @minSpeed.setter
    def minSpeed(self, value):
        self._stepper.minSpeed = value

    @property
    def maxSpeed(self):
        return self._stepper.maxSpeed

    @maxSpeed.setter
    def maxSpeed(self, value):
        if value > self._hard_max_speed:
            raise AxisError(
                f"maxSpeed {value} exceeds hard limit {self._hard_max_speed}"
            )
        self._stepper.maxSpeed = value

    @property
    def acceleration(self):
        return self._stepper.acceleration

    @acceleration.setter
    def acceleration(self, value):
        if value > self._hard_max_accel:
            raise AxisError(
                f"acceleration {value} exceeds hard limit {self._hard_max_accel}"
            )
        self._stepper.acceleration = value

    @property
    def stepsPerUnit(self):
        return self._stepper.stepsPerUnit

    @stepsPerUnit.setter
    def stepsPerUnit(self, value):
        self._stepper.stepsPerUnit = value

    @property
    def reverse(self):
        return self._stepper.reverse

    @reverse.setter
    def reverse(self, value):
        self._stepper.reverse = value

    # ------------------------------------------------------------------ #
    # Motion                                                               #
    # ------------------------------------------------------------------ #

    def prepare_move(self, target, relative=False, accel_time=None, triangular=False):
        """Set up motion profile and configure DMA without starting it.

        Enforces hard limits: raises AxisError if the stepper's current
        maxSpeed or acceleration exceeds the hard limits set at construction.

        Returns the DMA channel number for multi-axis bitmask construction.
        """
        if self._stepper.maxSpeed > self._hard_max_speed:
            raise AxisError(
                f"stepper maxSpeed {self._stepper.maxSpeed} exceeds hard limit "
                f"{self._hard_max_speed}"
            )
        if self._stepper.acceleration > self._hard_max_accel:
            raise AxisError(
                f"stepper acceleration {self._stepper.acceleration} exceeds hard limit "
                f"{self._hard_max_accel}"
            )
        return self._stepper._prepare_move(
            target, relative=relative, accel_time=accel_time, triangular=triangular
        )

    def start_move(self):
        """Start a previously prepared move (single-axis deferred start)."""
        self._stepper._pulseGenerator._triggerDMA()

    def moveTo(self, target, relative=False, timeout=None, triangular=False, accel_time=None):
        """Move to target immediately (convenience wrapper around SmartStepper.moveTo)."""
        if self._stepper.maxSpeed > self._hard_max_speed:
            raise AxisError(
                f"stepper maxSpeed {self._stepper.maxSpeed} exceeds hard limit "
                f"{self._hard_max_speed}"
            )
        if self._stepper.acceleration > self._hard_max_accel:
            raise AxisError(
                f"stepper acceleration {self._stepper.acceleration} exceeds hard limit "
                f"{self._hard_max_accel}"
            )
        self._stepper.moveTo(
            target, relative=relative, timeout=timeout,
            triangular=triangular, accel_time=accel_time,
        )

    def stop(self, emergency=False):
        self._stepper.stop(emergency)

    def enable(self):
        self._stepper.enable()

    def disable(self):
        self._stepper.disable()

    async def wait_done(self):
        """Async wait until this axis finishes moving."""
        while self._stepper.moving:
            await asyncio.sleep_ms(1)
