# -*- coding: utf-8 -*-

"""MultiAxis — synchronized multi-axis motion controller.

Uses the RP2040/RP2350 DMA_MULTI_CHAN_TRIGGER register to fire all DMA
channels in a single AHB bus write, guaranteeing hardware-simultaneous start
across all axes.
"""

import asyncio
import math
import machine

from .pulseGenerator import PulseGenerator

# RP2040/RP2350: single write triggers all DMA channels whose bits are set.
_DMA_MULTI_CHAN_TRIGGER = 0x50000430


class MultiAxis:
    """Synchronized multi-axis motion controller.

    Example usage::

        x_axis = Axis(stepper_x, hard_max_speed=200, hard_max_accel=500)
        y_axis = Axis(stepper_y, hard_max_speed=150, hard_max_accel=400)
        ma = MultiAxis([x_axis, y_axis])

        # Synchronised move: both axes finish their accel phase at the same time.
        ma.move({x_axis: 100, y_axis: 50})
        await ma.wait_done()
    """

    def __init__(self, axes):
        """axes: list of Axis objects (each wrapping a SmartStepper)."""
        self._axes = list(axes)

    def move(self, targets):
        """Synchronized multi-axis move.

        targets: dict {axis: target_position} or list of target positions
                 parallel to the axes list given at construction.

        Algorithm:
          1. For each axis compute its natural triangular accel time (or the
             time to reach hard_max_speed if the triangular peak would exceed
             it). Uses hard limits so no axis is over-driven.
          2. t_common = max of all per-axis accel times.
          3. Call axis.prepare_move(target, accel_time=t_common) for each axis.
          4. Fire all DMA channels simultaneously via DMA_MULTI_CHAN_TRIGGER.
        """
        if isinstance(targets, dict):
            axis_targets = [(ax, targets[ax]) for ax in self._axes if ax in targets]
        else:
            axis_targets = list(zip(self._axes, targets))

        # 1. Compute per-axis accel time
        accel_times = []
        for ax, tgt in axis_targets:
            distance = abs(tgt - ax.position)
            accel_times.append(self._compute_accel_time(ax, distance))

        # 2. Common accel time = slowest axis
        t_common = max(accel_times) if accel_times else 0.0

        # 3. Prepare each axis (programs DMA without triggering)
        bitmask = 0
        for ax, tgt in axis_targets:
            ch = ax.prepare_move(tgt, accel_time=t_common)
            bitmask |= (1 << ch)

        # 4. Simultaneous hardware start
        if bitmask:
            machine.mem32[_DMA_MULTI_CHAN_TRIGGER] = bitmask

    def _compute_accel_time(self, axis, distance):
        """Return the acceleration phase duration for this axis over this distance.

        Computes the natural triangular peak speed (starting from minSpeed):
            peak = sqrt(distance * accel + minSpeed²)

        If peak <= hard_max_speed: the move is triangular; accel time is the
            time to ramp from minSpeed to peak.
        If peak > hard_max_speed: the move is trapezoidal at hard_max_speed;
            accel time is the time to ramp from minSpeed to hard_max_speed.
        """
        stepper = axis.stepper
        accel = stepper.acceleration
        min_speed = stepper.minSpeed
        hard_max = axis.hard_max_speed

        if distance < 1e-9:
            return 0.0

        natural_peak = math.sqrt(distance * accel + min_speed ** 2)

        if natural_peak <= hard_max:
            return (natural_peak - min_speed) / accel
        else:
            return (hard_max - min_speed) / accel

    async def wait_done(self):
        """Async wait until all axes finish moving."""
        while any(ax.moving for ax in self._axes):
            await asyncio.sleep_ms(1)

    def stop(self, emergency=False):
        """Stop all moving axes."""
        for ax in self._axes:
            if ax.moving:
                ax.stop(emergency)
