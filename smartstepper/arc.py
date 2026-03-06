# -*- coding: utf-8 -*-

"""Arc — 2-axis circular arc motion (G02/G03 compatible).

Uses chord linearization: the arc is broken into straight-line chord segments
each satisfying a maximum chord-error tolerance.  Every segment is executed as
a MultiAxis.move(), giving hardware-simultaneous start and finish per segment.

G-code conventions:
  G02 → direction='cw'   (clockwise)
  G03 → direction='ccw'  (counter-clockwise)
  I, J → center offset from current position (same as G-code I/J)
"""

import math
import asyncio
from .multiaxis import MultiAxis


class ArcError(Exception):
    pass


class Arc:
    """2-axis circular arc motion controller.

    Example usage::

        x_axis = Axis(stepper_x, hard_max_speed=20, hard_max_accel=10)
        y_axis = Axis(stepper_y, hard_max_speed=20, hard_max_accel=10)
        arc = Arc(x_axis, y_axis)

        # Quarter-circle CCW (G03) from current position:
        # current=(10,0), center offset i=-10,j=0 => center=(0,0), end=(0,10)
        await arc.move(0, 10, i=-10, j=0, direction='ccw', chord_tol=0.1)
    """

    DEFAULT_CHORD_TOL = 0.1  # user units

    def __init__(self, x_axis, y_axis):
        """
        x_axis: Axis controlling the X dimension.
        y_axis: Axis controlling the Y dimension.
        """
        self._x = x_axis
        self._y = y_axis
        self._multi = MultiAxis([x_axis, y_axis])

    # ------------------------------------------------------------------ #
    # Geometry                                                             #
    # ------------------------------------------------------------------ #

    def chord_segments(self, x_end, y_end, i=0.0, j=0.0,
                       direction='ccw', chord_tol=None):
        """Compute chord endpoint waypoints for a circular arc.

        Reads the current axis positions as the arc start point.

        Parameters:
            x_end, y_end  Arc endpoint (user units).
            i, j          Center offset from start position (G-code I, J).
            direction     'ccw' (G03, default) or 'cw' (G02).
            chord_tol     Maximum chord-to-arc deviation (user units).
                          Defaults to Arc.DEFAULT_CHORD_TOL.

        Returns:
            List of (x, y) waypoints.  The last entry is always (x_end, y_end)
            to prevent floating-point drift from accumulating.

        Raises ArcError if the radius is zero or chord_tol is impractically
        small relative to the radius.
        """
        if chord_tol is None:
            chord_tol = self.DEFAULT_CHORD_TOL

        x_start = self._x.position
        y_start = self._y.position
        cx = x_start + i
        cy = y_start + j

        r = math.sqrt((x_start - cx) ** 2 + (y_start - cy) ** 2)
        if r < 1e-9:
            raise ArcError("Arc radius is zero")

        # Angular step from chord tolerance:
        #   chord_error = r * (1 - cos(dtheta/2)) <= chord_tol
        #   => dtheta = 2 * acos(1 - chord_tol/r)
        ratio = chord_tol / r
        if ratio >= 1.0:
            ratio = 1.0  # single segment covers any chord_tol >= diameter
        dtheta = 2.0 * math.acos(1.0 - ratio)
        if dtheta < 1e-9:
            raise ArcError("chord_tol too small relative to radius")

        start_angle = math.atan2(y_start - cy, x_start - cx)
        end_angle = math.atan2(y_end - cy, x_end - cx)

        if direction == 'ccw':
            # Sweep increases (counter-clockwise)
            if end_angle < start_angle:
                end_angle += 2.0 * math.pi
            total_angle = end_angle - start_angle
        else:
            # direction == 'cw': sweep decreases (clockwise)
            if end_angle > start_angle:
                end_angle -= 2.0 * math.pi
            total_angle = start_angle - end_angle  # positive value

        # Full-circle: start and end at the same angle
        if total_angle < 1e-9:
            total_angle = 2.0 * math.pi

        n_segments = max(1, math.ceil(total_angle / dtheta))

        waypoints = []
        for k in range(n_segments):
            frac = (k + 1) / n_segments
            if direction == 'ccw':
                theta = start_angle + frac * total_angle
            else:
                theta = start_angle - frac * total_angle

            if k == n_segments - 1:
                # Snap to exact endpoint to prevent float accumulation
                waypoints.append((x_end, y_end))
            else:
                waypoints.append((
                    cx + r * math.cos(theta),
                    cy + r * math.sin(theta),
                ))

        return waypoints

    # ------------------------------------------------------------------ #
    # Motion                                                               #
    # ------------------------------------------------------------------ #

    async def move(self, x_end, y_end, i=0.0, j=0.0,
                   direction='ccw', chord_tol=None, segment_min_speed=None):
        """Execute a circular arc move from the current position.

        Linearizes the arc into chord segments via chord_segments(), then
        executes each segment as a synchronized MultiAxis.move(), waiting
        for completion before starting the next.

        Parameters:
            x_end, y_end        Arc endpoint (user units).
            i, j                Center offset from current position (G-code I, J).
            direction           'ccw' (G03, default) or 'cw' (G02).
            chord_tol           Maximum chord deviation (user units).
            segment_min_speed   minSpeed used during arc segments (user units/s).
                                Defaults to ¼ of the axes' configured minSpeed.
                                Near-axis chords (e.g. the first chord of a
                                quarter-circle) can have extreme X:Y distance
                                ratios; reducing minSpeed lets MultiAxis slow the
                                minor axis enough to finish simultaneously with
                                the dominant axis.
        """
        waypoints = self.chord_segments(x_end, y_end, i, j, direction, chord_tol)

        orig_min_x = self._x.minSpeed
        orig_min_y = self._y.minSpeed
        if segment_min_speed is None:
            segment_min_speed = min(orig_min_x, orig_min_y) / 4.0
        self._x.minSpeed = segment_min_speed
        self._y.minSpeed = segment_min_speed
        try:
            for x_t, y_t in waypoints:
                self._multi.move({self._x: x_t, self._y: y_t})
                await self._multi.wait_done()
        finally:
            self._x.minSpeed = orig_min_x
            self._y.minSpeed = orig_min_y

    async def wait_done(self):
        """Async wait until the current segment completes."""
        await self._multi.wait_done()

    def stop(self, emergency=False):
        """Stop all axes."""
        self._multi.stop(emergency=emergency)
