"""HIL on-Pico script: Arc.move() — quarter-circle CCW (G03).

Geometry:
  Start:  (0,  0)  — default position (no position setter needed)
  Center: (0, 10)  — offset i=0, j=10 from start
  End:    (10, 10)

The arc sweeps from angle -π/2 (pointing down from center) to 0
(pointing right from center) — a quarter turn counter-clockwise.
Both axes move monotonically from 0 to 10 units throughout the arc:

  X: 0 → 10  (always increasing, DIR=up throughout)
  Y: 0 → 10  (always increasing, DIR=up throughout)

Parameters:
  stepsPerUnit = 10  →  1 unit = 10 steps
  chord_tol    = 0.5 units  →  3 segments  (ceil(π/2 / 2·acos(0.95)))
  maxSpeed     = 20 u/s
  acceleration = 10 u/s²
  minSpeed     = 2  u/s

Because both axes move monotonically, final _pulseCounter.value equals
the total step count, which equals the net displacement: 100 steps each.

Expected output line:
  done x_pos=10.00 y_pos=10.00 x_steps=100 y_steps=100
(counts may differ by ±a few steps due to integer rounding in profiles)
"""

import asyncio
from smartstepper import SmartStepper, Axis
from smartstepper.arc import Arc
from test_config import STEP_PIN, DIR_PIN, ENABLE_PIN, STEP_PIN_2, DIR_PIN_2, ENABLE_PIN_2

s1 = SmartStepper(STEP_PIN, DIR_PIN, ENABLE_PIN)
s1.stepsPerUnit = 100
s1.minSpeed = 1
s1.maxSpeed = 200
s1.acceleration = 10

s2 = SmartStepper(STEP_PIN_2, DIR_PIN_2, ENABLE_PIN_2)
s2.stepsPerUnit = 100
s2.minSpeed = 1
s2.maxSpeed = 200
s2.acceleration = 10

x = Axis(s1, hard_max_speed=200, hard_max_accel=100)
y = Axis(s2, hard_max_speed=200, hard_max_accel=100)

arc = Arc(x, y)


async def main():
    # Quarter-circle CCW from (0,0) to (10,10).
    # Center offset i=0, j=10  →  center at (0, 10), radius = 10.
    await arc.move(100, 100, i=0, j=100, direction='ccw', chord_tol=0.15)
    print("done x_pos={:.2f} y_pos={:.2f} x_steps={} y_steps={}".format(
        s1.position, s2.position,
        s1._pulseCounter.value, s2._pulseCounter.value))


asyncio.run(main())
