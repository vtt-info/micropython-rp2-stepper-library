"""HIL on-Pico script: MultiAxis.move() with two steppers.

Parameters (chosen so axes have different natural accel times):
  Axis 1: 50 units * 96 steps/unit = 4800 steps
    natural peak ≈ 122.6 u/s → capped to maxSpeed=50; accel time = 0.150 s
  Axis 2: 5 units * 96 steps/unit = 480 steps
    natural peak ≈ 39.1 u/s < 50; accel time ≈ 0.114 s
  t_common = 0.150 s (axis 1 dominates); axis 2 forced_peak = 50 u/s

Expected output line: done x_steps=<n> y_steps=<m>
"""

import asyncio
from smartstepper import SmartStepper, Axis, MultiAxis
from test_config import STEP_PIN, DIR_PIN, ENABLE_PIN, STEP_PIN_2, DIR_PIN_2, ENABLE_PIN_2

s1 = SmartStepper(STEP_PIN, DIR_PIN, ENABLE_PIN)
s1.stepsPerUnit = 96
s1.minSpeed = 5
s1.maxSpeed = 50
s1.acceleration = 300

s2 = SmartStepper(STEP_PIN_2, DIR_PIN_2, ENABLE_PIN_2)
s2.stepsPerUnit = 96
s2.minSpeed = 5
s2.maxSpeed = 50
s2.acceleration = 300

x = Axis(s1, hard_max_speed=50, hard_max_accel=300)
y = Axis(s2, hard_max_speed=50, hard_max_accel=300)
ma = MultiAxis([x, y])


async def main():
    ma.move({x: 50, y: 5})
    await ma.wait_done()
    print("done x_steps={} y_steps={}".format(
        s1._pulseCounter.value, s2._pulseCounter.value))


asyncio.run(main())
