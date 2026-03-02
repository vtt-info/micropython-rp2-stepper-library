"""HIL on-Pico script: moveTo(50) with known parameters.

Expected: 4800 step pulses (50 units * 96 steps/unit).
Prints "done steps=<n>" on completion; host checks this via mpremote stdout.

Deploy with: mpremote cp hil_moveto.py :
Run via:     mpremote run hil_moveto.py
"""

import time
from smartstepper import smartStepper
from test_config import STEP_PIN, DIR_PIN, ENABLE_PIN

s = smartStepper.SmartStepper(STEP_PIN, DIR_PIN, ENABLE_PIN)
s.stepsPerUnit = 96
s.minSpeed = 5
s.maxSpeed = 50
s.acceleration = 300

s.moveTo(50)
while s.moving:
    time.sleep_ms(10)

print("done steps={}".format(s._pulseCounter.value))
