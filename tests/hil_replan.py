"""HIL script: mid-move maxSpeed replan for test_hil.py.

Starts a long move at maxSpeed=50 (4800 steps/s), waits until cruising,
then reduces maxSpeed to 25 (2400 steps/s) to trigger _replan().
Prints the PulseCounter step total on completion.
"""

import time
from smartstepper import smartStepper
from test_config import STEP_PIN, DIR_PIN, ENABLE_PIN

s = smartStepper.SmartStepper(STEP_PIN, DIR_PIN, ENABLE_PIN)
s.stepsPerUnit = 96
s.minSpeed = 5
s.maxSpeed = 50      # 4800 steps/s
s.acceleration = 300

s.moveTo(100)
time.sleep_ms(500)  # wait until well into the fast cruise phase
s.maxSpeed = 25      # trigger _replan() — motor drops to 2400 steps/s
s.waitEndOfMove()
print(f'done steps={s._pulseCounter.value}')
