"""HIL script: PulseCounter accuracy through multiple mid-move speed replans.

Performs moveTo(100) at maxSpeed=50 (9600 steps total).  Three maxSpeed changes
are issued during the cruise phase to trigger _replan().  Because replans only
alter velocity -- not target position -- the counter must still reach close to
9600 at the end.

Prints:
  done steps=<n>    PulseCounter value after motion completes
  phases=<ms0>,<speed0>,<ms1>,<speed1>,...
    Comma-separated pairs of (elapsed_ms, cruise_speed_steps_per_sec) for each
    phase, used by the host to compute expected steps from actual timing.
"""

import time
from smartstepper import smartStepper
from test_config import STEP_PIN, DIR_PIN, ENABLE_PIN

s = smartStepper.SmartStepper(STEP_PIN, DIR_PIN, ENABLE_PIN)
s.stepsPerUnit = 96
s.minSpeed = 5
s.maxSpeed = 50      # 4800 steps/s
s.acceleration = 300

# Record (timestamp, cruise_speed_in_steps_per_sec) at each transition.
speeds = [50 * 96, 15 * 96, 40 * 96, 50 * 96]

s.moveTo(100)
t0 = time.ticks_ms()

time.sleep_ms(400)
t1 = time.ticks_ms()
s.maxSpeed = 15      # drop to 1440 steps/s; triggers _replan()

time.sleep_ms(400)
t2 = time.ticks_ms()
s.maxSpeed = 40      # raise to 3840 steps/s; triggers _replan()

time.sleep_ms(400)
t3 = time.ticks_ms()
s.maxSpeed = 50      # restore to 4800 steps/s; triggers _replan()

s.waitEndOfMove()
t4 = time.ticks_ms()

durations = [
    time.ticks_diff(t1, t0),
    time.ticks_diff(t2, t1),
    time.ticks_diff(t3, t2),
    time.ticks_diff(t4, t3),
]

print(f'done steps={s._pulseCounter.value}')
# Emit phase data: elapsed_ms,speed_sps pairs
parts = []
for d, sp in zip(durations, speeds):
    parts.append(f'{d},{sp}')
print(f'phases={";".join(parts)}')
