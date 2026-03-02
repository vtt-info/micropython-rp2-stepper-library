# smartStepper — MicroPython stepper motor library for RP2040

A MicroPython library for smooth, non-blocking stepper motor control on the
Raspberry Pi Pico (RP2040) and Pico2 (RP2350). Uses PIO state machines and DMA for precise,
CPU-independent pulse generation and position counting.

## Features

- Non-blocking absolute and relative moves (`moveTo`)
- Non-blocking continuous jog (`jog`) with configurable speed
- Async homing with configurable sensor polarity; handles pre-asserted sensor (`homing.py`)
- Smooth acceleration and deceleration (linear, smooth1, smooth2/smootherstep, sine curves)
- Graceful stop with deceleration, or emergency hard stop
- Dynamic parameter adjustment (speed, acceleration) mid-move with automatic motion replan
- Position tracking via PIO pulse counter
- Active-low enable pin support (auto-enabled on move start)
- Move timeout with automatic emergency stop
- Up to 4 simultaneous stepper instances (limited by RP2040 PIO state machines)

## Files

| File | Description |
| --- | --- |
| `smartStepper.py` | High-level `SmartStepper` class |
| `homing.py` | Async homing routine (three-phase, handles pre-asserted sensor) |
| `pulseGenerator.py` | PIO + DMA pulse generator (internal) |
| `pulseCounter.py` | PIO-based pulse counter for position tracking (internal) |
| `test_smartStepper.py` | Manual tests and usage examples |

## Usage

### Basic setup

```python
import smartStepper

stepper = smartStepper.SmartStepper(
    stepPin=27,       # step pulse output pin number (or machine.Pin)
    dirPin=26,        # direction output pin number (or machine.Pin)
    enablePin=25,     # optional active-low enable pin (or None)
    accelCurve='smooth2'  # 'linear', 'smooth1', 'smooth2', or 'sine'
)

stepper.stepsPerUnit = 96.   # microsteps per mm (or whatever unit you use)
stepper.minSpeed     = 1     # units/s — starting/stopping speed
stepper.maxSpeed     = 50    # units/s — peak speed
stepper.acceleration = 300   # units/s²
```

### Absolute move

```python
stepper.moveTo(100)          # move to 100 mm (absolute)
stepper.waitEndOfMove()      # block until done

stepper.moveTo(0)            # return to origin
stepper.waitEndOfMove()
```

### Relative move

```python
stepper.moveTo(50, relative=True)   # move forward 50 mm
stepper.waitEndOfMove()

stepper.moveTo(-20, relative=True)  # move back 20 mm
stepper.waitEndOfMove()
```

### Move with timeout

```python
try:
    stepper.moveTo(200, timeout=5.0)   # fail if not done in 5 seconds
    stepper.waitEndOfMove()
except smartStepper.SmartStepperError as e:
    print("Error:", e)   # "Move timed out" if motor stalled
```

Alternatively, poll without blocking:

```python
stepper.moveTo(200, timeout=5.0)
while stepper.moving:
    if stepper.timedOut:
        stepper.stop(emergency=True)
        break
    # ... do other work
```

### Jog (continuous motion)

```python
stepper.jog(maxSpeed=30, direction='up')   # start jogging
# ... application loop ...
stepper.stop()                             # decelerate and stop
stepper.waitEndOfMove()                    # wait for decel to finish
```

### Enable/disable motor driver

The enable pin is driven automatically when a move starts. You can also
control it manually:

```python
stepper.disable()   # de-energize coils (reduce heat / allow manual movement)
stepper.enable()    # re-energize coils
```

### Dynamic parameter changes mid-move

Speed and acceleration can be updated while the motor is moving. The motion
profile is automatically rebuilt and handed to the DMA controller without
stopping (~10 µs transition):

```python
stepper.moveTo(500)
time.sleep(0.5)
stepper.maxSpeed = 20     # slow down on the fly
time.sleep(0.5)
stepper.maxSpeed = 80     # speed back up
stepper.waitEndOfMove()
```

### Position

```python
print(stepper.position)    # current position in units (read from PIO counter)
stepper.position = 0       # reset/home position (only when not moving)
```

### Emergency stop

```python
stepper.stop(emergency=True)   # cut pulses immediately (may lose steps)
```

### Homing

`homing.py` provides an async homing routine that works with any sensor that
has a `value()` method (e.g. `machine.Pin`).

**Phase 0 — initial backoff (if needed)**: if the sensor is already asserted
when homing starts, jog away from it at `slowSpeed` until it de-asserts, then
stop. This makes homing repeatable regardless of starting position.

**Phase 1 — fast approach**: jog toward the sensor at `fastSpeed`. When the
sensor asserts, decelerate smoothly to a stop.

**Phase 2 — slow backoff**: jog away from the sensor at `slowSpeed`. The
instant the sensor de-asserts, stop immediately and define that position as
home (`position = 0`).

```python
import asyncio
import homing

sensor = machine.Pin(15, machine.Pin.IN, machine.Pin.PULL_UP)

async def main():
    await homing.home(
        stepper,
        sensor,
        fastSpeed  = 40,     # units/s, approach speed
        slowSpeed  = 2,      # units/s, backoff speed
        direction  = 'down', # direction toward home sensor
        activeState= 0,      # 0 = active-low (pull-up wiring)
        timeout    = 10.0,   # raise HomingError if not done in 10 s
    )
    print("Homed at", stepper.position)  # always 0

asyncio.run(main())
```

`activeState=1` for active-high sensors (pull-down or open-collector with
external pull-up to logic level). `timeout=None` disables the timeout.
`minSpeed` and `maxSpeed` are saved and restored after homing completes
or times out.

## API reference

### Constructor

```python
SmartStepper(stepPin, dirPin, enablePin=None, accelCurve='smooth2')
```

### Properties

| Property | Writable | Description |
| --- | --- | --- |
| `position` | yes (stopped only) | Current position in units |
| `target` | no | Target position set by last `moveTo()` |
| `speed` | no | Current speed in units/s |
| `direction` | no | `'up'`, `'down'`, or `None` |
| `moving` | no | `True` while motor is running |
| `timedOut` | no | `True` if move deadline has passed |
| `minSpeed` | yes | Start/stop speed in units/s (triggers replan if moving) |
| `maxSpeed` | yes | Peak speed in units/s (triggers replan if moving) |
| `acceleration` | yes | Acceleration in units/s² (triggers replan if moving) |
| `stepsPerUnit` | yes (stopped only) | Microsteps per unit |
| `reverse` | yes (stopped only) | Invert direction pin polarity |

### Methods

| Method | Description |
| --- | --- |
| `moveTo(target, relative=False, timeout=None)` | Start a move; non-blocking |
| `jog(maxSpeed=None, direction='up')` | Start continuous jogging; non-blocking |
| `stop(emergency=False)` | Stop with decel (default) or immediately |
| `waitEndOfMove()` | Block until stopped; raises on timeout |
| `enable()` | Assert enable pin (active-low) |
| `disable()` | Release enable pin |

## Hardware notes

- **Enable pin**: Most stepper drivers use an active-low enable signal. The
  pin is driven high (disabled) at startup and low (enabled) automatically
  when a move begins.
- **Direction pin setup time**: The RP2040 PIO begins pulsing immediately
  after direction is set. If your driver requires a direction-setup hold time,
  add a short `time.sleep_us()` before calling `moveTo()`.
- **Step pulse width**: The PIO generates a 50% duty-cycle square wave. Pulse
  width = `1 / (2 × freq)`. At 10 kHz step rate this is 50 µs per half-cycle,
  which is compatible with all common stepper drivers.

## Credits

Original library by **Frédéric** (fma@gbiloba.org) (2023), 
posted at [https://framagit.org/fma38/micropython-lib](https://framagit.org/fma38/micropython-lib), and licensed under the
[GNU Affero General Public License v3](LICENSE).

`pulseCounter.py` is based on original work by
[Dave Hylands](https://github.com/dhylands/upy-examples/blob/master/pico/pio_pulse_counter.py).

### Changes by Ned Konz (ned@metamagix.tech) (2026)

- Fixed garbage-collector bug in `pulseGenerator.py`: DMA sequence array is
  now pinned as an instance variable to prevent it from being collected while
  DMA is active.
- Added `PulseGenerator.update()` for non-blocking mid-run DMA replacement.
- Generalized `_accelPoints()` to handle both acceleration and deceleration
  (reversal of a symmetric smoothstep curve), replacing the separate
  `_decelPoints()` method.
- Added `_buildProfile()` unified motion planner used by `moveTo()` and
  `_replan()`.
- Added `_replan()` to rebuild the motion profile in-flight when speed or
  acceleration is changed mid-move.
- Made `minSpeed`, `maxSpeed`, and `acceleration` setters trigger `_replan()`
  instead of raising an error when the motor is moving.
- Added `stop(emergency=False)` with smooth deceleration by default.
- Added active-low enable pin support (`enablePin` constructor argument,
  `enable()` / `disable()` methods, auto-enable on move start).
- Added `timeout` parameter to `moveTo()`, `timedOut` property, and timeout
  handling in `waitEndOfMove()`.
- Fixed `position` setter unit-conversion bug.
- Fixed `waitEndOfMove()` self-reference bug.
- Fixed multi-instance bug in `pulseGenerator.py`: SM index is now captured
  as an instance variable at construction time rather than read from the class
  variable (which could have been incremented by later instances).
- Replaced custom `dma.py` with MicroPython's built-in `rp2.DMA`: uses
  `pack_ctrl()` / `config()` API, drops `import uctypes` (array passed
  directly via buffer protocol), and is inherently RP2350-compatible.
- Extracted test/demo code into `test_smartStepper.py`.
- Added `homing.py`: async three-phase homing with configurable sensor polarity,
  speed parameters, and timeout.
