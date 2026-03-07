# Tests

This directory contains two categories of tests:

- **Manual / demo tests** (run directly on the Pico via `mpremote`) — quick
  smoke-tests of individual modules.
- **Hardware-in-the-loop (HIL) tests** (run on the host PC) — automated tests
  that deploy scripts to the Pico, capture the step and direction signals with
  a Saleae Logic analyzer, and assert correctness against the raw edge data.

---

## Manual / demo tests (Pico-side)

These scripts run on the Pico. They are not automated; you observe the output
on the serial console (or measure the signals with an oscilloscope or logic
analyzer).

### Shared configuration

**`test_config.py`** — pin assignments used by every Pico-side script.

| Symbol | Default | Description |
| --- | --- | --- |
| `STEP_PIN` | 13 | GPIO pin for step signal (axis 1) |
| `DIR_PIN` | 14 | GPIO pin for direction signal (axis 1) |
| `ENABLE_PIN` | 15 | GPIO pin for enable signal (axis 1) |
| `STEP_PIN_2` | 16 | GPIO pin for step signal (axis 2) |
| `DIR_PIN_2` | 17 | GPIO pin for direction signal (axis 2) |
| `ENABLE_PIN_2` | 18 | GPIO pin for enable signal (axis 2) |

Edit this file to match your hardware before running any test.

### test_smartStepper.py

Interactive demo for `SmartStepper`. Performs a series of `moveTo()` calls
with different speeds and directions, prints position after each move, and
demonstrates mid-move `maxSpeed` changes.

```sh
PORT=/dev/cu.usbmodem1
mpremote connect $PORT cp -r smartstepper/ : + cp tests/test_config.py tests/test_smartStepper.py :
mpremote connect $PORT run tests/test_smartStepper.py
```

### test_pulseGenerator.py

Low-level demo for `PulseGenerator`. Sends a two-speed sequence directly to
the PIO/DMA and prints the number of pulses generated. Useful for verifying
PIO timing without the SmartStepper motion planner.

```sh
mpremote connect $PORT cp -r smartstepper/ : + cp tests/test_config.py tests/test_pulseGenerator.py :
mpremote connect $PORT run tests/test_pulseGenerator.py
```

### test_pulseCounter.py

Low-level demo for `PulseCounter`. Counts rising edges from a manually
driven step pin (or a signal generator) and prints the accumulated position.
Useful for verifying direction-sign convention and counter rollover.

```sh
mpremote connect $PORT cp -r smartstepper/ : + cp tests/test_config.py tests/test_pulseCounter.py :
mpremote connect $PORT run tests/test_pulseCounter.py
```

---

## Hardware-in-the-loop (HIL) tests (host-side)

The HIL suite runs on the host PC. It deploys the library and helper scripts
to the Pico, starts a Saleae Logic capture, runs the Pico script, stops the
capture, exports a `digital.csv`, and asserts correctness.

### Prerequisites

- [Logic 2](https://www.saleae.com/downloads/) open with Automation enabled
  (Settings → Automation, port 10430)
- Saleae channels wired to the Pico per `hil_config.py`
- Pico connected via USB
- Host Python packages: `pip install -e .` (installs `logic2-automation` and
  `mpremote`)

### Configuration

Edit **`hil_config.py`** to match your hardware:

| Symbol | Default | Description |
| --- | --- | --- |
| `STEP_CHANNEL` | `0` | Saleae channel → Pico `STEP_PIN` |
| `DIR_CHANNEL` | `1` | Saleae channel → Pico `DIR_PIN` |
| `STEP_CHANNEL_2` | `2` | Saleae channel → Pico `STEP_PIN_2` |
| `DIR_CHANNEL_2` | `3` | Saleae channel → Pico `DIR_PIN_2` |
| `STEP_PIN` | `13` | Pico GPIO for axis 1 step |
| `STEP_PIN_2` | `16` | Pico GPIO for axis 2 step |
| `DIGITAL_SAMPLE_RATE` | `12_500_000` | Saleae sample rate (Hz) |
| `LOGIC2_PORT` | `10430` | Logic 2 Automation port |
| `PICO_PORT` | `/dev/cu.usbmodem314201` | mpremote serial port |

### Running

```sh
python tests/test_hil.py
```

`test_hil.py` deploys the `smartstepper` package and all HIL helper scripts
to the Pico at startup, then runs each test in sequence. Captures are saved
under `tests/captures/<script_name>/` (directory is recreated on each run).

### HIL test descriptions

Each HIL test consists of two parts:
- A **Pico-side script** (`hil_*.py`) that runs the motion and prints a
  result line to stdout.
- A **host-side assertion** in `test_hil.py` that verifies the Saleae edge
  data and the Pico stdout.

---

#### `test_pulse_generator` — PulseGenerator timing

Pico script: `test_pulseGenerator.py`

Runs a two-speed sequence directly via `PulseGenerator`: 3 pulses at 1 Hz
then 5 pulses at 5 Hz (8 total). Asserts:
- Total edge count is 8 (±1).
- First three inter-pulse gaps are ~1 s (1 Hz segment).
- Next four inter-pulse gaps are ~0.2 s (5 Hz segment).

---

#### `test_moveto_pulse_count` — moveTo step count

Pico script: `hil_moveto.py`

`moveTo(50)` with `stepsPerUnit=96`, `minSpeed=5`, `maxSpeed=50`,
`acceleration=300`. Expected: exactly 4800 steps (50 × 96). Asserts:
- Saleae rising-edge count equals the on-board `PulseCounter` value.
- Count is within 1 of 4800.

---

#### `test_accel_profile` — acceleration/deceleration shape

Pico script: `hil_moveto.py` (same capture as above)

Checks that the step frequency is strictly increasing at the start and
strictly decreasing at the end of the move, confirming the accel/decel ramps
are present.

---

#### `test_triangular_profile` — triangular move

Pico script: `hil_triangular.py`

`moveTo(5, triangular=True)` with `stepsPerUnit=96`, `maxSpeed=50`,
`acceleration=300`. Natural peak ≈ 39.1 units/s (below `maxSpeed`), so no
constant-velocity section. Expected ~480 pulses. Asserts:
- Step count ≈ 480 (±2).
- Frequency peak is near the midpoint of the move (30–70 %).
- No constant-speed plateau (peak stays below the `maxSpeed` ceiling).

---

#### `test_accel_time_profile` — fixed acceleration time

Pico script: `hil_accel_time.py`

`moveTo(50, accel_time=0.1)` with `acceleration=300`. Forced peak =
5 + 300 × 0.1 = 35 units/s (well below `maxSpeed=50`). Expected ~4800
pulses. Asserts:
- Step count ≈ 4800 (±2).
- Peak frequency ~3360 Hz (35 × 96), not at the `maxSpeed` ceiling.
- Cruise section (middle 50 % of the move) averages near 3360 Hz.

---

#### `test_replan_profile` — mid-move speed change

Pico script: `hil_replan.py`

`moveTo(100)` at `maxSpeed=50`; after 1 s mid-cruise, `maxSpeed` is reduced
to 25, triggering `_replan()`. Asserts:
- First 20 % of steps are in the fast phase (peak > 4000 Hz).
- Last 20 % of steps are in the slow phase (peak < 3000 Hz after replan).

---

#### `test_stop_restart` — smooth stop and restart

Pico script: `hil_stop_restart.py`

`moveTo(500)` (≈48000 steps, ~10 s); `stop()` is called after 1 s
(mid-cruise), then `moveTo(0)` to return to origin. `stop()` decelerates
smoothly via `interrupt_with()`. Asserts:
- Phase 1 produced > 1000 steps (motor was running) and < 40000 (did not
  reach target before stop).
- Final `PulseCounter` value after phase 2 is within 1 step of zero.
- No inter-step gap exceeds 3 s (no stall).

---

#### `test_multiaxis_sync` — multi-axis synchronized move

Pico script: `hil_multiaxis.py`

`MultiAxis.move({x: 40, y: 30})` — axis 1 is the dominant axis (T_total ≈
0.934 s at `maxSpeed=50`); axis 2 is slowed to v_peak ≈ 35 u/s so that both
axes finish together. Both axes start via a single write to
`DMA_MULTI_CHAN_TRIGGER`. Asserts:
- First rising edge on each axis within 100 µs (hardware-simultaneous start).
- Axis 1: ≈ 3840 steps; Axis 2: ≈ 2880 steps (both ±2).
- Last rising edge on each axis within 20 ms (simultaneous finish).
- Saleae counts match on-board `PulseCounter` values.

---

#### `test_arc_quarter_circle` — circular arc (G03)

Pico script: `hil_arc.py`

`Arc.move()` — quarter-circle CCW from (0, 0) to (100, 100) with center
offset i=0, j=100 (center at (0, 100), radius=100), `stepsPerUnit=100`,
`chord_tol=0.15` (15 segments). Both axes move monotonically, so the net
displacement equals the step count on each axis. Asserts:
- X and Y step counts ≈ 10000 (±5).
- First rising edges on both axes within 100 µs (simultaneous segment start).
- Last rising edges on both axes within 80 ms (simultaneous last-segment end).
- At least 14 inter-segment gaps in each pulse train (15 segments → 14
  boundaries).
- No inter-step gap > 3 s (no stall).
- Pico-reported final positions: x_pos ≈ 100.00, y_pos ≈ 100.00 (±0.5).
- Saleae counts match on-board `PulseCounter` values.
