# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

MicroPython library for smooth, non-blocking stepper motor control on the RP2040 (Raspberry Pi Pico) and RP2350 (Pico2). Uses PIO state machines and DMA for CPU-independent pulse generation and position counting.

**Target runtime**: MicroPython on RP2040/RP2350. This is not standard Python — only modules available on MicroPython (`machine`, `rp2`, `array`, `asyncio`, `time`, `math`) are available.

## Running / testing

Test scripts live in `tests/`. There is no automated test runner — all tests are deployed to hardware manually or via the HIL suite.

**Manual tests (Pico-side):**

- `tests/test_smartStepper.py` — SmartStepper demo; run via `mpremote run`
- `tests/test_pulseGenerator.py` — PulseGenerator standalone demo
- `tests/test_pulseCounter.py` — PulseCounter standalone demo
- `tests/test_config.py` — shared pin assignments (`STEP_PIN`, `DIR_PIN`, `ENABLE_PIN`)

To deploy and run:
```
mpremote cp smartStepper.py pulseGenerator.py pulseCounter.py tests/test_config.py tests/test_smartStepper.py :
mpremote run tests/test_smartStepper.py
```

**Hardware-in-the-loop (HIL) tests (host-side, requires Saleae Logic 2):**

- `tests/test_hil.py` — host runner; deploys to Pico, captures signals, asserts correctness
- `tests/hil_config.py` — wiring and port config (edit to match hardware)
- `tests/hil_moveto.py` — Pico-side script executed by the HIL suite

```
python tests/test_hil.py
```

## Architecture

```
SmartStepper  (smartStepper.py)
├── PulseGenerator  (pulseGenerator.py)  — PIO0, SM 0–3; rp2.DMA
└── PulseCounter    (pulseCounter.py)    — PIO1, SM 4–7
```

**PulseGenerator** (`pulseGenerator.py`): Takes a list of `(freq_hz, n_pulses)` tuples (a motion profile), encodes them as a word array, and streams them to PIO0 via DMA. The PIO state machine generates a 50% duty-cycle square wave on the step pin. Class variable `_num` increments at construction; each instance captures its SM index as `self._smNum`. `_sequence` must be kept as an instance variable to prevent GC collection while DMA is active.

**PulseCounter** (`pulseCounter.py`): Uses PIO1 (SM 4–7) to count rising edges on the step pin. Direction is set by loading a different PIO program (`_pioCodeUp` increments X, `_pioCodeDown` decrements X). Position is stored offset by `2147483648` to allow signed values in an unsigned 32-bit register.

**SmartStepper** (`smartStepper.py`): High-level motion controller. Key internal methods:
- `_accelPoints(fromSpeed, toSpeed)` — generates `(freq, pulses)` segments for a ramp; decel is accel reversed (smoothstep curves are point-symmetric)
- `_buildProfile(fromSpeed, remaining)` — computes achievable peak speed then produces accel → const → decel segments
- `_replan()` — called by `minSpeed`/`maxSpeed`/`acceleration` setters when moving; rebuilds profile from current speed/position and calls `PulseGenerator.update()` for seamless DMA handoff (~10 µs)

**homing.py**: Async three-phase homing coroutine. Uses `asyncio.sleep_ms(1)` for polling. Saves and restores `minSpeed`/`maxSpeed`. Phase 2 ends with an emergency stop at the exact sensor de-assert edge.

## Key constraints

- Maximum **4 simultaneous SmartStepper instances** (limited by RP2040 PIO state machines)
- `stepsPerUnit`, `reverse`, and `position` (setter) can only be changed while **not moving**
- `minSpeed`, `maxSpeed`, `acceleration` can be changed mid-move and trigger `_replan()`
- `NB_ACCEL_PTS = 100` controls motion profile resolution (in `smartStepper.py`)
- PIO0 TX FIFO addresses and DREQ indices are hardcoded for RP2040/RP2350 at `0x50200010`+
