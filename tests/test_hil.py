"""Host-side hardware-in-the-loop tests for stepper-lib.

Prerequisites:
  - Logic 2 open with Automation enabled (Settings → Automation, port 10430)
  - Saleae wired to Pico per hil_config.py
  - Pico connected via USB

Run from the repo root or from tests/:
  python tests/test_hil.py
"""

import os

# Set these BEFORE importing grpc
os.environ["GRPC_VERBOSITY"] = "NONE" # Stops almost all gRPC internal logging
os.environ["GRPC_TRACE"] = ""         # Ensures no specific tracing is active

import argparse
import csv
import shutil
import subprocess
import sys
from pathlib import Path

import saleae.automation as saleae

# Allow running from repo root or from tests/
sys.path.insert(0, str(Path(__file__).parent))
import hil_config

# ---------------------------------------------------------------------------
# Infrastructure
# ---------------------------------------------------------------------------

_TESTS_DIR    = Path(__file__).parent
_LIB_DIR      = _TESTS_DIR.parent
_PACKAGE_DIR  = _LIB_DIR / 'smartstepper'
_CAPTURES_DIR = _TESTS_DIR / 'captures'

# Pico-side test helper scripts (deployed to the Pico root).
PICO_TEST_FILES = [
    str(_TESTS_DIR / 'test_config.py'),
    str(_TESTS_DIR / 'hil_moveto.py'),
    str(_TESTS_DIR / 'hil_replan.py'),
    str(_TESTS_DIR / 'hil_stop_restart.py'),
    str(_TESTS_DIR / 'hil_triangular.py'),
    str(_TESTS_DIR / 'hil_accel_time.py'),
    str(_TESTS_DIR / 'hil_multiaxis.py'),
    str(_TESTS_DIR / 'hil_arc.py'),
    str(_TESTS_DIR / 'hil_pc_speed_change.py'),
    str(_TESTS_DIR / 'hil_pc_stop_start.py'),
    str(_TESTS_DIR / 'hil_pc_direction_change.py'),
]


def _mpremote(*args):
    """Run an mpremote command; return (stdout, returncode)."""
    cmd = ['mpremote', 'connect', hil_config.PICO_PORT] + list(args)
    result = subprocess.run(cmd, capture_output=True, text=True)
    return result.stdout + result.stderr, result.returncode


def _force_step_low(pin=None):
    """Drive a step pin LOW on the Pico via mpremote exec.

    Defaults to hil_config.STEP_PIN (axis 1). Pass a different pin number to
    quiesce additional step outputs before/after a capture.
    """
    if pin is None:
        pin = hil_config.STEP_PIN
    _mpremote('exec', f'import machine; machine.Pin({pin}, machine.Pin.OUT, value=0)')


def deploy():
    """Copy the smartstepper package and HIL scripts to the Pico."""
    print('  Deploying files to Pico...')
    # Deploy the package directory recursively (creates lib/smartstepper/ on the Pico).
    stdout, rc = _mpremote('cp', '-r', str(_PACKAGE_DIR) + '/', ':/lib')
    if rc != 0:
        raise RuntimeError(f'deploy (package) failed:\n{stdout}')
    # Deploy test helper scripts to the Pico root.
    stdout, rc = _mpremote('cp', *PICO_TEST_FILES, ':')
    if rc != 0:
        raise RuntimeError(f'deploy (test scripts) failed:\n{stdout}')


def parse_rising_edges(digital_csv: Path, channel: int, min_time: float = 0.0) -> list:
    """Return list of timestamps (s) for rising edges on the given Saleae channel.

    min_time: skip edges before this many seconds into the capture, to ignore
    startup transients that occur before the Pico script begins executing.
    """
    col = f'Channel {channel}'
    edges = []
    prev = None
    with open(digital_csv, newline='') as f:
        for row in csv.DictReader(f):
            t = float(row['Time [s]'])
            val = int(row[col])
            if t >= min_time and prev == 0 and val == 1:
                edges.append(t)
            prev = val
    return edges


def parse_net_position(
    digital_csv: Path,
    step_channel: int,
    dir_channel: int,
    min_time: float = 0.0,
) -> int:
    """Return signed net position from direction-aware step edge counting.

    Each rising edge on step_channel counts as +1 (DIR=HIGH) or -1 (DIR=LOW).
    The raw CSV contains all channel values at every transition, so cur_dir
    is always current at the moment of each step rising edge.
    """
    step_col = f'Channel {step_channel}'
    dir_col = f'Channel {dir_channel}'
    net = 0
    prev_step = None
    cur_dir = 1
    with open(digital_csv, newline='') as f:
        for row in csv.DictReader(f):
            t = float(row['Time [s]'])
            cur_dir = int(row[dir_col])
            step_val = int(row[step_col])
            if t >= min_time and prev_step == 0 and step_val == 1:
                net += 1 if cur_dir == 1 else -1
            prev_step = step_val
    return net


def run_capture(manager: saleae.Manager, script: str, channels: list):
    """
    Start a manual Saleae capture, run a Pico script synchronously, then stop.

    Returns (tmpdir Path, mpremote stdout string).
    Raises RuntimeError if the Pico script exits non-zero.
    """
    tmpdir = _CAPTURES_DIR / script.replace('.py', '')
    tmpdir.mkdir(parents=True, exist_ok=True)
    dev_cfg = saleae.LogicDeviceConfiguration(
        enabled_digital_channels=channels,
        digital_sample_rate=hil_config.DIGITAL_SAMPLE_RATE,
        digital_threshold_volts=3.3
    )
    cap_cfg = saleae.CaptureConfiguration(
        capture_mode=saleae.ManualCaptureMode()
    )

    _force_step_low()
    cap = manager.start_capture(device_configuration=dev_cfg,
                                capture_configuration=cap_cfg)
    try:
        stdout, rc = _mpremote('run', str(_TESTS_DIR / script))
        cap.stop()
        _force_step_low()
        if rc != 0:
            raise RuntimeError(f'mpremote run {script} failed:\n{stdout}')
        cap.export_raw_data_csv(str(tmpdir), digital_channels=channels)
        cap.save_capture(str(tmpdir / f"{script}.sal"))
    finally:
        cap.close()

    return tmpdir, stdout


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_pulse_generator(manager: saleae.Manager):
    """
    PulseGenerator points=((1,3),(5,5)) → 8 pulses.
    Gaps 0-2 ~1 s (1 Hz + transition); gaps 3-6 ~0.2 s (5 Hz).
    """
    channels = [hil_config.STEP_CHANNEL]
    tmpdir, _ = run_capture(manager, 'test_pulseGenerator.py', channels)

    edges = parse_rising_edges(tmpdir / 'digital.csv', hil_config.STEP_CHANNEL)
    assert abs(len(edges) - 8) <= 1, f'Expected 8 pulses (±1), got {len(edges)}'

    # Use the first 8 edges (skip any trailing spurious edge from Pico soft-reset)
    edges = edges[:8]
    gaps = [edges[i + 1] - edges[i] for i in range(len(edges) - 1)]
    # Gaps 0-2: ~1 s (two 1 Hz inter-pulse gaps + one transition gap; the PIO
    # completes the final 1 Hz LOW half-period before starting the 5 Hz segment,
    # so the gap from the 3rd rising edge to the 1st 5 Hz rising edge = 1 full period)
    for i in range(3):
        assert 0.8 < gaps[i] < 1.25, f'Gap {i} expected ~1 s, got {gaps[i]:.3f} s'
    # Gaps 3-6: ~5 Hz (0.2 s)
    for i in range(3, 7):
        assert 0.15 < gaps[i] < 0.28, f'Gap {i} expected ~0.2 s, got {gaps[i]:.3f} s'

    print(f'  PASS  test_pulse_generator  ({len(edges)} pulses, timing OK)')


def test_moveto_pulse_count(manager: saleae.Manager):
    """
    moveTo(50) with stepsPerUnit=96 → ~4800 step pulses.
    Saleae and PulseCounter must agree exactly; count must be within 1 of 4800.
    """
    channels = [hil_config.STEP_CHANNEL, hil_config.DIR_CHANNEL]
    tmpdir, stdout = run_capture(manager, 'hil_moveto.py', channels)

    saleae_count = len(parse_rising_edges(tmpdir / 'digital.csv', hil_config.STEP_CHANNEL))

    # Cross-check with PulseCounter value printed by the Pico script
    pico_count = None
    for line in stdout.splitlines():
        if line.startswith('done steps='):
            pico_count = int(line.split('=')[1])
            break

    assert pico_count is not None, f'Pico script did not print "done steps=..."\nstdout: {stdout}'
    assert saleae_count == pico_count, \
        f'Saleae ({saleae_count}) and PulseCounter ({pico_count}) disagree'
    assert abs(saleae_count - 4800) <= 1, \
        f'Expected ~4800 pulses, got {saleae_count}'

    print(f'  PASS  test_moveto_pulse_count  ({saleae_count} pulses, matches PulseCounter)')


def test_accel_profile(manager: saleae.Manager):
    """
    moveTo should accel then cruise then decel.
    Checks that instantaneous frequency is strictly increasing at start
    and strictly decreasing at end of the move.
    """
    channels = [hil_config.STEP_CHANNEL]
    tmpdir, _ = run_capture(manager, 'hil_moveto.py', channels)

    edges = parse_rising_edges(tmpdir / 'digital.csv', hil_config.STEP_CHANNEL)
    assert len(edges) > 50, f'Too few pulses to check profile ({len(edges)})'

    # Instantaneous frequency between successive pulses
    freqs = [1.0 / (edges[i + 1] - edges[i]) for i in range(len(edges) - 1)]

    # Accel: first N frequency samples must be monotonically increasing
    n = min(20, len(freqs) // 4)
    accel = freqs[:n]
    assert all(accel[i] < accel[i + 1] for i in range(len(accel) - 1)), \
        f'Accel ramp not monotonically increasing: {[f"{f:.1f}" for f in accel]}'

    # Decel: last N frequency samples must be monotonically decreasing
    decel = freqs[-n:]
    assert all(decel[i] > decel[i + 1] for i in range(len(decel) - 1)), \
        f'Decel ramp not monotonically decreasing: {[f"{f:.1f}" for f in decel]}'

    peak_hz = max(freqs)
    print(f'  PASS  test_accel_profile  (peak {peak_hz:.0f} Hz, accel/decel ramps OK)')


def test_stop_restart(manager: saleae.Manager):
    """
    Start a long move (500 units ≈ 48000 steps, ~10 s), issue stop() after
    1 s mid-cruise, then move back to origin.

    stop() calls interrupt_with() which zeroes the PIO Y register so the
    current segment ends after one half-period, then plays a decel ramp to
    minSpeed before halting.

    Checks:
      - Phase 1 stops before reaching the target (fewer than 40000 steps).
      - Phase 1 produces at least 1000 steps, confirming the motor was
        running before stop() was called.
      - The final PulseCounter value is within 1 step of zero (origin).
      - No gap between any two consecutive steps exceeds 3 s.
    """
    channels = [hil_config.STEP_CHANNEL, hil_config.DIR_CHANNEL]
    tmpdir, stdout = run_capture(manager, 'hil_stop_restart.py', channels)

    # Parse per-direction edge lists
    step_edges  = parse_rising_edges(tmpdir / 'digital.csv', hil_config.STEP_CHANNEL)
    assert len(step_edges) > 50, f'Too few pulses to verify stop+restart ({len(step_edges)})'

    # Extract counts from Pico stdout
    stopped_steps = final_steps = None
    for line in stdout.splitlines():
        if line.startswith('stopped steps='):
            stopped_steps = int(line.split('=')[1])
        elif line.startswith('done steps='):
            final_steps = int(line.split('=')[1])

    assert stopped_steps is not None, f'Pico did not print "stopped steps=..."\\nstdout: {stdout}'
    assert final_steps   is not None, f'Pico did not print "done steps=..."\\nstdout: {stdout}'

    # Phase 1: must have moved but not reached target (500 units * 96 = 48000 steps).
    # After 1 s at ~4800 steps/s we expect ~4800 steps + decel tail.
    assert stopped_steps > 1000, \
        f'Phase 1 too short ({stopped_steps} steps); motor may not have started'
    assert stopped_steps < 40000, \
        f'Phase 1 reached target before stop() ({stopped_steps} steps)'

    # After phase 2 the motor must be back near origin
    assert abs(final_steps) <= 1, \
        f'Motor did not return to origin; PulseCounter={final_steps}'

    # No stall gap between the two phases
    gaps = [step_edges[i + 1] - step_edges[i] for i in range(len(step_edges) - 1)]
    max_gap = max(gaps)
    assert max_gap < 3.0, f'Suspiciously long gap between pulses: {max_gap:.3f} s'

    print(f'  PASS  test_stop_restart  '
          f'(stopped at {stopped_steps} steps, returned to {final_steps})')


def test_triangular_profile(manager: saleae.Manager):
    """
    moveTo(5, triangular=True) with stepsPerUnit=96, maxSpeed=50, accel=300.

    Natural peak ≈ 39.1 units/s → ~3753 Hz; no constant-velocity section.
    Expected ~480 pulses. Frequency must rise monotonically to a single peak
    near the midpoint of the move, then fall monotonically — with no plateau.
    """
    channels = [hil_config.STEP_CHANNEL]
    tmpdir, _ = run_capture(manager, 'hil_triangular.py', channels)

    edges = parse_rising_edges(tmpdir / 'digital.csv', hil_config.STEP_CHANNEL)
    assert abs(len(edges) - 480) <= 2, f'Expected ~480 pulses, got {len(edges)}'
    assert len(edges) > 20, f'Too few pulses to analyze profile ({len(edges)})'

    freqs = [1.0 / (edges[i + 1] - edges[i]) for i in range(len(edges) - 1)]
    peak_hz = max(freqs)
    peak_idx = freqs.index(peak_hz)
    total = len(freqs)

    # Peak should be near the midpoint (between 30 % and 70 % of the move).
    assert 0.30 * total < peak_idx < 0.70 * total, \
        f'Triangular peak not near midpoint: idx {peak_idx}/{total}'

    # First 1/8: frequency must trend upward (accel ramp).
    # Check every-other-sample to tolerate per-segment quantization jitter
    # (adjacent timing measurements can alternate ±1–2 Hz due to integer step counts).
    n = max(5, total // 8)
    accel_section = freqs[:n]
    assert all(accel_section[i] <= accel_section[i + 2]
               for i in range(len(accel_section) - 2)), \
        f'Accel ramp not trending upward: {[f"{f:.0f}" for f in accel_section]}'

    # Last 1/8: frequency must trend downward (decel ramp).
    decel_section = freqs[-n:]
    assert all(decel_section[i] >= decel_section[i + 2]
               for i in range(len(decel_section) - 2)), \
        f'Decel ramp not trending downward: {[f"{f:.0f}" for f in decel_section]}'

    # No plateau: cruise speed stays below maxSpeed ceiling (50 * 96 = 4800 Hz).
    assert peak_hz < 4500, \
        f'Triangular peak {peak_hz:.0f} Hz reached maxSpeed ceiling — const section present?'

    print(f'  PASS  test_triangular_profile  '
          f'({len(edges)} pulses, peak {peak_hz:.0f} Hz at idx {peak_idx}/{total})')


def test_accel_time_profile(manager: saleae.Manager):
    """
    moveTo(50, accel_time=0.1) with stepsPerUnit=96, maxSpeed=50, accel=300.

    forced_peak = 5 + 300*0.1 = 35 units/s → ~3360 Hz cruise (below maxSpeed=50).
    Expected ~4800 pulses. Cruise section must be near 3360 Hz, not 4800 Hz.
    """
    channels = [hil_config.STEP_CHANNEL]
    tmpdir, _ = run_capture(manager, 'hil_accel_time.py', channels)

    edges = parse_rising_edges(tmpdir / 'digital.csv', hil_config.STEP_CHANNEL)
    assert abs(len(edges) - 4800) <= 2, f'Expected ~4800 pulses, got {len(edges)}'
    assert len(edges) > 50, f'Too few pulses to analyze profile ({len(edges)})'

    freqs = [1.0 / (edges[i + 1] - edges[i]) for i in range(len(edges) - 1)]
    peak_hz = max(freqs)

    # forced_peak=35 units/s → ~3360 Hz; allow ±15 % for discretisation.
    assert 2800 < peak_hz < 4000, \
        f'Peak {peak_hz:.0f} Hz not in expected range for forced_peak=35 u/s (2800–4000 Hz)'

    # Motor must not reach maxSpeed (50 * 96 = 4800 Hz).
    assert peak_hz < 4500, \
        f'Peak {peak_hz:.0f} Hz reached maxSpeed ceiling — accel_time clamp not working?'

    # Middle 50 % of the move should be the constant-speed cruise near forced_peak.
    mid_start = len(freqs) // 4
    mid_end   = 3 * len(freqs) // 4
    avg_cruise = sum(freqs[mid_start:mid_end]) / (mid_end - mid_start)
    assert 2800 < avg_cruise < 4000, \
        f'Cruise avg {avg_cruise:.0f} Hz not near forced_peak=35 u/s (expected ~3360 Hz)'

    print(f'  PASS  test_accel_time_profile  '
          f'({len(edges)} pulses, peak {peak_hz:.0f} Hz, cruise avg {avg_cruise:.0f} Hz)')


def test_replan_profile(manager: saleae.Manager):
    """
    moveTo(100) at maxSpeed=50 (4800 Hz); after 1 s mid-cruise, maxSpeed is
    lowered to 25 (2400 Hz), triggering _replan().

    Checks that:
      - The first 20% of steps cruise near 4800 Hz (fast phase).
      - The last 20% of steps cruise near 2400 Hz (slow phase after replan).
    """
    channels = [hil_config.STEP_CHANNEL]
    tmpdir, _ = run_capture(manager, 'hil_replan.py', channels)

    edges = parse_rising_edges(tmpdir / 'digital.csv', hil_config.STEP_CHANNEL)
    assert len(edges) > 200, f'Too few pulses to check replan ({len(edges)})'

    freqs = [1.0 / (edges[i + 1] - edges[i]) for i in range(len(edges) - 1)]
    n = len(freqs) // 5  # 20 % window

    # First 20 %: must be in the fast cruise phase (maxSpeed=50 → ~4800 Hz)
    peak_fast = max(freqs[:n])
    assert peak_fast > 4000, \
        f'Fast phase peak {peak_fast:.0f} Hz, expected >4000 (maxSpeed=50)'

    # Last 20 %: must be in the slow phase after replan (maxSpeed=25 → ~2400 Hz)
    peak_slow = max(freqs[-n:])
    assert peak_slow < 3000, \
        f'Slow phase peak {peak_slow:.0f} Hz, expected <3000 after replan to maxSpeed=25'

    print(f'  PASS  test_replan_profile  '
          f'(fast peak {peak_fast:.0f} Hz → slow peak {peak_slow:.0f} Hz)')


def test_multiaxis_sync(manager: saleae.Manager):
    """MultiAxis.move({x:40, y:30}) — simultaneous start AND simultaneous finish.

    Axis 1: 40 units * 96 steps/unit = 3840 steps.
      Trapezoidal at maxSpeed=50; T_total ~= 0.934 s (dominant axis).
    Axis 2: 30 units * 96 steps/unit = 2880 steps.
      Natural peak ~95 u/s → capped to 50; T_natural ~= 0.733 s.
      MultiAxis binary-searches for v_peak ~= 35 u/s → T_total ~= 0.934 s.
      The 5→35 u/s accel ramp is visible on the logic analyser.

    Checks:
      - First rising edge on each axis within 100 µs of the other (hardware
        simultaneous start via DMA_MULTI_CHAN_TRIGGER).
      - Step counts: axis 1 ≈ 3840 ± 2, axis 2 ≈ 2880 ± 2.
      - Last rising edge on each axis within 20 ms of the other (simultaneous
        finish — the key CNC synchronization property).
      - Saleae counts match PulseCounter values from Pico stdout.
    """
    channels = [
        hil_config.STEP_CHANNEL, hil_config.DIR_CHANNEL,
        hil_config.STEP_CHANNEL_2, hil_config.DIR_CHANNEL_2,
    ]
    tmpdir, stdout = run_capture(manager, 'hil_multiaxis.py', channels)
    _force_step_low(hil_config.STEP_PIN_2)  # quiesce axis 2 step pin

    edges1 = parse_rising_edges(tmpdir / 'digital.csv', hil_config.STEP_CHANNEL)
    edges2 = parse_rising_edges(tmpdir / 'digital.csv', hil_config.STEP_CHANNEL_2)

    assert len(edges1) > 20, f'Axis 1: too few pulses ({len(edges1)})'
    assert len(edges2) > 20, f'Axis 2: too few pulses ({len(edges2)})'

    # 1. Simultaneous start: first rising edges must be within 100 µs.
    skew_us = abs(edges1[0] - edges2[0]) * 1e6
    assert skew_us < 100, \
        f'Start skew {skew_us:.1f} µs exceeds 100 µs — axes not started simultaneously?'

    # 2. Step counts.
    assert abs(len(edges1) - 3840) <= 2, \
        f'Axis 1: expected ~3840 pulses, got {len(edges1)}'
    assert abs(len(edges2) - 2880) <= 2, \
        f'Axis 2: expected ~2880 pulses, got {len(edges2)}'

    # 3. Simultaneous finish: last rising edges must agree within 20 ms.
    # This validates CNC-style T_total synchronization (both axes arrive
    # at the target at the same time, not just the same accel duration).
    # MultiAxis uses _profile_time() (actual segment sums) in the binary
    # search, so finish times should agree to within a few step periods.
    end_skew_ms = abs(edges1[-1] - edges2[-1]) * 1e3
    assert end_skew_ms < 20, \
        (f'End-time skew {end_skew_ms:.1f} ms > 20 ms — '
         f'axes not finishing together?')

    # 4. PulseCounter cross-check from Pico stdout.
    # stdout line: "done x_steps=3840 y_steps=2879"
    # parts:        ['done', 'x_steps=3840', 'y_steps=2879']
    pico_x = pico_y = None
    for line in stdout.splitlines():
        if line.startswith('done x_steps='):
            parts = line.split()
            pico_x = int(parts[1].split('=')[1])
            pico_y = int(parts[2].split('=')[1])
            break
    assert pico_x is not None, f'Pico did not print "done x_steps=..."\\nstdout: {stdout}'
    assert len(edges1) == pico_x, \
        f'Axis 1: Saleae ({len(edges1)}) ≠ PulseCounter ({pico_x})'
    assert len(edges2) == pico_y, \
        f'Axis 2: Saleae ({len(edges2)}) ≠ PulseCounter ({pico_y})'

    print(f'  PASS  test_multiaxis_sync  '
          f'(start skew {skew_us:.1f} µs, end skew {end_skew_ms:.1f} ms, '
          f'{len(edges1)}/{len(edges2)} steps)')


def test_arc_quarter_circle(manager: saleae.Manager):
    """Arc.move() — quarter-circle CCW (G03) from (0,0) to (100,100), center (0,100).

    Geometry:
      stepsPerUnit=100, chord_tol=0.15 → 15 segments
      X: 0→100 monotonically (DIR=UP throughout); 10000 steps.
      Y: 0→100 monotonically (DIR=UP throughout); 10000 steps.
      Net displacement = 100 units per axis = 10000 steps per axis regardless
      of intermediate chord waypoints — chord_tol affects shape, not count.

    Checks:
      - X and Y step counts ≈ 10000 ± 5.
      - First rising edges on both axes within 100 µs (simultaneous seg-1 start).
      - Last rising edges on both axes within 20 ms (simultaneous last-seg end).
      - ≥14 inter-segment gaps in each pulse train (15 segments → 14 boundaries).
      - No gap > 3 s (no stalls).
      - Pico stdout x_pos ≈ 100.00, y_pos ≈ 100.00 (position tracking correct).
      - Saleae counts match PulseCounter values from Pico stdout.
    """
    channels = [
        hil_config.STEP_CHANNEL, hil_config.DIR_CHANNEL,
        hil_config.STEP_CHANNEL_2, hil_config.DIR_CHANNEL_2,
    ]
    tmpdir, stdout = run_capture(manager, 'hil_arc.py', channels)
    _force_step_low(hil_config.STEP_PIN_2)

    edges1 = parse_rising_edges(tmpdir / 'digital.csv', hil_config.STEP_CHANNEL)
    edges2 = parse_rising_edges(tmpdir / 'digital.csv', hil_config.STEP_CHANNEL_2)

    # 1. Step counts: net displacement = 100 units × 100 steps/unit = 10000.
    assert abs(len(edges1) - 10000) <= 5, \
        f'Axis X: expected ~10000 steps, got {len(edges1)}'
    assert abs(len(edges2) - 10000) <= 5, \
        f'Axis Y: expected ~10000 steps, got {len(edges2)}'

    # 2. Simultaneous start of first segment.
    start_skew_us = abs(edges1[0] - edges2[0]) * 1e6
    assert start_skew_us < 100, \
        f'Start skew {start_skew_us:.1f} µs > 100 µs — axes not started simultaneously?'

    # 3. Simultaneous finish of last segment.
    end_skew_ms = abs(edges1[-1] - edges2[-1]) * 1e3
    assert end_skew_ms < 80, \
        f'End skew {end_skew_ms:.1f} ms > 80 ms — last segment not finishing together?'

    # 4. Segment boundaries: ≥14 large gaps (15 segments → 14 internal boundaries).
    def _count_large_gaps(edges, ratio=5.0):
        if len(edges) < 14:
            return 0
        gaps = [edges[i + 1] - edges[i] for i in range(len(edges) - 1)]
        median_gap = sorted(gaps)[len(gaps) // 2]
        return sum(1 for g in gaps if g > ratio * median_gap)

    gaps1 = _count_large_gaps(edges1)
    gaps2 = _count_large_gaps(edges2)
    assert gaps1 >= 14, f'Axis X: expected ≥14 inter-segment gaps, found {gaps1}'
    assert gaps2 >= 14, f'Axis Y: expected ≥14 inter-segment gaps, found {gaps2}'

    # 5. No stall.
    max_gap1 = max(edges1[i + 1] - edges1[i] for i in range(len(edges1) - 1))
    max_gap2 = max(edges2[i + 1] - edges2[i] for i in range(len(edges2) - 1))
    assert max_gap1 < 3.0, f'Axis X: stall detected (max gap {max_gap1:.2f} s)'
    assert max_gap2 < 3.0, f'Axis Y: stall detected (max gap {max_gap2:.2f} s)'

    # 6 & 7. Cross-check Pico stdout.
    # line: "done x_pos=100.00 y_pos=100.00 x_steps=10000 y_steps=10000"
    pico_x_steps = pico_y_steps = None
    pico_x_pos = pico_y_pos = None
    for line in stdout.splitlines():
        if line.startswith('done x_pos='):
            parts = line.split()
            pico_x_pos   = float(parts[1].split('=')[1])
            pico_y_pos   = float(parts[2].split('=')[1])
            pico_x_steps = int(parts[3].split('=')[1])
            pico_y_steps = int(parts[4].split('=')[1])
            break
    assert pico_x_steps is not None, \
        f'Pico did not print "done x_pos=..."\nstdout: {stdout}'
    assert abs(pico_x_pos - 100.0) < 0.5, \
        f'X final pos {pico_x_pos:.2f} not near 100.00'
    assert abs(pico_y_pos - 100.0) < 0.5, \
        f'Y final pos {pico_y_pos:.2f} not near 100.00'
    assert len(edges1) == pico_x_steps, \
        f'Axis X: Saleae ({len(edges1)}) ≠ PulseCounter ({pico_x_steps})'
    assert len(edges2) == pico_y_steps, \
        f'Axis Y: Saleae ({len(edges2)}) ≠ PulseCounter ({pico_y_steps})'

    print(f'  PASS  test_arc_quarter_circle  '
          f'(start skew {start_skew_us:.1f} µs, end skew {end_skew_ms:.1f} ms, '
          f'{len(edges1)}/{len(edges2)} steps, {gaps1}/{gaps2} seg-gaps)')


def test_pulse_counter_speed_change_accuracy(manager: saleae.Manager):
    """PulseCounter must stay accurate through repeated mid-move speed replans.

    moveTo(100) at maxSpeed=50 (9600 steps).  Three maxSpeed changes are
    issued during cruise: 50->15->40->50.  Each change triggers _replan() and
    replaces the active DMA sequence.  Because replans alter velocity but not
    target position, the PulseCounter must still reach close to 9600 at the
    end, and must agree with the Saleae signed edge count.

    Expected step count is computed from actual timestamps reported by the
    Pico (to account for time.sleep_ms() inaccuracy) rather than assuming
    a fixed 9600.
    """
    channels = [hil_config.STEP_CHANNEL, hil_config.DIR_CHANNEL]
    tmpdir, stdout = run_capture(manager, 'hil_pc_speed_change.py', channels)

    saleae_net = parse_net_position(
        tmpdir / 'digital.csv',
        hil_config.STEP_CHANNEL,
        hil_config.DIR_CHANNEL,
    )

    pico_steps = None
    phase_data = None
    for line in stdout.splitlines():
        if line.startswith('done steps='):
            pico_steps = int(line.split('=')[1])
        elif line.startswith('phases='):
            phase_data = line.split('=', 1)[1]
    assert pico_steps is not None, \
        f'Pico did not print "done steps=..."\nstdout: {stdout}'
    assert phase_data is not None, \
        f'Pico did not print "phases=..."\nstdout: {stdout}'

    # Compute expected steps from actual phase durations and cruise speeds.
    # Each phase entry is "elapsed_ms,speed_steps_per_sec".
    expected_steps = 0.0
    for entry in phase_data.split(';'):
        ms_str, sps_str = entry.split(',')
        elapsed_s = int(ms_str) / 1000.0
        speed_sps = int(sps_str)
        expected_steps += elapsed_s * speed_sps

    # Allow 5% tolerance to account for accel/decel ramps between phases
    # (the cruise speed is an upper bound; ramps reduce actual steps).
    tolerance = expected_steps * 0.05

    assert saleae_net == pico_steps, (
        f'Saleae net ({saleae_net}) != PulseCounter ({pico_steps}) '
        f'after speed replans'
    )
    assert abs(saleae_net - expected_steps) <= tolerance, (
        f'Expected ~{expected_steps:.0f} steps (from timestamps), '
        f'got {saleae_net} (delta {abs(saleae_net - expected_steps):.0f}, '
        f'tolerance {tolerance:.0f})'
    )

    print(
        f'  PASS  test_pulse_counter_speed_change_accuracy  '
        f'({saleae_net} steps, expected ~{expected_steps:.0f} '
        f'from timestamps, matches PulseCounter)'
    )


def test_pulse_counter_stop_start_accuracy(manager: saleae.Manager):
    """PulseCounter must stay accurate across stop/restart cycles.

    Phase 1: moveTo(50), stop() mid-cruise.
    Phase 2: moveTo(100), wait — forward to 100 units (~9600 steps).
    Phase 3: moveTo(0) — return to origin.

    Final net displacement is zero; the Saleae signed count and the
    PulseCounter must both read ~0.  Intermediate PulseCounter values are
    sanity-checked for ordering consistency.
    """
    channels = [hil_config.STEP_CHANNEL, hil_config.DIR_CHANNEL]
    tmpdir, stdout = run_capture(manager, 'hil_pc_stop_start.py', channels)

    saleae_net = parse_net_position(
        tmpdir / 'digital.csv',
        hil_config.STEP_CHANNEL,
        hil_config.DIR_CHANNEL,
    )

    stop1 = stop2 = done = None
    for line in stdout.splitlines():
        if line.startswith('stop1 steps='):
            stop1 = int(line.split('=')[1])
        elif line.startswith('stop2 steps='):
            stop2 = int(line.split('=')[1])
        elif line.startswith('done steps='):
            done = int(line.split('=')[1])
    assert stop1 is not None, \
        f'Pico did not print "stop1 steps=..."\nstdout: {stdout}'
    assert stop2 is not None, \
        f'Pico did not print "stop2 steps=..."\nstdout: {stdout}'
    assert done is not None, \
        f'Pico did not print "done steps=..."\nstdout: {stdout}'

    # Ordering sanity: stop1 < stop2 (continued forward); done near 0 (returned).
    assert 0 < stop1 < 4800, \
        f'Phase-1 stop {stop1} not in expected range (0, 4800)'
    assert stop1 < stop2, \
        f'stop2 ({stop2}) should exceed stop1 ({stop1})'
    assert abs(stop2 - 9600) <= 1, \
        f'After phase 2, expected ~9600 steps, got {stop2}'

    # Ground-truth check: Saleae net position must match final PulseCounter.
    assert saleae_net == done, (
        f'Saleae net ({saleae_net}) != final PulseCounter ({done}) '
        f'after stop/restart cycles'
    )
    assert abs(done) <= 1, \
        f'Motor did not return to origin after stop/restart; got {done}'

    print(
        f'  PASS  test_pulse_counter_stop_start_accuracy  '
        f'(stop1={stop1}, stop2={stop2}, final={done})'
    )


def test_pulse_counter_direction_change_accuracy(manager: saleae.Manager):
    """PulseCounter must track signed position correctly through direction changes.

    Move 1 (forward):  moveTo(30)  → +2880 steps.
    Move 2 (reverse):  moveTo(0)   → back to origin.
    Move 3 (forward):  moveTo(50)  → +4800 steps.

    The Saleae direction-aware net count and the PulseCounter must agree at
    the reported waypoints, and must both read ~4800 at the end.
    """
    channels = [hil_config.STEP_CHANNEL, hil_config.DIR_CHANNEL]
    tmpdir, stdout = run_capture(
        manager, 'hil_pc_direction_change.py', channels
    )

    saleae_net = parse_net_position(
        tmpdir / 'digital.csv',
        hil_config.STEP_CHANNEL,
        hil_config.DIR_CHANNEL,
    )

    fwd = rev = done = None
    for line in stdout.splitlines():
        if line.startswith('fwd steps='):
            fwd = int(line.split('=')[1])
        elif line.startswith('rev steps='):
            rev = int(line.split('=')[1])
        elif line.startswith('done steps='):
            done = int(line.split('=')[1])
    assert fwd is not None, \
        f'Pico did not print "fwd steps=..."\nstdout: {stdout}'
    assert rev is not None, \
        f'Pico did not print "rev steps=..."\nstdout: {stdout}'
    assert done is not None, \
        f'Pico did not print "done steps=..."\nstdout: {stdout}'

    # Waypoint sanity checks.
    assert abs(fwd - 2880) <= 1, \
        f'After moveTo(30), expected ~2880 steps, got {fwd}'
    assert abs(rev) <= 1, \
        f'After moveTo(0), expected ~0 steps, got {rev}'

    # Ground-truth check: Saleae net must match final PulseCounter.
    assert saleae_net == done, (
        f'Saleae net ({saleae_net}) != PulseCounter ({done}) '
        f'after direction changes'
    )
    assert abs(done - 4800) <= 1, \
        f'Expected ~4800 steps after fwd/rev/fwd sequence, got {done}'

    print(
        f'  PASS  test_pulse_counter_direction_change_accuracy  '
        f'(fwd={fwd}, rev={rev}, final={done}, saleae_net={saleae_net})'
    )


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

TESTS = [
    test_pulse_generator,
    test_moveto_pulse_count,
    test_accel_profile,
    test_triangular_profile,
    test_accel_time_profile,
    test_replan_profile,
    test_stop_restart,
    test_multiaxis_sync,
    test_arc_quarter_circle,
    test_pulse_counter_speed_change_accuracy,
    test_pulse_counter_stop_start_accuracy,
    test_pulse_counter_direction_change_accuracy,
]


def main():
    parser = argparse.ArgumentParser(description='Run HIL tests')
    parser.add_argument(
        'patterns', nargs='*', metavar='PATTERN',
        help='Run only tests whose names contain any of these substrings (default: all)',
    )
    parser.add_argument(
        '-l', '--list', action='store_true',
        help='List available test names and exit',
    )
    args = parser.parse_args()

    if args.list:
        for t in TESTS:
            print(t.__name__)
        sys.exit(0)

    if args.patterns:
        tests = [t for t in TESTS if any(p in t.__name__ for p in args.patterns)]
        if not tests:
            names = ', '.join(t.__name__ for t in TESTS)
            print(f'No tests matched. Available tests:\n  {names}')
            sys.exit(1)
    else:
        tests = TESTS

    if _CAPTURES_DIR.exists():
        shutil.rmtree(_CAPTURES_DIR)
    _CAPTURES_DIR.mkdir()

    deploy()

    print('Connecting to Logic 2...')
    with saleae.Manager.connect(port=hil_config.LOGIC2_PORT) as manager:
        passed = failed = 0
        for test in tests:
            print(f'\n{test.__name__}')
            try:
                test(manager)
                passed += 1
            except AssertionError as e:
                print(f'  FAIL  {e}')
                failed += 1
            except Exception as e:
                print(f'  ERROR  {e}')
                failed += 1

    print(f'\n{passed} passed, {failed} failed')
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
