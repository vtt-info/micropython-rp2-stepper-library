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

import csv
import subprocess
import sys
import tempfile
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

# Pico-side test helper scripts (deployed to the Pico root).
PICO_TEST_FILES = [
    str(_TESTS_DIR / 'test_config.py'),
    str(_TESTS_DIR / 'hil_moveto.py'),
    str(_TESTS_DIR / 'hil_replan.py'),
]


def _mpremote(*args):
    """Run an mpremote command; return (stdout, returncode)."""
    cmd = ['mpremote', 'connect', hil_config.PICO_PORT] + list(args)
    result = subprocess.run(cmd, capture_output=True, text=True)
    return result.stdout + result.stderr, result.returncode


def _force_step_low():
    """Drive the step pin LOW on the Pico via mpremote exec."""
    _mpremote('exec', f'import machine; machine.Pin({hil_config.STEP_PIN}, machine.Pin.OUT, value=0)')


def deploy():
    """Copy the smartstepper package and HIL scripts to the Pico."""
    print('  Deploying files to Pico...')
    # Deploy the package directory recursively (creates smartstepper/ on the Pico).
    stdout, rc = _mpremote('cp', '-r', str(_PACKAGE_DIR) + '/', ':')
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


def run_capture(manager: saleae.Manager, script: str, channels: list):
    """
    Start a manual Saleae capture, run a Pico script synchronously, then stop.

    Returns (tmpdir Path, mpremote stdout string).
    Raises RuntimeError if the Pico script exits non-zero.
    """
    tmpdir = Path(tempfile.mkdtemp())
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


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

TESTS = [
    test_pulse_generator,
    test_moveto_pulse_count,
    test_accel_profile,
    test_replan_profile,
]


def main():
    deploy()

    print('Connecting to Logic 2...')
    with saleae.Manager.connect(port=hil_config.LOGIC2_PORT) as manager:
        passed = failed = 0
        for test in TESTS:
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
