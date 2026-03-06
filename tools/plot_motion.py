#!/usr/bin/env python3
"""plot_motion.py — visualize 2-axis motion from a Saleae HIL capture.

Reads the digital.csv exported by tests/test_arc_hil.py (or test_hil.py)
and reconstructs each axis's position over time by replaying rising edges on
the STEP channels with DIR channel polarity.

Requirements: matplotlib  (pip install matplotlib)

Usage:
    python tools/plot_motion.py <capture_dir_or_csv> [options]

    <capture_dir_or_csv>  Path to a directory containing digital.csv, or
                          directly to a digital.csv file.
                          Typically: tests/captures_arc/hil_arc/

Examples:
    # Quarter-circle arc capture, default channel mapping:
    python tools/plot_motion.py tests/captures/hil_arc/ \\
        --steps-per-unit 100 --x-init 0 --y-init 0 \\
        --arc-cx 0 --arc-cy 100 --arc-r 100 --show

    # MultiAxis straight-line capture:
    python tools/plot_motion.py tests/captures/hil_multiaxis/ \\
        --steps-per-unit 96 --show

Options:
    --steps-per-unit F   Steps per unit for both axes (default: 96)
    --x-init F           Initial X position in units (default: 0)
    --y-init F           Initial Y position in units (default: 0)
    --step-ch N          Saleae channel for axis-1 STEP (default: 0)
    --dir-ch N           Saleae channel for axis-1 DIR  (default: 1)
    --step-ch2 N         Saleae channel for axis-2 STEP (default: 2)
    --dir-ch2 N          Saleae channel for axis-2 DIR  (default: 3)
    --dir-high-positive  DIR=1 means positive direction (default: True)
    --dir-high-negative  DIR=1 means negative direction
    --arc-cx F           Arc center X for ideal overlay (optional)
    --arc-cy F           Arc center Y for ideal overlay (optional)
    --arc-r F            Arc radius for ideal overlay   (optional)
    --output FILE        Save figure to FILE (default: motion.png)
    --show               Display interactively instead of saving
"""

import argparse
import csv
import math
import sys
from pathlib import Path

try:
    import matplotlib
    import matplotlib.pyplot as plt
except ImportError:
    print(
        "matplotlib is required.  Install with:  pip install matplotlib",
        file=sys.stderr,
    )
    sys.exit(1)


# ---------------------------------------------------------------------------
# CSV parsing and position reconstruction
# ---------------------------------------------------------------------------

def _load_csv(csv_path: Path):
    """Load digital.csv into a list of row dicts.

    Returns list of {'time': float, 'ch0': int, 'ch1': int, ...} dicts.
    """
    rows = []
    with open(csv_path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            entry = {'time': float(row['Time [s]'])}
            for k, v in row.items():
                if k.startswith('Channel '):
                    try:
                        ch = int(k.split()[1])
                        entry[ch] = int(v)
                    except (IndexError, ValueError):
                        pass
            rows.append(entry)
    return rows


def reconstruct_axis(rows, step_ch, dir_ch, init_pos, steps_per_unit,
                     dir_high_positive=True):
    """Reconstruct position vs time for one axis.

    Returns (times, positions) where each element corresponds to a rising
    edge on step_ch. DIR channel value at the same sample determines sign.

    dir_high_positive: when True, DIR=1 means positive (+1/spu), DIR=0 means
                       negative (-1/spu). Flip for inverted wiring.
    """
    times = []
    positions = []
    pos = init_pos
    prev_step = None

    for row in rows:
        step_val = row.get(step_ch, 0)
        dir_val  = row.get(dir_ch, 0)
        if prev_step == 0 and step_val == 1:
            # Rising edge: move one step in direction indicated by DIR
            if dir_high_positive:
                pos += (1.0 / steps_per_unit) if dir_val else -(1.0 / steps_per_unit)
            else:
                pos += -(1.0 / steps_per_unit) if dir_val else (1.0 / steps_per_unit)
            times.append(row['time'])
            positions.append(pos)
        prev_step = step_val

    return times, positions


def find_segment_boundaries(times, gap_ratio=5.0):
    """Return indices where the inter-step gap exceeds gap_ratio × median gap.

    These mark the boundaries between chord segments.
    """
    if len(times) < 3:
        return []
    gaps = [times[i + 1] - times[i] for i in range(len(times) - 1)]
    median_gap = sorted(gaps)[len(gaps) // 2]
    threshold = gap_ratio * median_gap
    return [i + 1 for i, g in enumerate(gaps) if g > threshold]


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_motion(args):
    # Locate digital.csv
    path = Path(args.capture)
    if path.is_dir():
        csv_path = path / 'digital.csv'
    else:
        csv_path = path
    if not csv_path.exists():
        print(f"Error: {csv_path} not found", file=sys.stderr)
        sys.exit(1)

    print(f"Reading {csv_path} ...")
    rows = _load_csv(csv_path)
    print(f"  {len(rows)} samples")

    times_x, pos_x = reconstruct_axis(
        rows, args.step_ch, args.dir_ch,
        args.x_init, args.steps_per_unit,
        dir_high_positive=args.dir_high_positive,
    )
    times_y, pos_y = reconstruct_axis(
        rows, args.step_ch2, args.dir_ch2,
        args.y_init, args.steps_per_unit,
        dir_high_positive=args.dir_high_positive,
    )
    print(f"  Axis X: {len(times_x)} steps, "
          f"range [{min(pos_x, default=args.x_init):.2f}, "
          f"{max(pos_x, default=args.x_init):.2f}]")
    print(f"  Axis Y: {len(times_y)} steps, "
          f"range [{min(pos_y, default=args.y_init):.2f}, "
          f"{max(pos_y, default=args.y_init):.2f}]")

    seg_x = find_segment_boundaries(times_x)
    seg_y = find_segment_boundaries(times_y)
    print(f"  Detected {len(seg_x)} X segment boundaries, "
          f"{len(seg_y)} Y segment boundaries")

    fig, (ax_traj, ax_time) = plt.subplots(1, 2, figsize=(14, 6))
    title = f"2-axis motion — {csv_path.parent.name}"
    fig.suptitle(title, fontsize=12)

    # ------------------------------------------------------------------ #
    # Left: Spatial trajectory (Y vs X)                                   #
    # ------------------------------------------------------------------ #

    ax_traj.set_title("Spatial trajectory (X–Y plane)")
    ax_traj.set_xlabel("X position (units)")
    ax_traj.set_ylabel("Y position (units)")
    ax_traj.set_aspect('equal')
    ax_traj.grid(True, alpha=0.3)

    # Combine step events from both axes into a merged timeline.
    # Build a dict: time → (x, y) snapshot by interleaving both channels.
    # Simple approach: sample Y at each X time step and vice-versa via
    # nearest-neighbor, then build a combined sorted list.
    if times_x and times_y:
        # Build index for quick Y lookup at any X time
        def interpolate_pos(times_src, pos_src, t_query, init):
            """Return position at time t_query (hold-last-value semantics)."""
            if not times_src or t_query < times_src[0]:
                return init
            lo, hi = 0, len(times_src) - 1
            while lo < hi:
                mid = (lo + hi + 1) // 2
                if times_src[mid] <= t_query:
                    lo = mid
                else:
                    hi = mid - 1
            return pos_src[lo]

        # Sample the spatial path at every step event on either axis
        all_times = sorted(set(times_x + times_y))
        traj_x = [interpolate_pos(times_x, pos_x, t, args.x_init) for t in all_times]
        traj_y = [interpolate_pos(times_y, pos_y, t, args.y_init) for t in all_times]

        ax_traj.plot(traj_x, traj_y, 'b-', linewidth=1.0, alpha=0.8, label='Path')

        # Mark segment boundaries (X axis boundaries as vertical marks)
        for idx in seg_x:
            if idx < len(all_times):
                bx = interpolate_pos(times_x, pos_x, all_times[idx], args.x_init)
                by = interpolate_pos(times_y, pos_y, all_times[idx], args.y_init)
                ax_traj.plot(bx, by, 'k|', markersize=10, markeredgewidth=1.5)

        # Start and end markers
        ax_traj.plot(args.x_init, args.y_init, 'go', markersize=10,
                     label=f'Start ({args.x_init:.1f}, {args.y_init:.1f})')
        ax_traj.plot(traj_x[-1], traj_y[-1], 'rs', markersize=10,
                     label=f'End ({traj_x[-1]:.1f}, {traj_y[-1]:.1f})')

    # Optional ideal arc overlay
    has_arc = (args.arc_cx is not None
               and args.arc_cy is not None
               and args.arc_r is not None)
    if has_arc:
        cx, cy, r = args.arc_cx, args.arc_cy, args.arc_r
        theta = [i * math.pi / 180 for i in range(361)]
        arc_x = [cx + r * math.cos(t) for t in theta]
        arc_y = [cy + r * math.sin(t) for t in theta]
        ax_traj.plot(arc_x, arc_y, 'r--', linewidth=1.0, alpha=0.5,
                     label=f'Ideal arc r={r}')
        ax_traj.plot(cx, cy, 'r+', markersize=10, markeredgewidth=1.5,
                     label=f'Center ({cx:.1f}, {cy:.1f})')

        # Radial error at each reconstructed point
        if times_x and times_y:
            errors = [abs(math.sqrt((x - cx)**2 + (y - cy)**2) - r)
                      for x, y in zip(traj_x, traj_y)]
            max_err = max(errors) if errors else 0.0
            mean_err = sum(errors) / len(errors) if errors else 0.0
            print(f"  Radial error: max={max_err:.4f}, mean={mean_err:.4f} units")
            ax_traj.set_title(
                f"Spatial trajectory — radial error max={max_err:.3f} mean={mean_err:.3f}"
            )

    ax_traj.legend(loc='best', fontsize=8)

    # ------------------------------------------------------------------ #
    # Right: Axis positions vs time                                        #
    # ------------------------------------------------------------------ #

    ax_time.set_title("Axis positions vs time")
    ax_time.set_xlabel("Time (s)")
    ax_time.set_ylabel("Position (units)")
    ax_time.grid(True, alpha=0.3)

    if times_x:
        ax_time.step([0.0] + times_x, [args.x_init] + pos_x,
                     where='post', color='#3498db', label='X', linewidth=1.5)
    if times_y:
        ax_time.step([0.0] + times_y, [args.y_init] + pos_y,
                     where='post', color='#e74c3c', label='Y', linewidth=1.5,
                     linestyle='--')

    # Mark segment boundaries from X axis as vertical dashed lines
    for idx in seg_x:
        if idx < len(times_x):
            ax_time.axvline(times_x[idx], color='gray', linewidth=0.8,
                            linestyle=':', alpha=0.8)

    ax_time.legend(loc='best', fontsize=8)

    plt.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(
        description="Plot 2-axis motion from a Saleae HIL capture (digital.csv).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument('capture', metavar='CAPTURE',
                   help="Path to capture directory (containing digital.csv) or "
                        "directly to a digital.csv file.")
    p.add_argument('--steps-per-unit', type=float, default=96, metavar='F',
                   help="Steps per unit for both axes")
    p.add_argument('--x-init', type=float, default=0.0, metavar='F',
                   help="Initial X position in units")
    p.add_argument('--y-init', type=float, default=0.0, metavar='F',
                   help="Initial Y position in units")
    p.add_argument('--step-ch', type=int, default=0, metavar='N',
                   help="Saleae channel for axis-1 STEP")
    p.add_argument('--dir-ch', type=int, default=1, metavar='N',
                   help="Saleae channel for axis-1 DIR")
    p.add_argument('--step-ch2', type=int, default=2, metavar='N',
                   help="Saleae channel for axis-2 STEP")
    p.add_argument('--dir-ch2', type=int, default=3, metavar='N',
                   help="Saleae channel for axis-2 DIR")

    dir_grp = p.add_mutually_exclusive_group()
    dir_grp.add_argument('--dir-high-positive', dest='dir_high_positive',
                         action='store_true', default=True,
                         help="DIR=1 means positive direction (default)")
    dir_grp.add_argument('--dir-high-negative', dest='dir_high_positive',
                         action='store_false',
                         help="DIR=1 means negative direction")

    p.add_argument('--arc-cx', type=float, default=None, metavar='F',
                   help="Arc center X for ideal arc overlay")
    p.add_argument('--arc-cy', type=float, default=None, metavar='F',
                   help="Arc center Y for ideal arc overlay")
    p.add_argument('--arc-r', type=float, default=None, metavar='F',
                   help="Arc radius for ideal arc overlay")
    p.add_argument('--output', default='motion.png', metavar='FILE',
                   help="Output filename (ignored when --show is used)")
    p.add_argument('--show', action='store_true',
                   help="Display interactively instead of saving")
    return p.parse_args()


def main():
    args = parse_args()

    if not args.show:
        matplotlib.use('Agg')

    fig = plot_motion(args)

    if args.show:
        plt.show()
    else:
        fig.savefig(args.output, dpi=150, bbox_inches='tight')
        print(f"Saved {args.output}")


if __name__ == '__main__':
    main()
