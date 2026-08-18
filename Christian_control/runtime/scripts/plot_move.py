#!/usr/bin/env python3
"""Analyze a run log written by ./controller.

Usage: python3 plot_move.py [path/to/log.csv] [--allow-old]
(no argument: the newest CSV in the newest <repo>/runs/YYYY-MM-DD/ folder;
--allow-old accepts pre-preamble recordings whose config is not embedded)

Prints per-joint tracking stats (final error, overshoot, lag) and cycle-time
(dt) stats, and — if matplotlib is installed — saves <log stem>_move.png
next to the CSV, with commanded-vs-measured curves for every joint that
moved, plus a dt trace.
"""

import csv
import numpy as np
import sys
from pathlib import Path

import runlog

NUM_JOINTS = 7


def load(path, allow_old):
    with open(path) as f:
        first = f.readline()
        if first.startswith("#"):
            # Self-describing run log: skip the '#' config preamble.
            while first.startswith("#"):
                first = f.readline()
        elif not allow_old:
            sys.exit(
                f"{path}: old-format CSV (no '#' config preamble — recorded "
                "before the self-describing-log change). The run's gains and "
                "thresholds are unknown; pass --allow-old to analyze anyway.")
        header = first.rstrip("\n").split(",")
        reader = csv.reader(f)
        # Short rows are skipped, not fatal: a run killed mid-drain leaves a
        # complete file up to its last full row (analyze_run.py does the
        # same).
        data = np.array([[float(x) for x in row]
                         for row in reader if len(row) == len(header)])
    if data.size == 0:
        sys.exit(f"{path}: no data rows")
    cols = {name: data[:, i] for i, name in enumerate(header)}
    return cols


def main():
    args = sys.argv[1:]
    allow_old = "--allow-old" in args
    args = [a for a in args if a != "--allow-old"]
    path = Path(args[0]) if args else runlog.find_default_csv()
    print(f"log: {path}")
    cols = load(path, allow_old)
    t = cols["time_s"]
    dt = cols["dt_s"][1:]  # first dt spans setup, not a steady cycle

    print(f"{len(t)} cycles over {t[-1]:.2f} s")

    # --- cycle time -------------------------------------------------------
    print("\ndt (cycle period):")
    if len(dt) > 0:
        print(f"  mean {dt.mean() * 1e3:.3f} ms   median {np.median(dt) * 1e3:.3f} ms"
              f"   p99 {np.percentile(dt, 99) * 1e3:.3f} ms   max {dt.max() * 1e3:.3f} ms")
        late = (dt > 1.5 * np.median(dt)).sum()
        print(f"  cycles >1.5x median: {late} ({100.0 * late / len(dt):.2f}%)")
    else:
        print("  (not enough cycles for dt stats)")

    # --- per-joint tracking ----------------------------------------------
    print("\nper-joint tracking (deg):")
    print(f"  {'joint':>5} {'delta':>8} {'target':>9} {'final err':>10}"
          f" {'overshoot':>10} {'max |err|':>10}")
    moved = []
    for j in range(1, NUM_JOINTS + 1):
        cmd, meas = cols[f"cmd_j{j}"], cols[f"meas_j{j}"]
        target = cmd[-1]  # command ends ON the target
        delta = target - cmd[0]
        err = meas - cmd
        if abs(delta) < 1e-9:
            continue  # joint was holding; skip
        moved.append(j)
        # overshoot: how far the measurement went PAST the target, in the
        # direction of travel (0 if it never crossed it)
        direction = np.sign(delta)
        overshoot = max(0.0, ((meas - target) * direction).max())
        final_err = meas[-1] - target
        print(f"  {j:>5} {delta:>8.2f} {target:>9.2f} {final_err:>10.3f}"
              f" {overshoot:>10.3f} {np.abs(err).max():>10.3f}")
    if not moved:
        print("  (no joint moved)")

    # --- plots (optional) -------------------------------------------------
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\nmatplotlib not installed -> stats only, no plot. "
              "Install it (e.g. in a venv) to get the _move.png")
        return

    n = len(moved)
    fig, axes = plt.subplots(n + 1, 1, figsize=(10, 3 * (n + 1)), sharex=True)
    axes = np.atleast_1d(axes)
    for ax, j in zip(axes, moved):
        ax.plot(t, cols[f"cmd_j{j}"], label="commanded", lw=1)
        ax.plot(t, cols[f"meas_j{j}"], label="measured", lw=1)
        ax.axhline(cols[f"cmd_j{j}"][-1], color="gray", ls=":", lw=0.8)
        ax.set_ylabel(f"j{j} (deg)")
        ax.legend(loc="best")
        ax.grid(True, alpha=0.3)
    if n > 0:
        axes[-1].plot(t[1:], dt * 1e3, lw=0.6)
        axes[-1].set_ylabel("dt (ms)")
        axes[-1].set_xlabel("time (s)")
        axes[-1].grid(True, alpha=0.3)
    else:
        axes[0].plot(t[1:], dt * 1e3, lw=0.6)
        axes[0].set_ylabel("dt (ms)")
        axes[0].set_xlabel("time (s)")
        axes[0].grid(True, alpha=0.3)
    fig.tight_layout()
    out = path.with_name(path.stem + "_move.png")  # next to the CSV
    fig.savefig(out, dpi=120)
    print(f"\nplot saved -> {out}")


if __name__ == "__main__":
    main()
