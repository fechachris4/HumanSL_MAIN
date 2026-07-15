#!/usr/bin/env python3
"""Analyze a move log written by ./controller during a move.

Usage: python3 plot_move.py [move_log_YYYY-MM-DD_HH-MM-SS.csv]
(no argument: the newest move_log_*.csv in the current directory)

Prints per-joint tracking stats (final error, overshoot, lag) and cycle-time
(dt) stats, and — if matplotlib is installed — saves move_log.png with
commanded-vs-measured curves for every joint that moved, plus a dt trace.
"""

import csv
import glob
import sys

import numpy as np

NUM_JOINTS = 7


def newest_move_log():
    """Latest move_log_*.csv here (timestamped names sort chronologically)."""
    logs = sorted(glob.glob("move_log_*.csv"))
    if not logs:
        sys.exit("no move_log_*.csv in this directory; pass a file explicitly")
    return logs[-1]


def load(path):
    with open(path) as f:
        reader = csv.reader(f)
        header = next(reader)
        data = np.array([[float(x) for x in row] for row in reader])
    if data.size == 0:
        sys.exit(f"{path}: no data rows")
    cols = {name: data[:, i] for i, name in enumerate(header)}
    return cols


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else newest_move_log()
    cols = load(path)
    t = cols["time_s"]
    dt = cols["dt_s"][1:]  # first dt spans setup, not a steady cycle

    print(f"{path}: {len(t)} cycles over {t[-1]:.2f} s")

    # --- cycle time -------------------------------------------------------
    print("\ndt (cycle period):")
    print(f"  mean {dt.mean()*1e3:.3f} ms   median {np.median(dt)*1e3:.3f} ms"
          f"   p99 {np.percentile(dt, 99)*1e3:.3f} ms   max {dt.max()*1e3:.3f} ms")
    late = (dt > 1.5 * np.median(dt)).sum()
    print(f"  cycles >1.5x median: {late} ({100.0*late/len(dt):.2f}%)")

    # --- per-joint tracking ----------------------------------------------
    print("\nper-joint tracking (deg):")
    print(f"  {'joint':>5} {'delta':>8} {'target':>9} {'final err':>10}"
          f" {'overshoot':>10} {'max |err|':>10}")
    moved = []
    for j in range(1, NUM_JOINTS + 1):
        cmd, meas = cols[f"cmd_j{j}"], cols[f"meas_j{j}"]
        target = cmd[-1]                     # command ends ON the target
        delta = target - cmd[0]
        err = meas - cmd
        if abs(delta) < 1e-9:
            continue                         # joint was holding; skip
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
              "Install it (e.g. in a venv) to get move_log.png")
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
    axes[-1].plot(t[1:], dt * 1e3, lw=0.6)
    axes[-1].set_ylabel("dt (ms)")
    axes[-1].set_xlabel("time (s)")
    axes[-1].grid(True, alpha=0.3)
    fig.tight_layout()
    out = path.rsplit(".", 1)[0] + ".png"
    fig.savefig(out, dpi=120)
    print(f"\nplot saved -> {out}")


if __name__ == "__main__":
    main()
