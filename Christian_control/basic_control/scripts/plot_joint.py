#!/usr/bin/env python3
"""Plot commanded vs measured angle of ONE joint from a move log.

The CSV is written by ./controller during a move (columns in Record.h);
this script uses time_s, cmd_j<N> and meas_j<N>.

Usage:
    python3 plot_joint.py 2                  # joint 2, newest run_*.csv
    python3 plot_joint.py 2 --csv path/to/run_....csv
    python3 plot_joint.py 2 --show                   # also open a window
    python3 plot_joint.py 2 --out j2.png             # choose the PNG name

Offline only: never talks to the robot.
"""

import argparse
import glob
import os
import sys

import pandas as pd
import matplotlib
import matplotlib.pyplot as plt

TIME_COL = "time_s"


def load_log(path, joint):
    """Read the CSV and return (time, commanded, measured) for one joint."""
    cmd_col, meas_col = f"cmd_j{joint}", f"meas_j{joint}"

    try:
        df = pd.read_csv(path)
    except FileNotFoundError:
        sys.exit(f"error: file not found: {path}")
    except pd.errors.EmptyDataError:
        sys.exit(f"error: {path} is empty")
    except pd.errors.ParserError as e:
        sys.exit(f"error: {path} is not valid CSV: {e}")

    missing = [c for c in (TIME_COL, cmd_col, meas_col) if c not in df.columns]
    if missing:
        sys.exit(f"error: {path} is missing column(s) {missing}; "
                 f"found: {list(df.columns)}")
    if df.empty:
        sys.exit(f"error: {path} has a header but no data rows")

    # Reject non-numeric junk (e.g. a truncated last line) with a clear message.
    cols = df[[TIME_COL, cmd_col, meas_col]].apply(pd.to_numeric, errors="coerce")
    if cols.isna().any().any():
        bad = int(cols.isna().any(axis=1).idxmax()) + 2  # +2: header + 0-based
        sys.exit(f"error: {path} has a non-numeric value around line {bad}")

    return cols[TIME_COL], cols[cmd_col], cols[meas_col]


def main():
    parser = argparse.ArgumentParser(
        description="Plot commanded vs measured angle of one joint over time.")
    parser.add_argument("joint", type=int, choices=range(1, 8),
                        help="joint number, 1..7")
    parser.add_argument("--csv", default=None,
                        help="run log to read (default: newest "
                             "run_*.csv in the current directory)")
    parser.add_argument("--out", default=None,
                        help="output PNG (default: joint<N>.png)")
    parser.add_argument("--show", action="store_true",
                        help="also display the plot in a window")
    args = parser.parse_args()

    if not args.show:
        matplotlib.use("Agg")  # headless: save the PNG without needing a display

    if args.csv is None:
        # Timestamped names sort chronologically, so the last one is newest.
        logs = sorted(glob.glob("run_*.csv"))
        if not logs:
            sys.exit("no run_*.csv in this directory; use --csv")
        args.csv = logs[-1]

    t, cmd, meas = load_log(args.csv, args.joint)

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(t, cmd, label="commanded", linewidth=1.2)
    ax.plot(t, meas, label="measured", linewidth=1.2, linestyle="--")
    ax.set_title(f"Joint {args.joint}: commanded vs measured "
                 f"({os.path.basename(args.csv)})")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("joint angle (deg)")
    ax.legend()
    ax.grid(True, alpha=0.4)
    fig.tight_layout()

    out = args.out or f"joint{args.joint}.png"
    fig.savefig(out, dpi=120)
    print(f"saved {out}")

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
