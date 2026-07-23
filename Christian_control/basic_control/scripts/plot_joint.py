#!/usr/bin/env python3
"""Plot commanded vs measured angle of ONE joint from a move log.

The CSV is written by ./controller during a move (columns in Record.h);
this script uses time_s, cmd_j<N> and meas_j<N>.

Usage:
    python3 plot_joint.py 2       # joint 2, newest CSV in newest runs/ day
    python3 plot_joint.py 2 --csv path/to/log.csv
    python3 plot_joint.py 2 --show                   # also open a window
    python3 plot_joint.py 2 --out j2.png             # choose the PNG name

Offline only: never talks to the robot.
"""

import argparse
import sys
from pathlib import Path

import pandas as pd
import matplotlib
import matplotlib.pyplot as plt

import runlog

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
                        help="run log to read (default: newest CSV in the "
                             "newest <repo>/runs/YYYY-MM-DD/ folder)")
    parser.add_argument("--out", default=None,
                        help="output PNG (default: <log stem>_joint<N>.png "
                             "next to the CSV)")
    parser.add_argument("--show", action="store_true",
                        help="also display the plot in a window")
    args = parser.parse_args()

    if not args.show:
        matplotlib.use("Agg")  # headless: save the PNG without needing a display

    csv_path = Path(args.csv) if args.csv else runlog.find_default_csv()
    print(f"log: {csv_path}")

    t, cmd, meas = load_log(csv_path, args.joint)

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(t, cmd, label="commanded", linewidth=1.2)
    ax.plot(t, meas, label="measured", linewidth=1.2, linestyle="--")
    ax.set_title(f"Joint {args.joint}: commanded vs measured "
                 f"({csv_path.name})")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("joint angle (deg)")
    ax.legend()
    ax.grid(True, alpha=0.4)
    fig.tight_layout()

    out = args.out or csv_path.with_name(
        csv_path.stem + f"_joint{args.joint}.png")  # next to the CSV
    fig.savefig(out, dpi=120)
    print(f"saved {out}")

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
