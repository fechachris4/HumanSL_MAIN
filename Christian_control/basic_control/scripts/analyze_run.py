#!/usr/bin/env python3
"""Tracking analysis for a run log written by ./controller.

Usage: python3 analyze_run.py [path/to/log.csv] [--force]
(no argument: the newest CSV in the newest <repo>/runs/YYYY-MM-DD/ folder)

Order of business:
  1. Log-integrity report: timestamp gaps, overruns, an estimate of dropped
     cycles. If more than DROP_FRACTION_LIMIT of the cycles look missing the
     script REFUSES to compute statistics (a log with holes silently biases
     every average) unless --force is given.
  2. Commanded and actual position are matched BY TIMESTAMP interpolation,
     never by row index (row pairing bakes the pipeline delay into every
     error number as a fake "always behind" offset).
  3. Command-to-feedback lag estimated by cross-correlating commanded and
     actual position along the dominant motion axis. NOTE: under closed-loop
     P-control this measures the closed-loop response lag, not the transport
     delay — measure_delay.py measures the latter from a dedicated step run.
  4. Plots (300 dpi PDF, saved next to the CSV): <stem>_tracking.pdf with
     stacked x/y/z (commanded dashed, actual solid) and <stem>_error.pdf
     with the error norm plus clamp/fault/refresh-failure ticks.
  5. Printed summary: RMS and max position error, raw AND lag-compensated
     (labelled), and the measured lag in ms.

Timestamp semantics (hardware/Record.h is the authority): commands take
effect at t_send_s; the FK position p_* in row i comes from the feedback
received at row i-1's t_recv_s. Old logs (no "log_format = 2" preamble
line) fall back to time_s for both signals, with a warning.
"""

import argparse
import csv
import re
import sys
from pathlib import Path

import numpy as np

import runlog

# Above this fraction of inferred-missing cycles the averages are computed
# over a log with holes — refuse unless --force.
DROP_FRACTION_LIMIT = 0.01
# A cycle counts as an overrun above this multiple of the run's own median
# dt (matches the controller's kOverrunFactor; never trust a nominal rate).
OVERRUN_FACTOR = 1.5
# Cross-correlation search window: generous vs any plausible pipeline lag,
# tiny vs run length, and it bounds the O(N*K) correlation cost.
MAX_LAG_S = 0.25
# Below 1 mm of commanded-motion spread the lag estimate is fitting noise.
MIN_MOTION_STD_M = 0.001
# Thesis-figure resolution.
PDF_DPI = 300


def parse_preamble(path):
    """Read the leading '#' lines into a {key: value} dict."""
    meta = {}
    with open(path) as f:
        for line in f:
            if not line.startswith("#"):
                break
            m = re.match(r"#\s*([A-Za-z_]\w*)\s*=\s*(.*?)(?:\s*\([^)]*\))?\s*$",
                         line)
            if m:
                meta[m.group(1)] = m.group(2)
    return meta


def load(path):
    """Columns by name (numpy arrays), skipping the '#' preamble."""
    with open(path) as f:
        first = f.readline()
        if not first:
            sys.exit(f"{path}: empty file")
        while first.startswith("#"):
            first = f.readline()
        if not first:
            sys.exit(f"{path}: preamble only, no data")
        header = first.rstrip("\n").split(",")
        reader = csv.reader(f)
        data = np.array([[float(x) for x in row]
                         for row in reader if len(row) == len(header)])
    if data.size == 0:
        sys.exit(f"{path}: no data rows")
    return {name: data[:, i] for i, name in enumerate(header)}


def integrity_report(cols):
    """Print gap/overrun/flag statistics; return the dropped-cycle fraction."""
    t = cols["time_s"]
    dt = cols["dt_s"][1:]  # first dt spans setup, not a steady cycle
    print(f"integrity: {len(t)} samples over {t[-1] - t[0]:.2f} s")
    if len(dt) == 0:
        print("  (not enough cycles for dt statistics)")
        return 0.0

    med = np.median(dt)
    print(f"  dt mean {dt.mean() * 1e3:.3f} ms   median {med * 1e3:.3f} ms"
          f"   p95 {np.percentile(dt, 95) * 1e3:.3f} ms"
          f"   p99 {np.percentile(dt, 99) * 1e3:.3f} ms"
          f"   max {dt.max() * 1e3:.3f} ms")

    overruns = int((dt > OVERRUN_FACTOR * med).sum())
    print(f"  overruns (> {OVERRUN_FACTOR}x median): {overruns}"
          f" ({100.0 * overruns / len(dt):.2f}%)")

    # Each gap of k median periods hides k-1 missing samples.
    dropped = int(np.sum(np.maximum(0, np.round(dt / med) - 1)))
    frac = dropped / (len(t) + dropped)
    print(f"  inferred dropped cycles: {dropped} ({100.0 * frac:.2f}%)")

    if t[0] > 5 * med:
        print(f"  WARNING: log starts at t={t[0]:.2f} s — the ring buffer "
              "wrapped and the beginning of the run was overwritten")

    bad_refresh = int((cols["refresh_ok"] == 0).sum())
    if bad_refresh:
        print(f"  refresh failures: {bad_refresh} row(s)")
    if "pd_beyond_reach" in cols:
        beyond = int((cols["pd_beyond_reach"] != 0).sum())
        if beyond:
            print(f"  target beyond reach sphere: {beyond} row(s)")
    faults = np.zeros(len(t), dtype=bool)
    for j in range(1, 8):
        faults |= cols[f"fault_j{j}"] != 0
    faults |= cols["base_fault"] != 0
    if faults.any():
        print(f"  rows with a nonzero fault bank: {int(faults.sum())}")
    return frac


def measurement_times(cols, new_format):
    """When each row's p_* position was actually sampled.

    New format: the FK position in row i is computed from the feedback
    received in row i-1, so its sample time is row i-1's t_recv_s (row 0's
    feedback predates the loop; its cycle-start stamp is the best available).
    Old format: only time_s exists.
    """
    t = cols["time_s"]
    if not new_format:
        return t
    t_meas = np.empty_like(t)
    t_meas[0] = t[0]
    t_meas[1:] = cols["t_recv_s"][:-1]
    return t_meas


def estimate_lag_s(t_cmd, pd, p_interp, median_dt):
    """Lag (s) that best aligns actual onto commanded.

    Positive = actual lags the command. Picks the shift along the dominant
    motion axis that MINIMIZES the mean-squared difference (a normalized
    cross-correlation peak is too flat for slow trajectories and its
    finite-window bias then picks a wrong shift; the squared-error minimum
    is exactly zero at the true lag for a purely delayed signal, and is the
    same criterion the lag-compensated error report uses). Returns None
    when the run has too little motion to align against.
    """
    axis = int(np.argmax(pd.max(axis=0) - pd.min(axis=0)))
    if pd[:, axis].std() < MIN_MOTION_STD_M:
        return None
    a = pd[:, axis]
    b = p_interp[:, axis]
    max_shift = min(int(MAX_LAG_S / median_dt), len(a) - 2)
    best_shift, best_err = 0, np.inf
    for k in range(-max_shift, max_shift + 1):
        # p(t) ≈ pd(t - lag): a positive shift k compares a[i] with b[i+k].
        if k >= 0:
            err = float(np.mean((a[: len(a) - k] - b[k:]) ** 2))
        else:
            err = float(np.mean((a[-k:] - b[: len(b) + k]) ** 2))
        if err < best_err:
            best_shift, best_err = k, err
    return best_shift * median_dt


def rising_edges(mask):
    """Indices where a boolean mask turns on (marks event starts, not rows)."""
    m = np.asarray(mask, dtype=bool)
    return np.flatnonzero(m & ~np.roll(m, 1) | (np.arange(len(m)) == 0) & m)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("csv_path", nargs="?", help="run CSV (default: newest)")
    ap.add_argument("--force", action="store_true",
                    help="compute statistics even if >1%% of cycles look dropped")
    args = ap.parse_args()

    path = Path(args.csv_path) if args.csv_path else runlog.find_default_csv()
    print(f"log: {path}")
    meta = parse_preamble(path)
    cols = load(path)
    new_format = meta.get("log_format") == "2" and "t_send_s" in cols
    if not new_format:
        print("NOTE: old log format — no t_send_s/t_recv_s columns; using the "
              "per-cycle time_s stamp for both command and feedback timelines")

    frac = integrity_report(cols)
    if frac > DROP_FRACTION_LIMIT and not args.force:
        sys.exit(f"\nREFUSING to compute statistics: {100.0 * frac:.2f}% of "
                 f"cycles look dropped (> {100.0 * DROP_FRACTION_LIMIT:.0f}% "
                 "limit) — averages over a log with holes mislead. "
                 "Rerun with --force to override.")

    pd = np.column_stack([cols["pd_x"], cols["pd_y"], cols["pd_z"]])
    p = np.column_stack([cols["p_x"], cols["p_y"], cols["p_z"]])
    t_cmd = cols["t_send_s"] if new_format else cols["time_s"]
    t_meas = measurement_times(cols, new_format)
    median_dt = float(np.median(cols["dt_s"][1:])) if len(cols["dt_s"]) > 1 \
        else float(cols["dt_s"][0])

    # Actual position at each command time — matching by clock, never by row.
    p_interp = np.column_stack(
        [np.interp(t_cmd, t_meas, p[:, i]) for i in range(3)])
    err_raw = pd - p_interp
    err_raw_norm = np.linalg.norm(err_raw, axis=1)

    lag_s = estimate_lag_s(t_cmd, pd, p_interp, median_dt)
    if lag_s is None:
        print("\nlag: insufficient commanded motion (< 1 mm spread) — "
              "not estimated; lag-compensated errors skipped")
        err_comp_norm = None
    else:
        if abs(lag_s) >= MAX_LAG_S - median_dt:
            print(f"\nWARNING: lag estimate hit the ±{MAX_LAG_S * 1e3:.0f} ms "
                  "search-window edge — the correlation peak is outside the "
                  "window or there is none (e.g. an unreachable target); "
                  "treat the lag and the compensated errors as meaningless")
        p_lagged = np.column_stack(
            [np.interp(t_cmd + lag_s, t_meas, p[:, i]) for i in range(3)])
        err_comp_norm = np.linalg.norm(pd - p_lagged, axis=1)

    print("\nsummary:")
    print(f"  RMS position error (raw):             "
          f"{np.sqrt(np.mean(err_raw_norm ** 2)) * 1e3:.2f} mm")
    print(f"  max position error (raw):             "
          f"{err_raw_norm.max() * 1e3:.2f} mm")
    if err_comp_norm is not None:
        print(f"  measured lag (closed-loop response):  {lag_s * 1e3:.1f} ms")
        print(f"  RMS position error (lag-compensated): "
              f"{np.sqrt(np.mean(err_comp_norm ** 2)) * 1e3:.2f} mm")
        print(f"  max position error (lag-compensated): "
              f"{err_comp_norm.max() * 1e3:.2f} mm")
        print("  (lag-compensated = actual re-sampled at t_send + lag; "
              "separates delay from true tracking error)")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\nmatplotlib not installed -> stats only, no plots")
        return

    # --- tracking: x/y/z stacked, one axis per direction so a weak axis
    # --- can't hide inside a combined norm.
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for i, (ax, name) in enumerate(zip(axes, "xyz")):
        ax.plot(t_cmd, pd[:, i], "--", lw=1, label="commanded")
        ax.plot(t_meas, p[:, i], "-", lw=1, label="actual")
        ax.set_ylabel(f"{name} (m)")
        ax.grid(True, alpha=0.3)
    axes[0].legend(loc="best")
    axes[0].set_title(f"{path.stem} — commanded vs actual (base frame)")
    axes[-1].set_xlabel("time (s)")
    fig.tight_layout()
    out = path.with_name(path.stem + "_tracking.pdf")
    fig.savefig(out, dpi=PDF_DPI)
    print(f"\nplot saved -> {out}")

    # --- error norm with event ticks, so controller performance and fault
    # --- handling can be told apart.
    fig2, ax = plt.subplots(figsize=(10, 4))
    ax.plot(t_cmd, err_raw_norm * 1e3, lw=1, label="|error| (raw)")
    if err_comp_norm is not None:
        ax.plot(t_cmd, err_comp_norm * 1e3, lw=1, alpha=0.7,
                label=f"|error| (lag-compensated, {lag_s * 1e3:.0f} ms)")
    events = []
    if "pd_beyond_reach" in cols:
        events.append((cols["pd_beyond_reach"] != 0, "target beyond reach",
                       "tab:orange"))
    faults = np.zeros(len(t_cmd), dtype=bool)
    for j in range(1, 8):
        faults |= cols[f"fault_j{j}"] != 0
    faults |= cols["base_fault"] != 0
    events.append((faults, "fault bank nonzero", "tab:red"))
    events.append((cols["refresh_ok"] == 0, "refresh failed", "tab:purple"))
    for mask, label, color in events:
        for n, idx in enumerate(rising_edges(mask)):
            ax.axvline(t_cmd[idx], color=color, ls=":", lw=1,
                       label=label if n == 0 else None)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("position error norm (mm)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    ax.set_title(f"{path.stem} — tracking error")
    fig2.tight_layout()
    out2 = path.with_name(path.stem + "_error.pdf")
    fig2.savefig(out2, dpi=PDF_DPI)
    print(f"plot saved -> {out2}")


if __name__ == "__main__":
    main()
