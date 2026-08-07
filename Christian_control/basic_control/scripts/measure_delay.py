#!/usr/bin/env python3
"""Measure the command-to-motion pipeline delay from a single-step run.

Usage: python3 measure_delay.py [path/to/log.csv]
(no argument: the newest CSV in the newest <repo>/runs/YYYY-MM-DD/ folder)

Run the calibration procedure in README.md ("Measuring command-to-motion
delay") first: hold ≥2 s, type exactly ONE target 2 cm away on one axis,
wait for arrival + ~2 s, Ctrl+C. This script then prints:
  - delay: time from the command step (t_send of the first cycle using the
    new target) until the tool position first moves above the noise floor —
    the pipeline's built-in reaction lag, reported once and separately so it
    never pollutes tracking-error statistics; and
  - settle: time from the step until the position stays within 1 mm of its
    final value.

The script is offline-only (never touches the robot) and REFUSES logs that
don't contain exactly one clean step, because a delay number measured from
a messy run would look authoritative while meaning nothing.
"""

import argparse
import sys
from pathlib import Path

import numpy as np

import runlog
from analyze_run import load, measurement_times, parse_preamble, tracking_mask

# Below 5 mm the step is too small to separate from FK noise reliably.
MIN_STEP_M = 0.005
# Quiet data needed before the step to establish the noise floor.
PRE_WINDOW_S = 1.0
# Motion threshold = this many pre-step noise sigmas ...
NOISE_SIGMA_FACTOR = 5.0
# ... but never below 0.2 mm — the FK jitter floor at standstill.
MIN_THRESHOLD_M = 0.0002
# "Settled" means staying within this of the final value (the spec's 1 mm).
SETTLE_TOL_M = 0.001
# The final value is averaged over this much of the end of the run.
FINAL_WINDOW_S = 0.5
# Thesis-figure resolution.
PDF_DPI = 300


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("csv_path", nargs="?", help="step-run CSV (default: newest)")
    args = ap.parse_args()

    path = Path(args.csv_path) if args.csv_path else runlog.find_default_csv()
    print(f"log: {path}")
    meta = parse_preamble(path)
    cols = load(path)
    has_timestamps = runlog.has_exchange_timestamps(meta, cols)
    if not has_timestamps:
        print("NOTE: old log format — using the per-cycle time_s stamp; "
              "delay resolution is limited to one cycle")

    pd = np.column_stack([cols["pd_x"], cols["pd_y"], cols["pd_z"]])
    p = np.column_stack([cols["p_x"], cols["p_y"], cols["p_z"]])
    t_cmd = cols["t_send_s"] if has_timestamps else cols["time_s"]
    t_meas = measurement_times(cols, has_timestamps)

    # Refuse a log with no Cartesian data, naming the reason. Without this the
    # step detector below sees NaN != 0.0 as True on nearly every row and
    # reports "N target changes found", which reads as an operator mistake
    # rather than as an absent column.
    #
    # Only the target-directed rows are inspected: the 25 takeover-hold rows
    # carry literal 0,0,0 rather than NaN (ControllerStatus defaults
    # p_desired/p_current to Zero, unlike every other dead field), so a
    # whole-column isfinite check is satisfied by rows that measured nothing.
    #
    # This script is obsolete in a second way worth stating: it measures the
    # step response to ONE typed Cartesian target, and that input path was
    # retired in stage 2 (docs/decisions/stage2-joint-trajectory-following.md).
    # The pipe accepts only TRAJ_BEGIN blocks now, so the calibration run its
    # README describes can no longer be performed.
    live = tracking_mask(cols)
    if not np.isfinite(pd[live]).any() or not np.isfinite(p[live]).any():
        sys.exit("REFUSED: pd_*/p_* are not populated in this log.\n"
                 "  The joint-trajectory reference source computes no "
                 "Cartesian pose, so those columns\n"
                 "  are NaN by design. This script also measures a step "
                 "response to a single typed\n"
                 "  Cartesian target, an input path retired in stage 2 — the "
                 "pipe accepts only\n"
                 "  TRAJ_BEGIN blocks. Delay against a joint trajectory is a "
                 "different measurement.")

    # A clean calibration run has exactly one target change.
    changed = np.any(np.diff(pd, axis=0) != 0.0, axis=1)
    change_rows = np.flatnonzero(changed) + 1
    if len(change_rows) == 0:
        sys.exit("REFUSED: the target never changes in this log — run the "
                 "calibration procedure in README.md (one typed step)")
    if len(change_rows) > 1:
        sys.exit(f"REFUSED: {len(change_rows)} target changes found — the "
                 "delay measurement needs exactly one clean step "
                 "(no retargeting, no mid-run typing)")
    step_idx = int(change_rows[0])
    t_step = t_cmd[step_idx]

    step_vec = pd[step_idx] - pd[step_idx - 1]
    step_m = float(np.linalg.norm(step_vec))
    if step_m < MIN_STEP_M:
        sys.exit(f"REFUSED: step is {step_m * 1e3:.1f} mm — below "
                 f"{MIN_STEP_M * 1e3:.0f} mm noise dominates the measurement")
    if t_step - t_cmd[0] < PRE_WINDOW_S:
        sys.exit(f"REFUSED: only {t_step - t_cmd[0]:.2f} s of pre-step data — "
                 f"need {PRE_WINDOW_S:.0f} s of quiet hold to establish the "
                 "noise floor (hold longer before typing the target)")

    axis = int(np.argmax(np.abs(step_vec)))
    axis_name = "xyz"[axis]
    pre = (t_meas >= t_step - PRE_WINDOW_S) & (t_meas < t_step)
    baseline = float(p[pre, axis].mean())
    sigma = float(p[pre, axis].std())
    threshold = max(NOISE_SIGMA_FACTOR * sigma, MIN_THRESHOLD_M)
    print(f"step: {step_m * 1e3:.1f} mm along {axis_name} at t={t_step:.3f} s")
    print(f"noise floor: sigma {sigma * 1e6:.1f} um -> motion threshold "
          f"{threshold * 1e3:.3f} mm")

    post = t_meas >= t_step
    moved = post & (np.abs(p[:, axis] - baseline) > threshold)
    if not moved.any():
        sys.exit("REFUSED: the tool never moved above the noise threshold "
                 "after the step — wrong log, or the arm did not move")
    t_motion = float(t_meas[np.flatnonzero(moved)[0]])
    print(f"\ndelay (command sent -> first motion): "
          f"{(t_motion - t_step) * 1e3:.1f} ms")

    final_win = t_meas >= t_meas[-1] - FINAL_WINDOW_S
    final = float(p[final_win, axis].mean())
    if p[final_win, axis].std() > SETTLE_TOL_M:
        print(f"WARNING: still moving in the last {FINAL_WINDOW_S:.1f} s — "
              "the run ended before settling; the settle time below is a "
              "lower bound")
    unsettled = post & (np.abs(p[:, axis] - final) > SETTLE_TOL_M)
    if unsettled.any():
        t_settle = float(t_meas[np.flatnonzero(unsettled)[-1]])
        print(f"settle (command sent -> within {SETTLE_TOL_M * 1e3:.0f} mm "
              f"of final): {(t_settle - t_step) * 1e3:.1f} ms")
    else:
        print("settle: already within tolerance for the whole post-step log")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\nmatplotlib not installed -> numbers only, no plot")
        return

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(t_cmd, pd[:, axis], "--", lw=1, label="commanded")
    ax.plot(t_meas, p[:, axis], "-", lw=1, label="actual")
    ax.axvline(t_step, color="tab:red", ls=":", lw=1, label="step sent")
    ax.axvline(t_motion, color="tab:green", ls=":", lw=1, label="first motion")
    ax.set_xlabel("time (s)")
    ax.set_ylabel(f"{axis_name} (m)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    ax.set_title(f"{path.stem} — step response along {axis_name}")
    fig.tight_layout()
    out = path.with_name(path.stem + "_step.pdf")
    fig.savefig(out, dpi=PDF_DPI)
    print(f"\nplot saved -> {out}")


if __name__ == "__main__":
    main()
