#!/usr/bin/env python3
"""Plot Vicon health and the world-frame hold, from a log_format 10/11 CSV.

One offline plotting tool for the vicon_*/hold_* columns Hardware.cpp
writes (Hardware.h's log_format comment is the schema authority). It
answers three questions the telemetry exists for:

    1. Is the Vicon measurement trustworthy — how stale, how often
       occluded, per segment?
    2. What did the world hold actually do — when did it engage, freeze,
       re-anchor, or latch off, and how big was its world-frame error?
    3. Did the end-effector actually stay still IN THE ROOM? This is
       computed offline (world_T_base(t) = the logged Mount pose composed
       with the fixed mount->base geometry from
       config/dual_arm_mounting.yaml), independently of anything the
       controller's own hold logic decided — the anti-gaming rule in
       CLAUDE.md: a claim of success needs evidence the implementation
       did not produce.

    scripts/plot_world_hold.py [RUN.csv] [-o OUTDIR]

With no CSV it picks the newest run under runs/ (scripts/runlog.py).

A log written before the world-hold slice (log_format < 10) has no
vicon_* columns at all; before log_format 11 it has no hold_* columns.
Each figure notes what it could not compute rather than failing.
"""

import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent))
from runlog import find_default_csv

# Categorical hues, distinct roles from plot_run.py's requested/sent/measured
# triad (this script draws different quantities) but the same colourblind-
# safe families and the same rule: colour is never the only encoding.
C_WORLD = "#CC79A7"   # world-frame / Vicon-derived quantities
C_BASE = "#0072B2"    # base-frame quantities (matches plot_run.py C_SENT)
C_MOUNT = "#E69F00"   # the moving base itself
C_INK = "#1a1a1a"
C_MUTED = "#6b6b6b"
C_GRID = "#d8d8d4"
C_BAD = "#D55E00"     # invalid / stale / diverged
C_GOOD = "#009E73"    # engaged / valid / trustworthy
SURFACE = "#fcfcfb"

STYLE = {
    "figure.facecolor": SURFACE,
    "axes.facecolor": SURFACE,
    "axes.edgecolor": C_GRID,
    "axes.labelcolor": C_INK,
    "axes.titlecolor": C_INK,
    "text.color": C_INK,
    "xtick.color": C_MUTED,
    "ytick.color": C_MUTED,
    "grid.color": C_GRID,
    "grid.linewidth": 0.6,
    "axes.grid": True,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "font.size": 8,
    "legend.frameon": False,
}

SEGMENTS = (
    ("mount", "Mount"), ("leftbase", "LeftBase"), ("rightbase", "RightBase"),
    ("leftee", "LeftEE"), ("rightee", "RightEE"),
)

HOLD_STATE_NAMES = {0: "inactive", 1: "engaged", 2: "frozen", 3: "latched-off"}
HOLD_STATE_COLORS = {0: C_MUTED, 1: C_GOOD, 2: C_MOUNT, 3: C_BAD}

# config/dual_arm_mounting.yaml, pinned against tests/test_dual_arm_mounting.cpp
# (right base: y=-sep/2, Rx(+tilt); left base: y=+sep/2, Rx(-tilt)). Read the
# YAML at runtime rather than trusting this copy silently drifting from it.
_MOUNTING_YAML = (
    Path(__file__).resolve().parents[1] / "config" / "dual_arm_mounting.yaml"
)


def read_mounting_constants():
    separation = tilt = None
    for line in _MOUNTING_YAML.read_text().splitlines():
        body = line.split("#", 1)[0].strip()
        if body.startswith("base_separation_m:"):
            separation = float(body.split(":", 1)[1])
        elif body.startswith("mount_tilt_rad:"):
            tilt = float(body.split(":", 1)[1])
    if separation is None or tilt is None:
        raise SystemExit(f"{_MOUNTING_YAML}: base_separation_m / mount_tilt_rad not found")
    return separation, tilt


def mount_T_base(arm):
    """Fixed mount->base transform (position m, 3x3 rotation) for "left" or
    "right", matching Kinematics.cpp's mount_from_{right,left}_base_."""
    separation, tilt = read_mounting_constants()
    sign = -1.0 if arm == "right" else 1.0
    signed_tilt = tilt if arm == "right" else -tilt
    position = np.array([0.0, sign * separation / 2.0, 0.0])
    c, s = math.cos(signed_tilt), math.sin(signed_tilt)
    rotation = np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
    return position, rotation


def quat_to_matrix(x, y, z, w):
    """Quaternion (x,y,z,w) -> 3x3 rotation matrix. Identity on a
    degenerate (all-zero / non-finite) quaternion rather than raising —
    callers already gate on validity before calling this."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-9:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


# --- CSV / preamble helpers (self-contained, matching plot_run.py's — kept
# a separate small copy rather than importing plot_run as a module, the
# convention plot_joint.py and plot_move.py already follow). ---

def read_run(path):
    preamble, data_lines = [], []
    with open(path, newline="") as handle:
        for line in handle:
            (preamble if line.startswith("#") else data_lines).append(line)
    if not data_lines:
        raise SystemExit(f"{path}: no data rows")
    rows = list(csv.DictReader(data_lines))
    return [p.rstrip("\n") for p in preamble], rows


def preamble_value(preamble, key):
    for line in preamble:
        body = line.lstrip("#").strip()
        if body.startswith(key):
            _, _, value = body.partition("=")
            return value.strip()
    return None


def is_synthetic(preamble):
    if (preamble_value(preamble, "data_source") or "").lower() == "synthetic":
        return True
    return any("SYNTHETIC" in line.upper() for line in preamble)


def col(rows, name):
    out = []
    for row in rows:
        raw = row.get(name)
        if raw is None or raw == "":
            out.append(float("nan"))
            continue
        try:
            out.append(float(raw))
        except ValueError:
            out.append(float("nan"))
    return out


def has_col(rows, name):
    return name in rows[0]


def finite(values):
    return [v for v in values if not math.isnan(v)]


def stamp(fig, path, preamble, synthetic, exit_note):
    fig.text(0.005, 0.004, f"source: {Path(path).name}", ha="left",
             va="bottom", fontsize=6.5, color=C_MUTED)
    if exit_note:
        fig.text(0.995, 0.004, exit_note, ha="right", va="bottom",
                 fontsize=6.5, color=C_MUTED)
    if synthetic:
        fig.text(0.5, 0.5, "SYNTHETIC — NOT ROBOT DATA", ha="center",
                 va="center", fontsize=34, color="#d94801", alpha=0.13,
                 rotation=27, zorder=0, weight="bold")


def spans_by_value(t, values):
    """Yield (value, start_t, end_t) for each contiguous run of a value."""
    if not values:
        return
    start_i = 0
    for i in range(1, len(values) + 1):
        if i == len(values) or values[i] != values[start_i]:
            end_t = t[i] if i < len(values) else t[-1]
            yield values[start_i], t[start_i], end_t
            start_i = i


def rising_int_edges(values):
    """Indices where an integer counter strictly increased vs the row before."""
    out, prev = [], None
    for i, v in enumerate(values):
        if prev is not None and not math.isnan(v) and v > prev:
            out.append(i)
        if not math.isnan(v):
            prev = v
    return out


def empty_figure(path, preamble, synthetic, exit_note, message, title):
    fig = plt.figure(figsize=(11, 4))
    ax = fig.add_subplot(111)
    ax.text(0.5, 0.5, message, ha="center", va="center", fontsize=11,
            color=C_BAD, wrap=True)
    ax.set_axis_off()
    fig.suptitle(title, fontsize=10, weight="bold")
    stamp(fig, path, preamble, synthetic, exit_note)
    return fig


# --- Figure 1: Vicon health ---

def fig_vicon_health(rows, t, path, preamble, synthetic, exit_note,
                     fresh_max_age_s):
    if not has_col(rows, "vicon_age_s"):
        return empty_figure(
            path, preamble, synthetic, exit_note,
            "No vicon_* columns — this log predates the world-frame "
            "slice (log_format < 10). Re-run the controller to get them.",
            "Vicon health")

    fig, axes = plt.subplots(2, 1, figsize=(11, 6.5), sharex=True,
                             gridspec_kw={"height_ratios": [1.6, 2]})

    ax = axes[0]
    age_ms = [v * 1e3 for v in col(rows, "vicon_age_s")]
    ax.plot(t, age_ms, color=C_WORLD, lw=0.8)
    if fresh_max_age_s is not None:
        ax.axhline(fresh_max_age_s * 1e3, color=C_BAD, lw=1.1,
                   ls=(0, (4, 3)),
                   label=f"freshness threshold ({fresh_max_age_s * 1e3:.0f} ms)")
        ax.legend(fontsize=7.5, loc="upper right")
    seen = finite(age_ms)
    if seen:
        seen_sorted = sorted(seen)
        p50 = seen_sorted[len(seen_sorted) // 2]
        p99 = seen_sorted[min(int(0.99 * len(seen_sorted)), len(seen_sorted) - 1)]
        ax.set_title(f"Vicon sample age — median {p50:.1f} ms, p99 {p99:.1f} ms, "
                    f"max {max(seen_sorted):.1f} ms, "
                    f"{100 * len(seen) / len(age_ms):.1f}% of rows ever saw a sample",
                    fontsize=9, loc="left", weight="bold")
    else:
        ax.set_title("Vicon sample age — no sample EVER arrived this run",
                     fontsize=9, loc="left", weight="bold", color=C_BAD)
    ax.set_ylabel("age (ms)")

    ax = axes[1]
    for i, (key, label) in enumerate(SEGMENTS):
        ax.broken_barh([(t[0], t[-1] - t[0])], (i - 0.4, 0.8),
                       facecolors=C_GOOD, alpha=0.18, zorder=0)
        if not has_col(rows, f"vicon_{key}_valid"):
            continue
        valid = col(rows, f"vicon_{key}_valid")
        spans = []
        gap_start = None
        for j in range(len(valid) + 1):
            bad = j < len(valid) and (math.isnan(valid[j]) or valid[j] < 0.5)
            if bad and gap_start is None:
                gap_start = t[j]
            elif not bad and gap_start is not None:
                end_t = t[j] if j < len(t) else t[-1]
                spans.append((gap_start, end_t - gap_start))
                gap_start = None
        ax.broken_barh(spans, (i - 0.4, 0.8), facecolors=C_BAD)
    ax.set_yticks(range(len(SEGMENTS)))
    ax.set_yticklabels([label for _, label in SEGMENTS])
    ax.set_ylim(-0.6, len(SEGMENTS) - 0.4)
    ax.set_xlabel("time_s")
    ax.set_title("Segment validity — red = occluded or invalid this sample",
                fontsize=9, loc="left", weight="bold")

    fig.suptitle("Vicon health", fontsize=11, weight="bold")
    fig.tight_layout(rect=(0, 0.02, 1, 0.95))
    stamp(fig, path, preamble, synthetic, exit_note)
    return fig


# --- Figure 2: world-hold state and error ---

def fig_hold_state_and_error(rows, t, path, preamble, synthetic, exit_note,
                             max_error_m, max_rot_error_rad):
    if not has_col(rows, "hold_state"):
        return empty_figure(
            path, preamble, synthetic, exit_note,
            "No hold_* columns — this log predates the world-hold slice "
            "(log_format < 11). Re-run the controller to get them.",
            "World-hold state and error")

    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True,
                             gridspec_kw={"height_ratios": [1, 2, 2, 1.3]})

    ax = axes[0]
    state = [int(v) if not math.isnan(v) else 0 for v in col(rows, "hold_state")]
    for value, start_t, end_t in spans_by_value(t, state):
        ax.broken_barh([(start_t, max(end_t - start_t, 1e-6))], (-0.5, 1),
                       facecolors=HOLD_STATE_COLORS.get(value, C_MUTED))
    reanchors = rising_int_edges(col(rows, "hold_reanchor_count"))
    for i in reanchors:
        ax.axvline(t[i], color=C_INK, lw=1.0, ls=(0, (1, 1.5)))
    handles = [plt.matplotlib.patches.Patch(color=HOLD_STATE_COLORS[v], label=n)
              for v, n in HOLD_STATE_NAMES.items()]
    if reanchors:
        handles.append(plt.matplotlib.lines.Line2D(
            [], [], color=C_INK, ls=(0, (1, 1.5)),
            label=f"re-anchor ({len(reanchors)})"))
    ax.legend(handles=handles, fontsize=7, loc="upper left", ncol=5)
    ax.set_yticks([])
    ax.set_title("Hold state", fontsize=9, loc="left", weight="bold")

    ax = axes[1]
    err_m = col(rows, "world_err_m")
    ramp = col(rows, "hold_ramp") if has_col(rows, "hold_ramp") else [float("nan")] * len(rows)
    ax.plot(t, err_m, color=C_WORLD, lw=1.1, label="world_err_m (unramped)")
    scaled_threshold = [r * max_error_m if not math.isnan(r) else float("nan")
                        for r in ramp]
    ax.plot(t, scaled_threshold, color=C_BAD, lw=1.0, ls=(0, (4, 3)),
           label=f"latch threshold x ramp ({max_error_m * 1e3:.0f} mm max)")
    ax.legend(fontsize=7.5, loc="best")
    ax.set_ylabel("m")
    ax.set_title("World-frame position error (divergence latch watches ramp x this)",
                fontsize=9, loc="left", weight="bold")

    ax = axes[2]
    err_rot = col(rows, "world_err_rot_rad") if has_col(rows, "world_err_rot_rad") else [float("nan")] * len(rows)
    ax.plot(t, err_rot, color=C_WORLD, lw=1.1, label="world_err_rot_rad (unramped)")
    scaled_rot = [r * max_rot_error_rad if not math.isnan(r) else float("nan")
                 for r in ramp]
    ax.plot(t, scaled_rot, color=C_BAD, lw=1.0, ls=(0, (4, 3)),
           label=f"latch threshold x ramp ({max_rot_error_rad:.2f} rad max)")
    ax.legend(fontsize=7.5, loc="best")
    ax.set_ylabel("rad")
    ax.set_title("World-frame rotation error", fontsize=9, loc="left", weight="bold")

    ax = axes[3]
    ax.plot(t, ramp, color=C_BASE, lw=1.1)
    ax.set_ylim(-0.05, 1.05)
    ax.set_ylabel("ramp (0-1)")
    ax.set_xlabel("time_s")
    ax.set_title("Authority ramp — effective error the law fights is ramp x error",
                fontsize=9, loc="left", weight="bold")

    n_engaged = sum(1 for s in state if s in (1, 2))
    fig.suptitle(f"World-hold state and error — engaged/frozen for "
                f"{100 * n_engaged / len(state):.1f}% of the run, "
                f"{len(reanchors)} re-anchor(s)", fontsize=11, weight="bold")
    fig.tight_layout(rect=(0, 0.02, 1, 0.94))
    stamp(fig, path, preamble, synthetic, exit_note)
    return fig


# --- Figure 3: end-effector position, world frame vs base frame ---

def fig_ee_world(rows, t, path, preamble, synthetic, exit_note, arm):
    if not has_col(rows, "vicon_mount_valid"):
        return empty_figure(
            path, preamble, synthetic, exit_note,
            "No vicon_mount_* columns — this log predates the "
            "world-frame slice (log_format < 10).",
            "End-effector position: world frame vs base frame")

    valid = col(rows, "vicon_mount_valid")
    n_valid = sum(1 for v in valid if v >= 0.5)
    if n_valid == 0:
        return empty_figure(
            path, preamble, synthetic, exit_note,
            "No valid Mount samples this run — the Mount segment was "
            "occluded or Vicon was unreachable for the whole session, "
            "so a world-frame end-effector position cannot be computed.",
            "End-effector position: world frame vs base frame")

    offset, rotation_offset = mount_T_base(arm)
    mx, my, mz = col(rows, "vicon_mount_x_m"), col(rows, "vicon_mount_y_m"), col(rows, "vicon_mount_z_m")
    qx, qy, qz, qw = (col(rows, f"vicon_mount_q{a}") for a in "xyzw")
    px, py, pz = col(rows, "p_x"), col(rows, "p_y"), col(rows, "p_z")
    cycle = col(rows, "cycle")

    # cycle == 0 is the T4 fixed-position hold BEFORE the controller runs at
    # all (Runner.cpp) — p_x/p_y/p_z there are LoopLogSample's uncomputed
    # struct default {0,0,0}, not real FK, and would silently read as "the
    # end-effector was exactly here" if trusted. The joint-tracking hold
    # that runs after (whenever the world hold has not engaged and no
    # Cartesian target is active) skips FK entirely and leaves p_x/y/z
    # blank -> NaN instead, which col() already handles correctly; cycle==0
    # is the one place a REAL zero and a MISSING value look identical.
    mount_world = np.full((len(rows), 3), np.nan)
    ee_world = np.full((len(rows), 3), np.nan)
    for i in range(len(rows)):
        if valid[i] < 0.5:
            continue
        quat_vals = (qx[i], qy[i], qz[i], qw[i])
        if any(math.isnan(v) for v in (mx[i], my[i], mz[i]) + quat_vals):
            continue
        R_world_ms = quat_to_matrix(*quat_vals)
        p_world_ms = np.array([mx[i], my[i], mz[i]])
        mount_world[i] = p_world_ms
        if cycle[i] < 1 or math.isnan(px[i]) or math.isnan(py[i]) or math.isnan(pz[i]):
            continue
        R_world_base = R_world_ms @ rotation_offset
        p_world_base = p_world_ms + R_world_ms @ offset
        ee_world[i] = p_world_base + R_world_base @ np.array([px[i], py[i], pz[i]])

    ee_valid_rows = [i for i in range(len(rows)) if not np.isnan(ee_world[i, 0])]
    if not ee_valid_rows:
        return empty_figure(
            path, preamble, synthetic, exit_note,
            "Mount was valid, but the end-effector's own pose (p_x/p_y/p_z) "
            "was never computed this run — that happens whenever the "
            "controller stays on the joint-tracking hold the whole time "
            "(it skips FK). A world-frame EE position needs at least one "
            "cycle where the pose channel ran: the world hold engaged, or "
            "an explicit Cartesian target was active.",
            "End-effector position: world frame vs base frame")
    ee_anchor = ee_world[ee_valid_rows[0]]
    ee_displacement = np.linalg.norm(ee_world - ee_anchor, axis=1)

    sparse = len(ee_valid_rows) < 200
    fig, axes = plt.subplots(4, 1, figsize=(11, 11), sharex=True,
                             gridspec_kw={"height_ratios": [1, 1, 1, 1.4]})
    axis_labels = ("x", "y", "z")
    for i, ax in enumerate(axes[:3]):
        ax.plot(t, mount_world[:, i], color=C_MOUNT, lw=1.0,
               label="Mount (world)" if i == 0 else None)
        ax.plot(t, ee_world[:, i], color=C_WORLD, lw=1.2,
               marker="o" if sparse else None, markersize=3,
               label="end-effector (world, computed)" if i == 0 else None)
        ax.set_ylabel(f"{axis_labels[i]} (m)")
        if i == 0:
            ax.legend(fontsize=7.5, loc="best")
    axes[0].set_title(
        "Base motion vs end-effector position, both in the WORLD frame — "
        "if the hold works, the top line moves and the bottom stays flat",
        fontsize=9, loc="left", weight="bold")

    ax = axes[3]
    ax.plot(t, ee_displacement * 1e3, color=C_WORLD, lw=1.3, marker="o",
           markersize=2 if len(ee_valid_rows) < 200 else 0)
    n = len(ee_valid_rows)
    if n < 50:
        # A handful of points reads as confident evidence unless the count
        # is right there in the title — the same trap as trusting cycle-0's
        # zero above, one level up.
        ax.set_title(
            f"End-effector displacement from its first valid world reading "
            f"— only n={n} pose-channel row(s) this run; NOT enough to "
            "claim a max/RMS. The world hold (or a Cartesian target) needs "
            "to run for longer than this to produce real evidence.",
            fontsize=9, loc="left", weight="bold", color=C_BAD)
    else:
        ax.set_title(
            f"End-effector displacement from its first valid world reading "
            f"(n={n} pose-channel rows) — "
            f"max {ee_displacement.max() * 1e3:.1f} mm, "
            f"RMS {math.sqrt(np.nanmean(ee_displacement ** 2)) * 1e3:.1f} mm "
            "(independent of hold_state — this is the raw geometry)",
            fontsize=9, loc="left", weight="bold")
    ax.set_ylabel("mm")
    ax.set_xlabel("time_s")

    fig.suptitle(f"End-effector position: world frame vs base frame ({arm} arm)",
                fontsize=11, weight="bold")
    fig.tight_layout(rect=(0, 0.02, 1, 0.95))
    stamp(fig, path, preamble, synthetic, exit_note)
    return fig


# --- Figure 4: world error vs base speed (tests e_ss ~ v_B / Kp) ---

def fig_error_vs_speed(rows, t, path, preamble, synthetic, exit_note):
    needed = ("vicon_mount_valid", "vicon_mount_x_m", "world_err_m", "hold_state")
    if not all(has_col(rows, c) for c in needed):
        return empty_figure(
            path, preamble, synthetic, exit_note,
            "Missing vicon_mount_*/world_err_m/hold_state columns for this "
            "check (log predates the world-hold slice).",
            "World error vs base speed")

    valid = col(rows, "vicon_mount_valid")
    mx, my, mz = col(rows, "vicon_mount_x_m"), col(rows, "vicon_mount_y_m"), col(rows, "vicon_mount_z_m")
    err = col(rows, "world_err_m")
    state = col(rows, "hold_state")

    speeds, errs = [], []
    for i in range(1, len(rows)):
        if valid[i] < 0.5 or valid[i - 1] < 0.5:
            continue
        if state[i] not in (1.0, 2.0):  # engaged or frozen only
            continue
        dt = t[i] - t[i - 1]
        if not (dt > 0) or math.isnan(err[i]):
            continue
        dx, dy, dz = mx[i] - mx[i - 1], my[i] - my[i - 1], mz[i] - mz[i - 1]
        speeds.append(math.sqrt(dx * dx + dy * dy + dz * dz) / dt)
        errs.append(err[i])

    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111)
    if not speeds:
        ax.text(0.5, 0.5,
                "No engaged/frozen rows with a valid Mount sample this run — "
                "nothing to correlate yet. This figure becomes meaningful "
                "once a run engages the hold while the base moves.",
                ha="center", va="center", fontsize=10, color=C_MUTED, wrap=True)
        ax.set_axis_off()
    else:
        ax.scatter(speeds, [e * 1e3 for e in errs], s=6, color=C_WORLD, alpha=0.35,
                  edgecolors="none")
        ax.set_xlabel("base speed (m/s, finite-differenced Mount position)")
        ax.set_ylabel("world_err_m (mm)")
        ax.set_title(
            f"World error vs base speed, engaged/frozen rows only (n={len(speeds)}) — "
            "derivation doc (B5) predicts e_ss ~ v_B / Kp, a roughly linear trend",
            fontsize=9, loc="left", weight="bold")
    fig.tight_layout()
    stamp(fig, path, preamble, synthetic, exit_note)
    return fig


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv", nargs="?", help="run CSV (default: newest under runs/)")
    parser.add_argument("-o", "--outdir", default=None,
                        help="directory for the PNGs (default: beside the CSV)")
    args = parser.parse_args()

    path = Path(args.csv) if args.csv else Path(find_default_csv())
    preamble, rows = read_run(path)
    synthetic = is_synthetic(preamble)

    if "time_s" not in rows[0]:
        raise SystemExit(f"{path}: not a controller run CSV (missing time_s)")
    t = col(rows, "time_s")

    exit_reason = preamble_value(preamble, "exit_reason")
    exit_note = f"exit: {exit_reason}" if exit_reason else "exit: not recorded"

    arm = (preamble_value(preamble, "arm") or "left").split()[0].strip().lower()
    if arm not in ("left", "right"):
        print(f"note: preamble arm={arm!r} unrecognised, defaulting to left",
              file=sys.stderr)
        arm = "left"

    def preamble_float(key, default):
        raw = preamble_value(preamble, key)
        if raw is None:
            return default
        try:
            return float(raw.split()[0])
        except ValueError:
            return default

    fresh_max_age_s = preamble_float("world_hold_fresh_max_age_s", None)
    max_error_m = preamble_float("world_hold_max_error_m", 0.08)
    max_rot_error_rad = preamble_float("world_hold_max_rot_error_rad", 0.5)

    outdir = Path(args.outdir) if args.outdir else path.parent
    outdir.mkdir(parents=True, exist_ok=True)
    prefix = ("SYNTHETIC_" if synthetic else "") + path.stem.replace("SYNTHETIC_", "")

    with plt.rc_context(STYLE):
        figures = [
            ("vicon_health",
             fig_vicon_health(rows, t, path, preamble, synthetic, exit_note,
                              fresh_max_age_s)),
            ("hold_state_and_error",
             fig_hold_state_and_error(rows, t, path, preamble, synthetic,
                                      exit_note, max_error_m, max_rot_error_rad)),
            ("ee_world_vs_base",
             fig_ee_world(rows, t, path, preamble, synthetic, exit_note, arm)),
            ("error_vs_base_speed",
             fig_error_vs_speed(rows, t, path, preamble, synthetic, exit_note)),
        ]
        for name, fig in figures:
            out = outdir / f"{prefix}__{name}.png"
            fig.savefig(out, dpi=140)
            plt.close(fig)
            print(out)

    if synthetic:
        print("\nThese figures are SYNTHETIC. They demonstrate the telemetry "
              "and the plots; they are not evidence about the robot.")


if __name__ == "__main__":
    main()
