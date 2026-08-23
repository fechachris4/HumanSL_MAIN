#!/usr/bin/env python3
"""Plot one recorded planner request and its available controller telemetry.

    scripts/plot_plan.py DEBUG_DIR [CART_TRAJ_FILE] [-o OUTDIR] [options]

DEBUG_DIR is a directory written by `planner_bridge --debug-dir`. It holds
plain CSVs (PlanDebugDump.h is the schema authority):

    meta.csv          arm, plan kind, status, and the same summary values
                      the terminal PLAN SUMMARY block printed
    joint_limits.csv  the band each joint was solved against, degrees,
                      with the planner's margin already applied
    joints.csv        the dense joint trajectory, degrees and deg/s
    path_ik.csv       traced paths only: one row per path sample — index,
                      progress, requested position, status (solved /
                      interpolated seed / no convergence / joint limits),
                      residuals, limit margin and configuration
    candidate_attempts.csv
                      bounded terminal-IK summaries and GPMP2 route attempts,
                      including serialized validation and selection evidence

CART_TRAJ_FILE is optional; cart_traj.txt in DEBUG_DIR is used by default.
With --controller-log, measured TCP, q and qdot are read only from recorded
telemetry columns. Mount-frame requests and obstacle geometry are overlaid on
world-frame trajectories only when the recorded transform is explicitly
identity.

Output: individual PNGs plus one plan_report.html. One path per line on stdout
remains the panel's discovery convention.

Everything drawn is a value the planner itself produced; this script
arranges, it does not recompute planning decisions.
"""

import argparse
import collections
import csv
import html
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 — registers "3d"

NUM_JOINTS = 7

# A sample is FAILED only when its anchor solve found nothing. Interpolated
# samples are the walk's normal state (anchor initialization, 2026-08-21) —
# drawing them as failures would make every healthy plan look broken.
FAILED_STATUSES = ("no_convergence", "joint_limits", "failed")
EXECUTABLE_PLAN_STATUSES = ("REACHED", "GOAL_BLOCKED")

C_PLANNED = "#0072B2"
C_MEASURED = "#E69F00"
C_REQUESTED = "#6b6b6b"
C_INVALID = "#D55E00"
C_HARD = "#9b2226"
C_PREFERRED = "#6c757d"
C_SELECTED = "#009E73"
C_GRID = "#d8d8d4"
C_SURFACE = "#fcfcfb"

plt.rcParams.update({
    "figure.facecolor": C_SURFACE,
    "axes.facecolor": C_SURFACE,
    "axes.edgecolor": C_GRID,
    "axes.grid": True,
    "grid.color": C_GRID,
    "grid.linewidth": 0.6,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "legend.frameon": False,
})


def plan_is_executable(status):
    return status in EXECUTABLE_PLAN_STATUSES


def is_failed(row):
    return row.get("status", "") in FAILED_STATUSES


def read_csv(path):
    """Rows as dicts, or None if the file is absent (a point plan has no
    path_ik.csv, and a failed plan has no joints.csv)."""
    if not path.exists():
        return None
    with path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    return rows or None


def read_meta(path):
    rows = read_csv(path)
    if rows is None:
        return {}
    return {row["key"]: row["value"] for row in rows}


def col(rows, name, default=float("nan")):
    """One column as floats. An empty field becomes `default` rather than
    zero — a gap in the data must never read as a genuine standstill."""
    values = []
    for row in rows:
        text = row.get(name, "")
        values.append(float(text) if text not in ("", None) else default)
    return values


def read_cart_traj(path):
    """The CART_TRAJ block: (t, x, y, z, speed). The wire format is
    WorldCartesianTrajectoryWire.cpp — 14 numbers then an eligibility flag,
    between CART_TRAJ_BEGIN and CART_TRAJ_END."""
    t, x, y, z, speed = [], [], [], [], []
    collecting = False
    for line in path.read_text().splitlines():
        token = line.split()
        if not token:
            continue
        if token[0] == "CART_TRAJ_BEGIN":
            collecting = True
            continue
        if token[0] == "CART_TRAJ_END":
            break
        if not collecting or len(token) != 15:
            continue
        values = [float(v) for v in token[:14]]
        t.append(values[0])
        x.append(values[1])
        y.append(values[2])
        z.append(values[3])
        speed.append(math.sqrt(values[8] ** 2 + values[9] ** 2 + values[10] ** 2))
    return t, x, y, z, speed


def read_controller_log(path):
    """Read only recorded telemetry. No FK or derived robot state is
    constructed here: measured TCP, q and qdot come from named CSV columns."""
    if path is None or not path.exists():
        return None
    with path.open(newline="") as handle:
        rows = list(csv.DictReader(line for line in handle
                                   if not line.startswith("#")))
    if not rows:
        return None
    required = ["time_s", "cart_replan_requested", "world_fresh"]
    required += [f"meas_j{joint}" for joint in range(1, NUM_JOINTS + 1)]
    required += [f"vel_j{joint}" for joint in range(1, NUM_JOINTS + 1)]
    required += ["cart_meas_x_world_m", "cart_meas_y_world_m",
                 "cart_meas_z_world_m", "cart_meas_vx_world_mps",
                 "cart_meas_vy_world_mps", "cart_meas_vz_world_mps"]
    missing = [name for name in required if name not in rows[0]]
    if missing:
        raise SystemExit(f"{path}: controller log lacks {', '.join(missing)}")
    absolute_time = col(rows, "time_s")
    edges = [index for index, row in enumerate(rows)
             if row.get("cart_replan_requested", "0") == "1"]
    origin = absolute_time[edges[-1]] if edges else absolute_time[0]
    relative_time = [value - origin for value in absolute_time]
    return {"path": path, "rows": rows, "time_s": relative_time,
            "has_replan_edge": bool(edges)}


def finite_number(text, default=float("nan")):
    try:
        value = float(text)
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def fresh_world_value(row, name):
    return (finite_number(row.get(name)) if row.get("world_fresh") == "1"
            else float("nan"))


def read_plot_config(path):
    if path is None or not path.exists():
        return {"minimum_clearance_m": None,
                "preferred_clearance_m": None,
                "maximum_planning_error_m": None,
                "maximum_orientation_error_rad": None, "scene": []}
    try:
        document = yaml.safe_load(path.read_text()) or {}
        obstacles = document.get("obstacles", {})
        scene = []
        for object_id, item in (obstacles.get("scene", {}) or {}).items():
            if not item.get("enabled", False):
                continue
            shape = item.get("shape")
            if shape not in ("box", "cylinder"):
                continue
            scene.append({"id": object_id, **item})
        return {
            "minimum_clearance_m": finite_number(
                obstacles.get("minimum_clearance_m"), None),
            "preferred_clearance_m": finite_number(
                obstacles.get("preferred_clearance_m"), None),
            "maximum_planning_error_m": finite_number(
                (document.get("path_following", {}) or {}).get(
                    "maximum_planning_error_m"), None),
            "maximum_orientation_error_rad": finite_number(
                (document.get("path_following", {}) or {}).get(
                    "maximum_orientation_error_rad"), None),
            "scene": scene,
        }
    except (OSError, yaml.YAMLError, TypeError, ValueError) as error:
        raise SystemExit(f"{path}: cannot read planner display geometry: {error}")


def controller_window(controller, planned_end_s=None):
    if controller is None:
        return [], []
    times = controller["time_s"]
    if planned_end_s is None:
        lower, upper = -0.5, 2.0
    else:
        lower, upper = -0.5, planned_end_s + 0.5
    picked = [index for index, value in enumerate(times)
              if lower <= value <= upper]
    if not picked:
        picked = list(range(len(times)))
    return picked, [times[index] for index in picked]


def draw_scene(axis, scene):
    """Display authored primitives only. This is geometry rendering, not a
    distance query and not a second collision calculation."""
    for item in scene:
        centre = np.asarray(item.get("center_mount_m", [0.0, 0.0, 0.0]),
                            dtype=float)
        if item["shape"] == "box":
            half_extent = np.asarray(
                item.get("half_extent_m", [0.0, 0.0, 0.0]), dtype=float)
            size = 2.0 * half_extent
            lower = centre - half_extent
            axis.bar3d(lower[0], lower[1], lower[2], size[0], size[1], size[2],
                       color=C_INVALID, alpha=0.16, edgecolor=C_INVALID,
                       linewidth=0.5, shade=False)
        else:
            radius = float(item.get("radius_m", 0.0))
            height = float(item.get("height_m", 0.0))
            theta = np.linspace(0.0, 2.0 * math.pi, 36)
            z = np.linspace(centre[2] - 0.5 * height,
                            centre[2] + 0.5 * height, 2)
            theta_grid, z_grid = np.meshgrid(theta, z)
            x_grid = centre[0] + radius * np.cos(theta_grid)
            y_grid = centre[1] + radius * np.sin(theta_grid)
            axis.plot_surface(x_grid, y_grid, z_grid, color=C_INVALID,
                              alpha=0.13, linewidth=0, shade=False)
        axis.text(centre[0], centre[1], centre[2], item["id"],
                  color=C_INVALID, fontsize=7)


def title_for(meta, what):
    arm = meta.get("arm", "?")
    kind = meta.get("plan_kind", "?")
    status = meta.get("status", "?")
    state = "FAILED" if not plan_is_executable(status) else status
    head = f"{what} — {arm} arm, {kind} plan, {state}"
    if not plan_is_executable(status):
        head += f"\n{status}: {meta.get('failure_reason', '')}"
    return head


def plot_cartesian(meta, cart, path_ik, controller, scene, requested_point,
                   frames_coincident, outpath):
    """Requested, planned and measured TCP evidence.

    The planner wire is world-frame; requested path and scene geometry are
    Mount-frame. They are overlaid only when the recorded transform is
    explicitly identity. Measured TCP is read directly from telemetry."""
    requested = None
    if path_ik:
        requested = (col(path_ik, "t_s"), col(path_ik, "target_x_m"),
                     col(path_ik, "target_y_m"), col(path_ik, "target_z_m"))
    elif requested_point is not None:
        requested = ([0.0], [requested_point[0]], [requested_point[1]],
                     [requested_point[2]])

    measured = None
    if controller is not None:
        planned_end = cart[0][-1] if cart is not None and cart[0] else None
        picked, measured_t = controller_window(controller, planned_end)
        rows = controller["rows"]
        measured = (
            measured_t,
            [fresh_world_value(rows[index], "cart_meas_x_world_m")
             for index in picked],
            [fresh_world_value(rows[index], "cart_meas_y_world_m")
             for index in picked],
            [fresh_world_value(rows[index], "cart_meas_z_world_m")
             for index in picked],
        )

    if cart is None and requested is None and measured is None:
        return None

    figure = plt.figure(figsize=(13, 5.8))
    axis3d = figure.add_subplot(1, 3, 1, projection="3d")
    if frames_coincident and requested is not None:
        _, req_x, req_y, req_z = requested
        if len(req_x) == 1:
            axis3d.scatter(req_x, req_y, req_z, color=C_REQUESTED, marker="x",
                           s=55, label="requested terminal")
        else:
            axis3d.plot(req_x, req_y, req_z, color=C_REQUESTED, linewidth=1.2,
                        linestyle="--", label="requested TCP")
    if cart is not None:
        _, plan_x, plan_y, plan_z, _ = cart
        axis3d.plot(plan_x, plan_y, plan_z, color=C_PLANNED, linewidth=1.5,
                    label="planned TCP")
    if measured is not None:
        _, meas_x, meas_y, meas_z = measured
        axis3d.plot(meas_x, meas_y, meas_z, color=C_MEASURED, linewidth=1.2,
                    linestyle=(0, (2, 2)),
                    label="measured TCP telemetry")
    if frames_coincident:
        draw_scene(axis3d, scene)
    if path_ik:
        bad = [i for i, row in enumerate(path_ik) if is_failed(row)]
        if bad:
            axis3d.scatter([col(path_ik, "target_x_m")[i] for i in bad],
                           [col(path_ik, "target_y_m")[i] for i in bad],
                           [col(path_ik, "target_z_m")[i] for i in bad],
                           color=C_INVALID, marker="x", s=44,
                           label="IK failed samples")
    axis3d.set_xlabel("x [m]")
    axis3d.set_ylabel("y [m]")
    axis3d.set_zlabel("z [m]")
    axis3d.set_title("TCP path and authored obstacle geometry")
    axis3d.legend(loc="upper left", fontsize=7)

    axis_pos = figure.add_subplot(1, 3, 2)
    if cart is not None:
        plan_t, plan_x, plan_y, plan_z, _ = cart
        for values, name in ((plan_x, "x"), (plan_y, "y"), (plan_z, "z")):
            axis_pos.plot(plan_t, values, linewidth=1.2, label=f"planned {name}")
    if measured is not None:
        meas_t, meas_x, meas_y, meas_z = measured
        for values, name in ((meas_x, "x"), (meas_y, "y"), (meas_z, "z")):
            axis_pos.plot(meas_t, values, linewidth=0.9,
                          linestyle=(0, (2, 2)), label=f"measured {name}")
    if cart is None:
        axis_pos.text(0.5, 0.92, "planned series unavailable: no emitted trajectory",
                      transform=axis_pos.transAxes, ha="center", va="top",
                      fontsize=8, color=C_INVALID)
    if measured is None:
        axis_pos.text(0.5, 0.82, "measured series unavailable: no controller log",
                      transform=axis_pos.transAxes, ha="center", va="top",
                      fontsize=8, color=C_REQUESTED)
    axis_pos.set_xlabel("time from planning request [s]")
    axis_pos.set_ylabel("position [m]")
    axis_pos.set_title("world TCP position vs time")
    if axis_pos.get_legend_handles_labels()[0]:
        axis_pos.legend(fontsize=6, ncol=2)

    axis_speed = figure.add_subplot(1, 3, 3)
    if cart is not None:
        plan_t, _, _, _, plan_speed = cart
        axis_speed.plot(plan_t, plan_speed, color=C_PLANNED, linewidth=1.2,
                        label="planned")
    if controller is not None:
        planned_end = cart[0][-1] if cart is not None and cart[0] else None
        picked, measured_t = controller_window(controller, planned_end)
        rows = controller["rows"]
        speed = []
        for index in picked:
            components = [fresh_world_value(rows[index], name) for name in
                          ("cart_meas_vx_world_mps", "cart_meas_vy_world_mps",
                           "cart_meas_vz_world_mps")]
            speed.append(math.sqrt(sum(value * value for value in components)))
        axis_speed.plot(measured_t, speed, color=C_MEASURED, linewidth=1.0,
                        linestyle=(0, (2, 2)), label="measured TCP telemetry")
    if cart is None and controller is None:
        axis_speed.text(0.5, 0.5, "no planned or measured speed series",
                        ha="center", va="center", transform=axis_speed.transAxes)
    axis_speed.set_xlabel("time from planning request [s]")
    axis_speed.set_ylabel("speed [m/s]")
    axis_speed.set_title("world TCP speed")
    if axis_speed.get_legend_handles_labels()[0]:
        axis_speed.legend(fontsize=7)

    if not frames_coincident and (requested is not None or scene):
        figure.text(0.5, 0.01,
                    "Mount-frame request/scene not overlaid on world data: "
                    "world_T_mount was not recorded as identity.",
                    ha="center", fontsize=8, color=C_INVALID)
    figure.suptitle(title_for(meta, "Cartesian"), fontsize=10)
    figure.tight_layout(rect=(0, 0.04, 1, 0.90))
    figure.savefig(outpath, dpi=130)
    plt.close(figure)
    return outpath


def plot_joints(meta, joints, limits, controller, outpath):
    """Planned and measured q/qdot, aligned to the recorded replan edge."""
    if joints is None and controller is None:
        return None

    planned_t = col(joints, "t_s") if joints else []
    planned_end = planned_t[-1] if planned_t else None
    picked, measured_t = controller_window(controller, planned_end)
    measured_rows = controller["rows"] if controller else []
    figure, axes = plt.subplots(NUM_JOINTS, 2, figsize=(14, 15), sharex=True)
    position_mismatch, velocity_mismatch = [], []
    for index in range(NUM_JOINTS):
        position_axis, velocity_axis = axes[index]
        joint = index + 1
        if joints:
            planned_q = col(joints, f"q{joint}_deg")
            planned_qdot = col(joints, f"qd{joint}_deg_s")
            position_axis.plot(planned_t, planned_q, color=C_PLANNED,
                               linewidth=1.2, label="planned")
            velocity_axis.plot(planned_t, planned_qdot, color=C_PLANNED,
                               linewidth=1.2, label="planned")
        if controller:
            measured_q = [finite_number(measured_rows[row][f"meas_j{joint}"])
                          for row in picked]
            measured_qdot = [finite_number(measured_rows[row][f"vel_j{joint}"])
                             for row in picked]
            position_axis.plot(measured_t, measured_q, color=C_MEASURED,
                               linewidth=1.0, linestyle=(0, (2, 2)),
                               label="measured")
            velocity_axis.plot(measured_t, measured_qdot, color=C_MEASURED,
                               linewidth=1.0, linestyle=(0, (2, 2)),
                               label="measured")
            if joints and measured_t:
                start_row = min(range(len(measured_t)),
                                key=lambda row: abs(measured_t[row]))
                position_mismatch.append(abs(planned_q[0] - measured_q[start_row]))
                velocity_mismatch.append(abs(planned_qdot[0] -
                                             measured_qdot[start_row]))
        if limits is not None:
            lower = float(limits[index]["lower_deg"])
            upper = float(limits[index]["upper_deg"])
            if abs(lower) < 1e6 and abs(upper) < 1e6:
                position_axis.axhline(lower, color=C_HARD, linewidth=0.7,
                                     linestyle="--")
                position_axis.axhline(upper, color=C_HARD, linewidth=0.7,
                                     linestyle="--")
        position_axis.set_ylabel(f"q{joint} [deg]")
        velocity_axis.set_ylabel(f"qdot{joint} [deg/s]")
    axes[0][0].set_title("joint position")
    axes[0][1].set_title("joint velocity")
    axes[0][0].legend(fontsize=7, loc="best")
    axes[0][1].legend(fontsize=7, loc="best")
    axes[-1][0].set_xlabel("time from planning request [s]")
    axes[-1][1].set_xlabel("time from planning request [s]")

    notes = []
    if joints is None:
        notes.append("planned series unavailable: no executable trajectory")
    if controller is None:
        notes.append("measured series unavailable: no controller log")
    if position_mismatch:
        notes.append(
            f"start max |planned-measured|: {max(position_mismatch):.4f} deg; "
            f"qdot: {max(velocity_mismatch):.4f} deg/s")
    if controller is not None and not controller["has_replan_edge"]:
        notes.append("controller time aligned to first row: no replan edge recorded")
    subtitle = title_for(meta, "planned/measured joint state")
    if notes:
        subtitle += "\n" + " | ".join(notes)
    figure.suptitle(subtitle, fontsize=10)
    figure.tight_layout(rect=(0, 0, 1, 0.94))
    figure.savefig(outpath, dpi=130)
    plt.close(figure)
    return outpath


def route_attempts(candidates):
    if not candidates:
        return []
    return [row for row in candidates
            if int(finite_number(row.get("duration_attempt"), 0)) > 0]


def plot_clearance(meta, candidates, minimum_clearance_m,
                   preferred_clearance_m, outpath):
    attempts = route_attempts(candidates)
    applicable = [row for row in attempts
                  if row.get("worst_scene_object_id") or
                  row.get("first_scene_violation_object_id")]
    figure, axis = plt.subplots(figsize=(11, 4.8))
    if applicable:
        for ordinal, row in enumerate(applicable):
            value = finite_number(row.get("minimum_scene_clearance_m"))
            terminal = row.get("terminal_kind")
            marker = "o" if terminal == "REACHED" else "s"
            selected = row.get("selected") == "1"
            executable = row.get("executable") == "1"
            axis.scatter(ordinal, value * 1000.0, marker=marker,
                         s=65 if selected else 35,
                         facecolors=C_SELECTED if selected else
                                    (C_PLANNED if executable else "none"),
                         edgecolors=C_SELECTED if selected else
                                    (C_PLANNED if executable else C_INVALID),
                         linewidth=1.2)
        axis.set_xticks(range(len(applicable)))
        axis.set_xticklabels(
            [f"{row.get('terminal_kind', '?').replace('GOAL_BLOCKED', 'short')} "
             f"b{row.get('terminal_branch', '?')}/"
             f"{row.get('route', '?')}/d{row.get('duration_attempt', '?')}"
             for row in applicable], rotation=45, ha="right", fontsize=7)
    else:
        reason = meta.get("failure_reason", "")
        axis.text(0.5, 0.55,
                  "No entered GPMP2 candidate carries scene-clearance evidence.\n"
                  + (f"Request outcome: {reason}" if reason else
                     "The fixture has no prohibited scene pair."),
                  ha="center", va="center", transform=axis.transAxes,
                  color=C_INVALID if reason else C_REQUESTED)
        axis.set_xticks([])
    if minimum_clearance_m is not None:
        axis.axhline(minimum_clearance_m * 1000.0, color=C_HARD,
                    linestyle="--", linewidth=1.2,
                    label=f"hard minimum {minimum_clearance_m * 1000.0:g} mm")
    if preferred_clearance_m is not None:
        axis.axhline(preferred_clearance_m * 1000.0, color=C_PREFERRED,
                    linestyle=(0, (2, 2)), linewidth=1.2,
                    label=f"preferred cap {preferred_clearance_m * 1000.0:g} mm")
    axis.set_ylabel("minimum prohibited-pair clearance [mm]")
    axis.set_xlabel("entered route candidate")
    axis.set_title("Candidate minimum clearance against configured boundaries")
    if minimum_clearance_m is not None or preferred_clearance_m is not None:
        axis.legend(fontsize=8)
    figure.suptitle(title_for(meta, "modelled clearance"), fontsize=10)
    figure.tight_layout(rect=(0, 0, 1, 0.91))
    figure.savefig(outpath, dpi=130)
    plt.close(figure)
    return outpath


def plot_candidates(meta, candidates, outpath):
    attempts = route_attempts(candidates)
    figure, axes = plt.subplots(1, 2, figsize=(13, 5.2))
    latency, disposition = axes
    if attempts:
        labels = []
        for ordinal, row in enumerate(attempts):
            solve_ms = finite_number(row.get("solve_time_s"), 0.0) * 1000.0
            selected = row.get("selected") == "1"
            executable = row.get("executable") == "1"
            colour = C_SELECTED if selected else (C_PLANNED if executable
                                                   else C_INVALID)
            latency.barh(ordinal, solve_ms, color=colour, alpha=0.85)
            labels.append(
                f"{row.get('terminal_kind', '?').replace('GOAL_BLOCKED', 'short')} "
                f"b{row.get('terminal_branch', '?')}/"
                f"{row.get('route', '?')}/d{row.get('duration_attempt', '?')}")
        latency.set_yticks(range(len(labels)))
        latency.set_yticklabels(labels, fontsize=7)
        latency.invert_yaxis()
        counts = collections.Counter(
            "selected" if row.get("selected") == "1" else
            ("validated" if row.get("executable") == "1" else "invalid")
            for row in attempts)
        names = ["selected", "validated", "invalid"]
        disposition.bar(names, [counts[name] for name in names],
                        color=[C_SELECTED, C_PLANNED, C_INVALID])
        for index, name in enumerate(names):
            disposition.text(index, counts[name], str(counts[name]),
                             ha="center", va="bottom", fontsize=8)
    else:
        latency.text(0.5, 0.5, "No GPMP2 route candidate entered",
                     transform=latency.transAxes, ha="center", va="center",
                     color=C_INVALID)
        disposition.text(0.5, 0.5, "0 route attempts",
                         transform=disposition.transAxes, ha="center",
                         va="center")
    latency.set_xlabel("recorded GPMP2 solve time [ms]")
    latency.set_ylabel("candidate")
    latency.set_title("Candidate latency")
    disposition.set_ylabel("candidate count")
    disposition.set_title("Candidate dispositions")
    figure.suptitle(title_for(meta, "bounded candidate search"), fontsize=10)
    figure.tight_layout(rect=(0, 0, 1, 0.92))
    figure.savefig(outpath, dpi=130)
    plt.close(figure)
    return outpath


def plot_path_ik(meta, path_ik, accept_m, accept_rad, outpath):
    """The continuation walk, sample by sample. This is the figure that
    explains a failed traced path: the unsolved samples are marked in red
    on the shape itself, so a failure has a place on the path rather than
    only a count."""
    if path_ik is None:
        return None
    progress = col(path_ik, "progress_pct")
    if all(math.isnan(v) for v in progress):  # dump predates progress column
        progress = list(range(len(path_ik)))
    solved = [row.get("solved", "1") == "1" for row in path_ik]
    status = [row.get("status", "") for row in path_ik]
    interpolated = [row.get("status", "") == "interpolated_seed"
                    for row in path_ik]
    x = col(path_ik, "target_x_m")
    y = col(path_ik, "target_y_m")
    z = col(path_ik, "target_z_m")
    residual_mm = [v * 1000.0 for v in col(path_ik, "position_residual_m")]
    orientation_deg = [math.degrees(v)
                       for v in col(path_ik, "orientation_residual_rad")]
    margin_deg = col(path_ik, "limit_margin_deg")

    good = [i for i, ok in enumerate(solved) if ok]
    bad = [i for i in range(len(path_ik)) if is_failed(path_ik[i])]
    interp = [i for i, flag in enumerate(interpolated) if flag]

    figure = plt.figure(figsize=(13, 8))

    axis3d = figure.add_subplot(2, 2, 1, projection="3d")
    axis3d.plot(x, y, z, color="lightgrey", linewidth=1.0, zorder=1)
    if good:
        axis3d.scatter([x[i] for i in good], [y[i] for i in good],
                       [z[i] for i in good], color="tab:green", s=18,
                       label="IK anchor solved")
    if interp:
        axis3d.scatter([x[i] for i in interp], [y[i] for i in interp],
                       [z[i] for i in interp], color="tab:orange", s=8,
                       label="interpolated")
    if bad:
        axis3d.scatter([x[i] for i in bad], [y[i] for i in bad],
                       [z[i] for i in bad], color="tab:red", s=44,
                       label="anchor FAILED")
    axis3d.set_xlabel("x [m]")
    axis3d.set_ylabel("y [m]")
    axis3d.set_zlabel("z [m]")
    axis3d.set_title("anchor outcomes along the path")
    if axis3d.get_legend_handles_labels()[0]:
        axis3d.legend(loc="upper left", fontsize=8)

    axis_res = figure.add_subplot(2, 2, 2)
    axis_res.plot(progress, residual_mm, color="tab:blue", linewidth=1.0,
                  marker=".", markersize=4, label="position [mm]")
    if bad:
        axis_res.scatter([progress[i] for i in bad],
                         [residual_mm[i] for i in bad],
                         color="tab:red", s=40, zorder=3, label="unsolved")
    if accept_m is not None:
        axis_res.axhline(accept_m * 1000.0, color="tab:red", linestyle="--",
                         linewidth=0.9,
                         label=f"accept {accept_m * 1000.0:.1f} mm")
    axis_res.set_xlabel("path progress [%]")
    axis_res.set_ylabel("position residual [mm]")
    axis_res.set_title("position error vs progress")
    axis_res.grid(alpha=0.3)
    axis_res.legend(fontsize=8)

    axis_ori = figure.add_subplot(2, 2, 3)
    axis_ori.plot(progress, orientation_deg, color="tab:brown", linewidth=1.0,
                  marker=".", markersize=4)
    if bad:
        axis_ori.scatter([progress[i] for i in bad],
                         [orientation_deg[i] for i in bad],
                         color="tab:red", s=40, zorder=3)
    if accept_rad is not None:
        axis_ori.axhline(math.degrees(accept_rad), color="tab:red",
                         linestyle="--", linewidth=0.9,
                         label=f"accept {math.degrees(accept_rad):.1f} deg")
        axis_ori.legend(fontsize=8)
    axis_ori.set_xlabel("path progress [%]")
    axis_ori.set_ylabel("orientation residual [deg]")
    axis_ori.set_title("orientation error vs progress")
    axis_ori.grid(alpha=0.3)

    axis_margin = figure.add_subplot(2, 2, 4)
    if not all(math.isnan(v) for v in margin_deg):
        axis_margin.plot(progress, margin_deg, color="tab:olive",
                         linewidth=1.0, marker=".", markersize=4)
        for i in bad:
            axis_margin.axvline(progress[i], color="tab:red", alpha=0.25,
                                linewidth=2.0)
        axis_margin.axhline(0.0, color="tab:red", linewidth=0.9)
        axis_margin.set_xlabel("path progress [%]")
        axis_margin.set_ylabel("min joint-limit margin [deg]")
        axis_margin.set_title("distance to nearest joint limit "
                              "(red bands = unsolved)")
        axis_margin.grid(alpha=0.3)
    else:
        axis_margin.set_axis_off()

    no_convergence = status.count("no_convergence")
    outside_limits = status.count("joint_limits")
    subtitle = title_for(meta, "path IK continuation walk")
    if bad:
        subtitle += (f"\nfailure reasons: {no_convergence} no-convergence, "
                     f"{outside_limits} converged-only-outside-joint-limits")
    figure.suptitle(subtitle, fontsize=10)
    figure.tight_layout(rect=(0, 0, 1, 0.90))
    figure.savefig(outpath, dpi=130)
    plt.close(figure)
    return outpath


def table_window(path_ik):
    """The rows worth reading closely: every failed sample plus 3 solved
    neighbours each side, or the 7 rows around the worst residual when
    everything solved. Returns (rows, note)."""
    if path_ik is None or not path_ik:
        return [], ""
    bad = [i for i, row in enumerate(path_ik) if is_failed(row)]
    if bad:
        picked = set()
        for index in bad:
            picked.update(range(max(0, index - 3),
                                min(len(path_ik), index + 4)))
        note = "every failed anchor with 3 neighbouring samples each side"
    else:
        residuals = col(path_ik, "position_residual_m")
        worst = max(range(len(residuals)), key=lambda i: residuals[i])
        picked = set(range(max(0, worst - 3), min(len(path_ik), worst + 4)))
        note = f"7 samples around the worst residual (sample {worst})"
    return [path_ik[i] for i in sorted(picked)], note


def write_report(meta, outdir, figures, path_ik):
    """One page holding the summary, the figures and the per-sample table —
    the single thing to open after a planner run."""
    failed = not plan_is_executable(meta.get("status", "?"))
    colour = "#c0392b" if failed else "#1e8449"
    state = "FAILED" if failed else meta.get("status", "?")

    rows, note = table_window(path_ik)
    table_columns = ["sample", "progress_pct", "status", "position_residual_m",
                     "orientation_residual_rad", "limit_margin_deg"] + \
                    [f"q{j}_deg" for j in range(1, NUM_JOINTS + 1)]
    table_headers = ["#", "progress %", "status", "pos err [mm]",
                     "ori err [deg]", "margin [deg]"] + \
                    [f"q{j}" for j in range(1, NUM_JOINTS + 1)]

    def cell(row, column):
        raw = row.get(column, "")
        if raw in ("", None):
            return ""
        if column == "status":
            return raw
        value = float(raw)
        if column == "position_residual_m":
            return f"{value * 1000.0:.2f}"
        if column == "orientation_residual_rad":
            return f"{math.degrees(value):.2f}"
        if column == "sample":
            return f"{int(value)}"
        return f"{value:.1f}"

    parts = [
        "<!doctype html><meta charset='utf-8'>",
        f"<title>plan report — {html.escape(meta.get('arm', '?'))} "
        f"{html.escape(meta.get('plan_kind', '?'))}</title>",
        "<style>body{font-family:sans-serif;margin:1.5em;max-width:1300px}"
        "table{border-collapse:collapse;font-size:0.85em}"
        "td,th{border:1px solid #ccc;padding:2px 8px;text-align:right}"
        "th{background:#f4f4f4}tr.failed{background:#fdecea}"
        "dt{font-weight:bold}dd{margin:0 0 0.4em 0}"
        "img{max-width:100%;border:1px solid #eee;margin:0.5em 0}</style>",
        f"<h1 style='color:{colour}'>Plan {state} "
        f"({html.escape(meta.get('arm', '?'))} arm, "
        f"{html.escape(meta.get('plan_kind', '?'))} plan)</h1>",
    ]
    parts.append("<dl>")
    for key, value in meta.items():
        if key in ("arm", "plan_kind"):
            continue
        parts.append(f"<dt>{html.escape(key)}</dt>"
                     f"<dd>{html.escape(value)}</dd>")
    parts.append("</dl>")

    for figure in figures:
        parts.append(f"<img src='{figure.name}' alt='{figure.stem}'>")

    if rows:
        parts.append(f"<h2>Per-sample detail — {html.escape(note)}</h2>")
        parts.append("<table><tr>" +
                     "".join(f"<th>{h}</th>" for h in table_headers) + "</tr>")
        for row in rows:
            css = " class='failed'" if is_failed(row) else ""
            parts.append(f"<tr{css}>" +
                         "".join(f"<td>{html.escape(cell(row, c))}</td>"
                                 for c in table_columns) + "</tr>")
        parts.append("</table>")

    outpath = outdir / "plan_report.html"
    outpath.write_text("\n".join(parts))
    return outpath


def requested_point_from_candidates(candidates):
    """Recover the authored point target from recorded evidence, if present."""
    if not candidates:
        return None
    requested = [row for row in candidates
                 if row.get("target_source") == "requested" and
                 abs(finite_number(row.get("target_fraction"), 0.0) - 1.0) < 1e-12]
    if not requested:
        return None
    row = requested[0]
    values = [finite_number(row.get(name)) for name in
              ("target_x_mount_m", "target_y_mount_m", "target_z_mount_m")]
    return None if any(math.isnan(value) for value in values) else values


def find_planner_config(debug_dir):
    for candidate in (debug_dir / "planner.yaml",
                      debug_dir.parent / "planner.yaml",
                      debug_dir.parent / "config" / "planner.yaml"):
        if candidate.exists():
            return candidate
    return None


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("debug_dir", nargs="?",
                        help="directory written by --debug-dir")
    parser.add_argument("cart_traj", nargs="?", default=None,
                        help="saved planner stdout holding the CART_TRAJ block")
    parser.add_argument("-o", "--outdir", default=None,
                        help="directory for the outputs (default: the debug dir)")
    parser.add_argument("--planner-config", default=None,
                        help="planner.yaml, for display geometry and thresholds")
    parser.add_argument("--controller-log", default=None,
                        help="recorded controller CSV with measured TCP/q/qdot")
    parser.add_argument("--requested-path-ik", default=None,
                        help="recorded requested path_ik.csv when replay failed preflight")
    parser.add_argument("--requested-point-m", nargs=3, type=float, default=None,
                        metavar=("X", "Y", "Z"),
                        help="Mount-frame requested point when evidence has no candidate row")
    parser.add_argument("--world-mount-identity", action="store_true",
                        help="confirm world_T_mount is identity for request/scene overlay")
    args = parser.parse_args()
    if args.debug_dir is None:
        parser.error("DEBUG_DIR is required")

    debug_dir = Path(args.debug_dir)
    if not debug_dir.is_dir():
        raise SystemExit(f"{debug_dir}: not a directory")

    meta = read_meta(debug_dir / "meta.csv")
    if not meta:
        raise SystemExit(f"{debug_dir}: no meta.csv — not a --debug-dir dump")

    joints = read_csv(debug_dir / "joints.csv")
    limits = read_csv(debug_dir / "joint_limits.csv")
    path_ik = read_csv(debug_dir / "path_ik.csv")
    candidates = read_csv(debug_dir / "candidate_attempts.csv")
    requested_path = (read_csv(Path(args.requested_path_ik))
                      if args.requested_path_ik else path_ik)
    requested_point = (args.requested_point_m or
                       requested_point_from_candidates(candidates))

    # Say what this plan DID before saying anything about figures. Missing
    # trajectory/path artifacts are normal for preflight and point failures.
    status = meta.get("status", "?")
    if not plan_is_executable(status):
        print(f"PLAN FAILED ({meta.get('arm', '?')} arm, "
              f"{meta.get('plan_kind', '?')} plan): {status} — "
              f"{meta.get('failure_reason', '')}", file=sys.stderr)
        print("  No trajectory was emitted, so there is nothing to draw for "
              "the planned motion.", file=sys.stderr)
        if path_ik:
            print("  The figures below show the requested path and its "
                  "recorded IK walk.", file=sys.stderr)
        elif requested_path:
            print("  The figures below show the requested path and available "
                  "telemetry; this replay recorded no IK walk.", file=sys.stderr)
        else:
            print("  The figures below show available request, candidate, and "
                  "telemetry evidence; no path IK walk was recorded.",
                  file=sys.stderr)
    else:
        print(f"plan {status} ({meta.get('arm', '?')} arm, "
              f"{meta.get('plan_kind', '?')} plan)", file=sys.stderr)
    cart_path = Path(args.cart_traj) if args.cart_traj else debug_dir / "cart_traj.txt"
    cart = read_cart_traj(cart_path) if cart_path.exists() else None
    if cart is not None and not cart[0]:
        reason = ("because the plan failed before emitting one"
                  if not plan_is_executable(status)
                  else "— is this the planner's saved stdout?")
        print(f"  {cart_path} holds no CART_TRAJ block {reason}.",
              file=sys.stderr)
        cart = None

    config_path = (Path(args.planner_config) if args.planner_config
                   else find_planner_config(debug_dir))
    plot_config = read_plot_config(config_path)
    accept_m = plot_config["maximum_planning_error_m"]
    accept_rad = plot_config["maximum_orientation_error_rad"]

    outdir = Path(args.outdir) if args.outdir else debug_dir
    outdir.mkdir(parents=True, exist_ok=True)
    controller = read_controller_log(
        Path(args.controller_log) if args.controller_log else None)
    frames_coincident = (args.world_mount_identity or
                         meta.get("world_T_mount") == "identity")

    written = [
        plot_cartesian(meta, cart, requested_path, controller,
                       plot_config["scene"], requested_point,
                       frames_coincident, outdir / "plan_cartesian.png"),
        plot_joints(meta, joints, limits, controller,
                    outdir / "plan_joints.png"),
        plot_clearance(meta, candidates, plot_config["minimum_clearance_m"],
                       plot_config["preferred_clearance_m"],
                       outdir / "plan_clearance.png"),
        plot_candidates(meta, candidates, outdir / "plan_candidates.png"),
        plot_path_ik(meta, path_ik, accept_m, accept_rad,
                     outdir / "plan_path_ik.png"),
    ]
    produced = [p for p in written if p is not None]
    if not produced:
        raise SystemExit(f"{debug_dir}: nothing to plot")
    report = write_report(meta, outdir, produced, path_ik)
    for path in produced:
        print(path)
    print(report)


if __name__ == "__main__":
    main()
