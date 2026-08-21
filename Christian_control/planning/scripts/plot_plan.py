#!/usr/bin/env python3
"""Plot a plan: what the planner produced, and where it failed.

    scripts/plot_plan.py DEBUG_DIR [CART_TRAJ_FILE] [-o OUTDIR]

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

CART_TRAJ_FILE is optional: the world-frame block the planner prints on
stdout, saved to a file. Give it and the Cartesian figure is drawn from
what the controller would actually receive; leave it out and the figure
falls back to the target positions in path_ik.csv.

Output: individual PNGs plus one plan_report.html tying them together with
the summary and a per-sample table around the failed or worst region. One
path per line on stdout, the panel's discovery convention.

Everything drawn is a value the planner itself produced; this script
arranges, it does not recompute planning decisions.
"""

import argparse
import csv
import html
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 — registers "3d"

NUM_JOINTS = 7

# A sample is FAILED only when its anchor solve found nothing. Interpolated
# samples are the walk's normal state (anchor initialization, 2026-08-21) —
# drawing them as failures would make every healthy plan look broken.
FAILED_STATUSES = ("no_convergence", "joint_limits", "failed")


def is_failed(row):
    return row.get("status", "") in FAILED_STATUSES


def read_csv(path):
    """Rows as dicts, or None if the file is absent (a point plan has no
    path_ik.csv, and a failed plan has no joints.csv)."""
    if not path.exists():
        return None
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


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


def title_for(meta, what):
    arm = meta.get("arm", "?")
    kind = meta.get("plan_kind", "?")
    status = meta.get("status", "?")
    state = "FAILED" if status != "ok" else "ok"
    head = f"{what} — {arm} arm, {kind} plan, {state}"
    if status != "ok":
        head += f"\n{status}"
    return head


def plot_cartesian(meta, cart, path_ik, outpath):
    """Where the tool goes, and how fast. Drawn from the emitted block when
    there is one, and from the path's TARGET positions when the plan failed
    before emitting anything — in the second case the figure says so, since
    a target is a request, not an achievement."""
    from_targets = cart is None
    if from_targets:
        if path_ik is None:
            return None
        t = col(path_ik, "t_s")
        x = col(path_ik, "target_x_m")
        y = col(path_ik, "target_y_m")
        z = col(path_ik, "target_z_m")
        speed = None
    else:
        t, x, y, z, speed = cart
    if not t:
        return None

    figure = plt.figure(figsize=(13, 5.5))
    axis3d = figure.add_subplot(1, 3, 1, projection="3d")
    axis3d.plot(x, y, z, color="tab:blue", linewidth=1.2,
                label="planned TCP" if not from_targets else "requested path")
    # The requested path overlaid on the achieved one, when both exist —
    # the gap between them IS the planning error, made visible.
    if not from_targets and path_ik is not None:
        axis3d.plot(col(path_ik, "target_x_m"), col(path_ik, "target_y_m"),
                    col(path_ik, "target_z_m"), color="tab:orange",
                    linewidth=1.0, linestyle="--", label="requested path")
    if path_ik is not None:
        bad = [i for i, row in enumerate(path_ik) if is_failed(row)]
        if bad:
            axis3d.scatter([col(path_ik, "target_x_m")[i] for i in bad],
                           [col(path_ik, "target_y_m")[i] for i in bad],
                           [col(path_ik, "target_z_m")[i] for i in bad],
                           color="tab:red", s=44, label="IK failed samples")
    axis3d.scatter([x[0]], [y[0]], [z[0]], color="tab:green", s=40, label="start")
    axis3d.scatter([x[-1]], [y[-1]], [z[-1]], color="black", s=40, label="end")
    axis3d.set_xlabel("x [m]")
    axis3d.set_ylabel("y [m]")
    axis3d.set_zlabel("z [m]")
    axis3d.set_title("tool path" + (" (requested only)" if from_targets else ""))
    axis3d.legend(loc="upper left", fontsize=7)

    axis_pos = figure.add_subplot(1, 3, 2)
    for values, name in ((x, "x"), (y, "y"), (z, "z")):
        axis_pos.plot(t, values, linewidth=1.0, label=name)
    axis_pos.set_xlabel("time [s]")
    axis_pos.set_ylabel("position [m]")
    axis_pos.set_title("position vs time")
    axis_pos.grid(alpha=0.3)
    axis_pos.legend(fontsize=8)

    axis_speed = figure.add_subplot(1, 3, 3)
    if speed is None:
        axis_speed.text(0.5, 0.5, "no emitted trajectory\n(plan did not reach output)",
                        ha="center", va="center", transform=axis_speed.transAxes,
                        fontsize=9, color="tab:red")
        axis_speed.set_axis_off()
    else:
        axis_speed.plot(t, speed, color="tab:purple", linewidth=1.0)
        axis_speed.set_xlabel("time [s]")
        axis_speed.set_ylabel("speed [m/s]")
        axis_speed.set_title("tool speed vs time")
        axis_speed.grid(alpha=0.3)

    figure.suptitle(title_for(meta, "Cartesian"), fontsize=10)
    figure.tight_layout(rect=(0, 0, 1, 0.90))
    figure.savefig(outpath, dpi=130)
    plt.close(figure)
    return outpath


def plot_joints(meta, joints, limits, path_ik, outpath):
    """Each joint against the band it was solved against. A trajectory that
    rides a limit is the usual reason a plan is strained even when it
    succeeds, and it is invisible in any Cartesian view. The task phase and
    any failed path samples are shaded so joint-space and path-space line
    up on the same time axis."""
    if joints is None:
        return None
    t = col(joints, "t_s")
    if not t:
        return None

    task_start = None
    try:
        task_start = float(meta["task_start_time_s"])
    except (KeyError, ValueError):
        pass
    failed_times = []
    if task_start is not None and path_ik is not None:
        failed_times = [task_start + float(row["t_s"])
                        for row in path_ik
                        if is_failed(row) and row.get("t_s")]

    figure, axes = plt.subplots(4, 2, figsize=(12, 10), sharex=True)
    flat = axes.flatten()
    for index in range(NUM_JOINTS):
        axis = flat[index]
        if task_start is not None:
            axis.axvspan(task_start, t[-1], color="tab:blue", alpha=0.05)
        for failed_time in failed_times:
            axis.axvline(failed_time, color="tab:red", alpha=0.3, linewidth=2.0)
        axis.plot(t, col(joints, f"q{index + 1}_deg"), color="tab:blue",
                  linewidth=1.0)
        if limits is not None:
            lower = float(limits[index]["lower_deg"])
            upper = float(limits[index]["upper_deg"])
            # A continuous joint carries a +/-1e20 sentinel; drawing that
            # would flatten every real curve to a horizontal line.
            if abs(lower) < 1e6 and abs(upper) < 1e6:
                axis.axhspan(lower, upper, color="tab:green", alpha=0.08)
                axis.axhline(lower, color="tab:red", linewidth=0.8, linestyle="--")
                axis.axhline(upper, color="tab:red", linewidth=0.8, linestyle="--")
            else:
                axis.text(0.02, 0.90, "continuous joint (no position limit)",
                          transform=axis.transAxes, fontsize=7, color="grey")
        axis.set_ylabel(f"q{index + 1} [deg]")
        axis.grid(alpha=0.3)
    flat[NUM_JOINTS].set_axis_off()
    note = []
    if task_start is not None:
        note.append("blue shading = constrained task phase")
    if failed_times:
        note.append("red lines = IK-failed path samples")
    if note:
        flat[NUM_JOINTS].text(0.05, 0.6, "\n".join(note), fontsize=8)
    for axis in (flat[5], flat[6]):
        axis.set_xlabel("time [s]")

    figure.suptitle(title_for(meta, "joint trajectory vs limits"), fontsize=10)
    figure.tight_layout(rect=(0, 0, 1, 0.93))
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


def table_window(path_ik, accept_m):
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


def write_report(meta, outdir, figures, path_ik, accept_m):
    """One page holding the summary, the figures and the per-sample table —
    the single thing to open after a planner run."""
    failed = meta.get("status", "?") != "ok"
    colour = "#c0392b" if failed else "#1e8449"
    state = "FAILED" if failed else "OK"

    rows, note = table_window(path_ik, accept_m)
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


def read_planner_yaml_value(planner_config, key):
    """One scalar from planner.yaml, by simple key match rather than a YAML
    parser — the file is the planner's, and this script must not become a
    second opinion on how to read it."""
    if planner_config is None or not planner_config.exists():
        return None
    for line in planner_config.read_text().splitlines():
        stripped = line.strip()
        if stripped.startswith(key + ":"):
            try:
                return float(stripped.split(":", 1)[1].split("#")[0].strip())
            except ValueError:
                return None
    return None


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("debug_dir", help="directory written by --debug-dir")
    parser.add_argument("cart_traj", nargs="?", default=None,
                        help="saved planner stdout holding the CART_TRAJ block")
    parser.add_argument("-o", "--outdir", default=None,
                        help="directory for the outputs (default: the debug dir)")
    parser.add_argument("--planner-config", default=None,
                        help="planner.yaml, for the acceptance tolerance lines")
    args = parser.parse_args()

    debug_dir = Path(args.debug_dir)
    if not debug_dir.is_dir():
        raise SystemExit(f"{debug_dir}: not a directory")

    meta = read_meta(debug_dir / "meta.csv")
    if not meta:
        raise SystemExit(f"{debug_dir}: no meta.csv — not a --debug-dir dump")

    # Say what this plan DID before saying anything about figures. A failed
    # plan emits no trajectory, so half the panels are empty and the fallback
    # notes below read as tooling breakage unless the failure is stated first
    # and stated loudly.
    status = meta.get("status", "?")
    if status != "ok":
        print(f"PLAN FAILED ({meta.get('arm', '?')} arm, "
              f"{meta.get('plan_kind', '?')} plan): {status}", file=sys.stderr)
        print("  No trajectory was emitted, so there is nothing to draw for "
              "the planned motion.", file=sys.stderr)
        print("  The figures below show the REQUESTED path and the IK walk "
              "that failed on it.", file=sys.stderr)
    else:
        print(f"plan ok ({meta.get('arm', '?')} arm, "
              f"{meta.get('plan_kind', '?')} plan)", file=sys.stderr)

    joints = read_csv(debug_dir / "joints.csv")
    limits = read_csv(debug_dir / "joint_limits.csv")
    path_ik = read_csv(debug_dir / "path_ik.csv")
    cart = read_cart_traj(Path(args.cart_traj)) if args.cart_traj else None
    if cart is not None and not cart[0]:
        reason = ("because the plan failed before emitting one"
                  if status != "ok"
                  else "— is this the planner's saved stdout?")
        print(f"  {args.cart_traj} holds no CART_TRAJ block {reason}.",
              file=sys.stderr)
        cart = None

    config_path = (Path(args.planner_config) if args.planner_config
                   else debug_dir.parent / "config" / "planner.yaml")
    accept_m = read_planner_yaml_value(config_path, "maximum_planning_error_m")
    accept_rad = read_planner_yaml_value(config_path,
                                         "maximum_orientation_error_rad")

    outdir = Path(args.outdir) if args.outdir else debug_dir
    outdir.mkdir(parents=True, exist_ok=True)

    written = [
        plot_cartesian(meta, cart, path_ik, outdir / "plan_cartesian.png"),
        plot_joints(meta, joints, limits, path_ik, outdir / "plan_joints.png"),
        plot_path_ik(meta, path_ik, accept_m, accept_rad,
                     outdir / "plan_path_ik.png"),
    ]
    if joints is None:
        print("  (no joints figure: joints.csv is absent, which is expected "
              "when the plan failed before a trajectory existed)",
              file=sys.stderr)
    produced = [p for p in written if p is not None]
    if not produced:
        raise SystemExit(f"{debug_dir}: nothing to plot")
    report = write_report(meta, outdir, produced, path_ik, accept_m)
    for path in produced:
        print(path)
    print(report)


if __name__ == "__main__":
    main()
