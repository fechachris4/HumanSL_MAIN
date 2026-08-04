#!/usr/bin/env python3
"""Plot a controller run: requested vs sent vs measured, per joint.

One offline plotting tool for the log_format 5 CSV the controller writes
(hardware/Record.h is the schema authority). It answers the question the
telemetry exists for: what did the controller ask for, what actually went
into the cyclic message, and what did the robot report back?

    scripts/plot_run.py [RUN.csv] [-o OUTDIR]

With no CSV it picks the newest run under runs/ (scripts/runlog.py).

Definitions, all straight from the CSV columns (see Record.h):

    requested  reqvel_j*  deg/s   controller output, before the speed clamp
               req_j*     deg     integrated setpoint, before lead/rate constraints
    sent       cmdvel_j*  deg/s   velocity the written command realised
               cmd_j*     deg     position written into the cyclic message
    measured   meas_j*    deg     position returned in the Send reply
               vel_j*     deg/s   velocity returned in the Send reply
    error                 deg     cmd_j* - meas_j*  (same units, same axis)

Only same-unit quantities are ever differenced or drawn together, and no
plot uses two y-scales.

A file whose preamble says `data_source = synthetic`, or whose first line
carries the SYNTHETIC marker, is stamped as synthetic on every figure. That
stamp is not decoration — synthetic rows are computed, never measured, and
must not be read as robot evidence.
"""

import argparse
import csv
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
from runlog import find_default_csv

NUM_JOINTS = 7

# Joint identity. The runtime model is config/GEN3_dual_mounted.urdf (the
# path is compiled in via GEN3_DUAL_URDF_PATH and echoed in every run
# preamble). Its right-arm chain is Actuator1..Actuator7, in the same order
# as the Kortex actuator array the loop indexes, so log column j<N> is
# URDF joint Actuator<N> is Kortex actuator index N-1.
JOINT_NAMES = [f"Actuator{i}" for i in range(1, NUM_JOINTS + 1)]

# Joints under review — drawn first, tinted, and given their own figure.
FOCUS_JOINTS = (3, 4)

# Position limits, degrees, as |q| <= limit; 0.0 means "continuous joint, no
# position limit". TWO sources, checked against each other on 2026-08-03:
#
#   URDF    config/GEN3_dual_mounted.urdf, the model the runtime loads.
#           Actuator1/3/5/7 are type="continuous" with no <limit> range;
#           Actuator2/4/6 are revolute at +/-128.34 / 147.25 / 119.75 deg.
#   CONFIG  src/app/Config.h kTrajectoryPosLimitDeg, the range the trajectory
#           validator actually enforces: 0 / 128.9 / 0 / 147.8 / 0 / 120.3 / 0.
#
# They AGREE on which joints are bounded, and differ by about 0.55 deg on
# each bounded joint, with Config.h the more permissive of the two. Both are
# drawn; neither is silently preferred.
#
# NEITHER is this arm's CONFIGURED soft limit. The README records configured
# limits far inside both (joint 4 near -19.6 deg, joint 6 near +36 deg), which
# can only be confirmed from the robot itself. Margins below are therefore
# labelled by source and must not be read as "distance to the arm's trip
# point".
URDF_LIMIT_DEG = [0.0, 128.34, 0.0, 147.25, 0.0, 119.75, 0.0]
CONFIG_LIMIT_DEG = [0.0, 128.9, 0.0, 147.8, 0.0, 120.3, 0.0]

# Categorical hues in fixed order, one per role, never cycled. Validated
# colourblind-safe against a light surface; line style is a second encoding
# so the three roles never depend on colour alone.
C_REQUESTED = "#E69F00"
C_SENT = "#0072B2"
C_MEASURED = "#009E73"
C_INK = "#1a1a1a"
C_MUTED = "#6b6b6b"
C_GRID = "#d8d8d4"
C_FOCUS_BG = "#f2f0e9"
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


def read_run(path):
    """Split a run CSV into (preamble lines, list of row dicts)."""
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
    """One column as floats; NaN for blanks and unparseable values."""
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
    """Provenance on every figure: which file, how it ended, and whether it
    is real. The synthetic banner is loud on purpose."""
    # Bottom edge, so it can never collide with the suptitle.
    fig.text(0.005, 0.004, f"source: {Path(path).name}", ha="left",
             va="bottom", fontsize=6.5, color=C_MUTED)
    if exit_note:
        fig.text(0.995, 0.004, exit_note, ha="right", va="bottom",
                 fontsize=6.5, color=C_MUTED)
    if synthetic:
        fig.text(0.5, 0.5, "SYNTHETIC — NOT ROBOT DATA", ha="center",
                 va="center", fontsize=34, color="#d94801", alpha=0.13,
                 rotation=27, zorder=0, weight="bold")
        fig.text(0.5, 0.965,
                 "SYNTHETIC DATA — computed by tools/make_synthetic_log.cpp, "
                 "not measured from a robot",
                 ha="center", va="top", fontsize=8, color="#a33900",
                 weight="bold")


def joint_axes(fig_title, path, preamble, synthetic, exit_note, ylabel):
    """A 4x2 grid of per-joint panels, joints 3 and 4 first and tinted."""
    fig, axes = plt.subplots(4, 2, figsize=(11, 11), sharex=True)
    flat = axes.flatten()
    order = list(FOCUS_JOINTS) + [j for j in range(1, NUM_JOINTS + 1)
                                  if j not in FOCUS_JOINTS]
    for slot, joint in enumerate(order):
        ax = flat[slot]
        focus = joint in FOCUS_JOINTS
        if focus:
            ax.set_facecolor(C_FOCUS_BG)
        ax.set_title(f"j{joint} — {JOINT_NAMES[joint - 1]}"
                     + ("  (under review)" if focus else ""),
                     fontsize=8.5, weight="bold" if focus else "normal",
                     loc="left")
        ax.set_ylabel(ylabel)
    flat[-1].axis("off")
    for ax in flat[-3:-1]:
        ax.set_xlabel("time (s)")
    fig.suptitle(fig_title, fontsize=12, weight="bold", x=0.008, ha="left",
                 y=0.995)
    stamp(fig, path, preamble, synthetic, exit_note)
    return fig, {joint: flat[slot] for slot, joint in enumerate(order)}


def fig_focus(rows, t, path, preamble, synthetic, exit_note):
    """Joints 3 and 4 alone, at full width: the three roles, then the error."""
    fig, axes = plt.subplots(2, 2, figsize=(12, 7), sharex=True)
    for column, joint in enumerate(FOCUS_JOINTS):
        req = col(rows, f"req_j{joint}")
        cmd = col(rows, f"cmd_j{joint}")
        meas = col(rows, f"meas_j{joint}")

        top = axes[0][column]
        top.set_facecolor(C_FOCUS_BG)
        top.plot(t, req, color=C_REQUESTED, lw=2.4, ls=(0, (5, 2)),
                 label="requested (pre lead/rate constraints)")
        top.plot(t, cmd, color=C_SENT, lw=2.0, label="sent (into cyclic msg)")
        top.plot(t, meas, color=C_MEASURED, lw=2.0, ls=(0, (1, 1.6)),
                 label="measured (robot reply)")
        top.set_title(f"j{joint} — {JOINT_NAMES[joint - 1]}", fontsize=10,
                      weight="bold", loc="left")
        top.set_ylabel("joint angle (deg)")
        top.legend(fontsize=7.5, loc="best")

        bottom = axes[1][column]
        bottom.set_facecolor(C_FOCUS_BG)
        err = [c - m for c, m in zip(cmd, meas)]
        bottom.axhline(0, color=C_GRID, lw=1)
        bottom.plot(t, err, color=C_INK, lw=1.6,
                    label="tracking error  cmd - meas")
        limiter = [r - c for r, c in zip(req, cmd)]
        bottom.plot(t, limiter, color=C_REQUESTED, lw=1.4, ls=(0, (5, 2)),
                    label="combined lead/rate effect  req - cmd")
        bottom.set_ylabel("degrees")
        bottom.set_xlabel("time (s)")
        bottom.legend(fontsize=7.5, loc="best")

    fig.suptitle("Joints 3 and 4 — requested, sent, measured, and the gap "
                 "between them", fontsize=12, weight="bold", x=0.008,
                 ha="left", y=0.995)
    stamp(fig, path, preamble, synthetic, exit_note)
    fig.tight_layout(rect=(0, 0, 1, 0.94 if synthetic else 0.97))
    return fig


def fig_req_sent_meas(rows, t, path, preamble, synthetic, exit_note):
    fig, panels = joint_axes(
        "Requested vs sent vs measured position — every joint",
        path, preamble, synthetic, exit_note, "joint angle (deg)")
    for joint, ax in panels.items():
        ax.plot(t, col(rows, f"req_j{joint}"), color=C_REQUESTED, lw=1.9,
                ls=(0, (5, 2)), label="requested")
        ax.plot(t, col(rows, f"cmd_j{joint}"), color=C_SENT, lw=1.5,
                label="sent")
        ax.plot(t, col(rows, f"meas_j{joint}"), color=C_MEASURED, lw=1.5,
                ls=(0, (1, 1.6)), label="measured")
    panels[FOCUS_JOINTS[0]].legend(fontsize=7.5, loc="best")
    fig.tight_layout(rect=(0, 0, 1, 0.94 if synthetic else 0.97))
    return fig


def fig_tracking_error(rows, t, path, preamble, synthetic, exit_note,
                       following_error_limit_deg):
    fig, panels = joint_axes(
        "Tracking error per joint  (sent - measured, same units)",
        path, preamble, synthetic, exit_note, "error (deg)")
    for joint, ax in panels.items():
        cmd = col(rows, f"cmd_j{joint}")
        meas = col(rows, f"meas_j{joint}")
        ax.axhline(0, color=C_GRID, lw=1)
        ax.plot(t, [c - m for c, m in zip(cmd, meas)], color=C_INK, lw=1.4,
                label="cmd - meas")
        if following_error_limit_deg:
            for sign in (1, -1):
                ax.axhline(sign * following_error_limit_deg, color="#c1272d",
                           lw=1.1, ls=(0, (4, 3)))
    handles = [plt.Line2D([], [], color=C_INK, lw=1.4, label="cmd - meas")]
    if following_error_limit_deg:
        handles.append(plt.Line2D([], [], color="#c1272d", lw=1.1,
                                  ls=(0, (4, 3)),
                                  label=f"following-error stop "
                                        f"(+/-{following_error_limit_deg:g} deg)"))
    panels[FOCUS_JOINTS[0]].legend(handles=handles, fontsize=7.5, loc="best")
    fig.tight_layout(rect=(0, 0, 1, 0.94 if synthetic else 0.97))
    return fig


def fig_motion(rows, t, path, preamble, synthetic, exit_note):
    """Commanded vs measured MOTION, matched as velocity against velocity."""
    fig, panels = joint_axes(
        "Commanded vs measured motion — velocity against velocity "
        "(matched quantities)",
        path, preamble, synthetic, exit_note, "velocity (deg/s)")
    for joint, ax in panels.items():
        ax.axhline(0, color=C_GRID, lw=1)
        if has_col(rows, f"reqvel_j{joint}"):
            ax.plot(t, col(rows, f"reqvel_j{joint}"), color=C_REQUESTED,
                    lw=1.7, ls=(0, (5, 2)), label="requested q̇ (pre-clamp)")
        ax.plot(t, col(rows, f"cmdvel_j{joint}"), color=C_SENT, lw=1.4,
                label="sent q̇ (command realised)")
        ax.plot(t, col(rows, f"vel_j{joint}"), color=C_MEASURED, lw=1.4,
                ls=(0, (1, 1.6)), label="measured q̇ (robot reply)")
    panels[FOCUS_JOINTS[0]].legend(fontsize=7.5, loc="best")
    fig.tight_layout(rect=(0, 0, 1, 0.94 if synthetic else 0.97))
    return fig


def unchanged_runs(values):
    """Length of the current run of identical values, per sample."""
    out, run = [], 0
    for i, v in enumerate(values):
        if i and (v == values[i - 1]):
            run += 1
        else:
            run = 0
        out.append(run)
    return out


def fig_freshness(rows, t, path, preamble, synthetic, exit_note):
    """Is the feedback stream advancing at all? Four independent views."""
    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)

    # 1. Acknowledgement staleness: consecutive cycles each joint's
    #    command_id repeated. Kortex 2.7.0 reports command_id as the last
    #    command PROCESSED — it is not an acceptance signal.
    ax = axes[0]
    if has_col(rows, "ack_unchanged_j1"):
        for joint in range(1, NUM_JOINTS + 1):
            focus = joint in FOCUS_JOINTS
            ax.plot(t, col(rows, f"ack_unchanged_j{joint}"),
                    lw=2.0 if focus else 1.0,
                    color=C_SENT if focus else C_MUTED,
                    alpha=1.0 if focus else 0.55,
                    label=f"j{joint}" if focus else None)
        ax.legend(fontsize=7.5, loc="upper left",
                  title="bold = joints under review", title_fontsize=7)
    else:
        ax.text(0.5, 0.5, "no ack_unchanged_j* columns (pre-format-5 log)",
                ha="center", va="center", transform=ax.transAxes,
                color=C_MUTED)
    ax.set_ylabel("cycles ack\nunchanged")
    ax.set_title("Feedback acknowledgement staleness — consecutive cycles the "
                 "actuator's command_id repeated", fontsize=9, loc="left",
                 weight="bold")

    # 2. Frame-id gaps: a healthy stream advances both ids by exactly 1.
    ax = axes[1]
    if has_col(rows, "feedback_frame_id"):
        fb = col(rows, "feedback_frame_id")
        gaps = [float("nan")] + [b - a for a, b in zip(fb, fb[1:])]
        ax.plot(t, gaps, color=C_INK, lw=1.2, label="feedback_frame_id step")
        ax.axhline(1, color=C_MEASURED, lw=1.1, ls=(0, (4, 3)),
                   label="expected step = 1")
        missing = sum(1 for g in gaps if not math.isnan(g) and g > 1)
        ax.set_title(f"Frame continuity — {missing} cycle(s) where the "
                     f"feedback frame id jumped by more than one",
                     fontsize=9, loc="left", weight="bold")
        ax.legend(fontsize=7.5, loc="best")
    ax.set_ylabel("frame id step")

    # 3. Unchanged measurements: how long has this joint reported the exact
    #    same angle? This is the signature the 31 July runs showed.
    ax = axes[2]
    for joint in range(1, NUM_JOINTS + 1):
        focus = joint in FOCUS_JOINTS
        ax.plot(t, unchanged_runs(col(rows, f"meas_j{joint}")),
                lw=2.0 if focus else 1.0,
                color=C_MEASURED if focus else C_MUTED,
                alpha=1.0 if focus else 0.55,
                label=f"j{joint}" if focus else None)
    ax.set_ylabel("cycles measurement\nunchanged")
    ax.set_title("Unchanged measurements — consecutive cycles reporting the "
                 "identical angle (logged, never a stop)",
                 fontsize=9, loc="left", weight="bold")
    ax.legend(fontsize=7.5, loc="upper left")

    # 4. Timing and refresh health.
    ax = axes[3]
    ax.plot(t, [v * 1000.0 for v in col(rows, "dt_s")], color=C_INK, lw=1.0,
            label="cycle dt")
    if has_col(rows, "t_send_s") and has_col(rows, "t_recv_s"):
        send, recv = col(rows, "t_send_s"), col(rows, "t_recv_s")
        ax.plot(t, [(r - s) * 1000.0 for s, r in zip(send, recv)],
                color=C_SENT, lw=1.0, label="cyclic exchange round trip")
    bad = [tt for tt, ok in zip(t, col(rows, "refresh_ok")) if ok == 0]
    for tt in bad:
        ax.axvline(tt, color="#c1272d", lw=1.0, alpha=0.8)
    ax.set_ylabel("milliseconds")
    ax.set_xlabel("time (s)")
    ax.set_title(f"Cycle timing and refresh health — {len(bad)} cycle(s) with "
                 f"refresh_ok = 0 (red)", fontsize=9, loc="left",
                 weight="bold")
    ax.legend(fontsize=7.5, loc="best")

    fig.suptitle("Feedback freshness, frame continuity and timing",
                 fontsize=12, weight="bold", x=0.008, ha="left", y=0.995)
    stamp(fig, path, preamble, synthetic, exit_note)
    fig.tight_layout(rect=(0, 0, 1, 0.94 if synthetic else 0.97))
    return fig


def fig_limits_and_exit(rows, t, path, preamble, synthetic, exit_reason,
                        exit_t, exit_note):
    """Distance to each documented position limit, and how the run ended."""
    fig, axes = plt.subplots(2, 1, figsize=(11, 9.5),
                             gridspec_kw={"height_ratios": [3, 1.5],
                                          "hspace": 0.75})

    ax = axes[0]
    plotted = False
    for joint in range(1, NUM_JOINTS + 1):
        meas = col(rows, f"meas_j{joint}")
        # Compare on the wrapped-to-+/-180 axis the limits are expressed on.
        wrapped = [((v + 180.0) % 360.0) - 180.0 for v in meas]
        for limit, source, colour, dash in (
                (CONFIG_LIMIT_DEG[joint - 1], "Config.h", C_SENT, None),
                (URDF_LIMIT_DEG[joint - 1], "URDF", C_REQUESTED, (0, (5, 2)))):
            if limit <= 0.0:
                continue
            margin = [limit - abs(v) for v in wrapped]
            focus = joint in FOCUS_JOINTS
            ax.plot(t, margin, color=colour, lw=2.0 if focus else 0.9,
                    ls=dash if dash else "-", alpha=1.0 if focus else 0.35,
                    label=f"j{joint} vs {source} ({limit:g} deg)"
                    if focus else None)
            plotted = True
    if plotted:
        ax.axhline(0, color="#c1272d", lw=1.2)
        ax.legend(fontsize=7.5, loc="best", ncol=2)
    else:
        ax.text(0.5, 0.5, "no bounded joints in either limit source",
                ha="center", va="center", transform=ax.transAxes,
                color=C_MUTED)
    ax.set_ylabel("margin to limit (deg)")
    ax.set_xlabel("time (s)")
    ax.set_title("Position-limit margin — bounded joints only, against BOTH "
                 "documented sources", fontsize=9.5, loc="left", weight="bold")
    ax.text(0.0, -0.16,
            "Both documented sources are drawn. Config.h kTrajectoryPosLimitDeg "
            "gives j2/j4/j6 = 128.9 / 147.8 / 120.3 deg;\n"
            "the loaded URDF gives 128.34 / 147.25 / 119.75 deg — they agree on "
            "which joints are bounded and differ\n"
            "by ~0.55 deg, Config.h being the more permissive. NEITHER is this "
            "arm's CONFIGURED soft limit: the README\n"
            "records j4 near -19.6 deg and j6 near +36 deg, which only the robot "
            "can confirm. Read these as model\n"
            "margins, not trip distance.",
            transform=ax.transAxes, fontsize=7, color=C_MUTED, va="top")

    ax = axes[1]
    ax.axis("off")
    lines = [f"exit reason : {exit_reason or 'not recorded in this CSV'}"]
    if exit_t is not None:
        lines.append(f"exit time   : {exit_t:g} s")
    for key in ("exit_cycle", "faults_observed", "log_format", "controller",
                "control_dt_s"):
        value = preamble_value(preamble, key)
        if value is not None:
            lines.append(f"{key:<12}: {value}")
    lines.append(f"rows        : {len(rows)}")
    lines.append(f"span        : {t[0]:g} to {t[-1]:g} s")
    ax.text(0.0, 1.02, "Final exit event", fontsize=10, weight="bold",
            va="top")
    ax.text(0.0, 0.80, "\n".join(lines), fontsize=7.5, va="top",
            family="monospace", linespacing=1.35)
    if exit_t is not None:
        axes[0].axvline(exit_t, color="#c1272d", lw=1.6)
        axes[0].annotate(f"exit: {exit_reason}", xy=(exit_t, 0),
                         xytext=(-6, 12), textcoords="offset points",
                         ha="right", fontsize=7.5, color="#c1272d",
                         weight="bold")

    fig.suptitle("Joint-limit margins and the final exit event", fontsize=12,
                 weight="bold", x=0.008, ha="left", y=0.995)
    stamp(fig, path, preamble, synthetic, exit_note)
    # Fixed margins rather than tight_layout: this figure places free text
    # below the axes, which tight_layout cannot measure.
    fig.subplots_adjust(left=0.13, right=0.97, hspace=0.75,
                        top=0.90 if synthetic else 0.93, bottom=0.06)
    return fig


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv", nargs="?", help="run CSV (default: newest under runs/)")
    parser.add_argument("-o", "--outdir", default=None,
                        help="directory for the PNGs (default: beside the CSV)")
    args = parser.parse_args()

    path = Path(args.csv) if args.csv else Path(find_default_csv())
    preamble, rows = read_run(path)
    synthetic = is_synthetic(preamble)

    missing = [c for c in ("time_s", "cmd_j1", "meas_j1") if c not in rows[0]]
    if missing:
        raise SystemExit(f"{path}: not a controller run CSV (missing {missing})")
    if "req_j1" not in rows[0]:
        print(f"note: {path.name} predates log_format 5 — requested-vs-sent "
              f"panels will be empty. Re-run the controller to get them.",
              file=sys.stderr)

    t = col(rows, "time_s")
    exit_reason = preamble_value(preamble, "exit_reason")
    exit_t_raw = preamble_value(preamble, "exit_time_s")
    exit_t = None
    if exit_t_raw:
        try:
            exit_t = float(exit_t_raw)
        except ValueError:
            exit_t = None
    exit_note = f"exit: {exit_reason}" if exit_reason else "exit: not recorded"

    limit_raw = preamble_value(preamble, "following_error_limit_deg")
    following_error_limit_deg = None
    if limit_raw:
        try:
            following_error_limit_deg = float(limit_raw.split()[0])
        except ValueError:
            pass

    outdir = Path(args.outdir) if args.outdir else path.parent
    outdir.mkdir(parents=True, exist_ok=True)
    prefix = ("SYNTHETIC_" if synthetic else "") + path.stem.replace(
        "SYNTHETIC_", "")

    with plt.rc_context(STYLE):
        figures = [
            ("focus_j3_j4", fig_focus(rows, t, path, preamble, synthetic, exit_note)),
            ("requested_sent_measured",
             fig_req_sent_meas(rows, t, path, preamble, synthetic, exit_note)),
            ("tracking_error",
             fig_tracking_error(rows, t, path, preamble, synthetic, exit_note,
                                following_error_limit_deg)),
            ("motion_velocity",
             fig_motion(rows, t, path, preamble, synthetic, exit_note)),
            ("feedback_freshness",
             fig_freshness(rows, t, path, preamble, synthetic, exit_note)),
            ("limits_and_exit",
             fig_limits_and_exit(rows, t, path, preamble, synthetic,
                                 exit_reason, exit_t, exit_note)),
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
