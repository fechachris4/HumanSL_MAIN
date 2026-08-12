"""Recorded runs: what is on disk, and what each one turned out like.

The controller writes one CSV per arm per run under runs/<date>/, and
run_session.sh writes a session directory beside it holding the goal file,
the planner config, the controller's terminal output, the plan it sent and a
session.json naming the git revision and the binaries' digests. The two are
linked only by the run log's name, which is why find_session_dir exists.

Everything here is read-only and after the fact. A summary streams the file
once and keeps running extremes, rather than sampling the tail: a 4 ms spike
in a 156 MB log is exactly the thing a summary exists to catch, and the last
five thousand rows of a run say nothing about the first minute of it.

Values that were 'nan' in the log stay absent (None) rather than becoming
zero. A zero position error reads as perfect tracking, so a column the
controller never computed must not be reported as one it did.
"""

from __future__ import annotations

import json
import math
import re
from datetime import datetime
from pathlib import Path
from typing import Any

from . import paths, telemetry

# loop_log_right_20260811_153920.csv — the arm is in the name because the
# controller writes one log per arm and nothing else records which is which.
_RUN_NAME_RE = re.compile(r"^loop_log_([A-Za-z]+)_")

# Quantities where the worst case is the largest value seen.
_WORST_MAX = (
    "rot_error_rad",
    "joint_follow_error_deg",
    "posture_error_deg",
    "base_comp_m",
    "null_leak_mps",
)

# Quantities where the worst case is the smallest — a margin and a
# conditioning measure, both of which get worse as they shrink.
_WORST_MIN = (
    "joint_margin_deg",
    "sigma_min",
)

# The one-cycle edges the run screen's banner is inferred from.
_EDGE_FLAGS = ("traj_activated", "traj_rejected", "traj_complete")

# A run that rejected a hundred plans has a problem no list can express, so
# the edge list is capped and says it was truncated.
_MAX_EDGES = 200


# ---- listing -----------------------------------------------------------


def list_runs(limit: int = 30, runs_root: Path | None = None) -> list[dict[str, Any]]:
    """The newest run logs, newest first.

    Only runs/<date>/*.csv are listed. run_session.sh hard-links a copy of
    each log into its session directory, so globbing deeper would list every
    run twice under two names for one set of bytes on disk.
    """
    root = runs_root or paths.RUNS
    if not root.is_dir():
        return []
    files = sorted(root.glob("*/loop_log_*.csv"),
                   key=lambda p: p.stat().st_mtime, reverse=True)[:limit]
    listed = []
    for path in files:
        stat = path.stat()
        session = find_session_dir(path)
        listed.append({
            "file": str(path.relative_to(root)),
            "mb": round(stat.st_size / 1e6, 1),
            "mtime": stat.st_mtime,
            "mtime_iso": datetime.fromtimestamp(stat.st_mtime).isoformat(
                timespec="seconds"),
            "arm": run_arm(path),
            "session": str(session.relative_to(root)) if session else None,
        })
    return listed


def run_arm(path: Path) -> str | None:
    """Which arm wrote this log, from its filename."""
    matched = _RUN_NAME_RE.match(path.name)
    return matched.group(1) if matched else None


def find_session_dir(path: Path) -> Path | None:
    """The session directory this run belongs to, if it has one.

    A run started from a bare `controller --arm right` has none, and that is
    normal rather than an error. run_session.sh links the log into its
    session directory under the same name, and the name carries the run's
    start time to the second, so matching by name is unambiguous in practice.
    """
    if path.parent.name.startswith("session_"):
        return path.parent
    for candidate in sorted(path.parent.glob("session_*")):
        if candidate.is_dir() and (candidate / path.name).is_file():
            return candidate
    return None


def read_session_json(directory: Path) -> dict[str, Any] | None:
    """run_session.sh's record of what was running: revision, digests, times.

    Absent for a session that was killed before its EXIT trap ran, which is
    worth showing as absent rather than as an error.
    """
    path = directory / "session.json"
    if not path.is_file():
        return None
    try:
        loaded = json.loads(path.read_text(errors="replace"))
    except (OSError, json.JSONDecodeError):
        return None
    return loaded if isinstance(loaded, dict) else None


def stop_reason(directory: Path) -> dict[str, Any] | None:
    """The controller's own account of why it stopped, from controller.log.

    Safety.cpp's PrintStopReport writes one 'loop stopped...' sentence naming
    the guard that fired, the joint involved and the limit it crossed. That
    sentence is the most useful thing in the whole session directory when a
    run ends unexpectedly, so it is lifted out whole rather than reduced to a
    category.
    """
    path = directory / "controller.log"
    if not path.is_file():
        return None
    line = None
    try:
        with open(path, "r", errors="replace") as handle:
            for text in handle:
                if text.startswith("loop stopped"):
                    line = text.rstrip()
    except OSError:
        return None
    if line is None:
        return None
    return {"line": line, "source": str(path)}


# ---- summarising -------------------------------------------------------


def _finite(text: str) -> float | None:
    """A CSV field as a number, or None for 'nan' and anything unparseable."""
    try:
        value = float(text)
    except ValueError:
        return None
    return value if math.isfinite(value) else None


def _resolve_run(rel: str, root: Path) -> Path | None:
    """A caller-supplied run path, refused unless it stays under runs/.

    `rel` arrives from a query string, so '../../etc/passwd' has to fail
    here. Resolving first and comparing afterwards is what makes a symlink or
    a '..' segment unable to escape.
    """
    resolved_root = root.resolve()
    candidate = (root / rel).resolve()
    if not candidate.is_relative_to(resolved_root) or not candidate.is_file():
        return None
    return candidate


def summarize_run(rel: str, runs_root: Path | None = None) -> dict[str, Any]:
    """Everything about one finished run, in one pass over the file.

    The preamble is authoritative for the compiled settings the run used, so
    log_format and reference_source are read from it and never from Config.h,
    which may have moved on since. The worst-case block is the run's own
    extremes; the bands they should be judged against are the thresholds in
    the same preamble.
    """
    root = runs_root or paths.RUNS
    path = _resolve_run(rel, root)
    if path is None:
        return {"error": f"no such run: {rel}"}

    preamble_lines: list[str] = []
    header: list[str] | None = None
    # Column positions are looked up once, from the header, and never
    # guessed. Hardware.h's format history is explicit that index-based
    # tooling breaks across format changes, and it has.
    at: dict[str, int] = {}
    max_wanted: list[tuple[str, int]] = []
    min_wanted: list[tuple[str, int]] = []
    edge_wanted: list[tuple[str, int]] = []
    joint_wanted: list[tuple[int, int, int]] = []
    pose_wanted: list[int] = []

    rows = 0
    first_time_s: float | None = None
    last_time_s: float | None = None
    worst: dict[str, Any] = {name: None for name in _WORST_MAX + _WORST_MIN}
    worst["position_error_m"] = None
    worst["following_error_deg"] = None
    worst["following_error_joint"] = None
    replan_advised_rows = 0
    edges: list[dict[str, Any]] = []
    edges_truncated = False

    with open(path, "r", errors="replace") as handle:
        for line in handle:
            if line.startswith("#"):
                preamble_lines.append(line.rstrip())
                continue
            if header is None:
                header = line.rstrip().split(",")
                at = {name: i for i, name in enumerate(header)}
                max_wanted = [(n, at[n]) for n in _WORST_MAX if n in at]
                min_wanted = [(n, at[n]) for n in _WORST_MIN if n in at]
                edge_wanted = [(n, at[n]) for n in _EDGE_FLAGS if n in at]
                joint_wanted = [(j, at[f"cmd_j{j}"], at[f"meas_j{j}"])
                                for j in range(1, 8)
                                if f"cmd_j{j}" in at and f"meas_j{j}" in at]
                pose_wanted = [at[n] for n in ("pd_x", "pd_y", "pd_z",
                                               "p_x", "p_y", "p_z") if n in at]
                continue
            parts = line.rstrip().split(",")
            if len(parts) < len(header):
                continue
            rows += 1

            time_s = _finite(parts[at["time_s"]]) if "time_s" in at else None
            if time_s is not None:
                if first_time_s is None:
                    first_time_s = time_s
                last_time_s = time_s

            for name, column in max_wanted:
                value = _finite(parts[column])
                if value is not None and (worst[name] is None or value > worst[name]):
                    worst[name] = value
            for name, column in min_wanted:
                value = _finite(parts[column])
                if value is not None and (worst[name] is None or value < worst[name]):
                    worst[name] = value

            if len(pose_wanted) == 6:
                pose = [_finite(parts[column]) for column in pose_wanted]
                if None not in pose:
                    error_m = math.dist(pose[:3], pose[3:])
                    if (worst["position_error_m"] is None or
                            error_m > worst["position_error_m"]):
                        worst["position_error_m"] = error_m

            # The following-error guard's own quantity: command minus
            # measured, unwrapped, because FillSample already brings the
            # measured angle within 180 deg of the command and Safety.cpp
            # compares them exactly like this.
            for joint, commanded_at, measured_at in joint_wanted:
                commanded = _finite(parts[commanded_at])
                measured = _finite(parts[measured_at])
                if commanded is None or measured is None:
                    continue
                gap = abs(commanded - measured)
                if (worst["following_error_deg"] is None or
                        gap > worst["following_error_deg"]):
                    worst["following_error_deg"] = gap
                    worst["following_error_joint"] = joint

            if "replan_advised" in at and parts[at["replan_advised"]].strip() == "1":
                replan_advised_rows += 1
            for name, column in edge_wanted:
                if parts[column].strip() != "1":
                    continue
                if len(edges) >= _MAX_EDGES:
                    edges_truncated = True
                    continue
                edge: dict[str, Any] = {
                    "event": name,
                    "time_s": time_s,
                    "cycle": _finite(parts[at["cycle"]]) if "cycle" in at else None,
                }
                if name == "traj_rejected" and "traj_start_error_deg" in at:
                    edge["start_error_deg"] = _finite(
                        parts[at["traj_start_error_deg"]])
                edges.append(edge)

    if header is None:
        return {"error": "no header row — the run wrote no data",
                "preamble": telemetry.parse_preamble(preamble_lines),
                "preamble_lines": preamble_lines}

    preamble = telemetry.parse_preamble(preamble_lines)
    session = find_session_dir(path)
    duration_s = (last_time_s - first_time_s
                  if first_time_s is not None and last_time_s is not None else None)

    return {
        "file": str(path.relative_to(root.resolve())),
        "path": str(path),
        "arm": run_arm(path) or preamble.get("arm"),
        "log_format": preamble.get("log_format"),
        "reference_source": preamble.get("reference_source"),
        "preamble": preamble,
        "preamble_lines": preamble_lines,
        "columns": len(header),
        "rows": rows,
        "duration_s": duration_s,
        "worst": worst,
        "replan_advised_rows": replan_advised_rows,
        "trajectory_edges": edges,
        "trajectory_edges_truncated": edges_truncated,
        "session": str(session.relative_to(root.resolve())) if session else None,
        "session_json": read_session_json(session) if session else None,
        "stop": _stop_of(preamble, session),
    }


def _stop_of(preamble: dict[str, str], session: Path | None) -> dict[str, Any]:
    """Why the run ended, from both places that say so.

    The CSV's trailer (exit_reason, exit_cycle) is written by the controller
    itself and is present even for a run started outside run_session.sh. The
    sentence from controller.log adds the joint and the limit, and only
    exists when there is a session directory to hold it.
    """
    stop: dict[str, Any] = {
        "exit_reason": preamble.get("exit_reason"),
        "exit_time_s": preamble.get("exit_time_s"),
        "exit_cycle": preamble.get("exit_cycle"),
        "faults_observed": preamble.get("faults_observed"),
        "line": None,
        "source": None,
    }
    if session is not None:
        reported = stop_reason(session)
        if reported is not None:
            stop["line"] = reported["line"]
            stop["source"] = reported["source"]
    return stop
