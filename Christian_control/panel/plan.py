"""goal.yaml, the offline solve, and reading back what the planner emitted.

Three jobs that belong together because they are the same question asked at
three moments: where the arm is being asked to go, what the planner made of
that, and what it actually put on the wire.

Nothing here can move the arm. solve() runs the standalone preview wrapper with
its text output captured to a temporary file; that file is never handed to the
controller, which uses the in-process typed planner path during a session.

The bridge's stderr is returned verbatim rather than summarised. It carries
the plan-initialisation quality, the validation report and the warning about
an inherited goal orientation — the diagnostics that used to scroll past in a
terminal and die with the scrollback.

There is no YAML library in this environment, so the goal file is hand-parsed
for exactly the keys the panel draws and no more. The file is heavily
commented and those comments are the documentation for the format, so the raw
text is what round-trips: an edit rewrites the text the operator typed, never
a file regenerated from the parsed structure.
"""

from __future__ import annotations

import json
import math
import re
import shutil
import subprocess
import tempfile
from pathlib import Path
from typing import Any, Callable

from . import paths, telemetry, yaml_text

# A goal file names arms exactly as the bridge's --arm does.
_ARM_KEYS = paths.ARMS

# What run_session.sh accepts in `session_arms:`. "both" is the session
# script's own idea; planner_bridge is always told one arm at a time.
_SESSION_ARMS = ("right", "left", "both")

# "  radius_m: 0.1" — a key, its indent, and whatever followed the colon. A
# block key ("box:", "path:") has nothing after the colon, which is how the
# parser below tells a nested block from a scalar.
_KEY_RE = re.compile(r"^(\s*)([A-Za-z_][A-Za-z0-9_]*)\s*:\s*(.*)$")

# JointTrajectory.h: TRAJ_BEGIN's count must be an integer in [2, 1000], and
# each row is fifteen numbers. Restated here so a preview can say why a block
# would be refused; the controller remains the authority that refuses it.
_MIN_TRAJECTORY_POINTS = 2
_MAX_TRAJECTORY_POINTS = 1000
_TRAJECTORY_ROW_FIELDS = 15

# The bridge densifies to about 1 kHz and runs the optimiser for up to a
# thousand iterations, so a solve is tens of seconds, not milliseconds. The
# timeout exists only so a wedged bridge cannot hold a browser request open
# for ever.
_SOLVE_TIMEOUT_S = 300.0

# BridgeMain.cpp's documented exit codes, so a failed solve says what kind of
# failure it was instead of showing a bare number.
_EXIT_MEANING = {
    0: "targets emitted",
    1: "bad arguments",
    2: "start state unavailable",
    3: "solve failed",
    4: "validation rejected the plan",
}

# The last plan solved for each arm, so the run screen can draw it without
# re-solving. Mirrored to PANEL_SCRATCH so a panel restart does not lose it.
_LAST_PLAN: dict[str, dict[str, Any]] = {}


# ---- goal.yaml ---------------------------------------------------------


_strip_comment = yaml_text.strip_comment
_scalar = yaml_text.scalar


def parse_goal(text: str) -> dict[str, Any]:
    """The parts of a goal file the panel draws.

    Returns {"session_arms": str|None, "arms": {arm: {...}}}. Each arm block
    carries whichever of frame, goal, orientation_rpy_deg and path it
    declares. A retired `box` key is retained only long enough for validation
    to reject it with migration guidance. Commented-out keys are invisible.
    """
    session_arms: str | None = None
    arms: dict[str, dict[str, Any]] = {}
    arm: str | None = None
    block: str | None = None   # "box" or "path" while inside one
    block_indent = 0

    for raw in text.splitlines():
        line = _strip_comment(raw)
        if not line.strip():
            continue
        matched = _KEY_RE.match(line)
        if not matched:
            continue
        indent, key, value = len(matched.group(1)), matched.group(2), matched.group(3)

        if indent == 0:
            block = None
            arm = key if key in _ARM_KEYS else None
            if arm is not None:
                arms[arm] = {}
            elif key == "session_arms":
                session_arms = value.strip() or None
            continue

        if arm is None:
            continue
        if block is not None and indent > block_indent:
            arms[arm][block][key] = _scalar(value)
            continue
        block = None
        if key in ("box", "path") and not value.strip():
            block, block_indent = key, indent
            arms[arm][key] = {}
            continue
        arms[arm][key] = _scalar(value)

    return {"session_arms": session_arms, "arms": arms}


def read_goal(path: Path | None = None) -> dict[str, Any]:
    """The goal file as text, and as much structure as the panel needs.

    The text is what an editor shows and what write_goal takes back; the
    parsed half is for drawing. A missing file is reported rather than raised,
    because the panel should say the file is not there.
    """
    target = path or paths.GOAL_YAML
    if not target.is_file():
        return {"text": "", "parsed": {"session_arms": None, "arms": {}},
                "path": str(target), "error": f"{target} does not exist"}
    text = target.read_text(errors="replace")
    return {"text": text, "parsed": parse_goal(text), "path": str(target)}


def session_arms(path: Path | None = None) -> str | None:
    """Which arm(s) run_session.sh drives when it is given no --arm flag."""
    return read_goal(path)["parsed"]["session_arms"]


def _three_finite(value: Any) -> bool:
    return (isinstance(value, list) and len(value) == 3 and
            all(isinstance(v, float) and math.isfinite(v) for v in value))


def validate_goal(text: str) -> list[str]:
    """Everything wrong with a proposed goal file, as sentences.

    Deliberately narrow. It checks the shapes whose corruption would make the
    session script or the bridge fail late and obscurely, and leaves every
    planning judgement — reachability, clearance, whether the orientation is
    sensible — to the bridge, which is the thing that can actually decide.
    """
    parsed = parse_goal(text)
    problems: list[str] = []

    arms_value = parsed["session_arms"]
    if arms_value is None:
        problems.append(
            "session_arms: is missing — run_session.sh reads it to decide "
            "which arm(s) to drive, and refuses a session without it")
    elif arms_value not in _SESSION_ARMS:
        problems.append(
            f"session_arms: is '{arms_value}' — it must be right, left or both")

    for arm, block in parsed["arms"].items():
        if "goal" in block and not _three_finite(block["goal"]):
            problems.append(
                f"{arm}: goal must be three finite numbers "
                f"[x, y, z] in metres, not {block['goal']!r}")
        if "box" in block:
            problems.append(
                f"{arm}: box is retired — edit obstacles.scene in planner.yaml")
    return problems


def _backup_of(target: Path) -> Path:
    return paths.panel_backup(target)


def write_goal(text: str, path: Path | None = None) -> tuple[bool, str]:
    """Validate, then replace the goal file's text.

    On any problem the file is left byte-for-byte unchanged and the reason is
    returned, so a half-valid edit can never become the target of a run.
    """
    target = path or paths.GOAL_YAML
    problems = validate_goal(text)
    if problems:
        return False, "; ".join(problems)
    if not target.parent.is_dir():
        return False, f"{target.parent} does not exist"
    backup = _backup_of(target)
    if target.is_file() and not backup.exists():
        shutil.copy2(target, backup)
    target.write_text(text)
    return True, str(target)


# The order path keys are (re)built in, matching the file's own layout.
_PATH_KEYS = ("type", "centre", "radius_m", "normal", "duration_s",
              "orientation", "orientation_rpy_deg")
def _set_key(text: str, path_t: tuple[str, ...], value: Any) -> str | None:
    """Replace the key's value, or insert the key at its parent's block end.
    Insertion is how a commented-out example key (invisible to the parser)
    becomes real."""
    if isinstance(value, (list, tuple)):
        value = [float(v) if isinstance(v, (int, float)) and
                  not isinstance(v, bool) else v for v in value]
    rendered = yaml_text.render(value)
    replaced = yaml_text.replace_value(text, path_t, rendered)
    if replaced is not None:
        return replaced
    return yaml_text.insert_key(text, path_t[:-1], path_t[-1], rendered)


def _set_block(text: str, parent: tuple[str, ...], key: str, keys: tuple,
               values: dict) -> str | None:
    """Ensure `parent.key:` exists, then set each of `keys` present in
    `values` under it, in order."""
    if yaml_text.locate(text.split("\n"), parent + (key,)) is None:
        text = yaml_text.insert_key(text, parent, key, "")
        if text is None:
            return None
    for k in keys:
        if k in values and values[k] is not None:
            text = _set_key(text, parent + (key, k), values[k])
            if text is None:
                return None
    return text


def _is_filled(value: Any) -> bool:
    """A field the operator actually typed something into. An empty string
    survives every `is not None` test and writes `radius_m:` with no value
    after it — a file the bridge cannot read."""
    if value is None:
        return False
    if isinstance(value, str):
        return bool(value.strip())
    if isinstance(value, (list, tuple)):
        return bool(value) and all(_is_filled(v) for v in value)
    return True


def _missing_circle_field(path_fields: dict) -> str | None:
    """The first circle field left blank, or None.

    validate_goal deliberately checks no path keys — planning judgements are
    the bridge's — so nothing else would catch a blank radius before the file
    was saved with a success message and the next solve failed on it.
    """
    for key in ("type", "centre", "radius_m", "normal", "duration_s",
                "orientation"):
        if not _is_filled(path_fields.get(key)):
            return key
    orientation = str(path_fields.get("orientation", "")).strip()
    if orientation == "fixed" and not _is_filled(
            path_fields.get("orientation_rpy_deg")):
        return "orientation_rpy_deg"
    return None


def write_goal_fields(arm: str, fields: dict,
                      path: Path | None = None) -> tuple[bool, str]:
    """Edit one arm's block of the goal file in place from structured fields.

    Applies the edits to the text, then runs the SAME validate_goal gate the
    raw editor goes through — one gate, not two opinions — and only a text
    that passes replaces the file. mode decides which of goal/path survives,
    so the bridge's "mutually exclusive" refusal is unreachable from here.
    """
    if arm not in paths.ARMS:
        return False, f"unknown arm {arm!r}"
    if "box" in fields:
        return False, "box is retired — edit obstacles.scene in planner.yaml"
    mode = fields.get("mode")
    if mode not in ("point", "circle"):
        return False, "mode must be point or circle"
    target = path or paths.GOAL_YAML
    if not target.is_file():
        return False, f"{target} does not exist"
    text = target.read_text()
    if yaml_text.locate(text.split("\n"), (arm,)) is None:
        return False, f"{target.name} has no {arm}: block"

    def step(new_text: str | None, what: str) -> str:
        if new_text is None:
            raise ValueError(f"could not {what}")
        return new_text

    try:
        if mode == "point":
            if yaml_text.locate(text.split("\n"), (arm, "path")) is not None:
                text = step(yaml_text.remove_key(text, (arm, "path")),
                            "remove the path block")
            if fields.get("goal") is not None:
                text = step(_set_key(text, (arm, "goal"), fields["goal"]),
                            "set the goal")
        else:
            path_fields = fields.get("path") or {}
            blank = _missing_circle_field(path_fields)
            if blank is not None:
                return False, (
                    f"{arm}: the circle needs {blank} — it was left blank, and "
                    "a path missing it cannot be planned")
            if yaml_text.locate(text.split("\n"), (arm, "goal")) is not None:
                text = step(yaml_text.remove_key(text, (arm, "goal")),
                            "remove the goal key")
            text = step(_set_block(text, (arm,), "path", _PATH_KEYS,
                                   path_fields), "build the path block")

        if "frame" in fields and fields["frame"] is not None:
            text = step(_set_key(text, (arm, "frame"), fields["frame"]),
                        "set the frame")

        if "orientation_rpy_deg" in fields:
            rpy = fields["orientation_rpy_deg"]
            if rpy is None:
                if yaml_text.locate(text.split("\n"),
                                    (arm, "orientation_rpy_deg")) is not None:
                    text = step(yaml_text.remove_key(
                        text, (arm, "orientation_rpy_deg")),
                        "remove the orientation")
            else:
                text = step(_set_key(text, (arm, "orientation_rpy_deg"), rpy),
                            "set the orientation")

    except ValueError as exc:
        return False, str(exc)

    problems = validate_goal(text)
    if problems:
        return False, "; ".join(problems)
    backup = _backup_of(target)
    if target.is_file() and not backup.exists():
        shutil.copy2(target, backup)
    target.write_text(text)
    return True, str(target)


# ---- the trajectory block ---------------------------------------------


def parse_trajectory_block(text: str) -> dict[str, Any]:
    """Read a TRAJ_BEGIN/TRAJ_END block the way the controller reads it.

    The wire format (TrajectoryEmit.cpp writes it, JointTrajectory.cpp reads
    it) is `TRAJ_BEGIN <count>`, then count rows of
    `<t_s> <q1..q7 deg> <v1..v7 deg/s>`, then `TRAJ_END`.

    Returns points, duration_s, rows as [t, q1..q7] for drawing, and the first
    and last joint vectors. `error` is set when there is no usable block at
    all; `warnings` carries the things JointTrajectory.cpp would reject the
    block for. The preview only reports them — the controller is still the
    only thing that accepts or refuses a plan, and this must never be read as
    a second opinion on that.
    """
    empty: dict[str, Any] = {"points": 0, "duration_s": 0.0, "rows": [],
                             "start_deg": [], "end_deg": [],
                             "declared_points": 0, "warnings": [], "error": None}
    lines = text.splitlines()
    begin = next((i for i, line in enumerate(lines)
                  if line.strip().startswith("TRAJ_BEGIN")), None)
    if begin is None:
        empty["error"] = "no TRAJ_BEGIN line — the bridge emitted no trajectory"
        return empty

    warnings: list[str] = []
    header = lines[begin].split()
    declared = 0
    if len(header) != 2:
        warnings.append("TRAJ_BEGIN should carry exactly one count")
    try:
        declared = int(header[1])
    except (IndexError, ValueError):
        empty["error"] = f"TRAJ_BEGIN has no integer count: {lines[begin].strip()!r}"
        return empty
    if not _MIN_TRAJECTORY_POINTS <= declared <= _MAX_TRAJECTORY_POINTS:
        warnings.append(
            f"TRAJ_BEGIN count {declared} is outside the controller's "
            f"[{_MIN_TRAJECTORY_POINTS}, {_MAX_TRAJECTORY_POINTS}]")

    rows: list[list[float]] = []
    velocities: list[list[float]] = []
    ended = False
    for line in lines[begin + 1:]:
        stripped = line.strip()
        if not stripped:
            continue
        if stripped.startswith("TRAJ_END"):
            ended = True
            break
        fields = [float(f) for f in stripped.split()] if _all_numeric(stripped) else None
        if fields is None or len(fields) != _TRAJECTORY_ROW_FIELDS:
            warnings.append(
                f"row {len(rows) + 1} is not fifteen numbers: {stripped[:60]!r}")
            continue
        if not all(math.isfinite(f) for f in fields):
            warnings.append(f"row {len(rows) + 1} has a non-finite number")
            continue
        rows.append(fields[:8])
        velocities.append(fields[8:])

    if not ended:
        warnings.append("no TRAJ_END — the block is truncated")
    if not rows:
        empty["error"] = "TRAJ_BEGIN was followed by no usable rows"
        empty["warnings"] = warnings
        empty["declared_points"] = declared
        return empty
    if len(rows) != declared:
        warnings.append(f"{len(rows)} rows for a declared count of {declared}")
    if abs(rows[0][0]) > 1e-9:
        warnings.append(f"first point is at t = {rows[0][0]} s, not 0")
    if any(rows[i][0] <= rows[i - 1][0] for i in range(1, len(rows))):
        warnings.append("times are not strictly increasing")

    return {
        "points": len(rows),
        "declared_points": declared,
        "duration_s": rows[-1][0],
        "rows": rows,
        "start_deg": rows[0][1:],
        "end_deg": rows[-1][1:],
        "max_speed_deg_s": max((max(abs(v) for v in row) for row in velocities),
                               default=0.0),
        "warnings": warnings,
        "error": None,
    }


def _all_numeric(line: str) -> bool:
    """True when every whitespace-separated token parses as a number."""
    for token in line.split():
        try:
            float(token)
        except ValueError:
            return False
    return True


# ---- solving -----------------------------------------------------------


def newest_run_log(arm: str, runs_root: Path | None = None) -> Path | None:
    """The run log the bridge would read a start state from, if not told one.

    StartState.cpp searches <runs>/<day>/loop_log_<arm>*.csv and takes the
    newest by write time; this looks in the same two levels so the panel can
    name the file before the bridge runs rather than reporting exit code 2
    afterwards.
    """
    root = runs_root or paths.RUNS
    if not root.is_dir():
        return None
    candidates = list(root.glob(f"*/loop_log_{arm}*.csv"))
    if not candidates:
        return None
    return max(candidates, key=lambda p: p.stat().st_mtime)


def start_state_options(arm: str, limit: int = 8) -> list[dict[str, Any]]:
    """The runs this arm could be planned from, newest first.

    Each entry carries its data-row count and the last measured joint angles,
    so the browser can show which runs are usable, offer an earlier one when
    the newest is empty, and prefill the typed-angles field from a real
    configuration rather than from zeros.
    """
    root = paths.RUNS
    if not root.is_dir():
        return []
    candidates = sorted(root.glob(f"*/loop_log_{arm}*.csv"),
                        key=lambda p: p.stat().st_mtime, reverse=True)[:limit]
    options: list[dict[str, Any]] = []
    for path in candidates:
        # Neither of these reads the whole file. Run logs reach gigabytes, and
        # this is on a request path: an exact row count over a 2.5 GB CSV once
        # took the panel out entirely.
        usable = telemetry.has_data_rows(path)
        stat = path.stat()
        options.append({
            "file": str(path.relative_to(paths.REPO)),
            "name": path.name,
            "usable": usable,
            "size_mb": round(stat.st_size / 1e6, 1),
            "mtime": stat.st_mtime,
            "final_deg": telemetry.final_measured_deg(path) if usable else None,
        })
    return options


def _run_bridge(command: list[str], stdout_path: Path) -> tuple[int, str]:
    """Run planner_bridge, its trajectory to a file and its report to stderr.

    The only place the panel executes a binary from the robot side of the
    repository. It is safe because the planner talks to nothing: it reads
    YAML and a CSV, and writes a text block for preview only. The production
    session does not invoke this wrapper or pass its file to the controller.
    """
    with stdout_path.open("w") as out:
        finished = subprocess.run(command, stdout=out, stderr=subprocess.PIPE,
                                  text=True, timeout=_SOLVE_TIMEOUT_S)
    return finished.returncode, finished.stderr


def solve(arm: str, start_deg: list[float] | None = None,
          run: Callable[[list[str], Path], tuple[int, str]] = _run_bridge,
          ) -> dict[str, Any]:
    """Plan one arm's move and return the trajectory without sending it.

    With no start_deg the bridge reads the newest run log, so an offline solve
    is only possible once the controller has recorded a run; that is stated
    here rather than left to surface as exit code 2. `run` is the seam the
    tests replace: no test executes the real bridge.
    """
    if arm not in _ARM_KEYS:
        return _failed(arm, [], f"unknown arm '{arm}' — expected right or left")
    if not paths.BRIDGE_BIN.is_file():
        return _failed(arm, [], f"{paths.BRIDGE_BIN} has not been built")

    command = [str(paths.BRIDGE_BIN), "--arm", arm]
    start_source = ""
    if start_deg is not None:
        if len(start_deg) != 7 or not all(
                isinstance(v, (int, float)) and math.isfinite(v) for v in start_deg):
            return _failed(arm, command,
                           "start_deg must be seven finite joint angles in degrees")
        command += ["--start-deg"] + [repr(float(v)) for v in start_deg]
        start_source = "start_deg given by the panel"
    else:
        newest = newest_run_log(arm)
        if newest is None:
            return _failed(
                arm, command,
                f"no runs/*/loop_log_{arm}*.csv to read a start state from — "
                "the bridge needs measured joint angles, so run the controller "
                "once or solve with explicit start angles")
        # The bridge takes the NEWEST log, and a session that failed before its
        # control loop started leaves a newer, emptier file than the good run
        # before it. Saying so here turns the bridge's correct but bare "no
        # complete data row" into something that names the file and the way out.
        # has_data_rows, not a count: this runs before every solve and the
        # newest log can be gigabytes.
        if not telemetry.has_data_rows(newest):
            return _failed(
                arm, command,
                f"the newest {arm}-arm run log has no data rows, so there are "
                f"no measured joint angles to plan from: {newest.name}. That "
                "happens when a session started and its control loop never ran "
                "— the controller log for that session says why. Choose an "
                "earlier run, or type the start angles.")
        start_source = str(newest)

    with tempfile.TemporaryDirectory(prefix="humansl_panel_solve_") as scratch:
        emitted = Path(scratch) / f"plan_{arm}.traj"
        try:
            code, stderr = run(command, emitted)
        except subprocess.TimeoutExpired:
            return _failed(arm, command,
                           f"the bridge did not finish within {_SOLVE_TIMEOUT_S:.0f} s")
        except OSError as error:
            return _failed(arm, command, f"could not run the bridge: {error}")
        block = emitted.read_text(errors="replace") if emitted.is_file() else ""

    parsed = parse_trajectory_block(block)
    plan = {
        "arm": arm,
        "start_source": start_source,
        "returncode": code,
        "exit_meaning": _EXIT_MEANING.get(code, "unrecognised exit code"),
        **parsed,
    }
    ok = code == 0 and parsed["error"] is None
    if ok:
        _remember(arm, plan)
    return {"ok": ok, "plan": plan, "stderr": stderr, "command": command}


def _failed(arm: str, command: list[str], reason: str) -> dict[str, Any]:
    """A solve that never reached the bridge, in the shape of one that did.

    Same keys either way, so the browser draws a refusal and a failure with
    one piece of code and cannot miss a field it did not expect.
    """
    return {
        "ok": False,
        "plan": {"arm": arm, "start_source": "", "returncode": None,
                 "exit_meaning": "the bridge was not run", "points": 0,
                 "declared_points": 0, "duration_s": 0.0, "rows": [],
                 "start_deg": [], "end_deg": [], "warnings": [],
                 "error": reason},
        "stderr": "",
        "command": command,
    }


# ---- the last plan, for the run screen ---------------------------------


def _plan_file(arm: str) -> Path:
    return paths.PANEL_SCRATCH / f"plan_{arm}.json"


def _remember(arm: str, plan: dict[str, Any]) -> None:
    """Keep the plan in memory and on disk.

    On disk because the run screen draws the plan behind the arm, and losing
    it to a panel restart mid-session would blank the one part of the display
    that says where the arm was going.
    """
    _LAST_PLAN[arm] = plan
    try:
        paths.PANEL_SCRATCH.mkdir(parents=True, exist_ok=True)
        _plan_file(arm).write_text(json.dumps(plan))
    except OSError:
        pass  # an unwritable scratch dir costs redraw after a restart, no more


def last_plan(arm: str) -> dict[str, Any]:
    """The most recent solved plan for one arm, or an empty one saying so."""
    if arm in _LAST_PLAN:
        return _LAST_PLAN[arm]
    path = _plan_file(arm)
    if path.is_file():
        try:
            plan = json.loads(path.read_text())
        except (OSError, json.JSONDecodeError):
            plan = None
        if isinstance(plan, dict):
            _LAST_PLAN[arm] = plan
            return plan
    return {"arm": arm, "start_source": "", "returncode": None,
            "exit_meaning": "the bridge was not run", "points": 0,
            "declared_points": 0, "duration_s": 0.0, "rows": [],
            "start_deg": [], "end_deg": [], "warnings": [],
            "error": "no plan solved yet"}
