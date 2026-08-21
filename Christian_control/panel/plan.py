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

import math
import re
import shutil
from pathlib import Path
from typing import Any

from . import paths, yaml_text

# The panel no longer owns an offline planner preview. Kept as a harmless
# compatibility name for older persistence-boundary tests and callers.
solve = None

# A goal file names arms exactly as the bridge's --arm does.
_ARM_KEYS = paths.ARMS

# What run_session.sh accepts in `session_arms:`. "both" is the session
# script's own idea; planner_bridge is always told one arm at a time.
_SESSION_ARMS = ("right", "left", "both")

# "  radius_m: 0.1" — a key, its indent, and whatever followed the colon. A
# block key ("box:", "path:") has nothing after the colon, which is how the
# parser below tells a nested block from a scalar.
_KEY_RE = re.compile(r"^(\s*)([A-Za-z_][A-Za-z0-9_]*)\s*:\s*(.*)$")

# A deliberately narrow supplement to _KEY_RE: YAML permits quoted mapping
# keys, but this hand-parser otherwise understands only the repository's bare
# keys. Retirement must not be bypassable by spelling box as "box" or 'box'.
# Validation applies this only while inside a known top-level arm block, after
# comments have been stripped.
_RETIRED_BOX_KEY_RE = re.compile(r"^\s+(?:box|\"box\"|'box')\s*:")

# ---- goal.yaml ---------------------------------------------------------


_strip_comment = yaml_text.strip_comment
_scalar = yaml_text.scalar


def parse_goal(text: str) -> dict[str, Any]:
    """The parts of a goal file the panel draws.

    Returns {"session_arms": str|None, "arms": {arm: {...}}}. Each arm block
    carries whichever of frame, goal, orientation_rpy_deg and path it
    declares. A bare retired `box` key is retained only long enough for
    validation to reject it; quoted retired spellings are detected separately
    without broadening this into a general YAML parser. Comments are invisible.
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


def _arms_with_retired_box_key(text: str) -> set[str]:
    """Known arm blocks containing an active bare or quoted ``box:`` key."""
    found: set[str] = set()
    arm: str | None = None
    for raw in text.splitlines():
        line = _strip_comment(raw)
        if not line.strip():
            continue
        key = _KEY_RE.match(line)
        if key and not key.group(1):
            arm = key.group(2) if key.group(2) in _ARM_KEYS else None
            continue
        if arm is not None and _RETIRED_BOX_KEY_RE.match(line):
            found.add(arm)
    return found


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
    retired_box_arms = _arms_with_retired_box_key(text)
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
        if "box" in block or arm in retired_box_arms:
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
