"""The build-generated DH tables, read for the browser's forward kinematics.

The browser draws the arm itself rather than asking the server for link
positions every frame, so it needs the same Denavit-Hartenberg parameters the
planner uses. Those live in files generated from the canonical URDF at build
time (planner_bridge/tools/generate_dh_params), which is why nothing here
computes them: a second source of truth for the geometry is exactly the bug
the generator exists to prevent.

run_session.sh refuses a session whose generated table is older than the URDF.
This module reports that same staleness so the panel can say the drawing may
not match the robot, but it does not refuse anything — drawing a stale arm with
a warning is more useful than drawing nothing.
"""

from __future__ import annotations

import re
from typing import Any

from . import paths

# "  - joint_id: 1" opens an entry; the scalars that follow belong to it until
# the next one. The generated file is machine-written and stable, so a small
# line matcher is honest here — this is not a general YAML parser and must not
# grow into one.
_ENTRY_RE = re.compile(r"^\s*-\s*joint_id:\s*(\d+)\s*$")
_SCALAR_RE = re.compile(r'^\s+(\w+):\s*"?([^"\n]*?)"?\s*$')

_NUMERIC_FIELDS = ("a", "alpha", "d", "theta_offset")


def parse_dh_yaml(text: str) -> list[dict[str, Any]]:
    """Return one dict per joint, in joint_id order.

    Each dict carries a, alpha, d, theta_offset (radians, floats),
    joint_name and joint_type.
    """
    joints: list[dict[str, Any]] = []
    current: dict[str, Any] | None = None
    for line in text.splitlines():
        if line.lstrip().startswith("#"):
            continue
        opened = _ENTRY_RE.match(line)
        if opened:
            current = {"joint_id": int(opened.group(1))}
            joints.append(current)
            continue
        if current is None:
            continue
        scalar = _SCALAR_RE.match(line)
        if not scalar:
            continue
        key, raw = scalar.group(1), scalar.group(2).strip()
        if key == "joint_id":
            continue
        if key in _NUMERIC_FIELDS:
            try:
                current[key] = float(raw)
            except ValueError:
                current[key] = None
        else:
            current[key] = raw
    joints.sort(key=lambda j: j.get("joint_id", 0))
    return joints


def read(arm: str) -> dict[str, Any]:
    """The DH table for one arm, with the staleness run_session.sh checks.

    Returns {"arm", "joints", "exists", "stale", "source", "reason"}. A missing
    file is not an error here: the planner_bridge build produces it, and the
    panel should say so rather than fail.
    """
    path = paths.DH_YAML.get(arm)
    if path is None:
        return {"arm": arm, "joints": [], "exists": False, "stale": False,
                "source": None,
                "reason": f"unknown arm '{arm}' — expected right or left"}
    if not path.is_file():
        return {"arm": arm, "joints": [], "exists": False, "stale": True,
                "source": str(path),
                "reason": f"{path.name} has not been generated — build planner_bridge"}
    joints = parse_dh_yaml(path.read_text(errors="replace"))
    stale = False
    reason = ""
    if paths.URDF.is_file() and paths.URDF.stat().st_mtime > path.stat().st_mtime:
        stale = True
        reason = f"{path.name} is older than the URDF — rebuild planner_bridge"
    return {
        "arm": arm,
        "joints": joints,
        "exists": True,
        "stale": stale,
        "source": str(path),
        "reason": reason,
    }
