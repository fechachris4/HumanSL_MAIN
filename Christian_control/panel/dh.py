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
import math
from typing import Any

from . import paths

# "  - joint_id: 1" opens an entry; the scalars that follow belong to it until
# the next one. The generated file is machine-written and stable, so a small
# line matcher is honest here — this is not a general YAML parser and must not
# grow into one.
_ENTRY_RE = re.compile(r"^\s*-\s*joint_id:\s*(\d+)\s*$")
_SCALAR_RE = re.compile(r'^\s+(\w+):\s*"?([^"\n]*?)"?\s*$')

_NUMERIC_FIELDS = ("a", "alpha", "d", "theta_offset")

_MOUNTING_RE = re.compile(
    r"^\s*(base_separation_m|mount_tilt_rad|mount_origin):\s*([^#\n]+?)\s*$")


def parse_mounting_yaml(text: str) -> dict[str, dict[str, list[float]]]:
    """Read the canonical fixed mount placement for both arm bases.

    The YAML deliberately stores the shared separation and mirrored tilt,
    rather than two copied transforms. This parser expands those values into
    the browser wire shape: metres in ``xyz`` and radians in ``rpy``.
    """
    values: dict[str, str] = {}
    for line in text.splitlines():
        match = _MOUNTING_RE.match(line)
        if match:
            values[match.group(1)] = match.group(2).strip()

    try:
        separation = float(values["base_separation_m"])
        tilt = float(values["mount_tilt_rad"])
    except (KeyError, ValueError) as error:
        raise ValueError("mounting YAML needs finite separation and tilt") from error
    if not math.isfinite(separation) or separation <= 0:
        raise ValueError("base_separation_m must be finite and positive")
    if not math.isfinite(tilt):
        raise ValueError("mount_tilt_rad must be finite")
    if values.get("mount_origin") != "base_midpoint":
        raise ValueError("mount_origin must be base_midpoint")

    half = separation / 2.0
    return {
        "right": {"xyz": [0.0, -half, 0.0], "rpy": [tilt, 0.0, 0.0]},
        "left": {"xyz": [0.0, half, 0.0], "rpy": [-tilt, 0.0, 0.0]},
    }


def read_mounting() -> tuple[dict[str, dict[str, list[float]]] | None, str]:
    """Return canonical mount geometry and a user-readable failure reason."""
    if not paths.MOUNTING_YAML.is_file():
        return None, f"{paths.MOUNTING_YAML.name} is missing"
    try:
        return parse_mounting_yaml(paths.MOUNTING_YAML.read_text(errors="replace")), ""
    except ValueError as error:
        return None, str(error)


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
    mount_from_base, mount_reason = read_mounting()
    path = paths.DH_YAML.get(arm)
    if path is None:
        return {"arm": arm, "joints": [], "exists": False, "stale": False,
                "source": None, "mount_from_base": mount_from_base,
                "reason": f"unknown arm '{arm}' — expected right or left"}
    if not path.is_file():
        return {"arm": arm, "joints": [], "exists": False, "stale": True,
                "source": str(path), "mount_from_base": mount_from_base,
                "reason": mount_reason or f"{path.name} has not been generated — build controller"}
    joints = parse_dh_yaml(path.read_text(errors="replace"))
    stale = False
    reason = mount_reason
    if paths.URDF.is_file() and paths.URDF.stat().st_mtime > path.stat().st_mtime:
        stale = True
        reason = f"{path.name} is older than the URDF — rebuild controller"
    return {
        "arm": arm,
        "joints": joints,
        "exists": True,
        "stale": stale,
        "source": str(path),
        "mount_from_base": mount_from_base,
        "reason": reason,
    }
