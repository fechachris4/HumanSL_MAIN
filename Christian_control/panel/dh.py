"""Existence and staleness of the planner's generated DH tables.

The panel never reads table contents or performs robot FK. These files remain
planner build inputs, so diagnostics still report when they are missing or
older than the canonical URDF.
"""

from __future__ import annotations

from . import paths


def read(arm: str) -> dict[str, Any]:
    """The DH table for one arm, with the staleness run_session.sh checks.

    Returns {"arm", "exists", "stale", "source", "reason"}. A missing
    file is not an error here: the planner_bridge build produces it, and the
    panel should say so rather than fail.
    """
    path = paths.DH_YAML.get(arm)
    if path is None:
        return {"arm": arm, "exists": False, "stale": False,
                "source": None,
                "reason": f"unknown arm '{arm}' — expected right or left"}
    if not path.is_file():
        return {"arm": arm, "exists": False, "stale": True,
                "source": str(path),
                "reason": f"{path.name} has not been generated — build controller"}
    stale = False
    reason = ""
    if paths.URDF.is_file() and paths.URDF.stat().st_mtime > path.stat().st_mtime:
        stale = True
        reason = f"{path.name} is older than the URDF — rebuild controller"
    return {
        "arm": arm,
        "exists": True,
        "stale": stale,
        "source": str(path),
        "reason": reason,
    }
