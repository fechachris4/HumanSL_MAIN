"""Run scripts/plot_*.py against a chosen run log and report the PNGs.

Four scripts, four different CLIs (checked directly against each script
rather than guessed): plot_run.py and plot_world_hold.py take a CSV and an
output directory and print one PNG path per line; plot_joint.py takes a
required joint number and a single --out file; plot_move.py takes only a
CSV and always writes <stem>_move.png next to it, alongside printed
tracking stats. One small command-building function per script (below)
covers that, rather than a generic "plugin" abstraction for four known
tools.

Output discovery does not depend on any of those conventions: every PNG in
the run's directory is stat()'d before and after the subprocess runs, and
anything new or newer is reported as this run's output. That is what makes
plot_move.py's fixed, overwrite-in-place filename work the same as the
other three scripts' printed paths.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path
from typing import Any

from . import paths
from .runs import _resolve_run

SCRIPTS_DIR = paths.BASIC_CONTROL / "scripts"
TIMEOUT_S = 90


def _plot_run(csv_path: Path, params: dict) -> list[str]:
    return [sys.executable, "plot_run.py", str(csv_path), "-o", str(csv_path.parent)]


def _plot_world_hold(csv_path: Path, params: dict) -> list[str]:
    return [sys.executable, "plot_world_hold.py", str(csv_path),
            "-o", str(csv_path.parent)]


def _plot_joint(csv_path: Path, params: dict) -> list[str]:
    joint = params.get("joint")
    if joint not in {"1", "2", "3", "4", "5", "6", "7"}:
        raise ValueError("joint must be 1-7")
    out = csv_path.with_name(f"{csv_path.stem}_joint{joint}.png")
    return [sys.executable, "plot_joint.py", joint, "--csv", str(csv_path),
            "--out", str(out)]


def _plot_move(csv_path: Path, params: dict) -> list[str]:
    return [sys.executable, "plot_move.py", str(csv_path)]


# id -> (label, one line building this script's argv, its required params).
# A flat table of four known scripts, not an extensible registry — add a
# fifth entry by hand when a fifth script exists, the same way the other
# four were added.
SCRIPTS: dict[str, dict[str, Any]] = {
    "plot_run": {
        "label": "Requested vs sent vs measured, per joint",
        "command": _plot_run,
        "params": [],
    },
    "plot_world_hold": {
        "label": "Vicon health + world-frame hold",
        "command": _plot_world_hold,
        "params": [],
    },
    "plot_joint": {
        "label": "One joint: commanded vs measured",
        "command": _plot_joint,
        "params": [{"name": "joint", "type": "select",
                    "options": ["1", "2", "3", "4", "5", "6", "7"],
                    "required": True}],
    },
    "plot_move": {
        "label": "Move overview + per-joint tracking stats",
        "command": _plot_move,
        "params": [],
    },
}


def list_scripts() -> list[dict[str, Any]]:
    return [{"id": script_id, "label": spec["label"], "params": spec["params"]}
            for script_id, spec in SCRIPTS.items()]


def _png_snapshot(directory: Path) -> dict[Path, float]:
    return {p: p.stat().st_mtime for p in directory.glob("*.png")}


def run(script_id: str, run_rel: str, params: dict,
       runs_root: Path | None = None) -> dict[str, Any]:
    """Run one script against one run CSV. Never raises — every failure
    mode (unknown script, unknown run, bad param, timeout, non-zero exit,
    no PNG produced) returns {"ok": False, "error": ...} instead, so the
    panel always has something to show rather than a 500."""
    spec = SCRIPTS.get(script_id)
    if spec is None:
        return {"ok": False, "error": f"unknown script {script_id!r}"}

    root = runs_root or paths.RUNS
    csv_path = _resolve_run(run_rel, root)
    if csv_path is None:
        return {"ok": False, "error": f"unknown run {run_rel!r}"}

    for param in spec["params"]:
        if param.get("required") and not params.get(param["name"]):
            return {"ok": False,
                    "error": f"missing required parameter {param['name']!r}"}

    try:
        command = spec["command"](csv_path, params)
    except ValueError as exc:
        return {"ok": False, "error": str(exc)}

    before = _png_snapshot(csv_path.parent)
    try:
        result = subprocess.run(command, cwd=SCRIPTS_DIR, capture_output=True,
                                text=True, timeout=TIMEOUT_S)
    except subprocess.TimeoutExpired:
        return {"ok": False,
                "error": f"timed out after {TIMEOUT_S}s — the log may be "
                "unusually large (a multi-hour session can be millions of "
                "rows; every script here loads the whole file at once)"}
    except OSError as exc:
        return {"ok": False, "error": f"could not start the script: {exc}"}
    after = _png_snapshot(csv_path.parent)

    changed = sorted((p for p, mtime in after.items()
                      if mtime > before.get(p, -1.0)),
                     key=lambda p: after[p])

    if result.returncode != 0:
        return {"ok": False,
                "error": f"exited with code {result.returncode}",
                "stdout": result.stdout, "stderr": result.stderr}
    if not changed:
        return {"ok": False,
                "error": "the script ran but produced no PNG "
                "(matplotlib may be missing in this environment)",
                "stdout": result.stdout, "stderr": result.stderr}

    return {"ok": True,
            "images": [str(p.relative_to(paths.REPO)) for p in changed],
            "stdout": result.stdout, "stderr": result.stderr}
