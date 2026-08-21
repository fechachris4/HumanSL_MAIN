"""Where things are on disk.

Every other panel module imports its paths from here, so a moved directory
is a one-line change and no module invents its own layout.
"""

from pathlib import Path

# panel/ -> Christian_control/ -> repo root
REPO = Path(__file__).resolve().parents[2]

CHRISTIAN_CONTROL = REPO / "Christian_control"

# The command pipeline (control/) and the program that runs it against the
# arm (runtime/) are separate directories; a rebuild has to watch both.
CONTROL = CHRISTIAN_CONTROL / "control"
RUNTIME = CHRISTIAN_CONTROL / "runtime"
CONFIG_H = CONTROL / "Config.h"
CONFIG_H_BACKUP = CONFIG_H.with_suffix(".h.panel.bak")
CONTROLLER_BUILD = RUNTIME / "build"
CONTROLLER_BIN = CONTROLLER_BUILD / "controller"
PLANNING = CHRISTIAN_CONTROL / "planning"
BRIDGE_SRC = PLANNING / "src"
BRIDGE_OPTIMISATION = PLANNING / "optimisation"
BRIDGE_BUILD = PLANNING / "build"
BRIDGE_BIN = BRIDGE_BUILD / "planner_bridge"
GOAL_YAML = PLANNING / "config" / "goal.yaml"
PLANNER_YAML = PLANNING / "config" / "planner.yaml"
JOINT_LIMITS_YAML = PLANNING / "config" / "joint_limits.yaml"
RUN_SESSION_SH = PLANNING / "scripts" / "run_session.sh"

URDF = CHRISTIAN_CONTROL / "model" / "GEN3_dual_mounted.urdf"

# The controller build's nested planner target generates the planner's DH
# tables. The panel reports only their existence/staleness; they never enter
# browser rendering.
DH_YAML = {
    "right": CONTROLLER_BUILD / "planning" / "config" / "dh_params_tool.yaml",
    "left": CONTROLLER_BUILD / "planning" / "config" / "dh_params_flange.yaml",
}

RUNS = REPO / "runs"

STATIC = Path(__file__).resolve().parent / "static"

# The panel's own record of the session it launched. Survives a panel
# restart so a running arm can be re-attached to rather than orphaned.
SESSION_STATE = Path("/tmp/humansl_panel_session.json")

# What the panel has computed for itself — at present the last plan solved
# from each arm's browser button. It lives in /tmp rather than under runs/
# because none of it is an experimental record: run_session.sh already
# archives the plan that a session actually sent, and a plan solved to look
# at must not be mistaken for one that ran.
PANEL_SCRATCH = Path("/tmp/humansl_panel")

ARMS = ("right", "left")


def panel_backup(target: Path) -> Path:
    """file.yaml -> file.yaml.panel.bak, beside the original. Written once,
    before the panel's first edit, so it holds the pre-panel state."""
    return target.with_name(target.name + ".panel.bak")
