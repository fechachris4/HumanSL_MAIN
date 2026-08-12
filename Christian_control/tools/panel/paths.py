"""Where things are on disk.

Every other panel module imports its paths from here, so a moved directory
is a one-line change and no module invents its own layout.
"""

from pathlib import Path

# panel/ -> tools/ -> Christian_control/ -> repo root
REPO = Path(__file__).resolve().parents[3]

CHRISTIAN_CONTROL = REPO / "Christian_control"

BASIC_CONTROL = CHRISTIAN_CONTROL / "basic_control"
CONFIG_H = BASIC_CONTROL / "src" / "Config.h"
CONFIG_H_BACKUP = CONFIG_H.with_suffix(".h.panel.bak")
CONTROLLER_SRC = BASIC_CONTROL / "src"
CONTROLLER_BUILD = BASIC_CONTROL / "build"
CONTROLLER_BIN = CONTROLLER_BUILD / "controller"

PLANNER_BRIDGE = CHRISTIAN_CONTROL / "planner_bridge"
BRIDGE_SRC = PLANNER_BRIDGE / "src"
BRIDGE_TRAJECTORY_GENERATION = PLANNER_BRIDGE / "trajectory_generation"
BRIDGE_BUILD = PLANNER_BRIDGE / "build"
BRIDGE_BIN = BRIDGE_BUILD / "planner_bridge"
GOAL_YAML = PLANNER_BRIDGE / "config" / "goal.yaml"
PLANNER_YAML = PLANNER_BRIDGE / "config" / "planner.yaml"
RUN_SESSION_SH = PLANNER_BRIDGE / "scripts" / "run_session.sh"

URDF = BASIC_CONTROL / "config" / "GEN3_dual_mounted.urdf"

# The build-generated DH tables the browser uses for forward kinematics.
# run_session.sh checks these against the URDF for staleness; the right arm
# is described to the tool frame, the left to the flange.
DH_YAML = {
    "right": BRIDGE_BUILD / "config" / "dh_params_tool.yaml",
    "left": BRIDGE_BUILD / "config" / "dh_params_flange.yaml",
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


def target_pipe(arm: str) -> Path:
    """The named pipe the controller reads trajectories from, per arm.

    Mirrors config::ArmConfig::target_pipe_path.
    """
    return Path(f"/tmp/humansl_bridge_targets_{arm}")
