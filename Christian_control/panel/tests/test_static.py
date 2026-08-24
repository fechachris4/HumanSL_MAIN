from pathlib import Path
import re

from Christian_control.runtime.scripts import runlog

STATIC = Path(__file__).resolve().parents[1] / "static"


def test_planning_selector_has_only_current_worker_or_hold_modes():
    """Breaks if the retired baseline planner becomes selectable again."""
    panel_root = STATIC.parent
    html = (STATIC / "index.html").read_text()
    selector = re.search(
        r'<select id="plan-select">(.*?)</select>', html, re.DOTALL)
    assert selector is not None
    assert re.findall(r'<option value="([^"]+)">', selector.group(1)) == [
        "on", "off"]

    active_sources = (
        STATIC / "index.html",
        STATIC / "panel.js",
        panel_root / "server.py",
        panel_root / "session.py",
        panel_root.parent / "planning" / "scripts" / "run_session.sh",
        panel_root.parent / "planning" / "CMakeLists.txt",
        panel_root.parent / "planning" / "src" / "PlannerRuntime.h",
        panel_root.parent / "runtime" / "MainArgs.h",
        panel_root.parent / "runtime" / "MainArgs.cpp",
        panel_root.parent / "runtime" / "Main.cpp",
        panel_root.parent / "runtime" / "CMakeLists.txt",
        panel_root.parent / "runtime" / "InProcessPlanner.h",
        panel_root.parent / "runtime" / "InProcessPlanner.cpp",
    )
    active_text = "\n".join(path.read_text() for path in active_sources)
    for marker in ("5abc1b2c", "SolveBaselineBridgeForRequest",
                   "--baseline-bridge"):
        assert marker not in active_text
    assert not (panel_root.parent / "planning" / "src" /
                "BaselineBridge.h").exists()
    assert not (panel_root.parent / "planning" / "src" /
                "BaselineBridge.cpp").exists()


def test_run_scene_has_numeric_torso_cylinder_editor():
    html = (STATIC / "index.html").read_text()
    for element_id in (
        "scene-torso-create",
        "scene-torso-x",
        "scene-torso-y",
        "scene-torso-z",
        "scene-torso-radius",
        "scene-torso-height",
        "scene-torso-reset",
        "scene-torso-save",
        "scene-torso-state",
    ):
        assert f'id="{element_id}"' in html
    for label in ("x [m]", "y [m]", "z [m]", "radius [m]", "height [m]"):
        assert f"<label>{label} " in html


def test_scene_editor_stays_visible_without_telemetry():
    script = (STATIC / "panel.js").read_text()
    start = script.index("function renderRun()")
    end = script.index("\nfunction ", start + 1)
    body = script[start:end]

    assert "document.querySelector('.run-grid').hidden = empty" not in body


def test_blank_scene_numbers_are_not_coerced_to_zero():
    script = (STATIC / "panel.js").read_text()
    start = script.index("function torsoFromInputs()")
    end = script.index("\nfunction ", start + 1)
    body = script[start:end]

    assert ".value.trim()" in body
    assert "value === ''" in body


def test_scene_save_uses_only_the_scene_endpoint():
    script = (STATIC / "panel.js").read_text()
    start = script.index("async function saveScene()")
    end = script.index("\nfunction ", start)
    body = script[start:end]

    assert "postJSON('/api/scene'" in body
    for forbidden in (
        "/api/plan/solve",
        "/api/session/start",
        "/api/build",
        "/api/config/set",
    ):
        assert forbidden not in body


def test_cylinder_renderer_uses_mount_centre_radius_and_full_height():
    script = (STATIC / "scene.js").read_text()

    assert "function addCylinder" in script
    assert "cylinder.center_mount_m" in script
    assert "cylinder.radius_m" in script
    assert "const halfHeight = cylinder.height_m / 2" in script
    assert "setObstacles" in script


def test_scene_has_no_cached_plan_overlay():
    script = (STATIC / "scene.js").read_text()
    panel = (STATIC / "panel.js").read_text()
    plan = (STATIC.parent / "plan.py").read_text()

    assert "paintDimension" not in script
    assert "setPlan" not in script
    assert "setProgress" not in script
    assert "refreshPlan" not in panel
    assert "planProgress" not in panel
    assert "row.traj_activated" not in panel
    assert "TRAJ_BEGIN" not in plan
    assert "_LAST_PLAN" not in plan


def test_scene_uses_runtime_tcp_markers_and_has_no_browser_fk():
    scene = (STATIC / "scene.js").read_text()
    panel = (STATIC / "panel.js").read_text()
    for forbidden in (
        "forwardKinematics",
        "jointTransform",
        "DH_ROOT_ROLL_RAD",
        "setDh",
        "mount_from_base",
        "meas_j1",
        "cmd_j1",
    ):
        assert forbidden not in scene
    assert "measured_tcp_x_mount_m" in scene
    assert "commanded_tcp_x_mount_m" in scene
    assert "measured_tcp_qx_mount" in scene
    assert "commanded_tcp_qx_mount" in scene
    assert "measured_chain_p" in scene and "x_mount_m" in scene
    assert "measuredArmChain" in scene
    assert "/api/dh" not in panel


def test_scene_exposes_fit_view_and_navigation_hint():
    html = (STATIC / "index.html").read_text()
    panel = (STATIC / "panel.js").read_text()

    assert 'id="scene-fit"' in html
    assert "drag to orbit" in html
    assert "fitView" in panel


def test_format_14_keeps_exchange_timestamps():
    assert runlog.has_exchange_timestamps(
        {"log_format": "14"}, {"t_send_s", "t_recv_s"})


def test_commanded_tcp_fk_is_after_every_stop_exit():
    runner = (STATIC.parents[1] / "runtime" / "Runner.cpp").read_text()
    tail = runner[runner.index("const ExecutionStopDecision stop_decision") :]
    query = tail.index("core.CommandedTcpMount()")
    stops = (
        "if (stop_decision.priority.reason != StopPriorityReason::kNone)",
        "if (stop_decision.nonfinite_stop)",
        "if (stop_decision.overrun_stop)",
    )
    for index, marker in enumerate(stops):
        start = tail.index(marker)
        end = tail.index(stops[index + 1]) if index + 1 < len(stops) else query
        assert start < query
        assert "break;" in tail[start:end]


def test_wrapper_exits_after_its_controller_ends():
    wrapper = (STATIC.parents[1] / "planning" / "scripts" / "run_session.sh").read_text()

    assert "read -r -t 1" in wrapper
    assert "kill -0 \"$CONTROLLER_PID\"" in wrapper


def test_running_session_accepts_goals_without_one_shot_startup_wait():
    panel = (STATIC / "panel.js").read_text()
    wrapper = (STATIC.parents[1] / "planning" / "scripts" / "run_session.sh").read_text()
    planner_runtime = (STATIC.parents[1] / "planning" / "src" /
                       "PlannerRuntime.cpp").read_text()

    assert "PLAN + EXECUTE" in panel
    assert "postJSON('/api/session/goal'" in panel
    assert "activate its first plan" not in wrapper
    assert 'cp "$GOAL_YAML"' not in wrapper
    assert '"--goal-file", config.goal_file' not in planner_runtime
