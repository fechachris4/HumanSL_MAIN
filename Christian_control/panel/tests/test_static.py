from pathlib import Path

from Christian_control.panel import dh, paths


STATIC = Path(__file__).resolve().parents[1] / "static"


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


def test_scene_uses_world_pd_directly_and_has_no_cached_plan_overlay():
    script = (STATIC / "scene.js").read_text()
    panel = (STATIC / "panel.js").read_text()
    plan = (STATIC.parent / "plan.py").read_text()

    assert "const desired = pointFromRow(row, 'pd_');" in script
    assert "transformPoint(baseMatrix(state.selected), desiredBase)" not in script
    assert "paintDimension" not in script
    assert "setPlan" not in script
    assert "setProgress" not in script
    assert "refreshPlan" not in panel
    assert "planProgress" not in panel
    assert "row.traj_activated" not in panel
    assert "TRAJ_BEGIN" not in plan
    assert "_LAST_PLAN" not in plan


def test_mount_geometry_is_read_from_canonical_yaml_and_supplied_to_scene():
    geometry = dh.parse_mounting_yaml(paths.MOUNTING_YAML.read_text())
    assert geometry == {
        "right": {"xyz": [0.0, -0.0375, 0.0], "rpy": [1.2085, 0.0, 0.0]},
        "left": {"xyz": [0.0, 0.0375, 0.0], "rpy": [-1.2085, 0.0, 0.0]},
    }

    scene = (STATIC / "scene.js").read_text()
    assert "DEFAULT_MOUNT_FROM_BASE" not in scene
    assert "0.0375" not in scene
    assert "1.2085" not in scene
    assert "mount_from_base" in scene
