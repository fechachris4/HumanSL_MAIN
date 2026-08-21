from pathlib import Path


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
