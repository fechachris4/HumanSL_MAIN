import importlib
import json
from pathlib import Path

import pytest

from Christian_control.panel import build, paths, plan, server, session


PLANNER_TEXT = """# planner header survives
motion:
  nominal_speed_mps: 0.25
  min_duration_s: 1.0
  waypoints: 10
obstacles:
  epsilon_dist_m: 0.05  # unrelated inline comment survives
  collision_sigma: 0.0005
  # scene comment belongs to the replaced block
  scene: {}
smoothness:
  qc_scale: 1.0
goal:
  position_sigma_xyz: [0.001, 0.01, 0.001]
  rotation_sigma_rpy: [0.01, 0.01, 0.01]
solver:
  max_iterations: 1000
path_following:
  position_prior_sigma_m: 0.0012
  rotation_prior_sigma_rad: 0.01
  maximum_planning_error_m: 0.005
  maximum_orientation_error_rad: 0.1
  validation_dt_s: 0.002
  approach_velocity_fraction: 0.9
  approach_min_duration_s: 0.1
  approach_waypoints: 5
  max_chord_error_m: 0.001
seeding:
  ik_seed: 20260807
  randomised: false
"""


def scene_config_module():
    try:
        return importlib.import_module("Christian_control.panel.scene_config")
    except ModuleNotFoundError:
        pytest.fail("Christian_control.panel.scene_config is missing")


def write_fixture(tmp_path: Path, scene: str = "{}") -> Path:
    path = tmp_path / "planner.yaml"
    path.write_text(PLANNER_TEXT.replace("scene: {}", f"scene: {scene}"))
    return path


def valid_cylinder(**updates):
    value = {
        "enabled": True,
        "shape": "cylinder",
        "center_mount_m": [0.1, -0.2, 0.3],
        "radius_m": 0.22,
        "height_m": 0.6,
    }
    value.update(updates)
    return value


def valid_box(**updates):
    value = {
        "enabled": False,
        "shape": "box",
        "center_mount_m": [0.4, 0.0, -0.1],
        "half_extent_m": [0.3, 0.5, 0.1],
    }
    value.update(updates)
    return value


def test_read_scene_returns_disk_truth_and_cpp_fnv_token(tmp_path):
    scene_config = scene_config_module()
    path = write_fixture(tmp_path)

    result = scene_config.read_scene(path)

    expected = 1469598103934665603
    for byte in path.read_bytes():
        expected ^= byte
        expected = (expected * 1099511628211) & 0xFFFFFFFFFFFFFFFF
    assert result == {
        "path": str(path),
        "scene": {},
        "source_fnv1a64": f"{expected:016x}",
        "error": None,
    }


def test_source_token_survives_a_browser_json_round_trip_exactly(tmp_path):
    scene_config = scene_config_module()
    path = write_fixture(tmp_path)

    result = scene_config.read_scene(path)
    browser_value = json.loads(json.dumps(result))["source_fnv1a64"]

    assert int(browser_value, 16) > 2**53
    assert browser_value == result["source_fnv1a64"]
    assert len(browser_value) == 16
    assert browser_value == browser_value.lower()


def test_read_scene_reports_malformed_disk_scene_without_inventing_truth(tmp_path):
    scene_config = scene_config_module()
    path = write_fixture(
        tmp_path,
        "\n    torso:\n      enabled: true\n      shape: cylinder\n"
        "      center_mount_m: [0.0, 0.0, 0.0]\n      radius_m: 0.2",
    )

    result = scene_config.read_scene(path)

    assert result["scene"] == {}
    assert "torso" in result["error"]
    assert "height_m" in result["error"]
    assert result["source_fnv1a64"] is not None


def test_inline_empty_scene_rejects_indented_children(tmp_path):
    scene_config = scene_config_module()
    path = write_fixture(tmp_path)
    path.write_text(path.read_text().replace(
        "  scene: {}\n",
        "  scene: {}\n"
        "    hidden:\n"
        "      enabled: true\n"
        "      shape: cylinder\n"
        "      center_mount_m: [0.0, 0.0, 0.0]\n"
        "      radius_m: 0.2\n"
        "      height_m: 0.4\n",
    ))

    result = scene_config.read_scene(path)

    assert result["scene"] == {}
    assert "inline empty table" in result["error"]


def test_write_refuses_while_commanding(tmp_path):
    scene_config = scene_config_module()
    path = write_fixture(tmp_path)
    before = path.read_bytes()
    source = scene_config.read_scene(path)["source_fnv1a64"]

    ok, reason = scene_config.write_scene(
        {"torso": valid_cylinder()},
        source,
        path=path,
        commanding=lambda: True,
    )

    assert not ok
    assert "controller is commanding" in reason
    assert path.read_bytes() == before
    assert not paths.panel_backup(path).exists()


def test_write_refuses_a_stale_source_token_without_touching_bytes(tmp_path):
    scene_config = scene_config_module()
    path = write_fixture(tmp_path)
    before = path.read_bytes()
    source = scene_config.read_scene(path)["source_fnv1a64"]

    ok, reason = scene_config.write_scene(
        {"torso": valid_cylinder()},
        f"{(int(source, 16) ^ 1):016x}",
        path=path,
        commanding=lambda: False,
    )

    assert not ok
    assert "changed on disk" in reason
    assert path.read_bytes() == before
    assert not paths.panel_backup(path).exists()


@pytest.mark.parametrize("token", [42, "2a", "000000000000002A", None])
def test_write_rejects_noncanonical_source_tokens_without_touching_bytes(
    tmp_path, token
):
    scene_config = scene_config_module()
    path = write_fixture(tmp_path)
    before = path.read_bytes()

    ok, reason = scene_config.write_scene(
        {}, token, path=path, commanding=lambda: False
    )

    assert not ok
    assert "16-digit lowercase hexadecimal" in reason
    assert path.read_bytes() == before


@pytest.mark.parametrize(
    "obstacle, phrase",
    [
        (valid_cylinder(radius_m=0.0), "radius_m"),
        (valid_cylinder(height_m=-0.1), "height_m"),
        (valid_cylinder(radius_m=float("inf")), "finite"),
        (valid_box(half_extent_m=[0.3, 0.0, 0.1]), "half_extent_m"),
        (valid_box(center_mount_m=[0.0, float("nan"), 0.0]), "finite"),
        (valid_cylinder(enabled=1), "enabled"),
        (valid_cylinder(extra=1.0), "unknown"),
    ],
)
def test_write_rejects_invalid_schema_without_touching_bytes(
    tmp_path, obstacle, phrase
):
    scene_config = scene_config_module()
    path = write_fixture(tmp_path)
    before = path.read_bytes()
    source = scene_config.read_scene(path)["source_fnv1a64"]

    ok, reason = scene_config.write_scene(
        {"torso": obstacle}, source, path=path, commanding=lambda: False
    )

    assert not ok
    assert phrase in reason
    assert path.read_bytes() == before


def test_write_accepts_large_positive_dimensions_without_an_arbitrary_rail(tmp_path):
    scene_config = scene_config_module()
    path = write_fixture(tmp_path)
    source = scene_config.read_scene(path)["source_fnv1a64"]

    ok, result = scene_config.write_scene(
        {"large": valid_box(half_extent_m=[5.1, 6.2, 7.3])},
        source,
        path=path,
        commanding=lambda: False,
    )

    assert ok, result
    assert result["scene"]["large"]["half_extent_m"] == [5.1, 6.2, 7.3]


def test_write_refuses_malformed_disk_scene_without_touching_bytes(tmp_path):
    scene_config = scene_config_module()
    path = write_fixture(tmp_path, "[]")
    before = path.read_bytes()
    source = scene_config.read_scene(path)["source_fnv1a64"]

    ok, reason = scene_config.write_scene(
        {}, source, path=path, commanding=lambda: False
    )

    assert not ok
    assert "obstacles.scene" in reason
    assert path.read_bytes() == before


def test_whole_scene_write_adds_renames_deletes_and_changes_shape(tmp_path):
    scene_config = scene_config_module()
    path = write_fixture(tmp_path)
    source = scene_config.read_scene(path)["source_fnv1a64"]

    ok, first = scene_config.write_scene(
        {"torso": valid_cylinder(), "bench": valid_box()},
        source,
        path=path,
        commanding=lambda: False,
    )
    assert ok, first
    ok, second = scene_config.write_scene(
        {"body": valid_box(enabled=True)},
        first["source_fnv1a64"],
        path=path,
        commanding=lambda: False,
    )

    assert ok, second
    assert second["scene"] == {"body": valid_box(enabled=True)}
    text = path.read_text()
    assert "torso:" not in text
    assert "bench:" not in text
    assert "shape: box" in text
    assert "radius_m:" not in text


def test_write_is_deterministic_and_preserves_every_unrelated_byte(tmp_path):
    scene_config = scene_config_module()
    path = write_fixture(tmp_path)
    source = scene_config.read_scene(path)["source_fnv1a64"]

    ok, result = scene_config.write_scene(
        {"zeta": valid_cylinder(), "alpha": valid_box()},
        source,
        path=path,
        commanding=lambda: False,
    )

    assert ok, result
    changed = path.read_text()
    assert changed.index("    alpha:") < changed.index("    zeta:")
    prefix, suffix = PLANNER_TEXT.split("  # scene comment belongs to the replaced block\n  scene: {}")
    assert changed.startswith(prefix)
    assert changed.endswith(suffix)
    assert "epsilon_dist_m: 0.05  # unrelated inline comment survives" in changed


def test_first_successful_write_makes_one_backup_of_the_original(tmp_path):
    scene_config = scene_config_module()
    path = write_fixture(tmp_path)
    original = path.read_bytes()
    source = scene_config.read_scene(path)["source_fnv1a64"]

    ok, first = scene_config.write_scene(
        {"torso": valid_cylinder()},
        source,
        path=path,
        commanding=lambda: False,
    )
    assert ok, first
    backup = paths.panel_backup(path)
    assert backup.read_bytes() == original
    ok, second = scene_config.write_scene(
        {},
        first["source_fnv1a64"],
        path=path,
        commanding=lambda: False,
    )

    assert ok, second
    assert backup.read_bytes() == original


def test_write_refuses_when_an_existing_planner_knob_is_missing(tmp_path):
    scene_config = scene_config_module()
    path = write_fixture(tmp_path)
    path.write_text(path.read_text().replace("  qc_scale: 1.0\n", ""))
    before = path.read_bytes()
    source = scene_config.read_scene(path)["source_fnv1a64"]

    ok, reason = scene_config.write_scene(
        {"torso": valid_cylinder()},
        source,
        path=path,
        commanding=lambda: False,
    )

    assert not ok
    assert "smoothness.qc_scale" in reason
    assert path.read_bytes() == before


def test_write_rejects_non_string_obstacle_names_as_a_schema_error(tmp_path):
    scene_config = scene_config_module()
    path = write_fixture(tmp_path)
    before = path.read_bytes()
    source = scene_config.read_scene(path)["source_fnv1a64"]

    ok, reason = scene_config.write_scene(
        {"torso": valid_cylinder(), 1: valid_box()},
        source,
        path=path,
        commanding=lambda: False,
    )

    assert not ok
    assert "names must be non-empty strings" in reason
    assert path.read_bytes() == before


def handler_for(path: str, body=None):
    responses = []
    handler = object.__new__(server._Handler)
    handler.path = path
    handler.client_address = ("127.0.0.1", 12345)
    handler._body = lambda: body or {}
    handler._json = lambda value, code=200: responses.append((code, value))
    handler._fail = lambda exc: pytest.fail(f"handler failed: {exc}")
    return handler, responses


def test_scene_get_reports_the_commanding_save_gate(monkeypatch):
    fake = {
        "path": "/tmp/planner.yaml",
        "scene": {},
        "source_fnv1a64": "000000000000002a",
        "error": None,
    }
    monkeypatch.setattr(server.scene_config, "read_scene", lambda: dict(fake))
    monkeypatch.setattr(session, "status", lambda: {"commanding": True})
    handler, responses = handler_for("/api/scene")

    handler.do_GET()

    assert responses == [(200, {
        **fake,
        "save_allowed": False,
        "save_blocked_reason": "controller is commanding",
    })]


def test_scene_post_reaches_only_guarded_persistence(monkeypatch):
    called = []
    disk_truth = {
        "path": "/tmp/planner.yaml",
        "scene": {"torso": valid_cylinder()},
        "source_fnv1a64": "0000000000000054",
        "error": None,
    }

    def forbidden(*args, **kwargs):
        pytest.fail("scene POST crossed into solve, build, session start, or subprocess")

    def fake_write(submitted, source_fnv1a64, *, commanding):
        called.append((submitted, source_fnv1a64, commanding()))
        return True, disk_truth

    monkeypatch.setattr(plan, "solve", forbidden)
    monkeypatch.setattr(build, "start_build", forbidden)
    monkeypatch.setattr(session, "start", forbidden)
    monkeypatch.setattr(server.subprocess, "run", forbidden)
    monkeypatch.setattr(session, "status", lambda: {"commanding": False})
    monkeypatch.setattr(server.scene_config, "write_scene", fake_write)
    body = {
        "scene": {"torso": valid_cylinder()},
        "source_fnv1a64": "000000000000002a",
    }
    handler, responses = handler_for("/api/scene", body)

    handler.do_POST()

    assert called == [(body["scene"], "000000000000002a", False)]
    assert responses == [(200, disk_truth)]


def test_scene_post_returns_conflict_for_a_rejected_save(monkeypatch):
    monkeypatch.setattr(
        server.scene_config,
        "write_scene",
        lambda *args, **kwargs: (False, "controller is commanding"),
    )
    handler, responses = handler_for(
        "/api/scene", {"scene": {}, "source_fnv1a64": "000000000000002a"}
    )

    handler.do_POST()

    assert responses == [(409, {"error": "controller is commanding"})]
