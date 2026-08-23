import importlib
import json
import math
import os
import threading
import time
from pathlib import Path

import pytest

from Christian_control.panel import build, paths, plan, server, session


PLANNER_TEXT = """# planner header survives
motion:
  nominal_speed_mps: 0.25
  min_duration_s: 1.0
  waypoints: 10
obstacles:
  minimum_clearance_m: 0.05  # unrelated inline comment survives
  preferred_clearance_m: 0.10
  collision_sigma: 0.0005
  # scene comment belongs to the replaced block
  scene: {}
smoothness:
  qc_scale: 1.0
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
        "permitted_sphere_groups": [],
    }
    value.update(updates)
    return value


def valid_box(**updates):
    value = {
        "enabled": False,
        "shape": "box",
        "center_mount_m": [0.4, 0.0, -0.1],
        "half_extent_m": [0.3, 0.5, 0.1],
        "permitted_sphere_groups": [],
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
        "      center_mount_m: [0.0, 0.0, 0.0]\n"
        "      permitted_sphere_groups: []\n      radius_m: 0.2",
    )

    result = scene_config.read_scene(path)

    assert result["scene"] == {}
    assert "torso" in result["error"]
    assert "height_m" in result["error"]
    assert result["source_fnv1a64"] is not None


@pytest.mark.parametrize("quote", ["", "'", '"'])
def test_fixed_container_keys_accept_yaml_cpp_quote_spellings(tmp_path, quote):
    scene_config = scene_config_module()
    path = write_fixture(tmp_path)
    text = path.read_text()
    text = text.replace("obstacles:\n", f"{quote}obstacles{quote}:\n")
    text = text.replace("  scene: {}\n", f"  {quote}scene{quote}: {{}}\n")
    path.write_text(text)
    source = scene_config.read_scene(path)["source_fnv1a64"]

    ok, result = scene_config.write_scene(
        {"torso": valid_cylinder()},
        source,
        path=path,
        commanding=lambda: False,
    )

    assert ok, result
    assert result["scene"] == {"torso": valid_cylinder()}
    assert f"{quote}obstacles{quote}:" in path.read_text()


@pytest.mark.parametrize(
    "duplicate_text, phrase",
    [
        (
            """obstacles:
  minimum_clearance_m: 0.06
  preferred_clearance_m: 0.10
  collision_sigma: 0.0006
  scene: {}
""",
            "obstacles is duplicated",
        ),
        ("  'scene': {}\n", "obstacles.scene is duplicated"),
        ('  "scene": {}\n', "obstacles.scene is duplicated"),
    ],
)
def test_duplicate_fixed_scene_containers_are_rejected_without_writing(
    tmp_path, duplicate_text, phrase
):
    scene_config = scene_config_module()
    path = write_fixture(tmp_path)
    if duplicate_text.startswith("obstacles:"):
        path.write_text(path.read_text() + duplicate_text)
    else:
        path.write_text(path.read_text().replace(
            "  scene: {}\n", "  scene: {}\n" + duplicate_text
        ))
    before = path.read_bytes()
    disk = scene_config.read_scene(path)

    assert disk["scene"] == {}
    assert phrase in disk["error"]
    ok, reason = scene_config.write_scene(
        {}, disk["source_fnv1a64"], path=path, commanding=lambda: False
    )
    assert not ok
    assert phrase in reason
    assert path.read_bytes() == before
    assert not paths.panel_backup(path).exists()


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


def test_permitted_sphere_groups_round_trip_and_unknown_group_rejects(tmp_path):
    scene_config = scene_config_module()
    path = write_fixture(tmp_path)
    source = scene_config.read_scene(path)["source_fnv1a64"]
    value = valid_cylinder(permitted_sphere_groups=["mount_interface"])
    ok, result = scene_config.write_scene({"torso": value}, source,
                                          path=path, commanding=lambda: False)
    assert ok, result
    assert result["scene"]["torso"]["permitted_sphere_groups"] == ["mount_interface"]
    bad = valid_cylinder(permitted_sphere_groups=["unknown_group"])
    ok, reason = scene_config.write_scene({"torso": bad}, result["source_fnv1a64"],
                                          path=path, commanding=lambda: False)
    assert not ok
    assert "unknown" in reason
    missing = valid_cylinder()
    missing.pop("permitted_sphere_groups")
    ok, reason = scene_config.write_scene({"torso": missing}, result["source_fnv1a64"],
                                          path=path, commanding=lambda: False)
    assert not ok
    assert "permitted_sphere_groups" in reason


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
    assert "minimum_clearance_m: 0.05  # unrelated inline comment survives" in changed


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


def test_backup_copy_failure_publishes_nothing_and_leaves_source_unchanged(
    tmp_path, monkeypatch
):
    scene_config = scene_config_module()
    path = write_fixture(tmp_path)
    original = path.read_bytes()
    source = scene_config.read_scene(path)["source_fnv1a64"]

    def fail_after_partial_copy(source_file, destination_file, *args, **kwargs):
        destination_file.write(b"partial backup")
        raise OSError("injected backup copy failure")

    monkeypatch.setattr(
        scene_config.shutil, "copyfileobj", fail_after_partial_copy
    )
    ok, reason = scene_config.write_scene(
        {"torso": valid_cylinder()},
        source,
        path=path,
        commanding=lambda: False,
    )

    assert not ok
    assert "injected backup copy failure" in reason
    assert path.read_bytes() == original
    assert not paths.panel_backup(path).exists()
    assert list(tmp_path.glob("*.tmp")) == []


def test_backup_publication_returns_its_inode_ownership_token(tmp_path):
    scene_config = scene_config_module()
    path = write_fixture(tmp_path)

    backup, ownership = scene_config._publish_backup(path, path.read_bytes())

    published = backup.lstat()
    assert ownership is not None
    assert ownership.st_dev == published.st_dev
    assert ownership.st_ino == published.st_ino


def test_replace_failure_removes_backup_created_by_that_failed_save(
    tmp_path, monkeypatch
):
    scene_config = scene_config_module()
    path = write_fixture(tmp_path)
    original = path.read_bytes()
    source = scene_config.read_scene(path)["source_fnv1a64"]

    def fail_replace(source_path, destination_path):
        raise OSError("injected planner replace failure")

    monkeypatch.setattr(scene_config.os, "replace", fail_replace)
    ok, reason = scene_config.write_scene(
        {"torso": valid_cylinder()},
        source,
        path=path,
        commanding=lambda: False,
    )

    assert not ok
    assert "injected planner replace failure" in reason
    assert path.read_bytes() == original
    assert not paths.panel_backup(path).exists()
    assert list(tmp_path.glob("*.tmp")) == []


def test_post_link_temp_unlink_failure_keeps_ownership_for_replace_rollback(
    tmp_path, monkeypatch
):
    scene_config = scene_config_module()
    path = write_fixture(tmp_path)
    original = path.read_bytes()
    token = scene_config.read_scene(path)["source_fnv1a64"]
    backup = paths.panel_backup(path)
    real_unlink = Path.unlink

    def fail_backup_temp_unlink(unlink_path, *args, **kwargs):
        if unlink_path.name.startswith(f".{backup.name}."):
            raise OSError("injected redundant backup temp unlink failure")
        return real_unlink(unlink_path, *args, **kwargs)

    def fail_replace(source_path, destination_path):
        raise OSError("injected planner replace failure after backup link")

    monkeypatch.setattr(Path, "unlink", fail_backup_temp_unlink)
    monkeypatch.setattr(scene_config.os, "replace", fail_replace)
    ok, reason = scene_config.write_scene(
        {"torso": valid_cylinder()},
        token,
        path=path,
        commanding=lambda: False,
    )

    assert not ok
    assert "planner replace failure after backup link" in reason
    assert path.read_bytes() == original
    assert not backup.exists()
    # The only permitted residue is the redundant complete temp hard link.
    leftovers = list(tmp_path.glob(f".{backup.name}.*.tmp"))
    assert len(leftovers) == 1
    assert leftovers[0].read_bytes() == original
    real_unlink(leftovers[0])


def test_replace_failure_does_not_delete_foreign_backup_path_replacement(
    tmp_path, monkeypatch
):
    scene_config = scene_config_module()
    path = write_fixture(tmp_path)
    original = path.read_bytes()
    token = scene_config.read_scene(path)["source_fnv1a64"]
    backup = paths.panel_backup(path)
    foreign = b"backup replaced by an external writer"
    foreign_path = tmp_path / "foreign-backup"
    foreign_path.write_bytes(foreign)
    real_replace = os.replace

    def replace_backup_path_then_fail(source_path, destination_path):
        # Keep both inodes allocated before the rename so inode-number reuse
        # cannot make the foreign replacement look like this save's link.
        real_replace(foreign_path, backup)
        raise OSError("injected planner replace failure after foreign backup")

    monkeypatch.setattr(
        scene_config.os, "replace", replace_backup_path_then_fail
    )
    ok, reason = scene_config.write_scene(
        {"torso": valid_cylinder()},
        token,
        path=path,
        commanding=lambda: False,
    )

    assert not ok
    assert "planner replace failure after foreign backup" in reason
    assert path.read_bytes() == original
    assert backup.read_bytes() == foreign


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


@pytest.mark.parametrize("kind", ["malformed", "missing"])
def test_scene_get_blocks_save_when_disk_truth_is_unavailable(
    tmp_path, monkeypatch, kind
):
    path = write_fixture(tmp_path, "[]")
    if kind == "missing":
        path.unlink()
    monkeypatch.setattr(paths, "PLANNER_YAML", path)
    monkeypatch.setattr(session, "status", lambda: {"commanding": False})
    handler, responses = handler_for("/api/scene")

    handler.do_GET()

    code, result = responses[0]
    assert code == 200
    assert result["save_allowed"] is False
    if kind == "missing":
        assert result["source_fnv1a64"] is None
    else:
        assert result["source_fnv1a64"] is not None
    assert result["error"]
    assert result["save_blocked_reason"].startswith("planner scene is unavailable:")


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


@pytest.mark.parametrize("submitted", [[], False, "", None])
def test_scene_post_rejects_falsey_non_mappings_without_changing_disk(
    tmp_path, monkeypatch, submitted
):
    scene_config = scene_config_module()
    path = write_fixture(tmp_path)
    before = path.read_bytes()
    token = scene_config.read_scene(path)["source_fnv1a64"]
    monkeypatch.setattr(paths, "PLANNER_YAML", path)
    monkeypatch.setattr(session, "status", lambda: {"commanding": False})
    handler, responses = handler_for(
        "/api/scene", {"scene": submitted, "source_fnv1a64": token}
    )

    handler.do_POST()

    assert responses == [(409, {"error": "scene must be a mapping"})]
    assert path.read_bytes() == before
    assert not paths.panel_backup(path).exists()


def test_scene_post_rejects_enormous_json_number_without_500_or_disk_change(
    tmp_path, monkeypatch
):
    scene_config = scene_config_module()
    path = write_fixture(tmp_path)
    before = path.read_bytes()
    token = scene_config.read_scene(path)["source_fnv1a64"]
    monkeypatch.setattr(paths, "PLANNER_YAML", path)
    monkeypatch.setattr(session, "status", lambda: {"commanding": False})
    handler, responses = handler_for(
        "/api/scene",
        {
            "scene": {"torso": valid_cylinder(radius_m=10**400)},
            "source_fnv1a64": token,
        },
    )

    handler.do_POST()

    assert responses == [(409, {
        "error": "obstacles.scene.torso.radius_m must be a finite number"
    })]
    assert path.read_bytes() == before
    assert not paths.panel_backup(path).exists()


def test_two_concurrent_same_token_writes_have_one_winner_and_one_stale_rejection(
    tmp_path, monkeypatch
):
    scene_config = scene_config_module()
    path = write_fixture(tmp_path)
    original = path.read_bytes()
    token = scene_config.read_scene(path)["source_fnv1a64"]
    real_replace = os.replace
    first_replace_entered = threading.Event()
    second_thread_started = threading.Event()
    replacement_calls = []

    def delayed_first_replace(source, destination):
        if Path(destination) == path:
            replacement_calls.append(Path(source))
            if len(replacement_calls) == 1:
                first_replace_entered.set()
                assert second_thread_started.wait(timeout=1.0)
                time.sleep(0.1)
        return real_replace(source, destination)

    monkeypatch.setattr(scene_config.os, "replace", delayed_first_replace)
    results = []
    errors = []

    def save(scene):
        try:
            results.append(scene_config.write_scene(
                scene, token, path=path, commanding=lambda: False
            ))
        except BaseException as error:
            errors.append(error)

    first = threading.Thread(
        target=save, args=({"first": valid_cylinder()},)
    )
    first.start()
    assert first_replace_entered.wait(timeout=1.0)
    second = threading.Thread(
        target=lambda: (
            second_thread_started.set(),
            save({"second": valid_box()}),
        )
    )
    second.start()
    first.join(timeout=2.0)
    second.join(timeout=2.0)

    assert not first.is_alive() and not second.is_alive()
    assert errors == []
    assert [ok for ok, _ in results].count(True) == 1
    assert [ok for ok, _ in results].count(False) == 1
    rejection = next(value for ok, value in results if not ok)
    assert "changed on disk" in rejection
    assert len(replacement_calls) == 1
    assert paths.panel_backup(path).read_bytes() == original


def test_scene_save_and_session_start_handlers_do_not_overlap(monkeypatch):
    state_lock = threading.Lock()
    scene_entered = threading.Event()
    start_thread_started = threading.Event()
    active = 0
    overlapped = False

    def enter(name):
        nonlocal active, overlapped
        with state_lock:
            active += 1
            overlapped = overlapped or active > 1
        if name == "scene":
            scene_entered.set()
            assert start_thread_started.wait(timeout=1.0)
            time.sleep(0.1)
        with state_lock:
            active -= 1

    def fake_write(*args, **kwargs):
        enter("scene")
        return True, {
            "path": "/tmp/planner.yaml",
            "scene": {},
            "source_fnv1a64": "0000000000000054",
            "error": None,
        }

    def fake_start(**kwargs):
        enter("start")
        return {"ok": True}

    monkeypatch.setattr(server.scene_config, "write_scene", fake_write)
    monkeypatch.setattr(session, "start", fake_start)
    scene_handler, scene_responses = handler_for(
        "/api/scene", {"scene": {}, "source_fnv1a64": "000000000000002a"}
    )
    start_handler, start_responses = handler_for(
        "/api/session/start", {"arm": "right", "confirm": "GO"}
    )
    errors = []

    def run(handler):
        try:
            handler.do_POST()
        except BaseException as error:
            errors.append(error)

    scene_thread = threading.Thread(target=run, args=(scene_handler,))
    scene_thread.start()
    assert scene_entered.wait(timeout=1.0)
    start_thread = threading.Thread(target=lambda: (
        start_thread_started.set(), run(start_handler)
    ))
    start_thread.start()
    scene_thread.join(timeout=2.0)
    start_thread.join(timeout=2.0)

    assert not scene_thread.is_alive() and not start_thread.is_alive()
    assert errors == []
    assert overlapped is False
    assert scene_responses[0][0] == 200
    assert start_responses == [(200, {"ok": True})]


def test_live_goal_formats_mount_point_and_sends_to_running_arm(monkeypatch):
    monkeypatch.setattr(session, "status", lambda: {
        "commanding": True, "arm": "left", "mount": "fixed",
        "planning": True, "planner": "current",
    })
    sent = []
    monkeypatch.setattr(session, "send_goal_datagram",
                        lambda path, line: sent.append((path, line)))

    result = session.send_goal("left", {
        "mode": "point", "frame": "mount",
        "goal": ["0.4", "0.2", "0.3"],
        "orientation_rpy_deg": ["90", "0", "-90"],
    })

    assert result["ok"] is True
    assert sent[0][0] == paths.GOAL_SOCKETS["left"]
    tokens = sent[0][1].split()
    assert tokens[0] == "POINT" and tokens[4] == "FIXED"
    assert [float(v) for v in tokens[1:4]] == pytest.approx([0.4, 0.2, 0.3])
    assert [float(v) for v in tokens[5:]] == pytest.approx(
        [math.pi / 2, 0.0, -math.pi / 2])


def test_live_goal_endpoint_delegates_without_restarting_session(monkeypatch):
    monkeypatch.setattr(session, "send_goal",
                        lambda arm, fields: {"ok": True, "arm": arm})
    handler, responses = handler_for(
        "/api/session/goal", {"arm": "left", "fields": {"mode": "point"}}
    )

    handler.do_POST()

    assert responses == [(200, {"ok": True, "arm": "left"})]
