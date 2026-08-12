"""Tests for panel/build.py.

Two halves, matching the module: the freshness comparison is tested against a
fabricated tree with hand-set modification times, and the build state machine
is tested with a fake command through the seam `build.build_command`. No
cmake runs here, and nothing touches the real repository — every path the
module reads is redirected at a temporary directory first.
"""

import os
import shutil
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from Christian_control.tools.panel import build, paths

# Fabricated timestamps, far enough apart that no filesystem's resolution can
# blur them. OLD is "when the sources were last edited", NEW is "when the
# binary was built".
OLD = 1_700_000_000.0
NEW = 1_700_001_000.0


def touch(path: Path, mtime: float, text: str = "x") -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    os.utime(path, (mtime, mtime))
    return path


class FreshnessTest(unittest.TestCase):
    def setUp(self) -> None:
        self.root = Path(tempfile.mkdtemp(prefix="panel_build_test_"))
        self.addCleanup(shutil.rmtree, self.root, ignore_errors=True)

        basic_control = self.root / "Christian_control" / "basic_control"
        planner_bridge = self.root / "Christian_control" / "planner_bridge"

        self.config_h = touch(basic_control / "src" / "Config.h", OLD)
        touch(basic_control / "src" / "Runner.cpp", OLD)
        self.urdf = touch(basic_control / "config" / "GEN3_dual_mounted.urdf", OLD)
        self.controller_build = basic_control / "build"
        self.controller_bin = touch(self.controller_build / "controller", NEW)
        self.controller_bin.chmod(0o755)

        self.bridge_source = touch(planner_bridge / "src" / "BridgeMain.cpp", OLD)
        self.trajectory_source = touch(
            planner_bridge / "trajectory_generation" / "src" / "GenerateTrajectory.cpp",
            OLD,
        )
        self.bridge_build = planner_bridge / "build"
        self.bridge_bin = touch(self.bridge_build / "planner_bridge", NEW)
        self.bridge_bin.chmod(0o755)

        self.dh = {
            "right": touch(self.bridge_build / "config" / "dh_params_tool.yaml", NEW),
            "left": touch(self.bridge_build / "config" / "dh_params_flange.yaml", NEW),
        }

        redirected = {
            "REPO": self.root,
            "BASIC_CONTROL": basic_control,
            "PLANNER_BRIDGE": planner_bridge,
            "CONTROLLER_SRC": basic_control / "src",
            "CONTROLLER_BUILD": self.controller_build,
            "CONTROLLER_BIN": self.controller_bin,
            "BRIDGE_SRC": planner_bridge / "src",
            "BRIDGE_TRAJECTORY_GENERATION": planner_bridge / "trajectory_generation",
            "BRIDGE_BUILD": self.bridge_build,
            "BRIDGE_BIN": self.bridge_bin,
            "URDF": self.urdf,
            "DH_YAML": dict(self.dh),
        }
        for name, value in redirected.items():
            patcher = mock.patch.object(paths, name, value)
            patcher.start()
            self.addCleanup(patcher.stop)

    def test_binaries_newer_than_their_sources_are_fresh(self) -> None:
        report = build.freshness()
        self.assertFalse(report["stale"])
        self.assertEqual(report["reasons"], [])
        self.assertTrue(report["controller"]["exists"])
        self.assertFalse(report["controller"]["stale"])
        self.assertFalse(report["bridge"]["stale"])
        self.assertFalse(report["dh"]["right"]["stale"])
        self.assertFalse(report["dh"]["left"]["stale"])

    def test_touching_a_source_makes_the_controller_stale(self) -> None:
        os.utime(self.config_h, (NEW + 10, NEW + 10))
        report = build.freshness()
        self.assertTrue(report["stale"])
        self.assertTrue(report["controller"]["stale"])
        self.assertFalse(report["bridge"]["stale"])
        self.assertEqual(report["controller"]["newest_source"], str(self.config_h))
        self.assertEqual(len(report["reasons"]), 1)
        self.assertIn("Config.h", report["reasons"][0])
        self.assertIn("controller", report["reasons"][0])

    def test_a_source_as_old_as_the_binary_is_not_stale(self) -> None:
        # bash's -nt is strictly newer, and fresh_or_die is what this mirrors.
        os.utime(self.config_h, (NEW, NEW))
        report = build.freshness()
        self.assertFalse(report["controller"]["stale"])

    def test_missing_binary_is_stale_and_says_so(self) -> None:
        self.controller_bin.unlink()
        report = build.freshness()
        self.assertTrue(report["stale"])
        self.assertFalse(report["controller"]["exists"])
        self.assertTrue(report["controller"]["stale"])
        self.assertEqual(report["controller"]["binary_mtime"], 0.0)
        self.assertIn("has not been built", report["reasons"][0])

    def test_a_binary_that_is_not_executable_is_stale(self) -> None:
        self.controller_bin.chmod(0o644)
        report = build.freshness()
        self.assertFalse(report["controller"]["exists"])
        self.assertTrue(report["controller"]["stale"])
        self.assertIn("not executable", report["reasons"][0])

    def test_the_bridge_watches_trajectory_generation_as_well_as_src(self) -> None:
        os.utime(self.trajectory_source, (NEW + 10, NEW + 10))
        report = build.freshness()
        self.assertTrue(report["bridge"]["stale"])
        self.assertEqual(
            report["bridge"]["newest_source"], str(self.trajectory_source)
        )
        self.assertIn("GenerateTrajectory.cpp", report["reasons"][0])

    def test_editor_droppings_are_not_sources(self) -> None:
        touch(self.config_h.with_name(".Config.h.swp"), NEW + 10)
        touch(self.config_h.with_name("Config.h~"), NEW + 10)
        touch(self.config_h.with_name("Config.h.orig"), NEW + 10)
        touch(
            self.config_h.parent / "__pycache__" / "generated.pyc",
            NEW + 10,
        )
        report = build.freshness()
        self.assertFalse(report["controller"]["stale"])

    def test_a_urdf_newer_than_the_generated_table_is_stale_per_arm(self) -> None:
        os.utime(self.dh["right"], (OLD - 100, OLD - 100))
        report = build.freshness()
        self.assertTrue(report["stale"])
        self.assertTrue(report["dh"]["right"]["stale"])
        self.assertFalse(report["dh"]["left"]["stale"])
        self.assertIn("right", report["reasons"][0])
        self.assertIn("dh_params_tool.yaml", report["reasons"][0])
        self.assertIn("URDF", report["reasons"][0])

    def test_a_missing_generated_table_is_stale(self) -> None:
        self.dh["left"].unlink()
        report = build.freshness()
        self.assertTrue(report["dh"]["left"]["stale"])
        self.assertFalse(report["dh"]["left"]["exists"])
        self.assertIn("has not been generated", report["reasons"][0])

    def test_an_unbuilt_checkout_reports_everything_rather_than_raising(self) -> None:
        self.controller_bin.unlink()
        self.bridge_bin.unlink()
        self.dh["right"].unlink()
        self.dh["left"].unlink()
        report = build.freshness()
        self.assertTrue(report["stale"])
        self.assertEqual(len(report["reasons"]), 4)


class BuildStateMachineTest(unittest.TestCase):
    """The state machine only. The command is fake; cmake never runs."""

    def setUp(self) -> None:
        self.root = Path(tempfile.mkdtemp(prefix="panel_build_test_"))
        self.addCleanup(shutil.rmtree, self.root, ignore_errors=True)
        self.controller_build = self.root / "basic_control" / "build"
        self.controller_build.mkdir(parents=True)
        self.bridge_build = self.root / "planner_bridge" / "build"
        self.bridge_build.mkdir(parents=True)

        for name, value in (
            ("CONTROLLER_BUILD", self.controller_build),
            ("BRIDGE_BUILD", self.bridge_build),
        ):
            patcher = mock.patch.object(paths, name, value)
            patcher.start()
            self.addCleanup(patcher.stop)

        # Leave no build running or listener attached for the next test.
        self.addCleanup(build.set_build_listener, None)
        self.addCleanup(build.wait_for_build, 30)

    def use_command(self, command: list[str]) -> None:
        patcher = mock.patch.object(build, "build_command", lambda target: command)
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_a_successful_build_reports_ok_and_keeps_its_output(self) -> None:
        self.use_command(["/bin/echo", "compiling nothing"])
        started, message = build.start_build("controller")
        self.assertTrue(started)
        self.assertIn("/bin/echo", message)
        self.assertTrue(build.wait_for_build(30))
        status = build.build_status()
        self.assertFalse(status["running"])
        self.assertTrue(status["ok"])
        self.assertEqual(status["target"], "controller")
        self.assertEqual(status["lines"], ["compiling nothing"])

    def test_a_failing_build_reports_not_ok(self) -> None:
        self.use_command(["/bin/sh", "-c", "echo error: no rule; exit 2"])
        self.assertTrue(build.start_build("bridge")[0])
        self.assertTrue(build.wait_for_build(30))
        status = build.build_status()
        self.assertFalse(status["ok"])
        self.assertEqual(status["target"], "bridge")
        self.assertIn("error: no rule", status["lines"][0])

    def test_a_command_that_cannot_start_is_a_failed_build_not_a_crash(self) -> None:
        self.use_command([str(self.root / "no_such_command")])
        self.assertTrue(build.start_build("controller")[0])
        self.assertTrue(build.wait_for_build(30))
        status = build.build_status()
        self.assertFalse(status["ok"])
        self.assertIn("could not start the build", status["lines"][0])

    def test_a_second_build_while_one_runs_is_refused_not_queued(self) -> None:
        self.use_command(["/bin/sh", "-c", "sleep 0.5; echo done"])
        self.assertTrue(build.start_build("controller")[0])
        running = build.build_status()
        self.assertTrue(running["running"])
        self.assertIsNone(running["ok"])

        started, message = build.start_build("bridge")
        self.assertFalse(started)
        self.assertIn("already running", message)

        self.assertTrue(build.wait_for_build(30))
        self.assertTrue(build.build_status()["ok"])
        self.assertEqual(build.build_status()["target"], "controller")

    def test_the_listener_gets_every_line_and_one_none_at_the_end(self) -> None:
        seen: list = []
        build.set_build_listener(seen.append)
        self.use_command(["/bin/sh", "-c", "echo one; echo two"])
        self.assertTrue(build.start_build("controller")[0])
        self.assertTrue(build.wait_for_build(30))
        self.assertEqual(seen, ["one", "two", None])

    def test_a_listener_that_raises_does_not_break_the_build(self) -> None:
        def angry(line):
            raise RuntimeError("the browser went away")

        build.set_build_listener(angry)
        self.use_command(["/bin/echo", "still fine"])
        self.assertTrue(build.start_build("controller")[0])
        self.assertTrue(build.wait_for_build(30))
        status = build.build_status()
        self.assertTrue(status["ok"])
        self.assertEqual(status["lines"], ["still fine"])

    def test_only_the_two_real_targets_are_accepted(self) -> None:
        started, message = build.start_build("everything")
        self.assertFalse(started)
        self.assertIn("unknown build target", message)
        self.assertFalse(build.build_status()["running"])

    def test_an_unconfigured_build_directory_is_refused(self) -> None:
        shutil.rmtree(self.controller_build)
        started, message = build.start_build("controller")
        self.assertFalse(started)
        self.assertIn("configure it with cmake first", message)

    def test_output_is_bounded_so_a_long_build_cannot_grow_without_limit(self) -> None:
        wanted = build._MAX_BUILD_LINES + 50
        self.use_command(["/bin/sh", "-c", f"seq 1 {wanted}"])
        self.assertTrue(build.start_build("controller")[0])
        self.assertTrue(build.wait_for_build(60))
        lines = build.build_status()["lines"]
        self.assertEqual(len(lines), build._MAX_BUILD_LINES)
        self.assertEqual(lines[-1], str(wanted))


if __name__ == "__main__":
    unittest.main()
