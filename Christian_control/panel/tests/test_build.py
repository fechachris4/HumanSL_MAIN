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

from Christian_control.panel import build, paths

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

        christian_control = self.root / "Christian_control"
        control = christian_control / "control"
        runtime = christian_control / "runtime"
        planning = christian_control / "planning"

        self.config_h = touch(control / "Config.h", OLD)
        touch(runtime / "Runner.cpp", OLD)
        self.urdf = touch(christian_control / "model" / "GEN3_dual_mounted.urdf", OLD)
        self.controller_build = runtime / "build"
        self.controller_bin = touch(self.controller_build / "controller", NEW)
        self.controller_bin.chmod(0o755)

        self.bridge_source = touch(planning / "src" / "BridgeMain.cpp", OLD)
        self.trajectory_source = touch(
            planning / "optimisation" / "GenerateTrajectory.cpp",
            OLD,
        )
        self.bridge_build = planning / "build"
        self.bridge_bin = touch(self.bridge_build / "planner_bridge", NEW)
        self.bridge_bin.chmod(0o755)

        self.dh = {
            "right": touch(self.bridge_build / "config" / "dh_params_tool.yaml", NEW),
            "left": touch(self.bridge_build / "config" / "dh_params_flange.yaml", NEW),
        }

        redirected = {
            "REPO": self.root,
            "CONTROL": control,
            "RUNTIME": runtime,
            "PLANNING": planning,
            "CONTROLLER_BUILD": self.controller_build,
            "CONTROLLER_BIN": self.controller_bin,
            "BRIDGE_SRC": planning / "src",
            "BRIDGE_OPTIMISATION": planning / "optimisation",
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

    def test_a_build_tree_beside_the_sources_is_not_a_source(self) -> None:
        """control/ and runtime/ hold sources and build/ side by side. Output
        written into build/ by the build we just ran must never make the
        binary it produced look stale, or the panel refuses every start."""
        touch(self.root / "Christian_control" / "runtime" / "build"
              / "Testing" / "Temporary" / "CTestCostData.txt", NEW + 1000)
        report = build.freshness()
        self.assertFalse(report["controller"]["stale"], report["reasons"])

    def test_the_bridge_watches_optimisation_as_well_as_src(self) -> None:
        os.utime(self.trajectory_source, (NEW + 10, NEW + 10))
        report = build.freshness()
        self.assertTrue(report["controller"]["stale"])
        self.assertTrue(report["bridge"]["stale"])
        self.assertIn("GenerateTrajectory.cpp", " ".join(report["reasons"]))

    def test_touching_a_cpp_makes_the_controller_stale(self) -> None:
        """The .h case is above; a .cpp is the other half of the contract."""
        runner = self.root / "Christian_control" / "runtime" / "Runner.cpp"
        os.utime(runner, (NEW + 10, NEW + 10))
        report = build.freshness()
        self.assertTrue(report["controller"]["stale"])
        self.assertEqual(report["controller"]["newest_source"], str(runner))
        self.assertIn("Runner.cpp", report["reasons"][0])

    def test_a_rewritten_test_fixture_does_not_make_the_controller_stale(self) -> None:
        """The 2026-08-19 refusal. A fixture CSV is not compiled into the
        controller, so rewriting it cannot make the controller wrong, and the
        panel must not refuse a session over it while run_session.sh passes."""
        touch(
            self.root / "Christian_control" / "control" / "tests" / "fixtures"
            / "execution_preextract_v1.csv",
            NEW + 10,
        )
        report = build.freshness()
        self.assertFalse(report["controller"]["stale"], report["reasons"])
        self.assertFalse(report["stale"], report["reasons"])

    def test_non_source_files_among_the_sources_are_not_sources(self) -> None:
        """Same rule, everything else it covers: docs, data, editor droppings.
        No file needs its own exclusion — only .cpp and .h are consulted."""
        control = self.root / "Christian_control" / "control"
        for name in ("README.md", "notes.txt", "table.yaml",
                     ".Config.h.swp", "Config.h~", "Config.h.orig"):
            touch(control / name, NEW + 10)
        report = build.freshness()
        self.assertFalse(report["controller"]["stale"], report["reasons"])

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
        self.controller_build = self.root / "runtime" / "build"
        self.controller_build.mkdir(parents=True)
        self.bridge_build = self.root / "planning" / "build"
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
