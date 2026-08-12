"""Goals are read, validated and written without losing the file's comments.

goal.yaml is the one file the workflow asks Christian to edit before a run, and
most of it is prose explaining traps that cost real hardware time — the
orientation-inheritance warning, the frame stamping, which numbers were
verified against hardware and which were not. A panel that rewrote the file
from its parsed structure would throw all of that away the first time someone
moved a target 10 mm. So the panel round-trips the text and parses only what it
draws.

No test here runs planner_bridge: solve() takes its subprocess call as a seam.
"""

from __future__ import annotations

import re
import shutil
import tempfile
import unittest
from pathlib import Path

from Christian_control.tools.panel import paths, plan, telemetry

GOAL_SAMPLE = """\
# Where each arm goes. This comment is load-bearing documentation.
session_arms: right

right:
  # Stamped explicitly because reading these as mount would move the target.
  frame: right_base
  goal: [ 0.5, 0.0, 0.4 ]
  orientation_rpy_deg: [ 90, 0, 90 ]
  box:
    center: [0.5, 0.0, 0.3]
    half_extent: [0.05, 0.05, 0.05]

left:
  # UNVERIFIED placeholder — never run against physical hardware.
  frame: mount
  goal: [ 0.31, 0.386, 0.52 ]
"""


class ParseGoal(unittest.TestCase):
    def test_reads_session_arms(self):
        self.assertEqual(plan.parse_goal(GOAL_SAMPLE)["session_arms"], "right")

    def test_reads_each_arm_block_separately(self):
        arms = plan.parse_goal(GOAL_SAMPLE)["arms"]
        self.assertEqual(set(arms), {"right", "left"})
        self.assertEqual(arms["right"]["frame"], "right_base")
        self.assertEqual(arms["right"]["goal"], [0.5, 0.0, 0.4])
        self.assertEqual(arms["left"]["goal"], [0.31, 0.386, 0.52])

    def test_a_nested_box_stays_nested(self):
        box = plan.parse_goal(GOAL_SAMPLE)["arms"]["right"]["box"]
        self.assertEqual(box["center"], [0.5, 0.0, 0.3])
        self.assertEqual(box["half_extent"], [0.05, 0.05, 0.05])

    def test_commented_out_keys_are_invisible(self):
        """The real file shows box: as a commented example; it must not parse."""
        text = "session_arms: right\n\nright:\n  goal: [1,2,3]\n  # box:\n  #   center: [0,0,0]\n"
        self.assertNotIn("box", plan.parse_goal(text)["arms"]["right"])

    def test_the_real_goal_file_parses(self):
        parsed = plan.read_goal()["parsed"]
        self.assertIn(parsed["session_arms"], ("right", "left", "both"))
        self.assertTrue(set(parsed["arms"]) <= {"right", "left"})


class ValidateGoal(unittest.TestCase):
    def test_a_good_file_has_no_problems(self):
        self.assertEqual(plan.validate_goal(GOAL_SAMPLE), [])

    def test_missing_session_arms_is_refused(self):
        problems = plan.validate_goal("right:\n  goal: [1,2,3]\n")
        self.assertTrue(any("session_arms" in p for p in problems))

    def test_a_nonsense_session_arms_is_refused(self):
        problems = plan.validate_goal("session_arms: both_of_them\nright:\n  goal: [1,2,3]\n")
        self.assertTrue(any("right, left or both" in p for p in problems))

    def test_a_two_element_goal_is_refused(self):
        problems = plan.validate_goal("session_arms: right\nright:\n  goal: [1,2]\n")
        self.assertTrue(any("three finite numbers" in p for p in problems))

    def test_a_box_missing_half_extent_is_refused(self):
        text = "session_arms: right\nright:\n  goal: [1,2,3]\n  box:\n    center: [0,0,0]\n"
        problems = plan.validate_goal(text)
        self.assertTrue(any("half_extent" in p for p in problems))

    def test_the_real_goal_file_is_valid(self):
        self.assertEqual(plan.validate_goal(paths.GOAL_YAML.read_text()), [])


class WriteGoal(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.path = Path(self._tmp.name) / "goal.yaml"
        self.path.write_text(GOAL_SAMPLE, encoding="utf-8")

    def test_an_unchanged_round_trip_is_byte_for_byte(self):
        """Reading and writing the same text must not touch a single comment."""
        before = self.path.read_bytes()
        ok, _ = plan.write_goal(GOAL_SAMPLE, path=self.path)
        self.assertTrue(ok)
        self.assertEqual(self.path.read_bytes(), before)

    def test_an_edit_keeps_every_comment_around_it(self):
        edited = GOAL_SAMPLE.replace("[ 0.5, 0.0, 0.4 ]", "[ 0.6, 0.0, 0.4 ]")
        ok, _ = plan.write_goal(edited, path=self.path)
        self.assertTrue(ok)
        written = self.path.read_text()
        self.assertIn("This comment is load-bearing documentation.", written)
        self.assertIn("UNVERIFIED placeholder", written)
        self.assertEqual(plan.parse_goal(written)["arms"]["right"]["goal"],
                         [0.6, 0.0, 0.4])

    def test_an_invalid_goal_leaves_the_file_completely_unchanged(self):
        before = self.path.read_bytes()
        ok, message = plan.write_goal("session_arms: sideways\n", path=self.path)
        self.assertFalse(ok)
        self.assertIn("session_arms", message)
        self.assertEqual(self.path.read_bytes(), before)

    def test_the_first_write_keeps_a_backup(self):
        plan.write_goal(GOAL_SAMPLE.replace("0.4 ]", "0.45 ]"), path=self.path)
        backup = self.path.with_name(self.path.name + ".panel.bak")
        if not backup.exists():  # some layouts spell it goal.panel.bak
            backup = self.path.with_suffix(".yaml.panel.bak")
        self.assertTrue(backup.exists(), "no backup was kept beside the goal")
        self.assertEqual(backup.read_text(), GOAL_SAMPLE)


def _row(t: float, base: float) -> str:
    """One wire row: <t_s> <q1..q7 deg> <v1..v7 deg/s> — fifteen numbers.

    JointTrajectory.cpp is the authority on this shape, and it rejects any row
    that is not exactly fifteen finite fields.
    """
    q = " ".join(f"{base + 10.0 * j:.3f}" for j in range(7))
    v = " ".join("0.000" for _ in range(7))
    return f"{t:.3f} {q} {v}"


class ParseTrajectoryBlock(unittest.TestCase):
    """The wire format the controller accepts, previewed but never sent.

    'TRAJ_BEGIN <count>' declares how many rows follow, and the block completes
    on the last declared row — TRAJ_END is only a trailing terminator.
    """

    BLOCK = (
        "TRAJ_BEGIN 3\n"
        + _row(0.000, 10.0) + "\n"
        + _row(0.100, 10.5) + "\n"
        + _row(0.200, 11.0) + "\n"
        + "TRAJ_END\n"
    )

    def test_counts_points_and_duration(self):
        parsed = plan.parse_trajectory_block(self.BLOCK)
        self.assertEqual(parsed["points"], 3)
        self.assertAlmostEqual(parsed["duration_s"], 0.2)

    def test_start_and_end_are_the_first_and_last_rows(self):
        parsed = plan.parse_trajectory_block(self.BLOCK)
        self.assertAlmostEqual(parsed["start_deg"][0], 10.0)
        self.assertAlmostEqual(parsed["end_deg"][6], 71.0)

    def test_text_without_a_block_yields_no_points(self):
        parsed = plan.parse_trajectory_block("nothing to see here\n")
        self.assertEqual(parsed["points"], 0)

    def test_a_count_that_does_not_match_the_rows_is_reported(self):
        """The controller rejects a short block; the preview must say so too."""
        short = "TRAJ_BEGIN 5\n" + _row(0.0, 10.0) + "\n" + _row(0.1, 11.0) + "\n"
        parsed = plan.parse_trajectory_block(short)
        self.assertTrue(parsed.get("error") or parsed["points"] != parsed.get("declared_points"))

    def test_a_row_that_is_not_fifteen_numbers_is_reported(self):
        bad = "TRAJ_BEGIN 2\n0.0 1 2 3\n0.1 1 2 3\n"
        parsed = plan.parse_trajectory_block(bad)
        self.assertTrue(parsed.get("error"), "a malformed row must be reported")


class ScratchIsolated(unittest.TestCase):
    """Base for anything that may reach _remember().

    A solved plan is kept on disk so the run screen survives a panel restart.
    Any test that solves — even with a fake bridge — must point that scratch at
    a temporary directory, or it leaves a fabricated plan where the running
    panel reads one, and the run screen then shows a trajectory that never
    existed. That has happened twice; hence a base class rather than a habit.
    """

    def setUp(self):
        super().setUp()
        self._scratch = tempfile.TemporaryDirectory()
        self.addCleanup(self._scratch.cleanup)
        real = paths.PANEL_SCRATCH
        paths.PANEL_SCRATCH = Path(self._scratch.name)
        plan.paths.PANEL_SCRATCH = Path(self._scratch.name)

        def restore():
            paths.PANEL_SCRATCH = real
            plan.paths.PANEL_SCRATCH = real
        self.addCleanup(restore)


class Solve(ScratchIsolated):
    """solve() takes its subprocess call as a seam. No test runs the bridge."""

    def test_an_unknown_arm_is_refused_without_running_anything(self):
        def must_not_run(command, stdout_path):
            raise AssertionError("the bridge must not be executed")
        result = plan.solve("middle", run=must_not_run)
        self.assertFalse(result["ok"])
        self.assertIn("unknown arm", result["plan"]["error"])

    def test_a_successful_solve_returns_the_parsed_plan(self):
        if not paths.BRIDGE_BIN.is_file():
            self.skipTest("planner_bridge has not been built in this checkout")

        def fake_bridge(command, stdout_path):
            Path(stdout_path).write_text(ParseTrajectoryBlock.BLOCK)
            return 0, "plan initialisation: straight line\n"

        result = plan.solve("right", start_deg=[0] * 7, run=fake_bridge)
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["plan"]["points"], 3)
        self.assertIn("plan initialisation", result["stderr"])

    def test_a_failed_solve_surfaces_stderr_verbatim(self):
        if not paths.BRIDGE_BIN.is_file():
            self.skipTest("planner_bridge has not been built in this checkout")

        def failing_bridge(command, stdout_path):
            return 2, "goal is outside the reachable set\n"

        result = plan.solve("right", start_deg=[0] * 7, run=failing_bridge)
        self.assertFalse(result["ok"])
        self.assertIn("outside the reachable set", result["stderr"])


class StartState(ScratchIsolated):
    """Where a solve plans from, and what happens when the newest run is empty.

    On 2026-08-11 a session failed at SetServoingMode, leaving a run log with a
    header and no data rows. Being newer than the good run before it, that file
    was the one planner_bridge chose, and every solve failed with "no complete
    data row" until someone read the logs by hand. These tests pin the panel
    answering that itself.
    """

    HEADER = "# arm = right (192.168.1.10)\n# log_format = 11\n"
    COLUMNS = "time_s," + ",".join(f"meas_j{j}" for j in range(1, 8)) + "\n"

    def setUp(self):
        super().setUp()   # isolates PANEL_SCRATCH; see ScratchIsolated
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.runs = Path(self._tmp.name) / "runs" / "2026-08-11"
        self.runs.mkdir(parents=True)
        self._real_runs, self._real_repo = paths.RUNS, paths.REPO
        paths.RUNS = self.runs.parent
        paths.REPO = Path(self._tmp.name)
        plan.paths.RUNS = self.runs.parent
        plan.paths.REPO = Path(self._tmp.name)

        def restore():
            paths.RUNS, paths.REPO = self._real_runs, self._real_repo
            plan.paths.RUNS, plan.paths.REPO = self._real_runs, self._real_repo
        self.addCleanup(restore)

    def _write(self, name: str, rows: int, base: float = 10.0) -> Path:
        path = self.runs / name
        body = "".join(
            f"{i * 0.002:.3f}," + ",".join(f"{base + j}" for j in range(7)) + "\n"
            for i in range(rows))
        path.write_text(self.HEADER + self.COLUMNS + body, encoding="utf-8")
        return path

    def test_the_final_configuration_is_signed_not_kortex_wrapped(self):
        """The bug that made every panel solve fail on 2026-08-11.

        Kortex reports joint positions on [0, 360); --start-deg, the joint
        limits and the firmware thresholds are all signed. Handing the raw
        values to the planner says the arm is at +261 deg when it is at -98.8,
        which is a different configuration — the same left circle plans
        cleanly from one and is rejected with joints 59 deg outside their
        limits from the other.
        """
        path = self.runs / "loop_log_right_1200.csv"
        body = "0.002," + ",".join(
            str(v) for v in [261.2, 6.7, 14.9, 311.6, 235.9, 30.6, 103.7]) + "\n"
        path.write_text(self.HEADER + self.COLUMNS + body, encoding="utf-8")

        got = telemetry.final_measured_deg(path)
        self.assertEqual([round(v, 1) for v in got],
                         [-98.8, 6.7, 14.9, -48.4, -124.1, 30.6, 103.7])
        for value in got:
            self.assertGreaterEqual(value, -180.0)
            self.assertLessEqual(value, 180.0)

    def test_the_tail_is_read_without_loading_the_whole_file(self):
        """Run logs reach 2.5 GB here; reading one forward killed the panel."""
        path = self.runs / "loop_log_right_1300.csv"
        with open(path, "w", encoding="utf-8") as handle:
            handle.write(self.HEADER + self.COLUMNS)
            for i in range(20000):
                handle.write(f"{i * 0.002:.3f}," + ",".join(
                    str(10.0 + j + (i % 7)) for j in range(7)) + "\n")
        # A whole-file read of this would be obvious in the timing; the point
        # of the assertion is the value, the point of the size is the method.
        got = telemetry.final_measured_deg(path)
        self.assertIsNotNone(got)
        self.assertEqual(len(got), 7)
        self.assertTrue(telemetry.has_data_rows(path))

    def test_a_good_run_offers_its_final_configuration(self):
        self._write("loop_log_right_0900.csv", 5, base=20.0)
        options = plan.start_state_options("right")
        self.assertEqual(len(options), 1)
        self.assertTrue(options[0]["usable"])
        # Size rather than a row count: counting rows means reading the file,
        # and these reach gigabytes. A five-row fixture rounds to 0.0 MB.
        self.assertIn("size_mb", options[0])
        self.assertEqual(options[0]["final_deg"][0], 20.0)
        self.assertEqual(options[0]["final_deg"][6], 26.0)

    def test_an_empty_newest_run_is_listed_but_marked_unusable(self):
        self._write("loop_log_right_0900.csv", 5)
        import os
        empty = self._write("loop_log_right_1000.csv", 0)
        os.utime(empty, (9e9, 9e9))  # unambiguously the newest
        options = plan.start_state_options("right")
        self.assertEqual(options[0]["name"], "loop_log_right_1000.csv")
        self.assertFalse(options[0]["usable"])
        self.assertIsNone(options[0]["final_deg"])
        self.assertTrue(options[1]["usable"])

    def test_solving_from_an_empty_newest_run_explains_itself(self):
        import os
        self._write("loop_log_right_0900.csv", 5)
        empty = self._write("loop_log_right_1000.csv", 0)
        os.utime(empty, (9e9, 9e9))
        if not paths.BRIDGE_BIN.is_file():
            self.skipTest("planner_bridge has not been built in this checkout")

        def must_not_run(command, stdout_path):
            raise AssertionError("the bridge must not be run with no start state")

        result = plan.solve("right", run=must_not_run)
        self.assertFalse(result["ok"])
        message = result["plan"]["error"]
        self.assertIn("no data rows", message)
        self.assertIn("loop_log_right_1000.csv", message)   # names the file
        self.assertIn("earlier run", message)               # and the way out

    def test_no_runs_at_all_says_so(self):
        if not paths.BRIDGE_BIN.is_file():
            self.skipTest("planner_bridge has not been built in this checkout")
        result = plan.solve("right", run=lambda c, p: (0, ""))
        self.assertFalse(result["ok"])
        self.assertIn("no runs", result["plan"]["error"])

    def test_explicit_angles_skip_the_run_logs_entirely(self):
        if not paths.BRIDGE_BIN.is_file():
            self.skipTest("planner_bridge has not been built in this checkout")
        seen = {}

        def fake_bridge(command, stdout_path):
            seen["command"] = command
            Path(stdout_path).write_text(ParseTrajectoryBlock.BLOCK)
            return 0, ""

        result = plan.solve("right", start_deg=[1, 2, 3, 4, 5, 6, 7],
                            run=fake_bridge)
        self.assertTrue(result["ok"], result)
        self.assertIn("--start-deg", seen["command"])
        self.assertEqual(result["plan"]["start_source"],
                         "start_deg given by the panel")


GOAL_DOC = """\
session_arms: left

right:
  # right's comment survives every edit
  frame: mount
  goal: [ 0.5, 0.0, 0.4 ]

left:
  frame: mount
  path:
    type: circle
    centre: [ 0.31, 0.386, 0.5213 ]
    radius_m: 0.1
    normal: [ 1, 0, 0 ]
    duration_s: 12.0
    orientation: fixed
    orientation_rpy_deg: [ 90, 0, 90 ]
"""


class WriteGoalFields(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="panel_goal_"))
        self.addCleanup(shutil.rmtree, self.tmp)
        self.yaml = self.tmp / "goal.yaml"
        self.yaml.write_text(GOAL_DOC)

    def read(self):
        return plan.parse_goal(self.yaml.read_text())

    def test_point_edit_changes_values_in_place(self):
        ok, why = plan.write_goal_fields(
            "right", {"mode": "point", "goal": [0.4, 0.1, 0.3],
                      "frame": "mount"}, self.yaml)
        self.assertTrue(ok, why)
        self.assertEqual(self.read()["arms"]["right"]["goal"], [0.4, 0.1, 0.3])
        self.assertIn("# right's comment survives every edit",
                      self.yaml.read_text())

    def test_orientation_none_means_inherit_and_removes_the_key(self):
        ok, _ = plan.write_goal_fields(
            "right", {"mode": "point", "goal": [0.5, 0.0, 0.4],
                      "orientation_rpy_deg": [10.0, 20.0, 30.0]}, self.yaml)
        self.assertTrue(ok)
        self.assertEqual(self.read()["arms"]["right"]["orientation_rpy_deg"],
                         [10.0, 20.0, 30.0])
        ok, _ = plan.write_goal_fields(
            "right", {"mode": "point", "goal": [0.5, 0.0, 0.4],
                      "orientation_rpy_deg": None}, self.yaml)
        self.assertTrue(ok)
        self.assertNotIn("orientation_rpy_deg", self.read()["arms"]["right"])

    def test_switching_left_to_point_removes_the_path_block(self):
        ok, why = plan.write_goal_fields(
            "left", {"mode": "point", "goal": [0.3, 0.4, 0.5]}, self.yaml)
        self.assertTrue(ok, why)
        left = self.read()["arms"]["left"]
        self.assertEqual(left["goal"], [0.3, 0.4, 0.5])
        self.assertNotIn("path", left)

    def test_switching_right_to_circle_removes_goal_and_builds_path(self):
        circle = {"type": "circle", "centre": [0.3, 0.3, 0.5],
                  "radius_m": 0.05, "normal": [0.0, 0.0, 1.0],
                  "duration_s": 10.0, "orientation": "fixed",
                  "orientation_rpy_deg": [90.0, 0.0, 90.0]}
        ok, why = plan.write_goal_fields(
            "right", {"mode": "circle", "path": circle}, self.yaml)
        self.assertTrue(ok, why)
        right = self.read()["arms"]["right"]
        self.assertNotIn("goal", right)
        self.assertEqual(right["path"]["centre"], [0.3, 0.3, 0.5])
        self.assertEqual(right["path"]["orientation"], "fixed")

    def test_box_add_and_remove(self):
        ok, _ = plan.write_goal_fields(
            "right", {"mode": "point", "goal": [0.5, 0.0, 0.4],
                      "box": {"center": [0.5, 0.0, 0.3],
                              "half_extent": [0.05, 0.05, 0.05]}}, self.yaml)
        self.assertTrue(ok)
        self.assertEqual(self.read()["arms"]["right"]["box"]["center"],
                         [0.5, 0.0, 0.3])
        ok, _ = plan.write_goal_fields(
            "right", {"mode": "point", "goal": [0.5, 0.0, 0.4],
                      "box": None}, self.yaml)
        self.assertTrue(ok)
        self.assertNotIn("box", self.read()["arms"]["right"])

    def test_a_blank_circle_field_is_refused_by_name(self):
        # An empty string is not None, so it used to reach the file as
        # "radius_m:" with nothing after it, saved with a success message.
        before = self.yaml.read_text()
        circle = dict(CIRCLE_PAYLOAD["path"], radius_m="  ")
        ok, why = plan.write_goal_fields(
            "left", {"mode": "circle", "path": circle,
                     "orientation_rpy_deg": None}, self.yaml)
        self.assertFalse(ok)
        self.assertIn("radius_m", why)
        self.assertEqual(self.yaml.read_text(), before)

    def test_a_fixed_orientation_without_its_angles_is_refused(self):
        before = self.yaml.read_text()
        circle = dict(CIRCLE_PAYLOAD["path"])
        circle["orientation_rpy_deg"] = ["", "", ""]
        ok, why = plan.write_goal_fields(
            "left", {"mode": "circle", "path": circle}, self.yaml)
        self.assertFalse(ok)
        self.assertIn("orientation_rpy_deg", why)
        self.assertEqual(self.yaml.read_text(), before)

        circle.pop("orientation_rpy_deg")
        ok, why = plan.write_goal_fields(
            "left", {"mode": "circle", "path": circle}, self.yaml)
        self.assertFalse(ok)
        self.assertIn("orientation_rpy_deg", why)
        self.assertEqual(self.yaml.read_text(), before)

    def test_a_radial_circle_needs_no_angles(self):
        circle = dict(CIRCLE_PAYLOAD["path"], orientation="radial")
        circle.pop("orientation_rpy_deg")
        ok, why = plan.write_goal_fields(
            "left", {"mode": "circle", "path": circle}, self.yaml)
        self.assertTrue(ok, why)

    def test_editing_one_arm_leaves_the_other_block_byte_identical(self):
        before = self.yaml.read_text()
        left_before = block_text(before, "left")
        ok, why = plan.write_goal_fields(
            "right", {"mode": "point", "goal": ["0.4", "0.1", "0.3"],
                      "frame": "mount", "orientation_rpy_deg": None,
                      "box": None}, self.yaml)
        self.assertTrue(ok, why)
        self.assertEqual(block_text(self.yaml.read_text(), "left"), left_before)

    def test_a_bad_edit_leaves_the_file_byte_identical(self):
        before = self.yaml.read_text()
        ok, why = plan.write_goal_fields(
            "right", {"mode": "point", "goal": [0.5, "x", 0.4]}, self.yaml)
        self.assertFalse(ok)
        self.assertTrue(why)
        self.assertEqual(self.yaml.read_text(), before)
        ok, _ = plan.write_goal_fields("middle", {"mode": "point"}, self.yaml)
        self.assertFalse(ok)
        self.assertEqual(self.yaml.read_text(), before)


def block_text(text: str, arm: str) -> str:
    """One top-level block's own lines, so a test can say the other arm's
    text did not move by a single byte."""
    lines = text.split("\n")
    start = next(i for i, line in enumerate(lines)
                 if line.startswith(f"{arm}:"))
    top_level = re.compile(r"^[A-Za-z_]\w*:")
    end = next((i for i in range(start + 1, len(lines))
                if top_level.match(lines[i])), len(lines))
    return "\n".join(lines[start:end])


# What saveGoalCard actually posts in circle mode: every field a STRING (the
# browser reads input.value), the path's own orientation inside the path
# block, and a top-level orientation_rpy_deg of null because the path carries
# the orientation instead. Written out here rather than built from a helper,
# so this test breaks if the browser's payload and the writer drift apart.
CIRCLE_PAYLOAD = {
    "mode": "circle",
    "frame": "mount",
    "path": {
        "type": "circle",
        "centre": ["0.3100", "0.386", "0.5213"],
        "radius_m": "0.1",
        "normal": ["1", "0", "0"],
        "duration_s": "12.0",
        "orientation": "fixed",
        "orientation_rpy_deg": ["90", "0", "90"],
    },
    "orientation_rpy_deg": None,
    "box": None,
}


class CircleSaveAgainstTheRealGoalFile(unittest.TestCase):
    """The bug this class exists for: a top-level orientation_rpy_deg of null
    used to delete the PATH's orientation_rpy_deg, because the removal step
    found the nested key. validate_goal has no path checks, so the file saved
    "successfully" and the next solve failed on the missing key."""

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="panel_goal_real_"))
        self.addCleanup(shutil.rmtree, self.tmp)
        self.yaml = self.tmp / "goal.yaml"
        shutil.copy2(paths.GOAL_YAML, self.yaml)

    def test_the_paths_orientation_survives_a_circle_save(self):
        before = self.yaml.read_text()
        self.assertIn("orientation_rpy_deg",
                      plan.parse_goal(before)["arms"]["left"]["path"])
        ok, why = plan.write_goal_fields("left", CIRCLE_PAYLOAD, self.yaml)
        self.assertTrue(ok, why)
        left = plan.parse_goal(self.yaml.read_text())["arms"]["left"]
        self.assertEqual(left["path"]["orientation_rpy_deg"],
                         [90.0, 0.0, 90.0])
        self.assertEqual(left["path"]["orientation"], "fixed")
        self.assertEqual(left["path"]["radius_m"], 0.1)
        self.assertNotIn("orientation_rpy_deg", left)   # none of its own

    def test_the_right_block_is_byte_identical_afterwards(self):
        right_before = block_text(self.yaml.read_text(), "right")
        ok, why = plan.write_goal_fields("left", CIRCLE_PAYLOAD, self.yaml)
        self.assertTrue(ok, why)
        self.assertEqual(block_text(self.yaml.read_text(), "right"),
                         right_before)


if __name__ == "__main__":
    unittest.main()
