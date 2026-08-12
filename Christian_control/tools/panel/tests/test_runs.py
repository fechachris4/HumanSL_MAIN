"""Run listing and summaries, against the real recorded runs.

The fixtures under runs/2026-08-11/ are read and never written. Two of them
are used deliberately: the short right-arm run of 2026-08-11 15:39, which
holds position and computes almost nothing (so most columns are 'nan' and
must come back absent, not zero), and its session directory, which holds the
controller's own account of why it stopped.
"""

import json
import tempfile
import unittest
from pathlib import Path

from Christian_control.tools.panel import paths, runs

DAY = paths.RUNS / "2026-08-11"
SHORT_RUN = "2026-08-11/loop_log_right_20260811_153920.csv"
SHORT_SESSION = DAY / "session_153919"

has_fixture = (paths.RUNS / SHORT_RUN).is_file()


@unittest.skipUnless(has_fixture, "the 2026-08-11 fixtures are not present")
class ListRuns(unittest.TestCase):
    def test_newest_first_with_the_fields_the_panel_shows(self):
        listed = runs.list_runs()
        self.assertTrue(listed)
        for entry in listed:
            self.assertIn(entry["arm"], ("right", "left", None))
            self.assertGreaterEqual(entry["mb"], 0.0)
        times = [entry["mtime"] for entry in listed]
        self.assertEqual(times, sorted(times, reverse=True))

    def test_the_session_copy_is_not_listed_twice(self):
        names = [entry["file"] for entry in runs.list_runs(limit=200)]
        self.assertEqual(len(names), len(set(names)))
        self.assertFalse([n for n in names if "session_" in n])

    def test_a_run_is_linked_to_its_session_directory(self):
        entry = next(e for e in runs.list_runs(limit=200)
                     if e["file"] == SHORT_RUN)
        self.assertEqual(entry["arm"], "right")
        self.assertEqual(entry["session"], "2026-08-11/session_153919")

    def test_a_run_without_a_session_says_so(self):
        entry = next(e for e in runs.list_runs(limit=200)
                     if e["file"].endswith("loop_log_right_20260811_154613.csv"))
        self.assertIsNone(entry["session"])

    def test_a_missing_runs_root_is_an_empty_list(self):
        with tempfile.TemporaryDirectory() as scratch:
            self.assertEqual(runs.list_runs(runs_root=Path(scratch) / "gone"), [])


@unittest.skipUnless(has_fixture, "the 2026-08-11 fixtures are not present")
class SummarizeRun(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.summary = runs.summarize_run(SHORT_RUN)

    def test_the_preamble_is_the_authority_on_the_format(self):
        self.assertEqual(self.summary["log_format"], "11")
        self.assertEqual(self.summary["reference_source"], "joint_trajectory")
        self.assertEqual(self.summary["preamble"]["kp"], "10")
        self.assertEqual(self.summary["preamble"]["following_error_limit_deg"], "3")

    def test_every_data_row_is_counted(self):
        self.assertEqual(self.summary["rows"], 633)
        self.assertEqual(self.summary["columns"], 146)
        self.assertAlmostEqual(self.summary["duration_s"], 1.2645, places=3)

    def test_a_column_the_run_never_computed_is_absent_not_zero(self):
        # This run held position under a joint reference, so there was no
        # Jacobian and no plan: sigma_min, the rotation error and the margins
        # are 'nan' in every row. Reporting 0.0 would read as perfect.
        worst = self.summary["worst"]
        for name in ("sigma_min", "rot_error_rad", "posture_error_deg",
                     "base_comp_m", "null_leak_mps"):
            self.assertIsNone(worst[name], name)
        # The two exceptions need only the measured angles and the joint
        # reference, so they are real from the first row. The margin's worst
        # case is the smallest value seen, not the largest.
        self.assertAlmostEqual(worst["joint_margin_deg"], 17.8973, places=3)
        self.assertLess(worst["joint_follow_error_deg"], 0.001)

    def test_the_following_error_is_the_guards_own_quantity(self):
        worst = self.summary["worst"]
        self.assertIsNotNone(worst["following_error_deg"])
        self.assertLess(worst["following_error_deg"], 3.0)
        self.assertIn(worst["following_error_joint"], range(1, 8))
        self.assertEqual(worst["position_error_m"], 0.0)

    def test_the_stop_reason_comes_from_both_places_that_say_it(self):
        stop = self.summary["stop"]
        self.assertEqual(stop["exit_reason"], "user_stop")
        self.assertEqual(stop["exit_cycle"], "608")
        self.assertEqual(stop["line"], "loop stopped by user (Ctrl+C)")
        self.assertTrue(stop["source"].endswith("session_153919/controller.log"))

    def test_the_session_record_is_attached(self):
        self.assertEqual(self.summary["session"], "2026-08-11/session_153919")
        self.assertEqual(self.summary["session_json"]["arm"], "right")
        self.assertFalse(self.summary["session_json"]["git_dirty"])

    def test_no_trajectory_was_ever_activated(self):
        self.assertEqual(self.summary["trajectory_edges"], [])
        self.assertFalse(self.summary["trajectory_edges_truncated"])
        self.assertEqual(self.summary["replan_advised_rows"], 0)

    def test_the_summary_is_json_serialisable(self):
        # The panel returns this straight down the wire, and a stray NaN
        # would produce output no browser will parse.
        text = json.dumps(self.summary)
        self.assertNotIn("NaN", text)


@unittest.skipUnless(has_fixture, "the 2026-08-11 fixtures are not present")
class RunsWithTrajectories(unittest.TestCase):
    def test_the_activation_edge_is_recorded_with_its_time(self):
        # The left arm of 15:46 accepted the traced circle 1.31 s in. The
        # edge is one cycle wide, so a summary that sampled rows instead of
        # reading them all could step straight over it.
        summary = runs.summarize_run(
            "2026-08-11/loop_log_left_20260811_154613.csv")
        self.assertEqual(summary["reference_source"], "joint_trajectory")
        self.assertEqual(summary["trajectory_edges"],
                         [{"event": "traj_activated", "time_s": 1.31,
                           "cycle": 630.0}])

    def test_a_run_that_never_got_a_plan_has_no_edges(self):
        summary = runs.summarize_run(
            "2026-08-11/loop_log_right_20260811_154613.csv")
        self.assertEqual(summary["rows"], 7406)
        self.assertEqual(summary["trajectory_edges"], [])


class PathGuard(unittest.TestCase):
    def test_a_path_outside_runs_is_refused(self):
        for attempt in ("../CLAUDE.md", "../../etc/passwd", "/etc/passwd",
                        "2026-08-11/../../Christian_control/tools/panel/runs.py"):
            with self.subTest(attempt=attempt):
                self.assertIn("error", runs.summarize_run(attempt))

    def test_a_missing_run_is_refused(self):
        self.assertIn("error", runs.summarize_run("2026-08-11/nothing.csv"))


class SessionArtifacts(unittest.TestCase):
    def test_a_directory_without_a_session_json_returns_none(self):
        with tempfile.TemporaryDirectory() as scratch:
            self.assertIsNone(runs.read_session_json(Path(scratch)))
            self.assertIsNone(runs.stop_reason(Path(scratch)))

    def test_a_malformed_session_json_is_not_an_exception(self):
        with tempfile.TemporaryDirectory() as scratch:
            (Path(scratch) / "session.json").write_text("{ not json")
            self.assertIsNone(runs.read_session_json(Path(scratch)))

    def test_the_last_stop_line_wins(self):
        with tempfile.TemporaryDirectory() as scratch:
            (Path(scratch) / "controller.log").write_text(
                "starting\n"
                "loop stopped by user (Ctrl+C)\n"
                "loop stopped: following error at t=2 s (cycle 9): joint 3 is "
                "4.1 deg from its command (limit 3)\n")
            reported = runs.stop_reason(Path(scratch))
            self.assertIn("following error", reported["line"])

    def test_a_run_inside_a_session_directory_finds_it(self):
        with tempfile.TemporaryDirectory() as scratch:
            session = Path(scratch) / "session_120000"
            session.mkdir()
            log = session / "loop_log_right_20260811_120001.csv"
            log.write_text("")
            self.assertEqual(runs.find_session_dir(log), session)


if __name__ == "__main__":
    unittest.main()
