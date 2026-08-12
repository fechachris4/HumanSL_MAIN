"""Tests for panel/telemetry.py.

The parser is tested against two real format-11 runs already in runs/, read
only and never written to, because a fixture invented here would only prove
that the parser agrees with itself. Everything else — partial lines, replay
pacing, staleness — is tested against small files this module writes into a
temporary directory. No robot, no network, no binary.
"""

import math
import shutil
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

from Christian_control.tools.panel import paths, telemetry

# Two recorded right-arm runs, log_format 11, 146 columns.
SMALL_RUN = paths.RUNS / "2026-08-11" / "loop_log_right_20260811_153920.csv"
BIG_RUN = paths.RUNS / "2026-08-11" / "loop_log_right_20260811_154613.csv"
CONTROLLER_LOG = paths.RUNS / "2026-08-11" / "session_153919" / "controller.log"

# Data rows, i.e. lines that are neither '#' nor the header. The controller's
# own stop report says "633 samples written" for the small run.
SMALL_RUN_ROWS = 633
BIG_RUN_ROWS = 7406


def read_lines(path: Path) -> list[str]:
    return path.read_text(encoding="utf-8").splitlines()


class PreambleTest(unittest.TestCase):
    """The '#' lines, which are what the binary actually compiled."""

    def setUp(self) -> None:
        self.preamble = telemetry.parse_preamble(read_lines(SMALL_RUN))

    def test_reads_the_compiled_reference_source(self) -> None:
        self.assertEqual(self.preamble["reference_source"], "joint_trajectory")

    def test_carries_the_thresholds_the_bars_are_drawn_from(self) -> None:
        self.assertEqual(self.preamble["following_error_limit_deg"], "3")
        self.assertEqual(self.preamble["traj_following_error_stop_deg"], "8")
        self.assertEqual(self.preamble["traj_start_tolerance_deg"], "2")
        self.assertEqual(self.preamble["arrival_tolerance_m"], "0.001")
        self.assertEqual(self.preamble["replan_joint_margin_deg"], "5")
        self.assertEqual(self.preamble["control_dt_s"], "0.002")

    def test_carries_the_guard_override_states(self) -> None:
        self.assertEqual(self.preamble["skip_startup_gates"], "false")
        self.assertEqual(self.preamble["disable_following_error_stop"], "false")
        self.assertEqual(self.preamble["allow_unverified_actuators"], "false")

    def test_reads_the_exit_trailer_written_after_the_data(self) -> None:
        self.assertEqual(self.preamble["exit_reason"], "user_stop")
        self.assertEqual(self.preamble["exit_cycle"], "608")

    def test_strips_the_provenance_annotation_only(self) -> None:
        self.assertEqual(self.preamble["log_format"], "11")
        self.assertEqual(
            self.preamble["startup_position_m"],
            "-0.172025 -0.980392 0.164513",
        )

    def test_skips_lines_that_are_not_key_value(self) -> None:
        self.assertNotIn("controller run config", " ".join(self.preamble))
        parsed = telemetry.parse_preamble([
            "# controller run config — parsers skip '#' lines",
            "not a comment at all",
            "# kp = 10 (Config.h)",
        ])
        self.assertEqual(parsed, {"kp": "10"})

    def test_read_preamble_streams_the_same_answer_from_the_file(self) -> None:
        self.assertEqual(telemetry.read_preamble(SMALL_RUN), self.preamble)

    def test_missing_file_is_an_empty_preamble_not_a_crash(self) -> None:
        self.assertEqual(telemetry.read_preamble(Path("/nonexistent/run.csv")), {})


class CsvTailRealRunTest(unittest.TestCase):
    """Parsing the recorded runs by column name."""

    def test_reads_every_data_row_of_both_runs(self) -> None:
        for path, expected in ((SMALL_RUN, SMALL_RUN_ROWS), (BIG_RUN, BIG_RUN_ROWS)):
            with self.subTest(path=path.name):
                tail = telemetry.CsvTail(path)
                rows = tail.poll()
                self.assertEqual(len(rows), expected)
                self.assertEqual(tail.rows_seen, expected)
                self.assertEqual(len(tail.header or []), 146)

    def test_values_are_addressed_by_name(self) -> None:
        rows = telemetry.CsvTail(SMALL_RUN).poll()
        first = rows[0]
        self.assertAlmostEqual(first["cmd_j1"], 284.951)
        self.assertAlmostEqual(first["meas_j4"], 18.731)
        self.assertAlmostEqual(first["dt_s"], 0.002)
        # cycle counts control cycles, so the takeover-hold rows at the start
        # are all cycle 0 and the last row matches the preamble's exit_cycle.
        self.assertEqual(first["cycle"], 0.0)
        self.assertEqual(rows[-1]["cycle"], 608.0)

    def test_nan_becomes_none_and_never_zero(self) -> None:
        rows = telemetry.CsvTail(SMALL_RUN).poll()
        first = rows[0]
        # sigma_min, rot_error_rad and the quaternion are nan on a run with
        # no pose law; read as 0.0 they would mean perfect tracking.
        for column in ("sigma_min", "rot_error_rad", "quat_w", "joint_margin_deg"):
            with self.subTest(column=column):
                self.assertIsNone(first[column])
                self.assertNotEqual(first[column], 0.0)
        # And a column that really is zero still reads as zero.
        self.assertEqual(first["fault_j1"], 0.0)

    def test_preamble_after_the_data_is_still_collected(self) -> None:
        tail = telemetry.CsvTail(SMALL_RUN)
        tail.poll()
        self.assertEqual(tail.preamble["exit_reason"], "user_stop")

    def test_the_recorded_run_ends_where_the_stop_report_says(self) -> None:
        rows = telemetry.CsvTail(SMALL_RUN).poll()
        self.assertAlmostEqual(rows[-1]["time_s"], 1.26456, places=5)
        self.assertAlmostEqual(rows[-1]["joint_margin_deg"], 17.8973, places=4)


class CsvTailGrowingFileTest(unittest.TestCase):
    """The file is being written while it is being read."""

    def setUp(self) -> None:
        self.root = Path(tempfile.mkdtemp(prefix="panel_telemetry_test_"))
        self.addCleanup(shutil.rmtree, self.root, ignore_errors=True)
        self.path = self.root / "loop_log_right_test.csv"

    def append(self, text: str) -> None:
        with open(self.path, "a", encoding="utf-8") as handle:
            handle.write(text)

    def test_a_missing_file_is_not_an_error(self) -> None:
        tail = telemetry.CsvTail(self.path)
        self.assertEqual(tail.poll(), [])
        self.append("# kp = 10 (Config.h)\ntime_s,dt_s\n0.1,0.002\n")
        self.assertEqual(len(tail.poll()), 1)

    def test_a_partial_line_is_not_parsed_until_its_newline_arrives(self) -> None:
        self.append("time_s,dt_s,cmd_j1\n0.1,0.002,10\n")
        tail = telemetry.CsvTail(self.path)
        self.assertEqual(len(tail.poll()), 1)

        # Half a row, as a mid-drain read would see it.
        self.append("0.2,0.002")
        self.assertEqual(tail.poll(), [])

        self.append(",20\n")
        rows = tail.poll()
        self.assertEqual(len(rows), 1)
        self.assertAlmostEqual(rows[0]["cmd_j1"], 20.0)
        self.assertAlmostEqual(rows[0]["time_s"], 0.2)

    def test_a_partial_header_is_not_used_as_a_header(self) -> None:
        self.append("time_s,dt_s,cmd")
        tail = telemetry.CsvTail(self.path)
        self.assertEqual(tail.poll(), [])
        self.assertIsNone(tail.header)
        self.append("_j1\n0.1,0.002,10\n")
        rows = tail.poll()
        self.assertEqual(tail.header, ["time_s", "dt_s", "cmd_j1"])
        self.assertAlmostEqual(rows[0]["cmd_j1"], 10.0)

    def test_a_short_or_renamed_column_does_not_crash(self) -> None:
        self.append("time_s,dt_s,cmd_j1,mystery\n")
        tail = telemetry.CsvTail(self.path)
        tail.poll()
        self.append("0.1,0.002\n")           # short row
        self.append("0.2,0.002,5,text,9\n")  # long row, one non-numeric field
        rows = tail.poll()
        self.assertEqual(len(rows), 2)
        self.assertNotIn("cmd_j1", rows[0])
        self.assertEqual(rows[1]["mystery"], "text")
        # And a row missing the columns derive() wants yields None, not zero.
        self.assertIsNone(telemetry.derive(rows[0])["pos_err_m"])
        self.assertIsNone(telemetry.derive(rows[0])["follow_err_deg"])

    def test_a_replaced_shorter_file_is_reread_from_the_top(self) -> None:
        self.append("time_s,cmd_j1\n0.1,10\n0.2,11\n")
        tail = telemetry.CsvTail(self.path)
        self.assertEqual(len(tail.poll()), 2)
        # A shorter file is a different file: its own header applies, and the
        # first line must not be parsed as a data row under the old one.
        self.path.write_text("time_s,meas_j1\n0.5,3\n", encoding="utf-8")
        rows = tail.poll()
        self.assertEqual(len(rows), 1)
        self.assertEqual(tail.header, ["time_s", "meas_j1"])
        self.assertAlmostEqual(rows[0]["time_s"], 0.5)
        self.assertEqual(tail.rows_seen, 1)


class WrapTest(unittest.TestCase):
    """The bug that made a joint look a full turn away from its command."""

    def test_wrap180_matches_state_h(self) -> None:
        self.assertAlmostEqual(telemetry.wrap180(1.0 - 359.0), 2.0)
        self.assertAlmostEqual(telemetry.wrap180(359.0 - 1.0), -2.0)
        self.assertAlmostEqual(telemetry.wrap180(0.0), 0.0)
        self.assertAlmostEqual(telemetry.wrap180(370.0), 10.0)
        self.assertAlmostEqual(abs(telemetry.wrap180(180.0)), 180.0)

    def test_following_error_of_a_wrapped_joint_is_two_degrees(self) -> None:
        row = {"cmd_j3": 1.0, "meas_j3": 359.0}
        derived = telemetry.derive(row)
        self.assertAlmostEqual(derived["follow_err_deg"], 2.0)
        self.assertEqual(derived["follow_err_joint"], 3)


class DeriveTest(unittest.TestCase):
    def test_position_error_is_the_distance_the_csv_does_not_store(self) -> None:
        row = {
            "pd_x": 1.0, "pd_y": 2.0, "pd_z": 3.0,
            "p_x": 1.3, "p_y": 2.0, "p_z": 3.4,
        }
        self.assertAlmostEqual(telemetry.derive(row)["pos_err_m"], 0.5)

    def test_a_missing_component_makes_the_distance_unknown(self) -> None:
        row = {"pd_x": 1.0, "pd_y": None, "pd_z": 3.0, "p_x": 0.0, "p_y": 0.0, "p_z": 0.0}
        self.assertIsNone(telemetry.derive(row)["pos_err_m"])

    def test_the_worst_joint_is_reported_not_the_first(self) -> None:
        row = {
            "cmd_j1": 10.0, "meas_j1": 10.5,
            "cmd_j5": 20.0, "meas_j5": 17.0,
            "cmd_j7": 30.0, "meas_j7": 30.0,
        }
        derived = telemetry.derive(row)
        self.assertAlmostEqual(derived["follow_err_deg"], 3.0)
        self.assertEqual(derived["follow_err_joint"], 5)

    def test_gap_is_between_the_rows_the_panel_saw(self) -> None:
        prev = {"time_s": 1.0}
        row = {"time_s": 1.05, "dt_s": 0.002}
        self.assertAlmostEqual(telemetry.derive(row, prev)["gap_s"], 0.05)
        self.assertIsNone(telemetry.derive(row)["gap_s"])

    def test_a_real_row_derives_without_inventing_values(self) -> None:
        # The recorded run has no pose reference, so pd_* is nan by the end
        # and the position error is genuinely unknown rather than zero.
        rows = telemetry.CsvTail(SMALL_RUN).poll()
        self.assertIsNone(telemetry.derive(rows[-1])["pos_err_m"])
        self.assertIsNotNone(telemetry.derive(rows[-1])["follow_err_deg"])


class AggregatorTest(unittest.TestCase):
    """The 500 Hz spike the 20 Hz frame would otherwise step over."""

    def rows(self) -> list[dict]:
        return [
            {"dt_s": 0.002, "rot_error_rad": 0.01, "joint_margin_deg": 12.0,
             "posture_error_deg": 1.0, "cmd_j1": 0.0, "meas_j1": 0.5,
             "pd_x": 0.0, "pd_y": 0.0, "pd_z": 0.0,
             "p_x": 0.01, "p_y": 0.0, "p_z": 0.0},
            # The spike: worst error and worst (smallest) margin, one row wide.
            {"dt_s": 0.005, "rot_error_rad": 0.09, "joint_margin_deg": 3.0,
             "posture_error_deg": 4.0, "cmd_j4": 0.0, "meas_j4": 2.5,
             "pd_x": 0.0, "pd_y": 0.0, "pd_z": 0.0,
             "p_x": 0.04, "p_y": 0.0, "p_z": 0.0,
             "traj_rejected": 1.0, "traj_start_error_deg": 4.1,
             "replan_advised": 2.0},
            {"dt_s": 0.002, "rot_error_rad": 0.02, "joint_margin_deg": 11.0,
             "posture_error_deg": 1.5, "cmd_j1": 0.0, "meas_j1": 0.4,
             "pd_x": 0.0, "pd_y": 0.0, "pd_z": 0.0,
             "p_x": 0.02, "p_y": 0.0, "p_z": 0.0},
        ]

    def worst_of(self, rows: list[dict]) -> dict:
        aggregator = telemetry.Aggregator()
        for row in rows:
            aggregator.add(row)
        return aggregator.take()

    def test_keeps_the_maximum_error_and_the_minimum_margin(self) -> None:
        worst = self.worst_of(self.rows())
        self.assertAlmostEqual(worst["pos_err_m"], 0.04)
        self.assertAlmostEqual(worst["rot_error_rad"], 0.09)
        self.assertAlmostEqual(worst["posture_error_deg"], 4.0)
        self.assertAlmostEqual(worst["follow_err_deg"], 2.5)
        self.assertEqual(worst["follow_err_joint"], 4)
        self.assertAlmostEqual(worst["joint_margin_deg"], 3.0)
        self.assertEqual(worst["rows"], 3)

    def test_a_single_cycle_edge_flag_survives_the_window(self) -> None:
        worst = self.worst_of(self.rows())
        self.assertTrue(worst["traj_rejected"])
        self.assertFalse(worst["traj_activated"])
        self.assertFalse(worst["joint_follow_stop"])
        self.assertAlmostEqual(worst["traj_start_error_deg"], 4.1)
        self.assertTrue(worst["replan_advised"])
        self.assertAlmostEqual(worst["replan_advised_max"], 2.0)

    def test_late_cycles_are_counted_against_the_nominal_period(self) -> None:
        self.assertEqual(self.worst_of(self.rows())["overruns"], 1)
        aggregator = telemetry.Aggregator(nominal_dt_s=0.004)
        for row in self.rows():
            aggregator.add(row)
        self.assertEqual(aggregator.take()["overruns"], 0)

    def test_take_starts_a_new_window(self) -> None:
        aggregator = telemetry.Aggregator()
        for row in self.rows():
            aggregator.add(row)
        aggregator.take()
        empty = aggregator.take()
        self.assertIsNone(empty["pos_err_m"])
        self.assertIsNone(empty["joint_margin_deg"])
        self.assertFalse(empty["traj_rejected"])
        self.assertEqual(empty["rows"], 0)

    def test_missing_values_never_pull_the_worst_towards_zero(self) -> None:
        aggregator = telemetry.Aggregator()
        aggregator.add({"joint_margin_deg": 6.0, "rot_error_rad": 0.5})
        aggregator.add({"joint_margin_deg": None, "rot_error_rad": None})
        worst = aggregator.take()
        self.assertAlmostEqual(worst["joint_margin_deg"], 6.0)
        self.assertAlmostEqual(worst["rot_error_rad"], 0.5)

    def test_a_real_run_aggregates_to_a_finite_worst_case(self) -> None:
        aggregator = telemetry.Aggregator()
        rows = telemetry.CsvTail(SMALL_RUN).poll()
        previous = None
        for row in rows:
            aggregator.add(row, telemetry.derive(row, previous))
            previous = row
        worst = aggregator.take()
        self.assertEqual(worst["rows"], SMALL_RUN_ROWS)
        self.assertAlmostEqual(worst["joint_margin_deg"], 17.8973, places=4)
        self.assertTrue(math.isfinite(worst["follow_err_deg"]))
        self.assertLess(worst["follow_err_deg"], 3.0)  # under the stop limit


class FindLatestCsvTest(unittest.TestCase):
    def setUp(self) -> None:
        self.root = Path(tempfile.mkdtemp(prefix="panel_runs_test_"))
        self.addCleanup(shutil.rmtree, self.root, ignore_errors=True)
        patch = mock.patch.object(telemetry.paths, "RUNS", self.root)
        patch.start()
        self.addCleanup(patch.stop)

    def write(self, relative: str, mtime: float) -> Path:
        path = self.root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("time_s\n0\n", encoding="utf-8")
        import os
        os.utime(path, (mtime, mtime))
        return path

    def test_picks_the_newest_run_for_that_arm(self) -> None:
        self.write("2026-08-10/loop_log_right_20260810_120000.csv", 1_700_000_000.0)
        newest = self.write(
            "2026-08-11/loop_log_right_20260811_153920.csv", 1_700_001_000.0)
        self.write("2026-08-11/loop_log_left_20260811_154040.csv", 1_700_002_000.0)
        self.assertEqual(telemetry.find_latest_csv("right"), newest)

    def test_ignores_the_copy_inside_a_session_directory(self) -> None:
        top = self.write(
            "2026-08-11/loop_log_right_20260811_153920.csv", 1_700_000_000.0)
        self.write(
            "2026-08-11/session_153919/loop_log_right_20260811_153920.csv",
            1_700_009_000.0)
        self.assertEqual(telemetry.find_latest_csv("right"), top)

    def test_no_runs_is_none_not_an_error(self) -> None:
        self.assertIsNone(telemetry.find_latest_csv("right"))
        self.assertIsNone(telemetry.find_latest_csv("left"))


class TailLogLinesTest(unittest.TestCase):
    def test_reads_the_stop_report_off_the_end_of_a_real_log(self) -> None:
        lines = telemetry.tail_log_lines(CONTROLLER_LOG, 6)
        self.assertEqual(len(lines), 6)
        self.assertTrue(any("loop stopped by user" in line for line in lines)
                        or any("samples written" in line for line in lines))

    def test_asking_for_more_lines_than_exist_returns_the_file(self) -> None:
        lines = telemetry.tail_log_lines(CONTROLLER_LOG, 10_000)
        self.assertEqual(len(lines), 44)

    def test_a_missing_log_is_empty_not_an_error(self) -> None:
        self.assertEqual(telemetry.tail_log_lines(Path("/nonexistent/x.log")), [])


class ReplaySourceTest(unittest.TestCase):
    """The harness the whole UI is built against, with no robot."""

    def test_a_fast_replay_finishes_quickly_and_stays_in_time_order(self) -> None:
        source = telemetry.Source("right", path=SMALL_RUN, mode="replay", speed=1000.0)
        started = time.monotonic()
        times: list[float] = []
        frames = 0
        ended = False
        for event, payload in source.frames():
            frames += 1
            if event == "telemetry":
                times.append(payload["row"]["time_s"])
                self.assertTrue(payload["growing"])
                self.assertEqual(payload["arm"], "right")
                self.assertIsInstance(payload["wall_ms"], int)
            else:
                self.assertEqual(payload["reason"], "replay ended")
                ended = True
                source.stop()
                break
            self.assertLess(time.monotonic() - started, 20.0)
        elapsed = time.monotonic() - started

        self.assertTrue(ended)
        self.assertLess(elapsed, 10.0)
        self.assertEqual(times, sorted(times))
        self.assertGreater(len(times), 5)
        self.assertAlmostEqual(times[-1], 1.26456, places=5)
        self.assertEqual(source.rows_seen, SMALL_RUN_ROWS)
        self.assertEqual(source.preamble["reference_source"], "joint_trajectory")

    def test_every_row_is_aggregated_even_though_few_are_sent(self) -> None:
        source = telemetry.Source("right", path=SMALL_RUN, mode="replay", speed=1000.0)
        sent = 0
        aggregated = 0
        for event, payload in source.frames():
            if event != "telemetry":
                source.stop()
                break
            sent += 1
            aggregated += payload["worst"]["rows"]
        self.assertEqual(aggregated, SMALL_RUN_ROWS)
        self.assertLess(sent, SMALL_RUN_ROWS // 4)  # the panel samples, the worst does not

    def test_the_frame_reports_age_and_leaves_staleness_to_the_browser(self) -> None:
        source = telemetry.Source("right", path=SMALL_RUN, mode="replay", speed=1000.0)
        for event, payload in source.frames():
            self.assertIn("server_age_s", payload)
            self.assertGreaterEqual(payload["server_age_s"], 0.0)
            source.stop()
            break

    def test_a_replay_with_no_file_heartbeats_instead_of_failing(self) -> None:
        missing = Path("/nonexistent/loop_log_right.csv")
        source = telemetry.Source("right", path=missing, mode="replay", speed=1000.0)
        event, payload = next(iter(source.frames()))
        source.stop()
        self.assertEqual(event, "heartbeat")
        self.assertEqual(payload["reason"], "replay ended")
        self.assertIsNone(payload["server_age_s"])
        self.assertFalse(payload["growing"])

    def test_a_bad_mode_or_speed_is_refused_at_construction(self) -> None:
        with self.assertRaises(ValueError):
            telemetry.Source("right", mode="pretend")
        with self.assertRaises(ValueError):
            telemetry.Source("right", mode="replay", speed=0.0)


class LiveSourceTest(unittest.TestCase):
    """The live path, driven by a file this test appends to itself."""

    def setUp(self) -> None:
        self.root = Path(tempfile.mkdtemp(prefix="panel_live_test_"))
        self.addCleanup(shutil.rmtree, self.root, ignore_errors=True)
        self.path = self.root / "loop_log_right_live.csv"

    def append(self, text: str) -> None:
        with open(self.path, "a", encoding="utf-8") as handle:
            handle.write(text)

    def test_it_heartbeats_before_the_controller_creates_its_file(self) -> None:
        source = telemetry.Source(
            "right", path=self.path, mode="live", poll_interval_s=0.0)
        event, payload = next(iter(source.frames()))
        source.stop()
        self.assertEqual(event, "heartbeat")
        self.assertEqual(payload["reason"], "no file yet")

    def test_it_reports_rows_then_says_the_file_stopped_growing(self) -> None:
        self.append("# reference_source = pose_trajectory (Config.h)\n")
        self.append("time_s,dt_s,cmd_j1,meas_j1,joint_margin_deg\n")
        self.append("0.002,0.002,10,9.5,20\n")
        source = telemetry.Source(
            "right", path=self.path, mode="live", poll_interval_s=0.0)
        frames = source.frames()

        event, payload = next(frames)
        self.assertEqual(event, "telemetry")
        self.assertAlmostEqual(payload["row"]["follow_err_deg"], 0.5)
        self.assertEqual(payload["rows_seen"], 1)
        self.assertEqual(source.preamble["reference_source"], "pose_trajectory")

        event, payload = next(frames)
        self.assertEqual(event, "heartbeat")
        self.assertEqual(payload["reason"], "file not growing")
        self.assertFalse(payload["growing"])

        self.append("0.004,0.002,10,8.0,19\n")
        event, payload = next(frames)
        source.stop()
        self.assertEqual(event, "telemetry")
        self.assertAlmostEqual(payload["worst"]["follow_err_deg"], 2.0)
        self.assertAlmostEqual(payload["worst"]["joint_margin_deg"], 19.0)
        self.assertEqual(payload["rows_seen"], 2)

    def test_it_finds_the_newest_run_when_given_no_path(self) -> None:
        self.use_fake_runs_dir()
        self.write_run("loop_log_right_20260811_153920.csv", 0.1)
        source = telemetry.Source("right", mode="live", poll_interval_s=0.0)
        event, payload = next(iter(source.frames()))
        source.stop()
        self.assertEqual(event, "telemetry")
        self.assertAlmostEqual(payload["row"]["time_s"], 0.1)

    def test_it_moves_to_the_next_run_once_the_old_one_goes_quiet(self) -> None:
        self.use_fake_runs_dir()
        self.write_run("loop_log_right_20260811_153920.csv", 0.1)
        source = telemetry.Source("right", mode="live", poll_interval_s=0.0)
        frames = source.frames()

        event, payload = next(frames)
        self.assertAlmostEqual(payload["row"]["time_s"], 0.1)

        # The old run stops; a new session writes a new file beside it.
        event, payload = next(frames)
        self.assertEqual(event, "heartbeat")
        self.write_run("loop_log_right_20260811_160000.csv", 5.0)

        event, payload = next(frames)
        source.stop()
        self.assertEqual(event, "telemetry")
        self.assertAlmostEqual(payload["row"]["time_s"], 5.0)

    def use_fake_runs_dir(self) -> None:
        patch = mock.patch.object(telemetry.paths, "RUNS", self.root)
        patch.start()
        self.addCleanup(patch.stop)

    def write_run(self, name: str, time_s: float) -> Path:
        day = self.root / "2026-08-11"
        day.mkdir(exist_ok=True)
        path = day / name
        path.write_text(
            f"time_s,cmd_j1,meas_j1\n{time_s},10,10\n", encoding="utf-8")
        import os
        now = time.time()
        os.utime(path, (now, now))
        return path


if __name__ == "__main__":
    unittest.main()
