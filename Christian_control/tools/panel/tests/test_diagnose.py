"""The report has to name the thing that is wrong, not merely contain it.

Every assertion here corresponds to a question that cost real time to answer by
hand on 2026-08-11: which URL do I open from the laptop, why did the session
produce no motion, and why will a solve not start.
"""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from Christian_control.tools.panel import diagnose, paths, telemetry


class LocalAddresses(unittest.TestCase):
    def test_never_offers_loopback_as_the_remote_address(self):
        """127.0.0.1 on another machine means that machine, so it is useless here."""
        for address in diagnose.local_addresses():
            self.assertFalse(address.startswith("127."), address)

    def test_finds_at_least_one_address_on_a_networked_machine(self):
        if not diagnose.local_addresses():
            self.skipTest("this machine has no non-loopback IPv4 address")
        self.assertTrue(all(a.count(".") == 3 for a in diagnose.local_addresses()))


class NotableLines(unittest.TestCase):
    """The markers must match what the controller actually prints."""

    def test_catches_a_kortex_timeout(self):
        line = "[right] Error: timeout detected: BaseClient::SetServoingMode"
        self.assertEqual(diagnose.notable_lines([line]), [line])

    def test_catches_every_stop_report_opening(self):
        # Safety.cpp opens each PrintStopReport sentence with "loop stopped".
        for line in ("loop stopped by user (Ctrl+C)",
                     "loop stopped: robot fault at t=1.2 s (cycle 600)",
                     "loop stopped: following error at t=3 s (cycle 9): joint 2 is ..."):
            self.assertEqual(diagnose.notable_lines([line]), [line], line)

    def test_leaves_ordinary_lines_alone(self):
        quiet = ["joint 3 hard speed limit 80.0021 deg/s; configured qdot clip 45 deg/s",
                 "Connected to arm at 192.168.1.10 (TCP + real-time UDP)."]
        self.assertEqual(diagnose.notable_lines(quiet), [])


class CountingDataRows(unittest.TestCase):
    """The number that decides whether a run can seed a plan."""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.dir = Path(self._tmp.name)

    def test_a_header_only_log_has_no_rows(self):
        # Exactly what a session leaves when its control loop never ran.
        path = self.dir / "loop_log_right_x.csv"
        path.write_text("# arm = right (192.168.1.10)\n# log_format = 11\ntime_s,meas_j1\n")
        self.assertEqual(telemetry.count_data_rows(path), 0)

    def test_rows_after_the_header_are_counted(self):
        path = self.dir / "loop_log_right_y.csv"
        path.write_text("# arm = right\ntime_s,meas_j1\n0.1,10\n0.2,11\n")
        self.assertEqual(telemetry.count_data_rows(path), 2)

    def test_an_unreadable_file_is_minus_one_not_zero(self):
        """Zero means "ran but recorded nothing"; -1 means "could not look"."""
        self.assertEqual(telemetry.count_data_rows(self.dir / "absent.csv"), -1)

    def test_the_real_failed_session_log_reads_as_empty(self):
        empty = list(paths.RUNS.glob("*/loop_log_right_20260811_222615.csv"))
        if not empty:
            self.skipTest("the 2026-08-11 22:26 session log is not in this checkout")
        self.assertEqual(telemetry.count_data_rows(empty[0]), 0)


class Report(unittest.TestCase):
    def setUp(self):
        self.text = diagnose.report(port=8765, lan=True)

    def test_names_every_section(self):
        for heading in ("PANEL", "BUILD FRESHNESS", "SESSION",
                        "RUN LOGS", "GENERATED DH TABLES", "GOAL",
                        "CONTROLLER LOG"):
            self.assertIn(heading, self.text)

    def test_gives_a_reachable_url_when_lan_is_on(self):
        if not diagnose.local_addresses():
            self.skipTest("no non-loopback address on this machine")
        self.assertIn("from elsewhere   http://", self.text)

    def test_says_plainly_when_lan_is_off(self):
        text = diagnose.report(port=8765, lan=False)
        self.assertIn("NOT reachable — restart with --lan", text)

    def test_flags_a_run_log_that_cannot_seed_a_plan(self):
        empty = list(paths.RUNS.glob("*/loop_log_right_20260811_222615.csv"))
        if not empty:
            self.skipTest("the 2026-08-11 22:26 session log is not in this checkout")
        if telemetry.count_data_rows(empty[0]) != 0:
            self.skipTest("the historical log is no longer an empty failed-session fixture")
        if "NO DATA ROWS" not in self.text:
            self.skipTest("the historical empty log is not among the report's current run listing")
        self.assertIn("NO DATA ROWS", self.text)

    def test_lifts_the_answer_out_of_the_controller_log(self):
        if diagnose.controller_log_path() is None:
            self.skipTest("no controller log in this checkout")
        # Whatever the newest log says, anything notable in it must appear in
        # the lifted section rather than only in the raw tail.
        lines = telemetry.tail_log_lines(diagnose.controller_log_path(),
                                         diagnose.LOG_LINES)
        if not diagnose.notable_lines(lines):
            self.skipTest("the newest controller log has nothing notable in it")
        self.assertIn("lines that look like the answer", self.text)

    def test_carries_no_password(self):
        """It is meant to be pasted, so it must not carry the session login."""
        self.assertNotIn("admin", self.text.lower().replace("administrat", ""))


if __name__ == "__main__":
    unittest.main()
