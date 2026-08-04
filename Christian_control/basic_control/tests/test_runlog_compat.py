#!/usr/bin/env python3
"""Regression checks for analyzer timing-column compatibility."""

import csv
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
ANALYZE_RUN = SCRIPTS_DIR / "analyze_run.py"
MEASURE_DELAY = SCRIPTS_DIR / "measure_delay.py"
sys.path.insert(0, str(SCRIPTS_DIR))

import runlog  # noqa: E402


class ExchangeTimestampCompatibilityTest(unittest.TestCase):
    def write_format_six_log(self, path):
        fields = [
            "time_s", "dt_s", "pd_x", "pd_y", "pd_z", "p_x", "p_y", "p_z",
            "refresh_ok", "base_fault", "t_send_s", "t_recv_s",
        ] + [f"fault_j{joint}" for joint in range(1, 8)]
        targets = [0.0, 0.0, 0.0, 0.02, 0.02, 0.02]
        measured = [0.0, 0.0, 0.0, 0.0, 0.010, 0.019]
        with path.open("w", newline="") as output:
            output.write("# log_format = 6 (compiled)\n")
            writer = csv.DictWriter(output, fieldnames=fields)
            writer.writeheader()
            for row, (target, actual) in enumerate(zip(targets, measured)):
                time_s = 0.5 * row
                values = {
                    "time_s": time_s,
                    "dt_s": 0.5,
                    "pd_x": target,
                    "pd_y": 0.0,
                    "pd_z": 0.0,
                    "p_x": actual,
                    "p_y": 0.0,
                    "p_z": 0.0,
                    "refresh_ok": 1,
                    "base_fault": 0,
                    "t_send_s": time_s + 0.01,
                    "t_recv_s": time_s + 0.02,
                }
                values.update({f"fault_j{joint}": 0 for joint in range(1, 8)})
                writer.writerow(values)

    def test_format_six_with_both_timing_columns_is_current(self):
        self.assertTrue(
            runlog.has_exchange_timestamps(
                {"log_format": "6"}, {"t_send_s": [], "t_recv_s": []}
            )
        )

    def test_both_timing_columns_are_required(self):
        self.assertFalse(
            runlog.has_exchange_timestamps({"log_format": "6"}, {"t_send_s": []})
        )
        self.assertFalse(
            runlog.has_exchange_timestamps({"log_format": "6"}, {"t_recv_s": []})
        )

    def test_legacy_log_without_timing_columns_falls_back(self):
        self.assertFalse(runlog.has_exchange_timestamps({}, {"time_s": []}))

    def test_format_six_timestamps_do_not_emit_old_format_note(self):
        with tempfile.TemporaryDirectory() as directory:
            log_path = Path(directory) / "format6.csv"
            self.write_format_six_log(log_path)
            for script, extra_args in ((ANALYZE_RUN, ["--force"]), (MEASURE_DELAY, [])):
                result = subprocess.run(
                    [sys.executable, str(script), str(log_path), *extra_args],
                    text=True,
                    capture_output=True,
                    check=False,
                )
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertNotIn("NOTE: old log format", result.stdout)


if __name__ == "__main__":
    unittest.main()
