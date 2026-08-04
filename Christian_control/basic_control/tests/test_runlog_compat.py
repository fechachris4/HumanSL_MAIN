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
    def write_log(self, path, log_format, include_timestamps):
        fields = [
            "time_s", "dt_s", "pd_x", "pd_y", "pd_z", "p_x", "p_y", "p_z",
            "refresh_ok", "base_fault",
        ] + [f"fault_j{joint}" for joint in range(1, 8)]
        if include_timestamps:
            fields += ["t_send_s", "t_recv_s"]
        targets = [0.0, 0.0, 0.0, 0.02, 0.02, 0.02]
        measured = [0.0, 0.0, 0.0, 0.0, 0.010, 0.019]
        with path.open("w", newline="") as output:
            if log_format is not None:
                output.write(f"# log_format = {log_format} (compiled)\n")
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
                }
                if include_timestamps:
                    values.update({
                        "t_send_s": time_s + 0.01,
                        "t_recv_s": time_s + 0.02,
                    })
                values.update({f"fault_j{joint}": 0 for joint in range(1, 8)})
                writer.writerow(values)

    def run_script(self, script, path, *extra_args):
        result = subprocess.run(
            [sys.executable, str(script), str(path), *extra_args],
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        return result.stdout

    def test_supported_formats_with_both_timing_columns_are_current(self):
        columns = {"t_send_s": [], "t_recv_s": []}
        for log_format in range(2, 7):
            with self.subTest(log_format=log_format):
                self.assertTrue(
                    runlog.has_exchange_timestamps({"log_format": str(log_format)}, columns)
                )

    def test_unsupported_or_malformed_formats_fall_back(self):
        columns = {"t_send_s": [], "t_recv_s": []}
        for preamble in (
            {"log_format": "1"},
            {"log_format": "7"},
            {},
            {"log_format": "six"},
            {"log_format": "6.0"},
        ):
            with self.subTest(preamble=preamble):
                self.assertFalse(runlog.has_exchange_timestamps(preamble, columns))

    def test_both_timing_columns_are_required(self):
        self.assertFalse(
            runlog.has_exchange_timestamps({"log_format": "6"}, {"t_send_s": []})
        )
        self.assertFalse(
            runlog.has_exchange_timestamps({"log_format": "6"}, {"t_recv_s": []})
        )

    def test_legacy_log_without_timing_columns_falls_back(self):
        self.assertFalse(runlog.has_exchange_timestamps({}, {"time_s": []}))

    def test_format_six_uses_exchange_timestamps_in_both_analyzers(self):
        with tempfile.TemporaryDirectory() as directory:
            log_path = Path(directory) / "format6.csv"
            legacy_path = Path(directory) / "legacy.csv"
            self.write_log(log_path, 6, include_timestamps=True)
            self.write_log(legacy_path, 1, include_timestamps=False)

            exchange_analyze = self.run_script(ANALYZE_RUN, log_path, "--force")
            legacy_analyze = self.run_script(ANALYZE_RUN, legacy_path, "--force")
            exchange_delay = self.run_script(MEASURE_DELAY, log_path)
            legacy_delay = self.run_script(MEASURE_DELAY, legacy_path)

            self.assertNotIn("NOTE: old log format", exchange_analyze)
            self.assertIn("NOTE: old log format", legacy_analyze)
            self.assertRegex(exchange_analyze, r"RMS position error \(raw\):\s+4\.21 mm")
            self.assertRegex(legacy_analyze, r"RMS position error \(raw\):\s+9\.14 mm")
            self.assertRegex(exchange_delay, r"delay \(command sent -> first motion\): 10\.0 ms")
            self.assertRegex(legacy_delay, r"delay \(command sent -> first motion\): 500\.0 ms")


if __name__ == "__main__":
    unittest.main()
