#!/usr/bin/env python3
"""Tests for scripts/extract_execution_history.py (Plan 01, Task 1).

The extractor turns a HISTORICAL controller run log (log_format 9, 10 or
11) into one versioned execution-history CSV. These logs predate the
world-Cartesian reference path (format 13), so they can only ever prove
the fields they actually contain: the extractor must copy available named
fields verbatim, mark the source format, and leave absent fields EMPTY.
Writing a zero where the source has no column would fabricate evidence
(prediction P7 of the accepted analysis packet), so several tests below
exist purely to catch zero-filling.

The format 9/10/11 header literals in this file were copied from real run
logs (runs/2026-08-13/loop_log_left_20260813_143008.csv, _182005.csv,
_194554.csv) so the extractor's idea of each format is pinned to evidence
independent of the extractor's own code.
"""

import csv
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
EXTRACTOR = SCRIPTS_DIR / "extract_execution_history.py"
sys.path.insert(0, str(SCRIPTS_DIR))

import extract_execution_history as extractor  # noqa: E402


def _j7(prefix):
    return [f"{prefix}_j{joint}" for joint in range(1, 8)]


# Copied from a real format-9 log header (2026-08-13). Do not regenerate
# from the extractor module: this literal is the independent oracle.
FORMAT9_HEADER = (
    ["time_s", "dt_s", "pd_x", "pd_y", "pd_z", "p_x", "p_y", "p_z"]
    + _j7("cmd") + _j7("cmdvel") + _j7("meas") + _j7("measraw")
    + _j7("vel") + _j7("torque") + _j7("fault")
    + ["arm_state", "base_fault", "refresh_ok", "sigma_min",
       "rot_error_rad", "t_send_s", "t_recv_s",
       "quat_x", "quat_y", "quat_z", "quat_w",
       "command_frame_id", "feedback_frame_id"]
    + _j7("command_ack") + _j7("status_flags") + _j7("jitter_us")
    + ["cycle"] + _j7("req") + _j7("reqvel") + _j7("lead_limited")
    + _j7("ack_unchanged") + _j7("taskvel") + _j7("nullvel")
    + ["null_leak_mps", "traj_activated", "traj_rejected", "traj_complete",
       "traj_start_error_deg", "joint_follow_stop", "joint_follow_error_deg"]
)

VICON_SEGMENTS = ["mount", "leftbase", "rightbase", "leftee", "rightee"]
VICON_BLOCK = ["vicon_seq", "vicon_frame", "vicon_latency_s", "vicon_age_s"] + [
    f"vicon_{seg}_{field}"
    for seg in VICON_SEGMENTS
    for field in ["x_m", "y_m", "z_m", "qx", "qy", "qz", "qw", "valid"]
]
HOLD_BLOCK = ["hold_state", "world_err_m", "world_err_rot_rad", "hold_ramp",
              "hold_reanchor_count"]

FORMAT10_HEADER = FORMAT9_HEADER + VICON_BLOCK
FORMAT11_HEADER = FORMAT10_HEADER + HOLD_BLOCK


def make_row(header, **overrides):
    """One plausible data row: named defaults, everything else 0."""
    row = {name: "0" for name in header}
    row.update({
        "time_s": "1.25", "dt_s": "0.002", "t_send_s": "1.2504",
        "t_recv_s": "1.2509", "cycle": "17", "refresh_ok": "1",
        "sigma_min": "0.21", "rot_error_rad": "0.001",
    })
    for joint in range(1, 8):
        row[f"meas_j{joint}"] = f"{10.0 + joint:.6g}"
        row[f"cmd_j{joint}"] = f"{10.1 + joint:.6g}"
        row[f"req_j{joint}"] = f"{10.2 + joint:.6g}"
        row[f"reqvel_j{joint}"] = "0.5"
        row[f"cmdvel_j{joint}"] = "0.4"
    row.update(overrides)
    return row


def write_log(path, log_format, header, rows, preamble=True,
              preamble_format=None):
    with path.open("w", newline="") as output:
        if preamble:
            output.write("# controller = basic_control\n")
            output.write("# qdot_limit_deg_s = 76 76 76 76 66.5 66.5 66.5\n")
            declared = log_format if preamble_format is None else preamble_format
            output.write(f"# log_format = {declared} (compiled)\n")
        writer = csv.writer(output)
        writer.writerow(header)
        for row in rows:
            writer.writerow([row.get(name, "0") for name in header])


class ExtractExecutionHistoryTest(unittest.TestCase):
    def run_extractor(self, input_path, output_path):
        return subprocess.run(
            [sys.executable, str(EXTRACTOR), str(input_path),
             "-o", str(output_path)],
            text=True, capture_output=True, check=False)

    def extract_ok(self, input_path, output_path):
        result = self.run_extractor(input_path, output_path)
        self.assertEqual(result.returncode, 0,
                         result.stdout + result.stderr)
        return result

    def read_output(self, path):
        preamble = {}
        with path.open() as handle:
            line = handle.readline()
            while line.startswith("#"):
                key, _, value = line[1:].partition("=")
                preamble[key.strip()] = value.strip()
                line = handle.readline()
            fieldnames = next(csv.reader([line]))
            rows = list(csv.DictReader(handle, fieldnames=fieldnames))
        return preamble, fieldnames, rows

    # ---- format headers are pinned to real logs, not to the module ----

    def test_format_headers_match_real_logs(self):
        self.assertEqual(extractor.FORMAT_HEADERS[9], FORMAT9_HEADER)
        self.assertEqual(extractor.FORMAT_HEADERS[10], FORMAT10_HEADER)
        self.assertEqual(extractor.FORMAT_HEADERS[11], FORMAT11_HEADER)

    def test_union_schema_has_no_cartesian_reference_fields(self):
        # Formats 9-11 predate the world-Cartesian reference path, so the
        # output schema must not even OFFER cart_* columns to zero-fill.
        for name in extractor.OUTPUT_FIELDS:
            self.assertFalse(name.startswith("cart_"), name)
        self.assertEqual(extractor.OUTPUT_FIELDS,
                         ["source_log_format"] + FORMAT11_HEADER)

    # ---- extraction of available fields, verbatim ----

    def test_format9_extracts_available_fields_and_leaves_rest_empty(self):
        with tempfile.TemporaryDirectory() as directory:
            src = Path(directory) / "f9.csv"
            out = Path(directory) / "f9_extract.csv"
            rows = [make_row(FORMAT9_HEADER,
                             meas_j3="123.456789012345", cycle="3"),
                    make_row(FORMAT9_HEADER, traj_activated="1")]
            write_log(src, 9, FORMAT9_HEADER, rows)
            self.extract_ok(src, out)

            preamble, fieldnames, out_rows = self.read_output(out)
            self.assertEqual(preamble.get("execution_history_extract"), "1")
            self.assertEqual(preamble.get("source_log_format"), "9")
            self.assertEqual(fieldnames, extractor.OUTPUT_FIELDS)
            self.assertEqual(len(out_rows), 2)
            self.assertEqual(out_rows[0]["source_log_format"], "9")
            # Verbatim copies, never reformatted floats.
            self.assertEqual(out_rows[0]["meas_j3"], "123.456789012345")
            self.assertEqual(out_rows[1]["traj_activated"], "1")
            # Fields a format-9 log cannot prove stay EMPTY, never zero.
            for name in VICON_BLOCK + HOLD_BLOCK:
                self.assertEqual(out_rows[0][name], "",
                                 f"{name} must be empty for format 9")

    def test_format11_passes_vicon_and_hold_fields(self):
        with tempfile.TemporaryDirectory() as directory:
            src = Path(directory) / "f11.csv"
            out = Path(directory) / "f11_extract.csv"
            row = make_row(FORMAT11_HEADER,
                           vicon_mount_x_m="0.5012", hold_state="2",
                           vicon_age_s="0.0097")
            write_log(src, 11, FORMAT11_HEADER, [row])
            self.extract_ok(src, out)
            preamble, _, out_rows = self.read_output(out)
            self.assertEqual(preamble.get("source_log_format"), "11")
            self.assertEqual(out_rows[0]["vicon_mount_x_m"], "0.5012")
            self.assertEqual(out_rows[0]["vicon_age_s"], "0.0097")
            self.assertEqual(out_rows[0]["hold_state"], "2")

    # ---- strict rejections ----

    def test_rejects_unsupported_format(self):
        with tempfile.TemporaryDirectory() as directory:
            src = Path(directory) / "f12.csv"
            write_log(src, 12, FORMAT11_HEADER, [make_row(FORMAT11_HEADER)])
            result = self.run_extractor(src, Path(directory) / "out.csv")
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("log_format", result.stderr)

    def test_rejects_missing_preamble(self):
        with tempfile.TemporaryDirectory() as directory:
            src = Path(directory) / "bare.csv"
            write_log(src, 9, FORMAT9_HEADER, [make_row(FORMAT9_HEADER)],
                      preamble=False)
            result = self.run_extractor(src, Path(directory) / "out.csv")
            self.assertNotEqual(result.returncode, 0)

    def test_rejects_header_that_does_not_match_declared_format(self):
        with tempfile.TemporaryDirectory() as directory:
            src = Path(directory) / "mismatch.csv"
            # Declares 9 but carries the format-11 columns.
            write_log(src, 9, FORMAT11_HEADER, [make_row(FORMAT11_HEADER)],
                      preamble_format=9)
            result = self.run_extractor(src, Path(directory) / "out.csv")
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("header", result.stderr.lower())

    def test_rejects_duplicate_header_column(self):
        with tempfile.TemporaryDirectory() as directory:
            src = Path(directory) / "dup.csv"
            header = FORMAT9_HEADER + ["meas_j1"]
            write_log(src, 9, header, [make_row(header)])
            result = self.run_extractor(src, Path(directory) / "out.csv")
            self.assertNotEqual(result.returncode, 0)

    def test_rejects_unnamed_header_column(self):
        with tempfile.TemporaryDirectory() as directory:
            src = Path(directory) / "unnamed.csv"
            header = FORMAT9_HEADER + [""]
            write_log(src, 9, header, [make_row(header)])
            result = self.run_extractor(src, Path(directory) / "out.csv")
            self.assertNotEqual(result.returncode, 0)

    def test_rejects_nonfinite_joint_measurement(self):
        for bad in ("nan", "inf", "-inf"):
            with tempfile.TemporaryDirectory() as directory:
                src = Path(directory) / "nonfinite.csv"
                write_log(src, 9, FORMAT9_HEADER,
                          [make_row(FORMAT9_HEADER, meas_j2=bad)])
                result = self.run_extractor(src, Path(directory) / "out.csv")
                self.assertNotEqual(result.returncode, 0, bad)
                self.assertIn("meas_j2", result.stderr)

    def test_rejects_short_row(self):
        with tempfile.TemporaryDirectory() as directory:
            src = Path(directory) / "short.csv"
            write_log(src, 9, FORMAT9_HEADER, [make_row(FORMAT9_HEADER)])
            with src.open("a") as handle:
                handle.write("1.0,0.002,0,0\n")  # far too few fields
            result = self.run_extractor(src, Path(directory) / "out.csv")
            self.assertNotEqual(result.returncode, 0)

    def test_rejects_overlong_row(self):
        with tempfile.TemporaryDirectory() as directory:
            src = Path(directory) / "long.csv"
            write_log(src, 9, FORMAT9_HEADER, [make_row(FORMAT9_HEADER)])
            with src.open("a") as handle:
                base = ",".join(["0"] * len(FORMAT9_HEADER))
                handle.write(base + ",99\n")  # one unnamed extra value
            result = self.run_extractor(src, Path(directory) / "out.csv")
            self.assertNotEqual(result.returncode, 0)

    def test_skips_comment_lines_after_the_header(self):
        # Real logs carry '#' lines after the header too: startup pose
        # lines right below it and the exit trailer at the end. They are
        # prose, not cycle rows, and must be skipped, not extracted.
        with tempfile.TemporaryDirectory() as directory:
            src = Path(directory) / "trailer.csv"
            out = Path(directory) / "trailer_extract.csv"
            write_log(src, 9, FORMAT9_HEADER, [])
            with src.open("a") as handle:
                handle.write("# startup_joint_deg = 1 2 3 4 5 6 7 "
                             "(measured, actuator order)\n")
                handle.write(",".join(
                    make_row(FORMAT9_HEADER)[n] for n in FORMAT9_HEADER)
                    + "\n")
                handle.write("# exit_reason = user_stop\n")
                handle.write("# exit_cycle = 1\n")
            self.extract_ok(src, out)
            _, _, rows = self.read_output(out)
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["cycle"], "17")

    # ---- the anti-fabrication check (prediction P7) ----

    def test_absent_fields_are_never_zero_filled(self):
        with tempfile.TemporaryDirectory() as directory:
            src = Path(directory) / "f10.csv"
            out = Path(directory) / "f10_extract.csv"
            write_log(src, 10, FORMAT10_HEADER, [make_row(FORMAT10_HEADER)])
            self.extract_ok(src, out)
            _, _, rows = self.read_output(out)
            for name in HOLD_BLOCK:  # absent in format 10
                value = rows[0][name]
                self.assertEqual(value, "", f"{name} fabricated as {value!r}")
                try:
                    fabricated = float(value) == 0.0
                except ValueError:
                    fabricated = False
                self.assertFalse(fabricated)

    def test_extract_does_not_invent_or_interpolate_values(self):
        # Every output value must literally appear in the source row (or be
        # the source_log_format tag / an explicit empty).
        with tempfile.TemporaryDirectory() as directory:
            src = Path(directory) / "f9.csv"
            out = Path(directory) / "f9_extract.csv"
            row = make_row(FORMAT9_HEADER)
            write_log(src, 9, FORMAT9_HEADER, [row])
            self.extract_ok(src, out)
            _, _, rows = self.read_output(out)
            for name, value in rows[0].items():
                if name == "source_log_format":
                    self.assertEqual(value, "9")
                elif name in FORMAT9_HEADER:
                    self.assertEqual(value, row[name])
                else:
                    self.assertEqual(value, "")


if __name__ == "__main__":
    unittest.main()
