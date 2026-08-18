#!/usr/bin/env python3
"""Strict extractor: historical run log (format 9-11) -> execution history CSV.

Purpose (Plan 01, Task 1). Historical controller logs predate the sole
world-Cartesian reference path (log_format 13), so they are PARTIAL
evidence about the execution pipeline: they can prove only the fields they
contain. This tool copies the named fields that exist in a format 9, 10 or
11 log verbatim into one fixed, versioned output schema, tags every row
with source_log_format, and leaves every column the source cannot prove
EXPLICITLY EMPTY. It never writes a zero, interpolates, or synthesizes a
value: an empty field means "this log has no evidence", which is exactly
what downstream comparison against format-13 characterization data needs.

Strictness contract:
  - the '#' preamble must exist and declare a supported log_format (9-11);
  - the CSV header must match that format's known column list exactly
    (pinned to real 2026-08-13 run logs by the tests);
  - duplicate or unnamed header columns are rejected;
  - every row must have exactly one value per named column;
  - meas_j1..meas_j7 (the required joint measurements) must be finite.

Cartesian reference/twist fields (cart_*) do not appear in the output
schema at all: no format 9-11 log can prove them (prediction P7).
"""

import argparse
import csv
import math
import re
import sys


def _j7(prefix):
    return [f"{prefix}_j{joint}" for joint in range(1, 8)]


# Column lists per supported format, pinned by tests to real run logs.
_FORMAT9_HEADER = (
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

_VICON_SEGMENTS = ["mount", "leftbase", "rightbase", "leftee", "rightee"]
_VICON_BLOCK = (
    ["vicon_seq", "vicon_frame", "vicon_latency_s", "vicon_age_s"]
    + [f"vicon_{seg}_{field}"
       for seg in _VICON_SEGMENTS
       for field in ["x_m", "y_m", "z_m", "qx", "qy", "qz", "qw", "valid"]]
)
_HOLD_BLOCK = ["hold_state", "world_err_m", "world_err_rot_rad",
               "hold_ramp", "hold_reanchor_count"]

FORMAT_HEADERS = {
    9: _FORMAT9_HEADER,
    10: _FORMAT9_HEADER + _VICON_BLOCK,
    11: _FORMAT9_HEADER + _VICON_BLOCK + _HOLD_BLOCK,
}

# One fixed output schema: the union of every supported source format
# (format 11 is a strict superset of 9 and 10), plus the provenance tag.
OUTPUT_FIELDS = ["source_log_format"] + FORMAT_HEADERS[11]

# Fields that must parse as finite floats in every row.
REQUIRED_FINITE = _j7("meas")

_PREAMBLE_RE = re.compile(
    r"#\s*([A-Za-z_]\w*)\s*=\s*(.*?)(?:\s*\([^)]*\))?\s*$")


class ExtractError(Exception):
    pass


def read_preamble(handle):
    """Read '#' lines into a dict; return (dict, first non-preamble line)."""
    preamble = {}
    line = handle.readline()
    saw_preamble = False
    while line.startswith("#"):
        saw_preamble = True
        match = _PREAMBLE_RE.match(line)
        if match:
            preamble[match.group(1)] = match.group(2)
        line = handle.readline()
    if not saw_preamble:
        raise ExtractError(
            "no '#' preamble found: refusing a log with no declared "
            "log_format (nothing identifies which fields it can prove)")
    return preamble, line


def source_format(preamble):
    raw = preamble.get("log_format")
    if raw is None:
        raise ExtractError("preamble declares no log_format")
    try:
        value = int(raw)
    except ValueError:
        raise ExtractError(f"log_format is not an integer: {raw!r}")
    if value not in FORMAT_HEADERS:
        raise ExtractError(
            f"unsupported log_format {value}: this extractor handles "
            f"historical formats {sorted(FORMAT_HEADERS)} only")
    return value


def check_header(header, log_format):
    for index, name in enumerate(header):
        if not name:
            raise ExtractError(f"unnamed header column at index {index}")
    duplicates = {name for name in header if header.count(name) > 1}
    if duplicates:
        raise ExtractError(f"duplicate header columns: {sorted(duplicates)}")
    expected = FORMAT_HEADERS[log_format]
    if header != expected:
        missing = [n for n in expected if n not in header]
        extra = [n for n in header if n not in expected]
        raise ExtractError(
            f"header does not match log_format {log_format}: "
            f"missing {missing[:5]}, unexpected {extra[:5]}")


def extract(input_path, output_stream):
    with open(input_path, newline="") as handle:
        preamble, header_line = read_preamble(handle)
        log_format = source_format(preamble)
        if not header_line:
            raise ExtractError("log ends before the CSV header")
        header = next(csv.reader([header_line]))
        check_header(header, log_format)

        output_stream.write("# execution_history_extract = 1\n")
        output_stream.write(f"# source_log_format = {log_format}\n")
        writer = csv.writer(output_stream, lineterminator="\n")
        writer.writerow(OUTPUT_FIELDS)

        source_columns = set(header)
        reader = csv.reader(handle)
        for row_number, values in enumerate(reader, start=1):
            if not values:
                continue  # trailing blank line
            if values[0].lstrip().startswith("#"):
                # Real logs carry '#' comment lines after the header too
                # (startup pose lines, the exit trailer). The log's own
                # convention is "parsers skip '#' lines"; they are prose,
                # not cycle evidence, and are not extracted.
                continue
            if len(values) != len(header):
                raise ExtractError(
                    f"row {row_number}: {len(values)} values for "
                    f"{len(header)} named columns")
            row = dict(zip(header, values))
            for name in REQUIRED_FINITE:
                try:
                    number = float(row[name])
                except ValueError:
                    raise ExtractError(
                        f"row {row_number}: {name} is not a number: "
                        f"{row[name]!r}")
                if not math.isfinite(number):
                    raise ExtractError(
                        f"row {row_number}: {name} is non-finite "
                        f"({row[name]!r}); a joint measurement the "
                        f"pipeline acted on must be finite")
            # Verbatim copy of available fields; explicit empty otherwise.
            writer.writerow(
                [str(log_format)]
                + [row[name] if name in source_columns else ""
                   for name in OUTPUT_FIELDS[1:]])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("input", help="historical run CSV (log_format 9-11)")
    parser.add_argument("-o", "--output",
                        help="output CSV path (default: stdout)")
    args = parser.parse_args(argv)
    try:
        if args.output:
            with open(args.output, "w", newline="") as output:
                extract(args.input, output)
        else:
            extract(args.input, sys.stdout)
    except ExtractError as error:
        print(f"extract_execution_history: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
