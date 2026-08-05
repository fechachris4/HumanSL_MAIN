"""Shared helper: locate the newest controller run CSV.

The controller writes one timestamped CSV per run into
<repo root>/runs/YYYY-MM-DD/ (see main.cpp / RUNS_ROOT_DIR in
CMakeLists.txt). Both plot scripts use find_default_csv() when no CSV
is given explicitly.
"""

from pathlib import Path

# scripts/ -> basic_control -> Christian_control -> repo root
RUNS_DIR = Path(__file__).resolve().parents[3] / "runs"


def find_default_csv():
    """Newest CSV in the newest runs/ day folder (skipping empty days)."""
    if not RUNS_DIR.is_dir():
        raise SystemExit(f"no {RUNS_DIR} directory; pass a CSV explicitly")
    for day in sorted((d for d in RUNS_DIR.iterdir() if d.is_dir()),
                      reverse=True):
        csvs = sorted(day.glob("*.csv"))
        if csvs:
            return csvs[-1]  # timestamped names sort chronologically
    raise SystemExit(f"no CSVs under {RUNS_DIR}; pass a CSV explicitly")


def has_exchange_timestamps(meta, columns):
    """Whether a log has both clocks required for timestamp matching.

    The preamble version describes the full schema, but formats 2 through 8
    all retain these two columns. Unknown, malformed, and legacy versions
    deliberately fall back even if a file happens to contain similarly named
    columns, because their timestamp semantics are not a supported contract.
    """
    try:
        log_format = int(meta.get("log_format"))
    except (AttributeError, TypeError, ValueError):
        return False
    return 2 <= log_format <= 8 and "t_send_s" in columns and "t_recv_s" in columns
