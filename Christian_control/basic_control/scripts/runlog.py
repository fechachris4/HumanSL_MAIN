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
