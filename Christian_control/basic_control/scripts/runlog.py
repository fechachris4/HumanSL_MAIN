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
    """Newest CSV in the newest runs/ day folder (skipping empty days).

    Newest by MODIFICATION TIME, not by name. Since --arm both, a day holds
    loop_log_left_* and loop_log_right_* side by side, and picking the
    lexically last name returns a right-arm log whenever one exists — on
    runs/2026-08-06 that was a log four hours older than the newest left one.
    Silently analysing the wrong arm is the worst possible failure for a tool
    whose whole job is to say what happened. Mirrors FindLatestRunCsv in
    planner_bridge/src/StartState.cpp, which has always used mtime.
    """
    if not RUNS_DIR.is_dir():
        raise SystemExit(f"no {RUNS_DIR} directory; pass a CSV explicitly")
    csvs = [c for c in RUNS_DIR.glob("*/*.csv") if c.is_file()]
    if not csvs:
        raise SystemExit(f"no CSVs under {RUNS_DIR}; pass a CSV explicitly")
    return max(csvs, key=lambda c: c.stat().st_mtime)


def has_exchange_timestamps(meta, columns):
    """Whether a log has both clocks required for timestamp matching.

    The preamble version describes the full schema, but formats 2 through 11
    all retain these two columns with the same semantics. Unknown, malformed,
    and legacy versions deliberately fall back even if a file happens to
    contain similarly named columns, because their timestamp semantics are
    not a supported contract.

    Raise the upper bound in the SAME commit as any log_format bump. A format
    past this bound does not error — it falls through to the time_s fallback
    with only a printed NOTE, so every timing number downstream silently
    becomes the wrong clock.
    """
    try:
        log_format = int(meta.get("log_format"))
    except (AttributeError, TypeError, ValueError):
        return False
    # 11 is a retired writer format kept readable here; the current writer
    # emits 9.
    return 2 <= log_format <= 11 and "t_send_s" in columns and "t_recv_s" in columns
