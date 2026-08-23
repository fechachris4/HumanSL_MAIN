"""Read the controller's own CSV and turn it into frames for the browser.

This module only ever reads. It follows the log the controller already
writes, so nothing here can reach the control loop even by accident, and
the panel keeps working whether the session was started from the panel or
from a terminal.

Two things it has to get right, both learned the hard way in this project:

Columns are read by NAME. The format history in Hardware.h records that
tooling reading by index broke every time a column was added or dropped,
and format 11 has 146 of them. A renamed or missing column here loses one
value; it never shifts every other value by one.

'nan' stays missing. The controller writes nan for a quantity that did not
exist on that cycle — no pose reference, no null-space law, no posture
guidance. Parsed as 0.0 that reads as perfect tracking, which is the most
dangerous possible misreading, so nan becomes None and the UI is left to
say "no value" rather than "no error".

The log runs at 500 Hz and the panel draws at about 20, so sampling alone
would step straight over a 4 ms spike. Every frame therefore carries the
newest row AND the worst value seen since the previous frame (Aggregator).

Replay mode reads a recorded run from runs/ and paces it by the file's own
time_s, so the whole screen can be built and demonstrated with no robot.
"""

from __future__ import annotations

import math
import re
import time
from pathlib import Path
from typing import Iterable, Iterator, Optional

from . import paths


# The preamble annotates each value with where it came from — "(Config.h)",
# "(--log)", "(compiled)", "(default)", "(measured FK, base_link)". The
# annotation is provenance, not data, so it is stripped. The one line where
# it also carries information is "arm = right (192.168.1.10)", whose IP is
# dropped with it; the panel gets the address from the session, not here.
_ANNOTATION = re.compile(r"\s*\([^()]*\)\s*$")

# What Aggregator counts as a badly late cycle, as a multiple of the
# nominal period. The controller's own overrun stop uses
# config::kOverrunFactor (1.5 today) and this counter is deliberately
# coarser: it is a display of lateness, not the guard, and must never be
# read as one.
DEFAULT_OVERRUN_FACTOR = 2.0

# The loop period the panel assumes until a run's preamble tells it
# otherwise (control_dt_s).
DEFAULT_NOMINAL_DT_S = 0.002

# The edge flags whose firing anywhere inside a frame's window matters, even
# though they are 1 on a single 2 ms cycle and sampling would miss them.
EDGE_FLAGS = (
    "traj_activated",
    "traj_rejected",
    "traj_complete",
    "joint_follow_stop",
    # Current world-Cartesian format-14 edge fields. They are true for one
    # 2 ms controller cycle, so include them in the panel frame window or the
    # browser can step over the event at its ~20 Hz display rate.
    "cart_replan_requested",
    "cart_traj_activated",
)


def parse_preamble(lines: Iterable[str]) -> dict[str, str]:
    """Read the '#' key = value lines the controller writes around its data.

    The preamble is authoritative for that run: it records the compiled
    reference source, the arrival tolerances, following_error_limit_deg and
    the guard override states as they were in the binary that produced these
    rows. Config.h may have moved on since, so anything shown against a
    recorded run should come from here.

    Values are returned as text and left uncoerced, because the preamble
    mixes numbers, booleans, enum names and whole vectors, and the caller
    knows which it wants.

    Lines without '=' (the banner line) are skipped. Trailer lines the
    controller appends after the data — exit_reason, exit_cycle,
    faults_observed — are ordinary preamble lines and land in the same dict.
    """
    out: dict[str, str] = {}
    for raw in lines:
        line = raw.strip()
        if not line.startswith("#"):
            continue
        body = line[1:].strip()
        if "=" not in body:
            continue
        key, _, value = body.partition("=")
        key = key.strip()
        value = value.strip()
        stripped = _ANNOTATION.sub("", value).strip()
        if key:
            out[key] = stripped if stripped else value
    return out


def read_preamble(path: Path) -> dict[str, str]:
    """Every '#' line of a finished run, including the exit trailer.

    Streams the file rather than reading it whole: the data between the
    header and the trailer can be tens of megabytes and none of it is
    wanted here.
    """
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as handle:
            return parse_preamble(line for line in handle if line.startswith("#"))
    except OSError:
        return {}


def wrap180(degrees: float) -> float:
    """The same angle expressed in [-180, 180].

    Kortex reports joint positions on [0, 360) while commands, limits and
    trajectories are signed, so one physical angle arrives a full turn
    apart: a joint truly at -20 deg reads 340 deg. A raw cmd - meas then
    reports 360 deg of following error for a joint that is exactly where it
    was told to be. math.remainder is the same fix State.h's
    WrappedJointError applies in the controller, in radians.
    """
    return math.remainder(degrees, 360.0)


def _num(row: dict, key: str) -> Optional[float]:
    """A finite float from a parsed row, or None if it is not one.

    Missing column, nan, or a value that never parsed as a number all
    answer None, so one odd column cannot take the whole frame down.
    """
    value = row.get(key)
    if isinstance(value, float) and math.isfinite(value):
        return value
    if isinstance(value, int) and not isinstance(value, bool):
        return float(value)
    return None


class LineTail:
    """A byte position in a growing text file.

    Holds the offset it has read to and any trailing bytes that are not yet
    a whole line. A line is only ever handed out once its newline has
    arrived, because the controller's writer thread drains to the kernel on
    a timer and the panel routinely reads a file mid-write; half a CSV row
    parsed as a whole one would be silent corruption.

    Genuinely stateful, hence a class. It does nothing else.
    """

    def __init__(self, path: Optional[Path] = None, at_end: bool = False) -> None:
        self.path: Optional[Path] = None
        # True when the last poll found the file shorter than it had already
        # read and started again from the top. The caller has to know,
        # because whatever it learned from the old contents — a CSV header,
        # most of all — no longer describes what it is now reading.
        self.rewound = False
        self._offset = 0
        self._partial = ""
        self._discard_until_newline = False
        if path is not None:
            self.open(path, at_end=at_end)

    def open(self, path: Path, at_end: bool = False) -> None:
        """Follow a different file from its beginning or its current end."""
        self.path = Path(path)
        self.rewound = False
        self._offset = 0
        self._partial = ""
        self._discard_until_newline = False
        if at_end:
            try:
                size = self.path.stat().st_size
                self._offset = size
                if size:
                    with open(self.path, "rb") as handle:
                        handle.seek(size - 1)
                        self._discard_until_newline = handle.read(1) != b"\n"
            except OSError:
                pass

    def poll(self) -> list[str]:
        """Complete lines appended since the last call, without newlines.

        Returns [] when the file does not exist yet: the controller creates
        its CSV after takeover, so "not there" is a normal early state and
        not an error. If the file has shrunk it is a different file under
        the same name, so reading restarts from the top.
        """
        self.rewound = False
        if self.path is None:
            return []
        try:
            size = self.path.stat().st_size
        except OSError:
            return []
        if size < self._offset:
            self.rewound = True
            self._offset = 0
            self._partial = ""
            self._discard_until_newline = False
        if size == self._offset:
            return []
        try:
            with open(self.path, "rb") as handle:
                handle.seek(self._offset)
                chunk = handle.read()
        except OSError:
            return []
        self._offset += len(chunk)
        text = self._partial + chunk.decode("utf-8", errors="replace")
        if self._discard_until_newline:
            newline = text.find("\n")
            if newline == -1:
                return []
            text = text[newline + 1:]
            self._discard_until_newline = False
        parts = text.split("\n")
        self._partial = parts.pop()
        return [part.rstrip("\r") for part in parts]


class CsvTail:
    """A LineTail that knows the controller's CSV shape.

    Carries the header it has seen, so rows are parsed by column name, and
    the preamble, which arrives both before the header and (the exit
    trailer) after the last data row.
    """

    def __init__(self, path: Optional[Path] = None) -> None:
        self.path: Optional[Path] = None
        self.header: Optional[list[str]] = None
        self.preamble: dict[str, str] = {}
        self.rows_seen = 0
        self._lines = LineTail()
        if path is not None:
            self.open(path)

    def open(self, path: Path, at_end: bool = False) -> None:
        """Start reading a file, keeping only its metadata when at its end."""
        self.path = Path(path)
        self.header = None
        self.preamble = {}
        self.rows_seen = 0
        if at_end:
            try:
                with open(self.path, "r", encoding="utf-8", errors="replace") as handle:
                    for line in handle:
                        if line.startswith("#"):
                            self.preamble.update(parse_preamble([line]))
                            continue
                        if line.strip():
                            self.header = [name.strip() for name in line.split(",")]
                            break
            except OSError:
                pass
        self._lines = LineTail(self.path, at_end=at_end)

    def poll(self) -> list[dict]:
        """Rows appended since the last call, parsed by column name.

        Values become floats; 'nan' and anything non-finite become None,
        and anything that is not a number at all is kept as its text. A row
        with fewer fields than the header keeps the columns it does have;
        extra fields are dropped. Neither case raises, because a format
        change should cost the panel a value, not the run screen.
        """
        rows: list[dict] = []
        lines = self._lines.poll()
        if self._lines.rewound:
            # A shorter file is a different file, so the header that is about
            # to arrive is the one that describes it. Keeping the old one
            # would silently map every value to the wrong column name.
            self.header = None
            self.preamble = {}
            self.rows_seen = 0
        for line in lines:
            if not line:
                continue
            if line.startswith("#"):
                self.preamble.update(parse_preamble([line]))
                continue
            fields = line.split(",")
            if self.header is None:
                self.header = [name.strip() for name in fields]
                continue
            rows.append({
                name: _parse_value(value)
                for name, value in zip(self.header, fields)
            })
            self.rows_seen += 1
        return rows


def _parse_value(text: str) -> object:
    """One CSV field: a finite float, None, or the text itself.

    'nan' must not survive as a number. json.dumps writes NaN and Infinity,
    which JSON.parse in the browser rejects outright, and a silent 0.0
    would read as perfect tracking. None is the only honest answer.
    """
    value = text.strip()
    if not value:
        return None
    try:
        number = float(value)
    except ValueError:
        return value
    return number if math.isfinite(number) else None


def derive(row: dict, prev_row: Optional[dict] = None) -> dict:
    """The quantities the UI needs that the CSV deliberately does not store.

    Hardware.h says the Cartesian error is not logged because it is exactly
    p_desired - p_current, so the panel does that subtraction here rather
    than asking for a column.

    pos_err_m         distance from the commanded tool point to the measured
                      one, metres, in the arm's base frame.
    follow_err_deg    the worst joint's |cmd - meas|, wrapped, which is the
                      same quantity the following-error guard tests.
    follow_err_joint  which joint that was, 1-7.
    gap_s             time between this row and the previous one the panel
                      actually saw. Not the loop's dt_s (that is its own
                      column): a gap much larger than dt_s means the panel
                      skipped rows, not that the loop was late.

    Any input that is missing or nan yields None for what depended on it.
    """
    out: dict = {
        "pos_err_m": None,
        "follow_err_deg": None,
        "follow_err_joint": None,
        "gap_s": None,
    }

    offsets = []
    for desired, measured in (("pd_x", "p_x"), ("pd_y", "p_y"), ("pd_z", "p_z")):
        a = _num(row, desired)
        b = _num(row, measured)
        if a is None or b is None:
            offsets = []
            break
        offsets.append(a - b)
    if offsets:
        out["pos_err_m"] = math.sqrt(sum(v * v for v in offsets))

    worst: Optional[float] = None
    worst_joint: Optional[int] = None
    for joint in range(1, 8):
        commanded = _num(row, f"cmd_j{joint}")
        measured = _num(row, f"meas_j{joint}")
        if commanded is None or measured is None:
            continue
        error = abs(wrap180(commanded - measured))
        if worst is None or error > worst:
            worst = error
            worst_joint = joint
    out["follow_err_deg"] = worst
    out["follow_err_joint"] = worst_joint

    if prev_row is not None:
        now = _num(row, "time_s")
        before = _num(prev_row, "time_s")
        if now is not None and before is not None:
            out["gap_s"] = now - before

    return out


class Aggregator:
    """The worst the run got between one frame and the next.

    The panel draws at about 20 Hz and the loop runs at 500, so 25 rows
    pass per frame and the newest one is not the interesting one. This
    keeps the maximum of every error, plus whether any single-cycle edge
    flag fired inside the window, since those are 1 for exactly one row and
    sampling would miss them entirely.

    take() answers the window and starts a new one.
    """

    def __init__(
        self,
        nominal_dt_s: float = DEFAULT_NOMINAL_DT_S,
        overrun_factor: float = DEFAULT_OVERRUN_FACTOR,
    ) -> None:
        self.nominal_dt_s = nominal_dt_s
        self.overrun_factor = overrun_factor
        self.reset()

    def reset(self) -> None:
        """Begin a new window."""
        self._max: dict[str, Optional[float]] = {
            "pos_err_m": None,
            "rot_error_rad": None,
            "follow_err_deg": None,
            "joint_follow_error_deg": None,
        }
        self._follow_err_joint: Optional[int] = None
        self._traj_start_error_deg: Optional[float] = None
        self._flags: dict[str, bool] = {name: False for name in EDGE_FLAGS}
        self._overruns = 0
        self._rows = 0

    def add(self, row: dict, derived: Optional[dict] = None) -> None:
        """Fold one parsed row, and its derived values, into the window."""
        if derived is None:
            derived = derive(row)
        self._rows += 1

        for key, source in (
            ("pos_err_m", derived),
            ("follow_err_deg", derived),
            ("rot_error_rad", row),
            ("joint_follow_error_deg", row),
        ):
            value = _num(source, key)
            if value is None:
                continue
            current = self._max[key]
            if current is None or value > current:
                self._max[key] = value
                if key == "follow_err_deg":
                    self._follow_err_joint = derived.get("follow_err_joint")

        for name in EDGE_FLAGS:
            value = _num(row, name)
            if value:
                self._flags[name] = True
                # A rejection row carries why it was rejected — how far the
                # plan's first point was from where the arm actually is.
                if name == "traj_rejected":
                    self._traj_start_error_deg = _num(row, "traj_start_error_deg")

        dt = _num(row, "dt_s")
        if dt is not None and dt > self.nominal_dt_s * self.overrun_factor:
            self._overruns += 1

    def snapshot(self) -> dict:
        """The window so far, without ending it."""
        return {
            "pos_err_m": self._max["pos_err_m"],
            "rot_error_rad": self._max["rot_error_rad"],
            "follow_err_deg": self._max["follow_err_deg"],
            "follow_err_joint": self._follow_err_joint,
            "joint_follow_error_deg": self._max["joint_follow_error_deg"],
            "traj_start_error_deg": self._traj_start_error_deg,
            "overruns": self._overruns,
            "rows": self._rows,
            **{name: self._flags[name] for name in EDGE_FLAGS},
        }

    def take(self) -> dict:
        """The window, and start the next one."""
        out = self.snapshot()
        self.reset()
        return out


def header_of(path: Path) -> Optional[list[str]]:
    """The CSV's column names, read without touching the rest of the file.

    Run logs reach gigabytes — 2.5 GB on this machine — so anything on a
    request path must read the front or the back of a file, never all of it.
    """
    try:
        with open(path, errors="replace") as handle:
            for line in handle:
                if line.startswith("#"):
                    continue
                return [name.strip() for name in line.rstrip("\n").split(",")]
    except OSError:
        return None
    return None


def has_data_rows(path: Path) -> bool:
    """Does this run hold at least one row a plan could start from?

    The question start_state_options actually needs, and it stops at the first
    data line instead of counting to three million. A session whose controller
    never reached its loop answers False, which is the case that matters.
    """
    try:
        with open(path, errors="replace") as handle:
            header_seen = False
            for line in handle:
                if line.startswith("#"):
                    continue
                if not header_seen:
                    header_seen = True
                    continue
                if line.strip():
                    return True
    except OSError:
        return False
    return False


def final_measured_deg(path: Path, joints: int = 7) -> Optional[list[float]]:
    """The last complete meas_j1..7 row of a run, in SIGNED degrees.

    The same configuration planner_bridge would take for itself. Read from the
    END of the file: seek back a window, keep whole lines, and walk them
    backwards until one has every joint finite. Reading forward would mean
    parsing gigabytes to reach the last line, which is how the panel's own
    start-state list managed to get itself killed by the kernel.

    WRAPPED, and that is not cosmetic. Kortex reports joint positions on
    [0, 360) while the planner's --start-deg, the joint limits and the
    firmware thresholds are all signed, so handing these values over raw says
    the arm is at +261 deg when it is physically at -98.8 — a different
    configuration. Measured 2026-08-11: the same left circle plans cleanly
    from the signed configuration and is rejected outright from the unwrapped
    one, with joints 59 deg outside their limits. State.h's WrappedJointError
    carries the same warning for the same reason.
    """
    header = header_of(path)
    if header is None:
        return None
    try:
        columns = [header.index(f"meas_j{j}") for j in range(1, joints + 1)]
    except ValueError:
        return None

    window = 1 << 16
    try:
        size = path.stat().st_size
        with open(path, "rb") as handle:
            while window <= (1 << 24):        # give up rather than crawl back
                start = max(0, size - window)
                handle.seek(start)
                chunk = handle.read().decode("utf-8", "replace")
                lines = chunk.split("\n")
                if start > 0:
                    lines.pop(0)              # a partial first line
                for line in reversed(lines):
                    if not line.strip() or line.startswith("#"):
                        continue
                    fields = line.split(",")
                    if len(fields) <= columns[-1]:
                        continue
                    try:
                        angles = [float(fields[i]) for i in columns]
                    except ValueError:
                        continue
                    if all(math.isfinite(a) for a in angles):
                        return [wrap180(a) for a in angles]
                if start == 0:
                    return None
                window <<= 2
    except OSError:
        return None
    return None


def count_data_rows(path: Path) -> int:
    """Data rows in a run CSV: not preamble, not the header. -1 if unreadable.

    O(file), so NOT for a request path — run logs reach gigabytes here. Use
    has_data_rows() when the question is only whether a plan could start from
    this run.

    This is the number that decides whether a run can seed a plan. A session
    whose controller never reached its loop — a SetServoingMode timeout, say —
    leaves a file with a full preamble, a header, and nothing else, and
    planner_bridge then refuses it with "start state unavailable: no complete
    data row". Counting here is what lets the panel explain that instead of
    relaying it.
    """
    rows = 0
    header_seen = False
    try:
        with open(path, errors="replace") as handle:
            for line in handle:
                if line.startswith("#"):
                    continue
                if not header_seen:
                    header_seen = True
                    continue
                if line.strip():
                    rows += 1
    except OSError:
        return -1
    return rows


def find_latest_csv(arm: str) -> Optional[Path]:
    """The newest runs/<date>/loop_log_<arm>*.csv, or None.

    Only one level down, so the hard-linked copy inside a session directory
    is not offered as a second, older-looking candidate for the same run.
    """
    if not paths.RUNS.is_dir():
        return None
    candidates = list(paths.RUNS.glob(f"*/loop_log_{arm}*.csv"))
    if not candidates:
        return None
    return max(candidates, key=lambda p: p.stat().st_mtime)


def tail_log_lines(path: Path, n: int = 40) -> list[str]:
    """The last n lines of a text file, cheaply.

    Used for the controller's stdout log, where PrintStopReport writes why
    the loop stopped — the one place that names following error, robot
    fault, stale feedback or overrun. Reads backwards in blocks so a long
    log costs nothing.
    """
    path = Path(path)
    try:
        size = path.stat().st_size
    except OSError:
        return []
    block = 8192
    data = b""
    try:
        with open(path, "rb") as handle:
            while size > 0 and data.count(b"\n") <= n:
                step = min(block, size)
                size -= step
                handle.seek(size)
                data = handle.read(step) + data
    except OSError:
        return []
    lines = data.decode("utf-8", errors="replace").split("\n")
    if lines and lines[-1] == "":
        lines.pop()
    return [line.rstrip("\r") for line in lines[-n:]]


class Source:
    """One arm's telemetry, live from the controller or replayed from a run.

    The same object does both, chosen by mode, because the difference is
    only where the rows come from and what paces them — everything after
    that (parse, derive, aggregate, frame) must be identical or replay
    would stop being a test of the live path.

    live    follow the newest loop_log_<arm>*.csv (or an explicit path) and
            emit a frame per poll, about 20 a second.
    replay  read a recorded CSV and pace rows by their own time_s, divided
            by speed. Frames are emitted once per frame_interval_s of LOG
            time, so a fast replay produces the same ~20 frames per logged
            second as a live run does per real second, and the aggregator
            still sees every row.

    Every frame reports server_age_s, the time since the newest row was
    read. It does not decide whether that is stale: the browser compares
    against its own clock, so a frozen server cannot hide behind frames it
    stopped sending.
    """

    def __init__(
        self,
        arm: str,
        path: Optional[Path] = None,
        mode: str = "live",
        speed: float = 1.0,
        poll_interval_s: float = 0.05,
        frame_interval_s: float = 0.05,
    ) -> None:
        if mode not in ("live", "replay"):
            raise ValueError(f"mode must be 'live' or 'replay', not {mode!r}")
        if speed <= 0.0:
            raise ValueError("speed must be positive")
        self.arm = arm
        self.mode = mode
        self.speed = speed
        self.poll_interval_s = poll_interval_s
        self.frame_interval_s = frame_interval_s
        self.path = Path(path) if path is not None else None
        self.tail = CsvTail()
        self.aggregator = Aggregator()
        # Rows this source has actually pushed into a frame. Not the same as
        # the tailer's count in replay, where the whole file is parsed at
        # once but delivered over the length of the recording.
        self.rows_seen = 0
        self._stopped = False
        self._prev_row: Optional[dict] = None
        self._last_read: Optional[float] = None  # monotonic clock

    def stop(self) -> None:
        """Ask frames() to return. Safe to call from another thread."""
        self._stopped = True

    @property
    def preamble(self) -> dict[str, str]:
        """The run's own compiled settings, once the header has been read."""
        return self.tail.preamble

    def frames(self) -> Iterator[tuple[str, dict]]:
        """Yield ("telemetry"|"heartbeat", payload) until stop() is called."""
        if self.mode == "replay":
            yield from self._replay_frames()
        else:
            yield from self._live_frames()

    # -- live ---------------------------------------------------------

    def _live_frames(self) -> Iterator[tuple[str, dict]]:
        while not self._stopped:
            started = time.monotonic()
            if self.tail.path is None:
                found = self.path or find_latest_csv(self.arm)
                if found is not None:
                    self.tail.open(found, at_end=True)
            if self.tail.path is None:
                yield self._heartbeat("no file yet")
            else:
                rows = self.tail.poll()
                self._sync_nominal_dt()
                if rows:
                    yield self._telemetry(rows)
                else:
                    # The file exists but nothing was appended. Either the
                    # controller has stopped or it never started writing;
                    # the browser is told, and decides nothing here.
                    reason = (
                        "file not growing"
                        if self.tail.path.exists()
                        else "no file yet"
                    )
                    yield self._heartbeat(reason)
                    self._follow_a_newer_run()
            self._sleep_until(started + self.poll_interval_s)

    def _follow_a_newer_run(self) -> None:
        """Move to a newer CSV once the current one has gone quiet.

        A panel left open across a session sees the old run's file stop
        growing and a new one appear beside it. Without this it would keep
        reporting a finished run as if it were the live one. Only checked
        when nothing was appended, so a live file is never abandoned
        mid-run, and never when a path was given explicitly — that caller
        asked for one particular file.
        """
        if self.path is not None:
            return
        newest = find_latest_csv(self.arm)
        if newest is not None and newest != self.tail.path:
            self.tail.open(newest, at_end=True)
            self._prev_row = None

    # -- replay -------------------------------------------------------

    def _replay_frames(self) -> Iterator[tuple[str, dict]]:
        source = self.path or find_latest_csv(self.arm)
        if source is None:
            while not self._stopped:
                yield self._heartbeat("no file yet")
                self._sleep_until(time.monotonic() + self.poll_interval_s)
            return

        self.tail.open(source)
        rows = self.tail.poll()
        self._sync_nominal_dt()

        first_log_t: Optional[float] = None
        wall_start = time.monotonic()
        last_frame_log_t: Optional[float] = None
        pending: list[dict] = []

        for index, row in enumerate(rows):
            if self._stopped:
                return
            log_t = _num(row, "time_s")
            if log_t is not None:
                if first_log_t is None:
                    first_log_t = log_t
                    last_frame_log_t = log_t
                due = wall_start + (log_t - first_log_t) / self.speed
                self._sleep_until(due)
                if self._stopped:
                    return
            pending.append(row)
            last_row = index == len(rows) - 1
            due_frame = (
                log_t is None
                or last_frame_log_t is None
                or (log_t - last_frame_log_t) >= self.frame_interval_s
            )
            if due_frame or last_row:
                if log_t is not None:
                    last_frame_log_t = log_t
                yield self._telemetry(pending)
                pending = []

        # End of the recording. Rows stop, frames do not: the browser's
        # staleness path is then exercised exactly as it would be by a
        # controller that exited.
        while not self._stopped:
            yield self._heartbeat("replay ended")
            self._sleep_until(time.monotonic() + self.poll_interval_s)

    # -- shared -------------------------------------------------------

    def _sync_nominal_dt(self) -> None:
        """Take the loop period from the run's own preamble once it is known."""
        text = self.tail.preamble.get("control_dt_s")
        if text is None:
            return
        try:
            dt = float(text)
        except ValueError:
            return
        if dt > 0.0:
            self.aggregator.nominal_dt_s = dt

    def _telemetry(self, rows: list[dict]) -> tuple[str, dict]:
        """One frame: the newest row, and the worst of every row behind it."""
        derived: dict = {}
        for row in rows:
            derived = derive(row, self._prev_row)
            self.aggregator.add(row, derived)
            self._prev_row = row
            self.rows_seen += 1
        self._last_read = time.monotonic()
        newest = dict(rows[-1])
        # The derived values ride inside the row because the browser wants
        # one flat record per sample, and none of their names collide with
        # a CSV column.
        newest.update(derived)
        return ("telemetry", {
            "arm": self.arm,
            "wall_ms": int(time.time() * 1000.0),
            "row": newest,
            "worst": self.aggregator.take(),
            "server_age_s": self._age_s(),
            "growing": True,
            "rows_seen": self.rows_seen,
        })

    def _heartbeat(self, reason: str) -> tuple[str, dict]:
        """A frame that says the file did not grow, and why it might not have."""
        return ("heartbeat", {
            "arm": self.arm,
            "wall_ms": int(time.time() * 1000.0),
            "server_age_s": self._age_s(),
            "growing": False,
            "reason": reason,
        })

    def _age_s(self) -> Optional[float]:
        """Seconds since the newest row was read; None if none ever was."""
        if self._last_read is None:
            return None
        return time.monotonic() - self._last_read

    def _sleep_until(self, deadline: float) -> None:
        """Sleep to a monotonic deadline, in slices so stop() stays prompt."""
        while not self._stopped:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return
            time.sleep(min(remaining, 0.1))
