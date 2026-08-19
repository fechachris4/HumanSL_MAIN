"""Is the built thing older than what it was built from, and rebuild it.

Two build products remain visible: the controller (which contains the
in-process planner) and the standalone planner preview executable.

The freshness half mirrors the `fresh_or_die` gate at the top of
`planner_bridge/scripts/run_session.sh`: the controller binary against
`control/`, `runtime/`, `planner_bridge/src`, and
`planning/optimisation`, the standalone preview binary against
the planner sources, and the build-generated DH table for each arm against
the URDF it is generated from. Mirroring means the panel can
show the answer BEFORE you commit to a run, which is the whole point — a
stale binary discovered by the session script is discovered after you have
already decided to move an arm. It does not replace that gate: run_session.sh
still makes the real decision, and it is the one that can be overridden with
--allow-stale.

The build half runs cmake in a background thread so its output can be
streamed to the browser line by line rather than arriving in one lump when
the compile finishes.
"""

from __future__ import annotations

import os
import subprocess
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any, Callable, Iterable, Optional

from . import paths

# What a binary is actually built from. Nothing else under these directories
# is compiled into it, so nothing else can make it wrong: a test fixture, a
# README or an editor's swap file is not a reason to refuse a session. On
# 2026-08-19 counting every file did exactly that — a rewritten fixture CSV
# under control/tests/ made the panel refuse to start while run_session.sh's
# gate, which has always compared only these two suffixes, passed.
_SOURCE_SUFFIXES = (".cpp", ".h")

# Build trees sit beside the sources they are built from, and cmake writes a
# CMakeCXXCompilerId.cpp into every one of them at configure time. It is a
# probe, not a source of anything. .git is skipped because walking it is slow
# and it holds no source either.
_IGNORED_DIRECTORY_NAMES = ("build", ".git")

# How many lines of build output are kept for a browser that connects late.
# A full rebuild prints thousands; the tail is what tells you whether it
# worked and, if it did not, which file failed.
_MAX_BUILD_LINES = 400


def _newest_file(directories: Iterable[Path]) -> tuple[Optional[Path], Optional[float]]:
    """The most recently modified C++ source anywhere under these directories.

    The same set of files `fresh_or_die` in run_session.sh compares, so the
    panel's answer and the gate's answer are one answer. A file that is not
    compiled is not consulted by either.
    """
    newest_path: Optional[Path] = None
    newest_mtime: Optional[float] = None
    for directory in directories:
        if not directory.is_dir():
            continue
        for root, subdirectories, filenames in os.walk(directory):
            subdirectories[:] = [
                name for name in subdirectories
                if name not in _IGNORED_DIRECTORY_NAMES
            ]
            for filename in filenames:
                if not filename.endswith(_SOURCE_SUFFIXES):
                    continue
                candidate = Path(root) / filename
                try:
                    mtime = candidate.stat().st_mtime
                except OSError:
                    continue  # deleted between the walk and the stat
                if newest_mtime is None or mtime > newest_mtime:
                    newest_path, newest_mtime = candidate, mtime
    return newest_path, newest_mtime


def _display(path: Path, base: Path) -> str:
    """A path short enough to read in a sentence, relative to `base` if it can."""
    for root in (base, paths.REPO):
        try:
            return str(path.relative_to(root))
        except ValueError:
            continue
    return str(path)


def _binary_freshness(
    binary: Path, sources: Iterable[Path], base: Path, label: str
) -> tuple[dict[str, Any], list[str]]:
    """One binary against the directories it is compiled from.

    Staleness is `source newer than binary`, strictly — the same comparison
    bash's `-nt` makes in fresh_or_die, so equal timestamps count as fresh
    there and here.
    """
    reasons: list[str] = []
    runnable = binary.is_file() and os.access(binary, os.X_OK)
    binary_mtime = binary.stat().st_mtime if binary.is_file() else 0.0
    newest_path, newest_mtime = _newest_file(sources)

    stale = False
    if not binary.is_file():
        stale = True
        reasons.append(f"{label} has not been built ({_display(binary, base)})")
    elif not runnable:
        stale = True
        reasons.append(f"{label} is not executable ({_display(binary, base)})")
    elif newest_mtime is not None and newest_mtime > binary_mtime:
        stale = True
        assert newest_path is not None
        reasons.append(f"{label} is older than {_display(newest_path, base)}")

    return (
        {
            "binary": str(binary),
            "exists": runnable,
            "stale": stale,
            "newest_source": str(newest_path) if newest_path is not None else None,
            "binary_mtime": binary_mtime,
            "source_mtime": newest_mtime,
            "reasons": list(reasons),
        },
        reasons,
    )


def _dh_freshness(arm: str) -> tuple[dict[str, Any], list[str]]:
    """One arm's build-generated DH table against the URDF it comes from.

    The generator runs as part of the controller's nested planner build, so an edited URDF
    with no rebuild leaves a table describing the previous robot. That is the
    quietest possible wrong answer: everything still runs, and the planner
    aims at the wrong place.
    """
    generated = paths.DH_YAML[arm]
    urdf = paths.URDF
    reasons: list[str] = []

    generated_mtime = generated.stat().st_mtime if generated.is_file() else 0.0
    urdf_mtime = urdf.stat().st_mtime if urdf.is_file() else None

    stale = False
    if not generated.is_file():
        stale = True
        reasons.append(
            f"the {arm} arm's {generated.name} has not been generated "
            "— build controller"
        )
    elif urdf_mtime is not None and urdf_mtime > generated_mtime:
        stale = True
        reasons.append(
            f"the {arm} arm's {generated.name} is older than the URDF "
            f"({_display(urdf, paths.REPO)})"
        )

    return (
        {
            "generated": str(generated),
            "exists": generated.is_file(),
            "stale": stale,
            "urdf": str(urdf),
            "generated_mtime": generated_mtime,
            "urdf_mtime": urdf_mtime,
            "reasons": list(reasons),
        },
        reasons,
    )


def freshness() -> dict[str, Any]:
    """What is out of date, and a sentence for each thing that is.

    `stale` is true if anything at all is stale or missing, so the panel has
    one flag to gate the start button on; `reasons` is what it shows the
    operator instead of just refusing.
    """
    controller, controller_reasons = _binary_freshness(
        paths.CONTROLLER_BIN,
        [paths.CONTROL, paths.RUNTIME, paths.BRIDGE_SRC,
         paths.BRIDGE_OPTIMISATION],
        paths.RUNTIME,
        "controller",
    )
    bridge, bridge_reasons = _binary_freshness(
        paths.BRIDGE_BIN,
        [paths.BRIDGE_SRC, paths.BRIDGE_OPTIMISATION],
        paths.PLANNING,
        "planner_bridge",
    )

    dh: dict[str, Any] = {}
    dh_reasons: list[str] = []
    for arm in paths.ARMS:
        dh[arm], arm_reasons = _dh_freshness(arm)
        dh_reasons.extend(arm_reasons)

    reasons = controller_reasons + bridge_reasons + dh_reasons
    return {
        "controller": controller,
        "bridge": bridge,
        "dh": dh,
        "stale": bool(reasons),
        "reasons": reasons,
    }


# ---------------------------------------------------------------------------
# Running cmake
# ---------------------------------------------------------------------------

# One build at a time, and the state it publishes. Module-level rather than an
# object because there is exactly one panel process and exactly one build in
# it; a class here would only be a place to keep these five names.
_state_lock = threading.Lock()
_running = False
_ok: Optional[bool] = None
_lines: deque[str] = deque(maxlen=_MAX_BUILD_LINES)
_target: Optional[str] = None
_started_at = 0.0
_finished = threading.Event()
_finished.set()
_listener: Optional[Callable[[Optional[str]], None]] = None


def build_dir(target: str) -> Path:
    """The cmake build directory for a target name.

    Read through `paths` on every call rather than captured in a table at
    import time, so a test can point the whole module at a temporary tree.
    """
    if target == "controller":
        return paths.CONTROLLER_BUILD
    if target == "bridge":
        return paths.BRIDGE_BUILD
    raise ValueError(f"unknown build target: {target!r}")


def build_command(target: str) -> list[str]:
    """The command a build runs. This is also the seam tests replace.

    Nothing here parses cmake's output or passes it a generator: whatever the
    build directory was configured with is what runs, so the panel's build and
    a hand-typed one are the same build.
    """
    jobs = os.cpu_count() or 1
    return ["cmake", "--build", str(build_dir(target)), f"-j{jobs}"]


def set_build_listener(callback: Optional[Callable[[Optional[str]], None]]) -> None:
    """Register who gets each line as it arrives; called with None at the end.

    The server uses this to push lines onto the SSE bus. Pass None to detach.
    """
    global _listener
    with _state_lock:
        _listener = callback


def _notify(line: Optional[str]) -> None:
    """Hand one line (or the end-of-build None) to the listener.

    Called with no lock held: the listener writes to a browser, and a slow or
    dead browser must not be able to block a build or freeze build_status().
    A listener that raises is ignored for the same reason.
    """
    with _state_lock:
        callback = _listener
    if callback is None:
        return
    try:
        callback(line)
    except Exception:
        pass


def _record(line: str) -> None:
    with _state_lock:
        _lines.append(line)
    _notify(line)


def _run_build(command: list[str]) -> None:
    global _running, _ok
    ok = False
    try:
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,  # one interleaved stream, in cmake's order
            text=True,
            bufsize=1,
        )
    except OSError as error:
        _record(f"could not start the build: {error}")
    else:
        # The context manager closes the pipe when the build ends, so a panel
        # that runs for days does not leak a file descriptor per build.
        with process:
            if process.stdout is not None:
                for line in process.stdout:
                    _record(line.rstrip("\n"))
            ok = process.wait() == 0
    finally:
        with _state_lock:
            _running = False
            _ok = ok
            _finished.set()
        _notify(None)


def start_build(target: str) -> tuple[bool, str]:
    """Start a build in the background. Returns (started, message).

    A second start while one is running is an error rather than a queue: two
    builds of the same tree would fight over the same object files, and a
    queue would mean a click's effect arrives minutes later, after you have
    changed a knob again.
    """
    global _running, _ok, _target, _started_at
    if target not in ("controller", "bridge"):
        return False, f"unknown build target: {target!r} (controller or bridge)"

    with _state_lock:
        if _running:
            elapsed = time.monotonic() - _started_at
            return False, (
                f"a {_target} build is already running ({elapsed:.0f} s so far)"
            )
        directory = build_dir(target)
        if not directory.is_dir():
            return False, (
                f"no build directory at {directory} — configure it with cmake first"
            )
        command = build_command(target)
        _running = True
        _ok = None
        _target = target
        _started_at = time.monotonic()
        _lines.clear()
        _finished.clear()

    threading.Thread(
        target=_run_build,
        args=(command,),
        name=f"panel-build-{target}",
        daemon=True,
    ).start()
    return True, " ".join(command)


def build_status() -> dict[str, Any]:
    """The current build's state. `ok` is None until a build has finished."""
    with _state_lock:
        return {
            "running": _running,
            "ok": _ok,
            "lines": list(_lines),
            "target": _target,
        }


def wait_for_build(timeout_s: Optional[float] = None) -> bool:
    """Block until the running build finishes. True if it has, False on timeout.

    Nothing in the server calls this — it is how a test waits for the thread
    without sleeping and hoping.
    """
    return _finished.wait(timeout_s)
