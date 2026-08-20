"""Start, stop, status and re-attach for one supervised session.

This is the only module in the panel whose failure reaches a moving arm, so
it does as little as possible and reuses paths that already exist.

What it actually does: it runs `scripts/run_session.sh` exactly as a terminal
would, types GO for it, and later sends the same SIGINT that Ctrl-C sends.
It does not talk to the robot, does not decide whether motion is safe, and
adds no stop mechanism of its own. Every gate the script has is still the
script's; the panel mirrors the freshness gate only so that it can refuse
early with a readable reason instead of showing the operator a script that
exited.

Three rules that the code below keeps returning to.

Start is localhost-only. You must be at the workstation, next to the physical
e-stop, to begin motion. Stop works from any address, because a remote stop
is always safe and a remote start never is.

The panel never owns the run's life. The session is launched detached
(`start_new_session=True`), so a panel crash, a closed browser or a dropped
network leaves the arm doing what it was told rather than halting it abruptly
because of a cosmetic failure. Nothing here kills the session on panel exit.

The stop is not a controlled stop. SIGINT reaches the process group;
`run_session.sh`'s EXIT trap kills the controller; the controller's own
handler (`Main.cpp:588` sets the stop flag, `Runner.cpp:296` throws
`TakeoverStop{kUserStop}`, `servoing_guard.Restore()` hands the arm back)
does the rest. There is no velocity ramp anywhere on that path: commanding
ceases. Do not describe it, in code or on screen, as a ramped or controlled
stop.
"""

from __future__ import annotations

import ipaddress
import json
import math
import os
import signal
import subprocess
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Optional

from . import paths

# The arms run_session.sh accepts. "both" is one controller process driving
# two arms, not two sessions.
VALID_ARMS = ("right", "left", "both")

# How long start() waits for the script to announce its session directory
# before returning without one. The script does its freshness checks and the
# GO read first, so this is generous rather than tight; a session that has
# not said anything by then is reported as started but undescribed rather
# than being killed, because killing it is the one thing this module must
# never do casually.
SESSION_DIR_TIMEOUT_S = 20.0

# run_session.sh prints this early, on stdout, immediately after creating the
# directory (scripts/run_session.sh:103). Parsing that one line is how the
# panel learns where the session's artifacts are going.
SESSION_DIR_MARKER = "session artifacts:"


# The mount sources the controller accepts (MainArgs.h).
VALID_MOUNTS = ("fixed", "vicon")


def default_launch_command(arm: str, mount: str, planning: bool,
                           recording: bool,
                           fixed_pose: Optional[list[float]] = None,
                           planner: str = "current") -> list[str]:
    """The command a session is started with.

    Note what is absent: `--allow-stale`. The script offers it and the panel
    must never be able to pass it, so the flag is not built here and start()
    refuses on a stale binary instead.
    """
    command = [str(paths.RUN_SESSION_SH), "--arm", arm, "--mount", mount,
               "--plan", "on" if planning else "off",
               "--record", "on" if recording else "off"]
    # "current" is the no-flag form: the command for an ordinary session is
    # unchanged. "baseline" runs the frozen known-good 5abc1b2c planner —
    # run_session.sh resolves the binary path and refuses if it is missing.
    if planner != "current":
        command += ["--planner", planner]
    if fixed_pose is not None:
        command += ["--fixed-pose", " ".join(f"{v:.9g}" for v in fixed_pose)]
    return command


def default_freshness() -> dict:
    """The build module's freshness report.

    Imported inside the function rather than at module import, so that this
    safety-critical module can be loaded and tested without dragging in the
    build machinery.
    """
    from . import build

    return build.freshness()


# The three seams tests substitute. They are plain module-level functions,
# not an injected object: there is exactly one production implementation of
# each, and a test that replaced them through a constructor would be testing
# the constructor.
launch_command = default_launch_command
send_signal_to_group: Callable[[int, int], None] = os.killpg
freshness_check: Callable[[], dict] = default_freshness

# The Popen handle for a session THIS panel process launched, kept for two
# reasons that both matter. Reaping it (see status()) stops a finished child
# from lingering as a zombie, whose pid os.kill() would still answer for —
# the panel would go on reporting a session that ended. And holding it keeps
# our end of the child's stdin pipe open; see _launch() for why that end must
# not close.
_process: Optional[subprocess.Popen] = None

# Our copy of the write end of the child's stdin pipe.
_stdin_write_fd: Optional[int] = None


def is_loopback(client_address: Any) -> bool:
    """Is this request from the workstation itself?

    Accepts what an HTTP server hands over: either the ("host", port) tuple
    from `BaseHTTPRequestHandler.client_address` or a bare host string. An
    address that cannot be parsed is not loopback — an unrecognised caller is
    treated as remote, which is the direction that refuses to start motion.
    """
    if isinstance(client_address, (tuple, list)):
        host = client_address[0] if client_address else None
    else:
        host = client_address
    if not isinstance(host, str) or not host:
        return False
    host = host.split("%", 1)[0]  # drop an IPv6 zone index, e.g. fe80::1%eth0
    if host.startswith("[") and host.endswith("]"):
        host = host[1:-1]
    try:
        address = ipaddress.ip_address(host)
    except ValueError:
        return host == "localhost"
    # A v4 address arriving over a dual-stack socket looks like ::ffff:127.0.0.1,
    # which is loopback in fact but not by IPv6 rules, so unwrap it first.
    if isinstance(address, ipaddress.IPv6Address) and address.ipv4_mapped is not None:
        address = address.ipv4_mapped
    return address.is_loopback


def required_stale_reasons(report: Any, planning: bool) -> list[str]:
    """The freshness reasons that block THE SELECTED session.

    The rule: a component may block startup only if the session actually
    requires its output. The controller binary always does. The generated DH
    tables feed GPMP2 only, so they block only when planning is on. The
    planner_bridge binary is a preview tool the running controller never
    executes, so it never blocks (it stays visible on the CONFIG page).

    A report that is not readable, or one that says stale without saying
    why, refuses everything — this gate must fail closed on a reshaped
    report, exactly like stale_reasons below.
    """
    if not isinstance(report, dict):
        return [f"freshness report was not readable: {report!r}"]

    def entry_reasons(entry: Any, name: str) -> list[str]:
        if not isinstance(entry, dict):
            return [f"freshness report has no readable {name} entry"]
        if entry.get("stale") is not True:
            return []
        return [str(r) for r in (entry.get("reasons") or [])] or [f"{name} is stale"]

    reasons = entry_reasons(report.get("controller"), "controller")
    if planning:
        dh = report.get("dh")
        if isinstance(dh, dict):
            for arm_name, entry in dh.items():
                reasons.extend(entry_reasons(entry, f"dh[{arm_name}]"))
        else:
            reasons.append("freshness report has no readable dh entry")
    return reasons


def stale_reasons(report: Any) -> list[str]:
    """The reasons a freshness report gives for refusing to start.

    `build.freshness()` puts a written sentence for every stale thing in
    `reasons`, and those sentences are what the operator should read, so they
    are used as they are. The fallback below exists because this gate must not
    silently pass a session through if that report is ever reshaped: if there
    is no `reasons` list, anything that still says it is stale — the top-level
    flag, or an entry describing one target — refuses the start on its own.
    """
    if not isinstance(report, dict):
        return [f"freshness report was not readable: {report!r}"]

    reasons = [str(reason) for reason in (report.get("reasons") or [])]
    if reasons:
        return reasons

    for name, entry in report.items():
        if name in ("stale", "reasons"):
            continue
        if isinstance(entry, dict) and entry.get("stale") is True:
            reasons.append(f"{name} is stale")
    if report.get("stale") is True and not reasons:
        reasons.append("the build is stale")
    return reasons


def stop_command_for(pgid: int) -> str:
    """The paste-into-a-terminal equivalent of the panel's stop button.

    The leading minus makes this the process GROUP, which is both what stop()
    signals and what Ctrl-C in the session's terminal would have done. If the
    script has somehow gone without taking the controller with it, the group
    form still reaches the controller; signalling the script's pid alone
    would not.
    """
    return f"kill -INT -{pgid}"


def _read_state() -> Optional[dict]:
    try:
        return json.loads(paths.SESSION_STATE.read_text())
    except (OSError, ValueError):
        return None


def _write_state(state: dict) -> None:
    paths.SESSION_STATE.parent.mkdir(parents=True, exist_ok=True)
    paths.SESSION_STATE.write_text(json.dumps(state, indent=2))


def _clear_state() -> None:
    try:
        paths.SESSION_STATE.unlink()
    except FileNotFoundError:
        pass


def _forget_process() -> None:
    """Drop our handle on a finished session and close our stdin end.

    Called only once the child is known to be gone. While it is alive the fd
    stays open; closing it early would send EOF to the script's final bare
    `read`, which is how the script decides to stop the controller.
    """
    global _process, _stdin_write_fd
    _process = None
    if _stdin_write_fd is not None:
        try:
            os.close(_stdin_write_fd)
        except OSError:
            pass
        _stdin_write_fd = None


def _pgid_alive(pgid: Optional[int]) -> bool:
    """Is the session leader still there?

    `start_new_session=True` makes the child a session and group leader, so
    its pid and pgid are the same number and signal 0 to that pid asks about
    run_session.sh itself. A pid we are not allowed to signal is reported as
    alive: that only happens if the number now belongs to someone else, and
    telling the operator a session has ended when it may not have is the
    wrong way to be wrong.
    """
    if not pgid:
        return False
    try:
        os.kill(pgid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def controller_alive(pgid: Optional[int]) -> bool:
    """Is a controller process actually running in this session's group?

    Not the same question as _pgid_alive, and the difference matters on screen.
    run_session.sh outlives its controller: after the loop exits it sits at
    `read` waiting for the Enter that ends the session, so the group stays
    alive with nothing commanding an arm. A panel that called that "running"
    said DRIVING BOTH ARMS over a workstation where the controller had already
    stopped — observed 2026-08-11.

    So: the group being alive means stop() still has something to signal;
    a controller being alive means an arm is actually being commanded.

    Reads /proc, which is Linux-only. The panel serves the workstation, and
    everything it talks to is Linux-only for the same reason.
    """
    if not pgid:
        return False
    binary = str(paths.CONTROLLER_BIN)
    try:
        entries = os.listdir("/proc")
    except OSError:
        return False
    for entry in entries:
        if not entry.isdigit():
            continue
        try:
            if os.getpgid(int(entry)) != pgid:
                continue
            with open(f"/proc/{entry}/cmdline", "rb") as handle:
                argv = handle.read().split(b"\0")
        except (OSError, ProcessLookupError, PermissionError):
            continue
        if argv and argv[0].decode("utf-8", "replace") == binary:
            return True
    return False


def _no_session() -> dict:
    return {
        "running": False,
        "commanding": False,
        "note": "",
        "pid": None,
        "pgid": None,
        "arm": None,
        "started": None,
        "session_dir": None,
        "log_path": None,
        "attached": False,
        "stop_command": None,
    }


def _staging_log_path() -> Path:
    """Where the child's output goes until the session directory is known.

    The script creates that directory itself, and only after the GO gate, so
    there is nowhere else to point the child's stdout at the moment it is
    launched. It goes under runs/ rather than /tmp because the session
    directory is under runs/ too: the log is later renamed into place, and a
    rename only works within one filesystem.
    """
    return paths.RUNS / "panel_session.log"


def _find_session_dir(log_path: Path) -> Optional[str]:
    try:
        text = log_path.read_text(errors="replace")
    except OSError:
        return None
    for line in text.splitlines():
        if SESSION_DIR_MARKER in line:
            return line.split(SESSION_DIR_MARKER, 1)[1].strip() or None
    return None


def _tail(log_path: Path, lines: int = 12) -> str:
    try:
        return "\n".join(log_path.read_text(errors="replace").splitlines()[-lines:])
    except OSError:
        return ""


def _launch(arm: str, log_path: Path, mount: str = "vicon",
            planning: bool = True, recording: bool = True,
            fixed_pose: Optional[list] = None,
            planner: str = "current") -> subprocess.Popen:
    """Start run_session.sh detached, with GO already waiting on its stdin.

    The stdin pipe is built by hand rather than with `stdin=subprocess.PIPE`
    because the child is given the write end as well (`pass_fds`). The script
    reads GO for its first prompt and then blocks on a bare `read` waiting for
    Enter before it stops the controller; if the only writer were this panel,
    the panel dying would close the pipe, that `read` would see EOF, and the
    arm would be stopped by the panel's crash. With the child holding a writer
    of its own the pipe never reaches EOF, the `read` blocks for ever, and the
    session ends only when it is signalled.
    """
    global _stdin_write_fd

    read_fd, write_fd = os.pipe()
    try:
        os.write(write_fd, b"GO\n")
        with open(log_path, "wb") as log:
            process = subprocess.Popen(
                launch_command(arm, mount, planning, recording, fixed_pose,
                               planner),
                cwd=str(paths.REPO),
                stdin=read_fd,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                pass_fds=(write_fd,),
            )
    except BaseException:
        os.close(write_fd)
        raise
    finally:
        os.close(read_fd)
    _stdin_write_fd = write_fd
    return process


VALID_PLANNERS = ("current", "baseline")


def start(arm: str, confirm: str, client_address: Any,
          is_local: Optional[bool] = None, mount: str = "vicon",
          planning: bool = True, recording: bool = True,
          fixed_pose: Optional[list] = None,
          planner: str = "current") -> dict:
    """Begin a supervised session, or say why not.

    `confirm` must be the literal "GO". The gate is not weakened by moving it
    into the browser, only relocated: run_session.sh:87 asks a human to type
    GO, and the panel may only answer on a human's behalf when that human
    typed the same word.

    `is_local` is the server's own reading of the same address. It is a second
    opinion, not a substitute: the start needs both it and the check here to
    say loopback, so neither one on its own is what stands between a remote
    browser and a moving arm.
    """
    global _process

    if arm not in VALID_ARMS:
        return {"ok": False, "error": f"arm must be one of {', '.join(VALID_ARMS)} (got {arm!r})"}

    if mount not in VALID_MOUNTS:
        return {"ok": False,
                "error": f"mount must be one of {', '.join(VALID_MOUNTS)} (got {mount!r})"}
    planning = bool(planning)
    recording = bool(recording)
    if planner not in VALID_PLANNERS:
        return {"ok": False,
                "error": f"planner must be one of {', '.join(VALID_PLANNERS)} (got {planner!r})"}
    if planner == "baseline" and not planning:
        return {"ok": False,
                "error": "planner=baseline is meaningless with planning off"}
    if fixed_pose is not None:
        if mount != "fixed":
            return {"ok": False,
                    "error": "a fixed mount pose is only meaningful with mount=fixed"}
        try:
            fixed_pose = [float(v) for v in fixed_pose]
        except (TypeError, ValueError):
            return {"ok": False, "error": "fixed_pose must be 7 numbers (x y z qx qy qz qw)"}
        if len(fixed_pose) != 7 or not all(map(math.isfinite, fixed_pose)):
            return {"ok": False, "error": "fixed_pose must be 7 finite numbers (x y z qx qy qz qw)"}
        # The identity default needs no flag at all; sending it explicitly is
        # the same session, so normalise to the no-flag form.
        if fixed_pose == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]:
            fixed_pose = None

    if is_local is False or not is_loopback(client_address):
        return {
            "ok": False,
            "error": "start is localhost-only: you must be at the workstation, "
                     "beside the physical e-stop, to begin motion. Stop works "
                     "from here.",
        }

    if confirm != "GO":
        return {"ok": False, "error": 'type GO to confirm the session checklist'}

    current = status()
    if current["running"]:
        return {
            "ok": False,
            "error": f"a session is already running (pid {current['pid']}, arm {current['arm']})",
            "status": current,
        }

    try:
        reasons = required_stale_reasons(freshness_check(), planning)
    except Exception as error:  # a freshness check that cannot answer is not an answer
        return {"ok": False, "error": f"could not check binary freshness: {error}"}
    if reasons:
        return {
            "ok": False,
            "error": "the built binaries are older than their sources; build first",
            "reasons": reasons,
        }

    log_path = _staging_log_path()
    log_path.parent.mkdir(parents=True, exist_ok=True)  # a fresh checkout has no runs/
    started = datetime.now().astimezone().isoformat(timespec="seconds")
    process = _launch(arm, log_path, mount, planning, recording, fixed_pose,
                      planner)
    _process = process

    # The script's own gates run after launch, so a refusal (missing binary,
    # a stale generated DH table, a bad arm) arrives as an early exit with the
    # reason on stdout. Wait for whichever comes first: the session directory
    # line, or the child giving up.
    session_dir: Optional[str] = None
    deadline = time.monotonic() + SESSION_DIR_TIMEOUT_S
    while time.monotonic() < deadline:
        session_dir = _find_session_dir(log_path)
        if session_dir:
            break
        if process.poll() is not None:
            break
        time.sleep(0.05)
    if session_dir is None:
        session_dir = _find_session_dir(log_path)

    if session_dir is None and process.poll() is not None:
        _forget_process()
        return {
            "ok": False,
            "error": f"run_session.sh exited {process.returncode} before starting a session",
            "output": _tail(log_path),
        }

    # Move the log into the session directory now that there is one. A rename
    # does not disturb the child: it is still writing to the same open file,
    # which has simply acquired a better name.
    if session_dir:
        moved = Path(session_dir) / "panel_session.log"
        try:
            os.replace(log_path, moved)
            log_path = moved
        except OSError:
            pass

    # pid and pgid are the same number by construction (start_new_session
    # calls setsid in the child), and taking it from the Popen handle rather
    # than asking os.getpgid avoids a race with a child that exits at once.
    state = {
        "pid": process.pid,
        "pgid": process.pid,
        "arm": arm,
        "mount": mount,
        "planning": planning,
        "recording": recording,
        "fixed_pose": fixed_pose,
        "started": started,
        "session_dir": session_dir,
        "log_path": str(log_path),
    }
    _write_state(state)

    return {
        "ok": True,
        "pid": process.pid,
        "pgid": process.pid,
        "arm": arm,
        "mount": mount,
        "planning": planning,
        "recording": recording,
        "started": started,
        "session_dir": session_dir,
        "log_path": str(log_path),
        "stop_command": stop_command_for(process.pid),
    }


def stop() -> dict:
    """Signal the session's process group with SIGINT.

    Nothing escalates. There is deliberately no SIGKILL after a timeout: the
    controller's SIGINT path is what hands low-level servoing back to the arm,
    and a killed controller would leave it in servoing mode with nobody
    commanding it. If SIGINT does not take, the answer is the physical e-stop,
    not a bigger signal from a browser.
    """
    current = status()
    if not current["running"]:
        return {"ok": True, "stopped": False, "message": "no session is running"}

    pgid = current["pgid"]
    try:
        send_signal_to_group(pgid, signal.SIGINT)
    except ProcessLookupError:
        _clear_state()
        _forget_process()
        return {"ok": True, "stopped": False, "message": "the session had already ended"}
    except OSError as error:
        return {"ok": False, "error": f"could not signal pgid {pgid}: {error}"}

    return {
        "ok": True,
        "stopped": True,
        "pid": current["pid"],
        "pgid": pgid,
        "signal": "SIGINT",
        "message": "SIGINT sent to the session's process group; commanding ceases "
                   "(there is no ramp) and the arm is released",
    }


def status() -> dict:
    """What session, if any, is running.

    Re-reads the state file every call, so a panel restarted while the arm was
    moving finds the session and re-attaches to it instead of orphaning it.
    `attached` is True exactly when the running session was launched by some
    earlier panel process or terminal rather than by this one.
    """
    global _process

    if _process is not None and _process.poll() is not None:
        _forget_process()  # reaped, so its pid cannot answer signal 0 any more
    launched_here = _process is not None

    state = _read_state()
    if state is None:
        return _no_session()

    pgid = state.get("pgid")
    if not (launched_here or _pgid_alive(pgid)):
        _clear_state()
        return _no_session()

    # Two different truths, and conflating them made the panel claim an arm was
    # being driven after its controller had stopped. `running` means the
    # session still exists and stop() has something to signal; `commanding`
    # means a controller process is actually there, driving an arm.
    commanding = controller_alive(pgid)
    return {
        "running": True,
        "commanding": commanding,
        "pid": state.get("pid"),
        "pgid": pgid,
        "arm": state.get("arm"),
        "mount": state.get("mount"),
        "planning": state.get("planning"),
        "recording": state.get("recording"),
        "started": state.get("started"),
        "session_dir": state.get("session_dir"),
        "log_path": state.get("log_path"),
        "attached": not launched_here,
        "stop_command": stop_command_for(pgid) if pgid else None,
        "note": "" if commanding else
                "the controller has exited; the session script is still "
                "waiting. Stop clears it.",
    }
