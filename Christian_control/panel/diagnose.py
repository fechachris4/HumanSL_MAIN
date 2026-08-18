"""One plain-text report answering "why is it not doing what I expect".

Written after an evening spent screenshotting the panel to ask what was wrong,
when the answer — a SetServoingMode timeout — was already in a log the panel
had captured and never showed. Everything that could explain a refusal to
start, a stale binary, an empty run log or a failed solve is gathered here in
one block that can be read in a terminal or pasted into a conversation.

Three consumers, one report: `control_panel.py --check` prints it, the server
serves it at /api/diagnose, and the panel's COPY DIAGNOSTICS button fetches it
and adds what only the browser knows.

It reports; it never fixes anything and never touches the arm.
"""

from __future__ import annotations

import platform
import socket
import sys
from pathlib import Path

from . import build, dh, paths, plan, runs, session, telemetry

# How many trailing controller-log lines to carry. Enough to hold the takeover
# sequence and whatever ended it, without turning a paste into a scroll.
LOG_LINES = 40

# Lines worth pulling out of a controller log. "loop stopped" opens every
# PrintStopReport sentence (Safety.cpp), and the Kortex transport failures and
# internal errors all announce themselves with one of the others.
NOTABLE = ("loop stopped", "Error:", "error:", "internal error",
           "WARNING", "takeover hold failed", "input rejected",
           "missing binary", "STALE:")


def local_addresses() -> list[str]:
    """The addresses this machine can be reached on, best guess first.

    The panel prints only 127.0.0.1 today, which on a second machine means
    that machine — so a --lan panel looks dead from the MacBook when it is
    running perfectly. Nothing here opens a connection: the UDP socket is
    connected but never sent to, which only asks the routing table which
    interface would carry the traffic.
    """
    found: list[str] = []
    # Ask the routing table which local address would carry traffic to each of
    # these. A UDP connect() sends nothing — it only fixes the peer and picks a
    # route — so nothing leaves the machine. The targets must be ROUTABLE or
    # connect() fails with ENETUNREACH and we learn nothing; the arm's own
    # subnet is tried first so a lab-only machine still gets an answer.
    for target in ("192.168.1.10", "8.8.8.8", "1.1.1.1"):
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.connect((target, 9))
            address = probe.getsockname()[0]
            if address not in found:
                found.append(address)
        except OSError:
            pass
        finally:
            probe.close()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            address = info[4][0]
            if address not in found:
                found.append(address)
    except OSError:
        pass
    return [a for a in found if not a.startswith("127.")]


def notable_lines(lines: list[str]) -> list[str]:
    return [line for line in lines
            if any(marker in line for marker in NOTABLE)]


def _describe_run(path: Path) -> str:
    """Whether this run can seed a plan, and how big it is.

    Deliberately not a row count: run logs reach gigabytes here and this report
    is generated on request, including from the panel. Whether there is at
    least one data row is the question that matters — a session whose
    controller never reached its loop leaves a preamble, a header and nothing
    else, and planner_bridge then refuses it with "no complete data row".
    """
    size_mb = path.stat().st_size / 1e6
    if not telemetry.has_data_rows(path):
        return f"{size_mb:.1f} MB, NO DATA ROWS: a solve cannot start from this"
    return f"{size_mb:.1f} MB, has data"


def _section(title: str) -> str:
    return f"\n{title}\n{'-' * len(title)}\n"


def report(port: int | None = None, lan: bool | None = None,
           replay: str | None = None) -> str:
    """The whole report as one block of text."""
    out: list[str] = []
    out.append("HumanSL control panel — diagnostics")
    out.append("(paste this whole block; it contains no credentials)")

    out.append(_section("PANEL").rstrip("\n"))
    out.append(f"  python        {platform.python_version()} on {platform.system()}")
    out.append(f"  repo          {paths.REPO}")
    if port is not None:
        out.append(f"  listening     port {port}, "
                   f"{'all interfaces (--lan)' if lan else 'loopback only'}")
        out.append(f"  on this machine  http://127.0.0.1:{port}")
        if lan:
            addresses = local_addresses()
            if addresses:
                for address in addresses:
                    out.append(f"  from elsewhere   http://{address}:{port}")
            else:
                out.append("  from elsewhere   no non-loopback address found")
        else:
            out.append("  from elsewhere   NOT reachable — restart with --lan")
    if replay:
        out.append(f"  replay        {replay}  (no arm involved)")

    out.append(_section("BUILD FRESHNESS").rstrip("\n"))
    try:
        fresh = build.freshness()
        out.append(f"  overall       {'STALE' if fresh['stale'] else 'up to date'}")
        for reason in fresh.get("reasons", []):
            out.append(f"    - {reason}")
        for name in ("controller", "bridge"):
            part = fresh.get(name, {})
            out.append(f"  {name:<13} {'missing' if not part.get('exists') else ''}"
                       f"{part.get('binary', '')}")
    except Exception as exc:  # a broken check must still leave a readable report
        out.append(f"  could not be read: {exc!r}")

    out.append(_section("SESSION").rstrip("\n"))
    try:
        state = session.status()
        out.append(f"  running       {'yes' if state.get('running') else 'no'}")
        for key in ("arm", "pid", "started", "session_dir", "log_path"):
            if state.get(key):
                out.append(f"  {key:<13} {state[key]}")
        if state.get("stop_command"):
            out.append(f"  stop with     {state['stop_command']}")
    except Exception as exc:
        out.append(f"  could not be read: {exc!r}")

    out.append(_section("RUN LOGS (what a solve would plan from)").rstrip("\n"))
    for arm in paths.ARMS:
        try:
            newest = telemetry.find_latest_csv(arm)
        except Exception:
            newest = None
        if newest is None:
            out.append(f"  {arm:<6} none under {paths.RUNS}")
            continue
        out.append(f"  {arm:<6} {newest.name}  {_describe_run(newest)}")

    out.append(_section("GENERATED DH TABLES").rstrip("\n"))
    for arm in paths.ARMS:
        table = dh.read(arm)
        status = "ok" if table["exists"] and not table["stale"] else (
            table.get("reason") or "stale")
        out.append(f"  {arm:<6} {status}")

    out.append(_section("GOAL").rstrip("\n"))
    try:
        goal = plan.read_goal()
        parsed = goal.get("parsed", {})
        out.append(f"  session_arms  {parsed.get('session_arms')}")
        for arm, block in (parsed.get("arms") or {}).items():
            out.append(f"  {arm:<6} frame={block.get('frame')} "
                       f"goal={block.get('goal')} "
                       f"rpy={block.get('orientation_rpy_deg')}")
        problems = plan.validate_goal(goal.get("text", ""))
        for problem in problems:
            out.append(f"    ! {problem}")
    except Exception as exc:
        out.append(f"  could not be read: {exc!r}")

    out.append(_section("LAST SOLVED PLAN").rstrip("\n"))
    for arm in paths.ARMS:
        try:
            last = plan.last_plan(arm) or {}
        except Exception:
            last = {}
        if last.get("points"):
            out.append(f"  {arm:<6} {last['points']} points, "
                       f"{last.get('duration_s', 0):.2f} s")
        elif last.get("error"):
            out.append(f"  {arm:<6} failed: {last['error']}")
        else:
            out.append(f"  {arm:<6} nothing solved in this panel session")

    out.append(_section(f"CONTROLLER LOG (last {LOG_LINES} lines)").rstrip("\n"))
    log_path = controller_log_path()
    if log_path is None:
        out.append("  no controller log found under runs/")
    else:
        out.append(f"  from {log_path}")
        try:
            lines = telemetry.tail_log_lines(log_path, LOG_LINES)
        except Exception as exc:
            lines = [f"(could not be read: {exc!r})"]
        interesting = notable_lines(lines)
        if interesting:
            out.append("  --- lines that look like the answer ---")
            for line in interesting:
                out.append(f"  >> {line}")
            out.append("  --- full tail ---")
        for line in lines:
            out.append(f"  {line}")

    return "\n".join(out) + "\n"


def controller_log_path() -> Path | None:
    """The live session's log, or the newest session's if none is running.

    The page needs this too: the event stream only carries lines written after
    a browser connects, so a panel opened after a session has already failed
    would show an empty log pane while the file holds the whole answer.
    """
    try:
        state = session.status()
    except Exception:
        state = {}
    if state.get("log_path") and Path(state["log_path"]).is_file():
        return Path(state["log_path"])
    candidates = sorted(paths.RUNS.glob("*/session_*/controller.log"),
                        key=lambda p: p.stat().st_mtime, reverse=True)
    return candidates[0] if candidates else None


def main() -> int:
    """`control_panel.py --check`. Zero means nothing obviously wrong."""
    text = report()
    sys.stdout.write(text)
    try:
        return 1 if build.freshness()["stale"] else 0
    except Exception:
        return 1
