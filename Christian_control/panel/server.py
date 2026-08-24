"""HTTP routing, static files, and the telemetry stream. Nothing else.

Every decision this file makes is about moving bytes. The judgements live in
the modules it calls: config_file owns what may be edited, build owns what
counts as stale, session owns what may start a run, telemetry owns what a row
means. Keeping them apart is what lets the safety-critical module (session) be
read and reviewed on its own.

Two rules from the design are enforced *here*, because they are properties of
the connection rather than of the work:

  * Starting a session is refused unless the request came from the loopback
    interface. You must be at the workstation, beside the physical e-stop, to
    begin motion. Stopping is accepted from anywhere: remote stop is always
    safe, remote start never is.
  * The panel binds loopback only unless it is started with --lan, so remote
    viewing is something switched on deliberately.
"""

from __future__ import annotations

import errno
import json
import mimetypes
import re
import subprocess
import sys
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse

from . import (build, config_file, dh, diagnose, paths, plan, planner_config,
               plots, runs, scene_config, session, telemetry)

# Set by serve(); read by the handler. Plain module state rather than a class
# attribute so the handler stays a thin dispatcher.
_REPLAY: dict[str, Any] = {"path": None, "speed": 1.0, "arm": None}

# What serve() was asked for, so the diagnostic report can state the URLs this
# panel is actually reachable on rather than guessing.
_SERVING: dict[str, Any] = {"port": None, "lan": False}

_LOOPBACK = ("127.0.0.1", "::1", "::ffff:127.0.0.1")

# How often the stream loop wakes. The controller logs at 500 Hz; the panel
# samples at ~20 Hz and every frame carries the worst value seen since the
# previous one, so a 4 ms spike between samples is still reported.
_STREAM_PERIOD_S = 0.05

# Scene persistence and session start both transition from an idle panel
# state. Serialising those two HTTP transactions closes the gap between the
# save's final commanding check and session.start beginning its launch path.
# This lock is panel workflow coordination only; it never enters the runtime
# or 500 Hz controller.
_SCENE_SESSION_TRANSACTION = threading.Lock()


def _is_local(client_address: tuple) -> bool:
    return bool(client_address) and client_address[0] in _LOOPBACK


def replay_active() -> bool:
    return _REPLAY["path"] is not None


def replay_arm() -> str | None:
    """Which arm the recording being replayed actually belongs to.

    A recording is one arm's. Serving it for both would draw a second arm that
    was never there — the panel would show a left arm moving because the right
    arm's log said so. The run's own preamble names the arm ("# arm = right
    (192.168.1.10)"); the filename prefix is the fallback.
    """
    path = _REPLAY["path"]
    if path is None:
        return None
    if _REPLAY["arm"] is None:
        _REPLAY["arm"] = _arm_of_recording(Path(path))
    return _REPLAY["arm"]


class _SilentArm:
    """The arm a replay is not about: heartbeats, never rows.

    Stands in for telemetry.Source on that arm's stream so the loop below is
    written once. It says why it is empty, because "no data" and "this
    recording is the other arm's" are different things to the person reading.
    """

    def __init__(self, arm: str, recorded_arm: str, period_s: float) -> None:
        self.arm = arm
        self.reason = f"the recording being replayed is the {recorded_arm} arm's"
        self.period_s = period_s
        self._stopped = False

    def stop(self) -> None:
        self._stopped = True

    def frames(self):
        while not self._stopped:
            yield "heartbeat", {
                "arm": self.arm,
                "wall_ms": _now_ms(),
                "server_age_s": None,
                "growing": False,
                "reason": self.reason,
            }
            time.sleep(self.period_s)


def _arm_of_recording(path: Path) -> str | None:
    try:
        with open(path, errors="replace") as handle:
            for line in handle:
                if not line.startswith("#"):
                    break
                if line.startswith("# arm ="):
                    named = line.split("=", 1)[1].strip().split()[0]
                    if named in paths.ARMS:
                        return named
    except OSError:
        pass
    for arm in paths.ARMS:
        if path.name.startswith(f"loop_log_{arm}"):
            return arm
    return None


class _Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "HumanSLPanel/1.0"

    # ---- small response helpers -------------------------------------

    def _send(self, body: bytes, content_type: str, code: int = 200) -> None:
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _json(self, obj: Any, code: int = 200) -> None:
        self._send(json.dumps(obj, allow_nan=False, default=_jsonable).encode(),
                   "application/json", code)

    def _body(self) -> dict:
        length = int(self.headers.get("Content-Length", 0) or 0)
        if length <= 0:
            return {}
        try:
            parsed = json.loads(self.rfile.read(length))
        except (json.JSONDecodeError, UnicodeDecodeError):
            return {}
        return parsed if isinstance(parsed, dict) else {}

    def log_message(self, *args) -> None:  # keep the terminal quiet
        pass

    # ---- static files -----------------------------------------------

    def _static(self, rel: str) -> None:
        if not rel or rel.endswith("/"):
            rel = "index.html"
        target = (paths.STATIC / rel).resolve()
        root = paths.STATIC.resolve()
        if not str(target).startswith(str(root)) or not target.is_file():
            self._json({"error": f"no such file: {rel}"}, 404)
            return
        kind, _ = mimetypes.guess_type(target.name)
        self._send(target.read_bytes(), kind or "application/octet-stream")

    def _plot_image(self, rel: str) -> None:
        """A PNG plots.run() produced, by its path relative to the repo
        root (as returned in that response's "images" list). Same
        resolve-then-compare traversal guard as _static(), rooted at the
        repo instead of static/ since these files live under runs/."""
        if not rel:
            self._json({"error": "missing path"}, 400)
            return
        target = (paths.REPO / rel).resolve()
        root = paths.REPO.resolve()
        if (not str(target).startswith(str(root)) or not target.is_file()
                or target.suffix.lower() != ".png"):
            self._json({"error": f"no such plot image: {rel}"}, 404)
            return
        self._send(target.read_bytes(), "image/png")

    # ---- GET ---------------------------------------------------------

    def do_GET(self) -> None:  # noqa: N802 (http.server's spelling)
        url = urlparse(self.path)
        route = url.path
        query = parse_qs(url.query)
        try:
            if route == "/api/status":
                self._json(status_snapshot())
            elif route == "/api/config":
                self._json({
                    "knobs": config_file.read_knobs(),
                    "vector_knobs": config_file.read_vector_knobs(),
                    "thresholds": config_file.read_thresholds(),
                    "joints": config_file.read_joint_limits(),
                })
            elif route == "/api/goal":
                self._json(plan.read_goal())
            elif route == "/api/runs":
                self._json(runs.list_runs())
            elif route == "/api/run":
                rel = (query.get("f") or [""])[0]
                self._json(runs.summarize_run(rel))
            elif route == "/api/plots":
                self._json(plots.list_scripts())
            elif route == "/api/plot-image":
                self._plot_image((query.get("path") or [""])[0])
            elif route == "/api/controller-log":
                # The tail as it stands, so a page that opens after the
                # interesting part was written still shows it.
                path = diagnose.controller_log_path()
                count = int((query.get("n") or ["60"])[0])
                self._json({
                    "path": str(path) if path else None,
                    "lines": telemetry.tail_log_lines(path, count) if path else [],
                })
            elif route == "/api/diagnose":
                # Plain text, so it is readable in a terminal with curl and
                # pasteable without unwrapping JSON.
                self._send(diagnose.report(port=_SERVING["port"],
                                           lan=_SERVING["lan"],
                                           replay=_REPLAY["path"]).encode(),
                           "text/plain; charset=utf-8")
            elif route == "/api/build/status":
                self._json(build.build_status())
            elif route == "/api/planner-config":
                self._json({
                    "planner": planner_config.read_planner_knobs(),
                    "joint_limits": planner_config.read_joint_limits_file(),
                })
            elif route == "/api/scene":
                value = scene_config.read_scene()
                commanding = bool(session.status().get("commanding"))
                available = (
                    value.get("error") is None
                    and value.get("source_fnv1a64") is not None
                )
                value["save_allowed"] = not commanding and available
                if commanding:
                    value["save_blocked_reason"] = "controller is commanding"
                elif not available:
                    value["save_blocked_reason"] = (
                        "planner scene is unavailable: "
                        + str(value.get("error") or "source token is missing")
                    )
                else:
                    value["save_blocked_reason"] = None
                self._json(value)
            elif route == "/api/session/status":
                self._json(session.status())
            elif route == "/api/stream":
                self._stream((query.get("arm") or ["right"])[0])
            elif route.startswith("/api/"):
                self._json({"error": f"no such endpoint: {route}"}, 404)
            else:
                self._static(route.lstrip("/"))
        except Exception as exc:  # a panel bug must not take the panel down
            self._fail(exc)

    # ---- POST --------------------------------------------------------

    def do_POST(self) -> None:  # noqa: N802
        route = urlparse(self.path).path
        try:
            if route == "/api/config/set":
                req = self._body()
                name = str(req.get("name", ""))
                if name in config_file.VECTOR_KNOBS:
                    # A vector value arrives as a JSON array of 7 numbers, so
                    # it must reach write_vector_knob unstringified — casting
                    # it with str() the way the scalar path does would turn
                    # the list into "[1, 2, ...]" and every write would fail.
                    ok, message = config_file.write_vector_knob(
                        name, req.get("value"))
                else:
                    ok, message = config_file.write_knob(
                        name, str(req.get("value", "")))
                self._json({"ok": True, "value": message} if ok
                           else {"error": message}, 200 if ok else 400)
            elif route == "/api/build":
                target = str(self._body().get("target", "controller"))
                ok, message = build.start_build(target)
                self._json({"started": True, "target": target} if ok
                           else {"error": message}, 200 if ok else 409)
            elif route == "/api/goal":
                ok, message = plan.write_goal(str(self._body().get("text", "")))
                self._json({"ok": True} if ok else {"error": message},
                           200 if ok else 400)
            elif route == "/api/goal/fields":
                req = self._body()
                ok, message = plan.write_goal_fields(
                    str(req.get("arm", "")), req.get("fields") or {})
                self._json({"ok": True} if ok else {"error": message},
                           200 if ok else 400)
            elif route == "/api/plots/run":
                req = self._body()
                self._json(plots.run(str(req.get("script", "")),
                                     str(req.get("run", "")),
                                     req.get("params") or {}))
            elif route == "/api/session/start":
                req = self._body()
                # session.start makes the loopback decision itself, from the
                # address it is handed. The server does not pre-judge it: one
                # module owns that rule, and it is the one that is tested.
                with _SCENE_SESSION_TRANSACTION:
                    result = session.start(
                        arm=str(req.get("arm", "")),
                        confirm=str(req.get("confirm", "")),
                        client_address=self.client_address,
                        mount=str(req.get("mount", "vicon")),
                        planning=bool(req.get("planning", True)),
                        recording=bool(req.get("recording", True)),
                        fixed_pose=req.get("fixed_pose"),
                    )
                self._json(result, 200 if result.get("ok") else 400)
            elif route == "/api/session/stop":
                # Never gated on UI state, staleness, or a live stream: this
                # must work from a page whose JavaScript has been dead for an
                # hour.
                result = session.stop()
                self._json(result, 200 if result.get("ok") else 400)
            elif route == "/api/session/goal":
                req = self._body()
                result = session.send_goal(
                    str(req.get("arm", "")), req.get("fields") or {})
                self._json(result, 200 if result.get("ok") else 409)
            elif route == "/api/planner-config/set":
                req = self._body()
                if req.get("file") == "joint_limits":
                    ok, message = planner_config.write_joint_limit(
                        str(req.get("section", "")),
                        str(req.get("actuator", "")),
                        str(req.get("bound", "")),
                        req.get("value"))
                else:
                    ok, message = planner_config.write_planner_knob(
                        str(req.get("name", "")), req.get("value"))
                self._json({"ok": True, "value": message} if ok
                           else {"error": message}, 200 if ok else 400)
            elif route == "/api/scene":
                req = self._body()
                with _SCENE_SESSION_TRANSACTION:
                    ok, value = scene_config.write_scene(
                        req.get("scene"),
                        req.get("source_fnv1a64"),
                        commanding=lambda: bool(
                            session.status().get("commanding")
                        ),
                    )
                self._json(value if ok else {"error": value},
                           200 if ok else 409)
            else:
                self._json({"error": f"no such endpoint: {route}"}, 404)
        except Exception as exc:
            self._fail(exc)

    def _fail(self, exc: Exception) -> None:
        detail = "".join(traceback.format_exception_only(type(exc), exc)).strip()
        try:
            self._json({"error": f"panel error: {detail}"}, 500)
        except Exception:
            pass

    # ---- the telemetry stream ---------------------------------------

    def _stream(self, arm: str) -> None:
        """One Server-Sent Events connection.

        Everything the page needs while a run is happening arrives here:
        telemetry rows, heartbeats when the log stops growing, the controller's
        own stdout, build output, and session state. The loop polls; nothing
        blocks on anything else, so one slow source cannot stall the others.
        """
        if arm not in paths.ARMS:
            self._json({"error": f"unknown arm '{arm}'"}, 400)
            return
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()

        # A recording belongs to one arm. The other arm must say it has
        # nothing rather than be drawn from someone else's telemetry, and it
        # must not quietly fall back to that arm's own newest log either — the
        # operator asked to replay one file.
        recorded_arm = replay_arm()
        if replay_active() and recorded_arm not in (None, arm):
            source = _SilentArm(arm, recorded_arm, _STREAM_PERIOD_S)
        else:
            source = telemetry.Source(
                arm,
                path=Path(_REPLAY["path"]) if replay_active() else None,
                mode="replay" if replay_active() else "live",
                speed=_REPLAY["speed"],
                poll_interval_s=_STREAM_PERIOD_S,
                frame_interval_s=_STREAM_PERIOD_S,
            )
        log_tail = telemetry.LineTail()
        log_path_followed: str | None = None
        build_sent = 0
        build_done_sent = False
        last_session: dict | None = None

        try:
            self._event("hello", {"arm": arm, "wall_ms": _now_ms(),
                                  "replay": replay_active(),
                                  "local": _is_local(self.client_address)})
            # The telemetry source yields once per poll interval whether or not
            # the log grew — a heartbeat when it did not — so its generator is
            # this loop's clock. Everything else is drained on the same tick,
            # which keeps one slow source from stalling the others.
            for kind, payload in source.frames():
                self._event(kind, payload)

                state = session.status()
                if state != last_session:
                    self._event("session", state)
                    last_session = state
                    log_path = state.get("log_path")
                    if log_path and log_path != log_path_followed:
                        log_tail.open(Path(log_path))
                        log_path_followed = log_path

                if log_path_followed:
                    for line in log_tail.poll():
                        self._event("log", {"line": line})

                status = build.build_status()
                lines = status.get("lines", [])
                if len(lines) < build_sent:  # a new build reset the buffer
                    build_sent = 0
                    build_done_sent = False
                for line in lines[build_sent:]:
                    self._event("build", {"line": line})
                build_sent = len(lines)
                if status.get("running"):
                    build_done_sent = False
                elif status.get("ok") is not None and not build_done_sent:
                    # Each connection reports the completion once. A page that
                    # joins after a build finished still learns how it went.
                    self._event("build", {"done": True,
                                          "ok": bool(status.get("ok")),
                                          "target": status.get("target")})
                    build_done_sent = True
        except (BrokenPipeError, ConnectionResetError):
            pass  # the browser went away; nothing to clean up
        except Exception:
            pass
        finally:
            source.stop()

    def _event(self, kind: str, payload: dict) -> None:
        body = f"event: {kind}\ndata: {json.dumps(payload, default=_jsonable)}\n\n"
        self.wfile.write(body.encode())
        self.wfile.flush()


def _jsonable(value: Any) -> Any:
    """NaN and Path are the two things that reach json.dumps from this code.

    NaN is deliberate in the log — it means "this cycle did not compute it" —
    and it must survive as null rather than as 0.0, which would read as perfect
    tracking.
    """
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, float):
        return None
    return str(value)


def _now_ms() -> int:
    return int(time.time() * 1000)


def status_snapshot() -> dict:
    """Everything the status pane needs in one request."""
    freshness = build.freshness()
    return {
        "freshness": freshness,
        "session": session.status(),
        "goal_arms": plan.session_arms(),
        "dh": {arm: dh.read(arm) for arm in paths.ARMS},
        "replay": replay_active(),
        "hardware": True,  # there is no simulator in this repository
        "wall_ms": _now_ms(),
    }


def _report_port_in_use(port: int) -> None:
    """Say who holds the port and give the exact command to free it.

    A bare "Address already in use" traceback leaves you guessing which
    process to stop; the panel already knows, so it should just say.
    """
    holders = _pids_listening_on(port)
    print(f"port {port} is already in use — another control panel is "
          f"probably still running.", file=sys.stderr)
    if holders:
        print("  stop it with:", file=sys.stderr)
        print(f"    kill {' '.join(str(pid) for pid in holders)}",
              file=sys.stderr)
    else:
        print("  stop it with:", file=sys.stderr)
        print(f"    fuser -k {port}/tcp", file=sys.stderr)
    print("  then start the panel again, or use a different port:",
          file=sys.stderr)
    print(f"    python3 Christian_control/panel/control_panel.py "
          f"--port {port + 1}", file=sys.stderr)


def _pids_listening_on(port: int) -> list[int]:
    """Process ids listening on a TCP port, newest-first, or [] if unknown."""
    try:
        found = subprocess.run(["ss", "-ltnpH"], capture_output=True,
                               text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    pids: list[int] = []
    for line in found.splitlines():
        address = line.split()[3] if len(line.split()) > 3 else ""
        if not address.endswith(f":{port}"):
            continue
        pids.extend(int(pid) for pid in re.findall(r"pid=(\d+)", line))
    return sorted(set(pids))


def serve(port: int = 8765, lan: bool = False, replay: str | None = None,
          speed: float = 1.0) -> None:
    _REPLAY["path"] = replay
    _REPLAY["speed"] = speed
    _SERVING["port"] = port
    _SERVING["lan"] = lan
    host = "0.0.0.0" if lan else "127.0.0.1"
    try:
        server = ThreadingHTTPServer((host, port), _Handler)
    except OSError as error:
        if error.errno != errno.EADDRINUSE:
            raise
        _report_port_in_use(port)
        raise SystemExit(1) from None
    server.daemon_threads = True
    print(f"control panel: http://127.0.0.1:{port}   (on this machine)")
    if lan:
        # Print the addresses, not the fact that they exist. "also reachable
        # from this machine's network addresses" sent someone to type
        # 127.0.0.1 into a laptop, where it means the laptop, and the panel
        # looked dead while it was running perfectly.
        for address in diagnose.local_addresses():
            print(f"               http://{address}:{port}   (from another machine)")
        print("  Starting a session is still refused from anywhere but this machine.")
    if replay:
        print(f"  replay: {replay} at {speed}x — no robot involved")
    print("  Ctrl+C to stop the panel. Stopping the panel never stops a run.")
    # serve_forever never returns, so a buffered stdout — which is what
    # redirecting to a file gives you — would hold the banner, and the URLs
    # above, until the panel was killed.
    sys.stdout.flush()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\npanel stopped. Any running session is untouched.")
    finally:
        server.server_close()


# Threads that outlive a request (the build) are daemons, so Ctrl+C on the
# panel is immediate and never waits on cmake.
threading.stack_size(1024 * 1024)
