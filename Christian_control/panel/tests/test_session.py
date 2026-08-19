"""Tests for panel/session.py — the one module whose failure reaches an arm.

No real binary is launched here. The session under test is a fake script the
test writes itself, and the one test that matters proves that stop() puts a
real SIGINT into a real process group: the fake starts a grandchild, and the
grandchild's record of the signal is the evidence, because a signal sent to
the leader alone could never have reached it.

That is the exact failure `planner_bridge/tests/test_run_session.sh` exists to
prevent — `$!` was a wrapper subshell, `kill -INT "$!"` never arrived, and the
arm kept moving.
"""

import inspect
import json
import os
import signal
import subprocess
import tempfile
import time
import unittest
from pathlib import Path

from .. import paths, session

# Shaped like build.freshness(): per-component entries carrying their own
# reasons, so the choice-aware gate can block on only what a session needs.
FRESH = {
    "stale": False, "reasons": [],
    "controller": {"stale": False, "reasons": []},
    "bridge": {"stale": False, "reasons": []},
    "dh": {"right": {"stale": False, "reasons": []},
           "left": {"stale": False, "reasons": []}},
}
STALE = {
    "stale": True,
    "reasons": ["controller is older than src/Runner.cpp"],
    "controller": {"stale": True,
                   "reasons": ["controller is older than src/Runner.cpp"]},
    "bridge": {"stale": False, "reasons": []},
    "dh": {"right": {"stale": False, "reasons": []},
           "left": {"stale": False, "reasons": []}},
}

LOOPBACK = ("127.0.0.1", 51234)


def wait_for(predicate, timeout_s: float = 5.0) -> bool:
    """Poll until a condition holds, so the tests do not race the OS."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return predicate()


def dead_pid() -> int:
    """A pid that is certainly gone, and reaped, so signal 0 fails on it."""
    finished = subprocess.Popen(["true"])
    finished.wait()
    return finished.pid


class SessionTestCase(unittest.TestCase):
    """Shared isolation: a temporary state file and stubbed seams."""

    def setUp(self) -> None:
        self.tmp = Path(tempfile.mkdtemp(prefix="panel_session_test_"))
        self.real_state = paths.SESSION_STATE
        self.real_runs = paths.RUNS
        paths.SESSION_STATE = self.tmp / "session.json"
        paths.RUNS = self.tmp / "runs"  # nothing is written into the real runs/

        self.real_launch = session.launch_command
        self.real_signal = session.send_signal_to_group
        self.real_freshness = session.freshness_check
        session.freshness_check = lambda: FRESH

        session._process = None
        session._stdin_write_fd = None

    def tearDown(self) -> None:
        # Whatever a test left running is stopped hard here — this is the test
        # process cleaning up its own fakes, not a stop path the panel has.
        state = session._read_state()
        if state and state.get("pgid"):
            try:
                os.killpg(state["pgid"], signal.SIGKILL)
            except OSError:
                pass
        if session._process is not None:
            try:
                session._process.wait(timeout=5)
            except (subprocess.TimeoutExpired, OSError):
                pass
        session._forget_process()

        paths.SESSION_STATE = self.real_state
        paths.RUNS = self.real_runs
        session.launch_command = self.real_launch
        session.send_signal_to_group = self.real_signal
        session.freshness_check = self.real_freshness


class RefusalTests(SessionTestCase):
    """Everything start() must say no to, before anything is launched."""

    def setUp(self) -> None:
        super().setUp()
        self.launched = []

        def record_only(arm: str, *choices) -> list[str]:
            self.launched.append(arm)
            return ["true"]

        session.launch_command = record_only

    def test_refuses_a_non_loopback_client(self) -> None:
        result = session.start("right", "GO", ("192.168.1.42", 5000))
        self.assertFalse(result["ok"])
        self.assertIn("localhost-only", result["error"])
        self.assertIn("workstation", result["error"])
        self.assertEqual(self.launched, [])

    def test_refuses_when_the_server_says_the_client_is_remote(self) -> None:
        # The server passes its own reading; disagreement must refuse.
        result = session.start("right", "GO", LOOPBACK, is_local=False)
        self.assertFalse(result["ok"])
        self.assertIn("localhost-only", result["error"])
        self.assertEqual(self.launched, [])

    def test_refuses_without_the_typed_go(self) -> None:
        for confirm in ("", "go", "GO ", "yes", "GOO"):
            result = session.start("right", confirm, LOOPBACK)
            self.assertFalse(result["ok"], confirm)
            self.assertIn("GO", result["error"])
        self.assertEqual(self.launched, [])

    def test_refuses_an_unknown_arm(self) -> None:
        result = session.start("both arms", "GO", LOOPBACK)
        self.assertFalse(result["ok"])
        self.assertEqual(self.launched, [])

    def test_refuses_a_stale_binary_and_names_the_reason(self) -> None:
        session.freshness_check = lambda: STALE
        result = session.start("right", "GO", LOOPBACK)
        self.assertFalse(result["ok"])
        self.assertIn("controller is older than src/Runner.cpp", result["reasons"])
        self.assertEqual(self.launched, [])

    def test_refuses_when_freshness_cannot_answer(self) -> None:
        def broken():
            raise OSError("build directory vanished")

        session.freshness_check = broken
        result = session.start("right", "GO", LOOPBACK)
        self.assertFalse(result["ok"])
        self.assertIn("build directory vanished", result["error"])
        self.assertEqual(self.launched, [])

    def test_stop_without_a_session_is_a_no_op_that_says_so(self) -> None:
        result = session.stop()
        self.assertTrue(result["ok"])
        self.assertFalse(result["stopped"])
        self.assertIn("no session", result["message"])

    def test_status_without_a_session(self) -> None:
        state = session.status()
        self.assertFalse(state["running"])
        self.assertIsNone(state["pid"])
        self.assertIsNone(state["stop_command"])
        self.assertFalse(state["attached"])


class LaunchCommandTests(unittest.TestCase):
    """The command itself, which is where --allow-stale could sneak in."""

    def test_runs_run_session_with_the_session_choices(self) -> None:
        self.assertEqual(
            session.default_launch_command("left", "vicon", True, True),
            [str(paths.RUN_SESSION_SH), "--arm", "left", "--mount", "vicon",
             "--plan", "on", "--record", "on"],
        )
        self.assertEqual(
            session.default_launch_command("right", "fixed", False, False),
            [str(paths.RUN_SESSION_SH), "--arm", "right", "--mount", "fixed",
             "--plan", "off", "--record", "off"],
        )

    def test_a_fixed_pose_becomes_seven_numbers_in_one_argument(self) -> None:
        command = session.default_launch_command(
            "right", "fixed", False, True,
            [0.1, -0.2, 0.3, 0.0, 0.0, 0.0, 1.0])
        self.assertIn("--fixed-pose", command)
        pose = command[command.index("--fixed-pose") + 1]
        self.assertEqual(pose.split(), ["0.1", "-0.2", "0.3", "0", "0", "0", "1"])

    def test_never_passes_allow_stale(self) -> None:
        for arm in session.VALID_ARMS:
            for mount in session.VALID_MOUNTS:
                for planning in (True, False):
                    self.assertNotIn(
                        "--allow-stale",
                        session.default_launch_command(arm, mount, planning, True))

    def test_a_session_with_no_controller_is_not_commanding(self) -> None:
        """run_session.sh outlives its controller, and the panel must not lie.

        After the loop exits the script sits at `read` waiting for the Enter
        that ends the session, so the process group stays alive with nothing
        driving an arm. On 2026-08-11 that made the panel show DRIVING BOTH
        ARMS over a workstation whose controller had already stopped.
        """
        # Our own group certainly has no controller process in it.
        self.assertFalse(session.controller_alive(os.getpgid(0)))

    def test_controller_alive_is_false_for_nothing_and_for_a_dead_group(self) -> None:
        self.assertFalse(session.controller_alive(None))
        self.assertFalse(session.controller_alive(0))
        self.assertFalse(session.controller_alive(2 ** 31 - 1))

    def test_no_session_reports_both_fields(self) -> None:
        state = session.status()
        if state["running"]:
            self.skipTest("a session is running on this machine")
        self.assertFalse(state["commanding"])
        self.assertEqual(state["note"], "")

    def test_both_arms_is_one_command_with_arm_both(self) -> None:
        """--arm both is ONE controller process driving two arms, not two runs.

        run_session.sh starts a single controller with two threads and one stop
        flag, then runs planner_bridge once per arm. The panel must not invent
        a second process: two controllers would both try to take servoing, and
        ProcessLock would refuse the second anyway.
        """
        self.assertEqual(
            session.default_launch_command("both", "vicon", True, True)[:3],
            [str(paths.RUN_SESSION_SH), "--arm", "both"],
        )

    def test_an_arm_that_is_not_right_left_or_both_is_refused(self) -> None:
        self.assertEqual(session.VALID_ARMS, ("right", "left", "both"))
        result = session.start("all", "GO", LOOPBACK)
        self.assertFalse(result["ok"])
        self.assertIn("both", result["error"])


class RequiredDependencyGateTests(SessionTestCase):
    """A component may block startup only if the selected session actually
    requires its output. Each case here is one of the session shapes the
    2026-08-19 dependency cleanup must support, against a freshness report
    where exactly one component is stale."""

    # A stand-in session: announces its directory like run_session.sh:103
    # does, then waits to be stopped. `exec` so the printed pid IS the group
    # leader stop() signals.
    FAKE = ["bash", "-c",
            "echo 'session artifacts: /tmp/fake_session'; exec sleep 30"]

    def setUp(self) -> None:
        super().setUp()
        session.launch_command = lambda arm, *choices: list(self.FAKE)

    @staticmethod
    def report(controller_stale=False, bridge_stale=False, dh_stale=False):
        def entry(stale, sentence):
            return {"stale": stale, "reasons": [sentence] if stale else []}
        return {
            "stale": controller_stale or bridge_stale or dh_stale,
            "reasons": [],
            "controller": entry(controller_stale, "controller is older than x.cpp"),
            "bridge": entry(bridge_stale, "planner_bridge is older than y.cpp"),
            "dh": {"right": entry(dh_stale, "dh_params_tool.yaml is older than the URDF"),
                   "left": entry(dh_stale, "dh_params_flange.yaml is older than the URDF")},
        }

    # The six session shapes under test. (arm, mount, planning)
    COMBOS = [
        ("right", "fixed", False),
        ("both", "fixed", False),
        ("right", "fixed", True),
        ("both", "fixed", True),
        ("right", "vicon", False),
        ("both", "vicon", True),
    ]

    def start(self, combo, report):
        session.freshness_check = lambda: report
        arm, mount, planning = combo
        result = session.start(arm, "GO", LOOPBACK, mount=mount,
                               planning=planning, recording=True)
        # Leave no session behind for the next case: stop, then wait for the
        # child to actually die, or the next start() sees it still running.
        process = session._process
        session.stop()
        if process is not None:
            process.wait(timeout=10)
        session.status()  # reaps and clears the state file
        return result

    def test_a_stale_controller_blocks_every_session_shape(self) -> None:
        for combo in self.COMBOS:
            result = self.start(combo, self.report(controller_stale=True))
            self.assertFalse(result["ok"], combo)
            self.assertIn("controller is older than x.cpp",
                          result.get("reasons", []), combo)

    def test_a_stale_planner_bridge_blocks_no_session_shape(self) -> None:
        """planner_bridge is a preview tool; the running controller never
        executes it, so its staleness must never stop hardware execution."""
        for combo in self.COMBOS:
            result = self.start(combo, self.report(bridge_stale=True))
            self.assertTrue(result["ok"], (combo, result))

    def test_stale_dh_tables_block_exactly_the_planning_sessions(self) -> None:
        for combo in self.COMBOS:
            planning = combo[2]
            result = self.start(combo, self.report(dh_stale=True))
            if planning:
                self.assertFalse(result["ok"], combo)
                self.assertTrue(any("older than the URDF" in r
                                    for r in result.get("reasons", [])), combo)
            else:
                self.assertTrue(result["ok"], (combo, result))

    def test_everything_fresh_starts_every_session_shape(self) -> None:
        for combo in self.COMBOS:
            result = self.start(combo, self.report())
            self.assertTrue(result["ok"], (combo, result))

    def test_the_choices_reach_the_launch_command(self) -> None:
        seen = []

        def record(arm, mount, planning, recording, fixed_pose=None):
            seen.append((arm, mount, planning, recording, fixed_pose))
            return list(self.FAKE)

        session.launch_command = record
        session.freshness_check = lambda: self.report()
        result = session.start("right", "GO", LOOPBACK, mount="fixed",
                               planning=False, recording=False,
                               fixed_pose=[0.1, 0, 0, 0, 0, 0, 1])
        process = session._process
        session.stop()
        if process is not None:
            process.wait(timeout=10)
        session.status()
        self.assertTrue(result["ok"], result)
        self.assertEqual(seen, [("right", "fixed", False, False,
                                 [0.1, 0, 0, 0, 0, 0, 1])])

    def test_the_identity_fixed_pose_is_the_no_flag_default(self) -> None:
        seen = []

        def record(arm, mount, planning, recording, fixed_pose=None):
            seen.append(fixed_pose)
            return list(self.FAKE)

        session.launch_command = record
        session.freshness_check = lambda: self.report()
        result = session.start("right", "GO", LOOPBACK, mount="fixed",
                               planning=False,
                               fixed_pose=[0, 0, 0, 0, 0, 0, 1])
        process = session._process
        session.stop()
        if process is not None:
            process.wait(timeout=10)
        session.status()
        self.assertTrue(result["ok"], result)
        self.assertEqual(seen, [None])

    def test_bad_session_choices_are_refused_before_launch(self) -> None:
        session.freshness_check = lambda: self.report()
        cases = [
            dict(mount="gps"),
            dict(mount="vicon", fixed_pose=[0, 0, 0, 0, 0, 0, 1]),
            dict(mount="fixed", fixed_pose=[0, 0, 0]),
            dict(mount="fixed", fixed_pose=[0, 0, 0, 0, 0, 0, float("nan")]),
            dict(mount="fixed", fixed_pose=["x", 0, 0, 0, 0, 0, 1]),
        ]
        for kwargs in cases:
            result = session.start("right", "GO", LOOPBACK, **kwargs)
            self.assertFalse(result["ok"], kwargs)

    def test_a_malformed_report_fails_closed_for_every_shape(self) -> None:
        for combo in self.COMBOS:
            result = self.start(combo, {"stale": True})
            self.assertFalse(result["ok"], combo)


class LoopbackTests(unittest.TestCase):
    def test_loopback_addresses(self) -> None:
        for address in [("127.0.0.1", 1), ("127.0.0.5", 1), ("::1", 1, 0, 0),
                        "localhost", "::ffff:127.0.0.1", "[::1]"]:
            self.assertTrue(session.is_loopback(address), address)

    def test_remote_addresses(self) -> None:
        for address in [("192.168.1.42", 1), ("10.0.0.1", 1), ("", 1), None,
                        (), "fe80::1%eth0", "example.com"]:
            self.assertFalse(session.is_loopback(address), address)


class StaleReasonTests(unittest.TestCase):
    def test_uses_the_reports_own_sentences(self) -> None:
        self.assertEqual(session.stale_reasons(STALE), STALE["reasons"])

    def test_fresh_report_gives_no_reasons(self) -> None:
        self.assertEqual(session.stale_reasons(FRESH), [])

    def test_a_reshaped_report_still_refuses(self) -> None:
        # No `reasons` key at all: the per-target flags must still be read.
        self.assertEqual(session.stale_reasons({"bridge": {"stale": True}}),
                         ["bridge is stale"])
        self.assertEqual(session.stale_reasons({"stale": True}),
                         ["the build is stale"])

    def test_an_unreadable_report_refuses(self) -> None:
        self.assertTrue(session.stale_reasons(None))


class FakeSessionTests(SessionTestCase):
    """start() and stop() against a fake session script.

    The fake stands in for run_session.sh: it announces a session directory,
    reads GO from stdin, keeps a child of its own, and traps SIGINT.
    """

    def setUp(self) -> None:
        super().setUp()
        self.record = self.tmp / "record.txt"
        self.session_dir = self.tmp / "session_120000"
        self.session_dir.mkdir()

        # The grandchild is Python, not shell, for the reason
        # test_run_session.sh documents: a background job of a non-interactive
        # shell inherits SIGINT as SIG_IGN and bash then refuses to trap it,
        # so a shell grandchild would prove nothing. signal.signal() resets an
        # inherited SIG_IGN, exactly as the real controller's
        # std::signal(SIGINT, ...) at Main.cpp:588 does.
        self.grandchild = self.tmp / "grandchild.py"
        self.grandchild.write_text(
            "import signal, sys, time\n"
            "record = sys.argv[1]\n"
            "def note(signum, frame):\n"
            "    open(record, 'a').write('GRANDCHILD_SIGINT\\n')\n"
            "    sys.exit(0)\n"
            "signal.signal(signal.SIGINT, note)\n"
            "open(record, 'a').write('GRANDCHILD_READY\\n')\n"
            "time.sleep(120)\n"
        )

        # The second `read` stands for run_session.sh's bare `read` — the one
        # waiting for Enter before it stops the controller. It must never
        # return, because the panel stops by signal instead.
        self.fake = self.tmp / "fake_session.sh"
        self.fake.write_text(
            "#!/usr/bin/env bash\n"
            f"record='{self.record}'\n"
            f"python3 '{self.grandchild}' \"$record\" &\n"
            "trap 'echo LEADER_SIGINT >> \"$record\"; exit 0' INT\n"
            f"echo 'session artifacts: {self.session_dir}'\n"
            "read -r typed\n"
            "echo \"typed: $typed\" >> \"$record\"\n"
            "read -r second\n"
            "echo 'SECOND_READ_RETURNED' >> \"$record\"\n"
            "sleep 120\n"
        )
        self.fake.chmod(0o755)
        session.launch_command = (
            lambda arm, *choices: ["bash", str(self.fake), arm])

    def recorded(self) -> str:
        try:
            return self.record.read_text()
        except OSError:
            return ""

    def test_start_types_go_and_reports_the_session(self) -> None:
        result = session.start("right", "GO", LOOPBACK)
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["session_dir"], str(self.session_dir))
        self.assertEqual(result["pgid"], result["pid"])
        self.assertEqual(result["stop_command"], f"kill -INT -{result['pid']}")

        # The GO gate did not disappear when it moved into the browser: the
        # script still received the word on its stdin.
        self.assertTrue(wait_for(lambda: "typed: GO" in self.recorded()),
                        self.recorded())

        # Launched detached, in its own session, so a panel exit is not the
        # arm's problem.
        self.assertEqual(os.getpgid(result["pid"]), result["pid"])
        self.assertNotEqual(result["pid"], os.getpid())

        state = json.loads(paths.SESSION_STATE.read_text())
        self.assertEqual(state["arm"], "right")
        self.assertEqual(state["pgid"], result["pid"])
        self.assertEqual(Path(state["log_path"]).parent, self.session_dir)
        self.assertIn("session artifacts:", Path(state["log_path"]).read_text())

        live = session.status()
        self.assertTrue(live["running"])
        self.assertFalse(live["attached"])
        self.assertEqual(live["arm"], "right")

    def test_stop_delivers_sigint_to_the_whole_process_group(self) -> None:
        result = session.start("right", "GO", LOOPBACK)
        self.assertTrue(result["ok"], result)
        self.assertTrue(wait_for(lambda: "GRANDCHILD_READY" in self.recorded()),
                        "the grandchild never started")
        self.assertTrue(wait_for(lambda: "typed: GO" in self.recorded()))

        stopped = session.stop()
        self.assertTrue(stopped["ok"], stopped)
        self.assertTrue(stopped["stopped"])
        self.assertEqual(stopped["signal"], "SIGINT")

        # THE ASSERTION THIS FILE EXISTS FOR. The grandchild is not the process
        # whose pid the panel holds; the only way SIGINT reached it is that the
        # signal went to the process GROUP. If this ever fails, a stop that
        # looks successful leaves a controller running.
        self.assertTrue(wait_for(lambda: "GRANDCHILD_SIGINT" in self.recorded()),
                        f"SIGINT never reached the group: {self.recorded()!r}")
        self.assertTrue(wait_for(lambda: "LEADER_SIGINT" in self.recorded()),
                        f"SIGINT never reached the leader: {self.recorded()!r}")

        # And once the fake has gone, the panel stops claiming a session.
        self.assertTrue(wait_for(lambda: not session.status()["running"]),
                        "status still reports a session after it exited")
        self.assertFalse(paths.SESSION_STATE.exists())

    def test_losing_the_panels_end_of_stdin_does_not_stop_the_session(self) -> None:
        # A panel crash closes the panel's fds. If that were the only writer on
        # the child's stdin, the script's bare `read` would see EOF and stop a
        # moving arm because a browser tool died. The child holds a writer of
        # its own, so the read stays blocked.
        result = session.start("right", "GO", LOOPBACK)
        self.assertTrue(result["ok"], result)
        self.assertTrue(wait_for(lambda: "typed: GO" in self.recorded()))

        os.close(session._stdin_write_fd)
        session._stdin_write_fd = None
        time.sleep(0.5)

        self.assertNotIn("SECOND_READ_RETURNED", self.recorded())
        self.assertTrue(session.status()["running"])

    def test_a_second_start_is_refused_while_one_runs(self) -> None:
        first = session.start("right", "GO", LOOPBACK)
        self.assertTrue(first["ok"], first)
        second = session.start("left", "GO", LOOPBACK)
        self.assertFalse(second["ok"])
        self.assertIn("already running", second["error"])
        self.assertEqual(json.loads(paths.SESSION_STATE.read_text())["pid"],
                         first["pid"])

    def test_a_script_that_exits_before_a_session_reports_its_output(self) -> None:
        refusing = self.tmp / "refuses.sh"
        refusing.write_text(
            "#!/usr/bin/env bash\n"
            "echo 'missing binary: build/controller — build it first'\n"
            "exit 1\n"
        )
        refusing.chmod(0o755)
        session.launch_command = (
            lambda arm, *choices: ["bash", str(refusing), arm])

        result = session.start("right", "GO", LOOPBACK)
        self.assertFalse(result["ok"])
        self.assertIn("missing binary", result["output"])
        self.assertFalse(paths.SESSION_STATE.exists())


class ReattachTests(SessionTestCase):
    """A restarted panel must find a running session, not orphan it."""

    def test_status_reattaches_across_a_panel_restart(self) -> None:
        # A stand-in for a session started by an earlier panel process: its
        # own process group, and nothing in this module's memory about it.
        child = subprocess.Popen(["sleep", "30"], start_new_session=True)
        self.addCleanup(child.wait)
        self.addCleanup(child.kill)
        paths.SESSION_STATE.write_text(json.dumps({
            "pid": child.pid,
            "pgid": child.pid,
            "arm": "left",
            "started": "2026-08-11T15:39:20+01:00",
            "session_dir": "/tmp/session_153920",
            "log_path": "/tmp/session_153920/panel_session.log",
        }))
        session._process = None  # this panel did not launch it

        state = session.status()
        self.assertTrue(state["running"])
        self.assertTrue(state["attached"])
        self.assertEqual(state["arm"], "left")
        self.assertEqual(state["pid"], child.pid)
        self.assertEqual(state["session_dir"], "/tmp/session_153920")
        self.assertEqual(state["stop_command"], f"kill -INT -{child.pid}")

    def test_status_clears_the_state_when_the_group_is_gone(self) -> None:
        paths.SESSION_STATE.write_text(json.dumps({
            "pid": dead_pid(),
            "pgid": dead_pid(),
            "arm": "right",
            "started": "2026-08-11T15:39:20+01:00",
            "session_dir": "/tmp/session_153920",
            "log_path": "/tmp/session_153920/panel_session.log",
        }))
        session._process = None

        state = session.status()
        self.assertFalse(state["running"])
        self.assertIsNone(state["pid"])
        self.assertFalse(paths.SESSION_STATE.exists())

    def test_stop_on_a_dead_session_reports_rather_than_raising(self) -> None:
        paths.SESSION_STATE.write_text(json.dumps({
            "pid": dead_pid(), "pgid": dead_pid(), "arm": "right",
            "started": "2026-08-11T15:39:20+01:00",
            "session_dir": None, "log_path": None,
        }))
        session._process = None

        result = session.stop()
        self.assertTrue(result["ok"])
        self.assertFalse(result["stopped"])

    def test_a_missing_state_file_is_not_a_crash(self) -> None:
        paths.SESSION_STATE.write_text("{ not json")
        self.assertFalse(session.status()["running"])


class SignalSeamTests(SessionTestCase):
    """Exactly one signal is sent, it is SIGINT, and it goes to the group."""

    def test_stop_sends_one_sigint_to_the_group_and_nothing_else(self) -> None:
        sent = []
        session.send_signal_to_group = lambda pgid, sig: sent.append((pgid, sig))

        # A real, live process stands in for the session so the liveness check
        # is the production one; the seam is what keeps the signal off it.
        child = subprocess.Popen(["sleep", "30"], start_new_session=True)
        self.addCleanup(child.wait)
        self.addCleanup(child.kill)
        paths.SESSION_STATE.write_text(json.dumps({
            "pid": child.pid, "pgid": child.pid, "arm": "right",
            "started": "2026-08-11T15:39:20+01:00",
            "session_dir": None, "log_path": None,
        }))
        session._process = None

        result = session.stop()
        self.assertTrue(result["stopped"])
        self.assertEqual(sent, [(child.pid, signal.SIGINT)])

        # No escalation: stopping again while it is still alive sends another
        # SIGINT, never a SIGKILL, because killing the controller would leave
        # the arm in servoing mode with nobody commanding it.
        session.stop()
        self.assertEqual([sig for _, sig in sent], [signal.SIGINT, signal.SIGINT])

    def test_stop_takes_no_client_address(self) -> None:
        # Stop works from anywhere by construction: there is no address for it
        # to gate on, so it cannot grow such a gate by accident.
        self.assertEqual(list(inspect.signature(session.stop).parameters), [])


if __name__ == "__main__":
    unittest.main()
