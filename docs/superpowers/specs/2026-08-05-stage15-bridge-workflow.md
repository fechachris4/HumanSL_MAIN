# Stage 1.5 — Bridge workflow cleanup (spec)

Decided with Christian 2026-08-05 after the first (aborted) supervised-run
attempt exposed the operator workflow as the weakest part of Stage 1.

## Problem

Running the Stage 1 bridge today takes three terminals, a shell
file-descriptor trick (`exec 3>fifo`), seven hand-typed joint values, and
an invisible ordering constraint. Each is an operator error waiting to
happen next to a live arm. Root causes, not symptoms:

1. The controller reads targets from **stdin**, and its input thread exits
   permanently on EOF — that is why the FIFO ceremony exists. The typed
   path is also the only unvalidated route to the arm (three-finite-numbers
   check only).
2. The controller drives to a **compiled terminal target** (`kFixedTargetM`)
   from wherever it wakes up. With a planner upstream, "move to a hardcoded
   point before the planner gets a say" is backwards.
3. The bridge's automatic state source (the controller's run CSV) must be
   located and passed **by hand**; the fallback is typing seven joint
   angles, which the bridge will confidently plan from even if mistyped.
4. The planner solves for **full pose** but the pipeline throws the
   orientation away; the controller freezes the takeover orientation.
5. Nothing checks that the binaries on disk match the sources — the first
   run attempt nearly executed a 3-hour-stale control law.

## Goals

- One-command session start; no FIFO ceremony, no hand-typed state.
- Every target that can reach the arm has passed the bridge's validation.
- The arm never moves at startup: it holds its measured pose until the
  first validated waypoint arrives.
- The pipe format can carry orientation end-to-end, WITHOUT enabling
  orientation tracking (that is Stage 1.6, gated on j6's config service
  being confirmed healthy).

## Non-goals

- Orientation *consumption* (arm actually rotating to commanded
  orientations) — explicitly out of scope; transport only.
- Automatic replanning / deviation triggers — Stage 2.
- Any change to the reactive law, safety guards, or actuation path.

## Design

### D1 — Named-pipe target input; interactive stdin deleted

- New `config::kTargetPipePath = "/tmp/humansl_bridge_targets"`.
- New `RunPoseTargetInputFromPipe(mailbox, stop, pipe_path)`: an outer
  loop that opens the pipe `O_RDONLY | O_NONBLOCK`, delegates to the
  existing, tested `RunPoseTargetInputFromFd`, and on EOF (writer closed)
  closes and reopens. A short sleep (the existing `kInputPollTimeoutMs`
  cadence) between reopen attempts bounds the no-writer spin. `stop`
  remains the only exit, so teardown join stays prompt.
- `Main.cpp` creates the pipe if missing (`mkfifo`, error out on failure
  or non-FIFO existing path) and spawns the pipe thread. The stdin wrapper
  `RunPoseTargetInput` and the "type x y z" banner are DELETED. Writers
  (the bridge) simply open, write lines, close — no persistent fd, no
  `exec 3>`.
- `RunPoseTargetInputFromFd` itself is unchanged (still the tested core;
  still used by tests over pipes/fds).

### D2 — Hold measured pose at startup; compiled target deleted

- `Main.cpp` already measures the takeover pose (`ee_now`). The terminal
  target becomes that measured position: `target.p_desired =
  ee_now.position`. `kFixedTargetM` is deleted from `Config.h`; the CSV
  preamble's `fixed_target_m` key is replaced by
  `startup_hold = measured` (the measured values are already in the
  `startup_position_m` preamble line). Arrival at a zero-distance target
  follows the existing settle/dwell machinery unchanged, after which the
  source is ready for queued waypoints — same phases, degenerate profile.
- Consequence: from launch to first bridge waypoint, the arm holds. No
  compiled coordinates anywhere.

### D3 — Bridge finds its own start state

- `FindLatestRunCsv(runs_root)`: newest `loop_log*.csv` under
  `<runs_root>/<date>/`, by file modification time.
- Bridge default when neither `--state-csv` nor `--start-deg` is given:
  auto-discover under the repo's `runs/` (resolved from `/proc/self/exe`
  like the other defaults; `--runs-root` overrides). Clear exit-2 message
  when nothing is found ("start the controller first — it creates the
  log").
- `--start-deg` survives for tests and bench work but the help text marks
  it test-only.

### D4 — Orientation transport (parse + emit, consumption off)

- Target line grammar extended: `x y z` (unchanged) or
  `x y z qx qy qz qw` (unit quaternion, xyzw order to match the telemetry
  quat columns, |norm−1| ≤ 1e-3).
- New `config::kAcceptOrientationTargets = false`. While false, a 7-field
  line is REJECTED with an explicit message ("orientation targets
  disabled") — never silently truncated to a position, so a
  misconfiguration is loud. When later enabled, the parsed rotation flows
  into `PoseTarget::rotation` (the field already exists) and from there to
  the reference; that wiring is Stage 1.6 and stays dead code behind the
  flag for now.
- Bridge: `--emit-orientation` (default off) appends the waypoint's FK
  quaternion to each line. Default off means production behavior is
  byte-identical to Stage 1.

### D5 — Session launcher

- `Christian_control/planner_bridge/scripts/run_session.sh`, one
  terminal:
  1. Freshness gate: refuse if `controller` is older than any file in
     `basic_control/src/`, or `planner_bridge` older than its sources
     (both overridable with `--allow-stale` for deliberate reruns).
  2. Prints the compiled guard flags (from the binary's `--help`/banner)
     and the standing checklist; requires typing `GO` (authorization
     remains Christian's, per project CLAUDE.md — the script only
     enforces the pause).
  3. Starts the controller (output visible), waits for its CSV to appear,
     then offers a prompt: `goal X Y Z [box CX CY CZ HX HY HZ]` runs the
     bridge with auto-state; `quit` tears down (controller first, then
     removes nothing — logs are evidence).
- The script never bypasses any gate; it only sequences what the operator
  did by hand.

## Safety analysis (Level 2 items: D1, D2)

- **Frames/units:** unchanged everywhere — metres, base_link, Kortex
  actuator order, radians internal / degrees at CSV boundary; quaternion
  xyzw in base_link (matches telemetry columns).
- **Startup:** D2 strictly reduces startup motion (hold instead of
  travel). The takeover hold path and settle/dwell logic are untouched.
- **Failure paths:** pipe writer crash mid-line → `FromFd` EOF handling
  processes/drops exactly as today, then reopens; no writer → input idle,
  arm holds; malformed/oversized lines → existing rejection paths.
  Teardown → `stop` flag exits both loops; joiner unchanged.
- **What could regress:** D1 touches the input thread lifecycle (reopen
  loop) — covered by hardware-free tests driving a real FIFO through
  write/close/rewrite cycles plus prompt-join-on-stop; D2 touches one
  assignment plus deletions — covered by the existing source-phase tests.
- **Telemetry:** preamble key change (`fixed_target_m` →
  `startup_hold`) noted for the plot scripts; log_format number
  unchanged (no column changes).

## Acceptance

1. `ctest` (bridge) and basic_control test suites green.
2. Offline: `run_session.sh` refuses on stale binaries; with `--dry-run`
   it performs every step except starting the controller.
3. Hardware-free pipe test proves two sequential writer sessions both
   deliver targets (the Stage 1 EOF failure mode is dead).
4. A 7-field target line is rejected while `kAcceptOrientationTargets`
   is false; bridge `--emit-orientation` output parses when the flag is
   conceptually on (parser unit test), and default output is unchanged.
5. Supervised hardware validation of D1+D2 (arm holds at startup; targets
   arrive over the pipe across two bridge invocations) — PENDING explicit
   authorization, reported as such.
