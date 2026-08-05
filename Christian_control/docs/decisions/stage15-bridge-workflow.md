# Decision: Stage 1.5 — bridge workflow cleanup (pipe input, hold-at-startup, orientation transport, launcher)

2026-08-05. Follows `stage1-planner-bridge.md` directly: the first
(aborted) supervised-run attempt exposed the Stage 1 operator workflow —
three terminals, a shell file-descriptor trick, seven hand-typed joint
values, an invisible ordering constraint — as the weakest part of the
system next to a live arm. Full problem statement and goals:
`docs/superpowers/specs/2026-08-05-stage15-bridge-workflow.md`. This
record covers the design decisions; the spec covers acceptance criteria.

## Why stdin died

Stage 1's target input was interactive stdin
(`RunPoseTargetInputFromFd`, `src/Targets.cpp`), fed either by an
operator typing `x y z` or by the bridge through a FIFO the operator
held open by hand (`exec 3>fifo`). Two problems, not one:

1. **Unvalidated path.** Typed stdin lines pass only
   `ParsePoseTarget`'s three-finite-numbers check — no reachability, no
   collision, no path validation. A mistyped coordinate reached the arm
   exactly as fast as a bridge-validated waypoint. Any route to the arm
   that bypasses the bridge's validation is a route worth deleting once
   the bridge exists.
2. **EOF ceremony.** `RunPoseTargetInputFromFd`'s poll loop treats EOF as
   final: once every writer closes, the loop `break`s and never reads
   another target for the rest of the run — no reopen, no retry. A named
   FIFO delivers EOF to its reader the moment its *last* writer closes,
   so if each `planner_bridge` invocation opened and closed the FIFO for
   its own shell redirect, the first run's exit silently killed the
   input thread for every run after it. The only fix available at the
   time was operator ceremony: hold a persistent writer FD open in a
   separate shell for the whole session, and never let it close. That
   ceremony is exactly the kind of step a tired operator skips.

Interactive stdin input is deleted outright — not just deprioritized.
The banner ("type x y z") and the plain stdin wrapper are gone. The
bridge is now the only path that can put a target in front of the
controller (see D1 below), and every one of its outputs has passed
`Waypoints.cpp` validation before it is ever written.

## Why hold-at-startup

Stage 1 drove the arm from wherever it woke up to a compiled coordinate,
`kFixedTargetM`, before anything from the planner had been read. With a
planner now upstream, that ordering is backwards: the planner is meant
to own all motion intent, and "move to a hardcoded point while the
planner hasn't spoken yet" contradicts that on every single run.

`Main.cpp` already measures the takeover pose via FK
(`ee_now`) before entering servoing. The terminal target is now that
measured pose — `target.p_desired = ee_now.position` — so the arm's
first commanded state is "stay where you are." `kFixedTargetM` is
deleted from `Config.h`; there is no compiled coordinate left anywhere
in the controller that can pull the arm anywhere. The CSV preamble key
changed from `fixed_target_m` to `startup_hold = measured` (see
`Main.cpp`'s preamble writer) to say plainly that the hold pose is
read, not chosen. The arm only ever moves again once a validated
waypoint arrives over the pipe — the same arrival/settle/dwell
machinery as before, just with a zero-distance first target instead of
a nonzero compiled one.

## Why orientation is transport-only

`docs/superpowers/specs/2026-08-05-stage15-bridge-workflow.md` (D4) is
explicit about the boundary: the pipe format must be able to *carry* a
full pose end-to-end without the controller *acting on* the rotation
component yet. The reason is joint 6: the spec's non-goals section ties
orientation consumption to Stage 1.6, "gated on j6's config service
being confirmed healthy" — j6's config RPCs are currently wedged (see
memory: joint6-config-service-wedged). Sending the arm a commanded
rotation through a joint whose limit-configuration path cannot currently
be trusted is not a risk worth taking to unblock a transport format.

So the grammar is extended (`x y z` or `x y z qx qy qz qw`, unit
quaternion, xyzw to match the telemetry quat columns, `Targets.cpp`'s
`ParsePoseTarget`), and the bridge can emit it
(`--emit-orientation`, `BridgeMain.cpp`, default off — production output
stays byte-identical to Stage 1 without the flag). But
`config::kAcceptOrientationTargets` (`Config.h`) is compiled `false`,
and while it is false a 7-field line is **rejected outright** with an
explicit "orientation targets disabled" message — never silently
truncated to a position. A silent truncation would hide a
misconfiguration behind apparently-normal position tracking; a loud
rejection cannot.

## Two independent consumption guards

Even with the parser accepting 7-field lines, two separate mechanisms
stand between a parsed rotation and the arm, deliberately not just one:

1. **The compile-time flag.** `config::kAcceptOrientationTargets = false`
   gates parsing itself — a 7-field line never becomes a `PoseTarget`
   with a populated `rotation` field while it is false.
2. **`PoseTargetSource`'s pre-existing `rotation.reset()` calls**
   (`Targets.cpp`). Even if a rotation ever reached this class, these
   calls clear it before it can reach the reference — a guard that
   predates this branch and was not touched by it.

Two guards mean a single mistake — flipping the compile flag without
also removing the reset calls, or vice versa — cannot alone put
orientation control live on the arm. Stage 1.6 must deliberately unwind
both.

## The pipe reopen design (D1)

`config::kTargetPipePath = "/tmp/humansl_bridge_targets"`. The
*controller* creates the pipe at startup now, not the operator
(`mkfifo`, 0600, `Main.cpp`) — it errors out if the path exists and is
not a FIFO, or if `mkfifo` itself fails, rather than starting against an
unknown path. `RunPoseTargetInputFromPipe` (`Targets.h/.cpp`) wraps the
unchanged, already-tested `RunPoseTargetInputFromFd`: it opens the pipe
`O_RDONLY | O_NONBLOCK`, delegates to `RunPoseTargetInputFromFd`, and on
EOF closes and reopens rather than exiting — bounded by the existing
`kInputPollTimeoutMs` cadence between reopen attempts. `stop` remains
the only way out, so teardown join stays prompt. This makes the Stage 1
failure mode structurally impossible rather than operator-avoidable:
writers (the bridge) simply open, write their lines, and close per
invocation — no persistent FD, no `exec 3>`, because the *reader* now
tolerates a writer closing.

## The launcher's role: sequencing, not authority

`Christian_control/planner_bridge/scripts/run_session.sh` exists to
remove the invisible ordering constraint (controller must be up and
logging before the bridge can find a state CSV; the pipe must exist
before `goal` can write to it), not to make any safety decision itself.
Per the project's standing hardware-safety rule (root `CLAUDE.md`),
authorization for a run is Christian's, given in person, for that
specific run — the script cannot substitute for that. Concretely:

- It refuses to proceed on stale binaries (built-artifact older than any
  source file) unless `--allow-stale` is passed explicitly.
- It prints the supervised-run checklist (presence, workspace clear,
  e-stop in reach, dashboard closed) and requires the operator to type
  `GO` — a pause the script enforces, not a decision it makes.
- `--dry-run` performs every step above and stops before starting the
  controller, so the gates themselves can be exercised with no
  hardware involved.
- ~~Its `goal X Y Z [box ...]` REPL is a thin wrapper~~ **Superseded
  2026-08-05, same day:** the REPL is gone. The goal now lives in
  `planner_bridge/config/goal.yaml`, edited before the session; the
  script waits for the controller's run log, runs the bridge exactly
  once with no `--goal` (the bridge reads the file itself via the
  `--goal-file` default in `BridgeMain.cpp`), and waits for Enter to
  tear down. Typing coordinates at a prompt next to a live arm was the
  REPL's whole interface, and declaring them in a reviewable file while
  the arm is powered down is both simpler and safer. It bypasses no
  validation the bridge or controller already perform, and the
  trap-based teardown still stops the controller on every exit path.

If the script is skipped entirely and every step is done by hand, the
underlying safety properties (validated-only targets, hold-at-startup,
the reopening pipe) are unchanged — the script only removes the manual
choreography that made Stage 1's first run attempt error-prone.
