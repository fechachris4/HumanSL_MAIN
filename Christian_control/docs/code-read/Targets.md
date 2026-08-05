# Targets.cpp / Targets.h — line-by-line read

*(Updated for current-pose startup. Targets remain position-only and the
source now profiles from the measured takeover position to the terminal
target. The source never talks to the robot or performs control math.)*

Targets provides `RotationFromRpy` (the rpy→matrix convention), target input,
and `PoseTargetSource` (the `ReferenceSource` that profiles from the measured
startup pose to the terminal target). The header's first line still states
the boundary: never talks to the robot, does no control math.

Execution order in a run:

1. Main measures the current FK position from the fresh actuator feedback.
2. Main builds the terminal target from the measured startup pose itself
   (Stage 1.5: `kFixedTargetM` is deleted, no compiled terminal target
   remains) and constructs
   `PoseTargetSource(start_position, terminal_target, mailbox)`.
3. Inside the loop: `Reset(state)` once at takeover (Runner.cpp:164), then
   `Get(state, dt, status)` every cycle (Runner.cpp:214).

---

## `PoseTarget` (Targets.h:15-20)

One target: a position (meters, right-arm base frame) and an
`std::optional<Eigen::Matrix3d>` rotation. First-time note:
`std::optional<T>` is a box that either holds a `T` or holds nothing
(`std::nullopt`) — and here "nothing" *means something*: an empty rotation
is "no orientation requested; keep the takeover orientation". Encoding
"keep" as absence instead of a magic value is the pattern this codebase
prefers.

## `RotationFromRpy` (Targets.cpp:9-17)

```cpp
return (Eigen::AngleAxisd(yaw, UnitZ()) *
        Eigen::AngleAxisd(pitch, UnitY()) *
        Eigen::AngleAxisd(roll, UnitX())).toRotationMatrix();
```

- **What**: build a rotation matrix from roll/pitch/yaw radians, composed
  R = Rz(yaw)·Ry(pitch)·Rx(roll). `Eigen::AngleAxisd` is "rotate by this
  angle about this axis"; multiplying them composes right-to-left, so the
  roll about X is applied first.
- **Why it exists**: the header (Targets.h:22-25) calls it "the one place
  that convention is written down" — it mirrors the Python sim's
  `rotation_from_rpy`, so a compiled `kFixedTargetRpyRad` triple and the
  sim mean the same thing. Main's startup print decomposes the measured
  orientation with `eulerAngles(2,1,0)` (Main.cpp:190) to match, so the
  printed rpy can be pasted straight into Config.h.
- **If changed**: swapping the multiplication order silently changes every
  orientation target *and* desynchronizes from the Python sim and from the
  startup printout the operator copies numbers from. **FLAG
  `Targets.cpp:9-17` | edit-hazard** — call sites in two languages plus
  the startup print depend on this exact composition order.

## `PoseTargetSource` (Targets.h:28-49, Targets.cpp:23-46)

The bridge from the measured startup position to the terminal target. Inherits
`ReferenceSource` (State.h:90-103); the loop only ever sees the base-class
interface. The class comment (Targets.h:28-36) carries the contract:
The initial profile starts at the measured takeover position and ends at the
terminal target; empty rotation keeps the takeover orientation.

### Constructor (Targets.cpp:23-26)

Validates and stores the measured startup position, terminal target, profile
limits, and hold duration, then creates the initial Cartesian segment profile.
The first profile sample is the measured startup position with zero velocity;
later samples advance toward the terminal target.

### `Reset` (Targets.cpp:28-30) — called once at takeover (T5)

Empty body. The override must exist because `Reset` is *pure virtual* in
`ReferenceSource` (a function the base class declares with `= 0`, obligating
every concrete source to provide it). The measured startup pose is captured by
`TrackingController::Reset`; the target source receives its FK position from
`Main.cpp` before the loop begins.

### `Get` (Targets.cpp:32-46) — called every cycle

- **Initial profile** — sequence 0 starts at the measured takeover position,
  supplies the seventh-order position and velocity reference, and becomes
  arrival-eligible only at the terminal sample.
- **Queued profiles** — after the arrival edge and dwell, each queued target
  starts at the previous terminal target and receives the next sequence number.

### What Get never does

No I/O, no allocation, no blocking — the `ReferenceSource` contract
(State.h:88-89) that keeps the 500 Hz loop honest. With the store's mutex
gone, `Get` is now trivially lock-free.

---

## What was removed (for orientation when reading old logs/commits)

Gone in f64325c0 and earlier 2026-08-04 work, recoverable from git
history: the original `ParsePoseTarget` (typed-line parsing),
`PoseTargetStore` (the mutex-guarded shared mailbox), `RunPoseTargetInput`
(the stdin poll/getline thread), the store plumbing inside
`PoseTargetSource` (baseline sequence, store snapshot branch), and —
earlier — the watched-target-file input. Runs recorded before the
removal may reference these in their console transcripts.

**Current (Stage 1.5, 2026-08-05):** `ParsePoseTarget`, the
`PoseTargetMailbox` SPSC queue, and a stdin-style reader are all back,
in a different shape — `RunPoseTargetInputFromFd` (unchanged tested
core) plus a new `RunPoseTargetInputFromPipe` wrapper that reopens the
named pipe `config::kTargetPipePath` on writer EOF instead of exiting.
There is no bare `RunPoseTargetInput` (stdin-only, no-reopen) in the
current source — see
`../decisions/stage15-bridge-workflow.md`.
