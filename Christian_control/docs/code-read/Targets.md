# Targets.cpp / Targets.h — line-by-line read

*(Updated for commit f64325c0, "remove trajectory playback: fixed-target-
only controller". The stdin input thread, `ParsePoseTarget`, and the whole
`PoseTargetStore` are gone — the earlier "slated for removal" set from this
doc's first edition has been carried out. What remains is small: one
rotation helper and one fixed-target reference source. Targets.h is now 49
lines, Targets.cpp 46.)*

Targets is now exactly two things: `RotationFromRpy` (the rpy→matrix
convention) and `PoseTargetSource` (the `ReferenceSource` that serves the
compiled fixed target every cycle). The header's first line still states
the boundary: never talks to the robot, does no control math.

Execution order in a run:

1. Main builds the compiled fixed target (Main.cpp:365-375), calling
   `RotationFromRpy` at Main.cpp:372 if `kFixedTargetUseRpy` is set.
2. `reference = std::make_unique<PoseTargetSource>(fixed_target);` —
   Main.cpp:376.
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

The bridge from the compiled target to the controller. Inherits
`ReferenceSource` (State.h:90-103); the loop only ever sees the base-class
interface. The class comment (Targets.h:28-36) carries the contract:
`initial_target` applies from the first cycle; empty rotation keeps the
takeover orientation; with no target at all the source returns an empty
reference and the controller holds where takeover happened.

### Constructor (Targets.cpp:23-26)

Moves the `std::optional<PoseTarget>` into the member. The default
argument (Targets.h:41) is `std::nullopt`, so a source with no target is
constructible — that is the "hold at takeover" configuration. Main always
passes a target today (Main.cpp:376, after the freshness gate at
Main.cpp:335-364 has verified the compiled target is within
`kMaxFixedTargetDistanceM` of the measured pose).

### `Reset` (Targets.cpp:28-30) — called once at takeover (T5)

Empty body. The override must exist because `Reset` is *pure virtual* in
`ReferenceSource` (a function the base class declares with `= 0`,
obligating every concrete source to provide it), but a fixed target has no
baseline to capture — the store-sequence snapshot that used to live here
went with the store. Harmless three lines; the interface slot stays
because a future source (Vicon) will need it.

### `Get` (Targets.cpp:32-46) — called every cycle

- **Line 35** — start from an empty `Reference`.
- **Lines 36-39 (comment)** — targets here are **stationary**: the
  reference `Twist{}` stays zero so the controller's Kd term is pure
  damping. A future source that *moves* its target must fill the twist or
  the damping term fights the motion it commands. Note the prose still
  opens "Operator targets are STATIONARY: a typed or compiled pose..." —
  "typed" is stale since the stdin thread's removal; compiled is the only
  kind left. Stale words only, not behaviour.
- **Lines 40-44** — if an initial target exists, serve it as a
  `PoseReference` with **sequence 0** — every cycle, the same target, the
  same sequence. The controller's arrival notice is edge-triggered per
  *sequence*, so the constant 0 makes "target reached" print exactly once
  per run. **FLAG `Targets.cpp:40-44` | edit-hazard** — the hard-coded
  sequence 0 looks arbitrary but is what arms the one-shot arrival
  detection; "improving" it to increment per cycle would re-fire the
  arrival edge every cycle, and giving two different fixed targets the
  same sequence in some future edit would suppress the second's notice.
- **Line 45** — no target → empty reference → the controller holds the
  takeover pose (State.h's "unset means no reference").

### What Get never does

No I/O, no allocation, no blocking — the `ReferenceSource` contract
(State.h:88-89) that keeps the 500 Hz loop honest. With the store's mutex
gone, `Get` is now trivially lock-free.

---

## What was removed (for orientation when reading old logs/commits)

Gone in f64325c0 and earlier 2026-08-04 work, recoverable from git
history: `ParsePoseTarget` (typed-line parsing), `PoseTargetStore` (the
mutex-guarded shared mailbox), `RunPoseTargetInput` (the stdin poll/getline
thread), the store plumbing inside `PoseTargetSource` (baseline sequence,
store snapshot branch), and — earlier — the watched-target-file input.
Runs recorded before the removal may reference these in their console
transcripts.
